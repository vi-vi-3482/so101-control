"""Tests for the CLI argument parsers in `so101_control.cli`.

These are pure functions with no hardware or lerobot dependency beyond the
`so101_control.control` constants they validate against.
"""

import pytest

from so101_control.cli import _parse_rpy, _parse_target_joints, _parse_xyz
from so101_control.control import ACTION_KEYS


# ── _parse_target_joints ─────────────────────────────────────────────────────
def test_parse_target_joints_basic():
    out = _parse_target_joints("shoulder_pan=0,shoulder_lift=-20,elbow_flex=60")
    assert out == {
        "shoulder_pan.pos": 0.0,
        "shoulder_lift.pos": -20.0,
        "elbow_flex.pos": 60.0,
    }


def test_parse_target_joints_accepts_pos_suffix():
    """Keys may already carry the `.pos` suffix."""
    out = _parse_target_joints("gripper.pos=50")
    assert out == {"gripper.pos": 50.0}


def test_parse_target_joints_strips_whitespace():
    out = _parse_target_joints("  shoulder_pan = 10 , wrist_roll = -5  ")
    assert out == {"shoulder_pan.pos": 10.0, "wrist_roll.pos": -5.0}


def test_parse_target_joints_all_keys():
    """Every action key should be parseable in one shot."""
    s = ",".join(f"{k.removesuffix('.pos')}={i}" for i, k in enumerate(ACTION_KEYS))
    out = _parse_target_joints(s)
    assert set(out) == set(ACTION_KEYS)
    assert all(isinstance(v, float) for v in out.values())


def test_parse_target_joints_rejects_missing_equals():
    with pytest.raises(ValueError, match="expected name=value"):
        _parse_target_joints("shoulder_pan")


def test_parse_target_joints_rejects_unknown_joint():
    with pytest.raises(ValueError, match="unknown joint"):
        _parse_target_joints("not_a_joint=1.0")


def test_parse_target_joints_rejects_non_numeric():
    with pytest.raises(ValueError):
        _parse_target_joints("shoulder_pan=abc")


def test_parse_target_joints_empty_string_yields_empty():
    assert _parse_target_joints("") == {}


# ── _parse_xyz ────────────────────────────────────────────────────────────────
def test_parse_xyz_basic():
    assert _parse_xyz("0.2,0.0,0.15") == [0.2, 0.0, 0.15]


def test_parse_xyz_negative():
    assert _parse_xyz("-1.0,-2.0,-3.0") == [-1.0, -2.0, -3.0]


def test_parse_xyz_rejects_two_values():
    with pytest.raises(ValueError, match="must be x,y,z"):
        _parse_xyz("1.0,2.0")


def test_parse_xyz_rejects_four_values():
    with pytest.raises(ValueError, match="must be x,y,z"):
        _parse_xyz("1.0,2.0,3.0,4.0")


def test_parse_xyz_rejects_non_numeric():
    with pytest.raises(ValueError):
        _parse_xyz("a,b,c")


# ── _parse_rpy ────────────────────────────────────────────────────────────────
def test_parse_rpy_basic():
    assert _parse_rpy("10,20,30") == [10.0, 20.0, 30.0]


def test_parse_rpy_negative():
    assert _parse_rpy("-90,0,-45") == [-90.0, 0.0, -45.0]


def test_parse_rpy_rejects_two_values():
    with pytest.raises(ValueError, match="must be roll,pitch,yaw"):
        _parse_rpy("1,2")


def test_parse_rpy_rejects_non_numeric():
    with pytest.raises(ValueError):
        _parse_rpy("x,y,z")
