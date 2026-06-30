"""Tests for the core control primitives in `so101_control.control`.

All tests run in dry-run mode (no hardware, no serial port). The `lerobot`
import is required because `so101_control.control` imports
`RobotKinematics` / `SO101Follower` at module load time.
"""

import pytest

import so101_control
from so101_control.control import (
    ACTION_KEYS,
    ALL_JOINTS,
    ARM_JOINTS,
    HOME_JOINTS,
    REST_JOINTS,
    home,
    move_to_joints,
    rest,
)


# ── Package / API surface ────────────────────────────────────────────────────
def test_version_is_string():
    assert isinstance(so101_control.__version__, str)
    # PEP 440-ish: at least one dot.
    assert "." in so101_control.__version__


def test_public_api_reexports():
    """Every name in __all__ must actually be importable from the package."""
    for name in so101_control.__all__:
        assert hasattr(so101_control, name), f"__all__ lists {name!r} but it is missing"


def test_joint_schema_consistency():
    assert ARM_JOINTS == [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    ]
    assert ALL_JOINTS == ARM_JOINTS + ["gripper"]
    assert ACTION_KEYS == [f"{j}.pos" for j in ALL_JOINTS]


def test_home_joints_keys_match_action_keys():
    assert set(HOME_JOINTS) == set(ACTION_KEYS)


def test_rest_joints_keys_match_action_keys():
    assert set(REST_JOINTS) == set(ACTION_KEYS)


# ── move_to_joints (dry-run) ─────────────────────────────────────────────────
def test_move_to_joints_dry_run_prints_trajectory(capsys):
    """Dry-run should print the planned trajectory and not touch hardware."""
    target = {"shoulder_pan.pos": 10.0, "gripper.pos": 50.0}
    move_to_joints(None, target, duration_s=1.0, fps=10, dry_run=True)

    out = capsys.readouterr().out
    assert "[dry-run] move_to_joints" in out
    assert "steps: 10 @ 10 Hz over 1.00s" in out
    # The target value should appear in the "to:" line.
    assert "shoulder_pan.pos=10.00" in out


def test_move_to_joints_dry_run_fills_missing_keys_from_home(capsys):
    """Keys absent from `target` should default to the current (HOME) value."""
    target = {"shoulder_pan.pos": 5.0}  # only one of six keys
    move_to_joints(None, target, duration_s=0.1, fps=2, dry_run=True)

    out = capsys.readouterr().out
    # Every action key must appear in the "to:" line.
    for key in ACTION_KEYS:
        assert key in out


def test_move_to_joints_dry_run_uses_explicit_current(capsys):
    """When `current` is passed, it seeds the trajectory start."""
    current = {k: 0.0 for k in ACTION_KEYS}
    target = {k: 100.0 for k in ACTION_KEYS}
    move_to_joints(None, target, current=current, duration_s=0.1, fps=2, dry_run=True)

    out = capsys.readouterr().out
    assert "from:" in out and "to:" in out
    # Start values should reflect `current` (0.00), end values `target` (100.00).
    assert "shoulder_pan.pos=0.00" in out
    assert "shoulder_pan.pos=100.00" in out


def test_move_to_joints_dry_run_min_duration_one_step(capsys):
    """A zero-duration move should still produce at least one step."""
    move_to_joints(None, HOME_JOINTS, duration_s=0.0, fps=30, dry_run=True)
    out = capsys.readouterr().out
    assert "steps: 1 @ 30 Hz" in out


# ── home / rest (dry-run) ────────────────────────────────────────────────────
def test_home_dry_run_targets_home_joints(capsys):
    home(None, duration_s=0.1, fps=2, dry_run=True)
    out = capsys.readouterr().out
    # The "to:" line must show HOME_JOINTS values.
    assert f"shoulder_pan.pos={HOME_JOINTS['shoulder_pan.pos']:.2f}" in out
    assert f"gripper.pos={HOME_JOINTS['gripper.pos']:.2f}" in out


def test_rest_dry_run_targets_rest_joints(capsys):
    rest(None, duration_s=0.1, fps=2, dry_run=True)
    out = capsys.readouterr().out
    assert f"shoulder_pan.pos={REST_JOINTS['shoulder_pan.pos']:.2f}" in out
    assert f"wrist_roll.pos={REST_JOINTS['wrist_roll.pos']:.2f}" in out


def test_home_and_rest_targets_differ(capsys):
    """HOME and REST are distinct poses — sanity check the constants."""
    assert HOME_JOINTS != REST_JOINTS
