#!/usr/bin/env python
"""Programmatic control of an SO-101 follower arm.

This module is part of the `so101_control` package (installed as
`so101-control`). It exposes three reusable primitives, all runnable without
hardware via `dry_run=True`:

    move_to_joints(robot, target)        # joint-angle waypoint move
    go_to_ee(kinematics, robot, xyz)     # inverse-kinematics Cartesian move
    home(robot)                          # return to HOME_JOINTS

The package also provides a `so101-control` command-line entry point (see
`so101_control.cli`) that wraps these primitives.

Examples
--------
Library usage (after `uv add so101-control` or `uv sync` from a checkout):

    from so101_control import go_to_ee, home, move_to_joints
    move_to_joints(robot, {"shoulder_pan.pos": 0.0, ...})

CLI dry-run (no hardware, prints the planned trajectory + FK of each waypoint):

    so101-control --mode ee --target-xyz 0.2,0.0,0.15 --dry-run

CLI live joint move:

    so101-control \
        --port /dev/tty.usbmodem585A0076841 \
        --robot-id my_awesome_follower_arm \
        --mode joint \
        --target-joints shoulder_pan=0,shoulder_lift=-20,elbow_flex=60,wrist_flex=30,wrist_roll=0,gripper=50

CLI live end-effector move (position-only IK by default):

    so101-control \
        --port /dev/tty.usbmodem585A0076841 \
        --robot-id my_awesome_follower_arm \
        --urdf-path /absolute/path/to/so101_new_calib.urdf \
        --mode ee \
        --target-xyz 0.2,0.0,0.15

Prerequisites
-------------
* `lerobot` (with the `kinematics` extra for IK/EE mode) installed in the
  active environment. The `so101-control` package already declares
  `lerobot[kinematics,feetech]` as a dependency, so installing the package
  (e.g. `uv sync` from a source checkout, or `uv add so101-control` in your
  own project) is enough.
* The SO-101 URDF + mesh assets (only needed for `--mode ee` / `go_to_ee`).
  Download both into a directory of your choice and pass the *absolute* path to
  the URDF (the path must be absolute so the sibling `assets/` folder can be
  located):
      curl -L -o so101_new_calib.urdf \
        https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/so101_new_calib.urdf
      mkdir -p assets && cd assets
      for f in base_motor_holder_so101_v1 base_so101_v2 motor_holder_so101_base_v1 \
        motor_holder_so101_wrist_v1 moving_jaw_so101_v1 rotation_pitch_so101_v1 \
        sts3215_03a_no_horn_v1 sts3215_03a_v1 under_arm_so101_v1 upper_arm_so101_v1 \
        waveshare_mounting_plate_so101_v2 wrist_roll_follower_so101_v1 \
        wrist_roll_pitch_so101_v2; do
        curl -sL -o "$f.stl" \
          "https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/assets/$f.stl"
      done
      cd ..
* A calibrated follower. Calibration produced by `lerobot-record` (or
  `lerobot-calibrate`) lives under
  `~/.cache/huggingface/lerobot/calibration/robots/<robot-id>/`. The
  `--robot-id` passed to the CLI must match the id used during calibration.
  See `so101_commands.md` for copy-pasteable calibration commands.

Notes
-----
* SO-101 has 5 arm DOF + 1 gripper. IK is therefore solved on the 5 arm joints
  only; the gripper value is passed through unchanged (this is handled
  automatically by `RobotKinematics.inverse_kinematics`).
* IK defaults to position-only (orientation_weight=0.0). Pass --target-rpy
  (roll,pitch,yaw in degrees) to add a soft orientation constraint.
* `--max-relative-target` (deg) clips per-step jumps inside `send_action` for
  safety. Strongly recommended when running on hardware.
"""

from __future__ import annotations

import time

import numpy as np

# lerobot APIs (must be importable on sys.path)
from lerobot.model.kinematics import RobotKinematics


# TODO: remove this inline helper if a future lerobot release (>=0.5.2) re-exports
# `follower_smooth_move_to` from `lerobot.common.control_utils`. As of lerobot
# 0.5.1 on PyPI, that submodule is absent, so we inline the function (ported
# verbatim from the lerobot repo's src/lerobot/common/control_utils.py).
def follower_smooth_move_to(
    robot, current: dict, target: dict, duration_s: float = 1.0, fps: int = 30
) -> None:
    """Smoothly move the follower robot from `current` to `target` action."""
    steps = max(int(duration_s * fps), 1)
    for step in range(steps + 1):
        t = step / steps
        interp = {
            k: current[k] * (1 - t) + target[k] * t if k in target else current[k]
            for k in current
        }
        robot.send_action(interp)
        time.sleep(1 / fps)


# ── Joint schema ──────────────────────────────────────────────────────────────
# Order matches robot.bus.motors and the action/observation key space.
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ALL_JOINTS = ARM_JOINTS + ["gripper"]
ACTION_KEYS = [f"{j}.pos" for j in ALL_JOINTS]

# Target frame name in the SO-ARM100 URDF.
EE_FRAME = "gripper_frame_link"

# ── Poses ─────────────────────────────────────────────────────────────────────
# The midpoint of every joint's calibrated range. For arm joints (DEGREES norm
# mode) the calibration midpoint maps to 0 degrees; for the gripper
# (RANGE_0_100 norm mode) it maps to 50. See lerobot's MotorsBusBase._normalize
# (motors_bus.py) for the mapping.
HOME_JOINTS = {
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": 0.0,
    "elbow_flex.pos": 0.0,
    "wrist_flex.pos": 0.0,
    "wrist_roll.pos": -90.0,
    "gripper.pos": 50.0,
}

REST_JOINTS = {  # TODO manually find the correct poses for this
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": -95.0,
    "elbow_flex.pos": 95.0,
    "wrist_flex.pos": 85.0,
    "wrist_roll.pos": -90.0,
    "gripper.pos": 50.0,
}


# ── Helpers ───────────────────────────────────────────────────────────────────
def _joints_dict_from_array(arr: np.ndarray) -> dict[str, float]:
    """Map a length-6 joint array (5 arm + gripper) onto `*.pos` action keys."""
    return {key: float(val) for key, val in zip(ACTION_KEYS, arr, strict=True)}


def _joints_array_from_dict(d: dict[str, float]) -> np.ndarray:
    """Inverse of `_joints_dict_from_array`. Missing keys default to HOME."""
    return np.array([float(d.get(k, HOME_JOINTS[k])) for k in ACTION_KEYS], dtype=float)


def _rpy_deg_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Roll-Pitch-Yaw (degrees, XYZ extrinsic) -> 3x3 rotation matrix."""
    r, p, y = np.deg2rad([roll, pitch, yaw])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _pose_from_xyz_rpy(xyz: list[float], rpy_deg: list[float] | None) -> np.ndarray:
    """Build a 4x4 SE(3) pose from xyz and optional roll-pitch-yaw (degrees)."""
    T = np.eye(4)
    T[:3, 3] = xyz
    if rpy_deg is not None:
        T[:3, :3] = _rpy_deg_to_rotmat(*rpy_deg)
    return T


def _rotmat_to_rpy_deg(R: np.ndarray) -> list[float]:
    """3x3 rotation matrix -> Roll-Pitch-Yaw (degrees, XYZ extrinsic)."""
    pitch = np.degrees(np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0])))
    roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return [float(roll), float(pitch), float(yaw)]


def current_joints(robot) -> dict[str, float]:
    """Read the current joint angles from the robot (deg for arm, 0-100 gripper)."""
    if robot is None:
        return dict(HOME_JOINTS)
    obs = robot.get_observation()
    return {k: float(obs.get(k, HOME_JOINTS[k])) for k in ACTION_KEYS}


def current_ee_pose(kinematics: RobotKinematics, robot) -> np.ndarray:
    """Return the current end-effector pose from encoder readings + FK."""
    joints = current_joints(robot)
    arm = np.array([joints[f"{j}.pos"] for j in ARM_JOINTS], dtype=float)
    return kinematics.forward_kinematics(arm)


def print_status(kinematics: RobotKinematics | None, robot) -> None:
    """Print current joint angles and, when available, the FK end-effector pose."""
    joints = current_joints(robot)

    print("Current joints (deg for arm, 0-100 for gripper):")
    for key in ACTION_KEYS:
        print(f"  {key:24s} = {joints[key]:8.2f}")

    if kinematics is None:
        print("EE pose unavailable: no URDF/kinematics loaded.")
        return

    T = current_ee_pose(kinematics, robot)
    xyz = T[:3, 3]
    rpy = _rotmat_to_rpy_deg(T[:3, :3])

    print("Current end-effector pose from FK:")
    print(f"  xyz (m)   = {xyz.round(4).tolist()}")
    print(f"  rpy (deg) = {[round(v, 2) for v in rpy]}")


# ── Core primitives ───────────────────────────────────────────────────────────
def move_to_joints(
    robot,
    target: dict[str, float],
    *,
    current: dict[str, float] | None = None,
    duration_s: float = 2.0,
    fps: int = 30,
    dry_run: bool = False,
) -> None:
    """Smoothly move the follower to `target` (a `*.pos` -> value dict).

    Linearly interpolates between `current` and `target` and calls
    `robot.send_action` at `fps` Hz for `duration_s` seconds. In dry-run mode
    nothing is sent to hardware; the planned trajectory is printed instead.
    """
    if current is None:
        if dry_run or robot is None:
            current = dict(HOME_JOINTS)
        else:
            current = robot.get_observation()

    # Ensure every expected key is present.
    cur = {k: float(current.get(k, HOME_JOINTS[k])) for k in ACTION_KEYS}  # type: ignore[union-attr]
    tgt = {k: float(target.get(k, cur[k])) for k in ACTION_KEYS}

    if dry_run:
        _print_trajectory(cur, tgt, duration_s, fps, header="[dry-run] move_to_joints")
        return

    follower_smooth_move_to(robot, cur, tgt, duration_s=duration_s, fps=fps)


def go_to_ee(
    kinematics: RobotKinematics,
    robot,
    xyz: list[float],
    *,
    rpy_deg: list[float] | None = None,
    gripper: float | None = None,
    duration_s: float = 2.0,
    fps: int = 30,
    position_weight: float = 1.0,
    orientation_weight: float = 0.0,
    dry_run: bool = False,
) -> dict[str, float]:
    """Move the end-effector to a Cartesian target via inverse kinematics.

    Solves IK on the 5 arm joints using the *current* joint configuration as the
    seed, then routes the resulting joint vector through `move_to_joints`. The
    gripper value is held at its current value (or `gripper` if given).

    Returns the joint action dict that was commanded.
    """
    # Current joint configuration (seed for IK).
    if dry_run or robot is None:
        current_dict = dict(HOME_JOINTS)
    else:
        current_dict = robot.get_observation()

    current_arr = _joints_array_from_dict(current_dict)
    seed = current_arr.copy()

    # Override gripper if requested (preserved through IK by RobotKinematics).
    if gripper is not None:
        seed[5] = float(gripper)

    desired_pose = _pose_from_xyz_rpy(xyz, rpy_deg)

    if rpy_deg is not None and orientation_weight == 0.0:
        orientation_weight = 0.01  # auto-enable a soft orientation constraint

    joint_arr = kinematics.inverse_kinematics(
        seed,
        desired_pose,
        position_weight=position_weight,
        orientation_weight=orientation_weight,
    )
    target_dict = _joints_dict_from_array(joint_arr)

    # Sanity print: FK of the solved configuration vs. the desired pose.
    achieved = kinematics.forward_kinematics(joint_arr[: len(ARM_JOINTS)])
    achieved_xyz = achieved[:3, 3]
    err = np.linalg.norm(achieved_xyz - np.array(xyz))
    print(
        "[IK] desired xyz=" + f"{np.array(xyz).round(4).tolist()}  "
        f"achieved xyz={achieved_xyz.round(4).tolist()}  "
        f"pos_err={err * 1000:.1f} mm"
    )
    print(
        "[IK] solved joints (deg/0-100): "
        + ", ".join(f"{k}={v:.2f}" for k, v in target_dict.items())
    )

    move_to_joints(
        robot,
        target_dict,
        current=current_dict,
        duration_s=duration_s,
        fps=fps,
        dry_run=dry_run,
    )
    return target_dict


def home(
    robot, *, duration_s: float = 3.0, fps: int = 30, dry_run: bool = False
) -> None:
    """Return the arm to HOME_JOINTS."""
    move_to_joints(robot, HOME_JOINTS, duration_s=duration_s, fps=fps, dry_run=dry_run)


def rest(
    robot, *, duration_s: float = 3.0, fps: int = 30, dry_run: bool = False
) -> None:
    """Return the arm to REST_JOINTS."""
    move_to_joints(robot, REST_JOINTS, duration_s=duration_s, fps=fps, dry_run=dry_run)


# ── Dry-run trajectory printer ────────────────────────────────────────────────
def _print_trajectory(
    cur: dict[str, float],
    tgt: dict[str, float],
    duration_s: float,
    fps: int,
    header: str = "[dry-run]",
) -> None:
    steps = max(int(duration_s * fps), 1)
    print(header)
    print("  from: " + ", ".join(f"{k}={v:.2f}" for k, v in cur.items()))
    print("  to:   " + ", ".join(f"{k}={v:.2f}" for k, v in tgt.items()))
    print(f"  steps: {steps} @ {fps} Hz over {duration_s:.2f}s")
    print("  sampled waypoints:")
    for s in (0, steps // 4, steps // 2, 3 * steps // 4, steps):
        if s < 0 or s > steps:
            continue
        t = s / steps
        interp = {k: cur[k] * (1 - t) + tgt[k] * t for k in ACTION_KEYS}
        print(f"    t={t:.2f}  " + ", ".join(f"{k}={v:.2f}" for k, v in interp.items()))
