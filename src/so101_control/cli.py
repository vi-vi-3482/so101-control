"""Command-line entry point for the `so101-control` package.

Installed as the `so101-control` console script (see `[project.scripts]` in
`pyproject.toml`). It wraps the primitives in `so101_control.control` so the
arm can be driven from the shell without writing any Python.

Modes:
    joint   -- move to a joint-angle target (--target-joints)
    ee      -- move the end-effector to a Cartesian target via IK (--target-xyz)
    home    -- return to HOME_JOINTS (neutral extended pose)
    status  -- print current joint angles and the FK end-effector pose

Run `so101-control --help` for the full argument list, or invoke the module
directly with `python -m so101_control.cli`.
"""

import argparse
import sys
from pathlib import Path

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from so101_control.control import (
    ACTION_KEYS,
    ARM_JOINTS,
    EE_FRAME,
    go_to_ee,
    home,
    move_to_joints,
    print_status,
)


# ── CLI ───────────────────────────────────────────────────────────────────────
def _parse_target_joints(s: str) -> dict[str, float]:
    """Parse 'shoulder_pan=0,shoulder_lift=-20,...' into a `.pos` keyed dict."""
    out: dict[str, float] = {}
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        if "=" not in part:
            raise ValueError(f"bad --target-joints item {part!r}; expected name=value")
        name, val = part.split("=", 1)
        name = name.strip()
        if not name.endswith(".pos"):
            name = f"{name}.pos"
        out[name] = float(val.strip())
    unknown = set(out) - set(ACTION_KEYS)
    if unknown:
        raise ValueError(f"unknown joint(s): {unknown}; valid: {ACTION_KEYS}")
    return out


def _parse_xyz(s: str) -> list[float]:
    vals = [float(v) for v in s.split(",")]
    if len(vals) != 3:
        raise ValueError("--target-xyz must be x,y,z")
    return vals


def _parse_rpy(s: str) -> list[float]:
    vals = [float(v) for v in s.split(",")]
    if len(vals) != 3:
        raise ValueError("--target-rpy must be roll,pitch,yaw in degrees")
    return vals


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Programmatic SO-101 control (joints + IK)."
    )
    p.add_argument(
        "--port",
        default="/dev/tty.usbmodem585A0076841",
        help="Serial port of the follower arm.",
    )
    p.add_argument(
        "--robot-id",
        default="my_awesome_follower_arm",
        help="Robot id used to locate the calibration file.",
    )
    p.add_argument(
        "--urdf-path",
        default="./so101_new_calib.urdf",
        help="Path to the SO-101 URDF (required for --mode ee).",
    )
    p.add_argument(
        "--mode",
        choices=["joint", "ee", "home", "status"],
        default="home",
        help="What to do.",
    )
    p.add_argument(
        "--target-joints",
        type=_parse_target_joints,
        default=None,
        help="joint=val,... (degrees for arm, 0-100 for gripper). Used with --mode joint.",
    )
    p.add_argument(
        "--target-xyz",
        type=_parse_xyz,
        default=None,
        help="x,y,z in meters. Used with --mode ee.",
    )
    p.add_argument(
        "--target-rpy",
        type=_parse_rpy,
        default=None,
        help="roll,pitch,yaw in degrees (optional, --mode ee).",
    )
    p.add_argument(
        "--gripper",
        type=float,
        default=None,
        help="Gripper target 0-100 for --mode ee (held at current if omitted).",
    )
    p.add_argument(
        "--duration-s", type=float, default=2.0, help="Move duration in seconds."
    )
    p.add_argument("--fps", type=int, default=30, help="Control rate during the move.")
    p.add_argument(
        "--max-relative-target",
        type=float,
        default=10.0,
        help="Per-step joint clip (deg) for safety. Applied inside send_action.",
    )
    p.add_argument(
        "--orientation-weight",
        type=float,
        default=0.0,
        help="IK orientation weight. 0.0 = position-only (default for SO-101).",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Plan and print only; do not open the serial port or move hardware.",
    )
    p.add_argument(
        "--home-after",
        action="store_true",
        help="Return to HOME_JOINTS after the requested move completes.",
    )
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    # Validate mode-specific args.
    if args.mode == "joint" and args.target_joints is None:
        print("ERROR: --mode joint requires --target-joints", file=sys.stderr)
        return 2
    if args.mode == "ee":
        if args.target_xyz is None:
            print("ERROR: --mode ee requires --target-xyz", file=sys.stderr)
            return 2
        if not Path(args.urdf_path).exists():
            print(
                f"ERROR: URDF not found at {args.urdf_path!r}.\n"
                f"`--mode ee` and `--mode status` require the SO-101 URDF, which also "
                f"references STL mesh assets in a sibling `assets/` folder.\n"
                f"Download both from the SO-ARM100 repo:\n"
                f"  curl -L -o so101_new_calib.urdf \\\n"
                f"    https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/so101_new_calib.urdf\n"
                f"  mkdir -p assets && cd assets\n"
                f"  for f in base_motor_holder_so101_v1 base_so101_v2 motor_holder_so101_base_v1 \\\n"
                f"    motor_holder_so101_wrist_v1 moving_jaw_so101_v1 rotation_pitch_so101_v1 \\\n"
                f"    sts3215_03a_no_horn_v1 sts3215_03a_v1 under_arm_so101_v1 upper_arm_so101_v1 \\\n"
                f"    waveshare_mounting_plate_so101_v2 wrist_roll_follower_so101_v1 \\\n"
                f"    wrist_roll_pitch_so101_v2; do\n"
                f'    curl -sL -o "$f.stl" \\\n'
                f'      "https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/assets/$f.stl"\n'
                f"  done\n"
                f"  cd ..\n"
                f"Then pass --urdf-path /absolute/path/to/so101_new_calib.urdf "
                f"(the path must be absolute so the assets/ folder can be located).",
                file=sys.stderr,
            )
            if not args.dry_run:
                return 2
            print(
                "WARNING: proceeding in dry-run without a URDF check (IK will still fail).",
                file=sys.stderr,
            )

    dry_run = args.dry_run

    # ── Build kinematics (needed for ee/status modes) ─────────────────────────
    kinematics: RobotKinematics | None = None
    if args.mode in ("ee", "status") and Path(args.urdf_path).exists():
        kinematics = RobotKinematics(
            urdf_path=str(Path(args.urdf_path).resolve()),
            target_frame_name=EE_FRAME,
            joint_names=ARM_JOINTS,  # IK only on the 5 arm joints
        )

    # ── Dry-run path: no hardware ─────────────────────────────────────────────
    if dry_run:
        print("=" * 72)
        print("DRY RUN — no hardware will be moved.")
        print("=" * 72)
        if args.mode == "joint":
            move_to_joints(
                None,
                args.target_joints,
                duration_s=args.duration_s,
                fps=args.fps,
                dry_run=True,
            )
        elif args.mode == "ee":
            go_to_ee(
                kinematics,
                None,
                args.target_xyz,
                rpy_deg=args.target_rpy,
                gripper=args.gripper,
                duration_s=args.duration_s,
                fps=args.fps,
                orientation_weight=args.orientation_weight,
                dry_run=True,
            )
        elif args.mode == "home":
            home(None, duration_s=args.duration_s, fps=args.fps, dry_run=True)
        elif args.mode == "status":
            print("[dry-run] status mode has no hardware to read; showing HOME_JOINTS.")
            print_status(kinematics, None)
        if args.home_after and args.mode != "home":
            home(None, duration_s=args.duration_s, fps=args.fps, dry_run=True)
        print("=" * 72)
        print("Dry run complete.")
        return 0

    # ── Live path: connect to the arm ─────────────────────────────────────────
    config = SO101FollowerConfig(
        port=args.port,
        id=args.robot_id,
        use_degrees=True,
        max_relative_target=args.max_relative_target,
    )
    robot = SO101Follower(config)

    try:
        robot.connect(calibrate=False)  # use existing calibration on disk
        print(f"Connected to {robot} ({args.port}).")

        if args.mode == "joint":
            move_to_joints(
                robot, args.target_joints, duration_s=args.duration_s, fps=args.fps
            )
        elif args.mode == "ee":
            go_to_ee(
                kinematics,
                robot,
                args.target_xyz,
                rpy_deg=args.target_rpy,
                gripper=args.gripper,
                duration_s=args.duration_s,
                fps=args.fps,
                orientation_weight=args.orientation_weight,
            )
        elif args.mode == "home":
            home(robot, duration_s=args.duration_s, fps=args.fps)
        elif args.mode == "status":
            print_status(kinematics, robot)

        if args.home_after and args.mode != "home":
            home(robot, duration_s=args.duration_s, fps=args.fps)
    finally:
        try:
            robot.disconnect()
            print("Disconnected.")
        except Exception as e:  # noqa: BLE001
            print(f"Warning during disconnect: {e}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
