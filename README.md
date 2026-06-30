# so101-control

A small Python package for programmatic control of an SO-101 robot arm via
[lerobot](https://github.com/huggingface/lerobot).

Move the arm to specific joint-angle configurations, or drive the end-effector to
a Cartesian target using inverse kinematics — no teleop or trained policy
required. Install it as a library in your own project, or use the bundled
`so101-control` CLI.

## What's included

| Path                          | Purpose                                                                |
|-------------------------------|------------------------------------------------------------------------|
| `src/so101_control/`          | The installable `so101_control` package (control wrappers + CLI).     |
| `src/so101_control/control.py`| Reusable primitives: `move_to_joints`, `go_to_ee`, `home`, `rest`.    |
| `src/so101_control/cli.py`    | The `so101-control` command-line entry point.                          |
| `examples/`                   | Example scripts showing library usage.                                 |
| `so101_commands.md`*          | Copy-pasteable lerobot CLI commands (calibrate, teleoperate, record).  |
| `PLACO_FIX.md`*               | Notes on the `placo` / `cmeel-urdfdom` wheel conflict and the pin that fixes it. |

\* If present alongside this README.

## Installation

Requires Python ≥3.12 and [uv](https://docs.astral.sh/uv/) (or any PEP 621-aware
installer such as `pip`).

### From source (development)

```bash
git clone <this-repo>
cd so101-control
uv sync
```

This installs the `so101_control` package (importable as `import so101_control`)
together with `lerobot[kinematics,feetech]` (which pulls in `placo` for IK) and
pins `cmeel-urdfdom==4.0.1` to work around a shared-library mismatch in the
placo wheels (see [`PLACO_FIX.md`](PLACO_FIX.md) if present).

### As a dependency in your own project

Add it to your `pyproject.toml` dependencies (or let `uv` do it for you):

```bash
# from a local checkout
uv add /path/to/so101-control
# …or from git
uv add "git+https://github.com/youruser/so101-control.git"
```

Then import it from your own code:

```python
from so101_control import (
    HOME_JOINTS,
    go_to_ee,
    home,
    move_to_joints,
    print_status,
    rest,
)
```

### Fetch the SO-101 URDF + meshes (only needed for end-effector / IK mode)

The URDF references STL meshes in an `assets/` folder. Download both:

```bash
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
```

### Calibrate the arm (one-time)

If you haven't already calibrated your follower, follow the commands in
[`so101_commands.md`](so101_commands.md). Calibration is stored under
`~/.cache/huggingface/lerobot/calibration/robots/<robot-id>/` and is located
automatically by the `--robot-id` you pass to the CLI.

## Quick start

Once installed, the `so101-control` CLI is available on your PATH.

Dry-run (no hardware needed — prints the planned trajectory):

```bash
so101-control --mode joint \
    --target-joints shoulder_pan=0,shoulder_lift=-20,elbow_flex=60,wrist_flex=30,wrist_roll=0,gripper=50 \
    --dry-run
```

Live joint move:

```bash
so101-control \
    --port /dev/tty.usbmodem585A0076841 \
    --robot-id my_awesome_follower_arm \
    --mode joint \
    --target-joints shoulder_pan=0,shoulder_lift=-20,elbow_flex=60,wrist_flex=30,wrist_roll=0,gripper=50
```

End-effector move (position-only IK by default):

```bash
so101-control \
    --port /dev/tty.usbmodem585A0076841 \
    --robot-id my_awesome_follower_arm \
    --urdf-path /absolute/path/to/so101_new_calib.urdf \
    --mode ee --target-xyz 0.2,0.0,0.15
```

You can also invoke the module directly without installing the console script:

```bash
uv run python -m so101_control.cli --mode home --dry-run
# or, from a source checkout:
uv run so101-control --mode home --dry-run
```

See the **Library usage** section below for the full API, library usage, safety
knobs, and the joint schema.

## Using the wrappers in your own code

```python
import time

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from so101_control import (
    HOME_JOINTS,
    go_to_ee,
    home,
    move_to_joints,
    print_status,
    rest,
)

robot = SO101Follower(
    SO101FollowerConfig(
        port="/dev/tty.usbmodem...", id="my_awesome_follower_arm"
    )
)
robot.connect(calibrate=False)

print("Moving to home joint angle")
move_to_joints(robot, HOME_JOINTS, duration_s=2.0)

kin = RobotKinematics(
    urdf_path="/absolute/path/to/so101_new_calib.urdf",  # Must be an absolute path
    target_frame_name="gripper_frame_link",
    joint_names=[
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    ],
)

print_status(kin, robot)

pos = [0.3, 0.0, 0.5]
print(f"Moving to EE position {pos}")
go_to_ee(kin, robot, pos, duration_s=3.0)
time.sleep(0.5)


rest(robot)
time.sleep(0.1)
robot.disconnect()
```

A ready-to-edit version of the above lives at
[`examples/example_move_ee.py`](examples/example_move_ee.py).

## Additional Notes
The URDF file path must be an absolute path from the system root, otherwise it
fails to find the `assets/` folder containing the mesh STLs.

### Robot Coordinate system

Orientation facing the robot. The EE is pointed forward in the home position.

| Axis | Orientation |
|------|-------------|
| X    | Forward     |
| Y    | Right       |
| Z    | Up          |
