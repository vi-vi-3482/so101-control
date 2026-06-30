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
    urdf_path="/absolute/path/to/so101_new_calib.urdf",  # Make absolute path
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

