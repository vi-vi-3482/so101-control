"""so101_control — programmatic control of an SO-101 follower arm.

This package wraps lerobot's SO-101 follower with four reusable primitives
(`move_to_joints`, `go_to_ee`, `home`, `rest`) plus a few helpers
(`print_status`, `current_joints`, `current_ee_pose`). It is installable as
the `so101-control` distribution and also exposes a `so101-control` CLI (see
`so101_control.cli`).
"""

from so101_control.control import (
    ACTION_KEYS,
    ALL_JOINTS,
    ARM_JOINTS,
    EE_FRAME,
    HOME_JOINTS,
    current_ee_pose,
    current_joints,
    follower_smooth_move_to,
    go_to_ee,
    home,
    move_to_joints,
    print_status,
    rest,
)

__version__ = "0.1.1"
__all__ = [
    "HOME_JOINTS",
    "ARM_JOINTS",
    "ALL_JOINTS",
    "ACTION_KEYS",
    "EE_FRAME",
    "follower_smooth_move_to",
    "go_to_ee",
    "home",
    "move_to_joints",
    "print_status",
    "current_joints",
    "current_ee_pose",
    "rest",
]
