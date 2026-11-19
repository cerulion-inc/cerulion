# SPDX-License-Identifier: AGPL-3.0-only
"""Franka Emika Panda (menagerie franka_emika_panda/panda.xml, unmodified). Tool: the closed fingers' tip."""

import os

from ._build import camera_on_tool

_MENAGERIE = os.path.join(os.path.dirname(__file__), "..", "..", "third_party", "mujoco_menagerie")

_TIP = {"body": "hand", "pos": [0.0, 0.0, 0.1123], "x_axis": [0, 0, 1], "z_axis": [1, 0, 0]}

SPEC = {
    "name": "panda",
    "mjcf": os.path.abspath(os.path.join(_MENAGERIE, "franka_emika_panda", "panda.xml")),
    "arm_joints": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
    "gripper": {"joints": ["finger_joint1", "finger_joint2"], "actuators": ["gripper_finger_joint1",
                "gripper_finger_joint2"], "open": [0.04, 0.04], "closed": [0.0, 0.0], "kp": [400.0, 400.0],
                "forcerange": [70.0, 70.0], "speed": None},
    "tip": _TIP,
    "tool_surfaces": [
        {"name": "tip", "body": "hand", "pos": [0.0, 0.0, 0.1123], "normal": [0, 0, 1], "size_mm": [18, 29]},
        {"name": "pad_top", "body": "hand", "pos": [0.009, 0.0, 0.104], "normal": [1, 0, 0], "size_mm": [16, 29]},
        {"name": "pad_bottom", "body": "hand", "pos": [-0.009, 0.0, 0.104], "normal": [-1, 0, 0], "size_mm": [16, 29]},
        # tool y (the hand's left) is hand -y: side_left is the outer face of the finger on hand -y
        {"name": "side_left", "body": "hand", "pos": [0.0, -0.0144, 0.104], "normal": [0, -1, 0],
         "size_mm": [16, 18]},
        {"name": "side_right", "body": "hand", "pos": [0.0, 0.0144, 0.104], "normal": [0, 1, 0], "size_mm": [16, 18]},
    ],
    "finger_tool": None,
    "camera": camera_on_tool(_TIP, back_m=0.10, up_m=0.06, fovy_deg=65.0),
    "q_home": [0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853],
    # kp, kd and torque_limit are not typed here: bench.robots fills them from the model file on first use
    # (bench/robots/_gains.py fill_from_file: the file's actuator gains and force ranges, made stable for the bench's
    # explicit servo law at its physics step).
    # RATED joint speeds (rad/s): the maker's own robot description, franka_ros franka_description
    # robots/panda/joint_limits.yaml: velocity 2.1750 (joints 1 to 4) and 2.6100 (joints 5 to 7).
    "speed_limit": [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100],
    "source": {
        "speed_limit": "rating: franka_ros franka_description robots/panda/joint_limits.yaml (the maker's robot "
                       "description): velocity 2.1750 rad/s on joints 1 to 4, 2.6100 rad/s on joints 5 to 7",
        "gripper": "person: kp 400 N/m and forcerange 70 N per finger are TYPED for the bench's own jaw servo, not "
                   "read from the file (its tendon actuator gives 50 N/m and 50 N per finger: 2 N of squeeze at a "
                   "40 mm opening, which holds nothing) and not checked against a maker's document in this spec; "
                   "speed: none stated, the robot times its own jaws",
    },
}
