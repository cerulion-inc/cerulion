# SPDX-License-Identifier: AGPL-3.0-only
"""Universal Robots UR5e (menagerie universal_robots_ur5e/ur5e.xml, unmodified). Tool: a fingertip paddle on the
flange (the arm has no finger)."""

import math
import os

from ._build import camera_on_tool

_MENAGERIE = os.path.join(os.path.dirname(__file__), "..", "..", "third_party", "mujoco_menagerie")

_TIP = {"body": "finger_tool", "pos": [0.09, 0.0, 0.0], "x_axis": [1, 0, 0], "z_axis": [0, 0, 1]}

SPEC = {
    "name": "ur5e",
    "mjcf": os.path.abspath(os.path.join(_MENAGERIE, "universal_robots_ur5e", "ur5e.xml")),
    "arm_joints": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                   "wrist_3_joint"],
    "gripper": None,
    "tip": _TIP,
    "tool_surfaces": [
        {"name": "tip", "body": "finger_tool", "pos": [0.09, 0.0, 0.0], "normal": [1, 0, 0], "size_mm": [20, 12]},
        {"name": "pad_bottom", "body": "finger_tool", "pos": [0.07, 0.0, -0.006], "normal": [0, 0, -1],
         "size_mm": [40, 20]},
        {"name": "pad_top", "body": "finger_tool", "pos": [0.07, 0.0, 0.006], "normal": [0, 0, 1], "size_mm": [40, 20]},
        {"name": "side_left", "body": "finger_tool", "pos": [0.07, 0.01, 0.0], "normal": [0, 1, 0], "size_mm": [40, 12]},
        {"name": "side_right", "body": "finger_tool", "pos": [0.07, -0.01, 0.0], "normal": [0, -1, 0],
         "size_mm": [40, 12]},
    ],
    "finger_tool": {"parent": "wrist_3_link", "pos": [0.0, 0.1, 0.0], "x_axis": [0, 1, 0], "z_axis": [0, 0, 1],
                    "size_mm": [90, 20, 12], "mass_kg": 0.05},
    "camera": camera_on_tool(_TIP, back_m=0.10, up_m=0.06, fovy_deg=65.0),
    "q_home": [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
    # kp, kd and torque_limit are not typed here: bench.robots fills them from the model file on first use
    # (bench/robots/_gains.py fill_from_file: the file's actuator gains and force ranges, made stable for the bench's
    # explicit servo law at its physics step).
    # RATED joint speeds (rad/s): the maker's own robot description, Universal_Robots_ROS2_Description
    # config/ur5e/joint_limits.yaml: max_velocity 180 deg/s on every joint (its header names the e-Series user manual).
    "speed_limit": [math.radians(180.0)] * 6,
    "source": {
        "speed_limit": "rating: Universal_Robots_ROS2_Description config/ur5e/joint_limits.yaml (the maker's robot "
                       "description, from the e-Series user manual): max_velocity 180 deg/s on every joint",
    },
}
