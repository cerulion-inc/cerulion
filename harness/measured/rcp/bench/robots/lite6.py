# SPDX-License-Identifier: AGPL-3.0-only
"""UFACTORY Lite 6 (menagerie ufactory_lite6/lite6.xml, unmodified). Tool: a fingertip paddle on the flange.

collision_excludes: link4 against link6. The simulator collides the convex hull of each collision mesh, and link4's
hull fills the hollow its own shape leaves around the wrist, so the hulls met at |joint5| over about 93 deg (home is
90) while the meshes themselves never come within 2.6 mm of each other over joint5's whole published range
(-124 to 124 deg, any joint6). tests/test_model_fix1.py re-measures that on the meshes for every pair listed here.
"""

import os

from ._build import camera_on_tool

_MENAGERIE = os.path.join(os.path.dirname(__file__), "..", "..", "third_party", "mujoco_menagerie")

_TIP = {"body": "finger_tool", "pos": [0.09, 0.0, 0.0], "x_axis": [1, 0, 0], "z_axis": [0, 0, 1]}

SPEC = {
    "name": "lite6",
    "mjcf": os.path.abspath(os.path.join(_MENAGERIE, "ufactory_lite6", "lite6.xml")),
    "arm_joints": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
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
    "finger_tool": {"parent": "link6", "pos": [0.0, 0.0, 0.0], "x_axis": [0, 0, 1], "z_axis": [1, 0, 0],
                    "size_mm": [90, 20, 12], "mass_kg": 0.05},
    "camera": camera_on_tool(_TIP, back_m=0.10, up_m=0.06, fovy_deg=65.0),
    "q_home": [0.0, 0.0, 1.57, 0.0, 1.57, 0.0],
    # kp, kd and torque_limit are not typed here: bench.robots fills them from the model file on first use
    # (bench/robots/_gains.py fill_from_file: the file's actuator gains and force ranges, made stable for the bench's
    # explicit servo law at its physics step).
    # RATED joint speeds (rad/s): the maker's own robot description, xArm-Developer xarm_ros
    # xarm_description/urdf/lite6/lite6.urdf.xacro: velocity 3.14 on every joint.
    "speed_limit": [3.14] * 6,
    "source": {
        "speed_limit": "rating: xArm-Developer xarm_ros xarm_description/urdf/lite6/lite6.urdf.xacro (the maker's "
                       "robot description): velocity 3.14 rad/s on every joint",
    },
    "collision_excludes": [
        {"bodies": ["link4", "link6"],
         "why": "hull artifact: the convex hull of the link4 collision mesh fills its own hollow around the wrist; "
                "the meshes themselves stay apart over the whole joint5 and joint6 ranges"},
    ],
}
