# SPDX-License-Identifier: AGPL-3.0-only
"""TRS SO-ARM100 (menagerie trs_so_arm100/so_arm100.xml, unmodified). Tool: the fixed jaw tip, gripper closed."""

import os

from ._build import camera_on_tool

_MENAGERIE = os.path.join(os.path.dirname(__file__), "..", "..", "third_party", "mujoco_menagerie")

_TIP = {"body": "Fixed_Jaw", "pos": [0.013, -0.1064, 0.0], "x_axis": [0, -1, 0], "z_axis": [1, 0, 0]}

SPEC = {
    "name": "so100",
    "mjcf": os.path.abspath(os.path.join(_MENAGERIE, "trs_so_arm100", "so_arm100.xml")),
    "arm_joints": ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"],
    # closed = the Jaw joint's lower limit in the menagerie file (-0.174 rad): the jaws as shut as they go (pads 2.6 mm
    # apart). At 0.0 rad the moving jaw stands 22 to 37 mm off the fixed jaw and is what meets a surface beside the
    # finger, so a press read its force on the wrong part.
    "gripper": {"joints": ["Jaw"], "actuators": ["gripper_Jaw"], "open": [1.2], "closed": [-0.174],
                "kp": None, "forcerange": None, "speed": None},   # kp, forcerange: from the file (_gains.py)
    "tip": _TIP,
    "tool_surfaces": [
        {"name": "tip", "body": "Fixed_Jaw", "pos": [0.013, -0.1064, 0.0], "normal": [0, -1, 0], "size_mm": [11, 14]},
        {"name": "back", "body": "Fixed_Jaw", "pos": [0.0186, -0.096, 0.0], "normal": [1, 0, 0], "size_mm": [20, 14]},
        # tool y (left) is Fixed_Jaw -z
        {"name": "side_left", "body": "Fixed_Jaw", "pos": [0.013, -0.096, -0.0068], "normal": [0, 0, -1],
         "size_mm": [20, 11]},
        {"name": "side_right", "body": "Fixed_Jaw", "pos": [0.013, -0.096, 0.0068], "normal": [0, 0, 1],
         "size_mm": [20, 11]},
        # the closed moving jaw is the tool's underside (tool -z): its outer face 8 mm behind the tip, 14 mm under it
        {"name": "jaw_under", "body": "Moving_Jaw", "pos": [-0.0061, -0.0762, 0.0], "normal": [0.985, 0.173, 0.0],
         "size_mm": [15, 10]},
    ],
    "finger_tool": None,
    "camera": camera_on_tool(_TIP, back_m=0.075, up_m=0.03, fovy_deg=70.0),
    "q_home": [0.0, -1.57, 1.57, 1.57, -1.57],
    # kp, kd and torque_limit are not typed here: bench.robots fills them from the model file on first use
    # (bench/robots/_gains.py fill_from_file: the file's actuator gains and force ranges, made stable for the bench's
    # explicit servo law at its physics step).
    # No RATED joint speed is typed: this spec names no maker's document that states one, so the robot's own
    # sweep measures what each joint tracks (source self_test).
    "speed_limit": None,
    "source": {
        "speed_limit": "self_test: no maker's document with a joint speed is named for this arm; the robot's own "
                       "sweep measures what it tracks",
    },
}
