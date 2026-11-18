# SPDX-License-Identifier: AGPL-3.0-only
"""Shared helpers for robot_spec files and the robot loader (no robot numbers here).

camera_on_tool: places a wrist camera in the tip body from a few semantic numbers (how far behind the tip, how far
above it along the tool z axis, field of view) so the camera looks along the tool x axis with the tip in the lower
part of the image. The result is the SPEC camera dict (body, pos, xyaxes, fovy, width, height).
"""

import math

import numpy as np


def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def tool_axes(x_axis, z_axis):
    """Orthonormal (X, Y, Z) of the tool frame expressed in the tip body; z is re-orthogonalised against x."""
    X = unit(x_axis)
    Z = np.asarray(z_axis, dtype=float)
    Z = unit(Z - X * float(Z @ X))
    Y = np.cross(Z, X)
    return X, Y, Z


def camera_on_tool(tip, back_m, up_m, fovy_deg, tip_v_frac=0.78, width=640, height=480):
    """Camera dict in tip["body"]: behind the tip by back_m along tool x, above it by up_m along tool z.

    The optical axis is pitched toward the tip so the tip lands at tip_v_frac of the image height (v down).
    """
    X, Y, Z = tool_axes(tip["x_axis"], tip["z_axis"])
    p_tip = np.asarray(tip["pos"], dtype=float)
    pos = p_tip - X * back_m + Z * up_m
    beta = math.atan2(up_m, back_m)  # angle below the tool x axis at which the tip sits
    want = math.atan((tip_v_frac - 0.5) * 2.0 * math.tan(math.radians(fovy_deg) / 2.0))
    alpha = beta - want  # pitch the optical axis down by alpha
    look = math.cos(alpha) * X - math.sin(alpha) * Z
    up = math.cos(alpha) * Z + math.sin(alpha) * X
    right = np.cross(look, up)
    return {
        "body": tip["body"],
        "pos": [float(v) for v in pos],
        "xyaxes": [float(v) for v in np.concatenate([right, up])],
        "fovy": float(fovy_deg),
        "width": int(width),
        "height": int(height),
    }


def mat_from_xz(x_axis, z_axis):
    """3x3 rotation whose columns are (X, Y, Z) of the frame given by its x and z axes."""
    X, Y, Z = tool_axes(x_axis, z_axis)
    return np.column_stack([X, Y, Z])


def mat_from_xyaxes(xyaxes):
    x = unit(xyaxes[:3])
    y = np.asarray(xyaxes[3:], dtype=float)
    y = unit(y - x * float(y @ x))
    return np.column_stack([x, y, np.cross(x, y)])


def quat_from_mat(R):
    import mujoco

    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).reshape(9))
    return q


def mat_from_quat(q):
    import mujoco

    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, dtype=float))
    return R.reshape(3, 3)
