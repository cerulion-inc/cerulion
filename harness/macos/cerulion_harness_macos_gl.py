# SPDX-License-Identifier: AGPL-3.0-only
"""macOS only. install.sh copies this file into the episode server environment, with a one-line .pth file beside
it that imports it when that environment's Python starts.

The measured runs set MUJOCO_GL=osmesa (the CPU renderer) for every process of an episode. MuJoCo's macOS build
refuses that value when it is imported. This file drops the value before MuJoCo loads, in the episode server
environment only; the simulator itself is started through macos/envhost_macos.sh, which renders through Apple's
OpenGL. No file of the code trees is changed.
"""
import os
import sys

if sys.platform == "darwin" and os.environ.get("MUJOCO_GL") in ("osmesa", "egl"):
    del os.environ["MUJOCO_GL"]
    os.environ.pop("PYOPENGL_PLATFORM", None)
