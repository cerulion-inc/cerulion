# SPDX-License-Identifier: AGPL-3.0-only
"""macOS only: start the benchmark's simulator host with Apple's OpenGL (CGL), offscreen.

The measured runs rendered on Linux with the CPU renderer (OSMesa). MuJoCo's macOS build takes OpenGL from the
system framework, so OSMesa cannot be used there. This file loads MuJoCo and robosuite with MUJOCO_GL=cgl, gives
robosuite MuJoCo's own offscreen CGL context (robosuite's choice on macOS needs a window), and then runs the
simulator host's own main(). The host knows two renderer names, "egl" (a GPU) and "osmesa" (the CPU); it is told
"egl", so it states no measured sub-pixel depth offset (that offset was measured for the CPU renderer only).
Pictures on macOS therefore differ slightly from the measured runs' pictures. No file of the code trees and no
installed package file is changed. Linux needs none of this: there the stock launcher runs.
"""
import os
import sys

os.environ["MUJOCO_GL"] = "cgl"
import mujoco  # noqa: E402,F401
import mujoco.cgl  # noqa: E402
import robosuite.utils.binding_utils as binding_utils  # noqa: E402


class CGLContext:
    """robosuite's context call shape over MuJoCo's offscreen CGL context."""

    def __init__(self, max_width, max_height, device_id=0):
        self._ctx = mujoco.cgl.GLContext(max_width, max_height)

    def make_current(self):
        self._ctx.make_current()

    def free(self):
        self._ctx.free()


binding_utils.GLContext = CGLContext

# The code tree the episode runs from: the launcher starts this file with that tree as the working folder.
sys.path.insert(0, os.environ.get("PLAYGROUND_SNAPSHOT") or os.getcwd())
from bench.ext.libero import envhost  # noqa: E402

argv = list(sys.argv[1:])
if "--gl" in argv:
    argv[argv.index("--gl") + 1] = "egl"
sys.exit(envhost.main(argv))
