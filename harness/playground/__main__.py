# SPDX-License-Identifier: AGPL-3.0-only
"""Start the sim server: python harness/playground [--stdio | --http [ADDR]] [--state LABEL] [--settings FILE].

Run as a folder, this folder is first on sys.path. It is replaced by its parent, so the modules here are imported
as the package `playground` and never shadow a top-level module of the measured code.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:] = [p for p in sys.path if os.path.abspath(p or os.curdir) != HERE]
sys.path.insert(0, os.path.dirname(HERE))

from playground.cli import main  # noqa: E402

sys.exit(main())
