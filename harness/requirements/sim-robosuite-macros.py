# SPDX-License-Identifier: AGPL-3.0-only
"""robosuite's private settings in the simulator environment: install.sh copies this file to
robosuite/macros_private.py there.

robosuite 1.4.0 imports robosuite.macros_private first, before any of its modules reads a setting, and reads every
setting from robosuite.macros. This file sets one there: FILE_LOGGING_LEVEL = None turns off robosuite's file log,
which every simulator start otherwise appends to /tmp/robosuite.log. Its console log is left as it is. A file that
only assigned the name here would change nothing: robosuite star-imports it into the robosuite package, not into
robosuite.macros.
"""
import robosuite.macros as _macros

__all__ = []

_macros.FILE_LOGGING_LEVEL = None
