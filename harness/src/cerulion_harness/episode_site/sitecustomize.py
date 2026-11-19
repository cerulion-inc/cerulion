# SPDX-License-Identifier: AGPL-3.0-only
"""Run at the start of each Python process of an episode that starts with run-cell's PYTHONPATH: run-cell puts
this folder first on it, and Python imports the first module named sitecustomize on its path when it starts
(unless it is started with -S, -E or -I). This one makes urllib follow no redirect in the process
(cerulion_harness/redirects.py), so the model key goes to the URL given and to no other, and then runs the
sitecustomize module it hides, if the interpreter has one. Standard library only."""
import importlib.machinery
import importlib.util
import os
import sys


def _run(name, spec):
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_HERE = os.path.realpath(os.path.dirname(os.path.abspath(__file__)))
_run("cerulion_harness_redirects", importlib.util.spec_from_file_location(
    "cerulion_harness_redirects", os.path.join(os.path.dirname(_HERE), "redirects.py"))).refuse_everywhere()
# This folder is left out of the search by its real path, however the path names it, so this module never finds
# itself again.
_hidden = importlib.machinery.PathFinder.find_spec(
    "sitecustomize", [p for p in sys.path if os.path.realpath(p or os.curdir) != _HERE])
if _hidden is not None:
    _run("_hidden_sitecustomize", _hidden)
