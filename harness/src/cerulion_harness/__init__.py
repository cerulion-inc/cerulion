# SPDX-License-Identifier: AGPL-3.0-only
"""The Cerulion harness's runtime: run one benchmark cell with and without the harness, against your own model
endpoint, and the robot body records the harness setup reads.

A process that imports this package writes no bytecode from then on: run-cell and the body record measurement
import this package's modules and the measured code (measured/assemble.py, measured/rcp) from the harness
folder, and leave no __pycache__ folder in it. Python compiles this file before it runs it, so only this file's
own bytecode is still written, unless Python starts with -B or PYTHONDONTWRITEBYTECODE set."""
import sys

sys.dont_write_bytecode = True
