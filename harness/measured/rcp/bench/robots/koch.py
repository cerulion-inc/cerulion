# SPDX-License-Identifier: AGPL-3.0-only
"""Koch / low cost robot arm (menagerie low_cost_robot_arm/low_cost_robot_arm.xml, unmodified), spec DERIVED from the
model file by bench.derive_spec (G20).

No number below is typed for this arm. OVERRIDES lists what a person had to supply (none). The file loads as it
is (spec["model_fixes"] is empty). The chain ends in a hinge jaw against a fixed finger, so the tool tip is the
fixed finger's tip with the jaw closed.
"""

import os

from bench.derive_spec import derive_spec

_MENAGERIE = os.path.join(os.path.dirname(__file__), "..", "..", "third_party", "mujoco_menagerie")
MJCF = os.path.abspath(os.path.join(_MENAGERIE, "low_cost_robot_arm", "low_cost_robot_arm.xml"))
OVERRIDES = {}
# No RATED joint speed is given: no maker's document with one is named for this arm, so the spec's speed_limit is None
# per joint and the robot's own sweep measures what each joint tracks (source self_test).

SPEC = derive_spec(MJCF, "koch", overrides=OVERRIDES)
