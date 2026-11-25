# SPDX-License-Identifier: AGPL-3.0-only
"""Kinova Gen3 (menagerie kinova_gen3/gen3.xml), spec DERIVED from the model file by bench.derive_spec (G20).

No number below is typed for this arm. OVERRIDES lists what a person had to supply (none). The derivation had to
fix the file for the loader (spec["model_fixes"], written to a copy under runs/derived_mjcf/): its four continuous
joints get +-180 deg, its own camera named "wrist" is renamed, and the base and shoulder (touching at the bearing,
the base fixed to the world) get a contact exclude. The arm has no finger, so the bench's standard paddle goes on
the flange.
"""

import os

from bench.derive_spec import derive_spec

_MENAGERIE = os.path.join(os.path.dirname(__file__), "..", "..", "third_party", "mujoco_menagerie")
MJCF = os.path.abspath(os.path.join(_MENAGERIE, "kinova_gen3", "gen3.xml"))
OVERRIDES = {}
# RATED joint speeds (rad/s): the maker's own robot description, Kinovarobotics ros_kortex kortex_description
# arms/gen3/7dof/urdf/gen3_macro.xacro: velocity 1.3963 (joints 1 to 4) and 1.2218 (joints 5 to 7).
RATINGS = {"speed_limit": ([1.3963, 1.3963, 1.3963, 1.3963, 1.2218, 1.2218, 1.2218],
                           "Kinovarobotics ros_kortex kortex_description arms/gen3/7dof/urdf/gen3_macro.xacro (the "
                           "maker's robot description): velocity 1.3963 rad/s on joints 1 to 4, 1.2218 rad/s on "
                           "joints 5 to 7")}

SPEC = derive_spec(MJCF, "gen3", overrides=OVERRIDES, ratings=RATINGS)
