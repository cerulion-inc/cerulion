# SPDX-License-Identifier: AGPL-3.0-only
"""Stretch robots whose spec is derived from the model file alone (G20): EXTRA_ROBOTS = {name: SPEC}.

bench.robots.ROBOTS lists both names and builds them from here the first time one is read (ROBOTS["gen3"], or any
walk over all robots). Importing this module derives both specs (about a second, deterministic: seeded search, no
wall clock).
"""

from bench.robots.gen3 import SPEC as GEN3
from bench.robots.koch import SPEC as KOCH

EXTRA_ROBOTS = {"gen3": GEN3, "koch": KOCH}
