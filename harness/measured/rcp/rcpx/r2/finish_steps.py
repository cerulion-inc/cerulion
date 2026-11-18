# SPDX-License-Identifier: AGPL-3.0-only
"""The measurement step of the claim gate's travel rules (package finish), pure.

No millimetre is typed: each act record carries rigid_mm, the arm's own measurement step at the act's pose (finish
writes it), and every travel rule is a multiple (a ratio of rcpx.bounds_ratios) of the step of the acts that made
the travel. An act with no recorded step counts a step of 0: then any travel over 0 reads as travel, and no travel
never does (a threshold that can be zero is never crossed by zero).
"""

from rcpx import bounds_ratios as R


def under(travel_mm, step_mm):
    """True when a travel reads as none: no travel at all, or less than the step."""
    return not travel_mm > 0 or travel_mm < step_mm


def step_mm(act_or_acts):
    """The largest measurement step (rigid_mm) recorded with these acts; an act with none counts 0."""
    acts = [act_or_acts] if isinstance(act_or_acts, dict) else act_or_acts
    return max((a.get("rigid_mm") or 0.0 for a in acts), default=0.0)


def small_mm(act):
    """The act's small move limit as recorded, else QUANT x its measurement step."""
    value = act.get("small_mm")
    return R.QUANT * step_mm(act) if value is None else value


__all__ = ["small_mm", "step_mm", "under"]
