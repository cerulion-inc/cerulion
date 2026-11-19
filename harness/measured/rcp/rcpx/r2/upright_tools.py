# SPDX-License-Identifier: AGPL-3.0-only
"""Schemas for a turn, a tipping push and an axis reading."""
from rcpx import surfaces as S
from rcpx.r2.shove_tools import CLEAR, LABEL, POINT, number, push_properties

SPOT = {"anyOf": [{"type": "string"}, POINT]}
TOOLS = [
    S._tool("set_upright", "Turn a held thing so its chosen axis points up, carry it to a support point, lower "
            "until its weight comes off, reopen to the pick's opening, rise and look. Locate both axis ends "
            "before the pick; returns placed, landed, tilt_deg_held, phases and bounds.",
            S._obj({"up_axis": {"anyOf": [{"type": "string", "enum": ["from_look"]}, POINT]},
                    "axis_from": SPOT, "axis_to": SPOT, "place_point_mm": POINT, "place_label": LABEL,
                    "release_mm": number("stated release height above support, mm; omitted means touchdown"),
                    "clear": CLEAR}),
            {"axis_from": "lower_end", "axis_to": "upper_end", "place_label": "set_spot"},
            "stand the held thing with its lower end down at the located support spot"),
    S._tool("tip_over", "Lay a standing free thing on its side by a push near its top. Find its support, push "
            "until it falls away or the travel ends, back off and look. Height, travel and standoff are derived "
            "when omitted; returns tipped, pushed_mm, stopped, phases and bounds.",
            S._obj({k: v for k, v in push_properties().items() if k != "max_force_n"}),
            {"label": "upper_end", "direction_deg": 0}, "tip the thing at the located upper end outward"),
    S._tool("axis_tilt", "Measure the tilt from up between two points located after set-down. With no axis_from, "
            "use the last set-down estimate. Returns tilt_deg and lean_bearing_deg; no motion.",
            S._obj({"axis_from": SPOT, "axis_to": SPOT}, required=("axis_to",)),
            {"axis_to": "upper_end_again"}, "measure the tilt from the set-down estimate to the relocated end")]

PRACTICES = ("Locate the lower and upper ends before the pick, then set_upright with those ends.\n"
             "Use tip_over to lay a standing thing down, then locate it again.\n"
             "After set_upright, locate the upper end again and call axis_tilt.")
