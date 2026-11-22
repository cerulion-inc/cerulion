# SPDX-License-Identifier: AGPL-3.0-only
"""Tool schemas and practices for a surface push."""
from rcpx import surfaces as S
from rcpx.r2.shove_args import CONTACTS

POINT = {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
         "description": "a point [x, y, z] in base frame mm"}
LABEL = {"type": "string", "description": "a fresh located label in robot://world"}
CLEAR = {"anyOf": [{"type": "boolean"}, {"type": "object", "properties": {"look": {"type": "string"}},
                                       "required": ["look"], "additionalProperties": False}],
         "description": "true after your own look when you judge the way clear, or {look: id}: recorded in the "
                        "result; it changes no speed"}


def number(text):
    return {"type": "number", "description": text}


def push_properties():
    return {"point_mm": POINT, "label": LABEL, "toward_mm": POINT, "toward_label": LABEL,
            "direction_deg": number("bearing in the base frame, degrees from x toward y"),
            "distance_mm": number("travel after meeting, mm; omitted travel is derived where offered"),
            "height_mm": number("push height over support, mm; omitted height is derived"),
            "contact": {"type": "string", "enum": list(CONTACTS), "description": "closed_jaws or open_jaws"},
            "max_force_n": number("stated force ceiling, N; omitted force is derived and reported"),
            "standoff_mm": number("distance behind the point, mm; omitted distance is derived"), "clear": CLEAR}


TOOLS = [S._tool("shove", "Push a free thing along its support: feel the top and support, choose the lowest "
                 "height that clears the hand, push within the derived force ceiling, back off and look. "
                 "Returns pushed_mm, followed, stopped, phases, bounds and next_stroke_mm.",
                 S._obj(push_properties(), required=("distance_mm",)),
                 {"label": "near_side", "toward_label": "farther_point", "distance_mm": 1},
                 "push from the located near side toward the farther point by the stated travel")]

PRACTICES = ("Give a fresh point near the hand, a direction and a travel to shove a free thing.\n"
             "Trust stopped and pushed_mm: lost_contact means it turned away or tipped; force means it resisted.\n"
             "look after a push; the result's next_stroke_mm says how far the next one may go before the thing's "
             "place is too uncertain to push again without a look")
