# SPDX-License-Identifier: AGPL-3.0-only
"""Pull and twist schemas expose intent; the session supplies every derived end."""

from rcpx import surfaces as S

POINT = {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
         "description": "three base frame coordinates"}
TARGET = {"anyOf": [{"type": "string"}, {**POINT, "description": "[x, y, z] mm"}],
          "description": "a located handle or part: world id, fresh label, or base frame point in mm"}
WIDTH = {"type": "number", "description": "width across the jaws in mm, if known; each jaw adds its own clearance"}
SIDE = {"anyOf": [{"type": "string", "enum": ["horizontal", "vertical"]}, POINT],
        "description": "horizontal, vertical, or a base frame direction"}
DIRECTION = {"anyOf": [{"type": "string", "enum": ["forward", "back", "left", "right", "up", "down"]}, POINT]}
CLEAR = {"anyOf": [{"type": "boolean"}, {"type": "object", "properties": {"look": {"type": "string"}},
           "required": ["look"], "additionalProperties": False}],
         "description": "true after your own look when you judge the way clear, or {look: id}: recorded in the "
                        "result; it changes no speed"}

TOOLS = [
    S._tool("pull", "Grasp a located handle and draw it out along the way the part gives, following measured "
            "motion and side force, straight paths only; stops when the measured direction turns beyond its "
            "relative fit error, at travel, resistance, loss of hold or a guard, then lets go.",
            S._obj({"target": TARGET, "travel_mm": {"type": "number", "description": "path travel in mm; omitted: "
                    "the stored end, or until the part stops; RCP says the source"},
                    "max_n": {"type": "number", "description": "end force in N; omitted: the derived ceiling with its source"},
                    "width_mm": WIDTH, "way": DIRECTION,
                    "along": {**SIDE, "description": "the handle's long side, horizontal when omitted"},
                    "clear": CLEAR}, ["target"]),
            {"target": "handle"}, "grip the located handle, follow until its end, then let go"),
    S._tool("twist", "Grasp or lightly pinch a located part and turn about the tool axis under a derived torque "
            "ceiling; positive is counter-clockwise looking toward the part about the reversed pointing axis; "
            "checks the whole turn against the body model; stops at angle, resistance, loss of hold or a guard, then lets go.",
            S._obj({"target": TARGET, "angle_deg": {"type": "number", "description": "finite nonzero signed turn in degrees, within one full turn"},
                    "max_nm": {"type": "number", "description": "end torque in N m; omitted: the joints' torque room with its source"},
                    "grip": {"type": "string", "enum": ["close", "pinch"], "description": "close squeezes; pinch rests lightly"},
                    "width_mm": WIDTH, "approach": DIRECTION,
                    "across": {**SIDE, "description": "jaws' closing bearing; omitted: halfway level then level"},
                    "clear": CLEAR}, ["target", "angle_deg"]),
            {"target": "part", "angle_deg": 1}, "grip the located part, turn a stated angle, then let go"),
]

PRACTICES = ("pull draws a held handle out along the way the part gives; twist turns a gripped part about the "
             "tool axis; both stop when the part stops moving (a stop or a stiff point) and let go. width_mm and way "
             "for pull, approach and across for twist, when you know them.")
