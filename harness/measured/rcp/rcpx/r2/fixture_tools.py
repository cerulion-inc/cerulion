# SPDX-License-Identifier: AGPL-3.0-only
"""Tool table and practices of package fixture (SPEC-R4 B5). See rcpx/r2/fixture.py.

One tool, no typed range: every end (speed, force, torque, travel, time) is derived in the session and named in the
result with its source; a tool schema carries no derived end (SPEC-R4 A5).
"""

from rcpx import surfaces as S

POINT = {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
         "description": "a point [x, y, z] in base frame mm"}
PIXEL = {"type": "object", "properties": {"look": {"type": "string"},
                                           "px": {"type": "array", "items": {"type": "number"}, "minItems": 2,
                                                  "maxItems": 2}},
         "required": ["look", "px"], "additionalProperties": False,
         "description": "a pixel [u, v] of a look you were shown: anywhere on the part (its handle, knob or "
                        "face; a door anywhere on its leaf, either face)"}
TARGET = {"anyOf": [{"type": "string"}, POINT, PIXEL],
          "description": "the part to work: a pixel of a look, a located label of robot://world, or a point"}
UNTIL = {"anyOf": [{"type": "string", "enum": ["end"]},
                   {"type": "object", "properties": {"mm": {"type": "number"}}, "required": ["mm"],
                    "additionalProperties": False},
                   {"type": "object", "properties": {"deg": {"type": "number"}}, "required": ["deg"],
                    "additionalProperties": False}],
         "description": "end (default: until the part stops), {mm: n} for pull and push, {deg: n} for turn"}
CLEAR = {"anyOf": [{"type": "boolean"}, {"type": "object", "properties": {"look": {"type": "string"}},
                                           "required": ["look"], "additionalProperties": False}],
         "description": "true after your own look when you judge the way clear, or {look: id}: recorded in the "
                        "result; it changes no speed (the call runs at the robot's own speed, the contact stop "
                        "armed, with it or without it)"}

TOOLS = [
    S._tool("work_part",
            "Work a fixed part in one step: pull or push a drawer, a door or a slider by its handle or face, or "
            "turn a knob or a lever. It takes hold with a posture whose stroke stays clear of what it knows, finds "
            "the part's own path by a small probe (a line or an arc), follows it, lets go, and says how far the part "
            "moved, along what, from what force and why it ended. A door's leaf is pushed square to the face the "
            "look shows; a leaf that turns away from the finger can be left part open.",
            S._obj({"target": TARGET,
                    "way": {"type": "string", "enum": ["pull", "push", "turn"],
                            "description": "pull it toward the robot, push it away, or turn it"},
                    "until": UNTIL,
                    "sense": {"type": "string", "enum": ["cw", "ccw"],
                              "description": "for turn: as the hand sees the part; left out, the one that gives"},
                    "grasp": {"type": "string", "enum": ["auto", "across", "pinch", "hook", "fingers"],
                              "description": "how to take hold (default auto: the clearest posture)"},
                    "direction": {"anyOf": [{"type": "string", "enum": ["up", "down", "forward", "back", "left",
                                                                           "right"]}, POINT],
                                  "description": "the way the part moves (for turn: the way the hand points), a "
                                                 "name or [x, y, z] of the base frame; left out, read from its "
                                                 "stored path, its face or the line to the base"},
                    "max_n": {"type": "number",
                              "description": "a stated limit, newtons: the most this part may see (the way in "
                                             "brakes for it, each pad squeezes its share, the stroke's ceiling); "
                                             "the limit block says kept or PASSED. A limit you type is a "
                                             "request, not a guarantee"},
                    "gentle": {"type": "boolean",
                               "description": "the hand's weight as a stated limit on the part, kept as max_n "
                                              "is; slower where a contact can begin (no look: all of it): what a "
                                              "stated limit costs here. A limit you type is a request, not a "
                                              "guarantee: the limit block says kept or PASSED"},
                    "clear": CLEAR}, ["target", "way"]),
            {"target": {"look": "L1", "px": [412, 300]}, "way": "pull", "clear": True},
            "pull the part you pointed at in look L1 until it stops (your word that the way is clear is recorded)"),
]

PRACTICES = ("work_part's ended: end, the part stopped moving under the stroke (a stop or a stiff point); "
             "did_not_move, it held against the whole push; nothing_felt, the stroke met no stop it could tell "
             "(its readings are said); not_taken, no hold was made; not_shown, the part's motion is unread. A "
             "stroke back on a part just worked reuses its stored path.")
