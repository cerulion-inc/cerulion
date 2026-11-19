# SPDX-License-Identifier: AGPL-3.0-only
"""Tool table and practices of package place, version two. See rcpx/r2/place.py.

A spot is {"look": id, "px": [u, v]} (a pixel on the thing or the place, in a look the operator was shown),
a located label, or [x, y, z] in base frame mm. No schema carries a range: what the arm cannot do is refused by the
check that knows why (reach, the jaws' opening, the space book), with its bound named in the refusal. The texts say
what the robot does; they state no speed, force or length, because the robot derives each one per session
(robot://body self) and reports the binding one in every result.
"""

_NUMBER = {"type": "number"}


def _point(text):
    return {"type": "array", "items": dict(_NUMBER), "minItems": 3, "maxItems": 3, "description": text}


def _text(text, enum=None):
    out = {"type": "string", "description": text}
    if enum is not None:
        out["enum"] = list(enum)
    return out


def _flag(text):
    return {"type": "boolean", "description": text}


def _obj(props, required=()):
    return {"type": "object", "properties": props, "required": list(required), "additionalProperties": False}


_PIXEL = _obj({"look": _text("the id of a look you were shown (a scene look or a wrist look)"),
               "px": {"type": "array", "items": dict(_NUMBER), "minItems": 2, "maxItems": 2,
                      "description": "the pixel [u, v] in that look"}}, required=("look", "px"))


def _spot(text):
    return {"anyOf": [_PIXEL, {"type": "string"}, {"type": "array", "items": dict(_NUMBER), "minItems": 3,
                                                    "maxItems": 3}],
            "description": text + ": {look, px} (a pixel on it in a look you were shown), a located label, "
                                  "or [x, y, z] in base frame mm"}


_PLAIN = {"anyOf": [{"type": "string"}, {"type": "array", "items": dict(_NUMBER), "minItems": 3, "maxItems": 3},
                    _PIXEL]}
_POINT = dict(_point("a point [x, y, z] in base frame mm (the older way to say a spot)"), **{"x-retired": True})
_LABEL = dict(_text("a label robot://world holds (the older way to say a spot)"), **{"x-retired": True})
_CORNERS = {"type": "array", "items": _PLAIN, "minItems": 2, "maxItems": 2, "x-retired": True,
            "description": "two opposite corners of the opening or the marked area it goes into (each a spot); the "
                           "thing goes to the middle"}
_RIM = {"type": "array", "items": _PLAIN, "minItems": 1,
        "description": "the corners or rim of the opening it goes into (spots on the rim itself, not on the thing "
                       "that carries the opening): the thing goes in over their middle"}
_IN = {"anyOf": [_PIXEL, {"type": "string"}, {"type": "array", "items": _PLAIN, "minItems": 1}],
       "description": "the opening it goes into: one spot inside it ({look, px} or a label), or the corners or rim "
                      "of the opening it goes into (a list of spots)"}
_CLEAR = {"anyOf": [{"type": "boolean"}, _obj({"look": _text("the look you judged the way clear in")},
                                               required=("look",))],
          "description": "your word that the way is clear, after you looked: true (your newest look) or {look: id}. "
                         "It is recorded in the result and changes no speed: with it or without it the robot "
                         "moves at its own speed with the contact stop armed"}
_WIDTH = dict(_NUMBER, description="the thing's width across the jaws in mm, when you know it (optional)")
_YAW = dict(_NUMBER, description="the bearing in the base frame, degrees from x toward y, the jaws close along "
                                 "(optional: the robot chooses it from the outline it sees)")
_GRASP = _text("how to take it (optional: the robot chooses from what it sees and says which)",
               enum=("auto", "top", "rim", "side"))
_MORE_CLEAR = dict(_NUMBER, description="more room than the robot derives between the carried thing's bottom and "
                                        "what it knows under the way, mm (optional)")
_RELEASE = _text("drop: let go above the spot instead of lowering until it touches down; the result states the "
                 "drop height and energy (optional)", enum=("drop",))
_RELEASE_MM = dict(_NUMBER, description="the older way to ask for a drop: let go with the thing's bottom this high "
                                        "above the surface, mm", **{"x-retired": True})
# a limit only a model typed is a request
_TYPED = "A limit you type is a request, not a guarantee: the result says kept or PASSED with its figure."
_GENTLE = _flag("a stated limit of the hand's weight on what the call touches; it slows only where a contact "
                "can begin, and the result says kept or PASSED and limit.cost. " + _TYPED)
_GENTLE_PRICED = _flag("a stated limit of the hand's weight on what the call touches: where a contact can begin "
                       "(where no look shows the way free, the carry too) the tool, the held thing and every link run "
                       "at the speed whose stop stays under it, each pad presses its share: what a stated limit costs "
                       "here. " + _TYPED)
_FIRM = _flag("the place may be pressed as hard as the robot's own ratings allow")
_MAX_N = dict(_NUMBER, description="a stated limit, newtons, kept as gentle is; a budget refusal names the word, "
                                   "its price and the least max_n whose plan fits. " + _TYPED)
_EXPECT = _text("contact: contact is meant at the end of the move", enum=("contact",))
_ITEM = _obj({"pick": _spot("the thing to pick"), "to": _spot("where it goes"), "on": _spot("the spot or the "
              "thing it is set down on"), "in": _IN, "grasp": _GRASP, "width_mm": _WIDTH, "jaw_yaw_deg": _YAW,
              "release": _RELEASE}, required=("pick",))
_INTENT = {"clear": _CLEAR, "gentle": _GENTLE, "firm": _FIRM}


def _tool(name, description, args, example_args, means, **extra):
    entry = {"name": name, "description": description, "args": args,
             "example": {"args": example_args, "means": means}}
    entry.update(extra)
    return entry


TOOLS = [
    _tool("pick",
          "Pick up a free thing with the gripper: a pixel on it names the thing; the robot takes it (its middle, or "
          "its narrowest part there that fits the jaws), says how far the grasp is from your pixel, closes, lifts "
          "and judges the hold from the jaws and the weight it feels. Returns held (true or false) with the "
          "evidence and a sentence first; a thing it cannot take is refused with the reason.",
          _obj({"pick": _spot("the thing to pick"), "point_mm": _POINT, "label": _LABEL,
                "approach": {"anyOf": [{"type": "string"}, {"type": "array", "items": dict(_NUMBER), "minItems": 3,
                                                             "maxItems": 3}],
                             "description": "the way the hand travels as it arrives (default down): a direction "
                                            "name or [x, y, z]"},
                "width_mm": _WIDTH, "jaw_yaw_deg": _YAW, "grasp": _GRASP,
                "touch": _flag("allow one touch of the support when the looks cannot tell how low the thing is"),
                "standoff_mm": dict(_NUMBER, description="more room than the robot derives before the thing, mm "
                                                         "(optional)", **{"x-retired": True}), **_INTENT}),
          {"pick": {"look": "C1", "px": [412, 233]}, "clear": True},
          "pick up the thing at pixel (412, 233) of scene look C1; you looked and judge the way clear"),
    _tool("carry",
          "Carry the held thing over a spot: the robot rises over what it knows on the way, crosses and comes "
          "down to just above the spot at its own speed, and stops when anything pushes on the thing from any "
          "side or the jaws lose it (stopped dropped, and where). Refused when nothing is held.",
          _obj({"to": _spot("the spot it goes over"), "to_point_mm": _POINT, "to_label": _LABEL,
                "clearance_mm": _MORE_CLEAR, "expect": _EXPECT, **_INTENT}),
          {"to": {"look": "C1", "px": [250, 301]}, "clear": True},
          "carry the held thing over the spot at pixel (250, 301) of scene look C1"),
    _tool("place",
          "Set the held thing down on a spot or a thing (on), or into an opening (in). A pixel on a thing sets it "
          "on the free spot nearest the pixel where it stands fully on one level the look shows (of two tops, "
          "the nearer); a pixel on a hollow thing or an opening, inside it. It lowers the thing until it touches "
          "down, lets go and rises; into an opening narrower than the hand it lets go above the rim and says from "
          "how high. A place under a cover is entered level through an open side, else refused with what blocks "
          "each side. A place inside an opening no look shows needs a look that shows it (a look costs 0 "
          "control periods; a wrist look from a pose facing it costs the move). Refused when nothing is held.",
          _obj({"on": _spot("the spot or the thing it is set down on"), "in": _IN, "to": _spot("where it goes"),
                "on_point_mm": _POINT, "on_label": _LABEL, "in_corners": _CORNERS, "in_rim": _RIM,
                "release": _RELEASE, "release_mm": _RELEASE_MM, "clearance_mm": _MORE_CLEAR, "expect": _EXPECT,
                **_INTENT}),
          {"on": {"look": "C1", "px": [250, 301]}, "clear": True},
          "set the held thing down on the spot at pixel (250, 301) of scene look C1",
          rename={"in": "into"}),
    _tool("put",
          "The whole step in one call: pick the thing you point at and set it down where you point (to), or "
          "several things in order (items); the take and the set down are pick's and place's. The result says in "
          "one sentence what happened, then per phase what it did and how many control periods it used, what is "
          "left of the budget, and the bound that set the pace. It stops at the first thing that fails and says "
          "what is held.",
          _obj({"pick": _spot("the thing to pick"), "to": _spot("where it goes: a spot on a surface, a thing to "
                "set it on, or a hollow thing or opening to put it into"), "on": _spot("the spot or the thing it "
                "is set down on"), "in": _IN,
                "items": {"type": "array", "items": _ITEM, "minItems": 1,
                          "description": "several things in order, each {pick, to}; they share one budget"},
                "pick_point_mm": _POINT, "pick_label": _LABEL, "on_point_mm": _POINT, "on_label": _LABEL,
                "in_corners": _CORNERS, "in_rim": _RIM, "width_mm": _WIDTH, "jaw_yaw_deg": _YAW, "grasp": _GRASP,
                "release": _RELEASE, "release_mm": _RELEASE_MM, "clearance_mm": _MORE_CLEAR, "expect": _EXPECT,
                "max_n": _MAX_N, **_INTENT, "gentle": _GENTLE_PRICED}),
          {"pick": {"look": "C1", "px": [412, 233]}, "to": {"look": "C1", "px": [250, 301]}, "clear": True},
          "pick the thing at pixel (412, 233) of scene look C1 and set it down at pixel (250, 301); you looked "
          "and judge the way clear",
          rename={"in": "into"}),
]

PRACTICES = ("A pixel ON a thing names the thing. A pixel on the bare support names a thing seen within the "
             "spot's own stated error of it (the receipt says so, with the distance); where none or two are, the "
             "tool does not move (0 control periods) and answers with what the look shows near the point. held "
             "comes from the jaws and the weight, placed from where the jaws opened and the same view before and "
             "after.")
