# SPDX-License-Identifier: AGPL-3.0-only
"""Tool table and practices of package grasp. See rcpx/r2/grasp.py; the robot://body patch is in
grasp_hooks.py."""

from rcpx import surfaces

_S = surfaces


def _num(text):
    """Numeric argument schema with no derived upper end."""
    return {"type": "number", "minimum": 0, "description": text}


def _vec(text):
    """Three coordinate numbers with no arbitrary range."""
    return {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3, "description": text}


_FORCE_FRAC = {"type": "number", "minimum": 0, "maximum": 1,
               "description": ("the share of the jaw servo's own force range to squeeze with; leave it out and "
                               "the jaws squeeze with the least that stalls them on the part and raise "
                                  "it only after a slip")}
_WIDTH = _num("width at the jaw tips in mm, at most the jaws' opening (robot://body grasp.max_opening_mm)")
_CLEAR = {"anyOf": [{"type": "boolean"}, {"type": "object", "properties": {"look": {"type": "string"}},
          "required": ["look"], "additionalProperties": False}],
          "description": "you looked and judge the way clear; the contact stop remains armed"}


TOOLS = [
    _S._tool("grasp",
             "Close the gripper on the part between its open jaws with a bounded squeeze; returns holding (the jaws "
             "stopped short of closed and press on something), width_mm (the opening at the jaw tips), "
             "closed_on_object and squeeze_n_est; while holding, every later result carries grasp {held, "
             "width_change_mm} and says slipped_out if the part is lost.",
             _S._obj({"width_mm": _WIDTH,
                      "force_frac": _FORCE_FRAC,
                      "max_n": _num("the most the part may take, in newtons (your stated limit): the jaws then "
                                    "close no faster at the touch than keeps it and squeeze no harder; leave it "
                                    "out and they close at their own rate")}),
             {"width_mm": 35},
             "close on the stated width and say if it holds"),
    _S._tool("release",
             "Open the gripper and let go of a held part: fully, or only to width_mm at the jaw tips; returns "
             "was_holding and the width reached.",
             _S._obj({"width_mm": _WIDTH}),
             {"width_mm": 60}, "open the jaws to 60 mm, letting go of whatever they held"),
    _S._tool("impulse",
             "Strike along a checked stroke at the speed the part may take; returns its force budget, "
             "arrival speed, peak estimate or unresolved peak with strike energy, contact and base movement.",
             _S._obj({"direction": _S._direction("which way to strike"),
                      "travel_mm": _num("how far the tip may travel in mm: your word"),
                      "speed_mm_s": _num("leave it out: the robot strikes as fast as the part may take "
                                         "(the result says the bound and what would lift it)"),
                      "force_n": _num("the peak the part may take, in newtons: your word"),
                      "clear": _CLEAR,
                      "frame": _S._str("frame the direction is in: base or tool", ["base", "tool"])},
                     ["direction", "travel_mm"]),
             {"direction": "up", "travel_mm": 6, "frame": "base"}, "strike upward along the stated travel"),
    _S._tool("pinch",
             "Put the moving jaw's tip on a part (base frame mm, between the open jaws) with the jaws "
             "open around it, then close by close_frac of the opening so the part is squeezed toward the other jaw "
             "and the force stays in the hand, not the base; returns closed_mm, gave and met_resistance, and ends "
             "with the jaws eased (still, not squeezing).",
             _S._obj({"part_point_mm": _vec("where the moving jaw's tip goes, [x, y, z] base mm"),
                      "close_frac": {"type": "number", "minimum": 0, "maximum": 1,
                          "description": "share of the opening to close; leave it out to stall "
                                          "at the least squeeze"},
                      "force_frac": _FORCE_FRAC,
                          "hold_s": _num("keep closing for this time after stillness; "
                                        "leave it out to end when still")}, ["part_point_mm"]),
             {"part_point_mm": [240, -20, 95]},
             "set the moving jaw on the part at the stated point and close until the jaws stall on it"),
    {"extends": "reach_check",
     "args": {"force_n": _num("the push force you plan to use there, in newtons")},
     "note": "With a measured base give (measure_base_give) it adds base_limited: true when the base would walk "
             "before that force (or, without force_n, the most the arm may push there) is reached, and the modes "
             "that do not hold a steady load on the base."},
]

PRACTICES = ("grasp says whether it holds (gripper does not), and later results carry grasp {held, "
             "width_change_mm}; slipped_out means the part is gone. When reach_check says base_limited, a steady "
             "press walks the base: impulse is short and fast, pinch keeps the force between the jaws. release's "
             "hint says how wide the jaws can open: the part, its error and what a hinged jaw sweeps.")
