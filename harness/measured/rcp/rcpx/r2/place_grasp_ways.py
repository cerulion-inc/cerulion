# SPDX-License-Identifier: AGPL-3.0-only
"""The candidates of a take that lie elsewhere on the thing than the pixel.

place_grasp.choose tries the whole thing at its seen middle first (wherever on it the pixel lies), then its wall at
the pixel and its part through the pixel. When none of those gives a plan, cands() adds every other way the look
shows of taking the SAME thing (rcpx/r2/see_grasps.py: the rim nearest the pixel wherever the pixel lies on it,
the narrowest chords that fit the opening anywhere on it), nearest the pixel first; the open jaws at the point
itself stay the last candidate, for a thing of which nothing fits. Each candidate's words say how far it lies from
the pixel. Nothing here reads ground truth or names a robot. Lengths m.
"""

import math

from rcpx.r2.place_grasp import MM


def whole(narrow, both, max_m, want="auto"):
    """True when the thing is taken WHOLE at its seen middle: its narrow side, its error twice and the
    hand's band twice (both) are under the opening max_m, or the operator asked for a grasp from above. Where on
    the thing the pixel lies is no part of this rule: a pixel on its side, low on it or at its edge takes the
    thing as one at its top does."""
    return narrow is not None and want in ("auto", "top") and (float(narrow) + float(both) < float(max_m) or want == "top")


def cands(ex, seen, S, J, F, fixed=None):
    """[(strategy, words, centre, [(yaw, width)], top, depth cap)] in place_grasp.choose's own form, nearest the
    pixel first: the ways of see_grasps.ways whose width, its error twice and the hand's band twice are under the
    opening. A rim among them is the thing's wall read from its top (both faces of the wall show there) and
    centred where the pads meet it: place_grasp.choose takes it ahead of the wall and the part it read at the
    pixel itself. fixed: the jaw bearing the operator stated, or None."""
    from rcpx.r2 import see_grasps
    both = F["err"] + F["err"] + F["band"] + F["band"]
    out = []
    fall = S.get("depth_inside")
    reach = min(x for x in (J.pad_height_m, J.finger_len_m, fall) if x is not None) if any(
        x is not None for x in (J.pad_height_m, J.finger_len_m, fall)) else None
    for w in see_grasps.ways(ex, seen, S["centre"], reach):
        if not w["width_m"] + both < J.max_m:
            continue
        away = w["from_pixel_m"] * MM
        a = float(w["bearing_rad"])
        opts = [(a, w["width_m"]), (a + math.pi, w["width_m"])] if fixed is None else [(float(fixed), w["width_m"])]
        if w["kind"] == "rim":

            def inside(open_m, fall=w["inside_m"], pad=J.pad_height_m):
                """As place_grasp's rim: the pads' own height under the rim, no deeper than the inside is seen."""
                known = [x for x in (pad, fall) if x is not None]
                return (min(known), "the pads' own height under its rim, no deeper than its inside is seen") \
                    if known else None
            out.append(("rim", f"a grasp of the rim of the thing your point lies on, where it is nearest your point "
                        f"({away:.0f} mm from it), the jaws along the wall's inward direction, one finger inside",
                        w["centre_xy"], opts, w["top_z"], inside))
        else:
            out.append(("part", f"a grasp of the narrowest part of the thing your point lies on that fits the jaws, "
                        f"{away:.0f} mm from your point, across its narrowest chord there", w["centre_xy"], opts,
                        w["top_z"], None))
    return out
