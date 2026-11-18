# SPDX-License-Identifier: AGPL-3.0-only
"""What the robot's own camera shows between its jaws.

    width_between(ex) -> {"m", "sigma_m", "look", "source"} or {"m": None, "why"} or None

One picture of the hand camera as the hand stands (no motion, no control period; it is no look of the session: the
operator cannot point in it and nothing is noted of it in the space book). With depth: the points the picture shows
BETWEEN the jaws (in the tool frame: along the closing axis between the two jaw faces as they stand, across it
within a pad's own width, along the tool no further from the tip plane than the fingers are long), the robot's own
body left out, that stand over the lowest level seen there by more than BAND of the depth's own error (the thing the
fingers stand around, not what it stands on), and whose four neighbour pixels agree with it in depth (a pixel at the
outline that mixes the thing's depth with what lies behind it stands in the air beside the thing; one pixel's
footprint is given back at each side). A thing that fills the space between the jaws down past what the camera
sees is read WIDE (on the bridge a round thing 62 mm across read 68.7 mm: bits of the fingers stand over its own
top); read by its own top it read 53.4 mm, under the truth, where the fingers hide the top's edges, and an
under-read makes the pads meet it at the jaws' own rate, so the wide reading stands. Its width is their extent
along the closing axis, an UPPER reading ("upper": True): its error (two edges of one pixel's footprint at that
range and the depth's own error, in quadrature) bounds how much wider the thing can be, not how much narrower (the
outline and the fingers read wide). A close arrives from that width plus its band, which an over-read only makes
longer. Where the points reach a jaw face the thing is at least that wide and the answer says so (the jaws stand on
it already: nothing to run to).

Without depth no width is given: the range from the hand's own geometry (the pads' plane) says where a pixel would
lie IF it lay on the thing, but which pixels are the thing's and which the support's a colour picture does not say
without a threshold on colour, and none is typed here: the answer is None with that reason, and the close prices the
arrival rate instead.

Clouds are read only through the frozen functions of rcpx/r2/see_spot.py (cloud_of, view_of, hand_of). Nothing here
reads ground truth or names a robot. Lengths m.
"""

import math

import numpy as np

from rcpx.bounds_ratios import BAND, QUANT

CAMERA = "wrist"
NO_DEPTH = ("the hand camera gives no depth: a colour picture alone does not say which pixels between the jaws are "
            "the thing's without a typed threshold")


def _none(why):
    return {"m": None, "sigma_m": None, "look": None, "source": None, "why": why}


def width_between(ex):
    """The width of what stands between the jaws, as the hand camera shows it now (module text)."""
    from rcpx.r2 import see_spot
    J = getattr(ex, "_pl_jaws", None)
    if J is None:
        from rcpx.r2.place_hand import Jaws
        J = Jaws(ex.model)
    s = ex._sense()
    q, _ = J.reading(s.gripper, ex._gripper_cmd)
    if q is None:
        return _none("the jaws give no reading")
    keeps = getattr(ex, "_look_keeps", None)
    if not callable(keeps):
        return _none("this session's core keeps no looks")
    rec = {**keeps(s, CAMERA), "q": np.asarray(s.q, dtype=float).copy(), "camera": CAMERA}
    if rec.get("cam") is None:
        return _none("this body has no hand camera")
    if rec.get("depth") is None:
        return _none(NO_DEPTH)
    got = see_spot.cloud_of(ex, rec)
    if got is None:
        return _none(rec.get("_depth_problem") or NO_DEPTH)
    view, P, valid, _ = got
    frac = float(J.frac_of_q(q))
    tip, R = np.asarray(s.tip, dtype=float), np.asarray(s.R, dtype=float)
    pts = np.asarray(J.points_tool(frac), dtype=float)             # the jaw points in the tool frame
    a_j = sorted(float(p @ J.axis_T) for p in pts)                 # the jaw faces along the closing axis
    if not a_j[-1] - a_j[0] > 0.0:
        return _none("the jaws stand closed")
    mid = tip + R @ (J.axis_T * (a_j[0] + a_j[-1]) / 2)            # the middle between the jaws, on the tip plane
    x, y, z = view.project(mid)
    if not (z > 0.0 and view.inside(x, y)):
        return _none("the hand camera does not see between the jaws as they stand")
    T = (P - tip) @ R                                              # every point in the tool frame
    a, b, along = T @ J.axis_T, T @ J.cross_T, T[..., 0]
    hand = see_spot.hand_of(ex)
    reach = float(J.finger_len_m or J.pad_height_m or hand["finger_m"])
    between = valid & (a > a_j[0]) & (a < a_j[-1]) & (np.abs(b) <= hand["pad_m"] / 2) & (np.abs(along) <= reach)
    if int(between.sum()) < int(QUANT * QUANT):
        return _none("the hand camera shows nothing between the jaws (the robot's own body left out)")
    rng = np.linalg.norm(P[between] - view.pos, axis=-1)
    far = float(np.median(rng))
    foot = far / view.f
    sigma_d = float(see_spot._sigma_at(rec, far, foot)[0])
    # the thing the fingers stand around: what stands nearer the hand than the furthest level seen between the
    # jaws (what it stands on) by more than BAND of the depth's own error and one pixel
    floor = float(np.max(along[between]))
    thing = between & (along < floor - max(BAND * sigma_d, foot))
    if int(thing.sum()) < int(QUANT * QUANT):
        return _none("the hand camera shows one level between the jaws: no thing stands out of it")
    # a pixel at the thing's outline whose depth mixes the thing's and what lies behind it stands in the air beside
    # the thing: kept only where each of its four neighbour pixels lies within BAND of the depth's own error and
    # one pixel's footprint of it along the ray; the outline's own pixel is given back as one footprint a side
    rng_img = np.linalg.norm(P - view.pos, axis=-1)
    tol = BAND * math.hypot(sigma_d, foot)
    solid = thing.copy()
    for shift, axis in ((1, 0), (-1, 0), (1, 1), (-1, 1)):
        solid &= np.roll(valid, shift, axis=axis) & (np.abs(rng_img - np.roll(rng_img, shift, axis=axis)) <= tol)
    edge = 0.0
    if int(solid.sum()) >= int(QUANT * QUANT):
        thing, edge = solid, foot
    lo, hi = float(np.min(a[thing])) - edge, float(np.max(a[thing])) + edge
    sigma = math.hypot(math.hypot(foot, foot), sigma_d)
    touches = (lo - a_j[0] <= foot) or (a_j[-1] - hi <= foot)
    return {"m": hi - lo, "sigma_m": sigma, "look": f"{CAMERA} at t {float(s.t):.2f} s",
            "source": (f"depth picture of the hand camera ({view.w} px; one pixel is {foot * 1000.0:.1f} mm at "
                       f"{far * 1000.0:.0f} mm): what stands between the jaws over what it stands on, an upper "
                       "reading (it may stand narrower by more than its error: its outline and the fingers read wide)"
                       + ("; it reaches a jaw as they stand: at least that wide" if touches else "")),
            "points": int(thing.sum()), "reaches_a_jaw": bool(touches), "upper": True}
