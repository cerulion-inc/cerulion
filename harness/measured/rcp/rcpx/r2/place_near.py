# SPDX-License-Identifier: AGPL-3.0-only
"""Package place_near: what a pixel
NAMES. No tools of its own and no motion: rcpx/r2/see_spot.py calls it for every pixel of a look with depth, and
rcpx/r2/place.py for the result of a call that is answered.

A pixel names the THING it lies on. A pixel whose own depth shows a level (bare support) names the ONE thing whose
seen pixels lie within the spot's own stated error ACROSS THE VIEW of it: the error the look states at that range
(half a pixel, and how far the camera moved while the look was read), measured in the picture, where it is known
exactly: from the pixel's middle to the nearest pixel of the thing (each a square of one footprint). No radius is
typed. With a still camera that is a thing with a pixel next to the pointed one along a row or a column. A pixel
next to a jump of the depth (its own slope is not read there) lies on the level beside it when it stands at that
level's height, and is read the same way (beside()).

When no thing lies that near, or two do, a take (roles pick and part) has nothing to close on. The tool then
ANSWERS without moving (spot["answer"]): what the same look shows near the pointed ray, each thing with a pixel
that lies on it, so that the next call can name one. It answers only where "the look shows empty" HOLDS:
  (i)   the look is of the scene as it stands: taken at or after the last act that changed the scene (scene_t);
  (ii)  the pixel is no edge pixel (no other surface within one pixel): no jump of the range to a surface within
        one pixel, and no thing the look shows with a pixel that touches the pointed one (at a corner: a thing
        along a row or a column lies within the error and is named);
  (iii) BAND times the look's stated depth error at that range is under the lowest thing this hand can take
        (the pads' own height, a body fact): a thing the hand could take would stand out of the level;
  (iv)  the level the pixel lies on is the stated support or lies under it (a drawn surface under the plane the
        hand meets): a level that stands OVER the stated support by more than its stated error and the depth's
        band is the top of something wider than this hand reads, and that is no empty place;
  (v)   no ONE thing the look shows stands around the pixel on every side (no quarter turn of the picture around
        it is free of that thing): a level closed in by a thing is the floor inside it, or a surface seen
        through it (a pane the depth does not show), not bare support.
Where one of them fails the tool ACTS on the point with the contact stop armed and the spot's doubt names
the condition that failed. A typed point is read the same way from the newest look that shows it (shown()). An
answer carries the look's help: the look's own picture with the pixel and each "near" entry marked, and a
crop around the pixel (rcpx/control_looks.pointed_pictures); answered() puts both in the receipt.

A PLACE (roles place and into) is read by the same rule where the pixel lies on the stated support: the ONE
thing with a pixel within the spot's own error names it, and the spot says that the pixel itself lay on bare
support. Where none does, or two do, the pixel is the point itself on that level, and spot["near"] lists
what the look shows near it, each thing with a pixel on it. A place pixel on a level that stands over the stated
support lies on that level, the top of something: no thing beside it is named. A place is never answered here.

    beside(P, valid, view, u, v, up, hand, sigma_m)  the things a look shows around a pixel that lies on a level
    name(cloud, u, v, up, hand, role, e, sigma, moved, support)    -> (extent, names, empty)
    answer(ex, rec, key, fixed, cloud, up, hand, out, empty, sigma, px)  sets out["answer"] with the look's help,
                                                     or the doubt that says why the tool acts
    over_support(base_z, support, sigma, foot)       how far a level stands over the stated support, or None
    shown(ex, p_m, role)                             what the newest look that shows a typed point shows there
    answered(ex, ans, tool, nth)                     the result of a call that was answered: nothing moved
    scene_t(ex)                                      the time of the last act that changed the scene, or None
Nothing here reads truth or names a robot or a thing.
"""

import cv2
import numpy as np

from rcpx.bounds_ratios import BAND, QUANT
from rcpx.r2 import see_cloud as SC

MM = 1000.0          # unit
ON = "the pixel lies on it"
WITHIN = "within the spot's own error"
AROUND = "it closes the pixel's level in on every side"      # role part: a pane the depth does not show
NO_LEVEL = "no level surface of a pad's size is seen below the pixel, nor at its own height beside it"
PLACES = ("place", "into")     # the roles of a place: a pixel never answered, named by the same rule on the support


def over_support(base_z, support, sigma, foot):
    """How far (m) a level at base_z stands over the stated support (support: (height, its stated error or None))
    by more than its stated error and the depth's band (condition iv), or None when it lies on the support or
    under it."""
    sup_z, sup_err = support
    over = float(base_z) - float(sup_z)
    return over if over > max(BAND * sigma, foot) + float(sup_err or 0.0) else None


def scene_t(ex):
    """The time (s, the session's clock) of the last control period in which the scene was acted on: a guard read
    a load, or the jaws were commanded to move (ex._scene_t, written by the core). Until
    the core writes it: this package's own reading of the same two events (place_motion.period_tick). None:
    nothing has acted on the scene."""
    t = getattr(ex, "_scene_t", None)
    if t is None:
        t = (ex.state.get("place") or {}).get("scene_t")
    return None if t is None else float(t)


def _low_m(ex, hand):
    """The lowest thing this hand can take: the pads' own height (place_hand.Jaws); an arm with no pads: the
    hand's own finger depth as see_spot.hand_of gives it."""
    st = ex.state.setdefault("place_near", {})
    if "low_m" not in st:
        try:
            from rcpx.r2 import place_hand
            low = place_hand.Jaws(ex.model).pad_height_m
        except Exception:
            low = None
        st["low_m"] = float(low) if low else float(hand["finger_m"])
    return st["low_m"]


def beside(P, valid, view, u, v, up, hand, sigma_m):
    """The things a look shows around pixel (u, v) when that pixel's OWN depth lies on a level (it is read as it
    is: no nearer surface within one pixel is taken for it); None when the pixel shows a thing, has no depth or
    shows no level. {"extent": see_cloud's extent of the pixel as read (kind "bare"), "foot_m", "things": [{"d_px":
    from the pixel's middle to the thing's nearest pixel square, in the camera's pixels, "next_px": that pixel,
    "px": the pixel of the thing that lies deepest inside its seen shape (nearest the pointed one among equals),
    "points", "around": whether its pixels stand on every side of the pointed one in the picture}], nearest first}. A thing is what stands over the level in one piece (see_cloud's own rule) and
    shows at least QUANT by QUANT pixels (the points it takes to tell a size across both ways)."""
    out, S = SC._resolve(P, valid, view, u, v, np.asarray(up, dtype=float), hand, sigma_m, snap=False, beside=True)
    if S is None or out.get("kind") != "bare":
        return None
    lab, k = S["lab"], S["k"]
    u0, _, v0, _ = S["win"]
    vv, uu = np.nonzero(lab)
    things = []

    def cam(col, row):          # a pixel of the window read, in the camera's own pixels
        return [int(S["ou"] + (col + u0) * k), int(S["ov"] + (row + v0) * k)]

    if len(vv):
        du = np.abs(uu + u0 - S["u"]) * k
        dv = np.abs(vv + v0 - S["v"]) * k
        d = np.hypot(np.maximum(du - SC.MID, 0.0), np.maximum(dv - SC.MID, 0.0))
        labs = lab[vv, uu]
        n = int(lab.max()) + 1
        least = np.full(n, np.inf)
        np.minimum.at(least, labs, d)
        count = np.bincount(labs, minlength=n)
        for L in np.flatnonzero(np.isfinite(least) & (count >= QUANT * QUANT)):
            sel = labs == L
            j = int(np.argmin(d[sel]))
            inner = cv2.distanceTransform(np.pad((lab == L).astype(np.uint8), 1), cv2.DIST_L2,
                                          cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
            deep = np.argwhere(inner >= float(inner.max()) - SC.MID)
            m = deep[int(np.argmin(np.hypot(deep[:, 0] + v0 - S["v"], deep[:, 1] + u0 - S["u"])))]
            rel = np.column_stack([uu[sel] + u0 - S["u"], vv[sel] + v0 - S["v"]]).astype(float)
            things.append({"d_px": float(least[L]), "next_px": cam(uu[sel][j], vv[sel][j]), "px": cam(m[1], m[0]),
                           "points": int(count[L]), "around": SC._surrounded(rel), "_cols": uu[sel], "_rows": vv[sel]})
    if k > 1:
        _fine(P, valid, up, u, v, S, things, cam)
    for t in things:
        t.pop("_cols", None)
        t.pop("_rows", None)
    things.sort(key=lambda t: t["d_px"])
    return {"extent": out, "foot_m": float(out["foot_m"]), "things": things}


def _fine(P, valid, up, u, v, S, things, cam):
    """A picture finer than the hand can use is read every k-th pixel (see_cloud.stride), so a thing's nearest
    sample may lie up to k - 1 pixels further from the pointed pixel than the thing does. Each thing's distance is
    read again in the picture's own pixels (P, valid: the picture given; u, v: the pointed pixel in it): of the
    pixels within k - 1 of one of its samples, those that stand over the level as its samples do (see_cloud's own
    threshold, the level and its margin), the nearest pixel square."""
    k = int(S["k"])
    H, W = valid.shape
    thr = float(S["base"]) + float(S["margin"])
    for t in things:
        reach = int(np.ceil(t["d_px"])) + 1
        U0, U1, V0, V1 = max(0, u - reach), min(W, u + reach + 1), max(0, v - reach), min(H, v + reach + 1)
        if U1 <= U0 or V1 <= V0:
            continue
        over = valid[V0:V1, U0:U1] & (P[V0:V1, U0:U1] @ np.asarray(up, dtype=float) > thr)
        near = np.zeros(over.shape, dtype=bool)
        cams = np.array([cam(c, r) for c, r in zip(t["_cols"], t["_rows"])], dtype=int).reshape(-1, 2)
        keep = (cams[:, 0] > U0 - k) & (cams[:, 0] < U1 + k) & (cams[:, 1] > V0 - k) & (cams[:, 1] < V1 + k)
        for cu, cv in cams[keep]:
            a0, a1, b0, b1 = max(U0, cu - k + 1), min(U1, cu + k), max(V0, cv - k + 1), min(V1, cv + k)
            if a1 > a0 and b1 > b0:
                near[b0 - V0:b1 - V0, a0 - U0:a1 - U0] = True
        rr, cc = np.nonzero(over & near)
        if not len(rr):
            continue
        d = np.hypot(np.maximum(np.abs(cc + U0 - u) - SC.MID, 0.0), np.maximum(np.abs(rr + V0 - v) - SC.MID, 0.0))
        j = int(np.argmin(d))
        if float(d[j]) < t["d_px"]:
            t["d_px"], t["next_px"] = float(d[j]), [int(cc[j] + U0), int(rr[j] + V0)]


def name(cloud, u, v, up, hand, role, e, sigma, moved, support=None, shown=None):
    """What pixel (u, v) names (module docstring): (extent, names or None, empty or None). e: see_cloud.extent of
    the pixel as see_spot read it (the nearer surface within one pixel taken for it). names: {"by", "distance_m",
    "error_m"} when the extent is a thing. empty: {"why": "none" | "two", "things", "foot_m", "error_m", "touch":
    a thing beyond the error has a pixel that touches this one, condition (ii)} when the pixel lies on a level and
    names no one thing (roles pick and part); the extent is then the level's own. support: (height, stated error)
    of the stated support, which a place reads; shown: what see_spot.shows() read of an edge pixel by its
    colour ("near", "far" or None); a place is read by _place().
    - The pixel's own depth lies on a thing: that thing ("the pixel lies on it").
    - Its own depth lies on a level: the ONE thing with a pixel within the spot's error across the view; role
      part: else the ONE thing that closes the level in on every side (a pane the depth does not show, the floor
      inside it); else empty. That holds whatever see_spot's extent made of the pixel: a pixel whose own depth
      shows no level (it lies within a jump's band, next to a thing: e kind None) is read by beside() too, which
      takes a pixel at the height of the level next to it as lying on that level.
    - Its own depth shows neither (no level under it, none at its own height beside it): the nearer thing
      see_cloud took for it when that thing's pixel lies within the error or the pixel is one surface with it;
      else the pixel as it is.
    A place is never answered: its extent stays as see_spot read it."""
    view, P, valid, _ = cloud
    foot0 = float(np.linalg.norm(P[v, u] - view.pos)) / view.f
    error = SC.MID * foot0 + moved
    err_px = SC.MID + moved / foot0
    on = {"by": ON, "distance_m": 0.0, "error_m": error}
    thing = e.get("kind") == "thing"
    if thing and not e.get("snapped"):
        return e, on, None
    snap = e.get("snapped") or [0, 0]
    snap_px = float(np.hypot(max(abs(snap[0]) - SC.MID, 0.0), max(abs(snap[1]) - SC.MID, 0.0)))
    taken = dict(on, by=WITHIN, distance_m=snap_px * foot0) if thing else None
    if role in PLACES:
        return _place(cloud, u, v, up, hand, e, sigma, taken, foot0, error, err_px, support, shown)
    got = beside(P, valid, view, u, v, up, hand, sigma)      # also when e made out nothing (kind None)
    if got is None:              # the pixel's own depth lies on no level: on a thing of its own, or on a surface
        if thing and snap_px > err_px:       # no level is seen under. The thing see_cloud took for it, one pixel
            comp = SC.component(P, valid, view, u + snap[0], v + snap[1], up, hand, sigma)     # off at a corner:
            if comp is not None and comp["mask"][v, u]:
                return e, on, None           # ... the pixel is one surface with it: it lies on that thing;
            own = SC.extent(P, valid, view, u, v, up, hand, sigma, snap=False)
            return own, (on if own.get("kind") == "thing" else None), None      # ... else the pixel as it is
        if e.get("kind") is None and not e.get("snapped"):     # the same pixel, read under it and beside it
            e = dict(e, why=NO_LEVEL)
        return e, taken, None
    within = [t for t in got["things"] if t["d_px"] <= err_px]
    one = within[0] if len(within) == 1 else None
    if one is None and not within and role == "part":
        around = [t for t in got["things"] if t.get("around")]
        one = around[0] if len(around) == 1 else None
    if one is not None:
        nu, nv = one["next_px"]
        e2 = SC.extent(P, valid, view, nu, nv, up, hand, sigma, snap=False)     # that very pixel, as it is
        if e2.get("kind") == "thing":
            names = {"by": WITHIN, "distance_m": one["d_px"] * foot0, "error_m": error, "pointed_px": [int(u), int(v)]}
            if one["d_px"] > err_px:
                names.update(by=AROUND)
            return e2, names, None
    return got["extent"], None, {"why": "two" if len(within) > 1 else "none", "things": got["things"],
                                 "foot_m": foot0, "error_m": error, "touch": _touches(got["things"], u, v, err_px)}


def _place(cloud, u, v, up, hand, e, sigma, taken, foot0, error, err_px, support, shown=None):
    """A place pixel (roles place and into; module docstring): (extent, names or None, near or
    None). near = {"place": True, "why": "none" | "two", "things", "foot_m", "error_m"}: the pixel lies on the
    stated support and names no one thing; it is the point itself and the spot lists what stands near it. An edge
    pixel whose colour is the nearer surface's (shown "near") lies on that nearer thing, as see_cloud took it."""
    view, P, valid, _ = cloud
    got = beside(P, valid, view, u, v, up, hand, sigma) if support is not None and shown != "near" else None
    if got is None:              # the pixel's own depth lies on no level (a thing's face), its colour is the nearer
        return e, taken, None    # thing's, or no support is stated
    lvl = got["extent"]
    if over_support(lvl["base_z"], support, sigma, foot0) is not None:   # the top of something wider than the hand
        return (e, taken, None) if e.get("snapped") else (lvl, None, None)   # reads: the pixel lies on it
    within = [t for t in got["things"] if t["d_px"] <= err_px]
    if len(within) == 1:
        nu, nv = within[0]["next_px"]
        e2 = SC.extent(P, valid, view, nu, nv, up, hand, sigma, snap=False)     # that very pixel, as it is
        if e2.get("kind") == "thing":
            return e2, {"by": WITHIN, "distance_m": within[0]["d_px"] * foot0, "error_m": error,
                        "pointed_px": [int(u), int(v)], "on_bare_support": True}, None
    return lvl, None, {"place": True, "why": "two" if len(within) > 1 else "none", "things": got["things"],
                       "foot_m": foot0, "error_m": error}


def near_rows(key, cloud, up, hand, sigma, near):
    """The "near" list of a place pixel on bare support (see _near): each thing with a pixel on it."""
    return _near(key, cloud, up, hand, sigma, near)


def _touches(things, u, v, err_px):
    """Whether a thing beyond the error has a pixel that touches pixel (u, v) (one of the eight around it):
    another surface within one pixel, condition (ii)."""
    return any(t["d_px"] > err_px and max(abs(t["next_px"][0] - u), abs(t["next_px"][1] - v)) <= 1 for t in things)


def _near(key, cloud, up, hand, sigma, empty):
    """The "near" list of an answer: each thing the look shows within the hand's own full opening of the pointed
    ray (across the view), nearest first; when none is that near, the nearest one."""
    view, P, valid, _ = cloud
    foot = empty["foot_m"]
    things = [t for t in empty["things"] if t["d_px"] * foot <= hand["open_m"]] or empty["things"][:1]
    out = []
    for t in things:
        e = SC.extent(P, valid, view, t["px"][0], t["px"][1], up, hand, sigma)
        row = {"look": key, "px": list(t["px"]), "distance_mm": round(t["d_px"] * foot * MM, 1), "narrow_mm": None,
               "wide_mm": None, "top_mm": None}
        if e.get("kind") == "thing":
            row.update(narrow_mm=round(e["narrow_m"] * MM, 1), wide_mm=round(e["wide_m"] * MM, 1),
                       top_mm=round(e["top_z"] * MM, 1))
        out.append(row)
    return out


def cannot_show_low(ex, hand, sigma):
    """Condition (iii) of "the look shows empty" as a text when it fails (BAND times the look's stated depth error
    at that range is not under the lowest thing this hand can take), else None."""
    low = _low_m(ex, hand)
    if BAND * sigma < low:
        return None
    return (f"the look cannot show a low thing: its depth is known to {BAND * sigma * MM:.1f} mm there ({BAND} times "
            f"its stated error) and the lowest thing this hand can take is {low * MM:.1f} mm (the pads' own height)")


def _fails(ex, rec, key, edge, base_z, foot, sigma, hand, things=()):
    """Which of the five conditions of "the look shows empty" fail, as texts (module docstring); [] when it holds."""
    from rcpx.r2 import see_spot
    fails = []
    t_act, t_look = scene_t(ex), rec.get("t")
    if t_act is not None and (t_look is None or float(t_look) < t_act):
        fails.append(f"look {key} was taken before the last act that changed the scene: it does not show the "
                     "scene as it stands (a new look would)")
    if edge:
        fails.append("the pixel lies on an edge: another surface stands within one pixel of it")
    low = cannot_show_low(ex, hand, sigma)
    if low:
        fails.append(low)
    up, sup_z, sup_err = see_spot.support_of(ex)
    over = over_support(base_z, (sup_z, sup_err), sigma, foot)
    if over is not None:
        fails.append(f"the pixel lies on a level {over * MM:.0f} mm over the stated support: the "
                     "top of something wider than this hand reads, not bare support")
    if any(t.get("around") for t in things):
        fails.append("the level under the pixel is closed in on every side by one thing the look shows: the floor "
                     "inside it, or a surface seen through it, not bare support")
    return fails


def answer(ex, rec, key, fixed, cloud, up, hand, out, empty, sigma, of="pixel", px=None):
    """Close a spot whose pixel names no one thing (roles pick and part): out["answer"] where the look shows
    empty (module docstring), else the doubt that names the condition that failed (the tool then acts on the
    point with the contact stop armed). px: the pixel (u, v) of look `key` the answer is about; with it the
    answer carries the look's help (rcpx/control_looks.pointed_pictures): the same picture with that
    pixel and each "near" entry marked, and a crop around the pixel that holds them all and the hand's own opening
    on each side."""
    from rcpx.r2 import see_spot
    fails = _fails(ex, rec, key, out.get("edge") or empty.get("touch"),
                   (out.get("extent") or {}).get("base_z", out["point_m"][-1]), empty["foot_m"], sigma, hand,
                   empty["things"])
    what = ("no thing lies within the look's own error" if empty["why"] == "none" else
            "two things lie within the look's own error")
    if fails:
        out["doubt"].append(f"{what} ({empty['error_m'] * MM:.1f} mm across) of your {of}, and the look cannot "
                            "say the place is empty: " + "; ".join(fails) + ". The hand goes to the point with the "
                            "contact stop armed")
        out["not_answered"] = fails
        out["remove_by"] = out.get("remove_by") or ({"tool": "look", "args": {}, "periods": 0} if "taken before"
                                                    in fails[0] else see_spot._other_camera(ex, rec, fixed))
        return out
    out["answer"] = {"empty": True, "why": empty["why"], "of": of, "look": key, "error_mm": round(
        empty["error_m"] * MM, 2), "near": _near(key, cloud, up, hand, sigma, empty)}
    if px is not None:
        from rcpx import control_looks
        try:
            out["answer"].update(control_looks.pointed_pictures(ex, key, rec, px[0], px[1], out["answer"]["near"],
                                                                hand["open_m"] / empty["foot_m"]))
        except Exception as exc:          # the pictures never cost the operator the answer itself
            out["answer"]["pictures_why"] = f"the marked picture could not be drawn ({type(exc).__name__}: {exc})"
    return out


def shown(ex, p_m, role):
    """What the newest look that shows a typed point shows there: ({"look", "age_s", "acts_since", "extent":
    "thing" | "bare" | "not in any look"}, answer or None). A look shows the point when the point projects into
    it, the robot's own body does not hide it, and the surface at that pixel is at the point, behind it (the look
    sees through where the point is: nothing stands there), or the top of a thing whose seen height holds the
    point. The answer is given for roles pick and part when that look shows the point's place empty by the five
    conditions."""
    from rcpx.r2 import see_spot
    p = np.asarray(p_m, dtype=float)
    up, _, _ = see_spot.support_of(ex)
    hand = see_spot.hand_of(ex)
    looks = [(k, r, True) for k, r in ((ex.state.get("scene_camera") or {}).get("looks") or {}).items()]
    looks += [(k, r, False) for k, r in (getattr(ex, "_looks", None) or {}).items()]
    looks = [x for x in looks if x[1].get("depth") is not None]
    t_act = scene_t(ex)
    for key, rec, fixed in sorted(looks, key=lambda x: -float(x[1].get("t") or 0.0)):
        cloud = see_spot.cloud_of(ex, rec)
        if cloud is None:
            continue
        view, P, valid, own = cloud
        x, y, zc = view.project(p)
        if not (zc > 0 and view.inside(x, y)):
            continue
        u, v = int(np.floor(x)), int(np.floor(y))
        if own[v, u] or not valid[v, u]:
            continue
        r_p, r_s = float(np.linalg.norm(p - view.pos)), float(np.linalg.norm(P[v, u] - view.pos))
        foot = r_s / view.f
        sigma, _ = see_spot._sigma_at(rec, r_s, foot)
        tol = max(BAND * sigma, foot)
        got = beside(P, valid, view, u, v, up, hand, sigma)
        if got is None:              # a thing's surface at that pixel: it holds the point, or it hides it
            e = SC.extent(P, valid, view, u, v, up, hand, sigma)
            if not (e.get("kind") == "thing" and e["base_z"] - tol <= float(p @ up) <= e["top_z"] + tol):
                continue
            kind = "thing"
        elif r_s < r_p - tol:        # the level stands in front of the point: the point lies under it, unseen
            continue
        else:
            kind = "bare"
        now = float(ex.sim_time())
        info = {"look": key, "px": [u, v], "age_s": round(now - float(rec.get("t") or now), 3),
                "acts_since": bool(t_act is not None and float(rec.get("t") or 0.0) < t_act), "extent": kind}
        if kind != "bare" or role not in ("pick", "part"):
            return info, None
        err_px = SC.MID
        within = [t for t in got["things"] if t["d_px"] <= err_px]
        if len(within) == 1:
            return dict(info, extent="thing"), None
        empty = {"why": "two" if within else "none", "things": got["things"], "foot_m": foot,
                 "error_m": SC.MID * foot, "touch": _touches(got["things"], u, v, err_px)}
        probe = {"doubt": [], "point_m": [float(c) for c in P[v, u]], "extent": got["extent"],
                 "edge": got["extent"].get("edge")}
        answer(ex, rec, key, fixed, cloud, up, hand, probe, empty, sigma, of="point", px=(u, v))
        return dict(info, why_acted=probe.get("not_answered")), probe.get("answer")
    return {"look": None, "age_s": None, "acts_since": None, "extent": "not in any look"}, None


def answer_of(seen):
    """The answer a Seen carries (place_point.resolve keeps the spot's), or None."""
    return seen.get("answer") if isinstance(seen, dict) else None


def answered(ans, tool, nth=""):
    """The result of a whole-step tool whose spot was answered: one sentence first, stopped "answered",
    nothing held, placed or moved, the "near" list; no motion, no control period, no ready call."""
    near = list(ans.get("near") or [])
    what = f"your {ans.get('of', 'pixel')}"
    lead = (f"look {ans.get('look')} shows no thing within its own error ({ans.get('error_mm')} mm across) of {what}: "
            "it shows bare support there" if ans.get("why") == "none" else
            f"look {ans.get('look')} shows two things within its own error ({ans.get('error_mm')} mm across) of {what}")
    if near:
        rows = "; ".join(f"({i}) " + (f"a thing {t['narrow_mm']:.0f} by {t['wide_mm']:.0f} mm"
                                      if t.get("narrow_mm") is not None else "a thing")
                         + f" {t['distance_mm']:.0f} mm from it, on pixel {t['px']}" for i, t in enumerate(near, 1))
        tail = f" Near it the same look shows: {rows}. A pixel on the one you mean names it (0 control periods)."
    else:
        tail = " The same look shows no thing near it."
    if ans.get("marked"):
        tail += (f" The look's own picture comes back marked ({ans['drawn']}), with a crop of it around your "
                 f"{ans.get('of', 'pixel')} enlarged {ans['crop_scale']:.1f} times (0 control periods).")
    out = {"said": f"Nothing moved: {nth}{lead}.{tail}", "stopped": "answered", "near": near,
           "answer": {k: ans.get(k) for k in ("why", "of", "look", "error_mm")}}
    for k in ("marked", "crop", "crop_box_px", "pictures_why"):
        if ans.get(k) is not None:
            out[k] = ans[k]
    out.update({"pick": {"held": False}, "work_part": {"moved": False}}.get(tool, {"held": False, "placed": False}))
    return out
