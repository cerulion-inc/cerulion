# SPDX-License-Identifier: AGPL-3.0-only
"""Package place_spot: the
place from any pixel. No tools of its own and no motion: rcpx/r2/place.py calls settle() once the thing is in the
hand (the hold's facts give its footprint) and before the set down is planned.

A pixel names the THING it lies on (rcpx/r2/place_near.py); where on that thing the held thing is set down is the
robot's to choose, from what the pointed look shows (rcpx/r2/see_place.py over the thing's points,
see_cloud.component, and the look's cloud):
- ONTO the floor of a DISH (a pixel anywhere in a thing whose inside the look shows under the pour height of its
  rim, the hand clearing that rim with the thing standing on the floor: _dish): the place of that floor nearest
  the pixel where the footprint lies inside the rim, else where the inside has the most room (a plate's middle).
  A dish whose rim the hand does not clear is an opening (INTO, below).
- ONTO a thing: the point of it nearest the pixel where the held thing's footprint (its half width across from
  the hold's facts, plus the spot's own error) lies fully on ONE level the look shows, with nothing standing
  there. The pointed point itself when it holds the footprint (nothing changes then).
- INTO a thing: a pixel anywhere on a container (its inside, its outer wall, its rim, its handle) goes into
  the opening the pixel lies in or nearest to: a hollow of the thing (see_place.basins: seen cells that lie
  under the walls around them, with the floor the look does not show behind the wall on the lens's side) that
  the pixel lies in or within a pad's width of (a wall is what a pad spans). Inside it: the place nearest the
  pixel where the footprint lies between the walls and the look shows nothing standing; when none holds it,
  the opening's middle. The place is entered over the rim (a target "into", as place_target makes one). A
  thing that runs on past the hand's window is read as far as the picture shows it, the hidden cells outside
  the outline of what the look shows of it closed (and a cell the picture's edge cuts below its top a wall).
- A pixel on a WALL whose top hides what lies past it (see_place.behind: walking away from the lens the look
  shows nothing past the wall's top as far as the footprint is wide; or, for an inner face, nothing before it):
  into the hollow the wall hides, the footprint's half width in along the look's own ray. Only where the look's
  slant lets the wall hide that much (its height times the ray's level length per unit of its drop), and never
  for a pixel that lies on a LEVEL (the cells the look shows at its point's own height within a pad's width of it
  cover at least a disc as wide as a pad: _level_at): what lies past a thing that stands beside such a pixel is not
  shown free.
- No level place of the footprint's size is seen on the thing, and the hand's window holds the thing whole. A
  thing smaller than the footprint (the look shows its top at its middle): the held thing's middle goes over the
  PLACE the pixel lies on (the level region of the pixel's own height: that top, or the level beside it where
  the pixel lies beside something standing on it) at its free spot nearest the pixel, the pixel itself where it
  is free (_free_on: the footprint, the outline the take's look showed of the held thing turned as the hand
  holds it, holds nothing the look shows standing over that level; where the hand's own bodies stand clear too,
  _hand_clear), never the thing's middle for its own sake; where no spot of it is free the place ANSWERS what
  stands there, its height and distance, and the nearest room on that level the look shows free for the
  footprint (place_side returns the answer before anything moves). Else into the middle over what stands around
  it (a hollow whose floor the look does not show). A thing that runs on past the window keeps the pointed
  point. Said either way.
- A thing the pointed look shows cut by the picture's edge (or running on past the hand's window) is read whole
  before its middle is used: with the thing in the hand the robot looks for itself with its wrist camera from
  where the hand stands (0 control periods), and where that look shows the same point on a thing it does not cut,
  the place is read from that look (_whole_look); the receipt says so.
- A pixel on bare support, a typed point, a label: the point itself (nothing here runs).
The receipt says how far the place is from the pixel and why. No size, name or pose of any thing is typed: the
sizes are the hold's, the look's own footprint and error, and ratios of rcpx/bounds_ratios.py.

    settle(ex, target, facts)     moves target["centre"] (and "pointed") to where the thing stands; returns the
                                  record it leaves in target["stands"], or None when nothing was read. It reads the
                                  pointed look as the scene stands now: newer looks of the same fixed camera, and
                                  what this session set down since (note_release, forget)
    arrival(ex, target, out, facts, before, picked)   whether the thing arrived: the set down's own evidence and
                                  what the same camera's depth shows at the place before and after
    say(out, target)              the sentence and the record on a place's result
Nothing here reads truth or names a robot or a thing.
"""

import numpy as np

from rcpx.bounds_ratios import BAND, QUANT, REL
from rcpx.r2.see_place import HALF
from rcpx.r2 import see_cloud as SC
from rcpx.r2 import see_place, see_spot

MM = 1000.0          # unit


def taken_of(ex, seen):
    """What rcpx/r2/place.py keeps of a thing before this call's take (state["place"]["taken"]): its middle, its
    sizes and bearing as the look showed it, and "reach_m": how far the farthest point the look showed of it lies
    from that middle on the level (the disc it covers about its middle whichever way the hand turns it: half the
    diagonal of a box, the radius of a round thing). None when the look did not show it whole."""
    sp = (seen.get("spot") or {}) if isinstance(seen, dict) else {}
    e = sp.get("extent") or {}
    sizes = [e.get("wide_m"), e.get("narrow_m")]
    if e.get("kind") != "thing" or e.get("cut") or e.get("border") or e.get("centre_xy") is None or not all(
            x is not None and np.isfinite(x) for x in sizes):
        return None
    b = e.get("bearing_rad")
    out = {"centre_xy": [float(x) for x in e["centre_xy"]], "wide_m": float(e["wide_m"]),
           "narrow_m": float(e["narrow_m"]), "bearing_rad": float(b) if b is not None and np.isfinite(b) else None}
    if all(e.get(k) is not None and np.isfinite(e[k]) for k in ("top_z", "base_z")):
        out["tall_m"] = max(0.0, float(e["top_z"]) - float(e["base_z"]))     # its top over what it stood on
    try:
        comp = _component(ex, sp, e)
    except Exception as exc:          # a look that cannot be read again costs the take nothing: the sizes stand
        out["reach_failed"] = f"{type(exc).__name__}: {exc}"
        comp = None
    if comp is not None:
        rel = comp[0]["xy"] - np.asarray(e["centre_xy"], dtype=float)
        d = np.linalg.norm(rel, axis=1)
        if d.size:
            out["reach_m"] = float(d.max())
        if len(rel) >= QUANT + QUANT:      # the outline it covers on the level about its middle, as the look showed it
            import cv2
            hull = cv2.convexHull(np.asarray(rel, dtype=np.float32)).reshape(-1, 2)
            out["outline_xy"] = [[float(x), float(y)] for x, y in hull]
    return out


def footprint_m(ex, facts):
    """(the half width a held thing covers about its middle whichever way it is turned, its source): how far the
    farthest point of it the look showed lies from its middle, when this call's take saw it whole; else half the
    diagonal of its seen sizes; else the half width the hold's facts state (what the jaws closed on)."""
    took = (ex.state.get("place") or {}).get("taken")
    if took and took.get("reach_m"):
        return float(took["reach_m"]), "how far the farthest point of the thing lay from its middle as the look " \
            "showed it before the take"
    if took and took.get("wide_m") and took.get("narrow_m"):
        return SC.MID * float(np.hypot(took["wide_m"], took["narrow_m"])), "half the diagonal of the thing as the " \
            "look showed it before the take"
    return float(facts["half_m"]), "the half width the hold states"


def _component(ex, sp, e, wide=False):
    """(see_cloud.component of the thing at a spot's point in the spot's newest look, its view, its cloud, the
    sigma it was read with, the look's key) or None. wide: read in a window as wide as the picture (a thing that
    runs on past the hand's window: the whole of it the picture shows)."""
    if not sp.get("looks"):
        return None
    key = sp["looks"][-1]
    _, rec, _ = see_spot.look_of(ex, key)
    cloud = None if rec is None else see_spot.cloud_of(ex, rec)
    if cloud is None:
        return None
    view, P, valid, _ = cloud
    up, _, _ = see_spot.support_of(ex)
    x, y, zc = view.project(np.asarray(sp["point_m"], dtype=float))
    if not (zc > 0 and view.inside(x, y)):
        return None
    u, v = int(np.floor(x)), int(np.floor(y))
    sigma = float(sp.get("depth_sigma_m") or e["sigma_m"])
    hand = see_spot.hand_of(ex)
    if wide:
        foot = float(np.linalg.norm(P[v, u] - view.pos)) / view.f if valid[v, u] else None
        if foot is None or not np.isfinite(foot):
            return None
        hand = dict(hand, open_m=max(view.w, view.h) * foot / BAND)
    comp = SC.component(P, valid, view, u, v, up, hand, sigma)
    return None if comp is None else (comp, view, (P, valid), sigma, key)


def _since(ex, key, H, origin, lo, hi, cell, A, up, skip):
    """The height map of look key as the scene stands now: every NEWER look of the same fixed camera read cell by
    cell (the newest look that shows a cell holds it: a hand over the place hides what an older look still
    shows), then each thing this session set down after the look that holds a cell was made (note_release):
    its footprint's cells stand as high as its top."""
    looks = (ex.state.get("scene_camera") or {}).get("looks") or {}
    rec = looks.get(key) or (getattr(ex, "_looks", None) or {}).get(key) or {}
    t0 = float(rec.get("t") or 0.0)
    when = np.full(H.shape, t0)
    for k, r in sorted(looks.items(), key=lambda kv: float(kv[1].get("t") or 0.0)):
        if key not in looks or k == key or r.get("camera") != rec.get("camera") or float(r.get("t") or 0.0) <= t0:
            continue
        got = see_spot.cloud_of(ex, r)
        if got is None:
            continue
        pts = got[1][got[2]]
        xy, z = pts @ A, pts @ up
        keep = np.all((xy >= lo) & (xy <= hi), axis=1)
        if skip is not None:
            keep &= np.linalg.norm(xy - np.asarray(skip[0], dtype=float), axis=1) > skip[1]
        Hk, _ = see_place.heights(np.column_stack([xy[keep], z[keep]]), lo, hi, cell)
        shown = np.isfinite(Hk)
        H, when = np.where(shown, Hk, H), np.where(shown, float(r.get("t") or 0.0), when)
    rows, cols = np.indices(H.shape)
    gx, gy = origin[0] + cols * cell, origin[1] + rows * cell
    for rel in (ex.state.get("place") or {}).get("released") or []:
        c = np.asarray(rel["at"], dtype=float) @ A
        disc = (np.hypot(gx - c[0], gy - c[1]) <= float(rel["radius_m"]) + HALF * cell) & (when < float(rel["t"]))
        H = np.where(disc, np.fmax(H, float(rel["top_z"])), H)
    return H


def note_release(ex, target, out, facts):
    """What a set down leaves in the scene, for the next free spot (_since): state place.released gets {"at": the
    place, "radius_m": the footprint and the spot's error, "top_z": where it rests (its bottom where a contact
    ended the way down, else the place's own height: it was let go over it and fell) and its seen height (else
    how far it hangs under the tip and reaches up the fingers), "t"}. A take there later takes it out (forget)."""
    rel = out.get("release") if isinstance(out, dict) else None
    if not rel or out.get("dropped") or out.get("object_bottom_z_mm") is None:
        return
    st = ex.state["place"]
    took = st.get("taken") or {}
    tall = took.get("tall_m")
    tall = float(tall) if tall is not None else float(facts["below_m"]) + float(facts["above_m"])
    err = target.get("sigma_xy")
    radius = footprint_m(ex, facts)[0] + (float(err) if err is not None and np.isfinite(err) else 0.0)
    rests = float(out["object_bottom_z_mm"]) / MM
    if not (out.get("landed") or out.get("stopped") in ("contact", "cap")):
        rests = min(rests, float(np.asarray(target["centre"], dtype=float)[-1]))
    st.setdefault("released", []).append({"at": [float(x) for x in target["centre"]], "radius_m": radius,
                                          "top_z": rests + tall, "t": float(ex._sense().t)})


def forget(ex, point):
    """A take at point: what this session set down within its footprint of it is in the hand now."""
    st = ex.state["place"]
    p = np.asarray(point, dtype=float)[:2]
    st["released"] = [r for r in st.get("released") or []
                      if float(np.linalg.norm(np.asarray(r["at"], dtype=float)[:2] - p)) > float(r["radius_m"])]


def _map(ex, target, facts, wide=False):
    """What settle() reads: (H, cell, origin, radius, tol, margin, comp, e, key, source, the thing's own cells,
    the cells outside the picture) or None. wide: the thing as far as the picture shows it (_component)."""
    seen = target.get("seen")
    sp = (seen or {}).get("spot") or {}
    e = sp.get("extent") or {}
    if e.get("kind") != "thing":
        return None
    got = _component(ex, sp, e, wide)
    if got is None:
        return None
    comp, view, (P, valid), sigma, key = got
    up, _, _ = see_spot.support_of(ex)
    e1, e2 = SC.axes_of(up)
    A = np.stack([e1, e2], 1)
    half, source = footprint_m(ex, facts)
    err = target.get("sigma_xy")
    radius = half + (float(err) if err is not None and np.isfinite(err) else 0.0)
    cell = QUANT * float(comp["foot"])
    lo, hi = comp["xy"].min(0) - radius - cell, comp["xy"].max(0) + radius + cell
    pts = P[valid]
    xy, z = pts @ A, pts @ up
    keep = np.all((xy >= lo) & (xy <= hi), axis=1)
    skip = ex._pl_skip()                      # what stood where the taken thing stood is in the hand now
    if skip is not None:
        keep &= np.linalg.norm(xy - np.asarray(skip[0], dtype=float), axis=1) > skip[1]
    H, origin = see_place.heights(np.column_stack([xy[keep], z[keep]]), lo, hi, cell)
    H = _since(ex, key, H, origin, lo, hi, cell, A, up, skip)
    margin = max(BAND * sigma, float(comp["foot"]))
    # the cells the picture does not hold at all (their column from the thing's base to its top projects outside
    # it): what stands there is not known, and it may be a wall
    rows, cols = np.indices(H.shape)
    gx, gy = origin[0] + cols * cell, origin[1] + rows * cell
    shows = []
    for level in (float(comp["base"]), float(comp["top"])):
        pts = gx[..., None] * e1 + gy[..., None] * e2 + level * np.asarray(up, dtype=float)
        px, py, pz = view.project(pts)
        with np.errstate(invalid="ignore"):
            shows.append((pz > 0) & (px >= 0) & (px < view.w) & (py >= 0) & (py < view.h))
    outside = ~shows[0] & ~shows[1]
    if wide:     # read whole: a cell whose column the picture's edge cuts below the thing's top may hold a wall
        outside |= shows[0] & ~shows[1]  # higher than the look shows (its seen height is a least)
    return (H, cell, origin, radius, margin + REL * radius, margin, comp, e, key, source,
            see_place.cells_of(comp["xy"], H.shape, cell, origin), outside)


def settle(ex, target, facts):
    """See the module docstring. target: a place target of place_target (kind "on" or "into") whose Seen is a
    thing a look with depth shows. Changes the target in place and returns target["stands"] = {"how", "moved_mm",
    "look", "footprint_mm", "footprint_source", "level_mm", "span_mm"}; None when the place is the point itself."""
    if target.get("kind") not in ("on", "into") or target.get("stands") is not None:
        return None
    whole = None
    try:
        whole = _whole_look(ex, target)
    except Exception as exc:          # a look that cannot be taken costs the place nothing: the pointed look stands
        target.setdefault("doubt", []).append({"what": f"the robot's own look at the thing the picture's edge cuts "
                                               f"failed ({type(exc).__name__}: {exc})", "acted": "read the place "
                                               "from the pointed look", "would_remove": None})
    try:
        got = _map(ex, target, facts)
    except Exception as exc:          # a look that cannot be read costs the place nothing: the point itself, said
        target.setdefault("doubt", []).append({"what": f"the look could not be read for a free place "
                                               f"({type(exc).__name__}: {exc})", "acted": "set it down at the "
                                               "pointed point", "would_remove": None})
        return None
    if got is None:
        return None
    H, cell, origin, radius, tol, margin, comp, e, key, source, own, outside = got
    seen = target["seen"]
    at = np.asarray(target["centre"], dtype=float)[:2] if target["kind"] == "into" else \
        np.asarray(seen.get("pointed_xy") or seen["centre_xy"], dtype=float)
    rec = {"look": key, "footprint_mm": round(radius * MM, 1), "footprint_source": source, "moved_mm": 0.0,
           "span_mm": None}
    if whole is not None:
        rec["whole"] = whole
    target["stands"] = rec
    wide = f"{2 * radius * MM:.0f} mm across: {source}, and the spot's own error"
    # (0) a DISH: the pointed point lies on a floor the look shows inside a rim that the hand clears with the
    # thing standing on that floor: onto the floor, the footprint inside the rim
    pad = float(see_spot.hand_of(ex)["pad_m"])      # a wall is what a pad spans: its gaps, and how near it a pixel
    dish = None
    for got_d in ([got] if not e.get("border") else [_map(ex, target, facts, wide=True), got]):
        if got_d is not None and dish is None:      # a thing that runs on past the hand's window: read whole first
            Hd, cd, od, _, _, md, compd, _, _, _, ownd, outd = got_d
            dish = _dish(ex, target, facts, Hd, cd, od, md, radius, outd, ownd, pad, got_d[4])
            if dish is not None and got_d is not got:
                rec["read"] = "the whole of the thing the picture shows (it runs on past the hand's window)"
    if dish is not None:
        rec.update(level_mm=round(dish["z"] * MM, 1), rim_mm=round(dish["rim_z"] * MM, 1),
                   room_mm=round(dish["room_m"] * MM, 1), moved_mm=round(dish["moved_m"] * MM, 1),
                   how={"fits": "onto the floor the look shows inside the rim, the nearest place to your pixel where "
                        "the footprint lies inside the rim", "room": "onto the floor the look shows inside the rim, "
                        "where it has the most room (no place on that floor holds the footprint inside the rim)"}[
                        dish["how"]])
        for k in ("seen_z", "rim_z", "rim_in_z"):       # a place onto a level: no rim, no floor under a rim
            target.pop(k, None)
        target.update(kind="on", surface_z=dish["z"], top_near_z=max(dish["top_z"], dish["z"]),
                      centre=np.array([dish["xy"][0], dish["xy"][1], dish["z"]]))
        target["pointed"] = np.asarray(target["centre"], dtype=float).copy()
        rec["said"] = (f"it goes onto the floor of the thing you pointed at, inside its rim, {rec['moved_mm']:.0f} mm "
                       "from your pixel" + (", where the footprint lies inside the rim" if dish["how"] == "fits" else
                                            ", where that floor has the most room"))
        if dish["how"] != "fits" or dish["moved_m"] > cell:
            target.setdefault("doubt", []).append({
                "what": f"the held thing's footprint is {wide}; the floor inside the rim has {rec['room_mm']:.0f} mm "
                        "of room at the place taken", "acted": rec["said"], "would_remove": None})
        return rec
    # (1) a hollow of the thing the pixel lies in, or within a pad's width of (its wall, its rim, its handle)
    hollow, off = _hollow(H, cell, origin, outside, own, comp, margin + tol, pad, at)
    if (hollow is None or off > pad) and e.get("border"):
        # a thing that runs on past the hand's window: read as far as the picture shows it, the hidden cells
        # outside the outline of what the look shows of it closed (what the look does not show beyond the thing
        # is no way out of an opening inside it)
        wide_got = _map(ex, target, facts, wide=True)
        if wide_got is not None:
            Hw, cw, ow, _, tw, mw, compw, _, _, _, ownw, outw = wide_got
            closed = ~np.isfinite(Hw) & ~see_place.outline(ownw)
            hw, offw = _hollow(Hw, cw, ow, outw | closed, ownw, compw, mw + tw, pad, at)
            if hw is not None and offw <= pad:
                H, cell, origin, tol, margin, comp, hollow, off = Hw, cw, ow, tw, mw, compw, hw, offw
                rec["read"] = "the whole of the thing the picture shows (it runs on past the hand's window)"
    if hollow is not None and off <= pad:
        spot = see_place.inside(H, hollow, cell, origin, at, radius, tol)
        z = float(next(v for v in (spot["z"], hollow["floor_z"], comp["base"]) if v is not None))
        rec.update(level_mm=round(z * MM, 1), rim_mm=round(hollow["rim_z"] * MM, 1), room_mm=round(spot["room_m"] * MM, 1))
        if spot["here"] and target["kind"] == "into":
            rec["how"] = "the pointed place inside the opening holds it"
            return rec
        rec["moved_mm"] = round(spot["moved_m"] * MM, 1)
        rec["how"] = {"fits": "the place inside the opening your pixel lies in or nearest to that holds the thing "
                      "fully", "lowest": "the place inside the opening your pixel lies in or nearest to where what "
                      "stands under the thing is lowest (the look shows something already inside, and no place that "
                      "holds the thing fully on the floor)", "middle": "the middle of the opening your pixel lies in "
                      "or nearest to (the look shows no place inside it that holds the thing fully)"}[spot["how"]]
        _move(target, spot["xy"], z, H, cell, origin, radius, None, rim=float(hollow["rim_z"]))
        rec["said"] = (f"it goes into the opening of the thing you pointed at, {rec['moved_mm']:.0f} mm from your "
                       "pixel" + {"fits": ", where it stands fully inside", "lowest": ", beside what already "
                                  "stands inside, as far as the opening allows", "middle": ", at its middle"}[spot["how"]])
        if spot["moved_m"] > cell:
            target.setdefault("doubt", []).append({
                "what": f"your pixel lies on a thing with an opening {rec['room_mm']:.0f} mm from its walls at the "
                        f"place taken; the held thing's footprint is {wide}",
                "acted": rec["said"], "would_remove": None})
        return rec
    into = target["kind"] == "into"
    if into and e.get("border"):      # an opening the pointed ray comes in over, of a thing wider than the hand's
        rec["how"] = "the pointed place (the rim the pointed ray comes in over; no floor is seen inside)"
        return rec                    # window: as pointed. (A thing the window holds whole: its middle, rule 3.)
    # (2) onto the thing: the pointed place when it holds the footprint on one level; a pixel on a wall whose top
    # hides what lies past it: into the hollow the wall hides; else the nearest place that holds the footprint
    spot = None if into else see_place.stand(H, cell, origin, at, radius, tol, floor_z=float(comp["base"]) + margin,
                                             on=own)
    if spot is not None and spot["here"]:
        rec.update(level_mm=round(spot["z"] * MM, 1), span_mm=round(spot["span_m"] * MM, 1),
                   how="the pointed place holds it")
        return rec
    sp = seen.get("spot") or {}
    up = see_spot.support_of(ex)[0]
    ray = None if sp.get("ray") is None else np.asarray(sp["ray"], dtype=float) @ np.stack(SC.axes_of(up), 1)
    # a wall hides at most its own height over the level the thing stands on times the look's slant (the level
    # length of its ray per unit of its drop): a look from above hides no place of the footprint's width
    hides = None if ray is None else float(np.linalg.norm(ray)) * (float(comp["top"]) - float(comp["base"])) / max(
        abs(float(np.asarray(sp["ray"], dtype=float) @ up)), float(np.finfo(float).tiny))
    z_pt = None if sp.get("point_m") is None else float(np.asarray(sp["point_m"], dtype=float) @ up)
    # a pixel on a LEVEL (_level_at: the look shows a level at its point's own height around it) lies on no wall,
    # whatever stands beside it: what lies past a thing that stands there is not shown free
    level_pt = z_pt is not None and _level_at(H, cell, origin, at, z_pt, pad, margin + tol) == z_pt
    wall = None if ray is None or into or level_pt or hides < radius + radius else see_place.behind(
        H, cell, origin, at, ray, z_pt, radius, pad, margin + tol)
    if wall is not None:
        rec.update(moved_mm=round(wall["moved_m"] * MM, 1), level_mm=round(float(comp["base"]) * MM, 1),
                   rim_mm=round(wall["rim_z"] * MM, 1), how="the hollow the wall you pointed at hides from the look "
                   f"({wall['side']} it along the look's ray, the footprint's half width in)")
        _move(target, wall["xy"], float(comp["base"]), H, cell, origin, radius, None, rim=float(wall["rim_z"]))
        rec["said"] = (f"your pixel lies on a wall and the look does not show what lies past its top: it goes into "
                       f"the thing {wall['side']} that wall, {rec['moved_mm']:.0f} mm from your pixel")
        target.setdefault("doubt", []).append({"what": f"the held thing's footprint is {wide}; what lies "
                                               f"{wall['side']} the wall you pointed at is not seen in look {key}",
                                               "acted": rec["said"], "would_remove": {"tool": "look", "args": {
                                                   "camera": "wrist"}, "periods": 0}})
        return rec
    if spot is not None:
        rec.update(level_mm=round(spot["z"] * MM, 1), span_mm=round(spot["span_m"] * MM, 1))
        rec.update(moved_mm=round(spot["moved_m"] * MM, 1), how="the nearest place on it that holds the thing fully")
        _move(target, spot["xy"], float(spot["z"]), H, cell, origin, radius, margin + tol)
        rec["said"] = f"it is set down {rec['moved_mm']:.0f} mm from your pixel as planned, so that it stands fully on it"
        target.setdefault("doubt", []).append({
            "what": f"the pointed point does not hold the thing's footprint ({wide}) on one level that look {key} "
                    "shows free", "acted": rec["said"], "would_remove": None})
        return rec
    # (3) no level place of the footprint's size is seen on it. A thing the hand's window holds whole: its middle
    # as the look shows it: ONTO it when the look shows its top there (a thing smaller than the footprint), else
    # INTO it over what stands around the middle (a hollow whose floor the look does not show, a ragged top).
    # A thing that runs on past the window: the pointed point, said.
    mid = _middle(ex, target, e)
    if mid is not None:
        c, narrow = mid
        ij = np.rint((c - origin) / cell).astype(int)
        ok = 0 <= ij[0] < H.shape[1] and 0 <= ij[1] < H.shape[0]
        here = float(H[ij[1], ij[0]]) if ok and np.isfinite(H[ij[1], ij[0]]) else None
        top = float(comp["top"])
        rec["moved_mm"] = round(float(np.linalg.norm(c - at)) * MM, 1)
        level = _level_at(H, cell, origin, at, z_pt, pad, margin + tol)
        if here is not None and top - here <= margin + tol and narrow < radius + radius:
            # ONTO a thing smaller than the footprint: its middle goes over the PLACE the pixel lies on (the level
            # region of the pixel's own height, see_place.level: that thing's top, or the level beside it where the
            # pixel lies beside something that stands on it), at the free spot of that level nearest the pixel
            # (_free_on: a cell of it whose footprint, the held thing's own outline as the hand holds it, holds
            # nothing the look shows standing over that level, lower cells being where the footprint hangs over
            # its edge, and still covers the pixel: the thing set there stands over the pointed place; where the
            # hand's own bodies stand clear too where one such spot exists), the pixel itself where it is free,
            # never the thing's middle for its own sake. Where no spot of that level is free the place ANSWERS
            # what stands there (nothing moves), with the nearest room on that level the look shows free for the
            # footprint, if any (a pixel there names it: the next call's to choose)
            on_top = level is None or here - level <= margin + tol
            lvl = here if on_top else level
            spot = None
            if z_pt is not None:      # the place: the level region the pixel lies on (that top, or the level beside)
                region = see_place.level(H, np.rint((at - origin) / cell).astype(int), z_pt, margin)
                if region is not None:
                    lvl = float(np.median(H[region]))
                    err = target.get("sigma_xy")
                    err = float(err) if err is not None and np.isfinite(err) else 0.0
                    spot = _free_on(H, cell, origin, region, lvl, radius, margin + tol, at,
                                    kernel=_held_kernel(ex, cell, err), hand=_hand_clear(ex, facts, H, cell, lvl, err),
                                    cover=True)
            if spot is None:
                return _answer(ex, target, rec, key, wide, H, cell, origin, at, radius, lvl, margin + tol,
                               _standing(H, cell, origin, at, radius, lvl, margin + tol),
                               _free_on(H, cell, origin, own, lvl, radius, margin + tol, at))
            rec.update(level_mm=round(lvl * MM, 1), moved_mm=round(spot["moved_m"] * MM, 1),
                       how="onto the level your pixel lies on, the free place nearest your pixel (the thing there "
                           "is smaller than the footprint)")
            for k in ("seen_z", "rim_z", "rim_in_z"):       # a place onto a level: no rim, no floor under a rim
                target.pop(k, None)
            rr, cc = np.indices(H.shape)
            sij = np.rint((np.asarray(spot["xy"]) - origin) / cell).astype(int)
            win = H[np.hypot(rr - sij[1], cc - sij[0]) * cell <= radius + cell]
            target.update(kind="on", surface_z=lvl, top_near_z=max(lvl, float(np.nanmax(win)) if np.isfinite(
                win).any() else lvl), centre=np.array([spot["xy"][0], spot["xy"][1], lvl]))
            target["pointed"] = target["centre"].copy()
            where = ("at your pixel" if spot["moved_m"] <= cell else f"{rec['moved_mm']:.0f} mm from your pixel, the "
                     "nearest place on it")
            rec["said"] = (f"the thing you pointed at is {narrow * MM:.0f} mm across, smaller than the held thing's "
                           f"footprint: it goes onto it {where}, where the look shows nothing standing within the "
                           "footprint" if on_top else f"it goes onto the level your pixel lies on {where}, where "
                           "the look shows nothing standing within the held thing's footprint (what stands there "
                           f"is {narrow * MM:.0f} mm across)")
        else:
            rows, cols = np.indices(H.shape)
            under = H[(np.hypot(rows - ij[1], cols - ij[0]) * cell <= radius) & np.isfinite(H)]
            z = float(under.min()) if under.size else float(comp["base"])
            rec.update(level_mm=round(z * MM, 1), how="the middle of the thing as the look shows it, entered over "
                       "what stands around it (no level place of the footprint's size is seen on it)")
            _move(target, c, z, H, cell, origin, radius, None, rim=top if not under.size else None, into=True)
            rec["said"] = (f"it goes into the middle of the thing you pointed at, {rec['moved_mm']:.0f} mm from your "
                           "pixel (the look shows no level place of its footprint's size on it)")
        target.setdefault("doubt", []).append({"what": f"the held thing's footprint is {wide}, and look {key} shows "
                                               "no level place of that size on the thing you pointed at",
                                               "acted": rec["said"], "would_remove": None})
        return rec
    rec["how"] = "the pointed point (the look shows no level place of the footprint's size on the thing)"
    target.setdefault("doubt", []).append({
        "what": f"look {key} shows no level place {2 * radius * MM:.0f} mm across on the thing you pointed at, and "
                "the picture's edge cuts the thing: its middle is not seen",
        "acted": "set it down at the pointed point with the contact stop armed", "would_remove": None})
    return rec


def _dish(ex, target, facts, H, cell, origin, band, radius, outside, own, pad, tol):
    """A dish the pointed point lies in: the pointed point lies on a
    thing whose outline holds an inside the look shows under the pour height of its rim (a pixel on the rim or a
    wall names that inside; the inside nearest the pixel; see_place.dish, within the outline of the
    thing's own cells: what the picture does not hold, and what the look does not show outside that outline, may
    be a wall; gaps narrower than a pad are closed), and the hand clears that rim with the thing standing
    on the floor (_clears: no wall beside the thing; a walled opening is place_set's into), and somewhere in it a
    pad's width from what the look shows outside it (a dip along a wall's top is none); what already stands in it
    over its floor by more than tol (the tilt a level may have under the footprint, as settle reads a level) is a
    wall of its room, so the footprint goes beside it. Then the thing goes
    ONTO that floor: the nearest place to the pixel where the footprint (radius) lies inside the rim (its room:
    how far from what the look shows outside the inside, see_place.room_in; a cell the look does not show is no
    wall), else the place inside with the most room (ties nearest the pixel). Its level: the highest the look
    shows of the floor under the footprint there. The inside of a thing lies over the stated support (a cell the
    look shows at the support's own height, within the look's band and the support's stated error, is the support
    beside the thing). Returns {"xy", "z", "rim_z", "top_z", "room_m", "moved_m", "how": "fits" | "room"} or None
    (not a dish: the rules after it run)."""
    sp = (target.get("seen") or {}).get("spot") or {}
    if sp.get("point_m") is None:
        return None
    up, base, base_sig = see_spot.support_of(ex)
    A = np.stack(SC.axes_of(up), 1)
    at = np.asarray(sp["point_m"], dtype=float) @ A
    ij = np.rint((at - origin) / cell).astype(int)
    gap = int(np.ceil(pad / cell))
    hull = see_place.outline(own)
    within = hull.copy()
    if base is not None and np.isfinite(base):     # the inside of a thing lies over the support it stands on
        sig = float(base_sig) if base_sig is not None and np.isfinite(base_sig) else 0.0
        with np.errstate(invalid="ignore"):
            within &= ~(np.isfinite(H) & (H <= float(base) + band + BAND * sig))
    got = see_place.dish(H, np.asarray(outside, dtype=bool) | (~np.isfinite(H) & ~hull), ij, band, gap, within)
    inner = None if got is None else got["mask"]
    if inner is None or int(inner.sum()) < QUANT * QUANT:
        return None
    with np.errstate(invalid="ignore"):     # what already stands inside over its floor is a wall of the room
        inner = inner & ~(H > float(got["floor_z"]) + float(tol))
    if int(inner.sum()) < QUANT * QUANT:
        return None
    R = see_place.room_in(H, inner, cell)
    rows, cols = np.nonzero(inner)
    r = R[rows, cols]
    if float(np.max(r)) < pad:      # a strip no pad's width wide (a dip along a wall's top) holds nothing
        return None
    if not _clears(ex, facts, got["floor_z"], got["rim_z"]):
        return None
    xy = np.asarray(origin, dtype=float) + np.column_stack([cols, rows]) * cell
    d = np.linalg.norm(xy - at, axis=1)
    fits = r > radius
    mid = None if fits.any() else _round_middle(own, outside, cell, origin)
    dm = None if mid is None else np.linalg.norm(xy - mid, axis=1)
    if fits.any():
        k, how = int(np.argmin(np.where(fits, d, np.inf))), "fits"
    elif dm is not None and float(np.min(dm)) <= QUANT * cell:
        # no place on the floor holds the footprint inside the rim, on a round dish: its middle, the centre of the
        # circle its outline shows (read also where the picture's edge cuts it), where the footprint stands over
        # the rim evenly all round
        k, how = int(np.argmin(dm)), "room"
    else:
        best = float(np.max(r))
        k, how = int(np.argmin(np.where(r >= best - cell, d, np.inf))), "room"
    rr, cc = np.indices(H.shape)
    near = np.hypot(rr - rows[k], cc - cols[k]) * cell
    under = H[inner & (near <= min(radius, float(r[k])))]
    level = float(np.max(under)) if under.size else float(H[rows[k], cols[k]])
    win = H[near <= radius + cell]
    top = float(np.nanmax(win)) if np.isfinite(win).any() else level
    return {"xy": [float(xy[k][0]), float(xy[k][1])], "z": level, "rim_z": got["rim_z"],
            "top_z": max(top, got["rim_z"]), "room_m": float(r[k]), "moved_m": float(d[k]), "how": how}


def _round_middle(own, outside, cell, origin):
    """The middle of a round thing (m, level axes) from its outline on the grid: the circle through its edge cells
    (own cells with a neighbour that is not its own), fitted by least squares, the edge cells the picture's edge
    cuts (beside a cell outside the picture, or on the grid's border) left out, fitted again on the edge cells
    within QUANT cells of it (a shadow's edge, a cut, what stands on it are off the circle). None where fewer than
    QUANT**3 edge cells are left, under half of them lie on the circle, or those lie off it by more than a cell
    (root mean square): not a round outline. The outline is the convex one of the own cells (see_place.outline): a
    dish whose floor the look reads apart from its rim shows a ring, or a U where the picture cuts it; its inner
    edge is not the circle."""
    import cv2
    m = see_place.outline(own).astype(np.uint8)
    if m.sum() == 0:
        return None
    ring = np.ones((3, 3), np.uint8)
    edge = (m > 0) & (cv2.erode(m, ring, borderType=cv2.BORDER_CONSTANT, borderValue=0) == 0)
    edge &= ~(cv2.dilate(np.asarray(outside, dtype=np.uint8), ring) > 0)
    edge[0, :] = edge[-1, :] = False
    edge[:, 0] = edge[:, -1] = False
    rows, cols = np.nonzero(edge)
    if len(rows) < QUANT ** 3:
        return None
    x, y = cols.astype(float), rows.astype(float)
    keep = np.ones(len(x), dtype=bool)
    for _ in range(QUANT + QUANT):     # the fit, then again on the edge cells within QUANT cells of its circle
        sol, *_ = np.linalg.lstsq(np.column_stack([x[keep], y[keep], np.ones(int(keep.sum()))]),
                                  -(x[keep] ** 2 + y[keep] ** 2), rcond=None)
        cx, cy = -sol[0] / 2.0, -sol[1] / 2.0
        rr = cx * cx + cy * cy - sol[2]
        if not rr > 0:
            return None
        off = np.abs(np.hypot(x - cx, y - cy) - np.sqrt(rr))
        keep = off <= QUANT
        if int(keep.sum()) < QUANT ** 3:
            return None
    # a round outline: most of its edge cells on the circle, and those within a cell of it
    if not (keep.mean() >= HALF and float(np.sqrt(np.mean(off[keep] ** 2))) <= 1.0):
        return None
    return np.asarray(origin, dtype=float) + np.array([cx, cy]) * cell


def _clears(ex, facts, floor_z, rim_z):
    """Whether a thing set onto a floor inside a rim has no wall beside it (place_set's walled rule, read before the
    set down is planned): the rim stands over the floor by no more than the hand's tracking error on the way down,
    and with the thing's bottom on the floor the hand's lowest point stands over the rim by more than that error.
    False where the arm's numbers cannot be read (the opening rules run)."""
    try:
        from rcpx import bounds
        B = bounds.of(ex)
        told = ex._told()
        down = np.array([0.0, 0.0, -1.0])
        below = float(facts["below_m"])
        track = float(ex._pl_track(B, told.q, ex._pl_room(B, told.q, down) + below))
        low = float(ex._pl_hand_low(told.R))
    except (AttributeError, KeyError, TypeError, ValueError):
        return False
    return rim_z - floor_z <= track and floor_z + below - low > rim_z + track


def _sigma_of(sp):
    """The larger of a spot's two stated errors (m), else its depth's stated error, else 0."""
    s = sp.get("sigma_m") or {}
    vals = [float(v) for v in (s.get("along_ray"), s.get("across"), sp.get("depth_sigma_m"))
            if v is not None and np.isfinite(v)]
    return max(vals) if vals else 0.0


def _whole_look(ex, target):
    """A thing the pointed look shows cut by the picture's edge (or running on past the hand's window) is read whole
    before its middle is used: the robot looks for itself with its own wrist camera from where the hand stands (a look
    costs no control period), and where that look shows the SAME point (its ray lands within
    BAND of the two spots' stated errors of the pointed point: a point hidden from it is not shown) on a thing it
    does not cut, the place target is made again from that look (place_target.target_from_seen: kind, rim, entry)
    and settle() reads the place from it. Only with something held (the place of this call; a count before the take
    takes no look). Returns {"look", "moved_mm"} or None (the pointed look stands)."""
    seen = target.get("seen")
    sp = (seen or {}).get("spot") or {}
    e = sp.get("extent") or {}
    if e.get("kind") != "thing" or not (e.get("cut") or e.get("border")) or sp.get("point_m") is None:
        return None
    if not callable(getattr(ex, "_look_impl", None)) or ex._pl_held() is None:
        return None
    from rcpx import control_hold
    if getattr(ex, "_pressing", False) or control_hold.rest_force(ex, ex._sense()) > control_hold.level(ex):
        return None                   # a look now would ease the arm first (control periods): it is no free look
    p = np.asarray(sp["point_m"], dtype=float)
    lk = ex._look_impl("place: the thing you pointed at is cut by the picture's edge: the robot's own look at it "
                       "(0 control periods)")
    if not isinstance(lk, dict) or lk.get("can_see") == "dark" or not lk.get("look_id"):
        return None
    _, rec, _ = see_spot.look_of(ex, lk["look_id"])
    cloud = None if rec is None else see_spot.cloud_of(ex, rec)
    if cloud is None:
        return None
    view = cloud[0]
    x, y, zc = view.project(p)
    if not (zc > 0 and view.inside(x, y)):
        return None
    from rcpx.r2 import place_point, place_target
    new = place_point.resolve(ex, {"look": lk["look_id"], "px": [int(np.floor(x)), int(np.floor(y))]}, "place")
    got = (new or {}).get("spot") or {}
    ge = got.get("extent") or {}
    if "refused" in new or ge.get("kind") != "thing" or ge.get("cut") or ge.get("border") or got.get("point_m") is None:
        return None
    off = float(np.linalg.norm(np.asarray(got["point_m"], dtype=float) - p))
    if off > BAND * float(np.hypot(_sigma_of(sp), _sigma_of(got))) + QUANT * float(np.linalg.norm(p - view.pos)) / view.f:
        return None                   # the wrist ray lands elsewhere: that look does not show the pointed point
    keep = {k: target.get(k) for k in ("labels", "ref", "doubt")}
    target.update(place_target.target_from_seen(new, "place"))
    target.update({k: v for k, v in keep.items() if v is not None})
    target["pointed"] = None
    out = {"look": lk["look_id"], "moved_mm": round(off * MM, 1)}
    target.setdefault("doubt", []).append({
        "what": "the pointed look's picture cuts the thing you pointed at: its middle is not seen there",
        "acted": f"read it whole from the robot's own look {lk['look_id']} (0 control periods), {out['moved_mm']:.1f} "
                 "mm from the pointed point", "would_remove": None})
    return out


def _level_at(H, cell, origin, at, z=None, pad=0.0, tol=0.0):
    """The level the pointed point lies on (m), or None where the look shows none: the point's own height z where
    the look shows a level there (the seen cells within a pad's width of it that stand at z within tol cover at
    least a disc as wide as a pad, the pad's own patch; a point beside a thing that stands there shares its cells
    with that thing's side, a point on a face has no level at its height around it), else the height of the cell
    under the point (a point on a face names the top over it)."""
    ij = np.rint((np.asarray(at, dtype=float) - origin) / cell).astype(int)
    if not (0 <= ij[0] < H.shape[1] and 0 <= ij[1] < H.shape[0]) or not np.isfinite(H[ij[1], ij[0]]):
        return None
    if z is not None and np.isfinite(z):
        from rcpx.r2.see_place import _disc
        rows, cols = np.indices(H.shape)
        near = (np.hypot(rows - ij[1], cols - ij[0]) * cell <= max(float(pad), cell)) & np.isfinite(H)
        with np.errstate(invalid="ignore"):
            on = near & (np.abs(H - float(z)) <= tol)
        if near.any() and int(on.sum()) >= int(_disc(HALF * float(pad), cell).sum()):
            return float(z)
    return float(H[ij[1], ij[0]])


def _standing(H, cell, origin, xy, radius, level, tol):
    """What the look shows standing within a footprint of radius about the level point xy over level by more than
    tol: {"xy": the such cell nearest xy, "z": the highest of them, "cells": how many} or None (the footprint is
    free there)."""
    ij = np.rint((np.asarray(xy, dtype=float) - origin) / cell).astype(int)
    rows, cols = np.indices(H.shape)
    d = np.hypot(rows - ij[1], cols - ij[0]) * cell
    with np.errstate(invalid="ignore"):
        hit = (d <= radius) & np.isfinite(H) & (H > float(level) + tol)
    if not hit.any():
        return None
    r, cc = np.unravel_index(int(np.argmin(np.where(hit, d, np.inf))), H.shape)
    return {"xy": [float(origin[0] + cc * cell), float(origin[1] + r * cell)], "z": float(np.max(H[hit])),
            "cells": int(hit.sum())}


def _free_on(H, cell, origin, own, level, radius, tol, at, kernel=None, hand=None, cover=False):
    """The free spot on a level, nearest the level point at: a cell of the thing (own) that stands at level
    (within tol) and whose footprint (kernel: a mask with its anchor, the cells the held thing covers about its
    middle; else a disc of radius) holds nothing the look shows higher than level by more than tol (lower cells are
    where a footprint wider than the thing hangs over its edge). hand: a mask of the cells where the hand's own
    bodies also stand clear of what the look shows (_hand_clear); the nearest such spot is taken where one exists.
    cover: only spots whose footprint still covers at (the thing set there stands over the pointed point).
    {"xy", "moved_m", "hand_clear": True | False | None} or None."""
    from rcpx.r2.see_place import _disc
    import cv2
    seen = np.isfinite(H)
    K, anchor = (_disc(radius, cell), None) if kernel is None else kernel
    src = np.where(seen, H, -np.inf).astype(np.float32)
    hi = cv2.dilate(src, K, anchor=(-1, -1) if anchor is None else anchor, borderType=cv2.BORDER_CONSTANT,
                    borderValue=float("-inf"))
    with np.errstate(invalid="ignore"):
        good = seen & (np.abs(H - float(level)) <= tol) & np.asarray(own, dtype=bool) & (hi <= float(level) + tol)
    if cover:       # the cells c whose footprint holds at: at - c is one of the footprint's own offsets
        ij = np.rint((np.asarray(at, dtype=float) - np.asarray(origin, dtype=float)) / cell).astype(int)
        a = (K.shape[1] // 2, K.shape[0] // 2) if anchor is None else anchor
        ky, kx = np.nonzero(K)
        cy, cx = ij[1] - (ky - a[1]), ij[0] - (kx - a[0])
        inb = (cy >= 0) & (cy < H.shape[0]) & (cx >= 0) & (cx < H.shape[1])
        holds = np.zeros(H.shape, dtype=bool)
        holds[cy[inb], cx[inb]] = True
        good &= holds
    if not good.any():
        return None
    clear = None
    if hand is not None:
        clear = bool((good & hand).any())
        if clear:
            good = good & hand
    rows, cols = np.nonzero(good)
    xy = np.asarray(origin, dtype=float) + np.column_stack([cols, rows]) * cell
    d = np.linalg.norm(xy - np.asarray(at, dtype=float), axis=1)
    k = int(np.argmin(d))
    return {"xy": [float(xy[k][0]), float(xy[k][1])], "moved_m": float(d[k]), "hand_clear": clear}


def _kernel(poly, grow, cell):
    """(a mask of the cells whose middle lies within the plan polygon poly (about the anchor cell's middle) grown by
    grow, its anchor) for cv2.dilate: the value at a cell is then the highest of what lies under that polygon there."""
    from rcpx.r2.place_side import inside_grown
    P = np.asarray(poly, dtype=float).reshape(-1, 2)
    a = int(np.ceil((float(np.max(np.linalg.norm(P, axis=1))) + float(grow)) / cell)) + 1
    g = np.arange(-a, a + 1) * cell
    X, Y = np.meshgrid(g, g)
    inside = inside_grown([np.asarray(p) for p in P], np.column_stack([X.ravel(), Y.ravel()]), float(grow))
    return inside.reshape(X.shape).astype(np.uint8), (a, a)


def _held_kernel(ex, cell, err):
    """The cells the held thing covers about its middle as it will stand (a kernel for _free_on), or None (the disc
    of its reach stands in): the outline the take's look showed of it on the level (taken outline_xy), turned by
    the hand's own turn about the vertical since the jaws closed (the carry keeps the hand's rotation), with that
    outline turned a half turn about its middle beside it (the side the look did not see is taken as wide as the
    side it saw), grown by err (the spot's own error) and half a cell. None where the take saw no outline, where
    the hold's record says the thing moved between the pads, or where the close's rotation is not known."""
    from rcpx import control_hold
    from rcpx.r2.place_side import hull_2d
    st = ex.state.get("place") or {}
    pts = (st.get("taken") or {}).get("outline_xy")
    R0 = st.get("closed_R")
    if not pts or R0 is None:
        return None
    stands = dict.get(control_hold.held(ex) or {}, "stands")
    if isinstance(stands, dict) and stands.get("moved"):
        return None
    Rd = np.asarray(ex._told().R, dtype=float) @ np.asarray(R0, dtype=float).T
    yaw = float(np.arctan2(Rd[1, 0], Rd[0, 0]))
    c, s_ = np.cos(yaw), np.sin(yaw)
    P = np.asarray(pts, dtype=float) @ np.array([[c, s_], [-s_, c]])
    poly = hull_2d(np.vstack([P, -P]))
    return _kernel(poly, float(err) + HALF * cell, cell)


def _hand_clear(ex, facts, H, cell, level, sig):
    """Per cell: whether, with the held thing's middle there and its bottom at level, every body of the hand stands
    over what the height map shows under its footprint (grown by the spot's error and the hand's tracking error on
    the way down): the rule of the hand's own stop (place_carry._pl_hand_stop, place_set._pl_hand_lift) would not
    hold the way down higher. None where the hand or the arm's numbers cannot be read."""
    import cv2
    try:
        from rcpx import bounds
        from rcpx.r2.place_side import hand_hulls, hull_2d
        told = ex._told()
        R = np.asarray(told.R, dtype=float)
        obj = R @ np.asarray(facts["offset_T"], dtype=float)
        B = bounds.of(ex)
        below = float(facts["below_m"])
        track = float(ex._pl_track(B, told.q, ex._pl_room(B, told.q, np.array([0.0, 0.0, -1.0])) + below))
        hulls = hand_hulls(ex)
    except (AttributeError, KeyError, TypeError, ValueError):
        return None
    if not hulls:
        return None
    src = np.where(np.isfinite(H), H, -np.inf).astype(np.float32)
    tip = float(level) + below
    ok = np.ones(H.shape, dtype=bool)
    for _, h in hulls:
        w = (R @ np.asarray(h, dtype=float).T).T
        K, anchor = _kernel(hull_2d(w[:, :2] - obj[:2]), float(sig) + track, cell)
        top = cv2.dilate(src, K, anchor=anchor, borderType=cv2.BORDER_CONSTANT, borderValue=float("-inf"))
        ok &= top <= tip + float(w[:, 2].min()) - track
    return ok


def _px_in(ex, key, p):
    """The pixel of look key that shows the point p (m), or None."""
    try:
        _, lk, _ = see_spot.look_of(ex, key)
        got = None if lk is None else see_spot.cloud_of(ex, lk)
        if got is None:
            return None
        x, y, zc = got[0].project(np.asarray(p, dtype=float))
        return [int(np.floor(x)), int(np.floor(y))] if zc > 0 and got[0].inside(x, y) else None
    except Exception:
        return None


def _answer(ex, target, rec, key, wide, H, cell, origin, at, radius, level, tol, busy, free=None):
    """The place ANSWERS (for a place the look shows taken): what the look shows standing within the
    held thing's footprint where it would go. target["stands"] carries "answer": {"why", "look", "standing":
    [{"z_mm", "over_level_mm", "distance_mm": from the pixel to the nearest of it, "px"}], "footprint_mm",
    "level_mm", "free": the nearest room on that level the look shows free for the footprint ({"distance_mm",
    "px"}) or None}; the place moves nothing (place_side returns it before any motion)."""
    near = []
    if busy is not None:
        near.append({"z_mm": round(busy["z"] * MM, 1), "over_level_mm": round((busy["z"] - float(level)) * MM, 1),
                     "distance_mm": round(float(np.linalg.norm(np.asarray(busy["xy"]) - np.asarray(at))) * MM, 1),
                     "px": _px_in(ex, key, [busy["xy"][0], busy["xy"][1], busy["z"]]), "cells": busy["cells"]})
    room = None if free is None else {"distance_mm": round(free["moved_m"] * MM, 1),
                                      "px": _px_in(ex, key, [free["xy"][0], free["xy"][1], float(level)])}
    rec.update(how="answered: the look shows something standing within the held thing's footprint where it would go",
               level_mm=round(float(level) * MM, 1),
               answer={"why": "taken", "look": key, "standing": near, "footprint_mm": round((radius + radius) * MM, 1),
                       "level_mm": round(float(level) * MM, 1), "free": room})
    rec["said"] = (f"look {key} shows something standing " + (
        f"{near[0]['over_level_mm']:.0f} mm over the level your pixel lies on, {near[0]['distance_mm']:.0f} mm from "
        "it" if near else "there") + f", within the held thing's footprint ({wide}) where it would go; " + (
        f"the nearest room on that level the look shows free for it lies {room['distance_mm']:.0f} mm from your "
        f"pixel" + (f", on pixel {room['px']}" if room["px"] else "") if room else
        "the look shows no room on that level free for it"))
    return rec


def _hollow(H, cell, origin, walled, own, comp, deep_m, pad, at):
    """(the hollow of the height map nearest the level point at, how far its nearest cell lies from it) or (None,
    inf). walled: cells taken as walls of any height (outside the picture; hidden cells beyond the thing). Gaps
    in a wall narrower than a pad are closed; a hollow takes half a pad at least."""
    held_by = see_place.walls(H, int(np.ceil(pad / cell)))
    held_by[walled] = np.inf
    found = see_place.basins(H, see_place.fill(held_by), deep_m, own, float(comp["base"]),
                             max(QUANT * QUANT, int((pad / cell) ** 2)))
    found = [b for b in found if b["room_cells"] * cell >= SC.MID * pad]
    return see_place.nearest_basin(found, cell, origin, at)


def _middle(ex, target, e):
    """(the middle of the thing as the look shows it, its narrow width) on the level, or None when the picture's
    edge cuts the thing. The extent's own when the hand's window holds the thing whole; a thing that runs on past
    that window is read again in a window as wide as the picture (see_cloud.component with the window's own
    measure, the widest opening, set to the picture's width at the pixel's range): the middle of the least
    rectangle around its top (the points within the fingers' depth of its highest)."""
    if not e.get("border"):
        return np.asarray(e["centre_xy"], dtype=float), float(e["narrow_m"])
    import cv2
    sp = target["seen"]["spot"]
    _, rec, _ = see_spot.look_of(ex, sp["looks"][-1])
    view, P, valid, _ = see_spot.cloud_of(ex, rec)
    x, y, _ = view.project(np.asarray(sp["point_m"], dtype=float))
    u, v = int(np.floor(x)), int(np.floor(y))
    up, _, _ = see_spot.support_of(ex)
    hand = see_spot.hand_of(ex)
    foot = float(np.linalg.norm(P[v, u] - view.pos)) / view.f
    wide = dict(hand, open_m=max(view.w, view.h) * foot / BAND)
    comp = SC.component(P, valid, view, u, v, up, wide, float(sp.get("depth_sigma_m") or e["sigma_m"]))
    if comp is None:
        return None
    m, (u0, u1, v0, v1) = comp["mask"], comp["window"]
    if m[v0, :].any() and v0 == 0 or m[:, u0].any() and u0 == 0 or m[v1 - 1, :].any() and v1 == view.h or \
            m[:, u1 - 1].any() and u1 == view.w:
        return None
    slab = comp["z"] >= float(comp["top"]) - float(hand["finger_m"])
    (cx, cy), (w, h), _ = cv2.minAreaRect(comp["xy"][slab].astype(np.float32))
    return np.array([float(cx), float(cy)]), float(min(w, h)) + float(comp["foot"])


def _move(target, xy, z, H, cell, origin, radius, wall_m, rim=None, into=False):
    """The target at a new place: onto a level at height z; or INTO the thing, over the rim around the place, when
    what the look shows within the footprint's reach of it and one cell more (the walls beside it) stands over
    that level by more than wall_m (the look's band and the tilt a level may have), or a rim is given."""
    ij = np.rint((np.asarray(xy, dtype=float) - origin) / cell).astype(int)
    rows, cols = np.indices(H.shape)
    win = H[np.hypot(rows - ij[1], cols - ij[0]) * cell <= radius + cell]
    near = float(np.nanmax(win)) if np.isfinite(win).any() else float(z)
    if into or rim is not None or near - float(z) > wall_m:
        rim_z = float(rim) if rim is not None else max(near, float(target.get("rim_z") or near))
        target.update(kind="into", surface_z=None, seen_z=float(z), rim_z=rim_z, rim_in_z=rim_z,
                      top_near_z=max(rim_z, near), centre=np.array([float(xy[0]), float(xy[1]), float(z)]))
    else:
        target.update(kind="on", surface_z=float(z), top_near_z=max(near, float(z)),
                      centre=np.array([float(xy[0]), float(xy[1]), float(z)]))
    target["pointed"] = np.asarray(target["centre"], dtype=float).copy()


def level_of(target):
    """The place's own level (m): the surface a place ONTO lands on, else the rim of a place INTO (the highest a
    thing let go inside it stands on), else the spot's own height."""
    for key in ("surface_z", "rim_z"):
        v = target.get(key)
        if v is not None and np.isfinite(v):
            return float(v)
    return float(np.asarray(target["centre"], dtype=float)[-1])


def onto_level(target):
    """Whether a place is ONTO a level the look shows (kind "on" with its surface): there a thing rests where it
    lands. A thing let go into an opening may fall on inside after a contact over the rim (the set down says so)."""
    v = target.get("surface_z")
    return target.get("kind") == "on" and v is not None and bool(np.isfinite(v))


def rests_over(target, out, facts, floor=None, tall=None):
    """Where a CONTACT ended the way down onto a level: {"over_m": how far the fingertips stood over the place's own
    level as the jaws opened, beyond the most a thing standing on that level puts them (the larger of its seen
    height and the most it can hang under them: the pads hold it, so its top is not under the fingertips), "band_m",
    "reach_m", "level_m"}; None when no contact ended it (a thing let go in the air falls), the tip's height is not
    known, or the thing's seen height is not (tall: what the take's look showed of it). The band: the set down's
    own tracking error and the place's height error BAND times over. floor: the stated support (a place under it
    is set down on it)."""
    rel = out.get("release") or {}
    tip = rel.get("opened_at_mm")
    if not (out.get("landed") or out.get("stopped") in ("contact", "cap")) or not tip or not onto_level(target) \
            or tall is None:
        return None
    most = facts.get("below_max_m")
    hang = float(most) if most is not None else float(facts["below_m"])
    sig = target.get("sigma_z")
    band = float(rel.get("track_mm") or 0.0) / MM + BAND * (float(sig) if sig is not None and np.isfinite(sig) else 0.0)
    level = level_of(target) if floor is None else max(level_of(target), float(floor))
    reach = max(float(tall), hang)
    return {"over_m": float(tip[-1]) / MM - level - reach, "band_m": band, "reach_m": reach, "level_m": level}


def arrival(ex, target, out, facts, before, picked):
    """What is known of the arrival ("placed" is true by truth): the set down's own evidence
    (place_set._pl_arrival: the release, the same view where a witness is loaded); then what the SAME fixed
    camera's depth shows at the place before and after (_depth: a look costs no motion), which decides where it
    can; where it cannot, the landing, for a place onto a level: a contact that ended the way down with the
    fingertips higher over the place's own level than a thing standing there puts them, by more than the band
    (rests_over), says it rests on something over the place: NOT arrived. Returns (arrived true | false | None, evidence)."""
    arrived, ev = ex._pl_arrival(target, out["release"], before, picked)
    floor = ex._pl_support_z() if hasattr(ex, "_pl_support_z") else None
    try:
        seen = _depth(ex, target, facts, before, floor, ev)
    except Exception as exc:          # a look that fails costs the place nothing: the other evidence stands
        ev["depth"], seen = {"failed": f"{type(exc).__name__}: {exc}"}, None
    if seen is not None:
        return seen, ev
    over = rests_over(target, out, facts, floor, ((ex.state.get("place") or {}).get("taken") or {}).get("tall_m"))
    if over is not None:
        ev["landing"] = {"tip_over_reach_mm": round(over["over_m"] * MM, 1), "band_mm": round(over["band_m"] * MM, 1),
                         "reach_mm": round(over["reach_m"] * MM, 1), "level_mm": round(over["level_m"] * MM, 1)}
        if over["over_m"] > over["band_m"]:
            ev["landing"]["rests"] = "over the place: the contact that ended the way down was not the place"
            return False, ev
    return arrived, ev


def _depth(ex, target, facts, before, floor, ev):
    """The same fixed camera's depth before and after the set down: of the pixels whose point lies in the
    footprint's column over the place after it, those that now stand nearer the lens than before by more than
    the look's band are something new. Onto a level, most of what is new higher than the thing can stand on it
    (its seen height before the take and the band): it rests over the place, False. Else something new: True.
    The place's own surface in plain view (onto a level: the pixels of the column that showed that level, every
    one of them seen again at the same range) and nothing new: False, whatever the hand felt on the way down.
    Else None (no fixed camera's look before, a place into a thing with nothing new seen, a place the arm
    hides). Writes ev["depth"]."""
    looks = (ex.state.get("scene_camera") or {}).get("looks") or {}
    rec0 = looks.get(before) if before else None
    c0 = None if rec0 is None else see_spot.cloud_of(ex, rec0)
    if c0 is None:
        return None
    from rcpx.r2 import scene_camera
    after = scene_camera.scene_look(ex, rec0["camera"], "after the set down")
    rec1 = looks.get(after.get("look_id")) if isinstance(after, dict) else None
    c1 = None if rec1 is None else see_spot.cloud_of(ex, rec1)
    if c1 is None:
        return None
    view, P0, ok0, _ = c0
    _, P1, ok1, _ = c1
    up, _, _ = see_spot.support_of(ex)
    A = np.stack(SC.axes_of(up), 1)
    c = np.asarray(target["centre"], dtype=float) @ A
    radius = footprint_m(ex, facts)[0]
    r0, r1 = np.linalg.norm(P0 - view.pos, axis=-1), np.linalg.norm(P1 - view.pos, axis=-1)
    foot = float(np.median(r1[ok1])) / view.f
    band = max(BAND * float(see_spot._sigma_at(rec1, float(np.median(r1[ok1])), foot)[0]), foot)
    col0 = ok0 & (np.linalg.norm(P0 @ A - c, axis=-1) <= radius)
    col1 = ok1 & (np.linalg.norm(P1 @ A - c, axis=-1) <= radius)
    new = col1 & ok0 & (r0 - r1 > band)
    tall = ((ex.state.get("place") or {}).get("taken") or {}).get("tall_m")
    tall = tall if onto_level(target) else None         # (into an opening: what stands inside is not judged)
    high = np.zeros(ok1.shape, dtype=bool) if tall is None else new & (
        P1 @ up > max(level_of(target), -np.inf if floor is None else float(floor)) + float(tall) + band)
    level = target.get("surface_z")         # (a place INTO a thing has no seen surface: never "not arrived")
    bare = np.zeros(ok0.shape, dtype=bool) if level is None else col0 & (np.abs(P0 @ up - float(level)) <= band)
    same = bare & ok1 & (np.abs(r0 - r1) <= band)
    ev["depth"] = {"by": "the same camera's depth before and after", "before": before,
                   "after": after.get("look_id"), "new_pixels": int(new.sum()),
                   "thing_tall_mm": None if tall is None else round(float(tall) * MM, 1),
                   "new_higher": int(high.sum()),
                   "place_surface_pixels": int(bare.sum()), "of_them_unchanged": int(same.sum())}
    n_low, n_high = int((new & ~high).sum()), int(high.sum())
    if n_high >= QUANT * QUANT and n_high > n_low:     # most of what is new stands higher than it can at the level
        ev["depth"]["rests"] = "over the place: what is new there stands higher than the thing stands on its level"
        return False
    if n_low >= QUANT * QUANT:
        return True
    if int(bare.sum()) >= QUANT * QUANT and int(same.sum()) == int(bare.sum()):
        return False
    return None


def say(out, target):
    """A place's result with where the thing was set relative to the pixel: out["stands"], and the
    sentence when the place is not the pointed point itself."""
    rec = target.get("stands") if isinstance(target, dict) else None
    if not isinstance(out, dict) or "refused" in out or rec is None:
        return out
    out["stands"] = {k: v for k, v in rec.items() if k != "said"}
    where = [x for x in ((f"the pointed look's picture cuts the thing you pointed at, so it was read whole from the "
                          f"robot's own look {rec['whole']['look']} (0 control periods)") if rec.get("whole") else None,
                         rec.get("said")) if x]
    if where and out.get("said"):
        out["said"] = out["said"].rstrip() + " Where: " + "; ".join(where) + "."
    return out
