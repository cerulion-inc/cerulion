# SPDX-License-Identifier: AGPL-3.0-only
"""What the sheet and the pictures say about a pointed part (package sight, I37; SPEC-R3 4.5 steps 11 to 13, the survey
fits, the sheet tile maps). Reads robot://devices as served (ex.context["devices"]), the frames and the survey result;
never a scene. Nothing here moves the arm or names a device: every rule reads the served text.

    doc(ex), plist(ex)                    the robot://devices document and sheet_words.parts of it
    part_check(colour, label, plist)      a warning when the spot's colour is another part's and not the named one's,
                                          or the names when the label is no part name
    spot_colour(img, px, gains)           the colour name of the central PART_CHECK_PX square
    floor_gains(img, keep_out)            grey-world gains from the frame outside the hand and the pointed spot
    relations(p_mm, others)               where the point is from the device's other located points, in side words
    fits(cand, doc, floor_z_mm)           {value null, band, score, footprint, says}: a survey candidate's measured
                                          size beside the device's rough size (the verdict is withdrawn: see fits)
    survey_post(ex, result)               the survey post hook: fits per candidate, best_fit, said, ex.state boxes,
                                          the survey sheet's tile map
    sheet_pixel(sheets, sheet_id, px, frame)   (look id, pixel in that look) of a contact-sheet pixel
"""

import math

import numpy as np

from rcpx import colour_words, r3_limits as L, sheet_words, side_words

MM = 1000.0
RELATION_NEAR_MM = 250.0     # another located point this near is on the same device (the devices this was tried on are at most
                             # 300 mm across) when no survey box says which device holds either point
RELATIONS_MAX = 2            # relations said (nearest first): the brief result stays under 900 bytes
RELATION_MIN_MM = 10.0       # a part of a relation this small is not said (the uncertainty of a located point)
SPOT_KEEP_OUT_PX = 60        # floor gains leave out this square round the pointed spot (it is the part, not floor)
HARNESS_MAX_SIDE_PX = 640    # the loop's resize rule (SPEC-R3 7): a longer image side is scaled to 640


def doc(ex):
    """The robot://devices document the session serves (ex.context["devices"], a dict or a callable)."""
    d = (getattr(ex, "context", None) or {}).get("devices")
    if callable(d):
        try:
            d = d()
        except Exception:      # a document that cannot be read now counts as none
            d = None
    return d if isinstance(d, dict) else {}


def plist(ex):
    return sheet_words.parts(doc(ex))


def part_of(label, parts):
    return sheet_words.label_part(label, parts)


def largest_size_mm(part):
    """The part's largest rough size in mm (its size_mm), or None."""
    vals = [float(v) for v in (part or {}).get("size_mm", {}).values() if isinstance(v, (int, float))]
    return max(vals) if vals else None


def device_entry(d, device_id):
    for dev in (d or {}).get("devices") or []:
        if dev.get("id") == device_id:
            return dev
    return None


def device_size(dev):
    """(width, depth, height) mm of a device entry's rough size, or None."""
    s = (dev or {}).get("size_mm")
    if isinstance(s, dict) and all(isinstance(s.get(k), (int, float)) for k in ("width", "depth", "height")):
        return float(s["width"]), float(s["depth"]), float(s["height"])
    return None


# ---------------------------------------------------------------- colour of the pointed spot (step 11)
def colours_of(part):
    """Every colour the part's own entry names (its how_to_tell states and any colour in its where, how_to_tell,
    notes or moves words)."""
    out = set((part or {}).get("colours", {}).values())
    for key in ("where", "how_to_tell", "notes", "moves", "direction"):
        for c, _ in colour_words.words(str((part or {}).get(key) or "")):
            out.add(c)
    return out


def floor_gains(img, keep_out=()):
    """Grey-world gains from the frame outside the boxes in keep_out ([u0, v0, u1, v1] each): the floor and the
    surroundings stand in for a grey reference, the pointed part and the robot's own hand left out."""
    a = np.asarray(img)
    H, W = a.shape[:2]
    mask = np.ones((H, W), bool)
    for b in keep_out:
        if b is None:
            continue
        u0, v0, u1, v1 = (int(round(x)) for x in b)
        mask[max(0, v0):max(0, v1), max(0, u0):max(0, u1)] = False
    if mask.sum() < 0.1 * H * W:
        mask[:] = True
    return colour_words.balance(a, mask)


def spot_colour(img, px, gains=None, size=L.PART_CHECK_PX):
    a = np.asarray(img)
    r = size // 2
    u, v = int(round(px[0])), int(round(px[1]))
    patch = a[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
    if patch.size == 0:
        return None
    return colour_words.name(patch, gains)


def part_check(colour, label, parts):
    """The step 11 warning, or None. It never refuses."""
    if not parts or label is None or str(label).strip().lower().startswith("other:"):
        return None
    part = part_of(label, parts)
    if part is None:
        names = sorted({p["name"] for p in parts if p["name"]})
        return f"robot://devices names no part {str(label)[:40]}; its parts: {', '.join(names)}"
    if colour is None:
        return None
    mine = colours_of(part)
    if any(colour_words.near(colour, c) for c in mine):
        return None
    for other in parts:
        if other is part or other["device"] != part["device"]:
            continue
        for state, c in other.get("colours", {}).items():
            if c == colour or colour_words.near(c, colour):
                return (f"this spot is {colour}; robot://devices says the {other['name']} is {c} when {state}: it "
                        f"may be the {other['name']}, not the {part['name']}")
    return None


# ---------------------------------------------------------------- relations (step 12)
def relation(p_mm, q_mm, label):
    """Where p is from the located point q (label) in side words, for example "33 mm below the lamp, toward the
    robot"; None when they are under RELATION_MIN_MM apart."""
    v = np.asarray(p_mm, dtype=float) - np.asarray(q_mm, dtype=float)
    n = float(np.linalg.norm(v))
    if n < RELATION_MIN_MM:
        return None
    h = np.array([v[0], v[1], 0.0])
    hw = side_words.words(h, q_mm) if float(np.linalg.norm(h)) >= RELATION_MIN_MM else None
    main = side_words.words(v, q_mm)
    if main in ("up", "down"):
        out = f"{n:.0f} mm {'above' if main == 'up' else 'below'} the {label}"
        if hw and float(np.linalg.norm(h)) >= n / 3.0:
            out += f", {hw}"
        return out
    out = f"{n:.0f} mm from the {label}, {main}"
    if abs(float(v[2])) >= max(RELATION_MIN_MM, n / 3.0):
        out += ", higher" if v[2] > 0 else ", lower"
    return out


def relations(p_mm, others, candidate=None):
    """Up to RELATIONS_MAX relations to other located entries ({label, point_mm, data}) on the same device: the same
    survey candidate when both name one, else within RELATION_NEAR_MM; nearest first."""
    rows = []
    for e in others:
        q = e.get("point_mm")
        if q is None:
            continue
        c2 = (e.get("data") or {}).get("candidate")
        dist = float(np.linalg.norm(np.asarray(p_mm, dtype=float) - np.asarray(q, dtype=float)))
        same = (candidate is not None and c2 is not None and c2 == candidate)
        if not same and (candidate is not None and c2 is not None):
            continue
        if not same and dist > RELATION_NEAR_MM:
            continue
        r = relation(p_mm, q, e.get("label", "point"))
        if r:
            rows.append((dist, r))
    rows.sort(key=lambda x: x[0])
    return [r for _, r in rows[:RELATIONS_MAX]]


# ---------------------------------------------------------------- survey fits and boxes
def fits(cand, d, floor_z_mm):
    """{value: null, band: bool, score, footprint, says, short, device} of a survey candidate {size_mm [sx, sy],
    top_mm} against each device's rough size (or its largest part's): the sorted horizontal sizes against the
    sorted width and depth, the height against top_mm - floor_z_mm; score the largest |ln ratio|, footprint the
    largest |ln ratio| of the two horizontal sizes, band whether every ratio lies in FITS_LO to FITS_HI. With
    several devices the best score wins.
    THE VERDICT IS WITHDRAWN: value is always null. A survey box is a rough measure (a box read from above takes in
    what stands round a thing and reads its height off the tallest point seen), and where it was measured the band
    did not tell the device from other things: the true device's own box fell outside it on most cells, and on
    cluttered ones another thing's box fell inside it. So neither "fits" nor "does not fit" is evidence; the sizes
    are given as what was measured, and the survey names where to look first, not what the device is."""
    try:
        sx, sy = sorted(float(v) for v in cand["size_mm"][:2])
        height = float(cand["top_mm"]) - float(floor_z_mm)
    except (KeyError, TypeError, ValueError):
        return {"value": None, "score": None, "says": "no measured size"}
    best = None
    for dev in (d or {}).get("devices") or []:
        size = device_size(dev)
        if size is None:
            parts = [p for p in sheet_words.parts({"devices": [dev]})]
            big = max((largest_size_mm(p) or 0.0 for p in parts), default=0.0)
            if big <= 0.0:
                continue
            size = (big, big, big)
        w, dd, hh = size
        rw, rd = sorted((w, dd))
        ratios = [sx / max(rw, 1e-6), sy / max(rd, 1e-6), max(height, 1e-6) / max(hh, 1e-6)]
        score = max(abs(math.log(max(r, 1e-6))) for r in ratios)
        inside = [L.FITS_LO <= r <= L.FITS_HI for r in ratios]
        ok = all(inside)
        foot = max(abs(math.log(max(r, 1e-6))) for r in ratios[:2])
        name = dev.get("name") or dev.get("id") or "device"
        rough = f"rough {w:.0f} x {dd:.0f} x {hh:.0f} mm"
        meas = f"measured {sx:.0f} x {sy:.0f} x {max(height, 0.0):.0f}"
        if ok:
            says = f"{meas} mm, near the {name}'s {rough}; a survey box is a rough measure: size is not proof"
        else:
            off = "its height" if all(inside[:2]) else ("its footprint" if inside[2] else "its footprint and height")
            says = (f"{meas} mm; {off} reads outside the {name}'s {rough}, which a survey box often does for the "
                    "device itself: size does not rule it out")
        short = f"{sx:.0f}x{sy:.0f}x{max(height, 0.0):.0f} vs {w:.0f}x{dd:.0f}x{hh:.0f} mm"
        row = {"value": None, "band": bool(ok), "score": round(score, 3), "footprint": round(foot, 3),
               "says": says, "short": short, "device": dev.get("id")}
        if best is None or row["footprint"] < best["footprint"]:
            best = row
    return best or {"value": None, "score": None, "says": "no rough size"}   # the sheet gives none (or none is served)


def survey_post(ex, result):
    """The survey post hook body: fits per candidate, best_fit and said (when the survey has none), the boxes and
    the candidates in ex.state["sight"], the sheet's tile map. Adds keys; never changes the survey's own."""
    if not isinstance(result, dict) or "refused" in result:
        return None
    st = ex.state["sight"]
    d = doc(ex)
    z = float(ex.cfg.get("floor_z_m") or 0.0) * MM
    cands = result.get("candidates") or []
    errs = ((ex.state.get("survey") or {}).get("last") or {}).get("err_mm") or {}     # the survey's own box errors
    served = bool((d or {}).get("devices"))     # no device served (knowledge none): nothing to compare, no fits
    kept = []
    for c in cands:
        f = fits(c, d, z) if served else {"value": None, "score": None, "says": "no rough size"}
        if not served:
            kept.append({"id": c.get("id"), "where_mm": c.get("where_mm"), "size_mm": c.get("size_mm"),
                         "top_mm": c.get("top_mm"), "floor_z_mm": z, "device": None, "seen_in": c.get("seen_in"),
                         "err_mm": errs.get(c.get("id")), "fits": f})
            continue
        # each candidate carries the short form (the survey result stays brief); best_fit and ex.state the sentence
        c["fits"] = {"value": f["value"], "score": f["score"], "says": f.get("short", f["says"])}
        kept.append({"id": c.get("id"), "where_mm": c.get("where_mm"), "size_mm": c.get("size_mm"),
                     "top_mm": c.get("top_mm"), "floor_z_mm": z, "device": f.get("device"), "seen_in": c.get("seen_in"),
                     "err_mm": errs.get(c.get("id")),     # the survey's own two-view error of that box [xy, z]
                     "fits": {"value": f["value"], "band": f.get("band"), "score": f["score"],
                              "footprint": f.get("footprint"), "says": f["says"]}})
    st["candidates"] = kept
    st["surveys"] = st.get("surveys", 0) + 1
    if not served:
        _boxes(ex, kept)
        _survey_sheet(ex, result)
        return result
    # size does not tell which thing is the device (see fits): the thing nearest the device's footprint is named
    # as the place to look first, marked not sure, with the others in the same order; nothing is ruled out
    things = f"survey found {len(kept)} raised thing{'s' if len(kept) != 1 else ''}"
    near = [c for c in kept if (c["fits"] or {}).get("footprint") is not None]
    near.sort(key=lambda c: (c["fits"]["footprint"], float(np.hypot(*(c["where_mm"] or [0, 0])))))
    if near:
        b = near[0]
        result["best_fit"] = {"candidate": b["id"], "says": b["fits"]["says"], "sure": False,
                              "others": [c["id"] for c in near[1:]],
                              "call": {"tool": "view", "args": {"target": b["id"], "side": "robot_side"},
                                       "why": "look at it: size alone does not tell which thing is the device"}}
        if len(near) == 1:
            said = f"{things}: {b['id']}; its size is no proof that it is the device; a look shows it (0 control periods)"
        else:
            said = (f"{things}; size does not settle which is the device: {b['id']} is nearest its footprint, "
                    f"then {', '.join(c['id'] for c in near[1:3])}; a look shows it (0 control periods)")
    else:
        result["best_fit"] = None
        said = f"{things}; none has a measured size to compare" if kept else \
            "survey found no raised thing on the floor it saw"
    if "said" not in result:
        result["said"] = said
    _boxes(ex, kept)
    _survey_sheet(ex, result)
    return result


def _boxes(ex, kept):
    """A World box per candidate when the World knows the kind (C6); the boxes stay in ex.state either way."""
    try:
        from rcpx.context import KINDS
    except ImportError:
        return
    if "box" not in KINDS:
        return
    for c in kept:
        if c.get("where_mm") is None or c.get("size_mm") is None:
            continue
        data = {"centre_mm": c["where_mm"][:2], "size_mm": c["size_mm"][:2], "top_mm": c["top_mm"],
                "source": "survey", "candidate": c["id"]}
        ex.world.add("box", c["id"], point_mm=[float(c["where_mm"][0]), float(c["where_mm"][1]),
                                                float(c["top_mm"])], source="survey", data=data)


def _survey_sheet(ex, result):
    """Register the survey sheet's tile map (survey_see.sheet: SURVEY_SHEET_COLS to a row, SURVEY_TILE_W wide,
    the tile height in the frame's proportion), checked against the saved sheet's own size."""
    st = ex.state["sight"]
    path = result.get("sheet")
    if not isinstance(path, str):
        return                       # a survey that took no picture has no sheet to point in
    n = int((ex.state.get("survey") or {}).get("runs") or st.get("surveys") or 0)
    sid = f"survey_{n}"
    tiles = [v.get("look_id") for v in result.get("views") or [] if v.get("look_id")]
    W, H = int(ex.model.cam_w), int(ex.model.cam_h)
    tw = L.SURVEY_TILE_W
    th = int(round(H * tw / float(W)))
    cols = L.SURVEY_SHEET_COLS
    entry = {"size": None, "cols": cols, "tile_px": [tw, th], "tiles": tiles, "path": path, "known": False}
    if isinstance(path, str) and tiles:
        try:
            from PIL import Image
            with Image.open(path) as im:
                sw, sh = im.size
            rows = int(math.ceil(len(tiles) / float(cols)))
            entry["size"] = [sw, sh]
            entry["known"] = (sw == min(cols, len(tiles)) * tw and sh == rows * th)
        except OSError:
            entry["known"] = False
    st["sheets"][sid] = entry
    result["sheet_id"] = sid


def resize_factor(size):
    """The loop's resize rule: an image whose longest side is over HARNESS_MAX_SIDE_PX is scaled to it."""
    w, h = size
    big = max(int(w), int(h))
    return 1.0 if big <= HARNESS_MAX_SIDE_PX else HARNESS_MAX_SIDE_PX / float(big)


def sheet_pixel(sheets, sheet_id, px, frame_w, frame_h):
    """(look id, (u, v) in that look's frame, None) or (None, None, why) for a pixel of a contact sheet as the
    operator saw it (after the loop's resize rule)."""
    e = (sheets or {}).get(str(sheet_id))
    if e is None:
        return None, None, "unknown"
    if not e.get("known") or not e.get("size"):
        return None, None, "layout"
    s = resize_factor(e["size"])
    u, v = float(px[0]) / s, float(px[1]) / s
    sw, sh = e["size"]
    if not (0 <= u < sw and 0 <= v < sh):
        return None, None, "outside"
    tw, th = e["tile_px"]
    k = int(v // th) * int(e["cols"]) + int(u // tw)
    if k >= len(e["tiles"]):
        return None, None, "outside"
    uu = (u - (u // tw) * tw) * frame_w / float(tw)
    vv = (v - (v // th) * th) * frame_h / float(th)
    return e["tiles"][k], (uu, vv), None


__all__ = ["colours_of", "device_entry", "device_size", "doc", "fits", "floor_gains", "largest_size_mm",
           "part_check", "part_of", "plist", "relation", "relations", "resize_factor", "sheet_pixel", "spot_colour",
           "survey_post"]
