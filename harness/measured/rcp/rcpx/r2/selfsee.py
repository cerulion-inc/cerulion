# SPDX-License-Identifier: AGPL-3.0-only
"""Package selfsee: the robot's own body and the view, in words, on every look.

Weak operators took their own mount for the device (r1b ph/02, 23, 29, 33, 35) and read a top view as a front view
(ph/03, 13, 15). This package adds no tool and moves nothing:

- view block (a post hook on every primitive whose result carries a NEW look: a top-level look_id, or a look dict
  with a look_id, as look, standard_look, look_after and wait_for_light give): result["view"], right after look_id,
  {says, kind, elevation_deg, roll_deg, up, right, own_body, below_tip_px}, at most VIEW_BLOCK_MAX_BYTES as JSON.
  kind by the optical axis' elevation; up and right are image up and image right in side words (read at the floor
  point under the image centre); roll_deg the angle from image up to world up projected (top views: to "away from
  the robot"), positive toward image right; own_body {hand | arm | base: [share, box_px]} from a 16 px grid of
  first_hit rays; below_tip_px the floor point under the tip, projected. A look whose result names a look the call
  was given (mark, crop) is not a new look and gets nothing. To stay within the bound, the boxes leave own_body
  first (the largest group's last), then the position words leave says (the largest group's last), then the roll's
  reason, then image right from says; a still oversized head drops its height and roll clauses. The right and
  roll keys stay, and the look's camera pose carries the height; says itself always stays.
- drawing, on the look's annotated frame in place (annotated_axes when the frames package drew it, the one the frame
  list sends, else annotated; the raw frame is never touched): outlines only, labelled own hand, own arm, own base,
  plus one arrow ("toward robot" on top views, "up" otherwise); labels and the arrow stay outside every survey
  candidate box projected into the frame and outside the central DRAW_CENTRE_FRAC of the image (that share of its
  area), and outline pixels inside a candidate box are left out, so nothing is drawn over a thing on the table.
- survey: the view words {kind, up} of each view row; when every row reads the same (the survey turns one view about
  the base axis) they are said once, as the survey's own view {kind, up} just before views (a survey-level view is
  every row's), else each row gains its own view; the candidates are kept for the drawing.
- mark (pre hook, after the frames package turned px, norm1000_yx or norm_xy into u and v): refused own_body when
  first_hit at (u, v) is the robot's own base, arm or hand (region B, the mount, included), or when MARK_RING_HITS of
  the 8 pixels MARK_RING_PX around it are. Nothing is stored.
  Where the installation states a camera pose uncertainty (robot://body camera.pose_uncertainty), the body model's
  outline in a picture is known only to edge_band px (f x (tan deg + mm / distance)): a refused pixel within that
  band of the outline reads "may show" (never "shows"), with own_body.sure false and band_px, and the hint says how
  far to point; a STORED mark with MARK_RING_HITS of the 8 pixels band px around it on the body gains near_own_body
  (a sentence; the mark stands). Without a stated uncertainty nothing changes.
- triangulate (post hook): when on_body(point_mm, the q of every look in rays[].look_id) names a group, the located
  entry is marked stale ("own_body") and the result becomes refusal own_body.

For other packages, ex.state["selfsee"]:
    first_hit(q, u, v)        {what: own_base | own_arm | own_hand | world | floor | none, dist_mm, floor_mm,
                              source: model | region_b | world | floor | none}
    first_hits(q, pixels)     the same for a list of (u, v), one ray cast
    on_body(p_mm, qs)         None or a group name: inside region B; inside a non-tool capsule grown 5 mm at a pose
                              of qs; or within 15 mm of one and not within 10 mm of a stored contact
    on_body_why(p_mm, qs)     (group or None, how: mount | inside | beside | "beside a stored contact" | None)
    view(q)                   the view block at joint pose q (no drawing)
    mask(q, step_px=16)       [(group, box_px, share)], largest share first
    edge_band(dist_mm)        the px by which the body outline may be off at that distance (0: none stated)
    near_note(q, u, v)        the near_own_body sentence of a pixel, or None
    candidates                the latest survey candidates (the drawing keeps out of their boxes)
    body                      the selfsee_body.Body itself (grid, cast, region_b, in_b)
first_hit's "world" is a World entry of kind box (centre_mm, size_mm, top_mm) met before the floor; none exists until
a package stores boxes. Everything comes from the body model at the look's stored joint pose and robot://body's
standing surface; nothing reads the picture or ground truth, and no robot or device is named.

What the session around it must do (not this package): the brief layer keeps view
(rcpx/surface_brief.py KEEP_BLOCKS gains "view" and FIRST shows "view" after "look_id"; today a brief standard look
over 1,800 bytes leaves view out first, as the largest block); the image an answer names is the newest annotated
frame by "supersedes" (rcpx.surface_args.frame_paths already reads it), so the
chat loop sends the annotated frame this package drew on; a LEAN_PROFILE keep list holds "view".
"""

import json
import math
import inspect
from rcpx import bounds
import os
from collections import Counter

import numpy as np
from PIL import Image

from rcpx import r3_limits as L
from rcpx.context import refusal
from rcpx.r2 import selfsee_body as SB
from rcpx.r2 import selfsee_words as SW

ID = "I33"
MM = 1000.0
PRACTICES = ("Every look says which way the picture faces and which of it is the robot itself (view).")
HINT = "point at a thing that stands apart from the robot"
HINT_SURVEY = "; survey lists the things on the table: survey {}"
LOOK_ARGS = ("look_id", "look", "like")      # arguments that name a look the call was given, not one it took


def _state(ex):
    return ex.state.setdefault("selfsee", {"known": set(), "candidates": [], "views": 0, "drawn": 0, "drawn_on": {},
                                           "marks_refused": 0, "points_refused": 0, "notes": []})


def _hint(ex):
    names = ex.primitive_names() if callable(getattr(ex, "primitive_names", None)) else []
    if "survey" not in names:
        return HINT
    from rcpx import price_words
    return HINT + HINT_SURVEY + price_words.priced(ex, "survey")


# ---------------------------------------------------------------- the view block and the drawing
def view_at(ex, body, q):
    """(view block, grid codes, rows) at joint pose q."""
    us, vs, codes = body.grid(q)
    W, H = int(ex.model.cam_w), int(ex.model.cam_h)
    rows, cents = SB.mask_from_grid(us, vs, codes, L.SELF_GRID_PX, W, H)
    floor, _, _ = body.surface()
    tip = np.asarray(ex.model.fk_tip(np.asarray(q, dtype=float))[0], dtype=float) * MM
    return SW.block(ex.model, q, rows, cents, floor, tip), codes, rows


def _avoid_boxes(ex, body, q):
    """Pixel boxes the drawing keeps out of: every survey candidate (the sight package's when it keeps them, else
    this package's copy of the latest survey) and every World box, projected into the frame at q."""
    floor, _, _ = body.surface()
    sight = ex.state.get("sight")
    cands = (sight or {}).get("candidates") if isinstance(sight, dict) else None
    if not cands:
        cands = _state(ex)["candidates"]
    out = []
    for c in cands or ():
        try:
            b = SW.box_px(ex.model, q, c["where_mm"], c["size_mm"], floor, c["top_mm"])
        except (KeyError, TypeError, ValueError, IndexError):
            continue
        if b is not None:
            out.append(b)
    for e in ex.world.query(kind="box", include_stale=False):
        d = e.get("data") or {}
        try:
            b = SW.box_px(ex.model, q, d["centre_mm"], d["size_mm"], floor, d["top_mm"])
        except (KeyError, TypeError, ValueError, IndexError):
            continue
        if b is not None:
            out.append(b)
    return out


def _source_image(res):
    """(key, path) of the newest annotated frame of a look result: the one the frame list would send."""
    for key in ("annotated_axes", "annotated"):
        p = res.get(key)
        if isinstance(p, str) and os.path.isfile(p):
            return key, p
    return None, None


def _draw(ex, body, res, q, codes, rows, view):
    """Draw the outlines, labels and arrow on the look's annotated frame, in place (module text). The result is
    returned unchanged: the frame keeps its path and key."""
    key, src = _source_image(res)
    if src is None:
        return res
    try:
        img = np.asarray(Image.open(src).convert("RGB"))
    except OSError:
        return res
    W, H = int(ex.model.cam_w), int(ex.model.cam_h)
    if img.shape[0] != H or img.shape[1] != W:
        return res
    masks = SW.group_masks(codes, {g: SB.CODES[g] for g in SB.GROUPS}, W, H)
    pc, Rc = SW.camera(ex.model, q)
    floor, _, _ = body.surface()
    ref = SW.reference(pc, Rc, floor)
    arrow = SW.arrow_for(ex.model, q, ref, view["kind"] == "top view")
    out_img, drawn, notes = SW.draw(img, masks, rows, arrow, _avoid_boxes(ex, body, q))
    Image.fromarray(out_img).save(src)
    st = _state(ex)
    st["drawn"] += 1
    st["drawn_on"][str(res.get("look_id"))] = key
    if notes:
        st["notes"] = (st["notes"] + [f"{res.get('look_id')}: {n}" for n in notes])[-20:]
    return res


def _with_view(ex, body, res, look_id):
    _, rec = ex._look_rec(look_id)
    q = np.asarray(rec["q"], dtype=float)
    view, codes, rows = view_at(ex, body, q)
    out = {}
    for k, v in res.items():
        out[k] = v
        if k == "look_id":
            out["view"] = view
    _state(ex)["views"] += 1
    return _draw(ex, body, out, q, codes, rows, view)


def _given_looks(kwargs):
    return {str(v) for k, v in (kwargs or {}).items() if k in LOOK_ARGS and isinstance(v, (str, int))}


def _post_any(ex, body, name, kwargs, result):
    st = _state(ex)
    now = set(getattr(ex, "_looks", {}) or {})
    new = now - st["known"]
    st["known"] = now
    if not isinstance(result, dict) or "refused" in result:
        return None
    if name == "survey":
        return _post_survey(ex, body, result)
    given = _given_looks(kwargs)
    out = None
    lid = result.get("look_id")
    if isinstance(lid, str) and lid in new and lid not in given and "view" not in result:
        out = _with_view(ex, body, result, lid)
    lk = result.get("look")
    if (isinstance(lk, dict) and "refused" not in lk and isinstance(lk.get("look_id"), str)
            and lk["look_id"] in new and "view" not in lk):
        out = dict(out or result)
        out["look"] = _with_view(ex, body, lk, lk["look_id"])
    return out


def kind_and_up(ex, body, q):
    pc, Rc = SW.camera(ex.model, q)
    floor, _, _ = body.surface()
    ref = SW.reference(pc, Rc, floor)
    return {"kind": SW.kind_of(SW.elevation_deg(Rc)), "up": SW.view_dir_words(Rc[:, 1], ref)}


def _post_survey(ex, body, result):
    st = _state(ex)
    cands = []
    for c in result.get("candidates") or ():
        if isinstance(c, dict) and all(k in c for k in ("where_mm", "size_mm", "top_mm")):
            cands.append({k: c.get(k) for k in ("id", "where_mm", "size_mm", "top_mm")})
    if result.get("candidates") is not None:
        st["candidates"] = cands
    rows = result.get("views")
    if not isinstance(rows, list):
        return None
    words = []
    for row in rows:
        lid = row.get("look_id") if isinstance(row, dict) else None
        _, rec = ex._look_rec(lid) if lid is not None else (None, None)
        words.append(None if rec is None or "view" in row else kind_and_up(ex, body, np.asarray(rec["q"], dtype=float)))
    given = [w for w in words if w is not None]
    if not given:
        return None
    if all(w == given[0] for w in given):
        # the survey turns one view about the base axis: every row reads the same, so it is said once
        out = {}
        for k, val in result.items():
            if k == "views":
                out["view"] = dict(given[0])
            out[k] = val
        return out
    new_rows = []
    for row, w in zip(rows, words):
        if w is None:
            new_rows.append(row)
            continue
        r = {}
        for k, val in row.items():
            r[k] = val
            if k == "look_id":
                r["view"] = w
        new_rows.append(r)
    return {**result, "views": new_rows}


# ---------------------------------------------------------------- mark and triangulate
def _mount_words(body):
    _, _, h = body.surface()
    if h is None:
        return "its mount, fastened at its foot"
    return f"its mount plate, {2.0 * h:g} mm square at its foot"


def _shows(body, group, source):
    if group == "own_base":
        return f"the robot's own base ({_mount_words(body)})" if source == "region_b" else \
            "the robot's own base (the part fixed at its foot)"
    if group == "own_hand":
        return "the robot's own hand (its tool and the link the camera rides on)"
    return "the robot's own arm"


def edge_band(ex, dist_mm):
    """(band px, mm, deg): how far off the body model's outline may lie in a picture, from the camera pose
    uncertainty the body model states (robot://body camera.pose_uncertainty {mm, deg}, read through
    r3_limits.inputs) seen at dist_mm: f x (tan deg + mm / distance), rounded up. (0, 0, 0) when none is stated
    (then the outline is the model's and nothing below changes). No typed limit: the band is the robot's own
    stated measurement."""
    fn = L.inputs
    facts = fn(bounds.of(ex)) if next(iter(inspect.signature(fn).parameters), None) in ("B", "bounds") else fn(ex)
    mm, deg = float(facts.get("pu_mm") or 0.0), float(facts.get("pu_deg") or 0.0)
    if (mm <= 0.0 and deg <= 0.0) or not dist_mm or float(dist_mm) <= 0.0:
        return 0, mm, deg
    px = float(ex.model.f_px) * (math.tan(math.radians(deg)) + mm / float(dist_mm))
    return int(math.ceil(px)), mm, deg


def _ring(u, v, r, W, H):
    """The 8 pixels r px around (u, v) that lie in the frame."""
    out = [(u + du, v + dv) for dv in (-r, 0, r) for du in (-r, 0, r) if (du, dv) != (0, 0)]
    return [(a, b) for a, b in out if 0.0 <= a < W and 0.0 <= b < H]


def _disc(u, v, r, W, H):
    """The pixels of a MARK_RING_PX grid within r px of (u, v) that lie in the frame (the centre left out)."""
    k = int(r // L.MARK_RING_PX)
    out = []
    for j in range(-k, k + 1):
        for i in range(-k, k + 1):
            du, dv = i * L.MARK_RING_PX, j * L.MARK_RING_PX
            if (i, j) != (0, 0) and du * du + dv * dv <= r * r and 0.0 <= u + du < W and 0.0 <= v + dv < H:
                out.append((u + du, v + dv))
    return out


def _band_words(band, mm, deg):
    return (f"the camera's pose is known to {mm:g} mm and {deg:g} deg, so the body model's outline may be about "
            f"{band} px off in this picture")


def near_note(ex, body, q, u, v):
    """For a mark that was stored: a sentence when the pixel lies within the outline's uncertainty band of the
    robot's own body in the body model (edge_band; None when no camera uncertainty is stated or nothing of the
    body is that near). The mark stands: the picture may still show a thing beside the hand."""
    W, H = int(ex.model.cam_w), int(ex.model.cam_h)
    if edge_band(ex, 1.0)[0] <= 0 or body.first_hit(q, u, v)["what"] in SB.OWN:
        return None                      # no uncertainty stated (nothing is cast), or a pixel the model refuses
    # the band depends on how far the body is: first the widest band any own-body cell of this look gives (its
    # nearest), to find the body near the pixel; then the band at that part's own distance
    us, vs = np.meshgrid(np.arange(L.SELF_GRID_PX / 2.0, W, L.SELF_GRID_PX),
                         np.arange(L.SELF_GRID_PX / 2.0, H, L.SELF_GRID_PX))
    code, dist, _, _ = body.cast(q, us.reshape(-1), vs.reshape(-1))
    own = code >= SB.CODES["own_base"]
    if not bool(own.any()):
        return None
    wide, _, _ = edge_band(ex, float(dist[own].min()))
    if wide <= 0:
        return None
    near = [h["dist_mm"] for h in body.first_hits(q, _disc(u, v, wide, W, H)) if h["what"] in SB.OWN and h["dist_mm"]]
    if not near:
        return None
    band, mm, deg = edge_band(ex, min(near))
    hits = [h for h in body.first_hits(q, _disc(u, v, band, W, H)) if h["what"] in SB.OWN]
    if len(hits) < L.MARK_RING_HITS:
        return None
    count = Counter(h["what"] for h in hits)
    group = sorted(count, key=lambda g: (-count[g], SB.GROUPS.index(g)))[0]
    return (f"this pixel may be the robot's own {SW.WORD[group]}: the body model puts it within {band} px of this "
            f"pixel, and {_band_words(band, mm, deg)}; if it is the robot, mark a thing that stands apart from it")


def _post_mark(ex, body, name, kwargs, result):
    if not isinstance(result, dict) or "refused" in result or "near_own_body" in result:
        return None
    key, rec = ex._look_rec((kwargs or {}).get("look_id"))
    if rec is None or rec.get("can_see") == "dark":
        return None
    try:
        u, v = float(kwargs["u"]), float(kwargs["v"])
    except (KeyError, TypeError, ValueError):
        return None
    W, H = int(ex.model.cam_w), int(ex.model.cam_h)
    if not (0.0 <= u < W and 0.0 <= v < H):
        return None
    note = near_note(ex, body, np.asarray(rec["q"], dtype=float), u, v)
    return None if note is None else {**result, "near_own_body": note}


def _pre_mark(ex, body, name, kwargs):
    key, rec = ex._look_rec(kwargs.get("look_id"))
    if rec is None or rec.get("can_see") == "dark":
        return None                     # the primitive itself refuses an unknown or dark look
    try:
        u, v = float(kwargs["u"]), float(kwargs["v"])
    except (KeyError, TypeError, ValueError):
        return None
    W, H = int(ex.model.cam_w), int(ex.model.cam_h)
    if not (0.0 <= u < W and 0.0 <= v < H):
        return None
    r = L.MARK_RING_PX
    ring = [(u + du, v + dv) for dv in (-r, 0, r) for du in (-r, 0, r) if (du, dv) != (0, 0)]
    hits = body.first_hits(np.asarray(rec["q"], dtype=float), [(u, v)] + ring)
    centre = hits[0]
    around = [h for h in hits[1:] if h["what"] in SB.OWN]
    pix = f"pixel ({u:g}, {v:g}) in {key}"
    margin = False
    band, hint = 0, None
    q = np.asarray(rec["q"], dtype=float)
    if centre["what"] in SB.OWN:
        group, source = centre["what"], centre["source"]
        why = f"{pix} shows {_shows(body, group, source)}, not a thing in the world"
        band, pu_mm, pu_deg = edge_band(ex, centre["dist_mm"])
        if band > 0 and any(h["what"] not in SB.OWN for h in body.first_hits(q, _ring(u, v, band, W, H)) or ()):
            # the camera's stated pose uncertainty puts this pixel within the outline's band: the model cannot say
            # the picture SHOWS the robot here, only that it may
            why = (f"{pix} may show {_shows(body, group, source)}: the body model puts the robot there, but "
                   f"{_band_words(band, pu_mm, pu_deg)} and this pixel is within that of the outline")
            hint = (f"point at least {band} px farther from the robot's own body, or move the hand aside and look "
                    "again")
        else:
            band = 0
        if source == "region_b":
            _, _, half = body.surface()
            if half is not None and centre["dist_mm"] is not None:
                pc, directions = body.rays(np.asarray(rec["q"], dtype=float), [u], [v])
                entry = pc * MM + float(centre["dist_mm"]) * directions[0]
                margin = bool(np.any(np.abs(entry[:2]) > half))
                if margin:
                    band, hint = 0, None
                    why = (f"{pix} is at the edge of the robot's own mount (within {L.MOUNT_MARGIN_MM:g} mm of its "
                           f"plate, {2.0 * half:g} mm square at its foot): too close to the robot to be told from it")
    elif len(around) >= L.MARK_RING_HITS:
        count = Counter(h["what"] for h in around)
        group = sorted(count, key=lambda g: (-count[g], SB.GROUPS.index(g)))[0]
        source = next(h["source"] for h in around if h["what"] == group)
        why = (f"{pix} is at the edge of {_shows(body, group, source)}: {len(around)} of the 8 pixels {r} px around "
               "it show the robot, not a thing in the world")
        dist = min((h["dist_mm"] for h in around if h["dist_mm"]), default=None)
        band, pu_mm, pu_deg = edge_band(ex, dist)
        if band > 0:
            why = (f"{pix} may be at the edge of {_shows(body, group, source)}: {len(around)} of the 8 pixels {r} px "
                   f"around it lie on the robot in the body model, and {_band_words(band, pu_mm, pu_deg)}")
            hint = (f"point at least {band} px farther from the robot's own body, or move the hand aside and look "
                    "again")
    else:
        return None
    _state(ex)["marks_refused"] += 1
    full_hint = _hint(ex) if hint is None else hint + _hint(ex)[len(HINT):]
    return {**refusal("own_body", why, full_hint),
            "own_body": {"group": group, "source": source, "look_id": key, "px": [u, v],
                         "ring_hits": len(around), **({"margin": True} if margin else {}),
                         **({"sure": False, "band_px": band} if band > 0 else {})}}


def _post_triangulate(ex, body, name, kwargs, result):
    if not isinstance(result, dict) or "refused" in result or result.get("point_mm") is None:
        return None
    qs = []
    for ray in result.get("rays") or ():
        _, rec = ex._look_rec(ray.get("look_id")) if isinstance(ray, dict) else (None, None)
        if rec is not None:
            qs.append(np.asarray(rec["q"], dtype=float))
    contacts = ex.world.query(kind="contact", include_stale=False)
    group, how = body.why_on_body(result["point_mm"], qs, contacts)
    if group is None:
        return None
    eid = result.get("id")
    stale = ex.world.mark_stale([eid], "own_body", source="selfsee")["stale"] if eid else []
    x, y, z = (float(c) for c in result["point_mm"])
    where = "mount" if how == "mount" else SW.WORD[group]
    why = (f"the point ({x:.0f}, {y:.0f}, {z:.0f}) lies on the robot's own {where}: the marks show the robot, not a "
           "device")
    _state(ex)["points_refused"] += 1
    return {**refusal("own_body", why, _hint(ex)),
            "own_body": {"group": group, "how": how, "id": eid, "stale": stale, "label": result.get("label")}}


# ---------------------------------------------------------------- setup
def setup(ex):
    st = _state(ex)
    body = SB.Body(ex)
    st["body"] = body
    st["first_hit"] = lambda q, u, v: body.first_hit(q, u, v)
    st["first_hits"] = lambda q, pixels: body.first_hits(q, pixels)
    st["on_body"] = lambda p_mm, qs: body.on_body(p_mm, qs, ex.world.query(kind="contact", include_stale=False))
    st["on_body_why"] = lambda p_mm, qs: body.why_on_body(p_mm, qs,
                                                          ex.world.query(kind="contact", include_stale=False))
    st["view"] = lambda q: view_at(ex, body, np.asarray(q, dtype=float))[0]
    st["mask"] = lambda q, step_px=L.SELF_GRID_PX: body.mask(np.asarray(q, dtype=float), step_px)
    st["known"] = set(getattr(ex, "_looks", {}) or {})
    ex.add_pre(lambda name, kw: _pre_mark(ex, body, name, kw), names=["mark"])
    ex.add_post(lambda name, kw, res: _post_triangulate(ex, body, name, kw, res), names=["triangulate"])
    ex.add_post(lambda name, kw, res: _post_mark(ex, body, name, kw, res), names=["mark"])
    st["edge_band"] = lambda dist_mm: edge_band(ex, dist_mm)[0]
    st["near_note"] = lambda q, u, v: near_note(ex, body, np.asarray(q, dtype=float), u, v)
    ex.add_post(lambda name, kw, res: _post_any(ex, body, name, kw, res))


def view_bytes(view):
    """The size of a view block as the session sends it (JSON)."""
    return len(json.dumps(view))


__all__ = ["ID", "PRACTICES", "kind_and_up", "setup", "view_at", "view_bytes"]
