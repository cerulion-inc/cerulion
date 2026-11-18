# SPDX-License-Identifier: AGPL-3.0-only
"""The rows of the eyes package (SPEC-R5 4.1, 7.4): make_rows asks the vision service when one is declared and
answers from geometry and colour when none is or it does not answer; the robot's own body in a look as polygons;
and the words of a row that come from the picture and the body model.

Level absent, and the fallback of every call the vision service did not answer:
- geometry candidates first: the boxes a survey measured (ex.state["sight"]["candidates"], when that package is
  loaded), the fresh located and contact entries of robot://world, and the look's bright spots, each projected
  into the look with the robot's own model at the look's joint pose;
- then colour regions (rcpx.eyes_service.backends.ColourRegions, no model, the CPU) ranked by the service's own
  rule (rcpx.eyes_service.protocol.select);
- a candidate whose point lies on the robot's own body is dropped.

own_polygons(ex, q): the robot's own body in the look as polygons for the proposer's exclude list. With the
selfsee package: the cells of a 16 px grid whose line of sight first meets the robot (ex.state["selfsee"]
first_hits) at the look's joint pose, merged into row runs, each run a 4 point polygon (the moving links too).
Without it: the base capsule and the mount plate of the robot's own collision model, projected; the note says
that the moving links are not excluded.

Standard library, numpy and PIL only. Nothing here reads simulator ground truth or names a robot or a device.
"""

import numpy as np
from PIL import Image

from rcpx import colour_words, perception, sheet_words
from rcpx.eyes_service import protocol as P
from rcpx.eyes_service.backends import ColourRegions
from rcpx.r2 import eyes_client as C
from rcpx.r2 import eyes_parts as EP

MM = 1000.0
GRID_PX = 16
OWN = ("own_base", "own_arm", "own_hand")
POINT_BOX_PX = 12.0          # half side of the square drawn for a stored point, in a picture 640 px wide
MAX_FRAC = 0.25              # the service's own bound: a region over this share of the picture is no candidate
MIN_PX = 30.0
PATCH_PX = 9                 # the colour of a candidate is read from this square at its point
NOTE_LOCAL = "no vision model: candidates from colour and geometry"
NOTE_NO_SELFSEE = "moving links not excluded"


def frame(rec):
    """The look's own picture as H x W x 3 uint8, or None: the kept array, else the frame file."""
    img = rec.get("img")
    if img is None and isinstance(rec.get("frame"), str):
        try:
            with Image.open(rec["frame"]) as im:
                img = np.asarray(im.convert("RGB"))
        except OSError:
            return None
    if img is None:
        return None
    a = np.asarray(img)
    return np.ascontiguousarray(a[:, :, :3]).astype(np.uint8) if a.ndim == 3 else None


# ---------------------------------------------------------------- the robot's own body in the look
def _hull(points):
    pts = sorted(set((round(float(x), 1), round(float(y), 1)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out

    lower, upper = half(pts), half(list(reversed(pts)))
    return lower[:-1] + upper[:-1]


def _static_polygons(ex, q):
    """The base capsule and the mount plate projected into the look: hulls of their projected surface points."""
    model = ex.model
    shapes = []
    try:
        caps = model.collision_capsules(np.asarray(q, dtype=float))
    except Exception:
        caps = []
    if caps:
        _, p0, p1, r = caps[0]
        p0, p1, r = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float), float(r)
        axis = p1 - p0
        n = float(np.linalg.norm(axis))
        axis = axis / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
        a = np.cross(axis, [1.0, 0.0, 0.0])
        if float(np.linalg.norm(a)) < 1e-6:
            a = np.cross(axis, [0.0, 1.0, 0.0])
        a = a / np.linalg.norm(a)
        b = np.cross(axis, a)
        ring = [np.cos(t) * a + np.sin(t) * b for t in np.linspace(0.0, 2.0 * np.pi, 12, endpoint=False)]
        shapes.append([c + r * d for c in (p0, p1) for d in ring] + [p0 - r * axis, p1 + r * axis])
    h = ex.cfg.get("mount_half_m")
    if h:
        h = float(h)
        shapes.append([np.array([sx * h, sy * h, 0.0]) for sx in (-1, 1) for sy in (-1, 1)])
    polys = []
    for pts in shapes:
        px = [model.project(q, p) for p in pts]
        if any(not (d > 1e-6) for _, _, d in px):
            continue                      # partly behind the camera: its outline in the picture is not a hull
        hull = _hull([(u, v) for u, v, _ in px])
        if len(hull) >= 3:
            polys.append([[u, v] for u, v in hull])
    return polys


def own_polygons(ex, q):
    """(polygons [[u, v], ...] of the robot's own body in the look taken at joint pose q, a note or None)."""
    w, h = int(ex.model.cam_w), int(ex.model.cam_h)
    ss = ex.state.get("selfsee")
    fn = ss.get("first_hits") if isinstance(ss, dict) else None
    one = ss.get("first_hit") if isinstance(ss, dict) else None
    if not callable(fn) and not callable(one):
        return _static_polygons(ex, q), NOTE_NO_SELFSEE
    us = np.arange(GRID_PX / 2.0, w, GRID_PX)
    vs = np.arange(GRID_PX / 2.0, h, GRID_PX)
    pixels = [(float(u), float(v)) for v in vs for u in us]
    try:
        hits = fn(q, pixels) if callable(fn) else [one(q, u, v) for u, v in pixels]
    except Exception:           # a selfsee that cannot answer here: the static body, and the note says so
        return _static_polygons(ex, q), NOTE_NO_SELFSEE
    own = np.array([isinstance(x, dict) and x.get("what") in OWN for x in hits], dtype=bool).reshape(len(vs), len(us))
    polys = []
    for r in range(own.shape[0]):
        c = 0
        while c < own.shape[1]:
            if not own[r, c]:
                c += 1
                continue
            c0 = c
            while c < own.shape[1] and own[r, c]:
                c += 1
            x0, x1 = c0 * GRID_PX, min(w, c * GRID_PX)
            y0, y1 = r * GRID_PX, min(h, (r + 1) * GRID_PX)
            polys.append([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    return polys, None


def on_own_body(ex, q, u, v):
    """The group name when the line of sight through pixel (u, v) at joint pose q first meets the robot itself,
    None when it does not, "unchecked" when nothing here can tell (no selfsee and no static body in the look)."""
    ss = ex.state.get("selfsee")
    fn = ss.get("first_hit") if isinstance(ss, dict) else None
    if callable(fn):
        try:
            hit = fn(q, float(u), float(v))
        except Exception:
            hit = None
        if isinstance(hit, dict):
            return hit.get("what") if hit.get("what") in OWN else None
    polys = _static_polygons(ex, q)
    if any(C.in_polygon(p, float(u), float(v)) for p in polys):
        return "own_base"
    return "unchecked" if not callable(fn) else None


# ---------------------------------------------------------------- candidates without a model
def _row(point, box, source, what=None, polygon=None, area=None):
    box = [float(v) for v in box]
    return {"point_px": [float(point[0]), float(point[1])], "box_px": box,
            "polygon_px": polygon or [[box[0], box[1]], [box[2], box[1]], [box[2], box[3]], [box[0], box[3]]],
            "area_px": int(area if area is not None else max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])),
            "confidence": None, "source": source, "what": what}


def _clip(box, w, h):
    return [max(0.0, min(float(w), box[0])), max(0.0, min(float(h), box[1])),
            max(0.0, min(float(w), box[2])), max(0.0, min(float(h), box[3]))]


def geometry_rows(ex, rec, img):
    """Candidates the robot holds as geometry, projected into the look: survey boxes, stored points, bright spots."""
    model, q = ex.model, np.asarray(rec["q"], dtype=float)
    w, h = int(model.cam_w), int(model.cam_h)
    out = []
    sight = ex.state.get("sight")
    for c in (sight.get("candidates") if isinstance(sight, dict) else None) or ():
        try:
            x, y = (float(v) for v in c["where_mm"][:2])
            sx, sy = (float(v) for v in c["size_mm"][:2])
            top, floor = float(c["top_mm"]), float(c.get("floor_z_mm") or 0.0)
        except (KeyError, TypeError, ValueError):
            continue
        corners = [np.array([x + dx * sx / 2.0, y + dy * sy / 2.0, z]) / MM
                   for dx in (-1, 1) for dy in (-1, 1) for z in (floor, top)]
        px = [model.project(q, p) for p in corners]
        if any(not (d > 1e-6) for _, _, d in px):
            continue
        us, vs = [p[0] for p in px], [p[1] for p in px]
        box = _clip([min(us), min(vs), max(us), max(vs)], w, h)
        area = (box[2] - box[0]) * (box[3] - box[1])
        if area < MIN_PX or area > MAX_FRAC * w * h:
            continue
        out.append(_row([(box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0], box, "geometry",
                        f"survey {c.get('id')}"))
    half = POINT_BOX_PX * w / 640.0
    try:
        entries = ex.world.query(include_stale=False)
    except Exception:
        entries = []
    for e in entries:
        if e.get("kind") not in ("located", "contact") or e.get("point_mm") is None:
            continue
        u, v, d = model.project(q, np.asarray(e["point_mm"], dtype=float) / MM)
        if not (d > 1e-6 and 0 <= u < w and 0 <= v < h):
            continue
        out.append(_row([u, v], _clip([u - half, v - half, u + half, v + half], w, h), "geometry",
                        f"{e.get('id')} ({e.get('label')})"))
    if rec.get("can_see") != "ok" and img is not None:
        for s in perception.bright_spots(img):
            (u, v), r = s["centre_px"], max(4.0, float(s["radius_px"]))
            out.append(_row([u, v], _clip([u - r, v - r, u + r, v + r], w, h), "geometry", "bright spot",
                            area=s.get("area_px")))
    return out


def colour_rows(img, exclude=(), n=16):
    """Colour regions of the picture (no model), ranked and cut by the service's own rule."""
    pic = Image.fromarray(img)
    regions = ColourRegions({}, "cpu").propose(pic)
    cands, _ = P.select(regions, pic.size, int(n), [[(float(u), float(v)) for u, v in p] for p in exclude or ()])
    return C.rows(cands, "local")


def propose(ex, rec, img, exclude=(), n=16):
    """The candidates of a look with no vision model: geometry first, then colour regions; a candidate whose point
    lies on the robot's own body is dropped. (rows, how many were dropped on the body)."""
    geo = geometry_rows(ex, rec, img)
    kept = [r for r in geo if not any(C.in_polygon(p, r["point_px"][0], r["point_px"][1]) for p in exclude or ())]
    return kept + colour_rows(img, exclude, n), len(geo) - len(kept)


# ---------------------------------------------------------------- the words of a row that come from the picture
def colour_at(img, point):
    """The colour word of the square at a pixel (rcpx.colour_words; PATCH_PX at a picture 640 px wide, scaled with
    the width), or None outside the picture. No white balance: the same pixel reads the same word here as in the
    gauge's own colour check of a pointed pixel."""
    r = max(3, int(round(PATCH_PX * img.shape[1] / 640.0))) // 2
    u, v = int(round(point[0])), int(round(point[1]))
    patch = img[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
    if patch.size == 0:
        return None
    return colour_words.name(patch)


def table_hit_mm(ex, q, u, v):
    """Where the line of sight through a pixel meets the surface the robot stands on, in mm, or None."""
    try:
        p = ex._floor_hit(np.asarray(q, dtype=float), float(u), float(v))
    except Exception:
        return None
    return None if p is None else [round(float(x) * MM, 1) for x in p]


# ---------------------------------------------------------------- the rows of a list
def make_rows(ex, st, rec, img, exclude, words=None, only_words=False, n_max=None):
    """The rows of a look's list (SPEC-R5 4.1) as a record {source, words, items, unseen, notes, threshold, floor,
    per_phrase}, or None for a find (only_words) that the vision model did not answer.
    Level words: one find for EVERY part name of the manual (and the operator's own words of this call, first),
    then the model's regions; level segments: the regions; level absent, or a service that did not answer: colour
    and geometry, with the note. Then the one merge (eyes_parts.merge), and per row its colour, size, where,
    what the manual says about its colour and keep_off. The robot asks for all the parts: it chooses none."""
    parts = st["parts"]
    notes = []
    word = str(words).strip() if isinstance(words, str) and words.strip() else None
    all_names = EP.names(parts)
    exact = word is not None and word.lower() in all_names
    named = sheet_words.label_part(word, [{"name": n} for n in all_names]) if word else None
    # the key a word's rows are kept under: the part's name when the words ARE a part name (or, in a find, name
    # one); else the operator's own words (such a row carries no role)
    word_key = word.lower() if exact else (named["name"] if only_words and named else word)
    per, proposed, info, source, first = {}, None, {}, None, []
    svc = C.service_for(st)
    if svc is not None:
        why = None
        if st["level"] == "words":
            phrases = [word] if only_words else ([word] if word and not exact else []) + all_names
            if phrases:
                raw, info, why, seconds, _ = C.find_each(svc["url"], rec["frame"], phrases,
                                                         max=n_max or EP.BEST_PER_PART, exclude=exclude,
                                                         many=svc["phrases"])
                svc["seconds_last"] = seconds
                if raw is not None:
                    svc["phrases"] = info.get("many")
                    per = {(word_key if word and k == word else k): C.rows(v, "find") for k, v in raw.items()}
                    first = [word_key] if word else []
        if why is None and not only_words:
            answer, why, seconds = C.propose(svc["url"], rec["frame"], max=EP.MAX_ROWS, exclude=exclude)
            if answer is not None:
                svc["seconds_last"] = seconds
                proposed = C.rows(answer["candidates"], "propose")
        if why is None:
            svc["ok"], svc["why"] = True, None
            source = "find" if only_words else ("find+propose" if per else "propose")
        else:
            C.failed(svc, why)
            per, proposed, info, first = {}, None, {}, []
            notes.append(f"the vision service did not answer ({why})")
    if source is None:
        if only_words:
            return None
        proposed, _ = propose(ex, rec, img, exclude, EP.MAX_ROWS)
        source = "local"
        notes.append(NOTE_LOCAL)
        if word:
            notes.append("your words were not used: no vision model that takes words answered")
    rows = EP.merge(per, proposed or [], parts, first=first)
    if n_max:
        rows = rows[:int(n_max)]
    h, w = img.shape[:2]
    lit = rec.get("can_see") == "ok"
    for r in rows:
        r["colour"] = colour_at(img, r["point_px"]) if lit else None
        r["size"] = EP.size_word(r.get("area_px"), w)
        r["where"] = EP.where_word(table_hit_mm(ex, rec["q"], *r["point_px"]))
        r["cut_by_frame"] = EP.cut_by_frame(r.get("box_px"), w, h)
    rows = EP.annotate(rows, parts, colour_words.near)
    hidden = []
    if rows:      # the line of sight to the first row, against an upright box where it meets the table
        _, ray = ex.model.pixel_ray(np.asarray(rec["q"], dtype=float), *rows[0]["point_px"])
        hidden = EP.unseen({"looks_along": [float(x) for x in ray]}, table_hit_mm(ex, rec["q"], *rows[0]["point_px"]))
    threshold, floor = C.limits(st.get("service"), info)
    return {"source": source, "words": word, "items": rows, "unseen": hidden, "notes": notes,
            "threshold": threshold, "floor": floor, "per_phrase": dict(info.get("per_phrase") or {})}


# ---------------------------------------------------------------- a pixel against the kept list and the service
def at(st, key, u, v):
    """The number of the candidate of the look's kept list whose polygon (or box) holds the pixel, else None."""
    for r in (st["lists"].get(key) or {}).get("items") or ():
        if C.inside(r, float(u), float(v)):
            return r["n"]
    return None


def find_at(ex, st, look_id, u, v, part):
    """The sensing job of SPEC-R5 section 3: "in" when the pixel lies in one of the first 8 candidates the vision
    model gives for the part's name on that look's frame file, "out" when it gives candidates and the pixel lies
    in none, "not known" otherwise (no service, a timeout, no candidate, a level that is not words). Safe off the
    session thread: it reads the frame's path, the kept body polygons and the service's address, makes one
    request, and writes nothing (no list, no journal, no service bookkeeping)."""
    svc = st.get("service")
    if st.get("level") != "words" or not isinstance(svc, dict) or svc.get("skip", 0) > 0 or not part:
        return "not known"
    key = str(look_id)
    path = st["frames"].get(key) or ((getattr(ex, "_looks", {}) or {}).get(key) or {}).get("frame")
    if not isinstance(path, str):
        return "not known"
    return C.pixel_in_find(svc["url"], path, part, u, v, (st["excl"].get(key) or ((), None))[0])


__all__ = ["NOTE_LOCAL", "NOTE_NO_SELFSEE", "at", "colour_at", "find_at", "colour_rows", "frame", "geometry_rows",
           "make_rows", "on_own_body", "own_polygons", "propose", "table_hit_mm"]
