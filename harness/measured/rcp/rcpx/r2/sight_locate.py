# SPDX-License-Identifier: AGPL-3.0-only
"""locate (package sight): point once at a part in a look and the robot measures where it is.

Steps (the spec's numbering): 1 refusals (unknown_look, sheet_unknown, outside_frame, can_see); 2 own body
(ex.state["selfsee"].first_hit at the look's pose; without selfsee a note); 3 on "table" or "contact_N": the ray
meets that plane, nothing moves; 4 the patch (rcpx.r2.sight_match.pick_patch; a plain patch is ray_only); 5 the
bracket along the ray; 6 two more views (rcpx.r2.sight_plan.locate_views; none: ray_only); 7 the runtime's own
guarded moves (ex.gates first), a look at each, the match along the epipolar band, then where the bands cross; a
dark view ends it (ray_only, the camera returns by retreat); 8 three-ray least squares within miss_mm; 9 sanity (own
body, under the table, beyond reach); 10 uncertain_mm; 11 part_check; 12 relations; 13 the survey box holding it; 14
the located entry, then the camera back where it was; 15 the result.

uncertain_mm is the step 10 propagation plus the whole stated camera pose uncertainty at the point's depth (pu_mm +
depth x pu_rad: one fixed camera error moves every ray the same way, which the three-ray miss cannot see) plus the
relief of the matched patch (its half width as the correlation weighs it, at that depth: the views fix the depth of
the patch's edges, on a rounded part its outline, and the surface the pixel shows may stand nearer; measured 9.5 mm
on a lamp dome against a stated 8.0 before this term), and never
less than the largest miss of the three rays, the spread of the two extra views' own depths, or the move of the
point when the next larger patch measures it again (disagreements the geometry alone does not show). The camera goes back to the pose the
arm held when locate was called (for a look the arm still holds, the naming look's own pose).
Step 7's matching rules (a weak or rivalled peak counts only when the way back finds the pointed pixel; a wider
patch round the same pixel when the pointed one repeats along a view's line; one replacement for a view that lost the
part) and step 15's next (no act on a point whose identity is not verified) are in rcpx/r2/sight_find.py. on
"table" is refused when a survey box stands on the line of sight. When the camera did not come back to where it
started (by the measured joints, against r3_limits.view_tol at the arm's own resolution) the result says so.
Nothing here reads simulator ground truth or names a robot or a device.
"""

import math
import inspect

import numpy as np

from rcpx import r3_limits as L
from rcpx.r2 import sight_bounds as SB
from rcpx import side_words
from rcpx.context import refusal
from rcpx.control_base import _mm
from rcpx.r2 import sight_match as M
from rcpx.r2 import sight_parts as SP
from rcpx.r2 import sight_plan as P
from rcpx.r2.sight_find import another as _another, band_px, find as _find, identity, next_call
from rcpx.r2.sight_find import second_scale as _second_scale

MM = 1000.0
PLANE_MM = 1.0               # the uncertainty of a plane locate meets (the floor or a touched face): a contact's
                             # place is measured to about a millimetre
OWN_BODY = ("own_base", "own_arm", "own_hand")
CROP_TILE_PX = 160           # each crop of the composite (three side by side fit 640 px)
SAID_WHY_WORDS = 16          # a reason longer than this loses its bracketed detail in said (the full result keeps it)


def _survey_price(ex):
    """The survey's price in a hint that names it, or ""."""
    from rcpx import price_words
    return price_words.priced(ex, "survey")


def _said_refusal(check, why, hint):
    return {"said": why, **refusal(check, why, hint)}


def _image(ex, rec):
    if rec.get("img") is not None:
        return np.asarray(rec["img"])
    f = rec.get("frame")
    if isinstance(f, str):
        from PIL import Image
        with Image.open(f) as im:
            return np.asarray(im.convert("RGB"))
    if isinstance(f, np.ndarray):
        return f
    return None


def resolve_look(ex, look, px):
    """(look id, record, pixel (u, v), via text) or a refusal: a look id, or a sheet id (survey_N, around_N)."""
    if not (isinstance(px, (list, tuple)) and len(px) == 2 and all(
            isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in px)):
        return _said_refusal("args", "px is [u, v]: two pixel numbers of the picture, from its top-left",
                             'for example locate {"look": "L3", "px": [412, 233], "part": "button"}')
    key = str(look).strip()
    st = ex.state["sight"]
    via = ""
    if key in st["sheets"]:
        lid, uv, why = SP.sheet_pixel(st["sheets"], key, px, ex.model.cam_w, ex.model.cam_h)
        if why == "layout":
            return _said_refusal("sheet_unknown", f"the tile layout of {key} is not known, so its pixel cannot be "
                                 "turned into a look's pixel", _like_hint(ex, st["sheets"][key]))
        if why == "outside":
            return _said_refusal("outside_frame", f"pixel ({px[0]:g}, {px[1]:g}) is outside the tiles of {key}",
                                 "point inside one of its tiles")
        via = f" (tile {lid} of {key})"
        key, px = lid, uv
    elif key.lower().startswith(("survey_", "around_")):
        return _said_refusal("unknown_look", f"no sheet {key!r} in this session",
                             "use a look id or a sheet id a survey or a view around returned")
    key, rec = ex._look_rec(key)
    if rec is None:
        return _said_refusal("unknown_look", f"no look {look!r} in this session", "pass a look id a look returned")
    u, v = float(px[0]), float(px[1])
    if not (0 <= u < ex.model.cam_w and 0 <= v < ex.model.cam_h):
        return _said_refusal("outside_frame", f"pixel ({u:g}, {v:g}) is outside the {ex.model.cam_w} x "
                             f"{ex.model.cam_h} frame of {key}", "u from 0 to width, v from 0 to height, from the "
                             "top-left")
    if rec.get("can_see") == "dark":
        return _said_refusal("can_see", f"look {key} is dark (brightness {rec.get('brightness', 0):.1f} of 255): "
                             "a point in it locates nothing", "look again once the light reads dim or ok")
    return key, rec, (u, v), via


def _like_hint(ex, entry):
    tiles = entry.get("tiles") or []
    lk = tiles[0] if tiles else "L1"
    if "witness" in [m.__name__.rsplit(".", 1)[-1] for m in getattr(ex, "mods", [])]:
        return f"look like={lk} for the tile you mean, then locate in that look"
    return f"locate in the tile's own look (for example {lk}), whose id the sheet's tile shows"


def own_body(ex, key, q, u, v):
    """A refusal when the pixel (or MARK_RING_HITS of the 8 pixels MARK_RING_PX round it) shows the robot itself
    by selfsee's first_hit; None when not; "unchecked" without selfsee."""
    ss = ex.state.get("selfsee")
    fh = ss.get("first_hit") if isinstance(ss, dict) else None
    if not callable(fh):
        return "unchecked"
    try:
        hit = fh(q, u, v)
        r = L.MARK_RING_PX
        ring = [fh(q, u + du, v + dv) for du in (-r, 0, r) for dv in (-r, 0, r) if du or dv]
    except Exception:          # a selfsee that cannot answer here leaves the check undone, and says so
        return "unchecked"
    groups = [h.get("what") for h in [hit] + ring if isinstance(h, dict)]
    bad = groups[0] if groups and groups[0] in OWN_BODY else None
    if bad is None and sum(g in OWN_BODY for g in groups[1:]) >= L.MARK_RING_HITS:
        bad = next(g for g in groups[1:] if g in OWN_BODY)
    if bad is None:
        return None
    return _said_refusal("own_body", f"pixel ({u:.0f}, {v:.0f}) in {key} shows {_body_words(ex, bad)}, not a thing "
                         "in the world", "point at a thing that stands apart from the robot; survey lists the "
                         "things on the table: survey {}" + _survey_price(ex))


def _body_words(ex, group):
    if group == "own_base":
        h = ex.cfg.get("mount_half_m")
        side = f", {2.0 * float(h) * MM:.0f} mm square" if h else ""
        return f"the robot's own base (its mount plate{side} at its foot)"
    return "the robot's own arm" if group == "own_arm" else "the robot's own hand"


def _t_plane(o, d, z):
    if d[2] > -1e-9:
        return None
    t = (z - float(o[2])) / float(d[2])
    return t if t > 0 else None


def _t_sphere(o, d, radius):
    b = float(o @ d)
    c = float(o @ o) - radius * radius
    disc = b * b - c
    if disc < 0:
        return None
    t = -b + math.sqrt(disc)
    return t if t > 0 else None


def _plane_uncertainty(depth_m, f_px, cos_inc, facts):
    flat = L.ON_TABLE_PX * depth_m / f_px / max(abs(cos_inc), 0.2) * MM
    pose = L.MISS_PU_FRAC * (facts["pu_mm"] + depth_m * MM * math.radians(facts["pu_deg"]))
    return flat + PLANE_MM + pose


def _box_holding(ex, p_m=None, seg=None):
    """(candidate entry, its centre mm, its top mm) of the survey box whose footprint grown DEVICE_BOX_NEAR_MM holds
    the point's floor projection (or, for a ray, the first box a bracket point falls in), else (None, None, None)."""
    for c in (ex.state["sight"].get("candidates") or []):
        b = P.box_of(c)
        if b is None:
            continue
        lo, hi, _ = b
        g = SB.limit(ex, "DEVICE_BOX_NEAR_MM", (max(c["err_mm"]) if c.get("err_mm") else None)) / MM
        pts = [p_m] if p_m is not None else list(seg or [])
        for p in pts:
            inside = lo[0] - g <= p[0] <= hi[0] + g and lo[1] - g <= p[1] <= hi[1] + g
            if p_m is None:
                inside = inside and lo[2] - g <= p[2] <= hi[2] + g
            if inside:
                return c, [round(float(c["where_mm"][0])), round(float(c["where_mm"][1]))], round(float(c["top_mm"]))
    return None, None, None


def _box_on_line(ex, o, d, t_end, n=64):
    """The first survey candidate whose box (as surveyed, not grown) holds a point of the line of sight between the
    camera and depth t_end (its end included: a table point under a raised thing), else None."""
    for c in (ex.state["sight"].get("candidates") or []):
        b = P.box_of(c)
        if b is None:
            continue
        lo, hi, _ = b
        for x in np.linspace(0.0, t_end, n):
            p = o + d * x
            if bool(np.all(p >= lo - 1e-9)) and bool(np.all(p <= hi + 1e-9)):
                return c
    return None


def move(ex, q, label, intent=None, moves=None):
    """A joint move the runtime makes by itself (kind look). The path check solves the share of full
    speed at which every body keeps its clearance from what the world holds (rcpx/r2/sight_path.py); the gates are
    asked for the move at that speed (a path gate that judges a speed is asked again at half of it);
    then the move runs at the share that passed, with the call's intent (the operator's word). Nothing here passes
    a guard, a reading count or a speed of its own.
    (ok, what, why, ran): what is "goal", "dark", a refusal's check or the stop; ran tells whether the arm moved."""
    q = np.asarray(q, dtype=float)
    pts, boxes = P.known(ex)
    info = {}
    bad = P.path_check(ex, np.asarray(ex._sense().q, dtype=float), q, pts, boxes, None, info, P.known_refs(ex))
    gate, share = SB.gate(ex, q, None if bad is not None else info.get("share"))
    if gate is not None and gate["refused"].get("check") == "dark":
        return False, "dark", gate["refused"].get("why", ""), False
    if bad is not None:
        return False, bad.get("check", "clearance"), bad.get("why", ""), False
    if gate is not None:
        return False, gate["refused"].get("check", "refused"), gate["refused"].get("why", ""), False
    r = SB.look_move(ex, q, intent, share=share)
    if moves is not None:
        moves.append(r)
    if "refused" in r:
        chk = r["refused"].get("check", "refused")
        return False, "dark" if chk == "dark" else chk, r["refused"].get("why", ""), False
    if r["stop"] != "goal":
        return False, r["stop"], f"the move to {label} stopped ({r['stop']})", True
    ex._ease_impl()
    return True, "goal", "", True


def _retreat(ex, steps, intent=None, moves=None):
    """The camera back along the arm's own history (the way it came), with the call's intent where the core takes
    one: "by retreat" when it ended at its goal, else "partly"."""
    retreat = ex._retreat_impl
    kw = {"intent": intent} if "intent" in inspect.signature(retreat).parameters else {}
    r = retreat(steps=steps, **kw)
    if moves is not None:
        moves.append(r)
    return "by retreat" if "refused" not in r and r.get("stopped") == "goal" else "partly"


def _view_look(ex, k):
    lk = ex._look_impl(f"locate view {k}", keep_image=True)
    rec = ex._looks[lk["look_id"]]
    img = rec.get("img")
    rec["img"] = None            # the frame file stays; locate keeps its own copy while it matches
    return lk, np.asarray(rec["q"], dtype=float), img


def _cam(ex, q):
    pc, Rc = P.cam_pose(ex.model, q)
    return M.Cam(pc, Rc, ex.model.f_px, ex.model.cam_w, ex.model.cam_h)


def measure(ex, key, rec, img, u, v, o, d, q_now, notes, intent=None, moves=None):
    """Steps 4 to 8 and the return of step 14: {depth, point_m or None, why, looks, views, span, misses, cams, pxs,
    composite tiles, patch}. Moves the arm (and back) only when two views are planned."""
    out = {"depth": "ray_only", "point_m": None, "why": "", "looks": [key], "views": [], "tiles": [], "patch": None}
    size, rms = M.pick_patch(img, (u, v))
    out["patch"], out["grad_rms"] = size, rms
    out["tiles"].append(M.crop(img, (u, v), size or L.PATCH_SIZES_PX[-1], CROP_TILE_PX, f"{key} (you)"))
    if size is None:
        out["why"] = "a plain patch: no depth from pictures"
        return out
    gdir, coh = M.dominant_dir(img, (u, v), size)
    z = float(ex.cfg.get("floor_z_m") or 0.0)
    t_table = _t_plane(o, d, z)
    bracket = SB.limit(ex, "BRACKET_REACH_EXTRA_MM", ex.model) / MM
    # With no model reach, use the naming camera's distance from the base plus the derived bracket.
    reach = float((ex.model.reach() or {}).get("max_reach_mm") or np.linalg.norm(o) * MM) / MM + bracket
    t_reach = _t_sphere(o, d, reach)
    t_near = SB.limit(ex, "BRACKET_NEAR_MM", ex.model) / MM
    t_far = min(x for x in (t_table, t_reach, reach + reach) if x is not None)      # no further than the sphere is wide
    out["bracket_mm"] = [round(t_near * MM), round(t_far * MM)]
    if t_far <= t_near:
        out["why"] = "the line of sight meets the table at once"
        return out
    q_look = np.asarray(rec["q"], dtype=float)
    plan = P.locate_views(ex, q_look, q_now, o, d, t_near, t_far, gdir, (u, v), patch_px=size)
    plan["ctx"]["coherence"] = coh
    out["plan_notes"], out["coherence"] = plan["notes"], coh
    if plan["pair"] is None:
        out["why"] = plan["why"]
        return out
    cam0, F0 = _cam(ex, q_look), M.features(img)
    radial = side_words.radial(_mm(o + d * 0.5 * (t_near + t_far)))
    normals = M.surface_normals(cam0, d, radial)
    band = band_px(ex, 0.5 * (t_near + t_far))
    out["band_px"] = round(band, 1)
    moved, matches, dark = 0, [], False
    pair = list(plan["pair"])
    most = 2 * len(pair)         # each planned view that does not find the part may be replaced once
    geo = (F0, cam0, (u, v), size, o, d, t_near, t_far, band, normals, radial)
    k = -1
    while k + 1 < len(pair):
        k += 1
        view = pair[k]
        ok, stop, why, ran = move(ex, view["q"], f"view {k + 1}", intent, moves)
        moved += int(ran)            # a move that ran, to its goal or stopped on the way: the history holds it
        if not ok:
            if stop == "dark":
                dark = True
            else:
                out["why"] = f"the runtime's move to view {k + 1} did not end at its goal ({stop}: {why[:80]})"
            break
        lk, qk, imgk = _view_look(ex, k + 1)
        out["looks"].append(lk["look_id"])
        if lk["can_see"] == "dark" or imgk is None:
            dark = True
            break
        camk, Fk = _cam(ex, qk), M.features(imgk)
        m = _find(geo, Fk, camk, matches)
        row = {"look_id": lk["look_id"], "ncc": m.get("ncc"), "runner": m.get("runner"),
               "az_deg": view["az_deg"], "el_deg": view["el_deg"], "ok": bool(m["ok"])}
        if m.get("weak_ok"):
            row["weak"] = True       # not a clear peak by itself: the way back found the pointed pixel
        if m.get("wider"):
            row["patch_px"] = m["size"]      # the pointed patch repeated along this line: a wider one matched
        if m.get("px") is not None:
            row["px"] = [round(float(m["px"][0]), 1), round(float(m["px"][1]), 1)]
        out["views"].append(row)
        if m["ok"]:
            ray_k = camk.ray(*m["px"])
            Pk, _, _ = M.triangulate([(o, d)] + [(mm_["cam"].ray(*mm_["px"])) for mm_ in matches] + [ray_k])
            matches.append({"cam": camk, "px": np.asarray(m["px"], dtype=float), "F": Fk, "P": Pk, "view": view,
                            "weak": bool(m.get("weak_ok")), "size": int(m.get("size") or size)})
            out["tiles"].append(M.crop(imgk, m["px"], matches[-1]["size"], CROP_TILE_PX,
                                       f"{lk['look_id']} ncc {m['ncc']:.2f}"))
            if len(matches) == 1:
                Pk = M.on_ray(o, d, [ray_k])[0]
                matches[-1]["P"] = Pk
                if Pk is not None and float((Pk - o) @ d) > t_near and k + 1 < len(pair):
                    # the next view re-centred on the point this one found, so its angle is an angle at the point
                    b = P.b_about(ex, view, Pk, plan["ctx"], exclude=pair[:k + 1]) or \
                        P.pick_b(ex, plan["ranked"], view, Pk, plan["ctx"], exclude=pair[:k + 1])
                    if b is not None:
                        pair[k + 1] = b
        else:
            notes.append(f"{lk['look_id']}: {m.get('why', 'no match')}")
        if k == len(pair) - 1 and len(matches) < 2 and len(pair) < most:
            # a view lost the part (it turned away, the hand stood in front, or the patch repeats along that view's
            # line): one more view in its place (the docstring says why)
            alt = _another(ex, plan, pair, matches)
            if alt is not None:
                pair.append(alt)
    _return(ex, q_now, moved, dark, out, intent, moves)
    if dark:
        out["why"] = "the light went dark between views: depth unknown"
        return out
    if out["why"]:
        return out
    if len(matches) < 2:
        out["why"] = "the extra views did not find the patch again: depth unknown"
        return out
    cams = [cam0] + [m_["cam"] for m_ in matches]
    pxs = [np.array([u, v])] + [m_["px"] for m_ in matches]
    rays = [c.ray(*p) for c, p in zip(cams, pxs)]
    p3, depth_m, t_k, miss, span = M.on_ray(o, d, rays[1:])
    out.update(cams=cams, pxs=pxs, span=span)
    if p3 is None:
        out["why"] = "the extra views did not agree: depth unknown"
        return out
    out["spread_mm"] = round((max(t_k) - min(t_k)) * MM, 2) if len(t_k) > 1 else 0.0
    facts = SB.inputs(ex)
    bound = SB.miss_mm(ex, depth_m * MM, facts)
    out.update(misses_mm=[round(float(x) * MM, 2) for x in miss], miss_bound_mm=round(bound, 2),
               span_deg=round(span, 1), depth_mm=round(depth_m * MM, 1))
    if float(np.max(miss)) * MM > bound or span < P.span_min_deg(ex, size, depth_m) - 1e-6:
        out["why"] = (f"the extra views did not agree (miss {float(np.max(miss)) * MM:.0f} mm): depth unknown"
                      if float(np.max(miss)) * MM > bound else
                      f"the views span only {span:.1f} deg: depth unknown")
        return out
    out["patch_used"] = max(m_["size"] for m_ in matches)
    alt = _second_scale(F0, cam0, matches, out["patch_used"], o, d, t_near, t_far, normals, u, v)
    if alt is not None:
        out["scale_check_mm"] = round(float(np.linalg.norm(alt - p3)) * MM, 2)
    out["depth"], out["point_m"] = "measured", p3
    return out


def _return(ex, q_now, moved, dark, out, intent=None, moves=None):
    """Step 14's return: back to the pose the arm held when locate was called (by retreat in the dark)."""
    if moved == 0:
        return
    if dark:
        out["returned"] = _retreat(ex, moved, intent, moves)
        return
    ok, stop, why, _ = move(ex, q_now, "the starting pose", intent, moves)
    if ok:
        # the move ended at its goal by the servo's own reading: is the camera where it started, by the joints
        # as now measured, to within what the arm's own resolution calls the same view (r3_limits.view_tol)?
        tol = SB.view_tol(ex, q_now)
        (p0, R0), (p1, R1) = P.cam_pose(ex.model, q_now), P.cam_pose(ex.model, np.asarray(ex._sense().q, dtype=float))
        off_mm = float(np.linalg.norm(p1 - p0)) * MM
        off_deg = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(R1 @ R0.T)) - 1.0) / 2.0))))
        out["returned"] = "yes" if off_mm <= tol["mm"] and off_deg <= tol["deg"] else \
            f"near: the camera is {off_mm:.0f} mm and {off_deg:.1f} deg from where it started"
        return
    if stop == "dark":           # the light went while the camera was out: no move is made blind, the way back is
        out["returned"] = _retreat(ex, moved, intent, moves)     # the way it came
        return
    out["returned"] = f"no: {stop}"
    out["return_why"] = why


def _run(ex, look, px, part, note="", on="space", detail="brief", intent=None, moves=None):
    """The locate primitive's body (rcpx/r2/sight.py wraps it)."""
    if detail not in ("brief", "full"):
        return _said_refusal("detail", f"detail is brief or full, not {str(detail)[:20]!r}", "leave it out")
    if not isinstance(part, str) or not part.strip() or len(part) > 80:
        return _said_refusal("args", "part is the part's name from robot://devices (or \"other\" with a note)",
                             'for example locate {"look": "L3", "px": [412, 233], "part": "button"}')
    on = "space" if on is None else str(on).strip()
    if on not in ("space", "table") and not on.startswith("contact_"):
        return _said_refusal("args", "on is space (default), table or a contact id such as contact_3",
                             "leave it out to measure the point in space")
    got = resolve_look(ex, look, px)
    if isinstance(got, dict):
        return got
    key, rec, (u, v), via = got
    if on == "space":
        dark = ex._dark_refusal_now()
        if dark is not None:
            return {"said": dark["refused"]["why"], **dark}
    q_look = np.asarray(rec["q"], dtype=float)
    notes = []
    ob = own_body(ex, key, q_look, u, v)
    if isinstance(ob, dict):
        return ob
    if ob == "unchecked":
        notes.append("own body not checked")
    img = _image(ex, rec)
    if img is None:
        return _said_refusal("unknown_look", f"the frame of {key} is not kept", "take a new look and point in it")
    label = part.strip().lower()
    if label == "other" or label.startswith("other:"):
        label = "other: " + (str(note).strip()[:60] or label.split(":", 1)[-1].strip() or "unnamed")
    parts = SP.plist(ex)
    d_doc = SP.doc(ex)
    hid = ex.hidden_by_hand(q_look)
    r_out = SP.SPOT_KEEP_OUT_PX
    gains = SP.floor_gains(img, [hid["box"] if hid else None, [u - r_out, v - r_out, u + r_out, v + r_out]])
    colour = SP.spot_colour(img, (u, v), gains)
    pc_text = SP.part_check(colour, label, parts)
    o, dvec = ex.model.pixel_ray(q_look, u, v)
    o, dvec = np.asarray(o, dtype=float), np.asarray(dvec, dtype=float)
    facts = SB.inputs(ex)
    q_now = np.asarray(ex._sense().q, dtype=float)
    meas = {"looks": [key], "views": [], "tiles": [], "why": ""}
    if on == "table" or on.startswith("contact_"):
        if on == "table":
            p0 = ex._floor_hit(q_look, u, v)
            if p0 is None:
                return _said_refusal("no_plane", f"pixel ({u:.0f}, {v:.0f}) in {key} looks above the table",
                                     "point at the table, or leave on out")
            z_plane = float(p0[2])
        else:
            c = ex.world.get(on)
            nrm = ((c or {}).get("data") or {}).get("normal_est")
            if c is None or c.get("kind") != "contact" or c.get("point_mm") is None:
                return _said_refusal("unknown_target", f"no contact entry {on}", "robot://world lists the ids")
            if nrm is None or side_words.angle_deg(nrm, [0, 0, 1]) > L.CONTACT_UP_DEG:
                return _said_refusal("no_plane", f"{on} was felt on a face that is not level (more than "
                                     f"{L.CONTACT_UP_DEG:g} deg from up)", "leave on out to measure in space")
            z_plane = float(c["point_mm"][2]) / MM
        t = _t_plane(o, dvec, z_plane)
        if t is None:
            return _said_refusal("no_plane", f"pixel ({u:.0f}, {v:.0f}) in {key} does not look down at that plane",
                                 "leave on out to measure in space")
        p3 = o + t * dvec
        if on == "table":
            # a raised thing on the line: the table point behind or under it is not what the pixel shows
            cand = _box_on_line(ex, o, dvec, t)
            if cand is not None:
                return _said_refusal("no_plane", f"the line of sight from pixel ({u:.0f}, {v:.0f}) in {key} meets "
                                     f"the raised thing {cand.get('id')} before the table: the point is on it, not "
                                     "on the table", "leave on out to measure the point in space")
        depth, unc = "flat", _plane_uncertainty(t, ex.model.f_px, float(dvec[2]), facts)
        meas["tiles"].append(M.crop(img, (u, v), 31, CROP_TILE_PX, f"{key} (you)"))
    else:
        meas = measure(ex, key, rec, img, u, v, o, dvec, q_now, notes, intent, moves)
        p3, depth = meas["point_m"], meas["depth"]
        unc = None
        if p3 is not None:
            if ex.state.get("selfsee") and callable(ex.state["selfsee"].get("on_body")):
                qs = [ex._looks[k]["q"] for k in meas["looks"] if k in ex._looks]
                grp = ex.state["selfsee"]["on_body"](_mm(p3), qs)
                if grp:
                    return _said_refusal("own_body", f"the point ({', '.join(f'{x:.0f}' for x in _mm(p3))}) lies on "
                                         f"{_body_words(ex, grp)}: the views show the robot, not a device",
                                         "point at a thing that stands apart from the robot; survey lists the "
                                         "things on the table: survey {}" + _survey_price(ex))
            if float(p3[2]) < ex._floor_at(p3) - SB.limit(ex, "UNDER_TABLE_MM") / MM:
                p3, depth = None, "ray_only"
                meas["why"] = "the views put the point under the table: depth unknown"
        if p3 is not None:
            dm = float((p3 - o) @ dvec)
            geo, axis = M.uncertainty(meas["cams"], meas["pxs"], [L.POINT_ERR_PX] + [L.MATCH_ERR_PX] *
                                      (len(meas["cams"]) - 1))
            pose = facts["pu_mm"] + dm * MM * math.radians(facts["pu_deg"])   # the stated bound, whole
            # the views fix the depth of what gave the patch its edges, which on a rounded part is its outline:
            # the surface the pixel itself shows may stand nearer by about the half width of the patch as the
            # correlation weighs it (NCC_SIGMA_FRAC of its half side), taken at that depth
            relief = M.NCC_SIGMA_FRAC * 0.5 * float(meas.get("patch_used") or meas.get("patch") or 0.0) * dm * MM \
                / float(ex.model.f_px)
            unc = max(geo + pose + relief, max(meas.get("misses_mm") or [0.0]), float(meas.get("spread_mm") or 0.0),
                      float(meas.get("scale_check_mm") or 0.0))
            meas["relief_mm"] = round(relief, 1)
            meas["unc_axis"] = axis
            reach = float((ex.model.reach() or {}).get("max_reach_mm") or 0.0)
            if reach and float(np.linalg.norm(p3)) * MM > reach + SB.limit(ex, "BRACKET_REACH_EXTRA_MM", ex.model):
                notes.append("beyond the arm's reach")
    return _result(ex, key, via, label, part, note, o, dvec, p3, depth, unc, meas, pc_text, parts, d_doc, notes,
                   detail, rec, colour)


def _result(ex, key, via, label, part_arg, note, o, dvec, p3, depth, unc, meas, pc_text, parts, d_doc, notes,
            detail, rec, colour=None):
    near = SB.limit(ex, "BRACKET_NEAR_MM", ex.model) / MM
    far = float((ex.model.reach() or {}).get("max_reach_mm") or np.linalg.norm(o) * MM) / MM \
        + SB.limit(ex, "BRACKET_REACH_EXTRA_MM", ex.model) / MM
    seg = [o + dvec * t for t in np.linspace(near, far, 16)]
    cand, centre, top = _box_holding(ex, p_m=p3) if p3 is not None else _box_holding(ex, seg=seg)
    template = None
    if meas.get("tiles"):
        template = ex._save(meas["tiles"][0], f"locate_{ex.state['sight']['locates'] + 1}_patch")
    pic = M.composite(meas.get("tiles") or [])
    ex.state["sight"]["locates"] += 1
    pic_path = None if pic is None else ex._save(pic, f"locate_{ex.state['sight']['locates']}_match")
    data = {"uncertain_mm": None if unc is None else round(float(unc), 1), "depth": depth,
            "ray": {"origin_mm": _mm(o), "dir": [round(float(x), 4) for x in dvec]}, "naming_look": key,
            "looks": list(meas.get("looks") or [key]), "device_centre_mm": centre, "device_top_mm": top,
            "candidate": None if cand is None else cand.get("id"), "part_check": pc_text, "normal": None,
            "template": template if isinstance(template, str) else None, "epoch": int(getattr(ex, "epoch", 0)),
            "note": str(note or "")[:120]}
    part = SP.part_of(label, parts)
    ident = identity(part, parts, colour, pc_text)
    data["identity"] = ident         # act reads it: an entry whose identity is not verified is the operator's word
    if depth == "ray_only":
        t_mid = float(np.mean(meas.get("bracket_mm"))) / MM if meas.get("bracket_mm") else float(np.mean([near, far]))
        data["bracket_mm"] = meas.get("bracket_mm")
        data["bracket_mid_mm"] = _mm(o + dvec * t_mid)
    e = ex.world.add("located", label, point_mm=None if p3 is None else _mm(p3), source="locate", data=data)
    eid = e.get("id")
    nxt, no_act = next_call(ex, part, eid, parts, d_doc, ident)
    others = [x for x in ex.world.query(kind="located", include_stale=False) if x["id"] != eid]
    rel = SP.relations(_mm(p3), others, data["candidate"]) if p3 is not None else []
    name = label if part is None else part["name"]
    if p3 is not None:
        pm = _mm(p3)
        seen = "seen from above" if dvec[2] < -L.VERTICAL_Z else \
            f"on a face {side_words.words(-dvec, pm).replace('away from', 'facing away from')}"
        views = len(meas.get("cams") or [key])       # the naming view and the views that found the part
        said = (f"the {name} is at x {pm[0]:.0f}, y {pm[1]:.0f}, z {pm[2]:.0f} mm, to about {unc:.0f} mm, {seen}"
                + (f", from {views} views {meas['span_deg']:.0f} deg apart" if depth == "measured" else
                   ", where the line of sight meets that plane") + (f"; {rel[0]}" if rel else "") + via)
        if str(meas.get("returned") or "").startswith("near"):
            said += "; " + meas["returned"].split(": ", 1)[1]
    else:
        why = meas.get("why") or "no depth"
        if len(why.split()) > SAID_WHY_WORDS:          # said stays one sentence of at most 45 words
            why = why.split(" (")[0]
        said = (f"stored the line of sight to the {name} as {eid}, depth unknown: {why}"
                + ("; push will feel along your line" if nxt and nxt["tool"] == "push" else "") + via)
    out = {"said": said, "id": eid, "part": name, "point_mm": None if p3 is None else _mm(p3),
           "uncertain_mm": data["uncertain_mm"], "depth": depth, "part_check": pc_text, "relations": rel}
    if meas.get("returned") and meas["returned"] != "yes":
        out["returned"] = meas["returned"]
        if meas.get("return_why"):
            out["return_why"] = meas["return_why"][:120]
    if notes:
        out["notes"] = notes[:3]
    if detail == "full":
        out.update(views=meas.get("views"), looks=meas.get("looks"), bracket_mm=meas.get("bracket_mm"),
                   patch_px=meas.get("patch"), grad_rms=meas.get("grad_rms"), misses_mm=meas.get("misses_mm"),
                   miss_bound_mm=meas.get("miss_bound_mm"), span_deg=meas.get("span_deg"),
                   spread_mm=meas.get("spread_mm"), scale_check_mm=meas.get("scale_check_mm"),
                   relief_mm=meas.get("relief_mm"),
                   plan_notes=meas.get("plan_notes"), ray=data["ray"], candidate=data["candidate"],
                   device_centre_mm=centre, device_top_mm=top, why=meas.get("why"))
    out["pictures"] = [pic_path] if isinstance(pic_path, str) else []
    if nxt is None and part is not None:
        out["no_act"] = no_act       # why no act is offered on this point (the sheet, or the spot's identity)
    out["next"] = nxt
    return out


__all__ = ["measure", "move", "next_call", "own_body", "resolve_look", "run"]


def run(ex, look, px, part, note="", on="space", detail="brief", clear=None):
    """Locate with the call's clear-space word carried through its camera moves; report periods and bounds."""
    intent, why = SB.clear_word(clear)
    if why:
        return _said_refusal("args", why, "leave clear out, or say true after a look")
    t0, moves = ex.sim_time(), []
    out = _run(ex, look, px, part, note, on, detail, intent, moves)
    if ex.sim_time() > t0:
        out = SB.report(ex, out, moves, t0, detail)
        if "next" in out:
            out["next"] = out.pop("next")
    return out
