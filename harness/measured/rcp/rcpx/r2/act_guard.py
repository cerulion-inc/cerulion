# SPDX-License-Identifier: AGPL-3.0-only
"""What a push or lift may act on (SPEC-R3.md 4.3): the target and what is known of it, the P0 preflight refusals
(nothing moves), and the carry press guard. Nothing here names a robot or a device; nothing reads truth.

    resolve(ex, target)          (entry, None) or (None, refusal unknown_target): an id, else the newest located
                                 entry whose label names the part (sheet_words.label_part) or equals the text
    target_info(ex, e)           P, uncertainty, depth, normal, device centre and top (the entry's own, else the
                                 survey candidate box holding P, ex.state["sight"]["candidates"]), the naming look
                                 (the most level of a triangulated point's marked looks) and ray, the part
    preflight(ex, ...)           (info, u, source, until) or the first refusal of P0 in the spec's order
    forbidden_where(ex, info, seg)   the refusal for a spot or a line that lies where a forbidden part is, or that
                                 cannot be told from one (below); forbidden_at(ex, point) the same for a felt point

Forbidden by WHERE, never by the label alone. A label names what the operator believes; the tool
touches a place. So besides the label check:
- every part the sheet forbids must be PLACED before a push or lift: a fresh located entry whose label names it.
  One that is not placed (never located, or stale: it moved) refuses forbidden_unplaced with the locate call as the
  hint, because the robot cannot tell the spot from a part it may not touch;
- the spot and the whole line the tool will run (from the standoff to the end of the touch) must keep off every
  placed forbidden point by: that point's uncertainty + the target's uncertainty + the radius of the tool body
  at the tip, from the robot's own collision model. Nothing in that sum is a typed limit. Nearer: refused
  forbidden, naming the part, the distance and the sum. (The part's sheet size is NOT added: a sphere of half the
  largest size round one located point of a 200 mm forbidden part swallowed the allowed part beside it.);
- with forbidden parts on the sheet a target whose label names no part of the sheet is refused unknown_part (the
  sheet's rules for it cannot be read);
- the same distance is checked again on the point the touch FELT, before any press (act: stopped forbidden, the
  tool backs off, no push).
What this cannot do: the sheet gives no position and no outline, so a forbidden part's extent beyond its located
points is not known; a spot on the same part further from every located point of it than the sum passes.
    check(ex, kwargs), install(ex)   the carry press guard (below)

The carry press guard (the maintainer's R4 in its stricter form). Under carry, press and touch run only through use;
the pre hook on press refuses, moving nothing:
- cue_near: a fresh located entry whose label names a cue part (sheet_words role cue), or whose part_check names
  one, lies within its point error, sheet radius and tool width of the stated press segment;
- same_spot_rigid: the two most recent presses or pushes whose start tip lay within both point errors and the tool half width of the present
  tip both read rigid (along_push_mm under rigid_mm, gave false) and force_n is higher than the last one's.
Active when ex.state["carry"] exists and ex.state["act"]["busy"] is false (act's own ramp presses are exempt).
The post hook on press records every operator press (start tip, force, along_push_mm, gave, rigid) in
ex.state["act"]["presses"] with or without carry, so the guard reads a sequence that began before carry turned on.
"""

import math

import numpy as np

from rcpx import r3_limits as L
from rcpx import sheet_words
from rcpx.context import refusal
from rcpx.r2 import act_engine as E

MM = 1000.0
_v, _unit = E._v, E._unit


def state(ex):
    """The session act records; noise and hidden level start unknown."""
    return ex.state.setdefault("act", {"acts": [], "busy": False, "force_noise_n": None, "force_hidden_n": None,
                                       "presses": []})


def lift_declared(ex):
    """cfg r3_lift_declared, else ex.context["r3_lift_declared"], else ex.state["carry"]["lift_declared"]."""
    v = (getattr(ex, "cfg", None) or {}).get("r3_lift_declared")
    if v is None:
        v = (getattr(ex, "context", None) or {}).get("r3_lift_declared")
    if v is None:
        v = ((getattr(ex, "state", None) or {}).get("carry") or {}).get("lift_declared")
    return bool(v)


def tools(ex):
    """The loaded primitive names a next or a hint may name (lift only when declared)."""
    names = set(ex.primitive_names())
    if not lift_declared(ex):
        names.discard("lift")
    return names


def devices_doc(ex):
    """The served device sheet document."""
    doc = (getattr(ex, "context", None) or {}).get("devices")
    return doc if isinstance(doc, dict) else {}


def plist(ex):
    """The parts as the served sheet names them."""
    return sheet_words.parts(devices_doc(ex))


def resolve(ex, target):
    """The entry a push target names: a located or contact id; else the newest fresh located entry whose label names that part (sheet_words.label_part) or equals the text. Never coordinates."""
    if not isinstance(target, str) or not target.strip():
        return None, refusal("args", "target must be a located or contact id or a part name, not coordinates",
                             'e.g. push {"target": "located_3"}')
    t = target.strip()
    e = ex.world.get(t)
    if e is not None and e["kind"] in ("located", "contact"):
        return e, None
    parts = plist(ex)
    want = sheet_words.label_part(t, parts)
    best = None
    for e in ex.world.query(kind="located", include_stale=True):
        if e["label"].strip().lower() == t.lower() or (want is not None and
                                                         sheet_words.label_part(e["label"], parts) is want):
            if best is None or not e["stale"] or best["stale"]:
                best = e
    if best is not None:
        return best, None
    for e in ex.world.query(kind="contact", label=t):
        best = e
    if best is not None:
        return best, None
    hint = ('locate {look, px, part} first; robot://world lists the ids' if "locate" in ex.primitive_names() else
            "mark it in two looks and triangulate, or touch it; robot://world lists the ids")
    return None, refusal("unknown_target", f"no located or contact entry or part named {t[:40]}", hint)


def epoch_stale(ex, e):
    """Why e is from an older epoch (the epochs package's stamp, or the entry's own data.epoch), else None."""
    st = (getattr(ex, "state", None) or {}).get("epochs") or {}
    stamp = (st.get("stamps") or {}).get(e["id"]) or {}
    ep = stamp.get("epoch", (e.get("data") or {}).get("epoch"))
    if ep is not None and int(ep) != int(getattr(ex, "epoch", 0)):
        return f"{e['id']} is from epoch {ep}, now {ex.epoch}: the base or the world moved since"
    return None


def _look_q(ex, look_id):
    """The stored camera pose of a look, or None."""
    rec = (getattr(ex, "_looks", None) or {}).get(str(look_id))
    return None if rec is None else np.asarray(rec["q"], dtype=float)


def _tri_uncertainty(ex, e):
    """The depth uncertainty of a triangulated point, read again from its marks (mm), else None."""
    marks = (getattr(ex, "_marks", None) or {}).get(e["label"])
    if not marks or e["point_mm"] is None:
        return None
    try:
        from rcpx import perception
        q = perception.triangulation_quality([(mk["origin"], mk["dir"]) for mk in marks.values()],
                                             _v(e["point_mm"]) / MM, ex.model.f_px, px_err=L.MATCH_ERR_PX,
                                             floor_z_m=ex.cfg.get("floor_z_m"))
        return float(q.get("depth_uncertainty_mm") or 0.0)
    except Exception:
        return None


def target_info(ex, e, B=None):
    """What the stages need of a target entry (SPEC-R3 3.2): P (mm or None), uncertain_mm, depth, normal (unit, the face's outward normal, or None), device centre (x, y) and top (mm or None), the naming look and its q, the naming ray (unit, camera to P), the part (sheet_words parts entry or None) and the label said."""
    d = e.get("data") or {}
    parts = plist(ex)
    part = sheet_words.label_part(e["label"], parts)
    if part is None and isinstance(d.get("part_check"), str):
        part = None
    P = None if e.get("point_mm") is None else _v(e["point_mm"])
    unc = d.get("uncertain_mm", d.get("sigma_p_mm"))
    if unc is None and e["kind"] in ("located", "contact"):
        unc = max(float(d.get("error_mm") or 0.0), _tri_uncertainty(ex, e) or 0.0)
    unc = float(unc or 0.0)
    normal = d.get("normal")
    if normal is None and e["kind"] == "contact":
        normal = d.get("normal_est")
    normal = None if normal is None else _unit(normal)
    looks = [k for k in (d.get("looks") or []) if _look_q(ex, k) is not None]
    naming = d.get("naming_look") or _most_level(ex, looks, P)
    ray, origin = None, None
    if isinstance(d.get("ray"), dict) and d["ray"].get("dir") is not None:
        ray, origin = _unit(d["ray"]["dir"]), _v(d["ray"].get("origin_mm") or [0, 0, 0])
    q_n = None if naming is None else _look_q(ex, naming)
    if q_n is not None and origin is None:
        origin = _v(ex.model.fk(q_n)["camera"][0]) * MM
    if ray is None and origin is not None and P is not None:
        ray = _unit(P - origin)
    if e["kind"] == "contact" and naming is None:
        naming, q_n = newest_seeing_look(ex, P)
    label = part["name"] if part is not None else e["label"]
    centre, top = d.get("device_centre_mm"), d.get("device_top_mm")
    if centre is None and P is not None:
        centre, top = candidate_box(ex, P, unc, B)
    return {"id": e["id"], "kind": e["kind"], "P": P, "uncertain_mm": unc, "depth": d.get("depth") or
            ("measured" if P is not None else "ray_only"), "normal": normal,
            "centre": centre, "top": top, "naming_look": naming,
            "q_naming": q_n, "ray": ray, "origin": origin, "part": part, "label": label,
            "device": part["device"] if part is not None else None, "entry": e}


def _most_level(ex, looks, P):
    """Of a point's marked looks (triangulate's), the one whose ray to P is the most level: the naming ray of P2 rule 4 (every marked ray is free space up to P; a level one meets a face on a side, a steep one the top of what stands over it). The first look when P or the poses are not known."""
    if not looks:
        return None
    if P is None:
        return looks[0]
    best, best_z = looks[0], None
    for k in looks:
        o = _v(ex.model.fk(_look_q(ex, k))["camera"][0]) * MM
        r = _unit(P - o)
        if r is not None and (best_z is None or abs(float(r[2])) < best_z):
            best, best_z = k, abs(float(r[2]))
    return best


def candidate_box(ex, P, sigma_mm=0, B=None):
    """The survey box holding the point projection within its uncertainty band and the tool width, else no centre or top."""
    B = L.bounds_of(ex) if B is None else B
    cands = ((getattr(ex, "state", None) or {}).get("sight") or {}).get("candidates") or []
    best = None
    for c in cands:
        w, sz = c.get("where_mm"), c.get("size_mm")
        if not w or not sz:
            continue
        dx = max(0.0, abs(float(P[0]) - float(w[0])) - L.half(sz[0]))
        dy = max(0.0, abs(float(P[1]) - float(w[1])) - L.half(sz[1]))
        d = math.hypot(dx, dy)
        if d <= L.device_box_near_mm(B, sigma_mm=sigma_mm) and (best is None or d < best[0]):
            best = (d, [float(w[0]), float(w[1])], c.get("top_mm"))
    return (None, None) if best is None else (best[1], best[2])


def newest_seeing_look(ex, P):
    """(look id, q) of the newest look whose camera saw P in frame and outside the hand's own picture, else (None, None)."""
    if P is None:
        return None, None
    for lid in reversed(list((getattr(ex, "_looks", None) or {}).keys())):
        rec = ex._looks[lid]
        if rec.get("can_see") == "dark":
            continue
        q = np.asarray(rec["q"], dtype=float)
        px = ex.model.project(q, P / MM)
        if px is None:
            continue
        u, v = float(px[0]), float(px[1])
        if not (0 <= u < ex.model.cam_w and 0 <= v < ex.model.cam_h):
            continue
        hid = ex.hidden_by_hand(q) if hasattr(ex, "hidden_by_hand") else None
        box = (hid or {}).get("box") if isinstance(hid, dict) else None
        if box and box[0] <= u <= box[2] and box[1] <= v <= box[3]:
            continue
        return lid, q
    return None, None


def tool_radius_mm(ex):
    """The radius of the tool body at the tip in the robot's own collision model (mm): of the tool's capsules, the one whose axis passes nearest the tip. 0 when the model gives none."""
    try:
        s = ex._sense()
        tool = getattr(ex, "_tool_bodies", None) or set()
        tip = _v(s.tip)
        best = None
        for c in ex.model.collision_capsules(np.asarray(s.q, dtype=float)):
            if c[0] in tool:
                d = _seg_dist(tip, _v(c[1]), _v(c[2]))
                if best is None or d < best[0]:
                    best = (d, float(c[3]))
        return 0.0 if best is None else MM * best[1]
    except Exception:
        return 0.0


def forbidden_zones(ex):
    """Per part the sheet forbids: {part, placed: [{id, P, uncertain_mm}], seen_stale}. placed holds the fresh located entries whose label names the part (sheet_words.label_part)."""
    parts = plist(ex)
    out = []
    for p in parts:
        if p["role"] != "forbidden":
            continue
        placed, stale = [], False
        for e in ex.world.query(kind="located", include_stale=True):
            if e.get("point_mm") is None or sheet_words.label_part(e["label"], parts) is not p:
                continue
            if e["stale"] or epoch_stale(ex, e):
                stale = True
                continue
            placed.append({"id": e["id"], "P": _v(e["point_mm"]), "uncertain_mm": target_info(ex, e)["uncertain_mm"]})
        out.append({"part": p, "placed": placed, "seen_stale": stale})
    return out


def _keep_off(zone, spot, own_unc, tool_mm):
    """Keep off by both points' position errors and the tool radius."""
    return float(spot["uncertain_mm"]) + float(own_unc) + float(tool_mm)


def _nearest(zones, a, b, own_unc, tool_mm):
    """The placed forbidden point nearest the segment a to b (a point when b is None) by its margin: (distance mm, keep-off mm, zone, spot) with the smallest distance - keep-off, or None when nothing is placed."""
    best = None
    for z in zones:
        for sp in z["placed"]:
            d = float(np.linalg.norm(sp["P"] - a)) if b is None else _seg_dist(sp["P"], a, b)
            k = _keep_off(z, sp, own_unc, tool_mm)
            if best is None or d - k < best[0] - best[1]:
                best = (d, k, z, sp)
    return best


def forbidden_where(ex, info, seg=None):
    """The refusal of a push or lift by where it will touch, or None. seg: (a, b) in mm, the line the tool runs (None: the target point alone; a ray-only target: its ray from the camera)."""
    zones = forbidden_zones(ex)
    if not zones:
        return None
    lid = "locate" in ex.primitive_names()
    if info["part"] is None:
        names = [p["name"] for p in plist(ex) if p["role"] == "control"]
        return refusal("unknown_part", f'robot://devices forbids parts here and "{str(info["label"])[:30]}" names no '
                       "part of the sheet: the sheet's rules for this spot cannot be read",
                       "name the part as the sheet does" + (f": {', '.join(names[:4])}" if names else ""))
    tool_mm = tool_radius_mm(ex)
    if seg is None and info["P"] is not None:
        a, b = info["P"], None
    elif seg is None and info["ray"] is not None and info["origin"] is not None:
        far = max([float(np.linalg.norm(sp["P"] - info["origin"])) for z in zones for sp in z["placed"]] or [0.0])
        a, b = info["origin"], info["origin"] + info["ray"] * (far + MM)
    elif seg is not None:
        a, b = _v(seg[0]), _v(seg[1])
    else:
        a = b = None
    hit = None if a is None else _nearest(zones, a, b, info["uncertain_mm"], tool_mm)
    if hit is not None and hit[0] <= hit[1]:
        d, k, z, sp = hit
        return refusal("forbidden", f'this spot is {d:.0f} mm from the {z["part"]["name"]} ({sp["id"]}): '
                       f'robot://devices says "{z["part"]["forbidding"]}"',
                       f"choose a spot over {k:.0f} mm from it (both points' uncertainty and the tool's width, "
                       "from the robot's own model)")
    for z in zones:
        if z["placed"]:
            continue
        name = z["part"]["name"]
        why = (f'robot://devices says "{z["part"]["forbidding"]}" and the {name} '
               + ("moved since it was located" if z["seen_stale"] else "is not located")
               + ": the robot cannot tell this spot from it")
        hint = (f'locate {{"look": ..., "px": ..., "part": "{name}"}} first, so the robot can keep off it' if lid
                else f"mark the {name} in two looks and triangulate it first, so the robot can keep off it")
        return refusal("forbidden_unplaced", why, hint)
    return None


def forbidden_at(ex, point_mm, own_unc=0.0):
    """(part name, entry id, distance mm, keep-off mm) when a felt point lies within a placed forbidden part's keep-off, else None."""
    hit = _nearest(forbidden_zones(ex), _v(point_mm), None, own_unc, tool_radius_mm(ex))
    if hit is not None and hit[0] <= hit[1]:
        return hit[2]["part"]["name"], hit[3]["id"], round(hit[0], 0), round(hit[1], 0)
    return None


def nearest_placed(ex, point_mm):
    """(label, id, distance mm) of the fresh located entry nearest a point, else None: what an unplanned contact was near (said of a stop on the way)."""
    best = None
    for e in ex.world.query(kind="located", include_stale=False):
        if e.get("point_mm") is None:
            continue
        d = float(np.linalg.norm(_v(e["point_mm"]) - _v(point_mm)))
        if best is None or d < best[2]:
            best = (e["label"], e["id"], round(d, 0))
    return best


def preflight(ex, target, way, until, firm, max_n, hold_s, detail, verb="push", B=None):
    """P0 in refusal order: choose the line, refuse unknown force with a ready max_n call before any look, approach or motion."""
    B = L.bounds_of(ex) if B is None else B
    dark = ex._dark_refusal_now()
    if dark is not None:
        return dark
    bad = _args(target, way, until, firm, max_n, hold_s, detail)
    if bad is not None:
        return bad
    e, bad = resolve(ex, target)
    if bad is not None:
        return bad
    info = target_info(ex, e, B)
    lid = "locate" in ex.primitive_names()
    if e["stale"] or epoch_stale(ex, e):
        why = epoch_stale(ex, e) or f"{e['id']} is stale: {(e.get('data') or {}).get('stale_why') or 'it moved'}"
        look = newest_seeing_look(ex, info["P"])[0] or info["naming_look"]
        hint = (f'locate {{"look": "{look}", "part": "{info["label"]}"}} with the pixel where it shows now' if lid
                else "mark it again in two looks and triangulate, or touch it")
        return refusal("stale_target", why, hint)
    part = info["part"]
    long_press = (part or {}).get("entry", {}).get("long_press_s")
    hold_max = L.hold_max_s(B, long_press_s=long_press)
    if hold_max is not None and hold_s > hold_max:
        return refusal("args", f"hold_s exceeds the sheet's long press time less detection time ({hold_max:g} s)",
                       f'push {{"target": "{info["id"]}", "hold_s": {max(0, hold_max):g}}}')
    if part is not None and part["role"] == "cue":
        moves = part["moves"] or "only shows the state"
        controls = [p["name"] for p in plist(ex) if p["device"] == part["device"] and p["role"] == "control"]
        return refusal("cue_not_control", f'robot://devices says the {part["name"]} "{moves}"',
                       "push the part that moves" + (f": locate the {controls[0]}" if controls and lid else
                                                     f": the {controls[0]}" if controls else ""))
    if part is not None and part["role"] == "forbidden":
        return refusal("forbidden", f'robot://devices says "{part["forbidding"]}"',
                       "choose another part; robot://devices lists them")
    P = info["P"]
    if verb != "push" or P is None:
        bad = forbidden_where(ex, info)
    else:
        u_w = E.line(way, info["normal"], info["centre"], info["top"], P, info["ray"], B, info["uncertain_mm"])[0]
        unc_w = info["uncertain_mm"]
        bad = forbidden_where(ex, info, None if u_w is None else (
            P - u_w * (L.standoff_mm(B, ex._sense().q, u_w, thing=L.thing_of(B, info["id"], sigma_mm=info["uncertain_mm"]))),
            P + u_w * (L.touch_reach_mm(B, ex._sense().q, u_w, thing=L.thing_of(B, info["id"], sigma_mm=info["uncertain_mm"]), sigma_mm=unc_w))))
    if bad is not None:
        return bad
    if P is not None and firm is not True:
        for a in reversed(state(ex)["acts"]):
            spots = [x for x in ((a.get("contact") or {}).get("point_mm"), a.get("point_mm")) if x is not None]
            if a.get("stopped") == "rigid" and any(float(np.linalg.norm(np.asarray(x, dtype=float) - P))
                                                   <= L.same_spot_mm(B, sigma_a_mm=info["uncertain_mm"], sigma_b_mm=a.get("uncertain_mm") or 0) for x in spots):
                return refusal("same_spot_rigid", f"the {a['tool']} at call {a['seq']} on this spot stopped rigid at "
                               f"{float(a.get('last_n') or 0):.1f} N", "this spot does not move: push another part, view another "
                               "side, or send firm true if you are sure")
    if ((getattr(ex, "state", None) or {}).get("grasp") or {}).get("held"):
        return refusal("held", "the hand holds something", "release it first")
    if until is None:
        until = "stop" if (part or {}).get("sliding") else "give"
    u, src = (np.array([0.0, 0.0, 1.0]), "up") if verb != "push" else E.line(way, info["normal"], info["centre"], info["top"], P, info["ray"], B, info["uncertain_mm"])
    if u is None:
        if "lift" in tools(ex):
            hint = f'lift {{"target": "{e["id"]}"}} pushes up from under an edge'
        else:
            dl = sheet_words.do_line(part, devices_doc(ex), tools(ex) | {"approach", "touch", "press"},
                                     _body(ex)) if part else None
            hint = (dl or {}).get("do") or "lift is not enabled here: an upward nudge runs through approach, " \
                                           "touch and press"
        return refusal("way_up", f"the line into the part points {src} deg up", hint)
    spec = L.force_spec(B, "press", L.thing_of(B, info["id"], sigma_mm=info["uncertain_mm"]),
                        ex._sense().q, u, intent={k: v for k, v in {"max_n": max_n, "firm": firm}.items() if v is not None},
                        sheet_forces=(part or {}).get("forces"), moved_n=moved_of(state(ex), info))
    bad = spec.get("refused")
    if bad:
        hint = (f'{verb} {{"target": "{info["id"]}", "max_n": {bad["offer_n"]:.1f}}}: '
                f"no more than the robot's own hand resting on it; the arm can push up to "
                f"{bad['most_n']:.1f} N along this line")
        return refusal("force_unknown", bad["why"], hint)
    return info, u, src, until


def moved_of(st, info):
    """The largest measured first-motion force on this target or its device."""
    values = [a["f_moved_n"] for a in st["acts"] if a.get("f_moved_n") is not None and
              (a.get("target") == info["id"] or info["device"] is not None and a.get("device") == info["device"])]
    return max(values) if values else None


def _body(ex):
    """The served body document or callable, else the loaded model description."""
    doc = (getattr(ex, "context", None) or {}).get("body")
    if callable(doc):
        try:
            doc = doc()
        except Exception:
            doc = None
    if not isinstance(doc, dict):
        try:
            doc = ex.model.describe()
        except Exception:
            doc = None
    return doc if isinstance(doc, dict) else None


def _args(target, way, until, firm, max_n, hold_s, detail):
    """Reject malformed push arguments before any motion."""
    def no(field, rule, ex):
        """An argument refusal naming its rule and one ready example."""
        return refusal("args", f"{field}: {rule}", ex)
    if not isinstance(target, str) or not target.strip():
        return no("target", "a located or contact id, or a part name (never coordinates)",
                  'push {"target": "located_3"}')
    if way not in ("in", "down"):
        return no("way", "in or down", 'push {"target": "located_3", "way": "down"}')
    if until not in (None, "give", "stop"):
        return no("until", "give or stop", 'push {"target": "located_3", "until": "stop"}')
    if not isinstance(firm, bool):
        return no("firm", "true or false", 'push {"target": "located_3", "firm": true}')
    if max_n is not None and (isinstance(max_n, bool) or not isinstance(max_n, (int, float)) or not math.isfinite(max_n) or not max_n > 0):
        return no("max_n", "a force in N over 0", 'push {"target": "located_3", '
                  '"max_n": 5.0}')
    if isinstance(hold_s, bool) or not isinstance(hold_s, (int, float)) or not math.isfinite(hold_s) or hold_s < 0:
        return no("hold_s", "a number of seconds at least 0", 'push {"target": "located_3", "hold_s": 2}')
    if detail not in ("brief", "full"):
        return no("detail", "brief or full", 'push {"target": "located_3", "detail": "full"}')
    return None

     # the press line checked runs this far ahead of the tip (SPEC-R3 4.3)


def _state(ex):
    """The shared act state."""
    return state(ex)


def _seg_dist(p, a, b):
    """Distance from p to the clamped segment a,b."""
    ab = b - a
    t = 0.0 if float(ab @ ab) == 0 else min(1.0, max(0.0, float((p - a) @ ab) / float(ab @ ab)))
    return float(np.linalg.norm(p - (a + t * ab)))


def _cue_names(plist):
    """The names that the sheet marks as cues."""
    return {p["name"] for p in plist if p["role"] == "cue"}


def _is_cue(e, plist, cues):
    """The cue that this located label or checked part names."""
    p = sheet_words.label_part(e["label"], plist)
    if p is not None and p["role"] == "cue":
        return p["name"]
    chk = str((e.get("data") or {}).get("part_check") or "").lower()
    for name in cues:
        if name and name in chk:
            return name
    return None


def _direction(ex, kwargs, s):
    """Resolve the call's direction in the base frame, or None."""
    try:
        d, bad = ex._unit_in_base(kwargs.get("direction"), kwargs.get("frame", "base"), s)
    except Exception:
        return None
    return None if bad is not None else d


def rigid_now(ex, q, d):
    """The current bounds-derived rigid travel along the line."""
    return L.rigid_mm(L.bounds_of(ex), q, d)


def check(ex, kwargs):
    """The guard's refusal for a press with these kwargs, or None (pure over the executor's present state)."""
    st = _state(ex)
    if "carry" not in (getattr(ex, "state", None) or {}) or st.get("busy"):
        return None
    s = ex._sense()
    d = _direction(ex, kwargs, s)
    if d is None:
        return None
    tip = np.asarray(s.tip, dtype=float) * MM
    doc = (getattr(ex, "context", None) or {}).get("devices")
    plist = sheet_words.parts(doc if isinstance(doc, dict) else {})
    cues = _cue_names(plist)
    B = L.bounds_of(ex)
    end = tip + d * float(kwargs.get("travel_mm") or 0)
    for e in ex.world.query(kind="located", include_stale=False):
        if e.get("point_mm") is None:
            continue
        name = _is_cue(e, plist, cues)
        if name is None:
            continue
        dist = _seg_dist(np.asarray(e["point_mm"], dtype=float), tip, end)
        part = sheet_words.label_part(name, plist)
        data = e.get("data") or {}
        err = data.get("uncertain_mm", data.get("error_mm")) or 0
        if dist <= L.cue_near_mm(B, sigma_mm=err, size_mm=E.largest_size(part)):
            return refusal("cue_near", f"the press would land on the {name} ({e['id']}, {dist:.0f} mm from the "
                           "line): robot://devices says it does not move", "push the part that moves: locate it")
    try:
        force = float(kwargs.get("force_n"))
    except (TypeError, ValueError):
        return None
    here = [p for p in st["presses"] if p.get("start_mm") is not None and
            float(np.linalg.norm(np.asarray(p["start_mm"], dtype=float) - tip)) <= L.same_spot_mm(B, sigma_b_mm=p.get("uncertain_mm") or 0)]
    if len(here) >= 2:
        a, b = here[-2], here[-1]
        if a.get("rigid") and b.get("rigid") and force > float(b.get("force_n") or 0.0):
            return refusal("same_spot_rigid", f"the presses at calls {a['call']} and {b['call']} here did not move it "
                           f"(under {rigid_now(ex, s.q, d):.1f} mm at {a['force_n']:.1f} and {b['force_n']:.1f} N)",
                           "this spot does not move: it is not the moving part; locate another part or view another "
                           "side")
    return None


def install(ex):
    """Install the carry press checks and retain operator press evidence."""
    from rcpx.r2 import act_evidence
    return act_evidence.install_press_guard(ex)


def tool_reach_mm(ex):
    """The tool bodies' furthest model reach from the tip, in mm."""
    from rcpx.r2 import act_evidence
    return act_evidence.tool_reach_mm(ex)


def hand_lands_first(ex, B, P, u, standoff, thing=None):
    """The non-tool body that would precede the finger at the touch target."""
    from rcpx.r2 import act_evidence
    return act_evidence.hand_lands_first(ex, B, P, u, standoff, thing)


def motion_call(ex, B, name, kwargs):
    """Call a core primitive and read its periods without replacing its servo method."""
    from rcpx.r2 import act_evidence
    return act_evidence.motion_call(ex, B, name, kwargs)


__all__ = ["check", "devices_doc", "epoch_stale", "forbidden_at", "forbidden_where",
           "forbidden_zones", "install", "lift_declared", "nearest_placed", "newest_seeing_look", "plist", "preflight",
           "resolve", "rigid_now", "state", "target_info", "tool_radius_mm", "tools"]
