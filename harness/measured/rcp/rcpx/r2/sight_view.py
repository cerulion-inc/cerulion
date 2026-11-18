# SPDX-License-Identifier: AGPL-3.0-only
"""view (package sight, I37; SPEC-R3 4.5): put the camera on a named side of a thing, or around it.

target: a located or contact id, a part label (the newest fresh located entry whose label names that part) or a
survey candidate id (c1, c2, ...). side: robot_side, left, right, far_side (robot-relative, rcpx.side_words), above,
front (the side a located part faces: from its device centre to it) or around (up to AROUND_MAX of robot_side, left,
right and far_side, whichever reach, as one 640 x 480 sheet of 320 x 240 tiles, id around_N, its tile map in
ex.state["sight"]["sheets"]). The pose search is rcpx.r2.sight_plan.side_pose (dry); a side that does not reach
gives way to the nearest one that does (tried says so). Every move is the runtime's own: ex.gates first, then a
guarded, governed joint move. Nothing here reads simulator ground truth or names a robot or a device.
"""

import math

import cv2
import numpy as np

from rcpx import r3_limits as L
from rcpx import side_words
from rcpx import bounds_ratios as R
from rcpx.r2 import sight_bounds as SB
from rcpx.r2 import sight_locate as SL
from rcpx.r2 import sight_parts as SP
from rcpx.r2 import sight_plan as P

MM = 1000.0
SIDES = ("robot_side", "left", "right", "far_side", "above", "front", "around")
SIDE_WORDS = {"robot_side": "the robot's side", "left": "the robot's left", "right": "the robot's right",
              "far_side": "the far side", "above": "above", "front": "its front"}
AROUND_SIDES = ("robot_side", "left", "far_side", "right")   # round the thing in one turn
UPRIGHT_SAID_DEG = 10.0      # a roll under this reads "image up is world up"
ARC_STEP_DEG = 15.0          # view around moves round the thing in steps of this turn


# ---------------------------------------------------------------- view's pose search
SIDE_ORDER = {"robot_side": ("left", "right", "above", "far_side"), "left": ("robot_side", "far_side", "above",
              "right"), "right": ("robot_side", "far_side", "above", "left"),
              "far_side": ("left", "right", "above", "robot_side"),
              "above": ("robot_side", "left", "right", "far_side"), "front": ("robot_side", "left", "right", "above")}


def side_dir(side, target_m, front=None):
    """The horizontal unit vector of a side at the target (front: the device-relative one given), or None."""
    if side == "front":
        return None if front is None else P._unit(np.array([front[0], front[1], 0.0]))
    if side == "above":
        return side_words.side_vector("robot_side", np.asarray(target_m) * MM)
    return side_words.side_vector(side, np.asarray(target_m) * MM)


def side_pose(ex, q_now, target_m, side, size_mm=None, distance_mm=None, elevation_deg=None, front=None,
              path=True, sigma_mm=None, extent_mm=None):
    """view's pose search for one side (SPEC-R3 4.5 view): {q, cam_m, d_mm, el_deg, az_deg, roll_deg, how} or
    {"why"}. The camera stands on the side at the distances of view_distances (from the nearest at which the whole
    thing is in the picture, out to the arm's reach); the elevation the given one (VIEW_EL_DEG, or
    VIEW_EL_ABOVE_DEG above), then +/-10 and 20; the azimuth the side's, then +/-VIEW_AZ_STEP_DEG; the axis through
    the target. path False skips the joint path check (a pose for later, from elsewhere)."""
    m = ex.model
    T = np.asarray(target_m, dtype=float)
    h = side_dir(side, T, front)
    if h is None:
        return {"why": "no direction for that side"}
    dists, wide = view_distances(ex, size_mm, distance_mm, sigma_mm, q_now, extent_mm)
    base_el = float(elevation_deg) if elevation_deg is not None else (
        L.VIEW_EL_ABOVE_DEG if side == "above" else L.VIEW_EL_DEG)
    els = [base_el]
    for s in L.VIEW_EL_STEPS_DEG:
        els += [base_el + s, base_el - s]
    azs = [0.0, L.VIEW_AZ_STEP_DEG, -L.VIEW_AZ_STEP_DEG]
    az0 = math.atan2(float(h[1]), float(h[0]))
    pts, boxes = P.known(ex)
    refs = P.known_refs(ex)
    floor_min = float(ex.cfg.get("floor_z_m") or 0.0) + P.floor_clear_m(ex, q_now)
    seeds = [np.asarray(q_now, dtype=float), m.q_home.copy()]
    first_why = ""
    for dist in dists:
        for el in els:
            for az in azs:
                w = P._sph(az0 + math.radians(az), math.radians(el))
                cam = T + (dist / MM) * w
                if cam[2] < floor_min:
                    continue
                sol = P.solve_view(m, seeds, cam, -w, T)
                if sol is None:
                    first_why = first_why or f"no joint pose puts the camera {dist:.0f} mm out at {el:g} deg"
                    continue
                q = sol["q"]
                held = P.cam_pose(m, q)[0]           # the camera position the pose really holds
                if held[2] < floor_min:
                    first_why = first_why or "the camera's held pose is too near the support"
                    continue
                if not P.in_frame(m, q, [T]):
                    first_why = first_why or "the target leaves the frame"
                    continue
                if P.hand_hides(ex, q, T):
                    first_why = first_why or "the hand would hide the target"
                    continue
                blk = P.line_blocked(ex, q, T)
                if blk is not None:
                    first_why = first_why or f"the {blk} would stand between the camera and the target"
                    continue
                if path:
                    info = {}
                    bad = P.path_check(ex, q_now, q, pts, boxes, None, info, refs)
                    if bad is None:     # the arm stands at q_now: the gates are asked of this move before it is chosen
                        gate = SB.gate(ex, q, info.get("share"))[0]
                        bad = None if gate is None else gate["refused"]
                    if bad is not None:
                        first_why = first_why or f"{bad.get('check')}: {bad.get('why', '')}"
                        continue
                return {"q": q, "cam_m": held, "d_mm": round(float(np.linalg.norm(held - T)) * MM),
                        "el_deg": round(el, 1), "az_deg": az,
                        "roll_deg": sol["roll_deg"], "how": sol["how"], "wide": wide and distance_mm is None}
    return {"why": first_why or "no pose passes"}


def arc(ex, q_from, cam_from, cam_to, target_m):
    """Waypoints that carry the camera from cam_from to cam_to round the target on the view sphere (directions
    turned in ARC_STEP_DEG steps, the distance interpolated), so the hand circles the thing instead of sweeping
    through it: [q, ...] ending at a pose for cam_to, or {"why"} when a waypoint does not solve or its leg fails."""
    T = np.asarray(target_m, dtype=float)
    a, b = np.asarray(cam_from, dtype=float) - T, np.asarray(cam_to, dtype=float) - T
    ra, rb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    ua, ub = a / ra, b / rb
    ang = P._ang(ua, ub)
    n = max(1, int(math.ceil(ang / ARC_STEP_DEG)))
    axis = np.cross(ua, ub)
    if float(np.linalg.norm(axis)) < 1e-6:
        axis = np.array([0.0, 0.0, 1.0])
    axis = P._unit(axis)
    pts, boxes = P.known(ex)
    refs = P.known_refs(ex)
    out, q = [], np.asarray(q_from, dtype=float)
    for k in range(1, n + 1):
        t = k / float(n)
        th = math.radians(ang * t)
        K = P._skew(axis)
        w = (np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)) @ ua
        cam = T + (ra + (rb - ra) * t) * w
        sol = P.solve_view(ex.model, [q, ex.model.q_home.copy()], cam, -w, T)
        if sol is None:
            return {"why": f"no joint pose on the way round ({ang * t:.0f} of {ang:.0f} deg)"}
        bad = P.path_check(ex, q, sol["q"], pts, boxes, refs=refs)
        if bad is not None:
            return {"why": f"{bad['check']} on the way round: {bad['why'][:80]}"}
        q = sol["q"]
        out.append(q)
    return out



def _r(ex, check, why, hint):
    return SL._said_refusal(check, why, hint)


def resolve(ex, target):
    """{point_m, size_mm, label, part, entry, candidate, front} or a refusal (unknown_target, stale_target)."""
    key = str(target).strip()
    st = ex.state["sight"]
    parts = SP.plist(ex)
    for c in st.get("candidates") or []:
        if c.get("id") == key:
            b = P.box_of(c)
            if b is None:
                break
            lo, hi, _ = b
            mid = 0.5 * (lo + hi)
            return {"point_m": mid, "size_mm": max(float(v) for v in c["size_mm"][:2]), "label": key, "part": None,
                    "entry": None, "candidate": key, "front": None, "centre_mm": c["where_mm"][:2]}
    e = ex.world.get(key)
    if e is None:
        part = SP.part_of(key, parts)
        if part is not None:
            found = [x for x in ex.world.query(kind="located", include_stale=False)
                     if SP.part_of(x["label"], parts) is part]
            e = found[-1] if found else None
        if e is None:
            return _r(ex, "unknown_target", f"no located or contact entry or part named {key[:40]}",
                      "locate {look, px, part} first; robot://world lists the ids; survey lists candidates c1, c2")
    if e.get("kind") not in ("located", "contact"):
        return _r(ex, "unknown_target", f"{key} is a {e.get('kind')} entry, not a point", "pass a located or "
                  "contact id")
    data = e.get("data") or {}
    if e.get("stale") or int(data.get("epoch", getattr(ex, "epoch", 0)) or 0) < int(getattr(ex, "epoch", 0)):
        why = data.get("stale_why") or "it may have moved"
        part = SP.part_of(e.get("label"), parts)
        return {"said": f"{e['id']} is stale ({why}): point at it again",
                "refused": {"check": "stale_target", "why": f"{e['id']} is stale: {why}",
                            "hint": {"tool": "locate", "args": {"look": _newest_look(ex),
                                                                "part": (part or {}).get("name", e.get("label"))},
                                     "needs": "px", "why": "point at it again in the newest look"}}}
    pt = e.get("point_mm") or data.get("bracket_mid_mm")
    if pt is None:
        return _r(ex, "unknown_target", f"{key} holds no point", "locate it again")
    part = SP.part_of(e.get("label"), parts)
    size = SP.largest_size_mm(part)
    centre = data.get("device_centre_mm")
    front = side_words.front(centre, pt) if centre is not None else None
    return {"point_m": np.asarray(pt, dtype=float) / MM, "size_mm": size, "label": e.get("label"), "part": part,
            "entry": e, "candidate": data.get("candidate"), "front": front, "centre_mm": centre}


def _newest_look(ex):
    ids = sorted(ex._looks, key=lambda k: int(k[1:]) if k[1:].isdigit() else 0)
    return ids[-1] if ids else None


def _sigma_mm(tg):
    """The target's own position error (mm): what its entry carries, None for a survey box or an entry with none."""
    data = (tg.get("entry") or {}).get("data") or {}
    return next((data[k] for k in ("sigma_p_mm", "uncertain_mm", "error_mm") if data.get(k) is not None), None)


def _extent_mm(ex, tg):
    """How far the thing may reach from the target point (mm), from what the survey and the device sheet say: a
    survey box reaches half its larger side from its middle; a located part sits somewhere on its device, so the
    device may reach its own largest rough size from the part (the sheet's size_mm); a part of a device with no
    stated size reaches its own largest rough size; nothing known: 0."""
    if tg.get("entry") is None:
        return float(tg.get("size_mm") or 0.0) / 2.0
    part = tg.get("part") or {}
    size = None
    if part.get("device") is not None:
        size = SP.device_size(SP.device_entry(SP.doc(ex), part.get("device")))
    return float(max(size)) if size else float(tg.get("size_mm") or 0.0)


def _pose(ex, q_now, tg, side, distance_mm, elevation_deg):
    return side_pose(ex, q_now, tg["point_m"], side, tg["size_mm"], distance_mm, elevation_deg, tg["front"],
                     sigma_mm=_sigma_mm(tg), extent_mm=_extent_mm(ex, tg))


def _go(ex, pose, note, intent=None, moves=None):
    ok, what, why, ran = SL.move(ex, pose["q"], note, intent, moves)
    if not ok:
        return None, what, why
    lk = ex._look_impl(note)
    return lk, "goal", ""


def _shows(ex, q):
    out = []
    W, H = ex.model.cam_w, ex.model.cam_h
    for e in ex.world.query(kind="located", include_stale=False):
        if e.get("point_mm") is None:
            continue
        u, v, dep = ex.model.project(q, np.asarray(e["point_mm"], dtype=float) / MM)
        if dep > 1e-3 and 0 <= u < W and 0 <= v < H:
            out.append(e["id"])
    return out


def _target_px(ex, q, tg):
    u, v, dep = ex.model.project(q, tg["point_m"])
    if not dep > 1e-3:
        return None
    return [round(u, 1), round(v, 1)]


def _roll_words(roll, axis):
    top = abs(float(axis[2])) >= math.cos(math.radians(P.UPRIGHT_VERTICAL_DEG))
    ref = "away from the robot" if top else "world up"
    if abs(roll) < UPRIGHT_SAID_DEG:
        return f"image up is {ref}"
    return f"image up is {abs(roll):.0f} deg from {ref}"


def _unlocated_control(ex, tg):
    """The first control part of the target's device with no fresh located entry (sheet order), or None."""
    parts = SP.plist(ex)
    dev = (tg.get("part") or {}).get("device")
    located = [SP.part_of(x["label"], parts) for x in ex.world.query(kind="located", include_stale=False)
               if x.get("point_mm") is not None]
    for p in parts:
        if p["role"] != "control" or (dev is not None and p["device"] != dev):
            continue
        if not any(q is p for q in located):
            return p["name"]
    return None


def _point_call(look_id, part, why):
    return {"tool": "locate", "args": {"look": look_id, "part": part}, "needs": "px", "why": why}


def _run(ex, target, side="robot_side", distance_mm=None, elevation_deg=None, detail="brief", intent=None,
         moves=None):
    """The view primitive's body (rcpx/r2/sight.py wraps it)."""
    if detail not in ("brief", "full"):
        return _r(ex, "detail", f"detail is brief or full, not {str(detail)[:20]!r}", "leave it out")
    side = "robot_side" if side is None else str(side).strip()
    if side not in SIDES:
        return _r(ex, "args", f"side is one of {', '.join(SIDES)}", 'for example view {"target": "c2", "side": '
                  '"around"}')
    for name, val, unit in (("distance_mm", distance_mm, "millimetres"), ("elevation_deg", elevation_deg, "degrees")):
        if val is not None and (isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val)
                                or (name == "distance_mm" and val <= 0)):
            return _r(ex, "args", f"{name} is a number of {unit}", "leave it out for the default")
    dark = ex._dark_refusal_now()
    if dark is not None:
        return {"said": dark["refused"]["why"], **dark}
    tg = resolve(ex, target)
    if "refused" in tg:
        return tg
    if side == "front" and tg["front"] is None:
        return _r(ex, "no_front", "front needs a located point with a known device centre (a survey box holds it)",
                  "locate a part first, or use around")
    q_now = np.asarray(ex._sense().q, dtype=float)
    if side == "around":
        return _around(ex, tg, q_now, distance_mm, elevation_deg, detail, intent, moves)
    pose = _pose(ex, q_now, tg, side, distance_mm, elevation_deg)
    got, tried = side, []
    if "q" not in pose:
        tried.append({"side": side, "why": pose["why"][:120]})
        for alt in SIDE_ORDER.get(side, ()):
            if alt == "front":
                continue
            p2 = _pose(ex, q_now, tg, alt, distance_mm, elevation_deg)
            if "q" in p2:
                pose, got = p2, alt
                break
            tried.append({"side": alt, "why": p2["why"][:80]})
    if "q" not in pose:
        return _r(ex, "no_view", f"no camera pose sees {tg['label']} from {SIDE_WORDS.get(side, side)} or a side "
                  f"next to it: the nearest try: {tried[0]['why']}", "survey, or locate the part in a look that "
                  "shows it")
    lk, what, why = _go(ex, pose, f"view of {tg['label']} from {got}", intent, moves)
    if lk is None:
        if what in ("contact", "cap") or what.startswith("stop"):
            return {"said": f"the camera's move stopped on the way ({what}): {why[:80]}", "stopped": what,
                    "next": {"tool": "look", "args": {}, "why": "see where the camera ended"}}
        return _r(ex, what, f"the camera's move did not run: {why[:120]}", "look, then try another side")
    q = np.asarray(ex._looks[lk["look_id"]]["q"], dtype=float)
    roll = P.roll_deg(P.cam_pose(ex.model, q)[1], P.upright(-P._unit(pose["cam_m"] - tg["point_m"]), tg["point_m"]))
    axis = P.cam_pose(ex.model, q)[1][:, 2] * -1.0
    said = (f"{lk['look_id']}: {tg['label']} seen from {SIDE_WORDS.get(got, got)}, {pose['d_mm']} mm away, "
            f"{pose['el_deg']:.0f} deg down; {_roll_words(roll, axis)}")
    if got != side:
        said += f" ({SIDE_WORDS.get(side, side)} was not reachable)"
    if pose.get("wide"):
        said += "; it is wider than one view from here: this look shows its middle, not all of it"
    out = {"said": said, "look_id": lk["look_id"], "roll_deg": round(roll, 1), "shows": _shows(ex, q),
           "target_px": _target_px(ex, q, tg), "got_side": got}
    if tried:
        out["tried"] = tried
    if lk.get("can_see") != "ok":
        out["can_see"] = lk.get("can_see")
    if detail == "full":
        out.update(look=lk, distance_mm=pose["d_mm"], elevation_deg=pose["el_deg"], azimuth_off_deg=pose["az_deg"],
                   how=pose["how"])
    out["pictures"] = [lk["annotated"]] if isinstance(lk.get("annotated"), str) else []
    part = _unlocated_control(ex, tg)
    if part is None and tg["entry"] is not None and tg["entry"].get("point_mm") is None and tg["part"]:
        part = tg["part"]["name"]
    out["next"] = None if part is None or lk.get("can_see") == "dark" else \
        _point_call(lk["look_id"], part, f"point at the {part} in this look")
    return out


def _tile(img, text):
    t = cv2.resize(np.ascontiguousarray(np.asarray(img)[:, :, :3]).astype(np.uint8), L.AROUND_TILE_PX,
                   interpolation=cv2.INTER_AREA)
    cv2.rectangle(t, (0, 0), (L.AROUND_TILE_PX[0], 18), (0, 0, 0), -1)
    cv2.putText(t, text[:40], (4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    return t


def _around(ex, tg, q_now, distance_mm, elevation_deg, detail, intent=None, moves=None):
    st = ex.state["sight"]
    tiles, looks, tried, sides = [], [], [], []
    tw, th = L.AROUND_TILE_PX
    q, cam = q_now, None
    for side in AROUND_SIDES[:L.AROUND_MAX]:
        pose = _pose(ex, q, tg, side, distance_mm, elevation_deg) if cam is None else \
            side_pose(ex, q, tg["point_m"], side, tg["size_mm"], distance_mm, elevation_deg, tg["front"], path=False,
                      sigma_mm=_sigma_mm(tg), extent_mm=_extent_mm(ex, tg))
        if "q" not in pose:
            tried.append({"side": side, "why": pose["why"][:80]})
            continue
        if cam is not None:          # from the last side to this one round the thing, not through it
            way = arc(ex, q, cam, pose["cam_m"], tg["point_m"])
            if isinstance(way, dict):
                tried.append({"side": side, "why": way["why"][:80]})
                continue
            stop = None
            for qw in way[:-1]:
                ok, what, why, _ = SL.move(ex, qw, f"round {tg['label']}", intent, moves)
                if not ok:
                    stop = (what, why)
                    break
            if stop is not None:
                tried.append({"side": side, "why": f"{stop[0]}: {stop[1][:60]}"})
                if stop[0] == "dark":
                    break
                q = np.asarray(ex._sense().q, dtype=float)
                continue
            pose = {**pose, "q": way[-1]}
        lk, what, why = _go(ex, pose, f"around {tg['label']}: {side}", intent, moves)
        if lk is None:
            tried.append({"side": side, "why": f"{what}: {why[:60]}"})
            if what == "dark":
                break
            continue
        rec = ex._looks[lk["look_id"]]
        q = np.asarray(rec["q"], dtype=float)
        img = SL._image(ex, rec)
        tiles.append(_tile(img, f"{lk['look_id']} {SIDE_WORDS[side]}"))
        looks.append({"look_id": lk["look_id"], "side": side, "q": q, "d_mm": pose["d_mm"], "can_see": lk["can_see"]})
        cam = P.cam_pose(ex.model, q)[0]
        sides.append(side)
        if lk["can_see"] == "dark":
            break
    if not looks:
        return _r(ex, "no_view", f"no side of {tg['label']} could be seen: {tried[0]['why'] if tried else ''}",
                  "survey, or locate the part in a look that shows it")
    sheet = np.zeros((2 * th, 2 * tw, 3), np.uint8)
    for i, t in enumerate(tiles):
        r, c = divmod(i, 2)
        sheet[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = t
    st["arounds"] = st.get("arounds", 0) + 1
    sid = f"around_{st['arounds']}"
    path = ex._save(sheet, f"{sid}_sheet")
    st["sheets"][sid] = {"size": [2 * tw, 2 * th], "cols": 2, "tile_px": [tw, th],
                         "tiles": [x["look_id"] for x in looks], "path": path, "known": True}
    best = None
    for x in looks:
        u, v, dep = ex.model.project(x["q"], tg["point_m"])
        if dep > 1e-3 and 0 <= u < ex.model.cam_w and 0 <= v < ex.model.cam_h and x["can_see"] != "dark":
            if best is None or dep < best[0]:
                best = (dep, x)
    said = (f"{sid}: {tg['label']} from {len(looks)} side{'s' if len(looks) != 1 else ''} "
            f"({', '.join(SIDE_WORDS[s] for s in sides)}) on one sheet")
    out = {"said": said, "sheet_id": sid, "looks": [{"look_id": x["look_id"], "side": x["side"]} for x in looks]}
    if tried:
        out["tried"] = tried
    if detail == "full":
        out["tile_px"] = [tw, th]
    out["pictures"] = [path] if isinstance(path, str) else []
    if best is None:
        out["next"] = None
    else:
        lid = best[1]["look_id"]
        names = [m.__name__.rsplit(".", 1)[-1] for m in getattr(ex, "mods", [])]
        part = _unlocated_control(ex, tg) or ((tg.get("part") or {}).get("name"))
        if "witness" in names:
            out["next"] = {"tool": "look", "args": {"like": lid}, "why": "the tile that shows it largest"}
        elif part:
            out["next"] = _point_call(lid, part, f"point at the {part} in the tile that shows it largest")
        else:
            out["next"] = None
    return out


def plan_camera(ex, target_mm, side, distance_mm=None, elevation_deg=None):
    """ex.state["sight"]["plan_camera"] (SPEC-R3 3.3): {q or None, got_side, why}, dry (nothing moves)."""
    if side not in SIDES or side in ("around", "front"):
        return {"q": None, "got_side": None, "why": f"side {side!r} has no single pose"}
    T = np.asarray(target_mm, dtype=float) / MM
    q_now = np.asarray(ex._sense().q, dtype=float)
    p = side_pose(ex, q_now, T, side, None, distance_mm, elevation_deg, None)
    if "q" in p:
        return {"q": p["q"], "got_side": side, "why": ""}
    return {"q": None, "got_side": None, "why": p["why"]}


__all__ = ["SIDES", "SIDE_ORDER", "arc", "plan_camera", "resolve", "run", "side_dir", "side_pose"]


def view_distances(ex, size_mm=None, distance_mm=None, sigma_mm=None, q=None, extent_mm=None):
    """([camera distances mm, the nearest first], wide): the distances view's pose search tries. The nearest is
    the greatest of: the camera's near focus (the round three limit BRACKET_NEAR_MM as the session derives it); the
    distance at which a thing of size_mm stays inside the central FRAME_KEEP_FRAC of the frame (size x f_px / the
    pixels of that share of the smaller frame side); and the tool's own reach from the lens (sight_path.tool_span_m)
    plus how far the thing may reach from the target point (extent_mm: half a survey box, the device's rough size
    for a part located on it) plus the band of the target's position error (BAND x sigma_mm), so that the hand the
    camera rides on cannot reach the thing it looks at. The farthest is the arm's reach (a model that states none:
    the present camera's distance from the base); between them the search steps by the relative error the laws
    accept (x (1 + REL)).
    wide: the thing does not fit one view even from the reach (then the reach alone). A distance_mm the operator
    gave is tried alone, cut to the reach (the operator's word; the path check and the gates still answer)."""
    m = ex.model
    q = ex._sense().q if q is None else q
    reach = float((m.reach() or {}).get("max_reach_mm") or np.linalg.norm(P.cam_pose(m, q)[0]) * MM)
    if distance_mm is not None:
        return [min(float(distance_mm), reach)], False
    near = SB.limit(ex, "BRACKET_NEAR_MM", m)
    fit = 0.0 if size_mm is None else float(size_mm) * m.f_px / (L.FRAME_KEEP_FRAC * min(m.cam_w, m.cam_h))
    if fit > reach:
        return [reach], True
    d = max(near, fit, P.tool_span_m(ex, q) * MM + float(extent_mm or 0.0) + R.BAND * float(sigma_mm or 0.0))
    out = [d]
    if d > 0:
        while d * (1 + R.REL) <= reach:
            d *= 1 + R.REL
            out.append(d)
    return out, False


def run(ex, target, side="robot_side", distance_mm=None, elevation_deg=None, detail="brief", clear=None):
    """The view primitive's body (rcpx/r2/sight.py wraps it): the operator's word (clear) goes to every camera move
    of the call, and a call that moved reports its control periods and the binding bound of its moves."""
    intent, why = SB.clear_word(clear)
    if why:
        return _r(ex, "args", why, "leave clear out, or say true after a look")
    t0, moves = ex.sim_time(), []
    out = _run(ex, target, side, distance_mm, elevation_deg, detail, intent, moves)
    if ex.sim_time() > t0:
        out = SB.report(ex, out, moves, t0, detail)
        if "next" in out:
            out["next"] = out.pop("next")
    return out
