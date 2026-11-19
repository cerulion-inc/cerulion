# SPDX-License-Identifier: AGPL-3.0-only
"""Package see_spot: a pixel becomes a point, a thing, a
part, a wall and an opening, with no motion and no control period. No tools of its own: rcpx/r2/place_point.py
resolves every spot of the whole-step tools through spot(), and a mark on a scene look reads its point here.

    spot(ex, value, role="pick") -> spot
    value: {"look": id, "px": [u, v]} (a look of this session, scene camera or hand camera; pixel (u, v) covers
           [u, u + 1), its centre is at u + 1/2), a located label, or [x, y, z] in base frame mm.

A spot is answered for every pixel inside a look and is never a refusal for doubt: its only refusals are malformed
arguments (check spot, look, pixel, label). What is unsure rides in "doubt" with the ONE call that would remove it.

A located label carries the error that was stored with it. A point of marks (triangulate) is worth what its marks
fix: marks_error() gives its error along the view and across, both triangulates store it with the label, and the
spot states it. Marks whose rays differ by no more than the camera resolves are ONE view: they fix no depth, and
the spot is then where that view meets the stated support, with the height not seen (source 3 below). A point
that was typed, or a label stored with no error, has no error to state: sigma is None and the doubt says so
(nothing stands in for an error nobody measured).

Sources of a pixel's point, the first that exists:
1. the look's own depth picture (the body's camera gives depth: Backend.depth, kept with the look);
2. another look of the session whose depth shows the pointed ray (the newest first);
3. one view and the stated support: where the ray meets the support plane; the height is not seen.
A depth point that lies inside the robot's own hulls at the look's joint pose is the robot's own (a 3D test) and
is no surface of the world: the next source is asked and the answer says so.
A pixel where two surfaces meet within one pixel is read as the surface its COLOUR shows, from the picture around
it (shows()): the depth under such a pixel is one sample inside it, of either surface; where the colours
do not tell the two apart, see_cloud's own reading stands (the nearer surface within one pixel).

spot = {"point_m": [x, y, z] (base frame; always present: where the pointed ray lands),
        "sigma_m": {"along_ray", "across"} (along_ray None: not seen; both None: no error is stated),
        "ray": the unit direction the along_ray error lies along (base frame); absent for a point with no view,
        "source": text,
        "entry_m": role place only: where the pointed ray crosses the height of the rim it comes in over
                   (extent.way_in_z: the top of what the ray passes over on its way in); when the ray came in
                   above that rim and landed on a wall, the level middle between the two walls (extent.slot) at
                   the landing point's height; absent when the ray passes over nothing of the thing,
        "edge": {"other_point_m", "behind_m"} when another surface lies within one pixel,
        "extent": {"kind": "thing" | "bare", "base_z", "top_z", "top_near_z", "narrow_m", "bearing_rad", "wide_m",
                   "centre_xy", "cut", "border", "sigma_m", "points", "part", "hollow_m", "wall", "way_in_z"} or
                  absent,
        "doubt": [text, ...], "remove_by": {"tool", "args"} or None, "periods": 0, "looks": [ids],
        "names": {"by": "the pixel lies on it" | "within the spot's own error" | (role part) "it closes the
                 pixel's level in on every side", "distance_m", "error_m", "on_bare_support" (a place)}
                 when the extent is a thing (rcpx/r2/place_near.py, a pixel on a level names the ONE thing
                 whose seen pixels lie within sigma_m.across of it, across the view; a place only on the support),
        "answer": roles pick and part, where the look shows the pointed place EMPTY (place_near's five
                  conditions): {"empty": True, "why": "none" | "two", "near": [{"look", "px", "distance_mm",
                  "narrow_mm", "wide_mm", "top_mm"}], "marked", "crop", "crop_box_px", "crop_scale"}: the tool that
                  gets it moves nothing and says so, with the look's help (the look's own picture with
                  the pixel and each near entry marked, a crop around the pixel),
        "near": a place pixel on bare support that names no thing: what the look shows near it, rows as above,
        "not_answered": the conditions that failed, when no thing is named and the tool acts on the point,
        "shown": a typed point: {"look", "age_s", "acts_since", "extent": "thing" | "bare" | "not in any look"},
        "depth_sigma_m": the depth's stated error at the point's range (a pixel read from a depth picture)}

The sizes that decide are the hand's own (rcpx/r2/place_hand.py Jaws, read only: widest opening, finger depth, pad
width), the pixel's footprint and the depth's stated sigma (rcpx/r2/see_cloud.py). Nothing here reads truth, names
a thing or moves the robot.
"""

import math

import numpy as np

from rcpx.bounds_ratios import BAND, GIVE, QUANT
from rcpx.context import refusal
from rcpx.r2 import place_near
from rcpx.r2 import see_cloud as SC

ID = "R6-see"
MM = 1000.0          # unit
XYZ = 3              # index: a point has three coordinates
TOOLS = []
SEEN_NOT_MET = "the surface the camera sees, not what the hand meets"
ONE_VIEW = "one view and the stated support: the height is not seen"
HOW = "{\"look\": id, \"px\": [u, v]}"


def _state(ex):
    return ex.state.setdefault("see_spot", {})


def support_of(ex):
    """(up unit vector, height of the support along it, its stated error or None) from the space book, else the
    stated floor."""
    try:
        from rcpx import bounds
        got = bounds.of(ex).space.support()
        if got and got[0] is not None:
            n = np.asarray(got[0][0], dtype=float)
            return n / np.linalg.norm(n), float(got[0][1]) / float(np.linalg.norm(n)), got[1]
    except Exception:
        pass
    return np.array([0.0, 0.0, 1.0]), float(ex.cfg["floor_z_m"]), None


def hand_of(ex):
    """{"open_m", "pad_m", "finger_m"}: the hand's own sizes; an arm with no jaws: its tool's width for all three."""
    st = _state(ex)
    if st.get("hand") is None:
        try:
            from rcpx.r2 import place_hand
            J = place_hand.Jaws(ex.model)
            boxes = [b for b in J.finger_boxes(1.0) if b is not None]
            pad = min(float(b["b"][1] - b["b"][0]) for b in boxes)
            finger = J.finger_len_m if J.finger_len_m else J.pad_height_m
            st["hand"] = {"open_m": float(J.max_m), "pad_m": pad, "finger_m": float(finger)}
        except Exception:
            from rcpx import bounds
            w = float(bounds.of(ex).robot.w_tool())
            st["hand"] = {"open_m": w, "pad_m": w, "finger_m": w}
    return st["hand"]


# ------------------------------------------------------------------------------------------------ looks
def camera_of_model(ex, q):
    """The hand camera at joint pose q as a look record keeps it: lens and pose (base frame; R in the simulator's
    camera convention: image right, image up, toward the viewer)."""
    m = ex.model
    pc, Rc = m.fk(q)["camera"]
    return {"name": "wrist", "w": int(m.cam_w), "h": int(m.cam_h), "fx": float(m.f_px), "fy": float(m.f_px),
            "cx": m.cam_w * SC.MID, "cy": m.cam_h * SC.MID, "pos": np.asarray(pc, dtype=float).copy(),
            "R": np.asarray(Rc, dtype=float).copy()}


def take_depth(ex, camera):
    """What a look keeps beside its picture: {"depth": metres along the optical axis or None, "depth_stated":
    {"sigma_m_at_1m", "grows", "source"} or None}. The picture and its depth are one frame (Backend.depth)."""
    from rcpx.backend import depth_of
    z, stated = depth_of(ex.backend, camera)
    return {"depth": z, "depth_stated": stated}


def look_of(ex, look_id):
    """(key, record, fixed: bool) of a look of this session, or (key, None, None)."""
    key = str(look_id)
    scene = (ex.state.get("scene_camera") or {}).get("looks", {})
    if key in scene:
        return key, scene[key], True
    key, rec = ex._look_rec(key)
    return key, rec, False


def view_of(rec):
    c = rec["cam"]
    flip = np.diag([1.0, -1.0, -1.0])          # image up and toward the viewer, to image down and along the view
    return SC.View(c["fx"], c["fy"], c["cx"], c["cy"], c["w"], c["h"], c["pos"], np.asarray(c["R"]) @ flip, c["name"])


def _sigma_at(rec, rng, foot):
    """(the depth's sigma at that range, said: bool): the stated error, growing as stated; none stated: one
    footprint."""
    stated = rec.get("depth_stated")
    if not isinstance(stated, dict) or stated.get("sigma_m_at_1m") is None:
        return foot, False
    s = float(stated["sigma_m_at_1m"])
    return (s * rng * rng if stated.get("grows") == "square" else s), True


def _bodies(ex, rec):
    """The robot's own hulls at the look's joint pose (and its jaws as read then), for see_cloud.inside_hulls."""
    import mujoco
    st = _state(ex)
    model = ex.model
    if st.get("faces") is None:
        st["faces"] = {name: SC.hull_faces(h["points_m"]) + (float(h["tol_m"]),) for name, h in model.hulls().items()}
        st["data"] = mujoco.MjData(model.mj)
    d, m = st["data"], model.mj
    d.qpos[:] = model._qfull
    d.qpos[model._qadr] = np.asarray(rec["q"], dtype=float)
    jaws, g = rec.get("jaws"), model.spec.get("gripper") or {}
    if jaws is not None and g.get("joints"):
        adr = [m.jnt_qposadr[m.joint(j).id] for j in g["joints"]]
        if len(adr) == len(np.atleast_1d(jaws)):
            d.qpos[adr] = np.atleast_1d(jaws)
    mujoco.mj_kinematics(m, d)
    out = []
    for name, (A, b, lo, hi, tol) in st["faces"].items():
        bid = m.body(name).id
        out.append({"R": d.xmat[bid].reshape(XYZ, XYZ).copy(), "p": d.xpos[bid].copy(), "A": A, "b": b, "lo": lo,
                    "hi": hi, "tol_m": tol})
    return out


def cloud_of(ex, rec):
    """(view, points H x W x 3, valid: has depth and is not the robot's own, own) of a look with depth, kept with the
    look; None when the look has no depth."""
    if rec.get("depth") is None or rec.get("cam") is None:
        return None
    if rec.get("_cloud") is None:
        view = view_of(rec)
        z = np.asarray(rec["depth"], dtype=float)
        if z.shape != (view.h, view.w):       # not this camera's picture: no point of it can be placed
            rec["_depth_problem"] = (f"its depth picture is {' x '.join(str(n) for n in z.shape[::-1])} for a "
                                     f"camera of {view.w} x {view.h}")
            return None
        P = SC.cloud(view, z)
        has = np.isfinite(z) & (z > 0.0)
        rng = np.linalg.norm(P - view.pos, axis=-1)
        foot = float(np.median(rng[has])) / view.f if has.any() else 0.0
        try:
            own = SC.inside_hulls(P, _bodies(ex, rec), max(BAND * _sigma_at(rec, float(np.median(rng[has])), foot)[0],
                                                           foot)) & has
        except Exception as exc:      # a body whose hulls cannot be read: nothing is called the robot's own, and said
            own = np.zeros(has.shape, dtype=bool)
            rec["_own_problem"] = f"{type(exc).__name__}: {exc}"
        rec["_cloud"] = (view, P, has & ~own, own)
    return rec["_cloud"]


def _moved_m(ex, rec, fixed):
    """How far the camera moved in one control period when the look was read in motion (0 for a fixed camera)."""
    qd = rec.get("qd")
    if fixed or qd is None or not np.any(np.asarray(qd) != 0.0):
        return 0.0
    q = np.asarray(rec["q"], dtype=float)
    p0 = np.asarray(ex.model.fk(q)["camera"][0], dtype=float).copy()
    p1 = np.asarray(ex.model.fk(q + np.asarray(qd, dtype=float) * float(ex.dt))["camera"][0], dtype=float)
    return float(np.linalg.norm(p1 - p0))


def _other_camera(ex, rec, fixed):
    """One look that would show the spot from elsewhere: the hand camera for a fixed look, a fixed camera for a
    hand look; None when the body has no other camera."""
    if fixed:
        return ({"tool": "look", "args": {"camera": "wrist"}, "periods": 0}
                if (ex.model.spec.get("camera") or {}).get("body") else None)
    try:
        from rcpx.r2 import scene_camera
        names = sorted(scene_camera.cameras(ex))
    except Exception:
        names = []
    return {"tool": "look", "args": {"camera": names[0]}, "periods": 0} if names else None


# ------------------------------------------------------------------------------------------------ the spot
def marks_error(rays, point, f_px):
    """What a point made of marked rays is worth. rays: [(origin, direction), ...] (base frame, m); point: the
    point they give; f_px: the focal length in pixels of the coarsest camera that marked. A mark is good to one
    pixel (as rcpx.perception.triangulation_quality takes it):
    - across the view: the larger of one pixel's footprint at the point's range and how far the rays miss each
      other;
    - along the view: QUANT footprints over the sine of the angle the two widest rays keep when each of the two
      marks is one pixel off toward the other (never under perception's own figure, which takes the angle as
      marked). Rays that differ by no more than those QUANT pixels are one ray within what the camera resolves:
      they fix no depth, and "along_m" is None.
    Returns {"along_m", "across_m", "miss_m", "origin", "dir" (the mean ray: where the cameras stood, the way they
    looked), "angle_rad", "pixel_rad", "f_px"}."""
    o = np.asarray([r[0] for r in rays], dtype=float)
    d = np.asarray([r[1] for r in rays], dtype=float)
    d = d / np.linalg.norm(d, axis=1, keepdims=True)
    w = np.asarray(point, dtype=float) - o
    miss = float(np.max(np.linalg.norm(w - np.sum(w * d, axis=1, keepdims=True) * d, axis=1)))
    foot = float(np.mean(np.linalg.norm(w, axis=1))) / float(f_px)
    angle = float(np.max(np.arccos(np.clip(np.abs(d @ d.T), 0.0, 1.0))))
    pixel = math.atan2(1.0, float(f_px))
    spare = angle - QUANT * pixel
    mean = d.mean(0)
    return {"along_m": None if spare <= 0.0 else QUANT * foot / math.sin(spare), "across_m": max(foot, miss),
            "miss_m": miss, "origin": o.mean(0), "dir": mean / np.linalg.norm(mean), "angle_rad": angle,
            "pixel_rad": pixel, "f_px": float(f_px)}


def no_depth_text(worth):
    """What a triangulate says first when its marks fix no depth."""
    return (f"the rays differ by only {math.degrees(worth['angle_rad']):.2f} deg, no more than {QUANT} pixels of the "
            f"camera subtend ({math.degrees(QUANT * worth['pixel_rad']):.2f} deg): the marks are one view and fix no "
            "depth along it, so the point may lie anywhere along the view; a look from further to the side would "
            "fix it")


def marks_data(worth):
    """What a triangulate stores with its label beside error_mm and looks (mm, as a world entry keeps numbers):
    {"sigma_mm": {"along_view" (None: the marks fix no depth), "across"}, "ray": {"origin_mm", "dir"},
    "ray_angle_deg", "f_px"}. _label() reads exactly these."""
    return {"sigma_mm": {"along_view": None if worth["along_m"] is None else float(worth["along_m"] * MM),
                         "across": float(worth["across_m"] * MM)},
            "ray": {"origin_mm": [float(x * MM) for x in worth["origin"]], "dir": [float(x) for x in worth["dir"]]},
            "ray_angle_deg": float(math.degrees(worth["angle_rad"])), "f_px": worth["f_px"]}


def _side_look(ex, looks):
    """One look that shows a marked point from another place: a camera none of its marks came from; None when the
    body has no such camera."""
    scene = (ex.state.get("scene_camera") or {}).get("looks") or {}
    used = {scene[k].get("camera") if k in scene else "wrist" for k in looks}
    try:
        from rcpx.r2 import scene_camera
        names = sorted(scene_camera.cameras(ex))
    except Exception:
        names = []
    if (ex.model.spec.get("camera") or {}).get("body"):
        names = ["wrist"] + names
    free = [n for n in names if n not in used]
    return {"tool": "look", "args": {"camera": free[0]}, "periods": 0} if free else None


def _label(ex, value):
    """A located label's point with the error that was stored with it (module docstring)."""
    found = ex.world.query(kind="located", label=value)
    if not found or not found[-1].get("point_mm"):
        return refusal("label", f"no located point {value!r}", "point with " + HOW)
    data = found[-1].get("data") or {}
    p = np.asarray(found[-1]["point_mm"], dtype=float) / MM
    out = {"point_m": [float(x) for x in p], "source": f"the located label {value!r}", "doubt": [], "remove_by": None,
           "periods": 0, "looks": [str(k) for k in data.get("looks") or []]}
    sig, ray = data.get("sigma_mm"), data.get("ray") or {}
    if isinstance(sig, dict) and sig.get("across") is not None and ray.get("dir") is not None:
        d = np.asarray(ray["dir"], dtype=float)
        o = np.asarray(ray["origin_mm"], dtype=float) / MM
        how = f"{len(out['looks'])} marks whose rays differ by {float(data.get('ray_angle_deg') or 0.0):.2f} deg"
        out.update(ray=[float(x) for x in d], remove_by=_side_look(ex, out["looks"]))
        if sig.get("along_view") is not None:
            along, across = float(sig["along_view"]) / MM, float(sig["across"]) / MM
            out["sigma_m"] = {"along_ray": along, "across": across}
            out["doubt"].append(f"the label rests on {how}: its point is known to {along * MM:.0f} mm along the view "
                                f"and {across * MM:.1f} mm across; a mark in a look from further to the side narrows "
                                "it")
            return out
        # one ray within what the camera resolves: the label is one view, and is read as one
        up, h, _ = support_of(ex)
        hit = SC.ray_at_height(o, d, up, h)
        text = (f"the label rests on {how}, no more than {QUANT} pixels of the camera subtend: they are one view "
                "and fix no depth")
        if hit is None:
            out["sigma_m"] = {"along_ray": None, "across": float(sig["across"]) / MM}
            out["doubt"].append(text + "; that view never meets the stated support, so the point is the marks' own "
                                "and may lie anywhere along the view")
            return out
        foot = float(np.linalg.norm(hit - o)) / float(data.get("f_px") or ex.model.f_px)
        out.update(point_m=[float(x) for x in hit], sigma_m={"along_ray": None, "across": foot})
        out["doubt"].append(text + "; the point is taken where that view meets the stated support, and the height of "
                            "what stands there is not seen; a mark in a look from further to the side fixes it")
        return out
    err = next((data[k] for k in ("sigma_p_mm", "uncertain_mm") if data.get(k) is not None), None)
    if err is not None and float(err) > 0.0:       # an error somebody stored with the point (a touch, a look's own)
        out["sigma_m"] = {"along_ray": float(err) / MM, "across": float(err) / MM}
        return out
    return _no_error(out)


def _no_error(out):
    out["sigma_m"] = {"along_ray": None, "across": None}
    out["doubt"].append("no error is stated for this point: it is taken as given, and nothing here says how far off "
                        "it may be")
    return out


def _typed(ex, value, role="pick"):
    if isinstance(value, str):
        return _label(ex, value)
    try:
        p = np.asarray([float(x) for x in value], dtype=float) / MM
    except (TypeError, ValueError):
        p = None
    if p is None or p.shape != (XYZ,) or not np.all(np.isfinite(p)):
        return refusal("spot", "a point is [x, y, z] in base frame mm", "or point in a look: " + HOW)
    out = _no_error({"point_m": [float(x) for x in p], "source": "the point you gave", "doubt": [], "remove_by": None,
                     "periods": 0, "looks": []})
    try:                        # what the newest look that shows the point shows there; no motion
        out["shown"], ans = place_near.shown(ex, p, role)
    except Exception as exc:
        out["shown"], ans = {"look": None, "age_s": None, "acts_since": None, "extent": "not in any look",
                             "problem": f"{type(exc).__name__}: {exc}"}, None
    if ans:
        out["answer"] = ans
    return out


def _colour_of(rec, shape):
    """The look's own colour picture as floats (H x W x 3), kept with the look; None when it is not a saved image
    of the depth picture's size."""
    if rec.get("_rgb") is None:
        from rcpx.r2 import frames_draw as draw
        img = draw.load_rgb(rec.get("img")) if rec.get("img") is not None else draw.load_rgb(rec.get("frame"))
        rec["_rgb"] = img.astype(float) if img is not None and img.shape[:2] == tuple(shape) else False
    return None if rec["_rgb"] is False else rec["_rgb"]


def shows(rec, cloud, u, v, hand, sigma):
    """Which of two surfaces an EDGE pixel shows: a pixel where two surfaces meet within one pixel
    (the ranges of its own ring differ as see_cloud tells two surfaces apart) holds the depth of ONE of them, at
    the point inside the pixel where the depth picture samples it, while its colour mixes what the whole pixel
    shows. It is read from the picture AROUND it: the pixels of the window QUANT pixels around it are split at the
    middle of the two ranges, and those whose own ring is all one side give each surface's colour (the median) and
    its spread (the median distance from it). Where the two colours differ by more than BAND times the larger
    spread (and than one level of the picture's own 8-bit step), the pixel shows the surface whose colour it is
    nearer along the line between them. Returns (u, v, snap, far, shown): the pixel to read and whether see_cloud's
    own snap to the nearer surface applies; far: {"said", "other_point_m", "in_front_m"} when the FARTHER surface
    is taken (at the pixel when its own depth is that one's, else at the nearest pixel of its ring that is), else
    None; shown: "far", "near" (the colour is the nearer surface's: see_cloud's snap to it stands, confirmed) or
    None (no edge, no colour picture, or colours that do not tell the two apart: the pixel is read as see_cloud
    reads it)."""
    view, P, valid, _ = cloud
    H, W = valid.shape
    ring = [(u + du, v + dv) for dv in SC.RING for du in SC.RING
            if 0 <= u + du < W and 0 <= v + dv < H and valid[v + dv, u + du]]
    rng = {q: float(np.linalg.norm(P[q[1], q[0]] - view.pos)) for q in ring}
    near, far = min(ring, key=lambda q: rng[q]), max(ring, key=lambda q: rng[q])
    gap = max(BAND * rng[near] / view.f, GIVE * hand["pad_m"], QUANT * BAND * sigma)
    if rng[far] - rng[near] <= gap:
        return u, v, True, None, None
    img = _colour_of(rec, valid.shape)
    if img is None:
        return u, v, True, None, None
    u0, u1, v0, v1 = max(0, u - QUANT), min(W, u + QUANT + 1), max(0, v - QUANT), min(H, v + QUANT + 1)
    ok = valid[v0:v1, u0:u1]
    r = np.linalg.norm(P[v0:v1, u0:u1] - view.pos, axis=-1)
    mid = (rng[near] + rng[far]) * SC.MID
    side = np.where(ok, np.where(r < mid, 1, -1), 0)          # 1: the nearer surface, -1: the farther, 0: no depth
    pad = np.pad(side, 1)
    same = np.ones(side.shape, dtype=bool)
    for dv in SC.RING:
        for du in SC.RING:
            nb = pad[1 + dv:1 + dv + side.shape[0], 1 + du:1 + du + side.shape[1]]
            same &= (nb == side) | (nb == 0)
    cols = img[v0:v1, u0:u1]
    got = []
    for k in (1, -1):
        sel = same & (side == k)
        if int(sel.sum()) < QUANT:
            return u, v, True, None, None
        c = np.median(cols[sel], axis=0)
        got.append((c, float(np.median(np.linalg.norm(cols[sel] - c, axis=-1)))))
    (cn, sn), (cf, sf) = got
    d = cf - cn
    length = float(np.linalg.norm(d))
    if length <= max(BAND * max(sn, sf), 1.0):        # 1.0: one level of the picture's 8-bit step
        return u, v, True, None, None
    c = img[v, u]
    if float((c - cn) @ d) / (length * length) <= SC.MID:
        return u, v, True, None, "near"
    if rng[(u, v)] >= mid:
        take = (u, v)
    else:
        take = min((q for q in ring if rng[q] >= mid), key=lambda q: (abs(q[0] - u) + abs(q[1] - v), q[1], q[0]))
    said = (f"your pixel lies where two surfaces meet within one pixel, and its colour is that of the farther one "
            f"({float(np.linalg.norm(c - cf)):.0f} levels from it, {float(np.linalg.norm(c - cn)):.0f} from the "
            f"nearer, which stands {(rng[far] - rng[near]) * MM:.0f} mm in front of it): the farther surface is "
            "taken")
    return take[0], take[1], False, {"said": said, "other_point_m": [float(x) for x in P[near[1], near[0]]],
                                     "in_front_m": rng[far] - rng[near]}, "far"


def _depth_source(look_id, view, foot, rng, of_ray=None):
    text = (f"depth picture of look {look_id} (camera {view.name}, {view.w} px; one pixel is {foot * MM:.1f} mm at "
            f"{rng:.2f} m): {SEEN_NOT_MET}")
    return text if of_ray is None else text + f"; cut by the ray of your pixel in look {of_ray}"


def _from_depth(ex, rec, key, fixed, u, v, role, o, d, of_ray=None):
    """The spot from a look's depth at pixel (u, v), or None when that pixel shows no surface of the world."""
    got = cloud_of(ex, rec)
    if got is None:
        return None
    view, P, valid, own = got
    if not valid[v, u]:
        return None
    up, sup_z, sup_err = support_of(ex)
    hand = hand_of(ex)
    rng0 = float(np.linalg.norm(P[v, u] - view.pos))
    sigma, said = _sigma_at(rec, rng0, rng0 / view.f)
    # a place's edge pixel is read as the surface its colour shows; a pixel of another look's depth,
    # cut by the pointed ray, is read as see_cloud reads it
    uu, vv, snap, far, shown = shows(rec, got, u, v, hand, sigma) if of_ray is None and role in place_near.PLACES \
        else (u, v, True, None, None)
    e = SC.extent(P, valid, view, uu, vv, up, hand, sigma, None if of_ray is None else (o, d), snap=snap)
    moved = _moved_m(ex, rec, fixed)
    names, empty = None, None
    if of_ray is None:          # what the pixel NAMES: the thing it lies on, or the one within its own error
        e, names, empty = place_near.name(got, uu, vv, up, hand, role, e, sigma, moved, support=(sup_z, sup_err),
                                          shown=shown)
    p = np.asarray(e["point"], dtype=float)
    foot, rng = e["foot_m"], e["range_m"]
    out = {"point_m": [float(x) for x in p], "sigma_m": {"along_ray": sigma, "across": SC.MID * foot + moved},
           "ray": [float(x) for x in (p - view.pos) / rng],       # the depth's own error lies along ITS camera's ray
           "source": _depth_source(key, view, foot, rng, of_ray), "doubt": [f"the figure is of {SEEN_NOT_MET}"],
           "remove_by": None, "periods": 0, "looks": [key] if of_ray is None else [of_ray, key],
           "depth_sigma_m": sigma}
    if names:
        out["names"] = names
        if names["by"] == place_near.WITHIN:
            out["doubt"].append(f"your pixel lies {names['distance_m'] * MM:.1f} mm beside the thing taken, within "
                                f"the look's own error across the view ({names['error_m'] * MM:.1f} mm)"
                                + ("; the pixel itself lies on bare support" if names.get("on_bare_support") else ""))
        elif names["by"] == place_near.AROUND:
            out["doubt"].append(f"your pixel shows a level that one thing closes in on every side (a pane the depth "
                                f"does not show, or the floor inside it): that thing is taken, at its seen point "
                                f"nearest your pixel ({names['distance_m'] * MM:.0f} mm across the view)")
    if not said:
        out["doubt"].append("depth error not stated: one pixel's footprint stands in for it")
    if moved > SC.MID * foot:
        out["doubt"].append(f"the look was read in motion: the camera moved {moved * MM:.1f} mm in one control period")
    if rec.get("_own_problem"):
        out["doubt"].append("the robot's own shape could not be read: its own body may be taken for a thing")
    if far:                     # the farther of two surfaces within one pixel was taken: the other stands IN FRONT
        out["edge"] = {"other_point_m": far["other_point_m"], "behind_m": -far["in_front_m"]}
        out["doubt"].append(far["said"])
        out["remove_by"] = _other_camera(ex, rec, fixed)
    elif e.get("edge"):
        out["edge"] = {"other_point_m": e["edge"]["other_point"], "behind_m": e["edge"]["behind_m"]}
        out["doubt"].append(f"the pixel lies on an edge: another surface stands {e['edge']['behind_m'] * MM:.0f} mm "
                            "behind the one taken, within one pixel")
        out["remove_by"] = _other_camera(ex, rec, fixed)
    if e["kind"] is None:
        out["doubt"].append("no thing is made out here (" + e.get("why", "no level is seen below it") + "): the point "
                            "alone is given")
        low = place_near.cannot_show_low(ex, hand, sigma) if role in ("pick", "part") else None
        if role in ("pick", "part") and not low:     # what the level reading found: the condition that failed
            low = (e.get("why", "no level is seen below it") + f" within {SC.REACH * hand['open_m'] * MM:.0f} mm of it "
                   "across the view: the look cannot say the place is bare support")
        if low:                 # (never answered then; the doubt says which condition failed)
            out["not_answered"] = [low]
            out["doubt"].append(low + ". The hand goes to the point with the contact stop armed")
        out["remove_by"] = out["remove_by"] or _other_camera(ex, rec, fixed)
        return out
    keys = ("kind", "base_z", "top_z", "top_near_z", "narrow_m", "bearing_rad", "wide_m", "centre_xy", "cut", "border",
            "sigma_m", "points", "part", "hollow_m", "way_in_z", "slot")
    ext = {k: e.get(k) for k in keys}
    e1, e2 = SC.axes_of(up)
    w = e.get("wall")
    ext["wall"] = None if not w else {"point_m": w["point"], "thick_m": w["thick_m"],
                                      "inward": [float(x) for x in e1 * w["inward_xy"][0] + e2 * w["inward_xy"][1]]}
    out["extent"] = ext
    if empty and empty.get("place"):   # a place pixel on bare support: the point itself, and what stands near it
        out["near"] = place_near.near_rows(key, got, up, hand, sigma, empty)
        what = ("two things lie within the look's own error of it, so neither is named" if empty["why"] == "two"
                else "no thing lies within the look's own error of it")
        rows = "; ".join(f"({i}) a thing" + (f" {t['narrow_mm']:.0f} by {t['wide_mm']:.0f} mm"
                                             if t.get("narrow_mm") is not None else "")
                         + f" {t['distance_mm']:.0f} mm from it, on pixel {t['px']}" for i, t in enumerate(out["near"], 1))
        out["doubt"].append(f"your pixel lies on bare support and {what} ({empty['error_m'] * MM:.1f} mm across): it "
                            "is the place itself" + (f"; near it the same look shows {rows}" if rows else ""))
    elif empty:                 # a take has no one thing here: answered where the look shows empty, else acted on
        place_near.answer(ex, rec, key, fixed, got, up, hand, out, empty, sigma, px=(u, v))
    if e["kind"] == "thing":
        if e["cut"]:
            out["doubt"].append("the thing is cut by the picture's edge or by what hides it: its width and top are "
                                "lower bounds")
            out["remove_by"] = out["remove_by"] or _other_camera(ex, rec, fixed)
        if e["border"]:
            out["doubt"].append(f"the thing runs on past {BAND} openings of the hand from the point: wider than this "
                                "hand spans")
        if role == "place" and e.get("way_in_z") is not None:
            entry = SC.ray_at_height(o, d, up, e["way_in_z"])
            if entry is not None:
                out["entry_m"] = [float(x) for x in entry]
                out["doubt"].append(f"the pointed ray comes in over a rim {(e['way_in_z'] - float(p @ up)) * MM:.0f} mm "
                                    "above where it lands: the place is entered at the rim's height, and the floor "
                                    "inside is not seen")
        elif role == "place" and e.get("slot"):
            mid = e1 * e["slot"]["middle_xy"][0] + e2 * e["slot"]["middle_xy"][1] + up * float(p @ up)
            out["entry_m"] = [float(x) for x in mid]
            out["doubt"].append(f"the pointed ray lands on a wall and came in above the wall in front of it: the place "
                                f"is taken as the middle of the {e['slot']['span_m'] * MM:.0f} mm between the two "
                                "(the ray shows no height for that opening)")
    return out


def _pixel(ex, value, role):
    try:
        x, y = (float(c) for c in value.get("px"))
    except (TypeError, ValueError):
        return refusal("spot", "a pixel spot is " + HOW, "take a look and point in it")
    key, rec, fixed = look_of(ex, value.get("look", ""))
    if rec is None:
        return refusal("look", f"no look {str(value.get('look'))[:40]!r} in this session", "look, then point in "
                       "that look: " + HOW)
    if rec.get("cam") is None:
        rec["cam"] = camera_of_model(ex, rec["q"])            # a look taken before this package kept its camera
    view = view_of(rec)
    if not (np.isfinite(x) and np.isfinite(y) and view.inside(x, y)):
        return refusal("pixel", f"pixel ({x:g}, {y:g}) is outside look {key} ({view.w} x {view.h})", "u from 0 to "
                       "width, v from 0 to height, from the top-left")
    u, v = int(np.floor(x)), int(np.floor(y))
    o, d = view.pos, view.ray(u + SC.MID, v + SC.MID)
    notes = []
    got = cloud_of(ex, rec)
    if got is None:
        notes.append(f"look {key} carries no depth picture" if not rec.get("_depth_problem") else
                     f"the depth picture of look {key} cannot be used ({rec['_depth_problem']})")
    elif got[3][v, u]:
        notes.append(f"the pixel shows the robot's own body in look {key}: what lies behind it is not seen there")
    out = _from_depth(ex, rec, key, fixed, u, v, role, o, d)
    if out is None:
        out = _from_other(ex, key, view, u, v, role, o, d)
    if out is None:
        up, h, sig = support_of(ex)
        p = SC.ray_at_height(o, d, up, h)
        if p is None:
            return refusal("pixel", f"the ray of pixel ({x:g}, {y:g}) of look {key} never meets the stated support, "
                           "and no depth picture shows where it lands", "point at a thing or a place the look shows")
        foot = float(np.linalg.norm(p - o)) / view.f
        out = {"point_m": [float(c) for c in p], "sigma_m": {"along_ray": None, "across": SC.MID * foot +
                                                             _moved_m(ex, rec, fixed)},
               "ray": [float(c) for c in d], "source": ONE_VIEW,
               "doubt": ["the height of what stands there is not seen: the point is where the ray meets the support"],
               "remove_by": _other_camera(ex, rec, fixed), "periods": 0, "looks": [key]}
        if role != "place":
            out["doubt"].append("the look cannot show whether a thing stands there (no depth is seen): the hand "
                                "goes to the point with the contact stop armed")
    out["doubt"] = notes + out["doubt"]
    return out


def _from_other(ex, key, view, u, v, role, o, d):
    """Source 2: the newest other look of the session whose depth shows the pointed ray."""
    up, _, _ = support_of(ex)
    looks = [(k, r, True) for k, r in ((ex.state.get("scene_camera") or {}).get("looks") or {}).items()]
    looks += [(k, r, False) for k, r in (getattr(ex, "_looks", None) or {}).items()]
    looks = [x for x in looks if x[0] != key and x[1].get("depth") is not None]
    for k, rec, fixed in sorted(looks, key=lambda x: -float(x[1].get("t") or 0.0)):
        got = cloud_of(ex, rec)
        if got is None:
            continue
        view_r, P, valid, _ = got
        rng = np.linalg.norm(P[valid] - view_r.pos, axis=-1)
        if not rng.size:
            continue
        step = GIVE * float(np.median(rng)) / view_r.f
        hit, px = SC.march(view, u + SC.MID, v + SC.MID, view_r, P, valid, up, step)
        if hit is None:
            continue
        out = _from_depth(ex, rec, k, fixed, px[0], px[1], role, o, d, of_ray=key)
        if out is not None:
            return out
    return None


def spot(ex, value, role="pick"):
    """C11 (module docstring). Returns a spot, or a refusal for a malformed argument."""
    if isinstance(value, dict) and "look" in value:
        return _pixel(ex, value, role)
    if isinstance(value, str) or (isinstance(value, (list, tuple)) and len(value) == XYZ):
        return _typed(ex, value, role)
    return refusal("spot", "a spot is " + HOW + ", a located label or [x, y, z] mm", "look, then point: " + HOW)


def setup(ex):
    _state(ex)
