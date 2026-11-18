# SPDX-License-Identifier: AGPL-3.0-only
"""What the approach package reads before it moves: the target and direction the operator named, the reach
verdict for a named tool surface, the point_source rule, and the robot://state document.

Targets come from the call or from robot://world (an entry id or label with a point), directions from the call,
from a world entry's measured surface normal, or from a device part in robot://devices that carries a base-frame
direction. Reach is solved on the robot's own kinematic model for the named surface, with the plane and self checks
the motion's path check uses. Nothing here reads simulator ground truth or names a robot.
"""

import math

import numpy as np

from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx.context import refusal
from rcpx.bounds_laws import TWO
from rcpx.r2.approach_kin import POS_TOL_M, SurfaceKin, rpy_deg, surface_names, unit
from rcpx.surfaces import DIRECTION_NAMES

MM = 1000.0
SOURCES = ("touch", "triangulated", "single_look")
UNKNOWN_VERDICT = "unknown: localise by touch or two views first"
POSITIONAL = ("contact", "located", "device")
START_SEED = 21           # the seeded extra starts of every solve (deterministic)
N_RANDOM_STARTS = 4
DIR_NAMES = {(1, 0, 0): "forward", (-1, 0, 0): "back", (0, 1, 0): "left", (0, -1, 0): "right", (0, 0, 1): "up",
             (0, 0, -1): "down"}


def _r(v, nd=2):
    """Round a vector for the reader."""
    return [round(float(x), nd) for x in v]


def _mm(p):
    """Report a position in millimetres rounded to the readers precision."""
    return [round(float(x) * MM, 2) for x in p]


def dir_name(v):
    """Name a nearly cardinal base direction for a reach verdict."""
    for k, name in DIR_NAMES.items():
        if float(np.dot(v, k)) > 0.95:
            return name
    return None


# ---------------------------------------------------------------------------------------------- world entries
def find_entry(world, key):
    """A positional world entry by id, else the newest by label (a fresh one before a stale one)."""
    e = world.get(str(key))
    if e is not None:
        return e
    found = [x for x in world.query(label=str(key)) if x["kind"] in POSITIONAL and x["point_mm"] is not None]
    fresh = [x for x in found if not x["stale"]]
    return (fresh or found or [None])[-1]


def entry_normal(e):
    """The outward surface normal stored with a world entry (a touch's normal_est), or None."""
    data = (e or {}).get("data") or {}
    for key in ("normal", "normal_est"):
        v = data.get(key)
        if isinstance(v, (list, tuple)) and len(v) == 3:
            u = unit(v)
            if u is not None and np.all(np.isfinite(u)):
                return u
    return None


def source_of(e):
    """touch, triangulated, or the entry's own source text (anything else is not a fixed point_source)."""
    if e["kind"] == "contact":
        return "touch"
    if e.get("source") == "triangulate":
        return "triangulated"
    return str(e.get("source") or "unknown")


def _known_labels(world):
    """List positional labels whose entries can resolve a target."""
    out = []
    for e in world.query():
        if e["kind"] in POSITIONAL and e["point_mm"] is not None:
            out.append(f"{e['id']} ({e['label']})")
    return out[-8:]


def _vec3(x):
    """Read three finite coordinate values without restricting their magnitude."""
    try:
        v = np.asarray(x, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    return v if v.size == 3 and np.all(np.isfinite(v)) else None


def resolve_target(world, target):
    """(point m, info, None) or (None, None, refusal)."""
    if isinstance(target, str):
        e = find_entry(world, target)
        if e is None:
            known = _known_labels(world)
            return None, None, refusal("target", f"no world entry with a point is named {target!r}",
                                       "use an id or label from robot://world" +
                                       (f" ({', '.join(known)})" if known else ", or give [x, y, z] in base mm"))
        if _vec3(e.get("point_mm")) is None:
            known = _known_labels(world)
            return None, None, refusal("target", f"world entry {e['id']} ({e['kind']} {e['label']!r}) has no point "
                                       "to go to", "use a touched contact or a triangulated point" +
                                       (f" ({', '.join(known)})" if known else ", or give [x, y, z] in base mm"))
        p = np.asarray(e["point_mm"], dtype=float) / MM
        info = {"id": e["id"], "label": e["label"], "kind": e["kind"], "point_mm": _r(e["point_mm"]),
                "point_source": source_of(e), "stale": bool(e["stale"])}
        return p, {**info, "_normal": entry_normal(e)}, None
    v = None if isinstance(target, (bool, dict)) else _vec3(target)
    if v is None:
        return None, None, refusal("target", f"target must be a world label or id, or three finite numbers; got "
                                   f"{str(target)[:60]}", "give [x, y, z] in base frame mm, or a label from "
                                                          "robot://world")
    return v / MM, {"point_mm": _r(v), "point_source": "given", "_normal": None}, None


def _devices_doc(ex):
    """Read the operator-visible device sheet from the executor context."""
    doc = (getattr(ex, "context", None) or {}).get("devices")
    if callable(doc):
        doc = doc()
    if isinstance(doc, dict):
        return doc.get("devices") or []
    return list(doc or [])


def _resolve_affordance(ex, text):
    """Use a device parts base direction; refuse directions given only in words."""
    dev, _, part = text.partition("/")
    devices = _devices_doc(ex)
    if not devices:
        return None, None, refusal("along", "no device knowledge in this session, so an affordance names no "
                                   "direction", "give along as [x, y, z] in base frame, a direction name, or "
                                                "normal:<id> after a touch")
    d = next((x for x in devices if str(x.get("id", "")).lower() == dev.lower()
              or str(x.get("name", "")).lower() == dev.lower()), None)
    if d is None:
        return None, None, refusal("along", f"no device {dev!r} in robot://devices",
                                   "devices: " + ", ".join(str(x.get("id")) for x in devices))
    parts = d.get("parts") or []
    pt = next((x for x in parts if str(x.get("part", x.get("name", ""))).lower() == part.lower()), None)
    if pt is None:
        return None, None, refusal("along", f"device {dev} has no part {part!r}",
                                   "parts: " + ", ".join(str(x.get("part", x.get("name"))) for x in parts))
    v = _vec3(pt.get("along_base"))
    if v is None or unit(v) is None:
        words = pt.get("direction") or pt.get("moves") or "not given"
        allowed = pt.get("approach_allowed")
        return None, None, refusal(
            "along", f"{dev}/{part} gives its direction only in words ({words}"
                     + (f"; approach: {allowed}" if allowed else "") + "), not as a base frame direction",
            "turn it into one: touch the part's face and use along normal:<that contact id>, or give [x, y, z]")
    return unit(v), {"along_source": f"affordance:{dev}/{part}", "from": pt.get("source")}, None


def resolve_along(ex, along, tinfo):
    """(unit direction in B, info, None) or (None, None, refusal). along is the way the surface travels onto the
    target: [x, y, z], a direction name, normal:<id or label> (into that surface) or affordance:<device>/<part>."""
    if along is None:
        return None, None, refusal("along", "along is needed: the direction the surface travels onto the target",
                                   "give [x, y, z] in base frame, a name such as down, or normal:<contact id>")
    if isinstance(along, str):
        text = along.strip()
        if text in DIRECTION_NAMES:
            return np.asarray(DIRECTION_NAMES[text], dtype=float), {"along_source": text}, None
        if text.startswith("normal:"):
            key = text[len("normal:"):].strip()
            e = find_entry(ex.world, key) if key else None
            if e is None:
                return None, None, refusal("along", f"no world entry {key!r} to take a normal from",
                                           "touch the surface first; use the id the touch returned")
            n = entry_normal(e)
            if n is None:
                return None, None, refusal("along", f"world entry {e['id']} ({e['label']}) has no measured surface "
                                           "normal", "touch the surface (touch returns normal_est), then use "
                                                     "normal:<that contact id>")
            return -n, {"along_source": f"normal:{e['id']}", "_normal": n}, None
        if text.startswith("affordance:"):
            return _resolve_affordance(ex, text[len("affordance:"):].strip())
        return None, None, refusal("along", f"unknown along {text[:40]!r}",
                                   "use [x, y, z], one of " + ", ".join(DIRECTION_NAMES) +
                                   ", normal:<id> or affordance:<device>/<part>")
    v = None if isinstance(along, (bool, dict)) else _vec3(along)
    if v is None or unit(v) is None:
        return None, None, refusal("along", "along must be a non-zero [x, y, z] in base frame or a name",
                                   "for example [0, 0, -1] (down) or normal:<contact id>")
    return unit(v), {"along_source": "given"}, None


def surface_kin(ex, name):
    """(SurfaceKin, None) or (None, refusal); cached per surface on the executor."""
    cache = ex.state.setdefault("approach", {}).setdefault("kin", {})
    key = str(name)
    if key not in cache:
        try:
            cache[key] = SurfaceKin(ex.model, key)
        except KeyError:
            return None, refusal("surface", f"this tool has no surface named {key[:40]!r}",
                                 "surfaces (robot://body tool.surfaces): " + ", ".join(surface_names(ex.model.spec)))
    return cache[key], None


# ---------------------------------------------------------------------------------------------- pose checks
def _body_fit(ex, B, name):
    """The margin a body's capsule owes its own shape (m): Bounds.clearance at rest with no thing, which is the
    farthest the body's hull stands out of its capsule (SPEC-R4 Q4). A fact of the model: kept per executor."""
    cache = ex.state.setdefault("approach", {}).setdefault("fit", {})
    if name not in cache:
        cache[name] = float(B.clearance(0.0, None, name))
    return cache[name]


def _body_low(ex, B, name, q, low, fit, band, plain):
    """How far a body stands above the surface under it (m), by SPEC-R4 Q4: on its capsule (low, from the
    executor's own geometry) while the capsule is farther than its fit error plus the band in force; nearer than
    that on the body's own hull against the support plane (Bounds.distance), so a link is measured as the link it
    is. The hull is used only where the model states one (Model.hulls, SPEC-R4 A8.4) and the surface under the
    body is the plain support (plain: not over a mount, and a support is stated); elsewhere the capsule is all the
    model states of the body and stands as measured."""
    if low >= fit + band or not plain or not callable(getattr(ex.model, "hulls", None)):
        return low
    return min(low, float(B.distance(q, "support", body=name)))


def pose_ok(ex, q, q_ref, stroke):
    """(ok, why): the pose keeps the band above the surface under the arm (the executor's _floor_at and
    _cap_clearance where it has them, else the mounting plane z = 0), judged against q_ref as the path check judges
    against the start (a part already low may stay as low). No margin is typed: a link or the tool must stay
    BAND x sigma_z of the support clear, measured on its capsule while that is far and on its own hull when near
    (_body_low); the links must not come newly within Bounds.clearance at rest of each other. stroke: the surface
    is on the target, so the tool is exempt and the tip may sit down to the search depth below the support
    (BAND x sigma_z, Bounds.travel)."""
    m, B = ex.model, bounds.of(ex)
    support = B.space.support()
    sigma_z = (support[1] or 0.0) if support is not None else 0.0
    band = R.BAND * sigma_z
    clear = getattr(ex, "_cap_clearance", None) or (lambda p0, p1, r: min(p0[2], p1[2]) - r)
    floor_at = getattr(ex, "_floor_at", None) or (lambda p: 0.0)

    def plain(p0, p1):
        """Both ends stand over the support plane itself (not over a mount)."""
        if support is None:
            return False
        normal, offset = support[0]
        normal = np.asarray(normal, dtype=float)
        if float(normal[-1]) == 0:
            return False                  # an upright plane has no height under a point
        return all(abs(float(floor_at(p)) - (float(offset) - float(normal[:-1] @ p[:-1])) / float(normal[-1]))
                   <= POS_TOL_M for p in (p0, p1))

    low0 = {c[0]: clear(c[1], c[2], c[3]) for c in m.collision_capsules(q_ref)[1:]}
    t0 = m.fk_tip(q_ref)[0]
    tip0 = float(t0[2] - floor_at(t0))
    tool_floor = -B.travel("touch", None, q, [0, 0, -1]).search_m if stroke else band
    for name, p0, p1, r in m.collision_capsules(q)[1:]:
        tool = name in ex._tool_bodies
        if tool and stroke:
            continue
        low = clear(p0, p1, r)
        if low >= low0.get(name, low) - 1e-4:
            continue                      # no nearer than it starts
        low = _body_low(ex, B, name, q, low, _body_fit(ex, B, name), band, plain(p0, p1))
        if low < band:
            return False, f"the {name} link would come within {max(low, 0.0) * MM:.0f} mm of the surface under it"
    tp = m.fk_tip(q)[0]
    tz = float(tp[2] - floor_at(tp))
    if tz < tool_floor and tz < tip0 - 1e-4:
        return False, "the tip would pass the surface under the arm"
    self_margin = B.clearance(0.0)
    new = set(m.self_collisions(q, self_margin)) - set(m.self_collisions(q_ref, self_margin))
    if new:
        a, b = sorted(new)[0]
        return False, f"the {a} and {b} links would meet"
    return True, ""


def starts(ex, q_now, extra=()):
    """Current, extra, home and deterministically seeded starts for the geometry solver, PA-026."""
    rng = np.random.default_rng(START_SEED)
    out = [np.asarray(q_now, dtype=float)] + [np.asarray(x, dtype=float) for x in extra]
    out.append(ex.model.q_home.copy())
    for _ in range(N_RANDOM_STARTS):
        out.append(rng.uniform(ex.model.q_lo, ex.model.q_hi))
    return out


def reach_past_m(B, thing, q, a):
    """One Bounds travel law for both the checked stroke and suggested touch, SPEC-R4 A4.3."""
    return B.travel("touch", thing, q, a).past_m


def reach_tol_m(ex, q, along=None, tol_m=None):
    """A solve cannot state error below its convergence tolerance, PA-015/PA-027, SPEC-R4 Q3."""
    return max(bounds.of(ex).rigid(q, [0, 0, -1] if along is None else along), tol_m or 0.0, POS_TOL_M)


def surface_reach(ex, kin, p, q_now, along=None, margin=True, tol_m=None):
    """(verdict dict, joint pose or None) for the named surface at point p (m, B). The verdict follows reach_check:
    reachable, margin_mm (further along the stroke the surface could still go), short_by_mm, direction, why, the
    force the surface can push with there, and with along given how squarely the surface can face it there."""
    m = ex.model
    B = bounds.of(ex)
    tol = reach_tol_m(ex, q_now, along, tol_m)
    best, collided, sol = None, None, None
    for q0 in starts(ex, q_now):
        q, err, _ = kin.solve(q0, p)
        if err <= tol:
            ok, why = pose_ok(ex, q, q_now, stroke=True)
            if ok:
                sol = q
                break
            collided = collided or why
        if best is None or err < best[1]:
            best = (q, err)
    out = {"surface": kin.name, "reach_tol_mm": round(tol * MM, 2)}
    if sol is None:
        if collided:
            return {**out, "reachable": False, "margin_mm": 0.0, "short_by_mm": 0.0, "direction": None,
                    "why": f"the arm poses that put the {kin.name} there fail a check: {collided}"}, None
        q, err = best
        p_best = kin.pose(q)[0]
        gap = p - p_best
        dist = float(np.linalg.norm(gap))
        u = gap / dist if dist > 1e-9 else np.zeros(3)
        name = dir_name(u)
        return {**out, "reachable": False, "margin_mm": 0.0, "short_by_mm": round(dist * MM, 2),
                "direction": _r(u, 3), "direction_name": name, "closest_mm": _mm(p_best),
                "why": f"out of reach by {dist * MM:.0f} mm" + (f" ({name})" if name else "")}, None
    q = sol
    if along is not None:
        qa, err_a, ang = kin.solve(q, p, along)
        if err_a <= tol and pose_ok(ex, qa, q_now, stroke=True)[0]:
            q = qa
            out["along_err_deg"] = round(math.degrees(ang), 2)
        else:
            out["along_err_deg"] = round(math.degrees(math.acos(float(np.clip(kin.pose(q)[1] @ along, -1, 1)))), 2)
    out_dir = along if along is not None else np.array([p[0], p[1], 0.0]) / max(float(np.hypot(p[0], p[1])), 1e-9)
    lo = 0.0
    if margin:
        hi, q_b = m.length_scale, q
        for _ in range(7):
            mid = (lo + hi) / TWO
            q2, e2, _ = kin.solve(q_b, p + out_dir * mid, None, iters=60)
            if e2 <= tol and pose_ok(ex, q2, q_now, stroke=True)[0]:
                lo, q_b = mid, q2
            else:
                hi = mid
    force_dir = along if along is not None else np.array([0.0, 0.0, -1.0])
    out.update({"reachable": True, "margin_mm": round(lo * MM, 2), "margin_along": _r(out_dir, 3),
                "short_by_mm": 0.0, "direction": None,
                "force_n": round(kin.force_capability(q, force_dir) * (1 - B.e_tau()), 2),
                "force_tool_n": None if B.f_tool() is None else round(B.f_tool(), 2),   # None: no rating is stated (r6 C1)
                "force_along": _r(force_dir, 3),
                "q_deg": {n: round(math.degrees(float(v)), 3) for n, v in zip(m.joint_names, q)}})
    return out, q


def apply_point_source(result, source):
    """reach_check's verdict read with where the point came from: a single look fixes no depth, so a point it gave
    that comes out unreachable is unknown, not out of reach."""
    if not isinstance(result, dict) or "refused" in result or "point_source" in result or source is None:
        return None
    out = dict(result, point_source=source)
    if source == "single_look":
        if result.get("reachable") is False:
            out["reachable"] = None
            out["verdict"] = UNKNOWN_VERDICT
            out["if_the_point_is_right"] = {k: result.get(k) for k in ("short_by_mm", "direction", "why")}
            out["why"] = UNKNOWN_VERDICT + " (one look fixes no depth)"
        else:
            out["verdict"] = "reachable if the point is right; one look fixes no depth, so touch or triangulate it " \
                             "before a stroke"
    else:
        out["verdict"] = "reachable" if result.get("reachable") else str(result.get("why") or "not reachable")
    return out


# ---------------------------------------------------------------------------------------------- robot://state
def _gripper_mm(ex, grip):
    """Read tip jaw width for the state document without motion."""
    import mujoco

    g = ex.model.spec.get("gripper")
    if not g or not isinstance(grip, dict) or grip.get("q") is None:
        return None
    m = ex.model.mj
    if not all(m.jnt_type[m.joint(j).id] == mujoco.mjtJoint.mjJNT_SLIDE for j in g["joints"]):
        return None            # a hinged jaw: its opening is an angle, given as open_frac only
    q = np.asarray(grip["q"], dtype=float).reshape(-1)
    return round(float(np.sum(np.abs(q - np.asarray(g["closed"], dtype=float)))) * MM, 2)


def state_doc(ex):
    """robot://state: where the robot is now, from the joint readings through the body model."""
    r = ex.backend.read()
    m = ex.model
    q = np.asarray(r["q"], dtype=float)
    p, R = m.fk_tip(q)
    t = float(r["t"])
    load = r.get("load")
    grip = r.get("gripper")
    last = None
    looks = getattr(ex, "_looks", None) or {}
    if looks:
        lid = list(looks)[-1]
        rec = looks[lid]
        last = {"id": lid, "can_see": rec.get("can_see"), "age_s": round(max(0.0, t - float(rec.get("t", t))), 3)}
    return {
        "q_deg": {n: round(math.degrees(float(v)), 3) for n, v in zip(m.joint_names, q)},
        "tip": {"pos_mm": _mm(p), "rpy_deg": rpy_deg(R), "x_axis": _r(R[:, 0], 4), "z_axis": _r(R[:, 2], 4)},
        "loads": None if load is None else {n: round(float(x), 3) for n, x in zip(m.joint_names, load)},
        "gripper_mm": _gripper_mm(ex, grip),
        "gripper_open_frac": None if not isinstance(grip, dict) or grip.get("open_frac") is None
        else round(float(grip["open_frac"]), 3),
        "pressing": bool(getattr(ex, "_pressing", False)),
        "epoch": int(getattr(ex, "epoch", 0)),
        "last_look": last,
        "t_s": round(t, 3),
        "conventions": {"frame": "base: x forward, y left, z up, mm",
                        "rpy_deg": "[roll, pitch, yaw] of the tool frame, R = Rz(yaw) Ry(pitch) Rx(roll); with the "
                                   "finger straight up or down (pitch +-90) roll is 0 and yaw carries the turn",
                        "x_axis": "the finger's pointing direction in base",
                        "loads": "signed share of each joint's torque limit in use now, gravity hold included",
                        "source": "measured joint angles through the body model"},
    }
