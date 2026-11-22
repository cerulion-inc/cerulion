# SPDX-License-Identifier: AGPL-3.0-only
"""Derived geometry, stroke bounds and the motion adapter shared by two packages."""

import inspect
import math

import numpy as np

MM = 1000
UP = np.array([0, 0, 1])
_MODERN = {}


def of(ex):
    from rcpx import bounds
    return bounds.of(ex)


def ratios():
    from rcpx import bounds_ratios
    return bounds_ratios


def item(what, value, unit, source, evidence, lift=None, over_mm=None):
    return dict(what=what, value=value, unit=unit, over_mm=over_mm, source=source, evidence=evidence, lift=lift)


def source(B, name):
    return B.src(name) if hasattr(B, "src") else ("model", "from the model through Bounds")


def thing(B, ref):
    return ref if hasattr(ref, "sigma_p") else B.thing(ref)


def band(B, ref):
    return ratios().BAND * float(getattr(thing(B, ref), "sigma_p", 0) or 0)


def support(B, point, floor=None):
    """Support at a point; the stated level is used only while the space map has none."""
    known = B.space.support()
    if known is None:
        if floor is None:
            raise ValueError("no support plane or stated floor height is known")
        normal, offset, sigma, src = UP, float(floor), 0, "stated"
    else:
        (normal, offset), sigma, src = known
        normal = np.asarray(normal, float)
    p = np.asarray(point, float).copy()
    if normal[-1] == 0:
        raise ValueError("the support has no height along up")
    p[-1] = (offset - normal[:2] @ p[:2]) / normal[-1]
    E = B.thing(p)
    extent = {"plane": (normal, offset), "source": src}
    if hasattr(E, "_replace"):
        return E._replace(sigma_p=float(sigma or 0), extent=extent)
    E.sigma_p, E.extent = float(sigma or 0), extent
    return E



def grip(B, held):
    known = dict(held or {})
    for key, empty in (("proofs", []), ("squeeze_n", 0)):
        if known.get(key) is None:
            known[key] = empty
    r = B.grip(known)
    if isinstance(r, dict):
        return r["capacity_n"], r["accel_ok"], r["items"]
    return r


def held_of(place, g):
    p = place or {}
    weight = p.get("weight_n")
    return {"mass_kg": None if weight is None or g == 0 else float(weight) / g,
            "weight_n": weight, "extent_m": p.get("extent_m"), "sigma_m": p.get("sigma_m"),
            "squeeze_n": p.get("squeeze_n"), "proofs": p.get("proofs"), "accel_ok": p.get("accel_ok")}


def push_height(height_m, overhang_m, clear_m, band_top_m, low_m):
    h = max(low_m, height_m + clear_m - overhang_m)
    note = "raised so the hand's overhang clears the top" if h > low_m else None
    return h, note, h > height_m - band_top_m


def reaches_base(point_xy, d_xy, distance_m, keep_m):
    p, d = np.asarray(point_xy, float), np.asarray(d_xy, float)
    dd = float(d @ d)
    t = max(0, min(distance_m, -float(p @ d) / dd)) if dd else 0
    return bool(np.linalg.norm(p + d * t) <= keep_m)


def base_distance(point_xy, d_xy, keep_m):
    p, d = np.asarray(point_xy, float), np.asarray(d_xy, float)
    dd, pd = float(d @ d), float(p @ d)
    disc = pd ** 2 - dd * (float(p @ p) - keep_m ** 2)
    if dd == 0 or disc < 0:
        return None
    return max(0, (-pd - math.sqrt(disc)) / dd)


def push_speed(B, ref, q, d, f_now, travel_stated_m, intent=None):
    E = thing(B, ref)
    fs = B.force("follow", E, q, d, intent)
    Z, _ = B.slope(q, d, E)
    v = max(0, fs.ceiling_n - f_now) / Z if Z is not None and Z > 0 else B.v_in(E, q, d, intent)[0]
    return min(B.v_free(q, d), v)


def stroke(B, ref, q, d, f_now, travel_m, intent=None):
    E = thing(B, ref)
    fs = B.force("follow", E, q, d, intent)
    v = push_speed(B, E, q, d, f_now, travel_m, intent)
    guard = B.guard("follow", v)
    meet = B.f_thr(v, 0, readings=guard.readings)
    settle = math.ceil(B.t_settle() / B.dt())
    rigid = B.rigid(q, d, meet)
    return dict(speed=v, force=fs, meet_n=meet, readings=guard.readings, lost_periods=settle,
                stall_count=settle, stall_m=max(ratios().QUANT * rigid,
                                                ratios().REL * v * B.t_settle()),
                give=ratios().GIVE, steady_min=1 - ratios().REL, moved_m=rigid)


def geometry(B, ref, support_ref, q, d, v, point_height_m):
    E, S = thing(B, ref), thing(B, support_ref)
    return dict(band_m=max(band(B, E), point_height_m + band(B, E)), ahead_m=B.rigid(q, d),
                profile_near_m=band(B, E), above_m=B.standoff(B.v_free(q, -UP), E, q, -UP),
                feel_m=B.standoff(0, E, q, d), past_m=B.travel("touch", E, q, -UP).past_m,
                search_m=B.travel("touch", S, q, -UP).search_m,
                on_top_m=ratios().BAND * math.hypot(E.sigma_p or 0, S.sigma_p or 0),
                face_m=band(B, E), palm_m=B.clearance(v, E, body=None),
                low_m=B.clearance(v, S, body=None), low_plan_m=B.clearance(0, S, body=None),
                backoff_m=B.travel("follow", E, q, d).backoff_m)


def next_stroke(B, pushed_m, off_m, yaw_rad, extent_m, absorb_m):
    if pushed_m <= 0:
        return None
    denominator = max(off_m + abs(yaw_rad) * extent_m, B.rigid(None, None))
    if denominator == 0:
        denominator = ratios().REL * pushed_m
    return absorb_m * pushed_m / denominator


def absorb(B, ref, width_m):
    return band(B, ref) + width_m / 2


def same_spot(B, a, b, q=None, d=None):
    return (thing(B, a).sigma_p or 0) + (thing(B, b).sigma_p or 0) + B.rigid(q, d)


def axis_min(B, a, b):
    return ratios().BAND * ((thing(B, a).sigma_p or 0) + (thing(B, b).sigma_p or 0))


def support_match(B, a, b, reach_m):
    sa, sb = thing(B, a).sigma_p or 0, thing(B, b).sigma_p or 0
    return ratios().BAND * math.hypot(sa, sb), reach_m + sa + sb


def turning(B, S, q, reach_m):
    return dict(clear_m=B.clearance(B.v_free(q, UP), S, body=None),
                carry_m=B.standoff(B.v_free(q, -UP), S, q, -UP) + reach_m,
                search_m=B.travel("lower", S, q, -UP).search_m,
                margin_m=B.clearance(0, S, body=None))


def lower(B, S, q, held, intent=None):
    v = B.v_in(S, q, -UP, intent, (held or {}).get("weight_n") or 0)[0]
    gs = B.guard("lower", v, held=held)
    return v, gs.force_n, gs.readings


def modern(ex):
    cls = type(ex)
    if cls not in _MODERN:
        _MODERN[cls] = "kind" in inspect.signature(cls._cart_move).parameters
    return _MODERN[cls]


def counts(r, elapsed_s=None, dt=None):
    n = r.get("periods")
    if isinstance(n, (list, tuple)):
        n = sum(n)
    if n is None:
        n = max(0, len(r["done"]) - 1) if "done" in r else 0
        if elapsed_s is not None and dt:
            n = round(elapsed_s / dt)
    moving, waiting = r.get("moving", n), r.get("waiting", 0)
    return int(n), int(moving), int(waiting)


def old_result(B, r, v, travel_m, intent, t0, ex, binding=None):
    if "refused" in r:
        return r
    n = max(0, len(r["done"]) - 1) if "done" in r else 0
    r["periods"] = n
    # The old joint helper discards its path. Its required periods field is zero;
    # measured elapsed periods still account for its phase's motion and waiting.
    elapsed = round((ex.sim_time() - t0) / B.dt())
    src, evidence = source(B, "v_free")
    if (intent or {}).get("clear") is not None:
        src, evidence = "word", "clear by the operator's word after its look"
    r.setdefault("moving", n if "done" in r else elapsed)
    r.setdefault("waiting", max(0, elapsed - r["moving"]))
    if binding is None:
        binding = item("speed", v, "m/s", src, evidence)
    else:
        binding = dict(binding)
    binding["over_mm"] = [0, travel_m * MM]
    return B.report(r, [binding])


def entry(B, s, dp, target):
    E = thing(B, target)
    b = band(B, E)
    length = float(np.linalg.norm(dp))
    if not length:
        return 0
    if isinstance(E.extent, dict) and "plane" in E.extent:
        normal, offset = E.extent["plane"]
        c = float(normal @ s.tip - offset)
        toward = -float(normal @ dp)
        return max(0, min(1, (c - b) / toward)) if toward > 0 else 1
    if E.point is None:
        return 0
    a = s.tip - np.asarray(E.point, float)
    if np.linalg.norm(a) <= b:
        return 0
    u = dp / length
    along = float(a @ u)
    disc = along ** 2 - float(a @ a) + b ** 2
    if disc < 0 or along >= 0:
        return 1
    return max(0, min(1, (-along - math.sqrt(disc)) / length))


def _old_cart(ex, dp, drot, kind, target, intent, away_from, held, inside=False):
    B, s = of(ex), sense(ex)
    u = dp / np.linalg.norm(dp) if np.linalg.norm(dp) else UP
    v = B.v_free(s.q, u)
    binding = None
    if inside:
        v, binding = B.v_in(thing(B, target), s.q, u, intent, (held or {}).get("weight_n") or 0)
    if (intent or {}).get("speed") is not None:
        if intent["speed"] < v:
            v = intent["speed"]
            binding = item("speed", v, "m/s", "intent", "your stated speed")
    gs = B.guard(kind, v, held=held, away_from=away_from)
    t0 = ex.sim_time()
    r = ex._cart_move(s, dp, drot, v, True, True, gs.force_n, limit=None)
    return old_result(B, r, v, float(np.linalg.norm(dp)), intent, t0, ex, binding)


def line(ex, dp, drot=None, *, kind="transit", target=None, intent=None, away_from=None, held=None):
    dp = np.asarray(dp, float)
    if modern(ex):
        return ex._cart_move(sense(ex), dp, drot, None, True, True, kind=kind, target=target,
                             intent=intent, away_from=away_from, held=held)
    B, s = of(ex), sense(ex)
    near = kind in ("approach", "touch", "press", "lower", "close")
    split = entry(B, s, dp, target) if near else 1
    if 0 < split < 1:
        goal = s.tip.copy() + dp
        r = _old_cart(ex, dp * split, None if drot is None else np.asarray(drot) * split,
                      kind, target, intent, away_from, held)
        if "refused" in r or r.get("stop", r.get("stopped")) != "goal":
            return r
        tail = _old_cart(ex, goal - sense(ex).tip,
                         None if drot is None else np.asarray(drot) * (1 - split),
                         kind, target, intent, away_from, held, inside=True)
        n, m, w = counts(r)
        tn, tm, tw = counts(tail)
        tail.update(periods=n + tn, moving=m + tm, waiting=w + tw)
        return tail
    return _old_cart(ex, dp, drot, kind, target, intent, away_from, held, inside=near and split == 0)


def joints(ex, q, *, kind="transit", target=None, intent=None, away_from=None, held=None,
           dark_ok=False, record_contact=True):
    if modern(ex):
        return ex._joint_move(q, None, kind=kind, target=target, intent=intent, away_from=away_from,
                              held=held, dark_ok=dark_ok, record_contact=record_contact)
    B, s, t0 = of(ex), sense(ex), ex.sim_time()
    r = ex._joint_move(q, 1, guarded=True, dark_ok=dark_ok, record_contact=record_contact, limit=None)
    return old_result(B, r, B.v_free(s.q, UP), float(np.linalg.norm(ex.model.fk_tip(q)[0] - s.tip)), intent, t0, ex)


def stream(ex, legs, *, held=None, looks=None, need_m=None):
    if modern(ex):
        return ex._stream(legs, held=held, looks=looks, need_m=need_m)
    periods, moving, waiting, reports = [], 0, 0, []
    out = dict(stopped="goal", leg=None, periods=periods, moving=moving, waiting=waiting, looks=[], bounds={})
    for i, leg in enumerate(legs):
        kw = {k: leg.get(k) for k in ("kind", "target", "intent", "away_from")}
        to = np.asarray(leg["to"], float)
        if to.ndim == 2:
            from rcpx.control_plan import vec_from_rot
            s = sense(ex)
            r = line(ex, to[:3, 3] - s.tip, vec_from_rot(to[:3, :3] @ s.R.T), held=held, **kw)
        else:
            r = joints(ex, to, held=held, **kw)
        n, m, w = counts(r)
        periods.append(n)
        moving, waiting = moving + m, waiting + w
        binding = (r.get("bounds") or {}).get("binding")
        if binding:
            reports.append(binding)
        out.update(stopped=r.get("stop", r.get("stopped", "refused")), leg=i, moving=moving, waiting=waiting)
        if "refused" in r:
            out.update(refused=r["refused"], said=r.get("said"))
        if "refused" in r or out["stopped"] != "goal":
            break
    return of(ex).report(out, reports)


class Phases:
    """Per phase accounting using motion and jaw results, with elapsed time on the old core."""

    def __init__(self, B):
        self.B, self.rows, self.row = B, [], None

    def begin(self, name, did):
        self.B.begin(name)
        self.row = dict(phase=name, did=did, periods=0, moving=0, waiting=0, jaw=0, seconds=0,
                        free_share=None, bounds=None)
        self.rows.append(self.row)

    def add(self, result, elapsed=None, jaw=False):
        r = result or {}
        n, m, w = counts(r, elapsed, self.B.dt())
        row = self.row
        if jaw:
            row["jaw"] += n
        else:
            row["moving"] += m
            row["waiting"] += w
        row["periods"] = row["moving"] + row["waiting"] + row["jaw"]
        row["seconds"] = row["periods"] * self.B.dt()
        bound = (r.get("bounds") or {}).get("binding")
        if bound:
            row["bounds"] = bound
        row["free_share"] = (r.get("bounds") or {}).get("free_share", r.get("free_share"))

    def result(self, r, items):
        r["phases"] = self.rows
        r["periods"] = sum(row["periods"] for row in self.rows)
        all_items = [it for it in (items or []) if it.get("value") is not None]
        all_items += [row["bounds"] for row in self.rows if row["bounds"]]
        return self.B.report(r, all_items)


def sense(ex):
    return ex._sense()


def jaw_command(ex):
    return ex._gripper_cmd


def refuse(check, why, hint):
    from rcpx.context import refusal
    return {"said": "Did not move: " + why + ".", **refusal(check, why, hint)}


def reach(ends, hang_m, height_m, jaw_max_m, finger_m, width_m):
    if ends is not None:
        size = max(float(np.linalg.norm(p)) for p in ends)
    elif hang_m is not None or height_m is not None:
        size = float(hang_m or 0) + float(height_m or 0)
    else:
        size = jaw_max_m + (finger_m or 0)
    return size + (width_m or 0) / 2


def keep_base(B, E, mount_m, standoff_m):
    return (mount_m or 0) + B.clearance(0, E) + standoff_m


def free_travel(B, E, q, d, point, front):
    return max(0, float((np.asarray(point) - front) @ d)) + B.travel("follow", E, q, d).past_m


def via(B, p0, p1, radius, q, E, held):
    route = B.space.route(p0, p1, radius, held)
    if route is not None:
        return route, None
    height = B.space.ceiling(p0[:2], p1[:2], radius)
    note = None
    if height is None:
        height = max(p0[2], p1[2]) + B.clearance(B.v_free(q, UP), E, body=None)
        note = "the space map gave no route; via height uses tip heights and Bounds clearance"
    return [np.array([p0[0], p0[1], height]), np.array([p1[0], p1[1], height]), p1], note


def whole(fn):
    """A call owns its word only until it returns, including an early refusal."""
    from functools import wraps
    @wraps(fn)
    def run(ex, *args, **kwargs):
        B = of(ex)
        B.begin("call")
        try:
            return fn(ex, *args, **kwargs)
        finally:
            B.space.end_call()
    return run
