# SPDX-License-Identifier: AGPL-3.0-only
"""Polyline timing with exact constant-acceleration phases, A4.2 and A8.1.

A stretch runs at the body's own speed. It is slowed only by a site speed or
zone, the operator's speed or accel, a proven grip's acceleration, and B.arrival for a stated limit: the bound_n of
an expected contact, or a limit stated on free space where a contact can begin (rcpx/bounds_free.py).
Unproven space, a thing seen and never touched, a band of position doubt and an unresolved stop slope slow
nothing; what the motion could strike is priced on the plan (plan.strike) and never lowers a speed.
"""

import math
import numpy as np
from rcpx import bounds_free as free
from rcpx import bounds_laws as laws
from rcpx import bounds_limit as limit
from rcpx import bounds_report as report
from rcpx.bounds_book import cloud_clearance, point_clearance
from rcpx.context_base import refusal
from rcpx.bounds_ratios import GIVE, REL
from rcpx.bounds_robot import Point


def _refused(why, hint, check="plan"):
    from rcpx.bounds import Plan
    return Plan(refused=refusal(check, why, hint))


def _unit(dp):
    norm = float(np.linalg.norm(dp))
    return dp / norm if norm > 0 else np.asarray((0, 0, -1), dtype=float)


def _joint_min(dq, ratings):
    return max((abs(d) / v if v > 0 else math.inf for d, v in zip(dq, ratings) if d != 0), default=0.0)


def _accel(dq, L, qdd):
    return min((a * L / abs(d) for d, a in zip(dq, qdd) if d != 0), default=math.inf)


def _moving_points(B, q0, q1, held):
    """The moving points at both ends of a step: a body's capsule ends while the capsule is farther from every
    thing than its own fit error plus the band in force; nearer than that, the body's own hull, as the hull point
    nearest the thing at each end (a thing of unknown sigma: when the book says the capsule is inside its band)."""
    starts, ends = B.robot.points(q0), B.robot.points(q1)
    fits, replace, asked = B.robot.e_fit(), {}, {}
    for a, b in zip(starts, ends):
        if a.body in ("tip", "held") or a.body in replace:
            continue
        asked[a.name] = B.space.kind(a.p, b.p, max(a.radius, b.radius))
        space_kind, ref, c, _ = asked[a.name]
        if ref is None:
            continue
        E = B.thing(ref)
        near = space_kind == "expected" if E.sigma_p is None else c <= fits.get(a.body, 0) + laws.band(E.sigma_p)
        if near:
            h0, h1 = B.robot.hull_points(q0, a.body), B.robot.hull_points(q1, a.body)
            if h0 is not None and h1 is not None and len(h0) == len(h1) and len(h0):
                picks = sorted({int(np.argmin(cloud_clearance(h0, E))), int(np.argmin(cloud_clearance(h1, E)))})
                replace[a.body] = [(h0[i], h1[i], i) for i in picks], a.tool
    out0, out1, used = [], [], set()
    for a, b in zip(starts, ends):
        if a.body not in replace:
            out0.append(a)
            out1.append(b)
        elif a.body not in used:
            pairs, tool = replace[a.body]
            for p0, p1, i in pairs:
                name = a.body + ":hull:" + str(i)
                out0.append(Point(name, p0, 0.0, tool, a.body))
                out1.append(Point(name, p1, 0.0, tool, a.body))
            used.add(a.body)
    return out0 + B.robot.held_points(q0, held), out1 + B.robot.held_points(q1, held), asked


def _gap(where, p, radius):
    if isinstance(where, np.ndarray):
        return float(np.linalg.norm(np.asarray(p, dtype=float) - where)) - radius
    return point_clearance(p, where, radius)


def _phase(seg, va, vb):
    L, up, down, limit = (seg[k] for k in ("L", "up", "down", "limit"))
    if L <= 0:
        work = {**seg, "L": 1.0, "up": seg["joint_up"], "down": seg["joint_up"], "limit": seg["joint_rate"]}
        phase = _phase(work, 0.0, 0.0)
        phase["joint_only"] = True
        phase["up"], phase["down"] = work["up"], work["down"]
        return phase
    peak = min(limit, math.sqrt(max(0, (laws.TWO * up * down * L + down * va ** 2 + up * vb ** 2) / (up + down))))
    da = max(0, (peak ** 2 - va ** 2) / (laws.TWO * up))
    dd = max(0, (peak ** 2 - vb ** 2) / (laws.TWO * down))
    dc = max(0, L - da - dd)
    ta, td = max(0, (peak - va) / up), max(0, (peak - vb) / down)
    tc = dc / peak if peak > 0 else math.inf
    return {"time": ta + tc + td, "ta": ta, "tc": tc, "td": td, "da": da, "dc": dc, "peak": peak, "va": va, "vb": vb}


def _timing(B, nodes, segments, intent):
    count = len(segments)
    caps = [math.inf] * len(nodes)
    labels, keys = ["free"] * len(nodes), ["free"] * len(nodes)
    for k, seg in enumerate(segments):
        for j, cap in ((k, min(seg["limit"], seg["left"])), (k + 1, min(seg["limit"], seg["right"]))):
            if cap < caps[j]:
                caps[j], labels[j], keys[j] = cap, seg["label"], seg["key"]
    for k in range(1, count):
        prev, nxt = segments[k - 1], segments[k]
        if prev["L"] <= 0 or nxt["L"] <= 0:
            continue
        delta = np.abs(prev["dq"] / prev["L"] - nxt["dq"] / nxt["L"])
        qdd = B.qdd_max(nodes[k])
        corner = min((min(a * B.dt() / d, math.sqrt(a * min(prev["L"], nxt["L"]) / d))
                      for a, d in zip(qdd, delta) if d > laws.ROUND), default=math.inf)
        if corner < caps[k]:
            caps[k], labels[k], keys[k] = corner, "corner", "corner"
    caps[0] = min(caps[0], intent.get("v_start", 0.0))
    caps[-1] = min(caps[-1], intent.get("v_end", 0.0))
    labels[-1], keys[-1] = "end", "free"
    for k in reversed(range(count)):
        seg = segments[k]
        v = math.sqrt(caps[k + 1] ** 2 + laws.TWO * seg["down"] * seg["L"])
        if v < caps[k] - laws.ROUND:
            caps[k], labels[k], keys[k] = v, labels[k + 1], keys[k + 1]
    for k, seg in enumerate(segments):
        v = math.sqrt(caps[k] ** 2 + laws.TWO * seg["up"] * seg["L"])
        if v < caps[k + 1]:
            caps[k + 1] = v
    phases, total = [], 0.0
    for k, seg in enumerate(segments):
        phase = _phase(seg, caps[k], caps[k + 1])
        phase["start"] = total
        total += phase["time"]
        phases.append(phase)
    return caps, labels, keys, phases, total


def _sample(nodes, segments, phases, dt, total, labels, keys):
    periods = max(0, int(math.ceil(total / dt - laws.ROUND)))
    path, counts, kinds, segment_indices, coordinates = [], {}, [], [], []
    k = 0
    for i in range(1, periods + 1):
        t = min(i * dt, total)
        while k < len(segments) - 1 and t > phases[k]["start"] + phases[k]["time"] + laws.ROUND:
            k += 1
        seg, phase = segments[k], phases[k]
        local = min(phase["time"], max(0, t - phase["start"]))
        length = 1.0 if phase.get("joint_only") else seg["L"]
        up, down = phase.get("up", seg["up"]), phase.get("down", seg["down"])
        if local <= phase["ta"]:
            x = phase["va"] * local + up * local ** 2 / laws.TWO
            fraction = x / length
        elif local <= phase["ta"] + phase["tc"]:
            x = phase["da"] + phase["peak"] * (local - phase["ta"])
            fraction = x / length
        else:
            remain = phase["time"] - local
            x = length - phase["vb"] * remain - down * remain ** 2 / laws.TWO
            fraction = x / length
        q = nodes[k] + min(1, max(0, fraction)) * seg["dq"]
        path.append(nodes[-1].copy() if i == periods else q)
        label, key = seg["label"], seg["key"]
        if label == "free" and local > phase["ta"] + phase["tc"] and labels[k + 1] not in ("free", "end"):
            label, key = labels[k + 1], keys[k + 1]
        counts[key] = counts.get(key, 0) + 1
        kinds.append(seg["kind"])
        segment_indices.append(k)
        coordinates.append(seg["s"] + (0 if phase.get("joint_only") else x))
    return path, counts, kinds, segment_indices, coordinates


def _shares(segments, phases, labels, keys, dt):
    """Periods governed by each bound, from the phases' own durations (for a plan that is not sampled)."""
    counts = {}
    for k, (seg, phase) in enumerate(zip(segments, phases)):
        tail = phase["td"] if seg["label"] == "free" and labels[k + 1] not in ("free", "end") else 0.0
        counts[seg["key"]] = counts.get(seg["key"], 0) + (phase["time"] - tail) / dt
        if tail > 0:
            counts[keys[k + 1]] = counts.get(keys[k + 1], 0) + tail / dt
    return counts


def _strike(B, nodes, segments, phases, guard, held, total):
    """plan.strike by rcpx.bounds_strike.price (None in a build with none): one sample a straight stretch (its
    fastest speed, the largest model slope of its nodes), every moving body at the plan's joint speeds beside it."""
    from rcpx import bounds_strike
    price = getattr(bounds_strike, "price", None)
    if price is None:
        return None
    stops = B._optional("bounds_stops")
    f_guard = getattr(guard, "force_n", None)
    best, link, slopes, asked = {}, None, {}, {}
    for k, (seg, phase) in enumerate(zip(segments, phases)):
        if not seg["L"] > 0:
            continue
        u = seg["u"]
        v = phase["peak"] * seg["lead"] / seg["L"]
        key = tuple(np.round(u / REL))
        if v > 0 and (key not in best or v > best[key][0]):
            best[key] = (v, k)
        for j in (k, k + 1):                         # both ends of the segment, each node asked once per direction
            if (j, key) not in asked:
                asked[j, key] = B.model_z(nodes[j], u)
            if asked[j, key] is not None and asked[j, key] > slopes.get(key, 0.0):
                slopes[key] = asked[j, key]
        if seg["link"] is not None:
            d_link, point, u_link = seg["link"]
            v_link = phase["peak"] * d_link / seg["L"]
            if link is None or v_link > link[0]:
                link = (v_link, k, point, u_link)
    if not best:
        return None
    samples = [{"v": v, "model_z": slopes.get(key), "f_guard": f_guard, "m_eff": B.m_eff(nodes[k], segments[k]["u"])}
               for key, (v, k) in best.items()]
    said = None
    if link is not None:
        v_link, k, point, u_link = link
        with limit.moving_point(B, nodes[k], point, segments[k]["dq"]):   # the energy bound at the leg's joint step
            said = {"body": point.body, "v": v_link, "model_z": B.model_z(nodes[k], u_link), "f_guard": f_guard}
    v, k = max(best.values())
    gentle = limit.gentle_fact(B, nodes[k], segments[k]["u"], held, sum(seg["tip"] for seg in segments), total)
    cap = {"n": getattr(guard, "cap_n", None), "source": getattr(guard, "source", None)}
    return price(samples, None if stops is None else stops.fit(B._stops()), B.t_det(), cap=cap, link=said, gentle=gentle,
                 moving=bounds_strike.moving_of(B, nodes, segments, phases, held))


def plan(B, q0, path, kind, target=None, intent=None, held=None, away_from=None, materialize=True, expect=None,
         _f_guard=None):
    """Bounds.plan (materialize True) and Bounds.plan_time (False: the same bounds, items and time_s with an empty
    path). _f_guard: set when the plan runs again on its own guard (the threshold its guard came out with, when that is over the
    level its arrivals were solved with)."""
    from rcpx.bounds import Plan
    intent = intent or {}
    nodes = [np.asarray(q0, dtype=float)] + [np.asarray(q, dtype=float) for q in path]
    if len(nodes) <= 1:
        return Plan(guard=B.guard(kind if isinstance(kind, str) else "transit", held=held))
    kinds = [kind] * len(path) if isinstance(kind, str) else list(kind)
    targets = target if isinstance(target, list) and len(target) == len(path) else [target] * len(path)
    if len(kinds) != len(path):
        raise ValueError("one kind is required per path segment")
    qd = B.qd_max()
    qd_source, qd_evidence = B.src("qd_max")
    if intent.get("clear") or intent.get("look") is not None:
        word = B.word(intent, path)       # recorded; it changes no speed and no guard
        if word.get("refused"):
            return Plan(refused=word["refused"])
    expects = limit.expects(B, expect, intent, kinds, targets)
    free_limit = B.stated_limit(None, intent, expected=False)     # a stated limit on free space, or limit_n None
    if free_limit.get("malformed"):       # a stated force that is no number over 0 N (a malformed argument)
        return _refused(free_limit["malformed"], "state a force over 0 N, or none", free_limit["malformed_check"])
    carried = limit.carried(held)             # the limit stated for the thing held bounds every leg that carries it
    if carried is not None and (free_limit["limit_n"] is None or carried["limit_n"] < free_limit["limit_n"]):
        free_limit = carried
    if intent.get("_no_limit"):               # the same motion with no limit: what the limit costs (limit.cost)
        free_limit = {"limit_n": None, "source": "none", "evidence": ""}
    zone = free.zone(B, free_limit)           # where a stated limit holds a stretch, None with none
    segments, bound_items, arrivals, seeks, spans, shown = [], {}, {}, {}, {}, []   # shown: per arrival under the
    bound_items["free"] = report.item("speed", 0.0, qd_source, qd_evidence, "nothing: the robot's own rating")
    coordinate = 0.0                                                    # stated free limit, is it shown kept
    for k, (q0s, q1s) in enumerate(zip(nodes, nodes[1:])):
        ps, pe, _ = _moving_points(B, q0s, q1s, held)
        dq = q1s - q0s
        distances = [float(np.linalg.norm(b.p - a.p)) for a, b in zip(ps, pe)]
        L = max(distances, default=0.0)
        if np.all(dq == 0):
            continue
        joint_time = _joint_min(dq, qd)
        joint_limit = L / joint_time if joint_time > 0 else math.inf
        seg = {"L": L, "dq": dq, "joint_time": joint_time, "joint_limit": joint_limit, "limit": joint_limit,
               "left": joint_limit, "right": joint_limit, "label": "free", "key": "free", "s": coordinate,
               "kind": kinds[k], "target": targets[k], "up": _accel(dq, L, B.qdd_max(q0s)), "down": math.inf}
        if L <= 0:
            seg["joint_rate"] = 1 / joint_time if joint_time > 0 else math.inf
            seg["joint_up"] = _accel(dq, 1.0, B.qdd_max(q0s))
            if not seg["joint_rate"] > 0 or not seg["joint_up"] > 0:
                return _refused("joint-only motion has no speed or acceleration available", "choose a pose with torque available")
        tip_dp = pe[-1].p - ps[-1].p if not held else B.robot.tip(q1s)[0] - B.robot.tip(q0s)[0]
        seg["tip"] = float(np.linalg.norm(tip_dp))
        direction = _unit(tip_dp)
        seg["u"] = direction
        # Every acceleration below is carried into the path coordinate (the fastest point's travel).
        to_path = L / seg["tip"] if seg["tip"] > 0 else math.inf
        braking = B.a_brake(q0s, direction) * to_path if math.isfinite(to_path) else math.inf
        bound_dec = math.inf
        leads = [(a, b, d) for a, b, d in zip(ps, pe, distances) if d > 0 and (a.tool or a.body in ("tip", "held"))]
        links = [(d, a, _unit(b.p - a.p)) for a, b, d in zip(ps, pe, distances) if d >= REL * L and d > 0
                 and not (a.tool or a.body in ("tip", "held"))]
        seg["lead"] = max((d for _, _, d in leads), default=0.0)
        seg["link"] = max(links, key=lambda v: v[0], default=None)

        def bind(key, cap, left, right, item):
            if min(cap, left, right) < min(seg["limit"], seg["left"], seg["right"]) - laws.ROUND:
                seg.update(limit=min(seg["limit"], cap), left=min(seg["left"], left), right=min(seg["right"], right),
                           label="limit", key=key)
            had = bound_items.get(key)       # the item of a stated limit is the slowest arrival of the plan
            if had is None or (item["value"] is not None and (had["value"] is None or item["value"] < had["value"])):
                bound_items[key] = item
            if item["value"] is not None:
                spans.setdefault(key, []).append(item["value"])

        # (1) a LIMIT stated on free space holds the tool, what it holds and (1b) the arm's links, where a
        # contact can begin (bounds_free). The force a touch ends at is no limit on a free stretch
        inside = free.holds(B, zone, ps, pe, coordinate, L)
        lim = zone["limit"] if inside else None
        if lim is not None and leads:
            got = limit.over(B, arrivals, q0s, q1s, direction, lim["limit_n"], lim["source"], held, _f_guard)
            cap = math.inf if got["v_m_s"] is None else got["v_m_s"] * L / seg["lead"]
            shown.append(bool(got["kept"]))
            bind("limit:free", cap, cap, cap, report.item(      # the item stands also where no speed could be solved
                "speed", got["v_m_s"], lim["source"], lim["evidence"] + "; " + got["text"],
                "state another limit, or none"))
        for cap, kept, item in limit.links_under(B, links, q0s, L, free_limit if inside else {}, held, _f_guard, dq):
            shown.append(kept)
            bind("limit:link", cap, cap, cap, item)
        # (2) an expected contact: brakes only for its bound_n, to B.arrival's speed at the region's edge
        want = expects[k]
        if want is not None and want.get("bound_n") is not None and leads:
            got = limit.over(B, arrivals, q0s, q1s, direction, want["bound_n"], want.get("bound_source") or "intent",
                             held, _f_guard, want.get("floor"))
            seeks[id(want)] = seeks.get(id(want), True) and got["seek"]
            where, band = limit.region(B, want)
            v_in, dec = got["v_m_s"], got["dec"]
            cap = left = right = math.inf
            if v_in is not None:
                for a, b, d in leads:
                    if where is None:                      # whatever lies ahead: the whole stretch
                        cap = left = right = min(cap, v_in * L / d)
                        continue
                    c0, c1 = _gap(where, a.p, a.radius), _gap(where, b.p, b.radius)
                    if c1 < c0 - laws.ROUND:               # this point nears the region: brake to its edge
                        share = L / (c0 - c1)
                        bound_dec = min(bound_dec, dec * share)
                        lo, hi = laws.v_toward(c0, band, v_in, dec, math.inf) * share, laws.v_toward(c1, band, v_in, dec, math.inf) * share
                        cap, left, right = min(cap, max(lo, hi)), min(left, lo), min(right, hi)
                    elif min(c0, c1) <= band + laws.ROUND and not c1 > c0 + laws.ROUND:
                        cap = left = right = min(cap, v_in * L / d)     # along it, inside the region
            text = str(want.get("evidence") or f"the expected contact's stated bound of {want['bound_n']:g} N")
            bind("limit:expect", cap, left, right, report.item(
                "speed", v_in, want.get("bound_source") or "intent", text + "; " + got["text"],
                "state another limit on the call (max_n)"))
        for a, b, d in zip(ps, pe, distances):
            if not d > 0 or not B.site:
                continue
            site_limits = []
            if B.site.get("speed_mm_s") is not None:
                site_limits.append(B.site["speed_mm_s"] / report.MM)
            for zone in B.site.get("zones", []):
                if zone.get("speed_mm_s") is not None:
                    lo, hi = np.asarray(zone["box_mm"], dtype=float) / report.MM
                    if np.all(np.maximum(a.p, b.p) + a.radius >= lo) and np.all(np.minimum(a.p, b.p) - a.radius <= hi):
                        site_limits.append(zone["speed_mm_s"] / report.MM)
            if site_limits:
                site_speed = min(site_limits)
                site_cap = site_speed * L / d
                if site_cap <= 0:
                    return _refused("site speed permits no travel", "state a positive site speed", "site")
                if site_cap < seg["limit"]:
                    seg.update(limit=site_cap, left=min(seg["left"], site_cap), right=min(seg["right"], site_cap), label="site", key="site")
                    bound_items["site"] = report.item("speed", site_speed, "site", "the stated site speed", "change the stated site speed")
        if intent.get("speed") is not None:
            speed = intent["speed"]
            if seg["tip"] > 0 and speed <= 0:
                return _refused("stated speed permits no travel", "state a positive speed", "args")
            cap = speed * L / seg["tip"] if seg["tip"] > 0 else math.inf
            if cap < seg["limit"]:
                seg.update(limit=cap, left=min(seg["left"], cap), right=min(seg["right"], cap), label="intent", key="intent")
                bound_items["intent"] = report.item("speed", speed, "intent", "on your word: lower speed", "repeat this call without the lower speed")
        if held and held.get("proofs"):
            capacity = laws.grip_capacity(held["proofs"], held["squeeze_n"])
            grip_accel = laws.accel_ok(capacity, held["mass_kg"], B.robot.g(), cos_up=direction[-1])
            if grip_accel is not None and math.isfinite(to_path):
                seg["up"] = min(seg["up"], grip_accel * to_path)
        if intent.get("accel") is not None and math.isfinite(to_path):
            seg["up"] = min(seg["up"], intent["accel"] * to_path)
        seg["down"] = min(seg["up"], braking, bound_dec)
        if L > 0 and (not seg["up"] > 0 or not seg["down"] > 0):
            return _refused("acceleration cannot carry or brake this motion", "prove the grip or choose a pose with torque available")
        segments.append(seg)
        coordinate += L
    for key, values in spans.items():        # one item for many stretches: say that it is the slowest of them
        if max(values) > min(values):
            bound_items[key] = dict(bound_items[key], evidence=bound_items[key]["evidence"] + (
                f"; this is the slowest stretch: along the path the limit's speed runs from {min(values):g} to "
                f"{max(values):g} mm/s (the stop slope changes with the pose and the direction)"))
    nodes = [nodes[0]]
    for seg in segments:
        nodes.append(nodes[-1] + seg["dq"])
    if not segments:
        return Plan(guard=B.guard(kinds[-1], held=held))
    caps, labels, keys, phases, total = _timing(B, nodes, segments, intent)
    if not math.isfinite(total):
        return _refused("path time is unresolved under the joint bounds", "choose a pose with speed and torque available")
    tip_speeds = [phase["peak"] * seg["tip"] / seg["L"] if seg["L"] > 0 else 0.0 for phase, seg in zip(phases, segments)]
    fastest_segment = int(np.argmax(tip_speeds))
    tip_peak = tip_speeds[fastest_segment]
    guard = B.guard(kinds[-1], v=tip_peak, a=max(seg["up"] for seg in segments), held=held, away_from=away_from)
    level = limit.assumed(arrivals)
    if _f_guard is None and level is not None and guard.force_n is not None and guard.force_n > level * (1 + laws.ROUND):
        # the guard of this plan runs over the level its arrivals were solved with (its fastest stretch and its
        # largest acceleration set it): solve them again with the threshold the guard really runs with
        return plan(B, q0, path, kind, target, intent, held, away_from, materialize, expect, guard.force_n)
    periods = max(0, int(math.ceil(total / B.dt() - laws.ROUND)))
    if materialize:
        output, counts, output_kinds, indices, coordinates = _sample(nodes, segments, phases, B.dt(), total, labels, keys)
    else:
        output, output_kinds, indices, coordinates = [], [], [], []
        counts = _shares(segments, phases, labels, keys, B.dt())
    rates = np.divide(np.abs(segments[fastest_segment]["dq"]), qd, out=np.zeros_like(qd), where=qd > 0)
    joint = int(np.argmax(rates))
    joint_evidence = f"{qd_evidence}; {B.robot.joint_names[joint]} joint rate {qd[joint]:g} per s sets the free-space speed"
    bound_items["free"] = report.item("speed", tip_peak, qd_source, joint_evidence, "nothing: the robot's own rating")
    if "corner" in counts:
        bound_items["corner"] = report.item("speed", min((v for v, label in zip(caps, labels) if label == "corner"), default=tip_peak), "model", "joint acceleration at the change in path tangent", "round the corner inside free space")
    ordered = sorted(counts, key=lambda key: -counts[key])
    items = [bound_items[key] for key in ordered if key in bound_items]
    items += [bound_items[k] for k in ("limit:free", "limit:link", "limit:expect") if k in bound_items and k not in counts]
    items.append(report.item("time", B.limit_s(total), B.src("dt")[0], "planned duration plus the relative margin", "change the path or its binding bound"))
    # free = governed by the body's own laws: joint speed and joint acceleration (a corner included)
    free_count = counts.get("free", 0) + counts.get("corner", 0)
    profile = [{"s_m": (segments[k]["s"] if k < len(segments) else coordinate), "v": v, "limit": label,
                "tip_m": B.robot.tip(q)[0].tolist()} for k, (q, v, label) in enumerate(zip(nodes, caps, labels))]
    guard.expect = limit.sought(expects[-1], seeks)  # MOVE's control_expect reads it
    ruled = sum(counts.values())
    free_share = free_count / ruled if ruled > 0 else 1.0
    result = Plan(output, guard, items, total, periods - int(round(free_share * periods)), free_share,
                  None, None, periods, profile, output_kinds)
    result._segment_indices, result._coordinates = indices, coordinates
    result._segments = segments
    result.strike = _strike(B, nodes, segments, phases, guard, held, total) if materialize else None
    result.limit = limit.whole(free_limit, shown)     # for the receipt's limit block (kept or passed is said)
    if result.limit is not None and materialize:      # what the limit cost and where it held
        bare = plan(B, q0, path, kind, target, {**intent, "_no_limit": True}, held, away_from, False, expect)
        result.limit.update(cost=free.cost(zone, periods, getattr(bare, "periods", periods)), zone=zone)
    return result


def _joint_samples(B, q0, q1):
    dq = q1 - q0
    count = max(1, int(math.ceil(_joint_min(dq, B.qd_max()) / B.dt() - laws.ROUND)))
    return [q0 + dq * (i / count) for i in range(1, count + 1)]


def _solve(B, q, pose):
    p = pose[:-1, -1]
    R = pose[:-1, :-1]
    result = B.robot.ik(q, p, R)
    if isinstance(result, tuple):
        return np.asarray(result[0], dtype=float) if result[1] else None
    return None if result is None else np.asarray(result, dtype=float)


def _shape_coordinates(B, shape, held):
    coordinates = [0.0]
    for q0, q1 in zip(shape, shape[1:]):
        points0 = B.robot.points(q0) + B.robot.held_points(q0, held)
        points1 = B.robot.points(q1) + B.robot.held_points(q1, held)
        distance = max((float(np.linalg.norm(b.p - a.p)) for a, b in zip(points0, points1)), default=0.0)
        coordinates.append(coordinates[-1] + distance)
    return coordinates


def _shape_at(shape, coordinates, s):
    if s <= 0:
        return shape[0]
    if s >= coordinates[-1]:
        return shape[-1]
    for k in range(1, len(shape)):
        if coordinates[k] >= s:
            span = coordinates[k] - coordinates[k - 1]
            share = (s - coordinates[k - 1]) / span if span > 0 else 1.0
            return shape[k - 1] + share * (shape[k] - shape[k - 1])
    return shape[-1]


def _shape_crop(shape, coordinates, start, end):
    return ([_shape_at(shape, coordinates, start)] + [q for q, s in zip(shape, coordinates) if start < s < end]
            + [_shape_at(shape, coordinates, end)])


def _reach(why, hint, leg):
    """A reach refusal of ONE route: it names the leg (plan.reach); the caller may try its plain route first."""
    out = _refused(why, hint, "reach")
    out.reach = {"leg": leg, "why": why}
    return out


def plan_legs(B, q0, legs, held=None, looks=None, intent=None):
    """A stream of legs with rounded corners. A leg may carry expect: its expected contact."""
    q0 = np.asarray(q0, dtype=float)
    stream_intent = intent or next((leg["intent"] for leg in legs if leg.get("intent")), {})
    end_nodes, line_paths = [q0], []
    for leg_index, leg in enumerate(legs):
        target = np.asarray(leg["to"], dtype=float)
        prev = end_nodes[-1]
        if target.ndim == 1:
            end = target
            samples = None
        elif leg.get("line"):
            p0, R0 = B.robot.tip(prev)
            dp = target[:-1, -1] - p0
            travel = float(np.linalg.norm(dp))
            speed = B.v_free(prev, _unit(dp))
            count = max(1, int(math.ceil(travel / (speed * B.dt()) - laws.ROUND))) if speed > 0 else 1
            samples, end = [], prev
            for i in range(1, count + 1):
                share = i / count
                pose = target.copy()
                pose[:-1, -1] = p0 + share * dp
                # Polar projection keeps the interpolated orientation orthogonal.
                U, _, V = np.linalg.svd((1 - share) * R0 + share * target[:-1, :-1])
                pose[:-1, :-1] = U @ V
                end = _solve(B, end, pose)
                if end is None:
                    return _reach("reach cannot solve the stated tip line", "choose a reachable tip pose", leg_index)
                samples.append(end)
        else:
            end = _solve(B, prev, target)
            samples = None
        if end is None:
            return _reach("reach cannot solve the stated end pose", "choose a reachable tip pose", leg_index)
        if np.any(end < B.robot.q_lo) or np.any(end > B.robot.q_hi):
            return _reach("reach exceeds a joint range", "choose a pose inside the joint ranges", leg_index)
        end_nodes.append(end)
        line_paths.append(samples)
    shapes = [[end_nodes[i]] + (samples if samples is not None else _joint_samples(B, end_nodes[i], end_nodes[i + 1]))
              for i, samples in enumerate(line_paths)]
    shape_coordinates = [_shape_coordinates(B, shape, held) for shape in shapes]
    blends = {}
    for k in range(1, len(legs)):
        left, right = shapes[k - 1], shapes[k]
        cs0, cs1 = shape_coordinates[k - 1], shape_coordinates[k]
        corner = end_nodes[k]
        d0, d1 = corner - left[:-1][-1], right[1] - corner
        L0, L1 = cs0[-1], cs1[-1]
        if not L0 > 0 or not L1 > 0:
            continue
        r = min(L0, L1) / laws.TWO
        span0, span1 = cs0[-1] - cs0[:-1][-1], cs1[1]
        if not span0 > 0 or not span1 > 0:
            continue
        change = np.abs(d0 / span0 - d1 / span1)
        corner_speed = min((acc * B.dt() / delta for acc, delta in zip(B.qdd_max(corner), change) if delta > laws.ROUND), default=math.inf)
        corner_speed = min(corner_speed, span0 / _joint_min(d0, B.qd_max()), span1 / _joint_min(d1, B.qd_max()))
        period_travel = corner_speed * B.dt()
        while r > 0 and r >= period_travel:
            p, q = _shape_at(left, cs0, L0 - r), _shape_at(right, cs1, r)
            count = max(1, int(math.ceil(laws.TWO * (_joint_min(corner - p, B.qd_max()) + _joint_min(q - corner, B.qd_max())) / B.dt())))
            blend = [p] + [(1 - t) ** 2 * p + laws.TWO * (1 - t) * t * corner + t ** 2 * q
                           for t in (i / count for i in range(1, count + 1))]
            good = True
            for x, y in zip(blend, blend[1:]):
                for start, finish in zip(B.robot.points(x) + B.robot.held_points(x, held), B.robot.points(y) + B.robot.held_points(y, held)):
                    if B.space.kind(start.p, finish.p, max(start.radius, finish.radius))[0] != "free":
                        good = False
                        break
                if not good:
                    break
            if good:
                blends[k] = {"path": blend, "radius": r}
                break
            r *= GIVE
    path, kinds, targets, expects, sample_legs, previous = [], [], [], [], [], q0
    for i, leg in enumerate(legs):
        incoming = blends.get(i)
        outgoing = blends.get(i + 1)
        start_s = incoming["radius"] if incoming else 0.0
        end_s = shape_coordinates[i][-1] - (outgoing["radius"] if outgoing else 0.0)
        samples = _shape_crop(shapes[i], shape_coordinates[i], start_s, end_s)[1:]
        if incoming:
            samples = incoming["path"][1:] + samples
        for q in samples:
            if np.any(q != previous):
                path.append(q)
                kinds.append(leg.get("kind", "transit"))
                targets.append(leg.get("target"))
                expects.append(leg.get("expect"))
                sample_legs.append(i)
                previous = q
    if all(e is None for e in expects):
        expects = None                        # intent expect "contact" may still mark the last stretch
    result = plan(B, q0, path, kinds, targets, stream_intent, held, legs[-1].get("away_from") if legs else None,
                  expect=expects)
    if result.refused:
        return result
    if not result.path:
        result.legs = [-1] * len(legs)
        result.looks = []
        return result
    period_legs = [sample_legs[k] for k in result._segment_indices]
    result.legs = [max((i for i, leg_id in enumerate(period_legs) if leg_id == j), default=-1) for j in range(len(legs))]
    result.looks = []
    for look in looks or []:
        leg = look["leg"]
        periods = [i for i, leg_id in enumerate(period_legs) if leg_id == leg]
        if periods:
            segment_ids = [i for i, leg_id in enumerate(sample_legs) if leg_id == leg]
            first, last = result._segments[segment_ids[0]], result._segments[segment_ids[-1]]
            at = first["s"] + look["s"] * (last["s"] + last["L"] - first["s"])
            index = min(periods, key=lambda i: abs(result._coordinates[i] - at))
            result.looks.append({**look, "period": index + 1})
    return result
