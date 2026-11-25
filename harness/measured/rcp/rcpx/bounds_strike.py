# SPDX-License-Identifier: AGPL-3.0-only
"""The price of a motion: the largest strike it could make on a rigid thing, and what that number rests on. The strike
never lowers a
free leg; a stated limit does (rcpx/bounds_limit.py).

- price(samples, fit, t_det_s, ...): pure. Per plan sample (a straight stretch at its fastest speed) one figure,
  F_guard + Z x v, by the first rule that applies:
  1. at or under the fastest stop that bears the fit (V1): the fitted bound, bounds_stops.slope (z_source starts
     "fitted:"), on the guard's level plus what the stops showed over that line whatever their speed (fit e0),
     so that the figure gives back at least every stop's own peak. A sample slower than the slowest such stop
     (V0) is priced no less than that stop's own peak: slower than anything measured, what a stop adds by itself
     is not known to shrink (one arm made 2.04 N at a crawl where the line said 2.0 N).
  2. above V1, or with no stop yet: THE ENERGY BOUND (z_source starts "bound:"), for a free
     leg and an arrival stretch alike, at the sample's own speed: the guard's level plus (1 + rel) times the
     body's model at the point held fast along its travel (rcpx/bounds_know_point.py), times max(1, c) where
     this body's stops read over the model, times the speed. Along the point's own travel the mass held fast
     times its speed squared is never more than the kinetic energy the joints carry (bounds_know_point.moving
     says which of the two set it when the plan hands its joint speeds, "moving" below). No slope is carried
     past the fastest stop it was fitted on, and nothing is held to V1 for want of a figure.
     Only where the body's description gives no model at all (no contact model) does a leg state no figure:
     peak_n None, z_source "unresolved".
  The link's figure: inside the stops made with a LINK (fit "link"), the fitted bound; else the energy bound of
  each moving body of the arm at the plan's own joint speeds ("moving": moving_of), one figure per body
  (link.by_body), the worst named (link.body, link.peak_n). Said, never held but under a stated limit.
- moving_of(B, nodes, segments, phases, held): per moving body of a plan, the energy bound's parts at the
  segment where that body runs fastest, at that segment's planned joint speeds (points_of).
- points_of(B, q, qd, held): per moving body at one pose and joint speeds: its fastest point's speed and the
  energy bound's slope there ("tool" stands for the tool's bodies, the tip and the held thing).
- arrive_most(fit): the fastest speed an arrival stretch may run at for a figure to be stated: V1, or None
  before any stop (nothing is measured to stay inside).
- ran(stop): what a contact stop states: the speed the contact was MET at (the reading before the first loaded
  one, or the hand's mean speed over that period where it was changing) and an upper figure of the strike: the
  LARGEST of the strike the stop slope gives for that speed, the bound the readings' own impulse gives and a
  peak a force signal measured; the impulse bound is left out only where the stops that bear the fit cover the
  stop's own travel and pose (rcpx.bounds_stops.covers). Above the fastest stop measured the slope's part is
  the body's model, said so. The arm's own readings cannot tell which body met, so the figure is also never
  under the energy bound of every moving body of the arm at the joint speeds the contact was met at (the
  stop's "moving", made by rcpx.bounds_stops.measure from the bounds' own model): ran.by_body, one per body.
- sample_of(B, q, qd, held, f_guard): one plan sample for price(), gathered from the bounds at ONE pose.
- stretch_of(B, poses, speeds, held, f_guard): one sample for a straight stretch: a thing can be met anywhere along
  it, to its end, and the body's model of the stop slope changes with the pose, so the stretch carries the LARGEST
  model slope over its poses, its fastest speed, and the LEAST moving mass (that one feeds a floor).
- model_z_of(B, q, u): the body's model of the stop slope at a pose (the bounds' own); normal_z_of: the same from
  its parts, as a plain number (a tap is measured against it).
"""

import math

import numpy as np

from rcpx import bounds_stops as stops
from rcpx.bounds_ratios import REL

MM = stops.MM
GENTLE = "your word gentle: the hand's own weight"
READ_NOTE = "the arm's reading once per period, not the peak"
BOUND_NOTE = ("at most: the impulse of two of the arm's readings in a row, gathered in one fast sample (an upper "
              "bound of the peak, not a measurement of it)")
MET_NOTE = "at most: the strike this arm's stop slope gives for the speed the contact was met at"
MODEL_NOTE = "by the body's model of the stop slope at the speed the contact was met at: no stop measured covers it"
PEAK_NOTE = "the peak a force signal at the fast rate gave"
NOT_SHOWN = "by the body's model of the stop slope: not shown kept"
BOUND = ("bound: the energy the motion carries (the guard's level plus the body's model at the point held fast "
         "along its travel: the mass held fast there times its speed squared is never more than the kinetic energy "
         "the joints carry; the contact rate the body's description states)")
NO_LINK = "no stop of this body was made with a link"
COLD = "no stop of this body measured yet"
UNRESOLVED = "unresolved: the body's model gives no stop slope here"


def _figure(model_z, v, guard, fit, rel, link=False, why=None):
    """One sample's figure by the rules of the module text: {"peak": N or None, "z", "source", "v": the speed the
    figure is for, "bound": True for the energy bound (rule 2), "v1": the fastest stop that bears the fit}. Every
    figure stands on the guard's level plus what this body's stops showed over the line whatever their speed (fit
    "e0": measured; dry friction, a force that grows between two readings). link: the figure of a link; the fitted
    bound only inside the stops made with a link and on their own fit (rcpx.bounds_stops.of_kind). why: which of
    the energy bound's two masses set the model (bounds_know_point.moving), said in the source."""
    fit = stops.of_kind(fit, link)
    n, v1 = fit.get("n") or 0, fit.get("v1")
    out = {"peak": None, "z": None, "source": None, "v": v, "bound": False, "v1": v1}
    if model_z is None or guard is None:
        return out
    level = guard + float(fit.get("e0") or 0.0)
    dry = f"; plus {fit['e0']:.3g} N this body's stops showed over that line whatever their speed" if fit.get("e0") else ""
    if n and v1 is not None and v <= v1:                                    # 1. the fitted bound
        z, source = stops.slope(model_z, fit, rel)
        v0, f0 = fit.get("v0"), fit.get("f0")
        least = 0.0
        if v0 is not None and f0 is not None and 0 < v < v0:              # slower than any stop measured: what a
            source += f"; no less than the slowest of them showed ({f0:.3g} N from {v0 * MM:.3g} mm/s)"   # stop adds
            least = f0                                                      # by itself may not shrink with the speed
        return dict(out, peak=max(level + z * v, least), z=z, source=source + dry)
    c = fit.get("c") if n else None                                         # 2. the energy bound, at its own speed
    raised = max(1.0, float(c)) if c is not None else 1.0
    z = (1 + rel) * raised * float(model_z)
    where = (f"above {v1 * MM:.3g} mm/s, the fastest stop measured" if n and v1 is not None
             else (NO_LINK if link else COLD))
    source = (BOUND + f": {where}" + (f"; {why}" if why else "")
              + (f"; raised {raised:.3g} times, as this body's stops read over the model" if raised > 1 else ""))
    return dict(out, peak=level + z * v, z=z, source=source + dry, bound=True)


def arrive_most(fit):
    """The fastest speed (m/s) an arrival stretch may run at inside the stops that bear the fit: V1; None before
    any stop. Above it the energy bound states the figure (rule 2)."""
    fit = fit or {}
    return fit.get("v1") if fit.get("n") else None


def _level(guard, cap):
    """The guard's level of a leg, or the cap where the leg has no contact threshold; None where neither is known."""
    if guard is not None and math.isfinite(guard) and guard > 0:
        return guard, False
    n = None if cap is None else cap.get("n")
    return (n if n is not None and math.isfinite(n) else None), True


def price(samples, fit, t_det_s, *, rel=REL, cap=None, link=None, gentle=None, moving=None):
    """The strike a motion could make (plan.strike).

    samples: [{"v": the fastest point of the tool's bodies and of the held thing's extent at that plan sample (m/s),
    "model_z": the body's model of the stop slope there (N per m/s), "f_guard": the threshold the guard will run
    with on that leg (N; None on a leg with no contact threshold), "m_eff": the moving mass along the travel (kg),
    "arrival": True for an arrival stretch}].
    fit: bounds_stops.fit of the robot's stored stops. t_det_s: the measured detection time. cap: {"n", "source"}
    for a leg with no threshold. link: {"body", "v", "model_z", "f_guard"} of the arm's fastest link, or None.
    moving: moving_of's answer (every moving body at the plan's joint speeds), or None.
    gentle: {"force_n": the hand's own weight, "path_m", "time_s": the plan's own, "v_tell": the tell speed}."""
    samples = [s for s in samples or ()]
    fit = fit or {"c": None, "n": 0, "v0": None, "v1": None}
    v1 = fit.get("v1")
    moving = moving or {}
    best, model_best, at, ref, momentum, blind = None, None, None, None, 0.0, False
    tool = moving.get("tool")
    if tool and tool.get("z") is not None and samples:      # the tool's fastest point at the plan's joint speeds
        samples = samples + [{"v": tool["v"], "model_z": tool["z"], "f_guard": samples[0].get("f_guard"),
                              "m_eff": tool.get("m_kg"), "why": _why(tool)}]
    for s in samples:
        v, mz = float(s.get("v") or 0.0), s.get("model_z")
        guard, capped = _level(s.get("f_guard"), cap)
        momentum = max(momentum, float(s.get("m_eff") or 0.0) * v)
        if mz is None or guard is None:
            blind = blind or v > 0
            continue
        got = _figure(mz, v, guard, fit, rel, why=s.get("why"))
        model_peak = guard + (1 + rel) * mz * v
        if model_best is None or model_peak > model_best:      # the body's model alone, beside; it also prices gentle
            model_best, ref = model_peak, dict(s, f_guard=guard, capped=capped, z=stops.slope(mz, fit, rel)[0])
        if got["peak"] is not None and (best is None or got["peak"] > best):
            best, at = got["peak"], dict(s, f_guard=guard, capped=capped, z=got["z"], z_source=got["source"])
    if blind:            # a part of this motion has no model slope (or no force level): no number stands for the whole
        best = model_best = at = ref = None
    v = max((float(s.get("v") or 0.0) for s in samples), default=0.0)
    stated = best is not None
    inside = bool(fit.get("n")) and v1 is not None and v <= v1
    floor = momentum / t_det_s if t_det_s and t_det_s > 0 and momentum > 0 else None
    if at is None:
        z, z_source = None, ("unresolved: a part of this motion has no model of the stop slope or no force level "
                             "to start from" if blind else UNRESOLVED)
    else:
        z, z_source = at["z"], at["z_source"]
    level = at if stated else ref                       # the sample whose guard's level the result names
    out = {"peak_n": best, "at_least_n": None if stated else floor, "model_peak_n": model_best, "v_m_s": v,
           "f_guard_n": None if level is None else level["f_guard"], "z_n_s_m": z, "z_source": z_source,
           "stops": int(fit.get("n") or 0), "v_max_m_s": v1, "link": None, "gentle": None}
    if level is not None and level["capped"]:
        out["f_guard_source"] = (f"no contact threshold on this leg: the cap of {cap['n']:.3g} N "
                                 f"({cap.get('source', 'what the arm can push')}) stands in")
    say = lambda x: "not known" if x is None else f"{x:.3g} newtons"
    if stated:
        out["text"] = (f"a strike on a rigid thing at {v * MM:.3g} mm/s could reach {best:.3g} newtons "
                       f"(the guard's level {at['f_guard']:.3g} N plus the stop slope: {z_source})")
    else:
        out["text"] = f"peak unresolved ({z_source[len('unresolved: '):]}): at least {say(floor)}"
    g0 = next((s.get("f_guard") for s in samples if s.get("f_guard") is not None), None)
    out["link"] = _links(moving, link, fit, rel, cap, g0)
    if gentle and gentle.get("force_n") is not None:
        out["gentle"] = {"force_n": float(gentle["force_n"]), "seconds": _gentle_s(gentle, level, None if level is None else level["z"]),
                         "source": GENTLE, "shown_by": "contact" if inside else "model"}
        if not inside:     # the speed that keeps it rests on the body's model of the stop slope, not on a stop
            out["gentle"]["note"] = NOT_SHOWN
    return out


def _why(entry):
    """Which of the energy bound's masses and stiffnesses set a moving() entry, in words."""
    if not entry:
        return None
    return f"at {entry.get('point') or 'the point'} the mass is {entry['m_from']}" + (
        f", the servo's term {entry['k_from']}" if entry.get("k_n_m") is not None else "")


def _links(moving, link, fit, rel, cap, guard=None):
    """The link block of price(): one figure per moving body of the arm that is no tool body (moving), the worst
    named; with no moving answer, the one fastest link the plan names (link). None with neither. guard: the
    leg's contact threshold (the arm's one estimate reads every body)."""
    rows = {name: e for name, e in (moving or {}).items() if name != "tool" and e.get("z") is not None}
    if link is not None and link.get("f_guard") is not None:
        guard = link["f_guard"]
    if not rows and link:
        rows = {link.get("body"): {"v": float(link.get("v") or 0.0), "z": link.get("model_z")}}
    if not rows:
        return None
    level, _ = _level(guard, cap)
    by, src, model = {}, {}, {}
    for name, e in rows.items():
        if e.get("z") is None or level is None:
            continue
        peak, source = _body_figure(e, level, fit, rel)
        if peak is None:
            continue
        by[name], src[name] = float(peak), source
        model[name] = level + (1 + rel) * float(e["z"]) * float(e["v"])
    if not by:
        body = None if link is None else link.get("body")
        return {"body": body, "peak_n": None, "model_peak_n": None, "z_source": UNRESOLVED}
    worst = max(by, key=by.get)
    out = {"body": worst, "peak_n": by[worst], "model_peak_n": model[worst], "z_source": src[worst]}
    if len(by) > 1 or moving:
        out["by_body"] = by
    return out


PUSH = ("plus what the joints' stated torque limits can push at that point (no stop was made with a link, so the "
        "guard is not shown to see a contact there and the servo may run on to them)")


def _body_figure(e, level, fit, rel, lead=None):
    """(figure N, source) of one moving body's entry (points_of) on a LINK: inside the stops made with a link the
    fitted bound of its model slope; else the energy bound's impact at its speed plus the servo's part: the
    joints' torque-limited push there (bounds_know_point.push) before the motion, or, after a stop the guard did
    see (lead: that stop's own detection time), the servo's stiffness times that lead and the speed, never over
    the push. With no contact model: (None, why)."""
    v = float(e["v"])
    got = _figure(e["z"], v, level, fit, rel, link=True, why=_why(e) if "m_from" in e else None)
    if got["peak"] is None or not got["bound"] or e.get("impact") is None:
        return got["peak"], got["source"]
    run_on = None if e.get("k_n_m") is None else float(e["k_n_m"]) * float(lead if lead is not None else e["lead_s"]) * v
    push = e.get("push_n")
    if lead is None:
        servo, said = (push, PUSH) if push is not None else (run_on, "plus the servo's stiffness over the measured lead")
        whole = None if e.get("k_n_m") is None or e.get("travel_m") is None else float(e["k_n_m"]) * float(e["travel_m"])
        if whole is not None and (servo is None or whole < servo):   # the command cannot lead by more than the
            servo, said = whole, ("plus the servo's stiffness held fast there times that body's whole travel in "  # plan
                                  "this motion (the most its command can lead the held point by)")
    else:
        servo = run_on if push is None or run_on is None else min(run_on, push)
        said = f"plus the servo's stiffness over this stop's own detection time ({lead:.3g} s), never over the joints' push"
    return level + (1 + rel) * (float(e["impact"]) * v + float(servo or 0.0)), got["source"] + "; " + said


def _gentle_s(gentle, at, z):
    """The seconds the same path takes when its strike is held to the hand's own weight: the path at the speed
    whose strike stays under it, never under the tell speed and never faster than the plan itself."""
    path, now, tell = gentle.get("path_m"), gentle.get("time_s"), gentle.get("v_tell") or 0.0
    if path is None or at is None or not z:
        return None
    v = max((float(gentle["force_n"]) - at["f_guard"]) / z, float(tell))
    if not v > 0:
        return None
    return max(float(path) / v, float(now or 0.0))


def ran(stop):
    """What a contact stop states, for result["strike"]["ran"]: {"v_m_s": the speed the contact was met at,
    "peak_n": an upper figure of the strike, "read_n", "note", "measured_n" where a force signal gave a peak}.
    At or under the fastest stop measured (V1): the fitted bound at the speed met (stop["said"], which the
    bounds hand over with the model slope: rcpx.bounds_stops.said_of), never under a peak a force signal
    measured, and never under the bound the readings' own impulse gives unless the stops that bear the fit cover
    this stop's travel and pose (rcpx.bounds_stops.covers: every tap runs along the support's normal, and a
    sideways or slanted stop onto a thing askew read over the fitted figure: 932 N said for 1330 N true). Where the hand was still speeding up over the readings before the contact (stop
    "v_gain", rcpx.bounds_stops.measure) the figure is for the speed met plus that gain: the contact may have come
    a period later (a bench arm's row met at 200 mm/s read 176 mm/s and stated 100.7 N for 106.7 N without it). Above V1, or before any stop, no fitted slope is carried: peak_n is the LARGEST of
    the body's model at the speed met, the upper bound of two readings' impulse (it holds only for the readings
    the stop was handed) and a measured peak, and the note says so. With none of them: the reading.
    A stop read while the hand already stood on the contact of the stop before (said "before": the bounds hand
    it over when the hand stands where that stop left it, and this stop's speed is under REL of that one's)
    states that arrival's own figures: it is the same contact, read again."""
    if not stop:
        return None
    v, said, thr = stop.get("v"), stop.get("said") or {}, stop.get("f_thr")
    before = said.get("before")
    if before and before.get("v_m_s") and (v is None or v <= REL * before["v_m_s"]):
        return dict(before, again="read again while the hand stood on the contact of the stop before")
    level = None if thr is None or v is None else float(thr) + float(said.get("e0") or 0.0)
    gain = float(stop.get("v_gain") or 0.0)            # a hand still speeding up: the speed one period on
    met = None if level is None or said.get("z") is None else level + float(said["z"]) * (float(v) + gain)
    if met is not None and said.get("v0") is not None and said.get("f0") is not None and float(v) < said["v0"]:
        met = max(met, float(said["f0"]))          # slower than any stop measured: no less than the slowest showed
    v1, n = said.get("v1"), said.get("n")
    fitted = met is not None and ("v1" not in said or (bool(n) and v1 is not None and float(v) <= v1))
    covered, why = stops.covers(said, stop.get("u") or said.get("u"), stop.get("q") or said.get("q"))
    measured, bound = stop.get("peak_n"), stop.get("bound_n")
    have = [x for x in (met, measured, None if fitted and covered else bound) if x is not None]
    if not have:
        return _met_bodies({"v_m_s": v, "read_n": stop.get("read_n"), "note": READ_NOTE}, stop, thr, said)
    top = max(have)
    if met is not None and top == met:
        if fitted:
            note = f"{MET_NOTE} ({said.get('source')})" + ("" if covered else f"; over what the readings bound ({why})")
        else:
            where = (f"above the fastest stop measured ({v1 * MM:.3g} mm/s)" if n and v1 is not None
                     else "no stop of this body measured yet")
            note = f"{MODEL_NOTE} ({where}); never under what the readings bound"
    else:
        note = PEAK_NOTE if measured is not None and top == measured else BOUND_NOTE
        if fitted and not covered and top != measured:
            note += f": the fitted figure does not cover this stop ({why})"
    out = {"v_m_s": v, "peak_n": top, "read_n": stop.get("read_n"), "note": note}
    if measured is not None:
        out["measured_n"] = measured
    return _met_bodies(out, stop, thr, said)


def _met_bodies(out, stop, thr, said):
    """ran()'s answer with the energy bound of every moving body at the joint speeds the contact was met at (the
    stop's "moving": rcpx.bounds_stops.measure, from the bounds' own model): ran.by_body, and peak_n never under
    the largest of them (the arm's one force estimate does not tell which body met)."""
    rows = stop.get("moving") if isinstance(stop.get("moving"), dict) else {}
    if thr is None or not rows:
        return out
    level = float(thr) + float((said or {}).get("e0") or 0.0)
    lead = stop.get("det_s")
    by = {}
    for name, e in rows.items():
        if e.get("z") is None or e.get("v") is None:
            continue
        mine = max(float(e.get("lead_s") or 0.0), float(lead or 0.0))
        if name == "tool":       # the tool's stops are measured, so its servo runs on for THIS stop's own detection
            run_on = 0.0 if e.get("k_n_m") is None else float(e["k_n_m"]) * mine * float(e["v"])
            if e.get("push_n") is not None:
                run_on = min(run_on, float(e["push_n"]))
            by[name] = level + (1 + REL) * max(float(e["z"]) * float(e["v"]),
                                               float(e.get("impact") or 0.0) * float(e["v"]) + run_on)
            continue
        by[name] = _body_figure(e, float(thr), {}, REL, lead=mine)[0]
    if not by:
        return out
    out["by_body"] = {k: round(x, 2) for k, x in by.items()}
    worst = max(by, key=by.get)
    if out.get("peak_n") is None or by[worst] > float(out["peak_n"]):
        out["peak_n"] = by[worst]
        out["note"] = (f"{BOUND}: at the joint speeds the contact was met at, {worst} the largest of the arm's moving "
                       f"bodies (the arm's force estimate does not tell which body met); " + str(out.get("note") or ""))
    return out


def sample_of(B, q, qd, held=None, f_guard=None, u=None):
    """One plan sample for price(): at pose q with joint speeds qd, v is the fastest point of the tool's bodies and
    of the held thing's extent (never a link of the arm: a link's strike is said apart), with the body's model of
    the stop slope and the moving mass along the tip's travel there (u: the travel's direction where the tip
    stands still at this pose; none given: straight down)."""
    q, qd = np.asarray(q, dtype=float), np.asarray(qd, dtype=float)
    points = [p for p in B.robot.points(q) if p.tool] + list(B.robot.held_points(q, held))
    tip = B.robot.jac_tip(q)[:3] @ qd
    v_tip = float(np.linalg.norm(tip))
    u = tip / v_tip if v_tip > 0 else np.asarray((0.0, 0.0, -1.0) if u is None else u, dtype=float)
    speeds = [float(np.linalg.norm(B.robot.point_jacobian(q, p) @ qd)) for p in points]
    return {"v": max(speeds + [v_tip]), "model_z": model_z_of(B, q, u), "f_guard": f_guard,
            "m_eff": B.m_eff(q, u), "u": u}


def points_of(B, q, qd, held=None, M=None):
    """{body: the energy bound at its point (rcpx.bounds_know_point.moving) that could strike hardest, with
    "point"}: per moving body of the arm at pose q with joint speeds qd. "tool" stands for the tool's bodies, the
    tip and the held thing. The points: the ends of every collision capsule of the body's description and, where
    the body has a simulator's model, their middle (the speed and the mass behind a point multiply most where
    neither is largest alone)."""
    from rcpx import bounds_know_point as kp
    q, qd = np.asarray(q, dtype=float), np.asarray(qd, dtype=float)
    if not float(np.linalg.norm(qd)) > 0:
        return {}
    pts = list(B.robot.points(q))
    if hasattr(getattr(B.robot, "model", None), "mj"):
        ends = [(a, b) for a, b in zip(pts, pts[1:]) if a.body == b.body and a.name.endswith(":start")
                and b.name.endswith(":end")]
        pts += [type(a)(a.body + ":middle", (a.p + b.p) / 2.0, a.radius, a.tool, a.body) for a, b in ends]
    pts += list(B.robot.held_points(q, held))
    held_kg = float(held.get("mass_kg") or 0.0) if isinstance(held, dict) else 0.0
    M = np.asarray(B.robot.mass_matrix(q) if M is None else M, dtype=float)
    out = {}
    for p in pts:
        tool = bool(p.tool) or p.body in ("tip", "held")
        got = kp.moving(B, q, qd, B.robot.point_jacobian(q, p), None if tool else p.body, held_kg, M=M)
        if got is None or got["z"] is None:
            continue
        name = "tool" if tool else p.body
        if name not in out or got["z"] * got["v"] > out[name]["z"] * out[name]["v"]:
            out[name] = dict(got, point=p.name)
    return out


def moving_at(B, q):
    """points_of at pose q as a function of the joint speeds alone: what rcpx.bounds_stops.measure asks with the
    joint speeds of the reading before a contact (the bounds' Slope.moving, rcpx/bounds.py model_z)."""
    q = np.array(q, dtype=float)
    return lambda qd: points_of(B, q, qd)


def moving_of(B, nodes, segments, phases, held=None):
    """{body: points_of's entry}: per moving body of a plan, the energy bound at the segment where that body's
    points run fastest, at that segment's planned joint speeds (its joint step over its length times its peak
    speed: the path coordinate is the fastest point's travel). nodes: the plan's joint poses in order; segments
    and phases: the plan's, one per joint step that is not zero."""
    nodes = [np.asarray(q, dtype=float) for q in nodes]
    starts = [a for a, b in zip(nodes, nodes[1:]) if not np.all(b - a == 0)]
    if len(starts) != len(segments):
        return {}
    fastest, travel = {}, {}
    for k, (seg, phase) in enumerate(zip(segments, phases)):
        if not seg.get("L") or not seg["L"] > 0 or not phase.get("peak"):
            continue
        scale = float(phase["peak"]) / float(seg["L"])
        step = {}
        for a, b in zip(B.robot.points(starts[k]), B.robot.points(starts[k] + np.asarray(seg["dq"], dtype=float))):
            name = "tool" if (a.tool or a.body in ("tip", "held")) else a.body
            d = float(np.linalg.norm(b.p - a.p))
            step[name] = max(step.get(name, 0.0), d)
            if d * scale > 0 and (name not in fastest or d * scale > fastest[name][0]):
                fastest[name] = (d * scale, k)
        for name, d in step.items():
            travel[name] = travel.get(name, 0.0) + d
    out = {}
    for k in sorted({k for _, k in fastest.values()}):
        qd = np.asarray(segments[k]["dq"], dtype=float) * float(phases[k]["peak"]) / float(segments[k]["L"])
        for name, e in points_of(B, starts[k], qd, held, M=np.asarray(B.robot.mass_matrix(starts[k]), dtype=float)).items():
            if name not in out or e["z"] * e["v"] > out[name]["z"] * out[name]["v"]:
                out[name] = dict(e, segment=k, travel_m=travel.get(name))
    return out


def stretch_of(B, poses, speeds, held=None, f_guard=None):
    """One sample for price() of a straight stretch (see the module text). poses: the joint poses of the stretch
    in order, as finely as the plan holds them, TO ITS END (where a thing that stands at the stretch's end is
    met); speeds: the joint speeds at each pose, or one for all. v is the fastest point of the tool and of the
    held thing over them; model_z the largest of the body's model over them (None when a pose has none); m_eff
    the least over them; "at": the index of the pose that gave model_z."""
    poses = [np.asarray(q, dtype=float) for q in poses]
    speeds = np.asarray(speeds, dtype=float)
    speeds = [speeds] * len(poses) if speeds.ndim == 1 else list(speeds)
    tips = [np.asarray(B.robot.jac_tip(q)[:3] @ qd, dtype=float) for q, qd in zip(poses, speeds)]
    along = max(tips, key=lambda t: float(np.linalg.norm(t)))          # the travel's direction, where it runs fastest
    u = along / float(np.linalg.norm(along)) if float(np.linalg.norm(along)) > 0 else None
    rows = [sample_of(B, q, qd, held, f_guard, u=u) for q, qd in zip(poses, speeds)]
    slopes = [r["model_z"] for r in rows]
    at = None if any(z is None for z in slopes) else int(np.argmax(slopes))
    return {"v": max(r["v"] for r in rows), "model_z": None if at is None else slopes[at], "f_guard": f_guard,
            "m_eff": min(float(r["m_eff"] or 0.0) for r in rows), "u": rows[0]["u"] if u is None else u,
            "at": at, "poses": len(rows)}


def model_z_of(B, q, u):
    """The body's model of the stop slope at pose q for the tool moving along u (N per m/s, no margin): the
    bounds' own B.model_z."""
    return B.model_z(q, u)


def normal_z_of(B, q, u):
    """The same model as a plain number from its parts (model_parts): what a tap onto the stated support is
    measured against."""
    return stops.model_z(**model_parts(B, q, u))


def model_parts(B, q, u):
    """The facts the model is made of at pose q along u at the tip: {"m_eff", "b_per_s", "k_servo", "t_det_s",
    "lag_s"} (rcpx.bounds_know_point.tap_parts: the tip held fast along u)."""
    from rcpx import bounds_know_point
    q, u = B._pose(q, u)
    got = bounds_know_point.tap_parts(B, q, u)
    seconds = detection(B)[0]
    return {"m_eff": got["m_eff"], "b_per_s": got["b_per_s"], "k_servo": got["k_servo"], "t_det_s": seconds,
            "lag_s": max(0.0, got["lead_s"] - float(B.t_det()))}


def detection(B):
    """(seconds, text, n): the detection time by the robot's stored stops (bounds_stops.t_det)."""
    listed = getattr(B.know, "stops", None)
    return stops.t_det(listed() if callable(listed) else [], B.dt())
