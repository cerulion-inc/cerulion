# SPDX-License-Identifier: AGPL-3.0-only
"""The strikes of package fixture: what each phase of work_part could put on what it met, as a figure with its
source.

Every phase record of work_part carries "strike": the worst of the motions that phase ran, so that a gate matches a
true contact to the leg that made it (the phase records partition the call's control periods in order: begin and
end below read the executor's own counters, and each record says at which period of the call it began).

- a stream (the way in, the way out, the hand going on): the plan's own price as the bounds made it (C2), with
  what its stop showed under "ran" (the core's record). note() keeps it for the phase that ran it.
- the close of the jaws: what the core's one close states (C8): the peak one pad can make on the part and, where
  the core gives it, on anything else a jaw body meets; the arrival speed; the squeeze. note_close().
- a stroke along a part's path (the follow) and a turn about the hand's axis are no planned streams: they are force
  led. Their figure is made here from what they ran: the largest force the stroke could hold or read (its ceiling,
  the largest reading along the way and across it) plus the stop slope the bounds give at that pose (B.slope) times
  the fastest speed the stroke ran; under "ran" the same sum at the speed the hand had when its largest reading
  came. Where the jaws hold the part, ONE pad can carry the hand's whole force on top of its own squeeze: the
  largest squeeze per pad the jaws read over the stroke is added to the level, so that the figure bounds every
  pair a pad makes with the part (a squeeze that rises while the stroke pulls is no part of the hand's reading).
  A turn holds the torque ceiling over the jaws' radius on one pad (with the hand's push and one pad's squeeze);
  what a stop of the turning hand adds is the energy bound at the tool's points at the turn's own joint speeds
  (the design rulings, as bounds_strike states the tool's figure of every leg), not a slope across the axis carried from
  the taps' pose. stroke() and turn().
- the jaws' opening (release()): what a jaw body can put on anything it meets while it opens, the way the
  close states its other_n (C8): the stop of the jaws' own opening rate by their stop slope, or the jaw servo's
  whole push where that is more, plus what the arm read as they began to open.
- a hand that stands on the part as it begins to leave it (stand()): the push it stands with, by its own reading
  and what the still arm cannot tell; easing and backing off go away from it.
- every link (the design rulings; final review 2, severe 3 (b): a link met while the result stated a tool figure
  only): a stroke and a turn also state, under "link", one figure per moving link of the arm at the joint speeds
  they ran (the stroke's: the least squares joint speeds of its fastest tip speed along its path; the turn's: its
  own joint shares), as bounds_strike states the links of every leg (its _links: the energy bound at the link's
  points and what the servo can push there), the worst named and each under "by_body"; the phase keeps for each
  link the largest of its motions'.
- of_phase(): the phase's own block: the worst of its motions by stated figure (rcpx/r2/place_count.py worst),
  "ran" the largest figure any of its stops showed (never under what the arm itself read at that stop), "close"
  the largest close, "motions" how many it ran.
A phase that ran no motion carries no strike. Where the bounds give no stop slope (the backend states no contact
model) the block says so and states the least it could be ("at_least_n"): no figure is made up.
Nothing here reads truth, names a robot or a device, or holds a typed limit. SI inside.
"""

import math

import numpy as np

from rcpx import bounds_report as report
from rcpx import bounds_strike as BS
from rcpx.bounds_ratios import REL
from rcpx.r2.place_count import worst as _worst

READ = "the arm's reading once per period, not the peak"


def _finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))


def total(ex):
    """The control periods this session ran, by kind (the executor's own counters), or None."""
    got = getattr(ex, "periods", None)
    return dict(got) if isinstance(got, dict) else None


def open_phase(ex, rec):
    """A phase begins where the phase before it ended: the counters of the executor as they stood then (what ran
    between two phases is the later one's), so that the records partition the call's periods in order."""
    fx = ex._fx
    now = total(ex)
    if now is None:
        return
    start = fx.get("_n_at") or now
    fx.setdefault("_n0", dict(start))
    rec["_n"] = dict(start)
    rec["from_period"] = int(sum(start.values()) - sum(fx["_n0"].values()))


def close_phase(ex, rec):
    """The phase ends: its periods are what the executor ran since it began, by kind; its strike is the worst of
    the motions it ran."""
    start, now = rec.pop("_n", None), total(ex)
    if start is not None and now is not None:
        for kind in ("moving", "waiting", "jaw"):
            rec[kind] = int(now.get(kind, 0) - start.get(kind, 0))
        rec["periods"] = int(sum(now.values()) - sum(start.values()))
        ex._fx["_n_at"] = now
    strike = of_phase(rec)
    if strike is not None:
        rec["strike"] = strike
    return rec


def _rec(ex):
    phases = (getattr(ex, "_fx", None) or {}).get("phases") or []
    return phases[-1] if phases else None


def note(ex, strike):
    """Keep the strike block of one motion for the phase that ran it."""
    rec = _rec(ex)
    if rec is not None and isinstance(strike, dict):
        rec.setdefault("_strikes", []).append(strike)
    return strike


def note_close(ex, c, pads):
    """Keep what one close of the jaws states (C8) for the phase that ran it. c: the core's result."""
    rec = _rec(ex)
    if rec is None or not isinstance(c, dict) or "refused" in c:
        return None
    given = c.get("strike") if isinstance(c.get("strike"), dict) else {}
    pad = given.get("pad_n", c.get("peak_n"))
    block = {"pad_n": None if not _finite(pad) else float(pad),
             "other_n": None if not _finite(given.get("other_n")) else float(given["other_n"]),
             "arrive_m_s": given.get("arrive_m_s", c.get("arrive_m_s")),
             "squeeze_n": c.get("squeeze_n"), "pads": int(pads), "ended": c.get("ended"),
             "source": given.get("source") or "the core's close: the jaws' own numbers (C7, C8)"}
    rec.setdefault("_closes", []).append(block)
    return block


def _slope(B, q, u, thing, part_kg=0.0):
    try:
        z, source = B.slope(q, u, thing, part_kg)
    except Exception as exc:      # a pose the bounds give no slope for: said, no figure made up
        return None, f"unresolved: {type(exc).__name__}: {exc}"
    return (None if not _finite(z) else float(z)), str(source)


def _links_at(ex, B, q, qd, level):
    """The link block of a force led motion at pose q with joint speeds qd (rcpx/bounds_strike.py _links over
    points_of: one figure per moving link that is no tool body, the worst named, each under by_body), over the
    guard's level `level`; None where no link moves or the body states no contact model (said in the journal when
    the bounds cannot answer)."""
    try:
        qd = np.asarray(qd, dtype=float).reshape(-1)
        if not float(np.linalg.norm(qd)) > 0:
            return None
        stops = B._optional("bounds_stops")
        fit = None if stops is None else stops.fit(B._stops())
        return BS._links(BS.points_of(B, q, qd), None, fit, REL, None, guard=float(level))
    except (AttributeError, KeyError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
        if callable(getattr(ex, "_journal", None)):
            ex._journal("fixture_problem", where="the links' figure of a force led motion",
                        error=f"{type(exc).__name__}: {exc}")
        return None


def _stroke_qd(ex, q, d, v):
    """The joint speeds of a tip moving at v (m/s) along d at pose q: the least squares answer of the tip's own
    Jacobian (the arm's body model)."""
    u = np.asarray(d, dtype=float).reshape(-1)[:3]
    size = float(np.linalg.norm(u))
    if not size > 0 or not v > 0:
        return None
    J = np.asarray(ex.model.jac(np.asarray(q, dtype=float)), dtype=float)[:3]
    return np.linalg.pinv(J) @ (u / size * float(v))


def _block(level, level_text, z, z_source, v_fast, v_met, read_n, what, link=None):
    """The strike block of a force led stroke: level + Z x v at the fastest speed it ran, and under "ran" at the
    speed the hand had when its largest reading came; link: the links' block at the fastest speed (_links_at)."""
    mm = report.MM
    if z is None:
        return {"peak_n": None, "at_least_n": float(level), "model_peak_n": None, "v_m_s": float(v_fast),
                "f_guard_n": float(level), "z_n_s_m": None, "z_source": z_source, "link": link, "stroke": what,
                "ran": {"v_m_s": float(v_met), "read_n": float(read_n), "note": READ},
                "text": f"peak unresolved ({z_source}): at least {level:.3g} newtons, {level_text}"}
    peak, met = level + z * v_fast, level + z * v_met
    return {"peak_n": float(peak), "at_least_n": None, "model_peak_n": None, "v_m_s": float(v_fast),
            "f_guard_n": float(level), "z_n_s_m": float(z), "z_source": z_source, "link": link, "stroke": what,
            "ran": {"v_m_s": float(v_met), "peak_n": float(met), "read_n": float(read_n),
                    "note": f"at most: {level_text}, plus the stop slope at the speed the hand had when its largest "
                            "reading came (an upper bound of the peak, not a measurement of it)"},
            "text": (f"{what}: a stop on a rigid thing at {v_fast * mm:.3g} mm/s, the fastest it ran, could reach "
                     f"{peak:.3g} newtons ({level_text}: {level:.3g} N, plus the stop slope: {z_source})")}


def stroke(ex, B, q, d, thing, ceiling_n, read_n, side_n, v_fast, v_met, part_kg=None, pad_n=None):
    """The strike of a follow (C12), kept for the phase: the largest force the stroke could hold along its path
    (its ceiling; the largest reading where one stood over it) with the largest reading across the path, plus
    the stop slope at this pose times the fastest speed it ran. pad_n: where the jaws hold the part, the largest
    squeeze per pad they read over the stroke: one pad can carry the hand's whole force on top of it, so it is
    added to the level, before the motion and under "ran" (a figure of the hand's push alone was 28.0 N after the
    stop where one jaw's pair with the drawer rose 31.6 N: the squeeze rose while the stroke pulled; own bench)."""
    level = math.hypot(max(float(ceiling_n), float(read_n)), float(side_n))
    z, source = _slope(B, q, d, thing, float(part_kg or 0.0))
    text = "the ceiling of the stroke, or its largest reading where that stood over it, with the largest reading across the path"
    if _finite(pad_n):
        level += max(0.0, float(pad_n))
        text += (f", and on top the largest squeeze one pad read over the stroke ({float(pad_n):.3g} N: one pad can "
                 "carry the hand's whole force over its own squeeze)")
    fast = max(0.0, float(v_fast))
    try:
        qd = _stroke_qd(ex, q, d, fast)
    except (AttributeError, TypeError, ValueError, np.linalg.LinAlgError):
        qd = None
    link = None if qd is None else _links_at(ex, B, q, qd, level)
    return note(ex, _block(level, text, z, source, fast, max(0.0, float(v_met)), float(read_n),
                           "the stroke along the part's path", link=link))


def release(ex, got, arm_n):
    """The strike of the jaws' opening, kept for the phase (the design rulings: an opening jaw can meet other bodies beside the
    part; a finger stood on a device's stop button through a let go with no figure): what a jaw body can put on
    anything it meets while it opens, as the core's close states its other_n (C8): the stop of the jaws' own
    opening rate by their stop slope, or the jaw servo's whole push (its force range through the pad's shortest
    lever) where that is more; a slope that is the body's model carries the accepted relative error on top; plus
    what the arm read as the jaws began to open (a finger that bears on something carries the arm's push too).
    got: the core's release result. No figure where the jaws state neither a slope nor a force range. No stop is
    met by an opening: the block carries no "ran" (a "ran" would read the phase's other motions as met at it)."""
    from rcpx import control_hold_jaws as CJ
    if not isinstance(got, dict) or "refused" in got or not got.get("periods"):
        return None
    F, N = CJ.facts(ex), CJ.numbers(ex)
    if not F or not N:
        return None
    v = float(N.get("speed_open") or N["speed"]) * float(N["gap_per_span_m"])
    z = N.get("z_jaw")
    dyn = None if not _finite(z) else float(N["floor_n"]) + float(z) * v
    push = None if F.get("force_range") is None else float(np.max(np.abs(np.asarray(F["force_range"], dtype=float))
                                                                    / np.asarray(F["lever_min"], dtype=float)))
    if dyn is None and push is None:
        return note(ex, {"peak_n": None, "at_least_n": float(arm_n), "model_peak_n": None, "link": None,
                         "stroke": "the release of the jaws", "z_source": "these jaws state no stop slope and no force range"})
    level = max(dyn or 0.0, push or 0.0) * ((1.0 + REL) if N.get("z_jaw_source") == "model" else 1.0)
    peak = level + max(0.0, float(arm_n))
    source = ("the jaws' own stop slope (" + ("the body's model" if N.get("z_jaw_source") == "model" else "measured")
              + f") at their opening rate, {v * report.MM:.0f} mm/s, or the jaw servo's whole push where that is more; "
              f"plus the {float(arm_n):.3g} N the arm read as they began to open")
    return note(ex, {"peak_n": peak, "at_least_n": None, "model_peak_n": None, "v_m_s": v, "f_guard_n": float(arm_n),
                     "z_n_s_m": None if not _finite(z) else float(z), "z_source": source, "link": None,
                     "stroke": "the release of the jaws",
                     "text": f"the jaws' opening: a jaw that meets something could put up to {peak:.3g} newtons on it"})


def stand(ex, B, s):
    """The push the hand stands with on what it worked as it begins to leave it, kept for the phase: its own
    reading with what the still arm cannot tell (B.f_thr at rest). Easing a wound servo and backing off go away
    from it, and one body of the hand can carry all of it (a finger's pair with a drawer rose 11.3 N while the
    hand eased and left, the phase stating nothing: own bench)."""
    read = float(np.linalg.norm(np.asarray(s.F, dtype=float)))
    level = read + float(B.f_thr(0, 0))
    source = "the push the hand stood with as it began to leave, by its own reading, with what the still arm cannot tell"
    return note(ex, {"peak_n": level, "at_least_n": None, "model_peak_n": None, "v_m_s": 0.0, "f_guard_n": level,
                     "z_n_s_m": None, "z_source": source, "link": None, "stroke": "the hand standing as it leaves",
                     "text": f"the hand stood with {read:.3g} N as it began to leave"})


def _energy(B, q, qd):
    """(the energy bound's figure over the guard's level at the tool's points moving with the joint speeds qd, its
    entry): the strike a stop there could add, as bounds_strike states it for the tool of every leg:
    (1 + REL) times the larger of the slope times the speed and the impact plus what the servo runs on by over the
    detection time, never over what the joints can push there. (None, None) where the body states no contact
    model or the tool does not move."""
    e = BS.points_of(B, q, qd).get("tool") if float(np.linalg.norm(qd)) > 0 else None
    if not e or e.get("z") is None:
        return None, None
    v = float(e["v"])
    run_on = 0.0 if e.get("k_n_m") is None else float(e["k_n_m"]) * float(e["lead_s"]) * v
    if e.get("push_n") is not None:
        run_on = min(run_on, float(e["push_n"]))
    return (1 + REL) * max(float(e["z"]) * v, float(e.get("impact") or 0.0) * v + run_on), e


def turn(ex, B, q, side, thing, ceiling_nm, read_nm, radius_m, force_n, w_fast, w_met, weights=None, pad_n=None):
    """The strike of a turn about the hand's axis (C12), kept for the phase. What the pads hold: at the jaws'
    radius the torque ceiling (the largest torque read where it stood over it) is a force on one pad, with the
    largest force the hand read while it turned (the push along the axis is no part of the torque) and, on top,
    the largest squeeze one pad read (pad_n). What a stop of the turning hand adds to it: the energy bound at the
    tool's points at the turn's own joint speeds (weights: the joints' share of a turn, rad per rad, times the
    speed: the design rulings, as bounds_strike states it for the tool of every leg), before the motion at the fastest the
    hand turned and under "ran" at the speed it turned when its largest torque came. Where the body states no
    contact model, the stop slope across the axis times the speed of the pads stands in, as before."""
    r = max(float(radius_m), np.finfo(float).tiny)
    level = math.hypot(max(float(ceiling_nm), float(read_nm)) / r, float(force_n))
    text = ("the torque ceiling of the turn at the jaws' radius, or the largest torque read where that stood over "
            "it, with the largest force the hand read while it turned")
    if _finite(pad_n):
        level += max(0.0, float(pad_n))
        text += f", and on top the largest squeeze one pad read ({float(pad_n):.3g} N)"
    what = "the turn about the hand's axis"
    w = None if weights is None else np.asarray(weights, dtype=float)
    fast, e = (None, None) if w is None else _energy(B, q, w * max(0.0, float(w_fast)))
    met, _ = (None, None) if w is None else _energy(B, q, w * max(0.0, float(w_met)))
    link = None if w is None else _links_at(ex, B, q, w * max(0.0, float(w_fast)), level)
    if fast is None:
        z, source = _slope(B, q, side, thing)
        return note(ex, _block(level, text, z, source, max(0.0, float(w_fast)) * r, max(0.0, float(w_met)) * r,
                               float(read_nm) / r, what, link=link))
    met = 0.0 if met is None else met
    source = (f"{BS.BOUND}: at {e.get('point') or 'the tool'}, the mass {e.get('m_from')}, the servo's term "
              f"{e.get('k_from')}")
    mm = report.MM
    return note(ex, {"peak_n": float(level + fast), "at_least_n": None, "model_peak_n": None,
                     "v_m_s": float(e["v"]), "f_guard_n": float(level), "z_n_s_m": float(e["z"]), "z_source": source,
                     "link": link, "stroke": what,
                     "ran": {"v_m_s": max(0.0, float(w_met)) * r, "peak_n": float(level + met),
                             "read_n": float(read_nm) / r,
                             "note": f"at most: {text}, plus the energy bound at the speed the hand turned when its "
                                     "largest torque came (an upper bound of the peak, not a measurement of it)"},
                     "text": (f"{what}: a stop of the turning hand at {float(e['v']) * mm:.3g} mm/s at its fastest "
                              f"point could reach {level + fast:.3g} newtons ({text}: {level:.3g} N, plus the energy "
                              f"bound at the turn's joint speeds: {source})")})


def _ran_size(ran):
    if not isinstance(ran, dict):
        return -1.0
    for key in ("peak_n", "read_n"):
        if _finite(ran.get(key)):
            return float(ran[key])
    return -1.0


def of_phase(rec):
    """The phase's own strike block (see the file's head), or None for a phase that ran no motion and no close.
    The scratch lists of the phase are taken off its record."""
    strikes = [s for s in rec.pop("_strikes", []) if isinstance(s, dict)]
    closes = rec.pop("_closes", [])
    if not strikes and not closes:
        return None
    out = dict(_worst(strikes) or {"peak_n": None, "at_least_n": None, "model_peak_n": None, "link": None,
                                    "z_source": "the phase ran no arm motion: the close's own figure stands"})
    out.pop("of_all", None)
    rans = [s["ran"] for s in strikes if _ran_size(s.get("ran")) >= 0]
    if rans:
        out["ran"] = dict(max(rans, key=_ran_size))
        read = [float(r["read_n"]) for r in rans if _finite(r.get("read_n"))]
        if read and _finite(out["ran"].get("peak_n")) and max(read) > float(out["ran"]["peak_n"]):
            # a figure after the stop is never under what the arm itself read at a stop of this phase
            out["ran"].update(peak_n=max(read), note=str(out["ran"].get("note") or "") + "; raised to the arm's own reading")
    else:
        out.pop("ran", None)
    links = [s["link"] for s in strikes if isinstance(s.get("link"), dict)]
    by = {}
    for blk in links:                 # every link keeps the largest figure any motion of the phase stated for it
        for name, v in ((blk.get("by_body") or {}) or ({blk.get("body"): blk.get("peak_n")} if blk.get("body") else {})).items():
            if _finite(v) and (name not in by or float(v) > by[name]):
                by[name] = float(v)
    if by:
        top = max(by, key=by.get)
        base = dict(out.get("link") or {}) if isinstance(out.get("link"), dict) else {}
        if base.get("body") != top:
            base = dict(max(links, key=lambda b: -1.0 if not _finite(b.get("peak_n")) else float(b["peak_n"])))
        base.update(body=top, peak_n=by[top], by_body=by)
        out["link"] = base
    out["motions"] = len(strikes)
    if closes:
        top = max(closes, key=lambda c: -1.0 if c["pad_n"] is None else c["pad_n"])
        out["close"] = dict(top, closes=len(closes))
        both = None if top["pad_n"] is None else float(top["pad_n"]) * int(top["pads"])
        if both is not None and (not strikes or (_finite(out.get("peak_n")) and float(out["peak_n"]) < both)):
            # every pad presses the part at once: where the phase's arm motions state less, that sum is its figure
            out.update(peak_n=both, z_source="the close of the jaws, every pad at once: the phase's arm motions state less")
    return out


def of_call(phases):
    """The call's strike: the worst of its phases' (the receipt carries the largest of its motions, C2), with the
    phase that states it."""
    rows = [(p.get("phase"), p["strike"]) for p in phases if isinstance(p.get("strike"), dict)]
    if not rows:
        return None
    out = _worst([s for _, s in rows])
    if out is not None:
        top = max(rows, key=lambda r: -1.0 if not _finite(r[1].get("peak_n")) else float(r[1]["peak_n"]))
        out = dict(out, phase=top[0])
    return out
