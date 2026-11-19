# SPDX-License-Identifier: AGPL-3.0-only
"""Expected contact and the stream of legs that may carry one.

A leg of a stream, a stroke, or a move called with expect "contact" carries

    expect = {"with": a thing id, [x, y, z] m or None (None: whatever lies ahead on this leg),
              "radius_m": the spot's own error plus the tool's half width, "along": a unit vector or None,
              "bound_n": N or None, "bound_source": text or None, "seek": bool}

arrived() says whether a guard stop IS that contact: the stop lies inside the region, its force opposes the travel,
and the guard's own locating (control_sense._locate) puts the push on the tool, or, while something is held, on the
hand and what it carries. Then the leg ends "arrived": the caller's own next phase goes on, no world entry of kind
contact is stored, and the bounds drop only that region. A push the locating puts on a link, a stop outside the
region or across the travel is a contact as before: stored, it ends the call, and the result names the link.

stream() is SpeedMixin._stream: several legs as ONE planned path. The guard is derived again at every leg boundary
(a hold's guard is valid for one leg), control_hold.after_move is told every leg's end, a look never stops the
stream, a stop the bounds call false resumes it, and the plain budget is counted for every piece before it runs.
retreat() is MotionMixin._retreat_impl: the paths to retrace as one line, counted before it moves.
record_contact() is MotionMixin._record_contact: what a stop stores in the world, and what it does not.

What a move that starts on a contact tells its guard is rcpx/control_tail.py start_of; stream() asks it
for every leg.
No robot is named here.
"""

import importlib
import inspect
import math

import numpy as np

from rcpx.bounds_ratios import GIVE, QUANT, REL, TWO_READ
from rcpx.control_base import MM, Felt, Guard, _mm, _val
from rcpx.control_plan import NEWLY

DOWN = np.array([0.0, 0.0, -1.0])
NOTHING_MET = "felt nothing at the stated height of the surface under the arm: the hand stands there, still"
MEETS = ("touch", "press", "lower", "approach", "close", "follow", "explore")   # kinds whose end may meet a thing
LIMIT_WORDS = ("max_n", "gentle")   # the words of a call that are limits; a force_n is none



def _stream_price(ex, plan, word, q0, part, held, moving):
    """control_move_price.price_of for a stream of legs: the same legs counted under another intent by the bounds'
    own plan (no motion), or None when the plan states no limit."""
    from rcpx import control_move_price

    def count(it):
        clean = {k: v for k, v in (it or {}).items() if k not in ("_limit_n", "_readings", "_share")}
        legs = [dict(lk, intent={**(lk.get("intent") or {}), **clean}) if lk.get("intent") else lk for lk in part]
        if any(k in clean for k in ("_no_limit",)) or not any(k in clean for k in control_move_price.OWN):
            legs = [dict(lk, intent={k: v for k, v in (lk.get("intent") or {}).items()
                                     if k not in control_move_price.OWN}) if lk.get("intent") else lk for lk in legs]
        rec = ex._held_for(held)
        if isinstance(rec, dict) and not any(k in clean for k in control_move_price.OWN) and \
                rec.get("limit_source") == "intent":
            rec = dict(rec, limit_n=None, limit_source=None)    # the same call without the word: its hold too
        got = ex.bounds.plan_legs(q0, legs, held=rec, looks=moving, intent=clean or None)
        return len(_attr(got, "path") or [])
    return control_move_price.price_of(getattr(plan, "limit", None), word or {}, count)

def ring(ex):
    """How many readings a drive keeps around a stop for the stop's own measurement: those of one settle time of
    this body (the lag tail a stop can be met in) and the readings a contact is counted on."""
    return int(math.ceil(_val(ex.bounds.t_settle()) / ex.dt)) + int(QUANT) + int(QUANT)


def _unit(v, fallback=DOWN):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return fallback.copy() if n <= 0.0 else v / n


def _attr(obj, name, default=None):
    """A field of a plan, spec or record, whether the bounds hand it as an attribute or a key."""
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _module(name):
    """An rcpx module another package of this round brings, or None in a tree that has none yet."""   # compat shim
    try:
        return importlib.import_module("rcpx." + name)
    except ImportError:
        return None


def _hold_module():
    """rcpx.control_hold: the one hold record and its levels."""
    return _module("control_hold")


def of_intent(ex, intent, u, thing=None):
    """The expect record of a move or stroke the operator called with expect "contact" (or of touch and the
    approach of press): whatever lies ahead on this leg, within the tool's own half width of its line; the bound
    is what somebody stated (B.stated_limit: the call's max_n, force_n or gentle, a sheet of the named thing, a
    give, the site, a rating), else None: nothing lowers the motion."""
    B = ex.bounds
    limit = B.stated_limit(thing, intent or None, expected=True)
    # what the call that makes or works this contact stated as a LIMIT, and the thing it resolved: a move that
    # later leaves the contact keeps that limit until the contact has ended (control_tail.start_of)
    ex._meant = {"call": getattr(ex, "_calls", None), "thing": thing if isinstance(thing, str) else getattr(thing, "id", None),
                 "words": {k: v for k, v in (intent or {}).items() if k in LIMIT_WORDS and v}}
    bound = limit.get("limit_n")              # with nothing stated the bounds name the source "none": no source here
    return {"with": thing, "radius_m": _val(B.standoff(0.0)), "along": None if u is None else _unit(u),
            "bound_n": bound, "bound_source": limit.get("source") if bound is not None else None, "seek": True,
            "floor": limit.get("floor")}        # the force at which a touch ends: never under the tell speed


def region(ex, expect):
    """expect with what arrived() needs resolved: the point or the plane of a thing named by id, as the bounds hold
    it, and, while something is held, the names of the hand's bodies (a push on them or beyond them is on the
    tool)."""
    out = dict(expect)
    ref = expect.get("with")
    if isinstance(ref, str):
        try:
            thing = ex.bounds.thing(ref)
        except Exception:
            thing = None
        extent = getattr(thing, "extent", None)
        out["_plane"] = extent.get("plane") if isinstance(extent, dict) else None
        out["_point"] = getattr(thing, "point", None)
    elif ref is not None:
        out["_point"] = np.asarray(ref, dtype=float)
    out["_hand"] = sorted(ex._tool_bodies) if ex._held_for(None) else None
    return out


def on_tool(expect, contact):
    """True when the guard's own locating puts the push on the tool: at the tip or on a tool surface; while
    something is held also a push no point of the hand explains whose loaded joints leave only the hand's bodies
    (it acts on what the hand carries)."""
    if contact.get("source", "tool") == "tool":
        return True
    hand = expect.get("_hand")
    links = contact.get("likely_links") or []
    return bool(hand) and bool(links) and all(name in hand for name in links)


def arrived(expect, contact, travel_u):
    """True only for a guard stop that is the contact this leg expected (see the module text). expect: the record
    (resolved by region() when "with" is an id); contact: the guard's contact as results carry it; travel_u: the
    leg's unit travel (base frame)."""
    if not expect or not contact:
        return False
    if not on_tool(expect, contact):
        return False
    along = expect.get("along")
    u = _unit(travel_u if along is None else along)
    f = np.asarray(contact.get("force_vec_n") or np.zeros(len(u)), dtype=float)
    size = float(np.linalg.norm(f))
    if size > 0.0 and float(f @ u) < GIVE * size:
        return False                       # the push runs across the travel, or with it: not the thing ahead
    point = contact.get("point_mm")
    p = None if point is None else np.asarray(point, dtype=float) / MM
    radius = float(expect.get("radius_m") or 0.0)
    plane, centre = expect.get("_plane"), expect.get("_point")
    if p is None:                          # a push on what the hand carries has no measured place: the tip's stands in
        at = contact.get("tip_mm_at_stop")
        if at is None:
            return plane is None and centre is None
        p = np.asarray(at, dtype=float) / MM
    if plane is not None:
        n = np.asarray(plane[0], dtype=float)
        return abs(float(n @ p - plane[1])) / float(np.linalg.norm(n)) <= radius
    if centre is not None:
        return float(np.linalg.norm(p - np.asarray(centre, dtype=float))) <= radius
    return True                            # whatever lies ahead on this leg


def link_of(contact):
    """The link a contact that is not the tool's is named with (None for a tool contact)."""
    if not contact or contact.get("source", "tool") == "tool":
        return None
    links = contact.get("likely_links") or []
    return links[0] if links else "a link of the arm"


def cut_at_support(ex, q0, way, stroke=False):
    """(way, cut): the joint waypoints of a move whose TIP would pass through the surface under the arm, ended
    where the tip reaches it and as far past it as a search for it may go (what is known of its height; nothing
    with no stated error), else unchanged. cut: None, or the sentence the result carries. The move is never
    refused for the surface: it ends there under its own contact stop, held until the hand is still
    (_drive). stroke: the caller states its own travel (touch, press): with no error stated for the surface's
    height nothing ends the stroke but its travel (the stated height is not where the hand meets the thing)."""
    if not way:
        return way, None

    def over(q):
        p = ex.model.fk_tip(np.asarray(q, dtype=float))[0]
        return float(p[2] - ex._floor_at(p))

    depth = ex._margins(0.0, "touch")[2]
    if depth is None and stroke:
        return way, None
    depth = depth or 0.0
    z0 = over(q0)
    prev_q, prev_z = np.asarray(q0, dtype=float), z0
    for k, q in enumerate(way):
        q = np.asarray(q, dtype=float)
        z = over(q)
        if z < -depth and z < z0 - NEWLY:
            share = min(1.0, max(0.0, (prev_z + depth) / (prev_z - z))) if prev_z > z else 0.0
            out = list(way[:k]) + ([prev_q + share * (q - prev_q)] if share > 0.0 else [])
            return out, (f"the line passes through the surface under the arm (z {ex._floor_at(ex.model.fk_tip(q)[0]) * MM:.0f} "
                         f"mm): it was cut there ({depth * MM:.1f} mm past it, what is known of its height) and "
                         "ends under the contact stop, which stays armed until the hand stands still")
        prev_q, prev_z = q, z
    return way, None


def arm_cap(ex, q, u, cap_n=None):
    """The cap of a move along u at q (N): what the arm itself can push there less its torque error, and the site's
    stated force when there is one. With a tool rating stated (spec tool_rating_n) the bounds' own cap stands
    (cap_n); the jaw motor's force range is no contact rating and caps nothing."""
    if ex.model.spec.get("tool_rating_n") is not None and cap_n is not None:
        return float(cap_n)
    B = ex.bounds
    arm = _val(B.f_cap(q, _unit(u))) * (1.0 - _val(B.e_tau()))
    site = (ex.cfg["site"] or {}).get("force_n")
    return arm if site is None else min(arm, float(site))


def own_cap(ex, g, s):
    """True when a cap stop is the arm's own: a joint taken to its rating from outside, the controller's overload,
    a tip force over what the arm can push along its travel, or over a rating somebody stated. A tip force over
    the jaw motor's force range alone is a contact."""
    if not str(g.cap_why).startswith("tip force") or ex.model.spec.get("tool_rating_n") is not None:
        return True
    u = getattr(ex, "_travel_direction", None)
    return max(s.Fmag, float(getattr(s, "Fend", None) or 0.0)) > arm_cap(ex, s.q, DOWN if u is None else u)


def _tip_speed(ex, s, u=None):
    v = ex.model.jac(s.q)[:3] @ s.qd
    return v if u is None else float(v @ u)


def _measure(ex, g, travel):
    """(the stop as rcpx.bounds_stops.measure reads it, or None; did the hand itself stop: bool; the reading before
    the stop began, or None). The samples are the readings the drive kept around the stop (g.recent: (reading,
    commanded pose) per control period)."""
    stops, recent = _module("bounds_stops"), list(getattr(g, "recent", None) or [])
    if len(recent) < QUANT:
        return None, False, None
    thr = g.force_n
    f = [max(s.Fmag, float(getattr(s, "Fend", None) or 0.0)) for s, _ in recent]
    first = len(recent) - 1
    while first > 0 and thr is not None and f[first] > thr and f[first - 1] > thr:
        first -= 1                                   # the first reading of the run that tripped
    b = first - 1                                    # the reading before it: the hand as it arrived
    if b < 0:
        return None, False, None
    vel = _tip_speed(ex, recent[b][0])
    u = _unit(vel, _unit(travel) if travel is not None else DOWN)
    # the hand itself stopped: it lost more of its speed than its own trend before the contact took off, by the
    # relative share (a stop no thing made leaves the hand its speed)
    v_b, v_e = float(vel @ u), _tip_speed(ex, recent[-1][0], u)
    trend = max(0.0, _tip_speed(ex, recent[b - 1][0], u) - v_b) if b >= 1 else 0.0
    hand = v_b > 0.0 and (v_b - v_e) - trend * (len(recent) - 1 - b) > REL * v_b
    if stops is None:
        return None, hand, recent[b][0]
    tips = [ex.model.fk_tip(q)[0] for _, q in recent]
    plan_v = float(np.linalg.norm(tips[b] - tips[b - 1])) / ex.dt if b >= 1 else None
    q = recent[b][0].q
    model_z = getattr(ex.bounds, "model_z", None)
    strike = _module("bounds_strike")
    if model_z is not None:
        z = model_z(q, u)
    elif strike is not None and hasattr(strike, "model_z_of"):
        z = strike.model_z_of(ex.bounds, q, u)
    else:
        z = ex.bounds.slope(q, u)[0]
    samples = [{"qd": s.qd, "J": ex.model.jac(s.q)[:3], "f": f[i], "fast": s.fast} for i, (s, _) in enumerate(recent)]
    samples[-1]["ended"] = True
    return stops.measure(samples, thr, z, ex.dt, plan_v=plan_v), hand, recent[b][0]


def _peak_r4(ex, g):
    """(peak N, resolved) as round four read it, where no bounds_stops is there: the wrist's largest reading of
    the stop's period when the backend hands one, else the estimate's own peak, unresolved."""   # compat shim
    fast = getattr(ex._last, "fast", None) or getattr(g.first, "fast", None)
    wrist = fast.get("wrist_n") if isinstance(fast, dict) else None
    if wrist is None or not len(wrist):
        return float(g.peak), False
    mags = np.linalg.norm(np.asarray(wrist, dtype=float).reshape(len(wrist), -1), axis=1)
    over = int(np.sum(mags > g.force_n)) if g.force_n is not None else len(mags)
    return float(mags.max()), over >= QUANT


def stop_note(ex, g, kind, target, held, at_rest, expect=None, travel=None):
    """Tell the bounds a guard stop (B.note_stop) and read what it was: {"false": a stop no thing made (the
    bounds say so: the impulse inside the noise and the force gone at rest; and the hand kept its speed),
    "arrived": the stop is the contact the leg expected (arrived(); never for a false stop or a cap), "speed_m_s":
    the MEASURED tip speed of the reading before the first one over the threshold, "u", "peak_n" (from a force
    signal at the physics rate, else the largest period reading), "read_n", "resolved", "det_periods", "ran":
    what the result says of it, "stop": the measured record}."""
    B = ex.bounds
    first = g.first if g.first is not None else at_rest
    # a stop found STANDING (a hand that waited on a standing command and stands on something: control_tail)
    # has no measured arrival: it is no stop of a slope fit, and the hand itself is what stopped
    m, hand, before = (None, True, None) if getattr(g, "standing", False) else _measure(ex, g, travel)
    # the speed the result states is the hand's BEFORE the stop began (the reading before the first one of the
    # stop): a stop that counted no contact (a cap on its first reading) has no first reading of its own, and the
    # eased hand's speed is no arrival
    vel = _tip_speed(ex, before if before is not None and g.first is None else first)
    v, u = float(np.linalg.norm(vel)), _unit(vel, _unit(ex.model.fk_tip(first.q)[1][:, 0]))
    if m is not None and m.get("v"):
        v, u = float(m["v"]), np.asarray(m["u"], dtype=float)
    if m is not None:
        read = float(m["read_n"] if m.get("read_n") is not None else g.peak)
        peak, resolved = (read if m.get("peak_n") is None else float(m["peak_n"])), bool(m["resolved"])
    else:
        peak, resolved = _peak_r4(ex, g)
        read = float(g.peak)
        resolved = resolved and not getattr(g, "standing", False)
    out = {"false": False, "peak_n": peak, "read_n": read, "resolved": resolved, "speed_m_s": v, "u": u,
           "impulse_ns": float(g.impulse), "det_periods": None if m is None else m["det_periods"], "stop": m}
    ran = getattr(_module("bounds_strike"), "ran", None)
    out["ran"] = ran(m) if ran is not None and m is not None else (
        {"v_m_s": v, "peak_n": peak} if resolved else
        {"v_m_s": v, "read_n": read, "note": "the arm's reading once per period, not the peak"})
    if getattr(g, "standing", False):
        out["ran"]["note"] = ("found standing after the hand came to rest: the force it stands with; the speed "
                              "it arrived at was not measured")
    thing = ex._thing(target if not isinstance(target, list) else None)
    args = (thing, first.q, u, v, peak if m is None else m.get("peak_n"), float(g.impulse), resolved,
            float(g.felt.resisted))
    kw = {"held": bool(held), "rest_n": float(at_rest.Fmag), "ext_peak": g.ext_peak}
    try:
        if m is not None and "stop" in inspect.signature(B.note_stop).parameters:
            r = B.note_stop(*args, **kw, det_periods=m["det_periods"], expect=expect, stop=m)
        else:                                        # compat shim: round four's note_stop takes a number for the peak
            r = B.note_stop(*args[:4], peak, *args[5:], **kw)
    except Exception as exc:
        ex._journal("bounds_problem", where="note_stop", error=f"{type(exc).__name__}: {exc}")
        r = None
    said = _attr(r, "false_stop", _attr(r, "false")) if r is not None else None
    if said is None:   # the law itself (A1.1), in its relative form where the noise reads zero
        size = TWO_READ * max(_val(B.sigma_f(v, 0.0)), REL * float(g.force_n or 0.0))
        said = g.impulse <= size * _val(B.t_det()) and at_rest.Fmag <= _val(B.f_thr(0.0, 0.0))
    # a stop the thing the move was leaving made (over its stated limit, or growing) is never false
    over = any((getattr(g, "leave", None) or {}).get(k) is not None for k in ("over_n", "rose_n"))
    # a reading over what the drives can push along the travel, gone at rest, is false even where it
    # passed the cap (the cap read the same impossible figure)
    drives = _attr(r, "over_drives") if r is not None else None
    out["false"] = bool(said) and (g.cap_why == "" or drives is not None) and not hand and not over
    if drives is not None and out["false"]:
        out["over_drives"] = drives
    out["hand_stopped"] = bool(hand)
    # meant: the stop is the contact the leg expected (region, direction, where the push is), whatever its force;
    # arrived: meant, and under the cap (a meant contact that passed the cap ends "cap" and is still no thing in
    # the way)
    out["meant"] = bool(expect) and not out["false"] and arrived(
        region(ex, expect), g.contact, u if travel is None else travel)
    out["arrived"] = out["meant"] and g.cap_why == ""
    return out


def record_contact(ex, contact, source, label="contact", info=None):
    """Store a contact the tool made in the world, with the error its arrival speed gives it: the travel over
    one detection time at that speed plus the tip's resolution (a contact stored at speed is that much less
    sure). Not stored: a stop the tool did not make (source "arm or self": no measured place), a stop the
    bounds called false, a contact the caller expected ("arrived", or "meant" where it passed the cap: it is the
    thing meant, not a thing in the way; the contact says so under "on", which a package's own contact record
    reads), and a contact made while something is held (it goes on the hold: control_hold.note_contact)."""
    if contact is None:
        return None
    hold = _hold_module()
    holding = hold is not None and hold.held(ex) is not None
    if holding and not (info or {}).get("false"):
        hold.note_contact(ex, contact)
    meant = bool((info or {}).get("arrived") or (info or {}).get("meant"))
    if meant and not contact.get("on"):
        contact["on"] = "the thing this move meant to meet: not stored as a thing in the way"
    if (info or {}).get("over_drives"):
        contact["false_stop"] = info["over_drives"]["words"]   # said, and the world keeps nothing
    if contact.get("point_mm") is None or (info or {}).get("false") or meant or holding:
        contact["id"] = None
        return None
    B = ex.bounds
    v = float(contact.get("tip_speed_mm_s") or 0.0) / MM
    s = ex._last
    u = (info or {}).get("u")
    u = _unit(ex.model.fk_tip(s.q)[1][:, 0]) if u is None else u
    err = v * _val(B.t_det()) + _val(B.quantum(s.q, u))
    contact["error_mm"] = round(err * MM, 3)
    data = {"normal_est": contact["normal_est"], "force_n": contact["force_n"], "error_mm": contact["error_mm"],
            "sigma_p_mm": contact["error_mm"], "tip_speed_mm_s": contact.get("tip_speed_mm_s")}
    if info:
        data.update({"peak_n": round(float(info["peak_n"]), 2), "peak_resolved": bool(info["resolved"])})
        if not info["resolved"]:
            contact["peak_note"] = ("peak unresolved: the force reading is a mean over a control period and "
                                    "the backend gives no faster signal")
    e = ex.world.add("contact", label, point_mm=contact["point_mm"], source=source, data=data)
    contact["id"] = e.get("id")
    return e


def retreat(ex, steps=None, limit=None, need=None, until=None, intent=None):
    """Retrace the newest paths of the history, the way the arm came (the contact stop armed and directional:
    the force of leaving does not stop it). steps: a count the caller states; None: until every link is clear
    of every stored contact (_links_clear). Which paths that is is read off their own poses BEFORE anything
    moves; they run as ONE line with no stop between them, and its control periods are counted against what
    is left first (a retreat that cannot finish is refused budget). until(s): a caller's own test after
    each path; such a retreat runs path by path. When the history runs out before the arm is clear, the
    retreat says so (plans_on): the caller plans the rest as a move of its own."""
    s, _ = ex._ease_if_pressing()
    start = s.tip.copy()
    t0 = ex._periods_now()
    n = len(ex._history) if steps is None else max(0, min(int(steps), len(ex._history)))
    intent, notes = ex._intent(intent, limit=limit, need=need, kind="retreat")
    away = ex._last_contact_point()
    k, stop, contact, felt, plan, refused = 0, "goal", None, None, None, None
    runs = []                                   # the paths of each drive, newest first
    if until is not None:
        runs = [[path] for path in reversed(ex._history[len(ex._history) - n:])]
    else:
        take = []
        for path in reversed(ex._history[len(ex._history) - n:]):
            if steps is None and take and ex._links_clear(take[-1][0]):
                break                           # where the last path taken began, every link is clear
            take.append(path)
        runs = [take] if take else []
    for paths in runs:
        if until is not None and until(s):
            break
        q_from = ex._told().q
        way, marks = [], []
        for path in paths:
            way += ex._thin(way[-1] if way else q_from, [np.asarray(q) for q in reversed(path)])
            marks.append(len(way))              # waypoints up to the end of this path
        r = ex._drive(q_from, way, "retreat", intent=intent, away_from=away, record=False, notes=notes,
                      need_m=intent.get("tolerance"))
        if "refused" in r:
            refused = r
            break
        stop, s, plan, notes = r["stop"], r["s"], r["plan"], r["notes"]
        felt = r["felt"] if felt is None else felt.merge(r["felt"])
        done = len(paths)
        if stop != "goal":                      # the path it was on when it stopped is spent too
            idx = getattr(plan, "_segment_indices", None)
            at = int(idx[r["streamed"] - 1]) if idx is not None and 0 < r["streamed"] <= len(idx) else 0
            done = min(len(paths), sum(1 for m in marks if m <= at) + 1)
            contact = r["guard"].contact
        del ex._history[len(ex._history) - done:]
        k += done
        if stop != "goal":
            break
    if refused is not None:
        return {**refused, "retreated": k, "stopped": "refused", "tip_mm": _mm(s.tip)}
    out = {"retreated": k, "left_in_history": len(ex._history), "stopped": stop, "contact": contact,
           "moved_mm": round(float(np.linalg.norm(s.tip - start)) * MM, 1)}
    if steps is None and stop == "goal" and not ex._links_clear(s):
        out["plans_on"] = ("the history ran out before every link was clear of what the arm touched: move on "
                           "away from it (move_tip away from the contact, or park)")
    if felt is not None:
        out.update(felt.report())
    out.update(ex._move_keys(plan, ex._periods_now() - t0, notes))
    return out


def _leg_levels(ex, prev_q, seg, v_in):
    """(peak tip speed, peak tip acceleration, tip speed at the end) of one leg's samples, from the plan itself."""
    tips = np.asarray([ex.model.fk_tip(np.asarray(q, dtype=float))[0] for q in [prev_q] + list(seg)])
    speed = np.concatenate([[v_in], np.linalg.norm(np.diff(tips, axis=0), axis=1) / ex.dt])
    accel = np.abs(np.diff(speed)) / ex.dt
    return float(speed.max()), float(accel.max()) if len(accel) else 0.0, float(speed[-1])


def stream(ex, legs, *, held=None, looks=None, need_m=None, intent=None):
    """SpeedMixin._stream (see its text for the arguments and the result)."""
    dark = ex._dark_refusal_now()
    if dark is not None:
        return {**dark, "stopped": "refused"}
    legs, looks = [dict(leg) for leg in legs], [dict(lk) for lk in (looks or [])]
    gate = ex._selfknow_gate(legs[0].get("kind", "transit") if legs else "transit")
    if gate is not None:
        return {**gate, "stopped": "refused"}
    word = intent or next((leg.get("intent") for leg in legs if leg.get("intent")), None)
    word, notes = ex._intent(word)
    counts0 = dict(ex.periods)
    n_legs = len(legs)
    per_leg, ended, records = [0] * n_legs, [None] * n_legs, []
    out_stop, out_leg, first, false_stops = "goal", n_legs - 1, 0, 0
    felt, g, s, plan, refused, last_tip, info = None, None, None, None, None, None, None
    ex._ease_if_pressing()
    while first < n_legs:
        info = None                                 # what a stop of THIS piece showed
        q0 = ex._q_des.copy()                       # where the hand was told to be: the command stays continuous
        part = legs[first:]
        end_pose = np.asarray(part[-1]["to"], dtype=float)
        p_end = end_pose[:-1, -1] if end_pose.ndim > 1 else ex.model.fk_tip(end_pose)[0]
        ex._travel_direction = _unit(p_end - ex.model.fk_tip(q0)[0])
        moving = [dict(lk, leg=lk["leg"] - first) for lk in looks if lk["leg"] >= first and not lk.get("_taken")]
        plan = ex.bounds.plan_legs(q0, part, held=ex._held_for(held), looks=moving, intent=word or None)
        bad = _attr(plan, "refused")
        if bad is None:
            path = [np.asarray(q, dtype=float) for q in (_attr(plan, "path") or [])]
            # the plain budget, counted for every piece; a stated limit that makes it long is named with its price,
            # the same call without it and the least max_n that fits
            bad = ex._budget(len(path), ex._shorter(q0, path), _stream_price(ex, plan, word, q0, part, held, moving))
        if bad is not None:
            refused = bad if isinstance(bad, dict) and "refused" in bad else {"refused": bad}
            if false_stops == 0:                    # nothing ran: the refusal alone
                return {**refused, "stopped": "refused", "leg": first, "looks": records}
            out_stop, out_leg = "refused", first
            break
        ends = [int(e) for e in (_attr(plan, "legs") or [len(path) - 1] * len(part))]
        if false_stops == 0:
            start, prev = 0, q0
            for i, end in enumerate(ends):
                seg = path[start:end + 1]
                bad = ex._path_check([prev] + seg, kind=part[i].get("kind", "transit")) if seg else None
                if bad is not None:
                    return {**bad, "stopped": "refused", "leg": first + i, "looks": records}
                if seg:
                    prev, start = seg[-1], end + 1
        stations = {int(lk["period"]) - 1: lk for lk in (_attr(plan, "looks") or [])}
        start, prev, v_in, resume = 0, q0, 0.0, False
        for i, end in enumerate(ends):
            li, leg, seg = first + i, part[i], path[start:end + 1]
            if not seg:
                ended[li] = ended[li] or "goal"
                continue
            kind = leg.get("kind", "transit")
            v, a, v_in = _leg_levels(ex, prev, seg, v_in)
            ex._travel_direction = _unit(ex.model.fk_tip(seg[-1])[0] - ex.model.fk_tip(prev)[0])
            # the guard of THIS leg, from the hold as it stands now (a hold with no weight read is valid for one leg)
            spec = ex._guard_spec(kind, v=v, a=a, held=ex._held_for(held), away_from=leg.get("away_from"))
            if leg.get("away_from") is None:        # the contact this leg starts in: its guard is told
                spec.start = _module("control_tail").start_of(ex, kind, prev, seg, leg.get("intent") or word,
                                                              ex._held_for(held),
                                                              meets=kind in MEETS or bool(leg.get("expect")))
            g = Guard(ex, spec=spec)

            def hook(k, sense, base=start, off=first):
                look = stations.get(base + k)
                if look is not None:
                    look["_taken"] = True
                    for lk in looks:
                        if lk.get("leg") == look["leg"] + off and lk.get("s") == look.get("s"):
                            lk["_taken"] = True
                    records.append(ex._stream_look(dict(look, leg=look["leg"] + off), sense))

            final = li == n_legs - 1
            need = need_m if final else None
            if need is None and leg.get("expect"):  # a leg that means a contact ends under its own guard: the
                need = ex._band_at(seg[-1])         # last target is held until the hand stands still
            stop, s, _, n = ex._run(seg, g, need_m=need, held=ex._held_for(held), hook=hook, leg=False,
                                    close=final)     # only the stream's last leg ends where it says
            if _hold_module() is not None:          # every leg's end; leg None where the run ends with it
                _hold_module().after_move(ex, s, stop, leg=li if stop == "goal" and not final else None)
            per_leg[li] += n
            felt = g.felt if felt is None else felt.merge(g.felt)
            if stop == "goal":
                ended[li], prev, start = "goal", seg[-1], end + 1
                continue
            if stop in ("contact", "cap"):
                info = ex._stop_note(g, kind, leg.get("target"), ex._held_for(held), s, expect=leg.get("expect"),
                                     travel=ex._travel_direction)
                moved = last_tip is None or float(np.linalg.norm(s.tip - last_tip)) > _val(
                    ex.bounds.rigid(s.q, info["u"]))
                if stop == "contact" and info["false"] and moved:
                    false_stops, last_tip, first, resume = false_stops + 1, s.tip.copy(), li, True
                    break                           # nothing was there: the stream goes on from where it stands
                if info.get("arrived"):
                    stop = "arrived"
                ex._record_contact(g.contact, "stream guard", info=info)   # nothing stored for a contact meant
            out_stop, out_leg, ended[li] = stop, li, stop
            break
        if not resume:
            break
    used = {k: ex.periods[k] - counts0.get(k, 0) for k in ex.periods}
    s = s if s is not None else ex._sense()
    out = {"stopped": out_stop, "leg": out_leg, "ended": ended, "arrived": out_stop == "arrived", "periods": per_leg,
           "moving": used["moving"], "waiting": used["waiting"], "looks": records,
           "contact": None if g is None else g.contact, "tip_mm": _mm(s.tip), "s": s,
           **(felt if felt is not None else Felt(ex)).report(None if g is None else g.force_n)}
    if refused is not None:
        out.update(refused)
    if false_stops:
        out["false_stops"] = false_stops
    if out_stop == "cap":
        out["why"] = g.cap_why
    elif out_stop == "contact" and link_of(g.contact):
        out["why"] = f"the push is on {link_of(g.contact)}, not on the tool"
    keys = ex._move_keys(plan, sum(used.values()), notes, {"_ran": info["ran"]} if info else None)
    keys.pop("periods", None)
    keys.pop("tip_mm", None)
    out.update(keys)
    return out
