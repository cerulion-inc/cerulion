# SPDX-License-Identifier: AGPL-3.0-only
"""The only things that lower a motion: stated limits.

First part: pure functions over stated facts (gave, same_kind, stated_limit, solve, arrival); they read no robot,
no world and no scene. Second part: what gathers the facts for the two methods of rcpx.bounds.Bounds
(gather_limit, gather_arrival) and what a plan reads of them (expects, region, arrive, over, carried). SI inside.

stated_limit: the least of
  (1) intent: max_n or force_n on the call; gentle = the hand's own weight. The force_n of a TOUCH is not one of
      them: it is the force at which the touch ends, not a bound on the way in; the touch hands it
      over as ends_n (see "floor" below);
  (2) the sheet of the thing the call NAMES (a label or an id), when the session's device knowledge serves a
      force for it. A thing that is not named has no sheet;
  (4) a give of this session: this thing, or another thing that carries the same stated label, gave way at a
      known force. gave() says what a give is: only a thing the session holds as fixed that moved;
  (5) the site's force, the body's stated contact rating, and people: true (the site's force, else the hand's
      own weight) on every motion.
In free space (expected false) only intent, site and rating apply. With nothing stated the limit is None and
nothing lowers the motion.

arrival: the speed at which a contact under the limit is met: v solves F_guard(v) + Z x v = limit, with F_guard the
threshold the leg's guard really runs with and Z the stop slope at that pose. A limit over the guard's level runs
at that speed, also under the tell speed (the slowest speed at which this arm resolves a stop): a stated limit
is no reason to strike harder than it says, whoever states it. kept is true only when the slope
rests on resolved stops of this robot and the speed is one at which a stop is resolved; else the text says what
the limit is not shown kept by.
floor: the number is the force at which a touch ends (ends_n: the call's force_n, else the
hand's own weight, which the touch hands over with ends_default), not a limit. Its way in runs at the
speed solved for it and never under the tell speed: where the solved speed is under it the touch runs at the
tell speed, nothing is shown kept and the text states the strike a stop from that speed could make. A limit
somebody stated beside it (max_n, gentle, a sheet, a give, the site, a rating) is still kept: the way in never
runs faster than that limit's own arrival.
A limit that is not over the guard's level by more than the level's own relative error (REL) cannot be told from
no contact at all: the operator's own limit is acted on at the tell speed and said not shown kept, with the
strike a stop from that speed could make; a sheet's, a rating's, the site's or a give's is not sought by force
(seek false). Where no contact threshold stands (a level nobody measured, a hold whose load is not read yet) the
speed is limit / Z, nothing is shown kept, and no contact is sought by force (only the guard's cap would end
it). v_m_s is None only where no slope and no tell speed stand; the limit's item is carried by the plan either way.
"""

import math
import sys
from contextlib import contextmanager

import numpy as np

from rcpx import bounds_laws as laws
from rcpx.bounds_ratios import REL
from rcpx import bounds_report as report
from rcpx.bounds_report import MM
from rcpx.bounds_robot import Thing

NOT_SOUGHT = ("sheet", "rating", "site", "contact")   # a limit of these sources under the guard's level: seek false
HALVINGS = sys.float_info.mant_dig - sys.float_info.min_exp   # numeric: the most halvings a float interval can take
RESTS = {"contact": "resolved stops of this robot", "model": "the body's model, not measured"}


def _force(value):
    """A stated force as a number over 0, or None when it is no such number (malformed)."""
    try:
        value = None if isinstance(value, bool) else float(value)
    except (TypeError, ValueError):
        return None
    return value if value is not None and math.isfinite(value) and value > 0 else None


def gave(fixed, moved_m, rigid_m):
    """True when a stop is a give: a thing the session holds as fixed (the support, a fixture, a part
    past its stated travel) moved by more than the rigid travel the body can tell. A free thing that moved under
    a pick, a push, a set down or an expected contact meant to take it is not held as fixed and records no give."""
    return bool(fixed and moved_m is not None and moved_m > rigid_m)


def same_kind(give, thing):
    """A give binds the thing it happened to, and things that carry the same stated label; nothing else."""
    thing = thing or {}
    if give.get("id") is not None and give.get("id") == thing.get("id"):
        return True
    label = give.get("label")
    return label is not None and label == thing.get("label")


def stated_limit(intent=None, expected=False, thing=None, sheet=None, gives=(), site_force_n=None, people=False,
                 rating_n=None, f_hand_n=None):
    """{"limit_n": N or None, "source": str, "evidence": str}: the least stated limit on a contact (expected true:
    with the thing {"id", "label"} when the call names one) or on a free motion (expected false).
    sheet: {"max_n", "what"} of the named thing as the session serves it, or None. gives: [{"id", "label",
    "force_n", "why"}] of this session. A stated force that is no number over 0 N is no limit: it is left out
    and named under "malformed" (with "malformed_check": args for the call's own, site for the site's), so that
    the plan refuses the call for it."""
    intent = intent or {}
    terms, bad = [], None
    ends = None                             # the force at which a touch ends: no limit, see below
    if intent.get("ends_n") is not None:
        ends = _force(intent["ends_n"])
        if ends is None:
            bad = (f"force_n = {intent['ends_n']!r} is not a force over 0 N", "args")
    for key in ("max_n", "force_n"):
        if intent.get(key) is not None:
            value = _force(intent[key])
            if value is None:           # a malformed argument: the caller refuses the call for it
                bad = (f"{key} = {intent[key]!r} is not a force over 0 N", "args")
                continue
            terms.append((value, "intent", f"on your word: {key} = {value:g} N", key == "force_n"))
    f_hand_n = _force(f_hand_n)
    if intent.get("gentle") and f_hand_n is not None:
        terms.append((f_hand_n, "intent", f"your word gentle: the hand's own weight ({f_hand_n:g} N)", False))
    if expected:
        high = _force((sheet or {}).get("max_n"))
        if high is not None:
            terms.append((high, "sheet", str(sheet.get("what") or "the device sheet states it"), False))
        for give in gives or ():
            force = _force(give.get("force_n"))
            if force is not None and same_kind(give, thing):
                own = give.get("id") == (thing or {}).get("id")
                who = "it" if own else f"another thing with the same stated label ({give.get('label')})"
                terms.append((force, "contact", f"{who} gave way at {force:g} N this session ({give.get('why')})",
                              False))
    site_n, rating_n = _force(site_force_n), _force(rating_n)
    if site_n is not None:
        terms.append((site_n, "site", f"the site's stated force ({site_n:g} N)", False))
    elif site_force_n is not None:
        bad = bad or (f"the site's stated force {site_force_n!r} is not a force over 0 N", "site")
    if rating_n is not None:
        terms.append((rating_n, "rating", f"the body's stated contact rating ({rating_n:g} N)", False))
    if people and site_n is None and f_hand_n is not None:
        terms.append((f_hand_n, "site", f"people: true with no site force stated: the hand's own weight "
                                        f"({f_hand_n:g} N)", False))
    if not terms and ends is None:
        what = "for this thing" if expected else "for free space"
        out = {"limit_n": None, "source": "none", "evidence": f"nothing is stated {what}: no force on the call, "
               + ("no sheet the session serves for a thing this call names, no give this session, " if expected else "")
               + "no site force, no contact rating"}
        if bad:
            out.update(malformed=bad[0], malformed_check=bad[1], evidence=f"{bad[0]}; {out['evidence']}")
        return out
    limit, source, evidence, _ = min(terms, key=lambda t: t[0]) if terms else (None, None, None, None)
    # the least LIMIT among them (never over this, the strike of the way in included): everything stated
    # but a force_n (each term's last entry says so), which is the force a stroke pushes with or a touch ends at;
    # the least limit is what holds the arm's links
    limits = [t for t in terms if not t[-1]]
    hold = None
    if limits:
        hold_n, hold_source, hold_why, _ = min(limits, key=lambda t: t[0])
        hold = {"limit_n": hold_n, "source": hold_source, "evidence": hold_why}
    if people and source == "site":
        evidence += "; people: true"
    if ends is not None and (limit is None or ends < limit):
        # the touch's own force_n is the least number stated: the way in is timed for it and floored at the tell
        # speed; "floor" carries the least LIMIT stated beside it (None, None with none), which is still kept
        said = (str(intent["ends_said"]) if intent.get("ends_said") else
                f"no force_n stated: this touch ends at the hand's own weight, {ends:g} N on this arm (no bound "
                "on the way in)" if intent.get("ends_default") else
                f"on your word: force_n = {ends:g} N, the force at which this touch ends (no bound on the way in)")
        out = {"limit_n": ends, "source": "intent", "floor": (limit, source), "hold": hold,
               "evidence": said if limit is None else f"{said}; beside it {evidence}"}
    else:
        out = {"limit_n": limit, "source": source, "evidence": evidence, "hold": hold}
    if bad:
        out.update(malformed=bad[0], malformed_check=bad[1])
    return out


def carried(held):
    """The limit somebody stated for the thing held (the hold's record: limit_n and limit_source), as a
    stated limit on every leg that carries it: {"limit_n", "source", "evidence"}, or None when the
    record states none."""
    value = _force(held.get("limit_n")) if isinstance(held, dict) else None
    if value is None:
        return None
    said = str(held.get("limit_source") or "")
    source = next((word for word in NOT_SOUGHT if word in said), "intent")
    return {"limit_n": value, "source": source,
            "evidence": f"the limit stated for the thing held ({said or 'no source said'}): {value:g} N"}


def solve(limit_n, f_guard, z):
    """The largest v with F_guard(v) + Z x v <= limit, or None when no stop slope is known, the limit is no finite
    number or it is at or under the guard's level at rest. f_guard: a number or a function of v that does not
    fall with v."""
    f = f_guard if callable(f_guard) else (lambda v: f_guard)
    if z is None or not z > 0 or not math.isfinite(z) or not math.isfinite(limit_n) or not limit_n > f(0.0):
        return None
    lo, hi = 0.0, (limit_n - f(0.0)) / z
    if f(hi) + z * hi <= limit_n:
        return hi
    for _ in range(HALVINGS):
        mid = lo + (hi - lo) / laws.TWO
        if mid <= lo or mid >= hi:
            break
        if f(mid) + z * mid <= limit_n:
            lo = mid
        else:
            hi = mid
    return lo


def arrival(limit_n, source, f_guard, z, v_tell, shown_by=None, v_most=None, floor=False):
    """{"v_m_s", "kept", "shown_by", "seek", "strike_n", "f_guard_n", "text"} for a contact under limit_n (N) of
    that source. f_guard: the threshold the guard really runs with on that leg (a number or a function of v; inf:
    no contact threshold stands); z: the stop slope N per (m/s) or None; v_tell: the tell speed m/s or None (the
    body resolves every motion); shown_by: what the slope rests on ("contact": resolved stops, "model": the
    body's model, not measured); v_most: a speed the answer never passes (the same arrival at rest, for a leg
    whose guard runs over it). floor: limit_n is the force at which a touch ends, not a limit: its solved
    speed never runs under the tell speed. strike_n: what a stop from v_m_s makes by that level and slope (None
    where one of them is not known). v_m_s None: nothing bounds the speed (no slope and no tell speed)."""
    f = f_guard if callable(f_guard) else (lambda v: f_guard)
    level = f(0.0)
    sloped = z is not None and math.isfinite(z) and z > 0
    rests = RESTS.get(shown_by, "no source")
    cut = lambda v: v if v is None or v_most is None else min(v, v_most)
    mm = lambda v: "its own speed" if v is None else f"{v * MM:.3g} mm/s"

    def out(v, kept, seek, text):
        at = f(v or 0.0)
        strike = at + z * v if sloped and v is not None and math.isfinite(at) else None
        if strike is not None and strike > limit_n * (1 + laws.ROUND):
            text += (f"; a stop from {mm(v)} could reach {strike:.3g} N (the guard's level {at:.3g} N plus the "
                     f"stop slope: {rests})")
        return {"v_m_s": v, "kept": bool(kept), "shown_by": shown_by if sloped else None, "seek": seek,
                "strike_n": strike, "f_guard_n": at, "text": text}

    if not math.isfinite(level):
        # no contact threshold stands: the guard tells no contact under its cap, so nothing is shown kept and a
        # contact cannot be sought by force without pressing to that cap (whoever states the limit). The limit
        # still lowers the motion: to the speed whose stop adds no more than the limit by the slope
        v = cut(limit_n / z if sloped else v_tell)
        how = (f"ran at {mm(v)}, where the stop slope ({rests}) alone gives {limit_n:g} N" if sloped
               else "no stop slope is known either: ran at " + ("its own speed" if v is None else f"the tell speed {mm(v)}"))
        return out(v, False, False, f"no contact threshold stands here (this arm's contact level is not measured "
                                    f"yet, or a hold's load is not read yet): only the guard's cap ends a contact, "
                                    f"so {limit_n:g} N is not shown kept and the contact is not sought by force; {how}")
    v_solved = solve(limit_n, f, z)
    if not limit_n > level * (1 + REL) or (sloped and not (v_solved or 0.0) > 0.0):
        # not over what the guard trips at by more than that level's own relative error (or no speed over 0
        # keeps it): it cannot be told from no contact
        v = cut(v_tell if v_tell is not None else (limit_n / z if sloped else None))
        tell = f"what this arm can tell ({level:.3g} N, a level measured to {REL:.0%})"
        if source in NOT_SOUGHT:
            return out(v, False, False, f"the limit of {limit_n:g} N (source {source}) is not over {tell}: the "
                                        "contact is not sought by force")
        return out(v, False, True, f"your limit of {limit_n:g} N is not over {tell}: ran at {mm(v)}; the limit is "
                                   "not shown kept")
    if not sloped:
        v = cut(v_tell)
        speed = "the body's own speed" if v is None else f"the tell speed {mm(v)}"
        return out(v, False, True, f"no stop slope is known here, so no speed is shown to keep {limit_n:g} N: {speed}")
    resolved = v_tell is None or v_solved >= v_tell
    if floor and not resolved:
        # the force at which a touch ends: its way in never runs under the tell speed, the slowest speed
        # at which this arm resolves a stop. Nothing is shown kept; out() states the strike a stop from that
        # speed makes
        v = cut(v_tell)
        return out(v, False, True, f"{limit_n:g} N would need {mm(v_solved)}, under the slowest speed at which "
                                   f"this arm resolves a stop ({mm(v_tell)}): ran at {mm(v)}; the limit is not "
                                   "shown kept")
    v = cut(v_solved)
    if shown_by == "contact" and resolved:
        return out(v, True, True, f"a stop from {mm(v)} stays at or under {limit_n:g} N (the guard's level "
                                  f"{f(v):.3g} N plus the stop slope: {rests})")
    why = ([] if shown_by == "contact" else [f"the stop slope is {rests}"]) + ([] if resolved else [
        f"that speed is under the slowest at which this arm resolves a stop ({mm(v_tell)}), so no stop of this "
        "robot shows it"])
    return out(v, False, True, f"ran at {mm(v)}, where a stop reaches {limit_n:g} N by the guard's level "
                               f"({f(v):.3g} N) and the stop slope; not shown kept: " + " and ".join(why))


# ---------------------------------------------------------------- gathering the facts (the two methods of Bounds)
def gather_limit(B, thing=None, intent=None, expected=False):
    """{"limit_n", "source", "evidence"}: the least limit somebody STATED on a contact with the thing
    (expected true) or on a free motion; limit_n None when nothing is stated. Gathers the facts for
    rcpx.bounds_limit.stated_limit: the call's intent, the sheet the session serves for a thing the call
    names, this session's gives, the site, the body's stated rating, people."""
    intent, site = intent or {}, B.site or {}
    ident, sheet = None, None
    if expected and thing is not None:
        entry = B._entry(thing) or {}
        ref = thing if isinstance(thing, str) else entry.get("id", getattr(thing, "id", None))
        label = intent.get("label") or entry.get("label")
        ident = {"id": entry.get("id", ref), "label": label}
        names = [n for n in (ref, label, (entry.get("data") or {}).get("part")) if n is not None]
        devices = (getattr(B.ex, "context", None) or {}).get("devices")
        from rcpx.context_devices import sheet_of
        sheet = sheet_of(devices() if callable(devices) else devices, names)
        own = B._sheet(Thing(id=entry.get("id")), intent).get("max_n") if entry else None
        if own is not None and (sheet is None or own < sheet["max_n"]):
            sheet = {"max_n": float(own), "what": f"the sheet this session holds for it states {float(own):g} N"}
    got = stated_limit(intent, expected, ident, sheet, B.gives, site.get("force_n"),
                             bool(B.ex.cfg.get("people")), B.robot.model.spec.get("tool_rating_n"), B.f_hand())
    return got


def gather_arrival(B, limit_n, source, q, u, held=None, f_guard_n=None, floor=None):
    """{"v_m_s", "kept", "shown_by", "seek", "strike_n", "f_guard_n", "text"}: the speed at which a contact
    under limit_n is met at q along u. The guard's level is the contact threshold at the arrival's own speed with
    the arm at rest, under a known hold no less than the hold's level; a hold whose load is not read yet (hold
    first) has no contact threshold on its one leg. f_guard_n: the threshold the leg's guard runs with, when
    the caller has derived that guard (a plan does, after it is timed: its fastest stretch and its largest
    acceleration set it). Over the level at rest the arrival is solved again with it, and never comes out
    faster than the arrival at rest. floor: None for a limit; for the force at which a touch ends the
    pair (limit_n, source) of the least limit stated beside it, (None, None) with none: the way in is floored
    at the tell speed and never runs faster than that limit's own arrival."""
    q, u = B._pose(q, u)
    hold = held if isinstance(held, dict) and "weight_n" in held else None
    if hold is not None and hold["weight_n"] is None:
        rest = lambda v: math.inf
    else:
        level = float(hold.get("level_n") or 0.0) if hold is not None else 0.0
        rest = lambda v: max(B.f_thr(v, 0.0, held=held), level)
    z, text = B.slope(q, u)
    shown = None if z is None else ("contact" if str(text).startswith("fitted") else "model")
    v_tell = laws.tell_speed(B.rigid(q, u, rest(0.0)), B.t_det(), B.a_brake(q, u))
    ends = floor is not None
    kept = None if not ends or floor[0] is None else arrival(floor[0], floor[1], rest, z, v_tell, shown)
    got = arrival(limit_n, source, rest, z, v_tell, shown, v_most=None if kept is None else kept["v_m_s"], floor=ends)
    if kept is not None and kept["v_m_s"] is not None and got["v_m_s"] == kept["v_m_s"]:
        got["text"] += f"; no faster than the limit stated beside it allows ({floor[0]:g} N, source {floor[1]})"
    if f_guard_n is None or not float(f_guard_n) > got["f_guard_n"]:
        return got
    leg = arrival(limit_n, source, lambda v: max(rest(v), float(f_guard_n)), z, v_tell, shown, v_most=got["v_m_s"],
                  floor=ends)
    leg["text"] += (f"; the guard of this leg trips at {float(f_guard_n):.3g} N (the threshold at its fastest speed "
                    f"and its largest acceleration), over the {got['f_guard_n']:.3g} N of the arm at rest")
    return leg


@contextmanager
def moving_point(B, q, point, qd=None):
    """B's bounds read at ONE moving point of the arm (bounds_robot.point_model) while it moves with the joint
    speeds qd (any scale; None: not known): there a link's model slope is the energy bound at those joint speeds
    (rcpx/bounds_know_point.py terms), with no typed share."""
    from rcpx.bounds_robot import point_model
    with point_model(B, q, point):
        was, B._point_qd = getattr(B, "_point_qd", None), None if qd is None else np.asarray(qd, dtype=float)
        try:
            yield
        finally:
            B._point_qd = was


def link_arrival(B, q, point, u, limit_n, source, held=None, f_guard_n=None, qd=None):
    """The arrival of ONE moving point of the arm that is no tool point (a link) under a stated limit: B.arrival's
    answer with that point's own Jacobian, so with its own stop slope along its own travel u (a
    link's strike is held under a stated limit; a limit stated on a call binds every leg of it): the energy
    bound at the leg's joint step qd (moving_point). The contact level is the arm's own (the guard reads one
    estimate for the whole arm)."""
    with moving_point(B, q, point, qd):
        return gather_arrival(B, limit_n, source, q, u, held, f_guard_n)


def hold_of(stated):
    """The LIMIT in what stated_limit (or carried) answers, {limit_n, source, evidence}, or None: the least of
    max_n, gentle, a sheet, the site, a rating, people, a give, the limit of a thing held (never
    over this, the strike of the way in included). A force_n is none: it is the force a stroke pushes with or a
    touch ends at and holds no link."""
    if "hold" in stated:
        return stated["hold"]
    return stated if stated.get("limit_n") is not None and "floor" not in stated else None


def links_under(B, links, q, L, free_limit, held=None, f_guard_n=None, dq=None):
    """[(cap in the path coordinate, kept, the bound's item)] for the arm's links on one path segment under the
    LIMIT stated on free space (hold_of): every moving point of each link body runs no faster than link_arrival's
    speed for it, solved by the energy bound at the segment's joint step dq (with the energy bound a slow point
    near a joint can bind before the fastest one); one entry per body, its least cap. links: [(the point's travel
    on the segment m, the point, its unit travel)]; L: the segment's own length (the fastest point's travel).
    Empty when no LIMIT is stated."""
    lim = hold_of(free_limit)
    if lim is None or not links:
        return []
    least = {}
    for d_link, point, u_link in links:
        got = link_arrival(B, q, point, u_link, lim["limit_n"], lim["source"], held, f_guard_n, dq)
        if got["v_m_s"] is None or not d_link > 0:
            continue
        cap = got["v_m_s"] * L / d_link
        if point.body not in least or cap < least[point.body][0]:
            least[point.body] = (cap, bool(got["kept"]), report.item(
                "speed", got["v_m_s"], lim["source"], lim["evidence"] + "; a stated limit binds the arm's "
                f"links too: {point.name} runs no faster than this; " + got["text"], "state another limit, or none"))
    return list(least.values())


def whole(free_limit, shown):
    """The LIMIT somebody stated on a motion as a whole (tool and links), for the receipt's limit block:
    {limit_n, source, shown: every arrival the plan solved under it is shown kept}; None when no LIMIT is stated
    (hold_of)."""
    lim = hold_of(free_limit)
    if lim is None:
        return None
    return {"limit_n": float(lim["limit_n"]), "source": lim["source"], "shown": bool(shown) and all(shown)}


# ---------------------------------------------------------------- what a plan reads (rcpx/bounds_plan.py)
def expects(B, expect, intent, kinds, targets):
    """One expected contact per path segment, or None. expect: a list with one entry per segment, or one
    dict for the last stretch; with none given, intent expect "contact" (or a dict there) marks the last stretch
    (the trailing segments of the last kind and target) with the bound B.stated_limit states for its target."""
    n = len(kinds)
    if isinstance(expect, list) and len(expect) == n:
        return list(expect)
    one = expect if isinstance(expect, dict) else intent.get("expect")
    if one == "contact":
        thing = targets[-1] if isinstance(targets[-1], (str, dict)) or hasattr(targets[-1], "id") else None
        got = B.stated_limit(thing, intent, expected=True)
        one = {"with": getattr(thing, "id", thing) if not isinstance(thing, dict) else thing.get("id"), "radius_m": None,
               "along": None, "bound_n": got["limit_n"], "bound_source": got["source"] if got["limit_n"] is not None else None,
               "seek": True, "evidence": got["evidence"], "floor": got.get("floor")}
    if not isinstance(one, dict):
        return [None] * n
    first = n - 1
    while first > 0 and kinds[first - 1] == kinds[-1] and targets[first - 1] is targets[-1]:
        first -= 1
    return [None] * first + [one] * (n - first)


def region(B, expect):
    """(the Thing or point an expected contact is with, the region's radius m); (None, 0) for whatever lies ahead.
    The radius is what the caller states (the spot's own error plus the tool's half width; with none stated,
    the tool's half width) plus BAND times the thing's own measured error: a thing may stand that much nearer
    than its stored place, and the limit's speed must hold where it is met."""
    ref = expect.get("with")
    if ref is None:
        return None, 0.0
    where = B.thing(ref) if isinstance(ref, (str, dict)) else np.asarray(ref, dtype=float)
    band = laws.band(where.sigma_p) if isinstance(where, Thing) and where.sigma_p else 0.0
    stated = expect.get("radius_m")
    return where, (B.robot.w_tool() if stated is None else float(stated)) + band


def arrive(B, cache, q, u, limit_n, source, held, f_guard_n=None, floor=None):
    """B.arrival for a limit at the pose q along u, asked once per pose and direction of a plan. The level and the
    slope change along a line and with the direction, so no pose's speed serves another."""
    q, u = np.asarray(q, dtype=float), np.asarray(u, dtype=float)
    key = (float(limit_n), source, u.tobytes(), q.tobytes(), floor)
    if key not in cache:
        got = dict(B.arrival(limit_n, source, q, u, held, floor) if f_guard_n is None
                   else gather_arrival(B, limit_n, source, q, u, held, f_guard_n, floor))
        brake = B.a_brake(q, u)
        got["dec"] = laws.a_dec(lambda a: B.f_thr(got["v_m_s"] or 0.0, a, held=held), limit_n, brake) or brake
        cache[key] = got
    return cache[key]


def over(B, cache, q0, q1, u, limit_n, source, held, f_guard_n=None, floor=None):
    """The arrival a stretch from q0 to q1 runs under: B.arrival at both its ends, the slower of the two; seek only
    when both ends seek."""
    ends = [arrive(B, cache, q, u, limit_n, source, held, f_guard_n, floor) for q in (q0, q1)]
    known = [g for g in ends if g["v_m_s"] is not None]
    got = dict(min(known, key=lambda g: g["v_m_s"]) if known else ends[0])
    got["seek"] = all(g["seek"] for g in ends)
    got["dec"] = min(g["dec"] for g in ends)
    return got


def assumed(cache):
    """The least guard level the arrivals of a plan were solved with (None: no arrival was asked)."""
    return min((g["f_guard_n"] for g in cache.values()), default=None)


def sought(expect, seeks):
    """The expect a plan hands to its guard: the caller's own, with seek false when the arrival under its
    bound says the contact is not sought by force (a sheet's, a rating's, the site's or a give's limit at or under
    the guard's level): the leg then ends at its planned height."""
    if expect is None or seeks.get(id(expect), True) or not expect.get("seek", True):
        return expect
    return {**expect, "seek": False}


def gentle_fact(B, q, u, held, path_m, time_s):
    """What the price is told of the word gentle: the force a stop makes at the speed gentle would run here
    (the hand's own weight where that speed keeps it; more where gentle runs at the tell speed, said as it is),
    and that speed under the key v_tell."""
    hand = B.f_hand()
    got = B.arrival(hand, "intent", q, u, held)
    force = got["strike_n"] if got["strike_n"] is not None else hand
    return {"force_n": max(force, hand), "path_m": path_m, "time_s": time_s, "v_tell": got["v_m_s"]}
