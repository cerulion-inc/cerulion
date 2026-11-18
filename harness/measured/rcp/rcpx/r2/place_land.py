# SPDX-License-Identifier: AGPL-3.0-only
"""The landing of a held thing.

A set down is fast to the band where a contact can begin and arrives inside it at the speed SOLVED for the force the
landing may make, never under the tell speed (the same law as a touch):

- ends(): the force the landing may make. A limit somebody stated for the thing (the call's word, a served sheet,
  the site, a rating, a give, the limit the hold's record carries) replaces everything else and is kept. With
  nothing stated it is what the thing already bears: the hold's own stated squeeze per pad, an ENDING force
  (expect["floor"] present; the receipt never calls it a limit). Where no squeeze stands (nothing is held by
  jaws that state one) nothing slows the landing.
- band(): where a contact can begin: about the tip height at which the thing's bottom reaches what the place's look
  shows, by BAND (laws.band) of the two stated errors together: the place's own height error (its Seen sigma_z; for
  a point on the stated support the support's stated error) and the error of the held thing's bottom under the tip
  (the take's own figure). Into an opening whose floor no look shows, a contact can begin anywhere under the rim.
  No stated error for the place: no band; the whole way down is the arrival stretch and the receipt prices it.
  Never a typed length.
- the hand-over: the fast stretch ends its COMMAND above the band's upper edge by how
  far this body's hand runs on past a command that comes to stand (place_land_watch.RunOn: measured on this
  session's own free stops, as a time, times the stretch's fastest speed), and where the hand is still faster
  than the arrival speed when that command stands it waits by motion alone (C10's wait, guarded). So the HAND,
  not its command, is at the arrival speed at the band's upper edge. A load that rose on the fast stretch or in
  that wait and still stands is the landing: no arrival stretch is armed on it (its guard would take that load
  for its baseline). The arrival stretch runs on to the band's LOWER edge (the place can lie that far under what
  its look or its statement says), never under the hand's own stop over what a look shows beside the place.
- figures(): the set down's ONE strike over every pair of bodies the landing loads: the thing on what it
  lands on, and each pad on the thing as a rise over the squeeze that stood: the arm's own figure along the travel
  at that speed (rcpx/bounds_strike.py: C2) plus the pads' rise by the lever law of rcpx/control_hold_pads.py. Two
  figures before the motion (fast_n: a contact outside the band at the plan's fastest sample; arrive_n: the band's
  figure at the arrival speed) and ONE after the stop that covers the whole way down (both stretches and the
  wait between them): for the largest of the speed the stop itself measured, the hand's measured speed in the
  control period before a load first rose on the arm's or the jaws' own reading (place_land_watch.LandWatch;
  said when no guard told that load) and, where nothing was told and nothing rose, the fastest the hand ran at
  or under the band's upper edge; never under what the arm itself read, nor under the strike of a thing caught
  again between the pads. Under a stated limit the receipt says PASSED whenever the arm's figure along the
  travel at that speed is over the limit.

Nothing here reads ground truth or names a robot. SI inside (m, s, N); mm in the receipt's block.
"""

import math

import numpy as np

from rcpx import bounds_laws as laws
from rcpx import bounds_ratios as R
from rcpx import control_hold
from rcpx import control_hold_pads as pads_of
from rcpx.r2.place_hand import finite
from rcpx.r2.place_land_said import SaidMixin, cost, let_go  # noqa: F401  (let_go: place_set asks it here)
from rcpx.r2.place_land_watch import LandWatch, run_on


def bounds_of(ex):
    from rcpx import bounds
    return bounds.of(ex)

MM = 1000.0
DOWN = np.array([0.0, 0.0, -1.0])
ENDS = "the hold's own squeeze per pad: what the held thing already bears"


def ends(ex, B, label, intent, above=False):
    """(the stated limit of the place, as B.stated_limit answers; the landing's force {"n", "source", "is": "limit"
    | "ends", "floor"} or None: nothing slows the landing; the pads' numbers or None). above: the thing is let go
    above the place on the operator's word: no contact is meant on the way down and nothing slows it."""
    limit = B.stated_limit(label, intent or None, expected=True)
    rec = control_hold.held(ex)
    pads = pads_of.of_record(rec) if rec is not None else pads_of.planned(ex)
    if limit.get("limit_n") is not None:
        return limit, {"n": float(limit["limit_n"]), "source": limit.get("source"), "is": "limit", "floor": None,
                       "evidence": limit.get("evidence")}, pads
    sq = None if pads is None else pads.get("squeeze_pad_n")
    if above or not finite(sq) or not float(sq) > 0.0:
        return limit, None, pads
    got = B.stated_limit(label, dict(intent or {}, ends_n=float(sq)), expected=True)
    if got.get("limit_n") is None or got.get("malformed"):
        return limit, None, pads
    return limit, {"n": float(got["limit_n"]), "source": "intent", "is": "ends", "floor": got.get("floor") or (None, None),
                   "evidence": f"with nothing stated the landing ends at {ENDS} ({float(sq):.3g} N, "
                               f"{pads.get('squeeze_pad_source')}); it is no limit"}, pads


def band(target, facts, z_end, into, support=None, sig_b=None, res=0.0, z_low=None, early=None):
    """Where a contact can begin on the way down to the tip height z_end: {"top_z": the tip height of the band's
    upper edge, "low_z": of its lower edge, "half_m", "sigma_m", "source"}; None for a way down with no end
    height. The band lies about the end the plan really lowers the thing's bottom to (z_low: the plan's end
    before a hand's stop raised it; a place typed under the stated support is lowered to the support, and the
    band lies about the support). The place's height error: its Seen sigma_z; for a place with none on the
    stated support, or one the plan lowered to the support, the support's stated error (support: (height, sigma,
    source)); else none is stated and the place is taken as given (said). The error of the held thing's bottom
    under the tip: sig_b, the take's own figure, never under the tip's own resolution res (the hang was read by
    the tip's own height). The two add in quadrature; the half depth is laws.band of the sum. early: (m, n) how
    far before their band this session's own landings met what they landed on (the largest of n): the upper
    edge is raised by that much (evidence of this session that the thing hangs lower, or the places lie higher,
    than was stated)."""
    if z_end is None:
        return None
    below = float(facts["below_m"])
    sig, src = target.get("sigma_z"), "the place's own look"
    base = target.get("surface_z")
    level = None if into or z_low is None else float(z_low) - below        # where the plan puts the bottom
    on = support is not None and finite(support[0], support[1])
    raised = on and level is not None and finite(base) and level > float(base) + float(res or 0.0) \
        and abs(level - float(support[0])) <= laws.band(float(support[1])) + float(res or 0.0)
    if raised or (not finite(sig) and on and finite(base)
                  and abs(float(base) - float(support[0])) <= laws.band(float(support[1]))):
        sig, src = float(support[1]), f"the stated support ({support[2] or 'stated'})" + (
            ": the place lies under it and the thing is lowered to the support" if raised else "")
    if not finite(sig):
        sig, src = 0.0, "the place as given (no error is stated for its height)"
    hang = max(float(sig_b) if finite(sig_b) else 0.0, float(res or 0.0))
    src += (" and the take's own reading of the thing's bottom under the tip" if finite(sig_b) and float(sig_b) >= float(res or 0.0)
            else " and the tip's own resolution (the take stated no error for the thing's bottom)")
    sigma = math.hypot(float(sig), hang)
    half = float(laws.band(sigma))
    seen_z = target.get("seen_z")
    if into and not finite(seen_z):
        rim = target.get("rim_z")
        if not finite(rim):
            return None
        top, src = float(rim) + below + half, src + ": the floor inside is not seen, a contact can begin anywhere under the rim"
        low = float(z_end)
    else:
        if into:
            level = float(seen_z)
        elif level is None:
            level = float(base if finite(base) else target["centre"][2])
        top, low = level + below + half, level + below - half
    out = {"top_z": top, "low_z": low, "half_m": half, "sigma_m": sigma, "source": src, "early_m": 0.0}
    if early is not None and finite(early[0]) and float(early[0]) > 0.0:
        out.update(top_z=top + float(early[0]), early_m=float(early[0]),
                   source=src + f"; its upper edge raised {float(early[0]) * MM:.1f} mm: that far before their "
                                f"band {int(early[1])} landing(s) of this session met what they landed on")
    return out


def expects(sigma_xy, half_width, land):
    """(the C9 record of the fast stretch, the C9 record of the arrival stretch or None). The fast stretch is an
    expected contact with nothing that lowers it (a thing met there is met at the speed that runs); the
    arrival stretch carries the landing's force as its bound (land: ends()'s force; None: nothing slows the
    landing and there is no arrival stretch), an ENDING force with its floor when nothing is stated."""
    fast = {"with": None, "radius_m": (float(sigma_xy) if finite(sigma_xy) else 0.0) + float(half_width),
            "along": DOWN.copy(), "bound_n": None, "bound_source": None, "seek": True}
    if land is None:
        return fast, None
    slow = dict(fast, bound_n=land["n"], bound_source=land["source"], evidence=land.get("evidence"))
    if land["is"] == "ends":
        slow["floor"] = tuple(land["floor"])
    return fast, slow


def _priced(B, q, v, f_guard):
    """C2's own figure for one sample at the pose q moving down at v under the guard level f_guard (N): what
    rcpx/bounds_strike.py price states for it (above the fastest stop measured: its bound where it states one),
    or None where it states none."""
    try:
        from rcpx import bounds_strike
        stops = B._optional("bounds_stops")
        got = bounds_strike.price([{"v": float(v), "model_z": B.model_z(q, DOWN), "f_guard": float(f_guard),
                                    "m_eff": B.m_eff(q, DOWN), "u": DOWN}],
                                  None if stops is None else stops.fit(B._stops()), B.t_det())
    except (AttributeError, KeyError, TypeError, ValueError) as exc:     # said in the journal, never silent
        ex = getattr(B, "ex", None)
        if callable(getattr(ex, "_journal", None)):
            ex._journal("land_problem", where="the plan's own price of a landing sample",
                        error=f"{type(exc).__name__}: {exc}")
        return None
    peak = got.get("peak_n") if isinstance(got, dict) else None
    return float(peak) if finite(peak) else None


def _arm(B, q, v, held):
    """The arm's own figure along the travel for a contact met at v (N): the threshold its guard runs with there
    (under a known hold no less than the hold's level) plus the stop slope at that pose times v, and never under
    what the plan's own price states for that sample; None with no slope."""
    z, text = B.slope(q, DOWN)
    if z is None or not finite(z):
        return None, None
    level = float(held.get("level_n") or 0.0) if isinstance(held, dict) and held.get("weight_n") is not None else 0.0
    thr = B.f_thr(float(v), 0.0, held=held)
    f = max(float(thr), level) if finite(thr) else level
    priced = _priced(B, q, v, f)
    return max(f + float(z) * float(v), priced or 0.0), str(text)


def block(ex, B, q, land, bnd, pads, z_from=None, z_end=None):
    """The receipt's band block as the plan knows it before anything moves: {"from_mm", "source",
    "sigma_mm", "half_mm", "fast_mm_s", "arrive_mm_s", "ends_n", "ends_source", "fast_n", "arrive_n", ...}, or None
    when nothing slows the landing, and beside it the unrounded figures {"v_arr", "arm_arr", "v_fast"}. q: the
    joints over the place; z_from: the tip height the way down starts at; z_end: where its fast stretch ends."""
    if land is None:
        return None, None
    held = control_hold.held(ex)
    arr = B.arrival(land["n"], land["source"], q, DOWN, held, land["floor"] if land["is"] == "ends" else None)
    v_free = float(B.v_free(q, DOWN))
    v_arr = v_free if arr.get("v_m_s") is None else min(v_free, float(arr["v_m_s"]))
    travel = None if z_from is None or z_end is None else max(0.0, float(z_from) - float(z_end))
    # the fastest the command can run on that stretch and still brake to its end (it cannot be faster there,
    # however quickly it speeds up): sqrt(2 x braking x travel), never over the body's own speed
    v_fast = v_free if travel is None else min(v_free, math.sqrt(max(0.0, laws.TWO * float(B.a_brake(q, DOWN)) * travel)))
    v_fast = max(v_fast, v_arr)
    arm_fast, slope = _arm(B, q, v_fast, held)
    arm_arr = arr.get("strike_n")
    if not finite(arm_arr):
        arm_arr = _arm(B, q, v_arr, held)[0]
    fast_n, _ = pads_of.rise(pads, arm_fast, v_fast)
    arrive_n, _ = pads_of.rise(pads, arm_arr, v_arr)
    out = {"from_mm": None if bnd is None else round(bnd["top_z"] * MM, 1),
           "source": "no end height is planned: the whole way down is the arrival stretch"
                     if bnd is None else bnd["source"],
           "sigma_mm": None if bnd is None else round(bnd["sigma_m"] * MM, 2),
           "half_mm": None if bnd is None else round(bnd["half_m"] * MM, 2),
           "fast_mm_s": round(v_fast * MM, 1), "arrive_mm_s": round(v_arr * MM, 2),
           "ends_n": round(float(land["n"]), 2),
           "ends_source": ENDS if land["is"] == "ends" else f"a stated limit ({land['source']}): {land.get('evidence')}",
           "ends_is": land["is"], "fast_n": None if fast_n is None else round(fast_n, 1),
           "arrive_n": None if arrive_n is None else round(arrive_n, 1),
           "arm_fast_n": None if arm_fast is None else round(float(arm_fast), 1),
           "arm_arrive_n": None if not finite(arm_arr) else round(float(arm_arr), 1),
           "arrival": str(arr.get("text") or ""), "slope": slope,
           "figure": "one figure for every pair the landing loads (the thing on what it lands on, each pad on the "
                     "thing over the squeeze that stood): the arm's own figure along the travel at that speed "
                     "plus the pads' rise, sqrt(arm figure x the jaws' stop slope x speed)"}
    if land["is"] == "ends":
        out["kept"] = None          # an ending force is no limit: nothing is kept or passed
    else:
        out["shown_kept"] = bool(arr.get("kept"))      # the arrival's own answer: shown by resolved stops
    return out, {"v_arr": v_arr, "arm_arr": float(arm_arr) if finite(arm_arr) else None, "v_fast": v_fast}


def after(band_block, r, pads, watch=None, arm_fn=None, exact=None):
    """The set down's strike block after the way down ran (r: the result of the stretch that ended it, with
    "fast": the fast stretch's own result when the arrival stretch ran after it): the plan's strike with the
    band, the two figures before the motion and ONE figure after the stop under "ran", which covers the whole
    way down. watch: what place_land_watch.LandWatch read over both stretches; arm_fn(v): the arm's own figure
    along the travel for a contact met at v (N). The speed the figure is for, the largest of:
    the speed the stop itself measured, when the way ended on a contact; the hand's measured speed in the
    control period before a load first rose on the arm's or the jaws' own reading (said when no guard told that
    load); and, when nothing was told and nothing rose, the fastest the hand ran at or under the band's upper
    edge, never under the arrival speed (with no band: the plan's fastest sample). The figure: the arm's figure
    along the travel at that speed, never under the stop's own figure nor under what the arm itself read, plus
    the pads' rise; never under the strike of a thing caught again between the pads."""
    end = dict(r.get("strike") or {}) if isinstance(r, dict) else {}
    first = r.get("fast") if isinstance(r, dict) and isinstance(r.get("fast"), dict) else None
    st = dict(first.get("strike") or {}) if first is not None else dict(end)   # the plan's fastest sample
    ran = dict(end.get("ran") or {})
    z, thr = end.get("z_n_s_m"), end.get("f_guard_n")
    plan_v, plan_peak = st.get("v_m_s"), st.get("peak_n")
    if not finite(plan_peak):
        plan_peak = st.get("model_peak_n")
    b = dict(band_block) if isinstance(band_block, dict) else None
    w = watch if isinstance(watch, dict) else {}

    def arm_at(v):
        got = [float(x) for x in ((arm_fn(v) if arm_fn is not None else None),
                                  ((float(thr) if finite(thr) else 0.0) + float(z) * float(v)) if finite(z, v) else None)
               if finite(x)]
        return max(got) if got else None

    if b is not None and finite(plan_peak, plan_v):       # the plan's own fastest sample states the fast figure
        fast, _ = pads_of.rise(pads, float(plan_peak), float(plan_v))
        if fast is not None:
            b.update(fast_n=round(max(fast, float(b.get("fast_n") or 0.0)), 1), fast_mm_s=round(float(plan_v) * MM, 1),
                     arm_fast_n=round(float(plan_peak), 1))
    stopped = str(r.get("stopped")) if isinstance(r, dict) else ""
    contact = stopped in ("arrived", "contact", "cap")
    met, banded = w.get("met"), b is not None and b.get("from_mm") is not None and finite(b.get("arrive_mm_s"))
    exact = exact if isinstance(exact, dict) else {}
    v_arr = (float(exact["v_arr"]) if finite(exact.get("v_arr")) else float(b["arrive_mm_s"]) / MM) if banded else None
    arm_arr = exact.get("arm_arr") if finite(exact.get("arm_arr")) else (b.get("arm_arrive_n") if banded else None)
    speeds = []
    if contact and finite(ran.get("v_m_s")) and not (isinstance(r, dict) and r.get("untold")):
        speeds.append((float(ran["v_m_s"]), "the speed the contact was met at (the stop's own measure)"))
    if isinstance(met, dict) and finite(met.get("v_m_s")):
        speeds.append((float(met["v_m_s"]), "the hand's measured speed in the control period before a load first "
                       f"rose on {met.get('by')}" + ("" if contact and not r.get("untold") else
                                                    ": no guard told that load")))
    told = bool(speeds)
    if not told and banded:
        inside = float(w.get("v_band_m_s") or 0.0)
        speeds.append((max(v_arr, inside), "no contact was told: the arrival speed the way ran at inside the band"
                       if inside <= v_arr * (1.0 + R.REL) else
                       "no contact was told: the fastest the hand itself ran at or under the band's upper edge"))
    elif not told:
        speeds.append((float(plan_v) if finite(plan_v) else float(w.get("v_fast_m_s") or 0.0),
                       "the plan's fastest sample (no band: the way down ran as one stretch)"))
    v, what = max(speeds, key=lambda x: x[0])
    # the arm's figure along the travel at that speed (at or under the arrival speed never over the figure the
    # arrival was solved with: the same law); never under what the arm itself read, nor under the stop's own
    # peak where a force signal measured one (a stop that only bounds its peak from its readings' impulse states
    # no measurement: the law at the measured speed stands for it)
    law = arm_at(v)
    if banded and v <= v_arr and finite(arm_arr):
        law = float(arm_arr) if law is None else min(float(arm_arr), law)
    arms = [law, w.get("read_n")]
    if contact:
        arms += [ran.get("read_n"), None if "at most" in str(ran.get("note") or "") else ran.get("peak_n")]
    if not told and not banded:
        arms.append(plan_peak)
    arm = max([float(x) for x in arms if finite(x)] or [0.0])
    one, lever = pads_of.rise(pads, arm, v)
    again = pads_of.caught(pads, float(w.get("gap_m_s") or 0.0))
    figure = None if one is None else max(one, again)
    ran.update(v_m_s=v, peak_n=None if figure is None else round(figure, 1), arm_n=round(arm, 1),
               pads_rise_n=None if lever is None else round(lever, 1), at=what)
    if isinstance(met, dict) and finite(met.get("z")):
        ran["load_rose_at_z_mm"] = round(float(met["z"]) * MM, 1)
    if finite(w.get("v_band_m_s")) and banded:
        ran["hand_in_band_mm_s"] = round(float(w["v_band_m_s"]) * MM, 1)
    if again > 0.0:
        ran["caught_n"] = round(again, 1)
        ran["gap_closed_mm_s"] = round(float(w.get("gap_m_s") or 0.0) * MM, 1)
    ran["note"] = ("one figure for every pair this landing loaded, over the whole way down: the arm's figure along "
                   "the travel plus the pads' rise over the squeeze that stood"
                   + ("" if not again > 0.0 else "; the jaws' own reading showed the gap close while it landed (the "
                      "thing moved between the pads): never under the strike of that closing"))
    st["ran"] = ran
    early = None
    if banded and isinstance(met, dict) and finite(met.get("z")) and float(met["z"]) > float(b["from_mm"]) / MM:
        early = float(met["z"]) - float(b["from_mm"]) / MM          # met before the band: session evidence
        b["met_before_band_mm"] = round(early * MM, 1)
    if b is not None and b.get("ends_is") == "limit":
        # kept is said only when every contact this landing made was under the limit. The limit is on
        # what the landing makes of the thing on what it lands on: the arm's figure along the travel at the
        # speed the figure is for; whenever that is over the limit the receipt says PASSED with it
        lim = float(b["ends_n"])
        if arm > lim * (1.0 + np.finfo(float).eps):
            b.update(kept=False, limit_said=f"PASSED: {what}, {v * MM:.1f} mm/s: up to {arm:.0f} N on what it "
                                            f"landed on where the limit is {lim:g} N (each pad up to "
                                            f"{0.0 if figure is None else figure:.0f} N over its squeeze)")
        elif b.get("shown_kept") and arm <= lim * (1.0 + np.finfo(float).eps):
            b.update(kept=True, limit_said=f"kept: the landing arrived at {v * MM:.1f} mm/s, no faster than the "
                                           f"speed solved for {lim:g} N")
        else:
            b.update(kept=None, limit_said=f"not shown kept: {b.get('arrival')}")
    if b is not None:
        st["band"] = b
        st["fast_n"], st["arrive_n"] = b.get("fast_n"), b.get("arrive_n")
    return st, b, early


class LandMixin(SaidMixin):
    """What the set down (rcpx/r2/place_set.py SetMixin) calls of the landing."""

    def _pl_land_plan(self, B, target, intent, facts, z_end, into, above, walled=False, z_low=None, hand_z=None):
        """The landing's part of a set down's plan, made when its end height is known: {"limit": the stated limit
        of the place, "expect": the C9 record of the way down's FAST stretch, "z_end": where that stretch
        ends (above the band's upper edge by how far this body's hand runs on past its command: RunOn; the true
        end where nothing slows the landing), "z_true": the true end (the band's lower edge for a thing set onto
        a place; never under hand_z, the hand's own stop over what a look shows beside the place),
        "arrive": {"expect", "periods"} for the arrival stretch the set down runs itself from there (None: none),
        "band": the receipt's band block or None, "land": the force, the band and the pads' numbers}.
        The plan brakes for an expected contact only at a leg's end (rcpx/bounds_plan.py), so the band is a leg
        of its own: fast to above its upper edge, the solved speed from there. walled: the place is an opening
        with a wall beside the thing (place_set); z_low: the plan's end before a hand's stop raised it."""
        label = next((x for x in (target.get("labels") or []) if x), None)
        limit, force, pads = ends(self, B, label, intent, above)
        told = self._told()
        try:
            from rcpx.r2.place_grasp import support_at
            sup = support_at(B, target["centre"][:2])
        except Exception as exc:    # a space book that states no support: none (said in the journal)
            self._journal("land_problem", where="the support under the place", error=f"{type(exc).__name__}: {exc}")
            sup = None
        rec = control_hold.held(self)
        hang = dict.get(rec, "extent") if rec is not None else None
        sig_b = hang.get("below_sigma_m") if isinstance(hang, dict) else None
        st = self.state["place"]
        bnd = None if force is None else band(target, facts, z_end, into, sup, sig_b, self._pl_res(B, told.q, DOWN),
                                              z_low, st.get("land_early"))
        fast, slow = expects(target.get("sigma_xy"), self._pl_half_width(), force)
        top = None if bnd is None else float(bnd["top_z"])
        blk, exact = block(self, B, told.q, force, bnd, pads, z_from=float(told.tip[2]), z_end=top)
        out = {"limit": limit, "expect": fast, "z_end": z_end, "z_true": z_end, "arrive": None, "band": blk,
               "land": {"force": force, "band": bnd, "pads": pads, "exact": exact, "q": np.asarray(told.q, dtype=float),
                        "held": rec}}
        st["land_periods"], st["land_fast"], st["land_top"], st["land_need"] = 0, False, top, None
        if slow is None or z_end is None:
            return out
        if bnd is None:                 # no end height to put a band about: the whole way down arrives
            out["expect"] = slow
            return out
        # the true end: a thing set ONTO a place is lowered on to the band's lower edge (the place can lie that
        # far under what was seen or stated; the guard ends the way where it lands), never under the hand's stop
        z_true = float(z_end)
        if not into and not above and z_low is not None and float(z_end) <= float(z_low):
            z_true = min(z_true, float(bnd["low_z"]))
            if finite(hand_z):
                z_true = max(z_true, min(float(z_end), float(hand_z)))
        # the fast stretch ends its command above the band by how far this body's hand runs on past a command
        # that comes to stand (measured on this session's free stops), never under the true end
        watch = run_on(self)
        over, why = (0.0, "this core runs no tick hooks: no stop was measured") if watch is None else \
            watch.margin(exact["v_fast"], float(B.lag(exact["v_fast"])), float(B.rigid(told.q, DOWN)),
                         st.get("land_ran_on"))
        z_fast = max(z_true, top + over)
        z_from = float(told.tip[2])
        if watch is not None and over > 0.0 and top < z_from < top + over:
            # the fast stretch's own margin would end it over the height its way down starts at (the speed was
            # read for braking over that travel: block()), and the carry would have to come in that much higher:
            # it runs no faster than the fastest speed whose margin ends under that start, and states the figures
            # of that speed (block() at the travel that brakes from it)
            lo_v, hi_v = float(exact["v_arr"]), float(exact["v_fast"])
            for _ in range(int(R.QUANT) ** 5):
                v_try = (lo_v + hi_v) / 2.0
                m_try = watch.margin(v_try, float(B.lag(v_try)), float(B.rigid(told.q, DOWN)), st.get("land_ran_on"))[0]
                lo_v, hi_v = (v_try, hi_v) if top + m_try <= z_from else (lo_v, v_try)
            if lo_v < float(exact["v_fast"]):
                a = max(float(B.a_brake(told.q, DOWN)), np.finfo(float).tiny)
                blk, exact = block(self, B, told.q, force, bnd, pads, z_from=top + lo_v * lo_v / (laws.TWO * a),
                                   z_end=top)
                out["band"], out["land"]["exact"] = blk, exact
                blk["fast_cut"] = (f"the fast stretch runs at {lo_v * 1000.0:.0f} mm/s at most: at a faster speed "
                                   "its own margin would end it over the height the way down starts at "
                                   f"(tip z {z_from * 1000.0:.0f} mm)")
                over, why = watch.margin(exact["v_fast"], float(B.lag(exact["v_fast"])), float(B.rigid(told.q, DOWN)),
                                         st.get("land_ran_on"))
                z_fast = max(z_true, top + over)
        depth = max(0.0, z_fast - z_true)
        v = max(float(exact["v_arr"]), np.finfo(float).tiny)
        # the way from the fast stretch's end to the band's edge is no arrival stretch (no contact can begin on
        # it): it runs at a MIDDLE speed and ends above the band by that speed's own margin (the same rule). The
        # middle speed that makes that way shortest in time under the rule (the margin grows with the speed,
        # the time on the margin's own rest with its inverse) is the geometric mean of the fast stretch's speed
        # and the arrival speed. Nothing typed
        mid, no_mid = None, None
        v_mid = math.sqrt(max(0.0, float(exact["v_fast"])) * v)
        if watch is not None and over > 0.0 and v_mid > v * (1.0 + R.REL):
            m2 = watch.margin(v_mid, float(B.lag(v_mid)), float(B.rigid(told.q, DOWN)), st.get("land_ran_on"))[0]
            z_mid = max(z_true, top + m2)
            if z_fast - z_mid > self._pl_res(B, told.q, DOWN):
                mid = {"z": z_mid, "v_m_s": v_mid, "periods": int(math.ceil((z_fast - z_mid) / (v_mid * float(self.dt))))}
            else:
                no_mid = (f"the middle speed's own margin ({m2 * MM:.1f} mm) leaves no way between the fast "
                          "stretch's end and the band's edge")
        elif over > 0.0:
            no_mid = "the middle speed would be no faster than the arrival"
        slow_m = depth if mid is None else max(0.0, mid["z"] - z_true)
        n = int(math.ceil(slow_m / (v * float(self.dt)))) + (0 if mid is None else mid["periods"])
        # the hand-over: the end of the last stretch before the band (the middle one, else the fast one) is waited
        # for until the hand has come to the arrival speed (what is left of its motion over one detection time:
        # C10's own test), whatever is left of the way to the band's edge: a hand still swinging from the stretches
        # before is still that fast there (measured on a bench arm: 171 mm/s at the band's edge for an
        # arrival of 13 mm/s, where the way to it was to let it settle). A hand that is slow already waits nothing
        need = v * float(B.t_det()) if depth > 0.0 else None
        st["land_need"] = None if mid is not None else need
        if mid is not None:
            mid["need"] = need
        out["land"].update(z_fast=z_fast, top=top)
        price = cost(self, B, told.q, force, bnd, z_end, pads, slow_m)
        if price is not None:
            blk["cost"] = price
        blk.update(to_mm=round(z_true * MM, 1), fast_to_mm=round(z_fast * MM, 1), run_on_mm=round(over * MM, 2),
                   run_on_source=why)
        if mid is not None:
            blk.update(mid_to_mm=round(mid["z"] * MM, 1), mid_mm_s=round(v_mid * MM, 1))
        elif no_mid is not None:
            blk["mid_none"] = no_mid
        out.update(z_end=z_fast, z_true=z_true, arrive={"expect": slow, "periods": n, "depth_m": depth, "mid": mid})
        st["land_periods"], st["land_fast"] = n, depth > 0.0
        if depth > 0.0 and not walled:
            # the fast stretch ends where the arrival stretch begins: the core is handed no expected contact for
            # it (it holds the end of a leg that means one until the hand stands still), and a contact on it is
            # judged here by the same test (_pl_land_down: control_expect.arrived): a landing before the band,
            # met at the speed that ran there. Into a walled opening the core judges it as before
            out["expect"], out["land"]["judge"] = None, fast
        return out

    def _pl_land_down(self, plan, target, facts, intent, r=None):
        """Run the way down from where the hand was told to be to the plan's true end: the fast stretch to above
        the band's upper edge (only when r is None: the caller did not run it), the hand-over (where the hand is
        to reach the band sooner than its own stop settles: the fast stretch's end is waited for), and then,
        when nothing was met and the thing is still held, the arrival stretch at the speed solved for the
        landing's force. A load that rose on the fast stretch or in the wait and still stands on the arm is the
        landing (stopped "arrived", before_band, untold when no guard told it): no arrival stretch is armed on
        it. Returns the result of the stretch that ended the way (with "fast": the fast stretch's own result
        when the arrival stretch ran after it), or a refusal of the bounds."""
        B = bounds_of(self)
        word = self._pl_core_intent(intent)
        label = next((x for x in (target.get("labels") or []) if x), None)

        def lower(z, expect, wait, speed=None):
            told = self._told()
            travel = float(told.tip[2]) - float(z)
            if not travel > self._pl_res(B, told.q, DOWN):
                return None
            how = word
            if speed is not None:       # the middle stretch: no faster than its own speed (nor than the operator's)
                was = (word or {}).get("speed")
                how = dict(word or {}, speed=float(speed) if was is None else min(float(was), float(speed)))
            leg = self._pl_leg("lower", np.asarray(told.tip, dtype=float) + DOWN * travel, told.R, told.q,
                               target=label, expect=expect, intent=how)
            if leg is None:
                return {"stopped": "refused", "refused": {"check": "not_reachable", "why": "the arm cannot come "
                        "straight down from here", "hint": "a place nearer the robot's base"}}
            need = self._pl_still_m(B, told.q, facts) if wait is True else wait
            return self._pl_stream([leg], ["set_down"], held=self._pl_held(), need_m=need or None)

        arrive = plan.get("arrive")
        land = plan.get("land") or {}
        judge = land.get("judge")
        if r is None:
            self._pl_land_watch(plan)
            r = lower(plan["z_end"], plan["expect"], True if arrive is None else self.state["place"].get("land_need")) \
                or {"stopped": "goal", "bounds": None}
            if "refused" in r:
                return r
        elif self.state["place"].get("land_watch") is None:
            self._pl_land_watch(plan)
        if judge is not None and r.get("stopped") == "contact" and plan.get("expect") is None:
            # a contact on the fast stretch: the landing itself when the push is on the hand or on what it
            # carries and opposes the travel (the core's own test for an expected contact, read only)
            from rcpx import control_expect
            if control_expect.arrived(control_expect.region(self, judge), r.get("contact"), DOWN):
                r = dict(r, stopped="arrived", before_band=True)
        if arrive is None or r.get("stopped") != "goal" or self._pl_held() is None:
            return r
        watch = self.state["place"].get("land_watch")
        if watch is not None:
            watch.mark()                # the fast stretch and its hand-over end here (how far the hand ran on)
        if watch is not None and watch.loaded_now():
            # a load rose on the way so far that no guard told, and it stands on the arm now: the thing has
            # landed. No arrival stretch is armed on it (its guard would take that load for its baseline)
            return dict(r, stopped="arrived", before_band=True, untold=True)
        mid = arrive.get("mid")
        if mid is not None:
            # the middle stretch, to above the band by its own speed's margin; a contact on it is judged as one
            # on the fast stretch; its end is waited for until the hand has come to the arrival speed
            rm = lower(mid["z"], None, mid.get("need"), speed=mid["v_m_s"])
            if rm is not None:
                if "refused" in rm:
                    return rm
                if judge is not None and rm.get("stopped") == "contact":
                    from rcpx import control_expect
                    if control_expect.arrived(control_expect.region(self, judge), rm.get("contact"), DOWN):
                        rm = dict(rm, stopped="arrived", before_band=True)
                if rm.get("stopped") != "goal" or self._pl_held() is None:
                    return dict(rm, route=r.get("route"), fast=r)
                if watch is not None and watch.loaded_now():
                    return dict(rm, stopped="arrived", before_band=True, untold=True, route=r.get("route"), fast=r)
        r2 = lower(plan["z_true"], arrive["expect"], True)
        if r2 is None:
            return r
        if "refused" in r2:
            return r2
        r2.update(route=r.get("route"), doubt=list(r.get("doubt") or []) + list(r2.get("doubt") or []), fast=r)
        return r2

    def _pl_land_ends(self, plan, z_true):
        """{"z_end", "z_true"} of the plan's way down for another true end (a second way down at another spot):
        the fast stretch ends as far above the band's upper edge as the plan's first way did, where that lies
        over the new end."""
        bnd = (plan.get("land") or {}).get("band")
        blk = plan.get("band") or {}
        top = None if plan.get("arrive") is None or bnd is None else float(bnd["top_z"]) + float(
            blk.get("run_on_mm") or 0.0) / MM
        return {"z_true": z_true, "z_end": z_true if top is None or z_true is None else max(float(z_true), top)}

    def _pl_land_watch(self, plan=None):
        """Start reading the way down that begins now (place_land_watch.LandWatch: the hand, the arm's reading,
        the jaws); the one before it ends here."""
        st = self.state["place"]
        old = st.pop("land_watch", None)
        if old is not None:
            old.done()
        bnd = ((plan or {}).get("land") or {}).get("band") if isinstance(plan, dict) else None
        st["land_watch"] = LandWatch(self, st.get("land_top") if bnd is None else bnd["top_z"])

    def _pl_land_after(self, plan, r):
        """(the set down's strike block, its band block) once the way down r ran (after()). A landing that met
        what it landed on before its band is this session's evidence for the next one's band (band(): early)."""
        st = self.state["place"]
        watch = st.pop("land_watch", None)
        read = None if watch is None else watch.done()
        land = plan.get("land") or {}
        B, q, held = bounds_of(self), land.get("q"), land.get("held")
        arm_fn = None if q is None else (lambda v: _arm(B, q, v, held)[0])
        strike, blk, early = after(plan.get("band"), r, land.get("pads"), read, arm_fn, land.get("exact"))
        if early is not None and early > 0.0:
            was = st.get("land_early") or (0.0, 0)
            st["land_early"] = (max(float(was[0]), float(early)), int(was[1]) + 1)
        z_fast, v_fast = land.get("z_fast"), (land.get("exact") or {}).get("v_fast")
        mark = None if read is None else (read.get("fast") or (read.get("z_low"), read.get("met") is not None))
        if mark is not None and finite(z_fast, v_fast, mark[0]) and float(v_fast) > 0.0 and blk is not None:
            ran_on = max(0.0, float(z_fast) - float(mark[0]))    # past the end of its fast stretch
            was = st.get("land_ran_on") or {"tau_s": 0.0, "n": 0, "cut": False}
            st["land_ran_on"] = {"tau_s": max(float(was["tau_s"]), ran_on / float(v_fast)), "n": int(was["n"]) + 1,
                                 "cut": bool(was["cut"] or mark[1])}
            blk["ran_on_mm"] = round(ran_on * MM, 1)
        return strike, blk
