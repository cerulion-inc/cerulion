# SPDX-License-Identifier: AGPL-3.0-only
"""The count of package place: what a whole-step tool will take,
before it moves, and the small parts of its receipt.

The budget line of a put and of a place is the control periods of the plan AS IT WILL RUN at its planned speeds:
the bounds' own plan of each stream (rcpx.bounds plan_legs: the same call the stream itself makes), the jaws at
their own rate, and the waits the plan holds, each one settle window long (the robot's own measured settle time:
the wait at the end of a leg that means a contact, the wait before the load is read and the read itself). No
margin is added and no branch that is not planned is counted (a second take is not planned). A plan that needs
more periods than are left is refused budget before anything moves; the same count is made again by the core
inside each stream piece.

- _pl_count(legs, q0, held): the periods one stream plans.
- _pl_wait(): the periods one planned wait holds.
- _pl_place_count(way, facts): a place: the carry's stream, the way down and its end wait, the release, the rise
  from where the way down ends.
- _pl_price(a, target, intent): one item of a put: the take (the pick's own count when the pick gives one through
  _pk_price(a, intent); else what a take runs: the plain route to the thing and the wait at its end, the close,
  the test lift, the wait and the load read at rest), then the place from where the take leaves the hand.
- budget(tool, need, left, price, word, call): the refusal; with a stated limit it names the word, its price in
  control periods, the least limit that fits and the ready calls. _pl_word, _pl_word_price and
  _pl_limit_cost make the blocks; result["limit"]["cost"] is what the word added, and where.
- worst(strikes): the largest strike of several motions, with the largest model and link figure any of them
  states. doubts(lines): the doubt lines of a receipt.
Nothing here reads ground truth or names a robot.
"""

import math
from types import SimpleNamespace

import sys

import numpy as np

from rcpx import bounds
from rcpx import bounds_ratios as ratios
from rcpx import control_hold
from rcpx.context import refusal
from rcpx.r2.place_hand import finite
from rcpx.r2.place_motion import UP


class CountMixin:
    def _pl_count(self, legs, q0=None, held=None):
        """The control periods a stream of these legs plans from q0 (default: where the hand was told to be): the
        bounds' own plan as it will run. None when the bounds refuse the plan (the stream then says why)."""
        if not legs:
            return 0
        q0 = self._told().q if q0 is None else np.asarray(q0, dtype=float)
        try:
            plan = bounds.of(self).plan_legs(q0, [dict(leg) for leg in legs], held=held)
        except Exception:
            return None
        if getattr(plan, "refused", None):
            return None
        return len(getattr(plan, "path", None) or [])

    def _pl_wait(self, stop=False):
        """The control periods one planned wait holds: the settle window (the robot's own measured settle time,
        no shorter than the readings a judgement needs); stop: and the one period the ease that follows a stop
        runs at the least (a leg that means a contact ends on one)."""
        return max(int(ratios.QUANT), int(math.ceil(float(bounds.of(self).t_settle()) / float(self.dt)))) + int(stop)

    def _pl_place_count(self, way, facts, start=None, intent=None):
        """The control periods a place plans: the carry's one stream; the way down as the bounds time it
        and the wait that ends a leg that means a contact; the release at the jaws' own rate; the rise from
        where the way down ends. No margin, and no branch that is not planned. start: the pose the way was
        planned from (default: where the hand was told to be). Counted before the take (nothing held yet), the
        streams are planned holding the bounding box the facts give (from up the fingers to below the tip, the half width
        across) under the limit intent states, as the take records both on the hold and the streams plan with it.
        None when the bounds refuse a plan (the stream then says why)."""
        B = bounds.of(self)
        told = self._told() if start is None else start
        held = self._pl_held()
        if held is None:        # counted before the take: the thing as the count's facts have it, as the streams
            held = {"extent_m": [[x, y, z] for x in (-float(facts["above_m"]), float(facts["below_m"]))  # will hold it
                                 for y in (-float(facts["half_m"]), float(facts["half_m"]))
                                 for z in (-float(facts["half_m"]), float(facts["half_m"]))]}
            lim = B.stated_limit(None, intent or None, expected=True) if intent else {}
            if lim.get("limit_n") is not None:      # ... and the call's stated limit, as the take records it on the hold
                held.update(limit_n=float(lim["limit_n"]), limit_source=str(lim.get("source") or "intent"))
        carry = self._pl_count(way["legs"], q0=told.q, held=held)
        q_over = told.q if way.get("q_over") is None else way["q_over"]
        down = 0 if way.get("set_leg") is None else self._pl_count([way["set_leg"]], q0=q_over, held=held)
        if carry is None or down is None:
            return None
        wait = 0 if way.get("set_leg") is None else self._pl_wait(stop=True)
        end, q_end = way["over"], q_over
        if way.get("set_leg") is not None:
            to = np.asarray(way["set_leg"]["to"], dtype=float)
            end = to[:3, 3] if to.ndim > 1 else self.model.fk_tip(to)[0]
            q_end = q_over if way.get("q_end") is None else way["q_end"]
        rise = float(facts["above_m"]) + self._pl_room(B, told.q, UP)
        leg = self._pl_leg("lift", np.asarray(end, dtype=float) + UP * rise, told.R, q_end)
        up = 0 if leg is None else (self._pl_count([leg], q0=q_end) or 0)
        release = self._pl_release_periods()
        return {"periods": carry + down + wait + release + up, "carry": carry, "set_down": down + wait,
                "release": release, "rise": up}

    def _pl_take_count(self, a, z_go, q_low, lift_m):
        """What a take runs when the pick gives no count of its own: the plain route to the thing (up to z_go,
        across, down) as the bounds plan it and the wait at the end of its way down (a leg that means a
        contact); the close from the open rest to the seen width at the jaws' own rate and what their reading
        trails by; the test lift from the grasp pose q_low; the wait until the hand is calm and the load read
        at rest (two settle windows)."""
        J = self._pl_jaws
        told = self._told()
        p = np.asarray(a["point"], dtype=float)
        R, q0 = np.asarray(told.R, dtype=float), told.q
        seen = a["seen"] if isinstance(a.get("seen"), dict) else {}
        legs = [self._pl_leg("transit", np.array([told.tip[0], told.tip[1], z_go]), R, q0),
                self._pl_leg("transit", np.array([p[0], p[1], z_go]), R, q0),
                self._pl_leg("approach", p, R, q0)]
        way_in = self._pl_count([leg for leg in legs if leg is not None]) or 0
        N = control_hold.numbers(self)
        dt = float(self.dt)
        width = (seen.get("outline") or {}).get("narrow")
        frac_to = float(J.frac_for_width(float(width))) if finite(width) else float(N["closed_gap"])
        close = int(math.ceil(max(0.0, float(N["open_rest"]) - frac_to) / (float(N["speed"]) * dt))) \
            + int(math.ceil(float(N["lag"][1]) / dt))
        up = None if q_low is None else self._pl_leg("lift", p + UP * lift_m, R, q_low)
        lift = 0 if up is None else (self._pl_count([up], q0=q_low) or 0)
        wait = self._pl_wait()
        return {"periods": way_in + wait + 1 + close + lift + wait + wait, "way": way_in + wait + 1, "close": close,
                "lift": lift, "rest": wait + wait}

    def _pl_price(self, a, target, intent):
        """The control periods one item of a put plans before it starts: the take (the pick's own count when
        the pick gives one, else _pl_take_count), then the place from where the take leaves the hand (its
        stream, the way down, the release, the rise), with the held thing taken as deep as the thing was seen
        tall. Returns {"periods", "take", "place", "take_parts", "place_parts", "take_by"}."""
        B = bounds.of(self)
        told = self._told()
        J = self._pl_jaws
        p = np.asarray(a["point"], dtype=float)
        R, q0 = np.asarray(told.R, dtype=float), told.q
        q_low, _ = self._pl_solve(p, R, R[:, 0], [q0, self.model.q_home])
        seen = a["seen"] if isinstance(a.get("seen"), dict) else {}
        tall = seen.get("height")
        floor = self._pl_support_z()
        if not finite(tall):
            tall = max(0.0, float(p[2]) - floor) if floor is not None else 0.0
        # where a take leaves the hand: over the thing, by the test lift (twice the fingers' own length, and no
        # further than the hand came down around it)
        z_go = max(float(told.tip[2]), float(p[2]) + self._pl_room(B, q0, UP))
        reach = float(J.finger_len_m or J.pad_height_m or 0.0)
        lift = min(reach + reach, z_go - float(p[2]))
        q_up = None if q_low is None else self._pl_solve(p + UP * lift, R, R[:, 0], [q_low])[0]
        start = SimpleNamespace(q=q_up if q_up is not None else (q_low if q_low is not None else q0),
                                tip=p + UP * lift, R=R)
        hook = getattr(self, "_pk_price", None)
        got = hook(a, intent) if callable(hook) else None
        take = got.get("periods") if isinstance(got, dict) else got
        by = "the pick's own count"
        if not finite(take):
            got = self._pl_take_count(a, z_go, q_low, lift)
            take, by = got["periods"], "the plain route, the close, the test lift and the load read"
        facts = {"offset_T": np.zeros(3), "below_m": min(float(tall), float(J.finger_len_m or tall)),
                 "half_m": J.max_m / 2, "above_m": float(J.finger_len_m or J.max_m)}
        target = self._pl_price_place(a, target, facts)
        plan = self._pl_set_plan(target, intent, facts)
        way = self._pl_way(target, facts, intent, B, z_end=plan["z_end"], expect=plan["expect"], start=start,
                           z_hand=plan.get("hand_stop_z"))
        place = None if "refused" in way else self._pl_place_count(way, facts, start=start, intent=intent)
        return {"periods": int(take) + int((place or {}).get("periods") or 0), "take": int(take),
                "place": None if place is None else place["periods"], "take_by": by,
                "take_parts": {k: v for k, v in got.items() if k != "periods"} if isinstance(got, dict) else None,
                "place_parts": None if place is None else {k: v for k, v in place.items() if k != "periods"}}


    def _pl_price_place(self, a, target, facts):
        """A copy of the place target as the put will run it: the free spot (place_spot.settle) read with this
        item's own take record (what the look shows of the thing to take, and where it stood), as place.py reads
        it once the thing is in the hand. The target itself is not changed; a look that cannot be read leaves the
        copy as pointed."""
        from rcpx.r2 import place_spot
        st = self.state["place"]
        keep = {k: st.get(k) for k in ("taken", "picked_at")}
        out = dict(target, doubt=list(target.get("doubt") or []))
        try:
            st["taken"] = place_spot.taken_of(self, a["seen"])
            st["picked_at"] = [float(x) for x in a["point"]]
            place_spot.settle(self, out, facts)
        except Exception:
            out = dict(target, doubt=list(target.get("doubt") or []))
        finally:
            st.update(keep)
        return out

    # ---------------------------------------------------------------- the price of a stated limit
    def _pl_word(self, target, intent):
        """The limit this call states for its contacts, as a budget line names it: {"word": "gentle" | "max_n" |
        "the sheet of X" | "the site" | the source, "limit_n", "own": the operator's own word on the call,
        "text"}, or None when nothing is stated."""
        label = next((x for x in ((target or {}).get("labels") or []) if x), None)
        lim = bounds.of(self).stated_limit(label, intent or None, expected=True)
        if lim.get("limit_n") is None:
            return None
        source, own = str(lim.get("source")), str(lim.get("source")) == "intent"
        word = ("gentle" if "gentle" in str(lim.get("evidence")) else "max_n") if own else \
            (f"the sheet of {label}" if source == "sheet" else ("the site" if source == "site" else source))
        return {"word": word, "limit_n": float(lim["limit_n"]), "own": own, "source": source,
                "text": (f"Your word {word}" if own else word[:1].upper() + word[1:]) + f" ({float(lim['limit_n']):.3g} N)"}

    def _pl_word_price(self, word, intent, count, left=None):
        """What a stated limit costs this plan: {"word", "limit_n", "without_periods": the same plan with the
        word left out, "fits_n": the least limit of a doubling ladder from the stated one whose plan fits the
        periods left (None: none fits, the word is not the operator's own, or nothing is left to ask for),
        "fits_periods"}. count(intent) -> control periods or None: the caller's own count of the same plan."""
        bare = {k: v for k, v in (intent or {}).items() if k not in ("gentle", "max_n")}
        without = count(bare)
        out = {"word": word["word"], "limit_n": word["limit_n"], "without_periods": without, "fits_n": None,
               "fits_periods": None}
        if not word["own"] or left is None or without is None or without > left:
            return out              # (left None: the plan fits, only the word's price is asked)
        n = float(word["limit_n"])
        # each step doubles the limit; it ends when the plan fits (the plan without the word fits, so a large
        # enough limit does), or after as many doublings as a float's mantissa tells apart. A doubling that gains
        # nothing does not end it (a plateau: another leg binds until the limit passes its figure;
        # without it 10 of 10 refusals under max_n 20 named no least max_n)
        for _ in range(sys.float_info.mant_dig):
            n *= ratios.DOUBLE
            got = count(dict(bare, max_n=n))
            if got is None:
                return out
            if got <= left:
                out.update(fits_n=float(f"{n:.3g}"), fits_periods=int(got))
                return out
        return out

    @staticmethod
    def _pl_limit_cost(word, without, used, phases):
        """result["limit"]["cost"]: the control periods the stated limit added, and in which phases a bound
        of that source was the binding one."""
        where = [p["phase"] for p in phases or [] if isinstance(p.get("bounds"), dict)
                 and str(p["bounds"].get("source")) == word["source"]]
        return {"periods": None if without is None else int(used) - int(without),
                "without_periods": None if without is None else int(without), "where": where}


def call_of(tool, item, intent):
    """{"tool", "args"} of a whole-step call as it was made: the item's own arguments and the call's words."""
    args = {k: v for k, v in (item or {}).items() if v is not None}
    args.update({k: intent[k] for k in ("gentle", "max_n", "firm", "expect", "release", "clear") if k in intent})
    return {"tool": tool, "args": args}


def budget(tool, need, left, price=None, word=None, call=None):
    """The budget refusal: the plan as it will run needs more control periods than are left; nothing moved.
    word: _pl_word_price's block when the call states a limit: the refusal then NAMES the word, says how many of
    the plan's periods are its price, and offers the same call with the least limit that fits (first: it keeps a
    limit) and the same call without the word (only the operator's own word can be left out), each with its
    periods. call: {"tool", "args"} of the call as it was made."""
    parts = "" if not price else f" (the take {price.get('take')}, the place {price.get('place')})"
    why = f"this {tool} plans {int(need)} control periods{parts} and {int(left)} are left: not started"
    out = refusal("budget", why, "a nearer place, or a shorter move")
    block = {"periods": int(need), "left": int(left), "word": None, "limit_n": None, "without_periods": None,
             "fits_n": None}
    ready = []
    if word:
        block.update({k: word.get(k) for k in ("word", "limit_n", "without_periods", "fits_n")})
        without = word.get("without_periods")
        if without is not None:
            text = word.get("text") or str(word["word"])
            out["refused"]["why"] = why + (f". {text} is {int(need) - int(without)} of them: without it the {tool} "
                                           f"plans {int(without)}") + (
                f"; the least max_n whose plan fits is {word['fits_n']:g} N ({word['fits_periods']} control periods)"
                if word.get("fits_n") is not None else "")
            args = dict((call or {}).get("args") or {})
            bare = {k: v for k, v in args.items() if k not in ("gentle", "max_n")}
            if word.get("fits_n") is not None:
                ready.append({"tool": tool, "args": dict(bare, max_n=word["fits_n"]), "periods": word["fits_periods"],
                              "what": f"the same {tool} under max_n {word['fits_n']:g} N: the least limit of a "
                                      "doubling ladder whose plan fits"})
            if word.get("own") and int(without) <= int(left):
                ready.append({"tool": tool, "args": bare, "periods": int(without),
                              "what": f"the same {tool} without the word {word['word']}"})
            out["refused"]["hint"] = ("one of the ready calls" if ready else
                                      "a nearer place, or a shorter move: the plan does not fit without the word either")
    out["refused"]["price"] = block
    if ready:
        out["refused"]["ready"] = ready
    return out


def _size(strike):
    """What a strike is ranked by: its stated peak, else the least it could be, else the body's model figure."""
    if not isinstance(strike, dict):
        return -1.0
    for key in ("peak_n", "at_least_n", "model_peak_n"):
        if finite(strike.get(key)):
            return float(strike[key])
    return 0.0


def worst(strikes):
    """The largest strike of several motions (a whole-step tool carries the largest of its motions), ranked by
    its own stated figure. Its model figure and its link figure are the largest ANY of the motions states (a
    motion with a small floor can hold the largest model figure, and a reader takes the receipt's strike for
    the whole tool); "of_all" says so."""
    got = [s for s in strikes if isinstance(s, dict)]
    if not got:
        return None
    out = dict(max(got, key=_size))
    if len(got) > 1:
        model = [float(s["model_peak_n"]) for s in got if finite(s.get("model_peak_n"))]
        links = [s["link"] for s in got if isinstance(s.get("link"), dict)]
        if model:
            out["model_peak_n"] = max(model)
        if links:
            out["link"] = max(links, key=lambda k: max([float(k[x]) for x in ("peak_n", "model_peak_n")
                                                        if finite(k.get(x))] or [0.0]))
        out["of_all"] = f"the model figure and the link figure are the largest of this call's {len(got)} motions"
    return out


def doubts(lines):
    """The doubt lines of a receipt, each {what, acted, would_remove}; a bare text becomes one; no line twice."""
    out, seen = [], set()
    for d in lines or []:
        if not isinstance(d, dict):
            d = {"what": str(d), "acted": "acted on what it had, the contact stop armed", "would_remove": None}
        key = str(d.get("what"))
        if key not in seen:
            seen.add(key)
            out.append({"what": d.get("what"), "acted": d.get("acted"), "would_remove": d.get("would_remove")})
    return out
