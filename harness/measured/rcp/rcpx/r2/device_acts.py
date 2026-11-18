# SPDX-License-Identifier: AGPL-3.0-only
"""Acts on a device: the hooks on touch and press, the long-press refusal, and the outcome of each act.

Pre hook (touch, press): pops expect_cue and cue_watch_s, attributes the act to a device (device_entry.attribute),
refuses a press whose model prediction, scaled by earlier measured contact times, would pass the entry's long_press_s (nothing moves), and reads the
device's cue before the act (not for a call the executor will refuse on its arguments). Tick hook: records force and
travel along the push each control period, so contact time and the travel under load are measured, and returns
"long_press" as the contact nears long_press_s (the motion loops honour it once the core lane's C6 lands; until then
the press runs on and the result says so). Post hook: outcome_class (ran_free_beyond_tip | touched_no_give | gave |
moved_under_load | slid_off, defined in OUTCOME_CLASSES), same_as (earlier acts this epoch within the two contacts' combined uncertainty with the same class), and on a device with a located cue: the
cue after the act and device_outcome (expected | other_transition | no_change | unobservable | unpowered_suspected);
two no-change acts with contact confirmed while the cue stays dark add the power hint. A cue that goes lit -> dark at
its located point across an act may have switched off or moved with a part the act moved: from then on a dark reading
there is "unobservable" (reason cue_may_have_moved) until the cue part is located anew. Device
kinds are never named; everything comes from the entry's fields and this session.
"""

import inspect
import math

import numpy as np

from rcpx.context import refusal, jsonable
from rcpx.r2 import device_cue as cue
from rcpx.r2 import device_entry as de
from rcpx.r2 import device_bounds as derived

MM = 1000.0
PKG = "device"
RAN_FREE_FRAC = 0.25     # a press whose peak stays under this share of force_n met nothing
LOADED_FRAC = 0.2        # travel under load counts from the first period over this share of force_n
POWER_HINT = ("the cue the robot watches stayed dark through this act and the one before it, both with contact: "
              "that is all the robot read")   # the design rulings: a reading, no advice to stop or check
# Loading a face slows under force; a ramp alone is not a measured contact-time bound.
# The largest measured/predicted ratio of earlier presses scales this session's prediction.
def new_state():
    return {"acts": [], "cue": {}, "watches": {}, "n_watch": 0, "active": None, "power": {}, "doubt": {}}


def ease_bound_s(ex):
    """The longest the tip can stay on a part after a push is asked to stop (s): the arm's settling time, the
    executor's own ease bound when its ease states one (read from the executor, never copied), and the longest this
    session's presses in fact stayed on after their stop was asked."""
    B = derived.of(ex)
    values = [float(B.t_settle())]
    try:
        periods = inspect.signature(type(ex)._ease_impl).parameters["max_periods"].default
        if isinstance(periods, (int, float)) and not isinstance(periods, bool):
            values.append(float(periods) * float(B.dt()))
    except (AttributeError, KeyError, TypeError, ValueError):
        pass
    measured = (getattr(ex, "state", {}).get(PKG) or {}).get("ease_s")
    if isinstance(measured, (int, float)):
        values.append(float(measured))
    return max(values)


def press_contact_s(B, kwargs, thing, q, u, history=()):
    """Predicted contact seconds from the force ramp, stated hold and settling time.

    Earlier presses of this session supply the largest measured/predicted ratio.
    The tick hook protects the sheet limit; this prediction is not a proven bound.
    """
    return derived.press_contact_s(B, kwargs, thing, q, u, history)[0]


OUTCOME_CLASSES = {
    "ran_free_beyond_tip": "the tip felt nothing push back (a touch without contact, or a press whose peak "
                           "stayed under the force share plus the measured detection floor)",
    "touched_no_give": "the tip met the part and it did not move under the push",
    "gave": "the part let go under the push (the result's gave is true: the force fell after its peak)",
    "moved_under_load": "the part moved with the tip pushing on it, over this robot's rigid travel at the measured peak force, without letting go (gave is "
                        "false): a spring, a slide or a part pushed along",
    "slid_off": "the tip slipped off the part sideways",
}


# ---------------------------------------------------------------- pure decisions
def outcome_class(name, result, force_n, moved_mm, B, q=None, u=None):
    """One of OUTCOME_CLASSES from a touch or press result and the travel the tip made while the part pushed back
    (moved_mm). gave only when the result's own gave is true."""
    if name == "touch":
        return "touched_no_give" if result.get("contact") else "ran_free_beyond_tip"
    if result.get("slipped"):
        return "slid_off"
    if result.get("gave"):
        return "gave"
    if float(result.get("peak_force_n") or 0.0) < max(B.f_thr(), RAN_FREE_FRAC * float(force_n)):
        return "ran_free_beyond_tip"
    if moved_mm is not None and moved_mm / MM > derived.moved_m(B, q, u, float(result.get("peak_force_n") or 0)):
        return "moved_under_load"     # the part moved with the tip on it, but no give (the result's gave is false)
    return "touched_no_give"


def cue_changed(before, after):
    """True when the cue was read before and after an act and the readings differ (a burst reads only lit or off,
    so lit and steady are the same reading)."""
    vb, va = (before or {}).get("verdict"), (after or {}).get("verdict")
    cb, ca = cue.cue_class(vb), cue.cue_class(va)
    if cb is None or ca is None:
        return False
    return not (ca == cb and (vb == va or "lit" in (vb, va)))


def went_dark(before, after):
    return (cue.cue_class((before or {}).get("verdict")) == "lit"
            and cue.cue_class((after or {}).get("verdict")) == "dark")


GONE_DARK = ("; read at the cue's located point, so the device went dark or the part that carries the cue moved: "
             "look at it, and locate the cue again if it moved")


def device_outcome(before, after, expect, contact_ok, dark_repeats):
    """(outcome, why) from the cue reading before and after an act. expect: the cue the operator expects after
    it (off | steady | blinking), else a dark cue turning lit is the device answering and a lit cue turning dark
    is another transition (which may also be the cue moving away with a part). dark_repeats: earlier acts on this
    device this epoch with contact and a dark cue that did not change."""
    vb, va = (before or {}).get("verdict"), (after or {}).get("verdict")
    cb, ca = cue.cue_class(vb), cue.cue_class(va)
    if cb is None or ca is None:
        side = "before" if cb is None else "after"
        why = ((before if cb is None else after) or {}).get("reason") or "no reading"
        return "unobservable", f"the cue could not be read {side} the act ({why})"
    same = not cue_changed(before, after)
    dark_note = GONE_DARK if went_dark(before, after) else ""
    if expect:
        if va == expect:
            return "expected", f"the cue reads {va} after the act, as expected"
        if not same:
            return "other_transition", f"the cue went {vb} -> {va}, not the expected {expect}{dark_note}"
    elif cb == "dark" and ca == "lit":
        return "expected", f"the cue went {vb} -> {va}: the device answered"
    elif not same:
        return "other_transition", f"the cue went {vb} -> {va}{dark_note}"
    if ca == "dark" and contact_ok and dark_repeats >= 1:
        return "unpowered_suspected", (f"the cue stayed dark through {dark_repeats + 1} acts with contact (the "
                                       "robot read no change)")
    note = "" if ca == "dark" else "; a part the cue does not show may still have moved (a look costs 0 control periods)"
    return "no_change", f"the cue stayed {va}{note}"


def timeout_range(idle, now, t_changed, t_last):
    """(lo, hi) seconds left before an idle timeout of idle = (lo, hi) s that counts from the device's last input.
    That input came no earlier than the start of the last act whose cue changed (t_changed; None: unknown, so it may
    be long past) and no later than the end of the last act (t_last): lo = idle_lo - (now - t_changed), 0 when
    unknown; hi = idle_hi - (now - t_last); neither under 0."""
    hi = max(0.0, float(idle[1]) - (now - t_last))
    lo = 0.0 if t_changed is None else max(0.0, float(idle[0]) - (now - t_changed))
    return min(lo, hi), hi


# ---------------------------------------------------------------- hooks
def _dist(a, b):
    return float(np.linalg.norm(np.asarray(a, float) - np.asarray(b, float)))


def _load(active):
    """(longest contact run s, total contact s, travel under load mm) from the tick series."""
    ser, dt = active["series"], active["dt"]
    run = best = total = 0
    for _, _, _, fmag in ser:
        if fmag >= active["contact_n"]:
            run, total = run + 1, total + 1
            best = max(best, run)
        else:
            run = 0
    loaded = [x for _, x, fd, _ in ser if fd >= max(active["contact_n"], LOADED_FRAC * active["force_n"])]
    moved = (max(loaded) - loaded[0]) * MM if loaded else None
    return round(best * dt, 3), round(total * dt, 3), None if moved is None else round(moved, 2)


def long_press_check(ex, entry, kwargs, s, d):
    lp = de.num_range(entry, "long_press_s")
    if lp is None or "travel_mm" not in kwargs or "force_n" not in kwargs:
        return None
    if kwargs.get("mode", "nudge") not in ("nudge", "firm", "rounds"):
        return None
    B = derived.of(ex)
    history = ex.state.get(PKG, {}).get("acts", [])
    est = press_contact_s(B, kwargs, B.thing(de.ident(entry)), s.q, d, history)
    limit = derived.long_limit_s(B, lp[0], history)
    if est <= limit:
        return None
    # ONE ready call that passes this same check: the same press with the hold the limit leaves (the ramp and the
    # settling time come off first); when no hold fits, the touch.
    ready = dict(kwargs)
    ready.pop("hold", None)
    ready["hold_s"] = 0
    base = press_contact_s(B, ready, B.thing(de.ident(entry)), s.q, d, history)
    ratio = max([1] + [r["contact_s"] / r["predicted_s"] for r in history if r.get("contact_s") is not None and (r.get("predicted_s") or 0) > 0])
    ready["hold_s"] = float(max(0, (limit - base) / ratio))
    if base <= limit and press_contact_s(B, ready, B.thing(de.ident(entry)), s.q, d, history) <= limit:
        hint = f"press(**{jsonable(ready)!r})"
    else:
        hint = f"touch(**{jsonable(dict(direction=kwargs.get('direction'), max_mm=kwargs['travel_mm']))!r})"
    return refusal("long_press", f"{derived.prediction_source(history)}: this press predicts {est:g} s of contact; long_press_s {lp[0]:g} s "
                   f"less the measured timing error gives {limit:g} s", hint)


def refused_by_arguments(ex, name, kwargs, s, d):
    """Why the executor will refuse this touch or press on its arguments alone (the first checks it makes, before
    any motion), or None. The cue is not read before such a call, so a refused call lets no world time pass."""
    if d is None:
        return "the direction or frame does not resolve"
    if name != "press":
        return None
    if kwargs.get("mode", "nudge") not in ("nudge", "firm", "rounds"):
        return "unknown press mode"
    try:
        force = float(kwargs["force_n"])
    except (KeyError, TypeError, ValueError):
        return "force_n is not a number"
    rated = derived.of(ex).f_tool()      # None: no contact rating is stated for the tool (r6 C1)
    if force > min(v for v in (derived.of(ex).f_cap(s.q, d) * (1 - derived.of(ex).e_tau()), rated) if v is not None):
        return "force_n is over what the tip can push here"
    give = getattr(ex, "base_give", None)
    if give and give.get("bound_n") is not None and force > give["bound_n"]:
        return "force_n is over the measured base give bound"
    return None


def make_hooks(ex):
    def st():
        return ex.state.setdefault(PKG, new_state())

    def pre(name, kwargs):
        expect = kwargs.pop("expect_cue", None)
        watch_s = kwargs.pop("cue_watch_s", None)
        s = ex._sense()
        start = [float(x) * MM for x in s.tip]
        d = None
        if kwargs.get("direction") is not None:
            d, bad = ex._unit_in_base(kwargs.get("direction"), kwargs.get("frame", "base"), s)
            d = None if bad is not None else d
        entry, how = de.attribute(ex, st(), start)
        if name == "press" and entry is not None and d is not None:
            bad = long_press_check(ex, entry, kwargs, s, d)
            if bad is not None:
                return bad
        lp = de.num_range(entry, "long_press_s") if entry is not None else None
        B = derived.of(ex)
        force_n = B.f_thr()
        fn = kwargs.get("force_n")
        if isinstance(fn, (int, float)):
            force_n = float(fn)
        prediction = None
        if name == "press" and d is not None:
            prediction = press_contact_s(B, kwargs, B.thing(de.ident(entry)) if entry else None, s.q, d, st()["acts"])
        a = {"name": name, "start_mm": start, "t0": float(s.t), "device": None if entry is None else de.ident(entry),
             "how": how, "expect": expect, "watch_s": watch_s, "d": d, "series": [], "dt": ex.dt,
             "force_n": force_n, "before": None, "before_idle_s": 0.0, "n_contact": 0,
             "rounds": name == "press" and kwargs.get("mode") == "rounds",
             "contact_n": B.f_thr(), "q": s.q.copy(), "predicted_s": prediction,
             "long_press_s": None if lp is None else lp[0],
             "long_s": None if lp is None else derived.long_stop_s(B, lp[0], ease_bound_s(ex))}
        st()["active"] = a
        if name == "press" and ("hold" in kwargs or "hold_s" in kwargs):
            hold = kwargs.get("hold", kwargs.get("hold_s"))
            if isinstance(hold, bool) or not isinstance(hold, (int, float)) or not math.isfinite(hold) or hold < 0:
                return refusal("hold", "hold must be a finite nonnegative duration", "state hold_s in seconds")
            takes = inspect.signature(type(ex).press).parameters
            for key in ("hold", "hold_s"):
                if key in kwargs and key not in takes:     # this core's press takes no stated hold: it runs its own
                    kwargs.pop(key)
                    a["hold_note"] = ("this runtime's press takes no stated hold: the press ran with its own; the "
                                      "long-press check counted the hold you stated")
        if entry is not None and watch_s != 0:
            why_not = refused_by_arguments(ex, name, kwargs, s, d)
            if why_not is not None:
                a["before"] = {"verdict": "unobservable", "reason": "not_read",
                               "why": f"the cue was not read: the call is refused ({why_not})"}
            else:
                a["before_read"] = True
                t_before = ex.sim_time()
                a["before"] = ex._dv_cue_before(entry)
                a["before_idle_s"] = round(ex.sim_time() - t_before, 3)
        st()["active"] = a
        return None

    def tick(sense):
        a = st().get("active")
        if a is None or a["d"] is None:
            return None
        a["series"].append((float(sense.t), float(sense.tip @ a["d"]), float(sense.F @ a["d"]), float(sense.Fmag)))
        if sense.Fmag >= a["contact_n"]:
            a["n_contact"] += 1
        if a["long_s"] is not None:
            if a["rounds"]:
                run = a["n_contact"]       # rounds never lift the tip between rounds: the part feels one hold
            else:
                run = 0
                for *_, fmag in reversed(a["series"]):
                    if fmag < a["contact_n"]:
                        break
                    run += 1
            if run * a["dt"] >= a["long_s"]:
                a.setdefault("asked_at_s", run * a["dt"])     # contact time when the stop was first asked
                return "long_press"
        return None

    def post(name, kwargs, result):
        a = st().pop("active", None)
        st()["active"] = None
        if a is not None and isinstance(result, dict) and "refused" in result and a.get("before_read"):
            return {**result, "world_s_passed": a["before_idle_s"],
                    "world_s_note": "the device's cue was read before this call was refused: the arm held still "
                                    "while that much world time passed"}
        if a is None or not isinstance(result, dict) or "refused" in result:
            return None
        run_s, total_s, moved = _load(a) if a["d"] is not None else (None, None, None)
        B = derived.of(ex)
        used_force = float(result.get("force_n", result.get("peak_force_n", a["force_n"])) or 0)
        oc = outcome_class(name, result, used_force, moved, B, a["q"], a["d"])
        contact_entry = ex.world.get(result.get("id")) if result.get("id") else None
        if contact_entry is None and isinstance(result.get("contact"), dict) and result["contact"].get("id"):
            contact_entry = ex.world.get(result["contact"]["id"])
        sigma, sigma_source, sigma_evidence = derived.contact_sigma_m(B, contact_entry, result, a["q"], a["d"])
        if name == "touch" and result.get("contact") and result.get("point_mm") is not None:
            point = list(result["point_mm"])
        else:
            point = a["start_mm"] if name == "press" else list(result.get("tip_mm") or a["start_mm"])
        entry = de.find(ex, a["device"]) if a["device"] else de.attribute(ex, st(), point)[0]
        how = a["how"] if a["device"] else "the act's end point"
        tip_end = list(result.get("tip_mm") or a["start_mm"])
        # the same attempt: the same class, where it met the part or where the tip ended within the combined contact uncertainty (the contact
        # point an estimate places on a tool face can move between two runs of one motion)
        same = [x for x in st()["acts"] if x["epoch"] == ex.epoch and x["outcome_class"] == oc
                and (_dist(x["point_mm"], point) <= derived.same_m(B, x.get("sigma_p", 0), sigma) * MM or _dist(x.get("tip_mm") or x["point_mm"], tip_end)
                     <= derived.same_m(B, x.get("sigma_p", 0), sigma) * MM)]
        out = dict(result)
        out["predicted_contact_s"] = a["predicted_s"]
        out["contact_prediction_source"] = derived.prediction_source(st()["acts"])
        out.update(outcome_class=oc, same_as=len(same), contact_s=run_s, contact_total_s=total_s,
                   moved_under_load_mm=moved, moved_threshold_mm=B.rigid(a["q"], a["d"], float(result.get("peak_force_n") or 0)) * MM)
        out["outcome_class_why"] = f"{B.src('rigid')[0]}: travel under load counts as motion over {out['moved_threshold_mm']:g} mm at the measured peak force"
        if same:
            out["same_as_calls"] = [x["call"] for x in same]
        if a.get("hold_note"):
            out["hold_note"] = a["hold_note"]
        held = total_s if a["rounds"] else run_s
        if held is not None and a.get("asked_at_s") is not None:
            # how long the tip stayed on the part after the stop was asked: the next stop is asked that much earlier
            st()["ease_s"] = max(float(st().get("ease_s") or 0.0), float(held) - float(a["asked_at_s"]))
        if held is not None and a["long_s"] is not None and held >= a["long_s"]:
            if result.get("stopped") == "long_press":
                out["long_press_stop"] = (f"stopped after {held:.1f} s of contact, near the device's long_press_s "
                                          f"({a['long_press_s']:g} s)")
            else:
                out["long_press_stop"] = (f"this press kept contact for {held:.1f} s, past the measured stop time for the "
                                          f"device's long_press_s ({a['long_press_s']:g} s), and did not "
                                          "stop early: the device may have changed state; read its cue before the "
                                          "next act")
        contact_ok = oc in ("touched_no_give", "gave", "moved_under_load")
        rec = {"call": ex._calls, "primitive": name, "epoch": ex.epoch, "point_mm": point, "outcome_class": oc,
               "contact_ok": contact_ok, "device": None, "t_start": a["t0"], "t_end": ex.sim_time(),
               "device_outcome": None, "cue_after": None, "cue_changed": False, "tip_mm": tip_end,
               "sigma_p": sigma, "contact_s": total_s if a["rounds"] else run_s, "predicted_s": a["predicted_s"]}
        if entry is not None:
            dev = de.ident(entry)
            rec["device"] = dev
            after = None
            if a["watch_s"] != 0:
                after = ex._dv_cue_after(entry, ex._dv_wait(entry)[0] if a["watch_s"] is None else float(a["watch_s"]))
            skipped = {"verdict": "unobservable", "reason": "cue_watch_s was 0"}
            late = {"verdict": "unobservable", "reason": "not_read",
                    "why": "the act was tied to this device only by where it ended, so no reading came before it"}
            before = a["before"] or (skipped if a["watch_s"] == 0 else late)
            after = after or skipped
            repeats = sum(1 for x in st()["acts"] if x["device"] == dev and x["epoch"] == ex.epoch
                          and x["contact_ok"] and x["device_outcome"] in ("no_change", "unpowered_suspected")
                          and cue.cue_class((x["cue_after"] or {}).get("verdict")) == "dark")
            outcome, why = device_outcome(before, after, a["expect"], contact_ok, repeats)
            rec.update(device_outcome=outcome, cue_after=after, cue_changed=cue_changed(before, after))
            out.update(device_id=dev, device_attributed=how, device_outcome=outcome, device_outcome_why=why,
                       cue_before=before, cue_after=after)
            loc = de.cue_located(ex, entry)
            if outcome == "other_transition" and went_dark(before, after) and loc is not None:
                # the dark reading may be the cue gone with a moved part: it no longer says "off" from that point
                st()["doubt"][dev] = {"since_call": ex._calls, "loc_id": loc["id"], "loc_call": loc["at_call"],
                                      "label": loc["label"]}
                if (st()["cue"].get(dev) or {}).get("id") == after.get("watch_id"):
                    st()["cue"].pop(dev, None)
                out["cue_note"] = (f"the cue's located point ('{loc['label']}') now reads dark: until the cue is "
                                   "located anew (under a new label for that part), a dark reading there is "
                                   "unobservable (cue_may_have_moved)")
            dark_now = outcome in ("no_change", "unpowered_suspected") and cue.cue_class(after.get("verdict")) == "dark"
            if contact_ok and dark_now and repeats + 1 >= 2:
                key = "hint" if "hint" not in out else "device_hint"
                out[key] = POWER_HINT
                st()["power"][dev] = {"value": "suspected_off", "source": f"{repeats + 1} acts with contact and no "
                                      "change in a dark cue", "seen_at_s": round(rec["t_end"], 3)}
        st()["acts"].append(rec)
        ex.note_event("device_act_outcome", primitive=name, device=rec["device"], outcome_class=oc,
                      device_outcome=rec["device_outcome"], same_as=len(same))
        predicted, items = (derived.press_contact_s(B, kwargs, B.thing(a["device"]), a["q"], a["d"], [])
                            if name == "press" and a["d"] is not None else (B.t_det(), []))
        out["periods"] = int(round((ex.sim_time() - a["t0"]) / B.dt()))
        if not items:
            items = [{"what": "force", "value": used_force, "unit": "N", "over_mm": None,
                      "source": "contact", "evidence": "force actually used by the finding touch", "lift": "state force_n"}]
        return B.report(out, items)

    return pre, tick, post


def setup(ex):
    ex.state.setdefault(PKG, new_state())
    pre, tick, post = make_hooks(ex)
    ex.add_pre(pre, names=["touch", "press"])
    ex.add_tick(tick)
    ex.add_post(post, names=["touch", "press"])
