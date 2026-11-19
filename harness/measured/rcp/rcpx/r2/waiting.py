# SPDX-License-Identifier: AGPL-3.0-only
"""Package "waiting": waiting safely.

Tools:
- hold(seconds): the world runs while every joint keeps its COMMANDED goal (never its present position, so a
  loaded arm does not creep onto its load); returns the drift from the goal. The time is the caller's: there is no
  cap but the session's budget. held is judged against the arm's rest angle at the pose (waiting_limits.py).
- release_check(joints, stay_s): which joints stay put without motor torque at this pose (waiting_release.py).
  The backend's torque share is whole-arm, so each joint is released by setting its goal to its present position
  for the time a free joint would need to turn the re-hold angle, and the sag is compared with the body model's
  prediction; the result says so and marks what is predicted. When no joint reads as dropping, a real cut of the
  whole arm's torque (the one cut the interface has) decides arm_holds. An arm that only stands balanced over its
  own weight is not cut: no short cut can tell it from one that rests, so the body model answers (balanced) and
  arm_holds is false. stay_s is the time the caller needs the arm to stay; every verdict is for that horizon.
- wait_for_light(max_s): parks (from any pose), then reads the camera's light level at a fixed interval of world
  time and returns {can_see, light_returned, waited_s, wait_bound, history}; one look at the end is the evidence
  either way. The waiting is world time: ex.uncharge takes it off the session's active clock (the park's motion
  stays charged). A wait lasts as long as the caller states; when the runner states the task's deadline
  (ex.context deadline_s) waits are cut to what is left of it, a call past it returns at once with stopped
  "bound" and the next step is the cannot claim with reason dark.
Hooks:
- park from any pose (waiting_park.py): a pre hook on park lifts the tool straight up, guarded, when the park's
  own path check would refuse for the surface under the arm; a post hook lifts and parks again when a park that
  started on that surface stopped short on a contact. A park whose plain path runs is left alone.
- light (waiting_light.py): an idle hook and a look listener keep the light history; robot://state gains
  light {can_see, since_s, history, waited_s, wait_left_s}. The dark gate is keyed on the NEWEST light reading
  (read again from one frame when there is none or it is older than FRESH_S): a primitive that needs light now
  (standard_look, and names in ex.state["waiting"]["needs_light"]) is refused with check can_see, and a move toward
  the world (TOWARD_WORLD, and names in ex.state["waiting"]["toward_world"]; move_joints to home or park excepted)
  with check dark, while it reads dark; round one's dark mark follows the same reading, so one dark look never
  latches once the light reads dim or ok again. A dark look or a refusal for darkness gains a pointer: to
  wait_for_light, or to the cannot claim once a stated deadline has passed.
A hold and a wait end what the robot knows of free space (rcpx.bounds space book): a person or a device may have
moved in it meanwhile.

No robot is named and no simulator ground truth is read: joint readings, the wrist camera and the body model only.
"""

import math

import numpy as np

from rcpx import surfaces, bounds
from rcpx import bounds_ratios as R
from rcpx import bounds_laws as laws
from rcpx.r2 import waiting_limits as limits
from rcpx.context import refusal as context_refusal
from rcpx.control_base import primitive
from rcpx.r2 import waiting_light as light
from rcpx.r2 import waiting_park as parking
from rcpx.r2 import waiting_release as release

ID = "I28"
PKG = light.PKG
MM = 1000.0
HOLD_CHUNK_S = 5.0       # hold lets the world run in pieces this long (the idle hooks run after each)
ENOUGH = ("ok", "dim")
RANK = {"dark": 0, "dim": 1, "ok": 2}


def _number(value, lo):
    """float(value) when it is a finite number at least lo, else None."""
    if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)):
        return None
    v = float(value)
    return v if math.isfinite(v) and lo <= v else None


def refusal(check, why, hint):
    return {"said": str(why).rstrip(".") + ".", **context_refusal(check, why, hint)}


class Mixin:
    # ------------------------------------------------------------------ hold
    @primitive
    def hold(self, seconds=None):
        t = _number(seconds, self.dt)
        if t is None:
            offer = max(self.dt, bounds.of(self).t_settle())
            if _number(seconds, 0) is not None and seconds > 0:
                offer = max(self.dt, seconds)
            return refusal("seconds", f"hold needs finite seconds at least one control period ({self.dt:g} s)",
                           f"hold(seconds={offer:.2g})")
        goal = np.asarray(self._q_des, dtype=float).copy()
        A, source = limits.rest_angle(self, goal)
        s0 = self._sense()
        t0 = self.sim_time()
        worst, left = float(np.max(np.abs(s0.q - goal))), t
        while left > self.dt / laws.TWO:
            step = min(HOLD_CHUNK_S, left)
            self.idle(step)                # the world runs; the arm keeps its commanded goal
            left -= step
            worst = max(worst, float(np.max(np.abs(self._sense().q - goal))))
        s = self._sense()
        drift = s.q - goal
        big = int(np.argmax(np.abs(drift)))
        out = {"said": f"The arm kept its commanded goal for {t:g} s and stayed inside its rest angle.",
               "held_s": round(self.sim_time() - t0, 3), "asked_s": t,
               "drift_deg": {n: round(math.degrees(float(v)), 3) for n, v in zip(self.model.joint_names, drift)},
               "max_drift_deg": round(math.degrees(float(np.max(np.abs(drift)))), 3),
               "worst_drift_deg": round(math.degrees(worst), 3),
               "moved_deg": round(math.degrees(float(np.max(np.abs(s.q - s0.q)))), 3),
               "tip_moved_mm": round(float(np.linalg.norm(s.tip - s0.tip)) * MM, 2),
               "held": bool(worst <= A),
               "force_n": round(float(s.Fmag), 2), "tip_mm": [round(float(x) * MM, 2) for x in s.tip],
               "light": light.doc(self)["can_see"],
               "note": "drift is the present angle minus the commanded goal; the goal was kept the whole time; "
                       f"worst_drift_deg is the largest drift read at the checks every {HOLD_CHUNK_S:g} s"}
        out["drift_limit_deg"] = round(math.degrees(A), 4)
        out["limits"] = {"drift_limit_deg": {"value": round(math.degrees(A), 4), "source": source}}
        bounds.of(self).space.drop("hold")
        if not out["held"]:
            out["said"] = (f"The goal was kept for {t:g} s, but joint {self.model.joint_names[big]} stands "
                           f"{abs(math.degrees(float(drift[big]))):.2f} deg from it, past the rest angle of "
                           f"{math.degrees(A):.2f} deg: something loads the arm; ease() lets go.")
            out["why"] = (f"joint {self.model.joint_names[big]} is {abs(math.degrees(float(drift[big]))):.2f} deg from "
                          "its goal: something loads the arm, or the goal leads into a contact")
        return out

    # ------------------------------------------------------------------ release_check
    @primitive
    def release_check(self, joints=None, stay_s=None):
        names = self.model.joint_names
        if joints is None:
            picked = list(range(self.model.n))
        else:
            if isinstance(joints, str):
                joints = [joints]
            hint = "release_check() tests every joint; joints: " + ", ".join(names)
            if not isinstance(joints, (list, tuple)) or not joints:
                return refusal("joint", "joints must be a list of joint names", hint)
            unknown = [x for x in joints if not isinstance(x, str) or x not in names]
            if unknown:
                return refusal("joint", f"no joint named {unknown[0]!r}", hint)
            picked = sorted({names.index(x) for x in joints})
        s = self._sense()
        if limits.moving(self, s):
            return refusal("moving", "the arm is still moving: the test needs it at rest",
                           f"hold(seconds={bounds.of(self).t_settle():.2g}) first, then release_check()")
        pushes, who, lead = limits.pushing(self, s)
        if self._pressing or pushes:
            return refusal("pressing", f"the arm is pushing on something (the goal of joint {who} leads it by "
                           f"{math.degrees(lead):.2f} deg, about {s.Fmag:.1f} N at the tip): the test would read "
                           "the push, not gravity", "ease() first, then release_check()")
        B = bounds.of(self)
        A, rest_source = limits.rest_angle(self, s.q)
        lim, rehold_sources = limits.rehold(self, s.q)
        initial = release.predict(self, s.q, lim, max(self.dt, B.t_settle()))
        measurable = [j for j in picked if initial[j]["measurable"]]
        offered, offer_source = limits.stay_offer(self, s.q, measurable, lim)
        stay = offered if stay_s is None else _number(stay_s, self.dt)
        if stay is None:
            return refusal("stay_s", "stay_s must be finite and at least one control period",
                           f"release_check(stay_s={offered:.2g})")
        stay_source = "offered: " + offer_source if stay_s is None else "stated"
        goal = np.asarray(self._q_des, dtype=float).copy()
        t0 = self.sim_time()
        rows = {}
        for j in sorted(picked, reverse=True):      # the far joints first: they carry the least
            pred = release.predict(self, self._sense().q, lim, stay)[j]
            rows[j] = release.release_one(self, j, goal, pred, lim, stay)
        # joints with no load to measure: do they hang or lie neutral, or stand balanced over their load?
        tipped, kept = release.unstable_balance(self, self._sense().q,
                                                [j for j in picked if rows[j]["basis"] == "predicted"], stay)
        for j in tipped:
            release.mark_balanced(rows[j])
        kept = [names[j] for j in kept]
        rows = [rows[j] for j in picked]
        holding = [r["joint"] for r in rows if r["holds"] is True]
        falling = [r["joint"] for r in rows if r["holds"] is False]
        unsure = [r["joint"] for r in rows if r["holds"] is None]
        tippy = [r["joint"] for r in rows if r["balanced"]]
        whole = len(picked) == self.model.n
        if not whole:
            cut = {"done": False, "why": "only some joints were tested: the whole-arm cut needs them all"}
        elif falling:
            cut = {"done": False, "why": "not tried: " + ", ".join(falling) + " would drop"}
        elif tippy:
            cut = {"done": False, "why": "not tried: " + ", ".join(tippy) + " stand balanced over their own weight "
                   "(predicted from the body model): an arm that is only balanced lies still through a short cut "
                   "and falls at the first push, so the cut could not show that it holds"}
        else:
            cut = release.cut_whole_arm(self, goal, lim, stay)     # a real cut: the interface has one for the whole arm
        arm_holds = bool(cut["done"] and cut["holds"])
        out = {"said": "", "arm_holds": arm_holds, "holding": holding, "not_holding": falling,
               "cannot_tell": unsure,
               "balanced": tippy, "held_by_friction": kept, "joints": rows, "whole_arm_cut": cut,
               "method": {"per_joint": "goal_at_present (emulated)", "whole_arm": "torque cut (real)"},
               "per_joint_torque_cut": False,
               "note": "this robot's interface cuts torque for the whole arm only, so each joint was released by "
                       "setting its goal to where it stood (no holding torque there, the other joints holding) for "
                       "the time a free joint would need to turn the re-hold angle (window_s is the longest); a "
                       "joint that sagged as one its motor carries was then followed down until it came to rest, "
                       f"passed the re-hold angle of {math.degrees(A):.3g} deg (re-held at once) or the stay time "
                       f"of {stay:g} s was up. moved_deg and sag_deg are measured; the fields named in predicted "
                       "come from the body model for an arm in free space (free_drop_deg: what a real torque cut "
                       "of the stay time would do; balanced: the joint stands over its load and would fall once "
                       "tipped). Each row's basis says whether its verdict was measured or, for a joint with no "
                       "load to measure, predicted. arm_holds comes from a real cut of the whole arm's torque "
                       "(until it lies still, re-held at once past the same angle), made only when no joint read "
                       f"as dropping or as balanced. Every verdict is for {stay:g} s: a sink slower than "
                       f"{math.degrees(A) / stay:.3g} deg/s cannot reach the re-hold angle in that time and is not "
                       "told from rest; state a longer stay_s for a longer watch",
               "predicted": list(release.PREDICTED),
               "window_s": max(r["window_s"] for r in rows), "rehold_deg": round(math.degrees(A), 4),
               "stay_s": stay, "stay_source": stay_source, "slowest_sink_deg_s": math.degrees(A) / stay,
               "limits": {"rehold_deg": {"value": math.degrees(A), "source": rest_source},
                          "stay_s": {"value": stay, "source": stay_source},
                          "rest_step_deg": {"value": math.degrees(float(np.max(R.QUANT * B.joint_quantum()))),
                                            "source": "two steps of the joint encoders"},
                          "prediction_error": {"value": R.REL, "source": "the accepted relative model error"}},
               "back_within_deg": round(math.degrees(float(np.max(np.abs(self._sense().q - s.q)))), 4),
               "took_s": round(self.sim_time() - t0, 3)}
        if arm_holds:
            out["advice"] = (f"the arm lay still with its torque off for {cut['cut_s']:g} s (it moved "
                             f"{cut['moved_deg']:g} deg): a torque cut is safe at this pose for the horizon of "
                             f"{stay:g} s this check answers for")
            if kept:
                out["advice"] += ("; " + ", ".join(kept) + " stand over their own weight and only joint friction "
                                  "keeps them there (predicted from the body model): a push would tip them")
        elif cut["done"]:
            out["advice"] = (f"do not cut torque at this pose: with the whole arm's torque off it moved "
                             f"{cut['moved_deg']:g} deg in {cut['cut_s']:g} s and " +
                             ("was re-held" if cut["reheld"] else "was still creeping"))
        elif tippy and not falling and whole:
            out["advice"] = ("do not cut torque at this pose: " + ", ".join(tippy) + " stand balanced over their own "
                             "weight, nothing carries them, and the arm would fall at the first push; park, or rest "
                             "the arm on a surface, and check again")
        else:
            out["advice"] = ("do not cut torque at this pose: " + (", ".join(falling) + " would drop" if falling else
                                                                   cut["why"]) + "; park, or rest the arm on a "
                             "surface, and check again")
        if arm_holds:
            out["said"] = (f"The arm stays where it is with its torque off: it moved {cut['moved_deg']:g} deg in a "
                           f"real cut, inside the re-hold angle of {math.degrees(A):.3g} deg, judged for {stay:g} s.")
        elif falling:
            out["said"] = ("Keep the torque on: " + ", ".join(falling) + " would drop; park or rest the arm on a "
                           "surface, then check again.")
        elif tippy:
            out["said"] = ("Keep the torque on: " + ", ".join(tippy) + " stand balanced over their own weight and "
                           "would fall at the first push.")
        elif cut["done"]:
            out["said"] = (f"Keep the torque on: with the whole arm's torque off it moved {cut['moved_deg']:g} deg "
                           f"in {cut['cut_s']:g} s and did not come to rest.")
        else:
            out["said"] = "Only some joints were tested, so the check does not answer for the whole arm."
        return out

    # ------------------------------------------------------------------ wait_for_light
    @primitive
    def wait_for_light(self, max_s=None, every_s=None, enough="ok", park=True, clear=None):
        every = light.SAMPLE_EVERY_S if every_s is None else _number(every_s, self.dt)
        if every is None:
            return refusal("every_s", "every_s must be finite and at least one control period",
                           f"wait_for_light(max_s={light.offer(self):.2g}, every_s={light.SAMPLE_EVERY_S:.2g})")
        left = light.wait_left_s(self)
        limit = left if max_s is None and left is not None else _number(max_s, self.dt)
        if limit is None:
            offer = light.offer(self, every)
            return refusal("max_s", "nothing states when light returns: state the time to wait",
                           f"wait_for_light(max_s={offer:.2g})")
        intent = None if clear is None else {"clear": clear}
        if enough not in ENOUGH:
            return refusal("enough", f"enough is ok or dim, not {enough!r}",
                           f"wait_for_light(max_s={light.offer(self, every):.2g}, enough='ok') waits for full light")
        st = light.st_of(self)
        L = st["light"]
        need = RANK[enough]
        left = light.wait_left_s(self)        # time left to the stated deadline
        allowed = limit if left is None else min(limit, left)
        L["busy"] = True                    # this call takes the samples itself
        try:
            t_call = self.sim_time()
            now = light.sample(self, "wait")
            if now is None:
                return refusal("camera", f"the wrist camera gave no frame ({L['problem']}): the light cannot be read",
                               f"wait_for_light(max_s={light.offer(self, every):.2g}); if it keeps failing the camera is down")
            samples, waited, parked, stopped = 1, 0.0, None, "light"
            was_low = RANK[now["can_see"]] < need
            if was_low:
                if park:
                    r = parking.park_from_anywhere(self, intent)
                    parked = {k: r[k] for k in ("stopped", "error_deg", "lifted_first", "first_try", "contact",
                                                "refused", "why") if r.get(k) is not None}
                    now = light.sample(self, "wait")    # the view from the rest pose is the one the wait watches
                    samples += 1
                current_left = light.wait_left_s(self)
                allowed = limit if current_left is None else min(limit, current_left)
                t_wait = self.sim_time()
                while now is not None and RANK[now["can_see"]] < need and waited < allowed - self.dt / laws.TWO:
                    self.idle(min(every, allowed - waited))
                    waited = self.sim_time() - t_wait
                    now = light.sample(self, "wait")
                    samples += 1
                if now is None:
                    stopped = "camera"
                elif RANK[now["can_see"]] < need:
                    stopped = "bound" if light.spent(self) else "max_s"
            st["waited_s"] += waited
            st["wait_calls"] += 1
            # waiting for light is world time, not active work: the session does not charge it (ex.uncharge)
            uncharge = getattr(self, "uncharge", None)
            if callable(uncharge) and waited > 0:
                uncharge(waited, "wait_for_light")
            lk = None
            if stopped != "camera":
                try:
                    lk = self._look_impl("wait_for_light")
                except Exception as exc:    # the arm parked and the world ran: a result that says so, not a refusal
                    stopped, L["problem"] = "camera", f"the look at the end failed: {type(exc).__name__}: {exc}"
                    self._journal("r2_problem", package=PKG, where="wait_for_light look", trace=L["problem"])
        finally:
            L["busy"] = False
        light.sync_looks(self)
        light.mark_dark(self, L["last"])
        cs = lk["can_see"] if lk is not None else (L["last"] or {}).get("can_see")
        enough_now = bool(lk is not None and RANK[cs] >= need)
        out = {"said": "", "can_see": cs, "enough": enough_now, "asked": enough,
               "light_returned": bool(was_low and enough_now),
               "waited_s": round(waited, 3), "world_wait_s": round(waited, 3),
               "max_s": limit, "every_s": every, "samples": samples, "stopped": stopped,
               "parked": parked if parked is not None else False,
               "world_s": round(self.sim_time() - t_call, 3),
               "charged": {"waited_s": 0.0, "note": "the waiting is world time, not charged to sim_seconds (the "
                                                    "park's motion is)"},
               "wait_bound": {"waited_total_s": round(st["waited_s"], 3), "deadline_s": self.context.get("deadline_s"),
                              "left_s": light.wait_left_s(self)},
               "look": None if lk is None else {k: lk.get(k) for k in ("look_id", "frame", "annotated", "brightness",
                                                                        "can_see")},
               "history": light.history(self), "light": {k: v for k, v in light.doc(self).items()
                                                         if k in ("can_see", "brightness", "since_s")}}
        if waited > 0:
            bounds.of(self).space.drop("wait")
        if left is not None and limit > left and was_low:
            out["max_s_note"] = f"asked {limit:g} s; the task deadline left {left:g} s"
        if enough_now and waited > 0:
            out["said"] = f"The light is back ({cs}) after {waited:g} s of waiting; look again before acting."
        elif enough_now:
            out["said"] = f"There is light enough ({cs}): nothing to wait for."
        elif stopped == "bound":
            out["said"] = (f"Still {cs} and the task's deadline has passed: claim cannot with reason dark, citing "
                           "this answer.")
        else:
            out["said"] = (f"Still {cs} after {waited:g} s of waiting: wait again or claim cannot with reason "
                           "dark.")
        if stopped == "camera":
            out["said"] = "The camera stopped giving frames during the wait: the light could not be read."
            out["why"] = f"the wrist camera stopped giving frames ({L['problem']}): the last reading was {cs}"
            out["next"] = "call wait_for_light again; if it keeps failing the camera is down"
        elif not enough_now and stopped == "bound":
            out["why"] = (f"the light did not return: still {cs} after {st['waited_s']:g} s of waiting in this session "
                          "(the task deadline has passed)")
            out["next"] = ("claim now with success false and cannot, reason \"dark\", evidence the seq of this answer: "
                           "that is the correct outcome when the scene stays dark; moves toward the world stay refused "
                           "while it reads dark")
        elif not enough_now:
            out["why"] = f"the light did not return in {out['waited_s']:g} s: still {cs}"
            out["next"] = (f"wait_for_light(max_s={light.offer(self, every):.2g}) to wait again {light.wait_words(self)}, or "
                           "claim with success false and cannot, reason \"dark\", citing this answer's seq; moves "
                           "toward the world stay refused while it reads dark")
        elif waited > 0:
            out["next"] = "the light is back: go back to the work and look again before acting"
        elif parked is not None:
            out["next"] = (f"the light reads {cs} from the rest pose and read less where the arm stood: the room is "
                           "lit, the camera was covered or faced something dark there; look from another pose "
                           "before going back to that one")
        return out


def setup(ex):
    st = light.st_of(ex)
    light.setup(ex)
    parking.setup(ex, st)


def _patch_state(ex, session, content):
    if not isinstance(content, dict):
        return None
    return {**content, "light": light.doc(ex)}


RESOURCE_PATCH = {"robot://state": _patch_state}

_S = surfaces
TOOLS = [
    _S._tool("hold",
             "Wait in place while the world runs: every joint keeps its commanded goal for the given seconds (the "
             "arm does not relax onto whatever loads it), then the result gives each joint's drift from its goal "
             "and held true or false: it waits for a device, a person or a timer.",
             _S._obj({"seconds": {"type": "number", "exclusiveMinimum": 0, "description": "how long to hold, s"}},
                     ["seconds"]),
             {"seconds": HOLD_CHUNK_S},
             f"keep the present pose for {HOLD_CHUNK_S:g} s of world time and report how far the joints drifted",
             brief="Wait in place for some seconds: the joints keep their goals; returns each joint's drift."),
    _S._tool("release_check",
             "Test which joints stay put without motor torque at the present pose (for a torque cut); each "
             "joint in turn loses its holding torque for the time a free joint would need to turn the re-hold "
             "angle (re-held at once if it moves further than the arm's rest angle at this pose) and its sag is "
             "compared with the body model; if no joint reads as dropping or as balanced upright, the whole arm's "
             "torque is then really cut (re-held the same way) to decide arm_holds; returns holding, not_holding, "
             "cannot_tell, balanced and arm_holds, each for the stay time stay_s.",
             _S._obj({"joints": {"type": "array", "items": {"type": "string"},
                                 "description": "joint names to test (robot://body lists them); leave out for all"},
                      "stay_s": {"type": "number", "exclusiveMinimum": 0,
                                 "description": "how long the arm must stay with its torque off, s: every verdict is "
                                                "for this time; leave out and the tool takes the time its own "
                                                "test needs and says so"}}),
             {}, "test every joint: would the arm stay where it is if its motors let go",
             brief="Before a torque cut: which joints stay put without motor torque here, and arm_holds."),
    _S._tool("wait_for_light",
             "When a look says dark: park the arm (from any pose) and read the camera's light level at a fixed "
             "interval of world time until it is back or max_s has passed; returns can_see (ok, dim or dark), "
             "light_returned, waited_s, the light history and a fresh look. Waiting is world time and is not "
             "charged to sim_seconds; wait as long as you state, until a task deadline when stated. If "
             "the light did not return, claim success false with cannot, reason dark, citing this answer's seq: that "
             "is the correct outcome when it stays dark. Moves toward the world are refused while the newest light "
             "reading is dark.",
             _S._obj({"max_s": {"type": "number", "exclusiveMinimum": 0,
                                "description": "longest time to wait, s (your own number: there is no upper limit; "
                                               "it may be left out only when the task states a deadline)"},
                      "every_s": {"type": "number", "exclusiveMinimum": 0,
                                  "description": "seconds between light readings (leave out for the tool's own "
                                                 "interval)"},
                      "enough": _S._str("the light level that ends the wait: ok (default) or dim (large things can "
                                        "be located, small states cannot be decided)", list(ENOUGH)),
                      "park": _S._bool("park the arm before waiting (default true; false waits in place)"),
                      "clear": {"anyOf": [{"type": "boolean"}, {"type": "object"}],
                                "description": "true after a look that shows the way to the rest pose is clear "
                                               "(or {look: id}): recorded; it changes no speed"}}),
             {"max_s": light.SAMPLE_EVERY_S},
             f"park, then wait up to {light.SAMPLE_EVERY_S:g} s (one light reading; state your own time) for the "
             "light to come back",
             brief="When a look says dark: park and wait until the light is back or max_s passed; light history."),
    {"extends": "park", "args": {},
     "note": "Works from any pose: when the straight path to the rest pose is blocked by the surface under the "
             "arm, or a park that started on that surface stops short on a contact, the tool is lifted straight "
             "up, guarded, before the park (or before a second try), and the result carries lifted_first."},
]

PRACTICES = ("wait_for_light(max_s) parks and returns when a look reads light again (world time, not "
             "charged); if it does not, the claim says dark, citing the wait's seq. hold(seconds) waits with the "
             "joints on their goals. release_check says whether the arm holds with its torque cut (arm_holds). park "
             "works from any pose: a blocked plain path lifts the tool straight up first.")

__all__ = ["ID", "Mixin", "PRACTICES", "RESOURCE_PATCH", "TOOLS", "setup"]
