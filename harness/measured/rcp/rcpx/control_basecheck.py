# SPDX-License-Identifier: AGPL-3.0-only
"""The base check after a push: did the push move the robot's base?

It compares the standard look with its reference. It is DUE only when the push could have shifted the base by
more than the stored points are good to: peak force x the base's give (mm per N: measured by measure_base_give;
until then no more than this push's own travel under load per newton, since a base that shifted shows as tip travel)
passes the smallest error of the points robot://world holds. It never runs on a base stated fixed or read rigid,
without a reference, or when the caller asked base_check "defer" or "skip".

Every move it makes is one the operator did not command. Each is planned by its direction against the part, with the
contact stop armed:

1. leave (kind retreat, away from the part): the arm retraces the way it came (its history, newest first) until
   every part of it stands off every stored contact and located point and the point just pushed by the bounds'
   standoff; when the history ends first it goes on straight back along the caller's line (the reverse of the
   push: `back`); when that is not clear either, or a leg stops, the check stops there and says so;
2. to_standard (kind transit, away from the part): the joint move to the standard pose, then the standard look;
3. back (kind approach, TOWARD the part: it returns to the pose beside it, so the bounds slow it inside the part's
   band): the joint move back to the pose the leave reached.

base_moved reports each leg (stopped, peak_force_n, tip_mm, and where a stop was felt), the largest force of all of
them, where the arm ended and why. A stop of any leg is stored in robot://world as a guard stop. No robot is named
here.
"""

import numpy as np

from rcpx import bounds
from rcpx.control_base import _mm

MM = 1000.0
MODES = ("auto", "defer", "skip")


class BaseCheckMixin:
    def _base_fixed(self):
        """True when the base is known not to give: the owner says fixed, or measure_base_give read rigid."""
        return self.cfg["base_kind"] == "fixed" or bool(self.base_give and self.base_give.get("verdict") == "rigid")

    def _base_due(self, stop, peak_n, mode=None, moved_m=None, d=None):
        """A push that could have shifted the base past what the stored points are good to (see the module
        docstring). moved_m: the tip's travel under load in this push along d (None: not known, taken as what the
        arm's own give at that force explains, so a push with no measured travel is never due by itself)."""
        if self._base_fixed():
            return False
        B = bounds.of(self)
        s = self._sense()
        peak = float(peak_n)
        own = B.rigid(s.q, d, peak)   # the tip travel the arm's own give explains at that force
        tol = self._point_error_m()
        tol = own if tol is None else tol
        if self.base_give and self.base_give.get("mm_per_n") is not None:
            shift = peak * float(self.base_give["mm_per_n"]) / MM
        else:
            shift = own if moved_m is None else float(moved_m)
        return bool(shift > tol)

    def _leg(self, name, r, start_tip):
        """One leg of the check as base_moved reports it."""
        s = self._sense()
        if "refused" in r:
            return {"leg": name, "stopped": "refused", "why": r["refused"].get("why"), "peak_force_n": 0.0,
                    "tip_mm": _mm(s.tip), "moved_mm": round(float(np.linalg.norm(s.tip - start_tip)) * MM, 1)}
        peak = r.get("peak_force_n", r.get("peak_n"))
        out = {"leg": name, "stopped": r.get("stopped", r.get("stop")), "peak_force_n": round(float(peak or 0.0), 2),
               "tip_mm": _mm(s.tip), "moved_mm": round(float(np.linalg.norm(s.tip - start_tip)) * MM, 1)}
        c = r.get("contact")
        if isinstance(c, dict) and out["stopped"] not in ("goal", None):
            out["felt"] = {k: c.get(k) for k in ("point_mm", "force_n", "surface", "source", "likely_links")
                           if c.get(k) is not None}
            if c.get("point_mm") is not None:
                e = self.world.add("contact", "base check stop", point_mm=c["point_mm"], source="base check guard",
                                   data={"normal_est": c.get("normal_est"), "force_n": c.get("force_n")})
                out["felt"]["id"] = e.get("id")
        return out

    def _base_check(self, hard, q_back, back=None, mode="auto", pushed_mm=None, intent=None, part=None):
        """See the module docstring. hard: the check is due (see _base_due); q_back: kept for callers; back: the
        line (m, frame B) to go on along when the history does not lead clear (the reverse of the push);
        pushed_mm: the point just pushed (frame B, mm), kept clear of like a stored contact; intent: the call's
        intent (the operator's word passes to every leg); part: the thing just pushed, as the bounds hold it (the
        legs that leave it are planned away from it, the last leg toward it). Returns base_moved."""
        B = bounds.of(self)
        mode = mode if mode in MODES else "auto"
        due = bool(hard) and not self._base_fixed()
        out = {"checked": False, "moved": None, "due": due}
        if mode == "skip":
            return {**out, "due": False, "why": "skipped as asked (base_check skip)"}
        if self._base_fixed():
            return {**out, "why": "the base is fixed: no check needed"}
        if not hard:
            return {**out, "why": "a light push: at this force the base cannot have shifted by more than the stored "
                                  "points are good to, so no check was made"}
        if self._std_ref is None:
            return {**out, "why": "no standard look reference: take standard_look(set_reference=true) once at the "
                                  "start; then standard_look() compares with it"}
        if mode == "defer":
            return {**out, "why": "deferred as asked: take standard_look() once the arm is clear of the part"}
        known = list(self._known_points())
        if pushed_mm is not None:
            known.append(np.asarray(pushed_mm, dtype=float) / MM)
            part = B.thing(known[-1]) if part is None else part
        known = np.asarray(known, dtype=float).reshape(-1, 3)
        legs, s0 = [], self._sense()
        clear = B.standoff(0.0, part, s0.q)   # the tool's half width and the part's own error: off the part
        word = {k: v for k, v in (intent or {}).items() if k == "clear"}

        def gap(s):
            return self._nearest_known(s.q, known)[0] if len(known) else float("inf")

        def finish(why, **more):
            peak = max([lg["peak_force_n"] for lg in legs] or [0.0])
            e = self._sense()
            off = gap(e)
            return {**out, "why": why, "legs": legs, "peak_force_n": round(peak, 2), "tip_mm": _mm(e.tip),
                    "clear_mm": None if not np.isfinite(off) else round(off * MM, 1), **more}

        if gap(s0) < clear:
            r = self._retreat_impl(len(self._history), until=lambda s: gap(s) >= clear, intent=word)
            legs.append(self._leg("leave", r, s0.tip))
            if "refused" in r or r["stopped"] != "goal":
                return finish("leaving the part by the way the arm came stopped "
                              f"({legs[-1]['stopped']}); the arm stays there", returned_to="where the leave stopped")
            s1 = self._sense()
            size = float(np.linalg.norm(back)) if back is not None else 0.0
            if gap(s1) < clear and size > 0.0:
                # the history ran out first: on straight back along the caller's line (the reverse of the push)
                u = np.asarray(back, dtype=float) / size
                r = self._cart_move(s1, u * (size + clear), None, None, True, True, kind="retreat",
                                    away_from=part, intent=word)
                legs.append(self._leg("back_off", r if "refused" in r else {**r, "stopped": r["stop"]}, s1.tip))
                if "refused" in r or r["stop"] not in ("goal", "limit"):
                    return finish("backing off the part stopped "
                                  f"({legs[-1]['stopped']}); the arm stays there", returned_to="where it stopped")
            if gap(self._sense()) < clear:
                return finish(f"the way the arm came does not lead {clear * MM:.1f} mm clear of the part: no check "
                              "made; move clear, then take standard_look()", returned_to="the end of the way back")
        n_hist = len(self._history)
        s1 = self._sense()
        q_clear = s1.q.copy()
        lk = self._standard_look_impl(False, record_contact=False,
                                      move_kw={"kind": "transit", "away_from": part, "intent": word})
        mv = lk.pop("_move", None) if isinstance(lk, dict) else None
        legs.append(self._leg("to_standard", mv if mv is not None else lk, s1.tip))
        # the way back goes TOWARD the part: an approach, slowed inside the part's band by the bounds
        toward = {"kind": "approach", "target": part, "intent": word}
        if "refused" in lk:
            back = self._joint_move(q_clear, None, record_contact=False, **toward)
            legs.append(self._leg("back", back, self._sense().tip))
            del self._history[n_hist:]
            return finish("the move to the standard pose: " + lk["refused"]["why"],
                          returned_to="the pose the leave reached, clear of the part")
        rel = lk.get("relocalize") or {}
        s2 = self._sense()
        back = self._joint_move(q_clear, None, record_contact=False, **toward)
        legs.append(self._leg("back", back, s2.tip))
        del self._history[n_hist:]   # the two legs cancel: a later retreat goes on along the way the arm came
        res = finish(rel.get("why") or ("compared with the reference" if rel else "no comparison"),
                     returned_to="the pose the leave reached, clear of the part")
        res.update(checked=bool(rel) and "refused" not in rel, look_id=lk.get("look_id"), frame=lk.get("frame"),
                   moved=rel.get("moved"), yaw_deg=rel.get("yaw_deg"), shift_mm_est=rel.get("shift_mm_est"),
                   confidence=rel.get("confidence"))
        if "refused" in rel:
            res["why"] = rel["refused"]["why"]
        return res
