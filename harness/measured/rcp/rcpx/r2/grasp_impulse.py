# SPDX-License-Identifier: AGPL-3.0-only
"""Impulse of package grasp: a guarded tip stroke built on _push and ease.

The force budget comes from Bounds.f_allow by its tiers. The speed offer comes from Bounds.v_in and v_free.
A lower stated speed leaves more of the budget for steady push; a higher one is cut to the offer. The steady aim
reserves the strike term and covers servo lead, the force threshold and the relative force step of SPEC-R4 A1.1.
A known stop slope gives an impact estimate. An unresolved slope gives strike energy and claims no impact peak.

Peak_force_n is the estimate held over control readings. An impact shorter than one control period can peak
higher than those readings. The stroke ends by its measured stop distance; coast_watch records force during ease,
so contact first met in the coast is reported. Resistance must be strictly above the threshold when it is zero.
Travel_mm is the measured resting travel. Periods covers the stroke, ease and any base check.

Nothing here reads world truth."""

import mujoco
import numpy as np

from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx.bounds_laws import strike_energy
from rcpx.context import refusal
from rcpx.control_base import _mm, primitive
from rcpx.r2.grasp_hand import finite as _finite

MM = 1000.0


def _grasp_readable(out):
    """Round dimensions in direct results and Bounds items for the reader, SPEC-R4 A5."""
    if isinstance(out, list):
        return [_grasp_readable(value) for value in out]
    if not isinstance(out, dict):
        return out
    result = {}
    for key, value in out.items():
        if isinstance(value, (dict, list)):
            value = _grasp_readable(value)
        elif isinstance(value, float):
            if "_mm" in key or "_n" in key or key == "strike_mj" or (
                    key == "value" and out.get("unit") in ("mm", "mm/s", "N")):
                value = round(value, 2)
            elif key.endswith("_s") or key == "value" and out.get("unit") == "s":
                value = round(value, 3)
        result[key] = value
    return result


class ImpulseMixin:
    """A tip strike bounded by the offered arrival and remaining steady-push budget, SPEC-R4 A4.5."""
    def _grasp_tip_mass(self, q, d):
        """Effective mass (kg) of the arm at the tip along unit d (frame B) at joint pose q, from the robot's own
        model (links and rotor armature): a tip moving at v along d carries this mass times v into a contact."""
        m = self.model.mj
        dd = self._grasp_dd
        dd.qpos[:] = self._qfull
        dd.qpos[self._qadr] = q
        mujoco.mj_kinematics(m, dd)
        mujoco.mj_comPos(m, dd)
        mujoco.mj_crb(m, dd)
        M = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, dd, M)
        a = self.model.jac(q)[:3].T @ d
        inv = float(a @ np.linalg.solve(M[np.ix_(self._dadr, self._dadr)], a))
        return 1.0 / max(inv, 1e-9)

    def _grasp_stop_coeff(self, q, d, thing=None):
        """Read the stop slope and its source from Bounds alone, SPEC-R4 Q9."""
        return bounds.of(self).slope(q, d, thing)

    def _grasp_thing_at(self, B, p):
        """The thing a stroke that ends at p (m, frame B) is meant for: the stored point of robot://world that is
        the same spot (within its own error plus the tool's half width, SPEC-R4 A4.3 "same spot"; the nearest of
        those), so what an earlier touch or press showed of that part bounds the strike; else the bare point, of
        which nothing is known."""
        w_tool = B.standoff(0.0)
        best = None
        for e in self.world.query():
            if e.get("point_mm") is None or e.get("stale"):
                continue
            E = B.thing(e["id"])
            if E.point is None:
                continue
            gap = float(np.linalg.norm(np.asarray(E.point, dtype=float) - p))
            if gap <= (E.sigma_p or 0.0) + w_tool and (best is None or gap < best[0]):
                best = (gap, E)
        return best[1] if best is not None else B.thing(p)

    def _grasp_impulse_ok(self, q, d):
        """Impulse is possible only with positive capability and arrival offer, SPEC-R4 A4.3."""
        B = bounds.of(self)
        return B.f_cap(q, d) * (1 - B.e_tau()) > B.f_thr(0.0, 0.0) and B.v_in(None, q, d)[0] > 0

    @primitive
    def impulse(self, direction, travel_mm, speed_mm_s=None, frame="base", force_n=None, clear=None):
        """Strike at the offered or lower stated speed, then ease and report the actual contact, SPEC-R4 A4.5."""
        out = self._grasp_impulse_impl(direction, travel_mm, speed_mm_s, frame, force_n, clear)
        if out.get("refused"):
            r = out["refused"]
            said = r.get("why", "the strike was refused") + "; " + r.get("hint", "look before retrying")
        else:
            said = out.get("said", "the strike ended; inspect its force and contact before striking again")
        out = {"said": said, **out}
        return _grasp_readable(out)

    def _grasp_impulse_impl(self, direction, travel_mm, speed_mm_s, frame, force_n, clear):
        """Reserve force for arrival, servo lead and a resolvable steady push, SPEC-R4 A4.5/A1.1."""
        B = bounds.of(self)
        s = self._sense()
        d, bad = self._unit_in_base(direction, frame, s)
        if bad is not None:
            return bad
        if not _finite(travel_mm) or float(travel_mm) <= 0:
            return refusal("travel_mm", "travel_mm must be a finite number > 0", "state the stroke travel in mm")
        if speed_mm_s is not None and (not _finite(speed_mm_s) or float(speed_mm_s) <= 0):
            return refusal("speed_mm_s", "speed_mm_s must be a finite number > 0",
                "leave it out to use the arrival offer")
        if force_n is not None and (not _finite(force_n) or float(force_n) <= 0):
            return refusal("force_n", "force_n must be a finite number > 0",
                "state the peak the part may take in newtons")
        intent = {}
        if force_n is not None:
            intent["force_n"] = float(force_n)
        if clear is not None:
            intent["clear"] = clear
        if speed_mm_s is not None:
            intent["speed"] = float(speed_mm_s) / MM
        if "clear" in intent:
            word = B.word(intent)
            if word["refused"] is not None:
                return word["refused"]
        dark = self._dark_refusal_now()
        if dark is not None:
            return dark
        if B.f_cap(s.q, d) * (1 - B.e_tau()) <= B.f_thr(0.0, 0.0):
            return refusal("force", "the tip cannot push over its own force threshold here",
                           "reach_check along another direction, then strike there")
        if s.Fmag > B.f_thr(0.0, 0.0):
            return refusal("pressing", f"the tip already feels {s.Fmag:.2f} N", "ease(), then strike")
        travel = float(travel_mm) / MM
        thing = self._grasp_thing_at(B, s.tip + travel * d)
        force_item = B.f_allow(thing, s.q, d, intent, meant=True)
        F = float(force_item["value"])
        Z, z_src = self._grasp_stop_coeff(s.q, d, thing)
        offer, speed_item = B.v_in(thing, s.q, d, intent)
        offer = min(offer, B.v_free(s.q, d))
        speed = offer if speed_mm_s is None else min(float(speed_mm_s) / MM, offer)
        notes = []
        if speed_mm_s is not None and float(speed_mm_s) / MM > offer:
            notes.append(f"speed cut to {offer * MM:.2f} mm/s; state force_n to raise the part's arrival bound")
        if not _finite(offer) or offer <= 0:
            return refusal("speed", "the arrival offer is not finite and positive: " + speed_item["evidence"],
                           speed_item["lift"])
        threshold = B.f_thr(speed)
        servo = B.k_servo(s.q, d)
        lead_n = servo * B.lag(speed)
        persist_n = servo * (speed * B.t_det() + B.lag(speed))
        strike_n = persist_n if Z is None else Z * speed
        aim = min(F, max(F - strike_n, threshold, lead_n, R.REL * R.REL * F))
        bad = self._stroke_check(s, d, travel + self._lead_for(aim, s.q, d) + B.d_stop(speed, s.q, d))
        if bad is not None:
            return bad
        q_pre, t0 = s.q.copy(), s.t
        short = max(0.0, min(travel - B.rigid(s.q, d), B.d_stop(speed, s.q, d)))
        r = self._push(s, d, travel - short, aim, speed, 0.0, stop_on_force=True)
        t1 = r["s"].t
        self._grasp_coast = {"d": d, "peak": 0.0}
        try:
            e = self._ease_impl()
        finally:
            coast, self._grasp_coast = self._grasp_coast["peak"], None
        moved = float((e.tip - s.tip) @ d)
        self._push_history(self._retrace(r["free"], e, d) if "free" in r else r["done"])
        peak, stop = r["peak_n"], r.get("stopped", r.get("stop"))
        met = stop in ("force", "slipped", "cap") or peak > threshold
        coast_contact = stop == "travel" and coast > B.f_thr(speed, 0.0, readings=1)
        met = met or coast_contact
        ref = f"I{self._calls}"
        one = max(float(r["felt"].peak) if r.get("felt") is not None else 0.0, coast)
        self.world.note_sense("impulse", ref, summary={"peak_force_n": round(peak, 2), "gave": bool(r["gave"]),
                                                       "travel_mm": round(moved * MM, 2)})
        errors = [float((x.get("data") or {})["error_mm"]) for x in self.world.query()
                  if _finite((x.get("data") or {}).get("error_mm"))]
        error_mm = min(errors) if errors else B.rigid(s.q, d) * MM
        give = self.base_give or {}
        base_due = _finite(give.get("mm_per_n")) and peak * give["mm_per_n"] > error_mm
        hard = stop == "cap" or r["gave"] or coast_contact or base_due
        back_m = max(moved, 0.0) + self._lead_for(aim, e.q, d) + B.travel("press", thing, e.q, d).backoff_m
        base = self._base_check(hard, q_pre, back=-d * back_m)
        impact = None if Z is None else threshold + Z * speed
        evidence = speed_item.get("evidence", "the arrival law")
        out = {"said": (f"the tip struck at {speed * MM:.2f} mm/s, bound by {evidence}; inspect "
            f"the contact before striking again"),
               "travel_mm": moved * MM, "asked_travel_mm": float(travel_mm), "speed_mm_s": speed * MM,
               "stroke_s": t1 - t0, "peak_force_n": peak, "force_bound_n": F,
               "force_source": {"source": force_item["source"], "evidence": force_item.get("evidence")},
               "over_bound": bool(peak > F), "aimed_force_n": aim, "coast_contact": bool(coast_contact),
               "impact_n_est": impact, "impact_source": z_src, "peak_one_reading_n": one,
               "met_resistance": bool(met), "contact": bool(met), "gave": bool(r["gave"]),
               "slipped": bool(r["slipped"]), "stopped": stop, "base_moved": base, "ref": ref,
               "tip_mm": _mm(self._sense().tip), "force_after_n": e.Fmag,
               "periods": round((self.backend.read()["t"] - t0) / B.dt())}
        if Z is None:
            out["impact"] = "unresolved"
            out["strike_mj"] = strike_energy(B.m_eff(s.q, d), speed) * MM
            out["said"] += "; peak unresolved"
        if r.get("cap_why"):
            out["why"] = r["cap_why"]
        if notes:
            out["notes"] = notes
        return B.report(out, [force_item, speed_item])


def coast_watch(ex):
    """Tick hook: only while an impulse eases after its stroke (ex._grasp_coast set), the largest force estimate
    along the stroke in one control period, so a part the tip runs into while it coasts is reported. Every other
    period it returns at once; it never asks a stop."""

    def tick(sense):
        """Read the coast force on this control period; request no motion and no stop."""
        w = ex._grasp_coast
        if w is None:
            return None
        w["peak"] = max(w["peak"], float(sense.F @ w["d"]))
        return None

    return tick
