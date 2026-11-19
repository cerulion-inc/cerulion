# SPDX-License-Identifier: AGPL-3.0-only
"""Located points and hand poses. All clearances come from the session Bounds."""

import functools
import math
import sys

import numpy as np

from rcpx.r2 import articulate_bounds as A
from rcpx.model import ROT_WEIGHT_M, _rotvec
from rcpx.r2.articulate_follow import guard_step, level_across, rot, unit

C = sys.modules[__name__]

UP = np.array([0.0, 0.0, 1.0])
TILTS = (45, 30, 60, 15, 0, -15, -30, -45, -60)  # A search grid of hand tilts.
TILTS_UPRIGHT = (0, 15, -15, 30, -30, 45, -45, 60, -60)


def refusal(check, why, hint):
    return {"said": "The call was refused: " + why.rstrip('.') + '.',
            "refused": {"check": check, "why": why, "hint": hint}}


class StepsSpent(Exception):
    """A nested call reached the backend's budget."""


def end_word_call(fn):
    @functools.wraps(fn)
    def wrapped(ex, *args, **kwargs):
        try:
            return fn(ex, *args, **kwargs)
        finally:
            end = getattr(A.of(ex).space, "end_call", None)
            if end is not None:
                end()
    return wrapped


def sense(ex):
    return ex._sense()


def journal(ex, kind, **fields):
    return ex._journal(kind, **fields)


def stop(ex):
    return ex._stop_asked()


def floor(ex, q):
    return ex.floor_clearance(q)


def jaw(ex, opening, hold):
    start = sense(ex).t
    result = ex.gripper(opening, hold=hold)
    if "periods" not in result:
        result["periods"] = round((sense(ex).t - start) / A.of(ex).dt())
    return result


def ease(ex):
    B = A.of(ex)
    count = math.ceil(B.t_settle() / B.dt())
    return ex._ease_impl(periods=count, max_periods=math.ceil(B.limit_s(B.t_settle()) / B.dt()))


def run(ex, fn, *args):
    original = ex._period
    def checked(q):
        if ex._ar_spent():
            raise StepsSpent()
        return original(q)
    ex._period = checked
    try:
        return fn(*args)
    finally:
        ex._period = original


def period(ex, q, *, kind, target, intent, away_from=None, held=None):
    # Each measured follow update is one period, not a separately settled motion primitive.
    B = A.of(ex)
    current = getattr(ex, "_q_des", sense(ex).q)
    step = A.joint_step(B)
    q = current + np.clip(np.asarray(q) - current, -step, step)
    result = ex._period(q)
    record(ex, {"periods": 1, "moving": 1})
    return result


def begin(ex, phase):
    A.of(ex).begin(phase)
    rec = dict(phase=phase, did=phase, periods=0, moving=0, waiting=0, jaw=0,
               seconds=0, free_share=None, bounds=None)
    ex._ar_phases.append(rec)
    ex._ar_phase_start = sense(ex).t
    return rec


def record(ex, result, jaw_motion=False):
    if not getattr(ex, "_ar_phases", None):
        return result
    phase = ex._ar_phases[-1]
    count = result.get("periods", 0)
    if isinstance(count, list):
        count = sum(count)
    if jaw_motion:
        phase["jaw"] += count
    else:
        waiting = result.get("waiting", 0)
        phase["waiting"] += waiting
        phase["moving"] += result.get("moving", max(0, count - waiting))
    phase["periods"] = phase["moving"] + phase["waiting"] + phase["jaw"]
    phase["seconds"] = sense(ex).t - ex._ar_phase_start
    bound = result.get("bounds")
    if bound:
        phase["bounds"] = bound.get("binding") if isinstance(bound, dict) else bound[0]
        if isinstance(bound, dict):
            phase["free_share"] = bound.get("free_share", phase["free_share"])
    return result


class Watch:
    """Contact and cap decisions for a period loop, from its live GuardSpec."""
    def __init__(self, ex):
        self.ex, self.contact, self.cap = ex, 0, 0
        self.last = None
        rest = sense(ex)
        self.rest_force = rest.F.copy()
        self.rest_residual = rest.ext - ex.model.jac(rest.q)[:3].T @ rest.F
        self.held = {"squeeze_n": ex._ar_hold().get("squeeze_n_est")}

    def check(self, s, speed, axis, torque_axis=False):
        ex, B = self.ex, A.of(self.ex)
        spec = B.guard("follow", speed, held=self.held)
        compensated = s.F - self.rest_force
        force = compensated - (compensated @ axis) * axis if not torque_axis else compensated
        magnitude = float(np.linalg.norm(force))
        residual = s.ext - ex.model.jac(s.q)[:3].T @ s.F - self.rest_residual
        if torque_axis:
            weights = ex.model.jac(s.q)[3:].T @ axis
            den = float(weights @ weights)
            if den > 0:
                residual = residual - (residual @ weights) / den * weights
        self.last = dict(side_force_n=magnitude, force_threshold_n=spec.force_n,
                         one_reading_n=spec.one_reading_n, held_rest_force_n=self.rest_force.tolist(), residual_nm=np.asarray(residual).tolist(),
                         torque_threshold_nm=np.asarray(spec.tau).tolist())
        room = np.asarray(B.tau_max()) - np.abs(ex.model.gravity_torque(s.q))
        self.contact, self.cap, decision = guard_step(self.contact, self.cap, magnitude, residual, s.ext, room,
                                                     spec.force_n, spec.one_reading_n, spec.tau, spec.cap_n, spec.readings,
                                                     total_force=float(np.linalg.norm(s.F)))
        if decision:
            return decision
        return getattr(ex, "_tick_stop", None)


def direction(value):
    names = {"forward": [1, 0, 0], "back": [-1, 0, 0], "left": [0, 1, 0],
             "right": [0, -1, 0], "up": [0, 0, 1], "down": [0, 0, -1]}
    if isinstance(value, str):
        value = names.get(value)
    try:
        v = np.asarray(value, dtype=float)
        if v.shape == (3,) and np.all(np.isfinite(v)) and np.linalg.norm(v) > 1e-9:
            return unit(v)
    except (TypeError, ValueError):
        pass
    return None


class GraspMixin:
    def _ar_stage(self, tool, stage, result):
        # Core motion results contain opaque Sense and Felt instances. Keep
        # their explicit evidence and avoid process addresses in the journal.
        clean = {k: v for k, v in result.items() if k not in ("s", "felt", "done")}
        C.journal(self, "articulate", tool=tool, stage=stage, result=clean)

    def _ar_spent(self):
        return self.backend.read().get("steps_left") == 0

    def _ar_target(self, target):
        if isinstance(target, str):
            found = self.world.get(target)
            entries = [found] if found is not None else self.world.query(label=target)
            entries = [e for e in entries if e.get("point_mm") is not None]
            fresh = [e for e in entries if not e.get("stale")]
            if not fresh:
                if entries:
                    return None, refusal("stale_target", "it moved since it was located", "look and locate it again")
                labels = list(dict.fromkeys(e["label"] for e in self.world.query() if e.get("point_mm") is not None))
                return None, refusal("unknown_target", f"no located point for {target!r}",
                                     "known labels: " + ", ".join(labels))
            e = fresh[-1]
            return {"p": np.asarray(e["point_mm"], float) / 1000, "entry": e,
                    "target": {"point_mm": list(e["point_mm"]), "from": e["id"]}}, None
        try:
            p = np.asarray(target, dtype=float)
            if p.shape == (3,) and np.all(np.isfinite(p)) and np.asarray(target).dtype.kind in "iuf":
                return {"p": p / 1000, "entry": None,
                        "target": {"point_mm": p.tolist(), "from": "point"}}, None
        except (TypeError, ValueError):
            pass
        return None, refusal("target", "target must be a world id, label or [x, y, z] mm", "locate a point first")

    def _ar_approach(self, p, given, pull):
        if given is not None:
            v = direction(given)
            if v is None:
                return None, None, refusal("args", "give a direction name or nonzero [x, y, z]", "give the way again")
            return -v if pull else v, "way" if pull else "approach", None
        for e in reversed(self.world.query(kind="contact", include_stale=False)):
            if e.get("point_mm") is None or np.linalg.norm(np.asarray(e["point_mm"]) / 1000 - p) > A.same_spot_m(A.of(self), A.of(self).thing(p), A.of(self).thing(e["id"]), self._ar_jaws):
                continue
            normal = direction(e.get("data", {}).get("normal_est"))
            if normal is not None:
                return -normal, "touched surface", None
        v = unit([p[0], p[1], 0])
        if not np.any(v):
            return None, None, refusal("args", "the point has no horizontal bearing from the base",
                                       "give way or approach")
        return v, "facing the base", None

    def _ar_rot(self, t, n):
        a = unit([0, self._ar_jaws.axis_T[1], self._ar_jaws.axis_T[2]])
        tool = np.column_stack([[1, 0, 0], a, np.cross([1, 0, 0], a)])
        return np.column_stack([t, n, np.cross(t, n)]) @ tool.T

    def _ar_solve(self, p, R, seeds):
        best_floor = -float("inf")
        for seed in seeds:
            q, ok, _ = self.model.ik(seed, p, R_target=R, restarts=4, seed=0)
            if not ok or not self.model.within_limits(q):
                continue
            floor = C.floor(self, q)
            best_floor = max(best_floor, floor)
            floor_margin, self_margin = A.margins(A.of(self), self._ar_support()[1])
            if floor >= floor_margin and not self.model.self_collisions(q, self_margin):
                return q, best_floor
        return None, best_floor

    def _ar_plan(self, data, a0, source, side, f_open, pull, angle=None, level=False, skip=()):
        s = C.sense(self)
        J = self._ar_jaws
        B = A.of(self)
        part = data["part"]
        depth = A.depth_m(B, part, J.finger_len_m)
        gap = J.gap_centre_T(f_open).copy()
        gap[0] = 0
        best_floor, rooms, turn_bads = -float("inf"), [], []
        tilts = TILTS if abs(float(side @ UP)) <= A.ratios().REL else TILTS_UPRIGHT
        choices = [(tilt, side) for tilt in tilts if tilt not in skip] if pull else (
            [(0, level_across(a0, angle)), (0, side)] if level else [(0, side)])
        for tilt, bearing in choices:
            side = bearing
            axis = side.copy()
            if pull and abs(side @ UP) < 1 - A.ratios().REL and np.cross(axis, a0) @ -UP < 0:
                axis = -axis
            t = unit(rot(axis, np.radians(tilt)) @ a0) if pull else a0
            n = unit(np.cross(side, t)) if pull else unit(side - (side @ t) * t)
            if not np.any(n):
                return None, refusal("args", "the bearing is parallel to the tool axis", "give along or across again")
            feasible = []
            for flip in (1, -1):
                R = self._ar_rot(t, flip * n)
                p_tip = data["p"] + depth * t - R @ gap
                p_pre = p_tip - B.standoff(B.v_free(s.q, t), part, s.q, t) * t
                q_pre, floor = self._ar_solve(p_pre, R, [s.q, self.model.q_home])
                best_floor = max(best_floor, floor)
                if q_pre is None:
                    continue
                q_mid, floor = self._ar_solve(np.mean([p_pre, p_tip], axis=0), R, [q_pre])
                best_floor = max(best_floor, floor)
                if q_mid is None:
                    continue
                q, floor = self._ar_solve(p_tip, R, [q_pre, s.q, self.model.q_home])
                best_floor = max(best_floor, floor)
                if q is None:
                    continue
                plan = {**data, "q": q, "q_pre": q_pre, "R": R, "p_tip": p_tip,
                        "p_pre": p_pre, "f_open": f_open,
                        "approach": {"from": source, "a0_b": a0.tolist(), "tilt_deg": tilt,
                                     "t_b": t.tolist(), "jaws_b": (flip * n).tolist()}}
                if not pull:
                    turn, bad = self._ar_turn_plan(q, R, angle, part)
                    if bad:
                        if "room_rad" in bad:
                            rooms.append(bad["room_rad"])
                        else:
                            turn_bads.append(bad)
                        continue
                    plan["turn"] = turn
                feasible.append((float(np.linalg.norm(q - s.q)), plan))
            if feasible:
                plan = min(feasible, key=lambda pair: pair[0])[1] if pull else feasible[0][1]
                plan["replan"] = (data, a0, source, bearing, f_open, pull, angle, level)
                points, text = A.via_points(B, s.tip, plan["p_pre"], J.max_m, part, s.q, -UP,
                                            self._ar_support()[0])
                legs = []
                for point in points[:-1]:
                    q_via, _ = self._ar_solve(point, plan["R"], [plan["q_pre"]])
                    if q_via is None:
                        return None, refusal("not_reachable", "the way over to the grasp has no feasible arm pose",
                                             "move the base, give way or along, or a point nearer the robot")
                    legs.append(dict(kind="transit", to=q_via, intent=data["intent"]))
                else:
                    pose = np.eye(4)
                    pose[:3, :3], pose[:3, 3] = plan["R"], plan["p_pre"]
                    legs.append(dict(kind="transit", to=pose, line=True, intent=data["intent"]))
                if not legs:
                    legs.append(dict(kind="transit", to=plan["q_pre"], intent=data["intent"]))
                plan["legs"] = legs
                plan["approach"].update(route=text, via_mm=float((points[0][2]-plan["p_pre"][2])*1000))
                return plan, None
        if not pull:
            if turn_bads:
                return None, next((b for b in turn_bads if b["refused"]["check"] == "not_reachable"), turn_bads[0])
            if rooms:
                return None, self._ar_range_refusal(max(rooms))
            if np.isfinite(best_floor):
                return None, self._ar_turn_refusal(angle, best_floor, 0, 0)
        floor_text = "none" if not np.isfinite(best_floor) else f"{1000 * best_floor:.1f} mm"
        tried = (TILTS if abs(float(side @ UP)) <= A.ratios().REL else TILTS_UPRIGHT) if pull else (0,)
        return None, refusal("not_reachable", f"no feasible pose; tilts tried {tried} deg; "
                             f"best floor clearance {floor_text}",
                             "move the base, give way or along, or a nearer point")

    def _ar_support(self):
        return A.support(A.of(self), self.cfg.get("floor_z_m"))

    def _ar_line(self, dp, *, kind="transit", target=None, intent=None, away_from=None):
        if self._ar_spent():
            return {"stop": "steps_spent", "periods": 0}
        args = dict(zip(("dx", "dy", "dz"), (np.asarray(dp) * 1000).tolist()))
        bad = self.gates("move_tip", {**args, "frame": "base", **(intent or {})})
        if bad:
            return {"stop": "gate", "periods": 0, **bad}
        try:
            return C.record(self, A.line(self, dp, kind=kind, target=target, intent=intent, away_from=away_from))
        except StepsSpent:
            return {"stop": "steps_spent", "periods": 0}

    def _ar_stream(self, legs):
        for leg in legs:
            goal = np.asarray(leg["to"])
            tool = "move_joints" if goal.ndim == 1 else "move_tip"
            args = ({"q_deg": np.degrees(goal).tolist()} if goal.ndim == 1 else
                    dict(zip(("dx", "dy", "dz"), ((goal[:3, 3]-C.sense(self).tip)*1000).tolist()), frame="base"))
            bad = self.gates(tool, {**args, **leg.get("intent", {})})
            if bad:
                return {"stopped": "gate", "periods": 0, **bad}
        try:
            return C.record(self, A.stream(self, legs))
        except StepsSpent:
            return {"stopped": "steps_spent", "periods": 0}

    def _ar_jaw(self, opening, hold=True):
        if self._ar_spent():
            return {"stopped": "steps_spent", "periods": 0}
        bad = self.gates("gripper", {"width_frac": opening, "hold": hold})
        if bad:
            return {"stopped": "gate", "periods": 0, **bad}
        try:
            return C.record(self, C.jaw(self, opening, hold), jaw_motion=True)
        except StepsSpent:
            return {"stopped": "steps_spent", "periods": 0}

    def _ar_hold(self, close=None):
        J = self._ar_jaws
        q, target = J.reading(self.backend.read().get("gripper"), self._gripper_cmd)
        w = 0.0 if q is None else float(J.width_m(q) * 1000)
        pressing = bool(J.pressing(q, target))
        return {"width_mm": w, "pressing": pressing,
                "squeeze_n_est": J.squeeze_n(q, target), "close_blocked": "blocked" in (close or {}),
                "held": w / 1000 >= A.min_gap_m(A.of(self), J) and pressing}

    def _ar_ease(self):
        if not self._ar_spent():
            try:
                start = C.sense(self).t
                C.ease(self)
                count = round((C.sense(self).t-start)/A.of(self).dt())
                C.record(self, {"periods": count, "waiting": count})
            except StepsSpent:
                pass

    def _ar_release(self, plan):
        if self._ar_spent():
            return False, {"stopped": "steps_spent", "periods": 0}
        B, part = A.of(self), plan["part"]
        before = self._ar_hold()["width_mm"] / 1000
        opened = self._ar_jaw(plan["f_open"])
        rounds = []
        while "blocked_opening" in opened or opened.get("stopped") == "room":
            after = self._ar_hold()["width_mm"] / 1000
            if after - before <= A.ratios().QUANT * A.jaw_step_m(B, self._ar_jaws):
                break
            before = after
            if self._ar_spent():
                return False, {"stopped": "steps_spent", "periods": 0}
            C.stop(self)
            s = C.sense(self)
            distance = B.travel("lift", part, s.q, -s.R[:, 0]).backoff_m
            line = self._ar_line(-s.R[:, 0] * distance, kind="lift", away_from=plan["ref"], intent=plan["intent"])
            rounds.append({"backoff": {k: v for k, v in line.items() if k not in ("s", "felt")}})
            if "refused" in line or line.get("stop") not in ("goal", "contact"):
                return False, {**line, "said": "The jaws are still on the part.",
                               "hint": "gripper(width_frac='open')", "rounds": rounds}
            opened = self._ar_jaw(plan["f_open"])
            rounds[-1]["open"] = opened
        if "refused" in opened or opened.get("stopped") != "goal" or "blocked_opening" in opened:
            return False, {"said": "The jaws are still on the part.", **opened,
                           "hint": "gripper(width_frac='open')", "rounds": rounds}
        s = C.sense(self)
        distance = B.standoff(B.v_free(s.q, -s.R[:, 0]), part, s.q, -s.R[:, 0])
        line = self._ar_line(-s.R[:, 0] * distance, kind="retreat", away_from=plan["ref"], intent=plan["intent"])
        return "refused" not in line and line.get("stop") == "goal", {**line, "rounds": rounds}

    def _ar_joints(self, q, *, kind="transit", target=None, intent=None, away_from=None):
        if self._ar_spent():
            return {"stopped": "steps_spent", "periods": 0}
        bad = self.gates("move_joints", {"q_deg": np.degrees(q).tolist(), **(intent or {})})
        if bad:
            return {"stopped": "gate", "periods": 0, **bad}
        try:
            return C.record(self, A.joints(self, q, kind=kind, target=target, intent=intent, away_from=away_from))
        except StepsSpent:
            return {"stopped": "steps_spent", "periods": 0}

    def _ar_grasp(self, plan, grip="close"):
        attempts, skipped = [], []
        while True:
            outcome = self._ar_grasp_once(plan, grip)
            depth = float((C.sense(self).tip - plan["p"]) @ plan["R"][:, 0] * 1000)
            attempts.append({"tilt_deg": plan["approach"]["tilt_deg"],
                             "stop": outcome.get("stop", outcome.get("stopped", "held" if outcome.get("held") else "refused")),
                             "at_mm": float(depth)})
            plan["approach"]["attempts"] = attempts
            if not outcome.pop("retry", False):
                return outcome
            C.stop(self)
            self._ar_ease()
            B, s = A.of(self), C.sense(self)
            t = -plan["R"][:, 0]
            retreat = self._ar_line(t * B.standoff(B.v_free(s.q, t), plan["part"], s.q, t),
                                    kind="lift", away_from=plan["ref"], intent=plan["intent"])
            if "refused" in retreat or retreat.get("stop") != "goal":
                return retreat
            skipped.append(plan["approach"]["tilt_deg"])
            if not plan["replan"][5] or len(skipped) >= len(TILTS):
                break
            next_plan, bad = self._ar_plan(*plan["replan"], skip=skipped)
            if bad:
                break
            plan.clear()
            plan.update(next_plan)
        plan["approach"]["attempts"] = attempts
        return {"said": "The open jaws met something before the part.", "held": False,
                "approach": plan["approach"], **refusal("not_held",
                f"the open jaws met something {max(0, -depth):.1f} mm before the handle point",
                "give width_mm, along or the point again")}

    def _ar_grasp_once(self, plan, grip):
        opened = self._ar_jaw(plan["f_open"])
        if "refused" in opened or opened.get("stopped") != "goal" or "blocked_opening" in opened:
            return opened
        self._gripper_cmd = plan["f_open"]
        move = self._ar_stream(plan["legs"])
        if "refused" in move or move.get("stopped") != "goal":
            return move
        line = self._ar_line(plan["p_tip"] - C.sense(self).tip, kind="approach",
                             target=plan["ref"], intent=plan["intent"])
        if "refused" in line or line.get("stop") not in ("goal", "contact"):
            return line
        t = plan["R"][:, 0]
        short = float((plan["p_tip"] - C.sense(self).tip) @ t)
        if line.get("stop") == "contact" and short > A.band(A.of(self), plan["part"]) + A.of(self).rigid(C.sense(self).q, t):
            return {**line, "retry": True}
        close = self._ar_jaw("close", hold=grip == "close")
        if "refused" in close or close.get("stopped") in ("steps_spent", "arm_guard"):
            return close
        evidence = self._ar_hold(close)
        if not evidence["held"]:
            released, rel = self._ar_release(plan)
            why = (f"the jaws met each other at {evidence['width_mm']:.1f} mm" if evidence["width_mm"] / 1000 < A.min_gap_m(A.of(self), self._ar_jaws)
                   else "the jaws do not press on anything")
            return {"said": "The jaws did not hold the part.", "held": False, "hold": evidence,
                    "released": released, "release": rel,
                    **refusal("not_held", why, "look, give width_mm or the point again")}
        return {"held": True, "hold": evidence}

    def _ar_dls(self, s, p, R):
        q = s.q.copy()
        clipped = False
        for _ in range(2):
            p_now, R_now = self.model.fk_tip(q)
            J = self.model.jac(q)
            dq = self.model._dls(np.vstack([J[:3], ROT_WEIGHT_M * J[3:]]),
                                 np.concatenate([p - p_now, ROT_WEIGHT_M * _rotvec(R @ R_now.T)]), q)
            step = A.joint_step(A.of(self))
            dq = np.clip(dq, -step, step)
            raw = q + dq
            q = self.model.clip(raw)
            clipped = clipped or bool(np.any(np.abs(q - raw) > 1e-9))
        return q, clipped
