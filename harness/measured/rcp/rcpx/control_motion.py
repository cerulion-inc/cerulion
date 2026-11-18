# SPDX-License-Identifier: AGPL-3.0-only
"""Motion primitives of the Executor: streaming and the moves (planning and the path check are in
control_plan.py, the bounds every move runs under in control_speed.py, the jaws in control.py).

A plan is the robot's own fastest along its path, slowed by the session's bounds only where they have reason to; it
is streamed one joint target per control period with the contact stop armed. A move ends where it says
(control_move_pace.py): its command never leads the measured hand by more than the body's own lag, and it ends
"goal" only when the command stands at its end and the hand inside its still band of it; a hand that stays
further behind and does not close ends the move "short", with told_mm at the measured pose. A stated tolerance
(tolerance_mm) has the end waited for and corrected to it; a move that means a contact holds its last target
under the guard until the hand stands still. Every result says what the hand was told to reach, where it is and
how far behind it stands (told_mm, tip_mm, behind_mm). A line starts where the hand was told to be, so a relative
move is relative to told_mm and the command stays continuous. A contact, a cap or a tick hook's stop request ends
the move eased; a stop the bounds call false resumes the leg; a contact the caller expected ends it "arrived". A
move whose plan needs more control periods than are left is refused before it starts, retreat, park and home
included. Every move reports what it met below its stop threshold, the bound that was binding with its source,
and the control periods it used. Several legs run as one stream (_stream). No robot is named here.
"""

import collections
import math
from types import SimpleNamespace

import numpy as np

from rcpx import control_expect, control_move_pace, control_move_price, control_tail
from rcpx.context import refusal
from rcpx.control_base import Felt, Guard, _deg, _mm, _ratio, _val, primitive
from rcpx.control_plan import PlanMixin, _rx, _ry, _rz, profile, rot_from_vec, vec_from_rot  # noqa: F401
from rcpx.control_speed import MEETS, SpeedMixin, _attr, _hold_module, _unit

MM = 1000.0   # unit: mm per m
TURN_SEEN_RAD = math.radians(0.5)   # numeric: a turn under this is no turn asked or made (wording)
TURN_LEFT_DEG = 1.0                 # numeric: a result says what turn was left undone from this much (wording)


class MotionMixin(PlanMixin, SpeedMixin):
    # ------------------------------------------------------------------ streaming
    def _push_history(self, done):
        if len(done) >= 2:
            self._history.append([np.asarray(q).copy() for q in done])
            del self._history[:-self.cfg["history_max"]]

    def _stop_asked(self):
        if getattr(self, "_tick_stop", None):
            self._tick_stop = None

    def _run(self, path, g, goal=None, need_m=None, record=True, held=None, hook=None, leg=None, close=True):
        """Stream path under the guard g; returns (stop, last sense, done, streamed). goal = (p, R or None).
        stop is goal, short, contact, cap or the text of a tick hook's stop request; after a guard's stop the arm
        is eased. The command is paced by the hand and the run ends where it says (control_move_pace.Pace; close
        False: a leg a stream runs on from). With need_m (what the caller needs of the pose, m) the end is waited
        for instead (_settle). hook(k, s) runs after sample k. The hold's own record is told the end of every
        run (control_hold.after_move, leg None); a stream passes leg False and tells it every leg's end itself."""
        q_from = self._q_des.copy() if self._q_des is not None else self._sense().q.copy()
        done, stop, s, first, k = [], None, None, None, 0
        self._period_kind = "moving"
        recent = collections.deque([(self._last, q_from)] if self._last is not None else [], control_expect.ring(self))
        pace = control_move_pace.Pace(self, q_from, path, close and need_m is None)
        while k < len(path):
            wait = pace.holds(k, s, g)          # the command would lead the hand by more than the body's lag
            q = (path[k - 1] if k else q_from) if wait else path[k]
            s = self._period(q)
            recent.append((s, q))
            if wait:
                pace.n_held += 1
            else:
                done.append(q)
                k += 1
            stop = g.check(s)
            if first is None and g.first is not None:
                first = len(done) - 1
            elif g.first is None:
                first = None
            if stop:
                break
            if hook is not None and not wait:
                hook(k - 1, s)
        streamed = len(done)
        self._arrived = None
        if stop is None:           # a reading over the threshold is counted out before the move ends
            stop, more = control_tail.finish_count(self, path, g, recent)
            s = s if more is None else more
        if stop is None and need_m is not None and path:
            stop, s = self._settle(s, path[-1], g, need_m, goal, recent)
        elif stop is None:         # the command stands at its end: goal once the hand is there, short when not
            stop, s = pace.close(s, g, recent)
        self._period_kind = "moving"
        if stop is not None:
            g.recent = list(recent)                   # the readings around the stop: what it is measured from
            if stop == "cap" and not control_expect.own_cap(self, g, s):
                stop, g.cap_why = "contact", ""       # over the jaw motor's force alone: a contact, not a cap
            s = self._ease_impl()
            self._stop_asked()
            # the targets streamed after the first touching sample led the tip into the surface; the path to
            # retrace ends where the arm now rests
            done = done[:max(0, (first if first is not None else len(done)) - 1)] + [s.q.copy()]
        elif pace.verdict == "short":
            stop, done = "short", done[:-1] + [s.q.copy()]
        pace.record(s)
        if s is None:
            s = self._sense()
        hold = _hold_module()
        if hold is not None and leg is not False:         # leg False: the stream itself tells the hold (it knows
            hold.after_move(self, s, "limit" if stop == "short" else stop or "goal", leg=leg)   # whether it goes on)
        if record:
            self._push_history([q_from] + done)   # with where it started: a retreat ends there
        return stop or "goal", s, done, streamed

    def _settle(self, s, hold, g, need_m, goal=None, recent=None):
        """Hold the last target until the arm is quiet there for what the caller needs of the pose (need_m): the
        force under its threshold at rest, the motion left over one detection time under need_m, the tip that
        near its goal (the force is judged less what the hand holds, against the hold's own level: _quiet). It
        ends without that, by motion alone, when the arm makes no progress: over one settle time the distance left
        fell by less than the relative share of itself (a call always returns). While the hand brakes onto the
        standing target only the guard's cap judges; once it stands, the contact threshold of an arm at rest
        (control_tail.standing). Returns (stop or None, sense); _arrived says whether the wait ended quiet."""
        window = max(int(_ratio("QUANT")), int(math.ceil(_val(self.bounds.t_settle()) / self.dt)))
        rest = control_tail.rest_levels(self, self._held_for(None))
        p_hold = self.model.fk_tip(np.asarray(hold, dtype=float))[0] if goal is None else goal[0]
        left = [float(np.linalg.norm(p_hold - s.tip))]
        self._period_kind, stop, self._arrived = "waiting", None, False
        while True:
            if self._quiet(s, need_m) and left[-1] <= need_m:
                self._arrived = True
                break
            if len(left) > window and left[-1 - window] - left[-1] <= _ratio("REL") * left[-1 - window] \
                    and self._still(s, max(need_m, left[-1])):
                break                                     # no progress, by motion alone: where it rests is where it is
            s = self._period(hold)
            if recent is not None:
                recent.append((s, hold))              # a stop met in the lag tail is measured like any other
            control_tail.standing(self, g, s, rest)   # the cap alone while it brakes; the rest level once it stands
            stop = g.check(s)
            if stop:
                break
            left.append(float(np.linalg.norm(p_hold - s.tip)))
        self._period_kind = "moving"
        return stop, s

    def _execute(self, path, force_n, goal=None, settle=True, record=True, need=2):
        """Kept for callers that stream a path they timed themselves: (stop, guard, last sense, done). force_n: the
        threshold the caller states (None: the cap and a stop request only). settle: wait at the end."""
        g = Guard(self, force_n, need=need)
        need_m = self._band_at(path[-1]) if settle and path else None      # the still band at the end pose
        stop, s, done, _ = self._run(path, g, goal=goal, need_m=need_m, record=record, close=False)
        return stop, g, s, done

    def _drive(self, q0, way, kind, *, target=None, intent=None, held=None, away_from=None, goal=None,
               need_m=None, record=True, notes=None, hook=None, check=True):
        """Time the waypoints `way` by the bounds (B.plan: the robot's own full speed, slowed where the bounds have
        reason), check the timed path, and stream it under the guard the bounds derive; a stop the bounds call
        false resumes the leg from where it stopped; with intent expect "contact" a stop that is the contact meant
        (control_expect.arrived) ends the leg "arrived". The plan's control periods are counted against what is
        left before each piece runs (_budget). kind and target may be lists, one per waypoint. Returns a refusal
        or {"stop", "guard", "s", "done", "plan", "felt", "false_stops", "notes", "stop_info", "streamed"}."""
        notes = list(notes or [])
        readings = (intent or {}).get("_readings")
        share = (intent or {}).get("_share")
        q0 = np.asarray(q0, dtype=float)
        way = [np.asarray(q, dtype=float) for q in way]
        cut = None
        if check:      # a tip planned through the surface under the arm: cut there, run to contact (never refused)
            way, cut = control_expect.cut_at_support(self, q0, way)
        if cut:
            notes.append(cut)
            last = kind if isinstance(kind, str) else (list(kind) or ["touch"])[-1]
            last = last if last in MEETS else "touch"
            kind = last if isinstance(kind, str) else list(kind)[:max(0, len(way) - 1)] + [last]
            target = target[:len(way)] if isinstance(target, list) else target
            intent = dict(intent or {}, expect="contact")
        done_all, felt, false_stops, info, g, plan, streamed = [], None, 0, None, None, None, 0
        last_stop_tip, stop, s = None, "goal", None
        meant = bool(cut) or (intent or {}).get("expect") == "contact"
        while True:
            if way:
                self._travel_direction = _unit(self.model.fk_tip(way[-1])[0] - self.model.fk_tip(q0)[0])
            plan, more = self._bplan(q0, way, kind, target=target, intent=intent, held=held, away_from=away_from)
            notes += [n for n in more if n not in notes]
            bad = _attr(plan, "refused")
            if bad is not None:
                if false_stops == 0:
                    return bad if isinstance(bad, dict) and "refused" in bad else {"refused": bad}
                break
            path = self._slow(q0, [np.asarray(q, dtype=float) for q in (_attr(plan, "path") or [])], share)
            over = self._budget(len(path) + control_move_pace.allowance(self, q0, path), self._shorter(q0, path),
                                control_move_price.of_move(self, plan, intent, q0, way, kind, target, held, away_from))
            # (the count: the plan's periods and the hold pacing adds for the arm's own lag; a refusal names a
            # limit's price)
            if over is not None:
                if false_stops == 0:
                    return over
                break
            if check and false_stops == 0:
                bad = self._path_check([q0] + path, kind=kind if isinstance(kind, str) else kind[-1])
                if bad is not None:
                    return bad
            spec = _attr(plan, "guard")
            if spec is None:
                spec = self._guard_spec(kind if isinstance(kind, str) else kind[-1], held=held,
                                        away_from=away_from, readings=readings or 2)
            g = Guard(self, spec=spec)
            if readings is not None:
                g.need = min(g.need, int(readings))   # a caller that asks for fewer readings is stricter: honoured
            # a move that means a contact ends under its own guard: the last target is held until the hand is still
            need = self._band_at(path[-1]) if (need_m is None and meant and path) else need_m
            stop, s, done, n = self._run(path, g, goal=None if cut else goal, need_m=need, record=record, held=held,
                                         hook=None if hook is None else (lambda k, ss, base=streamed: hook(base + k, ss)))
            done_all += done
            streamed += n
            felt = g.felt if felt is None else felt.merge(g.felt)
            if stop == "goal":
                self._note_free_leg(plan, g, q0, path, held)
            if stop not in ("contact", "cap"):
                break
            expect = (control_expect.of_intent(self, intent, self._travel_direction)
                      if (intent or {}).get("expect") == "contact" else None)
            info = self._stop_note(g, kind, target, held, s, expect=expect, travel=self._travel_direction)
            if info["arrived"]:
                stop = "arrived"                        # the contact the caller meant: no surprise
            if stop != "contact" or not info["false"]:
                break
            idx = getattr(plan, "_segment_indices", None)
            if idx is not None and share is None and 0 < n <= len(idx):
                k = int(idx[n - 1])                     # the waypoint the arm was heading for when it stopped
                rest = way[k:]
                kind = kind if isinstance(kind, str) else list(kind)[k:]
                target = target[k:] if isinstance(target, list) else target
            else:
                rest = self._thin(s.q, path[n:])
                kind = kind if isinstance(kind, str) else kind[-1]
                target = target[-1] if isinstance(target, list) and target else target
            moved = last_stop_tip is None or float(np.linalg.norm(s.tip - last_stop_tip)) > _val(
                self.bounds.rigid(s.q, info["u"]))
            if not rest or not moved:
                break
            false_stops += 1          # nothing was there: the hidden level rose, the leg goes on
            last_stop_tip = s.tip.copy()
            q0, way = s.q.copy(), rest
        if cut and stop == "goal":
            notes.append(control_expect.NOTHING_MET)
        return {"stop": stop, "guard": g, "s": s if s is not None else self._sense(), "done": done_all, "plan": plan,
                "felt": felt if felt is not None else Felt(self), "false_stops": false_stops, "notes": notes,
                "stop_info": info, "streamed": streamed, "cut": cut}

    def _periods_now(self):
        return sum(self.periods.values())

    def _told(self):
        """Where the hand was told to be (after a stop: the measured pose of the stop) as (q, tip, R)."""
        q = self._q_des.copy()
        tip, R = self.model.fk_tip(q)
        return SimpleNamespace(q=q, tip=tip, R=R)

    def _cart_move(self, s0, dp, drot, speed, guarded, keep, force_n=None, limit=None, need=None, *,
                   kind="transit", target=None, intent=None, away_from=None, held=None):
        """Plan, check, time by the bounds, stream and report one Cartesian move. Returns a dict or a refusal.
        speed (m/s) None: the derived full speed; a value is the caller's lower speed. kind, target, intent,
        away_from and held go to the bounds; guarded, limit, need and a numeric force_n are retired (control_speed
        .py). The goal is s0's pose moved by dp and drot; the line to it starts where the hand was told to be
        (_told). A caller that states a tolerance (intent "tolerance", m) has the end waited for and corrected."""
        dark = self._dark_refusal_now()
        if dark is not None:
            return dark
        gate = self._selfknow_gate(kind)
        if gate is not None:
            return gate
        held = self._held_for(held)
        t0 = self._periods_now()
        intent, notes = self._intent(intent, speed=speed, guarded=guarded, limit=limit, need=need,
                                     force_n=force_n, kind=kind)
        p_goal = s0.tip + dp
        R_goal = s0.R if drot is None else rot_from_vec(drot) @ s0.R
        told = self._told()
        turn = None if drot is None else vec_from_rot(R_goal @ told.R.T)
        way, pstop, pwhy, hold = self._plan_hold(told.q, p_goal - told.tip, turn, None, keep, timed=False)
        if isinstance(way, dict):
            return way
        goal = (p_goal, R_goal if keep else None)
        need_m = intent.get("tolerance")
        r = self._drive(told.q, way, kind, target=target, intent=intent, held=held, away_from=away_from, goal=goal,
                        need_m=need_m, notes=notes)
        if "refused" in r:
            return r
        stop, g, s, done, felt = r["stop"], r["guard"], r["s"], r["done"], r["felt"]
        if stop == "goal" and pstop == "goal" and way and need_m is not None and getattr(self, "_arrived", False) \
                and not r["cut"]:
            for _ in range(self.cfg["corrections"]):   # the caller asked for the pose: bring the tip onto it
                err = p_goal - s.tip
                rerr = self._rot_err(R_goal, s.R, hold) if keep else None
                if np.linalg.norm(err) <= need_m and (rerr is None or np.linalg.norm(rerr) < TURN_SEEN_RAD):
                    break
                q_from = self._q_des.copy()
                q_to = self.model.clip(q_from + self._step(s.q, err, rerr, hold))
                fix = self._drive(q_from, [q_to], kind, target=target, intent=intent, held=held,
                                  away_from=away_from, goal=goal, need_m=need_m, record=False)
                if "refused" in fix:
                    break
                stop, g, s = fix["stop"], fix["guard"], fix["s"]
                felt.merge(fix["felt"])
                done.extend(fix["done"])
                if stop != "goal":
                    break
        if stop == "goal" and pstop != "goal":
            stop = pstop
        extra = self._call_extra(intent, r)
        keys = self._move_keys(r["plan"], self._periods_now() - t0, r["notes"], extra)
        why = pwhy if stop == "limit" else (control_expect.NOTHING_MET if r["cut"] and stop == "goal" else "")
        return {"stop": stop, "why": g.cap_why if stop == "cap" else why,
                "contact": g.contact, "peak_n": felt.peak, "s": s, "done": done, "felt": felt,
                "felt_keys": felt.report(g.force_n), "hold": hold, "speed": keys, "plan": r["plan"],
                "stop_info": r["stop_info"]}

    def _dark_refusal_now(self):
        """The refusal of a move toward the world while the newest light reading is dark (None otherwise). The gate
        follows the light, never one look (LooksMixin._note_light reads it at every look). cfg dark_gate "always":
        a session with no reading yet reads the light once from a frame first; "looks" (the default): only a look
        arms it; "off": never. park, home, retreat and ease do not ask."""
        mode = self.cfg["dark_gate"]
        if mode == "off":
            return None
        if self._dark_look is None and mode == "always" and getattr(self, "_light", None) is None:
            self._read_light("before a move toward the world")
        if self._dark_look is None:
            return None
        look_id, b = self._dark_look
        return refusal("dark", f"the newest light reading ({look_id}) is dark (brightness {b:.1f} of 255): a move "
                       "toward the world would run unseen", self._no_light_hint())

    def _record_contact(self, contact, source, label="contact", info=None):
        return control_expect.record_contact(self, contact, source, label, info)   # what is stored: see there

    # ------------------------------------------------------------------ move_tip
    def _move_intent(self, clear=None, expect=None, tolerance_mm=None, max_n=None):
        """The intent of an operator's move: only what the call states. tolerance_mm: what the caller needs of
        the end pose (the end is then waited for and corrected). max_n: a stated limit."""
        intent = {k: v for k, v in (("clear", clear), ("expect", expect)) if v}
        if max_n is not None:
            intent["max_n"] = float(max_n)
        if tolerance_mm is not None and float(tolerance_mm) > 0.0:
            intent["tolerance"] = float(tolerance_mm) / MM
        return intent

    @primitive
    def move_tip(self, dx=0.0, dy=0.0, dz=0.0, frame="base", guarded=True, speed_mm_s=None, keep_orientation=True,
                 clear=None, expect=None, tolerance_mm=None, max_n=None):
        s, eased = self._ease_if_pressing()
        told = self._told()                         # a relative move is relative to where the hand was told to be
        R = self._rot_of(frame, told)
        if R is None:
            return refusal("frame", f"unknown frame {frame!r}", "use base, tool or camera")
        asked = np.array([dx, dy, dz], dtype=float)
        speed = None if speed_mm_s is None else float(speed_mm_s) / MM
        r = self._cart_move(told, R @ asked / MM, None, speed, guarded, bool(keep_orientation),
                            kind="touch" if expect == "contact" else "transit",
                            intent=self._move_intent(clear, expect, tolerance_mm, max_n))
        if "refused" in r:
            return {**r, "stopped": "refused", "asked_mm": asked.tolist(), "achieved_mm": [0.0, 0.0, 0.0]}
        achieved = R.T @ (r["s"].tip - s.tip) * MM
        if r["stop"] == "contact":
            self._record_contact(r["contact"], "move_tip guard", info=r["stop_info"])
        out = {"frame": frame, "asked_mm": [round(float(x), 2) for x in asked],
               "achieved_mm": [round(float(x), 2) for x in achieved],
               "error_mm": round(float(np.linalg.norm(asked - achieved)), 2), "stopped": r["stop"],
               "contact": r["contact"], **r["felt_keys"],
               **self._finger_keys(s.R, r["s"].R, r["hold"] if keep_orientation else None), **r["speed"],
               **self._joint_keys(r["s"].q)}
        if r["why"]:
            out["why"] = r["why"]
        if eased:
            out["note"] = "a joint was still pressing; eased before the move"
        return out

    # ------------------------------------------------------------------ orient_tool
    @primitive
    def orient_tool(self, pitch_deg=None, roll_deg=None, yaw_deg=None, relative=True, clear=None, expect=None,
                    tolerance_mm=None):
        dark = self._dark_refusal_now()
        if dark is not None:
            return {**dark, "stopped": "refused"}
        s0, _ = self._ease_if_pressing()
        s = self._told()                            # a relative turn is relative to how the hand was told to stand
        p, r_, y = (math.radians(float(v or 0.0)) for v in (pitch_deg, roll_deg, yaw_deg))
        R_ref = s.R if relative else self.model.fk_tip(self.model.q_home)[1]
        R_goal = R_ref @ _rz(y) @ _ry(p) @ _rx(r_)
        drot = vec_from_rot(R_goal @ s.R.T)
        short = self.model.n < 6   # with the tip held this arm can make only some turns
        asked = {"pitch": float(pitch_deg or 0), "roll": float(roll_deg or 0), "yaw": float(yaw_deg or 0),
                 "relative": bool(relative)}
        if short:
            can, cannot = self._turn_words(s.q)
            doable = float(np.linalg.norm(self._can_turn(s.q) @ drot))
            if float(np.linalg.norm(drot)) > TURN_SEEN_RAD and doable < TURN_SEEN_RAD:
                # none of the asked turn can be made: nothing moves, and the result says what can
                left = math.degrees(float(np.linalg.norm(drot)))
                return {"asked_deg": asked, "turned_deg": 0.0, "error_deg": round(left, 2), "tip_moved_mm": 0.0,
                        "stopped": "limit", "contact": None, "tip_mm": _mm(s.tip), **Felt(self).report(),
                        **self._finger_keys(s.R, s.R, None), "could_not_turn_deg": round(left, 2),
                        "why": f"this arm has {self.model.n} joints: with the tip held it cannot "
                               f"{' or '.join(cannot) or 'make this turn'} here; nothing moved",
                        "hint": f"here it can {' and '.join(can) or 'turn about other axes'} (roll is about tool "
                                "x, pitch about tool y, yaw about tool z); ask one of those, or move_joints"}
        r = self._cart_move(s, np.zeros(3), drot, None, True, True, intent=self._move_intent(clear, expect, tolerance_mm),
                            kind="touch" if expect == "contact" else "transit")
        if "refused" in r:
            return {**r, "stopped": "refused"}
        if r["stop"] == "contact":
            self._record_contact(r["contact"], "orient_tool guard", info=r["stop_info"])
        e = r["s"]
        left = math.degrees(np.linalg.norm(vec_from_rot(R_goal @ e.R.T)))
        out = {"asked_deg": asked,
               "turned_deg": round(math.degrees(np.linalg.norm(vec_from_rot(e.R @ s0.R.T))), 2),
               "error_deg": round(left, 2), "tip_moved_mm": round(float(np.linalg.norm(e.tip - s0.tip)) * MM, 2),
               "stopped": r["stop"], "contact": r["contact"], **r["felt_keys"],
               **self._finger_keys(s0.R, e.R, None), **r["speed"], **self._joint_keys(e.q)}
        if r["why"]:
            out["why"] = r["why"]
        if short and left > TURN_LEFT_DEG and r["stop"] in ("goal", "limit"):
            can, cannot = self._turn_words(e.q)
            out["could_not_turn_deg"] = round(left, 2)
            out["hint"] = (f"this arm has {self.model.n} joints: with the tip held it made the part of the turn it "
                           f"can and left {left:.1f} deg undone; here it can {' and '.join(can) or 'turn little'}"
                           + (f", not {' or '.join(cannot)}" if cannot else "")
                           + " (roll is about tool x, pitch about tool y, yaw about tool z)")
        return out

    # ------------------------------------------------------------------ move_joints
    def _named_poses(self):
        m = self.model
        return {"home": m.q_home.copy(),
                "park": np.asarray(self.cfg["park_q"] if self.cfg["park_q"] is not None else m.q_home, dtype=float),
                "standard": self._standard_q()}

    def _standard_q(self):
        q = self.cfg["standard_q"]
        return np.asarray(self.model.q_home if q is None else q, dtype=float).copy()

    def _parse_q(self, q_deg, q_now):
        m = self.model
        if isinstance(q_deg, str):
            poses = self._named_poses()
            key = "standard" if q_deg in ("standard", "standard_look") else q_deg
            if key not in poses:
                return None, refusal("pose", f"no named pose {q_deg!r}", "named poses: " + ", ".join(poses))
            return poses[key], None
        if isinstance(q_deg, dict):
            q = q_now.copy()
            for name, v in q_deg.items():
                if name not in m.joint_names:
                    return None, refusal("joint", f"no joint named {name!r}", "joints: " + ", ".join(m.joint_names))
                q[m.joint_names.index(name)] = math.radians(float(v))
            return q, None
        arr = np.asarray(q_deg, dtype=float).reshape(-1)
        if arr.size != m.n:
            return None, refusal("q_deg", f"{arr.size} angles given, the arm has {m.n} joints",
                                 "give one angle per joint in robot://body joint_order, or a map by name")
        return np.radians(arr), None

    def _joint_move(self, q_target, speed=None, guarded=True, dark_ok=False, record_contact=True, limit=None,
                    need=None, *, kind="transit", target=None, intent=None, away_from=None, held=None,
                    clear_of=None):
        """Shared by move_joints, park, home, standard_look: (stop dict) or a refusal. speed None: the derived full
        speed; a value is the caller's lower share of it. dark_ok: the move runs after a dark look too (park,
        home); record_contact False keeps a guard stop out of the world (the base check). kind, target, intent,
        away_from and held go to the bounds. clear_of (m, or True for the derived clearance): the path must keep
        that far from every stored contact, else a refusal."""
        m = self.model
        if not dark_ok:
            dark = self._dark_refusal_now()
            if dark is not None:
                return dark
        gate = self._selfknow_gate(kind)
        if gate is not None:
            return gate
        for i in range(m.n):   # before anything moves: a refusal changes nothing
            if not (m.q_lo[i] <= q_target[i] <= m.q_hi[i]):
                return refusal("joint_limit", f"joint {m.joint_names[i]} target {math.degrees(q_target[i]):.1f} deg "
                               f"is outside its limits [{math.degrees(m.q_lo[i]):.1f}, "
                               f"{math.degrees(m.q_hi[i]):.1f}] deg", "pick a target inside the limits in robot://body")
        self._ease_if_pressing()
        q_from = self._told().q                    # the joint line starts where the arm was told to be
        held = self._held_for(held)
        t0 = self._periods_now()
        intent, notes = self._intent(intent, guarded=guarded, limit=limit, need=need, kind=kind)
        if speed is not None:
            intent["_share"] = min(1.0, max(float(speed), np.finfo(float).eps))
        way = self._plan_joint(q_from, q_target, timed=False)
        if clear_of is not None:
            bad = self._contact_clearance([q_from] + way, None if clear_of is True else clear_of,
                                          exempt=target if isinstance(target, str) else None)
            if bad is not None:
                return bad
        r = self._drive(q_from, way, kind, target=target, intent=intent, held=held, away_from=away_from,
                        need_m=intent.get("tolerance"), notes=notes)
        if "refused" in r:
            return r
        g, e, stop = r["guard"], r["s"], r["stop"]
        if stop == "contact" and record_contact:
            self._record_contact(g.contact, "joint move guard", info=r["stop_info"])
        extra = self._call_extra(intent, r)
        return {"stop": stop, "s": e, "contact": g.contact, "why": g.cap_why, "felt": r["felt"],
                "felt_keys": r["felt"].report(g.force_n), "peak_n": r["felt"].peak, "plan": r["plan"],
                "speed": self._move_keys(r["plan"], self._periods_now() - t0, r["notes"], extra),
                "stop_info": r["stop_info"]}

    @primitive
    def move_joints(self, q_deg, speed=None, guarded=True, clear=None, expect=None, tolerance_mm=None):
        m = self.model
        s = self._sense()
        q_target, bad = self._parse_q(q_deg, s.q)
        if bad is not None:
            return {**bad, "stopped": "refused"}
        share = None if speed is None else min(1.0, max(0.0, float(speed)))   # a lower share of the full speed
        # the named rest poses are the moves park and home make: they run after a dark look too
        rest = isinstance(q_deg, str) and q_deg in ("home", "park")
        kind = "park" if rest else ("touch" if expect == "contact" else "transit")
        r = self._joint_move(q_target, share, dark_ok=rest, kind=kind,
                             intent=self._move_intent(clear, expect, tolerance_mm))
        if "refused" in r:
            return {**r, "stopped": "refused"}
        e = r["s"]
        out = {"asked": {n: _deg(v) for n, v in zip(m.joint_names, q_target)},
               "achieved": {n: _deg(v) for n, v in zip(m.joint_names, e.q)},
               "error_deg": round(math.degrees(float(np.max(np.abs(q_target - e.q)))), 3),
               "stopped": r["stop"], "contact": r["contact"], **r["felt_keys"], **r["speed"],
               "limit_margin_deg": self._joint_keys(e.q)["limit_margin_deg"]}
        if r["why"]:
            out["why"] = r["why"]
        return out

    # ------------------------------------------------------------------ ease, retreat, park, home
    @primitive
    def ease(self):
        s = self._ease_impl()
        return {"eased": True, "force_n": round(s.Fmag, 2), "tip_mm": _mm(s.tip)}

    def _links_clear(self, s, v=0.0):
        """True when every link is its stopping distance at speed v plus the point's own error clear of every
        stored contact (A4.8). s: a reading, or a joint pose."""
        known = self._known_points()
        if not len(known):
            return True
        B = self.bounds
        q = np.asarray(getattr(s, "q", s), dtype=float)
        need = _val(B.d_stop(v, q)) + _val(B.clearance(v))
        return self._nearest_known(q, known)[0] >= need

    def _retreat_impl(self, steps=None, limit=None, need=None, until=None, intent=None):
        return control_expect.retreat(self, steps, limit, need, until, intent)   # the way the arm came

    def _last_contact_point(self):
        entries = [e for e in self.world.query(kind="contact", include_stale=True) if e.get("point_mm") is not None]
        return None if not entries else np.asarray(entries[-1]["point_mm"], dtype=float) / MM

    @primitive
    def retreat(self, steps=None, clear=None, tolerance_mm=None):
        return self._retreat_impl(steps, intent=self._move_intent(clear, None, tolerance_mm))

    def _go_pose(self, q_target, what, intent=None):
        """park, home, the standard pose: a move of kind park (it retraces the way it came first when the direct
        way is refused). Its end is waited for and corrected only when the caller stated a tolerance."""
        s0 = self._sense()
        B = self.bounds
        if float(np.max(np.abs(q_target - s0.q) - np.asarray(B.joint_quantum(), dtype=float))) <= 0.0 \
                and s0.Fmag <= _val(B.f_thr(0.0, 0.0)):
            self._history.clear()
            return {"pose": what, "stopped": "goal", "already_there": True, "contact": None, **self._told_keys(),
                    "error_deg": round(math.degrees(float(np.max(np.abs(q_target - s0.q)))), 3), "eased": True,
                    "note": f"the arm already holds the {what} pose: nothing moved and no time passed"}
        intent = dict(intent or {})
        r = self._joint_move(q_target, None, dark_ok=True, kind="park", intent=intent)
        retreated = None
        if "refused" in r and self._history and r["refused"].get("check", "").startswith("path"):
            retreated = self._retreat_impl(intent={k: v for k, v in intent.items() if k == "clear"})
            if "refused" not in retreated and retreated["stopped"] == "goal":
                r = self._joint_move(q_target, None, dark_ok=True, kind="park", intent=intent)
        if "refused" in r:
            return {**r, "stopped": "refused"}
        e = r["s"]
        if r["stop"] == "goal":
            self._history.clear()
        out = {"pose": what, "stopped": r["stop"], "contact": r["contact"],
               "error_deg": round(math.degrees(float(np.max(np.abs(q_target - e.q)))), 3),
               "eased": r["stop"] != "goal" or "tolerance" in intent, **r["felt_keys"], **r["speed"]}
        if retreated is not None:
            out["retreated_first"] = retreated.get("retreated", 0)
        return out

    @primitive
    def park(self, clear=None, tolerance_mm=None):
        return self._go_pose(self._named_poses()["park"], "park", self._move_intent(clear, None, tolerance_mm))

    @primitive
    def home(self, clear=None, tolerance_mm=None):
        return self._go_pose(self.model.q_home.copy(), "home", self._move_intent(clear, None, tolerance_mm))

    @primitive
    def read_state(self):
        s = self._sense()
        return {"q_deg": {n: _deg(v) for n, v in zip(self.model.joint_names, s.q)}, "gripper": s.gripper,
                "t_s": round(s.t, 3), **self._told_keys()}
