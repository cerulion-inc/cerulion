# SPDX-License-Identifier: AGPL-3.0-only
"""Executor (SPEC section 3, rcpx/control.py): every operator primitive on a RobotModel and a Backend.

Split by concern: motion and planning in control_motion.py (move_joints, move_tip, orient_tool, retreat, park, home,
ease, gripper), contact work in control_contact.py (touch, press, reach_check, measure_base_give), looks in
control_looks.py (look, standard_look, relocalize, mark, triangulate, picture, snapshot), sensing in
control_sense.py (the external joint torque from two readings and where the force acts). This file holds the shared
machinery: configuration, the gravity feed-forward command, easing, frames; control_base.py holds the defaults,
the guard and the primitive wrapper that ticks the World, journals and never raises.

Rules every primitive follows (SPEC section 3 control.py, I6):
- every bound a move runs under (speed, force, torque, travel, time) is derived by the session's bounds
  (rcpx/bounds.py, ex.bounds) from the robot's own ratings and what it measured; none is typed here;
- a path check runs before motion; a refusal is a result and changes nothing;
- contact = the external tip force estimate over the derived threshold on its readings in a row, or a joint's
  external torque over its own; the contact stop is armed on every move;
- a tick hook's stop request ends a move like a contact, with stopped equal to its text;
- after a dark look, motion toward the world is refused (check "dark") until a look that is not dark;
- a force or torque cap ends the move with stopped "cap", never the session;
- after a contact or cap stop the command is eased to the present pose so nothing keeps pushing;
- what the hand holds is one record (ex._held, written only by rcpx/control_hold.py); the jaws close and open
  through that file's one close and one release, which no arm guard at the contact threshold judges.
SI inside; results speak mm, degrees, newtons, seconds. No robot is named here.
"""

import math

import mujoco
import numpy as np

from bench.robots import home_qpos, load_robot_mjcf
from rcpx import bounds as _bounds
from rcpx import control_hold, control_move_passed
from rcpx.context import World, refusal
from rcpx.control_base import COMPAT, DEFAULTS, FACTS, Cfg, Guard, Sense, _mm, _ratio, _val, primitive  # noqa: F401
from rcpx.control_contact import ContactMixin
from rcpx.control_looks import LooksMixin
from rcpx.control_motion import MotionMixin
from rcpx.control_sense import FRICTION_V_RAD_S, SenseMixin  # noqa: F401


MM = 1000.0   # unit: mm per m
TINY = np.finfo(float).tiny
JAW_ROOM_STEPS = 13   # numeric: openings the jaw room is read at
NEWLY = 1e-4          # numeric: m; "nearer than it started" means nearer by more than this


class Executor(MotionMixin, ContactMixin, LooksMixin, SenseMixin):
    """SPEC 3: Executor(model, backend, journal, world, cfg). Each primitive returns a dict and never raises."""

    def __init__(self, model, backend, journal=None, world=None, cfg=None):
        self.model = model
        self.backend = backend
        self.journal = journal
        self.world = world if world is not None else World(journal)
        self.cfg = Cfg(None, self)
        unknown = set(cfg or {}) - set(FACTS) - set(COMPAT)
        if unknown:
            raise ValueError(f"unknown executor cfg keys {sorted(unknown)}")
        for k, v in (cfg or {}).items():
            self.cfg[k] = v          # a key round four removed is ignored with a journal note (control_base.Cfg)
        dt = getattr(backend, "dt_s", None) or getattr(backend, "dt", None)
        if not dt or not float(dt) > 0.0:
            raise ValueError("the backend states no control period (Backend.dt_s)")
        self.dt = float(dt)
        spec = model.spec
        self.kp = np.asarray(spec["kp"], dtype=float)
        self.torque_limit = np.asarray(spec["torque_limit"], dtype=float)
        m = model.mj
        self._qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in model.joint_names])
        self._dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in model.joint_names])
        self._dd = mujoco.MjData(m)
        self._qfull = home_qpos(m, spec)
        self._rne = np.zeros(m.nv)
        tip_body = m.body(spec["tip"]["body"]).id
        self._tool_bodies = {m.body(b).name for b in range(m.nbody) if self._descends(b, tip_body)}
        self._calls = 0
        self._depth = 0
        self.kd = np.asarray(spec["kd"], dtype=float)
        self._armature = np.asarray(m.dof_armature[self._dadr], dtype=float)
        self._frictionloss = np.asarray(m.dof_frictionloss[self._dadr], dtype=float)
        self._damping = np.asarray(m.dof_damping[self._dadr], dtype=float)
        self._last = None            # the last Sense: the force estimate compares two readings one period apart
        ps = load_robot_mjcf(spec)   # the observer's own copy of the arm alone, stepped like the backend
        if self.cfg["physics_dt_s"] is None:   # the backend's own physics step; else the loaded model's
            step = getattr(backend, "physics_dt_s", None) or getattr(backend, "physics_dt", None)
            self.cfg["physics_dt_s"] = float(step if step else ps.option.timestep)
        ps.option.timestep = self.cfg["physics_dt_s"]
        ps.option.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self._pm = ps.compile()
        self._pd = mujoco.MjData(self._pm)
        pm = self._pm
        self._pqa = np.array([pm.jnt_qposadr[pm.joint(j).id] for j in model.joint_names])
        self._pda = np.array([pm.jnt_dofadr[pm.joint(j).id] for j in model.joint_names])
        self._pmot = np.array([pm.actuator("motor_" + j).id for j in model.joint_names])
        self._pfull = home_qpos(pm, spec)
        self._pctrl = np.zeros(pm.nu)
        g = spec.get("gripper")
        for a, v in zip(g["actuators"] if g else [], g["closed"] if g else []):
            self._pctrl[pm.actuator(a).id] = v
        self._q_cmd = None
        self._frac_cmd = 1.0
        self._q_des = None           # the commanded pose (without the gravity feed-forward)
        self._gripper_cmd = None
        self._pressing = False       # a stop left a joint pressing: ease before a picture
        self._history = []           # executed joint paths, newest last (I12 retreat)
        self.base_give = None        # I11: the last measured give, bounds later pushes
        self._looks = {}
        self._dark_look = None       # (reading id, brightness) of the newest light reading when it is dark, else None
        self._light = None           # the newest light reading (LooksMixin._note_light)
        self._n_look = 0
        self._marks = {}
        self._std_ref = None
        self._std_last = None
        self._n_picture = 0
        self.periods = {"moving": 0, "waiting": 0, "jaw": 0}   # control periods this session ran, by what for
        self._period_kind = "moving"
        self.__dict__["_hold"] = None   # what the hand holds: the one record (control_hold; read it as _held)
        self._selftest = None        # what the self-test did, once it was asked for (control_speed._selfknow_gate)
        self.bounds = _bounds.Bounds(self)   # before the packages' setup: every bound is asked of it
        s = self._sense()
        self._q_des = s.q.copy()

    # ---------------------------------------------------------------- the last act that changed the scene (C10)
    _scene_t = None   # s: the time of the last control period in which a guard read a load over its threshold or
    _pace = None      # the jaws were told to move; _pace: what the last move's pacing did

    def _scene_touch(self, s=None):
        """The scene may have changed in this control period: a look taken before it shows the scene as it was."""
        self._scene_t = float(self.sim_time() if s is None or getattr(s, "t", None) is None else s.t)

    # ---------------------------------------------------------------- what the hand holds
    @property
    def _held(self):
        """The hold record (rcpx/control_hold.py), or None."""
        return control_hold.held(self)

    def _quiet(self, s, need_m):   # r6 shim: until the motion files' own _quiet judges through control_hold
        return control_hold.quiet(self, s, need_m)

    # ---------------------------------------------------------------- the jaws in the body model
    def _jaws_enter(self):
        """While an operator's call runs (its pre hooks too), the body model poses the gripper joints at the newest
        reading, so the path check, the speed law and every package see an open jaw where it is (the capsules were
        fitted with the jaws closed). The model is shared by every user in the process (the scene's layout, the
        bench's planners): _jaws_exit poses it closed again when the outermost call ends."""
        self._jaw_depth = getattr(self, "_jaw_depth", 0) + 1
        if self._jaw_depth > 1:
            return
        last = self._last
        g = getattr(last, "gripper", None) if last is not None else None
        if isinstance(g, dict) and g.get("q") is not None and hasattr(self.model, "set_jaws"):
            self.model.set_jaws(g["q"])

    def _jaws_exit(self):
        self._jaw_depth = max(0, getattr(self, "_jaw_depth", 1) - 1)
        if self._jaw_depth == 0 and hasattr(self.model, "set_jaws"):
            closed = (self.model.spec.get("gripper") or {}).get("closed")
            if closed is not None:
                self.model.set_jaws(closed)

    def _jaws_follow(self, s):
        """Inside a call: pose the model's jaws at reading s (after the gripper moved them)."""
        g = getattr(s, "gripper", None)
        if getattr(self, "_jaw_depth", 0) > 0 and isinstance(g, dict) and g.get("q") is not None \
                and hasattr(self.model, "set_jaws"):
            self.model.set_jaws(g["q"])

    @staticmethod
    def _cfg_default(value, key, cfg):
        return cfg[key] if value is None else value

    def _descends(self, b, root):
        m = self.model.mj
        while b > 0:
            if b == root:
                return True
            b = m.body_parentid[b]
        return False

    # ---------------------------------------------------------------- time, journal
    def sim_time(self):
        return float(self.backend.read()["t"])

    def _journal(self, kind, **fields):
        if self.journal is None:
            return
        try:
            self.journal.append(kind, **fields)
        except Exception:  # a journal failure must not end the session (the server journals every call too)
            pass

    # ---------------------------------------------------------------- command
    def _command(self, q_des, torque_frac=None):
        """Servo target = q_des plus the gravity hold, so the arm sits at q_des and a push is the lead only."""
        q_des = np.asarray(q_des, dtype=float)
        self._q_des = q_des.copy()
        q_cmd = q_des + self.model.gravity_torque(q_des) / self.kp
        frac = self.cfg["torque_frac"] if torque_frac is None else torque_frac
        self._q_cmd, self._frac_cmd = q_cmd, frac
        jaws = self._gripper_cmd
        if hasattr(self, "_jaws_told") and np.any(np.asarray(jaws != self._jaws_told)):
            self._scene_touch()            # the jaws are told to move: the scene may change from here (C10)
        self._jaws_told = jaws
        self.backend.command(q_cmd, torque_frac=frac, gripper=jaws)

    def _period(self, q_des):
        self._command(q_des)
        self.backend.step(1)
        kind = self._period_kind
        self.periods[kind] = self.periods.get(kind, 0) + 1
        s = self._sense()
        if kind == "moving":         # the arm's last motion: a look taken before it frees no space in a later call
            self._moved_t = float(self.sim_time() if getattr(s, "t", None) is None else s.t)   #
        return s

    _moved_t = None   # s: the time of the last moving control period of this session

    def _open_call(self):
        """An operator's call begins: the arm's passage of earlier calls frees nothing from here, and a
        look frees space in this call only when it was taken after the arm's last motion."""
        control_move_passed.open_call(self)

    def _ease_impl(self, periods=None, max_periods=None):
        """Hold the present pose (gravity only) so nothing keeps pushing: the target follows the arm each
        period until the arm is quiet (control_hold.quiet: the motion left over one detection time under the
        tip's own resolution, and the force at rest less the held load under the level). It ends without that
        when nothing gets quieter: over one settle time neither the tip's speed nor that force fell by the
        relative share of itself. periods: the least a caller states; max_periods: a caller's own bound."""
        s = self._sense()
        least = 1 if periods is None else max(1, int(periods))
        B = self.bounds
        window = max(int(_ratio("QUANT")), int(math.ceil(_val(B.t_settle()) / self.dt)))
        rel, seen = _ratio("REL"), []
        was, self._period_kind = self._period_kind, "waiting"
        try:
            k = 0
            while max_periods is None or k < max(least, int(max_periods)):
                s = self._period(s.q)
                k += 1
                seen.append((float(np.linalg.norm(self.model.jac(s.q)[:3] @ s.qd)), control_hold.rest_force(self, s)))
                if k < least:
                    continue
                if control_hold.quiet(self, s, _val(B.rigid(s.q, s.R[:, 0]))):
                    break
                if len(seen) > window:
                    v0, f0 = seen[-1 - window]
                    if seen[-1][0] >= (1.0 - rel) * v0 and seen[-1][1] >= (1.0 - rel) * f0:
                        break                              # nothing gets quieter: this is how it rests
        finally:
            self._period_kind = was
        self._pressing = False
        return s

    def _ease_if_pressing(self):
        """Ease before a move when a stop left a joint pressing, or the still arm reads a force over its level
        that is not a hold's own (a hold whose load is not read yet, or one on a part of something fixed, reads
        what it holds: that is not eased away)."""
        s = self._sense()
        if self._pressing or control_hold.rest_force(self, s) > control_hold.level(self):
            return self._ease_impl(), True
        return s, False

    # ---------------------------------------------------------------- frames
    def _rot_of(self, frame, s):
        """Rotation whose columns are the named frame's axes in base coordinates."""
        if frame == "base":
            return np.eye(3)
        if frame == "tool":
            return s.R.copy()
        if frame == "camera":
            Rc = self.model.fk(s.q)["camera"][1]
            return np.column_stack([Rc[:, 0], -Rc[:, 1], -Rc[:, 2]])  # image right, image down, along the view
        return None

    def _unit_in_base(self, direction, frame, s):
        R = self._rot_of(frame, s)
        if R is None:
            return None, refusal("frame", f"unknown frame {frame!r}", "use base or tool")
        v = np.asarray(direction, dtype=float).reshape(-1)
        if v.size != 3 or not np.all(np.isfinite(v)) or np.linalg.norm(v) < 1e-9:
            return None, refusal("direction", "the direction must be a non-zero [x, y, z]", "e.g. [0, 0, -1] or down")
        return R @ (v / np.linalg.norm(v)), None

    def _verify_impl(self, claim):
        return self.world.verify(claim)

    @primitive
    def verify(self, **claim):
        """I9 through the World: a state is confirmed by a later look or a second sense."""
        return self._verify_impl(claim)

    # ---------------------------------------------------------------- the jaws (FINDINGS R3; A4.7)
    def _jaw_load(self, s):
        """The jaws' squeeze as a share of their servo's force (the largest jaw), from the reading; None without."""
        g, r = self.model.spec.get("gripper"), s.gripper
        if not g or not isinstance(r, dict) or r.get("q") is None or r.get("target") is None:
            return None
        kp = np.asarray(g.get("kp") or [0.0], dtype=float)
        fr = np.maximum(np.asarray(g.get("forcerange") or [1.0], dtype=float), TINY)
        load = np.abs(kp * (np.asarray(r["target"], dtype=float) - np.asarray(r["q"], dtype=float))) / fr
        return round(float(np.max(np.minimum(load, 1.0))), 3)

    def _squeeze_frac(self, share):
        """The opening (a share of the span) that makes the jaw servo squeeze with `share` of its force."""
        g = self.model.spec["gripper"]
        kp = np.asarray(g.get("kp") or [1.0], dtype=float)
        fr = np.asarray(g.get("forcerange") or [0.0], dtype=float)
        span = np.maximum(np.abs(np.asarray(g["open"], dtype=float) - np.asarray(g["closed"], dtype=float)), TINY)
        return float(np.mean(np.minimum(1.0, share * fr / np.maximum(kp, TINY) / span)))

    def _lead_share(self, lead):
        """The share of the jaw servo's force a command `lead` (a share of the span) past the reading gives."""
        unit = self._squeeze_frac(1.0)
        return round(min(1.0, max(0.0, float(lead)) / unit), 3) if unit > 0.0 else 0.0

    def _jaw_room(self, q, now, frac):
        """(opening, why): how far the jaws may open from `now` toward `frac` before a body they move comes newly
        within the clearance the bounds give a stored contact (its own error), nearer than it starts, by the body
        model with the jaws posed along the way. why is None when the whole opening is free. Only the jaw bodies
        are read (the tip resting on a part would hide them). Sets _jaw_near (clearance m, how fast it closes per
        unit of opening, the contact's id): the ramp reads it. The model's jaws are left at the newest reading."""
        # stored contacts only (a located part, a cube to close around, is what an opening makes room for)
        entries = [e for e in self.world.query(kind="contact", include_stale=True) if e.get("point_mm") is not None]
        known = np.asarray([np.asarray(e["point_mm"], dtype=float) / MM for e in entries], dtype=float).reshape(-1, 3)
        g = self.model.spec["gripper"]
        self._jaw_near = (float("inf"), 0.0, None)
        if not len(known) or frac <= now:
            return frac, None
        B = self.bounds
        clear = [max(0.0, _val(B.clearance(0.0, thing=e.get("id")))) for e in entries]
        lo, hi = np.asarray(g["closed"], dtype=float), np.asarray(g["open"], dtype=float)
        last = self._last.gripper.get("q") if isinstance(getattr(self._last, "gripper", None), dict) else None
        jaws = self.model.jaw_bodies()

        def nearest():
            best = (float("inf"), None, None)
            for name, p0, p1, r in self.model.collision_capsules(np.asarray(q, dtype=float))[1:]:
                if name not in jaws:
                    continue
                ab = p1 - p0
                L2 = float(ab @ ab)
                for i, kp in enumerate(known):
                    t = 0.0 if L2 <= 0.0 else min(1.0, max(0.0, float((kp - p0) @ ab) / L2))
                    dist = float(np.linalg.norm(p0 + t * ab - kp)) - r
                    if dist < best[0]:
                        best = (dist, i, name)
            return best

        steps = np.linspace(now, frac, JAW_ROOM_STEPS)
        self.model.set_jaws(lo + steps[0] * (hi - lo))
        c0, i0, _ = nearest()
        room, why, closing = frac, None, 0.0
        for k in range(1, len(steps)):
            self.model.set_jaws(lo + steps[k] * (hi - lo))
            c, i, name = nearest()
            closing = max(closing, (c0 - c) / max(steps[k] - steps[0], TINY))
            if c < clear[i] and c < c0 - NEWLY:
                room = float(steps[k - 1])
                why = (f"opening further would bring the {name} within {max(c, 0.0) * MM:.0f} mm of a stored point "
                       f"at [{', '.join(f'{x * MM:.0f}' for x in known[i])}] mm (an opening jaw beside a part pries "
                       f"it): the jaws opened to {room:.2f} of their span")
                break
        self._jaw_near = (c0, closing, None if i0 is None else entries[i0].get("id"))
        if last is not None:
            self.model.set_jaws(last)
        return room, why

    @primitive
    def gripper(self, width_frac, hold=True, force_frac=None, max_n=None):
        """Open or close the jaws (FINDINGS R3) through the core's one close and one release (rcpx/control_hold.py).
        A close runs at the jaws' own rate until their own reading tells a thing between them (ended on_thing) or
        they reach the asked opening (closed); then it holds: with force_frac stated, that share of the jaw
        servo's force; with a width stated between open and closed, that width (the caller's squeeze); else the
        closing command stands. hold false: the jaws rest on the part. max_n: the most the thing may take (your
        stated limit): the close then runs no faster at the touch than keeps it, and says what that rests on.
        An opening lets go of what was held and stops short of the jaws' own open end. No arm guard at the
        contact threshold judges either (the squeeze and the load leaving are the robot's own doing): the cap
        does. Both are moves toward the world: refused while the newest light reading is dark (cfg dark_gate)."""
        g = self.model.spec.get("gripper")
        if not g:
            return refusal("gripper", "this robot has no gripper: its tool is a fixed finger",
                           "use touch and press with the tip")
        if isinstance(width_frac, str):
            if width_frac not in ("open", "close"):
                return refusal("width_frac", f"unknown gripper command {width_frac!r}", "open, close or 0 to 1")
            frac = 1.0 if width_frac == "open" else 0.0
        else:
            frac = min(1.0, max(0.0, float(width_frac)))
        if force_frac is not None and not 0.0 <= float(force_frac) <= 1.0:
            return refusal("force_frac", "force_frac must be from 0 to 1 (a share of the jaws' servo force)",
                           "state the share of the jaw servo's force the hold needs, or none")
        if max_n is not None and not (math.isfinite(float(max_n)) and float(max_n) > 0.0):
            return refusal("max_n", "max_n must be a positive number of newtons", "the most the thing may take")
        dark = self._dark_refusal_now()
        if dark is not None:
            return dark
        gate = self._selfknow_gate("close")
        if gate is not None:
            return gate
        t0 = self._periods_now()
        s = self._sense()
        now = (s.gripper or {}).get("open_frac") if isinstance(s.gripper, dict) else None
        if now is None:   # no reading to close on: as asked, for the time a wait may last
            self._gripper_cmd = frac
            was, self._period_kind = self._period_kind, "jaw"
            for _ in range(self._wait_limit()):
                s = self._period(self._q_des)
            self._period_kind = was
            return {"width_frac": frac, "hold": bool(hold), "gripper": s.gripper, "jaw_load": None,
                    "stopped": "goal", "periods": self._periods_now() - t0}
        opening, why = frac >= float(now), None
        if opening:
            room = frac
            self._jaw_near = (float("inf"), 0.0, None)
            if control_hold.held(self) is None and hasattr(self.model, "set_jaws"):
                room, why = self._jaw_room(s.q, float(now), frac)   # empty jaws beside a stored part pry it
            if why is not None and room <= float(now):
                return refusal("jaw_room", why, "move the hand clear of the part first (retreat, or move_tip away "
                               "from it), then open")
            r = control_hold.release(self, room)
        else:
            share = None if force_frac is None else float(force_frac) * (control_hold.facts(self)["J"].squeeze_top_n()
                                                                         or 0.0)
            r = control_hold.close(self, to_frac=frac, squeeze_n=share, hold=bool(hold),
                                   limit_n=None if max_n is None else float(max_n), limit_source="intent")
        if "refused" in r:
            return {**r, "stopped": "refused"}
        if not hold and not opening and r["ended"] == "closed":
            self._gripper_cmd = float(r["frac"])     # stop squeezing: hold the opening it reached
            self._period(self._q_des)
        s = self._sense()
        self._jaws_follow(s)
        ended = r["ended"]
        out = {"width_frac": frac, "hold": bool(hold), "gripper": s.gripper, "jaw_load": self._jaw_load(s),
               "stopped": "room" if (why and ended == "opened") else ("goal" if ended in ("opened", "closed")
                                                                      else ended),
               "ended": ended, "periods": self._periods_now() - t0, "arrive_periods": r["arrive_periods"],
               "arrive_off_mm": round(float(r.get("arrive_off_m") or 0.0) * MM, 2)}
        if why and ended == "opened":
            out["why"] = why
            out["hint"] = "move the hand clear of the part first (retreat, or move_tip away from it), then open on"
        if opening:
            out["was_holding"] = bool(r["was_holding"])
            if r.get("why"):
                out["why"] = r["why"]
            if ended == "on_thing":
                out["blocked_opening"] = {"at_open_frac": round(float(r["frac"]), 4),
                                          "commanded_frac": round(float(self._gripper_cmd), 4),
                                          "why": "the opening jaw met something and stands there, holding that "
                                                 "opening (no push past it)"}
            return out
        out.update(peak_force_n=round(float(r["arm_read_n"]), 2), squeeze_n=r["squeeze_n"], price=r["price"],
                   close_span_s=round(float(r["rate_span_s"]), 4))
        if r.get("limit_n") is not None:
            out["limit"] = {"max_n": r["limit_n"], "source": r["limit_source"],
                            "kept": "shown kept" if r["kept"] else "not shown kept"}
        if r.get("pressed_world_n") is not None:
            out["pressed_world_n"] = r["pressed_world_n"]
        if ended == "on_thing":     # where the jaws met it (the key older readers look for)
            out["blocked"] = {"at_open_frac": round(float(r["frac"]), 4),
                              "commanded_frac": round(float(self._gripper_cmd), 4),
                              "squeeze_share": self._lead_share(float(r["frac"]) - float(self._gripper_cmd))}
        return out
