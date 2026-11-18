# SPDX-License-Identifier: AGPL-3.0-only
"""Shared machinery of the Executor (rcpx/control.py): the stated facts (DEFAULTS, Cfg), the primitive wrapper, Sense,
Guard and Felt.

No speed, force, torque, travel or time limit is stated here. Every bound a move runs under is asked of the session's
bounds (rcpx/bounds.py, ex.bounds): the contact thresholds are a hidden level plus a multiple of the reading's own
noise, the cap is the robot's own rating less its torque error, what counts as moved is the tip's own resolution.
A cfg holds stated facts and switches only; a key that used to hold a limit is answered from the bounds for readers
that have not moved yet (Cfg.COMPAT), with one journal note per key and session.

Contact = the external tip force estimate over the guard's threshold on its readings in a row (one reading over the
one reading threshold), or a joint's external torque over its own threshold; a cap ends the move with stopped "cap";
Felt keeps what a move met below those thresholds. While something is held the guard runs as
rcpx/control_hold.py Watch says (the first leg after a close has no threshold on the tool; a known hold has the
hold's own level; a contact goes on the hold). A threshold that was never measured is not known, never zero: such
a guard runs on the cap and says so. No robot is named here.
"""

import functools
import inspect
import math
import traceback
from types import SimpleNamespace

import numpy as np

from rcpx.context import jsonable, refusal
from rcpx.control_felt import BRUSH_BRIEF, HALF_STEP, Felt, _mm, _ratio  # noqa: F401  (their home since round six)

MM = 1000.0   # unit: mm per m

FACTS = {
    "torque_frac": 1.0,            # share of the torque limits the servo may use: the whole rating
    "physics_dt_s": None,          # the backend's physics step; None: read from the backend (its real servo step)
    "observer_max_s": 0.5,         # readings further apart than this fall back to the static torque estimate
    "observer_probe_frac": 0.02,   # the observer finds the arm's answer to a push with this share of each limit
    "observer_speed_weight": 1.0,  # weight of the end speed gap against the end pose gap (per period) in the solve
    "floor_z_m": 0.0,              # stated height (frame B) of the surface the robot stands on, away from its
                                   # mount: read as a level support plane with no stated error
    "support": None,               # the support plane as stated: {"normal", "offset_m", "sigma_m"}; None: floor_z_m
    "mount_half_m": None,          # half side of the square mount under the base (its top is z = 0 of B); over it
                                   # the floor is the mount top. None: no mount beyond the base itself
    "corrections": 3,              # end corrections of a move whose caller stated a tolerance (planner numerics)
    "lateral_gain_per_s": 15.0,    # per second: how fast a push undoes the tip's sideways drift off its line
    "give_lift": "none",           # after a give: "none" (the tool stands still where the part let go) or "back"
    "base_kind": None,             # what the owner says of the base: "fixed", "loose" or None (unknown)
    "history_max": 30,             # executed paths kept for a retreat (memory only: a retreat that runs out of
                                   # history plans on)
    "standard_q": None,            # standard look pose (rad, joint order); None: the home pose
    "park_q": None,                # rest pose; None: the home pose
    "frames_dir": None,            # where frames go when there is no journal
    "dark_gate": "looks",          # moves toward the world refused while the newest light reading is dark:
                                   # "looks" (a look arms it), "always", "off"
    "site": None,                  # the owner's site limits {"speed_mm_s", "force_n", "zones"}; None: none stated
    "people": None,                # True: people or machines may enter the reach
    "selfknow_dir": None,          # where the robot's self-knowledge file is kept
    "selfknow": None,              # None, "measure" (hardware: the owner allows the self-test) or "require"
}


def _val(x):
    """A bounds answer as a number: an ITEM dict's value, a (value, source) pair's first part, or the number."""
    if isinstance(x, dict):
        x = x.get("value")
    elif isinstance(x, (tuple, list)):
        x = x[0]
    elif hasattr(x, "value"):
        x = x.value
    return None if x is None else float(x)


class _Std:
    """The standard pose straight down: where a removed key's derived value is read."""

    def __init__(self, ex):
        self.ex, self.B = ex, ex.bounds
        self.q = np.asarray(ex._standard_q(), dtype=float)
        self.u = np.array([0.0, 0.0, -1.0])

    def thr(self, readings=2):
        return _val(self.B.f_thr(0.0, 0.0, readings=readings))

    def cap(self):
        B = self.B
        return min(x for x in (_val(B.f_cap(self.q, self.u)) * (1.0 - _val(B.e_tau())), _val(B.f_tool())) if x is not None)

    def unseen(self):
        return _val(self.B.f_allow(None, self.q, self.u))

    def v_free(self):
        return _val(self.B.v_free(self.q, self.u))

    def v_in(self):
        return _val(self.B.v_in(None, self.q, self.u))

    def rigid(self):
        return _val(self.B.rigid(self.q, self.u))

    def w_tool(self):
        return _val(self.B.standoff(0.0))

    def clear(self):
        return _val(self.B.clearance(0.0))

    def first(self):
        spec = self.B.force("touch", None, self.q, self.u)
        return _val(getattr(spec, "first_n", None) if not isinstance(spec, dict) else spec.get("first_n"))

    def travel(self, kind, name):
        spec = self.B.travel(kind, None, self.q, self.u)
        return _val(getattr(spec, name, None) if not isinstance(spec, dict) else spec.get(name))

    def ratio(self, a, b):
        return a / b if b else 1.0

    def periods(self, seconds):
        return seconds / _val(self.B.dt())

    def reach(self):
        return float(self.ex.model.length_scale)


# removed key -> (the bounds call that answers it now, the value in the key's old unit at the standard pose)
COMPAT = {
    "guard_force_n": ("B.f_thr(0, 0)", lambda c: c.thr()),
    "guard_one_reading_frac": ("B.f_thr(readings=1) over B.f_thr()", lambda c: c.ratio(c.thr(1), c.thr())),
    "guard_torque_frac": ("B.tau_thr(0) over B.tau_max()", lambda c: float(np.max(
        np.asarray(c.B.tau_thr(0.0), dtype=float) / np.asarray(c.B.tau_max(), dtype=float)))),
    "max_tip_force_n": ("min(B.f_cap(q, u) x (1 - B.e_tau()), B.f_tool())", lambda c: c.cap()),
    "cap_frac": ("1 - B.e_tau()", lambda c: 1.0 - _val(c.B.e_tau())),
    "observer_free_n": ("nothing: the estimate is never zeroed", lambda c: None),
    "static_estimate_max_n": ("nothing: a dead key", lambda c: None),
    "locate_resid_frac": ("nothing: a dead key", lambda c: None),
    "plane_margin_m": ("B.clearance(v, support, body)", lambda c: c.clear()),
    "tool_plane_margin_m": ("B.clearance(v, support, body)", lambda c: c.clear()),
    "self_margin_m": ("B.clearance(v)", lambda c: c.clear()),
    "guard_overtravel_m": ("B.travel(kind, thing, q, u).search_m", lambda c: c.travel("touch", "search_m") or 0.0),
    "hold_force_n": ("B.f_allow(None, q, u)", lambda c: c.unseen()),
    "hold_m": ("B.d_stop(B.v_free(q, u), q, u)", lambda c: _val(c.B.d_stop(c.v_free(), c.q, c.u))),
    "near_force_n": ("B.f_allow(thing, q, u)", lambda c: c.unseen()),
    "near_m": ("B.standoff(v, thing)", lambda c: c.w_tool()),
    "stop_periods": ("B.t_det() over B.dt()", lambda c: c.periods(_val(c.B.t_det()))),
    "impact_gain": ("B.slope(q, u, thing)", lambda c: 1.0),
    "stop_damping_gain": ("B.slope(q, u, thing)", lambda c: 0.0),
    "stop_ceiling_n": ("B.f_allow(None, q, u)", lambda c: c.unseen()),
    "creep_force_n": ("B.f_allow(thing, q, u)", lambda c: c.unseen()),
    "creep_m": ("B.standoff(v, thing)", lambda c: c.w_tool()),
    "leave_clear_m": ("B.clearance(v, thing)", lambda c: c.clear()),
    "path_clear_m": ("B.clearance(v, thing)", lambda c: c.clear()),
    "speed_mm_s": ("B.v_free(q, u)", lambda c: c.v_free() * MM),
    "touch_force_n": ("B.force('touch', thing, q, u).first_n", lambda c: c.first()),
    "touch_speed_mm_s": ("B.v_in(thing, q, u)", lambda c: c.v_in() * MM),
    "touch_speed_max_mm_s": ("B.v_free(q, u)", lambda c: c.v_free() * MM),
    "touch_slow_frac": ("B.f_thr() over the first-touch force", lambda c: min(1.0, c.ratio(c.thr(), c.first()))),
    "touch_slow_mm_s": ("B.v_in(thing, q, u)", lambda c: c.v_in() * MM),
    "touch_react_s": ("B.t_det()", lambda c: _val(c.B.t_det())),
    "touch_part_n_per_mm": ("B.k_servo(q, u)", lambda c: _val(c.B.k_servo(c.q, c.u)) / MM),
    "resist_force_n": ("B.f_thr(0, 0)", lambda c: c.thr()),
    "moved_part_mm": ("B.rigid(q, u, force_n)", lambda c: c.rigid() * MM),
    "moved_part_report_mm": ("B.rigid(q, u, force_n)", lambda c: c.rigid() * MM),
    "press_speed_mm_s": ("B.v_in(thing, q, u)", lambda c: c.v_in() * MM),
    "rot_speed_deg_s": ("B.w_free(q, axis)", lambda c: math.degrees(_val(c.B.w_free(c.q, -c.u)))),
    "joint_speed_rad_s": ("B.qd_max()", lambda c: float(np.min(np.asarray(c.B.qd_max(), dtype=float)))),
    "plan_joint_step_rad": ("B.qd_max() x B.dt()", lambda c: float(np.min(np.asarray(c.B.qd_max(), dtype=float)))
                            * _val(c.B.dt())),
    "ramp_s": ("B.v_free(q, u) over B.a_brake(q, u)", lambda c: c.ratio(c.v_free(), _val(c.B.a_brake(c.q, c.u)))),
    "settle_s": ("B.t_settle()", lambda c: _val(c.B.t_settle())),
    "goal_tol_mm": ("B.rigid(q, u)", lambda c: c.rigid() * MM),
    "max_move_s": ("B.limit_s(planned_s)", lambda c: _val(c.B.limit_s(c.ratio(c.reach(), c.v_in())))),
    "nudge_max_mm": ("the stated travel stands", lambda c: c.reach() * MM),
    "hold_s": ("B.t_settle()", lambda c: _val(c.B.t_settle())),
    "give_mm": ("B.rigid(q, u)", lambda c: c.rigid() * MM),
    "give_min_n": ("B.f_thr(0, 0)", lambda c: c.thr()),
    "give_tip_mm": ("B.rigid(q, u)", lambda c: c.rigid() * MM),
    "off_axis_frac": ("nothing: a stroke ends when the tip leaves its line by the tool's width (B.standoff(0))",
                      lambda c: math.inf),
    "release_lift_mm": ("B.rigid(q, u)", lambda c: c.rigid() * MM),
    "stale_near_mm": ("B.standoff(0)", lambda c: c.w_tool() * MM),
    "touch_release_mm": ("B.travel('touch', thing, q, u).backoff_m", lambda c: (c.travel("touch", "backoff_m")
                                                                               or c.rigid()) * MM),
    "slip_mm": ("B.standoff(0)", lambda c: c.w_tool() * MM),
    "stall_mm": ("B.rigid(q, u)", lambda c: c.rigid() * MM),
    "stall_periods": ("B.t_settle() over B.dt()", lambda c: max(1.0, c.periods(_val(c.B.t_settle())))),
    "rounds_load_frac": ("1 - B.e_tau()", lambda c: 1.0 - _val(c.B.e_tau())),
    "hard_push_n": ("B.f_hand()", lambda c: _val(c.B.f_hand())),
    "base_backoff_mm": ("B.standoff(0)", lambda c: c.w_tool() * MM),
    "max_base_shift_mm": ("B.rigid(q, u)", lambda c: c.rigid() * MM),
    "base_push_mm": ("B.travel('press', thing, q, u).past_m", lambda c: (c.travel("press", "past_m")
                                                                        or c.rigid()) * MM),
    "rigid_mm_per_n": ("1 over B.k_servo(q, u)", lambda c: c.ratio(MM, _val(c.B.k_servo(c.q, c.u)))),
    "eased_force_n": ("B.f_thr(0, 0)", lambda c: c.thr()),
    "retreat_default": ("nothing: a retreat ends when every link is clear", lambda c: c.ex.cfg["history_max"]),
    "grip_close_frac_s": ("the jaws' own measured speed", lambda c: c.ex._jaw_rate()),
    "grip_block_frac": ("the jaws' free lag and its spread", lambda c: c.ex._jaw_lag()),
    "grip_squeeze_share": ("B.grip(held)", lambda c: c.ex._jaw_share()),
    "grip_touch_share": ("B.grip(held)", lambda c: c.ex._jaw_share(hold=False)),
    "lateral_gain": ("cfg lateral_gain_per_s x B.dt()", lambda c: c.ex.cfg["lateral_gain_per_s"] * _val(c.B.dt())),
}
# what a reader with no session gets from DEFAULTS[key]: a stated plane with no stated error carries no margin
SESSIONLESS = {"plane_margin_m": 0.0, "tool_plane_margin_m": 0.0, "self_margin_m": 0.0, "guard_overtravel_m": 0.0}


class _Defaults(dict):
    """DEFAULTS: the stated facts. A removed key read here (no session to derive it from) answers what a robot
    with nothing measured gives it (SESSIONLESS), else None; `in` and get() see the facts only."""

    def __missing__(self, key):
        if key in COMPAT:
            return SESSIONLESS.get(key)
        raise KeyError(key)


DEFAULTS = _Defaults(FACTS)


class Cfg(dict):
    """Executor.cfg: stated facts and switches. A key round four removed is not stored: with COMPAT_ON a read of one
    returns what the session's bounds derive for it at the standard pose straight down and journals one note per
    key; a write of one is ignored with a note. With COMPAT_ON off (after the Review stage) a read raises KeyError
    naming the bounds call."""

    COMPAT_ON = True

    def __init__(self, values=None, ex=None):
        super().__init__()
        self._ex, self._noted, self.ignored = ex, set(), []
        for k, v in dict(FACTS, **(values or {})).items():
            self[k] = v

    def _note(self, key, what, **more):
        if (key, what) in self._noted:
            return
        self._noted.add((key, what))
        ex = self._ex
        if ex is not None:
            ex._journal("cfg_compat", key=key, what=what, ask=COMPAT[key][0], **more)

    def _derived(self, key):
        ex = self._ex
        if ex is None or getattr(ex, "bounds", None) is None:
            return SESSIONLESS.get(key)
        try:
            value = COMPAT[key][1](_Std(ex))
        except Exception as exc:   # a getter the bounds do not answer yet: no number is made up
            self._note(key, "unanswered", error=f"{type(exc).__name__}: {exc}")
            return SESSIONLESS.get(key)
        self._note(key, "read", value=value)
        return value

    def __missing__(self, key):
        if key in COMPAT:
            if not self.COMPAT_ON:
                raise KeyError(f"cfg[{key!r}] was removed in round four: ask {COMPAT[key][0]}")
            return self._derived(key)
        raise KeyError(key)

    def get(self, key, default=None):
        if key in COMPAT and self.COMPAT_ON and not dict.__contains__(self, key):
            v = self._derived(key)
            return default if v is None else v
        return dict.get(self, key, default)

    def __setitem__(self, key, value):
        if key in COMPAT:
            self.ignored.append(key)
            self._note(key, "write_ignored", value=jsonable(value))
            return
        dict.__setitem__(self, key, value)

    def update(self, *args, **kwargs):
        for k, v in dict(*args, **kwargs).items():
            self[k] = v


def primitive(fn):
    """Outermost call: tick the World with the call index and simulated time, begin the call in the bounds (what the
    fixed cameras show changed is no longer free), catch every exception into a refusal, take an operator's word
    back, journal the result. Nested primitive calls (park calling retreat) only run."""
    name = fn.__name__

    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        if self._depth > 0:
            return fn(self, *args, **kwargs)
        self._calls += 1
        if not getattr(self, "_call_opened", False):   # a kernel with hooks opened it at its own entry
            opened = getattr(self, "_open_call", None)
            if opened is not None:
                opened()
        try:      # the call as it was made: a refusal that offers the same call with another word reads it (C12)
            self._call_now = (name, dict(list(inspect.signature(fn).bind(self, *args, **kwargs).arguments.items())[1:]))
        except TypeError:
            self._call_now = (name, dict(kwargs))
        try:
            self.world.tick(self._calls, self.sim_time())
        except Exception:  # a backend that cannot read time still gets a result below
            pass
        self._depth += 1
        jaws = getattr(self, "_jaws_enter", None)
        if jaws is not None:
            jaws()
        begin, end = getattr(self, "_call_begin", None), getattr(self, "_call_end", None)
        moved0 = dict(getattr(self, "periods", None) or {}).get("moving", 0)
        try:
            if begin is not None:
                begin(name)
            out = fn(self, *args, **kwargs)
            out = _hold_lines(self, name, out, moved0)
        except Exception as exc:
            out = refusal("internal", f"{name} failed inside the executor: {type(exc).__name__}: {exc}",
                          "the arm may have moved; look or read robot://world before the next move")
            self._journal("primitive_error", name=name, trace=traceback.format_exc())
        finally:
            self._depth -= 1
            if end is not None:
                end(name)
            if jaws is not None:
                self._jaws_exit()
        out = jsonable(out if isinstance(out, dict) else {"value": out})
        self._journal("primitive", name=name, args=jsonable(kwargs), result=out)
        return out

    return wrapper


class Sense:
    """One reading: q, qd, tau, the external force the tip applies to the world (base frame, N) and the tip. raw:
    the estimate before a held load was taken off it (ext0: its joint torques; hidden: what the joints' dry
    friction can hide at this reading); fast: the backend's fast signal over the period, or None; overload: the
    firmware's own overload flag."""

    __slots__ = ("t", "q", "qd", "tau", "ext", "F", "Fmag", "tip", "R", "gripper", "where", "at", "loc", "Fend",
                 "raw", "fast", "overload", "peak_note", "ext0", "hidden")


def stated_guard(force_n, need=2, cap_n=math.inf):
    """A guard a caller states in full (a test on the Guard alone, a stroke whose force the operator named)."""
    return SimpleNamespace(force_n=force_n, one_reading_n=force_n, tau=None, direction=None, cap_n=cap_n,
                           readings=int(need))


class Guard:
    """Contact, cap and stop-request detection over a stream of Sense readings, by a guard spec the bounds derive
    (spec: force_n on `readings` readings in a row, one_reading_n on one, tau per joint, direction, cap_n).

    Contact: the tip force estimate over force_n on the spec's readings in a row, or over one_reading_n once, or a
    joint's external torque over its own threshold (a push on any link). With a direction (a move that leaves a
    touched thing) only a force that opposes the travel counts: the force of leaving does not.
    Cap: the tip force estimate over cap_n (once); the firmware's overload flag (once); or, on two readings in a
    row, a joint at its rating BECAUSE of an external torque: the applied torque is at the rating, the external
    part is over that joint's threshold and pushes the same way. A joint the plan itself holds at its rating (a
    torque limited start) is not a cap.
    A caller that passes force_n with no kind and no spec states the threshold itself (never under the derived
    hidden level); force_n None with no kind and no spec leaves only the cap and the stop request (a stroke that
    presses on purpose). A spec whose force_n is None, or whose threshold was never measured (it reads inf or zero),
    leaves the cap, a push located off the tool and, with something held, the lost hold; `unknown` says so.
    The cap is what the arm itself can push (control_hold.arm_cap), never a jaw motor's force. While something
    is held (hold True and a record stands) self.hold is the hold's Watch: it sets the thresholds, says what is
    judged (Watch.judged) and is told the word a leg ends on. A tick hook's stop request (ex._tick_stop) ends
    the move with stopped equal to its text; so does a lost hold.
    A move that starts on a contact (the design rulings; rcpx/control_tail.py start_of, leaving): the spec of a move the plan
    door timed carries `start`. Its base, the load the still hand read as the move began, is no contact the
    motion made. Until the contact with the thing the move leaves has ended, that thing ends the move only over
    its stated limit or by a push that keeps growing; the cap and a push on a link end it as ever. Every guard
    writes where the tool last bore on something over what an arm at rest can hide (ex._stand)."""

    def __init__(self, ex, force_n=None, joint_guard=True, need=2, *, spec=None, kind=None, held=None,
                 away_from=None, v=0.0, a=0.0, hold=True):
        self.ex, self.joint_guard, self.need = ex, joint_guard, int(need)
        stated = spec is None and kind is None       # a caller that states its own threshold (or none)
        derive = getattr(ex, "_guard_spec", None)
        if spec is None and derive is not None:
            spec = derive(kind or "transit", v=v, a=a, held=held, away_from=away_from, readings=self.need)
        if spec is None:
            spec = stated_guard(force_n, need)
        self.spec = spec
        self.need = int(getattr(spec, "readings", None) or self.need)
        base, one = getattr(spec, "force_n", None), getattr(spec, "one_reading_n", None)
        if stated and force_n is None:
            self.force_n = None                      # no contact stop on this stroke: the cap and a stop request
        elif stated and base is not None and float(force_n) > float(base):
            self.force_n = float(force_n)            # stated over the derived threshold: the caller's force stands
            one = self.force_n + (float(one) - float(base) if one is not None else 0.0)
        else:
            self.force_n = None if base is None else float(base)
        self.one_n = self.force_n if one is None else float(one)
        tau = getattr(spec, "tau", None)
        self.tau = None if tau is None else np.asarray(tau, dtype=float)
        d = getattr(spec, "direction", None)
        self.direction = None if d is None else np.asarray(d, dtype=float)
        self.unknown = None
        if not stated and self.force_n is not None and not 0.0 < self.force_n < math.inf:
            # a hidden level that was never measured (it reads inf, or zero) is not known: no threshold stands on it
            self.force_n = self.one_n = None
            self.unknown = ("the contact threshold of this arm is not known (nothing measured yet): this move ran "
                            "on the cap alone")
            notes = ex.__dict__.setdefault("_guard_notes", []) if hasattr(ex, "__dict__") else []
            if self.unknown not in notes:
                notes.append(self.unknown)
        if self.tau is not None:
            self.tau = np.where(self.tau > 0.0, self.tau, math.inf)   # a joint level never measured judges nothing
        self.cap_n = _hold().arm_cap(ex, spec) if hasattr(ex, "bounds") else float(
            math.inf if getattr(spec, "cap_n", None) is None else spec.cap_n)
        self.n_force = self.n_cap = 0
        self.first = None      # Sense at the first over-threshold sample of the run that tripped
        self.calm = self.before = None   # the last sample under the threshold, and the one before the run
        self.contact = None
        self.cap_why = ""
        self.asked = None      # the text of a tick hook's stop request, once one ended the move
        self.peak = 0.0
        self.impulse = 0.0     # N s: the force over the threshold, summed over the readings of the stop
        self.ext_peak = None   # N m per joint: the largest external torque the move read
        self.felt = Felt(ex)
        self.hold = None       # control_hold.Watch while something is held
        if hold and hasattr(ex, "bounds") and _hold().held(ex) is not None:
            self.hold = _hold().Watch(self, stated)
        # a move that starts on a contact: the baseline, and the thing the move leaves (control_tail)
        self.base, self.leave = _tail().started(self, spec, stated) if spec is not None else (None, None)

    def _joints_at_cap(self, s):
        """Joints the EXTERNAL torque took to their rating (bool per joint)."""
        if self.tau is None or getattr(s, "ext", None) is None:
            return None
        ex = self.ex
        lim = ex.torque_limit * ex.cfg["torque_frac"]
        ext, tau = np.asarray(s.ext, dtype=float), np.asarray(s.tau, dtype=float)
        if ext.shape != lim.shape or tau.shape != lim.shape:
            return None
        return (np.abs(tau) >= lim - self.tau) & (np.abs(ext) > self.tau) & (ext * tau > 0.0)

    def check(self, s):
        """One reading: None, or the word that ends the move (contact, cap, a stop request's or a lost hold's
        text). While something is held the hold's Watch is told the word (the leg's end reads it)."""
        stop = self._check(s)
        if stop and self.hold is not None:
            self.hold.stopped(stop)
        return stop

    def _check(self, s):
        ex = self.ex
        f = max(s.Fmag, float(getattr(s, "Fend", None) or 0.0))   # the period's end reads a rising push harder
        self.peak = max(self.peak, f)
        self.felt.add(s)
        tool = (getattr(s, "loc", None) or {}).get("source", "tool") == "tool"
        if tool and f > self.felt.floor > 0.0:
            # where the tool last bore on something, over what an arm at rest can hide: a move that starts
            # there starts on that contact (control_expect.start_of, the design rulings)
            ex._stand = {"tip": s.tip, "F": s.F, "call": getattr(ex, "_calls", None)}
        ext = getattr(s, "ext", None)
        if ext is not None:
            self.ext_peak = np.abs(ext) if self.ext_peak is None or np.shape(self.ext_peak) != np.shape(ext) \
                else np.maximum(self.ext_peak, np.abs(ext))
        asked = getattr(ex, "_tick_stop", None)
        if asked:
            self.asked = str(asked)
            return self.asked
        over_force = f > self.cap_n
        overload = bool(getattr(s, "overload", False))
        at_cap = self._joints_at_cap(s)
        joints = at_cap is not None and bool(at_cap.any())
        if over_force or overload or joints:
            self.n_cap += 1
            if over_force or overload or self.n_cap >= 2:   # two readings: one period of a hard start is no cap
                return self._cap(s, at_cap, over_force, overload)
        else:
            self.n_cap = 0
        hold = self.hold
        if hold is not None:
            lost = hold.see(s)                 # the lost hold ends the move; the first leg's travel and load
            if lost:
                self.asked = lost
                return lost
        blind = self.force_n is None           # no threshold on the tool (a stroke; the first leg; not measured)
        if blind and (self.tau is None or (hold is None and self.unknown is None)):
            return None
        leaving = False
        if blind:
            size = hard_size = 0.0
        elif self.direction is not None:
            size = max(0.0, float(np.asarray(s.F, dtype=float) @ self.direction))   # what opposes the travel
            hard_size = size
        else:
            size, hard_size = s.Fmag, f
            if hold is not None:               # a known hold: the change from a reading that already stood
                size, hard_size = hold.judged(s, size, hard_size)
            elif self.base is not None:        # the load the move started in is no contact the motion made
                size, hard_size = _tail().off_base(self, s, f)
            if self.leave is not None and tool:   # the thing the move started on, until that contact has ended
                if _tail().leaving(self, s, size, hard_size):
                    return "contact"
                leaving = self.leave is not None
                if leaving:
                    size = hard_size = 0.0
        over = not blind and size > self.force_n
        # the joints judge any push; with a direction or with no threshold on the tool, only a push located off
        # the tool; with something held, only what no wrench through the tool and the held thing explains (a
        # part that is worked loads the arm with a force and a couple at the tip), and where the arm's joints
        # cannot tell that apart, a push located off the tool unless the hold is on a part of something fixed.
        # With something held they are asked even when the tool's own threshold told the push: where it is
        # located (the reading's own locating, and what the joints say) decides whether it is kept on the hold
        # or stays a contact in the world.
        off_tool = False
        if self.joint_guard and self.tau is not None and (hold is not None or not over):
            ext = np.asarray(getattr(s, "ext", np.zeros(0)), dtype=float)
            if hold is not None:
                beyond = (getattr(s, "loc", None) or {}).get("beyond")
                if beyond is not None:     # over the fit's own accepted error of what the hold passes on
                    beyond = np.asarray(beyond, dtype=float)
                    judged = float(np.linalg.norm(beyond)) > _ratio("REL") * float(np.linalg.norm(ext))
                    ext = beyond
                else:
                    judged = not tool and hold.mode != "attached"
            else:
                judged = not tool or not (self.direction is not None or blind or leaving)
            off_tool = judged and ext.shape == self.tau.shape and bool(np.any(np.abs(ext) > self.tau))
            over = over or off_tool
        if over:
            touch = getattr(ex, "_scene_touch", None)   # a load over the threshold: the scene may have changed (C10)
            if touch is not None:
                touch(s)
            if self.n_force == 0:
                self.first, self.before = s, self.calm
                self.impulse = 0.0
            self.n_force += 1
            self.impulse += size * float(getattr(ex, "dt", 0.0) or 0.0)
            if self.n_force >= self.need or (self.one_n is not None and hard_size > self.one_n):
                self.contact = _move_guard().met(ex, ex._contact_info(self.first, s), self.before)
                if hold is not None and not off_tool:
                    # a push located on the tool while holding is on the hold, never a thing in the way; one
                    # the joints or the reading's own locating put off the tool (a link) stays a contact (C9:
                    # control_hold.note_contact leaves a contact whose source is not the tool as it is)
                    _hold().note_contact(ex, self.contact)
                return "contact"
        else:
            self.n_force = 0
            self.first, self.calm = None, s
        return None

    def _cap(self, s, at_cap, over_force, overload):
        ex = self.ex
        touch = getattr(ex, "_scene_touch", None)
        if touch is not None:
            touch(s)
        if over_force:
            f = max(s.Fmag, float(getattr(s, "Fend", None) or 0.0))
            # what the arm can push is no rating: said as what it is
            self.cap_why = (f"tip force {f:.1f} N over the {self.cap_n:.1f} N this arm and its tool can push here "
                            "(its joints' torque limits, or the tool's own stated rating where that is less)")
        elif overload:
            self.cap_why = "the robot's own controller reported an overload"
        else:
            lim = ex.torque_limit * ex.cfg["torque_frac"]
            j = int(np.argmax(np.where(at_cap, np.abs(s.ext) / lim, -1.0)))
            self.cap_why = (f"an outside torque took joint {ex.model.joint_names[j]} to its rating "
                            f"({abs(s.tau[j]):.2f} N m, {abs(s.ext[j]):.2f} N m of it from outside)")
        self.contact = _move_guard().met(ex, ex._contact_info(self.first or s, s), self.before if self.first is not None
                                         else self.calm)
        return "cap"


def _deg(x):
    return round(math.degrees(float(x)), 3)


def _hold():
    from rcpx import control_hold     # here: that module's close builds a Guard
    return control_hold


def _tail():
    from rcpx import control_tail     # here: that module imports this one
    return control_tail


def _move_guard():
    from rcpx import control_move_guard   # here: that module reads the readings this one makes
    return control_move_guard


def _hold_lines(ex, name, out, moved0):
    """The hold's own part of an operator's call (control_hold.call_end); a failure there keeps the result."""
    if not hasattr(ex, "bounds") or not hasattr(ex, "periods"):
        return out
    try:
        return _hold().call_end(ex, name, out, ex.periods.get("moving", 0) > moved0)
    except Exception as exc:
        ex._journal("hold_problem", where="call_end", error=f"{type(exc).__name__}: {exc}")
        return out


def budget_refusal(ex, plan, periods, intent=None):
    """None, or the refusal budget: a move is not started when the backend states how many control periods are
    left (a harness budget, Q14) and the plan needs more than are left. The hint is a shorter move."""
    left = getattr(ex.backend, "steps_left", None)
    left = left() if callable(left) else None
    if left is None or periods <= left:
        return None
    share = max(0.0, float(left) / float(periods)) if periods else 0.0
    return refusal("budget", f"this move would take {int(periods)} control periods and {int(left)} are left",
                   f"ask for a shorter move: about {int(share * 100)} percent of this one fits the periods left")
