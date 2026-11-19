# SPDX-License-Identifier: AGPL-3.0-only
"""A shim: today's five-method backend (rcpx/backend.py: read, command, step, render, camera_info) behind the eight
calls of the body contract, without editing it.

    from rcpx.body.shim import BackendBody
    body = BackendBody(backend, declaration)          # declaration: the body's own data, kept by whoever owns
    body.describe(); body.read(["body.time"])         # the body (never in this folder)

What the backend states, the shim passes on; what it does not state, the shim answers "unknown". It adds nothing
of its own about the body: groups, holders, sensors, safe states and the controller come from the declaration it
is given, and the mapping from the declaration's names to the backend's one joint vector and one optional
two-state holder is the declaration's "backend_map":

    {"arm": group name of the backend's joint vector, "holder": group name of its holder or null,
     "sensors": {declared sensor name: the backend's camera name}}

Signals (names accepted by read): for the arm group G: G.position, G.speed, G.effort, G.effort_share, G.target,
G.effort_cap; for the holder group H: H.position, H.target, H.opening; and body.time, body.commands,
body.faults. Units are the backend's (SI: rad or m, their rates, N m), stated as such. G.target is the target the
backend holds after its last command: its attribute q_target where it keeps one, else its reading of that name
(the five methods do not promise either; a backend that tells neither gets "unknown" for whether a command was
clipped).

Commands: the arm group takes mode "position" (one target per member) and the limit "effort_share" (the cap on
each joint's effort as a share of its own limit; it stays in force until changed). The holder group takes mode
"position" with one number, the share of its opening (0 closed, 1 open). Every command carries limits["action"].

Safe states the shim can honour, by kind: a controlled stop with power kept (the target becomes the measured
pose, so the command after a stop equals the measured pose) and power removed (the same, and the effort cap goes
to nothing). A declared state of another kind is refused, and the conformance check then says so.
"""

import copy
import math

from rcpx.body import checks
from rcpx.body import contract as C

ARM_READ = ("position", "speed", "effort", "effort_share", "target", "effort_cap")
HOLDER_READ = ("position", "target", "opening")


def _list(x):
    """A plain list of floats from whatever the backend returns."""
    if x is None:
        return None
    try:
        return [float(v) for v in x]
    except TypeError:
        return [float(x)]


class BackendBody(C.Body):
    def __init__(self, backend, declaration, controller=None):
        self.backend = backend
        self.decl = copy.deepcopy(declaration)
        bmap = self.decl.get("backend_map") or {}
        self.arm = bmap.get("arm")
        self.holder = bmap.get("holder")
        self.cameras = dict(bmap.get("sensors") or {})
        if C.group_named(self.decl, self.arm) is None:
            raise ValueError("the declaration's backend_map names no declared group for the backend's joints")
        if self.holder is not None and C.group_named(self.decl, self.holder) is None:
            raise ValueError("the declaration's backend_map names a holder group that is not declared")
        self._controller = copy.deepcopy(controller if controller is not None else self.decl.get("controller"))
        self._counts = {}
        self._faults = []
        self._period_measured = None
        first = backend.read()
        self._n = len(_list(first["q"]))
        if len(C.group_named(self.decl, self.arm)["members"]) != self._n:
            raise ValueError("the declared group has another number of members than the backend has joints")
        tgt = self._commanded(first)
        self._arm_target = tgt if tgt is not None else _list(first["q"])
        self._share = float(first.get("torque_frac", 1.0))
        self._holder_share = None

    def _commanded(self, r):
        """The target the backend holds after its last command (clipped to its range), as far as it tells: the
        attribute q_target where the backend keeps one, else the reading's "q_target". Some backends report under
        that reading the goal in force, which trails the command; the attribute is the command itself. None when
        the backend tells neither."""
        held = getattr(self.backend, "q_target", None)
        if held is not None:
            return _list(held)
        return _list(r["q_target"]) if r.get("q_target") is not None else None

    # ------------------------------------------------------------------ what the body states
    def describe(self):
        out = {k: copy.deepcopy(self.decl.get(k)) for k in C.DESCRIBE_KEYS}
        out["ungrouped"] = list(self.decl.get("ungrouped") or [])
        return out

    def _rate(self):
        dt = getattr(self.backend, "dt", None)
        if not isinstance(dt, (int, float)) or dt <= 0:
            return C.unknown("the backend states no control period")
        return C.stated(1.0 / float(dt), "Hz", "description: one command and one reading per control period, the "
                        "period as the backend states it")

    def _ranges(self, members, key):
        fields = {j["name"]: j.get(key) for j in self.decl.get("joints") or ()}
        vals = [fields.get(m) for m in members]
        if any(v is None or C.is_unknown(v) for v in vals):
            return C.unknown("not stated for every member")
        return C.stated([v["value"] for v in vals], vals[0].get("unit"), vals[0].get("source"))

    def interfaces(self):
        rate = self._rate()
        none = C.unknown("the backend states none")
        arm_members = C.group_named(self.decl, self.arm)["members"]
        units = {"position": "rad or m, per member as the joint is a hinge or a slide",
                 "speed": "rad/s or m/s", "effort": "N m or N, as the servo applies it",
                 "effort_share": "share of each joint's effort limit, signed", "target": "rad or m",
                 "effort_cap": "share of each joint's effort limit"}
        arm = {"command": {"position": {"unit": units["position"], "range": self._ranges(arm_members, "range"),
                                        "resolution": none, "rate": rate}},
               "limits": {"effort_share": {"unit": units["effort_cap"],
                                           "range": C.stated([0.0, 1.0], None, "description: the backend's "
                                                             "command takes a share from nothing to the whole "
                                                             "limit"),
                                           "resolution": none, "rate": rate}},
               "read": {q: {"unit": units[q], "range": (self._ranges(arm_members, "range") if q in
                                                         ("position", "target") else none),
                            "resolution": none, "rate": rate} for q in ARM_READ}}
        groups = {self.arm: arm}
        signals = [f"{self.arm}.{q}" for q in ARM_READ]
        if self.holder is not None:
            share = C.stated([0.0, 1.0], None, "description: the backend's holder takes a share of its opening")
            groups[self.holder] = {
                "command": {"position": {"unit": "share of the opening (0 closed, 1 open)", "range": share,
                                         "resolution": none, "rate": rate}},
                "read": {"position": {"unit": "rad or m, per member", "range": none, "resolution": none,
                                      "rate": rate},
                         "target": {"unit": "rad or m, per member", "range": none, "resolution": none,
                                    "rate": rate},
                         "opening": {"unit": "share of the opening (0 closed, 1 open)", "range": none,
                                     "resolution": none, "rate": rate}}}
            signals += [f"{self.holder}.{q}" for q in HOLDER_READ]
        sensors = {}
        for name in self.cameras:
            sensors[name] = {"read": {"frame": {"unit": "picture, 8 bit per channel", "range": none,
                                                "resolution": none, "rate": none}}}
        signals += [C.TIME_SIGNAL, C.COMMANDS_SIGNAL, C.FAULTS_SIGNAL]
        return {"groups": groups, "sensors": sensors, "outputs": {}, "signals": signals}

    def controller(self):
        if self._period_measured is not None:
            period = C.stated(self._period_measured, "s", "self-measured: the body's clock across one control "
                              "period")
        elif isinstance(getattr(self.backend, "dt", None), (int, float)):
            period = C.stated(float(self.backend.dt), "s", "description: the period the backend states; not yet "
                              "measured (no period has passed)")
        else:
            period = None
        return C.controller_answer(self._controller, period)

    # ------------------------------------------------------------------ reading
    def _signals(self):
        return self.interfaces()["signals"]

    def _sample(self, name, r):
        t = float(r["t"])
        measured = "self-measured: the body's own reading at this time"
        if name == C.TIME_SIGNAL:
            return {"value": t, "unit": "s", "t": t, "source": measured}
        if name == C.COMMANDS_SIGNAL:
            return {"value": dict(self._counts), "unit": "commands applied, by action id", "t": t,
                    "source": "self-measured: counted by this adapter as it applied them"}
        if name == C.FAULTS_SIGNAL:
            return {"value": list(self._faults) + list(r.get("faults") or []), "unit": None, "t": t,
                    "source": measured}
        group, quantity = name.rsplit(".", 1)
        if group == self.arm:
            if quantity == "target":
                held = self._commanded(r)
                if held is not None:
                    return {"value": held, "unit": "rad or m", "t": t,
                            "source": "self-measured: the target the backend holds at this time"}
                return {"value": list(self._arm_target), "unit": "rad or m", "t": t,
                        "source": "description: the last setpoint this adapter sent; the backend reports none"}
            if quantity == "effort_cap":
                if r.get("torque_frac") is not None:
                    return {"value": float(r["torque_frac"]), "unit": "share of each joint's effort limit", "t": t,
                            "source": measured}
                return {"value": self._share, "unit": "share of each joint's effort limit", "t": t,
                        "source": "description: the last share this adapter sent; the backend reports none"}
            key, unit = {"position": ("q", "rad or m"), "speed": ("qd", "rad/s or m/s"),
                         "effort": ("tau", "N m or N"),
                         "effort_share": ("load", "share of each joint's effort limit, signed")}[quantity]
            return {"value": _list(r[key]), "unit": unit, "t": t, "source": measured}
        g = r.get("gripper") or {}
        if quantity == "opening":
            return {"value": float(g["open_frac"]), "unit": "share of the opening (0 closed, 1 open)", "t": t,
                    "source": measured}
        return {"value": _list(g["q" if quantity == "position" else "target"]), "unit": "rad or m", "t": t,
                "source": measured}

    def read(self, recipe):
        recipe = list(recipe)
        known = self._signals()
        unknown = [n for n in recipe if n not in known]
        if unknown:
            return C.refusal("unknown_signal", "the recipe names signals this body does not have; nothing of it "
                             "was read", signals=unknown)
        r = self.backend.read()
        if self.holder is not None and r.get("gripper") is None and any(n.startswith(self.holder + ".")
                                                                         for n in recipe):
            return C.refusal("no_reading", "the backend returned no reading of the declared holder")
        return {"samples": {n: self._sample(n, r) for n in recipe}}

    # ------------------------------------------------------------------ commanding
    def _finite(self, setpoint, n):
        return (isinstance(setpoint, (list, tuple)) and len(setpoint) == n
                and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                        for v in setpoint))

    def command(self, group, mode, setpoint, limits):
        limits = dict(limits or {})
        action = limits.get("action")
        if action is None:
            return C.refusal("no_action", "a command must carry the id of the admitted action it belongs to")
        if group not in (self.arm, self.holder) or group is None:
            return C.refusal("undeclared_group", f"this body declares no group {group!r} that can be commanded")
        if mode != "position":
            return C.refusal("mode", f"the interface of {group!r} does not list the mode {mode!r}")
        if group == self.arm:
            if not self._finite(setpoint, self._n):
                return C.refusal("setpoint", f"the setpoint is not {self._n} finite numbers")
            share = limits.get("effort_share", self._share)
            if not isinstance(share, (int, float)) or isinstance(share, bool) or not 0.0 <= float(share) <= 1.0:
                return C.refusal("limit", "the effort share is not a number from nothing to the whole limit")
            try:
                self.backend.command([float(v) for v in setpoint], torque_frac=float(share), gripper=None)
            except Exception as exc:                      # a body that cannot take the command refuses
                return C.refusal("backend", f"the backend did not take the command: {type(exc).__name__}")
            r = self.backend.read()
            asked = [float(v) for v in setpoint]
            held = self._commanded(r)
            if held is not None:
                applied = held
                clipped = applied != asked
                by = "the backend, to what it holds as the joint range" if clipped else None
            else:
                applied, clipped, by = asked, C.UNKNOWN, None
            self._arm_target, self._share = list(applied), float(share)
            answer = {"group": group, "mode": mode, "asked": list(setpoint), "applied": applied,
                      "clipped": clipped, "clipped_by": by, "limits": {"effort_share": float(share)},
                      "action": action, "t": float(r["t"])}
        else:
            if not self._finite(setpoint, 1):
                return C.refusal("setpoint", "the setpoint is not one finite number, the share of the opening")
            asked = float(setpoint[0])
            applied = min(1.0, max(0.0, asked))
            try:
                self.backend.command(list(self._arm_target), torque_frac=self._share, gripper=applied)
            except Exception as exc:
                return C.refusal("backend", f"the backend did not take the command: {type(exc).__name__}")
            r = self.backend.read()
            self._holder_share = applied
            answer = {"group": group, "mode": mode, "asked": list(setpoint), "applied": [applied],
                      "clipped": applied != asked,
                      "clipped_by": "the range of the opening" if applied != asked else None, "limits": {},
                      "action": action, "t": float(r["t"])}
        self._counts[action] = self._counts.get(action, 0) + 1
        return answer

    def capture(self, sensor):
        if sensor not in self.cameras:
            return C.refusal("undeclared_sensor", f"this body declares no sensor {sensor!r} that can be captured")
        cam = self.cameras[sensor]
        try:
            t = float(self.backend.read()["t"])
            frame = self.backend.render(cam)
            geometry = dict(self.backend.camera_info(cam))
        except Exception as exc:
            return C.refusal("backend", f"the backend returned no frame: {type(exc).__name__}")
        return {"sensor": sensor, "frame": frame, "t": t, "geometry": geometry,
                "pose": C.unknown("the backend states the lens only; the sensor's pose comes from the body model"),
                "exposure": C.unknown("the backend states none"), "noise": C.unknown("the backend states none")}

    def advance(self):
        before = float(self.backend.read()["t"])
        try:
            self.backend.step(1)
        except Exception as exc:
            fault = {"what": "the body did not complete the control period", "detail": type(exc).__name__}
            self._faults.append(fault)
            return {"periods": 0, "t": before, "faults": [fault]}
        r = self.backend.read()
        after = float(r["t"])
        if self._period_measured is None and after > before:
            self._period_measured = after - before
        out = {"periods": 1, "t": after}
        if r.get("faults"):
            out["faults"] = list(r["faults"])
        return out

    def halt(self, state):
        decl = C.safe_state(self.decl, state)
        if decl is None:
            return C.refusal("undeclared_state", f"this body declares no safe state {state!r}; nothing was changed")
        kind = decl.get("kind")
        if kind not in ("controlled stop with power kept", "power removed"):
            return C.refusal("cannot_honour", f"this adapter cannot bring the backend to a state of the kind "
                             f"{kind!r} inside one call; nothing was changed")
        r = self.backend.read()
        pose = _list(r["q"])
        share = self._share if kind == "controlled stop with power kept" else 0.0
        try:
            self.backend.command(list(pose), torque_frac=share, gripper=None)
        except Exception as exc:
            return C.refusal("backend", f"the backend did not take the stop: {type(exc).__name__}")
        after = self.backend.read()
        held = self._commanded(after)
        self._arm_target, self._share = (held if held is not None else list(pose)), float(share)
        names = [f"{self.arm}.{q}" for q in ARM_READ]
        back = {n: self._sample(n, after) for n in names}
        back[C.TIME_SIGNAL] = self._sample(C.TIME_SIGNAL, after)
        done = {self.arm: list(pose)}          # what was commanded: the pose measured at the stop
        faults = list(self._faults) + list(after.get("faults") or [])
        return {"state": state, "kind": kind,
                "did": ("the target was set to the measured pose" if kind == "controlled stop with power kept"
                        else "the target was set to the measured pose and the effort cap to nothing"),
                "applied": done, "measured_pose_used": pose, "read_back": back,
                "reached": checks.reads_as(decl, back, done), "faults": faults}
