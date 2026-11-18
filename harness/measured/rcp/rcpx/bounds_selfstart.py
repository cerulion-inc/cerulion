# SPDX-License-Identifier: AGPL-3.0-only
"""How a session starts on what the robot knows of itself (SPEC-R4 A2.1; round six the design rulings, contract C17).
The measuring itself is rcpx/bounds_selftest.py (run); the record is rcpx/bounds_self.py (SelfKnow).

before_move(ex) is the gate a session calls before its first operator move. The stored record of this body is the
normal start: it is loaded with no motion. With none, by cfg selfknow:
- "require" (a scored cell): a plain error that names the key and the path, and, where a record of another
  version stands there, both versions; the instrument is at fault, never the task.
- "measure" (the owner's calibration session): the self-test runs now (run) and writes the record.
- none stated (cold): NO self-test and no motion of its own. The session starts on the body's own description
  (every bound has a value from it until a move of this session shows better). One thing the description
  cannot give is read at once, standing still: the level and the noise of the force estimate at rest (the
  self-test's still hold, a few control periods inside the first move's call). Without it the guard tells a
  contact only at what the arm can push while the limit's own solve takes the level for nothing: a stated
  limit was passed 50 times over that way. cold(ex) says what is lacking and what a calibration costs in
  control periods; the bounds put it into the first result that carries bounds (key "selfknow") and say it in
  one line in the first result of any tool. A backend that reports hardware and states no mode is still
  refused selfknow_missing (the real arm's own gate; its hint is the one setting).
A session with "require" or with none never writes a record. preflight(ex) says, with no motion, which record a
session of this body would read: {"key", "path", "version", "measured_by"}, or None.
"""

import importlib
import math

import numpy as np

from rcpx import backend as backend_facts
from rcpx.bounds_ratios import DOUBLE, GIVE, REL
from rcpx.bounds_self import VERSION, SelfKnow, identity, key_of
from rcpx.context import refusal

HINT = 'set cfg selfknow to "measure" (the owner allows the robot to measure itself here), or give it a stored file'


HOLD = int(math.ceil(1 / (2 * REL ** 2)))   # numeric: the still hold's periods, the count whose noise is read
                                            # to the named ratio REL (the self-test's own count)


# ------------------------------------------------------------------ identity and the stored file
BODY = ("body_mass", "body_inertia", "body_pos", "body_quat", "body_ipos", "body_iquat", "body_parentid",
        "jnt_type", "jnt_pos", "jnt_axis", "jnt_range", "jnt_bodyid", "dof_armature", "dof_damping",
        "dof_frictionloss", "geom_type", "geom_size", "geom_pos", "geom_quat", "geom_bodyid",
        # what its taps measure rests on these too: how the body's shapes meet a thing, and what drives its joints
        "geom_solref", "geom_solimp", "geom_friction", "geom_condim", "geom_priority", "geom_solmix", "geom_margin",
        "geom_gap", "actuator_trntype", "actuator_trnid", "actuator_gaintype", "actuator_biastype",
        "actuator_gainprm", "actuator_biasprm", "actuator_gear", "actuator_ctrlrange", "actuator_forcerange")


def body_bytes(model):
    """The compiled body as bytes: its masses, inertias, frames, joints (type, place, axis, range, armature,
    damping, dry friction), shapes with their contact parameters, drives (kind, gain, bias, gear, ranges) and
    the physics step its fast readings come at, as the model's own compiler gives them. The same body read from
    two folders gives the same bytes; a file's bytes, name or path are no part of it."""
    mj = model.mj
    rows = [(name, np.asarray(getattr(mj, name), dtype=float).tolist()) for name in BODY if hasattr(mj, name)]
    rows.append(("opt.timestep", float(mj.opt.timestep)))
    return repr(rows).encode()


def identity_of(ex):
    """The robot's identity (SelfKnow key): the compiled body (body_bytes; never a scene's and never a file's
    bytes), the control law with gains and period, the gripper block, a stated payload, a serial or instance id,
    gravity in the base frame, firmware and supply voltage where the backend gives them."""
    spec, be = ex.model.spec, ex.backend
    law = backend_facts.servo_law_of(be) or {"kind": "joint_pd, assumed from the body model",
                                             "kp": spec["kp"], "kd": spec["kd"]}
    law = {k: v for k, v in law.items() if k in ("kind", "kp", "kd", "torque_limit", "speed_limit", "step_s")}
    law["dt"] = backend_facts.period_of(be)
    cfg = ex.cfg
    return identity(body_bytes(ex.model), law, gripper=spec.get("gripper"), payload=spec.get("payload"),
                    instance=getattr(be, "serial", None) or cfg.get("instance_id") or spec.get("serial"),
                    gravity_base=np.asarray(ex.model.mj.opt.gravity, dtype=float),
                    firmware=getattr(be, "firmware", None), voltage=getattr(be, "voltage", None))


def know_of(ex, directory=None):
    """(SelfKnow, loaded): the stored self-knowledge of this robot and controller (the newest record of this
    VERSION in the folder the session states, else bounds_self.default_dir()), or a new empty one."""
    ident = identity_of(ex)
    key = key_of(ident)
    directory = directory or ex.cfg.get("selfknow_dir")
    stored = SelfKnow.load(key, directory)
    if stored is not None:
        return stored, True
    return SelfKnow(key, directory, ident), False


def preflight(ex, directory=None):
    """With no motion: the record a session of this body would read: {"key", "path", "version", "measured_by",
    "saved_at", "records"}; None when none of this VERSION stands (bounds_self.SelfKnow.peek says what does)."""
    know, loaded = know_of(ex, directory)
    if not loaded or not know.ready:
        return None
    got = SelfKnow.peek(know.key, know.directory) or {}
    return {"key": know.key, "path": know.path, "version": VERSION, "measured_by": know.measured_by,
            "saved_at": know.stored, "records": got.get("records")}


def expected_s(ex, which=None):
    """About how long the sweep of a calibration takes (s of the robot's own time), from the body's own model:
    per joint the passes about double the last one's way out and back, each with a still wait."""
    model = ex.model
    dt = backend_facts.period_of(ex.backend)
    which = list(range(model.n)) if which is None else which
    n_hold = HOLD
    acc = [GIVE * float(model.motion["acc"][j]) for j in which]
    top = [model.speed_limit[j] if model.speed_limit[j] is not None else float(model.motion["qd"][j]) for j in which]
    return n_hold * dt + sum(DOUBLE * DOUBLE * (DOUBLE * w / a + float(model.motion["lag_s"][j]) / REL)
                             + DOUBLE * n_hold * dt for j, w, a in zip(which, top, acc))


LACKS = (
    "the hidden level and the noise of the force estimate while moving: one still hold reads them at rest; a "
    "free leg of this session adds its own",
    "the detection time: the guard's own readings stand in",
    "the tracked speed, the lag and the run-on of each joint: the body's servo model stands in",
    "braking and acceleration per joint: the torque limit over the inertia stands in",
    "the jaws' own numbers: the body's description of the jaw joint stands in",
    "this body's own stops: no strike is stated before a contact (not measured), and a stated limit is solved by "
    "the body's model of the stop slope, which no tap of this body has tested: not shown kept",
)


def cold(ex, know=None, still=None):
    """What a session with no stored record says of it (contract C17): {"stored": False, "key", "path": where the
    record would be read from, "lacks": what stands on the body's description instead of a measurement, "cost":
    words, "calibration": {"periods": about how many control periods one calibration takes, from the body's own
    model}, "still_hold": {"periods": what the one still hold of this session's first move costs (None before
    it)}, "other_version": the version of a record that stands there and is not read, or None}."""
    know = know or know_of(ex)[0]
    dt = backend_facts.period_of(ex.backend)
    periods = int(math.ceil(DOUBLE * expected_s(ex) / dt))     # the sweep, and about as much again for ramps and taps
    hold = HOLD if still is None else int(still)
    stands = SelfKnow.peek(know.key, know.directory)
    return {"stored": False, "key": know.key, "path": know.path, "lacks": list(LACKS),
            "calibration": {"periods": periods}, "still_hold": {"periods": hold},
            "other_version": None if stands is None else stands.get("version"),
            "cost": (f"this body has no stored self-knowledge: the session runs on the body's own description, and "
                     f"each bound says so. It costs this task {hold} control periods, once, inside its first move "
                     f"(the arm stands still and reads what its force estimate shows at rest). One calibration "
                     f"outside any task takes about {periods} control periods (cfg selfknow \"measure\") and is "
                     f"then read, not run again")}


def notice(ex, B):
    """For the first call of a session, with no motion: the line a cold session's first result carries whatever
    tool made it (None when a record stands, a mode is stated or the backend is hardware). It also sets
    ex._selfknow_cold, which the bounds put into the first result that carries bounds."""
    know = getattr(B, "know", None)
    if getattr(ex, "_selfknow_cold", None) is not None or getattr(know, "ready", False):
        return None
    if ex.cfg.get("selfknow") or bool(getattr(ex.backend, "is_hardware", False)) or not isinstance(know, SelfKnow):
        return None
    ex._selfknow_cold = cold(ex, know)
    hook = getattr(ex, "add_post", None)
    if callable(hook):      # the first result of ANY tool carries the dict too (a look, a whole-step put)
        hook(lambda name, kwargs, result: first_result(ex, result))
    return "selfknow: " + ex._selfknow_cold["cost"]


def first_result(ex, result):
    """A post hook on the operator's own calls: the first result of a cold session, whatever tool made it, carries
    "selfknow" (contract C17); later results are left as they are."""
    said = getattr(ex, "_selfknow_cold", None)
    if said is None or getattr(ex, "_selfknow_first_said", False) or not isinstance(result, dict):
        return None
    ex._selfknow_first_said = True
    return dict(result, selfknow=result.get("selfknow") or dict(said))


def resolved_share():
    """(share, text): the least share of each joint's torque limit the arm's own force estimate resolves (the
    observer's torque solve stops once a step is under it: rcpx.control_sense FIT_STEP_DONE). No hidden level
    (e_tau) is taken under it: a level at the arithmetic's own residue is no level, and a cold bench arm whose still
    hold read 4.4e-7 of its limits told two contacts in free space at 2.6e-5 N."""
    share = float(getattr(importlib.import_module("rcpx.control_sense"), "FIT_STEP_DONE", 0.0) or 0.0)
    return share, "the force estimate's own resolution, a share of each joint's limit (its torque solve's step)"


def e_tau_of(*shown):
    """e_tau: the largest share of the torque limits the torque model's own error showed (shown), never under
    what the force estimate itself resolves (resolved_share)."""
    return max([float(x) for x in shown if x is not None] + [resolved_share()[0]])


def _still(ex, know):
    """The cold session's one measurement: the self-test's still hold at the pose the arm stands in (no motion):
    the level and the noise of the force estimate and of each joint's torque at rest, the jitter and the
    resolution of what it reads, and what the torque model's own error hides there (e_tau, never under what the
    estimate resolves: e_tau_of). Returns the control periods it took. Nothing is stored and the record is not
    ready."""
    from rcpx import bounds_selftest       # here, not at the top: that module imports this one

    at = f"cold start at t {ex.sim_time():.2f} s"
    _, n = bounds_selftest._still_hold(ex, know, at)
    know.entries.pop("s1", None)           # a still hold is no calibration: nothing is ready, nothing is written
    know.set("e_tau", e_tau_of(getattr(ex, "_st_e_tau", 0.0)), "self_test", at + "; " + resolved_share()[1] + " at least")
    return n


def apply_to_model(ex, know):
    """The body model's own motion facts (stopping angles near limits) follow what the robot measured."""
    qd = know.get("qd_max")[0]
    if qd is not None and hasattr(ex.model, "set_motion"):
        ex.model.set_motion(qd=qd, lag_s=know.get("lag_s")[0])   # None entries keep the model's own values


def _install(ex, know):
    ex._selfknow = know
    apply_to_model(ex, know)
    try:   # the executor's Bounds reads it from here on, where that module is loaded
        from rcpx import bounds

        bounds.of(ex).know = know
    except Exception:
        pass
    return know


def before_move(ex, intent=None, directory=None):
    """None when the session may move (the record was loaded; or it has just been measured on the owner's word; or
    the session starts cold on the body's own description), else the refusal selfknow_missing (hardware with no
    mode stated). Raises RuntimeError when cfg selfknow is "require" and no record of this version stands. See
    the module text."""
    know = getattr(ex, "_selfknow", None)
    if know is not None and ((know.ready and not know.is_stale) or getattr(know, "cold", False)):
        return None                       # the record stands, or this cold session has taken its still hold
    know, loaded = know_of(ex, directory)
    mode = ex.cfg.get("selfknow")
    if loaded and know.ready and not know.is_stale:
        _install(ex, know)
        ex._journal("self_test", loaded=know.path, stored=know.stored, moved=False)
        return None
    if mode == "require":
        stands = SelfKnow.peek(know.key, know.directory)
        if stands is not None and stands.get("version") != VERSION:
            raise RuntimeError(f"cfg selfknow is require and the record of this robot at {stands['path']} is of version "
                               f"{stands.get('version')}, this code reads version {VERSION}: calibrate once outside "
                               f"the task (cfg selfknow \"measure\"); the new record is written beside the old one")
        raise RuntimeError(f"cfg selfknow is require and this robot has no stored self-knowledge at {know.path} "
                           f"(key {know.key}): measure it once outside the task (rcpx.bounds_selftest.run, or cfg "
                           f"selfknow \"measure\") and keep the record")
    if mode == "measure":
        from rcpx import bounds_selftest       # here, not at the top: that module imports this one
        bounds_selftest.run(ex, intent=intent, directory=directory)
        return None
    if bool(getattr(ex.backend, "is_hardware", False)):
        return refusal("selfknow_missing", "this robot has not measured itself (its tracked speed, its lag, its "
                       "stopping distance, the noise of its force estimate) and the backend reports hardware: it "
                       "moves by itself only on its owner's word", HINT)
    # cold: no self-test inside a task. The body's own description stands until a move shows better;
    # the level of the force estimate at rest is read once, standing still (see the module text)
    _install(ex, know)
    n = _still(ex, know)
    know.cold = True
    said = getattr(ex, "_selfknow_cold", None)
    ex._selfknow_cold = dict(said or cold(ex, know, still=n), still_hold={"periods": n})
    ex._journal("self_test", cold=True, moved=False, still_periods=n, key=know.key,
                calibration_periods=ex._selfknow_cold["calibration"]["periods"])
    return None
