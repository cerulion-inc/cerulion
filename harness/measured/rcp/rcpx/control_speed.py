# SPDX-License-Identifier: AGPL-3.0-only
"""The core's one door to the session's bounds (rcpx/bounds.py): every speed, force, torque, travel and time bound a
core move runs under is asked here, and nowhere in the core is one typed.

- A move's path is planned at the robot's own full speed (control_plan.py) and handed to B.plan with what kind of
  move it is; the plan that comes back is slowed only where the bounds say why (an expected thing's band, unseen
  space, a stated limit), and its items name each bound's source.
- The contact stop is armed on every move with the guard spec B.guard derives; there is no switch.
- A move that starts where the tool last bore on something tells its guard so (_bplan and control_tail
  .start_of): the load it stands with is the baseline, a move away from the thing is not ended by that thing and
  keeps the thing's stated limit until the contact has ended, a move into it is a press and is priced as one.
- What the operating model states (a lower speed, clear after a look, contact expected at the end) goes to the
  bounds as intent. The old arguments guarded, limit, need and a numeric force_n change no bound upward: a stricter
  one is honoured as lower-only intent with a note, a looser one is ignored with the note.
- Every guard stop is told to the bounds with its measured arrival speed and what the body could read of its
  peak; a stop the bounds call false stores nothing and the leg resumes. An expected contact (control_expect.py)
  is no surprise: it ends its leg "arrived".
- A move ends where it says (control_move_pace.py): "goal" with the hand inside its still band of the end, "short"
  with told_mm at the measured pose. What the hand was told to reach, where it is and how far behind it stands go
  into every result (told_mm, tip_mm, behind_mm); a call that acts where the hand stands waits for it first, by
  motion alone (_arrive).
- A motion that needs more control periods than the backend says are left is refused before it starts (_budget).
- Before the first motion of a session the robot's self-knowledge is loaded or measured by the modes of cfg selfknow.

Only the moving robot's model is read (its capsules, Jacobians, gains, masses); nothing names a robot.
"""

import math

import numpy as np

from rcpx import control_expect, control_move_pace, control_move_price, control_tail
from rcpx.context import refusal
from rcpx.control_base import MM, _mm, _ratio, _val
from rcpx.control_expect import MEETS, _attr, _hold_module, _unit  # noqa: F401  (MEETS: its home is control_expect)

FD_EPS = 1e-5        # numeric: rad, the finite-difference step of the point Jacobians
DOWN = np.array([0.0, 0.0, -1.0])
RETIRED = "guarded is retired: the contact stop is always armed"


class SpeedMixin:
    # ------------------------------------------------------------------ what the arm can run into that is known
    def _known_points(self, exempt=None):
        """Points (m, frame B) of things robot://world holds: stored contacts, located points and device entries
        with a point, stale ones too (a point on a part that moved still stands next to the rest of the thing).
        exempt: the id or label of the thing a call takes, works or holds (left out)."""
        pts = []
        for kind in ("contact", "located", "device"):
            for e in self.world.query(kind=kind, include_stale=True):
                if e.get("point_mm") is not None and (exempt is None or exempt not in (e.get("id"), e.get("label"))):
                    pts.append(np.asarray(e["point_mm"], dtype=float) / MM)
        return np.asarray(pts, dtype=float).reshape(-1, 3)

    def _robot_points(self, q):
        """(P, r): the moving robot's points at q (both ends of every moving capsule, then the tip) and their radii."""
        caps = self.model.collision_capsules(q)[1:]
        P = [p for _, p0, p1, _ in caps for p in (p0, p1)]
        r = [rr for _, _, _, rr in caps for _ in (0, 1)]
        P.append(self.model.fk_tip(q)[0])
        r.append(0.0)
        return np.asarray(P, dtype=float), np.asarray(r, dtype=float)

    def _point_kinds(self):
        """(tool flag, contact rate b in 1/s) of each point of _robot_points. b is what the backend states of its
        own contact model (Backend.contact_model); a backend that states none leaves it unresolved (inf: a reader
        that bounds a strike with it gets the careful side). Kept for readers that have not moved to B.slope."""
        names = [c[0] for c in self.model.collision_capsules(self.model.q_home)[1:]]
        tool = [n in self._tool_bodies for n in names for _ in (0, 1)] + [True]
        cm = getattr(self.backend, "contact_model", None)
        cm = cm() if callable(cm) else cm
        b = float(cm["b_per_s"]) if isinstance(cm, dict) and cm.get("b_per_s") is not None else math.inf
        return np.asarray(tool, dtype=bool), np.full(len(tool), b)

    # ------------------------------------------------------------------ the call's frame in the bounds
    def _call_begin(self, name):
        """Start of an operator's call: the bounds re-see what the fixed cameras show (A3)."""
        self._call_notes = []
        B = getattr(self, "bounds", None)
        if B is None:
            return
        try:
            self._call_changed = B.begin("call")
        except Exception as exc:   # a camera that cannot be read must not end the call
            self._journal("bounds_problem", where="begin", error=f"{type(exc).__name__}: {exc}")

    def _call_end(self, name):
        """End of an operator's call: an operator's word is taken back (it frees a path for one call only)."""
        B = getattr(self, "bounds", None)
        if B is None:
            return
        self.__dict__.pop("_guard_notes", None)   # the sentences a call's end collects, where nothing took them
        try:
            B.space.end_call()
        except Exception as exc:
            self._journal("bounds_problem", where="end_call", error=f"{type(exc).__name__}: {exc}")

    def _say_once(self, key, text):
        """A sentence a session's results carry once (None after the first time)."""
        said = self.__dict__.setdefault("_said", set())
        if key in said:
            return None
        said.add(key)
        return text

    # ------------------------------------------------------------------ self-knowledge before the first motion
    def _selfknow_gate(self, kind="transit"):
        """None, or the refusal selfknow_missing. Before the first motion of a session, when the bounds hold no
        measured self-knowledge, the self-test's own gate is asked (rcpx.bounds_selftest.before_move): the stored
        file of this robot and controller is loaded; with none a simulator measures now, hardware only when the
        owner set cfg selfknow "measure", and cfg selfknow "require" refuses (it is measured outside the task
        and kept). The self-test's own moves (kind explore) pass. A build with no self-test runs on the model."""
        state = self.__dict__.get("_selftest") or {}
        if kind == "explore" or state.get("running") or state.get("done"):
            return None
        know = getattr(self.bounds, "know", None)
        if know is not None and getattr(know, "ready", False) and not getattr(know, "is_stale", False):
            return None
        try:
            from rcpx import bounds_selftest
        except ImportError:
            self._selftest = {"done": True, "why": "no self-test in this build: bounds from the model"}
            return None
        self._selftest = {"running": True}
        out = None
        try:
            gate = getattr(bounds_selftest, "before_move", None)
            out = gate(self) if gate is not None else bounds_selftest.run(self)
        except RuntimeError as exc:   # cfg selfknow require and no stored file
            out = refusal("selfknow_missing", str(exc), "measure it once outside the task: open a session on this "
                          "robot without cfg selfknow require and make one move; the file is kept and reused")
        finally:
            self._selftest["running"] = False
        if isinstance(out, dict) and "refused" in out:
            return out
        self._selftest.update(done=True, out=out)
        return None

    # ------------------------------------------------------------------ what is held
    def _held_for(self, held=None):
        """The hold in force: the core's one record (control_hold.held). A caller adds only "extent"
        (what the held thing adds below the tip and its half width); it never states a mass."""
        rec = _hold_module().held(self)
        if rec is not None and isinstance(held, dict) and held.get("extent") is not None:
            rec = dict(rec, extent=held["extent"])
        return rec

    def _note_free_leg(self, plan, g, q0, path, held):
        """A leg that ran wholly through space the robot's own body had swept, ended at its goal and held nothing
        tells the self-knowledge what the force estimate read with nothing there (its hidden level at that speed
        and acceleration: upward at once, never downward inside a session)."""
        note = getattr(getattr(self.bounds, "know", None), "note_free_leg", None)
        segs = getattr(plan, "_segments", None)
        if note is None or held or not segs or len(path) < 2 or any(seg.get("key") != "free" for seg in segs):
            return
        tips = np.asarray([self.model.fk_tip(np.asarray(q, dtype=float))[0] for q in [q0] + list(path)])
        speed = np.linalg.norm(np.diff(tips, axis=0), axis=1) / self.dt
        accel = np.abs(np.diff(speed)) / self.dt
        try:
            note(float(speed.max()), float(accel.max()) if len(accel) else 0.0, float(g.felt.peak),
                 tau_abs_max=None if g.ext_peak is None else [float(x) for x in g.ext_peak], swept=True,
                 ended="goal", held=False)
        except Exception as exc:
            self._journal("bounds_problem", where="note_free_leg", error=f"{type(exc).__name__}: {exc}")

    # ------------------------------------------------------------------ intent
    def _intent(self, intent=None, *, speed=None, clear=None, expect=None, guarded=True, limit=None, need=None,
                force_n=None, kind="transit"):
        """(intent for the bounds, notes for the result). speed is a LOWER speed (m/s) unless the bounds take it
        as the operator's word; the retired arguments are handled as the module docstring says."""
        out, notes = dict(intent or {}), []
        if speed is not None:
            out.setdefault("speed", float(speed))
        if clear:
            out["clear"] = clear
        if expect:
            out["expect"] = expect
        if guarded is False:
            note = self._say_once("guarded", RETIRED)
            if note:
                notes.append(note)
        if limit:
            forces = [float(v) for k, v in dict(limit).items() if k.endswith("_n") and v is not None]
            if forces:
                out["_limit_n"] = min(forces)
        if need is not None:
            out["_readings"] = int(need)
        if force_n is not None and kind in MEETS:
            out.setdefault("force_n", float(force_n))
        elif force_n is not None:
            thr = _val(self.bounds.f_thr(0.0, 0.0))
            if float(force_n) > thr * (1.0 + _ratio("REL")):
                note = self._say_once("force_n", f"a stated guard of {float(force_n):.2f} N is looser than the "
                                      f"contact threshold this robot derives ({thr:.2f} N): ignored, the contact "
                                      "stop is always armed at the derived level")
                if note:
                    notes.append(note)
        return out, notes

    def _thing(self, target):
        """The bounds' record of a target (a world id, a label or a point); None for no target or one unknown."""
        if target is None or isinstance(target, (list, tuple)) and len(target) and not np.isscalar(target[0]):
            return None
        try:
            return self.bounds.thing(target)
        except Exception:
            return None

    # ------------------------------------------------------------------ the plan
    def _bplan(self, q0, path, kind, target=None, intent=None, held=None, away_from=None):
        """(plan, notes): the path as the bounds time it (B.plan). A caller's force limit that is stricter than the
        derived bound at the path's end goes in as max_n; a looser one is dropped, each with a note."""
        B, notes = self.bounds, []
        intent = dict(intent or {})
        limit_n = intent.pop("_limit_n", None)
        intent.pop("_readings", None)
        intent.pop("_share", None)
        if limit_n is not None and len(path):
            q_end = np.asarray(path[-1], dtype=float)
            q_was = np.asarray(path[-2] if len(path) > 1 else q0, dtype=float)
            u = _unit(self.model.fk_tip(q_end)[0] - self.model.fk_tip(q_was)[0])
            last = target[-1] if isinstance(target, list) and target else target
            item = B.f_allow(self._thing(last), q_end, u, intent=intent or None)
            derived = _val(item)
            if derived is None or limit_n < derived:
                intent["max_n"] = min(limit_n, float(intent.get("max_n", math.inf)))
                notes.append(f"your limit of {limit_n:.1f} N is stricter than the "
                             + (f"{derived:.1f} N this robot derives there" if derived is not None else
                                "derived bound") + ": used")
            else:
                note = self._say_once("limit", f"a stated limit of {limit_n:.1f} N is looser than the bound this "
                                      f"robot derives ({derived:.1f} N, {_attr(item, 'source', 'derived')}): "
                                      "ignored; a stated limit only lowers a bound")
                if note:
                    notes.append(note)
        # the contact this move starts in: the load the hand stands with is the guard's baseline; a move
        # that leaves the thing it stands on is not ended by that thing, and keeps the thing's stated limit (its
        # first stretch at the speed that keeps it) until the contact has ended; one that goes into it is a press.
        # A caller that states what it leaves (away_from: retreat, lift, release) has its own directional guard
        start = None
        if away_from is None and len(path):
            kinds = [kind] if isinstance(kind, str) else list(kind)
            start = control_tail.start_of(self, kind, q0, path, intent, held,
                                          meets=any(k in MEETS for k in kinds) or intent.get("expect") == "contact")
        leave = (start or {}).get("leave")
        if leave is not None and leave.get("limit_n") is not None:
            plan = control_tail.plan_leave(self, leave, q0, list(path), kind, target, intent, held)
        else:
            plan = B.plan(q0, list(path), kind, target=target, intent=intent or None, held=held, away_from=away_from)
        if start and _attr(plan, "refused") is None and _attr(plan, "guard") is not None:
            plan.guard.start = start
        return plan, notes

    def _retime(self, q0, path, **limit):
        """Kept for callers that have not moved to kinds: the path as the bounds time a transit, and no info."""
        intent, _ = self._intent(limit=limit or None)
        plan, _ = self._bplan(q0, path, "transit", intent=intent)
        return (list(_attr(plan, "path") or path), None)

    def _guard_spec(self, kind, v=0.0, a=0.0, held=None, away_from=None, readings=2):
        """The guard the bounds derive for a move of this kind (B.guard)."""
        return self.bounds.guard(kind, v=v, a=a, held=held, away_from=away_from, readings=int(readings))

    def _felt_levels(self):
        """What Felt compares with, read once per call: the force the estimate can hide at rest plus its noise,
        the tip's resolution and the servo's stiffness along the tool, the give share."""
        cache = getattr(self, "_felt_cache", None)
        if cache is not None and cache[0] == self._calls:
            return cache[1]
        B, s = self.bounds, self._last
        q = self.model.q_home if s is None else s.q
        u = _unit(self.model.fk_tip(q)[1][:, 0])
        lv = {"floor_n": _val(B.f_thr(0.0, 0.0)), "rigid_m": _val(B.rigid(q, u)), "k_n_m": _val(B.k_servo(q, u)),
              "give": _ratio("GIVE")}
        self._felt_cache = (self._calls, lv)
        return lv

    # ------------------------------------------------------------------ waiting
    def _still(self, s, need_m):
        """Motion alone: what is left of the tip's motion over one detection time is under need_m (m)."""
        v = float(np.linalg.norm(self.model.jac(s.q)[:3] @ s.qd))
        return v * _val(self.bounds.t_det()) <= float(need_m)

    def _quiet(self, s, need_m):
        """True when the force at rest, less what the hand holds, is under the contact level in force (the hold's
        own level while holding) and the motion still left over one detection time is under need_m, what the
        next step needs of the pose."""
        hold = _hold_module()
        if hold is not None:
            calm = hold.rest_force(self, s) <= hold.level(self)
        else:
            calm = s.Fmag <= _val(self.bounds.f_thr(0.0, 0.0))
        return calm and self._still(s, need_m)

    def _still_band(self, s):
        """The body's still band where the hand stands (m): the tip travel its own give explains under a force
        it cannot tell from none (its resolution, its jitter, the guard's level at rest over the servo's
        stiffness)."""
        return _val(self.bounds.rigid(s.q, _unit(s.R[:, 0])))

    def _band_at(self, q):
        """The still band at a joint pose (m), along the tool."""
        q = np.asarray(q, dtype=float)
        return _val(self.bounds.rigid(q, _unit(self.model.fk_tip(q)[1][:, 0])))

    def _arrive(self):
        """Before a call acts where the hand stands: hold the pose the hand was told to reach until the
        measured tip speed times the detection time is under the body's still band, under a guard derived for
        the hand's measured speed (the lag tail of a move that did not wait is guarded: control_tail.arrive). It
        ends without that, by motion alone, when the hand gets no stiller. Returns the control periods waited
        (the caller charges them to its call as arrive_periods); a stop met in the wait is eased, stored and
        left in self._tail."""
        return control_tail.arrive(self)

    def _steps_left(self):
        """The control periods the backend says are left (a harness budget), or None when it states none."""
        left = getattr(self.backend, "steps_left", None)
        left = left() if callable(left) else None
        return None if left is None else int(left)

    def _budget(self, periods, hint=None, price=None):
        """None, or the refusal budget: the plan as it will run at its planned speeds needs more control
        periods than are left. No margin and no share of what is left: a motion that cannot finish is not
        started. The hint is a shorter move. price (control_move_price.price_of): when a stated limit is what
        makes the plan long, the refusal names the word, its price, the smallest limit that fits and the ready
        calls."""
        left = self._steps_left()
        if left is None or int(periods) <= left:
            return None
        named = None if price is None else control_move_price.refusal_of(self, int(periods), left, hint, price)
        return named or refusal("budget", f"this move would take {int(periods)} control periods and {left} are left",
                                hint or "ask for a shorter move")

    def _shorter(self, q0, path):
        """The hint of a budget refusal: how much of this very path the periods left would cover."""
        left = self._steps_left()
        if left is None or not len(path):
            return None
        if left <= 0:
            return "0 control periods are left: nothing can move"
        p0 = self.model.fk_tip(np.asarray(q0, dtype=float))[0]
        part = float(np.linalg.norm(self.model.fk_tip(np.asarray(path[min(len(path), left) - 1]))[0] - p0))
        whole = float(np.linalg.norm(self.model.fk_tip(np.asarray(path[-1]))[0] - p0))
        return (f"ask for a shorter move: the first {part * MM:.0f} mm of these {whole * MM:.0f} mm run in the "
                f"{left} control periods left (a move that ends there brakes a little earlier)")

    def _wait_limit(self):
        """Periods a wait may last: the measured settling time with the call margin (never a count)."""
        B = self.bounds
        return max(1, int(math.ceil(_val(B.limit_s(_val(B.t_settle()))) / self.dt)))

    # ------------------------------------------------------------------ what a move tells the bounds
    def _stop_note(self, g, kind, target, held, at_rest, expect=None, travel=None):
        """Tell the bounds a guard stop and read what it was (control_expect.stop_note)."""
        return control_expect.stop_note(self, g, kind, target, held, at_rest, expect, travel)

    # ------------------------------------------------------------------ what a result reports
    def _first_statement(self):
        """The first statement of a session's first result that moved (A3)."""
        B = self.bounds
        s = self._last
        q = self.model.q_home if s is None else s.q
        R = self.model.fk_tip(q)[1]
        top = max(_val(B.v_free(q, R[:, i])) for i in range(R.shape[1]))
        try:
            share = B.space.share("free")
        except Exception:
            share = None
        shares = share if isinstance(share, dict) else None
        try:
            from rcpx import bounds_report
            return bounds_report.first_statement(top, shares, self.cfg["site"])
        except Exception as exc:
            self._journal("bounds_problem", where="first_statement", error=f"{type(exc).__name__}: {exc}")
        return f"no site limit is stated; the bounds allow the tip up to {top * MM:.0f} mm/s in free space"

    def _call_extra(self, intent, r):
        """Result keys beside the bounds: the false stops a leg resumed after, and the operator's word when one
        was given (it is recorded; it changes no speed and no guard)."""
        extra = {}
        if r.get("false_stops"):
            extra["false_stops"] = r["false_stops"]
        if (r.get("stop_info") or {}).get("ran") and r.get("stop") in ("contact", "cap", "arrived"):
            extra["_ran"] = r["stop_info"]["ran"]
            at = getattr(r.get("plan"), "_coordinates", None)      # where on the plan's path the stop was
            if at and r.get("streamed"):
                extra["_at_s"] = float(at[min(len(at), int(r["streamed"])) - 1])
        if (intent or {}).get("clear"):
            info = getattr(self.bounds, "_word_info", None) or {}
            extra["word"] = {"given": True, "look": info.get("look"), "note": info.get("text") or ""}
        return extra or None

    def _told_keys(self):
        """told_mm (the end the tool was told to reach: the commanded pose; after a stop it is where the arm was
        eased, the measured pose of the stop), tip_mm (measured now) and behind_mm (their distance). An operator
        that closes its own loop computes its next line from told_mm."""
        s = self._last if self._last is not None else self._sense()
        told = s.tip if self._q_des is None else self.model.fk_tip(self._q_des)[0]
        return {"told_mm": _mm(told), "tip_mm": _mm(s.tip),
                "behind_mm": round(float(np.linalg.norm(told - s.tip)) * MM, 2), **control_tail.tail_keys(self)}

    def _move_keys(self, plan, periods, notes=(), extra=None):
        """Result keys of a move: bounds (the binding bound and how many more, by the bounds' own report), periods
        (control periods used), told_mm, tip_mm and behind_mm, strike (the plan's price as the bounds made it,
        with what a contact stop showed under "ran") and predicted_peak_n, what the notes say, and the session's
        first statement once."""
        out = {"periods": int(periods), **self._told_keys()}
        extra = dict(extra or {})
        ran, at_s = extra.pop("_ran", None), extra.pop("_at_s", None)
        out.update(extra)
        strike = _attr(plan, "strike") if plan is not None else None     # the price of this motion
        if strike is not None or ran is not None:
            out["strike"] = dict(strike or {}, **({"ran": ran} if ran is not None else {}))
            out["predicted_peak_n"] = (strike or {}).get("peak_n")
        lim = _attr(plan, "limit") if plan is not None else None
        if isinstance(lim, dict) and lim.get("limit_n") is not None:      # kept or passed is said
            con = extra.get("contact") if isinstance(extra.get("contact"), dict) else {}
            out["limit"] = control_tail.limit_block(lim, ran, None if con.get("source", "tool") == "tool" else
                                                    ", ".join(con.get("likely_links") or []) or con.get("surface"), at_s)
        items = list(_attr(plan, "items") or []) if plan is not None else []
        try:
            out = self.bounds.report(out, items, detail=getattr(self, "_detail", "brief"))
        except Exception as exc:
            self._journal("bounds_problem", where="report", error=f"{type(exc).__name__}: {exc}")
        slowed = _attr(plan, "slowed") if plan is not None else None
        if slowed:
            out["slowed"] = int(slowed)   # periods the bounds held under the robot's own free speed
        out.update(control_tail.left_keys(self))   # a move that started on a contact: left it, or pressed
        out.update(control_move_pace.keys(self))   # the command held for the hand, or the move ended short
        if (out.get("pace") or {}).get("why"):
            out["why"] = out["pace"]["why"]
        notes = [n for n in notes if n]
        if notes:
            out["bounds_notes"] = notes
        if periods and self._say_once("first_move", True):
            out["first_move"] = self._first_statement()
        return out

    # ------------------------------------------------------------------ several legs as one stream (A8.2)
    def _camera_latency(self, camera):
        """The camera's measured latency (s) from the robot's self-knowledge; None when it is not known."""
        cam = getattr(getattr(self.bounds, "know", None), "camera", None)
        cam = cam() if callable(cam) else cam
        if isinstance(cam, dict) and isinstance(cam.get(camera), dict):
            cam = cam[camera]
        return _val(cam.get("latency_s")) if isinstance(cam, dict) and cam.get("latency_s") is not None else None

    def _stream_look(self, look, s):
        """One look of a stream, taken while the hand moves, through the look primitive's own record path (the
        wrist camera; another camera through a package's hook ex._look_camera(camera, note)). The record says how
        fast the camera moved; with a measured latency also how far it travelled in it, else that the latency is
        not measured (a reader then widens the look's error by that speed times one control period)."""
        camera = look.get("camera") or "wrist"
        note = look.get("note") or f"stream look, leg {look.get('leg')}"
        if camera == "wrist":
            rec = self._look_impl(note=note)
        elif getattr(self, "_look_camera", None) is not None:
            rec = self._look_camera(camera, note)
        else:
            return {"camera": camera, "leg": look.get("leg"), "refused": "no record path for this camera"}
        rec = dict(rec) if isinstance(rec, dict) else {"value": rec}
        v = float(np.linalg.norm(self.model.jac(s.q)[:3] @ s.qd))
        latency = self._camera_latency(camera)
        rec.update(camera=camera, leg=look.get("leg"), in_motion=True, camera_speed_mm_s=round(v * MM, 1),
                   camera_travel_mm=None if latency is None else round(v * float(latency) * MM, 2))
        if latency is None:
            rec["latency"] = "not measured: the look was read in motion and its pose is that much less sure"
        return rec

    def _stream(self, legs, *, held=None, looks=None, need_m=None, intent=None):
        """Several legs as ONE planned path (B.plan_legs): one ramp up, one ramp down, no stop and no wait between
        legs; a corner is rounded inside the space the book shows free, else taken at the speed the joints can
        turn it. leg = {"kind", "to": a pose (4 x 4) or a joint vector, "line": True for a straight tip line,
        "target", "away_from", "intent", "expect": an expected contact (control_expect.py)}. looks = [{"leg",
        "s", "camera"}] are taken in motion at their stations: a look never stops a stream. need_m: what the
        caller needs of the end pose (the end is waited for only then). The guard is derived again at every leg
        boundary; a stop the bounds call false resumes the stream; the budget is counted for every piece.
        Returns {stopped: goal, arrived, contact, cap, refused or a stop request's text; leg; ended: per leg;
        arrived; periods: per leg; moving; waiting; looks: [records]; bounds; told_mm, tip_mm, behind_mm, ...}; a
        guard stop names the leg. No leg falls back to another primitive."""
        return control_expect.stream(self, legs, held=held, looks=looks, need_m=need_m, intent=intent)

    # ------------------------------------------------------------------ the jaws' own numbers
    def _jaw_know(self):
        """What the robot knows of its own jaws (the numbers of its stored jaw entry: speed, lag, lag_spread,
        closed_gap, ... each a share of the span or of the servo's force), from its self-knowledge; {} before."""
        know = getattr(self.bounds, "know", None)
        jaw = getattr(know, "jaw", None)
        jaw = jaw() if callable(jaw) else jaw
        if isinstance(jaw, (tuple, list)) and jaw and isinstance(jaw[0], dict):
            jaw = jaw[0]                            # the stored entry is (value, source, at): the value
        out = {}
        for k, v in (jaw.items() if isinstance(jaw, dict) else ()):
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                out[k] = float(v)
            elif isinstance(v, (list, tuple)) and v and all(isinstance(x, (int, float)) for x in v):
                out[k] = [float(x) for x in v]      # a fitted line (the free lag: a + b x rate), the rates run
        return out

    def _jaw_rate(self):
        """The jaws' own speed as a share of their span per second: measured (one timed open and close), else the
        model's: the fastest the jaw servo's force can still stop inside the span, sqrt(2 a / span)."""
        k = self._jaw_know()
        if k.get("speed"):
            return float(k["speed"])
        g = self.model.spec["gripper"]
        m = self.model.mj
        span = np.maximum(np.abs(np.asarray(g["open"], dtype=float) - np.asarray(g["closed"], dtype=float)),
                          np.finfo(float).tiny)
        force = np.asarray(g.get("forcerange") or [0.0], dtype=float)
        dofs = [int(m.jnt_dofadr[m.joint(j).id]) for j in g["joints"]]
        mass = np.array([float(m.dof_M0[d]) for d in dofs])
        acc = force / np.maximum(mass, np.finfo(float).tiny)
        return float(np.min(np.sqrt(2.0 * acc / span)))   # numeric: v^2 = 2 a d

    def _jaw_lag(self):
        """How far behind a closing or opening command a FREE jaw runs (a share of the span): its measured free lag
        plus its spread, else the model's: what the jaw travels in one detection time at its own speed."""
        k = self._jaw_know()
        lag = k.get("lag")
        if isinstance(lag, list):                  # the free lag as a line in the rate: a + b x rate
            lag = lag[0] + (lag[1] if len(lag) > 1 else 0.0) * self._jaw_rate()
        if lag is not None:
            return float(lag) + _ratio("TWO_READ") * float(k.get("lag_spread") or 0.0)
        return min(1.0, self._jaw_rate() * _val(self.bounds.t_det()))

    def _jaw_share(self, hold=True):
        """The least squeeze that stalls the jaws (a share of the jaw servo's force): their squeeze reading's hidden
        level plus its noise, from the robot's self-knowledge; 0 before it is measured. A close whose caller states
        no force keeps the squeeze at which the jaws were seen to stall (MotionMixin.gripper); with hold false the
        jaws rest on the part at this least squeeze."""
        k = self._jaw_know()
        least = float(k.get("squeeze_hidden") or 0.0) + _ratio("TWO_READ") * float(k.get("squeeze_noise") or 0.0)
        return min(1.0, least)
