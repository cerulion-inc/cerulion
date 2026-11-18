# SPDX-License-Identifier: AGPL-3.0-only
"""The stages of push and lift (SPEC-R3 4.3, SPEC-R4 derived bounds).

Line selection is pure over stored geometry. Run owns one call's gates, budget,
local free hold, approach options, confirmed touches, face fit and release.
Every nested motion passes ex.gates and reads light again; one unexpected strike
ends the approach and backs off rather than trying another surface. The final
stretch among nearby parts enters at the core touch's derived arrival speed.

A triangulated point's rays' miss alone is not its depth uncertainty. The marks
are read again and the larger error counts; L.thing_of carries that error
without making target_info write to the world. The one-reading magnitude guard
uses the hidden level plus ONE_READ noise: old review probes showed magnitude
noise stops too often at a two-reading threshold. The free hold samples the
count computed from REL and returns RMS and the hidden envelope separately.

A contact that does not repeat is not a surface. Confirm backoff and tolerance
come from shown travel, tool width, contact errors and servo give. Fitting a
face at the whole part's size lands off its edge: fit offsets instead follow
point error and tool width, with enough setback to approach the sloping face.
Only points on one plane within their own contact residual count.

Time follows the stated budget and the plan's bound, including the derived
settling time. Old review runs showed that looks, fitting and a new approach
can consume a whole step; no fixed short stage allowance survives. Run.ramp
lives in act_ramp so these rules and their reasons fit without deleting text.
No scene or scoring truth is read here.
"""
import math
import numpy as np
from rcpx import r3_limits as L
from rcpx import side_words
MM = 1000.0
FIT_LINE_FRAC = 1e-3  # Numeric conditioning of the plane solver.
FIT_TOUCHES = 3  # Three surrounding contacts, plus the centre, judge a plane.
MOTIONS = ("approach", "touch", "press", "move_tip")   # the nested calls that move toward the world


class Stop(Exception):
    """A stage ended the call: stopped is the result's stop word, fields fill its said."""
    def __init__(self, stopped, **fields):
        """Initialize this stage record from its caller."""
        super().__init__(stopped)
        self.stopped, self.fields = stopped, fields


def _v(x):
    """A flat float array."""
    return np.asarray(x, dtype=float).reshape(-1)


def _unit(v):
    """A unit direction, or None for a zero vector."""
    v = _v(v)
    n = float(np.linalg.norm(v))
    return None if n == 0 else v / n


def _r(v, nd=4):
    """A plain rounded list without negative zero."""
    return [round(float(x), nd) + 0.0 for x in v]


def line(way, normal, centre_mm, top_mm, p_mm, ray, B=None, sigma_mm=0):
    """
    (u, source) by the first rule that applies (SPEC-R3 4.3 P2), or (None, deg up) when u points up more
    than
    WAY_UP_Z: refused way_up. source: way | normal | centre | naming_ray | radial.
    """
    if way == "down":
        u, src = np.array([0.0, 0.0, -1.0]), "way"
    elif normal is not None:
        u, src = -_unit(normal), "normal"
    elif centre_mm is not None and p_mm is not None:
        P = _v(p_mm)
        dxy = P[:2] - _v(centre_mm)[:2]
        dist = float(np.linalg.norm(dxy))
        if top_mm is not None and P[2] >= float(top_mm) - L.line_top_mm(B, sigma_mm=sigma_mm):
            u, src = np.array([0.0, 0.0, -1.0]), "centre"
        elif dist >= L.line_centre_min_mm(B, sigma_mm=sigma_mm):
            u, src = np.array([-dxy[0] / dist, -dxy[1] / dist, 0.0]), "centre"
        else:
            u = level_line(P, ray)
            src = "radial" if ray is None or float(np.linalg.norm(_v(ray)[:2])) < L.RAY_H_MIN * float(
                np.linalg.norm(_v(ray))) else "naming_ray"
    elif ray is not None:
        u, src = _unit(ray), "naming_ray"
    else:
        u, src = (side_words.radial(p_mm) if p_mm is not None else np.array([1.0, 0.0, 0.0])), "radial"
    if float(u[2]) > L.WAY_UP_Z:
        return None, round(math.degrees(math.asin(min(1.0, float(u[2])))), 1)
    return u, src


def level_line(p_mm, ray):
    """
    P2 rule 3's line for a point with no face of its own: the naming ray's level part when it is
    RAY_H_MIN of
    its length or more, else away from the robot (+r).
    """
    if ray is not None:
        r = _v(ray)
        h = np.array([r[0], r[1], 0.0])
        if float(np.linalg.norm(h)) >= L.RAY_H_MIN * float(np.linalg.norm(r)):
            return h / float(np.linalg.norm(h))
    return side_words.radial(p_mm)


def across(u):
    """Two unit directions across u: e1 level (z x u, or x when u is vertical), e2 = u x e1."""
    u = _unit(u)
    e1 = np.cross([0.0, 0.0, 1.0], u)
    if float(np.linalg.norm(e1)) == 0:
        e1 = np.array([1.0, 0.0, 0.0]) - float(u[0]) * u
    e1 = _unit(e1)
    return e1, _unit(np.cross(u, e1))


def _plane(pts, resid_mm):
    """The least-squares plane and RMS residual, rejecting a line."""
    c = pts.mean(axis=0)
    _, sv, vt = np.linalg.svd(pts - c)
    if sv[1] < FIT_LINE_FRAC * sv[0] or sv[1] <= resid_mm:     # the points lie on one line
        return None, None
    n = vt[2]
    return n, float(np.sqrt(np.mean(((pts - c) @ n) ** 2)))


def fit_plane(points, resid_mm):
    """The unit plane normal from at least three contacts in mm; reject a line or a residual beyond contact error, allowing a leave-one-out fit with enough points."""
    pts = np.asarray(points, dtype=float)
    if len(pts) < FIT_TOUCHES:
        return None
    n, res = _plane(pts, resid_mm)
    if n is not None and res <= resid_mm:
        return _unit(n)
    if len(pts) > FIT_TOUCHES + 1:          # a subset of 4 or more still has a residual to judge (3 points always fit)
        best = None
        for i in range(len(pts)):
            m, r = _plane(np.delete(pts, i, axis=0), resid_mm)
            if m is not None and r <= resid_mm and (best is None or r < best[1]):
                best = (m, r)
        if best is not None:
            return _unit(best[0])
    return None


def largest_size(part):
    """The largest stated part size, or None."""
    sz = (part or {}).get("size_mm") or {}
    vals = [float(v) for v in sz.values() if isinstance(v, (int, float))]
    return max(vals) if vals else None


class Run:
    """One call's stages, local force readings, gates, plan-derived bounds and runtime phase records."""
    def __init__(self, ex, tool):
        """Initialize this stage record from its caller."""
        self.ex, self.tool = ex, tool
        self.B = L.bounds_of(ex)
        st = (getattr(ex, "state", None) or {}).get("act") or {}
        self.local = None if st.get("force_hidden_n") is None else (st["force_hidden_n"], st.get("force_noise_n") or 0)
        self.detail = "full"
        self.items, self.ramps, self.t0 = [], [], ex.sim_time()
        self.legs, self.stage, self.notes, self.touches = [], "preflight", [], []
        self.peak_total, self.moved, self.touch_bound_n = 0.0, False, 0.0
    def elapsed(self):
        """World seconds used by this call."""
        return self.ex.sim_time() - self.t0
    def check_time(self, stage):
        """Stop before a stage when the stated act budget is exhausted."""
        budget = (getattr(self.ex, "context", None) or {}).get("act_budget_s")
        exhausted = budget() <= 0 if callable(budget) else (
            isinstance(budget, (int, float)) and self.elapsed() >= budget)
        if exhausted:
            raise Stop("time", stage=stage, s=round(self.elapsed(), 1))
    def gate(self, stage, name, kwargs):
        """Check the budget, motion gates and current light before a nested call."""
        self.check_time(stage)
        # Re-read the light immediately before a nested motion. An old
        # frame once let a press continue after the light went dark.
        self.stage = stage
        r = self.ex.gates(name, kwargs)
        if r is not None:
            bad = r.get("refused") or {}
            raise Stop("dark" if bad.get("check") == "dark" else "refused", stage=stage,
                       check=bad.get("check"), refused=bad)
        if name in MOTIONS:
            try:
                self.ex._read_light(f"inside {self.tool}")
                dark = self.ex._dark_refusal_now()
            except (AttributeError, NotImplementedError):
                dark = None
            if dark is not None:
                self.ex._journal("gate", name=name, by="runtime", refused=dark.get("refused"))
                raise Stop("dark", stage=stage, check="dark", refused=dark.get("refused"))
    def call(self, stage, name, **kwargs):
        """
        ex.gates(name, kwargs), then the nested primitive; the result. A refusal of the call itself is
        returned
        (the caller decides); a gate refusal raises Stop.
        """
        self.gate(stage, name, kwargs)
        t1 = self.ex.sim_time()
        from rcpx.r2 import act_guard
        out, activity = act_guard.motion_call(self.ex, self.B, name, kwargs)
        t2 = self.ex.sim_time()
        if t2 > t1:
            self.moved = True
        if isinstance(out, dict):
            for k in ("peak_force_total_n", "peak_force_n", "force_n_end", "force_n"):   # a touch's own reading too
                if isinstance(out.get(k), (int, float)):
                    self.peak_total = max(self.peak_total, float(out[k]))
        rec = {"by": "runtime", "tool": self.tool, "stage": stage, "primitive": name, "t0": round(t1, 3),
               "t1": round(t2, 3), "phase": stage, "did": name, **activity,
               "bounds": self.items[0] if self.items else None}
        self.legs.append(rec)
        self.ex.note_event("r3_leg", **rec)
        return out
    def move(self, stage, dv_mm):
        """Move the tip by a base-frame displacement through its core primitive."""
        dv = _v(dv_mm)
        kw = {"dx": round(float(dv[0]), 3), "dy": round(float(dv[1]), 3), "dz": round(float(dv[2]), 3),
              "frame": "base"}
        return self.call(stage, "move_tip", **kw)
    def tip(self):
        """The sensed tip in millimetres."""
        return _v(self.ex._sense().tip) * MM
    def surfaces(self):
        """The tip and the tool surfaces the model serves."""
        names = [s.get("name") for s in (self.ex.model.spec.get("tool_surfaces") or []) if s.get("name")]
        return ["tip"] + [n for n in names if n != "tip"]
    def approach_line(self, target_mm, u, standoff, unc, P=None, source="triangulated", tip_only=False):
        """Try the tip and served tool surfaces along the line; check hand clearance and enter the last stretch by touch. One strike ends all options."""
        reasons = []
        lead = self.tool_reach_mm()
        target = _r(target_mm, 2)
        entry = self.ex.world.get(self.target_id) if hasattr(self, "target_id") else None
        if entry and entry.get("point_mm") is not None and np.array_equal(_v(entry["point_mm"]), _v(target_mm)):
            target = entry["id"]
        for surf in (["tip"] if tip_only else self.surfaces()):
            far = float(standoff) + lead
            r = self.call("approach", "approach", target=target, along=_r(u), surface=surf,
                          standoff_mm=round(far, 1), keep_in_view=False, point_source=source)
            if isinstance(r, dict) and "refused" in r and lead > 0:
                far = float(standoff)
                r = self.call("approach", "approach", target=target, along=_r(u), surface=surf,
                              standoff_mm=round(far, 1), keep_in_view=False, point_source=source)
            if not isinstance(r, dict) or "refused" in r:
                reasons.append(f"{surf}: {((r or {}).get('refused') or {}).get('why', 'refused')}")
                continue
            if r.get("stopped") in ("contact", "cap"):
                self.struck(r, "the approach")       # one strike ends the call: no other surface is tried
            if r.get("stopped") not in ("goal",):
                reasons.append(f"{surf}: the arm stopped on the way ({r.get('stopped')})")
                continue
            near = self.hand_lands_first(P if P is not None else _v(target_mm), u, far)
            if near is not None:
                reasons.append(f"{surf}: the {near} would land on the part before the finger (hand_lands_first)")
                continue
            # The hand behind the tool enters among nearby parts here.
            # It meets them at the touch's arrival law, rather than transit speed.
            if far > float(standoff) + L.rigid_mm(self.B, self.ex._sense().q, u):
                left, mv = far - float(standoff), None
                st = self.ex.state.setdefault("act", {}) if isinstance(getattr(self.ex, "state", None), dict) else {}
                nz = self.noise(u)           # the force noise, measured here in free space before the tool goes in
                if nz is not None:
                    st["force_noise_n"] = round(nz, 3)
                f_in = round(L.f_touch(self.B, readings=1, local=self.local), 2)
                stalls = 0
                while left > L.rigid_mm(self.B, self.ex._sense().q, u):
                    mv = self.touch("enter", u, left, f_in)
                    if not isinstance(mv, dict) or "refused" in mv:
                        break
                    if mv.get("stopped") == "cap":
                        self.struck(mv, "the last stretch before the part")
                    if not mv.get("contact"):
                        gone = float(mv.get("travelled_mm") or 0.0)
                        if gone <= L.rigid_mm(self.B, self.ex._sense().q, u):
                            break
                        left -= gone
                        continue
                    c1 = _v(mv["point_mm"]) if mv.get("point_mm") is not None else self.tip()
                    self.move("enter", c1 - _v(u) * L.confirm_back_mm(self.B, self.ex._sense().q, u) - self.tip())
                    # Joint friction can look like a contact once. A strike
                    # is reported only when a second touch meets it again.
                    again = self.touch("enter", u, L.confirm_max_mm(self.B, self.ex._sense().q, u, v_mm_s=float(mv.get("speed_mm_s") or 0), force_n=f_in), f_in)
                    if isinstance(again, dict) and again.get("contact"):
                        self.struck(again if again.get("point_mm") is not None else mv,
                                    "the last stretch before the part")
                    stalls += 1
                    if mv.get("id"):
                        self.ex.world.mark_stale([mv["id"]], "a stall that did not repeat: no surface here",
                                                 source="act")
                    self.notes.append("a stall on the way in did not repeat")
                    if stalls > L.CONFIRM_TRIES:
                        raise Stop("cannot_reach", why=f"the arm stalled {stalls} times on the way in with nothing "
                                                       "there")
                    left = float((_v(target_mm) - _v(u) * float(standoff) - self.tip()) @ _v(u))
                if not isinstance(mv, dict) or "refused" in mv:
                    reasons.append(f"{surf}: {((mv or {}).get('refused') or {}).get('why', 'refused')}")
                    continue
            if reasons:
                self.notes.append(f"approached with the {surf} ({len(reasons)} option(s) before it did not work: "
                                  f"{reasons[0][:90]})")
            return {"surface": surf, "result": r}
        raise Stop("cannot_reach", why=reasons[0] if reasons else "no tool surface faces the line",
                   reasons=reasons)
    def tool_reach_mm(self):
        """The tool bodies' furthest reach from the tip in the loaded collision model, in mm."""
        from rcpx.r2 import act_guard
        return act_guard.tool_reach_mm(self.ex)
    def struck(self, r, where):
        """
        The arm met something it did not mean to touch: one step back (the way it came), then Stop struck
        with
        the force felt and what located point the contact was near. Never tried again with another
        surface.
        """
        c = r.get("contact") if isinstance(r.get("contact"), dict) else r      # a touch result is the contact
        n = max(float(c.get("force_n") or 0.0), float(r.get("force_n_end") or 0.0), float(r.get("peak_n") or 0.0))
        near = None
        try:
            from rcpx.r2 import act_guard
            pt = c.get("point_mm") or list(self.tip())
            near = act_guard.nearest_placed(self.ex, pt)
            bad = act_guard.forbidden_at(self.ex, pt)
        except Exception:
            bad = None
        try:
            self.ex.retreat(steps=1)
        except Exception:
            pass
        links = ", ".join(c.get("likely_links") or []) if c.get("source") not in (None, "tool") else "the tool"
        raise Stop("struck", n=round(n, 1), where=where, by=links or "the arm",
                   near=(f"{near[2]:g} mm from the {near[0]} ({near[1]})" if near else "near no located point"),
                   forbidden=(f"; that is within the keep-off of the {bad[0]}, a part robot://devices forbids"
                              if bad else ""))
    def hand_lands_first(self, P, u, standoff):
        """The non-tool body that would precede the finger at this touch target, using the model and its derived clearance."""
        from rcpx.r2 import act_guard
        return act_guard.hand_lands_first(self.ex, self.B, P, u, standoff,
                                          L.thing_of(self.B, getattr(self, "target_id", None),
                                                     sigma_mm=getattr(self, "target_sigma_mm", None)))
    def noise(self, u):
        """Sample the REL-derived count of free still readings; return RMS along u and retain the hidden whole-force envelope. Skip near a stored point."""
        ex, B = self.ex, self.B
        tip = self.tip()
        for e in ex.world.query(include_stale=False):
            if e.get("point_mm") is not None and e["kind"] in ("contact", "located", "device"):
                d = e.get("data") or {}
                err = d.get("uncertain_mm", d.get("error_mm")) or 0
                if float(np.linalg.norm(_v(e["point_mm"]) - tip)) <= L.noise_clear_mm(B, sigma_mm=err):
                    return None
        self.check_time("noise")
        t1, vals, hidden = ex.sim_time(), [], 0
        for _ in range(L.noise_samples(B)):
            self.check_time("noise")
            ex.idle(B.dt())
            s = ex._sense()
            vals.append(float(_v(s.F) @ _v(u)))
            hidden = max(hidden, float(s.Fmag))
        sigma = float(np.sqrt(np.mean(np.square(vals)))) if vals else 0
        self.local = (hidden, sigma)
        st = ex.state.setdefault("act", {})
        st.update(force_noise_n=sigma, force_hidden_n=hidden)
        rec = {"by": "runtime", "tool": self.tool, "stage": "noise", "primitive": "idle",
               "t0": round(t1, 3), "t1": round(ex.sim_time(), 3), "periods": len(vals), "phase": "noise", "did": "idle",
               "moving": 0, "waiting": len(vals), "jaw": 0, "seconds": ex.sim_time() - t1,
               "free_share": None, "counts_source": "free hold readings counted directly",
               "bounds": self.items[0] if self.items else None}
        self.legs.append(rec)
        ex.note_event("r3_leg", **rec)
        return sigma
    def touch(self, stage, u, max_mm, f):
        """Call the core touch and retain its contact and reported peak bound. f is the derived one-reading
        threshold; where it is 0 (a robot that has measured no hidden level and no noise yet) the call states no
        force and the core's own threshold stands: a threshold of 0 would stop on nothing."""
        kw = {"direction": _r(u), "max_mm": float(max_mm), "frame": "base"}
        if f is not None and round(float(f), 2) > 0:
            kw["force_n"] = round(float(f), 2)
        r = self.call(stage, "touch", **kw)
        if isinstance(r, dict) and isinstance(r.get("speed_mm_s"), (int, float)):
            self.touch_v_mm_s = float(r["speed_mm_s"])      # the speed the part was found at: its contact's own error
        if isinstance(r, dict) and isinstance(r.get("peak_bound_n"), (int, float)):
            self.touch_bound_n = max(self.touch_bound_n, r["peak_bound_n"])
        self.touches.append({"stage": stage, "contact": bool((r or {}).get("contact")), "stopped": (r or {}).get(
            "stopped"), "point_mm": (r or {}).get("point_mm"), "force_n": (r or {}).get("force_n")})
        return r
    def touch_confirm(self, u, max_mm, f, P, unc, label):
        """
        P4: touch along u up to max_mm at f; back off the shown travel plus tool half width and touch again
        (the backoff plus both contact errors and servo give): a
        contact within tol of the first confirms
        it; otherwise the first was a stall, it is marked stale (never
        kept as a contact), and the touch
        goes on from there (at most CONFIRM_TRIES stalls). Returns the confirmed
        touch result. Stop
        not_found, blocked or cap.
        """
        left, stalls = float(max_mm), 0
        start = self.tip()
        while True:
            r = self.touch("touch", u, max(0, left), f)
            if not isinstance(r, dict) or "refused" in r:
                raise Stop("refused", stage="touch", check=((r or {}).get("refused") or {}).get("check"))
            if r.get("stopped") == "cap":
                raise Stop("cap", n=round(float(r.get("force_n") or self.peak_total), 1))
            moved_part = bool(r.get("contact") and r.get("stopped") == "moved_part")
            if not r.get("contact") and r.get("stopped") in ("time", "goal") \
                    and left - float(r.get("travelled_mm") or 0) > L.rigid_mm(self.B, self.ex._sense().q, u):
                if float(r.get("travelled_mm") or 0) <= L.rigid_mm(self.B, self.ex._sense().q, u):
                    raise Stop("not_found", mm=round(float((self.tip() - start) @ _v(u)), 0))
                left -= float(r.get("travelled_mm") or 0.0)     # a long line: the next piece (TOUCH_PIECE_S)
                continue
            if not r.get("contact"):
                gone = float((self.tip() - start) @ _v(u))
                raise Stop("not_found", mm=round(max(gone, float(max_mm)), 0) if stalls == 0 else round(gone, 0),
                           extra=None if stalls == 0 else "felt a stall that did not repeat")
            if r.get("source") not in (None, "tool") or r.get("at_tip") is False:
                what = ", ".join(r.get("likely_links") or []) or f"the {r.get('surface') or 'side of the tool'}"
                raise Stop("blocked", what=what, mm=round(self._before(P, r, u), 0), **self._met(r))
            q = self.ex._sense().q
            v = float(r.get("speed_mm_s") or 0)
            tol = L.confirm_tol_mm(self.B, q, u, v_mm_s=v, force_n=f)
            contact = self.B.thing(r["id"]) if r.get("id") else None
            if not L.need_confirm(self.B, q, u, contact=contact):
                if P is not None and self._before(P, r, u) > L.blocked_mm(self.B, q, u, sigma_mm=unc, v_mm_s=v):
                    raise Stop("blocked", what="a surface", mm=round(self._before(P, r, u), 0), **self._met(r))
                return r
            # Back off from where resistance began, even if a give or an
            # edge slide carried the touch beyond that point before it ended.
            c1 = _v(r["point_mm"])
            back = self.move("confirm", c1 - _v(u) * L.confirm_back_mm(self.B, self.ex._sense().q, u) - self.tip())
            if isinstance(back, dict) and back.get("stopped") not in ("goal", None) and "refused" not in back:
                self.notes.append(f"the back-off before the confirm stopped {back.get('stopped')}")
            r2 = self.touch("confirm", u, L.confirm_max_mm(self.B, q, u, v_mm_s=v, force_n=f), f)
            ok2 = isinstance(r2, dict) and r2.get("contact") and r2.get("point_mm") is not None
            if ok2 and float(np.linalg.norm(_v(r2["point_mm"]) - c1)) <= tol:
                if r2.get("source") not in (None, "tool") or r2.get("at_tip") is False:
                    what = ", ".join(r2.get("likely_links") or []) or "the side of the tool"
                    raise Stop("blocked", what=what, mm=round(self._before(P, r2, u), 0), **self._met(r2))
                if P is not None and self._before(P, r2, u) > L.blocked_mm(self.B, q, u, sigma_mm=unc, v_mm_s=v):
                    raise Stop("blocked", what="a surface", mm=round(self._before(P, r2, u), 0), **self._met(r2))
                if moved_part:
                    self.notes.append("the first touch moved the part (or slid past an edge); the second met it "
                                      "where the first began")
                return r2
            stalls += 1
            stale = [x for x in (r.get("id"), (r2 or {}).get("id") if not ok2 else None) if x]
            if r.get("id"):
                self.ex.world.mark_stale([r["id"]], "a stall that did not repeat: no surface here", source="act")
            self.notes.append(f"a stall at {[round(float(x), 1) for x in c1]} did not repeat")
            if stalls >= L.CONFIRM_TRIES:
                raise Stop("not_found", mm=round(float((self.tip() - start) @ _v(u)), 0),
                           extra="felt a stall that did not repeat: no surface here")
            if ok2:
                self.ex.world.mark_stale([x for x in stale if x != r2.get("id")], "a stall that did not repeat",
                                         source="act")
            left = float(max_mm) - float((self.tip() - start) @ _v(u))
            if left <= L.rigid_mm(self.B, self.ex._sense().q, u):
                raise Stop("not_found", mm=round(float(max_mm), 0),
                           extra="felt a stall that did not repeat: no surface here")
    def _met(self, r):
        """
        What a stage that ended on a contact read itself: the force (the larger of this touch's and the
        call's
        peak so far) and the stored contact, so the result never says nothing happened beside a
        stored contact.
        """
        n = max(float(r.get("force_n") or 0.0), float(self.peak_total))
        return {"n": round(n, 1), "at": {"id": r.get("id"), "point_mm": r.get("point_mm"),
                                         "normal": r.get("normal_est")}}
    @staticmethod
    def _before(P, r, u):
        """Distance along u from a felt contact to the expected point."""
        if P is None or r.get("point_mm") is None:
            return 0.0
        return float((_v(P) - _v(r["point_mm"])) @ _v(u))
    def face_fit(self, c, u, part, f, sigma_mm=0):
        """Fit a normal from the centre and three contacts at the point-error and tool-width offset, rejecting moved contacts and mixed surfaces."""
        off = L.fit_offset_mm(self.B, self.ex._sense().q, u, sigma_mm=sigma_mm)
        lean = off
        e1, e2 = across(u)
        c = _v(c)
        b0 = c - _v(u) * (L.confirm_back_mm(self.B, self.ex._sense().q, u) + lean)
        pts, speeds = [c], []
        for k in range(FIT_TOUCHES):
            a = math.tau * k / FIT_TOUCHES
            e = math.cos(a) * e1 + math.sin(a) * e2
            here = b0 + e * off
            mv = self.move("fit", here - self.tip())
            if not isinstance(mv, dict) or "refused" in mv or mv.get("stopped") not in ("goal",):
                continue
            r = self.touch("fit", u, L.confirm_back_mm(self.B, self.ex._sense().q, u) + lean + lean + L.confirm_back_mm(self.B, self.ex._sense().q, u), f)
            if isinstance(r, dict) and r.get("contact") and r.get("source") in (None, "tool") \
                    and r.get("point_mm") is not None and r.get("stopped") != "moved_part":
                speeds.append(float(r.get("speed_mm_s") or 0))
                pts.append(_v(r["point_mm"]))     # a touch that moved something (or slid off an edge) is no point
        self.move("fit", c - _v(u) * (L.confirm_back_mm(self.B, self.ex._sense().q, u) + lean) - self.tip())
        self.move("fit", c - _v(u) * L.confirm_back_mm(self.B, self.ex._sense().q, u) - self.tip())
        n = fit_plane(pts, L.fit_resid_mm(self.B, self.ex._sense().q, u, v_mm_s=max(speeds or [0]), force_n=f))
        if n is not None and float(n @ _v(u)) > 0:
            n = -n
        return n, pts
    def settle(self):
        """Wait for force to stay under its local hidden-plus-noise threshold through the derived settling window, or for no progress."""
        ex, B = self.ex, self.B
        step, window = L.settle_step_s(B), L.settle_s(B)
        threshold = L.settle_force_n(B)
        if self.local is not None:
            threshold = max(threshold, self.local[0] + L.ratio("TWO_READ") * self.local[1])
        quiet, waited, history = 0, 0, []
        while waited < L.settle_max_s(B) + window:
            self.check_time("settle")
            ex.idle(min(step, L.settle_max_s(B) + window - waited))
            waited += min(step, L.settle_max_s(B) + window - waited)
            f = float(ex._sense().Fmag)
            quiet = quiet + step if f <= threshold else 0
            history.append((waited, f))
            while len(history) > 1 and waited - history[1][0] >= window:
                history.pop(0)
            no_progress = waited - history[0][0] >= window and max(x[1] for x in history) - min(x[1] for x in history) <= threshold
            if quiet >= window or no_progress:
                break
        return round(waited, 2)
    def retract(self, back_dir, mm, up_mm=None):
        """Retract under the core contact guard by the shown travel and tool width, then up to standoff when requested."""
        note = None
        try:
            r = self.move("release", _v(back_dir) * float(mm))
        except Stop as stop:
            if stop.stopped == "dark":
                raise
            return None
        if isinstance(r, dict) and r.get("stopped") in ("contact", "cap"):
            return "something came onto the finger as it left"
        if up_mm is not None and isinstance(r, dict) and "refused" not in r:
            try:
                r2 = self.move("release", [0, 0, up_mm])
                if isinstance(r2, dict) and r2.get("stopped") in ("contact", "cap"):
                    note = "something came onto the finger as it left"
            except Stop as stop:
                if stop.stopped == "dark":
                    raise
        return note
    def ramp(self, u, thing, until, hold_s=0.0, intent=None, sheet_forces=None,
             sheet_travel_mm=None, moved_n=None, touched_n=None, shown_mm=0.0):
        """Run a force leg, stopping a give leg at motion beyond earlier shown travel."""
        from rcpx.r2 import act_ramp
        return act_ramp.run(self, u, thing, until, hold_s, intent, sheet_forces,
                            sheet_travel_mm, moved_n, touched_n, shown_mm)


def field(value, name, default=None):
    """Read a field from a dictionary or a value object."""
    return value.get(name, default) if isinstance(value, dict) else getattr(value, name, default)


def reach_mm(ex, tip_mm, u):
    """Distance along the line inside the model's reach sphere, centred on its base origin."""
    radius = float(ex.model.reach()["max_reach_mm"])
    p, u = _v(tip_mm), _unit(u)
    along = float(p @ u)
    cross_sq = float(p @ p) - along ** 2
    return max(0, -along + math.sqrt(max(0, radius ** 2 - cross_sq)))
__all__ = ["MOTIONS", "Run", "Stop", "across", "fit_plane", "largest_size", "level_line", "line", "reach_mm"]
