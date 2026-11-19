# SPDX-License-Identifier: AGPL-3.0-only
"""Surface push geometry, motion gates and pure stroke watches."""
import math
import mujoco
import numpy as np
from rcpx.r2.shove_bounds import refuse as refusal
from rcpx.control_base import _mm
from rcpx.control_plan import vec_from_rot
from rcpx.r2 import shove_bounds as b

MM = 1000
UP = np.array([0, 0, 1])
TILTS_DEG = (0, 15, 30, 45)


def unit(v):
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    return v / n if n else v


def horizontal(v):
    v = np.asarray(v, float).copy()
    v[2] = 0
    return unit(v)


def bearing_deg(v):
    return math.degrees(math.atan2(float(v[1]), float(v[0])))


def pose(p, R):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


class HandShape:
    """Corners of the hand's collision geometry in the tool frame (tip at the origin), jaws closed (0.0) and fully
    open (1.0), and the closing axis in the tool frame; from the body model alone."""

    def __init__(self, model):
        m = model.mj
        d = mujoco.MjData(m)
        g = model.spec.get("gripper") or {}
        tip = m.site("tip").id
        top = int(m.site_bodyid[tip])

        def under(b):
            while b > 0:
                if b == top:
                    return True
                b = int(m.body_parentid[b])
            return False

        geoms = [i for i in range(m.ngeom) if (m.geom_contype[i] or m.geom_conaffinity[i])
                 and under(int(m.geom_bodyid[i]))]
        jaw_bodies = [int(m.jnt_bodyid[m.joint(j).id]) for j in g.get("joints", [])]
        self.corners, jaw_at = {}, {}
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
        for frac in (0.0, 1.0):
            q = np.asarray(model._qfull, dtype=float).copy()
            for j, c, o in zip(g.get("joints", []), g.get("closed", []), g.get("open", [])):
                q[m.jnt_qposadr[m.joint(j).id]] = float(c) + frac * (float(o) - float(c))
            d.qpos[:] = q
            mujoco.mj_kinematics(m, d)
            R, p = d.site_xmat[tip].reshape(3, 3), d.site_xpos[tip]
            pts = []
            for i in geoms:
                c, h = m.geom_aabb[i][:3], m.geom_aabb[i][3:]
                Rg, pg = d.geom_xmat[i].reshape(3, 3), d.geom_xpos[i]
                pts.extend((R.T @ (pg[:, None] + Rg @ (c[:, None] + (h * signs).T) - p[:, None])).T)
            # a model with no collision shape under the tip body is taken as the tip alone
            self.corners[frac] = np.asarray(pts, dtype=float).reshape(-1, 3) if pts else np.zeros((1, 3))
            jaw_at[frac] = [R.T @ (d.xpos[b] - p) for b in jaw_bodies]
        self.axis_T = np.array([0.0, 1.0, 0.0])
        if jaw_bodies:
            move = jaw_at[1.0][0] - jaw_at[0.0][0]
            move[0] = 0.0                                   # across the pointing axis, not along it
            if float(np.linalg.norm(move)) > 0:
                self.axis_T = unit(move)

    def profile(self, R, frac, d, band_m, ahead_m, near_m):
        """How the hand meets an thing pushed along d (unit, horizontal) at hand rotation R: lowest_m (the lowest
        point's height relative to the tip, along up), front_m (the front of the part that meets it, along d from
        the tip), width_m (that part's width across the push), overhang_m (how high over the lowest point the
        first part of the hand ahead of that front stands; inf when none is)."""
        P = self.corners[frac] @ np.asarray(R, dtype=float).T
        along, height = P @ d, P @ UP
        across = P @ unit(np.cross(UP, d))
        low = float(height.min())
        band = height <= low + band_m
        front = float(along[band].max())
        ahead = along > front + ahead_m
        near = band & (along >= front - near_m)
        return {"lowest_m": low, "front_m": front,
                "width_m": float(across[near].max() - across[near].min()) if near.any() else 0.0,
                "overhang_m": float(height[ahead].min() - low) if ahead.any() else math.inf}


class PushLawMixin:
    def _sv_hand(self):
        if getattr(self, "_sv_shape", None) is None:
            self._sv_shape = HandShape(self.model)
        return self._sv_shape

    def _sv_rot(self, t, across):
        a_T, x_T = self._sv_hand().axis_T, np.array([1, 0, 0])
        a = unit(np.asarray(across, float) - float(np.dot(across, t)) * t)
        return np.column_stack([t, a, np.cross(t, a)]) @ np.column_stack([x_T, a_T, np.cross(x_T, a_T)]).T

    def _sv_poses(self, d, frac, geom):
        across = unit(np.cross(UP, d))
        for tilt in TILTS_DEG:
            a = math.radians(tilt)
            t = -math.cos(a) * UP + math.sin(a) * d
            for flip in (1, -1):
                R = self._sv_rot(t, flip * across)
                yield tilt, R, self._sv_hand().profile(R, frac, d, geom["band_m"], geom["ahead_m"], geom["profile_near_m"])

    def _sv_ik(self, p_tip, R, seeds):
        for q0 in seeds:
            if q0 is None:
                continue
            kw = {"R_target": R} if self.model.n >= 6 else {"keep_axis": ("x", R[:, 0]), "rot_tol": math.radians(3)}
            q, ok, _ = self.model.ik(np.asarray(q0, float), np.asarray(p_tip, float), restarts=4, **kw)
            if ok and self.model.within_limits(q):
                return q
        return None

    def _sv_gate(self, name, kwargs):
        return self.gates(name, kwargs) if hasattr(self, "gates") else None

    def _sv_record(self, contact):
        return self._record_contact(contact, "push guard")

    def _sv_line(self, dp, drot=None, *, kind="transit", target=None, intent=None, away_from=None, held=None,
                 record=True):
        dp = np.asarray(dp, float)
        bad = self._sv_gate("move_tip" if drot is None else "orient_tool",
                            {"dx": dp[0] * MM, "dy": dp[1] * MM, "dz": dp[2] * MM, "clear": (intent or {}).get("clear")})
        if bad:
            return bad
        r = b.line(self, dp, drot, kind=kind, target=target, intent=intent, away_from=away_from, held=held)
        if "refused" not in r:
            r.setdefault("stopped", r.get("stop"))
            if r["stopped"] == "contact" and record:
                self._sv_record(r.get("contact"))
        return r

    def _sv_joints(self, q, *, kind="transit", target=None, intent=None, away_from=None, held=None):
        bad = self._sv_gate("move_joints", {"q_deg": [math.degrees(float(x)) for x in q], "clear": (intent or {}).get("clear")})
        if bad:
            return bad
        r = b.joints(self, q, kind=kind, target=target, intent=intent, away_from=away_from, held=held)
        if "refused" not in r:
            r.setdefault("stopped", r.get("stop"))
        return r

    def _sv_stream(self, legs, *, held=None):
        for leg in legs:
            bad = self._sv_gate("move_tip" if leg.get("line") else "move_joints", {"clear": (leg.get("intent") or {}).get("clear")})
            if bad:
                return bad
        return b.stream(self, legs, held=held)

    def _sv_goto(self, q, *, kind="transit", target=None, intent=None, away_from=None, held=None):
        s = b.sense(self)
        p, R = self.model.fk_tip(q)
        bad = self._sv_joints(q, kind=kind, target=target, intent=intent, away_from=away_from, held=held)
        if "refused" not in bad:
            return bad
        if not str((bad.get("refused") or {}).get("check", "")).startswith("path"):
            return bad
        B = b.of(self)
        radius = max(float(np.linalg.norm(c)) for c in self._sv_hand().corners[0])
        route, note = b.via(B, s.tip, p, radius, s.q, b.thing(B, target), held)
        legs = []
        for via in route:
            q_via = self._sv_ik(via, R, [q, s.q])
            if q_via is None:
                return bad
            legs.append(dict(kind=kind, to=q_via, intent=intent, target=target, away_from=away_from))
        r = self._sv_stream(legs, held=held)
        if note:
            r["route_note"] = note
        return r

    def _sv_jaws(self, frac):
        if b.jaw_command(self) == frac:
            return {}
        bad = self._sv_gate("gripper", {"width_frac": frac})
        return bad if bad else self._sv_gripper(float(frac), hold=False)

    def _sv_gripper(self, frac, hold):
        return self.gripper(frac, hold=hold)

    def _sv_touch_core(self, kw, intent):
        import inspect
        if "clear" in inspect.signature(self.touch).parameters:
            kw["clear"] = (intent or {}).get("clear")
        return self.touch(**kw)

    def _sv_touch(self, z_lowest_goal, prof, intent, target):
        s = b.sense(self)
        down = float(s.tip[2]) + prof["lowest_m"] - z_lowest_goal
        if down <= b.of(self).rigid(s.q, -UP):
            return float(s.tip[2]) + prof["lowest_m"], False, None, {}
        # The old touch cannot traverse a long free leg without its removed duration cap.
        # Approach to the target band first; a contact stop is the finding, never a fallback.
        E = b.thing(b.of(self), target)
        edge = z_lowest_goal + b.band(b.of(self), E)
        fast = max(0, s.tip[2] + prof["lowest_m"] - edge)
        if fast > b.of(self).rigid(s.q, -UP):
            r = self._sv_line(-UP * fast, kind="approach", target=E, intent=intent, record=False)
            if "refused" in r:
                return None, False, r, r
            if r["stopped"] == "contact":
                return float(b.sense(self).tip[2]) + prof["lowest_m"], True, None, r
            if r["stopped"] != "goal":
                return None, False, r, r
            s = b.sense(self)
            down = max(0, float(s.tip[2]) + prof["lowest_m"] - z_lowest_goal)
            if down <= b.of(self).rigid(s.q, -UP):
                return float(s.tip[2]) + prof["lowest_m"], False, None, r
        kw = {"direction": [0, 0, -1], "max_mm": down * MM}
        bad = self._sv_gate("touch", dict(kw, clear=(intent or {}).get("clear")))
        if bad:
            return None, False, bad, {}
        r = self._sv_touch_core(kw, intent)
        if "refused" in r:
            return None, False, r, r
        return float(b.sense(self).tip[2]) + prof["lowest_m"], bool(r.get("contact")), None, r

    def _sv_stroke_check(self, s, d, travel):
        return self._stroke_check(s, d, travel)

    def _sv_core_push(self, s, d, travel_m, ceiling_n, v_push):
        return self._push(s, d, travel_m, ceiling_n, v_push, 0, stop_on_force=True, hold=self.model.n < 6)

    def _sv_history(self, done):
        return self._push_history(done)

    def _sv_ease(self):
        return self._ease_impl()

    def _sv_look(self, text):
        return self._look_impl(text)

    def _sv_stale(self, point_mm, text):
        return self._stale_near(point_mm, text)

    def _sv_stroke(self, d, distance_m, force_n, free_m, follow_m, target, intent):
        B, s = b.of(self), b.sense(self)
        spec = b.stroke(B, target, s.q, d, max(0, float(s.F @ d)), distance_m, intent)
        travel = free_m + distance_m + follow_m
        bad = self._sv_gate("press", {"direction": d.tolist(), "travel_mm": travel * MM,
                                      "force_n": force_n, "clear": intent.get("clear")})
        bad = bad if bad else self._sv_stroke_check(s, d, travel)
        if bad:
            return bad
        w = PushWatch(d, s.tip, distance_m, free_m, follow_m, spec["meet_n"], spec["readings"],
                      spec["lost_periods"], spec["stall_count"], spec["stall_m"], spec["give"],
                      spec["steady_min"], moved_m=spec["moved_m"])
        self.state.setdefault("push_law", {})["watch"] = w
        w(s)
        try:
            r = self._sv_core_push(s, d, travel, spec["force"].ceiling_n, spec["speed"])
        finally:
            self.state["push_law"]["watch"] = None
        out, stop = w.report(), r.get("stop", r.get("stopped"))
        if stop in ("cap", "off_axis"):
            stop = "force"
        elif stop in ("travel", "lead", "time"):
            stop = "no_contact" if w.met is None else "limit"
        elif stop == "slipped":
            stop, out["slipped"] = "lost_contact", True
        out.update(stopped=stop, end=b.sense(self).tip.copy(),
                   periods=r.get("periods", max(0, len(r.get("done", [])) - 1)), why=r.get("cap_why", ""))
        self._sv_history(r.get("done", []))
        return B.report(out, spec["force"].items)

    def _sv_plan(self, point, d, frac, standoff_m, geom):
        s = b.sense(self)
        B = b.of(self)
        S = b.support(B, point, self.cfg.get("floor_z_m"))
        for tilt, R, prof in self._sv_poses(d, frac, geom):
            z = point[2] + geom["above_m"] - prof["lowest_m"]
            feel = point + d * geom["feel_m"]
            feel[2] = z
            back = point - d * (prof["front_m"] + standoff_m)
            back[2] = z
            q_feel = self._sv_ik(feel, R, [s.q, self.model.q_home])
            q_back = self._sv_ik(back, R, [q_feel]) if q_feel is not None else None
            low = back.copy()
            low[2] = S.point[2] + geom["low_plan_m"] - prof["lowest_m"]
            if q_back is not None and self._sv_ik(low, R, [q_back]) is not None:
                return tilt, R, prof, q_feel, q_back
        return None

    def _sv_push_run(self, point, d, distance_m, height_m, frac, force_n, standoff_m, height_rule, feel, verb,
                     intent=None, distance_stated=True):
        B, s = b.of(self), b.sense(self)
        intent = intent or {}
        point = np.asarray(point, float)
        E = B.thing(point)
        S = b.support(B, point, self.cfg.get("floor_z_m"))
        spec = b.stroke(B, E, s.q, d, max(0, float(s.F @ d)), distance_m, intent)
        geom = b.geometry(B, E, S, s.q, d, spec["speed"], max(0, point[2] - S.point[2]))
        plan = self._sv_plan(point, d, frac, standoff_m, geom)
        if plan is None:
            return {}, refusal("not_reachable", "no arm pose holds the hand over the point and low behind it for this push",
                               "push from a point nearer the robot's base, or along another direction")
        tilt, R, prof, q_feel, q_back = plan
        # Refuse an unreachable stated end before any phase moves. Search shorter travel by halving.
        end = self.model.fk_tip(q_back)[0] + d * (standoff_m + distance_m)
        end[2] = S.point[2] + geom["low_m"] - prof["lowest_m"]
        if self._sv_ik(end, R, [q_back]) is None:
            asked = distance_m
            while asked > B.rigid(s.q, d):
                asked /= 2
                trial = end - d * (distance_m - asked)
                if self._sv_ik(trial, R, [q_back]) is not None:
                    break
            return {}, refusal("not_reachable", "the asked stroke end is outside the arm's own IK reach",
                               f"shove(point_mm={_mm(point)}, direction_deg={bearing_deg(d):g}, distance_mm={asked * MM:g})")
        phases = b.Phases(B)
        f = dict(hand_tilt_deg=tilt, contact_width_mm=prof["width_m"] * MM, direction_deg=bearing_deg(d))
        t0 = self.sim_time()
        def finish(bad=None):
            f["world_s"] = self.sim_time() - t0
            return phases.result(f, spec["force"].items), bad
        def move(fn):
            start = self.sim_time()
            r = fn()
            phases.add(r, self.sim_time() - start)
            if "refused" in r:
                return r
            if r.get("stopped", r.get("stop")) != "goal":
                f["stopped"] = r.get("stopped", r.get("stop"))
                return r
            return None
        phases.begin("prepare", "command the hand opening")
        start = self.sim_time()
        r = self._sv_jaws(frac)
        phases.add(r, self.sim_time() - start, jaw=True)
        if r and "refused" in r:
            return finish(r)
        top, how = float(point[2]), "the stated point height"
        if feel:
            phases.begin("top", "feel the top")
            bad = move(lambda: self._sv_goto(q_feel, intent=intent))
            if bad:
                return finish(bad if "refused" in bad else None)
            start = self.sim_time()
            z, touched, bad, r = self._sv_touch(point[2] - geom["past_m"], prof, intent, E)
            phases.add(r, self.sim_time() - start)
            if bad:
                return finish(bad)
            if touched:
                top, how = max(top, z), "felt by touch"
        f.update(top_z_mm=top * MM, top=how)
        phases.begin("support", "retrace, go behind the point and find support")
        back, support_z = 0, None
        while support_z is None:
            p_back = self.model.fk_tip(q_back)[0] - d * back
            q_go = self._sv_ik(p_back, R, [q_back, b.sense(self).q])
            if q_go is None:
                f.update(stopped="limit", why="the arm cannot reach farther behind the point")
                return finish()
            s = b.sense(self)
            legs = []
            if p_back[2] - s.tip[2] > B.rigid(s.q, UP):
                legs.append(dict(kind="lift", to=pose([s.tip[0], s.tip[1], p_back[2]], s.R), line=True,
                                 intent=intent, away_from=E))
            legs.append(dict(kind="transit", to=q_go, intent=intent))
            bad = move(lambda: self._sv_stream(legs))
            if bad:
                return finish(bad if "refused" in bad else None)
            s = b.sense(self)
            fast = max(0, s.tip[2] + prof["lowest_m"] - point[2])
            r = {"stopped": "goal"}
            if fast > B.rigid(s.q, -UP):
                start = self.sim_time()
                r = self._sv_line(-UP * fast, kind="approach", target=S, intent=intent, record=False)
                phases.add(r, self.sim_time() - start)
                if "refused" in r:
                    return finish(r)
                if r["stopped"] not in ("goal", "contact"):
                    f["stopped"] = r["stopped"]
                    return finish()
            if r["stopped"] == "contact":
                z, touched = b.sense(self).tip[2] + prof["lowest_m"], True
            else:
                start = self.sim_time()
                z, touched, bad, r = self._sv_touch(S.point[2] - geom["search_m"], prof, intent, S)
                phases.add(r, self.sim_time() - start)
                if bad:
                    return finish(bad)
            if touched and z >= top - geom["on_top_m"]:
                back += standoff_m
                continue
            support_z = z if touched else S.point[2]
        f.update(support_z_mm=support_z * MM, support="touched" if touched else "support from " + S.extent["source"],
                 stepped_back_mm=back * MM)
        height = max(0, top - support_z)
        h, note, tall = b.push_height(height, prof["overhang_m"], geom["palm_m"], geom["face_m"], geom["low_m"])
        if height_m is not None:
            if height_m > height:
                return finish(refusal("height_mm", f"height_mm {height_m * MM:.0f} is above the felt top "
                                      f"({height * MM:.0f} mm over the support)",
                                      "leave height_mm out, or state a height under the top"))
            h = max(height_m, h)
        elif verb == "tip":
            h = max(h, height - geom["face_m"])
        prof = self._sv_hand().profile(R, frac, d, max(0, height - h), geom["ahead_m"], geom["profile_near_m"])
        tall = tall or h > height - geom["face_m"] or h < height + geom["palm_m"] - prof["overhang_m"]
        if tall:
            f.update(stopped="too_tall", why="no push height keeps the hand's overhang clear of the felt top")
            return finish()
        if note:
            f["height_note"] = note
        f["contact_width_mm"] = prof["width_m"] * MM
        f["push_height_mm"] = h * MM
        if not distance_stated:
            distance_m = h
            intent["travel"] = h
        phases.begin("height", "raise to the derived push height")
        s = b.sense(self)
        bad = move(lambda: self._sv_line(UP * (support_z + h - s.tip[2] - prof["lowest_m"]),
                                         kind="lift", away_from=S, intent=intent, record=False))
        if bad:
            return finish(bad if "refused" in bad else None)
        phases.begin("stroke", "follow the thing along the stated direction")
        s = b.sense(self)
        free = b.free_travel(B, E, s.q, d, point, s.tip + d * prof["front_m"])
        start = self.sim_time()
        st = self._sv_stroke(d, distance_m, force_n, free, h if verb == "tip" else 0, E, intent)
        phases.add(st, self.sim_time() - start)
        if "refused" in st:
            return finish(st)
        f.update({k: v for k, v in st.items() if k not in ("end", "met_at", "bounds", "periods")})
        if st.get("met_at") is not None:
            f["met_at_mm"] = _mm(st["met_at"] + d * prof["front_m"])
            self._sv_stale(f["met_at_mm"], "a push moved what was there")
        f["hand_end_mm"] = _mm(st["end"] + d * prof["front_m"])
        f["moved_min_m"] = spec["moved_m"]
        met = st.get("met_at")
        delta = np.zeros(3) if met is None else st["end"] - met
        off = float(np.linalg.norm(delta - d * float(delta @ d)))
        yaw = math.atan2(float(np.dot(delta, unit(np.cross(UP, d)))), float(delta @ d)) if np.linalg.norm(delta) else 0
        next_m = b.next_stroke(B, st["pushed_m"], off, yaw, prof["width_m"], b.absorb(B, E, prof["width_m"]))
        f["next_stroke_mm"] = None if next_m is None else next_m * MM
        phases.begin("after", "ease, retrace away and look")
        start = self.sim_time()
        self._sv_ease()
        phases.add({}, self.sim_time() - start)
        s = b.sense(self)
        retreat = s.tip - d * geom["backoff_m"]
        over = retreat.copy()
        over[2] = self.model.fk_tip(q_feel)[0][2]
        legs = [dict(kind="lift", to=pose(retreat, s.R), line=True, intent=intent, away_from=E),
                dict(kind="lift", to=pose(over, s.R), line=True, intent=intent, away_from=E)]
        bad = move(lambda: self._sv_stream(legs))
        if bad:
            return finish(bad if "refused" in bad else None)
        lk = self._sv_look("after a push")
        f["look"] = {k: lk[k] for k in ("look_id", "annotated", "frame", "can_see") if k in lk}
        return finish()


class PushWatch:
    """One reading per period; every threshold is supplied by the caller."""

    def __init__(self, d, tip0, distance_m, free_m, follow_m, meet_n, readings, lost_periods,
                 stall_periods, stall_m, give, steady_min, moved_m=0):
        self.d, self.tip0 = np.asarray(d, float), np.asarray(tip0, float).copy()
        self.distance, self.free, self.follow = distance_m, free_m, follow_m
        self.meet_n, self.readings, self.lost_periods = meet_n, readings, lost_periods
        self.stall_periods, self.stall_m, self.give, self.steady_min = stall_periods, stall_m, give, steady_min
        self.moved_m, self.met, self.met_along, self.lost_at, self.bias = moved_m, None, None, None, None
        self.over, self.low, self.peak, self.contact_periods, self.steady_periods, self.hist = 0, 0, 0, 0, 0, []

    def __call__(self, s):
        along, f = float((s.tip - self.tip0) @ self.d), float(s.F @ self.d)
        self.hist.append(along)
        if self.bias is None:
            self.bias = f
            return None
        f -= self.bias
        self.peak = max(self.peak, f)
        if self.met is None:
            self.over = self.over + 1 if f >= self.meet_n and (f > 0 or self.meet_n > 0) else 0
            if self.over >= self.readings:
                self.met, self.met_along = s.tip.copy(), along
            elif along >= self.free:
                return "no_contact"
            elif self._stalled(f):
                return "limit"
            return None
        self.contact_periods += 1
        self.steady_periods += int(f >= self.give * self.meet_n)
        if along - self.met_along >= self.distance:
            return "done"
        self.low = self.low + 1 if f < self.give * self.meet_n else 0
        if f >= self.meet_n:
            self.lost_at = None
        if self.low >= self.lost_periods and self.lost_at is None:
            self.lost_at = along
        if self.lost_at is not None and along - self.lost_at >= self.follow:
            return "lost_contact"
        if self._stalled(f):
            return "limit"
        return None

    def _stalled(self, f):
        return (len(self.hist) > self.stall_periods and
                self.hist[-1] - self.hist[-1 - self.stall_periods] < self.stall_m and f < self.meet_n)

    def report(self):
        pushed = max(0, self.hist[-1] - self.met_along) if self.met is not None else 0
        share = self.steady_periods / self.contact_periods if self.contact_periods else 0
        return dict(pushed_m=pushed, peak_n=self.peak, met_at=self.met, meet_n=self.meet_n,
                    followed=bool(self.met is not None and share >= self.steady_min and pushed > self.moved_m),
                    steady_share=share, free_m=self.met_along if self.met is not None else (self.hist[-1] if self.hist else 0))


def install(ex):
    st = ex.state.setdefault("push_law", {"watch": None})
    if st.get("tick"):
        return
    def tick(s):
        w = (ex.state.get("push_law") or {}).get("watch")
        return None if w is None else w(s)
    ex.add_tick(tick)
    st["tick"] = True
