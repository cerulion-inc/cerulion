# SPDX-License-Identifier: AGPL-3.0-only
"""Turning and set-down with held-load context and a supplied touchdown watch."""
import math
import numpy as np
from rcpx.r2.shove_bounds import refuse as refusal
from rcpx.control_base import _mm
from rcpx.control_plan import vec_from_rot
from rcpx.r2 import shove_bounds as b
from rcpx.r2.shove_law import MM, UP, unit, pose

WRIST_BEARINGS_DEG = (0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180)


def rot_about(axis, angle):
    k = unit(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def tilt_deg(v):
    return math.degrees(math.acos(max(-1, min(1, float(unit(v)[2])))))


class LowerWatch:
    def __init__(self, f0_up, thr_n, readings):
        self.f0, self.thr, self.readings, self.n, self.seen = float(f0_up), thr_n, readings, 0, None

    def __call__(self, s):
        f = float(s.F[2]) if hasattr(s, "F") else float(s)
        self.n = self.n + 1 if self.f0 - f >= self.thr and (self.f0 - f > 0 or self.thr > 0) else 0
        if self.n >= self.readings:
            self.seen = s.tip.copy() if hasattr(s, "tip") else f
            return "touchdown"
        return None


class TurnMixin:
    def _up_axis(self, up_axis, axis_from, axis_to):
        """(a_T unit, from_T or None, to_T or None, refusal or None) in the grasp frame."""
        g = self.state["upright"].get("grasp")
        if g is None:
            return None, None, None, refusal("grasp", "no grasp pose is known for what the jaws hold",
                                             "pick it with pick, then set it upright")
        R_g, tip_g = np.asarray(g["R"], dtype=float), np.asarray(g["tip"], dtype=float)
        if up_axis not in (None, "from_look"):
            try:
                v = np.asarray(up_axis, dtype=float).reshape(-1)
            except (TypeError, ValueError):
                v = np.zeros(0)
            if v.size != 3 or not np.all(np.isfinite(v)) or float(np.linalg.norm(v)) == 0:
                return None, None, None, refusal("up_axis", f"up_axis must be from_look or [x, y, z] in the grasp "
                                                 f"frame, got {up_axis!r}", "from_look with axis_from and axis_to")
            return unit(v), None, None, None
        pts = []
        for value, what in ((axis_from, "axis_from"), (axis_to, "axis_to")):
            if value is None:
                return None, None, None, refusal(what, f"from_look needs axis_from and axis_to: the two points that "
                                                 "say which way is up on the thing", "mark the thing's bottom "
                                                 "and top in two looks before the pick, triangulate, give both labels")
            p, bad = self._up_point(value, what, g)
            if bad:
                return None, None, None, bad
            pts.append(p)
        if float(np.linalg.norm(pts[1] - pts[0])) <= b.axis_min(b.of(self), pts[0], pts[1]):
            return None, None, None, refusal("axis", "axis ends overlap within their position bands: no axis follows",
                                             "two points on the thing at least a few cm apart, bottom then top")
        a_T = R_g.T @ unit(pts[1] - pts[0])
        return a_T, R_g.T @ (pts[0] - tip_g), R_g.T @ (pts[1] - tip_g), None

    def _up_point(self, value, what, grasp):
        if isinstance(value, str):
            hits = [e for e in self.world.query(label=value) if e.get("point_mm") is not None]
            if not hits:
                return None, refusal(what, f"robot://world holds no point labelled {value!r}",
                                     "locate it (mark it in two looks, then triangulate) before the pick")
            e = hits[-1]
            if int((e.get("observed_at") or {}).get("call", 0)) > int(grasp.get("call", 0)):
                return None, refusal("marks_moved", f"{value!r} was located after the pick, while the thing was "
                                     "held: it moves with the hand between the looks, so its point is not where it "
                                     "was", "give the points located before the pick (or put it down, mark them "
                                     "again, pick again)")
            return np.asarray(e["point_mm"], dtype=float) / MM, None
        try:
            v = np.asarray(value, dtype=float).reshape(-1)
        except (TypeError, ValueError):
            v = np.zeros(0)
        if v.size != 3 or not np.all(np.isfinite(v)):
            return None, refusal(what, f"{what} must be a located label or [x, y, z] mm, got {value!r}",
                                 "the label you located the point under before the pick")
        return v / MM, None

    def _up_support_ok(self, p):
        B = b.of(self)
        S = b.support(B, p, self.cfg.get("floor_z_m"))
        reach = self._up_reach(None, None)
        candidates = [S]
        st = self.state.get("place") or {}
        if st.get("support_z") is not None:
            sp = p.copy()
            sp[2] = st["support_z"]
            candidates.append(B.thing(sp))
        for e in self.world.query(include_stale=False):
            q = e.get("point_mm")
            if q is not None and e.get("kind") in ("contact", "located"):
                candidates.append(B.thing(np.asarray(q, float) / MM))
        heights, ok = [], False
        for C in candidates:
            if C.point is None:
                continue
            tol, near = b.support_match(B, p, C, reach)
            if np.linalg.norm(np.asarray(C.point)[:2] - p[:2]) <= near:
                heights.append(float(C.point[2]))
                ok = ok or abs(p[2] - C.point[2]) <= tol
        return ok, heights

    def _up_reach(self, f_T, t_T):
        st = self.state.get("place") or {}
        J = getattr(self, "_pl_jaws", None)
        return b.reach(None if f_T is None else (f_T, t_T), st.get("hang_m"), st.get("height_m"),
                       float(getattr(J, "max_m", 0)), getattr(J, "finger_len_m", None), st.get("width_m"))

    def _up_plan_hold(self, q, dp, drot):
        u = unit(dp) if np.linalg.norm(dp) else UP
        return self._plan_hold(q, dp, drot, b.of(self).v_free(q, u), True)

    def _up_floor(self, q):
        return self.floor_clearance(q)

    def _up_plan(self, a_T, f_T, t_T, place, release_m):
        B, s = b.of(self), b.sense(self)
        st = self.state["place"]
        S = b.support(B, place, self.cfg.get("floor_z_m"))
        reach = self._up_reach(f_T, t_T)
        geom = b.turning(B, S, s.q, reach)
        support_z = st.get("support_z") if st.get("support_z") is not None else S.point[2]
        z_safe = max(s.tip[2], max(support_z, place[2]) + reach + geom["clear_m"])
        q_up = self._sv_ik([s.tip[0], s.tip[1], z_safe], s.R, [s.q])
        if q_up is None:
            return None
        a_now = s.R @ a_T
        k = np.cross(a_now, UP)
        ang = math.acos(max(-1, min(1, float(a_now @ UP))))
        if np.linalg.norm(k) == 0:
            k = s.R @ self._sv_hand().axis_T
        R_min = rot_about(k, ang) if ang else np.eye(3)
        ref = f_T if f_T is not None else np.asarray(st.get("offset_T") or [0, 0, 0], float)
        for psi in WRIST_BEARINGS_DEG:
            R = rot_about(UP, math.radians(psi)) @ R_min @ s.R
            path, stop, _, _ = self._up_plan_hold(q_up, np.zeros(3), vec_from_rot(R @ s.R.T))
            if isinstance(path, dict) or stop != "goal" or not path:
                continue
            q_turn = path[-1]
            p_turn = self.model.fk_tip(q_turn)[0]
            low = np.asarray(place, float) + UP * float(release_m or 0) - R @ ref
            if f_T is None:
                low += UP * float(st.get("hang_m") or 0)
            high = np.array([low[0], low[1], max(low[2] + geom["carry_m"], p_turn[2])])
            path2, stop2, _, _ = self._up_plan_hold(q_turn, high - p_turn, None)
            if isinstance(path2, dict) or stop2 != "goal":
                continue
            q_high = path2[-1] if path2 else q_turn
            q_set = self._sv_ik(low, R, [q_high])
            if q_set is None or self._up_floor(q_set) < geom["margin_m"]:
                continue
            search = 0
            if release_m is None:
                q_deep = self._sv_ik(low - UP * geom["search_m"], R, [q_set])
                if q_deep is not None and self._up_floor(q_deep) >= geom["margin_m"]:
                    search = geom["search_m"]
            return dict(z_safe=z_safe, R=R, q_up=q_up, q_turn=q_turn, high=high, q_high=q_high,
                        q_set=q_set, bearing_deg=psi, turned_deg=math.degrees(ang), search_m=search,
                        support=S, clear_m=geom["clear_m"],
                        extent_note="the held extent is not known; reach uses jaw opening and finger length" if
                        f_T is None and st.get("hang_m") is None and st.get("height_m") is None else None)
        return None

    def _up_run(self, plan, a_T, f_T, t_T, place, release_m, intent=None):
        B, st = b.of(self), self.state["place"]
        held = b.held_of(st, float(np.linalg.norm(self.model.mj.opt.gravity)))
        _, held["accel_ok"], grip_items = b.grip(B, held)
        phases, t0 = b.Phases(B), self.sim_time()
        f = dict(wrist_bearing_deg=plan["bearing_deg"], turned_deg=plan["turned_deg"], extent_note=plan["extent_note"])
        def finish(bad=None):
            f["world_s"] = self.sim_time() - t0
            return phases.result(f, grip_items), bad
        phases.begin("carry", "rise, turn and carry over the support")
        S = plan["support"]
        legs = [dict(kind="lift", to=plan["q_up"], intent=intent, away_from=S),
                dict(kind="carry", to=plan["q_turn"], intent=intent),
                dict(kind="carry", to=pose(plan["high"], plan["R"]), line=True, target=S, intent=intent)]
        start = self.sim_time()
        r = self._sv_stream(legs, held=held)
        phases.add(r, self.sim_time() - start)
        if "refused" in r:
            return finish(r)
        if r["stopped"] != "goal":
            f.update(stopped=r["stopped"], why="the carry stopped")
            return finish()
        phases.begin("lower", "lower until the support takes the held weight")
        p_set = self.model.fk_tip(plan["q_set"])[0]
        s = b.sense(self)
        _, thr, readings = b.lower(B, S, s.q, held, intent)
        w = LowerWatch(s.F[2], thr, readings)
        self.state["upright"]["watch"] = w
        start = self.sim_time()
        try:
            r = self._sv_line(-UP * max(0, s.tip[2] - p_set[2] + plan["search_m"]), kind="lower",
                              target=S, held=held, intent=intent, record=False)
        finally:
            self.state["upright"]["watch"] = None
        phases.add(r, self.sim_time() - start)
        if "refused" in r:
            return finish(r)
        landed = r["stopped"] in ("touchdown", "contact")
        lost = r["stopped"] == "dropped" or bool(st.get("dropped"))
        if not landed and not lost and r["stopped"] not in ("goal", "limit"):
            f.update(stopped=r["stopped"], why="the lower stopped")
            return finish()
        s = b.sense(self)
        f.update(tilt_deg_held=tilt_deg(s.R @ a_T), landed=landed, dropped=lost,
                 stopped="touchdown" if landed else ("dropped" if lost else
                         ("release_height" if release_m is not None else "no_touchdown")))
        if f_T is not None:
            bottom = s.tip + s.R @ f_T
            f["set_down_mm"] = _mm(bottom)
            self.state["upright"]["set_down"] = bottom.tolist()
        phases.begin("release", "reopen to the pick's opening")
        st["paused"] = True
        J = getattr(self, "_pl_jaws", None)
        opening = st.get("open_w")
        frac = J.frac_for_width(opening) if J is not None and opening is not None else 1
        start = self.sim_time()
        r = self._sv_gripper(frac, hold=True)
        phases.add(r, self.sim_time() - start, jaw=True)
        if r and "refused" in r:
            st["paused"] = False
            return finish(r)
        st.update(held=False, paused=False, dropped=False, report=None)
        phases.begin("after", "retrace up to the turning height and look")
        start = self.sim_time()
        r = self._sv_line(UP * max(0, plan["z_safe"] - b.sense(self).tip[2]), kind="lift",
                          intent=intent, away_from=S, record=False)
        phases.add(r, self.sim_time() - start)
        if "refused" in r:
            return finish(r)
        if r["stopped"] != "goal":
            f["why"] = "the rise after release stopped"
            return finish()
        start = self.sim_time()
        look, bad = self._up_look(place, plan["z_safe"], intent)
        phases.add({}, self.sim_time() - start)
        f["look"] = look
        self.note_event("upright_set", landed=landed, set_down_mm=f.get("set_down_mm"))
        return finish(bad)

    def _up_look(self, place, turning_z, intent):
        s = b.sense(self)
        down = -UP
        a = unit(s.R @ self._sv_hand().axis_T - float((s.R @ self._sv_hand().axis_T) @ down) * down)
        if np.linalg.norm(a) == 0:
            a = np.array([0, 1, 0])
        R = self._sv_rot(down, a)
        q = self._sv_ik([place[0], place[1], turning_z], R, [s.q, self.model.q_home])
        if q is not None:
            r = self._sv_goto(q, kind="look", intent=intent)
            if "refused" in r:
                return {}, r
            if r["stopped"] != "goal":
                return {}, None
        lk = self._sv_look("after set_upright")
        return {k: lk[k] for k in ("look_id", "annotated", "frame", "can_see") if k in lk}, None
