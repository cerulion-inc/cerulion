# SPDX-License-Identifier: AGPL-3.0-only
"""Pose choice and motion for approach, a base class of the package's Mixin.

The standoff pose is solved on the robot's own model (approach_kin): square facing first, then facings tilted off
the stroke when no square pose passes the plane and self checks. The motion runs only through the round one
Executor's planning and streaming (_plan_cart, _plan_joint, _path_check, _cart_move, _joint_move), so the path
check, the guard, the ease after a stop and the retreat history are the round one ones; nothing here streams a joint
target itself. Three ways, the first that plans dry is taken:
- line: one straight tool move to the standoff pose (position and orientation together);
- staged: rise to the travel height, a joint move to a pose above a staging point back along the stroke, a straight
  descent to it, and a straight leg along the stroke into the standoff point, so the arm does not sweep across the
  target on its way in (the way the bench oracle strokes);
- joints: one joint move to the standoff pose.
"""

import math

import numpy as np

from rcpx.bounds_laws import TWO
from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx.context import refusal
from rcpx.control_motion import vec_from_rot
from rcpx.r2.approach_kin import angle_deg, nearest_facing, turn_deg
from rcpx.r2.approach_resolve import pose_ok, starts, reach_tol_m, reach_past_m

SQUARE_DEG = 0.5          # a facing this close counts as met when a start is accepted at once
TILTS_DEG = (10.0, 20.0, 30.0, 45.0, 60.0, 75.0)   # facings tried off along when no square pose passes
AZIMUTHS = 8
LEG_REFUSED = "leg_refused"   # stopped: a later staged leg failed its check after the arm had moved
UP = np.array([0.0, 0.0, 1.0])


def motion_result(ex, result):
    """Read the core's motion record once, accepting both spellings of the stop."""
    r = dict(result)
    sense = r.get("s")
    r.update(stop=r.get("stopped", r.get("stop")), s=ex._sense() if sense is None else sense,
             contact=r.get("contact"), why=r.get("why") or "", bounds=r.get("bounds"),
             periods=r.get("periods"))
    contact = r["contact"]
    if isinstance(contact, dict) and not contact.get("id"):
        ex._record_contact(contact, "approach guard")
    return r


def readable(result, mm=False):
    """Round reader-facing millimetres, newtons and seconds without altering control arithmetic."""
    out = {}
    for key, value in result.items():
        if isinstance(value, dict):
            value = readable(value, mm or key.endswith("_mm"))
        elif isinstance(value, float):
            if key == "value" and result.get("unit") in ("mm", "mm/s", "N"):
                value = round(value, 2)
            elif key == "value" and result.get("unit") == "s":
                value = round(value, 3)
            elif mm or key == "mm" or "_mm" in key or "_n" in key:
                value = round(value, 2)
            elif key.endswith("_s"):
                value = round(value, 3)
        out[key] = value
    return out


def tip_pose(model, q):
    """Tool-tip transform at a joint pose for the cores stream."""
    p, rot = model.fk_tip(q)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rot, p
    return T


class ApproachMotion:
    # ------------------------------------------------------------------ the standoff pose
    """Dry geometry planning and one-stream execution for approach."""
    def _appr_stroke_ok(self, q, along, stroke_m, thing=None):
        """Whether the stroke a touch would make from q (a straight guarded line along `along`, orientation held)
        plans and passes the path check: dry, nothing moves."""
        length = stroke_m + reach_past_m(bounds.of(self), thing, q, along)
        path, pstop, _ = self._plan_cart(q, along * length, None, None, True)
        return not isinstance(path, dict) and pstop == "goal" and self._path_check([q] + path, True) is None

    def _appr_up(self, kin, q_now, facing):
        """The roll target nearest the present hand: (site axis index, direction) for the solve's roll task."""
        R_star = nearest_facing(kin.pose(q_now)[2], kin.axis, facing)
        j = (kin.axis + 1) % 3
        return j, R_star[:, j]

    def _appr_candidates(self, kin, q_now, goal, facing, along, p_t, view_point, seeds, accept, stroke_m,
        up=None, thing=None):
        """Solved standoff poses that pass the checks: [(start index, q, facing error rad, view, stroke plans)].
        accept: stop at the first square one that sees the target and whose stroke plans. up: the roll target
        (_appr_up), or None to leave the roll to the view task."""
        cands, why = [], ""
        for k, q0 in enumerate(seeds):
            q, err, _ = kin.solve(q0, goal, facing, view_point=view_point, up=up)
            if err > reach_tol_m(self, q_now, along, None if thing is None else thing.sigma_p):
                continue
            ok, w = pose_ok(self, q, q_now, stroke=False)
            if not ok:
                why = w
                continue
            ang = math.radians(angle_deg(kin.pose(q)[1], along))
            view = kin.view_of(q, p_t)
            stroke = self._appr_stroke_ok(q, along, stroke_m, thing)
            if not stroke:
                why = why or "the stroke from the standoff pose does not pass the path check"
            cands.append((k, q, ang, view, stroke))
            if accept and stroke and math.degrees(ang) < SQUARE_DEG and (view_point is None or view["in_view"]):
                return cands, why, True
        return cands, why, False

    def _appr_tilts(self, along):
        """Facings tilted off along, smallest tilt first; at each tilt the most downward first (the arm then sits
        higher above the surface under it, the usual reason a square facing fails)."""
        e1 = np.cross(along, UP)
        if np.linalg.norm(e1) < 1e-6:
            e1 = np.cross(along, [1.0, 0.0, 0.0])
        e1 = e1 / np.linalg.norm(e1)
        e2 = np.cross(along, e1)
        out = []
        for tilt in TILTS_DEG:
            t = math.radians(tilt)
            ring = [math.cos(t) * along + math.sin(t) * (math.cos(f) * e1 + math.sin(f) * e2)
                    for f in np.linspace(0.0, 2 * math.pi, AZIMUTHS, endpoint=False)]
            out.append((tilt, sorted(ring, key=lambda n: round(float(n[2]), 9))))
        return out

    def _appr_choose(self, kin, q_now, goal, along, p_t, keep_in_view, extra, stroke_m, thing=None):
        """The standoff pose. Of the hand orientations that face the stroke, the one NEAREST THE PRESENT hand is
        preferred (a roll left free landed the hand upside down): every start is solved with the roll
        held nearest the present one first, and the first fully met pose (square, in view, its stroke plans) is
        taken. Otherwise the roll is freed (the view task may turn it) and those poses join the pool. When no square
        pose passes the plane and self checks, or none has a stroke that plans, facings tilted 10 to 75 deg off
        along are tried, smallest tilt first. Of everything found: a stroke that plans first, then the squarest,
        then in view, then the smallest turn from the present hand, then the least joint travel. Returns (q, facing
        error rad, view, tilted, stroke plans) or (None, why, None, False, False); self.state["approach"]["choice"]
        says how the orientation was chosen."""
        view_point = p_t if keep_in_view else None
        seeds = starts(self, q_now, extra)
        R_now = kin.pose(q_now)[2]
        st = self.state.setdefault("approach", {})
        up = self._appr_up(kin, q_now, along)
        cands, why, done = self._appr_candidates(kin, q_now, goal, along, along, p_t, view_point, seeds, True,
                                                 stroke_m, up, thing)
        held = len(cands)
        if done:
            k, q, ang, view, stroke = cands[-1]
            st["choice"] = {"rule": "nearest_present", "turn_deg": turn_deg(R_now, kin.pose(q)[2])}
            return q, ang, view, False, stroke
        more, w2, _ = self._appr_candidates(kin, q_now, goal, along, along, p_t, view_point, seeds,
            False, stroke_m, thing=thing)
        cands += more
        why = why or w2
        square = len(cands)
        if not any(c[4] for c in cands):
            # the tilted pass solves without the view task (a ring of facings costs too much with it)
            for _tilt, ring in self._appr_tilts(along):
                for n_c in ring:
                    more, w, _ = self._appr_candidates(kin, q_now, goal, n_c, along, p_t, None, seeds[:2], False,
                                                       stroke_m, self._appr_up(kin, q_now, n_c), thing)
                    cands += more
                    why = w or why
                    good = [c for c in more if c[4] and (c[3]["in_view"] or not keep_in_view)]
                    if good:
                        k, q, ang, view, stroke = good[0]
                        st["choice"] = {"rule": "nearest_present_tilted", "turn_deg": turn_deg(R_now, kin.pose(q)[2])}
                        return q, ang, view, True, stroke
                if any(c[4] for c in cands[square:]):
                    break
        if not cands:
            return None, why or "no start reached the standoff point", None, False, False
        pool = [c for c in cands if c[4]] or cands
        best = min(c[2] for c in pool)
        pool = [c for c in pool if c[2] <= best + math.radians(1.0)]
        if view_point is not None and any(c[3]["in_view"] for c in pool):
            pool = [c for c in pool if c[3]["in_view"]]
        turns = {id(c): turn_deg(R_now, kin.pose(c[1])[2]) for c in pool}
        pick = min(pool, key=lambda c: (round(turns[id(c)], 1), float(np.sum(np.abs(c[1] - q_now))), c[0]))
        k, q, ang, view, stroke = pick
        tilted = any(c[1] is q for c in cands[square:])
        held_ids = {id(c) for c in cands[:held]}
        st["choice"] = {"rule": "nearest_present" if id(pick) in held_ids else "nearest_present_of_the_rest",
                        "turn_deg": turns[id(pick)],
                        "why": "" if id(pick) in held_ids else
                        "no pose with the roll held nearest the present hand met every check, so the nearest of "
                        "the poses that did was taken"}
        return q, ang, view, tilted, stroke

    # ------------------------------------------------------------------ motion
    def _appr_line_plan(self, q0, dp, drot):
        """Dry straight line at the speed the core derives."""
        path, pstop, _ = self._plan_cart(q0, dp, drot, None, True)
        if isinstance(path, dict) or pstop != "goal" or not path:
            return None
        return path if self._path_check([q0] + path, True) is None else None

    def _appr_stage_plan(self, kin, s, q_goal, goal, along, thing, intent):
        """Try a staging point behind the stroke, facing or radial direction."""
        n_face = kin.pose(q_goal)[1]
        backs = [along, n_face]
        flat = np.array([along[0], along[1], 0.0])
        if np.linalg.norm(flat) > 0.3:
            backs.append(flat / np.linalg.norm(flat))
        radial = np.array([goal[0], goal[1], 0.0])
        if np.linalg.norm(radial) > 1e-3:
            backs.append(radial / np.linalg.norm(radial))
        for back in backs:
            legs = self._appr_stage_legs(kin, s, q_goal, goal, n_face, np.asarray(back), thing, intent)
            if legs is not None:
                return legs
        return None

    def _appr_stage_legs(self, kin, s, q_goal, goal, n_face, back, thing, intent):
        """Stage from stop distance and tool size, rising only over known space."""
        B = bounds.of(self)
        size = max(kin.size_mm, default=0.0) / 1000.0
        v_travel = B.v_free(q_goal, back)
        stage_m = max(B.d_stop(v_travel, q_goal, back), size, R.BAND * (thing.sigma_p or 0.0))
        stage = goal - stage_m * back
        tol = reach_tol_m(self, s.q, back, thing.sigma_p)
        q_s, e_s, _ = kin.solve(q_goal, stage, n_face)
        if e_s > tol or not pose_ok(self, q_s, s.q, stroke=False)[0]:
            return None
        half_width = size / TWO + B.clearance(v_travel, thing)
        top = B.space.ceiling(s.tip[:2], stage[:2], half_width)
        hi = stage.copy()
        if top is not None:
            hi[2] = max(top, stage[2])
        q_h, e_h, _ = kin.solve(q_s, hi, n_face)
        if e_h > tol or not pose_ok(self, q_h, s.q, stroke=False)[0]:
            return None
        legs, q = [], np.asarray(s.q)
        lift = float(self.model.fk_tip(q_h)[0][2] - s.tip[2])
        if top is not None and lift > 0.005:
            path = self._appr_line_plan(q, UP * lift, None)
            if path is not None:
                legs.append({"kind": "transit", "to": tip_pose(self.model, path[-1]), "line": True,
                             "intent": intent})
                q = path[-1]
        jpath = self._plan_joint(q, q_h, None)
        if isinstance(jpath, dict) or self._path_check([q] + jpath, True) is not None:
            return None
        legs.append({"kind": "transit", "to": q_h, "intent": intent})
        q = q_h
        points = [("descend", stage), ("in", goal)] if top is not None else [("in", goal)]
        for _kind, point in points:
            dp = point - kin.pose(q)[0]
            path = self._appr_line_plan(q, dp, None)
            if path is None:
                return None
            T = tip_pose(self.model, q)
            T[:3, 3] += dp
            legs.append({"kind": "approach", "to": T, "line": True, "target": thing, "intent": intent})
            q = path[-1]
        return legs

    def _appr_move(self, s, q_goal, kin, goal, along, p_t, thing, intent, tol):
        """Choose dry, then execute one stream. A stopped stream has no fallback."""
        T = tip_pose(self.model, q_goal)
        dp = T[:3, 3] - s.tip
        drot = vec_from_rot(T[:3, :3] @ s.R.T)
        if float(np.linalg.norm(dp)) < 2e-4 and float(np.linalg.norm(drot)) < math.radians(0.2):
            return motion_result(self, {"stop": "goal", "s": s, "periods": []}), "none"
        behind = float((kin.pose(s.q)[0] - p_t) @ along) > max(thing.sigma_p or 0.0,
                                                                          bounds.of(self).rigid(s.q, along))
        if not behind and self._appr_line_plan(s.q, dp, drot) is not None:
            legs, path = [{"kind": "approach", "to": T, "line": True, "target": thing, "intent": intent}], "line"
        else:
            legs = self._appr_stage_plan(kin, s, q_goal, goal, along, thing, intent)
            path = "staged"
            if legs is None:
                if behind:
                    return refusal("way_in", "the surface starts past the target face and no staged way in plans",
                                   "move_tip to the front of the target, then approach again"), "none"
                legs, path = [{"kind": "approach", "to": q_goal, "target": thing, "intent": intent}], "joints"
        r = motion_result(self, self._stream(legs, need_m=tol))
        if r.get("refused") is not None and r.get("leg", 0) > 0:
            r["stop"] = LEG_REFUSED
        return r, path
