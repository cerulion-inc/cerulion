# SPDX-License-Identifier: AGPL-3.0-only
"""I20 obstacles: everything known is a volume, and a contact says which part touched.

robot://world gains "obstacles": every contact point (a ball of the contact error plus the touching surface half size; when the side the touching point came from is
known, the half of it behind the touched surface, so leaving the way it came is free and going through is not),
every triangulated or located point (a ball of its own location uncertainty), every located box (a world entry with
data.size_mm), and, after an act that triggers a moving part, the volume that part swings through (from its entry:
hinge, axis, start direction, radius, arc, width) as a keep-out until its within_s has passed. Stale entries (the
world moved) are not obstacles. robot://body gains "mounted_parts": the camera housing: the cuboid the robot's spec
states for its camera body (camera housing_mm) when it states one, else a ball of the carrying link's largest
capsule dimension around the camera pose, kept clear of every thing and marked "assumed until measured" (that ball
is a clearance, not a surface: touched_by then knows the camera only as its lens point).

Motion that expects no contact (move_joints, home, park, standard_look, and move_tip called with guarded false: the
contact stop stays armed on all of them, but their caller states that nothing is in the way) is checked before it runs:
every moving collision capsule (RobotModel.collision_capsules), the tool tip and every mounted part is followed
along the path the primitive will take (the straight joint path, or move_tip's own Cartesian plan), and a part that
would come within its margin of an obstacle it was not already that close to (the margin against a measured thing
is the body's fit error and the cross-track error at rest: B.clearance at speed 0; nothing in it grows with the
planned speed or with the thing's position doubt, which the thing's own volume already holds) refuses the call
with check
seen_obstacle and changes nothing. Only what is measured on both sides refuses: of a touched contact the
check holds the touched point with its own error (a ball of that error), not the volume drawn behind it nor the
tool's size around it, and a mounted part whose size nobody stated or measured (the assumed ball around the
camera) is not judged: both are doubt, which the armed contact stop answers, and both still count for "entered" (a part already inside the volume drawn behind a touched contact, as the tool is
after a press moved the touched part, may go that margin deeper than it stands before that counts). The path is
sampled at B.spacing of the smallest margin in force, as travel of its fastest part; a long path is checked in
consecutive pieces and never thinned. Moves that expect contact may enter an obstacle and report entered: [ids].
approach (it plans its own path) is followed and reports entered too, but is never checked before it runs; called
with guarded false its result says so (obstacles_note).

On a contact or cap stop the result gains touched_by {part, residual, runner_up, ambiguous, tool_excluded, ...}:
the external joint torque at the stop (re-estimated closed loop, obstacles_torque.py) fitted to J_p(q)^T f for
every candidate point, each part then searched for its best point (obstacles_fit.py). A part that is not the tool,
with the tool itself ruled out (tool_excluded), adds part_hit: true and a hint and raises the event part_hit; a part
that is not the tool but fits only about as well as the tool adds part_hit_possible: true and a softer hint. A
confident part that is not the tool records its point in the world contact entry (data.part_point_mm; the obstacle
sits there, point_mm keeps the tip position the guard saw). A tick hook asks the motion to stop with
"part_hit" when the fit (sampled points only, no per-part search: it runs on loud periods a tool surface does not
explain: over B.f_thr or B.tau_thr at the tip's speed) excludes the tool by at least TWO_READ x the torque
estimate's relative error for two control periods in a row; the motion loops end the
move with stopped "part_hit" and the post hook fits that stop again, refined, at the reading the watch decided on.
A loop that does not watch (part_hit_note) only reports it. On an arm with fewer than six joints the watch does not
judge a push the caller asked for (touch, press, impulse, probe_motion): in the bench's oracle sweep every such stop
(4 of 4, the five joint arm sliding a block with the side of its finger) was false by ground truth. When dry joint friction the model lacks explains the
torques as well as the best part (obstacles_fit friction_residual), the tool is not ruled out: an arm with stiff
joints moving through free space reads like a push on a link, so no part_hit and no watch stop then.

While the hand holds or squeezes something (a gripper, grasp, pinch or release call; a part ex.state["grasp"] reports
held; the last close stopped on something and holds it (the gripper result's blocked, with hold true) while the jaws
have not closed further; jaws standing open of their command by more than their own free lag), a contact can load
the hand with a force and a
moment, which no single point force explains: the fit then lets the tool carry any wrench (obstacles_fit
hand_wrench), so on an arm of six joints or fewer the tool is never ruled out, part_hit stays false and touched_by
says holding.

Moving part entry (world entry of kind device or located, data.moving_part, or a robot://devices part carrying the
same keys in base frame): {hinge_mm: [x, y, z], axis: [x, y, z], start_dir: [x, y, z], radius_mm, arc_deg,
width_mm (from the sheet or seen extent), within_s}. An act (touch or press with contact) whose point lies within the part's
radius plus the entries' own position errors of the hinge starts the keep-out. Nothing here reads simulator ground truth.

Files: this one (settings, the hooks, the package exports), obstacles_world.py (the obstacle list, moving parts,
keep-outs and the two resource documents), obstacles_geom.py (shapes and the path check), obstacles_fit.py
(touched_by) and obstacles_torque.py (the closed-loop torque re-estimate).
"""

import collections

import numpy as np

from rcpx.context import refusal
from rcpx.r2 import obstacles_geom as geom
from rcpx.r2 import obstacles_bounds as derived
from rcpx.r2.obstacles_fit import PartFit
from rcpx.r2.obstacles_path import PathMixin
from rcpx.r2.obstacles_torque import TorqueEstimator
from rcpx.r2.obstacles_world import CHECKED, MM, WorldMixin, _num, _rz, _vec  # noqa: F401

ID = "I20"
NAME = "obstacles"
GUARDED = ("move_tip", "touch", "press", "orient_tool", "retreat", "approach")
GRIP_CALLS = ("gripper", "grasp", "pinch", "release")   # calls that squeeze or let go: the hand carries a wrench
PUSHES = ("touch", "press", "impulse", "probe_motion")  # calls that load the tool on purpose (see tick)
SETTINGS = {
    "tol_mm": 1.0,             # ... unless the part was already that close at the start and gets no closer
                               # (a part that starts inside a contact's volume by more than this: margin_mm deeper)

}
HINT_OBSTACLE = "retreat leaves along the way the arm came; expect: contact says the contact is meant"
HINT_PART = "back out the way you came, then roll or pitch so the tool leads"
HINT_POSSIBLE = ("the joint torques fit a part that is not the tool about as well as the tool: if the tool was not "
                 "at the contact, " + HINT_PART)



class _Settings(dict):
    """The package's settings. "margin_mm" is no setting any more (the margin follows the speed of the move); a
    reader outside the package that still asks for it (the bench's obstacle cells) gets the margin a tip move
    straight up at its full speed would take at the present pose, the value robot://world obstacle_check states."""

    def __init__(self, state, values):
        super().__init__(values)
        self._state = state

    def __missing__(self, key):
        if key == "margin_mm":
            return self._state.full_speed_margin_mm()
        raise KeyError(key)


class _State(PathMixin, WorldMixin):
    def __init__(self, ex):
        self.ex = ex
        self.cfg = _Settings(self, SETTINGS)
        hb = derived.housing(derived.of(ex), ex.model)
        self.mounted = [dict(hb, name="camera_housing")]
        self.body = geom.Body(ex.model, ex._tool_bodies, self.mounted)
        # touched_by: a camera body whose size nobody stated or measured is only its lens point there (the ball
        # above is a clearance to keep, not a surface something can be said to have touched)
        fit_parts = [mp if mp.get("shape") != "sphere" else dict(mp, radius=0.0, half=np.zeros(3), size_m=np.zeros(3))
                     for mp in self.mounted]
        self.fit = PartFit(ex.model, ex._tool_bodies, fit_parts, B=derived.of(ex))
        self.torque = TorqueEstimator(ex.model, ex.cfg["physics_dt_s"])
        self.length = float(ex.model.length_scale)
        self.ring = collections.deque(maxlen=1500)  # memory only: newest readings of this call
        self.move_speed = None
        self.move_direction = [0, 0, 1]
        self.path = None
        self.snapshot = None
        self.run = 0
        self.tick_hit = None
        self.call = None
        self.hold_why = None
        self.jaw_block = None      # the gripper result's blocked of the last close that holds (None: none holds)
        self.tool_quiet = 0
        self.quiet_qd = None
        self.fric_seen = None      # 2 x n: least friction-signed quiet reading per joint and way (see _note_quiet)
        self.keepouts = {}
        self.new_keepouts = []
        self.problems = []
        self.counts = {"refused": 0, "touched_by": 0, "part_hit": 0, "entered": 0, "keep_outs": 0}

    # ------------------------------------------------------------------------------------------ hooks
    def pre(self, name, kwargs):
        ex = self.ex
        self.ring.clear()
        self.run, self.tick_hit, self.snapshot, self.new_keepouts, self.path = 0, None, None, [], None
        self.fric_seen = np.full((2, ex.model.n), np.inf)
        s = ex._sense()
        self.call = name
        self.tool_quiet = 0
        self.fit.quiet_residual = 0
        self.hold_why = self.holding(s)
        self.quiet_qd = s.qd.copy()
        self.ring.append(self._reading(s))
        checked = name in CHECKED or (name == "move_tip" and not bool(kwargs.get("guarded", True)))
        if not (checked or name in GUARDED):
            return None
        self.path = [s.q.copy()]
        pairs = self.obstacles()
        self.snapshot = pairs
        B = derived.of(ex)
        # the margin against a measured thing is the body's fit error and the tracking error at rest: no term
        # grows with the planned speed (it is read at rest) or with the thing's position doubt (its own volume
        # already holds its measured error), so the same path is refused or not at every speed
        self.things = [B.thing({"id": pub.get("from") or pub["id"], "sigma_p": 0.0,
                                "extent": derived.field(self.thing(B, pub), "extent")}) for pub, _ in pairs]
        self.margin_cache = {}
        self.segment_speeds = {}
        self.planning_path = None
        self.move_speed = 0.0
        self.path_spacing = self.body.step
        if not checked or not pairs:
            return None
        path = self.planned(name, kwargs, s)
        if path is None:
            return None
        self.planning_path = np.asarray(path, dtype=float)
        p0, p1 = ex.model.fk_tip(path[0])[0], ex.model.fk_tip(path[-1])[0]
        delta = p1 - p0
        self.move_direction = delta / np.linalg.norm(delta) if np.linalg.norm(delta) > 0 else np.array([0, 0, 1])
        margins = self.measured_only(self.path_margins(B, pairs), self.shown_gone(B, pairs))
        spacing = self.sample_spacing(B, path, pairs)
        self.path_spacing = spacing
        worst, _ = geom.check_path(self.body, path, [self.measured_shape(B, *p) for p in pairs], margins,
                                   self.cfg["tol_mm"] / MM,
                                   gives=[p[0].get("kind") == "contact" for p in pairs], spacing=spacing)
        if worst is None:
            return None
        pub = pairs[worst["obstacle"]][0]
        near = max(0.0, worst["clearance"] * MM)
        what = {"contact": "contact", "triangulated": "triangulated point", "swept": "the swept volume of"}.get(
            pub["kind"], pub["kind"].replace("_", " "))
        thing = self.things[worst["obstacle"]]
        item = derived.margin_item(B, 0.0, thing, worst["part"])
        margin_mm = worst["margin"] * MM
        why = (f"passes within {near:.0f} mm of {what} {pub['id']}, a measured thing: the {worst['part']} "
               f"({'tool' if worst['kind'] == 'tool' else 'not the tool'}) at {worst['pct']} % of the path "
               f"(the margin kept is {margin_mm:g} mm: that body's fit error and the arm's tracking error at "
               f"rest; {item['source']}: {item['evidence']})")
        self.counts["refused"] += 1
        return {**refusal("seen_obstacle", why, "retreat()"), "stopped": "refused",
                # speed_mm_s: the speed the margin is read at (the arm at rest); no speed clears a measured thing
                "seen_obstacle": {"margin_mm": margin_mm, "speed_mm_s": 0.0, "clearing_speed_mm_s": None,
                                  "bounds": item, "obstacle": pub, "part": worst["part"], "part_kind": worst["kind"],
                                  "clearance_mm": round(worst["clearance"] * MM, 1), "at_pct": worst["pct"],
                                  "start_clearance_mm": round(worst["start"] * MM, 1)}}

    def measured_shape(self, B, pub, shape):
        """The shape the pre check holds for an obstacle: of a touched contact the touched point with its own
        error (a ball of that error around it); every other obstacle as it is drawn."""
        if pub.get("kind") != "contact":
            return shape
        entry = self.ex.world.get(pub.get("from") or pub["id"]) or {}
        return {"shape": "sphere", "c": shape["c"], "R": derived.uncertainty_m(B, entry)}

    def shown_gone(self, B, pairs):
        """The touched contacts of the list that are measured no longer (the design rulings: only what is measured refuses): a
        look of the scene as it stands (taken after the last act that changed the scene, so after the touch)
        shows the touched point, with its own error, free by more than the look's stated error: the thing that
        was touched there has moved. Indices into pairs."""
        try:
            from rcpx.r2 import see_free
        except Exception:
            return []
        idx = [i for i, (pub, _) in enumerate(pairs) if pub.get("kind") == "contact"]
        if not idx or not see_free.looks_now(self.ex):
            return []
        balls = [self.measured_shape(B, *pairs[i]) for i in idx]
        return [i for i, ball in zip(idx, balls) if see_free.shown_free(self.ex, [ball["c"]], float(ball["R"]))[0][0]]

    def measured_only(self, margin, gone=()):
        """The margins of the pre check with a mounted part whose size nobody stated or measured left out (its
        ball is a clearance somebody assumed, not a body: it refuses nothing), and with the touched contacts a
        look has since shown gone (shown_gone) left out."""
        skip = [i for i, name in enumerate(self.body.names)
                if any(mp["name"] == name and str(mp.get("evidence", "")).startswith("assumed") for mp in self.mounted)]
        gone = list(gone)
        if not skip and not gone:
            return margin

        def cut(m):
            m = np.array(np.broadcast_to(np.asarray(m, dtype=float), (len(self.body.names), len(self.things))))
            m[skip, :] = -np.inf
            m[:, gone] = -np.inf
            return m
        out = lambda q: cut(margin(q))                                           # noqa: E731
        out.for_step = lambda q0, q1: cut(margin.for_step(q0, q1))
        out.for_points = lambda q0, q1, p0, p1: cut(margin.for_points(q0, q1, p0, p1))
        return out

    def planned(self, name, kwargs, s):
        """The joint path the primitive would take from the present pose (start first), or None to let the
        primitive answer for itself (bad arguments, a plan it refuses)."""
        ex, model = self.ex, self.ex.model
        if name == "move_tip":
            R = ex._rot_of(kwargs.get("frame", "base"), s)
            asked = _vec([kwargs.get("dx", 0.0), kwargs.get("dy", 0.0), kwargs.get("dz", 0.0)])
            if R is None or asked is None:
                return None
            sp = _num(kwargs.get("speed_mm_s"))
            direction = R @ asked
            norm = float(np.linalg.norm(direction))
            direction = direction / norm if norm > 0 else np.array([0, 0, 1])
            speed = derived.of(ex).v_free(s.q, direction) if sp is None else sp / MM
            path, _, _ = ex._plan_cart(s.q, R @ asked / MM, None, speed, bool(kwargs.get("keep_orientation", True)))
            if isinstance(path, dict):
                return None
            return [s.q] + list(path)
        if name == "move_joints":
            if "q_deg" not in kwargs:
                return None
            q1, bad = ex._parse_q(kwargs["q_deg"], s.q)
            if bad is not None:
                return None
        elif name == "home":
            q1 = model.q_home.copy()
        elif name == "park":
            q1 = ex._named_poses()["park"]
        else:
            q1 = ex._standard_q()
        q1 = np.asarray(q1, dtype=float)
        if q1.shape != s.q.shape or not np.all(np.isfinite(q1)) or not model.within_limits(q1, 1e-6):
            return None
        return geom.straight_joint_path(s.q, q1, self.length)

    def holding(self, s):
        """Why the hand may carry a load of its own in this call (None when nothing says so): a call that squeezes
        or lets go, a part another package reports held (ex.state["grasp"]["held"]), the last close stopped on
        something and holds it (its result's blocked) while the jaws stand no more than jaw_lag_frac closer than
        where it stopped, or jaws more open than their command by over jaw_lag_frac of their span (the measured
        opening against the commanded one; a contact close commands only a small squeeze past where it stopped)."""
        if self.call in GRIP_CALLS:
            return f"the call is {self.call}"
        g = self.ex.state.get("grasp")
        if isinstance(g, dict) and g.get("held"):
            return "the hand reports a part held"
        gr = getattr(s, "gripper", None)
        frac = _num(gr.get("open_frac")) if isinstance(gr, dict) else None
        at = _num((self.jaw_block or {}).get("at_open_frac"))
        if at is not None and (frac is None or frac > at - derived.jaw_lag_frac(derived.of(self.ex), self.ex.model)[0]):
            return (f"the jaws stand where the last close stopped on something ({100.0 * at:.0f} % of their span) "
                    "and hold it")
        cmd = _num(getattr(self.ex, "_gripper_cmd", None))
        if frac is not None and cmd is not None and frac - cmd > derived.jaw_lag_frac(derived.of(self.ex), self.ex.model)[0]:
            return (f"the jaws stand {100.0 * (frac - cmd):.0f} % of their span more open than commanded: something "
                    "is between them")
        return None

    def _reading(self, s):
        """(t, q, qd, executor ext, servo target and torque share over the period that ended at this reading)."""
        ex = self.ex
        r = ex.backend.read()
        qt = r.get("q_target")
        qt = ex._q_cmd if qt is None else qt
        frac = r.get("torque_frac")
        return (float(s.t), s.q.copy(), s.qd.copy(), np.asarray(s.ext, dtype=float).copy(),
                None if qt is None else np.asarray(qt, dtype=float).copy(),
                float(ex._frac_cmd if frac is None else frac))

    def _ext(self, ring, i):
        """The closed-loop external torque at ring[i] (from ring[i - 1]); the executor's estimate when there is
        no previous reading."""
        cur = ring[i]
        if i > 0:
            prev = ring[i - 1]
            est = self.torque.estimate(prev[:3], cur[:3], cur[4], cur[5], start=cur[3])
            if est is not None:
                return est
        return cur[3]

    def _note_quiet(self):
        """A quiet reading bounds the friction the model lacks: a joint turning one way through the period carried
        no more than its friction-signed reading then (kept per joint and way for this call)."""
        if len(self.ring) < 2 or self.fric_seen is None:
            return
        on, sg = PartFit.turning((self.ring[-2][2], self.ring[-1][2]))
        e = sg * self.ring[-1][3] / np.asarray(self.ex.torque_limit, dtype=float)
        for k, way in ((0, 1.0), (1, -1.0)):
            m = on & (sg == way)
            self.fric_seen[k, m] = np.minimum(self.fric_seen[k, m], np.maximum(e[m], 0.0))

    def tick(self, s):
        self.ring.append(self._reading(s))
        if self.hold_why is None:
            self.hold_why = self.holding(s)
        if self.path is not None and float(np.max(np.abs(s.q - self.path[-1]))) * self.length >= self.path_spacing:
            self.path.append(s.q.copy())
        B = derived.of(self.ex)
        velocity = self.ex.model.jac(s.q)[:3] @ s.qd
        v = float(np.linalg.norm(velocity))
        loud = s.Fmag > B.f_thr(v) or np.any(np.abs(s.ext) > np.asarray(B.tau_thr(v)))
        if not loud:
            self.run = 0
            self.quiet_qd = s.qd.copy()
            self._note_quiet()
            quiet = self.fit.tool_residual_quick(s.q, s.ext)
            if quiet is not None:
                self.tool_quiet = max(self.tool_quiet, quiet)
                self.fit.quiet_residual = self.tool_quiet
            return None
        if self.hold_why is not None and self.ex.model.n <= 6:
            self.run = 0       # a wrench through the hand fits any reading on six joints or fewer (away from a
            return None        # singular pose): the fit would not rule the tool out, so it is not run
        if self.call in PUSHES and self.ex.model.n < 6:
            self.run = 0       # a push loads the tool on purpose; five joint torques cannot tell that load plus
            return None        # the joints' own friction from a push on a link, so the watch does not judge it
        quick = self.fit.tool_residual_quick(s.q, s.ext)
        if quick is not None and quick < 0.5 * derived.watch_gap(B):
            self.run = 0       # a tool surface explains the executor's reading to under half the gap; the fit
            return None        # (on the re-estimate, within a few percent of it) would not count it either
        ring = self.ring
        tb = self.fit.fit(s.q, self._ext(ring, len(ring) - 1), self.quiet_qd, hand_wrench=self.hold_why is not None,
                          qd_period=(ring[-2][2], ring[-1][2]) if len(ring) >= 2 else None,
                          friction_seen=self.fric_seen)
        gap = None if tb is None or tb["tool_residual"] is None else tb["tool_residual"] - tb["residual"]
        if (tb is None or not tb["tool_excluded"] or tb["force_n"] < B.f_thr(v)
                or (gap is not None and gap < derived.watch_gap(B))):
            self.run = 0
            return None
        self.run += 1
        if self.run < 2:
            return None
        if self.tick_hit is None:
            self.tick_hit = dict(tb, t_s=round(float(s.t), 3))
        return "part_hit"

    def _approach_qd(self, ring, idx):
        """Joint speeds at the last reading before the push began to rise (under a tenth of its size at idx): the
        motion that brought the part to the contact, not the sliding the push caused once it began."""
        w = 1.0 / np.asarray(self.ex.torque_limit, dtype=float)
        loud = float(np.linalg.norm(w * ring[idx][3]))
        for j in range(idx - 1, -1, -1):
            if float(np.linalg.norm(w * ring[j][3])) < 0.1 * loud:
                return ring[j][2]
        return ring[0][2]

    def touched_by(self, result, t=None):
        """The fit at the reading the stop was decided on (t, else the contact's t_s), else the loudest reading."""
        if not self.ring:
            return None
        ring = list(self.ring)
        c = result.get("contact") if isinstance(result.get("contact"), dict) else None
        if t is None:
            t = _num(c.get("t_s")) if c else None
        idx = None
        if t is not None:
            near = [i for i, r in enumerate(ring) if abs(r[0] - t) <= derived.of(self.ex).dt() / 2]
            idx = near[-1] if near else None
        if idx is None:
            w = 1.0 / np.asarray(self.ex.torque_limit, dtype=float)
            idx = int(np.argmax([float(np.linalg.norm(w * r[3])) for r in ring]))
        q = ring[idx][1]
        tb = self.fit.fit(q, self._ext(ring, idx), self._approach_qd(ring, idx), refine=True,
                          hand_wrench=self.hold_why is not None,
                          qd_period=(ring[idx - 1][2], ring[idx][2]) if idx > 0 else None,
                          friction_seen=self.fric_seen)
        if tb is not None:
            tb["t_s"] = round(float(ring[idx][0]), 3)
            if self.hold_why is not None:
                tb["holding"] = self.hold_why
        return tb

    def record(self, name, result, tb):
        """Write the part and its point into the world contact entry (a cap stop gets one)."""
        world = self.ex.world
        c = result.get("contact") if isinstance(result.get("contact"), dict) else {}
        if c.get("on"):     # round six (C8): a contact made while holding is kept on the hold, never in the world
            return None
        eid = c.get("id") or result.get("id")
        data = {"touched_by": tb["part"], "touched_by_kind": tb["kind"], "touched_by_ambiguous": tb["ambiguous"]}
        if tb.get("approach_dir") is not None:      # the touching point came from this side: it is free
            data["approach_normal"] = [-x for x in tb["approach_dir"]]
        if tb["force_n"] >= derived.of(self.ex).f_thr():
            data["fit_normal"] = tb["normal_est"]   # the push's direction (leans by the friction angle)
        if tb["kind"] != "tool" and not tb["ambiguous"]:
            data.update(part_point_mm=tb["point_mm"], part_normal=tb["normal_est"],
                        part_spread_mm=tb["point_spread_mm"])
        elif tb.get("approach_dir") is not None and tb["kind"] != "tool":
            data.pop("approach_normal", None)       # the approach of an unsure part says nothing about the tip
        if eid is not None and world.get(eid) is not None:
            world.update(eid, data=data)
            return eid
        p = tb["point_mm"] if not tb["ambiguous"] else (c.get("point_mm") or tb["point_mm"])
        e = world.add("contact", "contact", point_mm=p, source=f"{name} {result.get('stopped')} stop (obstacles fit)",
                      data=dict(data, normal_est=tb["normal_est"], force_n=tb["force_n"]))
        return e.get("id")

    def post(self, name, kwargs, result):
        if not isinstance(result, dict) or "refused" in result:
            return None
        if name == "gripper":   # a close that stopped on something and holds it; any other gripper call ends that
            b = result.get("blocked")
            self.jaw_block = dict(b) if isinstance(b, dict) and result.get("hold", True) else None
        add = {}
        watch_stop = result.get("stopped") == "part_hit" and self.tick_hit is not None
        if result.get("stopped") in ("contact", "cap") or watch_stop:
            # a watch stop is fitted again, refined, at the reading the watch decided on
            tb = self.touched_by(result, t=self.tick_hit["t_s"] if watch_stop else None)
            if tb is None and watch_stop:
                tb = dict(self.tick_hit)
            if tb is not None:
                self.counts["touched_by"] += 1
                if watch_stop:
                    tb["watch"] = {k: self.tick_hit.get(k) for k in ("part", "residual", "tool_residual", "t_s")}
                tb["contact_id"] = self.record(name, result, tb)
                add["touched_by"] = tb
                # part_hit only when the tool itself is ruled out (the rule the tick watch uses); a part that fits
                # best while the tool fits about as well is a possible part hit, said as such
                add["part_hit"] = bool(tb["tool_excluded"])
                if tb["kind"] != "tool" and not tb["tool_excluded"]:
                    add["part_hit_possible"] = True
                    add["hint"] = (HINT_POSSIBLE if not result.get("hint") else
                                   f"{result['hint']}; {HINT_POSSIBLE}")
                if watch_stop and not tb["tool_excluded"]:
                    add["part_hit_note"] = ("the part_hit watch stopped the move (two readings in a row ruled the "
                                            "tool out), but the refined fit at the stop does not rule it out")
        elif self.tick_hit is not None:
            add["touched_by"] = self.tick_hit
            add["part_hit"] = True
            add["part_hit_note"] = ("for two control periods in a row the joint torques fit a part that is not the "
                                    "tool; a move or push stops on that (stopped part_hit), a call whose loop "
                                    "does not watch only reports it here")
        if add.get("part_hit"):
            self.counts["part_hit"] += 1
            add["hint"] = HINT_PART if not result.get("hint") else f"{result['hint']}; {HINT_PART}"
            self.ex.note_event("part_hit", primitive=name, part=add["touched_by"]["part"],
                               ambiguous=add["touched_by"]["ambiguous"])
        if self.path is not None:
            q_end = self.ex._sense().q
            if float(np.max(np.abs(q_end - self.path[-1]))) > 1e-9:
                self.path.append(q_end.copy())
        if self.snapshot and self.path is not None and len(self.path) >= 2:
            self.planning_path = np.asarray(self.path)
            self.segment_speeds = {}
            _, entered = geom.check_path(self.body, self.path, [p[1] for p in self.snapshot],
                                         self.path_margins(derived.of(self.ex), self.snapshot), self.cfg["tol_mm"] / MM, spacing=self.path_spacing)
            if entered:
                self.counts["entered"] += 1
                add["entered"] = [self.snapshot[j][0]["id"] for j in entered]
        if name == "approach" and self.snapshot and not bool(kwargs.get("guarded", True)):
            add["obstacles_note"] = ("approach plans its own path, so when it expects no contact it was not checked against "
                                     "robot://world obstacles before it ran; entered lists any it went into")
        if self.new_keepouts:
            add["keep_out"] = self.new_keepouts
        if not add:
            return None
        return {**result, **add}

# ---------------------------------------------------------------------------------------------- package exports
def setup(ex):
    st = _State(ex)
    ex.state[NAME] = st
    ex.add_pre(st.pre)
    ex.add_post(st.post)
    ex.add_tick(st.tick)
    ex.on_event("device_act", st.on_act)


def _st(ex):
    st = getattr(ex, "state", {}).get(NAME)
    return st if isinstance(st, _State) else None


def _world_patch(ex, session, content):
    st = _st(ex)
    return content if st is None else st.world_doc(content)


def _body_patch(ex, session, content):
    st = _st(ex)
    return content if st is None else st.body_doc(content)


RESOURCE_PATCH = {"robot://world": _world_patch, "robot://body": _body_patch}

_CHECK_NOTE = ("Refused with seen_obstacle when its straight joint path would bring any part of the arm within its "
               "clearance margin (it grows with the speed of the move) of a known contact or obstacle (robot://world obstacles); retreat leaves along the way the arm came, and expect: contact says the contact is meant.")
TOOLS = [
    {"extends": "move_joints", "args": {}, "note": _CHECK_NOTE},
    {"extends": "home", "args": {}, "note": _CHECK_NOTE},
    {"extends": "park", "args": {}, "note": _CHECK_NOTE},
    {"extends": "standard_look", "args": {},
     "note": "Refused with seen_obstacle when the straight joint path to the standard pose would bring any part of "
             "the arm within its clearance margin (it grows with the speed of the move) of a known contact or obstacle (robot://world obstacles); retreat leaves along the way the arm came."},
    {"extends": "move_tip", "args": {},
     "note": "For a move expecting no contact the planned path is checked against robot://world obstacles (seen_obstacle); a "
             "move expecting contact may enter one and reports entered; a contact, cap or part_hit stop reports touched_by "
             "(which part of the robot the joint torques say touched) and part_hit true when they rule the tool "
             "out."},
    {"extends": "touch", "args": {},
     "note": "A contact reports touched_by, the part of the robot the joint torques say touched (tool, "
             "camera_housing or a link); part_hit true means the torques rule the tool out, part_hit_possible true "
             "means it may not have been the tool."},
]

PRACTICES = ("After a contact, a move whose path is checked before it runs is refused with seen_obstacle when "
             "the path would pass within its clearance margin of the touched point (the margin grows with the "
             "speed); retreat leaves along the way the arm came. touched_by names the part that touched; part_hit "
             "true means it was not the tool.")

__all__ = ["ID", "NAME", "PRACTICES", "RESOURCE_PATCH", "TOOLS", "setup"]
