# SPDX-License-Identifier: AGPL-3.0-only
"""The speed, margin and sampling of a checked path for the obstacles package: a mixin over the package
state (obstacles._State). It reads self.ex, self.cfg, self.body, self.length, self.things, self.margin_cache,
self.segment_speeds, self.planning_path, self.move_speed and self.path_spacing, which the pre hook sets.

One law for Cartesian and joint paths: between two samples the time is the largest joint step over that joint's
rated speed (B.qd_max), a part's speed is its travel over that time, capped by a stated speed_mm_s; the margin of a
part against a thing is B.clearance at that speed (A4.3); the path is sampled so that no part travels more than
B.spacing of its own margin between two samples (A3), in consecutive pieces, never thinned. Nothing here is a typed
speed, margin or count, and nothing reads simulator ground truth.
"""

import numpy as np

from rcpx.r2 import obstacles_bounds as derived
from rcpx.r2 import obstacles_geom as geom
from rcpx.r2.obstacles_world import MM


class PathMixin:
    def full_speed_margin_mm(self):
        """The margin (mm) of the tip against a thing with no error of its own, for a move straight up from the
        present pose at its full speed: what robot://world obstacle_check shows as margin_mm."""
        B = derived.of(self.ex)
        return derived.margin(B, B.v_free(self.ex._sense().q, np.array([0.0, 0.0, 1.0])), None, "tip") * MM

    def sample_spacing(self, B, path, pairs):
        """The sample spacing of a checked path, as travel of its fastest point (A3: between two samples no part
        travels more than B.spacing of its own margin, half of it). A part moving at v_i while the fastest part moves
        at v_max travels spacing x v_i / v_max between two samples, so the spacing is the least, over the stretches
        of the path and the parts that move, of B.spacing(margin_i) x v_max / v_i. A part that stands still asks for
        nothing (its margin is not in force). With no part moving: the body's own sampling step."""
        best = None
        previous, previous_points = None, None
        for q in geom.iter_path(path, self.length, self.body.step, self.body):
            points = self.body.points(q)
            if previous is not None:
                speeds = derived.part_speeds(B, previous, q, previous_points, points, self.body.starts, self.move_speed)
                fastest = float(np.max(speeds))
                if fastest > 0:
                    least = np.min(self.margins(B, q, pairs, speeds=speeds), axis=1)   # per part, over the things
                    for v, m in zip(speeds, least):
                        if v > 0:
                            step = B.spacing(float(m)) * fastest / float(v)
                            best = step if best is None else min(best, step)
            previous, previous_points = q, points
        return best if best is not None and best > 0 else self.body.step

    def thing(self, B, pub):
        ref = pub.get("from") or pub["id"]
        entry = self.ex.world.get(ref) or {}
        value = B.thing(ref)
        return B.thing({"id": ref, "sigma_p": max(derived.field(value, "sigma_p"),
                         derived.uncertainty_m(B, entry)), "extent": derived.field(value, "extent")})

    def speed_at(self, B, q):
        path = self.planning_path
        if path is None or len(path) < 2:
            return np.zeros(len(self.body.names))
        index = min(int(np.argmin(np.sum((path - q)**2, axis=1))), len(path) - 2)
        if index not in self.segment_speeds:
            q0, q1 = path[index:index + 2]
            speeds = self.speeds(B, q0, q1)
            self.segment_speeds[index] = speeds
        return self.segment_speeds[index]

    def speeds(self, B, q0, q1):
        return derived.part_speeds(B, q0, q1, self.body.points(q0), self.body.points(q1),
                                   self.body.starts, self.move_speed)

    def path_margins(self, B, pairs):
        def margin(q):
            return self.margins(B, q, pairs)
        margin.for_step = lambda q0, q1: self.margins(B, q1, pairs, previous=q0)
        margin.for_points = lambda q0, q1, p0, p1: self.margins(B, q1, pairs,
            speeds=derived.part_speeds(B, q0, q1, p0, p1, self.body.starts, self.move_speed))
        return margin

    def margins(self, B, q, pairs, previous=None, speeds=None):
        if speeds is None:
            speeds = self.speed_at(B, q) if previous is None else self.speeds(B, previous, q)
        base = np.empty(len(self.body.names))
        for i, (part, speed) in enumerate(zip(self.body.names, speeds)):
            key = part, float(speed)
            if key not in self.margin_cache:
                self.margin_cache[key] = B.clearance(float(speed), thing={"sigma_p": 0}, body=part)
            base[i] = self.margin_cache[key]
        # Only the position band differs across obstacles; body and speed terms are cached.
        return base[:, None] + derived.position_bands(self.things)[None, :]

    def verified_clearing_speed(self, B, path, pairs, replan=None):
        """The largest speed found at which this same path passes this same check, or None. The solved speed
        (clearing_speed) is run through the check again at its own, finer sampling; while it still fails, the next
        candidate is the speed solved from that run, and at most half the last one. So the ready call the refusal
        names is one the check lets through."""
        shapes = [p[1] for p in pairs]
        gives = [p[0].get("kind") == "contact" for p in pairs]
        kept = (self.move_speed, self.path_spacing, self.segment_speeds, self.planning_path)
        speed = self.clearing_speed(B, path, pairs)
        try:
            for _ in range(8):                                    # numeric: tries
                if speed is None or not speed > 0:
                    return None
                self.move_speed, self.segment_speeds = float(speed), {}
                if replan is not None:
                    path = replan(float(speed))
                    if path is None:
                        return None
                    self.planning_path = np.asarray(path, dtype=float)
                self.path_spacing = self.sample_spacing(B, path, pairs)
                worst, _ = geom.check_path(self.body, path, shapes, self.path_margins(B, pairs),
                                           self.cfg["tol_mm"] / MM, gives=gives, spacing=self.path_spacing)
                if worst is None:
                    return float(speed)
                solved = self.clearing_speed(B, path, pairs)
                speed = None if solved is None else min(float(solved), 0.5 * float(speed))   # numeric: halving
            return None
        finally:
            self.move_speed, self.path_spacing, self.segment_speeds, self.planning_path = kept

    def clearing_speed(self, B, path, pairs):
        answer = self.move_speed
        if answer is None:
            answer = max(float(np.max(self.speed_at(B, q))) for q in path)
        obstacles = [p[1] for p in pairs]
        first = self.body.clearances(path[0], obstacles)
        least = first.copy()
        for q in geom.iter_path(path, self.length, self.path_spacing, self.body):
            least = np.minimum(least, self.body.clearances(q, obstacles))
        # A speed is solved once per part and obstacle, at its least clearance.
        for i, part in enumerate(self.body.names):
            for j, thing in enumerate(self.things):
                c = least[i, j]
                if c >= first[i, j] - self.cfg["tol_mm"] / MM:
                    continue
                speed = derived.clearing_speed(B, c, thing, part, answer)
                if speed is None:
                    return None
                answer = min(answer, speed)
        return answer
