# SPDX-License-Identifier: AGPL-3.0-only
"""What a look covers (the parts in the look itself): the patch of the surface the robot stands on that
the frame shows (floor_view_mm) and the part of the frame the robot's own hand and tool hide (hidden_by_hand_px).
Both come from the body model at the look's joint pose and the installation's floor height; nothing reads the
picture or the simulator. A survey (the packages owner's plug-in) calls look_footprint for each of its poses.
No robot is named here.
"""

import numpy as np

MM = 1000.0
EDGE_SAMPLES = 2      # points per frame edge projected onto the floor (the corner, then the edge's middle)
CAPSULE_SAMPLES = 6   # points along each tool capsule projected into the frame


class ViewMixin:
    def _floor_hit(self, q, u, v):
        """Where the ray through pixel (u, v) meets the surface the robot stands on (m, frame B), or None."""
        o, dv = self.model.pixel_ray(q, u, v)
        z = float(self.cfg["floor_z_m"])
        if dv[2] > -1e-6:
            return None
        t = (z - float(o[2])) / float(dv[2])
        if t <= 0.0:
            return None
        p = o + t * dv
        if self._floor_at(p) != z:   # over the mount the surface is the mount top, not the floor
            t = -float(o[2]) / float(dv[2])
            p = o + t * dv if t > 0.0 else None
        return p

    def look_footprint(self, q):
        """floor_view_mm: the floor patch the frame shows at joint pose q: its outline (points where the frame's
        edges meet the floor, clockwise from the top-left corner; edges that look above the horizon add none), the
        floor point at the frame centre, its nearest and farthest distance from the base axis, and whether part of
        the frame looks above the horizon. None when no part of the frame looks at the floor."""
        W, H = self.model.cam_w, self.model.cam_h
        k = EDGE_SAMPLES
        rim = ([(W * i / k, 0.0) for i in range(k)] + [(W, H * i / k) for i in range(k)]
               + [(W * (k - i) / k, H) for i in range(k)] + [(0.0, H * (k - i) / k) for i in range(k)])
        pts, sky = [], False
        for u, v in rim:
            p = self._floor_hit(q, u, v)
            if p is None:
                sky = True
            else:
                pts.append(p)
        if not pts:
            return None
        c = self._floor_hit(q, W / 2.0, H / 2.0)
        r = [float(np.hypot(p[0], p[1])) for p in pts]
        return {"z_mm": round(float(self.cfg["floor_z_m"]) * MM, 1),
                "outline": [[round(float(p[0]) * MM), round(float(p[1]) * MM)] for p in pts],
                "centre": None if c is None else [round(float(c[0]) * MM), round(float(c[1]) * MM)],
                "radius_mm": [round(min(r) * MM), round(max(r) * MM)], "above_horizon": sky}

    def hidden_by_hand(self, q):
        """hidden_by_hand_px: the bounding box, in the frame, of what the hand and tool cover at joint pose q (their capsules, each
        grown by its radius at its depth), clipped to the frame, and its share of the frame. None when they are
        out of view."""
        W, H, f = self.model.cam_w, self.model.cam_h, self.model.f_px
        box = [np.inf, np.inf, -np.inf, -np.inf]
        for name, p0, p1, r in self.model.collision_capsules(q)[1:]:
            if name not in self._tool_bodies and not self._hand_body(self.model.mj.body(name).id):
                continue
            for t in np.linspace(0.0, 1.0, CAPSULE_SAMPLES):
                u, v, depth = self.model.project(q, p0 + t * (p1 - p0))
                if not depth > 1e-3:
                    continue
                g = f * r / depth
                box = [min(box[0], u - g), min(box[1], v - g), max(box[2], u + g), max(box[3], v + g)]
        if not np.isfinite(box[0]):
            return None
        u0, v0, u1, v1 = max(0.0, box[0]), max(0.0, box[1]), min(float(W), box[2]), min(float(H), box[3])
        if u1 <= u0 or v1 <= v0:
            return None
        return {"box": [round(u0), round(v0), round(u1), round(v1)],
                "share": round((u1 - u0) * (v1 - v0) / float(W * H), 3)}
