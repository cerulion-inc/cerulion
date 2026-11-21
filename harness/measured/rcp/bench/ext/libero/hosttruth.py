# SPDX-License-Identifier: AGPL-3.0-only
"""Truth for the gates' instruments, served on the SCORING socket only. Runs inside the env host
(bench/ext/libero/envhost.py), on its numpy, mujoco, model and data handles: this file imports nothing. None of
these ops is on the public socket, and no rcpx code may call them: they score, and they give the gates' operator
the pixels of a thing that a look shows.

Scoring ops (methods of TruthMixin, a base of the host):
  things   the simulator's own object model of the episode, static: every object, fixture and region with its root
           body and its own joints, the body tree (each body with the mass of itself and all it carries), gravity,
           the geom, joint and site tables, the task file's parsed goal,
           things of interest, regions and init statements, the render groups and the robot's name prefixes.
  segment  {camera, width, height, path?, prove?}: what the colour picture of that camera SHOWS at this step, per
           pixel (objtype, objid), rows in true orientation as the colour render's are, saved as an int32 array.
           The simulator's segmentation draws every surface as opaque; a surface the colour render does not draw
           (alpha 0) or draws see-through (alpha under one half) is taken out of the picture (moved to a render
           group that is switched off) and the segmentation is rendered again until none is left; everything is
           put back. Saved beside: *_all.npy (the simulator's first answer) and *_drawn.npy (every surface the
           picture DRAWS at a pixel, front to back, down to the first that gives more than half of its colour).
           The same procedure as the pointing calibration's host. prove true adds the sha256 of the look's own colour
           render before, with the invisible
           surfaces out, and after. A render is no control step.
  ray      {camera, width, height, pixels: [[u, v], ...]}: for each pixel (column, row; the ray through its
           middle) the first surface the picture draws with more than half of its colour, by a ray cast through
           the model's geoms of the render groups that are on: its geom, its body, the range along the ray, the
           depth along the optical axis and the point (world, m); and "front": the first surface the picture
           draws at all, however faintly (a see-through pane: the depth picture holds ITS depth). through: n adds
           "behind": the next n surfaces the same ray meets (a shell drawn one-sided shows its far side through
           its near one, and a ray meets the near one first). An independent path from the segmentation render
           (no rasteriser), for the gates' test of the masks.
  shapes   {bodies: [id, ...]}: the geoms of those bodies as they stand now: type, size, world pose, group,
           whether they collide. For the test's reading of a part's true shape.
  kin      a kinematic copy of the scene with the arm put at given joints (sc_kin): what would be inside what,
           where the pads would stand and what would lie between them; and how near given points are to named
           bodies. For the census. No step runs and the episode's data is not touched.
  peaks    {from?, reset?}: the raw reading, kept by PairWatch below for every control step since
           the reset: per pair of bodies the largest NET contact force (the size of the vector sum of the pair's
           contact points) over the step's physics steps and the net at its last physics step; per body the
           largest whole push (the size of the sum over everything it touches); and where every free thing and
           the hand stood at the end of the step. Pairs kept: a robot body with anything, and a free thing the
           robot has touched with anything (so the held thing on what it lands on is read). reset true forgets
           the steps kept so far.
"""

SEE_HALF = 1.0 / 2.0     # the share of a pixel's colour above which a surface is what the pixel shows
RAY_HOPS = 32            # see-through surfaces one ray may pass before it is given up
RAY_PAST_M = 1e-6        # a ray starts again this far behind a see-through surface it passed
KEEP_N = 0.01            # a pair whose largest net force in a control step is under this is not kept (N)
DIGITS_N = 3             # the raw reading keeps its forces to this many decimals of a newton
PAD_LINES = 4            # lines from pad to pad cast along the fingers, from the pads' bodies to the tool point
CLOSE_STEP_M = 0.001     # the step by which the copy's jaws are closed to see what they come to bear on


class PairWatch:
    """Wraps sim.step: after every physics step the net contact force per pair of bodies (module docstring).
    Reading changes nothing in the physics."""

    def __init__(self, sim, m, d, np, mj, robot_bodies, hand_site):
        self.np, self.m, self.d, self.mj = np, m, d, mj
        self.geom_body = np.array(m.geom_bodyid, dtype=np.int64)
        self.is_robot = np.zeros(m.nbody, dtype=bool)
        self.is_robot[list(robot_bodies)] = True
        root = np.array(m.body_rootid, dtype=np.int64)
        free_root = np.zeros(m.nbody, dtype=bool)
        for j in range(m.njnt):
            if m.jnt_type[j] == mj.mjtJoint.mjJNT_FREE:
                free_root[m.jnt_bodyid[j]] = True
        self.is_free = free_root[root] & ~self.is_robot          # bodies of a thing that can be carried
        self.free_roots = [int(b) for b in np.nonzero(free_root & ~self.is_robot)[0]]
        self.root = root
        self.touched = np.zeros(m.nbody, dtype=bool)
        self.hand_site = int(hand_site)
        self.steps, self.index, self.cur, self.cur_whole = {}, None, None, None
        self._f6 = np.zeros(6)
        step = sim.step

        def watched():
            step()
            self.sample()
        sim.step = watched

    def begin(self, index):
        """Before a control step: index is its number (1 is the first counted step); None keeps nothing."""
        self.index, self.cur, self.cur_whole = index, {}, {}

    def sample(self):
        if self.index is None:
            return
        np, d, m = self.np, self.d, self.m
        n = int(d.ncon)
        now, whole = {}, {}
        if n:
            con = d.contact
            geoms = np.asarray(con.geom[:n]).reshape(n, 2)
            b1, b2 = self.geom_body[geoms[:, 0]], self.geom_body[geoms[:, 1]]
            live = np.asarray(con.efc_address[:n]) >= 0
            rob = self.is_robot[b1] | self.is_robot[b2]
            for k in np.nonzero(live & rob)[0]:               # a free thing the robot touches is watched from now on
                for b in (int(b1[k]), int(b2[k])):
                    if self.is_free[b]:
                        self.touched[self.root == self.root[b]] = True
            keep = live & (rob | self.touched[b1] | self.touched[b2])
            frames = np.asarray(con.frame[:n]).reshape(n, 3, 3)
            for k in np.nonzero(keep)[0]:
                self.mj.mj_contactForce(m, d, int(k), self._f6)
                f = frames[k].T @ self._f6[:3]                 # world force of geom 1 on geom 2
                a, b = int(b1[k]), int(b2[k])
                if a == b:
                    continue
                key, sign = ((a, b), 1.0) if a < b else ((b, a), -1.0)
                now[key] = now.get(key, 0.0) + sign * f        # the force on the body with the higher id
                whole[b] = whole.get(b, 0.0) + f
                whole[a] = whole.get(a, 0.0) - f
        for key, vec in now.items():
            size = float(np.linalg.norm(vec))
            rec = self.cur.get(key)
            if rec is None:
                self.cur[key] = [size, size, np.array(vec, dtype=float)]
            elif size > rec[0]:
                rec[0], rec[1], rec[2] = size, size, np.array(vec, dtype=float)
            else:
                rec[1] = size
        for key, rec in self.cur.items():
            if key not in now:
                rec[1] = 0.0
        for b, vec in whole.items():
            if self.is_robot[b] or self.touched[b]:
                size = float(np.linalg.norm(vec))
                if size > self.cur_whole.get(b, 0.0):
                    self.cur_whole[b] = size

    def _name(self, b):
        return self.mj.mj_id2name(self.m, self.mj.mjtObj.mjOBJ_BODY, int(b)) or f"body{int(b)}"

    def end(self):
        """After a control step: keep what it made, in the shape bench/r2/meter.py's split() reads: "pairs"
        {"mine|other": [largest net, net at the last physics step, the net on other when largest (3)]}, mine a
        robot body or a free thing the robot has touched; and where the free things and the hand stand."""
        if self.index is None:
            return
        d = self.d
        pairs = {}
        for (a, b), (top, last, vec) in sorted(self.cur.items()):
            ra, rb = bool(self.is_robot[a]), bool(self.is_robot[b])
            if top < KEEP_N or (ra and rb):                 # the robot on itself is not read
                continue
            if (rb and not ra) or (not ra and not rb and self.touched[b] and not self.touched[a]):
                mine, other, on_other = b, a, -vec          # vec is the force on b, the body with the higher id
            else:
                mine, other, on_other = a, b, vec
            pairs[self._name(mine) + "|" + self._name(other)] = [round(top, DIGITS_N), round(last, DIGITS_N)] + \
                [round(float(x), DIGITS_N) for x in on_other]
        rec = {"k": self.index, "pairs": pairs,
               "whole": {self._name(b): round(v, DIGITS_N) for b, v in sorted(self.cur_whole.items())
                         if v >= KEEP_N},
               "things": {self._name(b): [round(float(x), 4) for x in d.xpos[b]] for b in self.free_roots},
               "hand": [round(float(v), 4) for v in d.site_xpos[self.hand_site]]}
        self.steps[self.index] = rec
        self.index = None

    def reading(self, first=0):
        np = self.np
        return {"raw": [self.steps[k] for k in sorted(self.steps) if k >= first],
                "robot": [self._name(b) for b in np.nonzero(self.is_robot)[0]],
                "free": [self._name(b) for b in self.free_roots],
                "unit": "N and world metres per control step k: pairs {mine|other: [largest net over the step's "
                        "physics steps, net at its last physics step, the net on other when largest (x, y, z)]}; "
                        "whole {body: largest sum over everything it touches}; things {root body: [x, y, z]} and "
                        "hand [x, y, z] at the end of the step"}


class TruthMixin:
    """Scoring ops of the env host (module docstring). Uses the host's PUBLIC_CAMERAS table through self.cameras."""

    # ------------------------------------------------------------------ the simulator's own object model
    def _owner(self, obj):
        joints = getattr(obj, "joints", None)
        return {"root_body": str(getattr(obj, "root_body", "")),
                "joints": [str(j) for j in joints] if isinstance(joints, (list, tuple)) else [],
                "category": str(getattr(obj, "category_name", "") or "")}

    def sc_things(self, args):
        self._need_env()
        mj, m, d, inner = self.mj, self.m, self.d, self.inner
        K = mj.mjtObj
        objects, fixtures = {}, {}
        for group, out in (("objects_dict", objects), ("fixtures_dict", fixtures)):
            for name, obj in getattr(inner, group).items():
                rec = self._owner(obj)
                rec["body_id"] = int(inner.obj_body_id[name])
                out[str(name)] = rec
        regions = {}
        for name, site in inner.object_sites_dict.items():
            sid = mj.mj_name2id(m, K.mjOBJ_SITE, name)
            joints = getattr(site, "joints", None)
            rec = {"site_id": int(sid), "parent_name": getattr(site, "parent_name", None),
                   "joints": [str(j) for j in joints] if isinstance(joints, (list, tuple)) else [],
                   "class": type(site).__name__}
            if sid >= 0:
                rec.update(body_id=int(m.site_bodyid[sid]), type=int(m.site_type[sid]))
            regions[str(name)] = rec
        geoms = [{"id": g, "name": mj.mj_id2name(m, K.mjOBJ_GEOM, g) or "", "body": int(m.geom_bodyid[g]),
                  "group": int(m.geom_group[g]), "type": int(m.geom_type[g]), "contype": int(m.geom_contype[g]),
                  "conaffinity": int(m.geom_conaffinity[g]), "alpha": float(m.geom_rgba[g][3])}
                 for g in range(m.ngeom)]
        joints = [{"id": j, "name": mj.mj_id2name(m, K.mjOBJ_JOINT, j) or "", "body": int(m.jnt_bodyid[j]),
                   "type": int(m.jnt_type[j]), "range": m.jnt_range[j], "limited": bool(m.jnt_limited[j])}
                  for j in range(m.njnt)]
        vopt = inner.sim._render_context_offscreen.vopt
        problem = inner.parsed_problem
        return self.plain({
            "cell": self.cell, "language": self.env.language_instruction, "goal_state": self.goal_state,
            "obj_of_interest": list(inner.obj_of_interest),
            "init_state": [list(s) for s in problem.get("initial_state", [])],
            "problem_regions": {str(k): {kk: v[kk] for kk in ("target",) if kk in v}
                                for k, v in problem.get("regions", {}).items()},
            "objects": objects, "fixtures": fixtures, "regions": regions,
            "bodies": [{"id": i, "name": mj.mj_id2name(m, K.mjOBJ_BODY, i) or "", "parent": int(m.body_parentid[i]),
                        "mass_kg": float(m.body_subtreemass[i])} for i in range(m.nbody)],
            "gravity": [float(v) for v in m.opt.gravity],
            "geoms": geoms, "joints": joints,
            "sites": [{"id": s, "name": mj.mj_id2name(m, K.mjOBJ_SITE, s) or "", "body": int(m.site_bodyid[s])}
                      for s in range(m.nsite)],
            "render": {"geomgroup": [int(v) for v in vopt.geomgroup], "sitegroup": [int(v) for v in vopt.sitegroup]},
            "robot_prefixes": list(self.robot_prefixes),
            "objtype": {"geom": int(K.mjOBJ_GEOM), "site": int(K.mjOBJ_SITE), "body": int(K.mjOBJ_BODY)}})

    def sc_shapes(self, args):
        self._need_env()
        m, d, mj = self.m, self.d, self.mj
        want = args.get("bodies")
        if not isinstance(want, list) or not all(isinstance(b, int) and 0 <= b < m.nbody for b in want):
            raise self.BadArgs("bodies is a list of body ids")
        want = set(want)
        out = []
        for g in range(m.ngeom):
            if int(m.geom_bodyid[g]) in want:
                out.append({"id": g, "body": int(m.geom_bodyid[g]), "type": int(m.geom_type[g]),
                            "size": m.geom_size[g], "pos": d.geom_xpos[g], "mat": d.geom_xmat[g].reshape(3, 3),
                            "aabb": m.geom_aabb[g], "group": int(m.geom_group[g]),
                            "collides": bool(m.geom_contype[g] or m.geom_conaffinity[g]),
                            "alpha": float(m.geom_rgba[g][3])})
        return self.plain({"step": self.ledger.step, "geoms": out,
                           "types": {"box": int(mj.mjtGeom.mjGEOM_BOX), "mesh": int(mj.mjtGeom.mjGEOM_MESH)}})

    # ------------------------------------------------------------------ what a camera's picture shows
    def _seg_camera(self, args):
        cam = args.get("camera")
        if cam not in self.cameras:
            raise self.BadArgs("camera is one of the public cameras: " + ", ".join(sorted(self.cameras)))
        w, h = self._size(args)
        return cam, self.cameras[cam], w, h

    def _colour_hash(self, cam, w, h):
        import hashlib
        raw = self.inner.sim.render(camera_name=cam, width=w, height=h, depth=True)   # the look's own call
        return hashlib.sha256(self.np.ascontiguousarray(raw[0][::-1]).tobytes()).hexdigest()

    def _seg(self, cam, w, h):
        """(the segmentation, rows flipped as the colour is; {(objtype, objid): the alpha it was drawn with})."""
        np = self.np
        seg = self.inner.sim.render(camera_name=cam, width=w, height=h, segmentation=True)
        seg = np.ascontiguousarray(np.asarray(seg)[::-1]).astype(np.int32)
        if seg.shape != (h, w, 2):
            raise RuntimeError(f"the segmentation render has shape {seg.shape}, expected {(h, w, 2)}")
        scn = self.inner.sim._render_context_offscreen.scn
        alpha = {}
        for i in range(scn.ngeom):
            g = scn.geoms[i]
            if g.segid != -1:
                alpha[(int(g.objtype), int(g.objid))] = float(g.rgba[3])
        return seg, alpha

    def sc_segment(self, args):
        import hashlib
        import os
        self._need_env()
        cam, name, w, h = self._seg_camera(args)
        path = args.get("path")
        if path is None:
            folder = os.path.join(self.scoring_dir, "segments")
            os.makedirs(folder, exist_ok=True)
            self.renders += 1
            path = os.path.join(folder, f"e{self.ep:02d}_s{self.ledger.step:04d}_{cam}_{self.renders:04d}.npy")
        elif not (isinstance(path, str) and os.path.isabs(path) and path.endswith(".npy")
                  and os.path.isdir(os.path.dirname(path))):
            raise self.BadArgs("path must be an absolute .npy path in an existing folder")
        prove = bool(args.get("prove", False))
        np, mj, m = self.np, self.mj, self.m
        K = mj.mjtObj
        GEOM, SITE = int(K.mjOBJ_GEOM), int(K.mjOBJ_SITE)
        vopt = self.inner.sim._render_context_offscreen.vopt
        groups = {GEOM: (m.geom_group, vopt.geomgroup), SITE: (m.site_group, vopt.sitegroup)}
        off = {}
        for kind, (_, shown) in groups.items():
            dark = [k for k in range(len(shown)) if not shown[k]]
            if not dark:
                raise RuntimeError("every render group is on: nothing can be taken out of the picture")
            off[kind] = dark[0]
        step0, hash0 = self.ledger.step, self._state_hash()
        colour0 = self._colour_hash(name, w, h) if prove else None
        kept, layers, state = {}, [], {}

        def in_picture(pred):
            pairs = np.unique(state["seg"].reshape(-1, 2), axis=0, return_counts=True)
            return [(int(t), int(i), int(n)) for (t, i), n in zip(*pairs)
                    if int(t) in groups and pred(state["alpha"].get((int(t), int(i)), 1.0))]

        def take_out(found):
            for t, i, n in found:
                table = groups[t][0]
                kept.setdefault((t, i), int(table[i]))      # the group it had before anything was taken out
                table[i] = off[t]
                layers.append({"objtype": t, "objid": i, "alpha": state["alpha"][(t, i)], "pixels": n})
            state["seg"], state["alpha"] = self._seg(name, w, h)

        def strip_invisible():
            while True:
                found = in_picture(lambda a: a <= 0.0)
                if not found:
                    return
                take_out(found)

        colour_out = None
        try:
            seg_all, alpha = self._seg(name, w, h)
            state["seg"], state["alpha"] = seg_all, alpha
            strip_invisible()                       # what the colour render does not draw at all
            if prove:
                colour_out = self._colour_hash(name, w, h)
            drawn = [state["seg"]]                  # the front-most surface the picture draws, however faintly
            while True:
                faint = in_picture(lambda a: a < SEE_HALF)
                if not faint:
                    break
                take_out(faint)
                strip_invisible()
                drawn.append(state["seg"])          # the next drawn surface behind a see-through one
            seg = state["seg"]                      # the surface that gives more than half of the pixel's colour
        finally:
            for (t, i), g in kept.items():
                groups[t][0][i] = g
        colour1 = self._colour_hash(name, w, h) if prove else None
        np.save(path, seg)
        np.save(path[:-len(".npy")] + "_all.npy", seg_all)
        np.save(path[:-len(".npy")] + "_drawn.npy", np.stack(drawn))
        return {"path": path, "camera": cam, "width": w, "height": h, "layers_taken_out": layers,
                "pixels_changed": int((seg != seg_all).any(axis=2).sum()), "drawn_layers": len(drawn),
                "colour_sha256_before": colour0, "colour_sha256_after": colour1,
                "colour_sha256_invisible_out": colour_out,
                "seg_sha256": hashlib.sha256(seg.tobytes()).hexdigest(),
                "step_before": step0, "step_after": self.ledger.step,
                "state_sha256_before": hash0, "state_sha256_after": self._state_hash()}

    def sc_ray(self, args):
        import math
        self._need_env()
        cam, name, w, h = self._seg_camera(args)
        pixels = args.get("pixels")
        if not isinstance(pixels, list) or not all(isinstance(p, list) and len(p) == 2 for p in pixels):
            raise self.BadArgs("pixels is a list of [u, v]")
        through = args.get("through", 0)
        if isinstance(through, bool) or not isinstance(through, int) or not 0 <= through <= RAY_HOPS:
            raise self.BadArgs(f"through is a count of further surfaces from 0 to {RAY_HOPS}")
        np, mj, m, d = self.np, self.mj, self.m, self.d
        cid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_CAMERA, name)
        pos = np.array(d.cam_xpos[cid], dtype=float)
        R = np.array(d.cam_xmat[cid], dtype=float).reshape(3, 3)       # columns: right, up, back
        f = 0.5 * h / math.tan(math.radians(float(m.cam_fovy[cid])) / 2.0)
        groups = np.array(self.inner.sim._render_context_offscreen.vopt.geomgroup, dtype=np.uint8)
        gid = np.zeros(1, dtype=np.int32)
        # the alpha each geom is DRAWN with (a material's, where the geom has one: not always the geom's own
        # rgba), read off the renderer's scene of this camera at this step; kept while the step stands
        key = (self.ep, self.ledger.step, name, w, h)
        if getattr(self, "_ray_alpha", (None, None))[0] != key:
            drawn = self._seg(name, w, h)[1]
            geom_type = int(mj.mjtObj.mjOBJ_GEOM)
            self._ray_alpha = (key, {i: a for (t, i), a in drawn.items() if t == geom_type})
        drawn_alpha = self._ray_alpha[1]
        out = []
        for u, v in pixels:
            ray = np.array([(float(u) + 0.5 - w / 2.0) / f, -(float(v) + 0.5 - h / 2.0) / f, -1.0])
            along = float(np.linalg.norm(ray))
            vec = R @ (ray / along)
            origin, gone, hit, front = pos.copy(), 0.0, None, None
            for _ in range(RAY_HOPS):
                dist = float(mj.mj_ray(m, d, origin, vec, groups, 1, -1, gid))
                if dist < 0.0:
                    break
                gone += dist
                g = int(gid[0])
                alpha = float(drawn_alpha.get(g, m.geom_rgba[g][3]))
                if alpha > 0.0 and front is None:      # the first surface the picture draws at all (its depth)
                    front = {"geom": g, "body": int(m.geom_bodyid[g]), "depth_m": gone / along,
                             "point": pos + vec * gone}
                if alpha >= SEE_HALF:
                    hit = g
                    break
                origin = origin + vec * (dist + RAY_PAST_M)
                gone += RAY_PAST_M
            behind = []                                 # with through: the next surfaces along the same ray
            if hit is not None and through:
                o2, g2 = pos + vec * (gone + RAY_PAST_M), gone + RAY_PAST_M
                for _ in range(through):
                    dist = float(mj.mj_ray(m, d, o2, vec, groups, 1, -1, gid))
                    if dist < 0.0:
                        break
                    g2 += dist
                    behind.append({"geom": int(gid[0]), "body": int(m.geom_bodyid[int(gid[0])]),
                                   "depth_m": g2 / along, "point": pos + vec * g2})
                    o2, g2 = o2 + vec * (dist + RAY_PAST_M), g2 + RAY_PAST_M
            if hit is None:
                out.append({"geom": -1, "body": -1, "range_m": None, "depth_m": None, "point": None, "front": front,
                            "behind": behind})
            else:
                out.append({"geom": hit, "body": int(m.geom_bodyid[hit]), "range_m": gone, "depth_m": gone / along,
                            "point": pos + vec * gone, "front": front, "behind": behind})
        return self.plain({"camera": cam, "step": self.ledger.step, "rays": out})

    # ------------------------------------------------------------------ a kinematic copy of the scene
    def sc_kin(self, args):
        """{q: the arm's joints (rad), open_m: the gap between the jaws' pads (m) or null, points: [[x, y, z],
        ...] (world, m) or null, bodies: [body id, ...]}: the scene as it stands with the arm PUT at q and the
        jaws at open_m, in a copy of the simulation's data (the episode's own data is not touched, no step
        runs): "inside": every contact of a robot geom with another body that goes under its surface (the
        depth, m); "pads": where the two pads stand; "between": the bodies a straight line from pad to pad
        meets; with close true: "close": at which opening each jaw, closed step by step in the copy, comes to
        bear on the named bodies (null: it never does before the jaws meet); "near": for each point the distance to the nearest surface of the named bodies along 26
        bearings within reach_m (null: none within reach). For the census."""
        self._need_env()
        np, mj, m, d = self.np, self.mj, self.m, self.d
        r = self.robot
        d2 = getattr(self, "_kin_data", None)
        if d2 is None or getattr(self, "_kin_ep", None) != self.ep:
            d2 = self._kin_data = mj.MjData(m)
            self._kin_ep = self.ep
        d2.qpos[:], d2.qvel[:] = d.qpos, 0.0
        if m.nmocap:
            d2.mocap_pos[:], d2.mocap_quat[:] = d.mocap_pos, d.mocap_quat
        out = {"step": self.ledger.step}
        if args.get("q") is not None:
            d2.qpos[r._ref_joint_pos_indexes] = self.np.array(args["q"], dtype=float)
        if args.get("open_m") is not None:
            half = max(0.0, float(args["open_m"])) / 2.0
            gi = r._ref_gripper_joint_pos_indexes
            lo, hi = m.jnt_range[r._ref_gripper_joint_indexes].T if hasattr(r, "_ref_gripper_joint_indexes") else \
                (np.array([0.0, -1.0]), np.array([1.0, 0.0]))
            d2.qpos[gi[0]] = min(max(half, lo[0]), hi[0])
            d2.qpos[gi[1]] = min(max(-half, lo[1]), hi[1])
        mj.mj_forward(m, d2)
        is_robot, name = self.pairs.is_robot, self.pairs._name
        inside = []
        for k in range(d2.ncon):
            c = d2.contact[k]
            b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
            if bool(is_robot[b1]) != bool(is_robot[b2]) and float(c.dist) < 0.0:
                rb, ob = (b1, b2) if is_robot[b1] else (b2, b1)
                inside.append({"robot": name(rb), "other": ob, "other_name": name(ob), "depth_m": -float(c.dist)})
        out["inside"] = inside
        tips = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, n) for n in
                ("gripper0_finger_joint1_tip", "gripper0_finger_joint2_tip")]
        gid = np.zeros(1, dtype=np.int32)
        if all(t >= 0 for t in tips):
            p1, p2 = np.array(d2.xpos[tips[0]]), np.array(d2.xpos[tips[1]])
            gap = float(np.linalg.norm(p2 - p1))
            met = set()
            # the pads reach from their bodies' origins toward the fingertips: lines from pad to pad at the
            # origins and at steps along the fingers (away from the hand), down to the tool point's own reach
            hand = np.array(d2.xpos[int(m.body_parentid[int(m.body_parentid[tips[0]])])])
            mid = (p1 + p2) / 2.0
            along = (mid - hand) / max(float(np.linalg.norm(mid - hand)), 1e-9)
            reach = max(0.0, float((np.array(d2.site_xpos[r.eef_site_id]) - mid) @ along))
            for s_m in ([0.0] + [reach * k / PAD_LINES for k in range(1, PAD_LINES + 1)] if gap > 0.0 else []):
                vec, origin, gone = (p2 - p1) / gap, p1 + along * s_m, 0.0
                for _ in range(RAY_HOPS):
                    dist = float(mj.mj_ray(m, d2, origin, vec, None, 1, -1, gid))
                    if dist < 0.0 or gone + dist > gap:
                        break
                    b = int(m.geom_bodyid[int(gid[0])])
                    if not is_robot[b]:
                        met.add(b)
                    origin, gone = origin + vec * (dist + RAY_PAST_M), gone + dist + RAY_PAST_M
            out.update(pads=[p1, p2], gap_m=gap, between=sorted(met), pad_reach_m=reach)
        pts, bodies = args.get("points"), set(args.get("bodies") or [])
        if args.get("close") and bodies and args.get("open_m") is not None:
            # the jaws closed step by step in the copy: does each jaw come to bear on the named bodies before
            # the jaws meet each other? ("the thing lies between the pads and is narrower than they open")
            gi, gj = r._ref_gripper_joint_pos_indexes, [int(j) for j in r._ref_gripper_joint_indexes] \
                if hasattr(r, "_ref_gripper_joint_indexes") else None
            sides = []
            for k in range(2):
                root = int(m.jnt_bodyid[gj[k]]) if gj else int(m.body_parentid[tips[k]])
                sides.append({b for b in range(m.nbody) if self._under(b, root)})
            first = [None, None]
            w = float(args["open_m"])
            while w >= 0.0 and (first[0] is None or first[1] is None):
                d2.qpos[gi[0]], d2.qpos[gi[1]] = w / 2.0, -w / 2.0
                mj.mj_forward(m, d2)
                for k in range(d2.ncon):
                    c = d2.contact[k]
                    b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
                    for side in range(2):
                        if first[side] is None and ((b1 in sides[side] and b2 in bodies) or
                                                    (b2 in sides[side] and b1 in bodies)) and float(c.dist) <= 0.0:
                            first[side] = w
                w -= CLOSE_STEP_M
            out["close"] = {"jaw_1_bears_at_m": first[0], "jaw_2_bears_at_m": first[1]}
        if pts:
            reach = float(args.get("reach_m", 0.05))
            dirs = [np.array([x, y, z], dtype=float) for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)
                    if (x, y, z) != (0, 0, 0)]
            near = []
            for p in pts:
                best = None
                for v in dirs:
                    v = v / np.linalg.norm(v)
                    origin, gone = np.array(p, dtype=float), 0.0
                    for _ in range(RAY_HOPS):
                        dist = float(mj.mj_ray(m, d2, origin, v, None, 1, -1, gid))
                        if dist < 0.0 or gone + dist > reach:
                            break
                        if int(m.geom_bodyid[int(gid[0])]) in bodies:
                            best = gone + dist if best is None else min(best, gone + dist)
                            break
                        origin, gone = origin + v * (dist + RAY_PAST_M), gone + dist + RAY_PAST_M
                near.append(best)
            out["near"] = near
        return self.plain(out)

    def _under(self, body, root):
        m = self.m
        while body > 0:
            if body == root:
                return True
            body = int(m.body_parentid[body])
        return False

    # ------------------------------------------------------------------ contact truth per control step
    def sc_peaks(self, args):
        self._need_env()
        first = args.get("from", 0)
        if isinstance(first, bool) or not isinstance(first, int) or first < 0:
            raise self.BadArgs("from is a control step index of 0 or more")
        out = self.pairs.reading(first)
        out["step"] = self.ledger.step
        if args.get("reset"):
            self.pairs.steps.clear()
        return out
