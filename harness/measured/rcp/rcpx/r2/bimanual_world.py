# SPDX-License-Identifier: AGPL-3.0-only
"""One world for several arms (rcpx/r2/bimanual.py): every point any arm stores is kept once in the robot frame R
and mirrored into every arm's World in that arm's frame B, under the same id and label.

Frames holds each arm's installation (T_R_B, the arm's frame B in R) and turns points and directions between frames.
ArmWorld is an rcpx.context.World for one arm: after any call that changes what it knows (add, add_stop, update,
mark_stale, world_moved, claim_state, verify) every entry it changed is copied into every other arm's World, each
point and each stored direction (data keys in VECTOR_KEYS) turned into that arm's frame. Entry ids come from one
counter shared by every arm, so an id names the same thing in every arm's World. Each entry carries "arm": the arm
whose call stored it. Observations (looks, senses) and the last act stay with the arm that made them: verify judges
an arm's evidence against that arm's own moves.

world_doc is robot://world: the entries with their points in frame R and, per entry, the point in each arm's frame
B; then per arm its own document (observations, last act, what packages add) without the entries.
"""

import copy

import numpy as np

from rcpx.context import World

MM = 1000.0
VECTOR_KEYS = ("normal_est", "normal")      # data keys holding a direction in the storing arm's frame
MUTATORS = ("add", "add_stop", "update", "mark_stale", "world_moved", "claim_state", "verify")


class Frames:
    """Installation poses of every arm: T_R_B (4 x 4, metres)."""

    def __init__(self, poses, errors_mm=None):
        self.arms = tuple(poses)
        self.errors_m = {a: float(v) / MM for a, v in (errors_mm or {}).items() if v is not None}
        if any(not np.isfinite(v) or v < 0 for v in self.errors_m.values()):
            raise ValueError("base errors must be finite and nonnegative")
        self._T = {a: np.asarray(T, dtype=float).reshape(4, 4).copy() for a, T in poses.items()}
        self._Ti = {a: _inv(T) for a, T in self._T.items()}

    def T_R_B(self, arm):
        return self._T[arm].copy()

    def T_B_R(self, arm):
        return self._Ti[arm].copy()

    def point_to_R(self, arm, p_mm):
        p = np.asarray(p_mm, dtype=float).reshape(3) / MM
        return (self._T[arm][:3, :3] @ p + self._T[arm][:3, 3]) * MM

    def point_from_R(self, arm, p_mm):
        p = np.asarray(p_mm, dtype=float).reshape(3) / MM
        return (self._Ti[arm][:3, :3] @ p + self._Ti[arm][:3, 3]) * MM

    def point(self, src, dst, p_mm):
        """A point (mm) in arm src's frame B, in arm dst's frame B."""
        return self.point_from_R(dst, self.point_to_R(src, p_mm))

    def vector(self, src, dst, v):
        R = self._Ti[dst][:3, :3] @ self._T[src][:3, :3]
        return R @ np.asarray(v, dtype=float).reshape(3)

    def dir_to_R(self, arm, v):
        return self._T[arm][:3, :3] @ np.asarray(v, dtype=float).reshape(3)

    def dir_from_R(self, arm, v):
        return self._Ti[arm][:3, :3] @ np.asarray(v, dtype=float).reshape(3)

    def base_doc(self, arm):
        T = self._T[arm]
        return {"position_mm": _r(T[:3, 3] * MM, 3), "R": [_r(row, 6) for row in T[:3, :3]],
                "x_axis": _r(T[:3, 0], 6), "z_axis": _r(T[:3, 2], 6)}


def _inv(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def _r(v, nd):
    return [round(float(x), nd) + 0.0 for x in np.asarray(v, dtype=float).reshape(-1)]


class Store:
    """The arms' Worlds, the one id counter they share and each entry's canonical copy: (the arm whose World last
    changed it, the entry as that World holds it). Every other World holds that copy turned into its own frame, so
    an entry no World changes is never turned twice (no drift from going round the frames)."""

    def __init__(self, frames):
        self.frames = frames
        self.worlds = {}
        self.n = 0
        self.canon = {}

    def join(self, world):
        self.worlds[world.arm] = world

    def mirror(self, src):
        """Copy what src changed into every other arm's World, each in that arm's frame."""
        for eid, e in list(src.entries.items()):
            c = self.canon.get(eid)
            if c is not None:
                seen = c[1] if c[0] == src.arm else self._turned(c[1], c[0], src.arm)
                if seen == e:
                    continue
            self.canon[eid] = (src.arm, copy.deepcopy(e))
            for arm, w in self.worlds.items():
                if w is not src:
                    w.entries[eid] = self._turned(e, src.arm, arm)
        for w in self.worlds.values():
            w._n = self.n

    def point_R(self, eid):
        """An entry's point in frame R (mm, from its canonical copy), or None."""
        c = self.canon.get(eid)
        if c is None or c[1].get("point_mm") is None:
            return None
        return self.frames.point_to_R(c[0], c[1]["point_mm"])

    def _turned(self, e, src, dst):
        out = copy.deepcopy(e)
        if out.get("point_mm") is not None:
            out["point_mm"] = _r(self.frames.point(src, dst, out["point_mm"]), 6)
        data = out.get("data")
        if isinstance(data, dict):
            for key in VECTOR_KEYS:
                v = data.get(key)
                if isinstance(v, (list, tuple)) and len(v) == 3 and all(isinstance(x, (int, float)) for x in v):
                    data[key] = _r(self.frames.vector(src, dst, v), 6)
        return out


class ArmWorld(World):
    """One arm's World in its frame B, mirrored into the other arms' Worlds (module docstring)."""

    def __init__(self, journal, arm, store):
        super().__init__(journal)
        self.arm = arm
        self._store = store
        self._depth = 0
        store.join(self)

    def _new_entry(self, kind, label, point_mm, source, confirmed, data, call, t_s):
        self._n = self._store.n
        out = super()._new_entry(kind, label, point_mm, source, confirmed, data, call, t_s)
        if isinstance(out, dict) and out.get("id") in self.entries:
            self.entries[out["id"]]["arm"] = self.arm
            out["arm"] = self.arm
            self._store.n = self._n
        return out


def _mirrored(name):
    base = getattr(World, name)

    def method(self, *args, **kwargs):
        if name == "claim_state":
            self._n = self._store.n
        self._depth += 1
        try:
            out = base(self, *args, **kwargs)
        finally:
            self._depth -= 1
        if name == "claim_state":
            self._store.n = max(self._store.n, self._n)
            if isinstance(out, dict) and out.get("id") in self.entries:
                self.entries[out["id"]].setdefault("arm", self.arm)
        if self._depth == 0:
            self._store.mirror(self)
        return out

    method.__name__ = name
    method.__doc__ = base.__doc__
    return method


for _name in MUTATORS:
    setattr(ArmWorld, _name, _mirrored(_name))


def world_doc(store, arm_docs=None):
    """robot://world (module docstring). arm_docs {arm: that arm's robot://world content}, as packages patch it."""
    frames = store.frames
    arms = frames.arms
    ref = store.worlds[arms[0]]
    entries = []
    for eid in ref.entries:
        src, e = store.canon.get(eid, (arms[0], ref.entries[eid]))
        row = copy.deepcopy(e)
        if e.get("point_mm") is not None:
            row["point_mm"] = _r(frames.point_to_R(src, e["point_mm"]), 3)
            row["point_mm_in"] = {a: _r(store.worlds[a].entries[eid]["point_mm"], 3) for a in arms
                                  if eid in store.worlds[a].entries}
        data = row.get("data")
        if isinstance(data, dict):
            for key in VECTOR_KEYS:
                v = data.get(key)
                if isinstance(v, (list, tuple)) and len(v) == 3 and all(isinstance(x, (int, float)) for x in v):
                    data[key] = _r(frames.dir_to_R(src, v), 4)
        entries.append(row)
    per_arm = {}
    for a in arms:
        doc = (arm_docs or {}).get(a)
        if not isinstance(doc, dict):
            doc = store.worlds[a].to_json()
        doc = {k: v for k, v in doc.items() if k not in ("entries", "next")}
        doc["frame"] = f"B of arm {a}: its own base frame (mm); its tools take and give points in it"
        doc["base_in_R"] = frames.base_doc(a)
        per_arm[a] = doc
    return {"frame": "R: the robot frame every arm base is installed in (mm, z up)", "entries": entries,
            "next": store.n, "arms": per_arm,
            "note": "every point is stored once and shown here in frame R; point_mm_in gives it in each arm's frame "
                    "B, the frame that arm's tools use"}
