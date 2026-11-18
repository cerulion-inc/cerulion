# SPDX-License-Identifier: AGPL-3.0-only
"""The space book of a session: the stated support and the things the session has
MEASURED (touched, located, the extent of a spot, another arm's capsules), and nothing else. Nothing is inferred
and nothing is kept past the session.

kind() answers "expected" inside the extent of a measured thing (its surface grown by BAND x its own measured
error) and "free" everywhere else: space nobody measured is free, and what a motion there could strike is priced on
the plan (rcpx/bounds_strike.py), never slowed for. The book holds no grid, no swept cells, no word cells and no
ageing: speed does not rest on them, so there is nothing for them to prove.

It proves no way either: route() and ceiling() answer None and the caller takes its plain route (up, across,
down). Looks are recorded for the word (information only) and for the tools that point into them. drop() stamps an
event; with a region it stamps nothing (an expected contact changes only its own region, and the book holds no
cells there to take back).
"""

import math
from dataclasses import replace

import numpy as np

from rcpx.bounds_laws import ROUND
from rcpx.bounds_ratios import BAND
from rcpx.bounds_report import MM
from rcpx.bounds_book import entry_sigma, segment_clearance, segment_capsules
from rcpx.bounds_robot import Robot, Thing

MEASURED = ("contact", "located", "device")     # the world's kinds of entry that carry a measured point


class Share(dict):
    """The free part of the reach by source; the book counts no cells, so it is empty and its number is 0."""

    def __float__(self):
        return float(sum(self.values()))


class SpaceBook:
    def __init__(self, ex, robot=None):
        self.ex, self.robot = ex, robot or Robot(ex)
        self._bounds = getattr(ex, "bounds", None)
        self.notes, self.looks = [], {}
        self._support, self._things = None, {}
        self._order, self._event, self._rev, self._cache = 0, None, 0, None
        cfg = getattr(ex, "cfg", None) or {}
        stated = cfg.get("support")
        if stated is not None:
            self.support((stated["normal"], stated["offset_m"]), stated.get("sigma_m"), "stated")
        elif "floor_z_m" in cfg:
            self.support(((0, 0, 1), cfg["floor_z_m"]), None, "stated")

    # ------------------------------------------------------------------------------------------ things
    def support(self, plane=None, sigma=None, source=None):
        """The support plane ((normal, offset_m), sigma_m, source), or None. With a plane: state it; with a sigma
        alone: the stated plane's error as measured."""
        if plane is not None:
            self._support = (plane, sigma, source or "contact")
            self._rev += 1
        elif sigma is not None and self._support is not None:
            self._support = (self._support[0], sigma, source or self._support[-1])
            self._rev += 1
        return self._support

    def add_thing(self, id, shape, sigma, source):
        """A measured thing: {"box": (lo, hi)}, {"plane": (normal, offset_m)}, {"kind": "capsules", "capsules_m":
        [(p0, p1, r)]} (another arm) or a point. shape None removes it. A shape with no surface (a cone of rays)
        is not a measured thing and is not kept."""
        self._rev += 1
        self._things.pop(id, None)
        if shape is None or (isinstance(shape, dict) and "cone" in shape):
            return None
        if isinstance(shape, dict):
            self._things[id] = (Thing(id=id, extent=shape, sigma_p=sigma), source)
        else:
            self._things[id] = (Thing(id=id, point=np.asarray(shape, dtype=float).reshape(-1), sigma_p=sigma), source)
        return None

    def _world_things(self):
        """(the world's measured entries, a signature that changes when they do)."""
        world = getattr(self.ex, "world", None)
        if world is None:
            return [], None
        entries = getattr(world, "entries", None)
        if isinstance(entries, dict):
            rows = [e for e in entries.values() if e.get("kind") in MEASURED]
            seq = getattr(getattr(world, "journal", None), "seq", None)
            seq = seq() if callable(seq) else seq
            sig = (len(entries), getattr(world, "_n", None), seq if seq is not None else
                   tuple((e["id"], tuple(e["point_mm"] or ()), str(e.get("data"))) for e in rows))
        else:
            rows = [e for kind in MEASURED for e in world.query(kind=kind, include_stale=True)]
            sig = tuple((e.get("id"), tuple(e.get("point_mm") or ()), str(e.get("data"))) for e in rows)
        return rows, sig

    def _all(self):
        """Every measured thing with its source, the session's contact records merged in: [(Thing, source)]."""
        rows, sig = self._world_things()
        B = self._bounds or getattr(self.ex, "bounds", None)
        records = getattr(B, "_records", None) or {}
        key = (self._rev, sig, tuple((k, tuple(sorted((a, str(b)) for a, b in v.items()))) for k, v in records.items()))
        if self._cache is not None and self._cache[0] == key:
            return self._cache[1]
        out = list(self._things.values())
        if self._support is not None:
            plane, sigma, source = self._support
            out.append((Thing(id="support", extent={"plane": plane}, sigma_p=sigma), source))
        for e in rows:
            if e.get("point_mm") is None:
                continue
            data = e.get("data") or {}
            thing = Thing(id=e.get("id"), point=np.asarray(e["point_mm"], dtype=float) / MM,
                          sigma_p=entry_sigma(data), extent=data.get("extent"), k_c=data.get("k_c_n_per_m"),
                          f_gave=data.get("f_gave_n"), f_moved=data.get("f_moved_n"),
                          f_worked=data.get("f_worked_n"), touched=data.get("touched", False))
            out.append((thing, "contact" if e.get("kind") == "contact" else "seen"))
        fields = Thing.__dataclass_fields__
        out = [(replace(t, **{k: v for k, v in records[t.id].items() if k in fields}), "contact")
               if t.id in records else (t, s) for t, s in out]
        self._cache = (key, out)
        return out

    def kind(self, p0, p1, radius):
        """(expected | free, the nearest measured thing or None, its clearance m, its source): for a ball of this
        radius moving p0 to p1. Expected only within BAND x the thing's own measured error of its surface."""
        p0, p1, radius = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float), float(radius)
        best = (math.inf, None, None)
        for thing, source in self._all():
            extent = thing.extent if isinstance(thing.extent, dict) else {}
            if extent.get("capsules_m"):
                c, at = segment_capsules(p0, p1, extent["capsules_m"])
                c, thing = c - radius, replace(thing, point=at)     # the planner measures a point: the nearest face
            else:
                c = segment_clearance(p0, p1, thing, radius)
            if c < best[0]:
                best = (c, thing, source)
        c, thing, source = best
        if thing is not None and c <= BAND * (thing.sigma_p or 0.0) + ROUND:
            return "expected", thing, c, source
        return "free", thing, c, None

    # ------------------------------------------------------------------------------------------ looks and events
    def note_look(self, look_id, cam, frame=None, at=None, shown=True):
        """Record a look: for the word (information) and for the tools that point into it."""
        self._order += 1
        self.looks[look_id] = {"id": look_id, "look": look_id, "cam": cam, "frame": frame, "at": at,
                               "shown": bool(shown), "order": self._order}
        return self.looks[look_id]

    def newest_look(self):
        return max((v for v in self.looks.values() if v["shown"]), key=lambda v: v["order"], default=None)

    def last_event(self):
        return self._event

    def drop(self, event, region=None):
        """Something changed the scene (a contact nobody expected, a release, an act, a person, a new epoch): the
        event is stamped. With a region nothing is stamped: the change is that region's own."""
        if region is None:
            self._order += 1
            self._event = (event, self._order)
        return None

    def route(self, p0, p1, radius, held=None):
        """None: the book proves no way. The caller takes its plain route (up, across, down)."""
        return None

    def ceiling(self, a_xy, b_xy, half_width_m):
        """None: the book proves no free layer. The caller takes its own height."""
        return None

    # ------------------------------------------------------------------------------------------ compat shims
    # Names other packages still call until their callers move. Each does nothing.
    def add_swept(self, q0, q1, held=None):   # compat shim
        return None

    def covered(self, path, looks=None):      # compat shim
        return None, "not computed: the book keeps no cells"

    def word(self, look_id, path=None, held=None):   # compat shim
        return None

    def end_call(self):                       # compat shim
        return None

    def share(self, kind="free"):             # compat shim
        return Share() if kind == "free" else 0.0

    def unseen_hint(self, path):              # compat shim
        return ""

    def carve(self, *args, **kwargs):         # compat shim
        return 0

    def resee(self, frames):                  # compat shim
        return []
