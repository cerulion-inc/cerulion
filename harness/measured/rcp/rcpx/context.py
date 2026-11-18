# SPDX-License-Identifier: AGPL-3.0-only
"""Session context for the rcpx runtime: World (what is known), Journal, practices,
device knowledge and confirmed state bookkeeping.

Nothing in this module reads simulator ground truth, and the Journal refuses any record that carries
a key naming it. Positions are in millimetres in the base frame B (x forward, y left, z up), the
operator-facing unit. Time is the call index plus simulated seconds; there is no wall clock here.

Split by concern: the shared helpers live in context_base.py, practices and device knowledge in
context_devices.py; every name of both is re-exported here, so `from rcpx.context import ...` is the one import.
"""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import numpy as np

from rcpx.context_base import (FRAME_KEYS, TRUTH_MARKER, GroundTruthLeak, _check_no_truth, _is_image,  # noqa: F401
                               jsonable, refusal)
from rcpx.context_devices import (LEVEL_SOURCE, LEVELS, MANUAL_DEVICE_KEYS, MANUAL_KEYS, NEVER_KEYS,  # noqa: F401
                                  NOT_GIVEN_NOTE, PART_FIELDS, PRACTICES_PATH, _clean, _field, _sheet,
                                  device_knowledge, not_given, practices_text)

KINDS = ("contact", "located", "device", "state", "event", "box")
# A touch is a measurement; a located point is an image estimate; a device entry is a hypothesis;
# a state claim is unconfirmed until verify; an event is a record of something that happened.
# A box is the rough extent of a thing a survey saw: data {centre_mm [x, y], size_mm [sx, sy], top_mm,
# source, candidate}; an image estimate, so unconfirmed.
DEFAULT_CONFIRMED = {"contact": True, "located": False, "device": False, "state": False, "event": True,
                     "box": False}
POSITIONAL = ("contact", "located", "device")  # the point entries a world_moved event makes stale
# A box is an extent, not a point: it is not POSITIONAL (nothing that reads points takes its top centre for one) and
# query() returns boxes only when asked by kind. It goes stale like a point does (the base or the scene moved,
# mark_stale), and when a later call from the same source adds a box (a new survey replaces the last one's).
STALEABLE = POSITIONAL + ("box",)
UPDATABLE = ("label", "point_mm", "source", "data", "confirmed")
MARK_UNCONFIRMED, MARK_CONFIRMED, MARK_SUPERSEDED = "unconfirmed", "confirmed", "superseded"
# What to do about a dark look. It names no tool a session with verify lacks: what a surface offers in the
# dark (a wait, or nothing) is the surface's dark rule (rcpx/surface_session.py dark_rule, in the session's hints).
DARK_HINT = ("a dark look cannot decide a state: any new look reads the light again; while it stays dark, report the "
             "state as unconfirmed, and do not work by feel in the dark")
# One dim rule (T9), the same words as the look description's (rcpx/surfaces.py DIM_RULE).
DIM_NOTE = ("the look was dim: it can locate large things; it decides a small state only from close (the part large "
            "in the frame) or by a before and after pair from the same pose: say so in the claim")
# What World.verify itself checks and what it does not (S3): it never compares a picture with the state.
NOT_CHECKED = "the picture itself was not compared with the state: the state rests on your evidence"
BASIS = "your_evidence"

class Journal:
    """Append-only JSONL: one line per call, read, world event or mark, in order.

    Images inside a result are written to EPDIR/frames/ and replaced by their relative path.
    Reopening an existing journal continues its sequence; nothing is ever rewritten.
    """

    def __init__(self, ep_dir, filename: str = "journal.jsonl"):
        self.dir = Path(ep_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.path = self.dir / filename
        self.frames_dir = self.dir / "frames"
        self._seq = 0
        if self.path.exists():
            with self.path.open("r", encoding="utf-8") as f:
                self._seq = sum(1 for line in f if line.strip())

    @property
    def seq(self) -> int:
        return self._seq

    def save_frame(self, img: np.ndarray, stem: str) -> str:
        """Write an RGB frame as PNG under frames/; returns the path relative to the episode dir."""
        from PIL import Image  # local: only frame writers need pillow

        self.frames_dir.mkdir(parents=True, exist_ok=True)
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in stem)
        rel = Path("frames") / f"{safe}.png"
        Image.fromarray(np.ascontiguousarray(img[:, :, :3])).save(self.dir / rel)
        return str(rel)

    def _extract_frames(self, obj, key_path, frames):
        if _is_image(obj):
            rel = self.save_frame(obj, f"{self._seq:06d}_{key_path or 'frame'}")
            frames.append(rel)
            return rel
        if isinstance(obj, dict):
            out = {}
            for k, v in obj.items():
                nv = self._extract_frames(v, f"{key_path}_{k}" if key_path else str(k), frames)
                if isinstance(v, str) and str(k) in FRAME_KEYS and v.endswith((".png", ".jpg")):
                    frames.append(v)
                out[k] = nv
            return out
        if isinstance(obj, (list, tuple)):
            return [self._extract_frames(v, f"{key_path}{i}", frames) for i, v in enumerate(obj)]
        return obj

    def append(self, kind: str, **fields) -> dict:
        """Append one record {seq, kind, ...}; raises GroundTruthLeak on a ground truth key."""
        rec = {"seq": self._seq, "kind": str(kind)}
        rec.update(fields)
        _check_no_truth(rec)
        frames = list(rec.pop("frames", None) or [])
        rec = self._extract_frames(rec, "", frames)
        if frames:
            rec["frames"] = list(dict.fromkeys(frames))
        rec = jsonable(rec)
        line = json.dumps(rec, separators=(",", ":"), allow_nan=False)
        with self.path.open("a", encoding="utf-8") as f:
            f.write(line + "\n")
        self._seq += 1
        return rec

    def record_call(self, name, args, result, call=None, t_s=None, frames=None, op="call") -> dict:
        """One tool call with its arguments and result (images saved as frames)."""
        return self.append(op, name=str(name), args=args or {}, result=result, call=call, t_s=t_s, frames=frames)

    def records(self) -> list:
        return read_journal(self.path)

def read_journal(path) -> list:
    """Every record of a journal file, in order."""
    p = Path(path)
    if not p.exists():
        return []
    with p.open("r", encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]

def replay_calls(path) -> list:
    """(name, args) of every call in a journal, in order: the input of a replay."""
    return [(r["name"], r.get("args") or {}) for r in read_journal(path) if r.get("kind") == "call"]

def _guard_source(source) -> bool:
    """True for a source that names a guard stop (control records them as "move_tip guard" and the like)."""
    return "guard" in str(source).lower().split()

def _point(p):
    if p is None:
        return None
    a = np.asarray(p, dtype=float).reshape(-1)
    if a.size != 3 or not np.all(np.isfinite(a)):
        raise ValueError("point_mm must be three finite numbers")
    return [round(float(v), 3) for v in a]

class World:
    """Typed entries the session knows, each with id, label, kind, point_mm, source, observed_at
    ({call, t_s}), confirmed and stale. Observations (looks and other senses) are kept separately so
    verify can check when evidence was taken, and last_act is the last call in which the arm moved
    (note_act): evidence older than it shows the state before that call. Insertion ordered: iteration
    is deterministic.
    """

    def __init__(self, journal: Journal | None = None):
        self.journal = journal
        self.entries: dict[str, dict] = {}
        self.observations: list[dict] = []
        self.now = {"call": 0, "t_s": 0.0}
        self.last_act = None  # {"call", "t_s", "name"} of the last call that moved the arm
        self.spot_rule = None  # optional session rule: combined position uncertainty plus tool half width
        self.clock = None     # optional callable: the world clock observations are stamped with (note_sense)
        self._n = 0

    def tick(self, call: int, t_s: float) -> None:
        """Set the present call index and simulated time; later entries are stamped with it."""
        self.now = {"call": int(call), "t_s": round(float(t_s), 6)}

    def _stamp(self, call=None, t_s=None) -> dict:
        return {
            "call": self.now["call"] if call is None else int(call),
            "t_s": self.now["t_s"] if t_s is None else round(float(t_s), 6),
        }

    def _log(self, kind, **fields):
        if self.journal is not None:
            self.journal.append(kind, call=self.now["call"], t_s=self.now["t_s"], **fields)

    def add(self, kind, label, point_mm=None, source="", confirmed=None, data=None, call=None, t_s=None) -> dict:
        """Add an entry; returns a copy of it, or a refusal for a bad kind or point."""
        if kind not in KINDS:
            return refusal("kind", f"unknown kind {kind!r}", "use one of " + ", ".join(KINDS))
        if kind == "state":
            return refusal("claim", "a state entry is made by claim_state and confirmed only by verify",
                           "claim_state(device, state); a look (0 control periods) and verify decide it")
        if kind == "contact" and confirmed is None and _guard_source(source):
            return self.add_stop(label, point_mm, source, data, call, t_s)
        if kind == "box":
            self._stale_older_boxes(source, self._stamp(call, t_s)["call"])
        return self._new_entry(kind, label, point_mm, source, confirmed, data, call, t_s)

    def _stale_older_boxes(self, source, call):
        """A box from a later call of the same source replaces that source's earlier boxes (boxes added within one
        call stand together): the earlier ones go stale, so a reader of fresh boxes sees one survey, the newest."""
        old = [e["id"] for e in self.entries.values()
               if e["kind"] == "box" and not e["stale"] and e["source"] == str(source)
               and e["observed_at"]["call"] != call]
        if old:
            self.mark_stale(old, f"a newer {source or 'box'} replaced it", source=str(source) or "runtime")

    def add_stop(self, label, point_mm=None, source="", data=None, call=None, t_s=None) -> dict:
        """Record where a move stopped on its guard. That is a stop, not a measured touch: something resisted
        there, and it may have been the arm itself. It is stored as a contact with confirmed false and
        data.stop true until a measured contact lands within their combined uncertainty and tool half width."""
        note = "a guard stop, not a measured touch: touch here to confirm it"
        return self._new_entry("contact", label, point_mm, source, False, {**(data or {}), "stop": True, "note": note},
                               call, t_s)

    def same_spot_mm(self, a, b):
        """The session's rule, else the entries' own combined stated position errors."""
        if callable(self.spot_rule):
            try:
                value = float(self.spot_rule(a, b))
                if math.isfinite(value) and value >= 0:
                    return value
            except Exception:
                pass
        errors = []
        for entry in (a, b):
            try:
                error = float((entry.get("data") or {}).get("error_mm") or 0)
                errors.append(error if math.isfinite(error) and error >= 0 else 0)
            except Exception:
                errors.append(0)
        return math.hypot(*errors)

    def _agree(self, e):
        """A measured contact confirms stops within their combined uncertainty and tool half width."""
        if e["kind"] != "contact" or not e["confirmed"] or e["point_mm"] is None:
            return
        p = np.asarray(e["point_mm"], dtype=float)
        for o in self.entries.values():
            if (o["kind"] == "contact" and not o["confirmed"] and not o["stale"] and o["point_mm"] is not None
                    and float(np.linalg.norm(np.asarray(o["point_mm"], dtype=float) - p)) <= self.same_spot_mm(o, e)):
                o["confirmed"] = True
                o["data"] = {**o["data"], "confirmed_by": e["id"]}
                self._log("world_update", id=o["id"], changes={"confirmed": True, "data": o["data"]})

    def _new_entry(self, kind, label, point_mm, source, confirmed, data, call, t_s) -> dict:
        try:
            p = _point(point_mm)
        except ValueError as e:
            return refusal("point_mm", str(e), "give [x, y, z] in mm, frame base")
        self._n += 1
        eid = f"{kind}_{self._n}"
        e = {
            "id": eid,
            "kind": kind,
            "label": str(label),
            "point_mm": p,
            "source": str(source),
            "observed_at": self._stamp(call, t_s),
            "confirmed": DEFAULT_CONFIRMED[kind] if confirmed is None else bool(confirmed),
            "stale": False,
            "data": jsonable(data or {}),
        }
        self.entries[eid] = e
        self._log("world_add", entry=e)
        self._agree(e)
        return copy.deepcopy(e)

    def get(self, eid):
        e = self.entries.get(eid)
        return None if e is None else copy.deepcopy(e)

    def update(self, eid, **fields) -> dict:
        """Change label, point_mm, source, data (merged) or confirmed. A new point clears stale.
        A state entry's confirmed flag moves only through verify."""
        e = self.entries.get(eid)
        if e is None:
            return refusal("id", f"no world entry {eid!r}", "query the world for ids")
        bad = sorted(set(fields) - set(UPDATABLE))
        if bad:
            return refusal("field", "cannot change " + ", ".join(bad), "updatable: " + ", ".join(UPDATABLE))
        if "confirmed" in fields and e["kind"] == "state":
            return refusal("verify_required", "a state is confirmed only by verify",
                           "verify it with a look taken after the arm last moved")
        if "point_mm" in fields:
            try:
                p = _point(fields["point_mm"])
            except ValueError as err:
                return refusal("point_mm", str(err), "give [x, y, z] in mm, frame base")
        changes = {}
        if "label" in fields:
            changes["label"] = str(fields["label"])
        if "source" in fields:
            changes["source"] = str(fields["source"])
        if "confirmed" in fields:
            changes["confirmed"] = bool(fields["confirmed"])
        if "point_mm" in fields:
            changes.update(point_mm=p, stale=False, observed_at=self._stamp())
        if "data" in fields:
            changes["data"] = {**e["data"], **jsonable(fields["data"] or {})}
        e.update(changes)
        self._log("world_update", id=eid, changes=changes)
        return copy.deepcopy(e)

    def query(self, kind=None, label=None, near_mm=None, within_mm=None, confirmed=None, include_stale=True):
        """Entries matching every given filter, in insertion order (nearest first with near_mm). A box is returned
        only when asked for by kind (it is an extent: readers of points never meet one)."""
        out = []
        for e in self.entries.values():
            if (kind is not None and e["kind"] != kind) or (kind is None and e["kind"] == "box"):
                continue
            if label is not None and e["label"].lower() != str(label).lower():
                continue
            if confirmed is not None and e["confirmed"] != bool(confirmed):
                continue
            if not include_stale and e["stale"]:
                continue
            out.append(copy.deepcopy(e))
        if near_mm is not None:
            c = np.asarray(near_mm, dtype=float)
            ranked = []
            for e in out:
                if e["point_mm"] is None:
                    continue
                d = float(np.linalg.norm(np.asarray(e["point_mm"]) - c))
                if within_mm is None or d <= within_mm:
                    ranked.append((d, e))
            ranked.sort(key=lambda de: de[0])  # stable: ties keep insertion order
            out = [dict(e, distance_mm=round(d, 3)) for d, e in ranked]
        return out

    def event(self, name, data=None, source="runtime") -> dict:
        return self.add("event", name, source=source, data=data)

    def world_moved(self, info=None, source="relocalize") -> dict:
        """Record that the robot's base or the scene moved. Every positional entry taken
        before it is marked stale: its base-frame point may no longer be where the thing is."""
        ev = self.event("world_moved", data=info or {}, source=source)
        staled = []
        for e in self.entries.values():
            if e["kind"] in STALEABLE and not e["stale"] and e["point_mm"] is not None:
                e["stale"] = True
                e["data"] = {**e["data"], "stale_since": ev["id"]}
                staled.append(e["id"])
        if staled:
            self._log("world_stale", ids=staled, since=ev["id"])
        return {"event": ev, "stale": staled}

    def mark_stale(self, ids, why, source="runtime") -> dict:
        """Mark positional entries stale because the thing they are on moved (a part that gave way, a device that
        changed state, a part that a move entered): their point may no longer be where the thing is, so anything
        that reads points with include_stale False (the device package's verify, the obstacles) leaves them out.
        Returns {"stale": ids marked}; ids that are not positional or already stale are skipped."""
        done = []
        for eid in ids or ():
            e = self.entries.get(str(eid))
            if e is None or e["kind"] not in STALEABLE or e["stale"]:
                continue
            e["stale"] = True
            e["data"] = {**e["data"], "stale_why": str(why)}
            done.append(e["id"])
        if done:
            self._log("world_stale", ids=done, why=str(why), source=str(source))
        return {"stale": done}

    def note_look(self, look_id, can_see, frame=None, call=None, t_s=None) -> dict:
        """Register a look so verify can check when it was taken and whether it could see."""
        return self.note_sense("look", look_id, can_see=can_see, frame=frame, call=call, t_s=t_s)

    def note_sense(self, sense, ref, summary=None, can_see=None, frame=None, call=None, t_s=None) -> dict:
        """Register one observation by a sense (look, touch, press, force, ...) under a reference id. Its time is
        the world clock's reading when one is set (clock: a callable the session sets; a look taken at the end
        of a move is stamped when it was taken), else the present call's start."""
        if t_s is None and callable(getattr(self, "clock", None)):
            try:
                t_s = float(self.clock())
            except Exception:  # a clock that fails must not lose the observation: the call's start stands
                t_s = None
        obs = {"sense": str(sense), "ref": str(ref), **self._stamp(call, t_s)}
        if can_see is not None:
            obs["can_see"] = str(can_see)
        if frame is not None:
            obs["frame"] = str(frame)
        if summary is not None:
            obs["summary"] = jsonable(summary)
        self.observations.append(obs)
        return copy.deepcopy(obs)

    def note_act(self, name=None) -> None:
        """Register that the arm moved in the present call (a motion, a touch, a press, a grip). The executor
        calls this while it moves; a session over an executor that does not calls it after a call during which
        simulated time passed. Evidence taken in an earlier call is stale from here on (verify)."""
        act = self.last_act
        if act is not None and act["call"] == self.now["call"]:
            return  # once per call: this runs every control period
        self.last_act = {"call": self.now["call"], "t_s": self.now["t_s"], "name": str(name or "a move")}

    def _find_obs(self, sense, ref):
        for o in reversed(self.observations):
            if o["sense"] == sense and o["ref"] == str(ref):
                return o
        return None

    def _open_claim(self, device):
        for e in reversed(list(self.entries.values())):
            if e["kind"] == "state" and e["device"] == device and e["mark"] != MARK_SUPERSEDED:
                return e
        return None

    def claim_state(self, device, state, source="operator", sense=None, evidence="", reuse=False) -> dict:
        """Record a device state the operator asserts. It is unconfirmed until verify and supersedes
        the device's open claim. A repeat of the open claim's state in the same call, or with
        reuse=True (verify naming a claim by device and state), returns the open claim instead."""
        device, state = str(device or ""), str(state or "")
        if not device or not state:
            return refusal("claim", "a claim needs a device and a state", "e.g. {device: lid, state: open}")
        old = self._open_claim(device)
        if old is not None and old["state"] == state and (reuse or old["observed_at"]["call"] == self.now["call"]):
            return copy.deepcopy(old)
        self._n += 1
        e = {
            "id": f"state_{self._n}",
            "kind": "state",
            "label": f"{device}: {state}",
            "point_mm": None,
            "source": str(source),
            "observed_at": self._stamp(),
            "confirmed": False,
            "stale": False,
            "data": {"evidence": str(evidence)},
            "device": device,
            "state": state,
            "sense": None if sense is None else str(sense),
            "mark": MARK_UNCONFIRMED,
            "checks": [],
            "confirmed_by": None,
            "superseded_by": None,
        }
        self.entries[e["id"]] = e
        if old is not None:
            old["mark"] = MARK_SUPERSEDED
            old["superseded_by"] = e["id"]
        self._log("state_claim", claim_id=e["id"], device=device, state=state, mark=MARK_UNCONFIRMED,
                  sense=e["sense"], supersedes=None if old is None else old["id"])
        return copy.deepcopy(e)

    def verify(self, claim) -> dict:
        """Confirm a state claim by a look, or by a second sense that agrees, taken after the arm last moved.

        claim: {"id": state entry id} or {"device", "state"[, "from_sense"]} (the open claim with that
        state is reused, else a new one is made), plus the evidence route {"look_id": ...} or
        {"by_sense": name, "ref": id} with {"evidence": what the operator saw}. With no route the claim is
        made (or found) and returned with its id, unconfirmed until evidence comes.
        The evidence is judged against the last call that moved the arm (last_act): a look or a sense taken in
        an earlier call shows the state before that call and is not accepted (check "stale"). So press, look,
        verify with that look confirms in one call; look, press, verify with that look does not.
        Refusals (nothing changes): malformed claim, a route with no evidence text, both routes, an unknown
        look or observation, a dark look.
        """
        if not isinstance(claim, dict):
            return refusal("claim", "verify takes an object", "{device, state, look_id, evidence}")
        evidence = str(claim.get("evidence") or "").strip()
        look_id, sense, ref = claim.get("look_id"), claim.get("by_sense"), claim.get("ref")
        if look_id is None and sense == "look":
            look_id, sense = ref, None
        if look_id is not None and sense is not None:
            return refusal("evidence_route", "give a look_id or a second sense, not both", "one route per verify")
        obs = None
        if look_id is not None or sense is not None:
            if not evidence:
                return refusal("evidence", "no stated evidence", "say what the look shows that decides the state")
            if look_id is not None:
                obs = self._find_obs("look", look_id)
                if obs is None:
                    return refusal("look_id", f"no look {look_id!r} in this session",
                                   "pass the look_id a look returned")
                if obs.get("can_see") == "dark":
                    return refusal("can_see", f"look {look_id} is dark: it cannot decide a state", DARK_HINT)
            else:
                obs = self._find_obs(str(sense), ref)
                if obs is None:
                    return refusal("sense", f"no {sense} observation {ref!r} in this session",
                                   "pass the id the sensing call returned")
        if claim.get("id") is not None:
            e = self.entries.get(str(claim["id"]))
            if e is None or e["kind"] != "state":
                return refusal("id", f"no state claim {claim['id']!r}", "query the world for kind state")
        elif claim.get("device") and claim.get("state"):
            r = self.claim_state(claim["device"], claim["state"], sense=claim.get("from_sense"), evidence=evidence,
                                 reuse=True)
            e = self.entries[r["id"]]
        else:
            return refusal("claim", "name the claim by id or by device and state", "{device, state, look_id, evidence}")
        if obs is None:
            return self._claimed(e)
        return self._decide(e, obs, evidence)

    def _result(self, e, accepted, why="", check="", by=None, checked=None) -> dict:
        """The verify answer. by is THIS call's evidence (the confirming one on a first confirmation); a claim that
        was confirmed before keeps that under first_confirmed_by; checks counts the verifies it has had; checked
        says what this call tested and basis what a confirmation rests on."""
        res = {"claim_id": e["id"], "device": e["device"], "state": e["state"], "confirmed": e["confirmed"],
               "mark": e["mark"], "accepted": accepted, "by": by if by is not None else e["confirmed_by"],
               "why": why}
        if e["confirmed_by"] is not None and e["confirmed_by"] != res["by"]:
            res["first_confirmed_by"] = e["confirmed_by"]
        if e["checks"]:
            res["checks"] = len(e["checks"])
        if checked is not None:
            res["checked"] = checked
        if e["confirmed"]:
            res["basis"] = BASIS
        if check:
            res["check"] = check
        if why and not e["confirmed"] and e["mark"] != MARK_SUPERSEDED:
            newest = next((o for o in reversed(self.observations) if o["sense"] == "look"), None)
            if newest is not None and newest.get("can_see") == "dark":   # no look can decide it now (G17)
                res["hint"] = f"the newest look ({newest['ref']}) is dark: {DARK_HINT}; claim id {e['id']}"
            else:
                res["hint"] = (f"take a look now from a view that shows the state, then verify id {e['id']} "
                               "with that look_id and what it shows")
        return res

    def _claimed(self, e) -> dict:
        """verify with no evidence route: the claim exists and its id is returned; nothing is confirmed here."""
        if e["confirmed"]:
            return self._result(e, True, checked=["no new evidence: the claim was confirmed before"])
        if e["mark"] == MARK_SUPERSEDED:
            return self._result(e, False, f"superseded by {e['superseded_by']}", "superseded")
        return self._result(e, False, "the claim is recorded; no look and no second sense was given yet", "no_evidence")

    def _decide(self, e, obs, evidence) -> dict:
        why, note, check = "", "", ""
        act = self.last_act
        if e["mark"] == MARK_SUPERSEDED:
            why, check = f"superseded by {e['superseded_by']}", "superseded"
        elif act is not None and obs["call"] < act["call"]:
            why = (f"the {obs['sense']} {obs['ref']} was taken at call {obs['call']}, before {act['name']} moved "
                   f"the arm at call {act['call']}: it shows the state before that call, not the state now")
            check = "stale"
        elif obs["sense"] != "look" and e["sense"] is None:
            why = "the claim names no sense, so a touch or press cannot be a second sense; confirm it by a look"
            check = "second_sense"
        elif obs["sense"] != "look" and obs["sense"] == e["sense"]:
            why, check = f"the claim came from {e['sense']}; a second sense must differ", "second_sense"
        if obs.get("can_see") == "dim":
            note = DIM_NOTE
        by = {"sense": obs["sense"], "ref": obs["ref"], "at_call": obs["call"], "evidence": evidence}
        accepted = not why
        e["checks"].append({**by, "accepted": accepted, "why": why})
        if accepted and not e["confirmed"]:
            e["confirmed"] = True
            e["mark"] = MARK_CONFIRMED
            e["confirmed_by"] = by
        res = self._result(e, accepted, why, check, by=by, checked=self._checked(obs, act, e))
        if note:
            res["note"] = note
        self._log("verify", claim_id=e["id"], device=e["device"], state=e["state"], mark=e["mark"],
                  accepted=not why, by=by, why=why)
        return res

    @staticmethod
    def _checked(obs, act, e) -> list:
        """What one verify tested, in words (S3): the evidence's time against the last move, the look's light or
        the second sense, and what it did not test."""
        when = (f"fresh: {obs['sense']} {obs['ref']} was taken at call {obs['call']}"
                + (f", not before the arm last moved (call {act['call']}, {act['name']})" if act is not None
                   else ", and the arm has not moved in this session"))
        if act is not None and obs["call"] < act["call"]:
            when = f"stale: {obs['sense']} {obs['ref']} was taken at call {obs['call']}, before call {act['call']}"
        out = [when]
        if obs["sense"] == "look":
            out.append(f"view: the look could see ({obs.get('can_see', 'not recorded')}), not dark")
            out.append(NOT_CHECKED)
        else:
            out.append(f"second sense: {obs['sense']}, the claim came from {e.get('sense')}")
            out.append("what the sense measured was not compared with the state: the state rests on your evidence")
        return out

    def device_states(self) -> dict:
        """The open (latest) claim per device: {device: {state, confirmed, mark, claim_id}}."""
        out = {}
        for e in self.entries.values():
            if e["kind"] == "state" and e["mark"] != MARK_SUPERSEDED:
                out[e["device"]] = {"state": e["state"], "confirmed": e["confirmed"], "mark": e["mark"],
                                    "claim_id": e["id"]}
        return out

    def unconfirmed(self) -> list:
        return [copy.deepcopy(e) for e in self.entries.values()
                if e["kind"] == "state" and e["mark"] == MARK_UNCONFIRMED]

    def to_json(self) -> dict:
        return jsonable({"now": self.now, "entries": list(self.entries.values()), "observations": self.observations,
                         "last_act": self.last_act, "next": self._n})

    @classmethod
    def from_json(cls, d, journal: Journal | None = None) -> "World":
        w = cls(journal=journal)
        w.now = dict(d.get("now") or {"call": 0, "t_s": 0.0})
        w.entries = {e["id"]: copy.deepcopy(e) for e in d.get("entries", [])}
        w.observations = copy.deepcopy(list(d.get("observations", [])))
        w.last_act = copy.deepcopy(d.get("last_act"))
        w._n = int(d.get("next", len(w.entries)))
        return w

def verify(world: World, claim) -> dict:
    """Module-level form of World.verify (SPEC: verify(claim))."""
    return world.verify(claim)
