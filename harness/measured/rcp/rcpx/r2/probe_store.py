# SPDX-License-Identifier: AGPL-3.0-only
"""Package probe, the store behind robot://experience: this episode's entries plus those of earlier ones.

The file is JSON {version, robot, episodes, entries}. Whatever it holds is treated as untrusted at load (it may
have been edited by hand, or written by another version): every entry is linted again, a field of the wrong type
falls back to its default, evidence keeps only ids, measured_by keeps only the call's id and name, and what is
left out is reported by its check alone, never by quoting it. So nothing read from the file can bring a pose in,
and no file content can make the load raise. No robot or device is named here.
"""

import json
import math
import os

from rcpx.r2.probe_memory import MAX_ITEMS, RE_ID, lint

VERSION = 1
STATUSES = ("hypothesis", "measured", "kept")


def _int(x, default=0):
    return int(x) if isinstance(x, (int, float)) and not isinstance(x, bool) and x == x and abs(x) < 1e9 else default


def clean_ids(ids):
    """The call ids in a list: short id strings and call numbers; anything else is left out."""
    out = []
    for x in ids if isinstance(ids, (list, tuple)) else []:
        if isinstance(x, bool):
            continue
        if isinstance(x, (int, str)) and RE_ID.match(str(x).strip()):
            out.append(str(x).strip())
    return list(dict.fromkeys(out))[:MAX_ITEMS]


def _measured_by(x):
    """What a file may say of the call that bore an entry out: its id and its name, nothing else."""
    if not isinstance(x, dict):
        return None
    call = clean_ids([x.get("call")])
    name = x.get("name") if isinstance(x.get("name"), str) and RE_ID.match(x["name"]) else None
    return {"call": call[0], "name": name, "why": "borne out in an earlier episode"} if call else None


class Store:
    """What was remembered: this episode's entries plus those read from the experience file of earlier ones."""

    def __init__(self, robot, joint_names=()):
        self.robot, self.joint_names = str(robot), tuple(joint_names)
        self.entries, self.path, self.episode = [], None, 1
        self.dropped, self.problem = [], None

    def _key(self, e):
        return json.dumps([e.get("scope"), e.get("kind"), e.get("frame") or "none", e.get("about"), e.get("value")],
                          sort_keys=True)

    def _read(self, path):
        if not path or not os.path.exists(path):
            return None
        with open(path, "r", encoding="utf-8") as f:
            doc = json.load(f)
        return doc if isinstance(doc, dict) and isinstance(doc.get("entries"), list) else None

    def load(self, path):
        """Read the experience file (if any); returns how many entries it gave. Every entry is linted again, so a
        file edited by hand cannot bring a pose in; robot scope entries written on another robot are left out."""
        self.path = str(path)
        try:
            doc = self._read(self.path)
        except (OSError, ValueError) as exc:
            self.problem = f"the experience file could not be read ({type(exc).__name__}); starting empty"
            return 0
        if doc is None:
            return 0
        self.episode = max(0, _int(doc.get("episodes"))) + 1
        have = {self._key(e) for e in self.entries}
        n = 0
        for raw in doc["entries"]:
            if not isinstance(raw, dict):
                self.dropped.append({"check": "shape", "why": "not an entry"})
                continue
            typed = {k: raw.get(k) for k in ("scope", "kind", "frame", "about", "value")}
            typed["frame"] = "none" if typed["frame"] is None else typed["frame"]
            bad = lint({k: v for k, v in typed.items() if not (k == "about" and v is None)}, self.joint_names)
            if bad:      # the check alone: the reason would quote what the entry held
                self.dropped.append({"check": bad[0], "why": f"an entry was left out by the {bad[0]} check"})
                continue
            robot = raw.get("robot") if isinstance(raw.get("robot"), str) else None
            if typed["scope"] == "robot" and robot not in (None, self.robot):
                self.dropped.append({"check": "robot", "why": "a robot entry written on another robot"})
                continue
            if self._key(typed) in have:
                continue
            have.add(self._key(typed))
            by = _measured_by(raw.get("measured_by"))
            measured = raw.get("measured") is True and by is not None      # measured needs the call that showed it
            status = raw.get("status") if raw.get("status") in STATUSES else "kept"
            if status == "measured" and not measured:
                status = "hypothesis" if typed["scope"] == "device_kind" else "kept"
            e = {**typed, "id": f"E{len(self.entries) + 1}", "evidence": clean_ids(raw.get("evidence")),
                 "status": status, "measured": measured, "measured_by": by if measured else None,
                 "episode": _int(raw.get("episode")), "robot": robot, "from_file": True}
            for key in ("force_hidden_n", "sigma_force_n"):
                value = raw.get(key)
                if isinstance(value, (int, float)) and math.isfinite(value) and value >= 0:
                    e[key] = value
            self.entries.append(e)
            n += 1
        return n

    def add(self, entry, status, measured, measured_by, evidence):
        typed = {k: entry.get(k) for k in ("scope", "kind", "frame", "about", "value")}
        typed["frame"] = typed["frame"] or "none"
        for e in self.entries:
            if self._key(e) == self._key(typed):       # said again: keep one, take the better standing
                if measured and not e["measured"]:
                    e.update(status=status, measured=True, measured_by=measured_by)
                e["evidence"] = list(dict.fromkeys(list(e["evidence"]) + list(evidence)))
                return e, False
        e = {**typed, "id": f"E{len(self.entries) + 1}", "evidence": list(evidence), "status": status,
             "measured": bool(measured), "measured_by": measured_by, "episode": self.episode, "robot": self.robot,
             "from_file": False}
        self.entries.append(e)
        return e, True

    def save(self):
        """Write the file (merged with what another episode may have written since the load). True when kept."""
        if not self.path:
            return False
        try:
            doc = self._read(self.path) or {}
        except (OSError, ValueError):
            doc = {}
        keep = [e for e in (doc.get("entries") or []) if isinstance(e, dict)]
        seen = {self._key(e): i for i, e in enumerate(keep)}
        for e in self.entries:
            row = {k: e[k] for k in ("scope", "kind", "frame", "about", "value", "evidence", "status", "measured",
                                     "measured_by", "episode", "robot")}
            row.update({k: e[k] for k in ("force_hidden_n", "sigma_force_n") if k in e})
            if self._key(e) in seen:
                keep[seen[self._key(e)]] = row
            else:
                seen[self._key(e)] = len(keep)
                keep.append(row)
        out = {"version": VERSION, "robot": self.robot, "entries": keep,
               "episodes": max(_int(doc.get("episodes")), self.episode)}
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
            tmp = f"{self.path}.tmp{os.getpid()}"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(out, f, indent=1, sort_keys=True)
            os.replace(tmp, self.path)
        except (OSError, TypeError, ValueError) as exc:
            self.problem = f"the experience file could not be written ({type(exc).__name__})"
            return False
        return True

    def doc(self):
        rows = [{k: e[k] for k in ("id", "scope", "kind", "frame", "about", "value", "status", "measured",
                                   "measured_by", "evidence", "episode")} | {"from_earlier_episode": e["from_file"]}
                for e in self.entries]
        out = {"robot": self.robot, "episode": self.episode, "persisted": bool(self.path), "entries": rows,
               "from_earlier_episodes": sum(1 for e in self.entries if e["from_file"]),
               "note": "what remember kept. Entries hold no pose: no joint angle, base frame point, pixel or look. A "
                       "hypothesis is unchecked. measured means only that a press cited in that episode reached the "
                       "stated force, or ran along the stated tool direction; it does not say the device changed "
                       "state. Evidence ids belong to the episode that wrote them."}
        if not self.path:
            out["note"] += " No experience file was given to this session: these entries end with it."
        if self.dropped:
            out["dropped_at_load"] = self.dropped
        if self.problem:
            out["problem"] = self.problem
        return out
