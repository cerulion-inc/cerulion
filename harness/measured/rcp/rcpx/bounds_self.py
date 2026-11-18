# SPDX-License-Identifier: AGPL-3.0-only
"""SelfKnow: what a robot keeps about itself between sessions, and nothing
about a task or a place.

Where (default_dir): the folder a session states (cfg selfknow_dir), else env RCPX_SELFKNOW_DIR, else
~/.rcpx/selfknow; never a default inside the code tree. One record per calibration, <directory>/<key>.json, then
<key>.2.json, <key>.3.json beside it: a record is NEVER written over (save); the newest is the one read (load).
The key is the sha1 of the robot's identity (identity(): the COMPILED body, not a file's bytes or path; the
control law with gains and period, the gripper block, a stated payload, a serial, gravity in the base frame,
firmware and voltage where readable): the same body in two folders reads one record; another robot, controller
or mounting never reads it. A record carries its VERSION and what measured it; one of another version is not
read (peek says what stands there). A write is atomic. Only a calibration writes (rcpx/bounds_selftest.py run).

Every entry is {value, source, at}; a table is rows {x: [tip speed m/s (, tip acceleration m/s^2)], y, source, at}.
Hidden levels and noises are read as an envelope (the largest value at or above the asked speed and acceleration: at
a speed not yet run, the value of the nearest faster speed run); lag, cross-track error and the measured run-on of a
commanded stop are read by straight lines through the rows. Hidden levels are refined UPWARD only and only from
readings where no contact is proven (note_free_leg, note_held_leg: a leg through free space that ended at its goal);
what a false stop showed stands for the session alone and is never written to the file (note_false_stop,
note_session). A value can be exactly zero (a simulator adds no noise): zero is a value, None is "not known".

Stops: add_stop keeps every stop measured by rcpx.bounds_stops.measure, a self-test tap ("where":
"tap", written to the file) or a task's contact ("task": this session alone). The detection time and the stop
slope are read from them. Braking and acceleration are stored per joint (brake_j, accel_j); no rise term.

Another module asks through get(name, *args) or the method of that name; each answers (value, source, at) and
(None, None, None) when the quantity is not known. SI inside. The one ratio used (REL) comes from rcpx.bounds_ratios.
The self-test that fills this is rcpx/bounds_selftest.py.
"""

import hashlib
import json
import os
import time
import uuid

import numpy as np

from rcpx import bounds_stops
from rcpx.bounds_ratios import QUANT, REL

VERSION = 4   # the key is the compiled body (its contact model and drives too); stops keep
              # their pose and body; taps start raised, at a second pose and with a link; the point held fast
SOURCES = (
    "rating", "model", "self_test", "contact", "seen", "swept",
    "word", "sheet", "intent", "stated", "prior", "site",
)
_GETTERS = frozenset((
    "qd_max", "qd_sources", "qdd_max", "tau_max", "f_tool", "quantum",
    "jitter", "e_tau", "n_det", "rise", "t_settle", "a_brake", "t_sample",
    "slope", "dt", "jaw", "camera", "fast", "took_s", "qd_track", "qd_rated",
    "t_det", "f_hidden", "sigma_f", "tau_hidden", "sigma_tau", "lag",
    "e_cross", "stop_time", "d_stop",
))
_ABSENT = (None, None, None)


def _canonical(value):
    if isinstance(value, np.ndarray):
        return _canonical(value.tolist())
    if isinstance(value, (float, np.floating)):
        return float(repr(float(value)))
    if isinstance(value, np.generic):
        return _canonical(value.item())
    if isinstance(value, dict):
        return {str(key): _canonical(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_canonical(item) for item in value]
    if value is None or isinstance(value, (bool, int, str)):
        return value
    raise TypeError("value cannot be made JSON-able")


def default_dir() -> str:
    """Where records live when a session states no folder: env RCPX_SELFKNOW_DIR, else ~/.rcpx/selfknow."""
    stated = os.environ.get("RCPX_SELFKNOW_DIR")
    return os.fspath(stated) if stated else os.path.join(os.path.expanduser("~"), ".rcpx", "selfknow")


def identity(model_bytes, law, gripper=None, payload=None, instance=None,
             gravity_base=None, firmware=None, voltage=None) -> dict:
    """The identity a record is keyed by. model_bytes: the compiled body (rcpx.bounds_selfstart.body_bytes)."""
    if not isinstance(model_bytes, bytes):
        raise TypeError("model_bytes must be bytes")
    return {
        "body_sha1": hashlib.sha1(model_bytes).hexdigest(),
        "law": _canonical(law), "gripper": _canonical(gripper),
        "payload": _canonical(payload), "instance": _canonical(instance),
        "gravity_base": _canonical(gravity_base),
        "firmware": _canonical(firmware), "voltage": _canonical(voltage),
    }


def key_of(ident) -> str:
    encoded = json.dumps(ident, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha1(encoded).hexdigest()


def _source(source):
    if source not in SOURCES:
        raise ValueError("invalid source")


def _larger(old, new):
    if old is None:
        return _canonical(new)
    if new is None:
        return _canonical(old)
    if isinstance(old, list) and isinstance(new, list):
        return [_larger(old[i] if i < len(old) else None,
                        new[i] if i < len(new) else None)
                for i in range(max(len(old), len(new)))]
    return new if new > old else old


def _floats(value):
    value = _canonical(value)
    if isinstance(value, list):
        return [float(item) for item in value]
    return float(value)


def _scale(value, factor):
    if isinstance(value, list):
        return [item * factor for item in value]
    return value * factor


def _linear(left, right, fraction):
    if isinstance(left, list):
        return [_linear(lo, hi, fraction) for lo, hi in zip(left, right)]
    return left + (right - left) * fraction


def _result(rows, value):
    source = ", ".join(sorted({row["source"] for row in rows}))
    return _canonical(value), source, rows[-1]["at"]


class SelfKnow:
    def __init__(self, key, directory=None, ident=None):
        self.key = key
        self.directory = default_dir() if directory is None else os.fspath(directory)
        self.ident = _canonical(ident)
        self.entries = {}
        self.tables = {}
        self.session = {}      # tables of this session alone (what false stops showed): never written to the file
        self.stop_list = []    # the stops the file holds (rcpx.bounds_stops.measure), each with "where" and "at"
        self.session_stops = []   # a task's stops: this session alone, never written to the file
        self.stored = None
        self.is_stale = False

    @property
    def ready(self):
        entry = self.entry("s1")
        return entry is not None and bool(entry["value"])

    @classmethod
    def records(cls, key, directory=None):
        """[(n, path)] of the records that stand for this key, oldest first (<key>.json, then <key>.N.json)."""
        directory = default_dir() if directory is None else os.fspath(directory)
        out = []
        try:
            names = os.listdir(directory)
        except OSError:
            return out
        for name in names:
            if name == key + ".json":
                out.append((1, os.path.join(directory, name)))
            elif name.startswith(key + ".") and name.endswith(".json") and name[len(key) + 1:-5].isdigit():
                out.append((int(name[len(key) + 1:-5]), os.path.join(directory, name)))
        return sorted(out)

    @property
    def path(self):
        """The record this knowledge was read from or written to; before either, where the first one would go."""
        if getattr(self, "_path", None):
            return self._path
        stand = self.records(self.key, self.directory)
        return stand[-1][1] if stand else os.path.join(self.directory, self.key + ".json")

    def to_doc(self):
        return _canonical({"version": VERSION, "key": self.key, "ident": self.ident,
                           "measured_by": getattr(self, "measured_by", None), "saved_at": self.stored,
                           "entries": self.entries, "tables": self.tables, "stops": self.stop_list})

    @classmethod
    def from_doc(cls, doc, *, directory=None):
        if doc["version"] != VERSION:
            raise ValueError("invalid version")
        result = cls(doc["key"], directory=directory, ident=doc.get("ident"))
        result.measured_by = doc.get("measured_by")
        entries, tables = doc["entries"], doc["tables"]
        if not isinstance(entries, dict) or not isinstance(tables, dict):
            raise ValueError("invalid entries or tables")
        for name, entry in entries.items():
            result.set(name, entry["value"], entry["source"], entry["at"])
        for name, rows in tables.items():
            if not isinstance(rows, list):
                raise ValueError("invalid rows")
            result.tables[name] = []
            for row in rows:
                result.add_row(name, row["x"], row["y"], row["source"], row["at"])
        for stop in doc.get("stops") or []:
            result.add_stop(stop, store=True)
        result.stored = doc.get("saved_at")
        result.is_stale = result.has("stale")
        return result

    def save(self, at=None):
        """Write this knowledge as a NEW record (never over one that stands: beside it, <key>.N.json) and return
        its path; self.beside names the record that stood, or None."""
        os.makedirs(self.directory, exist_ok=True)
        stand = self.records(self.key, self.directory)
        self.beside = stand[-1][1] if stand else None
        target = os.path.join(self.directory, self.key + (".json" if not stand else f".{stand[-1][0] + 1}.json"))
        self.stored = at if at is not None else time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        temporary = os.path.join(self.directory, f".{self.key}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
        try:
            with open(temporary, "x", encoding="utf-8") as stream:
                json.dump(self.to_doc(), stream, sort_keys=True, separators=(",", ":"))
                stream.flush()
                os.fsync(stream.fileno())
            os.link(temporary, target)        # fails if the target stands: never over a record
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        self._path = target
        return target

    @classmethod
    def peek(cls, key, directory=None):
        """{"path", "version", "measured_by", "saved_at", "records"} of the newest record, whatever its version."""
        stand = cls.records(key, directory)
        if not stand:
            return None
        try:
            with open(stand[-1][1], encoding="utf-8") as stream:
                doc = json.load(stream)
        except Exception:
            return None
        if not isinstance(doc, dict) or doc.get("key") != key:
            return None
        return {"path": stand[-1][1], "version": doc.get("version"), "measured_by": doc.get("measured_by"),
                "saved_at": doc.get("saved_at"), "records": len(stand)}

    @classmethod
    def load(cls, key, directory=None):
        """The newest record of this key and this VERSION, or None (none stands, or it is of another version)."""
        try:
            got = cls.peek(key, directory)
            if got is None or got["version"] != VERSION:
                return None
            with open(got["path"], encoding="utf-8") as stream:
                doc = json.load(stream)
            result = cls.from_doc(doc, directory=default_dir() if directory is None else os.fspath(directory))
            result._path = got["path"]
            return result
        except Exception:
            return None

    def set(self, name, value, source, at=None):
        _source(source)
        self.entries[name] = {"value": _canonical(value), "source": source, "at": at}

    def has(self, name):
        return name in self.entries

    def entry(self, name):
        return self.entries.get(name)

    def raise_to(self, name, value, source, at=None):
        _source(source)
        value = _canonical(value)
        old = self.entry(name)
        raised = value if old is None else _larger(old["value"], value)
        if old is not None and raised == old["value"]:
            return False
        self.set(name, raised, source, at)
        return True

    def _entry(self, name):
        entry = self.entry(name)
        if entry is None:
            return _ABSENT
        return entry["value"], entry["source"], entry["at"]

    def get(self, name, *args):
        if name in _GETTERS:
            return getattr(self, name)(*args)
        return self._entry(name)

    def add_row(self, table, x, y, source="self_test", at=None):
        _source(source)
        x = _floats(x)
        x = x if isinstance(x, list) else [x]
        if not x:
            raise ValueError("x must have a coordinate")
        y = _floats(y)
        rows = self.tables.setdefault(table, [])
        for row in rows:
            if row["x"] == x:
                raised = _larger(row["y"], y)
                if raised == row["y"]:
                    return False
                row.update(y=raised, source=source, at=at)
                return True
        rows.append({"x": x, "y": y, "source": source, "at": at})
        rows.sort(key=lambda row: row["x"])
        return True

    def rows(self, table):
        return _canonical(self.tables.get(table, []))

    def note_session(self, table, x, y, at=None):
        """A level a false stop showed: it stands for this session (envelope reads it), never in the stored file."""
        kept, self.tables = self.tables, self.session
        try:
            return self.add_row(table, x, y, source="contact", at=at)
        finally:
            self.tables = kept

    def add_stop(self, stop, where="task", at=None, store=None):
        """Keep a stop measured by rcpx.bounds_stops.measure: where is "tap" (the self-test) or
        "task". A record that is not one (no det_periods) is not kept. A false stop is never handed here. store:
        whether the file keeps it (none given: a tap yes, a task's stop no: it stands for this session)."""
        if not isinstance(stop, dict) or "det_periods" not in stop:
            return False
        kept = _canonical(dict(stop, where=stop.get("where", where), at=stop.get("at", at)))
        store = kept["where"] == "tap" if store is None else store
        (self.stop_list if store else self.session_stops).append(kept)
        return True

    def stops(self):
        """Every stop this session reads: the file's and the session's own."""
        return [dict(stop) for stop in self.stop_list + self.session_stops]

    def envelope(self, table, v=0.0, a=None):
        rows = self.tables.get(table, []) + self.session.get(table, [])
        if not rows:
            return _ABSENT
        used = [row for row in rows if row["x"][0] >= v and (
            a is None or len(row["x"]) == 1 or row["x"][1] >= a)]
        if not used:
            used = [row for row in rows if row["x"][0] >= v]
        if not used:
            used = rows
        value = used[0]["y"]
        for row in used[1:]:
            value = _larger(value, row["y"])
        return _result(used, value)

    def _interp(self, table, v, proportional=True):
        rows = sorted(self.tables.get(table, []), key=lambda row: row["x"][0])
        if not rows:
            return _ABSENT
        first, last = rows[0], rows[-1]
        if proportional and v <= 0:
            return _result([first], _scale(first["y"], 0.0))
        if v == first["x"][0]:
            return _result([first], first["y"])
        if v < first["x"][0]:
            value = _scale(first["y"], v / first["x"][0]) if proportional else first["y"]
            return _result([first], value)
        for left, right in zip(rows, rows[1:]):
            if v == right["x"][0]:
                return _result([right], right["y"])
            if v < right["x"][0]:
                fraction = (v - left["x"][0]) / (right["x"][0] - left["x"][0])
                return _result([left, right], _linear(left["y"], right["y"], fraction))
        value = last["y"]
        if proportional and last["x"][0] != 0:
            value = _scale(value, v / last["x"][0])
        return _result([last], value)

    def interp(self, table, v):
        return self._interp(table, v)

    def _qd(self):
        rated, track = self.entry("qd_rated"), self.entry("qd_track")
        ratings = [] if rated is None or rated["value"] is None else rated["value"]
        sweeps = [] if track is None or track["value"] is None else track["value"]
        values, sources = [], []
        for i in range(max(len(ratings), len(sweeps))):
            rating = ratings[i] if i < len(ratings) else None
            sweep = sweeps[i] if i < len(sweeps) else None
            if rating is not None and (sweep is None or rating <= sweep):
                values.append(rating)
                sources.append("rating")
            else:
                values.append(sweep)
                sources.append(None if sweep is None else "self_test")
        return values, sources, rated, track

    def qd_sources(self):
        return self._qd()[1]

    def qd_max(self):
        values, sources, rated, track = self._qd()
        if rated is None and track is None:
            return _ABSENT
        used = sorted({source for source in sources if source is not None})
        source = ", ".join(used) if used else ("rating" if rated is not None else "self_test")
        stamp = track["at"] if "self_test" in sources else (rated["at"] if rated else None)
        return values, source, stamp

    def qdd_max(self):
        return self._entry("qdd_max")

    def tau_max(self):
        return self._entry("tau_max")

    def f_tool(self):
        return self._entry("f_tool")

    def quantum(self):
        return self._entry("quantum")

    def jitter(self):
        return self._entry("jitter")

    def e_tau(self):
        return self._entry("e_tau")

    def n_det(self):
        return self._entry("n_det")

    def rise(self):
        return _ABSENT   # no rise term is stored or read: t_det() is what the stops showed

    def t_settle(self):
        return self._entry("t_settle")

    def a_brake(self):
        return _ABSENT   # no single braking number is stored or read: brake_j is per joint

    def t_sample(self):
        return self._entry("t_sample")

    def slope(self):
        """({"c", "c2", "kind", "n", "v0", "v1", "f0"}, source, at): how far the resolved stops stood over the
        body's model (bounds_stops.fit; c is never under 1: the slope is never under the body's model)."""
        got = bounds_stops.fit(self.stops())
        value = dict(got, c=max(1.0, got["c"] or 1.0), c2=None, kind="fitted" if got["n"] else "model")
        return value, self._stop_source(resolved=True), f"{got['seen']} resolved stops"

    def _stop_source(self, resolved=False):
        used = [s for s in self.stops() if (s.get("resolved") if resolved else s.get("det_periods"))]
        if not used:
            return "model"
        return "self_test" if any(s.get("where") == "tap" for s in used) else "contact"

    def dt(self):
        return self._entry("dt")

    def jaw(self):
        return self._entry("jaw")

    def camera(self):
        return self._entry("camera")

    def fast(self):
        return self._entry("fast")

    def took_s(self):
        return self._entry("took_s")

    def qd_track(self):
        return self._entry("qd_track")

    def qd_rated(self):
        return self._entry("qd_rated")

    def t_det(self, readings=None):
        """(seconds, source, text): the largest detection time the stops showed (bounds_stops.t_det); with no stop
        the guard's own readings times the period. No rise term."""
        n = self.n_det()[0] if readings is None else readings
        dt = self.dt()[0]
        if dt is None:
            return _ABSENT
        seconds, text, _ = bounds_stops.t_det(self.stops(), dt, QUANT if n is None else n)
        return seconds, self._stop_source(), text

    def _force(self, table, v, a, held):
        held_table = table + "_held"
        if held and self.tables.get(held_table):
            table = held_table
        return self.envelope(table, v, a)

    def f_hidden(self, v=0.0, a=0.0, held=False):
        return self._force("f_hidden", v, a, held)

    def sigma_f(self, v=0.0, a=0.0, held=False):
        return self._force("sigma_f", v, a, held)

    def tau_hidden(self, v=0.0):
        return self.envelope("tau_hidden", v)

    def sigma_tau(self, v=0.0):
        return self.envelope("sigma_tau", v)

    def lag(self, v):
        return self.interp("lag", v)

    def e_cross(self, v):
        return self.interp("e_cross", v) if self.tables.get("e_cross") else self.lag(v)

    def stop_time(self, v):
        return self._interp("stop_time", v, proportional=False)

    def d_stop(self, v, readings=None):
        value, source, at = self.interp("d_stop", v)
        if value is None:
            return _ABSENT
        if readings is not None:
            detection = self.t_det(readings)[0]
            if detection is None:
                return _ABSENT
            extra = v * detection
            value = [item + extra for item in value] if isinstance(value, list) else value + extra
        return value, source, at

    def note_free_leg(self, v, a, f_abs_max, f_sigma=None, tau_abs_max=None,
                      tau_sigma=None, lag_m=None, e_cross_m=None, swept=False,
                      ended=None, held=False, at=None):
        if swept is not True or ended != "goal" or held:
            return False
        self.add_row("f_hidden", [v, a], f_abs_max, at=at)
        if f_sigma is not None:
            self.add_row("sigma_f", [v, a], f_sigma, at=at)
        if tau_abs_max is not None:
            self.add_row("tau_hidden", [v], tau_abs_max, at=at)
        if tau_sigma is not None:
            self.add_row("sigma_tau", [v], tau_sigma, at=at)
        self.refit_lag(v, lag_m, e_cross_m)
        return True

    def note_held_leg(self, v, a, f_abs_max, f_sigma=None, at=None):
        self.add_row("f_hidden_held", [v, a], f_abs_max, at=at)
        if f_sigma is not None:
            self.add_row("sigma_f_held", [v, a], f_sigma, at=at)
        return True

    def note_false_stop(self, stop, a=0.0, f_n=None, held=False):
        """A stop no thing made: the level it showed stands for this session only (never in the stored file)."""
        if isinstance(stop, dict):
            v, a, held = stop.get("v", 0.0), stop.get("a", 0.0), stop.get("held", False)
            f_n = next((stop[name] for name in ("peak_n", "f_n", "force_n")
                        if stop.get(name) is not None), None)
        else:
            v = stop
        table = "f_hidden_held" if held else "f_hidden"
        if f_n is not None:
            self.note_session(table, [v, a], f_n)
        return self.envelope(table, v, a)[0]

    def refit_lag(self, v, lag_m=None, e_cross_m=None):
        changed = False
        for table, measured in (("lag", lag_m), ("e_cross", e_cross_m)):
            if measured is None:
                continue
            stored = self.interp(table, v)[0]
            if stored is None or measured > stored * (1 + REL):
                changed = self.add_row(table, [v], measured) or changed
        return changed

    def refit(self, stop):
        """A stop measured by rcpx.bounds_stops.measure is kept and the slope and the detection time
        are read from the kept stops, up and down (a task's stop is kept for this session, not in the file). Any
        other record changes no slope: a peak made of a deceleration and the speed of the reading that tripped are
        no measurement. Returns {"stored", "n": the resolved stops kept, "c": their largest ratio to the body's
        model, "stops": all kept}."""
        stored = self.add_stop(stop)
        overrun, v = stop.get("overrun_m"), stop.get("v")
        if overrun is not None and v is not None:
            self.stale(overrun, v)
        got = bounds_stops.fit(self.stops())
        return {"stored": stored, "n": got["seen"], "c": got["c"], "stops": len(self.stops())}

    def stale(self, overrun_m, v):
        distance = self.d_stop(v)[0]
        if distance is not None and overrun_m > distance * (1 + REL):
            self.is_stale = True
            self.set("stale", {"overrun_m": overrun_m, "v": v, "d_stop_m": distance}, "contact")
            return True
        return False


def of(ex):
    """The self-knowledge of an executor's robot, with no motion: what the session already holds, else the stored
    file of this robot and controller, else an empty SelfKnow (ready False) that the self-test fills. This is what
    the executor's Bounds reads at session open."""
    know = getattr(ex, "_selfknow", None)
    if know is None:
        from rcpx import bounds_selftest   # here, not at the top: that module imports this one

        know, loaded = bounds_selftest.know_of(ex)
        ex._selfknow = know
        if loaded and know.ready:
            bounds_selftest.apply_to_model(ex, know)
    return know
