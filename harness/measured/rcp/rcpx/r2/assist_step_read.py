# SPDX-License-Identifier: AGPL-3.0-only
"""What a robot-run step reads and meters: the readers of other tools' results, the meter with
the bounds of rule 7, the back-off of rule 8 and the same-view pair. A sibling file of rcpx/r2/assist_step.py, split
off for length.

The reading of every other tool's result is in ONE small function each (read_act, read_touch, read_record, cue_read,
look_before, spec_of), because the tools under this file are still changing: a renamed field is one fix. Other
packages are reached only through the executor's public tools and ex.state; a tool that is not loaded is said so.
Nothing here reads ground truth or names a robot, a device or a model.
"""
import math

from rcpx import r3_limits as L
from rcpx import sheet_words
from rcpx.r2 import assist_until as U

try:                                    # the rules of 4.7 (a sibling file); absent: contact tools refuse not_guarded
    from rcpx.r2 import assist_guard as GUARD
except Exception:                       # pragma: no cover - the file is built by another work package
    GUARD = None

MM = 1000.0
PKG = "assist_routine"
UNSAFE = ("firm", "further", "force_n", "max_n")        # a call this package WRITES never holds one (rule 1)
MOVED_WORDS = ("gave", "stayed_in", "came_back", "moving", "reached_end", "bottomed")
SPRANG_WORDS = ("came_back", "bottomed")
CUE_WORDS = {"steady": ("on", "lit", "steady"), "off": ("off", "dark"), "blinking": ("blinking",)}
OWN = ("own_base", "own_arm", "own_hand")


class Bound(Exception):
    """A bound of rule 7 was met before a nested call."""


# ---------------------------------------------------------------- small readers (one place each)
def state(ex):
    return ex.state.setdefault(PKG, {"moved": [], "meter": None, "lower": {}, "last": None, "ticks": False})


def guard_of(ex):
    """The bound guard: the assist package's (ex.state["assist"]["guard"]), else this file's own import, else None."""
    a = ex.state.get("assist")
    g = a.get("guard") if isinstance(a, dict) else None
    if g is None and GUARD is not None:
        try:
            g = GUARD.bind(ex)
        except Exception:
            g = None
    return g if isinstance(g, dict) else None


def manual(ex):
    doc = (getattr(ex, "context", None) or {}).get("devices")
    return sheet_words.parts(doc) if isinstance(doc, dict) else []


def roles(ex):
    return {p["name"]: p["role"] for p in manual(ex)}


def loaded(ex, name):
    try:
        return name in ex.primitive_names()
    except Exception:
        return callable(getattr(ex, name, None))


def tip_mm(ex):
    try:
        return [float(x) * MM for x in ex._sense().tip]
    except Exception:
        return None


def tool_load(ex):
    """(the force the tip feels now, the level under which the tool counts as off the part), the robot's readings."""
    cfg = getattr(ex, "cfg", None) or {}
    lim = cfg.get("eased_force_n") or cfg.get("touch_force_n") or 3.0 * L.NOISE_DEFAULT_N
    try:
        return float(ex._sense().Fmag), float(lim)
    except Exception:
        return None, float(lim)


def holding(ex):
    """The package whose state says a hold or grasp is active, else None."""
    for name, st in (getattr(ex, "state", None) or {}).items():
        if isinstance(st, dict) and (st.get("held") is True or st.get("holding") is True):
            return name
    return None


def budget(session):
    """(calls left, sim seconds left); None where the session has no budget of that kind."""
    rem = session.remaining() if callable(getattr(session, "remaining", None)) else {}
    return rem.get("calls_left"), rem.get("sim_seconds_left")


def look_rec(ex, look_id):
    """A look's record (joint pose q, time t, can_see) or None."""
    return (getattr(ex, "_looks", None) or {}).get(str(look_id))


def look_before(ex, t):
    """The newest look that can see, taken at or before sim time t (the picture before an act), or None."""
    best = None
    for k, r in (getattr(ex, "_looks", None) or {}).items():
        if r.get("can_see") != "dark" and r.get("t") is not None and float(r["t"]) <= float(t) + 1e-6:
            if best is None or float(r["t"]) >= float(look_rec(ex, best)["t"]):
                best = k
    return best


def unsafe(args):
    """True when a call this file would write holds an argument that raises a force (rule 1)."""
    if isinstance(args, dict):
        return any(k in UNSAFE or unsafe(v) for k, v in args.items())
    if isinstance(args, (list, tuple)):
        return any(unsafe(v) for v in args)
    return False


def ready(ex, tool, args, needs=None):
    """A next call: written for this client by the assist package when it is loaded, else {tool,
    args}. None for a call that would hold firm, a force or max_n: such a call is never written or passed on."""
    g = guard_of(ex)
    if unsafe(args) or (g is not None and callable(g.get("safe_call")) and g["safe_call"](tool, args)):
        return None
    a = ex.state.get("assist")
    fn = a.get("ready_call") if isinstance(a, dict) else None
    if callable(fn):
        try:
            out = fn(tool, args, "next", needs) if needs else fn(tool, args, "next")
        except Exception:
            out = None
        if isinstance(out, dict):
            return out
    out = {"tool": tool, "args": args}
    if needs:
        out["needs"] = needs
    return out


def read_act(r):
    """ONE place that reads a push or lift result: its stop, its felt list, its seen sentence."""
    r = r if isinstance(r, dict) else {}
    ref = r.get("refused") if isinstance(r.get("refused"), dict) else None
    felt = [f for f in (r.get("felt") or []) if isinstance(f, dict)]
    words = [f.get("word") for f in felt]
    last = felt[-1] if felt else {}
    wit = r.get("witness") if isinstance(r.get("witness"), dict) else {}
    con = r.get("contact") if isinstance(r.get("contact"), dict) else {}
    stopped = "refused" if ref else r.get("stopped")
    return {"refused": ref, "check": (ref or {}).get("check") or r.get("check"), "stopped": stopped, "felt": felt,
            "gave": stopped in ("changed", "reached_end") or "gave" in words,
            "moved_part": any(w in MOVED_WORDS for w in words), "sprang_back": last.get("word") in SPRANG_WORDS,
            "moving": "moving" in words, "along_push_mm": max([float(f.get("mm") or 0.0) for f in felt] or [0.0]),
            "small_mm": float(r.get("small_mm") or 0.0), "last_n": last.get("n"), "last_word": last.get("word"),
            "peak_n": r.get("peak_n"), "seen": r.get("seen"), "said": r.get("said"), "record": wit.get("id"),
            "before": wit.get("before"), "point_mm": con.get("point_mm"), "pictures": list(r.get("pictures") or [])}


def read_touch(r):
    """ONE place that reads a touch result: contact, where, how it stopped, whether the thing gave under it."""
    r = r if isinstance(r, dict) else {}
    ref = r.get("refused") if isinstance(r.get("refused"), dict) else None
    stopped = "refused" if ref else r.get("stopped")
    return {"refused": ref, "check": (ref or {}).get("check"), "stopped": stopped, "contact": bool(r.get("contact")),
            "point_mm": r.get("point_mm"), "gave": bool(r.get("gave")) or stopped == "moved_part",
            "moved_part": stopped == "moved_part", "sprang_back": False,
            "along_push_mm": float(r.get("along_push_mm") or 0.0), "peak_n": r.get("peak_force_total_n"),
            "id": r.get("id")}


def box_label(b):
    """The label the compare gives a box target (rcpx/r2/witness_pose.py targets): how a record row is matched."""
    return "box ({:.0f}, {:.0f}, {:.0f}, {:.0f})".format(*[float(x) for x in b[:4]])


def _overlap(a, b, grow=0.0):
    return not (a[2] < b[0] - grow or a[0] > b[2] + grow or a[3] < b[1] - grow or a[1] > b[3] + grow)


def spec_of(ex, target, look=None):
    """What a same-view compare judges for a target: {"at": the compare's own argument, "box": its pixel region in
    `look` or None, "id", "n"}. target: {"id"} (a located or contact entry) or {"n", "row"} (a candidate row, whose
    pixels are the list look's own)."""
    if target is None:
        return None
    row = target.get("row")
    if target.get("id") is None and isinstance(row, dict) and row.get("box_px") is not None:
        b = [float(x) for x in row["box_px"][:4]]
        return {"at": {"box_px": b}, "box": b, "id": None, "n": target.get("n"), "label": box_label(b)}
    eid = target.get("id")
    box = None
    e = ex.world.get(eid) if eid is not None else None
    rec = look_rec(ex, look) if look is not None else None
    if e is not None and e.get("point_mm") is not None and rec is not None:
        try:
            u, v, depth = ex.model.project(rec["q"], [float(x) / MM for x in e["point_mm"]])
            if depth > 0:
                r = float(L.REGION_MIN_PX + L.RING_PX)
                box = [float(u) - r, float(v) - r, float(u) + r, float(v) + r]
        except Exception:
            box = None
    return {"at": eid, "box": box, "id": eid, "n": target.get("n"), "label": None}


def read_record(ex, rec, acted, watch, rec2=None, looks=()):
    """ONE place that reads a witness record into the plain record assist_until judges: at rows with
    role acted | watch, changes with where they lie, on_own_body (the robot's own outline in either look) and
    seen_again (a light or colour change at the same place in a second look's record)."""
    if not isinstance(rec, dict):
        return None

    def role(it):
        for name, sp in (("acted", acted), ("watch", watch)):
            if sp is None:
                continue
            if (sp["id"] is not None and it.get("id") == sp["id"]) or \
                    (sp["label"] is not None and it.get("label") == sp["label"]):
                return name
        return None

    first_hit = (ex.state.get("selfsee") or {}).get("first_hit")

    def own(box):
        if not callable(first_hit) or box is None:
            return False            # the compare leaves the robot's own outline out of both frames itself
        u, v = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
        for k in looks:
            r = look_rec(ex, k)
            try:
                if r is not None and (first_hit(r["q"], u, v) or {}).get("what") in OWN:
                    return True
            except Exception:
                continue
        return False

    again = [c for c in ((rec2 or {}).get("changes") or []) if c.get("kind") in U.LIGHT_KINDS]
    at = [{"id": it.get("id"), "label": it.get("label"), "role": role(it), "changed": it.get("changed"),
           "kind": it.get("kind"), "colour_before": it.get("colour_before"), "colour_after": it.get("colour_after"),
           "glowing": it.get("glowing"), "moved_mm": it.get("moved_mm"), "why": it.get("why")}
          for it in rec.get("at") or []]
    changes = []
    for c in rec.get("changes") or []:
        box = c.get("box_px")
        where = None
        if box is not None and acted is not None and acted["box"] is not None \
                and _overlap(box, acted["box"], L.RING_PX):
            where = "acted"
        elif box is not None and watch is not None and watch["box"] is not None \
                and _overlap(box, watch["box"], L.REGION_MIN_PX):
            where = "watch"
        changes.append({"kind": c.get("kind"), "box": box, "at": where, "on_own_body": own(box),
                        "colour_before": c.get("colour_before"), "colour_after": c.get("colour_after"),
                        "seen_again": None if rec2 is None else any(_overlap(box, c2["box_px"]) for c2 in again
                                                                   if box is not None and c2.get("box_px"))})
    return {"id": rec.get("id"), "at": at, "changes": changes, "light_changed": bool(rec.get("light_changed")),
            "device_moved": rec.get("device_moved") or None, "said": rec.get("said")}


def witness_record(ex, rid):
    for r in (ex.state.get("witness") or {}).get("records") or []:
        if r.get("id") == rid:
            return r
    return None


# ---------------------------------------------------------------- the meter: seconds, travel, calls, bounds
class Meter:
    """What one robot-run tool call has used, and the bounds of rule 7 tested before every nested call. travel is
    the sum the bound is tested against: per nested call the larger of the tip's displacement across the call and
    the call's own travel number (a camera move back counts by its displacement; a move that returns to where it
    began counts only by its own number). path is the tip's measured path, summed every control period by a tick
    hook (None without the hook): reported beside travel, not bounded."""

    def __init__(self, ex, session, bounds, one_step=False):
        self.ex, self.session, self.bounds = ex, session, bounds
        self.t0, self.travel, self.steps, self.charged, self.did = float(ex.sim_time()), 0.0, 0, 0, []
        self.path = 0.0 if state(ex).get("ticks") else None
        self._tip = tip_mm(ex)
        self.lower = dict(state(ex).get("lower") or {})     # a test or an installation may LOWER a bound, never raise
        move = getattr(bounds, "move_bound_mm", None)
        if one_step and move is not None:                   # one step: travel of at most 1 x the reach radius
            self.lower["travel_mm"] = min(float(move), float(self.lower.get("travel_mm", move)))
        state(ex)["meter"] = self

    def close(self):
        if state(self.ex).get("meter") is self:
            state(self.ex)["meter"] = None

    def seconds(self):
        return float(self.ex.sim_time()) - self.t0

    def tick(self, sense):
        tip = getattr(sense, "tip", None)
        if tip is None or self.path is None:
            return
        tip = [float(x) * MM for x in tip]
        if self._tip is not None:
            self.path += math.dist(tip, self._tip)
        self._tip = tip

    def why(self, next_mm=0.0, new_step=False):
        """The bound that is met now ("steps" | "time" | "travel"), else None."""
        started = self.steps if new_step else max(0, self.steps - 1)
        w = None if self.bounds is None else self.bounds.before(started, self.seconds(), self.travel, next_mm)
        if w is None and self.lower.get("time_s") is not None and self.seconds() >= float(self.lower["time_s"]):
            w = "time"
        if w is None and self.lower.get("travel_mm") is not None \
                and self.travel + next_mm > float(self.lower["travel_mm"]):
            w = "travel"
        return w

    def check(self, next_mm=0.0):
        w = self.why(next_mm)
        if w is not None:
            raise Bound(w)

    def charge(self):
        """A charged nested call starts: the first is the session's own charge for this tool call; each further one
        lowers the call budget by one, so the total is what the same calls would cost the operator (rule 9)."""
        left = budget(self.session)[0]
        if self.charged >= 1 and left is not None and left <= 0:
            raise Bound("calls")
        self.charged += 1
        b = getattr(self.session, "budget", None)
        if self.charged > 1 and isinstance(b, dict) and b.get("calls") is not None:
            b["calls"] = int(b["calls"]) - 1

    def call(self, name, fn=None, **kwargs):
        """A public tool as the operator's own call (fn: a function another package serves through ex.state)."""
        t0, tip0 = float(self.ex.sim_time()), tip_mm(self.ex)
        r = (fn or getattr(self.ex, name))(**kwargs)
        r = r if isinstance(r, dict) else {"value": r}
        tip1 = tip_mm(self.ex)
        own = 0.0
        for k in ("travelled_mm", "moved_mm", "achieved_mm"):
            v = r.get(k)
            if isinstance(v, (list, tuple)) and len(v) == 3:
                v = math.sqrt(sum(float(x) ** 2 for x in v))
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                own = max(own, abs(float(v)))
        mm = max(own, math.dist(tip0, tip1) if tip0 is not None and tip1 is not None else 0.0)
        self.travel += mm
        ref = r.get("refused")
        self.did.append({"tool": name, "stopped": ("refused: " + str(ref.get("check"))) if isinstance(ref, dict)
                         else r.get("stopped"), "seconds": round(float(self.ex.sim_time()) - t0, 1),
                         "mm": round(mm, 0)})
        return r

    def used(self):
        out = {"seconds": round(self.seconds(), 1), "travel_mm": round(self.travel, 0), "calls": max(1, self.charged)}
        if self.path is not None:
            out["path_mm"] = round(self.path, 0)
        return out


def install_tick(ex):
    """setup: the tip's path is summed while a meter is open (nothing is read or stopped otherwise)."""
    st = state(ex)

    def tick(sense):
        m = st.get("meter")
        if m is not None:
            m.tick(sense)
        return None

    ex.add_tick(tick)
    st["ticks"] = True


def back_off(ex, meter, touched=False):
    """Every exit: the tool off the part (rule 8). The verb's own retreat ran inside the verb; when the tip still
    feels a load, or the last nested motion ended resting on a contact (a touch), one retreat step is taken (a
    retreat runs in the dark too). Returns end {tip_mm, backed_off} and a note when the back-off failed."""
    f, lim = tool_load(ex)
    note = None
    if (touched or (f is not None and f >= lim)) and loaded(ex, "retreat"):
        r = meter.call("retreat", steps=1)
        if "refused" in r or r.get("stopped") not in ("goal", None):
            note = "the back-off stopped (" + str((r.get("refused") or {}).get("check") or r.get("stopped")) + ")"
        f, lim = tool_load(ex)
    tip = tip_mm(ex)
    out = {"tip_mm": None if tip is None else [round(x, 1) for x in tip], "backed_off": bool(f is None or f < lim)}
    if note or not out["backed_off"]:
        out["note"] = note or f"the tip still feels {f:.1f} N"
    return out


# ---------------------------------------------------------------- the same view, before and after
def same_view_pair(ex, meter, ref, specs, second=False):
    """The camera back where look `ref` was taken (look like: a guarded move through its own gates), then the
    witness package's compare at specs. {"look", "record", "record2", "pictures"}, or {"refused": body} when a gate
    refused the move, or {"why": text} when no same view could be had (no witness package, the camera could not
    return). second: one more look from the same place and its compare (a change must still be there)."""
    wit = ex.state.get("witness") if isinstance(ex.state.get("witness"), dict) else {}
    cmp_fn = wit.get("compare")
    if not callable(cmp_fn) or not loaded(ex, "look"):
        return {"why": "no same view to compare with"}
    meter.check()
    lk = meter.call("look", like=ref)
    if isinstance(lk.get("refused"), dict):
        return {"refused": lk["refused"]}
    at = [s["at"] for s in specs if s is not None] or None
    cmp = cmp_fn(ref, lk.get("look_id"), at)
    if not isinstance(cmp, dict) or "refused" in cmp:
        return {"why": ((cmp or {}).get("refused") or {}).get("why") or "the two looks could not be compared",
                "look": lk.get("look_id")}
    out = {"look": lk.get("look_id"), "record": witness_record(ex, cmp.get("record")), "record2": None,
           "pictures": list(cmp.get("pictures") or [])[:L.PICTURES_MAX], "said": cmp.get("said")}
    if second:
        meter.check()
        lk2 = meter.call("look")
        cmp2 = cmp_fn(ref, lk2.get("look_id"), at) if "refused" not in lk2 else None
        if isinstance(cmp2, dict) and "refused" not in cmp2:
            out["record2"] = witness_record(ex, cmp2.get("record"))
            out["look2"] = lk2.get("look_id")
    return out


def cue_read(ex, meter, watch, word):
    """ONE place that reads the device package's cue (its watch tool): the state word, or None when it cannot be
    read. watch: a spec (spec_of) whose entry or box is watched in the present view."""
    if not loaded(ex, "watch") or watch is None:
        return None
    if watch["id"] is not None:
        e = ex.world.get(watch["id"])
        target = e["label"] if e is not None else None
    else:
        b = watch["box"]
        target = {"px": [(b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0],
                  "radius_px": max(1.0, min(b[2] - b[0], b[3] - b[1]) / 2.0)}
    if target is None:
        return None
    meter.check()
    r = meter.call("watch", target=target)
    names = CUE_WORDS.get(r.get("verdict"))
    if names is None:
        return None
    return word if word in names else names[0]


__all__ = ["Bound", "GUARD", "Meter", "back_off", "box_label", "budget", "cue_read", "guard_of", "holding",
           "install_tick", "loaded", "look_before", "look_rec", "manual", "read_act", "read_record", "read_touch",
           "ready", "roles", "same_view_pair", "spec_of", "state", "tip_mm", "tool_load", "unsafe", "witness_record"]
