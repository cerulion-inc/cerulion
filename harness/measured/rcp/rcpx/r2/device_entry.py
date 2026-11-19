# SPDX-License-Identifier: AGPL-3.0-only
"""Device entries for the device package: read robot://devices entries and this session's records.

An entry is a hypothesis (the robot://devices document the runner hands over in ex.context["devices"]). Every field
is optional: states, transitions [{from, act, to, within_s}], idle_off_s, long_press_s, power, and per part: part
(its name), states, how_to_tell, where_mm. Device kinds are never named here: a cue part is a part whose listed
states are cue words (off, on, lit, blinking, ...) and that the sheet gives no way to work (no direction, travel,
force, trigger or allowed approach: a part with states off and on is worked, not read); a deciding
part is a part whose states include the claimed state. Points come from robot://world (located entries labelled
with a part's name or the device id) or from this session's own acts on the device; only a located cue part is
read as the device's cue. Nothing here reads simulator ground truth.
"""

import re
from rcpx.r2 import device_bounds as derived

import numpy as np

CUE_WORDS = {"off", "on", "lit", "unlit", "dark", "steady", "blinking", "flashing"}
WORK_FIELDS = ("direction", "travel_mm", "travel_deg", "force_n", "trigger", "approach_allowed")


def entries(ex):
    doc = (getattr(ex, "context", None) or {}).get("devices")
    if isinstance(doc, dict):
        doc = doc.get("devices")
    return [d for d in (doc or []) if isinstance(d, dict)]


def ident(entry):
    return str(entry.get("id") or entry.get("name") or "")


def find(ex, key):
    key = str(key or "").strip().lower()
    for e in entries(ex):
        if key and key in (ident(e).lower(), str(e.get("name") or "").lower()):
            return e
    return None


def part_name(part):
    return str(part.get("part") or part.get("name") or part.get("label") or "")


def _tokens(text):
    return set(re.findall(r"[a-z0-9]+", str(text).lower()))


def label_part(label, entry):
    """(matches, part or None) for a world label against an entry: the device id or name itself, or a part's
    name (all of one name's words inside the other's, so "ring" names a part called "front ring"), with or without
    the id."""
    lab = _tokens(label)
    if not lab:
        return False, None
    idt, nm = _tokens(ident(entry)), _tokens(entry.get("name") or "")
    if lab == idt or (nm and lab == nm):
        return True, None
    rest = lab - idt if (idt and idt <= lab) else lab      # "<id> <part>" names the part
    for part in entry.get("parts") or []:
        pt = _tokens(part_name(part)) if isinstance(part, dict) else set()
        if pt and rest and (rest <= pt or pt <= rest):
            return True, part
    return False, None


def located(ex, entry):
    """World points of this device, newest first: [{label, point_mm, id, part, source}]. Stale ones are left out."""
    out = []
    for kind in ("located", "device"):
        for e in ex.world.query(kind=kind, include_stale=False):
            if e.get("point_mm") is None:
                continue
            ok, part = label_part(e["label"], entry)
            if ok:
                out.append({"label": e["label"], "point_mm": e["point_mm"], "id": e["id"], "part": part,
                            "source": e.get("source"), "at_call": (e.get("observed_at") or {}).get("call")})
    return list(reversed(out))


def act_points(st, dev_id, epoch):
    return [a["point_mm"] for a in reversed(st["acts"]) if a["device"] == dev_id and a["epoch"] == epoch
            and a.get("contact_ok") and a.get("point_mm") is not None]


def _states(obj):
    return [str(s).lower() for s in (obj.get("states") or []) if isinstance(s, (str, int, float))]


def worked(part):
    """True for a part the sheet says how to work (a direction, travel, force, trigger or allowed approach)."""
    return any(part.get(k) not in (None, "", [], {}) for k in WORK_FIELDS)


def cue_parts(entry):
    """Parts that only show the device's state: cue-word states and nothing on the sheet that works them."""
    return [p for p in entry.get("parts") or [] if isinstance(p, dict) and set(_states(p)) & CUE_WORDS
            and not worked(p)]


def is_cue_part(entry, part):
    return part is not None and any(p is part for p in cue_parts(entry))


def cue_located(ex, entry):
    """The newest located point of one of the device's cue parts ({label, point_mm, id, part, at_call}), or None.
    A point labelled with the device id, or with a part that is worked, is never taken for the cue."""
    for p in located(ex, entry):
        if is_cue_part(entry, p["part"]):
            return p
    return None


def cue_point(ex, entry):
    """(point_mm, label, part) of the device's cue (cue_located), or (None, None, None)."""
    p = cue_located(ex, entry)
    return (None, None, None) if p is None else (p["point_mm"], p["label"], p["part"])


def shown_by_cue(entry, state):
    """True when the sheet says the device's cue shows `state`: a cue part lists it, or it is a state of the device
    itself, the device has a cue part and no other part lists it."""
    state = str(state or "").lower()
    cues = cue_parts(entry)
    if any(state in _states(p) for p in cues):
        return True
    others = [p for p in entry.get("parts") or [] if isinstance(p, dict) and not any(p is c for c in cues)]
    return bool(cues) and state in _states(entry) and not any(state in _states(p) for p in others)


def state_point(ex, st, entry, state):
    """(point_mm, how, part) deciding a claimed state: the located point of a cue part whose states include it,
    else of any part whose states include it, else of the cue part when the state is the device's own, else any
    located point of the device, else the device's last act."""
    state = str(state or "").lower()
    pts = located(ex, entry)
    for cue_first in (True, False):
        for p in pts:
            if (p["part"] is not None and state in _states(p["part"])
                    and is_cue_part(entry, p["part"]) == cue_first):
                return p["point_mm"], f"located '{p['label']}' (a part that shows {state})", p["part"]
    if state in _states(entry):
        pt, lab, part = cue_point(ex, entry)
        if pt is not None:
            return pt, f"located '{lab}' (the device's cue)", part
    if pts:
        return pts[0]["point_mm"], f"located '{pts[0]['label']}' (a point on the device)", pts[0]["part"]
    acts = act_points(st, ident(entry), ex.epoch)
    if acts:
        return acts[0], "the contact point of the last act on the device", None
    return None, None, None


def num_range(entry, key):
    """(lo, hi) of a numeric field on the device or any of its parts (a number or a [lo, hi] range), or None."""
    for obj in [entry] + [p for p in entry.get("parts") or [] if isinstance(p, dict)]:
        v = obj.get(key)
        if isinstance(v, bool):
            continue
        if isinstance(v, (int, float)):
            return float(v), float(v)
        if isinstance(v, (list, tuple)) and v and all(isinstance(x, (int, float)) for x in v):
            return float(min(v)), float(max(v))
    return None


def part_size_mm(entry, part):
    """A size for the part in mm (diameter, else width, else height), or None when the entry gives none."""
    for obj in ([part] if part else []):
        w = obj.get("where_mm") or obj.get("size_mm") or {}
        if isinstance(w, dict):
            for key in ("diameter", "width", "height"):
                if isinstance(w.get(key), (int, float)):
                    return float(w[key])
    return None


def _within(t):
    v = t.get("within_s")
    if isinstance(v, (int, float)) and not isinstance(v, bool):
        return float(v)
    if isinstance(v, (list, tuple)) and v and all(isinstance(x, (int, float)) for x in v):
        return float(max(v))
    return 0.0


def settle_s(entry, state, prior=None):
    """Seconds after an act before a look can show `state`, from the entry's transitions: a transition to the
    state that an act starts takes its within_s; one that runs by itself (act null) adds the within_s of the act
    that leads into its start. With the prior state known, transitions from it are used; else the longest. An idle
    timeout (within_s at least the entry's idle_off_s) is not started by an act, so it counts only when no other
    transition reaches the state."""
    def low(x):
        return str(x or "").lower()

    trs = [t for t in entry.get("transitions") or [] if isinstance(t, dict)]
    to = [t for t in trs if low(t.get("to")) == low(state)]
    if not to:
        return 0.0
    idle = num_range(entry, "idle_off_s")
    if idle is not None:
        by_act = [t for t in to if _within(t) < idle[0]]
        to = by_act or to

    def leads(t):
        return [a for a in trs if a.get("act") and low(a.get("to")) == low(t.get("from"))]

    if prior:
        mine = [t for t in to if low(t.get("from")) == low(prior)
                or (not t.get("act") and any(low(a.get("from")) == low(prior) for a in leads(t)))]
        to = mine or to
    need = 0.0
    for t in to:
        cost = _within(t)
        if not t.get("act"):
            ins = [a for a in leads(t) if not prior or low(a.get("from")) == low(prior)] or leads(t)
            cost += max((_within(a) for a in ins), default=0.0)
        need = max(need, cost)
    return need


def attribute(ex, st, point_mm):
    """(entry, how) for an act at point_mm: the device with a known point nearest it within the device's extent and uncertainty; with no
    point known for any device and exactly one device listed, that one. (None, why) otherwise."""
    devs = entries(ex)
    if not devs or point_mm is None:
        return None, "no device entries" if not devs else "no act point"
    p = np.asarray(point_mm, float)
    best, any_known = None, False
    for e in devs:
        pts = [x["point_mm"] for x in located(ex, e)] + act_points(st, ident(e), ex.epoch)
        any_known = any_known or bool(pts)
        for q in pts:
            d = float(np.linalg.norm(p - np.asarray(q, float)))
            if d / 1000.0 <= derived.extent_m(derived.of(ex), e, pts) and (best is None or d < best[1]):
                best = (e, d)
    if best is not None:
        return best[0], f"{best[1]:.0f} mm from a known point of the device"
    if not any_known and len(devs) == 1:
        return devs[0], "the only device listed (no point of it is known yet)"
    return None, "no known device point within its extent and uncertainty"
