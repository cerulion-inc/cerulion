# SPDX-License-Identifier: AGPL-3.0-only
"""What the bounds know of a thing (A3, A8.1): its sheet, its error, its clearance, the Thing a reference names
(thing_of), and what a contact stop writes on it (note_stop, note_give).

The space book itself is rcpx/bounds_space.py (measured things and the stated support only).
"""

import math
from dataclasses import asdict
import numpy as np
from rcpx import bounds_laws as laws
from rcpx.bounds_laws import TWO
from rcpx.bounds_limit import gave
from rcpx.bounds_report import MM
from rcpx.bounds_robot import Thing, note_joint_hidden


def thing_sheet(world, E, intent):
    """Read a thing's live device sheet and the call's stated sheet."""
    data = next(((e.get("data") or {}) for e in world.query(include_stale=True)
                 if E is not None and e.get("id") == E.id), {})
    sheet = dict(data.get("sheet") or {})
    sheet.update((intent or {}).get("sheet") or {})
    sheet.setdefault("low_n", data.get("sheet_low_n"))
    sheet.setdefault("max_n", data.get("sheet_high_n"))
    force = sheet.get("force_n")
    if force is not None:
        values = np.asarray(force, dtype=float).reshape(-1)
        sheet["low_n"], sheet["max_n"] = float(values.min()), float(values.max())
    return sheet


def entry_sigma(data):
    """A world entry's position error in m: what a contact stored (sigma_p_mm); what a label's marks are worth
    (sigma_mm: the larger of along the view and across, None where the marks fix no depth; the error_mm beside
    it is only how far the rays miss each other); else what a look or a contact stored (uncertain_mm,
    error_mm); None when the entry carries none."""
    worth = (data or {}).get("sigma_mm")
    if isinstance(worth, dict):
        sides = [worth.get("along_view"), worth.get("across")]
        return None if any(v is None for v in sides) else max(float(v) for v in sides) / MM
    for key in ("sigma_p_mm", "uncertain_mm", "error_mm"):
        value = (data or {}).get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return value / MM
    return None


def point_segment(p, a, b):
    p, a, b = (np.asarray(v, dtype=float) for v in (p, a, b))
    d = b - a
    den = float(d @ d)
    t = min(1, max(0, float((p - a) @ d) / den)) if den > 0 else 0.0
    return float(np.linalg.norm(p - a - t * d))


def point_clearance(p, thing, radius=0.0):
    p = np.asarray(p, dtype=float)
    extent = thing.extent or {}
    if "plane" in extent:
        n, offset = extent["plane"]
        n = np.asarray(n, dtype=float)
        return float(n @ p - offset) / float(np.linalg.norm(n)) - radius
    if "box" in extent:
        lo, hi = (np.asarray(v, dtype=float) for v in extent["box"])
        return float(np.linalg.norm(p - np.clip(p, lo, hi))) - radius
    return math.inf if thing.point is None else float(np.linalg.norm(p - thing.point)) - radius


def cloud_clearance(P, thing):
    """Clearance of every row of P (N x 3) from the thing, in one pass."""
    P = np.asarray(P, dtype=float)
    extent = thing.extent or {}
    if "plane" in extent:
        n, offset = extent["plane"]
        n = np.asarray(n, dtype=float)
        return (P @ n - offset) / float(np.linalg.norm(n))
    if "box" in extent:
        lo, hi = (np.asarray(v, dtype=float) for v in extent["box"])
        return np.linalg.norm(P - np.clip(P, lo, hi), axis=1)
    if thing.point is None:
        return np.full(len(P), math.inf)
    return np.linalg.norm(P - np.asarray(thing.point, dtype=float), axis=1)


def segment_clearance(a, b, thing, radius=0.0):
    extent = thing.extent or {}
    if "plane" in extent:
        return min(point_clearance(a, thing, radius), point_clearance(b, thing, radius))
    if "box" not in extent:
        return math.inf if thing.point is None else point_segment(thing.point, a, b) - radius
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    lo_box, hi_box = (np.asarray(v, dtype=float) for v in extent["box"])
    d, lo, hi = b - a, 0.0, 1.0
    while True:
        t = (lo + hi) / TWO
        if t <= lo or t >= hi:
            return point_clearance(a + t * d, thing, radius)
        p = a + t * d
        derivative = float(d @ (p - np.clip(p, lo_box, hi_box)))
        if derivative == 0:
            return point_clearance(p, thing, radius)
        if derivative < 0:
            lo = t
        else:
            hi = t


def segment_capsules(a, b, capsules):
    """(clearance of the segment a to b from the nearest of the capsules [(p0, p1, r)], the point of that
    capsule's face nearest the segment): the closed form of the least distance between two segments."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    best, at = math.inf, None
    for p0, p1, r in capsules:
        p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
        d1, d2, w = b - a, p1 - p0, a - p0
        aa, bb, cc, dd, ee = float(d1 @ d1), float(d1 @ d2), float(d2 @ d2), float(d1 @ w), float(d2 @ w)
        den = aa * cc - bb * bb
        s = min(1.0, max(0.0, (bb * ee - cc * dd) / den)) if den > 0 else 0.0
        t = min(1.0, max(0.0, (bb * s + ee) / cc)) if cc > 0 else 0.0
        s = min(1.0, max(0.0, (bb * t - dd) / aa)) if aa > 0 else 0.0
        on_seg, on_cap = a + s * d1, p0 + t * d2
        gap = float(np.linalg.norm(on_seg - on_cap))
        if gap - r < best:
            way = (on_seg - on_cap) / gap if gap > 0 else np.zeros_like(a)
            best, at = gap - float(r), on_cap + float(r) * way
    return best, at


def thing_of(B, ref):
    """The Thing a reference names (a Thing, a dict of its fields, the word support, a world entry's id or label,
    a point), with what this session recorded of it."""
    if isinstance(ref, Thing):
        return ref
    data = {}
    if isinstance(ref, dict):
        data = {k: ref.get(k) for k in Thing.__dataclass_fields__ if k in ref}
    elif isinstance(ref, str):
        if ref == "support":
            support = B.space.support()
            if support is not None:
                plane, sigma, source = support
                data = {"id": ref, "sigma_p": sigma, "extent": {"plane": plane}}
                n = np.asarray(plane[0], dtype=float)
                p = n * plane[1] / float(n @ n)
                near = B.space.kind(p, p, 0)[1]
                near = asdict(near) if isinstance(near, Thing) else near
                if isinstance(near, dict) and near.get("id") == "support":
                    data.update({k: v for k, v in near.items() if k in Thing.__dataclass_fields__})
        else:
            entries = B.ex.world.query(include_stale=True)
            entry = next((v for v in entries if v.get("id") == ref or v.get("label") == ref), None)
            if entry is not None:
                d = entry.get("data") or {}
                data = {"id": entry["id"], "point": None if entry.get("point_mm") is None else np.asarray(entry["point_mm"], dtype=float) / MM,
                        "extent": d.get("extent"), "touched": d.get("touched", False)}
                mapping = {"k_c": "k_c_n_per_m", "end": "end_mm", "mass_kg": "mass_kg"}
                mapping.update({k: k + "_n" for k in ("f_bear", "f_gave", "f_moved", "f_worked")})
                for name, key in mapping.items():
                    value = d.get(key)
                    data[name] = np.asarray(value) / MM if value is not None and name == "end" else value
                data["sigma_p"] = entry_sigma(d)
        data.setdefault("id", ref)
    elif ref is not None:
        data = {"point": np.asarray(ref, dtype=float)}
    data.update({k: v for k, v in B._records.get(data.get("id"), {}).items() if k in Thing.__dataclass_fields__})
    return Thing(**data)


def note_give(B, thing, force_n, why):
    """A give of this session: a thing held as fixed moved, a hold was lost at a known squeeze, or the
    operator says harm. It bounds later contacts with that thing and with things of the same stated label."""
    entry = B._entry(thing) or {}
    ref = thing if isinstance(thing, str) else entry.get("id", getattr(thing, "id", None))
    give = {"id": entry.get("id", ref), "label": entry.get("label"), "force_n": None if force_n is None else float(force_n), "why": str(why)}
    B.gives.append(give)
    return give


def over_drives(B, q, u, peak_n, rest_n):
    """{push_n, read_n, words} when a stop's reading is over what the drives can push at the tip along
    u (bounds_book_read.drives_push_n: each joint's torque limit over its lever) by more than the torque model's
    own error (e_tau) and the reading is gone at rest (rest_n at or under the free threshold); else None."""
    if peak_n is None or u is None or not math.isfinite(float(peak_n)):
        return None
    from rcpx import bounds_book_read
    try:
        e_tau = float(np.max(np.asarray(B.e_tau(), dtype=float)))
    except Exception:          # bounds with no torque error figure: none is added
        e_tau = 0.0
    over, push, words = bounds_book_read.over_drives(B.ex, float(peak_n), q, u, margin=e_tau)
    thr = B.f_thr(0, 0)
    thr = float(thr.get("value") if isinstance(thr, dict) else (thr[0] if isinstance(thr, tuple) else thr))
    gone = rest_n is None or float(rest_n) <= thr
    if not over or not gone or push is None or not math.isfinite(push):
        return None
    return {"push_n": float(push), "read_n": float(peak_n),
            "words": (f"the stop read {float(peak_n):.1f} N, over the {float(push):.1f} N the drives can push along "
                      f"the travel ({words}, with the torque model's error {e_tau:.3g}), and the reading was gone at "
                      "rest: no thing pushed so; a false stop, nothing stored")}


def note_stop(B, thing, q, u, v, peak_n, impulse_ns, resolved, moved_m, held=False, rest_n=None, k_c=None,
              ext_peak=None, det_periods=None, expect=None, stop=None):
    """A contact stop. stop: the dict of bounds_stops.measure when the caller has it. A false stop is
    counted and never stored. Every other stop goes to the robot's own list of stops (slope, detection time).
    expect (the leg's expected contact): only that region changes, no event is stamped, and the touch goes on
    the thing's own record. A give is written only for a thing held as fixed."""
    mass = B.m_eff(q, u)
    whole = mass * v if mass > 0 and math.isfinite(mass) and v > 0 else None
    ratio = impulse_ns / whole if whole and impulse_ns is not None else None
    drives = over_drives(B, q, u, peak_n, rest_n)
    if drives is not None:
        # a reading over what the drives can push along the travel (their torque limits, less nothing:
        # with the torque model's own error added), gone at rest, is no thing's push: a false stop. It is counted,
        # the session's level is raised to what the drives can push there (no level above that is ever needed:
        # the cap stops there), and nothing is stored
        B.false_stops += 1
        getattr(B.know, "note_false_stop", B.model_know.note_false_stop)(
            {"v": v, "peak_n": min(float(peak_n), drives["push_n"]), "held": held, "rest_n": rest_n})
        return {"false": True, "over_drives": drives, "stored": {}, "ratio_m_eff": ratio}
    if impulse_ns is not None and laws.false_stop(impulse_ns, B.sigma_f(v, 0, held), B.t_det(), rest_n, B.f_thr(0, 0), whole):
        B.false_stops += 1
        getattr(B.know, "note_false_stop", B.model_know.note_false_stop)({"v": v, "peak_n": peak_n, "held": held, "rest_n": rest_n})
        note_joint_hidden(B.know, v, ext_peak)   # a joint's reading tripped it: that level rises too
        return {"false": True, "stored": {}, "ratio_m_eff": ratio}
    kept = {"v": v, "peak_n": peak_n, "resolved": bool(resolved), "det_periods": det_periods, "impulse_ns": impulse_ns,
            "m_eff": mass, "b_per_s": B._contact_rate(), "held": held, "rest_n": rest_n, **(stop or {})}
    kept.setdefault("model_z", B.model_z(q, u))
    add = getattr(B.know, "add_stop", None)
    if callable(add):
        add(kept)
    else:   # compat shim: until KNOW's add_stop is merged the session keeps the list itself
        B.stops.append(kept)
    if thing is None and isinstance((expect or {}).get("with"), str):
        thing = expect["with"]
    E = B.thing(thing) if thing is not None else None
    stored = {"sigma_p": laws.contact_error(v, B.t_det(), B.quantum(q, u)), "touched": True}
    rigid = B.rigid(q, u, peak_n)
    if k_c is not None:
        stored["k_c"] = k_c
    force = peak_n if peak_n is not None else kept.get("read_n")
    entry = B._entry(E.id) if E is not None and E.id is not None else None
    fixed = E is not None and (E.id == "support" or bool(((entry or {}).get("data") or {}).get("fixed"))
                               or (E.end is not None and moved_m is not None and moved_m > float(np.linalg.norm(E.end)) + rigid))
    give = None
    if force is not None and gave(fixed, moved_m, rigid):
        stored["f_gave"] = min(E.f_gave, force) if E.f_gave is not None else force
        give = note_give(B, E.id, force, f"it is held as fixed and moved {moved_m * MM:.3g} mm under a stop")
    elif force is not None and (moved_m is None or moved_m <= rigid):
        stored["f_bear"] = max(E.f_bear or 0, force) if E else force   # what it bore: a fact, never a bound
    if expect is not None:
        B.space.drop("contact", region=expect)      # the region's own change: no event is stamped
    elif E is None:
        B.space.drop("contact")
    if E is not None and E.id is not None:
        B._records.setdefault(E.id, {}).update(stored)
        if E.id != "support":
            data = {"sigma_p_mm": stored["sigma_p"] * MM, "touched": True}
            data.update({key + "_n": stored[key] for key in ("f_bear", "f_gave") if key in stored})
            if "k_c" in stored:
                data["k_c_n_per_m"] = stored["k_c"]
            B.ex.world.update(E.id, data=data)
    return {"false": False, "stored": stored, "ratio_m_eff": ratio, "give": give}
