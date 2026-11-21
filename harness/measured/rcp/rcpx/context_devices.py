# SPDX-License-Identifier: AGPL-3.0-only
"""Practices and device knowledge of the session context; rcpx/context.py re-exports every name here.

A device entry is a hypothesis with a source. It never carries ground truth or the present state. What a source
leaves out is said outright: every served part lists under not_given which of PART_FIELDS it does not give, so an
operator can tell "the manual does not say" from "there is nothing to say".

At manual level (what a user manual says) a part keeps its words, where included (where it sits, in words), and
gains rough sizes (size_mm: the sizes of the device and of each part as a manual's drawing gives them, rounded;
never a position, a travel or a force). The texts name no tool: which tools a session has
is the surface's to say (rcpx/surfaces.py).
"""

from __future__ import annotations

from rcpx.surface_session import SLOTS

from pathlib import Path

from rcpx.context_base import TRUTH_MARKER, jsonable, refusal

PRACTICES_PATH = Path(__file__).with_name("practices.md")


def practices_text() -> str:
    """robot://practices: the session's first read."""
    return PRACTICES_PATH.read_text(encoding="utf-8")


LEVELS = ("full", "manual", "none")
LEVEL_SOURCE = {"full": "device sheet", "manual": "user manual"}
# What a user manual says about a part: what it is, where it sits (in words), which way it goes, where to push
# from, how to tell its state. No numbers but rough sizes (ROUGH_*): travel, force and position are left to the
# operator to find out.
MANUAL_KEYS = ("part", "name", "label", "kind", "where", "moves", "direction", "approach_allowed",
               "approach_forbidden", "how_to_tell", "state_cue", "states", "notes")
WORDS_ONLY = ("where",)   # kept at manual level only as words (a text or a list of texts), never numbers
# A size key that names where something is rather than how big it is: never served as a rough size.
POSITION_WORDS = ("above", "pivot", "distance", "axis_height", "centre_height", "center_height",
                  "behind", "gap")   # where a part sits behind a face or a bar
ROUGH_NOTE = ("size_mm at manual level is rough, as a manual's drawing gives it (rounded, within about 20 percent); "
              "it gives no position, travel or force")
NEVER_KEYS = ("state", "current_state", "qpos", "value")  # the present state is never knowledge
# Device-level keys a user manual keeps (no sizes, no geometry).
MANUAL_DEVICE_KEYS = ("name", "label", "kind", "notes", "source")
# What robot://devices says a part entry holds: the name the resource text uses, and the keys that give it. A
# name none of whose keys is in a served part goes into that part's not_given list.
PART_FIELDS = {
    "kind": ("kind",),
    "moves": ("moves",),
    "direction": ("direction",),
    "approach_allowed": ("approach_allowed",),
    "approach_forbidden": ("approach_forbidden",),
    "how_to_tell": ("how_to_tell", "state_cue"),
    "where": ("where",),
    "position": ("where_mm", "point_mm"),
    "travel": ("travel_mm", "travel_deg"),
    "force": ("force_n",),
}
NOT_GIVEN_NOTE = ("not_given lists what this source does not say about a part: where (in words) and position "
                  "(numbers) are found by looking and by a light contact; travel and force are found by the push "
                  "itself: RCP offers the first force (" + SLOTS["f_thr"] + ", or what the light contact that "
                  "found the part already put on it) and raises it in one ramp until the part moves")


def not_given(part: dict) -> list:
    """The PART_FIELDS names this part entry does not give (in table order)."""
    return [name for name, keys in PART_FIELDS.items() if not any(k in part for k in keys)]


def _field(obj, key, default=None):
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _sheet(knowledge, level):
    """(device-level fields, parts) from any accepted knowledge shape: a callable(level), a dict keyed by
    level (manual falls back to full, filtered), {"parts": [...], ...device fields}, one part dict, or a list."""
    k = knowledge(level) if callable(knowledge) else knowledge
    if isinstance(k, dict) and any(lv in k for lv in LEVELS):
        k = k.get(level, k.get("full"))
    if k is None:
        return {}, []
    if isinstance(k, dict):
        if "parts" in k:
            return {kk: v for kk, v in k.items() if kk not in ("parts", "level")}, list(k["parts"] or [])
        return {}, [k]
    return {}, list(k)


def _words(v):
    return isinstance(v, str) or (isinstance(v, list) and all(isinstance(x, str) for x in v))


def _clean(d, level, manual_keys):
    return {k: v for k, v in d.items()
            if TRUTH_MARKER not in str(k).lower() and k not in NEVER_KEYS and (level == "full" or k in manual_keys)
            and (level == "full" or k not in WORDS_ONLY or _words(v))}


def rough(v):
    """A size as a drawing gives it: to 10 mm from 100 mm, to 5 mm from 20 mm, to 1 mm below."""
    step = 10.0 if v >= 100 else 5.0 if v >= 20 else 1.0
    return int(max(step if v > 0 else 0.0, round(v / step) * step))


def rough_sizes(sizes):
    """{name: rough mm} of the sizes in a sheet's size dict: numbers only, never a key that names a position."""
    if not isinstance(sizes, dict):
        return {}
    return {k: rough(float(v)) for k, v in sizes.items()
            if isinstance(v, (int, float)) and not isinstance(v, bool) and v > 0 and "tolerance" not in k
            and TRUTH_MARKER not in str(k).lower() and not any(w in str(k) for w in POSITION_WORDS)}


def _drawing(knowledge):
    """(device sizes, {part name: sizes}) from the full sheet, as rough sizes; empty when there is no sheet."""
    try:
        head, parts = _sheet(knowledge, "full")
    except (TypeError, ValueError, KeyError):
        return {}, {}
    by_part = {}
    for p in parts:
        if isinstance(p, dict) and p.get("part") is not None:
            r = rough_sizes(p.get("where_mm")) or rough_sizes(p.get("size_mm"))
            if r:
                by_part[str(p["part"])] = r
    return rough_sizes(head.get("size_mm")), by_part


def device_knowledge(scene, level: str = "full") -> dict:
    """robot://devices: per device {id, name, source, hypothesis, parts: [...]}, each part a
    hypothesis with a source and a not_given list (see PART_FIELDS).

    full: the device sheet as written; manual: only qualitative keys (MANUAL_KEYS, MANUAL_DEVICE_KEYS);
    none: no devices. Keys naming ground truth or the present state are always dropped. Entries may be
    incomplete or wrong.
    """
    if level not in LEVELS:
        return refusal("level", f"unknown knowledge level {level!r}", "use full, manual or none")
    if level == "none":
        return {"level": level, "devices": [],
                "note": "no device knowledge for this task: find the device by looking and name it yourself"}
    devs = _field(scene, "devices", []) or []
    if isinstance(devs, dict):
        devs = list(devs.values())
    out = []
    for d in devs:
        dev_id = str(_field(d, "name", "device"))
        try:
            head, parts = _sheet(_field(d, "knowledge"), level)
        except (TypeError, ValueError, KeyError) as err:
            return refusal("device_knowledge", f"device {dev_id}: {err}", "the device sheet is malformed")
        head = _clean(head, level, MANUAL_DEVICE_KEYS)
        src = str(head.pop("source", LEVEL_SOURCE[level]))
        dev_sizes, part_sizes = _drawing(_field(d, "knowledge")) if level == "manual" else ({}, {})
        doc = {"id": dev_id, **head, **({"size_mm": dev_sizes} if dev_sizes else {}), "source": src,
               "hypothesis": True, "parts": []}
        for part in parts:
            if isinstance(part, dict):
                keep = _clean(part, level, MANUAL_KEYS)
                sizes = part_sizes.get(str(part.get("part")))
                if sizes:
                    keep["size_mm"] = sizes
                keep.update(source=str(part.get("source", src)), hypothesis=True, not_given=not_given(keep))
                doc["parts"].append(keep)
        out.append(jsonable(doc))
    note = ("entries are hypotheses from the named source and may be incomplete; check by looking. " + NOT_GIVEN_NOTE
            + ". A device's id (not a part's name) is what a call about a device takes")
    if level == "manual":
        note += ". " + ROUGH_NOTE
    return {"level": level, "devices": out, "note": note}


def _names(entry, keys):
    return {str(entry[k]).strip().lower() for k in keys if isinstance(entry.get(k), (str, int))}


def sheet_of(doc, names):
    """The force the SERVED robot://devices document states for a named thing:
    {"max_n", "what"} or None. doc is the document as the session serves it (device_knowledge at the session's
    level), never a scene; names are what the call and the session call the thing (an id, a label, a part's
    name). A device is matched by its id, name or label and gives its max_force_n; a part by its part, name or
    label and gives the least of its own highest working force (force_n) and its device's max_force_n. A document
    that serves no force for the name (the manual level, an unknown name) answers None: nothing is stated."""
    asked = {str(n).strip().lower() for n in (names if isinstance(names, (list, tuple, set)) else [names])
             if n is not None and str(n).strip()}
    if not asked or not isinstance(doc, dict):
        return None
    best = None
    for dev in doc.get("devices") or []:
        if not isinstance(dev, dict):
            continue
        source = str(dev.get("source", "device sheet"))
        top = dev.get("max_force_n")
        top = float(top) if isinstance(top, (int, float)) and not isinstance(top, bool) else None
        found = []
        if asked & _names(dev, ("id", "name", "label")) and top is not None:
            found.append((top, f"the {source} of {dev.get('id')} states a largest force of {top:g} N"))
        for part in dev.get("parts") or []:
            if not isinstance(part, dict) or not asked & _names(part, ("part", "name", "label")):
                continue
            force = part.get("force_n")
            values = [float(v) for v in (force if isinstance(force, (list, tuple)) else [force])
                      if isinstance(v, (int, float)) and not isinstance(v, bool)]
            if values:
                found.append((max(values), f"the {source} of {dev.get('id')} gives the part {part.get('part')} a "
                                           f"highest working force of {max(values):g} N"))
            if top is not None:
                found.append((top, f"the {source} of {dev.get('id')} (the device of the part {part.get('part')}) "
                                   f"states a largest force of {top:g} N"))
        for value, what in found:
            if best is None or value < best["max_n"]:
                best = {"max_n": value, "what": what}
    return best
