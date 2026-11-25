# SPDX-License-Identifier: AGPL-3.0-only
"""Package probe, the memory: what remember() may keep (the lint) and when a cited call bears it out.

An entry is {scope, kind, frame, about, value, evidence}. What is kept must still be true after the robot's mount
moved, the device was put somewhere else and the camera was re-aimed, so the lint refuses anything tied to one
placement: frame base or joint, joint angles, base-frame points, pixels, look ids and frame files. A number may
stand only as a force (newtons), a distance along a named device or tool direction (mm with "along"), a count, or
a time (seconds). The lint walks the value recursively and reads free text too. It is key names and patterns, not
understanding: a pose spelled out in words, or numbers passed off as counts or seconds, get through.

A device_kind entry is a hypothesis about every device of that kind; it is marked measured only when a press this
session's journal holds, and the entry cites, agrees with it (agrees()): the press reached the stated force, or ran
along the stated tool direction. That is all "measured" means; it does not say the device changed state. The store
behind robot://experience is in probe_store.py. Nothing here reads simulator ground truth, and no robot or device
is named.
"""

import math
from rcpx.r2 import probe_bounds as derived
import re

SCOPES = ("robot", "device_kind", "procedure")
KINDS = ("force", "direction", "side", "surface", "sequence", "timing", "note")
FRAMES = ("device", "tool", "none")
MAX_TEXT, MAX_DEPTH, MAX_ITEMS = 400, 6, 40

JOINT_KEYS = {"q", "q_deg", "q_rad", "qpos", "joint", "joints", "joint_angles", "joint_angle", "angles", "pose",
              "named_pose", "achieved", "asked", "encoder", "encoders", "ticks"}
ANGLE_KEYS = {"angle", "yaw", "pitch", "roll", "tilt", "turn"}
POINT_KEYS = {"point", "point_mm", "points", "tip_mm", "center_mm", "centre_mm", "position", "position_mm", "xyz",
              "xyz_mm", "base", "base_mm", "contact_mm", "at_mm", "camera_pose", "t_wb", "path_mm", "axis_dir",
              "motion_dir", "normal_est", "target"}
PIXEL_KEYS = {"px", "pixel", "pixels", "u", "v", "uv", "tip_px", "norm_xy", "norm1000_yx", "axes_px", "bbox"}
LOOK_KEYS = {"look_id", "look", "looks", "frames", "annotated", "image", "picture", "watch_id", "snapshot"}
COUNT_KEYS = {"count", "times", "presses", "rounds", "steps", "repeats", "tries", "nudges", "taps", "pushes",
              "max_rounds"}
FORCE_KEYS = {"n", "newtons", "force"}
SECOND_KEYS = {"s", "seconds", "sec"}
COORD_KEYS = {"x", "y", "z", "dx", "dy", "dz", "rx", "ry", "rz"}      # with or without a unit ending (x_mm)
BAD_FRAMES = {"base": "frame", "world": "frame", "joint": "frame", "joints": "frame", "camera": "frame",
              "image": "frame", "pixel": "frame"}

UNIT = (r"(?:n|newtons?|mm|millimet(?:er|re)s?|s|sec|secs|seconds?|x|times?|press(?:es)?|rounds?|steps?|nudges?|"
        r"taps?|push(?:es)?|tries|try)")
NUM = r"[-+]?\d+(?:\.\d+)?"
RE_NUM = re.compile(NUM)
RE_LOOK = re.compile(r"\b(?:look[ _]?id|look\s*(?:#|no\.?|number)?\s*\d+|[LW]\d+)\b", re.IGNORECASE)
RE_FILE = re.compile(r"\.(?:png|jpe?g|ppm|bmp)\b", re.IGNORECASE)
RE_PIXEL = re.compile(r"\b(?:px|pixels?)\b|\b[uv]\s*[=:]\s*" + NUM, re.IGNORECASE)
RE_ANGLE = re.compile(NUM + r"\s*(?:deg\b|degrees?\b|°|rad\b|radians?\b)", re.IGNORECASE)
RE_BASE = re.compile(r"\bbase[ _-]?frame\b|\bframe[ _]?base\b|\bin base\b|\bxyz\b|\b[xyz]\s*[=:]\s*" + NUM,
                     re.IGNORECASE)
RE_TUPLE = re.compile(r"[\[(]\s*" + NUM + r"(?:\s*[,;]\s*" + NUM + r")+\s*[\])]")
MM_UNIT = r"(?:mm|millimet(?:er|re)s?)"
RE_MM = re.compile(NUM + r"\s*" + MM_UNIT + r"\b", re.IGNORECASE)
RE_MM_RUN = re.compile(r"(?:" + NUM + r"\s*" + MM_UNIT + r"\s*(?:,|;|\band\b|\bby\b)?\s*){3,}", re.IGNORECASE)
RE_FROM_BASE = re.compile(r"\b(?:robot|arm)(?:'s)?\s+(?:base|mount|origin)\b|"
                          r"\b(?:from|of)\s+the\s+(?:base|mount|origin)\b", re.IGNORECASE)
RE_ALONG_BAD = re.compile(r"\b(?:base|world|robot|mount|origin|global|camera|image|pixels?|joints?)\b", re.IGNORECASE)
RE_ID = re.compile(r"^(?:[A-Za-z][A-Za-z0-9_]{0,39}|\d{1,9})$")    # P12, PM7, contact_3, L4, or a call number
RE_OK_NUM = re.compile(r"(?:\b(?:step|stage|press|round|try|nudge|push|tap)\s+#?" + NUM + r"\b)|(?:" + NUM +
                       r"\s*(?:st|nd|rd|th)\b)|(?:" + NUM + r"\s*" + UNIT + r"\b)", re.IGNORECASE)

HINTS = {
    "frame": "state it relative to the device (frame device), to the tool (frame tool), or with no frame (none)",
    "joint_angle": "say what the angle achieved instead: which tool surface led, which way the part moved, how hard",
    "base_point": "a place is found again by looking and touching; keep how to find it (which side, which part), "
                  "not where it was",
    "pixel": "a pixel belongs to one look from one pose; keep what the cue looks like and where on the device it is",
    "look_id": "a look or a frame file belongs to one session; cite it in evidence, and keep in value what it showed",
    "number": "give numbers only as a force (key ending _n), a distance along a named device or tool direction "
              "(key ending _mm beside an \"along\"), a count (count, times, presses, rounds, steps) or a time (key "
              "ending _s); in text write the unit after the number",
    "shape": "see the remember tool's example",
}


def _bad(check, why, hint=None):
    return check, why, hint or HINTS.get(check, "")


def _is_num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def lint_text(text, where, joint_names=(), frame=None):
    """None, or (check, why, hint) for a free text that names a pose, a pixel, a look or a bare number."""
    if len(text) > MAX_TEXT:
        return _bad("shape", f"{where} is {len(text)} characters long (at most {MAX_TEXT})")
    low = text.lower()
    if RE_FILE.search(text) or RE_LOOK.search(text):
        return _bad("look_id", f"{where} names a look or a frame file ({text[:50]!r})")
    if RE_PIXEL.search(text):
        return _bad("pixel", f"{where} names pixels ({text[:50]!r})")
    for name in joint_names:
        if name and re.search(r"(?<![a-z0-9])" + re.escape(str(name).lower()) + r"(?![a-z0-9])", low):
            return _bad("joint_angle", f"{where} names the joint {name!r}")
    if re.search(r"\bjoint\s*\d+\b|\bjoint angles?\b|\bq_deg\b|\bencoder", low) or RE_ANGLE.search(text):
        return _bad("joint_angle", f"{where} gives an angle or a joint reading ({text[:50]!r})")
    if RE_BASE.search(text):
        return _bad("base_point", f"{where} gives a base frame place ({text[:50]!r})")
    m = RE_TUPLE.search(text)
    if m:
        n = len(RE_NUM.findall(m.group(0)))
        return _bad("pixel" if n == 2 else "base_point", f"{where} holds the coordinates {m.group(0)}")
    if RE_MM.search(text) and RE_FROM_BASE.search(text):
        return _bad("base_point", f"{where} gives a distance from the robot's base ({text[:50]!r})")
    if RE_MM_RUN.search(text):
        return _bad("base_point", f"{where} lists three or more distances in a row: a point ({text[:50]!r})")
    if frame == "none" and re.search(NUM + r"\s*(?:mm|millimet(?:er|re)s?)\b", text, re.IGNORECASE):
        return _bad("number", f"{where} gives a distance, and this entry has frame none ({text[:50]!r})",
                    "set frame to device or tool and say along which of its directions the distance runs")
    rest = RE_OK_NUM.sub(" ", text)
    m = RE_NUM.search(rest)
    if m:
        return _bad("number", f"{where} holds the bare number {m.group(0)} ({text[:50]!r})")
    return None


def _lint_number(key, value, where, frame, siblings):
    k = key.lower()
    if not math.isfinite(float(value)):
        return _bad("number", f"{where} is not a finite number")
    if k in COUNT_KEYS or k.startswith("n_"):
        if float(value) != int(value) or value < 0:
            return _bad("number", f"{where}: a count is a whole number of at least 0")
        return None
    if k in FORCE_KEYS or k.endswith("_n"):
        if value < 0:
            return _bad("number", f"{where}: a force of {value:g} N is negative")
        return None
    if k in SECOND_KEYS or k.endswith("_s"):
        if value < 0:
            return _bad("number", f"{where}: a time of {value:g} s is negative")
        return None
    if k == "mm" or k.endswith("_mm"):
        along = siblings.get("along")
        if frame == "none":
            return _bad("number", f"{where}: a distance needs a frame; this entry has frame none",
                        "set frame to device or tool and name the direction beside it in \"along\"")
        example = "\"into its front face\"" if frame == "device" else "\"the finger (tool x)\""
        if not isinstance(along, str) or not re.search(r"[a-z]{3}", along.lower()):
            return _bad("number", f"{where}: a distance must run along a {frame} direction named in words",
                        f"put \"along\": \"<a direction of the {frame}>\" beside it, for example " + example)
        if RE_ALONG_BAD.search(along):
            return _bad("base_point", f"{where}: a distance along a direction of the base, the world, the camera "
                        "or a joint is a place in that frame", f"name a direction of the {frame} itself, for "
                        "example " + example)
        return None
    return _bad("number", f"{where} is a number with no unit in its name")


def _lint_key(key, value, where, joint_names):
    k = key.lower()
    if k == "frame":
        if isinstance(value, str) and value.lower() in FRAMES:
            return None
        if isinstance(value, str) and RE_FILE.search(value):
            return _bad("look_id", f"{where} is a frame file")
        return _bad("frame", f"{where} is {str(value)[:30]!r}: only device, tool or none may be kept")
    if k in LOOK_KEYS:
        return _bad("look_id", f"{where} is a look or a frame of one session")
    if k in PIXEL_KEYS or k.endswith("_px"):
        return _bad("pixel", f"{where} is a pixel position")
    if k in JOINT_KEYS or any(j and re.search(r"(?<![a-z0-9])" + re.escape(str(j).lower()) + r"(?![a-z0-9])", k)
                              for j in joint_names):
        return _bad("joint_angle", f"{where} is a joint reading")
    if re.sub(r"_(?:mm|cm|m|n|s|deg|rad|px)$", "", k) in COORD_KEYS:
        return _bad("pixel" if k.endswith("_px") else "base_point",
                    f"{where} is one coordinate of a place or a move in a frame")
    if k in ANGLE_KEYS or k.endswith("_deg") or k.endswith("_rad"):
        return _bad("joint_angle", f"{where} is an angle: a joint reading or a pose under another name")
    if k in POINT_KEYS or k.endswith("_point") or k.endswith("_xyz"):
        return _bad("base_point", f"{where} is a place or a direction in the base frame")
    return None


def lint_value(value, frame, joint_names=(), where="value", depth=0, key=None, siblings=None):
    """None when the value may be kept, else (check, why, hint) for the first thing that may not."""
    if depth > MAX_DEPTH:
        return _bad("shape", f"{where} is nested deeper than {MAX_DEPTH} levels")
    if value is None or isinstance(value, bool):
        return None
    if isinstance(value, str):
        return lint_text(value, where, joint_names, frame)
    if _is_num(value):
        if key is None:
            return _bad("number", f"{where} is a bare number")
        return _lint_number(key, value, where, frame, siblings or {})
    if isinstance(value, (list, tuple)):
        if len(value) > MAX_ITEMS:
            return _bad("shape", f"{where} has {len(value)} items (at most {MAX_ITEMS})")
        if len(value) >= 2 and all(_is_num(v) for v in value):
            n = len(value)
            check = "pixel" if n == 2 else ("joint_angle" if joint_names and n == len(joint_names) and n != 3
                                            else "base_point")
            what = {"pixel": "a pixel position", "joint_angle": "one value per joint: a joint pose",
                    "base_point": "a point or a direction in a frame"}[check]
            return _bad(check, f"{where} is a list of {n} numbers: {what}")
        for i, v in enumerate(value):
            if _is_num(v):
                return _bad("number", f"{where}[{i}] is a bare number")
            bad = lint_value(v, frame, joint_names, f"{where}[{i}]", depth + 1)
            if bad:
                return bad
        return None
    if isinstance(value, dict):
        if len(value) > MAX_ITEMS:
            return _bad("shape", f"{where} has {len(value)} keys (at most {MAX_ITEMS})")
        for k, v in value.items():
            here = f"{where}.{k}"
            bad = _lint_key(str(k), v, here, joint_names) or lint_text(str(k), f"the key {here}", ())
            if bad:
                return bad
            bad = lint_value(v, frame, joint_names, here, depth + 1, key=str(k), siblings=value)
            if bad:
                return bad
        return None
    return _bad("shape", f"{where} is a {type(value).__name__}: only text, numbers, true or false, lists and objects")


def lint(entry, joint_names=()):
    """None when the entry may be remembered, else (check, why, hint). Checks the type first, then the value."""
    if not isinstance(entry, dict):
        return _bad("shape", "an entry is an object {scope, kind, frame, about, value, evidence}")
    extra = sorted(set(entry) - {"scope", "kind", "frame", "about", "value", "evidence"})
    if extra:
        return _bad("shape", "unknown entry fields: " + ", ".join(str(x) for x in extra),
                    "an entry has scope, kind, frame, about, value and evidence; everything learned goes in value")
    scope, kind, frame = entry.get("scope"), entry.get("kind"), entry.get("frame", "none")
    if scope not in SCOPES:
        return _bad("scope", f"scope is {str(scope)[:30]!r}", "scope is one of " + ", ".join(SCOPES))
    if kind not in KINDS:
        return _bad("kind", f"kind is {str(kind)[:30]!r}", "kind is one of " + ", ".join(KINDS))
    if frame not in FRAMES:
        low = str(frame).lower()
        if low in BAD_FRAMES:
            return _bad("frame", f"frame {low} is tied to where the robot and the device stand now: what is kept "
                                 "must hold after either is moved")
        return _bad("frame", f"frame is {str(frame)[:30]!r}", "frame is one of " + ", ".join(FRAMES))
    about = entry.get("about")
    if scope != "robot" and not (isinstance(about, str) and about.strip()):
        return _bad("about", f"a {scope} entry must say what it is about",
                    "about: the device kind's name as robot://devices gives it (or your own words for the kind), "
                    "or the procedure's name")
    if about is not None:
        if not isinstance(about, str):
            return _bad("about", "about is a short text")
        bad = lint_text(about, "about", joint_names, "none")     # a name: no distance belongs in it
        if bad:
            return bad
    if "value" not in entry or entry["value"] in (None, "", [], {}):
        return _bad("value", "the entry has no value", "value holds what was learned, in words and typed numbers")
    if kind in ("direction", "side", "surface") and frame == "none":
        return _bad("frame", f"a {kind} needs a frame: device or tool",
                    "set frame to device (relative to the part) or tool (relative to the finger)")
    ev = entry.get("evidence", [])
    if (not isinstance(ev, list) or any(isinstance(x, bool) or not isinstance(x, (str, int)) for x in ev)
            or any(not RE_ID.match(str(x).strip()) for x in ev)):
        return _bad("evidence", "evidence is a list of call ids: each a short id a tool returned, or a call number",
                    "cite the ref a press or a probe returned (P<n>, PM<n>) or the id a touch returned")
    if len(ev) > MAX_ITEMS:
        return _bad("evidence", f"{len(ev)} calls cited (at most {MAX_ITEMS})", "cite the calls that decided it")
    return lint_value(entry["value"], frame, joint_names)


# ---------------------------------------------------------------------------------------------------------------
# evidence: the cited calls in this session's journal
def find_calls(records, ids):
    """(found {id: record}, missing [ids]). A cited id matches a journaled call whose result carries it as ref
    (a press, a probe), id or world_id (a touch, a probe), look_id or watch_id, or whose call number it is."""
    found, missing = {}, []
    for cid in ids:
        key = str(cid).strip()
        hit = None
        for rec in records:
            if rec.get("kind") not in ("primitive", "call"):
                continue
            res = rec.get("result") if isinstance(rec.get("result"), dict) else {}
            vals = {str(res.get(k)) for k in ("ref", "id", "world_id", "look_id", "watch_id") if res.get(k) is not None}
            if key in vals or (rec.get("kind") == "call" and key.isdigit() and rec.get("call") == int(key)) or (
                    key.isdigit() and res.get("ref") == f"P{key}"):
                hit = rec
                if rec.get("name") == "press" and "refused" not in res:
                    break
        if hit is None:
            missing.append(cid)
        else:
            found[key] = hit
    return found, missing


def _forces(value):
    """(single values, ranges) of every force stated in a value."""
    singles, lo, hi = [], None, None
    def walk(v, key=None):
        nonlocal lo, hi
        if isinstance(v, dict):
            for k, x in v.items():
                walk(x, str(k).lower())
        elif isinstance(v, list):
            for x in v:
                walk(x, key)
        elif _is_num(v) and key and (key in FORCE_KEYS or key.endswith("_n")):
            if key.startswith(("min_", "at_least_", "from_")):
                lo = float(v)
            elif key.startswith(("max_", "at_most_", "to_", "up_to_")):
                hi = float(v)
            else:
                singles.append(float(v))
        elif isinstance(v, str):
            for m in re.finditer(r"(" + NUM + r")\s*(?:n|newtons?)\b", v, re.IGNORECASE):
                singles.append(float(m.group(1)))
    walk(value)
    return singles, (lo, hi)


TOOL_DIRS = {"forward": (1, 0, 0), "back": (-1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0), "up": (0, 0, 1),
             "down": (0, 0, -1)}


def _tool_dir(value):
    words = []
    def walk(v):
        if isinstance(v, str):
            words.append(v.lower())
        elif isinstance(v, dict):
            for x in v.values():
                walk(x)
        elif isinstance(v, list):
            for x in v:
                walk(x)
    walk(value)
    hits = {name for w in words for name in TOOL_DIRS if re.search(r"\b" + name + r"\b", w)}
    return TOOL_DIRS[hits.pop()] if len(hits) == 1 else None


def agrees(entry, rec, B):
    """(True | False, why): does one journaled call bear the entry out? Only a press that ran can; it checks a
    force (its peak force against the stated force or range) or a tool frame direction (its own direction)."""
    res = rec.get("result") if isinstance(rec.get("result"), dict) else {}
    args = rec.get("args") if isinstance(rec.get("args"), dict) else {}
    if rec.get("name") != "press":
        return False, f"{rec.get('name')} is not a press"
    if "refused" in res:
        return False, "the cited press was refused: nothing was pushed"
    peak = res.get("peak_force_n")
    hidden = res.get("force_hidden_n", B.f_hidden())
    sigma = res.get("sigma_force_n", B.sigma_f())
    tolerance = derived.memory_tolerance_n(B, hidden, sigma)
    if not _is_num(peak) or peak <= hidden:
        return False, "the cited press felt nothing"
    if entry["kind"] == "force":
        singles, (lo, hi) = _forces(entry["value"])
        if lo is not None or hi is not None:
            ok = (lo is None or peak >= lo - tolerance) and (hi is None or peak <= hi + tolerance)
            return ok, (f"the press peaked at {peak:g} N, " + ("inside" if ok else "outside") + " the stated range")
        if not singles:
            return False, "the value states no force in newtons to compare with the press"
        near = min(singles, key=lambda f: abs(f - peak))
        ok = abs(near - peak) <= tolerance
        return ok, f"the press peaked at {peak:g} N against the stated {near:g} N"
    if entry["kind"] == "direction" and entry.get("frame") == "tool":
        want = _tool_dir(entry["value"])
        if want is None:
            return False, "the value names no single tool direction (forward, back, left, right, up, down)"
        if args.get("frame") != "tool":
            return False, "the cited press gave its direction in the base frame, not the tool frame"
        d = args.get("direction")
        d = TOOL_DIRS.get(d, d) if isinstance(d, str) else d
        try:
            x, y, z = (float(v) for v in d)
            n = math.sqrt(x * x + y * y + z * z)
            if n == 0:
                raise ValueError("zero direction")
        except (TypeError, ValueError):
            return False, "the cited press has no readable direction"
        vector = (x, y, z)
        nearest = max(TOOL_DIRS.values(), key=lambda axis: sum(a * b for a, b in zip(axis, vector)))
        return tuple(want) == tuple(nearest), "the press's nearest named tool axis agrees" if tuple(want) == tuple(nearest) else "the press's nearest named tool axis differs"
    return False, (f"a press cannot check a {entry['kind']} entry"
                   + (" in the device frame" if entry["kind"] == "direction" else "") + ": it stays a hypothesis")
