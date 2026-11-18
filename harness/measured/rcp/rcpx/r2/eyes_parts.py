# SPDX-License-Identifier: AGPL-3.0-only
"""Pure helpers of the eyes package: the one merged, numbered list, what the manual says about a
colour, which sides of a thing a picture cannot show, and the part a pick may take from a row.

Nothing here reads a picture, a file, a clock or the executor, and nothing names a robot, a device or a part: part
names, roles and colour words arrive as arguments at run time (rcpx/r2/eyes.py builds them from robot://devices).

parts (the shape of ex.state["eyes"]["parts"], built by parts_from): {"control": [names], "cue": [names],
"forbidden": [names], "fixed": [names], "colours": {name: {state: colour}}, "forbidding": {name: the manual's own
sentence}, "roles": {name: role}}. Any key may be missing.

A row is a dict. Keys read here: box_px [x1, y1, x2, y2], confidence, source, colour, n, found_by, role,
also_found_by, manual, manual_part, keep_off, area_px, cut_by_frame. Input rows and lists are never changed.

The robot does not choose the target part: merge asks nothing and ranks nothing by role except the ORDER of the
rows (control parts' finds before cue parts' before forbidden parts'), every row says which word found it and that
part's role, and default_part gives a pick a part only from the model or from a row found by a control part's word.
"""

import json
import math
from numbers import Real

SIDES = ("robot side", "far side", "left side", "right side", "top")
SEEN_DEG = 100.0        # a face is seen when its outward direction makes MORE than this angle with the line of sight
BEST_PER_PART = 3       # rows kept per part word
MAX_ROWS = 16           # two pages
PAGE_ROWS = 8
OVERLAP = 0.5           # two boxes with intersection over union at or over this are one thing
MANUAL_MAX = 60         # characters of the manual phrase
KEEP_OFF_MAX = 80       # characters of the manual's forbidding sentence in a brief row
ROLES = ("control", "cue", "forbidden", "fixed")
SMALL_PX, LARGE_PX = 600.0, 6000.0     # area bounds of small and large in a picture 640 px wide
NEAR_BASE_MM = 30.0     # nearer the base axis than this a thing has no direction of its own
WHERE = ("ahead", "ahead left", "left", "behind left", "behind", "behind right", "right", "ahead right")


def _vector(values, length):
    """A tuple of finite floats of the given length, or None."""
    try:
        if len(values) != length:
            return None
        if not all(isinstance(v, Real) and not isinstance(v, bool) and math.isfinite(v) for v in values):
            return None
        return tuple(float(v) for v in values)
    except (TypeError, ValueError, OverflowError):
        return None


def box_overlap(a, b) -> float:
    """Intersection over union of two boxes [x1, y1, x2, y2]; 0.0 for an unusable or empty pair."""
    a, b = _vector(a, 4), _vector(b, 4)
    if a is None or b is None:
        return 0.0
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    inter = max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


# ---------------------------------------------------------------- the manual's parts
def parts_from(plist) -> dict:
    """The parts dict from the manual's part list (rcpx.sheet_words.parts(doc): dicts with name, role, colours,
    forbidding). A name is used once (the first entry that carries it)."""
    out = {"control": [], "cue": [], "forbidden": [], "fixed": [], "colours": {}, "forbidding": {}, "roles": {}}
    for p in plist or ():
        name = str(p.get("name") or "").strip().lower()
        if not name or name in out["roles"]:
            continue
        role = p.get("role") if p.get("role") in ROLES else "fixed"
        out[role].append(name)
        out["roles"][name] = role
        if p.get("colours"):
            out["colours"][name] = dict(p["colours"])
        if role == "forbidden":
            out["forbidding"][name] = str(p.get("forbidding") or f"the manual forbids the {name}")
    return out


def names(parts) -> list:
    """Every part name of the manual, control parts first, then cue, forbidden and fixed parts; each once."""
    out = []
    for role in ROLES:
        for name in (parts or {}).get(role) or ():
            if name not in out:
                out.append(name)
    return out


def role_of(name, parts):
    """control | cue | forbidden | fixed for a part name of the manual, else None."""
    if not name:
        return None
    role = ((parts or {}).get("roles") or {}).get(name)
    if role is not None:
        return role
    for r in ROLES:
        if name in ((parts or {}).get(r) or ()):
            return r
    return None


def _forbidding(name, parts):
    return ((parts or {}).get("forbidding") or {}).get(name) or f"the manual forbids the {name}"


def keep_off(row, parts):
    """The manual's own forbidding sentence when the row was found by a forbidden part's word (found_by or
    also_found_by), or has a colour the manual gives a forbidden part (the manual's word exactly); else None."""
    banned = list((parts or {}).get("forbidden") or ())
    if not banned or not isinstance(row, dict):
        return None
    words = [row.get("found_by")] + [a.get("part") for a in row.get("also_found_by") or () if isinstance(a, dict)]
    for w in words:
        if w in banned:
            return _forbidding(w, parts)
    colour = row.get("colour")
    if colour:
        for name in banned:
            if colour in (((parts or {}).get("colours") or {}).get(name) or {}).values():
                return _forbidding(name, parts)
    return None


# ---------------------------------------------------------------- the merged list
def _score(row):
    c = row.get("confidence")
    return (c is not None, c if c is not None else 0.0)


def _combine_words(kept, dropped):
    """The same thing found by two part words: the kept row keeps its place and box and says both words; the word
    with the higher confidence becomes found_by (a thing a cue part's word finds with a higher score than a
    control part's is more likely the cue part), the other goes to also_found_by."""
    old, new = kept.get("found_by"), dropped.get("found_by")
    if old is None or new is None or old == new:
        return
    if _score(dropped) > _score(kept):
        extra = {"part": old, "confidence": kept.get("confidence")}
        kept["found_by"], kept["confidence"] = new, dropped.get("confidence")
    else:
        extra = {"part": new, "confidence": dropped.get("confidence")}
    kept["also_found_by"] = list(kept.get("also_found_by", [])) + [extra]


def merge(per_part, proposed, parts, first=()) -> list:
    """The numbered list. per_part: {word: [rows the vision model returned for it]}; proposed: the
    proposer's rows, best first. Word order: the words in `first` (the model's own words), then the manual's
    control parts, cue parts, forbidden parts, fixed parts, then any other key. Each word's rows by confidence, the
    best BEST_PER_PART kept, interleaved by rank (the first of every word, then the second of every word, ...);
    then the proposed rows; a row whose box overlaps an earlier row's at OVERLAP or more is dropped (the earlier
    row stays; two part words on one thing are both kept, see _combine_words); cut to MAX_ROWS; numbered from 1.
    Every row carries found_by, role (the manual's role of found_by, else None), source and keep_off."""
    per_part, parts = per_part or {}, parts or {}
    order = [w for w in first if w in per_part]
    order += [w for w in names(parts) if w in per_part and w not in order]
    order += [w for w in per_part if w not in order]
    ranked = [(w, sorted(per_part[w], key=_score, reverse=True)[:BEST_PER_PART]) for w in order]

    def sequence():
        for rank in range(BEST_PER_PART):
            for word, rows in ranked:
                if rank < len(rows):
                    yield rows[rank], word, "find"
        for row in proposed or ():
            yield row, None, "propose"

    kept = []
    for original, word, source in sequence():
        row = dict(original)
        row["found_by"] = word
        row["source"] = row.get("source") or source
        twin = next((k for k in kept if box_overlap(row.get("box_px"), k.get("box_px")) >= OVERLAP), None)
        if twin is not None:
            _combine_words(twin, row)
            continue
        row["n"] = len(kept) + 1
        kept.append(row)
        if len(kept) >= MAX_ROWS:
            break
    for row in kept:
        row["role"] = role_of(row.get("found_by"), parts)
        row["keep_off"] = keep_off(row, parts)
    return kept


# ---------------------------------------------------------------- what the manual says about a colour
def manual_entry(colour, parts, near=None):
    """None, or {"part", "state", "colour" (the manual's own word), "text"}: the first part whose colours hold the
    colour word. Parts are visited cue, forbidden, control, fixed, then any other key of parts["colours"]; an exact
    word wins over a near one (near(a, b) -> bool, when given). text: "the manual says dark grey = lamp when off",
    cut to MANUAL_MAX characters."""
    if not colour:
        return None
    parts = parts or {}
    colours = parts.get("colours") or {}
    order = []
    for role in ("cue", "forbidden", "control", "fixed"):
        order += [n for n in parts.get(role) or () if n not in order]
    order += [n for n in colours if n not in order]
    for match in ((lambda a, b: a == b), near):
        if match is None:
            continue
        for name in order:
            for state, word in (colours.get(name) or {}).items():
                if match(colour, word):
                    text = f"the manual says {word} = {name}" + (f" when {state}" if state is not None else "")
                    return {"part": name, "state": state, "colour": word, "text": text[:MANUAL_MAX]}
    return None


def manual(colour, parts, near=None):
    """The manual phrase for a colour word, or None."""
    entry = manual_entry(colour, parts, near)
    return entry["text"] if entry is not None else None


def annotate(rows, parts, near=None) -> list:
    """Copies of the rows with manual, manual_part and keep_off filled from each row's colour word."""
    out = []
    for row in rows or ():
        r = dict(row)
        entry = manual_entry(r.get("colour"), parts, near)
        r["manual"] = entry["text"] if entry else None
        r["manual_part"] = entry["part"] if entry else None
        r["keep_off"] = keep_off(r, parts)
        out.append(r)
    return out


def part_warning(row, part, parts=None, near=None):
    """The warning for a pick that names `part` on a row whose colour is the manual's colour of ANOTHER part, else
    None. No warning when the row carries no manual_part, when it is the named part, or when (parts given) the
    row's colour is also one of the named part's own colours."""
    if not part or not isinstance(row, dict):
        return None
    part = str(part).strip()
    other = row.get("manual_part")
    if not part or other is None or str(other).strip().lower() == part.lower():
        return None
    colour = row.get("colour")
    if parts is not None and colour:
        words = list(((parts.get("colours") or {}).get(part.lower()) or {}).values())
        if any(colour == w or (near is not None and near(colour, w)) for w in words):
            return None
    text = row.get("manual") or f"the manual gives that colour to the {other}"
    return f"number {row.get('n', '?')} is {colour or 'that colour'}; {text}: it may be the {other}, not the {part}"


def default_part(row, parts, given=None):
    """The part of a pick: the model's own argument when it gave one; else the row's found_by when the manual
    gives that part the role control; else None (the entry is then stored as "candidate N"). Never the manual's
    first part and never a label of the operator."""
    if isinstance(given, str) and given.strip():
        return given.strip().lower()
    found = (row or {}).get("found_by")
    if found and role_of(found, parts) == "control":
        return found
    return None


# ---------------------------------------------------------------- what a picture cannot show
def unseen(camera_pose, table_hit_mm, base_xy=(0, 0)) -> list:
    """The sides of an upright box standing at table_hit_mm that this line of sight cannot show, in SIDES order.
    camera_pose {"looks_along": [x, y, z]} (base frame: x forward, y left, z up; any length). r is the horizontal
    unit vector from the base axis to the hit (+x within 1 mm of it): robot side = -r, far side = +r, left side =
    z x r, right side = -(z x r), top = +z. A side is seen when its direction makes over SEEN_DEG with looks_along."""
    if table_hit_mm is None or not isinstance(camera_pose, dict):
        return []
    direction = _vector(camera_pose.get("looks_along"), 3)
    if direction is None:
        return []
    length = math.hypot(*direction)
    if length == 0.0:
        return []
    try:
        hit = _vector(list(table_hit_mm)[:2], 2)
    except TypeError:
        return []
    base = _vector(base_xy, 2)
    if hit is None or base is None:
        return []
    rx, ry = hit[0] - base[0], hit[1] - base[1]
    radius = math.hypot(rx, ry)
    rx, ry = (1.0, 0.0) if radius < 1.0 else (rx / radius, ry / radius)
    outward = ((-rx, -ry, 0.0), (rx, ry, 0.0), (-ry, rx, 0.0), (ry, -rx, 0.0), (0.0, 0.0, 1.0))
    viewing = tuple(v / length for v in direction)
    hidden = []
    for name, face in zip(SIDES, outward):
        cosine = sum(a * b for a, b in zip(face, viewing))
        if math.degrees(math.acos(max(-1.0, min(1.0, cosine)))) <= SEEN_DEG:
            hidden.append(name)
    return hidden


def unseen_clause(sides, n=1, short=False) -> str:
    """The clause of said: "this view cannot show the robot side or left side of number 1: a part there is in no
    picture yet" (short: without what follows the colon); "" when no side is unseen."""
    sides = [s for s in sides or () if s]
    if not sides:
        return ""
    text = sides[0] if len(sides) == 1 else ", ".join(sides[:-1]) + " or " + sides[-1]
    tail = "" if short else ": a part there is in no picture yet"
    return f"this view cannot show the {text} of number {n}" + tail


# ---------------------------------------------------------------- words of a row
def size_word(area_px, frame_w=640):
    """small | medium | large from a region's area, the bounds stated for a picture 640 px wide."""
    if not isinstance(area_px, Real) or isinstance(area_px, bool) or area_px <= 0:
        return None
    k = (float(frame_w) / 640.0) ** 2
    return "small" if area_px < SMALL_PX * k else "large" if area_px >= LARGE_PX * k else "medium"


def where_word(hit_mm, base_xy=(0, 0)):
    """Where a thing stands as seen from the robot's base (x forward, y left), from where its line of sight meets
    the table: ahead, ahead left, left, behind left, behind, behind right, right, ahead right; None with no hit."""
    try:
        hit = _vector(list(hit_mm)[:2], 2)
    except TypeError:
        return None
    base = _vector(base_xy, 2)
    if hit is None or base is None:
        return None
    x, y = hit[0] - base[0], hit[1] - base[1]
    if math.hypot(x, y) < NEAR_BASE_MM:
        return "at the base"
    return WHERE[int(((math.degrees(math.atan2(y, x)) + 22.5) % 360.0) // 45.0)]


def cut_by_frame(box, w, h, margin=1.0) -> bool:
    """True when the bounding box touches the frame's edge: the thing may go on outside the picture."""
    b = _vector(box, 4)
    return b is not None and (b[0] <= margin or b[1] <= margin or b[2] >= w - margin or b[3] >= h - margin)


# ---------------------------------------------------------------- the brief page
_BRIEF_KEYS = ("found_by", "role", "colour", "size", "where", "confidence", "manual", "keep_off")


def _brief_row(row, level):
    """One row of a brief page. level counts the cuts made so far, in this order: where, size, a manual phrase
    about the row's own part, confidence, role, the manual phrase as may_be (the other part's name alone)."""
    drop = set(("where", "size")[:min(level, 2)])
    if level >= 4:
        drop.add("confidence")
    if level >= 5:
        drop.add("role")
    out = {"n": row.get("n")}
    for k in _BRIEF_KEYS:
        v = row.get(k)
        if v is None or k in drop:
            continue
        if k == "manual":
            own = row.get("manual_part") is not None and row.get("manual_part") == row.get("found_by")
            if (level >= 3 and own) or (level >= 6 and row.get("manual_part") is None):
                continue
            if level >= 6:
                out["may_be"] = row["manual_part"]
                continue
        if k == "keep_off":
            v = str(v)[:KEEP_OFF_MAX]
        out[k] = v
    if row.get("cut_by_frame") and level == 0:
        out["cut"] = True
    return out


def brief_rows(rows, room) -> tuple:
    """(the rows of one page in their brief form, the cut level used): null keys are left out, and keys are cut
    in the order of _brief_row until the page prints in `room` bytes of JSON. n, found_by, colour and keep_off are
    never cut; when even that does not fit, rows are left off the end (the caller says how many)."""
    rows = list(rows or ())
    out = []
    for level in range(7):
        out = [_brief_row(r, level) for r in rows]
        if len(json.dumps(out)) <= room:
            return out, level
    while out and len(json.dumps(out)) > room:
        out.pop()
    return out, 6


def words_of(text, limit) -> str:
    """text cut to at most `limit` words (a said sentence stays readable whole)."""
    words = str(text).split()
    return " ".join(words) if len(words) <= limit else " ".join(words[:limit])


# ---------------------------------------------------------------- the said sentences of a list and of a find
def list_said(lst, page, page_rows, limit=45, short=False) -> str:
    """The said sentence of one page of a list record {look, items, source, words, unseen}: the source and the
    count, then what this picture cannot show; a candidate cut by the picture's edge is said when it fits. short:
    the unseen clause without its second half and no word about the edge (a page that would not fit otherwise)."""
    key, n = lst["look"], len(page_rows)
    if not lst["items"]:
        return f"nothing to number in {key}: the picture shows no separate thing apart from the robot itself"
    span = f"{n} things numbered in {key}" if page == 1 else \
        f"things {page_rows[0]['n']} to {page_rows[-1]['n']} of {key}"
    found = sum(1 for r in page_rows if r.get("found_by"))
    if lst["source"] == "local":
        how = " from colour and geometry (no vision model)"
    elif found:
        whose = "your words and the manual's part names" if lst.get("words") else "the manual's part names"
        how = f", the first {found} found by {whose}" if page == 1 else f", {found} found by {whose}"
    else:
        how = " by the vision model's regions"
    tail = unseen_clause(lst.get("unseen"), lst["items"][0]["n"], short)
    text = span + how + ("; " + tail if tail else "")
    cut = [] if short else [str(r["n"]) for r in page_rows if r.get("cut_by_frame")]
    if cut:
        more = f"; number {cut[0]} is cut by the picture's edge"
        if len((text + more).split()) <= limit:
            text += more
    return words_of(text, limit)


def find_answer(lst, parts, near=None, not_touched="not checked by touch") -> tuple:
    """(answer, said) of a find from its list record {look, words, items, threshold, floor}: "none" when no
    candidate reaches the equipment's floor (or there is none at all); "found" when the first candidate's
    confidence is at or over the equipment's threshold; else "unsure". A found whose colour is the manual's colour
    of another part than the one the words name is downgraded to unsure, and said says why. said always carries
    the words not checked by touch; the numbers in it are the model's score and the equipment's own."""
    key, word, rows = lst["look"], lst.get("words"), lst.get("items") or []
    thr, floor = lst.get("threshold"), lst.get("floor")
    best = rows[0] if rows else None
    conf = (best or {}).get("confidence")
    if best is None or (floor is not None and (conf is None or conf < floor)):
        said = f"the vision model found nothing like {word} in {key}"
        if best is not None:
            said += " (its best match " + ("has no score" if conf is None else f"scores {conf:g}") \
                + f", under its floor {floor:g})"
        return "none", f"{said}; {not_touched}"
    answer = "found" if thr is not None and conf is not None and conf >= thr else "unsure"
    score = "no score" if conf is None else f"{conf:g}" + (f", threshold {thr:g}" if thr is not None else "")
    said = f"the vision model's best match for {word} is number 1 ({score})"
    if answer == "found" and best.get("role") is not None \
            and part_warning(best, best.get("found_by"), parts, near) is not None:
        answer = "unsure"
        said += f", but it is {best.get('colour')} and {best.get('manual')}: unsure"
    elif answer == "unsure":
        said = f"the vision model is unsure about {word}: its best match is number 1 ({score})"
    return answer, f"{said}; {not_touched}"


__all__ = ["BEST_PER_PART", "MAX_ROWS", "OVERLAP", "PAGE_ROWS", "ROLES", "SEEN_DEG", "SIDES", "annotate",
           "box_overlap", "brief_rows", "cut_by_frame", "default_part", "find_answer", "keep_off", "list_said",
           "manual", "manual_entry", "merge", "names", "part_warning", "parts_from", "role_of", "size_word", "unseen",
           "unseen_clause", "where_word", "words_of"]
