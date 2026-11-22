# SPDX-License-Identifier: AGPL-3.0-only
"""The stop conditions of a routine the robot runs itself. Pure: standard library only.

The operating model states a stop condition in two words; this file reads them (parse), says whether a condition may
be used now (valid), and decides from plain data whether one step met it and with WHICH KIND OF EVIDENCE (met):
cue (the part the manual says shows the state), seen (a same-view pair of pictures), felt (the pushed thing's own
motion), found (a seeing step) or none. A missing or unclear piece gives None (unknown), never True. There is no
default stop condition: from_manual gives "cue changes" only when the manual declares a cue part, for a call the
robot writes. A package may add conditions for its own tools as a table {words: fn(step, record, cue)}; an entry
keyed by a form's first word and a space ("cue ", "found ") decides every condition of that form.
"""

import math
from numbers import Real


FORMS = (
    "cue changes", "cue WORD", "seen changes", "part moves", "part stays",
    "found WORDS", "new none",
)
EXAMPLES = {
    "cue changes": "cue changes",
    "cue WORD": "cue on",
    "seen changes": "seen changes",
    "part moves": "part moves",
    "part stays": "part stays",
    "found WORDS": "found the handle",
    "new none": "new none",
}
EVIDENCE = ("cue", "seen", "felt", "found", "none")
LIGHT_KINDS = ("lit", "went_dark", "colour_changed")


def _normalise(value):
    """Read underscores as spaces, fold whitespace, and lower-case text."""
    return " ".join(value.replace("_", " ").lower().split())


def parse(until):
    """Return exactly one canonical key for an accepted condition, else None."""
    if isinstance(until, str):
        pieces = _normalise(until).split(" ", 1)
        if len(pieces) != 2:
            return None
        key, value = pieces
    elif isinstance(until, dict) and len(until) == 1:
        key, value = next(iter(until.items()))
        if not isinstance(value, str):
            return None
        value = _normalise(value)
    else:
        return None
    if key == "cue" and value and len(value.split()) == 1:
        return {key: value}
    if key == "found" and value:
        return {key: value}
    if (key, value) in (
        ("seen", "changes"), ("part", "moves"), ("part", "stays"),
        ("new", "none"),
    ):
        return {key: value}
    return None


def words(until):
    """Return normalized condition text, or None when parsing fails."""
    condition = parse(until)
    if condition is None:
        return None
    key, value = next(iter(condition.items()))
    return key + " " + value


def form(until):
    """Return the declared form for a parseable condition, else None."""
    condition = parse(until)
    if condition is None:
        return None
    key, value = next(iter(condition.items()))
    if key == "cue" and value != "changes":
        return "cue WORD"
    if key == "found":
        return "found WORDS"
    return key + " " + value


def from_manual(parts):
    """Suggest cue changes only when the manual explicitly declares a cue."""
    if isinstance(parts, (list, tuple)):
        if any(isinstance(part, dict) and part.get("role") == "cue"
               for part in parts):
            return {"cue": "changes"}
    elif isinstance(parts, dict):
        names = parts.get("cue")
        if isinstance(names, (list, tuple)) and names:
            return {"cue": "changes"}
    return None


def cue_part(parts):
    """Return the first declared cue name when it is non-empty text."""
    name = None
    if isinstance(parts, (list, tuple)):
        for part in parts:
            if isinstance(part, dict) and part.get("role") == "cue":
                name = part.get("name")
                break
    elif isinstance(parts, dict):
        names = parts.get("cue")
        if isinstance(names, (list, tuple)) and names:
            name = names[0]
    return name if isinstance(name, str) and name.strip() else None


def shows_change(evidence, identity_verified):
    """Only cue evidence or identity-verified seen evidence shows a change."""
    return evidence == "cue" or (evidence == "seen" and identity_verified is True)


def _examples(available):
    """List one prescribed example for every recognized available form."""
    return " | ".join(EXAMPLES[item] for item in available if item in EXAMPLES)


def valid(until, available, parts=None):
    """Refuse missing, unreadable, unavailable, or misleading conditions."""
    available = tuple(available)
    if until is None or isinstance(until, str) and not _normalise(until):
        return {
            "check": "needs_until",
            "why": "a stop condition is required and is the operator's to state",
            "hint": _examples(available),
        }
    condition = parse(until)
    if condition is None:
        return {
            "check": "until",
            "why": "the stop condition could not be read: " + repr(until)[:40],
            "hint": _examples(FORMS),
        }
    if condition == {"seen": "changes"} and from_manual(parts) is not None:
        return {
            "check": "until_unavailable",
            "why": ("pushing any free thing changes its view; this cannot show "
                    "that the device changed; use cue changes"),
            "hint": _examples(available),
        }
    if form(condition) not in available:
        return {
            "check": "until_unavailable",
            "why": "its evidence source is not loaded",
            "hint": _examples(available),
        }
    return None


def _as_dict(value):
    """Treat absent or non-dict plain data as an empty reading."""
    return value if isinstance(value, dict) else {}


def _rows(record, key):
    """Return dict rows from a list reading, ignoring malformed entries."""
    entries = record.get(key, [])
    if not isinstance(entries, list):
        return []
    return [entry for entry in entries if isinstance(entry, dict)]


def _finite(value):
    """Accept finite real readings, excluding booleans and missing numbers."""
    if isinstance(value, Real) and not isinstance(value, bool):
        try:
            return float(value) if math.isfinite(value) else None
        except (OverflowError, ValueError):
            return None
    return None


def _move_reading(step):
    """Movement needs gave, moved_part, or positive threshold travel."""
    along = _finite(step.get("along_push_mm", 0))
    small = _finite(step.get("small_mm", 0))
    along = along if along is not None else 0.0
    small = small if small is not None else 0.0
    moved = (step.get("gave") is True or step.get("moved_part") is True
             or (small > 0 and along >= small))
    return moved, along


def _change_guard(step, record):
    """Unknown pair, whole movement, or lighting prevents a change verdict."""
    if not isinstance(record, dict):
        return None, "no same-view pair", "none"
    if _as_dict(step).get("moved_whole") is True:
        return None, "the pushed thing moved as a whole: that is not a state change", "none"
    if record.get("device_moved"):
        return None, "the thing in the picture moved as a whole between the looks", "none"
    if record.get("light_changed") is True:
        return None, "the light changed between the pictures", "none"
    return None


def _watch_message(row):
    """Describe a watched state change with colours when both are known."""
    before, after = row.get("colour_before"), row.get("colour_after")
    if before is not None and after is not None:
        return "the watched part changed from {} to {}".format(before, after)
    return "the watched part changed"


def _cue_changes(step, record, cue):
    """Only watched state changes or confirmed remote light changes count."""
    refused = _change_guard(step, record)
    if refused is not None:
        return refused
    changes = _rows(record, "changes")
    if _as_dict(cue).get("watch") is True:
        watched = [row for row in _rows(record, "at") if row.get("role") == "watch"]
        if not watched:
            return None, "what the robot watches is not in the picture", "none"
        for row in watched:
            before, after = row.get("colour_before"), row.get("colour_after")
            colour_change = (row.get("kind") is None and before is not None
                             and after is not None and before != after)
            if row.get("changed") == "yes" and (
                    row.get("kind") in LIGHT_KINDS or colour_change):
                return True, _watch_message(row), "cue"
        for change in changes:
            if change.get("at") == "watch" and change.get("kind") in LIGHT_KINDS:
                return True, _watch_message(change), "cue"
        if any(row.get("changed") == "yes" for row in watched):
            return (None, "the watched part moved or changed shape: "
                    "this is not a state change", "none")
        if all(row.get("changed") == "no" for row in watched):
            return False, "no change at the watched part", "cue"
        return None, "the watched part reads unclear", "none"
    remote = [change for change in changes
              if change.get("kind") in LIGHT_KINDS and change.get("at") != "acted"]
    if any(not change.get("on_own_body") and change.get("seen_again") is True
           for change in remote):
        return True, "a light or colour changed away from the pushed thing", "cue"
    if remote:
        if all(change.get("on_own_body") for change in remote):
            return None, "the change is on the robot's own body", "none"
        return None, "the change was not seen again in a second look", "none"
    return False, "no light or colour change away from the pushed thing", "cue"


def _cue_word(step, record, cue, word):
    """Compare an available cue state with the requested state word."""
    state = _as_dict(cue).get("state")
    if not isinstance(state, str) or not _normalise(state):
        return None, "no cue reading", "none"
    state = _normalise(state)
    if state == word:
        return True, "the cue reads " + word, "cue"
    return False, "the cue reads {}, not {}".format(state, word), "cue"


def _seen_changes(step, record, cue):
    """Read acted or watch rows, refusing whole movement and changed lighting."""
    refused = _change_guard(step, record)
    if refused is not None:
        return refused
    rows = [row for row in _rows(record, "at")
            if row.get("role") in ("acted", "watch")]
    for row in rows:
        if row.get("changed") == "yes":
            place = "pushed thing" if row.get("role") == "acted" else "watched part"
            return True, "changed at the " + place, "seen"
    if rows and all(row.get("changed") == "no" for row in rows):
        return False, "no change seen", "seen"
    return None, "the picture reads unclear", "none"


def _part_moves(step, record, cue):
    """Report movement from the step, with missing travel treated as zero."""
    if not isinstance(step, dict):
        return None, "no step reading", "none"
    moved, along = _move_reading(step)
    if moved:
        return True, "the pushed thing moved {:.1f} mm".format(along), "felt"
    return False, "the pushed thing did not move", "felt"


def _part_stays(step, record, cue):
    """A moved part stays only when the step does not report springing back."""
    if not isinstance(step, dict):
        return None, "no step reading", "none"
    moved, along = _move_reading(step)
    if not moved:
        return False, "the pushed thing did not move", "felt"
    if step.get("sprang_back"):
        return False, "it moved and sprang back", "felt"
    return True, "the pushed thing moved and stayed", "felt"


def _found(step, record, cue, word):
    """Match phrases or contained words, requiring comparable confidence."""
    if not isinstance(step, dict):
        return None, "no step reading", "none"
    entries = step.get("found_by", [])
    if not isinstance(entries, list):
        entries = []
    hit = None
    for entry in entries:
        if not isinstance(entry, str):
            continue
        entry = _normalise(entry)
        if entry == word or entry in word.split() or word in entry.split():
            hit = entry
            break
    if hit is None:
        return False, "nothing found by those words", "found"
    confidence = _finite(step.get("confidence"))
    threshold = _finite(step.get("threshold"))
    if confidence is None or threshold is None:
        return None, "no confidence to compare", "none"
    why = "found by {} ({:g}, threshold {:g})".format(hit, confidence, threshold)
    return confidence >= threshold, why, "found"


def _new_none(step, record, cue):
    """A known zero new count meets the condition; positive counts do not."""
    if not isinstance(step, dict) or "new" not in step:
        return None, "no step reading", "none"
    count = step["new"]
    if not isinstance(count, int) or isinstance(count, bool) or count < 0:
        return None, "the new count is unclear", "none"
    if count == 0:
        return True, "the look added nothing new", "found"
    return False, "the look added {} new".format(count), "found"


UNTIL = {
    "cue changes": _cue_changes,
    "cue ": _cue_word,
    "seen changes": _seen_changes,
    "part moves": _part_moves,
    "part stays": _part_stays,
    "found ": _found,
    "new none": _new_none,
}


def met(until, step, record, cue, table=None):
    """Dispatch exact text before prefixes; caller overrides return unchanged."""
    condition = parse(until)
    if condition is None:
        return None, "the stop condition could not be read", "none"
    text = words(condition)
    key, word = next(iter(condition.items()))
    prefix = key + " " if form(condition) in ("cue WORD", "found WORDS") else None
    if table is not None and text in table:
        return table[text](step, record, cue)
    if table is not None and prefix is not None and prefix in table:
        fn = table[prefix]
    else:
        fn = UNTIL[text if text in UNTIL else prefix]
    if prefix is not None and fn in (_cue_word, _found):
        return fn(step, record, cue, word)
    return fn(step, record, cue)


__all__ = [
    "EVIDENCE", "EXAMPLES", "FORMS", "LIGHT_KINDS", "UNTIL", "cue_part", "form",
    "from_manual", "met", "parse", "shows_change", "valid", "words",
]
