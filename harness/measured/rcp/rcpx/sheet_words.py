# SPDX-License-Identifier: AGPL-3.0-only
"""Sheet words: what the robot://devices document says, read the same way by every package. Pure:
the document as served (rcpx.context_devices.device_knowledge, any level) and text in, dicts out. Nothing here
names a device or a part: every rule reads a part's OWN entry (text in another part's entry never counts).

    parts(doc)                       per part: device, name, kind, role, moves, direction, approach_allowed,
                                     approach_forbidden, how_to_tell, notes, states, where, size_mm, forces,
                                     colours, self_moving (per device), sliding, forbidding (the item that forbids
                                     it), entry (the served entry itself)
    forces(entry)                    the sheet's forces for a part (force_n, a number or a range), [] when none
    role(part)                       forbidden | cue | control | fixed (precedence in that order)
    label_part(label, plist)         the part a located label names (3.2), or None
    goal_states(sentence, doc)       the goal states in sentence order, [None] when nothing matches
    step(state, doc)                 {state, device, deciding, control, done_class, colour, mark}
    steps(sentence, doc)             step() for every goal state
    do_line(part, doc, tools, body=None, f0_n=None)   {do, verbs, source, hypothesis} or None (no parts: D8)
    provenance(line, part, extra=()) (ok, words outside the template): outside quotes every word is in DO_WORDS

Roles (booleans over the part's own entry):
- forbidden = some item of its own approach_forbidden, moves or notes starts with "do not" and holds a word in
  FORBID_VERBS;
- cue = kind in CUE_KINDS, or (own moves holds "does not move" and its own how_to_tell or notes is not empty);
- control = not forbidden, not cue, and it gives a direction or approach_allowed;
- fixed = everything else.
sliding = kind in SLIDING_KINDS or own moves holds "slides"; self_moving (per device) = some part's own moves holds
"by itself" or "on its own".
"""

import math
import re

from rcpx import colour_words, r3_limits
from rcpx.context_devices import POSITION_WORDS

FORBID_VERBS = ("press", "push", "lift", "touch", "move", "open", "turn")
CUE_KINDS = ("lamp", "light", "indicator", "led", "display")
SLIDING_KINDS = ("slider", "drawer", "block", "object")
SELF_MOVING_WORDS = ("by itself", "on its own")
UP_WORDS = ("up", "upward", "lift", "raise")
SOURCE = "derived from this entry's kind and direction words"
TEMPLATE_QUOTES = ("up", "nudge")   # the D5 route's own quoted argument values (template words, not quotes)
# the fixed template words of every do line (outside quotes): the provenance test checks each line against this
DO_WORDS = frozenset("""
do not push it only shows the state seen reads after a on part that moves touch robot devices says no whole step
free object onto mark this round tool_help press runs through use sheet locate its face then pushes until stops
asks for an upward nudge and lift is enabled here approach target along surface direction up max_mm travel_mm
force_n mode raising x after moved came back from under edge or as of tool faces possible nothing to with
read act gap you saw rcp offers the force raises in one ramp
""".split())


def _items(v):
    """The text items of a field: a text, a list of texts, or a dict keyed by state (a full sheet's direction
    {"open": "nudge the lip up"} reads "open: nudge the lip up")."""
    if isinstance(v, str):
        return [v.strip()] if v.strip() else []
    if isinstance(v, (list, tuple)):
        return [str(x).strip() for x in v if str(x).strip()]
    if isinstance(v, dict):
        return [f"{k}: {str(x).strip()}" for k, x in v.items() if isinstance(x, str) and x.strip()]
    return []


def _text(v):
    return "; ".join(_items(v))


def _words(text):
    return re.findall(r"[a-z_]+", text.lower())


def _forbidding(entry):
    for key in ("approach_forbidden", "moves", "notes"):
        for item in _items(entry.get(key)):
            low = item.lower()
            if low.startswith("do not") and any(w in FORBID_VERBS for w in _words(low)):
                return item
    return None


def role(entry):
    """forbidden | cue | control | fixed, from the part's own entry (precedence in that order)."""
    if _forbidding(entry) is not None:
        return "forbidden"
    kind = str(entry.get("kind") or "").strip().lower()
    moves = _text(entry.get("moves")).lower()
    told = _text(entry.get("how_to_tell")) or _text(entry.get("state_cue")) or _text(entry.get("notes"))
    if kind in CUE_KINDS or ("does not move" in moves and told):
        return "cue"
    if _items(entry.get("direction")) or _items(entry.get("approach_allowed")):
        return "control"
    return "fixed"


def _sliding(entry):
    kind = str(entry.get("kind") or "").strip().lower()
    return kind in SLIDING_KINDS or "slides" in _text(entry.get("moves")).lower()


def _self_moving_entry(entry):
    moves = _text(entry.get("moves")).lower()
    return any(w in moves for w in SELF_MOVING_WORDS)


def _sizes(entry):
    """The part's sizes in mm: its size_mm (manual level, rough), else the sizes in where_mm (full level), never a
    key that names a position (rcpx.context_devices.POSITION_WORDS)."""
    for key in ("size_mm", "where_mm"):
        v = entry.get(key)
        if isinstance(v, dict):
            out = {k: float(x) for k, x in v.items() if isinstance(x, (int, float)) and not isinstance(x, bool)
                   and x > 0 and "tolerance" not in k and not any(w in str(k) for w in POSITION_WORDS)}
            if out:
                return out
    return {}


def forces(entry):
    """The sheet's forces for a part in N (force_n: a number or a [low, high] range); [] when not given."""
    v = entry.get("force_n")
    vals = v if isinstance(v, (list, tuple)) else [v]
    return [float(x) for x in vals if isinstance(x, (int, float)) and not isinstance(x, bool) and x > 0]


def _colours(entry):
    """{state: colour} from the part's own how_to_tell (or state_cue) words."""
    out = {}
    for colour, state in colour_words.words(_text(entry.get("how_to_tell")) or _text(entry.get("state_cue"))):
        if state is not None and state not in out:
            out[state] = colour
    return out


def parts(doc):
    """Every part of every device in the document, in document order, with its role and words."""
    out = []
    for dev in (doc or {}).get("devices") or []:
        entries = [p for p in dev.get("parts") or [] if isinstance(p, dict)]
        moving = any(_self_moving_entry(p) for p in entries)
        for p in entries:
            nm = str(p.get("part") or p.get("name") or "").strip().lower()
            out.append({
                "device": dev.get("id"), "name": nm, "kind": str(p.get("kind") or "").strip().lower(),
                "role": role(p), "moves": _text(p.get("moves")), "direction": _text(p.get("direction")),
                "approach_allowed": _items(p.get("approach_allowed")),
                "approach_forbidden": _items(p.get("approach_forbidden")),
                "how_to_tell": _text(p.get("how_to_tell")) or _text(p.get("state_cue")), "notes": _text(p.get("notes")),
                "states": [str(s).strip().lower() for s in _items(p.get("states"))], "where": _text(p.get("where")),
                "size_mm": _sizes(p), "forces": forces(p), "colours": _colours(p), "self_moving": moving,
                "sliding": _sliding(p), "forbidding": _forbidding(p), "entry": p})
    return out


def _label_words(text):
    return [w for w in re.split(r"[\s_\-]+", str(text).strip().lower()) if w]


def label_part(label, plist):
    """The part a label names: the part whose name equals the label; else the part all of whose name's words are
    among the label's words (split on space, "_" and "-"), the one with the most words winning (the first on a
    tie); else None. "block_now" gives block, "square_centre" square, "dark_dot" none."""
    if label is None:
        return None
    lab = str(label).strip().lower()
    if lab.startswith("other:"):
        return None
    for p in plist:
        if p["name"] == lab:
            return p
    have = set(_label_words(lab))
    best, best_n = None, 0
    for p in plist:
        need = _label_words(p["name"])
        if need and set(need) <= have and len(need) > best_n:
            best, best_n = p, len(need)
    return best


# ---------------------------------------------------------------- goal states
_CLAUSE_SPLIT = re.compile(r",|;|\bthen\b|\bafter that\b|\band then\b")


def _close_state(plist):
    has = {s for p in plist for s in p["states"]}
    for p in plist:
        for s in p["states"]:
            if s in ("closed", "shut"):
                return s
    return "closed" if "closed" in has or not has else "shut"


def goal_states(sentence, doc):
    """The goal states of a task sentence, in sentence order: open or opened gives open; close,
    closed or shut gives whichever of closed or shut a part's states list holds; "turn ... on", "switch ... on" or a
    clause's last word "on" gives on (the same for off); onto, into or inside with a mark part on the device gives
    the object's state that starts with in_. Nothing matched: [None]."""
    plist = parts(doc)
    text = str(sentence or "").lower()
    found = []                          # (position, state)
    pos = 0
    for clause in _CLAUSE_SPLIT.split(text):
        start = text.find(clause, pos)
        pos = start + len(clause) if start >= 0 else pos
        ws = re.findall(r"[a-z_']+", clause)
        offs = [m.start() + max(start, 0) for m in re.finditer(r"[a-z_']+", clause)]
        for i, w in enumerate(ws):
            if w in ("open", "opened"):
                found.append((offs[i], "open"))
            elif w in ("close", "closed", "shut"):
                found.append((offs[i], _close_state(plist)))
            elif w in ("on", "off"):
                led = any(v in ("turn", "switch") for v in ws[:i])
                if led or i == len(ws) - 1:
                    found.append((offs[i], w))
            elif w in ("onto", "into", "inside"):
                if any(p["role"] == "fixed" and p["kind"] == "mark" for p in plist):
                    obj = next((s for p in plist if p["sliding"] for s in p["states"] if s.startswith("in_")),
                               None)
                    if obj is not None:
                        found.append((offs[i], obj))
    found.sort(key=lambda t: t[0])
    states = []
    for _, s in found:
        if not states or states[-1] != s:
            states.append(s)
    return states or [None]


def _device_parts(plist, device):
    return [p for p in plist if p["device"] == device]


def step(state, doc, device=None):
    """For one goal state: the deciding part (a cue whose colours give one for the state; else a control part
    whose states hold it; else any part whose states hold it), the control part (the control part whose states
    hold it; else the device's only control part) and the done class (cue, mark, sliding, self_moving, generic).
    device: the device id (the first device whose deciding part exists, else the first device)."""
    plist = parts(doc)
    devs = [device] if device is not None else list(dict.fromkeys(p["device"] for p in plist))
    pick = None
    for dev in devs:
        dp = _device_parts(plist, dev)
        deciding = next((p for p in dp if p["role"] == "cue" and state in p["colours"]), None) \
            or next((p for p in dp if p["role"] == "control" and state in p["states"]), None) \
            or next((p for p in dp if state in p["states"]), None)
        if deciding is not None or pick is None:
            pick = (dev, dp, deciding)
        if deciding is not None:
            break
    if pick is None:
        return {"state": state, "device": None, "deciding": None, "control": None, "done_class": "generic",
                "colour": None, "mark": None}
    dev, dp, deciding = pick
    controls = [p for p in dp if p["role"] == "control"]
    control = next((p for p in controls if state in p["states"]), None) or (controls[0] if len(controls) == 1
                                                                             else None)
    mark = next((p for p in dp if p["role"] == "fixed" and p["kind"] == "mark"), None)
    colour = deciding["colours"].get(state) if deciding is not None and deciding["role"] == "cue" else None
    if colour is not None:
        cls = "cue"
    elif mark is not None and control is not None and control["sliding"]:
        cls = "mark"
    elif control is not None and control["sliding"]:
        cls = "sliding"
    elif deciding is not None and _self_moving_entry(deciding["entry"]):
        cls = "self_moving"
    else:
        cls = "generic"
    return {"state": state, "device": dev, "deciding": deciding["name"] if deciding else None,
            "control": control["name"] if control else None, "done_class": cls, "colour": colour,
            "mark": mark["name"] if mark and cls == "mark" else None}


def steps(sentence, doc):
    return [step(s, doc) for s in goal_states(sentence, doc)]


# ---------------------------------------------------------------- do lines
def _cut(text, n=r3_limits.QUOTE_MAX_CHARS):
    t = str(text or "").strip()
    if len(t) <= n:
        return t
    head = t[:n]
    return head[:head.rfind(" ")].rstrip(" ,;:") if " " in head else head


def _up_surface(body):
    """(name, None) of the tool surface whose outward normal is nearest the hand's up (tool z) within
    SURFACE_UP_DEG, from robot://body tool.surfaces; (None, why) when none faces up or the body is not given."""
    surfaces = ((body or {}).get("tool") or {}).get("surfaces") or []
    best, best_a = None, None
    for s in surfaces:
        n = s.get("normal_in_tool")
        if not n or len(n) != 3:
            continue
        norm = math.sqrt(sum(float(x) ** 2 for x in n)) or 1.0
        a = math.degrees(math.acos(max(-1.0, min(1.0, float(n[2]) / norm))))
        if a <= r3_limits.SURFACE_UP_DEG and (best_a is None or a < best_a):
            best, best_a = s.get("name"), a
    return best


def _fmt(x):
    return f"{float(x):g}"


def do_line(part, doc, tools, body=None, f0_n=None):
    """The do line of one part (rules D1 to D7, the first that fits); None for no part.
    part: a parts() entry (or a part name); tools: the names the session loads and lists or reaches through use;
    body: robot://body (the D5 route's surface); f0_n: retained for callers, ignored."""
    plist = parts(doc)
    if isinstance(part, str):
        part = label_part(part, plist)
    if not part:
        return None
    tools = set(tools or ())
    dev_parts = _device_parts(plist, part["device"])
    has_mark = any(p["role"] == "fixed" and p["kind"] == "mark" for p in dev_parts)
    q = _cut(part["direction"])
    says = f'; the sheet says "{q}"' if q else ""
    r = part["role"]
    verbs = []
    if r == "cue":
        line = "do not push: it only shows the state; seen reads it after a push on the part that moves"
    elif r == "forbidden":
        line = f'do not touch: robot://devices says "{part["forbidding"]}"'
    elif r == "control" and part["sliding"] and has_mark:
        line, verbs = "no whole step moves a free object onto a mark this round: tool_help press (press runs " \
                      "through use)" + says, ["press"]
    elif r == "control" and part["sliding"] and "push" in tools:
        line, verbs = "locate its face, then push it: it pushes until the part stops" + says, ["push"]
    elif r == "control" and any(w in UP_WORDS for w in _words(q)):
        if "lift" in tools:
            line, verbs = f'locate it, then lift it from under its edge or push it, as the sheet says: "{q}"', \
                          ["lift", "push"]
        else:
            surf = _up_surface(body)
            if surf is None:
                line = "no surface of this tool faces up: an upward nudge is not possible here" + says
            else:
                line = ("the sheet asks for an upward nudge and lift is not enabled here: use approach {target, "
                        f"along: [0, 0, 1], surface: {surf}}}, then use touch {{direction: \"up\", max_mm: "
                        "the gap you saw}, then use press {direction: \"up\", mode: \"nudge\"}: "
                        "RCP offers the force and raises it in one ramp until the part moves" + says)
                verbs = ["approach", "touch", "press"]
    elif r == "control" and "push" in tools:
        line, verbs = ("locate it, then push it" + says if q else "locate it, then push it; read seen"), ["push"]
    elif r == "control":
        line, verbs = ('act on it as the sheet says: "' + q + '"' if q else "act on it; read seen"), ["press"]
    else:
        line = "nothing to do with this part" + (f'; the sheet says "{part["how_to_tell"]}"'
                                                 if part["how_to_tell"] else "")
    return {"do": line, "verbs": verbs, "source": SOURCE, "hypothesis": True}


def provenance(line, part, extra=()):
    """(ok, bad): outside quotes every word of the line is in DO_WORDS (or extra: a tool surface name from
    robot://body); every quote is a substring of the part's own served entry."""
    text = line["do"] if isinstance(line, dict) else str(line)
    quotes = re.findall(r'"([^"]*)"', text)
    outside = re.sub(r'"[^"]*"', " ", text)
    allowed = set(DO_WORDS) | {str(w).lower() for w in extra}
    bad = [w for w in _words(outside) if w not in allowed]
    entry = part.get("entry", part) if isinstance(part, dict) else {}
    served = " ".join(_text(v) for v in entry.values() if isinstance(v, (str, list, tuple, dict)))
    bad += [f'quote "{qt}"' for qt in quotes if qt and qt not in served and qt not in TEMPLATE_QUOTES]
    return not bad, bad


__all__ = ["CUE_KINDS", "DO_WORDS", "FORBID_VERBS", "SLIDING_KINDS", "do_line", "forces", "goal_states", "label_part",
           "parts", "provenance", "role", "step", "steps"]
