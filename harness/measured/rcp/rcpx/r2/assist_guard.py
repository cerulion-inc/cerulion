# SPDX-License-Identifier: AGPL-3.0-only
"""What the robot may do on its own: pure rules, plain data in, a verdict out.

Every tool of this package that moves the robot without a call of the model for that very motion, and every call the
surface writes for the model, asks this file first. A verdict is None (allowed) or the reason; a refusal is always
{"check", "why", "hint"}. Data that is missing or cannot be read gets the strict verdict, never the loose one.
Nothing here names a robot, a device, a part or a task, and no force is typed here (a force comes from
rcpx.r3_limits or from the caller). No clock, no randomness, no file. Only bind reads an executor.

    safe_call(tool, args)                   rule 1: why a written call is not allowed (it would raise a force)
    target_ok(entry, flags, roles)          rule 2: why a written act call may not name this located entry
    keep_off(forbidden, a, b, own, tool)    rule 3: the refusal for a line (a spot when b is None) that runs too
                                            near a forbidden part, or that cannot be told from one not located
    felt_in_keep_off(forbidden, p, own, tool)   rule 3 again for the point a touch felt (no unplaced clause)
    first_force(noise_n, sheet_forces)      rule 4: the one step a robot-made press takes on an unverified entry
    skip_reason(row, listed, watch_n, roles)    rule 5: why a routine must not act on a candidate row
    list_stale(lst, epoch, moved_since, look_stale)   rule 6: why a candidate list may no longer be acted on
    Bounds(steps, calls_left, seconds_left, reach_mm, step_seconds)   rule 7: steps, time and travel
    charge(step)                            rule 9: what a robot-run step costs the operator
    bind(ex)                                the rules bound to one executor (the only reader of one)

Rule 3 restates the act package's own rule (rcpx/r2/act_guard.py: forbidden_zones, _keep_off, _nearest,
forbidden_where, forbidden_at, tool_radius_mm, epoch_stale) because a package may not import another; on the same
numbers it gives the same verdict (tests/test_r2_assist_guard.py holds the vectors and runs both). Where this file
knows less it refuses: a forbidden part located with no stated uncertainty counts as not located (the act package
derives one from the marks). When the act package serves forbidden_where and forbidden_at through ex.state["act"],
bind calls those in place of the restatement. Not restated: the refusal of a target whose label names no part
(target_ok is the rule for labels) and the ray of a target with no depth (a caller gives the line it will run).
"""

import math
import re

from rcpx import r3_limits as L
from rcpx import sheet_words

UNSAFE_ARGS = ("firm", "further", "force_n", "max_n")
FORCE_KEY_END = "_n"         # on this surface an argument whose name ends so carries a force
MAX_DEPTH = 12               # a written call nested deeper than this cannot be checked, so it is not allowed
ROLE_RANK = {"control": 0, "fixed": 1, "cue": 2, "forbidden": 3}     # two parts of one name: the strictest role
_NUMBER = r"(?<![A-Za-z0-9_])(?:\d+(?:\.\d+)?|\.\d+)\s*"
_NEWTONS = (re.compile(_NUMBER + r"N(?![A-Za-z0-9_])"), re.compile(_NUMBER + r"newtons?\b", re.IGNORECASE))


# ---------------------------------------------------------------- reading plain data strictly
def _num(value, default=None):
    """value as a finite float; default for None, a text, a truth value, NaN, infinity or anything else."""
    if value is None or isinstance(value, (bool, str, bytes)) or type(value).__name__ in ("bool_", "bool"):
        return default
    try:
        number = float(value)
    except Exception:
        return default
    return number if math.isfinite(number) else default


def _int(value):
    number = _num(value)
    return int(number) if number is not None and number == int(number) else None       # whole numbers only


def _flag(value):
    """Is a flag set: None is not; a text or a sequence is when it is not empty; a value that cannot be read is."""
    try:
        return value is not None and (len(value) > 0 if hasattr(value, "__len__") else bool(value))
    except Exception:
        return True


def _is_true(value):
    """Is this one plain yes (a sequence, a text or a value that cannot be read is not)."""
    try:
        return not hasattr(value, "__len__") and bool(value == True)      # noqa: E712  (numpy's yes is not True)
    except Exception:
        return False


def _word(value):
    return value.strip().lower() if isinstance(value, str) else None


def _dict(value):
    return value if isinstance(value, dict) else {}


def _point(value):
    """[x, y, z] as three finite floats, else None."""
    if value is None or isinstance(value, (str, bytes, dict)):
        return None
    try:
        point = [_num(x) for x in value]
    except Exception:
        return None
    return point if len(point) == 3 and all(x is not None for x in point) else None


def _dist(p, a, b=None):
    """The distance from p to the segment a to b (to the point a when b is None): the act package's _seg_dist."""
    t = 0.0
    if b is not None:
        ab = [b[i] - a[i] for i in range(3)]
        dot = sum(x * x for x in ab)
        t = 0.0 if dot < 1e-12 else min(1.0, max(0.0, sum((p[i] - a[i]) * ab[i] for i in range(3)) / dot))
        a = [a[i] + t * ab[i] for i in range(3)]
    return math.sqrt(sum((p[i] - a[i]) * (p[i] - a[i]) for i in range(3)))


def _refusal(check, why, hint):
    return {"check": check, "why": why, "hint": hint}


# ---------------------------------------------------------------- rule 1: written calls
def _unsafe_text(text, is_key):
    word = text.strip().lower()
    if word in UNSAFE_ARGS or (is_key and word.endswith(FORCE_KEY_END)):
        return f"{word} raises a force: a call the robot writes never carries it"
    if _NEWTONS[0].search(text) or _NEWTONS[1].search(text):
        return "it states a force in newtons: a call the robot writes never does"
    return None


def _walk(value, depth):
    if depth > MAX_DEPTH:
        return "nested too deep to check"
    if value is None or isinstance(value, (bool, int, float)):
        return None
    if isinstance(value, str):
        return _unsafe_text(value, False)
    if isinstance(value, dict):
        for key, item in value.items():
            reason = _unsafe_text(key, True) if isinstance(key, str) else _walk(key, depth + 1)
            reason = reason or _walk(item, depth + 1)
            if reason is not None:
                return reason
        return None
    if isinstance(value, (list, tuple)):
        return next((r for r in (_walk(item, depth + 1) for item in value) if r is not None), None)
    if isinstance(getattr(value, "ndim", None), int) and callable(getattr(value, "tolist", None)):
        return _walk(value.tolist(), depth)          # an array or a scalar of the array library
    return "it holds a value that cannot be checked"


def safe_call(tool, args):
    """Rule 1. The reason a call the surface writes is not allowed, else None. Not allowed at any depth of args
    (a nested call's own arguments included): a key or a text value that is one of UNSAFE_ARGS; a key that ends in
    FORCE_KEY_END; a text that states a number of newtons; a value that is no plain data; nesting over MAX_DEPTH."""
    try:
        return _walk(tool, 1) or _walk(args, 1)
    except Exception:
        return "it holds a value that cannot be checked"


# ---------------------------------------------------------------- rule 2: targets of a written act call
def target_ok(entry, flags, roles):
    """Rule 2. Why a written act call may not name this located entry, the first that holds, else None. entry:
    {"id", "label", "stale", "epoch_ok", "in_reach" (True | False | None), "identity": {"verified"} | None,
    "part_check"}; flags: {"mislabel", "body", "air", "no_give", "pressed_no_change"} (missing: false or 0); roles:
    {part name: role}. A missing epoch_ok or in_reach passes (no stamp; the caller's dry check decides)."""
    if not isinstance(entry, dict):
        return "no entry"
    flags, roles = _dict(flags), _dict(roles)
    if _flag(entry.get("stale")) or (entry.get("epoch_ok") is not None and not _is_true(entry["epoch_ok"])):
        return "stale"
    if entry.get("in_reach") is not None and not _is_true(entry["in_reach"]):
        return "out of reach"
    label = entry.get("label")
    role = None if label is None else _word(roles.get(str(label).strip().lower()))
    if role != "control":
        return role if role in ("cue", "fixed", "forbidden") else "names no part"
    identity = entry.get("identity")
    if identity is not None and not (isinstance(identity, dict) and _is_true(identity.get("verified"))):
        return "not verified"
    if _flag(entry.get("part_check")):
        return "colour warning"
    for key, reason in (("mislabel", "mislabel"), ("body", "own body"), ("air", "ran free")):
        if _flag(flags.get(key)):
            return reason
    no_give = flags.get("no_give")          # a count; a plain yes or a value that is no number counts as many
    if _flag(no_give) and _num(no_give, math.inf) >= 2:
        return "did not give twice"
    return "pressed, no change" if _flag(flags.get("pressed_no_change")) else None


# ---------------------------------------------------------------- rule 3: keep off what the manual forbids
def _words(zone):
    """(the name, the manual's sentence) of one forbidden part, with stand-ins for what is missing."""
    name, said = zone.get("name"), zone.get("forbidding")
    name = name if isinstance(name, str) and name.strip() else "NAME"
    return name, said if isinstance(said, str) and said.strip() else f"do not touch the {name}"


def _spots(zone):
    """The usable placed points of one forbidden part: [(id, point, uncertain_mm)]."""
    out = []
    try:
        for spot in list(zone.get("placed") or []):
            point, unc = _point(_dict(spot).get("point_mm")), _num(_dict(spot).get("uncertain_mm"))
            if point is not None and unc is not None and unc >= 0.0:
                out.append((spot.get("id"), point, unc))
    except Exception:
        return []
    return out


def _contact(forbidden, a_mm, b_mm, own_unc_mm, tool_mm, lead, unplaced_too):
    try:                                # a list that cannot be read: one unnamed part that is not located
        zones = [_dict(z) for z in list(forbidden or [])]
    except Exception:
        zones = [{}]
    if not zones:
        return None
    no_numbers = _refusal("forbidden", "the line the tool will run has no numbers: the robot cannot keep it off "
                          "the " + _words(zones[0])[0], "locate the target first")
    a, b = _point(a_mm), _point(b_mm)
    if a is None or (b_mm is not None and b is None):
        return no_numbers
    own, tool = max(0.0, _num(own_unc_mm, 0.0)), max(0.0, _num(tool_mm, 0.0))
    best, unplaced = None, None
    for zone in zones:
        spots = _spots(zone)
        for eid, point, unc in spots:
            d, keep = _dist(point, a, b), unc + own + tool          # the act package's sum, in its order
            if not (math.isfinite(d) and math.isfinite(keep)):
                return no_numbers
            if best is None or d - keep < best[0] - best[1]:        # the smallest margin; the first on a tie
                best = (d, keep, zone, eid)
        if not spots and unplaced is None:
            unplaced = zone
    if best is not None and best[0] <= best[1]:
        d, keep, zone, eid = best
        name, said = _words(zone)
        return _refusal("forbidden", f'{lead} {d:.0f} mm from the {name} ({eid}): the manual says "{said}"',
                        f"choose a spot over {keep:.0f} mm from it")
    if unplaced is not None and unplaced_too:
        name, said = _words(unplaced)
        state = ("moved since it was located" if _flag(unplaced.get("seen_stale")) else
                 "is located with no stated uncertainty" if _flag(unplaced.get("unsized")) else "is not located")
        return _refusal("forbidden_unplaced", f'the manual says "{said}" and the {name} {state}: '
                        "the robot cannot tell this spot from it",
                        f"locate the {name} first, or make the push yourself: your own push has the same rule")
    return None


def keep_off(forbidden, a_mm, b_mm=None, own_unc_mm=0.0, tool_mm=0.0):
    """Rule 3, before a contact move the robot makes on its own. forbidden: [{"name", "forbidding" (the manual's
    sentence), "placed": [{"id", "point_mm", "uncertain_mm"}], "seen_stale", "unsized"}], one per part the manual
    forbids; a_mm to b_mm is the line the tool will run (mm; a spot when b_mm is None). The placed point with the
    smallest distance less keep-off (its uncertain_mm + own_unc_mm + tool_mm) decides: at or inside its keep-off
    the line is refused forbidden. Else a part with no usable placed point refuses forbidden_unplaced."""
    lead = "this spot is" if b_mm is None else "this line passes"
    return _contact(forbidden, a_mm, b_mm, own_unc_mm, tool_mm, lead, True)


def felt_in_keep_off(forbidden, point_mm, own_unc_mm=0.0, tool_mm=0.0):
    """Rule 3 again for the point a touch felt: the same distance test, without the unplaced clause. A refusal
    means the caller backs off and stops."""
    return _contact(forbidden, point_mm, None, own_unc_mm, tool_mm, "the felt point is", False)


# ---------------------------------------------------------------- rule 4: the first force
def first_force(noise_n, sheet_forces):
    """Rule 4. The verb's own first step (rcpx.r3_limits.f0 of the force noise and the manual's forces for the
    part: a list, one number or None), rounded as the ladder rounds. The caller sends it as the ceiling of a press
    or push on an entry whose identity is not verified, so the ladder has that one step and does not climb."""
    noise = _num(noise_n, -1.0)
    noise = noise if noise >= 0.0 else L.NOISE_DEFAULT_N        # no number, or under 0: the default noise
    try:
        given = [] if sheet_forces is None else [sheet_forces] if _num(sheet_forces) is not None else list(sheet_forces)
        forces = [x for x in (_num(f) for f in given) if x is not None and x > 0.0] or None
    except Exception:
        forces = None
    f = _num(L.f0(noise, forces))
    if f is None:                                    # a noise too large to multiply: the default noise
        f = L.f0(L.NOISE_DEFAULT_N, forces)
    return round(round(f / L.LADDER_ROUND_N) * L.LADDER_ROUND_N, 1)


# ---------------------------------------------------------------- rule 5: rows a routine never acts on
def _same(n, watch_n):
    """Is row number n the watched one (an answer that is not one yes or no counts as yes)."""
    try:
        same = n == watch_n
        return hasattr(same, "__len__") or bool(same) or (n is not None and str(n).strip() == str(watch_n).strip())
    except Exception:
        return True


def skip_reason(row, listed, watch_n, roles):
    """Rule 5. Why a routine must not act on this candidate row, the first that holds, else None. row: {"n",
    "found_by", "role", "colour", "manual", "keep_off", "own_body", "stale", "cue_colour"}; roles: {part name:
    role}. Every reason but the last holds even for a number the model listed; the last is for a manual that
    forbids a part (the numbers are then the model's to list)."""
    if not isinstance(row, dict):
        return "no row"
    roles, found_by = _dict(roles), row.get("found_by")
    words = {_word(row.get("role"))}
    if isinstance(found_by, str):
        words |= {_word(roles.get(found_by)), _word(roles.get(_word(found_by)))}
    if _flag(row.get("keep_off")) or "forbidden" in words:
        return "the manual forbids it"
    if _flag(row.get("own_body")):
        return "the robot's own body"
    if watch_n is not None and _same(row.get("n"), watch_n):
        return "it is what the robot watches"
    if _flag(row.get("cue_colour")) or "cue" in words:
        return "it has the cue's colour"
    if _flag(row.get("stale")):
        return "stale"
    if row.get("colour") is None:
        return "no colour reading"
    if not _flag(listed) and any(_word(role) == "forbidden" for role in roles.values()):
        return "not listed: the manual forbids a part here"
    return None


# ---------------------------------------------------------------- rule 6: a list goes stale with the scene
def list_stale(lst, epoch, moved_since, look_stale=False):
    """Rule 6. Why a candidate list may no longer be acted on, else None. lst: {"look", "epoch", "call"};
    moved_since: [{"call", "why"}], the acts that moved the scene (the base or the device moved, a thing was shoved
    as a whole, a contact slid or was not confirmed). An act counts when its call is over the list's or unknown."""
    if not isinstance(lst, dict):
        return "no list"
    old, now = _int(lst.get("epoch")), _int(epoch)
    if old is None or now is None:
        return "its epoch is not known"
    if old != now:
        return "the scene's epoch rose"
    if _flag(look_stale):
        return "its look is stale"
    made = _int(lst.get("call"))
    try:
        records = [] if moved_since is None else list(moved_since)
    except Exception:
        records = [None]
    for record in map(_dict, records):
        call = _int(record.get("call"))
        if made is None or call is None or call > made:
            return (f'the act at call {record.get("call")} reported {record.get("why")}: the scene moved since the '
                    "list was made")
    return None


# ---------------------------------------------------------------- rule 7: bounds of what the robot runs itself
class Bounds:
    """Rule 7. The bounds of one robot-run tool: the steps asked, a quarter of the seconds the session has left
    at its start (none with no time budget), commanded travel of ROUTINE_REACH x the reach radius in all and
    MOVE_REACH x in one move. before() is called before EVERY nested call; a nested call cannot be cut short."""
    TIME_SHARE = 0.25
    ROUTINE_REACH = 3.0
    MOVE_REACH = 1.0
    SPARE_CALLS = 2

    def __init__(self, steps, calls_left, seconds_left, reach_mm, step_seconds=None):
        self.steps = max(0, _int(steps) or 0)
        self.calls_left = None if calls_left is None else _num(calls_left, 0.0)          # None: no call budget
        self.seconds_left = None if seconds_left is None else _num(seconds_left, 0.0)    # None: no time budget
        self.reach_mm = max(0.0, _num(reach_mm, 0.0))                                    # unknown: no move at all
        self.step_seconds = None if step_seconds is None else _num(step_seconds, math.inf)
        self.time_bound_s = None if self.seconds_left is None else self.seconds_left * self.TIME_SHARE
        self.travel_bound_mm = self.reach_mm * self.ROUTINE_REACH
        self.move_bound_mm = self.reach_mm * self.MOVE_REACH

    def start(self):
        """None, or the refusal budget_short (calls under steps + SPARE_CALLS) or time_short (under one step)."""
        need = self.steps + self.SPARE_CALLS
        if self.calls_left is not None and self.calls_left < need:
            return _refusal("budget_short", f"{self.steps} steps need {need} calls and {self.calls_left:g} are left",
                            "ask for fewer steps")
        if self.step_seconds is not None and self.seconds_left is not None and self.seconds_left < self.step_seconds:
            return _refusal("time_short", f"one step can take {self.step_seconds:g} s and {self.seconds_left:g} s "
                            "are left", "make the move with your own call")
        return None

    def before(self, steps_started, seconds_used, travel_mm, next_mm=0.0):
        """The bound the next nested call would pass, else None: "steps" (steps_started at or over the steps), "time"
        (seconds_used at or over its bound), "travel" (travel_mm + next_mm over the routine's, next_mm over one's)."""
        if _num(steps_started, math.inf) >= self.steps:
            return "steps"
        seconds = _num(seconds_used)
        if self.time_bound_s is not None and (seconds is None or seconds >= self.time_bound_s):
            return "time"
        travel, nxt = _num(travel_mm, -1.0), _num(next_mm, -1.0)      # one that cannot be read reads as under 0
        over = travel < 0.0 or nxt < 0.0 or travel + nxt > self.travel_bound_mm or nxt > self.move_bound_mm
        return "travel" if over else None

    def worst_case(self):          # one sentence for a tool's description
        return ("A bound is tested before every nested call and a nested call cannot be cut short, so this tool "
                "can pass its bound by one nested move.")


# ---------------------------------------------------------------- rule 9: the charge
def charge(step):
    """Rule 9. What a robot-run step {"chose", "acted", "looked"} costs the operator: 1 for each that is true."""
    return sum(1 for key in ("chose", "acted", "looked") if _flag(_dict(step).get(key)))


# ---------------------------------------------------------------- bind: the one reader of an executor
def _state(ex, key):
    return _dict(_dict(getattr(ex, "state", None)).get(key))


def _manual(ex):
    """(known, parts): the manual is known when ex.context["devices"] is a document with a list of devices."""
    try:
        doc = _dict(getattr(ex, "context", None)).get("devices")
        if isinstance(doc, dict) and isinstance(doc.get("devices"), list):
            return True, sheet_words.parts(doc)
    except Exception:
        pass
    return False, []


def _epoch_ok(ex, e):
    """Is the entry of this epoch (the act package's epoch_stale): the epochs stamp, else data.epoch; none passes."""
    try:
        stamp = _dict(_dict(_state(ex, "epochs").get("stamps")).get(e.get("id")))
        ep = stamp.get("epoch", _dict(e.get("data")).get("epoch"))
        return ep is None or (_int(ep) is not None and _int(ep) == _int(getattr(ex, "epoch", 0)))
    except Exception:
        return False


def _tool_mm(ex, sensed=True):
    """The radius (mm) of the tool capsule of the robot's own collision model whose axis passes nearest the tip
    (the act package's tool_radius_mm, restated); 0 when there is none. sensed False reads no sensor: the body
    model's home pose stands in (bind itself must not take a reading: a reading moves the force observer)."""
    try:
        s = ex._sense() if sensed else None
        q = s.q if sensed else ex.model.q_home
        tip, best = _point(s.tip if sensed else ex.model.fk_tip(q)[0]), None
        tool = getattr(ex, "_tool_bodies", None) or ()
        for capsule in ex.model.collision_capsules(q):
            if capsule[0] in tool:
                d, radius = _dist(tip, _point(capsule[1]), _point(capsule[2])), _num(capsule[3])
                if radius is not None and radius >= 0.0 and (best is None or d < best[0]):
                    best = (d, radius)
        return 0.0 if best is None else 1000.0 * best[1]
    except Exception:
        return 0.0


def _forbidden(ex, parts):
    """The list keep_off takes: per part the manual forbids, the fresh located entries whose label names it. A
    stale entry or one of an older epoch places nothing and sets seen_stale; one with no stated uncertainty
    places nothing and sets unsized (this file cannot derive one, so the part counts as not located)."""
    try:
        entries = [e for e in list(ex.world.query(kind="located", include_stale=True) or []) if isinstance(e, dict)]
    except Exception:
        entries = []
    out = []
    for p in parts:
        if p.get("role") != "forbidden":
            continue
        zone = {"name": p.get("name"), "forbidding": p.get("forbidding"), "placed": [], "seen_stale": False,
                "unsized": False}
        for e in entries:
            try:
                if e.get("point_mm") is None or sheet_words.label_part(e.get("label"), parts) is not p:
                    continue
                if _flag(e.get("stale")) or not _epoch_ok(ex, e):
                    zone["seen_stale"] = True
                    continue
                point, unc = _point(e.get("point_mm")), _num(_dict(e.get("data")).get("uncertain_mm"))
                if unc is None or unc < 0.0:
                    zone["unsized"] = True
                elif point is not None:
                    zone["placed"].append({"id": e.get("id"), "point_mm": point, "uncertain_mm": unc})
            except Exception:
                zone["unsized"] = True
        out.append(zone)
    return out


def _reach_mm(ex, fallback):
    """The reach radius of robot://body (as the runner served it, else the body model's own), else the caller's
    fallback, else 0: never None."""
    def served():
        doc = _dict(getattr(ex, "context", None)).get("body")
        return doc() if callable(doc) else doc

    for read in (served, lambda: {"reach": ex.model.reach()}):
        try:
            radius = _num(read()["reach"]["max_reach_mm"])
            if radius is not None and radius > 0.0:
                return radius
        except Exception:
            pass
    return max(0.0, _num(fallback, 0.0))


def _served_answer(answer):
    """The act package's own verdict in this file's shape: only None allows; anything else is a refusal."""
    inner = answer.get("refused") if isinstance(answer, dict) else None
    inner = inner if isinstance(inner, dict) else answer
    if isinstance(inner, dict) and "check" in inner:
        return _refusal(inner.get("check"), inner.get("why"), inner.get("hint"))
    return None if answer is None else _refusal("forbidden", str(answer), "choose another spot")


def bind(ex):
    """The rules bound to one executor, as a dict: keep_off(a_mm, b_mm=None, own_unc_mm=0.0), felt(point_mm,
    own_unc_mm=0.0), target_ok(entry_id, flags=None; flags may carry the caller's dry check as in_reach),
    first_force(part_name=None), bounds(steps, session=None, step_seconds=None, reach_mm=None), roles(),
    forbidden(), keep_off_from() ("act" | "guard" | "none"), has_forbidden_now(), has_forbidden and tool_mm (both
    read once, at bind, with no sensor reading), and the pure rules. Each function reads the executor at its call."""
    unknown = _refusal("forbidden_unplaced", "the manual (robot://devices) could not be read: the robot cannot "
                       "tell this spot from a part it must not touch", "make the move with your own call")

    def served():
        act = _state(ex, "act")
        where, at = act.get("forbidden_where"), act.get("forbidden_at")
        return (where, at) if callable(where) and callable(at) else (None, None)

    def keep_off_fn(a_mm, b_mm=None, own_unc_mm=0.0):
        known, parts = _manual(ex)
        if not known:
            return dict(unknown)
        where, a, b = served()[0], _point(a_mm), _point(b_mm)
        if where is not None and a is not None and (b_mm is None or b is not None):
            info = {"id": None, "kind": "located", "part": {"name": "", "role": "control"}, "label": "", "P": a,
                    "uncertain_mm": max(0.0, _num(own_unc_mm, 0.0)), "ray": None, "origin": None, "normal": None}
            try:
                return _served_answer(where(ex, info, None if b_mm is None else (a, b)))
            except Exception:
                pass                       # the served rule failed: the restated one answers
        return keep_off(_forbidden(ex, parts), a_mm, b_mm, own_unc_mm, _tool_mm(ex))

    def felt_fn(point_mm, own_unc_mm=0.0):
        known, parts = _manual(ex)
        if not known:
            return dict(unknown)
        at, point = served()[1], _point(point_mm)
        if at is not None and point is not None:
            try:
                hit = at(ex, point, max(0.0, _num(own_unc_mm, 0.0)))
                if hit is None:
                    return None
                name, eid, d, keep = hit
                said = _dict(sheet_words.label_part(name, parts)).get("forbidding") or f"do not touch the {name}"
                return _refusal("forbidden", f'the felt point is {float(d):.0f} mm from the {name} ({eid}): the '
                                f'manual says "{said}"', f"choose a spot over {float(keep):.0f} mm from it")
            except Exception:
                pass                       # the served rule failed: the restated one answers
        return felt_in_keep_off(_forbidden(ex, parts), point_mm, own_unc_mm, _tool_mm(ex))

    def roles_fn():
        out = {}
        for p in _manual(ex)[1]:
            name, role = p.get("name"), p.get("role")
            if name not in out or ROLE_RANK.get(role, 3) > ROLE_RANK.get(out[name], 3):
                out[name] = role
        return out

    def target_ok_fn(entry_id, flags=None):
        try:
            e = ex.world.get(entry_id)
        except Exception:
            e = None
        if not isinstance(e, dict):
            return "no entry"
        if e.get("kind") != "located":
            return "not a located entry"
        try:
            part, data = sheet_words.label_part(e.get("label"), _manual(ex)[1]), _dict(e.get("data"))
            entry = {"id": e.get("id"), "label": part["name"] if part is not None else e.get("label"),
                     "stale": e.get("stale"), "epoch_ok": _epoch_ok(ex, e), "in_reach": _dict(flags).get("in_reach"),
                     "identity": data.get("identity"), "part_check": data.get("part_check")}
            return target_ok(entry, flags, roles_fn())
        except Exception:
            return "names no part"

    def first_force_fn(part_name=None, part=None):          # part: the same argument under its other name
        try:
            name = part_name if part_name is not None else part
            forces = _dict(sheet_words.label_part(name, _manual(ex)[1])).get("forces")
        except Exception:
            forces = None
        noise = _num(_state(ex, "act").get("force_noise_n"))
        return first_force(noise if noise is not None and noise > 0.0 else L.NOISE_DEFAULT_N, forces)

    def bounds_fn(steps, session=None, step_seconds=None, reach_mm=None):
        try:                                        # a missing key: the session has no budget of that kind
            left = session.remaining() if callable(getattr(session, "remaining", None)) else {}
            calls_left, seconds_left = left.get("calls_left"), left.get("sim_seconds_left")
        except Exception:
            calls_left, seconds_left = 0, 0.0       # a budget that cannot be read is a spent one
        return Bounds(steps, calls_left, seconds_left, _reach_mm(ex, reach_mm), step_seconds)

    def forbids():                     # a manual that cannot be read counts as one that forbids a part
        return not _manual(ex)[0] or any(p.get("role") == "forbidden" for p in _manual(ex)[1])

    return {"keep_off": keep_off_fn, "felt": felt_fn, "target_ok": target_ok_fn, "first_force": first_force_fn,
            "bounds": bounds_fn, "roles": roles_fn, "forbidden": lambda: _forbidden(ex, _manual(ex)[1]),
            "keep_off_from": lambda: "none" if not _manual(ex)[0] else "act" if served()[0] is not None else "guard",
            "has_forbidden": forbids(), "has_forbidden_now": forbids, "tool_mm": _tool_mm(ex, sensed=False),
            "safe_call": safe_call, "skip_reason": skip_reason, "list_stale": list_stale, "charge": charge,
            "Bounds": Bounds}


__all__ = ["UNSAFE_ARGS", "safe_call", "target_ok", "keep_off", "felt_in_keep_off", "first_force", "skip_reason",
           "list_stale", "Bounds", "charge", "bind"]
