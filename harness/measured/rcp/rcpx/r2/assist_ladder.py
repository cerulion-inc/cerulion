# SPDX-License-Identifier: AGPL-3.0-only
"""The ladder registry and the table of ready calls (package assist).

THE LADDER. Help comes in rungs per skill (seeing, acting, checking); every rung is a real tool. A row is
{skill, rung, tool, line (at most 100 characters), needs [tool or state names]}. build(ex, session) reads each
loaded package's own export LADDER first, then this file's table of known names, and keeps the rows whose tool
the session can run; available(row, ex, session) says whether a row's needs hold now and, when not, why in a few
words. Nothing here assumes a tool is loaded: a name that is not in session.tools gives no row.

READY CALLS. Every call the robot writes (an offer's next, a new tool's next, a claim answer's next) is written
for THIS client and kept in one numbered table per session, ex.state["assist"]["ready"]:
    ready[N] = {tool, args, from, call, epoch, look, needs, taken}
- written {tool, args} when the client was sent the tool's name in its last list answer; else, when it was sent
  assist, {"tool": "assist", "args": {"take": N}, "runs": "TOOL ARGS" cut to 60 characters} (a choose with a
  number is {"pick": n}); else, when it was sent use, a use call; else nothing is written.
- tested when it is written AND when it is taken: no argument that raises a force (firm, further, force_n,
  max_n, a number in newtons); a contact the robot would make is written only with the guard loaded and only on
  an entry the guard's target rule passes; a call whose scene changed since (the epoch rose, its list or look is
  stale, its target no longer passes) is refused stale_offer at take time, with a fresh seeing call as the hint.
No robot, device or part is named here; the part words come from the manual at run time.
"""
import json
import re

from rcpx.context import refusal

HELP = ("candidates", "find", "choose", "seek", "act_on", "try_each", "changed")   # run by name, not in the start list
FEW = ("look", "candidates", "find", "choose", "seek", "act_on", "push", "lift", "try_each", "changed", "compare",
       "retreat", "park", "assist")                                                # the list after assist set few
UNSAFE_ARGS = ("firm", "further", "force_n", "max_n")
TARGET_TOOLS = ("touch", "press", "push", "lift", "approach")      # an act call that names an entry in target
CONTACT_TOOLS = ("touch", "press", "push", "lift", "act_on", "try_each")
RUNS_MAX = 60
LINE_MAX = 100
UNTIL_WORDS = ("cue changes", "cue WORD", "seen changes", "part moves", "part stays")
_NEWTONS = re.compile(r"\d\s*N\b")

# skill, rung, tool, line, needs. needs: a tool name (it must be loaded), "eyes:words" (the vision equipment finds
# by words), "list" (a fresh numbered list exists)
KNOWN = (
    ("seeing", 0, "look", "take a picture and read it yourself", ()),
    ("seeing", 0, "crop", "a closer cut of a picture you already have", ()),
    ("seeing", 0, "mark", "name a pixel in a look; the same label in two looks gives a point", ()),
    ("seeing", 0, "triangulate", "the point in space of a label marked in two looks", ("mark",)),
    ("seeing", 1, "locate", "measure a point from one pixel: the robot takes the other views", ()),
    ("seeing", 1, "view", "move the camera to another side of a thing", ()),
    ("seeing", 2, "candidates", "the robot numbers what it sees in a look; you pick a number, not a pixel", ()),
    ("seeing", 2, "choose", "turn numbered candidate N (or your own pixel) into a located point", ("list",)),
    ("seeing", 3, "find", "the robot's vision model finds what your own words name and numbers it", ("eyes:words",)),
    ("seeing", 4, "seek", "the robot looks from the sides no picture shows and lists what it finds there", ()),
    ("seeing", 4, "survey", "list the raised things on the table from several views (slow)", ()),
    ("acting", 1, "approach", "one checked move that brings the tip in front of a point", ()),
    ("acting", 1, "touch", "feel for a surface along a direction; stops at the first contact", ()),
    ("acting", 1, "press", "push on a part with a force you choose", ()),
    ("acting", 2, "push", "the whole step on a located part: reach, feel, push by evidence, compare", ()),
    ("acting", 2, "lift", "the whole step for a part that is raised from under its edge", ()),
    ("acting", 2, "act_on", "from a number or a label to a checked result in one call", ()),
    ("acting", 3, "try_each", "the robot tries each numbered candidate until your stop condition; reports each try",
     ("list",)),
    ("checking", 1, "verify", "state what a look shows; the robot checks when the look was taken", ()),
    ("checking", 2, "compare", "the robot compares two looks taken from the same place", ()),
    ("checking", 2, "changed", "the robot looks again from the view before the last act and says what changed", ()),
)
EXAMPLE = {"choose": {"n": 1}, "try_each": {"until": "cue changes"}, "find": {"words": "WORDS"}}


# ---------------------------------------------------------------- the ladder
def _state(ex):
    st = getattr(ex, "state", None)
    return st.get("assist") if isinstance(st, dict) else None


def _eyes(ex):
    st = getattr(ex, "state", None)
    eyes = st.get("eyes") if isinstance(st, dict) else None
    return eyes if isinstance(eyes, dict) else None


def has_list(ex):
    """A fresh numbered list exists (the eyes package keeps the lists and says when one is stale)."""
    eyes = _eyes(ex)
    look = eyes.get("newest") if eyes else None
    if look is None or look not in (eyes.get("lists") or {}):
        return False
    stale = eyes.get("stale")
    try:
        return not (callable(stale) and stale(look))
    except Exception:
        return False


def _tools(session):
    tools = getattr(session, "tools", None)
    return set(tools) if isinstance(tools, dict) else set(tools or ())


def _rung(value):
    """A rung as a number: 2, "2" and "S2" all read 2 (a package may write its rung with the skill's letter)."""
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    digits = "".join(ch for ch in str(value or "") if ch.isdigit())
    return int(digits) if digits else 0


def build(ex, session):
    """The rows whose tool this session can run, a package's own LADDER export first, lowest rung first."""
    have, rows, seen = _tools(session), [], set()
    for mod in getattr(ex, "mods", ()) or ():
        for row in getattr(mod, "LADDER", None) or ():
            if isinstance(row, dict) and row.get("tool") in have and row["tool"] not in seen:
                seen.add(row["tool"])
                rows.append({"skill": str(row.get("skill")), "rung": _rung(row.get("rung")), "tool": row["tool"],
                             "line": str(row.get("line") or "")[:LINE_MAX], "needs": list(row.get("needs") or ())})
    for skill, rung, tool, line, needs in KNOWN:
        if tool in have and tool not in seen:
            seen.add(tool)
            rows.append({"skill": skill, "rung": rung, "tool": tool, "line": line[:LINE_MAX], "needs": list(needs)})
    order = {"seeing": 0, "acting": 1, "checking": 2}
    rows.sort(key=lambda r: (order.get(r["skill"], 3), r["rung"]))      # stable: table order within a rung
    return rows


def available(row, ex, session):
    """(True, None), or (False, the reason in 6 words or fewer)."""
    have = _tools(session)
    if row.get("tool") not in have:
        return False, f"{row.get('tool')} is not loaded"
    for need in row.get("needs") or ():
        if need == "eyes:words":
            if (_eyes(ex) or {}).get("level") != "words":
                return False, "needs a vision model with words"
        elif need == "list":
            if not has_list(ex):
                return False, "needs a numbered list first"
        elif need not in have:
            return False, f"needs {need}"
    return True, None


def call_words(tool, args, listed):
    """How this client says the call, as text: by its name when it was sent the name, else through assist run,
    else through use; None when it has none of the three."""
    args = args or {}
    if tool in listed:
        return f"{tool} {json.dumps(args)}"
    if "assist" in listed:
        return "assist " + json.dumps({"run": tool, **({"args": args} if args else {})})
    if "use" in listed:
        return "use " + json.dumps({"name": tool, "args": args})
    return None


def index(ex, session, listed):
    """{skill: [{rung, tool, line, call} | {rung, tool, off}]}: what assist {} and robot://assist show."""
    out = {}
    for row in build(ex, session):
        ok, why = available(row, ex, session)
        item = {"rung": row["rung"], "tool": row["tool"]}
        if ok:
            item["call"] = call_words(row["tool"], EXAMPLE.get(row["tool"], {}), listed)
            item["line"] = row["line"]
        else:
            item["off"] = why
        out.setdefault(row["skill"], []).append(item)
    return out


# ---------------------------------------------------------------- how a call is written, and whether it may be
def unsafe(tool, args):
    """Rule 1 of what the robot may write: the reason a call is not allowed, else None (restated here so that a
    session with no guard file still never writes a force)."""
    def walk(v):
        if isinstance(v, dict):
            for k, x in v.items():
                if k in UNSAFE_ARGS:
                    return f"it holds {k}"
                bad = walk(x)
                if bad:
                    return bad
        elif isinstance(v, (list, tuple)):
            for x in v:
                bad = walk(x)
                if bad:
                    return bad
        elif isinstance(v, str) and _NEWTONS.search(v):
            return "it holds a force in newtons"
        return None
    return walk(args if isinstance(args, dict) else {})


def not_safe(ex, tool, args):
    st = _state(ex) or {}
    guard = st.get("guard")
    bad = unsafe(tool, args)
    if bad is None and isinstance(guard, dict) and callable(guard.get("safe_call")):
        try:
            bad = guard["safe_call"](tool, args)
        except Exception:
            bad = "the guard failed on it"
    return bad


def target_of(tool, args):
    """The located or contact entry an act call names, or None."""
    args = args or {}
    t = args.get("target") if tool in TARGET_TOOLS else args.get("on") if tool == "act_on" else None
    return t if isinstance(t, str) and t.startswith(("located_", "contact_")) else None


def is_contact(tool, args):
    """A call that makes (or can make) a contact: an act, or a choose that feels along its line."""
    args = args or {}
    return tool in CONTACT_TOOLS or (tool == "choose" and args.get("depth") == "touch")


def reads_list(tool, args):
    """A call that reads the numbered list: its numbers mean nothing once the list is stale."""
    args = args or {}
    if tool == "try_each":
        return True
    if tool == "choose":
        return "n" in args and "px" not in args
    if tool == "act_on":
        return isinstance(args.get("on"), int) and not isinstance(args.get("on"), bool)
    if tool == "seek":
        return isinstance(args.get("around"), int) and not isinstance(args.get("around"), bool)
    return False


def render(tool, args, listed, n, needs=None, offers=None):
    """The next object of a call for this client (module text), or None. offers: the offer engine when it is
    loaded (its call_text is the one renderer); without it this file's own copy of the rule."""
    args = dict(args or {})
    if unsafe(tool, args):
        return None
    fn = getattr(offers, "call_text", None)
    if callable(fn):
        try:
            return fn(tool, args, listed, n, needs)
        except TypeError:                      # an engine whose call_text takes no needs
            out = fn(tool, args, listed, n)
            if out is not None and needs:
                out = dict(out, needs=needs)
            return out
    if tool in listed:
        out = {"tool": tool, "args": args}
    elif "assist" in listed:
        if tool == "choose" and isinstance(args.get("n"), int) and "px" not in args:
            out = {"tool": "assist", "args": {"pick": args["n"]}}
        else:
            take = {"take": n}
            if "until" in need_keys(needs):
                take["until"] = "WORDS"
            out = {"tool": "assist", "args": take, "runs": (tool + " " + json.dumps(args))[:RUNS_MAX]}
    elif "use" in listed:
        out = {"tool": "use", "args": {"name": tool, "args": args}}
    else:
        return None
    if needs:
        out["needs"] = needs
    return out


def need_keys(needs):
    """The names a call still needs from the model: "until", ["until", "candidates"], or phrases such as
    "until: your stop condition; candidates: the numbers you choose" all read as their leading words."""
    items = needs if isinstance(needs, (list, tuple)) else str(needs).split(";") if needs else []
    return [str(i).split(":", 1)[0].strip() for i in items if str(i).strip()]


def _look_of(ex, tool, args):
    look = (args or {}).get("look")
    if isinstance(look, str):
        return look
    if reads_list(tool, args):
        return (_eyes(ex) or {}).get("newest")
    return None


def ready_call(ex, tool, args, source, needs=None):
    """Write one call for this client and register it; None for a call that may not be written (it holds a force
    argument; it is a contact and no guard is loaded; this client can say it in no way)."""
    st = _state(ex)
    if st is None:
        return None
    args = dict(args or {})
    listed = st.get("listed") or set()
    if not_safe(ex, tool, args):
        return None
    if is_contact(tool, args) and st.get("guard") is None:
        return None                              # with no guard the robot writes no act call at all
    if tool == "assist":                         # a call of assist itself (pick with the number left open)
        return render(tool, args, listed, None, needs, st.get("offers"))
    if tool == "choose" and needs and "n" not in args and "px" not in args:
        # the number is left to the model: nothing to take, pick N is the flat form
        if "choose" in listed:
            return {"tool": "choose", "args": args, "needs": needs}
        if "assist" in listed:
            return {"tool": "assist", "args": {"pick": "N"}, "needs": "N: the number you choose"}
        if "use" in listed:
            return {"tool": "use", "args": {"name": "choose", "args": args}, "needs": needs}
        return None
    key = json.dumps([tool, args, needs], sort_keys=True, default=str)
    ready = st["ready"]
    n = next((k for k, e in ready.items() if e.get("key") == key and e["epoch"] == getattr(ex, "epoch", 0)
              and e.get("taken") is None), None)
    new = n is None
    if new:
        n = len(ready) + 1
    out = render(tool, args, listed, n, needs, st.get("offers"))
    if out is None:
        return None
    if new:
        ready[n] = {"tool": tool, "args": args, "from": str(source), "call": int(st.get("calls") or 0),
                    "epoch": getattr(ex, "epoch", 0), "look": _look_of(ex, tool, args), "needs": needs,
                    "taken": None, "key": key}
    return out


def open_ready(st, limit=4):
    """The newest open ready calls, for robot://assist and the index: [{n, tool, args, needs?}]."""
    rows = [{"n": n, "tool": e["tool"], "args": e["args"], **({"needs": e["needs"]} if e.get("needs") else {})}
            for n, e in st.get("ready", {}).items() if e.get("taken") is None]
    return rows[-int(limit):]


# ---------------------------------------------------------------- the test at take time
def retest(ex, session, entry):
    """None when ready call `entry` may run now, else why not (the scene changed, the call is not safe any more)."""
    st = _state(ex) or {}
    tool, args = entry["tool"], entry.get("args") or {}
    bad = not_safe(ex, tool, args)
    if bad:
        return bad
    if tool not in _tools(session):
        return f"{tool} is not loaded any more"
    if entry.get("epoch") != getattr(ex, "epoch", 0):
        return "the scene changed since it was written (the epoch rose)"
    eyes = _eyes(ex)
    if reads_list(tool, args):
        look = entry.get("look")
        if eyes is None or look is None:
            return "its numbered list is gone"
        if eyes.get("newest") != look and "look" not in args:
            return f"a newer list than the one of {look} exists: its numbers changed"
        stale = eyes.get("stale")
        try:
            why = stale(look) if callable(stale) else None
        except Exception:
            why = "the list could not be tested"
        if why:
            return f"the list of {look} is stale ({why})"
    look = args.get("look")
    if isinstance(look, str):
        looks = getattr(ex, "_looks", None) or {}
        rec = looks.get(look)
        if rec is None or rec.get("can_see") == "dark":
            return f"look {look} cannot be used any more"
    guard = st.get("guard")
    target = target_of(tool, args)
    if target is not None:
        world = getattr(ex, "world", None)
        e = getattr(world, "entries", {}).get(target) if world is not None else None
        if e is None:
            return f"{target} is gone"
        if guard is None:
            return "no guard is loaded: the robot runs no contact it wrote"
        gauge = st.get("gauge")
        flags = dict((getattr(gauge, "flags", None) or {}).get(target) or {})
        try:
            why = guard["target_ok"](target, flags)
        except Exception:
            why = "the guard failed on it"
        if why:
            return f"{target} no longer passes ({why})"
    elif is_contact(tool, args) and guard is None:
        return "no guard is loaded: the robot runs no contact it wrote"
    return None


def _integer(v):
    if isinstance(v, int) and not isinstance(v, bool):
        return v
    return int(v) if isinstance(v, str) and v.strip().isdigit() else None


def plan(ex, session, args):
    """What an assist call asks: ("index",), ("set", word), ("refuse", refusal), ("stale", n, why) or
    ("direct", tool, args, how, n)."""
    st = _state(ex) or {}
    have = _tools(session)
    ready = st.get("ready") or {}
    if args.get("take") is not None:
        n = _integer(args["take"])
        entry = ready.get(n)
        if entry is None:
            opened = ", ".join(str(k) for k, e in ready.items() if e.get("taken") is None) or "none"
            return ("refuse", refusal("unknown_ready", f"no ready call {args['take']!r} in this session",
                                      f"open ready calls: {opened}; assist {{}} lists the help"))
        why = retest(ex, session, entry)
        if why:
            return ("stale", n, why)
        targs = dict(entry["args"])
        needs = need_keys(entry.get("needs"))
        tools = getattr(session, "tools", None)
        own = tools.get(entry["tool"]) if isinstance(tools, dict) else None
        props = ((own or {}).get("args") or {}).get("properties") or {}
        if isinstance(args.get("until"), str) and ("until" in needs or "until" in props):
            targs["until"] = args["until"]
        more = args.get("args") if isinstance(args.get("args"), dict) else {}
        for key in needs:                # what the call asked the model to complete, and nothing else
            if key != "until" and key in more:
                targs[key] = more[key]
        if "until" in needs and "until" not in targs:
            return ("refuse", refusal("needs_until", f"ready call {n} ({entry['tool']}) needs your stop condition",
                                      f"send assist {json.dumps({'take': n, 'until': 'part stays'})} with your "
                                      "own words: " + " | ".join(UNTIL_WORDS)))
        if not_safe(ex, entry["tool"], targs):
            return ("stale", n, "it would hold a force argument")
        return ("direct", entry["tool"], targs, "take", n)
    if args.get("pick") is not None:
        n = _integer(args["pick"])
        if "choose" not in have or n is None:
            return ("refuse", refusal("not_built" if n is not None else "args",
                                      "no numbered list can be picked from here" if n is not None else
                                      "pick takes a candidate's number",
                                      "numbers come from a list: "
                                      + (call_words("candidates", {}, st.get("listed") or set()) or "look {}")))
        targs = {"n": n}
        part = args.get("part") if isinstance(args.get("part"), str) else None
        if part is None:                 # the row's own word when that is a control part; never a part chosen here
            eyes = ex.state.get("eyes") if isinstance(ex.state.get("eyes"), dict) else {}
            rows = ((eyes.get("lists") or {}).get(eyes.get("newest")) or {}).get("items") or []
            row = next((r for r in rows if isinstance(r, dict) and r.get("n") == n), None)
            if row is not None and row.get("role") == "control" and row.get("found_by"):
                part = row["found_by"]
        if part is not None:
            targs["part"] = part
        return ("direct", "choose", targs, "pick", n)
    if args.get("run") is not None:
        valid = [t for t in HELP if t in have]
        if args["run"] not in valid:
            return ("refuse", refusal("unknown_tool", f"assist runs no tool named {str(args['run'])[:40]!r}",
                                      "help tools here: " + (", ".join(valid) or "none is loaded")))
        return ("direct", args["run"], dict(args["args"]) if isinstance(args.get("args"), dict) else {}, "run", None)
    if args.get("set") is not None:
        return ("set", args["set"])
    return ("index",)


def fresh_seeing(session):
    """(tool, args) of a fresh seeing call for a stale_offer hint: the numbered list when loaded, else a look."""
    return ("candidates", {}) if "candidates" in _tools(session) else ("look", {})


def stale_refusal(ex, session, n, why):
    st = _state(ex) or {}
    tool, args = fresh_seeing(session)
    words = call_words(tool, args, st.get("listed") or set()) or f"{tool} {json.dumps(args)}"
    return refusal("stale_offer", f"ready call {n} was not run: {why}",
                   f"nothing moved; see the scene as it is now first: {words}")


# ---------------------------------------------------------------- what the offer engine and the claim answers read
def newest_look(ex):
    """The newest look that can see (not dark), or None."""
    looks = getattr(ex, "_looks", None) or {}
    for key in reversed(list(looks)):
        if (looks[key] or {}).get("can_see") != "dark":
            return key
    return None


def _in_candidate(ex, st, point):
    """True | False | None: the operator's last pointed pixel lies in one of the robot's own candidates. From a
    fresh list of that look when there is one, else from the off-path finding of that pointing act."""
    if not isinstance(point, dict) or point.get("px") is None:
        return None
    eyes = _eyes(ex) or {}
    look = point.get("look")
    at = eyes.get("at")
    if callable(at) and look in (eyes.get("lists") or {}):
        try:
            stale = eyes.get("stale")
            if not (callable(stale) and stale(look)):
                return at(look, point["px"][0], point["px"][1]) is not None
        except Exception:
            pass
    for f in reversed((st.get("sense") or {}).get("findings") or []):
        if f.get("call") == point.get("call") and f.get("verdict") in ("in", "out"):
            return f["verdict"] == "in"
    return None


def _watch(ex, kinds):
    """The row (a number) or the located entry (an id) of the manual's cue part, for a routine to watch."""
    cues = set(kinds.get("cue") or ())
    if not cues:
        return None
    eyes = _eyes(ex) or {}
    lst = (eyes.get("lists") or {}).get(eyes.get("newest")) if has_list(ex) else None
    for row in (lst or {}).get("items") or ():
        if row.get("found_by") in cues:
            return row.get("n")
    world = getattr(ex, "world", None)
    for e in reversed(list(getattr(world, "entries", {}).values())) if world is not None else ():
        if e.get("kind") == "located" and not e.get("stale") and str(e.get("label")).strip().lower() in cues:
            return e["id"]
    return None


def context(ex, session):
    """The ctx dict of the offer engine and of its untried rule, from what the session holds now."""
    from rcpx.r2 import assist_check
    st = _state(ex) or {}
    gauge = st.get("gauge")
    eyes = _eyes(ex) or {}
    kinds = assist_check.manual(ex)["kinds"]
    rem = session.remaining() if callable(getattr(session, "remaining", None)) else {}
    fresh = has_list(ex)
    lst = (eyes.get("lists") or {}).get(eyes.get("newest")) if fresh else None
    point = getattr(gauge, "last_point", None)
    return {"call": int(st.get("calls") or 0), "calls_left": (rem or {}).get("calls_left"),
            "eyes_level": eyes.get("level"), "last_point": point,
            "pixel_in_candidate": _in_candidate(ex, st, point), "newest_look": newest_look(ex),
            "located": assist_check.located_rows(ex, st.get("guard"), getattr(gauge, "flags", None)),
            "has_list": fresh, "list_taken": bool(st.get("list_taken")),
            "unseen": list((lst or {}).get("unseen") or []) or None,
            "parts": {k: list(v) for k, v in kinds.items()}, "watch": _watch(ex, kinds), "untried": [],
            "next_n": len(st.get("ready") or {}) + 1, "paused": bool(st.get("paused")),
            "picks": list(st.get("picks") or [])}


def along_needs(ex, eid):
    """What an approach the robot writes leaves to the operator: the way onto the part, with the manual's own
    sentence for the part the entry is labelled with (the robot knows where the point is, not how the part is
    worked; a direction it filled in could be one the manual forbids)."""
    from rcpx import sheet_words
    from rcpx.r2 import assist_check
    e = ex.world.get(eid) if getattr(ex, "world", None) is not None and eid is not None else None
    try:
        part = sheet_words.label_part((e or {}).get("label"), assist_check.manual(ex)["plist"])
    except Exception:
        part = None
    quote = " ".join(str((part or {}).get("direction") or "").split())[:110]
    return ("along: your direction onto the part (forward, back, left, right, up, down or [x, y, z])"
            + (f"; the manual: {quote}" if quote else ""))


def untried(ex, session, ctx=None):
    """At most 3 calls not tried yet, the seeing rung first (the offer engine's rule, 4.4), each one a call the
    session can run now and the robot may write: no force argument, a contact only with the guard loaded and only
    on an entry that passes it. [] when the engine is not loaded."""
    st = _state(ex) or {}
    fn = getattr(st.get("offers"), "untried", None)
    gauge = st.get("gauge")
    if not callable(fn) or gauge is None:
        return []
    ctx = ctx if ctx is not None else context(ex, session)
    have = _tools(session)
    snap = gauge.snap() if callable(getattr(gauge, "snap", None)) else {}
    out = []
    for it in fn(getattr(gauge, "stage", "no_point"), snap, have, ctx) or ():
        tool, args = it.get("tool"), dict(it.get("args") or {})
        if tool not in have or not_safe(ex, tool, args):
            continue
        if is_contact(tool, args) and st.get("guard") is None:
            continue
        target = target_of(tool, args) or (args.get("target") if tool == "approach" else None)
        if target is not None and not any(r["id"] == target and r["target_ok"] is None for r in ctx["located"]):
            continue
        if tool == "approach" and "along" not in args:       # the engine names the target; the way is the operator's
            it = dict(it, needs=along_needs(ex, target))
        out.append({"tool": tool, "args": args, **({"needs": it["needs"]} if it.get("needs") else {})})
    return out[:3]


# ---------------------------------------------------------------- the list
def listed_entry(entry):
    """One tool entry in the shape of a session's list answer."""
    from rcpx import surfaces
    args = entry["example"]["args"]
    return {"name": entry["name"], "description": entry["description"], "args": surfaces.public_schema(entry["args"]),
            "example": {"args": args, "means": entry["example"]["means"],
                        "cli": f"call {entry['name']} '{json.dumps(args)}'"}}


__all__ = ["FEW", "HELP", "KNOWN", "UNSAFE_ARGS", "UNTIL_WORDS", "available", "build", "call_words", "context",
           "fresh_seeing", "has_list", "index", "is_contact", "listed_entry", "need_keys", "newest_look", "along_needs", "not_safe",
           "open_ready", "plan", "reads_list", "ready_call", "render", "retest", "stale_refusal", "target_of",
           "unsafe", "untried"]
