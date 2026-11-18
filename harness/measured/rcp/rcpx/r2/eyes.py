# SPDX-License-Identifier: AGPL-3.0-only
"""Package eyes (SPEC-R5 4.1, 7.1, 7.4, 9.3): the robot does the pointing. It numbers the things in a picture and
the operator picks a number (candidates, choose), or it looks for a part by words with a small vision model that
is robot equipment (find). With no vision model declared the same tools answer from colour regions, geometry and
touch. No tool here runs unless it is called, and nothing here chooses which part is the target.

Tools (each result leads with said and ends with next; a brief result is at most 900 bytes besides its picture):
- candidates(look, words, page, detail): the numbered list of a look. At level words the default list asks the
  vision model for EVERY part name the manual (robot://devices) gives, whatever the part's role, keeps each
  part's best three, interleaved by rank (control parts, then cue parts, then forbidden parts), then fills with
  the model's regions; every row says which word found it (found_by), that part's role, its colour, what the
  manual says about that colour, and keep_off (the manual's own sentence) when the row was found by a forbidden
  part's word or has a forbidden part's colour. The operator's own labels are never sent to the vision model:
  only the manual's part names and the words of this very call.
- find(words, look, max, detail): the vision model's matches for the operator's own words, with one answer word
  (found, unsure, none) from the equipment's own threshold and floor; never "checked by touch".
- choose(n, part, look, px, depth, detail): rcpx/r2/eyes_choose.py (a primitive: it may move the arm).

The numbered picture is the look's own picture (rcpx/r2/eyes_draw.py): never turned, scaled or cut.

Equipment (7.1): the level (absent | segments | words) and the service address come from the installation
(ex.context["installation"]["vision"] {level, url}), else from the environment name RCPX_EYES_URL (the level then
reads words until the first call's /health answer corrects it), else the level is absent. There is no default
address. setup makes no service request and reads no picture; robot://body gains one key, equipment.vision, whose
value is the level word. A service that does not answer within 3 s (7.4) makes that call answer from colour and
geometry with a note, and the service is not asked again for the next 10 calls of this package.

Lists go stale with the scene (4.7 rule 6): a list is bound to its look, to the epoch and to the call it was
made on. The one hook of this package is a post hook that reads result FIELDS only (base_moved, device_moved,
moved_whole, stopped sliding or unconfirmed; and a look's id for its place in the order of calls).

ex.state["eyes"] (9.3): level, service {url, ok, seconds_last, fallbacks}, lists {look: {source, words, items,
unseen, epoch, call}}, newest, parts {control, cue, forbidden, fixed, colours, forbidding, roles}, and the
functions stale(look_id=None), propose(look_id, max=8, words=None), list(look_id=None, words=None, page=1,
for_routine=False, detail="brief"), at(look_id, u, v), find_at(look_id, u, v, part), choose(...). A caller that
reached the list through a routine sets ex.state["eyes"]["for_routine"] = True (read once) or passes for_routine.

Without the assist package this package keeps its three tools out of the session's listing itself (they still
run by name); with it loaded, that package's FILTER does (SPEC-R5 6.2). Depends on nothing: selfsee (the own body
in a look), sight (survey boxes, locate), assist (ready calls, the guard) are read lazily from ex.state and each
is worked without, with a note. Nothing here reads simulator ground truth or names a robot, a device or a model.
"""

import json
import math
import os
import time

from rcpx import colour_words, sheet_words
from rcpx import surfaces as S
from rcpx.context import refusal
from rcpx.control_base import primitive
from rcpx.r2 import eyes_client as C
from rcpx.r2 import eyes_draw as D
from rcpx.r2 import eyes_local as LOC
from rcpx.r2 import eyes_parts as P

ID = "eyes"
ENV_URL = "RCPX_EYES_URL"
LEVELS = C.LEVELS
BRIEF_MAX = 900
SAID_WORDS = 45
OWN_TOOLS = ("candidates", "find", "choose")
NOT_TOUCHED = "not checked by touch"


# ---------------------------------------------------------------- the declaration and the state
def _declared(ex):
    """(level, url) as the installation declares the equipment; no request is made."""
    inst = (getattr(ex, "context", None) or {}).get("installation")
    vis = inst.get("vision") if isinstance(inst, dict) else None
    if isinstance(vis, dict):
        url = vis.get("url") if isinstance(vis.get("url"), str) and vis.get("url").strip() else None
        level = vis.get("level") if vis.get("level") in LEVELS else ("words" if url else "absent")
        return (level if url or level == "absent" else "absent"), (url if level != "absent" else None), True
    url = os.environ.get(ENV_URL, "").strip() or None
    return ("words" if url else "absent"), url, False


def _doc(ex):
    d = (getattr(ex, "context", None) or {}).get("devices")
    if callable(d):
        try:
            d = d()
        except Exception:
            d = None
    return d if isinstance(d, dict) else {}


def _declare(ex, st):
    """Read the declaration and the manual again (the runner sets the context after the executor is built)."""
    level, url, by_installation = _declared(ex)
    svc = st.get("service")
    if url is None:
        st["service"] = None
    elif svc is None or svc.get("url") != url:
        st["service"] = C.new_service(url, by_installation)
    st["level"] = level
    try:
        st["parts"] = P.parts_from(sheet_words.parts(_doc(ex)))
    except Exception:
        st["parts"] = P.parts_from([])


def _flag(v):
    if isinstance(v, dict):
        return bool(v.get("moved")) or v.get("answer") == "yes"
    return bool(v)


def _post(ex, st, name, result):
    """The staleness hook: result fields only. Counts the operator's calls and keeps the ones that moved the
    scene; notes at which count and epoch a look was taken and where its frame file is."""
    st["seq"] += 1
    if not isinstance(result, dict) or "refused" in result:
        return None
    why = None
    if _flag(result.get("base_moved")):
        why = "the base moved"
    elif _flag(result.get("device_moved")):
        why = "the device moved"
    elif _flag(result.get("moved_whole")):
        why = "the thing moved as a whole"
    elif result.get("stopped") in ("sliding", "unconfirmed"):
        why = f"an act stopped {result['stopped']}"
    if why is not None:
        st["moved"].append({"seq": st["seq"], "tool": name, "why": why})
    look = result.get("look_id")
    if isinstance(look, str):
        st["look_seq"][look] = st["seq"]
        st["look_epoch"][look] = int(getattr(ex, "epoch", 0))
        if isinstance(result.get("frame"), str):
            st["frames"][look] = result["frame"]
    return None


def setup(ex):
    st = ex.state["eyes"] = {
        "level": "absent", "service": None, "lists": {}, "newest": None, "parts": P.parts_from([]),
        "moved": [], "seq": 0, "look_seq": {}, "look_epoch": {}, "frames": {}, "excl": {}, "calls": 0,
        "session": None, "hidden": []}
    _declare(ex, st)
    ex.on_event("context", lambda **_: _declare(ex, st))
    ex.add_post(lambda name, kwargs, result: _post(ex, st, name, result))
    st["stale"] = lambda look_id=None: stale(ex, look_id)
    st["propose"] = lambda look_id, max=8, words=None: propose(ex, look_id, max, words)
    st["list"] = lambda look_id=None, words=None, page=1, for_routine=False, detail="brief": list_(
        ex, look_id, words, page, for_routine=for_routine, detail=detail)
    st["at"] = lambda look_id, u, v: LOC.at(st, _key(ex, look_id), u, v)
    st["find_at"] = lambda look_id, u, v, part: LOC.find_at(ex, st, look_id, u, v, part)
    st["choose"] = lambda n=None, part=None, look=None, px=None, depth="auto", unconfirmed=False: ex.choose(
        n=n, part=part, look=look, px=px, depth=depth, unconfirmed=unconfirmed)


def _state(ex):
    return ex.state["eyes"]


def _journal(ex, kind, **fields):
    try:
        ex._journal(kind, **fields)
    except Exception:
        pass


def _said(text):
    return P.words_of(text, SAID_WORDS)


def _refuse(check, why, hint):
    return {"said": why, **refusal(check, why, hint)}


# ---------------------------------------------------------------- looks and staleness
def _newest_look(ex):
    """The newest look id that can see, else None."""
    for key in reversed(list(getattr(ex, "_looks", {}) or {})):
        if ex._looks[key].get("can_see") != "dark":
            return key
    return None


def _look_epoch(ex, st, look_id):
    if look_id in st["look_epoch"]:
        return st["look_epoch"][look_id]
    ep = ex.state.get("epochs")
    looks = ep.get("looks") if isinstance(ep, dict) else None
    known = looks.get(look_id) if isinstance(looks, dict) else None
    if isinstance(known, dict) and isinstance(known.get("epoch"), int):
        return known["epoch"]
    return known if isinstance(known, int) else int(getattr(ex, "epoch", 0))


def look_outdated(ex, look_id):
    """None, or why the look's picture no longer shows where things are: the epoch rose since it was taken, its
    record is marked stale, or an act after it moved the scene."""
    st = _state(ex)
    rec = (getattr(ex, "_looks", {}) or {}).get(look_id) or {}
    if rec.get("stale"):
        return f"look {look_id} is marked stale"
    if int(getattr(ex, "epoch", 0)) > _look_epoch(ex, st, look_id):
        return f"the robot's base or the world moved since {look_id} was taken (the epoch rose)"
    since = st["look_seq"].get(look_id)
    for m in st["moved"]:
        if since is not None and m["seq"] > since:
            return f"{m['why']} ({m['tool']}) after {look_id} was taken"
    return None


def stale(ex, look_id=None):
    """None when the list of the look (default: the newest list) is fresh; else the reason (SPEC-R5 4.7 rule 6)."""
    st = _state(ex)
    key = st["newest"] if look_id is None else _key(ex, look_id)
    lst = st["lists"].get(key)
    if lst is None:
        return "no list was made yet" if look_id is None else f"no list was made for {look_id}"
    if int(getattr(ex, "epoch", 0)) > int(lst["epoch"]):
        return f"the robot's base or the world moved since the list of {key} was made (the epoch rose)"
    for m in st["moved"]:
        if m["seq"] > lst["seq"]:
            return f"{m['why']} ({m['tool']}) since the list of {key} was made"
    return look_outdated(ex, key)


def _key(ex, look_id):
    key = str(look_id).strip()
    looks = getattr(ex, "_looks", {}) or {}
    return f"L{key}" if key not in looks and key.isdigit() else key


def fresh_hint(ex):
    """The call that makes a fresh list: candidates {}, after a new look when the newest look is outdated."""
    key = _newest_look(ex)
    if key is None or look_outdated(ex, key) is not None:
        return "look {} first, then candidates {}"
    return "candidates {}"


def _resolve(ex, look):
    """(look id, record) or a refusal: the given look, default the newest look that can see."""
    looks = getattr(ex, "_looks", {}) or {}
    if look is None or look == "":
        key = _newest_look(ex)
        if key is None:
            return _refuse("no_look", "no look of this session can see anything to number" if looks else
                           "there is no look yet to number things in", "look {} first, then candidates {}")
    else:
        key = _key(ex, look)
        if key not in looks:
            return _refuse("unknown_look", f"no look {str(look)[:20]!r} in this session",
                           "pass a look id a look returned, or leave look out for the newest")
    rec = looks[key]
    if rec.get("can_see") == "dark":
        return _refuse("can_see", f"look {key} is dark (brightness {float(rec.get('brightness') or 0):.1f} of 255): "
                       "nothing in it can be numbered", "look again once the light reads dim or ok")
    why = look_outdated(ex, key)
    if why is not None:
        return _refuse("stale_look", f"{why}: its picture no longer shows where things are",
                       "look {} first, then candidates {}")
    return key, rec


def _exclude(ex, st, look_id, rec):
    if look_id not in st["excl"]:
        st["excl"][look_id] = LOC.own_polygons(ex, rec["q"])
    return st["excl"][look_id]


# ---------------------------------------------------------------- the list
def _make(ex, st, key, rec, words=None, tool="candidates", only_words=False, n_max=None):
    """Make the list of a look and keep it: the record of ex.state["eyes"]["lists"][key], a refusal, or None (a
    find that the vision model did not answer). The rows are made in rcpx/r2/eyes_local.py make_rows."""
    t0 = time.time()
    st["calls"] += 1
    img = LOC.frame(rec)
    if img is None:
        return _refuse("unknown_look", f"the frame of {key} is not kept", "take a new look: look {}")
    excl, body_note = _exclude(ex, st, key, rec)
    lst = LOC.make_rows(ex, st, rec, img, excl, words, only_words, n_max)
    if lst is None:
        return None
    if body_note:
        lst["notes"].insert(0, body_note)
    session = st.get("session")
    lst.update(look=key, epoch=_look_epoch(ex, st, key), call=int(getattr(session, "calls_used", 0) or 0),
               seq=st["seq"], k=st["calls"], pictures={}, layouts={}, seconds=round(time.time() - t0, 3))
    st["lists"][key] = lst
    st["newest"] = key
    _journal(ex, "eyes_call", tool=tool, source=lst["source"], seconds=lst["seconds"], n=len(lst["items"]),
             look=key)
    return lst


def _has(ex, name):
    session = _state(ex).get("session")
    if session is not None:
        return name in (getattr(session, "tools", None) or {})
    return callable(getattr(ex, name, None))


def ready(ex, tool, args, needs=None):
    """A next object: through the assist package's ready_call when it is loaded (SPEC-R5 4.6), else {tool, args}."""
    a = ex.state.get("assist")
    fn = a.get("ready_call") if isinstance(a, dict) else None
    if callable(fn):
        try:
            out = fn(tool, args, "next", needs)
        except Exception:
            out = None
        if isinstance(out, dict):
            return out
    out = {"tool": tool, "args": args}
    if needs:
        out["needs"] = needs
    return out


def _routine(ex, st, flag):
    """True when the next of a list is a routine: the caller says so, or the operator's standing preference for
    acting is routine, or acting has an open event (read from the assist package when it is loaded)."""
    if flag or st.pop("for_routine", False):
        return True
    a = ex.state.get("assist")
    if not isinstance(a, dict):
        return False
    if (a.get("prefs") or {}).get("acting") == "routine":
        return True
    try:
        return int((a["gauge"].standing() or {}).get("acting") or 0) > 0
    except Exception:
        return False


def along_needs(ex, label):
    """What an approach the robot writes leaves to the operator: the way onto the part. The robot knows where the
    touched point is, not which way the part is worked: that is the manual's sentence, quoted, and the operator's
    choice (a direction the robot filled in could be one the manual forbids)."""
    part = sheet_words.label_part(label, sheet_words.parts(_doc(ex))) if label else None
    quote = " ".join(str((part or {}).get("direction") or "").split())[:110]
    return ("along: your direction onto the part (forward, back, left, right, up, down or [x, y, z])"
            + (f"; the manual: {quote}" if quote else ""))


def can_run_steps(ex):
    """True when a routine of whole steps can run here: try_each is loaded and so is a verb it runs (push or lift).
    Without one the routine answers not_built, so no next may name it."""
    return _has(ex, "try_each") and (_has(ex, "push") or _has(ex, "lift"))


def next_of(ex, lst, for_routine=False):
    """The next call of a list (SPEC-R5 4.1): never a number filled in for the operator."""
    st, parts = _state(ex), _state(ex)["parts"]
    if not lst["items"]:
        return ready(ex, "seek", {}) if _has(ex, "seek") else ready(ex, "look", {})
    if _routine(ex, st, for_routine) and can_run_steps(ex):
        args, needs = {}, []
        if parts.get("cue"):
            args["until"] = "cue changes"
        else:
            needs.append("until: your stop condition in two words")
        if parts.get("forbidden"):
            needs.append("candidates: the numbers you choose")
        return ready(ex, "try_each", args, "; ".join(needs) or None)
    # Integrator, 2026-11-12 (DECISIONS): the pick comes first whenever a row was found by a control part's word.
    # As first written the next was seek unless a control row stood at the equipment's threshold, which a vision
    # model that answers "unsure" on nearly every look never reaches: the chain then ran list, seek, a person.
    # seek stays the next only at level words, once per session, while no row at all was found by a control word.
    named = [r for r in lst["items"] if r.get("role") == "control"]
    if (st.get("level") == "words" and parts.get("control") and not named and lst["unseen"] and not st.get("sought")
            and _has(ex, "seek")):
        return ready(ex, "seek", {})
    if _has(ex, "assist"):
        return ready(ex, "assist", {"pick": "N"}, "N: the number you choose")
    return ready(ex, "choose", {"look": lst["look"]}, "n: the number you choose")


def _picture(ex, lst, rec, page, page_rows):
    if page not in lst["pictures"]:
        img = LOC.frame(rec)
        others = [r for r in lst["items"] if r not in page_rows]
        pic, layout = D.numbered(img, page_rows, others)
        lst["pictures"][page] = ex._save(pic, f"{lst['look']}_numbered_{lst['k']}_{page}")
        lst["layouts"][page] = layout
    return lst["pictures"][page], lst["layouts"][page]


def result_of(ex, lst, page=1, detail="brief", for_routine=False, head=None):
    """The answer of candidates, find and list for one page of a kept list (SPEC-R5 4.1)."""
    rows = lst["items"]
    pages = max(1, int(math.ceil(len(rows) / float(P.PAGE_ROWS))))
    if page > pages:
        return _refuse("page", f"the list of {lst['look']} has {len(rows)} numbers: page {page} is empty",
                       f"pages 1 to {pages}; candidates {{\"look\": \"{lst['look']}\"}} is page 1")
    page_rows = rows[(page - 1) * P.PAGE_ROWS:page * P.PAGE_ROWS]
    rec = ex._looks[lst["look"]]
    nxt = next_of(ex, lst, for_routine)
    saids = [head["said"]] if (head or {}).get("said") else [P.list_said(lst, page, page_rows, SAID_WORDS),
                                                             P.list_said(lst, page, page_rows, SAID_WORDS, True)]
    for k, said in enumerate(saids):        # a page that does not hold its rows whole gets the shorter sentence
        out = {"said": said}
        out.update({key: v for key, v in (head or {}).items() if key != "said"})
        out.update(look=lst["look"], frame=lst["look"], source=lst["source"], page=page, pages=pages, candidates=[])
        if lst["notes"]:
            out["notes"] = list(lst["notes"])[:3]
        room = BRIEF_MAX - len(json.dumps({**out, "next": nxt})) - 4
        brief, level = P.brief_rows(page_rows, room)
        if (len(brief) == len(page_rows) and level <= 3) or k == len(saids) - 1:
            break
    out["candidates"] = brief
    if len(brief) < len(page_rows):
        out["rows_left_out"] = len(page_rows) - len(brief)
    if detail == "full":
        if page_rows:
            _picture(ex, lst, rec, page, page_rows)
        out["full"] = {"rows": [dict(r) for r in page_rows], "layout": lst["layouts"].get(page),
                       "unseen": list(lst["unseen"]), "threshold": lst.get("threshold"), "floor": lst.get("floor"),
                       "seconds": lst["seconds"], "epoch": lst["epoch"], "call": lst["call"]}
    path = _picture(ex, lst, rec, page, page_rows)[0] if page_rows else None
    out["pictures"] = [path] if isinstance(path, str) else []
    out["next"] = nxt
    return out


def _same(lst, words):
    word = str(words).strip() if isinstance(words, str) and words.strip() else None
    return lst is not None and lst.get("words") == word and lst.get("source") != "find"


def list_(ex, look_id=None, words=None, page=1, for_routine=False, detail="brief", tool="list"):
    """The shared core of candidates, the numbers preference, seek and try_each: the page of the look's list, made
    now or kept (a fresh list of the same look and words is kept: its numbers stay). No call is charged here."""
    st = _state(ex)
    got = _resolve(ex, look_id)
    if isinstance(got, dict):
        return got
    key, rec = got
    lst = st["lists"].get(key)
    if not (_same(lst, words) and stale(ex, key) is None):
        lst = _make(ex, st, key, rec, words, tool=tool)
        if isinstance(lst, dict) and "refused" in lst:
            return lst
    else:
        st["newest"] = key
    return result_of(ex, lst, page, detail, for_routine)


def propose(ex, look_id, max=8, words=None):
    """The candidates of a look (9.3 shape), best first, at most max; [] when the look cannot be listed."""
    out = list_(ex, look_id, words, 1, tool="propose")
    lst = _state(ex)["lists"].get(out.get("look")) if "refused" not in out else None
    return [dict(r) for r in (lst or {}).get("items", [])[:int(max)]]


# ---------------------------------------------------------------- the two session tools
def _page_arg(page):
    if isinstance(page, bool) or not isinstance(page, int) or not 1 <= page <= 2:
        return _refuse("args", "page is 1 (numbers 1 to 8) or 2 (numbers 9 to 16)", "leave page out for page 1")
    return None


def candidates(ex, session, look=None, words=None, page=1, detail="brief", **_):
    bad = _page_arg(page)
    if bad is not None:
        return bad
    if detail not in ("brief", "full"):
        return _refuse("detail", "detail is brief or full", "leave it out")
    return list_(ex, look, words, page, detail=detail, tool="candidates")


def find(ex, session, words=None, look=None, max=8, detail="brief", **_):
    st = _state(ex)
    if not isinstance(words, str) or not words.strip() or len(words) > 200:
        return _refuse("args", "words is a short text that names one part or thing", 'for example find {"words": '
                       '"the round button"}')
    if isinstance(max, bool) or not isinstance(max, int) or not 1 <= max <= P.MAX_ROWS:
        return _refuse("args", f"max is a whole number from 1 to {P.MAX_ROWS}", "leave it out for 8")
    if st["level"] != "words":
        return _refuse("not_equipped", "this robot has no vision model that takes words (equipment.vision is "
                       f"{st['level']})", "candidates {} numbers the things in the picture from "
                       + ("the vision model's regions" if st["level"] == "segments" else "colour and geometry"))
    got = _resolve(ex, look)
    if isinstance(got, dict):
        return got
    key, rec = got
    lst = _make(ex, st, key, rec, words, tool="find", only_words=True, n_max=max)
    if isinstance(lst, dict) and "refused" in lst:
        return lst
    if lst is None:
        why = (st.get("service") or {}).get("why") or "it is not answering"
        return _refuse("not_equipped", f"the vision model did not answer ({why}), so words cannot be looked for now",
                       "candidates {} numbers the things in the picture from colour and geometry")
    answer, said = P.find_answer(lst, st["parts"], colour_words.near, NOT_TOUCHED)
    if answer == "none":
        lst["items"] = []
    head = {"said": _said(said), "answer": answer, "words": lst["words"]}
    if lst.get("threshold") is not None:
        head["threshold"] = lst["threshold"]
    return result_of(ex, lst, 1, detail, head=head)


HANDLERS = {"candidates": candidates, "find": find}


class Mixin:
    @primitive
    def choose(self, n=None, part=None, look=None, px=None, depth="auto", detail="brief", unconfirmed=False):
        from rcpx.r2 import eyes_choose
        return eyes_choose.run(self, n=n, part=part, look=look, px=px, depth=depth, detail=detail,
                               unconfirmed=unconfirmed)


# ---------------------------------------------------------------- the surface
_LOOK = {"anyOf": [{"type": "string"}, {"type": "integer", "minimum": 0}],
         "description": "a look id (L3); leave out for the newest look that can see"}
_DETAIL = S._str("brief (default) or full: full adds every row's pixels and the drawing's layout", ["brief", "full"])

TOOLS = [
    S._tool(
        "candidates",
        "The robot numbers the things it sees in a look and returns the picture with the numbers drawn on it, so "
        "you pick a number instead of a pixel. Each row says which part name of the manual found it (found_by) "
        "and that part's role, its colour, what the manual says about that colour, and keep_off when the manual "
        "forbids it. The picture is the look's own: a pixel read off it is a pixel of that look. Nothing moves. "
        "Example: candidates {\"look\": \"L3\"}.",
        S._obj({"look": _LOOK,
                "words": S._str("your own words for a thing to look for first (needs a vision model that takes "
                                "words)"),
                "page": S._int(1, 2, "1 (default): numbers 1 to 8; 2: numbers 9 to 16"),
                "detail": _DETAIL}),
        {"look": "L3"}, "number the things in look L3 and say what the manual tells about each",
        brief="The robot numbers the things in a look; you pick a number, not a pixel."),
    S._tool(
        "find",
        "The robot's vision model looks for a thing by your words in a look and numbers its matches, best first, "
        "with one answer word: found, unsure or none. A match is a guess from a picture and is not checked by "
        "touch: pick a number and let a touch or the device's own sign confirm it. Needs a vision model that takes "
        "words (robot://body equipment.vision is words). Nothing moves. Example: find {\"words\": \"the round "
        "button\"}.",
        S._obj({"words": S._str("what to look for, in your own words"),
                "look": _LOOK,
                "max": S._int(1, 16, "how many matches to number (default 8)"),
                "detail": _DETAIL}, ["words"]),
        {"words": "the round button"}, "ask the vision model where the round button is in the newest look",
        brief="The robot's vision model looks for a thing by your words and numbers its matches."),
    S._tool(
        "choose",
        "Pick a numbered candidate (or your own pixel in a look): the robot finds where it is in space and stores "
        "it as a located entry. depth auto measures it from more views when it can; views only measures; touch "
        "feels along the line of sight with a light guarded touch and backs off (one step, bounded in time and "
        "travel: it may run one nested move past a bound). Name the part when you know it; without a part the "
        "entry is stored as \"candidate N\". A candidate the manual forbids is refused. Example: choose {\"n\": 2, "
        "\"part\": \"button\"}.",
        S._obj({"n": S._int(1, 16, "the candidate's number in the newest list (or in the list of look)"),
                "part": S._str("the part's name from robot://devices, when you know which part it is"),
                "look": _LOOK,
                "px": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2,
                       "description": "[u, v]: your own pixel in look, instead of a number"},
                "depth": S._str("auto (default), views or touch", ["auto", "views", "touch"]),
                "detail": S._str("brief (default) or full", ["brief", "full"])}),
        {"n": 2, "part": "button"}, "take candidate 2 as the button and find where it is",
        brief="Pick a numbered candidate; the robot finds where it is and stores it."),
]

GAUGE = {"choose": "feel"}      # the gauge's class of this package's contact: it stops at the first contact, loads nothing
LADDER = [
    {"skill": "seeing", "rung": "S2", "tool": "candidates", "needs": ["look"],
     "line": "the robot numbers the things in a look; you pick a number, not a pixel"},
    {"skill": "seeing", "rung": "S3", "tool": "find", "needs": ["look", "eyes:words"],
     "line": "the robot's vision model looks for a thing by your words and numbers its matches"},
    {"skill": "seeing", "rung": "S2", "tool": "choose", "needs": ["candidates"],
     "line": "pick a number: the robot finds where it is, from more views or by a light touch"},
]


def _body_patch(ex, session, content):
    """robot://body gains exactly one key: equipment.vision, the level word."""
    if not isinstance(content, dict):
        return None
    equipment = dict(content.get("equipment") or {}) if isinstance(content.get("equipment"), dict) else {}
    equipment["vision"] = _state(ex)["level"]
    return {**content, "equipment": equipment}


RESOURCE_PATCH = {"robot://body": _body_patch}


def _attach(ex, session):
    st = _state(ex)
    st["session"] = session
    tools = getattr(session, "tools", None)
    if isinstance(tools, dict):
        for entry in st["hidden"]:
            tools.setdefault(entry["name"], entry)


def FILTER(surface, resources, handlers, ex=None, box=None):
    """Without the assist package: this package's tools leave the listing (they run by name; SPEC-R5 0.1 G5: an
    operator that never calls them is sent the same list as without this package). With it loaded, its own FILTER
    takes the help tools out and brings them back when the operator first takes help (6.2, 6.3)."""
    if ex is None:
        return surface, resources, handlers
    assist = any(m.__name__.rsplit(".", 1)[-1] == "assist" and callable(getattr(m, "FILTER", None))
                 for m in getattr(ex, "mods", ()) or ())
    out = surface
    if not assist:
        _state(ex)["hidden"] = [t for t in surface["tools"] if t.get("name") in OWN_TOOLS]
        out = dict(surface, tools=[t for t in surface["tools"] if t.get("name") not in OWN_TOOLS])
    if box is not None:
        box.on_session(lambda session: _attach(ex, session))
    return out, resources, handlers


__all__ = ["FILTER", "HANDLERS", "ID", "LADDER", "Mixin", "RESOURCE_PATCH", "TOOLS", "list_", "next_of", "propose",
           "ready", "result_of", "setup", "stale"]
