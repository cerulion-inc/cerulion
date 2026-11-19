# SPDX-License-Identifier: AGPL-3.0-only
"""seek (package assist): the robot looks from the sides a picture cannot show.

seek(words=None, around=None, max=3, until=None, detail="brief"). A short routine the robot runs itself: it takes up
to max looks from the unseen sides of one thing (robot side first), asks the eyes package for each look's numbered
list, and stops when the model's stop condition is met, a gate refuses, two sides in a row are out of reach or a
bound is reached. It moves the camera only, touches nothing, stores nothing in the world and never claims. The
camera moves and how they keep clear of the thing are rcpx/r2/assist_seek_move.py (public primitives only).

- around: a number of the newest FRESH list (a stale one: refused stale_list before anything moves), a located or
  contact id, a survey candidate id. Default: the first row of the newest fresh list when a word found it, else
  that list's largest row, the first of them that is whole in its picture and within the arm's reach; else the
  largest survey candidate. A row is looked round at the thing it sits on: the largest outline of that list that
  holds the row's point. A bare table point (the middle of a look) is NOT a default here: nothing is known of what
  stands there, so no camera move round it can be kept clear (refused no_around).
- words: the model's own, else every control part the manual names (the robot does not choose one). The eyes list
  is asked with the model's words as text, else with the list of those names.
- until (the model's, optional): "found WORDS" (level words only; else refused until_unavailable) or "new none"; a
  bare "found" means found by the words of the call, and by the manual's control part names when the call gives
  none. until_from says where the words came from. With no until the looks run to the bound, stopped "looked".
- sides: robot side, left, right, far, those the picture cannot show: the list's own unseen record when around is
  its first row, else the same rule worked for around; all four when no look is kept.
- Bounds (the guard's Bounds; tested before EVERY nested call): looks at most max (1 to 4); one call charged per
  look taken (the session charges the first, the budget is lowered for each further one); a quarter of the seconds
  the session has left at the start (no time bound without a time budget, and said says so); tip travel at most
  3 x the reach radius of robot://body, one move at most 1 x. Not started when the calls left are under max + 2
  (budget_short) or the seconds left under one move's own limit, cfg max_move_s (time_short), nor while the hand
  holds something (holding).

Result: said, stopped (found | no_new | looked | refused | bound | out_of_reach), until_from, look,
steps [{i, side, look, stopped, new, found_by, seconds, travel_mm, why}], candidates, end {tip_mm}, by "robot",
pictures, next. Journal: assist_step {by, routine, i, of, side, verdict, seconds, travel_mm} per look. Nothing here
reads simulator ground truth or names a robot, a device, a task or a model.
"""

import json

import numpy as np

from rcpx import r3_limits as L
from rcpx import side_words, surfaces
from rcpx.context import refusal
from rcpx.r2 import assist_seek_move as M

try:                                   # the pure siblings of this package: used when they import
    from rcpx.r2 import assist_until as _until
except Exception:                      # noqa: BLE001 (a sibling that is missing or broken is worked without)
    _until = None
try:
    from rcpx.r2 import assist_guard as _guard
except Exception:                      # noqa: BLE001
    _guard = None

MM = M.MM
LOOKS_MAX = 4
BRIEF_MAX = 900
SAID_WORDS = 45
SIDE_OF = {"robot side": "robot_side", "left side": "left", "right side": "right", "far side": "far_side"}
SEEN_DEG = 100.0             # a face is seen when its outward direction makes over this with the line of sight (4.1)
NOT_TOUCHED = "not checked by touch"


class _Bounds:
    """The stand-in for assist_guard.Bounds (same signatures), used only when the guard does not import."""

    def __init__(self, steps, calls_left, seconds_left, reach_mm, step_seconds=None):
        self.steps, self.calls_left, self.seconds_left = int(steps), calls_left, seconds_left
        self.reach_mm, self.step_seconds = float(reach_mm or 0.0), step_seconds
        self.time_bound_s = None if seconds_left is None else 0.25 * float(seconds_left)

    def start(self):
        if self.calls_left is not None and self.calls_left < self.steps + 2:
            return {"check": "budget_short", "why": f"{self.steps} looks need {self.steps + 2} calls and "
                    f"{self.calls_left:g} are left"}
        if None not in (self.step_seconds, self.seconds_left) and self.seconds_left < self.step_seconds:
            return {"check": "time_short", "why": f"one look can take {self.step_seconds:g} s and "
                    f"{self.seconds_left:g} s are left"}
        return None

    def before(self, steps_started, seconds_used, travel_mm, next_mm=0.0):
        if steps_started >= self.steps:
            return "steps"
        if self.time_bound_s is not None and seconds_used >= self.time_bound_s:
            return "time"
        return "travel" if travel_mm + next_mm > 3.0 * self.reach_mm or next_mm > self.reach_mm else None


GUARD_RAN = "the sibling assist_guard" if getattr(_guard, "Bounds", None) else "the stand-in in assist_seek"
UNTIL_RAN = "the sibling assist_until" if callable(getattr(_until, "parse", None)) else "the stand-in in assist_seek"


def _reach_mm(ex):
    try:
        body = ex.context.get("body")
        return float((body() if callable(body) else body)["reach"]["max_reach_mm"])
    except Exception:                  # noqa: BLE001 (no body document here: the model's own figure)
        return float((ex.model.reach() or {}).get("max_reach_mm") or 0.0)


def _has(session, ex, name):
    tools = getattr(session, "tools", None)
    return name in tools if tools else callable(getattr(ex, name, None))


def _ready(ex, tool, args, needs=None, why=None):
    """A next object: through the assist package's ready_call when it is loaded, else {tool, args}."""
    a = ex.state.get("assist")
    fn = a.get("ready_call") if isinstance(a, dict) else None
    if callable(fn):
        try:
            out = fn(tool, args, "next", needs)
        except Exception:              # noqa: BLE001
            out = None
        if isinstance(out, dict):
            return out
    out = {"tool": tool, "args": args}
    if needs:
        out["needs"] = needs
    if why:
        out["why"] = why
    return out


def _refuse(check, why, tool=None, args=None, text=None):
    hint = {"tool": tool, "args": args or {}} if tool else (text or "look {} shows where things stand now")
    return {"said": why, **refusal(check, why, hint)}


# ---------------------------------------------------------------- what to look round
def _area(r):
    b = r.get("box_px") or [0, 0, 0, 0]
    return (b[2] - b[0]) * (b[3] - b[1])


def _unseen(ex, look, point_m):
    """The sides of an upright box at the point that the look cannot show, by the rule (the eyes
    package's rule, restated here: a package may not import another); None when the look is not kept."""
    rec = (getattr(ex, "_looks", None) or {}).get(look)
    if rec is None:
        return None
    v = np.asarray(point_m, dtype=float) - np.asarray(ex.model.fk(np.asarray(rec["q"], dtype=float))["camera"][0])
    v, r = v / max(float(np.linalg.norm(v)), 1e-9), side_words.radial(np.asarray(point_m, dtype=float) * MM)
    left = np.cross(side_words.Z, r)
    faces = {"robot side": -r, "far side": r, "left side": left, "right side": -left, "top": side_words.Z}
    return [name for name, f in faces.items() if side_words.angle_deg(f, v) <= SEEN_DEG]


def _newest_look(ex):
    looks = getattr(ex, "_looks", None) or {}
    for key in sorted(looks, key=lambda k: int(k[1:]) if k[1:].isdigit() else 0, reverse=True):
        if looks[key].get("can_see") != "dark":
            return key
    return None


def _row_target(ex, look, lst, row, reach_mm):
    """The thing a row sits on (the largest outline of the list that holds the row's point), as a target."""
    rec = (getattr(ex, "_looks", None) or {}).get(look)
    if rec is None:
        return _refuse("unknown_look", f"the look {look} of that list is not kept", "candidates", {})
    q, n, box = np.asarray(rec["q"], dtype=float), row.get("n"), row.get("box_px")
    pt = row.get("point_px") or ([0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3])] if box else None)
    if pt is None:
        return _refuse("no_around", f"number {n} of {look} carries no pixel to look round", "candidates", {})
    hosts = [r for r in lst.get("items") or () if r.get("box_px")
             and r["box_px"][0] <= pt[0] <= r["box_px"][2] and r["box_px"][1] <= pt[1] <= r["box_px"][3]]
    keep = M.keep_of(ex, q, max(hosts, key=_area) if hosts else row)
    if keep is None:
        return _refuse("no_around", f"the outline of number {n} (or of what it stands on) is cut by the picture's "
                       "edge or does not meet the table: its size is not known, so the camera cannot be kept clear "
                       "of it", text="look at it from further away so that it is whole in the picture, then "
                       "candidates {} and seek {}")
    point = np.array([keep[0][0], keep[0][1], M.floor(ex) + 0.5 * keep[2]])
    off = float(np.hypot(point[0], point[1])) * MM
    if off > reach_mm or keep[1] * MM > reach_mm / 3.0:
        return _refuse("no_around", f"number {n} stands {off:.0f} mm from the robot's base axis (reach {reach_mm:.0f} "
                       f"mm) and reads {2.0 * keep[1] * MM:.0f} mm wide: the camera cannot go round it",
                       "candidates", {})
    tid = None
    for c in (ex.state.get("sight") or {}).get("candidates") or ():     # a survey box that holds it: view takes it
        try:
            (x, y), (sx, sy) = c["where_mm"][:2], c["size_mm"][:2]
        except (KeyError, TypeError, ValueError):
            continue
        g = L.DEVICE_BOX_NEAR_MM
        if abs(point[0] * MM - x) <= 0.5 * sx + g and abs(point[1] * MM - y) <= 0.5 * sy + g:
            tid = c.get("id")
    first = bool(lst.get("items")) and lst["items"][0] is row and lst.get("unseen")
    return {"id": tid, "n": n, "look": look, "point_m": point, "keep": keep, "say": f"number {n}",
            "unseen": lst["unseen"] if first else _unseen(ex, look, point)}


def _stale(ex, eyes, look, lst):
    fn = eyes.get("stale")
    if callable(fn):
        return fn(look)
    if callable(getattr(_guard, "list_stale", None)):
        return _guard.list_stale(lst, getattr(ex, "epoch", 0), [])
    return "the scene's epoch rose" if int(lst.get("epoch", 0)) != int(getattr(ex, "epoch", 0)) else None


def _candidate(ex, c):
    x, y = (float(v) for v in c["where_mm"][:2])
    z = 0.5 * (float(c.get("top_mm") or 0.0) + float(c.get("floor_z_mm") or 0.0))
    point = np.array([x, y, z]) / MM
    return {"id": c["id"], "n": None, "look": None, "point_m": point, "keep": None, "say": c["id"],
            "unseen": _unseen(ex, _newest_look(ex), point)}


def _around(ex, session, eyes, around, reach_mm):
    """What to look round: {id, n, look, point_m, keep, say, unseen}, or a refusal. Nothing moves here."""
    lists, newest = eyes.get("lists") or {}, eyes.get("newest")
    lst = lists.get(newest) if newest is not None else None
    stale = _stale(ex, eyes, newest, lst) if lst is not None else "no list was made yet"
    fresh = ("candidates", {}) if _has(session, ex, "candidates") else ("look", {})
    cands = [c for c in (ex.state.get("sight") or {}).get("candidates") or () if c.get("id")]
    num = around if isinstance(around, int) and not isinstance(around, bool) else (
        int(around) if isinstance(around, str) and around.strip().isdigit() else None)
    if num is not None or around is None:
        row = None
        if num is not None:
            if stale:
                return _refuse("stale_list", f"number {num} cannot be used: {stale}; its numbers may point at the "
                               "wrong place now", *fresh)
            row = next((r for r in lst.get("items") or () if r.get("n") == num), None)
            if row is None:
                valid = ", ".join(str(r.get("n")) for r in lst.get("items") or ()) or "none"
                return _refuse("unknown_candidate", f"the list of {newest} has no number {num}; it holds: {valid}",
                               *fresh)
        if row is not None:
            return _row_target(ex, newest, lst, row, reach_mm)
        if not stale and lst.get("items"):
            # the default: the first row when a word found it, else the largest numbered thing; the first of them
            # that stands where the camera can go round it (a list from colour alone may open with the background)
            items = lst["items"]
            for row in ([items[0]] if items[0].get("found_by") else []) + sorted(items, key=_area, reverse=True):
                tg = _row_target(ex, newest, lst, row, reach_mm)
                if "refused" not in tg:
                    return tg
        if cands:
            return _candidate(ex, max(cands, key=lambda c: float(c["size_mm"][0]) * float(c["size_mm"][1])))
        return _refuse("no_around", "there is nothing the camera can be kept clear of while it looks round: no "
                       "numbered thing that is whole in its picture and within reach, and no survey candidate",
                       *(("assist", {"pick": "N"}) if not stale and lst.get("items") and _has(session, ex, "assist")
                         else fresh if not stale or not _has(session, ex, "survey") else ("survey", {})))
    key = str(around).strip()
    for c in cands:
        if c["id"] == key:
            return _candidate(ex, c)
    e = ex.world.get(key)
    if e is None or e.get("kind") not in ("located", "contact"):
        return _refuse("unknown_target", f"{key[:40]!r} is no number of the list, located or contact id, or survey "
                       "candidate", *fresh)
    data = e.get("data") or {}
    if e.get("stale") or int(data.get("epoch", getattr(ex, "epoch", 0)) or 0) < int(getattr(ex, "epoch", 0)):
        return _refuse("stale_target", f"{key} is stale: {data.get('stale_why') or 'it may have moved'}", *fresh)
    pt = e.get("point_mm") or data.get("bracket_mid_mm")
    if pt is None:
        return _refuse("unknown_target", f"{key} holds no point to look round", *fresh)
    point = np.asarray(pt, dtype=float) / MM
    return {"id": key, "n": None, "look": None, "point_m": point, "keep": None, "say": key,
            "unseen": _unseen(ex, _newest_look(ex), point)}


# ---------------------------------------------------------------- the stop condition and the rows of a look
def _parse(until):
    """(form, words): form is found | new | None (no condition) | "bad"."""
    if until is None or until == "" or until == {}:
        return None, None
    if isinstance(until, str) and until.strip().lower() == "found":
        return "found", None
    got = None
    if callable(getattr(_until, "parse", None)):
        got = _until.parse(until)
    elif isinstance(until, dict) and len(until) == 1:
        got = {str(k).lower(): v for k, v in until.items()}
    elif isinstance(until, str) and len(until.split(None, 1)) == 2:
        key, value = until.strip().lower().split(None, 1)
        got = {key: " ".join(value.split())}
    if isinstance(got, dict) and got.get("new") == "none":
        return "new", None
    if isinstance(got, dict) and isinstance(got.get("found"), str) and got["found"].strip():
        return "found", got["found"].strip()
    return "bad", None


def _rows(ex, eyes, look, words):
    """{rows, threshold, brief, pic, why} of a look's list (eyes makes and keeps it; no call is charged)."""
    fn = eyes.get("list")
    if not callable(fn):
        return {"rows": None, "why": "no candidates package is loaded: the looks are not numbered"}
    try:
        res = fn(look, words)
    except Exception as exc:           # noqa: BLE001
        ex._journal("r2_problem", package="assist", where="seek list", error=f"{type(exc).__name__}: {exc}")
        res = None
    if not isinstance(res, dict) or "refused" in res:
        why = ((res or {}).get("refused") or {}).get("why") or "the list could not be made"
        return {"rows": [], "why": str(why)[:120]}
    lst = (eyes.get("lists") or {}).get(res.get("look") or look) or {}
    rows = lst.get("items") if isinstance(lst.get("items"), list) else (res.get("candidates") or [])
    pics = res.get("pictures") or []
    return {"rows": rows, "threshold": lst.get("threshold", res.get("threshold")), "brief": res.get("candidates"),
            "pic": pics[0] if pics and isinstance(pics[0], str) else None, "why": None}


def _best(rows, names, threshold):
    """The best row found by one of the names at or over the equipment's threshold, else None. A row whose colour
    the manual gives another part, or that the manual forbids, is never one."""
    best = None
    for r in rows or ():
        fb, conf = " ".join(str(r.get("found_by") or "").lower().split()), r.get("confidence")
        if fb not in names or r.get("keep_off") or r.get("manual_part") not in (None, r.get("found_by")):
            continue
        if not isinstance(threshold, (int, float)) or not isinstance(conf, (int, float)) or conf < threshold:
            continue
        step = {"found_by": [fb], "confidence": conf, "threshold": threshold}
        if callable(getattr(_until, "met", None)) and _until.met({"found": fb}, step, None, None)[0] is not True:
            continue                   # the sibling's own reading of the same row
        if best is None or conf > best["confidence"]:
            best = r
    return best


# ---------------------------------------------------------------- the tool
def seek(ex, session, words=None, around=None, until=None, detail="brief", **more):
    n_max = more.get("max", 3)
    if isinstance(n_max, bool) or not isinstance(n_max, int) or not 1 <= n_max <= LOOKS_MAX:
        return _refuse("args", f"max is a whole number of looks from 1 to {LOOKS_MAX}, not {str(n_max)[:20]!r}",
                       "seek", {"max": LOOKS_MAX})
    if detail not in ("brief", "full"):
        return _refuse("detail", f"detail is brief or full, not {str(detail)[:20]!r}", "seek", {})
    if any(isinstance(v, dict) and v.get("held") for v in ex.state.values()):
        return _refuse("holding", "the hand holds something: a camera move would carry it along",
                       text="set it down or release it first, then seek {}")
    eyes = ex.state.get("eyes") if isinstance(ex.state.get("eyes"), dict) else {}
    if eyes:
        eyes["sought"] = True      # a list's next names seek once per session (rcpx/r2/eyes.py next_of)
    level, parts = eyes.get("level") or "absent", eyes.get("parts") or {}
    control = [str(n).lower() for n in parts.get("control") or ()]
    own = " ".join(str(words).split()) if isinstance(words, str) and words.strip() else None
    form, uwords = _parse(until)
    other = dict({"words": own} if own else {}, **({"around": around} if around is not None else {}))
    if form == "bad":
        return _refuse("args", 'until is "found WORDS" or "new none" (or left out: the robot then takes its looks '
                       "to its bound)", "seek", {**other, "until": "new none"})
    if form == "new" and not callable(eyes.get("list")):
        return _refuse("until_unavailable", "new none needs numbered lists, and no candidates package is loaded",
                       "seek", other)
    names = [uwords.lower()] if uwords else ([own.lower()] if own else control)
    if form == "found" and (level != "words" or not names):
        why = (f"found needs a vision model that finds by words; this robot's vision equipment is {level}"
               if level != "words" else "found needs words, and the manual names no control part")
        return _refuse("until_unavailable", why + '; the other stop condition is "new none"', "seek",
                       {**other, "until": "new none"})
    until_from = None if form is None else ("the manual's part names" if form == "found" and not (uwords or own)
                                            else "you")
    ask = own or (uwords if uwords and uwords.lower() not in parts.get("roles", control) else None) or control or None
    reach_mm, step_s = _reach_mm(ex), float(ex.cfg.get("max_move_s") or 0.0) or None
    tg = _around(ex, session, eyes, around, reach_mm)
    if "refused" in tg:
        return tg
    a = ex.state.get("assist")
    bound_fn = (a.get("guard") or {}).get("bounds") if isinstance(a, dict) else None
    if callable(bound_fn):
        bounds, ran = bound_fn(n_max, session, step_s, reach_mm), "the bound guard of the assist package"
    else:
        left = session.remaining() if callable(getattr(session, "remaining", None)) else {}
        bounds = (getattr(_guard, "Bounds", None) or _Bounds)(n_max, left.get("calls_left"),
                                                             left.get("sim_seconds_left"), reach_mm, step_s)
        ran = GUARD_RAN
    bad = bounds.start()
    if bad:
        bad = bad.get("refused", bad)
        room = int(getattr(bounds, "calls_left", None) or 0) - 2
        if bad.get("check") == "budget_short" and room >= 1:
            return _refuse("budget_short", f"{bad.get('why')}: a seek of {n_max} looks is not started", "seek",
                           {**other, "max": min(room, LOOKS_MAX)})
        return _refuse(str(bad.get("check")), f"{bad.get('why')}: seek is not started",
                       text="look {} is one look of your own; claim what you achieved when nothing is left")
    sides = [SIDE_OF[s] for s in SIDE_OF if not tg["unseen"] or s in tg["unseen"]]
    run = M.Run(ex, session, bounds, reach_mm)
    seen = {(r.get("colour"), r.get("size")) for r in ((eyes.get("lists") or {}).get(tg["look"]) or {}).get("items")
            or ()}
    steps, stopped, stop, misses, found, best_new, done = [], None, {}, 0, None, None, set()
    for side in sides:
        if run.taken >= n_max:
            break
        if side in done:               # view gave way to this side earlier: it was looked from
            continue
        t1, trav1 = ex.sim_time(), run.travel
        got = M.camera_to(run, tg, side)
        st = {"i": len(steps) + 1, "side": M.SAY.get(got.get("side") or side), "stopped": got["stopped"],
              "seconds": round(ex.sim_time() - t1, 1), "travel_mm": round(run.travel - trav1)}
        steps.append(st)
        if got["stopped"] != "looked":
            st["why"] = str(got.get("why") or "")[:100]
            misses += 1
            if got["stopped"] == "out_of_reach" and misses < 2:
                continue
            stopped, stop = got["stopped"], got
            break
        misses = 0
        done.add(got.get("side") or side)
        if run.taken >= 1 and "calls" in getattr(session, "budget", {}):
            session.budget["calls"] = int(session.budget["calls"]) - 1      # one call per look after the first
        run.taken += 1
        st["look"] = got["look"]
        dark = got["can_see"] == "dark"
        lst = {"rows": [], "why": "a dark look"} if dark else _rows(ex, eyes, got["look"], ask)
        lst["pic"] = lst.get("pic") or got.get("pic")
        hit = _best(lst["rows"], names, lst.get("threshold")) if level == "words" else None
        if lst["rows"] is not None:
            st["new"] = sum((r.get("colour"), r.get("size")) not in seen for r in lst["rows"])
            seen |= {(r.get("colour"), r.get("size")) for r in lst["rows"]}
        if hit is not None:
            st["found_by"], st["n"] = hit.get("found_by"), hit.get("n")
            if found is None or hit["confidence"] > found[1]["confidence"]:
                found = (st, hit, lst)
        if best_new is None or (st.get("new") or 0) > (best_new[0].get("new") or 0):
            best_new = (st, None, lst)
        verdict = "dark" if dark else "found" if hit is not None else "nothing new" if st.get("new") == 0 else "looked"
        ex._journal("assist_step", by="robot", routine="seek", i=run.taken, of=n_max, side=st["side"], verdict=verdict,
                    seconds=st["seconds"], travel_mm=st["travel_mm"], look=got["look"])
        if dark:
            st["stopped"] = "dark"
            stopped, stop = "refused", {"check": "dark", "why": f"the look from the {st['side']} read dark"}
        elif form == "found" and hit is not None:
            stopped = "found"
        elif form == "new" and st.get("new") == 0:
            stopped = "no_new"
        if stopped:
            break
    if stopped is None and run.taken >= n_max:     # the looks ran out: a bound when a condition was left unmet
        stopped = "bound" if form else "looked"
    elif stopped is None:                          # the sides ran out
        missed = any(s["stopped"] == "out_of_reach" for s in steps)
        stopped = "out_of_reach" if missed and (form or not run.taken) else "looked"
    return _result(ex, session, eyes, tg, run, steps, stopped, stop, until_from, names, ask, found, best_new, detail,
                   {"looks": n_max, "time_s": getattr(bounds, "time_bound_s", None),
                    "travel_mm": round(3.0 * reach_mm), "tested_by": ran, "until_read_by": UNTIL_RAN})


def _said(clauses):
    """One sentence of at most SAID_WORDS words: the clauses in order, the bracketed reasons dropped first."""
    text = "; ".join(c for c, _ in clauses)
    if len(text.split()) > SAID_WORDS:
        text = "; ".join(short for _, short in clauses if short)
    return " ".join(text.split()[:SAID_WORDS])


def _result(ex, session, eyes, tg, run, steps, stopped, stop, until_from, names, ask, found, best_new, detail,
            bounds):
    pick = found or (best_new if best_new and best_new[2].get("rows") else None)
    look = pick[0]["look"] if pick else next((s["look"] for s in reversed(steps) if s.get("look")), None)
    lst = pick[2] if pick else {}
    if pick and eyes.get("newest") != look:        # the result's look holds the newest list from here on
        lst = dict(_rows(ex, eyes, look, ask), pic=lst.get("pic"))
    looked = [s["side"] for s in steps if s.get("look")]
    missed = [s for s in steps if s["stopped"] == "out_of_reach"]
    nxt, why_none = _next(ex, session, eyes, tg, look, found, pick, stopped)
    clauses = []                                   # (the clause, its short form or None when it may be dropped)
    if found:
        st, hit, flst = found
        text = (f"number {hit.get('n')} in {st['look']} was found by {hit.get('found_by')} ({hit['confidence']:g}, "
                f"threshold {flst.get('threshold'):g}); {NOT_TOUCHED}")
        clauses.append((text, text))
    if looked:
        text = "looked from the " + ", the ".join(looked)
        clauses.append((text, text))
    if missed:
        text = "could not reach the " + ", the ".join(s["side"] for s in missed)
        clauses.append((f"{text} ({missed[0].get('why', '')[:60]})", text))
    if stopped == "refused":
        clauses.append((f"stopped by {stop.get('check')}: {str(stop.get('why') or '')[:80]}",
                        f"stopped by {stop.get('check')}"))
    elif steps and steps[-1]["stopped"] == "bound":
        text = f"its {steps[-1].get('why')} bound was reached before the next move"
        clauses.append((text, text))
    if not found and looked:
        new = sum(s.get("new") or 0 for s in steps)
        text = f"nothing was found by {' or '.join(names)}" if names and eyes.get("level") == "words" else ""
        if any("new" in s for s in steps):
            text += ("; " if text else "") + f"{new} thing{'' if new == 1 else 's'} not listed before" + (
                f", numbered in {look}" if new else "")
        clauses.append((text or "the looks are not numbered", text or "the looks are not numbered"))
    elif not steps:
        text = f"no side of {tg['say']} is left unseen by the newest list: nothing was looked at"
        clauses.append((text, text))
    if nxt is None:
        clauses.append((why_none, why_none))
    if bounds["time_s"] is None:
        clauses.append(("no time bound: this session has no time budget", None))
    out = {"said": _said(clauses), "stopped": stopped}
    if stopped == "refused":
        out.update(check=stop.get("check"), why=str(stop.get("why") or "")[:120])
    keys = {"i", "side", "look", "stopped", "new", "found_by", "seconds", "why"}
    out.update(until_from=until_from, look=look, charged_calls=max(1, run.taken),
               steps=[{k: v for k, v in s.items() if detail == "full" or k in keys} for s in steps],
               candidates=list(lst.get("brief") or ())[:8])
    if detail == "full":
        out["full"] = {"bounds": bounds, "seconds": round(ex.sim_time() - run.t0, 1), "travel_mm": round(run.travel),
                       "around": tg["say"], "words_asked": ask, "rows": lst.get("rows")}
    out.update(end={"tip_mm": [round(float(x) * MM, 1) for x in M.tip(ex)]}, by="robot",
               pictures=[lst["pic"]] if lst.get("pic") else [])
    out["next"] = nxt
    while detail != "full" and len(json.dumps({k: v for k, v in out.items() if k != "pictures"})) > BRIEF_MAX:
        if out["candidates"]:                      # the numbered picture shows them all: the list is one call away
            out["candidates"].pop()
            out["rows_left_out"] = out.get("rows_left_out", 0) + 1
        elif any(len(s.get("why", "")) > 20 for s in out["steps"]):
            out["steps"] = [dict(s, why=s["why"][:20]) if "why" in s else s for s in out["steps"]]
        else:
            break
    out["next"] = out.pop("next")                  # next stays the last key
    return out


def _next(ex, session, eyes, tg, look, found, pick, stopped):
    """(next, why there is none). Never a number filled in for the model, never a claim."""
    a = ex.state.get("assist") if isinstance(ex.state.get("assist"), dict) else {}
    parts = eyes.get("parts") or {}
    if pick:        # a look listed rows: the pick is the next whether or not a word reached the threshold
        routine = (a.get("prefs") or {}).get("acting") == "routine"
        try:
            routine = routine or int((a["gauge"].standing() or {}).get("acting") or 0) > 0
        except Exception:              # noqa: BLE001 (no gauge here: no open event is known)
            pass
        if routine and _has(session, ex, "try_each") and _has(session, ex, "push") and not parts.get("forbidden"):
            cue = bool(parts.get("cue"))               # until is filled only from the cue the manual declares
            return _ready(ex, "try_each", {"until": "cue changes"} if cue else {},
                          None if cue else "until: your stop condition in two words"), None
        if _has(session, ex, "assist"):
            return _ready(ex, "assist", {"pick": "N"}, "N: the number you choose"), None
        if _has(session, ex, "choose"):
            return _ready(ex, "choose", {"look": look}, "n: the number you choose"), None
        return None, "no tool that takes a number is loaded: point at it yourself in that look"
    if stopped == "refused":
        return _ready(ex, "look", {}, why="see where the camera ended"), None
    if "top" in (tg["unseen"] or ()) and tg["id"] is not None and _has(session, ex, "view"):
        return _ready(ex, "view", {"target": tg["id"], "side": "above"}, why="the top was not seen either"), None
    if _has(session, ex, "ask_person"):
        return _ready(ex, "ask_person", {"kind": "device_info", "question": "which side of the device is the part "
                                         "on? the camera did not find it on the sides it could reach"}), None
    return None, "nothing more to try from here"


HANDLERS = {"seek": seek}
TOOLS = [surfaces._tool(
    "seek",
    "The robot moves its camera to the sides of a thing that no picture so far shows (the robot's side first), "
    "numbers what each look shows, and stops when your stop condition is met or its bound is reached. It moves the "
    "camera only and touches nothing. Bounds: at most max looks (1 to 4), one call charged per look, a quarter of "
    "the time you have left, camera travel at most 3 times the arm's reach. The bounds are tested before every "
    "camera move and a move cannot be cut short, so it can pass one bound by a single camera move. Example: seek "
    "{\"words\": \"button\", \"until\": \"found button\"}.",
    surfaces._obj({
        "words": surfaces._str("what to look for, in your words (default: every control part the manual names)"),
        "around": {"anyOf": [{"type": "integer", "minimum": 1}, {"type": "string"}],
                   "description": "what to look round: a number of the newest list, a located or contact id, or a "
                                  "survey candidate (default: number 1 of the newest list)"},
        "max": {"type": "integer", "description": "looks at most, 1 to 4 (default 3)"},
        "until": {"anyOf": [{"type": "string"}, {"type": "object"}],
                  "description": "your stop condition, optional: \"found WORDS\" (a look shows a thing the vision "
                                 "model finds by those words) or \"new none\" (a look shows nothing new); left out, "
                                 "the robot takes its looks to its bound and reports each"},
        "detail": surfaces._str("brief (default) or full", ["brief", "full"])}),
    {"words": "button", "until": "found button"},
    "look from the sides no picture shows until a look shows a thing found by the word button",
    brief="The robot looks from the sides no picture shows and numbers what it finds there.")]

__all__ = ["GUARD_RAN", "HANDLERS", "TOOLS", "UNTIL_RAN", "seek"]
