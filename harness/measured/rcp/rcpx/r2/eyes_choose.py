# SPDX-License-Identifier: AGPL-3.0-only
"""choose (package eyes): pick a numbered candidate, or the operator's own pixel in a look, and
the robot finds where it is in space and stores it as a located entry.

choose(n=None, part=None, look=None, px=None, depth="auto", detail="brief")
- n: a number of the newest list (or of the list of look). A list that went stale with the scene is refused
  stale_list (4.7 rule 6); an unknown number is refused unknown_candidate naming the valid numbers; a row that
  carries keep_off is refused forbidden with the manual's own sentence. Or look and px: the operator points itself.
- the pixel on the robot's own body: refused own_body.
- part: the operator's own word; else the row's found_by when the manual gives that part the role control; else
  none: the entry is then labelled "candidate N" with data.unconfirmed true. The robot fills in no other part.
- depth auto and views: the sight package's locate (its gates first: ex.gates("locate")), when it is loaded. The
  located entry gains data.candidate "eyes:LOOK:N" and data.outline_px (an id a survey gave it moves to
  data.survey_candidate). depth auto goes on to a touch only when locate is absent or answered ray_only AND no
  whole-step tool (push, act_on) is loaded that would feel along the line itself.
- depth touch is a contact the robot makes on its own and obeys the robot's own rules:
  the line of sight from a standoff (the point of the line nearest the camera that reach_check accepts) to where
  it meets the table (or leaves the arm's reach) plus 10 mm is tested BEFORE anything moves by the guard the
  assist package serves (ex.state["assist"]["guard"] keep_off); with no guard and a part the manual forbids:
  refused not_guarded. Then, each after ex.gates and a light reading: a guarded joint move to the pose reach_check
  solved for the standoff (the tool's leading face then points down the line; with no such pose, or a joint
  path that is not free, a guarded move_tip and a note), and a touch along the line at the session's own touch
  force, in pieces. A contact is tested by the guard again (inside a keep-off: stopped harm_guard, no entry) and
  stored as the located point (a contact the tool made with another face than its tip is kept, with a note);
  nothing met: stopped not_found. One step's bounds: commanded travel at most the standoff move plus the line,
  never over the reach radius of robot://body; time at most a quarter of the session's simulated seconds left,
  tested before each nested call (a nested call cannot be cut short: the worst case is the bound plus one nested
  call). Every exit backs the tool off (30 mm back along the line; by retreat
  when that is refused, as in the dark) and reports end {tip_mm, backed_off}; every leg is reported with its
  seconds and journaled as r3_leg {by: "runtime", tool: "choose"}.
  No force is typed here: the touch uses cfg touch_force_n, as an operator's touch with no force does.

Result: said, stopped (a touch), located, point_mm, uncertain_mm, depth, chosen {n, look},
part_check, identity, legs, end, notes, pictures, next. next is a whole-step call only when the guard's target_ok
passes for the new entry; with a part_check warning it is a routine over the other candidates (or the list again);
else a touch. Nothing here reads simulator ground truth or names a robot or a device.
"""

import json
import math

import numpy as np

from rcpx import colour_words
from rcpx import r3_limits as L
from rcpx.context import refusal
from rcpx.r2 import eyes as E
from rcpx.r2 import eyes_local as LOC
from rcpx.r2 import eyes_parts as P
from rcpx.r2 import eyes_views as VIEWS

MM = 1000.0
END_EXTRA_MM = 10.0          # the touch may run this far past where the line meets the table
STANDOFF_STEP_MM = 20.0      # the spacing of the standoff search along the line
POINT_ERR_PX = 2.0           # the pointing error the line's own uncertainty is stated for
TOUCH_UNC_MM = 2.0           # the uncertainty of a point a touch measured
TABLE_TOL_MM = 4.0           # a contact this near the standing surface is said to be at table height
TIME_SHARE = 0.25            # of the simulated seconds the session has left (4.7 rule 7)
PIECE_MARGIN_S = 4.0         # a touch piece leaves this much of r3_limits.TOUCH_PIECE_S for its ramp
BACK_MM = 30.0               # every exit backs the tool this far off along the line it came down
BACKED_OFF_MM = 5.0          # the tip counts as backed off when it stands this far from the felt point
BODY_WORDS = {"own_base": "the robot's own base", "own_arm": "the robot's own arm",
              "own_hand": "the robot's own hand"}


class _Stop(Exception):
    def __init__(self, stopped, **info):
        super().__init__(stopped)
        self.stopped, self.info = stopped, info


def _mm(p):
    return [round(float(x) * MM, 1) for x in p]


def _refuse(check, why, hint):
    return {"said": why, **refusal(check, why, hint)}


def _body(r):
    """The refusal body of a gate, a guard or a primitive answer: {"check", "why", "hint"}."""
    b = r.get("refused") if isinstance(r, dict) and isinstance(r.get("refused"), dict) else r
    b = b if isinstance(b, dict) else {}
    return {"check": str(b.get("check") or "refused"), "why": str(b.get("why") or "refused"),
            "hint": str(b.get("hint") or "")}


def _guard(ex):
    a = ex.state.get("assist")
    g = a.get("guard") if isinstance(a, dict) else None
    return g if isinstance(g, dict) else None


# ---------------------------------------------------------------- the target
def _target(ex, st, n, look, px, depth=None):
    """(look id, record, u, v, row or None, n or None) or a refusal."""
    looks = getattr(ex, "_looks", {}) or {}
    if n is None:
        key = E._key(ex, look)
        rec = looks.get(key)
        if rec is None:
            return _refuse("unknown_look", f"no look {str(look)[:20]!r} in this session", "pass a look id a look "
                           "returned, with px [u, v]; or a number of a list: candidates {}")
        if rec.get("can_see") == "dark":
            return _refuse("can_see", f"look {key} is dark: a pixel in it shows nothing",
                           "look again once the light reads dim or ok")
        u, v = float(px[0]), float(px[1])
        if not (0 <= u < ex.model.cam_w and 0 <= v < ex.model.cam_h):
            return _refuse("outside_frame", f"pixel ({u:g}, {v:g}) is outside the {ex.model.cam_w} x "
                           f"{ex.model.cam_h} picture of {key}", "u from 0 to width, v from 0 to height")
        lst = st["lists"].get(key)
        row = next((r for r in (lst or {}).get("items") or () if r["n"] == LOC.at(st, key, u, v)), None)
        return key, rec, u, v, row, None
    key = st["newest"] if look in (None, "") else E._key(ex, look)
    lst = st["lists"].get(key) if key is not None else None
    if lst is None:
        return _refuse("no_list", "no numbered list was made yet" if key is None else f"no numbered list was made "
                       f"for {key}", E.fresh_hint(ex))
    why = E.stale(ex, key)
    if why is not None:
        return _refuse("stale_list", f"{why}: its numbers no longer say where things are", E.fresh_hint(ex))
    row = next((r for r in lst["items"] if r["n"] == n), None)
    if row is None:
        have = f"numbers 1 to {len(lst['items'])}" if lst["items"] else "no numbers"
        return _refuse("unknown_candidate", f"the list of {key} has {have}, not {n}",
                       f"pick one of them, or list again: candidates {{\"look\": \"{key}\"}}")
    if row.get("keep_off") and depth != "views":
        return _refuse("forbidden", f"number {n} is kept off: the manual says \"{row['keep_off']}\"",
                       f"pick another number of {key}; choose {{\"n\": {n}, \"look\": \"{key}\", \"depth\": "
                       "\"views\"} only places it, without touching it, so that the robot knows where to keep off")
    return key, looks[key], float(row["point_px"][0]), float(row["point_px"][1]), row, n


def _warning(ex, st, row, part, key, rec, u, v):
    """The manual's colour warning for this pick (eyes_parts.part_warning), or None."""
    if not part:
        return None
    if row is not None:
        return P.part_warning(row, part, st["parts"], colour_words.near)
    img = LOC.frame(rec)
    if img is None or rec.get("can_see") != "ok":
        return None
    colour = LOC.colour_at(img, (u, v))
    entry = P.manual_entry(colour, st["parts"], colour_words.near)
    if entry is None:
        return None
    text = P.part_warning({"n": "at your pixel", "colour": colour, "manual": entry["text"],
                           "manual_part": entry["part"]}, part, st["parts"], colour_words.near)
    return None if text is None else text.replace("number at your pixel", "the spot at your pixel")


# ---------------------------------------------------------------- next
def _unplaced_row(ex, st, key):
    """The number of the first row of this look's list that a forbidden part's word found while no fresh located
    entry carries that part's name (the guard refuses every contact until it is placed), or None."""
    placed = {str(e.get("label") or "").strip().lower() for e in ex.world.query(kind="located", include_stale=False)
              if e.get("point_mm") is not None}
    names = [str(x).lower() for x in st["parts"].get("forbidden") or ()]
    for row in (st["lists"].get(key) or {}).get("items") or ():
        if (key, row.get("n")) in (st.get("place_failed") or ()):
            continue
        if row.get("keep_off") and row.get("found_by") in names and row["found_by"] not in placed:
            return row.get("n")
    return None


def _routine_next(ex, st, key):
    if E.can_run_steps(ex):
        args, needs = {}, []
        if st["parts"].get("cue"):
            args["until"] = "cue changes"
        else:
            needs.append("until: your stop condition in two words")
        if st["parts"].get("forbidden"):
            needs.append("candidates: the numbers you choose")
        return E.ready(ex, "try_each", args, "; ".join(needs) or None)
    if E._has(ex, "assist"):         # no routine can run here: another number of the same list, the model's choice
        return E.ready(ex, "assist", {"pick": "N"}, "N: another number of that list (the manual's colour words speak "
                                                    "against this one)")
    return E.ready(ex, "candidates", {"look": key})


def _next(ex, st, eid, warning, touched, key, n, u, v, part):
    """A whole step only on an entry the guard's target_ok passes; a routine after a warning; else a
    touch (after a touch that ran, whatever it met: a look, never the same touch again)."""
    if warning:
        return _routine_next(ex, st, key)
    guard = _guard(ex)
    ok = False
    if eid is not None and guard is not None and callable(guard.get("target_ok")):
        a = ex.state.get("assist") or {}
        flags = getattr(a.get("gauge"), "flags", None) or {}
        try:
            ok = guard["target_ok"](eid, flags.get(eid) or {}) is None
        except Exception:
            ok = False
    e = ex.world.get(eid) if eid is not None else None
    if ok:
        if E._has(ex, "push"):
            return E.ready(ex, "push", {"target": eid})
        if E._has(ex, "act_on") and E._has(ex, "lift"):
            return E.ready(ex, "act_on", {"on": eid, "verb": "lift"})
        # no whole step is loaded here: the next is the operator's own first move onto
        # the touched point. A move to a standoff, no contact, no force; the direction is left to the operator.
        if e is not None and e.get("point_mm") is not None and E._has(ex, "approach"):
            return E.ready(ex, "approach", {"target": eid}, E.along_needs(ex, e.get("label")))
    if touched:
        return E.ready(ex, "look", {})
    if e is not None and e.get("point_mm") is not None and E._has(ex, "act_on"):
        return E.ready(ex, "act_on", {"on": eid, "verb": "touch"})
    args = {"n": n, "look": key} if n is not None else {"look": key, "px": [round(u, 1), round(v, 1)]}
    if part:
        args["part"] = part
    return E.ready(ex, "choose", {**args, "depth": "touch"})


# ---------------------------------------------------------------- the touch along the line
def _t_sphere(o, d, radius):
    b = float(o @ d)
    disc = b * b - (float(o @ o) - radius * radius)
    if disc < 0:
        return None
    t = -b + math.sqrt(disc)
    return t if t > 0 else None


def _line(ex, rec, u, v):
    """(o, d, t_near, t_end, reach_m) of the line of sight in m, or a refusal: from 60 mm to the nearer of the
    table hit and the edge of the arm's reach."""
    q = np.asarray(rec["q"], dtype=float)
    o, d = ex.model.pixel_ray(q, u, v)
    o, d = np.asarray(o, dtype=float), np.asarray(d, dtype=float)
    reach = float((ex.model.reach() or {}).get("max_reach_mm") or 0.0) / MM
    hit = ex._floor_hit(q, u, v)
    ends = [float(np.linalg.norm(np.asarray(hit) - o))] if hit is not None else []
    if reach > 0:
        t = _t_sphere(o, d, reach)
        if t is not None:
            ends.append(t)
    t_near = L.BRACKET_NEAR_MM / MM
    if not ends or min(ends) <= t_near:
        return _refuse("no_line", "the line of sight of that pixel meets neither the table nor the arm's reach "
                       "in front of the camera", "pick a pixel that looks down at the thing, or depth views")
    return o, d, t_near, min(ends), reach


def _standoff(ex, o, d, t_near, t_end):
    """(distance along the line, joint pose in degrees or None) of the point nearest the camera that reach_check
    accepts: arriving ALONG the line first (the pose it solved is returned: the tool's leading face then points
    down the line), any orientation second (no pose); or None when no point of the line is in reach."""
    ts = np.arange(t_near, max(t_near, t_end - 0.01) + 1e-9, STANDOFF_STEP_MM / MM)
    for approach in ([round(float(x), 4) for x in d], None):
        for t in ts:
            r = ex.reach_check(point_mm=_mm(o + d * t), approach=approach)
            if isinstance(r, dict) and r.get("reachable"):
                return float(t), (r.get("q_deg") if approach is not None else None)
    return None


class _Run:
    """One touch along a line: the gates, the light reading and the bounds before every nested call; the legs."""

    def __init__(self, ex, st):
        self.ex, self.legs, self.notes, self.pieces, self.undo, self.moved = ex, [], [], 0, 0, False
        self.t0 = ex.sim_time()
        session = st.get("session")
        left = None
        try:
            left = float(session.remaining()["sim_seconds_left"]) if session is not None else None
        except Exception:
            left = None
        self.bound_s = None if left is None else TIME_SHARE * left
        if self.bound_s is None:
            self.notes.append("this session has no time budget: the touch has no time bound")

    def call(self, stage, name, **kw):
        ex = self.ex
        if self.bound_s is not None and ex.sim_time() - self.t0 > self.bound_s:
            raise _Stop("bound", why=f"its time bound of {self.bound_s:.0f} s was reached before the {stage}")
        g = ex.gates(name, kw)
        if g is not None:
            raise _Stop("refused", **_body(g))
        if name != "retreat":             # every nested move but a retreat: the light is read before it
            try:
                ex._read_light("inside choose")
                dark = ex._dark_refusal_now()
            except Exception:
                dark = None
            if dark is not None:
                raise _Stop("refused", **_body(dark))
        t1, tip1 = ex.sim_time(), np.asarray(ex._sense().tip, dtype=float)
        out = getattr(ex, name)(**kw)
        t2, tip2 = ex.sim_time(), np.asarray(ex._sense().tip, dtype=float)
        self.moved = self.moved or t2 > t1
        ex.note_event("r3_leg", by="runtime", tool="choose", stage=stage, primitive=name, t0=round(t1, 3),
                      t1=round(t2, 3))
        out = out if isinstance(out, dict) else {}
        leg = {"leg": stage, "stopped": "refused" if "refused" in out else out.get("stopped"),
               "mm": round(float(np.linalg.norm(tip2 - tip1)) * MM, 1), "seconds": round(t2 - t1, 2)}
        if self.legs and self.legs[-1]["leg"] == stage:       # the pieces of one long touch read as one leg
            last = self.legs[-1]
            last.update(stopped=leg["stopped"], seconds=round(last["seconds"] + leg["seconds"], 2),
                        mm=round(last["mm"] + leg["mm"], 1))
        else:
            self.legs.append(leg)
        if "refused" in out:
            raise _Stop("refused", **_body(out))
        return out

    def back_off(self, d, felt_mm):
        """Every exit (4.7 rule 8): BACK_MM back along the line the tool came down (a guarded move_tip); when that
        is refused (the light went dark: retreat still runs) or the tool never felt along the line, the newest
        paths are retraced by retreat. (tip mm, backed off or not)."""
        ex = self.ex
        if self.undo > 0:
            done = False
            if self.pieces > 0:
                try:
                    back = -np.asarray(d, dtype=float) * BACK_MM
                    r = self.call("back off", "move_tip", dx=round(float(back[0]), 2), dy=round(float(back[1]), 2),
                                  dz=round(float(back[2]), 2), frame="base", guarded=True)
                    done = r.get("stopped") == "goal"
                except _Stop:
                    done = False
                except Exception as exc:
                    self.notes.append(f"the back-off failed ({type(exc).__name__})")
            if not done:
                try:
                    self.call("back off", "retreat", steps=int(self.undo))
                except _Stop as stop:
                    self.notes.append(f"the back-off did not run ({stop.info.get('check') or stop.stopped})")
                except Exception as exc:      # the tool must still be reported where it is
                    self.notes.append(f"the back-off failed ({type(exc).__name__})")
        s = ex._sense()
        tip = _mm(s.tip)
        gap = None if felt_mm is None else float(np.linalg.norm(np.asarray(tip) - np.asarray(felt_mm)))
        eased = float(getattr(s, "Fmag", 0.0) or 0.0) < float(ex.cfg.get("eased_force_n") or 1.0)
        return tip, bool(eased and (gap is None or gap >= BACKED_OFF_MM))


def _touch(ex, st, key, rec, u, v, what):
    """The contact along the line of sight (see the module text): a dict {stopped, contact, felt (the touch's
    answer), ran_mm, legs, end, notes, why, check} or a refusal (nothing moved)."""
    got = _line(ex, rec, u, v)
    if isinstance(got, dict):
        return got
    o, d, t_near, t_end, reach = got
    got = _standoff(ex, o, d, t_near, t_end)
    if got is None:
        return _refuse("out_of_reach", f"no point of the line of sight of {what} is in the arm's reach",
                       "depth views measures it without touching; or look from nearer and list again")
    t_s, pose = got
    standoff, end = o + d * t_s, o + d * (t_end + END_EXTRA_MM / MM)
    unc = POINT_ERR_PX * t_end / float(ex.model.f_px) * MM + 1.0
    guard = _guard(ex)
    banned = list(st["parts"].get("forbidden") or ())
    if guard is not None and callable(guard.get("keep_off")):
        bad = guard["keep_off"](_mm(standoff), _mm(end), round(unc, 1))
        if bad:
            b = _body(bad)
            return {"said": b["why"], "refused": b}
    elif banned:
        return _refuse("not_guarded", f"the manual forbids a part ({', '.join(banned)}) and nothing here keeps a "
                       "touch the robot makes on its own off it", "depth views measures it without touching; or "
                       "touch it yourself")
    run = _Run(ex, st)
    tip0 = np.asarray(ex._sense().tip, dtype=float)
    move_mm = float(np.linalg.norm(standoff - tip0)) * MM
    line_mm = (t_end - t_s) * MM + END_EXTRA_MM
    if reach > 0 and move_mm + line_mm > reach * MM:
        line_mm = reach * MM - move_mm
        run.notes.append("the touch was cut short by its travel bound (the reach radius)")
    out = {"stopped": None, "contact": False, "felt": None, "ran_mm": 0.0, "check": None, "why": ""}
    try:
        if line_mm < 1.0:
            raise _Stop("bound", why="its travel bound (the reach radius) leaves no room for the touch")
        if move_mm > 1.0:
            r = None
            if pose is not None:      # the pose reach_check solved: the tool's leading face points down the line
                try:
                    r = run.call("standoff", "move_joints", q_deg=pose)
                except _Stop as stop:     # the way to that pose is not free (a path check): the straight move instead
                    if stop.stopped != "refused" or not str(stop.info.get("check") or "").startswith("path"):
                        raise
                    run.notes.append(f"the joint move to the start of the line was refused ({stop.info['check']})")
            if r is None:
                dv = (standoff - tip0) * MM
                r = run.call("standoff", "move_tip", dx=round(float(dv[0]), 2), dy=round(float(dv[1]), 2),
                             dz=round(float(dv[2]), 2), frame="base", guarded=True)
                run.notes.append("the tool may not point down the line: it kept the turn it had")
            run.undo = 1
            if r.get("stopped") != "goal":
                raise _Stop("blocked", why=f"the move to the start of the line stopped ({r.get('stopped')})")
        piece = max(1.0, (L.TOUCH_PIECE_S - PIECE_MARGIN_S) * float(ex._touch_speed(ex.cfg["touch_force_n"])) * MM)
        left = line_mm
        direction = [round(float(x), 4) for x in d]
        while left > 0.5:
            r = run.call("touch", "touch", direction=direction, max_mm=round(min(left, piece), 1), frame="base")
            run.pieces += 1
            run.undo += 1
            gone = float(r.get("travelled_mm") or 0.0)
            out["ran_mm"] += gone
            if r.get("contact"):
                out["felt"] = r
                break
            if r.get("stopped") in ("goal", "time") and left - gone > 1.0 and gone > 0.5:
                left -= gone
                continue
            out["why"] = str(r.get("why") or "")
            break
        felt = out["felt"]
        if felt is None:
            out["stopped"] = "not_found"
        elif felt.get("stopped") == "cap":
            out.update(stopped="harm_guard", check="cap", why=str(felt.get("why") or "the touch met its force cap"))
        elif felt.get("source") not in (None, "tool") or felt.get("point_mm") is None:
            out.update(stopped="blocked", why="a link of the arm met something, not the tool: the place is not "
                                              "measured")
        else:
            bad = guard["felt"](felt["point_mm"], TOUCH_UNC_MM) if guard is not None and callable(
                guard.get("felt")) else None
            if bad:
                out.update(stopped="harm_guard", check=_body(bad)["check"], why=_body(bad)["why"])
            else:
                out.update(stopped="contact", contact=True)
                if felt.get("at_tip") is False:
                    run.notes.append(f"the tool's {felt.get('surface') or 'side'} met it, not its tip: the point "
                                     "may lie beside the line")
    except _Stop as stop:
        out.update(stopped=stop.stopped, check=stop.info.get("check"), why=str(stop.info.get("why") or ""))
    finally:
        felt_mm = (out["felt"] or {}).get("point_mm") if isinstance(out.get("felt"), dict) else None
        tip, backed = run.back_off(d, felt_mm)
        out.update(legs=run.legs, notes=run.notes, end={"tip_mm": tip, "backed_off": backed}, moved=run.moved,
                   ray={"origin_mm": _mm(o), "dir": [round(float(x), 4) for x in d]})
    return out


def _touch_said(t, what, eid, label, at_table):
    end = "" if t["end"]["backed_off"] else "the tool could NOT be backed off: "
    tail = "; backed off" if t["end"]["backed_off"] else ""
    if t["stopped"] == "contact":
        p = t["felt"]["point_mm"]
        said = (f"{what} met a surface at x {p[0]:.0f}, y {p[1]:.0f}, z {p[2]:.0f} mm after {t['ran_mm']:.0f} mm "
                f"along its line of sight; stored as {eid} ({label})")
        if at_table:
            said += "; that is table height: the table or a thing flat on it"
    elif t["stopped"] == "not_found":
        said = (f"the touch ran {t['ran_mm']:.0f} mm along the line of sight of {what} and felt nothing: no point "
                "stored")
    elif t["stopped"] == "refused":
        said = ("nothing moved" if not t["moved"] else "the touch stopped") + f" ({t['check']}): {t['why']}"
    elif t["stopped"] == "harm_guard":
        said = f"the touch along the line of {what} stopped ({t['check']}): {t['why']}; no point stored"
    else:
        said = (f"the touch along the line of {what} did not run to its end ({t['stopped']}): {t['why']}; no point "
                "stored")
    return P.words_of(end + said + tail, E.SAID_WORDS)


# ---------------------------------------------------------------- the located entry
def _ray_entry(ex, key, rec, u, v, label):
    """A located entry with the line of sight alone (depth ray_only), when locate is not loaded."""
    o, d = ex.model.pixel_ray(np.asarray(rec["q"], dtype=float), u, v)
    data = {"uncertain_mm": None, "depth": "ray_only", "ray": {"origin_mm": _mm(o), "dir": [round(float(x), 4)
                                                                                       for x in d]},
            "naming_look": key, "looks": [key], "device_centre_mm": None, "device_top_mm": None, "candidate": None,
            "part_check": None, "normal": None, "template": None, "epoch": int(getattr(ex, "epoch", 0))}
    return ex.world.add("located", label, point_mm=None, source="choose", data=data)


def _mark(ex, eid, key, n, row, label, part, named, warning, extra=None):
    """The entry's own record of the pick: data.candidate, data.outline_px; unconfirmed when no part names it."""
    e = ex.world.get(eid) or {}
    old = (e.get("data") or {}).get("candidate")
    data = {"candidate": f"eyes:{key}:{n}" if n is not None else None,
            "outline_px": [[round(float(a), 1), round(float(b), 1)] for a, b in (row or {}).get("polygon_px") or ()]
            or None, "chosen": {"n": n, "look": key}}
    if old is not None and not str(old).startswith("eyes:"):
        data["survey_candidate"] = old
    if warning:
        data["part_check"] = warning
    if not named:
        data.update(unconfirmed=True, asked=part)
    data.update(extra or {})
    fields = {"data": data}
    if not named:
        fields["label"] = label
    return ex.world.update(eid, **fields)


def _fit(out):
    """A brief result within 900 bytes besides its pictures: the least needed keys go first."""
    def size():
        return len(json.dumps({k: v for k, v in out.items() if k not in ("pictures", "full")}))
    for key in ("notes", "legs", "identity", "part_check"):
        if size() <= E.BRIEF_MAX:
            break
        if key == "identity" and isinstance(out.get(key), dict):
            out[key] = {"verified": out[key].get("verified")}
        elif key == "part_check" and out.get(key):
            out[key] = str(out[key])[:120]
        elif key in out:
            out.pop(key)
    return out


def run(ex, n=None, part=None, look=None, px=None, depth="auto", detail="brief", unconfirmed=False):
    """The choose primitive's body (rcpx/r2/eyes.py wraps it)."""
    st = E._state(ex)
    if depth not in ("auto", "views", "touch") or detail not in ("brief", "full"):
        return _refuse("args", "depth is auto, views or touch; detail is brief or full", "leave both out")
    if part is not None and not (isinstance(part, str) and 0 < len(part.strip()) <= 80):
        return _refuse("args", "part is a part's name from robot://devices", "leave it out when you do not know it")
    has_px = isinstance(px, (list, tuple)) and len(px) == 2 and all(
        isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in px)
    if n is not None and (isinstance(n, bool) or not isinstance(n, int) or n < 1):
        return _refuse("args", "n is a candidate's number, 1 or more", 'for example choose {"n": 2}')
    if n is None and not (has_px and look not in (None, "")):
        return _refuse("args", "give n (a number of a list) or look and px [u, v] (your own pixel)",
                       'for example choose {"n": 2}; candidates {} makes the list')
    got = _target(ex, st, n, look, px, depth)
    if isinstance(got, dict):
        return got
    key, rec, u, v, row, n = got
    q_look = np.asarray(rec["q"], dtype=float)
    notes = []
    body = LOC.on_own_body(ex, q_look, u, v)
    if body in BODY_WORDS:
        return _refuse("own_body", f"pixel ({u:.0f}, {v:.0f}) in {key} shows {BODY_WORDS[body]}, not a thing in "
                       "the world", "candidates {} numbers the things that stand apart from the robot")
    if body == "unchecked":
        notes.append("own body not checked")
    asked = part.strip().lower() if isinstance(part, str) else None
    named = None if unconfirmed else P.default_part(row, st["parts"], asked)
    kept = bool(row and row.get("keep_off"))      # depth views only (_target): the part is placed, never touched
    if kept:
        asked, named = None, (row.get("found_by") or "kept off")
    what = f"number {n}" if n is not None else f"your pixel ({u:.0f}, {v:.0f})"
    label = named or (f"candidate {n}" if n is not None else "candidate")
    warning = _warning(ex, st, row, named or asked, key, rec, u, v)
    out = {"said": "", "located": None, "point_mm": None, "uncertain_mm": None, "depth": None,
           "chosen": {"n": n, "look": key}, "part_check": warning}
    eid, said, pictures, touched, unplaced, on_table = None, "", [], False, None, False
    can_locate = callable(getattr(ex, "locate", None)) and isinstance(ex.state.get("sight"), dict)
    if depth in ("auto", "views"):
        if can_locate:
            args = {"look": key, "px": [u, v], "part": named or "other", "note": "" if named else label}
            gate = ex.gates("locate", args)
            if gate is not None:
                b = _body(gate)
                return {"said": b["why"], "refused": b}
            r = ex.locate(**args)
            if not isinstance(r, dict) or "refused" in r:
                b = _body(r)
                return {"said": b["why"], "refused": b}
            eid = r.get("id")
            _mark(ex, eid, key, n, row, label, asked, named, warning)
            said = str(r.get("said") or "").replace(f"the other: {label}", label)
            pictures = list(r.get("pictures") or [])[:1]
            out.update(located=eid, point_mm=r.get("point_mm"), uncertain_mm=r.get("uncertain_mm"),
                       depth=r.get("depth"))
            if r.get("notes"):
                notes += list(r["notes"])
        else:
            # no locate tool: the robot takes a second look from the side itself and triangulates (eyes_views)
            two = VIEWS.run(ex, st, E.list_, key, rec, u, v, row or {}, n, label, kept)
            notes += list(two.get("notes") or [])
            if two.get("id"):
                eid = two["id"]
                _mark(ex, eid, key, n, row, label, asked, named, warning,
                      {"depth": "views", "uncertain_mm": two["uncertain_mm"], "naming_look": key,
                       "looks": [key, two["look2"]], "epoch": int(getattr(ex, "epoch", 0))})
                said = (f"{what} seen again as number {two['n2']} of {two['look2']} from "
                        f"{two['moved_mm']:.0f} mm to the side: the two lines of sight meet at x "
                        f"{two['point_mm'][0]:.0f}, y {two['point_mm'][1]:.0f}, z {two['point_mm'][2]:.0f} mm "
                        f"(within {two['uncertain_mm']:.0f} mm); stored as {eid} ({label}); "
                        + ("the robot keeps off it from here on; nothing was touched" if kept
                           else "not checked by touch"))
                out.update(located=eid, point_mm=two["point_mm"], uncertain_mm=two["uncertain_mm"], depth="views",
                           legs=two["legs"])
            elif depth == "views":
                if kept:                   # this row could not be placed: it is not offered for placing again
                    st.setdefault("place_failed", set()).add((key, n))
                if isinstance(two.get("refused"), dict):
                    return {"said": str(two["refused"].get("why")), "refused": two["refused"]}
                return _refuse("no_second_view", str(two.get("none")), "look at it from another place, list again "
                               "(candidates {}) and place it from that list" if kept else "depth touch feels "
                               "along the line of sight instead")
            else:
                notes.append(f"no point from two views ({str(two.get('none') or (two.get('refused') or {}).get('check'))[:110]})")
        if eid is None and depth != "views" and not can_locate:
            e = _ray_entry(ex, key, rec, u, v, label)
            eid = e.get("id")
            _mark(ex, eid, key, n, row, label, asked, named, warning)
            said = (f"stored the line of sight to {what} as {eid} ({label}), depth unknown: no tool here measures "
                    "it from pictures")
            out.update(located=eid, depth="ray_only")
            notes.append("locate is not loaded: depth from pictures is not available")
    whole = E._has(ex, "push")       # only push feels along a line of sight itself; without it the touch is made here
    if depth == "touch" or (depth == "auto" and out["depth"] in (None, "ray_only") and not whole):
        t = _touch(ex, st, key, rec, u, v, what)
        if "refused" in t:
            place = _unplaced_row(ex, st, key) if t["refused"].get("check") == "forbidden_unplaced" else None
            if depth == "touch" or eid is None:
                if place is not None:        # the call that places the part the robot must keep off, no contact
                    t["refused"]["hint"] = (f"choose {{\"n\": {place}, \"look\": \"{key}\", \"depth\": \"views\"}} "
                                            "places that part without touching it; then pick again")
                    t["next"] = E.ready(ex, "choose", {"n": place, "look": key, "depth": "views"})
                return t
            notes.append(f"no touch was made ({t['refused']['check']}): {t['refused']['why']}"[:160])
            unplaced = place
        else:
            touched = True                 # a touch ran: the next call is never the same touch again
            at_table = False
            if t["stopped"] == "contact":
                p = t["felt"]["point_mm"]
                floor_mm = float(ex._floor_at(np.asarray(p, dtype=float) / MM)) * MM
                at_table = on_table = abs(float(p[2]) - floor_mm) <= TABLE_TOL_MM
                extra = {"depth": "touch", "uncertain_mm": TOUCH_UNC_MM, "ray": t["ray"], "naming_look": key,
                         "looks": [key], "contact": t["felt"].get("id"), "normal": t["felt"].get("normal_est"),
                         "at_table": at_table, "epoch": int(getattr(ex, "epoch", 0))}
                if eid is None:
                    eid = ex.world.add("located", label, point_mm=p, source="choose", data=extra).get("id")
                    _mark(ex, eid, key, n, row, label, asked, named, warning)
                else:
                    ex.world.update(eid, point_mm=p, data=extra)
                out.update(located=eid, point_mm=p, uncertain_mm=TOUCH_UNC_MM, depth="touch")
            said = _touch_said(t, what, eid, label, at_table)
            out = {"said": said, "stopped": t["stopped"], "contact": bool(t["contact"]), **{
                k: v for k, v in out.items() if k != "said"}}
            if t.get("check"):
                out["check"] = t["check"]
            out.update(ran_mm=round(t["ran_mm"], 1), legs=t["legs"], end=t["end"])
            notes += t["notes"]
    e = ex.world.get(eid) if eid is not None else None
    ident = ((e or {}).get("data") or {}).get("identity")
    if isinstance(ident, dict):
        out["identity"] = {"verified": bool(ident.get("verified")), "why": str(ident.get("why") or "")[:100]}
    if not named and eid is not None and "stored" not in said:
        said = P.words_of(f"{said}; stored as {eid} ({label}): name the part when you know it", E.SAID_WORDS)
    out["said"] = said
    if notes:
        out["notes"] = [str(x)[:160] for x in notes[:3]]
    if detail == "full":
        out["full"] = {"row": None if row is None else dict(row), "entry": e}
    else:
        _fit(out)
    out["pictures"] = pictures
    if kept or on_table or unplaced is not None:
        # placed a part to keep off; or the line met the table, not a part; or a part to keep off is not placed yet
        out["next"] = (E.ready(ex, "choose", {"n": unplaced, "look": key, "depth": "views"}) if unplaced is not None
                       else E.ready(ex, "assist", {"pick": "N"}, "N: the number of the part you act on" if kept else
                                    "N: another number of that list (this one met the table, not a part)")
                       if E._has(ex, "assist") else E.ready(ex, "candidates", {"look": key}))
    else:
        out["next"] = _next(ex, st, eid, warning, touched, key, n, u, v, named or asked)
    return out


__all__ = ["run"]
