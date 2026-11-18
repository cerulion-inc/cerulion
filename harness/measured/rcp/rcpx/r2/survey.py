# SPDX-License-Identifier: AGPL-3.0-only
"""Package survey: search for something that is outside the standard look, in one call.

The standard look sees only the floor patch under the hand, and operators spent 8 to
19 calls per episode turning the arm by hand to find a device placed elsewhere. survey() does that search:

- the views: the standard look turned about the base's vertical axis, a ring around the robot from the body model
  (rcpx/r2/survey_plan.py), the step half a view's angular width so neighbouring views overlap by half, the span the
  first joint's whole turn, as far as the arm's own path check finds it free, unless span_deg narrows it;
- the motion: one guarded joint move to the first view (path checked against the surface under the arm and the arm
  itself, as standard_look's move; when that first leg is refused before it runs the arm retreats along its own
  history once and the leg is asked again, as standard_look does), then legs from view to view, each path checked
  before it runs, every leg the runtime's own look move at the session's derived speed with the call's intent (the
  operator's word clear), the contact stop armed; a contact stop or a refused leg
  ends the survey with the views taken so far; it never moves after a look that reads dark;
- the looks: one round one look per view (so mark, crop and triangulate take its look id), each with the floor
  patch it covers in base mm (survey_see.floor_view);
- the candidates: regions that do not lie on the floor plane in two neighbouring views (plane-sweep stereo, from
  the frames and the body model only: survey_see.py), merged by floor position: an id (c1, c2, ...), where on the
  floor (base mm), how high it stands, its size, and every look and pixel box it shows in. They are drawn on one
  frame per view (frame); names never: the survey does not know what anything is;
- the result is brief: one line per view and per candidate; detail "full" adds the camera pose, the joint pose
  and each candidate's region heights per view.

Dark: the call is a move toward the world, so the waiting package's dark gate refuses it while the newest light
reading is dark; without that package the round one dark mark refuses it (check dark), and the survey stops at the
first dark view. Nothing here reads simulator ground truth or names a robot or a device.
"""

import json
import math

import numpy as np

from rcpx import bounds
from rcpx import surface_brief
from rcpx import surfaces as S
from rcpx.context import refusal
from rcpx.control_base import _mm, primitive
from rcpx.r2 import survey_plan as plan_mod
from rcpx.r2 import survey_see as see

MM = 1000.0  # metres to millimetres
ID = "I32"
MAX_CANDIDATES = 8          # candidates listed (the largest, seen in most views); the rest are counted
HOW = ("sheet: every view in order, labelled with its look id and bearing, candidates boxed; floor_mm: the floor "
       "corners each view covers (base mm; null above the horizon); candidates: things that do not lie on the floor "
       "in two views, objects or a moving shadow; where_mm and top_mm are two-view estimates: triangulate or touch "
       "before a stroke")
MOUNT_MARGIN_M = 0.02       # a candidate standing on the robot's own mount plate (this close) is the robot's own


def _state(ex):
    return ex.state.setdefault("survey", {"runs": 0, "last": None})


def _r(x, nd=1):
    return round(float(x), nd) + 0.0


class Mixin:
    @primitive
    def survey(self, span_deg=None, max_views=None, max_s=None, detail="brief", clear=None):
        if detail not in ("brief", "full"):
            return refusal("detail", f"detail is brief or full, not {str(detail)[:20]!r}", "leave it out")
        if span_deg is not None:
            ok = isinstance(span_deg, (list, tuple)) and len(span_deg) == 2 and all(
                isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in span_deg)
            if not ok:
                return refusal("span_deg", "span_deg is [from, to]: two bearings in degrees in the base frame "
                               "(0 = forward, + = left)", "leave it out to survey the arm's whole turn")
        if max_views is not None and (isinstance(max_views, bool) or not isinstance(max_views, int)
                                      or not 2 <= max_views <= plan_mod.MAX_VIEWS):
            return refusal("max_views", f"max_views is a whole number from 2 to {plan_mod.MAX_VIEWS}",
                           "leave it out: the step is chosen so neighbouring views overlap by half")
        if max_s is not None and (isinstance(max_s, bool) or not isinstance(max_s, (int, float))
                                  or not math.isfinite(max_s) or max_s <= 0):
            return refusal("max_s", "max_s is a positive finite number of world seconds",
                           "leave it out for the whole planned sweep")
        intent, why = plan_mod.clear_word(clear)
        if why:
            return refusal("args", why, "leave clear out, or say true after a look")
        budget = _budget_left_s(self)
        limits = [(max_s, "your max_s"), (budget, "the seconds left in the session's budget")]
        limit, source = min(((v, text) for v, text in limits if v is not None), default=(None, ""))
        dark = self._dark_refusal_now()
        if dark is not None:
            return {**dark, "stopped": "refused"}
        p = plan_mod.plan(self, span_deg, max_views or plan_mod.MAX_VIEWS,
                          keep_overlap=span_deg is None and max_views is None)
        if "views" not in p:
            return {**refusal("survey_plan", p["why"], "take a look from where the arm is instead"),
                    "stopped": "refused"}
        if not p["views"]:
            return {**refusal("survey_plan", "no survey view could be solved: " + "; ".join(p["notes"]),
                              "take a look from where the arm is instead"), "stopped": "refused"}
        views, free_notes = plan_mod.free_views(self, p["views"], self._sense().q)
        p["notes"].extend(free_notes)
        if not views:
            return refusal("survey_plan", "; ".join(free_notes), "look from where the arm is")
        kept, planned_s, all_s = plan_mod.fit_time(self, views, self._sense().q, limit, intent)
        if len(kept) < len(views):
            done = [v["bearing_deg"] for v in kept]
            p["notes"].append(f"{source} ({limit:g} s) cuts the sweep to {len(kept)} views, bearings "
                              f"{min(done):g} to {max(done):g} deg; all views take about {all_s:.0f} s")
        p["views"], p["planned_s"] = kept, planned_s
        return _run(self, p, detail, intent)


def _move(ex, q, first=False, intent=None):
    """The runtime's own look move to a survey pose (kind look, the derived speed, the call's intent): the core's
    stop dict or a refusal. The first leg may retreat along the arm's own history once when its path is refused
    before it runs (as standard_look's move does); a leg that stopped on the way is not tried again."""
    r = plan_mod.look_move(ex, q, intent)
    if first and "refused" in r and ex._history:
        ex._retreat_impl()
        r = plan_mod.look_move(ex, q, intent)
    return r


def _run(ex, p, detail, intent=None):
    st = _state(ex)
    st["runs"] += 1
    z = float(ex.cfg.get("floor_z_m") or 0.0)
    t0 = ex.sim_time()
    moves = []
    views = p["views"]
    n = len(views)
    taken, stopped, why, contact = [], "done", "", None
    for k, v in enumerate(views):
        r = _move(ex, v["q"], k == 0, intent)
        moves.append(r)
        if "refused" in r:
            if k == 0:
                out = {**r, "stopped": "refused", "views": [], "candidates": []}
                return plan_mod.report(ex, out, moves, t0, detail) if ex.sim_time() > t0 else out
            stopped, why = "refused", f"the leg to view {k + 1} was refused: {r['refused']['why']}"
            break
        if r["stop"] != "goal":
            stopped, contact = r["stop"], r.get("contact")
            why = f"the move to view {k + 1} stopped ({r['stop']}){': ' + r['why'] if r.get('why') else ''}"
            break
        ex._ease_impl()
        lk = ex._look_impl(f"survey view {k + 1} of {n}", keep_image=True)
        rec = ex._looks[lk["look_id"]]
        img = rec.get("img")
        rec["img"] = None            # the frame file stays; the survey keeps its own copy for the comparison
        taken.append({"look_id": lk["look_id"], "q": np.asarray(rec["q"], dtype=float).copy(), "img": img,
                      "bearing_deg": v["bearing_deg"], "can_see": lk["can_see"], "brightness": lk["brightness"],
                      "look": lk})
        if lk["can_see"] == "dark":
            stopped, why = "dark", f"view {k + 1} reads dark: the survey does not move on in the dark"
            break
    cands, dropped = _candidates(ex, taken, z)
    out = _result(ex, p, taken, cands, dropped, n, stopped, why, contact, t0, detail, z)
    st["last"] = {"looks": [t["look_id"] for t in taken], "candidates": len(cands),
                  # each listed candidate's own two-view error [xy, z] mm, for a package that keeps the boxes
                  "err_mm": {c["id"]: [_r(v * MM) for v in c["err_m"]] for c in cands if c.get("id")}}
    return plan_mod.report(ex, out, moves, t0, detail) if ex.sim_time() > t0 else out


def _candidates(ex, taken, z):
    """Merged candidates over the views taken (lit ones), and how many regions were left out as the robot's own."""
    lit = [t for t in taken if t["can_see"] != "dark" and t["img"] is not None]
    if len(lit) < 2:
        return [], {"mount": 0, "rides_along": 0}
    for t in lit:
        t["prep"] = see._prep(t["img"])
        t["mask"] = see.robot_mask(ex.model, t["q"])
    raw = [see.regions(ex.model, lit, k, z) for k in range(len(lit))]
    kept = see.rides_along(raw)
    rides = sum(len(a) - len(b) for a, b in zip(raw, kept))
    regs = [r for rs in kept for r in rs]
    half = float(ex.cfg.get("mount_half_m") or 0.0)
    own = [r for r in regs if half > 0 and max(abs(float(r["where_m"][0])), abs(float(r["where_m"][1])))
           <= half + MOUNT_MARGIN_M]
    regs = [r for r in regs if r not in own]
    return see.merge(regs), {"mount": len(own), "rides_along": rides}


def _result(ex, p, taken, cands, dropped, n, stopped, why, contact, t0, detail, z):
    full = detail == "full"
    ids = {}
    listed = cands[:MAX_CANDIDATES]
    for i, c in enumerate(listed):
        c["id"] = f"c{i + 1}"
        for s in c["seen_in"]:
            ids.setdefault(s["look_id"], []).append((c["id"], s["box_px"]))
    _to_book(ex, listed, taken, z)
    views, tiles = [], []
    for k, t in enumerate(taken):
        boxes = ids.get(t["look_id"], [])
        frame = t["look"]["frame"]
        if t["img"] is not None:
            tile = see.draw(t["img"], k, n, t["bearing_deg"], boxes, t["look_id"])
            tiles.append(tile)
            if full:
                frame = ex._save(tile, f"{t['look_id']}_survey")
        fv = see.floor_view(ex.model, t["q"], z)
        row = {"look_id": t["look_id"], "bearing_deg": t["bearing_deg"], "floor_mm": fv["corners"]}
        if boxes:
            row["candidates"] = [b[0] for b in boxes]
        if t["can_see"] != "ok":
            row["can_see"] = t["can_see"]
        if full:
            row.update(frame=frame, floor_centre_mm=fv["centre"], camera_pose=ex._camera_pose(t["q"]),
                       q_deg=[_r(math.degrees(x), 2) for x in t["q"]], raw_frame=t["look"]["frame"],
                       brightness=t["brightness"], can_see=t["can_see"])
        views.append(row)
    sheet_img = see.sheet(tiles)
    sheet = None if sheet_img is None else ex._save(sheet_img, f"survey{_state(ex)['runs']}_sheet")
    out_c = []
    for c in listed:
        w = c["where_m"]
        e = {"id": c["id"], "where_mm": [int(round(w[0] * MM)), int(round(w[1] * MM))],
             "bearing_deg": _r(math.degrees(math.atan2(w[1], w[0])), 0),
             "distance_mm": int(round(math.hypot(w[0], w[1]) * MM)),
             "top_mm": int(round((z + c["top_m"]) * MM)),
             "size_mm": [int(round(c["size_m"][0] * MM)), int(round(c["size_m"][1] * MM))],
             "seen_in": {s["look_id"]: s["box_px"] for s in c["seen_in"]}}
        if full:
            e.update(err_mm=[_r(v * MM) for v in c["err_m"]], height_above_floor_mm=int(round(c["top_m"] * MM)),
                     median_height_mm=int(round(c["height_m"] * MM)), area_px=c["area_px"])
        out_c.append(e)
    s = ex._sense()
    out = {"stopped": stopped, "sheet": sheet, "views": views, "candidates": out_c,
           "views_taken": len(taken), "span_deg": p["span_deg"], "tip_mm": _mm(s.tip),
           "took_s": _r(ex.sim_time() - t0, 1)}
    if len(cands) > len(listed):
        out["more_candidates"] = len(cands) - len(listed)
    if dropped["mount"]:
        out["left_out"] = f"{dropped['mount']} regions on the robot's own mount plate"
    if why:
        out["why"] = why
    if contact is not None:
        out["contact"] = contact
    if p["notes"]:
        out["notes"] = p["notes"]
    if stopped == "done":
        out["next"] = ("the sheet shows the candidates; marking one in two of its looks and triangulating costs 0 "
                       "control periods" if out_c else
                       "no raised thing was found on the floor the views cover (a flat or textureless thing can be "
                       "missed); span_deg covers other bearings")
    elif stopped == "dark":
        out["next"] = "it is dark: the survey stopped there; a look (0 control periods) reads the light again"
    else:
        out["next"] = "the survey stopped early: the views it took are in the sheet (0 control periods to read)"
    if len(taken) < n:
        out["views_planned"] = n
    if not full and len(json.dumps(out, default=str)) > surface_brief.LIMIT:
        for row in views:                   # a long sweep stays brief: the floor corners of each view are in full
            row.pop("floor_mm", None)
        out["notes"] = list(out.get("notes") or []) + ["the floor corners each view covers are in detail full"]
    if full:
        out.update(planned_s=_r(p.get("planned_s", 0.0), 1), how=HOW)
        out.update(method=p["method"], width_deg=p["width_deg"], full_turn=p["full_turn"], step_deg=p["step_deg"],
                   standard_bearing_deg=p["standard_bearing_deg"], rides_along_dropped=dropped["rides_along"])
    return out


def setup(ex):
    _state(ex)
    w = ex.state.get("waiting")
    if isinstance(w, dict) and isinstance(w.get("toward_world"), set):
        w["toward_world"].add("survey")      # the waiting package's dark gate covers it (it lists it already too)


TOOLS = [
    S._tool(
        "survey",
        "Search the floor around the robot for something not in the standard look, in one call: the arm turns the "
        "standard look about its base through overlapping views (guarded, path checked; the first joint's whole turn, "
        "as far as the arm's own path check finds it free, unless span_deg narrows it; bounded by your max_s or the "
        "session's stated seconds left), takes a look at each and "
        "returns one sheet image of every view, per view its look id, bearing and the floor corners it covers "
        "(base mm), and candidates: raised things seen in two views, each with where_mm on the floor, top_mm, size "
        "and the look ids and pixel boxes it shows in (boxed on the sheet). It names nothing; it stops at a contact, "
        "a refused leg or a dark view. where_mm and top_mm are two-view estimates (a moving shadow can make a "
        "candidate too); a triangulation or a touch measures one. The looks work with mark, crop and triangulate.",
        S._obj({"span_deg": {"type": "array", "items": {"type": "number", "minimum": -360, "maximum": 360},
                             "minItems": 2, "maxItems": 2,
                             "description": "[from, to] bearings in base degrees to cover (0 forward, + left; "
                                            "[-180, 180] is the whole turn); default: the first joint's whole turn, "
                                            "as far as the arm's own path check finds it free"},
                "max_views": S._int(2, plan_mod.MAX_VIEWS, "most views to take (default: enough to see each floor "
                                                           "point three times, at most 16)"),
                "max_s": {"type": "number", "description": "the most world seconds the sweep may take, if you want "
                          "to bound it (no default; the seconds left in the session's budget bound it when one "
                          "is stated): the legs are planned first and the sweep ends where the time runs out"},
                "clear": plan_mod.CLEAR_ARG,
                "detail": S._str("brief (default) or full: full adds a boxed frame per view, camera and joint poses",
                                 ["brief", "full"])}),
        {}, "turn the standard look around the base and list the raised things it sees, with where they are",
        brief="Search the floor all round in one call: overlapping views, each floor patch, raised candidates."),
]

PRACTICES = ""

__all__ = ["ID", "Mixin", "PRACTICES", "TOOLS", "setup", "FILTER"]


def FILTER(surface, resources, handlers, ex=None, box=None):
    if ex is not None and box is not None:
        _state(ex)["box"] = box
    return surface, resources, handlers


def _budget_left_s(ex):
    """Only the session's stated world-time budget bounds the sweep when no max_s is given."""
    box = _state(ex).get("box")
    session = getattr(box, "session", None)
    remaining = getattr(session, "remaining", None)
    return remaining().get("sim_seconds_left") if callable(remaining) else None


def _to_book(ex, listed, taken, z):
    """The survey's boxes into the session's space book: each listed candidate as a box with sigma =
    its own two-view error (err_m; nothing is grown by a typed margin). The looks themselves are the core's looks
    (ex._look_impl): the core hands each one to the session's seeing, so the survey does not feed
    them a second time."""
    space = bounds.of(ex).space
    if hasattr(space, "add_thing"):
        for c in listed:
            where, half = np.asarray(c["where_m"], dtype=float), np.asarray(c["size_m"], dtype=float) / 2.0
            lo, hi = where - half, where + half
            shape = {"box": [[float(lo[0]), float(lo[1]), z], [float(hi[0]), float(hi[1]), z + float(c["top_m"])]]}
            looks = ", ".join(s["look_id"] for s in c["seen_in"])
            space.add_thing(c["id"], shape, float(max(c["err_m"])), "seen: survey looks " + looks)
