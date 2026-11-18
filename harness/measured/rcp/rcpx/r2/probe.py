# SPDX-License-Identifier: AGPL-3.0-only
"""Package "probe": learn the device by probing, and keep what was learned, never a pose.

Tools:
- probe_motion(target, direction, max_mm, force_n): with the tip at a part, push it in measured force-limited
  steps whose command is reset to the measured pose across the push line (probe_push.py), so the part, not the
  command, decides the path; fit a line and a circle to the tip path (probe_fit.py) and pick by residual
  (probe_report.py). Returns {kind: prismatic | revolute | stuck | free | unknown, axis_dir, center_mm, radius_mm,
  travel_mm, residual_mm, peak_force_n, path_mm, base_moved, motion_dir, why, ...}. The fit goes to the world as a
  device entry with source "probe", confirmed false, and to ex.state["probe"]["fits"][label] for other packages.
- remember(entry): keep one typed fact {scope, kind, frame, about, value, evidence} (probe_memory.py). Refused:
  frame base or joint, joint angles, base-frame points, pixels, look ids, bare numbers. A device_kind entry is a
  hypothesis, marked measured only when a press this session's journal holds, and the entry cites, agrees with it.
- robot://experience: what was remembered (probe_store.py), written to ex.context["experience_path"] when one is
  given, so the next episode on this robot reads it when its context arrives. robot://devices gains the remembered
  device_kind entries as hypotheses.

What the kind can and cannot say. It is read from the TIP's path, which is the part's path only while friction
carries the tip with the part. kind is "unknown" (with why, and motion_dir when the tip went somewhere) when the
path is too short to tell a slide from a hinge, when neither a line nor an arc fits it, when the tip was seen to
slide on the part, or when the push ended early. A slide that is NOT seen (a contact that creeps evenly) still
reads as a clean line or arc and can give the wrong kind; nothing here can tell.

Like touch and press, a probe is refused after a dark look, leaves a retreat only the path from before the part
resisted (never the push itself), and backs off the part before a base check. No hooks on other primitives.
Nothing here reads simulator ground truth, and no robot or device is named.
"""

import math

import numpy as np

from rcpx import bounds_book_read, surfaces
from rcpx.context import refusal
from rcpx.control_base import primitive
from rcpx.r2 import probe_memory as memory
from rcpx.r2 import probe_bounds as derived
from rcpx.r2 import probe_push as push
from rcpx.r2 import probe_report as report
from rcpx.r2.probe_store import Store

ID = "I27"
PKG = "probe"
MM = 1000.0


def _r(v, nd=2):
    return None if v is None else [round(float(x), nd) for x in v]


def _state(ex):
    st = ex.state.setdefault(PKG, {})
    if "store" not in st:
        st.update(store=Store(ex.model.name, ex.model.joint_names), fits={}, n=0, loaded=None)
    return st


def _load_experience(ex):
    """Read the experience file once its path is known (the runner sets the context after construction)."""
    st = _state(ex)
    path = (getattr(ex, "context", None) or {}).get("experience_path")
    if path and st["loaded"] != str(path):
        n = st["store"].load(path)
        st["loaded"] = str(path)
        ex._journal("experience_loaded", entries=n, dropped=len(st["store"].dropped), episode=st["store"].episode)
    return st["store"]


def _find_entry(world, key):
    """A world entry by id, else the newest one with that label that has a point (fresh before stale)."""
    key = str(key).strip()
    if key in world.entries:
        return world.entries[key]
    hits = [e for e in world.entries.values() if e["label"].lower() == key.lower() and e.get("point_mm")]
    hits.sort(key=lambda e: bool(e.get("stale")))
    fresh = [e for e in hits if not e.get("stale")]
    return (fresh or hits)[-1] if hits else None


def _target(ex, target):
    """({"point": m, "label", "id"}, None) or (None, refusal)."""
    if isinstance(target, str) and target.strip():
        e = _find_entry(ex.world, target)
        if e is None:
            known = sorted({x["label"] for x in ex.world.entries.values() if x.get("point_mm")})
            return None, refusal("target", f"no world entry is named {target.strip()[:40]!r}",
                                 "touch the part first and pass the id the touch returned, or a label from "
                                 "robot://world" + (f" ({', '.join(known[:8])})" if known else "") +
                                 ", or the point as [x, y, z] in base frame mm")
        if not e.get("point_mm"):
            return None, refusal("target", f"world entry {e['id']} ({e['kind']} {e['label']!r}) has no point",
                                 "touch the part and pass that contact's id, or give [x, y, z] in base frame mm")
        if e.get("stale"):
            return None, refusal("stale_target", f"world entry {e['id']} ({e['label']!r}) was taken before the "
                                 "robot's base or the scene moved: the part may no longer be there",
                                 "find the part again (look, then touch it) and probe the new contact")
        named = e["label"] if e["kind"] != "contact" else None      # every touch is labelled "contact"
        return {"point": np.asarray(e["point_mm"], dtype=float) / MM, "label": named, "id": e["id"]}, None
    try:
        p = np.asarray(target, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        p = np.zeros(0)
    if isinstance(target, (bool, dict)) or p.size != 3 or not np.all(np.isfinite(p)):
        return None, refusal("target", "target is a world label or id, or three finite numbers",
                             "the id a touch returned (for example \"contact_3\"), or [x, y, z] in base frame mm")
    return {"point": p / MM, "label": None, "id": None}, None


def _tool_gap(ex, s, p):
    """Distance from point p to the tool (m): to the tip, or to the nearest body of the tool when that is nearer
    (a touch may place its contact on a tool surface away from the tip)."""
    best = float(np.linalg.norm(s.tip - p))
    tool = getattr(ex, "_tool_bodies", None) or ()
    for name, a, b, radius in ex.model.collision_capsules(s.q):
        if name not in tool:
            continue
        ab = b - a
        t = 0.0 if float(ab @ ab) < 1e-12 else min(1.0, max(0.0, float((p - a) @ ab) / float(ab @ ab)))
        best = min(best, max(0.0, float(np.linalg.norm(p - (a + t * ab))) - float(radius)))
    return best


class Mixin:
    # ------------------------------------------------------------------ probe_motion
    @primitive
    def probe_motion(self, target, direction, max_mm=None, force_n=None, frame="base", label=None, intent=None, tolerance_deg=None, max_n=None):
        B = derived.of(self)
        started_s = self.sim_time()
        if max_mm is not None:
            try:
                max_mm = float(max_mm)
            except (TypeError, ValueError):
                return refusal("max_mm", "max_mm must be finite and nonnegative", "probe_motion(target=" + repr(target) + ", direction=" + repr(direction) + ")")
            if not math.isfinite(max_mm) or max_mm < 0:
                return refusal("max_mm", "max_mm must be finite and nonnegative", "state the intended travel")
        if label is not None and not (isinstance(label, str) and label.strip()):
            return refusal("label", "label is a short name for the part", "for example \"lever\"")
        if frame not in ("base", "tool"):
            return refusal("frame", f"unknown frame {str(frame)[:30]!r}", "the direction is in frame base or tool")
        dark = self._dark_refusal_now()         # a probe is a push toward the world: not after a dark look
        if dark is not None:
            return dark
        initial = self._sense()
        stated_force = force_n if force_n is not None else max_n
        if isinstance(stated_force, (int, float)) and stated_force > 0 and initial.Fmag > stated_force:
            return refusal("pressing", f"the tip already feels {initial.Fmag:.1f} N, over the {stated_force:g} N of this probe",
                           "ease or retreat, or state more force")
        s, eased = self._ease_if_pressing()
        if isinstance(direction, str):
            direction = surfaces.DIRECTION_NAMES.get(direction.strip().lower(), direction)
        if isinstance(direction, str):
            return refusal("direction", f"unknown direction {direction[:30]!r}",
                           "a name (" + ", ".join(surfaces.DIRECTION_NAMES) + ") or a vector [x, y, z]")
        d, bad = self._unit_in_base(direction, frame, s)
        if bad is not None:
            return bad
        tgt, bad = _target(self, target)
        if bad is not None:
            return bad
        thing = B.thing(tgt["id"] or tgt["point"])
        entry = self.world.get(tgt["id"]) if tgt["id"] else None
        thing = derived.target_thing(B, thing, entry, s.q, d)
        if force_n is None and max_n is not None:
            force_n = max_n
        defaulted = None
        if force_n is None:
            force_n, source, evidence = derived.default_force_n(B, thing, entry)
            if force_n is None:
                # the design rulings: a force nobody stated and nothing has shown is a doubt, never a refusal: the ramp runs to
                # what the arm can push here, as the core's press does (r3_limits.force_spec), and the result says so
                spec0 = B.force("probe", thing, s.q, d, intent=dict(intent or {}))
                force_n, source = float(spec0.ceiling_n), "model"
                defaulted = evidence = ("no force was stated, no sheet serves one and no finding touch recorded the resisting "
                            f"force: the ramp runs to {force_n:.3g} N, what the arm can push along this line here; "
                            "state max_n to bound it")
        try:
            force_n = float(force_n)
        except (TypeError, ValueError):
            return refusal("force", "force_n must be a finite intended force", "state force_n")
        call_intent = dict(intent or {}, max_n=force_n)
        force_spec = B.force("probe", thing, s.q, d, intent=call_intent)
        if not math.isfinite(force_n) or force_n < B.f_thr() or force_n > force_spec.ceiling_n:
            return refusal("force", f"model/contact: force_n must lie between the detection threshold {B.f_thr():g} N and ceiling {force_spec.ceiling_n:g} N", "state force_n within these measured bounds")
        travel_item = derived.travel_item(B, s.q, d, self.model.length_scale)
        if max_mm is None:
            max_mm = travel_item["value"] * MM
            if not max_mm > 0:
                return refusal("travel_unknown", "no resolution and no still-hold jitter is known for this robot, so "
                               "RCP cannot offer the least travel that tells a slide from a hinge",
                               f"probe_motion(target={target!r}, direction={direction!r}, force_n={force_n:g}, "
                               "max_mm=<the travel you mean, mm>)")
        else:
            travel_item = dict(travel_item, value=max_mm / MM, source="intent", evidence="max_mm argument", lift="state more travel")
        seek = derived.seek_m(B, s.tip, d, tgt["point"], thing, s.q)
        near_m = derived.near_m(B, thing, seek)
        gap = _tool_gap(self, s, tgt["point"])
        if gap > near_m:
            return refusal("not_at_target", f"contact/seen: the tool is {gap * MM:g} mm from the target; its uncertainty and allowed seek give {near_m * MM:g} mm",
                           f"touch(direction={direction!r})")
        if s.Fmag > force_n:
            return refusal("pressing", f"the tip already feels {s.Fmag:.1f} N, over the {force_n:g} N of this probe",
                           "ease or retreat, or probe with more force")
        able = B.f_cap(s.q, d) * (1 - B.e_tau())
        if force_n > able:
            return refusal("force", f"the tip can push at most {able:.1f} N along this direction here",
                           "lower force_n, or come at the part so the push runs along a stronger direction")
        give = self.base_give
        if give and give.get("bound_n") is not None and force_n > give["bound_n"]:
            return refusal("base_give", f"the base gives {give['mm_per_n']:.2f} mm per N: {force_n:g} N would shift "
                           "it beyond the stored points' uncertainty",
                           f"probe with at most {give['bound_n']:.1f} N, or brace the base")
        length = seek + max_mm / MM
        path, ok_len = push.reach_plan(self, s, d, length)
        if ok_len < length - 1e-6:
            return refusal("reach", f"the arm can follow this push for {ok_len * MM:.0f} mm of the "
                           f"{length * MM:.0f} mm it may need: a joint limit or the edge of its reach",
                           "lower max_mm, or come at the part from a pose with more room along the push")
        bad = self._path_check([s.q] + path, True)
        if bad is not None:
            return bad
        B.begin("probe")
        word = B.word(call_intent, path=[s.q] + path)
        if word.get("refused"):
            return word["refused"]
        try:
            st = _state(self)
            q_pre = s.q.copy()
            run = push.run(self, s, d, max_mm / MM, force_n, seek, thing=thing, intent=call_intent, tolerance_deg=tolerance_deg)
            e = self._ease_impl()
            if run.get("tick"):
                self._stop_asked()
            # a retreat may retrace only what came before anything resisted, then a lift off the part: the targets
            # streamed during the push lead INTO the part, and retracing them would push it again
            self._push_history(self._retrace(run["free"], e, d))
            back_m = float((run["s"].tip - e.tip) @ run["d"])
            out = report.build(self, st, run, tgt, d, label, force_n, max_mm, back_m, float(e.Fmag))
            errors = [(e.get("data") or {}).get("error_mm") for e in self.world.entries.values()]
            errors = [e / MM for e in errors if isinstance(e, (int, float))]
            hard = derived.base_due(B, run["peak_n"], self.base_give, errors, self.cfg.get("base_kind"), cap_stop=run["stop"] in ("cap", "stall")) and not run.get("tick")
            gone = max(0.0, float((e.tip - s.tip) @ d)) + self._lead_for(force_n, e.q, d) + B.travel("retreat", thing, e.q, -d, intent=call_intent).backoff_m
            out["base_moved"] = self._base_check(hard, q_pre, back=-d * gone)
            out["tip_mm"] = _r(self._sense().tip * MM)
            out["force_after_n"] = round(float(e.Fmag), 2)
            if eased:
                out["eased_first"] = "a joint was still pressing; eased before the probe"
            total = int(round((self.sim_time() - started_s) / B.dt()))
            out["periods"] = [["probe", len(run["done"])], ["ease_and_checks", total - len(run["done"])]]
            out["periods_used"] = total
            out["said"] = f"The probe reports {out['kind']}: {out['why']}"
            if defaulted:
                out["force_default"] = defaulted
                out["said"] += f" ({defaulted})"
            if word.get("text"):
                out["space_word"] = word["text"]
            return B.report(out, force_spec.items + [travel_item])

        finally:
            end_call = getattr(B.space, "end_call", None)
            if callable(end_call):
                end_call()

    # ------------------------------------------------------------------ remember
    @primitive
    def remember(self, entry=None, **fields):
        if entry is None and fields:
            entry = fields                      # the entry's fields given flat
        elif isinstance(entry, dict) and fields:
            entry = {**entry, **fields}
        bad = memory.lint(entry, self.model.joint_names)
        if bad is not None:
            return refusal(bad[0], bad[1], bad[2])
        store = _load_experience(self)
        cited = list(entry.get("evidence") or [])
        found, checked = {}, self.journal is not None
        if cited and checked:
            found, missing = memory.find_calls(self.journal.records(), cited)
            if missing:
                return refusal("evidence", "no call in this session's journal carries the id "
                               + ", ".join(repr(x) for x in missing),
                               "cite the ref a press or a probe returned (P<n>, PM<n>), the id a touch returned "
                               "(contact_<n>) or a look_id; leave evidence empty when there is none")
        measured, by, whys = False, None, []
        for cid, rec in found.items():
            ok, why = memory.agrees(entry, rec, B=derived.of(self))
            whys.append(f"{cid}: {why}")
            if ok and not measured:
                measured, by = True, {"call": cid, "name": rec.get("name"), "why": why}
        hypothesis = entry["scope"] == "device_kind"
        status = "measured" if measured else ("hypothesis" if hypothesis else "kept")
        e, new = store.add(entry, status, measured, by, [str(x) for x in cited])
        if measured:
            e["force_hidden_n"] = derived.of(self).f_hidden()
            e["sigma_force_n"] = derived.of(self).sigma_f()
        persisted = store.save()
        out = {"remembered": True, "id": e["id"], "new": new, "scope": e["scope"], "kind": e["kind"],
               "frame": e["frame"], "about": e["about"], "status": e["status"], "measured": e["measured"],
               "measured_by": e["measured_by"], "persisted": persisted, "entries": len(store.entries)}
        if not e["measured"]:
            if not cited:
                out["why"] = "no evidence cited: kept as stated, unchecked"
            elif not checked:
                out["why"] = "this session keeps no journal, so the cited calls could not be read"
            else:
                out["why"] = "; ".join(whys)
            if hypothesis:
                out["hint"] = ("it stays a hypothesis for this kind of device; cite a press whose result agrees "
                               "(a force entry: its peak_force_n; a tool frame direction: its direction) to mark it "
                               "measured")
        if not persisted:
            out["note"] = store.problem or "no experience file was given to this session: it ends with the session"
        return out


def false_stops_over_drives(ex, result):
    """p4-read (b), a post hook after the obstacles fit: a contact this call's stop made whose own force on the point
    it was fitted to (data force_n) is over what the drives can push at that point along its push (their torque
    limits over the point's levers, rcpx/bounds_book_read.py) is no thing's push: the entry is marked stale (the
    obstacles, the jaws' room and the clearances leave it out), the stop is counted false, the session's level is
    raised by the guard's own reading (SelfKnow.note_false_stop) and the result says so. Returns None."""
    if not isinstance(result, dict) or "refused" in result:
        return None
    if result.get("stopped") not in ("contact", "cap", "part_hit"):
        return None
    contact = result.get("contact") if isinstance(result.get("contact"), dict) else {}
    now = ex.world.now.get("call")
    B = derived.of(ex)
    marked = []
    for e in list(ex.world.entries.values()):
        if e["kind"] != "contact" or e.get("stale") or (e.get("observed_at") or {}).get("call") != now:
            continue
        data = e.get("data") or {}
        force = data.get("force_n")
        if not isinstance(force, (int, float)) or isinstance(force, bool):
            continue
        point = data.get("part_point_mm") or e.get("point_mm")
        u = data.get("part_normal") or data.get("normal_est") or data.get("fit_normal")
        rows, _ = bounds_book_read._readings_at(ex, contact.get("t_s"))
        q = rows[0][1] if rows else ex._sense().q
        over, push, words = bounds_book_read.over_drives(ex, float(force), q, u, point, data.get("touched_by"))
        if not over:
            continue
        why = (f"a false stop: {float(force):.1f} N on {data.get('touched_by') or 'the point'} is over the "
               f"{float(push):.1f} N the drives can push there along it ({words}): no thing pushed so")
        ex.world.mark_stale([e["id"]], why, source="p4-read false stop")
        marked.append(e["id"])
        try:
            B.false_stops += 1
            v = float(contact.get("tip_speed_mm_s") or 0.0) / MM
            read = contact.get("force_n")
            if isinstance(read, (int, float)):
                getattr(B.know, "note_false_stop", B.model_know.note_false_stop)(
                    {"v": v, "peak_n": float(read), "held": bool(result.get("hold"))})
        except Exception as exc:          # bounds that cannot take the note: the entry is still left out
            ex._journal("bounds_problem", where="false_stops_over_drives", error=f"{type(exc).__name__}: {exc}")
        result.setdefault("bounds_notes", [])
        if why not in result["bounds_notes"]:
            result["bounds_notes"].append(why)
    if marked:
        ex._journal("false_stop_over_drives", ids=marked)
    return None


def setup(ex):
    _state(ex)
    def stopped(name, kwargs, result):
        if not isinstance(result, dict) or "refused" in result:
            return None
        contact = result.get("contact")
        if isinstance(contact, dict):
            eid = contact.get("id")
            speed = contact.get("tip_speed_mm_s")
        elif contact is True:
            eid, speed = result.get("id"), result.get("tip_speed_mm_s", result.get("speed_mm_s"))
        else:
            return None
        entry = ex.world.get(eid) if eid else None
        if entry is None:
            return None
        B, pose = derived.of(ex), ex._sense()
        normal = (contact if isinstance(contact, dict) else result).get("normal_est", [0, 0, 1])
        u = np.asarray(normal, dtype=float)
        size = float(np.linalg.norm(u))
        u = u / size if size else np.array([0, 0, 1])
        words = "contact speed and detection delay"
        if speed is None:
            # p4-read (a): the speed the arm READ at the stop at the touching point, never the free speed bound
            t_s = contact.get("t_s") if isinstance(contact, dict) else result.get("t_s")
            v, said = bounds_book_read.read_speed_m_s(ex, entry.get("point_mm"),
                                                      (entry.get("data") or {}).get("touched_by"), t_s)
            words = said + " and detection delay"
        else:
            v = float(speed) / MM
        error = derived.contact_error_m(B, v, pose.q, u)
        data = entry.get("data") or {}
        ex.world.update(eid, data={"error_mm": max((data.get("error_mm") or 0), error * MM),
                                   "sigma_p_mm": max((data.get("sigma_p_mm") or 0), (data.get("error_mm") or 0), error * MM),
                                   "error_source": words})
        return None
    ex.add_post(stopped)
    ex.add_post(lambda name, kwargs, result: false_stops_over_drives(ex, result))
    ex.on_event("context", lambda **f: _load_experience(ex) if "experience_path" in (f.get("keys") or []) else None)
    _load_experience(ex)


def _experience(ex, session):
    return _load_experience(ex).doc()


def _patch_devices(ex, session, content):
    """robot://devices gains what was remembered about kinds of device, each a hypothesis with its standing."""
    if not isinstance(content, dict):
        return None
    rows = [{"about": e["about"], "kind": e["kind"], "frame": e["frame"], "value": e["value"], "hypothesis": True,
             "measured": e["measured"], "source": "robot://experience " + e["id"]}
            for e in _load_experience(ex).entries if e["scope"] == "device_kind"]
    return {**content, "remembered": rows} if rows else None


_S = surfaces
_ENTRY = _S._obj({
    "scope": _S._str("what the fact is about: this robot, every device of a kind, or a way of doing something",
                     memory.SCOPES),
    "kind": _S._str("what sort of fact", memory.KINDS),
    "frame": _S._str("what directions and distances in the value are relative to: the device's own part, the tool, "
                     "or none (default none)", memory.FRAMES),
    "about": _S._str("the kind of device (as robot://devices names it, or your own words) or the procedure's name; "
                     "needed for scope device_kind and procedure"),
    "value": {"description": "what was learned: text, or an object of text and typed numbers (keys ending _n are "
                             "newtons, _s seconds, _mm a distance beside an \"along\" naming a device or tool "
                             "direction; count, times, presses, rounds, steps are counts)"},
    "evidence": {"type": "array", "items": {"anyOf": [{"type": "string"}, {"type": "integer", "minimum": 0}]},
                 "description": "ids of the calls that showed it: a press ref (P<n>), a probe ref (PM<n>), a touch "
                                "id, a look_id"}}, ["scope", "kind", "value"])
TOOLS = [
    _S._tool("probe_motion",
             "With the tip resting on a movable part, find out how the part moves: push it gently in measured "
             "steps along a direction while the tip is left free to be carried sideways, and fit the tip's path. "
             "Returns kind (prismatic: it slides; revolute: it swings about a hinge; stuck: it held from the start "
             "at this force; free: nothing pushed back, or the part gave way; unknown, with why: too little travel "
             "to tell a slide from a hinge, a path neither fits, the tip slid on the part, or the push ended "
             "early), axis_dir (the slide's line or the hinge's axis, base frame), center_mm and radius_mm of a "
             "hinge, hinge_ruled_out_mm with prismatic (a hinge farther away than that cannot be told from a slide "
             "over this travel), motion_dir (the way the tip went with the part: press along it to keep the part "
             "moving; given with unknown too), travel, residual and peak force. The kind is read from the tip's "
             "path, so it holds only while friction carries the tip with the part: on a slippery part a slide can "
             "read as a hinge or the reverse (slid_mm and notes report a slide that was seen), so treat it as a "
             "hypothesis. A result with a direction, or stuck, is stored in robot://world as an unconfirmed device "
             "entry.",
             _S._obj({"target": {"anyOf": [{"type": "string"}, {"type": "array", "minItems": 3, "maxItems": 3, "items": {"type": "number"}}],
                                 "description": "the part: the id a touch returned or a label in robot://world, or "
                                                "its point [x, y, z] in base frame mm; the tip must be within "
                                                "its uncertainty plus allowed seek distance"},
                      "direction": _S._direction("which way to push the part"),
                      "max_mm": {"type": "number", "minimum": 0, "description": "intended travel in mm; the least measurable travel is offered"},
                      "force_n": {"type": "number", "description": "intended force in N; the finding touch force is offered"},
                      "max_n": {"type": "number", "description": "the intended force ceiling in N"},
                      "intent": {"type": "object", "description": "the motion intent, including clear after a look"},
                      "tolerance_deg": {"type": "number", "minimum": 0, "description": "allowed direction change in degrees"},
                      "frame": _S._str("frame the direction is in: base or tool (default base)", ["base", "tool"]),
                      "label": _S._str("a name for the part in robot://world (default: the target's label)")},
                     ["target", "direction"]),
             {"target": "contact_3", "direction": "down"},
             "probe the touched part and report how it moves",
             brief="Push a touched part gently and report how it moves: slides, swings, holds, or unknown."),
    _S._tool("remember",
             "Keep one fact for later episodes on this robot (read it back in robot://experience): a force, a "
             "direction, a side, a surface, a sequence, a timing or a note, about this robot, a kind of device or a "
             "procedure. Poses are refused, since they stop being true when anything is moved: no joint angles, no "
             "base frame points, no pixels, no look ids, no bare numbers. A fact about a kind of device is a "
             "hypothesis; it is marked measured only when a press you cite in evidence reached the stated force "
             "or ran along the stated tool direction (other kinds of fact stay hypotheses).",
             _S._obj({"entry": _ENTRY}, ["entry"]),
             {"entry": {"scope": "procedure", "kind": "note", "frame": "none", "about": "opening the lid",
                        "value": "the lid hinges on its far side", "evidence": []}},
             "keep a procedure that uses observations from this session",
             brief="Keep one typed fact (a force, a direction, a side, an order; never a pose) for later."),
]
RESOURCES = {"robot://experience": (_experience, "What was remembered on this robot, in this episode and earlier "
                                                 "ones: typed facts with no pose in them, each a hypothesis or "
                                                 "measured.")}
RESOURCE_PATCH = {"robot://devices": _patch_devices}
PRACTICES = ("robot://experience (free) holds what earlier episodes on this robot learned. probe_motion "
             "pushes a touched part lightly and says how it moved (axis_dir, motion_dir, kind: a hypothesis); stuck "
             "means it did not move that way. remember keeps what stays true after the robot or the device moves "
             "(forces, sides, the order of steps), never where something was.")
