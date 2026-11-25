# SPDX-License-Identifier: AGPL-3.0-only
"""epochs: one world frame W, epochs, and refusal of stale targets.

Every stored point is in frame B, so a base that turns makes every point wrong at once. This package keeps a world
frame W (frame B as it stood at the standard look reference), the base pose T_WB {x_mm, y_mm, yaw_deg, source,
error_mm} measured by a ground-plane fit of each standard look against its reference (epochs_fit), and stamps every
positional world entry with the epoch its point is expressed in (ex.state["epochs"]["stamps"], by entry id).

ex.epoch goes up on:
- a measured base move (a standard look, alone or inside a press's base check, whose fit shows a turn or shift):
  every stored point with a known epoch pose is re-expressed through the motion with World.update, the frame B
  directions and points in its data moved with it and its error grown by the estimate's error; the measured base
  give goes stale (ex.base_give = None) and so do earlier reach checks;
- a cap or part_hit stop, and a measured give over the push bound (the smallest error of the stored points): the base may have moved, so
  the new epoch's pose is unknown until the next standard look measures it (no motion: points carry over, their
  error grown by the motion under the thresholds that look measured plus its error);
- intervention_detected (fields what="base" or "device", device=name; see _intervention_kind): base = as a cap;
  anything else = World.world_moved, and every positional entry is gone (only a new look or touch replaces it): the
  World does not say which entry lies on which device; marks on looks from before it are refused by triangulate.
  The amount of a move is never read.

A point first seen (or seen again) during a call is stamped in the epoch that call ends in, at the point it was
seen: a stop's contact (stored by another package's post hook) is sensed at the end of the push that may have
moved the base, and a press's base check looks after it, so the call's own measurement never moves that point.
A new reference (standard_look(set_reference=true)) that replaces another is first fitted against it, so a base
move since the last measured pose is measured before W carries over; when that fit cannot measure, or the pose is
unknown with no reference to compare with, W starts again and older points are gone.

Round one's relocalize marks every stored point stale in the World when it says moved. Inside a standard look this
package's fit decides: a measured base move re-expresses those points (which lifts the marks), a still base keeps
them. An explicit relocalize() re-compares two looks already measured here, so the marks it sets are lifted again
(robot://world otherwise refuses points re-expressed a call earlier). A stale mark from any other source, or one
kept earlier, is never lifted or re-expressed; only a new observation replaces it.

A target from an older epoch is refused before motion with check stale_target only when the base was MEASURED
to have moved since (both epochs' base poses are known and the point was not re-expressed): on reach_check (its
`label` argument, or a point_mm equal to such a stored point), on triangulate (marks from looks between which the
measured base motion is over their own error), on approach and on any package tool argument whose name says it is
a target (point, target, goal or where in the name, at, to, and along for normal:<id>) that names a stored point
by id or label, or gives one of its points. A target that is only doubted (the world's stale mark, a point gone
after an intervention, an epoch whose base pose nobody measured) is acted on with the guard armed and journalled
(stale_target_acted): doubt alone does not refuse. Other arguments (a device id, a look id, a note) are
not read as stored points. A stale give bound refuses nothing.

Reads executor state the round one Executor keeps (_std_ref, _std_last, _looks, _marks, _calls); reads no ground
truth. RobotModel stays in frame B; only this package composes with T_WB.
"""

import math
import numpy as np
from rcpx.bounds_ratios import QUANT
from rcpx.r2 import epochs_bounds as derived

from rcpx import surfaces
from rcpx.context import refusal
from rcpx.r2 import epochs_book as book
from rcpx.r2 import epochs_fit as fit

ID = "I23"

ROUND_ONE = {t["name"] for t in surfaces.VNEXT_TOOLS} | {"read_state", "snapshot", "idle"}
POINT_WORDS = ("point", "target", "goal", "where")
POINT_KEYS = {"at", "to", "at_mm", "to_mm"}
ALONG_KEYS = {"along"}       # a direction that may name a stored surface (normal:<id>), never a point


def _names_target(key):
    """An argument whose name says it is a place to go (or a stored surface to go along). Others (an id of a device,
    a look, a note) are never read as stored points, so a read-only call is not refused for a stale label."""
    return key in POINT_KEYS or key in ALONG_KEYS or any(w in key for w in POINT_WORDS)


# ------------------------------------------------------------------ helpers
def _is_point(v):
    return (isinstance(v, (list, tuple)) and len(v) == 3
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v))


def _look_id_of(ex, rec):
    for k, v in (getattr(ex, "_looks", None) or {}).items():
        if v is rec:
            return k
    return None


def _stamp_looks(ex, st):
    for k in (getattr(ex, "_looks", None) or {}):
        if k not in st["looks"]:
            st["looks"][k] = ex.epoch


# ------------------------------------------------------------------ pre hook
def _reach_label(ex, st, kwargs):
    label = kwargs.pop("label", None)
    if label is None:
        if kwargs.get("point_mm") is None:        # the surface makes point_mm optional once label exists
            return refusal("point_mm", "reach_check needs point_mm or label",
                           "give point_mm [x, y, z] in mm, frame base, or label: a stored point's id from "
                           "robot://world such as contact_3")
        return None
    eid = book.find(ex, label)
    if eid is None:
        return refusal("label", f"no stored point named {label!r} in robot://world",
                       "use an id from robot://world such as contact_3, or pass point_mm")
    bad = _measured_stale(ex, st, eid, book.unusable(ex, st, eid))
    if bad is not None:
        return refusal("stale_target", *bad)
    e = ex.world.entries[eid]
    given = kwargs.get("point_mm")
    ignored = given if _is_point(given) and math.dist(given, e["point_mm"]) > book.MATCH_MM else None
    kwargs["point_mm"] = list(e["point_mm"])
    s = st["stamps"].get(eid) or {}
    st["label_used"] = {"label": str(label), "id": eid, "epoch": ex.epoch, "point_mm": list(e["point_mm"]),
                        "error_mm": book.rnd(s.get("error_mm"))}
    if ignored is not None:
        st["label_used"]["point_mm_ignored"] = list(ignored)
    return None


def _measured_stale(ex, st, eid, bad):
    """bad (why, hint) when the entry is unusable because the base was MEASURED to have moved since its epoch
    (both base poses known), else None: a target that is only doubted is acted on, and the doubt journalled."""
    if bad is None:
        return None
    s = st["stamps"].get(eid) or {}
    rec, cur = book.epoch_rec(st, s.get("epoch")), book.current(st)
    e = ex.world.entries.get(eid) or {}
    moved = (s.get("gone") is None and not e.get("stale") and s.get("epoch") != ex.epoch
             and rec is not None and rec["pose"] is not None and cur["pose"] is not None)
    if moved:
        return bad
    ex._journal("stale_target_acted", id=eid, why=bad[0], would_remove=bad[1])
    return None


def _pose_gap(st, ep):
    """(turn deg, shift mm at the work area) between epoch ep and now, or None when a pose is unknown."""
    rec, cur = book.epoch_rec(st, ep), book.current(st)
    if rec is None or rec["pose"] is None or cur["pose"] is None:
        return None
    at = (st.get("estimate") or {}).get("foot_mm") or [0.0, 0.0]
    return fit.motion_size(book.relative(rec["pose"], cur["pose"]), at)


def _world_moved_since(st, ep, now):
    """Why the world (not the base) moved in an epoch after ep up to now, else None."""
    whys = [r["world_moved"] for r in st["epochs"] if ep < r["epoch"] <= now and r.get("world_moved")]
    return whys[-1] if whys else None


def _epoch_error(ex, st, ep):
    rec = book.epoch_rec(st, ep)
    current = book.current(st)
    return QUANT * (fit.error_at((rec or {}).get("err"), [0, 0, 0]) +
                    fit.error_at(current.get("err"), [0, 0, 0]))


def _triangulate_check(ex, st, label):
    marks = (getattr(ex, "_marks", None) or {}).get(str(label)) or {}
    if len(marks) < 2:
        return None
    points = [e["point_mm"] for e in ex.world.query(include_stale=True)
              if e.get("label") == label and e.get("point_mm") is not None]
    if points:
        lever_mm = float(np.linalg.norm(points[-1]))
    else:
        A, b = np.zeros((3, 3)), np.zeros(3)
        for mark in marks.values():
            projection = np.eye(3) - np.outer(mark["dir"], mark["dir"])
            A += projection
            b += projection @ mark["origin"]
        lever_mm = float(np.linalg.norm(np.linalg.lstsq(A, b, rcond=None)[0])) * 1000.0
    eps = {k: st["looks"].get(k, ex.epoch) for k in marks}
    worst, moved_world = None, None
    for k, ep in eps.items():
        if ep == ex.epoch:
            continue
        moved_world = moved_world or _world_moved_since(st, ep, ex.epoch)
        gap = _pose_gap(st, ep)
        if gap is None:
            worst = worst or ("the base pose was not measured in between", None)
        elif math.radians(abs(gap[0])) * lever_mm > _epoch_error(ex, st, ep) or gap[1] > _epoch_error(ex, st, ep):
            if worst is None or worst[1] is None or gap[1] > worst[1]:
                worst = (f"the base turned about {abs(gap[0]):.0f} deg and shifted about {gap[1]:.0f} mm since", gap[1])
    seen = ", ".join(f"{k} (epoch {ep})" for k, ep in eps.items())
    if worst is None or worst[1] is None:        # nothing MEASURED says the base moved between the looks
        if worst is not None or moved_world is not None:
            ex._journal("stale_target_acted", id=str(label), why=f"marked in looks {seen}: "
                        + (moved_world or worst[0]), would_remove="mark it again in two looks taken now")
        return None
    hint = ("mark it again under a new label in two looks taken now, a few cm apart, then triangulate that label")
    return refusal("stale_target", f"{label!r} is marked in looks {seen}, now epoch {ex.epoch}; {worst[0]}: rays "
                   "from before the move do not meet the new ones", hint)


def _pre(ex, name, kwargs):
    st = book.st_of(ex)
    st.pop("label_used", None)
    book.sync(ex, st)
    st["stale_before"] = book.stale_ids(ex)     # the post hook tells this call's stale marks from older ones
    if name == "reach_check":
        r = _reach_label(ex, st, kwargs)
        if r is not None:
            return r
    elif name == "triangulate":
        r = _triangulate_check(ex, st, kwargs.get("label"))
        if r is not None:
            return r
    for key, value in list(kwargs.items()):
        k = str(key).lower()
        if not _names_target(k):
            continue
        if _is_point(value) and k not in ALONG_KEYS:
            hit = book.point_check(ex, st, [float(x) for x in value])
            if hit is not None and _measured_stale(ex, st, hit[0], (hit[1], hit[2])) is not None:
                return refusal("stale_target", hit[1], hit[2])
        elif isinstance(value, str) and name not in ROUND_ONE:
            for ref in (value, value.rsplit(":", 1)[-1]):      # an id alone, or inside a form such as normal:<id>
                eid = book.find(ex, ref.strip())
                bad = None if eid is None else _measured_stale(ex, st, eid, book.unusable(ex, st, eid))
                if bad is not None:
                    return refusal("stale_target", *bad)
    return None


# ------------------------------------------------------------------ post hook
def _unknown_motion(ex, name, result, pending):
    whys = []
    stops = {p.get("stopped") for p in pending if p["kind"] == "contact"} | {result.get("stopped")}
    if result.get("part_hit"):
        stops.add("part_hit")
    for stop in ("cap", "part_hit"):
        if stop in stops:
            whys.append(f"a {stop} stop in {name} at call {getattr(ex, '_calls', 0)} may have moved the base")
    if name == "measure_base_give":
        shift = result.get("shift_mm")
        errors = [s["error_mm"] for s in book.st_of(ex)["stamps"].values() if s.get("error_mm") is not None]
        pose = ex._sense()
        bound = min(errors) if errors else derived.of(ex).rigid(pose.q, [0, 0, 1]) * 1000.0
        if isinstance(shift, (int, float)) and shift > bound:
            whys.append(f"the base gave {shift:.1f} mm under measure_base_give's test push (over the {bound:g} mm "
                        "push bound)")
    return "; ".join(whys)


def _anchor(ex, st, ref):
    """A new standard look reference. One that replaces another is first fitted against it (as a standard look
    would be), so a base move since the last measured pose is measured and stored points re-expressed before W
    carries over to the new reference; a pose that was unknown is measured the same way. When that fit cannot
    measure, or the pose is unknown with no reference to compare with, W starts again and older points are gone."""
    lid = _look_id_of(ex, ref)
    info, restart = {}, None
    old = st["anchor"]
    old_rec = (getattr(ex, "_looks", None) or {}).get(old["look_id"]) if old else None
    if old_rec is not None and old_rec is not ref:
        m = _measure(ex, st, old_rec, ref, False, [])
        info["compared"] = {"with": old["look_id"], **{k: v for k, v in m.items() if k != "look_id"}}
        if not m["measured"]:
            restart = f"could not be compared with the reference it replaces ({old['look_id']}: {m['why']})"
    cur = book.current(st)
    if cur["pose"] is None or restart:
        why = restart or "was set while the base pose was unknown"
        for rec in st["epochs"]:              # their relation to the new W is unknown: their points are lost
            rec["pose"], rec["err"] = None, None
        book.raise_epoch(ex, st, f"the standard look reference {lid} {why}: W starts again here", pose=fit.pose(),
                         source=f"reference look {lid}: W is frame B as it stood then")
        hold = book.held(st)
        book.mark_gone(ex, st, [k for k, s in st["stamps"].items() if s["epoch"] != ex.epoch and k not in hold],
                       f"W started again at the reference look {lid} (it {why}): a point from before it was never "
                       "measured against the new W", "touch or look at it again")
        T = fit.pose()
    else:
        T = cur["pose"]
        if old is None and cur["epoch"] == 0:
            cur["source"] = f"reference look {lid}: W is frame B as it stood then"
    st["anchor"] = {"look_id": lid, "pose": T}
    st["last_std"] = None
    info["anchored"] = lid
    return info


def _measure(ex, st, ref, last, core_moved, fresh):
    """Fit the latest standard look against the reference; raise and re-express on a base move. fresh = the entries
    this call's relocalize marked stale: re-expressed with a measured base move (it explains them), kept stale when
    the base is found still (a device may have moved). Older stale marks are never lifted here."""
    lid = _look_id_of(ex, last)
    st["last_std"] = lid
    if ref.get("img") is None or last.get("img") is None or "dark" in (ref.get("can_see"), last.get("can_see")):
        est = {"ok": False, "why": "a look is dark or kept no image"}
    else:
        est = fit.estimate(ex.model, ref, last, B=derived.of(ex))
    if not est["ok"]:
        st["estimate"] = {"look_id": lid, "ok": False, "why": est["why"]}
        out = {"measured": False, "look_id": lid, "why": est["why"]}
        if core_moved:
            why = f"relocalize ({lid}) says the base or world moved and the ground fit could not measure it"
            book.raise_epoch(ex, st, why)
            book.give_stale(ex, st, why)
            out["raised"] = True
        return out
    T = fit.compose(st["anchor"]["pose"], fit.invert(est["D"]))
    foot = est["foot_mm"]
    err_foot = fit.error_at(est["err"], fit.apply(est["D"], [foot[0], foot[1], 0.0]))
    err_origin = fit.error_at(est["err"], [0.0, 0.0, 0.0])    # base_pose's x_mm, y_mm are the base origin
    known, cur = book.last_known(st), book.current(st)
    _, disp = fit.motion_size(book.relative(known["pose"], T), foot)
    dyaw = T["yaw"] - known["pose"]["yaw"]                  # the base's own turn since the last measured pose
    yaw = math.degrees(math.atan2(math.sin(dyaw), math.cos(dyaw)))
    work_points = [foot] + [stamp["point_mm"] for stamp in st["stamps"].values()]
    moved = any(fit.motion_size(book.relative(known["pose"], T), p)[1] >
                QUANT * fit.error_at(est["err"], p) for p in work_points)
    source = (f"standard look {lid} against the reference {st['anchor']['look_id']} ({est['inliers']} features on "
              "the surface)")
    st["estimate"] = {"look_id": lid, "ok": True, "turn_deg": book.rnd(yaw, 3), "shift_mm": book.rnd(disp),
                      "error_mm": book.rnd(err_foot), "base_origin_error_mm": book.rnd(err_origin),
                      "inliers": est["inliers"], "rms_mm": book.rnd(est["rms_mm"]),
                      "foot_mm": [book.rnd(x, 1) for x in foot], "moved": bool(moved)}
    raised = False
    if moved:
        why = f"the base turned about {abs(yaw):.1f} deg and its work area shifted about {disp:.0f} mm ({lid})"
        if cur["pose"] is None and cur["entries"] == 0:
            cur.update(pose=T, err=est["err"], error_mm=err_origin, source=source, why=cur["why"] + "; " + why)
        else:
            book.raise_epoch(ex, st, why, pose=T, err=est["err"], source=source)
            book.current(st)["error_mm"] = err_origin
            raised = True
        book.give_stale(ex, st, why)
    elif cur["pose"] is None:                 # the event did not move the base: the last measured pose holds
        cur.update(pose=known["pose"], err=known["err"], error_mm=known.get("error_mm"),
                   source=f"{known['source']}; standard look {lid} found the base had not moved since")
    skip = set(book.stale_ids(ex)) - (set(fresh) if moved else set())
    carry = None if moved else {"rel": book.relative(known["pose"], T), "err": est["err"]}
    done, _ = book.reexpress(ex, st, lid, skip=skip, carry=carry)
    lost = book.mark_lost(ex, st)            # seen while the pose was unknown: never re-expressible
    out = {"measured": True, "look_id": lid, "moved": bool(moved), "raised": raised, "turn_deg": book.rnd(yaw, 2),
           "shift_mm": book.rnd(disp, 1), "re_expressed": done, "base_pose": book.base_pose_doc(st)}
    if lost:
        out["lost"] = lost
    if core_moved and not moved:
        out["note"] = ("relocalize said moved, the ground fit says the base is still: stored points relocalize "
                       "marked stale stay stale")
    return out


def _standard_look_update(ex, st, core_moved, fresh):
    ref, last = getattr(ex, "_std_ref", None), getattr(ex, "_std_last", None)
    info = {}
    if ref is not None and (st["anchor"] is None or _look_id_of(ex, ref) != st["anchor"]["look_id"]):
        info.update(_anchor(ex, st, ref))
    if ref is not None and last is not None and _look_id_of(ex, last) != st["last_std"]:
        info.update(_measure(ex, st, ref, last, core_moved, fresh))
    return info


def _relocalize_again(ex, st, fresh):
    """An explicit relocalize compares the same two standard looks this package already measured, so the stale marks
    it sets carry nothing new: lift them (from entries not marked gone). Marks that measurement kept were set before
    this call and stay. None when the two looks were not measured here."""
    ref, last = getattr(ex, "_std_ref", None), getattr(ex, "_std_last", None)
    if (ref is None or last is None or st["anchor"] is None or _look_id_of(ex, ref) != st["anchor"]["look_id"]
            or _look_id_of(ex, last) != st["last_std"]):
        return None
    pair = f"{st['anchor']['look_id']} with {st['last_std']}"
    done = book.restore(ex, st, fresh, f"relocalize re-compared {pair}, already measured by the epochs fit")
    verdict = "a base move" if (st.get("estimate") or {}).get("moved") else "no base move it could measure"
    out = {"restored": done, "note": f"relocalize re-compared {pair}; the ground fit of those looks already found "
                                     f"{verdict} and stored points were handled then"}
    kept = [eid for eid in fresh if eid not in done]
    if kept:
        out["kept_stale"] = kept
    return out


def _post(ex, name, kwargs, result):
    st = book.st_of(ex)
    pending, st["pending"] = st["pending"], []
    used = st.pop("label_used", None)
    if not isinstance(result, dict):
        return None
    seen = book.sync(ex, st, result)     # seen during this call: in the epoch it ends in, never moved by it
    st["held"] = seen
    add = {}
    if "refused" not in result:
        e0 = ex.epoch
        fresh = book.new_relocalize_marks(ex, st.get("stale_before"))
        why = _unknown_motion(ex, name, result, pending)
        if why:
            book.raise_epoch(ex, st, why)
        info = _standard_look_update(ex, st, any(p["kind"] == "core_moved" for p in pending), fresh)
        if name == "relocalize" and fresh and "measured" not in info:
            again = _relocalize_again(ex, st, fresh)
            if again is not None:
                info.update(again)
        book.restamp(st, seen)
        if info or ex.epoch != e0:
            add["epochs"] = {"epoch": ex.epoch, "raised_from": e0 if ex.epoch != e0 else None, **info}
            add["epochs"].setdefault("base_pose", book.base_pose_doc(st))
            if ex.epoch != e0 and seen:
                add["epochs"]["seen_in_call"] = list(seen)
        if name == "measure_base_give" and result.get("verdict") in ("rigid", "gives"):
            st["give"] = {"stale": False, "epoch": ex.epoch, "verdict": result["verdict"],
                          "mm_per_n": result.get("mm_per_n"), "push_bound_n": result.get("push_bound_n")}
        if name == "reach_check":
            point = (used or {}).get("point_mm") or kwargs.get("point_mm") or []
            st["reach"].append({"call": getattr(ex, "_calls", 0), "point_mm": [book.rnd(x, 1) for x in point],
                                "reachable": result.get("reachable"), "margin_mm": result.get("margin_mm"),
                                "epoch": ex.epoch, "stale": False})
            del st["reach"][:-20]
            add["epoch"] = ex.epoch
            if used:
                add["point_from"] = used
        if name == "press" and (st.get("give") or {}).get("stale"):
            g = st["give"]
            add["base_give_stale"] = {"since_epoch": g.get("since_epoch"), "why": g.get("why"),
                                      "hint": "measure_base_give on a firm surface to bound pushes again"}
    st["held"] = []
    _stamp_looks(ex, st)
    st["stale_before"] = book.stale_ids(ex)     # for a call whose pre hook another package's refusal skipped
    return {**result, **add} if add else None


# ------------------------------------------------------------------ events
INTERVENTION_KEYS = ("what", "moved", "target", "object", "part", "intervention", "record")


def _intervention_kind(fields):
    """"base" or what else moved, from whichever field the event uses (note_event reserves the name kind). A record
    passed whole contributes its kind only: amounts and poses in it are never read."""
    for key in INTERVENTION_KEYS:
        v = fields.get(key)
        if isinstance(v, dict):
            v = v.get("kind")
        if isinstance(v, str) and v.strip():
            return v.strip().lower()
    return "device"


def _on_intervention(ex, **fields):
    st = book.st_of(ex)
    book.sync(ex, st)
    kind = _intervention_kind(fields)
    call = int(getattr(ex, "_calls", 0))
    if kind == "base":
        why = f"the base was moved (an intervention detected at call {call})"
        book.raise_epoch(ex, st, why)
        book.give_stale(ex, st, why)
        return
    name = fields.get("device") or fields.get("name")
    what = str(name) if isinstance(name, str) and name.strip() else ("a device" if kind == "device" else kind)
    why = f"{what} was moved (an intervention detected at call {call})"
    ex.world.world_moved({"kind": kind, "what": what}, source="intervention")
    book.mark_gone(ex, st, list(st["stamps"]), why)
    cur = book.current(st)
    book.raise_epoch(ex, st, why, pose=cur["pose"], err=cur["err"], source=cur["source"])
    book.current(st).update(error_mm=cur.get("error_mm"), world_moved=why)   # triangulate reads world_moved


def setup(ex):
    ex.state[book.KEY] = book.new_state()

    def pre(name, kwargs):
        return _pre(ex, name, kwargs)

    def post(name, kwargs, result):
        return _post(ex, name, kwargs, result)

    def on_contact(**f):
        book.st_of(ex)["pending"].append({"kind": "contact", "stopped": f.get("stopped"),
                                          "primitive": f.get("primitive")})

    def on_base_moved(**f):
        book.st_of(ex)["pending"].append({"kind": "core_moved", "primitive": f.get("primitive")})

    def on_intervention(**f):
        _on_intervention(ex, **f)

    ex.add_pre(pre)
    ex.add_post(post)
    ex.on_event("contact", on_contact)
    ex.on_event("base_moved", on_base_moved)
    ex.on_event("intervention_detected", on_intervention)


# ------------------------------------------------------------------ surface
def _epoch_resource(ex, session):
    return book.epoch_doc(ex, book.st_of(ex))


def _patch_state(ex, session, content):
    st = book.st_of(ex)
    if not isinstance(content, dict):
        return None
    return {**content, "epoch": ex.epoch, "base_pose": book.base_pose_doc(st)}


def _patch_world(ex, session, content):
    st = book.st_of(ex)
    if not isinstance(content, dict) or not isinstance(content.get("entries"), list):
        return None
    rows = []
    for e in content["entries"]:
        s = st["stamps"].get(e.get("id")) if isinstance(e, dict) else None
        if s is None:
            rows.append(e)
            continue
        bad = book.unusable(ex, st, e["id"])
        rows.append({**e, "epoch": s["epoch"], "usable_now": bad is None, **({"why_not": bad[0]} if bad else {})})
    return {**content, "entries": rows, "epoch": ex.epoch}


TOOLS = [
    {"extends": "reach_check",
     "args": {"label": surfaces._str("instead of copying numbers: a stored point's id or label in robot://world, such "
                                     "as contact_3; its present point is used (point_mm may then be left out, and "
                                     "is ignored if given)")},
     "optional": ["point_mm"],
     "note": "Give point_mm or label; label names a stored point: one seen before the base moved and not "
             "re-expressed is refused as stale_target."},
]

RESOURCES = {
    "robot://epoch": (_epoch_resource, "The world epoch: the base pose in W (frame B as it stood at the standard "
                                       "look reference), why each epoch began, and which stored points are usable "
                                       "now."),
}

RESOURCE_PATCH = {"robot://state": _patch_state, "robot://world": _patch_world}

PRACTICES = ("standard_look measures the base pose and re-expresses stored points (robot://epoch); a point "
             "of an older epoch is refused as stale_target. reach_check takes a stored point by its id.")
