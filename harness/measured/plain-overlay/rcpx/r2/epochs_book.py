# SPDX-License-Identifier: AGPL-3.0-only
"""Bookkeeping of the epochs package: epochs, the base pose in W, stamps on world entries, re-expression.

W is frame B as it stood at the standard look reference (x forward, y left, z up, mm). The base pose T_WB is a planar
pose {x, y, yaw} of frame B in W, measured by the ground-plane fit of a standard look against its reference
(epochs_fit). Every positional world entry (contact, located, device with a point) carries a stamp: the epoch its
point is expressed in, its error, and every point it has held. An epoch has a pose when the base pose during it is
known, else None (after a cap stop, a hard give or a moved base, until a standard look measures it).

Re-expression of an entry from epoch k to the present epoch: p_now = T_now^-1 T_k p_k (z unchanged); the frame B
directions stored with it (data DIRECTION_KEYS) turn with the base and its frame B points (data POINT_DATA_KEYS) move
with it; its error grows by the error of both estimates at that point. An epoch whose base was found not to have
moved shares the last measured pose, and its entries carry over at the same point, their error grown by what the
look measured (a motion under the move thresholds) plus the estimate's error there. An entry the World marks stale
is never re-expressed or carried (that would clear the mark): only the caller's `skip` set decides, so stale marks
the same call's relocalize set for a base move this package measured are re-expressed with it. An entry first
seen (or seen again) during a call is stamped in the epoch that call ends in, at the point it was seen, and is
never moved by that call's own measurement (restamp). An entry is usable
only when nothing marked it gone (its thing moved: only a new look or touch helps), the World does not mark it
stale, and its stamp is in the present epoch. No ground truth is read here.
"""

import copy
import math

from rcpx.context import POSITIONAL
from rcpx.r2 import epochs_fit as fit
from rcpx.r2 import epochs_bounds as derived

KEY = "epochs"
MATCH_MM = 1.0               # a point argument this close to a stored point names that point
HISTORY_KEEP = 8             # points kept per entry stamp
# frame B directions and points a world entry's data may carry (touch, approach, the obstacles fit)
DIRECTION_KEYS = ("normal_est", "normal", "approach_normal", "fit_normal", "part_normal")
POINT_DATA_KEYS = ("part_point_mm",)


def new_state():
    return {"epochs": [{"epoch": 0, "why": "start", "at_call": 0, "pose": fit.pose(), "err": None, "entries": 0,
                        "source": "assumed: W is frame B as it stood at the start, until a standard look reference "
                                  "is kept"}],
            "stamps": {}, "anchor": None, "last_std": None, "looks": {}, "estimate": None, "give": None,
            "reach": [], "pending": [], "stale_before": [], "held": []}


def st_of(ex):
    return ex.state[KEY]


def current(st):
    return st["epochs"][-1]


def epoch_rec(st, k):
    for rec in st["epochs"]:
        if rec["epoch"] == k:
            return rec
    return None


def last_known(st):
    for rec in reversed(st["epochs"]):
        if rec["pose"] is not None:
            return rec
    return None


def rnd(x, nd=2):
    return None if x is None else round(float(x), nd)


def base_pose_doc(st):
    """robot://state base_pose: {x_mm, y_mm, yaw_deg, source, error_mm} of frame B in W. error_mm is the estimate's
    error at the base origin (the point x_mm, y_mm give); a stored point carries its own error at its place."""
    cur = current(st)
    if cur["pose"] is not None:
        p = cur["pose"]
        return {"x_mm": rnd(p["x"]), "y_mm": rnd(p["y"]), "yaw_deg": rnd(math.degrees(p["yaw"]), 3),
                "source": cur["source"], "error_mm": rnd(cur.get("error_mm") or 0.0)}
    known = last_known(st)
    p = known["pose"] if known else fit.pose()
    return {"x_mm": rnd(p["x"]), "y_mm": rnd(p["y"]), "yaw_deg": rnd(math.degrees(p["yaw"]), 3),
            "source": f"unknown since epoch {cur['epoch']} ({cur['why']}); the numbers are the last measured pose "
                      "(epoch %d); take standard_look() to measure it" % (known["epoch"] if known else 0),
            "error_mm": None}


# ------------------------------------------------------------------ stamps
def _initial_error(ex, e, result=None):
    data = e.get("data") or {}
    errors = [float(data[k]) for k in ("error_mm",) if isinstance(data.get(k), (int, float))]
    for values in (e, data):
        if isinstance(values.get("sigma_p"), (int, float)):
            errors.append(float(values["sigma_p"]) * 1000.0)
        if isinstance(values.get("sigma_p_mm"), (int, float)):
            errors.append(float(values["sigma_p_mm"]))
    if e["kind"] != "contact":
        return (max(errors) if errors else None), e.get("source")
    B = derived.of(ex)
    contact = (result or {}).get("contact")
    contact = contact if isinstance(contact, dict) else {}
    matches = contact.get("id") == e["id"] or (result or {}).get("id") == e["id"]
    speed, said = data.get("tip_speed_mm_s"), "reported tip speed"
    if matches:
        # the tip's speed at the first reading over the threshold (a motion's contact record), else the speed the
        # result itself states: a touch reports the approach speed it ran at (speed_mm_s), which the tip had then
        for value, words in ((contact.get("tip_speed_mm_s"), "reported tip speed"),
                             ((result or {}).get("tip_speed_mm_s"), "reported tip speed"),
                             ((result or {}).get("speed_mm_s"), "the approach speed the call reports")):
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                speed, said = value, words
                break
    if errors and speed is None:
        return max(errors), data.get("error_source", e.get("source"))
    s = ex._sense()
    u = data.get("approach_normal") or data.get("normal_est") or [0, 0, 1]
    norm = math.sqrt(sum(float(x)**2 for x in u))
    u = [float(x) / norm for x in u] if norm > 0 else [0, 0, 1]
    source = f"contact: {said} x detection delay + quantum"
    if speed is None:
        speed_m_s = B.v_free(s.q, u)
        source = "model: speed not reported: the free speed bound"
    else:
        speed_m_s = float(speed) / 1000.0
    error = max(errors + [derived.contact_error_m(B, speed_m_s, s.q, u) * 1000.0])
    ex.world.update(e["id"], data={"error_mm": error, "sigma_p_mm": error, "error_source": source})
    return error, source


def sync(ex, st, result=None):
    """Stamp every positional entry that is new or was observed again since its stamp, with the present epoch.
    Returns the ids stamped."""
    cur = current(st)
    out = []
    for eid, e in ex.world.entries.items():
        if e["kind"] not in POSITIONAL or e["point_mm"] is None:
            continue
        s = st["stamps"].get(eid)
        if s is not None and s["observed_at"] == e["observed_at"]:
            continue
        error, error_source = _initial_error(ex, e, result)
        hist = (s["history"] if s else []) + [{"epoch": cur["epoch"], "point_mm": list(e["point_mm"]),
                                               "how": "seen"}]
        st["stamps"][eid] = {"epoch": cur["epoch"], "seen_epoch": cur["epoch"], "point_mm": list(e["point_mm"]),
                             "observed_at": dict(e["observed_at"]), "error_mm": error, "error_source": error_source, "gone": None,
                             "history": hist[-HISTORY_KEEP:], "label": e["label"], "kind": e["kind"]}
        cur["entries"] += 1
        out.append(eid)
    return out


def restamp(st, ids):
    """Entries first seen (or seen again) during a call belong to the epoch the call ends in, at the point they
    were seen: a stop's contact is sensed at the end of the motion that may have moved the base, and a base check
    looks after the push. The post hook holds them out of that call's own re-expression, carry-over and lost marks
    (st["held"]) and moves their stamps here."""
    cur = current(st)
    for eid in ids:
        s = st["stamps"].get(eid)
        if s is None or s["epoch"] == cur["epoch"]:
            continue
        old = epoch_rec(st, s["epoch"])
        if old is not None:
            old["entries"] = max(0, old["entries"] - 1)
        cur["entries"] += 1
        s["epoch"] = s["seen_epoch"] = cur["epoch"]
        if s["history"]:
            s["history"][-1]["epoch"] = cur["epoch"]


def held(st):
    """Ids seen during the call whose post hook is running (see restamp)."""
    return set(st.get("held") or ())


def raise_epoch(ex, st, why, pose=None, err=None, source=None):
    ex.epoch += 1
    st["epochs"].append({"epoch": ex.epoch, "why": why, "at_call": int(getattr(ex, "_calls", 0)), "pose": pose,
                         "err": err, "entries": 0, "source": source or why})
    ex.note_event("epoch_raised", why=why, pose_known=pose is not None)  # note_event adds the epoch
    return ex.epoch


def give_stale(ex, st, why):
    """A base move: the measured give no longer describes the base under the arm (the next push measures again)."""
    g = getattr(ex, "base_give", None)
    if g:
        st["give"] = {"stale": True, "was": dict(g), "since_epoch": ex.epoch, "why": why}
        ex.base_give = None
    elif st.get("give") and not st["give"].get("stale"):
        st["give"] = {**st["give"], "stale": True, "since_epoch": ex.epoch, "why": why}
    for rc in st["reach"]:
        rc["stale"] = rc["stale"] or rc["epoch"] != ex.epoch


def mark_gone(ex, st, ids, why, hint=None):
    for eid in ids:
        s = st["stamps"].get(eid)
        if s is not None and s["gone"] is None:
            s["gone"] = why
            if hint:
                s["gone_hint"] = hint


def mark_lost(ex, st):
    """Stamps in a past epoch whose base pose was never measured can never be re-expressed (only the present
    epoch's pose is ever measured): mark them gone with that reason. Returns their ids."""
    out, hold = [], held(st)
    for eid, s in st["stamps"].items():
        rec = epoch_rec(st, s["epoch"])
        if eid in hold:
            continue
        if s["gone"] is None and s["epoch"] != current(st)["epoch"] and (rec is None or rec["pose"] is None):
            why = rec["why"] if rec else "an earlier epoch"
            mark_gone(ex, st, [eid], f"the base pose of epoch {s['epoch']} ({why}) was never measured, so it cannot "
                      "be re-expressed", "touch or look at it again: it was seen while the base pose was unknown")
            out.append(eid)
    return out


def stale_ids(ex):
    """Ids of the positional entries the World marks stale now, sorted (kept in ex.state, so a list)."""
    return sorted(eid for eid, e in ex.world.entries.items()
                  if e["kind"] in POSITIONAL and e["point_mm"] is not None and e.get("stale"))


def new_relocalize_marks(ex, before):
    """Entries marked stale since `before` (a stale_ids list) by a relocalize's world_moved event (round one's
    image comparison), not by another source such as an intervention."""
    out = []
    for eid in sorted(set(stale_ids(ex)) - set(before or [])):
        ev = ex.world.entries.get((ex.world.entries[eid].get("data") or {}).get("stale_since"))
        if ev is not None and ev.get("source") == "relocalize":
            out.append(eid)
    return out


def restore(ex, st, ids, why):
    """Clear the World's stale mark on entries this package stamps and has not marked gone (same point,
    World.update). The epoch check still applies to them: one from an older epoch stays unusable until it is
    re-expressed. Returns the ids cleared."""
    done = []
    for eid in ids:
        s, e = st["stamps"].get(eid), ex.world.entries.get(eid)
        if s is None or e is None or s["gone"] is not None or not e.get("stale"):
            continue
        r = ex.world.update(eid, point_mm=list(e["point_mm"]), data={"stale_cleared": why})
        if "refused" in r:
            continue
        s["observed_at"] = dict(r["observed_at"])     # the same observation: sync must not stamp it as new
        done.append(eid)
    return done


def relative(a, b):
    """The map taking frame B coordinates under pose a to those under pose b (fixed points): b^-1 a."""
    return fit.compose(fit.invert(b), a)


def _vec3(v):
    return (isinstance(v, (list, tuple)) and len(v) == 3
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v))


def _moved_data(data, rel):
    """An entry's frame B directions turned and its frame B points moved by the planar map rel; {} when it carries
    none."""
    c, s = math.cos(rel["yaw"]), math.sin(rel["yaw"])
    out = {}
    for key in DIRECTION_KEYS:
        v = (data or {}).get(key)
        if _vec3(v):
            out[key] = [round(c * v[0] - s * v[1], 6), round(s * v[0] + c * v[1], 6), float(v[2])]
    for key in POINT_DATA_KEYS:
        v = (data or {}).get(key)
        if _vec3(v):
            out[key] = [round(x, 3) for x in fit.apply(rel, v)]
    return out


def motion_text(a, b, at_mm):
    yaw, disp = fit.motion_size(relative(a, b), at_mm)
    return f"the base turned about {abs(yaw):.0f} deg and the point shifted about {disp:.0f} mm in frame B"


def _carry(ex, st, eid, s, cur, by, carry):
    """The same measured pose: the point carries over; its error grows by the motion the look measured under the
    move thresholds plus the estimate's error there (carry = {rel, err} of that look; None = nothing measured).
    A data-only update: the World keeps its observation time and any stale mark."""
    grow = 0.0
    if carry is not None:
        _, disp = fit.motion_size(carry["rel"], s["point_mm"])
        grow = disp + fit.error_at(carry["err"], s["point_mm"])
    new_err = None if s["error_mm"] is None else s["error_mm"] + grow
    r = ex.world.update(eid, data={"epoch": cur["epoch"], "error_mm": rnd(new_err), "sigma_p_mm": new_err,
                                   "carried": {"from_epoch": s["epoch"], "by": by, "grown_mm": rnd(grow)}})
    if "refused" in r:
        return False
    s["history"] = (s["history"] + [{"epoch": cur["epoch"], "point_mm": list(s["point_mm"]),
                                     "how": f"carried over from epoch {s['epoch']} by {by}"}])[-HISTORY_KEEP:]
    s.update(epoch=cur["epoch"], error_mm=new_err)
    return True


def reexpress(ex, st, by, skip=(), carry=None):
    """Bring every stamp with a known epoch pose into the present epoch (pose known). Entries in `skip` (the World
    marks them stale for a reason this measurement does not explain) and entries seen during this call (held, see
    restamp) are left as they are. Returns (re-expressed or carried ids, lost ids: their epoch's pose was never
    measured)."""
    cur = current(st)
    T, err_now = cur["pose"], cur["err"]
    skip = set(skip) | held(st)
    done, lost = [], []
    for eid, s in st["stamps"].items():
        if s["gone"] is not None or s["epoch"] == cur["epoch"] or eid in skip:
            continue
        e = ex.world.entries.get(eid)
        if e is None:
            continue
        rec = epoch_rec(st, s["epoch"])
        if rec is None or rec["pose"] is None:
            lost.append(eid)
            continue
        if rec["pose"] == T:
            if _carry(ex, st, eid, s, cur, by, carry):
                done.append(eid)
            continue
        rel = relative(rec["pose"], T)
        p_new = fit.apply(rel, s["point_mm"])
        grow = fit.error_at(rec["err"], s["point_mm"]) + fit.error_at(err_now, p_new)
        new_err = None if s["error_mm"] is None else s["error_mm"] + grow
        moved = math.dist(p_new[:2], s["point_mm"][:2])
        r = ex.world.update(eid, point_mm=p_new, data={"epoch": cur["epoch"], "error_mm": rnd(new_err), "sigma_p_mm": new_err,
                                                       **_moved_data(e.get("data"), rel),
                                                       "re_expressed": {"from_epoch": s["epoch"], "by": by,
                                                                        "moved_mm": rnd(moved)}})
        if "refused" in r:
            continue
        s["history"] = (s["history"] + [{"epoch": cur["epoch"], "point_mm": list(r["point_mm"]),
                                         "how": f"re-expressed from epoch {s['epoch']} by {by}"}])[-HISTORY_KEEP:]
        s.update(epoch=cur["epoch"], point_mm=list(r["point_mm"]), observed_at=dict(r["observed_at"]),
                 error_mm=new_err)
        done.append(eid)
    return done, lost


# ------------------------------------------------------------------ is a stored point usable now
def find(ex, key):
    """A positional entry id named by id, else by label the newest the World does not mark stale, else the newest
    (the rule approach resolves a target by, so the check is made on the entry the tool will use); else None."""
    key = str(key)
    e = ex.world.entries.get(key)
    if e is not None and e["kind"] in POSITIONAL and e["point_mm"] is not None:
        return key
    found = [eid for eid, e in ex.world.entries.items()
             if e["kind"] in POSITIONAL and e["point_mm"] is not None and e["label"].lower() == key.lower()]
    fresh = [eid for eid in found if not ex.world.entries[eid].get("stale")]
    return (fresh or found or [None])[-1]


def unusable(ex, st, eid):
    """None when the entry is usable in the present epoch, else (why, hint)."""
    e = ex.world.entries.get(eid)
    s = st["stamps"].get(eid)
    if e is None or s is None:
        return None
    now = ex.epoch
    head = f"{eid} ({e['label']}) was seen in epoch {s['seen_epoch']}, now {now}"
    if s["gone"] is not None:
        return f"{head}; {s['gone']}", s.get("gone_hint") or "look or touch again: what it marked has moved"
    if e.get("stale"):                        # before the epoch: a standard look does not lift the World's mark
        return (f"{head}; robot://world marks it stale (the world moved, {e['data'].get('stale_since', 'event')})",
                "look or touch again")
    if s["epoch"] != now:
        rec, cur = epoch_rec(st, s["epoch"]), current(st)
        if rec and rec["pose"] is not None and cur["pose"] is not None:
            what = motion_text(rec["pose"], cur["pose"], s["point_mm"])
        else:
            later = [r["why"] for r in st["epochs"] if r["epoch"] > s["epoch"]]
            what = "; ".join(later) or "the base may have moved"
            if rec is not None and rec["pose"] is None:
                what += f"; the base pose of epoch {s['epoch']} was never measured"
        return (f"{head}; {what}; it was not re-expressed",
                "take standard_look() so stored points are re-expressed, or look or touch again")
    return None


def point_check(ex, st, p):
    """None, or (eid, why, hint) when point p (mm) is a stored point that is no longer usable as given: a stale
    entry's point, or an older epoch's value of an entry that has since been re-expressed."""
    usable_match = False
    hit = None
    for eid, s in st["stamps"].items():
        e = ex.world.entries.get(eid)
        if e is None:
            continue
        bad = unusable(ex, st, eid)
        if math.dist(p, e["point_mm"]) <= MATCH_MM:
            if bad is None:
                usable_match = True
            elif hit is None:
                hit = (eid, bad[0], bad[1])
            continue
        for h in s["history"][:-1]:
            if math.dist(p, h["point_mm"]) <= MATCH_MM and hit is None:
                now = [round(float(x), 1) for x in e["point_mm"]]
                why = (f"{[round(float(x), 1) for x in p]} is where {eid} ({e['label']}) was in epoch {h['epoch']}, "
                       f"now {ex.epoch}")
                if bad is None:
                    hint = f"use its present point {now} (re-expressed) or reach_check(label='{eid}')"
                else:
                    why += "; " + bad[0].split("; ", 1)[-1]
                    hint = bad[1]
                hit = (eid, why, hint)
    if usable_match or hit is None:
        return None
    return hit


# ------------------------------------------------------------------ documents
def entries_doc(ex, st):
    out = []
    for eid, s in st["stamps"].items():
        e = ex.world.entries.get(eid)
        if e is None:
            continue
        bad = unusable(ex, st, eid)
        row = {"id": eid, "label": e["label"], "epoch": s["epoch"], "seen_epoch": s["seen_epoch"],
               "point_mm": e["point_mm"], "error_mm": rnd(s["error_mm"]), "usable": bad is None}
        if bad is not None:
            row["why"] = bad[0]
        out.append(row)
    return out


def epoch_doc(ex, st):
    """robot://epoch."""
    doc = {"epoch": ex.epoch, "base_pose": base_pose_doc(st),
           "frame": "W is frame B as it stood at the standard look reference (x forward, y left, z up, mm); "
                    "base_pose is frame B in W, its error_mm at the base origin; each stored point carries its own "
                    "error_mm at its place (smallest near where the standard look's camera points)",
           "history": [{"epoch": r["epoch"], "why": r["why"], "at_call": r["at_call"],
                        "pose_known": r["pose"] is not None} for r in st["epochs"]],
           "entries": entries_doc(ex, st),
           "base_give": copy.deepcopy(st["give"]) if st["give"] else {"stale": False, "note": "not measured yet"},
           "reach_checks": copy.deepcopy(st["reach"][-10:])}
    if st["estimate"]:
        doc["last_estimate"] = copy.deepcopy(st["estimate"])
    return doc
