# SPDX-License-Identifier: AGPL-3.0-only
"""Package probe, the report: what one push (probe_push.run) says about the part, and no more than that.

kind is prismatic or revolute only when the fit (probe_fit.classify) gives that answer AND the tip was not seen to
slide on the part by more than a little AND, for revolute, the path went on bending at the fitted rate while the
push direction was held fixed (probe_push.tail_turn: a bend that stops there was the push's own; a push the part
itself ended, by holding or by giving way, keeps an unchecked bend with a note, a push cut short does not). stuck
only when the part held from the first step; free when nothing pushed back, or when the part stopped pushing back;
unknown, with why, for everything else: a path too short to tell a slide from a hinge, a path neither fits, a
measured slide, a bend that followed the push, a push that ended early. An unknown result still gives motion_dir
(the way the tip went with the part) when the path has a direction.

The fit is written to the world as a device entry (source "probe", confirmed false) and to
ex.state["probe"]["fits"][label] whenever the part was met and either held or moved along a direction. No robot or
device is named here, and nothing reads simulator ground truth.
"""

import math

import numpy as np

from rcpx.r2 import probe_fit as fit
from rcpx.r2 import probe_push as push
from rcpx.r2 import probe_bounds as derived
BEND_SHARE = 0.5             # with the push direction held, a hinge bends the path by at least this share of what
                             # the fitted circle says; less, and the bend was the push direction's own turning
MM = 1000.0


def _r(v, nd=2):
    return None if v is None else [round(float(x), nd) for x in v]


def held_bend(cl, tail):
    """For a path the fit reads as a hinge: how far it bent over the stretch pushed along one fixed direction
    against what the fitted circle says, {"held_deg", "expected_deg", "ok"}; None when the fit is no hinge or the
    push ended before that stretch was made (tail None)."""
    if cl["kind"] != "revolute" or not tail:
        return None
    turn, length = tail
    expected = length / cl["circle"]["radius"]
    return {"held_deg": round(math.degrees(turn), 2), "expected_deg": round(math.degrees(expected), 2),
            "ok": bool(turn >= BEND_SHARE * expected)}


def decide(cl, stop, travel, slid, force_n, bend=None, *, B, q=None, u=None, planned_m=None, force_now=None):
    """(kind, why) from the fit `cl`, the way the push ended, the path length (m), the tip's measured slide on
    the part (m) and the held bend (held_bend), for a part that was met. Pure: the whole decision is here, so it
    can be tested by hand."""
    force_now = force_n if force_now is None else force_now
    kind, why, mm = cl["kind"], cl["why"], f"{travel * MM:.1f} mm"
    if stop == "cap":
        return "unknown", (f"the push ended (cap) after {mm}; how it moves is not known: "
                           "the push was cut short before its motion could be checked")
    if kind is not None and slid > B.rigid(q, u, force_now):
        kind, why = None, (f"the fit reads {kind}, but the tip slid {slid * MM:.1f} mm on the part during the push: "
                           "its path is not the part's own, so the fit is not trusted")
    elif kind == "revolute" and bend is not None and not bend["ok"]:
        kind, why = None, (f"the fit reads revolute, but with the push direction held fixed the path bent "
                           f"{bend['held_deg']:g} deg where that hinge would bend it {bend['expected_deg']:g} deg: "
                           "the bend followed the push, so the tip was sliding on the part")
    elif kind == "revolute" and bend is None and stop not in ("goal", "stall", "released"):
        kind, why = None, ("the fit reads revolute, but the push was cut short before the bend could be checked "
                           "with the push direction held fixed")
    if stop == "stall" and travel < B.rigid(q, u, force_now):
        return "stuck", (f"the part moved {mm} and then held against {force_n:g} N: it does not move this way at "
                         "this force")
    if kind is not None:
        return kind, why
    if stop == "released":
        return "free", f"the part stopped pushing back after {mm}: it gave way, or the tip left it"
    how = {"stall": f"the part moved {mm} and then held against {force_n:g} N (a stop or a stiff point, or it needs more force)",
           "goal": f"the part moved {mm}"}.get(stop, f"the push ended ({stop}) after {mm}")
    return "unknown", f"{how}; how it moves is not known: {why}"


def build(ex, st, run, tgt, d0, label, force_n, max_mm, back_m=0.0, force_after=0.0):
    """The result of probe_motion from one run. back_m: how far the tip came back along the push when the push was
    let go (m); force_after: the tip force after the closing ease (N)."""
    B = derived.of(ex)
    pose = ex._sense()
    pts = np.asarray(run["points"], dtype=float).reshape(-1, 3)
    stop = run["stop"]
    met = run["contact"] is not None
    slid = float(run["slid_m"])
    cl = fit.classify(pts, B, pose.q, d0, ex.model.length_scale) if met else {"kind": None, "why": "", "line": None, "circle": None, "short": False,
                                        "hinge_ruled_out": None}
    travel = fit.path_length(pts) if met else 0.0
    line, circle = cl.get("line"), cl.get("circle")
    resisted = None
    sprang = met and (back_m > B.rigid(run["s"].q, run["d"], max(0.0, float(run["s"].F @ run["d"]))) or force_after > B.f_thr(0))
    bend = held_bend(cl, run.get("tail_turn")) if met else None
    if met:
        kind, why = decide(cl, stop, travel, slid, force_n, bend, B=B, q=pose.q, u=d0, planned_m=max_mm / MM, force_now=max(0.0, float(run["s"].F @ run["d"])))
    elif stop == "no_contact":
        kind, why = "free", f"nothing pushed back within {run['free_m'] * MM:.0f} mm along the push"
        felt = run["felt"].report()
        if felt["moved_part"]:      # under the threshold that counts as meeting the part, yet it resisted all along
            resisted = felt["resisted_mm"]
            why = (f"nothing pushed back with the {run.get('touch_force_n', push.touch_force(B, force_n)):.2f} N that counts as meeting a part "
                   f"within {run['free_m'] * MM:.0f} mm, but something lighter resisted (up to "
                   f"{felt['peak_force_n']} N over {resisted} mm): a light part may have been pushed along")
    else:
        kind, why = "unknown", f"the push ended ({stop}) before the part was met"
    out = {"kind": kind, "axis_dir": None, "center_mm": None, "radius_mm": None,
           "travel_mm": round(travel * MM, 2), "residual_mm": None, "peak_force_n": round(run["peak_n"], 2),
           "path_mm": [_r(p * MM, 3) for p in pts], "why": why, "stopped": stop, "contact": met,
           "steps": run["steps"], "free_mm": round(run["free_m"] * MM, 2), "motion_dir": None,
           "asked": {"direction": _r(d0, 3), "max_mm": max_mm, "force_n": force_n},
           "source": "probe", "confirmed": False}
    if resisted is not None:
        out["resisted_mm"] = resisted
    if kind == "prismatic":
        out.update(axis_dir=_r(line["dir"], 4), residual_mm=round(line["rms"] * MM, 4),
                   motion_dir=_r(line["dir"], 4), hinge_ruled_out_mm=round(cl["hinge_ruled_out"] * MM))
    elif kind == "revolute":
        out.update(axis_dir=_r(circle["axis"], 4), center_mm=_r(circle["center"] * MM, 1),
                   radius_mm=round(circle["radius"] * MM, 1), residual_mm=round(circle["rms"] * MM, 4),
                   arc_deg=round(math.degrees(circle["arc_rad"]), 1),
                   motion_dir=_r(fit.tangent_at(circle, pts[-1]), 4), bend_checked=bend is not None)
    elif met and line is not None:      # no kind, but the tip went somewhere with the part
        out["motion_dir"] = _r(line["dir"], 4)
    if line is not None:
        out["fits"] = {"line_residual_mm": round(line["rms"] * MM, 4),
                       "circle_residual_mm": None if circle is None else round(circle["rms"] * MM, 4),
                       "samples_used": cl.get("used"), "samples_left_out": cl.get("dropped"),
                       "kind_by_fit": cl["kind"]}
        if bend is not None:
            out["fits"].update(bend_with_direction_held_deg=bend["held_deg"], bend_expected_deg=bend["expected_deg"])
    if met:
        out["contact_mm"] = _r(run["contact"]["point"] * MM)
        out["force_by_step_n"] = [round(float(f), 2) for f in run["forces"]]
    notes = []
    if stop == "stall" and kind in ("prismatic", "revolute"):
        notes.append(f"the part stopped after {travel * MM:.1f} mm under {force_n:g} N: a stop or a stiff point, or it "
                     "needs more force to go on")
    if kind == "revolute" and bend is None:
        notes.append("the push ended before the bend could be checked with the push direction held fixed: if the "
                     "tip was sliding on the part, the bend may be the push's own")
    if kind == "unknown" and met and cl.get("short") and travel > 0:
        notes.append("to tell a slide from a hinge the part must travel further under the tip: probe again with "
                     "more max_mm, or with more force_n if it held")
    if slid > B.rigid(pose.q, d0, run["peak_n"]):
        out["slid_mm"] = round(slid * MM, 2)
        if kind in ("prismatic", "revolute"):
            notes.append(f"the tip slid {slid * MM:.1f} mm on the part while it stood still; that slide is taken "
                         "out of the path, but the fit is less sure (a centre least of all)")
    if stop == "cap":
        notes.append(run["cap_why"])
    elif stop == "limit":
        notes.append("the arm could not follow the part further: it reached a joint limit or its reach")
    elif stop == "slipped":
        notes.append("the tip ran off sideways more than along the push: it slid on the part")
    elif run.get("tick"):
        notes.append(f"stopped by a watch of another package ({run['tick']})")
    if sprang:
        out["sprang_back_mm"] = round(max(back_m, 0.0) * MM, 2)
        notes.append(f"the part pushed the tip back {max(back_m, 0.0) * MM:.1f} mm when the push was let go"
                     + (f" and still presses on it with {force_after:.1f} N" if force_after > B.f_thr() else "")
                     + ": it springs back; retreat to let it return")
    if notes:
        out["notes"] = notes
    out["ref"] = f"PM{ex._calls}"
    if kind in ("prismatic", "revolute", "stuck") or (kind == "unknown" and out["motion_dir"] is not None):
        name = (label.strip() if isinstance(label, str) else None) or tgt["label"]
        if not name:
            st["n"] += 1
            name = f"probed part {st['n']}"
        out["label"] = name
        data = {k: out[k] for k in ("kind", "axis_dir", "center_mm", "radius_mm", "travel_mm", "residual_mm",
                                    "peak_force_n", "motion_dir", "ref")}
        data.update(sigma_force_n=B.sigma_f(), force_hidden_n=B.f_hidden(), pushed_along=out["asked"]["direction"], force_n=force_n, epoch=ex.epoch,
                    target_id=tgt["id"], hypothesis=True)
        old = next((e for e in ex.world.entries.values()
                    if e["kind"] == "device" and e["source"] == "probe" and e["label"] == name), None)
        if old is None:
            entry = ex.world.add("device", name, point_mm=out["contact_mm"], source="probe", confirmed=False,
                                 data=data)
        else:
            entry = ex.world.update(old["id"], point_mm=out["contact_mm"], data=data, confirmed=False)
        out["world_id"] = entry.get("id")
        st["fits"][name] = {**data, "contact_mm": out["contact_mm"], "world_id": out["world_id"]}
        ex.world.note_sense("probe", out["ref"], summary={"kind": kind, "travel_mm": out["travel_mm"],
                                                          "peak_force_n": out["peak_force_n"]})
    return out
