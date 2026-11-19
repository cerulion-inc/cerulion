# SPDX-License-Identifier: AGPL-3.0-only
"""Lift: a lip from under its edge.

The declaration gate lists and enables lift only when r3_lift_declared is true;
its refusal offers the sheet's use route otherwise. P0 checks forbidden places
and refuses force_unknown along up before any look or approach.

A ray-only target is first felt along its naming ray inside reach and the floor
plane. The open side is the device-relative front, else the naming camera's
horizontal direction when resolved. The fingertip itself approaches below P
by the finger's thickness and the point's uncertainty band. A guarded profile
must pass the point by its own band plus rigid travel to prove it is under the
lip. Otherwise one finger-thickness descent tries again, then stops no_edge.
The upward touch reaches by the same uncertainty and rigid laws. A forbidden
felt point causes a downward and outward release without a lift.

The underside normal chooses the line when it agrees with up; the common ramp
then eases on motion or give and looks between legs. On a change the arm settles
and retracts down by shown travel and tool width, then out to its standoff.
The after witness and the sensed give back the result's words. A moved patch
marks the point stale and supplies a locate call with its new pixel where known.
No scene or scoring truth is read here.
"""

import numpy as np

from rcpx import r3_limits as L
from rcpx import sheet_words, side_words
from rcpx.context import refusal
from rcpx.r2 import act, act_ladder, act_words
from rcpx.r2 import act_engine as E
from rcpx.r2 import act_guard as G
from rcpx.r2.act_engine import Stop

UP = np.array([0.0, 0.0, 1.0])
LIFT_WORDS = ("lift", "lifting", "lifted")


def _d5_hint(ex, part):
    """The sheet's use route when lift is not enabled."""
    tools = G.tools(ex) - {"lift"}
    dl = sheet_words.do_line(part, G.devices_doc(ex), tools | {"approach", "touch", "press"}, G._body(ex)) \
        if part else None
    if dl and dl.get("verbs") and "press" in dl["verbs"]:
        return dl["do"]
    return ("lift is not enabled here: robot://devices gives each part's do line; an upward nudge runs through use "
            "(approach, touch up, press up)")


def _lift_forbidden(part):
    """The sheet clause forbidding a lift, or None."""
    for item in (part or {}).get("approach_forbidden") or []:
        low = str(item).lower()
        if any(w in low.replace(",", " ").split() for w in LIFT_WORDS):
            return item
    return None


def run_lift(ex, target, max_n=None, firm=False, detail="brief"):
    """Check the declaration and P0, then run and report the lift stages."""
    dark = ex._dark_refusal_now()
    if dark is not None:
        return act._said_refusal(dark, "lift")
    if not G.lift_declared(ex):
        e, _ = G.resolve(ex, target) if isinstance(target, str) else (None, None)
        part = G.target_info(ex, e)["part"] if e is not None else None
        return act._said_refusal(refusal("lift_gate", "lift is not enabled on this robot yet", _d5_hint(ex, part)),
                                 "lift")
    run = E.Run(ex, "lift")
    run.detail = detail
    pre = G.preflight(ex, target, "in", "give", firm, max_n, 0, detail, verb="lift", B=run.B)
    if isinstance(pre, dict):
        return act._said_refusal(pre, "lift")
    info = pre[0]
    bad = _lift_forbidden(info["part"])
    if bad is not None:
        return act._said_refusal(refusal("forbidden", f'robot://devices says "{bad}"',
                                         "choose another part; robot://devices lists them"), "lift")
    st = G.state(ex)
    st["busy"] = True
    try:
        out = _stages(ex, run, info, firm, max_n)
    finally:
        st["busy"] = False
    if isinstance(out, dict) and "refused" in out:
        return out
    return out if detail == "full" else act_words.brief(out)


def _open_side(info, P):
    """The open horizontal side from the device centre or naming camera."""
    o = side_words.front(info["centre"], P)
    if o is not None:
        return o
    if info["origin"] is None:
        return None
    h = np.asarray(info["origin"], dtype=float) - np.asarray(P, dtype=float)
    hh = np.array([h[0], h[1], 0.0])
    n = float(np.linalg.norm(h))
    if n > 0 and float(np.linalg.norm(hh)) >= L.RAY_H_MIN * n:
        return hh / float(np.linalg.norm(hh))
    return None


def _stages(ex, run, info, firm, max_n):
    """Profile under the edge, feel up, ramp and witness the lift."""
    st = G.state(ex)
    part, unc, label = info["part"], info["uncertain_mm"], info["label"]
    P = info["P"]
    felt, presses, peak, ceiling_n, contact, stopped, fields = [], [], 0.0, None, None, None, {}
    before = after = record = None
    res = {}
    run.target_id, run.target_sigma_mm = info["id"], info["uncertain_mm"]
    thing = L.thing_of(run.B, info["id"], sigma_mm=info["uncertain_mm"])
    q_w = act.witness_q(ex, info, run.B)
    run.profile_out = 0.0
    try:
        initial = L.force_spec(run.B, "press", thing, ex._sense().q, UP,
                               intent={k: v for k, v in {"max_n": max_n, "firm": firm}.items() if v is not None}, sheet_forces=(part or {}).get("forces"),
                               moved_n=act.moved_of(st, info), local=run.local)
        run.items, ceiling_n = initial["items"], initial["ceiling_n"]
        if P is None:
            ray = info["ray"]
            tgt = info["origin"] + ray * L.ray_only_ahead_mm(run.B, model=ex.model)
            run.approach_line(tgt, ray, 0.0, unc, P=tgt, source="single_look")
            noise = run.noise(ray)
            if noise is not None:
                st["force_noise_n"] = round(noise, 3)
            ft = round(L.f_touch(run.B, readings=1, local=run.local), 2)
            r = run.touch_confirm(ray, act._floor_bound(ex, run.tip(), ray), ft, None, unc, label)
            P = np.asarray(r["point_mm"], dtype=float)
            res["ray_only_point_mm"] = E._r(P, 1)
            run.move("ray_back", -ray * L.standoff_mm(run.B, ex._sense().q, ray, thing=thing))
        o = _open_side(info, P)
        if o is None:
            return act._said_refusal(refusal("no_side", "no open side is known for this part: no device centre and "
                                             "the naming view looks straight down at it",
                                             "survey, then view the part from its front and locate it there"), "lift")
        res["open_side"] = E._r(o, 3)
        before = act.witness_look(run, q_w, f"before lift {info['id']}")
        h0 = float(P[2]) - (L.lift_drop_mm(run.B, sigma_mm=unc))
        T = np.array([P[0], P[1], h0])
        # The fingertip itself goes under the edge. A leading tool surface
        # would put the hand where the profile cannot read that edge.
        run.approach_line(T, -o, L.standoff_mm(run.B, ex._sense().q, -o, thing=thing), unc, P=T, source="triangulated", tip_only=True)
        noise = run.noise(-o)
        if noise is not None:
            st["force_noise_n"] = round(noise, 3)
        noise = float(st["force_noise_n"] or 0)
        ft = round(L.f_touch(run.B, readings=1, local=run.local), 2)
        under = _profile(ex, run, P, o, ft, unc, thing, E.largest_size(part))
        if under is None:
            run.move("profile", [0.0, 0.0, -L.lift_lower_mm(run.B)])
            under = _profile(ex, run, P, o, ft, unc, thing, E.largest_size(part))
        if under is None:
            raise Stop("no_edge", why=f"the face under the {label} is flush: met it {run.profile_out:.0f} mm out")
        up_mm = L.lift_drop_mm(run.B, sigma_mm=unc) + L.touch_reach_mm(run.B, ex._sense().q, UP, thing=thing, sigma_mm=unc)
        r = run.touch("touch_up", UP, up_mm, ft)
        if not (isinstance(r, dict) and r.get("contact")):
            raise Stop("no_edge", why=f"nothing above the finger for {up_mm:.0f} mm")
        if r.get("source") not in (None, "tool") or r.get("at_tip") is False:
            raise Stop("blocked", what=", ".join(r.get("likely_links") or []) or "the side of the tool", mm=0)
        contact = {"id": r.get("id"), "point_mm": r.get("point_mm"), "normal": r.get("normal_est")}
        # The felt place is checked again against forbidden points, as for
        # push. A trusted label is no permission to lift a forbidden surface.
        hit = G.forbidden_at(ex, r["point_mm"]) if r.get("point_mm") is not None else None
        if hit is not None:
            run.move("release", [0.0, 0.0, -L.retract_mm(run.B, ex._sense().q, -UP, travel_mm=sum(max(0, r["along_mm"]) for r in run.ramps))])
            run.move("release", o * L.standoff_mm(run.B, ex._sense().q, -o, thing=thing))
            raise Stop("forbidden", what=hit[0], mm=hit[2], keep=hit[3])
        n = E._unit(r.get("normal_est") or [0, 0, -1])
        u = -n if n is not None and float((-n)[2]) > L.VERTICAL_Z else UP.copy()
        res["line"] = {"u": E._r(u, 3), "source": "underside" if u is not UP and not np.allclose(u, UP) else "up"}
        facts = L.inputs(ex, B=run.B)
        q_c = ex._sense().q
        # the travel that shows the part moved (never zero: L.moved_mm) and twice that, motion for certain
        rigid = L.moved_mm(run.B, q_c, u, v_mm_s=getattr(run, "touch_v_mm_s", None), thing=thing)
        small = L.ratio("QUANT") * rigid
        spec = L.force_spec(run.B, "press", thing, q_c, u, intent={k: v for k, v in {"max_n": max_n, "firm": firm}.items() if v is not None},
                            sheet_forces=(part or {}).get("forces"), moved_n=act.moved_of(st, info), touched_n=r.get("force_n"), local=run.local)
        ceiling_n = spec["ceiling_n"]
        res.update(noise_n=noise, hidden_n=run.local[0] if run.local else 0, f_touch_n=ft, f0_n=spec["first_n"], rigid_mm=rigid,
                   reach_n=run.B.f_cap(q_c, u))
        contact["force_n"] = r.get("force_n")
        stopped, fields, felt, presses, peak, _ = act._ramp(ex, run, info, u, contact, thing, "give", firm,
                                                          max_n, 0, rigid, small)
        if stopped == "changed":
            run.settle()
            run.move("release", [0.0, 0.0, -L.retract_mm(run.B, ex._sense().q, -UP, travel_mm=sum(max(0, r["along_mm"]) for r in run.ramps))])
            r = run.move("release", o * L.standoff_mm(run.B, ex._sense().q, -o, thing=thing))
            if isinstance(r, dict) and r.get("stopped") in ("contact", "cap"):
                fields["extra"] = act_words.LEFT_SAID
        elif felt:
            run.move("release", [0.0, 0.0, -L.retract_mm(run.B, ex._sense().q, -UP, travel_mm=sum(max(0, r["along_mm"]) for r in run.ramps))])
            run.move("release", o * L.standoff_mm(run.B, ex._sense().q, -o, thing=thing))
    except Stop as stop:
        stopped, fields = stop.stopped, dict(stop.fields)
        done = fields.pop("ladder", None)
        if done is not None:
            felt, presses, peak, _ = done
        if stopped == "dark" and run.moved:
            try:
                ex.retreat(steps=1)
            except Exception:
                pass
    if felt and stopped != "dark":
        try:
            after = act.witness_look(run, q_w, f"after lift {info['id']}")
        except Stop as stop:
            run.notes.append(f"the after look stopped {stop.stopped}")
        record = act.witness_compare(ex, before, after, act.at_list(ex, info))
    # A change or end claim stands only on the sensed and pictured
    # evidence (act.backed).
    stopped, fields = act.backed(stopped, fields, record, getattr(run, "step_seen", False), None, "way", felt)
    if run.ramps:
        ceiling_n = run.ramps[-1]["ceiling_n"]
    return _result(ex, run, info, stopped, fields, felt, presses, peak, ceiling_n, contact, before, after, record,
                   res, label)


def _profile(ex, run, P, o, ft, unc, thing, size_mm=None):
    """
    A guarded touch along -o; the finger's depth past P (mm) when it is under the lip, else None
    (run.profile_out:
    how far out it met the face).
    """
    r = run.touch("profile", -o, L.lift_profile_mm(run.B, ex._sense().q, -o, thing=thing, sigma_mm=unc, size_mm=size_mm), ft)
    tip = run.tip()
    if isinstance(r, dict) and r.get("contact") and r.get("point_mm") is not None:
        depth = float((np.asarray(r["point_mm"], dtype=float) - P) @ -o)
        if depth >= L.lift_under_mm(run.B, ex._sense().q, -o, sigma_mm=unc):
            run.move("profile", o * L.confirm_back_mm(run.B, ex._sense().q, -o))
            return depth
        run.profile_out = max(0.0, -depth)
        run.move("profile", o * L.confirm_back_mm(run.B, ex._sense().q, -o))
        return None
    depth = float((tip - P) @ -o)
    if depth >= L.lift_under_mm(run.B, ex._sense().q, -o, sigma_mm=unc):
        back = depth - (L.lift_under_mm(run.B, ex._sense().q, -o, sigma_mm=unc) + L.confirm_back_mm(run.B, ex._sense().q, -o))
        if back > 0:
            run.move("profile", o * back)
        return depth
    run.profile_out = max(0.0, -depth)
    return None


def _result(ex, run, info, stopped, fields, felt, presses, peak, ceiling_n, contact, before, after, record, res,
            label):
    """Report a lift and store its measured force and witness evidence."""
    st = G.state(ex)
    seen = (record or {}).get("said") or act_words.seen_missing(ex, before, after)
    f = {"label": label, "seen": seen, "dir": "up", **{k: v for k, v in fields.items() if v is not None}}
    for k, v in (("stage", run.stage), ("check", "a gate"), ("s", round(run.elapsed(), 0)), ("what", "no lift made"),
                 ("mm", 0), ("why", "no tool surface reaches it"), ("n", round(peak, 1)), ("how", "stayed in"),
                 ("where", "the way"), ("near", "near no located point"), ("forbidden", "")):
        f.setdefault(k, v)
    if stopped == "ceiling":
        f["why"] = fields.get("why") or act_words.ceiling_why(run)
    said = act_words.lift_said(stopped, **f)
    wit, moved = None, {}
    if record is not None:
        at = record.get("at") or []
        mine = next((a for a in at if a.get("id") == info["id"]), at[0] if at else {})
        wit = {"id": record.get("record") or record.get("id"), "before": (before or {}).get("look_id"),
               "after": (after or {}).get("look_id"), "changed": mine.get("changed")}
        if mine.get("moved_px") is not None:
            moved = {"moved_px": mine.get("moved_px"), "moved_mm": mine.get("moved_mm")}
            ex.world.mark_stale([info["id"]], "it moved with the change (lift)", source="act")
    tools = G.tools(ex)
    look = (after or {}).get("look_id")
    if stopped == "changed" and "locate" in tools and look:
        if moved.get("moved_px") is not None:
            nxt = {"tool": "locate", "args": {"look": look, "px": moved["moved_px"], "part": label},
                   "why": "it moved with the change: locate it again before the next push"}
        else:
            nxt = act_words.point_call(look, label, "it moved with the change: locate it again before the next push")
    elif stopped == "changed":
        nxt = {"tool": "look", "args": {}, "why": "it moved with the change: find it again before the next push"}
    else:
        nxt = act_words.next_for(stopped, {"tools": tools, "target": info["id"], "ceiling_n": ceiling_n})
    peak = max(float(peak or 0.0), float(run.peak_total))      # the most force read in any stage of this call
    out = {"said": said, "stopped": stopped, "felt": felt, "peak_n": round(float(peak), 1), "ceiling_n": ceiling_n,
           "line": res.pop("line", {"u": [0.0, 0.0, 1.0], "source": "up"}),
           "contact": contact, "seen": seen, "witness": wit, **moved,
           "by": "runtime", "pictures": list((record or {}).get("pictures") or [])[:L.PICTURES_MAX], "next": nxt}
    if moved.get("moved_mm") is not None:
        out["moved_dir"] = "up"
    act_words.report_bounds(out, run)
    act_words.add_refusal(out, fields, info, "lift")
    if fields.get("check"):
        out["check"] = fields["check"]
    if run.detail == "full":
        out.update({k: fields[k] for k in ("why", "reasons") if k in fields})
    if run.notes:
        out["notes"] = run.notes[:4]
    out.update({"target": info["id"], "part": (info["part"] or {}).get("name"), "stages": run.legs,
                "touches": run.touches, "presses": presses, "peak_total_n": round(run.peak_total, 2),
                "sim_s": round(run.elapsed(), 2), **res})
    st["acts"].append({"seq": ex._calls, "tool": "lift", "target": info["id"], "label": label,
                       "part": (info["part"] or {}).get("name"), "device": info["device"], "stopped": stopped,
                       "felt": felt, "peak_n": out["peak_n"], "ceiling_n": ceiling_n, "travel_mm": 0.0,
                       "witness": wit, "contact": contact, "point_mm": None if info["P"] is None else
                       E._r(info["P"], 1), "last_n": felt[-1]["n"] if felt else None, "uncertain_mm": info["uncertain_mm"],
                       **act_words.force_evidence(run)})
    for p in presses:
        st["presses"].append({"call": ex._calls, "tool": "lift", "start_mm": p["start_mm"], "force_n": p["n"],
                              "along_mm": p["along_mm"], "gave": p["gave"], "rigid": p["word"] == "rigid"})
    return out

__all__ = ["run_lift"]
