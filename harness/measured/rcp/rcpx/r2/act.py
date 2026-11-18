# SPDX-License-Identifier: AGPL-3.0-only
"""Package act (I34, SPEC-R3 4.3 and SPEC-R4 A4.5): push and lift as whole steps.

push chooses its line from the stored point, normal, device geometry or naming
ray. P0 refuses force_unknown before any look or approach when no sheet force,
stated force or earlier motion of this part or its device supports the push.
The hint gives one ready max_n call at the hand's own weight along that line.
firm is the operator's word to use the robot side of the ceiling.

The approach feels and confirms contact, then fits the face with surrounding
touches at an offset derived from point uncertainty and the tool's half width.
One continuous ramp leg starts at the finding touch, never under F_thr. Its
rate follows force noise, torque error and detection time. A give leg eases on
resolved motion beyond what earlier legs showed. A return or small move below
the ceiling resumes at the last reading with shown_mm raised to the greatest
travel so far. A same-view look between legs can show the change of a part that
returns by design. A stop leg follows to the part's own end. Every leg retains
one felt entry and one press record, including a stop inside the loop.

A ray-only point is found by touch first. A steep naming ray that meets a
surface above a sliding part gets one level approach. A stayed-in move gets a
post-release touch: a return without pushing the tool back otherwise looks like
it stayed. A fresh seating touch prevents the ramp striking across a gap.
Each of these checks stops a failure seen in measured episodes; no old typed limit
survives as a default. The release leaves by a guarded retract. A give or end
claim needs the sensed and pictured evidence described by act_evidence.

lift is listed only with r3_lift_declared. It profiles under an edge, touches up
and uses the same ramp and evidence. The carry press guard refuses cue contact
and a third rising press on a spot that twice read rigid.

Full results retain ceiling_source, bounds, periods and phase records as well
as felt, presses, touch evidence and pictures. The brief keeps one sentence and
one binding bound inside the byte envelope. ex.state["act"] holds acts with
f_moved_n, f_bear_n and uncertain_mm, busy, presses, force_noise_n and
force_hidden_n. Both noise values begin None and become the local free hold's
readings. No scoring truth enters the package.

The witness and sight packages are read lazily. Every stage emits r3_leg with
runtime origin, its motion, time, periods and the bound's source. The dark gate
reads light again before a nested motion and permits one retreat after a stop.
"""

import math

import numpy as np

from rcpx import r3_limits as L
from rcpx import sheet_words, side_words
from rcpx import surfaces as S
from rcpx.control_base import primitive
from rcpx.r2 import act_engine as E
from rcpx.r2 import act_guard as G
from rcpx.r2 import act_ladder, act_words
from rcpx.r2.act_engine import Stop
from rcpx.r2.act_evidence import (TRUSTED_LINES, StepWatch, at_list, backed, line_trust,
                                  witness_compare, witness_look, witness_q)

ID = "I34"
DIRECT = ("push", "lift")
NO_PUSH = ("not_found", "blocked", "cannot_reach", "not_square", "forbidden", "struck")   # no press was made
state, lift_declared, preflight = G.state, G.lift_declared, G.preflight


def _said_refusal(r, verb="push"):
    """Lead each refusal with one bounded sentence saying no push or lift and why."""
    why = ((r.get("refused") or {}).get("why") or "").rstrip(".")
    return {"said": act_words.cut_words(f"no {verb}: {why}", L.SAID_MAX_WORDS), **r}


def run_push(ex, target, way="in", until=None, firm=False, max_n=None, hold_s=0, detail="brief"):
    """Check P0, run push stages and return the requested full or brief result."""
    run = E.Run(ex, "push")
    run.detail = detail
    pre = preflight(ex, target, way, until, firm, max_n, hold_s, detail, B=run.B)
    if isinstance(pre, dict):
        return _said_refusal(pre)
    info, u, src, until = pre
    st = state(ex)
    st["busy"] = True
    try:
        out = _push_stages(ex, run, info, u, src, way, until, firm, max_n, float(hold_s))
    finally:
        st["busy"] = False
    return out if detail == "full" else act_words.brief(out)


def _push_stages(ex, run, info, u, src, way, until, firm, max_n, hold_s):
    """Approach, confirm, fit, ramp, release and witness a push, retaining every completed stage on an interruption."""
    st = state(ex)
    P, unc, part = info["P"], info["uncertain_mm"], info["part"]
    label = info["label"]
    sliding = bool((part or {}).get("sliding"))
    res = {"u0": E._r(u), "line_source": src, "until": until}
    felt, presses, before, after, record, stopped, fields = [], [], None, None, None, None, {}
    peak_n, ceiling_n, contact, travel = 0.0, None, None, 0.0
    run.target_id, run.target_sigma_mm = info["id"], info["uncertain_mm"]
    thing = L.thing_of(run.B, info["id"], sigma_mm=info["uncertain_mm"])
    q_w = witness_q(ex, info, run.B)
    try:
        initial = L.force_spec(run.B, "press", thing, ex._sense().q, u,
                               intent={k: v for k, v in {"max_n": max_n, "firm": firm}.items() if v is not None}, sheet_forces=(part or {}).get("forces"),
                               moved_n=moved_of(st, info), local=run.local)
        run.items = initial["items"]
        ceiling_n = initial["ceiling_n"]
        before = witness_look(run, q_w, f"before push {info['id']}")
        if before is None:
            run.notes.append(act_words.NO_BEFORE)
        standoff = L.standoff_mm(run.B, ex._sense().q, u, thing=thing)
        first = None
        # A steep view of a sliding part met its top in the review probes.
        # The line is level from the start, rather than loading it across its travel.
        if sliding and P is not None and src == "naming_ray" and abs(float(u[2])) > L.VERTICAL_Z:
            u, src = side_words.radial(P), "radial"
            res.update(u0=E._r(u), line_source=src)
            run.notes.append("a part that slides is not pushed down along a view from above: the line is level")
        # A naming ray carries no depth. Feel that ray first; a stored point
        # cannot be invented from it before the confirmed contact.
        if P is None:
            ray = info["ray"]
            tgt = info["origin"] + ray * L.ray_only_ahead_mm(run.B, model=ex.model)
            run.approach_line(tgt, ray, 0.0, unc, P=tgt, source="single_look")
        else:
            run.approach_line(P, u, standoff, unc, P=P)
        noise = run.noise(u)
        if noise is not None:
            st["force_noise_n"] = round(noise, 3)
        noise = float(st["force_noise_n"] or 0)
        ft = round(L.f_touch(run.B, readings=1, local=run.local), 2)
        facts = L.inputs(ex, B=run.B)
        rigid = L.rigid_mm(run.B, ex._sense().q, u)
        tol = L.confirm_tol_mm(run.B, ex._sense().q, u, v_mm_s=0, force_n=ft)
        max_mm = standoff + L.touch_reach_mm(run.B, ex._sense().q, u, thing=thing, sigma_mm=unc)
        if float(u[2]) < 0:      # a line going down feels no further than the floor (the path check's plane)
            max_mm = min(max_mm, max(0, _floor_bound(ex, run.tip(), u) - L.floor_keep_mm(run.B)))
        if P is None:
            first = run.touch_confirm(u, _floor_bound(ex, run.tip(), u), ft, None, unc, label)
            P = np.asarray(first["point_mm"], dtype=float)
            res["ray_only_point_mm"] = E._r(P, 1)
            u2, src2 = E.line(way, info["normal"], info["centre"], info["top"], P, info["ray"], run.B, unc)
            if u2 is None:
                raise Stop("refused", stage="line", check="way_up")
            if side_words.angle_deg(u2, u) > L.fit_skip_deg(run.B, force_n=ft):
                run.move("line", -u * standoff)
                u, src = u2, src2
                res["line_source"] = src
                run.approach_line(P, u, standoff, unc, P=P, source="touch")
                first = None
        try:
            c, u, fit = _touch_and_fit(ex, run, info, u, max_mm, ft, P, tol, unc, standoff, src, first=first)
        # Only a sliding part gets the level retry. Review probes on other
        # faces pressed beside the part and changed a cue without proving the spot.
        except Stop as stop:
            if stop.stopped != "blocked" or src != "naming_ray" or abs(float(u[2])) <= L.VERTICAL_Z or not sliding:
                raise
            u, src = side_words.radial(P), "radial"
            res.update(line_source=src, retried="the first line met a surface above the point: approached level")
            run.notes.append(f"met a surface {stop.fields.get('mm')} mm before the point from above: approached "
                             "level")
            run.move("line", [0.0, 0.0, L.retract_up_mm(run.B, ex._sense().q, thing=thing)])          # off what it met, then back before P's face
            ahead = float((run.tip() - P) @ u) + standoff + L.retract_mm(run.B, ex._sense().q, u)
            if ahead > 0:
                run.move("line", -u * ahead)
            run.approach_line(P, u, standoff, unc, P=P)
            c, u, fit = _touch_and_fit(ex, run, info, u, max_mm, ft, P, tol, unc, standoff, src)
        contact = c
        # The touched place matters, not the label believed before contact
        # (act_guard: forbidden by WHERE). A forbidden felt point backs off before any ramp.
        hit = G.forbidden_at(ex, c["point_mm"])
        if hit is not None:
            run.retract(-u, L.retract_mm(run.B, ex._sense().q, u))
            raise Stop("forbidden", what=hit[0], mm=hit[2], keep=hit[3])
        q_c = ex._sense().q
        # the travel that shows the part moved (never zero: L.moved_mm) and twice that, motion for certain
        rigid = L.moved_mm(run.B, q_c, u, v_mm_s=getattr(run, "touch_v_mm_s", None), thing=thing)
        small = L.ratio("QUANT") * rigid
        intent = {k: v for k, v in {"max_n": max_n, "firm": firm}.items() if v is not None}
        spec = L.force_spec(run.B, "press", thing, q_c, u, intent=intent, sheet_forces=(part or {}).get("forces"),
                            moved_n=moved_of(st, info), touched_n=c.get("force_n"), local=run.local)
        ceiling_n = spec["ceiling_n"]
        res.update(noise_n=noise, hidden_n=run.local[0] if run.local else 0, f_touch_n=ft, f0_n=spec["first_n"], rigid_mm=rigid,
                   small_mm=small, reach_n=run.B.f_cap(q_c, u), fit=fit, thresholds_from=facts["source"])
        stopped, fields, felt, presses, peak_n, travel = _ramp(ex, run, info, u, c, thing, until,
                                                            firm, max_n, hold_s, rigid, small)
        note = _release(ex, run, info, u, stopped, felt)
        if note:
            fields["extra"] = note
    except Stop as stop:
        stopped, fields = stop.stopped, dict(stop.fields)
        done = fields.pop("ladder", None)
        if done is not None:
            felt, presses, peak_n, travel = done
        met = fields.pop("at", None)
        if contact is None and isinstance(met, dict) and met.get("point_mm") is not None:
            contact = met                        # the contact the stage ended on is in the result
        if stopped == "dark" and run.moved:
            try:
                ex.retreat(steps=1)              # the dark gate allows a retreat: one step back, nothing more
            except Exception:
                pass
    pushed = stopped not in NO_PUSH and bool(presses or felt)
    if pushed and stopped != "dark":
        try:
            after = witness_look(run, q_w, f"after push {info['id']}")
        except Stop as stop:
            run.notes.append(f"the after look stopped {stop.stopped}")
        record = witness_compare(ex, before, after, at_list(ex, info))
    stopped, fields = backed(stopped, fields, record, getattr(run, "step_seen", False), res.get("fit"),
                             res.get("line_source"), felt)
    if run.ramps:
        ceiling_n = run.ramps[-1]["ceiling_n"]
    return act_words.result(ex, run, info, u, stopped, fields, felt, presses, peak_n, ceiling_n, contact, travel, before,
                   after, record, res, label)


def _floor_bound(ex, tip_mm, u):
    """Reach along u, clipped at the stated floor plane when the line goes down."""
    reach = E.reach_mm(ex, tip_mm, u)
    z0 = float(ex.cfg.get("floor_z_m") or 0) * E.MM
    return min(reach, max(0, (float(tip_mm[2]) - z0) / -u[2])) if u[2] < 0 else reach


def _touch_and_fit(ex, run, info, u, max_mm, ft, P, tol, unc, standoff, src="centre", first=None):
    """Confirm contact, fit the face at its uncertainty-derived offset and turn until the fit resolves or stops improving; one new square approach is allowed."""
    fit = {"skipped": None, "turns": []}
    again = False
    r = first if first is not None else run.touch_confirm(u, max_mm, ft, P, unc, info["label"])
    c = _contact(r)
    while True:
        n_e = E._unit(c.get("normal") or [0, 0, 0])
        trusted = src in TRUSTED_LINES or fit["turns"]
        if trusted and n_e is not None and side_words.angle_deg(n_e, -u) <= L.fit_skip_deg(run.B, ex._sense().q, u, force_n=c.get("force_n") or 0, v=(c.get("speed_mm_s") or 0) / E.MM):
            fit["skipped"] = f"the felt normal is within {L.fit_skip_deg(run.B, force_n=ft):g} deg of the line"
            break
        n, pts = run.face_fit(c["point_mm"], u, info["part"], ft, sigma_mm=unc)
        src = "fit"
        if n is None:
            fit["near_an_edge"] = True
            run.notes.append("the touches around found no one face (near an edge): the line is kept")
            break
        off = L.fit_offset_mm(run.B, ex._sense().q, u, sigma_mm=unc)
        done = L.fit_done_deg(run.B, ex._sense().q, u, v_mm_s=c.get("speed_mm_s") or 0, offset_mm=off)
        fit["done_deg"] = done
        ang = side_words.angle_deg(u, -n)
        prev = fit["turns"][-1]["off_deg"] if fit["turns"] else None
        fit["turns"].append({"from": src, "off_deg": round(ang, 1), "points": len(pts)})
        # If the fit no longer improves by its own angular resolution, the
        # fits do not agree. Further turns would falsely claim a square line.
        if prev is not None and prev - ang <= done:
            fit["diverged"] = ang > prev
            run.notes.append(f"the face fit did not settle ({prev:g} then {ang:.1f} deg off): the line is kept")
            break
        if ang <= done:
            u = -n                     # the face agrees with the line: square already
            break
        if ang > math.degrees(math.atan2(off, off)):
            if again:
                raise Stop("not_square", dir=side_words.words(n, c["point_mm"]), deg=round(ang, 0))
            again = True
            run.notes.append(f"the face is {ang:.0f} deg off the line: approached again square to it")
            u = -n
            P = np.asarray(c["point_mm"], dtype=float)
            run.approach_line(P, u, standoff, unc, P=P, source="touch")
            c = _contact(run.touch_confirm(u, standoff + L.touch_reach_mm(run.B, ex._sense().q, u, sigma_mm=unc), ft, P, unc, info["label"]))
            continue
        u = -n
        cp = np.asarray(c["point_mm"], dtype=float)
        run.move("recentre", cp - u * L.confirm_back_mm(run.B, ex._sense().q, u) - run.tip())
        r2 = run.touch("recentre", u, L.confirm_back_mm(run.B, ex._sense().q, u) + L.confirm_max_mm(run.B, ex._sense().q, u, v_mm_s=c.get("speed_mm_s") or 0, force_n=ft), ft)
        if isinstance(r2, dict) and r2.get("contact") and r2.get("point_mm") is not None \
                and r2.get("source") in (None, "tool"):
            c = _contact(r2)
        else:
            c["normal"] = E._r(n, 3)
            break
    _store_normal(ex, info, -u)
    c["normal"] = E._r(-u, 3)
    return c, u, fit


def _contact(r):
    """Retain the touch's contact point, normal, force and arrival speed."""
    return {"id": r.get("id"), "point_mm": r.get("point_mm"), "normal": r.get("normal_est"),
            "moved_part_touch": r.get("moved_part_touch"), "part_gave": r.get("part_gave"),
            "along_push_mm": r.get("along_push_mm"), "force_n": r.get("force_n"),
            "speed_mm_s": r.get("speed_mm_s")}


def _store_normal(ex, info, n):
    """Store the fitted outward normal on the target entry."""
    e = ex.world.get(info["id"])
    if e is not None:
        ex.world.update(info["id"], data={"normal": E._r(n, 4)})


def moved_of(st, info):
    """The largest measured first-motion force on the target or its device."""
    return G.moved_of(st, info)


def _ramp(ex, run, info, u, c, thing, until, firm, max_n, hold_s, rigid, small):
    """Resume force legs from the last reading and greatest shown travel; keep one felt entry and press record per leg."""
    part = info["part"] or {}
    intent = {k: v for k, v in {"max_n": max_n, "firm": firm}.items() if v is not None}
    first = np.asarray(c["point_mm"], dtype=float)
    ref, felt, presses, peak, total = first.copy(), [], [], 0, 0
    touched, shown = c.get("force_n"), 0.0
    watch = None
    try:
        # Refuse unknown force before a seating touch or a between-leg look can move the arm.
        spec = L.force_spec(run.B, "press", thing, ex._sense().q, u, intent=intent, sheet_forces=part.get("forces"),
                            moved_n=moved_of(state(ex), info), touched_n=touched, local=run.local)
        if spec.get("refused"):
            raise Stop("refused", stage="ramp", check="force_unknown", refused=spec["refused"])
        run.items = spec["items"]
        watch = StepWatch(ex, run, info, u) if until == "give" else None
        while True:
            gap = max(0, float((ref - run.tip()) @ u))
            v = c.get("speed_mm_s") or 0
            # Seating by touch prevents a ramp striking across the gap left
            # by release or a look. Prior probes saw a hard strike from that gap.
            if gap > L.seat_gap_mm(run.B, ex._sense().q, u, v_mm_s=v):
                seated = run.touch("seat", u, gap + L.touch_reach_mm(run.B, ex._sense().q, u, thing=thing),
                                   round(L.f_touch(run.B, readings=1, local=run.local), 2))
                if isinstance(seated, dict) and seated.get("contact"):
                    touched = seated.get("force_n")
            try:
                r = run.ramp(u, thing, until, hold_s, intent, part.get("forces"),
                             part.get("entry", {}).get("travel_mm"), moved_of(state(ex), info), touched, shown_mm=shown)
                interrupt = None
            except Stop as stop:
                r = stop.fields.pop("ramp", None)
                if r is None:
                    raise
                interrupt = stop
            if interrupt is None and r["stopped"] not in ("cap", "off_axis", "gave", "moved", "stall", "force", "travel"):
                interrupt = Stop(r["stopped"], stage="ramp")
            stop_tip = np.asarray(r["tip_mm_at_stop"], dtype=float)
            along = float((stop_tip - ref) @ u)
            total = float((stop_tip - first) @ u)
            r.update(along_mm=along, total_mm=total)
            word = act_ladder.word(dict(r, gave=False) if until == "stop" and r["stopped"] in act_ladder.END_STOPS else r, until, rigid, small)
            if word in ("came_back", "small_move") and r["stopped"] == "stall":
                word = "bottomed"
            force = round(r["force_n"], 1)
            peak = max(peak, r["peak_total_n"])
            felt.append({"n": force, "mm": round(along, 1), "word": word})
            presses.append({"seq": ex._calls, "n": force, "start_mm": r["start_mm"], "stopped": r["stopped"],
                            "along_mm": round(along, 2), "gave": bool(r["gave"]), "word": word,
                            "sprang_back_mm": r["sprang_back_mm"]})
            if word in ("stayed_in", "small_move") and until == "give" and interrupt is None:
                word = _stay_check(run, u, ref, stop_tip, round(L.f_touch(run.B, readings=1, local=run.local), 2),
                                   rigid, small, word)
            if word in ("came_back", "small_move", "stayed_in") and watch is not None and along >= small and interrupt is None:
                # A return by design shows its click in this same view. A
                # stayed-in move with no pictured change can instead be a slide.
                seen = watch.changed()
                if seen is True:
                    word, run.step_seen = "gave", True
                elif word == "stayed_in" and seen is False:
                    word = "came_back"
                    run.notes.append("the same view showed no change after the move")
            felt[-1]["word"] = presses[-1]["word"] = word
            if interrupt is not None:
                raise interrupt
            at_ceiling = r["target_n"] >= r["ceiling_n"]
            action, stopped = act_ladder.decide(word, until, r["stopped"], at_ceiling)
            if action == "stop":
                result = _end(stopped, felt, presses, peak, total, r["ceiling_n"], last_n=force, along=along)
                if word == "bottomed":
                    result[1]["why"] = "where its travel ended"
                return result
            # the next leg starts where the part rests now: it travels again what sprang back (shown, at a force
            # the part has already borne) and must then go small further; a part that stayed in has banked its travel
            ref = stop_tip - u * float(r.get("sprang_back_mm") or 0)
            # the next leg ramps again from the touch threshold: starting it at the force the last one reached
            # steps the command ahead and strikes the part (measured: 16.4 N true on a part that triggers at 2.1 N)
            touched = None
            shown = float(r.get("sprang_back_mm") or 0)
    # A gate or time stop keeps the legs already made. The result cannot
    # claim no push after a push, and the carry guard needs the same evidence.
    except Stop as stop:
        stop.fields["ladder"] = (felt, presses, peak, total)
        raise


def _stay_check(run, u, ref, stop_tip, ft, rigid, small, w):
    """
    After the release: touch along u for the surface. Back within rigid_mm of where the press began: it
    came
    back (came_back); within small_mm: small_move; deeper, or gone (nothing met): the reading
    stands.
    """
    reach = max(0.0, float((stop_tip - run.tip()) @ u)) + L.stay_reach_mm(run.B, run.ex._sense().q, u, v_mm_s=0)
    r = run.touch("stay_check", u, reach, ft)
    if not (isinstance(r, dict) and r.get("contact") and r.get("point_mm") is not None):
        return w
    depth = float((np.asarray(r["point_mm"], dtype=float) - ref) @ u)
    if depth < rigid:
        return "came_back"
    if depth < small:
        return "small_move"
    return w


def _end(stopped, felt, presses, peak, total, ceiling_n, last_n=None, along=0.0):
    """Fill the final force and travel fields from the completed ramp legs."""
    fields = {"n": last_n, "mm": round(max([0.0] + [x["mm"] for x in felt]) if stopped == "ceiling" else along, 1),
              "total": round(total, 0), "C": ceiling_n}
    if stopped == "time":
        fields.update(stage="ladder", s=None, what=f"the part was still moving after {len(felt)} pushes")
    return stopped, fields, felt, presses, peak, total


def _release(ex, run, info, u, stopped, felt):
    """Settle after a change, then retract with the contact guard along the open side or back along the line, and up to the derived standoff."""
    part = info["part"]
    thing = L.thing_of(run.B, info["id"], sigma_mm=info["uncertain_mm"])
    travel = sum(max(0, r["along_mm"]) for r in run.ramps)
    # The earlier after-look move swept the hand into the device when it
    # began too near. Backoff uses shown travel and tool width before the look.
    up = L.retract_up_mm(run.B, ex._sense().q, thing=thing)
    if stopped in ("changed", "reached_end"):
        run.settle()
        front = side_words.front(info["centre"], info["P"]) if info["P"] is not None else None
        if (part or {}).get("self_moving") and front is not None:
            return run.retract(front, L.retract_mm(run.B, ex._sense().q, front, travel_mm=travel, size_mm=E.largest_size(part)), up_mm=up)
        return run.retract(-u, L.retract_mm(run.B, ex._sense().q, u, travel_mm=travel), up_mm=up)
    if felt:
        return run.retract(-u, L.retract_mm(run.B, ex._sense().q, u, travel_mm=travel), up_mm=up)
    return None


class Mixin:
    """The whole-step push and lift tool mixin."""
    @primitive
    def push(self, target=None, way="in", until=None, firm=False, max_n=None, hold_s=0, detail="brief"):
        """Run the whole push tool."""
        return run_push(self, target, way, until, firm, max_n, hold_s, detail)

    @primitive
    def lift(self, target=None, max_n=None, firm=False, detail="brief"):
        """Run the declared whole lift tool."""
        from rcpx.r2 import act_lift
        return act_lift.run_lift(self, target, max_n, firm, detail)


def setup(ex):
    """Install act state and carry press hooks."""
    state(ex)
    w = (getattr(ex, "state", None) or {}).get("waiting")
    if isinstance(w, dict) and isinstance(w.get("toward_world"), set):
        w["toward_world"].update(DIRECT)        # the dark gate meets an operator's push and lift
    G.install(ex)


def FILTER(surface, resources, handlers, ex=None):
    """lift is listed only when r3_lift_declared is true (SPEC-R3 0.1 R10)."""
    if ex is not None and not lift_declared(ex):
        surface = dict(surface)
        surface["tools"] = [t for t in surface["tools"] if t.get("name") != "lift"]
    return surface, resources, handlers

_TARGET = S._str("a located or contact id from robot://world (located_3, contact_2), or a part name from "
                 "robot://devices (the newest located point of that part); never coordinates")
TOOLS = [
    S._tool("push",
            "Pushes a located part the way it moves: the finger comes straight onto its face, feels it, pushes a "
            "smooth ramp from the touch force until the part moves, lets go and shows the same view "
            "before and after. Its most force comes from the sheet, an earlier push that moved the part, or max_n. "
            "Example: push {\"target\": \"located_3\"}.",
            S._obj({"target": _TARGET,
                    "way": S._str("in (into the face along the line the robot chooses, default) or down (straight "
                                  "down onto a top face)", ["in", "down"]),
                    "until": S._str("give (stop when it gives) or stop (push until it stops: the default for a "
                                    "part that slides)", ["give", "stop"]),
                    "firm": S._bool("allow the robot side alone when no part force is known (default false)"),
                    "max_n": {"type": "number", "description": "the most force you allow (N): it raises or lowers what the robot would use, never past the arm's own rating"},
                    "hold_s": {"type": "number", "description": "hold the last force this long (s, default 0); a sheet's long press time bounds it"},
                    "detail": S._str("brief (default) or full (every stage)", ["brief", "full"])}, ["target"]),
            {"target": "located_3"}, "push the located part until it gives, then show before and after"),
    S._tool("lift",
            "Lifts a part from under its edge: the fingertip goes under it from outside, ramps the force smoothly "
            "from the touch until it moves, then comes out. Its most force comes from the sheet, a push that moved it, "
            "or max_n. Example: lift "
            "{\"target\": \"located_2\"}.",
            S._obj({"target": _TARGET,
                    "max_n": {"type": "number", "description": "the most force you allow (N): it raises or lowers what the robot would use, never past the arm's own rating"},
                    "firm": S._bool("allow the robot side alone when no part force is known (default false)"),
                    "detail": S._str("brief (default) or full (every stage)", ["brief", "full"])}, ["target"]),
            {"target": "located_2"}, "lift the located part from under its edge until it gives"),
]

PRACTICES = ("push and lift raise force in one smooth ramp from the touch until the part moves. "
             "The most force comes from the sheet, an earlier push that moved it, or max_n.")

__all__ = ["FILTER", "ID", "Mixin", "PRACTICES", "TOOLS", "backed", "lift_declared", "line_trust", "preflight",
           "run_push", "setup", "state"]
