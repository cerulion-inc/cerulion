# SPDX-License-Identifier: AGPL-3.0-only
"""handover(giver, taker, at_mm=None, frame="R") of the bimanual package (rcpx/r2/bimanual.py): a held object
passes from one arm's jaws to another's.

What counts as held is the place package's held state on the giver (ex.state["place"], set by its pick); without
it the call is refused with check "not_held" and nothing moves. Geometry, all in frame R (z up):
- u: the horizontal direction from the giver's base to the taker's base;
- the meeting point P: at_mm (frame R, or an arm's frame B when frame names that arm), else the midpoint of the two
  bases raised by the first height in derived heights at which both arms reach their poses and, there, keep the clearance
  margin between them everywhere but the two hands (said in the result);
- the giver's tip goes to P pointing along u with its jaws closing up and down; the taker's tip comes to the derived meeting gap
  past P pointing back along -u with its jaws closing across, so the four jaws do not meet: the taker closes on what
  stands out of the giver's jaws toward it.
Steps (each one an arm call with every check of an operator's own call; the two hands are exempt from the arm
clearance while they meet): the taker opens; the giver carries the object to P; the taker comes to the derived standoff
before its pose, then on along -u under its guard; the giver's drop watch pauses and the taker closes (the round one
guarded close); the taker's hold is the place package's held test (jaw gap over its smallest gap, jaws pressing).
Held: the taker's held state is set (its drop watch takes over), the giver opens and backs off the derived retreat along -u,
and the taker's hold is read again. Not held: nothing is reopened; the giver still holds it and the answer says so.
The answer: {"said": one sentence, "held_by": taker | giver | None, "meet_mm", "evidence": jaw gaps, the taker's
squeeze estimate, "steps"}. Nothing here names a robot or an object.
"""

import math

import numpy as np

from rcpx.context import refusal as _refusal
from rcpx.r2 import bimanual_bounds as D

def refusal(check, why, hint):
    return {"said": "Refused: " + why + ".", **_refusal(check, why, hint)}

MM = 1000.0

def _unit(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return None if n < 1e-9 else v / n


def _rot(a1, a2, b1, b2):
    """The rotation taking tool vectors a1, a2 (a2 orthogonalised to a1) onto frame vectors b1, b2."""
    a1, b1 = _unit(a1), _unit(b1)
    a2 = _unit(np.asarray(a2, float) - (np.asarray(a2, float) @ a1) * a1)
    b2 = _unit(np.asarray(b2, float) - (np.asarray(b2, float) @ b1) * b1)
    A = np.column_stack([a1, a2, np.cross(a1, a2)])
    B = np.column_stack([b1, b2, np.cross(b1, b2)])
    return B @ A.T


def _jaw_reading(ex):
    J = ex._pl_jaws
    q, target = J.reading(ex.backend.read().get("gripper"), ex._gripper_cmd)
    if q is None:
        return None
    return {"gap_m": J.width_m(q), "gap_mm": round(J.width_m(q) * MM, 1), "pressing": bool(J.pressing(q, target)),
            "squeeze_n_est": None if J.squeeze_n(q, target) is None else round(J.squeeze_n(q, target), 2),
            "q": q, "target": target}


def _held(r, min_gap_m):
    return r is not None and r.get('gap_m',r['gap_mm']/MM) >= min_gap_m and r["pressing"]


def _phase_ok(r, accepted):
    return 'refused' not in r and D.stopped(r) in accepted


def _short(r):
    if not isinstance(r, dict):
        return r
    keep = ("stopped", "said", "error_deg", "error_mm", "why", "held", "arm_clearance", "blocked")
    out = {k: r[k] for k in keep if k in r}
    if D.stopped(r) is not None:
        out['stopped'] = D.stopped(r)
    if "refused" in r:
        out["refused"] = r["refused"]
    return out


class _Plan:
    def __init__(self, bm, giver, taker):
        self.bm, self.g, self.t = bm, giver, taker
        f = bm.frames
        pg, pt = f.T_R_B(giver)[:3, 3], f.T_R_B(taker)[:3, 3]
        d = pt - pg
        d[2] = 0.0
        self.u = _unit(d)
        self.mid = np.mean([pg, pt], axis=0)
        up = np.array([0.0, 0.0, 1.0])
        if self.u is not None:
            Jg, Jt = bm.exs[giver]._pl_jaws, bm.exs[taker]._pl_jaws
            x = np.array([1.0, 0.0, 0.0])
            self.Rg = _rot(x, Jg.axis_T, self.u, up)
            self.Rt = _rot(x, Jt.axis_T, -self.u, np.cross(up, self.u))

    def ik(self, arm, p_R, R_R):
        f, ex = self.bm.frames, self.bm.exs[arm]
        p = f.point_from_R(arm, np.asarray(p_R) * MM) / MM
        R = f.T_B_R(arm)[:3, :3] @ R_R
        q, ok, err = ex.model.ik(np.asarray(ex.backend.read()["q"], float), p, R)
        return q, bool(ok)

    def margin(self, qg, qt):
        exg, ext = self.bm.exs[self.g], self.bm.exs[self.t]
        Bg, Bt = D.of(exg), D.of(ext)
        ea = max((D.fit_error(exg.model,c[0])[0] for c in exg.model.collision_capsules(qg)), default=0)
        eb = max((D.fit_error(ext.model,c[0])[0] for c in ext.model.collision_capsules(qt)), default=0)
        ug = self.bm.frames.dir_from_R(self.g,self.u)
        ut = self.bm.frames.dir_from_R(self.t,-self.u)
        # The giver is held at its meeting pose while the taker approaches.
        # The individual giver leg has its own path check before this phase.
        return D.plan_margin(Bg,Bt,0,Bt.v_free(qt,ut),ea,eb,
                             D.base_error(self.bm.frames,self.g,self.t)[0])

    def poses(self, P):
        """(giver q, taker pre q, taker q) or None when an arm cannot reach its pose at P (m, frame R)."""
        exg, ext = self.bm.exs[self.g], self.bm.exs[self.t]
        Bg, Bt = D.of(exg), D.of(ext)
        qg0, qt0 = exg.backend.read()["q"], ext.backend.read()["q"]
        err = D.base_error(self.bm.frames, self.g, self.t)[0]
        pad = max(D.pad_thickness(exg._pl_jaws), D.pad_thickness(ext._pl_jaws))
        ug = self.bm.frames.dir_from_R(self.g,self.u)
        ut = self.bm.frames.dir_from_R(self.t,-self.u)
        gap = D.meet_gap(Bg, Bt, err, pad, qg0, qt0, ug, ut)
        so = D.standoff(Bt, qt0, ut, Bt.thing("arm:" + self.g), err)
        self.gap, self.standoff = gap, so
        qg, okg = self.ik(self.g, P, self.Rg)
        if not okg:
            return None
        qt, okt = self.ik(self.t, P + self.u * gap, self.Rt)
        if not okt:
            return None
        qp, okp = self.ik(self.t, P + self.u * (gap + so), self.Rt)
        if not okp:
            return None
        c, ex = self.bm.clear, self.bm.exs
        hands = {self.g: set(ex[self.g]._tool_bodies), self.t: set(ex[self.t]._tool_bodies)}
        margin = self.margin(qg, qt)
        if c.between(self.t, qp, self.g, qg) < margin or c.between(self.t, qt, self.g, qg, hands) < margin:
            return None       # the arms would come too near each other there (only the hands may meet)
        return qg, qp, qt


def _handover(bm, session, giver=None, taker=None, at_mm=None, frame="R", clear=None):
    for name, a in (("giver", giver), ("taker", taker)):
        if a not in bm.arms:
            return refusal("arm", f"{name} must be one of this robot's arms ({', '.join(bm.arms)}), got {a!r}",
                           f'look(arm="{bm.arms[0]}")')
    if giver == taker:
        return refusal("arm", "giver and taker are the same arm", f'look(arm="{giver}")')
    exg, ext = bm.exs[giver], bm.exs[taker]
    stg, stt = exg.state.get("place"), ext.state.get("place")
    if stg is None or getattr(exg, "_pl_jaws", None) is None:
        return refusal("not_held", f"arm {giver} keeps no held state (no gripper, or picking is not loaded)",
                       f'look(arm="{giver}")')
    if not stg.get("held"):
        return refusal("not_held", f"arm {giver} holds nothing: its last pick did not say held, or it let go",
                       f'look(arm="{giver}")')
    if stt is None or getattr(ext, "_pl_jaws", None) is None:
        return refusal("no_gripper", f"arm {taker} has no gripper to take the object with", f'look(arm="{taker}")')
    if stt.get("held"):
        return refusal("holding", f"arm {taker} already holds an object", f'look(arm="{taker}")')
    intent = {"clear": clear}
    for ex in (exg, ext):
        B = D.of(ex)
        if clear is not None:
            word = B.word(intent)
            if word.get("refused"):
                return word["refused"]
    plan = _Plan(bm, giver, taker)
    if plan.u is None:
        return refusal("arm", "the two arms stand on the same spot: no direction between them", f'look(arm="{giver}")')
    if at_mm is not None:
        if frame not in ("R",) + tuple(bm.arms):
            return refusal("frame", f"frame is R or an arm name, got {frame!r}", f'handover(giver="{giver}",taker="{taker}",frame="R")')
        p = np.asarray(at_mm, dtype=float).reshape(-1)
        if p.size != 3 or not np.all(np.isfinite(p)):
            return refusal("at_mm", "at_mm is [x, y, z] in mm", f'handover(giver="{giver}",taker="{taker}")')
        P = (p if frame == "R" else bm.frames.point_to_R(frame, p)) / MM
        poses = plan.poses(P)
        how = f"at_mm as given (frame {frame})"
        if poses is None:
            return refusal("unreachable", f"the arms cannot both reach their handover poses at {_mm(P)} mm (frame R)",
                           f'handover(giver="{giver}",taker="{taker}")')
    else:
        poses, P, how = None, None, ""
        qg0,qt0 = exg.backend.read()["q"],ext.backend.read()["q"]
        margin = plan.margin(qg0,qt0)
        heights = D.raise_heights(D.of(exg), [exg.model,ext.model], plan.mid, D.of(exg).spacing(margin),
                                 lambda p: plan.ik(giver,p,plan.Rg)[1] and plan.ik(taker,p,plan.Rt)[1])
        for cand in heights:
            poses = plan.poses(cand)
            if poses is not None:
                P, how = cand, f"the midpoint of the two bases, raised {(cand[2]-plan.mid[2])*MM:g} mm at derived spacing"
                break
        if poses is None:
            return refusal("unreachable", "no height over the midpoint of the bases lets both arms reach their "
                           "handover poses in the model's reachable height search",
                           f'look(arm="{giver}")')
    qg, qp, qt = poses
    phases = []
    steps, ev = {}, {"giver_before": _view(_jaw_reading(exg))}
    meet = _mm(P)

    def call(key, arm, action, value=None, *, kind='transit', target=None, away_from=None):
        ex, B = bm.exs[arm], D.of(bm.exs[arm])
        B.begin(key)
        gate = ex.gates('gripper' if action == 'jaw' else 'move_joints', {})
        if gate is not None:
            r = gate
        elif action == 'jaw':
            r = D.jaw(ex,value,intent,ex.state.get('place'))
        elif action == 'stream':
            r = D.stream(ex,value,held=ex.state.get('place'))
        elif action == 'joints':
            r = D.joints(ex,value,kind=kind,target=target,intent=intent,held=ex.state.get('place'))
        else:
            r = D.line(ex,value,kind=kind,target=target,intent=intent,away_from=away_from,
                       held=ex.state.get('place'))
        r.setdefault('said', f'Arm {arm} ended {key} with {D.stopped(r) or "refused"}.')
        st = ex.state.get('bimanual_clear',{})
        if D.stopped(r) == 'arm_clearance' and st.get('stop'):
            r['arm_clearance'] = st['stop']
            B.report(r,[st['stop']['bounds']])
        steps[key] = _short(r)
        counts = r.get('periods',0)
        n = sum(counts) if isinstance(counts,list) else counts
        waiting = r.get('waiting',0)
        jaw_n = n-waiting if action == 'jaw' else r.get('jaw',0)
        moving = 0 if action == 'jaw' else r.get('moving',n-jaw_n-waiting)
        phases.append(dict(phase=key,did=r['said'],periods=n,moving=moving,waiting=waiting,jaw=jaw_n,
                           seconds=n*B.dt(),free_share=r.get('free_share',(r.get('bounds') or {}).get('free_share')),bounds=(r.get('bounds') or {}).get('binding')))
        return r

    def stop(said, held_by):
        bm.clear.exempt = {}
        stg.pop("paused", None)
        return {"said": said, "held_by": held_by, "meet_mm": meet, "meet_frame": "R", "meet_how": how,
                "evidence": ev, "steps": steps, "periods": sum(p["periods"] for p in phases), "phases": phases,
                "bounds": {"binding": next((p["bounds"] for p in phases if p["bounds"]), None), "more": 0}}

    r = call("taker_open", taker, "jaw", 1)
    if not _phase_ok(r,('goal','done')):
        return stop(f"No handover: arm {taker} could not open ({_why(r)}).",giver)
    r = call("giver_to_meet", giver, "joints", qg, kind="carry")
    if "refused" in r or D.stopped(r) != "goal" or not stg.get("held"):
        return stop(f"No handover: arm {giver} did not reach the meeting point ({_why(r)}).",
                    giver if stg.get("held") else None)
    bm.clear.exempt = {giver:set(exg._tool_bodies),taker:set(ext._tool_bodies)}
    tip,R = ext.model.fk_tip(qt)
    pose = np.eye(4)
    pose[:3,:3],pose[:3,3] = R,tip
    legs = [{'kind':'transit','to':qp,'intent':intent},
            {'kind':'approach','to':pose,'line':True,'target':'arm:' + giver,'intent':intent}]
    r = call('taker_to_meet',taker,'stream',legs)
    if 'refused' in r or D.stopped(r) not in ('goal','contact'):
        return stop(f"No handover: arm {taker}'s approach stopped ({_why(r)}).",giver)
    stg["paused"] = True
    r = call("taker_close", taker, "jaw", 0)
    if not _phase_ok(r,('goal','done','blocked','contact')):
        return stop(f"No handover: arm {taker}'s close stopped ({_why(r)}).",giver)
    rt = _jaw_reading(ext)
    ev["taker_closed"] = _view(rt)
    if not _held(rt,D.min_gap_m(D.of(ext),ext._pl_jaws)):
        stg.pop("paused", None)
        gap = "no reading" if rt is None else f"{rt['gap_mm']:g} mm"
        return stop(f"No handover: arm {taker}'s jaws closed to {gap} and do not press on the object, so it "
                    f"holds nothing; arm {giver} still holds it and nothing was reopened.", giver)
    J = ext._pl_jaws
    w = rt["gap_m"]
    stt.update(held=True, dropped=False, width_m=w, lead_m=w - (J.width_m(rt["target"]) if rt["target"] is not None
                                                                 else w),
               squeeze_n=rt["squeeze_n_est"], hang_m=float(stg.get("width_m") or 0.0) / 2,
               height_m=stg.get("width_m"), support_z=None, colour=stg.get("colour"), picked_at=None,
               offset_T=J.gap_centre_T(J.frac_of_q(rt["q"])).tolist(), open_w=stg.get("open_w"),
               weight_n=stg.get("weight_n"), since_call=ext._calls, report=None, label=stg.get("label"),
               handed_over_from=giver)
    r = call("giver_open", giver, "jaw", 1)
    if not _phase_ok(r,('goal','done')):
        rg = _jaw_reading(exg)
        ev['giver_release_stopped'] = _view(rg)
        stg['held'] = _held(rg,D.min_gap_m(D.of(exg),exg._pl_jaws))
        return stop(f"The taker closed, but arm {giver}'s release stopped ({_why(r)}); inspect the two holds.",
                    giver if stg['held'] else taker)
    stg.update(held=False, dropped=False, report=None)
    stg.pop("paused", None)
    ev["giver_open"] = _view(_jaw_reading(exg))
    Bg = D.of(exg)
    q = exg.backend.read()['q']
    err = D.base_error(bm.frames,giver,taker)[0]
    away = bm.frames.dir_from_R(giver,-plan.u)
    retreat = D.standoff(Bg,q,away,None,err)
    d = away * retreat
    r = call('giver_back_off',giver,'line',d,kind='retreat',away_from='arm:' + taker)
    rt2 = _jaw_reading(ext)
    ev["taker_after_release"] = _view(rt2)
    if _held(rt2,D.min_gap_m(D.of(ext),ext._pl_jaws)) and stt.get("held"):
        if not _phase_ok(r,('goal','done')):
            return stop(f"Arm {taker} holds it after release, but arm {giver}'s retreat stopped ({_why(r)}).",taker)
        return stop(f"Handed over: arm {taker}'s jaws hold it {rt2['gap_mm']:g} mm apart and press on it after arm "
                    f"{giver} let go and backed off.", taker)
    stt.update(held=False, dropped=True)
    return stop(f"Dropped in the handover: after arm {giver} let go, arm {taker}'s jaws no longer press on anything "
                f"(gap {'unknown' if rt2 is None else rt2['gap_mm']} mm); look before the next move.", None)


def _view(r):
    return None if r is None else {k: r[k] for k in ("gap_mm", "pressing", "squeeze_n_est")}


def _why(r):
    if isinstance(r, dict) and "refused" in r:
        return f"refused, check {r['refused'].get('check')}: {r['refused'].get('why')}"
    return f"stopped {D.stopped(r) if isinstance(r, dict) else r}"


def _mm(p):
    return [round(float(x) * MM, 1) for x in np.asarray(p, dtype=float).reshape(3)]


def handover(bm,session,giver=None,taker=None,at_mm=None,frame='R',clear=None):
    try:
        return _handover(bm,session,giver,taker,at_mm,frame,clear)
    finally:
        bm.clear.exempt = {}
        if giver in bm.exs:
            (bm.exs[giver].state.get('place') or {}).pop('paused',None)
        if clear is not None:
            for a in (giver,taker):
                if a in bm.exs:
                    D.end_word(D.of(bm.exs[a]))
