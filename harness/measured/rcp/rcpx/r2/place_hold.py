# SPDX-License-Identifier: AGPL-3.0-only
"""The hold of package place: the names the place and fixture files call, over the core's one hold.

Since round six the hold itself is the core's (rcpx/control_hold.py: one record, one close, one release, the load
read at rest, the jaws' own numbers from steady rates in the self-test). This file keeps the names older callers
use and turns the core's answers into the keys they read; it measures nothing inside a task, searches no squeeze
and holds no level of its own:

- profile(ex): the free jaws' numbers in the old keys (speed and gaps in m), from control_hold.numbers(ex): the
  robot's stored self-knowledge, else the body's own description (said in "source"). Never a timed stroke.
- close(ex, expect_m, band_m): control_hold.close. No limit stated: the jaws' own rate. With one jaw and no width
  seen the hand moves with the jaw so that the gap's middle stays where it is.
- stroke(ex, cmd): bring the jaws to an opening (an opening is control_hold.release).
- note_lift(ex): the test lift ran: the load is read at rest (control_hold.read_load).
- judge(ex, reading, cmd): is something between the jaws, from the core's record: held = the record stands (the
  jaws did not close past where they stalled); the weight read at rest is evidence beside it.
- stands(ex, seen, s_close, s_now, mid_T): where the held thing stands in the hand after the test lift, as the arm's own
  readings and ONE look show it. A thing taken off its middle (a rim, a handle, a corner)
  turns and slides between the pads while it is lifted; the pose before the close then says wrongly where its
  middle is. The reading: the look the take was pointed in showed the thing's surface where it stood; the hand's
  own motion since the close says where that surface would be now had nothing moved between the pads; one depth
  picture of the same fixed camera, taken now (no motion, no control period; it is no look of the session and
  nothing is noted of it in the space book), shows where it is. The seen surface is laid onto
  the new one (point to plane, the robot's own body left out both ways) and the record takes the result: the
  thing's own motion in the tool frame since the close, where that puts the middle of its bottom, how far it
  turned and moved between the pads, its long side's direction. Only the difference of the two looks is used
  (what the camera is off by against the arm is in both). A turn of a round thing about its own axis shows
  nothing and is not in it. The reading replaces the pose before the close only where it differs from it by more
  than its own band (BAND of the larger of a pixel's footprint and the fit's rms) and by no more than half the
  thing's own narrow width: past that the thing hangs from the pads and swings, which is said. Never a gate: where
  nothing can be read (no look with depth, too little seen, a fit that does not hold) the answer says why and
  the place acts on the pose before the close, as it did.

Other packages use judge(ex, reading, cmd) and place_hand.Jaws; nothing else. SI inside (m, s, N).
"""

import math

import numpy as np

from rcpx import control_hold

MM = 1000.0     # unit
HALF = math.ldexp(1, -1)     # the middle of a width


def _ratios():
    from rcpx import bounds_ratios as R
    return R


def _state(ex):
    return ex.state.setdefault("place_hold", {})


def read(ex, s=None):
    """(q, target, gap m, squeeze N or None, largest lag in joint units or None) of the jaws now."""
    J = ex._pl_jaws
    g = (s.gripper if s is not None else ex.backend.read().get("gripper"))
    q, target = J.reading(g, ex._gripper_cmd)
    if q is None:
        return None, None, None, None, None
    lag = J.lag(q, target)
    return q, target, J.width_m(q), J.squeeze_n(q, target), None if lag is None else float(np.max(lag))


def _period(ex):
    q_des = getattr(ex, "_q_des", None)
    return ex._period(ex._sense().q if q_des is None else q_des)


# ---------------------------------------------------------------------------------------------- the free jaws
def profile(ex, may_measure=True):
    """The free jaws' numbers in the keys older callers read (gaps and speeds in m and m/s at the jaw tips), from
    the core's numbers (control_hold.numbers: the self-test's steady rates, else the body's description). Nothing
    is measured here: may_measure is kept as an argument and ignored. Also sets Jaws.know, so that Jaws.pressing
    judges a squeeze against the spread of the free jaw's own lag."""
    R = _ratios()
    J = ex._pl_jaws
    N = control_hold.numbers(ex)
    F = control_hold.facts(ex)
    per = float(N["gap_per_span_m"])
    a, b = (float(v) for v in N["lag"])
    spread_n = 0.0 if F["kp"] is None else float(np.max(F["kp"] * N["lag_spread"] * F["span"] / F["lever_tip"]))
    P = {"speed": float(N["speed"]) * per, "lag": a + b * float(N["speed"]), "lag_spread": float(N["lag_spread"]),
         "closed_gap": float(J.width_m(J.q_of_frac(N["closed_gap"]))), "squeeze_hidden": 0.0,
         "squeeze_noise": spread_n, "gap_still": float(N["still_band"]) * per, "step": 0.0,
         "t_close": 1.0 / max(float(N["speed"]), np.finfo(float).tiny),
         "arm_hidden": float(N.get("arm_free_n") or 0.0), "floor_n": float(N["floor_n"]),
         "source": ("the robot's own self-knowledge (steady closing rates)" if N["source"] == "self_test" else
                    "the body's own description (these jaws were not measured at steady rates)")}
    J.know = {"press_n": R.TWO_READ * spread_n,
              "lag": float(np.max((float(N.get("lag_rest") or a) + R.TWO_READ * N["lag_spread"]) * F["span"]))}
    return P


# ---------------------------------------------------------------------------------------------- the judgement
def judge(ex, reading, cmd):
    """held_evidence of a gripper reading (a Backend.read()["gripper"] block) under the command cmd (a fraction):
    {jaws_stopped_short_mm, still_s, pressing, lag_over_free, weight_felt_n, gap_change_on_lift_mm, on, held,
    width_mm, squeeze_n, kind, rest_read_periods}. on and held: the core's record stands (the close ended on a
    thing and the jaws have not closed past where they stalled, nor been opened). The weight read at rest after
    the test lift is evidence beside it: a thing lighter than the arm can tell is held by the jaws' gap alone."""
    J = ex._pl_jaws
    P = profile(ex)
    q, target = J.reading(reading, cmd)
    rec = control_hold.held(ex)
    out = {"jaws_stopped_short_mm": None, "still_s": None, "pressing": False, "lag_over_free": False,
           "weight_felt_n": None if rec is None else dict.get(rec, "weight_n"), "gap_change_on_lift_mm": None,
           "on": rec is not None, "held": rec is not None, "width_mm": None, "squeeze_n": None,
           "kind": None if rec is None else dict.get(rec, "kind"),
           "rest_read_periods": None if rec is None else dict.get(rec, "read_periods")}
    if rec is None:
        lost = control_hold.ended(ex)
        if lost is not None:
            out["ended"] = lost.get("why")
    if q is None:
        return out
    gap = J.width_m(q)
    sq = J.squeeze_n(q, target)
    lag = J.lag(q, target)
    st = _state(ex)
    out.update(jaws_stopped_short_mm=round((gap - P["closed_gap"]) * MM, 2), pressing=bool(J.pressing(q, target)),
               lag_over_free=bool(lag is not None and float(np.max(lag)) > J.know["lag"]),
               width_mm=round(gap * MM, 2), squeeze_n=None if sq is None else round(sq, 3))
    if st.get("stall_gap") is not None and st.get("lifted"):
        out["gap_change_on_lift_mm"] = round((gap - st["stall_gap"]) * MM, 2)
    return out


def note_lift(ex, weight_n=None):
    """The test lift after a close ran: the load is read at rest from the raw estimate (control_hold.read_load;
    its control periods are the read's own and are said in judge's rest_read_periods). weight_n is what an older
    caller read itself: ignored, the record's weight is the one read here."""
    st = _state(ex)
    st["lifted"] = True
    rec = control_hold.held(ex)
    if rec is not None and dict.get(rec, "weight_n") is None:
        control_hold.read_load(ex)


# ---------------------------------------------------------------------------------------------- where it stands
def _normals(P, mask, eye, gap):
    """(unit normals toward the camera at eye, where they are known) of a depth picture's points P (H x W x 3):
    at the pixels of mask whose four neighbours are in mask too and no further apart than gap (one surface)."""
    a, b = np.zeros_like(P), np.zeros_like(P)
    a[:, 1:-1] = P[:, 2:] - P[:, :-2]
    b[1:-1, :] = P[2:, :] - P[:-2, :]
    ok = np.zeros(mask.shape, dtype=bool)
    ok[1:-1, 1:-1] = mask[1:-1, 1:-1] & mask[1:-1, 2:] & mask[1:-1, :-2] & mask[2:, 1:-1] & mask[:-2, 1:-1]
    ok &= (np.linalg.norm(a, axis=-1) <= gap) & (np.linalg.norm(b, axis=-1) <= gap)
    n = np.cross(a, b)
    size = np.linalg.norm(n, axis=-1)
    ok &= size > 0.0
    n = n / np.where(size > 0.0, size, 1.0)[..., None]
    n[np.sum(n * (eye - P), axis=-1) < 0.0] *= -1.0
    return n, ok


def _lay(X, N, Y, NY, gate, gate_end, tol, hidden):
    """Lay the surface X (points, with normals N) onto the surface Y (normals NY): the rotation Rf about X's own
    middle c and the shift tf that bring each point of X onto the plane of the point of Y nearest to it, in steps
    (the pairs no further apart than a gate that halves down to gate_end, their normals within a sixth of a turn,
    the points the robot's own body hides now left out; each step damped by REL, so that a direction the surfaces
    do not fix, a round thing about its own axis, stays where the hand's motion put it). Returns (Rf, tf, c, the
    pairs at the end, how many points of X the look can show, the rms distance of the pairs from their planes),
    or None when fewer pairs are left than a pose has numbers."""
    from scipy.spatial import cKDTree
    from scipy.spatial.transform import Rotation
    R = _ratios()
    tree, n = cKDTree(Y), X.shape[-1]
    c = X.mean(axis=0)
    rho = float(np.sqrt(np.mean(np.sum((X - c) ** 2, axis=1))))      # the thing's own radius: a turn in its units
    Rf, tf = np.eye(n), np.zeros(n)

    def pairs(gate):
        Xk, Nk = (X - c) @ Rf.T + c + tf, N @ Rf.T
        d, j = tree.query(Xk)
        shown = ~hidden(Xk)
        keep = shown & (d <= gate) & (np.sum(Nk * NY[j], axis=1) > R.GIVE)
        return Xk[keep], Y[j[keep]], NY[j[keep]], int(shown.sum())
    for _ in range(int(R.QUANT / (R.REL * R.REL))):
        Xk, Yk, Nk, _ = pairs(gate)
        if len(Xk) < n + n:
            return None
        J = np.hstack([np.cross(Xk - c, Nk) / rho, Nk])
        A = J.T @ J
        A += R.REL * float(np.mean(np.diag(A))) * np.eye(len(A))
        step = np.linalg.solve(A, -J.T @ np.sum(Nk * (Xk - Yk), axis=1))
        dR = Rotation.from_rotvec(step[:n] / rho).as_matrix()
        Rf, tf = dR @ Rf, dR @ tf + step[n:]
        if gate <= gate_end and float(np.linalg.norm(step)) < tol:
            break
        gate = max(gate_end, R.GIVE * gate)
    Xk, Yk, Nk, shown = pairs(gate_end)
    if len(Xk) < n + n:
        return None
    return Rf, tf, c, len(Xk), shown, float(np.sqrt(np.mean(np.sum(Nk * (Xk - Yk), axis=1) ** 2)))


def judged(moved_m, turn_rad, wide_m, narrow_m, sigma_m):
    """What a reading of the held thing says against the pose before the close: {"band_m": BAND of the reading's
    own error, "moved": its middle, or the end of its long side (half its wide side out, turned), stands further
    than that band from where the hand's own motion puts it, "swings": its middle moved by more than half its
    own narrow width (it hangs from the pads)}."""
    band = _ratios().BAND * float(sigma_m)
    return {"band_m": band, "moved": bool(max(float(moved_m), float(turn_rad) * HALF * float(wide_m)) > band),
            "swings": bool(float(moved_m) > HALF * float(narrow_m))}


def stands(ex, seen, s_close, s_now, mid_T):
    """Where the held thing stands in the hand now (module text). seen: the Seen the take was pointed at (with
    its spot); s_close, s_now: the hand as the jaws closed and as it stands after the test lift; mid_T: the
    middle of the thing's bottom in the tool frame as the jaws closed, had it stood where the first look showed
    it. Returns {"read": True, "camera", "R_T", "t_T": the thing's own motion in the tool frame since then (a
    point x of it now stands at R_T x + t_T), "middle_T": where that makes its bottom's middle stand,
    "long_T": the direction of its long side, "turn_rad" and "moved_m": how far it turned, and how far its
    middle moved across the tool, against a thing that kept its place between the pads, "below_tip_m",
    "sigma_m": the larger of one pixel's footprint and the fit's own rms, "band_m": BAND of them, "moved": its
    middle, or the end of its long side, stands further than that band from where the hand's own motion puts it
    (within the band the reading and the pose before the close say the same, and the pose stands), "swings":
    its middle moved by more than half its own narrow width (it no longer stands between the pads as it was
    taken: it hangs from them, and one look does not say where a thing that swings will be at the place),
    "pairs", "shown"}, or {"read": False, "why"}: nothing is refused for it. Only the difference of two looks
    of one camera goes into the answer: what that camera is off by against the arm is in both."""
    R, J = _ratios(), ex._pl_jaws

    def no(why):
        return {"read": False, "why": why}
    sp = seen.get("spot") if isinstance(seen, dict) and isinstance(seen.get("spot"), dict) else {}
    e = sp.get("extent") if isinstance(sp.get("extent"), dict) else {}
    if e.get("kind") != "thing" or e.get("centre_xy") is None or e.get("bearing_rad") is None:
        return no("no thing was made out at the spot in a look with depth")
    try:
        from rcpx.r2 import see_spot
        recs = [see_spot.look_of(ex, key) for key in sp.get("looks") or []]
        before = next((rec for _, rec, fixed in recs if rec is not None and fixed), None)
        got = None if before is None else see_spot.cloud_of(ex, before)
        if got is None:
            return no("the spot came with no look of a fixed camera that gives depth")
        view, PA, okA, _ = got
        # one depth picture of that camera now, with the joints and the jaws as they stand (what a look keeps); it
        # is no look of the session: nothing is noted in the space book and the operator cannot point in it
        now = {**ex._look_keeps(s_now, before["camera"]), "cam": before["cam"],
               "q": np.asarray(s_now.q, dtype=float).copy()}
        got = see_spot.cloud_of(ex, now)
        if got is None or got[1].shape != PA.shape:
            return no("the camera gave no depth picture after the lift")
        _, PB, okB, ownB = got
        eye = np.asarray(view.pos, dtype=float)
        rngA, rngB = np.linalg.norm(PA - eye, axis=-1), np.linalg.norm(PB - eye, axis=-1)
        far = float(np.median(rngA[okA]))
        foot = far / view.f
        margin = max(R.BAND * float(see_spot._sigma_at(before, far, foot)[0]), foot)
    except Exception as exc:        # a build with no seeing package, or a look that cannot be read: said
        return no(f"the held thing could not be looked at ({type(exc).__name__})")
    # the thing as the first look showed it: what stood inside its outline, over what it stood on
    c0, base, top = np.asarray(e["centre_xy"], dtype=float), float(e["base_z"]), float(e["top_z"])
    narrow, wide, bearing = float(e["narrow_m"]), float(e["wide_m"]), float(e["bearing_rad"])
    u = np.array([math.cos(bearing), math.sin(bearing)])
    v = np.array([-u[1], u[0]])
    grow = float(e.get("sigma_m") or 0.0) + foot
    d = PA[..., :-1] - c0
    thing = okA & (np.abs(d @ u) <= HALF * narrow + grow) & (np.abs(d @ v) <= HALF * wide + grow) \
        & (PA[..., -1] > base + margin) & (PA[..., -1] < top + margin)
    # where the hand's own motion since the close puts it, had nothing moved between the pads
    R0, R1 = np.asarray(s_close.R, dtype=float), np.asarray(s_now.R, dtype=float)
    tip1 = np.asarray(s_now.tip, dtype=float)
    Rh = R1 @ R0.T
    th = tip1 - Rh @ np.asarray(s_close.tip, dtype=float)
    at = Rh @ np.append(c0, base) + th
    # what the new look shows there that the first did not: nearer or further than before, over what it stood on,
    # no further from where the hand carries its middle than a thing still between the pads can stand
    new = okB & (~okA | (np.abs(rngB - rngA) > margin)) & (PB[..., -1] > base + margin) \
        & (np.linalg.norm(PB[..., :-1] - at[:-1], axis=-1) <= HALF * wide + wide)
    gap = R.BAND * R.QUANT * foot
    nA, hasA = _normals(PA, thing, eye, gap)
    nB, hasB = _normals(PB, new, eye, gap)
    X, N, Y, NY = PA[hasA] @ Rh.T + th, nA[hasA] @ Rh.T, PB[hasB], nB[hasB]
    least = R.QUANT * (len(at) + len(at))
    if len(X) < least or len(Y) < least:
        return no(f"too little of it is seen ({len(X)} points of its surface before the take, {len(Y)} after the lift)")

    def hidden(Xk):
        x, y, z = view.project(Xk)
        ok = np.isfinite(x) & np.isfinite(y) & (z > 0.0) & (x >= 0.0) & (x < view.w) & (y >= 0.0) & (y < view.h)
        out = ~ok
        out[ok] = ownB[np.floor(y[ok]).astype(int), np.floor(x[ok]).astype(int)]
        return out
    try:
        got = _lay(X, N, Y, NY, HALF * narrow, margin + foot, R.REL * R.REL * foot, hidden)
    except Exception as exc:        # a fit that cannot be solved: said, the pose before the close stays
        return no(f"its surface could not be laid onto the new look ({type(exc).__name__})")
    if got is None:
        return no("its surface before the take is not found again in the look after the lift")
    Rf, tf, c, pairs, shown, rms = got
    if pairs < R.GIVE * shown or rms > margin:
        return no(f"its surface before the take is found again in {pairs} of {shown} points the look after the "
                  f"lift can show, {rms * MM:.1f} mm off (one pixel is {foot * MM:.1f} mm): not taken as a reading")
    # the thing's own motion in the tool frame: a point of it that stood at x as the jaws closed stands at
    # R_T x + t_T now (the hand's motion is out of it, and so is what the camera is off by against the arm)
    R_T = R1.T @ Rf @ R1
    t_T = R1.T @ (Rf @ (tip1 - c) + c + tf - tip1)
    mid_T = np.asarray(mid_T, dtype=float)
    m_T = R_T @ mid_T + t_T
    q, _ = J.reading(s_now.gripper, ex._gripper_cmd)
    jaws = np.zeros(len(m_T)) if q is None else np.asarray(J.gap_centre_T(J.frac_of_q(q)), dtype=float)
    off = float(np.linalg.norm((m_T - jaws)[1:]))
    if off > wide:
        return no(f"the look puts its middle {off * MM:.0f} mm from the jaws, further than it is wide "
                  f"({wide * MM:.0f} mm): not taken as a reading")
    from rcpx.model import _rotvec
    turn, moved, sigma = float(np.linalg.norm(_rotvec(Rf))), float(np.linalg.norm((m_T - mid_T)[1:])), max(foot, rms)
    return {"read": True, "camera": before["camera"], "R_T": [[float(x) for x in row] for row in R_T],
            "t_T": [float(x) for x in t_T], "middle_T": [float(x) for x in m_T],
            "long_T": [float(x) for x in R1.T @ Rf @ Rh @ np.append(v, 0.0)], "turn_rad": turn, "moved_m": moved,
            **judged(moved, turn, wide, narrow, sigma), "narrow_m": narrow,
            "below_tip_m": float(m_T[0]), "sigma_m": sigma, "pairs": int(pairs), "shown": int(shown)}


# ---------------------------------------------------------------------------------------------- the close
def _follow(ex):
    """One moving jaw and no width seen: the arm's own target for each jaw command, so that the hand moves with
    the jaw and the gap's middle stays where it is in the world (both faces arrive together): the joints that
    put the middle of the gap at that opening (Jaws.gap_centre_T) on the point where it stands now, the tool
    pointing as it points. Where the arm cannot take that pose it stands where it last could. None when the
    hand has jaws that move together or the package cannot solve the hand's poses."""
    J = ex._pl_jaws
    solve = getattr(ex, "_pl_ik_held", None)
    if not J.one_jaw or solve is None:
        return None
    s = ex._sense()
    q0, _ = J.reading(s.gripper, ex._gripper_cmd)
    if q0 is None:
        return None
    full = ex.model.n >= len(s.tip) + len(s.tip)       # an arm that can hold a whole pose keeps its rotation
    centre = s.tip + s.R @ J.gap_centre_T(J.frac_of_q(q0))
    R0, last = s.R.copy(), [np.asarray(ex._q_des, dtype=float).copy()]

    def arm(frac):
        q = solve(last[0], centre, J.gap_centre_T(frac), R0[:, 0], R0 if full else None)
        if q is not None:
            last[0] = np.asarray(q, dtype=float)
        return last[0]
    return arm


def close(ex, expect_m=None, band_m=0.0, limit_n=None, limit_source=None, depth_m=None):
    """Close until the jaws' own reading tells a thing between them, or they meet each other: the core's one close
    (control_hold.close), with the keys the pick reads: {on, stopped: stalled | met | limit | time, stall_gap_m,
    wider, squeeze_n, squeeze_first_n, squeeze_cap_n, periods, arrive_periods, price, evidence}. A stall wider
    than expect_m + band_m is said (wider), never an ending here."""
    st = _state(ex)
    st.update(lifted=False, stall_gap=None)
    P = profile(ex)
    arm = _follow(ex) if expect_m is None else None
    r = control_hold.close(ex, expect_m=expect_m, band_m=band_m, limit_n=limit_n, limit_source=limit_source, arm=arm,
                           depth_m=depth_m)
    if "refused" in r:
        return {"on": False, "stopped": "refused", "refused": r["refused"], "stall_gap_m": None, "wider": False,
                "squeeze_n": None, "squeeze_first_n": None, "squeeze_cap_n": None, "periods": 0,
                "evidence": {"squeeze_cap_from": r["refused"]["why"], "jaw_numbers": P["source"]}}
    on = bool(r["held"])           # a touch was told and the jaws stand on the thing: the core's record stands
    gap = r["gap_m"]
    wider = bool(on and expect_m is not None and gap is not None and gap > float(expect_m) + float(band_m or 0.0))
    if on:
        st["stall_gap"] = gap
    cap = r.get("limit_n")
    return {"on": on, "stopped": {"on_thing": "stalled", "closed": "met"}.get(r["ended"], r["ended"]),
            "stall_gap_m": gap, "wider": wider, "squeeze_first_n": r["squeeze_n"], "squeeze_n": r["squeeze_n"],
            "squeeze_cap_n": None if cap is None else round(cap, 3), "periods": r["periods"],
            "arrive_periods": r["arrive_periods"], "price": r["price"],
            "evidence": {"arm_force_peak_n": round(r["arm_read_n"], 2), "closed_gap_mm": round(P["closed_gap"] * MM, 2),
                         "close_gap_speed_mm_s": round(r["rate_span_s"] * control_hold.numbers(ex)["gap_per_span_m"]
                                                       * MM, 2),
                         "squeeze_cap_n": None if cap is None else round(cap, 3),
                         "squeeze_cap_from": r["price"], "jaw_numbers": P["source"],
                         "hand_moved_with_jaw": arm is not None}}


def stroke(ex, cmd):
    """Bring the jaws to the opening cmd (a fraction) at their own rate: an opening is the core's release (it ends
    a hold), a closing its close with nothing held after it. Returns the rows older callers summed: one per
    control period, {"arm": the arm's largest force change over the stroke}."""
    frac, _ = control_hold.read(ex, ex._sense())
    if frac is None:
        return []
    if float(cmd) >= frac:
        r = control_hold.release(ex, float(cmd))
        return [{"arm": 0.0}] * int(r.get("periods") or 0)
    r = control_hold.close(ex, to_frac=float(cmd), hold=False)
    return [{"arm": float(r.get("arm_read_n") or 0.0)}] * int(r.get("periods") or 0)


def squeeze_to(ex, want_n, P=None):
    """Command the jaws to press want_n on what they stalled on (the command leads the reading by want_n over the
    servo's gain); returns (the squeeze read back once the reading stands, periods). Without a servo gain the
    closing command stays (its whole force) and the squeeze is the one read now."""
    R = _ratios()
    J = ex._pl_jaws
    q, target, gap, sq, lag = read(ex)
    lead = J.lead_for_squeeze(want_n)
    if q is None or lead is None:
        return sq, 0
    sign = np.sign(np.where(np.abs(J.span) > 0.0, J.span, 1.0))
    ex._gripper_cmd = min(1.0, max(0.0, J.frac_of_q(q - sign * lead)))
    n, last = 0, gap
    for _ in range(int(ex._wait_limit())):
        s = _period(ex)
        n += 1
        g1 = read(ex, s)[2]
        if g1 is None or abs(g1 - last) <= R.REL * profile(ex)["gap_still"]:
            break
        last = g1
    control_hold.note_squeeze(ex)    # the record follows the squeeze its holder asked
    return read(ex)[3], n


def ramp_step(ex, s_n, P):
    """The least step of a squeeze ramp at s_n (A4.5, the relative form): REL of the squeeze, never finer than
    TWO_READ times the squeeze's own spread or REL x REL of the jaw servo's own range."""
    R = _ratios()
    top = ex._pl_jaws.squeeze_top_n() or 0.0
    return max(R.TWO_READ * P["squeeze_noise"], R.REL * float(s_n), R.REL * R.REL * top)
