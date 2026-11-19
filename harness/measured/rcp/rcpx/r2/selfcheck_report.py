# SPDX-License-Identifier: AGPL-3.0-only
"""Reports of the selfcheck package: the fit's numbers as operator results (degrees, mm, pixels), the rules
that call an error significant or measurable, the verdict, and the "measured" block of robot://body.

Rules (one place, so the tools and the body document agree):
- an offset is MEASURABLE when the camera can tell it apart (the fit kept it free), its spread is at most
  SIGMA_OK_DEG and the joint's sign and gain are right; only a measurable offset gets apply_with_deg;
- it is SIGNIFICANT when it is at least OFFSET_MIN_DEG and three spreads; self_check corrects those;
- the camera mount is significant at CAMERA_MIN_DEG or CAMERA_MIN_MM (and three spreads) and correctable when
  every mount number was free in the fit and the spreads are small;
- when any joint's wiggle moved the picture by another amount than predicted (a gain off 1, or no motion), no
  offset and no mount from that check is corrected or offered to apply_offset (scale_problems says why);
- a joint asked for that got no usable look makes the verdict inconclusive, and what the camera could not judge
  at all is listed (unchecked), so "ok" never covers a joint nobody looked at.
"""

import math

import numpy as np
from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx.r2 import selfcheck_limits as limits
from rcpx.r2 import selfcheck_fit as fit

DEG = math.pi / 180.0
MM = 1000.0
OFFSET_MIN_DEG = 0.3
SIGMA_OK_DEG = 0.35
CAMERA_MIN_DEG = 0.5
CAMERA_MIN_MM = 3.0
CAMERA_SIGMA_OK = (0.3, 3.0)      # deg, mm
SAID_JOINTS = 3  # numeric: names in the one-sentence compensation summary


def false_stop_row(names, rec):
    out = {**rec, 'joint': names[rec['joint']]}
    if rec['kind'] == 'dynamic':
        out['said'] = (f"The force rise of joint {out['joint']} was gone while its last target was held; "
                       f"the wiggle resumes at speed factor {rec['speed_factor']:g}.")
    else:
        out['said'] = (f"The force estimate followed joint {out['joint']}'s angle as a wrong body would make it; "
                       "the wiggle resumes with that bias line removed.")
    return out


def result_sentence(out, sign_bad=()):
    """One sentence describing the measured ending and any necessary next call."""
    if 'zero_id' in out:
        joint = out['joint']
        if out.get('verdict') == 'no_stop':
            why = out['why'].rstrip('.')
            return f"Joint {joint} supplied no usable stop because {why}."
        return (f"Joint {joint}'s {out['end']} stop gives offset {out['offset_deg']:+.3f} deg with spread "
                f"{out['offset_sigma_deg']:.3f} deg, so its zero is {out['verdict']}.")
    verdict = out.get('verdict', 'inconclusive')
    if verdict == 'ok':
        return "The checked joints and camera agree with the body description within their measured spreads."
    if verdict == 'refuse_motion':
        names = list(sign_bad) or [n for n, row in out.get('joints', {}).items() if row.get('sign_ok') is False]
        return f"Joint {', '.join(names)} turns the wrong way for the body description; do not move."
    if verdict == 'compensated':
        rows = out.get('compensation', {}).get('applied_now', [])
        parts = [f"{r['joint']} {r['offset_deg']:+.3f} deg" for r in rows[:SAID_JOINTS]]
        if len(rows) > SAID_JOINTS:
            parts.append(f"and {len(rows) - SAID_JOINTS} more")
        camera = out.get('camera', {})
        if camera.get('state') == 'corrected':
            parts.append(f"camera mount {camera['rot_err_deg']:g} deg and {camera['shift_mm_est']:g} mm")
        return "The body check corrected " + ', '.join(parts) + "."
    if verdict == 'uncompensated':
        names = [n for n, row in out.get('joints', {}).items() if row.get('problem') or row.get('offset_significant')]
        why = ('joints ' + ', '.join(names) + ' disagree with the body description') if names else 'the camera mount disagrees with the body description'
        return f"The body check left errors uncorrected because {why}; use the findings before moving."
    why = out.get('why') or next(iter(out.get('findings', [])), 'some requested joints could not be judged')
    hint = out.get('hint', 'self_check() after moving to a free marked view')
    if 'self_check(' not in hint:
        hint += ', then self_check()'
    return f"The body check is inconclusive because {why.rstrip('.')}; next call: {hint.rstrip('.')}."


def joint_table(names, a, active_deg, wiggled, max_deg):
    """({joint name: row}, [names whose offset self_check should correct]) from fit.analyse's result."""
    n = len(names)
    th, sg, free_gain = a["theta"], a["sigma"], a["theta_gains_free"]
    table, todo = {}, []
    for j, name in enumerate(names):
        rms = [r for lk, r in zip(a["looks_used"], a["look_rms_px"]) if lk["joint"] == j]
        state = a["gain_state"][j]
        row = {"checked": j in wiggled and bool(rms), "sign_ok": None, "gain_ratio": None,
               "offset_deg_est": None, "offset_sigma_deg": None, "seen": False,
               "residual_px": round(max(rms), 3) if rms else None,
               "applied_offset_deg": round(float(active_deg[j]), 3), "apply_with_deg": None}
        if not rms:
            row["note"] = ("this joint was not wiggled" if j not in wiggled else
                           "no look of this joint's wiggle could be matched to the reference look")
        elif state == "unseen":
            row["note"] = "turning this joint does not move the camera over the surface: the camera cannot check it"
        else:
            g = float(free_gain[n + j])
            row["gain_ratio"] = round(g, 4)
            row["gain_sigma"] = round(float(sg[n + j]), 4) if math.isfinite(sg[n + j]) else None
            if state == "none":
                row["note"] = "the picture barely moved when this joint turned: the camera may not be where the " \
                              "body description puts it"
                row["problem"] = "no_motion"
            else:
                row["sign_ok"] = state != "sign"
                if state == "sign":
                    row["problem"] = "sign"
                elif state == "gain":
                    row["problem"] = "gain"
        if a["free"][j] and rms and state in ("sign", "gain", "none"):
            row["not_seen_why"] = "this joint does not turn as its description says, so its zero cannot be read"
        elif a["free"][j] and rms:
            est, sig = float(th[j]) / DEG, float(sg[j])
            row.update({"offset_deg_est": round(est, 3), "offset_sigma_deg": round(sig, 3), "seen": True})
            if abs(est) >= max(OFFSET_MIN_DEG, 3.0 * sig):
                row["offset_significant"] = True
            if sig <= SIGMA_OK_DEG:
                row["apply_with_deg"] = round(float(active_deg[j]) + est, 3)
                if row.get("offset_significant"):
                    todo.append(name)
        elif rms:
            row["not_seen_why"] = a["why_held"].get(j, "the camera cannot tell this joint's offset from these looks")
        table[name] = row
    scale = scale_problems(table)
    if scale:                 # see scale_problems: nothing read with the gains held at 1 is offered for use
        for row in table.values():
            if row["apply_with_deg"] is not None:
                row["apply_with_deg"] = None
                row["withheld"] = ("not offered to apply_offset: the picture did not move as far as predicted when "
                                   f"{', '.join(scale)} turned, and this estimate would absorb that mismatch")
        todo = []
    for name in list(todo):   # apply_offset sets at most max_deg either way; self_check does not go past that
        bound = max_deg[names.index(name)]
        if abs(table[name]["apply_with_deg"]) > bound:
            todo.remove(name)
            table[name]["withheld"] = (f"over {bound:g} deg: too large to be a zero offset, more likely a wrong "
                                       "description or a view the camera lost track of")
            table[name]["apply_with_deg"] = None
    return table, todo


def camera_block(model, a, cc):
    """(the camera part of a self_check result, (turn rad, shift m) to correct or None)."""
    n = model.n
    th, sg = a["theta"], a["sigma"]
    r, s = np.asarray(th[2 * n:2 * n + 3], float), np.asarray(th[2 * n + 3:2 * n + 6], float)
    sr, ss = sg[2 * n:2 * n + 3], sg[2 * n + 3:2 * n + 6]
    all_free = bool(np.all(a["free"][2 * n:]))
    turn_sig = float(np.max(sr)) if all_free else float("inf")
    shift_sig = float(np.max(ss)) if all_free else float("inf")
    rot_err, shift = float(np.linalg.norm(r)) / DEG, float(np.linalg.norm(s)) * MM
    t3 = 3.0 * turn_sig if math.isfinite(turn_sig) else 0.0
    s3 = 3.0 * shift_sig if math.isfinite(shift_sig) else 0.0
    significant = rot_err >= max(CAMERA_MIN_DEG, t3) or shift >= max(CAMERA_MIN_MM, s3)
    correctable = all_free and turn_sig <= CAMERA_SIGMA_OK[0] and shift_sig <= CAMERA_SIGMA_OK[1]
    turn_now, shift_now = cc.mount_deg_mm()
    cam = {"rot_err_deg": round(rot_err, 3), "rot_err_vec_deg": [round(float(x) / DEG, 3) for x in r],
           "rot_sigma_deg": round(turn_sig, 3) if math.isfinite(turn_sig) else None,
           "shift_mm_est": round(shift, 2), "shift_vec_mm": [round(float(x) * MM, 2) for x in s],
           "shift_sigma_mm": round(shift_sig, 2) if math.isfinite(shift_sig) else None,
           "axes": "the camera's own frame: x image right, y image up, z toward the viewer",
           "state": "uncertain" if significant else "ok",
           "correction_before_this_check": {"turn_deg": [round(x, 3) for x in turn_now],
                                            "shift_mm": [round(x, 2) for x in shift_now]},
           "note": "an offset of the joint that carries the camera is the same picture as a mount turn and is "
                   "counted here"}
    if not all_free:
        cam["note"] += "; these looks could not tell every mount number, so nothing is corrected"
    return cam, ((r, s) if significant and correctable else None)


SCALE_PROBLEMS = ("gain", "no_motion")   # the picture did not move as far as the description predicts


def scale_problems(table):
    """Names of the joints whose wiggle moved the picture by another amount than the description predicts. With any
    of these the zero offsets and the mount of the same check are not to be trusted: they are read with every
    gain held at 1 and the surface at its configured height, and a picture that moves a few percent off is
    explained by joint zeros that are wrong by degrees."""
    return [name for name, row in table.items() if row.get("problem") in SCALE_PROBLEMS]


def unchecked(names, table, wiggled):
    """(rows, missing). rows: what the check asked for and could not judge, each {"joint", "what", "why", "next"}.
    missing: the wiggled joints with no usable look at all (a leg refused by the path check, a look that matched
    nothing): with one of those the check is not a whole check, and the verdict says so."""
    rows, missing = [], []
    for j, name in enumerate(names):
        row = table[name]
        if j not in wiggled or row.get("problem"):
            continue
        if not row["checked"]:
            missing.append(name)
            rows.append({"joint": name, "what": "direction, scale and zero offset", "why": row.get("note", ""),
                         "next": "move so this joint can turn a few degrees each way with the surface in view, "
                                 "then self_check again"})
        elif row["sign_ok"] is None:
            rows.append({"joint": name, "what": "direction, scale and zero offset", "why": row.get("note", ""),
                         "next": f"zero_check(joint={name!r}) reads its zero at the joint's end stop"})
        elif not row["seen"]:
            rows.append({"joint": name, "what": "zero offset", "why": row.get("not_seen_why", ""),
                         "next": f"zero_check(joint={name!r}) reads its zero at the joint's end stop"})
    return rows, missing


def verdict_of(table, cam, sign_bad, applied, missing=(), surface_z_mm=None):
    """(verdict, findings): refuse_motion on a sign error; uncompensated when something found is not corrected;
    inconclusive when a joint asked for got no usable look; compensated when everything found is corrected; ok
    with nothing found."""
    findings, open_ = [], 0
    scale = scale_problems(table)
    if scale:
        at = "" if surface_z_mm is None else f" (taken as z = {surface_z_mm:g} mm)"
        findings.append("the picture did not move as far as the body description predicts when "
                        f"{', '.join(scale)} turned: that joint's scale is off, or the surface under the arm is not "
                        f"at the configured height{at}, or the surface repeats and the camera lost track. No zero "
                        "offset or camera correction from this check is applied: they would absorb that mismatch")
    for name in missing:
        findings.append(f"joint {name} was not checked: {table[name].get('note', 'no usable look')}")
    for name, row in table.items():
        prob = row.get("problem")
        if prob == "sign":
            findings.append(f"joint {name} turns the opposite way to its description (gain {row['gain_ratio']:+.2f})")
            open_ += 1
        elif prob == "gain":
            findings.append(f"joint {name} turns {row['gain_ratio']:.3f} times what its reading says (not corrected)")
            open_ += 1
        elif prob == "no_motion":
            findings.append(f"joint {name}: the picture did not move as the description says it should")
            open_ += 1
        if row.get("offset_significant") and scale:
            findings.append(f"joint {name} zero reads {row['offset_deg_est']:+.2f} deg off in this check (not "
                            "trusted, not corrected)")
        elif row.get("offset_significant"):
            done = name in applied
            findings.append(f"joint {name} zero is off by {row['offset_deg_est']:+.2f} deg "
                            f"({'corrected' if done else 'not corrected'})")
            open_ += 0 if done else 1
    if cam["state"] != "ok" and scale and cam["state"] != "corrected":
        findings.append(f"the camera mount reads {cam['rot_err_deg']:.2f} deg and {cam['shift_mm_est']:.1f} mm off "
                        "in this check (not trusted, not corrected)")
    elif cam["state"] != "ok":
        findings.append(f"the camera mount is off by {cam['rot_err_deg']:.2f} deg and {cam['shift_mm_est']:.1f} mm "
                        f"({'corrected' if cam['state'] == 'corrected' else 'not corrected'})")
        open_ += 0 if cam["state"] == "corrected" else 1
    if sign_bad:
        return "refuse_motion", findings
    if open_:
        return "uncompensated", findings
    if missing:
        return "inconclusive", findings
    return ("compensated" if findings else "ok"), findings


def measured_doc(ex, st):
    """The "measured" block of robot://body: what was checked on the running robot, and what is corrected."""
    names = ex.model.joint_names
    last = st["last"]
    joints = {}
    rows = (last or {}).get("joints") or {}
    for name in names:
        row, rec = rows.get(name) or {}, st["measured"].get(name)
        joints[name] = {"zero_offset_applied_deg": st.get("offsets_deg", {}).get(name, 0.0),
                        "sign_ok": row.get("sign_ok"), "gain_ratio": row.get("gain_ratio"),
                        "offset_seen_by_camera": row.get("seen"),
                        "zero_offset_measured": None if rec is None else
                        {"deg": rec["deg"], "sigma_deg": rec["sigma_deg"], "by": rec["source"], "kind": rec["kind"],
                         "usable_for_apply_offset": bool(rec["epoch"] == ex.epoch and rec["gen"] == st["gen"])}}
    doc = {"source": "measured",
           "checked": bool(last) and last.get("verdict") != "inconclusive",
           "self_check": None if last is None else {k: last.get(k) for k in ("check_id", "verdict", "epoch", "t_s",
                                                                             "findings", "why", "unchecked")},
           "stale": st["stale"], "joints": joints, "camera": dict(st["camera"]), "zero_checks": dict(st["zero"]),
           "motion": "refused (body_sign)" if st["sign_error"] else "allowed",
           "note": "true joint angle = reading + zero offset; the joint table and limits above are the description, "
                   "this block is what the running robot showed"}
    if last is None:
        doc["note"] = ("not checked yet: self_check measures whether the running robot matches this "
                       "description (zero offsets, joint signs). Run it when a move or a look lands away from where "
                       "the description puts it, or after the arm was struck; otherwise the description holds. "
                       + doc["note"])
    try:
        B, q = bounds.of(ex), ex._sense().q
        zero = {}
        for j, name in enumerate(names):
            end = q.copy()
            end[j] = ex.model.q_hi[j] if ex.model.q_hi[j] - q[j] <= q[j] - ex.model.q_lo[j] else ex.model.q_lo[j]
            zero[name] = {"fast_deg_s": math.degrees(B.qd_max()[j]),
                          "slow_deg_s": math.degrees(limits.zero_slow(ex, end, j, False)[0]),
                          "slow_zone_deg": math.degrees(limits.zero_zone(ex, st, end, j, name)[0])}
        amps = limits.amplitudes(ex, q, range(ex.model.n), ex.cfg["floor_z_m"])[0] if fit.Body(ex.model, q, ex.cfg["floor_z_m"]).sees_floor() >= fit.MIN_FLOOR_SHARE else None
        doc["limits"] = {"zero_check": zero, "self_check": {
            "amplitudes_deg": None if amps is None else {names[j]: math.degrees(v) for j, v in amps.items()},
            "contact_n": B.f_thr(0, 0)}}
    except Exception as error:
        doc["limits"] = None
        doc["limits_why"] = f"{type(error).__name__}: {error}"
    return doc
