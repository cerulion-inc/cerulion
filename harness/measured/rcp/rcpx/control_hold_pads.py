# SPDX-License-Identifier: AGPL-3.0-only
"""What the pads of a hold bear and what they can make.
Pure over the jaws' own numbers (control_hold_jaws); nothing here moves anything.

    by_side(F, q, target, d)    the design rulings: what each pad presses the thing with where the jaws stand, by the jaw
                                servo's own reading through each pad's lever at the opening the jaws read
    stated(F, N, lever, c)      what a close states of its pads, for the hold's record and for the gates:
                                squeeze_pad_n (the squeeze the hardest pressed pad bears once the close stands, by
                                the jaw servo's own reading: by_side()), squeeze_by_body (the same per pad), the
                                goal the close was solved for as its own field (squeeze_goal_n), and the jaws'
                                numbers at that depth (z_jaw, floor_n, own_m_s) for a later landing
    strike(F, N, P, c, ...)     C8: {"pad_n", "other_n", "arrive_m_s", "source"}: the peak one pad can make on the
                                thing between the jaws, and the peak a jaw body can make on anything else it meets
                                while it closes
    rise(pads, f_arm_n, v)      the design rulings: the most a landing at the speed v can raise one pad's load on the held thing
                                over the squeeze that stood, by the arm's own figure along the travel and the jaws'
                                own stop slope
    planned(ex)                 the squeeze per pad a close of these jaws leaves standing, before any close ran (a
                                count made before the take)

Why rise() is an upper bound for a load that comes ALONG THE TRAVEL (the jaws' numbers are numbers along the closing
axis). A landing loads a pad two ways. (1) Along the travel, by friction: the held thing stops and the pads slide on
it or carry the arm's push into it; what all pads carry that way is what the arm puts into the thing along its
travel, f_arm (the set down's own strike along the travel: the guard's level plus the stop slope at that speed).
(2) Across the travel, on the closing axis: only when the landing moves the thing between the pads so that the gap
must change (it turns on a corner, it wedges up a jaw that is not square): a lever with some ratio r of gap speed to
travel speed. Whatever r is, two things hold at once: the lever gives the pad no more than f_arm / r (the arm's push
is all the lever has), and a jaw that is driven at the gap speed r x v answers with no more than z_jaw x r x v over
the squeeze that stood (its own stop slope: the inertia the jaw joint moves on the contact's own rate and what its
servo winds up; a servo that already stands on its force range adds nothing at rest). The lesser of the two is
largest where they are equal: sqrt(f_arm x z_jaw x v). So one pad's rise is at most f_arm + sqrt(f_arm x z_jaw x v),
for every lever the thing can make. What that does NOT bound is a thing that comes loose between the pads and is
caught again by the closing jaw: that strike is the jaw servo's own and has nothing of the landing's speed in it;
the set down reads the jaws while it lands and states that strike from the gap speed it read (after()).

SI inside; rates in spans per s, openings as shares of the span. Nothing here names a robot.
"""

import math

import numpy as np

from rcpx import bounds_ratios as R


def _finite(*xs):
    try:
        return all(x is not None and math.isfinite(float(x)) for x in xs)
    except (TypeError, ValueError):
        return False


def by_side(F, q, target, depth_m=None):
    """What each pad presses the thing with where the jaws stand now, by the jaw servo's own reading:
    per moving jaw joint its gain times its lag less the joint's friction, inside its own force range, through the
    lever where the thing stands (Jaws.pad_lever_at: the opening the jaws read, the depth the thing is taken at).
    A one-jaw gripper's fixed side bears what the moving jaw presses (the thing between them passes it on).
    {"n": [N per side], "bodies": [the side's body name], "lever_m": [per joint], "source"}, or None without a
    reading, a servo gain or a force range."""
    J = F["J"]
    lag = J.lag(q, target)
    if lag is None or F.get("kp") is None or F.get("force_range") is None:
        return None
    f = np.minimum(np.maximum(np.asarray(F["kp"], dtype=float) * lag - np.asarray(F["friction"], dtype=float), 0.0),
                   np.asarray(F["force_range"], dtype=float))
    lever = np.asarray(J.pad_lever_at(J.frac_of_q(q), depth_m), dtype=float)
    n = [float(v) for v in f / lever]
    names = J.side_names()
    if len(n) < len(names):
        n = n + [max(n)] * (len(names) - len(n))
    return {"n": n[:len(names)], "bodies": names, "lever_m": [float(v) for v in lever],
            "source": "the jaw servo's own lag at rest (its gain times the lag, less the joint's friction, inside its "
                      "force range) through each pad's lever where the jaws stand, at the depth the thing is taken at"
                      + ("; the fixed jaw bears what the moving jaw presses" if F.get("one_jaw") else "")}


def stated(F, N, lever, c):
    """The pads' part of a hold's record after a close c (control_hold_close.close's result, before the record is
    made). N, lever: control_hold_close.at_depth (the numbers and the pad's shortest lever at the depth the thing
    is taken at). squeeze_pad_n: what the jaw servo's own reading says the pad that presses hardest bears once the
    close stands (the design rulings: the squeeze is read where it stands; the force a landing with nothing stated ends at,
    the design rulings), from c["pads_by_side"] (by_side()) where the close read it, else the servo's lag at the jaw tip
    through the shortest lever; None when the jaws give no reading. squeeze_by_body: the same per pad, by the
    side's body name. The squeeze the close was SOLVED for is its own field (squeeze_goal_n): a goal, never the
    statement of what stands. What a pad can make in the close itself, its strike included, is the close's own
    strike.pad_n (strike())."""
    sides = c.get("pads_by_side") if isinstance(c.get("pads_by_side"), dict) else None
    by_body, src = None, None
    if sides is not None and sides.get("n") and all(_finite(v) for v in sides["n"]):
        by_body = {str(b): float(v) for b, v in zip(sides["bodies"], sides["n"])}
        stands, src = max(by_body.values()), sides.get("source")
    else:
        sq = c.get("squeeze_n")
        stands = None if sq is None else float(sq) * float(np.max(np.asarray(F["lever_tip"], dtype=float) / lever))
        src = "the jaw servo's own lag at rest through the pad's shortest lever at the depth the thing is taken at"
    per = float(N["gap_per_span_m"])
    goal = c.get("squeeze_goal_n")
    return {"squeeze_pad_n": stands if _finite(stands) else None, "squeeze_by_body": by_body,
            "squeeze_pad_source": src, "squeeze_goal_n": float(goal) if _finite(goal) else None,
            "z_jaw": N.get("z_jaw"), "z_jaw_source": N.get("z_jaw_source"), "floor_n": float(N["floor_n"]),
            "own_m_s": float(N["speed"]) * per, "gap_per_span_m": per, "cone": float(N.get("cone") or 1.0)}


def planned(ex):
    """Before any close ran: the pads' numbers a close of these jaws will leave ({"squeeze_pad_n", "z_jaw", ...} as
    stated()), from the jaws as they stand open; None for a robot with no jaws or no servo gain."""
    from rcpx import control_hold_close as C
    from rcpx.control_hold_jaws import facts, numbers, read
    F = facts(ex)
    frac, _ = read(ex, ex._sense())
    if F is None or frac is None:
        return None
    N, lever = C.at_depth(F, numbers(ex), None)
    goal = C.squeeze_goal(F, lever, frac, 0.0)
    out = stated(F, N, lever, {"squeeze_n": None, "squeeze_goal_n": goal})
    out["squeeze_pad_n"] = goal if _finite(goal) else None     # nothing stands yet: the plan is all there is
    out["squeeze_pad_source"] = "the squeeze a close of these jaws leaves standing (planned: nothing is held yet)"
    return out


def of_record(rec):
    """The pads' numbers a hold's record carries (stated()), or None for a record made without them."""
    pads = dict.get(rec, "pads") if rec is not None else None
    return pads if isinstance(pads, dict) else None


def strike(F, N, P, c, frac0, arm_n=0.0):
    """C8: the close's own strike. pad_n: the peak one pad can make on the thing between the jaws (the close's
    peak_n, ONE number: the strike of the rate the jaws ran at when their reading told the thing plus the
    squeeze they read standing, with what the arm read on top). other_n: the
    peak a jaw body can make on anything ELSE it meets while it closes: that contact is not backed by the
    other jaw and can begin anywhere on the jaw's way, so it is the strike of the FASTEST rate the close ran (the
    jaws' own rate where the free travel ran at it; frac0: the opening the close began at) by the jaws' stop
    slope, or the whole push of the jaw servo (its force range through the pad's shortest lever, else the squeeze
    the close was solved for) where that is more, plus what the arm read while the jaws closed (a finger that
    bears on the support carries the arm's push too). other_n None where the jaws state no stop slope."""
    z, per = N.get("z_jaw"), float(N["gap_per_span_m"])
    pad = c.get("peak_n")
    if not z:
        return {"pad_n": pad, "other_n": None, "arrive_m_s": c.get("arrive_m_s"),
                "source": "these jaws state no stop slope: no figure"}
    lever = np.asarray(F["lever_min"], dtype=float)
    ran = float(P["own"] if float(P["slow_from"]) < float(frac0) else P["rate"])
    dyn = float(N["floor_n"]) + float(z) * ran * per
    push = None
    if F.get("force_range") is not None:
        push = float(np.max(np.asarray(F["force_range"], dtype=float) / lever))
    elif _finite(P.get("squeeze_goal_n")):
        push = float(P["squeeze_goal_n"])
    other = max(dyn, push or 0.0)
    if N.get("z_jaw_source") == "model":
        other *= 1.0 + R.REL
    other = other * float(N.get("cone") or 1.0) + float(arm_n or 0.0)      # inside the jaws' friction cone
    return {"pad_n": pad, "other_n": other, "arrive_m_s": c.get("arrive_m_s"), "fastest_m_s": ran * per,
            "source": ("the jaws' own stop slope (" + ("the body's model" if N.get("z_jaw_source") == "model" else
                                                       "measured") + ") at the fastest rate this close ran, "
                       f"{ran * per * 1000.0:.0f} mm/s, or the jaw servo's whole push where that is more, times "
                       f"{float(N.get('cone') or 1.0):.2f} (the friction cone of the jaws' own shapes); plus what "
                       "the arm read while the jaws closed")}


def rise(pads, f_arm_n, v_m_s):
    """the design rulings: (the most a landing at v_m_s can raise one pad's load on the held thing over the squeeze that stood,
    N; the lever term alone) by the arm's own figure along the travel f_arm_n and the jaws' own stop slope (module
    text): f_arm + sqrt(f_arm x z_jaw x v). (None, None) where the arm states no figure; with no stop slope of the
    jaws the lever term is the arm's figure again (a lever cannot give a pad more than the arm's push where the
    gap moves at least as fast as the hand; a slower gap needs the slope to bound it, said by the caller)."""
    if not _finite(f_arm_n, v_m_s):
        return None, None
    f, v = max(0.0, float(f_arm_n)), max(0.0, float(v_m_s))
    z = None if not isinstance(pads, dict) else pads.get("z_jaw")
    lever = f if not _finite(z) or not float(z) > 0.0 else math.sqrt(f * float(z) * v)
    return f + lever, lever


def caught(pads, gap_m_s):
    """A thing that came loose between the pads and was caught again by the closing jaw: the strike of the gap
    speed the jaws' own reading showed, by the jaws' stop slope (N); 0 where the jaws did not move or state no
    slope."""
    z = None if not isinstance(pads, dict) else pads.get("z_jaw")
    if not _finite(z, gap_m_s) or not float(gap_m_s) > 0.0:
        return 0.0
    out = float(pads.get("floor_n") or 0.0) + float(z) * float(gap_m_s)
    out = out * (1.0 + R.REL) if pads.get("z_jaw_source") == "model" else out
    return out * float(pads.get("cone") or 1.0)
