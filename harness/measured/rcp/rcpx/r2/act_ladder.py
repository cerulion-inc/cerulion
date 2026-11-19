# SPDX-License-Identifier: AGPL-3.0-only
"""Pure continuous force ramp and the words of a measured press.
Numbers in, numbers and words out; no motion or executor. The hand-stated vectors
and exact source mutants are in tests/test_r2_act.py and its baseline fixture.
"""
from rcpx import r3_limits as L

WORDS = ("cap", "base_moved", "not_square", "gave", "reached_end", "moving", "came_back", "stayed_in", "small_move", "rigid")
STOP_WORDS = {"cap": "cap", "base_moved": "base_moved", "not_square": "not_square", "reached_end": "reached_end"}
END_STOPS = ("force", "stall")
MOVING_STOPS = ("travel",)


def rate(f_n, sigma_f_n, e, t_det_s, ceiling_n=None):
    """N/s at the present force: max(TWO_READ sigma, e F) / t_det, e the measured torque error or REL; where the
    noise is zero the least step is REL x REL x the ceiling per t_det (rcpx.bounds_laws.ramp_rate)."""
    from rcpx import bounds_laws
    return bounds_laws.ramp_rate(f_n, sigma_f_n, e, t_det_s, ceiling_n or 0)


def ramp_time(first_n, ceiling_n, sigma_f_n, e, t_det_s):
    """Seconds the law takes from first to ceiling: linear up to its knee, then exponential
    (rcpx.bounds_laws.ramp_time)."""
    from rcpx import bounds_laws
    return bounds_laws.ramp_time(first_n, ceiling_n, sigma_f_n, e, t_det_s, ceiling_n)


def step(target_n, read_n, moving, rate_n_s, dt_s, ceiling_n, tol_n):
    """Advance by rate times dt, clamped to the ceiling.
    Wait while a moving part leaves the force reading under target minus tolerance."""
    if moving and read_n < target_n - tol_n:
        return target_n
    return min(ceiling_n, target_n + rate_n_s * dt_s)


def follows_tol(target_n, sigma_f_n, e):
    """Force follow error: max(TWO_READ sigma, e target), e nonpositive uses REL."""
    e = e if e > 0 else L.ratio("REL")
    return max(L.ratio("TWO_READ") * sigma_f_n, e * target_n)


def gave(peak_n, read_n, past_peak_mm, rigid_mm, thr_n):
    """Give: a resolved peak, reading under GIVE of it, and rigid travel past it."""
    return peak_n >= thr_n and read_n < L.ratio("GIVE") * peak_n and past_peak_mm >= rigid_mm


def hard_stop(moved, f_ref_n, f_now_n, gained_mm, rigid_mm, k_shown_n_per_mm, tol_n):
    """After motion, a resolved force rise with less than GIVE of its predicted travel.
    With no shown stiffness, require a tolerance rise and less than rigid travel."""
    if not moved:
        return False
    rise = f_now_n - f_ref_n
    k = k_shown_n_per_mm or 0
    if k > 0:
        return rise >= max(tol_n, k * rigid_mm) and gained_mm < L.ratio("GIVE") * rise / k
    return rise >= tol_n and gained_mm < rigid_mm


def ceiling(terms):
    """Return the smallest (force, source, evidence) term, or None when empty."""
    return min(terms, key=lambda t: t[0]) if terms else None


def word(reading, until, rigid_mm, small_mm, reached_mm=None, moving_mm=None):
    """The first measured press word that holds; end and moving default to QUANT rigid."""
    reached_mm = L.ratio("QUANT") * rigid_mm if reached_mm is None else reached_mm
    moving_mm = L.ratio("QUANT") * rigid_mm if moving_mm is None else moving_mm
    r = reading or {}
    stopped = r.get("stopped")
    along = float(r.get("along_mm") or 0.0)
    total = float(r.get("total_mm") if r.get("total_mm") is not None else along)
    if stopped == "cap":
        return "cap"
    if r.get("base_moved") is True:
        return "base_moved"
    if stopped == "off_axis":
        return "not_square"
    if stopped == "gave" or r.get("gave") is True:
        return "gave"
    if until == "stop" and stopped in END_STOPS and total >= reached_mm:
        return "reached_end"
    if until == "stop" and along >= moving_mm and stopped in MOVING_STOPS:
        return "moving"
    if r.get("sprang_back") is True and float(r.get("sprang_back_mm") or 0.0) >= rigid_mm:
        return "came_back"
    if along >= small_mm:
        return "stayed_in"
    if along >= rigid_mm:
        return "small_move"
    return "rigid"


def decide(w, until, stopped, at_ceiling):
    """What follows a ramp leg whose word is w: ("stop", stop word) or ("on", None), on = another leg from the
    present force. cap, base_moved, not_square, reached_end stop with that word; bottomed stops ceiling; gave or
    stayed_in stop changed (until stop: on, to reached_end); moving stops time when the stroke ended with the part
    still moving, else on; came_back or small_move stop ceiling at the ceiling or at the part's own end (the leg
    stopped force or stall), else on; rigid stops rigid."""
    if w in STOP_WORDS:
        return "stop", STOP_WORDS[w]
    if w == "bottomed":
        return "stop", "ceiling"
    if w in ("gave", "stayed_in"):
        return ("on", None) if until == "stop" else ("stop", "changed")
    if w == "moving":
        return ("stop", "time") if until == "stop" and stopped == "travel" else ("on", None)
    if w in ("came_back", "small_move"):
        return ("stop", "ceiling") if at_ceiling or stopped in END_STOPS else ("on", None)
    if w == "rigid":
        return "stop", "rigid"
    return ("stop", "ceiling") if at_ceiling else ("on", None)


def run(first_n, ceiling_n, legs, until="give"):
    """Test driver: retain each leg's measured end force and apply decide in order."""
    forces = []
    for w, stopped, force_n in legs:
        forces.append(force_n)
        action, stop = decide(w, until, stopped, force_n >= ceiling_n)
        if action == "stop":
            return forces, stop
    return forces, None


def binding(items):
    """Smallest force item and 'the most allowed here' with its evidence."""
    forces = [item for item in items if item.get("what") == "force" and item.get("value") is not None]
    item = min(forces, key=lambda item: item["value"]) if forces else None
    return item, "the most allowed here (" + str(item.get("evidence") or "") + ")" if item is not None else ""


def lane_reading(res):
    """The lane's travel less slack is along travel; preserve its give and lateral reading."""
    r = res or {}
    travel = float(r.get("travel_mm") or 0.0)
    slack = float(r.get("slack_mm") or 0.0)
    stop = r.get("stopped")
    stopped = {"give": "gave", "stall": "stall", "time": "time", "travel": "travel", "cap": "cap"}.get(stop, stop)
    along = max(0.0, travel - slack)
    return {"stopped": stopped, "gave": bool(r.get("gave")), "base_moved": False, "along_mm": round(along, 2),
            "total_mm": round(along, 2), "sprang_back": False, "sprang_back_mm": 0.0,
            "lateral_mm": r.get("lateral_mm")}


__all__ = ["WORDS", "STOP_WORDS", "END_STOPS", "MOVING_STOPS", "rate", "ramp_time", "step", "follows_tol",
           "gave", "hard_stop", "ceiling", "word", "decide", "run", "binding", "lane_reading"]
