# SPDX-License-Identifier: AGPL-3.0-only
"""The hold: one record, one close, and what a guard does with what the robot does to its
own force reading.

The robot knows three things it does to its own reading: it squeezes, it carries, it lets go. Each has a name here
and a bound with a source; a reading it cannot explain is still a contact.

- ex._held (control.Executor) is the only record of what the hand holds and only this file writes it:
  {since_call, gap_m, squeeze_n, squeeze_source, rest_n, rest_ext, leg_in_m, weight_n (None until read at rest),
  mass_kg (0 until then: a planner that bounds the grip's acceleration by the held mass reads a number), sigma_n,
  read_at, kind (None, free, attached), level_n, level_source, limit_n, limit_source, extent, stands (stated)};
  beside them what the Watch keeps between legs (judged_at, first_ran, load_seen, shown_n, leg_stop, reread).
- The record ends when the jaws are opened (release, or a command open of where they stand), when they close
  past where they held the thing and come to their command or meet each other (slipped; jaws that stand again
  short of their command hold a thing that shifted: the record follows them), or when a rise shows no load and
  no press (stayed).
- close(): the one close. No arm guard at the contact threshold judges it, only the cap. With no limit the jaws run
  at their own rate and the receipt prices that; with a limit L the rate at the touch is at most
  (L - floor_n) / z_jaw and never under rate_tell, and the command never leads the reading by more than L allows
  through the pad's shortest lever. "Told" is the jaws' own reading: the lag over the free lag of the running rate
  by REL of it plus TWO_READ spreads, on QUANT readings in a row. Then the command goes onto the reading, then to
  the hold squeeze. The arm's estimate is read through the whole close and said when the squeeze went into the
  world.
- release(): opening is the third expected event (the load leaves); it stops short of the jaws' own open end.
- Watch: what Guard (control_base) does while a record stands. First leg after a close (the load not read yet):
  no contact threshold on the tool for no more travel than the last leg in, the cap, the lost hold and a push
  located off the tool stay; then a level made of the rest reading and the load seen so far, until a move that
  ended at its goal let the load be read. Known hold: the larger of the free threshold and the hold's level; a
  leg that begins on a reading that already stands along gravity takes the weight from it (the load is then
  read again at the leg's end). A hold on a part of something fixed (attached): the part's own resistance is
  no contact up to DOUBLE what it showed while it was worked (shown_n); the cap stays. A contact located on
  the tool while holding is kept on the hold; a push located off the tool (a link) stays a contact of the arm.
- read_load(): waits by motion alone, then reads the load from the RAW estimate over the settle window. It is
  read at the rest that ends the first move after a close when that move ended at its goal and was long enough to
  tell, at every such rest while the hold is on a part of something fixed (it may have come free), and again for
  a free hold whose leg began on a standing reading (the weight follows down as well as up).

SI inside. Nothing here names a robot; no number is typed (rcpx/bounds_ratios.py holds the ratios).
"""

import math

import numpy as np

from rcpx import bounds_ratios as R
from rcpx.control_hold_jaws import facts, measure_jaws, numbers, presses, read  # noqa: F401  (measure_jaws: the jaws' own numbers)

MM = 1000.0


def _num(x):
    """A bounds answer as a number (an item's value, a pair's first part, or the number)."""
    if isinstance(x, dict):
        x = x.get("value")
    elif isinstance(x, (tuple, list)):
        x = x[0]
    return None if x is None else float(x)


class Hold(dict):
    """The record. A key whose value is not known yet (None) answers get() with the default."""

    def get(self, key, default=None):     # compat shim: until every reader takes a weight of None as "not read yet"
        value = dict.get(self, key, default)
        return default if value is None else value


# ---------------------------------------------------------------------------------------------- the record
def held(ex):
    """The record, or None. It ends here when a caller set the jaws' command open of where they stand on the
    thing (the jaws were opened on purpose by something that did not call release)."""
    rec = ex.__dict__.get("_hold")
    if rec is None:
        return None
    cmd = getattr(ex, "_gripper_cmd", None)
    if cmd == "open" or (isinstance(cmd, (int, float)) and float(cmd) > rec["frac"] + rec["still_frac"]):
        _end(ex, "opened")
        return None
    return rec


def check(ex):
    """The record after one look at the jaws' newest reading (a hold that was lost ends here), or None."""
    rec = held(ex)
    if rec is not None and ex._last is not None and _lost(ex, rec, ex._last):
        return None
    return rec


def stated(ex, value):
    """A caller's own held block (a tool's facts about what it took): the record takes its extent, the error of
    that extent, the grip's proofs and where the thing stands in the hand; never a mass or a weight."""
    rec = held(ex)
    if rec is None or not isinstance(value, dict):
        return
    for key in ("extent", "extent_m", "sigma_m", "proofs", "stands"):
        if value.get(key) is not None:
            rec[key] = value[key]


def note_squeeze(ex):
    """The hold's own holder moved the jaws' command after the close (a squeeze it asked): the record takes the
    squeeze and the command as they stand now."""
    rec = held(ex)
    F = facts(ex)
    if rec is None or F is None or ex._last is None:
        return
    q, target = F["J"].reading(ex._last.gripper, ex._gripper_cmd)
    if q is not None:
        rec.update(squeeze_n=F["J"].squeeze_n(q, target), cmd_frac=float(ex._gripper_cmd))


def ended(ex):
    """How the last hold ended ({why, at_call, squeeze_n, weight_n}), or None."""
    return ex.__dict__.get("_hold_ended")


def _end(ex, why, **more):
    rec = ex.__dict__.get("_hold")
    ex.__dict__["_hold"] = None
    if rec is not None:
        note = {"why": why, "at_call": int(ex._calls), "squeeze_n": dict.get(rec, "squeeze_n"),
                "weight_n": dict.get(rec, "weight_n"), **more}
        ex.__dict__["_hold_ended"] = note
        ex._journal("hold_ended", **{k: v for k, v in note.items() if not isinstance(v, np.ndarray)})


def _relevel(ex, rec):
    """level_n: the largest of the free threshold, the size of the rest reading the close left and the weight
    as it was last read (it follows the weight down as well as up); for a hold on a part of something fixed also
    DOUBLE what the part showed while it was worked (in its first leg, and at each rest it was read at).
    level_source names the one that set it."""
    terms = [(_num(ex.bounds.f_thr(0.0, 0.0)) or 0.0, "the free contact threshold"),
             (float(np.linalg.norm(rec["rest_n"])), "this hold: the squeeze's own reaction at rest"),
             (float(dict.get(rec, "weight_n") or 0.0), "this hold: the weight read at rest")]
    if dict.get(rec, "kind") == "attached":
        terms.append((R.DOUBLE * float(dict.get(rec, "shown_n") or 0.0),
                      "this hold: DOUBLE what the part showed while it was worked"))
    rec["level_n"], rec["level_source"] = max(terms, key=lambda t: t[0])


def level(ex):
    """The contact level in force: B.f_thr(0, 0) with no hold, the hold's own level with one."""
    free = _num(ex.bounds.f_thr(0.0, 0.0)) or 0.0
    rec = held(ex)
    return free if rec is None else max(free, float(rec["level_n"]))


def rest_force(ex, s):
    """The force at rest less the held load (N). Nothing held: the estimate. A free hold whose load is read: the
    estimate, which has that load off. A hold whose load is not read yet, one on a part of something fixed, or
    one whose load is to be read again (its leg began on a standing reading): nothing (what the still arm reads
    then is the hold's own doing, so a wait judges its motion alone)."""
    rec = held(ex)
    if rec is None or (dict.get(rec, "kind") == "free" and not rec.get("reread")):
        return float(s.Fmag)
    return 0.0


def _still(ex, s):
    """The motion left over one detection time is under the tip's own resolution."""
    B = ex.bounds
    v = float(np.linalg.norm(ex.model.jac(s.q)[:3] @ s.qd))
    return v * (_num(B.t_det()) or 0.0) <= (_num(B.rigid(s.q, s.R[:, 0])) or 0.0)


def quiet(ex, s, need_m):
    """A wait's test: the motion left over one detection time under need_m, and the force at rest less the held
    load at or under the level. While a load is not read yet, or the hold is on a part of something fixed, the
    reading at rest is the hold's own and only the motion is judged: a wait always has an exit by motion alone."""
    v = float(np.linalg.norm(ex.model.jac(s.q)[:3] @ s.qd))
    return v * (_num(ex.bounds.t_det()) or 0.0) <= float(need_m) and rest_force(ex, s) <= level(ex)


def note_contact(ex, contact):
    """A contact made while holding goes on the hold (the held thing met something), never into the world as a
    thing in the way: its place is kept under held_point_mm and the world entry is not made. A push the reading
    locates off the tool (source "arm or self": a link met something) is not the hold's: it is left as it is."""
    rec = held(ex)
    if rec is None or not isinstance(contact, dict) or contact.get("source") not in (None, "tool"):
        return
    met = {k: contact.get(k) for k in ("point_mm", "force_n", "normal_est", "t_s", "source")}
    met["call"] = int(ex._calls)
    rec["contacts"] = (rec.get("contacts") or [])[-int(R.QUANT):] + [met]
    contact["held_point_mm"], contact["point_mm"] = contact.get("point_mm"), None
    contact["on"] = "the held thing or the tool that holds it: kept on the hold, not stored as a thing in the way"


def _lost(ex, rec, s):
    """The hold is lost when the jaws close past the stall gap by more than their still band plus REL of the
    command's lead (what the jaw servo itself gives while the squeeze shifts) and come to their command, or meet
    each other: nothing is left between them. It ends the record and says so. Jaws that closed past the stall
    gap and stand again short of their command still press on the thing: it shifted in the jaws. The record then
    follows them (frac, gap_m) and counts how far (shifted_m); while they move nothing is judged."""
    frac, _ = read(ex, s)
    if frac is None:
        return None
    lead = max(0.0, rec["frac"] - rec["cmd_frac"])
    band = rec["still_frac"] + R.REL * lead
    if frac >= rec["frac"] - band:
        rec.pop("slipping", None)
        return None
    N = numbers(ex)
    per = float(N["gap_per_span_m"])
    if frac > max(float(rec["cmd_frac"]), float(N["closed_gap"])) + band:
        at = rec.get("slipping")
        if at is None or abs(frac - at[0]) > rec["still_frac"]:
            rec["slipping"] = [frac, 1]          # the jaws move: not judged yet
            return None
        at[1] += 1
        if at[1] < int(R.QUANT) + int(math.ceil(float(N["lag"][1]) / float(ex.dt))):
            return None
        moved = (rec["frac"] - frac) * per       # they stand again, still open of their command: it shifted
        J = facts(ex)["J"]
        rec.update(frac=float(frac), gap_m=float(J.width_m(J.q_of_frac(frac))),
                   shifted_m=float(rec.get("shifted_m") or 0.0) + moved)
        rec.pop("slipping", None)
        ex._journal("hold_shifted", closed_mm=round(moved * MM, 2), gap_mm=round(rec["gap_m"] * MM, 2))
        return None
    closed = (rec["frac"] - frac) * per * MM
    _end(ex, "slipped", closed_mm=round(closed, 2))
    limit = dict.get(rec, "limit_n")
    return (f"hold lost: the jaws closed {closed:.1f} mm past where they held it"
            + ("" if dict.get(rec, "squeeze_n") is None else f" (it slipped at {rec['squeeze_n']:.1f} N of squeeze)")
            + ("" if limit is None else f"; the close wound its squeeze no further than the stated limit of "
                                        f"{float(limit):g} N ({dict.get(rec, 'limit_source')}) on the thing allows"))


def _last_stretch(ex):
    """The length (m) of the last straight stretch the tip ran: back from the end of the newest executed path
    while each step keeps the last step's direction within REL."""
    hist = getattr(ex, "_history", None)
    if not hist:
        return 0.0
    tips = [ex.model.fk_tip(np.asarray(q, dtype=float))[0] for q in hist[-1]]
    total, u = 0.0, None
    for i in range(len(tips) - 1, 0, -1):
        step = tips[i] - tips[i - 1]
        n = float(np.linalg.norm(step))
        if n <= 0.0:
            continue
        if u is None:
            u = step / n
        elif float(step @ u) < (1.0 - R.REL) * n:
            break
        total += n
    return total


def _leg_in(ex):
    """The length of the last leg before a close: what the core's own leg ends told (after_move); else, where the
    last call that moved ended and nothing has run since (an operator's move, then its close), the line of the
    path that call ran; else (a tool that comes in and closes in one call) the last straight stretch of the
    newest executed path."""
    st = ex.__dict__.get("_hold_legs") or {}
    if st.get("leg_m") is not None:
        hist = getattr(ex, "_history", None)
        if st.get("by_leg") or (hist[-1] if hist else None) is st.get("path"):
            return float(st["leg_m"])
    return _last_stretch(ex)


def brief(ex):
    """What a result says of the hold: None with nothing held and nothing just lost."""
    rec = held(ex)
    if rec is None:
        note = ended(ex)
        if note is None or note.get("at_call") != ex._calls:
            return None
        return {"held": False, "ended": note["why"], **{k: v for k, v in note.items() if k in ("closed_mm", "said")}}
    w = dict.get(rec, "weight_n")
    out = {"held": True, "gap_mm": round(rec["gap_m"] * MM, 2), "kind": dict.get(rec, "kind"),
           "weight_n": None if w is None else round(w, 3), "level_n": round(float(rec["level_n"]), 3),
           "level_source": rec["level_source"]}
    if dict.get(rec, "said") is not None:
        out["said"] = rec["said"]
    if rec.get("shifted_m"):
        out["shifted_mm"] = round(float(rec["shifted_m"]) * MM, 2)     # the thing shifted in the jaws this far
    if rec.get("read_call") == ex._calls:
        out["rest_read_periods"] = int(rec.get("read_periods") or 0)
    if rec.get("contacts") and rec["contacts"][-1].get("call") == ex._calls:
        out["met"] = rec["contacts"][-1]
    return out


# ---------------------------------------------------------------------------------------------- the load
def read_load(ex, stop=None, whole=False):
    """At a rest that ends a held move: wait by motion alone (the motion left over one detection time under the
    tip's own resolution, QUANT readings in a row), then read the load from the RAW estimate (before anything is
    taken off it) over the settle window, and judge it.
    Not told: the hand stands further from where it was sent than REL of how far it came since the load was last
    judged, and further than the arm itself gives under that load (a move too short to tell the load from the
    arm's own lag, or a hand that is held back): nothing is named and the hold keeps what it had.
    Free: the load stands along gravity; weight_n is that part, mass_kg the weight over g, sigma_n its spread. A
    hold that was on a part of something fixed has come free; a free hold's weight follows the reading (down as
    well as up) and the receipt says when it changed.
    Attached (only a hold not yet known free): the move ended on the cap, or the load does not stand along
    gravity by more than the hold's own reading at rest can explain (a part of something fixed, or a thing too
    heavy); the receipt says what it resists with. A free
    hold whose rest reading stands across gravity keeps its weight and says so (something presses on it).
    A free hold whose weight was read over what the still arm tells and whose load along gravity is now at or under
    it (the thing stands on what it met, or the hand presses it onto that) says "the jaws close on it; its weight
    rests on what it met": in_hand False. whole (a read after a stop on a contact of the held thing): the weight
    stands in the hand only where all of the weight read before stands there (less what the still arm tells); a
    part of it gone is a thing that rests in part on what it met: in_hand False, said so.
    Returns {weight_n, sigma_n, kind, periods, said, told, in_hand (True: the weight stands in the hand; False: it
    does not; None: this rest cannot tell)}; periods are the control periods the read itself used."""
    rec = held(ex)
    if rec is None:
        return {"weight_n": None, "sigma_n": None, "kind": None, "periods": 0, "said": "nothing is held",
                "told": False, "in_hand": False}
    B, dt = ex.bounds, float(ex.dt)
    s = ex._last if ex._last is not None else ex._sense()
    window = max(int(R.QUANT), int(math.ceil((_num(B.t_settle()) or 0.0) / dt)))
    was, ex._period_kind = ex._period_kind, "waiting"
    k, calm = 0, 0
    try:
        for _ in range(int(ex._wait_limit())):
            calm = calm + 1 if _still(ex, s) else 0
            if calm >= int(R.QUANT):
                break
            s = ex._period(ex._q_des)
            k += 1
        raws = [np.asarray(s.raw, dtype=float)]
        for _ in range(window - 1):
            s = ex._period(ex._q_des)
            k += 1
            raws.append(np.asarray(s.raw, dtype=float))
    finally:
        ex._period_kind = was
    g_vec = np.asarray(ex.model.mj.opt.gravity, dtype=float)
    g = float(np.linalg.norm(g_vec))
    up = -g_vec / g if g > 0.0 else np.array([0.0, 0.0, 1.0])
    F = np.mean(raws, axis=0)
    size = float(np.linalg.norm(F))
    w = float(F @ up)
    side = float(np.linalg.norm(F - w * up))
    sigma = float(np.std(np.asarray(raws) @ up))
    thr = _num(B.f_thr(0.0, 0.0)) or 0.0
    sent = ex.model.fk_tip(np.asarray(ex._q_des, dtype=float))[0]
    behind = float(np.linalg.norm(sent - s.tip))
    asked = 0.0 if rec.get("judged_at") is None else float(np.linalg.norm(sent - rec["judged_at"]))
    # it followed: it stands within REL of how far it came of where it was sent, or within what the arm itself
    # gives under that load
    followed = behind <= max(R.REL * asked, (1.0 + R.REL) * (_num(B.rigid(s.q, up, max(size, thr))) or 0.0))
    capped = stop == "cap"
    kind0, w0 = dict.get(rec, "kind"), dict.get(rec, "weight_n")
    rec.update(read_periods=k, read_call=int(ex._calls))
    if not followed and not capped:
        said = (f"the load is not read at this rest: the hand stands {behind * MM:.1f} mm from where it was sent "
                f"after {asked * MM:.1f} mm of travel (too short a move to tell the load from the arm's own lag, "
                "or the thing holds the hand back)")
        if kind0 is None:
            rec["said"] = said
        ex._journal("hold_load", load_kind=kind0, told=False, behind_mm=round(behind * MM, 2), periods=k)
        return {"weight_n": w0, "sigma_n": sigma, "kind": kind0, "periods": k, "said": said, "told": False,
                "in_hand": None}
    # what tells a load across gravity from the hold itself: the free threshold and the size of the rest reading
    # the close left (a hold shifts the reading by about that much: a smaller reading across gravity is its own)
    own = max(thr, float(np.linalg.norm(rec["rest_n"])))
    along = not capped and w >= -own and side <= max(own, R.REL * abs(w), R.TWO_READ * sigma)
    tell = R.TWO_READ * sigma + (_num(B.f_hidden(0.0, 0.0)) or 0.0)    # what the still arm's reading resolves
    # in_hand: whether the thing's weight stands in the hand at this rest (True), rests on something else (False:
    # a free hold whose weight was read over what the still arm tells and reads none now), or cannot be told by
    # this arm (None: its weight was never read over what the still arm tells)
    weighed = kind0 == "free" and float(rec.get("weight_before", w0) or 0.0) > tell
    w_was = float(rec.get("weight_before", w0) or 0.0)
    if along:
        weight = max(w, 0.0)
        if weight <= tell and not presses(ex, s):
            # no load stands and the jaws' own reading NOW shows no press (a close the limit or the cap ended left
            # its command on the reading): no reading of the robot's own shows a hold; the gap alone is none
            said = (f"not holding it: the hand rose and reads no weight ({weight:.2f} N, where the still arm tells "
                    f"{tell:.2f} N) and the jaws do not press: the thing stayed")
            _end(ex, "stayed", said=said)
            return {"weight_n": weight, "sigma_n": sigma, "kind": None, "periods": k, "said": said, "told": True,
                    "in_hand": False}
        in_hand = True if weight > tell else (False if weighed else None)
        part = bool(whole and weighed and in_hand and weight < w_was - tell)    # a part of its weight is gone
        in_hand = False if part else in_hand
        said = (f"the jaws close on it; its weight rests in part on what it met: {weight:.2f} N of the "
                f"{w_was:.2f} N it weighed stands in the hand at rest (the still arm tells {tell:.2f} N)" if part else
                f"holding it: {weight:.2f} N read at rest" if weight > tell else
                f"the jaws close on it; its weight rests on what it met: {weight:.2f} N read at rest (the still arm "
                f"tells {tell:.2f} N) where it weighed {w_was:.2f} N" if weighed else
                f"holding it by the jaws' own reading (they stand open of their command: they press on something "
                f"between them): no weight is read at rest ({weight:.2f} N, where the still arm tells {tell:.2f} N)")
        if kind0 == "attached":
            said += f"; it came free (it resisted with {float(dict.get(rec, 'shown_n') or 0.0):.2f} N before)"
        elif kind0 == "free" and in_hand and abs(weight - w_was) > tell:
            said += f"; the weight read changed: {w_was:.2f} N before"
        rec.update(kind="free", weight_n=weight, mass_kg=weight / g if g > 0.0 else 0.0)
        for key in ("shown_n", "shown_hard_n", "resists_n"):
            rec.pop(key, None)
    elif kind0 == "free":
        # the load does not stand along gravity: something presses on the held thing. Its weight still stands in
        # the hand only where the part along gravity is read over what the still arm tells; a hand that presses
        # the thing onto what it met reads that part at or under it (the reaction from below takes it)
        in_hand = True if w > tell else (False if weighed else None)
        if whole and weighed and in_hand and w < w_was - tell:
            in_hand = False                # a part of its weight is gone: it rests in part on what it met
        said = (f"holding it ({float(w0):.2f} N read before); at this rest {side:.2f} N stands across gravity: "
                "something presses on the held thing" if in_hand or not weighed else
                f"the jaws close on it; its weight rests on what it met: {w:.2f} N along gravity at rest (the still "
                f"arm tells {tell:.2f} N) where it weighed {w_was:.2f} N, and {side:.2f} N across gravity")
    else:
        in_hand = False
        why = "the move ended on the cap" if capped else "the load does not stand along gravity"
        said = (f"it did not come free: it resists with {size:.2f} N ({why}); it is a part of something fixed, "
                "or too heavy")
        rec.update(kind="attached", weight_n=0.0, mass_kg=0.0, resists_n=[float(x) for x in F],
                   shown_n=max(float(dict.get(rec, "shown_n") or 0.0), float(rec.get("load_seen") or 0.0), size),
                   shown_hard_n=max(float(dict.get(rec, "shown_hard_n") or 0.0),
                                    float(rec.get("load_hard") or 0.0), size))
    rec.update(sigma_n=sigma, read_at={"call": int(ex._calls), "t_s": float(s.t)}, said=said,
               judged_at=np.asarray(s.tip, dtype=float).copy())
    rec.pop("reread", None)
    rec.pop("weight_before", None)
    _relevel(ex, rec)
    resense = getattr(ex, "_resense", None)
    if resense is not None:        # the last reading again, now with the load off it
        resense()
    ex._journal("hold_load", load_kind=rec["kind"], told=True, weight_n=round(float(rec["weight_n"]), 4), periods=k)
    return {"weight_n": rec["weight_n"], "sigma_n": sigma, "kind": rec["kind"], "periods": k, "said": said,
            "told": True, "in_hand": in_hand}


def after_move(ex, s, stop, leg=None, own=True):
    """The end of a leg (the core calls it at every leg boundary and at the end of a run; leg None: the end of a
    run). It keeps the leg's length for the next close, ends a hold that was lost, and reads the load at the end
    of a run that ended at its goal: the first after a close (a run that stopped reads nothing: the thing may
    not be free of what it stood on; what the run showed is kept as the load seen so far), every one while the
    hold is on a part of something fixed (what it resists with at that rest is what it has shown), and one
    whose leg began on a standing reading.
    own False: the call's own end stands in for the core's leg ends (compat shim, control_base.primitive)."""
    st = ex.__dict__.setdefault("_hold_legs", {})
    st["seen_call"] = int(ex._calls)
    if s is None:
        return
    hist = getattr(ex, "_history", None)
    newest = hist[-1] if hist else None
    if own:
        st["by_leg"] = True
    if not own and newest is not None and newest is not st.get("path") and len(newest) > 1:
        # a call's own end: the leg is the line of the newest path this call ran, from its start to its end
        st["leg_m"] = float(np.linalg.norm(ex.model.fk_tip(np.asarray(newest[-1], dtype=float))[0]
                                           - ex.model.fk_tip(np.asarray(newest[0], dtype=float))[0]))
    elif st.get("from") is not None:
        st["leg_m"] = float(np.linalg.norm(s.tip - st["from"]))
    st["from"] = s.tip.copy()
    st["path"] = newest                            # the newest path at this leg's end (see _leg_in)
    rec = held(ex)
    if rec is None or _lost(ex, rec, s):
        return
    # what the guards of this leg said (a stop the core resumed is forgotten by the next guard) beside the
    # word the caller gives
    said_stop = rec.get("leg_stop")
    goal = stop in (None, "goal", "limit") and said_stop is None     # limit: the plan itself ended short, at rest
    capped = "cap" in (stop, said_stop)
    peak, hard = rec.pop("leg_peak", None), rec.pop("leg_hard", None)
    if dict.get(rec, "weight_n") is None:
        if not rec.get("first_ran"):
            return
        if not goal and not capped:
            if peak is not None:
                rec["load_seen"] = max(float(rec.get("load_seen") or 0.0), float(peak))
                rec["load_hard"] = max(float(rec.get("load_hard") or 0.0), float(hard or peak))
            return
        if leg is None and rec.get("read_call") != ex._calls:
            read_load(ex, "cap" if capped else stop)
        return
    if dict.get(rec, "kind") == "attached":
        if leg is None and goal and rec.get("read_call") != ex._calls:
            read_load(ex, stop)            # at every rest: it may have come free
        return
    if leg is None and goal and rec.get("reread") and rec.get("read_call") != ex._calls:
        read_load(ex, stop)                # the leg began on a standing reading: the weight is read again


def call_end(ex, name, out, moved):
    """The end of an operator's call (control_base.primitive): stands in for the core's own leg ends where the
    motion files do not call after_move yet (compat shim), and puts the hold's own lines into the result."""
    st = ex.__dict__.get("_hold_legs") or {}
    if moved and st.get("seen_call") != ex._calls:        # compat shim
        after_move(ex, ex._last, out.get("stopped") if isinstance(out, dict) else None, own=False)
    if not isinstance(out, dict):
        return out
    block = brief(ex)
    if block is not None and "hold" not in out:
        out["hold"] = block
    notes = ex.__dict__.pop("_guard_notes", None)
    if notes:
        out.setdefault("bounds_notes", [])
        out["bounds_notes"] += [n for n in notes if n not in out["bounds_notes"]]
    return out


# ---------------------------------------------------------------------------------------------- the guard
def arm_cap(ex, spec):
    """The cap a guard runs with (N): what the arm itself can push along its travel (its joints' torques less
    their measured error) and a tool rating its maker states; a jaw motor's force range is no cap. A spec that
    names its own source already carries that cap (compat shim: an older spec's cap held the jaw motor's force)."""
    cap = getattr(spec, "cap_n", None)
    if getattr(spec, "source", None) is not None:
        return math.inf if cap is None else float(cap)
    try:
        B = ex.bounds
        q, u = B._pose(getattr(ex, "_q_des", None), getattr(ex, "_travel_direction", None))
        own = _num(B.f_cap(q, u))
        own = math.inf if own is None else own * (1.0 - (_num(B.e_tau()) or 0.0))
    except Exception:          # bounds that cannot say: the spec's own
        return math.inf if cap is None else float(cap)
    rating = ex.model.spec.get("tool_rating_n")
    return own if rating is None else min(own, float(rating))


class Watch:
    """What a guard does while a record stands (made by control_base.Guard; None with nothing held)."""

    def __init__(self, guard, stated_by_caller):
        ex = guard.ex
        rec = held(ex)
        self.g, self.ex, self.rec = guard, ex, rec
        s = ex._last
        free, one = guard.force_n, guard.one_n
        self.gap = 0.0 if free is None or one is None else max(0.0, one - free)
        self.free = float(free) if free is not None else (_num(ex.bounds.f_thr(0.0, 0.0)) or 0.0)
        self.p0 = None if s is None else s.tip.copy()
        self.seen = 0                      # readings judged (a known hold looks at the first: judged)
        self.after_stop = rec.get("leg_stop") is not None      # the leg before this one ended on a stop
        guard.direction = None             # the direction rule is for leaving a touched thing with empty jaws
        rec["leg_stop"] = None             # what this guard's leg ends on (stopped) is kept for the leg's end
        limit = dict.get(rec, "limit_n")
        if limit is not None:
            guard.cap_n = min(guard.cap_n, float(limit))
        if stated_by_caller and free is None:
            self.mode = "cap"              # a stroke that presses on purpose: the cap and the lost hold
        elif dict.get(rec, "weight_n") is None and not rec.get("first_ran"):
            self.mode = "first"            # one leg, no longer than the last leg in: no threshold on the tool
            rec["first_ran"] = True
            guard.force_n = guard.one_n = None
        elif dict.get(rec, "weight_n") is None:
            self._soon()
        elif rec.get("kind") == "attached":
            # a part of something fixed is worked: its own resistance is no contact up to DOUBLE what it showed
            # so far (the reading over one period, and the hardest reading inside a period); the cap stays
            self.mode = "attached"
            base = max(self.free, float(rec["level_n"]))
            guard.force_n = base
            guard.one_n = max(base, R.DOUBLE * float(dict.get(rec, "shown_hard_n") or 0.0)) + self.gap
        else:
            self.mode = "known"
            self._set(max(self.free, float(rec["level_n"])))

    def _set(self, force_n):
        self.g.force_n, self.g.one_n = float(force_n), float(force_n) + self.gap

    def _soon(self):
        """After the first leg's travel and before the load is read: a level made of the size of the rest reading
        and the load seen so far (the hardest reading inside a period for the one reading threshold)."""
        rec = self.rec
        self.mode = "soon"
        base = max(self.free, float(np.linalg.norm(rec["rest_n"])))
        self.g.force_n = base + float(rec.get("load_seen") or 0.0)
        self.g.one_n = base + max(float(rec.get("load_seen") or 0.0), float(rec.get("load_hard") or 0.0)) + self.gap

    @property
    def first(self):
        return self.mode == "first"

    def see(self, s):
        """One reading: the text that ends the move when the hold is lost, else None."""
        rec, ex = self.rec, self.ex
        if rec is None or ex.__dict__.get("_hold") is not rec:
            return None
        lost = _lost(ex, rec, s)
        if lost:
            return lost
        f = float(s.Fmag)
        hard = max(f, float(getattr(s, "Fend", None) or 0.0))
        if self.mode == "first":
            rec["load_seen"] = max(float(rec.get("load_seen") or 0.0), f)
            rec["load_hard"] = max(float(rec.get("load_hard") or 0.0), hard)
            if self.p0 is not None and float(np.linalg.norm(s.tip - self.p0)) > float(rec["leg_in_m"]):
                self._soon()
        elif self.mode in ("soon", "known", "attached"):
            rec["leg_peak"] = max(float(rec.get("leg_peak") or 0.0), f)
            rec["leg_hard"] = max(float(rec.get("leg_hard") or 0.0), hard)
        return None

    def judged(self, s, size, hard):
        """(size, hardest) of what the guard judges at reading s. A known free hold whose leg begins on a
        reading over the free threshold that stands along gravity (it stood before the leg made anything, and
        no stop went before it) carries a load that is not what was read: the weight is taken from that first
        reading as it stands (the raw estimate along gravity), that reading is not judged, and the load is read
        again properly at the run's end. A reading that stands across gravity, or one a stop left, is a
        contact as before."""
        if self.mode != "known":
            return size, hard
        self.seen += 1
        if self.seen == 1 and size > self.free and not self.after_stop:
            rec, F = self.rec, np.asarray(s.F, dtype=float)
            g_vec = np.asarray(self.ex.model.mj.opt.gravity, dtype=float)
            g = float(np.linalg.norm(g_vec))
            up = -g_vec / g if g > 0.0 else np.array([0.0, 0.0, 1.0])
            w = float(F @ up)
            if float(np.linalg.norm(F - w * up)) <= max(self.free, R.REL * abs(w)):
                now = max(0.0, float(np.asarray(s.raw, dtype=float) @ up))
                rec.setdefault("weight_before", float(dict.get(rec, "weight_n") or 0.0))
                rec.update(weight_n=now, mass_kg=now / g if g > 0.0 else 0.0, reread=True)
                _relevel(self.ex, rec)
                self._set(max(self.free, float(rec["level_n"])))
                return 0.0, 0.0
        return size, hard

    def stopped(self, stop):
        """The guard ended the leg with this word: kept for the leg's end (control_hold.after_move)."""
        rec = self.rec
        if rec is not None and self.ex.__dict__.get("_hold") is rec:
            rec["leg_stop"] = str(stop)


# ---------------------------------------------------------------------------------------------- close and release
def close(ex, **how):
    """The one close (rcpx/control_hold_close.py close: its arguments and result)."""
    from rcpx import control_hold_close
    return control_hold_close.close(ex, **how)


def release(ex, to_frac=None):
    """Open the jaws and end the hold (rcpx/control_hold_close.py release)."""
    from rcpx import control_hold_close
    return control_hold_close.release(ex, to_frac)
