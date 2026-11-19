# SPDX-License-Identifier: AGPL-3.0-only
"""Taking hold and letting go of package fixture: phases 2 and 5 of work_part (rcpx/r2/fixture.py).

- _fx_limit: the limit somebody stated on a contact with the part. A limit stated on the call binds every
  leg of it, free or expected: every stream of a call carries the call's own intent, the stated
  force in it (a build that took it out of the legs through free space struck a drawer on a free leg with
  261 N under max_n 20).
- _fx_close: the core's one close where the hand stands, the hold on a part of something fixed; under a stated
  limit each pad presses its share of it.
- routes_in: the ways from where the arm stands to the pose before the part, the best first: clear of every
  measured band and of what the newest look shows (rcpx/r2/fixture_look.py under), a hand that pushes with a
  straight run in; where none is clear, the straight joint way.
- _fx_take: ONE stream to the pose before the part and on to the hold pose, its last leg an expected contact
  with whatever lies ahead (a free line, no ending force); the jaws close where it stopped. Where it stopped
  short of the pose and the jaws tell nothing between them, they open again and the hand goes on to the pose
  under the guard from where it stands, once. What the way in met is named against the part's own region.
- _fx_let_go: the release and the way back to the pose before the part: a leave of the contact the hand stood in
 where this call's own records show the hand on the part (_fx_stand writes the core's stand record,
  only then, with the reading as it is); a sheet, max_n or gentle stated for the part stands over it.
  Where the bounds refuse the way back to that pose, the leave alone runs (the lift along the hand's own axis);
  where they refuse that too, the hand stands where it is. What ran is what the phase and the receipt say ("did",
  _fx["out_said"]), never "backed off" for a way out that did not move. The phase states a figure for the push
  the hand stood with as it began to leave and for the jaws' opening (fixture_strike stand, release).
What each of them read is held against a stated limit for the receipt (fixture_say._fx_seen). Nothing here reads
truth, names a robot or a device, or holds a typed limit.
"""

import math

import numpy as np

from rcpx import bounds_laws as laws
from rcpx import bounds_report as report
from rcpx import control_hold
from rcpx.bounds_ratios import BAND, QUANT
from rcpx.r2 import fixture_follow as F
from rcpx.r2 import fixture_hold as H
from rcpx.r2 import fixture_strike as S


def _pose(R, p):
    T = np.eye(len(p) + 1)
    T[:len(p), :len(p)], T[:len(p), len(p)] = R, p
    return T


def _mm(p):
    return [round(float(x) * report.MM, 1) for x in p]


def _refused_text(got):
    """The check and the why of a stream the bounds refused, in a few words."""
    bad = got.get("refused")
    bad = bad.get("refused", bad) if isinstance(bad, dict) else bad
    if not isinstance(bad, dict):
        return str(bad or "refused")
    return f"{bad.get('check')}: {str(bad.get('why') or '')}".rstrip(": ")


# ---------------------------------------------------------------------------------------------- the ways in
def routes_in(ex, choice):
    """The ways from where the arm stands to the standoff pose before the part, the best first: [(legs, note)],
    note None for a way that is clear of every measured band and of what the newest look shows, else the words
    for how it goes. First, for a hand that pushes and where the arm holds a pose one more standoff back on the
    last leg's own line (choice q_back): a clear way there and a straight run in (the corner a stream rounds then
    lies a standoff before the last leg, and the last leg meets the part along its approach: a corner rounded AT
    the standoff brought the fingers down onto a part's top edge). Then a clear way to the standoff pose. Only
    where neither is clear: the straight joint way, under the guard, and said. The caller takes the first whose
    plan the bounds do not refuse; a pose whose clear ways the bounds all refuse gives way to the next ranked
    pose."""
    out = []
    if choice.get("q_back") is not None:
        legs = _route_to(ex, choice, choice["q_back"], choice["p_back"])
        if legs is not None:
            pose = np.eye(len(choice["p_pre"]) + 1)
            pose[:-1, :-1], pose[:-1, -1] = choice["R"], choice["p_pre"]
            out.append((legs + [{"kind": "transit", "to": pose, "line": True}], None))
    legs = _route_to(ex, choice, choice["q_pre"], choice["p_pre"])
    if legs is not None:
        out.append((legs, None))
    if not out:
        out.append(([{"kind": "transit", "to": choice["q_pre"]}], "went the straight way, the contact stop armed"))
    return out


def _route_to(ex, choice, q_end, p_end):
    """The legs from where the arm stands to the pose (q_end, tip at p_end), or None when no way is clear: a
    joint move where its own path keeps every body out of every band and from under what the newest look shows
    (fixture_look.under); else a joint move to a pose above it (as high as the tip stands now, then half of
    that) and a straight line down."""
    m, B = ex.model, H.bounds_of(ex)
    s = ex._sense()
    up = H.up_of(ex)
    support = B.space.support()
    spacing = max(B.spacing(laws.band(support[1] or 0)) if support is not None else 0.0, B.robot.w_tool())

    def first_hit(q0, q1):
        """The first (body, thing or look, clearance) on the joint path from q0 to q1, or None: the path is clear."""
        span = float(np.linalg.norm(m.fk_tip(q1)[0] - m.fk_tip(q0)[0]))
        n = max(1, int(math.ceil(span / spacing))) if spacing > 0 else 1
        for i in range(1, n + 1):
            q = q0 + (q1 - q0) * (i / n)
            hits = H.blockers(ex, q)[0] or H.L.under(ex, q)
            if hits:
                return hits[0]
        return None

    notes = choice.setdefault("route_notes", [])
    hit = first_hit(s.q, q_end)
    if hit is None:
        return [{"kind": "transit", "to": q_end}]
    notes.append(("direct", hit[:QUANT]))
    rise = max(0.0, float((s.tip - p_end) @ up))
    pose = np.eye(len(up) + 1)
    pose[:len(up), :len(up)], pose[:len(up), len(up)] = choice["R"], p_end
    for share in (1.0, H.HALF):
        if not rise * share > spacing:
            break
        p_via = p_end + rise * share * up
        q_via, ok, _ = m.ik(q_end, p_via, R_target=choice["R"], restarts=int(QUANT))
        hit = first_hit(s.q, q_via) if ok and m.within_limits(q_via) else None
        if not ok or not m.within_limits(q_via) or hit is not None:
            notes.append(("via", share, bool(ok), bool(m.within_limits(q_via)), None if hit is None else hit[:QUANT]))
            continue
        down = H.walk(ex, q_via, H._line(p_via, choice["R"], -up), rise * share, None, spacing, seen=True)
        notes.append(("down", share, round(down["clear"], int(BAND)), round(down["reach"], int(BAND)),
                      None if down["block"] is None else down["block"][:QUANT]))
        if down["clear"] >= rise * share - spacing:
            return [{"kind": "transit", "to": q_via}, {"kind": "transit", "to": pose, "line": True}]
    return None


class TakeMixin:
    def _fx_limit(self, part, intent):
        """The limit somebody stated on a contact with this part, or limit_n None."""
        ref = part["owner"] if part["owner"] is not None else part["ref"]
        return H.bounds_of(self).stated_limit(ref if isinstance(ref, str) else None, intent or None, expected=True)

    def _fx_close(self, part, choice, limit):
        """The jaws close where the hand stands, through the core's one close, the hold on a part of something
        fixed. Under a stated limit each pad may press its share of it (the part is pressed from every pad at
        once and sees their sum); what the close then leaves on the part is held against the limit in the
        receipt. Returns (the close's result or its refusal, the evidence block or None)."""
        J = self._fx_jaws
        bound, pads = limit.get("limit_n"), len(J.sides)
        s = self._sense()
        short = float((choice["p_hold"] - s.tip) @ choice["a"])
        seen_w = None if part["width_m"] is None else part["width_m"] + BAND * part["sigma_m"]
        c = control_hold.close(self, expect_m=seen_w, band_m=None if seen_w is None else BAND * part["sigma_m"],
                               limit_n=None if bound is None else bound / pads,
                               limit_source=limit.get("source") if bound is not None else None, kind="attached")
        if "refused" in c:
            return c, None
        S.note_close(self, c, pads)                      # what the close states is the phase's own
        if bound is not None:                            # the close's own arrival under the limit is not priced here
            self._fx.setdefault("cost", []).append({"where": "the close of the jaws", "periods": None, "ran": c.get("periods")})
        sq = c.get("squeeze_n")
        if c.get("held") and sq is not None:
            self._fx_seen("the pads' squeeze", pads * float(sq), f"the {pads} pads press the part with "
                          f"{float(sq):.1f} N each, {pads * float(sq):.1f} N together")
        if c.get("kept") is not True and bound is not None:
            self._fx_seen("the close", None, "the close: " + str(c.get("price") or "").split(";")[0], shown=False)
        return c, {"held": bool(c.get("held")), "ended": c.get("ended"), "price": c.get("price"),
                   "stalled_mm": None if c.get("gap_m") is None else c["gap_m"] * report.MM,
                   "short_of_pose_mm": round(short * report.MM, 1)}

    def _fx_take(self, part, choice, intent):
        """The take_hold phase for one choice: the jaws go to the way-in opening while the hand moves; ONE stream to the pose
        before the part and on to the hold pose, its last leg an expected contact with the part that only a
        stated limit bounds; the jaws close where it stopped, through the core, the hold on a part of
        something fixed. Where the last leg stopped short of the pose and the jaws tell nothing between them
        there, they open again and the hand goes on to the pose under the guard, once, before the pose is given
        up. The way to the pose before the part is the first of routes_in whose plan the bounds do
        not refuse for reach or a link at the support: where none is clear of every measured band and of what
        the newest look shows, the straight way is taken under the guard, and said."""
        J, B = self._fx_jaws, H.bounds_of(self)
        if J is not None and control_hold.held(self) is None:
            self._gripper_cmd = float(choice["open"])                            # they open on the way
        ref = part["owner"] if part["owner"] is not None else part["ref"]
        ref = ref if isinstance(ref, str) else None
        goal, push = choice["p_hold"], choice["name"] in ("fingers", "tip")
        if push:                                                                 # a push ends IN contact
            goal = goal + B.travel("touch", part["thing"], choice["q_hold"], choice["a"]).past_m * choice["a"]
        routes = routes_in(self, choice)
        H.restore_jaws(self)
        limit = self._fx_limit(part, intent)
        bound = limit.get("limit_n")
        # what this call states as a limit, with the part it resolved: a move that later leaves the contact this
        # take makes keeps that limit until the contact has ended (the core reads it: control_tail.stand_limit).
        # What shows the hand ON the part is noted as the take finds it (_fx["bore"])
        self._fx["bore"] = None
        self._meant = {"call": getattr(self, "_calls", None), "thing": ref,
                       "words": {k: v for k, v in (intent or {}).items() if k in ("max_n", "gentle") and v}}
        if bound is not None and not self._fx.get("limit"):
            self._fx["limit"] = {"limit_n": float(bound), "source": limit.get("source"), "read": {}, "passed": [],
                                 "not_shown": []}
        # the last leg in is a free line to its pose (no ending force, no slow arrival; the guard ends
        # it and only a stated limit bounds it). Its expected contact is whatever lies ahead on the leg: until a look
        # shows the part's own extent, a contact on the part's own body some way from the pointed point is the
        # part, not a thing in the way (stored as one, it ranked away the pose that takes the part: measured on
        # the own bench, an open drawer was no longer pulled). The part's region (its point with its error, half
        # of what a look shows of it, how far from the tip this hand can bear on a thing) only NAMES what was met
        hand = float((J.finger_len_m or J.max_m) if J is not None else B.robot.w_tool())
        half = max(float(part.get("width_m") or 0.0), float(part.get("wide_m") or 0.0)) / laws.TWO
        region = BAND * float(part["sigma_m"] or 0.0) + half + float(B.robot.w_tool()) + hand
        expect = {"with": None, "radius_m": float(part["sigma_m"] or 0.0) + float(B.robot.w_tool()),
                  "along": np.asarray(choice["a"], dtype=float), "bound_n": bound,
                  "bound_source": limit.get("source") if bound is not None else None, "seek": True}
        last = {"kind": "approach" if choice["closes"] else "touch", "to": _pose(choice["R"], goal), "line": True,
                "target": ref, "expect": expect}
        word = dict(intent or {})       # a limit stated on the call binds every leg of it, free or expected
        for n, (route, note) in enumerate(routes):       # the first way whose plan the bounds do not refuse
            got = F.stream(self, route + [last], word)
            bad = got.get("refused") if got["stopped"] == "refused" else None
            bad = bad.get("refused", bad).get("check") if isinstance(bad, dict) else None
            if bad not in ("reach", "path_plane"):
                break
        self._journal("fixture", stage="route", legs=len(route), note=note, ways=len(routes), taken=n,
                      notes=choice.get("route_notes"))
        said = "; ".join(t for t in (H.passes_text(choice.get("passes")), H.swept_text(choice.get("swept"))) if t) or None
        if note:
            hits = H.blockers(self, choice["q_pre"])[0]
            said = (said or (H.passes_text(hits[0]) if hits else "no way to the pose before the part is clear of "
                             "every measured thing and of what the newest look shows")) + "; " + note
        self._journal("fixture", stage="way_in", **{k: v for k, v in got.items() if k not in ("refused", "strike")})
        out = {"stream": got, "held": None, "said": said, "limit": limit}
        if got["stopped"] == "refused":
            return dict(out, ok=False, why=got["refused"])

        def judge(got, n_before):
            """(met: the last leg ended on a contact with the tool, where: text when it ended anywhere else). What
            the leg met is noted for the receipt whatever stopped it (fixture_say.met: every sentence of the
            receipt is built from the call's own stop records)."""
            on_last = got.get("leg") in (None, n_before)
            on_tool = "not on the tool" not in str(got.get("why") or "")
            met = got["stopped"] in ("arrived", "contact", "cap") and on_last and on_tool
            n = self._fx_met("the way in", got, short_m=float((choice["p_hold"] - self._sense().tip) @ choice["a"]),
                             part_p=part["p"], region_m=region)
            if met:
                ran = (got.get("strike") or {}).get("ran") or {}
                self._fx_seen("the way in", n, None if n is None else f"the way in ended on {float(n):.1f} N"
                              + ("" if "peak_n" in ran else " (the arm's reading once per period, not the peak)"))
            where = None if got["stopped"] == "goal" or met else (
                "on the way to the pose before the part" if not on_last else "on the last leg in")
            return met, where
        met, where = judge(got, len(route))
        if where:                                      # the last leg in is where a contact is meant: it is taken
            return dict(out, ok=False, why=f"the way in stopped {where} ({got['stopped']}"
                        + (f": {got['why']}" if got.get("why") else "") + ")")
        if choice["closes"]:
            # take hold where it stopped: at the hold pose, or where the expected contact ended the leg. A hand
            # that stands pressed on what stopped it eases first (the core's own ease, only where the still arm
            # reads a force over its level): the jaws never close on a part the hand bears down on
            if met:
                self._ease_if_pressing()
            c, ev = self._fx_close(part, choice, limit)
            if ev is None:
                return dict(out, ok=False, why=c["refused"])
            if not ev["held"] and met and ev["short_of_pose_mm"] > 0 and not F.spent(self):
                # what stopped the leg does not stand between the jaws: open again, go on to the pose under the
                # guard FROM WHERE THE HAND STANDS, and close there. The whole way in is never run again on this
                # pose (measured on the own bench: run again at the body's speed with the jaws narrower, the
                # fingers landed on the part at 378 mm/s and stood on it with 170 N while the jaws closed)
                control_hold.release(self, float(choice["open"]))
                more = F.stream(self, [last], word)
                self._fx_met("the way in, going on", more, part_p=part["p"], region_m=region)
                self._journal("fixture", stage="went_on", short_mm=ev["short_of_pose_mm"],
                              **{k: v for k, v in more.items() if k not in ("refused", "strike")})
                if more["stopped"] != "refused" and judge(more, 0)[1] is None:
                    first = ev["short_of_pose_mm"]
                    if more["stopped"] != "goal":
                        self._ease_if_pressing()
                    c, ev = self._fx_close(part, choice, limit)
                    if ev is None:
                        return dict(out, ok=False, why=c["refused"])
                    ev["went_on_from_mm"] = first
            out["held"] = ev
            if not ev["held"]:
                again = "" if ev.get("went_on_from_mm") is None else (
                    f" (it stopped {ev['went_on_from_mm']:.0f} mm before the pose first; the jaws opened and the "
                    "hand went on under the guard)")
                return dict(out, ok=False, why="the jaws closed on nothing where the way in stopped"
                            + (f", {ev['short_of_pose_mm']:.0f} mm before the pose" if met else "")
                            + f" ({c.get('ended')})" + again)
            # where the hold was taken: a turn keeps this height along the hand's axis. The pads press the part:
            # the hand stands on it (what the leave of this hold is told, _fx_stand)
            at = self._sense()
            self._fx["hold_at"] = {"tip": at.tip.copy()}
            self._fx["bore"] = "the pads press the part"
        elif met and choice["name"] == "hook":
            # a hook holds a part only from BEHIND it: a way in that ended on a contact further before the pose
            # than the tip goes past the part's point there (with the point's error and the arm's own give) has
            # not come down beside the part: it stands on something else, and a pull from there pulls nothing
            short = float((choice["p_hold"] - self._sense().tip) @ choice["a"])
            reach = float(choice.get("depth_m") or 0.0) + BAND * float(part["sigma_m"] or 0.0) \
                + B.rigid(self._sense().q, choice["a"])
            if short > reach:
                return dict(out, ok=False, why=f"the hook's way in ended on a contact {short * report.MM:.0f} mm "
                            f"before the pose behind the part ({got['stopped']}): the finger is not beside the "
                            "part, what it met stands in the way")
        if met and not choice["closes"]:
            self._fx["bore"] = "the way in ended on the part"
        return dict(out, ok=True, why=None, met=bool(met))

    def _fx_unwind(self):
        """A stroke or a turn can leave the servo wound up against the part (the command stands off the hand by
        more than the hand's still band): the command is brought to where the arm stands (the core's ease) before
        the hold is let go, so that the wind up is not let go into the part and the leave starts where the hand
        is. Costs control periods only where the servo is wound."""
        s = self._sense()
        q_des = getattr(self, "_q_des", None)
        wound = 0.0 if q_des is None else float(np.linalg.norm(self.model.fk_tip(q_des)[0] - s.tip))
        if wound > self._still_band(s) and not F.spent(self):
            self._ease_impl()
        return wound

    def _fx_stand(self, into):
        """Before the hand leaves a hold or a part it worked: bring a wound servo to where the arm stands
        (_fx_unwind) and, ONLY where this call's own records show the hand on the part (_fx["bore"]: the pads
        press it, the way in ended on it, a stroke read a force over what the still arm tells), write the record
        of the contact the hand stands in: ex._stand = {"tip", "F", "call"}, the same record every guard of the
        core writes, so that the core's leave runs. F is the hand's reading along `into` (the unit
        direction from the hand to the part: the way it came in, or the stroke) as it is; a reading that does
        not point that way is recorded as no push (the record then names no direction: the core prices the move
        as a leave, never as a press). Where nothing shows the hand on anything, nothing is written: a hand in
        free air leaves nothing, and what the core's own guards recorded stands as they left it. Returns the
        words of what showed it, or None."""
        wound = self._fx_unwind()
        bore = self._fx.get("bore")
        s = self._sense()
        call = getattr(self, "_calls", None)
        if bore:
            into = np.asarray(into, dtype=float)
            push = max(0.0, float(np.asarray(s.F, dtype=float) @ into))
            self._stand = {"tip": s.tip.copy(), "F": push * into, "call": call}
            self._fx["bore"], self._fx["stood"] = None, bore      # one record per contact: the core ends or keeps it
        elif not self._fx.get("met") and (self.__dict__.get("_stand") or {}).get("call") == call:
            # no motion of this call ended on anything, the pads press nothing and no stroke read the part: a
            # record a guard wrote in this call (a reading of the arm's own motion over its floor) is of no
            # contact, and the way out is no leave
            self._stand = None
        self._journal("fixture", stage="stand", wound_mm=wound * report.MM, tip_mm=_mm(s.tip), written=bool(bore),
                      shown_by=bore, push_n=None if not bore else float(np.linalg.norm(self._stand["F"])))
        return bore

    def _fx_let_go(self, part, choice, intent, d=None):
        """The let_go phase: open through the core's release (a hold), back off against the way in (kind lift: the contact
        stop only looks ahead), and on to the pose before the part where the hand came in through that pose and no
        stroke moved the part along (a turn, a hold that was not taken). Where the way in stopped short of that
        pose on something, the pose lies on the far side of what stopped the hand; after a pull or a push (d
        given) it lies where the part has moved to: a free leg there strikes it (measured on the own bench: 261 N
        on a drawer whose stated limit was 20 N), and the hand stays where the back off ended. A hook first comes
        off the part against the stroke d by the room it had behind it, so that leaving does not draw the part
        on."""
        J, B = self._fx_jaws, H.bounds_of(self)
        S.stand(self, B, self._sense())          # the push the hand stands with as it begins to leave (a figure)
        if choice["name"] == "hook" and d is not None and choice.get("side_m"):
            s = self._sense()
            off = choice["side_m"] + B.rigid(s.q, d)
            self._fx_stand(d)
            F.stream(self, [{"kind": "lift", "to": _pose(s.R, s.tip - off * np.asarray(d, dtype=float)), "line": True}],
                     intent)
        if choice["closes"] and J is not None:
            # jaws stand on the part where the core holds something between them (its own hold record: made only
            # where a touch was told) and their pads press it as they are about to open, and only there (a hold
            # that slipped presses nothing; jaws that closed on nothing press their own end, and the core holds
            # nothing)
            on = control_hold.held(self) is not None and H.jaw_read(self)[1]
            self._fx["bore"] = "the pads pressed the part until they opened" if on else None
            self._fx_unwind()
            arm = float(np.linalg.norm(np.asarray(self._sense().F, dtype=float)))
            S.release(self, control_hold.release(self, float(choice["open"])), arm)     # a jaw motion: its own figure
        s = self._sense()
        a = s.R @ (choice["R"].T @ choice["a"])
        back = (J.finger_len_m or J.max_m) if J is not None else B.robot.w_tool()
        back += B.standoff(0, part["thing"], s.q, -a)
        # the hand leaves the hold or the part it stood on: the core's guard is told so by the record
        # every guard writes where the tool last bore on something (ex._stand), written here only where this
        # call's own records show the hand on the part (_fx_stand). What the part puts on the tool as the hand
        # backs off then ends no move (a hand that a long turn left pressed on its part read 58 N in the first
        # period of its way back and stood); a push that keeps growing does, and so does one over a limit
        # somebody stated for the part: a sheet, or max_n or gentle on this call, stand until the contact has ended
        self._fx_stand(a)
        s = self._sense()
        legs = [{"kind": "lift", "to": _pose(s.R, s.tip - back * a), "line": True}]
        home = d is None and float((s.tip - choice["p_pre"]) @ np.asarray(choice["a"], dtype=float)) >= 0
        home_refused = None
        if home:
            # ... and on to the pose before the part (a turn leaves the wrist wound up): as a straight line of the
            # tip with the hand turning back on the way, so that the hand stays beside the part it has just
            # left. A joint leg to the posture the way in was planned with swings the whole arm when the turn
            # has carried the joints far from it (measured on a development cell: after a 39 deg turn the joint
            # leg took the hand 110 mm aside and 40 mm down onto what stood beside the part, 98 N read); it is
            # kept as the second choice, for an arm the bounds plan no such line for
            line = legs + [{"kind": "transit", "to": _pose(choice["R"], choice["p_pre"]), "line": True}]
            got = F.stream(self, line, intent)
            if got["stopped"] == "refused":
                got = F.stream(self, legs + [{"kind": "transit", "to": choice["q_pre"]}], intent)
            if got["stopped"] == "refused":
                # the way home is refused: the leave alone runs, so that the hand does not stay on the part it
                # stood on (a [lift, home] refused path_plane left a finger on a device's stop button, 7.5 N
                # under max_n 5)
                home_refused = _refused_text(got)
                got = F.stream(self, legs, intent)
        else:
            got = F.stream(self, legs, intent)
        opened = "opened and " if choice["closes"] and J is not None else ""
        full, share, why = back, 1.0, None
        while got["stopped"] == "refused" and back * share * H.HALF > B.rigid(s.q, a):
            # the leave is refused: the hand backs off along its own axis as far as the bounds plan it, half the
            # way and half again, down to what the arm resolves (a turn left a link where every whole way out was
            # refused path_plane at 61 % of its path, and a finger stood on a device's stop button)
            why, share = why or _refused_text(got), share * H.HALF
            got = F.stream(self, [{"kind": "lift", "to": _pose(s.R, s.tip - back * share * a), "line": True}], intent)
        if share < 1.0 and got["stopped"] != "refused":
            got["did"] = (opened + f"backed off {back * share * report.MM:.0f} mm along the hand's own axis, as far as "
                          "the bounds plan it")
            self._fx.setdefault("out_said", []).append(
                f"The way back from the part was refused ({why}): the hand backed off {back * share * report.MM:.0f} of "
                f"the {full * report.MM:.0f} mm along its own axis, as far as the bounds plan it"
                + (f"; the way back to the pose before the part was refused too ({home_refused})" if home_refused else "")
                + ".")
            home_refused = None
        elif got["stopped"] == "refused":
            why = _refused_text(got)
            got["did"] = (opened.replace(" and ", "; ") if opened else "") + f"the way out was refused ({why}): the hand stands where it stopped"
            self._fx.setdefault("out_said", []).append(
                ("The jaws opened; the" if opened else "The") + f" way back from the part was refused ({why})"
                + (f", and so was the way back to the pose before it ({home_refused})" if home_refused else "")
                + ": the hand stands where it stopped.")
        elif home_refused:
            got["did"] = opened + "backed off along the hand's own axis; the way back to the pose before the part was refused"
            self._fx.setdefault("out_said", []).append(
                f"The way back to the pose before the part was refused ({home_refused}): the hand backed off along "
                "its own axis only.")
        self._fx_met("the way out", got)
        self._journal("fixture", stage="let_go", back_mm=back * report.MM, tip_mm=_mm(self._sense().tip), home=home,
                      stand_kept=self.__dict__.get("_stand") is not None,
                      **{k: v for k, v in got.items() if k not in ("refused", "bounds", "strike")})
        return got
