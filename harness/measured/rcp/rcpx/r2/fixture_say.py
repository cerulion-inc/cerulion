# SPDX-License-Identifier: AGPL-3.0-only
"""The receipt of package fixture (round six, the design notes package PLACE stage C; contracts C2, C12): what
work_part says of what it did.

- _fx_words: a direction in the operator's words.
- _fx_seen: what one part of the call put on the part, held against the limit somebody stated on it (C3): the way
  in's stop, the pads' squeeze (both pads press it: the part sees their sum), the stroke's largest force.
- _fx_result: the frame of every receipt: phases, each with its control periods (they partition periods_used in
  order) and the worst strike of the motions it ran (rcpx/r2/fixture_strike.py; C12), the bounds, the call's
  strike (the worst of its phases'), the doubt lines, and, when a limit was stated, the block "limit": {limit_n,
  source, kept, read, passed, not_shown, cost} (cost: the control periods the operator's own word cost,
  rcpx/r2/fixture_cost.py, said in one sentence). A limit that was passed, or that the arm or the jaws cannot
  show kept, is said in the first sentence; the tool never says a limit was kept that it did not keep. A way
  out the bounds refused is said (fixture_take._fx_let_go: _fx["out_said"]).
- _fx_said, _fx_turn_said: the first sentence and the facts of a pull or push, and of a turn. A stroke that
  moved the part and ended on a stop says in next what it stopped against: the arm feels a stop, not whether it
  is the part's own end.
- _fx_met: what one motion of the call met, from that motion's own stop record (its strike block's "ran", else
  the contact's reading): where, the force, the speed it was met at. Every sentence of the receipt is built from
  these records (the design rulings rule 2): the largest force of the call is the largest over EVERY motion,
  the way in and the way out included ("largest_force_n", "largest_force_where"); a way in that ended on a
  contact is said as met; a stroke that met no stop says what it read along the way and across it against what
  this arm tells (never "did not move" or "did not come along" from a silence), and, where the hand stood on
  something or read over the still arm's level, that whether the part moved is not shown.
- The words of "ended" (and of "stopped", which keeps the stroke's own word where it has one more: steps_spent,
  time, contact, posture): end, amount, link_contact, slipped; did_not_move (the part
  held against the whole push this call may make: the ramp reached its ceiling and the tip did not pass rigid);
  nothing_felt (the stroke met no resistance it could tell: its readings are said; moved None where the hand
  stood on something or read over the still arm's level, else False); over_ceiling (the force stood over the
  ceiling before the part moved); not_taken (no hold was made); not_shown (a turn whose torque about the hand's
  axis stayed under what this arm can tell, or a stroke that ended before it showed the part moving for a reason
  of its own: the step budget, a side force; moved None). The word "refused" is kept for a refusal (the key
  "refused"), which this file never writes.
- A turn says "Turned the part" only where the hand turned WHILE a torque over the arm's own noise stood about its
  axis over a resolved share of the turn (more than GIVE of it: a part that comes along resists nearly all the
  way; pads that slip round a part read a torque over a fraction of a degree), and it claims the angle turned
  under that torque, never the hand's whole angle; else "the hand turned ..., the part's motion is not shown"
  and moved is None (not shown), never true. The same for a pull or a push with no fitted path that lost the part
  or went no further than the free play of its hold (unless that was the amount asked): the hand moved, the
  part's motion is not shown.
Nothing here reads truth, names a robot or a device, or holds a typed limit.
"""

import math

import numpy as np

from rcpx import bounds_laws as laws
from rcpx import bounds_report as report
from rcpx.bounds_ratios import GIVE
from rcpx.r2 import fixture_cost as COST
from rcpx.r2 import fixture_follow as F
from rcpx.r2 import fixture_hold as H
from rcpx.r2 import fixture_strike as S
from rcpx.r2.fixture_probe import unit


class SayMixin:
    def _fx_words(self, d):
        up = H.up_of(self)
        s = self._sense()
        back = -unit(s.tip - float(s.tip @ up) * up)
        if abs(float(d @ up)) > math.cos(math.pi / (laws.TWO * laws.TWO)):
            return "up" if float(d @ up) > 0 else "down"
        if abs(float(d @ back)) > math.cos(math.pi / (laws.TWO * laws.TWO)):
            return "toward the robot" if float(d @ back) > 0 else "away from the robot"
        return "to the robot's left" if float(np.cross(up, back) @ d) < 0 else "to the robot's right"


    def _fx_seen(self, where, newtons, text=None, shown=True):
        """One part of this call against the stated limit: where (the way in, the close, the stroke), the force it
        put on the part as far as the robot reads it (N; None: not read), text: the words for it, shown false:
        this part cannot show the limit kept (what it rests on is said in text). Nothing is kept when no limit
        is stated."""
        lim = self._fx.get("limit")
        if not lim:
            return
        if newtons is not None and math.isfinite(float(newtons)):
            lim["read"][where] = max(float(newtons), lim["read"].get(where, 0.0))
            if float(newtons) > lim["limit_n"]:
                lim["passed"].append(text or f"{where} read {float(newtons):.1f} N")
        if not shown:
            lim["not_shown"].append(text or where)

    def _fx_met(self, where, got, short_m=None, part_p=None, region_m=None):
        """Note what one motion of this call met, from its own stop record; returns the force (N) or None. got:
        a stream's result (fixture_follow.stream). short_m: how far before the hold pose it ended. part_p,
        region_m: the part's point and the radius of its own region (its error, half of what a look shows of it,
        the hand's reach): the contact is NAMED by where the core located it against that region ("from_part_m",
        "in_region"); nothing is ranked or stored by it."""
        if not isinstance(got, dict):
            return None
        ran = (got.get("strike") or {}).get("ran") or {}
        con = got.get("contact") or {}
        n = ran.get("peak_n", ran.get("read_n", con.get("force_n")))
        v = ran.get("v_m_s")
        if v is None and con.get("tip_speed_mm_s") is not None:
            v = float(con["tip_speed_mm_s"]) / report.MM
        if got.get("stopped") in ("arrived", "contact", "cap") and n is not None:
            off = None
            if part_p is not None and con.get("point_mm") is not None:
                off = float(np.linalg.norm(np.asarray(con["point_mm"], dtype=float) / report.MM
                                           - np.asarray(part_p, dtype=float)))
            self._fx.setdefault("met", []).append(
                {"where": where, "force_n": float(n), "v_m_s": None if v is None else float(v),
                 "stopped": got.get("stopped"), "peak": "peak_n" in ran, "on": con.get("surface"),
                 "short_m": short_m, "from_part_m": off, "region_m": region_m,
                 "in_region": None if off is None or region_m is None else bool(off <= float(region_m))})
        return n

    def _fx_largest(self, rec_peak=None, rec_where="the stroke"):
        """(the largest force any motion of this call read, where), over the noted stops and the stroke."""
        rows = [(m["force_n"], m["where"]) for m in self._fx.get("met") or []]
        if rec_peak is not None:
            rows.append((float(rec_peak), rec_where))
        return max(rows) if rows else (None, None)

    def _fx_way_in_said(self):
        """One clause on what the ways in of this call met, or '' when none met anything."""
        ins = [m for m in self._fx.get("met") or [] if m["where"].startswith("the way in")]
        if not ins:
            return ""
        top = max(ins, key=lambda m: m["force_n"])
        at = "" if top["v_m_s"] is None else f" at {top['v_m_s'] * report.MM:.0f} mm/s"
        short = "" if not top.get("short_m") or top["short_m"] <= 0 else (
            f", {top['short_m'] * report.MM:.0f} mm before the hold pose")
        many = "" if len(ins) == 1 else f" ({len(ins)} ways in ended on a contact; this is the hardest)"
        if top.get("in_region") is not None:        # named against the part's own region, never ranked by it
            short += (f", {top['from_part_m'] * report.MM:.0f} mm from the point you gave ("
                      + ("inside" if top["in_region"] else "outside")
                      + f" the {top['region_m'] * report.MM:.0f} mm about it that this hand can bear on the part from)")
        return (f"the way in met something{short}: {top['force_n']:.1f} N read{at}"
                + ("" if top["peak"] else ", the arm's reading once per period, not the peak") + many)

    def _fx_out_item(self, back):
        """The binding bound of the way out for its phase record: the stream's own; where its plan names none
        (a short leave with nothing stated), the body's own speed, which is what ran. What the core says
        of the contact the hand left (the design rulings: left it, the limit that stood over the leave) goes on the record."""
        if self._fx["phases"]:               # how the way out itself ended (goal, or what stopped it)
            self._fx["phases"][-1]["way_out"] = back.get("stopped")
        if back.get("left") and self._fx["phases"] and (self._fx.get("stood") or self._fx.get("met")):
            # only where this call's own records show the hand ON something (a stop of one of its motions, pads
            # that pressed, a stroke that read the part): a hand in free air left nothing (the design rulings rule 2)
            self._fx["phases"][-1]["left"] = back["left"]
        if back.get("bounds") is not None:
            return back["bounds"]
        source, evidence = H.bounds_of(self).src("qd_max")
        return report.item("speed", None, source, "nothing stated lowers the way out: " + str(evidence),
                           "nothing: the robot's own rating")

    def _fx_squeeze_seen(self, choice):
        """What the pads press the part with after a stroke (a stroke may have raised the squeeze after a slip)."""
        if not choice.get("closes") or self._fx_jaws is None:
            return
        sq, pads = H.jaw_read(self)[2], len(self._fx_jaws.sides)
        if sq is not None:
            self._fx_seen("the pads' squeeze", pads * float(sq), f"the {pads} pads press the part with "
                          f"{float(sq):.1f} N each, {pads * float(sq):.1f} N together")

    def _fx_result(self, out, items):
        phases = self._fx["phases"]
        for rec in phases:                 # a phase the call left open ends here: its periods and its strike
            if rec.pop("_t0", None) is not None:
                S.close_phase(self, rec)
        used = sum(rec["periods"] for rec in phases)
        out.update(phases=phases, periods=[[rec["phase"], rec["periods"]] for rec in phases], periods_used=used)
        left = self.backend.read().get("steps_left")
        if left is not None:
            out["periods_left"] = int(left)
        out["bounds"] = [it for it in items if it is not None]
        worst = S.of_call(phases)           # every phase states its own (C12); the call's is the worst of them
        out["strike"] = worst
        top, where = self._fx_largest(out.get("peak_force_n"))
        out["largest_force_n"] = None if top is None else round(top, 2)
        out["largest_force_where"] = where
        out["met"] = [{"where": m["where"], "force_n": round(m["force_n"], 2), "stopped": m["stopped"],
                       "mm_s": None if m["v_m_s"] is None else round(m["v_m_s"] * report.MM, 1),
                       "from_part_mm": None if m.get("from_part_m") is None else round(m["from_part_m"] * report.MM, 1),
                       "in_part_region": m.get("in_region")}
                      for m in self._fx.get("met") or []]
        out["predicted_peak_n"] = None if worst is None else worst.get("peak_n")
        out["doubt"] = list(self._fx.get("doubt") or [])
        out["take"] = self._fx.get("take")
        lim = self._fx.get("limit")
        if lim:
            passed = list(dict.fromkeys(lim["passed"]))
            unshown = list(dict.fromkeys(lim["not_shown"]))
            out["limit"] = {"limit_n": lim["limit_n"], "source": lim["source"], "kept": not passed and not unshown,
                            "read_n": {k: round(v, 2) for k, v in lim["read"].items()}, "passed": passed,
                            "not_shown": unshown}
            if passed:
                out["said"] += (f" The limit of {lim['limit_n']:g} N stated for the part was PASSED: "
                                + "; ".join(passed) + ".")
            elif unshown:
                out["said"] += (f" The limit of {lim['limit_n']:g} N stated for the part is not shown kept: "
                                + "; ".join(unshown) + ".")
            cost = COST.of_call(self, used, (getattr(self, "_meant", None) or {}).get("words"))
            if cost is not None:
                out["limit"]["cost"] = cost
                if cost["periods"] >= 1:
                    out["said"] += (f" Your limit cost about {cost['periods']} of the {used} control periods this call "
                                    f"ran: without it about {cost['without_periods']}"
                                    + (f" ({' and '.join(cost['not_priced'])} under it not counted)"
                                       if cost.get("not_priced") else "") + ".")
        for line in self._fx.get("out_said") or []:
            out["said"] += " " + line
        self.state["fixture"]["last"] = {k: v for k, v in out.items() if k != "phases"}
        return out

    def _fx_said(self, way, rec, fs, tried, eid, items):
        fit, moved = rec["fit"], rec["moved"]
        verb = {"pull": "Pulled", "push": "Pushed"}[way]
        mm = rec["travel_m"] * report.MM
        # the path's shape where the stroke fitted one; where it fitted none no shape is said (the fit claims a
        # line only past what the arm's give under the stroke's ceiling spans: a drawer pushed 119 mm was said
        # pushed "along a path too short to fit")
        along = ("along a straight path " if fit["shape"] == "line" else
                 f"along an arc of radius {fit['radius_m'] * report.MM:.0f} mm " if fit["shape"] == "arc" else "")
        # the stroke's own words for a part that did not move (the record's "refused" is the stroke's, no refusal)
        none = {"seek": "nothing_felt", "ramp": "did_not_move"}.get(rec.get("state"), "did_not_move")
        if rec["stopped"] == "refused" and "over the ceiling" in str(rec["why"] or ""):
            none = "over_ceiling"
        ended = rec["stopped"] if rec["stopped"] in F.ENDS and rec["stopped"] != "refused" else (
            "link_contact" if moved else none)
        stopped = ended if rec["stopped"] == "refused" else rec["stopped"]
        tail = {"end": "until it stopped", "amount": "to the amount you stated",
                "link_contact": "until a link met something",
                "slipped": "until the jaws lost it" if way == "pull" else "until it was no longer felt"}.get(rec["stopped"],
                                                                                                      f"and stopped: {rec['stopped']}")
        way_in = self._fx_way_in_said()
        play = float(rec.get("play_m") or 0.0)
        if moved and fit["shape"] == "none" and rec["stopped"] != "amount" and (
                rec["stopped"] == "slipped" or rec["travel_m"] <= play):
            # no path was fitted, and the hand lost what it held, or went no further than the free play its hold
            # leaves: whether the part came along is not shown (the design rulings rule 2; on the own bench a hand's jaws stalled
            # on the face beside a handle, the hand went 10 mm, the drawer stood shut by truth, and the receipt said
            # it had pulled the part 10 mm; a shut drawer pushed the wrong way was said pushed 1 mm). A long stroke
            # with no fitted shape (a drawer pushed 134 mm shut on a development cell) stays a motion
            moved, ended = None, "not_shown"
            how = ("no path was fitted before that" if rec["stopped"] == "slipped" else
                   f"no further than the {play * report.MM:.0f} mm of free play its hold leaves, and no path was fitted")
            said = (f"The hand went {mm:.0f} mm {self._fx_words(rec['d'])} {tail}, reading up to {rec['peak_n']:.1f} N "
                    f"(it went on from {rec['f_moved_n']:.1f} N), {how}: whether the part moved is not shown"
                    + (f"; {way_in}" if way_in else "") + ".")
        elif moved:
            said = (f"{verb} the part {mm:.0f} mm {along}{self._fx_words(rec['d'])} {tail}; it moved from "
                    f"{rec['f_moved_n']:.1f} N.")
            if rec["stopped"] == "end" and rec["why"]:
                said = said[:-1] + f" ({rec['why']})."
        elif ended == "nothing_felt":
            # an absence of a contact stop is no proof that nothing moved (the design rulings rule 2: an absence is reported
            # as an absence, never as a conclusion): the stroke says what it read along the way and across it,
            # against what this arm tells, and nothing more (a contact under what the arm tells can load the part
            # while the stroke reads nothing). Where the hand stood on something as the stroke began and went on
            # through where that stood, or read a force over what the still arm tells, whether the part moved is
            # not shown (moved None: a part can go on moving under a reading that stays under that level); where it
            # stood on nothing and read nothing over that level, the hand felt nothing (moved False)
            went = abs(float((rec['tip'] - rec['start_tip']) @ rec['d'])) * report.MM
            level = rec.get("level_n")
            over = level is not None and rec["peak_n"] > float(level)
            if rec.get("stood_on") or over:
                moved = None
            read = (f"it read up to {rec['peak_n']:.1f} N along the way and "
                    f"{float(rec.get('side_peak_n') or 0.0):.1f} N across it"
                    + ("" if level is None else
                       f": over what this arm tells at rest ({float(level):.1f} N), under what it tells from its own "
                       f"motion at the {rec['v_fast_m_s'] * report.MM:.0f} mm/s it ran ({rec['level_fast_n']:.1f} N)"
                       if over else f", under what this arm tells there ({float(level):.1f} N)"))
            if moved is None:
                said = (f"The stroke met no stop over {went:.0f} mm {self._fx_words(rec['d'])} from where the hand stood"
                        + (" on what its way in had met" if rec.get("stood_on") else "") + f". {read[0].upper()}{read[1:]}"
                        + ": whether the part moved is not shown" + (f"; {way_in}" if way_in else "") + ".")
            else:
                said = (f"The stroke felt nothing over {went:.0f} mm {self._fx_words(rec['d'])} from where the hand "
                        f"stood: {read}, of a ceiling of {rec['ceiling_n']:.1f} N" + (f"; {way_in}" if way_in else "")
                        + ".")
        elif not moved and rec["stopped"] not in ("refused", "end"):
            # the stroke ended before it showed the part moving, for a reason of its own (a link met something,
            # a side force, the jaws lost it, the step budget): what it read is said; that the part did not move
            # is not shown by it (a drawer stood shut by truth after "The part did not move: steps_spent", on a
            # development cell)
            moved, ended = None, (ended if ended in ("link_contact", "slipped") else "not_shown")
            said = (f"The stroke ended before it showed the part moving: {rec['why'] or rec['stopped']} (it read up to "
                    f"{rec['peak_n']:.1f} N along the way of a ceiling of {rec['ceiling_n']:.1f} N): whether the part "
                    "moved is not shown" + (f"; {way_in}" if way_in else "") + ".")
        else:
            said = (f"The part did not move: {rec['why'] or rec['stopped']} (the stroke read {rec['peak_n']:.1f} N of "
                    f"a ceiling of {rec['ceiling_n']:.1f} N)" + (f"; {way_in}" if way_in else "") + ".")
        out = {"said": said, "way": way, "moved": None if moved is None else bool(moved),
               "travel_mm": round(mm, 1) if moved else (None if moved is None else 0.0),
               "axis": rec["axis"], "moved_from_n": None if rec["f_moved_n"] is None else round(rec["f_moved_n"], 2),
               "ended": ended, "stopped": stopped, "why": rec["why"], "peak_force_n": round(rec["peak_n"], 2),
               "ceiling_n": round(rec["ceiling_n"], 2), "ceiling": fs.items[0]["evidence"],
               "part_mass_kg": None if rec["mass_kg"] is None else round(rec["mass_kg"], 3), "part": eid,
               "tried": tried,
               "next": ("a look shows it (0 control periods)" if moved else
                        "the stroke cannot tell if it moved; a look can (0 control periods)" if moved is None else
                        "the hand may have missed it: look (0 control periods)" if ended == "nothing_felt" else
                        "a stop, a stiff point or held: look (0 control periods)")}
        if moved and rec["stopped"] == "end":
            # a stop is all the arm felt: the part's own end, or something in its way that holds more than the
            # push; which of the two only a look tells
            out["next"] = (f"it stopped at {rec['peak_n']:.0f} N, under the {rec['ceiling_n']:.0f} N the call could "
                           "push: a stop, a stiff point or something in its way: look (0 control periods)"
                           + (f"; the other way ({'push' if way == 'pull' else 'pull'}) may be the one that gives"
                              if rec["why"] else ""))
        return self._fx_result(out, items)

    def _fx_turn_said(self, rec, rec_all, total, sign, strokes, eid, tried, items):
        """The receipt of a turn: rec the last stroke's record, rec_all the same with the whole angle and, under
        loaded_rad, how far the hand turned over all strokes while a torque over the arm's own noise stood about
        its axis."""
        deg = math.degrees(total)
        loaded = float(rec_all.get("loaded_rad") or 0.0)
        # the part came along only over a resolved share of the turn under a torque (module text): under max_n 5
        # the pads slipped round a knob that stood still by truth, a torque stood over a fraction of a degree,
        # and the receipt claimed the hand's whole 44 deg
        shown = bool(rec_all["moved"] and loaded > max(float(rec.get("resolved_rad") or 0.0), GIVE * total))
        ended = rec["stopped"] if rec["stopped"] in F.ENDS and rec["stopped"] != "refused" else (
            "end" if rec["stopped"] == "posture" else ("link_contact" if rec_all["moved"] and rec["stopped"] == "contact"
                                                      else "did_not_move"))
        way_word = "clockwise" if sign > 0 else "counterclockwise"
        tail = {"end": "until it stopped", "amount": "to the angle you stated"}.get(rec["stopped"],
                                                                                 f"and stopped: {rec['why'] or rec['stopped']}")
        if rec["stopped"] == "end" and rec.get("why"):     # an end the turn itself named (a whole turn, a stated force)
            tail += f" ({rec['why']})"
        way_in = self._fx_way_in_said()
        if rec.get("unsure"):
            # the strokes of a turn whose sense nobody stated ended with the hold not found again: what they
            # measured is said; whether the part turned is not shown (never "did not turn" from that)
            shown, ended = False, "not_shown"
            rec_all = dict(rec_all, moved=True)
            said = f"The part's motion is not shown: {rec['unsure']}" + (f"; {way_in}" if way_in else "") + "."
        elif shown:
            said = (f"Turned the part {math.degrees(loaded):.0f} deg {way_word} as the hand sees it {tail}: over that "
                    f"angle the hand read a torque about its axis (the largest {rec['peak_nm']:.2f} N m) and the part "
                    f"came along against it" + ("" if loaded >= total else f" (the hand turned {deg:.0f} deg in all)") + ".")
        elif rec_all["moved"]:
            ended = "not_shown" if ended in ("amount", "end") else ended
            under = ("read no torque about its axis over what this arm can tell" if loaded <= float(
                rec.get("resolved_rad") or 0.0) else f"read a torque about its axis over what this arm can tell over "
                f"only {math.degrees(loaded):.1f} deg of it, too little of the turn to show the part came along")
            said = (f"The hand turned {deg:.0f} deg {way_word} with the jaws closed on the part {tail} and {under} "
                    f"(the largest {rec['peak_nm']:.2f} N m): the part's motion is not shown"
                    + (f"; {way_in}" if way_in else "") + ".")
        else:
            said = (f"The part did not turn: {rec['why'] or rec['stopped']} (the largest torque about the hand's "
                    f"axis was {rec['peak_nm']:.2f} N m of a ceiling of {rec['ceiling_nm']:.2f} N m)"
                    + (f"; {way_in}" if way_in else "") + ".")
        out = {"said": said, "way": "turn", "moved": True if shown else (None if rec_all["moved"] else False),
               "hand_turned_deg": round(deg, 1),
               "angle_deg": round(math.degrees(loaded), 1) if shown else (None if rec_all["moved"] else 0.0),
               "sense": "cw" if sign > 0 else "ccw", "axis": {"shape": "turn", "direction": rec["axis"]},
               "moved_from_nm": None if rec["tau_moved_nm"] is None else round(rec["tau_moved_nm"], 3),
               "ended": ended, "stopped": ended if rec["stopped"] == "refused" else rec["stopped"], "why": rec["why"],
               "peak_torque_nm": round(rec["peak_nm"], 3),
               "ceiling_nm": round(rec["ceiling_nm"], 2), "strokes": strokes, "part": eid, "tried": tried,
               "jaw_gap_change_mm": None if rec["gap_change_m"] is None else round(rec["gap_change_m"] * report.MM, 2),
               "under_load_deg": round(math.degrees(loaded), 1),
               "next": "a look shows it (0 control periods)"}
        if rec.get("along_axis_m") is not None:
            # where the hand stands along its own axis against the height its hold was taken at, with the band
            # this arm cannot tell from none there (the turn keeps that height: fixture_follow.turn)
            out["off_hold_mm"] = round(float(rec["along_axis_m"]) * report.MM, 2)
            out["still_band_mm"] = round(float(rec.get("still_m") or 0.0) * report.MM, 2)
        return self._fx_result(out, items)
