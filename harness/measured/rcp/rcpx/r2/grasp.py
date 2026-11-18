# SPDX-License-Identifier: AGPL-3.0-only
"""Package grasp: force that closes through the hand and a watched held state, over the core's one hold.

Grasp closes through rcpx/control_hold.py close: at the jaws' own rate until their own reading tells a thing between
them, then the squeeze the caller states (force_frac) or the closing command itself. max_n states the most the
thing may take: the close then runs no faster at the touch than keeps it and says what that rests on; a limit under
what the jaws leave standing is refused (limit_not_keepable) before they move. The core keeps the one record of the
hold; this package keeps its tool facts (the width, the watch) and points at it. A stated width that the stall does
not match is said, never an ending. Release opens through control_hold.release and ends the hold.

Pinch places the moving jaw's tip with move_tip, closes by the stated share of the opening, then lets the squeeze
go (the command a rest band open of the reading). It reports give and resistance without leaving a held state.
Impulse needs no jaws; its arrival budget, slope, steady push and coast are ImpulseMixin's (grasp_impulse.py).

The post watch reports opening or slip from the core's record and ends the watched state with grasp_lost. The
gripper post hook reads the core's result; it never steps the robot. Reach hooks compare the operator's planned
force, or the reach verdict itself, with the measured base give. The body patch exposes the hand's model geometry
and servo range.

A robot without jaws refuses grasp, release and pinch. Every observation is from the robot's reading or its model;
nothing here reads world truth."""

import mujoco
import numpy as np

from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx import control_hold
from rcpx.bounds_report import item
from rcpx.context import refusal
from rcpx.control_base import _mm, primitive
from rcpx.r2.grasp_hand import Hand, finite as _finite, jaw_facts, width_err_m, release_width_m
from rcpx.r2.grasp_hooks import body_patch, gripper_post, reach_post, reach_pre, watch
from rcpx.r2.grasp_impulse import ImpulseMixin, coast_watch
from rcpx.r2.grasp_tools import PRACTICES, TOOLS

ID = "I26"
MM = 1000.0


def _grasp_n_text(value):
    """A reader-facing force, including the unresolved case, SPEC-R4 A5."""
    return "unknown" if value is None else f"{value:.2f}"


def _no_gripper():
    """Refuse jaw tools on a robot without jaws; impulse remains available."""
    return refusal("no_gripper", "this robot has no gripper: its tool is a fixed finger",
                   "use touch and press with the tip; impulse needs no gripper")


def _frac_arg(value, name, default=None):
    """Accept any finite stated share above zero and up to the servo range, SPEC-R4 A6."""
    v = default if value is None else value
    if not _finite(v) or not (0 < float(v) <= 1):
        return None, refusal(name, f"{name} must be a finite share with 0 < share <= 1, got {v!r}",
                             "leave it out to use the hand's derived squeeze")
    return float(v), None


def _said(out):
    """Lead every result with what happened and the next action, SPEC-R4 A5."""
    if out.get("refused"):
        r = out["refused"]
        text = r.get("why", "the call was refused") + "; " + r.get("hint", "look before retrying")
    else:
        text = out.get("said") or ("the jaws hold the part; lift and watch the hold" if out.get("holding") else
                                   "the jaws finished; read the hold judgement before lifting")
    return _readable({"said": text, **out})


def _readable(out):
    """Round outside readings to 0.01 mm, 0.01 N and 0.001 s for the reader."""
    if not isinstance(out, dict):
        return out
    result = {}
    for key, value in out.items():
        if isinstance(value, dict):
            value = _readable(value)
        elif isinstance(value, float):
            if key == "value" and out.get("unit") in ("mm", "mm/s", "N"):
                value = round(value, 2)
            elif key == "value" and out.get("unit") == "s":
                value = round(value, 3)
            elif "_mm" in key or "_n" in key:
                value = round(value, 2)
            elif key.endswith("_s"):
                value = round(value, 3)
            elif "frac" in key or "share" in key:
                value = round(value, 4)
        result[key] = value
    return result


def _squeeze_items(squeeze, planned_s, limit=None):
    """The squeeze and the jaw travel time as bounds with their sources (C1): a stated limit binds first."""
    out = []
    if limit is not None and limit.get("max_n") is not None:
        out.append(item("force", limit["max_n"], "intent", "your stated limit on the thing (max_n): "
                        + limit["kept"], "state a larger max_n"))
    if squeeze["source"] == "stated":
        out.append(item("force", squeeze["asked_n"], "intent", "the squeeze you stated (force_frac)",
                        "state a larger force_frac"))
    else:
        out.append(item("force", squeeze["range_n"], "model", "the jaw servo's own force range: the closing command "
                        "stands, as the plain interface closes", "nothing: the jaw servo's own range (state "
                        "force_frac or max_n to squeeze less)"))
    out.append(item("time", planned_s, "self_test" if squeeze.get("jaw_numbers") == "self_test" else "model",
                    "the opening over the jaws' own rate", "nothing: the jaws' own rate"))
    return out


class Mixin(ImpulseMixin):
    # ---------------------------------------------------------------- readings (helpers are prefixed: every
    # package's Mixin shares one class)
    """Package primitives and helpers; private names are scoped to their package."""
    def _grasp_read(self, track=False):
        """(q, target, width_m) of the gripper now, from Backend.read()["gripper"]; (None, None, None) if absent.
        track: this reading follows the last one by one control period (the package's own loops), so the change
        between the two may be the jaw's step (Hand.note_width); any other reading breaks that chain, so a jump
        between two calls is never taken for a step."""
        hand = self._grasp_hand
        q, target = hand.reading(self.backend.read().get("gripper"), self._gripper_cmd)
        if q is None or q.shape[0] != len(hand.joints):
            return None, None, None
        width = hand.width_m(q)
        if not track:
            hand.last_width_m = None
        hand.note_width(width)
        return q, target, width

    def _grasp_squeeze(self, hand, force_frac):
        """What the hold squeezes with: the stated share of the jaw servo's range (N at the jaw tips), else the
        closing command itself (the servo's own force)."""
        full = hand.full_squeeze_n()
        asked = None if force_frac is None or full is None else float(force_frac) * full
        return {"asked_n": asked, "source": "stated" if force_frac is not None else "the closing command",
                "least_n": None, "range_n": full, "jaw_numbers": control_hold.numbers(self)["source"]}

    # ---------------------------------------------------------------- grasp
    @primitive
    def grasp(self, width_mm=None, force_frac=None, max_n=None):
        """Close on the part through the core's one close and start a watched hold (C8)."""
        return _said(self._grasp_impl(width_mm, force_frac, max_n))

    def _grasp_impl(self, width_mm, force_frac, max_n=None):
        """Validate the stated width, squeeze and limit, close, and report what the jaws' own reading told."""
        hand = self._grasp_hand
        if hand is None:
            return _no_gripper()
        if force_frac is not None:
            ff, bad = _frac_arg(force_frac, "force_frac")
            if bad is not None:
                full = hand.full_squeeze_n()
                bad["refused"]["hint"] = (f"use 0 < share <= 1 of the servo's {_grasp_n_text(full)} newtons, "
                                         "or leave it out")
                return bad
        else:
            ff = None
        if max_n is not None and (not _finite(max_n) or float(max_n) <= 0):
            return refusal("max_n", "max_n must be a positive number of newtons", "the most the part may take")
        q, _, w0 = self._grasp_read()
        if q is None:
            return refusal("gripper_reading", "the gripper reports no joint reading", "read_state shows the gripper")
        gate = self._selfknow_gate("close")
        if gate is not None:
            return gate
        B, facts = bounds.of(self), jaw_facts(self)
        N = control_hold.numbers(self)
        err = width_err_m(hand, facts)
        wm = None
        if width_mm is not None:
            if not _finite(width_mm) or float(width_mm) <= 0:
                return refusal("width_mm", "width_mm must be a finite positive width",
                    "state the part width across the jaws")
            wm = float(width_mm) / MM
            if wm > hand.max_m + err:
                return refusal("width_mm", f"the jaws open at most {hand.max_m * MM:g} mm", "try a narrower side")
            if w0 < wm - err:
                hint = release_width_m(hand, facts, B, wm) * MM
                return refusal("too_narrow", f"the jaws open {w0 * MM:g} mm, under the stated part width",
                               f"release(width_mm={hint:g}), then put the jaws around the part and grasp")
        closed_m = hand.width_m(hand.q_of_frac(N["closed_gap"]))
        if w0 - closed_m <= R.QUANT * N["still_band"] * N["gap_per_span_m"]:
            return refusal("gripper_closed", "the jaws are at their closed gap: nothing new fits",
                           "release(), put the open jaws around the part, then grasp")
        squeeze = self._grasp_squeeze(hand, ff)
        moved_at = self.backend.read()["t"]
        # the stated width says how wide the part is, not where it stands between the jaws: no stretch of the
        # close is known free, so under a stated limit the jaws are slow from where they stand
        r = control_hold.close(self, squeeze_n=squeeze["asked_n"],
                               limit_n=None if max_n is None else float(max_n), limit_source="intent")
        if "refused" in r:
            return r
        q, target, w1 = self._grasp_read()
        holding = control_hold.held(self) is not None      # the core's record, as it stands after the close
        matches = None if wm is None else abs(w1 - wm) <= err
        sq = hand.squeeze_n(q, target)
        cmd_mm = None if target is None else hand.width_m(target) * MM
        full = squeeze["range_n"]
        speed = N["speed"] * N["gap_per_span_m"]
        limit = None if r.get("limit_n") is None else {
            "max_n": r["limit_n"], "source": r["limit_source"], "kept": "shown kept" if r["kept"] else "not shown kept"}
        out = {"holding": holding, "closed_on_object": bool(r["on_thing"]),
               "width_mm": round(w1 * MM, 2), "width_before_mm": round(w0 * MM, 2),
               "closed_mm": round((w0 - w1) * MM, 2), "squeeze_n_est": sq, "force_frac": ff,
               "peak_squeeze_n_est": sq, "servo_share": None if sq is None or not full else min(1.0, sq / full),
               "stopped": r["ended"], "gripper": {"width_frac": self._gripper_cmd, "open_frac": hand.frac_of_q(q)},
               "jaws_stopped_short_mm": round((w1 - closed_m) * MM, 2), "pressing": bool(sq),
               "still": True, "squeeze": squeeze, "price": r["price"],
               "close_mm_s": round(r["rate_span_s"] * N["gap_per_span_m"] * MM, 2),
               "close_rate_spans": float(r["rate_span_s"]),    # spans per s, as it ran
               "close_s": (self.backend.read()["t"] - moved_at),
               "planned_s": max(0.0, w0 - w1) / speed if speed > 0 else None,
               "periods": round((self.backend.read()["t"] - moved_at) / B.dt()),
               "arrive_periods": r["arrive_periods"], "arrive_off_mm": round(r["arrive_off_m"] * MM, 2)}
        if limit is not None:
            out["limit"] = limit
        if r.get("pressed_world_n") is not None:
            out["pressed_world_n"] = r["pressed_world_n"]
        if wm is not None:
            out.update(expected_width_mm=width_mm, width_matches=bool(matches))
            if any(hand.hinge):
                out["squeeze_at_part_n_est"] = hand.squeeze_at_part_n(q, target, wm)
            if holding and matches is False:
                out["doubt"] = [{"what": f"the jaws stalled {w1 * MM:.1f} mm apart where you stated {float(width_mm):g} "
                                         f"mm (known to {err * MM:.1f} mm): they hold something of another width, "
                                         "or a corner of it", "acted": "holding what they closed on",
                                 "would_remove": {"tool": "look", "args": {}, "periods": 0}}]
        if not holding:
            out["why"] = ("the jaws met each other: nothing was between them" if r["ended"] == "closed" else
                          "the close ended on its limit (the stated limit, or what the arm itself can push) "
                          "before the jaws' own reading told a thing between them: nothing is held"
                          if r["ended"] == "limit" else
                          f"the close ended {r['ended']} before the jaws stood on a thing")
            out["hint"] = ("release(), then grasp with a limit stated (max_n): the jaws then close slowly enough "
                           "to tell the part" if r["ended"] == "limit" and limit is None else
                           "release(), put the open jaws around the part, then grasp")
            out["said"] = "the jaws do not hold the part: " + out["why"]
        st = self.state.setdefault("grasp", {})
        st.update(held=holding, width_mm=w1 * MM if holding else None, since_call=self._calls,
                  slipped_out=False, squeeze_n=sq if holding else None, squeeze_asked_n=squeeze["asked_n"],
                  lead_mm=None if cmd_mm is None else max(0.0, w1 * MM - cmd_mm), cmd_mm=cmd_mm)
        self.note_event("grasp", holding=holding, width_mm=round(w1 * MM, 2),
            closed_on_object=out["closed_on_object"])
        return B.report(out, _squeeze_items(squeeze, out["planned_s"], limit))

    # ---------------------------------------------------------------- release
    @primitive
    def release(self, width_mm=None):
        """Open the jaws and end the watched hold, SPEC-R4 A4.7."""
        return _said(self._grasp_release(width_mm))

    def _grasp_release(self, width_mm):
        """Open to the stated tip width, or fully, and give geometric release advice, SPEC-R4 A4.7."""
        hand = self._grasp_hand
        if hand is None:
            return _no_gripper()
        frac = 1.0
        if width_mm is not None:
            if not _finite(width_mm) or float(width_mm) <= 0:
                return refusal("width_mm", f"width_mm must be a positive number of mm, got {width_mm!r}",
                               "leave it out to open fully")
            if float(width_mm) > (hand.max_m + width_err_m(hand, jaw_facts(self))) * MM:
                return refusal("width_mm", f"{float(width_mm):g} mm is wider than the jaws open "
                               f"({hand.max_m * MM:.0f} mm at the tips)", "leave width_mm out to open fully")
            frac = hand.frac_for_width(float(width_mm) / MM)
        if self._grasp_read()[0] is None:
            return refusal("gripper_reading", "the gripper reports no joint reading", "read_state shows the gripper")
        st = self.state.setdefault("grasp", {})
        was = bool(st.get("held"))
        part_m = st.get("width_mm")
        hint_m = None if part_m is None else release_width_m(hand, jaw_facts(self), bounds.of(self), part_m / MM)
        moved_at = self.backend.read()["t"]
        move = self.gripper(frac, hold=True)
        if "refused" in move:
            return move
        q, _, w = self._grasp_read()
        st.update(held=False, width_mm=None, slipped_out=False)
        if was:
            self.note_event("grasp_lost", reason="released", width_mm=round(w * MM, 2))
        return {"said": "the jaws opened; place them around the part before closing", "was_holding": was,
            "width_mm": round(w * MM, 2),
                "asked_width_mm": None if width_mm is None else float(width_mm), "open_frac": round(frac, 4),
                **({"hint": f"release(width_mm={hint_m * MM:g}): the part plus its error and the jaw sweep"}
                   if hint_m is not None else {}),
                "periods": round((self.backend.read()["t"] - moved_at) / bounds.of(self).dt()),
                "arrive_periods": move.get("arrive_periods", 0),
                "bounds": move.get("bounds") or {"binding": item(
                    "travel", hand.max_m, "model", "the jaws' own opening, from the model",
                    "nothing: the hand cannot open wider"), "more": 0}}

    # ---------------------------------------------------------------- pinch
    @primitive
    def pinch(self, part_point_mm, close_frac=None, force_frac=None, hold_s=None):
        """Place the moving jaw, close until still for the stated hold, then ease the squeeze, SPEC-R4 A4.7."""
        return _said(self._grasp_pinch(part_point_mm, close_frac, force_frac, hold_s))

    def _grasp_pinch(self, part_point_mm, close_frac, force_frac, hold_s):
        """Keep the placed motion and jaw close within their own bounds, then report resistance and give."""
        hand = self._grasp_hand
        if hand is None:
            return _no_gripper()
        if (self.state.get("grasp") or {}).get("held"):
            return refusal("holding", "the gripper is holding a part", "release() it first, or carry it")
        p = np.asarray(part_point_mm if isinstance(part_point_mm, (list, tuple, np.ndarray)) else [],
                       dtype=object).reshape(-1)
        if p.size != 3 or not _finite(*p):
            return refusal("part_point_mm", "part_point_mm must be three numbers", "[x, y, z] in base frame mm")
        p = p.astype(float) / MM
        cf, bad = (None, None) if close_frac is None else _frac_arg(close_frac, "close_frac")
        if bad is not None:
            return bad
        ff, bad = (None, None) if force_frac is None else _frac_arg(force_frac, "force_frac")
        if bad is not None:
            return bad
        q, _, w0 = self._grasp_read()
        if q is None:
            return refusal("gripper_reading", "the gripper reports no joint reading", "read_state shows the gripper")
        B, facts = bounds.of(self), jaw_facts(self)
        if hold_s is not None and (not _finite(hold_s) or float(hold_s) < 0):
            return refusal("hold_s", "hold_s must be a finite time >= 0", "leave it out: close until still")
        if w0 - facts.closed_gap_m <= R.QUANT * facts.step_m:
            return refusal("gripper_closed", f"the jaws are open only {w0 * MM:.1f} mm: no part fits between them",
                           "release(width_mm=...) wider than the part first, then pinch")
        s = self._sense()
        off = p - hand.jaw_point(s.q, q)
        dist = float(np.linalg.norm(off)) * MM
        if dist > w0 * MM:
            return refusal("too_far", f"the moving jaw's tip is {dist:.0f} mm from part_point_mm; a pinch moves the "
                           f"hand at most {w0 * MM:g} mm", "bring the open jaws around the part first "
                           "(move_tip), one jaw by the part and the other on its far side, then pinch")
        moved_at = self.backend.read()["t"]
        placed = {"stopped": "goal", "moved_mm": 0.0, "contact": None}
        if dist > B.rigid(s.q, off / np.linalg.norm(off) if dist > 0 else [0, 0, -1]) * MM:
            mv = self.move_tip(dx=float(off[0] * MM), dy=float(off[1] * MM), dz=float(off[2] * MM), frame="base")
            if "refused" in mv:
                return mv
            placed = {"stopped": mv["stopped"], "moved_mm": round(float(np.linalg.norm(mv["achieved_mm"])), 2),
                      "contact": mv.get("contact")}
            if mv["stopped"] not in ("goal", "contact"):
                out = {"placed": placed, "closed_mm": 0.0, "gave": False, "met_resistance": False,
                       "stopped": "not_placed", "width_mm": round(w0 * MM, 2),
                       "periods": round((self.backend.read()["t"] - moved_at) / B.dt()),
                       "why": f"the jaw could not be put on the part: {mv.get('why') or mv['stopped']}",
                       "hint": "picture(view=side) shows the arm; come closer with move_tip, then pinch"}
                if mv.get("bounds") is not None:
                    out["bounds"] = mv["bounds"]
                else:
                    _speed, speed_item = B.v_in(B.thing(p), s.q, off / np.linalg.norm(off))
                    B.report(out, [speed_item])
                return out
        q, _, w0 = self._grasp_read()
        target_w = 0.0 if cf is None else w0 * (1.0 - cf)
        squeeze = self._grasp_squeeze(hand, ff)
        N = control_hold.numbers(self)
        r = control_hold.close(self, to_frac=hand.frac_for_width(target_w), squeeze_n=squeeze["asked_n"])
        if "refused" in r:
            return r
        if hold_s:
            for _ in range(int(np.ceil(float(hold_s) / B.dt()))):
                self._period(self._q_des)
        q, target, w1 = self._grasp_read()
        peak = hand.squeeze_n(q, target) or 0.0
        # let the squeeze go: the command a rest band open of the reading (nothing keeps squeezing, I6); this
        # ends the core's record, so a pinch leaves no held state
        eased = control_hold.release(self, min(1.0, hand.frac_of_q(q) + R.QUANT * N["still_band"]
                                               + float(N["lag"][0])))
        _, _, w2 = self._grasp_read()
        band = R.QUANT * N["still_band"] * N["gap_per_span_m"] + facts.rest_band_m
        met = r["ended"] == "on_thing"
        told_m = r.get("told_gap_m")
        crawled = bool(met and told_m is not None and w0 - told_m <= band)
        fb = None if crawled or not met else told_m    # a crawl's stop is the jaws' own friction, not the part
        gave = bool(fb is not None and fb - w1 > band)
        ref = f"N{self._calls}"
        self.world.note_sense("pinch", ref, summary={"closed_mm": round((w0 - w1) * MM, 2), "gave": gave})
        out = {"placed": placed, "width_before_mm": round(w0 * MM, 2), "width_mm": round(w2 * MM, 2),
               "asked_close_mm": round((w0 - target_w) * MM, 2), "closed_mm": round((w0 - w1) * MM, 2),
               "met_resistance": fb is not None, "resisted_at_mm": None if fb is None else round(fb * MM, 2),
               "gave": gave, "peak_squeeze_n_est": round(peak, 2), "stopped": r["ended"], "price": r["price"],
               "eased": eased.get("ended") == "opened", "ref": ref, "tip_mm": _mm(self._sense().tip)}
        if crawled:
            out.update(stopped="crawled", why="the jaws made no progress before stopping; resistance is unknown",
                       hint=f"state force_frac; this close used {_grasp_n_text(squeeze['asked_n'])} of "
                            f"{_grasp_n_text(squeeze['range_n'])} newtons")
        out["periods"] = round((self.backend.read()["t"] - moved_at) / B.dt())
        speed = N["speed"] * N["gap_per_span_m"]
        return B.report(out, _squeeze_items(squeeze, max(0.0, w0 - w1) / speed if speed > 0 else None))


RESOURCE_PATCH = {"robot://body": body_patch}


def setup(ex):
    """Register the package readers and hooks without performing motion."""
    g = ex.model.spec.get("gripper")
    ex._grasp_hand = Hand(ex.model) if g else None
    ex._grasp_dd = mujoco.MjData(ex.model.mj)   # scratch data on the robot's own model (the impulse's tip mass)
    ex._grasp_coast = None
    ex.add_tick(coast_watch(ex))
    ex.state["grasp"] = {"held": False, "width_mm": None, "since_call": None, "slipped_out": False}
    ex.add_post(watch(ex))
    ex.add_post(gripper_post(ex), names=["gripper"])
    ex.add_pre(reach_pre, names=["reach_check"])
    ex.add_post(reach_post(ex), names=["reach_check"])


__all__ = ["ID", "Mixin", "PRACTICES", "RESOURCE_PATCH", "TOOLS", "setup"]   # what rcpx.r2.core reads
