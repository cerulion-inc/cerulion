# SPDX-License-Identifier: AGPL-3.0-only
"""Evidence-checked verify for the device package, a mixin of device.Mixin.

verify_evidence (served as the verify tool by device.HANDLERS) checks the evidence look itself before the round one
rule (Executor._verify_impl, World.verify) decides: the deciding part's located point projects inside the look's
frame at the look's pose (RobotModel.project), the robot's own body does not cover it, a small cue (or a cue of
unknown size) is not decided from a dim look, the cue's located point is not in doubt (it went dark there across an
act), and the look comes at least the entry's transition time after the last act. Otherwise the verdict is
"unobservable", with a view_hint (a move_tip delta from the camera model) when a move would help. Device kinds are
never named; no simulator ground truth is read here.

What it checked is said, never implied (FINDINGS S3): evidence_checked.tested is "cue_read" when the deciding part
is the device's cue and its state is a lit or dark word (the cue's light is read at the point in the look itself and
must agree with the state: a lit cue for on or lit, a dark one for off, unlit or dark; a reading that disagrees is
the verdict "contradicted" and the claim stays unconfirmed), else "in_view_only" (the point projects into the look,
clear of the robot, the look newer than the last act; the picture is not compared with the state: the state rests
on the operator's evidence, verdict "not_checked" when the round one rules accept it: the design rulings, no verdict of
acceptance without a reading of the robot's own behind it; the result's said names what the robot read, what it
could not check and what would check it). A point that
was located before a motion moved something near it (a result that says moved_part, gave or a probe that moved) is
not used unless it is a cue part: the part may have left it (reason "point_moved", locate it again).
"""

import os

import numpy as np
from PIL import Image

from rcpx.context import MARK_UNCONFIRMED, refusal
from rcpx.control_base import primitive
from rcpx.r2 import device_cue as cue
from rcpx.r2 import device_entry as de
from rcpx.r2 import device_bounds as derived

MM = 1000.0
SMALL_CUE_PX = 24.0      # a cue under this many pixels across is small: a dim look cannot decide it
LIT_STATES = ("on", "lit")
DARK_STATES = ("off", "unlit", "dark")
MOVES = ("touch", "press", "probe_motion", "impulse", "move_tip", "move_joints", "orient_tool", "approach")


def moved_something(name, result):
    """True when a motion's result says a part moved under it (the round one keys, read as they are)."""
    if not isinstance(result, dict) or "refused" in result:
        return False
    if result.get("moved_part") is True or result.get("gave") is True or result.get("stopped") == "moved_part":
        return True
    if result.get("outcome_class") in ("gave", "moved_under_load"):
        return True
    return name == "probe_motion" and result.get("kind") not in (None, "stuck")


def note_moves(ex):
    """Post hook: remember where and when a motion moved something (st["moved"])."""
    def post(name, kwargs, result):
        if not moved_something(name, result):
            return None
        at = None
        c = result.get("contact")
        if isinstance(c, dict) and c.get("point_mm") is not None:
            at = c["point_mm"]
        at = at or result.get("point_mm") or result.get("tip_mm")
        if at is not None:
            ex._dv().setdefault("moved", []).append({"call": ex._calls, "point_mm": [float(x) for x in at],
                                                      "primitive": name, "travel_m": (result.get("moved_under_load_mm") or result.get("travel_mm") or 0) / MM})
        return None
    ex.add_post(post, names=list(MOVES))


class VerifyMixin:
    def _dv_evidence_look(self, look_id):
        key, rec = self._look_rec(look_id)
        if rec is not None:
            return key, {"q": rec["q"], "t": rec["t"], "can_see": rec["can_see"], "verdict": None}
        w = self._dv()["watches"].get(str(look_id))
        return str(look_id), w

    def _dv_check(self, entry, state, key, rec):
        st, dev = self._dv(), de.ident(entry)
        if rec.get("verdict") == "unobservable":
            return {"reason": rec.get("reason") or "unobservable", "why": f"watch {key} could not see the cue"}
        pt, how, part = de.state_point(self, st, entry, state)
        if pt is None:
            listed = de.find(self, dev) is not None
            return {"reason": "no_point", "why": f"no located point on {dev} to check the look against",
                    "hint": ("touch the device, or mark the part that shows the state in two looks and triangulate it "
                             "with that part's name as the label, then look again") if listed else (
                        f"{dev} is not in robot://devices: mark the device in two looks with the label {dev!r}, "
                        "triangulate it, then look again")}
        radius_m = self._dv_radius_m(entry, part)
        geom = self._dv_point_geom(pt, radius_m, how, rec["q"])
        base = {"point_mm": geom["point_mm"], "point_from": how}
        # a cue that only shows the state is read below (and its doubt covers a cue that moved); another part's point
        # located before a motion moved something near it no longer says where that part is
        moved = None if de.is_cue_part(entry, part) else self._dv_moved_since(entry, pt)
        if moved is not None:
            return {**base, "reason": "point_moved", "view_hint": None,
                    "why": f"the point checked was located at call {moved['located_at']}, and the {moved['primitive']} "
                           f"at call {moved['call']} moved something {moved['mm']:.0f} mm from it: the part may "
                           "have left that point, so it cannot decide the state",
                    "hint": "locate the part that shows the state again (mark it in two looks taken now, triangulate "
                            "with the part's name as the label), then verify"}
        if not geom["in_frame"]:
            return {**base, "reason": "out_of_view", "why": f"look {key}: {geom.get('why')}",
                    "view_hint": self._dv_hint(pt, radius_m)}
        if geom["blocked_by"]:
            return {**base, "reason": "occluded_by_robot", "why": f"look {key}: the robot's {geom['blocked_by']} "
                    "covers the point", "view_hint": self._dv_hint(pt, radius_m)}
        doubt = self._dv_doubt(entry) if de.is_cue_part(entry, part) else None
        if doubt is not None:
            d = self._dv_doubted(doubt)
            return {**base, "reason": d["reason"], "why": d["why"], "hint": d["hint"], "view_hint": None}
        size = de.part_size_mm(entry, part)
        across = None if size is None else self.model.f_px * size / MM / max(geom["depth_mm"] / MM, 1e-6)
        # small: known to be under SMALL_CUE_PX in this look, or of unknown size and shown by the device's cue
        # (an indicator); a part of unknown size that is not a cue is judged as round one judges a dim look
        small = across < SMALL_CUE_PX if across is not None else de.shown_by_cue(entry, state)
        if rec.get("can_see") == "dim" and small and across is None:
            sized = [de.part_name(p) for p in de.cue_parts(entry) if de.part_size_mm(entry, p)]
            return {**base, "reason": "dim", "view_hint": None,
                    "why": f"look {key} is dim and {state} is shown by a cue whose size in the image is unknown "
                           "(no size on the sheet for the point checked), so no nearer dim look can decide it",
                    "hint": "get light on the scene (or wait for light), then look again and verify with that look"
                            + (f"; or locate {' or '.join(sized)} (mark it in two looks, triangulate) so its size is "
                               "known" if sized and part is None else "")}
        if rec.get("can_see") == "dim" and small:
            depth = max(derived.near_focus_m(derived.of(self), self.model)[0], (geom["depth_mm"] / MM) * (across / (2 * SMALL_CUE_PX)))
            cam_mm, _ = cue.camera_delta(self.model, self._sense().q, np.asarray(pt) / MM,
                                         (self.model.cam_w / 2, self.model.cam_h / 2), depth)
            return {**base, "reason": "dim", "why": f"look {key} is dim and the part is small in it "
                    f"({across:.0f} px across)",
                    "view_hint": {"tool": "move_tip", "args": {"dx": cam_mm[0], "dy": cam_mm[1], "dz": cam_mm[2],
                                                               "frame": "camera"}, "checked": "not checked: closer "
                                  "on the same line of sight; more light works too"}}
        last = self._dv_last_act(dev)
        if last is not None:
            prior = (self.world.device_states().get(dev) or {}).get("state")
            need = de.settle_s(entry, state, prior if prior and prior != state else None)
            gap = rec["t"] - last["t_end"]
            if gap < 0.0:
                return {**base, "reason": "too_soon", "why": f"look {key} was taken before the last act on {dev} "
                        f"(call {last['call']}): it shows the device as it was before that act",
                        "wait_s": round(need, 2),
                        "hint": f"look again now{f' (at least {need:g} s after the act)' if need > 0 else ''} and "
                                "verify with that look"}
            if gap < need - 1e-6:
                return {**base, "reason": "too_soon", "why": f"look {key} was taken {gap:.1f} s after the last act "
                        f"on {dev} (call {last['call']}); its entry says {state} is reached within {need:g} s",
                        "wait_s": round(need - gap, 2),
                        "hint": f"wait at least {need - gap:.1f} s more (watch or idle), then look again"}
        out = {**base, "reason": None, "px": [geom["u"], geom["v"]], "depth_mm": geom["depth_mm"]}
        word = str(state or "").lower()
        if de.is_cue_part(entry, part) and word in LIT_STATES + DARK_STATES:
            lv = self._dv_read_cue(rec, key, geom, across)
            if lv is None:
                out["tested"] = "in_view_only"
                out["cue_read"] = "the cue's patch could not be read in the look's frame"
            else:
                out["tested"], out["cue"] = "cue_read", lv
                if bool(lv["lit"]) != (word in LIT_STATES):
                    out.update(reason="contradicted", view_hint=None,
                               why=f"the cue at the located point reads {'lit' if lv['lit'] else 'not lit'} in look "
                                   f"{key} (peak {lv['peak']:g} against {lv['ring']:g} around it), which is not "
                                   f"{word}",
                               hint="look at the cue again (crop the look around its point); verify the state the "
                                    "cue shows")
        else:
            out["tested"] = "in_view_only"
        return out

    def _dv_moved_since(self, entry, pt):
        """The newest motion that moved something within the moved part's travel and extent of point pt after pt was located, or None."""
        at = None
        for p in de.located(self, entry):
            if np.allclose(np.asarray(p["point_mm"], dtype=float), np.asarray(pt, dtype=float)):
                at = p.get("at_call")
                break
        if at is None:
            return None
        for m in reversed(self._dv().get("moved", [])):
            d = float(np.linalg.norm(np.asarray(m["point_mm"]) - np.asarray(pt, dtype=float)))
            if m["call"] > at and d / MM <= m.get("travel_m", 0) + derived.extent_m(derived.of(self), entry, [pt]):
                return {**m, "located_at": at, "mm": d}
        return None

    def _dv_read_cue(self, rec, key, geom, across):
        """The cue's light level at the point in the look's own frame (device_cue.cue_level), or None."""
        img = rec.get("img")
        if img is None:
            _, look = self._look_rec(key)
            frame = None if look is None else look.get("frame")
            if not (isinstance(frame, str) and os.path.exists(frame)):
                return None
            img = np.asarray(Image.open(frame).convert("RGB"))
        r_px = max(3.0, 0.5 * across) if across is not None else 6.0
        return cue.cue_level(np.asarray(img), geom["u"], geom["v"], r_px)

    def _dv_unobservable(self, claim, key, check):
        w = self.world
        evidence = str(claim.get("evidence") or "")
        if claim.get("id") is not None:
            e = w.entries.get(str(claim["id"]))
        else:
            r = w.claim_state(claim["device"], claim["state"], sense=claim.get("from_sense"), evidence=evidence,
                              reuse=True)
            e = w.entries.get(r.get("id")) if "refused" not in r else None
        verdict = "contradicted" if check.get("reason") == "contradicted" else "unobservable"
        out = {"verdict": verdict, "accepted": False, "look_id": key, **{k: v for k, v in check.items()}}
        if e is not None and e.get("kind") == "state":
            why = f"{verdict}: {check['why']}"
            e["checks"].append({"sense": "look", "ref": key, "at_call": self._calls, "evidence": evidence,
                                "accepted": False, "why": why})
            out.update(claim_id=e["id"], device=e["device"], state=e["state"], confirmed=e["confirmed"],
                       mark=e["mark"])
        self._journal("device_verify", claim_id=out.get("claim_id"), look_id=key, verdict=verdict,
                      reason=check["reason"])
        return out

    def _dv_was_confirmed(self, claim):
        """True when the claim's record stood confirmed before this call (only a reading of the robot's own confirms
        one: the design rulings)."""
        if claim.get("id") is not None:
            e = self.world.entries.get(str(claim["id"]))
            return bool(e and e.get("kind") == "state" and e.get("confirmed"))
        got = self.world.device_states().get(str(claim.get("device"))) if claim.get("device") else None
        return bool(got and got.get("confirmed") and got.get("state") == claim.get("state"))

    def _dv_unchecked(self, out, before, said):
        """the design rulings: a claim the robot did not read itself is recorded and NOT confirmed: accepted false, the record
        unconfirmed, the said line names what the robot read, what it could not check and what would check it. A
        record a reading of the robot's own confirmed before stays confirmed by that reading."""
        out["robot_checked"] = bool(before)
        if not (out.get("confirmed") and out.get("accepted")):
            out["verdict"] = "not_confirmed"
            return out
        if before:
            out.update(verdict="confirmed", basis="the robot's own reading of an earlier call",
                       said="Confirmed earlier by the robot's own reading; this call's evidence was not compared with "
                            "the state.")
            return out
        e = self.world.entries.get(str(out.get("claim_id")))
        if e is not None:
            e.update(confirmed=False, mark=MARK_UNCONFIRMED, confirmed_by=None)
            if e.get("checks"):
                e["checks"][-1].update(accepted=False, why=NOT_ROBOT)
        for k in ("basis", "by", "first_confirmed_by"):
            out.pop(k, None)
        out.update(verdict="not_checked", accepted=False, confirmed=False, mark=MARK_UNCONFIRMED, said=said)
        self._journal("device_verify", claim_id=out.get("claim_id"), verdict="not_checked", reason="no_robot_reading")
        return out

    def _dv_own_look(self, device, state):
        """the design rulings: the check that costs no motion. When the state is a lit or dark word of a device with a cue
        part, the robot takes one look of its own camera as it stands (0 control periods) and reads the cue in it.
        (look_id, the check of _dv_check) or (None, None) when no such look can read a cue (no cue part, no lit or
        dark word, the newest light reading dark)."""
        entry = de.find(self, device) if device else None
        if entry is None or str(state or "").lower() not in LIT_STATES + DARK_STATES or not de.cue_parts(entry):
            return None, None
        dark = getattr(self, "_dark_refusal_now", None)
        if dark is not None and dark() is not None:
            return None, None
        lk = self._look_impl("verify: the robot's own look at the cue", keep_image=True)
        key, rec = self._dv_evidence_look(lk["look_id"])
        if rec is None or rec.get("can_see") == "dark":
            return key, {"reason": "dark", "why": f"its own look {key} reads dark"}
        return key, self._dv_check(entry, state, key, rec)

    def _dv_by_own_look(self, claim, device, state):
        """The answer when the robot's own look read the cue (confirmed or contradicted), else (None, words of what
        its own look could not read, or "")."""
        key, check = self._dv_own_look(device, state)
        if key is None:
            return None, ""
        if check.get("reason") is None and check.get("tested") == "cue_read":
            routed = {k: v for k, v in claim.items() if k not in ("look_id", "by_sense", "ref", "evidence")}
            out = self._verify_impl({**routed, "look_id": key, "evidence": "the robot read the cue's light itself"})
            if "refused" in out:
                return None, f" Its own look {key} read the cue, and the claim was not taken: {out['refused'].get('why')}."
            out["evidence_checked"] = {"checked": True, **check, "means": "the cue's light was read at the point in "
                                       "the robot's own look and agrees with the state"}
            ok = bool(out.get("confirmed") and out.get("accepted"))
            out.update(own_look=key, robot_checked=ok, verdict="confirmed" if ok else "not_confirmed")
            if ok:
                out.update(basis="the robot's own reading", said=f"Confirmed by the robot: it took look {key} itself "
                           "(0 control periods) and read the cue's light; it agrees with the state.")
            return out, ""
        if check.get("reason") == "contradicted":
            return self._dv_unobservable({**claim, "look_id": key}, key, check), ""
        return None, f" Its own look {key} (0 control periods) could not read the cue: {check.get('why')}."

    @primitive
    def verify_evidence(self, **claim):
        """verify with the evidence look checked first (served as the verify tool by HANDLERS). Only a reading of
        the robot's own confirms a claim: the given look's cue, or the cue in a look the robot takes
        itself where that costs no motion."""
        look_id = claim.get("look_id")
        if look_id is None and claim.get("by_sense") == "look":
            look_id = claim.get("ref")
        named = claim.get("id") is not None or (claim.get("device") and claim.get("state"))
        two_routes = claim.get("look_id") is not None and claim.get("by_sense") is not None   # round one refuses
        before = self._dv_was_confirmed(claim)
        device, state = claim.get("device"), claim.get("state")
        if claim.get("id") is not None:
            e = self.world.entries.get(str(claim["id"]))
            if e is not None and e.get("kind") == "state":
                device, state = e["device"], e["state"]
        rec = None
        if look_id is not None and str(claim.get("evidence") or "").strip() and named and not two_routes:
            key, rec = self._dv_evidence_look(look_id)
        if rec is not None and rec.get("verdict") == "unobservable" and rec.get("reason") == "dark":
            return refusal("can_see", f"watch {key} was too dark to read the cue: it cannot decide a state",
                           "get light on the scene or wait for light, then watch or look again")
        if rec is None or rec.get("can_see") == "dark":
            out = self._verify_impl(claim)        # the round one rules decide (and refuse) as before
            if "refused" in out:
                return out
            out["evidence_checked"] = {"checked": False, "why": "no look of this session to check (a second "
                                                                "sense, or an unknown look)"}
            out = self._dv_unchecked(out, before, NOT_CHECKED_SENSE)
            if not out.get("confirmed") and named and not two_routes:
                own, words = self._dv_by_own_look(claim, device, state)
                if own is not None:
                    return own
                if words and out.get("said"):
                    out["said"] += words
                elif words:
                    out["own_look"] = words.strip()
            return out
        if claim.get("id") is not None:
            e = self.world.entries.get(str(claim["id"]))
            if e is None or e.get("kind") != "state":
                return self._verify_impl(claim)
        entry = de.find(self, device) or {"id": str(device), "parts": []}
        check = self._dv_check(entry, state, key, rec)
        words = ""
        if check["reason"] is not None and check["reason"] != "contradicted" or \
                check["reason"] is None and check.get("tested") != "cue_read" and not before:
            own, words = self._dv_by_own_look(claim, device, state)
            if own is not None:
                return own
        if check["reason"] is not None:
            out = self._dv_unobservable(claim, key, check)
            if words:
                out["own_look"] = words.strip()
            return out
        out = self._verify_impl(claim)
        if "refused" in out:
            return out
        out["evidence_checked"] = {"checked": True, **check,
                                   "means": ("the cue's light was read at the point in the look and agrees with the "
                                             "state" if check.get("tested") == "cue_read" else
                                             "the point projects into the look, clear of the robot, after the last "
                                             "act; the picture was not compared with the state")}
        if check.get("tested") == "cue_read":
            ok = bool(out.get("confirmed") and out.get("accepted"))
            out.update(verdict="confirmed" if ok else "not_confirmed", robot_checked=ok)
            if ok:
                out.update(basis="the robot's own reading", said=f"Confirmed by the robot: it read the cue's light in "
                           f"{key} and it agrees with the state.")
            return out
        return self._dv_unchecked(out, before, NOT_CHECKED_LOOK.format(look=key) + words)


# What verify says when the robot did not read the state itself: what it read, what it could not
# check, and what would check it. A claim is recorded, never refused for it, and stays unconfirmed.
NOT_ROBOT = "not checked by the robot: the state rests on the operator's evidence alone"
NOT_CHECKED_LOOK = ("Not checked by the robot, and not confirmed: it read that the deciding point lies in {look}, "
                    "clear of the robot, after the last act; it did not compare the picture with the state. A cue of "
                    "the device read in a look would check it.")
NOT_CHECKED_SENSE = ("Not checked by the robot, and not confirmed: no look of this session was given to check, and the "
                     "sense you cite is recorded as you state it. A cue of the device read in a look would check it.")
