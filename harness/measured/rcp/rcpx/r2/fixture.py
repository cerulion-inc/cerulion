# SPDX-License-Identifier: AGPL-3.0-only
"""Package fixture (SPEC-R4 B5; round six, the design notes package PLACE stage C): working a fixed part in one
step, with no model of any device.

One tool (Mixin primitive): work_part(target, way: pull | push | turn, until: "end" | {"mm": n} | {"deg": n},
sense: cw | ccw | None, grasp: auto | across | pinch | hook | fingers, direction, max_n, gentle, clear).
Phases, each reported with its control periods and its binding bound:
1. find       the target is a spot (a pixel of a look through place_point.resolve role part, a located label, a
              point): a point with what is known of its error. Nothing more is asked of it. The direction
              the way implies: the one the call states; else the part's stored path where this part was worked
              before; else square to the face it stands on; else, for a part a look shows longer than wide,
              square to its long side, into the thing the look shows behind it for a push and out of it for a
              pull (rcpx/r2/fixture_look.py side), and where the look tells no side, away from the robot or
              toward it; else the level line between the robot's base and the part (a part on the robot's own
              base axis: up);
2. take_hold  the best ranked hand pose in reach (rcpx/r2/fixture_hold.py: poses clear of every MEASURED band
              first; a pose that passes inside one is taken and said; for a push, the hold where the hand, moved
              along the stroke, clears what the newest look shows over the part until the part comes to the face
              it goes into, the nearest the pointed point; where none does, the hold as solved and the receipt
              says what the hand would meet and after how far). ONE stream to the pose before the part
              and on to the hold: its last leg is an expected contact with the part (C9), bounded only by a limit
              somebody stated (C3). The jaws close where it stops through the core's one close
              (rcpx/control_hold.py), the hold on a part of something fixed. A pose that meets something on the
              way in gives way to the next one, ranked again from where the arm then stands, once;
3. probe      one continuous force ramp along the direction until the tip's path passes rigid (no threshold);
4. follow     the part's own fitted line or arc, compliant across it, by the follow law of A4.5
              (rcpx/r2/fixture_follow.py); a turn is the same about the tool axis, in strokes where the posture
              cannot turn the whole angle clear of every band;
5. let_go     the jaws open through the core's release, the hand backs off the way it came in; the part's path,
              the force that moved it and the end it found are stored on its world entry.
The result leads with one sentence; then way, travel_mm or angle_deg, axis, moved_from_n, ended (end | amount |
link_contact | slipped | refused), moved, phases, periods_used, bounds, strike, doubt, next. A part that moved less
than rigid is never reported as moved. What may refuse: a malformed argument, no gripper for a pull or a turn, no
hand pose in this arm's reach, a stated limit that cannot be kept, the budget. Never doubt. Nothing here reads
truth, names a robot or a device, or holds a typed limit.
"""

import math

import numpy as np

from rcpx import bounds_laws as laws
from rcpx import bounds_report as report
from rcpx.bounds_ratios import BAND, QUANT
from rcpx.context import refusal
from rcpx.control_base import primitive
from rcpx.r2 import fixture_follow as F
from rcpx.r2 import fixture_hold as H
from rcpx.r2 import fixture_probe as P
from rcpx.r2.fixture_probe import unit
from rcpx.r2.fixture_say import SayMixin
from rcpx.r2.fixture_take import TakeMixin
from rcpx.r2.fixture_tools import PRACTICES, TOOLS
from rcpx.r2.place_hand import Jaws

ID = "R4-fixture"
WAYS = ("pull", "push", "turn")
GRASPS = ("auto", "across", "pinch", "hook", "fingers")
READY = "work_part(target=[x, y, z], way='pull')"


def _bad(check, why, hint=READY):
    return refusal(check, why, hint)


def _mm(p):
    return [round(float(x) * report.MM, 1) for x in p]


def _first_met(tried):
    """What the first way in that moved said (a pose the bounds refused before it moved said nothing)."""
    return next((t["why"] for t in tried if t["why"] not in (None, "refused")), "a way in ended before its pose")


class Mixin(SayMixin, TakeMixin):
    # ---------------------------------------------------------------- arguments
    def _fx_args(self, target, way, until, sense, grasp):
        if way not in WAYS:
            return None, _bad("way", f"way must be pull, push or turn, got {way!r}")
        if grasp not in GRASPS:
            return None, _bad("grasp", f"grasp must be one of {', '.join(GRASPS)}, got {grasp!r}")
        if sense not in (None, "cw", "ccw"):
            return None, _bad("sense", f"sense must be cw or ccw (as the hand sees the part), got {sense!r}",
                              "work_part(target=[x, y, z], way='turn', until={'deg': 90}, sense='cw')")
        amount = None
        if until != "end":
            key = "deg" if way == "turn" else "mm"
            try:
                amount = float(until[key])
                ok = math.isfinite(amount) and amount > 0 and len(until) == 1
            except (TypeError, KeyError, ValueError):
                ok = False
            if not ok:
                return None, _bad("until", f"until must be \"end\" or {{\"{key}\": n}} with n above zero for way "
                                  f"{way}, got {until!r}", f"work_part(target=[x, y, z], way='{way}', until='end')")
            amount = math.radians(amount) if way == "turn" else amount / report.MM
        if self._fx_jaws is None and way != "push":
            return None, _bad("no_gripper", "this robot has no gripper: it cannot take hold of a part to pull or "
                              "turn it", "work_part(target=[x, y, z], way='push')")
        return amount, None

    def _fx_find(self, target, intent, way=None):
        """The part of a target: (part, None), or (None, a refusal for a malformed target)."""
        seen = None
        if isinstance(target, dict) and "kind" in target and "top_z" in target:
            seen = target
        elif isinstance(target, dict):
            try:
                from rcpx.r2 import place_point
                seen = place_point.resolve(self, target, "part", intent=intent)
            except Exception as exc:      # the pointing package is not in this build, or it failed
                return None, _bad("target", f"this pixel could not be resolved ({type(exc).__name__}: {exc})",
                                  "give the part as a point: " + READY)
            if isinstance(seen, dict) and "refused" in seen:
                return None, seen
        answer = (seen.get("answer") or (seen.get("spot") or {}).get("answer")) if isinstance(seen, dict) else None
        if isinstance(answer, dict):
            # the look shows nothing at that pixel (contract C11, the design rulings): the tool answers at once what
            # the look shows near it, with no motion and no control period; it is no refusal. The same for a
            # resolved spot handed in as the target that carries the answer
            near = list(answer.get("near") or [])
            out = {"said": "The look shows no part at that pixel: nothing was moved. "
                   + (f"Near it the look shows {len(near)} thing(s): point at one of them."
                      if near else "It shows nothing near it either: take a new look.")
                   + (f" The look's own picture comes back marked ({answer['drawn']}), with a crop of it around your "
                      f"{answer.get('of', 'pixel')} enlarged {answer['crop_scale']:.1f} times (0 control periods)."
                      if answer.get("marked") and answer.get("drawn") and answer.get("crop_scale") else ""),
                   "stopped": "answered", "moved": False, "why": answer.get("why"), "near": near,
                   "periods_used": 0}
            for k in ("marked", "crop", "crop_box_px", "pictures_why"):     # the look's help, as a put's
                if answer.get(k) is not None:                               # answered receipt carries it
                    out[k] = answer[k]
            return None, out
        part, bad = H.part_of(self, target, seen)
        if bad:
            return None, bad
        # The target is a spot (C11, role part): a point with what is known of its error. Nothing more is asked of
        # it: the way in's last leg is an expected contact with it (C9), and what is not known of it is a
        # line of the receipt's doubt.
        part["spot"] = seen.get("spot") if isinstance(seen, dict) else None
        part["doubt"] = list(seen.get("doubt") or []) if isinstance(seen, dict) else []
        # a part that the look shows at the top of the thing it belongs to (within one pad of that top) is worked
        # from above; one on a thing's side is worked from the side
        e = (part["spot"] or {}).get("extent") or {}
        J = self._fx_jaws
        reach = float(J.pad_height_m or J.max_m) if J is not None else float(H.bounds_of(self).robot.w_tool())
        top = e.get("top_near_z") if e.get("kind") == "thing" else None
        part["on_top"] = bool(top is not None and float(top) - float(part["p"] @ H.up_of(self)) <= reach)
        if way == "turn" and J is not None:
            # a part that is turned from above turns about its own middle: the hand goes over the middle of what
            # the look shows standing highest about the pointed point, at the height of its top, not to the
            # pointed point (a hold off the part's axis is pushed aside as it turns; a hold at the height of a
            # pixel on the part's side or foot puts the fingers into what the part stands on). A pixel anywhere
            # on the part works it (the design rulings, f): the hold is taken where this hand fits the part nearest the
            # pixel: on what stands highest within half the jaws' opening of the point, where the fingers reach
            # from that top down to the point (their own length); said with the distance. Where the point does
            # not lie under that top the look's outline at the point is of another piece of the part: the jaws
            # then come in as wide as they go
            got = H.part_top(self, part["p"], J.max_m / laws.TWO, part["sigma_m"])
            rise = None if got is None else float(got["top_h"]) - float(part["p"] @ H.up_of(self))
            if got is not None and (part["on_top"] or got["covers"] or rise <= float(J.finger_len_m or J.max_m)):
                self._fx_onto_top(part, got)
            elif got is not None:
                part["top_alt"] = got       # the fingers do not reach from that top to the point: the second way
        return part, None

    def _fx_onto_top(self, part, got):
        """The part is worked from above, about the middle and at the top of what the look shows standing
        highest about the pointed point (fixture_look.part_top); said with the distance. Where the point does
        not lie under that top, the look's outline at the point is of another piece of the part and sizes
        nothing."""
        J = self._fx_jaws
        rise = float(got["top_h"]) - float(part["p"] @ H.up_of(self))
        if not got["covers"]:
            part.update(width_m=None, across=None, bearing=None, long=False, wide_m=None)
        part["on_top"], part["p"] = True, np.asarray(got["top"], dtype=float)
        part["doubt"].append(f"the part is turned about its own middle as look {got['look']} shows it from above "
                             f"({got['wide_m'] * report.MM:.0f} mm wide, {got['off_m'] * report.MM:.0f} mm from the "
                             f"point you gave" + (f", its top {rise * report.MM:.0f} mm over that point"
                                                  if rise > float(J.pad_height_m or J.max_m) else "") + ")")

    def _fx_stored(self, part):
        """The stored entry of the part worked before at this spot (within the two position errors and the jaws'
        own opening of where its held point came to rest), or None."""
        J = self._fx_jaws
        span = J.max_m if J is not None else H.bounds_of(self).robot.w_tool()
        best = None
        for e in self.world.query(kind="located", include_stale=False):
            data = e.get("data") or {}
            if not data.get("fixture") or e.get("point_mm") is None:
                continue
            dist = float(np.linalg.norm(np.asarray(e["point_mm"], dtype=float) / report.MM - part["p"]))
            if dist <= span + part["sigma_m"] + float(data.get("sigma_p_mm") or 0) / report.MM:
                if best is None or dist < best[0]:
                    best = (dist, e)
        return None if best is None else best[1]

    def _fx_direction(self, part, way, stored):
        """(unit direction the part should move or, for a turn, the direction the hand points; the words for it;
        metres of path already moved through that way or None)."""
        p = part["p"]
        up = H.up_of(self)
        level = unit(p - float(p @ up) * up)
        if part.get("direction") is not None:    # the caller's own word: the way the part moves (a turn: the hand's axis)
            return np.asarray(part["direction"], dtype=float), "the direction you stated", None
        if stored is not None:
            data = stored["data"]["fixture"]
            axis = np.asarray(data["direction"], dtype=float)
            if way == "turn":
                return unit(np.asarray(data.get("hand_axis", axis), dtype=float)), "the axis it turned about before", None
            if data.get("shape") in ("line", "arc"):
                same = (way == data["way"])
                known = None if same else float(data.get("travel_mm") or 0) / report.MM
                return (axis if same else -axis), "this part's own stored path", known
        J = self._fx_jaws
        normal, _ = H.face_normal(self, p, J.max_m if J is not None else H.bounds_of(self).robot.w_tool())
        if normal is not None:                   # the part stands on a face of a thing: it is worked square to it
            if way == "turn":
                return -normal, "into the face the part stands on", None
            return (normal, "out of the face it stands on", None) if way == "pull" else \
                (-normal, "into the face it stands on", None)
        if way == "turn" and part.get("on_top"):
            return -up, "down, into the top of the thing the look shows it on", None
        if way == "turn":
            return (level if np.any(level) else -up), "level, away from the robot (no face is known here)", None
        if not np.any(level):       # the part stands on the robot's own base axis: no level line to it; it goes up
            return (up, "up (the part stands on the robot's own base axis: no level direction to it)", None) \
                if way == "pull" else (-up, "down (the part stands on the robot's own base axis)", None)
        if part["bearing"] is not None and part.get("long"):     # a seen long side: the part is worked square to it
            across = unit(np.cross(up, part["bearing"]))
            half = (part["width_m"] or 0.0) / laws.TWO + BAND * part["sigma_m"]
            box = H.finger_box(self, J, 0) if J is not None else None      # right beside the part: the room the
            room = QUANT * (box[1] if box is not None else H.bounds_of(self).robot.w_tool())   # fingers stand in
            side, look = H.L.side(self, p, across, half, half + room)
            if side:      # the look shows on which side of the part the thing it belongs to stands: a push goes
                into = side * across                     # into that thing, a pull draws the part out of it
                return (-into, f"square to its long side, out of what look {look} shows behind it", None) \
                    if way == "pull" else (into, f"square to its long side, into what look {look} shows behind it",
                                           None)
            whole = part.get("whole_across")
            if whole is not None:
                # the chord through the point tells no side (a pixel near a bar's end reads its narrowest chord
                # along the bar): the same reading across the whole thing's outline, so that a handle pointed
                # near its end is not pushed across the part's own way
                side, look = H.L.side(self, p, whole, half, half + room)
                if side:
                    into = side * whole
                    return (-into, f"square to the long side of its outline, out of what look {look} shows behind "
                            "it", None) if way == "pull" else (
                        into, f"square to the long side of its outline, into what look {look} shows behind it", None)
            square = unit(level - float(level @ part["bearing"]) * part["bearing"])
            level = square if np.any(square) else level
        return (-level, "toward the robot", None) if way == "pull" else (level, "away from the robot", None)

    # ---------------------------------------------------------------- phases (take_hold and let_go: fixture_take.py)
    def _fx_store(self, part, stored, way, d, rec, hand_axis=None):
        """The part's world entry after a stroke: where its held point rests now, its path, the force that moved
        it, the end it found. A later call at that spot reads it (the stroke back runs at free speed)."""
        s = self._sense()
        fit = rec.get("fit") or {}
        data = {"way": way, "direction": [float(x) for x in d], "shape": fit.get("shape"),
                "travel_mm": float(rec.get("travel_m") or 0) * report.MM, "start_mm": _mm(rec.get("start_tip", s.tip)),
                "ended": rec["stopped"]}
        if hand_axis is not None:
            data.update(hand_axis=[float(x) for x in hand_axis], angle_deg=math.degrees(rec.get("angle_rad") or 0))
        if fit.get("shape") == "arc":
            data.update(centre_mm=_mm(fit["centre"]), radius_mm=fit["radius_m"] * report.MM,
                        plane_normal=[float(x) for x in fit["axis"]])
        B = H.bounds_of(self)
        extra = {"fixture": data, "touched": True,
                 "sigma_p_mm": max(part["sigma_m"], B.rigid(s.q, np.asarray(d, dtype=float))) * report.MM}
        if rec.get("f_moved_n") is not None:
            extra.update(f_moved_n=float(rec["f_moved_n"]), f_worked_n=float(rec["f_moved_n"]))
        if rec.get("mass_kg") is not None:
            extra["mass_kg"] = float(rec["mass_kg"])
        if rec["stopped"] == "end":
            extra["end_mm"] = _mm(s.tip)
        point = _mm(part["p"] + (s.tip - rec.get("start_tip", s.tip))) if rec.get("moved") and way != "turn" else _mm(part["p"])
        eid = stored["id"] if stored is not None else (part["entry"] or {}).get("id")
        if eid is not None:
            self.world.update(eid, point_mm=point, data=extra, source="work_part: moved with the part")
            return eid
        made = self.world.add("located", "worked part", point_mm=point, source="work_part", data=extra)
        return made.get("id") if isinstance(made, dict) else None

    # ---------------------------------------------------------------- the tool
    @primitive
    def work_part(self, target=None, way=None, until="end", sense=None, grasp="auto", clear=None, direction=None,
                  max_n=None, gentle=None):
        amount, bad = self._fx_args(target, way, until, sense, grasp)
        if bad:
            return bad
        stated, bad = H.direction_of(direction)
        if bad:
            return bad
        if max_n is not None and not (isinstance(max_n, (int, float)) and math.isfinite(max_n) and max_n > 0):
            return _bad("max_n", f"max_n is the largest force the part may see, in newtons, got {max_n!r}",
                        "work_part(target=[x, y, z], way='pull', max_n=20)")
        B = H.bounds_of(self)
        self._fx = {"phases": [], "doubt": [], "direction": stated}
        if not hasattr(self, "_call_begin"):
            B.begin("call")
        intent = {"clear": clear} if clear else {}
        word = None
        if intent:               # the word is information: refused only for a look this session does not hold
            word = B.word(intent)
            if word.get("refused"):
                bad = word["refused"]
                return bad if isinstance(bad, dict) and "refused" in bad else {"refused": bad}
        if max_n is not None:
            intent["max_n"] = float(max_n)
        if gentle:               # a limit on any call: the hand's own weight, kept and priced as max_n is
            intent["gentle"] = True
        try:
            out = self._fx_run(target, way, amount, sense, grasp, intent)
            if isinstance(out, dict) and "refused" not in out and word and word.get("given"):
                out["word"] = {"given": True, "look": word.get("look"), "note": word.get("text")}
            return out
        finally:
            ender = getattr(B.space, "end_call", None)
            if ender is not None and not hasattr(self, "_call_end"):
                ender()
            g = self.backend.read().get("gripper")
            if isinstance(g, dict) and g.get("q") is not None and hasattr(self.model, "set_jaws"):
                self.model.set_jaws(g["q"])

    def _fx_run(self, target, way, amount, sense, grasp, intent, from_top=False):
        B, J = H.bounds_of(self), self._fx_jaws
        n_start = self._periods_now()
        # ---- 1. find
        F.begin(self, "find")
        part, bad = self._fx_find(target, intent, way)
        if bad:
            return bad
        if from_top and part.get("top_alt") is not None:
            # no hand pose from the side has a plan (the design rulings: a plan that exists is taken, the tool does not refuse):
            # the look shows a part's top within half the jaws' opening of the point, and it is worked from above
            self._fx_onto_top(part, part.pop("top_alt"))
            part["doubt"].append("no hand pose that comes at the point from the side has a plan: the part is "
                                 "worked from above")
        from_above = (lambda: way == "turn" and not from_top and part.get("top_alt") is not None
                      and self._periods_now() == n_start)
        part["direction"] = self._fx.get("direction")
        for line in part.get("doubt") or []:
            self._fx["doubt"].append({"what": str(line), "acted": "went to the point with the contact stop armed",
                                      "would_remove": (part.get("spot") or {}).get("remove_by")})
        if part.get("stale"):
            self._fx["doubt"].append(part["stale"])
        stored = self._fx_stored(part)
        d, d_words, known = self._fx_direction(part, way, stored)
        if stored is not None and part["owner"] is None:
            part["owner"] = stored["id"]
        F.end(self, f"the part at {_mm(part['p'])} mm; direction {d_words}"
              + ("" if stored is None else "; worked before: its stored path is used"))
        # ---- 2. take hold
        F.begin(self, "take_hold")
        s = self._sense()
        reach = (J.finger_len_m or J.max_m) if J is not None else B.robot.w_tool()
        standoff = reach + B.standoff(0, part["thing"], s.q, d)
        if way == "turn":
            asked = amount if amount is not None else math.pi
            cands = H.turn_candidates(self, part, d, grasp)
            choices, why = H.rank(self, part, cands, d, 0.0, standoff, turn=asked)
            sign = 1 if sense != "ccw" else -1
            if choices and choices[0]["turn_clear"][sign] < asked and choices[0]["turn_clear"][-sign] > 0:
                # the posture turns clear of every band over a window: begin the stroke at the window's far edge
                best = choices[0]
                R2 = P.rot(-sign * best["R"][:, 0], best["turn_clear"][-sign]) @ best["R"]
                shifted = dict(best, R=R2, p_base=part["p"] - R2 @ J.gap_centre_T(best["open"]))
                more, _ = H.rank(self, part, [shifted], d, 0.0, standoff, turn=asked)
                if more and more[0]["turn_clear"][sign] > best["turn_clear"][sign]:
                    choices = more + choices
        else:
            cands = H.candidates(self, part, way, d, grasp)
            length = amount if amount is not None else self._fx_stroke(part, d, known)
            choices, why = H.rank(self, part, cands, d, length, standoff, tool_free=(way == "push"))
        if not choices:
            F.end(self, "no hand pose in reach")
            if from_above():
                self._fx.update(phases=[], doubt=[])
                return self._fx_run(target, way, amount, sense, grasp, intent, from_top=True)
            return H.no_posture(why, way)
        self._journal("fixture", stage="choices", count=len(choices), best=[
            {"grasp": c["name"], "tilt_deg": c["tilt_deg"], "open": round(c["open"], 3),
             "depth_mm": round(c["depth_m"] * report.MM, 1), "clear": round(float(c["stroke"]["clear"]), 4),
             "reach": round(float(c["stroke"]["reach"]), 4), "least_mm": round(float(c["room_m"]) * report.MM, 1),
             "joint_travel_rad": round(-float(c["score"][-1]), 3), "block": c["stroke"]["block"]}
            for c in choices[:int(BAND * QUANT)]])
        tried, taken, choice, refusals, moved_tries = [], None, None, [], 0
        while True:
            choice = choices[0]
            taken = self._fx_take(part, choice, intent)
            self._journal("fixture", stage="take", grasp=choice["name"], tilt_deg=choice["tilt_deg"], ok=taken["ok"],
                          why=taken["why"], held=taken["held"], stopped=taken["stream"]["stopped"],
                          contact=taken["stream"].get("contact"), tip_mm=_mm(self._sense().tip),
                          hold_mm=_mm(choice["p_hold"]), passes=taken.get("said"))
            tried.append({"grasp": choice["name"], "tilt_deg": choice["tilt_deg"], "ok": taken["ok"],
                          "why": taken["why"] if isinstance(taken["why"], (str, type(None))) else "refused",
                          "passes": taken.get("said")})
            bad = taken["why"].get("refused", taken["why"]).get("check") if isinstance(taken["why"], dict) else None
            if bad not in (None, "budget", "limit_not_keepable") and len(choices) > 1 and not F.spent(self):
                # the bounds refused THIS pose's plan before it moved (a link through a measured thing, a reach):
                # no plan exists for this pose; the next ranked pose has a plan of its own
                refusals.append(taken["why"])
                choices = choices[1:]
                continue
            if taken.get("said"):
                self._fx["doubt"].append({"what": f"the {choice['name']} pose: {taken['said']}",
                                          "acted": "took it with the contact stop armed",
                                          "would_remove": {"tool": "look", "args": {}, "periods": 0}})
            if moved_tries and bad == "budget":
                # the next ranked pose was counted against the budget before it started and does not fit: the
                # call ends with what the first way in met, never with a refusal after the hand has moved
                why = taken["why"].get("refused", taken["why"]).get("why")
                taken = dict(taken, why=f"{_first_met(tried)}; the next pose ({choice['name']}) was counted before it "
                             f"started and does not fit the budget ({why})", stood_back=True)
                tried[-1]["why"] = "not started: the budget"
                break
            if taken["ok"] or isinstance(taken["why"], dict) or F.spent(self) or moved_tries + 1 >= int(QUANT):
                break
            moved_tries += 1
            self._fx_let_go(part, choice, intent)
            # the poses are ranked again from where the arm stands now (what the way in met is named in the
            # receipt, not stored: fixture_say._fx_met), and the pose that met it is not tried again
            if way == "turn":
                again, _ = H.rank(self, part, cands, d, 0.0, standoff, turn=asked)
            else:
                again, _ = H.rank(self, part, cands, d, length, standoff, tool_free=(way == "push"))
            again = [c for c in again if not (c["name"] == choice["name"] and c["tilt_deg"] == choice["tilt_deg"]
                                              and np.allclose(c["R"], choice["R"]))]
            if not again:
                break
            choices = again
        self._fx["take"], take_item = taken.get("held"), taken["stream"].get("bounds")
        F.end(self, f"{choice['name']} (the hand tilted {choice['tilt_deg']} deg), "
              + ("holding" if taken["ok"] and choice["closes"] else "touching" if taken["ok"] else "not taken"),
              take_item, taken["stream"].get("free_share"))
        if isinstance(taken["why"], dict) and moved_tries:
            # the hand has moved: a way in met something and the poses ranked after it have no plan from where
            # the hand stands: the call ends with what the first way in met, never with a refusal after the hand
            # has moved (as for the budget above)
            bad = taken["why"].get("refused", taken["why"])
            taken = dict(taken, why=f"{_first_met(tried)}; no other pose has a plan from where the hand stands "
                         f"({bad.get('check')}: {bad.get('why')})", stood_back=True)
        if isinstance(taken["why"], dict):          # no pose has a plan: the first pose's refusal says why
            bad = (refusals + [taken["why"]])[0]
            if from_above() and bad.get("refused", bad).get("check") in ("reach", "path_plane"):
                self._fx.update(phases=[], doubt=[])
                return self._fx_run(target, way, amount, sense, grasp, intent, from_top=True)
            return bad if "refused" in bad else {"refused": bad}
        if not taken["ok"]:
            if not taken.get("stood_back"):
                F.begin(self, "let_go")
                back = self._fx_let_go(part, choice, intent)
                F.end(self, (back.get("did") or "backed off") + ": no hold was taken", self._fx_out_item(back),
                      back.get("free_share"))
            way_in = self._fx_way_in_said()
            return self._fx_result({"said": f"Did not take hold of the part: {taken['why']}"
                                    + (f"; {way_in}" if way_in and "ended on a contact" not in str(taken["why"])
                                       else "") + ".", "way": way,
                                    "moved": False, "ended": "not_taken", "stopped": "not_taken", "tried": tried,
                                    "next": "point at the part again in a new look, or state another grasp"}, [take_item])
        # ---- 3 and 4. probe and follow
        if way == "turn":
            return self._fx_turn(part, stored, choice, d, amount, sense, intent, tried, take_item)
        F.begin(self, "probe")
        s = self._sense()
        fs = B.force("work_part", part["thing"], s.q, d, intent)
        play = (J.width_m(J.q_of_frac(choice["open"])) if choice["closes"] else
                B.travel("touch", part["thing"], s.q, d).past_m + standoff)
        rec = F.follow(self, part, choice, d, amount, intent, fs, known_m=known,
                       hold_r=None if choice["closes"] else choice["R"], play_m=play)
        # did the hand stand on something as the stroke began (its way in ended on a contact, its jaws held)?
        rec["stood_on"] = bool(taken.get("met") or (taken.get("held") or {}).get("held"))
        if not choice["closes"]:
            # a hand that pushes or hooks stands on the part as the stroke ends only where the stroke itself read
            # the part to its end (over what the still arm tells): the leave is told only then (_fx_stand)
            felt = rec["state"] != "seek" and rec["stopped"] != "slipped" and rec["peak_n"] > rec["level_n"]
            self._fx["bore"] = f"the stroke read {rec['peak_n']:.1f} N along the way" if felt else None
        F.end(self, "ramp and follow", rec["item"], rec["free_share"])
        self._fx["phases"][-1]["phase"] = "probe_follow"
        self._fx_seen("the stroke", rec["peak_n"], f"the stroke read {rec['peak_n']:.1f} N along the way")
        self._fx_squeeze_seen(choice)
        # ---- 5. let go
        F.begin(self, "let_go")
        eid = self._fx_store(part, stored, way, rec["d"], rec)
        back = self._fx_let_go(part, choice, intent, rec["d"])
        F.end(self, back.get("did") or ("opened and backed off" if choice["closes"] else "backed off"),
              self._fx_out_item(back), back.get("free_share"))
        return self._fx_said(way, rec, fs, tried, eid, [rec["item"], fs.items[0], take_item, back.get("bounds")])

    def _fx_stroke(self, part, d, known):
        """The stroke an 'end' implies, for the posture check: the stretch already moved through where one is
        stored; else as far as the arm's own length along d."""
        if known:
            return known
        return float(self.model.length_scale)

    def _fx_turn(self, part, stored, choice, axis_in, amount, sense, intent, tried, take_item):
        B = H.bounds_of(self)
        signs = [1.0 if sense == "cw" else -1.0] if sense else [1.0, -1.0]
        total, rec, strokes, sign = 0.0, None, 0, signs[0]
        F.begin(self, "probe")
        room = choice.get("turn_clear") or {}
        if sense is None and amount is None:       # to its end, no sense stated: the sense that gives more
            rec, sign, strokes, found = F.turn_sense(self, part, choice, intent, room, lambda: (
                self._fx_let_go(part, choice, intent), self._fx_take(part, choice, intent)["ok"])[-1])
            if found:
                self._fx["doubt"].append({"what": found, "acted": "kept the sense that gives", "would_remove": None})
        else:
            for sign in signs:
                rec = F.turn(self, part, choice, sign, amount, intent, clear_rad=room.get(int(sign)))
                strokes += 1
                if rec["moved"] or rec["stopped"] != "refused":
                    break
        total, loaded = rec["angle_rad"], float(rec.get("loaded_rad") or 0.0)
        # a stroke that ended because the hand could no longer keep the height of its hold (the part would leave
        # the pads: fixture_follow.turn) is not taken up again by this call: the same pose on a part that has
        # turned under it rides the same way; the receipt says where it stands, and a new call with a new look
        # turns it on. A stroke that ended where the posture stays clear, or at a joint's end, is taken up again
        # A turn "to its end" that has come a whole turn round has found no end: the part stands where it began
        # (a knob with no stop was turned 20 times round until the hand met something: measured on the own bench)
        while rec["stopped"] == "posture" and (amount is None or amount - total > P.turn_resolved(B, [1.0])) \
                and rec["moved"] and not rec.get("hold_gave") and not F.spent(self):
            if amount is None and total >= laws.TWO * math.pi:
                rec = dict(rec, stopped="end", why="the part came a whole turn round and no end was found: it "
                           "turns without a stop this way")
                break
            gone = self._fx_let_go(part, choice, intent)      # a new stroke: let go and back off, turn back, take hold
            if gone["stopped"] != "goal":
                rec = dict(rec, stopped="contact", why="the way back for the next stroke was refused" if gone["stopped"]
                           == "refused" else "the hand met something as it backed off for the next stroke")
                break
            again = self._fx_take(part, choice, intent)
            self._journal("fixture", stage="take_again", ok=again["ok"], why=again["why"], held=again["held"],
                          stopped=again["stream"]["stopped"], tip_mm=_mm(self._sense().tip))
            if not again["ok"]:
                rec = dict(rec, stopped="slipped", why="the jaws did not find the part again")
                break
            more = F.turn(self, part, choice, sign, None if amount is None else amount - total, intent,
                          clear_rad=room.get(int(sign)))
            strokes += 1
            if more["angle_rad"] <= P.turn_resolved(B, [1.0]):     # nothing more gives: the part stands at its end
                rec = dict(rec, stopped="end", why=None)
                break
            total += more["angle_rad"]
            loaded += float(more.get("loaded_rad") or 0.0)
            rec = more
        F.end(self, f"torque ramp and turn in {strokes} stroke(s)", rec["item"], rec["free_share"])
        self._fx["phases"][-1]["phase"] = "probe_follow"
        if rec.get("along_axis_m") is not None:      # where the hand stands against the height of its hold
            self._fx["phases"][-1].update(off_hold_mm=round(float(rec["along_axis_m"]) * report.MM, 2),
                                          still_band_mm=round(float(rec.get("still_m") or 0.0) * report.MM, 2))
        self._fx_squeeze_seen(choice)
        F.begin(self, "let_go")
        rec_all = dict(rec, angle_rad=total, loaded_rad=loaded,
                       moved=bool(total > P.turn_resolved(B, [1.0]) and rec.get("moved", True) is not False))
        eid = self._fx_store(part, stored, "turn", np.asarray(rec["axis"], dtype=float), rec_all, hand_axis=axis_in)
        back = self._fx_let_go(part, choice, intent)
        F.end(self, back.get("did") or "opened and backed off", self._fx_out_item(back), back.get("free_share"))
        return self._fx_turn_said(rec, rec_all, total, sign, strokes, eid, tried, [rec["item"], take_item, back.get("bounds")])


def setup(ex):
    g = ex.model.spec.get("gripper")
    ex._fx_jaws = Jaws(ex.model) if g else None
    ex._fx = {"phases": []}
    ex.state["fixture"] = {"last": None}


__all__ = ["ID", "Mixin", "PRACTICES", "TOOLS", "setup"]
