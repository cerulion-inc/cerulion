# SPDX-License-Identifier: AGPL-3.0-only
"""Package approach: go to a point with a named tool surface, and know where you are.

Tools:
- state (and the resource robot://state): joint angles, the tip's position and roll/pitch/yaw, joint loads, the
  gripper opening, the world epoch and the last look with its age, all from the joint readings through the body
  model.
- approach(target, along, surface="tip", standoff_mm=None, keep_in_view=true, guarded=true): runs the reach check
  for the named surface first and refuses with its verdict (moving nothing) when the target is short; solves a joint
  pose that puts the surface at target - standoff * along with its outward normal along `along` (the face meets
  the stroke square) and, as a lower task, keeps the target in the wrist view (RobotModel.project); when no square
  pose passes the plane and self checks, facings tilted off along are tried and the tilt is reported. Then it moves
  there through one guarded stream (a straight tool line when it plans, else staged or joint legs), path
  checked by the motion core. A stopped stream has no fallback. The result states what was reached: the surface
  point, the standoff left, the facing error, what the
  wrist camera sees of the target, and the next stroke.
- look_after on move_tip, touch, press, orient_tool and approach: the step and its look in one call. The pre hook
  pops the argument; the post hook takes the look and returns it as result["look"].
- reach_check gains surface and point_source (touch, triangulated, single_look): the pre hook pops them, the round
  one check runs (so every other package's pre hook still sees the call), and the post hook replaces its tip verdict
  with the named surface's (the tip's kept as tip_verdict). A point from a single look that comes out unreachable
  reads "unknown: localise by touch or two views first".

The look taken for look_after is the round one look (_look_impl), given the post hooks other packages register
for "look" by name and core's look event, and costs no extra call. Hooks registered for every primitive run once,
on the operator's call that carries the look. No robot is named here; surfaces come from the robot spec's
tool_surfaces. Nothing here reads simulator ground truth.
"""

import math

import numpy as np

from rcpx.bounds_laws import TWO
from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx import surfaces as S
from rcpx.context import jsonable, refusal
from rcpx.control_base import _mm, primitive
from rcpx.r2.approach_kin import POS_TOL_M, angle_deg, surface_names
from rcpx.r2.approach_kin import turn_deg as angle_from
from rcpx.r2.approach_motion import ApproachMotion, motion_result, readable
from rcpx.r2.approach_resolve import (SOURCES, UNKNOWN_VERDICT, apply_point_source, resolve_along, resolve_target,
                                      state_doc, surface_kin, surface_reach, reach_past_m, reach_tol_m)

ID = "I21"
MM = 1000.0
LOOK_NAMES = ("move_tip", "touch", "press", "orient_tool")
# what round one's reach_check says about the tip; beside a surface verdict each is the surface's or null
TIP_KEYS = ("reachable", "margin_mm", "margin_along", "short_by_mm", "direction", "direction_name", "closest_mm",
            "why", "leading", "force_n", "q_deg")


def _own(fn):
    """Mark a package method that intentionally supplies the public state document."""
    fn._approach_own = True
    return fn


class Mixin(ApproachMotion):
    # ------------------------------------------------------------------ the embedded look (look_after)
    """Package primitives and helpers; private names are scoped to their package."""
    def _appr_look(self, after):
        """The round one look, with the post hooks registered for "look" by name (another package's look keys)
        and core's event hook (the look event); it ticks no call and journals as r2_look_after. A hook registered
        for every primitive (names None) is not run on it, so what such a hook adds to a separate look call (a
        pointer on a dark look) is not on this one: such a hook runs once on this operator call already, and a
        second run on the same call repeats what it counts or raises (an epoch raised twice for one cap stop)."""
        note = f"after {after}"
        try:
            lk = self._look_impl(note)
        except Exception as exc:  # the move stands; the operator learns the look did not happen
            self._journal("r2_problem", package="approach", where="look_after", trace=f"{type(exc).__name__}: {exc}")
            return refusal("look", f"the look after {after} failed: {type(exc).__name__}: {exc}",
                           "the move itself ran; call look for a picture")
        kwargs = {"note": note}
        core_events = getattr(self, "_core_events", None)
        for fn, names in list(getattr(self, "_post", [])):
            if getattr(fn, "_approach_own", False):
                continue
            if not ((names is not None and "look" in names) or (core_events is not None and fn == core_events)):
                continue
            guarded = getattr(self, "_guarded", None)
            r = guarded(fn, "post look (look_after)", "look", kwargs, lk) if guarded else fn("look", kwargs, lk)
            if isinstance(r, dict):
                lk = jsonable(r)
        self._journal("r2_look_after", after=after, look=jsonable(lk))
        return jsonable(lk)

    # ------------------------------------------------------------------ state
    @primitive
    def robot_state(self):
        """Read the packages state document from current senses and the world book."""
        return state_doc(self)

    # ------------------------------------------------------------------ reach_check with a named surface
    def _appr_reach_check(self, result, kwargs, surface):
        """The round one verdict (for the tip) replaced by the named surface's, its own kept as tip_verdict. kwargs
        are the call's live arguments after every pre hook (a label another package turned into point_mm
        included); the round one call already validated them."""
        kin, _ = surface_kin(self, surface)
        p = np.asarray(kwargs.get("point_mm"), dtype=float).reshape(-1) / MM
        s = self._sense()
        a = None
        if kwargs.get("approach") is not None:
            a, bad = self._unit_in_base(kwargs["approach"], "base", s)
            if bad is not None:
                return bad
        verdict, _ = surface_reach(self, kin, p, s.q, a, tol_m=bounds.of(self).thing(p).sigma_p)
        tip = {k: result[k] for k in TIP_KEYS if k in result}
        out = {**result, **verdict, "tip_verdict": tip}
        if "leading" in result:
            out["leading"] = f"{kin.name} (asked)"
        for k in TIP_KEYS:
            if k in result and k not in verdict and k != "leading":
                out[k] = None          # a tip-only value (its pose, its forces) would pass for the surface's
        return out

    # ------------------------------------------------------------------ approach
    @primitive
    def approach(self, target, along=None, surface="tip", standoff_mm=None, keep_in_view=True, guarded=True,
                 point_source=None, speed_mm_s=None, look_after=False, clear=None):
        """Reach the stated or derived standoff using one intent and one stream."""
        out = self._appr_impl(target, along, surface, standoff_mm, keep_in_view, point_source,
                             speed_mm_s, look_after, clear)
        if out.get("refused"):
            said = out["refused"].get("why", "the approach was refused") + "; " + out["refused"].get("hint", "look")
        else:
            said = ("the surface reached its standoff; touch along the suggested stroke" if out.get("reached") else
                    "the approach stopped before its standoff; look at the result before moving again")
        out = readable({"said": said, **out})
        st = self.state.setdefault("approach", {})
        if guarded is False and not st.get("retired_guarded"):
            st["retired_guarded"] = True
            out["retired"] = ("guarded is retired: the contact stop is always armed; look, then say clear: true "
                              "to state that the way is clear")
        return out

    def _appr_impl(self, target, along, surface, standoff_mm, keep_in_view, point_source,
                  speed_mm_s, look_after, clear):
        """Resolve and plan the target before moving; correction uses the same intent and tolerance."""
        p_t, tinfo, bad = resolve_target(self.world, target)
        if bad is not None:
            return {**bad, "reached": False, "stopped": "refused"}
        a, ainfo, bad = resolve_along(self, along, tinfo)
        if bad is not None:
            return {**bad, "reached": False, "stopped": "refused"}
        kin, bad = surface_kin(self, surface)
        if bad is not None:
            return {**bad, "reached": False, "stopped": "refused"}
        B = bounds.of(self)
        thing = B.thing(tinfo.get("id", p_t))
        intent = {}
        if clear is not None:
            intent["clear"] = clear
        if speed_mm_s is not None:
            try:
                speed = float(speed_mm_s) / MM
            except (TypeError, ValueError):
                speed = float("nan")
            if not math.isfinite(speed) or speed <= 0:
                return {**refusal("speed_mm_s", "speed_mm_s must be a finite number > 0",
                                  "leave it out: the robot derives its speed"), "reached": False, "stopped": "refused"}
            intent["speed"] = speed
        if "clear" in intent:
            word = B.word(intent)
            if word["refused"] is not None:
                return {**word["refused"], "reached": False, "stopped": "refused"}
        if point_source is not None and point_source not in SOURCES:
            return {**refusal("point_source", f"unknown point_source {str(point_source)[:30]!r}",
                              "touch, triangulated or single_look"), "reached": False, "stopped": "refused"}
        if not isinstance(look_after, bool):
            return {**refusal("look_after", "look_after must be true or false", "true takes a look after the move"),
                    "reached": False, "stopped": "refused"}
        src = point_source or tinfo["point_source"]
        target_out = {k: v for k, v in tinfo.items() if not k.startswith("_")}
        s = self._sense()
        reach, q_reach = surface_reach(self, kin, p_t, s.q, a, tol_m=thing.sigma_p)
        reach = apply_point_source(reach, src if src in SOURCES else None) or reach
        if not reach["reachable"]:
            unknown = reach.get("verdict") == UNKNOWN_VERDICT
            why = reach["why"]
            hint = ("touch the point or mark it in two looks and triangulate, then approach the id" if unknown else
                    "approach something nearer, or move the robot's base toward the target")
            return {**refusal("reach_unknown" if unknown else "reach", f"the {kin.name} cannot get to the target: "
                              f"{why}", hint), "reached": False, "stopped": "refused", "reach": reach,
                    "target": target_out, "tip_mm": _mm(s.tip)}
        # the room left before the target is the stop from the speed the surface may arrive with:
        # the band speed v_in where the target's own error is known (the core slows inside that band); where it
        # is not known there is no band to slow in, so the room is the stop from full speed
        v_in, _speed_item = B.v_in(thing, q_reach, a, intent)
        v_arrive = v_in if thing.sigma_p is not None else B.v_free(q_reach, a)
        derived_m = B.standoff(v_arrive, thing, q_reach, a)
        if standoff_mm is None:
            standoff = derived_m * MM
        else:
            try:
                standoff = float(standoff_mm)
            except (TypeError, ValueError):
                standoff = float("nan")
            if not math.isfinite(standoff) or standoff < 0:
                return {**refusal("standoff_mm", "standoff_mm is not a finite number >= 0",
                                  f"leave it out: this robot would stop {derived_m * MM:.2f} mm before this target"),
                        "reached": False, "stopped": "refused"}
        stop_m = B.d_stop(v_arrive, q_reach, a)
        error_m = R.QUANT * (thing.sigma_p or 0.0)
        tool_m = B.standoff(0.0)
        standoff_out = {"mm": standoff, "source": "derived" if standoff_mm is None else "stated",
                        "terms_mm": {"stopping": stop_m * MM, "target_error": error_m * MM, "tool": tool_m * MM},
                        "why": f"the robot derives {stop_m * MM:.2f} mm to stop from {v_arrive * MM:.1f} mm/s "
                               + ("(the speed it may arrive with at this target)" if thing.sigma_p is not None else
                                  "(full speed: the target's error is not known, so there is no band to slow in)")
                               + f", {error_m * MM:.2f} mm for the target's error "
                               f"and {tool_m * MM:.2f} mm for the tool"}
        goal = p_t - standoff / MM * a
        stroke_m = standoff / MM + reach_past_m(B, thing, q_reach, a)
        rigid = B.rigid(q_reach, a)
        sigma = thing.sigma_p or 0.0
        relative = R.REL * standoff / MM
        tol = max(reach_tol_m(self, q_reach, a, sigma), relative)
        R_before = kin.pose(s.q)[2]
        q_goal, ang_or_why, _, tilted, stroke_ok = self._appr_choose(kin, s.q, goal, a, p_t, bool(keep_in_view),
                                                                     [q_reach], standoff / MM, thing)
        if q_goal is None:
            return {**refusal("standoff_pose", f"no arm pose puts the {kin.name} {standoff:.2f} mm before the target "
                              f"facing along {[round(float(x), 3) for x in a]}: {ang_or_why}",
                              "try a smaller standoff, another along, or another surface"),
                    "reached": False, "stopped": "refused", "reach": reach, "target": target_out,
                    "tip_mm": _mm(s.tip)}
        stroke_m = standoff / MM + reach_past_m(B, thing, q_goal, a)
        rigid = B.rigid(q_goal, a)
        tol = max(reach_tol_m(self, q_goal, a, sigma), relative)
        started = s.t
        s, eased = self._ease_if_pressing()
        r, path = self._appr_move(s, q_goal, kin, goal, a, p_t, thing, intent, tol)
        if r.get("refused") is not None and r.get("stop") != "leg_refused":
            return {**r, "reached": False, "stopped": "refused", "reach": reach, "target": target_out,
                    "path": path, "tip_mm": _mm(s.tip)}
        r = motion_result(self, r)
        periods, report = r.get("periods"), r.get("bounds")
        e = r["s"]
        stop, contact, why = r["stop"], r.get("contact"), r.get("why") or ""
        ps = kin.pose(e.q)[0]
        fix_refused = None
        if stop == "goal" and float(np.linalg.norm(goal - ps)) > tol:
            fix = motion_result(self, self._cart_move(e, goal - ps, None, None, True, True,
                                                         kind="approach", target=thing, intent=intent))
            if fix.get("periods") is not None:
                periods = fix["periods"]
            if fix.get("bounds") is not None:
                report = fix["bounds"]
            if "refused" in fix:
                fix_refused = fix["refused"]
            else:
                e, stop, contact, why = fix["s"], fix["stop"], fix["contact"], fix.get("why") or why
        out = self._appr_result(kin, e, p_t, goal, a, tinfo, ainfo, stop, contact, keep_in_view, thing, tol, stroke_m)
        out["standoff"] = standoff_out
        out["reach_tol_mm"] = {"mm": tol * MM, "rigid_mm": rigid * MM, "target_error_mm": sigma * MM,
                               "relative_standoff_mm": relative * MM, "solver_mm": POS_TOL_M * MM}
        if report is not None:
            out["bounds"] = report
        else:
            B.report(out, [_speed_item])
        out["orientation"] = self._appr_orientation(kin, e.q, R_before)
        if fix_refused is not None:
            out["correction_refused"] = {"check": fix_refused.get("check"), "why": fix_refused.get("why"),
                                         "means": "the last straight correction onto the standoff point did not "
                                                  "run; landing_err_mm is where the move left the surface"}
        out.update({"path": path, "reach": reach, "target": target_out,
                    "stroke": {"plans": bool(stroke_ok), "length_mm": round(stroke_m * MM, 2),
                               "why": "" if stroke_ok else "from the standoff pose a straight stroke along `along` "
                                                           "does not pass the path check (planned, not run)"}})
        if tilted:
            out["tilted"] = (f"no square pose with a stroke that plans passes the checks here, so the {kin.name} "
                             f"faces {math.degrees(ang_or_why):.0f} deg off along; another surface may face square: "
                             + ", ".join(n for n in surface_names(self.model.spec) if n != kin.name))
        if why and stop != "goal":
            out["why"] = why
        if r.get("leg") and stop != "goal":
            out["stopped_in_leg"] = r["leg"]
        if eased:
            out["note"] = "a joint was still pressing; eased before the move"
        if look_after:
            out["look"] = self._appr_look("approach")
        out["periods"] = round((self.backend.read()["t"] - started) / B.dt())
        return out

    def _appr_orientation(self, kin, q, R_before):
        """How the hand arrived: which orientation rule chose it, the turn from the hand before
        the call, the hand's up and the image's up against base up, whether the camera rides above the finger, and
        the joint pose used."""
        choice = dict((self.state.get("approach") or {}).get("choice") or {})
        fk = self.model.fk(q)
        p_tip, R_tip = fk["tip"]
        p_cam, R_cam = fk["camera"]
        hand_up = R_tip[:, 2]
        img_up = R_cam[:, 1]          # the camera frame's y is image up
        up = np.array([0.0, 0.0, 1.0])
        out = {"chosen": {"nearest_present": "the orientation nearest the hand before the call, of those that face "
                                             "the stroke",
                          "nearest_present_tilted": "the orientation nearest the hand before the call, at the "
                                                    "smallest tilt off the stroke that passes the checks",
                          "nearest_present_of_the_rest": "the nearest to the hand before the call of the poses "
                                                         "that met the checks"}.get(choice.get("rule"),
                                                                                    "the solve's pose"),
               "turned_deg": round(angle_from(R_before, kin.pose(q)[2]), 1),
               "hand_up_from_base_up_deg": round(angle_deg(hand_up, up), 1),
               "image_up_from_base_up_deg": round(angle_deg(img_up, up), 1),
               "camera_above_finger": bool(float(p_cam[2]) > float(p_tip[2])),
               "hand_inverted": bool(float(hand_up @ up) < -0.5),
               "q_deg": [round(math.degrees(float(x)), 2) for x in q]}
        if choice.get("why"):
            out["why"] = choice["why"]
        return out

    def _appr_result(self, kin, e, p_t, goal, a, tinfo, ainfo, stop, contact, keep_in_view, thing, tol, stroke_m):
        """Describe the measured landing, facing, contact and the same reach-past touch stroke."""
        ps, n_s, _ = kin.pose(e.q)
        to_t = p_t - ps
        along_dist = float(to_t @ a)
        line_off = float(np.linalg.norm(to_t - along_dist * a))
        landing = float(np.linalg.norm(goal - ps))
        along_err = angle_deg(n_s, a)
        n_t = ainfo.get("_normal")
        if n_t is None:
            n_t = tinfo.get("_normal")
        view = kin.view_of(e.q, p_t)
        out = {"reached": bool(stop == "goal" and landing <= tol), "stopped": stop,
               "tip_mm": _mm(e.tip),
               "surface": {"name": kin.name, "pos_mm": _mm(ps), "faces": [round(float(x), 4) for x in n_s]},
               "goal_mm": _mm(goal), "landing_err_mm": round(landing * MM, 2),
               "standoff_achieved_mm": round(along_dist * MM, 2), "line_offset_mm": round(line_off * MM, 2),
               "along": [round(float(x), 4) for x in a], "along_source": ainfo.get("along_source"),
               "along_err_deg": round(along_err, 2),
               "normal_err_deg": None if n_t is None else round(angle_deg(n_s, -np.asarray(n_t)), 2),
               "view": {k: v for k, v in view.items() if k in ("target_px", "in_view", "in_frame", "hidden_by",
                                                               "depth_mm", "why")},
               "contact": contact, "force_n_end": round(float(e.Fmag), 2)}
        if stop == "goal" and e.Fmag > bounds.of(self).f_thr(0.0, 0.0):
            out["touching"] = (f"the arm feels {e.Fmag:.1f} N at the standoff: something is against it (not "
                               "enough to stop the guard); look or picture(view=side) before the stroke")
        if not keep_in_view:
            out["view"]["note"] = "keep_in_view was off: the view was not part of the solve"
        w_tool = (min(kin.size_mm) / MM / TWO if kin.size_mm else bounds.of(self).standoff(0.0))
        off_m = stroke_m * abs(math.tan(math.radians(along_err)))
        if off_m > w_tool:
            out["warning"] = (f"the facing is {along_err:g} deg off along: over the {stroke_m * MM:.2f} mm stroke "
                              f"the tip leaves its line by {off_m * MM:.2f} mm, over the tool half width "
                              f"{w_tool * MM:.2f} mm; choose another facing")
        if stop == "goal":
            out["next"] = {"tool": "touch", "args": {"direction": out["along"],
                "max_mm": round((along_dist + reach_past_m(bounds.of(self), thing, e.q, a)) * MM, 1), "frame": "base"},
                           "means": "feel along the stroke for the target's surface"}
        return out


# ------------------------------------------------------------------------------------------------ hooks
def setup(ex):
    """Register the package readers and hooks without performing motion."""
    st = ex.state.setdefault("approach", {"kin": {}})

    @_own
    def pre(name, kwargs):
        """Validate the extensions arguments before the underlying primitive moves."""
        if name in LOOK_NAMES:
            if "look_after" in kwargs:
                v = kwargs.pop("look_after")
                if not isinstance(v, bool):
                    return refusal("look_after", "look_after must be true or false",
                                   "true takes a wrist look after the move and returns it in the same result")
            return None
        if name == "reach_check":
            st.pop("reach_call", None)
            surface = kwargs.pop("surface", None)
            src = kwargs.pop("point_source", None)
            if src is not None and src not in SOURCES:
                return refusal("point_source", f"unknown point_source {str(src)[:30]!r}",
                               "touch, triangulated or single_look: where the point came from")
            if surface is not None and surface != "tip":
                _, bad = surface_kin(ex, surface)
                if bad is not None:
                    return bad
                # the live arguments: a later package's pre hook may still fill point_mm (a label), so the
                # surface verdict is made in the post hook from this same dict
                st["reach_call"] = {"kwargs": kwargs, "surface": surface}
        return None

    @_own
    def post(name, kwargs, result):
        """Read the result and session measurements without stepping the robot."""
        if not isinstance(result, dict):
            return None
        if name in LOOK_NAMES:
            if kwargs.get("look_after") is True and "refused" not in result:
                return {**result, "look": ex._appr_look(name)}
            return None
        if name == "reach_check":
            call = st.pop("reach_call", None)
            if "refused" in result:
                return None
            out = result
            if call is not None:
                out = ex._appr_reach_check(result, call["kwargs"], call["surface"])
            return apply_point_source(out, kwargs.get("point_source")) or (out if out is not result else None)
        return None

    ex.add_pre(pre, names=list(LOOK_NAMES) + ["reach_check"])
    ex.add_post(post, names=list(LOOK_NAMES) + ["reach_check"])


def _state_resource(ex, session, **_kw):
    """Serve the package state document as an operator-visible resource."""
    return state_doc(ex)


def _state_handler(ex, session, **_args):
    """state() reads robot://state through the session, so what other packages add to the resource is in it too."""
    fn = (getattr(session, "resources", None) or {}).get("robot://state")
    if callable(fn):
        return fn()
    return ex.robot_state()


RESOURCES = {"robot://state": (_state_resource, "Where the robot is now: joint angles, the tip's position and "
                                                "roll/pitch/yaw in base, joint loads, the gripper opening, the "
                                                "world epoch and the last look with its age.")}
HANDLERS = {"state": _state_handler}

_LOOK_AFTER = S._bool("take a wrist look when the move ends and return it as look in this result (default false)")
_TARGET = {"anyOf": [{"type": "string"}, {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}],
           "description": "the point to go to: an id or label from robot://world (a touched contact or a "
                          "triangulated point), or [x, y, z] in base frame mm"}
_ALONG = {"anyOf": [{"type": "string"}, {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}],
          "description": "the way the surface travels onto the target, in base frame: [x, y, z], a name (forward, "
                         "back, left, right, up, down), normal:<world id> (straight into that touched surface, "
                         "as touch estimated it) or affordance:<device>/<part> (only when that part in "
                         "robot://devices gives along_base; a part described in words is refused)"}

TOOLS = [
    S._tool("state",
            "Read where the robot is now: joint angles in degrees, the tool tip's position (base frame mm) and "
            "roll/pitch/yaw, how hard each joint is working, the gripper opening, the world epoch and the last "
            "look with its age; no motion.",
            S._obj({}), {}, "read the tip position and orientation before planning a move", method="robot_state"),
    S._tool("approach",
            "Go to a point with a named tool surface: checks reach first (and moves nothing if the target is "
            "short), then puts the surface standoff_mm before the target facing along the stroke as squarely as "
            "the poses it tries allow (an arm with few joints may face far off), preferring a pose with the "
            "target in the wrist view and, of the hand orientations that face the stroke, the one nearest the "
            "present hand (an upright hand stays upright); returns reached, "
            "the surface point, standoff_achieved_mm, along_err_deg (how far the facing is off), whether the "
            "camera sees the target, orientation (which rule chose it, turned_deg from the hand before the call, "
            "hand and image up against base up, camera_above_finger, q_deg) and the touch to make next.",
            S._obj({"target": _TARGET, "along": _ALONG,
                    "surface": S._str("which tool surface leads (robot://body tool.surfaces names them; default tip)"),
                    "standoff_mm": {"type": "number", "minimum": 0,
                        "description": ("how far before the target to stop, mm; default: derived by the robot "
                        "(the result says from what)")},
                    "keep_in_view": S._bool("prefer an arm pose that keeps the target in the wrist view (default "
                                            "true)"),
                    "guarded": S._bool("retired: the contact stop is always armed"),
                    "point_source": S._str("where the target point came from, when it is a raw point (a world id "
                                           "says it itself)", list(SOURCES)),
                    "speed_mm_s": {"type": "number", "minimum": 0,
                        "description": ("tool speed in mm per second; default: derived by the robot (the result "
                        "says from what)")},
                    "clear": {"anyOf": [{"type": "boolean"}, {"type": "object",
                        "properties": {"look": {"type": "string"}}, "required": ["look"],
                            "additionalProperties": False}],
                                "description": ("you looked and judge the way clear: recorded in the result; it "
                        "changes no speed")},
                    "look_after": _LOOK_AFTER}, ["target", "along"]),
            {"target": "contact_3", "along": "normal:contact_3", "surface": "tip"},
            ("put the tip at the derived standoff in front of the touched point "
                "contact_3, square to its surface, ready to touch")),
] + [{"extends": n, "args": {"look_after": _LOOK_AFTER},
      "note": "look_after true returns the wrist look taken when it ends, in the same call."} for n in LOOK_NAMES] + [
    {"extends": "reach_check",
     "args": {"surface": S._str("which tool surface must arrive (robot://body tool.surfaces; default tip)"),
              "point_source": S._str("where the point came from: touch, triangulated or single_look (one look "
                                     "fixes no depth, so an unreachable answer then reads unknown)", list(SOURCES))},
     "note": "surface names the tool face that must arrive; point_source says how far to trust the point."},
]

PRACTICES = ("robot://state (free) gives the tip's pose. approach(target, along, surface) checks reach, then "
             "lands the named surface at a standoff facing the stroke; look_after=true on a move, touch, press or "
             "approach adds its look to the same call.")
