# SPDX-License-Identifier: AGPL-3.0-only
"""Package "selfcheck": check the body model against the robot before trusting it.

Tools:
- self_check(joints="all", amplitude_deg=None, apply=true): wiggles each joint a few degrees in free space (path
  checked, timed by Bounds, guarded with held-pose dynamic and bias classification), takes a wrist look at each end, and fits the
  body that explains how the picture of the surface under the arm moved (selfcheck_image, selfcheck_fit). Per
  joint: sign_ok, gain_ratio, offset_deg_est with its spread, residual_px. Camera: rot_err_deg, shift_mm_est.
  Verdict ok | compensated | uncompensated | inconclusive | refuse_motion; "unchecked" lists what was asked for
  and could not be judged, and a joint with no usable look makes the verdict inconclusive, never ok.
  A force stop without progress on a freely following arm distrusts the force estimate; that wiggle continues
  under joint lead, allowing for the servo lag, commanded inertia, starting lead and each joint's own step.
  The zero offsets and the mount are read against the configured surface height (cfg floor_z_m) with every gain
  held at 1. When a wiggle moves the picture by another amount than predicted (a joint's scale, a surface that
  is not at that height, a repeating surface) nothing from that check is corrected or offered to apply_offset.
- zero_check(joint, end="auto", firm=false): one joint swung to an end of its range at the full rated speed
  where nothing is known near, braking into a slow zone derived from measured zero error, or slow over the whole
  range when no zero was measured. Lead, speed and applied torque rise over learned free drive, with the model's
  gravity change allowed for, find the stop; the force observer is not used beyond the model's range. The slow
  speed bounds servo wind-up by the hand's weight at this joint, or the joint rating on the operator's firm word.
  Easing measures stop stiffness from give and load; the reported reading's spread includes the encoder step
  and the servo and stop give under torque the joint cannot distinguish from none. The arm returns along the
  searched path. Refused after a dark look, like any move toward the world.
- apply_offset(joint, deg): set a joint's zero offset (true angle = reading + deg). Refused unless a self_check or
  zero_check measured that value in this epoch, or it restores a value that was active before (the undo, which
  every application journals).
Compensation (selfcheck_comp): ex.backend is wrapped so offsets are added on read and removed on command; ex.model
is wrapped so the camera sits where it was measured. What the camera cannot tell is said, never guessed: a joint
that turns about the surface normal through the base, and the joint that carries the camera (its offset is the
same picture as a mount turn and is counted there); zero_check measures those.
Hooks: pre on every motion primitive (MOTION: the round one moves and, by name only, the other packages' tools
that move the arm): refusal body_sign while a self_check has found a joint turning the wrong way; post on look,
standard_look, mark, triangulate: camera_uncertain when a mount error was found and not corrected; event
base_moved: the check is stale. robot://body gains "measured".
No robot is named and no simulator ground truth is read here.
"""

import math
import traceback

import mujoco
import numpy as np

from rcpx.control_base import budget_refusal
from rcpx import perception, surfaces, bounds
from rcpx import bounds_ratios as R
from rcpx.r2 import selfcheck_limits as limits
from rcpx.r2 import selfcheck_latency as latency
from rcpx.context import refusal as context_refusal
from rcpx.control_base import _mm, primitive
from rcpx.r2 import selfcheck_comp as comp
from rcpx.r2 import selfcheck_fit as fit
from rcpx.r2 import selfcheck_image as image
from rcpx.r2 import selfcheck_motion as motion
from rcpx.r2.selfcheck_report import (camera_block, joint_table, measured_doc, scale_problems, unchecked,
                                      verdict_of, result_sentence, false_stop_row)

ID = "I18"
PKG = "selfcheck"
DEG = math.pi / 180.0
MM = 1000.0
MOTION = frozenset(["move_joints", "move_tip", "orient_tool", "touch", "press", "home", "park", "retreat",
                    "standard_look", "measure_base_give", "zero_check",
                    "approach", "grasp", "release", "impulse", "pinch", "probe_motion",
                    "release_check", "wait_for_light"])   # the self-check names; wait_for_light parks first
CAMERA_RESULTS = ("look", "standard_look", "mark", "triangulate")


def _state(ex):
    return ex.state[PKG]


def refusal(check, why, hint):
    return {"said": str(why).rstrip(".") + ".", **context_refusal(check, why, hint)}


class Mixin:
    # ------------------------------------------------------------------ helpers
    def _sc_joint(self, joint):
        names = self.model.joint_names
        if isinstance(joint, str) and joint in names:
            return names.index(joint), None
        return None, refusal("joint", f"no joint named {joint!r}", 'self_check(joints="all"): joints: ' + ", ".join(names))

    def _sc_revolute(self, j):
        m = self.model.mj
        return int(m.jnt_type[m.joint(self.model.joint_names[j]).id]) == int(mujoco.mjtJoint.mjJNT_HINGE)

    def _sc_valid(self, rec):
        st = _state(self)
        return rec is not None and rec["epoch"] == self.epoch and rec["gen"] == st["gen"]

    def _sc_note(self, name, deg, sigma, source, kind):
        _state(self).setdefault("zero_history", {}).setdefault(name, []).append(
            {"deg": float(deg), "sigma_deg": float(sigma)})
        _state(self)["measured"][name] = {"deg": round(float(deg), 3), "sigma_deg": round(float(sigma), 3),
                                          "source": source, "kind": kind, "epoch": self.epoch,
                                          "gen": _state(self)["gen"], "t_s": round(self.sim_time(), 3)}

    def _sc_set_offsets(self, new_deg, source):
        """Set joints' zero offsets ({name: total deg}); the arm does not move. Returns the journaled records."""
        st, jc, names = _state(self), self._sc_jc, self.model.joint_names
        tip0 = self._sense().tip.copy()
        delta = np.zeros(self.model.n)
        recs = []
        for name, deg in new_deg.items():
            j = names.index(name)
            was = float(jc.offset[j]) / DEG
            delta[j] = float(deg) * DEG - jc.offset[j]
            recs.append({"joint": name, "was_deg": round(was, 3), "offset_deg": round(float(deg), 3),
                         "source": source, "epoch": self.epoch, "t_s": round(self.sim_time(), 3),
                         "undo": {"tool": "apply_offset", "args": {"joint": name, "deg": round(was, 3)}}})
        jc.offset = jc.offset + delta
        s = comp.resync(self, delta)
        stale = self._sc_stale_world("the body model's joint zeros were corrected")
        for r in recs:
            r["tip_shift_mm"] = round(float(np.linalg.norm(s.tip - tip0)) * MM, 2)
            st["applied"].append(r)
            self._journal("selfcheck_offset", **r)
        st["offsets_deg"] = {n: round(float(o) / DEG, 3) for n, o in zip(names, jc.offset) if abs(o) > 1e-12}
        return recs, stale, s

    def _sc_stale_world(self, why):
        if not any(e["kind"] in ("contact", "located", "device") and e.get("point_mm") is not None and not e["stale"]
                   for e in self.world.entries.values()):
            return []
        return self.world.world_moved({"why": why}, source="self_check")["stale"]

    def _sc_frames(self, cid, w):
        names, out = self.model.joint_names, []
        items = [("ref", w["ref_img"], "the reference look, where the arm was")]
        for lk in w["looks"]:
            side = "plus" if lk["side"] > 0 else "minus"
            turned = (lk["q"][lk["joint"]] - w["q_ref"][lk["joint"]]) / DEG
            items.append((f"{names[lk['joint']]}_{side}", lk["img"],
                          f"joint {names[lk['joint']]} turned {turned:+.1f} deg from the reference look"))
        for stem, img, caption in items:
            path = self._save(img, f"{cid}_{stem}")
            out.append({"frame": path if isinstance(path, str) else None, "caption": caption})
        return out

    def _sc_result(self, out, t0):
        out['said'] = result_sentence(out, _state(self)['sign_error'])
        B = bounds.of(self)
        plans = [p for p in self._sc_plans if not p.refused and getattr(p, '_sc_ran', False)]
        longest = max(plans, key=lambda p: len(p.path), default=None)
        if longest is not None and longest.items and out.get('arm_moved', True):
            B.report(out, longest.items)
        out["periods"] = round((self.sim_time() - t0) / self.dt)
        return out

    # ------------------------------------------------------------------ self_check
    @primitive
    def self_check(self, joints="all", amplitude_deg=None, apply=True, clear=None):
        t0 = self.sim_time()
        self._sc_plans = []
        intent = None if clear is None else {"clear": clear}
        st, model = _state(self), self.model
        names = model.joint_names
        if joints is None or isinstance(joints, str) and joints == "all":
            idx = list(range(model.n))
        else:
            idx = []
            if not isinstance(joints, (str, list, tuple)) or not joints:
                return refusal("joint", f"joints must be \"all\", a joint name or a list of names, not {joints!r}",
                               'self_check(joints="all"): joints: ' + ", ".join(names))
            for name in ([joints] if isinstance(joints, str) else list(joints)):
                j, bad = self._sc_joint(name)
                if bad is not None:
                    return bad
                idx.append(j)
        amp_deg = None
        if amplitude_deg is not None:
            if isinstance(amplitude_deg, bool) or not isinstance(amplitude_deg, (int, float, np.integer, np.floating)):
                return refusal("amplitude", "amplitude_deg must be a finite positive number",
                               "self_check(): leave amplitude_deg out to take the least turn the camera can read")
            amp_deg = float(amplitude_deg)
            if not math.isfinite(amp_deg) or amp_deg <= 0:
                return refusal("amplitude", "amplitude_deg must be a finite positive number",
                               "self_check(): leave amplitude_deg out to take the least turn the camera can read")
        skipped = [{"joint": names[j], "why": "not a revolute joint: the wiggle check covers hinges only"}
                   for j in idx if not self._sc_revolute(j)]
        idx = [j for j in dict.fromkeys(idx) if self._sc_revolute(j)]
        s, _ = self._ease_if_pressing()
        pre = np.asarray(self.backend.render("wrist"))
        if perception.can_see(pre, cam=limits.camera_facts(self)) == "dark":
            return refusal("can_see", f"the scene is dark (brightness {perception.brightness(pre):.1f} of 255): the "
                           "camera cannot follow the surface", "get light on the scene, then run self_check()")
        floor_z, mount = float(self.cfg["floor_z_m"]), self.cfg["mount_half_m"]
        body = fit.Body(model, s.q, floor_z)
        share = body.sees_floor()
        if not idx:
            return refusal("joint_kind", "none of the joints asked for is a revolute joint",
                           'self_check(joints="all") checks the hinge joints listed in robot://body')
        if share < fit.MIN_FLOOR_SHARE:
            return refusal("view", "by the body description the camera does not look at the surface the robot "
                           "stands on from this pose, so nothing was checked",
                           "if the frame of a look does show that surface, the description is wrong about this "
                           "pose (a joint's zero or direction is far off): do not move on distances, have the "
                           "joint zeros checked. Otherwise move so the wrist camera looks down at the surface "
                           "with some of it in view (the home pose usually does), then run self_check()")
        edges = image.Reference(pre, image.robot_mask(model, s.q, floor_z, mount)).n_edges
        if edges < image.MIN_INLIERS:
            return refusal("texture", f"the surface in view shows too few marks to follow ({edges} edge pixels)",
                           "move the camera over a part of the surface with marks, edges or print on it, then self_check()")
        amps, amplitude_source = limits.amplitudes(self, s.q, idx, floor_z,
                                                  None if amp_deg is None else amp_deg * DEG)
        # a part at or inside the arm's own volume along the turns is refused before anything is planned (the
        # straight joint turns themselves, at the least clearance any speed needs); the planned legs are checked
        # again below at the clearance their own speed needs
        near = motion.free_space(self, motion.turn_lines(self, idx, amps), bounds.of(self).clearance(0.0),
                                 touching=True)
        if near is not None:
            return refusal("free_space", *near)
        legs = motion.planned_legs(self, idx, amps, intent)
        for plan in legs:
            if plan.refused:
                return plan.refused
        over = budget_refusal(self, next((p for p in legs if getattr(p, "unseen", None)), legs[0] if legs else None),
                              sum(len(p.path) for p in legs), intent)
        if over is not None:   # the wiggles would not fit the control periods the harness left: nothing moves
            return over
        peak_v = 0.0
        for plan in legs:
            points = [self._q_des] + plan.path
            peak_v = max(peak_v, max((float(np.linalg.norm(model.fk_tip(y)[0] - model.fk_tip(x)[0])) / self.dt
                                      for x, y in zip(points, points[1:])), default=0.0))
        crowded = motion.free_space(self, legs, bounds.of(self).clearance(peak_v))
        if crowded is not None:
            return refusal("free_space", *crowded)
        latency_record, capture = latency.observer(self, idx, s.q, floor_z)
        w = motion.wiggle(self, idx, amps, intent, legs=legs, capture=capture)
        if "refused" in w:
            return w
        st["n_check"] += 1
        cid = f"SC{st['n_check']}"
        out = {"said": "The camera watched the body check.", "check_id": cid, "amplitude_deg": amp_deg, "epoch": self.epoch, "t_s": round(self.sim_time(), 3),
               "returned_to_start": bool(w["returned"]), "planned_looks": w["planned_looks"], "frames": self._sc_frames(cid, w),
               "skipped": skipped + [{"joint": names[k["joint"]], "side": k["side"], "why": k["why"]}
                                     for k in w["skipped"]]}
        out["amplitudes_deg"] = {names[j]: math.degrees(amps[j]) for j in idx}
        out["amplitude_source"] = amplitude_source
        out['contact_guard'] = w['contact_guard']
        out["limits"] = {"contact_n": {"value": bounds.of(self).f_thr(0, 0),
                                       "source": "the force hidden at rest plus the measured noise band"},
                         "contact_lead_deg": {"value": math.degrees(w['contact_lead_rad']),
                                              "source": "the largest joint step: two encoder steps or servo give under the joint torque threshold"},
                         "amplitude_deg": {"value": max(out["amplitudes_deg"].values(), default=0),
                                           "source": amplitude_source}}
        out["bias_stops"] = []
        for r in w['false_stops']:
            if r['kind'] == 'distrusted':
                out['bias_stops'].append({**r, 'joint': names[r['joint']],
                    'said': f"the force estimate rose by {r['rise_n']:g} newtons while every joint followed its target freely, "
                            "as a body that disagrees with its description makes it: the wiggle went on under the joint lead guard"})
            else:
                out['bias_stops'].append(false_stop_row(names, r))
        out["camera_latency"] = {"latency_s": None, "resolution_s": None,
                                 "joint": names[latency_record["joint_index"]], "known": False,
                                 "why": "camera latency has not been measured by this check"}
        a = None
        if w["stopped"]:
            out.update({"verdict": "inconclusive", "stopped": w["stopped"]["stopped"],
                        "why": f"the wiggle of joint {names[w['stopped']['joint']]} stopped "
                               f"({w['stopped']['stopped']}) before its look",
                        "hint": "retreat, then self_check(clear=true) after looking at the free way"})
        else:
            a = fit.analyse(model, w["q_ref"], w["ref_img"], w["looks"], floor_z, mount)
            if not a["ok"]:
                out.update({"verdict": "inconclusive", "why": a["why"],
                            "hint": "look down at a marked part of the surface the robot stands on and run again"})
        if w["stopped"] and w["stopped"]["stopped"] == "contact":
            if w['contact_guard'] == 'joint lead':
                stopped = w['stopped']
                out['why'] += (f"; joint {names[stopped['lead_joint']]} led its target by an extra "
                               f"{stopped['extra_lead_deg']:g} deg over its {stopped['step_deg']:g} deg step")
            else:
                out["why"] += (f"; the tool felt a push of about {w['stopped'].get('peak_n', 0):g} newtons "
                               f"over the leg's threshold of {w['stopped'].get('threshold_n', 0):g} newtons")
        if a is None or not a["ok"]:
            st["last"] = {"check_id": cid, "verdict": "inconclusive", "epoch": self.epoch, "gen": st["gen"],
                          "t_s": out["t_s"], "why": out.get("why")}
            return self._sc_result(out, t0)
        active = self._sc_jc.offset / DEG
        table, todo = joint_table(names, a, active, idx, [math.degrees(limits.offset_range(self, j)) for j in range(model.n)])
        out["camera_latency"] = latency.measure(self, w, latency_record, table, floor_z)
        cam, cam_fix = camera_block(model, a, self._sc_cc)
        if scale_problems(table):     # the mount too was read with the gains held at 1: not trusted, so not
            cam_fix = None            # corrected, and later looks are not told the camera is off by its numbers
            if cam["state"] == "uncertain":
                cam["state"] = "untrusted"
        left, missing = unchecked(names, table, idx)
        out["joints"], out["camera"], out["unchecked"] = table, cam, left
        out["fit"] = {"surface_z_mm": round(floor_z * MM, 1),
                      "assumes": "the surface under the arm is flat, level with the base frame and at surface_z_mm "
                                 "(from the configuration, not measured here); a height that is off by a few mm "
                                 "reads as joint zeros that are off by up to a degree or more",
                      "edge_pixels": a["edges"], "looks_used": len(a["looks_used"]),
                      "looks_dropped": [{"joint": names[d["joint"]], "why": d["why"]} for d in a["looks_dropped"]],
                      "residual_px": round(a["s_px"], 3), "surface_share_of_view": a["floor_share"],
                      "edges": "lightness and colour" if a["light_edges"] else "colour"}
        # a sign error stands until a check of that joint clears it: a partial check does not forget it
        kept = [n for n in st["sign_error"] if table[n].get("sign_ok") is None]
        sign_bad = [n for n in names if table[n].get("sign_ok") is False or n in kept]
        for name, row in table.items():
            if row.get("apply_with_deg") is not None:
                self._sc_note(name, row["apply_with_deg"], row["offset_sigma_deg"], cid, "self_check")
        st["sign_error"] = sign_bad
        applied, stale = [], []
        if apply and not sign_bad:
            new = {n: table[n]["apply_with_deg"] for n in todo}
            if new:
                applied, stale, _ = self._sc_set_offsets(new, cid)
            if cam_fix is not None:
                self._sc_cc.add_mount(*cam_fix)
                turn, shift = self._sc_cc.mount_deg_mm()
                rec = {"was": dict(st["camera"]), "turn_deg": [round(x, 3) for x in turn],
                       "shift_mm": [round(x, 2) for x in shift], "source": cid}
                self._journal("selfcheck_camera", **rec)
                comp.resync(self)
                stale = stale or self._sc_stale_world("the camera mount was corrected")
                cam["state"] = "corrected"
        still = cam["state"]
        if still in ("ok", "untrusted") and self._sc_cc.active:
            still = "corrected"                                         # an earlier fix holds
        st["camera"] = {"state": still, "rot_err_deg": cam["rot_err_deg"], "shift_mm_est": cam["shift_mm_est"],
                        "source": cid, "mount_turn_deg": [round(x, 3) for x in self._sc_cc.mount_deg_mm()[0]],
                        "mount_shift_mm": [round(x, 2) for x in self._sc_cc.mount_deg_mm()[1]]}
        for r in applied:
            table[r["joint"]]["applied_offset_deg"] = r["offset_deg"]
        out["verdict"], out["findings"] = verdict_of(table, cam, sign_bad, {r["joint"] for r in applied}, missing,
                                                     round(floor_z * MM, 1))
        out["findings"] += [f"joint {n} turned the opposite way in an earlier check and was not checked again"
                            for n in kept]
        out["compensation"] = {"offsets_deg": dict(st.get("offsets_deg", {})), "applied_now": applied,
                               "camera": cam["state"], "world_entries_made_stale": stale}
        out["tip_mm"] = _mm(self._sense().tip)
        st["stale"] = None
        st["last"] = {"check_id": cid, "verdict": out["verdict"], "epoch": self.epoch, "gen": st["gen"],
                      "t_s": out["t_s"], "findings": out["findings"], "joints": table, "camera": cam,
                      "unchecked": left}
        try:
            self.world.event("self_check", data={"check_id": cid, "verdict": out["verdict"],
                                                 "findings": out["findings"]}, source="self_check")
        except Exception:      # the world is a record: the check's result does not depend on it, the journal says
            self._journal("r2_problem", package=PKG, where="self_check world event", trace=traceback.format_exc())
        return self._sc_result(out, t0)

    # ------------------------------------------------------------------ zero_check
    @primitive
    def zero_check(self, joint, end="auto", firm=False, clear=None):
        t0 = self.sim_time()
        self._sc_plans = []
        intent = None if clear is None else {"clear": clear}
        if not isinstance(firm, bool):
            return refusal("firm", "firm must be a boolean", f"zero_check(joint={joint!r}, firm=false)")
        st, model = _state(self), self.model
        j, bad = self._sc_joint(joint)
        if bad is not None:
            return bad
        if end not in ("auto", "low", "high"):
            return refusal("end", f"unknown end {end!r}", f"zero_check(joint={joint!r}, end='auto')")
        if not self._sc_revolute(j):
            return refusal("joint_kind", f"joint {joint} is not a revolute joint", "self_check() checks the hinge joints")
        dark = self._dark_refusal_now()      # a sweep to a joint's end is a move toward the world, unseen
        if dark is not None:
            return dark
        s, _ = self._ease_if_pressing()
        lo, hi = float(model.q_lo[j]), float(model.q_hi[j])
        near_high = (hi - s.q[j]) <= (s.q[j] - lo)
        order = {"low": [-1], "high": [1], "auto": [1, -1] if near_high else [-1, 1]}[end]
        st["n_zero"] += 1
        zid = f"Z{st['n_zero']}"
        tried, r, direction, blind = [], None, None, None
        for direction in order:
            r = motion.zero_search(self, j, direction, st, joint, firm, intent)
            if "refused" in r:
                if end == 'auto' and r['refused']['check'] == 'zero_blind':
                    blind = r
                    tried.append({'end': 'high' if direction > 0 else 'low', 'why': r['refused']['why']})
                    r = None
                    continue
                if not r["refused"]["check"].startswith("path"):
                    return r
                tried.append({"end": "high" if direction > 0 else "low", "why": r["refused"]["why"]})
                r = None
                continue
            break
        out = {"said": "The joint stop search ended.", "zero_id": zid, "joint": joint, "epoch": self.epoch, "t_s": round(self.sim_time(), 3)}
        if r is None:
            if blind is not None:
                return blind
            out.update({"verdict": "no_stop", "stop_reading_deg": None, "declared_stop_deg": None, "offset_deg": None,
                        "why": "the way to the joint's end is not free of the floor or of the arm itself",
                        "ends_tried": tried, "arm_moved": False})
            return self._sc_result(out, t0)
        out["search"] = r["search"]
        out["planned_s"] = r["planned_s"]
        out["limits"] = {k: {"value": v, "source": r["search"]["sources"][k]}
                         for k, v in r["search"].items() if k != "sources"}
        if limits.zero_known(self, st, joint)[0] is None:
            out["note"] = ("the stop was searched at the slow rule over the whole joint range because no check "
                           "has shown how far this joint's zero can be off; a stall anywhere in that range "
                           "was taken as the stop: check it against self_check or the other end before applying it")
        declared = hi if direction > 0 else lo
        out.update({"end": "high" if direction > 0 else "low", "declared_stop_deg": round(declared / DEG, 3),
                    "travel_deg": round(r["travel"] / DEG, 2), "returned_to_start": bool(r["returned"]),
                    "load_at_stop": r["load"], "ends_tried": tried, "arm_moved": bool(r["moved"])})
        if not r["found"]:
            out.update({"verdict": "no_stop", "stop_reading_deg": None, "offset_deg": None, "why": r["why"]})
            if r["stopped"]:
                out["stopped"] = r["stopped"]
            return self._sc_result(out, t0)
        sigma = math.degrees(limits.stop_spread(self, j, r["give"], r["load_nm"]))
        out['limits']['offset_sigma_deg'] = {
            'value': round(sigma, 3),
            'source': "the encoder step plus servo give and measured stop stiffness under torque the joint cannot tell from none"}
        offset = (declared - r["reading"]) / DEG
        total = float(self._sc_jc.offset[j]) / DEG + offset
        out.update({"stop_reading_deg": round(r["reading"] / DEG, 3), "offset_deg": round(offset, 3),
                    "offset_sigma_deg": round(sigma, 3), "verdict": "same" if abs(offset) <= R.QUANT * sigma else "moved",
                    "applied_offset_deg": round(float(self._sc_jc.offset[j]) / DEG, 3),
                    "apply_with_deg": round(total, 3),
                    "meaning": "the joint's true angle is its reading plus offset_deg; apply_offset(joint, "
                               "apply_with_deg) makes later readings true"})
        self._sc_note(joint, total, sigma, zid, "zero_check")
        st["zero"][joint] = {k: out[k] for k in ("zero_id", "end", "stop_reading_deg", "declared_stop_deg",
                                                 "offset_deg", "verdict", "epoch", "t_s")}
        return self._sc_result(out, t0)

    # ------------------------------------------------------------------ apply_offset
    @primitive
    def apply_offset(self, joint, deg):
        st = _state(self)
        j, bad = self._sc_joint(joint)
        if bad is not None:
            return bad
        try:
            deg = float(deg)
        except (TypeError, ValueError):
            deg = float("nan")
        range_deg = math.degrees(limits.offset_range(self, j))
        if not math.isfinite(deg) or abs(deg) > range_deg:
            return refusal("offset_range", f"an offset of {deg!r} deg is not a zero offset this tool sets (at most "
                           f"{range_deg:g} deg either way)", f"zero_check(joint={joint!r}), then pass the apply_with_deg it reports")
        if joint in st["sign_error"]:
            return refusal("body_sign", f"joint {joint} turns the opposite way to the body model: an offset does not "
                           "fix that", "this needs a person or a corrected robot description, then self_check()")
        now = float(self._sc_jc.offset[j]) / DEG
        rec = st["measured"].get(joint)
        earlier = [r["was_deg"] for r in st["applied"] if r["joint"] == joint]
        if self._sc_valid(rec) and abs(deg - rec["deg"]) <= R.QUANT * (rec["sigma_deg"] + math.degrees(bounds.of(self).joint_quantum()[j])):
            source = rec["source"]
        elif any(abs(deg - x) < 1e-6 for x in earlier):
            source = "restore"
        else:
            if rec is None:
                why = f"no self_check or zero_check has measured joint {joint}'s zero offset"
            elif not self._sc_valid(rec):
                why = (f"the measurement of joint {joint} ({rec['deg']:+.2f} deg by {rec['source']}) is from before "
                       "the world last changed")
            else:
                why = (f"{deg:+.2f} deg is not what {rec['source']} measured for joint {joint} "
                       f"({rec['deg']:+.2f} deg, spread {rec['sigma_deg']:.2f})")
            out = refusal("offset_unmeasured", why, f"run zero_check(joint={joint!r}) or self_check, then pass the "
                          "apply_with_deg it reports")
            out["refused"]["measured"] = None if rec is None else {"deg": rec["deg"], "by": rec["source"],
                                                                   "valid": self._sc_valid(rec)}
            return out
        recs, stale, s = self._sc_set_offsets({joint: deg}, source)
        return {"said": f"Joint {joint}'s offset is now {deg:+.3f} deg, previously {now:+.3f} deg.", "joint": joint, "offset_deg": round(deg, 3), "was_deg": round(now, 3), "source": source,
                "undo": recs[0]["undo"], "tip_mm": _mm(s.tip), "tip_shift_mm": recs[0]["tip_shift_mm"],
                "world_entries_made_stale": stale, "offsets_deg": dict(st.get("offsets_deg", {})),
                "meaning": "readings of this joint now have the offset added; the arm did not move, its believed "
                           "pose did"}


# ---------------------------------------------------------------------------------------------------- hooks
def _pre_motion(ex, name, kwargs):
    st = _state(ex)
    if not st["sign_error"] or name not in MOTION:
        return None
    if name == "wait_for_light" and kwargs.get("park", True) is False:
        return None                          # waits where it is: nothing moves
    last = st["last"] or {}
    return refusal("body_sign", f"self_check {last.get('check_id', '')} found joint "
                   f"{', '.join(st['sign_error'])} turning the opposite way to the robot's body description: a move "
                   "planned from that description would go the wrong way",
                   "do not move. Looks and reads still work. This needs a corrected robot description or a person "
                   "(a hardware check of that joint); run self_check() again after it is fixed")


def _post_camera(ex, name, kwargs, result):
    cam = _state(ex)["camera"]
    if not isinstance(result, dict) or "refused" in result or cam.get("state") not in ("uncertain", "corrected"):
        return None
    if cam["state"] == "uncertain":
        return {**result, "camera_uncertain": {
            "rot_err_deg": cam["rot_err_deg"], "shift_mm_est": cam["shift_mm_est"], "by": cam.get("source"),
            "why": "self_check found the camera is not mounted as the body description says and did not correct it: "
                   "pixel rays, the drawn tip and triangulated points are off by about this much"}}
    if name in ("look", "standard_look"):
        return {**result, "camera_mount": {"corrected_by": cam.get("source"), "turn_deg": cam["mount_turn_deg"],
                                           "shift_mm": cam["mount_shift_mm"]}}
    return None


def _on_base_moved(ex, **fields):
    st = _state(ex)
    st["gen"] += 1
    from rcpx import price_words
    st["stale"] = ("the base moved after the last self_check: its numbers were measured before that; a self_check "
                   "measures them again" + price_words.priced(ex, "self_check"))


def setup(ex):
    ex._sc_jc, ex._sc_cc = comp.install(ex)
    ex.state[PKG] = {"n_check": 0, "n_zero": 0, "gen": 0, "last": None, "zero": {}, "measured": {},
                     "sign_error": [], "stale": None, "camera": {"state": "unchecked"}, "applied": [],
                     "offsets_deg": {}}
    ex.add_pre(lambda name, kwargs: _pre_motion(ex, name, kwargs))
    ex.add_post(lambda name, kwargs, result: _post_camera(ex, name, kwargs, result), names=CAMERA_RESULTS)
    ex.on_event("base_moved", lambda **f: _on_base_moved(ex, **f))


def _body_patch(ex, session, content):
    out = dict(content)
    out["measured"] = measured_doc(ex, _state(ex))
    src = dict(out.get("sources") or {})
    if _state(ex).get("offsets_deg"):
        src["joints.zero_offset"] = "measured"
    if _state(ex)["camera"].get("state") == "corrected":
        src["camera.mount"] = "measured"
    if src:
        out["sources"] = src
    return out


_S = surfaces
_JOINTS = {"anyOf": [{"type": "string"}, {"type": "array", "items": {"type": "string"}, "minItems": 1}],
           "description": "\"all\" (default), one joint name, or a list of joint names from robot://body"}
TOOLS = [
    _S._tool("self_check",
             "Measure the real arm against the robot's body description: wiggles each "
             "joint a few degrees in free space while the wrist camera watches the surface the robot stands on, and "
             "returns per joint whether it turns the right way (sign_ok), how far it turns per reading (gain_ratio) "
             "and its zero offset in degrees, plus the camera's mount error and what it could not judge "
             "(unchecked); verdict ok, compensated (errors found and corrected from now on), uncompensated, "
             "inconclusive or refuse_motion. Zero offsets are read against the configured surface height.",
             _S._obj({"joints": _JOINTS,
                      "amplitude_deg": {"type": "number", "exclusiveMinimum": 0, "description": "the stated turn each way in degrees; omit for the least turn the camera can read"},
                      "clear": {"anyOf": [{"type": "boolean"}, {"type": "object"}]},
                      "apply": _S._bool("correct what was measured from now on (default true); false only reports")}),
             {"joints": "all"},
             "wiggle every joint by the least turn the camera can read, report what is off and correct what the camera could measure"),
    _S._tool("zero_check",
             "Find one joint's mechanical end stop: swings that one joint all the way to an end of its range "
             "(at full joint speed where nothing is known to be near, slowly where the stop can be; the arm and surface "
             "are path checked, and it stops on a stall, not on a light touch, so keep the swept arc clear), reads "
             "the angle where it stalls and returns offset_deg against the declared end with verdict same, moved "
             "or no_stop (no free way to an end, or no stop met); the arm returns to where it was.",
             _S._obj({"joint": _S._str("the joint's name in robot://body"),
                      "end": _S._str("which end to find: auto (the nearer free one, default), low or high",
                                     ["auto", "low", "high"]),
                      "firm": _S._bool("the stop may take the joint's own rated torque; default false: no more than the hand's weight at that joint"),
                      "clear": {"anyOf": [{"type": "boolean"}, {"type": "object"}]}}, ["joint"]),
             {"joint": "elbow", "end": "auto"},
             "run the joint named elbow to its nearer end stop and report how far its zero is off"),
    _S._tool("apply_offset",
             "Set one joint's zero offset in degrees (true angle = reading + offset) so every later reading and "
             "move uses it; only a value that zero_check or self_check measured since the world last changed is "
             "accepted (pass its apply_with_deg), or a value that was active before (the undo it returns).",
             _S._obj({"joint": _S._str("the joint's name in robot://body"),
                      "deg": {"type": "number", "description": "the measured total zero offset, degrees"}},
                     ["joint", "deg"]),
             {"joint": "elbow", "deg": 0},
             "the joint named elbow reads as its encoder says again (offset 0, the undo of an offset applied "
             "before); to apply a measured offset pass the apply_with_deg its zero_check or self_check reported"),
]
RESOURCE_PATCH = {"robot://body": _body_patch}
PRACTICES = ""
