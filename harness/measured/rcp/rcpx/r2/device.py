# SPDX-License-Identifier: AGPL-3.0-only
"""Package "device": a device is a timed state machine, and every act on it returns an outcome.

Tools: device_state(id) (from the robot://devices entry and this session's records, never from the device itself)
and watch(target, seconds, rate_hz) (the world idles between wrist frames; verdict off | steady | blinking |
unobservable with a reason out_of_view | occluded_by_robot | dark | cue_may_have_moved). Only a located part that
robot://devices lists as an indicator (device_entry.cue_parts) is read as a device's cue. Hooks on touch and press
(device_acts.py): outcome_class, same_as, device_outcome from the cue before and after, the power hint, the
long-press refusal.
HANDLERS["verify"]: verify checks its evidence look first (the deciding part's located point projects inside the
look, the robot does not cover it, the look is not dim for a small cue, the look comes at least the entry's
transition time after the last act) and returns verdict "unobservable" with a view_hint otherwise; a look that
passes goes to the round one verify (Executor._verify_impl, World.verify); see device_verify.py.

Device kinds are never named: everything comes from the entry's fields (states, transitions, idle_off_s,
long_press_s, power, how_to_tell, parts). No simulator ground truth is read here.
"""

import cv2
import numpy as np

from rcpx import perception, surfaces
from rcpx.context import refusal
from rcpx.control_base import primitive
from rcpx.r2 import device_acts as acts
from rcpx.r2 import device_cue as cue
from rcpx.r2 import device_entry as de
from rcpx.r2 import device_bounds as derived
import math
from rcpx.r2.device_verify import VerifyMixin
from rcpx.r2.device_verify import note_moves as verify_moves

ID = "I19"
MM = 1000.0
PKG = acts.PKG
RANK = {"ok": 0, "dim": 1, "dark": 2}


class Mixin(VerifyMixin):
    # ------------------------------------------------------------------ records and geometry
    def _dv(self):
        return self.state.setdefault(PKG, acts.new_state())

    def _dv_radius_m(self, entry, part):
        size = de.part_size_mm(entry, part) if entry is not None else None
        return size / 2.0 / MM if size else derived.of(self).quantum(self._sense().q, [0, 0, 1])

    def _dv_frame(self):
        return derived.frame_time_s(derived.of(self), self.backend, self.model)[0]

    def _dv_wait(self, entry):
        measured = (self._dv()["cue"].get(de.ident(entry or {})) or {}).get("period_s")
        return derived.wait_s(derived.of(self), entry, self._dv_frame(), measured)

    def _dv_last_act(self, dev):
        mine = [a for a in self._dv()["acts"] if a["device"] == dev]
        return mine[-1] if mine else None

    def _dv_doubt(self, entry):
        """The open doubt on the device's cue point (its cue went lit -> dark there across an act, so it may have
        switched off or moved with a part), or None. It ends when the cue part is located anew."""
        if entry is None:
            return None
        dev = de.ident(entry)
        d = self._dv()["doubt"].get(dev)
        if d is None:
            return None
        loc = de.cue_located(self, entry)
        if loc is None or loc["id"] != d["loc_id"] or loc["at_call"] != d["loc_call"]:
            self._dv()["doubt"].pop(dev, None)
            return None
        return d

    def _dv_doubted(self, doubt):
        return {"verdict": "unobservable", "reason": "cue_may_have_moved", "period_s": None,
                "why": f"the cue went dark at its located point ('{doubt['label']}') across call "
                       f"{doubt['since_call']}: "
                       "it may have switched off there, or moved away with a part that act moved",
                "hint": f"look; if the part that carries the cue moved, mark the cue where it is now in two new looks "
                        f"under a new label that names the part (for example '{doubt['label']} 2'; marks under the "
                        "old label still hold the old looks), triangulate that label, then watch"}

    def _dv_watch(self, dev, geom, seconds, rate_hz, save, register=True):
        """Watch one cue patch from the present pose: the series, the verdict and the record. geom["is_cue"]: the
        patch is the device's cue (its reading becomes the device's cue reading)."""
        st = self._dv()
        st["n_watch"] += 1
        wid = f"W{st['n_watch']}"
        q, t0 = self._sense().q.copy(), self.sim_time()
        out = {"watch_id": wid, "device": dev, "label": geom.get("label"), "px": None, "radius_px": None,
               "seconds": seconds, "rate_hz": rate_hz, "started_s": round(t0, 3), "series": [], "period_s": None}
        if geom.get("u") is not None and geom.get("v") is not None:
            out["px"] = [geom["u"], geom["v"]]
            out["radius_px"] = geom.get("radius_px")
        cs_all, frames = [], {}
        if not geom.get("in_frame"):
            res = {"verdict": "unobservable", "reason": "out_of_view", "why": geom.get("why") or "not in the frame"}
        elif geom.get("blocked_by"):
            res = {"verdict": "unobservable", "reason": "occluded_by_robot",
                   "why": (f"the robot's {geom['blocked_by']} fills most of the cue's surroundings in this view, so "
                           "the cue cannot be read against them" if geom.get("blocked_how") == "surround" else
                           f"the robot's {geom['blocked_by']} covers the line of sight to the cue in this view")}
        else:
            samples, first, last = [], None, None
            n = int(round(seconds * rate_hz)) + 1
            for k in range(n):
                img = np.asarray(self.backend.render("wrist"))
                cs = perception.can_see(img)
                lv = cue.cue_level(img, geom["u"], geom["v"], geom["radius_px"], geom.get("cover"))
                t = self.sim_time()
                samples.append((t, lv, cs))
                cs_all.append(cs)
                out["series"].append({"at_s": round(t - t0, 3), "lit": None if lv is None else lv["lit"],
                                      "level": None if lv is None else lv["peak"],
                                      "around": None if lv is None else lv["ring"]})
                first = img if first is None else first
                last = img
                if k < n - 1:
                    self.idle(1.0 / rate_hz)
            res = cue.classify(samples, rate_hz)
            if res.get("reason") == "dark":
                res["why"] = "too dark around the cue to tell off from unlit"
            elif res.get("reason") == "out_of_view":
                res["why"] = "the cue patch fell off the frame"
            # a lit reading does not end a doubt: a bright edge where a moved part was can read lit too
            doubt = self._dv_doubt(de.find(self, dev)) if geom.get("is_cue") and geom.get("point_mm") else None
            if doubt is not None and res["verdict"] == "off":
                res = {**self._dv_doubted(doubt), "min_period_s": res.get("min_period_s")}
            if save and first is not None:
                ann = np.ascontiguousarray(first[:, :, :3]).copy()
                c = (int(round(geom["u"])), int(round(geom["v"])))
                cv2.circle(ann, c, int(round(geom["radius_px"])), (255, 0, 255), 1, cv2.LINE_AA)
                for stem, img in ((f"{wid}_first_marked", ann), (f"{wid}_last", last)):
                    path = self._save(img, stem)
                    frames[stem.split("_", 1)[1]] = path if isinstance(path, str) else None
        out.update(res)
        if frames:
            out["frames"] = frames
        out["ended_s"] = round(self.sim_time(), 3)
        worst = max(cs_all, key=lambda c: RANK.get(c, 0)) if cs_all else None
        out["can_see"] = worst
        observable = out["verdict"] != "unobservable"
        st["watches"][wid] = {"q": q, "t": t0, "t_end": out["ended_s"], "verdict": out["verdict"],
                              "reason": out.get("reason"), "device": dev, "epoch": self.epoch,
                              "cue": bool(geom.get("is_cue")),
                              "can_see": "dim" if observable and worst != "ok" else (worst or "ok")}
        if observable and register:
            self.world.note_look(wid, st["watches"][wid]["can_see"], frame=frames.get("first_marked"))
        if observable and dev and geom.get("is_cue"):
            st["cue"][dev] = {"verdict": out["verdict"], "id": wid, "t": out["ended_s"], "epoch": self.epoch, "period_s": out.get("period_s")}
        return out

    def _dv_point_geom(self, point_mm, radius_m, label, q=None, surround=False):
        """Where a located point falls in the wrist view at q. surround: also where the robot's own body lies
        around it (cue.patch_cover), for a reading; a patch whose ring is mostly the robot counts as covered."""
        q = self._sense().q if q is None else q
        p = np.asarray(point_mm, float) / MM
        pv = cue.point_view(self.model, q, p, radius_m)
        out = {**pv, "label": label, "point_mm": [round(float(x), 1) for x in point_mm]}
        if surround and pv["in_frame"] and not pv["blocked_by"] and pv["radius_px"]:
            reach = float(np.linalg.norm(p - self.model.fk(q)["camera"][0])) + self.model.length_scale
            self._dv_surround(out, q, reach)
        return out

    def _dv_surround(self, geom, q, reach_m):
        cov = cue.patch_cover(self.model, q, geom["u"], geom["v"], geom["radius_px"], reach_m)
        geom["cover"] = cov
        if cov["covered"]:
            geom["blocked_by"], geom["blocked_how"] = cov["by"], "surround"

    def _dv_hint(self, point_mm, radius_m):
        q = self._sense().q
        return cue.view_hint(self.model, q, np.asarray(point_mm, float) / MM, radius_m, valid=self._valid, B=derived.of(self))

    def _dv_cue_before(self, entry):
        """The cue just before an act: a recent reading with no act since, else a short burst of frames."""
        st, dev = self._dv(), de.ident(entry)
        pt, label, part = de.cue_point(self, entry)
        if pt is None:
            return {"verdict": "unobservable", "reason": "no_cue_point"}
        now, rec, last = self.sim_time(), st["cue"].get(dev), self._dv_last_act(dev)
        if (rec and rec["epoch"] == self.epoch and now - rec["t"] <= self._dv_wait(entry)[0]
                and (last is None or rec["t"] >= last["t_end"])):
            return {"verdict": rec["verdict"], "from": rec["id"], "age_s": round(now - rec["t"], 3)}
        geom = self._dv_point_geom(pt, self._dv_radius_m(entry, part), label, surround=True)
        if not geom["in_frame"] or geom["blocked_by"]:
            return {"verdict": "unobservable", "reason": "out_of_view" if not geom["in_frame"] else "occluded_by_robot"}
        n, gap, source, evidence = derived.burst(derived.of(self), entry, self._dv_frame(),
                                                   (rec or {}).get("period_s"))
        lits = []
        for k in range(n):
            img = np.asarray(self.backend.render("wrist"))
            lits.append((cue.cue_level(img, geom["u"], geom["v"], geom["radius_px"], geom.get("cover")),
                         perception.can_see(img)))
            if k < n - 1:
                self.idle(gap)
        if any(lv and lv["lit"] for lv, _ in lits):
            return {"verdict": "lit", "frames": n}
        if all(lv is None or lv["dark"] or cs == "dark" for lv, cs in lits):
            return {"verdict": "unobservable", "reason": "dark"}
        doubt = self._dv_doubt(entry)
        if doubt is not None:
            return {k: v for k, v in self._dv_doubted(doubt).items() if k in ("verdict", "reason", "why")}
        return {"verdict": "off", "frames": n}

    def _dv_cue_after(self, entry, seconds):
        pt, label, part = de.cue_point(self, entry)
        if pt is None:
            return {"verdict": "unobservable", "reason": "no_cue_point"}
        s, _ = self._ease_if_pressing()
        geom = self._dv_point_geom(pt, self._dv_radius_m(entry, part), label, s.q, surround=True)
        geom["is_cue"] = True
        w = self._dv_watch(de.ident(entry), geom, seconds, 1 / self._dv_frame(), save=False)
        out = {"verdict": w["verdict"], "watch_id": w["watch_id"], "period_s": w.get("period_s")}
        if w.get("reason"):
            out["reason"] = w["reason"]
        return out

    # ------------------------------------------------------------------ watch
    def _dv_target(self, target, q):
        m = self.model
        if isinstance(target, dict):
            px = target.get("px")
            if not (isinstance(px, (list, tuple)) and len(px) == 2 and all(isinstance(x, (int, float)) for x in px)):
                return None, refusal("target", "a pixel target needs px: [u, v]", "e.g. {\"px\": [320, 200], "
                                     "\"radius_px\": 8}, u right and v down from the top-left of the present view")
            u, v = float(px[0]), float(px[1])
            if not (0 <= u < m.cam_w and 0 <= v < m.cam_h):
                return None, refusal("pixel", f"pixel ({u:g}, {v:g}) is outside the {m.cam_w} x {m.cam_h} frame",
                                     "u from 0 to width, v from 0 to height, from the top-left")
            r = float(target.get("radius_px", 8.0))
            if not (1.0 <= r <= 100.0):
                return None, refusal("radius_px", f"radius_px {r:g} is outside 1 to 100", "give the cue's radius")
            entry = de.find(self, target.get("device")) if target.get("device") else None
            dev = de.ident(entry) if entry else (str(target["device"]) if target.get("device") else None)
            blocked = cue.pixel_blocked(m, q, u, v, 2.0 * m.length_scale)
            geom = {"u": round(u, 1), "v": round(v, 1), "radius_px": r, "in_frame": True, "blocked_by": blocked,
                    "label": None, "device": dev, "point_mm": None, "is_cue": dev is not None}
            if blocked is None:
                self._dv_surround(geom, q, 2.0 * m.length_scale)
            return geom, None
        if not isinstance(target, str) or not target.strip():
            return None, refusal("target", "target must be a device id, a located label or {px, radius_px}",
                                 "e.g. \"device\" or {\"px\": [320, 200], \"radius_px\": 8}")
        lab = target.strip()
        named = de.find(self, lab)
        hits = [e for e in self.world.query(kind="located", label=lab, include_stale=False) if e["point_mm"]]
        entry, part, point = None, None, None
        if named is not None and de.cue_point(self, named)[0] is not None:
            entry = named                    # a device id: its located cue comes before a point of the same name
            point, lab, part = de.cue_point(self, entry)
        elif hits:
            point = hits[-1]["point_mm"]
            for e in de.entries(self):
                ok, p = de.label_part(lab, e)
                if ok:
                    entry, part = e, p
                    break
        elif named is None:
            ids = [de.ident(e) for e in de.entries(self)]
            return None, refusal("target", f"no device and no located point named {lab!r}",
                                 ("devices: " + ", ".join(ids) if ids else "robot://devices lists no device")
                                 + "; or locate the cue (mark it in two looks, triangulate) or pass {px, radius_px}")
        else:
            names = [de.part_name(p) for p in de.cue_parts(named)]
            return None, refusal("cue_point", f"no located point for the cue of {de.ident(named)}",
                                 ("mark the part that shows its state" + f" ({', '.join(names)})"
                                  + " in two looks, labelled with that part's name, triangulate it, then watch; "
                                  if names else f"robot://devices lists no part of {de.ident(named)} that only "
                                  "shows its state (a lamp or other indicator): mark the indicator you see in "
                                  "two looks, triangulate it and watch that label; ")
                                 + "or watch {px, radius_px, device} in the present view")
        geom = self._dv_point_geom(point, self._dv_radius_m(entry, part), lab, q, surround=True)
        geom["device"] = de.ident(entry) if entry else None
        geom["is_cue"] = entry is not None and de.is_cue_part(entry, part)
        return geom, None

    @primitive
    def watch(self, target, seconds=None, rate_hz=None):
        entry = de.find(self, target) if isinstance(target, str) else None
        seconds = self._dv_wait(entry)[0] if seconds is None else float(seconds)
        camera_rate = 1 / self._dv_frame()
        asked_rate = camera_rate if rate_hz is None else float(rate_hz)
        rate_hz = min(camera_rate, asked_rate)
        if not (math.isfinite(seconds) and math.isfinite(rate_hz) and seconds >= 0 and rate_hz > 0):
            return refusal("args", "seconds and rate must be finite, with nonnegative time and positive rate", "watch(target=" + repr(target) + ")")
        s, eased = self._ease_if_pressing()
        geom, bad = self._dv_target(target, s.q)
        if bad is not None:
            return bad
        out = self._dv_watch(geom.get("device"), geom, seconds, rate_hz, save=True)
        out["target"] = target
        if asked_rate > camera_rate:
            out["rate_note"] = f"model: camera frame rate bounds the requested rate at {camera_rate:g} Hz"
        if (out["verdict"] == "unobservable" and geom.get("point_mm") is not None
                and out["reason"] in ("out_of_view", "occluded_by_robot")):
            out["view_hint"] = self._dv_hint(geom["point_mm"], self._dv_radius_m(de.find(self, geom.get("device")),
                                                                                  None))
        elif out["verdict"] == "unobservable" and out["reason"] == "dark":
            out["hint"] = "get light on the scene, or wait for light, then watch again"
        if not geom.get("is_cue") and geom.get("device"):
            out["cue_note"] = ("this point is not a part robot://devices lists as the device's indicator, so its "
                               "reading is not taken as the device's cue")
        if out["verdict"] == "blinking":
            out["note"] = f"a blink faster than one per {out['min_period_s']:g} s cannot be measured at this rate"
        if eased:
            out["eased"] = "a joint was still pressing; eased before watching"
        return out

    # ------------------------------------------------------------------ device_state
    def _dv_power(self, entry, dev):
        st = self._dv()
        p = (entry or {}).get("power")
        out = dict(p) if isinstance(p, dict) else ({"value": p, "source": "robot://devices"} if p else
                                                     {"value": "unknown", "source": "nothing stated"})
        lit = [w for w in st["watches"].values() if w["device"] == dev and w.get("cue")
               and cue.cue_class(w["verdict"]) == "lit"]
        sus = st["power"].get(dev)
        if lit and (sus is None or lit[-1]["t_end"] >= sus["seen_at_s"]):
            out = {"value": "on", "source": "its cue was seen lit in this session", "seen_at_s": lit[-1]["t_end"]}
        elif sus is not None:
            out = dict(sus)
        return out

    @primitive
    def device_state(self, id):
        st = self._dv()
        entry = de.find(self, id)
        dev = de.ident(entry) if entry else str(id)
        mine = [a for a in st["acts"] if a["device"] == dev]
        if entry is None and not mine and dev not in st["cue"]:
            ids = [de.ident(e) for e in de.entries(self)]
            return refusal("device", f"no device {id!r} in robot://devices and no act on it in this session",
                           "devices: " + ", ".join(ids) if ids else "robot://devices lists no device here")
        now = self.sim_time()
        claim = self.world.device_states().get(dev)
        rec, last = st["cue"].get(dev), (mine[-1] if mine else None)
        out = {"id": dev, "state": claim["state"] if claim else "unknown",
               "confirmed": bool(claim and claim["confirmed"]), "claim_id": claim["claim_id"] if claim else None,
               "states": (entry or {}).get("states"), "cue": None, "last_act": None, "power": self._dv_power(entry, dev),
               "next_timeout_s": None, "acts_this_epoch": sum(1 for a in mine if a["epoch"] == self.epoch),
               "source": "the robot://devices entry and this session's acts, watches and claims"}
        if rec is not None:
            out["cue"] = {"reading": rec["verdict"], "look_id": rec["id"], "age_s": round(now - rec["t"], 3)}
            out["cue_suggests"] = ("lit: the device shows power" if cue.cue_class(rec["verdict"]) == "lit" else
                                   "dark: asleep, switched off or unpowered, or this cue is dark in its present state")
        if last is not None:
            out["last_act"] = {"call": last["call"], "primitive": last["primitive"],
                               "age_s": round(now - last["t_end"], 3), "outcome_class": last["outcome_class"],
                               "device_outcome": last["device_outcome"]}
        doubt = self._dv_doubt(entry)
        out["cue_doubt"] = None if doubt is None else {k: v for k, v in self._dv_doubted(doubt).items()
                                                       if k in ("reason", "why", "hint")}
        idle = de.num_range(entry or {}, "idle_off_s")
        if idle and last is not None and (rec is None or cue.cue_class(rec["verdict"]) != "dark"):
            changed = [a for a in mine if a.get("cue_changed")]
            lo, hi = acts.timeout_range(idle, now, changed[-1]["t_start"] if changed else None, last["t_end"])
            out["next_timeout_s"] = (round(lo, 1) if lo == hi else {"min": round(lo, 1), "max": round(hi, 1)})
            out["timeout_from"] = ("idle_off_s in robot://devices, from the device's last input: no earlier than the "
                                   "last act that changed its cue (min 0 when none did), no later than the last act")
        lp = de.num_range(entry or {}, "long_press_s")
        if lp:
            out["long_press_s"] = lp[0]
        return out


def setup(ex):
    acts.setup(ex)
    verify_moves(ex)


def _verify_handler(ex, session, **args):
    return ex.verify_evidence(**args)


_S = surfaces
_PX = {"type": "array", "items": {"type": "number", "minimum": 0, "maximum": 100000}, "minItems": 2, "maxItems": 2,
       "description": "[u, v]: u right, v down, from the top-left of the present view"}
_CUE_ARGS = {"expect_cue": _S._str("the cue you expect on the device after this act (off, steady or blinking); "
                                   "device_outcome compares with it", ["off", "steady", "blinking"]),
             "cue_watch_s": {"type": "number", "minimum": 0, "description": "seconds to watch after the act; the device timing is offered; zero skips readings"}}
_OUTCOME_TEXT = ("outcome_class: ran_free_beyond_tip (felt nothing), touched_no_give (did not move), gave (let go: gave "
                 "true), moved_under_load (moved over its rigid travel at the measured peak force, gave false) or slid_off")
TOOLS = [
    _S._tool("device_state",
             "Read what this session knows about one device: the state last claimed and whether it is confirmed, "
             "the last reading of its cue, the last act on it and what came of it, power evidence, and the time "
             "left before it may switch itself off; built from robot://devices and this session's records. Free: "
             "it does not use up a call.",
             _S._obj({"id": _S._str("the device's id in robot://devices")}, ["id"]),
             {"id": "device"}, "what is known right now about the device whose id is device"),
    _S._tool("watch",
             "Hold still and watch a device's cue (a small light or other indicator) for its sheet transition time while the "
             "world runs; returns off, steady or blinking (with period_s), or unobservable with a reason "
             "(out_of_view, occluded_by_robot, dark, or cue_may_have_moved once the cue went dark at its located "
             "point across an act) and, for a located point out of view or hidden, a view_hint; target is a device "
             "id or a located label (a cue triangulated from two looks, labelled with the cue part's name) or "
             "{px, radius_px} in the present view.",
             _S._obj({"target": {"anyOf": [{"type": "string"},
                                           _S._obj({"px": _PX, "radius_px": _S._num(1, 100, "the cue's radius in "
                                                                                             "pixels (default 8)"),
                                                    "device": _S._str("the device this cue belongs to (optional)")},
                                                   ["px"])],
                                 "description": "a device id, a located label, or {px: [u, v], radius_px}"},
                      "seconds": {"type": "number", "minimum": 0, "description": "watch time in seconds; the device timing is offered"},
                      "rate_hz": {"type": "number", "description": "frames per second, bounded by the camera frame rate"}}, ["target"]),
             {"target": "device"},
             "watch the device's located cue at its derived timing"),
    {"extends": "touch", "args": dict(_CUE_ARGS),
     "note": "On a device the result adds outcome_class, same_as and, once its cue is located, device_outcome from "
             "the cue before and after (outcome_class as for press)."},
    {"extends": "press", "args": dict(_CUE_ARGS),
     "note": "On a device the result adds outcome_class, same_as and device_outcome. " + _OUTCOME_TEXT + ". A "
             "press that could keep the tip "
             "on the part past the device's long_press_s (if the part gave the whole travel) is refused before it "
             "moves, and the refusal names the longest travel that fits."},
    {"extends": "verify", "args": {},
     "note": "Here only a reading of the robot's own confirms: verdict confirmed when it read the deciding cue's "
             "light (in your look, or in a look it takes itself where that needs no motion, 0 control periods) and "
             "it agrees; not_checked when the claim rests on your evidence alone (accepted false, the record stays "
             "unconfirmed; said names what the robot read, what it could not check and what would check it); "
             "contradicted when the cue disagrees; unobservable when the look cannot show the deciding part clear "
             "of the robot after the device's transition time (a view_hint when a move would help)."},
]
HANDLERS = {"verify": _verify_handler}
PRACTICES = ("A device's cue is read by watch and device_state (0 control periods); device_outcome says what the "
             "robot itself read change after an act. A long hold on a device can switch it off (long_press_s in "
             "robot://devices).")
