# SPDX-License-Identifier: AGPL-3.0-only
"""Follow a held part with a force ramp, a compliance lead and a braked stated travel."""

import numpy as np

from rcpx.r2 import articulate_bounds as A, articulate_grasp as C
from rcpx.r2.articulate_follow import (end_index, fit_path, follow_step, slip_index, trim_path,
                                      unit, window_motion, window_stalled)

STRAIGHT_ONLY = True


class PullMixin:
    def _ar_pull_loop(self, plan, travel, fs):
        B, part, intent = A.of(self), plan["part"], plan["intent"]
        s0 = s = C.sense(self)
        d = -np.asarray(plan["approach"]["a0_b"])
        reach = sum(float(np.linalg.norm(b-a)) for _, a, b, _ in self.model.collision_capsules(s.q))
        planned_travel = travel if travel is not None else A.reach_left_m(B, s.tip, d, reach)
        bad = self.gates("move_tip", dict(zip(("dx", "dy", "dz"), (planned_travel*d*1000).tolist()),
                                         frame="base", **intent))
        k0 = unit(-s0.R[:, 0] - float(-s0.R[:, 0] @ d) * d)
        if not np.any(k0):
            k0 = None
        points, times, walked = [s.tip.copy()], [s.t], [0]
        forces, motions, gaps, presses, expected, at_cap = [], [], [], [], [], []
        k = B.k_servo(s.q, d)
        lead, moved, f_moved = A.lead_cap(fs.first_n, k), 0, None
        off_s, held_s, high_t = 0, 0, -float("inf")
        end_by, stop, watch = None, "gate" if bad else None, C.Watch(self)
        seat = A.seat_m(B, part, s.q, d)
        limit = A.follow_limit_s(B, planned_travel, B.v_free(s.q, d), fs)
        p_cmd = s.tip.copy()
        bound = fs.items[0]
        while stop is None:
            if self._ar_spent():
                stop = "steps_spent"
                break
            remaining = None if travel is None else max(0, travel - moved)
            f_now = max(0, float(s.F @ d))
            speed = A.follow_speed(B, part, s.q, d, f_now, fs.ceiling_n, remaining, intent)
            bound = A.follow_item(B, speed, remaining)
            if f_now >= A.reached_n(B, fs.ceiling_n, speed):
                bound = fs.items[0]
            seated = trim_path(points, seat)
            motion = window_motion(seated, seat) if len(seated) >= int(A.ratios().QUANT) else None
            if f_now < A.reached_n(B, fs.ceiling_n, speed):
                side = s.F - f_now*d
                direction = unit(side) if np.any(side) else s.R[:, 1]
                k_side, dead, min_motion = A.follower(B, s.q, d, speed, direction)
                d = follow_step(d, motion, side, lead, k_side, dead, min_motion, keep_off=k0)
            k = B.k_servo(s.q, d)
            if f_moved is None:
                lead = min(A.lead_cap(fs.ceiling_n, k), lead + fs.rate_n_s(f_now)*B.dt()/k)
            else:
                lead = A.lead_cap(A.worked_cap(B, f_moved, fs.ceiling_n), k)
            # Advance the command at the planned speed, with no servo wind-up past the force lead.
            p_cmd += speed*B.dt()*d
            ahead = float((p_cmd-s.tip) @ d)
            if ahead > lead:
                p_cmd -= (ahead-lead)*d
            if f_now > fs.ceiling_n:
                p_cmd -= min(lead, (f_now-fs.ceiling_n)/k)*d
            first = window_motion(seated, seat, first=True) if len(seated) >= int(A.ratios().QUANT) else None
            newest = window_motion(points, seat)
            back = next((i for i in range(len(walked)-1, -1, -1) if walked[-1]-walked[i] >= seat), None)
            recent = back is not None and times[-1]-times[back] <= A.turn_window_s(B, seat, speed)
            recent = recent and s.t-high_t > B.t_settle()
            if recent and first is not None and newest is not None and k0 is not None:
                first, newest = first-(first@k0)*k0, newest-(newest@k0)*k0
            if STRAIGHT_ONLY and recent and first is not None and newest is not None and A.turned(first,newest,A.ratios().REL):
                stop = "turned"
                break
            q, clipped = self._ar_dls(s, p_cmd, s0.R)
            if clipped:
                stop = "limit"
                break
            floor_margin, self_margin = A.margins(B, self._ar_support()[1])
            if C.floor(self, q) < floor_margin or self.model.self_collisions(q, self_margin):
                stop = "limit"
                break
            prev = s
            s = C.period(self, q, kind="follow", target=plan["ref"], intent=intent)
            delta = s.tip-prev.tip
            progress = float(delta @ d)
            moved += max(0, progress)
            walked.append(walked[-1]+float(np.linalg.norm(delta)))
            points.append(s.tip.copy())
            times.append(s.t)
            forces.append(max(0, float(s.F@d)))
            motions.append(progress)
            if f_moved is None and moved > B.rigid(s.q,d,forces[-1]):
                f_moved = forces[-1]
            hold = self._ar_hold()
            gaps.append(hold["width_mm"]/1000)
            presses.append(hold["pressing"])
            still = A.still_m(B,s.q,d,speed*B.dt())
            off_s = off_s+B.dt() if np.linalg.norm(s.tip-p_cmd)>A.tracking_m(B,s.q,d,speed,lead) else 0
            expected.append(speed * B.dt())
            at_cap.append(ahead >= lead)
            settle_periods = A.settle_periods(B)
            still_window = A.still_m(B, s.q, d, sum(expected[-settle_periods:]))
            reached = A.reached_n(B,fs.ceiling_n,speed)
            held_s = held_s+B.dt() if forces[-1]>=reached else 0
            if forces[-1]>=reached:
                high_t=s.t
            if travel is not None and moved>=travel:
                stop="travel"
            elif end_index(forces,motions,reached,A.periods(B),still) is not None:
                stop,end_by="end","force"
            elif window_stalled(motions,settle_periods,still_window,at_cap):
                stop,end_by="end","stall"
            elif held_s>=B.t_settle():
                stop,end_by="end","held"
            elif slip_index(gaps,A.min_gap_m(B,self._ar_jaws),presses,B.guard("follow").readings) is not None:
                stop="slip"
            else:
                stop=watch.check(s,speed,d)
            if stop is None and off_s>=B.t_settle():
                stop="limit"
            if stop is None and s.t-s0.t>=limit:
                stop="time"
        C.stop(self)
        seated=trim_path(points,seat)
        path=fit_path(seated if len(seated)>=int(A.ratios().QUANT) else points,B.rigid(s.q,d),seat)
        hold=self._ar_hold()
        if stop!="steps_spent":
            self._ar_ease()
        if self._ar_spent():
            stop="steps_spent"
        result={"stopped":stop, **({"end_by":end_by} if stop=="end" else {}), "held":hold["held"],
                "moved_mm":float(moved*1000),"path":path,"f_moved_n":f_moved,
                "peak_force_n":float(max([0]+forces)),"end_force_n":fs.ceiling_n,
                "hold_stop":hold,"s_end":s,"refusal":bad,"guard":watch.last}
        B.report(result,[bound]+list(fs.items))
        C.record(self,{"periods":0,"bounds":result["bounds"]})
        return result

    def _ar_pull_impl(self, plan, travel, fs, clamped):
        B=A.of(self)
        start=C.sense(self).t
        C.begin(self,"grasp")
        grasp=self._ar_grasp(plan)
        self._ar_stage("pull","grasp",grasp)
        grasp_end=C.sense(self).t
        if not grasp.get("held"):
            failed=self._ar_failed_grasp(plan,grasp,"pull",start,grasp_end)
            if "said" in failed and "refused" not in failed:
                failed.update(moved_mm=0,path=fit_path([],B.rigid(plan["q"],plan["R"][:,0]),A.seat_m(B,plan["part"],plan["q"],plan["R"][:,0])),
                              peak_force_n=0,end_force_n=fs.ceiling_n,f_moved_n=None)
            return self._ar_result(failed,fs.items)
        tip0,R0=C.sense(self).tip.copy(),C.sense(self).R.copy()
        C.begin(self,"pull")
        out=self._ar_pull_loop(plan,travel,fs)
        end,bad,hold_stop=out.pop("s_end"),out.pop("refusal"),out.pop("hold_stop")
        pull_end=C.sense(self).t
        self._ar_stage("pull","pull",out)
        entry=plan["entry"]
        if entry is not None:
            p_end=end.tip+end.R@(R0.T@(plan["p"]-tip0))
            data={"f_moved_n":out["f_moved_n"],"f_worked_n":out["f_moved_n"]}
            if out["stopped"]=="end":
                data["end_mm"]=(p_end*1000).tolist()
            self.world.update(entry["id"],point_mm=(p_end*1000).tolist(),data=data,source="articulate: moved with the part")
            others=[e["id"] for e in self.world.query() if e["id"]!=entry["id"] and e.get("point_mm") is not None
                    and np.linalg.norm(np.asarray(e["point_mm"])/1000-plan["p"])<=A.same_spot_m(B,plan["part"],B.thing(e["id"]),self._ar_jaws)]
            self.world.mark_stale(others,"on the part that moved",source="articulate")
        C.begin(self,"release")
        released,release=self._ar_release(plan) if out["stopped"]!="steps_spent" else (False,{"stopped":"steps_spent"})
        self._ar_stage("pull","release",release)
        if self._ar_spent():
            out["stopped"]="steps_spent"
        self.note_event("articulate_pull",moved_mm=out["moved_mm"],shape=out["path"]["shape"])
        said=("The part stopped moving: a stop or a stiff point, or it is held fast." if out["stopped"]=="end" else
              f"Drew the part {out['moved_mm']:.1f} mm and stopped at {out['stopped']}.")
        if clamped:
            said=said.rstrip('.')+f"; the derived end ceiling stands at {fs.ceiling_n:g} N: {fs.items[0]['evidence']}."
        result={"said":said,**out,"released":released,"hold":{"grasp":grasp["hold"],"stop":hold_stop},
                "approach":plan["approach"],"target":plan["target"],"tip_mm":(end.tip*1000).tolist(),
                "world_s":{"grasp":grasp_end-start,"pull":pull_end-grasp_end,"release":C.sense(self).t-pull_end}}
        if plan.get("offered_travel"):
            result["offered_travel"]=plan["offered_travel"]
            result["said"]+=" The offered travel came from this part's stored end."
        if bad:
            result["refusal"]=bad
        if not released:
            result["release"]=release
            if release.get("said")=="The jaws are still on the part.":
                result["said"]+=" The jaws are still on the part."
                result["hint"]="gripper(width_frac='open')"
        binding=out["bounds"]["binding"]
        result=self._ar_result(result,[binding]+list(fs.items))
        self.state["articulate"]["last"]=result
        return result
