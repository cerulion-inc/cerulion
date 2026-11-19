# SPDX-License-Identifier: AGPL-3.0-only
"""Turn with the joints' free speed and a live torque ramp inside the contact band."""

import math

import numpy as np

from rcpx.r2 import articulate_bounds as A, articulate_grasp as C
from rcpx.r2.articulate_grasp import refusal
from rcpx.r2.articulate_follow import end_index, joint_turn, rot, slip_index, torque_about


class TwistMixin:
    def _ar_range_refusal(self, room):
        return refusal("args", f"the last joint can turn {room:.1f} rad more that way, from the model's joint range",
                       "twist(target='part', angle_deg=-1)")

    def _ar_turn_refusal(self, angle, floor, after, clear, collision=False):
        why=(f"turning {angle:g} deg here brings the hand within {floor*1000:.1f} mm of the surface "
             f"under the arm after {after:g} deg; at most {clear:g} deg is clear this way")
        why += "; the surface clearance is derived from the support uncertainty and body tracking"
        if collision:
            why+="; the body model also finds self collision"
        return refusal("not_reachable", why, "turn less, regrip (twist again from a new grip), or bring the part higher")

    def _ar_turn_plan(self, q, R, angle, part=None):
        B=A.of(self)
        u=-R[:,0]
        anchor,axis=self.model.joint_frames(q)[-1]
        dot=float(axis@u)
        p=self.model.fk_tip(q)[0]
        part=B.thing(p) if part is None else part
        off=p-anchor
        off_axis=float(np.linalg.norm(off-(off@axis)*axis))
        margin=A.band(B,part)+B.rigid(q,u)
        if abs(dot)>0 and A.last_joint_drift(off_axis,dot,np.radians(angle))<=margin+np.finfo(float).eps*len(q):
            goal,room=joint_turn(q[-1],np.radians(angle),dot,self.model.q_lo[-1],self.model.q_hi[-1])
            if goal is None:
                return None,{**self._ar_range_refusal(room),"room_rad":room}
            turn={"how":"last joint","sign":1 if dot>=0 else -1,"goal":goal}
        else:
            turn={"how":"tool axis"}
        floor_margin,self_margin=A.margins(B,self._ar_support()[1])
        # The jaw tips and the hand capsule cover bound the farthest swept hand point.
        corners=[]
        if getattr(self,"_ar_jaws",None) is not None:
            corners.extend(p+R@point for point in self._ar_jaws.points_tool(1))
        for name,a,b,r in self.model.collision_capsules(q):
            if name in getattr(self,"_tool_bodies",()):
                corners.extend((a,b))
        radius=max((float(np.linalg.norm((point-anchor)-((point-anchor)@axis)*axis)) for point in corners),default=off_axis)
        if corners:
            radius+=max((r for name,_,_,r in self.model.collision_capsules(q) if name in getattr(self,"_tool_bodies",())),default=0)
        step=A.turn_sample_rad(B,radius,floor_margin,np.radians(angle))
        count=max(1,math.ceil(abs(np.radians(angle))/step)) if step>0 else 1
        q_sample,clear=q.copy(),0
        for amount in np.linspace(0,abs(angle),count+1):
            signed=float(np.sign(angle)*amount)
            if turn["how"]=="last joint":
                q_sample=q.copy()
                q_sample[-1]+=turn["sign"]*np.radians(signed)
            elif amount:
                q_sample,ok,_=self.model.ik(q_sample,p,R_target=rot(u,np.radians(signed))@R,restarts=4,seed=0)
                if not ok or not self.model.within_limits(q_sample):
                    return None,refusal("args", "no arm pose turns the tool that far here", "turn the other way or regrip")
            floor=C.floor(self,q_sample)
            collision=bool(self.model.self_collisions(q_sample,self_margin))
            if floor<floor_margin or collision:
                return None,self._ar_turn_refusal(angle,floor,amount,clear,collision)
            clear=float(amount)
        weights=self.model.jac(q)[3:].T@u
        tau_plan=self.model.gravity_torque(q)
        turn["ceiling_nm"]=A.torque_ceiling(B,tau_plan,weights,turn["how"]=="last joint")
        return turn,None

    def _ar_twist_loop(self, plan, angle, max_nm):
        B,part,intent=A.of(self),plan["part"],plan["intent"]
        s0=s=C.sense(self)
        q_start=self._q_des.copy()
        u=-s0.R[:,0]
        how=plan["turn"]["how"]
        last=how=="last joint"
        asked=np.radians(angle)
        bad=self.gates("orient_tool",{"roll_deg":-angle,"relative":True,**intent})
        stop,watch="gate" if bad else None,C.Watch(self)
        theta_cmd,measured,previous,settle,off_s=0,0,0,0,0
        torques,advances,gaps,presses=[],[],[],[]
        w_free=B.w_free(s.q,u)
        weights=self.model.jac(s.q)[3:].T@u
        stiffness=A.rotational_k(B,weights,self.kp)
        tolerance=A.angle_tolerance(B,asked,A.angle_gain(B,weights))
        limit=B.limit_s(abs(asked)/w_free+B.t_settle()+max_nm/A.torque_rate(B,0,max_nm,0))
        while stop is None:
            if self._ar_spent():
                stop="steps_spent"
                break
            weights=self.model.jac(s.q)[3:].T@u
            threshold=A.torque_threshold(B,weights,last,w_free)
            torque=torques[-1] if torques else 0
            speed=w_free
            if torque>=min(threshold,max_nm) and torques:
                # Before contact these readings give the dry spread; stiffness is updated only on a resolved rise.
                sigma=float(np.std(torques[:int(A.ratios().QUANT)]))
                speed=min(w_free,A.torque_rate(B,torque,max_nm,sigma)/stiffness)
            theta_cmd=float(np.sign(asked))*min(abs(asked),abs(theta_cmd)+speed*B.dt())
            if torque>=min(threshold,max_nm) and abs(theta_cmd-measured)>A.lead_cap(max_nm,stiffness):
                theta_cmd=measured+np.sign(asked)*A.lead_cap(max_nm,stiffness)
            if last:
                q=q_start.copy()
                q[-1]+=plan["turn"]["sign"]*theta_cmd
                clipped=not self.model.within_limits(q)
            else:
                q,clipped=self._ar_dls(s,s0.tip,rot(u,theta_cmd)@s0.R)
            if clipped:
                stop="limit"
                break
            s=C.period(self,q,kind="follow",target=plan["ref"],intent=intent)
            relative=s.R@s0.R.T
            v=s0.R[:,1]
            principal=float(np.arctan2(u@np.cross(v,relative@v),v@(relative@v)))
            full=math.radians(360)
            half=full/ (1+1)
            measured=previous+((principal-previous+half)%full-half)
            advance=abs(measured-previous)
            advances.append(advance)
            previous=measured
            next_torque=abs(float(s.ext[-1])) if last else abs(torque_about(self.model.jac(s.q)[3:],u,s.ext))
            if torques and advance>A.rotation_resolution(B,weights,speed*B.dt()) and next_torque>torque:
                stiffness=(next_torque-torque)/advance
            torques.append(next_torque)
            hold=self._ar_hold()
            gaps.append(hold["width_mm"]/1000)
            presses.append(hold["pressing"])
            settle=settle+B.dt() if abs(theta_cmd)>=abs(asked) else 0
            off_s=off_s+B.dt() if np.linalg.norm(s.tip-s0.tip)>A.band(B,part)+B.rigid(s.q,u) else 0
            still=bool(np.all(np.abs(s.qd)<=A.joint_still(B,speed)))
            expected=speed*B.dt()
            joint_still=A.rotation_resolution(B,weights,expected)
            if abs(theta_cmd)>=abs(asked) and abs(measured-asked)<=tolerance and (still or settle>=B.t_settle()):
                stop="angle"
            elif end_index(torques,advances,max_nm,A.periods(B),joint_still) is not None:
                stop="end"
            elif slip_index(gaps,A.min_gap_m(B,self._ar_jaws),presses,B.guard("follow").readings) is not None:
                stop="slip"
            else:
                stop=watch.check(s,speed,u,torque_axis=True)
            if stop is None and off_s>=B.t_settle():
                stop="limit"
            if stop is None and s.t-s0.t>=limit:
                stop="time"
        C.stop(self)
        hold=self._ar_hold()
        if stop!="steps_spent":
            self._ar_ease()
        if self._ar_spent():
            stop="steps_spent"
        result={"stopped":stop,"how":how,"turned_deg":float(np.degrees(measured)),"asked_deg":angle,
                "peak_nm":float(max([0]+torques)),"end_nm":max_nm,"hold_stop":hold,"s_end":s,"refusal":bad,"guard":watch.last}
        B.report(result,[plan["torque_item"]])
        C.record(self,{"periods":0,"bounds":result["bounds"]})
        return result

    def _ar_twist_impl(self, plan, angle, max_nm, grip):
        start=C.sense(self).t
        C.begin(self,"grasp")
        grasp=self._ar_grasp(plan,grip)
        grasp_end=C.sense(self).t
        self._ar_stage("twist","grasp",grasp)
        if not grasp.get("held"):
            failed=self._ar_failed_grasp(plan,grasp,"twist",start,grasp_end)
            if "said" in failed and "refused" not in failed:
                failed.update(how=plan["turn"]["how"],turned_deg=0,asked_deg=angle,peak_nm=0,end_nm=max_nm)
            return self._ar_result(failed,[plan["torque_item"]])
        C.begin(self,"twist")
        out=self._ar_twist_loop(plan,angle,max_nm)
        end,hold_stop,bad=out.pop("s_end"),out.pop("hold_stop"),out.pop("refusal")
        turn_end=C.sense(self).t
        self._ar_stage("twist","twist",out)
        C.begin(self,"release")
        released,release=self._ar_release(plan) if out["stopped"]!="steps_spent" else (False,{"stopped":"steps_spent"})
        self._ar_stage("twist","release",release)
        if self._ar_spent():
            out["stopped"]="steps_spent"
        self.note_event("articulate_twist",turned_deg=out["turned_deg"])
        said=("The part reached the end of its turn or is held fast." if out["stopped"]=="end" else
              f"Turned the part {out['turned_deg']:.1f} degrees and stopped at {out['stopped']}.")
        if plan.get("clamped_nm"):
            said += f" The derived torque ceiling stands at {max_nm:g} N m: {plan['torque_item']['evidence']}."
        result={"said":said,**out,"released":released,"hold":{"grasp":grasp["hold"],"stop":hold_stop},
                "approach":plan["approach"],"target":plan["target"],"tip_mm":(end.tip*1000).tolist(),
                "world_s":{"grasp":grasp_end-start,"twist":turn_end-grasp_end,"release":C.sense(self).t-turn_end}}
        if bad:
            result["refusal"]=bad
        if not released:
            result["release"]=release
            if release.get("said")=="The jaws are still on the part.":
                result["said"]+=" The jaws are still on the part."
                result["hint"]="gripper(width_frac='open')"
        result=self._ar_result(result,[plan["torque_item"]])
        self.state["articulate"]["last"]=result
        return result
