# SPDX-License-Identifier: AGPL-3.0-only
"""Mobile interface for any backend offering base odometry and a height joint.

New primitives: drive, torso and base_pose; free resource robot://base. The robot
frame rides both motions. Every motion with a control period re-expresses fresh
World positions and their documented data vectors, updates the floor, invalidates
all earlier looks and marks, clears old arm paths and standard references, and
emits base_driven. No world epoch or measured base give is changed here.

Private state is documented in mobile_world. Other packages listen to base_driven:
primitive, moved, pose, T_new_old_mm, records_moved and ids. Only documented World
data directions and points and the documented witness act points are transformed;
other private geometry is left to its owner. Settings and initial floor policy are
documented in mobile_setup.
"""

import numpy as np

from rcpx.context import refusal
from rcpx.control_base import primitive
from rcpx.r2 import mobile_bounds as D, mobile_geom as G, mobile_motion as M, mobile_setup as S
from rcpx.r2.mobile_setup import FILTER, setup
from rcpx.r2.mobile_tools import PRACTICES, TOOLS, drive_said, torso_said

ID = "M1"
RESOURCES = {"robot://base": (S.base_document, "Base odometry, height, travel, limits and footprint without stepping.")}
RESOURCE_PATCH = {"robot://body": S.body_patch}


class Mixin:
    def _mb_info(self):
        return self.state.get("mobile", {}).get("info")

    def _mb_refuse(self, check, why, hint):
        return {"said": f"Refused, nothing moved: {why}.", **refusal(check, why, hint)}

    def _mb_inert(self):
        return self._mb_refuse("not_mobile", "this robot has no mobile base or height joint it can drive",
                               "look()")

    @primitive
    def drive(self,forward_mm=0,left_mm=0,turn_deg=0,clearance_mm=None,clear=None):
        info = self._mb_info()
        if info is None:
            return self._mb_inert()
        for key,value in (('forward_mm',forward_mm),('left_mm',left_mm),('turn_deg',turn_deg)):
            if not S.finite(value):
                return self._mb_refuse('args',key+' must be a finite number','base_pose()')
        if abs(turn_deg) > 360:
            return self._mb_refuse('args','a turn must fit within one full turn','base_pose()')
        if clearance_mm is not None and (not S.finite(clearance_mm) or clearance_mm < 0):
            return self._mb_refuse('args','clearance_mm must be a finite nonnegative number','base_pose()')
        if forward_mm == left_mm == turn_deg == 0:
            return self._mb_refuse('args','nothing to do','base_pose()')
        if info['drive'] == 'differential' and (left_mm != 0 or forward_mm != 0 and turn_deg != 0):
            return self._mb_refuse('not_holonomic','this base cannot move sideways or drive straight while turning','base_pose()')
        B = D.of(self)
        intent = {'clear':clear,'travel':float(np.hypot(forward_mm,left_mm))/1000}
        B.begin('drive')
        if clear is not None:
            word = B.word(intent)
            if word.get('refused'):
                return word['refused']
            intent['look'] = word.get('look')
            D.end_word(B)
        know = self.state['mobile']['know']
        odo0 = dict(self.backend.odometry())
        v,w,unused = D.base_limits(info)
        bad = D.available(B,know,v,float(np.linalg.norm(self.model.mj.opt.gravity)))
        if bad is not None:
            return bad
        stated = None if clearance_mm is None else clearance_mm/1000
        swept_speed = D.swept_speed(info,float(np.hypot(forward_mm,left_mm))/1000,np.deg2rad(turn_deg),M.radius(info))
        margin = D.clearance(B,know,swept_speed,0,0,stated)
        step,angular_step = D.sweep_step(B,margin,M.radius(info))
        poses = G.sweep_poses(forward_mm,left_mm,turn_deg,step*1000,np.rad2deg(angular_step))
        static = lambda e: max(M.point_sigma(B,e)+M.point_error(e),stated or 0)
        top = lambda e: D.top_margin(M.point_sigma(B,e),M.point_error(e))
        entries = list(self.world.entries.values())
        blocked,nearest = G.path_check(entries,poses,info,odo0['torso_m'],static,top,D.comparison_slack(know))
        if blocked:
            bound = D.item('travel',static(blocked),'m','seen','stored point '+blocked['id']+' and its odometry uncertainty')
            r = {**self._mb_refuse('path_blocked',f"{blocked['id']} lies inside the static footprint margin: {bound['evidence']}",'look()'), 'blocking':blocked}
            return B.report(r,[bound])
        q = M.sense(self).q
        capsules = self.model.collision_capsules(q)
        swept_travel = float(np.hypot(forward_mm,left_mm))/1000+abs(np.deg2rad(turn_deg))*M.radius(info)
        period_step = np.asarray([forward_mm/1000,left_mm/1000,np.deg2rad(turn_deg)])*B.dt()*swept_speed/swept_travel
        clearance_fn = lambda e,pose: D.clearance(B,know,M.speed(self,pose,np.asarray(pose)+period_step,
                         info,odo0['torso_m'],swept_speed,stated)[0],M.point_sigma(B,e),M.point_error(e),stated)
        blocked,arm_nearest = G.arm_sweep(entries,capsules,poses,info,odo0['torso_m'],clearance_fn,D.comparison_slack(know))
        if blocked:
            e = next(e for e in entries if e['id'] == blocked['id'])
            bound = D.item('travel',clearance_fn(e,poses[0]),'m','model',f"the base stopping distance, point {e['id']} uncertainty and odometry error")
            r = {**self._mb_refuse('arm_in_the_way',f"{blocked['link']} would pass {blocked['id']} inside the derived clearance: {bound['evidence']}",'home()'), 'blocking':blocked}
            return B.report(r,[bound])
        target = np.asarray(G.pose_compose((odo0['x_m'],odo0['y_m'],odo0['yaw_rad']),forward_mm/1000,left_mm/1000,np.deg2rad(turn_deg)))
        r = M.motion(self,'drive',odo0,target,intent=intent,clearance_m=stated)
        r.pop('odo')
        return {'said':drive_said(r['stopped'],r['moved'],r['records_moved']),
                'asked':{'forward_mm':float(forward_mm),'left_mm':float(left_mm),'turn_deg':float(turn_deg)},
                'arm_nearest':arm_nearest,**r}

    @primitive
    def torso(self,height_mm,clear=None):
        info = self._mb_info()
        if info is None:
            return self._mb_inert()
        if info['torso'] is None:
            return self._mb_refuse('no_torso','this arm has no height joint','base_pose()')
        if not S.finite(height_mm):
            return self._mb_refuse('args','height_mm must be a finite number','base_pose()')
        B = D.of(self)
        intent = {'clear':clear}
        B.begin('torso')
        if clear is not None:
            word = B.word(intent)
            if word.get('refused'):
                return word['refused']
            intent['look'] = word.get('look')
            D.end_word(B)
        bad = D.available(B,self.state['mobile']['know'],D.base_limits(info)[-1],float(np.linalg.norm(self.model.mj.opt.gravity)))
        if bad is not None:
            return bad
        odo0 = dict(self.backend.odometry())
        lo,hi = info['torso']['range_m']
        target = float(np.clip(height_mm/1000,lo,hi))
        clamped = height_mm/1000 < lo or height_mm/1000 > hi
        dh = target-odo0['torso_m']
        if abs(dh) <= D.ratios().QUANT * (self.state['mobile']['know'].get('quantum_m') or 0):
            self.backend.hold_base()
            r = dict(stopped='done',odo=odo0,seconds=0,periods=0,records_moved=0,peak_force_n=M.force(self,M.sense(self)))
            B.report(r,[D.speed_item(B,self.state['mobile']['know'],0,D.base_limits(info)[-1],intent=intent)])
        else:
            r = M.motion(self,'torso',odo0,target,intent=intent)
        odo1 = r.pop('odo')
        r.pop('moved',None)
        r.pop('pose',None)
        h = odo1['torso_m']*1000
        return {'said':torso_said(r['stopped'],h,clamped,r['records_moved']),'height_mm':h,
                'moved_mm':(odo1['torso_m']-odo0['torso_m'])*1000,'asked_mm':float(height_mm),
                'clamped':bool(clamped),**r,'floor_z_mm':self.state['mobile']['floor_z_m']*1000}

    @primitive
    def base_pose(self):
        if self._mb_info() is None:
            return self._mb_inert()
        doc = S.base_document(self)
        p = doc["pose"]
        return {"said": f"The base is {p['x_mm']:.1f} mm forward and {p['y_mm']:.1f} mm left of where it started, "
                        f"turned {p['yaw_deg']:.2f} deg, with the torso at {doc['height_mm']:.1f} mm.", **doc}
