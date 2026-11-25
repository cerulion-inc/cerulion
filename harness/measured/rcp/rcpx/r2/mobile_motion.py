# SPDX-License-Identifier: AGPL-3.0-only
"""Adaptive base waypoints, contact and progress watches, and session self-knowledge."""
import math
import numpy as np
from rcpx.r2 import mobile_bounds as D, mobile_geom as G, mobile_setup as S, mobile_world as W


def sense(ex):
    return ex._sense()


def period(ex):
    return ex._period(ex._q_des)


def ease(ex):
    return ex._ease_impl()


def force(ex, s):
    reading = ex.backend.read()
    if 'wrist_force_n' in reading:
        return float(np.linalg.norm(np.asarray(reading['wrist_force_n'],dtype=float).reshape(3)))
    return float(s.Fmag)


def radius(info):
    fp = info['footprint_b']
    return max(float(np.hypot(x,y)) for x in fp['x'] for y in fp['y'])


def point_error(e):
    return float(((e.get('data') or {}).get('odometry') or {}).get('est_error_mm') or 0) / 1000


def point_sigma(B,e):
    return float(getattr(B.thing(e['id']),'sigma_p',0) or 0)


def speed(ex,pose,next_pose,info,torso_m,vmax,clearance_m=None):
    B,know = D.of(ex),ex.state['mobile']['know']
    best,point = vmax,None
    top = G.footprint_now(info,torso_m)['z_top']
    for e in G.fresh_points(ex.world.entries.values()):
        sigma,err = point_sigma(B,e),point_error(e)
        if e['point_mm'][2]/1000 > top+D.top_margin(sigma,err):
            continue
        d = G.current_distance(e,pose,info,torso_m)
        future = G.current_distance(e,next_pose,info,torso_m)
        if future > d:
            continue
        offer = D.speed_at(min(d,future),sigma,err,D.stop_time(B,know),vmax)
        if offer < best:
            best,point = offer,e
    return best,point


def motion(ex,name,odo0,target,*,intent=None,clearance_m=None):
    be,dt,B = ex.backend,ex.dt,D.of(ex)
    st,info = ex.state['mobile'],ex.state['mobile']['info']
    know = st['know']
    height = name == 'torso'
    start = np.array([odo0['x_m'],odo0['y_m'],odo0['yaw_rad']])
    initial = odo0['torso_m'] if height else start
    change = target-initial
    distance = abs(change) if height else float(np.linalg.norm(change[:2]))
    angle = 0 if height else abs(float(change[2]))
    v,w,torso_v = D.base_limits(info)
    corner = radius(info)
    vmax = torso_v if height else D.swept_speed(info,distance,angle,corner)
    rate = min(([v/distance] if distance and not height else [torso_v/distance] if distance else []) +
               ([w/angle] if angle else []))
    planned = 1/rate
    local = np.array(change,copy=True)
    if not height:
        c,s = np.cos(start[2]),np.sin(start[2])
        local[0],local[1] = c*change[0]+s*change[1],-s*change[0]+c*change[1]
    # Integrate the same speed law over geometry samples to time a slowed path before moving.
    if not height:
        margin = D.clearance(B,know,vmax,0,0,clearance_m)
        step,step_rad = D.sweep_step(B,margin,corner)
        poses = G.sweep_poses(distance*1000,0,math.degrees(angle),step*1000,math.degrees(step_rad))
        planned = 0
        for k in range(len(poses)-1):
            a,b = k/(len(poses)-1),(k+1)/(len(poses)-1)
            p = (local[0]*a,local[1]*a,local[2]*a)
            pn = (local[0]*b,local[1]*b,local[2]*b)
            offer,_ = speed(ex,p,pn,info,odo0['torso_m'],vmax,clearance_m)
            if offer <= 0:
                return {'stopped':'path_blocked','odo':odo0,'periods':0,'seconds':0,'records_moved':0,
                        'moved':{'forward_mm':0,'left_mm':0,'turn_deg':0},'pose':S.pose(odo0)}
            planned += (b-a)/(rate*min(1,offer/vmax))
    time_limit = B.limit_s(planned+D.settle_time(B,know))
    settle_limit = B.limit_s(D.settle_time(B,know))
    max_periods = math.ceil(time_limit/dt)
    window = D.stall_window(B,know)
    watch_quantum_m,watch_quantum_rad = know.get('quantum_m') or 0,know.get('quantum_rad') or 0
    # The watch uses knowledge available at the call's start. A newly commanded
    # step is not evidence that this same step failed to resolve motion.
    f0 = force(ex,sense(ex))
    gravity = float(np.linalg.norm(ex.model.mj.opt.gravity))
    peak,periods,moving,waiting,false_stops,free = f0,0,0,0,0,0
    fraction,signed_turn = 0,0
    travel = dict(distance_mm=0,turn_rad=0,torso_mm=0)
    prev = dict(odo0)
    velocities = []
    measured,plans,measured_w,plans_w,forces = [],[],[],[],[]
    stopped,binding,nearby = 'done',None,None
    last_step,last_turn_step = distance*rate*dt,angle*rate*dt
    settle_started = None
    last_waypoint_period, still_period = 0, None

    def read_step():
        nonlocal prev,periods,peak,signed_turn,last_velocity,still_period
        s = period(ex)
        periods += 1
        odo = dict(be.odometry())
        d = float(np.hypot(odo['x_m']-prev['x_m'],odo['y_m']-prev['y_m']))
        turn = G.wrap(odo['yaw_rad']-prev['yaw_rad'])
        dh = abs(odo['torso_m']-prev['torso_m'])
        if d == turn == dh == 0:
            if still_period is None:
                still_period = periods
        else:
            still_period = None
        travel['distance_mm'] += d*1000
        travel['turn_rad'] += abs(turn)
        travel['torso_mm'] += dh*1000
        signed_turn += turn
        for key,value in (('quantum_m',dh if height else d),('quantum_rad',abs(turn))):
            if value > 0:
                know[key] = value if know[key] is None else min(know[key],value)
        vel = (dh if height else max(d,abs(turn)*corner))/dt
        acc = abs(vel-last_velocity)/dt
        last_velocity = vel
        velocities.append(vel)
        f = force(ex,s)
        peak = max(peak,f)
        forces.append(max(0,f-f0))
        prev = odo
        tick = getattr(ex,'_tick_stop',None)
        reason = tick if isinstance(tick,str) and tick else 'steps_spent' if getattr(be,'spent',False) or getattr(be,'lost',False) else None
        if reason is None and f > f0+D.contact_rise(B,know,vel,acc,gravity):
            reason = 'contact'
        return d,abs(turn),dh,reason,f

    def close(d,turn,dh):
        if height:
            err = abs(prev['torso_m']-target)
            return D.settled(err,dh,know.get('quantum_m') or 0,last_step)
        err = float(np.hypot(prev['x_m']-target[0],prev['y_m']-target[1]))
        terr = abs(G.wrap(prev['yaw_rad']-target[2]))
        return D.settled(err,d,know.get('quantum_m') or 0,last_step) and D.settled(
               terr,turn,know.get('quantum_rad') or 0,last_turn_step)

    last_velocity = 0
    try:
        while periods < max_periods:
            settling = fraction >= 1
            if settling:
                settle_started = periods if settle_started is None else settle_started
                if (periods-settle_started)*dt >= settle_limit:
                    stopped = 'settle_timeout'
                    break
                waypoint = target
                waiting += 1
            else:
                offer,point = (vmax,None) if height else speed(ex,
                    (local[0]*fraction,local[1]*fraction,local[2]*fraction),
                    (local[0]*min(1,fraction+rate*dt),local[1]*min(1,fraction+rate*dt),
                     local[2]*min(1,fraction+rate*dt)),info,odo0['torso_m'],vmax,clearance_m)
                if offer <= 0:
                    stopped = 'path_blocked'
                    break
                step_fraction = min(1-fraction,rate*dt*min(1,offer/vmax))
                fraction += step_fraction
                last_step,last_turn_step = distance*step_fraction,angle*step_fraction
                waypoint = initial+change*fraction
                moving += 1
                if point is None:
                    free += 1
                bound = D.speed_item(B,know,offer,vmax,point,intent)
                if binding is None or offer < binding['value']:
                    binding,nearby = bound,point
            if not settling:
                last_waypoint_period, still_period = periods, None
            if height:
                be.command_torso(float(waypoint))
            else:
                be.command_base(*map(float,waypoint))
            d,turn,dh,reason,f = read_step()
            plans.append(0 if settling else last_step)
            measured.append(dh if height else d)
            plans_w.append(0 if settling else last_turn_step)
            measured_w.append(turn)
            if not settling:
                actual = prev['torso_m'] if height else np.array([prev['x_m'],prev['y_m'],odo0['yaw_rad']+signed_turn])
                lag = abs(waypoint-actual) if height else float(np.linalg.norm((waypoint-actual)[:2]))
                know['lag_s'] = max(know.get('lag_s') or 0,lag/max(offer,np.finfo(float).eps))
            if reason is not None:
                stopped = reason
                if reason == 'contact':
                    contact_odo = dict(prev)
                    contact_speed = last_velocity
                    be.hold_base()
                    if height:
                        be.command_torso(float(prev['torso_m']))
                    for k in range(math.ceil(settle_limit/dt)):
                        d,turn,dh,next_reason,at_rest = read_step()
                        waiting += 1
                        if next_reason not in (None,'contact'):
                            stopped = next_reason
                            break
                        if d == turn == dh == 0:
                            break
                    if still_period is not None:
                        know['settled_s'] = max(know.get('settled_s') or 0,
                                                  (still_period-last_waypoint_period)*dt)
                    contact_run = abs(prev['torso_m']-contact_odo['torso_m']) if height else float(np.hypot(prev['x_m']-contact_odo['x_m'],prev['y_m']-contact_odo['y_m']))
                    if contact_speed > 0:
                        know['run_on_s'] = max(know.get('run_on_s') or 0,contact_run/contact_speed)
                    if stopped == 'contact' and at_rest <= f0+D.contact_rise(B,know,0,0,gravity):
                        know['f_free'] = max(know.get('f_free') or 0,peak-f0)
                        false_stops += 1
                        stopped = 'done'
                        # Resume the same target from the measured point, with the hidden level raised.
                        continue
                break
            if not settling and (D.stall_index(plans[-window:],measured[-window:],window,watch_quantum_m) is not None
               or D.stall_index(plans_w[-window:],measured_w[-window:],window,watch_quantum_rad) is not None):
                stopped = 'stalled'
                break
            if fraction >= 1 and close(d,turn,dh):
                break
        else:
            stopped = 'time_limit'
    finally:
        hold_odo = dict(be.odometry())
        hold_speed = last_velocity
        be.hold_base()
        if height and stopped != 'done':
            be.command_torso(float(hold_odo['torso_m']))
        # Measure the run-on and time to still, without issuing another waypoint.
        rest_periods = 0
        for k in range(math.ceil(settle_limit/dt)):
            if getattr(be,'spent',False) or getattr(be,'lost',False):
                break
            d,turn,dh,reason,f = read_step()
            waiting += 1
            rest_periods += 1
            if d == turn == dh == 0:
                break
        if still_period is not None:
            know['settled_s'] = max(know.get('settled_s') or 0,
                                      (still_period-last_waypoint_period)*dt)
        run_distance = abs(prev['torso_m']-hold_odo['torso_m']) if height else max(
            float(np.hypot(prev['x_m']-hold_odo['x_m'],prev['y_m']-hold_odo['y_m'])),
            abs(G.wrap(prev['yaw_rad']-hold_odo['yaw_rad']))*corner)
        if hold_speed > 0:
            know['run_on_s'] = max(know.get('run_on_s') or 0,run_distance/hold_speed)
        if stopped == 'done':
            know['f_free'] = max(know.get('f_free') or 0,max(forces,default=0))
            know['sigma_free'] = max(know.get('sigma_free') or 0,float(np.std(forces)) if forces else 0)
        if stopped == 'contact':
            B.space.drop('contact')
            before_ease = be.read()['t']
            ease(ex)
            ease_periods = round((be.read()['t']-before_ease)/dt)
            periods += ease_periods
            waiting += ease_periods
        odo1 = dict(be.odometry())
        dx,dy = odo1['x_m']-odo0['x_m'],odo1['y_m']-odo0['y_m']
        c,s = np.cos(odo0['yaw_rad']),np.sin(odo0['yaw_rad'])
        moved = dict(forward_mm=float(c*dx+s*dy)*1000,left_mm=float(-s*dx+c*dy)*1000,turn_deg=math.degrees(signed_turn))
        records = W.moved(ex,name,odo0,odo1,travel,{**moved,'torso_mm':(odo1['torso_m']-odo0['torso_m'])*1000},S.pose(odo1,True)) if periods else 0
    r = dict(stopped=stopped,moved=moved,pose=S.pose(odo1),odo=odo1,seconds=periods*dt,periods=periods,
             moving=moving,waiting=waiting,records_moved=records,peak_force_n=peak,false_stops=false_stops,
             free_share=free/moving if moving else 0,near_point=None if nearby is None else nearby['id'])
    return B.report(r,[binding or D.speed_item(B,know,vmax,vmax,intent=intent),D.time_item(B,planned)])
