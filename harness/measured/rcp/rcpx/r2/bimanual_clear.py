# SPDX-License-Identifier: AGPL-3.0-only
"""Plan and live capsule separation between arms, from both stopping distances.

The live snapshots use a common world timestamp. A lockstep stop latches the same
pair record for both arms before either can ease. A pair already near may stay or
move apart. Hand pairs may be exempt while meeting on purpose.
"""
import math
import numpy as np
from rcpx.context import refusal
from rcpx.r2 import bimanual_bounds as D
ID = "BM1-clear"
MM = 1000.0
OVERRIDES = ("_path_check",)


def seg_dist(p1, q1, p2, q2):
    """Least distance between segments p1q1 and p2q2 (closest points of two segments)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r))
    if a <= eps:
        s, t = 0.0, min(1.0, max(0.0, f / e))
    else:
        c = float(d1 @ r)
        if e <= eps:
            t, s = 0.0, min(1.0, max(0.0, -c / a))
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s = min(1.0, max(0.0, (b * f - c * e) / den)) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(1.0, max(0.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, min(1.0, max(0.0, (b - c) / a))
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


class Clearance:
    def __init__(self, multi, frames, models):
        self.multi, self.frames, self.models = multi, frames, dict(models)
        self.arms = tuple(frames.arms)
        self.exempt, self.executors, self.live, self.pending = {}, {}, {}, {}
        self.samples_checked = 0

    def caps_R(self, arm, q, jaws=None, moving_only=False):
        m = self.models[arm]
        if jaws is not None:
            m.set_jaws(jaws)
        T = self.frames.T_R_B(arm)
        R, t = T[:3, :3], T[:3, 3]
        caps = [(name, R @ p0 + t, R @ p1 + t, float(r)) for name, p0, p1, r in m.collision_capsules(q)]
        return caps[1:] if moving_only else caps

    def present(self, arm, moving_only=False):
        st = self.multi.read(arm)
        g = st.get('gripper')
        return self.caps_R(arm, np.asarray(st['q']), g.get('q') if isinstance(g, dict) else None, moving_only)

    def refresh(self):
        for a in self.arms:
            reading = self.multi.read(a)
            t = reading['t']
            old = self.live.get(a)
            if old is not None and old['t'] == t:
                continue
            caps = self.present(a)
            ends = {name: np.array([p0, p1]) for name, p0, p1, r in caps}
            speeds = {name: D.link_speed(None if old is None else old['ends'].get(name), p,
                                       float(self.multi.dt)) for name, p in ends.items()}
            self.live[a] = dict(t=t, ends=ends, speeds=speeds, caps=caps, q=reading['q'])
        for ex in self.executors.values():
            ex.state['bimanual_clear']['live'] = self.live

    def _skip(self, a, na, b, nb):
        ea, eb = self.exempt.get(a), self.exempt.get(b)
        return bool(ea and eb and na in ea and nb in eb)

    def nearest(self, arm, caps, others):
        best = (math.inf, None, None, None)
        for b, ocaps in others:
            for na, p0, p1, ra in caps:
                for nb, s0, s1, rb in ocaps:
                    if self._skip(arm, na, b, nb):
                        continue
                    d = seg_dist(p0, p1, s0, s1) - ra - rb
                    if d < best[0]:
                        best = (d, na, b, nb)
        return best

    def pairs(self, arm, q=None):
        caps = self.present(arm,True) if q is None else self.caps_R(arm,q,None,True)
        return {(na,b,nb):seg_dist(p0,p1,s0,s1)-ra-rb
                for na,p0,p1,ra in caps for b,ocaps in self.others(arm)
                for nb,s0,s1,rb in ocaps if not self._skip(arm,na,b,nb)}

    def others(self, arm):
        return [(b, self.present(b)) for b in self.arms if b != arm]

    def now(self, arm, q=None, jaws=None):
        caps = self.present(arm, True) if q is None else self.caps_R(arm, q, jaws, True)
        return self.nearest(arm, caps, self.others(arm))

    def between(self, a, qa, b, qb, exempt=None):
        saved, self.exempt = self.exempt, dict(exempt or {})
        try:
            return self.nearest(a, self.caps_R(a, qa, None, True), [(b, self.caps_R(b, qb))])[0]
        finally:
            self.exempt = saved

    def fit_values(self):
        return {a: m.e_fit() if callable(getattr(m,'e_fit',None)) else {} for a,m in self.models.items()}

    def margin(self, a, na, b, nb, va, vb, planned=False, fits=None):
        Ba, Bb = D.of(self.executors[a]), D.of(self.executors[b])
        fits = self.fit_values() if fits is None else fits
        ea, ta = D.fit_value(fits[a], na)
        eb, tb = D.fit_value(fits[b], nb)
        err, text = D.base_error(self.frames, a, b)
        bound = D.margin_item(Ba, Bb, va, vb, ea, eb, err, (ta, tb, text), planned)
        return bound['value'], bound

    def register(self, arm):
        ex = self.executors[arm]
        B = D.of(ex)
        T = self.frames.T_B_R(arm)
        fits = self.fit_values()
        try:
            for b in self.arms:
                if b == arm:
                    continue
                caps = self.present(b)
                shape = {'kind': 'capsules', 'capsules_m': [(T[:3,:3] @ p0 + T[:3,3],
                          T[:3,:3] @ p1 + T[:3,3], r) for name, p0, p1, r in caps]}
                ea = max((D.fit_value(fits[arm], c[0])[0] for c in self.present(arm)), default=0)
                eb = max((D.fit_value(fits[b], c[0])[0] for c in caps), default=0)
                err = D.base_error(self.frames, arm, b)[0]
                answer = B.space.add_thing('arm:' + b, shape, sigma=ea + eb + err, source='model')
                if isinstance(answer, dict) and answer.get('refused'):
                    raise ValueError('the space shape was refused')
        except (AttributeError, TypeError, ValueError, NotImplementedError) as exc:
            st = ex.state['bimanual_clear']
            if not st.get('space_problem'):
                st['space_problem'] = True
                D.journal(ex,'r2_problem', package='bimanual_clear', why=f'arm capsule space shape unavailable: {exc}')

    def available(self, arm):
        B = D.of(self.executors[arm])
        try:
            values = [B.dt(),B.t_settle(),B.lag(0),B.d_stop(0)]
            if not all(isinstance(v,(int,float,np.number)) and math.isfinite(v) for v in values):
                raise ValueError('a scalar bounds getter is unavailable')
        except Exception as exc:
            return {'said':'Refused: separation bounds are unavailable.',
                    **refusal('bounds_unavailable',f'separation needs scalar stopping and tracking bounds: {exc}','look()')}
        return None

    def path_problem(self, arm, path, control_periods=False):
        self.samples_checked = 0
        if len(path) <= 1 or len(self.arms) <= 1:
            return None
        bad = self.available(arm)
        if bad is not None:
            return bad
        self.refresh()
        ex, B = self.executors[arm], D.of(self.executors[arm])
        q0 = np.asarray(path[0])
        fits = self.fit_values()
        others = self.others(arm)
        # The smallest pair margin along the input path sets spacing. Directions
        # come from each link's own end motion, not from the tip's tool axis.
        spacing = []
        previous = self.caps_R(arm,q0,None,True)
        for q in path[1:]:
            caps = self.caps_R(arm,q,None,True)
            for na,p0,p1,r in caps:
                old = next(c for c in previous if c[0] == na)
                delta = np.mean([p0-old[1],p1-old[2]],axis=0)
                norm = np.linalg.norm(delta)
                u = self.frames.dir_from_R(arm,delta/norm) if norm else ex.model.fk_tip(q)[1][:,0]
                va = D.link_speed([old[1],old[2]],[p0,p1],B.dt()) if control_periods else B.v_free(q,u)
                for b,other in others:
                    for nb,s0,s1,rb in other:
                        margin = self.margin(arm,na,b,nb,va,self.live[b]['speeds'].get(nb,0),True,fits)[0]
                        if margin > 0:
                            spacing.append(B.spacing(margin))
            previous = caps
        if not spacing:
            return None
        ends = lambda q: np.array([[p0,p1] for name,p0,p1,r in self.caps_R(arm, q, None, True)])
        samples = D.path_samples(list(path), ends, min(spacing))
        self.samples_checked = 1
        others = self.others(arm)
        starts = self.pairs(arm,q0)
        previous = self.caps_R(arm, q0, None, True)
        segment,previous_share = 0,0
        for q in samples[1:]:
            self.samples_checked += 1
            caps = self.caps_R(arm,q,None,True)
            delta_q = np.asarray(path[segment+1])-np.asarray(path[segment])
            norm_q = float(delta_q @ delta_q)
            share = float((q-np.asarray(path[segment])) @ delta_q)/norm_q if norm_q else 1
            elapsed = B.dt()*max(0,share-previous_share)
            for na,p0,p1,ra in caps:
                old = next(c for c in previous if c[0]==na)
                delta = np.mean([p0-old[1],p1-old[2]],axis=0)
                norm = np.linalg.norm(delta)
                u = self.frames.dir_from_R(arm,delta/norm) if norm else ex.model.fk_tip(q)[1][:,0]
                va = D.link_speed([old[1],old[2]],[p0,p1],elapsed) if control_periods and elapsed>0 else B.v_free(q,u)
                slack = B.rigid(q,u)
                for b,ocaps in others:
                    for nb,s0,s1,rb in ocaps:
                        if self._skip(arm,na,b,nb):
                            continue
                        d = seg_dist(p0,p1,s0,s1)-ra-rb
                        d0 = starts[(na,b,nb)]
                        if d >= d0-slack:
                            continue
                        vb = self.live[b]['speeds'].get(nb,0)
                        margin,bound = self.margin(arm,na,b,nb,va,vb,True,fits)
                        if D.live_stop(d,d0,margin,slack):
                            out = {'said':f'Refused: {na} of arm {arm} would enter the separation from {nb} of arm {b}.',
                                   **refusal('arm_clearance',f'{na} of arm {arm} would be {max(0,d)*MM:g} mm from {nb} '
                                   f'of arm {b}; the derived plan margin is {margin*MM:g} mm: {bound["evidence"]}',
                                   f'home(arm="{b}")')}
                            out['refused'].update(links=[na,nb],arms=[arm,b],distance_mm=max(0,d)*MM,
                                                  clear_mm=margin*MM,samples=self.samples_checked,planned_samples=len(samples))
                            return B.report(out,[bound])
            previous,previous_share = caps,share
            if share>=1-1e-12 and segment+1<len(path)-1:
                segment,previous_share = segment+1,0
        return None


class Mixin:
    def _path_check(self, path, guarded):
        bad = super()._path_check(path, guarded)
        if bad is not None:
            return bad
        clear = getattr(self, '_bm_clear', None)
        return None if clear is None else clear.path_problem(self._bm_arm, path, control_periods=True)


def setup(ex):
    st = ex.state.setdefault('bimanual_clear', {'d0': None, 'stop': None, 'live': {}})

    def pre(name, kwargs):
        clear = getattr(ex, '_bm_clear', None)
        if clear is not None:
            clear.executors[ex._bm_arm] = ex
            bad = clear.available(ex._bm_arm)
            if bad is not None:
                return bad
            clear.refresh()
            st['d0'], st['stop'] = clear.now(ex._bm_arm)[0], None
            st['d0_pairs'] = clear.pairs(ex._bm_arm)
            clear.pending.pop(ex._bm_arm, None)
            clear.register(ex._bm_arm)
        return None

    def live(sense):
        clear = getattr(ex, '_bm_clear', None)
        if clear is None:
            return None
        a = ex._bm_arm
        bad = clear.available(a)
        if bad is not None:
            return 'bounds_unavailable'
        clear.refresh()
        clear.register(a)
        if a in clear.pending:
            st['stop'] = clear.pending[a]
            return 'arm_clearance'
        Ba = D.of(ex)
        slack = Ba.rigid(sense.q,sense.R[:,0])
        candidates = []
        fits = clear.fit_values()
        for (na,b,nb),d in clear.pairs(a,sense.q).items():
            d0 = st.get('d0_pairs',{}).get((na,b,nb),st['d0'] if st['d0'] is not None else math.inf)
            if d >= d0-slack:
                continue
            va,vb = clear.live[a]['speeds'].get(na,0),clear.live[b]['speeds'].get(nb,0)
            margin,bound = clear.margin(a,na,b,nb,va,vb,fits=fits)
            if D.live_stop(d,d0,margin,slack):
                candidates.append((d-margin,d,na,b,nb,va,vb,margin,bound))
        if candidates:
            _,d,na,b,nb,va,vb,margin,bound = min(candidates,key=lambda x:x[0])
            st['stop'] = dict(links=[na,nb],arms=[a,b],distance_mm=max(0,d)*MM,
                              stop_mm=margin*MM,speeds_m_s=[va,vb],bounds=bound)
            clear.pending[a] = st['stop']
            if vb > 0:
                clear.pending[b] = dict(st['stop'],links=[nb,na],arms=[b,a],speeds_m_s=[vb,va])
            return 'arm_clearance'
        return None

    def tick(sense):
        try:
            return live(sense)
        except Exception as exc:
            if not st.get('live_problem'):
                st['live_problem'] = True
                D.journal(ex,'r2_problem',package='bimanual_clear',why=f'live separation bounds failed: {exc}')
            return 'bounds_unavailable'

    def post(name, kwargs, result):
        if isinstance(result, dict) and result.get('stopped') == 'arm_clearance' and st.get('stop'):
            out = {'said': 'The arm stopped at the derived separation from another arm.', **result,
                   'arm_clearance': dict(st['stop'])}
            out.setdefault('periods', max(0,len(out.get('done',()))-1))
            return D.of(ex).report(out, [st['stop']['bounds']])
        return None

    st['idle_tick'] = tick
    ex.add_pre(pre)
    ex.add_tick(tick)
    ex.add_post(post)


def check_idle(ex, sense):
    fn = ex.state.get('bimanual_clear',{}).get('idle_tick')
    return fn(sense) if fn is not None else None
