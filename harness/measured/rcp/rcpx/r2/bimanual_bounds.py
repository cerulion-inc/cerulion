# SPDX-License-Identifier: AGPL-3.0-only
"""Derived separation, hand geometry and the motion seam for several arms. SI."""

import functools
import inspect
import math

import numpy as np


def of(ex):
    from rcpx import bounds
    return bounds.of(ex)


def ratios():
    from rcpx import bounds_ratios
    return bounds_ratios


def item(what, value, unit, source, evidence, lift=None):
    return dict(what=what, value=float(value), unit=unit, over_mm=None,
                source=source, evidence=evidence, lift=lift)


def stop_margin(Ba, Bb, va, vb, e_fit_a, e_fit_b, base_err):
    return Ba.d_stop(abs(va)) + Bb.d_stop(abs(vb)) + e_fit_a + e_fit_b + base_err


def plan_margin(Ba, Bb, va, vb, e_fit_a, e_fit_b, base_err):
    return stop_margin(Ba, Bb, va, vb, e_fit_a, e_fit_b, base_err) + Ba.lag(abs(va)) + Bb.lag(abs(vb))


def fit_error(model, body):
    fn = getattr(model, 'e_fit', None)
    values = fn() if callable(fn) else {}
    return fit_value(values,body)


def fit_value(values,body):
    if body in values:
        return float(values[body]), f"the model's capsule fit error of {body}"
    return 0, "no fit error is known for this body's capsule"


def base_error(frames, a, b):
    errors = getattr(frames, 'errors_m', {})
    if a in errors or b in errors:
        return float(errors.get(a, 0) + errors.get(b, 0)), 'the stated errors of the two bases, added'
    return 0, 'no error is stated between the bases'


def link_speed(p_prev, p_now, dt):
    if p_prev is None:
        return 0
    return float(np.max(np.linalg.norm(np.asarray(p_now) - np.asarray(p_prev), axis=-1)) / dt)


def live_stop(d, d0, margin, slack):
    return d < margin and d < d0 - slack


def path_samples(path, ends_fn, spacing_m):
    """Split joint intervals; check nonlinear midpoint motion too. Never thin a path."""
    if len(path) == 0:
        return []
    if not math.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError('sample spacing must be positive and finite')
    out = [np.asarray(path[0], dtype=float)]

    def split(a, b, ea, eb):
        mid = np.mean([a, b], axis=0)
        em = np.asarray(ends_fn(mid), dtype=float)
        distances = [np.linalg.norm(y - x, axis=-1).max(initial=0)
                     for x, y in ((ea, eb), (ea, em), (em, eb))]
        n = max(1, math.ceil(max(distances) / spacing_m - 1e-12))
        if n == 1:
            out.append(b)
            return
        last, ends = a, ea
        for k in range(1, n + 1):
            q = a + (b - a) * (k / n)
            eq = np.asarray(ends_fn(q), dtype=float)
            split(last, q, ends, eq)
            last, ends = q, eq

    for q in path[1:]:
        a, b = out[-1], np.asarray(q, dtype=float)
        split(a, b, np.asarray(ends_fn(a)), np.asarray(ends_fn(b)))
    return out


def margin_item(Ba, Bb, va, vb, ea, eb, err, texts=(), planned=False):
    va,vb = abs(va),abs(vb)
    da,db = Ba.d_stop(va),Bb.d_stop(vb)
    value = da+db+ea+eb+err
    src = getattr(Ba, 'src', None)
    source = src('d_stop')[0] if callable(src) else 'self_test'
    evidence = (f'both stopping distances {da:g} m and {db:g} m; '
                f'both fit errors {ea:g} m and {eb:g} m; base error {err:g} m')
    if planned:
        la,lb = Ba.lag(va),Bb.lag(vb)
        value += la+lb
        evidence += f'; both tracking lags {la:g} m and {lb:g} m'
    if texts:
        evidence += '; ' + '; '.join(texts)
    return item('travel', value, 'm', source, evidence, 'move the other arm away')


def meet_gap(Bg, Bt, base_err, pad_m, qg=None, qt=None, u=None, ut=None):
    return Bg.rigid(qg, u) + Bt.rigid(qt, u if ut is None else ut) + base_err + pad_m


def min_gap_m(B, jaws):
    closed = float(jaws.width_m(jaws.closed))
    step = getattr(jaws, 'step_m', None)
    if step is None:
        dq = np.asarray(B.joint_quantum(), dtype=float)
        if dq.ndim == 0 or dq.shape == np.asarray(jaws.closed).shape:
            step = abs(float(jaws.width_m(np.asarray(jaws.closed) + dq)) - closed)
        else:
            # Arm joint quanta do not describe a different set of jaw joints.
            # An unknown jaw step is zero, so any resolved gap counts.
            step = 0
    return closed + ratios().QUANT * float(step)


def grip(B, held):
    value = B.grip(held)
    if isinstance(value, dict):
        return value['capacity_n'], value['accel_ok'], value['items']
    return value


def standoff(B, q, u, thing=None, base_err=0):
    return B.standoff(B.v_free(q, u), thing, q, u) + base_err


@functools.lru_cache(None)
def _new_core(cls):
    return 'kind' in inspect.signature(cls._cart_move).parameters


def sense(ex):
    return ex._sense()


def stopped(result):
    return result.get('stopped',result.get('stop'))


def _shape(r):
    if isinstance(r, dict) and 'periods' not in r:
        r['periods'] = max(0, len(r.get('done', ())) - 1)
    return r


def _old_speed(B, q, u, kind, target, intent, held, inside):
    v = B.v_free(q, u)
    if inside and kind in ('approach', 'touch', 'press', 'lower', 'close'):
        v = min(v, B.v_in(B.thing(target), q, u, intent, (held or {}).get('weight_n', 0))[0])
    if (intent or {}).get('speed') is not None:
        v = min(v, intent['speed'])
    return v


def line(ex, dp, drot=None, *, kind='transit', target=None, intent=None, away_from=None, held=None):
    s = sense(ex)
    if _new_core(type(ex)):
        return ex._cart_move(s, dp, drot, None, True, True, kind=kind, target=target,
                             intent=intent, away_from=away_from, held=held)
    B = of(ex)
    dp = np.asarray(dp, dtype=float)
    length = float(np.linalg.norm(dp))
    u = dp / length if length else s.R[:, 0]
    thing = B.thing(target) if target is not None else None
    p = getattr(thing, 'point', None)
    sigma = getattr(thing, 'sigma_p', 0) or 0
    clear = getattr(ex,'_bm_clear',None)
    if p is None and clear is not None and isinstance(target,str) and target.startswith('arm:'):
        other_arm = target.split(':',1)[-1]
        if other_arm in clear.executors:
            other = clear.executors[other_arm]
            qo = clear.multi.read(other_arm)['q']
            T = clear.frames.T_R_B(other_arm)
            po = T[:3,:3] @ other.model.fk_tip(qo)[0] + T[:3,3]
            back = clear.frames.T_B_R(ex._bm_arm)
            p = back[:3,:3] @ po + back[:3,3]
            ea = max((fit_error(ex.model,c[0])[0] for c in ex.model.collision_capsules(s.q)),default=0)
            eb = max((fit_error(other.model,c[0])[0] for c in other.model.collision_capsules(qo)),default=0)
            sigma = max(sigma,ea+eb+base_error(clear.frames,ex._bm_arm,other_arm)[0])
    band = ratios().BAND * sigma
    inside = p is not None and np.linalg.norm(s.tip - p) <= band
    if not inside and p is not None and length and kind in ('approach', 'touch', 'press', 'lower', 'close'):
        delta = s.tip - np.asarray(p)
        along = float(delta @ u)
        disc = along * along - float(delta @ delta) + band * band
        if disc >= 0:
            edge = -along - math.sqrt(disc)
            if 0 < edge < length:
                first = line(ex, u * edge, None if drot is None else np.asarray(drot) * edge / length,
                             kind='transit', target=target, intent=intent, away_from=away_from, held=held)
                if 'refused' in first or first.get('stopped') != 'goal':
                    return first
                s = sense(ex)
                dp = dp - u * edge
                drot = None if drot is None else np.asarray(drot) * (1 - edge / length)
                inside = True
            else:
                first = None
        else:
            first = None
    else:
        first = None
    v = _old_speed(B, s.q, u, kind, target, intent, held, inside)
    force = B.guard(kind, v, held=held, away_from=away_from).force_n
    r = _shape(ex._cart_move(s, dp, drot, v, True, True, force, limit=None))
    if first is not None:
        r['periods'] += first['periods']
    if 'bounds' not in r:
        B.report(r, [item('speed', v, 'm/s', 'model', 'the derived speed for this leg')])
    return r


def joints(ex, q, *, kind='transit', target=None, intent=None, away_from=None, held=None,
           dark_ok=False, record_contact=True):
    if _new_core(type(ex)):
        return ex._joint_move(q, None, kind=kind, target=target, intent=intent, away_from=away_from,
                              held=held, dark_ok=dark_ok, record_contact=record_contact)
    B, s = of(ex), sense(ex)
    p = ex.model.fk_tip(q)[0]
    delta = p - s.tip
    n = np.linalg.norm(delta)
    u = delta / n if n else s.R[:, 0]
    # The old joint helper's speed is a share of its own setting. Full share is the seam's fact.
    r = _shape(ex._joint_move(q, 1, dark_ok=dark_ok, record_contact=record_contact, limit=None))
    if 'bounds' not in r:
        B.report(r, [item('speed', B.v_free(s.q, u), 'm/s', 'model', 'derived joint motion, full joint share')])
    return r


def stream(ex, legs, *, held=None, looks=None, need_m=None):
    if _new_core(type(ex)) and hasattr(ex, '_stream'):
        return ex._stream(legs, held=held, looks=looks, need_m=need_m)
    results, periods = [], []
    for k, leg in enumerate(legs):
        to = np.asarray(leg['to'], dtype=float)
        kw = {key: leg.get(key) for key in ('kind', 'target', 'intent', 'away_from') if key in leg}
        if to.ndim == 1:
            r = joints(ex, to, held=held, **kw)
        else:
            from rcpx.control_plan import vec_from_rot
            s = sense(ex)
            r = line(ex, to[:3, 3] - s.tip, vec_from_rot(to[:3, :3] @ s.R.T), held=held, **kw)
        results.append(r)
        periods.append(r.get('periods', 0))
        if 'refused' in r or stopped(r) != 'goal':
            break
    last = results[-1] if results else {}
    return {**last, 'stopped': stopped(last) if results else 'goal', 'leg': len(results) - 1,
            'periods': periods, 'moving': sum(periods), 'waiting': 0, 'looks': [],
            'bounds': last.get('bounds'), 'results': results}


def command_hold(ex, q=None):
    if q is not None:
        ex._q_des = np.array(q,copy=True)
    return ex._command(ex._q_des)


def idle_hook(ex, fn, seconds):
    return ex._guarded(fn, 'idle', seconds)


def pad_thickness(jaws):
    """Projected pad thickness from the jaws' collision geometry, along tool x."""
    if getattr(jaws, 'pad_m', None) is not None:
        return float(jaws.pad_m)
    jaws.points_tool(0)
    m, d = jaws.m, jaws.d
    R = d.site_xmat[jaws.tip_site].reshape(3,3)
    extents = []
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) not in jaws.sides or not (m.geom_contype[g] or m.geom_conaffinity[g]):
            continue
        half = m.geom_aabb[g][3:]
        axis = R[:,0] @ d.geom_xmat[g].reshape(3,3)
        extents.append(float(np.abs(axis) @ half))
    return max(extents, default=0) * 2


def raise_heights(B, models, midpoint, spacing_m, reachable):
    """The heights tried for a meeting point: from the midpoint of the two bases upward at the separation's own
    sample spacing, as far as the shorter arm's links reach; only the heights the caller's test reaches are kept."""
    reach = min(sum(np.linalg.norm(p1-p0) + r for name,p0,p1,r in m.collision_capsules(m.q_home))
                for m in models)
    step = spacing_m if spacing_m > 0 else B.spacing(reach)
    if not step > 0:
        return []
    points = [midpoint + np.array([0,0,k*step]) for k in range(math.ceil(reach/step)+1)]
    return [p for p in points if reachable(p)]


def jaw(ex, width, intent=None, held=None):
    params = inspect.signature(ex.gripper).parameters
    kw = {'width_frac': width, 'hold': True}
    if 'clear' in params:
        kw['clear'] = (intent or {}).get('clear')
    started = ex.backend.read()['t']
    result = ex.gripper(**kw)
    if 'periods' not in result:
        result['periods'] = round((ex.backend.read()['t']-started)/of(ex).dt())
    return result


def end_word(B):
    fn = getattr(B.space,'end_call',None)
    if callable(fn):
        fn()


def journal(ex, kind, **fields):
    return ex._journal(kind, **fields)
