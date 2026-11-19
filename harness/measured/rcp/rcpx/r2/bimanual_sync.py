# SPDX-License-Identifier: AGPL-3.0-only
"""Time aligned moves of several arms (rcpx/r2/bimanual.py): move_arms and home_arms.

move_arms(moves={ARM: {"tool": NAME, "args": {...}}, ...}) runs one motion tool per arm in lockstep (the
Maintainer, rcpx/r2/bimanual_arms.py): every arm's call starts in the same control period, each period the world
advances once with every arm's command, an arm whose call ends holds its pose while the others finish. Before
anything moves, every arm's arguments are checked against that arm's own tool entry and every arm's gates run
(rcpx.r2.core ex.gates: the pre hooks an operator's call meets); a call a primitive refuses before its first
control period also stops the others before the world moves (their first command is withdrawn). So one arm's
refusal moves no arm. The answer: {ARM: that call's own result, ..., "periods": control periods the whole move
took, "started_period": the world period it started in}; a refusal adds "refused" naming the arm.

home_arms(open_grippers=True): every arm with a gripper opens it (time aligned), then every arm goes to its home
pose (the round one home, time aligned: its path is checked against the floor, itself and the other arms).

The tools move_arms takes: every arm tool that is not free and is not a look (a look in lockstep would wait for
the others' periods to pass for nothing). Nothing here names a robot.
"""

import inspect
from rcpx.context import refusal as _refusal
from rcpx.r2 import bimanual_bounds as D

def refusal(check, why, hint):
    hint = hint if '(' in hint else 'home_arms()'
    return {'said':'Refused: '+why+'.',**_refusal(check,why,hint)}


LOOKS = ("look", "standard_look", "relocalize", "snapshot")   # not run in lockstep (module docstring)
RESERVED = ("periods", "started_period", "refused", "said", "moved")


def lockstep_tools(bm, arm):
    """The tools move_arms may run on that arm."""
    from rcpx import surfaces

    return sorted(n for n, e in bm.entries[arm].items() if n not in LOOKS and not surfaces.is_free(n, e))


def _saved(bm, arm):
    ex = bm.exs[arm]
    st = bm.multi.read(arm)
    return {"q_des": None if ex._q_des is None else ex._q_des.copy(),
            "q_cmd": None if ex._q_cmd is None else ex._q_cmd.copy(), "frac": ex._frac_cmd,
            "grip": ex._gripper_cmd, "q_target": st.get("q_target"), "torque_frac": st.get("torque_frac", 1.0)}


def _restore(bm, arm, s):
    """Withdraw a first command the world never ran (an aborted lockstep job)."""
    ex = bm.exs[arm]
    ex._q_des, ex._q_cmd, ex._frac_cmd, ex._gripper_cmd = s["q_des"], s["q_cmd"], s["frac"], s["grip"]
    q = s["q_cmd"] if s["q_cmd"] is not None else s["q_target"]
    if q is not None:
        bm.multi.command(arm, q, torque_frac=s["frac"] if s["q_cmd"] is not None else s["torque_frac"],
                         gripper=s["grip"])


def run(bm, plans):
    """plans {arm: (tool, kwargs)} checked already. Returns the answer of move_arms (module docstring)."""
    saved = {a: _saved(bm, a) for a in plans}
    jobs = {a: (lambda a=a: bm.call_raw(a, plans[a][0], plans[a][1])) for a in plans}
    results, started, periods, aborted = bm.coord.run_lockstep(jobs)
    first = next((a for a in bm.arms if a in results and isinstance(results[a], dict) and "refused" in results[a]
                  and a not in aborted), None)
    failed = bm.coord.world_error
    for a in aborted:
        if failed:
            results[a] = refusal("world", f"the robot stopped answering during the move ({failed})",
                                 "read robot://world and look before the next move")
            bm.note("lockstep_world_error", arm=a, error=failed)
            continue
        _restore(bm, a, saved[a])
        why = results[first]["refused"] if first else {"check": "internal", "why": "a call failed"}
        results[a] = refusal("other_arm_refused", f"arm {first}'s {plans[first][0]} was refused ({why.get('check')}) "
                             "before anything moved, so this arm did not move either",
                             f"fix arm {first}'s call, then send move_arms again")
        bm.note("lockstep_abort", arm=a, by=first)
    out = {"said": "The arms completed their time aligned calls.", **{a: results.get(a) for a in bm.arms if a in plans}}
    out["bounds"] = {"binding": next(((r.get("bounds") or {}).get("binding") for r in results.values() if isinstance(r,dict) and (r.get("bounds") or {}).get("binding")),None),"more":0}
    out.update(periods=int(periods), started_period=int(started))
    if failed:
        out["refused"] = {"check": "world", "why": f"the robot stopped answering after {int(periods)} control "
                          f"periods of this move ({failed})", "hint": "home_arms()", "arm": next((a for a in bm.arms if a in aborted), None)}
        out["moved"] = periods > 0
    elif aborted or (first is not None and periods == 0):
        r = results[first]["refused"]
        out["refused"] = {"check": r.get("check"), "why": f"arm {first}: {r.get('why')}; no arm moved",
                          "hint": r.get("hint", ""), "arm": first}
        out["moved"] = False
    return out


def _prepare_all(bm, moves, clear=None):
    if not isinstance(moves, dict) or not moves:
        return None, refusal("moves", "moves must name at least one arm: {ARM: {tool, args}}",
                             f"for example {{\"{bm.arms[0]}\": {{\"tool\": \"move_tip\", \"args\": {{\"dz\": 50}}}}}}")
    unknown = [a for a in moves if a not in bm.arms]
    if unknown:
        return None, refusal("arm", f"no arm named {unknown[0]!r}: this robot's arms are {', '.join(bm.arms)}",
                             "name each arm as robot://body lists it")
    plans = {}
    for a in bm.arms:
        if a not in moves:
            continue
        m = moves[a]
        tool = m.get("tool") if isinstance(m, dict) else None
        if not isinstance(tool, str):
            return None, dict(refusal("moves", f"moves.{a} must be {{tool, args}}", "tool is a tool name, args its "
                                      "arguments without arm"), arm=a)
        allowed = lockstep_tools(bm, a)
        if tool not in allowed:
            return None, dict(refusal("tool", f"move_arms runs motion tools only; {tool!r} is not one of arm {a}'s",
                                      "motion tools: " + ", ".join(allowed)), arm=a)
        D.of(bm.exs[a]).begin(tool)
        args = m.get("args") or {}
        if isinstance(args, dict) and "arm" in args and args["arm"] != a:
            return None, dict(refusal("arm", f"moves.{a}.args names arm {args['arm']!r}",
                                      "leave arm out of args: the key of moves names the arm"), arm=a)
        args = {k: v for k, v in args.items() if k != "arm"} if isinstance(args, dict) else args
        if isinstance(args,dict) and clear is not None:
            args = dict(args)
            if "clear" in bm.entries[a][tool].get("args",{}).get("properties",{}):
                args.setdefault("clear",clear)
            word = D.of(bm.exs[a]).word({"clear":clear})
            if word.get("refused"):
                return None, dict(word["refused"],arm=a)
        kwargs, bad = bm.prepare(a, tool, args)
        if bad is not None:
            return None, dict(bad, arm=a)
        plans[a] = (tool, kwargs)
    for a, (tool, kwargs) in plans.items():
        method = bm.entries[a][tool].get("method", tool)
        gate = bm.exs[a].gates(method, kwargs)
        if gate is not None:
            return None, dict(gate, arm=a)
    return plans, None


def _refused(bad):
    arm = bad.pop("arm", None)
    body = dict(bad["refused"])
    if '(' not in body.get('hint',''):
        body['hint'] = 'home_arms()' 
    if arm is not None:
        body["arm"] = arm
        body["why"] = f"arm {arm}: {body.get('why', '')}; no arm moved"
    return {"said": "Refused: " + body.get("why", "the move was refused") + ".", "refused": body, "moved": False}


def _move_arms(bm, session, moves=None, clear=None):
    for a in bm.arms:
        D.of(bm.exs[a]).begin("move_arms")
    plans, bad = _prepare_all(bm, moves, clear)
    if bad is not None:
        return _refused(bad)
    return run(bm, plans)


def _home_arms(bm, session, open_grippers=True, clear=None):
    out = {"periods": 0}
    grips = {}
    if open_grippers:
        plans = {a: ("gripper", {"width_frac": 1.0}) for a in bm.arms
                 if bm.exs[a].model.spec.get("gripper") and "gripper" in bm.entries[a]}
        if plans:
            g, bad = _prepare_all(bm, {a: {"tool": t, "args": k} for a, (t, k) in plans.items()},clear)
            if bad is not None:
                return {**_refused(bad), "said": "No arm went home: opening the grippers was refused."}
            grips = run(bm, g)
            out["started_period"] = grips["started_period"]
            out["periods"] += grips["periods"]
            if "refused" in grips:
                return {**grips, "said": "No arm went home: opening the grippers was refused."}
    moves = {a: {"tool": "home", "args": {}} for a in bm.arms if "home" in bm.entries[a]}
    plans, bad = _prepare_all(bm, moves,clear)
    if bad is not None:
        arm = bad.pop("arm", None)
        r = dict(bad["refused"], arm=arm)
        return {"refused": r, "said": f"No arm went home: arm {arm}'s home was refused ({r.get('check')}).",
                "grippers": {a: grips.get(a) for a in grips if a in bm.arms}, "moved": bool(grips)}
    homes = run(bm, plans)
    out.setdefault("started_period", homes["started_period"])
    out["periods"] += homes["periods"]
    out["home"] = {a: homes.get(a) for a in bm.arms if a in plans}
    if grips:
        out["grippers"] = {a: grips.get(a) for a in bm.arms if a in grips}
    out["bounds"] = homes.get("bounds")
    if "refused" in homes:
        out["refused"] = homes["refused"]
        out["said"] = f"No arm went home: {homes['refused']['why']}."
        return out
    parts = []
    for a in bm.arms:
        r = out["home"].get(a) or {}
        if r.get("stopped") == "goal":
            parts.append(f"{a} is home (within {r.get('error_deg', 0):g} deg)")
        else:
            parts.append(f"{a} stopped ({r.get('stopped')}{': ' + str(r['why']) if r.get('why') else ''})")
    out["said"] = ("Grippers opened, then " if grips else "") + "; ".join(parts) + "."
    return out


def move_arms(bm,session,moves=None,clear=None):
    try:
        return _move_arms(bm,session,moves,clear)
    finally:
        if clear is not None:
            for ex in bm.exs.values():
                D.end_word(D.of(ex))


def home_arms(bm,session,open_grippers=True,clear=None):
    try:
        return _home_arms(bm,session,open_grippers,clear)
    finally:
        if clear is not None:
            for ex in bm.exs.values():
                D.end_word(D.of(ex))
