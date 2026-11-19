# SPDX-License-Identifier: AGPL-3.0-only
"""Pace: the session's simulated-time budget as the binding limit, for any robot (a round two package).

Load it by name ("all,pace"). It changes no motion. What it adds:
  - every operator call's answer gains sim_s_spent (the world seconds that call took) and sim_s_left (what the
    session's time budget has left after it), when the session has a time budget;
  - a free resource robot://pace: the budget, what is left, what each tool cost so far in this session (calls,
    seconds, the dearest call), and, when the backend states them (backend.pace_costs: {tool: {"sim_s": [low,
    high], "what": text}}), the costs measured for this robot before the session;
  - a tool that takes max_s (a limit on its own world time) and is called without one gets
    max_s = SHARE of the seconds left, kept inside the tool's own bounds and never above its own default;
  - a short paragraph at the top of robot://practices: the budget is the binding limit when it is short, and
    any "run this once before" rule gives way to it.
Nothing here names a robot, a device or a task.
"""

import importlib
import math

ID = "P1"
SHARE = 0.25        # a tool's own default time limit never takes more than this share of the seconds left
# (tool, its max_s argument's default and bounds) for the tools whose default limit is a module constant
LIMITED = {"survey": ("rcpx.r2.survey", "MAX_S", "MIN_S", "MAX_S_TOP")}
HEADING = "## Time first"
TEXT = ("This session has a simulated-time budget (budget.sim_seconds; every answer gives sim_s_spent and "
        "sim_s_left). When it is short for this robot's tools it is the binding limit: robot://pace lists what each "
        "tool cost on this robot and in this session so far. Before a call, check that its cost fits what is left "
        "with room for the task itself. A rule below that says to run a check or a search once before moving gives "
        "way to the budget: skip it when its cost is more than a quarter of what is left, and use the free calls "
        "instead (a look with each named camera, crop, mark, triangulate, state, the robot:// resources).")


def _state(ex):
    return ex.state.setdefault("pace", {"calls": {}, "t0": None, "session": None, "limits": {}})


def _budget(ex):
    """(total, used, left) world seconds of the session's time budget, or None when there is none."""
    session = _state(ex)["session"]
    if session is None:
        return None
    try:
        total = float((getattr(session, "budget", None) or {})["sim_seconds"])
        used = float(session.sim_seconds_used())
    except (KeyError, TypeError, ValueError, AttributeError):
        return None
    if not math.isfinite(total) or total <= 0:
        return None
    return total, used, max(0.0, total - used)


def _clock(ex):
    """The session's own clock of charged world seconds (what its budget counts), else the executor's sim time."""
    session = _state(ex)["session"]
    if session is not None and callable(getattr(session, "sim_seconds_used", None)):
        return float(session.sim_seconds_used())
    return float(ex.sim_time())


def _limits():
    """{tool: (default, low, high)} of the LIMITED tools whose module loads."""
    out = {}
    for tool, (module, default, low, high) in LIMITED.items():
        try:
            mod = importlib.import_module(module)
            out[tool] = (float(getattr(mod, default)), float(getattr(mod, low)), float(getattr(mod, high)))
        except (ImportError, AttributeError, TypeError, ValueError):
            continue
    return out


def limit_for(left_s, default_s, low_s, high_s, share=SHARE):
    """The max_s a call left without one gets: share of the seconds left, within [low, min(default, high)] (pure)."""
    return round(min(default_s, high_s, max(low_s, share * float(left_s))), 2)


def _pre(ex):
    def pre(name, kwargs):
        st = _state(ex)
        if getattr(ex, "_depth", 0) > 0:
            return None    # ex.gates for a move the runtime makes inside a primitive: not an operator's call
        try:
            st["t0"] = _clock(ex)
        except Exception:
            st["t0"] = None
        lim = st["limits"].get(name)
        if lim is not None and kwargs.get("max_s") is None:
            b = _budget(ex)
            if b is not None:
                kwargs["max_s"] = limit_for(b[2], *lim)
                st["capped"] = {"tool": name, "max_s": kwargs["max_s"]}
        return None
    return pre


def _post(ex):
    def post(name, kwargs, result):
        st = _state(ex)
        t0, st["t0"] = st.get("t0"), None
        capped = st.pop("capped", None)
        if t0 is None or not isinstance(result, dict):
            return None
        try:
            spent = max(0.0, _clock(ex) - t0)
        except Exception:
            return None
        rec = st["calls"].setdefault(name, {"calls": 0, "sim_s": 0.0, "dearest_s": 0.0})
        rec["calls"] += 1
        rec["sim_s"] = round(rec["sim_s"] + spent, 3)
        rec["dearest_s"] = round(max(rec["dearest_s"], spent), 3)
        out = dict(result, sim_s_spent=round(spent, 2))
        b = _budget(ex)
        if b is not None:
            out["sim_s_left"] = round(b[2], 2)
        if capped is not None and capped["tool"] == name:
            out["pace_note"] = (f"max_s was left out, so it was set to {capped['max_s']:g} s "
                                f"({int(SHARE * 100)} percent of the seconds left); pass max_s to choose")
        return out
    return post


def setup(ex):
    st = _state(ex)
    st["limits"] = _limits()
    ex.add_pre(_pre(ex))
    ex.add_post(_post(ex))


def pace_document(ex, session):
    st = _state(ex)
    if st["session"] is None and session is not None:
        st["session"] = session
    b = _budget(ex)
    measured = getattr(ex.backend, "pace_costs", None)
    doc = {"sim_seconds_budget": None if b is None else round(b[0], 2),
           "sim_seconds_used": None if b is None else round(b[1], 2),
           "sim_seconds_left": None if b is None else round(b[2], 2),
           "share": SHARE,
           "this_session": {k: dict(v) for k, v in sorted(st["calls"].items())},
           "measured_on_this_robot": dict(measured) if isinstance(measured, dict) else None,
           "default_limits": {k: {"max_s_default": v[0], "range": [v[1], v[2]],
                                  "when_left_out": f"{SHARE:g} of the seconds left, in that range"}
                              for k, v in sorted(st["limits"].items())}}
    if doc["measured_on_this_robot"] is None:
        doc["note"] = "no costs were measured for this robot before the session: read this_session after each call"
    return doc


RESOURCES = {"robot://pace": (pace_document, "The time budget, what is left, and what each tool cost on this robot "
                                             "and in this session so far, without stepping.")}


def FILTER(surface, resources, handlers, ex=None, box=None):
    if ex is None:
        return surface, resources, handlers
    st = _state(ex)
    if box is not None:
        box.on_session(lambda session: st.__setitem__("session", session))
    names = {t.get("name") for t in surface.get("tools", [])}
    st["limits"] = {k: v for k, v in st["limits"].items() if k in names}
    for tool in surface.get("tools", []):
        if tool.get("name") in st["limits"]:
            tool["description"] = (tool.get("description", "") + " When max_s is left out in a session with a time "
                                   f"budget, max_s is {SHARE:g} of the seconds left (never above its default).")
    inner = resources.get("robot://practices")
    if inner is not None:
        def practices(inner=inner, **kw):
            content = inner(**kw)
            if _budget(ex) is None:
                return content
            head = HEADING + "\n" + TEXT + "\n\n"
            if isinstance(content, str):
                return head + content
            if isinstance(content, dict) and isinstance(content.get("text"), str):
                return {**content, "text": head + content["text"]}
            return content
        resources["robot://practices"] = practices
    return surface, resources, handlers


__all__ = ["FILTER", "ID", "LIMITED", "RESOURCES", "SHARE", "TEXT", "limit_for", "pace_document", "setup"]
