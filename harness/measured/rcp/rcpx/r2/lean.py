# SPDX-License-Identifier: AGPL-3.0-only
"""The lean profile (opt-in: packages "all,lean", "lean" or a list that names it): the same capability in a
fraction of the context. The tool list, every answer and every document are context too, and they have a budget.

Nothing is hidden for good and nothing is cut silently: every tool still runs, every cut says where the whole is.
Lean is a RENDERING of what the session answers. Inside the session nothing changes: the journal, the scoring,
the harm windows and every package that reads the journal see the same calls and the same whole results as
without lean. Only what leaves Session.dispatch is narrowed.

1. DECLARED TOOLS (FILTER). A session declares (lists with a full argument schema) only the CORE tools, each
   described by its one line and its worked example, plus four tools of this package:
     tools()             the index of EVERY tool: name, one line, argument names, and how it is called
     tool_help(name)     one tool's full description, argument schema and worked example
     use(name, args)     runs any tool by name
     last_result(key)    the whole answer of the last tool call, nothing cut
   Membership is the FLAGS table below (one line of reason per tool), not a guess at run time; a tool that is not
   in the table is reached through use. A tool that is not declared is still CALLABLE BY ITS OWN NAME: only the
   declaration is narrowed, so an operator that calls a tool it read about in tools() is not refused.
2. use IS A DIRECT CALL. When the session exists (box.on_session), its dispatch is wrapped: a request
   {"name": "use", "args": {"name": X, "args": A}} is sent on as the call of X with A (marked "via": "use").
   So the argument checks, the shape tolerance, the refusals, the budget (one call), the journal record, the harm
   window and a replay are a direct call's, because it is one. Arguments written beside the name instead of under
   args are read as the tool's arguments, and the answer says so (arguments_note).
3. ANSWERS. The same wrapper returns the lean form of every tool answer (rcpx/r2/lean_cut.py: the rule, the keep
   list, one frame per answer) after the session has journaled the whole one, so the journal always holds the
   whole answer; a "lean" record beside it names what was cut and which frame was sent. The last whole answer
   that was not a refusal is kept for last_result. A refusal passes whole, byte for byte.
   WHO GETS WHOLE ANSWERS: any caller that does not go through Session.dispatch. A package, the scripted person
   or a test that holds the executor reads whole results, so no package logic ever sees a cut value. A caller that
   uses the session (an operator on the socket, an in-process operator such as an oracle) gets lean answers.
4. DOCUMENTS. A read of a document resource robot://NAME, except the practices and the task, returns its lean
   form (the same rule, no frame attached); robot://NAME/full is the whole document (one template resource,
   robot://{doc}/full, listed once). A template resource (one entry of a document, asked for by id) is whole.
   No read attaches a frame: a whole document names its frames as text where they stand.
5. THE SESSION LINE "lean profile: N core tools declared, each by one line (tool_help(name) gives a tool's full
   text); M more run through use(name, args); tools() lists every tool; last_result() gives any cut detail" ends
   the surface's conventions (every list answer carries it) and the practices (they ride with the first answer
   that is not a compact list). It is what says that a declared description is the short form.
6. list NAME (the session's own way to one tool's full text, which the compact list and the practices name) is
   answered in full for ANY tool, declared or not: the full description, schema and example, and for a tool
   that is not declared how it runs. Without this the session would answer a declared tool by its one line with
   nothing said, and refuse a tool that runs through use as unknown.

How the free tools stay free: tools, tool_help and last_result raise the session's call total by one as they
run, so calls_left does not drop (calls_used does count them: they are requests, and every journal and truth log
index stays unique). A spent budget refuses them like any call; robot://journal/full still holds every answer.
"""
import difflib
import json
import traceback

from rcpx import server, surfaces
from rcpx.context import refusal
from rcpx.r2 import lean_cut
from rcpx.surface_list import brief

ID = "lean"
SESSION_LINE = ("lean profile: {n} core tools declared, each by one line (tool_help(name) gives a tool's full text); "
                "{m} more run through use(name, args); tools() lists every tool; last_result() gives any cut detail")
OWN = ("tools", "tool_help", "use", "last_result")       # this package's tools: always declared, never cut
FREE = ("tools", "tool_help", "last_result")
WHOLE_URIS = ("robot://practices", "robot://task")       # texts and the task are served as they are
FULL = "/full"
FULL_URI = "robot://{doc}/full"                           # one template serves every whole document
LINE_MAX = 100                                            # characters of a tool's one line in tools()

# name: (declared, why). Declared = what an operator needs to look, move, touch, act, verify, recover and finish.
FLAGS = {
    "look": (True, "the only way to know a state: every act starts and ends with one"),
    "standard_look": (True, "the fixed view later looks are compared with; says when the base or the world moved"),
    "state": (True, "where the joints and the tip are now, without a picture"),
    "move_tip": (True, "the basic move: the tip by a small offset"),
    "orient_tool": (True, "turning the hand about the tip, needed before most pushes"),
    "approach": (True, "one checked call that brings the tip in front of a part"),
    "reach_check": (True, "asked before a stroke: is the point in reach and how hard can the tip push there"),
    "touch": (True, "feeling for a surface: the only depth an operator has"),
    "press": (True, "the act itself: a push on a part"),
    "verify": (True, "confirming a state from a look: what a claim rests on"),
    "device_state": (True, "what is known of a device and what its cue shows"),
    "mark": (True, "naming a pixel so that two looks give a point"),
    "triangulate": (True, "the point in space of a label marked in two looks"),
    "retreat": (True, "backing out the way the arm came: the first step of every recovery"),
    "home": (True, "a known pose to start again from"),
    "park": (True, "the rest pose an episode ends in"),
    "wait_for_light": (True, "the one thing to do when a look says dark"),
    "ask_person": (True, "asking when the robot cannot find out by itself"),
    "move_joints": (False, "joint space moves: move_tip, approach, home and park cover the work near a device"),
    "relocalize": (False, "standard_look already says whether the base or the world moved"),
    "picture": (False, "an outside drawing for planning a stroke, not needed for each act"),
    "gripper": (False, "opening and closing the hand: only a task that holds something needs it"),
    "ease": (False, "touch, press and every guarded stop already end eased"),
    "measure_base_give": (False, "a measurement made once, before hard pushes on a loose base"),
    "self_check": (False, "a body check made once per session"),
    "zero_check": (False, "a follow-up of self_check for one joint"),
    "apply_offset": (False, "a follow-up of self_check: applying a measured offset"),
    "watch": (False, "following a cue over time: device_state covers the common read"),
    "grasp": (False, "holding an object: only a task that holds something needs it"),
    "release": (False, "letting go of a held object"),
    "impulse": (False, "a short strike: only a task that asks for one"),
    "pinch": (False, "a measured squeeze: only a task that asks for one"),
    "probe_motion": (False, "finding how an unknown part moves, when no device knowledge is given"),
    "remember": (False, "keeping a fact for a later episode"),
    "hold": (False, "waiting in place: rare, and wait_for_light covers the dark case"),
    "release_check": (False, "asked before a torque cut only"),
    "crop": (False, "a closer cut of a past look, for reading a small state"),
}


def is_core(name):
    """A tool's flag; a tool this table does not know is not declared (it is reached through use)."""
    return FLAGS.get(name, (False, ""))[0]


TOOLS = [
    surfaces._tool("tools", "The index of every tool this robot has: name, one line, argument names (* marks a "
                   "required one). declared tools are called directly; a through_use tool runs as use(name, args). "
                   "Free: it does not use up a call.",
                   surfaces._obj({}), {}, "list every tool, the ones not declared here too",
                   brief="Index of every tool: name, one line, argument names; says which run through use. Free."),
    surfaces._tool("tool_help", "One tool's full description, argument schema and worked example, for any tool "
                   "tools() lists. Free: it does not use up a call.",
                   surfaces._obj({"name": surfaces._str("the tool's name, as tools() lists it")}, ["name"]),
                   {"name": "press"}, "read everything about press before the first push",
                   brief="One tool's full description, argument schema and worked example. Free."),
    surfaces._tool("use", "Run a tool that is not declared here, by its name, with that tool's arguments. The "
                   "arguments are checked and the call is refused, counted and answered exactly as if the tool "
                   "had been called directly; one call is counted. tools() lists the names and tool_help(name) "
                   "gives the arguments.",
                   surfaces._obj({"name": surfaces._str("the tool's name, as tools() lists it"),
                                  "args": {"anyOf": [{"type": "object"}, {"type": "string"}],
                                           "description": "that tool's arguments as one JSON object (default {})"}},
                                 ["name"]),
                   {"name": "gripper", "args": {"width_frac": "open"}},
                   "open the gripper fully (gripper is not declared here, so use runs it)",
                   brief="Run any tool by name with its arguments: the same checks, refusals and budget as a "
                         "direct call."),
    surfaces._tool("last_result", "The whole answer of your last tool call, nothing cut. An answer shows a block "
                   "it left out as {\"omitted\": N, \"see\": \"last_result\"}: key names that block (for example "
                   "axes_px, or look.axes_px for a nested one) and returns it alone; a key listed under "
                   "more_frames attaches that frame. With no key the whole answer comes back (frames are named, "
                   "not attached). Free: it does not use up a call.",
                   surfaces._obj({"key": surfaces._str("the part to return: a key of the last answer, a dotted "
                                                       "path into it, or nothing for the whole answer")}),
                   {"key": "axes_px"}, "the axes block the last look left out",
                   brief="The whole answer of the last tool call, or the part named by key (a cut block, a frame). "
                         "Free."),
]


def setup(ex):
    ex.state["lean"] = {"last": None, "all": [], "declared": [], "more": [], "line": ""}


def _state(ex):
    return ex.state.setdefault("lean", {"last": None, "all": [], "declared": [], "more": [], "line": ""})


def _journal(ex, kind, **fields):
    try:
        ex._journal(kind, **fields)
    except Exception:   # a journal that cannot take the record must not cost the operator its answer
        pass


def is_document(uri):
    """A resource lean renders: robot://NAME, one segment, not the practices or the task."""
    name = uri[len("robot://"):] if isinstance(uri, str) and uri.startswith("robot://") else ""
    return bool(name) and "/" not in name and "{" not in name and uri not in WHOLE_URIS


# ---------------------------------------------------------------------------------------------------------------
# What leaves Session.dispatch


def direct(request):
    """(the request to send on, a note or None). A use request becomes the call it stands for; anything else,
    and a use request this cannot read, goes on as it is (the session then answers it, refusals included)."""
    try:
        if not (isinstance(request, dict) and request.get("op") == "call" and request.get("name") == "use"):
            return request, None
        args = request.get("args")
        if args is None and isinstance(request.get("args_text"), str):
            args = json.loads(request["args_text"])
        if not isinstance(args, dict) or not isinstance(args.get("name"), str) or args["name"] in OWN:
            return request, None
        beside = {k: v for k, v in args.items() if k not in ("name", "args")}
        inner, note = args.get("args"), None
        if inner is None:        # no args: what stands beside the name is what the tool was meant to get
            inner = beside
            if beside:
                note = f"use: read {', '.join(sorted(beside))} as the arguments of {args['name']}"
        elif beside:
            return request, None  # both: the session refuses the stray names as it does for any tool
        out = {k: v for k, v in request.items() if k not in ("name", "args", "args_text")}
        out.update(name=args["name"], via="use")
        out["args_text" if isinstance(inner, str) else "args"] = inner
        return out, note
    except Exception:
        return request, None


def render(state, response, base_dir=None, note=None):
    """(the response as lean sends it, what was done or None). A tool answer and a document read are cut by
    the rule and keep at most one frame; everything else, a refusal and this package's own answers pass as they
    are. The whole answer of a tool that did not refuse is kept in state["last"]."""
    result = response.get("result") if isinstance(response, dict) else None
    if not isinstance(result, dict) or "refused" in result:
        return response, None
    op, name = response.get("op"), response.get("name")
    if op == "call" and name not in OWN:
        state["last"] = {"name": name, "result": result}
        small, done = lean_cut.lean(result, "last_result", base_dir, keep=1)
        if note:
            small = dict(small, arguments_note=list(small.get("arguments_note") or []) + [note])
    elif op == "read" and isinstance(result.get("content"), (dict, list)) \
            and not (isinstance(result["content"], dict) and "refused" in result["content"]):
        content, see = result["content"], str(name) + FULL
        if not is_document(name):    # a whole document or a template entry: nothing cut, its frames named as text
            body = lean_cut.name_frames(content, base_dir)
            done = {"cut": [], "frames_left_out": len(lean_cut.frames_in(content)), "frame": None}
        elif isinstance(content, dict):
            body, done = lean_cut.lean(content, see, base_dir, keep=0)
        else:
            body, cuts = lean_cut.cut(content, see, protect=False)
            body, names, _ = lean_cut.one_frame(content, body, base_dir, keep=0, limit=None)
            done = {"cut": cuts, "frames_left_out": len(names), "frame": None}
        small = dict(result, content=body)
    else:
        return response, None
    if small == result:
        return response, None
    out = dict(response, result=small)
    frames = surfaces.frame_paths(small, base_dir)
    if frames:
        out["frames"] = frames
    else:
        out.pop("frames", None)
    return out, dict(done, name=name, op=op, req=response.get("seq"))


def _attach(ex, session):
    """Once the Session exists: every tool that is not declared is callable by its own name, a list that names
    tools is answered in full (_listed), a use request is sent on as the call it stands for, and every answer
    leaves in its lean form."""
    tools = getattr(session, "tools", None)
    inner = getattr(session, "dispatch", None)
    if not isinstance(tools, dict) or not callable(inner) or getattr(inner, "lean", False):
        return
    st = _state(ex)
    by_name = {t["name"]: t for t in st["all"]}
    for name in st["more"]:
        tools.setdefault(name, by_name[name])
    base = getattr(session, "ep_dir", None) or getattr(getattr(session, "journal", None), "dir", None)
    if callable(getattr(session, "_list_answer", None)) and callable(getattr(session, "_list", None)):
        session._list_answer = _listed(st, session, session._list_answer)

    def dispatch(request):
        sent, note = direct(request)
        response = inner(sent)
        try:
            out, done = render(st, response, base, note)
        except Exception:    # a rendering bug must not cost the operator its answer: the whole one goes out
            _journal(ex, "r2_problem", package="lean", where="render", trace=traceback.format_exc())
            return response
        if done is not None:
            _journal(ex, "lean", **done)
        return out

    dispatch.lean = True
    session.dispatch = dispatch


def _listed(st, session, inner):
    """The session's list answer under lean. A list that names tools (list NAME) returns each one's FULL entry as
    the session lists a tool, for any tool this robot has; one that is not declared adds how it runs. The
    session's own answer holds the declared tools only, by their one line. Every other list goes to `inner`."""
    def list_answer(request):
        names = request.get("name") if isinstance(request, dict) else None
        if names is None or names == [] or names == "":
            return inner(request)
        names = names if isinstance(names, list) else [names]
        by_name = {t["name"]: t for t in st["all"]}
        for name in names:
            if not (isinstance(name, str) and (name in by_name or name == "claim")):
                return _unknown(name, list(by_name) + ["claim"])
        tools = [_entry(st, by_name[n]) for n in dict.fromkeys(n for n in names if n in by_name)]
        return surfaces.named(dict(session._list(), tools=tools), names)
    return list_answer


def _through_use(name, args):
    return {"through": "use", "args": {"name": name, "args": args}, "note": "it also runs when called by its name"}


def _entry(st, entry):
    """One tool's full entry in the shape of a session's listing (rcpx.server.Session._list)."""
    args = entry["example"]["args"]
    out = {"name": entry["name"], "description": entry["description"], "args": surfaces.public_schema(entry["args"]),
           "example": {"args": args, "means": entry["example"]["means"],
                       "cli": f"call {entry['name']} '{json.dumps(args)}'"}}
    if entry["name"] not in st["declared"]:
        out["call"] = _through_use(entry["name"], args)
    return out


# ---------------------------------------------------------------------------------------------------------------
# The four tools


def _free(session):
    """This call is not charged: the call total grows by the one call the session just counted."""
    budget = getattr(session, "budget", None)
    if isinstance(budget, dict) and "calls" in budget:
        budget["calls"] = int(budget["calls"]) + 1


def _arg_names(entry):
    props = (entry.get("args") or {}).get("properties") or {}
    required = set((entry.get("args") or {}).get("required") or [])
    return ", ".join(k + ("*" if k in required else "") for k in props) or "nothing"


def _row(entry):
    return {"name": entry["name"], "does": brief(entry, LINE_MAX), "args": _arg_names(entry)}


def _tools(ex, session, **_):
    _free(session)
    st = _state(ex)
    by_name = {t["name"]: t for t in st["all"]}
    return {"declared": [_row(by_name[n]) for n in st["declared"]],
            "through_use": [_row(by_name[n]) for n in st["more"]],
            "how": "a declared tool is called directly; a through_use tool runs as use(name, args), for example "
                   "use(name=\"gripper\", args={\"width_frac\": \"open\"}); tool_help(name) gives one tool's full "
                   "description, argument schema and example; in args * marks a required argument"}


def _unknown(name, names):
    near = difflib.get_close_matches(str(name), names, n=5, cutoff=0.5)
    shown = json.dumps(name) if isinstance(name, (str, int, float)) else repr(name)
    return refusal("unknown_tool", f"no tool named {shown[:60]}",
                   ("did you mean: " + ", ".join(near) + "? " if near else "") + "tools() lists every tool")


def _tool_help(ex, session, name=None, **_):
    _free(session)
    st = _state(ex)
    entry = next((t for t in st["all"] if t["name"] == name), None)
    if entry is None:
        return _unknown(name, [t["name"] for t in st["all"]])
    args = entry["example"]["args"]
    declared = name in st["declared"]
    return {"name": name, "description": entry["description"], "args": surfaces.public_schema(entry["args"]),
            "example": {"args": args, "means": entry["example"]["means"]},
            "call": "directly, by its name" if declared else _through_use(name, args)}


def _use(ex, session, name=None, args=None, **_):
    """Reached only for a name that is one of this package's own tools, or when no session was ever attached
    (the wrapped dispatch sends every other use request on as a direct call before it gets here)."""
    if name in OWN:
        return refusal("use", f"{name} is one of the tools that manage the tool list: call it directly",
                       "use(name, args) takes the name of a robot tool; tools() lists them")
    if session is None or not callable(getattr(session, "_call", None)):
        return refusal("unavailable", "use needs a session to run a tool", "call the tool by its own name")
    request = {"args_text": args} if isinstance(args, str) else {"args": {} if args is None else args}
    session.calls_used -= 1           # the call below counts itself: one call for the pair
    return session._call(session.seq, name, request)


def _last_result(ex, session, key=None, **_):
    _free(session)
    last = _state(ex).get("last")
    if last is None:
        return refusal("no_result", "no tool has answered yet in this session",
                       "call a tool first; last_result then returns its whole answer")
    full, base = last["result"], getattr(session, "ep_dir", None)
    if key is None or key == "":
        body, names, _chosen = lean_cut.one_frame(full, full, base, keep=0, limit=None)
        out = {"of": last["name"], "result": body}
    else:
        found, part = lean_cut.dig(full, key)
        if not found:
            keys = ", ".join(str(k) for k in full) if isinstance(full, dict) else "none"
            return refusal("key", f"the last answer (of {last['name']}) has no part {json.dumps(key)}",
                           f"its keys: {keys}; write a nested part as a.b and a list item as a.0; no key returns "
                           "the whole answer")
        if lean_cut.is_image_path(part):
            return {"of": last["name"], "key": key, "value": part}
        body, names, _chosen = lean_cut.one_frame(part, part, base, keep=1, limit=None)
        names = [f"{key}.{n}" for n in names]      # named from the answer's top: what last_result(key) takes
        out = {"of": last["name"], "key": key, "value": body}
    if names:
        out["more_frames"] = names
    return out


HANDLERS = {"tools": _tools, "tool_help": _tool_help, "use": _use, "last_result": _last_result}


# ---------------------------------------------------------------------------------------------------------------
# The surface


def _one_line(entry):
    """A declared tool as lean lists it: the one line, the full argument schema, the worked example."""
    return dict(entry, description=brief(entry))


def _edge():
    """The session's edge module when this tree has it (it leaves a gripper tool out of a fixed finger's
    listing), else None: lean's index and its session line count the tools the listing would show."""
    try:
        from rcpx import server_edge
        return server_edge
    except Exception:
        return None


def _with_line(inner, line):
    def practices(**kwargs):
        content = inner(**kwargs)
        if isinstance(content, str):
            return content.rstrip() + "\n" + line + "\n"
        if isinstance(content, dict) and isinstance(content.get("text"), str):
            return {**content, "text": content["text"].rstrip() + "\n" + line + "\n"}
        return content
    return practices


def _whole(fulls):
    """robot://{doc}/full: the document robot://doc as its owner serves it."""
    def read(doc=None, **_):
        fn = fulls.get(f"robot://{doc}")
        if fn is None:
            return refusal("unknown_resource", f"no document robot://{doc} has a whole form here",
                           "whole forms: " + ", ".join(uri + FULL for uri in fulls))
        return fn()
    return read


def FILTER(surface, resources, handlers, ex=None, box=None):
    """Narrow what the session declares (see the module text). Called after every package is merged."""
    joints = server._joint_names(ex) if ex is not None else None
    every = surfaces.for_robot(surface, joints)["tools"]            # examples name a real joint, as a session's do
    unlisted = getattr(_edge(), "_without_absent_gripper", None)    # what the session leaves out of its listing
    if callable(unlisted) and ex is not None:
        try:
            every = unlisted({"tools": every}, ex)["tools"]
        except Exception:                    # the session's own rule could not be read: count every tool
            _journal(ex, "r2_problem", package="lean", where="unlisted tools", trace=traceback.format_exc())
    declared = [t for t in every if is_core(t["name"]) or t["name"] in OWN]
    more = [t for t in every if not (is_core(t["name"]) or t["name"] in OWN)]
    line = SESSION_LINE.format(n=sum(1 for t in declared if t["name"] not in OWN), m=len(more))
    out = dict(surface, tools=[_one_line(t) for t in declared],
               about=(str(surface.get("about", "")).rstrip() + " " + line).strip())
    out["resources"] = dict(surface.get("resources", {}))

    fulls = {uri: fn for uri, fn in resources.items() if is_document(uri)}
    new_resources = {}
    for uri, fn in resources.items():
        if "{" in uri and fulls and FULL_URI not in new_resources:
            new_resources[FULL_URI] = _whole(fulls)     # ahead of every other template: its URIs resolve first
        new_resources[uri] = _with_line(fn, line) if uri == "robot://practices" else fn
    if fulls:
        new_resources.setdefault(FULL_URI, _whole(fulls))
        out["resources"][FULL_URI] = ("A document above whole, nothing cut: " +
                                      ", ".join(uri + FULL for uri in list(fulls)[:2]) + " and so on.")

    if ex is not None:
        _state(ex).update(all=every, declared=[t["name"] for t in declared], more=[t["name"] for t in more],
                          line=line)
    if box is not None and ex is not None:
        box.on_session(lambda session: _attach(ex, session))
    return out, new_resources, dict(handlers)


__all__ = ["FILTER", "FLAGS", "FREE", "FULL_URI", "HANDLERS", "ID", "OWN", "SESSION_LINE", "TOOLS", "direct",
           "is_core", "is_document", "render", "setup"]
