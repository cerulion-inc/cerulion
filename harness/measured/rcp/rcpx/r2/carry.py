# SPDX-License-Identifier: AGPL-3.0-only
"""The carry profile (SPEC-R3.md 4.6): what a weak model is served. Opt-in ("all,carry"; naming carry also loads
lean, rcpx/r2/core.py IMPLIES). It adds no tool and moves nothing: it decides what is declared, how each declared
tool is described, what the practices say and what each part's do line is, all from the tools the session has.

- setup: ex.state["carry"] = {on: true, lift_declared, declared, tools, packages, profile}; a pre hook on lift.
  lift_declared is the executor cfg r3_lift_declared (else ex.context["r3_lift_declared"]); only the value true
  counts, anything else is false.
- FILTER (runs before lean's: carry is earlier in PACKAGES): lift is listed only when lift_declared. When it is not,
  lift leaves the surface (so lean neither declares nor indexes it) but stays callable by its own name and through
  use (box.on_session puts its entry back in the session's callables; when a package's own FILTER took it off
  first, the entry is that package's TOOLS row, read through ex.mods), so a lift call meets a refusal, never an
  unknown tool. The pre hook refuses it with check lift_gate and the part's do line as the hint (the same check
  act's preflight makes; this one runs first, so lift never moves under carry while its gate is closed).
- RESOURCE_PATCH robot://devices: each part gains do = rcpx.sheet_words.do_line(part, document, the tools the
  session lists or reaches through use, robot://body, f0 from act's measured noise when act has measured it). A
  line that still asks for a tool the session does not reach (D4 and D6 say "locate" whatever is loaded) is made
  again without push and lift, so it falls to the sheet's own words through use, and says not_reached [names]. The
  document is copied first: the context's own document is never changed.
- LEAN_PROFILE and lean_profile(ex, surface): the profile lean serves (C1).

PART B (core lane C1, rcpx/r2/lean.py) must call, for a loaded package that exports LEAN_PROFILE:

    prof = mod.LEAN_PROFILE
    if callable(prof.get("resolve")):
        prof = prof["resolve"](ex, surface)     # carry.lean_profile(ex=None, surface=None) -> dict
    # prof: {name, declared, lines, practices, whole_uris, keep, never, free}
    #   declared  [tool names], in order: declare exactly these plus lean's own (tools, tool_help, use, last_result)
    #   lines     {tool name: description}: a declared tool without an entry keeps lean's one line
    #   practices callable(session) -> str (also takes None): serve it as robot://practices, and the practices the
    #             session had before as robot://practices/full
    #   whole_uris [URIs]: served whole (no lean cut)
    #   keep, never [keys]: extra lean_cut KEEP and NEVER keys
    #   free      [tool names]: answered without charging a call (calls_left unchanged)

surface is the merged surface lean's FILTER receives (carry's FILTER has run on it). Without C1 nothing of the
profile reaches the session: lean declares by its own FLAGS table, and robot://practices is lean's.
"""

import copy
import numbers

import inspect
from rcpx import bounds
from rcpx import r3_limits, sheet_words
from rcpx.context import refusal
from rcpx.r2 import carry_text

ID = "carry"
LIFT = "lift"
# 4.6: what carry declares, in this order; lift joins only when lift_declared. A name whose tool is not loaded is
# dropped (the carry-minus arms). lean adds tools, tool_help and use; claim and read_resource are the harness's.
DECLARED = ("survey", "look", "view", "locate", "push", "compare", "crop", "move_tip", "retreat", "park", "last_result")
WHOLE_URIS = ("robot://practices", "robot://task", "robot://devices")   # gap M12: the device sheet is never cut
KEEP = ("said", "next", "untried", "felt", "seen", "at", "candidates", "fits", "best_fit", "view", "part_check",
        "witness", "compared_with")
NEVER = ("said", "next", "untried", "refused", "hint")
# the tool names rcpx.sheet_words' do-line templates use (D1 to D7): a line naming one the session lacks is re-made
DO_TOOLS = ("locate", "push", "lift", "approach", "touch", "press", "tool_help", "use")
FREE = ("compare", "tool_help", "tools", "last_result")
LIFT_GATE_WHY = "lift is not enabled on this robot yet"
LIFT_GATE_SAID = "no lift: it is not enabled on this robot yet; the part's do line names the route through use"
LIFT_GATE_HINT = ("read the part's do line in robot://devices: it gives the route through use (approach, touch, then "
                  "press)")


def lift_declared(ex):
    """True only when the run declared lift (cfg r3_lift_declared true, set after the lift gate)."""
    cfg = getattr(ex, "cfg", None) or {}
    if "r3_lift_declared" in cfg:
        return cfg.get("r3_lift_declared") is True
    return (getattr(ex, "context", None) or {}).get("r3_lift_declared") is True


def packages(ex):
    return [m.__name__.rsplit(".", 1)[-1] for m in (getattr(ex, "mods", None) or [])]


def _state(ex):
    return ex.state.setdefault(ID, {"on": True, "lift_declared": lift_declared(ex), "declared": [], "tools": [],
                                    "packages": packages(ex), "profile": None})


def setup(ex):
    ex.state[ID] = {"on": True, "lift_declared": lift_declared(ex), "declared": [], "tools": [],
                    "packages": packages(ex), "profile": None}
    ex.add_pre(_lift_gate(ex), names=[LIFT])


# ---------------------------------------------------------------------------------------------------------------
# what the session reaches


def reach(ex, session=None):
    """The tool names the session lists or reaches through use: its callables (lean adds every tool it does not
    declare), else the names carry's FILTER saw; never lift while lift is not declared."""
    tools = getattr(session, "tools", None)
    names = set(tools) if isinstance(tools, dict) and tools else set(_state(ex).get("tools") or [])
    if not lift_declared(ex):
        names.discard(LIFT)
    return names


def _body(ex, session=None):
    """robot://body as the session serves it (the D5 route's up-facing tool surface), read once."""
    st = _state(ex)
    if isinstance(st.get("body"), dict):
        return st["body"]
    fn = (getattr(session, "resources", None) or {}).get("robot://body") or st.get("body_fn")
    doc = None
    try:
        if callable(fn):
            doc = fn()
        else:
            ctx = (getattr(ex, "context", None) or {}).get("body")
            doc = ctx() if callable(ctx) else ctx
    except Exception:       # a body that cannot be read: the D5 route says no surface faces up, never a guess
        doc = None
    if isinstance(doc, dict) and "refused" not in doc:
        st["body"] = doc
        return doc
    return None


def _noise_n(ex):
    """The force noise act measured (ex.state["act"]["force_noise_n"]); else the session's own: r3_limits.noise_n(B)
    once it exists, else the force noise the bounds hold at rest (0 until the robot measured it)."""
    v = (ex.state.get("act") or {}).get("force_noise_n")
    if isinstance(v, numbers.Real) and not isinstance(v, bool) and v > 0:
        return float(v)
    try:
        B = bounds.of(ex)
        fn = getattr(r3_limits, "noise_n", None)
        return float(fn(B)) if callable(fn) and _takes_bounds(fn) else float(B.sigma_f())
    except Exception:      # an executor with no bounds (a text rendered with no session): no noise is known
        return 0.0


def do_lines(ex, doc, session=None, tools=None):
    """A copy of the robot://devices document with each part's do line; the document when it has no parts (D8)."""
    if not isinstance(doc, dict) or not isinstance(doc.get("devices"), list):
        return doc
    out = copy.deepcopy(doc)
    plist = sheet_words.parts(out)
    if not plist:
        return doc
    names = set(tools) if tools is not None else reach(ex, session)
    body = _body(ex, session)
    noise = _noise_n(ex)
    found = [(p, _do_line(p, out, names, body, _f0(ex, p, noise))) for p in plist]
    for p, line in found:       # every line is made before any is written, so no line reads another's do
        if line is not None:
            p["entry"]["do"] = line
    return out


def _do_line(part, doc, names, body, f0_n):
    """sheet_words.do_line; when that line asks for a tool the session does not reach (a template names locate
    while sight is not loaded), the line made without the R3 verbs (the sheet's own words, through use)."""
    line = sheet_words.do_line(part, doc, names, body=body, f0_n=f0_n)
    if line is None:
        return None
    asked = [n for n in carry_text.named_tools(line["do"], DO_TOOLS) + list(line.get("verbs") or [])
             if n not in names]
    if not asked:
        return line
    line = sheet_words.do_line(part, doc, set(names) - {"push", LIFT}, body=body, f0_n=f0_n)
    return dict(line, not_reached=sorted(set(asked))) if line is not None else None


def _devices_patch(ex, session, content):
    return do_lines(ex, content, session)


RESOURCE_PATCH = {"robot://devices": _devices_patch}


# ---------------------------------------------------------------------------------------------------------------
# the lift gate


def _target_part(ex, target, doc):
    """The sheet part a lift target names (a world id's label, or a part label), else None."""
    plist = sheet_words.parts(doc)
    label = target
    world = getattr(ex, "world", None)
    get = getattr(world, "get", None)
    if isinstance(target, str) and callable(get):
        try:
            e = get(target)
        except Exception:
            e = None
        if e is not None:
            label = getattr(e, "label", None) if not isinstance(e, dict) else e.get("label")
    return sheet_words.label_part(label, plist) if isinstance(label, str) else None


def _lift_gate(ex):
    def pre(name, kwargs):
        if lift_declared(ex):
            return None
        hint = LIFT_GATE_HINT
        doc = (getattr(ex, "context", None) or {}).get("devices")
        try:
            part = _target_part(ex, (kwargs or {}).get("target"), doc) if isinstance(doc, dict) else None
            if part is not None:
                line = _do_line(part, doc, reach(ex), _body(ex), _f0(ex, part, _noise_n(ex)))
                if line is not None:
                    hint = f"the {part['name']}'s do line: {line['do']}"
        except Exception:   # the hint is help, never a reason to let lift run
            hint = LIFT_GATE_HINT
        return {"said": LIFT_GATE_SAID, **refusal("lift_gate", LIFT_GATE_WHY, hint)}
    return pre


# ---------------------------------------------------------------------------------------------------------------
# the profile lean serves (C1)


def practices_text(ex=None, session=None, tools=None):
    """6.1 as this session gets it (rcpx/r2/carry_text.py): the call budget is the session's."""
    budget = getattr(session, "budget", None)
    if isinstance(budget, dict) and isinstance(budget.get("calls"), numbers.Real):
        calls = int(budget["calls"])
    else:
        calls = None            # no budget is stated: line 7 names no count
    names = set(tools) if tools is not None else (reach(ex, session) if ex is not None else set())
    return carry_text.practices(names, packages(ex) if ex is not None else (), calls,
                                lift_listed=ex is not None and lift_declared(ex) and LIFT in names)


def lean_profile(ex=None, surface=None):
    """The profile for this session (module text, PART B): declared and described from the loaded, listed tools."""
    if surface is not None:
        entries = {t["name"]: t for t in surface.get("tools") or [] if isinstance(t, dict) and "name" in t}
        names = set(entries)
    else:
        names = set(_state(ex).get("tools") or []) if ex is not None else set(DECLARED) | {LIFT}
        entries = {}
    lift_ok = ex is not None and lift_declared(ex)
    if not lift_ok:
        names.discard(LIFT)
    declared = [n for n in DECLARED if n in names] + ([LIFT] if lift_ok and LIFT in names else [])
    schemas = {n: (e.get("args") or {}) for n, e in entries.items()}
    described = carry_text.lines(names, schemas)

    def practices(session=None):
        return practices_text(ex, session, tools=reach(ex, session) if ex is not None else names)

    return {"name": ID, "declared": declared, "lines": {n: described[n] for n in declared if n in described},
            "practices": practices, "whole_uris": list(WHOLE_URIS), "keep": list(KEEP), "never": list(NEVER),
            "free": [n for n in FREE if n in names]}


def FILTER(surface, resources, handlers, ex=None, box=None):
    """lift listed only when lift_declared (kept callable for its refusal); the profile for lean (module text)."""
    if ex is None:
        return surface, resources, handlers
    st = _state(ex)
    lift_ok = lift_declared(ex)
    tools = list(surface.get("tools") or [])
    hidden = [t for t in tools if t.get("name") == LIFT and not lift_ok]
    if not lift_ok and not hidden:
        hidden = _lift_entries(ex)       # a package's own FILTER took lift off the surface before carry's ran
    out = dict(surface, tools=[t for t in tools if not (t.get("name") == LIFT and not lift_ok)])
    names = [t["name"] for t in out["tools"]]
    st.update(lift_declared=lift_ok, tools=names, packages=packages(ex),
              declared=[n for n in DECLARED if n in names] + ([LIFT] if lift_ok and LIFT in names else []))
    st["profile"] = lean_profile(ex, out)
    if box is not None:
        box.on_session(lambda session: _attach(ex, session, hidden))
    return out, resources, handlers


def _attach(ex, session, hidden):
    """Once the session exists: lift kept callable (module text) and robot://body known to the lift gate's hint."""
    _keep_callable(session, hidden)
    fn = (getattr(session, "resources", None) or {}).get("robot://body")
    if callable(fn):
        _state(ex)["body_fn"] = fn


def _lift_entries(ex):
    """lift's tool entry from the loaded package that declares it, when the executor has the primitive (read from
    the package's TOOLS table through ex.mods; carry imports no package)."""
    if not callable(getattr(ex, LIFT, None)):
        return []
    for mod in getattr(ex, "mods", None) or []:
        for entry in getattr(mod, "TOOLS", None) or ():
            if isinstance(entry, dict) and entry.get("name") == LIFT and "extends" not in entry:
                return [dict(entry, **{"x-package": mod.__name__.rsplit(".", 1)[-1]})]
    return []


def _keep_callable(session, entries):
    tools = getattr(session, "tools", None)
    if isinstance(tools, dict):
        for entry in entries:
            tools.setdefault(entry["name"], entry)


def _static_profile():
    every = set(DECLARED) | {LIFT}
    return {"name": ID, "declared": list(DECLARED) + [LIFT], "lines": dict(carry_text.LINES),
            "practices": lambda session=None: carry_text.practices(every, ("sight", "act", "witness", "finish"),
                                                                    None, lift_listed=True),
            "whole_uris": list(WHOLE_URIS), "keep": list(KEEP), "never": list(NEVER), "free": list(FREE),
            "resolve": lean_profile}


# The profile with every carry tool present; "resolve" gives the one for a session's loaded set (PART B above).
LEAN_PROFILE = _static_profile()

__all__ = ["DECLARED", "FILTER", "FREE", "ID", "KEEP", "LEAN_PROFILE", "NEVER", "RESOURCE_PATCH", "WHOLE_URIS",
           "do_lines", "lean_profile", "lift_declared", "practices_text", "reach", "setup"]


def _takes_bounds(fn):
    try:
        return next(iter(inspect.signature(fn).parameters), None) in ("B", "bounds")
    except (TypeError, ValueError):
        return False


def _f0(ex, part, noise):
    """The first force a do line offers for the part (N). r3_limits.force_spec(B, "press", thing, q, u,
    sheet_forces=...)["first_n"] once it exists (the session's derived first force); until then r3_limits.f0 with
    the arguments it takes today (the Review stage removes that branch)."""
    spec = getattr(r3_limits, "force_spec", None)
    if callable(spec) and _takes_bounds(spec):
        return float(spec(bounds.of(ex), "press", part.get("name"), None, None, sheet_forces=part["forces"])["first_n"])
    return r3_limits.f0(noise, part["forces"])
