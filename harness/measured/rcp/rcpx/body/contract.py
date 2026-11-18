# SPDX-License-Identifier: AGPL-3.0-only
"""C0, the body contract: the eight calls between one body's adapter and the kernel (ARCHITECTURE.md section 2.3).

An adapter makes one body (a simulator, an environment host, a real machine's driver) speak these eight calls.
Every answer is plain data (dicts, lists, numbers, strings), so the same request and answer pairs can be checked
in any language (tests/conformance). A call the body cannot honour answers a refusal, {"refused": {"check",
"why"}}, and changes nothing; it never raises for a reason the caller could have caused.

    describe()      what the body states about itself. Every field is {value, unit, source}; the value
                    "unknown" is legal. Holds the declared groups (members, and what each is for), the tool
                    points, the holders, the sensors (each says what it measures), the switched outputs (anything
                    commanded that is not a joint) and the body's safe states. Groups are declared, never
                    inferred from the joint tree; a joint in no declared group is listed under "ungrouped".
    interfaces()    per joint, sensor and output: what can be commanded and what can be read, with rate,
                    resolution and range as the body states them.
    controller()    the servo law in force (kind, gains) or "unknown"; the loop period as measured; whether it
                    can hold its last target; whether its stiffness can be lowered; what it does on its own.
                    What nobody stated is listed under "to_be_measured". A controller that states nothing is
                    legal.
    read(recipe)    the named signals, each sample with the time it was acquired. An unknown signal name
                    refuses the whole recipe.
    command(group, mode, setpoint, limits)
                    returns what was actually applied (clipped or not). limits carries "action", the id of the
                    admitted action the command belongs to; a command with no action id is refused, and the
                    adapter counts commands by action id (readable as the signal "body.commands").
    capture(sensor) a frame or reading, its acquisition time, the sensor's pose at that time, exposure and
                    noise as far as the body states them.
    advance()       one control period passes (a simulator steps; a real body waits for its tick).
    halt(state)     one of the body's declared safe states, then the faults the body reports. The answer is read
                    back from the body, never taken from the acceptance of a command.

Guarantees the adapter gives: SI units, or the body's own unit named as such; acquisition times; an unknown signal
name fails the whole recipe; an answer never reports more than the body did; an exit state is read back from the
body. Guarantees the kernel gives: every command belongs to an admitted action; no command exceeds a stated
rating; a front that goes silent leads to a safe stop.

The lists below are closed lists about a robot's own motion and about where knowledge comes from. None names a
kind of robot, device or task. Shapes are checked by rcpx/body/checks.py.
"""

from abc import ABC, abstractmethod

UNKNOWN = "unknown"

# where a stated or measured value comes from (ARCHITECTURE.md section 2.6): a closed list about knowledge
SOURCES = ("description", "firmware", "self-measured", "contact", "seen", "swept", "operator", "owner", "protocol",
           "document", "equipment", "prior")

# what a declared group is for (section 2.6, the Card)
PURPOSES = ("places a tool point", "holds", "moves the base frame", "aims a sensor", "other")

# what can be commanded on a joint or an output (section 2.3, interfaces)
COMMAND_MODES = ("position", "speed", "effort", "stiffness", "on or off", "level")

# the kinds of safe state, after the stop categories of machine practice (section 2.3, the third amendment)
SAFE_KINDS = ("power removed", "controlled stop, then power removed", "controlled stop with power kept")

CALLS = ("describe", "interfaces", "controller", "read", "command", "capture", "advance", "halt")

DESCRIBE_KEYS = ("identity", "joints", "groups", "tool_points", "holders", "sensors", "outputs", "safe_states")
CONTROLLER_KEYS = ("law", "period", "holds_last_target", "stiffness_can_be_lowered", "on_its_own")

# signals every adapter serves beside its own
TIME_SIGNAL = "body.time"            # seconds on the body's own clock
COMMANDS_SIGNAL = "body.commands"    # {action id: commands applied}, counted by the adapter
FAULTS_SIGNAL = "body.faults"        # the faults the body reports now (an empty list when it reports none)


def stated(value, unit=None, source=UNKNOWN, **more):
    """One field of a description: {value, unit, source} and whatever else the body says about it (spread, age,
    how it was learned). source is one of SOURCES, optionally followed by ": " and where exactly (a file, a
    document, a measurement), or "unknown" when the value is "unknown"."""
    out = {"value": value, "unit": unit, "source": source}
    out.update(more)
    return out


def unknown(why=None):
    """A field nobody stated. Legal everywhere a stated field is; the kernel lists it as to be measured."""
    out = {"value": UNKNOWN, "unit": None, "source": UNKNOWN}
    if why:
        out["why"] = str(why)
    return out


def is_unknown(field):
    return isinstance(field, dict) and field.get("value") == UNKNOWN


def refusal(check, why, **more):
    """The answer of a call the body cannot honour. Nothing was changed by the call."""
    out = {"refused": {"check": str(check), "why": str(why)}}
    out["refused"].update(more)
    return out


def is_refusal(answer):
    return isinstance(answer, dict) and isinstance(answer.get("refused"), dict)


def source_word(source):
    """The closed-list word a source string starts with ("description: the model file" -> "description")."""
    return str(source).split(":", 1)[0].strip()


def group_named(description, name):
    for g in description.get("groups") or ():
        if g.get("name") == name:
            return g
    return None


def groups_for(description, purpose):
    """The declared groups with this purpose, in declared order. Reads the declaration only: it never looks at
    the joint tree."""
    return [g for g in description.get("groups") or () if g.get("purpose") == purpose]


def holder_groups(description):
    """Names of the groups a declared holder closes with. A group no holder names is not a hand, whatever its
    joints look like."""
    return [h["group"] for h in description.get("holders") or () if h.get("group")]


def safe_state(description, name):
    for s in description.get("safe_states") or ():
        if s.get("name") == name:
            return s
    return None


class Body(ABC):
    """One body behind the eight calls. Answers are plain data; see the module text for each call."""

    @abstractmethod
    def describe(self) -> dict:
        """{"identity": {...}, "joints": [{"name", "kind", "range", "top_speed", "top_effort"}],
        "groups": [{"name", "members": [joint or output names], "purpose": one of PURPOSES, "for": words,
        "source"}], "ungrouped": [joint names], "tool_points": [{"name", "group", "link", "faces"}],
        "holders": [{"name", "group" | "output", "how", "hold_signals": [{"signal", "held_when"}], "source"}],
        "sensors": [{"name", "measures", "mount", "source"}],
        "outputs": [{"name", "switches", "effect", "mode", "source"}],
        "safe_states": [{"name", "kind": one of SAFE_KINDS, "motion", "power", "may_be_held_for", "reads_as",
        "source"}]}. The first safe state listed is the one a session closes to."""

    @abstractmethod
    def interfaces(self) -> dict:
        """{"groups": {name: {"command": {mode: {"unit", "range", "resolution", "rate"}},
        "read": {quantity: {"unit", "range", "resolution", "rate"}}}}, "sensors": {name: {"read": {...}}},
        "outputs": {name: {"command": {...}, "read": {...}}}, "signals": [every name read() accepts]}."""

    @abstractmethod
    def controller(self) -> dict:
        """{"law": {"kind", "gains"}, "period", "holds_last_target", "stiffness_can_be_lowered",
        "on_its_own": [...], "to_be_measured": [names of the fields above nobody stated]}."""

    @abstractmethod
    def read(self, recipe) -> dict:
        """recipe: a list of signal names. {"samples": {name: {"value", "unit", "t", "source"}}} or a refusal
        that names every unknown signal; nothing of a refused recipe is returned."""

    @abstractmethod
    def command(self, group, mode, setpoint, limits) -> dict:
        """{"group", "mode", "asked", "applied", "clipped": true | false | "unknown", "clipped_by", "limits",
        "action", "t"} or a refusal (an undeclared group, a mode the group's interface does not list, a setpoint
        of the wrong size, no action id). A refused command changes nothing."""

    @abstractmethod
    def capture(self, sensor) -> dict:
        """{"sensor", "frame", "t", "pose", "exposure", "noise", "geometry"} or a refusal (an undeclared sensor)."""

    @abstractmethod
    def advance(self) -> dict:
        """{"periods": 1, "t": the body's time after the period} and, when the body reported one, "faults"."""

    @abstractmethod
    def halt(self, state) -> dict:
        """{"state", "did", "applied", "read_back": {signal: sample}, "reached": "yes" | "no" | "cannot tell",
        "faults"} or a refusal (a state the body did not declare; nothing changed). reached is judged on
        read_back, the body's own readings taken after the state was commanded."""


def controller_answer(declared, period=None):
    """The controller() answer from what a body's owner declared (a dict with any of law, period,
    holds_last_target, stiffness_can_be_lowered, on_its_own, by_group; or None for a controller that states
    nothing) and the period the adapter measured (a stated field, or None). Every field nobody stated is
    "unknown" and is listed under to_be_measured. A body whose groups run under different laws keeps them under
    by_group: {group: the same fields}, each with its own to_be_measured."""
    d = declared if isinstance(declared, dict) else {}
    law = d.get("law") if isinstance(d.get("law"), dict) else {}
    out = {"law": {"kind": law.get("kind") or unknown(), "gains": law.get("gains") or unknown()},
           "period": period or d.get("period") or unknown(),
           "holds_last_target": d.get("holds_last_target") or unknown(),
           "stiffness_can_be_lowered": d.get("stiffness_can_be_lowered") or unknown(),
           "on_its_own": d.get("on_its_own") if d.get("on_its_own") is not None else unknown()}
    flat = {"law.kind": out["law"]["kind"], "law.gains": out["law"]["gains"], "period": out["period"],
            "holds_last_target": out["holds_last_target"],
            "stiffness_can_be_lowered": out["stiffness_can_be_lowered"], "on_its_own": out["on_its_own"]}
    out["to_be_measured"] = sorted(k for k, v in flat.items() if is_unknown(v))
    if isinstance(d.get("by_group"), dict):
        out["by_group"] = {}
        for group, part in d["by_group"].items():
            sub = controller_answer(dict(part, period=part.get("period") or out["period"]))
            sub.pop("by_group", None)
            out["by_group"][group] = sub
    return out


# the checks a refusal names, for the cases every adapter meets; an adapter may add its own
REFUSAL_CHECKS = ("unknown_signal", "undeclared_group", "mode", "setpoint", "limit", "no_action",
                  "undeclared_sensor", "undeclared_state", "cannot_honour")
