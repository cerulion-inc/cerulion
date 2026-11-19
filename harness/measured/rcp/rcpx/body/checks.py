# SPDX-License-Identifier: AGPL-3.0-only
"""Shape checks of the eight answers of the body contract (rcpx/body/contract.py).

Each check returns a list of plain sentences, one per fault found; an empty list is a pass. They are pure
functions of the answers: no body is called here, nothing is measured, and no check knows a kind of robot. The
conformance runner (tests/conformance) calls them on every answer an adapter gives.

What they hold an adapter to:
- every field of a description is {value, unit, source}, the source begins with a word of contract.SOURCES, and
  "unknown" is a legal value (then the source is "unknown" too);
- groups are declared: every joint is a member of a declared group or is listed under "ungrouped"; a holder names
  a group whose declared purpose is to hold, or a switched output; a tool point names a group that places one;
- at least one safe state is declared, each with what it does to motion and to power and how long it may be held;
- a controller may state nothing, and then lists what is to be measured;
- a refused recipe returns no sample; a sample carries its acquisition time;
- a command's answer says what was applied, and "not clipped" means applied equals asked.
"""

from rcpx.body import contract as C


def _is_number(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def check_field(field, where):
    """A stated field: {value, unit, source}; "unknown" is legal."""
    bad = []
    if not isinstance(field, dict) or "value" not in field or "source" not in field or "unit" not in field:
        return [f"{where}: not a field of the form value, unit, source"]
    word = C.source_word(field["source"])
    if field["value"] == C.UNKNOWN:
        if word != C.UNKNOWN:
            bad.append(f"{where}: an unknown value names a source ({field['source']!r})")
    elif word not in C.SOURCES:
        bad.append(f"{where}: source {field['source']!r} does not begin with a word of the closed list")
    return bad


def _names(items, where, bad):
    names = []
    for i, it in enumerate(items or ()):
        n = it.get("name") if isinstance(it, dict) else None
        if not isinstance(n, str) or not n:
            bad.append(f"{where}[{i}]: no name")
            continue
        if n in names:
            bad.append(f"{where}: the name {n!r} is declared twice")
        names.append(n)
    return names


def _words(item, key, where, bad):
    if not isinstance(item.get(key), str) or not item[key].strip():
        bad.append(f"{where}: no {key!r} in words")


def _source(item, where, bad):
    word = C.source_word(item.get("source", ""))
    if word not in C.SOURCES:
        bad.append(f"{where}: source {item.get('source')!r} does not begin with a word of the closed list")


def check_describe(d):
    bad = []
    if not isinstance(d, dict):
        return ["describe: the answer is not an object"]
    for key in C.DESCRIBE_KEYS:
        if key not in d:
            bad.append(f"describe: no {key!r}")
    if bad:
        return bad
    for key, field in (d["identity"] or {}).items():
        bad += check_field(field, f"identity.{key}")
    joints = _names(d["joints"], "joints", bad)
    for j in d["joints"]:
        for key in ("range", "top_speed", "top_effort"):
            if key not in j:
                bad.append(f"joint {j.get('name')!r}: no {key!r} (say unknown where nobody stated it)")
            else:
                bad += check_field(j[key], f"joint {j.get('name')!r}.{key}")
    outputs = _names(d["outputs"], "outputs", bad)
    for o in d["outputs"]:
        where = f"output {o.get('name')!r}"
        _words(o, "switches", where, bad)
        _words(o, "effect", where, bad)
        _source(o, where, bad)
        if o.get("mode") not in ("on or off", "level"):
            bad.append(f"{where}: mode is {o.get('mode')!r}, not on or off or a level")
    groups = _names(d["groups"], "groups", bad)
    grouped = set()
    purpose = {}
    for g in d["groups"]:
        where = f"group {g.get('name')!r}"
        purpose[g.get("name")] = g.get("purpose")
        if g.get("purpose") not in C.PURPOSES:
            bad.append(f"{where}: purpose {g.get('purpose')!r} is not declared from the closed list")
        _words(g, "for", where, bad)
        _source(g, where, bad)
        members = g.get("members")
        if not isinstance(members, list) or not members:
            bad.append(f"{where}: no members")
            continue
        for m in members:
            if m not in joints and m not in outputs:
                bad.append(f"{where}: member {m!r} is neither a declared joint nor a declared output")
            grouped.add(m)
    ungrouped = d.get("ungrouped") or []
    for j in joints:
        if j not in grouped and j not in ungrouped:
            bad.append(f"joint {j!r} is in no declared group and not listed as ungrouped (a group is never guessed)")
    for j in ungrouped:
        if j in grouped:
            bad.append(f"joint {j!r} is listed as ungrouped and is a member of a declared group")
    _names(d["tool_points"], "tool_points", bad)
    for t in d["tool_points"]:
        where = f"tool point {t.get('name')!r}"
        if purpose.get(t.get("group")) != "places a tool point":
            bad.append(f"{where}: its group {t.get('group')!r} is not a declared group that places a tool point")
    _names(d["holders"], "holders", bad)
    for h in d["holders"]:
        where = f"holder {h.get('name')!r}"
        _words(h, "how", where, bad)
        _source(h, where, bad)
        if h.get("group") is not None:
            if purpose.get(h["group"]) != "holds":
                bad.append(f"{where}: its group {h['group']!r} is declared for "
                           f"{purpose.get(h['group'])!r}, not to hold")
        elif h.get("output") not in outputs:
            bad.append(f"{where}: names neither a declared group that holds nor a declared output")
        sig = h.get("hold_signals")
        if not isinstance(sig, list) or not sig:
            bad.append(f"{where}: no hold signals (the signals that change when a load is attached)")
        for s in sig or ():
            if not isinstance(s, dict) or not isinstance(s.get("signal"), str) or "held_when" not in s:
                bad.append(f"{where}: a hold signal needs a signal name and held_when (a rule, or unknown)")
    _names(d["sensors"], "sensors", bad)
    for s in d["sensors"]:
        where = f"sensor {s.get('name')!r}"
        _words(s, "measures", where, bad)
        _source(s, where, bad)
    states = _names(d["safe_states"], "safe_states", bad)
    if not states:
        bad.append("no safe state is declared: a session on this body cannot be closed")
    for s in d["safe_states"]:
        where = f"safe state {s.get('name')!r}"
        if s.get("kind") not in C.SAFE_KINDS:
            bad.append(f"{where}: kind {s.get('kind')!r} is not one of the stop kinds")
        _words(s, "motion", where, bad)
        _words(s, "power", where, bad)
        _source(s, where, bad)
        if "may_be_held_for" not in s:
            bad.append(f"{where}: does not say how long it may be held (say unknown where nobody stated it)")
        else:
            bad += check_field(s["may_be_held_for"], f"{where}.may_be_held_for")
        if not isinstance(s.get("reads_as"), list):
            bad.append(f"{where}: no reads_as (the signals whose readings show the state; an empty list when none)")
    return bad


def _check_entry(entry, where, bad):
    if not isinstance(entry, dict) or "unit" not in entry:
        bad.append(f"{where}: no unit")
        return
    for key in ("range", "resolution", "rate"):
        if key not in entry:
            bad.append(f"{where}: no {key!r} (say unknown where the body states none)")
        else:
            bad.extend(check_field(entry[key], f"{where}.{key}"))


def check_interfaces(i, description):
    bad = []
    if not isinstance(i, dict):
        return ["interfaces: the answer is not an object"]
    for key in ("groups", "sensors", "outputs", "signals"):
        if key not in i:
            bad.append(f"interfaces: no {key!r}")
    if bad:
        return bad
    declared = {g["name"] for g in description.get("groups") or ()}
    for name, face in i["groups"].items():
        if name not in declared:
            bad.append(f"interfaces: group {name!r} is not declared in describe()")
        for mode, entry in (face.get("command") or {}).items():
            if mode not in C.COMMAND_MODES:
                bad.append(f"interfaces: group {name!r} lists the command mode {mode!r}, not one of the contract's")
            _check_entry(entry, f"interfaces {name}.command.{mode}", bad)
        for quantity, entry in (face.get("read") or {}).items():
            _check_entry(entry, f"interfaces {name}.read.{quantity}", bad)
    for kind, key in (("sensor", "sensors"), ("output", "outputs")):
        declared_k = {x["name"] for x in description.get(key) or ()}
        for name, face in i[key].items():
            if name not in declared_k:
                bad.append(f"interfaces: {kind} {name!r} is not declared in describe()")
            for mode, entry in (face.get("command") or {}).items():
                if mode not in C.COMMAND_MODES:
                    bad.append(f"interfaces: {kind} {name!r} lists the command mode {mode!r}")
                _check_entry(entry, f"interfaces {name}.command.{mode}", bad)
            for quantity, entry in (face.get("read") or {}).items():
                _check_entry(entry, f"interfaces {name}.read.{quantity}", bad)
    signals = i["signals"]
    if not isinstance(signals, list) or not all(isinstance(s, str) for s in signals):
        bad.append("interfaces: signals is not a list of names")
    else:
        for need in (C.TIME_SIGNAL, C.COMMANDS_SIGNAL, C.FAULTS_SIGNAL):
            if need not in signals:
                bad.append(f"interfaces: the signal {need!r} every adapter serves is missing")
    return bad


def check_controller(c):
    bad = []
    if not isinstance(c, dict):
        return ["controller: the answer is not an object"]
    for key in C.CONTROLLER_KEYS + ("to_be_measured",):
        if key not in c:
            bad.append(f"controller: no {key!r}")
    if bad:
        return bad
    law = c["law"]
    fields = {"law.kind": law.get("kind") if isinstance(law, dict) else None,
              "law.gains": law.get("gains") if isinstance(law, dict) else None,
              "period": c["period"], "holds_last_target": c["holds_last_target"],
              "stiffness_can_be_lowered": c["stiffness_can_be_lowered"]}
    unknown = []
    for name, field in fields.items():
        bad += check_field(field, f"controller.{name}")
        if C.is_unknown(field):
            unknown.append(name)
    own = c["on_its_own"]
    if C.is_unknown(own):
        unknown.append("on_its_own")
    elif not isinstance(own, list):
        bad.append("controller.on_its_own: neither a list of what the controller does on its own nor unknown")
    else:
        for k, item in enumerate(own):
            if not isinstance(item, dict) or not isinstance(item.get("what"), str):
                bad.append(f"controller.on_its_own[{k}]: no 'what' in words")
            else:
                _source(item, f"controller.on_its_own[{k}]", bad)
    if sorted(c["to_be_measured"]) != sorted(unknown):
        bad.append(f"controller.to_be_measured lists {sorted(c['to_be_measured'])}, the unknown fields are "
                   f"{sorted(unknown)}")
    if not C.is_unknown(c["period"]) and not (_is_number(c["period"].get("value")) and c["period"]["value"] > 0):
        bad.append("controller.period: neither a positive number nor unknown")
    for group, part in (c.get("by_group") or {}).items():
        bad += [f"by_group {group}: {b}" for b in check_controller(part)]
    return bad


def check_read(answer, recipe, signals):
    """recipe: the names asked. signals: the names interfaces() lists."""
    bad = []
    unknown = [n for n in recipe if n not in signals]
    if unknown:
        if not C.is_refusal(answer):
            bad.append(f"read: the recipe names unknown signals {unknown} and was not refused whole")
        elif "samples" in answer:
            bad.append("read: a refused recipe returned samples")
        return bad
    if C.is_refusal(answer):
        return [f"read: a recipe of known signals was refused ({answer['refused']})"]
    samples = answer.get("samples") if isinstance(answer, dict) else None
    if not isinstance(samples, dict) or sorted(samples) != sorted(set(recipe)):
        return [f"read: the samples are not exactly the signals asked ({sorted(recipe)})"]
    for name, s in samples.items():
        if not isinstance(s, dict) or "value" not in s or "unit" not in s:
            bad.append(f"read: sample {name!r} has no value and unit")
            continue
        if not _is_number(s.get("t")):
            bad.append(f"read: sample {name!r} carries no acquisition time")
        if C.source_word(s.get("source", "")) not in C.SOURCES:
            bad.append(f"read: sample {name!r} has source {s.get('source')!r}, not of the closed list")
    return bad


def check_command(answer, group, mode, setpoint, limits):
    if C.is_refusal(answer):
        return [] if answer["refused"].get("check") and answer["refused"].get("why") else [
            "command: a refusal names no check or no reason"]
    bad = []
    if not isinstance(answer, dict):
        return ["command: the answer is not an object"]
    for key in ("group", "mode", "asked", "applied", "clipped", "action", "t"):
        if key not in answer:
            bad.append(f"command: no {key!r} in the answer")
    if bad:
        return bad
    if answer["group"] != group or answer["mode"] != mode:
        bad.append("command: the answer names another group or mode than the one commanded")
    if answer["action"] != (limits or {}).get("action"):
        bad.append("command: the answer does not carry the action id the command came with")
    if answer["clipped"] not in (True, False, C.UNKNOWN):
        bad.append("command: clipped is not true, false or unknown")
    if answer["asked"] != setpoint:
        bad.append("command: asked is not the setpoint that was sent")
    if answer["clipped"] is False and answer["applied"] != answer["asked"]:
        bad.append("command: says not clipped, and applied differs from asked")
    if answer["clipped"] is True and not answer.get("clipped_by"):
        bad.append("command: says clipped and does not say by what")
    if not _is_number(answer["t"]):
        bad.append("command: no time")
    return bad


def check_capture(answer, sensor):
    if C.is_refusal(answer):
        return []
    bad = []
    if not isinstance(answer, dict):
        return ["capture: the answer is not an object"]
    for key in ("sensor", "frame", "t", "pose", "exposure", "noise"):
        if key not in answer:
            bad.append(f"capture: no {key!r}")
    if bad:
        return bad
    if answer["sensor"] != sensor:
        bad.append("capture: the answer names another sensor")
    if not _is_number(answer["t"]):
        bad.append("capture: no acquisition time")
    for key in ("pose", "exposure", "noise"):
        bad += check_field(answer[key], f"capture.{key}")
    return bad


def check_advance(answer):
    """One period passed; or none did and the answer carries the fault the body reported."""
    bad = []
    if not isinstance(answer, dict):
        return ["advance: the answer is not an object"]
    if answer.get("periods") == 0:
        if not answer.get("faults"):
            bad.append("advance: no period passed and no fault is reported")
    elif answer.get("periods") != 1:
        bad.append("advance: the answer does not say one period passed")
    if not _is_number(answer.get("t")):
        bad.append("advance: no time after the period")
    return bad


def check_halt(answer, state, description):
    declared = C.safe_state(description, state)
    if C.is_refusal(answer):
        return [f"halt: the declared safe state {state!r} was refused ({answer['refused']})"] if declared else []
    bad = []
    if declared is None:
        return [f"halt: {state!r} is not a declared safe state and was not refused"]
    if not isinstance(answer, dict):
        return ["halt: the answer is not an object"]
    for key in ("state", "did", "read_back", "reached", "faults"):
        if key not in answer:
            bad.append(f"halt: no {key!r}")
    if bad:
        return bad
    if answer["state"] != state:
        bad.append("halt: the answer names another state than the one asked")
    if answer["reached"] not in ("yes", "no", "cannot tell"):
        bad.append("halt: reached is not yes, no or cannot tell")
    back = answer["read_back"]
    if not isinstance(back, dict) or not back:
        bad.append("halt: nothing was read back from the body")
    else:
        for name, s in back.items():
            if not isinstance(s, dict) or not _is_number(s.get("t")):
                bad.append(f"halt: read-back sample {name!r} carries no acquisition time")
        for cond in declared.get("reads_as") or ():
            if cond.get("signal") not in back:
                bad.append(f"halt: the signal {cond.get('signal')!r} that shows this state was not read back")
    if not isinstance(answer["faults"], list) and not C.is_unknown(answer["faults"]):
        bad.append("halt: faults is neither a list nor unknown")
    return bad


def _value(sample):
    return sample.get("value") if isinstance(sample, dict) else None


def _close(a, b, band):
    """a equals b; with a stated band, every number of a lies within it of b."""
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_close(x, y, band) for x, y in zip(a, b))
    if _is_number(a) and _is_number(b):
        return abs(a - b) <= band if band is not None else a == b
    return a == b


def reads_as(state, samples, applied=None):
    """Whether the body's own readings show a declared safe state: "yes" | "no" | "cannot tell".

    state["reads_as"] is a list of conditions, each on one signal:
        {"signal": S, "equals": v}                 the reading is v
        {"signal": S, "equals_applied": group}     the reading is the setpoint the halt applied to that group
        either may carry "within": a stated field {value, unit, source}, the band inside which numbers count as
        equal (stated by the body, never typed here).
    samples: {signal: sample}. applied: {group: setpoint} as the halt answered it. A state that declares no
    condition, or a condition whose signal was not read, cannot be told."""
    conds = state.get("reads_as") or ()
    if not conds:
        return "cannot tell"
    verdict = "yes"
    for cond in conds:
        if cond.get("signal") not in samples:
            return "cannot tell"
        got = _value(samples[cond["signal"]])
        band = cond.get("within", {}).get("value") if isinstance(cond.get("within"), dict) else None
        if "equals_applied" in cond:
            if applied is None or cond["equals_applied"] not in applied:
                return "cannot tell"
            want = applied[cond["equals_applied"]]
        else:
            want = cond.get("equals")
        if not _close(got, want, band if _is_number(band) else None):
            verdict = "no"
    return verdict


def held(holder, samples):
    """Whether a declared holder's hold signals read held: "held" | "not held" | "cannot tell".

    Each hold signal is {"signal": S, "held_when": rule}; a rule is {"at_least": field} or {"at_most": field}
    (a stated field in the signal's own unit), or "unknown" where the body states none: then nothing can be told
    and the until condition "lost hold" is not offered on this body. Every decidable rule must hold."""
    verdict = "cannot tell"
    for sig in holder.get("hold_signals") or ():
        rule = sig.get("held_when")
        if not isinstance(rule, dict) or sig.get("signal") not in samples:
            continue
        got = _value(samples[sig["signal"]])
        if isinstance(got, (list, tuple)):
            got = min(got) if "at_least" in rule else max(got)
        if not _is_number(got):
            continue
        if "at_least" in rule and _is_number(rule["at_least"].get("value")):
            ok = got >= rule["at_least"]["value"]
        elif "at_most" in rule and _is_number(rule["at_most"].get("value")):
            ok = got <= rule["at_most"]["value"]
        else:
            continue
        if not ok:
            return "not held"
        verdict = "held"
    return verdict
