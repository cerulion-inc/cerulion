# SPDX-License-Identifier: AGPL-3.0-only
"""What the chat operator (bench/operators/chat_operator.py) gives the model and how it reads the model's replies:
the tool definitions from the episode server's list answer, the first user message, argument parsing and the
reply's tool calls. Pure functions of their inputs (no socket, no endpoint, no clock).
"""
import json
import os

from bench.operators.portable import plain_schema

READ_TOOL, CLAIM_TOOL = "read_resource", "claim"
TASK_URI, PRACTICES_URI = "robot://task", "robot://practices"


# ---------------------------------------------------------------------------------------------------------------
# What the model is given


NUDGE = "call a tool or claim"
NUDGE_LISTED = "call a tool"
CUT_OFF_NOTE = ("Your last reply was cut off at the reply limit of {n} tokens (thinking counts against it) before "
                "any tool call. Think briefly, then {then}.")
FORMAT_NOTE = ("Two replies in a row had no tool call, so nothing ran. A tool runs only when you call it through the "
               "tool-calling interface: one function call with the tool's name and a JSON object of arguments, not "
               "written out in the reply text. For example, the tool {name} with the arguments {args}. When you are "
               "done, call claim with a statement and success true or false. If the next reply has no tool call "
               "either, the episode ends.")
TEXT_CALL_NOTE = ("Your last reply wrote the call {name} as text instead of calling the tool. It was run as a call "
                  "and its real result is above; the text you wrote after the call was dropped, it is not a result. "
                  "Call tools through the tool-calling interface.")
REPLIES_CUT_OFF = "operator stopped: the model's last reply was cut off at the token limit before any tool call"
BUDGET_NOTICE = ("The budget is spent: no more tool calls will run. Call claim now with an honest statement of "
                 "what you did and what you saw; say plainly what you could not do or could not see.")
LAST_TURN_NOTICE = ("This is your last turn. Call claim now with an honest statement of what you did and what you "
                    "saw; say plainly what you could not do or could not see.")
CLAIM_NOW = "call claim now"
STOPPED_CALLING = "operator stopped calling tools"
NO_CLAIM_AFTER_BUDGET = "operator did not claim after the budget was spent"
TURN_LIMIT = "operator reached its turn limit without a claim"
NOT_RUN = "not run: one call per reply"
ONE_CALL_RULE = ("One tool call per reply: only the first tool call of a reply runs; any other call in the "
                 "same reply is not run.")
REPLY_BUDGET_SPENT = "operator stopped: the reply budget was spent without a claim"
BUDGET_SPENT_STOP = "operator stopped: the budget was spent"
FIRST_MESSAGE_KEYS = ("text", "images", "state", "system", "own_tools")


def describe_tool(entry):
    """A tool's description followed by its worked example (as rcpx/mcp_stdio.py words it)."""
    text = str(entry.get("description", "")).strip()
    example = entry.get("example") or {}
    if "args" in example:
        means = str(example.get("means", "")).strip()
        text += f" Example arguments {json.dumps(example['args'])}" + (f": {means}." if means else ".")
    return text


def _function(name, description, parameters):
    return {"type": "function", "function": {"name": name, "description": description, "parameters": parameters}}


def build_tools(listing, plain=False, own_tools=True):
    """(tool definitions, {name: argument schema}, names the listing had that the operator's own tools took). plain:
    the definitions carry bench.operators.portable.plain_schema of each schema (bounds said in words); the returned
    schemas are always the full ones."""
    tools, schemas, taken = [], {}, []
    for entry in listing.get("tools", []):
        name = entry.get("name")
        if own_tools and name in (READ_TOOL, CLAIM_TOOL):
            taken.append(name)
            continue
        params = entry.get("args") or {"type": "object", "properties": {}}
        tools.append(_function(name, describe_tool(entry), plain_schema(params) if plain else params))
        schemas[name] = params
    if not own_tools:
        return tools, schemas, taken
    resources = [r for r in listing.get("resources", []) if isinstance(r, dict) and r.get("uri")]
    listed = "; ".join(f"{r['uri']}: {r.get('description', '')}".rstrip(": ") for r in resources)
    first = resources[0]["uri"] if resources else TASK_URI
    read_params = {"type": "object", "additionalProperties": False, "required": ["uri"],
                   "properties": {"uri": {"type": "string", "description": "the resource URI; a {name} part in a "
                                                                           "listed URI is filled in by you"}}}
    tools.append(_function(READ_TOOL, f"Read one of the robot's resources by URI. Resources: {listed}. Example "
                                      f"arguments {json.dumps({'uri': first})}.", read_params))
    schemas[READ_TOOL] = read_params
    claim = listing.get("claim") or {}
    claim_params = claim.get("args") or {"type": "object", "required": ["statement", "success"],
                                         "properties": {"statement": {"type": "string"},
                                                        "success": {"type": "boolean"}}}
    claim_text = describe_tool(claim) if claim.get("description") else (
        "End the episode with your own statement of what you achieved; nothing else runs after it.")
    tools.append(_function(CLAIM_TOOL, claim_text, plain_schema(claim_params) if plain else claim_params))
    schemas[CLAIM_TOOL] = claim_params
    return tools, schemas, taken


def call_example(listing):
    """(name, arguments) for the format note: the first listed tool whose worked example has arguments, else the
    first with an example at all, else read_resource of robot://task."""
    with_args = None
    for entry in listing.get("tools", []):
        example = entry.get("example") or {}
        if entry.get("name") in (READ_TOOL, CLAIM_TOOL) or not isinstance(example.get("args"), dict):
            continue
        if example["args"]:
            return entry["name"], example["args"]
        with_args = with_args or (entry["name"], example["args"])
    return with_args or (READ_TOOL, {"uri": TASK_URI})


def format_note(listing, own_tools=True):
    """The call-format reminder, naming only tools offered to the model."""
    if own_tools:
        name, args = call_example(listing)
        return FORMAT_NOTE.format(name=name, args=json.dumps(args))
    chosen = None
    for entry in listing.get("tools", []):
        example = entry.get("example") or {}
        if not isinstance(example.get("args"), dict):
            continue
        chosen = chosen or (entry["name"], example["args"])
        if example["args"]:
            chosen = entry["name"], example["args"]
            break
    note = ("Two replies in a row had no tool call, so nothing ran. A tool runs only when you call it through the "
            "tool-calling interface: one function call with the tool's name and a JSON object of arguments, not "
            "written out in the reply text.")
    if chosen is not None:
        name, args = chosen
        note += f" For example, the tool {name} with the arguments {json.dumps(args)}."
    return note + " If the next reply has no tool call either, the episode ends."


def positional_names(schema):
    """The argument names a positional value in a call written as text fills, in order: required, then the rest."""
    props = list((schema or {}).get("properties") or {})
    required = [n for n in (schema or {}).get("required") or [] if n in props]
    return required + [n for n in props if n not in required]


def task_sentence(content):
    """The task sentence of the robot://task content (a text, or an object holding it)."""
    if isinstance(content, str):
        return content.strip()
    if isinstance(content, dict):
        for key in ("instruction", "task", "sentence", "text"):
            if isinstance(content.get(key), str) and content[key].strip():
                return content[key].strip()
    return json.dumps(content, ensure_ascii=False)


def first_message_of(content):
    """Return (spec, None), (None, why) if malformed, or (None, None) if no first_message is present."""
    if not isinstance(content, dict) or "first_message" not in content:
        return None, None
    value = content["first_message"]
    if not isinstance(value, dict):
        return None, "first_message must be a JSON object"
    unknown = sorted(set(value) - set(FIRST_MESSAGE_KEYS))
    if unknown:
        return None, (f"first_message has unknown keys: {', '.join(unknown)} "
                      f"(known: {', '.join(FIRST_MESSAGE_KEYS)})")
    if not isinstance(value.get("text"), str) or not value["text"].strip():
        return None, "first_message.text must be a text that is not empty"
    if "images" in value and not isinstance(value["images"], list):
        return None, "first_message.images must be a list"
    images, seen = [], set()
    for i, item in enumerate(value.get("images", [])):
        path = item if isinstance(item, str) else item.get("path") if isinstance(item, dict) else None
        valid = isinstance(path, str) and bool(path.strip())
        label = os.path.splitext(os.path.basename(path))[0] if valid else None
        if isinstance(item, dict):
            valid = valid and not (set(item) - {"path", "label"})
            if "label" in item:
                label = item["label"]
                valid = valid and isinstance(label, str) and bool(label.strip())
        if not valid:
            return None, f"first_message.images[{i}] must be a path or an object with path and label"
        if path in seen:
            return None, f"first_message.images names {path} twice"
        images.append((path, label))
        seen.add(path)
    if "state" in value and not isinstance(value["state"], dict):
        return None, "first_message.state must be a JSON object"
    if "system" in value and (not isinstance(value["system"], str) or not value["system"].strip()):
        return None, "first_message.system must be a text that is not empty"
    if "own_tools" in value and not isinstance(value["own_tools"], bool):
        return None, "first_message.own_tools must be true or false"
    return {"text": value["text"], "images": images, "state": value.get("state"),
            "system": value.get("system"), "own_tools": value.get("own_tools", True)}, None


def first_message_text(spec):
    """The surface's text verbatim, then its state on its own line when present."""
    return spec["text"] if spec["state"] is None else (
        spec["text"] + "\nstate: " + json.dumps(spec["state"], ensure_ascii=False))


def first_user_text(task, practices, listing, reply_budget=None, one_call=False):
    parts = [f"Task: {task}"]
    if isinstance(practices, str) and practices.strip():
        parts.append("Practices (from the robot's server):\n" + practices.strip())
    if listing.get("about"):
        parts.append("Conventions (from the robot's server): " + str(listing["about"]).strip())
    total = listing.get("budget_total") or {}
    if reply_budget is not None:
        budget = (f"Budget: you have {reply_budget} replies for the whole episode (each message you send is one "
                  "reply). Every reply counts as one, whatever it calls (a tool, a read or a claim, also a call "
                  "the robot's server refuses or does not charge) and also when it calls nothing. After reply "
                  f"number {reply_budget} the episode ends.")
        limits = []
        if "sim_seconds" in total:
            limits.append(f"{total['sim_seconds']} simulated seconds")
        if "calls" in total and not (one_call and total["calls"] >= reply_budget):
            limits.append(f"{total['calls']} tool calls that its server charges")
        if limits:
            budget += " The robot's own limits: " + " and ".join(limits) + "."
        parts.append(budget)
    elif "calls" in total and "sim_seconds" in total:
        parts.append(f"Budget: {total['calls']} calls and {total['sim_seconds']} simulated seconds.")
    if one_call:
        parts.append(ONE_CALL_RULE)
    return "\n\n".join(parts)


def parse_arguments(raw):
    """(arguments, None) or (None, the JSON parse error). No arguments, an empty text and null mean {}."""
    if raw is None or isinstance(raw, dict):
        return (raw or {}), None
    if not isinstance(raw, str):
        return raw, None
    if not raw.strip():
        return {}, None
    try:
        value = json.loads(raw)
    except ValueError as exc:
        return None, str(exc)
    return ({} if value is None else value), None


def argument_problem(schemas, name, args):
    """None, or why the arguments do not fit the schema's keys: unknown_tool, not_object, unknown_keys,
    missing_keys. Types and ranges are the server's to judge."""
    schema = schemas.get(name)
    if schema is None:
        return "unknown_tool"
    if not isinstance(args, dict):
        return "not_object"
    props = schema.get("properties") or {}
    if schema.get("additionalProperties") is False and set(args) - set(props):
        return "unknown_keys"
    if set(schema.get("required") or []) - set(args):
        return "missing_keys"
    return None


def tool_calls_of(message, turn):
    """The reply's tool calls as [{"id", "type", "function": {"name", "arguments" text}}]; ids filled if absent."""
    calls = message.get("tool_calls") or []
    if not calls and isinstance(message.get("function_call"), dict):
        calls = [{"function": message["function_call"]}]
    out = []
    for i, call in enumerate(c for c in calls if isinstance(c, dict)):
        fn = call.get("function") if isinstance(call.get("function"), dict) else {}
        args = fn.get("arguments")
        if args is not None and not isinstance(args, str):
            args = json.dumps(args)
        item = {"id": str(call.get("id") or f"c{turn:05d}{i:03d}"), "type": "function",
                "function": {"name": str(fn.get("name") or ""), "arguments": "" if args is None else args}}
        # Some endpoints attach an opaque block to a tool call that must be sent back unchanged with the history
        # (a "thought signature": without it the next request is refused with a 400). It is carried, never read.
        if isinstance(call.get("extra_content"), dict):
            item["extra_content"] = call["extra_content"]
        out.append(item)
    return out


def text_of(content):
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        return "".join(str(p.get("text", "")) for p in content if isinstance(p, dict))
    return ""
