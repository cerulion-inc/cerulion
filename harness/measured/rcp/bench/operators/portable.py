# SPDX-License-Identifier: AGPL-3.0-only
"""Portability of the tool surface to strict chat endpoints: the plain schema mode (the chat operator and the MCP
bridge, rcpx/mcp_stdio.py) and the two kinds of HTTP 400 the chat operator answers without counting a failure.

plain_schema(schema) is a copy of an argument schema with the JSON schema keywords a strict endpoint may refuse
taken out (STRIPPED: minimum, maximum, exclusiveMinimum, exclusiveMaximum, minItems, maxItems, minLength, maxLength,
pattern, format, default, examples) and said in words in the description of the argument they belong to, so the
model still sees every bound: "how far, in mm (from 0 to 300)". A bound inside a list's items or an object's values
is said on the argument as "each item ..." or "each value ...". An anyOf or oneOf with a plain branch (one scalar
type: string, number, integer or boolean) is flattened to its FIRST branch (the surface's own first form), and every
other branch is said in words ("also accepted: a list of exactly 3 numbers, each from -1000 to 1000"); a union with
no plain branch stays, with the bounds of each branch said in that branch's description. type, enum, properties,
required, items, additionalProperties and description are kept. The episode server still checks every call against
its own full schema, so a bound said in words is still enforced.

schema_refusal(text) says whether a 400's error text is about the tool schemas; unknown_body_fields(text, body)
returns the top-level body keys it names as unknown or unsupported (never model, messages or tools) and any rename
it asks for ("use 'max_completion_tokens' instead").

Pure functions of their inputs: no clock, no I/O.
"""
import copy
import json
import re

STRIPPED = ("minimum", "maximum", "exclusiveMinimum", "exclusiveMaximum", "minItems", "maxItems", "minLength",
            "maxLength", "pattern", "format", "default", "examples")
UNIONS = ("anyOf", "oneOf")
SCALARS = ("string", "number", "integer", "boolean")
PROTECTED_BODY_KEYS = ("model", "messages", "tools")   # never dropped: without them there is no request

_UNKNOWN = re.compile(r"unknown|unrecogni[sz]ed|not permitted|not allowed|unsupported|not supported|"
                      r"does not support|doesn't support|"
                      r"extra (?:inputs?|fields?|keys?|parameters?)|additional propert|unexpected (?:field|keyword|"
                      r"parameter|argument|key)|invalid (?:field|parameter|argument|key)|no such (?:field|parameter)",
                      re.IGNORECASE)
_RENAME = re.compile(r"use\s+['\"`]?([A-Za-z_][A-Za-z0-9_]*)['\"`]?\s+instead", re.IGNORECASE)
_SCHEMA_WORD = re.compile(r"schema|\b(?:exclusiveMinimum|exclusiveMaximum|minItems|maxItems|minLength|maxLength|"
                          r"anyOf|oneOf|allOf)\b", re.IGNORECASE)
_BOUND_WORD = re.compile(r"\b(?:minimum|maximum|pattern|format|default|examples)\b", re.IGNORECASE)
_SCHEMA_CONTEXT = re.compile(r"\b(?:tools?|function|parameters|properties|keyword|types?)\b|not (?:supported|"
                             r"permitted|allowed)|do(?:es)? not support", re.IGNORECASE)


# ---------------------------------------------------------------------------------------------------------------
# Words for a schema


def _num(x):
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        return json.dumps(x)
    return str(int(x)) if float(x) == int(x) else repr(float(x))


def _range(node):
    """'from A to B', 'at least A', 'more than A and at most B', ... for the numeric bounds of one node."""
    lo, hi = node.get("minimum"), node.get("maximum")
    xlo, xhi = node.get("exclusiveMinimum"), node.get("exclusiveMaximum")
    lo_open = xlo is True                      # draft 4: a boolean makes minimum itself exclusive
    hi_open = xhi is True
    if isinstance(xlo, (int, float)) and not isinstance(xlo, bool):
        lo, lo_open = xlo, True
    if isinstance(xhi, (int, float)) and not isinstance(xhi, bool):
        hi, hi_open = xhi, True
    if lo is not None and hi is not None and not lo_open and not hi_open:
        return f"from {_num(lo)} to {_num(hi)}"
    bits = []
    if lo is not None:
        bits.append(("more than " if lo_open else "at least ") + _num(lo))
    if hi is not None:
        bits.append(("less than " if hi_open else "at most ") + _num(hi))
    return " and ".join(bits)


def _count(node, lo_key, hi_key, unit):
    lo, hi = node.get(lo_key), node.get(hi_key)
    if lo is None and hi is None:
        return ""
    if lo is not None and lo == hi:
        return f"exactly {_num(lo)} {unit}"
    if lo is not None and hi is not None:
        return f"from {_num(lo)} to {_num(hi)} {unit}"
    return (f"at least {_num(lo)} {unit}" if lo is not None else f"at most {_num(hi)} {unit}")


def bound_words(node):
    """The phrases for the STRIPPED keywords of one schema node (not its children), in a fixed order."""
    out = []
    rng = _range(node)
    if rng:
        out.append(rng)
    for lo, hi, unit in (("minItems", "maxItems", "items"), ("minLength", "maxLength", "characters")):
        text = _count(node, lo, hi, unit)
        if text:
            out.append(text)
    if "pattern" in node:
        out.append(f"matching the pattern {node['pattern']}")
    if "format" in node:
        out.append(f"in the format {node['format']}")
    if "default" in node:
        out.append(f"default {json.dumps(node['default'])}")
    if isinstance(node.get("examples"), list) and node["examples"]:
        out.append("for example " + " or ".join(json.dumps(x) for x in node["examples"][:3]))
    return out


_TYPE_WORDS = {"string": "text", "number": "a number", "integer": "a whole number", "boolean": "true or false",
               "null": "null"}


def describe(node):
    """A schema in plain words (used for the branches a flattened union leaves out)."""
    if not isinstance(node, dict):
        return "any value"
    for key in UNIONS:
        if isinstance(node.get(key), list):
            return " or ".join(describe(b) for b in node[key])
    words = bound_words(node)
    t = node.get("type")
    if isinstance(node.get("enum"), list):
        head = "one of " + ", ".join(v if isinstance(v, str) else json.dumps(v) for v in node["enum"])
    elif t == "array":
        count = _count(node, "minItems", "maxItems", "")
        item = describe(node.get("items")) if isinstance(node.get("items"), dict) else "values"
        head = "a list of " + (count.strip() + " " if count else "") + _plural(item)
        words = [w for w in words if not w.endswith(" items")]
    elif t == "object":
        extra = node.get("additionalProperties")
        head = "an object mapping names to " + _plural(describe(extra)) if isinstance(extra, dict) else "an object"
    else:
        head = _TYPE_WORDS.get(t, str(t) if t else "any value")
    return head + ("" if not words else (", " if t == "array" else " ") + ", ".join(words))


def _plural(text):
    """'a number from 0 to 1' -> 'numbers, each from 0 to 1'; other texts are kept."""
    for one, many in (("a number", "numbers"), ("a whole number", "whole numbers"), ("text", "texts")):
        if text == one:
            return many
        if text.startswith(one + " "):
            return many + ", each " + text[len(one) + 1:]
    return text


# ---------------------------------------------------------------------------------------------------------------
# The plain schema


def _is_plain(branch):
    return isinstance(branch, dict) and branch.get("type") in SCALARS and not any(k in branch for k in UNIONS)


def _say(node, notes):
    if notes:
        desc = str(node.get("description") or "").strip()
        node["description"] = (desc + " " if desc else "") + "(" + "; ".join(notes) + ")"
    return node


def _plain(node):
    """(plain copy of node, phrases to say on the nearest named argument). node is not changed."""
    if not isinstance(node, dict):
        return node, []
    node = dict(node)
    notes, kept = [], set()
    while True:   # a kept first branch may bring a union of its own
        key = next((k for k in UNIONS if k not in kept and isinstance(node.get(k), list) and node[k]), None)
        if key is None:
            break
        branches = node[key]
        if any(_is_plain(b) for b in branches):
            node.pop(key)
            first = branches[0] if isinstance(branches[0], dict) else {}
            node = {**first, **node}            # the union's own description wins over the branch's
            others = [describe(b) for b in branches[1:]]
            if others:
                notes.append("also accepted: " + "; or ".join(others))
        else:
            node[key] = [_say(*_plain(b)) for b in branches]
            kept.add(key)
    own = bound_words(node)
    for k in STRIPPED:
        node.pop(k, None)
    lifted = []
    if isinstance(node.get("properties"), dict):
        node["properties"] = {name: _say(*_plain(p)) for name, p in node["properties"].items()}
    if isinstance(node.get("items"), dict):
        node["items"], sub = _plain(node["items"])
        lifted += ["each item " + s for s in sub]
    if isinstance(node.get("additionalProperties"), dict):
        node["additionalProperties"], sub = _plain(node["additionalProperties"])
        lifted += ["each value " + s for s in sub]
    return node, own + notes + lifted


def plain_schema(schema):
    """The plain copy of an argument schema (module docstring). The input is never changed."""
    node, notes = _plain(copy.deepcopy(schema))
    return _say(node, notes) if isinstance(node, dict) else node


def stripped_keywords(schema):
    """Every STRIPPED keyword, and anyOf or oneOf, found anywhere in a schema (for tests and the journal)."""
    found = set()

    def walk(x):
        if isinstance(x, dict):
            for k, v in x.items():
                if k in STRIPPED or k in UNIONS:
                    found.add(k)
                if k == "properties" and isinstance(v, dict):
                    for p in v.values():
                        walk(p)
                elif k not in ("enum", "required", "description", "default", "examples"):
                    walk(v)
        elif isinstance(x, list):
            for v in x:
                walk(v)
    walk(schema)
    return sorted(found)


# ---------------------------------------------------------------------------------------------------------------
# HTTP 400s from strict endpoints


def schema_refusal(text):
    """True when a 400's error text is about the tool schemas ("Invalid JSON schema - number types do not support
    minimum", "'minItems' is not permitted"). An over-long context ("maximum context length") is not."""
    t = str(text or "")
    return bool(_SCHEMA_WORD.search(t) or (_BOUND_WORD.search(t) and _SCHEMA_CONTEXT.search(t)
                                           and "context length" not in t.lower()))


def unknown_body_fields(text, body):
    """(keys to drop, {old: new} renames): the top-level body keys a 400's error text names as unknown, extra or
    unsupported. Only keys the body has, never PROTECTED_BODY_KEYS; nothing when the text names no such word."""
    t = str(text or "")
    if not _UNKNOWN.search(t):
        return [], {}
    named = [k for k in body if k not in PROTECTED_BODY_KEYS
             and re.search(r"(?<![A-Za-z0-9_])" + re.escape(k) + r"(?![A-Za-z0-9_])", t)]
    renames = {}
    m = _RENAME.search(t)
    if len(named) == 1 and m and m.group(1) not in body and m.group(1) != named[0]:
        renames[named[0]] = m.group(1)
    return [k for k in named if k not in renames], renames
