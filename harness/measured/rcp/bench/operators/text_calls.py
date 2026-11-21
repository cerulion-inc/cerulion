# SPDX-License-Identifier: AGPL-3.0-only
"""Tool calls a model wrote as TEXT in its reply instead of a tool call (bench/operators/chat_operator.py).

Some models answer a tool-calling request with the call written out in the reply and then go on to write the tool's
answer themselves. find_text_call(text, known) finds the FIRST call written in one of the common forms:

- bracket (pythonic): [name(arg=value, other=value)], several calls in one bracket allowed; values are Python or
  JSON literals (true, false and null read as JSON); only a name the tool list knows counts, so prose in brackets
  is never taken for a call;
- JSON: an object {"name": N, "arguments": {...}} ("parameters" or "args" in place of "arguments"; arguments may be
  a JSON text), alone, in a fenced block or inside a tag;
- tagged: <tool_call>...</tool_call> (or <function_call>, <tools>) holding that JSON object, or the XML form
  <function=name><parameter=key>value</parameter></function>.

It returns None, or {"form", "calls": [(name, arguments)], "start", "end", "before", "after"}: "before" is the text
up to the call (an opening tag or fence just before it dropped) and "after" everything past the end of the call's
block (its closing tag or fence included in the block). The operator runs the calls and drops "after": text after
a call written as text is never a tool result. Pure: no clock, no I/O.
"""
import ast
import json
import re

FORMS = ("bracket", "json", "tagged")
_BRACKET = re.compile(r"\[\s*([A-Za-z_][A-Za-z0-9_]*)\s*\(")
_XML_FN = re.compile(r"<function=([A-Za-z_][\w.-]*)>(.*?)</function>", re.DOTALL)
_XML_PARAM = re.compile(r"<parameter=([A-Za-z_][\w.-]*)>(.*?)</parameter>", re.DOTALL)
_CLOSERS = re.compile(r"\s*(?:</(?:tool_call|function_call|tools|tool|function)>|```)")
_OPENER_TAIL = re.compile(r"(?:<(?:tool_call|function_call|tools|tool)>|```[A-Za-z_]*|\[TOOL_CALLS\]|<\|python_tag\|>)"
                          r"\s*$")
_ARG_KEYS = ("arguments", "parameters", "args")
_JSON_NAMES = {"true": True, "false": False, "null": None}


def _closing_bracket(text, start):
    """The index just past the ']' that closes the '[' at start, quotes respected; None when it never closes."""
    depth, quote, i = 0, None, start
    while i < len(text):
        c = text[i]
        if quote:
            if c == "\\":
                i += 2
                continue
            if c == quote:
                quote = None
        elif c in "'\"":
            quote = c
        elif c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
            if depth == 0:
                return i + 1 if c == "]" else None
        i += 1
    return None


def _literal(node):
    if isinstance(node, ast.Name) and node.id in _JSON_NAMES:
        return _JSON_NAMES[node.id]
    if isinstance(node, (ast.List, ast.Tuple)):
        return [_literal(e) for e in node.elts]
    if isinstance(node, ast.Dict):
        return {_literal(k): _literal(v) for k, v in zip(node.keys, node.values)}
    return ast.literal_eval(node)


def _bracket_calls(src, known, positional):
    """[(name, args)] of one bracket text, or None when it is not a list of calls to known tools."""
    try:
        tree = ast.parse(src, mode="eval").body
    except (SyntaxError, ValueError):
        return None
    if not isinstance(tree, ast.List) or not tree.elts:
        return None
    calls = []
    for el in tree.elts:
        if not (isinstance(el, ast.Call) and isinstance(el.func, ast.Name) and el.func.id in known):
            return None
        try:
            args = {kw.arg: _literal(kw.value) for kw in el.keywords if kw.arg}
            pos = [_literal(a) for a in el.args]
        except (ValueError, TypeError, SyntaxError):
            return None
        names = [n for n in positional(el.func.id) if n not in args]
        if len(pos) > len(names):
            return None
        args.update(zip(names, pos))
        calls.append((el.func.id, args))
    return calls


def _find_bracket(text, known, positional):
    for m in _BRACKET.finditer(text):
        if m.group(1) not in known:
            continue
        end = _closing_bracket(text, m.start())
        if end is None:
            continue
        calls = _bracket_calls(text[m.start():end], known, positional)
        if calls:
            return {"form": "bracket", "calls": calls, "start": m.start(), "end": end}
    return None


def _json_call(obj):
    if isinstance(obj, dict) and isinstance(obj.get("function"), dict):
        obj = obj["function"]
    if not isinstance(obj, dict) or not isinstance(obj.get("name"), str) or not obj["name"].strip():
        return None
    key = next((k for k in _ARG_KEYS if k in obj), None)
    if key is None:
        return None
    args = obj[key]
    if isinstance(args, str):
        try:
            args = json.loads(args) if args.strip() else {}
        except ValueError:
            return None
    if args is None:
        args = {}
    return (obj["name"].strip(), args) if isinstance(args, dict) else None


def _find_json(text):
    dec = json.JSONDecoder()
    i = text.find("{")
    while i != -1:
        try:
            obj, end = dec.raw_decode(text, i)
        except ValueError:
            i = text.find("{", i + 1)
            continue
        call = _json_call(obj)
        if call is not None:
            tagged = bool(re.search(r"<(?:tool_call|function_call|tools|tool)>\s*$", text[:i]))
            return {"form": "tagged" if tagged else "json", "calls": [call], "start": i, "end": end}
        i = text.find("{", end)
    return None


def _xml_value(raw):
    v = raw.strip()
    try:
        return json.loads(v)
    except ValueError:
        return v


def _find_xml(text):
    m = _XML_FN.search(text)
    if m is None:
        return None
    args = {p.group(1): _xml_value(p.group(2)) for p in _XML_PARAM.finditer(m.group(2))}
    return {"form": "tagged", "calls": [(m.group(1), args)], "start": m.start(), "end": m.end()}


def find_text_call(text, known, positional=None):
    """The first call written as text (module docstring), or None. known: the tool names a bracket call may use;
    positional(name) -> the argument names positional values fill, in order (default: none)."""
    if not isinstance(text, str) or not text.strip():
        return None
    positional = positional or (lambda name: [])
    found = [f for f in (_find_bracket(text, set(known), positional), _find_json(text), _find_xml(text)) if f]
    if not found:
        return None
    best = min(found, key=lambda f: (f["start"], FORMS.index(f["form"])))
    end = best["end"]
    closer = _CLOSERS.match(text, end)
    while closer and closer.end() > end:
        end = closer.end()
        closer = _CLOSERS.match(text, end)
    before = _OPENER_TAIL.sub("", text[:best["start"]]).rstrip()
    return {**best, "end": end, "before": before, "after": text[end:]}
