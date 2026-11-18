# SPDX-License-Identifier: AGPL-3.0-only
"""Argument checking and output filters of the operator surface (rcpx/surfaces.py re-exports every name here).

Schema language: a JSON Schema subset. Checked keywords: type (a name or a list of names), properties, required,
enum, minimum, maximum, plus items, minItems, maxItems, additionalProperties and anyOf (needed for vectors, maps by
joint name and either-or arguments). "description" and "default" are annotations. Keys starting with "x-" are
internal and never shown to the operator; "x-kind": "direction" turns a direction name into a unit vector before the
executor is called and refuses a zero vector.

Round four (SPEC-R4 A5, A6): a schema carries no derived end. A speed, force, travel, time or count argument has its
type and its sign and no range: "x-positive": true refuses a value that is not more than 0. "x-retired": true marks
an argument that is still accepted and no longer listed (guarded): public_schema, the takes line and every hint
leave it out. "x-intent": true marks one of the operating model's words (clear, expect, max_n): the session passes
it to a primitive only when the primitive takes it (rcpx/server_edge.py _fit_call). "x-offered": true marks an
argument that may be left out because RCP offers a value (press travel_mm and force_n): a primitive that still
requires it gets a refusal that names it. minimum and maximum remain for
ranges that are geometry or shares (an angle to turn, an opening as a share, a pixel index).

The constructors a tool table is written with (_num, _pos, _obj, _tool ...) live here; rcpx/surfaces.py re-exports
them, so a package writes surfaces._num.

A bad argument is a refused result whose hint names the field. An unknown argument that is one unambiguous
rename away from a call that fits carries that call under "corrected"; it is offered, never run.

On the way out, scrub_truth drops any key naming ground truth and frame_paths lists every image path in a result.
"""
import difflib
import json
import math
import os

# Direction names, read in the frame the call names: base (x forward, y left, z up) or tool (x along the finger,
# z the hand's up). The server turns a name into this vector before the executor sees it.
DIRECTION_NAMES = {
    "forward": [1.0, 0.0, 0.0],
    "back": [-1.0, 0.0, 0.0],
    "left": [0.0, 1.0, 0.0],
    "right": [0.0, -1.0, 0.0],
    "up": [0.0, 0.0, 1.0],
    "down": [0.0, 0.0, -1.0],
}


# ---------------------------------------------------------------------------------------------------------------
# The constructors a tool table is written with (rcpx/surfaces.py and the packages use them as surfaces._num ...)


def _ranged(kind, lo, hi, text):
    out = {"type": kind}
    if lo is not None:
        out["minimum"] = lo
    if hi is not None:
        out["maximum"] = hi
    out["description"] = text
    return out


def _num(lo=None, hi=None, text=""):
    """A number; lo and hi only for a range that is geometry or a share, never for a limit of the robot."""
    return _ranged("number", lo, hi, text)


def _int(lo=None, hi=None, text=""):
    return _ranged("integer", lo, hi, text)


def _pos(text, kind="number"):
    """A number that must be more than 0 and has no range: the sign is the schema's, the end is the robot's."""
    return {"type": kind, "x-positive": True, "description": text}


def _bool(text):
    return {"type": "boolean", "description": text}


def _str(text, enum=None):
    out = {"type": "string", "description": text}
    if enum is not None:
        out["enum"] = list(enum)
    return out


def _vec3(lo=None, hi=None, text=""):
    item = {"type": "number"}
    if lo is not None:
        item["minimum"] = lo
    if hi is not None:
        item["maximum"] = hi
    return {"type": "array", "items": item, "minItems": 3, "maxItems": 3, "description": text}


def _direction(text):
    return {"anyOf": [{"type": "string", "enum": list(DIRECTION_NAMES)},
                      {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}],
            "x-kind": "direction",
            "description": text + " (a name: forward, back, left, right, up, down; or a vector [x, y, z])"}


def _obj(props, required=()):
    return {"type": "object", "properties": props, "required": list(required), "additionalProperties": False}


def _tool(name, description, args, example_args, means, **extra):
    entry = {"name": name, "description": description, "args": args,
             "example": {"args": example_args, "means": means}}
    entry.update(extra)  # optional "method" (executor method name) and "rename" (surface arg -> executor kwarg)
    return entry


# ---------------------------------------------------------------------------------------------------------------
# Argument checking (I6: a bad argument is a refused result whose hint names the field)


def _fmt(x):
    if isinstance(x, float) and x.is_integer():
        return str(int(x))
    return str(x)


def _show(value):
    text = json.dumps(value) if _jsonish(value) else repr(value)
    return text if len(text) <= 60 else text[:57] + "..."


def _jsonish(value):
    try:
        json.dumps(value)
        return True
    except (TypeError, ValueError):
        return False


def _is_type(value, t):
    if t == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    if t == "integer":
        if isinstance(value, bool):
            return False
        return isinstance(value, int) or (isinstance(value, float) and math.isfinite(value) and value.is_integer())
    if t == "string":
        return isinstance(value, str)
    if t == "boolean":
        return isinstance(value, bool)
    if t == "object":
        return isinstance(value, dict)
    if t == "array":
        return isinstance(value, list)
    if t == "null":
        return value is None
    return False


def _type_fits(t, value):
    return any(_is_type(value, one) for one in (t if isinstance(t, list) else [t]))


def describe(schema):
    """A short English phrase for what a schema accepts, used in hints."""
    if "anyOf" in schema:
        return "either " + ", or ".join(describe(alt) for alt in schema["anyOf"])
    if "enum" in schema:
        return "one of " + ", ".join(json.dumps(e) for e in schema["enum"])
    t = schema.get("type")
    types = t if isinstance(t, list) else [t]
    parts = []
    for one in types:
        if one in ("number", "integer"):
            word = "a number" if one == "number" else "a whole number"
            if schema.get("x-positive"):
                word += " above 0"
            lo, hi = schema.get("minimum"), schema.get("maximum")
            if lo is not None and hi is not None:
                word += f" from {_fmt(lo)} to {_fmt(hi)}"
            elif lo is not None:
                word += f" of at least {_fmt(lo)}"
            elif hi is not None:
                word += f" of at most {_fmt(hi)}"
            parts.append(word)
        elif one == "string":
            parts.append("a text")
        elif one == "boolean":
            parts.append("true or false")
        elif one == "array":
            n_lo, n_hi = schema.get("minItems"), schema.get("maxItems")
            count = f"{n_lo} " if n_lo is not None and n_lo == n_hi else ""
            each = describe(schema["items"]) if "items" in schema else "anything"
            parts.append(f"a list of {count}items, each {each}")
        elif one == "object":
            ap = schema.get("additionalProperties", True)
            if isinstance(ap, dict):
                parts.append(f"an object mapping names to {describe(ap)}")
            else:
                parts.append("an object")
        elif one == "null":
            parts.append("null")
        else:
            parts.append("anything")
    return " or ".join(parts)


def _problem(path, why, schema):
    return {"path": path, "why": why, "schema": schema}


def check(schema, value, path):
    """None when value fits schema, else {"path", "why", "schema"} for the first misfit found."""
    if "anyOf" in schema:
        alts = schema["anyOf"]
        if any(check(alt, value, path) is None for alt in alts):
            return _direction_zero(schema, value, path)
        typed = [alt for alt in alts if "type" in alt and _type_fits(alt["type"], value)]
        if len(typed) == 1:  # the value has the right kind for one branch: report that branch's misfit
            return check(typed[0], value, path)
        return _problem(path, f"{path} must be {describe(schema)}, got {_show(value)}", schema)
    t = schema.get("type")
    if t is not None and not _type_fits(t, value):
        return _problem(path, f"{path} must be {describe(schema)}, got {_show(value)}", schema)
    if "enum" in schema and not any(value == e and type(value) is type(e) for e in schema["enum"]):
        near = difflib.get_close_matches(str(value), [str(e) for e in schema["enum"]], n=1, cutoff=0.5)
        extra = f" (did you mean {json.dumps(near[0])}?)" if near else ""
        return _problem(path, f"{path} must be {describe(schema)}, got {_show(value)}{extra}", schema)
    if _is_type(value, "number"):
        if schema.get("x-positive") and value <= 0:
            return _problem(path, f"{path} must be more than 0, got {_fmt(value)}", schema)
        lo, hi = schema.get("minimum"), schema.get("maximum")
        if lo is not None and value < lo:
            return _problem(path, f"{path} must be at least {_fmt(lo)}, got {_fmt(value)}", schema)
        if hi is not None and value > hi:
            return _problem(path, f"{path} must be at most {_fmt(hi)}, got {_fmt(value)}", schema)
    if isinstance(value, list):
        n_lo, n_hi = schema.get("minItems"), schema.get("maxItems")
        if (n_lo is not None and len(value) < n_lo) or (n_hi is not None and len(value) > n_hi):
            return _problem(path, f"{path} must be {describe(schema)}, got {len(value)} items", schema)
        if "items" in schema:
            for i, item in enumerate(value):
                p = check(schema["items"], item, f"{path}[{i}]")
                if p is not None:
                    return p
    if isinstance(value, dict):
        p = _check_object(schema, value, path)
        if p is not None:
            return p
    return _direction_zero(schema, value, path)


def _direction_zero(schema, value, path):
    if schema.get("x-kind") == "direction" and isinstance(value, list):
        if all(_is_type(x, "number") for x in value) and not any(abs(x) > 0 for x in value):
            return _problem(path, f"{path} must not be the zero vector", schema)
    return None


def _check_object(schema, value, path):
    props = schema.get("properties", {})
    prefix = f"{path}." if path else ""
    for name in schema.get("required", []):
        if name not in value:
            return _problem(prefix + name, f"missing argument {json.dumps(prefix + name)}", props.get(name, {}))
    ap = schema.get("additionalProperties", True)
    known = [k for k in props if k in value]
    unknown = sorted(k for k in value if k not in props)
    for key in known + unknown:
        if key in props:
            p = check(props[key], value[key], prefix + key)
        elif ap is False:
            near = difflib.get_close_matches(key, listed(schema), n=1, cutoff=0.6)
            p = _problem(prefix + key, f"unknown argument {json.dumps(prefix + key)}", {})
            p["near"] = near
        elif isinstance(ap, dict):
            p = check(ap, value[key], prefix + key)
        else:
            p = None
        if p is not None:
            return p
    return None


def _plain(name):
    return "".join(c for c in str(name).lower() if c.isalnum())


def _meant(key, names):
    """The one name in names that key nearly spells, or None. A name that key is the start or the end of (or
    that starts key, or equals it apart from case and separators) counts first: "dir" for direction, "z" for
    dz, "force" for force_n. Failing that, a single close spelling. Two candidates of the same rank is no
    answer."""
    k = _plain(key)
    if not k:
        return None
    strong = [n for n in names if _plain(n) == k or _plain(n).startswith(k) or _plain(n).endswith(k)
              or k.startswith(_plain(n))]
    if strong:
        return strong[0] if len(strong) == 1 else None
    near = difflib.get_close_matches(str(key), list(names), n=3, cutoff=0.6)
    return near[0] if len(near) == 1 else None


def _rename_fix(schema, args):
    """(args with every unknown top-level argument renamed to the one argument it nearly spells, the renames)
    when that leaves a call that fits the schema; else (None, {}). Nothing is dropped and no value is changed:
    only names move, and only to a name the call does not already use (see _meant)."""
    props = schema.get("properties", {})
    unknown = [k for k in args if k not in props]
    if not unknown or schema.get("additionalProperties", True) is not False:
        return None, {}
    renames = {}
    for key in unknown:
        to = _meant(key, [n for n in listed(schema) if n not in args])
        if to is None or to in renames.values():
            return None, {}
        renames[key] = to
    fixed = {renames.get(k, k): v for k, v in args.items()}
    return (fixed, renames) if check(schema, fixed, "") is None else (None, {})


def check_args(tool, args, tools=None):
    """None when args fit the tool's schema, else a refusal body {"check": "args", "field", "why", "hint"}.
    When renaming the unknown arguments to the names they nearly spell makes the call fit, the body adds
    "corrected" {"name", "args"} and the hint spells that call out (it is offered, never run). Given the
    surface's tool entries in tools, an unknown argument with no near name lists the tools that do take it."""
    schema = tool["args"]
    if not isinstance(args, dict):
        return {"check": "args", "field": "args", "why": f"arguments must be a JSON object, got {_show(args)}",
                "hint": f"args must be one JSON object, for example {json.dumps(tool['example']['args'])}"}
    p = check(schema, args, "")
    if p is None:
        return None
    field = p["path"].split(".")[0].split("[")[0]
    names = listed(schema)
    out = {"check": "args", "field": field, "why": p["why"]}
    if "near" in p:
        takes = f"{tool['name']} takes: {', '.join(names)}" if names else f"{tool['name']} takes no arguments"
        near = f"did you mean {json.dumps(p['near'][0])}? " if p["near"] else ""
        hint = f"{field} is not an argument of {tool['name']}; {near}{takes}"
        elsewhere = [t["name"] for t in (tools or ()) if t.get("name") != tool["name"]
                     and field in listed(t.get("args") or {})]
        if elsewhere and not p["near"]:
            hint += f". Tools that take {field}: {', '.join(elsewhere)}"
    else:
        prop = schema.get("properties", {}).get(field, p["schema"])
        meaning = prop.get("description")
        hint = f"{field} is {meaning}; give {describe(prop)}" if meaning else f"{field}: give {describe(prop)}"
        example = tool["example"]["args"]
        if field in example:
            hint += f", for example {json.dumps(example[field])}"
    fixed, renames = _rename_fix(schema, args)
    if fixed is not None:
        out["corrected"] = {"name": tool["name"], "args": fixed}
        said = ", ".join(f"{json.dumps(a)} for {json.dumps(b)}" for a, b in renames.items())
        hint += (f". You wrote {said}. This call fits: call {tool['name']} '{json.dumps(fixed)}' (it was not run: "
                 "send it if it is what you meant)")
    out["hint"] = hint
    return out


def normalize_args(tool, args):
    """(args, notes): what a weaker model or a tool-call parser gets wrong about SHAPE, undone only where the
    meaning cannot change. A tool that takes no arguments of its own ignores stray ones (there is nothing they
    could mean).
    A value that does not fit its schema as sent but does once read as JSON is taken as that JSON: "10" for a
    number, "true" for a boolean, "[0, 0, -1]" for a vector, "\"home\"" for a string. Anything else is left
    for check_args to refuse. notes lists every change so the result can show it."""
    if not isinstance(args, dict):
        return args, []
    props = tool["args"].get("properties", {})
    notes = []
    if not props and args:
        return {}, [f"{tool['name']} takes no arguments; ignored: {', '.join(sorted(map(str, args)))}"]
    # a tool whose only arguments are the operating model's words (clear) or retired ones has none of its own:
    # strays are ignored the same way, and a word that was sent is kept
    words = [n for n, p in props.items() if p.get("x-intent")]
    strays = sorted(str(k) for k in args if k not in props)
    if strays and all(p.get("x-intent") or p.get("x-retired") for p in props.values()):
        args = {k: v for k, v in args.items() if k in props}
        notes.append(f"{tool['name']} takes no arguments" + (f" but {', '.join(words)}" if words else "")
                     + f"; ignored: {', '.join(strays)}")
    out = dict(args)
    for key, value in args.items():
        if key not in props or not isinstance(value, str):
            continue
        fits = check(props[key], value, key) is None
        text = value.strip()
        try:
            parsed = json.loads(text)
        except ValueError:
            continue
        if isinstance(parsed, str) and fits and not (len(text) >= 2 and text[0] == text[-1] == '"'):
            continue
        # the design rulings: a text that holds an object or a list the argument takes is read as that object, also where a
        # text is legal too (a label), and the result says so in one line
        whole = isinstance(parsed, (dict, list))
        if (not fits or whole or isinstance(parsed, str)) and parsed != value and check(props[key], parsed, key) is None:
            out[key] = parsed
            notes.append(f"{key}: came as a text and was read as {'an object' if isinstance(parsed, dict) else 'a list'}"
                         if whole else f"{key}: read the text {_show(value)} as {_show(parsed)}")
    return out, notes


def coerce(tool, args):
    """Turn direction names into unit vectors so the executor always gets [x, y, z] (frame as the call says)."""
    props = tool["args"].get("properties", {})
    out = dict(args)
    for key, value in args.items():
        if props.get(key, {}).get("x-kind") == "direction" and isinstance(value, str):
            out[key] = list(DIRECTION_NAMES[value])
    return out


def listed(schema):
    """Property names an operator is shown, in schema order."""
    return [name for name, prop in schema.get("properties", {}).items() if not prop.get("x-retired")]


def public_schema(schema):
    """The schema as listed: no internal markers or retired properties."""
    if isinstance(schema, dict):
        out = {}
        visible = listed(schema)
        for key, value in schema.items():
            if key.startswith("x-"):
                continue
            if key == "properties":
                value = {name: prop for name, prop in value.items() if name in visible}
            elif key == "required" and "properties" in schema:
                value = [name for name in value if name in visible]
            out[key] = public_schema(value)
        return out
    if isinstance(schema, list):
        return [public_schema(v) for v in schema]
    return schema


# ---------------------------------------------------------------------------------------------------------------
# What leaves the surface: no ground truth, and every frame path listed for the reader

TRUTH_MARKER = "truth"  # a key containing it never reaches the operator or the journal (as rcpx.context does)
IMAGE_EXT = (".png", ".jpg", ".jpeg", ".ppm", ".bmp")


def scrub_truth(obj, path=""):
    """Drop ground-truth keys anywhere in obj; returns (clean, list of removed paths)."""
    removed = []
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            here = f"{path}.{k}" if path else k
            if TRUTH_MARKER in str(k).lower():
                removed.append(here)
                continue
            out[k], sub = scrub_truth(v, here)
            removed.extend(sub)
        return out, removed
    if isinstance(obj, list):
        out = []
        for i, v in enumerate(obj):
            item, sub = scrub_truth(v, f"{path}[{i}]")
            out.append(item)
            removed.extend(sub)
        return out, removed
    return obj, removed


def _is_image_path(x):
    return isinstance(x, str) and x.lower().endswith(IMAGE_EXT) and "\n" not in x


PICTURES_SENT = 1   # frames an answer carries when its result names its own pictures (one picture per answer)


def frame_paths(obj, base_dir=None):
    """Every image path in obj, in order, without repeats; relative paths are resolved against base_dir. A result
    (at any depth) may say "supersedes": {newer key: older key}: when it holds an image under the newer key, the
    image under the older key is left out, so a frame that is another frame with more drawn on it is sent once.
    A result with a top-level "pictures" list says itself which picture shows it (SPEC-R3 C4): its frames are that
    list only, at most PICTURES_SENT of them (an empty list: no frame), whatever other paths the result holds."""
    found = []
    pictures = obj.get("pictures") if isinstance(obj, dict) else None

    def walk(x):
        if isinstance(x, dict):
            sup = x.get("supersedes")
            skip = {old for new, old in sup.items() if _is_image_path(x.get(new))} if isinstance(sup, dict) else set()
            for k, v in x.items():
                if k not in skip:
                    walk(v)
        elif isinstance(x, list):
            for v in x:
                walk(v)
        elif _is_image_path(x):
            p = x if os.path.isabs(x) or base_dir is None else os.path.join(base_dir, x)
            if p not in found:
                found.append(p)

    if isinstance(pictures, list):
        walk([p for p in pictures if _is_image_path(p)][:PICTURES_SENT])
        return found
    walk(obj)
    return found
