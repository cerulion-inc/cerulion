# SPDX-License-Identifier: AGPL-3.0-only
"""The list answers of the operator surface (rcpx/surfaces.py re-exports the names here).

Three answers come from one full listing (what Session._list builds: every tool with its description, argument
schema and worked example, the claim, the resources, the budget):

- compact(listing, briefs): the index an operator reads first. Per tool its name, one line, and its argument names
  with types (* marks a required one); the claim form; each resource with one line; the budget; how to get more.
  No nested schemas, no examples, no practices: it is sized to be shown whole (under 12,000 bytes printed).
- named(listing, names): the full entries of the tools asked for by name ("claim" names the claim form).
- the full listing itself, for a client that builds tool definitions from it in one request.
"""
import difflib
import json

from rcpx.surface_args import DIRECTION_NAMES

COMPACT_BYTES = 11800  # the compact answer, printed by rcpx.cli inside its response, stays under this (with the
                       # envelope allowance below, under the 12,000 the index was built to print in)
ENVELOPE_BYTES = 450   # allowance for the response around the result (seq, op, the budget block)
# How the index is shortened when a surface grows past COMPACT_BYTES: (characters of a tool's one line, characters
# of a resource's one line, show the types of optional arguments). The first level that fits is used; the last
# (0 characters) lists the tools by name alone.
FIT_LEVELS = ((120, 110, True), (90, 110, True), (90, 80, False), (90, 40, False), (60, 60, False), (40, 40, False),
              (0, 40, False))   # (90, 40): resource lines give way before a tool's line does (fix2: 37 tools)
BRIEF_MAX = FIT_LEVELS[0][0]
ENUM_MAX = 60          # an enum longer than this, written out, is shown as its type instead
DIRECTIONS = tuple(DIRECTION_NAMES)  # forward, back, left, right, up, down: shown as the type "direction"
MORE = ('list NAME (socket: {"op": "list", "name": "NAME"}) returns that tool\'s full description, argument schema '
        'and a worked example; several names may be given, and "claim" is one of them. {"op": "list", "full": true} '
        "returns every tool's at once. In takes, * marks a required argument.")
MORE_DIRECTION = " A direction is one of " + "|".join(DIRECTIONS) + " or a vector [x, y, z]."
SHORTENED = " This index was shortened to fit one screen: list NAME has each tool's full text."
CLAIM_HOW = ("claim is its own verb, not a tool: rcpx.cli claim '{...}' on the command line, or {\"op\": \"claim\", "
             "\"args\": {...}} on the socket")


def cut(text, limit):
    """text if it fits in limit characters; else its start up to the last clause end (a semicolon or colon past
    the first third) that fits, or failing that up to a word end followed by "..."."""
    text = " ".join(str(text or "").split())
    if len(text) <= limit:
        return text
    head = text[:limit + 1]
    clause = max(head.rfind("; "), head.rfind(": "))
    if clause >= limit // 3:
        return head[:clause]
    head = text[:limit - 3]
    if " " in head[limit // 2:]:
        head = head[:head.rindex(" ")]
    return head.rstrip(" ,;:.") + "..."


def first_sentence(text):
    """The text up to its first sentence end (the summary a long description starts with)."""
    text = " ".join(str(text or "").split())
    end = text.find(". ")
    return text[:end + 1] if end > 0 else text


def brief(entry, limit=BRIEF_MAX):
    """A tool's one line: its own "brief" when the table gives one, else the first sentence of its description,
    cut to fit."""
    return cut(entry.get("brief") or first_sentence(entry.get("description", "")), limit)


def type_text(schema):
    """A schema as a short type: number, integer, text, true|false, a|b|c for an enum, [x, y, z] for three
    numbers, {name: number} for a map, {a*, b} for an object with named parts, "X or Y" for either-or."""
    if not isinstance(schema, dict):
        return "any"
    if "anyOf" in schema:
        alts = schema["anyOf"]
        if len(alts) == 2 and tuple(alts[0].get("enum") or ()) == DIRECTIONS and type_text(alts[1]) == "[x, y, z]":
            return "direction"
        seen = []
        for alt in schema["anyOf"]:
            t = type_text(alt)
            if t not in seen:
                seen.append(t)
        return " or ".join(seen)
    if "enum" in schema:
        text = "|".join(str(e) for e in schema["enum"])
        if len(text) <= ENUM_MAX:
            return text
    t = schema.get("type")
    if isinstance(t, list):
        return " or ".join(type_text({**schema, "type": one}) for one in t)
    if t == "boolean":
        return "true|false"
    if t == "string":
        return "text"
    if t in ("number", "integer"):
        return t
    if t == "array":
        item = type_text(schema.get("items", {})) if "items" in schema else "any"
        n = schema.get("minItems")
        if n is not None and n == schema.get("maxItems"):
            if item in ("number", "integer"):
                return "[x, y, z]" if n == 3 else "[u, v]" if n == 2 else f"[{n} {item}s]"
            return f"[{n} of {item}]"
        return f"[{item}, ...]"
    if t == "object":
        props = schema.get("properties") or {}
        if props:
            req = set(schema.get("required", []))
            return "{" + ", ".join(k + ("*" if k in req else "") for k in props) + "}"
        ap = schema.get("additionalProperties")
        return "{name: " + type_text(ap) + "}" if isinstance(ap, dict) else "object"
    return "any"


def takes(args_schema, optional_types=True):
    """A tool's arguments on one line: "name*: type; name: type", the required ones first and marked *;
    "nothing" for a tool that takes none. With optional_types false the optional arguments are names only."""
    props = (args_schema or {}).get("properties") or {}
    if not props:
        return "nothing"
    req = [k for k in (args_schema.get("required") or []) if k in props]
    parts = [f"{k}*: {type_text(props[k])}" for k in req]
    rest = [k for k in props if k not in req]
    parts += [f"{k}: {type_text(props[k])}" for k in rest] if optional_types else ([", ".join(rest)] if rest else [])
    return "; ".join(parts)


def printed_bytes(result):
    """How many bytes a list result takes once rcpx.cli prints its response (the compact layout of
    rcpx/surface_brief.py dumps; it printed with indent 2 before)."""
    from rcpx.surface_brief import dumps
    return len(dumps({"seq": 1, "op": "list", "result": result}).encode("utf-8")) + ENVELOPE_BYTES


def _index(listing, briefs, brief_max, resource_max, optional_types):
    if brief_max:
        tools = [{"name": t["name"],
                  "description": cut(briefs.get(t["name"]) or first_sentence(t.get("description", "")), brief_max),
                  "takes": takes(t.get("args"), optional_types)} for t in listing.get("tools", [])]
    else:  # too many tools for one line each: names alone, and list NAME for the rest
        tools = [{"name": t["name"]} for t in listing.get("tools", [])]
    claim = listing.get("claim") or {}
    directions = any("direction" in takes(t.get("args")) for t in listing.get("tools", []))
    more = MORE + (MORE_DIRECTION if directions else "")   # only a surface whose tools take a direction says it
    from rcpx.surface_words import conventions   # the compact index lists the tools itself: no opening in it
    out = {"surface": listing.get("surface"), "about": conventions(listing.get("about", "")), "more": more,
           "tools": tools,
           "claim": {"description": cut(first_sentence(claim.get("description", "")), brief_max or BRIEF_MAX),
                     "takes": takes(claim.get("args")), "how": CLAIM_HOW},
           "resources": [{"uri": r["uri"], "description": cut(r.get("description", ""), resource_max)}
                         for r in listing.get("resources", [])]}
    if "budget_total" in listing:
        out["budget_total"] = listing["budget_total"]
    return out


def compact(listing, briefs=None, limit=COMPACT_BYTES):
    """The compact index of a full listing (see the module docstring). briefs: {tool name: one line} from the
    surface table; a tool not in it gets the first sentence of its description. The index is built at the first
    of FIT_LEVELS whose printed size is within limit, so a surface with many tools still fits one screen; a
    shortened index says so."""
    briefs = briefs or {}
    out = None
    for level, (brief_max, resource_max, optional_types) in enumerate(FIT_LEVELS):
        out = _index(listing, briefs, brief_max, resource_max, optional_types)
        if level:
            out["more"] += SHORTENED
        if printed_bytes(out) <= limit:
            break
    return out


def named(listing, names):
    """The full entries of the named tools, in the order asked; "claim" names the claim form. A name the listing
    does not hold gives a refusal body {"check": "unknown_tool", ...} under "refused"."""
    by_name = {t["name"]: t for t in listing.get("tools", [])}
    out = {"surface": listing.get("surface"), "tools": []}
    for name in names:
        if name == "claim" and "claim" not in by_name and listing.get("claim"):
            out["claim"] = {**listing["claim"], "how": CLAIM_HOW}
        elif isinstance(name, str) and name in by_name:
            if by_name[name] not in out["tools"]:
                out["tools"].append(by_name[name])
        else:
            known = list(by_name) + ["claim"]
            near = difflib.get_close_matches(str(name), known, n=5, cutoff=0.5)
            shown = json.dumps(name) if isinstance(name, (str, int, float)) else repr(name)
            return {"refused": {"check": "unknown_tool", "why": f"no tool named {shown[:60]}",
                                "hint": ("did you mean: " + ", ".join(near) + "? " if near else "")
                                + "list with no name shows every tool", "near": near or sorted(known)}}
    return out
