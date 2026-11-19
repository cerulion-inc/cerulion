# SPDX-License-Identifier: AGPL-3.0-only
"""The opening a model reads, GENERATED from the session's own tool list.

    opening(listing, P)   one line per tool the listing holds, in its order: the tool's name, what it does (the
                          first sentence of its own card, cut to LINE_CHARS) and what it costs here in control
                          periods (rcpx/price_words.py tool_price over the session's prices P); then the resources.
                          It holds no ordered procedure: no step is first, none is recommended, and the body check
                          and the survey stand only with their price and what they buy.
    with_opening(listing, P)   the full listing whose "about" starts with the opening (once: a listing that already
                          holds it is returned as it is)
    conventions(about)    the "about" text without the opening (the compact index lists the tools itself)

Pure: text in, text out; it names no robot, task or benchmark and reads no scene.
"""

OPENING_HEAD = "What this robot has in this session (one line a tool: what it does; what it costs here)."
READ_HEAD = "Resources (read is free): "
LINE_CHARS = 90           # the characters of a tool's line before its price (a layout width, no bound)
END = "\n\n"


CLAUSE_ENDS = ("; ", ": ", ", ", " (")   # where a long first sentence may be cut (never inside a bracket)


def _first(text):
    """The first sentence of a card, cut to LINE_CHARS at the last clause end that fits outside every bracket (a
    third of the width in at least), else at a word end with "..."."""
    from rcpx.surface_list import first_sentence
    text = " ".join(first_sentence(text).split()).rstrip(".")
    if len(text) <= LINE_CHARS:
        return text
    head, best = text[:LINE_CHARS + 1], -1
    for mark in CLAUSE_ENDS:
        at = head.rfind(mark)
        while at > 0 and head[:at].count("(") != head[:at].count(")"):
            at = head.rfind(mark, 0, at)
        best = max(best, at)
    if best >= LINE_CHARS // 3:
        return head[:best]
    head = text[:LINE_CHARS - 3]
    return head[:head.rindex(" ")].rstrip(" ,;:") + "..." if " " in head else head + "..."


def opening(listing, P=None):
    """The opening text (see the module docstring)."""
    from rcpx import price_words
    lines = [OPENING_HEAD]
    for t in (listing or {}).get("tools") or []:
        if not isinstance(t, dict) or not t.get("name"):
            continue
        lines.append(f"- {t['name']}: {_first(t.get('description', ''))}; {price_words.tool_price(t['name'], t, P)}.")
    uris = [r.get("uri") for r in (listing or {}).get("resources") or []
            if isinstance(r, dict) and isinstance(r.get("uri"), str) and "{" not in r["uri"]]
    if uris:
        lines.append(READ_HEAD + ", ".join(uris) + ".")
    return "\n".join(lines)


def is_full_listing(obj):
    """A session's full list answer (rcpx/server.py Session._list: its tools with their schemas, the claim, the
    resources as a list), not a surface table (resources as a map) nor a compact index (no schemas)."""
    tools = obj.get("tools") if isinstance(obj, dict) else None
    return (isinstance(tools, list) and "about" in obj and "claim" in obj and isinstance(obj.get("resources"), list)
            and bool(tools) and all(isinstance(t, dict) and "args" in t for t in tools))


PLAIN_SURFACES = ("v0",)   # the plain-manifest baseline (rcpx/surfaces.py V0): a control arm, its texts unchanged


def with_opening(listing, P=None):
    about = str(listing.get("about") or "")
    if about.startswith(OPENING_HEAD) or listing.get("surface") in PLAIN_SURFACES:
        return listing
    return {**listing, "about": opening(listing, P) + (END + about if about else "")}


def conventions(about):
    about = str(about or "")
    if about.startswith(OPENING_HEAD):
        return about.split(END, 1)[1] if END in about else ""
    return about


__all__ = ["OPENING_HEAD", "conventions", "is_full_listing", "opening", "with_opening"]
