# SPDX-License-Identifier: AGPL-3.0-only
"""Brief results and compact printing: what an operator's command prints by default.

An operator may not pipe or redirect a command's output, so every answer must be short enough to read whole. The
session (rcpx/server.py) journals every result whole and answers an operator's request (one that arrives as a text
line on the socket: the command line, the MCP bridge, the chat operator) with brief(result): unchanged when it
prints under its limit; otherwise the drawing aids and diagnostics behind DETAIL_KEYS are left out, long lists of
records are cut to the entries that matter, and then the largest remaining blocks are left out one by one until it
fits. Scalars (stopped, contact, gave, peak_force_n, can_see ...), refusals, frame paths and the protected blocks
(KEEP_BLOCKS) are never left out; the keys that decide the next call come first (FIRST); every cut is named under
"brief" with the free read that returns the whole result (robot://result/SEQ). A call sent with "detail": "full"
is answered whole. In-process callers (the oracle, tests, replays) call Session.dispatch and always get whole
results.

Round four (SPEC-R4 A5). A result that moved or pushed carries bounds: a brief answer holds the one binding bound
and the count of the others ({"binding": ITEM, "more": N}; the whole list is in robot://result). A whole-step
tool's result (it has said and phases or periods) is brief in at most STEP_LIMIT bytes: said first, then bounds,
then periods as one compact list [[phase, periods], ...], periods_used and periods_left; the phase records and
the working blocks (pick, place, evidence, release) give way first, held, placed, arrived and next last.

dumps(obj) is the compact layout the command line prints: a value stays on one line when it fits WIDTH characters,
else its items go one per line (a list of short items is packed, several to a line). It is valid JSON.
page_journal() is robot://journal's paging (newest records as one line each, ?last=N, ?kind=K, ?from=SEQ,
?seq=SEQ for one whole record, ?whole=true).

Pure functions on JSON values: no numpy, no clock, no randomness; deterministic for a given input.
"""
import json

WIDTH = 110            # characters of one printed line before a value is broken into one item per line
LIMIT_LOOK = 1800      # bytes a brief look result prints in (FINDINGS S1: a default look answer under 2 kB)
LIMIT = 2400           # bytes any other brief result prints in (any default result under 3 kB; a result with a look
                       # after it under 2,500, the FINDINGS S1 test)
STEP_LIMIT = 900       # bytes a whole-step tool's brief result prints in, besides pictures (SPEC-R4 A5)
PACK_ITEM = 24         # characters: list items that print this short share a printed line
# SPEC-R4 A5: what a brief result never loses, in whole or in part: the one binding bound and how many more there
# are, and a whole-step tool's periods.
NEVER_CUT = ("bounds", "periods", "periods_used", "periods_left")
# A whole-step result over its limit gives up its working records first (they are whole at detail full and in
# robot://result) and what the operator decides on last: whether it holds, what it placed, where it arrived, what
# to call next.
STEP_FIRST = ("phases", "pick", "place", "evidence", "release")
STEP_LAST = ("held", "placed", "arrived", "next")
SMALL = 80             # a value printing in this many bytes or fewer is never left out
LIST_ROWS = 4          # a list of records longer than this keeps its first LIST_KEEP (or the ones in view)
LIST_KEEP = 3
LEFT_OUT_SHOWN = 10    # names listed under brief.left_out; the rest are counted
# The frames package's drawing aids and other diagnostics: the operator's next call never needs them, the whole
# result (robot://result/SEQ) keeps them.
DETAIL_KEYS = ("axes_px", "ticks_px", "image_to_base", "runner_up", "rays", "per_period", "trace", "samples")
# Blocks whose own keys are cut one by one instead of the block as a whole. next, wait, orientation and tilted
# carry the next decision (the tool to call next, wait_for_light in the dark, the hand's real pose after an
# approach): they are never left out whole (FINDINGS S7, R5; the fix round two information check).
KEEP_BLOCKS = ("session", "look", "contact", "refused", "base_moved", "reach", "next", "wait", "orientation", "tilted",
               # round three (SPEC-R3 C4): what a result says, what was seen and compared, what to try next
               "said", "untried", "stopped", "seen", "part_check", "witness", "felt", "fits", "best_fit",
               "compared_with", "not_recorded")
# Kept as well at the top of a look result (limit_for: LIMIT_LOOK): the view block, what the picture shows in words.
# Not elsewhere: an approach result has its own small view key, and a look carried inside another result gives way
# to that result's limit.
KEEP_BLOCKS_LOOK = ("view",)
# Words a result says in plain text: never left out, at any depth, however long (SPEC-R3 C4).
KEEP_WORDS = ("said", "says")
# Of the removable blocks, those under these keys go last (they decide the next call): their own keys first, the
# whole block only when nothing else is left to cut (marked WHOLE).
LAST_CUT = ("next", "wait", "orientation", "tilted")
WHOLE = "\x00whole"
# Keys shown first, in this order, when present (the rest keep their order after them).
FIRST = ("said", "bounds", "periods", "periods_used", "periods_left", "refused", "stopped", "why", "hint", "warning",
         "next", "wait", "verdict", "accepted", "confirmed",
         "check", "reached", "reachable", "contact", "gave", "moved_part", "reached_force", "can_see", "brightness",
         "look_id", "view", "frame", "annotated", "annotated_axes", "image")
# Inside a record list that says what is in view (the frames package's world_px), the keys a row keeps.
ROW_KEYS = ("id", "label", "kind", "px", "depth_mm", "stale", "confirmed")
WHOLE_URI = "robot://result/{seq}"


# ---------------------------------------------------------------------------------------------------------------
# Printing


def dumps(obj, width=WIDTH, _at=0):
    """JSON text in the compact layout: a value on one line when it fits `width` characters from where it starts,
    else one item per line, indented two spaces per level."""
    flat = json.dumps(obj, separators=(", ", ": "))
    if _at + len(flat) <= width or not isinstance(obj, (dict, list)) or not obj:
        return flat
    pad = " " * (_at + 2)
    if isinstance(obj, dict):
        rows = []
        for k, v in obj.items():
            head = json.dumps(str(k)) + ": "
            rows.append(pad + head + dumps(v, width, _at + 2 + len(head)) if _fits(v, width, _at + 2 + len(head))
                        else pad + head + dumps(v, width, _at + 2))
        return "{\n" + ",\n".join(rows) + "\n" + " " * _at + "}"
    items = [json.dumps(v, separators=(",", ":")) for v in obj]
    if all(len(item) <= PACK_ITEM and "\n" not in item for item in items):
        rows, row = [], pad
        for item in items:
            if row != pad and len(row) + len(item) + 2 > width:
                rows.append(row)
                row = pad
            row += (", " if row != pad else "") + item
        rows.append(row)
        return "[\n" + ",\n".join(rows) + "\n" + " " * _at + "]"
    return "[\n" + ",\n".join(pad + dumps(v, width, _at + 2) for v in obj) + "\n" + " " * _at + "]"


def _fits(v, width, at):
    return at + len(json.dumps(v, separators=(", ", ": "))) <= width or not isinstance(v, (dict, list))


def size(obj):
    """Bytes obj takes printed by dumps."""
    return len(dumps(obj).encode("utf-8"))


# ---------------------------------------------------------------------------------------------------------------
# Brief results


def is_look(name, result):
    """A result that is a look itself (look, standard_look, a package's look-like tool)."""
    return name in ("look", "standard_look") or (isinstance(result, dict) and "look_id" in result
                                                  and "camera_pose" in result)


def is_step(result):
    if not isinstance(result, dict) or "said" not in result:
        return False
    phases, periods = result.get("phases"), result.get("periods")
    return (isinstance(phases, list) and all(isinstance(p, dict) for p in phases)) or \
        (isinstance(periods, list) and all(isinstance(p, list) and len(p) == 2 for p in periods))


def limit_for(name, result):
    if is_step(result):
        return STEP_LIMIT
    return LIMIT_LOOK if is_look(name, result) else LIMIT


def _copy(x):
    return json.loads(json.dumps(x))


def _drop_detail(obj, path, cut):
    if isinstance(obj, dict):
        for k in list(obj):
            here = f"{path}.{k}" if path else str(k)
            if k in NEVER_CUT:
                continue
            if k in DETAIL_KEYS and isinstance(obj[k], (dict, list)):
                del obj[k]
                cut.append(here)
            else:
                _drop_detail(obj[k], here, cut)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            _drop_detail(v, f"{path}.{i}", cut)


def _rows(obj, path, cut):
    """Cut long lists of records: rows that say in_view keep only those in view (with ROW_KEYS), others keep the
    first LIST_KEEP. Lists of numbers or words stay whole."""
    if isinstance(obj, dict):
        for k in list(obj):
            if k in NEVER_CUT:
                continue
            v = obj[k]
            here = f"{path}.{k}" if path else str(k)
            if isinstance(v, list) and len(v) > LIST_ROWS and all(isinstance(r, dict) for r in v):
                if all("in_view" in r for r in v):
                    keep = [{kk: r[kk] for kk in ROW_KEYS if kk in r} for r in v if r.get("in_view")]
                    keep = keep[:LIST_ROWS]
                    cut.append(f"{here} ({len(keep)} of {len(v)} shown: those in view)")
                else:
                    keep = v[:LIST_KEEP]
                    cut.append(f"{here} ({len(keep)} of {len(v)} shown)")
                obj[k] = keep
            else:
                _rows(v, here, cut)


def _blocks(obj, path, out, top=True, keep_top=()):
    """Every removable block under obj: (printed size, path tuple). A block is a dict or list over SMALL bytes, or a
    string over SMALL bytes that is not a FIRST key nor a file path (a long explanation). Numbers, small values,
    frame paths and the keys of a refusal are never removable; a KEEP_BLOCKS block is never removed whole (its own
    keys may be), nor is a keep_top block at the top of the result."""
    if not isinstance(obj, dict):
        return
    for k, v in obj.items():
        if k in NEVER_CUT:
            continue
        here = path + (k,)
        if isinstance(v, str) and len(v) > SMALL and k not in FIRST and k not in KEEP_WORDS and not _is_path(v):
            out.append((size(v), here))   # a long explanation (a *_why) goes before the result runs over
            continue
        if not isinstance(v, (dict, list)) or k == "refused" or k == "supersedes":
            continue
        n = size(v)
        if (k in KEEP_BLOCKS or (top and k in keep_top)) and isinstance(v, dict):
            _blocks(v, here, out, top=False)
            if k in LAST_CUT and n > SMALL:
                out.append((n, here + (WHOLE,)))   # the whole block, once nothing else is left to cut
            continue
        if n > SMALL:
            out.append((n, here))


def _is_path(text):
    """A file path (a frame, a sheet): never left out of a brief result, the operator opens it next."""
    return text.startswith("/") and " " not in text


def _remove(obj, path):
    for k in path[:-1]:
        obj = obj[k]
    del obj[path[-1]]


def _order(result):
    head = [k for k in FIRST if k in result]
    return {**{k: result[k] for k in head}, **{k: v for k, v in result.items() if k not in head}}


def _binding(bounds):
    if isinstance(bounds, dict) and set(bounds) == {"binding", "more"}:
        return bounds
    items = bounds if isinstance(bounds, list) else bounds.get("items") if isinstance(bounds, dict) else None
    if not isinstance(items, list) or not all(isinstance(item, dict) for item in items):
        return bounds
    chosen = bounds.get("binding") if isinstance(bounds, dict) else None
    if not isinstance(chosen, dict):
        chosen = next((item for item in items if item.get("binding") is True), items[0] if items else None)
    return {"binding": {k: v for k, v in chosen.items() if k != "binding"} if chosen is not None else None,
            "more": max(0, len(items) - 1)}


def _shape(result):
    out, cut = result, []
    if "bounds" in result:
        small = _binding(result["bounds"])
        if small is not result["bounds"]:
            out = dict(out, bounds=small)
    phases = result.get("phases")
    if isinstance(phases, list) and all(isinstance(p, dict) and "phase" in p and "periods" in p for p in phases):
        out = dict(out)
        if "periods" not in out:
            out["periods"] = [[p["phase"], p["periods"]] for p in phases]
        del out["phases"]
        cut.append("phases")
    return out, cut


def brief(result, seq=None, name=None, limit=None):
    """The brief form of one tool result (module docstring). Bounds and phases are shaped to their brief form
    (SPEC-R4 A5) also in a result that is under its limit; a result with nothing to shape that prints within its
    limit, a refusal and anything that is not a JSON object come back as they are (the same object)."""
    if not isinstance(result, dict) or "refused" in result:
        return result
    limit = limit_for(name, result) if limit is None else limit
    step = is_step(result)
    shaped, cut = _shape(result)
    if size(shaped) <= limit and not cut:
        return _order(shaped) if shaped is not result else result
    out = _copy(shaped)
    _drop_detail(out, "", cut)
    _rows(out, "", cut)
    whole = (f"robot://result/{seq}" if seq is not None else 'detail "full"') if step else _whole_text(seq)
    note = {"left_out": [], "whole": whole}
    keep_top = KEEP_BLOCKS_LOOK if is_look(name, result) else ()
    while size({**out, "brief": _note(cut, note)}) > limit:
        found = []
        _blocks(out, (), found, keep_top=keep_top)
        if not found:
            break
        found.sort(key=lambda nb: (nb[1][0] in LAST_CUT or (step and nb[1][0] in STEP_LAST), nb[1][-1] == WHOLE,
                                   not (step and nb[1][0] in STEP_FIRST), -nb[0], nb[1]))
        n, path = found[0]
        path = path[:-1] if path[-1] == WHOLE else path
        _remove(out, path)
        cut.append(".".join(str(p) for p in path))
    if step:   # the whole-step limit is the contract: small blocks go too, what the operator decides on last
        rest = [k for k, v in out.items() if isinstance(v, (dict, list)) and k not in NEVER_CUT + KEEP_WORDS]
        rest.sort(key=lambda k: (k in STEP_LAST, STEP_LAST.index(k) if k in STEP_LAST else 0))
        for k in rest:
            if size({**out, "brief": _note(cut, note)}) <= limit:
                break
            del out[k]
            cut.append(k)
    out = _order(out)
    out["brief"] = _note(cut, note)
    return out


def _whole_text(seq):
    if seq is None:
        return 'send the call with "detail": "full" for the whole result'
    return f"read {WHOLE_URI.format(seq=seq)} for the whole result (free; nothing runs again)"


def _note(cut, note):
    shown = cut[:LEFT_OUT_SHOWN]
    more = len(cut) - len(shown)
    return {"left_out": shown + ([f"and {more} more"] if more > 0 else []), "whole": note["whole"]}


# ---------------------------------------------------------------------------------------------------------------
# robot://journal paging

JOURNAL_SHOWN = 20     # records the plain read shows, one line each
JOURNAL_MAX = 50       # most records one read shows
WHOLE_MAX = 5          # most whole records one read shows
SUMMARY_KEYS = ("stopped", "contact", "can_see", "look_id", "accepted", "confirmed", "verdict", "reached")
JOURNAL_HOW = ("robot://journal shows the newest records one line each; add ?last=N (up to 50), ?kind=K (call, "
               "request, response, world_add, verify ...), ?from=SEQ, or ?seq=SEQ for one whole record; "
               "?whole=true shows up to 5 whole records")


def _summary(rec):
    out = {k: rec[k] for k in ("seq", "kind", "req", "name", "call", "t_s", "event", "refused") if k in rec}
    res = rec.get("result")
    if isinstance(res, dict):
        if "refused" in res and isinstance(res["refused"], dict):
            out["refused"] = res["refused"].get("check")
        for k in SUMMARY_KEYS:
            v = res.get(k)
            if isinstance(v, (str, int, float, bool)) or v is None and k in res:
                out[k] = v
            elif isinstance(v, dict) and k == "contact":
                out[k] = True
    req = rec.get("request")
    if isinstance(req, dict):
        out["request"] = {k: req[k] for k in ("op", "name") if k in req}
    entry = rec.get("entry")
    if isinstance(entry, dict):
        out["entry"] = {k: entry[k] for k in ("id", "kind", "label") if k in entry}
    return out


def _int(v, default=None):
    try:
        return int(str(v))
    except (TypeError, ValueError):
        return default


def page_journal(doc, params):
    """robot://journal paged: doc is the journal document ({"count", "records": [...]} or a list of records),
    params the query ({name: value}). Returns a document a read can show whole."""
    records = doc.get("records") if isinstance(doc, dict) else doc
    if not isinstance(records, list):
        return doc
    count = doc.get("count", len(records)) if isinstance(doc, dict) else len(records)
    if "seq" in params:
        want = _int(params["seq"])
        rec = next((r for r in records if isinstance(r, dict) and r.get("seq") == want), None)
        if rec is None:
            seqs = [r.get("seq") for r in records if isinstance(r, dict)]
            return {"refused": {"check": "seq", "why": f"no journal record {params['seq']} among those kept",
                                "hint": f"records kept: seq {min(seqs) if seqs else '-'} to "
                                        f"{max(seqs) if seqs else '-'}; " + JOURNAL_HOW}}
        return {"count": count, "record": rec}
    rows = [r for r in records if isinstance(r, dict)]
    if "kind" in params:
        kinds = set(str(params["kind"]).split(","))
        rows = [r for r in rows if str(r.get("kind")) in kinds]
    if "from" in params:
        first = _int(params["from"], 0)
        rows = [r for r in rows if _int(r.get("seq"), -1) >= first]
    whole = str(params.get("whole", "")).lower() in ("1", "true", "yes")
    cap = WHOLE_MAX if whole else JOURNAL_MAX
    last = max(1, min(cap, _int(params.get("last"), WHOLE_MAX if whole else JOURNAL_SHOWN)))
    picked = rows[-last:]
    return {"count": count, "matching": len(rows), "shown": len(picked),
            "records": picked if whole else [_summary(r) for r in picked], "how": JOURNAL_HOW}


def split_query(uri):
    """(uri without its query, {name: value}) for robot://x?a=1&b=2; the last value of a repeated name wins."""
    if not isinstance(uri, str) or "?" not in uri:
        return uri, {}
    base, _, q = uri.partition("?")
    params = {}
    for part in q.split("&"):
        if not part:
            continue
        k, _, v = part.partition("=")
        params[k.strip()] = v.strip()
    return base, params


# ---------------------------------------------------------------------------------------------------------------
# The practices as they ride on a call answer (rcpx/server.py Session._ride_practices)

PRACTICES_URI = "robot://practices"
PRACTICES_RIDE_B = 1000   # bytes the practices take when they ride on a call answer (the whole text: the free read)
PRACTICES_RULE_CHARS = 72


def practices_brief(text, limit=PRACTICES_RIDE_B):
    """The practices as they ride on a call answer: each rule's opening words (a rule starts a line with its number
    or with "- "; a section heading before its rules), until `limit` bytes, then the free read that gives every rule
    whole with its reasons. A text that fits in `limit` rides whole."""
    if len(str(text).encode("utf-8")) <= limit:
        return text
    rules, cur = [], None
    for line in str(text).splitlines():
        t = line.strip()
        head = t.split(" ", 1)[0]
        if t.startswith("## "):
            rules.append(t[3:].strip() + ":")
            cur = None
        elif (head[:-1].isdigit() and head.endswith(".")) or t.startswith("- "):
            cur = [t[2:] if t.startswith("- ") else t]
            rules.append(cur)
        elif cur is not None and t:
            cur.append(t)
        else:
            cur = None
    tail = (f" ... read {PRACTICES_URI} for every rule whole with its reasons (free; {len(str(text).encode('utf-8'))} "
            "bytes; nothing runs)")
    out = ""
    for r in rules:
        if isinstance(r, list):
            words = " ".join(r)
            end = words.find(". ", 3)
            first = words if end < 0 else words[:end + 1]
            piece = first if len(first) <= PRACTICES_RULE_CHARS else first[:PRACTICES_RULE_CHARS].rsplit(" ", 1)[0] + "..."
        else:
            piece = r
        nxt = (out + " " + piece).strip()
        if len((nxt + tail).encode("utf-8")) > limit:
            break
        out = nxt
    return out + tail
