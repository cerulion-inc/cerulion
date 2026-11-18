# SPDX-License-Identifier: AGPL-3.0-only
"""The words of push and lift: the said sentence of each stop, the next call, and the
brief result. Pure: dicts in, dicts and text out.

    said(stopped, **f)             the one sentence that leads the result (at most SAID_MAX_WORDS words)
    lift_said(stopped, **f)        lift's sentences
    next_for(stopped, ctx)         a ready call, a point call, a fork or None (only loaded tools, never stale ids)
    brief(out)                     the result in its brief form: the envelope keys in order, under BRIEF_MAX_BYTES
"""

import json

import numpy as np

from rcpx import r3_limits as L
from rcpx import sheet_words, side_words
from rcpx.context import refusal
from rcpx.r2 import act_ladder
from rcpx.r2 import act_engine as E
from rcpx.r2 import act_guard as G

SAID = {
    "changed": "pushed the {label} {dir} at {n} N: it gave after {mm} mm; same view before and after: {seen}",
    "reached_end": "pushed the {label} {total} mm {dir} until it stopped at {n} N; same view before and after: {seen}",
    "ceiling": "pushed the {label} up to {C} N, {why}: it moved {mm} mm and came back; "
               "{seen}",
    "rigid": "the {label} did not move at {n} N: this spot is not the moving part, or the part stands at a stop or a stiff point",
    "not_square": "the face at the {label} faces {dir}, {deg} deg off the push: no push made",
    "blocked": "touched {what} at {n} N, {mm} mm before the point given for the {label}, and stopped there: it "
               "may be the {label} itself or something before it; nothing was pushed",
    "not_found": "felt nothing along the line to the {label} for {mm} mm: the point may be off; no push made",
    "cannot_reach": "the arm cannot bring a finger onto the {label} along {dir}: {why}",
    "time": "stopped after {s} s of work, at {stage}: {what}",
    "dark": "the light went dark during the push: the finger backed off and stopped",
    "refused": "a safety check stopped the push at {stage}: {check}",
    "cap": "the force reached the arm's cap at {n} N",
    "base_moved": "the robot's base moved under the push",
    "unconfirmed": "pushed the {label} {dir} at {n} N: it moved {mm} mm and {how}, but no picture shows a change, "
                   "so this is not known to have worked; {seen}",
    "stalled": "pushed the {label} {total} mm {dir}; it stopped at {n} N, but the push is not known square to its "
               "face ({why}): it may have jammed short of its end; {seen}",
    "forbidden": "the surface felt lies {mm} mm from the {what}, a part robot://devices forbids: the finger backed "
                 "off; no push made",
    "struck": "the arm felt {n} N on {where}, {near}{forbidden}: it stopped and backed off; no push made",
}
LIFT_SAID = {
    "changed": "lifted the {label} from under its edge at {n} N: it gave after {mm} mm; same view before and after: "
               "{seen}",
    "no_edge": "found no edge under the {label}: {why}; no push made",
}
STALL_SAID = "felt a stall that did not repeat: no surface here"
LEFT_SAID = "something came onto the finger as it left"
NO_BEFORE = "no before picture"
NO_WITNESS = "no same-view pair (the witness package is not loaded): look to see it"


def seen_missing(ex, before, after):
    """The seen sentence when no compare record exists: which piece is missing."""
    if not callable(((getattr(ex, "state", None) or {}).get("witness") or {}).get("go_and_look")):
        return NO_WITNESS
    if before is None:
        return NO_BEFORE + ": look to see the state"
    if after is None:
        return "no after picture: look to see the state"
    return "the two pictures could not be compared: look to see the state"


def _fmt(x):
    """Format a display value with at most one decimal."""
    if isinstance(x, float):
        return f"{x:.1f}".rstrip("0").rstrip(".") if abs(x) < E.MM else f"{x:.0f}"
    return str(x)


def cut_words(text, n):
    """Cut display text at its word envelope."""
    ws = str(text).split()
    if len(ws) <= n:
        return str(text)
    return " ".join(ws[:max(0, n - 1)]).rstrip(",;:") + " ..." if n > 0 else ""


def _fill(template, f):
    """Fill a sentence, rendering force fields with one decimal."""
    vals = {k: (f"{v:.1f}" if k in ("n", "C") and isinstance(v, (int, float)) else _fmt(v))
            for k, v in f.items()}
    out = template
    for k, v in vals.items():
        out = out.replace("{" + k + "}", v)
    return out


def said(stopped, **f):
    """
    The sentence of a push that stopped `stopped`; extra (a release note) is appended after a semicolon.
    The
    whole is cut to SAID_MAX_WORDS words, cutting the seen sentence first.
    """
    f.setdefault("why", "the most allowed here")
    f.setdefault("seen", NO_WITNESS)
    extra = f.pop("extra", None)
    text = _fill(SAID.get(stopped, "the push stopped: {stopped}"), {"stopped": stopped, **f})
    if extra:
        text = f"{text}; {extra}"
    if len(text.split()) > L.SAID_MAX_WORDS and "{seen}" in SAID.get(stopped, ""):
        over = len(text.split()) - L.SAID_MAX_WORDS
        seen_words = str(f["seen"]).split()
        f["seen"] = cut_words(f["seen"], max(1, len(seen_words) - over - 1))
        text = _fill(SAID[stopped], {"stopped": stopped, **f}) + (f"; {extra}" if extra else "")
    return cut_words(text, L.SAID_MAX_WORDS)


def lift_said(stopped, **f):
    """The lift sentence, bounded by the same word envelope as push."""
    f.setdefault("why", "the most allowed here")
    f.setdefault("seen", NO_WITNESS)
    if stopped in LIFT_SAID:
        return cut_words(_fill(LIFT_SAID[stopped], f), L.SAID_MAX_WORDS)
    return said(stopped, **f).replace("the push", "the lift")


def _why(text):
    """Keep next-call reasons inside the word envelope."""
    return cut_words(text, L.NEXT_WHY_MAX_WORDS)


def _call(tool, args, why):
    """One ready next call with its bounded reason."""
    return {"tool": tool, "args": args, "why": _why(why)}


def next_for(stopped, ctx):
    """
    The next of a push. ctx: tools (loaded names), target (id), label, part (name or
    None),
    others (fresh located ids of other control parts), up (the felt face points up), how (the
    goal's how_to_tell
    words), record (the witness record id or None), ceiling_n, unseen_side,
    sides_tried. Names only loaded tools.
    """
    tools = set(ctx.get("tools") or ())
    tgt = ctx.get("target")
    view = "view" in tools
    side = ctx.get("unseen_side") or "robot_side"
    if stopped in ("changed", "reached_end"):
        rec = ctx.get("record")
        then = f"claim, citing {rec}" if rec else "look, and claim only when the look shows it"
        return {"if_seen_shows": ctx.get("how") or "the state robot://devices gives in how_to_tell",
                "then": then, "else": "push again or view another side" if view else "push again or look again"}
    if stopped in ("unconfirmed", "stalled"):
        why = ("no picture shows a change: look at the part and at what shows its state" if stopped == "unconfirmed"
               else "it may have jammed: see how far it went before pushing again")
        return _call("view", {"target": tgt, "side": "around"}, why) if view else _call("look", {}, why)
    if stopped in ("forbidden", "struck"):
        why = ("this spot is at a part the sheet forbids: see the device and choose another part"
               if stopped == "forbidden" else "something stands on the way: see it before another try")
        return _call("look", {}, why)
    if stopped == "ceiling":
        why = f"needs more than {float(ctx.get('ceiling_n') or 0):.1f} N here, or this is not the spot"
        return _call("view", {"target": tgt, "side": "around"}, why) if view else _call("look", {}, why)
    if stopped == "rigid":
        others = ctx.get("others") or []
        if others and "push" in tools:
            return _call("push", {"target": others[0]}, "this spot does not move: push the part that moves")
        if view:
            return _call("view", {"target": tgt, "side": side}, "this spot does not move: see another side")
        return _call("look", {}, "this spot does not move: look for the part that moves")
    if stopped in ("not_found", "blocked"):
        why = "the point may be off or something stands in front"
        return _call("view", {"target": tgt, "side": "around"}, why) if view else _call("look", {}, why)
    if stopped == "not_square":
        if ctx.get("up") and "push" in tools:
            return _call("push", {"target": tgt, "way": "down"}, "the face points up: push straight down")
        why = "see the face square from its own side"
        return _call("view", {"target": tgt, "side": side}, why) if view else _call("look", {}, why)
    if stopped == "cannot_reach":
        why = "another side may let a finger reach it"
        return _call("view", {"target": tgt, "side": side}, why) if view else None
    return None


def point_call(look_id, part, why):
    """A point call: the operator supplies the pixel (3.1)."""
    return {"tool": "locate", "args": {"look": look_id, "part": part}, "needs": "px", "why": _why(why)}

BRIEF_ORDER = ("said", "stopped", "check", "felt", "peak_n", "ceiling_n", "ceiling_source", "bounds", "periods", "periods_used", "periods_left", "line", "contact", "seen", "witness", "moved_mm",
               "moved_dir", "moved_px", "by", "pictures", "next")


def brief(out):
    """The byte-bounded result in envelope order: aggregate named stage periods, keep the binding bound, then shorten optional evidence and words."""
    phases = {}
    if "stages" in out:
        for s in out["stages"]:
            stage = s.get("stage")
            if stage is None:
                continue
            n, old = s.get("periods"), phases.get(stage, 0)
            phases[stage] = None if n is None or old is None else old + n
    values = dict(out)
    if phases:
        values["periods"] = [[k, n] for k, n in phases.items()]
    res = {k: values[k] for k in BRIEF_ORDER if k in values}
    for k in ("refused",):
        if k in out:
            res = {k: out[k], **res}

    if isinstance(res.get("bounds"), list):
        items = res["bounds"]
        res["bounds"] = {"binding": items[0] if items else None, "more": max(0, len(items) - 1)}

    def size(d):
        """Encoded result bytes excluding pictures."""
        return len(json.dumps({k: v for k, v in d.items() if k != "pictures"}, separators=(",", ":")))

    if size(res) > L.BRIEF_MAX_BYTES and isinstance(res.get("bounds"), dict):
        bounds = dict(res["bounds"])
        if isinstance(bounds.get("binding"), dict):
            bounds["binding"] = {k: v for k, v in bounds["binding"].items() if k != "evidence"}
        res["bounds"] = bounds
    if size(res) > L.BRIEF_MAX_BYTES and isinstance(res.get("felt"), list) and len(res["felt"]) > 4:
        res["felt"] = res["felt"][-4:]
    nx = res.get("next")
    if size(res) > L.BRIEF_MAX_BYTES and isinstance(nx, dict) and isinstance(nx.get("if_seen_shows"), str):
        res["next"] = dict(nx, if_seen_shows=cut_words(nx["if_seen_shows"], 12))
    if size(res) > L.BRIEF_MAX_BYTES and isinstance(res.get("contact"), dict):
        res["contact"] = {k: v for k, v in res["contact"].items() if k in ("id", "point_mm")}
    if size(res) > L.BRIEF_MAX_BYTES and isinstance(res.get("seen"), str):
        res["seen"] = cut_words(res["seen"], 20)
    if size(res) > L.BRIEF_MAX_BYTES and isinstance(res.get("witness"), dict):
        res["witness"] = {k: v for k, v in res["witness"].items() if k in ("id", "changed")}
    if size(res) > L.BRIEF_MAX_BYTES and isinstance(res.get("seen"), str):
        res["seen"] = cut_words(res["seen"], 8)
    if size(res) > L.BRIEF_MAX_BYTES and isinstance(res.get("felt"), list):
        res["felt"] = res["felt"][-2:]
    while size(res) > L.BRIEF_MAX_BYTES:
        removed = False
        for key in ("seen", "next", "witness", "contact", "line", "moved_dir", "moved_px", "felt"):
            if key in res and res[key] is not None and res[key] != []:
                res[key] = [] if isinstance(res[key], list) else None
                removed = True
                break
        if not removed:
            current = res.get("said", "")
            shorter = cut_words(current, max(0, len(current.split()) - 1))
            if shorter == current:
                break
            res["said"] = shorter
    return res


def _controls(ex, info, exclude_id):
    """Fresh located ids of other control parts of the same device (for rigid's next)."""
    parts = G.plist(ex)
    out = []
    for e in ex.world.query(kind="located", include_stale=False):
        if e["id"] == exclude_id or e.get("point_mm") is None:
            continue
        p = sheet_words.label_part(e["label"], parts)
        if p is not None and p["role"] == "control" and (info["device"] is None or p["device"] == info["device"]) \
                and (info["part"] is None or p["name"] != info["part"]["name"]):
            out.append(e["id"])
    return out


def _how(ex, info):
    """
    The goal's how_to_tell words: the deciding part of the task step whose control part is this part,
    else the
    part's own how_to_tell.
    """
    doc = G.devices_doc(ex)
    task = (getattr(ex, "context", None) or {}).get("task") or {}
    sentence = task.get("task") or task.get("sentence") or task.get("text") or ""
    if info["part"] is None:
        return None
    try:
        for st in sheet_words.steps(sentence, doc):
            if st.get("control") == info["part"]["name"] and st.get("deciding"):
                dp = sheet_words.label_part(st["deciding"], sheet_words.parts(doc))
                if dp and dp["how_to_tell"]:
                    return dp["how_to_tell"]
    except Exception:
        pass
    return info["part"]["how_to_tell"] or None


def result(ex, run, info, u, stopped, fields, felt, presses, peak_n, ceiling_n, contact, travel, before, after,
            record, res, label):
    """Build a push result and retain the call's measured contact, force and witness evidence."""
    st = G.state(ex)
    P = info["P"] if info["P"] is not None else (np.asarray(contact["point_mm"], dtype=float)
                                                  if contact and contact.get("point_mm") else None)
    dir_words = side_words.words(u, P if P is not None else [1, 0, 0])
    seen = (record or {}).get("said") or seen_missing(ex, before, after)
    f = {"label": label, "dir": dir_words, "seen": seen, **{k: v for k, v in fields.items() if v is not None}}
    if stopped == "time":
        f.setdefault("s", round(run.elapsed(), 0))
        f.setdefault("stage", run.stage)
        f.setdefault("what", f"{len(felt)} pushes" if felt else "no push made")
    if stopped == "refused":
        f.setdefault("stage", run.stage)
        f.setdefault("check", "a gate")
    if stopped == "cap":
        f.setdefault("n", round(peak_n or run.peak_total, 1))
    if stopped == "cannot_reach":
        f.setdefault("why", "no tool surface reaches it")
    if stopped == "blocked":
        f.setdefault("n", round(run.peak_total, 1))
    if stopped == "not_found":
        f["mm"] = max(0, f.get("mm") or 0)          # how far the touch went: never under 0
    if stopped in ("unconfirmed", "stalled"):
        f.setdefault("how", "stayed in")
        f.setdefault("why", "the face was not fitted")
    if stopped == "struck":
        for k, v in (("n", round(run.peak_total, 1)), ("where", "the way"), ("near", "near no located point"),
                     ("forbidden", "")):
            f.setdefault(k, v)
    f.setdefault("why", ceiling_why(run))
    text = said(stopped, **f)
    off = None
    if contact and contact.get("point_mm") is not None and info["P"] is not None:
        off = round(float(np.linalg.norm(np.asarray(contact["point_mm"], dtype=float) - info["P"])), 1)
    wit = None
    if record is not None:
        at = record.get("at") or []
        mine = next((a for a in at if a.get("id") == info["id"]), at[0] if at else {})
        wit = {"id": record.get("record") or record.get("id"), "before": (before or {}).get("look_id"),
               "after": (after or {}).get("look_id"), "changed": mine.get("changed")}
    ctx = {"tools": G.tools(ex), "target": info["id"], "label": label, "others": _controls(ex, info, info["id"]),
           "up": bool(contact and contact.get("normal") and contact["normal"][2] > L.VERTICAL_Z),
           "how": _how(ex, info), "record": (wit or {}).get("id"), "ceiling_n": ceiling_n,
           "unseen_side": "left"}
    # Report the largest force in any stage, including the approach and
    # finding touch. Reporting only the ramp would hide an earlier strike.
    peak_n = max(float(peak_n or 0.0), float(run.peak_total))
    out = {"said": text, "stopped": stopped, "felt": felt, "peak_n": round(float(peak_n), 1),
           "ceiling_n": ceiling_n,
           "line": {"u": E._r(u, 3), "source": res.get("line_source")},
           "contact": None if not contact else {"id": contact.get("id"), "point_mm": contact.get("point_mm"),
                                                "normal": contact.get("normal"), "off_point_mm": off},
           "seen": seen, "witness": wit, "by": "runtime",
           "pictures": list((record or {}).get("pictures") or [])[:L.PICTURES_MAX],
           "next": next_for(stopped, ctx)}
    report_bounds(out, run)
    add_refusal(out, fields, info, "push")
    if fields.get("check"):
        out["check"] = fields["check"]          # the gate or check that stopped it, as a key (finish reads it)
    if run.detail == "full":
        out.update({k: fields[k] for k in ("why", "reasons") if k in fields})
    if run.notes:
        out["notes"] = run.notes[:4]
    out.update({"target": info["id"], "part": (info["part"] or {}).get("name"), "stages": run.legs,
                "touches": run.touches, "presses": presses, "peak_total_n": round(run.peak_total, 2),
                "sim_s": round(run.elapsed(), 2), **res})
    st["acts"].append({"seq": ex._calls, "tool": "push", "target": info["id"], "label": label,
                       "part": (info["part"] or {}).get("name"), "device": info["device"], "stopped": stopped,
                       "felt": felt, "peak_n": out["peak_n"], "ceiling_n": ceiling_n,
                       "travel_mm": round(float(travel or 0.0), 1), "witness": wit,
                       "contact": out["contact"], "point_mm": None if info["P"] is None else E._r(info["P"], 1),
                       "last_n": felt[-1]["n"] if felt else None, "uncertain_mm": info["uncertain_mm"],
                       **force_evidence(run)})
    for p in presses:
        st["presses"].append({"call": ex._calls, "tool": "push", "start_mm": p["start_mm"], "force_n": p["n"],
                              "along_mm": p["along_mm"], "gave": p["gave"], "rigid": p["word"] == "rigid"})
    return out


def ceiling_why(run):
    """The binding force item's reason, or the found end of travel."""
    if run.ramps and run.ramps[-1]["stopped"] == "stall":
        return "where its travel ended"
    return act_ladder.binding(run.items)[1] or "the most allowed here"


def report_bounds(out, run):
    """Attach the binding source, core touch peak bound, full bounds and compact phase periods."""
    item, _ = act_ladder.binding(run.items)
    out["ceiling_source"] = run.ramps[-1]["source"] if run.ramps else (item or {}).get("source")
    if out.get("ceiling_n") is None:
        out["ceiling_n"] = (item or {}).get("value") or 0
    phases = {}
    for rec in run.legs:
        phase = rec.get("phase", rec.get("stage"))
        if phase is not None:
            phases[phase] = phases.get(phase, 0) + (rec.get("periods") or 0)
    out["periods"] = [[phase, n] for phase, n in phases.items()]
    out["periods_used"] = sum(phases.values())
    left = getattr(run.ex, "periods_left", None)
    if left is not None:
        out["periods_left"] = left() if callable(left) else left
    out["touch_bound_n"] = getattr(run, "touch_bound_n", 0.0)
    run.B.report(out, run.items, run.detail)
    return out


def force_evidence(run):
    """The largest first-motion force and largest force held without motion across legs."""
    moved = [r["f_moved_n"] for r in run.ramps if r.get("f_moved_n") is not None]
    return {"f_moved_n": max(moved) if moved else None,
            "f_bear_n": max([r.get("f_bear_n") or 0 for r in run.ramps] or [0])}


def add_refusal(out, fields, info, verb):
    """Retain a nested stage refusal; force_unknown is already handled by P0."""
    if out.get("stopped") != "refused":
        return
    bad = fields.get("refused") or {}
    if bad:
        out["refused"] = bad

__all__ = ["result", "LEFT_SAID", "NO_BEFORE", "NO_WITNESS", "SAID", "STALL_SAID", "brief", "cut_words", "lift_said",
           "next_for", "point_call", "said"]
