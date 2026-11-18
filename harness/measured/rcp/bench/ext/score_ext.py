# SPDX-License-Identifier: AGPL-3.0-only
"""The result of one benchmark episode, from its truth log (the design notes S2; standard library only, so
it runs in either venv).

    python -m bench.ext.score_ext --ep EPDIR [--out FILE] [--prices PRICES.json]

Reads, and never writes, the episode's files:
    scoring dir (bench.run_episode.scoring_dir(EPDIR), outside EPDIR): episode.json (the cell, surface, budgets) and
        the env host's per-step truth log (TRUTH_NAMES: one record per counted control step with "step" and
        "success"; records with "event" (reset, first_success, end) or "kind" (cell, end) are not steps; when the
        log holds several episodes ("ep"), only the last reset's are read)
    EPDIR: episode_end.json and journal.jsonl (the rcpx server's: claim, calls, motion calls, refusals) and
        operator.jsonl (the chat operator's: replies with usage, the end record with its counters)
and writes the result (default <scoring dir>/result.json):
    success                the predicate held at some counted control step k <= budget_steps
    first_success_step     that first k (None when it never held within the budget)
    success_within_ref     k <= the reference max steps of the suite (openpi examples/libero main.py)
    success_after_budget_step  a first success logged after the budget (reported, never counted)
    steps_used, model_requests (completed model replies), http_requests, tool_calls (every call the model sent),
    server_calls, motion_calls, refusals, claim, claim_by, claimed_success, false_claim (a success claim when the
    predicate never held within the budget), end_reason, harm (peak tared wrist force from the truth log's
    sensing, None when the log has none), tokens and cost (only when the operator log or a price table has them),
    cost_usd and cost: the endpoint's own reported USD when it sent one (basis "reported"), else the dated list
    prices of bench/ext/published.py PRICES applied reply by reply, cached and long-prompt rates included (basis
    "list_price"); None for a model without a list price (never guessed).
    One call per reply: tool_calls is every call the model SENT; calls_not_run the calls the operator refused to
    run under --one-call-per-reply; calls_run = tool_calls - calls_not_run; more_calls_than_requests (the model
    sent more calls than it made replies) and protocol_break (more calls RAN than replies: the published protocol
    is one call per reply) are None when the operator log lacks the counts. length_cutoffs: replies cut off at
    the token cap.
    problems (every input that was missing or unreadable: a result is never silently partial).
"""

import argparse
import json
import math
import os
import sys

SCHEMA = "ext_result/1"
TRUTH_NAMES = ("truth_steps.jsonl", "truth_log.jsonl", "truth.jsonl", "truth_log.json")
EPISODE_NAME = "episode.json"
END_NAME = "episode_end.json"
JOURNAL_NAME = "journal.jsonl"
OPERATOR_NAME = "operator.jsonl"
RESULT_NAME = "result.json"
DEFAULT_BUDGET_STEPS = 900
# Reference max steps per suite (openpi examples/libero/main.py, scout NOTES.md); LIBERO-PRO cells keep their suite's.
REF_MAX_STEPS = {"libero_10": 520, "libero_spatial": 220, "libero_object": 280, "libero_goal": 300,
                 "libero_90": 400}
ARMS = {"native": "without", "native_sense": "without_sense", "native_calib": "without_calib", "vnext": "with"}
SUCCESS_TOOLS = {"done": True, "give_up": False}   # the native surface's ending tools, read when no claim is recorded


def scoring_dir(ep_dir):
    """bench.run_episode.scoring_dir, restated so this file needs nothing outside the standard library."""
    ep = os.path.abspath(ep_dir)
    return os.path.join(os.path.dirname(ep), "_scoring", os.path.basename(ep))


def _read_json(path, problems, what):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        problems.append(f"{what}: {os.path.basename(path)} missing")
    except (OSError, ValueError) as exc:
        problems.append(f"{what}: {os.path.basename(path)} unreadable ({type(exc).__name__}: {exc})")
    return None


def _read_jsonl(path, problems, what, required=True):
    out = []
    try:
        with open(path, encoding="utf-8") as f:
            for i, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except ValueError:
                    problems.append(f"{what}: line {i} of {os.path.basename(path)} is not JSON")
                    continue
                if isinstance(rec, dict):
                    out.append(rec)
    except FileNotFoundError:
        if required:
            problems.append(f"{what}: {os.path.basename(path)} missing")
        return None
    except OSError as exc:
        problems.append(f"{what}: {os.path.basename(path)} unreadable ({exc})")
        return None
    return out


# ---------------------------------------------------------------------------------------------------- truth log
def _force_n(rec):
    v = rec.get("wrist_force_n")
    if isinstance(v, (int, float)) and math.isfinite(v):
        return abs(float(v))
    vec = rec.get("wrist_force")
    if isinstance(vec, (list, tuple)) and len(vec) == 3 and all(isinstance(x, (int, float)) for x in vec):
        return math.sqrt(sum(float(x) * float(x) for x in vec))
    return None


def load_truth(scoring, problems):
    """(header dict, [(step, success, force or None)] in step order, end dict) from the first truth log found."""
    for name in TRUTH_NAMES:
        path = os.path.join(scoring, name)
        if not os.path.exists(path):
            continue
        if name.endswith(".json"):
            doc = _read_json(path, problems, "truth log")
            records = list(doc.get("steps", [])) if isinstance(doc, dict) else []
            header = {k: v for k, v in doc.items() if k != "steps"} if isinstance(doc, dict) else {}
            end = header.get("end") if isinstance(header.get("end"), dict) else {}
        else:
            records = _read_jsonl(path, problems, "truth log") or []
            resets = [r for r in records if r.get("event") == "reset"]
            if resets and "ep" in resets[-1]:   # the env host's log: keep the last episode it built
                records = [r for r in records if r.get("ep") == resets[-1]["ep"]]
            header = next((r for r in records if r.get("kind") in ("cell", "header", "start")
                           or r.get("event") == "reset"), {})
            end = next((r for r in reversed(records) if r.get("kind") == "end" or r.get("event") == "end"), {})
            if "steps_used" not in end and isinstance(end.get("step"), int):
                end = dict(end, steps_used=end["step"])
        steps = []
        for r in records:
            if not isinstance(r, dict) or "step" not in r or "event" in r or \
                    r.get("kind") in ("cell", "header", "start", "end"):
                continue
            try:
                k = int(r["step"])
            except (TypeError, ValueError):
                problems.append(f"truth log: a step record has step {r.get('step')!r}")
                continue
            steps.append((k, r.get("success") is True, _force_n(r)))
        steps.sort(key=lambda s: s[0])
        return header, steps, end, name
    problems.append(f"truth log: none of {', '.join(TRUTH_NAMES)} in the scoring dir")
    return {}, [], {}, None


def ref_max_steps(cell):
    for key in ("ref_max_steps", "reference_max_steps", "max_steps_ref"):
        if isinstance(cell.get(key), int):
            return cell[key]
    suite = str(cell.get("suite") or "")
    for name, n in sorted(REF_MAX_STEPS.items(), key=lambda kv: -len(kv[0])):
        if name in suite:
            return n
    return None


def judge(steps, budget_steps, ref_steps):
    """The success fields from (step, success, force) records (pure: the scorer's decision)."""
    first = next((k for k, ok, _ in steps if ok and k <= budget_steps), None)
    after = next((k for k, ok, _ in steps if ok and k > budget_steps), None) if first is None else None
    used = max((k for k, _, _ in steps), default=0)
    forces = [f for k, _, f in steps if f is not None and k <= budget_steps]
    return {"success": first is not None, "first_success_step": first,
            "success_within_ref": first is not None and ref_steps is not None and first <= ref_steps,
            "success_after_budget_step": after, "steps_used": min(used, budget_steps),
            "steps_over_budget": sum(1 for k, _, _ in steps if k > budget_steps),
            "peak_wrist_force_n": round(max(forces), 3) if forces else None}


# ------------------------------------------------------------------------------------------ server and operator
def _claim_from_journal(journal):
    """For a surface that ends with tools (done, give_up) and records no claim: the last unrefused ending call."""
    for rec in reversed(journal or []):
        if rec.get("kind") != "call" or rec.get("name") not in SUCCESS_TOOLS:
            continue
        res = rec.get("result")
        if isinstance(res, dict) and "refused" in res:
            continue
        args = rec.get("args") if isinstance(rec.get("args"), dict) else {}
        text = args.get("summary") or args.get("reason") or ""
        return {"statement": str(text), "success": SUCCESS_TOOLS[rec["name"]], "tool": rec["name"]}
    return None


def server_side(ep_dir, problems):
    end = _read_json(os.path.join(ep_dir, END_NAME), problems, "server end record") or {}
    journal = _read_jsonl(os.path.join(ep_dir, JOURNAL_NAME), problems, "server journal", required=False)
    claim = end.get("claim") if isinstance(end.get("claim"), dict) else None
    if claim is None:
        claim = _claim_from_journal(journal)
    return {"end_reason": end.get("reason"), "server_calls": end.get("calls_used"),
            "calls_free": end.get("calls_free"), "motion_calls": end.get("motion_calls"),
            "refusals": end.get("refusals"), "claim": claim}


def operator_side(ep_dir, problems):
    recs = _read_jsonl(os.path.join(ep_dir, OPERATOR_NAME), problems, "operator log", required=False)
    if recs is None:
        problems.append("operator log: operator.jsonl missing (requests, tool calls and tokens not scored)")
        return {}
    end = next((r for r in reversed(recs) if r.get("kind") == "end"), None)
    counts = end.get("counts", {}) if isinstance(end, dict) else {}
    if end is None:
        problems.append("operator log: no end record (the operator did not finish)")
    replies = [r for r in recs if r.get("kind") == "reply"]
    tok = {"prompt": 0, "completion": 0, "total": 0, "cached": 0, "replies_with_usage": 0}
    cost = 0.0
    cost_seen = False
    usages = []
    cached_seen = False
    for r in replies:
        u = r.get("usage")
        if not isinstance(u, dict):
            continue
        usages.append(u)
        p, c, cached, seen = usage_tokens(u)
        tok["cached"] += cached
        cached_seen = cached_seen or seen
        if isinstance(p, int) or isinstance(c, int):
            tok["replies_with_usage"] += 1
            tok["prompt"] += p if isinstance(p, int) else 0
            tok["completion"] += c if isinstance(c, int) else 0
            tok["total"] += u["total_tokens"] if isinstance(u.get("total_tokens"), int) else (
                (p if isinstance(p, int) else 0) + (c if isinstance(c, int) else 0))
        if isinstance(u.get("cost"), (int, float)):
            cost += float(u["cost"])
            cost_seen = True
    start = next((r for r in recs if r.get("kind") == "start"), {})
    run = start.get("run") if isinstance(start.get("run"), dict) else {}
    return {"model": run.get("model"),
            "model_requests": counts.get("turns", len(replies)) if end is not None else len(replies),
            "http_requests": counts.get("http_requests"), "http_failures": counts.get("http_failures"),
            "tool_calls": counts.get("tool_calls"), "reads": counts.get("reads"),
            "length_cutoffs": counts.get("length_cutoffs"), "calls_not_run": counts.get("calls_not_run"),
            "multi_call_replies": counts.get("multi_call_replies"),
            "claim_by": end.get("claim_by") if isinstance(end, dict) else None,
            "operator_end": end.get("reason") if isinstance(end, dict) else None,
            "operator_exit": end.get("exit_code") if isinstance(end, dict) else None,
            "operator_claim": end.get("claim") if isinstance(end, dict) else None,
            "tokens": tok if tok["replies_with_usage"] else None,
            "cost_usd_reported": round(cost, 6) if cost_seen else None,
            "usages": usages, "cached_tokens_seen": cached_seen}


def usage_tokens(usage):
    """Prompt, completion, clipped cache and whether a cache counter was supplied."""
    p = usage.get("prompt_tokens")
    c = usage.get("completion_tokens")
    if not isinstance(p, int):
        p = usage.get("input_tokens")
    if not isinstance(c, int):
        c = usage.get("output_tokens")
    details = usage.get("prompt_tokens_details") or {}
    cached = details.get("cached_tokens") if isinstance(details, dict) else None
    if not isinstance(cached, int):
        cached = usage.get("cache_read_input_tokens")
    seen = isinstance(cached, int)
    cached = min(max(0, cached), max(0, p)) if seen and isinstance(p, int) else 0
    # Vertex's OpenAI-compatible route counts thinking apart: total = prompt + completion + reasoning, where the
    # other routes have reasoning inside completion (total = prompt + completion). Thinking is billed as output.
    out_details = usage.get("completion_tokens_details") or {}
    think = out_details.get("reasoning_tokens") if isinstance(out_details, dict) else None
    total = usage.get("total_tokens")
    if isinstance(think, int) and think > 0 and isinstance(p, int) and isinstance(c, int) \
            and isinstance(total, int) and total >= p + c + think:
        c += think
    return p, c, cached, seen


def list_cost(usages, row):
    """USD from per-reply list prices, cache and prompt thresholds (pure)."""
    if row is None or not usages:
        return None
    cost, seen = 0.0, False
    for u in usages:
        if not isinstance(u, dict):
            continue
        p, c, cached, _ = usage_tokens(u)
        if not isinstance(p, int) and not isinstance(c, int):
            continue
        p, c = p if isinstance(p, int) else 0, c if isinstance(c, int) else 0
        long = row.get("long_prompt_over") is not None and p > row["long_prompt_over"]
        prefix = "long_" if long else ""
        inp, out = row.get(prefix + "in_per_mtok"), row.get(prefix + "out_per_mtok")
        cin = row.get(prefix + "cached_in_per_mtok")
        if inp is None or out is None:
            return None
        cin = inp if cin is None else cin
        cost += ((p - cached) * inp + cached * cin + c * out) / 1e6
        seen = True
    return round(cost, 6) if seen else None


def priced(tokens, model_key, prices):
    """Legacy total-token helper for short-context price tables."""
    if not tokens or not prices:
        return None
    return list_cost([{"prompt_tokens": tokens["prompt"], "completion_tokens": tokens["completion"]}],
                     prices.get(model_key))


# ------------------------------------------------------------------------------------------------------- score
def score(ep_dir, out=None, prices=None, write=True):
    """The result dict for EPDIR; written to out (default <scoring dir>/result.json) unless write is False."""
    ep_dir = os.path.abspath(ep_dir)
    scoring = scoring_dir(ep_dir)
    problems = []
    info = _read_json(os.path.join(scoring, EPISODE_NAME), problems, "episode identity") or {}
    header, steps, tend, tname = load_truth(scoring, problems)
    cell = {**(header.get("cell") or {}), **info}   # the episode's own identity (the grid's cell id) wins
    budget = info.get("budget") if isinstance(info.get("budget"), dict) else {}
    budget_steps = int(header.get("budget_steps") or budget.get("steps") or info.get("budget_steps")
                       or DEFAULT_BUDGET_STEPS)
    ref = header.get("ref_max_steps") if isinstance(header.get("ref_max_steps"), int) else ref_max_steps(cell)
    if ref is None:
        problems.append(f"no reference max steps for suite {cell.get('suite')!r}")
    verdict = judge(steps, budget_steps, ref)
    if isinstance(tend.get("steps_used"), int):
        verdict["steps_used"] = min(int(tend["steps_used"]), budget_steps)
    if tname is not None and not steps:
        problems.append("truth log: no step records (the episode ran no control step, or the log is cut)")
    srv = server_side(ep_dir, problems)
    op = operator_side(ep_dir, problems)
    claim = srv["claim"] or op.get("operator_claim")
    claimed = claim.get("success") if isinstance(claim, dict) and isinstance(claim.get("success"), bool) else None
    surface = cell.get("surface") or info.get("surface")
    model = cell.get("model") or op.get("model")
    key = _canonical(model)
    from bench.ext import published
    row = prices.get(key) if prices is not None else published.price(key)
    cost = op.get("cost_usd_reported")
    basis = "reported" if cost is not None else None
    if cost is None:
        cost = list_cost(op.get("usages"), row)
        basis = "list_price" if cost is not None else None
    calls, requests = op.get("tool_calls"), op.get("model_requests")
    counted = isinstance(calls, int) and isinstance(requests, int)
    not_run = op.get("calls_not_run") if isinstance(op.get("calls_not_run"), int) else 0
    calls_run = calls - not_run if isinstance(calls, int) else None
    result = {
        "schema": SCHEMA, "ep_dir": ep_dir, "truth_log": tname,
        "cell": {k: cell.get(k) for k in ("id", "bench", "suite", "task", "init", "perturb", "surface",
                                           "packages", "language") if k in cell},
        "arm": ARMS.get(surface, surface), "model": model, "model_key": key,
        "budget_steps": budget_steps, "ref_max_steps": ref,
        **{k: verdict[k] for k in ("success", "first_success_step", "success_within_ref",
                                   "success_after_budget_step", "steps_used", "steps_over_budget")},
        "model_requests": op.get("model_requests"), "http_requests": op.get("http_requests"),
        "http_failures": op.get("http_failures"), "tool_calls": op.get("tool_calls"), "reads": op.get("reads"),
        "server_calls": srv["server_calls"], "calls_free": srv["calls_free"], "motion_calls": srv["motion_calls"],
        "refusals": srv["refusals"], "end_reason": srv["end_reason"], "operator_end": op.get("operator_end"),
        "operator_exit": op.get("operator_exit"),
        "length_cutoffs": op.get("length_cutoffs"), "calls_not_run": op.get("calls_not_run"),
        "multi_call_replies": op.get("multi_call_replies"), "calls_run": calls_run,
        "more_calls_than_requests": calls > requests if counted else None,
        "protocol_break": calls_run > requests if counted else None,
        "claim": claim, "claim_by": op.get("claim_by"), "claimed_success": claimed,
        "false_claim": claimed is True and not verdict["success"],
        "harm": {"peak_wrist_force_n": verdict["peak_wrist_force_n"],
                 "measured": verdict["peak_wrist_force_n"] is not None,
                 "source": "truth log wrist force (tared), within the budget"},
        "tokens": op.get("tokens"),
        "cost_usd": cost,
        "cost": {"basis": basis, "url": row.get("url") if row and basis == "list_price" else None,
                 "date": row.get("date") if row and basis == "list_price" else None,
                 "cached_tokens_seen": op.get("cached_tokens_seen", False)},
        "problems": problems,
    }
    if write:
        path = out or os.path.join(scoring, RESULT_NAME)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, sort_keys=True)
        os.replace(tmp, path)
    return result


def _canonical(model):
    try:
        from bench.ext.published import canonical
    except ImportError:          # run as a plain file: the sibling module by path
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        from bench.ext.published import canonical
    return canonical(model)


def main(argv=None):
    p = argparse.ArgumentParser(prog="python -m bench.ext.score_ext", description=__doc__.splitlines()[0])
    p.add_argument("--ep", required=True, help="the episode directory")
    p.add_argument("--out", default=None, help="result file (default <scoring dir>/result.json)")
    p.add_argument("--prices", default=None, help="JSON {model key: {in_per_mtok, out_per_mtok}} (optional)")
    ns = p.parse_args(argv)
    prices = None
    if ns.prices:
        with open(ns.prices, encoding="utf-8") as f:
            prices = json.load(f)
    result = score(ns.ep, out=ns.out, prices=prices)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
