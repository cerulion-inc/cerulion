#!/usr/bin/env python3
"""Recompute every count, interval and test of this package from its CSV files.

Reads cells.csv, episodes.csv, unscored.csv and not_finished.csv in this folder and writes summary.csv,
paired.csv and SUMMARY.md. For the extension's lists present (extension_list2/, extension_libero10/) it does the
same there (plus task_rows.csv), with the one family of twelve tests the extension's rule fixes over both lists;
for extension_libero10_development/ it writes counts only. The rules are in RULES.md. Standard library only.

usage: recompute.py [--name NAME] [--check]
  --check  writes nothing; exits 1 if the files on disk differ from what the CSV files give
  either way it exits 1 if a row of a cells.csv no longer matches its content hash, or if a row of an episodes.csv
  has replies by finish reason (the five replies_* columns) that do not add up to its requests
"""
import argparse, csv, json, math, os, sys
from fractions import Fraction

HERE = os.path.dirname(os.path.abspath(__file__))
ALPHA = 0.05
MAIN_MODEL = "Gemini 3.8 Flash"
CLEAN_MODEL = "GPT-6 Astra"
ADDED_LATE = ["GPT-6.1 Terra"]          # a column added after the grid's first rows had been seen
CROSS_WITHOUT = ["GPT-6 Astra", "GPT-6.2 Astra"]   # RULES.md section 3: MAIN_MODEL with against these without
CLOCK_REASON = "the 90 minute episode clock ran out"   # the unscored.csv reason of an episode the clock ended
CLOCK_S = 5400                           # the episode clock, seconds
RETRY_WAIT_S, HTTP_TIMEOUT_S = 20, 600   # a failed model request is sent again after this wait; a request's timeout
MEASURES = [("solved", "SOLVED"), ("clean", "CLEAN"), ("false_claim", "FALSE CLAIM")]
READ = {}                                # list name -> the time its rows were read (from its list_info.json)


def load(folder, name):
    p = os.path.join(folder, name)
    if not os.path.exists(p):
        return []
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def wilson(k, n, z=1.959964):
    if not n:
        return None
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def sign_p(a, b):
    """Exact two-sided sign test on a against b discordant cells."""
    n = a + b
    if n == 0:
        return None
    k = min(a, b)
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)


def newcombe_paired(e, f, g, h):
    """Newcombe's score interval for a difference of paired proportions (1998, the method built on the two
    Wilson intervals and the phi coefficient of the paired table, phi taken without a continuity correction).
    e both, f first only, g second only, h neither. Returns (low, high) of p1 - p2."""
    n = e + f + g + h
    if not n:
        return None
    p1, p2 = (e + f) / n, (e + g) / n
    l1, u1 = wilson(e + f, n)
    l2, u2 = wilson(e + g, n)
    A = (e + f) * (g + h) * (e + g) * (f + h)
    phi = 0.0 if A == 0 else (e * h - f * g) / math.sqrt(A)      # the phi coefficient, no continuity correction
    lo = (p1 - p2) - math.sqrt(max(0.0, (p1 - l1) ** 2 - 2 * phi * (p1 - l1) * (u2 - p2) + (u2 - p2) ** 2))
    hi = (p1 - p2) + math.sqrt(max(0.0, (u1 - p1) ** 2 - 2 * phi * (u1 - p1) * (p2 - l2) + (p2 - l2) ** 2))
    return lo, hi


def holm(ps):
    """ps: {key: p or None}. Returns {key: (threshold, passed)} by Holm's step-down at ALPHA."""
    keys = sorted((k for k in ps if ps[k] is not None), key=lambda k: ps[k])
    out, alive = {}, True
    for i, k in enumerate(keys):
        t = ALPHA / (len(keys) - i)
        alive = alive and ps[k] <= t
        out[k] = (t, alive)
    return out


def pct(iv):
    return "" if iv is None else f"{100 * iv[0]:.1f} to {100 * iv[1]:.1f}"


def fp(p):
    """An exact p as every table of this package prints it: three significant digits, or two decimals of a power
    of ten under 0.001."""
    return "" if p is None else (f"{p:.2e}" if p < 0.001 else f"{p:.3g}")


def fnum(v, d=1):
    return "" if v is None else f"{v:,.{d}f}"


class Column:
    """One model on one list: both arms, every cell of the list."""

    def __init__(self, model, cells, eps, uns, left):
        self.model, self.cells = model, cells
        self.arm = {}
        for arm in ("without", "with"):
            sc = {e["cell"]: e for e in eps if e["model"] == model and e["arm"] == arm}
            un = {u["cell"]: u for u in uns if u["model"] == model and u["arm"] == arm}
            lf = {x["cell"] for x in left if x["model"] == model and x["arm"] == arm}
            self.arm[arm] = {"scored": sc, "unscored": un, "left": lf}
        self.complete = all(not self.arm[a]["left"] and len(self.arm[a]["scored"]) + len(self.arm[a]["unscored"])
                            == len(cells) for a in ("without", "with"))

    def finished(self, arm, cell):
        a = self.arm[arm]
        return cell in a["scored"] or cell in a["unscored"]

    def val(self, arm, cell, measure):
        e = self.arm[arm]["scored"].get(cell)          # an unscored cell is a failure: 0 on every measure
        return int(e[measure]) if e else 0

    def stats(self, arm):
        a = self.arm[arm]
        sc = list(a["scored"].values())
        n = len(sc) + len(a["unscored"])
        tot = lambda k: sum(float(e[k] or 0) for e in sc)
        mean = lambda k: tot(k) / len(sc) if sc else None
        solved = sum(int(e["solved"]) for e in sc)
        fc = sum(int(e["false_claim"]) for e in sc)
        return {"finished": n, "scored": len(sc), "unscored": len(a["unscored"]), "left": len(self.cells) - n,
                "solved": solved, "solved_wilson": wilson(solved, n),
                "clean": sum(int(e["clean"]) for e in sc),
                "clean_wilson": wilson(sum(int(e["clean"]) for e in sc), n),
                "false_claims": fc, "false_claims_per_unsolved": (fc / (n - solved)) if n - solved else None,
                "unsolved": n - solved,
                "requests": mean("requests"), "steps": mean("steps"), "wall_s": mean("wall_s"),
                "usd": mean("usd_list"), "usd_sum": tot("usd_list"),
                "usd_per_solved": (tot("usd_list") / solved) if solved else None,
                "tokens_in": mean("tokens_in"), "tokens_cached": mean("tokens_cached"),
                "cached_share": (tot("tokens_cached") / tot("tokens_in")) if tot("tokens_in") else None,
                "tokens_out": mean("tokens_out"), "failed": int(tot("failed_requests"))}

    def paired(self, measure, scored_only=False, other=None, other_arm="without", cells=None):
        """Cells where both arms have ended. with-only, without-only, both, neither."""
        o = other or self
        e = f = g = h = 0
        for c in (cells or [x["cell"] for x in self.cells]):
            if not (self.finished("with", c) and o.finished(other_arm, c)):
                continue
            if scored_only and (c not in self.arm["with"]["scored"] or c not in o.arm[other_arm]["scored"]):
                continue
            w, p = self.val("with", c, measure), o.val(other_arm, c, measure)
            e += w and p; f += w and not p; g += p and not w; h += (not w) and (not p)
        return {"pairs": e + f + g + h, "both": e, "with_only": f, "without_only": g, "neither": h,
                "p": sign_p(f, g), "diff": ((f - g) / (e + f + g + h)) if e + f + g + h else None,
                "newcombe": newcombe_paired(e, f, g, h)}


def columns(folder):
    cells = load(folder, "cells.csv")
    eps, uns, left = load(folder, "episodes.csv"), load(folder, "unscored.csv"), load(folder, "not_finished.csv")
    models = list(dict.fromkeys([r["model"] for r in eps] + [r["model"] for r in uns] + [r["model"] for r in left]))
    return cells, [Column(m, cells, eps, uns, left) for m in models], uns


def verdict(pr, t, measure):
    """The wording of RULES.md for one test of a Holm family: the grid's SOLVED and CLEAN families, and the
    extension's twelve tests. The grid declares no per-model false-claim test, so paired_rows never calls this for
    a grid false-claim row."""
    if pr["p"] is None:
        return "no discordant cell"
    ahead = pr["with_only"] > pr["without_only"]
    if measure == "false_claim":
        ahead = not ahead                      # fewer false claims is the favourable side
    if t and t[1]:
        return "established" if ahead else "established AGAINST"
    if pr["p"] <= ALPHA:
        return ("points up" if ahead else "points down") + ", not established"
    return "no difference shown"


def arm_table(cols, name, N):
    L = ["| model | arm | finished | solved | Wilson 95, percent | clean | Wilson 95, percent | false claims | "
         "per unsolved episode | unscored | not finished |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in cols:
        for arm in ("without", "with"):
            s = c.stats(arm)
            star = "" if c.complete else "*"
            L.append(f"| {c.model}{star} | {arm} {name} | {s['finished']} | {s['solved']} | {pct(s['solved_wilson'])} | "
                     f"{s['clean']} | {pct(s['clean_wilson'])} | {s['false_claims']} | "
                     f"{'' if s['false_claims_per_unsolved'] is None else format(s['false_claims_per_unsolved'], '.2f')}"
                     f" | {s['unscored']} | {s['left']} |")
    return L


def means_table(cols, name):
    L = ["| model | arm | scored episodes | replies | control steps | wall seconds | failed requests in all, each "
         "sent again | USD, list price | tokens in | of which cached | cached share, percent | tokens out | USD per "
         "solved start state |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in cols:
        for arm in ("without", "with"):
            s = c.stats(arm)
            share = "" if s["cached_share"] is None else f"{100 * s['cached_share']:.1f}"
            L.append(f"| {c.model}{'' if c.complete else '*'} | {arm} {name} | {s['scored']} | {fnum(s['requests'])} | "
                     f"{fnum(s['steps'])} | {fnum(s['wall_s'])} | {s['failed']:,} | {fnum(s['usd'], 3)} | "
                     f"{fnum(s['tokens_in'], 0)} | {fnum(s['tokens_cached'], 0)} | {share} | "
                     f"{fnum(s['tokens_out'], 0)} | "
                     f"{'none solved' if s['usd_per_solved'] is None else fnum(s['usd_per_solved'], 2)} |")
    return L


def joined(items):
    """a, b and c"""
    return items[0] if len(items) == 1 else ", ".join(items[:-1]) + " and " + items[-1]


def clock_ended(uns):
    """{(model, arm): the number of that arm's episodes the episode clock ended, unscored}"""
    out = {}
    for u in uns:
        if u["reason"] == CLOCK_REASON:
            out[u["model"], u["arm"]] = out.get((u["model"], u["arm"]), 0) + 1
    return out


def lost_usd(lost, model, arm):
    return sum(float(x["usd_list"]) for x in lost if x["model"] == model and x["arm"] == arm)


def all_in(c, arm, uns, lost):
    """The arm's means with its unscored episodes counted too: each one the clock ended at CLOCK_S seconds (None for
    seconds when another reason ended one, whose length no file holds), and every attempt in no row at its cost."""
    s = c.stats(arm)
    ck = clock_ended(uns).get((c.model, arm), 0)
    usd = s["usd_sum"] + lost_usd(lost, c.model, arm)
    wall = ((s["wall_s"] or 0.0) * s["scored"] + CLOCK_S * ck) / (s["scored"] + ck) \
        if ck == s["unscored"] and s["scored"] + ck else None
    return {"wall_s": wall, "usd": usd / s["finished"], "usd_per_solved": usd / s["solved"] if s["solved"] else None}


def means_notes(cols, name, uns, lost):
    """The notes printed under the means: the stall note that goes with seconds and the caching note that goes with
    USD (RULES.md section 6), what no figure counts, and the episodes the clock ended, which are in no mean."""
    M = len(cols)
    ck = clock_ended(uns)
    by = {"without": [], "with": []}
    for c in cols:
        for arm in ("without", "with"):
            if ck.get((c.model, arm)) and c.stats(arm)["wall_s"] < CLOCK_S:
                by[arm].append(c.model)
    lowered = []
    if by["without"]:
        lowered.append(f"{joined(by['without'])} without {name}")
    if by["with"]:
        lowered.append(f"{joined(by['with'])} with {'it' if by['without'] else name}")
    nck = sum(ck.values())
    lower_cache = sum(1 for c in cols if c.stats("without")["cached_share"] is not None
                      and c.stats("with")["cached_share"] is not None
                      and c.stats("without")["cached_share"] < c.stats("with")["cached_share"])
    total_lost = sum(float(x["usd_list"]) for x in lost)
    T = [f"Wall seconds are wall clock from the episode's start to its end, tool execution and failed requests "
         f"included. A failed request was sent again inside its episode after a {RETRY_WAIT_S} second wait, and a "
         f"request could wait up to {HTTP_TIMEOUT_S} seconds before it timed out, so an arm's seconds carry its "
         f"endpoint's stalls: read them with the arm's failed requests, printed beside them. Other workloads "
         f"shared the machines."
         + (f" The {nck} episodes that the 90 minute clock ended ({CLOCK_S} seconds each) are in no mean here"
            + (f"; leaving them out lowers the mean wall seconds of {', and of '.join(lowered)}." if lowered else ".")
            if nck else "")
         + " A statement about speed in wall seconds is read with all of this in mind (RULES.md section 6).", "",
         f"USD is the model's reported token counts of the scored attempt at the list prices of prices.csv on the "
         f"date in that file; cached input is priced at the cached rate; simulator compute is not included; nothing "
         f"here is a billed amount. USD per solved start state is the spend of the arm's scored episodes over the "
         f"start states it solved: the {len(uns)} episodes that ended without a score and every attempt in no row "
         f"are not in it ({total_lost:.2f} USD in all, lost_attempts.csv), and no figure counts the robot's "
         f"calibration episodes or the two attempts lost with the stopped machines. Part of the gap in USD between "
         f"the arms is caching, not only fewer requests: cached input is priced lower, and the arm without {name} "
         f"had the lower cached share of input tokens for {lower_cache} of the {M} models. Where an arm's prompt "
         f"changes near its start from turn to turn, as the published interface's does when its oldest images drop "
         f"out, less of it can be read from a cache. Tokens in counts every prompt token, cached ones included; "
         f"tokens out counts reasoning tokens.", ""]
    return T


def write(path, text, check, changed):
    old = open(path).read() if os.path.exists(path) else None
    if old != text:
        changed.append(os.path.relpath(path, HERE))
        if not check:
            with open(path, "w", newline="") as f:
                f.write(text)


def csv_text(head, rows):
    import io
    b = io.StringIO()
    w = csv.writer(b, lineterminator="\n")
    w.writerow(head)
    for r in rows:
        w.writerow(["" if r.get(k) is None else r[k] for k in head])
    return b.getvalue()


def r4(v):
    return None if v is None else round(v, 4)


def summary_rows(cols, fam, list_name):
    rows = []
    for c in cols:
        pr = c.paired("solved")
        t = fam["solved"].get(c.model)
        for arm in ("without", "with"):
            s = c.stats(arm)
            w, cw = s["solved_wilson"] or (None, None), s["clean_wilson"] or (None, None)
            rows.append({"list": list_name, "model": c.model, "arm": arm, "column_complete": int(c.complete),
                         "finished": s["finished"], "scored": s["scored"], "unscored": s["unscored"],
                         "not_finished": s["left"], "solved": s["solved"], "solved_wilson_low": r4(w[0]),
                         "solved_wilson_high": r4(w[1]), "clean": s["clean"], "clean_wilson_low": r4(cw[0]),
                         "clean_wilson_high": r4(cw[1]), "false_claims": s["false_claims"],
                         "mean_requests": r4(s["requests"]), "mean_steps": r4(s["steps"]),
                         "mean_wall_s": r4(s["wall_s"]), "mean_usd_list": r4(s["usd"]),
                         "usd_sum": r4(s["usd_sum"]), "mean_tokens_in": r4(s["tokens_in"]),
                         "mean_tokens_cached": r4(s["tokens_cached"]), "mean_tokens_out": r4(s["tokens_out"]),
                         "cached_share": r4(s["cached_share"]), "failed_requests": s["failed"],
                         "paired_cells": pr["pairs"], "with_only": pr["with_only"],
                         "without_only": pr["without_only"], "both": pr["both"], "neither": pr["neither"],
                         "sign_p_solved": pr["p"], "holm_threshold": None if not t else r4(t[0]),
                         "verdict_solved": verdict(pr, t, "solved") if c.complete else
                         f"incomplete at the read of {READ[list_name]}: {pr['pairs']} of {len(c.cells)}"})
    return rows


SUM_HEAD = ["list", "model", "arm", "column_complete", "finished", "scored", "unscored", "not_finished", "solved",
            "solved_wilson_low", "solved_wilson_high", "clean", "clean_wilson_low", "clean_wilson_high",
            "false_claims", "mean_requests", "mean_steps", "mean_wall_s", "mean_usd_list", "usd_sum",
            "mean_tokens_in", "mean_tokens_cached", "mean_tokens_out", "cached_share", "failed_requests",
            "paired_cells", "with_only", "without_only", "both", "neither", "sign_p_solved", "holm_threshold",
            "verdict_solved"]
PAIR_HEAD = ["list", "model", "measure", "column_complete", "paired_cells", "with_only", "without_only", "both",
             "neither", "difference", "newcombe_low", "newcombe_high", "sign_p", "holm_threshold", "passes_holm",
             "verdict"]


def paired_rows(cols, fam, list_name, settled=True):
    rows = []
    for c in cols:
        for key, _ in MEASURES:
            pr = c.paired(key)
            t = fam.get(key, {}).get(c.model)
            nc = pr["newcombe"] or (None, None)
            if not c.complete:
                v = f"incomplete at the read of {READ[list_name]}: {pr['pairs']} of {len(c.cells)}"
            elif not settled:
                v = "family not settled"
            elif key not in fam:
                v = "descriptive, no test declared"     # the grid's false claims: only the pooled test is declared
            else:
                v = verdict(pr, t, key)
            rows.append({"list": list_name, "model": c.model, "measure": key, "column_complete": int(c.complete),
                         "paired_cells": pr["pairs"], "with_only": pr["with_only"],
                         "without_only": pr["without_only"], "both": pr["both"], "neither": pr["neither"],
                         "difference": r4(pr["diff"]), "newcombe_low": r4(nc[0]), "newcombe_high": r4(nc[1]),
                         "sign_p": pr["p"], "holm_threshold": None if not t else r4(t[0]),
                         "passes_holm": None if not t else int(t[1]), "verdict": v})
    return rows


EARLIER = {"with": (0.615, 0.819), "without": (0.101, 0.276)}   # RULES.md section 3: the main test's statement
CLEAN_NEED = 63                          # RULES.md section 3: GPT-6 Astra CLEAN stands on at least 63 start states


def times(x):
    """A factor as the charts print it: two decimals under 1.1, one under 10, a whole number from 10."""
    return f"{x:.2f}" if x < 1.1 else f"{x:.1f}" if x < 10 else f"{x:.0f}"


def task_rows(c, measure, cells):
    """The task row as the unit: the start states of each task row counted together in each arm; the rows where the
    arm with the harness has more, fewer and the same; the exact sign test on more against fewer. Descriptive."""
    by = {}
    for x in cells:
        k = by.setdefault(x["task_row"], [0, 0])
        k[0] += c.val("with", x["cell"], measure)
        k[1] += c.val("without", x["cell"], measure)
    more = sum(1 for w, o in by.values() if w > o)
    fewer = sum(1 for w, o in by.values() if w < o)
    return {"rows": len(by), "more": more, "fewer": fewer, "tied": len(by) - more - fewer, "p": sign_p(more, fewer)}


def rows_txt(tr):
    return f"{tr['more']} task rows more, {tr['fewer']} fewer, {tr['tied']} the same, p = {fp(tr['p'])}"


def per_unsolved(s):
    return None if not s["unsolved"] else s["false_claims"] / s["unsolved"]


FINISH = ["tool_calls", "stop", "length", "content_filter", "other"]   # episodes.csv and unscored.csv: replies_<it>
STOP_STREAK = 3       # replies in a row with no tool call that end an episode (the operator's rule)


def finish(c, arm):
    """The arm's replies by finish reason: over its scored episodes (the scored attempt of each) and, apart, over
    every attempt of its unscored ones; the scored episodes that held a filtered reply (finish reason
    content_filter), those that ended on one, and the unsolved ones, claims and false claims among those."""
    a = c.arm[arm]
    sc, un = list(a["scored"].values()), list(a["unscored"].values())
    num = lambda rows, k: sum(int(r[k] or 0) for r in rows)
    out = {k: num(sc, f"replies_{k}") for k in FINISH}
    out["replies"] = sum(out[k] for k in FINISH)
    out["no_reply"] = num(sc, "requests_no_reply")
    out["episodes_filtered"] = sum(1 for e in sc if int(e["replies_content_filter"]))
    ended = [e for e in sc if int(e["content_filter_at_end"])]
    out["ended"] = ended
    out["ended_unsolved"] = sum(1 for e in ended if not int(e["solved"]))
    out["ended_claims"] = sum(int(e["claimed_success"]) for e in ended if not int(e["solved"]))
    out["ended_false_claims"] = sum(int(e["false_claim"]) for e in ended)
    out["unscored"] = {k: num(un, f"replies_{k}") for k in FINISH}
    out["unscored_replies"] = sum(out["unscored"].values())
    out["unscored_filtered_episodes"] = sum(1 for u in un if int(u["replies_content_filter"] or 0))
    out["filtered"] = out["content_filter"] + out["unscored"]["content_filter"]
    return out


def filtered_arms(cols):
    """[(column, arm, finish)] for every arm with a reply the provider's content filter stopped, scored or not,
    in the order of cols."""
    return [(c, arm, f) for c in cols for arm in ("without", "with") for f in [finish(c, arm)] if f["filtered"]]


def filtered_item(c, arm, f, name):
    """One arm's filtered replies, as "Read these first" and the finish-reason table name them."""
    s = []
    if f["content_filter"]:
        n = f["episodes_filtered"]
        s.append(f"{f['content_filter']:,} of its {f['replies']:,} replies in scored episodes, in {n} of its "
                 f"{c.stats(arm)['scored']} scored episodes, "
                 + (f"{len(f['ended'])} of which ended on filtered replies" if f["ended"] else
                    "none of which ended on one" if n > 1 else "which did not end on it"))
    if f["unscored"]["content_filter"]:
        s.append(f"{f['unscored']['content_filter']:,} of the {f['unscored_replies']:,} replies of its "
                 f"{c.stats(arm)['unscored']} unscored episodes, in {f['unscored_filtered_episodes']} of them")
    return f"{c.model} {arm} {name} (" + "; and ".join(s) + ")"


def false_claims_filtered(cols, name):
    """The false-claim figures read with the content filter, descriptive (no rule declared these readings): for each
    arm whose unsolved episodes ended on filtered replies, whether a claim from the model reached any of them
    (ended_by claim), and its rate per unsolved episode with and without those episodes beside the other arm's; then
    the model's two arms compared per unsolved episode twice, with each arm's such episodes counted and left out:
    when the two comparisons point opposite ways the comparison is read neither way, else the rates are printed with
    no verdict. Then the pooled rate over every model and without the models of those arms."""
    hit = [(c, arm, f) for c, arm, f in filtered_arms(cols) if f["ended_unsolved"]]
    if not hit:
        return []
    label = "(descriptive; no rule declared this reading)"

    def rate(c, arm):
        s, f = c.stats(arm), finish(c, arm)
        n = s["unsolved"] - f["ended_unsolved"]
        return (s["false_claims"] - f["ended_false_claims"]) / n if n else None

    def way(a, b):
        return None if a is None or b is None else (a > b) - (a < b)

    def fmt(v):
        return "no rate (no unsolved episode left)" if v is None else f"{v:.3f}"

    L = []
    for c, arm, f in hit:
        other = "with" if arm == "without" else "without"
        s, o, w, wo = c.stats(arm), c.stats(other), c.stats("with"), c.stats("without")
        left, fl = s["unsolved"] - f["ended_unsolved"], s["false_claims"] - f["ended_false_claims"]
        ended_un = [e for e in f["ended"] if not int(e["solved"])]
        by_model = sum(1 for e in ended_un if e["ended_by"] == "claim")
        whole = all(e["ended_by"] == "stopped_calling_tools" and int(e["content_filter_at_end"]) >= STOP_STREAK
                    for e in ended_un)
        raw = per_unsolved(w), per_unsolved(wo)              # with, without: every unsolved episode counted
        adj = rate(c, "with"), rate(c, "without")            # each arm's such unsolved episodes left out
        opposite = {way(*raw), way(*adj)} == {1, -1}
        L.append(f"- Read with the content filter {label}: {c.model} {arm} {name} ended {f['ended_unsolved']} of its "
                 f"{s['unsolved']} unsolved episodes on filtered replies, and "
                 + (("no claim from the model reached any of them"
                     + (": every reply that ended them came back content_filter, with no text and no tool call."
                        if whole else "."))
                    if not by_model else
                    f"{by_model} of those ended on the model's own claim, {f['ended_claims']} of them a claim of "
                    f"success.")
                 + f" Its false claims per unsolved episode: {s['false_claims']} of {s['unsolved']} "
                 f"({r3(s['false_claims'], s['unsolved'])}) with them counted, {fl} of the {left} "
                 f"that did not end that way ({r3(fl, left) or 'none left'}); {other} {name}, "
                 f"{o['false_claims']} of {o['unsolved']} ({r3(o['false_claims'], o['unsolved'])}). With {name} "
                 f"against without, {c.model} made {w['false_claims']} against {wo['false_claims']} false claims in "
                 f"all, and per unsolved episode {fmt(raw[0])} against {fmt(raw[1])} with them counted, "
                 f"{fmt(adj[0])} against {fmt(adj[1])} with them left out."
                 + (" The two readings point opposite ways, so the comparison measures the filter as much as the "
                    "model and is read neither way." if opposite else ""))
    out = {c.model for c, _, _ in hit}
    rest = [c for c in cols if c.model not in out]

    def pool(cs):
        fw, uw = sum(c.stats("with")["false_claims"] for c in cs), sum(c.stats("with")["unsolved"] for c in cs)
        fo, uo = sum(c.stats("without")["false_claims"] for c in cs), sum(c.stats("without")["unsolved"] for c in cs)
        return f"{fw} in {uw} ({r3(fw, uw)}) with {name} against {fo} in {uo} ({r3(fo, uo)}) without"

    hi = [c for c in cols if per_unsolved(c.stats("with")) is not None and per_unsolved(c.stats("without")) is not None
          and per_unsolved(c.stats("with")) >= per_unsolved(c.stats("without"))]
    hi2 = [c for c in cols if rate(c, "with") is not None and rate(c, "without") is not None
           and rate(c, "with") >= rate(c, "without")]
    M = len(cols)
    L.append(f"- The pooled rate per unsolved episode, over all {M} models: {pool(cols)}; with "
             f"{joined(sorted(out))} left out (both arms, so each pool holds the same {len(rest)} models): "
             f"{pool(rest)} {label}. With each arm's unsolved episodes that ended on filtered replies left out of its "
             f"count, the arm with {name} claims falsely as often or more for {len(hi2)} of the {M} models {label}, "
             f"against {len(hi)} with them counted"
             + ("; most, either way" if 2 * len(hi) > M and 2 * len(hi2) > M else
                "; not most, either way" if 2 * len(hi) <= M and 2 * len(hi2) <= M else
                "; the two counts fall on either side of most, and the rule reads the count with them") + ".")
    return L


def refused_note(cols, refused):
    """Where the requests of refusals.csv (HTTP 400 from the provider's image filter) are counted: a refused request
    got no reply, so it is a request with no reply in the scored attempt (episodes.csv) or in an unscored episode's
    attempts (unscored.csv), and in no count when its attempt is in no row. Empty when the list has no refusal."""
    if not refused:
        return ""
    by = {c.model: c for c in cols}
    n = lambda k, word: f"{k:,} {word}{'' if k == 1 else 's'}"
    groups = {"scored": [], "unscored": [], "other": []}
    for r in refused:
        a = by[r["model"]].arm[r["arm"]]
        groups["scored" if int(r["scored_attempt"]) else "unscored" if r["cell"] in a["unscored"] else
               "other"].append(r)
    q = {k: sum(int(r["refused_requests"]) for r in rs) for k, rs in groups.items()}
    parts = []
    if groups["scored"]:
        parts.append(f"{n(q['scored'], 'request')} in {n(len(groups['scored']), 'scored attempt')} are in the "
                     f"table's requests with no reply")
    if groups["unscored"]:
        eps = len({(r["model"], r["arm"], r["cell"]) for r in groups["unscored"]})
        parts.append(f"{n(q['unscored'], 'request')} in {n(len(groups['unscored']), 'attempt')} of "
                     f"{n(eps, 'unscored episode')} are in their requests_no_reply in unscored.csv")
    if groups["other"]:
        parts.append(f"{n(q['other'], 'request')} in {n(len(groups['other']), 'other attempt')} of episodes scored "
                     f"on a later attempt are counted nowhere as requests with no reply (lost_attempts.csv lists those "
                     f"attempts)")
    return (f" The provider's image filter answered {n(sum(q.values()), 'request')} in "
            f"{n(len(refused), 'attempt')} with HTTP 400 (refusals.csv). A refused request got no reply, so none is "
            f"a content_filter reply: " + "; ".join(parts) + ".")


def finish_table(cols, name, refused=()):
    """Every arm's replies by finish reason, over its scored episodes, with its requests that got no reply; with the
    list's refusals.csv rows, where the requests the image filter refused are counted."""
    T = ["| model | arm | replies | tool_calls | stop | length | content_filter | other | requests with no reply | "
         "scored episodes with a content_filter reply | ended on content_filter replies |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in cols:
        for arm in ("without", "with"):
            f = finish(c, arm)
            T.append(f"| {c.model}{'' if c.complete else '*'} | {arm} {name} | {f['replies']:,} | "
                     + " | ".join(f"{f[k]:,}" for k in FINISH)
                     + f" | {f['no_reply']:,} | {f['episodes_filtered']} | {len(f['ended'])} |")
    fa = filtered_arms(cols)
    un = [(c, arm, f) for c, arm, f in fa if f["unscored"]["content_filter"]]
    T += ["", "A reply's finish reason is the one its model endpoint returned: `tool_calls` a reply with a tool call, "
          "`stop` and `length` one that ended on its own or at the output limit, `content_filter` one the provider's "
          "content filter stopped, `other` any other. Every reply of each scored attempt is counted, so the five "
          "reasons add up to the replies. A request with no reply is a failed request (sent again) or one the "
          "endpoint refused for a request field it does not take (sent again without the field).",
          "",
          (f"{len(fa)} of the {2 * len(cols)} arms had a content_filter reply: "
           + "; ".join(filtered_item(c, arm, f, name) for c, arm, f in fa) + "." if fa else
           "No reply came back content_filter in any arm.")
          + (" The unscored episodes' replies are in unscored.csv, per episode." if un else "")
          + refused_note(cols, refused), ""]
    return T


def clean_tally(done, name):
    ahead = sum(1 for c in done if c.stats("with")["clean"] > c.stats("without")["clean"])
    behind = [c for c in done if c.stats("with")["clean"] < c.stats("without")["clean"]]
    return (f" On CLEAN, with {name} more in {ahead}, fewer in {len(behind)}, the same number in "
            f"{len(done) - ahead - len(behind)}."
            + (" Fewer: " + "; ".join(f"{c.model} ({c.stats('with')['clean']} against {c.stats('without')['clean']}, "
                                      f"p = {fp(c.paired('clean')['p'])})" for c in behind) + "." if behind else ""))


def said_first(cells, name):
    """RULES.md section 2, word for word, and the lines the rule's write-up carries with it: what the start states
    and the two arms are, said before any number."""
    R = len({x["task_row"] for x in cells})
    return ["Said before any number (RULES.md section 2):", "",
            f"1. They are new start states of {R} task rows that the {name} harness was developed on at other start "
            f"states. The tasks themselves are not new to it. Task rows that the {name} harness never met exist in "
            f"the benchmark and are on no list here.",
            f"2. The result is on one benchmark family, the one the {name} harness was built and checked on.",
            f"3. The arm with {name} reads a record of the robot's own calibration that the other arm does not have. "
            f"The comparison is the whole package against the published interface; it cannot be split into tools, "
            f"record and documents.",
            "4. One episode a start state at temperature 1.0. Provider model versions are known only by the model's "
            "name.",
            "5. Other groups publish higher rates on this benchmark under their own protocols. Nothing here supports "
            "a ranking against any of them.", "",
            f"With them: the arm without {name} is the benchmark's published minimal interface, not the best harness "
            f"there is. The calibration records were made by the robot's own calibration episodes on development "
            f"start states of LIBERO-10 and LIBERO-PRO, and which record each episode read is not recorded. The arm "
            f"without {name} reads the gripper in one of the two ways the published paper allows, the one that scored "
            f"lower when both ran in an earlier check on LIBERO-10 (not in this package); the other way was not run "
            f"here, so a small gain over this arm is not shown to hold against it. CLEAN was defined before the first "
            f"episode with {name} on the earlier lists, and became a claim only after the first of those lists had "
            f"shown a gain on it; for this grid it was declared before the first episode.", ""]


def main_passes(pr):
    """A declared test passes: exact p at or under ALPHA, in the direction of the arm with the harness."""
    return pr["p"] is not None and pr["p"] <= ALPHA and pr["with_only"] > pr["without_only"]


def main_statement(mc, pr, sw, so, name, N, cells):
    """RULES.md section 3, the main test's row: the statement stands only if the test passes AND the rate with the
    harness overlaps the earlier interval AND the rate without overlaps its earlier interval. If the test passes
    with a rate outside the earlier interval, the earlier draws were lucky. If it fails, there is no solve
    statement."""
    sw_iv, so_iv = sw["solved_wilson"], so["solved_wilson"]
    over = lambda iv, e: iv is not None and iv[0] <= e[1] and e[0] <= iv[1]
    ow, oo = over(sw_iv, EARLIER["with"]), over(so_iv, EARLIER["without"])
    need = (f"Read with it: the statement it carries (RULES.md section 3) needs the main test to pass, the rate with "
            f"{name} to overlap the earlier 61.5 to 81.9 percent and the rate without to overlap 10.1 to 27.6 percent "
            f"(here {pct(sw_iv)} and {pct(so_iv)}). ")
    rows = f" With the task row as the unit (descriptive): {rows_txt(task_rows(mc, 'solved', cells))}."
    ratio = (f", {times(sw['solved'] / so['solved'])} times as many" if so["solved"] else "")
    if not main_passes(pr):
        return (need + f"The main test does not pass, so the statement is printed as two earlier lists that did not "
                f"replicate a third time, and it is not the headline; there is no solve statement (RULES.md section "
                f"5 (6)). The counts: {sw['solved']} against {so['solved']} of {N}." + rows)
    if ow and oo:
        return (need + f"The test passes and both overlap, so the statement stands, with these counts: "
                f"{sw['solved']} against {so['solved']} of {N}{ratio}." + rows)
    miss = [w for w, ok in (("the rate with " + name, ow), ("the rate without", oo)) if not ok]
    verb = "does not overlap its earlier interval" if len(miss) == 1 else "do not overlap their earlier intervals"
    return (need + f"The test passes, but {joined(miss)} {verb}, so, in the rule's words, "
            f"the earlier draws were lucky; the statement carries these counts and the two earlier lists beside "
            f"them: {sw['solved']} against {so['solved']} of {N}{ratio}." + rows)


def grid_first(cols, name, fam, N, cells, uns, lost):
    """What must be read before any number of the grid, each line computed from the CSV files. RULES.md section 5
    (1): a column against the harness is named with its counts in the first paragraph."""
    M = len(cols)
    L = ["## Read these first", ""]
    st = {(c.model, arm): c.stats(arm) for c in cols for arm in ("without", "with")}
    # fewer with the harness, SOLVED and CLEAN, and the tally printed whole
    tally = []
    for key, word in MEASURES[:2]:
        behind = sorted((c for c in cols if st[c.model, "with"][key] < st[c.model, "without"][key]),
                        key=lambda c: (-st[c.model, "without"][key], c.model))
        ahead = sum(1 for c in cols if st[c.model, "with"][key] > st[c.model, "without"][key])
        tally.append(f"on {word} in {ahead} of {M}" + (f" (level in {M - ahead - len(behind)})"
                                                         if M - ahead - len(behind) else ""))
        if not behind:
            continue
        top = sorted(cols, key=lambda c: -st[c.model, "without"][key])
        highest = (len(top) > len(behind) and {c.model for c in top[:len(behind)]} == {c.model for c in behind}
                   and st[top[len(behind) - 1].model, "without"][key] > st[top[len(behind)].model, "without"][key])
        items, words = [], set()
        for c in behind:
            pr = c.paired(key)
            words.add(verdict(pr, fam[key].get(c.model), key))
            items.append(f"{c.model} ({st[c.model, 'with'][key]} against {st[c.model, 'without'][key]} of {N}; "
                         f"with only {pr['with_only']}, without only {pr['without_only']}; p = {fp(pr['p'])})")
        n = len(behind)
        lead = (f"With {name}, {n} of the {M} models solved fewer start states than without it"
                + (f", the {n} with the highest rates without it" if highest and n > 1 else "") if key == "solved"
                else f"On CLEAN, {n} of the {M} models had fewer with {name}")
        tail = ((" None of them is significant at 0.05, so each reads" if n > 1 else
                 " It is not significant at 0.05, so it reads") + " \"no difference shown\" (RULES.md section 5 (3))."
                if words == {"no difference shown"} else
                " Their readings: " + "; ".join(f"{c.model} {verdict(c.paired(key), fam[key].get(c.model), key)}"
                                               for c in behind) + ".")
        L.append(f"- {lead}: {'; '.join(items)}.{tail}")
    L.append(f"- With {name} ahead, printed whole: " + ", and ".join(tally) + " models.")
    # the tally is counts, not verdicts: how many pass Holm, and every model ahead that does not
    est, short = [], []
    for key, word in MEASURES[:2]:
        k = sum(1 for c in cols if verdict(c.paired(key), fam[key].get(c.model), key) == "established")
        est.append(f"on {word} for {k} of {M}")
        for c in cols:
            pr, t = c.paired(key), fam[key].get(c.model)
            v = verdict(pr, t, key)
            if st[c.model, "with"][key] > st[c.model, "without"][key] and v != "established":
                short.append(f"{c.model} {word} ({st[c.model, 'with'][key]} against {st[c.model, 'without'][key]} of "
                             f"{N}; with only {pr['with_only']}, without only {pr['without_only']}; p = {fp(pr['p'])}"
                             + (f" against its Holm threshold {t[0]:.4f}" if t else "") + f"): {v}")
    L.append(f"- Established after Holm's correction over the {M} models (RULES.md section 3): " + ", and ".join(est)
             + "." + (f" Ahead with {name} but not established: " + "; ".join(short) + "." if short else "")
             + f" Only a model established on SOLVED may be read as \"{name} lifts\" it.")
    # the replies the provider's content filter stopped: every arm with one, with its counts
    fa = filtered_arms(cols)
    if fa:
        L.append(f"- The provider's content filter stopped replies (finish reason content_filter) in {len(fa)} of the "
                 f"{2 * M} arms: " + "; ".join(filtered_item(c, arm, f, name) for c, arm, f in fa)
                 + ". No other arm had one. Every arm's replies by finish reason are in the table under \"How the "
                 "replies ended\".")
    # false claims: the pooled test beside the rate per unsolved episode
    fw = sum(st[c.model, "with"]["false_claims"] for c in cols)
    fo = sum(st[c.model, "without"]["false_claims"] for c in cols)
    uw = sum(st[c.model, "with"]["unsolved"] for c in cols)
    uo = sum(st[c.model, "without"]["unsolved"] for c in cols)
    a = sum(c.paired("false_claim")["without_only"] for c in cols)
    b = sum(c.paired("false_claim")["with_only"] for c in cols)
    pooled = sign_p(a, b)
    hi = [c for c in cols if per_unsolved(st[c.model, "with"]) is not None
          and per_unsolved(st[c.model, "without"]) is not None
          and per_unsolved(st[c.model, "with"]) >= per_unsolved(st[c.model, "without"])]
    passes = pooled is not None and pooled <= ALPHA and a > b
    most = 2 * len(hi) > M                # RULES.md section 3: the statement narrows if this holds for most models
    rw, ro = (fw / uw if uw else 0.0), (fo / uo if uo else 0.0)
    L.append(f"- The pooled false-claim test (the second secondary test below) "
             + (("passes: fewer episodes ended with the model wrongly saying it had succeeded. " if not most else
                 "passes, and the rule narrows what it supports to \"fewer because it solves more\". ") if passes else
                "does not pass, so the statement on fewer false claims does not stand. ")
             + f"Per unsolved episode the rate is {'higher' if rw > ro else 'not higher'} with {name}: {fw} false "
             f"claims in {uw} unsolved episodes ({rw:.3f}) against {fo} in {uo} ({ro:.3f}) without it, over all {M} "
             f"models; as high or higher with {name} for {len(hi)} of the {M}"
             + (" (with against without): " + "; ".join(f"{c.model} {per_unsolved(st[c.model, 'with']):.3f} against "
                                 f"{per_unsolved(st[c.model, 'without']):.3f}" for c in hi) if hi else "") + ".")
    more = sorted((c for c in cols if st[c.model, "with"]["false_claims"] > st[c.model, "without"]["false_claims"]),
                  key=lambda c: (st[c.model, "without"]["false_claims"] - st[c.model, "with"]["false_claims"],
                                 c.model))
    if more:
        L.append(f"- With {name} the model made more false claims in all for {len(more)} of the {M} models: "
                 + "; ".join(f"{c.model} ({st[c.model, 'with']['false_claims']} against "
                             f"{st[c.model, 'without']['false_claims']}; "
                             f"with only {c.paired('false_claim')['with_only']}, "
                             f"without only {c.paired('false_claim')['without_only']}; "
                             f"p = {fp(c.paired('false_claim')['p'])})" for c in more)
                 + ". Each per-model count is descriptive; the declared test is the pooled one.")
    L += false_claims_filtered(cols, name)
    # episodes in which the model stopped acting, and episodes that ran no control step
    for c in cols:
        for arm in ("without", "with"):
            sc = list(c.arm[arm]["scored"].values())
            stop = [e for e in sc if e["ended_by"] == "stopped_calling_tools"]
            if 10 * len(stop) <= N:
                continue
            zero = sum(1 for e in stop if int(e["steps"]) == 0)
            unsolved = st[c.model, arm]["unsolved"]
            stop_un = sum(1 for e in stop if not int(e["solved"]))
            ends = [int(e["content_filter_at_end"]) for e in stop]
            whole = sum(1 for k in ends if k >= STOP_STREAK)
            line = (f"- {c.model} {arm} {name} ended {len(stop)} of its {N} episodes on {STOP_STREAK} replies in a "
                    f"row with no tool call, which end an episode (ended_by stopped_calling_tools), {zero} of them "
                    f"before any control step.")
            some = sum(1 for k in ends if k)
            if some:
                short = sorted({k for k in ends if k})
                line += (f" {'All' if some == len(stop) else f'{some} of those'} {len(stop)} ended on content_filter "
                         f"replies"
                         + (f": in {'every one' if whole == some else f'{whole} of them'}, the {STOP_STREAK} replies "
                            f"with no tool call that ended it were all ones the provider's content filter stopped."
                            if whole else
                            f", but in each only the last {' or '.join(str(k) for k in short)} of the {STOP_STREAK} "
                            f"replies with no tool call that ended it came back content_filter."))
            if arm == "without" and 2 * stop_un > unsolved:
                line += (f" {stop_un} of the {unsolved} start states it did not solve without {name} ended that way, "
                         f"so its gain on SOLVED ({st[c.model, 'with']['solved']} against "
                         f"{st[c.model, 'without']['solved']}) is mostly against an arm that "
                         + ("the content filter stopped." if whole * 2 > len(stop) else "stopped acting."))
            L.append(line)
    zero = {}
    flagged = zero_ok = 0
    for c in cols:
        for arm in ("without", "with"):
            for e in c.arm[arm]["scored"].values():
                z, p = int(e["steps"]) == 0, int(e["problems"]) > 0
                flagged += p
                if z:
                    zero[c.model, arm] = zero.get((c.model, arm), 0) + 1
                    zero_ok += p and not int(e["solved"]) and not int(e["false_claim"])
    nz = sum(zero.values())
    if nz:
        L.append(f"- {nz} scored episodes ran no control step: "
                 + ", ".join(f"{m} {arm} {name} {k}" for (m, arm), k in zero.items()) + "."
                 + (" Each carries a problem row and none was solved or is a false claim, so no count of solved, "
                    "clean or false claims changes with them." if zero_ok == nz == flagged else
                    f" {flagged} scored episodes carry a problem row."))
    # requests, steps, cost and time
    notless = [c for c in cols if st[c.model, "with"]["requests"] >= st[c.model, "without"]["requests"]
               or st[c.model, "with"]["steps"] >= st[c.model, "without"]["steps"]]
    if notless:
        L.append(f"- With {name}, {len(notless)} of the {M} models did not use fewer model replies and control steps "
                 f"per scored episode than without it: "
                 + "; ".join(f"{c.model} ({st[c.model, 'with']['requests']:.1f} against "
                             f"{st[c.model, 'without']['requests']:.1f} replies, {st[c.model, 'with']['steps']:.1f} "
                             f"against {st[c.model, 'without']['steps']:.1f} control steps)" for c in notless) + ".")
    for c in cols:
        w, o = st[c.model, "with"], st[c.model, "without"]
        if w["usd"] > o["usd"] or w["wall_s"] > o["wall_s"]:
            parts = []
            if w["usd"] > o["usd"]:
                parts.append(f"cost more at list price ({w['usd']:.2f} against {o['usd']:.2f} USD an episode)")
            if w["wall_s"] > o["wall_s"]:
                parts.append(f"took longer ({w['wall_s']:.1f} against {o['wall_s']:.1f} wall seconds)")
            per = ""
            if w["usd_per_solved"] is not None and o["usd_per_solved"] is not None:
                per = (f"; a success cost {'less' if w['usd_per_solved'] < o['usd_per_solved'] else 'more'} "
                       f"({w['usd_per_solved']:.2f} against {o['usd_per_solved']:.2f} USD)")
            line = f"- With {c.model}, an episode with {name} " + " and ".join(parts) + per + "."
            if w["unscored"] or o["unscored"]:
                aw, ao = all_in(c, "with", uns, lost), all_in(c, "without", uns, lost)
                figs = []
                walls = aw["wall_s"] is not None and ao["wall_s"] is not None
                if walls:
                    figs.append(f"{aw['wall_s']:.1f} against {ao['wall_s']:.1f} wall seconds")
                figs.append(f"{aw['usd']:.2f} against {ao['usd']:.2f} USD an episode")
                if aw["usd_per_solved"] is not None and ao["usd_per_solved"] is not None:
                    figs.append(f"{aw['usd_per_solved']:.2f} against {ao['usd_per_solved']:.2f} USD a success")
                line += (f" These means are over scored episodes, and {w['unscored']} of its episodes with {name} "
                         f"and {o['unscored']} without ended without a score. Counted too"
                         + (f" (an episode the 90 minute clock ended at its {CLOCK_S} seconds)" if walls else "")
                         + ", with every attempt in no row at its cost (lost_attempts.csv): " + joined(figs) + ".")
            L.append(line)
    # the column added late, and the passes that do not hold with the task row as the unit
    for m in ADDED_LATE:
        c = next((x for x in cols if x.model == m), None)
        t = fam["solved"].get(m)
        if c and t:
            L.append(f"- {m} was added to the grid after its first rows had been seen. It "
                     f"{'passes' if t[1] else 'does not pass'} SOLVED at p = {fp(c.paired('solved')['p'])} against its "
                     f"Holm threshold {fp(t[0])}.")
    weak = []
    for key, word in MEASURES[:2]:
        for c in cols:
            t = fam[key].get(c.model)
            tr = task_rows(c, key, cells)
            if t and t[1] and (tr["p"] is None or tr["p"] > ALPHA):
                weak.append(f"{c.model} {word} ({rows_txt(tr)})")
    if weak:
        R = len({x["task_row"] for x in cells})
        L.append(f"- With the task row as the unit (the start states of each of the {R} task rows counted together "
                 f"in each arm; descriptive, no test is declared on it), {len(weak)} of the passes below do not reach "
                 f"0.05: " + "; ".join(weak) + ".")
    # the cross-model pairs named before the grid
    pairs = cross_pairs(cols, "solved")
    if pairs:
        L.append(f"- The cross-model pairs named before the grid (RULES.md section 3), SOLVED: "
                 + "; ".join(f"{MAIN_MODEL} with {name} against {x['other']} without, {x['a']} against {x['b']} of "
                             f"{N} (first only {x['pr']['with_only']}, second only {x['pr']['without_only']}, "
                             f"p = {fp(x['pr']['p'])}): {x['wording']}"
                             + (f", {100 * x['pr']['diff']:+.1f} points" if x["a"] < x["b"] else "") for x in pairs)
                 + ". Any other such pair found in the grid is exploratory.")
    # unscored episodes, and the families on scored pairs only
    if uns:
        why = {}
        for u in uns:
            why[u["reason"]] = why.get(u["reason"], 0) + 1
        nw = sum(1 for u in uns if u["arm"] == "without")
        famsc = {key: holm({c.model: c.paired(key, scored_only=True)["p"] for c in cols})
                 for key in ("solved", "clean")}
        moved = [f"{c.model} {word}" for key, word in MEASURES[:2] for c in cols
                 if bool((fam[key].get(c.model) or (0, False))[1]) != bool((famsc[key].get(c.model) or (0, False))[1])]
        L.append(f"- {len(uns)} episodes ended without a score ({nw} without {name}, {len(uns) - nw} with), each "
                 f"counted as a failure for its arm: " + "; ".join(f"{k} because {r}" for r, k in why.items()) + ". "
                 + ("On scored pairs only (start states scored in both arms) the same models pass Holm on SOLVED and "
                    "on CLEAN." if not moved else
                    "On scored pairs only (start states scored in both arms) these readings change: "
                    + "; ".join(moved) + "."))
    L += ["- Each start state ran once in each arm, except that the four Claude columns with Cerulion were drawn a "
          "second time on 2026-11-19, with another picture mode and code snapshot (README.md, \"How the episodes "
          "ran\"); this package holds that second draw. How much the outcome of one start state varies from run to "
          "run under the same settings was not measured.",
          "- How the run went, and where it differed from the protocol (the second draw of the four Claude columns "
          "with Cerulion, the pause of the Claude models, the start "
          "states whose two arms started more than an hour apart, two machines that stopped and a third whose "
          "waiting episodes were moved, a network outage, one model answered from two provider deployments, the "
          "machines shared with other work), is in README.md under \"Things to know before quoting a number\".", ""]
    return L


def cross_pairs(cols, key):
    """RULES.md section 3: MAIN_MODEL with the harness against each model of CROSS_WITHOUT without it, paired by start
    state. Descriptive, outside the corrected family; worded "no difference shown" or "differs"."""
    by = {c.model: c for c in cols}
    m = by.get(MAIN_MODEL)
    out = []
    for other in CROSS_WITHOUT:
        o = by.get(other)
        if m is None or o is None or not (m.complete and o.complete):
            continue
        pr = m.paired(key, other=o)
        out.append({"other": other, "pr": pr, "a": m.stats("with")[key], "b": o.stats("without")[key],
                    "n_a": m.stats("with")["finished"], "n_b": o.stats("without")["finished"],
                    "wording": "no difference shown" if pr["p"] is None or pr["p"] > ALPHA else
                    f"differs (p = {fp(pr['p'])}; a pair named beforehand, outside the corrected family)"})
    return out


def cross_table(cols, name):
    T = []
    for key, word in MEASURES[:2]:
        for x in cross_pairs(cols, key):
            pr = x["pr"]
            T.append(f"| {word} | {x['other']} without {name} | {x['a']} of {x['n_a']} "
                     f"({pct(wilson(x['a'], x['n_a']))}) | {x['b']} of {x['n_b']} ({pct(wilson(x['b'], x['n_b']))}) | "
                     f"{100 * pr['diff']:+.1f} ({pct(pr['newcombe'])}) | {pr['with_only']} | {pr['without_only']} | "
                     f"{fp(pr['p'])} | {x['wording']} |")
    if not T:
        return []
    return ([f"## The cross-model pairs named before the grid: {MAIN_MODEL} with {name} against "
             + " and against ".join(f"{m} without" for m in CROSS_WITHOUT) + " (descriptive, outside the corrected "
             "family)", "",
             f"| measure | against | {MAIN_MODEL} with {name} (Wilson 95, percent) | the other model without {name} "
             "(Wilson 95, percent) | difference, points (Newcombe 95) | first only | second only | exact p | wording |",
             "|---|---|---|---|---|---|---|---|---|"] + T
            + ["", f"RULES.md section 3 names these two pairs, and only these, before the grid was read: any other "
               f"pair of a cheaper model with {name} against a dearer model without it found in these rows is "
               f"exploratory. \"No difference shown\" does not mean the two were shown equal.", ""])


def grid(name, check, changed):
    cells, cols, uns = columns(HERE)
    if not cols:
        return None
    lost = load(HERE, "lost_attempts.csv")
    N = len(cells)
    info = json.load(open(os.path.join(HERE, "list_info.json")))
    READ["grid"] = info["read"]
    fam = {key: holm({c.model: c.paired(key)["p"] for c in cols}) for key in ("solved", "clean")}
    done = [c for c in cols if c.complete]
    write(os.path.join(HERE, "summary.csv"), csv_text(SUM_HEAD, summary_rows(cols, fam, "grid")), check, changed)
    write(os.path.join(HERE, "paired.csv"), csv_text(PAIR_HEAD, paired_rows(cols, fam, "grid")), check, changed)
    L = [f"# Summary of the grid: {N} start states, {len(cols)} models, both arms", "",
         f"Computed by recompute.py from episodes.csv, unscored.csv and not_finished.csv as read at {info['read']}. "
         f"recompute.py computes this file again from the CSV files; with --check it writes nothing and says "
         f"whether this file still matches them.", ""]
    L += said_first(cells, name)
    final = len(done) == len(cols)
    if final:
        L += [f"Every model has finished all {N} start states in both arms, so every verdict below is settled.", ""]
    else:
        L += [f"THE GRID WAS STILL RUNNING AT THIS READ. {len(done)} of {len(cols)} models had finished all {N} "
              f"start states in both arms. A model marked * is incomplete: its counts are of the start states that "
              f"had ended, with n printed, and it gets no verdict. Start states that end first are the short "
              f"episodes, so an incomplete row is not a fair sample of its list. Incomplete: "
              + "; ".join(f"{c.model} ({c.stats('without')['finished']} without, {c.stats('with')['finished']} with)"
                          for c in cols if not c.complete) + ".", ""]
    L += [f"Arms: `without` is the same model on a published three-tool interface; `with` is the same model on "
          f"{name}'s tools. Same start states, same budgets (80 replies, one tool call a reply, 900 control steps), "
          f"one episode a start state an arm. An episode that ended without a score counts as a failure for its arm "
          f"and stays in the count.", ""]
    if final:
        L += grid_first(cols, name, fam, N, cells, uns, lost)
    # the main test
    L += ["## The main test, as declared before the first episode", "",
          f"Declared: {MAIN_MODEL}, SOLVED within the 900-step budget, with {name} against without, paired by start "
          f"state on the grid's {N} start states, exact two-sided sign test on the discordant start states, alpha "
          f"0.05, no correction. One test.", ""]
    mc = next((c for c in cols if c.model == MAIN_MODEL), None)
    if mc:
        pr, sw, so = mc.paired("solved"), mc.stats("with"), mc.stats("without")
        state = "complete" if mc.complete else f"INCOMPLETE ({pr['pairs']} of {N} paired)"
        L += [f"Result ({state}): with {name} {sw['solved']} of {sw['finished']} ({pct(sw['solved_wilson'])} "
              f"percent), without {so['solved']} of {so['finished']} ({pct(so['solved_wilson'])} percent). Solved "
              f"with only: {pr['with_only']}. Solved without only: {pr['without_only']}. Both: {pr['both']}. "
              f"Neither: {pr['neither']}. Exact two-sided sign test p = {fp(pr['p'])}. Difference "
              f"{100 * pr['diff']:.1f} points (Newcombe 95 for paired rates: {pct(pr['newcombe'])}).", "",
              ("The test passes at 0.05." if main_passes(pr) else "The test does not pass at 0.05.")
              if mc.complete else "No verdict before the column is complete.", ""]
        if final:
            L += [main_statement(mc, pr, sw, so, name, N, cells), ""]
    # the two declared secondary tests
    L += ["## The two secondary tests, declared with it (alpha 0.05 each, apart from the per-model family)", ""]
    ac = next((c for c in cols if c.model == CLEAN_MODEL), None)
    if ac:
        pr, sw, so = ac.paired("clean"), ac.stats("with"), ac.stats("without")
        ps = ac.paired("solved")
        L += [f"1. {CLEAN_MODEL}, CLEAN (solved at or before the suite's published step limit, with no protocol "
              f"break, no problem row and no false claim){'' if ac.complete else ' (INCOMPLETE)'}: with {name} "
              f"{sw['clean']} of {sw['finished']} ({pct(sw['clean_wilson'])}), without {so['clean']} of "
              f"{so['finished']} ({pct(so['clean_wilson'])}); with only {pr['with_only']}, without only "
              f"{pr['without_only']}, p = {fp(pr['p'])}. Its SOLVED counts, to be read with it: {sw['solved']} "
              f"against {so['solved']} (with only {ps['with_only']}, without only {ps['without_only']}, "
              f"p = {fp(ps['p'])}).", ""]
        if final:
            cpass = main_passes(pr)
            against = pr["p"] is not None and pr["p"] <= ALPHA and pr["with_only"] < pr["without_only"]
            if cpass and N >= CLEAN_NEED:
                means = (f" What it supports: with {CLEAN_MODEL}, {name} did "
                         f"{'not ' if sw['solved'] <= so['solved'] else ''}solve more start states ({sw['solved']} "
                         f"against {so['solved']} of {N}); more of its solves came inside the published step limit "
                         f"({sw['clean']} against {so['clean']}, p = {fp(pr['p'])}).")
            elif cpass:
                means = (f" That is fewer start states than the rule asks for, so the statement that more solves come "
                         f"inside the published step limit does not stand.")
            elif against:
                means = f" It comes in against {name}, and RULES.md section 5 applies."
            else:
                means = (" Not significant, so in the rule's words the statement becomes \"no solve gain; a step-limit "
                         "gain not confirmed\".")
            L += [f"   {'The test passes at 0.05' if cpass else 'The test does not pass at 0.05'}, on {N} start "
                  f"states (the rule asks for at least {CLEAN_NEED})." + means
                  + f" With the task row as the unit (descriptive): {rows_txt(task_rows(ac, 'clean', cells))}.", ""]
    a = sum(c.paired("false_claim")["without_only"] for c in cols)
    b = sum(c.paired("false_claim")["with_only"] for c in cols)
    L += [f"2. Fewer false claims, pooled over every model that ran both arms, one sign test over all discordant "
          f"pairs of (model, start state){'' if len(done) == len(cols) else ' (INCOMPLETE: it pools the pairs ended so far)'}: "
          f"a false claim without {name} only on {a} pairs, with {name} only on {b} pairs, p = {fp(sign_p(a, b))}. "
          f"The rate per unsolved episode is in the table below for each model and arm.", ""]
    if final:
        fpass = sign_p(a, b) is not None and sign_p(a, b) <= ALPHA and a > b
        a2 = sum(c.paired("false_claim", scored_only=True)["without_only"] for c in cols)
        b2 = sum(c.paired("false_claim", scored_only=True)["with_only"] for c in cols)
        tot = {arm: sum(c.stats(arm)["false_claims"] for c in cols) for arm in ("without", "with")}
        eps = {arm: sum(c.stats(arm)["finished"] for c in cols) for arm in ("without", "with")}
        hi = sum(1 for c in cols if per_unsolved(c.stats("with")) is not None
                 and per_unsolved(c.stats("without")) is not None
                 and per_unsolved(c.stats("with")) >= per_unsolved(c.stats("without")))
        most = 2 * hi > len(cols)
        L += [f"   {'The test passes at 0.05' if fpass else 'The test does not pass at 0.05'}. False claims in all: "
              f"{tot['without']} of {eps['without']} episodes without {name}, {tot['with']} of {eps['with']} with it. "
              f"On scored pairs only: without only {a2}, with only {b2}, p = {fp(sign_p(a2, b2))}."
              + ((" What it supports: " + ("fewer episodes ended with the model wrongly saying it had succeeded"
                                           if not most else "fewer false claims because it solves more")
                  + f"; not that {name} stops false claims. The rule narrows the statement to \"fewer because it "
                  f"solves more\" if, per unsolved episode, the arm with {name} claims falsely as often or more for "
                  f"most models: it does for {hi} of the {len(cols)}, "
                  + ("which is not most, so the statement stands as written." if not most else
                     "which is most, so the statement narrows.")
                  + " The rates are under \"Read these first\" and in the table below.") if fpass else
                 f" The statement that fewer episodes end on a false claim with {name} does not stand."), ""]
    # per model
    L += ["## Every model, both arms", ""] + arm_table(cols, name, N) + [""]
    L += ["Wilson intervals are over the finished start states of the arm (scored plus unscored). A false claim is "
          "an episode that ends with the model saying it succeeded on a start state that was not solved; per "
          "unsolved episode divides them by the arm's finished episodes that were not solved.", ""]
    L += [f"## Paired by start state, and the per-model family ({len(cols)} models, Holm's step-down at 0.05)", "",
          "The family is every model that ran, the main test's model included, however each came out; the main "
          "test above is read alone at 0.05 and does not depend on this table. Thresholds are computed from the "
          "p values of all the models"
          + ("; every column is complete, so they are final." if final else
             " as they stand, so they can move until the last column completes. Until then the verdict printed for "
             "a complete column can change with them."), ""]
    for key, word in MEASURES[:2]:
        L += [f"### {word}", "",
              "| model | paired start states | with only | without only | both | neither | difference, points "
              "(Newcombe 95) | exact p | Holm threshold | verdict |", "|---|---|---|---|---|---|---|---|---|---|"]
        for c in cols:
            pr, t = c.paired(key), fam[key].get(c.model)
            v = verdict(pr, t, key) if c.complete else f"incomplete at the read of {info['read']}: {pr['pairs']} of {N}"
            if c.model in ADDED_LATE:
                v += " (column added after the first rows were seen)"
            d = "" if pr["diff"] is None else f"{100 * pr['diff']:+.1f} ({pct(pr['newcombe'])})"
            L.append(f"| {c.model}{'' if c.complete else '*'} | {pr['pairs']} | {pr['with_only']} | "
                     f"{pr['without_only']} | {pr['both']} | {pr['neither']} | {d} | {fp(pr['p'])} | "
                     f"{'' if not t else format(t[0], '.4f')} | {v} |")
        L.append("")
    ahead = sum(1 for c in done if c.stats("with")["solved"] > c.stats("without")["solved"])
    behind = [c for c in done if c.stats("with")["solved"] < c.stats("without")["solved"]]
    tied = len(done) - ahead - len(behind)
    L += [f"Tally over the {len(done)} complete columns, printed whole: with {name} solved more start states in "
          f"{ahead}, fewer in {len(behind)}, the same number in {tied}."
          + (" Fewer: " + "; ".join(f"{c.model} ({c.stats('with')['solved']} against {c.stats('without')['solved']}, "
                                    f"p = {fp(c.paired('solved')['p'])})" for c in behind) + "." if behind else "")
          + clean_tally(done, name), ""]
    L += ["### FALSE CLAIM (descriptive for each model; the declared test is the pooled one above)", "",
          "| model | paired start states | false claim without only | with only | exact p |", "|---|---|---|---|---|"]
    for c in cols:
        pr = c.paired("false_claim")
        L.append(f"| {c.model}{'' if c.complete else '*'} | {pr['pairs']} | {pr['without_only']} | "
                 f"{pr['with_only']} | {fp(pr['p'])} |")
    L += [""] + cross_table(cols, name)
    L += ["## Means per scored episode", ""] + means_table(cols, name) + [""]
    L += means_notes(cols, name, uns, lost)
    L += ["## How the replies ended (finish reason), scored episodes", ""] + finish_table(cols, name)
    L += ["## Episodes that ended without a score", ""]
    if uns:
        by = {}
        for u in uns:
            by.setdefault((u["model"], u["arm"], u["reason"]), []).append(u["cell"])
        L += ["| model | arm | count | reason | start states |", "|---|---|---|---|---|"]
        for (m, arm, why), cs in by.items():
            L.append(f"| {m} | {arm} {name} | {len(cs)} | {why} | {', '.join(cs)} |")
    else:
        L.append("None.")
    L += ["", "Each one counts as not solved, not clean and no false claim for its arm. Attempts that are in no "
          "row are listed and priced in lost_attempts.csv.", ""]
    write(os.path.join(HERE, "SUMMARY.md"), "\n".join(L), check, changed)
    return cols


# ---------------------------------------------------------------- the extension: three lists, one family of twelve

EXT = [("list2", "extension_list2", "list 2", "LIBERO-PRO"),
       ("libero10", "extension_libero10", "the LIBERO-10 list", "LIBERO-10")]
DEV = ("libero10_development", "extension_libero10_development")
FAMILY_MODELS = [MAIN_MODEL, CLEAN_MODEL]
WORD = dict(MEASURES)


p3 = fp                                  # the extension prints every p as the grid does


def w1(iv):
    return "" if iv is None else f"{100 * iv[0]:.1f} to {100 * iv[1]:.1f}"


def d2(f, g, n):
    """A difference in points, printed to two decimals (on 400 start states it is an exact quarter point)."""
    return f"{float(Fraction(100 * (f - g), n)):+.2f}"


def nc1(iv):
    return f"{100 * iv[0]:+.1f} to {100 * iv[1]:+.1f}"


def r3(a, b):
    return "" if not b else f"{a / b:.3f}"


def usd2(v):
    return f"{v:.2f}"


class ExtList:
    """One list of the extension: its cells, its two model columns, its unscored, lost, refused and stopped attempts."""

    def __init__(self, key, folder, label, bench):
        self.key, self.label, self.bench = key, label, bench
        self.folder = os.path.join(HERE, folder)
        self.cells, cols, self.uns = columns(self.folder)
        self.by = {c.model: c for c in cols}
        self.cols = [self.by[m] for m in FAMILY_MODELS if m in self.by] + [c for c in cols
                                                                           if c.model not in FAMILY_MODELS]
        self.info = json.load(open(os.path.join(self.folder, "list_info.json")))
        self.lost = load(self.folder, "lost_attempts.csv")
        self.refused = load(self.folder, "refusals.csv")
        self.stopped = load(self.folder, "stopped_attempts.csv")
        self.eps = load(self.folder, "episodes.csv")
        self.N = len(self.cells)
        self.rows = {}
        for c in self.cells:
            self.rows.setdefault(c["task_row"], []).append(c["cell"])
        self.task_of = {c["cell"]: int(c["task"]) for c in self.cells}
        self.complete = bool(self.cols) and all(c.complete for c in self.cols)

    def count(self, col, arm, key, cells=None):
        return sum(col.val(arm, x, key) for x in (cells if cells is not None else [c["cell"] for c in self.cells]))

    def rowlevel(self, col, key):
        more = fewer = 0
        for cs in self.rows.values():
            a, b = self.count(col, "with", key, cs), self.count(col, "without", key, cs)
            more += a > b
            fewer += a < b
        return {"rows": len(self.rows), "more": more, "fewer": fewer, "tied": len(self.rows) - more - fewer,
                "p": sign_p(more, fewer)}

    def money(self, col, arm):
        sc = list(col.arm[arm]["scored"].values())
        lost = [x for x in self.lost if x["model"] == col.model and x["arm"] == arm]
        n = len(self.cells)
        rows_usd = sum(float(e["usd_list"]) for e in sc)
        lost_usd = sum(float(x["usd_list"]) for x in lost)
        solved = sum(int(e["solved"]) for e in sc)
        tin = sum(int(e["tokens_in"]) for e in sc)
        return {"scored": len(sc), "requests": sum(int(e["requests"]) for e in sc),
                "failed": sum(int(e["failed_requests"]) for e in sc),
                "mean_requests": sum(int(e["requests"]) for e in sc) / len(sc),
                "mean_steps": sum(int(e["steps"]) for e in sc) / len(sc),
                "mean_wall_s": sum(float(e["wall_s"]) for e in sc) / len(sc),
                "tokens_in": tin, "cached_share": sum(int(e["tokens_cached"]) for e in sc) / tin if tin else None,
                "tokens_out": sum(int(e["tokens_out"]) for e in sc), "usd_rows": rows_usd, "lost_n": len(lost),
                "usd_lost": lost_usd, "usd_cell": (rows_usd + lost_usd) / n,
                "usd_solved": (rows_usd + lost_usd) / solved if solved else None,
                "rate_mean": rows_usd / sum(float(e["wall_s"]) for e in sc),
                "rate_max": max(float(e["usd_list"]) / float(e["wall_s"]) for e in sc if float(e["wall_s"]))}


def family(lists):
    """The twelve tests (two models x the two lists x three measures), Holm's step-down at 0.05. Settled only when
    all four columns are complete."""
    tests = []
    for m in FAMILY_MODELS:
        for L in lists:
            col = L.by.get(m)
            if col is None:
                continue
            for key, word in MEASURES:
                pr = col.paired(key)
                tests.append({"model": m, "list": L, "col": col, "key": key, "word": word, "pr": pr,
                              "with": L.count(col, "with", key), "without": L.count(col, "without", key),
                              "so": col.paired(key, scored_only=True), "row": L.rowlevel(col, key)})
    settled = len(lists) == len(EXT) and len(tests) == 12 and all(L.complete for L in lists)
    order = sorted(tests, key=lambda t: t["pr"]["p"] if t["pr"]["p"] is not None else 2.0)
    alive = True
    for rank, t in enumerate(order, 1):
        t["rank"], t["threshold"] = rank, ALPHA / (len(order) - rank + 1)
        alive = alive and t["pr"]["p"] is not None and t["pr"]["p"] <= t["threshold"]
        t["passes"] = alive
        f, g = t["pr"]["with_only"], t["pr"]["without_only"]
        t["ahead"] = (f < g) if t["key"] == "false_claim" else (f > g)
        t["even"] = f == g
        r = t["row"]
        t["row_ahead"] = r["fewer"] if t["key"] == "false_claim" else r["more"]
        t["row_behind"] = r["more"] if t["key"] == "false_claim" else r["fewer"]
        t["clause"] = (t["passes"] and settled and r["p"] is not None and r["p"] > ALPHA)
        if not settled:
            t["verdict"] = "family not settled"
        elif t["passes"]:
            t["verdict"] = ("established on the extension list" if t["ahead"] else
                            "established on the extension list AGAINST Cerulion")
            if t["key"] == "false_claim" and t["ahead"]:
                t["verdict"] += " (fewer with Cerulion)"
        elif t["pr"]["p"] is not None and t["pr"]["p"] <= ALPHA:
            t["verdict"] = ("points up, not established" if t["ahead"] else "points down, not established")
        else:
            t["verdict"] = "no difference shown"
        if t["clause"]:
            t["verdict"] += f", on {t['row_ahead']} of the {r['rows']} {unit_of(t['list'])}"
    return order, settled


def list_name(L):
    return "list 2 (LIBERO-PRO)" if L.key == "list2" else L.bench


def unit_of(L):
    return "tasks" if L.bench == "LIBERO-10" else "task rows"


S2_LINE = ("- The 400-cell lists were added after the first rows of the 100-cell grid had been scored and seen. The "
           "100-cell test declared beforehand remains the primary result. That test is the grid's main test, in "
           "../SUMMARY.md.")


def s_lines(name, libero10=False, pool=False):
    L = [f"- These are new start states of task rows {name} was developed on at other start states. They are not "
         f"unseen tasks.",
         S2_LINE]
    if libero10:
        L.append(f"- LIBERO-10's ten tasks are the ones {name} was developed and gated on; these are 40 other start "
                 f"states of each. Supporting evidence, not a second benchmark.")
    if pool:
        L.append("- With any number that pools two lists: " + POOL_CAVEAT)
    return L


# How the LIBERO-10 list and its development start states ran when two machines stopped. The rows carry no machine,
# so these counts are stated here, not computed: the episodes moved, the machines they moved to, the start states
# whose two arms then ran on two machines hours apart, and the episodes of one model and arm run at once.
L10_MOVED = {"episodes": 141, "list": 105, "development": 36, "machines": 3}
L10_SPLIT = [("GPT-6 Astra", "l10_t06_i44"), ("GPT-6 Astra", "l10_t02_i29"), ("Gemini 3.8 Flash", "l10_t06_i29")]
L10_AT_ONCE = {"machines": 2, "model": "GPT-6 Astra", "episodes": 4, "allowed": 2}
L10_LAST_SECONDS = [("Gemini 3.8 Flash", "without", "l10_t06_i29"), ("Gemini 3.8 Flash", "without", "l10_t05_i30")]

POOL_CAVEAT = ("a pool of two lists: 100 cells whose first rows had been seen when the other 400 were added; "
               "25 task rows, 20 start states each, six suite by perturbation columns; not the published 10 tasks "
               "x 50 of one column.")
FIELD_500 = ("500 start states an arm is the field's count for trained policies; 25 task rows x 20 start states over "
             "six columns here, 10 tasks x 50 in one column there")


def l10_size_line(L10):
    per = {}
    for c in L10.cells:
        per[int(c["task"])] = per.get(int(c["task"]), 0) + 1
    full = 40 * len(per)
    return (f"The LIBERO-10 list holds {min(per.values())} to {max(per.values())} start states a task, {L10.N} in all: "
            f"{full - L10.N} of the {full} start states 10 to 49 had been named by earlier files and were left out, "
            f"not replaced.")


def family_table(order, settled):
    L = ["| Holm rank | model | list | measure | with (Wilson 95) | without (Wilson 95) | difference, points "
         "(Newcombe 95) | with only | without only | exact p | Holm threshold | met | verdict |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for t in order:
        pr, n = t["pr"], t["list"].N
        L.append(f"| {t['rank']} | {t['model']} | {list_name(t['list'])} | {t['word']} | {t['with']} of {n} "
                 f"({w1(wilson(t['with'], n))}) | {t['without']} of {n} ({w1(wilson(t['without'], n))}) | "
                 f"{d2(pr['with_only'], pr['without_only'], pr['pairs'])} ({nc1(pr['newcombe'])}) | {pr['with_only']} | "
                 f"{pr['without_only']} | {p3(pr['p'])} | 0.05 / {len(order) - t['rank'] + 1} = {t['threshold']:.5f} | "
                 f"{'yes' if t['passes'] else 'no'} | {t['verdict']} |")
    passed = [t for t in order if t["passes"]]
    first_fail = next((t for t in order if not t["passes"]), None)
    L += ["", f"Holm's step-down: the {len(passed)} smallest p values each meet their threshold, in order"
          + (f"; test {first_fail['rank']} ({first_fail['model']}, {first_fail['word']} on {list_name(first_fail['list'])}, "
             f"p {p3(first_fail['pr']['p'])}) misses its threshold {first_fail['threshold']:.5f} and stops the rest."
             if first_fail else ".")
          + (" The family is settled: every start state of both lists has ended in both arms of both models."
             if settled else " The family is NOT settled: a column is incomplete, so no test carries a verdict.")]
    against = [t for t in passed if not t["ahead"]]
    up = [t for t in order if not t["passes"] and t["pr"]["p"] is not None and t["pr"]["p"] <= ALPHA]
    L.append(f"Tests passing against Cerulion: {len(against)}. Tests at or under 0.05 that miss Holm: {len(up)}.")
    return L


def bad_news(name, lists, order, grid_cols):
    """The lines that qualify the family's numbers, each computed from the CSV files of this package."""
    by = {(t["model"], t["list"].key, t["key"]): t for t in order}
    P, L10 = {L.key: L for L in lists}.get("list2"), {L.key: L for L in lists}.get("libero10")
    out = []
    if L10 and CLEAN_MODEL in L10.by and MAIN_MODEL in L10.by:
        a, f = L10.by[CLEAN_MODEL], L10.by[MAIN_MODEL]
        ref = [r for r in L10.refused]
        tasks = sorted({L10.task_of[r["cell"]] for r in ref})
        on = [c["cell"] for c in L10.cells if int(c["task"]) in tasks]
        off = [c["cell"] for c in L10.cells if int(c["task"]) not in tasks]
        tw = " and ".join(str(x) for x in tasks)
        t = by.get((CLEAN_MODEL, "libero10", "solved"))
        if t and t["clause"]:
            out.append(f"- {CLEAN_MODEL}'s LIBERO-10 SOLVED pass comes from tasks {tw}. It passes Holm at the start-state level "
                       f"({t['with']} against {t['without']} of {L10.N}, p {p3(t['pr']['p'])}), but by task "
                       f"{name} is ahead on {t['row_ahead']} of the 10 and behind on {t['row_behind']} (task-level p "
                       f"{p3(t['row']['p'])}), so a sentence may only say \"on {t['row_ahead']} of the 10 tasks\". The "
                       f"whole start-state difference is on tasks {tw}: without {name} {L10.count(a, 'without', 'solved', on)} "
                       f"of {len(on)} solved, with {name} {L10.count(a, 'with', 'solved', on)} of {len(on)}. On the other "
                       f"{10 - len(tasks)} tasks, with {name} {L10.count(a, 'with', 'solved', off)} and without "
                       f"{L10.count(a, 'without', 'solved', off)} of {len(off)} (exploratory: no test, not a finding). "
                       f"{MAIN_MODEL} without {name} also solved {L10.count(f, 'without', 'solved', on)} of {len(on)} on "
                       f"tasks {tw}, with no refusal, so those tasks are at zero for the published interface with both "
                       f"models.")
        hl = {r["cell"] for r in ref}
        un = {u["cell"] for u in L10.uns if u["model"] == CLEAN_MODEL and u["arm"] == "without"}
        later = sorted(hl - un)
        sc = a.arm["without"]["scored"]
        dev = load(os.path.join(HERE, DEV[1]), "refusals.csv")
        per_task = "; ".join(
            f"task {k}: {sum(1 for r in ref if L10.task_of[r['cell']] == k)} attempts over "
            f"{len({r['cell'] for r in ref if L10.task_of[r['cell']] == k})} start states" for k in tasks)
        models = {r["model"] for r in ref + dev}
        arms = {r["arm"] for r in ref + dev}
        out.append(f"- The provider's image filter refused {CLEAN_MODEL}'s arm without {name} on exactly those "
                   f"tasks: HTTP 400 \"Image processing blocked due to content policy violation\" in {len(ref)} attempts "
                   f"over {len(hl)} start states of the LIBERO-10 list ({per_task}), and {len(dev)} more attempts over "
                   f"{len({r['cell'] for r in dev})} of the development start states (refusals.csv in each folder). "
                   + ("Every one is of that model and arm: no attempt with " + name + " and no attempt of any other "
                      "model carries it. " if models == {CLEAN_MODEL} and arms == {"without"} else "")
                   + f"Of those {len(hl)} start states, {len(hl & un)} ended unscored after 3 attempts (failures for "
                   f"that arm in every test) and {len(later)} were scored on a later attempt or past the refusals: "
                   f"{sum(int(sc[c]['solved']) for c in later)} of the {len(later)} solved, "
                   f"{sum(int(sc[c]['false_claim']) for c in later)} of them ended on a false claim. With {name}, "
                   f"{L10.count(a, 'with', 'solved', sorted(hl))} of the {len(hl)} were solved. No start state was run "
                   f"again for it: the requests reached the provider and were answered.")
        t = by.get((CLEAN_MODEL, "libero10", "false_claim"))
        if t:
            s = a.stats("with"), a.stats("without")
            out.append(f"- {CLEAN_MODEL}'s LIBERO-10 FALSE CLAIM pass also comes from tasks {tw}: {t['with']} against "
                       f"{t['without']} of {L10.N} (p {p3(t['pr']['p'])}), but by task {name} has fewer on "
                       f"{t['row_ahead']} of the 10 and more on {t['row_behind']} (task-level p {p3(t['row']['p'])}). Per "
                       f"unsolved episode {name} is not lower: {r3(s[0]['false_claims'], s[0]['unsolved'])} against "
                       f"{r3(s[1]['false_claims'], s[1]['unsolved'])} (or "
                       f"{r3(s[1]['false_claims'], s[1]['scored'] - s[1]['solved'])} on scored rows only). Outside "
                       f"tasks {tw}: with {name} {L10.count(a, 'with', 'false_claim', off)} and without "
                       f"{L10.count(a, 'without', 'false_claim', off)} false claims (exploratory).")
        t = by.get((CLEAN_MODEL, "libero10", "clean"))
        if t and t["clause"]:
            r = t["row"]
            out.append(f"- {CLEAN_MODEL}'s LIBERO-10 CLEAN pass holds on {t['row_ahead']} of the 10 tasks only (ahead "
                       f"{r['more']}, behind {r['fewer']}, tied {r['tied']}, task-level p {p3(r['p'])}): a sentence must "
                       f"say \"on {t['row_ahead']} of the 10 tasks\".")
        t = by.get((MAIN_MODEL, "libero10", "false_claim"))
        if t:
            s = f.stats("with"), f.stats("without")
            out.append(f"- {MAIN_MODEL} on LIBERO-10: {t['verdict']} in false claims ({t['with']} against "
                       f"{t['without']} of {L10.N}, p {p3(t['pr']['p'])}), and per unsolved episode {name} claims "
                       f"falsely {'MORE' if s[0]['false_claims'] / s[0]['unsolved'] > s[1]['false_claims'] / s[1]['unsolved'] else 'less'} "
                       f"often ({r3(s[0]['false_claims'], s[0]['unsolved'])} against "
                       f"{r3(s[1]['false_claims'], s[1]['unsolved'])}).")
    if L10 and L10.stopped and all(m in L10.by for m, _, _ in L10_LAST_SECONDS):
        cells = {c["cell"] for c in L10.cells}
        assert all(c in cells for _, c in L10_SPLIT) and all(c in cells for _, _, c in L10_LAST_SECONDS)
        stops = len({s["machine_stopped"] for s in L10.stopped})
        last = [(m, arm, c) for m, arm, c in L10_LAST_SECONDS if not L10.by[m].val(arm, c, "solved")]
        split = {}
        for m, c in L10_SPLIT:
            split.setdefault(m, []).append(c)
        lm = {}
        for m, arm, c in L10_LAST_SECONDS:
            lm.setdefault(f"{m} {arm} {name}", []).append(c)
        solved_last = len(L10_LAST_SECONDS) - len(last)
        dev_stopped = load(os.path.join(HERE, DEV[1]), "stopped_attempts.csv")
        out.append(f"- The LIBERO-10 list ran under changed conditions: {stops} machines stopped during its run, with "
                   f"{len(L10.stopped)} attempts running (../extension_libero10/stopped_attempts.csv, priced under the cost "
                   f"table of ../extension_libero10/SUMMARY.md), and "
                   f"the {L10_MOVED['episodes']} episodes they had not ended ({L10_MOVED['list']} of the LIBERO-10 list, "
                   f"{L10_MOVED['development']} of its development start states) ran from attempt 1 ("
                   f"{len(L10.stopped) + len(dev_stopped)} of them again) on {L10_MOVED['machines']} other machines "
                   f"hours later. For {len(L10_SPLIT)} start states this put the "
                   f"two arms of a model on two machines hours apart ("
                   + ", ".join(f"{m} {' and '.join(cs)}" for m, cs in split.items())
                   + f"), and {L10_AT_ONCE['machines']} machines then ran up to {L10_AT_ONCE['episodes']} "
                   f"{L10_AT_ONCE['model']} episodes at once where the protocol allowed {L10_AT_ONCE['allowed']} a "
                   f"machine. A row "
                   f"written in a stopped machine's last seconds and never copied off cannot be ruled out; the "
                   f"{len(L10_LAST_SECONDS)} start states that could hold one ("
                   + "; ".join(f"{k}, {' and '.join(cs)}" for k, cs in lm.items())
                   + ") ran again and " + ("were not solved." if not solved_last else
                                           f"{solved_last} of them were solved."))
    if P and MAIN_MODEL in P.by:
        f = P.by[MAIN_MODEL]
        s = f.stats("with"), f.stats("without")
        if s[0]["false_claims"] < s[1]["false_claims"] and \
                s[0]["false_claims"] / s[0]["unsolved"] >= s[1]["false_claims"] / s[1]["unsolved"]:
            out.append(f"- {MAIN_MODEL}'s fewer false claims on list 2 are because {name} solved more, not because "
                       f"the model was more candid when it failed: per unsolved episode "
                       f"{r3(s[0]['false_claims'], s[0]['unsolved'])} with {name} against "
                       f"{r3(s[1]['false_claims'], s[1]['unsolved'])} without.")
    if P and CLEAN_MODEL in P.by:
        a = P.by[CLEAN_MODEL]
        t = by.get((CLEAN_MODEL, "list2", "solved"))
        g = {c.model: c for c in (grid_cols or [])}.get(CLEAN_MODEL)
        if t and not t["passes"]:
            parts = [f"- {CLEAN_MODEL} SOLVED: {t['verdict']} on list 2 ({t['with']} against {t['without']} of "
                     f"{P.N}, p {p3(t['pr']['p'])}; the expected result at this count, RULES.md section 7)"]
            if g and g.complete:
                gw, go = g.stats("with")["solved"], g.stats("without")["solved"]
                pw, po = gw + t["with"], go + t["without"]
                parts.append(f", none in the two lists together ({pw} against {po} of {P.N + len(g.cells)}; "
                             f"{FIELD_500}), none on "
                             f"the grid's {len(g.cells)} ({gw} against {go})")
            r = t["row"]
            worst = sorted(((P.count(a, "without", "solved", cs) - P.count(a, "with", "solved", cs), row,
                             P.count(a, "with", "solved", cs), P.count(a, "without", "solved", cs), len(cs))
                            for row, cs in P.rows.items()), reverse=True)
            behind = [x for x in worst if x[0] > 0]
            gt = [c["cell"] for c in P.cells if c["suite"] == "libero_goal" and c["perturbation"] == "task"]
            parts.append(f". By task row {name} is behind on {r['fewer']} of the {r['rows']}, by up to "
                         f"{behind[0][0]} start states of {behind[0][4]} ("
                         + "; ".join(f"{x[1]}: {x[2]} against {x[3]}" for x in behind[:2])
                         + f"), and on the Goal by task column {name} solved "
                         f"{P.count(a, 'with', 'solved', gt)} and without {P.count(a, 'without', 'solved', gt)} of "
                         f"{len(gt)} (exploratory).")
            if L10 and CLEAN_MODEL in L10.by:
                b = L10.by[CLEAN_MODEL]
                beh = [(k, cs) for k, cs in sorted(L10.rows.items(), key=lambda kv: L10.task_of[kv[1][0]])
                       if L10.count(b, "with", "solved", cs) < L10.count(b, "without", "solved", cs)]
                parts.append(f" With {CLEAN_MODEL} on LIBERO-10 {name} is behind on tasks "
                             + ", ".join(f"{L10.task_of[cs[0]]} ({L10.count(b, 'with', 'solved', cs)} against "
                                         f"{L10.count(b, 'without', 'solved', cs)})" for _, cs in beh) + ".")
            out.append("".join(parts))
    if P and MAIN_MODEL in P.by and CLEAN_MODEL in P.by:
        pr = P.by[MAIN_MODEL].paired("solved", other=P.by[CLEAN_MODEL])
        if pr["p"] is not None and pr["p"] > ALPHA:
            out.append(f"- {MAIN_MODEL} with {name} against {CLEAN_MODEL} without: \"as many\" on list 2 is \"no "
                       f"difference shown\" ({P.count(P.by[MAIN_MODEL], 'with', 'solved')} against "
                       f"{P.count(P.by[CLEAN_MODEL], 'without', 'solved')} of {P.N}, p {p3(pr['p'])}), not shown equal.")
    return out


def replicated_section(name, P, order, grid_cols):
    """How list 2 relates to the grid's tests: a statement is said once, and 'replicated' only where both pass."""
    g = {c.model: c for c in (grid_cols or [])}
    by = {(t["model"], t["list"].key, t["key"]): t for t in order}
    gdone = bool(grid_cols) and all(c.complete for c in grid_cols)
    held = "and it held on 400 further start states (a list added after the first grid rows were seen)"
    L = ["| question | the grid's test, grid 100 | list 2's test (Holm) | what may be said |", "|---|---|---|---|"]
    rep = []

    def ext(m, key):
        t = by.get((m, "list2", key))
        return t, ("passes" if t and t["passes"] else "does not pass")

    rows = []
    gm = g.get(MAIN_MODEL)
    if gm and gm.complete:
        pr = gm.paired("solved")
        ok = pr["p"] is not None and pr["p"] <= ALPHA and pr["with_only"] > pr["without_only"]
        rows.append((f"{MAIN_MODEL} SOLVED", f"the main test: {gm.stats('with')['solved']} against "
                     f"{gm.stats('without')['solved']} of {len(gm.cells)} (with only {pr['with_only']}, without only "
                     f"{pr['without_only']}), p {p3(pr['p'])}: {'passes' if ok else 'does not pass'} at 0.05", ok,
                     ext(MAIN_MODEL, "solved")))
    ga = g.get(CLEAN_MODEL)
    if ga and ga.complete:
        pr = ga.paired("clean")
        ok = pr["p"] is not None and pr["p"] <= ALPHA and pr["with_only"] > pr["without_only"]
        rows.append((f"{CLEAN_MODEL} CLEAN", f"declared secondary: {ga.stats('with')['clean']} against "
                     f"{ga.stats('without')['clean']} ({pr['with_only']} against {pr['without_only']}), p "
                     f"{p3(pr['p'])}: {'passes' if ok else 'does not pass'} at 0.05", ok, ext(CLEAN_MODEL, "clean")))
        pr = ga.paired("solved")
        famp = holm({c.model: c.paired("solved")["p"] for c in grid_cols}).get(CLEAN_MODEL)
        ok = bool(gdone and famp and famp[1])
        rows.append((f"{CLEAN_MODEL} SOLVED", f"in the per-model family: {ga.stats('with')['solved']} against "
                     f"{ga.stats('without')['solved']} ({pr['with_only']} against {pr['without_only']}), p "
                     f"{p3(pr['p'])}" + (": cannot pass at any threshold" if pr["p"] is not None and pr["p"] > ALPHA
                                         else ""), ok, ext(CLEAN_MODEL, "solved")))
    if gm and gm.complete:
        pr = gm.paired("clean")
        strict = ALPHA / len(grid_cols)
        famp = holm({c.model: c.paired("clean")["p"] for c in grid_cols}).get(MAIN_MODEL)
        ok = bool(gdone and famp and famp[1])
        txt = (f"in the grid's CLEAN family, " + ("settled" if gdone else "not settled while the grid runs") +
               f"; {gm.stats('with')['clean']} against {gm.stats('without')['clean']} ({pr['with_only']} against "
               f"{pr['without_only']}), p {p3(pr['p'])}"
               + (f", under 0.05 / {len(grid_cols)}, the strictest threshold that family can set"
                  if pr["p"] is not None and pr["p"] <= strict else ""))
        rows.append((f"{MAIN_MODEL} CLEAN", txt, ok, ext(MAIN_MODEL, "clean")))
    for q, gtxt, gok, (t, etxt) in rows:
        if gok and t and t["passes"]:
            say = f"the statement, then \"{held}\""
            rep.append(q)
        elif gok:
            say = "the statement stands, and the same sentence carries \"it did not hold on the 400-cell extension list\" with its counts and p"
        elif t and t["passes"]:
            say = ("when the grid's test is settled and passes: the statement, then \"" + held + "\"; until then, "
                   "no upgrade") if "not settled" in gtxt else "no upgrade: the grid's rule decides"
        else:
            say = "no statement; both printed"
        L.append(f"| {q} | {gtxt} | {etxt} | {say} |")
    fc = "; ".join(f"{m} {g[m].stats('with')['false_claims']} against {g[m].stats('without')['false_claims']}"
                   for m in FAMILY_MODELS if m in g and g[m].complete)
    fcx = all(by.get((m, "list2", "false_claim"), {}).get("passes") for m in FAMILY_MODELS)
    L.append(f"| FALSE CLAIM, a model | the grid's test pools every model and is " +
             ("settled" if gdone else "not settled while the grid runs") + f"; the grid's per-model counts ({fc}) "
             f"are not a grid test | {'passes for both models' if fcx else 'see the family table'} | this list's own "
             f"sentences only |")
    L += ["", "\"Replicated\" may be said only where both columns read passes: " + (" and ".join(rep) if rep else "none")
          + ". Not of " + ", ".join(f"{t['model']} {t['word']}" for t in order if t["list"].key == "list2"
                                    and t["passes"] and f"{t['model']} {t['word']}" not in rep)
          + " from this list. One effect is one statement: the grid's test and this list's test on the same model "
          "and measure are one finding, their p values are never combined, and nothing is confirmed twice."]
    return L


def scored_pairs_table(L, order):
    T = ["| model | measure | pairs | with | without | with only | without only | exact p | difference (Newcombe 95) | "
         "under the same Holm threshold | start states left out |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for t in order:
        if t["list"] is not L:
            continue
        s = t["so"]
        out = sorted({u["cell"] for u in L.uns if u["model"] == t["model"]},
                     key=lambda c: next(i for i, x in enumerate(L.cells) if x["cell"] == c))
        T.append(f"| {t['model']} | {t['word']} | {s['pairs']} | {s['both'] + s['with_only']} | "
                 f"{s['both'] + s['without_only']} | {s['with_only']} | {s['without_only']} | {p3(s['p'])} | "
                 f"{d2(s['with_only'], s['without_only'], s['pairs'])} ({nc1(s['newcombe'])}) | "
                 f"{'yes' if s['p'] is not None and s['p'] <= t['threshold'] else 'no'} | {', '.join(out) or 'none'} |")
    return T


def row_table(L, order):
    unit = "task rows" if L.bench != "LIBERO-10" else "tasks"
    T = [f"| model | measure | {unit} | with more | with fewer | tied | exact p on untied | start-state test | "
         f"the sentence must say |", "|---|---|---|---|---|---|---|---|---|"]
    for t in order:
        if t["list"] is not L:
            continue
        r = t["row"]
        must = (f"on {t['row_ahead']} of the {r['rows']} {unit} (with more {r['more']}, fewer {r['fewer']}, tied "
                f"{r['tied']}); not \"across the suite\"" if t["clause"] else "no extra clause")
        T.append(f"| {t['model']} | {t['word']} | {r['rows']} | {r['more']} | {r['fewer']} | {r['tied']} | "
                 f"{p3(r['p'])} | {'passes Holm' if t['passes'] else 'does not pass'} | {must} |")
    return T


def every_row_table(L):
    unit = "task row" if L.bench != "LIBERO-10" else "task"
    head = [f"{'Flash' if m == MAIN_MODEL else 'Astra'} {w}" for m in FAMILY_MODELS for _, w in MEASURES]
    T = [f"| {unit} | start states | " + " | ".join(head) + " |", "|---|---|" + "---|" * len(head)]
    rows = list(L.rows.items())
    if L.bench == "LIBERO-10":
        rows.sort(key=lambda kv: L.task_of[kv[1][0]])
    for row, cs in rows:
        vals = [f"{L.count(L.by[m], 'with', k, cs)} / {L.count(L.by[m], 'without', k, cs)}"
                for m in FAMILY_MODELS for k, _ in MEASURES]
        T.append(f"| {row} | {len(cs)} | " + " | ".join(vals) + " |")
    return T


def arms_table(L, name):
    T = ["| model | arm | start states | scored | unscored | solved (Wilson 95) | clean (Wilson 95) | false claims "
         "(Wilson 95) | false claims per unsolved episode | the same, scored rows only |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for c in L.cols:
        for arm in ("with", "without"):
            s = c.stats(arm)
            n = s["finished"]
            T.append(f"| {c.model}{'' if c.complete else '*'} | {arm} {name} | {n} | {s['scored']} | {s['unscored']} | "
                     f"{s['solved']} ({w1(wilson(s['solved'], n))}) | {s['clean']} ({w1(wilson(s['clean'], n))}) | "
                     f"{s['false_claims']} ({w1(wilson(s['false_claims'], n))}) | {s['false_claims']} of "
                     f"{s['unsolved']} ({r3(s['false_claims'], s['unsolved'])}) | {s['false_claims']} of "
                     f"{s['scored'] - s['solved']} ({r3(s['false_claims'], s['scored'] - s['solved'])}) |")
    return T


def money_table(L, name):
    T = ["| model | arm | mean requests | mean control steps | requests answered | failed requests, each sent again | "
         "tokens in (cached share) | tokens out | USD, scored rows | attempts in no row, USD | USD a start state | "
         "USD a solved start state | mean seconds an episode |", "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in L.cols:
        for arm in ("with", "without"):
            m = L.money(c, arm)
            T.append(f"| {c.model} | {arm} {name} | {m['mean_requests']:.1f} | {m['mean_steps']:.1f} | "
                     f"{m['requests']} | {m['failed']} | {m['tokens_in']:,} ({100 * m['cached_share']:.1f} percent) | "
                     f"{m['tokens_out']:,} | {usd2(m['usd_rows'])} | {m['lost_n']}, {usd2(m['usd_lost'])} | "
                     f"{m['usd_cell']:.2f} | {'none solved' if m['usd_solved'] is None else format(m['usd_solved'], '.2f')} | "
                     f"{m['mean_wall_s']:.1f} |")
    return T


def cross_lines(L, name):
    if not (MAIN_MODEL in L.by and CLEAN_MODEL in L.by):
        return []
    T = ["| measure | " + f"{MAIN_MODEL} with {name} (Wilson 95) | {CLEAN_MODEL} without (Wilson 95) | difference, "
         "points (Newcombe 95) | first only | second only | exact p | wording |", "|---|---|---|---|---|---|---|---|"]
    for key, word in MEASURES[:2]:
        pr = L.by[MAIN_MODEL].paired(key, other=L.by[CLEAN_MODEL])
        a, b = L.count(L.by[MAIN_MODEL], "with", key), L.count(L.by[CLEAN_MODEL], "without", key)
        wd = ("no difference shown" if pr["p"] is None or pr["p"] > ALPHA else
              f"differs (p {p3(pr['p'])}; a pair named beforehand, outside the corrected family)")
        T.append(f"| {word} | {a} of {L.N} ({w1(wilson(a, L.N))}) | {b} of {L.N} ({w1(wilson(b, L.N))}) | "
                 f"{d2(pr['with_only'], pr['without_only'], pr['pairs'])} ({nc1(pr['newcombe'])}) | {pr['with_only']} | "
                 f"{pr['without_only']} | {p3(pr['p'])} | {wd} |")
    return T


def stopped_lines(L, name):
    if not L.stopped:
        return []
    mean = high = 0.0
    for s in L.stopped:
        col = L.by[s["model"]]
        m = L.money(col, s["arm"])
        mean += m["rate_mean"] * int(s["ran_at_most_s"])
        high += m["rate_max"] * int(s["ran_at_most_s"])
    # the episode's scored row after the stop: its attempt number is the rerun's, counted again from 1
    later = [L.by[s["model"]].arm[s["arm"]]["scored"].get(s["cell"]) for s in L.stopped]
    scored = [e for e in later if e]
    on1 = sum(1 for e in scored if int(e["attempt"]) == 1)
    rerun = (f"each start state ran again from attempt 1 on another machine ({len(scored)} of the {len(L.stopped)} "
             f"were then scored, all on that attempt 1)" if scored and on1 == len(scored) else
             f"each start state ran again on another machine ({len(scored)} of the {len(L.stopped)} were then "
             f"scored, {on1} of them on attempt 1)")
    return [f"{len(L.stopped)} attempts were running when their machine stopped and left no token count on any copy "
            f"(stopped_attempts.csv, with the longest each can have run). They are in no row and in no total above, "
            f"and {rerun}. Priced at the mean spend a second of their model and arm on this list (the USD of its scored "
            f"rows over their seconds), they come to about {mean:.0f} USD; at the highest spend a second of any scored "
            f"row of their model and arm, about {high:.0f} USD. No row bounds them."]


def sentences(name, lists, order, grid_cols, this):
    """The sentences the reading rule allows for this list's results, filled from the counts above."""
    by = {(t["model"], t["list"].key, t["key"]): t for t in order}
    P, L10 = {L.key: L for L in lists}.get("list2"), {L.key: L for L in lists}.get("libero10")
    P_this, L10_this = (P if this is P else None), (L10 if this is L10 else None)
    out = []

    def ab(t):
        return f"{t['with']} against {t['without']}"

    if P_this:
        t = by.get((MAIN_MODEL, "list2", "solved"))
        g = {c.model: c for c in (grid_cols or [])}.get(MAIN_MODEL)
        if t and t["passes"] and t["ahead"]:
            line = (f"- {MAIN_MODEL} SOLVED, list 2: \"With {MAIN_MODEL}, on {P.N} further start states, {name} solved "
                    f"{t['with']} of {P.N} and the plain interface {t['without']} of {P.N} (exact paired test p "
                    f"{p3(t['pr']['p'])}; {name} only {t['pr']['with_only']}, plain only {t['pr']['without_only']}).\"")
            if g and g.complete:
                gp = g.paired("solved")
                if gp["p"] is not None and gp["p"] <= ALPHA and gp["with_only"] > gp["without_only"]:
                    line += " The main test passed: \"The 100-cell result held on 400 further start states.\""
            out.append(line)
            if g and g.complete and P.by[MAIN_MODEL].complete:
                a1, a2 = g.paired("solved"), P.by[MAIN_MODEL].paired("solved")
                arms, ok = [], True
                for side in ("with_only", "without_only"):
                    k1, k2 = a1["both"] + a1[side], a2["both"] + a2[side]
                    x1, x2 = wilson(k1, a1["pairs"]), wilson(k2, a2["pairs"])
                    ok = ok and not (x1[1] < x2[0] or x2[1] < x1[0])
                    arms.append((k1 + k2, a1["pairs"] + a2["pairs"]))
                if ok:
                    (kw, n), (ko, _) = arms
                    out.append(f"- The two lists together, {MAIN_MODEL} SOLVED: \"Over {n} start states an arm (two "
                               f"lists), {name} solved {kw} of {n} ({w1(wilson(kw, n))} percent) and the plain "
                               f"interface {ko} of {n} ({w1(wilson(ko, n))} percent).\" No p in this sentence; with "
                               f"the pool's line, \"{POOL_CAVEAT}\", and \"{FIELD_500}.\"")
        elif t:
            out.append(f"- {MAIN_MODEL} SOLVED, list 2: \"With {MAIN_MODEL}, the gain did not hold on the "
                       f"400-cell extension list: {ab(t)} of {P.N} (p {p3(t['pr']['p'])}).\"")
        t = by.get((CLEAN_MODEL, "list2", "clean"))
        ts = by.get((CLEAN_MODEL, "list2", "solved"))
        if t and ts:
            if t["passes"] and t["ahead"]:
                out.append(f"- {CLEAN_MODEL} CLEAN, list 2, with its SOLVED counts: \"With {CLEAN_MODEL}, more of "
                           f"{name}'s solves came inside the published step limit: {ab(t)} of {P.N} (p "
                           f"{p3(t['pr']['p'])}); it solved {ab(ts)} of {P.N} (p {p3(ts['pr']['p'])}).\"")
            else:
                out.append(f"- {CLEAN_MODEL} CLEAN, list 2: \"With {CLEAN_MODEL}, the step-limit gain was not "
                           f"established on 400 further start states ({ab(t)}, p {p3(t['pr']['p'])}).\"")
        if ts and not ts["passes"]:
            out.append(f"- {CLEAN_MODEL} SOLVED, list 2: \"With {CLEAN_MODEL}, {name} did not solve more start states "
                       f"({ab(ts)} of {P.N}, p {p3(ts['pr']['p'])}). A difference this small could not be shown at "
                       f"this count.\"")
        for m in FAMILY_MODELS:
            t = by.get((m, "list2", "false_claim"))
            if t and t["passes"] and t["ahead"]:
                s = t["col"].stats("with"), t["col"].stats("without")
                notlower = s[0]["false_claims"] / s[0]["unsolved"] >= s[1]["false_claims"] / s[1]["unsolved"]
                out.append(f"- {m} FALSE CLAIM, list 2: \"Fewer episodes ended with the model wrongly saying it had "
                           f"succeeded: {ab(t)} of {P.N} (p {p3(t['pr']['p'])})"
                           + (", because it solved more, not because the model was more candid when it failed"
                              if notlower else "") + f".\" Beside it: {s[0]['false_claims']} of {s[0]['unsolved']} "
                           f"unsolved ({r3(s[0]['false_claims'], s[0]['unsolved'])}) against {s[1]['false_claims']} of "
                           f"{s[1]['unsolved']} ({r3(s[1]['false_claims'], s[1]['unsolved'])}).")
        if MAIN_MODEL in P.by and CLEAN_MODEL in P.by:
            pr = P.by[MAIN_MODEL].paired("solved", other=P.by[CLEAN_MODEL])
            a, b = P.count(P.by[MAIN_MODEL], "with", "solved"), P.count(P.by[CLEAN_MODEL], "without", "solved")
            out.append(f"- The cross-model pair, list 2: \"{MAIN_MODEL} with {name} solved {a} of {P.N}; {CLEAN_MODEL} "
                       f"without it solved {b} of {P.N} ("
                       + (f"no difference shown, p {p3(pr['p'])}" if pr["p"] > ALPHA else
                          f"differs, p {p3(pr['p'])}; a pair named beforehand, outside the corrected family") + ").\"")
        rl = []
        for m in FAMILY_MODELS:
            for key, word in MEASURES:
                t = by.get((m, "list2", key))
                if t:
                    r = t["row"]
                    rl.append(f"{m} {word}{' (fewer)' if key == 'false_claim' else ''}: \"{name} ahead on "
                              f"{t['row_ahead']} of {r['rows']} task rows, behind on {t['row_behind']}, tied on "
                              f"{r['tied']}\"")
        out.append("- Task-row reads, list 2: " + "; ".join(rl) + ".")
    if L10_this:
        for m in FAMILY_MODELS:
            for key, word in MEASURES:
                t = by.get((m, "libero10", key))
                if not t:
                    continue
                r = t["row"]
                if t["passes"] and t["ahead"]:
                    clause = ""
                    if t["clause"]:
                        side = ("fewer on", "more on") if key == "false_claim" else ("ahead on", "behind on")
                        clause = (f", on {t['row_ahead']} of the 10 tasks ({name} {side[0]} {t['row_ahead']}, "
                                  f"{side[1]} {t['row_behind']}, tied on {r['tied']})")
                    if key == "false_claim":
                        s = t["col"].stats("with"), t["col"].stats("without")
                        notlower = s[0]["false_claims"] / s[0]["unsolved"] >= s[1]["false_claims"] / s[1]["unsolved"]
                        line = (f"- {m} FALSE CLAIM, LIBERO-10, with the LIBERO-10 line: \"On LIBERO-10, the same "
                                f"direction on {L10.N} other start states of its ten tasks: fewer episodes ended with the "
                                f"model wrongly saying it had succeeded{clause}: {ab(t)} (p {p3(t['pr']['p'])})"
                                + (", because it solved more, not because the model was more candid when it failed"
                                   if notlower else "") + f".\" Beside it: {s[0]['false_claims']} of "
                                f"{s[0]['unsolved']} unsolved ({r3(s[0]['false_claims'], s[0]['unsolved'])}) against "
                                f"{s[1]['false_claims']} of {s[1]['unsolved']} "
                                f"({r3(s[1]['false_claims'], s[1]['unsolved'])}), and the bad news on tasks 4 and 6.")
                    else:
                        line = (f"- {m} {word}, LIBERO-10, with the LIBERO-10 line: \"On LIBERO-10, the same direction "
                                f"on {L10.N} other start states of its ten tasks{clause}: {ab(t)} (p "
                                f"{p3(t['pr']['p'])}).\"")
                        if m == CLEAN_MODEL and key == "solved":
                            ps = by.get((CLEAN_MODEL, "list2", "solved"))
                            line += (" Never without, in the same place, the bad news on tasks 4 and 6 and the "
                                     "image filter")
                            if ps and not ps["passes"]:
                                line += (f", and the LIBERO-PRO null beside it: \"With {CLEAN_MODEL}, {name} did not "
                                         f"solve more start states ({ab(ps)} of {P.N if P else ''}, p "
                                         f"{p3(ps['pr']['p'])}).\"")
                            else:
                                line += "."
                    out.append(line)
                elif not t["passes"]:
                    out.append(f"- {m} {word}, LIBERO-10, with the LIBERO-10 line: \"On LIBERO-10, no difference "
                               f"shown for {m} ({ab(t)} of {L10.N}, p {p3(t['pr']['p'])}).\""
                               if t["verdict"] == "no difference shown" else
                               f"- {m} {word}, LIBERO-10: {t['verdict']} ({ab(t)} of {L10.N}, p {p3(t['pr']['p'])}).")
        if MAIN_MODEL in L10.by and CLEAN_MODEL in L10.by:
            parts = []
            for key, word in MEASURES[:2]:
                pr = L10.by[MAIN_MODEL].paired(key, other=L10.by[CLEAN_MODEL])
                a, b = L10.count(L10.by[MAIN_MODEL], "with", key), L10.count(L10.by[CLEAN_MODEL], "without", key)
                parts.append(f"{word} {a} against {b} of {L10.N}, " + (
                    "no difference shown" if pr["p"] > ALPHA else
                    f"differs (p {p3(pr['p'])}; a pair named beforehand, outside the corrected family)"))
            out.append(f"- The cross-model pair, LIBERO-10, with the LIBERO-10 line: " + "; ".join(parts) + ".")
        rl = []
        for m in FAMILY_MODELS:
            for key, word in MEASURES:
                t = by.get((m, "libero10", key))
                if t:
                    r = t["row"]
                    rl.append(f"{m} {word}{' (fewer)' if key == 'false_claim' else ''}: \"{name} ahead on "
                              f"{t['row_ahead']} of 10 tasks, behind on {t['row_behind']}, tied on {r['tied']}\"")
        out.append("- Task reads, LIBERO-10, with the LIBERO-10 line: " + "; ".join(rl) + ".")
    return out


NOT_SAID = ("Not said, whatever these rows show: that the tasks are new, held back or never developed on; \"the full "
            "LIBERO-PRO benchmark\", \"500 a suite\", \"the full LIBERO-10 suite\" or \"LIBERO-10 at 500\"; that the "
            "extension's existence or size was declared before the grid's rows (its reading rule was fixed before its "
            "own rows; its decision came after the first grid rows were seen); \"confirmed twice\" or any combined "
            "p; anything resting on the pooled p; any comparison verb, multiple or rank against a published system; "
            "\"{name} helps strong models solve more\" (GPT-6 Astra SOLVED did not pass on LIBERO-PRO, its LIBERO-10 "
            "pass is on 5 of the 10 tasks, and LIBERO-10 alone may not carry it); \"{name} stops false claims\" or "
            "\"makes the model honest\"; \"LIBERO-10 confirms it on a "
            "second benchmark\"; any development number in a headline, chart of results, pool or test; anything about "
            "another model from these lists; any split of these rows worded as a finding, the tasks 4 and 6 reading "
            "included.")


def ext_summary_csv(L, order, folder):
    rows = []
    for c in L.cols:
        for arm in ("without", "with"):
            s = c.stats(arm)
            w, cw = s["solved_wilson"] or (None, None), s["clean_wilson"] or (None, None)
            rows.append({"list": L.key, "model": c.model, "arm": arm, "column_complete": int(c.complete),
                         "finished": s["finished"], "scored": s["scored"], "unscored": s["unscored"],
                         "not_finished": s["left"], "solved": s["solved"], "solved_wilson_low": r4(w[0]),
                         "solved_wilson_high": r4(w[1]), "clean": s["clean"], "clean_wilson_low": r4(cw[0]),
                         "clean_wilson_high": r4(cw[1]), "false_claims": s["false_claims"],
                         "mean_requests": r4(s["requests"]), "mean_steps": r4(s["steps"]),
                         "mean_wall_s": r4(s["wall_s"]), "mean_usd_list": r4(s["usd"]),
                         "usd_sum": r4(s["usd_sum"]), "mean_tokens_in": r4(s["tokens_in"]),
                         "mean_tokens_cached": r4(s["tokens_cached"]), "mean_tokens_out": r4(s["tokens_out"]),
                         "cached_share": r4(s["cached_share"]), "failed_requests": s["failed"],
                         **{k: v for k, v in c.paired("solved").items() if k in ("both", "with_only", "without_only",
                                                                                  "neither")},
                         "paired_cells": c.paired("solved")["pairs"], "sign_p_solved": c.paired("solved")["p"]})
    return csv_text(SUM_HEAD[:-2], rows)


def ext_paired_csv(L, order):
    rows = []
    for t in order:
        if t["list"] is not L:
            continue
        pr, nc = t["pr"], t["pr"]["newcombe"] or (None, None)
        rows.append({"list": L.key, "model": t["model"], "measure": t["key"], "column_complete": int(t["col"].complete),
                     "paired_cells": pr["pairs"], "with_only": pr["with_only"], "without_only": pr["without_only"],
                     "both": pr["both"], "neither": pr["neither"], "difference": r4(pr["diff"]),
                     "newcombe_low": r4(nc[0]), "newcombe_high": r4(nc[1]), "sign_p": pr["p"],
                     "holm_threshold": r4(t["threshold"]), "passes_holm": int(t["passes"]), "verdict": t["verdict"]})
    return csv_text(PAIR_HEAD, rows)


def ext_task_rows_csv(L):
    tr = []
    for col in L.cols:
        for row, cs in L.rows.items():
            d = {"model": col.model, "task_row": row, "start_states": len(cs)}
            for arm in ("with", "without"):
                for key, _ in MEASURES:
                    d[f"{key}_{arm}"] = L.count(col, arm, key, cs)
            tr.append(d)
    return csv_text(["model", "task_row", "start_states"] + [f"{k}_{a}" for a in ("with", "without")
                                                            for k, _ in MEASURES], tr)


def unscored_lines(L, name):
    if not L.uns:
        return ["None."]
    lost = {}
    for x in L.lost:
        lost.setdefault((x["model"], x["arm"], x["cell"]), []).append(x)
    T = ["| model | arm | start state | attempts | reason | USD of its attempts |", "|---|---|---|---|---|---|"]
    for u in L.uns:
        T.append(f"| {u['model']} | {u['arm']} {name} | {u['cell']} | {u['attempts']} | {u['reason']} | "
                 f"{float(u['usd_attempts']):.2f} |")
    n_arm = {}
    for u in L.uns:
        n_arm[(u["model"], u["arm"])] = n_arm.get((u["model"], u["arm"]), 0) + 1
    most = max(n_arm.values())
    T += ["", f"Each one is a failure for its arm: not solved, not clean, no false claim. It stays in the count "
          f"({L.N}), enters the paired tests as a failure, and the scored-pairs table prints every test without it; "
          f"the first verdict rules. The most unscored in one arm is {most} of {L.N} "
          f"({100 * most / L.N:.1f} percent), under the 5 percent at which a column's heading would carry the count."]
    return T


def write_ext_list(L, lists, order, settled, name, grid_cols, check, changed):
    P = {x.key: x for x in lists}.get("list2")
    write(os.path.join(L.folder, "summary.csv"), ext_summary_csv(L, order, L.folder), check, changed)
    write(os.path.join(L.folder, "paired.csv"), ext_paired_csv(L, order), check, changed)
    write(os.path.join(L.folder, "task_rows.csv"), ext_task_rows_csv(L), check, changed)
    l10 = {x.key: x for x in lists}.get("libero10")
    title = (f"# Summary of list 2: {L.N} further start states of LIBERO-PRO, {len(L.cols)} models, both arms"
             if L.key == "list2" else
             f"# Summary of the LIBERO-10 list: {L.N} start states, {len(L.cols)} models, both arms")
    done = "Every start state of this list has ended in both arms of both models." if L.complete else \
        ("THIS LIST WAS STILL RUNNING AT THIS READ: " + "; ".join(
            f"{c.model} ({c.stats('without')['finished']} without, {c.stats('with')['finished']} with)"
            for c in L.cols if not c.complete) + ".")
    T = [title, "", f"Computed by ../recompute.py from this folder's CSV files and those of the grid (../) and the "
         f"other extension folders, as read at {L.info['read']}. {done} The two arms: `with` is the model on {name}'s tools; `without`, "
         f"the plain interface, is the same model on the published three-tool interface.", "",
         "Read these lines before any number here:", ""]
    news = bad_news(name, lists, order, grid_cols)
    T += s_lines(name, libero10=True, pool=(L.key == "list2" or any(FIELD_500 in x for x in news)))
    if l10:
        T += ["", l10_size_line(l10)]
    T += ["", "## Bad news first", ""] + news
    T += ["", "## Every model, both arms", ""] + arms_table(L, name)
    T += ["", "Wilson intervals are over the start states of the list (scored plus unscored). A false claim is an "
          "episode that ends with the model saying it succeeded on a start state that was not solved; per unsolved "
          "episode divides them by the arm's start states that were not solved (unscored counted as not solved)."]
    T += ["", "## The family of twelve tests (two models, two lists, three measures), Holm's step-down at 0.05", "",
          "Exact two-sided sign test on the discordant start states, paired by start state, one model and one list "
          "each; fixed with the reading rule before the first row of either extension list. For FALSE CLAIM the "
          "favourable side is fewer. The LIBERO-10 rows carry the LIBERO-10 line above and the bad news on its tasks "
          "4 and 6.", ""]
    T += family_table(order, settled)
    T += ["", "## The same tests on this list, scored pairs only (the first verdict rules)", ""] + scored_pairs_table(L, order)
    T += ["", f"## The {unit_of(L)} as the unit (descriptive, in no family)", "",
          f"Start states of one {'task row' if L.bench != 'LIBERO-10' else 'task'} are not independent, so the count "
          f"of {unit_of(L)} where the arm with {name} has more of the measure, fewer and the same is printed beside "
          f"every test, with an exact sign test on the untied ones. Where a start-state test passes Holm but this p "
          f"is above 0.05, a sentence must give the count.", ""] + row_table(L, order)
    T += ["", f"Every {'task row' if L.bench != 'LIBERO-10' else 'task'}, both models (with / without, of the "
          f"start states in it; printed whole, rows at zero included; also in task_rows.csv). A split read off this "
          f"table is exploratory, not a finding.", ""] + every_row_table(L)
    if L.key == "list2":
        T += ["", "## By suite and perturbation column (counts and Wilson 95 only, no test)", "",
              "| column | tasks | start states | model | arm | solved (Wilson 95) | clean (Wilson 95) | false claims "
              "(Wilson 95) |", "|---|---|---|---|---|---|---|---|"]
        groups = {}
        for x in L.cells:
            groups.setdefault((x["suite"], x["perturbation"]), []).append(x)
        for (suite, pert), xs in groups.items():
            cs = [x["cell"] for x in xs]
            colname = f"{suite.replace('libero_', '').capitalize()} by {'position' if pert == 'swap' else 'task'}"
            for c in L.cols:
                for arm in ("with", "without"):
                    k = [L.count(c, arm, key, cs) for key, _ in MEASURES]
                    T.append(f"| {colname} | {len({x['task'] for x in xs})} | {len(cs)} | {c.model} | {arm} {name} | "
                             + " | ".join(f"{v} ({w1(wilson(v, len(cs)))})" for v in k) + " |")
        g = {c.model: c for c in (grid_cols or [])}
        gp = [x["cell"] for x in L.cells if x["suite"] == "libero_goal" and x["perturbation"] == "swap"]
        T += ["", "## Goal by position, the one column holding all ten tasks: this list, the grid's part, and the two "
              "together", "", "With the pool's line: " + POOL_CAVEAT, "",
              "| model | arm | list 2: solved, clean (Wilson 95) | the grid: solved, clean (Wilson 95) | together: "
              "solved, clean (Wilson 95) | the two parts' intervals overlap |", "|---|---|---|---|---|---|"]
        for c in L.cols:
            gc = g.get(c.model)
            if not gc or not gc.complete:
                T.append(f"| {c.model} | | | the grid column is not complete: no pool | | |")
                continue
            gg = [x["cell"] for x in gc.cells if x["suite"] == "libero_goal" and x["perturbation"] == "swap"]
            for arm in ("with", "without"):
                part = []
                ov = []
                for cs, col in ((gp, c), (gg, gc)):
                    part.append(", ".join(f"{col_count(col, arm, k, cs)} ({w1(wilson(col_count(col, arm, k, cs), len(cs)))})"
                                          for k in ("solved", "clean")))
                tot = []
                for k in ("solved", "clean"):
                    a1, a2 = col_count(c, arm, k, gp), col_count(gc, arm, k, gg)
                    w_a, w_b = wilson(a1, len(gp)), wilson(a2, len(gg))
                    ov.append(not (w_a[1] < w_b[0] or w_b[1] < w_a[0]))
                    tot.append(f"{a1 + a2} ({w1(wilson(a1 + a2, len(gp) + len(gg)))})" if ov[-1] else
                               "the two lists disagree: not pooled")
                T.append(f"| {c.model} | {arm} {name} | {part[0]} | {part[1]} | {', '.join(tot)} | solved "
                         f"{'yes' if ov[0] else 'NO'}, clean {'yes' if ov[1] else 'NO'} |")
        T += ["", f"Goal by position here: {len(gp)} start states of list 2 and the grid's "
              f"{len([x for x in (g.get(MAIN_MODEL).cells if g.get(MAIN_MODEL) else []) if x['suite'] == 'libero_goal' and x['perturbation'] == 'swap'])}, "
              f"20 start states a task over the two lists."]
        T += ["", "## The two lists together, 500 start states an arm (an estimate, not a test)", "",
              "With the pool's line: " + POOL_CAVEAT + " " + FIELD_500 + ". No "
              "sentence rests on the pooled p; where a test is needed, the grid's main test or this list's own test "
              "is cited.", "",
              "| model | measure | with, of 500 (Wilson 95) | without, of 500 (Wilson 95) | difference, points "
              "(Newcombe 95) | with only | without only | pooled p (contains the main test's 100 start states; not an "
              "independent test; not in any family) |", "|---|---|---|---|---|---|---|---|"]
        parts = ["| model | measure | the grid: with (Wilson 95) | the grid: without (Wilson 95) | list 2: with "
                 "(Wilson 95) | list 2: without (Wilson 95) | the parts' intervals overlap: with, without |",
                 "|---|---|---|---|---|---|---|"]
        for c in L.cols:
            gc = g.get(c.model)
            if not gc or not gc.complete:
                T.append(f"| {c.model} | | the grid column is not complete: no pool | | | | | |")
                continue
            for key, word in MEASURES:
                a1, a2 = gc.paired(key), c.paired(key)
                e, f, gg2, h = (a1[k] + a2[k] for k in ("both", "with_only", "without_only", "neither"))
                n = e + f + gg2 + h
                kw1, ko1 = a1["both"] + a1["with_only"], a1["both"] + a1["without_only"]
                kw2, ko2 = a2["both"] + a2["with_only"], a2["both"] + a2["without_only"]
                ovs = []
                for k1, k2 in ((kw1, kw2), (ko1, ko2)):
                    x1, x2 = wilson(k1, a1["pairs"]), wilson(k2, a2["pairs"])
                    ovs.append(not (x1[1] < x2[0] or x2[1] < x1[0]))
                cw = f"{e + f} ({w1(wilson(e + f, n))})" if ovs[0] else "the two lists disagree: not pooled"
                co = f"{e + gg2} ({w1(wilson(e + gg2, n))})" if ovs[1] else "the two lists disagree: not pooled"
                T.append(f"| {c.model} | {word} | {cw} | {co} | {d2(f, gg2, n)} ({nc1(newcombe_paired(e, f, gg2, h))}) | "
                         f"{f} | {gg2} | {p3(sign_p(f, gg2))} |")
                parts.append(f"| {c.model} | {word} | {kw1} ({w1(wilson(kw1, a1['pairs']))}) | {ko1} "
                             f"({w1(wilson(ko1, a1['pairs']))}) | {kw2} ({w1(wilson(kw2, a2['pairs']))}) | {ko2} "
                             f"({w1(wilson(ko2, a2['pairs']))}) | {'yes' if ovs[0] else 'NO'}, "
                             f"{'yes' if ovs[1] else 'NO'} |")
        T += ["", "The two parts apart:", ""] + parts
        T += ["", "## How list 2 relates to the grid's tests (no statement counted twice)", "",
              "The grid's tests are read on the grid's 100 start states only, by the grid's rule; no list 2 start "
              "state enters them, and no grid start state enters the family above.", ""]
        T += replicated_section(name, L, order, grid_cols)
    T += ["", f"## The one cross-model pair named beforehand: {MAIN_MODEL} with {name} against {CLEAN_MODEL} without "
          "(descriptive, outside the corrected family)", ""] + cross_lines(L, name)
    T += ["", "## Requests, steps, tokens and cost (means over scored rows; USD at the list prices of ../prices.csv)",
          ""] + money_table(L, name)
    T += ["", "USD a start state and USD a solved start state count the scored rows and the attempts in no row "
          "(lost_attempts.csv) over all start states of the arm, or over its solved ones. USD is list price only, not "
          "a billed amount; part of the gap between the arms is caching (the cached share of input tokens), not only "
          "fewer requests. Seconds are wall clock from the episode's start to its end and include the time an endpoint "
          "stalled or refused requests; a failed request was sent again inside its episode."]
    st = stopped_lines(L, name)
    if st:
        T += [""] + st
    T += (["", "## How the replies ended (finish reason), scored episodes", ""]
          + finish_table(L.cols, name, L.refused)[:-1])
    T += ["", "## Episodes that ended without a score", ""] + unscored_lines(L, name)
    T += ["", "## What these numbers allow a sentence to say", "",
          "Each sentence after the lines at the top of this file, with the LIBERO-10 line wherever a LIBERO-10 number "
          f"is in it, and with the bad news beside the numbers it qualifies. LIBERO-10 sentences carry {l10.N if l10 else 395}, "
          "the size of that list.", ""] + sentences(name, lists, order, grid_cols, L)
    T += ["", NOT_SAID.format(name=name), ""]
    write(os.path.join(L.folder, "SUMMARY.md"), "\n".join(T), check, changed)


def col_count(col, arm, key, cells):
    return sum(col.val(arm, x, key) for x in cells)


def development(name, check, changed):
    folder = os.path.join(HERE, DEV[1])
    if not os.path.isdir(folder):
        return
    cells, cols, uns = columns(folder)
    info = json.load(open(os.path.join(folder, "list_info.json")))
    cols = [c for m in FAMILY_MODELS for c in cols if c.model == m] + [c for c in cols if c.model not in FAMILY_MODELS]
    N = len(cells)
    rows = []
    for c in cols:
        for arm in ("without", "with"):
            s = c.stats(arm)
            rows.append({"list": "libero10_development", "model": c.model, "arm": arm,
                         "column_complete": int(c.complete), "finished": s["finished"], "scored": s["scored"],
                         "unscored": s["unscored"], "not_finished": s["left"], "solved": s["solved"],
                         "clean": s["clean"], "false_claims": s["false_claims"]})
    write(os.path.join(folder, "summary.csv"), csv_text(
        ["list", "model", "arm", "column_complete", "finished", "scored", "unscored", "not_finished", "solved",
         "clean", "false_claims"], rows), check, changed)
    done = all(c.complete for c in cols)
    T = [f"# Development cells, not a result: {N} LIBERO-10 start states, {len(cols)} models, both arms", "",
         f"Computed by ../recompute.py from this folder's CSV files as read at {info['read']}. "
         + ("Every start state here has ended in both arms of both models." if done else
            "NOT EVERY START STATE HERE HAD ENDED AT THIS READ."), "",
         "Read this line before any number here:", "", S2_LINE, "",
         f"These are start states 0 to 3 of each LIBERO-10 task, the start states {name} was developed on. They are "
         f"in no test, no pool, no headline and no chart, and nothing here is a result: counts only.", "",
         "## Development cells, not a result", "",
         "| model | arm | solved | clean | false claims | unscored | scored |", "|---|---|---|---|---|---|---|"]
    for c in cols:
        for arm in ("with", "without"):
            s = c.stats(arm)
            T.append(f"| {c.model} | {arm} {name} | {s['solved']} of {N} | {s['clean']} of {N} | "
                     f"{s['false_claims']} of {N} | {s['unscored']} | {s['scored']} |")
    T += ["", "The sentence the reading rule allows, and nothing else: " + "; ".join(
        f"\"On {N} development start states (not a result): {c.stats('with')['solved']} against "
        f"{c.stats('without')['solved']}.\" ({c.model})" for c in cols) + ".", ""]
    if uns:
        T += ["Ended without a score (a failure for its arm in the counts above): " + "; ".join(
            f"{u['model']} {u['arm']} {name} {u['cell']}, {u['attempts']} attempts: {u['reason']}" for u in uns)
            + ".", ""]
    write(os.path.join(folder, "SUMMARY.md"), "\n".join(T), check, changed)


def extension(name, check, changed, grid_cols):
    lists = [ExtList(*e) for e in EXT if os.path.isdir(os.path.join(HERE, e[1]))]
    if lists:
        order, settled = family(lists)
        for L in lists:
            write_ext_list(L, lists, order, settled, name, grid_cols, check, changed)
    development(name, check, changed)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="Cerulion", help="the product name printed in the tables")
    ap.add_argument("--check", action="store_true")
    ns = ap.parse_args()
    changed = []
    cols = grid(ns.name, ns.check, changed)
    extension(ns.name, ns.check, changed, cols)
    # the cell list's own hash, recomputed from cells.csv: a row that no longer matches fails the run, with or
    # without --check, since no output can be computed again from a changed list
    import hashlib
    broken = []
    for folder in [HERE] + [os.path.join(HERE, e[1]) for e in EXT] + [os.path.join(HERE, DEV[1])]:
        if os.path.exists(os.path.join(folder, "cells.csv")):
            bad = 0
            for c in load(folder, "cells.csv"):
                canon = "|".join(c[k] for k in ("cell", "suite", "perturbation", "task", "start_state", "sentence",
                                                "start_state_sha256"))
                bad += hashlib.sha256(canon.encode()).hexdigest() != c["cell_sha256"]
            print(f"{os.path.relpath(folder, HERE)}: cells.csv, {bad} rows whose hash does not match")
            if bad:
                broken.append(f"{os.path.relpath(os.path.join(folder, 'cells.csv'), HERE)} ({bad} rows whose hash "
                              f"does not match)")
        # every reply of a scored attempt has one finish reason, so the five replies columns add up to requests;
        # a row where they do not (or one left blank) fails the run, with or without --check
        if os.path.exists(os.path.join(folder, "episodes.csv")):
            off = 0
            for e in load(folder, "episodes.csv"):
                v = [e.get(f"replies_{k}") or "" for k in FINISH]
                off += not all(x.isdigit() for x in v) or not e["requests"].isdigit() \
                    or sum(map(int, v)) != int(e["requests"])
            print(f"{os.path.relpath(folder, HERE)}: episodes.csv, {off} scored rows whose replies by finish reason do "
                  f"not add up to requests")
            if off:
                broken.append(f"{os.path.relpath(os.path.join(folder, 'episodes.csv'), HERE)} ({off} scored rows "
                              f"whose replies by finish reason do not add up to requests)")
    if ns.check:
        print("differs from the CSV files: " + (", ".join(changed + broken) if changed or broken else "nothing"))
        return 1 if changed or broken else 0
    print("written: " + (", ".join(changed) if changed else "nothing changed"))
    if broken:
        print("the CSV files fail their own checks: " + ", ".join(broken))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
