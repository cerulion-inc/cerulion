#!/usr/bin/env python3
"""Recompute every count, interval and test of this package from its CSV files.

Reads cells.csv, episodes.csv, unscored.csv and not_finished.csv in this folder and writes summary.csv,
paired.csv and SUMMARY.md. If extension_list2/ is present it does the same there (plus task_rows.csv), by that
list's own rules. The rules are in RULES.md. Standard library only.

usage: recompute.py [--name NAME] [--check]
  --check  writes nothing; exits 1 if the files on disk differ from what the CSV files give
"""
import argparse, csv, json, math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ALPHA = 0.05
MAIN_MODEL = "Gemini 3.8 Flash"
CLEAN_MODEL = "GPT-6 Astra"
ADDED_LATE = ["GPT-6.1 Terra"]          # a column added after the grid's first rows had been seen
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
    return "" if p is None else (f"{p:.2g}" if p < 0.001 else f"{p:.3f}".rstrip("0").rstrip("."))


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
                "requests": mean("requests"), "steps": mean("steps"), "operator_s": mean("operator_s"),
                "usd": mean("usd_list"), "usd_sum": tot("usd_list"),
                "usd_per_solved": (tot("usd_list") / solved) if solved else None,
                "tokens_in": mean("tokens_in"), "tokens_cached": mean("tokens_cached"),
                "tokens_out": mean("tokens_out")}

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
    """The wording of RULES.md for one test of a Holm family."""
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
    L = ["| model | arm | scored episodes | replies | control steps | operator seconds | USD, list price | "
         "tokens in | of which cached | tokens out | USD per solved start state |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in cols:
        for arm in ("without", "with"):
            s = c.stats(arm)
            L.append(f"| {c.model}{'' if c.complete else '*'} | {arm} {name} | {s['scored']} | {fnum(s['requests'])} | "
                     f"{fnum(s['steps'])} | {fnum(s['operator_s'])} | {fnum(s['usd'], 3)} | {fnum(s['tokens_in'], 0)} | "
                     f"{fnum(s['tokens_cached'], 0)} | {fnum(s['tokens_out'], 0)} | "
                     f"{'none solved' if s['usd_per_solved'] is None else fnum(s['usd_per_solved'], 2)} |")
    return L


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
                         "mean_operator_s": r4(s["operator_s"]), "mean_usd_list": r4(s["usd"]),
                         "usd_sum": r4(s["usd_sum"]), "mean_tokens_in": r4(s["tokens_in"]),
                         "mean_tokens_cached": r4(s["tokens_cached"]), "mean_tokens_out": r4(s["tokens_out"]),
                         "paired_cells": pr["pairs"], "with_only": pr["with_only"],
                         "without_only": pr["without_only"], "both": pr["both"], "neither": pr["neither"],
                         "sign_p_solved": pr["p"], "holm_threshold": None if not t else r4(t[0]),
                         "verdict_solved": verdict(pr, t, "solved") if c.complete else
                         f"incomplete at the read of {READ[list_name]}: {pr['pairs']} of {len(c.cells)}"})
    return rows


SUM_HEAD = ["list", "model", "arm", "column_complete", "finished", "scored", "unscored", "not_finished", "solved",
            "solved_wilson_low", "solved_wilson_high", "clean", "clean_wilson_low", "clean_wilson_high",
            "false_claims", "mean_requests", "mean_steps", "mean_operator_s", "mean_usd_list", "usd_sum",
            "mean_tokens_in", "mean_tokens_cached", "mean_tokens_out", "paired_cells", "with_only", "without_only",
            "both", "neither", "sign_p_solved", "holm_threshold", "verdict_solved"]
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
            else:
                v = verdict(pr, t, key)
            rows.append({"list": list_name, "model": c.model, "measure": key, "column_complete": int(c.complete),
                         "paired_cells": pr["pairs"], "with_only": pr["with_only"],
                         "without_only": pr["without_only"], "both": pr["both"], "neither": pr["neither"],
                         "difference": r4(pr["diff"]), "newcombe_low": r4(nc[0]), "newcombe_high": r4(nc[1]),
                         "sign_p": pr["p"], "holm_threshold": None if not t else r4(t[0]),
                         "passes_holm": None if not t else int(t[1]), "verdict": v})
    return rows


def grid(name, check, changed):
    cells, cols, uns = columns(HERE)
    if not cols:
        return None
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
    if len(done) == len(cols):
        L += [f"Every model has finished all {N} start states in both arms.", ""]
    else:
        L += [f"THE GRID WAS STILL RUNNING AT THIS READ. {len(done)} of {len(cols)} models had finished all {N} "
              f"start states in both arms. A model marked * is incomplete: its counts are of the start states that "
              f"had ended, with n printed, and it gets no verdict. Start states that end first are the short "
              f"episodes, so an incomplete row is not a fair sample of its list. Incomplete: "
              + "; ".join(f"{c.model} ({c.stats('without')['finished']} without, {c.stats('with')['finished']} with)"
                          for c in cols if not c.complete) + ".", ""]
    L += ["Every start state here is a new start state of a task row that " + name + " was developed on at other "
          "start states. The tasks themselves are not new to it.", "",
          f"Arms: `without` is the same model on a published three-tool interface; `with` is the same model on "
          f"{name}'s tools. Same start states, same budgets (80 replies, one tool call a reply, 900 control steps), "
          f"one episode a start state an arm. An episode that ended without a score counts as a failure for its arm "
          f"and stays in the count.", ""]
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
              ("The test passes at 0.05." if pr["p"] is not None and pr["p"] <= ALPHA and
               pr["with_only"] > pr["without_only"] else "The test does not pass at 0.05.")
              if mc.complete else "No verdict before the column is complete.", ""]
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
    a = sum(c.paired("false_claim")["without_only"] for c in cols)
    b = sum(c.paired("false_claim")["with_only"] for c in cols)
    L += [f"2. Fewer false claims, pooled over every model that ran both arms, one sign test over all discordant "
          f"pairs of (model, start state){'' if len(done) == len(cols) else ' (INCOMPLETE: it pools the pairs ended so far)'}: "
          f"a false claim without {name} only on {a} pairs, with {name} only on {b} pairs, p = {fp(sign_p(a, b))}. "
          f"The rate per unsolved episode is in the table below for each model and arm.", ""]
    # per model
    L += ["## Every model, both arms", ""] + arm_table(cols, name, N) + [""]
    L += ["Wilson intervals are over the finished start states of the arm (scored plus unscored). A false claim is "
          "an episode that ends with the model saying it succeeded on a start state that was not solved; per "
          "unsolved episode divides them by the arm's finished episodes that were not solved.", ""]
    L += [f"## Paired by start state, and the per-model family ({len(cols)} models, Holm's step-down at 0.05)", "",
          "The family is every model that ran, the main test's model included, however each came out; the main "
          "test above is read alone at 0.05 and does not depend on this table. Thresholds are computed from the "
          "p values of all the models as they stand, so they can move until the last column completes."
          + ("" if len(done) == len(cols) else " Until then the verdict printed for a complete column can change "
             "with them."), ""]
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
                                    f"p = {fp(c.paired('solved')['p'])})" for c in behind) + "." if behind else ""), ""]
    L += ["### FALSE CLAIM (descriptive for each model; the declared test is the pooled one above)", "",
          "| model | paired start states | false claim without only | with only | exact p |", "|---|---|---|---|---|"]
    for c in cols:
        pr = c.paired("false_claim")
        L.append(f"| {c.model}{'' if c.complete else '*'} | {pr['pairs']} | {pr['without_only']} | "
                 f"{pr['with_only']} | {fp(pr['p'])} |")
    L += ["", "## Means per scored episode", ""] + means_table(cols, name) + [""]
    L += ["Operator seconds are wall clock from the operator's start to its end, tool execution and failed "
          "requests included. USD is the model's reported token counts of the scored attempt at the list prices of "
          "prices.csv on the date in that file; cached input is priced at the cached rate; simulator compute is "
          "not included; nothing here is a billed amount. Tokens in counts every prompt token, cached ones "
          "included; tokens out counts reasoning tokens.", ""]
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


def extension(name, check, changed, grid_cols):
    folder = os.path.join(HERE, "extension_list2")
    if not os.path.isdir(folder):
        return
    cells, cols, uns = columns(folder)
    N = len(cells)
    info = json.load(open(os.path.join(folder, "list_info.json")))
    READ["list2"] = info["read"]
    fam = {key: holm({c.model: c.paired(key)["p"] for c in cols}) for key, _ in MEASURES}
    TESTS, STRICT = 12, ALPHA / 12
    write(os.path.join(folder, "summary.csv"), csv_text(SUM_HEAD[:-2], [
        {k: v for k, v in r.items() if k in SUM_HEAD[:-2]} for r in summary_rows(cols, fam, "list2")]), check, changed)
    prs = paired_rows(cols, {}, "list2", settled=False)
    for r in prs:
        r["holm_threshold"] = r4(STRICT)
        r["passes_holm"] = None
    write(os.path.join(folder, "paired.csv"), csv_text(PAIR_HEAD, prs), check, changed)
    rows_of = {}
    for c in cells:
        rows_of.setdefault(c["task_row"], []).append(c["cell"])
    tr = []
    for col in cols:
        for row, cs in rows_of.items():
            d = {"model": col.model, "task_row": row, "start_states": len(cs)}
            for arm in ("with", "without"):
                for key, _ in MEASURES:
                    d[f"{key}_{arm}"] = sum(col.val(arm, c, key) for c in cs)
            tr.append(d)
    write(os.path.join(folder, "task_rows.csv"), csv_text(
        ["model", "task_row", "start_states"] + [f"{k}_{a}" for a in ("with", "without") for k, _ in MEASURES], tr),
        check, changed)
    L = [f"# Summary of list 2: {N} further start states, {len(cols)} models, both arms", "",
         f"Computed by ../recompute.py from this folder's CSV files as read at {info['read']}. Every start state of "
         f"this list has ended in both arms of both models.", "",
         "Read these three lines before any number here:", "",
         f"- These are new start states of task rows {name} was developed on at other start states. The tasks "
         f"themselves are not new to it.",
         f"- This {N}-start-state list was added after the first rows of the 100-start-state grid had been scored "
         f"and seen. The 100-start-state test declared beforehand remains the main result.",
         "- The reading rule for this list was fixed before its first row. It has a family of twelve tests: two "
         "models, this list and a second list on another suite, three measures. Holm's step-down at 0.05 over "
         f"the twelve; the strictest threshold is 0.05 / 12 = {STRICT:.5f}. The second list is not in this "
         "package, so no test here carries a family verdict. A p at or under the strictest threshold passes "
         "whatever the other tests show.", "",
         "## Every model, both arms", ""] + arm_table(cols, name, N) + ["",
         "## The six tests on this list", "",
         "Exact two-sided sign test on the discordant start states, paired by start state. For FALSE CLAIM the "
         "favourable side is fewer.", "",
         "| model | measure | with | without | with only | without only | difference, points (Newcombe 95) | "
         "exact p | at or under 0.05 / 12 | scored pairs only: with only, without only, p | task rows: with "
         "ahead, behind, tied, p |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in cols:
        for key, word in MEASURES:
            pr, so = c.paired(key), c.paired(key, scored_only=True)
            w = sum(c.val("with", x["cell"], key) for x in cells)
            p = sum(c.val("without", x["cell"], key) for x in cells)
            up = dn = 0
            for row, cs in rows_of.items():
                a, b = sum(c.val("with", x, key) for x in cs), sum(c.val("without", x, key) for x in cs)
                up += a > b; dn += a < b
            L.append(f"| {c.model} | {word} | {w} of {N} | {p} of {N} | {pr['with_only']} | {pr['without_only']} | "
                     f"{100 * pr['diff']:+.1f} ({pct(pr['newcombe'])}) | {fp(pr['p'])} | "
                     f"{'yes' if pr['p'] is not None and pr['p'] <= STRICT else 'no'} | {so['with_only']}, "
                     f"{so['without_only']}, {fp(so['p'])} | {up}, {dn}, {len(rows_of) - up - dn}, {fp(sign_p(up, dn))} |")
    L += ["", f"The task row columns count the {len(rows_of)} task rows where the arm with {name} has more of the "
          "measure, fewer, and the same; start states of one task row are not independent, so read the row counts "
          "beside the start-state test. Every row is in task_rows.csv.", "",
          "## False claims per unsolved episode", "",
          "| model | arm | false claims | unsolved episodes | per unsolved episode |", "|---|---|---|---|---|"]
    for c in cols:
        for arm in ("without", "with"):
            s = c.stats(arm)
            L.append(f"| {c.model} | {arm} {name} | {s['false_claims']} | {s['unsolved']} | "
                     f"{'' if s['false_claims_per_unsolved'] is None else format(s['false_claims_per_unsolved'], '.2f')} |")
    L += ["", "## By suite and perturbation (counts and intervals only, no test)", "",
          "| model | suite, perturbation | start states | with: solved (Wilson 95) | without: solved (Wilson 95) |",
          "|---|---|---|---|---|"]
    groups = {}
    for x in cells:
        groups.setdefault(f"{x['suite']}, {x['perturbation']}", []).append(x["cell"])
    for c in cols:
        for gname, cs in groups.items():
            a, b = sum(c.val("with", x, "solved") for x in cs), sum(c.val("without", x, "solved") for x in cs)
            L.append(f"| {c.model} | {gname} | {len(cs)} | {a} ({pct(wilson(a, len(cs)))}) | {b} ({pct(wilson(b, len(cs)))}) |")
    by = {c.model: c for c in cols}
    if MAIN_MODEL in by and CLEAN_MODEL in by:
        L += ["", f"## The one cross-model pair named beforehand (outside the family): {MAIN_MODEL} with {name} "
              f"against {CLEAN_MODEL} without", ""]
        for key, word in MEASURES[:2]:
            pr = by[MAIN_MODEL].paired(key, other=by[CLEAN_MODEL])
            a = sum(by[MAIN_MODEL].val("with", x["cell"], key) for x in cells)
            b = sum(by[CLEAN_MODEL].val("without", x["cell"], key) for x in cells)
            L.append(f"- {word}: {a} of {N} against {b} of {N}; first only {pr['with_only']}, second only "
                     f"{pr['without_only']}, p = {fp(pr['p'])} ("
                     f"{'no difference shown' if pr['p'] is None or pr['p'] > ALPHA else 'differs; outside the corrected family'}).")
    L += ["", "## Means per scored episode", ""] + means_table(cols, name) + [""]
    # the pool of 500
    g = {c.model: c for c in (grid_cols or [])}
    L += ["## The two lists together (an estimate, not a test)", "",
          "A pool of two lists: 100 start states whose first rows had been seen when the other 400 were added; 25 "
          "task rows, 20 start states each, six suite by perturbation columns; not the published 10 tasks x 50 of "
          "one column.", ""]
    L += ["| model | measure | arm | grid 100 (Wilson 95) | list 2 (Wilson 95) | together (Wilson 95) | with only, "
          "without only | difference, points (Newcombe 95) | pooled p (contains the main test's 100 start states; "
          "not an independent test; not in any family) |", "|---|---|---|---|---|---|---|---|---|"]
    for c in cols:
        gc = g.get(c.model)
        if not gc or not gc.complete:
            L.append(f"| {c.model} | | | the grid column is not complete: no pool | | | | | |")
            continue
        for key, word in MEASURES:
            a1, a2 = gc.paired(key), c.paired(key)
            e, f, gg, h = (a1[k] + a2[k] for k in ("both", "with_only", "without_only", "neither"))
            n = e + f + gg + h
            for arm, k in (("with", e + f), ("without", e + gg)):
                k1 = sum(gc.val(arm, x["cell"], key) for x in gc.cells)
                k2 = sum(c.val(arm, x["cell"], key) for x in cells)
                w1, w2 = wilson(k1, len(gc.cells)), wilson(k2, N)
                apart = w1[1] < w2[0] or w2[1] < w1[0]
                both = "the two lists disagree: not pooled" if apart else f"{k} of {n} ({pct(wilson(k, n))})"
                tail = (f"{f}, {gg} | {100 * (f - gg) / n:+.1f} ({pct(newcombe_paired(e, f, gg, h))}) | "
                        f"{fp(sign_p(f, gg))}") if arm == "with" else " | | "
                L.append(f"| {c.model} | {word} | {arm} {name} | {k1} of {len(gc.cells)} ({pct(w1)}) | "
                         f"{k2} of {N} ({pct(w2)}) | {both} | {tail} |")
    L += ["", "## Episodes that ended without a score", "",
          ("None." if not uns else "; ".join(f"{u['model']} {u['arm']} {u['cell']}: {u['reason']}" for u in uns)), ""]
    write(os.path.join(folder, "SUMMARY.md"), "\n".join(L), check, changed)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="Cerulion", help="the product name printed in the tables")
    ap.add_argument("--check", action="store_true")
    ns = ap.parse_args()
    changed = []
    cols = grid(ns.name, ns.check, changed)
    extension(ns.name, ns.check, changed, cols)
    # the cell list's own hash, recomputed from cells.csv
    for folder in (HERE, os.path.join(HERE, "extension_list2")):
        if os.path.exists(os.path.join(folder, "cells.csv")):
            import hashlib
            bad = 0
            for c in load(folder, "cells.csv"):
                canon = "|".join(c[k] for k in ("cell", "suite", "perturbation", "task", "start_state", "sentence",
                                                "start_state_sha256"))
                bad += hashlib.sha256(canon.encode()).hexdigest() != c["cell_sha256"]
            print(f"{os.path.relpath(folder, HERE)}: cells.csv, {bad} rows whose hash does not match")
    if ns.check:
        print("differs from the CSV files: " + (", ".join(changed) if changed else "nothing"))
        return 1 if changed else 0
    print("written: " + (", ".join(changed) if changed else "nothing changed"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
