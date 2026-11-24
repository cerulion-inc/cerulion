#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Draw the two harness benchmark charts from their results package.

It replots a committed package. It never runs a benchmark and never edits a package.

  docs/media/harness-success-rate.svg       success rate per model, without and with Cerulion
  docs/media/harness-cost-per-success.svg   model cost per successful episode, without and with Cerulion

Every count and cost is computed by the package's own recompute.py from the package's CSV files,
so a figure on a chart is the figure SUMMARY.md prints (cost per success is SUMMARY.md's "USD per
solved start state"). A chart prints no figure the package does not: no ratio of two of its figures, no
total it leaves out. Standard library only: the same package gives the same bytes on every machine.

    python3 docs/media/charts/build_harness_charts.py           # write the two SVGs
    python3 docs/media/charts/build_harness_charts.py --check   # write nothing; exit 1 if a chart differs

The two SVGs are theme-aware sources (see THEME). After a run, write their fixed-palette twins
(docs/media/<name>-light.svg and <name>-dark.svg) with:

    python3 tools/scripts/build_media_theme_variants.py

The form of both charts:
  - one row per model, ordered by the share of finished episodes solved WITHOUT Cerulion, lowest
    first, ties by name; the same order in both charts; no model is marked for its results; a model
    whose column was added after grid rows had been seen (the package's ADDED_LATE) carries a
    dagger, and a footnote says when, in the package's words;
  - the first line under the title is the scope: new start states of task rows Cerulion was developed
    on at other start states, one episode each, the same model in both arms;
  - linear scales; the success axis in percent, each success bar labelled with the package's count, solved of
    finished ("73 of 100", SUMMARY.md's form);
  - the cost scale ends at 30 USD, and a bar past it is drawn to the edge with a break and its value
    printed at its end; an arm that solved nothing reads "none solved", as in SUMMARY.md;
  - no factor column: a ratio of two package figures is a number the package does not print, so each
    row carries only the two arms' own figures;
  - a model whose column had not finished at the package's read carries a star, and a footnote gives
    its finished counts and the read time;
  - the success chart's footer counts the models that solved fewer start states with Cerulion and
    names each under its verdict in the package's per-model SOLVED family (Holm over every model,
    worded by the package's verdict()) with its exact p as the package prints it; a column still
    running gets no verdict, as in the package;
  - the cost chart's footer says what its figure leaves out (the episodes that ended without a score
    and the other attempts in lost_attempts.csv) and for how many models part of the factor is
    caching, in SUMMARY.md's words ("Part of the gap ... is caching"): every model when
    every model's arm without Cerulion had the lower cached share of input, else that count;
  - both footers name every arm that had a reply the provider's content filter stopped (finish reason
    content_filter, the package's filtered_arms()): an arm whose scored episodes ended on such replies by
    name with its counts, the other arms by count, every count of replies over scored and unscored
    episodes together.
"""
import html
import importlib.util
import json
import os
import re
import sys
import textwrap

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
PACKAGE = os.path.join(ROOT, "docs", "benchmarks", "results", "7cb9f322c10f6fa7-2026-11-18-harness-libero-pro")
MEDIA = os.path.join(ROOT, "docs", "media")
NAME = "Cerulion"

WIDTH = 1000
LEFT = 190          # x of the zero line
LABEL_X = 178       # right edge of the model names
TOP = 100           # y of the first row's band under a two-line subtitle; each further line adds SUB_LINE
SUB_LINE = 18       # one subtitle line
ROW = 40            # one model
BAR = 13            # one bar
PCT_FULL = 850      # x of 100 percent
USD_CUT = 30.0      # the cost scale ends here
USD_CUT_X = 810     # x of the cut
USD_BROKEN_W = 650  # a broken bar is drawn this wide
# The charts are theme-aware: every colour is a CSS custom property, with the light value in `:root`
# and the dark value in one `prefers-color-scheme: dark` block. tools/scripts/build_media_theme_variants.py
# writes the fixed `-light` and `-dark` twins the README names in a <picture> block. The dark values are
# for GitHub's dark page (#0d1117): the two arms and every text colour at least 3:1 against it.
THEME = [  # (custom property, light, dark)
    ("paper", "#fff", "#0d1117"),        # the ground, and the white gap that breaks a long bar
    ("ink", "#171717", "#e6edf3"),
    ("value", "#333", "#c9d1d9"),        # the figure at the end of a bar
    ("muted", "#555", "#9da7b3"),
    ("soft", "#666", "#9198a1"),
    ("grid", "#e6e6e6", "#262c36"),
    ("axis", "#8a8a8a", "#6e7681"),
    ("without", "#eb6834", "#df6c32"),
    ("with", "#2a78d6", "#498de5"),
]
PAPER, INK, VALUE, MUTED, SOFT, GRID, AXIS, WITHOUT, WITH = (f"var(--{name})" for name, _, _ in THEME)
NUMBER_WORDS = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five"}


def load_recompute():
    """The package's recompute.py, imported by path; no bytecode is written into the package."""
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location("harness_recompute", os.path.join(PACKAGE, "recompute.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def read_info():
    """(the time the package's rows were read, the number of start states in its list)"""
    with open(os.path.join(PACKAGE, "list_info.json"), encoding="utf-8") as fh:
        info = json.load(fh)
    return info["read"], info["cells"]


def solved_of(s):
    """An arm's count as SUMMARY.md prints it: "73 of 100"."""
    return f"{s['solved']} of {s['finished']}"


def esc(s):
    return html.escape(s, quote=True)


def theme_style():
    """The light values in `:root`, the dark values in one `prefers-color-scheme: dark` block."""
    light = ";".join(f"--{name}:{value}" for name, value, _ in THEME)
    dark = ";".join(f"--{name}:{value}" for name, _, value in THEME)
    return f"<style>:root{{{light}}}@media(prefers-color-scheme:dark){{:root{{{dark}}}}}</style>"


def text(x, y, s, size, fill, anchor=None, weight=None):
    a = f' text-anchor="{anchor}"' if anchor else ""
    w = f' font-weight="{weight}"' if weight else ""
    return f'<text x="{x}" y="{y}" font-size="{size}"{w} style="fill:{fill}"{a}>{esc(s)}</text>'


def bar(y, width, fill, title):
    """A bar from the zero line with its right end rounded."""
    r = min(4.0, width / 2)
    return (f'<path d="M{LEFT:.1f},{y} h{width - r:.1f} a{r:g},{r:g} 0 0 1 {r:g},{r:g} v{BAR - 2 * r:.1f} '
            f'a{r:g},{r:g} 0 0 1 -{r:g},{r:g} h-{width - r:.1f} z" style="fill:{fill}"><title>{esc(title)}</title></path>')


def rows_of(rc):
    _cells, cols, _uns = rc.columns(PACKAGE)
    family = rc.holm({c.model: c.paired("solved")["p"] for c in cols})   # the grid's per-model SOLVED family
    out = []
    for c in cols:
        wo, wi = c.stats("without"), c.stats("with")
        pr = c.paired("solved")
        out.append({"model": c.model, "complete": c.complete, "late": rc.ADDED_LATE.get(c.model), "without": wo,
                    "with": wi, "p": None if pr["p"] is None else rc.fp(pr["p"]),
                    "verdict": rc.verdict(pr, family.get(c.model), "solved") if c.complete else None})
    for r in out:
        if not r["without"]["finished"] or not r["with"]["finished"]:
            raise SystemExit(f"build_harness_charts: {r['model']} has an arm with no finished episode")
    out.sort(key=lambda r: (r["without"]["solved"] / r["without"]["finished"], r["model"]))
    return out


def frame(title, subtitle, ticks, rows, footer):
    n = len(rows)
    shift = SUB_LINE * (len(subtitle) - 2)      # the legend and the rows move down under a longer subtitle
    top = TOP + shift
    bottom = top + ROW * n + 4
    height = bottom + 48 + 18 * len(footer)
    s = [f'<svg viewBox="0 0 {WIDTH} {height}" xmlns="http://www.w3.org/2000/svg" '
         f'font-family="system-ui,-apple-system,Segoe UI,sans-serif" role="img" aria-label="{esc(title)}">',
         theme_style(),
         f'<rect width="{WIDTH}" height="{height}" style="fill:{PAPER}"/>',
         text(24, 34, title, 19, INK, weight="650")]
    s += [text(24, 56 + SUB_LINE * j, line, 13, MUTED) for j, line in enumerate(subtitle)]
    s += [f'<rect x="24" y="{84 + shift}" width="14" height="10" rx="2" style="fill:{WITHOUT}"/>',
          text(44, 93 + shift, f"without {NAME}", 13, INK),
          f'<rect x="170" y="{84 + shift}" width="14" height="10" rx="2" style="fill:{WITH}"/>',
          text(190, 93 + shift, f"with {NAME}", 13, INK)]
    for x, label in ticks:
        s.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{bottom}" style="stroke:{GRID}" stroke-width="1"/>')
        s.append(text(f"{x:.1f}", bottom + 18, label, 12, SOFT, anchor="middle"))
    s.append(f'<line x1="{LEFT:.1f}" y1="{top}" x2="{LEFT:.1f}" y2="{bottom}" style="stroke:{AXIS}" stroke-width="1"/>')
    for i, parts in enumerate(rows):
        y = top + ROW * i
        s.extend(parts(y))
    for j, line in enumerate(footer):
        s.append(text(24, bottom + 44 + 18 * j, line, 12, SOFT))
    s.append("</svg>")
    svg = "".join(s) + "\n"
    body = svg.replace(theme_style(), "", 1)
    if re.search(r"#[0-9a-fA-F]{3,8}\b", body) or re.search(r'\b(?:fill|stroke)="', body):
        raise SystemExit("build_harness_charts: a colour outside THEME")
    return svg


def star_footer(rows, when, cells):
    out = []
    for r in rows:
        if not r["complete"]:
            out.append(f"* Still running at the read of {when}: {r['model']}, {r['without']['finished']} of {cells} "
                       f"start states finished without {NAME} and {r['with']['finished']} with.")
    late = {}
    for r in rows:
        if r["late"]:
            late.setdefault(r["late"], []).append(r["model"])
    if late:
        out.append("\u2020 " + " ".join(f"{', '.join(ms)}: added {after}." for after, ms in late.items()))
    return out


def fewer_footer(rows, joined):
    """The models that solved fewer start states with Cerulion, in the chart's order, each under the verdict the
    package's per-model SOLVED family gives it, with its exact p as the package prints it. A column still running
    gets no verdict, as in the package."""
    fewer = [r for r in rows if r["with"]["solved"] < r["without"]["solved"]]
    if not fewer:
        return []
    groups = {}
    for r in fewer:
        v = r["verdict"] if r["complete"] else "no verdict yet (still running)"
        groups.setdefault(v, []).append(r["model"] + ("" if r["p"] is None else f" (p = {r['p']})"))
    return [f"With {NAME}, {len(fewer)} of the {len(rows)} models solved fewer start states than without it. "
            "Exact paired sign test on SOLVED, read after Holm's correction:",
            "; ".join(f"{v} for {joined(ms)}" for v, ms in groups.items()) + "."]


def filter_footer(filtered):
    """The arms with a reply the provider's content filter stopped: each one whose scored episodes ended on such
    replies by name with its counts, the rest by count. Every count of replies is over the arm's scored and
    unscored episodes together (the package's filtered_item() gives the two apart), for the named arms and the
    rest alike. Wrapped at 140 characters a line, never at a hyphen: a fixed count of characters, not a measured
    width, and the other footer lines are not wrapped."""
    named = [(c, arm, f) for c, arm, f in filtered if f["ended"]]
    rest = [f for c, arm, f in filtered if not f["ended"]]
    s = []
    for c, arm, f in named:
        s.append(f"{c.model} {arm} {NAME}: the provider's content filter stopped {f['filtered']:,} of its "
                 f"{f['replies'] + f['unscored_replies']:,} replies; {len(f['ended'])} of its "
                 f"{c.stats(arm)['scored']} scored episodes ended "
                 f"on them" + (", with no claim from the model."
                               if not any(e["ended_by"] == "claim" for e in f["ended"]) else "."))
    if rest:
        n = sum(f["filtered"] for f in rest)
        word = NUMBER_WORDS.get(len(rest), str(len(rest))).capitalize()
        each = "one filtered reply" + (" each" if len(rest) > 1 else "")
        s.append(f"{word}{' other' if named else ''} arm{'' if len(rest) == 1 else 's'} had "
                 + (each if n == len(rest) else f"{n:,} filtered replies in all")
                 + ", ending no scored episode.")
    if not s:
        return []
    return textwrap.wrap(" ".join(s), 140, break_on_hyphens=False)


def scope(cells, task_rows):
    """The fine print every chart of these rows carries, one sentence on two subtitle lines so it fits the
    width: what the start states are."""
    return [f"{cells} new start states of {task_rows} LIBERO-PRO task rows {NAME} was developed on at other start "
            f"states,", "one episode each, the same model with and without."]


def label(r):
    return r["model"] + ("" if r["complete"] else "*") + ("\u2020" if r["late"] else "")


def success_chart(rows, when, cells, task_rows, joined, filtered):
    def rate(s):
        return s["solved"] / s["finished"]

    def parts_for(r):
        def parts(y):
            out = [text(LABEL_X, y + 25, label(r), 13.5, INK, anchor="end")]
            for arm, fill, dy in (("without", WITHOUT, 10), ("with", WITH, 26)):
                s = r[arm]
                w = rate(s) * (PCT_FULL - LEFT)
                tip = f"{r['model']}, {arm} {NAME}: {solved_of(s)} solved"
                if w > 0:
                    out.append(bar(y + dy, w, fill, tip))
                out.append(text(f"{LEFT + w + 5:.1f}", y + dy + 11, solved_of(s), 11.5, VALUE))
            return out
        return parts

    ticks = [(LEFT + k / 100 * (PCT_FULL - LEFT), f"{k}%") for k in range(0, 101, 20)]
    footer = (["Success rate: solved of finished episodes."]
              + fewer_footer(rows, joined)
              + filter_footer(filtered)
              + star_footer(rows, when, cells))
    return frame(f"Success rate, without and with {NAME}",
                 scope(cells, task_rows)
                 + [f"Ordered by success rate without {NAME}, lowest first."
                  + ("" if all(r["complete"] for r in rows) else " * still running.") + f" Read {when}."],
                 ticks, [parts_for(r) for r in rows], footer)


def cost_chart(rows, when, cells, task_rows, fnum, unscored, lost_usd, filtered):
    scale = (USD_CUT_X - LEFT) / USD_CUT
    broken = sum(1 for r in rows for a in ("without", "with")
                 if r[a]["usd_per_solved"] is not None and r[a]["usd_per_solved"] > USD_CUT)

    def parts_for(r):
        def parts(y):
            out = [text(LABEL_X, y + 25, label(r), 13.5, INK, anchor="end")]
            for arm, fill, dy in (("without", WITHOUT, 10), ("with", WITH, 26)):
                s = r[arm]
                v = s["usd_per_solved"]
                if v is None:
                    out.append(text(f"{LEFT + 5:.1f}", y + dy + 11, "none solved", 11.5, VALUE))
                    continue
                tip = f"{r['model']}, {arm} {NAME}: ${fnum(v, 2)} a success, {solved_of(s)} solved"
                if v > USD_CUT:
                    out.append(bar(y + dy, USD_BROKEN_W, fill, tip))
                    for x0 in (LEFT + USD_BROKEN_W - 23, LEFT + USD_BROKEN_W - 16):
                        d = f"M{x0:.1f},{y + dy + 16} L{x0 + 6:.1f},{y + dy - 3}"
                        out.append(f'<path d="{d}" style="stroke:{PAPER}" stroke-width="4"/>')
                        out.append(f'<path d="{d}" style="stroke:{MUTED}" stroke-width="1.2"/>')
                    out.append(text(f"{LEFT + USD_BROKEN_W + 5:.1f}", y + dy + 11, f"${fnum(v, 2)}", 11.5, VALUE,
                                    weight="600"))
                    continue
                w = v * scale
                out.append(bar(y + dy, w, fill, tip))
                out.append(text(f"{LEFT + w + 5:.1f}", y + dy + 11, f"${fnum(v, 2)}", 11.5, VALUE))
            return out
        return parts

    ticks = [(LEFT + k * scale, f"${k}") for k in range(0, int(USD_CUT) + 1, 5)]
    tail = ""
    if broken:
        word = NUMBER_WORDS.get(broken, str(broken))
        tail = f" Linear scale; {word} bar{'' if broken == 1 else 's'} {'is' if broken == 1 else 'are'} broken."
    lower_cache = sum(1 for r in rows if r["without"]["cached_share"] is not None
                      and r["with"]["cached_share"] is not None
                      and r["without"]["cached_share"] < r["with"]["cached_share"])
    caching = (f"Part of the gap is caching: cached input costs less, and the arm without {NAME} "
               f"had the lower cached share of input for {lower_cache} of the {len(rows)} models."
               if lower_cache == len(rows) else
               f"Part of the gap is caching for the {lower_cache} of the {len(rows)} models whose "
               f"arm without {NAME} had the lower cached share of input: cached input costs less.")
    footer = ["Model spend only; the simulator is not counted. Failed scored episodes are included: their cost is "
              "spread over the successes.",
              f"The {unscored} episodes that ended without a score and the other attempts in lost_attempts.csv are "
              f"left out (${fnum(lost_usd, 2)} of spend in all)."] \
        + ([f"A broken bar runs past ${USD_CUT:.0f}."] if broken else []) \
        + [caching] \
        + filter_footer(filtered) \
        + star_footer(rows, when, cells)
    return frame(f"Model cost per successful episode, without and with {NAME}",
                 scope(cells, task_rows)
                 + ["Model spend of every scored episode divided by the episodes solved, at list prices." + tail,
                  f"Ordered by success rate without {NAME}, lowest first."
                  + ("" if all(r["complete"] for r in rows) else " * still running.") + f" Read {when}."],
                 ticks, [parts_for(r) for r in rows], footer)


def main(argv):
    check = "--check" in argv[1:]
    rc = load_recompute()
    rows = rows_of(rc)
    when, cells = read_info()
    listed, cols, uns = rc.columns(PACKAGE)
    task_rows = len({x["task_row"] for x in listed})
    lost_usd = sum(float(x["usd_list"]) for x in rc.load(PACKAGE, "lost_attempts.csv"))
    filtered = rc.filtered_arms(cols)
    outputs = {"harness-success-rate.svg": success_chart(rows, when, cells, task_rows, rc.joined, filtered),
               "harness-cost-per-success.svg": cost_chart(rows, when, cells, task_rows, rc.fnum, len(uns), lost_usd,
                                                          filtered)}
    stale = []
    for name, svg in outputs.items():
        path = os.path.join(MEDIA, name)
        current = None
        if os.path.exists(path):
            with open(path, encoding="utf-8") as fh:
                current = fh.read()
        if current == svg:
            continue
        if check:
            stale.append(f"docs/media/{name}")
        else:
            with open(path, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(svg)
            print(f"wrote docs/media/{name}")
    if stale:
        print("build_harness_charts: differs from the package (run without --check): " + ", ".join(stale))
        return 1
    print(f"build_harness_charts: OK ({len(outputs)} charts, {len(rows)} models, read {when})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
