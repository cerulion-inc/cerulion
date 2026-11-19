#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Draw the two harness benchmark charts from their results package.

It replots a committed package. It never runs a benchmark and never edits a package.

  docs/media/harness-success-rate.svg       success rate per model, without and with Cerulion
  docs/media/harness-cost-per-success.svg   model cost per successful episode, without and with Cerulion

Every count and cost is computed by the package's own recompute.py from the package's CSV files,
so a figure on a chart is the figure SUMMARY.md prints (cost per success is SUMMARY.md's "USD per
solved start state"). Standard library only: the same package gives the same bytes on every machine.

    python3 docs/media/charts/build_harness_charts.py           # write the two SVGs
    python3 docs/media/charts/build_harness_charts.py --check   # write nothing; exit 1 if a chart differs

The form of both charts:
  - one row per model, ordered by the share of finished episodes solved WITHOUT Cerulion, lowest
    first, ties by name; the same order in both charts; no model is marked;
  - linear scales; success rate in percent;
  - the cost scale ends at 30 USD, and a bar past it is drawn to the edge with a break and its value
    printed at its end;
  - a factor at the right of every row: success rate with over without, and cost per success without
    over with ("times cheaper"). Under 1.1 it is printed to two decimals, so a small loss stays
    visible; from 1.1 to 10 to one decimal; from 10 as a whole number. A model that solved nothing
    without Cerulion reads "from 0%" on the success chart and "no figure" on the cost chart;
  - a model whose column had not finished at the package's read carries a star, and a footnote gives
    its finished counts and the read time.
"""
import decimal
import html
import importlib.util
import json
import os
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
PACKAGE = os.path.join(ROOT, "docs", "benchmarks", "results", "7cb9f322c10f6fa7-2026-11-18-harness-libero-pro")
MEDIA = os.path.join(ROOT, "docs", "media")
NAME = "Cerulion"

WIDTH = 1000
LEFT = 190          # x of the zero line
LABEL_X = 178       # right edge of the model names
RIGHT = 976         # right edge of the factor column
TOP = 100           # y of the first row's band
ROW = 40            # one model
BAR = 13            # one bar
PCT_FULL = 850      # x of 100 percent
USD_CUT = 30.0      # the cost scale ends here
USD_CUT_X = 810     # x of the cut
USD_BROKEN_W = 650  # a broken bar is drawn this wide
WITHOUT = "#eb6834"
WITH = "#2a78d6"
INK = "#171717"
MUTED = "#555"
SOFT = "#666"
GRID = "#e6e6e6"
AXIS = "#8a8a8a"
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


def half_up(value, places):
    q = decimal.Decimal(1).scaleb(-places)
    return decimal.Decimal(repr(value)).quantize(q, rounding=decimal.ROUND_HALF_UP)


def percent(rate):
    return f"{half_up(100 * rate, 0)}%"


def factor(x):
    if x < 1.1:
        return f"{half_up(x, 2)}×"
    if x < 10:
        return f"{half_up(x, 1)}×"
    return f"{half_up(x, 0)}×"


def esc(s):
    return html.escape(s, quote=True)


def text(x, y, s, size, fill, anchor=None, weight=None):
    a = f' text-anchor="{anchor}"' if anchor else ""
    w = f' font-weight="{weight}"' if weight else ""
    return f'<text x="{x}" y="{y}" font-size="{size}"{w} fill="{fill}"{a}>{esc(s)}</text>'


def bar(y, width, fill, title):
    """A bar from the zero line with its right end rounded."""
    r = min(4.0, width / 2)
    return (f'<path d="M{LEFT:.1f},{y} h{width - r:.1f} a{r:g},{r:g} 0 0 1 {r:g},{r:g} v{BAR - 2 * r:.1f} '
            f'a{r:g},{r:g} 0 0 1 -{r:g},{r:g} h-{width - r:.1f} z" fill="{fill}"><title>{esc(title)}</title></path>')


def rows_of(rc):
    _cells, cols, _uns = rc.columns(PACKAGE)
    out = []
    for c in cols:
        wo, wi = c.stats("without"), c.stats("with")
        out.append({"model": c.model, "complete": c.complete, "without": wo, "with": wi})
    for r in out:
        if not r["without"]["finished"] or not r["with"]["finished"]:
            raise SystemExit(f"build_harness_charts: {r['model']} has an arm with no finished episode")
    out.sort(key=lambda r: (r["without"]["solved"] / r["without"]["finished"], r["model"]))
    return out


def frame(title, subtitle, header, ticks, rows, footer):
    n = len(rows)
    bottom = TOP + ROW * n + 4
    height = bottom + 48 + 18 * len(footer)
    s = [f'<svg viewBox="0 0 {WIDTH} {height}" xmlns="http://www.w3.org/2000/svg" '
         f'font-family="system-ui,-apple-system,Segoe UI,sans-serif" role="img" aria-label="{esc(title)}">',
         f'<rect width="{WIDTH}" height="{height}" fill="#fff"/>',
         text(24, 34, title, 19, INK, weight="650"),
         text(24, 56, subtitle[0], 13, MUTED),
         text(24, 74, subtitle[1], 13, MUTED),
         f'<rect x="24" y="84" width="14" height="10" rx="2" fill="{WITHOUT}"/>',
         text(44, 93, f"without {NAME}", 13, INK),
         f'<rect x="170" y="84" width="14" height="10" rx="2" fill="{WITH}"/>',
         text(190, 93, f"with {NAME}", 13, INK),
         text(RIGHT, 93, header, 12.5, "#444", anchor="end", weight="600")]
    for x, label in ticks:
        s.append(f'<line x1="{x:.1f}" y1="{TOP}" x2="{x:.1f}" y2="{bottom}" stroke="{GRID}" stroke-width="1"/>')
        s.append(text(f"{x:.1f}", bottom + 18, label, 12, SOFT, anchor="middle"))
    s.append(f'<line x1="{LEFT:.1f}" y1="{TOP}" x2="{LEFT:.1f}" y2="{bottom}" stroke="{AXIS}" stroke-width="1"/>')
    for i, parts in enumerate(rows):
        y = TOP + ROW * i
        s.extend(parts(y))
    for j, line in enumerate(footer):
        s.append(text(24, bottom + 44 + 18 * j, line, 12, SOFT))
    s.append("</svg>")
    return "".join(s) + "\n"


def star_footer(rows, when, cells):
    out = []
    for r in rows:
        if not r["complete"]:
            out.append(f"* Still running at the read of {when}: {r['model']}, {r['without']['finished']} of {cells} "
                       f"start states finished without {NAME} and {r['with']['finished']} with.")
    return out


def label(r):
    return r["model"] + ("" if r["complete"] else "*")


def success_chart(rows, when, cells):
    def rate(s):
        return s["solved"] / s["finished"]

    def parts_for(r):
        def parts(y):
            out = [text(LABEL_X, y + 25, label(r), 13.5, INK, anchor="end")]
            for arm, fill, dy in (("without", WITHOUT, 10), ("with", WITH, 26)):
                s = r[arm]
                w = rate(s) * (PCT_FULL - LEFT)
                tip = (f"{r['model']}, {arm} {NAME}: {s['solved']} of {s['finished']} solved "
                       f"({percent(rate(s))})")
                if w > 0:
                    out.append(bar(y + dy, w, fill, tip))
                out.append(text(f"{LEFT + w + 5:.1f}", y + dy + 11, percent(rate(s)), 11.5, "#333"))
            wo, wi = rate(r["without"]), rate(r["with"])
            out.append(text(RIGHT, y + 27, "from 0%" if wo == 0 else factor(wi / wo), 14, INK, anchor="end",
                            weight="650"))
            return out
        return parts

    ticks = [(LEFT + k / 100 * (PCT_FULL - LEFT), f"{k}%") for k in range(0, 101, 20)]
    below = any(rate(r["without"]) > rate(r["with"]) for r in rows)
    footer = ([f"Success rate: solved of finished episodes. The factor is the success rate with {NAME} over the "
               f"rate without."]
              + ([f"Below 1.0× the model did a little better without {NAME}."] if below else [])
              + star_footer(rows, when, cells))
    return frame(f"Success rate, without and with {NAME}",
                 [f"Solved start states of LIBERO-PRO, {cells} new start states per model and arm, zero-shot, the "
                  "same model in both arms.",
                  f"Ordered by success rate without {NAME}, lowest first."
                  + ("" if all(r["complete"] for r in rows) else " * still running.") + f" Read {when}."],
                 "with ÷ without", ticks, [parts_for(r) for r in rows], footer)


def cost_chart(rows, when, cells, fnum):
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
                    out.append(text(f"{LEFT + 5:.1f}", y + dy + 11, f"none solved (${fnum(s['usd_sum'], 2)} spent)",
                                    11.5, "#333"))
                    continue
                tip = (f"{r['model']}, {arm} {NAME}: ${fnum(s['usd_sum'], 2)} over {s['solved']} solved of "
                       f"{s['finished']} = ${fnum(v, 2)} a success")
                if v > USD_CUT:
                    out.append(bar(y + dy, USD_BROKEN_W, fill, tip))
                    for x0 in (LEFT + USD_BROKEN_W - 23, LEFT + USD_BROKEN_W - 16):
                        d = f"M{x0:.1f},{y + dy + 16} L{x0 + 6:.1f},{y + dy - 3}"
                        out.append(f'<path d="{d}" stroke="#fff" stroke-width="4"/>')
                        out.append(f'<path d="{d}" stroke="{MUTED}" stroke-width="1.2"/>')
                    out.append(text(f"{LEFT + USD_BROKEN_W + 5:.1f}", y + dy + 11, f"${fnum(v, 2)}", 11.5, "#333",
                                    weight="600"))
                    continue
                w = v * scale
                out.append(bar(y + dy, w, fill, tip))
                out.append(text(f"{LEFT + w + 5:.1f}", y + dy + 11, f"${fnum(v, 2)}", 11.5, "#333"))
            wo, wi = r["without"]["usd_per_solved"], r["with"]["usd_per_solved"]
            out.append(text(RIGHT, y + 27, "no figure" if wo is None or wi is None else factor(wo / wi), 14, INK,
                            anchor="end", weight="650"))
            return out
        return parts

    ticks = [(LEFT + k * scale, f"${k}") for k in range(0, int(USD_CUT) + 1, 5)]
    tail = ""
    if broken:
        word = NUMBER_WORDS.get(broken, str(broken))
        tail = f" Linear scale; {word} bar{'' if broken == 1 else 's'} {'is' if broken == 1 else 'are'} broken."
    footer = ["Model spend only; the simulator is not counted. Failed episodes are included: their cost is spread "
              "over the successes.",
              f"The factor is the cost per success without {NAME} over the cost with it."
              + (f" A broken bar runs past ${USD_CUT:.0f}." if broken else "")] + star_footer(rows, when, cells)
    return frame(f"Model cost per successful episode, without and with {NAME}",
                 ["Model spend of every scored episode divided by the episodes solved, at list prices." + tail,
                  f"Ordered by success rate without {NAME}, lowest first."
                  + ("" if all(r["complete"] for r in rows) else " * still running.") + f" Read {when}."],
                 "times cheaper", ticks, [parts_for(r) for r in rows], footer)


def main(argv):
    check = "--check" in argv[1:]
    rc = load_recompute()
    rows = rows_of(rc)
    when, cells = read_info()
    outputs = {"harness-success-rate.svg": success_chart(rows, when, cells),
               "harness-cost-per-success.svg": cost_chart(rows, when, cells, rc.fnum)}
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
