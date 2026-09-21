#!/usr/bin/env python3
"""plot.py — round-trip latency plots for the benches/latency/ suite.

Consumes the per-cell CSVs written by compile_csv.py and produces one
PNG + one SVG per plot group. The expected CSV set for every group is
DERIVED from the cell enumerations in bench.py (the single source of truth
for the line inventory), through the pinned coupling:

    raw_prefix  →  results_<raw_prefix>.csv

A missing expected CSV FAILS the plot run loudly (listing every missing
file) instead of silently dropping the line — the old tree's warn-and-skip
behavior made lines vanish from plots with no visible signal (the
documented raw_prefix footgun). Strictness also covers ROW completeness:
every present series must carry ALL expected payload rows (the pinned
sweep, bench.PAYLOAD_SIZES) — a partially-swept cell would otherwise
render as a plausible-looking shorter line. Pass --skip-missing to accept
a partial run: it still prints one loud warning per missing series/row,
then plots what exists. Structural skips never appear in the expected set
at all (the enumerations exclude them — e.g. zenoh×loan, whose take-loan
lane is a NO-OP stub).

Plot groups:

    native         iox2 / zenoh_shm host lines (floor + comparison)
    workspace      cerulion_workspace_{split,mono} legs (split = headline)
    ros2-humble    all enumerated ROS 2 cells for that distro
    ros2-jazzy       (be1 solid; rel10 dotted, color-paired with its be1
    ros2-lyrical      sibling; chrt1 dashed, color-paired with chrt0)
    hero-tuned     the two heroes: the same
    hero-stock     six-line cross-stack set rendered twice from
                   posture-keyed run dirs — workspace split (headline) +
                   mono + raw iox2 floor + zenoh SHM + jazzy stock ROS 2
                   (zero config) + jazzy composed ROS 2 (intra-process
                   on). 'tuned' = all machine tunings on (C-state cap +
                   SCHED_FIFO ⇒ the chrt1 CSVs, except the chrt0-only
                   iox2 floor); 'stock' = nothing tuned (chrt0
                   everywhere, run measured under CER_BENCH_DMA_LOCK=0).
                   The pair is the story: same software, same lines, one
                   variable. Loan-take + CycloneDDS lanes are NOT hero
                   lines anymore — they live in the per-distro groups.
                   A hero group VERIFIES the run dir's manifest posture
                   (run.json dma_lock_posture) and ignores --chrt (the
                   posture decides the chrt suffixes).
    all            every non-hero group above (one PNG + SVG each; the
                   heroes are posture-keyed and must be rendered
                   explicitly against their posture run dirs — 'all'
                   prints a loud note so their absence is never silent)

Rendering (public-chart polish — the default for EVERY group):

  - endpoint value labels ride every primary/secondary line AND the
    ROS 2 CycloneDDS distro lines (the hero's comparison family). Placement:
    every label sits in the RIGHT GUTTER — left-anchored just past its
    line's LAST point and vertically centered on that endpoint — so the
    number unambiguously belongs to its line. Collision handling is
    VERTICAL-ONLY nudging within the gutter (labels keep the ascending
    order of the values they report — numeric order == vertical order —
    and a collision cluster settles CENTERED on its members' endpoints,
    splitting the displacement between its labels instead of riding the
    whole stack upward; _layout_label_column). The axes keep a widened
    right margin (AXES_WIDTH) to fit the longest label; every label wears
    a surface-colored halo so it stays readable over anything a nudged
    label still crosses.
  - a figure that carries a loan-take lane also carries the compact
    LOAN_CALLOUT stating what that lane IS —
    raw rcl API, POD types only, manual loan return, cross-process —
    so the flat line cannot be misread as the rclcpp callback path.
  - p50 line + a shaded p50→p99 band per series (the tail-spread visual).
    Where a row's round_trip_p99_ns cell is EMPTY (the audit-A2 suppression:
    fewer than MIN_TAIL_EXCEEDANCES=20 pooled samples back that tail) the
    band NARROWS TO THE LINE at that point and an in-figure footnote counts
    the suppressions — a suppressed tail is never interpolated or faked.
  - cross-rep spread: when a CSV carries rep_count > 1 the line
    is the MEDIAN of the per-rep p50s and thin capped WHISKERS at each
    marker show the per-rep p50 min–max. Whiskers (crisp, centered on the
    line) deliberately read differently from the tail band (a soft one-sided
    wash above the line), so the two spreads never fight; this replaces the
    earlier min–max fill, which the p50→p99 band would have shadowed.
  - visual hierarchy: role-mapped colors/weights (see ROLE_RULES) — the
    Cerulion workspace-split headline is visually primary; the raw-iox2
    floor and the zenoh SHM comparison are secondary; ROS 2 cells are
    tertiary but readable. Colors follow the ENTITY (an rmw family keeps
    its hue in every figure), never the series index — palette hexes are
    the validated dataviz reference palette (six-check validator, light
    surface #fcfcfb; the two WARNs it reports — red↔aqua CVD 6.9 in the
    6–8 floor band, aqua/magenta sub-3:1 contrast — are discharged by the
    secondary encoding shipped here: legend + per-series markers + endpoint
    labels, with the results CSVs as the table-view twin).
  - theme: the reference rmw chart look (#fcfcfb surface,
    system-ui type, title/subtitle hierarchy, hairline horizontal decade
    grid, muted footnote block under a hairline rule). SVG output keeps
    real <text> (svg.fonttype='none') and rewrites the font stack to
    system-ui so it renders like the reference in any browser.

(The old boundary-dagger footnote machinery, which marked rmw_cerulion
cells drawn beside native lines, is gone.)

Release renders (--release): the repo-embeddable variant strips the
AUTO annotation layer (the auto caption, the suppression note, the
fallback-ladder and type-class lines, the hero knob-matrix footnotes) and
the watermark, while title/subtitle/legend/endpoint labels stay and
explicit --footnote lines are KEPT so a citable render retains its
author-supplied caveats. Release strips annotations, never data integrity:
the A2 suppressed-tail rule still narrows the band to the p50 line wherever a
p99 cell is empty (no tail is fabricated just because the note that
explains the suppression is gone). --release refuses --watermark loudly
(a watermark marks a NON-citable render; release is the citable one).
Footnote lines wrap at the hairline rule's right edge (measured with the
renderer, greedy on word boundaries), so a long line is never clipped.

Usage:

    plot.py --results-dir <run-dir> [--out-dir <run-dir>/plots]
            [--group all] [--chrt {0,1,both}] [--skip-missing]
            [--subtitle TEXT] [--footnote TEXT ...] [--watermark TEXT]
            [--release]
    plot.py --results-dir <run-dir> --series LABEL=path[:dashed][@KEY] ... \
            [--out-name custom.png] [--title TEXT]   # manual overlay,
                                                     # groups ignored

A --series entry may end in @KEY (after the optional style token): an
explicit COLOR KEY. Entries sharing a KEY share a color, and a keyed entry
takes its color from the fallback slots in first-seen order instead of the
entity rule (_role_for), which would otherwise paint every Cerulion
workspace CSV blue whatever it is meant to distinguish. That is the
cross-platform figure: the same two workspace cells measured on three
machines, color by MACHINE (the validated machines trio, blue / orange /
violet) and run shape by line style (solid multi-process, dashed
single-process), markers and legend as always. Three keys at most stay
inside the validated trio; the pool ends at six, then gray, loudly.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

# bench.py lives in the same directory and owns the cell enumerations.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import bench  # noqa: E402

LINESTYLES = {"solid": "-", "dashed": "--", "dotted": ":", "dashdot": "-."}


@dataclass(frozen=True)
class Series:
    label: str        # raw_prefix for group series (greppable 1:1 with the
                      # CSVs; the legend shows pretty_label(label), falling
                      # back to the label verbatim for manual overlays)
    csv_name: str     # results_<raw_prefix>.csv
    style: str        # matplotlib linestyle
    color_key: str    # series sharing a color_key share a color
    pretty: Optional[str] = None  # legend-text override (the hero groups
                      # carry fixed labels that stay identical
                      # across the two postures — the posture lives in the
                      # title/footnote, never per-line); None = derive via
                      # pretty_label(label)
    keyed: bool = False  # True when a --series entry named its color key
                      # explicitly (@KEY): the color follows the KEY and
                      # skips the entity rule, so one cell measured on
                      # several machines can wear one color per machine


def _style_for(chrt: int, qos: Optional[str]) -> str:
    """chrt1 = dashed; rel10 = dotted; both = dashdot; base = solid."""
    if qos == "rel10":
        return "-." if chrt == 1 else ":"
    return "--" if chrt == 1 else "-"


def _chrt_modes(chrt_arg: str) -> Tuple[int, ...]:
    return (0, 1) if chrt_arg == "both" else (int(chrt_arg),)


# ---------------------------------------------------------------- groups


def native_series(chrt_arg: str) -> List[Series]:
    out: List[Series] = []
    for b in bench.NATIVE_BENCHES:
        for chrt in _chrt_modes(chrt_arg):
            if chrt == 1 and not b.chrt_on:
                continue  # never enumerated (spin-bound), so never expected
            name = f"{b.raw_prefix}_chrt{chrt}"
            out.append(Series(name, f"results_{name}.csv",
                              _style_for(chrt, None), b.raw_prefix))
    return out


def workspace_series(chrt_arg: str,
                     variant: str = "quiescent") -> List[Series]:
    """The workspace legs a run under `variant` can carry: a leg the
    variant gate (bench.workspace_leg_skip_reason — split x backtoback is
    structurally unmeasurable) never produces is not EXPECTED here either,
    so a strict render of a backtoback run dir does not fail on a CSV the
    sweep was never going to write.

    Type-class axis (METHODOLOGY § "The type-class axis"): each leg
    carries a variable row (the incumbent Image legs — pinned token-less
    prefixes) AND a pod row (`_pod` prefixes, the fixed-PodPayload
    twin). The two classes are separate Series — never merged into one
    plotted line — with the class named in the legend vocabulary."""
    out: List[Series] = []
    for leg in bench.WORKSPACE_LEGS:
        reason = bench.workspace_leg_skip_reason(leg, variant)
        if reason is not None:
            print(f"note: workspace leg '{leg}' is not expected under "
                  f"variant {variant}: {reason}", file=sys.stderr)
            continue
        for msg in bench.WORKSPACE_MSG_CLASSES:
            for chrt in _chrt_modes(chrt_arg):
                name = bench.workspace_raw_name(leg, chrt, msg)
                key = (f"workspace_{leg}" if msg == "variable"
                       else f"workspace_{leg}_pod")
                out.append(Series(name, f"results_{name}.csv",
                                  _style_for(chrt, None), key))
    return out


def ros2_series(distro: str, chrt_arg: str) -> List[Series]:
    out: List[Series] = []
    for chrt in _chrt_modes(chrt_arg):
        cells, _skips = bench.enumerate_ros2_cells(distro, chrt)
        for c in cells:
            # The type-class token rides in the color_key so an image
            # twin gets its OWN visual assignment (_assign_visuals is
            # keyed by color_key and first-seen wins: with the pod key
            # shared, the image tint branch in _role_for was reachable
            # but never assigned, and both rows painted the pod shade).
            # Pod keys stay byte-identical (no token) so the incumbent
            # role mapping + hero keys never move.
            msg_tok = "" if c.msg == "pod" else f"_{c.msg}"
            color_key = f"{c.distro}_{c.rmw}_{c.shm}{msg_tok}_{c.recv}"
            out.append(Series(c.name, f"results_{c.name}.csv",
                              _style_for(c.chrt, c.qos), color_key))
    return out


# The two heroes ("a hero with all tunings on
# and a hero with nothing tuned"). Same six-line set in both — the pair is
# the story: same software, same lines, one variable (the machine posture).
# Loan-take + the CycloneDDS distro lanes were REMOVED from the hero (they
# live in the per-distro group plots with the audit callout); ipc-off
# composed likewise stays per-distro only.
HERO_POSTURES = ("tuned", "stock")

# Fixed legend vocabulary — identical across both postures (the
# posture is named by the title and the knob-matrix footnote, never per
# line, so the two figures read as one comparison).
_HERO_PRETTY = {
    "split": "Cerulion multi-process (cerulion graph run)",
    "mono": "Cerulion single-process (graph run --single-process)",
    "iox2": "raw iceoryx2 (transport floor)",
    "zenoh_shm": "zenoh SHM (z_ping / z_pong)",
    "stock": "ROS 2 defaults (Jazzy, rmw_fastrtps_cpp)",
    "composed": "ROS 2 composed + intra-process (Jazzy): one process, "
                "no process isolation",
}


def hero_series(posture: str) -> List[Series]:
    """The six-line cross-stack hero for one machine POSTURE.

    posture='tuned'  — all machine tunings on: C-state cap + SCHED_FIFO
                       ⇒ the chrt1 CSVs everywhere the cell supports RT;
                       the raw iox2 floor is chrt0-only by design
                       (spin-bound — bench.NativeBench.chrt_on=False).
    posture='stock'  — nothing tuned: chrt0 everywhere; the run dir must
                       have been measured under CER_BENCH_DMA_LOCK=0.

    Lines render SOLID in both postures (the chrt dash convention exists
    to separate chrt0/chrt1 lines sharing one figure; here the posture is
    figure-wide and carried by the title + knob-matrix footnote), and the
    legend labels are the posture-invariant fixed vocabulary
    (_HERO_PRETTY). The caller (main) verifies the run dir's manifest
    posture via verify_hero_posture."""
    if posture not in HERO_POSTURES:
        raise SystemExit(f"unknown hero posture '{posture}' — choose from: "
                         f"{', '.join(HERO_POSTURES)}")
    chrt = 1 if posture == "tuned" else 0
    out: List[Series] = []
    for b in bench.NATIVE_BENCHES:
        c = chrt if b.chrt_on else 0
        name = f"{b.raw_prefix}_chrt{c}"
        out.append(Series(name, f"results_{name}.csv", "-", b.raw_prefix,
                          pretty=_HERO_PRETTY[b.raw_prefix]))
    # The HEADLINE Cerulion line is the declared 2-group split leg
    # (it replaces the flagless default,
    # which reads ~25.9 µs p50 under the park-wake inflation until its
    # fix lands).
    # mono rides along, clearly labeled, as the single-process opt-in.
    for leg in bench.WORKSPACE_LEGS:
        name = bench.workspace_raw_name(leg, chrt)
        out.append(Series(name, f"results_{name}.csv", "-",
                          f"workspace_{leg}", pretty=_HERO_PRETTY[leg]))
    # ROS 2 usage-pattern lanes (ros2/memo.md), jazzy:
    # stock = the zero-config default (what `ros2 run` gives you);
    # composed ipc-on = the fastest-common shape (Nav2-default
    # composition + the intra-process opt-in).
    cells, _skips = bench.enumerate_ros2_cells("jazzy", chrt)
    for c in cells:
        if c.rmw == "stock" or (c.rmw == "composed" and c.shm == "ipcon"):
            out.append(Series(c.name, f"results_{c.name}.csv", "-",
                              f"{c.distro}_{c.rmw}_{c.shm}",
                              pretty=_HERO_PRETTY[c.rmw]))
    return out


def verify_hero_posture(results_dir: Path, posture: str,
                        context: str) -> None:
    """A hero figure titled with a posture must come from a run dir whose
    manifest RECORDS that posture — otherwise the render is a mislabel
    (e.g. capped-chrt0 data titled 'stock computer: no root, no tuning').
    run.json invocations carry dma_lock_posture since the CER_BENCH_DMA_LOCK
    knob landed (13a9ddec7): 'tuned' = cap held, 'stock' = no cap. Every
    invocation that carries the key must match; a dir with no key at all
    (pre-knob sweep) is refused — this is a label-accuracy gate, so
    --skip-missing deliberately does NOT demote it."""
    run_json = results_dir / "run.json"
    seen: List[str] = []
    if run_json.exists():
        import json
        try:
            manifest = json.loads(run_json.read_text())
            seen = [v for v in (inv.get("dma_lock_posture")
                                for inv in manifest.get("invocations", []))
                    if v is not None]
        except (OSError, ValueError) as e:
            raise SystemExit(f"error: {context}: unreadable {run_json}: {e}")
    else:
        raise SystemExit(
            f"error: {context}: {run_json} is missing — a hero render is "
            f"posture-keyed and needs the run manifest to prove the posture")
    if not seen:
        raise SystemExit(
            f"error: {context}: no invocation in {run_json} records "
            f"dma_lock_posture — this run dir predates the "
            f"CER_BENCH_DMA_LOCK knob (or is not posture-keyed), so a "
            f"'{posture}'-titled hero from it would be an unverifiable "
            f"label. Re-sweep under the posture (CER_BENCH_DMA_LOCK="
            f"{'1' if posture == 'tuned' else '0'}) or point --results-dir "
            f"at the posture run dir.")
    wrong = sorted(set(v for v in seen if v != posture))
    if wrong:
        raise SystemExit(
            f"error: {context}: run manifest posture mismatch — "
            f"{run_json} records dma_lock_posture={wrong} but this hero "
            f"claims '{posture}'. A posture-titled figure from "
            f"differently-postured data is a mislabel; use the matching "
            f"run dir.")


def resolve_variant(results_dir: Path, requested: Optional[str]) -> str:
    """The pacing variant the run dir was measured under.

    Explicit --variant wins but is CHECKED against the run manifest (a
    variant-keyed run dir records it — bench.RunManifest refuses to mix
    variants in one dir); a mismatch is a mislabel, refused loudly. With
    no --variant the manifest's value is used; a dir with no manifest
    (legacy / hand-assembled) defaults to 'quiescent'."""
    recorded: Optional[str] = None
    run_json = results_dir / "run.json"
    if run_json.exists():
        import json
        try:
            manifest = json.loads(run_json.read_text())
        except (OSError, ValueError) as e:
            raise SystemExit(f"error: unreadable {run_json}: {e}")
        recorded = manifest.get("variant")
        if recorded is None:
            for inv in manifest.get("invocations", []):
                if inv.get("variant") is not None:
                    recorded = inv["variant"]
                    break
    if requested is not None:
        if recorded is not None and recorded != requested:
            raise SystemExit(
                f"error: --variant {requested} but {run_json} records "
                f"variant {recorded!r} — a render enumerated for one "
                f"pacing variant over another variant's data is a "
                f"mislabel; pass --variant {recorded} (or omit it to "
                f"take the manifest's value)")
        return requested
    if recorded is not None:
        if recorded not in bench.PACING_VARIANTS:
            raise SystemExit(
                f"error: {run_json} records unknown variant {recorded!r} "
                f"(known: {', '.join(bench.PACING_VARIANTS)})")
        return recorded
    # Neither stated nor recorded. `bench.py plots` used to always forward a
    # variant, so this fell only to a direct plot.py call; since it stopped
    # forwarding a parser default, bench.py reaches it too. The
    # default stays (a legacy / hand-assembled run dir must remain
    # plottable) but the inference is announced, never silent: the variant
    # decides which workspace legs are EXPECTED, so guessing it wrong is a
    # missing-line report against a leg the run never had.
    print(f"warn: {results_dir}: no --variant given, and "
          f"{'run.json records none' if run_json.exists() else 'there is no run.json to read one from'}"
          f" — assuming 'quiescent'. Pass --variant explicitly if this run "
          f"dir is a different pacing variant.", file=sys.stderr)
    return "quiescent"


def group_series(group: str, chrt_arg: str,
                 variant: str = "quiescent") -> List[Series]:
    if group == "native":
        return native_series(chrt_arg)
    if group == "workspace":
        return workspace_series(chrt_arg, variant)
    if group.startswith("ros2-"):
        distro = group[len("ros2-"):]
        if distro not in bench.ROS2_DISTROS:
            raise SystemExit(f"unknown ros2 plot group '{group}' — distros: "
                             f"{', '.join(bench.ROS2_DISTROS)}")
        return ros2_series(distro, chrt_arg)
    if group in HERO_GROUPS:
        # Posture-keyed: the posture decides the chrt suffixes; --chrt is
        # deliberately ignored here (documented in the module docstring).
        return hero_series(group[len("hero-"):])
    if group == "hero":
        raise SystemExit(
            "the 'hero' group is posture-keyed (two heroes, same lines, "
            "one machine variable) — "
            "choose 'hero-tuned' or 'hero-stock' and point --results-dir "
            "at the matching posture run dir")
    raise SystemExit(f"unknown plot group '{group}' — choose from: "
                     f"{', '.join(ALL_GROUPS + HERO_GROUPS)}, all")


HERO_GROUPS = ("hero-tuned", "hero-stock")

# 'all' expands to these; the hero groups are EXCLUDED because they are
# posture-keyed (each needs its own posture run dir + manifest proof) —
# main() prints a loud note on every 'all' expansion so the heroes'
# absence is never silent.
ALL_GROUPS = ("native", "workspace",
              *(f"ros2-{d}" for d in bench.ROS2_DISTROS))

GROUP_TITLES = {
    "native": "Native host round-trip latency (Cerulion API, raw iceoryx2 "
              "floor, zenoh SHM comparison)",
    "workspace": "Cerulion workspace round-trip latency "
                 "(cerulion graph run, split vs mono)",
    # The hero pair: titles/subtitles name the posture; footnotes carry
    # the knob matrix.
    "hero-tuned": "Round-trip latency across stacks (SHM ping-pong) — "
                  "all machine tunings on: C-state cap + SCHED_FIFO",
    "hero-stock": "Round-trip latency across stacks (SHM ping-pong) — "
                  "stock computer: no root, no tuning",
}

# The per-posture knob matrix, baked into every hero render's footnote
# block as a tuple of pre-wrapped lines (kept
# short by hand; an over-long footnote line wraps at the
# rule's right edge instead of clipping). --release strips the knob
# matrix like every other AUTO annotation (explicit --footnote lines are
# kept); the posture itself survives in the title.
HERO_FOOTNOTES = {
    "hero-tuned": (
        "tuned posture, every host knob ON: CPU C-state cap held "
        "(/dev/cpu_dma_latency=0) + SCHED_FIFO (chrt -f 80) on every "
        "measured process;",
        "the raw iceoryx2 floor stays chrt0 by design (spin-bound). "
        "CPU governor is not a suite knob — recorded per run in "
        "run.json."),
    "hero-stock": (
        "stock posture, no root, no tuning: no C-state cap "
        "(CER_BENCH_DMA_LOCK=0), no SCHED_FIFO, no container --device;",
        "the flagless Cerulion graph applies no cap of its own "
        "(monitor-wait park: per-core shallow idle, no global C-state "
        "cap). CPU governor is not a suite knob — recorded per run in "
        "run.json."),
}

DEFAULT_SUBTITLE = ("dot = p50 at each payload size, line joins the p50s, "
                    "shaded band = p50 to p99 tail spread; log/log axes")


def _csv_stem(csv_name: str) -> str:
    """The raw_prefix implied by a CSV filename (basename, results_ prefix
    and .csv suffix stripped). For group series this equals the label; for
    manual --series overlays it recovers the cell identity behind a pretty
    label so role mapping still works."""
    stem = Path(csv_name).name
    if stem.startswith("results_"):
        stem = stem[len("results_"):]
    if stem.endswith(".csv"):
        stem = stem[: -len(".csv")]
    return stem


# ---------------------------------------------------------------- theme
#
# Design language extracted from the approved rmw chart
# (surface, ink hierarchy, hairline grid, footnote block).
#
# PALETTE — these choices replace the
# dataviz reference palette slots this module previously shipped:
#   Cerulion  = #0080FF — both workspace legs; mono is the same-hue light
#               shade (the legs are ONE product, two run shapes).
#   ROS 2     = the Apple Messages green-bubble green family, base #65C466
#               (hex verified against css-tricks.com/apple-messages-color-
#               contrast) — EVERY ROS 2 line lives in this family; the
#               CycloneDDS distros are shade-stepped (release-ordered:
#               newest = the signature green, older = darker), the other
#               rmw families take the olive / jade members, and no_shm
#               variants tint toward the surface as before. Distro/rmw
#               identity additionally rides marker + dash + legend +
#               endpoint labels, never color alone.
#   zenoh SHM / raw iceoryx2 = deliberately NEITHER blue NOR green:
#               orange comparison line + de-emphasis gray floor.
# Validator (dataviz six-check, 2026-08-14, light mode, surface #fcfcfb),
# hero fixed order [#0080ff, #66b3ff, #eb6834, #006038, #2f9440, #65c466]:
# ALL CHECKS PASS — lightness band, chroma floor, CVD (worst adjacent
# ΔE 14.2 protan), normal-vision floor (worst adjacent ΔE 15.1); one
# contrast-relief WARN (#66b3ff 2.16:1, #65c466 2.12:1 vs surface),
# discharged by the secondary encoding this module always ships (legend +
# per-series markers + endpoint labels; the results CSVs are the table-view
# twin). Machines-driver fallback trio [#0080ff, #eb6834, #4a3aa7]: ALL
# CHECKS PASS, no WARNs. DOCUMENTED RESIDUAL of the all-green override:
# a per-distro ros2 group facet that draws cyclonedds + fastdds + rmw_zenoh
# together cannot clear the normal-vision floor inside one green family
# (best achievable worst-pair ΔE ≈ 11.5; the jazzy facet also carries a
# protan 4.9 pair) — those figures put rmw identity on marker + dash +
# legend + endpoint labels; revisit if the FastDDS / rmw_zenoh SHM sweeps
# return to a public page.

SURFACE = "#fcfcfb"   # chart surface
INK = "#0b0b0b"       # primary text (title, legend)
INK2 = "#52514e"      # secondary text (subtitle, axis titles)
MUTED = "#898781"     # tick labels, footnotes
GRID = "#e1e0d9"      # hairline gridlines / footnote rule
AXIS = "#c3c2b7"      # bottom axis rule

BLUE = "#0080ff"      # Cerulion — the
                      # workspace-split headline
BLUE_LT = "#66b3ff"   # Cerulion light shade — the second workspace leg
                      # (mono): a deliberate same-hue shade pair (both legs
                      # ARE the product), disambiguated by direct labels +
                      # legend
ORANGE = "#eb6834"    # native comparison (zenoh SHM) — deliberately neither
                      # blue (Cerulion) nor green (ROS 2)
GREEN_ROS2 = "#65c466"       # Apple Messages green-bubble green (verified —
                             # see the palette comment): lyrical CycloneDDS
GREEN_ROS2_MID = "#2f9440"   # jazzy CycloneDDS (release-ordered shade ramp:
                             # newest distro = the signature green)
GREEN_ROS2_DARK = "#006038"  # humble CycloneDDS
GREEN_FASTDDS = "#989800"    # FastDDS family — the olive member of the
                             # green family
GREEN_RMW_ZENOH = "#10a898"  # rmw_zenoh family — the jade member of the
                             # green family
OCHRE_COMPOSED = "#976125"   # ROS 2 composed + intra-process: a DULL NON-GREEN
                             # ochre-brown (green reads
                             # as "good", and the composed lane is a one
                             # process, no process isolation shape). The
                             # literal taupe region around #a08060 has OKLCH
                             # chroma 0.06 to 0.09, under the dataviz 0.10
                             # floor (reads gray), so the shade was chosen by
                             # a grid search through validate_palette.js: hue
                             # 65, L 0.54, C 0.102. Validator, five-slot legend
                             # order [#0080ff, #66b3ff, #eb6834, #bbbb58,
                             # #976125], light mode, surface #fcfcfb: chroma
                             # PASS, adjacent CVD PASS (worst 9.4 deutan),
                             # normal-vision PASS (worst 15.5), and under
                             # --pairs all PASS on both (worst 8.4 protan vs
                             # the zenoh orange; normal 15.5). The one FAIL in
                             # that run is PRE-EXISTING and not this slot:
                             # the stock tint #bbbb58 sits at L 0.772, just
                             # over the 0.77 band cap (it also drew the same
                             # verdict in the four-color set this replaced,
                             # where the old composed green #65c466 vs #bbbb58
                             # was a protan 1.4 adjacent FAIL). Contrast WARNs
                             # (#66b3ff 2.16:1, #bbbb58 1.97:1) are the
                             # documented relief case: legend + markers +
                             # endpoint labels + the CSV table twin.
YELLOW = "#eda100"    # fallback pool only
MAGENTA = "#e87ba4"   # fallback pool only
VIOLET = "#4a3aa7"    # fallback pool only
RED = "#e34948"       # fallback pool only
GRAY_FLOOR = "#898781"  # de-emphasis gray — the raw iox2 floor is context,
                        # not a comparison line (the dataviz 'emphasis' form);
                        # also 'neither blue nor green'

TIER_PRIMARY, TIER_SECONDARY, TIER_TERTIARY = "primary", "secondary", "tertiary"

# Mark weights per tier (dataviz mark specs: 2px-class lines, >=8px markers
# with a surface ring; tertiary thinner but readable).
TIER_SPEC = {
    TIER_PRIMARY: dict(lw=2.6, ms=7.6, band=0.14, z=3.6),
    TIER_SECONDARY: dict(lw=2.0, ms=6.8, band=0.11, z=3.3),
    TIER_TERTIARY: dict(lw=1.5, ms=6.2, band=0.09, z=3.0),
}

# Fixed fallback order for color_keys no role rule names, assigned
# first-seen, skipping hues already on the figure. Green-family hues are
# excluded from the pool — green is reserved for ROS 2 entities,
# and the blue shade pair stays Cerulion's. Never
# cycled: past the last free slot a series renders in the de-emphasis gray
# with a loud warning — fold or facet instead of minting a 7th hue
# (dataviz non-negotiable). First three slots are validator-PASS as a trio
# (the machines/knobs drivers' per-machine colors ride them).
FALLBACK_SLOTS = (BLUE, ORANGE, VIOLET, MAGENTA, YELLOW, RED)

MARKERS = ["o", "s", "^", "D", "v", "P", "X", "*", "p", "h", "<", ">"]

# CycloneDDS distro → shade within the ROS 2 green family: release-ordered,
# newest = the signature Apple Messages green.
_CDDS_DISTRO_SHADE = {"humble": GREEN_ROS2_DARK, "jazzy": GREEN_ROS2_MID,
                      "lyrical": GREEN_ROS2}


def _role_for(s: Series) -> Optional[Tuple[str, str]]:
    """Entity → (color, tier). Color follows the ENTITY, never the series
    index, so an rmw family keeps its hue in every figure and a filtered
    plot never repaints survivors. Probes color_key, label AND csv stem so
    manual --series overlays with pretty labels still role-map.
    'workspace_split' IS the headline;
    'workspace_default' is the RETIRED name of the flagless mp leg
    (park-wake inflation) — an old default artifact overlaid
    manually still role-maps to the headline slot via the generic
    `cerulion_workspace` probe, but the enumerations no longer emit it.
    The native rules are ANCHORED (exact token / <prefix>_chrtN) so an
    rmw cell stem like `jazzy_zenoh_shm_rclcpp` can never capture the
    native zenoh_shm comparison's slot."""
    parts = (s.color_key, s.label, _csv_stem(s.csv_name))

    def has(sub: str) -> bool:
        return any(sub in p for p in parts)

    def anchored(prefix: str) -> bool:
        return any(p == prefix or p.startswith(prefix + "_chrt")
                   for p in parts)

    # Type-class pod twins BEFORE the generic workspace probes (their
    # names contain the generic substrings): same entity, same hue
    # family, tinted shade — the sanctioned 1-hue-2-shades pairing (the
    # class is also in the legend + marker, never color alone), one tier
    # down so the incumbent variable rows keep the headline weight.
    if has("workspace_mono_pod") or has("cerulion_workspace_mono_pod"):
        return _tint(BLUE_LT, 0.40), TIER_TERTIARY
    if has("workspace_split_pod") or has("cerulion_workspace_split_pod"):
        # 0.20, NOT the 0.40 its mono twin uses, and the asymmetry is
        # forced: BLUE_LT is itself ~a 40% tint of BLUE toward SURFACE,
        # so _tint(BLUE, 0.40) lands on #65b2fd against BLUE_LT's
        # #66b3ff -- a delta of (1, 1, 2), indistinguishable, and it is
        # the MONO row's color. That collision breaks the very pairing
        # the comment above promises: split_pod would read as belonging
        # to the mono hue family. At 0.20 (#3299fe) the four workspace
        # rows separate by at least 50 per channel pairwise, and
        # split_pod stays a shade of BLUE, so the pairing holds on both
        # sides for the first time.
        return _tint(BLUE, 0.20), TIER_SECONDARY
    if has("cerulion_workspace_mono") or has("workspace_mono"):
        return BLUE_LT, TIER_SECONDARY
    if has("cerulion_workspace") or has("workspace_default") \
            or has("workspace_split"):
        return BLUE, TIER_PRIMARY
    if anchored("iox2"):
        return GRAY_FLOOR, TIER_SECONDARY
    if anchored("zenoh_shm"):
        return ORANGE, TIER_SECONDARY
    # Usage-pattern lanes (ros2/memo.md).
    # Both are ROS 2 entities, so both live in the ROS 2 green
    # family (palette rule: never a fallback slot, which
    # would paint a ROS 2 line blue/violet). Shade choices are
    # Provisional, pending plot review:
    #   stock    — rmw_fastrtps at its zero-config defaults = the
    #              FastDDS entity on its default path → the olive
    #              family member, tinted (the same-hue-shade pairing
    #              this module already uses for no_shm).
    #   composed: its own usage-pattern entity; a
    #              dull NON-GREEN ochre-brown (OCHRE_COMPOSED, see the
    #              palette block): green reads as "good", and this lane
    #              is one process with no process isolation, so it is
    #              the one ROS 2 entity that leaves the green family on
    #              purpose. ipcoff = the same entity with the opt-in
    #              disabled: same hue, lighter shade (the sanctioned
    #              degraded-path pairing).
    if has("_stock_"):
        return _tint(GREEN_FASTDDS, 0.35), TIER_TERTIARY
    if has("_composed_"):
        tinted = (_tint(OCHRE_COMPOSED, 0.45) if has("ipcoff")
                  else OCHRE_COMPOSED)
        return tinted, TIER_TERTIARY
    for needle, color in (("_cyclonedds_", None),
                          ("_fastdds_", GREEN_FASTDDS),
                          ("_zenoh_", GREEN_RMW_ZENOH)):
        if has(needle):
            if color is None:
                # CycloneDDS: distro-stepped shade within the ROS 2 green
                # family — release-ordered,
                # newest distro = the signature Apple Messages green.
                color = next((shade for d, shade in _CDDS_DISTRO_SHADE.items()
                              if has(d)), GREEN_ROS2_MID)
            # Within an rmw family, the no-SHM variant is the same entity
            # on a degraded path: same hue, lighter shade (the sanctioned
            # 1-hue-2-shades pairing; identity is also in the legend +
            # marker, never color alone). The image (variable) class
            # rides the same rule at a shallower tint — same rmw entity,
            # different message class, disambiguated by the legend's
            # explicit "sensor_msgs/Image (variable)" vocabulary.
            if has("_no_shm_"):
                tinted = _tint(color, 0.45)
            elif has("_image_"):
                tinted = _tint(color, 0.28)
            else:
                tinted = color
            return tinted, TIER_TERTIARY
    return None


def _tint(hex_color: str, frac: float) -> str:
    """Blend a hex color toward the surface by `frac` (same-hue shade)."""
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (1, 3, 5))
    sr, sg, sb = (int(SURFACE[i:i + 2], 16) for i in (1, 3, 5))
    mix = tuple(round(c + (s - c) * frac) for c, s in ((r, sr), (g, sg), (b, sb)))
    return "#{:02x}{:02x}{:02x}".format(*mix)


def _rel_luminance(hex_color: str) -> float:
    def chan(c: int) -> float:
        v = c / 255.0
        return v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (1, 3, 5))
    return 0.2126 * chan(r) + 0.7152 * chan(g) + 0.0722 * chan(b)


def _contrast(a: str, b: str) -> float:
    la, lb = _rel_luminance(a), _rel_luminance(b)
    hi, lo = max(la, lb), min(la, lb)
    return (hi + 0.05) / (lo + 0.05)


def _label_ink(color: str) -> str:
    """Endpoint labels wear the series color only when it clears 3:1 on the
    surface (computed, not eyeballed); light hues fall back to the
    secondary text token — text never becomes illegible to carry identity,
    which the adjacent colored line already carries."""
    return color if _contrast(color, SURFACE) >= 3.0 else INK2


def _wants_endpoint_label(s: Series, tier: str) -> bool:
    """Endpoint value labels ride every primary/secondary line, plus the
    ROS 2 CycloneDDS cells (tertiary — the per-distro comparison family)
    and the usage-pattern lanes (stock/composed — the hero pair's ROS 2
    lines, so every hero line carries its endpoint value).
    Probes label AND csv stem so manual --series overlays with pretty
    labels keep their labels (same discipline as _role_for)."""
    if tier in (TIER_PRIMARY, TIER_SECONDARY):
        return True
    parts = (s.label, _csv_stem(s.csv_name))
    return any("_cyclonedds_" in p or "_stock_" in p or "_composed_" in p
               for p in parts)


# ------------------------------------------- beneath-axis rate braces

# Beneath the x-axis, horizontal curly
# BRACES span contiguous payload groups sharing a (message type, rate)
# label. On quiescent figures the braces group by the pinned schedule's
# rate classes (derived from bench.quiescent_schedule — ONE truth, never
# a re-typed table); on fixed100 figures one brace spans the whole sweep
# at the uniform target. backtoback figures draw none (saturation pacing
# makes no rate claim). The braces are AXIS FURNITURE, not commentary —
# like the loan callout they prevent a mis-reading (a mixed-rate axis is
# the §17 hazard), so they are kept under --release. The type-class half
# of the decision rides the legend (the variable rows carry the real type
# name, sensor_msgs/Image; pod rows the fixed-array vocabulary) plus a
# footnote line when a figure carries both classes.

# The three sensor-class nouns the decision names; other schedule rates
# carry the rate alone (no invented vocabulary).
_RATE_CLASS_NOUN = {1000: "IMU class", 30: "camera class", 10: "lidar class"}


def _fmt_rate(hz: int) -> str:
    return f"@{hz // 1000} kHz" if hz % 1000 == 0 and hz >= 1000 else f"@{hz} Hz"


def _axis_brace_groups(sizes: Sequence[int],
                       variant: str) -> List[Tuple[int, int, str]]:
    """(x_lo, x_hi, label) spans for the beneath-axis braces — contiguous
    runs of the plotted sizes sharing a schedule rate.

    Pure, and derived from the VARIANT'S PINNED SCHEDULE
    (bench.quiescent_schedule / bench.FIXED100_RATE_HZ) — which is what
    the sweep TARGETED, not necessarily what it paced. TWO knobs override
    the schedule rate for every size and are recorded in no CSV column,
    sidecar or manifest field, so a figure can see neither: the workspace
    runner's quiescent diagnostic CER_BENCH_FORCE_RATE_HZ (METHODOLOGY
    § "The rate axis"), and — on the ROS 2 side — an AMBIENT
    CER_BENCH_TARGET_RATE_HZ, which run_bench.sh takes as the highest
    override precedence for every payload. bench.py can leak neither: it
    passes `-e KEY=VALUE` only, and only on the fixed100 ladder, so both
    are hand-run exposures.
    Every label therefore says `target`, the wording the fixed100 brace
    has always used, and a forced-rate run is a DIAGNOSTIC figure, not a
    citable one (stated in METHODOLOGY beside the knob)."""
    if not sizes:
        return []
    if variant == "fixed100":
        return [(sizes[0], sizes[-1],
                 f"{_fmt_rate(bench.FIXED100_RATE_HZ)} uniform target")]
    if variant != "quiescent":
        return []
    groups: List[Tuple[int, int, str]] = []
    run_start = sizes[0]
    run_rate = bench.quiescent_schedule(sizes[0])[0]
    prev = sizes[0]
    for sz in list(sizes[1:]) + [None]:  # type: ignore[list-item]
        rate = bench.quiescent_schedule(sz)[0] if sz is not None else None
        if rate != run_rate:
            noun = _RATE_CLASS_NOUN.get(run_rate)
            # "target", like the fixed100 brace: the schedule is what the
            # sweep asked for, and CER_BENCH_FORCE_RATE_HZ can have
            # overridden it invisibly (see the docstring).
            label = (f"{noun} {_fmt_rate(run_rate)} target" if noun
                     else f"{_fmt_rate(run_rate)} target")
            groups.append((run_start, prev, label))
            if sz is not None:
                run_start, run_rate = sz, rate
        if sz is not None:
            prev = sz
    return groups


def _draw_axis_braces(ax, sizes: Sequence[int], variant: str) -> bool:
    """Draw the specified curly braces beneath the x-axis. x in DATA
    coordinates (log-aware: geometry computed in log10 space so a brace
    reads symmetric on the log axis), y in axes fraction via
    get_xaxis_transform(); clip_on=False so they render below the axis.
    Returns True when any brace was drawn (the caller widens the bottom
    margin + pushes the x-title below the brace row)."""
    import math

    from matplotlib.patches import PathPatch
    from matplotlib.path import Path as MplPath

    groups = _axis_brace_groups(sorted(sizes), variant)
    if not groups:
        return False

    y = -0.075        # brace ends (just under the tick labels)
    shoulder = -0.105  # horizontal body
    tip = -0.135       # center tip (points at the label)
    # Narrow (single-size) groups sit one tick apart at the top of the
    # sweep, so their labels would collide on one row — alternate
    # consecutive narrow groups between two label rows.
    label_row = 0
    prev_narrow = False
    for x_lo, x_hi, label in groups:
        l0, l1 = math.log10(x_lo), math.log10(x_hi)
        # A single-size group has zero log-span; give it a visible arm
        # half a tick-gap wide (the sweep is power-of-4-ish spaced).
        narrow = l1 - l0 < 0.2
        if narrow:
            pad = 0.22
            l0, l1 = l0 - pad, l1 + pad
        label_row = (label_row + 1) % 2 if (narrow and prev_narrow) else 0
        prev_narrow = narrow
        lm = (l0 + l1) / 2.0
        r = min(0.18, (l1 - l0) * 0.18)  # end/tip curl, in log-x units

        def x(lv: float) -> float:
            return 10.0 ** lv

        verts = [
            (x(l0), y),
            (x(l0), shoulder), (x(l0 + r), shoulder),   # left end curl
            (x(lm - r), shoulder),                       # left arm
            (x(lm), shoulder), (x(lm), tip),             # center dip →
            (x(lm), shoulder), (x(lm + r), shoulder),    # ← center rise
            (x(l1 - r), shoulder),                       # right arm
            (x(l1), shoulder), (x(l1), y),               # right end curl
        ]
        codes = [
            MplPath.MOVETO,
            MplPath.CURVE3, MplPath.CURVE3,
            MplPath.LINETO,
            MplPath.CURVE3, MplPath.CURVE3,
            MplPath.CURVE3, MplPath.CURVE3,
            MplPath.LINETO,
            MplPath.CURVE3, MplPath.CURVE3,
        ]
        ax.add_patch(PathPatch(
            MplPath(verts, codes), transform=ax.get_xaxis_transform(),
            facecolor="none", edgecolor=AXIS, linewidth=1.2,
            clip_on=False, zorder=2.5))
        ax.text(x(lm), tip - 0.012 - label_row * 0.034, label,
                transform=ax.get_xaxis_transform(),
                ha="center", va="top", fontsize=8.5, color=INK2,
                clip_on=False, zorder=2.5)
    return True


# ------------------------------------------------------- pretty labels

_WS_RE = re.compile(r"^cerulion_workspace_([a-z0-9_]+)_chrt([01])$")
_NATIVE_RE = re.compile(r"^(iox2|zenoh_shm)_chrt([01])$")
# Type-class image cells (CER_BENCH_MSG=image — the `image` token rides
# between the shm mode and the recv path). Matched BEFORE _ROS2_RE
# (whose shm alternation would not admit the token anyway).
# The non-class groups are as WIDE as this file's label vocabulary can
# render, even though bench.py enumerates image cells on shm x rclcpp x
# be1 only. A narrow spelling would encode ENUMERATION POLICY in a
# PRESENTATION regex: an image cell on any other axis would match neither
# this nor the pod stem matcher, so _type_class_of would answer None, the
# class footnote would vanish and pretty_label would fall through to the
# raw CSV stem — the same class of loss as the zc pod rows before
# _ROS2_POD_STEM_RE existed, though through a different alternation
# (recv/qos here, shm-mode there). The `_image_` literal is what makes the
# class, and no pod stem carries it, so the policy stays where it is
# enforced (bench.py's enumeration and run_bench.sh's axis gates). `zc`
# is deliberately NOT admitted here: image x zc is structurally refused
# (DataSharing needs a plain bounded type), and this vocabulary has no zc
# word to render — the same gap _ROS2_RE has, tracked as its deferral.
_ROS2_IMG_RE = re.compile(
    r"^(humble|jazzy|lyrical)_(cyclonedds|fastdds|zenoh|cerulion)_"
    r"(shm|no_shm)_image_(rclcpp|loan)_(be1|rel10)_chrt([01])$")
_ROS2_RE = re.compile(
    r"^(humble|jazzy|lyrical)_(cyclonedds|fastdds|zenoh|cerulion)_"
    r"(shm|no_shm)_(rclcpp|loan)_(be1|rel10)_chrt([01])$")
# Usage-pattern lanes: qos-token-less names by design —
# both lanes run qos=stock (rmw_qos_profile_default) definitionally.
_STOCK_RE = re.compile(
    r"^(humble|jazzy|lyrical)_stock_rclcpp_chrt([01])$")
_COMPOSED_RE = re.compile(
    r"^(humble|jazzy|lyrical)_composed_(ipcon|ipcoff)_rclcpp_chrt([01])$")
# Pod MATRIX rows for the type-class detector: every shm mode bench.py can
# mint — shm | no_shm | zc (the FastDDS DataSharing lane). _ROS2_RE stays
# (shm|no_shm) on purpose: it drives pretty_label, whose vocabulary has no
# zc entry (a zc row's legend text is the base cascade's open item), so the
# detector carries its own stem matcher instead of widening a vocabulary
# regex it does not own. check_percentile_parity.py cross-pins this
# alternation against the REAL enumeration (every enumerated pod cell must
# classify pod), so it cannot drift from bench.py silently.
_ROS2_POD_STEM_RE = re.compile(
    r"^(humble|jazzy|lyrical)_(cyclonedds|fastdds|zenoh|cerulion)_"
    r"(shm|no_shm|zc)_(rclcpp|loan)_(be1|rel10)_chrt([01])$")


# The ROS 2 stack spells the unbounded class `image` — in its env var, its
# cell token and its CSV stem — while the workspace stack spells it
# `variable`. The vocabularies differ deliberately (METHODOLOGY § "The
# type-class axis"), and _type_class_of below is the ONE place they meet,
# so the normalized values are named rather than written as literals: a
# comparison against ("ros2", "image") reads as obviously correct and is
# permanently False.
CLASS_POD = "pod"
CLASS_VARIABLE = "variable"


def _type_class_of(s: Series) -> Optional[Tuple[str, str]]:
    """(stack, class) for a series on the type-class axis (METHODOLOGY
    § "The type-class axis"), or None for a series the axis does not
    describe (the native floor lines carry no typed schema; an unknown
    manual overlay claims nothing). Read off the csv STEM — the cell
    identity behind a pretty label — never the label text.
      workspace: the pinned token-less legs ARE the variable class
                 (sensor_msgs/Image, data loaned per tick); `_pod`
                 legs are the fixed-PodPayload class.
      ros2:      the `image` token marks the variable class; every
                 other matrix cell (shm, no_shm AND the zc DataSharing
                 lane) and both usage lanes ride Pod<N>."""
    stem = _csv_stem(s.csv_name)
    m = _WS_RE.match(stem)
    if m:
        return ("workspace",
                CLASS_POD if m.group(1).endswith("_pod") else CLASS_VARIABLE)
    if _ROS2_IMG_RE.match(stem):
        return ("ros2", CLASS_VARIABLE)
    if (_ROS2_POD_STEM_RE.match(stem) or _STOCK_RE.match(stem)
            or _COMPOSED_RE.match(stem)):
        return ("ros2", CLASS_POD)
    return None


def _paired_class_stacks(series: Sequence[Series]) -> Set[str]:
    """The stacks that carry BOTH classes in this figure — an image cell
    beside a ROS 2 pod cell, or a workspace pod leg beside a variable
    leg.

    Per stack, not per figure: a group that pairs the two WORKSPACE legs
    says nothing about whether the ROS 2 rows have an image twin, and
    after this axis landed the standard groups always carry both
    workspace legs — so a figure-wide boolean would tag every ROS 2 pod
    row on every incumbent figure.

    Two callers, two readings. The matched-quantity footnote describes a
    PAIRING and fires for ANY paired stack, so a lone tokened row (a
    --skip-missing render whose twin CSV is absent, a single image
    --series overlay) must not carry a caption claiming a pair, and the
    incumbent token-less figures keep their captions because the axis is
    keyed on the tokened rows this PR added, not on the cross-stack
    contrast the two HERO groups have (workspace variable legs beside
    ROS 2 pod cells). The ROS 2 legend token reads the SET and asks for
    its own stack.

    Per stack, not per figure, is also the reading that means what it
    says. On the groups this suite SHIPS the two readings agree —
    `workspace` carries workspace rows only, `ros2-*` ROS 2 rows only,
    and the heroes are cross-stack but carry no pair — so the scoping
    changes no shipped figure today. It is reachable all the same: a
    `--series` overlay mixing a workspace pair with ROS 2 pod rows is
    exactly the shape a figure-wide boolean mislabels, and a future group
    carrying both stacks inherits the right answer instead of a latent
    bug."""
    by_stack: Dict[str, set] = {}
    for s in series:
        tc = _type_class_of(s)
        if tc is not None:
            by_stack.setdefault(tc[0], set()).add(tc[1])
    return {stack for stack, classes in by_stack.items()
            if {CLASS_POD, CLASS_VARIABLE} <= classes}


# The ROS 2 pod class has no token in its cell name, so its incumbent
# legend text names transport/recv/QoS and nothing about the type. That
# is fine on a figure with no image row — but where the two classes SHARE
# a figure the image rows say "sensor_msgs/Image (variable)" while the
# pod rows say nothing, and under `--release` the matched-quantity
# footnote (which is where the pod class is named) is stripped as an
# annotation: the artifact that leaves the repo then identifies one class
# and not the other. So a paired figure states BOTH, in the legend, which
# release renders keep. Scoped to the pairing on purpose: an incumbent
# figure carrying no image row keeps its legend text unchanged. (The
# FIGURE is not unchanged — this PR also adds the beneath-axis rate
# braces, which draw on every quiescent and fixed100 render.)
_ROS2_POD_CLASS_TOKEN = "Pod<N> (fixed array)"


def _legend_label(s: Series, *, paired_stacks: Set[str]) -> str:
    """The legend text for one series. `paired_stacks` is the per-figure
    set from `_paired_class_stacks` — the ROS 2 token is added only when
    the ROS 2 stack itself carries both classes, so an incumbent ROS 2
    figure's LEGEND TEXT is unchanged even where the workspace legs
    beside it are a pair. Keyword-only: `True`/`False` at a call site said
    nothing about which decision it carried. A pretty label — a
    `--series` override, or a hero group's fixed `_HERO_PRETTY`
    entry, which stays IDENTICAL across the two postures by design — is
    not this function's to edit and is returned verbatim."""
    if s.pretty:
        return s.pretty
    text = pretty_label(s.label)
    if "ros2" in paired_stacks and _type_class_of(s) == ("ros2", CLASS_POD):
        text += f" · {_ROS2_POD_CLASS_TOKEN}"
    return text


_WS_PRETTY = {
    # Type-class vocabulary (plot decision): variable legs carry
    # the REAL type name (sensor_msgs/Image), pod legs the "fixed
    # array" label. The token-less legs ARE the variable class (they
    # predate the axis; their pinned prefixes stay).
    "split": "Cerulion — cerulion graph run (multi-process split) · "
             "sensor_msgs/Image (variable)",
    "mono": "Cerulion — graph run --single-process (mono) · "
            "sensor_msgs/Image (variable)",
    "split_pod": "Cerulion — cerulion graph run (multi-process split) · "
                 "fixed array (pod)",
    "mono_pod": "Cerulion — graph run --single-process (mono) · "
                "fixed array (pod)",
    "default": "Cerulion — retired flagless default leg",
}
_NATIVE_PRETTY = {
    "iox2": "raw iceoryx2 — transport floor",
    "zenoh_shm": "zenoh SHM (z_ping / z_pong)",
}
# `cerulion` renders as rmw_cerulion, matching how it is selected
# (RMW_IMPLEMENTATION=rmw_cerulion) rather than as a product name: on a
# chart beside CycloneDDS and FastDDS the reader is comparing rmw
# implementations, and the row is ROS 2 running on one of them.
_RMW_PRETTY = {"cyclonedds": "CycloneDDS", "fastdds": "FastDDS",
               "zenoh": "rmw_zenoh", "cerulion": "rmw_cerulion"}
_RECV_PRETTY = {"rclcpp": "rclcpp",
                "loan": "loan take (zero-copy receive)"}
_QOS_PRETTY = {"be1": "best-effort/1", "rel10": "reliable/10"}


def pretty_label(raw: str) -> str:
    """Human legend vocabulary for the pinned raw_prefix inventory; unknown
    labels (manual overlays) pass through verbatim."""
    m = _WS_RE.match(raw)
    if m:
        leg, chrt = m.groups()
        base = _WS_PRETTY.get(leg, f"Cerulion workspace — {leg} leg")
        return base + (" · chrt1" if chrt == "1" else "")
    m = _NATIVE_RE.match(raw)
    if m:
        return _NATIVE_PRETTY[m.group(1)] + (
            " · chrt1" if m.group(2) == "1" else "")
    m = _ROS2_IMG_RE.match(raw)
    if m:
        distro, rmw, shm, recv, qos, chrt = m.groups()
        # Plot decision: the variable class carries the real type
        # name. (A pod cell's label stays the incumbent vocabulary and
        # gains "Pod<N> (fixed array)" only where the ROS 2 stack carries
        # both classes in one figure — see _legend_label.) The transport
        # word is READ from the stem, never assumed: hardcoding "SHM"
        # here would label a no_shm row as shared-memory.
        text = " · ".join([f"ROS 2 {distro}", _RMW_PRETTY[rmw],
                           "SHM" if shm == "shm" else "no SHM",
                           "sensor_msgs/Image (variable)",
                           _RECV_PRETTY.get(recv, recv),
                           _QOS_PRETTY[qos]])
        if chrt == "1":
            text += " · chrt1"
        return text
    m = _ROS2_RE.match(raw)
    if m:
        distro, rmw, shm, recv, qos, chrt = m.groups()
        text = " · ".join([f"ROS 2 {distro}", _RMW_PRETTY[rmw],
                           "SHM" if shm == "shm" else "no SHM",
                           _RECV_PRETTY.get(recv, recv),
                           _QOS_PRETTY[qos]])
        if chrt == "1":
            text += " · chrt1"
        return text
    m = _STOCK_RE.match(raw)
    if m:
        distro, chrt = m.groups()
        text = (f"ROS 2 {distro} · stock zero-config "
                f"(rmw_fastrtps defaults) · rclcpp · reliable/10")
        if chrt == "1":
            text += " · chrt1"
        return text
    m = _COMPOSED_RE.match(raw)
    if m:
        distro, ipc, chrt = m.groups()
        text = (f"ROS 2 {distro} · composed (one process) · "
                f"intra-process {'on' if ipc == 'ipcon' else 'off'} · "
                f"rclcpp · reliable/10")
        if chrt == "1":
            text += " · chrt1"
        return text
    return raw


# ---------------------------------------------------------------- csv / plot


@dataclass
class SeriesData:
    """One parsed results_<prefix>.csv.

    p99_us is None per row when the A2 suppression emptied the cell
    (fewer than 20 pooled tail samples) — the band narrows to the line
    there. band_lo/hi_us carry the cross-rep p50 spread (rep_p50_min/max_ns);
    None per row when rep_count <= 1 or on legacy CSVs that
    predate the rep columns; drawn as capped whiskers. The p50 line is the
    MEDIAN of per-rep p50s (compile_csv's headline rule).

    achieved_rate_hz (fixed100 variant): the rate the row
    actually ran at, per row — None on quiescent/backtoback and legacy
    CSVs (no sidecar); an int below the 100 Hz target means the fallback
    ladder engaged (the point is annotated '@NHz'); the string 'mixed'
    means the reps disagreed. did_not_sustain_sizes are payloads whose
    fixed100 ladder was EXHAUSTED — no latency exists for them (the CSV
    row is empty by design); they count as PRESENT for row-completeness
    and are named in an in-figure note, never silently absent."""
    sizes: List[int]
    p50_us: List[float]
    p99_us: List[Optional[float]]
    band_lo_us: List[Optional[float]]
    band_hi_us: List[Optional[float]]
    iterations: List[int]
    achieved_rate_hz: List[Optional[object]]
    did_not_sustain_sizes: List[int]


def parse_results_csv(path: Path) -> SeriesData:
    """Parse a results_<prefix>.csv.

    Tolerates (a) leading '#' schema-comment lines (compile_csv writes
    them since the A6 one_way definition landed), (b) missing rep columns
    (legacy pre-A1 CSVs), and (c) EMPTY percentile cells (the A2
    suppression) — a row whose headline p50 was suppressed is dropped with
    a loud warning rather than crashing the parse; an empty p99 keeps the
    row and narrows the tail band to the line at that point. A fixed100
    did_not_sustain row (empty stats, achieved_rate_hz=did_not_sustain)
    is an ACCOUNTED outcome, recorded in did_not_sustain_sizes rather
    than warned as a suppressed point."""
    data = SeriesData([], [], [], [], [], [], [], [])
    with path.open() as f:
        rows = [line for line in f if not line.startswith("#")]
    for row in csv.DictReader(rows):
        achieved_raw = (row.get("achieved_rate_hz") or "").strip()
        p50_raw = (row.get("round_trip_p50_ns") or "").strip()
        if achieved_raw == "did_not_sustain":
            data.did_not_sustain_sizes.append(int(row["payload_bytes"]))
            continue
        if not p50_raw:
            print(f"warn: {path.name}: payload {row.get('payload_bytes')} "
                  f"has a suppressed/empty p50 — dropping the point",
                  file=sys.stderr)
            continue
        if not achieved_raw:
            data.achieved_rate_hz.append(None)
        elif achieved_raw.isdigit():
            data.achieved_rate_hz.append(int(achieved_raw))
        else:
            data.achieved_rate_hz.append(achieved_raw)   # 'mixed'
        data.sizes.append(int(row["payload_bytes"]))
        data.p50_us.append(int(p50_raw) / 1000.0)  # ns → µs
        p99_raw = (row.get("round_trip_p99_ns") or "").strip()
        data.p99_us.append(int(p99_raw) / 1000.0 if p99_raw else None)
        try:
            data.iterations.append(int((row.get("iterations") or "0").strip()
                                       or "0"))
        except ValueError:
            data.iterations.append(0)
        lo_raw = (row.get("rep_p50_min_ns") or "").strip()
        hi_raw = (row.get("rep_p50_max_ns") or "").strip()
        try:
            rep_count = int((row.get("rep_count") or "1").strip() or "1")
        except ValueError:
            rep_count = 1
        if rep_count > 1 and lo_raw and hi_raw:
            data.band_lo_us.append(int(lo_raw) / 1000.0)
            data.band_hi_us.append(int(hi_raw) / 1000.0)
        else:
            data.band_lo_us.append(None)
            data.band_hi_us.append(None)
    return data


def format_size(b: int) -> str:
    """Binary units, labeled as binary (the sweep is power-of-two)."""
    if b >= 1024 * 1024:
        return f"{b // (1024 * 1024)} MiB"
    if b >= 1024:
        return f"{b // 1024} KiB"
    return f"{b} B"


def _fmt_us(v: float) -> str:
    """Human latency units for ticks/labels (input µs)."""
    if v >= 1e6:
        return f"{v / 1e6:.3g} s"
    if v >= 1000.0:
        return f"{v / 1000.0:.3g} ms"
    return f"{v:.3g} µs"


def csv_path(results_dir: Path, s: Series) -> Path:
    """Resolve a series' CSV path (absolute paths pass through — used by
    manual --series overlays; group series are always run-dir-relative)."""
    p = Path(s.csv_name)
    return p if p.is_absolute() else results_dir / p


def check_missing(series: Sequence[Series], results_dir: Path,
                  skip_missing: bool, context: str) -> List[Series]:
    """FAIL LOUDLY on missing expected CSVs (or warn-and-drop under
    --skip-missing). Returns the series that exist."""
    present: List[Series] = []
    missing: List[Series] = []
    for s in series:
        if csv_path(results_dir, s).exists():
            present.append(s)
        else:
            missing.append(s)
    if missing:
        for s in missing:
            print(f"{'warn' if skip_missing else 'error'}: {context}: expected "
                  f"CSV missing: {s.csv_name} (cell {s.label})", file=sys.stderr)
        if not skip_missing:
            print(f"error: {context}: {len(missing)} expected CSV(s) missing "
                  f"from {results_dir} — a missing line silently vanishing is "
                  f"the exact footgun this check exists for. Either produce "
                  f"the cells (bench.py) or pass --skip-missing to accept a "
                  f"partial plot loudly.", file=sys.stderr)
            raise SystemExit(1)
    return present


def check_row_completeness(series: Sequence[Series], results_dir: Path,
                           skip_missing: bool, context: str) -> None:
    """Strict mode: every PRESENT series must carry ALL expected payload
    rows (the pinned sweep, bench.PAYLOAD_SIZES) — file existence alone
    would let a partially-swept cell render as a plausible shorter line.
    Under --skip-missing the incomplete series warn loudly per missing row
    and still plot their partial rows."""
    bad: List[Tuple[Series, List[int]]] = []
    for s in series:
        data = parse_results_csv(csv_path(results_dir, s))
        # A fixed100 did_not_sustain payload is PRESENT (an accounted
        # no-latency outcome with its own empty row) — only a payload
        # with neither samples nor a verdict is a completeness hole.
        missing = [sz for sz in bench.PAYLOAD_SIZES
                   if sz not in data.sizes
                   and sz not in data.did_not_sustain_sizes]
        if missing:
            bad.append((s, missing))
    if bad:
        for s, missing in bad:
            print(f"{'warn' if skip_missing else 'error'}: {context}: "
                  f"{s.csv_name} is missing payload row(s): "
                  f"{', '.join(map(str, missing))}", file=sys.stderr)
        if not skip_missing:
            print(f"error: {context}: {len(bad)} series with incomplete "
                  f"payload rows — the CSVs exist but do not carry the full "
                  f"{len(bench.PAYLOAD_SIZES)}-size sweep. Re-run the "
                  f"failed cells + compile_csv.py (strict by default), or "
                  f"pass --skip-missing to plot the partial rows loudly.",
                  file=sys.stderr)
            raise SystemExit(1)


# ------------------------------------------------------- figure plumbing


def _apply_theme(matplotlib) -> None:
    """Reference-chart theme: system-sans type resolved against what the
    machine actually has (no findfont warnings), real text in SVG output."""
    from matplotlib import font_manager
    available = {f.name for f in font_manager.fontManager.ttflist}
    fams = [f for f in ("Helvetica Neue", "Helvetica", "Arial",
                        "Liberation Sans") if f in available]
    fams.append("DejaVu Sans")
    matplotlib.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": fams,
        "svg.fonttype": "none",       # keep SVG text as <text>, like the ref
        "text.color": INK,
        "axes.edgecolor": AXIS,
        "axes.labelcolor": INK2,
        "xtick.color": AXIS,
        "ytick.color": AXIS,
        "xtick.labelcolor": MUTED,
        "ytick.labelcolor": MUTED,
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
    })


_SVG_FONT_STACK = "system-ui, -apple-system, 'Segoe UI', sans-serif"


def _rewrite_svg_font_stack(path: Path) -> None:
    """Best-effort: swap the locally-resolved font family in the saved SVG
    for the reference chart's system-ui stack, so the SVG renders with the
    viewer's UI sans anywhere (the concrete font matplotlib resolved here
    won't exist on every machine). Cosmetic — a no-op on unexpected SVG
    shapes, never a failure."""
    try:
        text = path.read_text()
    except OSError:
        return
    new = re.sub(r"font-family:\s*(?:'[^']*'|\"[^\"]*\"|[^;\"'>]+)",
                 f"font-family: {_SVG_FONT_STACK}", text)
    new = re.sub(r"(font:\s*(?:[0-9.]+[a-z%]*\s+|[a-z-]+\s+)*?[0-9.]+px)\s+"
                 r"(?:'[^']*'|\"[^\"]*\"|[^;\"'>]+)",
                 rf"\1 {_SVG_FONT_STACK}", new)
    if new != text:
        path.write_text(new)


def _assign_visuals(series: Sequence[Series]) -> Dict[str, Tuple[str, str]]:
    """color_key → (color, tier). Role-mapped entities first; unmapped keys
    take fallback slots in first-seen order (skipping hues already on the
    figure); an exhausted palette renders in de-emphasis gray, loudly."""
    assigned: Dict[str, Tuple[str, str]] = {}
    used: set = set()
    for s in series:
        if s.color_key in assigned or s.keyed:
            # An explicit @KEY never role-maps: the entity rule reads the
            # csv stem, and a keyed overlay exists precisely because that
            # stem (one cell on several machines) is not what the color
            # should distinguish.
            continue
        role = _role_for(s)
        if role is not None:
            assigned[s.color_key] = role
            used.add(role[0])
    for s in series:
        if s.color_key in assigned:
            continue
        slot = next((c for c in FALLBACK_SLOTS if c not in used), None)
        if slot is None:
            print(f"warn: palette exhausted for series '{s.label}' — "
                  f"rendering in de-emphasis gray. More than "
                  f"{len(FALLBACK_SLOTS)} distinct entities on one figure "
                  f"is past the categorical ceiling: split the plot "
                  f"(groups/facets) instead.", file=sys.stderr)
            slot = GRAY_FLOOR
        assigned[s.color_key] = (slot, TIER_SECONDARY)
        used.add(slot)
    return assigned


# Endpoint-label geometry (display px; y grows upward in display space).
# Every endpoint value label sits in the right
# GUTTER — left-anchored LABEL_XPAD px past its line's LAST point and
# vertically centered on that endpoint — so the number unambiguously
# belongs to its line. Collision handling is VERTICAL-ONLY nudging within
# the gutter: labels keep the ascending order of the values they report
# (numeric order == vertical order — the attribution property the old
# obstacle-search could invert), and a collision cluster settles CENTERED
# on its members' endpoints via _layout_label_column. The axes keep a
# widened right margin (AXES_WIDTH, was 0.905 pre-gutter) so the gutter
# fits the longest label.
LABEL_XPAD = 9.0    # gap between the endpoint marker and the label's left edge
LABEL_GAP = 3.0     # minimum vertical gap between two label boxes
LABEL_AXES_PAD = 4.0  # gutter pad (px) the label column may extend past
                      # the axes' top/bottom edge — never into the
                      # title/legend band or the footnote block
AXES_LEFT = 0.062
AXES_WIDTH = 0.858  # the freed right margin IS the label gutter

# The loan-lane fine print: whenever a figure
# carries a loan-take lane, a compact in-figure callout states what that
# lane IS — per the verified 2026-08-14 audit: the lane is CROSS-PROCESS
# (3 separate OS processes talking only via the RMW), reads via the raw
# rcl API (rcl_take_loaned_message — rclcpp has no waitset-compatible
# loaned take, rclcpp#1699), carries POD payloads only, and returns each
# loan manually. Kept in --release renders: like the legend, it prevents a
# mis-reading (the flat lane is NOT the rclcpp callback path) — that is
# data integrity, not annotation garnish.
LOAN_CALLOUT = ("zero-copy take: raw rcl API, POD types only, manual loan "
                "return —\nnot the rclcpp callback path; cross-process")


def _is_loan_series(s: Series) -> bool:
    """True when the series is a ROS 2 loan-take receive lane (probes label
    AND csv stem, same discipline as _role_for). Matched on the pod-stem
    matcher, which admits every shm mode bench.py mints (shm | no_shm |
    zc) — on _ROS2_RE the FastDDS DataSharing loan row
    (`*_zc_loan_*`) went undetected and a figure whose only loan lane
    was that row drew no LOAN_CALLOUT. Image rows never match: the
    variable class cannot loan (its lane is a structural skip)."""
    for cand in (s.label, _csv_stem(s.csv_name)):
        m = _ROS2_POD_STEM_RE.match(cand)
        if m and m.group(4) == "loan":
            return True
    return False


def _layout_label_column(prefs: Sequence[float], heights: Sequence[float],
                         gap: float, lo: float, hi: float) -> List[float]:
    """Balanced 1-D column layout for the gutter labels (pool-adjacent-
    violators): boxes keep their (ascending, numeric) order and never
    overlap, and each collision CLUSTER settles on the mean of its members'
    preferred slots — so two near-coincident endpoints split the
    displacement between their two labels (one eases down, one eases up)
    instead of the whole stack riding upward away from its lines, which is
    the attribution hazard the vertical-only rule exists to avoid.

    `prefs` are preferred box BOTTOMS in ascending order; returns the
    placed bottoms, clamped to [lo, hi]."""
    clusters: List[dict] = []
    for p, h in zip(prefs, heights):
        clusters.append(dict(offs=[0.0], pref_sum=p, n=1, height=h))
        while len(clusters) >= 2:
            prev, cur = clusters[-2], clusters[-1]
            if (cur["pref_sum"] / cur["n"]
                    >= prev["pref_sum"] / prev["n"] + prev["height"] + gap):
                break
            off = prev["height"] + gap
            prev["offs"].extend(o + off for o in cur["offs"])
            prev["pref_sum"] += cur["pref_sum"] - cur["n"] * off
            prev["n"] += cur["n"]
            prev["height"] = off + cur["height"]
            clusters.pop()
    out: List[float] = []
    for c in clusters:
        base = min(max(c["pref_sum"] / c["n"], lo), hi - c["height"])
        out.extend(base + o for o in c["offs"])
    return out


def plot_group(series: Sequence[Series], results_dir: Path, out_path: Path,
               title: str,
               subtitle: Optional[str] = None,
               footnotes: Sequence[str] = (),
               omitted: Sequence[str] = (),
               watermark: Optional[str] = None,
               release: bool = False,
               variant: Optional[str] = None,
               ylabel: Optional[str] = None,
               legend_order: str = "tier") -> None:
    import matplotlib
    matplotlib.use("Agg")
    _apply_theme(matplotlib)
    import matplotlib.patheffects as path_effects
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import FuncFormatter, LogLocator, NullLocator

    visuals = _assign_visuals(series)

    # Parse everything first (drop empties loudly), then draw tertiary →
    # primary so the headline sits on top.
    parsed: List[Tuple[Series, SeriesData, str, str]] = []
    for s in series:
        data = parse_results_csv(csv_path(results_dir, s))
        if not data.sizes:
            # Header-only CSV (every payload row was empty) — same loud
            # treatment as a missing file, but it cannot be recovered by
            # re-plotting, so warn and drop.
            print(f"warn: skipping empty csv: {s.label} ({s.csv_name})",
                  file=sys.stderr)
            continue
        color, tier = visuals[s.color_key]
        parsed.append((s, data, color, tier))

    if not parsed:
        print(f"error: no plottable series for {out_path.name}", file=sys.stderr)
        raise SystemExit(1)

    tier_rank = {TIER_TERTIARY: 0, TIER_SECONDARY: 1, TIER_PRIMARY: 2}
    draw_order = sorted(range(len(parsed)),
                        key=lambda i: tier_rank[parsed[i][3]])

    # Decided once, from the series that will actually be DRAWN (an empty
    # CSV was dropped above, so a twin whose file is header-only cannot
    # make the figure claim a pairing it does not show).
    paired_stacks = _paired_class_stacks([s for s, _d, _c, _t in parsed])

    # Footnote block height decides the bottom margin (reference layout:
    # hairline rule + 11px muted lines under the axis).
    fig_w, fig_h = 12.6, 7.6
    fig = plt.figure(figsize=(fig_w, fig_h))
    ax = fig.add_axes((AXES_LEFT, 0.30, AXES_WIDTH, 0.55))  # resized below

    sizes_union: List[int] = sorted({sz for _, d, _, _ in parsed
                                     for sz in d.sizes})
    total_pts = 0
    p99_pts = 0
    any_caps = False
    n_values: List[int] = []
    # (rank, handle, text, series index): sorted by rank (tier order) or,
    # under --legend-order series, by the index the caller listed it at.
    legend_entries: List[Tuple[int, object, str, int]] = []
    endpoint_labels: List[dict] = []
    n_fallback_pts = 0          # fixed100 points that ran below the target
    dns_notes: List[str] = []   # fixed100 did-not-sustain payloads, per series

    for i in draw_order:
        s, data, color, tier = parsed[i]
        spec = TIER_SPEC[tier]
        total_pts += len(data.sizes)
        n_values.extend(n for n in data.iterations if n > 0)

        # p50→p99 tail band over contiguous runs of citable p99 (a
        # suppressed p99 narrows the band to the line — never interpolated
        # across the gap, never fabricated).
        run: List[Tuple[int, float, float]] = []
        runs: List[List[Tuple[int, float, float]]] = []
        for sz, p50, p99 in zip(data.sizes, data.p50_us, data.p99_us):
            if p99 is None:
                if run:
                    runs.append(run)
                    run = []
                continue
            p99_pts += 1
            run.append((sz, p50, p99))
        if run:
            runs.append(run)
        for r in runs:
            if len(r) == 1:
                # An isolated citable p99 (suppressed neighbors) has no
                # band width to show — render a soft vertical tail stem so
                # the citable value never silently vanishes. Deliberately a
                # THICK low-alpha wash (a degenerate band), distinct from
                # the thin capped rep whiskers.
                sz, p50, p99 = r[0]
                ax.plot([sz, sz], [p50, p99], color=color,
                        linewidth=3.2, alpha=min(spec["band"] * 2.2, 0.35),
                        solid_capstyle="butt", zorder=spec["z"] - 1.5)
                continue
            bx, blo, bhi = zip(*r)
            ax.fill_between(bx, blo, bhi, color=color, alpha=spec["band"],
                            linewidth=0, zorder=spec["z"] - 1.5)

        # A single-point series has no line to find it by — its lone marker
        # gets a size bump so it stays discoverable (the surface ring keeps
        # overlapping marks distinct).
        ms = spec["ms"] * (1.45 if len(data.sizes) == 1 else 1.0)
        line = ax.plot(
            data.sizes, data.p50_us,
            color=color, linewidth=spec["lw"], linestyle=s.style,
            marker=MARKERS[i % len(MARKERS)], markersize=ms,
            markeredgecolor=SURFACE, markeredgewidth=1.3,
            solid_capstyle="round", solid_joinstyle="round",
            zorder=spec["z"],
        )[0]
        legend_entries.append((-tier_rank[tier], line,
                               _legend_label(s, paired_stacks=paired_stacks),
                               i))

        # Cross-rep p50 spread: crisp capped whiskers — a
        # different visual grammar from the soft tail wash, so the two
        # spreads coexist without fighting.
        caps = [(sz, p50, lo, hi) for sz, p50, lo, hi in
                zip(data.sizes, data.p50_us, data.band_lo_us, data.band_hi_us)
                if lo is not None and hi is not None]
        if caps:
            cx, cp, clo, chi = zip(*caps)
            yerr = ([max(p - lo, 0.0) for p, lo in zip(cp, clo)],
                    [max(hi - p, 0.0) for p, hi in zip(cp, chi)])
            ax.errorbar(cx, cp, yerr=yerr, fmt="none", ecolor=color,
                        elinewidth=1.1, capsize=2.6, capthick=1.1,
                        alpha=0.95, zorder=spec["z"] + 0.1)
            any_caps = True

        # fixed100 achieved-rate annotations: any point that ran BELOW
        # the uniform target —
        # the fallback ladder engaged, or the reps disagreed ('mixed')
        # — wears its achieved rate right at the marker ('@50Hz'). A
        # mixed-rate line is NEVER silent, and the annotation survives
        # --release (it is data integrity, not annotation garnish: an
        # unmarked fallback point would claim a 100 Hz measurement the
        # cell never made).
        for sz, p50, achieved in zip(data.sizes, data.p50_us,
                                     data.achieved_rate_hz):
            if achieved is None or achieved == bench.FIXED100_RATE_HZ:
                continue
            n_fallback_pts += 1
            tag = (f"@{achieved}Hz" if isinstance(achieved, int)
                   else f"@{achieved}")
            # The LAST point's annotation leans inboard (right-anchored)
            # so it never collides with that line's right-gutter endpoint
            # value label.
            last = sz == data.sizes[-1]
            # The tag sits on the side of its marker with the clearer sky:
            # a fixed "always above" placement put a tag on a NEIGHBORING
            # line when another series ran just above this point (two
            # rows of one platform a few us apart on a log axis), so by
            # eye the tag read as that line's. Clearance is the log
            # distance to the nearest other series' p50 at this size,
            # above and below; the tag goes to the roomier side and a
            # short leader ties it to its own marker, so the attachment
            # is drawn, not inferred.
            others = [d2.p50_us[d2.sizes.index(sz)]
                      for j, (_, d2, _, _) in enumerate(parsed)
                      if j != i and sz in d2.sizes]
            room_up = min((math.log10(o / p50) for o in others if o > p50),
                          default=math.inf)
            room_dn = min((math.log10(p50 / o) for o in others if o < p50),
                          default=math.inf)
            below = room_dn > room_up
            ink = _label_ink(color)
            ax.annotate(
                tag, xy=(sz, p50),
                xytext=(-2 if last else 0, -11 if below else 11),
                textcoords="offset points", fontsize=7.6,
                fontweight="semibold", color=ink,
                ha="right" if last else "center",
                va="top" if below else "bottom",
                zorder=6, annotation_clip=False,
                arrowprops=dict(arrowstyle="-", color=ink, linewidth=0.8,
                                shrinkA=0, shrinkB=3.5),
                path_effects=[path_effects.withStroke(linewidth=2.6,
                                                      foreground=SURFACE)])

        if data.did_not_sustain_sizes:
            dns_notes.append(
                f"{pretty_label(s.label)}: "
                f"{', '.join(format_size(sz) for sz in sorted(data.did_not_sustain_sizes))}")

        if _wants_endpoint_label(s, tier):
            # The label reports the p50 and centers on the LINE's endpoint
            # — never the band top, which the
            # old anchor rule rode and which could invert two labels'
            # vertical order relative to the values they report.
            endpoint_labels.append(dict(
                sz=data.sizes[-1], p50=data.p50_us[-1],
                text=_fmt_us(data.p50_us[-1]), ink=_label_ink(color)))

    # ---- axes chrome (reference look: horizontal hairline decade grid,
    # bottom rule only, muted ticks, no y spine)
    ax.set_xscale("log", base=2)
    ax.set_yscale("log")
    ax.yaxis.grid(True, which="major", color=GRID, linewidth=0.9, zorder=0)
    ax.xaxis.grid(False)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(AXIS)
    ax.set_xticks(sizes_union)
    ax.set_xticklabels([format_size(sz) for sz in sizes_union], fontsize=9.5)
    ax.xaxis.set_minor_locator(NullLocator())
    ax.yaxis.set_major_locator(LogLocator(base=10.0))
    ax.yaxis.set_minor_locator(NullLocator())
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: _fmt_us(v)))
    ax.tick_params(axis="x", length=4, width=1, labelsize=9.5)
    ax.tick_params(axis="y", length=0, labelsize=9.5)
    # Beneath-axis rate braces (see _draw_axis_braces): drawn from the
    # variant's pinned schedule; the
    # x-title drops below the brace row and the bottom margin widens.
    braces_drawn = (variant is not None
                    and _draw_axis_braces(ax, sizes_union, variant))
    ax.set_xlabel("payload size (log scale)", fontsize=10.5, color=INK2,
                  labelpad=52 if braces_drawn else 8)
    # Opt-in y-axis title (--ylabel). Off by default: the y ticks already
    # read in time units and the group titles name the quantity, but a
    # figure whose quantity is NOT the round trip (the rmw chart plots a
    # one-way leg) states it on the axis so the ticks cannot be misread.
    # Axis furniture: kept under --release.
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=10.5, color=INK2, labelpad=6)
    ax.set_xmargin(0.045)
    ax.set_ymargin(0.14)

    # ---- loan-lane callout: a figure that
    # carries a loan-take lane states IN-FIGURE what that lane is, with a
    # thin leader to the topmost loan line at the sweep's midpoint. Kept
    # under --release (see LOAN_CALLOUT) — it prevents a mis-reading.
    loan_data = [d for s, d, _c, _t in parsed if _is_loan_series(s)]
    if loan_data:
        # Anchor BY PAYLOAD SIZE, never by positional index: under
        # --skip-missing the loan lanes' `sizes` lists can differ (a lane
        # missing a middle row), so a shared index would point the leader
        # at a y that belongs to a different payload. The anchor size is
        # the first lane's sweep midpoint; the y is the topmost loan p50
        # AT that size over the lanes that carry it, falling back to the
        # largest size every loan lane shares.
        anchor_sz = loan_data[0].sizes[len(loan_data[0].sizes) // 2]
        by_size = [dict(zip(d.sizes, d.p50_us)) for d in loan_data]
        if not any(anchor_sz in m for m in by_size):
            common = set.intersection(*(set(m) for m in by_size))
            anchor_sz = max(common) if common else anchor_sz
        anchor_y = max(m[anchor_sz] for m in by_size if anchor_sz in m)
        ax.annotate(
            LOAN_CALLOUT, xy=(anchor_sz, anchor_y), xycoords="data",
            xytext=(0.025, 0.86), textcoords="axes fraction",
            fontsize=8.2, color=INK2, ha="left", va="top", linespacing=1.5,
            path_effects=[path_effects.withStroke(linewidth=2.6,
                                                  foreground=SURFACE)],
            arrowprops=dict(arrowstyle="-", color=MUTED, linewidth=0.9,
                            shrinkA=6, shrinkB=4,
                            connectionstyle="arc3,rad=0.16"),
            zorder=5, annotation_clip=False)

    # fixed100 did-not-sustain note: a payload whose fallback
    # ladder was exhausted has NO point on its line — the gap gets an
    # in-figure explanation naming the series + sizes, KEPT under
    # --release (a bare gap reads as "not measured"; the truth is "could
    # not sustain any ladder rate, so no latency was minted" — that is a
    # finding, and data integrity like the A2 band narrowing).
    if dns_notes:
        dns_text = ("did not sustain the fixed100 rate ladder "
                    "(no latency minted):\n" + "\n".join(dns_notes))
        ax.text(0.975, 0.03, dns_text, transform=ax.transAxes,
                fontsize=8.2, color=INK2, ha="right", va="bottom",
                linespacing=1.5, zorder=5,
                path_effects=[path_effects.withStroke(linewidth=2.6,
                                                      foreground=SURFACE)])

    if watermark and not release:  # release strips the annotation layer
        ax.text(0.5, 0.52, watermark, transform=ax.transAxes,
                fontsize=52, fontweight="bold", rotation=27,
                ha="center", va="center", color=INK, alpha=0.06, zorder=0)

    # Legend: always present for >=2 series (identity never rests on color
    # alone); headline first; frameless; ink text tokens. It lives in a
    # horizontal band BETWEEN the subtitle and the plot — never inside the
    # axes, where a rising line (the ROS 2 cells reach ms) would collide
    # with it. Its measured height sets the axes top below.
    # The subtitle never wraps (one fig.text line clips at the right
    # edge), so a long subtitle is passed with explicit newlines; every
    # extra line pushes the legend band, and the axes top below it, down
    # by one subtitle line height (10.5 pt on a 7.6 in figure).
    sub_text = subtitle or DEFAULT_SUBTITLE
    sub_drop = 0.024 * sub_text.count("\n")
    legend = None
    if len(parsed) >= 2:
        # Default: tier order (headline first, so the legend reads in the
        # same order the lines stack). --legend-order series keeps the
        # order the entries were given in, for a figure whose caption
        # lists its rows in a fixed order the legend must mirror.
        if legend_order == "series":
            legend_entries.sort(key=lambda e: e[3])
        else:
            legend_entries.sort(key=lambda e: e[0])
        n_leg = len(legend_entries)
        legend = fig.legend(
            [e[1] for e in legend_entries],
            [e[2] for e in legend_entries],
            loc="upper left", bbox_to_anchor=(0.045, 0.885 - sub_drop),
            frameon=False, fontsize=8.7, labelcolor=INK,
            handlelength=2.4, borderaxespad=0,
            # Cap at 2 columns: the ROS 2 cell labels run ~70 chars once
            # the loan-take lane text rides along (hero is 9 entries now),
            # and a third column pushes past the right edge of the canvas
            # — the legend's measured height already reflows the axes, so
            # growing DOWN is safe where growing RIGHT clips.
            ncols=n_leg if n_leg <= 3 else 2,
            columnspacing=1.6)

    # ---- footnote block (reference layout: hairline rule + muted lines)
    caption = "line = p50"
    if any_caps:
        caption += " (median of per-rep p50s)"
    if p99_pts:
        caption += " · shaded band = p50 to p99 tail spread"
    if any_caps:
        caption += " · whiskers = per-rep p50 min to max"
    if n_values:
        lo_n, hi_n = min(n_values), max(n_values)
        caption += (f" · samples/point (pooled): "
                    f"{lo_n if lo_n == hi_n else f'{lo_n} to {hi_n}'}")
    lines: List[str] = []
    # NOT inside the `not release` gate below, by the same rule that gate
    # states: release strips annotations, never data integrity. A group title
    # ENUMERATES its lines, so a render missing one is a figure whose title
    # is wrong about its own contents — and the release render is the one
    # that leaves the repo.
    if omitted:
        lines.append(
            "NOT IN THIS FIGURE (no CSV in the run dir): "
            + ", ".join(omitted)
            + "; the title names this group's full line set.")
    if not release:  # release strips ANNOTATIONS, never data integrity —
        # the suppressed-tail band narrowing above already happened per
        # point; only the explanatory text layer is dropped here. (The
        # fixed100 '@NHz' point annotations and the did-not-sustain note
        # are data integrity and stay in release renders.)
        lines.append(caption)
        suppressed = total_pts - p99_pts
        if suppressed:
            lines.append(
                f"p99 suppressed at {suppressed} of {total_pts} plotted "
                f"points (fewer than 20 pooled samples back that tail at "
                f"this n): the band narrows to the p50 line "
                f"there; no tail value is fabricated.")
        if n_fallback_pts:
            lines.append(
                f"{n_fallback_pts} point(s) ran below the fixed100 "
                f"{bench.FIXED100_RATE_HZ} Hz target (fallback ladder; "
                f"METHODOLOGY § 'The rate axis'); each is annotated "
                f"'@NHz' at its marker.")
        # Type-class footnote (METHODOLOGY § "The type-class axis"): a
        # figure carrying BOTH classes of one stack (a tokened row AND
        # its other-class twin — _paired_class_stacks) names both
        # classes + the matched quantity, so the pod/variable pairing is
        # readable without the docs. A lone tokened row earns no
        # footnote: the text claims a pairing the figure would not show.
        # The two classes are always separate series — never merged
        # into one line.
        if paired_stacks:
            lines.append(
                "type classes: fixed array (pod: Pod<N>/PodPayload, N "
                "total bytes) · sensor_msgs/Image (variable: N bytes in "
                "the unbounded data array, fixed fields as constant "
                "overhead); matched per sweep point, never merged into "
                "one series (METHODOLOGY § 'The type-class axis').")
    # Explicit --footnote lines are NOT annotation garnish: the author typed
    # them as claims the figure must carry (a rate the compared stack did not
    # sustain, what a lane's process shape does and does not isolate), so
    # they survive --release. Everything auto-generated above does not.
    lines.extend(footnotes)

    fig.text(0.045, 0.962, title, fontsize=15, fontweight="semibold", color=INK,
             va="top", ha="left")
    fig.text(0.045, 0.918, sub_text, fontsize=10.5,
             color=INK2, va="top", ha="left", linespacing=1.35)

    # Measure the legend band (needs a renderer) and hang the axes below
    # it; without a legend the plot claims the full height.
    axes_top = 0.868 - sub_drop
    if legend is not None:
        fig.canvas.draw()
        leg_bbox = legend.get_window_extent()
        leg_bottom_frac = leg_bbox.y0 / fig.bbox.height
        axes_top = min(axes_top, leg_bottom_frac - 0.035)
    line_h = 0.021
    # Footnote lines never wrapped (one fig.text line clips at the right
    # edge of the canvas), so an ordered line longer than the rule was
    # silently truncated in the PNG. Wrap each line greedily on word
    # boundaries at the rule's right edge, measured with the renderer at
    # the footnote font size, so the block height below counts the lines
    # that are actually drawn. Explicit newlines in a line are honoured
    # as breaks.
    rule_x0, rule_x1 = 0.045, 0.968
    if lines:
        renderer = fig.canvas.get_renderer()
        max_w = (rule_x1 - rule_x0) * fig.bbox.width

        def text_w(text: str) -> float:
            probe = fig.text(0, 0, text, fontsize=8)
            try:
                return probe.get_window_extent(renderer).width
            finally:
                probe.remove()

        wrapped: List[str] = []
        for raw in lines:
            for para in raw.split("\n"):
                words = para.split(" ")
                cur = ""
                for w in words:
                    cand = w if not cur else f"{cur} {w}"
                    if cur and text_w(cand) > max_w:
                        wrapped.append(cur)
                        cur = w
                    else:
                        cur = cand
                wrapped.append(cur)
        lines = wrapped
    # The brace row (rate braces + their labels + the dropped x-title)
    # needs extra bottom margin below the axis.
    brace_pad = 0.075 if braces_drawn else 0.0
    if lines:
        block_top = 0.016 + line_h * (len(lines) - 1)
        rule_y = block_top + 0.030
        axes_bottom = rule_y + 0.088 + brace_pad
    else:
        # No footnote block (--release): the axes claim the space, keeping
        # only the tick-label + x-title margin (+ the brace row when
        # drawn — braces are axis furniture, kept under --release).
        axes_bottom = 0.085 + brace_pad
    ax.set_position((AXES_LEFT, axes_bottom, AXES_WIDTH,
                     max(axes_top - axes_bottom, 0.25)))

    if lines:
        fig.add_artist(Line2D([rule_x0, rule_x1], [rule_y, rule_y],
                              transform=fig.transFigure, color=GRID,
                              linewidth=1))
        for i, text in enumerate(lines):
            fig.text(rule_x0, block_top - i * line_h, text, fontsize=8,
                     color=MUTED, va="bottom", ha="left")

    # ---- endpoint value labels (selective direct labels: the p50 at each
    # primary/secondary line's end + the CycloneDDS distro lines —
    # _wants_endpoint_label; ink contrast-gated). Each label sits in the
    # RIGHT GUTTER — left-anchored
    # LABEL_XPAD px past its line's last point, vertically centered on
    # that endpoint — so the number unambiguously belongs to its line.
    # Collision nudging is VERTICAL-ONLY within the gutter: labels keep
    # the ascending order of the values they report (numeric order ==
    # vertical order, the attribution property), and each x-overlapping
    # cluster is solved as one balanced column (_layout_label_column) so
    # near-coincident endpoints split the displacement between their
    # labels. The surface halo keeps text readable over anything a
    # nudged label still crosses.
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    inv = ax.transData.inverted()
    halo = [path_effects.withStroke(linewidth=3.0, foreground=SURFACE)]
    # The column solver is bounded by the AXES' rendered extent (plus a
    # small gutter pad), not the whole figure: a dense endpoint cluster
    # nudged to the figure edge would climb into the title/legend band or
    # the footnote block. Labels stay BESIDE the plot area.
    axbb = ax.get_window_extent(renderer)
    label_lo = axbb.y0 - LABEL_AXES_PAD
    label_hi = axbb.y1 + LABEL_AXES_PAD
    for item in endpoint_labels:
        item["artist"] = ax.text(
            item["sz"], item["p50"], item["text"], fontsize=9.5,
            fontweight="semibold", color=item["ink"], ha="left",
            va="bottom", zorder=6, clip_on=False, path_effects=halo)
        bb = item["artist"].get_window_extent(renderer)
        item["w"], item["h"] = bb.width, bb.height
        ex, ey = ax.transData.transform((item["sz"], item["p50"]))
        item["ax_x"], item["ax_y"] = ex + LABEL_XPAD, ey
    ordered = sorted(endpoint_labels, key=lambda d: (d["p50"], d["ax_y"]))
    # Transitive x-overlap grouping: only labels sharing gutter x-extent
    # can collide (a partial sweep can end a line at an earlier size);
    # each group is solved as ONE balanced column, in numeric order.
    columns: List[List[dict]] = []
    for item in ordered:
        bx0, bx1 = item["ax_x"], item["ax_x"] + item["w"]
        hits = [col for col in columns
                if any(o["ax_x"] < bx1 and o["ax_x"] + o["w"] > bx0
                       for o in col)]
        merged = [o for col in hits for o in col] + [item]
        for col in hits:
            columns.remove(col)
        merged.sort(key=lambda d: (d["p50"], d["ax_y"]))
        columns.append(merged)
    for col in columns:
        bottoms = _layout_label_column(
            [o["ax_y"] - o["h"] / 2.0 for o in col],  # centered preference
            [o["h"] for o in col], LABEL_GAP, label_lo, label_hi)
        for o, by in zip(col, bottoms):
            o["artist"].set_position(inv.transform((o["ax_x"], by)))

    # ---- save: PNG + SVG (the public artifact ships both; the SVG keeps
    # real text and the system-ui stack, like the reference)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    base = out_path.with_suffix("")
    png_path = base.with_suffix(".png")
    svg_path = base.with_suffix(".svg")
    fig.savefig(png_path, dpi=150)
    fig.savefig(svg_path)
    plt.close(fig)
    _rewrite_svg_font_stack(svg_path)
    print(f"wrote {png_path}")
    print(f"wrote {svg_path}")


# ---------------------------------------------------------------- main


def parse_series_arg(arg: str) -> Series:
    """Manual `LABEL=path[:style][@KEY]` overlay entry. The @KEY suffix (an
    explicit color key, read off the END of the value so a path may carry
    an @ of its own) makes the entry keyed: entries sharing a KEY share a
    color, assigned from the fallback slots in first-seen order, never by
    the entity rule. The label is everything before the LAST `=`: a legend
    entry may name an environment assignment verbatim (the rmw chart's
    `... with ROS_DISABLE_LOANED_MESSAGES=0 (...)` row), and the value
    (a results path plus its optional :style / @KEY tail) never carries
    one, whereas splitting at the first `=` silently truncated that label
    and then reported the series as a missing CSV."""
    label, sep, value = arg.rpartition("=")
    if not sep or not label:
        raise SystemExit(f"--series must be LABEL=path[:style][@KEY], got {arg!r}")
    key: Optional[str] = None
    head, at, tail = value.rpartition("@")
    if at and tail and "/" not in tail and ":" not in tail:
        value, key = head, tail
    style = "-"
    for name, ls in LINESTYLES.items():
        if value.endswith(":" + name):
            style = ls
            value = value[: -(len(name) + 1)]
            break
    if key is None:
        return Series(label, value, style, label)
    return Series(label, value, style, key, keyed=True)


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__.strip().split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("--results-dir", type=Path, required=True,
                   help="Run directory containing results_<raw_prefix>.csv files")
    p.add_argument("--out-dir", type=Path, default=None,
                   help="Output dir (default: <results-dir>/plots); every "
                        "figure is written as PNG + SVG")
    p.add_argument("--group", action="append",
                   help=f"Plot group (repeatable): "
                        f"{', '.join(ALL_GROUPS + HERO_GROUPS)}, all "
                        f"(default: all — which EXCLUDES the posture-keyed "
                        f"hero groups; render those explicitly against "
                        f"their posture run dirs)")
    p.add_argument("--chrt", choices=("0", "1", "both"), default="0",
                   help="Which chrt lines the groups expect (default: 0)")
    p.add_argument("--variant", choices=bench.PACING_VARIANTS, default=None,
                   help="Pacing variant the run dir was measured under — "
                        "decides which workspace legs are EXPECTED (split "
                        "x backtoback is structurally unmeasurable and is "
                        "never a missing CSV). Default: the run.json "
                        "manifest's variant, else quiescent; an explicit "
                        "value that contradicts the manifest is refused. "
                        "In --series mode an omitted --variant draws NO "
                        "rate braces (a custom overlay carries no "
                        "schedule); pass it explicitly to claim one.")
    p.add_argument("--skip-missing", action="store_true",
                   help="Warn loudly instead of failing when an expected CSV "
                        "is missing (partial runs)")
    p.add_argument("--series", action="append",
                   help="Manual overlay: LABEL=csv_path[:dashed|:dotted][@KEY]. "
                        "When given, groups are ignored and ONE custom plot "
                        "is produced. @KEY names the entry's color key "
                        "explicitly: entries sharing a KEY share a color "
                        "taken from the fallback slots in first-seen order "
                        "(blue, orange, violet, ...), bypassing the entity "
                        "rule, so one cell measured on several machines can "
                        "be colored per machine and shaped per run.")
    p.add_argument("--out-name", default="custom.png",
                   help="Output filename for --series mode (default: "
                        "custom.png; the SVG twin lands beside it)")
    p.add_argument("--title", default=None,
                   help="Figure title override (mainly for --series mode)")
    p.add_argument("--subtitle", default=None,
                   help=f"Subtitle line (default: {DEFAULT_SUBTITLE!r})")
    p.add_argument("--footnote", action="append", default=[],
                   help="Extra footnote line under the hairline rule: "
                        "machine, date, pacing, provenance (repeatable; a "
                        "long line wraps at the rule's right edge). Caveat "
                        "annotations belong here, not in the title. Kept "
                        "under --release.")
    p.add_argument("--ylabel", default=None,
                   help="Y-axis title (opt-in; none by default, the ticks "
                        "read in time units). Use it when the plotted "
                        "quantity is not the round trip, e.g. "
                        "'one-way latency'. Kept under --release.")
    p.add_argument("--legend-order", choices=("tier", "series"),
                   default="tier",
                   help="Legend entry order: 'tier' (default: primary, "
                        "then secondary, then tertiary, as the lines stack) "
                        "or 'series' (the order the --series entries were "
                        "given in, for a caption that lists the rows in a "
                        "fixed order).")
    p.add_argument("--watermark", default=None,
                   help="Diagonal in-plot watermark, e.g. 'SMOKE PREVIEW' — "
                        "the marker for non-citable renders")
    p.add_argument("--release", action="store_true",
                   help="Repo-embeddable release render: strip the AUTO "
                        "footnote lines (caption, suppression, fallback "
                        "ladder, type class, hero knob matrix) and the "
                        "watermark; explicit --footnote lines are kept. "
                        "Annotations only: data integrity (e.g. the A2 "
                        "suppressed-tail band narrowing) still applies. "
                        "Refuses --watermark.")
    args = p.parse_args(argv)

    if args.release and args.watermark:
        p.error("--release strips the watermark, and a watermark marks a "
                "NON-citable render — drop one of the two flags")

    if not args.results_dir.is_dir():
        print(f"error: {args.results_dir} is not a directory", file=sys.stderr)
        return 2
    out_dir = args.out_dir if args.out_dir is not None else args.results_dir / "plots"

    try:
        import matplotlib  # noqa: F401
    except ImportError:
        print("matplotlib is required: pip install matplotlib", file=sys.stderr)
        return 1

    if args.series:
        # Manual paths may be absolute or run-dir-relative (csv_path resolves).
        series = [parse_series_arg(a) for a in args.series]
        present = check_missing(series, args.results_dir,
                                args.skip_missing, "custom")
        check_row_completeness(present, args.results_dir,
                               args.skip_missing, "custom")
        # Custom overlays carry NO suite pacing metadata — their paths may
        # be absolute, from any run dir — so the beneath-axis rate braces
        # (derived from a variant's pinned schedule) must not be inferred
        # from the results dir's manifest: that would claim a pacing the
        # overlay never established. An explicit --variant still resolves
        # through the manifest gate (a contradiction is refused, as for
        # the groups); omitted = None = no braces drawn.
        plot_group(present, args.results_dir, out_dir / args.out_name,
                   args.title or "Round-trip latency (custom overlay)",
                   subtitle=args.subtitle, footnotes=args.footnote,
                   watermark=args.watermark, release=args.release,
                   variant=(resolve_variant(args.results_dir, args.variant)
                            if args.variant else None),
                   ylabel=args.ylabel, legend_order=args.legend_order)
        return 0

    groups = args.group or ["all"]
    if "all" in groups:
        explicit = [g for g in groups if g not in ("all", *ALL_GROUPS)]
        groups = list(ALL_GROUPS) + explicit
        print(f"note: 'all' excludes the posture-keyed hero groups "
              f"({', '.join(HERO_GROUPS)}) — render each explicitly "
              f"against its posture run dir (run.json dma_lock_posture "
              f"must match).", file=sys.stderr)

    variant = resolve_variant(args.results_dir, args.variant)
    for g in groups:
        series = group_series(g, args.chrt, variant)
        footnotes: List[str] = list(args.footnote)
        if g in HERO_GROUPS:
            # Label-accuracy gate: the hero's posture-naming title must be
            # backed by the run manifest (never demoted by --skip-missing).
            verify_hero_posture(args.results_dir, g[len("hero-"):], g)
            if args.chrt != "0":
                print(f"note: {g}: --chrt is ignored for hero groups — "
                      f"the posture decides the chrt suffixes",
                      file=sys.stderr)
            if not args.release:  # the knob matrix is AUTO annotation
                footnotes.extend(HERO_FOOTNOTES[g])
        present = check_missing(series, args.results_dir, args.skip_missing, g)
        # A group's title is a static string that ENUMERATES its lines
        # ("... Cerulion API, raw iceoryx2 floor, zenoh SHM comparison"), so
        # a render that dropped a line under --skip-missing would carry a
        # title naming a comparison that is not in the figure, with only a
        # stderr line — in a multi-hour log — to say so. The figure has to
        # carry its own caveat, the way a did_not_sustain payload already
        # does. Never demoted: --skip-missing decides whether the render
        # HAPPENS, not whether it states what it contains.
        omitted = [pretty_label(s.label) for s in series if s not in present]
        check_row_completeness(present, args.results_dir, args.skip_missing, g)
        if not present:
            print(f"warn: nothing to plot for group {g} — skipped",
                  file=sys.stderr)
            continue
        title = args.title or GROUP_TITLES.get(
            g, f"ROS 2 round-trip latency — {g[len('ros2-'):]} "
               f"(dockerized, per-cell containers)")
        plot_group(present, args.results_dir, out_dir / f"rtt_{g}.png",
                   title,
                   subtitle=args.subtitle, footnotes=footnotes,
                   omitted=omitted,
                   watermark=args.watermark, release=args.release,
                   variant=variant,
                   ylabel=args.ylabel, legend_order=args.legend_order)
    return 0


if __name__ == "__main__":
    sys.exit(main())
