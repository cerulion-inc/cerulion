# SPDX-License-Identifier: AGPL-3.0-only
"""Published numbers printed beside the study's own (the design notes section 3, with the corrections of
the design notes where the two differ), and the list prices the cost column uses. Standard library
only; nothing here is measured by this lane.

arXiv 2610.30051v1 (the protocol the WITHOUT arm follows), read 2026-11-11 at https://arxiv.org/html/2610.30051v1:
TABLE_1 holds all 18 rows of its Table 1 (model, effort, solved of 100, failure, limit reached, total cost, calls)
and PER_TASK its Tables 3 to 8 (solved of 10 inits per LIBERO-10 task, per model and effort). The paper prints
per-task counts, never per-init results: published_on_cells() recomputes the published rate on a set of cells by
taking each task at its published rate (exact only when all ten inits of every task present are in the set).
PRICES: vendor list prices per million tokens with URL and the day they were read (PRICE_DATE); a model without a
row has no cost, a price is never guessed.

Every row: section (where the report prints it: "libero_10" or "libero_pro"), kind, system, model (a canonical
model key, see canonical(), or None), effort, value_pct (percent), n (episodes behind it when the source states it,
else None), scope (what exactly was measured), comparable ("same_protocol" or "context"), date (ISO), url, note.

kind:
    trained_sota       the best trained policy on that benchmark
    trained            other trained-policy rows (shortcuts, collapses, aggregates)
    zero_shot_plain    a general model through a plain one-call-per-turn tool interface
    general_harness    a general model inside a richer harness or a coding-agent loop
comparable "same_protocol" is set ONLY for arXiv 2610.30051 on LIBERO-10 (the protocol the WITHOUT arm follows).
LIBERO-PRO has no same-protocol plain-tool comparator: every LIBERO-PRO general-model row
is context.

canonical(name) maps an endpoint model name or a short label to the study's model key, so the report can put a
published row of the SAME model beside a measured one. A different version is a different model (Fable 5.1 is not
Fable 5.2).
"""

import re

VLA_EVAL = "https://media.githubusercontent.com/media/allenai/vla-evaluation-harness/main/leaderboard/data/leaderboard.json"
AUDIT = "https://arxiv.org/html/2606.04233v1"
INSPECT = "https://arxiv.org/html/2610.30051v1"
PRO_README = "https://github.com/Zxy-MLlab/LIBERO-PRO"
ASPIRE = "https://arxiv.org/html/2607.00272v2"
K1 = "https://arxiv.org/html/2609.29389v1"
OPENRUA = "https://arxiv.org/html/2610.02459v1"

INSPECT_SCOPE = ("LIBERO-10, 10 tasks x inits 0 to 9 (100 episodes); move_to absolute end-effector targets, done, "
                 "give_up; 80 LLM calls, 900 control steps; one tool call per turn; the two newest observations "
                 "(at most 4 images)")
INSPECT_NOTE = "the paper reports no confidence intervals"
ASPIRE_SCOPE = ("LIBERO-PRO set A cell (Goal x position), 10 tasks x 50 held-out seeds; code as policy with "
                "iterative debugging from an empty skill library, not a single zero-shot pass")
K1_SCOPE = ("LIBERO-PRO set B subset (Spatial, Object, Goal x swap, task x tasks 0 to 2, init 0; 18 cells); "
            "roll/pitch/yaw targets relative to reset, 1,200 native steps, 400-call limit, effort not stated")


def _row(section, kind, system, value_pct, date, url, *, model=None, effort=None, n=None, scope="",
         comparable="context", note="", cost_usd=None):
    return {"section": section, "kind": kind, "system": system, "model": model, "effort": effort,
            "value_pct": float(value_pct), "n": n, "scope": scope, "comparable": comparable, "date": date,
            "url": url, "note": note, "cost_usd_per_episode": cost_usd,
            "k": None, "failure": None, "limit_reached": None, "calls": None}


def _inspect(system, model, effort, pct, cost=None):
    return _row("libero_10", "zero_shot_plain", system, pct, "2026-10-21", INSPECT, model=model, effort=effort, n=100,
                scope=INSPECT_SCOPE, comparable="same_protocol", note=INSPECT_NOTE, cost_usd=cost)


def _aspire(system, model, pct, cost=None):
    return _row("libero_pro", "general_harness", system, pct, "2026-10-20", ASPIRE, model=model, n=500,
                scope=ASPIRE_SCOPE, note="ASPIRE Table 18; seeds do not map to init indices 0 and 1",
                cost_usd=cost)


PER_TASK_TABLE = {"gpt-6-astra": 3, "gemini-4-argon": 4, "gpt-6.1-sol": 5, "claude-opus-5.5": 6,
                  "claude-opus-5": 7, "gpt-6-luna": 8}
PER_TASK = {
    "gpt-6-astra": {"low": [7, 8, 2, 1, 0, 0, 0, 8, 2, 0], "medium": [8, 10, 7, 2, 0, 1, 0, 8, 2, 0],
                    "high": [9, 10, 8, 2, 0, 0, 0, 9, 6, 0]},
    "gemini-4-argon": {"low": [10, 8, 6, 0, 0, 0, 0, 9, 1, 0], "medium": [10, 9, 4, 4, 0, 1, 0, 10, 0, 0],
                       "high": [10, 10, 6, 2, 0, 2, 0, 10, 2, 1]},
    "gpt-6.1-sol": {"low": [8, 7, 0, 1, 0, 2, 0, 6, 0, 0], "medium": [7, 5, 4, 0, 0, 4, 0, 8, 2, 0],
                   "high": [9, 9, 4, 0, 0, 2, 0, 5, 4, 0]},
    "claude-opus-5.5": {"low": [1, 2, 3, 0, 0, 0, 0, 3, 0, 0], "medium": [4, 7, 3, 1, 0, 0, 0, 3, 0, 0],
                        "high": [8, 9, 8, 0, 0, 0, 0, 8, 0, 0]},
    "claude-opus-5": {"low": [4, 5, 0, 0, 0, 0, 0, 0, 1, 0], "medium": [3, 4, 3, 0, 0, 0, 0, 2, 0, 0],
                      "high": [1, 2, 1, 0, 0, 0, 0, 1, 1, 0]},
    "gpt-6-luna": {"low": [0] * 10, "medium": [0] * 10, "high": [0] * 10},
}
TABLE_1 = [
    ("gpt-6-astra", "high", 44, 53, 3, 709, 4936), ("gemini-4-argon", "high", 43, 16, 41, 784, 5376),
    ("gpt-6-astra", "medium", 38, 62, 0, 532, 4773), ("gemini-4-argon", "medium", 38, 12, 50, 295, 4848),
    ("gemini-4-argon", "low", 34, 28, 38, 205, 5567), ("claude-opus-5.5", "high", 33, 57, 10, 180, 6358),
    ("gpt-6.1-sol", "high", 33, 65, 2, 164, 5376), ("gpt-6.1-sol", "medium", 30, 69, 1, 120, 5155),
    ("gpt-6-astra", "low", 28, 72, 0, 466, 4833), ("gpt-6.1-sol", "low", 24, 75, 1, 102, 5155),
    ("claude-opus-5.5", "medium", 18, 73, 9, 156, 6796), ("claude-opus-5", "medium", 12, 79, 9, 309, 6214),
    ("claude-opus-5", "low", 10, 89, 1, 207, 6152), ("claude-opus-5.5", "low", 9, 89, 2, 94.2, 5945),
    ("claude-opus-5", "high", 6, 88, 6, 567, 6442), ("gpt-6-luna", "low", 0, 100, 0, 0.294, 852),
    ("gpt-6-luna", "medium", 0, 79, 21, 18.4, 6745), ("gpt-6-luna", "high", 0, 57, 43, 28.7, 7401),
]
SYSTEMS = {"gpt-6-astra": "GPT-6 Astra", "gemini-4-argon": "Gemini-4 Argon", "gpt-6.1-sol": "GPT-6.1 Sol",
           "claude-opus-5.5": "Claude Opus 5.5", "claude-opus-5": "Claude Opus 5", "gpt-6-luna": "GPT-6 Luna"}


def _table_row(data):
    model, effort, k, failure, limit, cost, calls = data
    row = _inspect(SYSTEMS[model], model, effort, k, round(cost / 100, 5))
    return dict(row, k=k, failure=failure, limit_reached=limit, calls=calls)


def per_task(model, effort="high"):
    """Published solved-of-ten counts by task, or None when no table exists."""
    return PER_TASK.get(model, {}).get(effort)


def published_on_cells(model, effort, cells):
    """Published task rates on valid cells; exact only for whole ten-init tasks."""
    counts = per_task(model, effort)
    if counts is None:
        return None
    tasks, n, k, outside = {}, 0, 0.0, 0
    for task, init in cells:
        if not (0 <= task <= 9 and 0 <= init <= 9):
            outside += 1
            continue
        tasks.setdefault(task, set()).add(init)
        n += 1
        k += counts[task] / 10
    exact = bool(tasks) and all(inits == set(range(10)) for inits in tasks.values())
    if exact:
        k = float(sum(counts[t] for t in tasks))
    k = round(k, 10)
    return {"n": n, "k": k, "pct": 100 * k / n if n else None, "exact": exact,
            "tasks": sorted(tasks), "out_of_range": outside}


PRICE_DATE = "2026-11-11"
OPENAI_PRICE = "https://platform.openai.com/docs/pricing"
VERTEX_PRICE = "https://cloud.google.com/vertex-ai/generative-ai/pricing"
CLAUDE_PRICE = "https://platform.claude.com/docs/en/about-claude/pricing"
OPENAI_NOTE = ("OpenAI standard tier, short context. The grid reaches the model through an Azure deployment; "
               "Azure's price page prints no number in its static text, so the Azure rate is not confirmed. "
               "The long-context tier is not entered (its threshold is not in the page's static text).")
CLAUDE_NOTE = ("Anthropic first-party list price; the grid reaches Claude through a gateway whose billed rate "
               "is not known here.")


def _price(inp, out, cached, url, note, over=None, long_in=None, long_out=None, long_cached=None):
    return {"in_per_mtok": inp, "out_per_mtok": out, "cached_in_per_mtok": cached,
            "long_prompt_over": over, "long_in_per_mtok": long_in, "long_out_per_mtok": long_out,
            "long_cached_in_per_mtok": long_cached, "url": url, "date": PRICE_DATE, "note": note}


PRICES = {
    "gpt-6-astra": _price(10.00, 50.00, 1.00, OPENAI_PRICE, OPENAI_NOTE),
    "gpt-6-luna": _price(0.10, 0.50, 0.01, OPENAI_PRICE, OPENAI_NOTE),
    "gemini-4-argon": _price(2.00, 10.00, 0.10, VERTEX_PRICE,
        "Vertex global, the price through 2026-11-30 (the page calls it promotional: 50 percent credits back); "
        "4.00 / 20.00 / 0.20 from 2026-12-01."),
    "gemini-3.8-flash": _price(0.75, 3.75, 0.075, VERTEX_PRICE,
        "Vertex global, introductory price through 2026-12-31; 1.50 / 7.50 / 0.15 from 2027-01-01."),
    "claude-fable-5.2": _price(8.00, 40.00, 0.20, CLAUDE_PRICE, CLAUDE_NOTE),
    "claude-sonnet-5.5": _price(2.00, 10.00, 0.10, CLAUDE_PRICE, CLAUDE_NOTE),
    "claude-haiku-5.5": _price(0.10, 0.50, 0.01, CLAUDE_PRICE, CLAUDE_NOTE +
        " A request whose prompt is over 100,000 tokens pays the long prices, priced request by request.",
        100000, 0.50, 2.50, 0.05),
    "qwen3.8-27b": None,
}


def price(model_key):
    """Dated vendor list-price row, or None; never infer a price."""
    return PRICES.get(model_key)


ROWS = [
    # ---- LIBERO-10: trained
    _row("libero_10", "trained_sota", "LaST-R1", 99.8, "2026-10-20", VLA_EVAL, scope="libero_10 (LIBERO-Long)",
         note="vla-eval data last_updated 2026-10-20; replaces the 98.8 of the June audit (skeptic correction 3)"),
    _row("libero_10", "trained", "best reported LIBERO-Long, June audit", 98.8, "2026-06-02", AUDIT,
         scope="LIBERO-Long", note="superseded by LaST-R1 99.8 in the October data"),
    _row("libero_10", "trained", "0.09B DINO+MLP probe", 92.4, "2026-06-02", AUDIT, scope="LIBERO-Long",
         note="trained shortcut with no language encoder"),
    # ---- LIBERO-10: zero-shot, the protocol the WITHOUT arm follows (arXiv 2610.30051)
    *[_table_row(r) for r in TABLE_1],
    # ---- LIBERO-10: context (other harnesses or other task mixes)
    _row("libero_10", "general_harness", "Kinex bare action API, GPT-6 Astra high via Codex", 85.0, "2026-11-11",
         "https://stacx-lab.github.io/kinex/", model="gpt-6-astra", effort="high",
         scope="40 LIBERO tasks, coding-agent loop over the native API",
         note="coding agent, not comparable to a one-call-per-turn loop; date is the day the page was read"),
    _row("libero_10", "general_harness", "LIBERO-Agent, GPT-6 Astra", 45.0, "2026-09-30",
         "https://arxiv.org/html/2609.39507v1", model="gpt-6-astra",
         scope="MCP with native OSC delta tools, a different 30-task mix", note="not LIBERO-10 alone"),
    _row("libero_10", "general_harness", "LIBERO-Agent, Claude Opus 5", 19.3, "2026-09-30",
         "https://arxiv.org/html/2609.39507v1", model="claude-opus-5",
         scope="MCP with native OSC delta tools, a different 30-task mix", note="not LIBERO-10 alone"),
    _row("libero_10", "general_harness", "LIBERO-Agent, Claude Fable 5.1", 15.8, "2026-09-30",
         "https://arxiv.org/html/2609.39507v1", model="claude-fable-5.1",
         scope="MCP with native OSC delta tools, a different 30-task mix", note="not LIBERO-10 alone"),
    _row("libero_10", "general_harness", "OpenETA, GPT-5.6 Luna, 44 tools", 1.0, "2026-08-04",
         "https://arxiv.org/html/2608.03924v1", model="gpt-5.6-luna", scope="LIBERO-Long",
         note="14.0 over four suites; 3 Long episodes lack complete records"),
    _row("libero_10", "general_harness", "ORA + DeepSeek-V4.1-Flash", 63.0, "2026-10-15",
         "https://haoailab.com/blogs/ora/", model="deepseek-v4.1-flash", scope="LIBERO-Long",
         note="20-tool harness with grasp planning; 83.5 over 400 standard cases"),
    # ---- LIBERO-PRO: trained (README table, position column; task column 0.00 to 0.01)
    _row("libero_pro", "trained_sota", "pi0.5, position column, LIBERO-PRO Goal", 38.0, "2025-11-05", PRO_README,
         scope="LIBERO-PRO Goal x position", note="README latest entry; task column 0.00 to 0.01"),
    _row("libero_pro", "trained", "pi0.5, position column, LIBERO-PRO Spatial", 20.0, "2025-11-05", PRO_README,
         scope="LIBERO-PRO Spatial x position"),
    _row("libero_pro", "trained", "pi0.5, position column, LIBERO-PRO Object", 17.0, "2025-11-05", PRO_README,
         scope="LIBERO-PRO Object x position"),
    _row("libero_pro", "trained", "pi0.5, position column, LIBERO-PRO LIBERO-10", 8.0, "2025-11-05", PRO_README,
         scope="LIBERO-PRO LIBERO-10 x position"),
    _row("libero_pro", "trained", "OpenVLA and pi0, position and task columns", 0.0, "2025-11-05", PRO_README,
         scope="LIBERO-PRO position and task columns"),
    _row("libero_pro", "trained", "QuoVLA, aggregated", 69.8, "2026-10-20", VLA_EVAL, scope="mixed protocols"),
    _row("libero_pro", "trained", "pi0.5, aggregated", 62.0, "2026-10-20", VLA_EVAL, scope="mixed protocols"),
    # ---- LIBERO-PRO: general models, context only
    _aspire("Claude Haiku 5.5", "claude-haiku-5.5", 68.0, 3.97),
    _aspire("Claude Sonnet 5.5", "claude-sonnet-5.5", 98.8, 12.11),
    _aspire("Claude Fable 5.1", "claude-fable-5.1", 96.6, 42.24),
    _aspire("Claude Opus 5.5", "claude-opus-5.5", 99.2, 18.29),
    _aspire("GPT-6 Luna", "gpt-6-luna", 14.8, 0.83),
    _aspire("GPT-6 Sol", "gpt-6-sol", 59.2, 7.64),
    _aspire("GPT-6 Astra", "gpt-6-astra", 94.4, 35.37),
    _aspire("Kimi K3", "kimi-k3", 78.4),
    _aspire("Qwen 3.8 Max", "qwen3.8-max", 56.0),
    _row("libero_pro", "zero_shot_plain", "GPT-6 Astra, RGB-only Inspect Robots harness", 61.1, "2026-09-24", K1,
         model="gpt-6-astra", n=18, scope=K1_SCOPE, note="11 of 18; not the WITHOUT arm's protocol"),
    _row("libero_pro", "general_harness", "GPT-6 Astra + K1 perception tools", 88.9, "2026-09-24", K1,
         model="gpt-6-astra", n=18, scope=K1_SCOPE,
         note="16 of 18; K1 says the paired gain does not reach the 0.05 threshold"),
    _row("libero_pro", "general_harness", "Gemini 3.7 Flash + K1", 77.8, "2026-09-24", K1,
         model="gemini-3.7-flash", n=18, scope=K1_SCOPE, note="14 of 18; 139 of 180 (77.2) on the full set"),
    _row("libero_pro", "general_harness", "OpenRUA, Claude Code + Opus 5, raw ROS 2 terminal", 87.0, "2026-10-01",
         OPENRUA, model="claude-opus-5", scope="8 cells: P and T columns of the four suites, 10 trials per task",
         note="coding-agent loop"),
    _row("libero_pro", "general_harness", "OpenRUA scaling, Claude Fable 5.1", 75.0, "2026-10-01", OPENRUA,
         model="claude-fable-5.1", n=200, scope="LIBERO-PRO LIBERO-10 perturbation cells, 200 trials"),
    _row("libero_pro", "general_harness", "OpenRUA scaling, Claude Opus 5", 72.5, "2026-10-01", OPENRUA,
         model="claude-opus-5", n=200, scope="LIBERO-PRO LIBERO-10 perturbation cells, 200 trials"),
    _row("libero_pro", "general_harness", "OpenRUA scaling, GPT-6 Astra high", 69.0, "2026-10-01", OPENRUA,
         model="gpt-6-astra", effort="high", n=200, scope="LIBERO-PRO LIBERO-10 perturbation cells, 200 trials"),
    _row("libero_pro", "general_harness", "OpenRUA scaling, Claude Sonnet 5", 24.0, "2026-10-01", OPENRUA,
         model="claude-sonnet-5", n=200, scope="LIBERO-PRO LIBERO-10 perturbation cells, 200 trials"),
    _row("libero_pro", "general_harness", "CaP-Agent0", 18.0, "2026-11-11", "https://capgym.github.io/",
         scope="30 tasks with position and instruction perturbations, code as policy",
         note="cited as 18, 18.2 and 23.2 elsewhere; venue not confirmed on the page; date is the day it was read"),
    _row("libero_pro", "general_harness", "AMP1 + Gemini 4 Argon", 95.2, "2026-11-06",
         "https://arxiv.org/html/2611.09892v1", model="gemini-4-argon", scope="CaP-Gym's LIBERO-PRO subset"),
]

# What each arm's operator receives: the report's access table (bench/ext/report_ext.py prints it first in every
# report). DECLARED facts of the surfaces as built on 2026-11-11 (bench/ext/libero/native.py, native_calib.py,
# backend.py; the owners' lines in the design notes); the hand speed row is filled at report time from
# the measured speed files. Change a row when a surface changes.
ARMS = ("without", "without_sense", "without_calib", "with")
_PLAIN_PROMPT = ("the paper's system prompt and tool descriptions byte for byte (arXiv 2610.30051v1 Figures 2 and 3, "
                 "CC BY 4.0); the first message holds the task sentence, the budget lines and the first observation "
                 "with both start pictures")
_PLAIN_TOOLS = ("exactly the paper's three tools: move_to(targets, note), done(summary, hindsight), "
                "give_up(reason, hindsight)")
_SENSE_STATE = "the 12 values plus 7 joint positions and 7 joint velocities"
_SENSE_FORCE = "7 joint torques, wrist force and torque (tared at reset)"
_NO_HEIGHT = "not given (the move_to bounds state the lowest allowed hand height)"
_NO_MODEL = "none (one sentence: a single Panda arm)"


def _all(text):
    return {arm: text for arm in ARMS}


ACCESS = [
    ("Prompt and first message", {
        "without": _PLAIN_PROMPT,
        "without_sense": "the same as WITHOUT, plus one declared paragraph on the added sensing",
        "without_calib": "the same as WITHOUT, plus declared paragraphs on the added sensing and the calibration",
        "with": "the lane's operator brief as the system text and rcpx's own tool descriptions; the first message "
                "holds the task sentence, rcpx's practices and conventions text and the budget lines; no picture "
                "until the model looks"}),
    ("Pictures", {
        "without": "agentview and wrist camera, 256 x 256 px, with every observation",
        "without_sense": "the same two cameras at 512 x 512 px, with every observation",
        "without_calib": "the same two cameras at 512 x 512 px, with every observation",
        "with": "the same two cameras at 256 x 256 px, on request (the look tool); a drawn picture of the arm"}),
    ("State values", {
        "without": "12 values with every observation: hand position x, y, z (m, world), 6 rotation values, the "
                   "gripper command, 2 finger positions",
        "without_sense": _SENSE_STATE, "without_calib": _SENSE_STATE,
        "with": "joint angles and speeds, the tool tip pose in the robot's base frame, finger positions and targets"}),
    ("Joint torques and wrist force", {
        "without": "none", "without_sense": _SENSE_FORCE, "without_calib": _SENSE_FORCE,
        "with": "joint torques and their share of the limits, wrist force and torque (tared by the arm model), "
                "rcpx's contact force estimate"}),
    ("Camera calibration", {
        "without": "none", "without_sense": "none",
        "without_calib": "given as text: each camera's size, intrinsics and pose (the rcpx arm's own camera "
                         "document, restated in the plain arm's frames)",
        "with": "the fixed camera's fx, fy, cx, cy and its pose in the base frame; the wrist camera's intrinsics "
                "and mount"}),
    ("Support height", {
        "without": _NO_HEIGHT, "without_sense": _NO_HEIGHT,
        "without_calib": "given as text: the height of the support surface in the world frame",
        "with": "given: the height of the support surface in the base frame (an installation fact)"}),
    ("Robot model", {
        "without": _NO_MODEL, "without_sense": _NO_MODEL,
        "without_calib": "the kinematic facts as text (joint limits, axes and anchors, link lengths, the tool's "
                         "shape, reach); not the model file and none of rcpx's computations",
        "with": "the arm's kinematic and dynamic model, derived from the robot's own description file (reach "
                "checks, inverse kinematics, the force estimate)"}),
    ("Hand speed, measured (mm per control step)", _all("")),
    ("Controller", {
        "without": "the benchmark's own OSC_POSE controller as shipped; a straight line interpolated at the "
                   "published per-step caps (0.01 m per axis per control step)",
        "without_sense": "the same as WITHOUT", "without_calib": "the same as WITHOUT",
        "with": "the env's joint position controller at kp 150, damping ratio 1, a 0.2 rad goal step (this "
                "study's choice), driven closed loop by rcpx"}),
    ("Tools", {
        "without": _PLAIN_TOOLS, "without_sense": "the same as WITHOUT", "without_calib": "the same as WITHOUT",
        "with": "the rcpx tool surface (packages all,place; the open model runs all,place,lean; the packages of "
                "each arm row are printed in the table below); the harness adds read_resource and claim"}),
    ("Budgets", _all("80 model replies and 900 control steps per episode, one tool call per reply")),
    ("Never given", _all("object poses, object names beyond the task sentence, simulator contact names, the success "
                         "signal")),
]

SECTIONS = ("libero_10", "libero_pro")
KINDS = ("trained_sota", "trained", "zero_shot_plain", "general_harness")

# Our grid's models (RESEARCH.md section 4): canonical key -> the words that identify it in an endpoint name.
GRID_MODELS = {
    "gpt-6-astra": ("astra",),
    "gpt-6-luna": ("luna",),
    "claude-fable-5.2": ("fable-5-2", "fable-5.2", "fable52"),
    "claude-sonnet-5.5": ("sonnet-5-5", "sonnet-5.5", "sonnet55"),
    "claude-haiku-5.5": ("haiku-5-5", "haiku-5.5", "haiku55"),
    "gemini-4-argon": ("argon",),
    "gemini-3.8-flash": ("gemini-3-8-flash", "3-8-flash", "gemini-3.8-flash"),
    "qwen3.8-27b": ("qwen3.8-27b", "qwen3-8-27b", "qwen38-27b"),
}


def canonical(name):
    """The study's model key for an endpoint model name or a label (case, dots, dashes and underscores ignored),
    or None when it is none of the grid's eight models. A locally served open model is named "operator" (Qwen3.8-27B-FP8)."""
    if not isinstance(name, str) or not name.strip():
        return None
    text = re.sub(r"[\s_./]+", "-", name.strip().lower())
    if text in ("operator", "open", "open-27b"):
        return "qwen3.8-27b"
    for key, words in GRID_MODELS.items():
        if any(re.sub(r"[\s_./]+", "-", w) in text for w in words):
            return key
    return None


def rows(section=None, kind=None, model=None, comparable=None):
    """The rows that match every given field."""
    out = []
    for r in ROWS:
        if section is not None and r["section"] != section:
            continue
        if kind is not None and r["kind"] != kind:
            continue
        if model is not None and r["model"] != model:
            continue
        if comparable is not None and r["comparable"] != comparable:
            continue
        out.append(r)
    return out


def same_protocol(section, model, effort="high"):
    """The published same-protocol row for this model and effort, or None (none exists for LIBERO-PRO)."""
    hits = [r for r in rows(section=section, model=model, comparable="same_protocol") if r["effort"] == effort]
    return hits[0] if hits else None


def trained_sota(section):
    hits = rows(section=section, kind="trained_sota")
    return hits[0] if hits else None


def label(r):
    """One line for a report: system, effort, value, n, date and url."""
    eff = f" ({r['effort']})" if r.get("effort") else ""
    n = f", n {r['n']}" if r.get("n") else ""
    return f"{r['system']}{eff}: {r['value_pct']:.1f} percent{n}; {r['scope']}; {r['date']}; {r['url']}"


__all__ = ["GRID_MODELS", "KINDS", "ROWS", "SECTIONS", "canonical", "label", "rows", "same_protocol",
           "trained_sota", "PER_TASK_TABLE", "PER_TASK", "TABLE_1", "per_task", "published_on_cells",
           "PRICE_DATE", "PRICES", "price", "ACCESS", "ARMS"]
