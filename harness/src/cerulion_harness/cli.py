# SPDX-License-Identifier: AGPL-3.0-only
"""Run one benchmark cell with and without the Cerulion harness, against your own model endpoint.

    python -m cerulion_harness run-cell --model MODEL [--base-url URL] [--key-env NAME] [--cell ID]
                                        [--arm both|plain|rcp] [--out DIR]
    python -m cerulion_harness cells

cerulion rcp bench runs run-cell. It starts the simulator, serves the robot to the model through one of two tool
sets, lets the model work within the budget, and scores the episode with the benchmark's own success check:

    plain   the published zero-shot interface of arXiv:2610.30051v1: three tools (move_to, done, give_up)
    rcp     the Cerulion harness tools

Both setups get the same start state, the same budget (80 model replies, 900 control steps), temperature 1.0 and
one tool call per reply. The model endpoint is any OpenAI-compatible chat completions URL that supports tools and
images. The URL is read from --base-url or OPENAI_BASE_URL; the key is read from the environment variable named by
--key-env (default OPENAI_API_KEY) and is sent only to that URL: urllib follows no redirect in run-cell or in the
episode's Python processes that start with run-cell's PYTHONPATH, its driver and model client among them
(redirects.py), and unless --no-endpoint-check is given, an endpoint that answers the test request with a
redirect is named before any episode starts.
"""
import argparse
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request

from cerulion_harness import layout, prices, redirects

DEFAULT_CELL = "pro_spatial_swap_t00_i00"
DEVELOPMENT_CELLS = [
    ("pro_spatial_swap_t00_i00", "pick the akita black bowl between the plate and the ramekin and place it on "
                                 "the plate (LIBERO-PRO, objects swapped, start state 0)"),
]
BUDGET = {"model_replies": 80, "control_steps": 900}
OP_ARGS = ("--context-tokens 200000 --max-tokens 32000 --http-timeout 600 --temperature 1.0 --retry-wait 20 "
           "--seed 1 --extra-body '{extra}'")
ARMS = {
    "plain": {"grid_arm": "without", "work": "p", "flags": []},
    "rcp": {"grid_arm": "with", "work": "r", "flags": ["--packages", "all,place"]},
}
THREADS = {"MUJOCO_GL": "osmesa", "PYOPENGL_PLATFORM": "osmesa", "PYTHONDONTWRITEBYTECODE": "1",
           "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "LP_NUM_THREADS": "2",
           "PYTHONPATH": "."}


def cell_line(cell_id):
    """The cell list line for a cell id: ID BENCH SUITE TASK INIT PERTURB."""
    m = re.fullmatch(r"pro_([a-z0-9]+)_(swap|task)_t(\d+)_i(\d+)", cell_id)
    if m:
        suite, perturb, task, init = m.group(1), m.group(2), int(m.group(3)), int(m.group(4))
        return f"{cell_id} libero_pro libero_{suite}_{perturb} {task} {init} {perturb}"
    m = re.fullmatch(r"l10_t(\d+)_i(\d+)", cell_id)
    if m:
        return f"{cell_id} libero libero_10 {int(m.group(1))} {int(m.group(2))} none"
    raise ValueError(f"unknown cell id {cell_id!r}: use pro_SUITE_swap_tNN_iNN, pro_SUITE_task_tNN_iNN or "
                     f"l10_tNN_iNN")


def endpoint_refusal(ns):
    """One small request before any simulator starts. Returns the endpoint's own refusal as text when it answers
    401, 402, 403 or 404 (a wrong key, an account that may not call it, a wrong URL or model name), or says so
    when the address cannot be reached at all (its name does not resolve, or nothing listens there), or names the
    address when it answers with a redirect, which is not followed: the key goes to the URL given and to no other.
    Else None: every other answer or failure is left to the episode, which retries for itself."""
    url = ns.base_url.rstrip("/")
    url = url if url.endswith("/chat/completions") else url + "/chat/completions"
    key = os.environ[ns.key_env]
    body = {"model": ns.model, "messages": [{"role": "user", "content": "Reply with the word ok."}]}
    req = urllib.request.Request(url, data=json.dumps(body).encode("utf-8"), method="POST",
                                 headers={"Content-Type": "application/json", "Authorization": "Bearer " + key})
    local = any(h in url.split("/")[2] for h in ("127.0.0.1", "localhost", "[::1]")) if "://" in url else False
    handlers = [redirects.Refuse()] + ([urllib.request.ProxyHandler({})] if local else [])
    opener = urllib.request.build_opener(*handlers)
    try:
        with opener.open(req, timeout=120) as reply:
            reply.read()
    except urllib.error.HTTPError as exc:
        if exc.code in (401, 402, 403, 404):
            text = exc.read(4000).decode("utf-8", "replace").replace(key, "[KEY]")
            return f"HTTP {exc.code}: {' '.join(text.split())[:600]}"
        if exc.code in redirects.REDIRECT_CODES:
            location = exc.headers.get("Location")
            where = urllib.parse.urljoin(url, location) if location else "no address"
            where = " ".join(where.replace(key, "[KEY]").split())[:300]
            return (f"HTTP {exc.code}: the endpoint redirects to {where}. A redirect is not followed, so the key "
                    f"goes only to the URL given: pass the address it redirects to as --base-url")
    except urllib.error.URLError as exc:
        if isinstance(exc.reason, (socket.gaierror, ConnectionRefusedError)):
            return f"the address could not be reached ({type(exc.reason).__name__}: {exc.reason})"
    except Exception:
        return None
    return None


def body_record_of(ns, out_dir):
    """Before the Cerulion harness setup's episode: the record of the robot body this cell needs. Noted earlier on
    this machine: nothing runs. Else python -m cerulion_harness.body_record checks it with one simulator start
    and, where none is stored, lets the robot measure itself now, once; its messages go to the terminal. Returns
    None, or the reason there is still no record."""
    from cerulion_harness import body_record
    if body_record.checked(ns.cell, layout.RECORDS):
        return None
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(p for p in (layout.SRC, os.environ.get("PYTHONPATH")) if p))
    proc = subprocess.run([sys.executable, "-m", "cerulion_harness.body_record", "--for-cell", ns.cell,
                           "--work", os.path.join(ns.work, "b")], stdin=subprocess.DEVNULL,
                          stdout=subprocess.PIPE, text=True, env=env)
    with open(os.path.join(out_dir, "body_record.json"), "w", encoding="utf-8") as f:
        f.write(proc.stdout)
    if proc.returncode == 0:
        return None
    try:
        return "; ".join(str(r.get("problem")) for r in json.loads(proc.stdout)["rows"] if r.get("problem"))
    except (ValueError, KeyError, TypeError):
        return f"python -m cerulion_harness.body_record ended with {proc.returncode}"


def group_dir(ns, arm, out_dir):
    """The folder the episode driver writes one arm's result of this cell into."""
    bench = cell_line(ns.cell).split()[1]
    return os.path.join(out_dir, arm, bench, f"{ns.label}_{ARMS[arm]['grid_arm']}")


def run_arm(ns, arm, out_dir, cells_file, tree):
    spec = ARMS[arm]
    env = layout.simulator_env(dict(os.environ, **THREADS))
    # First on the path: each Python process of the episode that starts with this PYTHONPATH (the driver, and the
    # model client it starts) runs episode_site/sitecustomize.py first, which makes urllib follow no redirect.
    env["PYTHONPATH"] = os.pathsep.join((redirects.SITE, env["PYTHONPATH"]))
    if arm == "plain":
        env["RCPX_NATIVE_VARIANT"] = "gripper=ramp"
    else:
        env["RCPX_SELFKNOW_DIR"] = layout.RECORDS
    results = os.path.join(out_dir, arm)
    argv = [sys.executable, "runs/tools/grid_ext.py", "run", "--model", ns.model, "--base-url", ns.base_url,
            "--key-env", ns.key_env, "--cells", cells_file, "--only", ns.cell, "-j", "1", "--label", ns.label,
            "--results", results, "--op-args", OP_ARGS.format(extra=ns.extra_body), "--clock", str(ns.clock),
            "--max-attempts", "3", "--retry-wait", "60", "--retry-failed", "--egl-slots", "0",
            "--budget-requests", str(BUDGET["model_replies"]), "--budget-steps", str(BUDGET["control_steps"]),
            "--arm", spec["grid_arm"], *spec["flags"], "--work", os.path.join(ns.work, spec["work"])]
    os.makedirs(results, exist_ok=True)
    t0 = time.time()
    with open(os.path.join(results, "driver.log"), "a", encoding="utf-8") as log:
        rc = subprocess.run(argv, cwd=tree, env=env, stdin=subprocess.DEVNULL, stdout=log,
                            stderr=subprocess.STDOUT).returncode
    return rc, round(time.time() - t0, 1)


def read_result(ns, arm, out_dir):
    """The scored row of one arm, or the reasons there is none."""
    group = group_dir(ns, arm, out_dir)
    row = {"arm": arm, "cell": ns.cell, "model": ns.model, "scored": False}
    try:
        with open(os.path.join(group, f"{ns.cell}.json"), encoding="utf-8") as f:
            res = json.load(f)
    except (OSError, ValueError):
        try:
            with open(os.path.join(group, "_status.json"), encoding="utf-8") as f:
                rec = json.load(f)["cells"].get(ns.cell, {})
            row["state"], row["reasons"] = rec.get("state"), rec.get("reasons")
        except (OSError, ValueError, KeyError, AttributeError):
            row["state"], row["reasons"] = "not started", ["see driver.log"]
        return row
    grid = res.get("grid") or {}
    attempt = grid.get("attempt") or 1
    log = os.path.join(group, "_episodes", ns.cell, f"a{attempt}", "operator.jsonl")
    key = prices.key_of(ns.model)
    row.update({
        "scored": True,
        "solved": res.get("success"),
        "first_success_step": res.get("first_success_step"),
        "solved_within_published_step_limit": res.get("success_within_ref"),
        "model_claimed_success": res.get("claimed_success"),
        "false_claim": res.get("false_claim"),
        "model_replies": res.get("model_requests"),
        "control_steps": res.get("steps_used"),
        "operator_seconds": grid.get("operator_wall_s"),
        "attempt": attempt,
        "end_reason": res.get("end_reason"),
        "list_price_usd": prices.cost_of(prices.usages_of(log), key) if key else None,
        "price_table": {"key": key, "date": prices.DATE},
        "result_file": os.path.join(group, f"{ns.cell}.json"),
    })
    return row


def show(row):
    if not row["scored"]:
        print(f"  {row['arm']:5s}  NOT SCORED: {row.get('state')}; {'; '.join(row.get('reasons') or [])}")
        return
    usd = row["list_price_usd"]
    cost = f"{usd:.2f} USD at list price ({row['price_table']['date']})" if usd is not None else \
        "no list price for this model name in the table"
    print(f"  {row['arm']:5s}  solved: {row['solved']}  within the published step limit: "
          f"{row['solved_within_published_step_limit']}  false claim: {row['false_claim']}")
    print(f"         replies: {row['model_replies']}  control steps: {row['control_steps']}  "
          f"operator seconds: {row['operator_seconds']}  cost: {cost}")


def refusal_before_start(ns, arms):
    """Every reason this run cannot start, checked before anything is written or sent: (exit code, text) or None."""
    if not ns.base_url:
        return 2, "no endpoint: pass --base-url or set OPENAI_BASE_URL"
    if not os.environ.get(ns.key_env):
        return 2, f"no key: set the environment variable {ns.key_env} (or name another with --key-env)"
    try:
        cell_line(ns.cell)
    except ValueError as exc:
        return 2, str(exc)
    try:
        if not isinstance(json.loads(ns.extra_body), dict):
            return 2, f"--extra-body must be a JSON object, not {ns.extra_body!r}"
    except ValueError as exc:
        return 2, f"--extra-body is not JSON ({exc})"
    for arm in arms:
        done = os.path.join(group_dir(ns, arm, os.path.abspath(ns.out)), f"{ns.cell}.json")
        if os.path.exists(done):
            return 2, (f"{ns.out} already holds a {arm} result of {ns.cell} ({done}), which the driver would report "
                       f"again instead of running a new episode: name a new --out folder")
    if not os.path.isfile(layout.SIM_PYTHON):
        return 2, f"the simulator is not installed in {layout.BX}: run {os.path.join(layout.HARNESS, 'install.sh')}"
    if os.path.realpath(sys.prefix) != os.path.realpath(os.path.dirname(os.path.dirname(layout.VENV_PYTHON))):
        return 2, (f"run-cell runs in the episode server environment that install.sh builds: "
                   f"{layout.VENV_PYTHON} -m cerulion_harness run-cell")
    if not os.path.isdir(layout.RCP_TREE):
        return 2, f"no measured code tree at {layout.RCP_TREE}"
    return None


def cmd_run_cell(ns):
    ns.base_url = ns.base_url or os.environ.get("OPENAI_BASE_URL")
    ns.out = ns.out or f"cerulion-bench-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}"
    arms = ["plain", "rcp"] if ns.arm == "both" else [ns.arm]
    refused = refusal_before_start(ns, arms)
    if refused:
        print(refused[1], file=sys.stderr)
        return refused[0]
    redirects.refuse_everywhere()   # this process sends the key too: it follows no redirect from here on
    trees = {"rcp": layout.RCP_TREE}
    if "plain" in arms:
        try:
            trees["plain"] = layout.plain_tree()
        except (OSError, ValueError) as exc:
            print(f"the plain setup's code tree could not be assembled: {exc}", file=sys.stderr)
            return 2
    if not ns.no_endpoint_check:
        refusal = endpoint_refusal(ns)
        if refusal:
            print(f"the model endpoint did not take a test request, so no episode was started.\n  {refusal}\n"
                  f"Check the URL ({ns.base_url}), the key in {ns.key_env} and the model name ({ns.model}).",
                  file=sys.stderr)
            return 2
    own_work = ns.work is None
    ns.work = ns.work or tempfile.mkdtemp(prefix="ch", dir="/tmp")   # short: unix socket paths live under it
    try:
        return run_cell(ns, arms, trees)
    finally:
        if own_work:
            shutil.rmtree(ns.work, ignore_errors=True)


def run_cell(ns, arms, trees):
    out_dir = os.path.abspath(ns.out)
    os.makedirs(out_dir, exist_ok=True)
    cells_file = os.path.join(out_dir, "cell.txt")
    with open(cells_file, "w", encoding="utf-8") as f:
        f.write(cell_line(ns.cell) + "\n")
    rows = []
    for arm in arms:
        if arm == "rcp":
            problem = body_record_of(ns, out_dir)
            if problem:
                print(f"the robot body of {ns.cell} has no record on this machine and measuring one failed, so "
                      f"the {arm} episode was not started (without a record every move is refused).\n  {problem}",
                      file=sys.stderr)
                return 2
        print(f"{time.strftime('%H:%M:%S')} running {arm} on {ns.cell} with {ns.model} ...", flush=True)
        rc, wall = run_arm(ns, arm, out_dir, cells_file, trees[arm])
        row = read_result(ns, arm, out_dir)
        row.update(driver_exit=rc, wall_seconds=wall)
        rows.append(row)
        show(row)
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"cell": ns.cell, "model": ns.model, "budget": BUDGET, "rows": rows}, f, indent=2, sort_keys=True)
    print(f"summary: {os.path.join(out_dir, 'summary.json')}")
    return 0 if all(r["scored"] for r in rows) else 1


def cmd_cells(ns):
    for cid, text in DEVELOPMENT_CELLS:
        print(f"{cid}\t{text}")
    return 0


def parser():
    p = argparse.ArgumentParser(prog="python -m cerulion_harness", description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run-cell", help="run one cell with the plain tools, the Cerulion harness tools, or both")
    r.add_argument("--model", required=True, help="the model name your endpoint serves")
    r.add_argument("--base-url", default=None, help="OpenAI-compatible API root (default: OPENAI_BASE_URL)")
    r.add_argument("--key-env", default="OPENAI_API_KEY",
                   help="NAME of the environment variable that holds your key (default OPENAI_API_KEY)")
    r.add_argument("--cell", default=DEFAULT_CELL, help=f"cell id (default {DEFAULT_CELL})")
    r.add_argument("--arm", default="both", choices=("both", "plain", "rcp"))
    r.add_argument("--out", default=None,
                   help="results folder, which must not hold a result of this cell yet (default: a new folder "
                        "cerulion-bench-UTCTIME in the current folder)")
    r.add_argument("--work", default=None, help="scratch folder; keep it short (unix socket paths). Default: a "
                                                "new folder under /tmp, removed at the end")
    r.add_argument("--label", default="run", help="short name for the results folders")
    r.add_argument("--extra-body", default='{"reasoning_effort": "high"}',
                   help="JSON object merged into every request body (default: high reasoning effort)")
    r.add_argument("--clock", type=float, default=5400.0, help="wall seconds before an episode is stopped")
    r.add_argument("--no-endpoint-check", action="store_true",
                   help="skip the one small test request that is sent to the endpoint before the simulator starts")
    r.set_defaults(fn=cmd_run_cell)
    sub.add_parser("cells", help="list the example cells").set_defaults(fn=cmd_cells)
    return p


def main(argv=None):
    ns = parser().parse_args(argv)
    return ns.fn(ns)
