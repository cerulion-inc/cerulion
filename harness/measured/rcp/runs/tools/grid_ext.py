# SPDX-License-Identifier: AGPL-3.0-only
"""Resumable benchmark grid: one model, one arm, ONE cell list for every arm. Run from ROOT on the machine that
runs the episodes and operators (a
model key never reaches it: an API model is reached through a model gateway, http://127.0.0.1:PORT/NAME, or
an open model served on that machine directly).

    venv/bin/python runs/tools/grid_ext.py cells [--set libero10|calib|pro_set_a|pro_set_b|day_one]
        [--oracle NAME=FILE ...] > CELLS.txt
    venv/bin/python runs/tools/grid_ext.py run --model NAME --base-url URL [--key-env NAME] --arm ARM
        --cells CELLS.txt [-j 8] [--label LABEL] [--results the design notes [--work /tmp/rx]
        [--packages all,place] [--op-args FLAGS] [--clock 7200] [--max-attempts 6] [--retry-wait 300] [--only ID,ID]
    venv/bin/python runs/tools/grid_ext.py status --results DIR --label LABEL --arm ARM --bench BENCH

Cells. `cells` prints a set of bench/ext/libero/cells.py (libero10 = all 10 tasks x inits 0 to 9, 100 cells). No
cell is ever excluded: --oracle only adds a trailing comment per line ("# oracle: plain 427, rcpx not solved,
rcpx_tools not run"). Both arms of a model run the SAME file; `run` refuses a list that holds "# excluded:" lines
(the old per-arm lists). ARM is without (surface native), without_sense (native_sense), without_calib
(native_calib, the declared control with the rcpx arm's sensing and calibration as text) or with (vnext); CELLS
lines are ID BENCH SUITE TASK INIT PERTURB [SURFACE [PACKAGES]], '#' starts a comment, a SURFACE that is not the
arm's is refused, the WITH arm's PACKAGES default to all,place. Every cell of the list is written to the group's
_status.json as pending before the queue starts (also the cells --only leaves out), so the report can count a
cell that is never scored as a failure (intention to treat).

Each episode: python -m bench.ext.episode_ext start --cell "ID ... SURFACE [PACKAGES]" --ep EPDIR
--budget-requests 80 --budget-steps 900; then the operator, bench.operators.chat_operator with --reply-budget 80
--one-call-per-reply --keep-images 4 in EVERY arm (80 model replies, one tool call per reply, the published
protocol); then bench.ext.score_ext (cost from token counts and the list prices of bench/ext/published.py) and
episode_ext stop. Refusals before anything is paid for: the grid exits 2 when chat_operator's --help lacks either
flag, and, on the WITH arm, when a requested rcpx package is absent or fails to import. After each start of a
vnext cell EPDIR/r2_problems.json must show every requested package loaded; if not, the cell is failed
(package_absent), the operator never runs and nothing is scored. --episode-cmd and --operator-cmd replace the
commands with a JSON list template ({python} {verb} {cell} {ep} {model} {base_url} {key_env} {root}
{budget_requests} {reply_budget} {budget_steps}) for dry grids with a scripted operator; they skip those checks.
Only socket-only operators may run on that machine.

Results: RESULTS/<bench>/<label>_<arm>/<id>.json, one per cell, written only for a SCORED episode; a cell with a
result file is skipped, so a killed grid resumes where it stopped. _status.json holds every cell's state and
attempts; _episodes/<id>/ keeps the logs, the scoring dir and the last frames. Never scored, retried later: an
operator that gave up on the endpoint (exit 4; reason rate_limit after an HTTP 429) and an episode that did not
start or was lost; after --max-attempts the cell is failed, still unscored. A wall-clock kill (--clock) and a
package refusal are failed at once; --retry-failed runs failed cells again. A rate limit holds every new start
for --retry-wait seconds. EGL: at most --egl-slots episodes render on the GPU at once ACROSS grid processes; the
rest get osmesa. Every process this launcher stops is its own child, stopped by pid. Run ONE grid process per
model and arm.
"""

import argparse
import fcntl
import json
import os
import shlex
import shutil
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ARM_SURFACE = {"without": "native", "without_sense": "native_sense", "without_calib": "native_calib",
               "with": "vnext"}
OP_REPLY_FLAG = "--reply-budget"
OP_ONE_CALL_FLAG = "--one-call-per-reply"
STATUS_NAME = "_status.json"
EPISODES_DIR = "_episodes"
MAX_SOCK = 103
EXIT_ENDPOINT, EXIT_NO_EPISODE, EXIT_LOST, EXIT_USAGE = 4, 3, 5, 64
VNEXT_PACKAGES = "all,place"   # the WITH arm: the round two packages plus pick, carry and place (rcpx/r2/place.py)
LOG_FILES = ("episode_end.json", "journal.jsonl", "operator.jsonl", "episode.json", "server.log", "r2_problems.json")


# ------------------------------------------------------------------------------------------------- cells
def parse_cells(text, arm, packages=None):
    """[(cell dict, problem or None)] from the cell list text, in file order."""
    out, seen = [], set()
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        f = line.split()
        if len(f) < 6:
            out.append(({"id": f"line{n}", "line": line}, f"line {n}: needs ID BENCH SUITE TASK INIT PERTURB"))
            continue
        cell = {"id": f[0], "bench": f[1], "suite": f[2], "task": f[3], "init": f[4], "perturb": f[5]}
        surface = ARM_SURFACE[arm]
        problem = None
        if len(f) >= 7 and f[6] != surface:
            problem = f"line {n}: surface {f[6]} is not arm {arm}'s ({surface})"
        pk = packages or (f[7] if len(f) >= 8 else None) or (VNEXT_PACKAGES if surface == "vnext" else None)
        cell["surface"] = surface
        cell["packages"] = pk if surface == "vnext" else None
        cell["line"] = " ".join(f[:6] + [surface] + ([pk] if surface == "vnext" else []))
        if cell["id"] in seen:
            problem = f"line {n}: cell id {cell['id']} appears twice"
        seen.add(cell["id"])
        out.append((cell, problem))
    return out


# ------------------------------------------------------------------------------------------------- decisions
def saw_rate_limit(operator_log):
    """True when the operator log has an HTTP 429 (or a rate-limit text) among its endpoint errors."""
    try:
        with open(operator_log, encoding="utf-8") as f:
            for line in f:
                if '"http_error"' not in line:
                    continue
                try:
                    rec = json.loads(line)
                except ValueError:
                    continue
                why = str(rec.get("why", "")).lower()
                if rec.get("status") == 429 or "rate limit" in why or "too many requests" in why:
                    return True
    except OSError:
        pass
    return False


def classify(start_rc, op_rc, clock_killed, rate_limited, result, refusal=None):
    """("scored", None) | ("retry", reason) | ("failed", reason) for one attempt (pure)."""
    if start_rc != 0:
        return "retry", "start_failed"
    if refusal:
        return "failed", refusal
    if clock_killed:   # not part of the protocol, and a retry would pay again for a slow model: --retry-failed
        return "failed", "clock"
    if op_rc == EXIT_USAGE:
        return "failed", "operator_usage"
    if op_rc == EXIT_ENDPOINT:
        return "retry", "rate_limit" if rate_limited else "endpoint"
    if op_rc in (EXIT_NO_EPISODE, EXIT_LOST):
        return "retry", "operator_lost"
    if result is None:
        return "retry", "score_failed"
    if result.get("truth_log") is None:
        return "retry", "no_truth_log"
    return "scored", None


def operator_flags_missing(help_text):
    """Required enforcement flags absent from operator help (pure)."""
    return [flag for flag in (OP_REPLY_FLAG, OP_ONE_CALL_FLAG) if flag not in help_text]


def packages_problem(doc):
    """Package preflight decision (pure); a failed probe cannot authorize a grid."""
    if not isinstance(doc, dict) or "absent" not in doc or "problems" not in doc:
        return "package check failed"
    if not isinstance(doc["absent"], list) or not isinstance(doc["problems"], (list, dict)):
        return "package check failed"
    if not all(isinstance(n, str) for n in list(doc["absent"]) + list(doc["problems"])):
        return "package check failed"
    names = sorted(set(doc["absent"]) | set(doc["problems"]))
    return "packages unavailable: " + ", ".join(names) if names else None


def r2_refusal(doc, requested):
    """Post-start package refusal, including requested extension packages (pure)."""
    if not isinstance(doc, dict):
        return "no r2_problems.json"
    names = set(doc.get("absent") or []) | set(doc.get("problems") or {})
    names.update(n.strip() for n in requested.split(",") if n.strip() not in
                 ("", "all", "none") and n.strip() not in (doc.get("loaded") or []))
    return "package_absent: " + ", ".join(sorted(names)) if names else None


PACKAGE_PROBE = """
import importlib, json, sys
from rcpx.r2 import core
names = [n.strip() for n in sys.argv[1].split(",") if n.strip()]
core_names = [n for n in names if n in core.PACKAGES or n in ("all", "none")]
core.load(",".join(core_names) if core_names else "none")
absent = list(core.load.absent); problems = dict(core.load.problems)
for n in names:
    if n in core.PACKAGES or n in ("all", "none"):
        continue
    try:
        importlib.import_module("rcpx.r2." + n)
    except ModuleNotFoundError:
        absent.append(n)
    except Exception as exc:
        problems[n] = str(exc)
print(json.dumps({"absent": absent, "problems": sorted(problems)}))
"""


def preflight(ns, cells):
    """Run isolated local checks before starting the default episode or operator."""
    python = ns.python or sys.executable
    if not ns.operator_cmd:
        if any(arg.split("=", 1)[0] == "--max-turns" for arg in shlex.split(ns.op_args or "")):
            return "--max-turns is obsolete; the grid enforces " + OP_REPLY_FLAG
        try:
            out = subprocess.run([python, "-m", "bench.operators.chat_operator", "--help"], cwd=ROOT,
                                 capture_output=True, text=True, timeout=120)
            missing = operator_flags_missing(out.stdout + out.stderr)
            if out.returncode or missing:
                return "operator cannot enforce required flags: " + ", ".join(missing or
                        [OP_REPLY_FLAG, OP_ONE_CALL_FLAG])
        except (OSError, subprocess.TimeoutExpired):
            return "operator help check failed: " + OP_REPLY_FLAG + ", " + OP_ONE_CALL_FLAG
    if not ns.episode_cmd and ns.arm == "with":
        for packages in sorted({c["packages"] for c in cells}):
            doc = None
            try:
                out = subprocess.run([python, "-c", PACKAGE_PROBE, packages], cwd=ROOT,
                                     capture_output=True, text=True, timeout=300)
                if out.returncode == 0:
                    doc = json.loads(out.stdout.strip().splitlines()[-1])
                why = (out.stderr.strip().splitlines() or [""])[-1][:300]
            except (OSError, ValueError, IndexError, subprocess.TimeoutExpired) as exc:
                why = f"{type(exc).__name__}: {exc}"[:300]
            problem = packages_problem(doc)
            if problem:
                return f"packages {packages}: {problem}" + (f" ({why})" if doc is None and why else "")
    return None


# ------------------------------------------------------------------------------------------------- EGL slots
class GlSlots:
    """At most n EGL renderers across every grid process: fcntl locks on DIR/slot<i>.lock; osmesa otherwise."""

    def __init__(self, n, directory):
        self.n, self.dir = int(n), directory
        os.makedirs(directory, exist_ok=True)

    def take(self):
        for i in range(self.n):
            f = open(os.path.join(self.dir, f"slot{i}.lock"), "a")
            try:
                fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return "egl", f
            except OSError:
                f.close()
        return "osmesa", None

    @staticmethod
    def give(handle):
        if handle is not None:
            fcntl.flock(handle, fcntl.LOCK_UN)
            handle.close()


# ------------------------------------------------------------------------------------------------- grid
class Grid:
    def __init__(self, ns, cells, listed_cells=None):
        self.listed_cells = cells if listed_cells is None else listed_cells
        self.ns = ns
        self.cells = cells
        self.label = ns.label or _safe(ns.model)
        self.group = os.path.join(os.path.abspath(ns.results), "{bench}", f"{self.label}_{ns.arm}")
        self.work = os.path.join(os.path.abspath(ns.work), f"{self.label}_{ns.arm}")
        self.lock = threading.Lock()
        self.status = {}
        self.cooldown_until = 0.0
        self.gl = GlSlots(ns.egl_slots, ns.egl_dir)
        self.python = ns.python or sys.executable
        self.counts = {"scored": 0, "skipped": 0, "retry": 0, "failed": 0}

    def group_dir(self, cell):
        return self.group.format(bench=cell["bench"])

    def result_path(self, cell):
        return os.path.join(self.group_dir(cell), f"{cell['id']}.json")

    # -- status file (one per group, rewritten whole under the lock) ---------------------------------------------
    def _status_path(self, cell):
        return os.path.join(self.group_dir(cell), STATUS_NAME)

    def note(self, cell, state, reason=None, **extra):
        with self.lock:
            path = self._status_path(cell)
            doc = self.status.get(path)
            if doc is None:
                doc = _read(path) or {}
                doc.update({"bench": cell["bench"], "label": self.label, "arm": self.ns.arm, "model": self.ns.model,
                            "base_url": self.ns.base_url})
                doc.setdefault("cells", {})
                self.status[path] = doc
            rec = doc["cells"].setdefault(cell["id"], {"attempts": 0, "reasons": []})
            rec["state"] = state
            if reason:
                rec["reasons"].append(reason)
            rec.update(extra)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            _write(path, doc)
            return rec

    def record_plan(self):
        """Record all parsed cells, preserving every existing cell record."""
        benches = sorted({c["bench"] for c in self.listed_cells})
        for bench in benches:
            listed = [c for c in self.listed_cells if c["bench"] == bench]
            path = self._status_path(listed[0])
            doc = _read(path) or {"bench": bench, "label": self.label, "arm": self.ns.arm,
                                  "model": self.ns.model, "base_url": self.ns.base_url}
            doc.update(cells_file=os.path.basename(self.ns.cells), listed=len(listed))
            records = doc.setdefault("cells", {})
            for cell in listed:
                records.setdefault(cell["id"], {"state": "pending", "attempts": 0, "reasons": []})
            self.status[path] = doc
            os.makedirs(os.path.dirname(path), exist_ok=True)
            _write(path, doc)

    # -- commands ------------------------------------------------------------------------------------------------
    def _fill(self, template, **kw):
        return [str(x).format(**kw) for x in template]

    def episode_argv(self, verb, cell, ep, gl=None):
        kw = self._kw(cell, ep, verb)
        if self.ns.episode_cmd:
            return self._fill(json.loads(self.ns.episode_cmd), **kw)
        argv = [self.python, "-m", "bench.ext.episode_ext", verb, "--ep", ep]
        if verb == "start":
            argv += ["--cell", cell["line"], "--budget-requests", str(self.ns.budget_requests),
                     "--budget-steps", str(self.ns.budget_steps)]
            if gl is not None:   # egl from this grid's slot = auto for the host: osmesa if the machine's slots are taken
                argv += ["--gl", "auto" if gl == "egl" else gl]
        return argv

    def operator_argv(self, cell, ep):
        kw = self._kw(cell, ep, "operate")
        if self.ns.operator_cmd:
            return self._fill(json.loads(self.ns.operator_cmd), **kw)
        argv = [self.python, "-m", "bench.operators.chat_operator", "--ep", ep, "--base-url", self.ns.base_url,
                "--model", self.ns.model]
        argv += shlex.split(self.ns.op_args or "")
        argv += [OP_REPLY_FLAG, str(self.ns.budget_requests), OP_ONE_CALL_FLAG, "--keep-images", "4"]
        if self.ns.key_env:
            argv += ["--api-key-env", self.ns.key_env]
        return argv

    def score_argv(self, ep, out):
        return [self.python, "-m", "bench.ext.score_ext", "--ep", ep, "--out", out]

    def _kw(self, cell, ep, verb):
        return {"python": self.python, "verb": verb, "cell": cell["line"], "ep": ep, "model": self.ns.model,
                "base_url": self.ns.base_url, "key_env": self.ns.key_env or "", "root": ROOT,
                "budget_requests": self.ns.budget_requests, "reply_budget": self.ns.budget_requests,
                "budget_steps": self.ns.budget_steps}

    def _run(self, argv, log, env, timeout=None):
        """(return code, killed) for a child process; on timeout the child (and only it) is stopped by pid."""
        with open(log, "a", encoding="utf-8") as out:
            out.write(f"$ {' '.join(shlex.quote(a) for a in argv)}\n")
            out.flush()
            child = subprocess.Popen(argv, cwd=ROOT, env=env, stdin=subprocess.DEVNULL, stdout=out,
                                     stderr=subprocess.STDOUT)
            try:
                return child.wait(timeout=timeout), False
            except subprocess.TimeoutExpired:
                child.terminate()
                try:
                    child.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
                return child.returncode, True

    # -- one attempt ---------------------------------------------------------------------------------------------
    def attempt(self, cell):
        """("scored" | "retry" | "failed", attempt number) for one attempt at the cell."""
        rec = self.note(cell, "running")
        n = rec["attempts"] + 1
        while os.path.exists(os.path.join(self.work, f"{cell['id']}_a{n}")):   # a crashed earlier run's dir
            n += 1
        rec["attempts"] = n - 1
        ep = os.path.join(self.work, f"{cell['id']}_a{n}")
        if len(os.fsencode(os.path.join(ep, "rcp.sock"))) > MAX_SOCK:
            self.note(cell, "failed", "episode dir too long for a unix socket: use a shorter --work")
            return "failed", n
        os.makedirs(self.work, exist_ok=True)
        log = os.path.join(self.work, f"{cell['id']}_a{n}.log")
        gl, handle = self.gl.take()
        env = dict(os.environ, MUJOCO_GL=gl, PYOPENGL_PLATFORM=gl, PYTHONDONTWRITEBYTECODE="1")
        t0 = time.time()
        op_rc, clock_killed, wall, result, refusal = None, False, None, None, None
        try:
            start_rc, _ = self._run(self.episode_argv("start", cell, ep, gl), log, env,
                                    timeout=self.ns.start_timeout)
            gl = (_read(os.path.join(os.path.dirname(ep), "_scoring", os.path.basename(ep), "episode.json"))
                  or {}).get("renderer") or gl
            if start_rc == 0 and cell["surface"] == "vnext":
                path = os.path.join(ep, "r2_problems.json")
                doc = _read(path)
                if doc is not None or os.path.exists(path) or not self.ns.episode_cmd:
                    refusal = r2_refusal(doc, cell["packages"] or "none")
            if start_rc == 0 and refusal is None:
                t_op = time.time()
                op_rc, clock_killed = self._run(self.operator_argv(cell, ep), log, env, timeout=self.ns.clock)
                wall = round(time.time() - t_op, 1)
                tmp = os.path.join(self.work, f"{cell['id']}_a{n}.result.json")
                rc, _ = self._run(self.score_argv(ep, tmp), log, env, timeout=300)
                result = _read(tmp) if rc == 0 else None
        finally:
            self._run(self.episode_argv("stop", cell, ep), log, env, timeout=120)
            GlSlots.give(handle)
        rate = saw_rate_limit(os.path.join(ep, "operator.jsonl"))
        state, reason = classify(start_rc, op_rc, clock_killed, rate, result, refusal)
        info = {"label": self.label, "model": self.ns.model, "base_url": self.ns.base_url, "arm": self.ns.arm,
                "attempt": n, "operator_exit": op_rc, "operator_wall_s": wall, "gl": gl,
                "attempt_wall_s": round(time.time() - t0, 1), "rate_limited_once": rate,
                "budget_requests": self.ns.budget_requests, "budget_steps": self.ns.budget_steps,
                "operator": "custom" if self.ns.operator_cmd else f"chat_operator {OP_REPLY_FLAG} "
                            f"{self.ns.budget_requests} {OP_ONE_CALL_FLAG}",
                "op_args": self.ns.op_args or "", "native_variant": os.environ.get("RCPX_NATIVE_VARIANT", "")}
        self._keep(cell, ep, n)
        if state == "scored":
            result.update(scored=True, grid=info)
            os.makedirs(self.group_dir(cell), exist_ok=True)
            _write(self.result_path(cell), result)
            self.note(cell, "scored", None, attempts=n, last=info)
            return "scored", n
        if state == "retry" and n >= self.ns.max_attempts:
            state, reason = "failed", f"{reason} (attempt {n} of {self.ns.max_attempts})"
        self.note(cell, "pending" if state == "retry" else "failed", reason, attempts=n, last=info)
        if reason and reason.startswith("rate_limit"):
            with self.lock:
                self.cooldown_until = max(self.cooldown_until, time.time() + self.ns.retry_wait)
        return state, n

    def _keep(self, cell, ep, n):
        """The attempt's logs, scoring dir and last frames under <group>/_episodes/<id>/a<n>/."""
        dest = os.path.join(self.group_dir(cell), EPISODES_DIR, cell["id"], f"a{n}")
        os.makedirs(dest, exist_ok=True)
        for name in LOG_FILES:
            src = os.path.join(ep, name)
            if os.path.isfile(src):
                shutil.copy2(src, dest)
        frames = os.path.join(ep, "frames")
        if os.path.isdir(frames) and self.ns.keep_frames > 0:
            os.makedirs(os.path.join(dest, "frames"), exist_ok=True)
            for name in sorted(os.listdir(frames))[-self.ns.keep_frames:]:
                shutil.copy2(os.path.join(frames, name), os.path.join(dest, "frames"))
        scoring = os.path.join(os.path.dirname(ep), "_scoring", os.path.basename(ep))
        if os.path.isdir(scoring):
            shutil.copytree(scoring, os.path.join(dest, "_scoring"), dirs_exist_ok=True,
                            ignore=lambda d, names: [x for x in names if x.endswith(".sock")])

    # -- the queue -----------------------------------------------------------------------------------------------
    def run(self):
        self.record_plan()
        queue = []
        for cell in self.cells:
            if os.path.exists(self.result_path(cell)):
                self.counts["skipped"] += 1
                continue
            rec = (_read(self._status_path(cell)) or {}).get("cells", {}).get(cell["id"], {})
            if rec.get("state") == "failed" and not self.ns.retry_failed:
                self.counts["failed"] += 1
                continue
            queue.append([cell, 0.0])
        qlock = threading.Lock()
        busy = [0]

        def worker():
            while True:
                with qlock:
                    now = time.time()
                    if not queue and busy[0] == 0:
                        return
                    ready = [q for q in queue if q[1] <= now and now >= self.cooldown_until]
                    item = ready[0] if ready else None
                    if item is not None:
                        queue.remove(item)
                        busy[0] += 1
                if item is None:
                    time.sleep(self.ns.poll)
                    continue
                try:
                    state, n = self.attempt(item[0])
                except Exception as exc:   # a launcher bug loses one attempt, never the grid
                    rec = self.note(item[0], "pending", f"grid_error: {type(exc).__name__}: {exc}"[:300])
                    state, n = ("retry", rec["attempts"]) if rec["attempts"] < self.ns.max_attempts else (
                        "failed", rec["attempts"])
                with qlock:
                    busy[0] -= 1
                    self.counts[state] += 1
                    if state == "retry":   # later, and later again after each failure: 1x, 2x, 4x ... 16x
                        queue.append([item[0], time.time() + self.ns.retry_wait * 2 ** min(4, max(0, n - 1))])

        with ThreadPoolExecutor(max_workers=max(1, self.ns.jobs)) as pool:
            for f in [pool.submit(worker) for _ in range(max(1, self.ns.jobs))]:
                f.result()
        return self.counts


# ------------------------------------------------------------------------------------------------- files
def _safe(text):
    return "".join(c if c.isalnum() or c in "-." else "-" for c in str(text)).strip("-")[:40] or "model"


def oracle_steps(path):
    """Cell id -> first step or None, from JSONL annotation rows."""
    out = {}
    with open(path, encoding="utf-8") as f:
        for text in f:
            if text.lstrip().startswith("{"):
                row = json.loads(text)
                if isinstance(row, dict) and "id" in row and "first_success_step" in row:
                    out[row["id"]] = row["first_success_step"]
    return out


def oracle_spec(spec):
    """A named path, or a bare path named by its stem."""
    if "=" in spec:
        return tuple(spec.split("=", 1))
    return os.path.splitext(os.path.basename(spec))[0], spec


def _read(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _write(path, doc):
    tmp = f"{path}.tmp{os.getpid()}.{threading.get_ident()}"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2, sort_keys=True, default=str)
    os.replace(tmp, path)


# ------------------------------------------------------------------------------------------------- main
def build_parser():
    p = argparse.ArgumentParser(prog="runs/tools/grid_ext.py")
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--model", required=True, help="the model name the endpoint serves")
    r.add_argument("--base-url", required=True, help="API root of an OpenAI-compatible endpoint")
    r.add_argument("--key-env", default=None, help="NAME of an env var holding a key (never the key itself)")
    r.add_argument("--arm", required=True, choices=sorted(ARM_SURFACE))
    r.add_argument("--cells", required=True)
    r.add_argument("--only", default=None, help="comma list of cell ids")
    r.add_argument("-j", "--jobs", type=int, default=8)
    r.add_argument("--label", default=None, help="short model name for the results folder (default from --model)")
    r.add_argument("--results", default=os.path.join(ROOT, "runs", "benchmarks"))
    r.add_argument("--work", default="/tmp/rx", help="episode dirs (short: the socket path must fit 103 bytes)")
    r.add_argument("--packages", default=None, help="vnext packages (default: the cell line's, else all)")
    r.add_argument("--op-args", default="", help="extra chat_operator flags, one shell-quoted string")
    r.add_argument("--budget-requests", type=int, default=80)
    r.add_argument("--budget-steps", type=int, default=900)
    r.add_argument("--clock", type=float, default=7200.0, help="wall seconds before an operator is stopped")
    r.add_argument("--start-timeout", type=float, default=300.0)
    r.add_argument("--max-attempts", type=int, default=6)
    r.add_argument("--retry-wait", type=float, default=300.0)
    r.add_argument("--retry-failed", action="store_true", help="also retry cells marked failed")
    r.add_argument("--egl-slots", type=int, default=8)
    r.add_argument("--egl-dir", default="/tmp/rcpx_egl_slots")
    r.add_argument("--keep-frames", type=int, default=8)
    r.add_argument("--poll", type=float, default=2.0)
    r.add_argument("--python", default=None, help="python for the episode and the operator (default this one)")
    r.add_argument("--episode-cmd", default=None, help="JSON list template replacing episode_ext (dry runs)")
    r.add_argument("--operator-cmd", default=None, help="JSON list template replacing chat_operator (dry runs)")
    c = sub.add_parser("cells", help="print a cell list from bench/ext/libero/cells.py")
    sys.path.insert(0, ROOT)
    from bench.ext.libero import cells as C
    c.add_argument("--set", default="libero10", choices=tuple(C.SETS))
    c.add_argument("--oracle", action="append", default=None, help="NAME=FILE or FILE JSONL annotation (repeatable)")
    s = sub.add_parser("status")
    s.add_argument("--results", default=os.path.join(ROOT, "runs", "benchmarks"))
    s.add_argument("--bench", required=True)
    s.add_argument("--label", required=True)
    s.add_argument("--arm", required=True, choices=sorted(ARM_SURFACE))
    return p


def main(argv=None):
    ns = build_parser().parse_args(argv)
    if ns.cmd == "cells":
        sys.path.insert(0, ROOT)
        from bench.ext.libero import cells as C
        oracles = [(name, oracle_steps(path)) for name, path in map(oracle_spec, ns.oracle or [])]
        for c in C.SETS[ns.set]():
            line = f"{c['id']} {c['bench']} {c['suite']} {c['task']} {c['init']} {c['perturb']}"
            if oracles:
                entries = [f"{name} " + (str(steps[c["id"]]) if steps.get(c["id"]) is not None else
                           "not solved" if c["id"] in steps else "not run") for name, steps in oracles]
                line += "   # oracle: " + ", ".join(entries)
            print(line)
        return 0
    if ns.cmd == "status":
        doc = _read(os.path.join(ns.results, ns.bench, f"{ns.label}_{ns.arm}", STATUS_NAME)) or {}
        states = {}
        for cid, rec in sorted(doc.get("cells", {}).items()):
            states[rec.get("state")] = states.get(rec.get("state"), 0) + 1
            print(f"{cid} {rec.get('state')} attempts={rec.get('attempts')} {'; '.join(rec.get('reasons', []))}")
        print(json.dumps(states, sort_keys=True))
        return 0
    with open(ns.cells, encoding="utf-8") as f:
        text = f.read()
    if any(line.lstrip().startswith("# excluded:") for line in text.splitlines()):
        sys.stderr.write(f"grid_ext: {ns.cells} holds '# excluded:' lines: it was made by oracle exclusion for one "
                         "arm. The grid runs one list for both arms and excludes no cell: make it with "
                         "`grid_ext.py cells --set NAME`.\n")
        return 2
    parsed = parse_cells(text, ns.arm, ns.packages)
    bad = [p for _, p in parsed if p]
    if bad:
        sys.stderr.write("".join(f"grid_ext: {p}\n" for p in bad))
        return 2
    cells = [c for c, _ in parsed]
    problem = preflight(ns, cells)
    if problem:
        sys.stderr.write(f"grid_ext: {problem}\n")
        return 2
    listed_cells = cells
    if ns.only:
        want = set(ns.only.split(","))
        cells = [c for c in cells if c["id"] in want]
    counts = Grid(ns, cells, listed_cells).run()
    print(json.dumps({"label": ns.label or _safe(ns.model), "arm": ns.arm, **counts}, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
