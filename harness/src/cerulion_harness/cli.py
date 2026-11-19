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
redirect is named before any episode starts. The episode reaches the URL through a relay on the loopback address
inside this process, which sends the model's reasoning blocks back with the tool calls they came with, as the
measured runs of Gemini models did with the signature Google's endpoint returns; the operator's context estimate
does not count those blocks (route.py, which also says where the routes still differ). The relay follows no
redirect either and passes no Location header back to the episode. The episode's own files name the relay's
address (http://127.0.0.1:PORT/v1); summary.json names the URL (without a user name, password or query), and the
relay's address each setup used.

What it prints: a start line with the route, the object every request body adds and what one episode took in the
measured runs; one line per model reply (UTC time, setup, reply number, control step, tokens, USD so far); each
setup's verdict; the path of summary.json, which records the same. Paths are printed as given. Scratch files go
to a new folder under /tmp (short, because the simulator's socket paths live there); that folder is removed at
the end, after an error and after SIGINT, SIGTERM or SIGHUP, which also stop the episode within 40 s of the signal
(STOP_WINDOW_S). A signal that run-cell starts with ignored stays ignored: under nohup, which ignores SIGHUP, a
closed terminal does not stop the run.
"""
import argparse
import contextlib
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

from cerulion_harness import expected, layout, prices, progress, redirects, route

DEFAULT_CELL = "pro_spatial_swap_t00_i00"
DEVELOPMENT_CELLS = [
    ("pro_spatial_swap_t00_i00", "pick the akita black bowl between the plate and the ramekin and place it on "
                                 "the plate (LIBERO-PRO, objects swapped, start state 0)"),
]
BUDGET = {"model_replies": 80, "control_steps": 900}
OP_ARGS = ("--context-tokens 200000 --max-tokens 32000 --http-timeout {timeout} --temperature 1.0 --retry-wait 20 "
           "--seed 1 --extra-body '{extra}'")
ARMS = {
    "plain": {"grid_arm": "without", "work": "p", "flags": []},
    "rcp": {"grid_arm": "with", "work": "r", "flags": ["--packages", "all,place"]},
}
THREADS = {"MUJOCO_GL": "osmesa", "PYOPENGL_PLATFORM": "osmesa", "PYTHONDONTWRITEBYTECODE": "1",
           "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "LP_NUM_THREADS": "2",
           "PYTHONPATH": "."}
MAX_ATTEMPTS = 3
# The longest unix socket path macOS binds (104 bytes with the terminating zero); Linux takes 107.
SOCKET_PATH_MAX = 103
SCRATCH_PARENT, SCRATCH_PREFIX = "/tmp", "cerulion-bench-"
# After an interrupt: the driver's process group gets SIGTERM and, DRIVER_GRACE_S seconds later, SIGKILL; then
# the episode stop commands run. The body record's process group gets SIGINT and, STOP_WINDOW_S seconds later,
# SIGKILL. Every stop ends within STOP_WINDOW_S of the interrupt, and the scratch folder is removed after it:
# cerulion rcp bench ends run-cell 60 s after it forwards an interrupt.
DRIVER_GRACE_S = 10.0
STOP_WINDOW_S = 40.0
# Why an interrupt's stop may have left part of the episode running (ns.stop_problem; None when every stop ended).
STOP_TIMED_OUT, STOP_REPORTED_RUNNING, STOP_NOT_STARTED = "timed out", "reported running", "not started"
_PRINT = threading.Lock()


def say(text, err=False):
    """One line to the terminal; a terminal that is gone (SIGHUP) costs nothing else."""
    with _PRINT:
        try:
            print(text, file=sys.stderr if err else sys.stdout, flush=True)
        except (OSError, ValueError):
            pass


class Interrupted(BaseException):
    """SIGINT, SIGTERM or SIGHUP arrived: the run stops its episode, removes the scratch folder it made, and ends."""

    def __init__(self, signum):
        super().__init__(signum)
        self.signum = signum


class Signals:
    """While entered (in the main thread), the first SIGINT, SIGTERM or SIGHUP raises Interrupted; after that, or
    after hold(), every one of them is ignored, so the clean-up runs to its end. One of them that is ignored on
    entry (nohup ignores SIGHUP; a non-interactive shell starts a background job with SIGINT ignored) is left
    ignored. Inside deferred(), the first one is noted and raised when the block ends."""
    CAUGHT = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)

    def __init__(self):
        self.seen, self.held, self._old, self._deferring = None, False, {}, False

    def _handle(self, signum, frame):
        if self.held or self.seen is not None:
            return
        self.seen = signum
        if not self._deferring:
            raise Interrupted(signum)

    def hold(self):
        self.held = True

    @contextlib.contextmanager
    def deferred(self):
        """For `proc = subprocess.Popen(...)`: a signal that arrives while the child starts (a signal handler runs
        between Python instructions, so inside Popen, or before its result is assigned) is raised as Interrupted
        once the block has ended, when proc holds the child the caller's clean-up stops."""
        self._deferring = True
        try:
            yield
        finally:
            self._deferring = False
            if self.seen is not None and not self.held:
                raise Interrupted(self.seen)

    def __enter__(self):
        if threading.current_thread() is threading.main_thread():
            for sig in self.CAUGHT:
                if signal.getsignal(sig) is not signal.SIG_IGN:
                    self._old[sig] = signal.signal(sig, self._handle)
        return self

    def __exit__(self, *exc):
        for sig, old in self._old.items():
            signal.signal(sig, old if old is not None else signal.SIG_DFL)
        self._old = {}
        return False


def stop_group(proc, first, grace_s):
    """End a child that leads its own process group, and what is left in the group: first, then SIGKILL when the
    child has not ended after grace_s seconds. A child already reaped is left alone. Returns False when it took
    SIGKILL, else True."""
    if proc.returncode is not None:
        return True
    for sig, wait_s in ((first, grace_s), (signal.SIGKILL, None)):
        try:
            os.killpg(proc.pid, sig)
        except (ProcessLookupError, PermissionError):
            pass
        try:
            proc.wait(timeout=wait_s)
            return sig != signal.SIGKILL
        except subprocess.TimeoutExpired:
            continue
    return False


def _do_nothing(signum, frame):
    pass


def popen_with_default_sigint(argv, **kw):
    """subprocess.Popen(argv, **kw), with SIGINT at its default in the child even when this process ignores it, so
    a Python child turns SIGINT into KeyboardInterrupt and runs its own clean-up. A signal ignored here stays
    ignored in the child, while one with a handler here starts at its default there: so, called in the main thread
    (the one thread that may set a handler), it gives an ignored SIGINT a handler that does nothing while the child
    starts. Called in another thread, it starts the child as subprocess.Popen does."""
    if (signal.getsignal(signal.SIGINT) is not signal.SIG_IGN
            or threading.current_thread() is not threading.main_thread()):
        return subprocess.Popen(argv, **kw)
    signal.signal(signal.SIGINT, _do_nothing)
    try:
        return subprocess.Popen(argv, **kw)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)


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


def op_args(extra_text):
    """The operator flags of every episode: the measured runs' flags, with this route's --extra-body."""
    return OP_ARGS.format(timeout=route.HTTP_TIMEOUT_S, extra=extra_text)


def longest_socket_path(work, label, cell):
    """Bytes of the longest unix socket path an episode binds under the scratch folder work: the simulator's truth
    socket, in the scoring folder of the last attempt of the plain setup."""
    return max(len(os.fsencode(os.path.join(work, spec["work"], f"{label}_{spec['grid_arm']}", "_scoring",
                                            f"{cell}_a{MAX_ATTEMPTS}", "truth.sock"))) for spec in ARMS.values())


def scratch_for(tmp=SCRATCH_PARENT):
    """A new scratch folder under tmp, named cerulion-bench- and eight characters. The episode's unix socket paths
    live under it: under /tmp, with the default label and cell, the longest is 90 bytes (SOCKET_PATH_MAX is 103)."""
    return tempfile.mkdtemp(prefix=SCRATCH_PREFIX, dir=tmp)


def endpoint_refusal(ns):
    """One small request before any simulator starts, with the object every request body adds (--extra-body).
    Returns the endpoint's own refusal as text when it answers 401, 402, 403 or 404 (a wrong key, an account that
    may not call it, a wrong URL or model name), or says so when the address cannot be reached at all (its name
    does not resolve, or nothing listens there), or names the address when it answers with a redirect, which is
    not followed: the key goes to the URL given and to no other. Else None: every other answer or failure is left
    to the episode, which retries for itself."""
    url = ns.base_url.rstrip("/")
    url = url if url.endswith("/chat/completions") else url + "/chat/completions"
    key = os.environ[ns.key_env]
    body = {"model": ns.model, "messages": [{"role": "user", "content": "Reply with the word ok."}]}
    # Merged in as the operator merges it into every request body: a key whose value is null is dropped.
    for name, value in (json.loads(ns.extra_body) if ns.extra_body is not None else {}).items():
        if value is None:
            body.pop(name, None)
        else:
            body[name] = value
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


def body_record_of(ns, out_dir, signals):
    """Before the Cerulion harness setup's episode: the record of the robot body this cell needs. Noted earlier on
    this machine: nothing runs. Else python -m cerulion_harness.body_record checks it with one simulator start
    and, where none is stored, lets the robot measure itself now, once; its messages go to the terminal. Returns
    None, or the reason there is still no record."""
    from cerulion_harness import body_record
    if body_record.checked(ns.cell, layout.RECORDS):
        return None
    # It imports the measured code into its own process: PYTHONDONTWRITEBYTECODE keeps it from writing __pycache__
    # folders into the harness folder.
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1",
               PYTHONPATH=os.pathsep.join(p for p in (layout.SRC, os.environ.get("PYTHONPATH")) if p))
    path = os.path.join(out_dir, "body_record.json")
    with open(path, "w", encoding="utf-8") as f:
        proc = None
        try:
            with signals.deferred():
                proc = popen_with_default_sigint([sys.executable, "-m", "cerulion_harness.body_record",
                                                  "--for-cell", ns.cell, "--work", os.path.join(ns.work, "b")],
                                                 stdin=subprocess.DEVNULL, stdout=f, env=env,
                                                 start_new_session=True)
            rc = proc.wait()
        except BaseException:
            signals.hold()
            # Its own clean-up stops its simulator; one that takes SIGKILL may leave the simulator running.
            if proc is not None and not stop_group(proc, signal.SIGINT, STOP_WINDOW_S):
                ns.stop_problem = STOP_TIMED_OUT
            raise
    if rc == 0:
        return None
    try:
        with open(path, encoding="utf-8") as f:
            return "; ".join(str(r.get("problem")) for r in json.load(f)["rows"] if r.get("problem"))
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return f"python -m cerulion_harness.body_record ended with {rc}"


def group_dir(ns, arm, out_dir):
    """The folder the episode driver writes one arm's result of this cell into."""
    bench = cell_line(ns.cell).split()[1]
    return os.path.join(out_dir, arm, bench, f"{ns.label}_{ARMS[arm]['grid_arm']}")


def scratch_group(ns, arm):
    """The driver's folder for one arm in the scratch folder: one folder CELL_aN per attempt, _scoring beside."""
    spec = ARMS[arm]
    return os.path.join(ns.work, spec["work"], f"{ns.label}_{spec['grid_arm']}")


def episode_dirs(ns, arm):
    pattern = re.compile(re.escape(ns.cell) + r"_a\d+")
    group = scratch_group(ns, arm)
    try:
        names = sorted(os.listdir(group))
    except OSError:
        return []
    return [os.path.join(group, n) for n in names if pattern.fullmatch(n) and os.path.isdir(os.path.join(group, n))]


def stop_episodes(ns, arm, tree, env, log, until):
    """After the driver was stopped: the episode server and the simulator run in sessions of their own, so each
    attempt's episode is stopped with the command the driver stops it with, each until the time.monotonic() value
    until at the latest. Returns None when every stop command ended in time and said it left nothing running (exit
    0). Else, when the time ran out: STOP_TIMED_OUT. Else the first of STOP_REPORTED_RUNNING, a stop command that
    exited non-zero (the real one exits 1 when a part it stops is still running), and STOP_NOT_STARTED, one that
    could not be started."""
    problem = None
    for ep in episode_dirs(ns, arm):
        left = until - time.monotonic()
        if left <= 0:
            return STOP_TIMED_OUT
        try:
            done = subprocess.run([sys.executable, "-m", "bench.ext.episode_ext", "stop", "--ep", ep], cwd=tree,
                                  env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                  timeout=left)
        except subprocess.TimeoutExpired:
            return STOP_TIMED_OUT
        except (OSError, subprocess.SubprocessError):
            problem = problem or STOP_NOT_STARTED
            continue
        if done.returncode != 0:
            problem = problem or STOP_REPORTED_RUNNING
    return problem


def run_arm(ns, arm, out_dir, cells_file, tree, base_url, signals):
    """(driver exit code, wall seconds) of one arm's episode. The driver leads a process group of its own: an
    interrupt while it runs ends that group and stops the episode, and then goes on to the caller."""
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
    argv = [sys.executable, "runs/tools/grid_ext.py", "run", "--model", ns.model, "--base-url", base_url,
            "--key-env", ns.key_env, "--cells", cells_file, "--only", ns.cell, "-j", "1", "--label", ns.label,
            "--results", results, "--op-args", op_args(ns.extra_body), "--clock", str(ns.clock),
            "--max-attempts", str(MAX_ATTEMPTS), "--retry-wait", "60", "--retry-failed", "--egl-slots", "0",
            "--budget-requests", str(BUDGET["model_replies"]), "--budget-steps", str(BUDGET["control_steps"]),
            "--arm", spec["grid_arm"], *spec["flags"], "--work", os.path.join(ns.work, spec["work"])]
    os.makedirs(results, exist_ok=True)
    t0 = time.time()
    with open(os.path.join(results, "driver.log"), "a", encoding="utf-8") as log:
        proc = None
        try:
            with signals.deferred():
                proc = subprocess.Popen(argv, cwd=tree, env=env, stdin=subprocess.DEVNULL, stdout=log,
                                        stderr=subprocess.STDOUT, start_new_session=True)
            rc = proc.wait()
        except BaseException:
            signals.hold()
            if proc is not None:
                until = time.monotonic() + STOP_WINDOW_S
                stop_group(proc, signal.SIGTERM, DRIVER_GRACE_S)
                log.flush()
                ns.stop_problem = stop_episodes(ns, arm, tree, env, log, until)
            raise
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
    usages = prices.usages_of(os.path.join(group, "_episodes", ns.cell, f"a{attempt}", "operator.jsonl"))
    own = [x for x in map(progress.reported_cost, usages) if x is not None]
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
        "list_price_usd": prices.cost_of(usages, key) if key else None,
        "price_table": {"key": key, "date": prices.DATE},
        "reported_usd": round(sum(own), 6) if own else None,
        "reported_cost_replies": len(own),
        "usage_replies": len(usages),
        "result_file": os.path.join(group, f"{ns.cell}.json"),
    })
    return row


def cost_text(row):
    """Both prices of a scored row, each saying which price it is."""
    usd = row["list_price_usd"]
    text = (f"{usd:.4f} USD at list price (the package's price table of {row['price_table']['date']})"
            if usd is not None else "no list price for this model name in the package's price table")
    if row.get("reported_usd") is not None:
        text += (f"; {row['reported_usd']:.4f} USD reported by the endpoint (usage.cost of "
                 f"{row['reported_cost_replies']} of {row['usage_replies']} replies)")
    return text


def show(row):
    if not row["scored"]:
        say(f"  {row['arm']:5s}  NOT SCORED: {row.get('state')}; {'; '.join(row.get('reasons') or [])}")
        return
    say(f"  {row['arm']:5s}  solved: {row['solved']}  within the published step limit: "
        f"{row['solved_within_published_step_limit']}  false claim: {row['false_claim']}")
    say(f"         replies: {row['model_replies']}  control steps: {row['control_steps']}  "
        f"operator seconds: {row['operator_seconds']}")
    say(f"         cost: {cost_text(row)}")


def refusal_before_start(ns, arms):
    """Every reason this run cannot start, checked before anything is written or sent: (exit code, text) or None."""
    if not ns.base_url:
        return 2, "no endpoint: pass --base-url or set OPENAI_BASE_URL"
    try:
        scheme = urllib.parse.urlparse(ns.base_url).scheme
    except ValueError:
        scheme = ""
    if scheme not in ("http", "https") or not route.host_of(ns.base_url):
        return 2, f"the base URL must be http or https with a host, not {ns.base_url!r}"
    if not os.environ.get(ns.key_env):
        return 2, f"no key: set the environment variable {ns.key_env} (or name another with --key-env)"
    try:
        cell_line(ns.cell)
    except ValueError as exc:
        return 2, str(exc)
    extra = ns.extra_body if ns.extra_body is not None else route.extra_body_for(ns.base_url, ns.model)
    try:
        if not isinstance(json.loads(extra), dict):
            return 2, f"--extra-body must be a JSON object, not {extra!r}"
    except ValueError as exc:
        return 2, f"--extra-body is not JSON ({exc})"
    for arm in arms:
        done = os.path.join(group_dir(ns, arm, ns.out), f"{ns.cell}.json")
        if os.path.exists(done):
            return 2, (f"{ns.out} already holds a {arm} result of {ns.cell} ({done}), which the driver would report "
                       f"again instead of running a new episode: name a new --out folder")
    work = os.path.abspath(ns.work) if ns.work else os.path.join(SCRATCH_PARENT, SCRATCH_PREFIX + "x" * 8)
    if longest_socket_path(work, ns.label, ns.cell) > SOCKET_PATH_MAX:
        return 2, (f"the episode's socket paths under {ns.work or SCRATCH_PARENT} would be longer than "
                   f"{SOCKET_PATH_MAX} bytes: use a shorter --label" + (" or --work" if ns.work else ""))
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
    ns.extra_body_given = ns.extra_body
    ns.extra_body = route.extra_body_for(ns.base_url or "", ns.model, ns.extra_body)
    arms = ["plain", "rcp"] if ns.arm == "both" else [ns.arm]
    interrupted, ns.own_work, ns.started, ns.stop_problem = None, ns.work is None, False, None
    own_work = ns.own_work
    with Signals() as signals:
        try:
            code = checked_run(ns, arms, signals)
            # Held inside the try: a first signal before this line is handled below, and none raises after it, so
            # the clean-up runs to its end.
            signals.hold()
        except Interrupted as exc:
            interrupted = exc.signum
            code = 128 + exc.signum
        finally:
            try:
                signals.hold()
            except Interrupted as exc:   # a first signal on the way out of an error: no other one raises now
                interrupted = exc.signum
            if own_work and ns.work:
                shutil.rmtree(ns.work, ignore_errors=True)
            if interrupted is not None:
                say(f"{progress.utc_now()} stopped by {signal.Signals(interrupted).name}: "
                    + (stopped_text(ns, own_work) if ns.started else "no episode had started"), err=True)
    return code


def checked_run(ns, arms, signals):
    """Every check before the start, then the run: its exit code."""
    refused = refusal_before_start(ns, arms)
    if refused:
        say(refused[1], err=True)
        return refused[0]
    redirects.refuse_everywhere()   # this process sends the key too: it follows no redirect from here on
    trees = {"rcp": layout.RCP_TREE}
    if "plain" in arms:
        try:
            trees["plain"] = layout.plain_tree()
        except (OSError, ValueError) as exc:
            say(f"the plain setup's code tree could not be assembled: {exc}", err=True)
            return 2
    if not ns.no_endpoint_check:
        refusal = endpoint_refusal(ns)
        if refusal:
            say(f"the model endpoint did not take a test request, so no episode was started.\n  {refusal}\n"
                f"Check the URL ({ns.base_url}), the key in {ns.key_env} and the model name ({ns.model}).",
                err=True)
            return 2
    if ns.own_work:
        with signals.deferred():   # a signal while the folder is made is raised once ns.work names it
            ns.work = ns.work_shown = scratch_for()
    else:
        ns.work, ns.work_shown = os.path.abspath(ns.work), ns.work
    return run_cell(ns, arms, trees, signals)


def stopped_text(ns, own_work):
    """What an interrupt after the start ended, and what it may have left running."""
    if ns.stop_problem is not None:
        why = {STOP_TIMED_OUT: f"the running episode did not stop within {STOP_WINDOW_S:g} s",
               STOP_REPORTED_RUNNING: "the episode's stop command reported part of it still running",
               STOP_NOT_STARTED: "the episode's stop command could not be started"}[ns.stop_problem]
        text = why + ", so its simulator may still be running" + ("; the scratch folder was removed" if own_work
                                                                   else "")
    else:
        text = "the running episode was ended" + (" and the scratch folder removed" if own_work else "")
    return text + f"; {os.path.join(ns.out, 'summary.json')} holds the setups that finished"


def write_summary(out_dir, record):
    """summary.json, replaced whole: a reader, or an interrupt during the write, never finds part of it."""
    path = os.path.join(out_dir, "summary.json")
    with open(path + ".tmp", "w", encoding="utf-8") as f:
        json.dump(record, f, indent=2, sort_keys=True)
    os.replace(path + ".tmp", path)


def start_lines(ns, arms, cond):
    return [
        f"{progress.utc_now()} start: {ns.cell} with {ns.model}, setup {' then '.join(arms)}; results in {ns.out}",
        "  " + route.condition_line(cond),
        "  " + expected.expected_line(ns.model, arms, BUDGET, ns.clock),
        f"  scratch folder: {ns.work_shown}" + (" (removed at the end)" if ns.own_work else ""),
    ]


def run_cell(ns, arms, trees, signals):
    ns.started = True
    out_dir = os.path.abspath(ns.out)
    os.makedirs(out_dir, exist_ok=True)
    cells_file = os.path.join(out_dir, "cell.txt")
    with open(cells_file, "w", encoding="utf-8") as f:
        f.write(cell_line(ns.cell) + "\n")
    cond = route.condition(ns.base_url, ns.extra_body, ns.extra_body_given)
    record = {"cell": ns.cell, "model": ns.model, "budget": BUDGET, "route": cond, "rows": []}
    try:
        write_summary(out_dir, record)
        for text in start_lines(ns, arms, cond):
            say(text)
        for arm in arms:
            if arm == "rcp":
                problem = body_record_of(ns, out_dir, signals)
                if problem:
                    say(f"the robot body of {ns.cell} has no record on this machine and measuring one failed, so "
                        f"the {arm} episode was not started (without a record every move is refused).\n  {problem}",
                        err=True)
                    return 2
            say(f"{progress.utc_now()} running {arm} on {ns.cell} with {ns.model} ...")
            monitor = progress.Progress(arm, scratch_group(ns, arm), ns.cell, ns.model, BUDGET["control_steps"], say)
            with route.Relay(ns.base_url, on_request=monitor.poll) as relay:
                rc, wall = run_arm(ns, arm, out_dir, cells_file, trees[arm], relay.base_url, signals)
                counts = {"base_url": relay.base_url, **relay.counts}
            try:
                monitor.poll()   # the last reply: no request follows it
            except Exception:   # a progress line that cannot be read never costs the verdict
                pass
            row = read_result(ns, arm, out_dir)
            row.update(driver_exit=rc, wall_seconds=wall, relay=counts)
            record["rows"].append(row)
            write_summary(out_dir, record)
            show(row)
    except Interrupted as exc:
        record["interrupted"] = signal.Signals(exc.signum).name
        record["running_episode_stopped"] = ns.stop_problem is None
        record["stop_problem"] = ns.stop_problem   # None, or which of the STOP_* causes left it unconfirmed
        try:
            write_summary(out_dir, record)
        except OSError:
            pass
        raise
    say(f"summary: {os.path.join(ns.out, 'summary.json')}")
    return 0 if all(r["scored"] for r in record["rows"]) else 1


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
    r.add_argument("--work", default=None,
                   help="scratch folder; keep it short (unix socket paths). Default: a new folder under /tmp, "
                        "removed at the end, after an error and after an interrupt")
    r.add_argument("--label", default="run", help="short name for the results folders")
    r.add_argument("--extra-body", default=None,
                   help='JSON object merged into every request body (default: {"reasoning_effort": "high"}, and '
                        'through OpenRouter a google/ model also gets {"provider": {"only": ["google-vertex"]}}, '
                        'the backend the measured runs used)')
    r.add_argument("--clock", type=float, default=5400.0, help="wall seconds before an episode is stopped")
    r.add_argument("--no-endpoint-check", action="store_true",
                   help="skip the one small test request that is sent to the endpoint before the simulator starts")
    r.set_defaults(fn=cmd_run_cell)
    sub.add_parser("cells", help="list the example cells").set_defaults(fn=cmd_cells)
    return p


def main(argv=None):
    ns = parser().parse_args(argv)
    return ns.fn(ns)
