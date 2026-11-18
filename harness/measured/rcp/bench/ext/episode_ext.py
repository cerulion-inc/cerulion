# SPDX-License-Identifier: AGPL-3.0-only
"""Start, serve, score and stop one LIBERO or LIBERO-PRO episode of the with and without rcpx study. The env host (bench/ext/libero/envhost.py, the benchmark's own venv, started by
bench.ext.libero.launch) holds the physics, the success check and the truth; the serve process (this file, rcpx
venv) serves EPDIR/rcp.sock with the rcpx socket protocol, so rcpx.cli and bench/operators/chat_operator.py work
unchanged.

    python -m bench.ext.episode_ext start --cell "ID BENCH SUITE TASK INIT PERTURB SURFACE [PACKAGES]" --ep EPDIR \
        [--budget-requests 80] [--budget-steps 900] [--gl osmesa|egl|auto] [--wait-s 300]
    python -m bench.ext.episode_ext score --ep EPDIR      bench.ext.score_ext.score: <scoring>/result.json
    python -m bench.ext.episode_ext stop --ep EPDIR       the serve process and the env host, by recorded pid

Run from ROOT. The cell is bench/ext/libero/cells.py's: BENCH libero (SUITE libero_10) or libero_pro (SUITE the base
suite, libero_goal, or the runtime one, libero_goal_swap; PERTURB swap or task); ID is the grid's name for it
(recorded beside cells.py's id). SURFACE: native (the published plain interface, bench/ext/libero/native.py),
native_sense (the same plus the rcpx arm's sensing) or vnext (the rcpx arm, VNEXT_MODULE.build_session:
bench/ext/libero/backend.py). PACKAGES is for vnext cells only.

start launches the env host on <scoring>/env.sock, resets it on the cell (its env seed is cells.py's, 142 + INIT;
its step budget --budget-steps), writes <scoring>/serve.json (serve_facts: the task sentence, the workspace and the
task's index in its suite, all the serve process is told about the cell) and starts the serve process; it exits 0
once EPDIR/rcp.sock answers. The scoring folder (EPDIR/../_scoring/<name>, outside EPDIR) holds episode.json (the cell, surface, packages, budgets, pids),
the env host's truth log, truth socket and logs, and server.log. EPDIR holds episode.json (surface and the serve
pid only), server_ready.json, rcp.sock, the rcpx journal, frames and episode_end.json.

RCPX_EXT_ENVHOST (a JSON list of str.format templates; placeholders python, root, env_sock, scoring, runtime, gl)
runs another env host in place of bench.ext.libero.launch (the tests' stub); it must write <scoring>/ready.json.
"""
import argparse
import importlib
import json
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from bench.ext import envproto  # noqa: E402
from bench.ext.libero import cells, native  # noqa: E402
from rcpx import server  # noqa: E402

SURFACES = ("native", "native_sense", "vnext")
VNEXT_MODULE = "bench.ext.libero.backend"
EPISODE_NAME, READY_NAME, SERVE_NAME = "episode.json", "server_ready.json", "serve.json"
ENV_SOCK, TRUTH_SOCK, HOST_READY = "env.sock", "truth.sock", "ready.json"
MAX_SOCK = server.MAX_SOCK_PATH


def scoring_dir(ep_dir):
    """bench.run_episode.scoring_dir, restated (that module loads the whole bench)."""
    ep = os.path.abspath(ep_dir)
    return os.path.join(os.path.dirname(ep), "_scoring", os.path.basename(ep))


def _write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, sort_keys=True)
    os.replace(tmp, path)


def _read_json(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def merge_json(path, fields):
    out = _read_json(path)
    out.update(fields)
    _write_json(path, out)
    return out


# ---------------------------------------------------------------------------------------------------------------
# Cells, hooks, processes


def parse_cell(line):
    """(the line's fields, cells.py's cell) for a cell line, or ValueError."""
    fields = str(line or "").split()
    if len(fields) not in (7, 8):
        raise ValueError(f"a cell line is ID BENCH SUITE TASK INIT PERTURB SURFACE [PACKAGES], got {len(fields)} "
                         f"fields")
    cid, bench, suite, task, init, perturb, surface = fields[:7]
    try:
        task, init = int(task), int(init)
    except ValueError:
        raise ValueError("TASK and INIT must be whole numbers") from None
    if task < 0 or init < 0:
        raise ValueError("TASK and INIT must be 0 or more")
    if surface not in SURFACES:
        raise ValueError(f"unknown SURFACE {surface!r}; known: {', '.join(SURFACES)}")
    if len(fields) == 8 and surface != "vnext":
        raise ValueError("PACKAGES is for vnext cells only")
    try:
        if bench == "libero" and suite == "libero_10" and perturb == "none":
            cell = cells.libero10(inits=[init], tasks=[task])[0]
        elif bench == "libero_pro" and perturb in ("swap", "task"):
            base = suite[:-len(perturb) - 1] if suite.endswith("_" + perturb) else suite
            cell = cells.pro_cell(base, perturb, task, init)
        else:
            raise KeyError(bench)
    except (KeyError, IndexError):
        raise ValueError(f"no cell for BENCH {bench} SUITE {suite} TASK {task} INIT {init} PERTURB {perturb} "
                         f"(bench/ext/libero/cells.py)") from None
    line = {"id": cid, "bench": bench, "suite": suite, "task": task, "init": init, "perturb": perturb,
            "surface": surface, "packages": fields[7] if len(fields) == 8 else ("all" if surface == "vnext" else None)}
    return line, cell


def vnext_hook():
    """The vnext arm's module, or None while it is missing (an import error inside it is raised)."""
    try:
        return importlib.import_module(VNEXT_MODULE)
    except ModuleNotFoundError as exc:
        if exc.name == VNEXT_MODULE:
            return None
        raise


def _cmdline(pid):
    try:
        out = subprocess.run(["ps", "-p", str(int(pid)), "-o", "command="], capture_output=True, text=True)
        return out.stdout.split()
    except (OSError, ValueError):
        return []


def _gone(pid):
    try:
        if os.waitpid(int(pid), os.WNOHANG)[0] == int(pid):
            return True
    except ChildProcessError:
        pass
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return True
    return not _cmdline(pid)


def _stop_pid(pid, must_hold, wait_s=30.0, ask=None):
    """End a process this runner started (its command line must hold every text in must_hold): ask() first when
    given (a polite close), then SIGTERM, then SIGKILL, waiting up to wait_s each. Returns what happened."""
    if not pid or not _cmdline(pid) or not all(m in _cmdline(pid) for m in must_hold):
        return "already ended"
    for how in ("ask", signal.SIGTERM, signal.SIGKILL):
        if how == "ask":
            if ask is None:
                continue
            try:
                ask()
            except Exception:   # a host that does not answer is ended by signal below
                pass
        else:
            try:
                os.kill(int(pid), how)
            except ProcessLookupError:
                return "stopped"
        waited = 0.0
        while waited < wait_s:
            if _gone(pid):
                return "stopped" if how != signal.SIGKILL else "killed after it ignored SIGTERM"
            time.sleep(0.2)
            waited += 0.2
    return "still running"


def _launch_host(cell, scoring, env_sock, gl, wait_s):
    """(pid, None) once the env host's ready.json is there, else (None, why); the host's output: scoring/host.out."""
    ready = os.path.join(scoring, HOST_READY)
    if os.path.exists(ready):
        os.unlink(ready)
    template = os.environ.get("RCPX_EXT_ENVHOST")
    if not template:
        from bench.ext import libero as L
        try:
            return L.launch(cell["runtime"], env_sock, scoring, gl=gl, wait_s=wait_s).pid, None
        except RuntimeError as exc:
            return None, str(exc)
    values = {"python": sys.executable, "root": ROOT, "env_sock": env_sock, "scoring": scoring,
              "runtime": cell["runtime"], "gl": gl}
    cmd = [str(t).format(**values) for t in json.loads(template)]
    with open(os.path.join(scoring, "host.out"), "ab") as log:
        proc = subprocess.Popen(cmd, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
    why = _wait(lambda: os.path.exists(ready), proc, wait_s)
    if why:
        _stop_pid(proc.pid, [env_sock])
        return None, f"the env host {why}; see {scoring}/host.out"
    return proc.pid, None


def _wait(cond, child, wait_s):
    waited = 0.0
    while not cond():
        if child.poll() is not None:
            return f"exited early (code {child.returncode})"
        if waited > wait_s:
            return f"was not ready after {wait_s:g} s"
        time.sleep(0.2)
        waited += 0.2
    return None


# ---------------------------------------------------------------------------------------------------------------
# Verbs


def start_main(ns):
    try:
        line, cell = parse_cell(ns.cell)
    except ValueError as exc:
        print(f"cannot start: {exc}", file=sys.stderr)
        return 2
    if ns.budget_requests < 1 or ns.budget_steps < 1:
        print("cannot start: --budget-requests and --budget-steps must be at least 1", file=sys.stderr)
        return 2
    ep_dir = os.path.abspath(ns.ep)
    scoring = scoring_dir(ep_dir)
    env_sock = os.path.join(scoring, ENV_SOCK)
    for path in (os.path.join(ep_dir, server.SOCK_NAME), os.path.join(scoring, TRUTH_SOCK)):
        if len(os.fsencode(path)) > MAX_SOCK:
            print(f"cannot start: {path} is longer than {MAX_SOCK} bytes; use a shorter EPDIR", file=sys.stderr)
            return 2
    used = [p for p in (os.path.join(ep_dir, EPISODE_NAME), os.path.join(scoring, EPISODE_NAME),
                        os.path.join(scoring, "truth_log.jsonl")) if os.path.exists(p)]
    if used:   # a second host would append to the same truth log: one directory, one episode
        print(f"cannot start: {ep_dir} already held an episode ({used[0]}); use a new directory", file=sys.stderr)
        return 2
    if line["surface"] == "vnext":
        hook = vnext_hook()
        if hook is None or not callable(getattr(hook, "build_session", None)):
            print(f"cannot start: the vnext arm is not wired yet ({VNEXT_MODULE}.build_session is missing)",
                  file=sys.stderr)
            return 2
    cell = dict(cell, budget_steps=int(ns.budget_steps))
    budget = {"requests": int(ns.budget_requests), "steps": int(ns.budget_steps)}
    os.makedirs(ep_dir, exist_ok=True)
    ident = os.path.join(scoring, EPISODE_NAME)
    _write_json(ident, {**line, "cell_id": cell["id"], "runtime": cell["runtime"], "base_suite": cell["base_suite"],
                        "seed": cell["env_seed"], "language": cell["language"], "budget": budget,
                        "budget_steps": budget["steps"], "max_steps_ref": cell["max_steps_ref"],
                        "workspace": cell["workspace"], "env_sock": env_sock})
    public = os.path.join(ep_dir, EPISODE_NAME)
    _write_json(public, {"surface": line["surface"]})
    gl = ns.gl or os.environ.get("MUJOCO_GL") or "osmesa"
    gl = gl if gl in ("egl", "osmesa", "auto") else "osmesa"   # auto: a free box EGL slot, else osmesa
    pid, why = _launch_host(cell, scoring, env_sock, gl, ns.wait_s)
    if pid is None:
        print(f"cannot start: {why}", file=sys.stderr)
        return 1
    host_info = _read_json(os.path.join(scoring, HOST_READY))
    merge_json(ident, {"env_pid": pid, "gl": gl, "renderer": host_info.get("gl", gl),
                       "egl_slot": host_info.get("egl_slot")})
    try:
        with envproto.Client(env_sock, timeout_s=ns.wait_s) as c:
            c.call("reset", cell=cell)
    except envproto.EnvHostError as exc:
        _stop_pid(pid, [env_sock])
        print(f"cannot start: the env host's reset failed ({exc.kind}); see {scoring}", file=sys.stderr)
        return 1
    _write_json(os.path.join(scoring, SERVE_NAME), serve_facts(cell))
    cmd = [sys.executable, "-m", "bench.ext.episode_ext", "serve", "--ep", ep_dir, "--env-sock", env_sock,
           "--surface", line["surface"], "--budget-requests", str(budget["requests"]), "--budget-steps",
           str(budget["steps"])] + (["--packages", line["packages"]] if line["packages"] else [])
    with open(os.path.join(scoring, "server.log"), "w") as log:
        child = subprocess.Popen(cmd, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
    merge_json(public, {"pid": child.pid})
    why = _wait(lambda: os.path.exists(os.path.join(ep_dir, READY_NAME)), child, ns.wait_s)
    if why:
        _stop_pid(child.pid, [ep_dir, "serve"])
        _stop_pid(pid, [env_sock])
        print(f"cannot start: the episode server {why}; see {scoring}/server.log", file=sys.stderr)
        return 1
    print(f"episode ready: {json.dumps(_read_json(public))}")   # never the cell: an operator may see this output
    cli = f"{ROOT}/venv/bin/python -m rcpx.cli --ep {ep_dir}"
    print("operator cli (run from ROOT):")
    for text in (f"cd {ROOT}", f"{cli} list", f"{cli} read robot://practices", f"{cli} read robot://task",
                 f"{cli} call NAME 'JSON ARGS'"):
        print("  " + text)
    return 0


def serve_facts(cell):
    """<scoring>/serve.json, all the serve process is told about the cell: the task sentence, the workspace, the
    support height, the base suite, the benchmark, the perturbation and the task's index in its base suite. The index
    is what the plain arm's embodiment name carries (bench/ext/libero/native.py embodiment_name: 'libero_goal_task05'
    on LIBERO-PRO, the form 'libero_10_task05' has on LIBERO-10); no init, seed, file name or goal is in it."""
    return {"language": cell["language"], "workspace": cell["workspace"], "surface_z": cell["surface_z"],
            "base_suite": cell["base_suite"], "bench": cell["bench"], "perturb": cell["perturb"],
            "task": int(cell["task"])}


def build(env, ep_dir, public, surface, budget_requests, budget_steps, packages=None):
    """The session the serve process serves: the native ones here, vnext through VNEXT_MODULE.build_session(env,
    ep_dir, public, {"requests", "steps"}, packages)."""
    if surface in native.SURFACES:
        return native.build_session(env, ep_dir, public, surface, budget_requests, budget_steps)
    hook = vnext_hook()
    if hook is None:
        raise RuntimeError(f"{VNEXT_MODULE} is missing")
    return hook.build_session(env, ep_dir, public, {"requests": int(budget_requests), "steps": int(budget_steps)},
                              packages or "all")


def serve_main(ns):
    ep_dir = os.path.abspath(ns.ep)
    public = _read_json(os.path.join(scoring_dir(ep_dir), SERVE_NAME))
    env = envproto.Client(ns.env_sock)
    session = build(env, ep_dir, public, ns.surface, ns.budget_requests, ns.budget_steps, ns.packages)

    def ready():
        _write_json(os.path.join(ep_dir, READY_NAME), {"pid": os.getpid(),
                                                        "socket": os.path.join(ep_dir, server.SOCK_NAME)})
    try:
        reason = server.serve(session, os.path.join(ep_dir, server.SOCK_NAME), on_ready=ready)
    finally:
        env.close()
    print(f"episode ended: {reason}", flush=True)
    return 0


def score_main(ns):
    from bench.ext import score_ext
    print(json.dumps(score_ext.score(os.path.abspath(ns.ep)), indent=2, sort_keys=True, default=str))
    return 0


def stop_main(ns):
    ep_dir = os.path.abspath(ns.ep)
    ident = _read_json(os.path.join(scoring_dir(ep_dir), EPISODE_NAME))
    public = _read_json(os.path.join(ep_dir, EPISODE_NAME))
    srv = _stop_pid(public.get("pid"), [ep_dir, "serve"])
    sock = ident.get("env_sock") or "?"

    def close():
        with envproto.Client(sock, timeout_s=30.0) as c:
            c.call("close")
    host = _stop_pid(ident.get("env_pid"), [sock], ask=close)
    print(f"episode server: {srv}; env host: {host}")
    return 0 if "still running" not in (srv, host) else 1


def main(argv=None):
    p = argparse.ArgumentParser(prog="bench.ext.episode_ext")
    sub = p.add_subparsers(dest="cmd", required=True)
    q = sub.add_parser("start")
    q.add_argument("--cell", required=True, help="ID BENCH SUITE TASK INIT PERTURB SURFACE [PACKAGES]")
    q.add_argument("--ep", required=True)
    q.add_argument("--budget-requests", dest="budget_requests", type=int, default=80)
    q.add_argument("--budget-steps", dest="budget_steps", type=int, default=900)
    q.add_argument("--gl", default=None, choices=("osmesa", "egl", "auto"),
                   help="the env host's renderer (default MUJOCO_GL, else osmesa; auto = a free EGL slot or osmesa)")
    q.add_argument("--wait-s", dest="wait_s", type=float, default=300.0, help="seconds to wait for readiness")
    s = sub.add_parser("serve")
    s.add_argument("--ep", required=True)
    s.add_argument("--env-sock", dest="env_sock", required=True)
    s.add_argument("--surface", required=True, choices=SURFACES)
    s.add_argument("--packages", default=None)
    s.add_argument("--budget-requests", dest="budget_requests", type=int, default=80)
    s.add_argument("--budget-steps", dest="budget_steps", type=int, default=900)
    for name in ("score", "stop"):
        sub.add_parser(name).add_argument("--ep", required=True)
    ns = p.parse_args(argv)
    return {"start": start_main, "serve": serve_main, "score": score_main, "stop": stop_main}[ns.cmd](ns)


if __name__ == "__main__":
    sys.exit(main())
