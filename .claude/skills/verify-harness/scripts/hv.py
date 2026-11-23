#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Verify the Cerulion harness: run its offline gates, start its sim server, drive one episode the way an MCP
client does, and keep the evidence. Standard library only.

    hv.py doctor                      is this checkout and its server worth driving? read-only
    hv.py test                        unit tests on a clean Python, the transport tests, shellcheck, verify
    hv.py up [--task T] [--cli PATH]  start a sim server for this checkout on a free loopback port
    hv.py tools [--episode L]         start or join episode L; save its instructions, tools and resources
    hv.py call TOOL [JSON] [--episode L]
    hv.py read URI [--episode L]
    hv.py episode [ID] [--solved]     the episode record and verdict; --solved: exit 1 unless solved
    hv.py smoke                       a scripted bowl-on-plate episode on the running server; exit 0 if solved
    hv.py stdio                       the stdio transport end to end, on a server of its own
    hv.py down                        stop the server this checkout started; evidence stays

Exit codes: 0 ok, 1 a check or a request failed, 2 usage, 3 the tool answered with is_error.
"""
import argparse
import base64
import contextlib
import datetime
import importlib.util
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

PROTOCOL = "2026-07-28"
REPO = Path(__file__).resolve().parents[4]
RUNS = Path(os.environ.get("HV_RUNS") or Path.home() / ".cache" / "cerulion-verify-harness")


def harness_dir(given):
    folder = Path(given or os.environ.get("HV_HARNESS") or REPO / "harness").resolve()
    if not (folder / "rcp.sh").is_file() or not (folder / "install.sh").is_file():
        sys.exit(f"hv: {folder} holds no harness (rcp.sh and install.sh)")
    return folder


def pointer(harness):
    return harness / "out" / "verify-current"


def current_run(harness, required=True):
    p = pointer(harness)
    run = Path(p.read_text().strip()) if p.is_file() else None
    if run is None or not (run / "run.json").is_file():
        if required:
            sys.exit("hv: no server started from this checkout; run `hv.py up` first")
        return None, None
    return run, json.loads((run / "run.json").read_text())


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def get(url, timeout=10):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status, r.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read().decode("utf-8", "replace")
    except (urllib.error.URLError, OSError) as exc:
        return None, str(exc)


def rpc(url, method, params, query, n=1, timeout=900):
    message = {"jsonrpc": "2.0", "id": n, "method": method, "params": dict(params, _meta={
        "io.modelcontextprotocol/protocolVersion": PROTOCOL, "io.modelcontextprotocol/clientCapabilities": {}})}
    headers = {"content-type": "application/json", "accept": "application/json, text/event-stream",
               "mcp-protocol-version": PROTOCOL, "mcp-method": method}
    if method == "tools/call":
        headers["mcp-name"] = params["name"]
    elif method == "resources/read":
        headers["mcp-name"] = params["uri"]
    req = urllib.request.Request(f"{url}/mcp{query}", data=json.dumps(message).encode(), headers=headers,
                                 method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            kind, body = r.headers.get("content-type", ""), r.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as exc:
        kind, body = exc.headers.get("content-type", ""), exc.read().decode("utf-8", "replace")
    except (urllib.error.URLError, OSError) as exc:
        sys.exit(f"hv: {url}/mcp cannot be reached ({exc}); run `hv.py doctor`")
    try:
        if kind.startswith("text/event-stream"):
            docs = [json.loads(line[5:]) for line in body.splitlines() if line.startswith("data:")]
        else:
            docs = [json.loads(body)] if body.strip() else []
    except ValueError:
        sys.exit(f"hv: the answer to {method} is not JSON: {body[:400]}")
    answer = next((d for d in docs if d.get("id") == n), None)
    if answer is None:
        sys.exit(f"hv: no answer to {method} from {url}/mcp{query}: {body[:400]}")
    return answer


def query_of(episode, task):
    return f"?episode={episode}" + (f"&state={task}" if task else "")


def evidence(run, name):
    folder = run / "evidence"
    folder.mkdir(parents=True, exist_ok=True)
    seq = len(list(folder.glob("[0-9][0-9][0-9]-*.json"))) + 1
    return folder, f"{seq:03d}-{re.sub(r'[^A-Za-z0-9_.-]+', '_', name)}"


def python_without_mcp_sdk():
    # harness/tests skips its transport cases without an MCP SDK and fails them against an unpinned one.
    names = []
    if shutil.which("uv"):
        for version in ("3.12", "3.13", "3.11", "3.10"):
            found = subprocess.run(["uv", "python", "find", version], capture_output=True, text=True)
            if found.returncode == 0 and found.stdout.strip():
                names.append(found.stdout.strip())
    names += [n for n in ("python3.12", "python3.13", "python3.11", "python3.10", "python3") if shutil.which(n)]
    probe = ("import importlib.util as u, sys; ok = sys.version_info >= (3, 10) and u.find_spec('mcp') is None; "
             "print(sys.version.split()[0]); sys.exit(0 if ok else 3)")
    for name in names:
        r = subprocess.run([name, "-I", "-c", probe], capture_output=True, text=True)
        if r.returncode == 0:
            return name, r.stdout.strip()
    return None, None


def report(ok, what, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {what}" + (f": {detail}" if detail else ""))
    return ok


def cmd_doctor(ns):
    h = harness_dir(ns.harness)
    good = report(True, "harness folder", str(h))
    check = subprocess.run(["bash", str(h / "install.sh"), "--check"], capture_output=True, text=True)
    good &= report(check.returncode == 0, "install", (check.stdout + check.stderr).strip().replace("\n", "; "))
    diff = subprocess.run(["git", "-C", str(h), "status", "--porcelain", "--", "measured", "provenance"],
                          capture_output=True, text=True)
    good &= report(diff.returncode == 0 and not diff.stdout.strip(), "measured/ and provenance/ untouched",
                   diff.stdout.strip().replace("\n", "; ") or ("git: " + diff.stderr.strip() if diff.returncode else ""))
    free_gb = shutil.disk_usage(h).free / 2**30
    good &= report(free_gb >= 5, "disk free beside the harness", f"{free_gb:.1f} GB (the install refuses under 5)")
    py, version = python_without_mcp_sdk()
    good &= report(py is not None, "clean Python for the unit tests", f"{py} {version}" if py else
                   "no Python 3.10+ without an MCP SDK on PATH (uv python install 3.12 gives one)")
    cli = ns.cli or shutil.which("cerulion")
    if cli:
        r = subprocess.run([cli, "rcp", "--help"], capture_output=True, text=True)
        v = subprocess.run([cli, "--version"], capture_output=True, text=True).stdout.strip()
        report(r.returncode == 0, "cerulion rcp (optional)", f"{cli}, {v}" + ("" if r.returncode == 0 else
               ", has no rcp verb: drive through rcp.sh, or pass --cli with a cerulion of 2.1.0 or later"))
    run, meta = current_run(h, required=False)
    if meta:
        up = alive(meta["pid"])
        good &= report(up, "server process of this checkout", f"pid {meta['pid']}, run {run}")
        if up:
            status, body = get(meta["url"] + "/health")
            good &= report(status == 200, "server health", body.strip()[:300])
    else:
        print("info  no server started from this checkout (hv.py up starts one)")
    return 0 if good else 1


def cmd_test(ns):
    h = harness_dir(ns.harness)
    run = RUNS / ("test-" + stamp())
    run.mkdir(parents=True, exist_ok=True)
    py, version = python_without_mcp_sdk()
    tests = str(h / "tests")
    steps, good = [], True
    if py:
        steps.append(("unit tests, " + version + " without an MCP SDK",
                      [py, "-I", "-B", "-m", "unittest", "discover", "-s", tests], h.parent))
    else:
        good = report(False, "unit tests", "no clean Python; see hv.py doctor")
    venv = h / "venv" / "bin" / "python"
    if venv.exists():
        steps.append(("transport tests with the pinned MCP SDK (harness venv)",
                      [str(venv), "-I", "-B", "-m", "unittest", "discover", "-s", tests, "-p",
                       "test_playground_*.py"], h.parent))
    else:
        print("info  no install: the transport tests run only with the pinned MCP SDK of harness/venv")
    scripts = subprocess.run(["git", "-C", str(h), "ls-files", "*.sh"], capture_output=True,
                             text=True).stdout.split()
    if shutil.which("shellcheck"):
        steps.append(("shellcheck -x over the tracked scripts", ["shellcheck", "-x", *scripts], h))
    else:
        good = report(False, "shellcheck", "not on PATH (CI runs it)")
    steps.append(("verify: the measured code against its provenance", ["bash", str(h / "rcp.sh"), "verify"], h))
    for n, (what, argv, cwd) in enumerate(steps, 1):
        log = run / f"{n}-{re.sub(r'[^a-z]+', '-', what.split(',')[0].lower()).strip('-')}.log"
        t0 = time.monotonic()
        with open(log, "w") as out:
            code = subprocess.run(argv, cwd=cwd, stdout=out, stderr=subprocess.STDOUT).returncode
        tail = log.read_text(errors="replace").strip().splitlines()[-1:] or [""]
        good &= report(code == 0, what, f"exit {code} in {time.monotonic() - t0:.0f} s, {tail[0][:120]}; log {log}")
    return 0 if good else 1


def stamp():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"-{os.getpid()}"


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def cmd_up(ns):
    h = harness_dir(ns.harness)
    run, meta = current_run(h, required=False)
    if meta and alive(meta["pid"]):
        print(f"hv: this checkout's server already runs: {meta['url']} (pid {meta['pid']}); `hv.py down` first")
        return 1
    run = RUNS / stamp()
    home = run / "home"
    home.mkdir(parents=True)
    addr = f"127.0.0.1:{ns.port or free_port()}"
    env = dict(os.environ)
    if ns.cli:
        argv = [ns.cli, "rcp", "--sim", "--http", addr]
        env["CERULION_HARNESS_DIR"] = str(h)
    else:
        argv = ["bash", str(h / "rcp.sh"), "sim", "--http", addr]
        env["CERULION_HOME"] = str(home)
    argv += ["--task", ns.task] if ns.task else []
    with open(run / "server.stdout", "w") as out, open(run / "server.log", "w") as err:
        proc = subprocess.Popen(argv, cwd=run, env=env, stdin=subprocess.DEVNULL, stdout=out, stderr=err,
                                start_new_session=True)
    url = f"http://{addr}"
    meta = {"pid": proc.pid, "url": url, "argv": argv, "harness": str(h), "task": ns.task,
            "cerulion_home": None if ns.cli else str(home), "started": stamp()}
    (run / "run.json").write_text(json.dumps(meta, indent=2))
    pointer(h).parent.mkdir(parents=True, exist_ok=True)
    pointer(h).write_text(str(run))
    deadline = time.monotonic() + ns.wait
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            print(f"hv: the server exited with {proc.returncode}; see {run / 'server.log'}")
            return 1
        status, body = get(url + "/health", timeout=2)
        if status == 200:
            print(f"up: {url}/mcp (pid {proc.pid})\nrun: {run}\nhealth: {body.strip()[:300]}")
            return 0
        time.sleep(1)
    print(f"hv: no answer from {url}/health after {ns.wait} s; see {run / 'server.log'}; `hv.py down` stops it")
    return 1


def save_answer(run, name, request, answer):
    folder, base = evidence(run, name)
    result = answer.get("result") or {}
    images = []
    for k, block in enumerate(result.get("content") or []):
        if block.get("type") == "image" and block.get("data"):
            ext = {"image/jpeg": "jpg", "image/png": "png"}.get(block.get("mimeType"), "bin")
            path = folder / f"{base}-{k}.{ext}"
            path.write_bytes(base64.b64decode(block["data"]))
            images.append(str(path))
            block = dict(block, data=f"<{len(block['data'])} base64 chars, saved as {path.name}>")
            result["content"][k] = block
    (folder / f"{base}.json").write_text(json.dumps({"request": request, "answer": answer}, indent=2))
    return folder / f"{base}.json", images


def cmd_tools(ns):
    h = harness_dir(ns.harness)
    run, meta = current_run(h)
    q = query_of(ns.episode, ns.task)
    hello = rpc(meta["url"], "server/discover", {}, q)
    if "error" in hello:
        print(json.dumps(hello["error"], indent=2))
        return 1
    instructions = hello["result"].get("instructions", "")
    folder, base = evidence(run, f"episode-{ns.episode}-instructions")
    told = folder / f"{base}.md"
    told.write_text(instructions)
    (folder / f"{base}.json").write_text(json.dumps(hello, indent=2))
    tools = rpc(meta["url"], "tools/list", {}, q, n=2)
    resources = rpc(meta["url"], "resources/list", {}, q, n=3)
    templates = rpc(meta["url"], "resources/templates/list", {}, q, n=4)
    _, base = evidence(run, f"episode-{ns.episode}-listing")
    (folder / f"{base}.json").write_text(json.dumps({"tools": tools, "resources": resources,
                                                     "templates": templates}, indent=2))
    eid = re.search(r"/view/([A-Za-z0-9_-]+)", instructions)
    print(f"episode {ns.episode}: {eid.group(1) if eid else '(id not in the instructions)'}")
    print(f"instructions ({len(instructions)} chars): {told}")
    for t in (tools.get("result") or {}).get("tools", []):
        print(f"tool  {t['name']}: {(t.get('description') or '').splitlines()[0][:110]}")
    for r in (resources.get("result") or {}).get("resources", []):
        print(f"resource  {r['uri']}")
    for r in (templates.get("result") or {}).get("resourceTemplates", []):
        print(f"template  {r['uriTemplate']}")
    return 1 if "error" in tools else 0


def cmd_call(ns):
    h = harness_dir(ns.harness)
    run, meta = current_run(h)
    try:
        args = json.loads(ns.args) if ns.args else {}
    except ValueError as exc:
        print(f"hv: the arguments are not JSON: {exc}")
        return 2
    params = {"name": ns.tool, "arguments": args}
    t0 = time.monotonic()
    answer = rpc(meta["url"], "tools/call", params, query_of(ns.episode, ns.task))
    path, images = save_answer(run, ns.tool, params, answer)
    if "error" in answer:
        print(json.dumps(answer["error"], indent=2))
        return 1
    result = answer["result"]
    for block in result.get("content") or []:
        if block.get("type") == "text":
            print(block["text"])
    for image in images:
        print(f"image: {image}")
    print(f"took {time.monotonic() - t0:.1f} s; saved {path}")
    return 3 if result.get("isError") else 0


def cmd_read(ns):
    h = harness_dir(ns.harness)
    run, meta = current_run(h)
    answer = rpc(meta["url"], "resources/read", {"uri": ns.uri}, query_of(ns.episode, ns.task))
    folder, base = evidence(run, "read-" + ns.uri)
    (folder / f"{base}.json").write_text(json.dumps({"uri": ns.uri, "answer": answer}, indent=2))
    if "error" in answer:
        print(json.dumps(answer["error"], indent=2))
        return 1
    for c in answer["result"].get("contents", []):
        print(c.get("text", f"<{c.get('mimeType')} blob>"))
    print(f"saved {folder / base}.json")
    return 0


def episode_ids(url):
    """Every episode id of the server at url, live first; None when /episodes does not answer."""
    status, body = get(url + "/episodes")
    if status != 200:
        return None
    listing = json.loads(body)
    return [e["id"] for e in listing["live"]] + [e["id"] for e in listing["ended"]]


def verdict_line(run, url, eid):
    """(solved, the one-line summary) of episode eid, its record saved as evidence."""
    status, body = get(f"{url}/api/episode/{eid}")
    folder, base = evidence(run, f"episode-record-{eid}")
    (folder / f"{base}.json").write_text(body)
    if status != 200:
        return False, f"episode {eid}: /api/episode answered {status}"
    doc = json.loads(body)
    v = doc.get("verdict") or {}
    verdict = ("not scored yet" if not v else f"solved={v.get('solved')} claimed={v.get('claimed_success')} "
               f"false_claim={v.get('false_claim')} steps={v.get('steps_used')}/{v.get('budget_steps')}")
    return v.get("solved") is True, (f"episode {eid}: {doc['status']}" + (f" ({doc['reason']})" if doc.get("reason")
                                     else "") + f", {doc['calls']} calls, {doc['events_total']} events; verdict: "
                                     f"{verdict}; saved {folder / base}.json")


def cmd_episode(ns):
    h = harness_dir(ns.harness)
    run, meta = current_run(h)
    ids = [ns.id] if ns.id else episode_ids(meta["url"])
    if not ids:
        print(f"hv: no episode on {meta['url']}" + (" (/episodes does not answer)" if ids is None else ""))
        return 1
    good = True
    for eid in ids:
        solved, line = verdict_line(run, meta["url"], eid)
        print(line)
        good &= solved or not ns.solved
    return 0 if good else 1


def cmd_smoke(ns):
    """bowl-on-plate by a fixed script: one scene look, one put from the bowl to the plate."""
    h = harness_dir(ns.harness)
    run, meta = current_run(h)
    label = "smoke-" + stamp()
    q = query_of(label, "bowl-on-plate")
    hello = rpc(meta["url"], "server/discover", {}, q)
    eid = re.search(r"/view/([A-Za-z0-9_-]+)", (hello.get("result") or {}).get("instructions", ""))
    if eid is None:
        print(f"hv: the episode did not start: {json.dumps(hello)[:300]}")
        return 1
    for tool, args in (("look", {"camera": "agentview"}),
                       ("put", {"pick": {"look": "C1", "px": ns.pick}, "to": {"look": "C1", "px": ns.to},
                                "clear": True})):
        params = {"name": tool, "arguments": args}
        path, images = save_answer(run, tool, params, rpc(meta["url"], "tools/call", params, q))
        print(f"{tool}: saved {path}" + "".join(f"\nimage: {i}" for i in images))
    solved, line = verdict_line(run, meta["url"], eid.group(1))
    print(line)
    return 0 if solved else 1


def leftover_episode_processes(meta, eids):
    # The simulators run in sessions of their own, outside the server's process group.
    try:
        settings = json.loads((Path(meta["harness"]) / "playground" / "settings.json").read_text())
        work = settings["work_root"].replace("{uid}", str(os.getuid()))
    except (OSError, ValueError, KeyError) as exc:
        print(f"hv: no work_root in playground/settings.json ({exc}); leftover processes were not looked for")
        return []
    ps = subprocess.run(["ps", "-axo", "pid=,command="], capture_output=True, text=True).stdout.splitlines()
    left = []
    for line in ps:
        pid, _, command = line.strip().partition(" ")
        if any(f"{work}/{eid}" in command for eid in eids) and int(pid) != os.getpid():
            left.append((int(pid), command[:160]))
    return left


def cmd_down(ns):
    h = harness_dir(ns.harness)
    run, meta = current_run(h, required=False)
    if not meta:
        print("hv: nothing to stop: no server started from this checkout")
        return 0
    eids = episode_ids(meta["url"]) or []
    pid, left, what = meta["pid"], [], "was not running"
    try:
        if alive(pid) and not started_here(meta):
            what = f"is gone (pid {pid} now runs another program, not stopped)"
        elif alive(pid):
            what = "stopped"
            with contextlib.suppress(ProcessLookupError):
                os.killpg(pid, signal.SIGTERM)
            deadline = time.monotonic() + ns.wait
            while alive(pid) and time.monotonic() < deadline:
                time.sleep(0.5)
            if alive(pid):
                print(f"hv: the server did not stop in {ns.wait} s; sending SIGKILL to its group")
                with contextlib.suppress(ProcessLookupError):
                    os.killpg(pid, signal.SIGKILL)
        left = leftover_episode_processes(meta, eids)
        for p, command in left:
            print(f"hv: stopping a leftover process of this run: {p} {command}")
            with contextlib.suppress(ProcessLookupError):
                os.kill(p, signal.SIGTERM)
    finally:
        pointer(h).unlink(missing_ok=True)
    print(f"down: server {pid} {what}, {len(left)} leftover processes stopped; evidence kept in {run}")
    return 0


def started_here(meta):
    """The process at meta's pid is still the server `up` started: its command line names the listen address."""
    command = subprocess.run(["ps", "-o", "command=", "-p", str(meta["pid"])], capture_output=True,
                             text=True).stdout
    return meta["url"].split("//", 1)[1] in command


def cmd_stdio(ns):
    import queue
    import threading
    h = harness_dir(ns.harness)
    run = RUNS / ("stdio-" + stamp())
    (run / "home").mkdir(parents=True)
    argv = ["bash", str(h / "rcp.sh"), "sim"] + (["--task", ns.task] if ns.task else [])
    env = dict(os.environ, CERULION_HOME=str(run / "home"))
    err = open(run / "server.log", "w")
    proc = subprocess.Popen(argv, cwd=run, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err,
                            text=True, bufsize=1, start_new_session=True)
    lines, transcript = queue.Queue(), []

    def pump():
        for line in proc.stdout:
            lines.put(line)
        lines.put(None)

    threading.Thread(target=pump, daemon=True).start()
    good, n = True, 0

    def ask(method, params=None, notify=False):
        nonlocal n, good
        message = {"jsonrpc": "2.0", "method": method, **({"params": params} if params is not None else {})}
        if not notify:
            n += 1
            message["id"] = n
        proc.stdin.write(json.dumps(message) + "\n")
        proc.stdin.flush()
        transcript.append({"sent": message})
        while not notify:
            line = lines.get(timeout=ns.wait)
            if line is None:
                raise RuntimeError("the server closed stdout")
            try:
                doc = json.loads(line)
            except ValueError:
                good = report(False, "stdout carries only MCP messages", f"not JSON: {line[:160]!r}")
                continue
            transcript.append({"got": doc})
            if doc.get("id") == n:
                return doc

    def text(doc):
        return " ".join(b.get("text", "") for b in (doc.get("result") or {}).get("content", []))

    try:
        hello = ask("initialize", {"protocolVersion": "2025-06-18", "capabilities": {},
                                   "clientInfo": {"name": "verify-harness", "version": "0"}})
        ask("notifications/initialized", notify=True)
        good &= report("result" in hello, "initialize", hello.get("result", {}).get("serverInfo", {}).get("name", ""))
        tools = ask("tools/list")
        names = [t["name"] for t in (tools.get("result") or {}).get("tools", [])]
        good &= report("claim" in names, "tools/list starts the episode", f"{len(names)} tools")
        state = ask("tools/call", {"name": "state", "arguments": {}})
        good &= report('"op": "call"' in text(state), "a call reaches the robot", " ".join(text(state).split())[:80])
        claim = ask("tools/call", {"name": "claim", "arguments": {"statement": "stdio check", "success": False}})
        good &= report("has ended" in text(claim), "the claim ends the episode", text(claim)[-160:])
        after = ask("tools/call", {"name": "state", "arguments": {}})
        good &= report('"episode": "ended"' in text(after), "a call after the end is answered ended")
        ask("tools/list")
        again = ask("tools/call", {"name": "state", "arguments": {}})
        good &= report('"op": "call"' in text(again), "listing the tools again starts the next episode")
    except Exception as exc:
        good = report(False, "stdio exchange", f"{type(exc).__name__}: {exc}; see {run / 'server.log'}")
    finally:
        (run / "transcript.json").write_text(json.dumps(transcript, indent=2))
        with contextlib.suppress(BrokenPipeError):
            proc.stdin.close()
        try:
            code = proc.wait(timeout=ns.wait)
        except subprocess.TimeoutExpired:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(proc.pid, signal.SIGTERM)
            try:
                code = proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                code = proc.wait()
        err.close()
    good &= report(code == 0, "the server exits 0 when stdin closes", f"exit {code}")
    print(f"run: {run}")
    return 0 if good else 1


def main(argv=None):
    p = argparse.ArgumentParser(prog="hv.py", description=__doc__.split("\n\n")[0])
    p.add_argument("--harness", help="the harness folder (default: this checkout's harness/, or HV_HARNESS)")
    sub = p.add_subparsers(dest="verb", required=True)
    d = sub.add_parser("doctor")
    d.add_argument("--cli", help="a cerulion binary to check for the rcp verb (default: cerulion on PATH)")
    sub.add_parser("test")
    u = sub.add_parser("up")
    u.add_argument("--task", help="the start state: bowl-on-plate (default), open-drawer or soup-in-basket")
    u.add_argument("--port", type=int, help="the loopback port (default: a free one)")
    u.add_argument("--cli", help="start it through this cerulion binary (`cerulion rcp --sim`) instead of rcp.sh")
    u.add_argument("--wait", type=int, default=600, help="seconds to wait for /health (default 600)")
    for name in ("tools", "call", "read"):
        s = sub.add_parser(name)
        if name == "call":
            s.add_argument("tool")
            s.add_argument("args", nargs="?", help="the tool's arguments as one JSON object")
        if name == "read":
            s.add_argument("uri")
        s.add_argument("--episode", default="a", help="the episode label; calls with one label share a robot")
        s.add_argument("--task", help="the start state of a new episode (default: the server's)")
    e = sub.add_parser("episode")
    e.add_argument("id", nargs="?", help="the episode id (default: every episode of this server)")
    e.add_argument("--solved", action="store_true", help="exit 1 unless every episode listed is solved")
    k = sub.add_parser("smoke")
    k.add_argument("--pick", type=json.loads, default=[195, 152], help="the bowl's pixel in look C1, as JSON")
    k.add_argument("--to", type=json.loads, default=[140, 185], help="the plate's pixel in look C1, as JSON")
    o = sub.add_parser("stdio")
    o.add_argument("--task", help="the start state (default: the server's)")
    o.add_argument("--wait", type=int, default=600, help="seconds to wait for each answer (default 600)")
    w = sub.add_parser("down")
    w.add_argument("--wait", type=int, default=120, help="seconds to let the server score and stop (default 120)")
    ns = p.parse_args(argv)
    verbs = {"doctor": cmd_doctor, "test": cmd_test, "up": cmd_up, "tools": cmd_tools, "call": cmd_call,
             "read": cmd_read, "episode": cmd_episode, "smoke": cmd_smoke, "stdio": cmd_stdio, "down": cmd_down}
    return verbs[ns.verb](ns)


if __name__ == "__main__":
    sys.exit(main())
