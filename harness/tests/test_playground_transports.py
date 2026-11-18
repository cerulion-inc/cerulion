# SPDX-License-Identifier: AGPL-3.0-only
"""The sim server as a program, over stdio and over HTTP, against the fixture snapshot (a stand-in launcher and
bridge: no simulator runs). These need the MCP SDK in the interpreter that runs them (the harness's own venv);
without it they are skipped. The command line's refusals and the connection keep-alive need nothing and always
run."""
import importlib.util
import json
import os
import queue
import signal
import socket
import sqlite3
import subprocess
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.client import HTTPConnection

from playground_support import FIXTURE, HARNESS, FakeSim, Folder, write_settings

from playground.app import AddressError, http_names, keep_alive_s, parse_addr
from playground.pool import AGAIN_HTTP, AGAIN_STDIO, make_pool
from playground.settings import Settings

HAS_MCP = importlib.util.find_spec("mcp") is not None and importlib.util.find_spec("starlette") is not None
SERVER = os.path.join(HARNESS, "playground")
FIRST_CELL = "pro_spatial_swap_t00_i00 libero_pro libero_spatial_swap 0 0 swap vnext all,place"


def initialize(n=1):
    return {"jsonrpc": "2.0", "id": n, "method": "initialize",
            "params": {"protocolVersion": "2025-06-18", "capabilities": {},
                       "clientInfo": {"name": "playground-test", "version": "0"}}}


def call(n, name, arguments=None):
    return {"jsonrpc": "2.0", "id": n, "method": "tools/call", "params": {"name": name, "arguments": arguments or {}}}


def tool_doc(result):
    return json.loads(result["content"][0]["text"])


def answer_of(message, content_type, body):
    """The response to message in a /mcp answer body: one JSON document, or the data lines of an event stream."""
    docs = [json.loads(line[5:]) for line in body.splitlines() if line.startswith("data:")] if \
        content_type.startswith("text/event-stream") else ([json.loads(body)] if body else [])
    return next((d for d in docs if message and d.get("id") == message.get("id")), None)


class Run:
    """One server process with the fixture settings, its log of launcher calls, and its folders."""

    def __init__(self, test, *args, **changes):
        self.folder = Folder()
        test.addCleanup(self.folder.remove)
        self.settings = write_settings(self.folder, **changes)
        self.log = self.folder.at("sim.jsonl")
        env = dict(os.environ, FAKE_SIM_LOG=self.log, PYTHONDONTWRITEBYTECODE="1")
        self.proc = subprocess.Popen([sys.executable, "-B", SERVER, "--settings", self.settings, *args],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True, env=env)
        test.addCleanup(self.kill)
        self.lines = queue.Queue()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            self.lines.put(line)
        self.lines.put(None)

    def kill(self):
        if self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait()
        for stream in (self.proc.stdin, self.proc.stdout, self.proc.stderr):
            stream.close()

    def send(self, message):
        self.proc.stdin.write(json.dumps(message) + "\n")
        self.proc.stdin.flush()

    def answer(self, message, timeout=60):
        """Send a request and return its response (notifications before it are skipped)."""
        self.send(message)
        deadline = time.monotonic() + timeout
        while True:
            line = self.lines.get(timeout=max(0.1, deadline - time.monotonic()))
            if line is None:
                raise AssertionError("the server closed stdout: " + self.proc.stderr.read()[-2000:])
            doc = json.loads(line)
            if doc.get("id") == message["id"]:
                return doc

    def launcher(self):
        with open(self.log, encoding="utf-8") as f:
            return [json.loads(line) for line in f]

    def history(self):
        db = sqlite3.connect(self.folder.at("data", "episodes.sqlite"))
        try:
            return db.execute("SELECT id, state, reason, calls, verdict FROM episodes ORDER BY rowid").fetchall()
        finally:
            db.close()


@unittest.skipUnless(HAS_MCP, "the MCP SDK is not in this interpreter (the harness venv has it)")
class Stdio(unittest.TestCase):
    def test_one_client_runs_an_episode_to_its_claim_and_the_next_listing_starts_another(self):
        run = Run(self)
        init = run.answer(initialize())["result"]
        self.assertEqual(init["serverInfo"]["name"], "cerulion-rcp-sim")
        self.assertIn("Cerulion simulated robot: this connection is one simulated robot episode", init["instructions"])
        self.assertIn("fixture instructions: 1 tool(s); practices: fixture practices", init["instructions"])
        self.assertIn("the simulated robot measured itself once", init["instructions"])
        self.assertIn("https://github.com/cerulion-inc/cerulion", init["instructions"])
        run.send({"jsonrpc": "2.0", "method": "notifications/initialized"})
        tools = run.answer({"jsonrpc": "2.0", "id": 2, "method": "tools/list"})["result"]["tools"]
        self.assertEqual([t["name"] for t in tools], ["look", "claim"])
        look = run.answer(call(3, "look"))["result"]
        self.assertEqual(tool_doc(look), {"ok": True, "frames": ["frames/look001.png"]})
        read = run.answer({"jsonrpc": "2.0", "id": 4, "method": "resources/read", "params": {"uri": "robot://task"}})
        self.assertEqual(read["result"]["contents"][0]["text"], "fixture task")
        claim = run.answer(call(5, "claim", {"success": True, "cannot": None}))["result"]
        self.assertEqual(tool_doc(claim), {"claimed": True, "args": {"success": True}})   # null taken as absent
        # the answer to the claim carries the end and the benchmark's verdict after the tool's own text
        self.assertEqual(len(claim["content"]), 2)
        self.assertTrue(claim["content"][1]["text"].startswith(
            "The episode has ended: the claim was recorded. The benchmark's own success check says the task is "
            "solved. The simulator ran for "), claim["content"][1]["text"])
        self.assertTrue(claim["content"][1]["text"].endswith(AGAIN_STDIO))
        after = run.answer(call(6, "look"))["result"]
        self.assertTrue(after["isError"])
        ended = tool_doc(after)
        self.assertEqual((ended["episode"], ended["reason"]), ("ended", "claim"))
        self.assertEqual(ended["message"], claim["content"][1]["text"])
        again = run.answer({"jsonrpc": "2.0", "id": 7, "method": "tools/list"})["result"]["tools"]
        self.assertEqual(len(again), 2)
        run.proc.stdin.close()                            # the client goes away
        self.assertEqual(run.proc.wait(timeout=60), 0)

        verbs = [(c["verb"], c["argv"][c["argv"].index("--cell") + 1] if c["verb"] == "start" else None)
                 for c in run.launcher()]
        self.assertEqual(verbs, [("start", FIRST_CELL), ("score", None), ("stop", None),
                                 ("start", FIRST_CELL), ("score", None), ("stop", None)])
        start = run.launcher()[0]
        self.assertEqual(start["argv"][start["argv"].index("--budget-requests") + 1], "80")
        self.assertEqual(start["argv"][start["argv"].index("--budget-steps") + 1], "900")
        self.assertEqual(start["env"]["PYTHONPATH"], os.path.realpath(FIXTURE))
        self.assertEqual(start["env"]["OMP_NUM_THREADS"], "1")
        if sys.platform == "darwin":
            self.assertIsNone(start["env"]["MUJOCO_GL"])
            self.assertIn("episode_ext.py", start["env"]["RCPX_EXT_ENVHOST"])
        else:
            self.assertEqual((start["env"]["MUJOCO_GL"], start["env"]["PYOPENGL_PLATFORM"]), ("osmesa", "osmesa"))

        rows = run.history()
        self.assertEqual([(r[1], r[2], r[3]) for r in rows], [("bowl-on-plate", "claim", 2),
                                                              ("bowl-on-plate", "session_closed", 0)])
        self.assertTrue(json.loads(rows[0][4])["solved"])
        first = rows[0][0]
        kept = os.path.join(run.folder.at("data", "episodes"), first)
        self.assertEqual(os.listdir(os.path.join(kept, "frames")), ["look001.png"])
        with open(os.path.join(kept, "events.jsonl"), encoding="utf-8") as f:
            kinds = [json.loads(line)["kind"] for line in f]
        self.assertEqual(kinds, ["status", "status", "status", "call", "result", "read", "call", "claim", "status",
                                 "verdict"])
        self.assertEqual(os.listdir(run.folder.work), [])  # every simulator's files are gone
        self.assertEqual(os.listdir(run.folder.at("records")), ["pro_spatial_swap_t00_i00.json"])

    def test_the_state_flag_picks_the_start_state(self):
        run = Run(self, "--state", "open-drawer")
        run.answer(initialize())
        run.proc.stdin.close()
        self.assertEqual(run.proc.wait(timeout=60), 0)
        start = run.launcher()[0]
        self.assertTrue(start["argv"][start["argv"].index("--cell") + 1].startswith("pro_goal_swap_t00_i00 "))
        self.assertEqual(run.history()[0][1:3], ("open-drawer", "session_closed"))

    def test_a_stop_signal_ends_the_live_episode_as_stopped(self):
        run = Run(self)
        run.answer(initialize())
        run.proc.send_signal(signal.SIGTERM)
        self.assertEqual(run.proc.wait(timeout=60), 0)
        self.assertEqual(run.history()[0][2], "server_stopped")

    def test_a_repeat_of_the_stop_signal_within_a_second_is_the_same_stop(self):
        # A terminal's Ctrl+C reaches cerulion and the server, and cerulion forwards the one it got: the two arrive
        # a moment apart, while the live episode's simulator is still stopping (the fixture's stop takes 2 s).
        os.environ["FAKE_SIM_STOP_S"] = "2"
        self.addCleanup(os.environ.pop, "FAKE_SIM_STOP_S", None)
        run = Run(self)
        run.answer(initialize())
        run.proc.send_signal(signal.SIGINT)
        time.sleep(0.1)
        run.proc.send_signal(signal.SIGINT)
        self.assertEqual(run.proc.wait(timeout=60), 0)
        self.assertEqual(run.history()[0][2], "server_stopped")
        self.assertEqual([c["verb"] for c in run.launcher()], ["start", "score", "stop"])

    def test_a_second_stop_signal_after_a_second_exits_at_once(self):
        os.environ["FAKE_SIM_STOP_S"] = "5"
        self.addCleanup(os.environ.pop, "FAKE_SIM_STOP_S", None)
        run = Run(self)
        run.answer(initialize())
        run.proc.send_signal(signal.SIGINT)
        time.sleep(1.5)
        run.proc.send_signal(signal.SIGINT)
        self.assertEqual(run.proc.wait(timeout=4), 1)
        self.assertEqual(run.history(), [])

    def test_a_simulator_that_does_not_start_is_an_error_with_its_reason(self):
        os.environ["FAKE_SIM_FAIL"] = "start"
        self.addCleanup(os.environ.pop, "FAKE_SIM_FAIL", None)
        run = Run(self)
        err = run.answer(initialize())["error"]
        self.assertIn("The simulator did not start: the simulator did not start: the fixture refuses to start "
                      "scene not found", err["message"])
        run.proc.stdin.close()
        self.assertEqual(run.proc.wait(timeout=60), 0)
        self.assertEqual(run.history()[0][2], "start_failed")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Http:
    def __init__(self, test):
        self.port = free_port()
        self.base = f"http://127.0.0.1:{self.port}"
        self.run = Run(test, "--http", f"127.0.0.1:{self.port}")
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            try:
                with urllib.request.urlopen(self.base + "/health", timeout=2) as r:
                    if r.status == 200:
                        return
            except OSError:
                time.sleep(0.2)
        raise AssertionError("the server did not answer /health: " + self.run.proc.stderr.read()[-2000:])

    def get(self, path):
        try:
            with urllib.request.urlopen(self.base + path, timeout=30) as r:
                return r.status, dict(r.headers), r.read()
        except urllib.error.HTTPError as exc:
            return exc.code, dict(exc.headers), exc.read()

    def mcp(self, message, session=None, host=None, method="POST"):
        headers = {"content-type": "application/json", "accept": "application/json, text/event-stream"}
        if session:
            headers["mcp-session-id"] = session
        if host:
            headers["host"] = host
        req = urllib.request.Request(self.base + "/mcp", data=json.dumps(message).encode() if message else None,
                                     headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                status, got, body = r.status, dict(r.headers), r.read().decode()
        except urllib.error.HTTPError as exc:
            return exc.code, {}, None
        got = {k.lower(): v for k, v in got.items()}
        return status, got, answer_of(message, got.get("content-type", ""), body)

    def mcp_without_session(self, message):
        """POST message to /mcp in the per-request form: the protocol version in the request itself, no session."""
        version = "2026-07-28"
        body = dict(message, params={**message.get("params", {}), "_meta": {
            "io.modelcontextprotocol/protocolVersion": version, "io.modelcontextprotocol/clientCapabilities": {}}})
        headers = {"content-type": "application/json", "accept": "application/json, text/event-stream",
                   "mcp-protocol-version": version, "mcp-method": message["method"]}
        if message["method"] == "tools/call":
            headers["mcp-name"] = message["params"]["name"]
        req = urllib.request.Request(self.base + "/mcp", data=json.dumps(body).encode(), headers=headers,
                                     method="POST")
        with urllib.request.urlopen(req, timeout=60) as r:
            got = {k.lower(): v for k, v in r.headers.items()}
            return r.status, got, answer_of(message, got.get("content-type", ""), r.read().decode())

    def mcp_on(self, conn, message, session=None):
        """POST message to /mcp over the caller's own connection, which stays open for the next request."""
        headers = {"content-type": "application/json", "accept": "application/json, text/event-stream"}
        if session:
            headers["mcp-session-id"] = session
        conn.request("POST", "/mcp", body=json.dumps(message).encode(), headers=headers)
        r = conn.getresponse()
        body, got = r.read().decode(), {k.lower(): v for k, v in r.getheaders()}
        return r.status, got, answer_of(message, got.get("content-type", ""), body)


@unittest.skipUnless(HAS_MCP, "the MCP SDK is not in this interpreter (the harness venv has it)")
class StreamableHttp(unittest.TestCase):
    def test_a_session_runs_an_episode_with_its_viewer_page_and_refuses_another_host(self):
        http = Http(self)
        status, headers, init = http.mcp(initialize())
        self.assertEqual(status, 200)
        session = headers["mcp-session-id"]
        text = init["result"]["instructions"]
        self.assertIn(f"Watch it: {http.base}/view/", text)
        self.assertIn(f" (not private: anyone who can reach this server can watch it, and {http.base}/episodes "
                      "lists every episode).", text)
        eid = text.split("/view/", 1)[1].split(" ", 1)[0]
        http.mcp({"jsonrpc": "2.0", "method": "notifications/initialized"}, session)
        self.assertEqual(tool_doc(http.mcp(call(2, "look"), session)[2]["result"])["frames"], ["frames/look001.png"])
        claim = http.mcp(call(3, "claim", {"success": True}), session)[2]["result"]
        self.assertIn("The benchmark's own success check says the task is solved.", claim["content"][-1]["text"])
        ended = http.mcp(call(4, "look"), session)[2]["result"]
        self.assertTrue(tool_doc(ended)["message"].endswith(AGAIN_HTTP))

        status, headers, page = http.get(f"/view/{eid}")
        self.assertEqual((status, headers.get("x-robots-tag")), (200, "noindex, nofollow"))
        self.assertIn(b'<meta name="robots" content="noindex, nofollow">', page)
        self.assertIn(b"This page is not private: anyone who can reach this server can watch this episode, and "
                      b"<code>/episodes</code> lists every episode.", page)
        self.assertNotIn(b"listed nowhere", page)
        self.assertEqual([e["id"] for e in json.loads(http.get("/episodes")[2])["ended"]], [eid])
        status, _, body = http.get(f"/api/episode/{eid}")
        doc = json.loads(body)
        self.assertEqual((status, doc["status"], doc["reason"], doc["verdict"]["solved"]),
                         (200, "ended", "claim", True))
        self.assertEqual(http.get(f"/frames/{eid}/look001.png")[2], b"fixture picture 1")
        self.assertEqual(http.get(f"/frames/{eid}/..%2Fmeta.json")[0], 404)
        self.assertEqual(http.get("/view/nosuchepisode")[0], 404)
        self.assertIn(http.mcp(initialize(9), host=f"evil.example:{http.port}")[0], (400, 403, 421))
        status, _, home = http.get("/")
        self.assertIn(f"claude mcp add --transport http cerulion-sim {http.base}/mcp".encode(), home)
        self.assertIn(b"These are development start states", home)

        # a second session, closed by the client: its episode ends as session_closed
        status, headers, _ = http.mcp(initialize(10))
        http.mcp(None, headers["mcp-session-id"], method="DELETE")
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            ended = json.loads(http.get("/episodes")[2])["ended"]
            if len(ended) == 2:
                break
            time.sleep(0.2)
        self.assertEqual([e["reason"] for e in ended], ["claim", "session_closed"])

        # a third, live when the server is stopped
        http.mcp(initialize(11))
        health = json.loads(http.get("/health")[2])
        self.assertEqual((health["ok"], health["live_simulators"], health["idle_limit_s"]), (True, 1, 430.2))
        http.run.proc.send_signal(signal.SIGTERM)
        self.assertEqual(http.run.proc.wait(timeout=60), 0)
        self.assertEqual([r[2] for r in http.run.history()], ["claim", "session_closed", "server_stopped"])


@unittest.skipUnless(HAS_MCP, "the MCP SDK is not in this interpreter (the harness venv has it)")
class StreamableHttpSessions(unittest.TestCase):
    def open_session(self, http, n):
        status, headers, init = http.mcp(initialize(n))
        self.assertEqual(status, 200)
        session = headers["mcp-session-id"]
        http.mcp({"jsonrpc": "2.0", "method": "notifications/initialized"}, session)
        return session, init["result"]["instructions"].split("/view/", 1)[1].split(" ", 1)[0]

    def test_two_sessions_on_the_default_url_each_drive_their_own_robot(self):
        http = Http(self)
        (a, first), (b, second) = self.open_session(http, 1), self.open_session(http, 2)
        self.assertNotEqual(first, second)
        self.assertEqual(json.loads(http.get("/health")[2])["live_simulators"], 2)
        self.assertEqual(tool_doc(http.mcp(call(3, "look"), a)[2]["result"])["frames"], ["frames/look001.png"])
        self.assertEqual(tool_doc(http.mcp(call(4, "look"), b)[2]["result"])["frames"], ["frames/look001.png"])
        claim = http.mcp(call(5, "claim", {"success": True}), a)[2]["result"]
        self.assertTrue(claim["content"][-1]["text"].startswith("The episode has ended: the claim was recorded."))
        # the claim ended the first session's episode only: the second session's robot runs on
        self.assertEqual(tool_doc(http.mcp(call(6, "look"), b)[2]["result"]),
                         {"ok": True, "frames": ["frames/look002.png"]})
        self.assertTrue(tool_doc(http.mcp(call(7, "look"), a)[2]["result"])["message"].endswith(AGAIN_HTTP))
        episodes = json.loads(http.get("/episodes")[2])
        self.assertEqual([e["id"] for e in episodes["live"]], [second])
        self.assertEqual([(e["id"], e["reason"]) for e in episodes["ended"]], [(first, "claim")])
        self.assertEqual([c["verb"] for c in http.run.launcher()], ["start", "start", "score", "stop"])

    def test_a_client_without_a_session_never_reaches_a_sessions_episode(self):
        http = Http(self)
        session, eid = self.open_session(http, 1)
        self.assertEqual(tool_doc(http.mcp(call(2, "look"), session)[2]["result"])["frames"], ["frames/look001.png"])
        status, _, listed = http.mcp_without_session({"jsonrpc": "2.0", "id": 3, "method": "tools/list"})
        self.assertEqual((status, [t["name"] for t in listed["result"]["tools"]]), (200, ["look", "claim"]))
        status, _, look = http.mcp_without_session(call(4, "look"))
        self.assertEqual((status, tool_doc(look["result"])), (200, {"ok": True, "frames": ["frames/look001.png"]}))
        live = [e["id"] for e in json.loads(http.get("/episodes")[2])["live"]]
        self.assertEqual((len(live), live[0]), (2, eid))
        self.assertEqual([c["verb"] for c in http.run.launcher()], ["start", "start"])


@unittest.skipUnless(HAS_MCP, "the MCP SDK is not in this interpreter (the harness venv has it)")
class StreamableHttpKeptConnection(unittest.TestCase):
    def test_a_client_reuses_one_connection_after_a_turn_longer_than_five_seconds(self):
        # uvicorn closes a connection idle for 5 s unless told otherwise. A model's turn between two calls can take
        # longer, and an MCP client, or a forwarder in front of the server, sends the next call over the
        # connection it already holds.
        http = Http(self)
        conn = HTTPConnection("127.0.0.1", http.port, timeout=60)
        self.addCleanup(conn.close)
        status, headers, init = http.mcp_on(conn, initialize())
        self.assertEqual(status, 200)
        session = headers["mcp-session-id"]
        self.assertEqual(http.mcp_on(conn, {"jsonrpc": "2.0", "method": "notifications/initialized"}, session)[0],
                         202)
        self.assertIsNotNone(conn.sock)
        local = conn.sock.getsockname()
        time.sleep(6)
        status, _, look = http.mcp_on(conn, call(2, "look"), session)
        self.assertEqual(status, 200)
        self.assertIsNotNone(conn.sock)
        self.assertEqual(conn.sock.getsockname(), local)              # the same connection, not a new one
        self.assertEqual(tool_doc(look["result"]), {"ok": True, "frames": ["frames/look001.png"]})


@unittest.skipUnless(HAS_MCP, "the MCP SDK is not in this interpreter (the harness venv has it)")
class StreamableHttpStop(unittest.TestCase):
    def test_a_repeat_of_the_stop_signal_within_a_second_is_the_same_stop(self):
        os.environ["FAKE_SIM_STOP_S"] = "2"
        self.addCleanup(os.environ.pop, "FAKE_SIM_STOP_S", None)
        http = Http(self)
        http.mcp(initialize())
        http.run.proc.send_signal(signal.SIGINT)
        time.sleep(0.1)
        http.run.proc.send_signal(signal.SIGINT)
        self.assertEqual(http.run.proc.wait(timeout=60), 0)
        self.assertEqual([r[2] for r in http.run.history()], ["server_stopped"])


class CommandLine(unittest.TestCase):
    def setUp(self):
        self.folder = Folder()
        self.addCleanup(self.folder.remove)

    def run_cli(self, *args):
        p = subprocess.run([sys.executable, "-B", SERVER, *args], capture_output=True, text=True, timeout=60,
                           env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
        return p.returncode, p.stdout, p.stderr

    def test_check_reports_ready_and_what_is_missing(self):
        rc, out, _ = self.run_cli("--check", "--settings", write_settings(self.folder))
        self.assertEqual((rc, json.loads(out)["ready"]), (0, True))
        rc, out, _ = self.run_cli("--check", "--settings", write_settings(self.folder, bench_root=self.folder.at("x")))
        doc = json.loads(out)
        self.assertEqual((rc, doc["ready"], doc["missing"][0]["what"]), (1, False, "the simulator and its scenes"))

    def test_serving_without_the_install_exits_1_and_says_what_installs_it(self):
        rc, _, err = self.run_cli("--settings", write_settings(self.folder, bench_root=self.folder.at("x")))
        self.assertEqual(rc, 1)
        self.assertIn("installs the simulator and its scenes on its first run", err)

    def test_usage_and_settings_errors_exit_2(self):
        path = write_settings(self.folder)
        for args, needle in ((("--http", "8765"), "is not HOST:PORT"), (("--state", "kitchen"), "'kitchen' is not "
                             "offered"), (("--stdio", "--http"), "not allowed with argument"),
                             (("--settings", self.folder.at("none.json")), "cannot read")):
            rc, _, err = self.run_cli("--settings", path, *args) if args[0] != "--settings" else self.run_cli(*args)
            self.assertEqual(rc, 2, args)
            self.assertIn(needle, err)

    def test_a_sealed_start_state_is_refused_before_serving(self):
        os.environ["FAKE_SEALED"] = "pro_object_swap_t00_i00"
        self.addCleanup(os.environ.pop, "FAKE_SEALED", None)
        rc, out, _ = self.run_cli("--check", "--settings", write_settings(self.folder))
        self.assertEqual((rc, json.loads(out)["ready"]), (2, False))
        self.assertIn("'soup-in-basket'", json.loads(out)["problem"])


class KeepAlive(unittest.TestCase):
    def setUp(self):
        self.folder = Folder()
        self.addCleanup(self.folder.remove)

    def pool(self, **changes):
        return make_pool(Settings(write_settings(self.folder, **changes), platform="linux"), FakeSim())

    def test_an_idle_connection_is_kept_for_the_idle_limit_rounded_up_to_a_whole_second(self):
        pool = self.pool()
        self.assertEqual(pool.idle_limit()[0], 430.2)                          # 10 x 43.02 s, the shipped rule
        self.assertEqual(keep_alive_s(pool), 431)
        pool = self.pool(idle_s=4.5)
        self.assertEqual(keep_alive_s(pool), 5)
        pool = self.pool(idle_s=60)
        self.assertEqual(keep_alive_s(pool), 60)

    def test_the_keep_alive_reads_the_idle_limit_a_longer_history_gives(self):
        pool = self.pool()
        pool.s.idle_reap = dict(pool.s.idle_reap, gaps_counted=3)
        pool.history.record({"id": "ep000001", "state": "x", "verdict": None}, [10.0, 20.0, 30.0, 50.0])
        pool.gaps_changed()
        self.assertEqual(pool.idle_limit()[0], 500.0)
        self.assertEqual(keep_alive_s(pool), 500)

    def test_with_no_idle_limit_an_idle_connection_is_kept_for_the_episode_clock(self):
        pool = self.pool(idle_reap=None)
        self.assertEqual(pool.idle_limit(), (None, "no idle limit is set"))
        self.assertEqual(keep_alive_s(pool), 5400)
        pool = self.pool(idle_reap=None, clock_s=1200.5)
        self.assertEqual(keep_alive_s(pool), 1201)


class Addresses(unittest.TestCase):
    def test_addresses_parse_with_ipv6_in_brackets(self):
        self.assertEqual(parse_addr("127.0.0.1:8765"), ("127.0.0.1", 8765))
        self.assertEqual(parse_addr("localhost:9000"), ("localhost", 9000))
        self.assertEqual(parse_addr("[::1]:8765"), ("::1", 8765))
        self.assertEqual(parse_addr("0.0.0.0:80"), ("0.0.0.0", 80))
        for bad in ("8765", "host:", ":8765", "host:0", "host:65536", "::1:8765", "[::1]8765", "", "host:x"):
            with self.assertRaises(AddressError, msg=bad):
                parse_addr(bad)

    def test_a_loopback_server_answers_every_loopback_name_and_another_only_its_own(self):
        self.assertEqual(http_names("127.0.0.1", 8765), ("http://127.0.0.1:8765",
                                                         ["127.0.0.1:8765", "localhost:8765", "[::1]:8765"]))
        self.assertEqual(http_names("::1", 8765)[0], "http://[::1]:8765")
        self.assertEqual(http_names("0.0.0.0", 8765), ("http://0.0.0.0:8765", ["0.0.0.0:8765"]))
        self.assertEqual(http_names("192.0.2.20", 80), ("http://192.0.2.20:80", ["192.0.2.20:80"]))


if __name__ == "__main__":
    unittest.main()
