# SPDX-License-Identifier: AGPL-3.0-only
"""A whole run-cell run, with a stand-in for the episode driver (tests/fixtures/fake_driver) and for the model
endpoint (run_cell_support.FakeEndpoint): the request bodies each route's endpoint receives, the lines it prints,
summary.json, the scratch folder, and what an interrupt or an error leaves behind. Standard library only: no
simulator and no network. The endpoint is reached as a named host (Google's or OpenRouter's) by being the HTTP
proxy, and every expected body is written out by hand below."""
import contextlib
import io
import json
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from cerulion_harness import body_record, cli, layout, progress  # noqa: E402
from run_cell_support import (FIXTURES, KEY, MODEL, FakeEndpoint, block_signals, keep_redirects,  # noqa: E402
                              unignore_signals)

CELL = "pro_spatial_swap_t00_i00"
T = "2026-11-18T16:00:00Z"
GOOGLE = "http://aiplatform.googleapis.com/v1/projects/p/locations/global/endpoints/openapi"
OPENROUTER = "http://openrouter.ai/api/v1"
HEAD = (r'{"model": "google/gemini-3.8-flash", "messages": [{"role": "system", "content": "You operate a robot."}, '
        r'{"role": "user", "content": "Task: pick the bowl."}')
TAIL = (r'"tools": [{"type": "function", "function": {"name": "move_to", "parameters": {"type": "object"}}}], '
        r'"tool_choice": "auto", "temperature": 1.0, "max_tokens": 32000, "seed": 1, "parallel_tool_calls": false, '
        r'"reasoning_effort": "high"')
TOOL = r'{"role": "tool", "tool_call_id": "call_1", "content": "moved"}'
CALL = r'{"id": "call_1", "type": "function", "function": {"name": "move_to", "arguments": "{\"x\": 0.1}"}'
# Google's endpoint: the bodies of the measured runs, the signature carried in extra_content.
GOOGLE_BODIES = [
    (HEAD + "], " + TAIL + "}").encode(),
    (HEAD + r', {"role": "assistant", "content": "", "tool_calls": [' + CALL + r', "extra_content": {"google": '
     r'{"thought_signature": "c2lnLTE="}}}]}, ' + TOOL + "], " + TAIL + "}").encode(),
]
# OpenRouter: the provider pin, and the first reply's blocks back on its assistant message.
OPENROUTER_BODIES = [
    (HEAD + "], " + TAIL + r', "provider": {"only": ["google-vertex"]}}').encode(),
    (HEAD + r', {"role": "assistant", "content": "", "tool_calls": [' + CALL + r'}], "reasoning_details": [{"type": '
     r'"reasoning.text", "text": "Moving above the bowl.", "format": "google-gemini-v1", "index": 0}, {"type": '
     r'"reasoning.encrypted", "data": "QUJDRA==", "format": "google-gemini-v1", "id": "call_1", "index": 1}]}, '
     + TOOL + "], " + TAIL + r', "provider": {"only": ["google-vertex"]}}').encode(),
]
DRIVER = str(FIXTURES / "fake_driver")
SRC = str(pathlib.Path(__file__).resolve().parent.parent / "src")


def tmp_names():
    return {n for n in os.listdir("/tmp") if n.startswith("cerulion-bench-")}


def signal_inside_popen(test, sig):
    """For the length of a test: the first subprocess.Popen started sends sig to this process once its child exists
    and before Popen returns, as a signal that arrives while a child starts does. Returns the list that gets that
    Popen. A child still running 30 s later is killed with its group, so a run that lost the signal ends and the
    test fails instead of waiting for the child; one left running at the end of the test is killed then."""
    started, real = [], subprocess.Popen

    def popen(*a, **kw):
        proc = real(*a, **kw)
        if not started:
            started.append(proc)
            watchdog = threading.Timer(30, kill_group, (proc,))
            watchdog.daemon = True
            watchdog.start()
            test.addCleanup(end_group, proc)
            test.addCleanup(watchdog.cancel)
            os.kill(os.getpid(), sig)
        return proc

    patch = mock.patch.object(subprocess, "Popen", popen)
    patch.start()
    test.addCleanup(patch.stop)
    return started


def kill_group(proc):
    if proc.returncode is None:
        with contextlib.suppress(ProcessLookupError, PermissionError):
            os.killpg(proc.pid, signal.SIGKILL)


def end_group(proc):
    if proc.poll() is None:
        kill_group(proc)
        proc.wait()


class Bench(unittest.TestCase):
    """One run-cell run in a folder of its own, with the install, the trees and the clock stood in for."""

    kind = "google"
    url = GOOGLE

    def setUp(self):
        unignore_signals(self)
        keep_redirects(self)   # run-cell makes this process's urllib follow no redirect
        self.base = tempfile.mkdtemp(dir="/tmp")
        self.addCleanup(shutil.rmtree, self.base, True)
        self.end = FakeEndpoint(self.kind)
        self.addCleanup(self.end.close)
        sim = os.path.join(self.base, "sim-python")
        open(sim, "w").close()
        self.env = {"TEST_KEY": KEY, "http_proxy": self.end.url, "HTTP_PROXY": self.end.url,
                    "PATH": os.environ.get("PATH", ""), "HOME": os.environ.get("HOME", self.base),
                    "FAKE_DRIVER_MODE": "episode"}
        for patch in (mock.patch.dict(os.environ, self.env, clear=True),
                      mock.patch.object(layout, "SIM_PYTHON", sim),
                      mock.patch.object(layout, "VENV_PYTHON", os.path.join(sys.prefix, "bin", "python")),
                      mock.patch.object(layout, "RCP_TREE", DRIVER),
                      mock.patch.object(layout, "plain_tree", return_value=DRIVER),
                      mock.patch.object(body_record, "checked", return_value=True),
                      mock.patch.object(progress, "utc_now", return_value=T)):
            patch.start()
            self.addCleanup(patch.stop)
        self.cwd = os.getcwd()
        os.chdir(self.base)
        self.addCleanup(os.chdir, self.cwd)
        self.before = tmp_names()

    def run_cell(self, *extra, out="out"):
        self.stdout, self.stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(self.stdout), contextlib.redirect_stderr(self.stderr):
            return cli.main(["run-cell", "--model", MODEL, "--base-url", self.url, "--key-env", "TEST_KEY",
                             "--out", out, "--no-endpoint-check", *extra])

    def summary(self, out="out"):
        with open(os.path.join(out, "summary.json"), encoding="utf-8") as f:
            return json.load(f)

    def scratch_line(self):
        lines = [x for x in self.stdout.getvalue().splitlines() if x.startswith("  scratch folder: ")]
        self.assertEqual(len(lines), 1, self.stdout.getvalue())
        return lines[0][len("  scratch folder: "):].replace(" (removed at the end)", "")

    def assert_nothing_left(self):
        self.assertFalse(os.path.exists(self.scratch_line()))
        self.assertEqual(tmp_names(), self.before)

    def relays(self):
        """Each row's relay record, with the loopback address it was reached at checked and taken out."""
        rows = []
        for row in self.summary()["rows"]:
            relay = dict(row["relay"])
            self.assertRegex(relay.pop("base_url"), r"\Ahttp://127\.0\.0\.1:[1-9][0-9]*/v1\Z")
            rows.append(relay)
        return rows


class GoogleEndpoint(Bench):
    def test_the_endpoint_receives_the_bodies_of_the_measured_runs(self):
        self.assertEqual(self.run_cell(), 0)
        self.assertEqual([s[2] for s in self.end.seen], GOOGLE_BODIES * 2)
        self.assertEqual({s[:2] for s in self.end.seen}, {(GOOGLE + "/chat/completions", "Bearer " + KEY)})

    def test_it_prints_the_route_the_expectation_one_line_per_reply_and_the_summary_path(self):
        self.run_cell()
        lines = self.stdout.getvalue().splitlines()
        self.assertEqual(lines[:3], [
            f"{T} start: {CELL} with {MODEL}, setup plain then rcp; results in out",
            '  route aiplatform.googleapis.com: every request body adds {"reasoning_effort": "high"} (the default '
            'for this route); reasoning blocks a reply carries are sent back on the assistant message whose tool '
            "calls they came with, outside the operator's context estimate",
            "  expected from the measured runs of gemini-3.8-flash: plain 846.644 operator seconds, 55.87 replies, "
            "1.6665 USD at list price (mean of 100 scored episodes); rcp 258.659 operator seconds, 25.55 replies, "
            "0.3479 USD at list price (mean of 100 scored episodes); an episode stops at 80 replies, 900 control "
            "steps or 5400 s"])
        self.assertTrue(lines[3].endswith(" (removed at the end)"), lines[3])
        # gemini-3.8-flash: 1,000 x 0.75 + 200 x 3.75 = 0.0015; then 1,000 x 0.75 + 1,000 x 0.075 + 100 x 3.75
        # = 0.0012, so far 0.0027. Google's endpoint reports no cost of its own.
        per_reply = [
            f"{T} {{arm}} reply 1: control step 30 of 900; tokens in 1000 (0 cached), out 200 (150 reasoning); so "
            f"far 0.0015 USD at list price",
            f"{T} {{arm}} reply 2: control step 42 of 900; tokens in 2000 (1000 cached), out 100 (50 reasoning); so "
            f"far 0.0027 USD at list price"]
        verdict = [
            "  {arm}  solved: True  within the published step limit: True  false claim: False",
            "         replies: 2  control steps: 42  operator seconds: 1.5",
            "         cost: 0.0027 USD at list price (the package's price table of 2026-11-17)"]
        want = []
        for arm in ("plain", "rcp"):
            want += [f"{T} running {arm} on {CELL} with {MODEL} ..."] + [x.format(arm=arm) for x in per_reply]
            want += [x.format(arm=f"{arm:5s}") for x in verdict]
        self.assertEqual(lines[4:], want + ["summary: out/summary.json"])
        self.assertEqual(self.stderr.getvalue(), "")

    def test_summary_json_records_the_route_and_the_relay(self):
        self.run_cell()
        doc = self.summary()
        self.assertEqual(doc["route"], {
            "base_url": GOOGLE, "host": "aiplatform.googleapis.com", "extra_body": {"reasoning_effort": "high"},
            "extra_body_from": "the default for this route", "provider_only": None, "reasoning_effort": "high",
            "reasoning_blocks": "sent back on the assistant message whose tool calls they came with, outside the "
                                "operator's context estimate"})
        self.assertEqual(self.relays(), [{"requests": 2, "not_reached": 0, "replies_with_reasoning_blocks": 0,
                                          "requests_carrying_blocks": 0}] * 2)
        self.assertEqual([(r["arm"], r["scored"], r["list_price_usd"], r["reported_usd"]) for r in doc["rows"]],
                         [("plain", True, 0.0027, None), ("rcp", True, 0.0027, None)])

    def test_the_default_scratch_folder_is_under_tmp_and_is_removed(self):
        # An --out too long to hold the episode's socket paths: the scratch folder does not depend on --out.
        out = os.path.join(self.base, "o" * 60)
        self.assertEqual(self.run_cell("--arm", "plain", out=out), 0)
        self.assertTrue(self.scratch_line().startswith("/tmp/cerulion-bench-"), self.scratch_line())
        self.assert_nothing_left()

    def test_a_given_scratch_folder_is_used_and_kept(self):
        work = os.path.join(self.base, "w")
        self.assertEqual(self.run_cell("--arm", "plain", "--work", work), 0)
        self.assertEqual(self.scratch_line(), work)
        self.assertNotIn("(removed at the end)", self.stdout.getvalue())
        self.assertTrue(os.path.isdir(os.path.join(work, "p", "run_without", f"{CELL}_a1")))

    def test_two_runs_send_the_same_bytes_and_print_the_same_lines(self):
        self.run_cell("--arm", "plain", out="a")
        first = (list(self.end.seen), [x for x in self.stdout.getvalue().splitlines() if "scratch" not in x])
        self.end.seen.clear()
        self.run_cell("--arm", "plain", out="b")
        second = (list(self.end.seen), [x.replace("results in b", "results in a").replace("b/summary", "a/summary")
                                        for x in self.stdout.getvalue().splitlines() if "scratch" not in x])
        self.assertEqual(first, second)


class OpenRouter(Bench):
    kind = "openrouter"
    url = OPENROUTER

    def test_the_endpoint_receives_the_pin_and_the_blocks_back(self):
        self.assertEqual(self.run_cell(), 0)
        self.assertEqual([s[2] for s in self.end.seen], OPENROUTER_BODIES * 2)
        self.assertEqual({s[0] for s in self.end.seen}, {OPENROUTER + "/chat/completions"})

    def test_the_scratch_folder_is_removed_and_the_out_folder_holds_only_results(self):
        self.run_cell("--arm", "plain")
        self.assertTrue(self.scratch_line().startswith("/tmp/cerulion-bench-"), self.scratch_line())
        self.assert_nothing_left()
        self.assertEqual(sorted(os.listdir("out")), ["cell.txt", "plain", "summary.json"])

    def test_lines_and_summary_carry_the_endpoints_own_cost(self):
        self.run_cell("--arm", "rcp")
        lines = self.stdout.getvalue().splitlines()
        self.assertIn('  route openrouter.ai: every request body adds {"reasoning_effort": "high", "provider": '
                      '{"only": ["google-vertex"]}} (the default for this route); reasoning blocks a reply carries '
                      "are sent back on the assistant message whose tool calls they came with, outside the "
                      "operator's context estimate", lines)
        self.assertIn(f"{T} rcp reply 2: control step 42 of 900; tokens in 2000 (1000 cached), out 100 (50 "
                      f"reasoning); so far 0.0027 USD at list price, 0.0028 USD reported by the endpoint", lines)
        self.assertIn("         cost: 0.0027 USD at list price (the package's price table of 2026-11-17); 0.0028 USD "
                      "reported by the endpoint (usage.cost of 2 of 2 replies)", lines)
        doc = self.summary()
        self.assertEqual(doc["route"]["provider_only"], ["google-vertex"])
        self.assertEqual(self.relays(), [{"requests": 2, "not_reached": 0, "replies_with_reasoning_blocks": 2,
                                          "requests_carrying_blocks": 1}])

    def test_a_given_extra_body_is_sent_as_given(self):
        self.run_cell("--arm", "plain", "--extra-body", '{"reasoning_effort": "high"}')
        self.assertEqual(self.end.seen[0][2], (HEAD + "], " + TAIL + "}").encode())
        self.assertEqual(self.summary()["route"]["extra_body_from"], "--extra-body")


class Interrupts(Bench):
    """A signal during an episode: the driver's group ends, the episode is stopped, the scratch folder goes."""

    def setUp(self):
        super().setUp()
        self.ready = os.path.join(self.base, "ready.json")
        self.mark = os.path.join(self.base, "stopped.txt")
        os.environ.update(FAKE_DRIVER_MODE="hang", FAKE_DRIVER_READY=self.ready, FAKE_STOP_MARK=self.mark)

    def send_when_ready(self, sig, ready):
        def send():
            block_signals()
            deadline = time.monotonic() + 30
            while not os.path.exists(ready) and time.monotonic() < deadline:
                time.sleep(0.02)
            os.kill(os.getpid(), sig)

        t = threading.Thread(target=send, daemon=True)
        t.start()
        return t

    def gone(self, pid):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                return True
            time.sleep(0.05)
        return False

    def kill(self, pid):
        with contextlib.suppress(ProcessLookupError):
            os.kill(pid, signal.SIGKILL)

    def interrupt(self, sig):
        before = signal.getsignal(sig)
        sender = self.send_when_ready(sig, self.ready)
        code = self.run_cell("--arm", "plain")
        sender.join(5)
        self.assertEqual(signal.getsignal(sig), before)
        with open(self.ready, encoding="utf-8") as f:
            return code, json.load(f)

    def check(self, sig, code, info):
        name = signal.Signals(sig).name
        self.assertEqual(code, 128 + sig)
        self.assertTrue(self.gone(info["driver"]), "the driver still runs")
        self.assertTrue(self.gone(info["server"]), "the episode server still runs")
        with open(self.mark, encoding="utf-8") as f:
            self.assertEqual(f.read().splitlines(), [info["ep"]])
        self.assert_nothing_left()
        self.assertEqual(self.summary()["interrupted"], name)
        self.assertIs(self.summary()["running_episode_stopped"], True)
        self.assertEqual(self.summary()["rows"], [])
        self.assertEqual(self.stderr.getvalue(), f"{T} stopped by {name}: the running episode was ended and the "
                                                 f"scratch folder removed; out/summary.json holds the setups that "
                                                 f"finished\n")

    def test_sigint(self):
        self.check(signal.SIGINT, *self.interrupt(signal.SIGINT))

    def test_sigterm(self):
        self.check(signal.SIGTERM, *self.interrupt(signal.SIGTERM))

    def test_sighup(self):
        self.check(signal.SIGHUP, *self.interrupt(signal.SIGHUP))

    def test_a_stop_that_cannot_end_its_episode_is_given_up_within_the_window(self):
        os.environ["FAKE_STOP_HANG"] = "1"
        sender = self.send_when_ready(signal.SIGINT, self.ready)
        t0 = time.monotonic()
        with mock.patch.object(cli, "DRIVER_GRACE_S", 1.0), mock.patch.object(cli, "STOP_WINDOW_S", 3.0):
            code = self.run_cell("--arm", "plain")
        took = time.monotonic() - t0
        sender.join(5)
        with open(self.ready, encoding="utf-8") as f:
            info = json.load(f)
        self.addCleanup(self.kill, info["server"])   # the stand-in server the stop could not end
        self.assertEqual(code, 130)
        self.assertLess(took, 3.0 + 5.0)
        self.assertTrue(self.gone(info["driver"]), "the driver still runs")
        self.assertFalse(os.path.exists(self.mark))
        self.assert_nothing_left()
        self.assertIs(self.summary()["running_episode_stopped"], False)
        self.assertEqual(self.stderr.getvalue(), f"{T} stopped by SIGINT: the running episode did not stop within "
                                                 f"3 s, so its simulator may still be running; the scratch folder was "
                                                 f"removed; out/summary.json holds the setups that finished\n")

    def test_a_stop_that_says_something_still_runs_is_reported(self):
        os.environ["FAKE_STOP_EXIT"] = "1"
        t0 = time.monotonic()
        code, info = self.interrupt(signal.SIGTERM)
        self.assertLess(time.monotonic() - t0, cli.STOP_WINDOW_S)   # it said so at once: no time ran out
        self.assertEqual(code, 143)
        self.assertTrue(self.gone(info["server"]), "the episode server still runs")
        self.assert_nothing_left()
        self.assertIs(self.summary()["running_episode_stopped"], False)
        self.assertEqual(self.stderr.getvalue(), f"{T} stopped by SIGTERM: the episode's stop command reported part "
                                                 f"of it still running, so its simulator may still be running; the "
                                                 f"scratch folder was removed; out/summary.json holds the setups "
                                                 f"that finished\n")

    def test_a_stop_command_that_cannot_be_started_is_reported(self):
        sender = self.send_when_ready(signal.SIGINT, self.ready)
        with mock.patch.object(cli.subprocess, "run", side_effect=FileNotFoundError(2, "No such file")):
            code = self.run_cell("--arm", "plain")
        sender.join(5)
        with open(self.ready, encoding="utf-8") as f:
            info = json.load(f)
        self.addCleanup(self.kill, info["server"])   # the stand-in server no stop reached
        self.assertEqual(code, 130)
        self.assertTrue(self.gone(info["driver"]), "the driver still runs")
        self.assertFalse(os.path.exists(self.mark))
        self.assert_nothing_left()
        self.assertIs(self.summary()["running_episode_stopped"], False)
        self.assertEqual(self.stderr.getvalue(), f"{T} stopped by SIGINT: the episode's stop command could not be "
                                                 f"started, so its simulator may still be running; the scratch folder "
                                                 f"was removed; out/summary.json holds the setups that finished\n")

    def test_a_signal_that_arrives_while_the_driver_starts_still_stops_it(self):
        # The signal arrives inside Popen, once the child exists: run-cell still has the child, and ends it.
        os.environ["FAKE_DRIVER_MODE"] = "episode"   # starts nothing of its own that could outlive it
        started = signal_inside_popen(self, signal.SIGTERM)
        self.assertEqual(self.run_cell("--arm", "plain"), 143)
        self.assertEqual(len(started), 1)
        self.assertIsNotNone(started[0].returncode, "the driver still runs")
        self.assertEqual(self.summary()["interrupted"], "SIGTERM")
        self.assert_nothing_left()

    def test_a_signal_before_any_episode_starts(self):
        with mock.patch.object(layout, "plain_tree", side_effect=cli.Interrupted(signal.SIGINT)):
            self.assertEqual(self.run_cell("--arm", "plain"), 130)
        self.assertEqual(self.stderr.getvalue(), f"{T} stopped by SIGINT: no episode had started\n")
        self.assertFalse(os.path.exists("out"))
        self.assertEqual(tmp_names(), self.before)


class IgnoredSignals(Bench):
    """A signal run-cell starts with ignored stays ignored: that signal during an episode changes nothing, and the
    run goes on to its verdict."""

    def go_on_through(self, sig):
        old = signal.signal(sig, signal.SIG_IGN)
        self.addCleanup(signal.signal, sig, old)
        self.end.gate = threading.Event()
        sent = []

        def send():
            block_signals()
            deadline = time.monotonic() + 30
            while not self.end.seen and time.monotonic() < deadline:
                time.sleep(0.01)
            os.kill(os.getpid(), sig)
            sent.append(len(self.end.seen))   # the first request, still unanswered
            self.end.gate.set()

        sender = threading.Thread(target=send, daemon=True)
        sender.start()
        code = self.run_cell("--arm", "plain")
        sender.join(5)
        self.assertEqual((code, sent, len(self.end.seen)), (0, [1], 2))
        self.assertIs(signal.getsignal(sig), signal.SIG_IGN)
        self.assertEqual(self.stderr.getvalue(), "")
        self.assertNotIn("interrupted", self.summary())
        self.assertEqual([(r["arm"], r["scored"]) for r in self.summary()["rows"]], [("plain", True)])
        self.assertEqual(self.stdout.getvalue().splitlines()[-1], "summary: out/summary.json")
        self.assert_nothing_left()

    def test_sighup_under_nohup(self):
        self.go_on_through(signal.SIGHUP)

    def test_sigint_in_a_background_job_of_a_script(self):
        self.go_on_through(signal.SIGINT)


class Errors(Bench):
    def test_an_error_after_the_episode_still_removes_the_scratch_folder(self):
        with mock.patch.object(cli, "read_result", side_effect=RuntimeError("disk")):
            with self.assertRaises(RuntimeError):
                self.run_cell("--arm", "plain")
        self.assert_nothing_left()

    def test_signals_are_restored_after_a_refusal(self):
        before = {s: signal.getsignal(s) for s in cli.Signals.CAUGHT}
        with mock.patch.dict(os.environ, {"TEST_KEY": ""}):
            self.assertEqual(self.run_cell("--arm", "plain"), 2)
        self.assertIn("no key", self.stderr.getvalue())
        self.assertEqual({s: signal.getsignal(s) for s in cli.Signals.CAUGHT}, before)


class BodyRecord(Bench):
    """The Cerulion harness setup's robot body record, measured by a process of its own before the episode."""

    def setUp(self):
        super().setUp()
        for patch in (mock.patch.object(body_record, "checked", return_value=False),
                      mock.patch.object(layout, "SRC", str(FIXTURES / "fake_src"))):
            patch.start()
            self.addCleanup(patch.stop)

    def test_a_found_record_lets_the_episode_run_and_is_kept(self):
        os.environ["FAKE_BODY_MODE"] = "ok"
        self.assertEqual(self.run_cell("--arm", "rcp"), 0)
        with open(os.path.join("out", "body_record.json"), encoding="utf-8") as f:
            self.assertEqual(json.load(f)["found"], 1)
        self.assertEqual(len(self.end.seen), 2)

    def test_it_runs_without_writing_bytecode(self):
        # It imports the measured code into its own process: with bytecode on, that writes __pycache__ folders into
        # the harness folder. This test's environment sets nothing of the kind itself.
        os.environ["FAKE_BODY_MODE"] = "ok"
        self.assertNotIn("PYTHONDONTWRITEBYTECODE", os.environ)
        self.assertEqual(self.run_cell("--arm", "rcp"), 0)
        with open(os.path.join("out", "body_record.json"), encoding="utf-8") as f:
            self.assertIs(json.load(f)["dont_write_bytecode"], True)

    def test_a_failed_record_stops_before_the_episode_with_its_problem(self):
        os.environ["FAKE_BODY_MODE"] = "fail"
        self.assertEqual(self.run_cell("--arm", "rcp"), 2)
        self.assertIn("the simulator did not start", self.stderr.getvalue())
        self.assertEqual(self.end.seen, [])
        self.assert_nothing_left()

    def test_an_answer_that_is_not_its_json_names_the_exit_code(self):
        os.environ["FAKE_BODY_MODE"] = "garbage"
        self.assertEqual(self.run_cell("--arm", "rcp"), 2)
        self.assertIn("python -m cerulion_harness.body_record ended with 1", self.stderr.getvalue())
        self.assertEqual(self.end.seen, [])

    def test_an_interrupt_while_it_measures_lets_it_clean_up_first(self):
        ready, cleaned = os.path.join(self.base, "body-ready"), os.path.join(self.base, "body-cleaned")
        os.environ.update(FAKE_BODY_MODE="hang", FAKE_BODY_READY=ready, FAKE_BODY_CLEANED=cleaned)
        sender = Interrupts.send_when_ready(self, signal.SIGINT, ready)
        self.assertEqual(self.run_cell("--arm", "rcp"), 130)
        sender.join(5)
        with open(cleaned, encoding="utf-8") as f:
            self.assertEqual(f.read(), "cleaned")
        self.assertEqual(self.end.seen, [])
        self.assert_nothing_left()

    def test_with_sigint_ignored_from_the_start_a_sigterm_still_lets_it_clean_up_first(self):
        # A background job of a script starts with SIGINT ignored, and a child would inherit that; the body record
        # starts with SIGINT at its default, so the SIGINT that stops it still runs its clean-up within the grace.
        old = signal.signal(signal.SIGINT, signal.SIG_IGN)
        self.addCleanup(signal.signal, signal.SIGINT, old)
        ready, cleaned = os.path.join(self.base, "body-ready"), os.path.join(self.base, "body-cleaned")
        os.environ.update(FAKE_BODY_MODE="hang", FAKE_BODY_READY=ready, FAKE_BODY_CLEANED=cleaned)
        sender = Interrupts.send_when_ready(self, signal.SIGTERM, ready)
        with mock.patch.object(cli, "STOP_WINDOW_S", 20.0):
            self.assertEqual(self.run_cell("--arm", "rcp"), 143)
        sender.join(5)
        with open(cleaned, encoding="utf-8") as f:
            self.assertEqual(f.read(), "cleaned")
        self.assertIs(signal.getsignal(signal.SIGINT), signal.SIG_IGN)
        self.assertEqual(self.end.seen, [])
        self.assert_nothing_left()

    def test_a_signal_that_arrives_while_it_starts_still_stops_it(self):
        os.environ["FAKE_BODY_MODE"] = "hang"
        os.environ.update(FAKE_BODY_READY=os.path.join(self.base, "body-ready"),
                          FAKE_BODY_CLEANED=os.path.join(self.base, "body-cleaned"))
        started = signal_inside_popen(self, signal.SIGTERM)
        self.assertEqual(self.run_cell("--arm", "rcp"), 143)
        self.assertEqual(len(started), 1)
        self.assertIsNotNone(started[0].returncode, "the body record still runs")
        self.assertEqual(self.end.seen, [])
        self.assert_nothing_left()

    def test_one_that_does_not_take_sigint_is_killed_when_the_window_ends(self):
        ready, cleaned = os.path.join(self.base, "body-ready"), os.path.join(self.base, "body-cleaned")
        os.environ.update(FAKE_BODY_MODE="deaf", FAKE_BODY_READY=ready, FAKE_BODY_CLEANED=cleaned)
        sender = Interrupts.send_when_ready(self, signal.SIGTERM, ready)
        t0 = time.monotonic()
        with mock.patch.object(cli, "STOP_WINDOW_S", 2.0):
            self.assertEqual(self.run_cell("--arm", "rcp"), 143)
        self.assertLess(time.monotonic() - t0, 2.0 + 5.0)
        sender.join(5)
        self.assertFalse(os.path.exists(cleaned))
        self.assertEqual(self.stderr.getvalue(), f"{T} stopped by SIGTERM: the running episode did not stop within "
                                                 f"2 s, so its simulator may still be running; the scratch folder was "
                                                 f"removed; out/summary.json holds the setups that finished\n")
        self.assert_nothing_left()


class StopWindow(unittest.TestCase):
    def test_every_stop_ends_well_before_the_bench_command_ends_run_cell(self):
        # cerulion rcp bench ends run-cell 60 s after it forwards an interrupt; the removal of the scratch folder
        # comes after the stops.
        self.assertLess(cli.DRIVER_GRACE_S, cli.STOP_WINDOW_S)
        self.assertLessEqual(cli.STOP_WINDOW_S, 60 - 15)
        self.assertIn(f"within {cli.STOP_WINDOW_S:g} s of the signal", " ".join(cli.__doc__.split()))


class StopGroup(unittest.TestCase):
    """Ending a child that leads its own process group."""

    def spawn(self, code):
        return subprocess.Popen([sys.executable, "-c", code], start_new_session=True, stdout=subprocess.PIPE)

    def test_a_child_that_ignores_the_first_signal_is_killed_after_the_grace(self):
        proc = self.spawn("import signal, time, sys; signal.signal(signal.SIGTERM, signal.SIG_IGN); "
                          "print('up', flush=True); time.sleep(60)")
        proc.stdout.readline()
        t0 = time.monotonic()
        self.assertIs(cli.stop_group(proc, signal.SIGTERM, 0.5), False)
        self.assertEqual(proc.returncode, -signal.SIGKILL)
        self.assertLess(time.monotonic() - t0, 10)
        proc.stdout.close()

    def test_what_is_left_in_the_group_ends_with_the_child(self):
        proc = self.spawn("import subprocess, sys, time; c = subprocess.Popen([sys.executable, '-c', "
                          "'import time; time.sleep(60)']); print(c.pid, flush=True); time.sleep(60)")
        grandchild = int(proc.stdout.readline())
        self.assertIs(cli.stop_group(proc, signal.SIGTERM, 5), True)
        self.assertEqual(proc.returncode, -signal.SIGTERM)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                os.kill(grandchild, 0)
            except ProcessLookupError:
                break
            time.sleep(0.05)
        else:
            self.fail("the grandchild still runs")
        proc.stdout.close()

    def test_a_child_already_reaped_is_left_alone(self):
        proc = self.spawn("pass")
        proc.wait()
        proc.stdout.close()
        with mock.patch.object(cli.os, "killpg") as killpg:
            self.assertIs(cli.stop_group(proc, signal.SIGTERM, 1), True)
        killpg.assert_not_called()


class SignalsUnit(unittest.TestCase):
    def setUp(self):
        unignore_signals(self)

    def test_the_first_signal_raises_and_the_next_is_ignored(self):
        with cli.Signals() as s:
            with self.assertRaises(cli.Interrupted) as caught:
                os.kill(os.getpid(), signal.SIGTERM)
                time.sleep(1)
            os.kill(os.getpid(), signal.SIGTERM)
            time.sleep(0.05)
        self.assertEqual((caught.exception.signum, s.seen), (signal.SIGTERM, signal.SIGTERM))

    def test_a_signal_inside_deferred_is_raised_when_the_block_ends(self):
        done = []
        with cli.Signals() as s:
            with self.assertRaises(cli.Interrupted) as caught:
                with s.deferred():
                    os.kill(os.getpid(), signal.SIGTERM)
                    time.sleep(0.2)
                    done.append("the block ran to its end")
            os.kill(os.getpid(), signal.SIGTERM)   # a second one changes nothing
            time.sleep(0.05)
        self.assertEqual((done, caught.exception.signum, s.seen),
                         (["the block ran to its end"], signal.SIGTERM, signal.SIGTERM))

    def test_a_deferred_block_with_no_signal_raises_nothing(self):
        with cli.Signals() as s:
            with s.deferred():
                pass
            with self.assertRaises(cli.Interrupted):   # and the guard works as before after it
                os.kill(os.getpid(), signal.SIGINT)
                time.sleep(1)
        self.assertEqual(s.seen, signal.SIGINT)

    def test_after_hold_every_signal_is_ignored(self):
        with cli.Signals() as s:
            s.hold()
            os.kill(os.getpid(), signal.SIGINT)
            time.sleep(0.05)
        self.assertIsNone(s.seen)

    def test_outside_the_main_thread_nothing_is_installed(self):
        seen = []

        def enter():
            with cli.Signals():
                seen.append(signal.getsignal(signal.SIGTERM))

        before = signal.getsignal(signal.SIGTERM)
        t = threading.Thread(target=enter)
        t.start()
        t.join()
        self.assertEqual(seen, [before])

    def test_the_exit_code_names_the_signal(self):
        self.assertEqual([128 + s for s in cli.Signals.CAUGHT], [130, 143, 129])

    def test_a_signal_ignored_from_the_start_stays_ignored(self):
        # nohup starts a command with SIGHUP ignored, and a non-interactive shell starts a background job with
        # SIGINT ignored: inside the guard both stay ignored, either signal changes nothing and the run goes on.
        code = ("import os, signal, sys, time\n"
                f"sys.path.insert(0, {SRC!r})\n"
                "from cerulion_harness import cli\n"
                "def ignored():\n"
                "    return [signal.getsignal(s) is signal.SIG_IGN for s in cli.Signals.CAUGHT]\n"
                "with cli.Signals() as s:\n"
                "    inside = ignored()\n"
                "    os.kill(os.getpid(), signal.SIGHUP)\n"
                "    os.kill(os.getpid(), signal.SIGINT)\n"
                "    time.sleep(0.2)\n"
                "    print('went on', s.seen, inside)\n"
                "print('after', ignored())\n")
        old = {sig: signal.signal(sig, signal.SIG_IGN) for sig in (signal.SIGHUP, signal.SIGINT)}
        try:   # this process is the parent that ignores them, as nohup and the shell are
            proc = subprocess.Popen([sys.executable, "-I", "-B", "-c", code], stdin=subprocess.DEVNULL,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        finally:
            for sig, handler in old.items():
                signal.signal(sig, handler)
        out, err = proc.communicate(timeout=60)
        self.assertEqual((proc.returncode, out.decode(), err.decode()),
                         (0, "went on None [True, False, True]\nafter [True, False, True]\n", ""))

    def test_a_child_starts_with_sigint_at_its_default_while_this_process_ignores_it(self):
        probe = [sys.executable, "-I", "-B", "-c",
                 "import signal; print(signal.getsignal(signal.SIGINT) is signal.default_int_handler)"]
        old = signal.signal(signal.SIGINT, signal.SIG_IGN)
        self.addCleanup(signal.signal, signal.SIGINT, old)
        got = []
        for spawn in (subprocess.Popen, cli.popen_with_default_sigint):   # a plain child inherits the ignore
            proc = spawn(probe, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE)
            got.append((proc.communicate(timeout=60)[0].decode(), signal.getsignal(signal.SIGINT) is signal.SIG_IGN))
        self.assertEqual(got, [("False\n", True), ("True\n", True)])


if __name__ == "__main__":
    unittest.main()
