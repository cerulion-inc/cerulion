# SPDX-License-Identifier: AGPL-3.0-only
"""run-cell, the engine behind cerulion rcp bench: what it refuses before anything starts, the one test request
to the model endpoint, and the row it reads from a scored episode. Standard library only: the endpoint is a
server on the loopback address inside the test, and the episode results are hand-written fixtures."""
import contextlib
import http.server
import io
import json
import os
import pathlib
import socket
import sys
import tempfile
import threading
import unittest
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from cerulion_harness import cli  # noqa: E402

CELL = "pro_spatial_swap_t00_i00"
KEY = "test-key-0123456789"


def args(*extra):
    return cli.parser().parse_args(["run-cell", "--model", "gemini-3.8-flash", *extra])


def write_json(path, doc):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f)


class Refusals(unittest.TestCase):
    """Each reason a run cannot start, found before anything is written or sent."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.out = os.path.join(self.tmp.name, "out")
        self.env = mock.patch.dict(os.environ, {"OPENAI_API_KEY": KEY}, clear=True)
        self.env.start()
        # No install here, whatever the machine running the test has installed.
        self.sim = mock.patch.object(cli.layout, "SIM_PYTHON", os.path.join(self.tmp.name, "no", "python"))
        self.sim.start()

    def tearDown(self):
        self.sim.stop()
        self.env.stop()
        self.tmp.cleanup()

    def refusal(self, *extra, arms=("plain", "rcp")):
        ns = args("--base-url", "http://127.0.0.1:9/v1", "--out", self.out, *extra)
        return cli.refusal_before_start(ns, list(arms))

    def test_no_endpoint_is_refused(self):
        ns = args("--out", self.out)
        ns.base_url = None
        self.assertEqual(cli.refusal_before_start(ns, ["rcp"]),
                         (2, "no endpoint: pass --base-url or set OPENAI_BASE_URL"))

    def test_no_key_is_refused_with_the_variable_it_read(self):
        code, text = self.refusal("--key-env", "MY_KEY")
        self.assertEqual(code, 2)
        self.assertIn("MY_KEY", text)

    def test_an_unknown_cell_is_refused(self):
        code, text = self.refusal("--cell", "pro_spatial_swap")
        self.assertEqual(code, 2)
        self.assertIn("unknown cell id", text)

    def test_an_extra_body_that_is_not_a_json_object_is_refused(self):
        for bad in ("{reasoning_effort: high}", "[1, 2]", "3"):
            code, text = self.refusal("--extra-body", bad)
            self.assertEqual(code, 2, bad)
            self.assertIn("--extra-body", text, bad)

    def test_an_out_folder_that_holds_a_result_of_this_cell_is_refused(self):
        # The driver resumes a folder: a cell with a result there would be reported again, not run.
        write_json(os.path.join(self.out, "rcp", "libero_pro", "run_with", CELL + ".json"), {"success": True})
        code, text = self.refusal()
        self.assertEqual(code, 2)
        self.assertIn("already holds a rcp result", text)

    def test_a_result_of_the_other_arm_only_refuses_a_run_of_that_arm(self):
        write_json(os.path.join(self.out, "plain", "libero_pro", "run_without", CELL + ".json"), {})
        code, text = self.refusal(arms=("plain",))
        self.assertIn("already holds a plain result", text)
        code, text = self.refusal(arms=("rcp",))
        self.assertNotIn("already holds", text)

    def test_without_the_install_the_simulator_is_named_as_missing(self):
        code, text = self.refusal()
        self.assertEqual(code, 2)
        self.assertIn("the simulator is not installed", text)
        self.assertIn("install.sh", text)

    def test_outside_the_episode_server_environment_the_command_to_use_is_named(self):
        python = os.path.join(self.tmp.name, "sim", "python")
        write_json(python, {})
        with mock.patch.object(cli.layout, "SIM_PYTHON", python), \
                mock.patch.object(cli.layout, "VENV_PYTHON", os.path.join(self.tmp.name, "venv", "bin", "python")):
            code, text = self.refusal()
        self.assertEqual(code, 2)
        self.assertIn("-m cerulion_harness run-cell", text)

    def test_the_command_exits_2_and_says_why_on_stderr(self):
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            self.assertEqual(cli.main(["run-cell", "--model", "m", "--out", self.out]), 2)
        self.assertIn("no endpoint", err.getvalue())
        self.assertFalse(os.path.exists(self.out))

    def test_without_out_each_run_names_a_new_folder(self):
        err = io.StringIO()
        ns = args()
        with contextlib.redirect_stderr(err):
            cli.cmd_run_cell(ns)
        self.assertRegex(ns.out, r"^cerulion-bench-\d{8}T\d{6}Z$")


class Endpoint(unittest.TestCase):
    """The one small request sent before any simulator starts."""

    def setUp(self):
        self.seen = []
        self.answer = (200, b'{"choices": []}')
        self.answer_headers = {}
        # A second server: any request that reaches it, with the key it carried.
        self.elsewhere = []
        test = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
                test.seen.append((self.path, self.headers.get("Authorization"), json.loads(body)))
                code, text = test.answer
                self.send_response(code)
                for name, value in test.answer_headers.items():
                    self.send_header(name, value)
                self.send_header("Content-Length", str(len(text)))
                self.end_headers()
                self.wfile.write(text)

            def log_message(self, *a):
                pass

        class Elsewhere(http.server.BaseHTTPRequestHandler):
            def answer(self):
                test.elsewhere.append((self.command, self.path, self.headers.get("Authorization")))
                self.send_response(200)
                self.send_header("Content-Length", "2")
                self.end_headers()
                self.wfile.write(b"{}")

            do_GET = do_POST = answer

            def log_message(self, *a):
                pass

        self.server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.other = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Elsewhere)
        for server in (self.server, self.other):
            threading.Thread(target=server.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}/v1"
        self.other_url = f"http://127.0.0.1:{self.other.server_address[1]}/v1/chat/completions"
        # A proxy that does not exist: the loopback endpoint must be reached directly.
        self.env = mock.patch.dict(os.environ, {"TEST_KEY": KEY, "http_proxy": "http://127.0.0.1:9",
                                                "HTTP_PROXY": "http://127.0.0.1:9"}, clear=True)
        self.env.start()

    def tearDown(self):
        self.env.stop()
        for server in (self.server, self.other):
            server.shutdown()
            server.server_close()

    def ns(self, url=None):
        return args("--base-url", url or self.url, "--key-env", "TEST_KEY")

    def test_an_accepted_request_is_no_refusal_and_carries_the_key_and_model(self):
        self.assertIsNone(cli.endpoint_refusal(self.ns()))
        path, auth, body = self.seen[0]
        self.assertEqual(path, "/v1/chat/completions")
        self.assertEqual(auth, "Bearer " + KEY)
        self.assertEqual(body["model"], "gemini-3.8-flash")

    def test_a_url_that_already_names_the_route_is_used_as_it_is(self):
        self.assertIsNone(cli.endpoint_refusal(self.ns(self.url + "/chat/completions/")))
        self.assertEqual(self.seen[0][0], "/v1/chat/completions")

    def test_a_refused_key_is_reported_with_the_key_masked(self):
        for code in (401, 402, 403, 404):
            self.answer = (code, ('{"error": "bad key ' + KEY + '"}').encode())
            text = cli.endpoint_refusal(self.ns())
            self.assertTrue(text.startswith(f"HTTP {code}: "), text)
            self.assertIn("[KEY]", text)
            self.assertNotIn(KEY, text)

    def test_any_other_answer_is_left_to_the_episode(self):
        for code in (300, 400, 429, 500, 503):
            self.answer = (code, b"busy")
            self.assertIsNone(cli.endpoint_refusal(self.ns()), code)

    def test_a_redirect_is_reported_and_never_followed_so_the_key_reaches_no_other_server(self):
        for code in (301, 302, 303, 307, 308):
            self.seen.clear()
            self.answer, self.answer_headers = (code, b""), {"Location": self.other_url}
            text = cli.endpoint_refusal(self.ns())
            self.assertEqual(text, f"HTTP {code}: the endpoint redirects to {self.other_url}. A redirect is not "
                                   f"followed, so the key goes only to the URL given: pass the address it "
                                   f"redirects to as --base-url")
            self.assertEqual(len(self.seen), 1, code)
            self.assertEqual(self.elsewhere, [], code)

    def test_a_redirect_is_named_as_a_whole_address_with_the_key_masked(self):
        for location, named in (("/v2/chat/completions", f"{self.url[:-len('/v1')]}/v2/chat/completions"),
                                (f"{self.other_url}?key={KEY}", f"{self.other_url}?key=[KEY]")):
            self.answer, self.answer_headers = (302, b""), {"Location": location}
            text = cli.endpoint_refusal(self.ns())
            self.assertTrue(text.startswith(f"HTTP 302: the endpoint redirects to {named}. "), text)
            self.assertNotIn(KEY, text)
        self.assertEqual(self.elsewhere, [])

    def test_an_address_where_nothing_listens_is_reported(self):
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
        sock.close()
        text = cli.endpoint_refusal(self.ns(f"http://127.0.0.1:{port}/v1"))
        self.assertIn("could not be reached", text)
        self.assertIn("ConnectionRefusedError", text)


class Results(unittest.TestCase):
    """The row run-cell prints and writes for one arm, read from the driver's files."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.out = self.tmp.name
        self.group = os.path.join(self.out, "rcp", "libero_pro", "run_with")

    def tearDown(self):
        self.tmp.cleanup()

    def test_a_scored_episode_reads_its_score_counts_and_list_price(self):
        write_json(os.path.join(self.group, CELL + ".json"), {
            "success": True, "first_success_step": 412, "success_within_ref": True, "claimed_success": True,
            "false_claim": False, "model_requests": 21, "steps_used": 640, "end_reason": "done",
            "grid": {"attempt": 2, "operator_wall_s": 301.5}})
        reply = {"kind": "reply", "usage": {"prompt_tokens": 200_000, "completion_tokens": 2_000,
                                            "prompt_tokens_details": {"cached_tokens": 150_000}}}
        logs = os.path.join(self.group, "_episodes", CELL)
        os.makedirs(os.path.join(logs, "a1"))
        os.makedirs(os.path.join(logs, "a2"))
        with open(os.path.join(logs, "a1", "operator.jsonl"), "w", encoding="utf-8") as f:
            f.write(json.dumps({"kind": "reply", "usage": {"prompt_tokens": 9_999_999}}) + "\n")
        with open(os.path.join(logs, "a2", "operator.jsonl"), "w", encoding="utf-8") as f:
            f.write(json.dumps(reply) + "\n" + json.dumps(reply) + "\n")
        row = cli.read_result(args("--out", self.out), "rcp", self.out)
        self.assertEqual(
            {k: row[k] for k in ("scored", "solved", "solved_within_published_step_limit", "false_claim",
                                 "model_replies", "control_steps", "operator_seconds", "attempt", "end_reason")},
            {"scored": True, "solved": True, "solved_within_published_step_limit": True, "false_claim": False,
             "model_replies": 21, "control_steps": 640, "operator_seconds": 301.5, "attempt": 2,
             "end_reason": "done"})
        # The scored attempt's log only. gemini-3.8-flash, per reply: 50,000 x 0.75 + 150,000 x 0.075
        # + 2,000 x 3.75 = 0.05625 USD; two replies 0.1125.
        self.assertEqual(row["list_price_usd"], 0.1125)
        self.assertEqual(row["price_table"], {"key": "gemini-3.8-flash", "date": "2026-11-17"})

    def test_a_model_the_price_table_lacks_has_no_price(self):
        write_json(os.path.join(self.out, "rcp", "libero_pro", "run_with", CELL + ".json"), {"success": False})
        ns = cli.parser().parse_args(["run-cell", "--model", "a-local-model", "--out", self.out])
        row = cli.read_result(ns, "rcp", self.out)
        self.assertTrue(row["scored"])
        self.assertIsNone(row["list_price_usd"])

    def test_an_unscored_episode_reads_its_state_and_reasons(self):
        write_json(os.path.join(self.group, "_status.json"),
                   {"cells": {CELL: {"state": "failed", "reasons": ["the simulator did not start"]}}})
        row = cli.read_result(args("--out", self.out), "rcp", self.out)
        self.assertEqual((row["scored"], row["state"], row["reasons"]),
                         (False, "failed", ["the simulator did not start"]))

    def test_an_arm_that_wrote_nothing_reads_as_not_started(self):
        row = cli.read_result(args("--out", self.out), "plain", self.out)
        self.assertEqual((row["scored"], row["state"], row["reasons"]), (False, "not started", ["see driver.log"]))

    def test_a_status_file_that_is_not_json_reads_as_not_started(self):
        os.makedirs(self.group)
        with open(os.path.join(self.group, "_status.json"), "w", encoding="utf-8") as f:
            f.write("{")
        row = cli.read_result(args("--out", self.out), "rcp", self.out)
        self.assertEqual(row["state"], "not started")


if __name__ == "__main__":
    unittest.main()
