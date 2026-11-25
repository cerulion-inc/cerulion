# SPDX-License-Identifier: AGPL-3.0-only
"""The model key reaches no address but the one given, through run-cell, its relay and the episode it starts.
Standard library only, no simulator: the model endpoint is a server on the loopback address that answers every
request with a redirect to a second server, and the episode driver is a stand-in in a folder made here. Like the
episode driver, it starts the model client as a new Python process with its own environment; the client builds
its urllib opener as the episode's model client does, so what is tested is what that process sends, not how."""
import contextlib
import http.server
import io
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import threading
import unittest
import urllib.request
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from cerulion_harness import cli, redirects  # noqa: E402

KEY = "test-key-0123456789"
DRIVER = '''
import argparse, json, os, subprocess, sys
p = argparse.ArgumentParser()
p.add_argument("verb")
for flag in ("--base-url", "--key-env", "--results"):
    p.add_argument(flag)
ns, _ = p.parse_known_args()
CLIENT = """
import json, os, sys, urllib.error, urllib.request
req = urllib.request.Request(sys.argv[1].rstrip("/") + "/chat/completions", data=b"{}", method="POST",
                             headers={"Content-Type": "application/json",
                                      "Authorization": "Bearer " + os.environ[sys.argv[2]]})
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
# Whether this process's urllib follows a redirect at all, whatever the relay in front of the endpoint does.
guard = urllib.request.HTTPRedirectHandler().redirect_request(req, None, 302, "", {}, "http://127.0.0.1:9/") is None
try:
    with opener.open(req, timeout=30) as reply:
        print(json.dumps({"status": reply.status, "guard": guard}))
except urllib.error.HTTPError as exc:
    print(json.dumps({"status": exc.code, "guard": guard, "location": exc.headers.get("Location")}))
"""
done = subprocess.run([sys.executable, "-c", CLIENT, ns.base_url, ns.key_env], env=dict(os.environ),
                      stdin=subprocess.DEVNULL, capture_output=True, text=True)
os.makedirs(ns.results, exist_ok=True)
with open(os.path.join(ns.results, "client.json"), "w") as f:
    f.write(done.stdout if done.returncode == 0 else json.dumps({"failed": done.stderr[-2000:]}))
'''


class RedirectGuard(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.endpoint_saw, self.elsewhere_saw = [], []
        test = self

        class Elsewhere(http.server.BaseHTTPRequestHandler):
            def answer(self):
                test.elsewhere_saw.append((self.command, self.path, self.headers.get("Authorization")))
                self.send_response(200)
                self.send_header("Content-Length", "2")
                self.end_headers()
                self.wfile.write(b"{}")

            do_GET = do_POST = answer

            def log_message(self, *a):
                pass

        class Endpoint(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                self.rfile.read(int(self.headers.get("Content-Length") or 0))
                test.endpoint_saw.append(self.headers.get("Authorization"))
                self.send_response(302)
                self.send_header("Location", test.elsewhere_url)
                self.send_header("Content-Length", "0")
                self.end_headers()

            def log_message(self, *a):
                pass

        servers = [http.server.ThreadingHTTPServer(("127.0.0.1", 0), h) for h in (Endpoint, Elsewhere)]
        for server in servers:
            threading.Thread(target=server.serve_forever, daemon=True).start()
            self.addCleanup(server.server_close)
            self.addCleanup(server.shutdown)
        self.url = f"http://127.0.0.1:{servers[0].server_address[1]}/v1"
        self.elsewhere_url = f"http://127.0.0.1:{servers[1].server_address[1]}/v1/chat/completions"

        tree = os.path.join(self.tmp.name, "tree")
        os.makedirs(os.path.join(tree, "runs", "tools"))
        with open(os.path.join(tree, "runs", "tools", "grid_ext.py"), "w", encoding="utf-8") as f:
            f.write(DRIVER)
        sim = os.path.join(self.tmp.name, "sim", "python")
        os.makedirs(os.path.dirname(sim))
        open(sim, "w").close()
        handler = urllib.request.HTTPRedirectHandler
        self.addCleanup(setattr, handler, "redirect_request", handler.__dict__["redirect_request"])
        for patch in (mock.patch.object(cli.layout, "SIM_PYTHON", sim),
                      mock.patch.object(cli.layout, "VENV_PYTHON", os.path.join(sys.prefix, "bin", "python")),
                      mock.patch.object(cli.layout, "RCP_TREE", tree),
                      mock.patch.object(cli.layout, "plain_tree", lambda: tree),
                      mock.patch.dict(os.environ, {"TEST_KEY": KEY})):
            patch.start()
            self.addCleanup(patch.stop)
        self.out = os.path.join(self.tmp.name, "out")
        self.client = os.path.join(self.out, "plain", "client.json")

    def run_cell(self, *extra):
        err, out = io.StringIO(), io.StringIO()
        with contextlib.redirect_stderr(err), contextlib.redirect_stdout(out):
            # No --work: the default scratch folder under /tmp, short enough for the episode's socket paths.
            code = cli.main(["run-cell", "--model", "m", "--base-url", self.url, "--key-env", "TEST_KEY",
                             "--arm", "plain", "--out", self.out, *extra])
        return code, err.getvalue()

    def test_with_the_endpoint_check_the_redirect_is_named_and_no_episode_starts(self):
        code, err = self.run_cell()
        self.assertEqual(code, 2, err)
        self.assertIn(f"HTTP 302: the endpoint redirects to {self.elsewhere_url}. ", err)
        self.assertIn("pass the address it redirects to as --base-url", err)
        self.assertEqual(self.endpoint_saw, ["Bearer " + KEY])
        self.assertEqual(self.elsewhere_saw, [])
        self.assertFalse(os.path.exists(self.client))

    def test_without_the_endpoint_check_the_episode_client_follows_no_redirect(self):
        # The client reaches the endpoint through run-cell's relay, which follows no redirect and passes no
        # Location back; the client's own urllib follows none either.
        code, err = self.run_cell("--no-endpoint-check")
        with open(self.client, encoding="utf-8") as f:
            self.assertEqual(json.load(f), {"status": 302, "guard": True, "location": None})
        self.assertEqual(code, 1, err)   # the stand-in driver scores nothing
        self.assertEqual(self.endpoint_saw, ["Bearer " + KEY])
        self.assertEqual(self.elsewhere_saw, [])

    def test_a_python_started_with_the_episode_path_runs_the_sitecustomize_it_hides_once(self):
        # A sitecustomize the interpreter already had still runs, once, after the one that refuses redirects, also
        # when the path names the episode folder twice (once through a link).
        hidden = os.path.join(self.tmp.name, "hidden")
        os.makedirs(hidden)
        ran = os.path.join(self.tmp.name, "hidden-ran.txt")
        with open(os.path.join(hidden, "sitecustomize.py"), "w", encoding="utf-8") as f:
            f.write(f"with open({ran!r}, 'a') as f:\n    f.write('ran\\n')\n")
        link = os.path.join(self.tmp.name, "site-link")
        os.symlink(redirects.SITE, link)
        probe = ("import urllib.request\n"
                 "print(urllib.request.HTTPRedirectHandler().redirect_request(None, None, 302, '', {}, "
                 "'http://127.0.0.1:9/'))\n")
        # PYTHONDONTWRITEBYTECODE as the episode has it (cli.THREADS): no bytecode written into the source folder.
        env = dict(os.environ, PYTHONPATH=os.pathsep.join((link, redirects.SITE, hidden)), PYTHONDONTWRITEBYTECODE="1")
        done = subprocess.run([sys.executable, "-c", probe], env=env, stdin=subprocess.DEVNULL,
                              capture_output=True, text=True, timeout=60)
        self.assertEqual((done.returncode, done.stdout.strip(), done.stderr), (0, "None", ""))
        with open(ran, encoding="utf-8") as f:
            self.assertEqual(f.read(), "ran\n")


if __name__ == "__main__":
    unittest.main()
