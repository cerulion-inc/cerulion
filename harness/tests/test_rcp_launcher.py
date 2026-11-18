# SPDX-License-Identifier: AGPL-3.0-only
"""rcp.sh, the program `cerulion rcp --sim` and `cerulion rcp bench` run.

Each test copies rcp.sh into a scratch harness folder whose install.sh and
venv/bin/python are stand-ins that write down how they were run, then runs the
launcher the way cerulion does (`bash rcp.sh --home DIR <verb> [flags]`) and
checks what ran, with which words, and what reached stdout. No simulator, no
network.
"""

import os
import pathlib
import shutil
import stat
import subprocess
import tempfile
import unittest

LAUNCHER = pathlib.Path(__file__).resolve().parent.parent / "rcp.sh"

# install.sh: `--check` answers from the file `installed`; a run writes it,
# prints a progress line and exits with the code in `install-exit` (0 when
# there is none). Every run appends its words to install-runs.txt.
INSTALL = """#!/usr/bin/env bash
here=$(cd "$(dirname "$0")" && pwd -P)
printf '%s\\n' "${*:-<none>}" >> "$here/install-runs.txt"
if [ "${1:-}" = --check ]; then [ -f "$here/installed" ]; exit $?; fi
echo "INSTALL-PROGRESS"
code=0
[ -f "$here/install-exit" ] && code=$(cat "$here/install-exit")
[ "$code" = 0 ] && : > "$here/installed"
exit "$code"
"""

# venv/bin/python: writes the folder it ran from and its words, prints PY-OUT
# and exits 0.
PYTHON = """#!/usr/bin/env bash
here=$(cd "$(dirname "$0")/../.." && pwd -P)
{ echo "$here"; printf '%s\\n' "$@"; } > "$here/python-ran.txt"
echo "PY-OUT"
"""

# src/cerulion_harness/verify.py: verify(home) reports one problem when
# home/tampered exists, and writes the folder it was given to home/verified.txt.
VERIFY = """def verify(home):
    import os
    with open(os.path.join(home, "verified.txt"), "w") as f:
        f.write(home)
    problems = ["rcp: rcpx/x.py differs from the ledger"] if os.path.exists(os.path.join(home, "tampered")) else []
    return problems, {"rcp": {"snapshot": "sha256:ab", "shipped_files": 3}}
"""


def write(path, text, mode=0o644):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    path.chmod(mode)


class Launcher(unittest.TestCase):
    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp()).resolve()
        self.addCleanup(self._cleanup)
        self.harness = self.tmp / "harness"
        self.home = self.tmp / "config" / "harness" / "2.1.0"
        self.harness.mkdir()
        shutil.copy(LAUNCHER, self.harness / "rcp.sh")
        write(self.harness / "install.sh", INSTALL, 0o755)
        write(self.harness / "venv" / "bin" / "python", PYTHON, 0o755)
        (self.harness / "playground").mkdir()
        write(self.harness / "src" / "cerulion_harness" / "__init__.py", "")
        write(self.harness / "src" / "cerulion_harness" / "verify.py", VERIFY)
        self.env = {k: v for k, v in os.environ.items()
                    if k not in ("OPENAI_BASE_URL", "OPENAI_API_KEY", "PYTHONPATH")}

    def _cleanup(self):
        for root, dirs, _files in os.walk(self.tmp):
            for d in dirs:
                os.chmod(os.path.join(root, d), 0o755)
        os.chmod(self.tmp, 0o755)
        shutil.rmtree(self.tmp)

    def run_launcher(self, *words, env=None, folder=None):
        folder = folder or self.harness
        return subprocess.run(
            ["bash", str(folder / "rcp.sh"), "--home", str(self.home), *words],
            cwd=self.tmp, env={**self.env, **(env or {})},
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=60)

    def python_ran(self, folder=None):
        return (folder or self.harness).joinpath("python-ran.txt").read_text().splitlines()

    def install_runs(self, folder=None):
        path = (folder or self.harness) / "install-runs.txt"
        return path.read_text().splitlines() if path.exists() else []

    def test_sim_serves_stdio_by_default_and_stdout_is_the_server_alone(self):
        (self.harness / "installed").touch()
        run = self.run_launcher("sim")
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(run.stdout, "PY-OUT\n")
        self.assertEqual(self.python_ran(), [str(self.harness), str(self.harness / "playground"), "--stdio"])
        self.assertEqual(self.install_runs(), ["--check"])

    def test_sim_over_http_with_a_task_names_the_servers_state(self):
        (self.harness / "installed").touch()
        run = self.run_launcher("sim", "--http", "127.0.0.1:8765", "--task", "open-drawer")
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(self.python_ran()[1:5],
                         [str(self.harness / "playground"), "--http", "127.0.0.1:8765", "--state"])
        self.assertEqual(self.python_ran()[5], "open-drawer")

    def test_the_first_run_installs_once_with_its_progress_on_stderr(self):
        first = self.run_launcher("sim")
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertEqual(first.stdout, "PY-OUT\n", "the install's progress must not reach the MCP stream")
        self.assertIn("INSTALL-PROGRESS", first.stderr)
        self.assertIn("First run: installing the simulator and the benchmark scenes", first.stderr)
        second = self.run_launcher("sim")
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertNotIn("INSTALL-PROGRESS", second.stderr)
        self.assertEqual(self.install_runs(), ["--check", "<none>", "--check"])

    def test_a_failed_install_stops_with_its_code_and_runs_nothing(self):
        (self.harness / "install-exit").write_text("7")
        run = self.run_launcher("sim")
        self.assertEqual(run.returncode, 7)
        self.assertIn("the install stopped (exit 7); run the same command again to resume it", run.stderr)
        self.assertFalse((self.harness / "python-ran.txt").exists())

    def test_bench_hands_its_flags_to_run_cell_word_for_word(self):
        (self.harness / "installed").touch()
        run = self.run_launcher("bench", "--model", "my model", "--base-url", "https://example.test/v1",
                                "--key-env", "MY_KEY", "--out", "out dir", env={"MY_KEY": "k"})
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(self.python_ran()[1:], ["-m", "cerulion_harness", "run-cell", "--model", "my model",
                                                 "--base-url", "https://example.test/v1", "--key-env", "MY_KEY",
                                                 "--out", "out dir"])

    def test_bench_without_an_endpoint_or_a_key_refuses_before_any_install(self):
        cases = [
            ({}, ("--model", "m"), "no endpoint: pass --base-url or set OPENAI_BASE_URL"),
            ({"OPENAI_BASE_URL": "https://example.test/v1"}, ("--model", "m"),
             "no key: set the environment variable OPENAI_API_KEY (or name another with --key-env)"),
            ({"OPENAI_BASE_URL": "https://example.test/v1", "OPENAI_API_KEY": "k"},
             ("--model", "m", "--key-env", "NOT_SET_HERE"),
             "no key: set the environment variable NOT_SET_HERE (or name another with --key-env)"),
            ({"OPENAI_BASE_URL": "https://example.test/v1"}, ("--model", "m", "--key-env", "A B"),
             "--key-env: 'A B' is not an environment variable name"),
        ]
        for env, words, said in cases:
            with self.subTest(said=said):
                run = self.run_launcher("bench", *words, env=env)
                self.assertEqual(run.returncode, 2)
                self.assertIn(said, run.stderr)
                self.assertEqual(self.install_runs(), [])
                self.assertFalse((self.harness / "python-ran.txt").exists())

    def test_verify_checks_this_folder_installs_nothing_and_fails_on_a_problem(self):
        # No install yet: verify runs under the python3 on PATH, with this folder's src/.
        (self.harness / "venv" / "bin" / "python").unlink()
        ok = self.run_launcher("verify")
        self.assertEqual(ok.returncode, 0, ok.stderr)
        self.assertEqual(ok.stdout, "rcp: snapshot sha256:ab, 3 shipped files\n"
                                    "verify: ok, this folder holds the code the benchmark measured\n")
        self.assertEqual((self.harness / "verified.txt").read_text(), str(self.harness))
        (self.harness / "tampered").touch()
        bad = self.run_launcher("verify")
        self.assertEqual(bad.returncode, 1, bad.stderr)
        self.assertIn("PROBLEM: rcp: rcpx/x.py differs from the ledger", bad.stderr)
        self.assertIn("verify: 1 problem(s); this folder is not the measured code", bad.stdout)
        self.assertEqual(self.install_runs(), [])

    def test_verify_never_reports_ok_when_the_check_cannot_run(self):
        (self.harness / "src" / "cerulion_harness" / "verify.py").write_text("raise ImportError('broken')\n")
        (self.harness / "venv" / "bin" / "python").unlink()
        run = self.run_launcher("verify")
        self.assertNotEqual(run.returncode, 0)
        self.assertNotIn("verify: ok", run.stdout)

    def test_a_folder_this_user_cannot_write_is_copied_once_and_runs_from_the_copy(self):
        if hasattr(os, "geteuid") and os.geteuid() == 0:
            self.skipTest("root writes to a read-only folder")
        (self.harness / "installed").touch()
        self.harness.chmod(stat.S_IRUSR | stat.S_IXUSR)
        first = self.run_launcher("sim")
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertIn("is read-only for this user: copying it once to " + str(self.home), first.stderr)
        self.assertEqual(self.python_ran(self.home)[:3], [str(self.home), str(self.home / "playground"),
                                                          "--stdio"])
        (self.home / "copied-once").touch()
        second = self.run_launcher("sim")
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertNotIn("copying", second.stderr)
        self.assertTrue((self.home / "copied-once").exists(), "the second run made a new copy")
        (self.home / "venv" / "bin" / "python").unlink()
        verify = self.run_launcher("verify")
        self.assertEqual(verify.returncode, 0, verify.stderr)
        self.assertEqual((self.home / "verified.txt").read_text(), str(self.home), "verify reads the folder that runs")

    def test_a_usage_mistake_exits_2(self):
        for words in [(), ("serve",), ("sim", "--port", "1"), ("sim", "--http"), ("verify", "--all")]:
            with self.subTest(words=words):
                run = self.run_launcher(*words)
                self.assertEqual(run.returncode, 2, run.stderr)
                self.assertIn("usage: rcp.sh", run.stderr)


if __name__ == "__main__":
    unittest.main()
