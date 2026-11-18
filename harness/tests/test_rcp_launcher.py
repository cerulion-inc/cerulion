# SPDX-License-Identifier: AGPL-3.0-only
"""rcp.sh, the program `cerulion rcp --sim` and `cerulion rcp bench` run.

Each test copies rcp.sh into a scratch harness folder whose install.sh and
venv/bin/python are stand-ins that write down how they were run, then runs the
launcher the way cerulion does (`bash rcp.sh --home DIR [--packaged] <verb>
[flags]`) and checks what ran, with which words, from which folder, and what
reached stdout. No simulator, no network.
"""

import hashlib
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


# What the stand-ins above write into the folder they run from.
STAND_IN_OUTPUTS = {"installed", "install-runs.txt", "python-ran.txt", "verified.txt"}


def write(path, text, mode=0o644):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    path.chmod(mode)


def tree(folder, metadata=True):
    """Every entry under folder, the folder itself first, in sorted order: its path, its kind with the SHA-256 of a
    file or the target of a link, and with metadata its permission bits and modification time. Two equal trees hold
    the same bytes; with metadata they also show that nothing was written, removed or changed in place."""
    rows = []

    def add(path):
        st = path.lstat()
        if stat.S_ISLNK(st.st_mode):
            kind = "link " + os.readlink(path)
        elif stat.S_ISDIR(st.st_mode):
            kind = "dir"
        else:
            kind = "file " + hashlib.sha256(path.read_bytes()).hexdigest()
        row = (str(path.relative_to(folder)), kind)
        rows.append(row + (stat.S_IMODE(st.st_mode), st.st_mtime_ns) if metadata else row)

    add(folder)
    for root, dirs, files in os.walk(folder):
        dirs.sort()
        for name in sorted(dirs + files):
            add(pathlib.Path(root) / name)
    return rows


def read_only(folder):
    """chmod -R a-w, as a system package leaves its folder."""
    for root, dirs, files in os.walk(folder, topdown=False):
        for name in files + dirs:
            path = os.path.join(root, name)
            if not os.path.islink(path):
                os.chmod(path, stat.S_IMODE(os.lstat(path).st_mode) & ~0o222)
    os.chmod(folder, stat.S_IMODE(os.stat(folder).st_mode) & ~0o222)


def is_root():
    return hasattr(os, "geteuid") and os.geteuid() == 0


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

    def run_launcher(self, *words, env=None, folder=None, home=None, packaged=False):
        flags = ["--home", str(home or self.home)] + (["--packaged"] if packaged else [])
        return self.run_raw(*flags, *words, env=env, folder=folder)

    def run_raw(self, *argv, env=None, folder=None):
        folder = folder or self.harness
        return subprocess.run(
            ["bash", str(folder / "rcp.sh"), *argv],
            cwd=self.tmp, env={**self.env, **(env or {})},
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=60)

    def copied(self, home):
        """The copy's bytes without what the stand-ins wrote into it once it ran."""
        return [row for row in tree(home, metadata=False) if row[0] not in STAND_IN_OUTPUTS]

    def packaged_copy_said(self, home):
        return (f"{self.harness} is a release package's folder, which nothing writes into: copying it once to "
                f"{home}, where the install goes")

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
        if is_root():
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

    def test_a_folder_this_user_cannot_write_and_no_copy_home_is_refused_with_the_way_out(self):
        if is_root():
            self.skipTest("root writes to a read-only folder")
        (self.harness / "installed").touch()
        self.harness.chmod(stat.S_IRUSR | stat.S_IXUSR)
        run = self.run_raw("sim")
        self.assertEqual(run.returncode, 2, run.stderr)
        self.assertIn(f"this user cannot write to {self.harness}, where the install goes; run it through cerulion rcp, "
                      "which names a folder to copy it to, or point CERULION_HARNESS_DIR at a copy you own", run.stderr)
        self.assertFalse((self.harness / "python-ran.txt").exists())

    def test_a_folder_that_is_not_packaged_runs_in_place_and_is_never_copied(self):
        for words, env in [(("sim",), {}), (("bench", "--model", "m", "--base-url", "https://example.test/v1"),
                                             {"OPENAI_API_KEY": "k"}), (("verify",), {})]:
            with self.subTest(verb=words[0]):
                run = self.run_launcher(*words, env=env)
                self.assertEqual(run.returncode, 0, run.stderr)
                self.assertNotIn("copying", run.stderr)
                self.assertFalse(self.home.parent.exists(), "a folder that may be written was copied")
        self.assertEqual(self.python_ran()[0], str(self.harness))
        self.assertEqual(self.install_runs(), ["--check", "<none>", "--check"])

    def test_a_writable_packaged_folder_runs_from_its_copy_and_is_never_written(self):
        before = tree(self.harness)
        first = self.run_launcher("sim", packaged=True)
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertEqual(first.stdout, "PY-OUT\n", "the copy's and the install's lines must not reach the MCP stream")
        self.assertIn(self.packaged_copy_said(self.home), first.stderr)
        self.assertEqual(self.python_ran(self.home)[:3], [str(self.home), str(self.home / "playground"), "--stdio"])
        self.assertEqual(self.install_runs(self.home), ["--check", "<none>"], "the install ran in the copy")
        self.assertEqual(tree(self.harness), before, "the first run wrote into the packaged folder")
        self.assertEqual(self.copied(self.home), tree(self.harness, metadata=False),
                         "the copy is not the packaged folder byte for byte")
        (self.home / "copied-once").touch()
        second = self.run_launcher("bench", "--model", "m", "--base-url", "https://example.test/v1",
                                   env={"OPENAI_API_KEY": "k"}, packaged=True)
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertNotIn("copying", second.stderr)
        self.assertTrue((self.home / "copied-once").exists(), "the second run made a new copy")
        self.assertEqual(self.python_ran(self.home)[1:4], ["-m", "cerulion_harness", "run-cell"])
        self.assertEqual(self.install_runs(self.home), ["--check", "<none>", "--check"])
        (self.home / "venv" / "bin" / "python").unlink()
        verify = self.run_launcher("verify", packaged=True)
        self.assertEqual(verify.returncode, 0, verify.stderr)
        self.assertEqual((self.home / "verified.txt").read_text(), str(self.home), "verify reads the folder that runs")
        self.assertEqual(tree(self.harness), before, "a later run wrote into the packaged folder")

    def test_a_read_only_packaged_folder_is_copied_once_and_the_copy_is_reused(self):
        if is_root():
            self.skipTest("root writes to a read-only folder")
        read_only(self.harness)
        before = tree(self.harness)
        first = self.run_launcher("sim", packaged=True)
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertIn(self.packaged_copy_said(self.home), first.stderr)
        self.assertNotIn("read-only for this user", first.stderr)
        self.assertEqual(self.python_ran(self.home)[:3], [str(self.home), str(self.home / "playground"), "--stdio"])
        self.assertEqual(self.install_runs(self.home), ["--check", "<none>"])
        self.assertEqual(self.copied(self.home), tree(self.harness, metadata=False))
        (self.home / "copied-once").touch()
        second = self.run_launcher("sim", packaged=True)
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertNotIn("copying", second.stderr)
        self.assertTrue((self.home / "copied-once").exists(), "the second run made a new copy")
        self.assertEqual(self.install_runs(self.home), ["--check", "<none>", "--check"])
        self.assertEqual(tree(self.harness), before)

    def test_an_upgrade_copies_the_new_release_beside_the_old_copy_and_leaves_the_old_one_alone(self):
        old_home, new_home = self.home, self.home.parent / "2.2.0"
        first = self.run_launcher("sim", packaged=True, home=old_home)
        self.assertEqual(first.returncode, 0, first.stderr)
        old_copy = tree(old_home)
        write(self.harness / "playground" / "release.txt", "2.2.0\n")  # the upgrade's new packaged folder
        before = tree(self.harness)
        second = self.run_launcher("sim", packaged=True, home=new_home)
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertIn(self.packaged_copy_said(new_home), second.stderr)
        self.assertEqual(self.python_ran(new_home)[0], str(new_home))
        self.assertEqual((new_home / "playground" / "release.txt").read_text(), "2.2.0\n")
        self.assertEqual(self.copied(new_home), tree(self.harness, metadata=False))
        self.assertEqual(tree(old_home), old_copy, "the upgrade changed the old release's copy")
        self.assertEqual(tree(self.harness), before)

    def test_a_failed_install_in_the_copy_stops_with_its_code_and_the_next_run_resumes_in_the_copy(self):
        (self.harness / "install-exit").write_text("7")
        before = tree(self.harness)
        run = self.run_launcher("sim", packaged=True)
        self.assertEqual(run.returncode, 7)
        self.assertIn("the install stopped (exit 7); run the same command again to resume it", run.stderr)
        self.assertFalse((self.home / "python-ran.txt").exists())
        self.assertEqual(tree(self.harness), before)
        (self.home / "install-exit").write_text("0")
        again = self.run_launcher("sim", packaged=True)
        self.assertEqual(again.returncode, 0, again.stderr)
        self.assertNotIn("copying", again.stderr)
        self.assertEqual(self.install_runs(self.home), ["--check", "<none>", "--check", "<none>"])
        self.assertEqual(self.python_ran(self.home)[0], str(self.home))
        self.assertEqual(tree(self.harness), before)

    def test_packaged_needs_a_copy_home_and_one_that_names_the_packaged_folder_is_refused(self):
        before = tree(self.harness)
        for argv in [("--packaged", "sim"), ("--packaged", "--home", str(self.home), "sim"),
                     ("--home", "", "--packaged", "sim"), ("--home", str(self.home), "--packaged")]:
            with self.subTest(argv=argv):
                run = self.run_raw(*argv)
                self.assertEqual(run.returncode, 2, run.stderr)
                self.assertIn("usage: rcp.sh", run.stderr)
        link = self.tmp / "link-to-harness"
        link.symlink_to(self.harness)
        for home in [str(self.harness), str(self.harness) + "/", str(link)]:
            with self.subTest(home=home):
                run = self.run_raw("--home", home, "--packaged", "sim")
                self.assertEqual(run.returncode, 2, run.stderr)
                self.assertIn(f"--home names {self.harness} itself, a release package's folder, which nothing writes "
                              "into; name another folder", run.stderr)
        self.assertEqual(tree(self.harness), before)
        self.assertFalse(self.home.parent.exists())

    def test_a_usage_mistake_exits_2(self):
        for words in [(), ("serve",), ("sim", "--port", "1"), ("sim", "--http"), ("verify", "--all")]:
            with self.subTest(words=words):
                run = self.run_launcher(*words)
                self.assertEqual(run.returncode, 2, run.stderr)
                self.assertIn("usage: rcp.sh", run.stderr)


if __name__ == "__main__":
    unittest.main()
