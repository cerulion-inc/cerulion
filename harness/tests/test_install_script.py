# SPDX-License-Identifier: AGPL-3.0-only
"""install.sh: its flags, the stamps that let a finished install return at once, and the robosuite settings file
it installs. Standard library only, no network: every run here stops before the first download. The expected
stamps are computed here from the rule the script states (the SHA-256 of a step's input files), not read from the
script's own output."""
import hashlib
import os
import pathlib
import platform
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest

HARNESS = pathlib.Path(__file__).resolve().parent.parent
SIM_INPUTS = ("requirements/sim-py310.txt", "requirements/sim-torch-cpu.txt", "requirements/sim-robosuite-macros.py")
INPUTS = ("install.sh", *SIM_INPUTS, "requirements/server-py312.txt", "benchmarks/fetch.sh",
          "benchmarks/LIBERO-PRO-data.files.txt", "benchmarks/LIBERO-PRO-data.sha256")
SUPPORTED = (platform.system(), platform.machine()) in (("Darwin", "arm64"), ("Linux", "x86_64"),
                                                         ("Linux", "aarch64"))


def sha_of(root, *rels):
    h = hashlib.sha256()
    for rel in rels:
        h.update((root / rel).read_bytes())
    return h.hexdigest()


class InstallScript(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        # The real path: the script names its folder by pwd -P.
        self.copy = pathlib.Path(os.path.realpath(self.tmp.name)) / "harness"
        for rel in INPUTS:
            (self.copy / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(HARNESS / rel, self.copy / rel)

    def tearDown(self):
        for dp, dn, _ in os.walk(self.tmp.name):
            for d in dn:
                os.chmod(os.path.join(dp, d), stat.S_IRWXU)
        os.chmod(self.copy, stat.S_IRWXU)
        self.tmp.cleanup()

    def run_script(self, *args):
        return subprocess.run(["bash", str(self.copy / "install.sh"), *args], stdin=subprocess.DEVNULL,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=60)

    def stamp(self, name, digest):
        (self.copy / "bx").mkdir(exist_ok=True)
        (self.copy / "bx" / (".step-" + name)).write_text(digest + "\n", encoding="utf-8")

    def install_everything_by_stamp(self):
        self.stamp("sim", sha_of(self.copy, *SIM_INPUTS))
        self.stamp("scenes", sha_of(self.copy, "benchmarks/fetch.sh", "benchmarks/LIBERO-PRO-data.files.txt",
                                    "benchmarks/LIBERO-PRO-data.sha256"))
        server = sha_of(self.copy, "requirements/server-py312.txt")
        text = f"{server}\n{self.copy}\n{platform.system()}\n"
        self.stamp("server", hashlib.sha256(text.encode("utf-8")).hexdigest())
        for python in ("bx/libero-env/bin/python", "venv/bin/python"):
            path = self.copy / python
            path.parent.mkdir(parents=True)
            path.write_text("#!/bin/sh\n", encoding="utf-8")
            path.chmod(0o755)
        for tree in ("LIBERO", "LIBERO-PRO"):
            (self.copy / "bx" / tree / "libero/libero").mkdir(parents=True)

    def run_with_stand_ins(self):
        """install.sh with stand-ins on PATH: uv records its words and makes an environment as uv does (refusing an
        existing folder); git fails, so a run that reaches the fetch stops there and reaches no network; ldconfig
        lists the CPU renderer. Returns (the words uv was given, the finished run)."""
        mac_gl = "macos/cerulion_harness_macos_gl.py"
        (self.copy / mac_gl).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(HARNESS / mac_gl, self.copy / mac_gl)
        tools = pathlib.Path(self.tmp.name) / "tools"
        tools.mkdir()
        log = pathlib.Path(self.tmp.name) / "uv.log"
        (tools / "uv").write_text(
            '#!/bin/sh\n'
            'echo "$*" >> "$UV_LOG"\n'
            # The simulator's pinned packages bring robosuite into the environment's site folder.
            'case "$*" in "pip install --python "*" -r "*/sim-py310.txt)\n'
            '  mkdir -p "$(dirname "$(dirname "$4")")/site/robosuite" ;; esac\n'
            '[ "$1" = venv ] || exit 0\n'
            'for env; do :; done\n'
            '[ -e "$env" ] && { echo "A virtual environment already exists at $env" >&2; exit 2; }\n'
            'mkdir -p "$env/bin" "$env/site"\n'
            'printf \'#!/bin/sh\\n[ "$1" = -c ] && echo "%s/site"\\nexit 0\\n\' "$env" > "$env/bin/python"\n'
            'chmod +x "$env/bin/python"\n', encoding="utf-8")
        (tools / "git").write_text('#!/bin/sh\necho "git stand-in: no network in this test" >&2\nexit 1\n',
                                   encoding="utf-8")
        (tools / "ldconfig").write_text('#!/bin/sh\necho "libOSMesa.so.8 (libc6) => /usr/lib/libOSMesa.so.8"\n',
                                        encoding="utf-8")
        for tool in ("uv", "git", "ldconfig"):
            (tools / tool).chmod(0o755)
        env = dict(os.environ, PATH=f"{tools}{os.pathsep}{os.environ['PATH']}", UV_LOG=str(log))
        done = subprocess.run(["bash", str(self.copy / "install.sh")], stdin=subprocess.DEVNULL, env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=60)
        words = log.read_text(encoding="utf-8").splitlines() if log.exists() else []
        return words, done

    def test_help_prints_the_usage(self):
        done = self.run_script("--help")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertIn("install.sh [--bench-from DIR] [--check]", done.stdout)

    def test_help_prints_the_whole_header_and_no_code(self):
        lines = self.run_script("--help").stdout.splitlines()
        self.assertEqual(lines[0], "The harness's one-time install: the simulator, the benchmark scenes and the "
                                   "episode server, built inside this")
        self.assertEqual(lines[-1], "the CPU renderer (Ubuntu: sudo apt-get install libosmesa6 libegl1 libgl1 "
                                    "libglib2.0-0).")

    def test_an_unknown_flag_is_refused_with_the_usage(self):
        done = self.run_script("--fast")
        self.assertEqual(done.returncode, 2)
        self.assertIn("usage: install.sh", done.stderr)

    def test_bench_from_needs_an_existing_folder(self):
        done = self.run_script("--bench-from", os.path.join(self.tmp.name, "nowhere"))
        self.assertEqual(done.returncode, 2)
        self.assertIn("is not a folder", done.stderr)
        self.assertEqual(self.run_script("--bench-from").returncode, 2)

    @unittest.skipUnless(SUPPORTED, "the script runs on macOS on Apple silicon and Linux on x86-64 or arm64")
    def test_check_on_a_fresh_folder_names_each_missing_step_and_writes_nothing(self):
        done = self.run_script("--check")
        self.assertEqual(done.returncode, 1)
        self.assertEqual(done.stdout.count("not installed: "), 3, done.stdout)
        self.assertFalse((self.copy / "bx").exists())

    @unittest.skipUnless(SUPPORTED, "the script runs on macOS on Apple silicon and Linux on x86-64 or arm64")
    def test_check_passes_when_every_stamp_matches_its_inputs(self):
        self.install_everything_by_stamp()
        done = self.run_script("--check")
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertEqual(done.stdout.strip(), f"installed: {self.copy}")

    @unittest.skipUnless(SUPPORTED, "the script runs on macOS on Apple silicon and Linux on x86-64 or arm64")
    def test_a_changed_robosuite_settings_file_makes_the_simulator_step_stale(self):
        self.install_everything_by_stamp()
        with open(self.copy / "requirements/sim-robosuite-macros.py", "a", encoding="utf-8") as f:
            f.write("# one more line\n")
        done = self.run_script("--check")
        self.assertEqual(done.returncode, 1)
        self.assertEqual(done.stdout.strip().splitlines(),
                         [f"not installed: the simulator environment ({self.copy}/bx/libero-env)"])

    @unittest.skipUnless(SUPPORTED, "the script runs on macOS on Apple silicon and Linux on x86-64 or arm64")
    def test_a_changed_pin_makes_its_step_stale(self):
        self.install_everything_by_stamp()
        with open(self.copy / "requirements/server-py312.txt", "a", encoding="utf-8") as f:
            f.write("# one more line\n")
        done = self.run_script("--check")
        self.assertEqual(done.returncode, 1)
        self.assertEqual(done.stdout.strip().splitlines(),
                         [f"not installed: the episode server environment ({self.copy}/venv)"])

    @unittest.skipUnless(SUPPORTED, "the script runs on macOS on Apple silicon and Linux on x86-64 or arm64")
    def test_check_names_the_scenes_when_one_benchmark_tree_is_gone(self):
        for tree in ("LIBERO", "LIBERO-PRO"):
            with self.subTest(tree=tree):
                shutil.rmtree(self.copy / "bx", ignore_errors=True)
                shutil.rmtree(self.copy / "venv", ignore_errors=True)
                self.install_everything_by_stamp()
                shutil.rmtree(self.copy / "bx" / tree)
                done = self.run_script("--check")
                self.assertEqual(done.returncode, 1)
                self.assertEqual(done.stdout.strip().splitlines(),
                                 [f"not installed: the benchmark scenes ({self.copy}/bx)"])

    @unittest.skipUnless(SUPPORTED, "the script runs on macOS on Apple silicon and Linux on x86-64 or arm64")
    def test_an_environment_whose_interpreter_is_gone_is_made_again(self):
        # Both Python environments exist, but their interpreters are links to a Python that was removed. A stand-in
        # uv on PATH refuses to make an environment over an existing folder, as uv does, and records its words.
        # The scenes are current, so the run reaches no network, and step 2 does not run: the new simulator
        # environment gets LIBERO from the fetched tree in step 1.
        self.stamp("scenes", sha_of(self.copy, "benchmarks/fetch.sh", "benchmarks/LIBERO-PRO-data.files.txt",
                                    "benchmarks/LIBERO-PRO-data.sha256"))
        for tree in ("LIBERO", "LIBERO-PRO"):
            (self.copy / "bx" / tree / "libero/libero").mkdir(parents=True)
        gone = pathlib.Path(self.tmp.name) / "removed-python"
        for env in ("bx/libero-env", "venv"):
            (self.copy / env / "bin").mkdir(parents=True)
            (self.copy / env / "bin/python").symlink_to(gone)
        words, done = self.run_with_stand_ins()
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertIn(f"venv --python 3.10 {self.copy}/bx/libero-env", words)
        self.assertIn(f"venv --python 3.12 {self.copy}/venv", words)
        sim_py = f"{self.copy}/bx/libero-env/bin/python"
        libero = (f"pip install --python {sim_py} --no-deps -e {self.copy}/bx/LIBERO "
                  f"--config-settings editable_mode=compat")
        self.assertEqual(words.count(libero), 1, words)
        # After the environment is made and its pinned packages are in it.
        self.assertGreater(words.index(libero), words.index(f"venv --python 3.10 {self.copy}/bx/libero-env"))
        self.assertGreater(words.index(libero), words.index(
            f"pip install --python {sim_py} -r {self.copy}/requirements/sim-py310.txt"))
        self.assertIn("[2/4] the benchmark scenes at their pinned revisions (about 1.6 GB)\n      installed",
                      done.stdout)
        for env_dir in ("bx/libero-env", "venv"):
            python = self.copy / env_dir / "bin/python"
            self.assertFalse(python.is_symlink(), env_dir)
            self.assertTrue(os.access(python, os.X_OK), env_dir)
        self.assertEqual((self.copy / "venv/site/cerulion_harness.pth").read_text(encoding="utf-8").strip(),
                         f"{self.copy}/src")
        # The new simulator environment gets robosuite's settings file with its file log off.
        self.assertEqual((self.copy / "bx/libero-env/site/robosuite/macros_private.py").read_bytes(),
                         (self.copy / "requirements/sim-robosuite-macros.py").read_bytes())

    @unittest.skipUnless(SUPPORTED, "the script runs on macOS on Apple silicon and Linux on x86-64 or arm64")
    def test_with_scenes_to_fetch_the_simulator_step_leaves_the_libero_install_to_the_fetch(self):
        # A LIBERO tree is there but the scenes step is not current (no stamp: an earlier fetch stopped part way):
        # step 1 installs no LIBERO from it, and step 2 fetches again (the git stand-in stops it there).
        (self.copy / "bx/LIBERO/libero/libero").mkdir(parents=True)
        words, done = self.run_with_stand_ins()
        self.assertNotEqual(done.returncode, 0)
        self.assertIn("[2/4] the benchmark scenes", done.stdout)
        self.assertNotIn("installed", done.stdout.split("[2/4]")[1])
        self.assertIn(f"pip install --python {self.copy}/bx/libero-env/bin/python -r "
                      f"{self.copy}/requirements/sim-py310.txt", words)
        self.assertEqual([w for w in words if " -e " in w], [], words)

    @unittest.skipUnless(SUPPORTED, "the script runs on macOS on Apple silicon and Linux on x86-64 or arm64")
    def test_the_body_record_step_runs_without_writing_bytecode(self):
        # The body record imports the measured code into its own process. Every other step is installed by stamp,
        # and a stand-in uv and ldconfig on PATH keep the run off the network and off this machine's library list;
        # the stand-in episode server Python notes its arguments and PYTHONDONTWRITEBYTECODE.
        self.install_everything_by_stamp()
        tools = pathlib.Path(self.tmp.name) / "tools"
        tools.mkdir()
        for name in ("uv", "ldconfig"):
            (tools / name).write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
            (tools / name).chmod(0o755)
        noted = pathlib.Path(self.tmp.name) / "noted.txt"
        (self.copy / "venv/bin/python").write_text(
            '#!/bin/sh\nprintf "%s|%s\\n" "${PYTHONDONTWRITEBYTECODE-unset}" "$*" > "$NOTED"\n', encoding="utf-8")
        env = {k: v for k, v in os.environ.items() if k != "PYTHONDONTWRITEBYTECODE"}
        env.update(PATH=f"{tools}{os.pathsep}{env.get('PATH', '')}", NOTED=str(noted))
        done = subprocess.run(["bash", str(self.copy / "install.sh")], env=env, stdin=subprocess.DEVNULL,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=60)
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertEqual(noted.read_text(encoding="utf-8"), "1|-m cerulion_harness.body_record\n")

    @unittest.skipUnless(SUPPORTED, "the script runs on macOS on Apple silicon and Linux on x86-64 or arm64")
    @unittest.skipIf(hasattr(os, "geteuid") and os.geteuid() == 0, "root can write any folder")
    def test_a_folder_this_user_cannot_write_is_refused_before_anything_is_fetched(self):
        os.chmod(self.copy, stat.S_IRUSR | stat.S_IXUSR)
        done = self.run_script()
        self.assertEqual(done.returncode, 2)
        self.assertIn("which this user cannot write", done.stderr)
        self.assertEqual(done.stdout, "")


# A stand-in robosuite laid out as robosuite 1.4.0 is: robosuite/__init__.py star-imports robosuite.macros_private
# first, and robosuite/utils/log_utils.py opens a FileHandler on its log file unless robosuite.macros sets
# FILE_LOGGING_LEVEL to None. The log file here is inside the test's folder, not /tmp/robosuite.log.
STAND_IN = {
    "robosuite/__init__.py": "try:\n    from robosuite.macros_private import *\nexcept ImportError:\n    pass\n"
                             "import robosuite.utils.log_utils\n",
    "robosuite/macros.py": "CONSOLE_LOGGING_LEVEL = \"WARN\"\nFILE_LOGGING_LEVEL = \"DEBUG\"\n",
    "robosuite/utils/__init__.py": "",
    "robosuite/utils/log_utils.py": "import logging, os\nimport robosuite.macros as macros\n"
                                    "if macros.FILE_LOGGING_LEVEL is not None:\n"
                                    "    logging.getLogger(\"robosuite_logs\").addHandler("
                                    "logging.FileHandler(os.environ[\"STAND_IN_LOG\"]))\n",
}


class RobosuiteFileLog(unittest.TestCase):
    """requirements/sim-robosuite-macros.py, installed as robosuite/macros_private.py, turns robosuite's file log
    off and leaves every other setting as it was."""

    def start(self, private=None):
        """(robosuite.macros' two log settings as printed, which of the names FILE_LOGGING_LEVEL and _macros the
        star import put in the robosuite package, whether the log file exists) after one import of the stand-in,
        with private as its macros_private.py."""
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = pathlib.Path(tmp.name)
        for rel, text in STAND_IN.items():
            (root / rel).parent.mkdir(parents=True, exist_ok=True)
            (root / rel).write_text(text, encoding="utf-8")
        if private is not None:
            (root / "robosuite/macros_private.py").write_text(private, encoding="utf-8")
        log = root / "robosuite.log"
        code = (f"import sys; sys.path.insert(0, {str(root)!r}); import robosuite, robosuite.macros as m; "
                "print(m.FILE_LOGGING_LEVEL, m.CONSOLE_LOGGING_LEVEL); "
                "print([k for k in ('FILE_LOGGING_LEVEL', '_macros') if k in vars(robosuite)])")
        done = subprocess.run([sys.executable, "-I", "-B", "-c", code], env={"STAND_IN_LOG": str(log)},
                              stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                              timeout=60)
        self.assertEqual(done.returncode, 0, done.stderr)
        settings, names = done.stdout.splitlines()
        return settings, names, log.exists()

    def test_without_private_settings_every_start_opens_the_log_file(self):
        self.assertEqual(self.start(), ("DEBUG WARN", "[]", True))

    def test_the_installed_settings_turn_the_file_log_off(self):
        private = (HARNESS / "requirements/sim-robosuite-macros.py").read_text(encoding="utf-8")
        self.assertEqual(self.start(private), ("None WARN", "[]", False))

    def test_a_file_that_only_assigns_the_name_leaves_the_file_log_on(self):
        self.assertEqual(self.start("FILE_LOGGING_LEVEL = None\n"),
                         ("DEBUG WARN", "['FILE_LOGGING_LEVEL']", True))


if __name__ == "__main__":
    unittest.main()
