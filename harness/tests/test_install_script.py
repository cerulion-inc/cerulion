# SPDX-License-Identifier: AGPL-3.0-only
"""install.sh: its flags, and the stamps that let a finished install return at once. Standard library only, no
network: every run here stops before the first download. The expected stamps are computed here from the rule
the script states (the SHA-256 of a step's input files), not read from the script's own output."""
import hashlib
import os
import pathlib
import platform
import shutil
import stat
import subprocess
import tempfile
import unittest

HARNESS = pathlib.Path(__file__).resolve().parent.parent
INPUTS = ("install.sh", "requirements/sim-py310.txt", "requirements/sim-torch-cpu.txt",
          "requirements/server-py312.txt", "benchmarks/fetch.sh", "benchmarks/LIBERO-PRO-data.files.txt",
          "benchmarks/LIBERO-PRO-data.sha256")
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
        self.stamp("sim", sha_of(self.copy, "requirements/sim-py310.txt", "requirements/sim-torch-cpu.txt"))
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
        (self.copy / "bx/LIBERO-PRO/libero/libero").mkdir(parents=True)

    def test_help_prints_the_usage(self):
        done = self.run_script("--help")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertIn("install.sh [--bench-from DIR] [--check]", done.stdout)

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
    def test_a_changed_pin_makes_its_step_stale(self):
        self.install_everything_by_stamp()
        with open(self.copy / "requirements/server-py312.txt", "a", encoding="utf-8") as f:
            f.write("# one more line\n")
        done = self.run_script("--check")
        self.assertEqual(done.returncode, 1)
        self.assertEqual(done.stdout.strip().splitlines(),
                         [f"not installed: the episode server environment ({self.copy}/venv)"])

    @unittest.skipUnless(SUPPORTED, "the script runs on macOS on Apple silicon and Linux on x86-64 or arm64")
    def test_an_environment_whose_interpreter_is_gone_is_made_again(self):
        # Both Python environments exist, but their interpreters are links to a Python that was removed. A stand-in
        # uv on PATH refuses to make an environment over an existing folder, as uv does, and records its words.
        # The scenes are current, so the run reaches no network.
        self.stamp("scenes", sha_of(self.copy, "benchmarks/fetch.sh", "benchmarks/LIBERO-PRO-data.files.txt",
                                    "benchmarks/LIBERO-PRO-data.sha256"))
        (self.copy / "bx/LIBERO-PRO/libero/libero").mkdir(parents=True)
        gone = pathlib.Path(self.tmp.name) / "removed-python"
        for env in ("bx/libero-env", "venv"):
            (self.copy / env / "bin").mkdir(parents=True)
            (self.copy / env / "bin/python").symlink_to(gone)
        mac_gl = "macos/cerulion_harness_macos_gl.py"
        (self.copy / mac_gl).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(HARNESS / mac_gl, self.copy / mac_gl)
        tools = pathlib.Path(self.tmp.name) / "tools"
        tools.mkdir()
        log = pathlib.Path(self.tmp.name) / "uv.log"
        (tools / "uv").write_text(
            '#!/bin/sh\n'
            'echo "$*" >> "$UV_LOG"\n'
            '[ "$1" = venv ] || exit 0\n'
            'for env; do :; done\n'
            '[ -e "$env" ] && { echo "A virtual environment already exists at $env" >&2; exit 2; }\n'
            'mkdir -p "$env/bin" "$env/site"\n'
            'printf \'#!/bin/sh\\n[ "$1" = -c ] && echo "%s/site"\\nexit 0\\n\' "$env" > "$env/bin/python"\n'
            'chmod +x "$env/bin/python"\n', encoding="utf-8")
        # The renderer check reads ldconfig; this stand-in lists the renderer, as a machine that has it does.
        (tools / "ldconfig").write_text('#!/bin/sh\necho "libOSMesa.so.8 (libc6) => /usr/lib/libOSMesa.so.8"\n',
                                        encoding="utf-8")
        for tool in ("uv", "ldconfig"):
            (tools / tool).chmod(0o755)
        env = dict(os.environ, PATH=f"{tools}{os.pathsep}{os.environ['PATH']}", UV_LOG=str(log))
        done = subprocess.run(["bash", str(self.copy / "install.sh")], stdin=subprocess.DEVNULL, env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=60)
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        words = log.read_text(encoding="utf-8").splitlines()
        self.assertIn(f"venv --python 3.10 {self.copy}/bx/libero-env", words)
        self.assertIn(f"venv --python 3.12 {self.copy}/venv", words)
        for env_dir in ("bx/libero-env", "venv"):
            python = self.copy / env_dir / "bin/python"
            self.assertFalse(python.is_symlink(), env_dir)
            self.assertTrue(os.access(python, os.X_OK), env_dir)
        self.assertEqual((self.copy / "venv/site/cerulion_harness.pth").read_text(encoding="utf-8").strip(),
                         f"{self.copy}/src")

    @unittest.skipUnless(SUPPORTED, "the script runs on macOS on Apple silicon and Linux on x86-64 or arm64")
    @unittest.skipIf(hasattr(os, "geteuid") and os.geteuid() == 0, "root can write any folder")
    def test_a_folder_this_user_cannot_write_is_refused_before_anything_is_fetched(self):
        os.chmod(self.copy, stat.S_IRUSR | stat.S_IXUSR)
        done = self.run_script()
        self.assertEqual(done.returncode, 2)
        self.assertIn("which this user cannot write", done.stderr)
        self.assertEqual(done.stdout, "")


if __name__ == "__main__":
    unittest.main()
