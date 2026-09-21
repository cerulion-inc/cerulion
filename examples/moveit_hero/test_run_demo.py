#!/usr/bin/env python3
"""Exercise the host launcher and its Docker file mapping without running ROS."""

import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest


def docker_probe(args):
    """Check the paths Docker would consume and record the actual invocation."""
    with open(os.environ["MOVEIT_TEST_CALLS"], "a", encoding="utf-8") as log:
        log.write(json.dumps(args) + "\n")
    if args[0] == "build":
        context = Path(args[-1])
        dockerfile = Path(args[args.index("-f") + 1])
        assert context.is_dir(), f"missing build context: {context}"
        assert dockerfile.is_file(), f"missing Dockerfile: {dockerfile}"
        for line in dockerfile.read_text().splitlines():
            if line.startswith("COPY "):
                for source in shlex.split(line)[1:-1]:
                    assert (context / source).is_file(), f"missing COPY source: {source}"
    elif args[0] == "run":
        host, container = args[args.index("-v") + 1].rsplit(":", 1)
        workdir = args[args.index("-w") + 1]
        assert container == workdir == "/work"
        assert args[-2] == "bash"
        entrypoint = Path(host) / args[-1]
        assert entrypoint.is_file(), f"missing mounted entrypoint: {entrypoint}"
        assert (entrypoint.parent / "launch_move_group.py").is_file()
        assert (entrypoint.parent / "scripted_plan.py").is_file()
    else:
        raise AssertionError(f"unexpected Docker command: {args}")
    return int(os.environ.get(f"MOVEIT_TEST_{args[0].upper()}_EXIT", "0"))


class HostLauncherTest(unittest.TestCase):
    def setUp(self):
        scratch = tempfile.TemporaryDirectory(prefix="moveit launcher ")
        self.addCleanup(scratch.cleanup)
        self.scratch = Path(scratch.name)
        self.repo = self.scratch / "checkout with spaces"
        self.demo = self.repo / "examples" / "moveit_hero"
        shutil.copytree(
            Path(__file__).resolve().parent,
            self.demo,
            ignore=shutil.ignore_patterns("out", "__pycache__"),
        )
        bindir = self.scratch / "bin"
        bindir.mkdir()
        docker = bindir / "docker"
        docker.write_text(
            '#!/bin/sh\nexec "$MOVEIT_TEST_PYTHON" "$MOVEIT_TEST_PROBE" --docker "$@"\n'
        )
        docker.chmod(0o755)
        self.calls = self.scratch / "calls.jsonl"
        self.env = {
            key: value
            for key, value in os.environ.items()
            if not key.startswith(("CER_DEMO_", "MOVEIT_TEST_"))
        }
        self.env.update(
            PATH=f"{bindir}{os.pathsep}{os.environ['PATH']}",
            MOVEIT_TEST_PYTHON=sys.executable,
            MOVEIT_TEST_PROBE=str(Path(__file__).resolve()),
            MOVEIT_TEST_CALLS=str(self.calls),
        )

    def run_launcher(self, *args, **env):
        result = subprocess.run(
            ["bash", str(self.demo / "run_demo.sh"), *args],
            cwd=self.scratch,
            env=self.env | env,
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        calls = [json.loads(line) for line in self.calls.read_text().splitlines()]
        return result, calls

    def test_build_and_run_resolve_from_another_directory(self):
        result, calls = self.run_launcher()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([call[0] for call in calls], ["build", "run"])
        self.assertIn("CER_DEMO_CAPTURE=0", calls[1])
        self.assertIn(f"{self.repo}:/work", calls[1])

    def test_capture_and_custom_image_reach_container(self):
        result, calls = self.run_launcher("--capture", CER_DEMO_IMAGE="local-moveit:test")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("local-moveit:test", calls[0])
        self.assertIn("local-moveit:test", calls[1])
        self.assertIn("CER_DEMO_CAPTURE=1", calls[1])

    def test_prebuilt_image_skips_only_the_build(self):
        result, calls = self.run_launcher(CER_DEMO_NO_BUILD="1")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([call[0] for call in calls], ["run"])

    def test_failed_build_does_not_start_container(self):
        result, calls = self.run_launcher(MOVEIT_TEST_BUILD_EXIT="42")
        self.assertEqual(result.returncode, 42, result.stderr)
        self.assertEqual([call[0] for call in calls], ["build"])

    def test_failed_container_result_reaches_caller(self):
        result, calls = self.run_launcher(MOVEIT_TEST_RUN_EXIT="17")
        self.assertEqual(result.returncode, 17, result.stderr)
        self.assertEqual([call[0] for call in calls], ["build", "run"])


if __name__ == "__main__":
    if sys.argv[1:2] == ["--docker"]:
        sys.exit(docker_probe(sys.argv[2:]))
    unittest.main()
