# SPDX-License-Identifier: AGPL-3.0-only
"""The simulator of one episode, started, scored and stopped through the measured snapshot's own launcher."""
import asyncio
import contextlib
import json
import os
import signal

from .common import ENDED, read_json

SURFACE = "vnext"


class Simulator:
    """Starts, scores and stops one episode's simulator with the measured snapshot's own verbs
    (python -m bench.ext.episode_ext start | score | stop). A test hands the pool another object with the same
    three coroutines."""

    def __init__(self, settings):
        self.s = settings
        self._record_lock = None

    def env(self, base=None):
        """The environment of every process the simulator runs."""
        env = dict(os.environ if base is None else base)
        env.update({"PYTHONDONTWRITEBYTECODE": "1", "PYTHONPATH": self.s.snapshot_root,
                    "LIBERO_BX": self.s.bench_root, "RCPX_SELFKNOW_DIR": self.s.body_records})
        if self.s.envhost:
            # macOS: the simulator host starts through the macOS host script, which renders with Apple's OpenGL;
            # a renderer named by the shell would send it to the CPU renderer, which does not load there
            env.pop("MUJOCO_GL", None)
            env.pop("PYOPENGL_PLATFORM", None)
            env["RCPX_EXT_ENVHOST"] = json.dumps(self.s.envhost)
            env["PLAYGROUND_SNAPSHOT"] = self.s.snapshot_root
        else:
            env["MUJOCO_GL"] = self.s.gl
        env.update(self.s.env)
        return env

    async def _run(self, *argv):
        proc = await asyncio.create_subprocess_exec(
            self.s.python, "-m", "bench.ext.episode_ext", *argv, cwd=self.s.snapshot_root, env=self.env(),
            stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        out, _ = await proc.communicate()
        return proc.returncode, out.decode("utf-8", "replace")

    async def body_record(self, ep):
        """The setting lazy_records: the record of the robot body this start state needs, found or measured now.
        One at a time, so two first episodes of one body measure it once. Sets ep.record; raises RuntimeError
        with the reason when there is still no record."""
        cell = self.s.states[ep.state]["cell"].split()[0]
        if self._record_lock is None:
            self._record_lock = asyncio.Lock()
        async with self._record_lock:
            proc = await asyncio.create_subprocess_exec(
                self.s.python, "-m", self.s.body_record_module, "--for-cell", cell, "--records",
                self.s.body_records, "--work", os.path.join(ep.work_dir, "b"), cwd=self.s.snapshot_root,
                env=self.env(), stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE)
            out = asyncio.ensure_future(proc.stdout.read())
            said, stopped = [], False
            while True:
                try:
                    line = await asyncio.wait_for(proc.stderr.readline(), 1.0)
                except asyncio.TimeoutError:
                    line = None
                if (ep.status in ENDED or ep.ending is not None) and not stopped:   # an end came: stop cleanly
                    stopped = True
                    with contextlib.suppress(ProcessLookupError):
                        proc.send_signal(signal.SIGINT)
                if line is None:
                    continue
                if not line:
                    break
                text = line.decode("utf-8", "replace").rstrip()
                if text and not text.startswith(" "):      # the self-test's own step lines are indented
                    said.append(text)
                    ep.event("status", text=text[:400])
            rc = await proc.wait()
            try:
                row = json.loads((await out).decode("utf-8", "replace"))["rows"][0]
            except (ValueError, KeyError, IndexError, TypeError):
                row = {"problem": " ".join(said[-2:]) or f"python -m {self.s.body_record_module} ended with {rc}"}
        if stopped:
            raise RuntimeError("the episode ended while the robot body's record was being checked")
        if rc != 0 or row.get("problem"):
            raise RuntimeError(f"the robot body of start state {ep.state!r} has no calibration record on this "
                               f"machine and measuring one failed: {row.get('problem')}")
        ep.record = {k: row.get(k) for k in ("record", "measured_now", "checked", "key", "measured_in",
                                             "self_test_wall_seconds", "robot_seconds", "control_periods",
                                             "wall_seconds")}

    async def start(self, ep):
        """Sets ep.pids, ep.bridge and ep.practices; raises RuntimeError with the reason when it cannot."""
        from rcpx.mcp_stdio import Bridge
        if self.s.lazy_records:
            await self.body_record(ep)
        cell = f"{self.s.states[ep.state]['cell']} {SURFACE} {self.s.packages}"
        rc, out = await self._run("start", "--gl", self.s.gl, "--cell", cell, "--ep", ep.ep_dir,
                                  "--budget-requests", str(self.s.budget_requests),
                                  "--budget-steps", str(self.s.budget_steps))
        if rc != 0:
            raise RuntimeError("the simulator did not start: " + " ".join(out.strip().splitlines()[-2:]))
        absent = read_json(os.path.join(ep.ep_dir, "r2_problems.json")).get("absent")
        if absent:
            raise RuntimeError(f"the episode server did not load: {', '.join(map(str, absent))}")
        ep.pids = {"episode_server": read_json(os.path.join(ep.ep_dir, "episode.json")).get("pid"),
                   "env_host": read_json(os.path.join(os.path.dirname(ep.ep_dir), "_scoring", "ep",
                                                      "episode.json")).get("env_pid")}
        ep.bridge = Bridge(ep.ep_dir, self.s.call_timeout_s)
        ep.practices = await asyncio.to_thread(ep.bridge.start)
        if ep.bridge.listing is None:
            raise RuntimeError("the episode server gave no tool list")

    async def score(self, ep_dir):
        """The benchmark's own verdict, read from the simulator's truth log; {} when there is none."""
        rc, out = await self._run("score", "--ep", ep_dir)
        try:
            return json.loads(out[out.index("{"):]) if rc == 0 else {}
        except ValueError:
            return {}

    async def stop(self, ep_dir):
        await self._run("stop", "--ep", ep_dir)
