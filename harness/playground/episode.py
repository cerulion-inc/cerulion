# SPDX-License-Identifier: AGPL-3.0-only
"""One episode: one simulator, its watch, its end, and what its viewer page shows."""
import asyncio
import contextlib
import logging
import os
import secrets
import shutil
import time

from .common import ENDED, OWNER_FILE, cputime, read_json, utc

END_FILE = "episode_end.json"   # written into the episode folder by the episode server when the episode ends
log = logging.getLogger("cerulion.sim")


class Episode:
    """status: starting, ready, then ended (or failed, for a start that failed). reason, once ended: claim (or
    the episode server's own reason), over (the simulator halted it), clock, idle, session_closed,
    server_stopped, start_failed."""

    def __init__(self, pool, state, key):
        self.pool, self.s, self.state, self.key = pool, pool.s, state, key
        self.eid = secrets.token_urlsafe(9)
        self.work_dir = os.path.join(self.s.work_root, self.eid)
        self.ep_dir = os.path.join(self.work_dir, "ep")
        self.status, self.reason, self.why = "starting", None, None
        self.started_m, self.started_utc, self.ended_utc, self.ended_epoch = time.monotonic(), utc(), None, None
        self.ready_s = self.sim_wall_s = None
        # The simulator's processor seconds and peak memory as ps last read them; None until ps has read them
        # (no simulator process yet, or no ps on this machine).
        self.cpu_s, self.peak_rss_mb, self.calls = None, None, 0
        self.last_m = self.started_m
        # Calls in flight; when the episode last became free of them (None until the simulator is ready); every
        # gap between calls; the idle limit that ended the episode, when one did.
        self.in_flight, self.free_m, self.gaps, self.idle_limit_s = 0, None, [], None
        self.bridge, self.practices, self.verdict, self.pids = None, None, None, {}
        self.record = None        # lazy_records: what the check of this start state's body record found or did
        self.events, self.ready, self._end_lock = [], asyncio.Event(), asyncio.Lock()
        self.start_done = asyncio.Event()  # set once the simulator's start has returned or failed
        self.ending = None                 # the reason of the first end asked for, while the end waits for the start
        self._rss, self._cpu, self._ps_failed = {}, {}, False
        os.makedirs(self.work_dir, exist_ok=True)
        with open(os.path.join(self.work_dir, OWNER_FILE), "w", encoding="utf-8") as f:
            f.write(str(os.getpid()))
        self.pool.store.begin(self)
        self.event("status", text=f"simulator starting for start state {state!r}")

    # -- the viewer's record ---------------------------------------------------------------------------------

    def event(self, kind, **fields):
        event = {"n": len(self.events), "t": round(time.monotonic() - self.started_m, 2), "utc": utc(),
                 "kind": kind, **fields}
        self.events.append(event)
        with contextlib.suppress(OSError):
            self.pool.store.append(self.eid, event)

    def keep_frames(self, names):
        """Copy the pictures a result named out of the simulator's folder into the kept record."""
        for name in names:
            source = os.path.join(self.ep_dir, "frames", os.path.basename(name))
            if os.path.isfile(source):
                self.pool.store.add_frame(self.eid, source)

    def elapsed(self):
        return self.sim_wall_s if self.sim_wall_s is not None else time.monotonic() - self.started_m

    def call_begins(self):
        """A tool call or a resource read arrives: the time since the episode was last free of calls is a gap."""
        now = time.monotonic()
        if self.in_flight == 0 and self.free_m is not None:
            self.gaps.append(round(now - self.free_m, 2))
        self.in_flight += 1
        self.last_m = now

    def call_ends(self):
        now = time.monotonic()
        self.in_flight = max(0, self.in_flight - 1)
        self.last_m = now
        if self.in_flight == 0:
            self.free_m = now

    def meta(self):
        """The head of the viewer page."""
        return {"id": self.eid, "state": self.state, "title": self.s.states[self.state].get("title", self.state),
                "status": self.status, "reason": self.reason, "why": self.why, "started": self.started_utc,
                "ended": self.ended_utc, "ended_epoch": self.ended_epoch, "ready_after_s": self.ready_s,
                "simulator_seconds": round(self.elapsed(), 1), "calls": self.calls, "verdict": self.verdict}

    def public(self, since=0):
        return {**self.meta(), "events": self.events[since:], "events_total": len(self.events)}

    # -- start -----------------------------------------------------------------------------------------------

    async def start(self):
        try:
            await self.pool.sim.start(self)
        except Exception as exc:
            failed = str(exc) or type(exc).__name__
        else:
            failed = None
        finally:
            self.start_done.set()
        if failed is not None:
            self.why = failed
            await self.finish("start_failed")
            return
        if self.ending is not None:
            return                    # an end waits for this start; it sets ready once the episode has ended
        self.ready_s = round(time.monotonic() - self.started_m, 1)
        if self.status == "starting":
            self.status = "ready"
            self.free_m = time.monotonic()        # the idle limit and the first gap count from here
            listing = getattr(self.bridge, "listing", None) or {}
            note = self.record_note()
            self.event("status", text=f"simulator ready after {self.ready_s} s; "
                                      f"{len(listing.get('tools', []))} tools and claim" + ("." + note if note else ""))
            self.pool.store.write_meta(self)
        self.ready.set()

    def record_note(self):
        """One sentence when this episode's start measured the robot body's record, else nothing."""
        r = self.record or {}
        if not r.get("measured_now"):
            return ""
        return (f" Before this episode the simulated robot measured itself once, because its body had no "
                f"calibration record on this machine yet: {r.get('self_test_wall_seconds')} s of self-test, "
                f"{r.get('wall_seconds')} s in all. The record is stored; no later episode of this robot body "
                f"pays this again.")

    # -- the watch: memory, processor time, the end file, the clock, idle ------------------------------------

    async def sample(self):
        """Read the simulator processes' memory and processor time with ps. A machine where ps does not run
        leaves both figures as last read (None when never read), with one warning per episode: the watch and the
        end go on without them."""
        pids = [str(p) for p in self.pids.values() if p]
        if not pids:
            return
        try:
            proc = await asyncio.create_subprocess_exec("ps", "-o", "pid=,rss=,time=", "-p", ",".join(pids),
                                                        stdout=asyncio.subprocess.PIPE,
                                                        stderr=asyncio.subprocess.DEVNULL)
            out, _ = await proc.communicate()
        except OSError as exc:
            if not self._ps_failed:
                self._ps_failed = True
                log.warning("episode %s: ps did not run, so its memory and processor time are not measured: %s",
                            self.eid, exc)
            return
        for line in out.decode("utf-8", "replace").splitlines():
            f = line.split()
            if len(f) == 3:
                with contextlib.suppress(ValueError):
                    self._rss[f[0]] = max(self._rss.get(f[0], 0.0), float(f[1]) / 1024.0)
                    self._cpu[f[0]] = cputime(f[2])
        if self._cpu:
            self.cpu_s = round(sum(self._cpu.values()), 1)
        if self._rss:
            self.peak_rss_mb = round(sum(self._rss.values()), 1)

    def end_reason(self):
        """The reason the episode server wrote at its own end, or None while it runs."""
        path = os.path.join(self.ep_dir, END_FILE)
        if not os.path.exists(path):
            return None
        return str(read_json(path).get("reason") or "ended")

    def due(self, now):
        """The reason the watch ends this episode at monotonic time now, or None."""
        ended = self.end_reason() if self.status == "ready" else None
        if ended is not None:
            return ended
        if now - self.started_m >= self.s.clock_s:
            return "clock"
        limit = self.pool.idle_limit()[0]
        if self.status == "ready" and self.in_flight == 0 and self.free_m is not None and limit is not None \
                and now - self.free_m >= limit:
            self.idle_limit_s = limit
            return "idle"
        return None

    async def watch(self, every_s=2.0):
        """Every every_s seconds until the episode ends: measure, then end the episode when it is due. A tick that
        raises is logged (the first one of this episode) and the next tick runs: the watch ends only with the
        episode."""
        warned = False
        while self.status not in ENDED:
            await asyncio.sleep(every_s)
            if self.status in ENDED:
                break
            try:
                await self.sample()
                reason = self.due(time.monotonic())
                if reason is not None:
                    await self.finish(reason)
            except Exception as exc:
                if not warned:
                    warned = True
                    log.warning("episode %s: a check of its watch failed; the watch goes on: %s", self.eid, exc)

    # -- end -------------------------------------------------------------------------------------------------

    def _close_books(self):
        """The end as recorded: the seconds, the history row, the viewer's last lines."""
        self.sim_wall_s = round(time.monotonic() - self.started_m, 1)
        self.ended_utc, self.ended_epoch = utc(), time.time()
        self.status = "failed" if self.reason == "start_failed" else "ended"
        self.pool.history.record({"id": self.eid, "state": self.state, "started": self.started_utc,
                                  "ended": self.ended_utc, "reason": self.reason, "sim_wall_s": self.sim_wall_s,
                                  "sim_cpu_s": self.cpu_s, "peak_rss_mb": self.peak_rss_mb, "calls": self.calls,
                                  "verdict": self.verdict}, self.gaps)
        if self.gaps:
            self.pool.gaps_changed()
        self.event("status", text=self.ended_message())
        if self.verdict:
            self.event("verdict", verdict=self.verdict)
        with contextlib.suppress(OSError):
            self.pool.store.write_meta(self)

    async def _score(self):
        """Measure once more, then read the benchmark's verdict. A score that fails is logged and leaves the
        verdict unset; nothing here raises, so the end goes on to stop the simulator."""
        try:
            await self.sample()
            r = await self.pool.sim.score(self.ep_dir)
            if r:
                self.verdict = {
                    "solved": bool(r.get("success")), "claimed_success": r.get("claimed_success"),
                    "false_claim": bool(r.get("false_claim")), "steps_used": r.get("steps_used"),
                    "first_success_step": r.get("first_success_step"), "budget_steps": r.get("budget_steps"),
                    "within_reference_steps": bool(r.get("success_within_ref")),
                    "reference_steps": r.get("ref_max_steps"), "end_reason": r.get("end_reason"),
                    "source": "the benchmark's own success check, read from the simulator's truth log"}
        except Exception as exc:
            log.warning("episode %s was not scored: %s", self.eid, exc)

    async def finish(self, reason):
        """End the episode once: score it, stop the simulator, record it, clear its work folder. An end that
        comes while the simulator is starting waits for the start to return first (a body record being measured
        stops at once), so the stop finds every process the start made. A score that fails never skips the
        stop."""
        if self.ending is None:
            self.ending = reason
        if reason != "start_failed":
            await self.start_done.wait()
        async with self._end_lock:
            if self.status in ENDED:
                return
            started = self.status != "starting" or os.path.exists(self.ep_dir)
            self.reason = reason
            if reason != "start_failed":
                await self._score()
            if started:
                with contextlib.suppress(Exception):
                    await self.pool.sim.stop(self.ep_dir)
            self._close_books()
            if not self.s.keep_work:
                await asyncio.to_thread(shutil.rmtree, self.work_dir, True)
            self.pool.retire(self)
            self.ready.set()

    def verdict_note(self):
        """The benchmark's verdict as one sentence, for an episode that was scored; else nothing."""
        v = self.verdict
        if not v:
            return ""
        if v.get("solved"):
            return " The benchmark's own success check says the task is solved."
        if v.get("claimed_success"):
            return " The benchmark's own success check says the task is not solved, although the claim said it was."
        return " The benchmark's own success check says the task is not solved."

    def ended_message(self):
        """What a call is answered once the episode has ended. The verdict is in it once the episode was scored,
        so a client learns the benchmark's own check from the answer to the call that ended the episode."""
        ran = f" The simulator ran for {self.sim_wall_s:g} s." if self.sim_wall_s is not None else ""
        again = " " + self.pool.again
        if self.reason == "start_failed":
            return f"The simulator did not start: {self.why}"
        if self.reason == "idle":
            limit = f"{self.idle_limit_s:g} s" if self.idle_limit_s is not None else "the idle limit"
            return (f"The episode has ended: no call arrived for {limit}, the idle limit, so the simulator was "
                    "stopped." + self.verdict_note() + ran + again)
        if self.reason == "over":   # the simulator's own halt: at the first success, or at the step budget
            v = self.verdict or {}
            if v.get("solved"):
                return (f"The episode has ended: the benchmark's own success check turned true at control step "
                        f"{v.get('first_success_step')}, and the simulator halted the episode there. The task "
                        f"is solved; no claim is needed." + ran + again)
            return ("The episode has ended: the simulator halted it at its budget of control steps, and the "
                    "benchmark's success check had not turned true." + ran + again)
        why = {"claim": "the claim was recorded", "clock": f"the {self.s.clock_s:g} s episode clock ran out",
               "session_closed": "the client closed its session", "server_stopped": "the sim server was stopped"}
        return f"The episode has ended: {why.get(self.reason, self.reason)}." + self.verdict_note() + ran + again
