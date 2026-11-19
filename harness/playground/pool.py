# SPDX-License-Identifier: AGPL-3.0-only
"""The pool: one simulator per episode, started on demand, and what the server holds of them."""
import asyncio
import contextlib
import logging
import os
import shutil

from .common import ENDED, OWNER_FILE, pid_alive
from .episode import Episode
from .history import History
from .simulator import Simulator
from .store import Store

log = logging.getLogger("cerulion.sim")
AGAIN_HTTP = "Connect again: a new connection starts a new episode."
AGAIN_STDIO = "To run again, list the tools again or restart the server: either starts a new episode."


class Refused(Exception):
    """A request the server refuses, with a message for the person."""


class Pool:
    """again: the sentence an ended episode ends with, which says how this transport starts the next one."""

    def __init__(self, settings, history, store, simulator, again=AGAIN_HTTP):
        self.s, self.history, self.store, self.sim, self.again = settings, history, store, simulator, again
        self.episodes, self.by_key, self._tasks = {}, {}, set()
        self.stop_reason, self._closing, self._idle = "server_stopped", None, None

    def idle_limit(self):
        """(seconds, where the figure comes from), or (None, why) when no idle limit is set. The operator's own
        idle_s wins. Else idle_reap: factor times the stated percentile of the gaps between calls, or times the
        history's own percentile once the history holds more gaps than the stated source and its percentile is
        the longer one. The history can lengthen the limit and never shortens it: its gaps are every call's, and a
        client that sends calls in a quick loop (a script reading a resource) fills it with gaps of milliseconds."""
        if self._idle is None:
            reap = self.s.idle_reap
            if self.s.idle_s is not None:
                self._idle = (float(self.s.idle_s), "idle_s, stated in the settings")
            elif not reap:
                self._idle = (None, "no idle limit is set")
            else:
                factor, pct, held = float(reap["factor"]), float(reap["percentile"]), self.history.gap_count()
                stated = float(reap["gap_at_percentile_s"])
                p = self.history.gap_percentile(pct) if held > int(reap["gaps_counted"]) else None
                if p is not None and p >= stated:
                    self._idle = (round(factor * p, 1), f"{factor:g} times the {pct:g}th percentile ({p:g} s) of "
                                                        f"the {held} gaps between calls in this server's history")
                else:
                    held_note = (f"this server's history holds {held}" if p is None else
                                 f"this server's history of {held} gaps puts it at {p:g} s, and a history never "
                                 "shortens the limit")
                    self._idle = (round(factor * stated, 1), f"{factor:g} times the {pct:g}th percentile "
                                                             f"({stated:g} s) of {int(reap['gaps_counted'])} gaps "
                                                             f"between calls: {reap['source']} ({held_note})")
        return self._idle

    def gaps_changed(self):
        self._idle = None

    @property
    def stopping(self):
        """True once the server has begun to stop: no new episode starts."""
        return self._closing is not None

    def live(self):
        return [e for e in self.episodes.values() if e.status not in ENDED]

    def spawn(self, coro):
        task = asyncio.ensure_future(coro)
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)
        return task

    def open(self, state, key):
        """A new episode of start state state. key: what a connection with no session names it by, in by_key;
        None for an MCP session's own episode, which only that session reaches."""
        if state not in self.s.states:
            raise Refused(f"no start state {state!r}; the start states offered: {', '.join(self.s.states)}")
        if self.stopping:
            raise Refused("The sim server is stopping, so no simulator was started.")
        if self.s.pool_size is not None and len(self.live()) >= self.s.pool_size:
            raise Refused(f"All {self.s.pool_size} simulators of this server are in use (pool_size); connect again "
                          "when one has ended.")
        ep = Episode(self, state, key)
        self.episodes[ep.eid] = ep
        if key is not None:
            self.by_key[key] = ep
        self.spawn(ep.start())
        self.spawn(ep.watch())
        return ep

    def retire(self, ep):
        """An ended episode leaves the server's memory: its page is served from the kept record."""
        self.episodes.pop(ep.eid, None)
        ep.events = ep.events[-2:]

    async def end_all(self, reason):
        """End every live episode; an end that raises is logged by episode, and the others still end."""
        live = self.live()
        done = await asyncio.gather(*(ep.finish(reason) for ep in live), return_exceptions=True)
        for ep, result in zip(live, done):
            if isinstance(result, BaseException):
                log.warning("episode %s did not end cleanly: %s", ep.eid, result)

    def close(self):
        """The server is stopping: end every live episode, once, with the reason the stop carries."""
        if self._closing is None:
            self._closing = self.spawn(self.end_all(self.stop_reason))
        return self._closing

    async def clear_orphans(self):
        """At start: work folders left by a server that was killed (their owner process is gone). Their
        simulators are stopped and the folders removed. Returns how many."""
        cleared = 0
        for name in os.listdir(self.s.work_root) if os.path.isdir(self.s.work_root) else []:
            folder = os.path.join(self.s.work_root, name)
            try:
                with open(os.path.join(folder, OWNER_FILE), encoding="utf-8") as f:
                    owner = f.read().strip()
            except OSError:
                continue                      # not a folder this server made
            if pid_alive(owner):
                continue
            if os.path.isdir(os.path.join(folder, "ep")):
                try:
                    await self.sim.stop(os.path.join(folder, "ep"))
                except Exception as exc:
                    log.warning("the simulator of the work folder %s, left by a stopped server, did not stop: %s",
                                folder, exc)
            await asyncio.to_thread(shutil.rmtree, folder, True)
            cleared += 1
        return cleared

    async def sweep_loop(self):
        while True:
            with contextlib.suppress(Exception):
                await asyncio.to_thread(self.store.sweep, None, tuple(self.episodes))
            await asyncio.sleep(self.s.sweep_every_s)


def make_pool(settings, simulator=None, again=AGAIN_HTTP):
    """The pool of one server: its history file and kept record under the settings' folders."""
    os.makedirs(settings.work_root, exist_ok=True)
    return Pool(settings, History(os.path.join(settings.data_dir, "episodes.sqlite")), Store(settings),
                simulator or Simulator(settings), again)
