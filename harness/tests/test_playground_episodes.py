# SPDX-License-Identifier: AGPL-3.0-only
"""Episodes and the pool with an in-process simulator that runs nothing: start, every end and its message,
the idle limit and where its figure comes from, the gaps between calls, and the work folders a stopped server
left. Standard library only."""
import asyncio
import json
import os
import subprocess
import sys
import unittest
from unittest import mock

from playground_support import FakeSim, Folder, write_settings

from playground import episode as episode_mod
from playground.history import History
from playground.pool import AGAIN_HTTP, AGAIN_STDIO, Pool, Refused, make_pool
from playground.settings import Settings
from playground.common import OWNER_FILE

SOLVED = {"success": True, "claimed_success": True, "false_claim": False, "steps_used": 150,
          "first_success_step": 150, "budget_steps": 900, "success_within_ref": True, "ref_max_steps": 220,
          "end_reason": "claim"}


class Clock:
    """time.monotonic for the episode module, moved by hand."""

    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now


class Episodes(unittest.TestCase):
    def setUp(self):
        self.folder = Folder()
        self.addCleanup(self.folder.remove)

    def pool(self, sim=None, again=AGAIN_HTTP, **changes):
        s = Settings(write_settings(self.folder, **changes), platform="linux")
        return make_pool(s, sim or FakeSim(SOLVED), again)

    def run_async(self, coro):
        return asyncio.run(coro)

    def test_an_episode_starts_ends_at_the_claim_and_leaves_its_record(self):
        async def go():
            pool = self.pool()
            ep = pool.open("bowl-on-plate", "bowl-on-plate:1")
            await ep.ready.wait()
            self.assertEqual(ep.status, "ready")
            os.makedirs(ep.ep_dir, exist_ok=True)
            with open(os.path.join(ep.ep_dir, episode_mod.END_FILE), "w", encoding="utf-8") as f:
                json.dump({"reason": "claim"}, f)
            self.assertEqual(ep.due(ep.started_m + 1), "claim")
            await ep.finish(ep.end_reason())
            return pool, ep
        pool, ep = self.run_async(go())
        self.assertEqual((ep.status, ep.reason), ("ended", "claim"))
        self.assertEqual(ep.verdict["solved"], True)
        self.assertEqual(ep.verdict["first_success_step"], 150)
        self.assertFalse(os.path.exists(ep.work_dir))                         # the simulator's files are gone
        self.assertNotIn(ep.eid, pool.episodes)                               # and the server holds none of it
        kept = pool.store.load(ep.eid)
        self.assertEqual((kept["status"], kept["reason"]), ("ended", "claim"))
        kinds = [e["kind"] for e in kept["events"]]
        self.assertEqual(kinds, ["status", "status", "status", "verdict"])
        self.assertEqual(kept["events"][1]["text"], f"simulator ready after {ep.ready_s} s; 1 tools and claim")
        self.assertEqual([r["id"] for r in pool.history.episodes()], [ep.eid])
        self.assertEqual([c[0] for c in pool.sim.calls], ["start", "score", "stop"])

    def test_a_start_that_fails_is_failed_with_the_reason_and_never_scored(self):
        async def go():
            pool = self.pool(FakeSim(fail_start="the scene is missing"))
            ep = pool.open("open-drawer", "open-drawer:1")
            await ep.ready.wait()
            return pool, ep
        pool, ep = self.run_async(go())
        self.assertEqual((ep.status, ep.reason), ("failed", "start_failed"))
        self.assertEqual(ep.ended_message(), "The simulator did not start: the scene is missing")
        self.assertEqual([c[0] for c in pool.sim.calls], ["start"])            # nothing was started to stop

    def test_an_end_that_comes_during_the_start_waits_for_it_then_stops_what_started(self):
        class SlowSim(FakeSim):
            def __init__(self):
                super().__init__(SOLVED)
                self.gate = asyncio.Event()

            async def start(self, ep):
                self.calls.append(("start begins", ep.eid))
                await self.gate.wait()
                await super().start(ep)

        async def go():
            sim = SlowSim()
            pool = self.pool(sim)
            ep = pool.open("bowl-on-plate", "k")
            await asyncio.sleep(0)
            stop = asyncio.ensure_future(pool.close())
            await asyncio.sleep(0.05)
            self.assertEqual((ep.status, ep.ending), ("starting", "server_stopped"))
            self.assertFalse(ep.ready.is_set())
            sim.gate.set()
            await stop
            return sim, ep
        sim, ep = self.run_async(go())
        self.assertEqual([c[0] for c in sim.calls], ["start begins", "start", "score", "stop"])
        self.assertEqual((ep.status, ep.reason), ("ended", "server_stopped"))
        self.assertTrue(ep.ready.is_set())

    def without_ps(self):
        """PATH holds only an empty folder: ps does not run, as on a machine without it."""
        empty = self.folder.at("empty-path")
        os.makedirs(empty, exist_ok=True)
        return mock.patch.dict(os.environ, {"PATH": empty})

    def test_an_end_where_ps_does_not_run_still_scores_and_stops_the_simulator_and_records_no_figures(self):
        async def go():
            pool = self.pool()
            ep = pool.open("bowl-on-plate", "k")
            await ep.ready.wait()
            ep.pids = {"episode_server": os.getpid()}
            with self.without_ps(), self.assertLogs("cerulion.sim", "WARNING") as logs:
                await ep.finish("claim")
            return pool, ep, logs
        pool, ep, logs = self.run_async(go())
        self.assertEqual((ep.status, ep.reason, ep.verdict["solved"]), ("ended", "claim", True))
        self.assertEqual([c[0] for c in pool.sim.calls], ["start", "score", "stop"])
        self.assertFalse(os.path.exists(ep.work_dir))
        self.assertTrue(ep.ready.is_set())
        row = pool.history.episodes()[0]
        self.assertEqual((row["sim_cpu_s"], row["peak_rss_mb"]), (None, None))     # not measured, not 0
        self.assertEqual(len(logs.output), 1)
        self.assertIn(f"episode {ep.eid}: ps did not run, so its memory and processor time are not measured",
                      logs.output[0])

    def test_ps_reads_a_live_process_and_a_process_that_is_gone_leaves_the_figures_unset(self):
        async def go():
            pool = self.pool()
            live, gone = pool.open("bowl-on-plate", "a"), pool.open("open-drawer", "b")
            await live.ready.wait()
            await gone.ready.wait()
            ended = subprocess.Popen([sys.executable, "-c", ""])
            ended.wait()
            live.pids, gone.pids = {"episode_server": os.getpid()}, {"episode_server": ended.pid}
            await live.sample()
            await gone.sample()
            return live, gone
        live, gone = self.run_async(go())
        self.assertGreater(live.peak_rss_mb, 0.0)
        self.assertGreaterEqual(live.cpu_s, 0.0)
        self.assertEqual((gone.cpu_s, gone.peak_rss_mb), (None, None))

    def test_the_watch_where_ps_does_not_run_goes_on_and_ends_the_episode_at_its_clock(self):
        async def go():
            pool = self.pool(clock_s=0.3)
            ep = pool.open("bowl-on-plate", "k")
            await ep.ready.wait()
            ep.pids = {"episode_server": os.getpid()}
            with self.without_ps(), self.assertLogs("cerulion.sim", "WARNING") as logs:
                await asyncio.wait_for(ep.watch(every_s=0.05), 10)
            return pool, ep, logs
        pool, ep, logs = self.run_async(go())
        self.assertEqual((ep.status, ep.reason), ("ended", "clock"))
        self.assertEqual([c[0] for c in pool.sim.calls], ["start", "score", "stop"])
        self.assertEqual(len(logs.output), 1)                                     # once, not once per tick

    def test_a_watch_check_that_raises_is_logged_once_and_the_clock_still_ends_the_episode(self):
        async def go():
            pool = self.pool(clock_s=0.3)
            ep = pool.open("bowl-on-plate", "k")
            await ep.ready.wait()
            with mock.patch.object(pool, "idle_limit", side_effect=RuntimeError("history unreadable")), \
                    self.assertLogs("cerulion.sim", "WARNING") as logs:
                await asyncio.wait_for(ep.watch(every_s=0.05), 10)
            return pool, ep, logs
        pool, ep, logs = self.run_async(go())
        self.assertEqual((ep.status, ep.reason), ("ended", "clock"))
        self.assertEqual([c[0] for c in pool.sim.calls], ["start", "score", "stop"])
        self.assertEqual(len(logs.output), 1)
        self.assertIn(f"episode {ep.eid}: a check of its watch failed; the watch goes on: history unreadable",
                      logs.output[0])

    def test_a_score_that_fails_leaves_no_verdict_and_still_stops_the_simulator(self):
        class NoScore(FakeSim):
            async def score(self, ep_dir):
                self.calls.append(("score", ep_dir))
                raise FileNotFoundError("no python at the settings' path")

        async def go():
            pool = self.pool(NoScore())
            ep = pool.open("bowl-on-plate", "k")
            await ep.ready.wait()
            with self.assertLogs("cerulion.sim", "WARNING") as logs:
                await ep.finish("claim")
            return pool, ep, logs
        pool, ep, logs = self.run_async(go())
        self.assertEqual((ep.status, ep.reason, ep.verdict), ("ended", "claim", None))
        self.assertEqual([c[0] for c in pool.sim.calls], ["start", "score", "stop"])
        self.assertEqual(ep.ended_message(), f"The episode has ended: the claim was recorded. The simulator ran for "
                                             f"{ep.sim_wall_s:g} s. {AGAIN_HTTP}")
        self.assertIn(f"episode {ep.eid} was not scored: no python at the settings' path", logs.output[0])

    def test_a_stop_logs_an_end_that_raised_and_still_ends_every_other_episode(self):
        async def go():
            pool = self.pool()
            first, second = pool.open("bowl-on-plate", "a"), pool.open("open-drawer", "b")
            await first.ready.wait()
            await second.ready.wait()
            with mock.patch.object(first, "_close_books", side_effect=RuntimeError("the disk is full")), \
                    self.assertLogs("cerulion.sim", "WARNING") as logs:
                await pool.close()
            return pool, first, second, logs
        pool, first, second, logs = self.run_async(go())
        self.assertEqual(logs.output, [f"WARNING:cerulion.sim:episode {first.eid} did not end cleanly: the disk is "
                                       "full"])
        self.assertEqual((second.status, second.reason), ("ended", "server_stopped"))
        self.assertEqual(sorted(c for c in pool.sim.calls if c[0] == "stop"),
                         sorted([("stop", first.ep_dir), ("stop", second.ep_dir)]))

    def test_the_pool_refuses_an_unknown_state_a_full_pool_and_a_stopping_server(self):
        async def go():
            pool = self.pool(pool_size=1)
            with self.assertRaises(Refused) as caught:
                pool.open("kitchen", "kitchen:1")
            self.assertIn("no start state 'kitchen'", str(caught.exception))
            first = pool.open("bowl-on-plate", "a")
            await first.ready.wait()
            with self.assertRaises(Refused) as caught:
                pool.open("bowl-on-plate", "b")
            self.assertIn("All 1 simulators of this server are in use", str(caught.exception))
            await pool.close()
            self.assertEqual(first.reason, "server_stopped")
            with self.assertRaises(Refused) as caught:
                pool.open("bowl-on-plate", "c")
            self.assertIn("stopping", str(caught.exception))
        self.run_async(go())

    def test_a_sessions_episode_is_kept_out_of_the_keys_other_connections_look_up(self):
        async def go():
            pool = self.pool()
            own = pool.open("bowl-on-plate", None)
            keyed = pool.open("bowl-on-plate", "bowl-on-plate:1")
            await own.ready.wait()
            await keyed.ready.wait()
            return pool, own, keyed
        pool, own, keyed = self.run_async(go())
        self.assertEqual(pool.by_key, {"bowl-on-plate:1": keyed})
        self.assertEqual(sorted(pool.episodes), sorted([own.eid, keyed.eid]))

    def test_every_end_reason_has_its_own_message(self):
        async def go():
            pool = self.pool(again=AGAIN_STDIO)
            ep = pool.open("bowl-on-plate", "k")
            await ep.ready.wait()
            await ep.finish("clock")
            return ep
        ep = self.run_async(go())
        self.assertEqual(ep.verdict["solved"], True)
        ep.verdict = None                         # an episode that could not be scored
        ran = f" The simulator ran for {ep.sim_wall_s:g} s. "
        cases = {"clock": "The episode has ended: the 5400 s episode clock ran out." + ran + AGAIN_STDIO,
                 "claim": "The episode has ended: the claim was recorded." + ran + AGAIN_STDIO,
                 "session_closed": "The episode has ended: the client closed its session." + ran + AGAIN_STDIO,
                 "server_stopped": "The episode has ended: the sim server was stopped." + ran + AGAIN_STDIO,
                 "budget": "The episode has ended: budget." + ran + AGAIN_STDIO}
        for reason, text in cases.items():
            ep.reason = reason
            self.assertEqual(ep.ended_message(), text)
        ep.reason, ep.idle_limit_s = "idle", 430.2
        self.assertEqual(ep.ended_message(), "The episode has ended: no call arrived for 430.2 s, the idle limit, "
                                             "so the simulator was stopped." + ran + AGAIN_STDIO)
        ep.reason, ep.verdict = "over", {"solved": True, "first_success_step": 136}
        self.assertIn("turned true at control step 136", ep.ended_message())
        ep.verdict = {"solved": False}
        self.assertIn("at its budget of control steps", ep.ended_message())

    def test_an_ended_message_carries_the_verdict_of_a_scored_episode(self):
        async def go():
            pool = self.pool(again=AGAIN_HTTP)
            ep = pool.open("bowl-on-plate", "k")
            await ep.ready.wait()
            await ep.finish("claim")
            return ep
        ep = self.run_async(go())
        ran = f" The simulator ran for {ep.sim_wall_s:g} s. "
        check = " The benchmark's own success check says the task is "
        cases = [({"solved": True, "claimed_success": True}, "solved."),
                 ({"solved": False, "claimed_success": True, "false_claim": True},
                  "not solved, although the claim said it was."),
                 ({"solved": False, "claimed_success": False}, "not solved.")]
        for verdict, says in cases:
            ep.verdict = verdict
            self.assertEqual(ep.ended_message(), "The episode has ended: the claim was recorded." + check + says + ran
                             + AGAIN_HTTP)
        ep.reason, ep.idle_limit_s, ep.verdict = "idle", 430.2, {"solved": False, "claimed_success": None}
        self.assertEqual(ep.ended_message(), "The episode has ended: no call arrived for 430.2 s, the idle limit, "
                                             "so the simulator was stopped." + check + "not solved." + ran + AGAIN_HTTP)

    def test_gaps_count_only_the_time_with_no_call_in_flight(self):
        clock = Clock()

        async def go():
            with mock.patch.object(episode_mod.time, "monotonic", clock):
                pool = self.pool()
                ep = pool.open("bowl-on-plate", "k")
                await ep.ready.wait()           # free from 1000.0
                clock.now = 1004.0
                ep.call_begins()                # gap 4.0
                clock.now = 1005.0
                ep.call_begins()                # a second call while one runs: no gap
                clock.now = 1006.0
                ep.call_ends()
                clock.now = 1007.5
                ep.call_ends()                  # free from 1007.5
                clock.now = 1010.0
                ep.call_begins()                # gap 2.5
                ep.call_ends()
                await ep.finish("claim")
                return pool, ep
        pool, ep = self.run_async(go())
        self.assertEqual(ep.gaps, [4.0, 2.5])
        self.assertEqual(pool.history.gap_count(), 2)

    def test_the_idle_limit_comes_from_the_stated_source_until_the_history_holds_more_and_longer_gaps(self):
        pool = self.pool()
        self.assertEqual(pool.idle_limit()[0], 430.2)                          # 10 x 43.02 s
        self.assertIn("2386 gaps between calls: 72 model-driven episodes", pool.idle_limit()[1])
        self.assertTrue(pool.idle_limit()[1].endswith("(this server's history holds 0)"))
        pool.s.idle_reap = dict(pool.s.idle_reap, gaps_counted=3)
        pool.history.record({"id": "ep000001", "state": "x", "verdict": None}, [10.0, 20.0, 30.0, 50.0])
        pool.gaps_changed()
        self.assertEqual(pool.idle_limit(), (500.0, "10 times the 99th percentile (50 s) of the 4 gaps between "
                                                    "calls in this server's history"))

    def test_a_history_of_quick_calls_never_shortens_the_idle_limit_even_after_a_restart(self):
        # A client that reads a resource in a loop fills the history with gaps of 20 ms. Read as the history's
        # 99th percentile, the limit would be 0.2 s, and every later episode would end idle before its first answer.
        reap = None
        for restart in range(2):
            pool = self.pool()
            pool.s.idle_reap = reap = dict(reap or pool.s.idle_reap, gaps_counted=3)
            if restart == 0:
                pool.history.record({"id": "ep000001", "state": "x", "verdict": None}, [0.02] * 500)
                pool.gaps_changed()
            limit, source = pool.idle_limit()
            self.assertEqual(limit, 430.2, restart)
            self.assertTrue(source.endswith("(this server's history of 500 gaps puts it at 0.02 s, and a history "
                                            "never shortens the limit)"), source)

    def test_an_idle_limit_stated_by_the_operator_wins_and_none_means_no_limit(self):
        self.assertEqual(self.pool(idle_s=60).idle_limit(), (60.0, "idle_s, stated in the settings"))
        self.assertEqual(self.pool(idle_reap=None).idle_limit(), (None, "no idle limit is set"))

    def test_the_watch_ends_an_idle_episode_and_never_one_with_a_call_in_flight(self):
        clock = Clock()

        async def go():
            with mock.patch.object(episode_mod.time, "monotonic", clock):
                pool = self.pool(idle_s=60)
                ep = pool.open("bowl-on-plate", "k")
                await ep.ready.wait()
                self.assertIsNone(ep.due(1059.0))
                ep.call_begins()
                self.assertIsNone(ep.due(1100.0))       # a call in flight is never idle
                clock.now = 1100.0
                ep.call_ends()
                self.assertIsNone(ep.due(1159.9))
                self.assertEqual(ep.due(1160.0), "idle")
                self.assertEqual(ep.idle_limit_s, 60.0)
                self.assertEqual(ep.due(1000.0 + pool.s.clock_s), "clock")
                await ep.finish("idle")
        self.run_async(go())

    def test_work_folders_of_a_server_that_is_gone_are_cleared_at_start(self):
        async def go():
            pool = self.pool()
            work = pool.s.work_root
            for name, owner in (("gone", "999999999"), ("mine", str(os.getpid()))):
                os.makedirs(os.path.join(work, name, "ep"))
                with open(os.path.join(work, name, OWNER_FILE), "w", encoding="utf-8") as f:
                    f.write(owner)
            os.makedirs(os.path.join(work, "foreign"))
            cleared = await pool.clear_orphans()
            return pool, work, cleared
        pool, work, cleared = self.run_async(go())
        self.assertEqual(cleared, 1)
        self.assertEqual(sorted(os.listdir(work)), ["foreign", "mine"])
        self.assertEqual(pool.sim.calls, [("stop", os.path.join(work, "gone", "ep"))])

    def test_two_runs_of_one_script_record_the_same_episode(self):
        def once():
            clock = Clock()

            async def go():
                with mock.patch.object(episode_mod.time, "monotonic", clock):
                    pool = self.pool()
                    ep = pool.open("soup-in-basket", "k")
                    await ep.ready.wait()
                    clock.now = 1003.0
                    ep.call_begins()
                    clock.now = 1004.0
                    ep.call_ends()
                    await ep.finish("claim")
                    return ep
            ep = asyncio.run(go())
            return (ep.status, ep.reason, ep.gaps, ep.sim_wall_s, ep.verdict, ep.ended_message())
        self.assertEqual(once(), once())


class PoolWithoutSettingsFile(unittest.TestCase):
    def test_the_pool_takes_any_history_and_store(self):
        folder = Folder()
        self.addCleanup(folder.remove)
        s = Settings(write_settings(folder), platform="linux")
        pool = Pool(s, History(folder.at("h.sqlite")), mock.Mock(), FakeSim())
        self.assertEqual(pool.again, AGAIN_HTTP)
        self.assertEqual(pool.live(), [])


if __name__ == "__main__":
    unittest.main()
