# SPDX-License-Identifier: AGPL-3.0-only
"""What the sim server keeps: the history file of ended episodes (its gap percentile against hand-computed
values) and the kept record a viewer page is served from (its keep rules and the names it refuses).
Standard library only; no simulator."""
import json
import os
import sqlite3
import time
import unittest

from playground_support import Folder, write_settings

from playground.history import History
from playground.settings import Settings
from playground.store import Store


def row(eid, **changes):
    base = {"id": eid, "state": "bowl-on-plate", "started": "2026-11-18T10:00:00Z", "ended": "2026-11-18T10:05:00Z",
            "reason": "claim", "sim_wall_s": 300.0, "sim_cpu_s": 120.5, "peak_rss_mb": 900.0, "calls": 12,
            "verdict": {"solved": True}}
    return dict(base, **changes)


class HistoryFile(unittest.TestCase):
    def setUp(self):
        self.folder = Folder()
        self.addCleanup(self.folder.remove)
        self.history = History(self.folder.at("data", "episodes.sqlite"))

    def test_an_ended_episode_is_written_once_and_read_back(self):
        self.assertTrue(self.history.record(row("ep000001"), [1.0, 2.0]))
        self.assertTrue(self.history.record(row("ep000001", reason="idle"), [9.0]))
        self.assertEqual(self.history.episodes(), [row("ep000001")])
        self.assertEqual(self.history.gap_count(), 2)

    def test_the_gap_percentile_is_the_nearest_rank(self):
        # ten gaps 1..10 s: the 99th percentile is rank ceil(9.9) = 10 -> 10 s; the 50th is rank 5 -> 5 s;
        # the 25th is rank ceil(2.5) = 3 -> 3 s
        self.history.record(row("ep000001"), [7.0, 1.0, 9.0, 3.0, 5.0])
        self.history.record(row("ep000002"), [2.0, 10.0, 4.0, 8.0, 6.0])
        self.assertEqual(self.history.gap_percentile(99), 10.0)
        self.assertEqual(self.history.gap_percentile(50), 5.0)
        self.assertEqual(self.history.gap_percentile(25), 3.0)

    def test_no_gaps_has_no_percentile(self):
        self.assertIsNone(self.history.gap_percentile(99))

    def test_a_write_that_fails_is_counted_and_never_raised(self):
        self.history.db.close()
        self.history.db = sqlite3.connect(":memory:", check_same_thread=False)   # no tables: every write fails
        with self.assertLogs("cerulion.sim", "WARNING") as logs:
            self.assertFalse(self.history.record(row("ep000003")))
        self.assertIn("episode ep000003 not written", logs.output[0])
        self.assertEqual(self.history.state()["not_written"], 1)

    def test_a_read_that_fails_answers_the_last_figure(self):
        self.history.record(row("ep000001"), [4.0, 8.0])
        self.assertEqual(self.history.gap_percentile(50), 4.0)
        self.history.db.close()
        self.history.db = sqlite3.connect(":memory:", check_same_thread=False)
        self.assertEqual(self.history.gap_count(), 2)
        self.assertEqual(self.history.gap_percentile(50), 4.0)


class FakeEpisode:
    def __init__(self, eid):
        self.eid = eid

    def meta(self):
        return {"id": self.eid, "status": "ready"}


class KeptRecord(unittest.TestCase):
    def setUp(self):
        self.folder = Folder()
        self.addCleanup(self.folder.remove)
        self.s = Settings(write_settings(self.folder, keep_full_pictures_s=100, keep_viewer_s=1000,
                                         keep_last_pictures=2), platform="linux")
        self.store = Store(self.s)

    def ended(self, eid, ended_epoch, frames):
        ep = FakeEpisode(eid)
        self.store.begin(ep)
        for name in frames:
            with open(self.store.dir(eid) + "/frames/" + name, "wb") as f:
                f.write(b"x")
        self.store.append(eid, {"n": 0, "kind": "result", "frames": frames})
        with open(os.path.join(self.store.dir(eid), "meta.json"), "w", encoding="utf-8") as f:
            json.dump({"id": eid, "status": "ended", "ended_epoch": ended_epoch}, f)

    def test_a_kept_episode_loads_from_the_folder(self):
        self.ended("ep000001", 1000.0, ["a.png"])
        doc = self.store.load("ep000001")
        self.assertEqual((doc["status"], doc["events_total"], doc["events"][0]["frames"]), ("ended", 1, ["a.png"]))
        self.assertEqual(self.store.load("ep000001", since=1)["events"], [])

    def test_names_that_are_not_episode_ids_or_pictures_are_refused(self):
        self.ended("ep000001", 1000.0, ["a.png"])
        for eid in ("..", "../ep000001", "", "a/b", "x" * 41, None):
            self.assertIsNone(self.store.dir(eid))
            self.assertIsNone(self.store.load(eid))
        for name in ("../meta.json", "..", ".", "", "frames/a.png"):
            self.assertIsNone(self.store.frame_path("ep000001", name))
        self.assertTrue(self.store.frame_path("ep000001", "a.png").endswith("/frames/a.png"))

    def test_the_sweep_trims_pictures_then_removes_the_episode(self):
        self.ended("ep000001", 1000.0, ["a.png", "b.png", "c.png"])
        self.assertEqual(self.store.sweep(now=1050.0)["trimmed"], 0)          # younger than keep_full
        done = self.store.sweep(now=1200.0)                                     # older: the last 2 stay
        self.assertEqual((done["trimmed"], done["pictures_removed"]), (1, 1))
        self.assertEqual(sorted(os.listdir(self.store.dir("ep000001") + "/frames")), ["b.png", "c.png"])
        self.assertEqual(self.store.sweep(now=1300.0)["trimmed"], 0)          # trimmed once
        self.assertEqual(self.store.sweep(now=2100.0)["removed"], 1)          # older than keep_viewer
        self.assertFalse(os.path.exists(self.store.dir("ep000001")))

    def test_the_sweep_never_touches_a_live_episode(self):
        self.ended("ep000001", 1000.0, ["a.png", "b.png", "c.png"])
        self.assertEqual(self.store.sweep(now=5000.0, live=("ep000001",))["removed"], 0)
        self.assertTrue(os.path.isdir(self.store.dir("ep000001")))

    def test_an_episode_a_lost_server_left_is_shown_as_ended_once_old(self):
        ep = FakeEpisode("ep000002")
        self.store.begin(ep)
        self.store.append("ep000002", {"n": 0, "kind": "status"})
        self.assertEqual(self.store.load("ep000002")["status"], "away")
        old = time.time() - self.s.clock_s - 700
        os.utime(os.path.join(self.store.dir("ep000002"), "events.jsonl"), (old, old))
        doc = self.store.load("ep000002")
        self.assertEqual((doc["status"], doc["reason"]), ("ended", "server_lost"))

    def test_the_last_pictures_are_the_newest_distinct_names(self):
        events = [{"frames": ["a", "b"]}, {"frames": ["c", "b"]}, {}]
        self.assertEqual(self.store.last_pictures(events), (["b", "c"], 3))


if __name__ == "__main__":
    unittest.main()
