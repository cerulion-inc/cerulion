#!/usr/bin/env python3
"""Checks for wake_census.py: the Linux counter parsers on captured text, the
perf wakeup filter, and one live census of this process on the host it runs on.

    python3 benches/wake_census/test_wake_census.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wake_census  # noqa: E402

STATUS = """Name:\tcerulion
State:\tS (sleeping)
voluntary_ctxt_switches:\t1520
nonvoluntary_ctxt_switches:\t37
"""

SCHED = """cerulion (4242, #threads: 1)
-------------------------------------------------------------------
se.exec_start                                :      91234.567890
nr_switches                                  :                1557
nr_wakeups                                   :                1490
nr_wakeups_sync                              :                   3
"""

SCHED_OLD = "se.statistics.nr_wakeups                     :                 812\n"

PERF = """\
         swapper     0 [003] 100.000001: sched:sched_waking: comm=cerulion pid=4243 prio=120 target_cpu=003
     ksoftirqd/2    24 [002] 100.000002: sched:sched_waking: comm=cerulion pid=4242 prio=120 target_cpu=002
        cerulion  4242 [001] 100.000003: sched:sched_waking: comm=cerulion pid=4243 prio=120 target_cpu=001
         swapper     0 [000] 102.000001: sched:sched_waking: comm=bash pid=999 prio=120 target_cpu=000
"""


class Parsers(unittest.TestCase):
    def test_status(self):
        self.assertEqual(wake_census.parse_status_csw(STATUS), (1520, 37))
        self.assertEqual(wake_census.parse_status_csw("Name: x\n"), (None, None))

    def test_schedstat(self):
        self.assertEqual(wake_census.parse_schedstat_runs("123456 7890 1557\n"), 1557)
        self.assertIsNone(wake_census.parse_schedstat_runs("1 2\n"))

    def test_sched(self):
        self.assertEqual(wake_census.parse_sched_wakeups(SCHED), 1490)
        self.assertEqual(wake_census.parse_sched_wakeups(SCHED_OLD), 812)
        self.assertIsNone(wake_census.parse_sched_wakeups("nr_switches : 3\n"))

    def test_perf_counts_only_idle_and_softirq_wakings_of_the_census_threads(self):
        counts, span = wake_census.parse_perf_wakings(PERF, {4242: 4242, 4243: 4242})
        self.assertEqual(counts, {4242: 2})
        self.assertAlmostEqual(span, 2.0)

    def test_rates_pass_reasons_through_and_wrap_32_bit_counters(self):
        r = wake_census.rates({"a": 10, "b": "unreadable"}, {"a": 30, "b": 5}, 2.0)
        self.assertEqual(r, {"a": 10.0, "b": "unreadable"})
        r = wake_census.rates({"csw": 2**32 - 2}, {"csw": 2}, 1.0, wrap={"csw"})
        self.assertEqual(r, {"csw": 4.0})

    def test_linux_rates_count_new_threads_from_zero_and_report_exited_ones(self):
        c = lambda n: {k: n for k in wake_census.LINUX_KEYS}
        before = {1: c(10), 2: c(5)}
        after = {1: c(30), 3: c(4)}
        r, exited = wake_census.linux_rates(before, after, 2.0)
        self.assertEqual(r["vol_csw"], 12.0)
        self.assertEqual(exited, 1)
        after[3]["wakeups"] = "unavailable"
        r, _ = wake_census.linux_rates(before, after, 2.0)
        self.assertEqual(r["wakeups"], "unavailable")


class Live(unittest.TestCase):
    def test_a_census_of_this_process_reads_its_counters(self):
        result = wake_census.census([os.getpid()], 0.2, use_perf=False)
        row = result["rows"][0]
        self.assertFalse(row.get("gone"), row)
        key = "vol_csw" if sys.platform.startswith("linux") else "csw"
        self.assertIsInstance(row.get(key), float, row)
        self.assertIn("total", wake_census.render(result))

    def test_a_gone_process_is_reported_gone(self):
        result = wake_census.census([2**22 + 12345], 0.05, use_perf=False)
        self.assertTrue(result["rows"][0].get("gone"), result)


if __name__ == "__main__":
    unittest.main()
