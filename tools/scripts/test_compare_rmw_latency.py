#!/usr/bin/env python3
"""Hand-written format and arithmetic oracles for RMW report parsing."""

import unittest

from compare_rmw_latency import measurements, summarize

# Parser inputs only; these values never become a benchmark report.
FORMAT = (
    "discriminator[spin=0 unpinned] RTT p50=500ns p90=1µs p99=2ms\n"
    "discriminator[spin=50 unpinned] RTT p50=0.001ms p90=3µs p99=0.01s"
)


class ReportOracles(unittest.TestCase):
    def test_duration_units_and_both_phases(self):
        expected = {"spin=0 unpinned": {"p50_us": 0.5, "p99_us": 2000.0},
                    "spin=50 unpinned": {"p50_us": 1.0, "p99_us": 10000.0}}
        self.assertEqual(measurements(FORMAT), expected)
        self.assertEqual(measurements(FORMAT), expected)

    def test_missing_duplicate_zero_and_inverted_values_refuse(self):
        for invalid in ("", FORMAT + FORMAT,
                        FORMAT.replace("p50=500ns", "p50=0ns"),
                        FORMAT.replace("p99=2ms", "p99=100ns")):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                measurements(invalid)

    def test_paired_deltas_keep_spread_instead_of_ratio_of_medians(self):
        before = {"spin=0 unpinned": {"p50_us": 1.0, "p99_us": 2.0},
                  "spin=50 unpinned": {"p50_us": 2.0, "p99_us": 4.0}}
        after = {"spin=0 unpinned": {"p50_us": 2.0, "p99_us": 4.0},
                 "spin=50 unpinned": {"p50_us": 4.0, "p99_us": 8.0}}
        rows = summarize([{"baseline": before, "candidate": after},
                          {"baseline": after, "candidate": before}])
        expected = {"before_median": 1.5, "after_median": 1.5,
                    "paired_delta_percent": 25.0,
                    "paired_delta_min_percent": -50.0,
                    "paired_delta_max_percent": 100.0}
        self.assertEqual(rows[0]["p50_us"], expected)
        self.assertEqual(rows[0]["size_bytes"], 24)


if __name__ == "__main__":
    unittest.main()
