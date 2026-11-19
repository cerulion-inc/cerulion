# SPDX-License-Identifier: AGPL-3.0-only
"""The list-price arithmetic run-cell prints a cost with. Standard library only.

Every expected cost below is worked by hand from the table row it names (USD per million tokens), never read
back from the code under test.
"""
import json
import os
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from cerulion_harness import prices  # noqa: E402


class ModelNames(unittest.TestCase):
    def test_a_served_model_name_maps_to_the_row_it_contains(self):
        self.assertEqual(prices.key_of("google/gemini-3.8-flash-preview"), "gemini-3.8-flash")
        self.assertEqual(prices.key_of("claude-opus-5-5"), "claude-opus-5.5")
        self.assertEqual(prices.key_of("gpt-6.2-sol-2026-10-01"), "gpt-6.2-sol")
        self.assertEqual(prices.key_of("gpt-6-luna"), "gpt-6-luna")

    def test_a_name_the_table_lacks_has_no_key_and_no_cost(self):
        self.assertIsNone(prices.key_of("some-local-model"))
        self.assertIsNone(prices.cost_of([{"prompt_tokens": 10, "completion_tokens": 10}], None))
        self.assertIsNone(prices.cost_of([{"prompt_tokens": 10, "completion_tokens": 10}], "no-such-row"))

    def test_every_name_maps_to_a_row_of_the_table(self):
        for word, key in prices.NAMES:
            self.assertIn(key, prices.PRICES, word)
            self.assertIn(prices.PRICES[key][8], prices.SOURCES, key)


class Costs(unittest.TestCase):
    def test_cached_input_is_charged_at_the_cached_price(self):
        # gemini-3.8-flash: input 0.75, cached 0.075, output 3.75.
        # 500,000 uncached x 0.75 + 500,000 cached x 0.075 + 100,000 out x 3.75 = 0.375 + 0.0375 + 0.375
        usage = {"prompt_tokens": 1_000_000, "completion_tokens": 100_000,
                 "prompt_tokens_details": {"cached_tokens": 500_000}}
        self.assertEqual(prices.cost_of([usage], "gemini-3.8-flash"), 0.7875)

    def test_a_prompt_over_the_threshold_is_charged_at_the_long_prices(self):
        # gpt-6.2-sol over 272,000 prompt tokens: input 4.00, cached 0.20, output 15.00.
        # 200,000 x 4.00 + 100,000 x 0.20 + 10,000 x 15.00 = 0.8 + 0.02 + 0.15
        usage = {"prompt_tokens": 300_000, "completion_tokens": 10_000,
                 "prompt_tokens_details": {"cached_tokens": 100_000}}
        self.assertEqual(prices.cost_of([usage], "gpt-6.2-sol"), 0.97)

    def test_a_prompt_at_the_threshold_is_charged_at_the_short_prices(self):
        # 272,000 x 2.00 = 0.544
        self.assertEqual(prices.cost_of([{"prompt_tokens": 272_000, "completion_tokens": 0}], "gpt-6.2-sol"), 0.544)

    def test_a_cache_write_is_charged_at_the_cache_write_price(self):
        # claude-opus-5.5: input 4.00, cached 0.20, output 20.00, cache write 5.00.
        # 2,000 uncached x 4 + 6,000 cached x 0.2 + 2,000 written x 5 + 1,000 out x 20 = 0.008 + 0.0012 + 0.01 + 0.02
        usage = {"prompt_tokens": 10_000, "completion_tokens": 1_000, "cache_creation_input_tokens": 2_000,
                 "prompt_tokens_details": {"cached_tokens": 6_000}}
        self.assertEqual(prices.cost_of([usage], "claude-opus-5.5"), 0.0392)

    def test_the_costs_of_several_replies_add_up(self):
        # gpt-6-luna under its 272,000 threshold: input 0.10, output 0.50.
        # Two replies of 100,000 in and 100,000 out: 2 x (0.01 + 0.05)
        usage = {"prompt_tokens": 100_000, "completion_tokens": 100_000}
        self.assertEqual(prices.cost_of([usage, usage], "gpt-6-luna"), 0.12)

    def test_an_empty_list_costs_nothing(self):
        self.assertEqual(prices.cost_of([], "gpt-6-luna"), 0.0)


class Tokens(unittest.TestCase):
    def test_reasoning_counted_apart_from_completion_is_added_to_it(self):
        usage = {"prompt_tokens": 100, "completion_tokens": 50, "total_tokens": 250,
                 "completion_tokens_details": {"reasoning_tokens": 100}}
        self.assertEqual(prices.tokens(usage), (100, 0, 0, 150, 100))

    def test_reasoning_inside_completion_is_not_counted_twice(self):
        usage = {"prompt_tokens": 100, "completion_tokens": 150, "total_tokens": 250,
                 "completion_tokens_details": {"reasoning_tokens": 100}}
        self.assertEqual(prices.tokens(usage), (100, 0, 0, 150, 100))

    def test_input_and_output_names_are_read_too(self):
        self.assertEqual(prices.tokens({"input_tokens": 7, "output_tokens": 3}), (7, 0, 0, 3, 0))

    def test_a_cached_count_above_the_prompt_is_clamped_to_the_prompt(self):
        usage = {"prompt_tokens": 10, "completion_tokens": 1, "prompt_tokens_details": {"cached_tokens": 99}}
        self.assertEqual(prices.tokens(usage), (10, 10, 0, 1, 0))

    def test_a_missing_or_negative_count_reads_as_zero(self):
        self.assertEqual(prices.tokens({}), (0, 0, 0, 0, 0))
        usage = {"prompt_tokens": 10, "completion_tokens": 1, "prompt_tokens_details": {"cached_tokens": -5}}
        self.assertEqual(prices.tokens(usage), (10, 0, 0, 1, 0))


class OperatorLog(unittest.TestCase):
    def test_only_reply_records_with_a_usage_are_read(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, "operator.jsonl")
            with open(log, "w", encoding="utf-8") as f:
                f.write(json.dumps({"kind": "reply", "usage": {"prompt_tokens": 1}}) + "\n")
                f.write("not json\n")
                f.write(json.dumps({"kind": "request", "usage": {"prompt_tokens": 2}}) + "\n")
                f.write(json.dumps({"kind": "reply", "usage": "not a record"}) + "\n")
                f.write(json.dumps({"kind": "reply", "usage": {"prompt_tokens": 3}}) + "\n")
            self.assertEqual(prices.usages_of(log), [{"prompt_tokens": 1}, {"prompt_tokens": 3}])

    def test_a_missing_log_reads_as_no_replies(self):
        self.assertEqual(prices.usages_of("/nonexistent/operator.jsonl"), [])


if __name__ == "__main__":
    unittest.main()
