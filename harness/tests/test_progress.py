# SPDX-License-Identifier: AGPL-3.0-only
"""The line run-cell prints per model reply, and the start line's expectation. Standard library only: the
episode files are hand-written, and every expected line and cost is written out by hand from the price table
row it names (USD per million tokens), never read back from the code under test."""
import json
import os
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from cerulion_harness import expected, progress  # noqa: E402

CELL = "pro_spatial_swap_t00_i00"
T = "2026-11-18T16:00:00Z"
BUDGET = {"model_replies": 80, "control_steps": 900}


def clock():
    return T


class Episode:
    """The scratch folder of one setup: attempt folders with an operator log and a truth log."""

    def __init__(self, root):
        self.group = os.path.join(root, "run_without")

    def append(self, attempt, name, *records, raw=None):
        sub = (f"{CELL}_a{attempt}",) if name == "operator.jsonl" else ("_scoring", f"{CELL}_a{attempt}")
        folder = os.path.join(self.group, *sub)
        os.makedirs(os.path.join(self.group, f"{CELL}_a{attempt}"), exist_ok=True)
        os.makedirs(folder, exist_ok=True)
        with open(os.path.join(folder, name), "a", encoding="utf-8") as f:
            for rec in records:
                f.write(json.dumps(rec) + "\n")
            if raw is not None:
                f.write(raw)

    def steps(self, attempt, first, last):
        self.append(attempt, "truth_log.jsonl", *({"step": s, "success": False} for s in range(first, last + 1)))


def reply(turn, usage):
    return {"kind": "reply", "turn": turn, "usage": usage, "content": ""}


# gemini-3.8-flash: input 0.75, cached 0.075, output 3.75 USD per million tokens.
# Reply 1: 1,000 in x 0.75 + 200 out x 3.75 = 0.00075 + 0.00075 = 0.0015.
# Reply 2: 1,000 uncached x 0.75 + 1,000 cached x 0.075 + 100 out x 3.75 = 0.0012; so far 0.0027.
U1 = {"prompt_tokens": 1000, "completion_tokens": 200, "completion_tokens_details": {"reasoning_tokens": 150},
      "cost": 0.0016}
U2 = {"prompt_tokens": 2000, "completion_tokens": 100, "completion_tokens_details": {"reasoning_tokens": 50},
      "prompt_tokens_details": {"cached_tokens": 1000}, "cost": 0.0012}
LINE_1 = (f"{T} plain reply 1: control step 30 of 900; tokens in 1000 (0 cached), out 200 (150 reasoning); so far "
          f"0.0015 USD at list price, 0.0016 USD reported by the endpoint")
LINE_2 = (f"{T} plain reply 2: control step 42 of 900; tokens in 2000 (1000 cached), out 100 (50 reasoning); so far "
          f"0.0027 USD at list price, 0.0028 USD reported by the endpoint")


class Lines(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.ep = Episode(self.tmp.name)
        self.out = []

    def tearDown(self):
        self.tmp.cleanup()

    def monitor(self, model="google/gemini-3.8-flash"):
        return progress.Progress("plain", self.ep.group, CELL, model, 900, self.out.append, clock=clock)

    def test_each_reply_is_one_line_with_the_control_step_after_its_call(self):
        m = self.monitor()
        self.ep.append(1, "truth_log.jsonl", {"event": "reset"})
        self.ep.append(1, "operator.jsonl", {"kind": "request", "turn": 0}, reply(1, U1))
        self.ep.steps(1, 1, 30)
        m.poll()
        self.ep.append(1, "operator.jsonl", {"kind": "tool_call"}, {"kind": "request"}, reply(2, U2))
        self.ep.steps(1, 31, 42)
        m.poll()
        self.assertEqual(self.out, [LINE_1, LINE_2])

    def test_a_poll_with_nothing_new_prints_nothing_and_no_reply_is_printed_twice(self):
        m = self.monitor()
        m.poll()
        self.ep.append(1, "operator.jsonl", reply(1, U1))
        self.ep.steps(1, 1, 30)
        m.poll()
        m.poll()
        self.assertEqual(self.out, [LINE_1])

    def test_a_line_still_being_written_waits_for_its_end(self):
        m = self.monitor()
        self.ep.steps(1, 1, 30)
        text = json.dumps(reply(1, U1))
        self.ep.append(1, "operator.jsonl", raw=text[:20])
        m.poll()
        self.assertEqual(self.out, [])
        self.ep.append(1, "operator.jsonl", raw=text[20:] + "\n")
        m.poll()
        self.assertEqual(self.out, [LINE_1])

    def test_before_any_step_the_control_step_is_0_and_without_a_cost_field_none_is_printed(self):
        m = self.monitor()
        self.ep.append(1, "operator.jsonl", reply(1, {"prompt_tokens": 1000, "completion_tokens": 200}))
        m.poll()
        self.assertEqual(self.out, [f"{T} plain reply 1: control step 0 of 900; tokens in 1000 (0 cached), out 200 "
                                    f"(0 reasoning); so far 0.0015 USD at list price"])

    def test_a_model_the_price_table_lacks_and_a_reply_without_counts(self):
        m = self.monitor(model="a-local-model")
        self.ep.append(1, "operator.jsonl", reply(1, None), reply(2, {"prompt_tokens": "many"}))
        m.poll()
        self.assertEqual(self.out, [
            f"{T} plain reply 1: control step 0 of 900; no token counts reported; so far no list price for this model",
            f"{T} plain reply 2: control step 0 of 900; no token counts reported; so far no list price for this model"])

    def test_a_second_attempt_counts_from_its_own_start(self):
        m = self.monitor()
        self.ep.append(1, "operator.jsonl", reply(1, U1))
        self.ep.steps(1, 1, 30)
        m.poll()
        self.ep.append(2, "operator.jsonl", reply(1, U1))
        self.ep.steps(2, 1, 30)
        m.poll()
        self.assertEqual(self.out, [LINE_1, LINE_1.replace("reply 1:", "reply 1 (attempt 2):")])

    def test_lines_that_are_not_records_and_steps_that_are_not_counts_are_skipped(self):
        m = self.monitor()
        self.ep.append(1, "truth_log.jsonl", {"step": 30}, {"step": "31"}, {"step": True}, {"step": 7},
                       raw="garbage\n[1]\n")
        self.ep.append(1, "operator.jsonl", raw="{not json\n[]\n\"x\"\n")
        self.ep.append(1, "operator.jsonl", {"kind": "reply", "turn": "one", "usage": U1})
        m.poll()
        self.assertEqual(self.out, [LINE_1])   # the reply number falls back to the count of replies

    def test_a_folder_that_is_not_an_attempt_of_this_cell_is_not_read(self):
        m = self.monitor()
        for name in (f"{CELL}_ax", f"{CELL}_a1.log", f"other_{CELL}_a1", "_scoring"):
            path = os.path.join(self.ep.group, name)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            if name.endswith(".log"):
                open(path, "w").close()
            else:
                os.makedirs(path, exist_ok=True)
                with open(os.path.join(path, "operator.jsonl"), "w") as f:
                    f.write(json.dumps(reply(1, U1)) + "\n")
        m.poll()
        self.assertEqual(self.out, [])

    def test_no_scratch_folder_yet_prints_nothing(self):
        progress.Progress("plain", os.path.join(self.tmp.name, "absent"), CELL, "m", 900, self.out.append).poll()
        self.assertEqual(self.out, [])

    def test_the_same_files_give_the_same_lines(self):
        self.ep.append(1, "operator.jsonl", reply(1, U1), reply(2, U2))
        self.ep.steps(1, 1, 42)
        runs = []
        for _ in range(2):
            out = []
            progress.Progress("plain", self.ep.group, CELL, "google/gemini-3.8-flash", 900, out.append,
                              clock=clock).poll()
            runs.append(out)
        self.assertEqual(runs[0], runs[1])
        self.assertEqual(len(runs[0]), 2)

    def test_the_clock_is_utc_with_a_zone(self):
        self.assertRegex(progress.utc_now(), r"^\d{4}-\d\d-\d\dT\d\d:\d\d:\d\dZ$")

    def test_a_reported_cost_is_a_number(self):
        self.assertEqual([progress.reported_cost(u) for u in ({"cost": 0.5}, {"cost": 2}, {"cost": True},
                                                              {"cost": "0.5"}, {}, None)],
                         [0.5, 2.0, None, None, None, None])


class Expected(unittest.TestCase):
    """The start line's expectation, from the results package's summary.csv."""

    def test_a_measured_model_states_each_setup_as_the_package_prints_it(self):
        self.assertEqual(expected.expected_line("google/gemini-3.8-flash", ["plain", "rcp"], BUDGET, 5400.0),
                         "expected from the measured runs of gemini-3.8-flash: plain 846.644 operator seconds, "
                         "55.87 replies, 1.6665 USD at list price (mean of 100 scored episodes); rcp 258.659 operator "
                         "seconds, 25.55 replies, 0.3479 USD at list price (mean of 100 scored episodes); an episode "
                         "stops at 80 replies, 900 control steps or 5400 s")

    def test_one_setup_names_only_that_setup(self):
        self.assertEqual(expected.expected_line("claude-opus-5-5", ["rcp"], BUDGET, 1800.0),
                         "expected from the measured runs of claude-opus-5.5: rcp 278.0592 operator seconds, "
                         "15.5102 replies, 2.762 USD at list price (mean of 98 scored episodes); an episode stops at "
                         "80 replies, 900 control steps or 1800 s")

    def test_a_model_without_a_measured_run(self):
        for model in ("a-local-model", "claude-haiku-5-5"):
            self.assertEqual(expected.expected_line(model, ["plain"], BUDGET, 5400.0),
                             "expected: no measured run of this model; an episode stops at 80 replies, 900 control "
                             "steps or 5400 s", model)

    def test_rows_as_the_package_prints_them(self):
        # Copied by hand from summary.csv: scored, mean_requests, the seconds column, mean_usd_list.
        self.assertEqual(expected.MEASURED["gpt-6.2-sol"],
                         {"plain": ("100", "51.16", "983.089", "1.89"), "rcp": ("100", "28.76", "340.556", "1.1284")})
        self.assertEqual(expected.MEASURED["grok-4.7"], {"plain": ("99", "74.5758", "2257.498", "2.5703"),
                                                         "rcp": ("100", "29.85", "816.231", "1.347")})
        self.assertEqual(len(expected.MEASURED), 14)

    def test_every_row_is_a_price_key_with_both_setups(self):
        from cerulion_harness import prices
        for key, rows in expected.MEASURED.items():
            self.assertIn(key, prices.PRICES, key)
            self.assertEqual(sorted(rows), ["plain", "rcp"], key)
            for arm, row in rows.items():
                self.assertEqual(len(row), 4, (key, arm))
                for value in row:
                    float(value)


if __name__ == "__main__":
    unittest.main()
