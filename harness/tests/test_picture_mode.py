# SPDX-License-Identifier: AGPL-3.0-only
"""The picture mode: the one run-cell gives each model and setup, the --image-mode option, the operator flags each
mode adds, and what the measured operator does with them (bench/operators/chat_support.py, History, and the
operator's own parser, run through fixtures/measured_operator.py in a process of their own). Standard library only.
Every expected flag string, message list and estimate is written out by hand below."""
import contextlib
import io
import json
import pathlib
import shlex
import subprocess
import sys
import unittest

TESTS = pathlib.Path(__file__).resolve().parent
HARNESS = TESTS.parent
sys.path.insert(0, str(HARNESS / "src"))

from cerulion_harness import cli, route  # noqa: E402

MEASURED_RCP = HARNESS / "measured" / "rcp"
OPERATOR = TESTS / "fixtures" / "measured_operator.py"
APPEND = ["--image-mode", "append"]
MARKED = ["--image-mode", "append", "--cache-prefix-mark"]

# (model, setup, mode, operator flags) without --image-mode.
DEFAULTS = [
    ("system.ai.claude-fable-5-2", "rcp", "append", MARKED),
    ("system.ai.claude-fable-5-2", "plain", "swap", []),
    ("claude-opus-5-5", "rcp", "append", MARKED),
    ("claude-opus-5-5", "plain", "swap", []),
    ("anthropic/claude-sonnet-4.5", "rcp", "append", MARKED),
    ("us.anthropic.Claude-Haiku-5-5", "rcp", "append", MARKED),
    ("CLAUDE-SONNET-5-5", "plain", "swap", []),
    ("gemini-3.8-flash", "rcp", "swap", []),
    ("google/gemini-3.8-flash", "plain", "swap", []),
    ("gpt-6.1-sol", "rcp", "swap", []),
    ("grok-4.7", "plain", "swap", []),
]
# (model, setup, --image-mode, mode, operator flags).
GIVEN = [
    ("claude-opus-5-5", "rcp", "swap", "swap", []),
    ("claude-opus-5-5", "rcp", "append", "append", MARKED),
    ("claude-opus-5-5", "plain", "append", "append", APPEND),
    ("claude-opus-5-5", "plain", "swap", "swap", []),
    ("gemini-3.8-flash", "rcp", "append", "append", APPEND),
    ("gemini-3.8-flash", "plain", "append", "append", APPEND),
    ("gemini-3.8-flash", "rcp", "swap", "swap", []),
]
# The object the Claude rows' route merged into every request body, given as --extra-body, and the operator flags
# of a Claude model with the harness tools in append mode as its episode's command line carried them.
CLAUDE_BODY = '{"output_config": {"effort": "high"}, "cache_control": {"type": "ephemeral"}}'
CLAUDE_RCP_FLAGS = ("--context-tokens 200000 --max-tokens 32000 --http-timeout 600 --temperature 1.0 --retry-wait 20 "
                    "--seed 1 --extra-body '{\"output_config\": {\"effort\": \"high\"}, \"cache_control\": {\"type\": "
                    "\"ephemeral\"}}' --image-mode append --cache-prefix-mark")
CLAUDE_PLAIN_FLAGS = ("--context-tokens 200000 --max-tokens 32000 --http-timeout 600 --temperature 1.0 "
                      "--retry-wait 20 --seed 1 --extra-body '{\"output_config\": {\"effort\": \"high\"}, "
                      "\"cache_control\": {\"type\": \"ephemeral\"}}'")
# What the episode driver puts after the operator flags (runs/tools/grid_ext.py).
DRIVER_WORDS = ["--ep", "/nonexistent-episode", "--base-url", "http://127.0.0.1:9/v1", "--model", "m",
                "--reply-budget", "80", "--one-call-per-reply", "--keep-images", "4"]

SYSTEM = "You operate a robot."
FIRST = "Task: pick the bowl."
DROPPED = "[frame dropped to save context]"
PREFIX = [{"role": "system", "content": SYSTEM}, {"role": "user", "content": FIRST}]
MARK = {"type": "ephemeral"}


def turn(n, picture=True):
    """One model turn as the operator stores it: the reply's tool call, its result, and the frame it brought; with
    picture False, the frame's picture is the text that replaces it."""
    image = ({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,RlJBTUU{n}"}} if picture
             else {"type": "text", "text": DROPPED})
    call = {"id": f"call_{n}", "type": "function", "function": {"name": "look", "arguments": "{}"}}
    return [{"role": "assistant", "content": "", "tool_calls": [call]},
            {"role": "tool", "tool_call_id": f"call_{n}", "content": "ok"},
            {"role": "user", "content": [{"type": "text", "text": f"frame {n}"}, image]}]


def gone(n):
    return turn(n, picture=False)


def operator(what, data):
    """What fixtures/measured_operator.py prints for data, as JSON."""
    done = subprocess.run([sys.executable, "-I", "-B", str(OPERATOR), str(MEASURED_RCP), what],
                          input=json.dumps(data), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                          timeout=60)
    if done.returncode != 0:
        raise AssertionError(f"measured_operator.py {what} exited {done.returncode}: {done.stderr}")
    return json.loads(done.stdout)


def history(args, steps):
    return operator("history", {"history": args, "steps": steps})


def turns_and_builds(count, limit):
    """Build, then for each of count turns add it and build again."""
    steps = [{"build": limit}]
    for n in range(1, count + 1):
        steps += [{"add_turn": turn(n)}, {"build": limit}]
    return steps


class Defaults(unittest.TestCase):
    """Without --image-mode: append for a Claude model with the harness tools, swap for every other one."""

    def test_each_model_and_setup(self):
        for model, arm, mode, flags in DEFAULTS:
            self.assertEqual(cli.picture_mode(model, arm), (mode, flags), (model, arm))

    def test_the_operator_flags_of_a_claude_model(self):
        extra = route.extra_body_for("https://example.test/v1", "system.ai.claude-fable-5-2", CLAUDE_BODY)
        for arm, want in (("rcp", CLAUDE_RCP_FLAGS), ("plain", CLAUDE_PLAIN_FLAGS)):
            got = cli.op_args(extra, cli.picture_mode("system.ai.claude-fable-5-2", arm)[1])
            self.assertEqual(got, want, arm)

    def test_another_model_gets_the_flags_it_got_before(self):
        extra = route.extra_body_for("https://example.test/v1", "gemini-3.8-flash")
        want = ("--context-tokens 200000 --max-tokens 32000 --http-timeout 600 --temperature 1.0 --retry-wait 20 "
                "--seed 1 --extra-body '{\"reasoning_effort\": \"high\"}'")
        for arm in ("plain", "rcp"):
            self.assertEqual(cli.op_args(extra, cli.picture_mode("gemini-3.8-flash", arm)[1]), want, arm)
        self.assertEqual(cli.op_args(extra), want)

    def test_the_start_line_names_each_setups_mode(self):
        self.assertEqual(cli.picture_line("claude-opus-5-5", ["plain", "rcp"]),
                         "pictures: plain swap, rcp append with the opening marked for the prompt cache (the "
                         "default for this model; --image-mode names another)")
        self.assertEqual(cli.picture_line("gemini-3.8-flash", ["rcp"]),
                         "pictures: rcp swap (the default for this model; --image-mode names another)")


class Option(unittest.TestCase):
    """--image-mode names one mode for every setup of the run."""

    def test_each_given_mode(self):
        for model, arm, given, mode, flags in GIVEN:
            self.assertEqual(cli.picture_mode(model, arm, given), (mode, flags), (model, arm, given))

    def test_the_start_line_says_the_option_named_it(self):
        self.assertEqual(cli.picture_line("claude-opus-5-5", ["plain", "rcp"], "swap"),
                         "pictures: plain swap, rcp swap (named by --image-mode)")
        self.assertEqual(cli.picture_line("gemini-3.8-flash", ["plain", "rcp"], "append"),
                         "pictures: plain append, rcp append (named by --image-mode)")

    def test_the_parser(self):
        parse = cli.parser().parse_args
        self.assertIsNone(parse(["run-cell", "--model", "m"]).image_mode)
        for mode in ("swap", "append"):
            self.assertEqual(parse(["run-cell", "--model", "m", "--image-mode", mode]).image_mode, mode)

    def test_another_word_is_a_usage_error(self):
        err = io.StringIO()
        with contextlib.redirect_stderr(err), self.assertRaises(SystemExit) as caught:
            cli.parser().parse_args(["run-cell", "--model", "m", "--image-mode", "keep"])
        self.assertEqual(caught.exception.code, 2)
        self.assertIn("argument --image-mode: invalid choice: 'keep'", err.getvalue())

    def test_an_unknown_mode_in_code_is_refused(self):
        with self.assertRaises(ValueError) as caught:
            cli.picture_mode("claude-opus-5-5", "rcp", "keep")
        self.assertEqual(str(caught.exception), "unknown picture mode 'keep': use one of swap, append")


class MeasuredOperatorFlags(unittest.TestCase):
    """The measured operator's own parser reads the flags run-cell hands the episode driver."""

    def parsed(self, model, arm, given=None):
        words = shlex.split(cli.op_args(CLAUDE_BODY, cli.picture_mode(model, arm, given)[1])) + DRIVER_WORDS
        return operator("flags", words)

    def test_a_claude_model_with_the_harness_tools(self):
        self.assertEqual(self.parsed("claude-opus-5-5", "rcp"),
                         {"image_mode": "append", "cache_prefix_mark": True, "image_high_water": 0, "keep_images": 4})

    def test_swap_is_the_operators_own_default(self):
        for model, arm in (("claude-opus-5-5", "plain"), ("gemini-3.8-flash", "rcp")):
            self.assertEqual(self.parsed(model, arm), {"image_mode": "swap", "cache_prefix_mark": False,
                                                       "image_high_water": 0, "keep_images": 4}, (model, arm))

    def test_append_without_the_mark(self):
        self.assertEqual(self.parsed("gemini-3.8-flash", "plain", "append"),
                         {"image_mode": "append", "cache_prefix_mark": False, "image_high_water": 0,
                          "keep_images": 4})


class AppendHistory(unittest.TestCase):
    """What the measured History sends in each mode. Estimates by hand (chat_support.py: 8 tokens a message, a text
    at 3 characters a token rounded up, 800 a picture, a tool call list by its JSON): the prefix 30 (15 + 15); a
    turn 857 (the reply 8 + 29 for its call, the result 8 + 1, the frame message 8 + 3 + 800); a frame message whose
    picture went 22 (8 + 3 + 11)."""

    def test_every_request_starts_with_the_whole_previous_request(self):
        out = history([SYSTEM, FIRST, 1, "append", 0, False], turns_and_builds(6, 100000))
        builds = out["builds"]
        self.assertEqual(len(builds), 7)
        want = list(PREFIX)
        for n, (messages, trimmed, _) in enumerate(builds):
            if n:
                want += turn(n)
            self.assertEqual(messages, want, n)
            self.assertEqual(trimmed, {}, n)
        for n in range(1, 7):
            previous = builds[n - 1][0]
            self.assertEqual(builds[n][0][:len(previous)], previous, n)
        self.assertEqual(out["counts"], {"frames_dropped": 0, "results_shortened": 0, "replies_shortened": 0,
                                         "turns_dropped": 0, "image_epochs": 0})

    def test_swap_edits_the_older_turn_on_every_request(self):
        out = history([SYSTEM, FIRST, 1, "swap", 0, False], turns_and_builds(3, 100000))
        builds = out["builds"]
        self.assertEqual(builds[1][:2], [PREFIX + turn(1), {}])
        self.assertEqual(builds[2][:2], [PREFIX + gone(1) + turn(2), {"frames_dropped": 1}])
        self.assertEqual(builds[3][:2], [PREFIX + gone(1) + gone(2) + turn(3), {"frames_dropped": 1}])
        self.assertNotEqual(builds[2][0][:len(builds[1][0])], builds[1][0])

    def test_pictures_go_in_one_request_only_past_the_context_limit(self):
        out = history([SYSTEM, FIRST, 1, "append", 0, False], turns_and_builds(5, 2000))
        b = out["builds"]
        self.assertEqual(b[1], [PREFIX + turn(1), {}, 887])
        self.assertEqual(b[2], [PREFIX + turn(1) + turn(2), {}, 1744])
        # 2601 with every picture: past 2000, so both pictures beyond the newest go in this one request.
        self.assertEqual(b[3], [PREFIX + gone(1) + gone(2) + turn(3), {"frames_dropped": 2, "image_epochs": 1}, 1023])
        # 1880: two pictures again, one more than kept, and no edit, so the request starts with the previous one.
        self.assertEqual(b[4], [PREFIX + gone(1) + gone(2) + turn(3) + turn(4), {}, 1880])
        self.assertEqual(b[4][0][:len(b[3][0])], b[3][0])
        # 2737: past the limit again.
        self.assertEqual(b[5], [PREFIX + gone(1) + gone(2) + gone(3) + gone(4) + turn(5),
                                {"frames_dropped": 2, "image_epochs": 1}, 1159])
        self.assertEqual(out["counts"], {"frames_dropped": 4, "results_shortened": 0, "replies_shortened": 0,
                                         "turns_dropped": 0, "image_epochs": 2})

    def test_the_cache_mark_sits_on_the_opening(self):
        out = history([SYSTEM, FIRST, 4, "append", 0, True], turns_and_builds(1, 100000))
        opening = [{"role": "system", "content": [{"type": "text", "text": SYSTEM, "cache_control": MARK}]},
                   {"role": "user", "content": [{"type": "text", "text": FIRST, "cache_control": MARK}]}]
        self.assertEqual(out["builds"][0], [opening, {}, 30])
        self.assertEqual(out["builds"][1][:2], [opening + turn(1), {}])

    def test_without_the_mark_the_opening_is_two_plain_texts(self):
        out = history([SYSTEM, FIRST, 4, "append", 0, False], [{"build": 100000}])
        self.assertEqual(out["builds"][0], [PREFIX, {}, 30])

    def test_an_unknown_mode_is_refused(self):
        self.assertEqual(history([SYSTEM, FIRST, 4, "keep", 0, False], []),
                         {"error": "image_mode must be one of swap, append"})

    def test_two_runs_send_the_same_messages(self):
        steps = turns_and_builds(5, 2000)
        self.assertEqual(history([SYSTEM, FIRST, 1, "append", 0, True], steps),
                         history([SYSTEM, FIRST, 1, "append", 0, True], steps))


if __name__ == "__main__":
    unittest.main()
