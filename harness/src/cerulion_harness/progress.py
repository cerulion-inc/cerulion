# SPDX-License-Identifier: AGPL-3.0-only
"""One line per model reply while an episode runs. Standard library only.

The lines are read from the files the episode writes in its scratch folder: the operator's log (one "reply"
record per model reply, with the token counts the endpoint reported) and the simulator's truth log (one line per
control step). poll() is called as each request reaches the relay, which is after the previous reply was logged
and its tool call ran, and once more when the episode has ended; each reply is printed once:

    2026-11-18T16:11:20Z plain reply 3: control step 53 of 900; tokens in 14432 (6100 cached), out 1234
    (1000 reasoning); so far 0.0345 USD at list price, 0.0379 USD reported by the endpoint

The control step is the number of control steps the episode has used once that reply's call ran. USD at list
price is the package's price table (prices.py); the endpoint's own figure is the sum of usage.cost over the
replies that carry one, printed only once a reply does.
"""
import json
import os
import re
import threading
import time

from cerulion_harness import prices


def utc_now():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


class Tail:
    """The complete lines a growing file gained since the last read."""

    def __init__(self, path):
        self.path, self.offset, self.rest = path, 0, b""

    def lines(self):
        try:
            with open(self.path, "rb") as f:
                f.seek(self.offset)
                data = f.read()
        except OSError:
            return []
        self.offset += len(data)
        *done, self.rest = (self.rest + data).split(b"\n")
        return done


def _record(raw):
    try:
        rec = json.loads(raw)
    except ValueError:
        return None
    return rec if isinstance(rec, dict) else None


def reported_cost(usage):
    """usage.cost of one reply when the endpoint reports one (OpenRouter does), else None."""
    cost = usage.get("cost") if isinstance(usage, dict) else None
    return float(cost) if isinstance(cost, (int, float)) and not isinstance(cost, bool) else None


class Progress:
    """The progress lines of one setup's episode. group is the driver's folder for it in the scratch folder: one
    folder CELL_aN per attempt, and _scoring/CELL_aN beside them."""

    def __init__(self, arm, group, cell, model, control_steps, write, clock=None):
        self.arm, self.group, self.cell, self.control_steps = arm, group, cell, control_steps
        self.key = prices.key_of(model)
        self.write, self.clock = write, clock
        self._attempts = {}
        self._lock = threading.Lock()
        self._name = re.compile(re.escape(cell) + r"_a(\d+)")

    def _numbers(self):
        try:
            names = os.listdir(self.group)
        except OSError:
            return []
        found = [int(m.group(1)) for m in map(self._name.fullmatch, names) if m]
        return sorted(n for n in found if os.path.isdir(os.path.join(self.group, f"{self.cell}_a{n}")))

    def poll(self):
        """Print a line for every reply logged since the last poll."""
        with self._lock:
            for n in self._numbers():
                name = f"{self.cell}_a{n}"
                state = self._attempts.setdefault(n, {
                    "log": Tail(os.path.join(self.group, name, "operator.jsonl")),
                    "truth": Tail(os.path.join(self.group, "_scoring", name, "truth_log.jsonl")),
                    "usages": [], "step": 0, "replies": 0})
                for raw in state["truth"].lines():
                    rec = _record(raw)
                    step = rec.get("step") if rec else None
                    if isinstance(step, int) and not isinstance(step, bool) and step > state["step"]:
                        state["step"] = step
                for raw in state["log"].lines():
                    rec = _record(raw)
                    if rec and rec.get("kind") == "reply":
                        self.write(self.line(n, rec, state))

    def line(self, attempt, rec, state):
        usage = rec.get("usage") if isinstance(rec.get("usage"), dict) else {}
        try:
            counts = prices.tokens(usage) if usage else None
        except (TypeError, ValueError):   # counts that are not numbers are not counted
            counts, usage = None, {}
        state["usages"].append(usage)
        state["replies"] += 1
        turn = rec.get("turn")
        number = turn if isinstance(turn, int) and not isinstance(turn, bool) else state["replies"]
        again = f" (attempt {attempt})" if attempt > 1 else ""
        if counts is not None:
            p, cached, _, c, think = counts
            tokens = f"tokens in {p} ({cached} cached), out {c} ({think} reasoning)"
        else:
            tokens = "no token counts reported"
        listed = prices.cost_of(state["usages"], self.key) if self.key else None
        cost = f"{listed:.4f} USD at list price" if listed is not None else "no list price for this model"
        own = [x for x in map(reported_cost, state["usages"]) if x is not None]
        if own:
            cost += f", {sum(own):.4f} USD reported by the endpoint"
        return (f"{(self.clock or utc_now)()} {self.arm} reply {number}{again}: control step {state['step']} of "
                f"{self.control_steps}; {tokens}; so far {cost}")
