# SPDX-License-Identifier: AGPL-3.0-only
"""The texts and limits of the offer engine, and the one rule for cutting an evidence phrase.

An offer is {n, noticed, can, next}: noticed is the gauge's own evidence phrase for the event (never rewritten
here, only cut), can is one of the texts below (what the robot can take over), next is a call. No text here grades
the operator, guesses what it is, or tells it what it ought to do; part names never stand here, they arrive at run
time from the manual. Pure: no import but re, no state.
"""

import re

OFFER_MAX = 300        # characters of the whole offer object as JSON; over it: can is dropped, then noticed is cut
NOTICED_MAX = 110      # characters of the evidence phrase
CAN_MAX = 80           # characters of a can text
RUNS_MAX = 60          # characters of the "runs" note that shows which call a take stands for
UNSAFE_ARGS = ("firm", "further", "force_n", "max_n")   # no written call holds one of these, at any depth
UNSAFE_KEY_PARTS = ("force", "newton")                  # nor a key that holds one of these words
UNSAFE_KEY_ENDS = ("_n",)                               # nor a key that ends so (the suffix for newtons here)
GAP_CALLS = 3          # no offer within this many calls of the last, unless the skill holds 2 or more open events
AGAIN_CALLS = 10       # the same offer id a second time only after this many calls, with the same exception
ROUTINE_CALLS = 5      # a routine is refused with fewer calls left than its default 3 steps plus 2
STEP_DOWN_RUN = 3      # confirmed picks or acts in a row before a step-down offer
UNTRIED_MAX = 3        # ready calls in an untried list
SKILLS = ("seeing", "acting", "checking")
SIGNAL_SKILL = {
    "body": "seeing", "mislabel": "seeing", "samepx": "seeing", "air": "seeing", "rays15": "seeing",
    "refuse": "acting", "repeat": "acting", "idle": "acting", "wind_down": "checking",
}
OFFERS = {
    "O1": {
        "skill": "seeing", "signals": ("body", "mislabel", "samepx", "air", "rays15"),
        "can": {
            "words": "number what its vision model finds for each part the manual names; you pick",
            "plain": "number the things in the picture: you pick a number, not a pixel",
        },
        "tool": "candidates", "needs": (),
    },
    "O2": {
        "skill": "seeing", "signals": ("body", "mislabel", "samepx", "air", "rays15"),
        "can": "look from the sides this picture cannot show and list what it finds there",
        "tool": "seek", "needs": (),
    },
    "O3": {
        "skill": "seeing", "signals": ("rays15",),
        "can": "measure the point itself from one pixel", "tool": "locate", "needs": (),
    },
    "O4": {
        "skill": "seeing", "signals": ("air",),
        "can": "go down the line of your mark until it meets the part", "tool": "choose", "needs": (),
    },
    "O5": {
        "skill": "acting", "signals": ("refuse",),
        "can": "do the whole step: reach, feel, push by evidence, compare before and after",
        "tool": "push", "needs": (),
    },
    "O7": {
        "skill": "acting", "signals": ("repeat", "idle", "refuse"),
        "can": {
            "cue": "try each numbered candidate until the manual's cue changes; report each try",
            "yours": "try each numbered candidate under your stop condition; report each try",
        },
        "tool": "try_each", "needs": ("until", "candidates"),
    },
    "W1": {
        "skill": "checking", "signals": ("wind_down",),
        "can": "there is a step not tried yet; the robot can run it", "tool": None, "needs": (),
    },
    "D1": {
        "skill": "seeing", "signals": (), "can": "stop numbering every look", "tool": "assist", "needs": (),
        "args": {"set": "plain"},
    },
    "D2": {
        "skill": "acting", "signals": (), "can": "leave the steps to you", "tool": "assist", "needs": (),
        "args": {"set": "own"},
    },
}
STEP_DOWN_NOTICED = {
    "D1": "your picks at calls {calls} were each confirmed by the robot's check",
    "D2": "your acts at calls {calls} were each confirmed at the first try",
}
CALLS_RE = re.compile(r"\bcalls? \d+(?:(?:, | and )\d+)*")


def cut(text, limit):
    """text cut to at most limit characters. Every call number the cut would lose or break is kept in a tail
    "... (calls 12, 20)"; a phrase that cannot be cut that way (no room for its numbers, or nothing left before
    them) gives "", and the engine then makes no offer rather than one without its evidence."""
    if not isinstance(text, str) or not isinstance(limit, int) or isinstance(limit, bool):
        return ""
    text = " ".join(text.split())
    if len(text) <= limit:
        return text
    if limit <= 0:
        return ""
    refs = [(m.start(), m.end(), re.findall(r"\d+", m.group(0))) for m in CALLS_RE.finditer(text)]
    end = limit - 3
    tail = "..."
    for _ in range(len(refs) + 2):
        lost = [r for r in refs if r[1] > end]
        nums = []
        for r in lost:
            for x in r[2]:
                if x not in nums:
                    nums.append(x)
        tail = "..." if not nums else "... (call" + ("s " if len(nums) > 1 else " ") + ", ".join(nums) + ")"
        new_end = limit - len(tail)
        if lost:
            new_end = min(new_end, lost[0][0])
        new_end = max(new_end, 0)
        if new_end >= end:
            break
        end = new_end
    head = text[:end].rstrip(" ,;:(")
    if not head or len(head) + len(tail) > limit:
        return ""
    return head + tail


def noticed(event):
    """The event's own evidence phrase, cut to NOTICED_MAX; "" when the event carries none."""
    if not isinstance(event, dict):
        return ""
    evidence = event.get("evidence")
    return cut(evidence, NOTICED_MAX) if isinstance(evidence, str) and evidence.strip() else ""
