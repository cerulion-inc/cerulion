# SPDX-License-Identifier: AGPL-3.0-only
"""The words of the side entry's refusal in an order that is the same in every process.

A set of texts iterates in the order of the process's string hash (PYTHONHASHSEED), so a list made from one gives
other words on every run of the same code. The reasons are ordered here by a total key: the count of bearings each
ended, then the text.
"""


def reasons_by_count(whys):
    """[(count, reason)] of the bearings' reasons, each line of whys "bearing B deg: REASON": the most bearings first,
    reasons of the same count by their text. The count is of the lines that end with the reason, as the refusal's
    words have always counted it."""
    reasons = list(dict.fromkeys(w.split(": ", 1)[1] for w in whys))
    counted = [(sum(1 for x in whys if x.endswith(r)), r) for r in reasons]
    return sorted(counted, key=lambda nr: (-nr[0], nr[1]))
