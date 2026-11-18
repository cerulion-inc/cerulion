# SPDX-License-Identifier: AGPL-3.0-only
"""The exit words: one table, and the only one (ARCHITECTURE.md section 3.3).

Every stage ends on one physical event, read from measured state. The table maps each event to one exit word and
says whether that event was the end the caller stated ("as asked"). The turn frame, the conformance vectors of the
body contract and every adapter read this table; none keeps a second one. ROWS carries the rows in the document's
order with the document's own wording, and tests/conformance/test_exit_table.py compares them with the document
word for word, so the document stays the source.

    from rcpx.body import exits
    exits.exit_for("goal_reached")                       -> {"exit": "reached", "as_asked": "yes"}
    exits.exit_for("no_further_motion", stated=("stalled",))  -> {"exit": "stalled", "as_asked": "yes"}
    exits.exit_for("no_further_motion", stated=("reached",))  -> {"exit": "stalled", "as_asked": "no"}
    exits.contact_event(stated=("contact",), on_other_thing=False)  -> "contact_stated_end"

A row's key names the event (what ended the stage); the exit word is what the operator reads. The keys are this
module's own; the words and the "as asked" rule are the document's.
"""

import json

YES, NO, IF_STATED = "yes", "no", "yes if stated, else no"

# (event key, what ended the stage [the document's wording], exit word, as asked [the document's wording],
#  the until condition that states this end, or None when no until condition can state it)
ROWS = (
    ("goal_reached", "The goal was reached within the tolerance", "reached", YES, "reached"),
    ("motion_ended_short",
     "The motion ended, nothing was met, and the achieved pose is short of the asked one by more than the "
     "tolerance (lag; the body clipped the command)", "short", NO, None),
    ("contact_stated_end", "Contact, and contact was the stated end", "contact", YES, "contact"),
    ("contact_not_stated_end",
     "Contact that was not the stated end: before the target, or on something other than the thing named "
     "(touched says on what)", "contact", NO, None),
    ("effort_reached", "The effort the caller stated was reached", "effort", YES, "effort"),
    ("travel_reached", "The travel the caller stated was reached", "travel", YES, "travel"),
    ("resistance_fell", "The resistance at the tool fell while the tool went on", "gave", IF_STATED, "gave"),
    ("no_further_motion", "No further motion under the commanded effort", "stalled", IF_STATED, "stalled"),
    ("part_lost", "The tool lost the part it was pushing or following", "slipped", NO, None),
    ("hold_lost", "The declared hold signals read \"not held\"", "lost_hold", IF_STATED, "lost_hold"),
    ("watch_changed", "The watched region or signal changed", "watched", YES, "watched"),
    ("time_ran_out", "The stated time ran out", "timed", YES, "time"),
    ("bound_reached",
     "A bound in force was reached before the stated end: a travel bound, the effort ceiling, the time in "
     "contact, the reach margin. This includes the bound reached while the part still gives", "bound", NO, None),
    ("safety_ended",
     "The safety core ended the stage (the rating stop, the path check at run time, the dark gate, a declared "
     "never condition, a fault the body reported), or an expectation monitor did (the target left view). A field "
     "\"why\" names which. A contact that the contact stop caught is not this row: it is the contact row above",
     "stopped", NO, None),
    ("cancelled", "The operator cancelled, or the front went silent past its declared wait", "cancelled", NO, None),
    ("not_admitted", "The call was not admitted; nothing moved", "refused", NO, None),
)

EVENTS = tuple(r[0] for r in ROWS)
WORDS = tuple(dict.fromkeys(r[2] for r in ROWS))            # the exit words, in the table's order, each once
# the until conditions of the stage (ARCHITECTURE.md section 2.5), each with the one event that satisfies it
UNTIL = {r[4]: r[0] for r in ROWS if r[4] is not None}
_BY_EVENT = {r[0]: r for r in ROWS}


def exit_for(event, stated=()):
    """{"exit": word, "as_asked": "yes" | "no"} for the event that ended a stage. stated: the until conditions the
    caller wrote in the stage (names of UNTIL). Raises KeyError for an event the table does not hold: an adapter
    that meets one has found a hole in the table, and the answer is a change to the document, never a new word."""
    key, _what, word, rule, until = _BY_EVENT[event]
    if rule == IF_STATED:
        asked = YES if until in tuple(stated) else NO
    else:
        asked = rule
    return {"exit": word, "as_asked": asked}


def contact_event(stated=(), on_other_thing=False):
    """Which of the two contact rows a contact is: the stated end only when the caller stated contact AND the
    contact was not on something other than the thing named."""
    return "contact_stated_end" if ("contact" in tuple(stated) and not on_other_thing) else "contact_not_stated_end"


def consistent(exit_word, as_asked, stated=()):
    """True when (exit word, as asked) is a pair the table can give for a stage that stated these until
    conditions. Used to check an adapter's recorded answer without knowing the event."""
    stated = tuple(stated)
    for key in EVENTS:
        _key, _what, word, rule, until = _BY_EVENT[key]
        if word != exit_word:
            continue
        if rule == YES and until is not None and until not in stated:
            continue              # an end only a stated until condition can give (reached, effort, travel, ...)
        if exit_for(key, stated)["as_asked"] == as_asked:
            return True
    return False


def as_table():
    """The table as plain data (what tests/conformance/exit_table.json holds, byte for byte through dumps())."""
    return {"source": "ARCHITECTURE.md section 3.3, the exit words",
            "as_asked_values": [YES, NO, IF_STATED],
            "rows": [{"event": k, "what_ended_the_stage": what, "exit": word, "as_asked": rule, "until": until}
                     for k, what, word, rule, until in ROWS],
            "words": list(WORDS), "until": dict(UNTIL)}


def dumps():
    return json.dumps(as_table(), indent=1, sort_keys=True) + "\n"


if __name__ == "__main__":
    import sys

    sys.stdout.write(dumps())
