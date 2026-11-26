// SPDX-License-Identifier: AGPL-3.0-only
//! The vectors. Each names the lines of `docs/rcp.md` (or of the crate's
//! `AGENTS.md`) it is written from; the manifest and the stand-in graph are
//! the ones in [`super`].

use serde_json::json;

use super::{
    call, entry, frame, latch_holds, no_latch_file, reply, start, Clearing, Effect, Event, Given,
    Latch, Outcome, Says, Stage, Vector,
};

/// `tools/list` is the manifest's tools in the order it declares them, then
/// `estop`, then `estop_clear`.
///
/// docs/rcp.md, Tools: "`tools/list` returns the manifest's tools in the
/// order the manifest declares them, then the built-in `estop`, then, under
/// `limits.estop_clear: agent`, the built-in `estop_clear`."
pub fn the_tool_list_is_the_manifest_then_estop_then_estop_clear() -> Vector {
    Vector {
        given: Given::fresh(),
        events: vec![Event::List],
        expect: vec![Effect::Listed(vec![
            "arm.step",
            "arm.slow",
            "arm.mute",
            "arm.peek",
            "arm.cut",
            "estop",
            "estop_clear",
        ])],
    }
}

/// Each call is published once, answered by its own `call_id` past answers to
/// no call, and receipted `intent` then `ok`.
///
/// docs/rcp.md, Tools, steps 5 to 7: a `reversible` tool "appends an `intent`
/// receipt"; the call "is published on `journal.calls_topic` as one
/// `std_msgs/String` frame, `{"call_id": N, "tool": "...", "args": {...}}`";
/// the daemon waits "for a frame on `journal.results_topic` whose JSON carries
/// the same `call_id`", and "frames that answer no pending call are dropped";
/// "An `ok` or `error:<class>` receipt closes the call". A success returns
/// `{"call_id": N, "receipt": ..., "result": <the graph's answer>,
/// "safety_class": "..."}`, and "`N` is an integer below 2^53, so a JSON
/// number holds it exactly". Safety classes: "Every result of a manifest tool
/// call after the name lookup, success or refusal, carries
/// `"safety_class"`".
pub fn each_call_is_published_once_answered_by_its_call_id_and_receipted() -> Vector {
    let q = || json!({ "q": [0.1, -0.2] });
    Vector {
        given: Given::fresh(),
        events: vec![
            call(1, "arm.step", q()),
            call(2, "arm.step", q()),
            call(3, "arm.step", q()),
        ],
        expect: vec![
            entry("arm.step", Stage::Intent),
            frame("arm.step", q()),
            entry("arm.step", Stage::Ok),
            reply(1, Outcome::Ran)
                .answering(0)
                .carrying("/safety_class", json!("reversible")),
            entry("arm.step", Stage::Intent),
            frame("arm.step", q()),
            entry("arm.step", Stage::Ok),
            reply(2, Outcome::Ran)
                .answering(1)
                .carrying("/safety_class", json!("reversible")),
            entry("arm.step", Stage::Intent),
            frame("arm.step", q()),
            entry("arm.step", Stage::Ok),
            reply(3, Outcome::Ran)
                .answering(2)
                .carrying("/safety_class", json!("reversible")),
        ],
    }
}

/// A name the manifest does not declare, and arguments outside a tool's
/// schema, are refused and receipted; none reaches the graph.
///
/// docs/rcp.md, Tools, step 1: "A name the manifest does not declare is
/// refused (`not_in_manifest`) and receipted `denied`"; step 2: "The arguments
/// are checked against `input_schema` (`invalid_args`, receipted
/// `error:invalid_args`); the error message names the argument that failed,
/// in double quotes".
/// The manifest: `minItems` holds the array length,
/// and `additionalProperties` is `false` by default. The page's opening:
/// "refused calls are in the receipt log only". Safety classes: "Every result
/// of a manifest tool call after the name lookup, success or refusal, carries
/// `"safety_class"`". AGENTS.md: "Any other name is receipted `denied` and
/// never reaches the graph".
pub fn an_undeclared_tool_and_bad_arguments_never_reach_the_graph() -> Vector {
    Vector {
        given: Given::fresh(),
        events: vec![
            call(1, "arm.fly", json!({})),
            call(2, "arm.step", json!({ "q": [1.0] })),
            call(3, "arm.step", json!({ "q": [1.0, 2.0], "speed": 3 })),
        ],
        expect: vec![
            entry("arm.fly", Stage::Denied),
            reply(1, Outcome::Refused("not_in_manifest")),
            entry("arm.step", Stage::Error("invalid_args")),
            reply(2, Outcome::Refused("invalid_args"))
                .carrying("/safety_class", json!("reversible"))
                .mentioning("/error/message", "\"q\""),
            entry("arm.step", Stage::Error("invalid_args")),
            reply(3, Outcome::Refused("invalid_args"))
                .carrying("/safety_class", json!("reversible"))
                .mentioning("/error/message", "\"speed\""),
        ],
    }
}

/// A name no tool can have, too long or with a character outside the rule, is
/// refused like any undeclared name, and its receipt names a fixed verb in
/// its place.
///
/// docs/rcp.md, Tools, step 1: "A name outside the tool name rule (1 to 128
/// characters from `[A-Za-z0-9_.-]`) is receipted under the verb
/// `<invalid-tool-name>`"; it is still a name the manifest does not declare
/// (`not_in_manifest`, receipted `denied`).
pub fn a_name_no_tool_can_have_is_receipted_under_a_fixed_verb() -> Vector {
    Vector {
        given: Given::fresh(),
        events: vec![
            call(1, "x".repeat(300), json!({})),
            call(2, "arm/step", json!({})),
        ],
        expect: vec![
            entry("<invalid-tool-name>", Stage::Denied),
            reply(1, Outcome::Refused("not_in_manifest")),
            entry("<invalid-tool-name>", Stage::Denied),
            reply(2, Outcome::Refused("not_in_manifest")),
        ],
    }
}

/// A call the graph never answers times out, says it may have run, and is
/// receipted `intent` then `error:timeout`.
///
/// docs/rcp.md, Tools, step 6: the daemon "waits up to `timeout_ms`" (300 for
/// `arm.mute`), and "On `timeout` the error carries `"may_have_executed":
/// true`: the call was published and the graph may still act on it"; step 7:
/// "An `ok` or `error:<class>` receipt closes the call". Safety classes:
/// "Every result of a manifest tool call after the name lookup, success or
/// refusal, carries `"safety_class"`".
pub fn a_silent_graph_times_out_and_the_call_may_have_run() -> Vector {
    let q = || json!({ "q": [0.0, 0.0] });
    Vector {
        given: Given::fresh(),
        events: vec![call(1, "arm.mute", q())],
        expect: vec![
            entry("arm.mute", Stage::Intent),
            frame("arm.mute", q()),
            entry("arm.mute", Stage::Error("timeout")),
            reply(1, Outcome::TimedOut).carrying("/safety_class", json!("reversible")),
        ],
    }
}

/// `estop` closes the latch and writes its file; a motion call is then
/// refused and never published, while a read tool still runs.
///
/// docs/rcp.md, E-stop: `estop` "takes an optional `reason` (cut to 200
/// characters)", "publishes its own frame, `{"call_id": N, "tool": "estop",
/// "args": {"reason": ...}}`", "returns `estopped: true`,
/// `graph_acknowledged` saying whether the graph answered, `call_id` (its
/// frame's), `receipt` (its `intent` receipt, `{"seq", "hash", "prev"}`) and
/// `result` (the graph's answer ...)", and "Its `intent` receipt is closed by
/// `ok` when the graph answered"; "Its description in `tools/list` names
/// `estop_clear` under `limits.estop_clear: agent`"; "The latch is also
/// written to `<receipts>.estop`, a file holding the `reason` of the `estop`
/// that wrote it (cut to 200 characters), then a newline". Tools, step 4:
/// "While the e-stop is engaged a `reversible` or `irreversible` tool is
/// refused (`estopped`) and receipted `denied`; the message says how the latch
/// clears: with `estop_clear` under `limits.estop_clear: agent`"; "`read`
/// tools skip this step." Safety classes: `read` writes no intent receipt and
/// "runs" while e-stopped.
pub fn estop_refuses_motion_and_lets_reads_run() -> Vector {
    Vector {
        given: Given::fresh(),
        events: vec![
            Event::Describe("estop"),
            call(1, "estop", json!({ "reason": "x".repeat(500) })),
            call(2, "arm.step", json!({ "q": [0.0, 0.0] })),
            call(3, "arm.peek", json!({})),
            Event::ProbeLatchFile,
        ],
        expect: vec![
            Effect::Described(vec![Says::Mentions("estop_clear")]),
            entry("estop", Stage::Intent),
            frame("estop", json!({ "reason": "x".repeat(200) })),
            entry("estop", Stage::Ok),
            reply(1, Outcome::Ran)
                .answering(0)
                .receipted()
                .carrying("/estopped", json!(true))
                .carrying("/graph_acknowledged", json!(true)),
            entry("arm.step", Stage::Denied),
            reply(2, Outcome::Refused("estopped"))
                .carrying("/safety_class", json!("reversible"))
                .mentioning("/error/message", "estop_clear"),
            frame("arm.peek", json!({})),
            entry("arm.peek", Stage::Ok),
            reply(3, Outcome::Ran)
                .answering(1)
                .carrying("/safety_class", json!("read")),
            latch_holds(format!("{}\n", "x".repeat(200))),
        ],
    }
}

/// A daemon that starts and finds the latch file an earlier one left starts
/// with the e-stop engaged.
///
/// docs/rcp.md, E-stop: "a daemon that starts and finds that file starts with
/// the latch closed", and the file holds "the `reason` of the `estop` that
/// wrote it", here the earlier daemon's. Tools, step 4: refused
/// "(`estopped`) and receipted `denied`; the message says how the latch
/// clears: with `estop_clear` under `limits.estop_clear: agent`". AGENTS.md:
/// "a daemon that finds the file starts latched".
pub fn a_latch_left_by_an_earlier_daemon_refuses_motion() -> Vector {
    Vector {
        given: Given {
            clearing: Clearing::Agent,
            latch: Latch::LeftClosed,
        },
        events: vec![
            call(1, "arm.step", json!({ "q": [0.0, 0.0] })),
            Event::ProbeLatchFile,
        ],
        expect: vec![
            entry("arm.step", Stage::Denied),
            reply(1, Outcome::Refused("estopped"))
                .carrying("/safety_class", json!("reversible"))
                .mentioning("/error/message", "estop_clear"),
            latch_holds("earlier stop\n"),
        ],
    }
}

/// An `irreversible` tool runs with an intent receipt, is labelled in every
/// result, and is refused while e-stopped like any motion call.
///
/// docs/rcp.md, Safety classes: `irreversible` has an intent receipt and is
/// "refused" while e-stopped; "Every result of a manifest tool call after the
/// name lookup, success or refusal, carries `"safety_class"`"; "Receipt lines
/// record the tool name as `verb`". E-stop and Tools, step 4, as in
/// [`estop_refuses_motion_and_lets_reads_run`].
pub fn an_irreversible_tool_is_receipted_labelled_and_latch_gated() -> Vector {
    Vector {
        given: Given::fresh(),
        events: vec![
            call(1, "arm.cut", json!({})),
            call(2, "estop", json!({ "reason": "stop" })),
            call(3, "arm.cut", json!({})),
            call(4, "arm.peek", json!({})),
        ],
        expect: vec![
            entry("arm.cut", Stage::Intent),
            frame("arm.cut", json!({})),
            entry("arm.cut", Stage::Ok),
            reply(1, Outcome::Ran)
                .answering(0)
                .carrying("/safety_class", json!("irreversible")),
            entry("estop", Stage::Intent),
            frame("estop", json!({ "reason": "stop" })),
            entry("estop", Stage::Ok),
            reply(2, Outcome::Ran)
                .answering(1)
                .receipted()
                .carrying("/estopped", json!(true))
                .carrying("/graph_acknowledged", json!(true)),
            entry("arm.cut", Stage::Denied),
            reply(3, Outcome::Refused("estopped")).carrying("/safety_class", json!("irreversible")),
            frame("arm.peek", json!({})),
            entry("arm.peek", Stage::Ok),
            reply(4, Outcome::Ran)
                .answering(2)
                .carrying("/safety_class", json!("read")),
        ],
    }
}

/// `estop_clear` writes its intent, deletes the latch file, opens the latch
/// and tells the graph; motion runs again; a later `estop` closes the latch
/// and writes the file again.
///
/// docs/rcp.md, E-stop: `estop_clear` "appends an `intent` receipt", "deletes
/// `<receipts>.estop` and opens the latch", "when the latch was closed,
/// publishes `{"call_id": N, "tool": "estop_clear", "args": {"reason":
/// ...}}` on the calls topic", "appends `ok`", and "returns `estopped` (the
/// latch after the clear), `was_estopped`, `graph_acknowledged`, `receipt`
/// (its `intent` receipt) and, when it published its frame, that frame's
/// `call_id` and the graph's answer as `result`". "A later `estop` closes the
/// latch and writes the file again." The estops, their latch file and the
/// refused motion calls as in [`estop_refuses_motion_and_lets_reads_run`].
pub fn a_clear_reopens_motion_and_a_later_estop_closes_it_again() -> Vector {
    let q = || json!({ "q": [0.0, 0.0] });
    Vector {
        given: Given::fresh(),
        events: vec![
            call(1, "estop", json!({ "reason": "first" })),
            Event::ProbeLatchFile,
            call(2, "arm.step", q()),
            call(3, "estop_clear", json!({ "reason": "safe" })),
            Event::ProbeLatchFile,
            call(4, "arm.step", q()),
            call(5, "estop", json!({ "reason": "second" })),
            Event::ProbeLatchFile,
            call(6, "arm.step", q()),
        ],
        expect: vec![
            entry("estop", Stage::Intent),
            frame("estop", json!({ "reason": "first" })),
            entry("estop", Stage::Ok),
            reply(1, Outcome::Ran)
                .answering(0)
                .receipted()
                .carrying("/estopped", json!(true))
                .carrying("/graph_acknowledged", json!(true)),
            latch_holds("first\n"),
            entry("arm.step", Stage::Denied),
            reply(2, Outcome::Refused("estopped")).mentioning("/error/message", "estop_clear"),
            entry("estop_clear", Stage::Intent),
            frame("estop_clear", json!({ "reason": "safe" })),
            entry("estop_clear", Stage::Ok),
            reply(3, Outcome::Ran)
                .answering(1)
                .receipted()
                .carrying("/estopped", json!(false))
                .carrying("/was_estopped", json!(true))
                .carrying("/graph_acknowledged", json!(true)),
            no_latch_file(),
            entry("arm.step", Stage::Intent),
            frame("arm.step", q()),
            entry("arm.step", Stage::Ok),
            reply(4, Outcome::Ran).answering(2),
            entry("estop", Stage::Intent),
            frame("estop", json!({ "reason": "second" })),
            entry("estop", Stage::Ok),
            reply(5, Outcome::Ran)
                .answering(3)
                .receipted()
                .carrying("/estopped", json!(true))
                .carrying("/graph_acknowledged", json!(true)),
            latch_holds("second\n"),
            entry("arm.step", Stage::Denied),
            reply(6, Outcome::Refused("estopped")),
        ],
    }
}

/// A clear with arguments outside its schema is refused; a clear with nothing
/// latched writes its receipts and publishes nothing.
///
/// docs/rcp.md, E-stop: `estop_clear` takes "the same optional `reason` (at
/// most 200 characters; any other argument is refused as `invalid_args`,
/// receipted `error:invalid_args`)"; it "appends an `intent` receipt", "when
/// it was open, publishes nothing", "appends `ok`", and returns `estopped`,
/// `was_estopped`, `receipt` (its `intent` receipt), and `call_id` and
/// `result` "both `null` when it published nothing".
pub fn a_clear_with_nothing_latched_is_receipted_and_publishes_nothing() -> Vector {
    let q = || json!({ "q": [0.0, 0.0] });
    Vector {
        given: Given::fresh(),
        events: vec![
            call(1, "estop_clear", json!({ "speed": 1 })),
            call(2, "estop_clear", json!({ "reason": "idle" })),
            call(3, "arm.step", q()),
            Event::ProbeLatchFile,
        ],
        expect: vec![
            entry("estop_clear", Stage::Error("invalid_args")),
            reply(1, Outcome::Refused("invalid_args")),
            entry("estop_clear", Stage::Intent),
            entry("estop_clear", Stage::Ok),
            reply(2, Outcome::Ran)
                .receipted()
                .carrying("/estopped", json!(false))
                .carrying("/was_estopped", json!(false))
                .carrying("/call_id", json!(null))
                .carrying("/result", json!(null)),
            entry("arm.step", Stage::Intent),
            frame("arm.step", q()),
            entry("arm.step", Stage::Ok),
            reply(3, Outcome::Ran).answering(0),
            no_latch_file(),
        ],
    }
}

/// A clear opens a latch an earlier daemon left, and motion runs.
///
/// docs/rcp.md, E-stop: "a daemon that starts and finds that file starts with
/// the latch closed"; `estop_clear` "deletes `<receipts>.estop` and opens the
/// latch" and, "when the latch was closed", publishes its frame; it returns
/// `receipt` (its `intent` receipt) and "that frame's `call_id` and the
/// graph's answer as `result`", as in
/// [`a_clear_reopens_motion_and_a_later_estop_closes_it_again`].
pub fn a_clear_opens_a_latch_left_by_an_earlier_daemon() -> Vector {
    let q = || json!({ "q": [0.0, 0.0] });
    Vector {
        given: Given {
            clearing: Clearing::Agent,
            latch: Latch::LeftClosed,
        },
        events: vec![
            call(1, "estop_clear", json!({ "reason": "restart" })),
            Event::ProbeLatchFile,
            call(2, "arm.step", q()),
        ],
        expect: vec![
            entry("estop_clear", Stage::Intent),
            frame("estop_clear", json!({ "reason": "restart" })),
            entry("estop_clear", Stage::Ok),
            reply(1, Outcome::Ran)
                .answering(0)
                .receipted()
                .carrying("/estopped", json!(false))
                .carrying("/was_estopped", json!(true))
                .carrying("/graph_acknowledged", json!(true)),
            no_latch_file(),
            entry("arm.step", Stage::Intent),
            frame("arm.step", q()),
            entry("arm.step", Stage::Ok),
            reply(2, Outcome::Ran).answering(1),
        ],
    }
}

/// A latch file that cannot be deleted keeps the e-stop engaged.
///
/// docs/rcp.md, E-stop: `estop_clear` "appends an `intent` receipt", then "a
/// file that cannot be deleted leaves the latch closed and returns
/// `clear_failed`, receipted `error:clear_failed`". AGENTS.md: "An undeletable
/// file leaves the latch closed (`clear_failed`)". A daemon that finds the
/// file at start starts latched and refuses motion with a message naming
/// `estop_clear`, as in [`a_latch_left_by_an_earlier_daemon_refuses_motion`].
/// The undeletable latch file here is a directory, which has no text to
/// probe, so the closed latch is seen through the refused motion call.
pub fn an_undeletable_latch_file_keeps_the_estop_engaged() -> Vector {
    Vector {
        given: Given {
            clearing: Clearing::Agent,
            latch: Latch::Undeletable,
        },
        events: vec![
            call(1, "estop_clear", json!({ "reason": "try" })),
            call(2, "arm.step", json!({ "q": [0.0, 0.0] })),
        ],
        expect: vec![
            entry("estop_clear", Stage::Intent),
            entry("estop_clear", Stage::Error("clear_failed")),
            reply(1, Outcome::Refused("clear_failed")),
            entry("arm.step", Stage::Denied),
            reply(2, Outcome::Refused("estopped")).mentioning("/error/message", "estop_clear"),
        ],
    }
}

/// Under `limits.estop_clear: operator` the agent cannot clear the e-stop:
/// `estop_clear` is not listed, a call to it is refused, and the latch file
/// stays.
///
/// docs/rcp.md, The manifest, `limits`: "`operator`: `estop_clear` is not
/// listed, a call to it is refused (`estop_clear_disabled`), and only deleting
/// the latch file and restarting the daemon clears the e-stop". AGENTS.md:
/// "`limits.estop_clear: operator` unlists `estop_clear` and refuses it
/// (`estop_clear_disabled`, receipted `denied`)". E-stop: the description of
/// `estop` under `operator` "says an operator clears the latch and does not
/// name `estop_clear`"; Tools, step 4: the `estopped` message says the latch
/// clears "by an operator under `operator`, where it does not name
/// `estop_clear`".
pub fn under_operator_clearing_the_agent_cannot_clear() -> Vector {
    Vector {
        given: Given {
            clearing: Clearing::Operator,
            latch: Latch::Absent,
        },
        events: vec![
            Event::List,
            Event::Describe("estop"),
            call(1, "estop", json!({ "reason": "stop" })),
            call(2, "estop_clear", json!({ "reason": "try" })),
            call(3, "arm.step", json!({ "q": [0.0, 0.0] })),
            Event::ProbeLatchFile,
        ],
        expect: vec![
            Effect::Listed(vec![
                "arm.step", "arm.slow", "arm.mute", "arm.peek", "arm.cut", "estop",
            ]),
            Effect::Described(vec![Says::Mentions("operator"), Says::Omits("estop_clear")]),
            entry("estop", Stage::Intent),
            frame("estop", json!({ "reason": "stop" })),
            entry("estop", Stage::Ok),
            reply(1, Outcome::Ran)
                .answering(0)
                .receipted()
                .carrying("/estopped", json!(true))
                .carrying("/graph_acknowledged", json!(true)),
            entry("estop_clear", Stage::Denied),
            reply(2, Outcome::Refused("estop_clear_disabled")),
            entry("arm.step", Stage::Denied),
            reply(3, Outcome::Refused("estopped"))
                .mentioning("/error/message", "operator")
                .omitting("/error/message", "estop_clear"),
            latch_holds("stop\n"),
        ],
    }
}

/// `estop` does not wait behind a call in flight: it is published and
/// answered while the graph still holds that call's answer; the call in
/// flight then completes, and a motion call sent after the estop is refused.
///
/// docs/rcp.md, E-stop: `estop` "closes the latch before anything else,
/// without waiting for a call in flight; publishes its own frame ... ahead of
/// anything queued"; "While the latch is closed, `reversible` and
/// `irreversible` calls are refused". Tools: "Calls other than `estop` are
/// handled one at a time, in the order they arrive." Reaching rcpd from a
/// desk: "the call it is running has already reached the graph, which
/// receives the e-stop frame next". AGENTS.md: `estop` "never waits on
/// `call_lock`". The estop's own receipts, frame and reply as in
/// [`estop_refuses_motion_and_lets_reads_run`].
pub fn estop_does_not_wait_behind_a_call_in_flight() -> Vector {
    let q = || json!({ "q": [0.0, 0.0] });
    Vector {
        given: Given::fresh(),
        events: vec![
            start(1, "arm.slow", q()),
            Event::AwaitFrame("arm.slow"),
            call(2, "estop", json!({ "reason": "now" })),
            start(3, "arm.step", q()),
            Event::Release,
            Event::AwaitReply(1),
            Event::AwaitReply(3),
        ],
        expect: vec![
            entry("arm.slow", Stage::Intent),
            frame("arm.slow", q()),
            entry("estop", Stage::Intent),
            frame("estop", json!({ "reason": "now" })),
            entry("estop", Stage::Ok),
            reply(2, Outcome::Ran)
                .answering(1)
                .receipted()
                .carrying("/estopped", json!(true))
                .carrying("/graph_acknowledged", json!(true)),
            entry("arm.slow", Stage::Ok),
            reply(1, Outcome::Ran).answering(0),
            entry("arm.step", Stage::Denied),
            reply(3, Outcome::Refused("estopped")),
        ],
    }
}

/// An `estop` that lands while a clear waits behind a call in flight is not
/// cleared by that clear.
///
/// docs/rcp.md, E-stop: `estop_clear` "waits behind a call in flight, then:
/// refuses with `estop_overtaken`, receipted `error:estop_overtaken` with
/// nothing changed, when an `estop` was called after the clear was
/// requested (the refusal carries `estopped`, the state of the latch)". The
/// estop's own receipts, frame, reply and latch file as in
/// [`estop_refuses_motion_and_lets_reads_run`].
/// Nothing a client sees shows the clear has reached the daemon, so a
/// [`Event::Pause`] lets it arrive before the estop.
pub fn an_estop_during_a_waiting_clear_is_not_cleared_by_it() -> Vector {
    let q = || json!({ "q": [0.0, 0.0] });
    Vector {
        given: Given::fresh(),
        events: vec![
            start(1, "arm.slow", q()),
            Event::AwaitFrame("arm.slow"),
            start(2, "estop_clear", json!({ "reason": "queued" })),
            Event::Pause,
            call(3, "estop", json!({ "reason": "late" })),
            Event::Release,
            Event::AwaitReply(1),
            Event::AwaitReply(2),
            call(4, "arm.step", q()),
            Event::ProbeLatchFile,
        ],
        expect: vec![
            entry("arm.slow", Stage::Intent),
            frame("arm.slow", q()),
            entry("estop", Stage::Intent),
            frame("estop", json!({ "reason": "late" })),
            entry("estop", Stage::Ok),
            reply(3, Outcome::Ran)
                .answering(1)
                .receipted()
                .carrying("/estopped", json!(true))
                .carrying("/graph_acknowledged", json!(true)),
            entry("arm.slow", Stage::Ok),
            reply(1, Outcome::Ran).answering(0),
            entry("estop_clear", Stage::Error("estop_overtaken")),
            reply(2, Outcome::Refused("estop_overtaken"))
                .receipted()
                .carrying("/estopped", json!(true)),
            entry("arm.step", Stage::Denied),
            reply(4, Outcome::Refused("estopped")),
            latch_holds("late\n"),
        ],
    }
}
