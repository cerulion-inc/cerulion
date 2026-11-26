// SPDX-License-Identifier: AGPL-3.0-only
//! The vectors. Each names the lines of `docs/rcp.md` (or of the crate's
//! `AGENTS.md`) it is written from; the manifest and the stand-in graph are
//! the ones in [`super`].

use serde_json::json;

use super::{call, entry, frame, reply, Effect, Event, Outcome, Stage, Vector};

/// `tools/list` is the manifest's tools in the order it declares them, then
/// `estop`, then `estop_clear`.
///
/// docs/rcp.md, Tools: "`tools/list` returns the manifest's tools in the
/// order the manifest declares them, then the built-in `estop`, then, under
/// `limits.estop_clear: agent`, the built-in `estop_clear`."
pub fn the_tool_list_is_the_manifest_then_estop_then_estop_clear() -> Vector {
    Vector {
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
        events: vec![call(1, "arm.mute", q())],
        expect: vec![
            entry("arm.mute", Stage::Intent),
            frame("arm.mute", q()),
            entry("arm.mute", Stage::Error("timeout")),
            reply(1, Outcome::TimedOut).carrying("/safety_class", json!("reversible")),
        ],
    }
}
