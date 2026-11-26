// SPDX-License-Identifier: AGPL-3.0-only
//! Checks over a run whose effects no one can list by hand, such as racing
//! calls whose order the client does not set. Each holds for any order the
//! daemon picks; each names the lines of `docs/rcp.md` it is written from.

use serde_json::json;

use super::{call, play, start, Daemon, Event, Given};

/// The latch the receipt log implies: an `estop` intent closes it; an
/// `estop_clear` intent opens it unless that clear's closing line is
/// `error:clear_failed`, since every other clear outcome is written after the
/// latch opened. Clears wait their turn, so a clear's closing line is the
/// next `estop_clear` line after its intent.
fn replay_latch(entries: &[(String, String)], initially: bool) -> bool {
    let mut closed = initially;
    for (i, (verb, outcome)) in entries.iter().enumerate() {
        match (verb.as_str(), outcome.as_str()) {
            ("estop", "intent") => closed = true,
            ("estop_clear", "intent") => {
                let closing = entries[i + 1..]
                    .iter()
                    .find(|(v, _)| v == "estop_clear")
                    .map(|(_, o)| o.as_str());
                if closing != Some("error:clear_failed") {
                    closed = false;
                }
            }
            _ => {}
        }
    }
    closed
}

/// 24 racing `estop` and `estop_clear` calls leave the latch, the receipt
/// log, the latch file and the graph's last frame agreeing.
///
/// docs/rcp.md, E-stop: `estop` "returns `estopped: true`", and each writes
/// "its `intent` receipt" and "publishes its own frame"; `estop` "closes the
/// latch, then, inside a lock, closes it again, writes the file and its
/// `intent` receipt, and queues its frame ahead of everything else, so a
/// clear that follows it publishes after that frame"; `estop_clear` "writes
/// its `intent`, deletes the file and opens the latch inside the same lock.
/// The receipt log therefore orders engages and clears as they took effect."
/// A clear that an `estop` overtakes is refused with `estop_overtaken`,
/// "carrying `estopped`, the state of the latch", and "leaves the `estop`
/// frame as the graph's last", and a clear that opens the latch "publishes
/// after that frame", so the graph's last e-stop frame is an `estop` exactly
/// when the latch ends closed. Tools, step 4: a motion call is refused
/// (`estopped`) while the latch is closed.
pub async fn racing_engages_and_clears_leave_the_latch_the_log_and_the_graph_agreeing<D: Daemon>() {
    let mut events = Vec::new();
    for i in 0..24 {
        let tool = if i % 2 == 0 { "estop" } else { "estop_clear" };
        events.push(start(i, tool, json!({ "reason": format!("r{i}") })));
    }
    events.extend((0..24).map(Event::AwaitReply));
    events.push(call(24, "arm.step", json!({ "q": [0.0, 0.0] })));
    events.push(Event::ProbeLatchFile);
    let trace = play::<D>(&Given::fresh(), &events).await;

    for i in 0..24 {
        let (is_error, body) = &trace.replies[&i];
        if i % 2 == 0 {
            assert!(!is_error, "an estop is never refused: {body}");
            assert_eq!(body["estopped"], true, "{body}");
        } else if *is_error {
            assert_eq!(body["error"]["code"], "estop_overtaken", "{body}");
            assert_eq!(
                body["estopped"], true,
                "an overtaken clear leaves the latch closed: {body}"
            );
        }
    }
    let intents = trace
        .entries
        .iter()
        .filter(|(verb, outcome)| verb == "estop" && outcome == "intent");
    assert_eq!(intents.count(), 12, "every estop writes its intent");
    let frames = trace.frames.iter().filter(|f| f["tool"] == "estop");
    assert_eq!(frames.count(), 12, "every estop publishes its frame");

    let latched = replay_latch(&trace.entries, false);
    let (refused, body) = &trace.replies[&24];
    assert_eq!(
        *refused, latched,
        "the latch matches the receipt log: {body} after {:?}",
        trace.entries
    );
    if latched {
        assert_eq!(body["error"]["code"], "estopped", "{body}");
    }
    let file = match trace.latch.as_slice() {
        [file] => file.is_some(),
        probes => panic!("one latch file probe, not {probes:?}"),
    };
    assert_eq!(file, latched, "the latch file matches the latch");
    let last = trace
        .frames
        .iter()
        .filter_map(|f| f["tool"].as_str())
        .rfind(|t| *t == "estop" || *t == "estop_clear");
    let expected = if latched { "estop" } else { "estop_clear" };
    assert_eq!(
        last,
        Some(expected),
        "the graph's last word matches the latch"
    );
}
