// SPDX-License-Identifier: AGPL-3.0-only
//! The RUN registry `/__cerulion/runs` over REAL
//! iceoryx2.
//!
//! Every test mints its OWN isolated SHM root (`testing::iceoryx_test_config`),
//! so a gather here sees exactly the runs this test published — which is what
//! makes an EXACT-SET assertion on the answer meaningful at all. Parallel-safe:
//! no process-global state, no `#[serial]`.
//!
//! # What these arms are for
//!
//! The pure oracles in `transport::run_registry`'s own `mod tests` pin the wire
//! codec and the early-exit predicate. They cannot see the two properties that
//! decide whether the registry WORKS:
//!
//! * **A gather opens its subscriber AFTER the writer already sent.** The
//!   control service carries no history, so a gather can NEVER receive a run's
//!   first send — everything it hears is a republish (or a doorbell answer).
//!   If the belt were inert, every arm below would find nothing.
//! * **An empty answer is not an absence.** A live writer that never speaks
//!   must be reported `Incomplete`, not as a confident "no runs" — an earlier
//!   lesson, re-earned here because `cerulion bag record` will refuse to attach
//!   on exactly this answer.
//!
//! # Load discipline
//!
//! No arm states a wall in units of the republish interval or the gather
//! window (macOS background-QoS timer coalescing charges sleep slack PER
//! WAKEUP; a nominal 150 ms has been measured as 1100 to 1696 ms). Every claim is
//! an exact record set, a verdict, or a counter floor reached under a generous
//! seconds-scale condition wait, all of which load can delay but not invert.

use std::time::{Duration, Instant};

use cerulion_core::transport::mirror_registry::GatherCompleteness;
use cerulion_core::transport::run_registry::{
    gather_runs_on_config, RunHandle, RunRecord, RunState, RUN_GATHER_WINDOW,
    RUN_REGISTRY_MAX_READERS, RUN_REGISTRY_MAX_WRITERS, RUN_REGISTRY_QUEUE_DEPTH,
    RUN_REGISTRY_SERVICE_NAME,
};

type CerService = iceoryx2::service::ipc_threadsafe::Service;

/// A hand-built run record. Every field is DISTINCT and non-default so a
/// field-order slip in the codec cannot pass by coincidence.
fn record(run_id: u128, graph: &str) -> RunRecord {
    RunRecord {
        run_id,
        supervisor_pid: 4711,
        run_started_at_ns: 1_753_000_000_000_000_000,
        state: RunState::Live,
        graph_name: graph.to_string(),
        run_dir: format!("/tmp/{graph}-{run_id:032x}"),
    }
}

/// Gather until `want` records are visible, or fail after a GENEROUS seconds
/// ceiling. A ceiling in seconds is load-proof in the safe direction: load can
/// only delay a republish, and a broken belt never converges at all.
fn gather_until(config: &iceoryx2::config::Config, want: usize, what: &str) -> Vec<RunRecord> {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let gather = gather_runs_on_config(config, RUN_GATHER_WINDOW).expect("gather");
        if gather.records.len() == want {
            return gather.records;
        }
        assert!(
            Instant::now() < deadline,
            "{what}: expected {want} run record(s) within 10s, last answer was {:?} ({:?})",
            gather.records,
            gather.completeness
        );
    }
}

/// THE headline: a run published on this machine is DISCOVERED by a gather that
/// opened afterwards, byte-for-byte as it was published.
///
/// The gather's subscriber does not exist when `RunHandle::publish` sends, and
/// the control service carries no history — so finding the record at all proves
/// the republish/doorbell belt is live, not merely that `send` returned Ok.
#[test]
fn a_live_run_is_discovered_by_a_gather_that_opened_after_it() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let want = record(0x1111_2222_3333_4444_5555_6666_7777_8888, "go2_attach");
    let handle = RunHandle::publish_on_config(&config, want.clone()).expect("publish");

    let got = gather_until(&config, 1, "the live run");
    assert_eq!(
        got,
        vec![want],
        "the gathered record must equal the published one field for field"
    );

    // The gather ASKED and was ANSWERED — the no-inert pin. A wall
    // reading cannot separate a doorbell answer from a lucky timer tick on a
    // loaded runner; this counter can. A generous condition wait, because load
    // may delay the pump's wake but cannot stop the ring being queued.
    let deadline = Instant::now() + Duration::from_secs(10);
    while handle.doorbell_rings_answered() == 0 {
        assert!(
            Instant::now() < deadline,
            "the gather rings the doorbell every pass; this run's pump answered ZERO rings in \
             10s, so the gather was catching the republish timer instead of asking"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// A SECOND gather still finds the run. The first gather drained every frame
/// then dropped its subscriber, so the second can only be served by a fresh
/// republish — this is the belt asserted directly rather than inferred.
#[test]
fn a_second_gather_still_finds_the_run_so_the_republish_belt_is_real() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let want = record(0xABCD_0000_0000_0000_0000_0000_0000_0001, "belt");
    let _handle = RunHandle::publish_on_config(&config, want.clone()).expect("publish");

    let first = gather_until(&config, 1, "the first gather");
    assert_eq!(first, vec![want.clone()]);
    let second = gather_until(&config, 1, "the second gather");
    assert_eq!(
        second,
        vec![want],
        "a run must stay discoverable to every later gather, not only the first"
    );
}

/// TWO concurrent runs are BOTH discovered — the gather's completeness gate
/// exists precisely so a second writer on a different republish phase is not
/// silently dropped (the mirror registry's own independent-writer race).
#[test]
fn two_concurrent_runs_are_both_discovered() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let a = record(0x0000_0000_0000_0000_0000_0000_0000_00AA, "alpha");
    let b = record(0x0000_0000_0000_0000_0000_0000_0000_00BB, "beta");
    let _ha = RunHandle::publish_on_config(&config, a.clone()).expect("publish a");
    let _hb = RunHandle::publish_on_config(&config, b.clone()).expect("publish b");

    let mut got = gather_until(&config, 2, "two concurrent runs");
    got.sort_by_key(|r| r.run_id);
    assert_eq!(
        got,
        vec![a, b],
        "both live runs must appear, each with its own identity"
    );
}

/// A run that ENDS gracefully says so on the wire, and a run whose writer is
/// GONE disappears from the next gather.
///
/// Both halves in one body, because they are the two ways a run stops and a
/// consumer bound to a run's lifetime must tell them apart: `Ending` is the
/// operator stopping it, absence is the process vanishing.
#[test]
fn a_graceful_end_is_published_and_a_dropped_writer_disappears() {
    let config = cerulion_core::testing::iceoryx_test_config();
    let a = record(0x1234_0000_0000_0000_0000_0000_0000_0001, "stays");
    let b = record(0x1234_0000_0000_0000_0000_0000_0000_0002, "goes");
    let ha = RunHandle::publish_on_config(&config, a.clone()).expect("publish a");
    let hb = RunHandle::publish_on_config(&config, b).expect("publish b");
    let _ = gather_until(&config, 2, "both runs before the end");

    assert!(
        ha.set_ending(),
        "the first flip to Ending changes the state"
    );
    assert!(
        !ha.set_ending(),
        "a repeat flip is a no-op that still re-sends (idempotent for callers)"
    );

    // The surviving run now reports Ending; the dropped one is gone entirely.
    drop(hb);
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let got = gather_runs_on_config(&config, RUN_GATHER_WINDOW).expect("gather");
        if got.records.len() == 1 && got.records[0].state == RunState::Ending {
            assert_eq!(
                got.records[0],
                RunRecord {
                    state: RunState::Ending,
                    ..a
                },
                "the surviving run is unchanged except for its state"
            );
            return;
        }
        assert!(
            Instant::now() < deadline,
            "expected exactly the ENDING survivor within 10s; got {:?}",
            got.records
        );
    }
}

/// A machine with NO live run answers instantly and SETTLED — and a machine
/// whose only writer never speaks answers INCOMPLETE.
///
/// Both in one body, because the whole point is the DISCRIMINATION: an empty
/// record set means "there are no runs" in the first case and "I could not
/// establish what is running" in the second, and a consumer that conflates
/// them attaches to the wrong thing (or refuses to attach to the right one).
///
/// The unanswered writer is built the way a CRASHED one really looks — a live
/// publisher port on the registry service that never sends — from the SHIPPED
/// provisioning constants, so a change to the production service config makes
/// this open fail loudly rather than silently diverge.
#[test]
fn an_unanswered_live_writer_is_incomplete_never_a_settled_empty() {
    let config = cerulion_core::testing::iceoryx_test_config();

    // (a) the CONTROL: nothing is publishing, so an empty answer is EVIDENCE.
    let quiet = gather_runs_on_config(&config, RUN_GATHER_WINDOW).expect("gather");
    assert!(quiet.records.is_empty());
    assert_eq!(
        quiet.completeness,
        GatherCompleteness::Settled,
        "with zero publishers the registry itself reports there is nothing to hear"
    );

    // (b) a live-but-silent writer. Its port keeps `number_of_publishers()` at
    // 1 while it never answers, so the gather must spend its window and refuse
    // to call the empty answer an absence.
    let node = iceoryx2::prelude::NodeBuilder::new()
        .name(&"silent-writer".try_into().expect("node name"))
        .config(&config)
        .create::<CerService>()
        .expect("node");
    let service = node
        .service_builder(&RUN_REGISTRY_SERVICE_NAME.try_into().expect("service name"))
        .publish_subscribe::<[u8]>()
        .subscriber_max_buffer_size(RUN_REGISTRY_QUEUE_DEPTH)
        .max_subscribers(RUN_REGISTRY_MAX_READERS)
        .max_publishers(RUN_REGISTRY_MAX_WRITERS)
        .open_or_create()
        .expect("the shipped provisioning constants must open the production service");
    let _silent = service
        .publisher_builder()
        .create()
        .expect("silent publisher");

    // A SHORT window: the verdict is the assertion, not the wall, and there is
    // nothing to wait for — this writer will never speak.
    let stuck = gather_runs_on_config(&config, Duration::from_millis(120)).expect("gather");
    assert!(
        stuck.records.is_empty(),
        "a silent writer contributes no records"
    );
    assert_eq!(
        stuck.completeness,
        GatherCompleteness::Incomplete {
            live_writers: 1,
            writers_heard: 0,
        },
        "an empty answer with a live unheard writer must be reported UNKNOWN, never as a \
         settled absence — a recorder attaching on this answer would record the wrong machine"
    );
    assert!(
        !stuck.completeness.is_settled(),
        "is_settled() is the one question consumers ask; it must be false here"
    );
}
