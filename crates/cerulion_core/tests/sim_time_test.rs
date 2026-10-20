// SPDX-License-Identifier: AGPL-3.0-only
//! The `sim_time:` graph YAML switch and the rules the graph clock follows
//! `/clock` by, pinned at the public seams: `parse_graph` / `validate_graph`
//! for the YAML key, and the pure decision functions of
//! `cerulion_core::graph::sim_time` for the clock-sample policy.
//!
//! Every expected value below is hand-written: the YAML a graph author would
//! type, the error text they must read, and the nanosecond arithmetic of each
//! clock-sample verdict. Nothing is compared against a second run of the code.

use cerulion_core::graph::sim_time::{
    clock_sample_ns, judge_sim_clock_sample, SimClockFeed, SimClockVerdict, SIM_CLOCK_TOPIC,
    SIM_TIME_KEY,
};
use cerulion_core::graph::{parse_graph, validate_graph};

/// A one-node graph, with `extra` appended at the top level.
fn graph_yaml(extra: &str) -> String {
    format!(
        r#"
prefix: sim
nodes:
  - id: ticker
    type: ticker
    outputs:
      - name: out
        schema: std_msgs/Int32
{extra}"#
    )
}

// ---------------------------------------------------------------------------
// The YAML key
// ---------------------------------------------------------------------------

#[test]
fn sim_time_is_off_by_default_and_an_absent_key_stays_absent_on_rewrite() {
    let config = parse_graph(&graph_yaml("")).expect("a graph without the key parses");
    assert!(!config.sim_time, "the default is the real clock");
    let rewritten = serde_yaml::to_string(&config).expect("serialize");
    assert!(
        !rewritten.contains(SIM_TIME_KEY),
        "an author who never wrote `sim_time:` must not find one after a rewrite:\n{rewritten}"
    );
}

#[test]
fn sim_time_true_parses_and_round_trips() {
    let config = parse_graph(&graph_yaml("sim_time: true\n")).expect("parses");
    assert!(config.sim_time);
    let rewritten = serde_yaml::to_string(&config).expect("serialize");
    assert!(
        rewritten.contains("sim_time: true"),
        "the switch survives a rewrite:\n{rewritten}"
    );
    let again = parse_graph(&rewritten).expect("the rewrite parses back");
    assert!(again.sim_time);
    // An explicit `false` is the default spelled out: it parses, and it is the
    // real clock.
    let off = parse_graph(&graph_yaml("sim_time: false\n")).expect("parses");
    assert!(!off.sim_time);
}

#[test]
fn a_misspelled_sim_time_key_is_a_loud_parse_error_that_names_the_real_key() {
    let err = parse_graph(&graph_yaml("sim_tme: true\n"))
        .expect_err("graph YAML denies unknown fields")
        .to_string();
    assert!(
        err.contains("sim_tme"),
        "the error names the offending key: {err}"
    );
    assert!(
        err.contains("sim_time"),
        "the error lists the accepted key the author meant: {err}"
    );
}

#[test]
fn sim_time_takes_only_a_boolean() {
    let err = parse_graph(&graph_yaml("sim_time: 1\n"))
        .expect_err("a number is not a switch")
        .to_string();
    assert!(
        err.contains("expected a boolean"),
        "the error says what the key takes: {err}"
    );
}

#[test]
fn sim_time_refuses_a_second_clock_source_at_load() {
    // `/clock` opted into several publishers is more than one clock source.
    let yaml = graph_yaml(&format!(
        "sim_time: true\nmulti_publisher_topics:\n  - {SIM_CLOCK_TOPIC}\n"
    ));
    let err = validate_graph(&parse_graph(&yaml).expect("parses"))
        .expect_err("two clock sources are refused at load")
        .to_string();
    assert!(
        err.contains("one clock source") && err.contains(SIM_CLOCK_TOPIC),
        "the refusal states the rule and names the topic: {err}"
    );
    assert!(
        err.contains("sim_time") && err.contains("multi_publisher_topics"),
        "the refusal names both keys so the author can pick which to change: {err}"
    );

    // With the switch off the listing is today's behaviour: it validates.
    let yaml = graph_yaml(&format!("multi_publisher_topics:\n  - {SIM_CLOCK_TOPIC}\n"));
    validate_graph(&parse_graph(&yaml).expect("parses"))
        .expect("without sim time the listing is an ordinary opt-in");

    // With the switch on and no listing the graph validates: one source.
    validate_graph(&parse_graph(&graph_yaml("sim_time: true\n")).expect("parses"))
        .expect("sim time with one clock source validates");
}

// ---------------------------------------------------------------------------
// `rosgraph_msgs/Clock` sample decoding
// ---------------------------------------------------------------------------

#[test]
fn a_clock_sample_is_seconds_times_a_billion_plus_nanoseconds() {
    // (sec, nanosec) -> ns, hand-computed.
    let cases: &[(i32, u32, Option<u64>)] = &[
        (0, 0, Some(0)),
        (0, 1, Some(1)),
        (1, 500_000_000, Some(1_500_000_000)),
        (12, 345, Some(12_000_000_345)),
        (1_704_067_200, 250_000_000, Some(1_704_067_200_250_000_000)),
        // The largest well-formed stamp: no overflow.
        (i32::MAX, 999_999_999, Some(2_147_483_647_999_999_999)),
        // A negative second count is not a point in sim time.
        (-1, 0, None),
        (i32::MIN, 0, None),
        // A nanosecond field at or past one second is malformed.
        (0, 1_000_000_000, None),
        (5, u32::MAX, None),
    ];
    for &(sec, nanosec, want) in cases {
        assert_eq!(
            clock_sample_ns(sec, nanosec),
            want,
            "clock_sample_ns({sec}, {nanosec})"
        );
    }
}

// ---------------------------------------------------------------------------
// The clock-sample policy: first, forward, equal, backward, large jump
// ---------------------------------------------------------------------------

#[test]
fn the_first_sample_is_the_epoch_whatever_its_value() {
    assert_eq!(
        judge_sim_clock_sample(None, 0),
        SimClockVerdict::First { epoch_ns: 0 }
    );
    assert_eq!(
        judge_sim_clock_sample(None, 1_000_000_000),
        SimClockVerdict::First {
            epoch_ns: 1_000_000_000
        }
    );
}

#[test]
fn a_forward_sample_advances_by_exactly_the_difference() {
    assert_eq!(
        judge_sim_clock_sample(Some(1_000_000_000), 1_010_000_000),
        SimClockVerdict::Forward {
            delta_ns: 10_000_000
        }
    );
    assert_eq!(
        judge_sim_clock_sample(Some(0), 1),
        SimClockVerdict::Forward { delta_ns: 1 }
    );
}

#[test]
fn an_equal_sample_is_accepted_as_no_advance() {
    assert_eq!(judge_sim_clock_sample(Some(42), 42), SimClockVerdict::Equal);
}

#[test]
fn a_backward_sample_is_refused_and_says_by_how_much() {
    assert_eq!(
        judge_sim_clock_sample(Some(1_000_000_000), 999_999_999),
        SimClockVerdict::Backward { by_ns: 1 }
    );
    assert_eq!(
        judge_sim_clock_sample(Some(5_000_000_000), 0),
        SimClockVerdict::Backward {
            by_ns: 5_000_000_000
        }
    );
}

#[test]
fn a_large_forward_jump_is_a_forward_sample_with_no_overflow() {
    // A simulator that seeks far ahead: the full u64 range is representable.
    assert_eq!(
        judge_sim_clock_sample(Some(5), u64::MAX),
        SimClockVerdict::Forward {
            delta_ns: u64::MAX - 5
        }
    );
    // The largest well-formed `rosgraph_msgs/Clock` value after an epoch of 0.
    assert_eq!(
        judge_sim_clock_sample(Some(0), 2_147_483_647_999_999_999),
        SimClockVerdict::Forward {
            delta_ns: 2_147_483_647_999_999_999
        }
    );
}

#[test]
fn the_feed_stores_accepted_samples_and_never_a_refused_one() {
    let mut feed = SimClockFeed::new();
    assert!(!feed.is_valid(), "no sample yet: time is not valid");
    assert_eq!(feed.last_ns(), None);

    assert_eq!(feed.offer(100), SimClockVerdict::First { epoch_ns: 100 });
    assert!(feed.is_valid());
    assert_eq!(feed.last_ns(), Some(100));

    assert_eq!(feed.offer(100), SimClockVerdict::Equal);
    assert_eq!(feed.offer(150), SimClockVerdict::Forward { delta_ns: 50 });
    assert_eq!(feed.last_ns(), Some(150));

    // Backwards: refused, counted, and the last ACCEPTED sample stands.
    assert_eq!(feed.offer(120), SimClockVerdict::Backward { by_ns: 30 });
    assert_eq!(
        feed.last_ns(),
        Some(150),
        "a refused sample is never stored"
    );
    assert_eq!(feed.refused(), 1);

    // After a refusal the next sample is judged against the last ACCEPTED one.
    assert_eq!(feed.offer(150), SimClockVerdict::Equal);
    assert_eq!(feed.offer(160), SimClockVerdict::Forward { delta_ns: 10 });
    assert_eq!(feed.refused(), 1, "accepted samples do not touch the count");
}

#[test]
fn two_feeds_given_the_same_samples_give_the_same_verdicts() {
    // Determinism: the policy is a pure function of the sample sequence.
    let samples = [7u64, 7, 9, 3, 9, 10, 1_000, 999, 1_000];
    let run = || {
        let mut feed = SimClockFeed::new();
        samples.iter().map(|&s| feed.offer(s)).collect::<Vec<_>>()
    };
    let a = run();
    let b = run();
    assert_eq!(a, b);
    assert_eq!(
        a,
        vec![
            SimClockVerdict::First { epoch_ns: 7 },
            SimClockVerdict::Equal,
            SimClockVerdict::Forward { delta_ns: 2 },
            SimClockVerdict::Backward { by_ns: 6 },
            SimClockVerdict::Equal,
            SimClockVerdict::Forward { delta_ns: 1 },
            SimClockVerdict::Forward { delta_ns: 990 },
            SimClockVerdict::Backward { by_ns: 1 },
            SimClockVerdict::Equal,
        ]
    );
}
