// SPDX-License-Identifier: AGPL-3.0-only
//! The CLI's domain events: one per finished workflow (a graph run, a node
//! build, a recording, a replay, a resimulation, a connect session, a
//! pairing) and one when `ros2 attach` starts its bridge graph. Every
//! property is a flag, a count or a coarse bucket; none names anything the
//! user created. The public list is `docs/telemetry.md`.

use std::time::Duration;

use cerulion_telemetry::{EventSpec, Props};

use crate::telemetry::duration_bucket;

pub const GRAPH_RUN_STARTED: EventSpec = EventSpec {
    name: "graph_run_started",
    allowlist: &["is_single_process"],
};
pub const GRAPH_RUN_COMPLETED: EventSpec = EventSpec {
    name: "graph_run_completed",
    allowlist: &["duration_bucket", "is_success"],
};
pub const NODE_BUILD_COMPLETED: EventSpec = EventSpec {
    name: "node_build_completed",
    allowlist: &["duration_bucket", "is_success", "is_release"],
};
pub const BAG_RECORD_COMPLETED: EventSpec = EventSpec {
    name: "bag_record_completed",
    allowlist: &["duration_bucket", "size_bucket", "topic_count"],
};
pub const BAG_RECORD_FAILED: EventSpec = EventSpec {
    name: "bag_record_failed",
    allowlist: &["duration_bucket"],
};
pub const BAG_REPLAY_COMPLETED: EventSpec = EventSpec {
    name: "bag_replay_completed",
    allowlist: &["duration_bucket", "is_success"],
};
pub const RESIM_COMPLETED: EventSpec = EventSpec {
    name: "resim_completed",
    allowlist: &["duration_bucket", "exit_code", "is_divergent"],
};
pub const ROS2_BRIDGE_STARTED: EventSpec = EventSpec {
    name: "ros2_bridge_started",
    allowlist: &[],
};
pub const CONNECT_SESSION_COMPLETED: EventSpec = EventSpec {
    name: "connect_session_completed",
    allowlist: &["duration_bucket", "exit_code"],
};
pub const PAIR_COMPLETED: EventSpec = EventSpec {
    name: "pair_completed",
    allowlist: &["is_success"],
};

/// Every domain event, for the contract tests.
#[cfg(test)]
pub const ALL: &[EventSpec] = &[
    GRAPH_RUN_STARTED,
    GRAPH_RUN_COMPLETED,
    NODE_BUILD_COMPLETED,
    BAG_RECORD_COMPLETED,
    BAG_RECORD_FAILED,
    BAG_REPLAY_COMPLETED,
    RESIM_COMPLETED,
    ROS2_BRIDGE_STARTED,
    CONNECT_SESSION_COMPLETED,
    PAIR_COMPLETED,
];

fn bucket(elapsed: Duration) -> (String, cerulion_telemetry::Value) {
    ("duration_bucket".into(), duration_bucket(elapsed).into())
}

fn flag(key: &str, value: bool) -> (String, cerulion_telemetry::Value) {
    (key.into(), value.into())
}

pub fn graph_run_started(is_single_process: bool) -> Props {
    vec![flag("is_single_process", is_single_process)]
}

pub fn graph_run_completed(elapsed: Duration, is_success: bool) -> Props {
    vec![bucket(elapsed), flag("is_success", is_success)]
}

pub fn node_build_completed(elapsed: Duration, is_success: bool, is_release: bool) -> Props {
    vec![
        bucket(elapsed),
        flag("is_success", is_success),
        flag("is_release", is_release),
    ]
}

/// `topic_count` is the number of topics that recorded at least one message.
pub fn bag_record_completed(elapsed: Duration, bytes: u64, topic_count: usize) -> Props {
    vec![
        bucket(elapsed),
        ("size_bucket".into(), size_bucket(bytes).into()),
        (
            "topic_count".into(),
            i64::try_from(topic_count).unwrap_or(i64::MAX).into(),
        ),
    ]
}

pub fn bag_record_failed(elapsed: Duration) -> Props {
    vec![bucket(elapsed)]
}

pub fn bag_replay_completed(elapsed: Duration, is_success: bool) -> Props {
    vec![bucket(elapsed), flag("is_success", is_success)]
}

pub fn resim_completed(elapsed: Duration, exit_code: u8) -> Props {
    vec![
        bucket(elapsed),
        ("exit_code".into(), i64::from(exit_code).into()),
        flag(
            "is_divergent",
            exit_code == cerulion_cli_engine::resim_cmd::EXIT_TRACE_DIVERGENCE,
        ),
    ]
}

pub fn connect_session_completed(elapsed: Duration, exit_code: i64) -> Props {
    vec![bucket(elapsed), ("exit_code".into(), exit_code.into())]
}

pub fn pair_completed(is_success: bool) -> Props {
    vec![flag("is_success", is_success)]
}

/// A coarse bag size.
pub fn size_bucket(bytes: u64) -> &'static str {
    const MB: u64 = 1 << 20;
    match bytes {
        b if b < MB => "lt_1mb",
        b if b < 10 * MB => "1mb_10mb",
        b if b < 100 * MB => "10mb_100mb",
        b if b < 1 << 30 => "100mb_1gb",
        _ => "gte_1gb",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_telemetry::guard;

    fn assert_exact(spec: EventSpec, props: Props) {
        let keys: Vec<&str> = props.iter().map(|(k, _)| k.as_str()).collect();
        assert_eq!(keys, spec.allowlist, "{}", spec.name);
        let (_, dropped) = guard::filter(props, spec.allowlist);
        assert!(dropped.is_empty(), "{}: {dropped:?}", spec.name);
    }

    #[test]
    fn every_builder_sends_exactly_its_allowlist() {
        let d = Duration::from_millis(1500);
        assert_exact(GRAPH_RUN_STARTED, graph_run_started(true));
        assert_exact(GRAPH_RUN_COMPLETED, graph_run_completed(d, false));
        assert_exact(NODE_BUILD_COMPLETED, node_build_completed(d, true, false));
        assert_exact(BAG_RECORD_COMPLETED, bag_record_completed(d, 5 << 20, 3));
        assert_exact(BAG_RECORD_FAILED, bag_record_failed(d));
        assert_exact(BAG_REPLAY_COMPLETED, bag_replay_completed(d, true));
        assert_exact(RESIM_COMPLETED, resim_completed(d, 6));
        assert_exact(ROS2_BRIDGE_STARTED, Vec::new());
        assert_exact(CONNECT_SESSION_COMPLETED, connect_session_completed(d, 0));
        assert_exact(PAIR_COMPLETED, pair_completed(false));
        for spec in ALL {
            guard::check_event_name(spec.name).expect(spec.name);
        }
    }

    #[test]
    fn size_buckets_and_divergence_are_pinned() {
        assert_eq!(size_bucket(0), "lt_1mb");
        assert_eq!(size_bucket((1 << 20) - 1), "lt_1mb");
        assert_eq!(size_bucket(1 << 20), "1mb_10mb");
        assert_eq!(size_bucket(10 << 20), "10mb_100mb");
        assert_eq!(size_bucket(100 << 20), "100mb_1gb");
        assert_eq!(size_bucket(1 << 30), "gte_1gb");
        assert_eq!(size_bucket(u64::MAX), "gte_1gb");
        let divergent = |code| resim_completed(Duration::ZERO, code)[2].1 == true.into();
        assert!(divergent(6));
        for code in [0, 1, 2, 3, 7] {
            assert!(!divergent(code));
        }
    }
}
