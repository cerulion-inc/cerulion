// SPDX-License-Identifier: AGPL-3.0-only
//! The vizd telemetry module without a network: event specs, properties and
//! the heartbeat thread's cadence and prompt stop.

use std::sync::mpsc;
use std::time::{Duration, Instant};

use cerulion_telemetry::{guard, Value};
use cerulion_vizd::telemetry::{
    common, heartbeat_props, started_props, Heartbeat, HEARTBEAT_INTERVAL, VIZD_HEARTBEAT,
    VIZD_STARTED,
};

#[test]
fn heartbeat_interval_is_fifteen_minutes() {
    assert_eq!(HEARTBEAT_INTERVAL, Duration::from_secs(900));
}

#[test]
fn event_names_and_common_values_pass_the_guard() {
    for spec in [VIZD_STARTED, VIZD_HEARTBEAT] {
        guard::check_event_name(spec.name).expect(spec.name);
    }
    let c = common();
    assert_eq!(c.surface, "vizd");
    for value in [&c.surface, &c.env, &c.app_version] {
        guard::check_str(value).expect(value);
    }
}

#[test]
fn properties_stay_inside_their_allowlists_and_pass_the_guard() {
    for (spec, props) in [
        (VIZD_STARTED, started_props()),
        (VIZD_HEARTBEAT, heartbeat_props(3 * HEARTBEAT_INTERVAL)),
    ] {
        let (kept, dropped) = guard::filter(props.clone(), spec.allowlist);
        assert!(dropped.is_empty(), "{}: {dropped:?}", spec.name);
        assert_eq!(kept, props, "{}", spec.name);
    }
}

#[test]
fn uptime_is_whole_minutes_of_elapsed_time() {
    let props = heartbeat_props(4 * HEARTBEAT_INTERVAL);
    assert_eq!(props, vec![("uptime_minutes".to_string(), Value::Int(60))]);
    assert_eq!(
        heartbeat_props(Duration::from_secs(179))[0].1,
        Value::Int(2)
    );
    assert_eq!(
        heartbeat_props(Duration::from_secs(180))[0].1,
        Value::Int(3)
    );
    assert_eq!(
        heartbeat_props(Duration::MAX)[0].1,
        Value::Int((u64::MAX / 60) as i64)
    );
}

#[test]
fn heartbeat_ticks_in_order_and_stops_promptly_on_drop() {
    let (tx, rx) = mpsc::channel();
    let beat = Heartbeat::spawn(Duration::from_millis(20), move |n| {
        let _ = tx.send(n);
    });
    let seen: Vec<u64> = (0..3)
        .map(|_| rx.recv_timeout(Duration::from_secs(5)).expect("tick"))
        .collect();
    assert_eq!(seen, vec![1, 2, 3]);
    drop(beat);
    while rx.try_recv().is_ok() {}
    assert!(
        rx.recv_timeout(Duration::from_millis(100)).is_err(),
        "no tick after drop"
    );
}

#[test]
fn dropping_a_long_interval_heartbeat_does_not_wait_for_the_interval() {
    let beat = Heartbeat::spawn(Duration::from_secs(3600), |_| {});
    let start = Instant::now();
    drop(beat);
    assert!(start.elapsed() < Duration::from_secs(5));
}
