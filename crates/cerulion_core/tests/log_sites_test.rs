// SPDX-License-Identifier: AGPL-3.0-only
//! The throttled and once-only log macros (`warn_throttle!`, `warn_once!` and
//! their siblings), driven at two seams:
//!
//! 1. A `Probe` struct holding the same hidden runtime field a
//!    `#[cerulion_node]` struct carries, over a `VirtualClock` the test sets
//!    by hand. This reaches the cases a scheduler never produces on demand:
//!    a clock that goes back, a zero period, an exact period boundary.
//! 2. A real macro node in an in-process `GraphRuntime`, stepped on a
//!    `VirtualClock`: the throttle reads the node's own clock, a once site
//!    emits once per node instance, and the same site in a loop is one site.
//!
//! Every emitted event is captured by a thread-local subscriber that records
//! its level, target, message and fields.
//!
//! ```bash
//! cargo test -p cerulion_core --test log_sites_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime};

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::graph::node::{CerNodeRuntimeFields, NodeEntry};
use cerulion_core::graph::{parse_graph, validate_graph, GraphRuntime};
use cerulion_core::prelude::*;
use cerulion_core::transport::TransportManager;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing::field::{Field, Visit};
use tracing::{Event, Level, Subscriber};
use tracing_subscriber::layer::{Context, Layer};
use tracing_subscriber::prelude::*;

/// One captured event.
#[derive(Debug, Clone, PartialEq)]
struct Line {
    level: Level,
    target: String,
    message: String,
    /// Every non-message field as `name=value`, in call order.
    fields: Vec<String>,
}

#[derive(Default)]
struct LineVisitor {
    message: String,
    fields: Vec<String>,
}

impl Visit for LineVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            self.message = format!("{value:?}");
        } else {
            self.fields.push(format!("{}={value:?}", field.name()));
        }
    }
}

#[derive(Clone, Default)]
struct Capture(Arc<Mutex<Vec<Line>>>);

impl<S: Subscriber> Layer<S> for Capture {
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        let mut v = LineVisitor::default();
        event.record(&mut v);
        self.0.lock().unwrap().push(Line {
            level: *event.metadata().level(),
            target: event.metadata().target().to_string(),
            message: v.message,
            fields: v.fields,
        });
    }
}

impl Capture {
    fn lines(&self) -> Vec<Line> {
        self.0.lock().unwrap().clone()
    }
    fn messages(&self) -> Vec<String> {
        self.lines().into_iter().map(|l| l.message).collect()
    }
}

/// Run `f` with every event on this thread captured.
fn capture<R>(f: impl FnOnce() -> R) -> (R, Capture) {
    let cap = Capture::default();
    let subscriber = tracing_subscriber::registry().with(cap.clone());
    let r = tracing::subscriber::with_default(subscriber, f);
    (r, cap)
}

/// Stands in for a node struct: the macros need only its hidden field.
struct Probe {
    __cer_rt: CerNodeRuntimeFields,
    clock: Arc<VirtualClock>,
}

impl Probe {
    fn at(ns: u64) -> Self {
        let clock = Arc::new(VirtualClock::new());
        clock.set(ns);
        Self {
            __cer_rt: CerNodeRuntimeFields {
                clock: clock.clone(),
                ..Default::default()
            },
            clock,
        }
    }
    fn set(&self, ns: u64) {
        self.clock.set(ns);
    }
}

const MS: u64 = 1_000_000;

/// One throttle site, called at each of `times` in order on a fresh probe.
/// Returns the times at which it emitted.
fn emits_at(times: &[u64], period: Duration) -> Vec<u64> {
    let probe = Probe::at(0);
    let (_, cap) = capture(|| {
        for &t in times {
            probe.set(t);
            warn_throttle!(probe, period, t, "tick");
        }
    });
    cap.lines()
        .iter()
        .map(|l| l.fields[0].strip_prefix("t=").unwrap().parse().unwrap())
        .collect()
}

#[test]
fn a_throttle_emits_first_then_once_per_period_of_graph_time() {
    let period = Duration::from_millis(100);
    // Calls every 30 ms from 5 ms. Hand oracle: 5 (first), then the first
    // call at or after 105 is 125, then at or after 225 is 245.
    let times: Vec<u64> = (0..10).map(|i| (5 + 30 * i) * MS).collect();
    assert_eq!(emits_at(&times, period), vec![5 * MS, 125 * MS, 245 * MS]);
}

#[test]
fn a_call_exactly_at_the_period_end_emits_and_one_ns_before_does_not() {
    let period = Duration::from_millis(100);
    assert_eq!(
        emits_at(&[0, 100 * MS - 1, 100 * MS], period),
        vec![0, 100 * MS]
    );
}

#[test]
fn a_clock_that_goes_back_emits_no_second_line_and_restarts_the_period() {
    let period = Duration::from_millis(100);
    // Emit at 500; back to 450 (suppressed, period now starts at 450); 540
    // is inside it; 550 is its end.
    assert_eq!(
        emits_at(&[500 * MS, 450 * MS, 500 * MS, 540 * MS, 550 * MS], period),
        vec![500 * MS, 550 * MS]
    );
    // Straight back to zero does not panic and does not emit.
    assert_eq!(emits_at(&[u64::MAX, 0], period), vec![u64::MAX]);
}

#[test]
fn a_zero_period_emits_on_every_call_of_a_forward_clock() {
    assert_eq!(emits_at(&[0, 0, 1, 7], Duration::ZERO), vec![0, 0, 1, 7]);
}

#[test]
fn a_once_site_emits_one_line_however_often_it_runs_and_whatever_the_clock_does() {
    let probe = Probe::at(10 * MS);
    let (_, cap) = capture(|| {
        for t in [10, 20, 5, 1_000_000] {
            probe.set(t * MS);
            info_once!(probe, "started");
        }
    });
    assert_eq!(cap.messages(), vec!["started"]);
}

#[test]
fn two_call_sites_keep_separate_state_and_a_site_in_a_loop_is_one_site() {
    let probe = Probe::at(0);
    let period = Duration::from_secs(1);
    let (_, cap) = capture(|| {
        for i in 0..3u32 {
            warn_throttle!(probe, period, i, "site a");
            warn_throttle!(probe, period, i, "site b");
            warn_once!(probe, i, "site c");
        }
    });
    let lines = cap.lines();
    let got: Vec<(&str, &str)> = lines
        .iter()
        .map(|l| (l.message.as_str(), l.fields[0].as_str()))
        .collect();
    assert_eq!(
        got,
        vec![("site a", "i=0"), ("site b", "i=0"), ("site c", "i=0")]
    );
}

#[test]
fn each_probe_instance_keeps_its_own_once_and_throttle_state() {
    let a = Probe::at(0);
    let b = Probe::at(0);
    let (_, cap) = capture(|| {
        for p in [&a, &b, &a, &b] {
            warn_once!(p, "once per instance");
            warn_throttle!(p, Duration::from_secs(1), "throttle per instance");
        }
    });
    assert_eq!(
        cap.messages(),
        vec![
            "once per instance",
            "throttle per instance",
            "once per instance",
            "throttle per instance",
        ]
    );
}

/// One once site and one throttle site, shared by every caller.
fn log_both(p: &Probe) {
    warn_once!(p, "once per instance");
    warn_throttle!(p, Duration::from_secs(1), "throttle per instance");
}

#[test]
fn a_cloned_runtime_is_a_new_instance_with_no_once_line_printed() {
    let a = Probe::at(0);
    let (_, cap) = capture(|| {
        log_both(&a);
        let clone = Probe {
            __cer_rt: a.__cer_rt.clone(),
            clock: a.clock.clone(),
        };
        // The clone did not print these lines, so it prints them now, at the
        // same graph time, inside the original's throttle window.
        log_both(&clone);
        // The original keeps its own state: both suppressed.
        log_both(&a);
    });
    assert_eq!(
        cap.messages(),
        vec![
            "once per instance",
            "throttle per instance",
            "once per instance",
            "throttle per instance",
        ]
    );
}

static EVALS: AtomicU64 = AtomicU64::new(0);

fn counted(v: u64) -> u64 {
    EVALS.fetch_add(1, Ordering::SeqCst);
    v
}

#[test]
fn a_suppressed_call_does_not_evaluate_its_message_arguments() {
    let probe = Probe::at(0);
    let before = EVALS.load(Ordering::SeqCst);
    let (_, cap) = capture(|| {
        for _ in 0..5 {
            warn_throttle!(
                probe,
                Duration::from_secs(1),
                v = counted(1),
                "x {}",
                counted(2)
            );
            info_once!(probe, v = counted(3), "y {}", counted(4));
        }
    });
    // One emit of each: two evaluations each, and none for the eight
    // suppressed calls.
    assert_eq!(EVALS.load(Ordering::SeqCst) - before, 4);
    assert_eq!(cap.messages(), vec!["x 2", "y 4"]);
}

#[test]
fn every_variant_forwards_its_level_target_and_fields_as_the_plain_macro() {
    let probe = Probe::at(0);
    let p = Duration::from_secs(1);
    let (_, cap) = capture(|| {
        tracing::error!(n = 1, "plain");
        error_throttle!(probe, p, n = 1, "plain");
        error_once!(probe, n = 1, "plain");
        tracing::warn!(n = 2, "plain");
        warn_throttle!(probe, p, n = 2, "plain");
        warn_once!(probe, n = 2, "plain");
        tracing::info!(n = 3, "plain");
        info_throttle!(probe, p, n = 3, "plain");
        info_once!(probe, n = 3, "plain");
        tracing::debug!(n = 4, "plain");
        debug_throttle!(probe, p, n = 4, "plain");
        debug_once!(probe, n = 4, "plain");
        tracing::trace!(n = 5, "plain");
        trace_throttle!(probe, p, n = 5, "plain");
        trace_once!(probe, n = 5, "plain");
        // An explicit target is forwarded too.
        tracing::warn!(target: "my_node", n = 6, "plain");
        warn_throttle!(probe, p, target: "my_node", n = 6, "plain");
        warn_once!(probe, target: "my_node", n = 6, "plain");
    });
    let lines = cap.lines();
    assert_eq!(lines.len(), 18, "every call is the first at its site");
    let levels = [
        Level::ERROR,
        Level::WARN,
        Level::INFO,
        Level::DEBUG,
        Level::TRACE,
        Level::WARN,
    ];
    for (group, want_level) in lines.chunks(3).zip(levels) {
        assert_eq!(group[0].level, want_level);
        assert_eq!(group[1], group[0], "throttle matches the plain macro");
        assert_eq!(group[2], group[0], "once matches the plain macro");
    }
    assert_eq!(lines[0].target, "log_sites_test");
    assert_eq!(lines[15].target, "my_node");
    assert_eq!(lines[3].fields, vec!["n=2"]);
}

// ---- A real node in an in-process graph ------------------------------------

/// Logs through a throttle and a once site on every tick. `instance` tells
/// the two instances of the graph apart in the captured lines.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Chatty {
    #[output]
    out: Vector3,
    instance: u64,
}

static NEXT_INSTANCE: AtomicU64 = AtomicU64::new(0);

#[cerulion_node_impl]
impl Chatty {
    fn init(&mut self, _ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.instance = NEXT_INSTANCE.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        let tick_ns = self.now_ns();
        let instance = self.instance;
        warn_throttle!(
            self,
            Duration::from_millis(100),
            instance,
            tick_ns,
            "throttled"
        );
        for _ in 0..3 {
            info_once!(self, instance, "first tick");
        }
        self.out.x = 1.0;
        Ok(())
    }
}

fn build(clock: Arc<VirtualClock>) -> GraphRuntime {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .expect("a clock after the epoch")
        .as_nanos();
    let yaml = format!(
        r#"
name: log_sites_graph
prefix: log_sites_{nanos}
nodes:
  - id: a
    type: chatty
    outputs:
      - name: out
        schema: geometry_msgs/Vector3
  - id: b
    type: chatty
    outputs:
      - name: out
        schema: geometry_msgs/Vector3
"#
    );
    let config = parse_graph(&yaml).expect("parse graph");
    validate_graph(&config).expect("validate graph");
    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("a".to_string(), Box::new(ChattyEntry::new()));
    nodes.insert("b".to_string(), Box::new(ChattyEntry::new()));
    GraphRuntime::build(config, nodes, &mgr, clock).expect("build graph")
}

/// The (instance, tick_ns) of every throttled line, and the instances of
/// every once line.
fn run(steps: u64, jump_after: u64) -> (Vec<(u64, u64)>, Vec<u64>) {
    let clock = Arc::new(VirtualClock::new());
    let mut rt = build(clock.clone());
    let (_, cap) = capture(|| {
        for i in 0..steps {
            if i == jump_after {
                // Graph time jumps 10 s ahead between two steps. (A real
                // pause freezes the clock instead; `log_sites_pause_iox2_test`
                // pins that seam.)
                clock.set(clock.now_ns() + 10_000 * MS);
            }
            rt.step(Duration::from_millis(1));
        }
    });
    rt.shutdown();
    let field = |l: &Line, name: &str| -> u64 {
        let pre = format!("{name}=");
        l.fields
            .iter()
            .find_map(|f| f.strip_prefix(&pre))
            .unwrap()
            .parse()
            .unwrap()
    };
    // Only the node's own lines; the framework logs too.
    let lines: Vec<Line> = cap
        .lines()
        .into_iter()
        .filter(|l| l.message == "throttled" || l.message == "first tick")
        .collect();
    // Number the instances 0 and 1 in this run, whatever earlier runs took.
    let base = lines.iter().map(|l| field(l, "instance")).min().unwrap();
    let throttled = lines
        .iter()
        .filter(|l| l.message == "throttled")
        .map(|l| (field(l, "instance") - base, field(l, "tick_ns")))
        .collect();
    let once = lines
        .iter()
        .filter(|l| l.message == "first tick")
        .map(|l| field(l, "instance") - base)
        .collect();
    (throttled, once)
}

#[test]
#[serial]
fn a_node_throttles_on_its_graph_clock_and_logs_once_per_instance_across_a_graph_time_jump() {
    let (throttled, once) = run(1_000, 500);
    // One once line per instance, the first time each ticks, not again after
    // the jump.
    assert_eq!(once, vec![0, 1]);
    // Per instance, every gap between emitted lines is a whole number of
    // periods of GRAPH time: exactly 100 ms while the node ticks every 1 ms,
    // and the 10 s jump in one step.
    for instance in [0, 1] {
        let times: Vec<u64> = throttled
            .iter()
            .filter(|(i, _)| *i == instance)
            .map(|(_, t)| *t)
            .collect();
        let gaps: Vec<u64> = times.windows(2).map(|w| w[1] - w[0]).collect();
        // 500 steps before the jump emit at 0, 100, .., 400 ms of ticking
        // (5 lines); the first tick after the jump emits; then 4 more in the
        // remaining 500 steps.
        assert_eq!(times.len(), 10, "instance {instance}: {times:?}");
        let mut want = vec![100 * MS; 4];
        want.push(10_100 * MS);
        want.extend([100 * MS; 4]);
        assert_eq!(gaps, want, "instance {instance}");
    }
}

#[test]
#[serial]
fn two_runs_of_the_same_graph_emit_the_same_lines() {
    let first = run(400, 200);
    let second = run(400, 200);
    assert_eq!(first, second);
    assert!(!first.0.is_empty());
}
