// SPDX-License-Identifier: AGPL-3.0-only
//! The throttle and once log macros past the per-instance table: a binary
//! with more than `LOG_SITE_SLOTS` (64) call sites.
//!
//! Slot numbers are counted per linked binary, so this file is its own binary
//! with ONE test and no other call site: its 65 `warn_once!` sites take slots
//! 0 to 63 and the 65th overflows, in run order. Hand oracle, over two probe
//! instances `a` then `b`, each running all 65 sites once:
//!
//! - `a` prints all 65 lines;
//! - `b` prints sites 0 to 63 (their state is per instance) and NOT site 64,
//!   whose state lives in the site's `static` and is shared with `a`;
//! - the overflow `warn` prints exactly once, at site 64's first run, and
//!   names this file.
//!
//! ```bash
//! cargo test -p cerulion_core --test log_sites_overflow_test
//! ```

use std::sync::{Arc, Mutex};

use cerulion_core::graph::log_sites::LOG_SITE_SLOTS;
use cerulion_core::graph::node::CerNodeRuntimeFields;
use cerulion_core::warn_once;
use tracing::field::{Field, Visit};
use tracing::{Event, Level, Subscriber};
use tracing_subscriber::layer::{Context, Layer};
use tracing_subscriber::prelude::*;

/// `(message, fields, level)` of one event.
type Line = (String, Vec<String>, Level);

#[derive(Default)]
struct LineVisitor(String, Vec<String>);

impl Visit for LineVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            self.0 = format!("{value:?}");
        } else {
            self.1.push(format!("{}={value:?}", field.name()));
        }
    }
}

#[derive(Clone, Default)]
struct Capture(Arc<Mutex<Vec<Line>>>);

impl<S: Subscriber> Layer<S> for Capture {
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        let mut v = LineVisitor::default();
        event.record(&mut v);
        self.0
            .lock()
            .unwrap()
            .push((v.0, v.1, *event.metadata().level()));
    }
}

struct Probe {
    __cer_rt: CerNodeRuntimeFields,
}

/// One `warn_once!` call site per literal: each expansion declares its own
/// `static` site. Evaluates to `line!()`, which inside a macro expansion is
/// the line of the outermost invocation: the line every one of these sites
/// reports.
macro_rules! sites {
    ($p:expr; $($k:literal)*) => {{ $( warn_once!($p, k = $k, "site"); )* line!() }};
}

/// Runs all 65 sites on `p` and returns the line they report.
fn run_all_sites(p: &Probe) -> u32 {
    sites!(p;
        0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31
        32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60
        61 62 63 64
    )
}

fn site_keys(lines: &[Line]) -> Vec<u32> {
    lines
        .iter()
        .filter(|(m, _, _)| m == "site")
        .map(|(_, f, _)| f[0].strip_prefix("k=").unwrap().parse().unwrap())
        .collect()
}

#[test]
fn sites_past_the_table_share_state_across_instances_and_say_so_once() {
    assert_eq!(LOG_SITE_SLOTS, 64, "the oracle below is written for 64");
    let a = Probe {
        __cer_rt: CerNodeRuntimeFields::default(),
    };
    let b = Probe {
        __cer_rt: CerNodeRuntimeFields::default(),
    };
    let run = |p: &Probe| {
        let cap = Capture::default();
        let sub = tracing_subscriber::registry().with(cap.clone());
        let line = tracing::subscriber::with_default(sub, || run_all_sites(p));
        let lines = cap.0.lock().unwrap().clone();
        (lines, line)
    };
    let (on_a, sites_line) = run(&a);
    let (on_b, _) = run(&b);

    assert_eq!(site_keys(&on_a), (0..=64).collect::<Vec<u32>>());
    assert_eq!(
        site_keys(&on_b),
        (0..=63).collect::<Vec<u32>>(),
        "sites 0 to 63 are per instance; site 64 shares a's state"
    );

    let overflow = |lines: &[Line]| -> Vec<Line> {
        lines
            .iter()
            .filter(|(m, _, _)| m.contains("per-instance table"))
            .cloned()
            .collect()
    };
    let warned_a = overflow(&on_a);
    assert_eq!(warned_a.len(), 1, "one overflow warn: {on_a:?}");
    assert!(overflow(&on_b).is_empty(), "never a second one: {on_b:?}");
    // It comes right before site 64's own line, and names this file.
    let at = on_a.iter().position(|l| l == &warned_a[0]).unwrap();
    assert_eq!(on_a[at + 1].1, vec!["k=64"]);
    assert_eq!(warned_a[0].2, Level::WARN, "the overflow notice is a warn");
    assert!(
        warned_a[0]
            .1
            .iter()
            .any(|f| f.starts_with("site=") && f.contains("log_sites_overflow_test.rs")),
        "the warn names the site: {:?}",
        warned_a[0]
    );
    let want_line = format!("line={sites_line}");
    assert!(
        warned_a[0].1.iter().any(|f| f == &want_line),
        "the warn names the site's line, {want_line}: {:?}",
        warned_a[0]
    );
    assert!(warned_a[0]
        .1
        .iter()
        .any(|f| f == "module=log_sites_overflow_test"));
}
