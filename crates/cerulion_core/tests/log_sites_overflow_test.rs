// SPDX-License-Identifier: AGPL-3.0-only
//! The throttle and once log macros past the per-instance table: a node
//! instance that runs more than `LOG_SITE_SLOTS` (64) distinct call sites.
//!
//! Each site keys its state by its own stable id, so this file runs 66
//! `warn_once!` sites on one probe instance: the first 64 to run take the 64
//! entries and the 65th and 66th find none. Two sites past the table, not
//! one, so that a flag shared by every site (one error per process, as the
//! first version had) fails the oracle: it would report site 64 and stay
//! silent on site 65. Hand oracle, over two probe instances `a` (run twice)
//! then `b`, each run calling all 66 sites once:
//!
//! - the first run on `a` prints all 66 lines;
//! - the second run on `a` prints only sites 64 and 65: sites 0 to 63 keep
//!   their once state, sites 64 and 65 keep none, so every call to them
//!   prints;
//! - `b` prints all 66 lines: its state is its own, never `a`'s;
//! - the `error` that a site keeps no state prints exactly once PER SITE,
//!   twice in all, each one right before its own site's first line, and
//!   names this file, the line and the module.
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

/// One `warn_once!` call site per literal: all share one file, line and
/// column, and the arguments (`k = N`) give each its own id. Evaluates to
/// `line!()`, which inside a macro expansion is the line of the outermost
/// invocation: the line every one of these sites reports.
macro_rules! sites {
    ($p:expr; $($k:literal)*) => {{ $( warn_once!($p, k = $k, "site"); )* line!() }};
}

/// Runs all 66 sites on `p` and returns the line they report.
fn run_all_sites(p: &Probe) -> u32 {
    sites!(p;
        0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31
        32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60
        61 62 63 64 65
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
fn a_site_past_the_table_keeps_no_state_prints_every_call_and_says_so() {
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
    let (on_a_again, _) = run(&a);
    let (on_b, _) = run(&b);

    assert_eq!(site_keys(&on_a), (0..=65).collect::<Vec<u32>>());
    assert_eq!(
        site_keys(&on_a_again),
        vec![64, 65],
        "sites 0 to 63 keep their once state; sites 64 and 65 keep none and print again"
    );
    assert_eq!(
        site_keys(&on_b),
        (0..=65).collect::<Vec<u32>>(),
        "b prints every line: no site shares a's state"
    );

    // `(position, line)` of every "keeps no state" error, in the order printed.
    let unkept = |lines: &[Line]| -> Vec<(usize, Line)> {
        lines
            .iter()
            .enumerate()
            .filter(|(_, (m, _, _))| m.contains("than its table holds"))
            .map(|(at, l)| (at, l.clone()))
            .collect()
    };
    let said = unkept(&on_a);
    assert_eq!(
        said.len(),
        2,
        "one error per unkept site, two unkept sites: {on_a:?}"
    );
    assert!(unkept(&on_a_again).is_empty(), "said once: {on_a_again:?}");
    assert!(unkept(&on_b).is_empty(), "said once: {on_b:?}");
    let want_line = format!("line={sites_line}");
    for (n, (at, error)) in said.iter().enumerate() {
        // The error for site 64 + n comes right before that site's own first
        // line, and names the site.
        let site = format!("k={}", 64 + n);
        assert_eq!(on_a[at + 1].1, vec![site.clone()], "the error for {site}");
        assert_eq!(error.2, Level::ERROR, "a site with no state is an error");
        assert!(
            error
                .1
                .iter()
                .any(|f| f.starts_with("site=") && f.contains("log_sites_overflow_test.rs")),
            "the error names the site: {error:?}"
        );
        assert!(
            error.1.iter().any(|f| f == &want_line),
            "the error names the site's line, {want_line}: {error:?}"
        );
        assert!(error
            .1
            .iter()
            .any(|f| f == "module=log_sites_overflow_test"));
    }
}
