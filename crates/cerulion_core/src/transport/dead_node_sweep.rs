// SPDX-License-Identifier: AGPL-3.0-only
//! The BOUNDED iceoryx2 dead-node sweep, and the ONE copy of its policy.
//!
//! # Why this is bounded, and why the policy lives here
//!
//! iceoryx2's dead-node reaper is `O(dead nodes)` times the cost of removing
//! ONE dead node's stale resources, and that inner cost is not constant: it
//! walks the node's services and, per service, lists the shared-memory
//! namespace (`stale_resource_cleanup::connections` → `Connection::list_cfg`
//! → `SharedMemory::list` → the PAL's `shm_list()`).
//!
//! On **Linux** `shm_list()` reads `/dev/shm`, which contains shared memory and
//! nothing else. On **macOS** there is no such filesystem, so
//! `iceoryx2-pal-posix` keeps one `<name>.shm_state` file per segment in
//! `TEMP_DIRECTORY` — literally `/tmp/` — and `shm_list()` is a full `readdir`
//! of `/tmp`. A segment whose owner was SIGKILLed leaks its `.shm_state` file,
//! so on a long-lived host that directory grows without bound and every listing
//! grows with it.
//!
//! MEASURED on a development machine (2026-08-22): `/tmp` held **73,290 entries,
//! 72,707 of them `.shm_state`**, and one full scan took **393 s** under load
//! (13 s quiet). After removing the stale files: **898 entries, 0.00 s**. A
//! `cerulion_cli` test binary went from `1 of 8` arms passing in 668 s to
//! **8 of 8 in 51 s**, the failures reporting `bagd never created the bag` with
//! EMPTY stdout AND stderr — a supervisor that had not reached its first log
//! line because it was still inside this sweep.
//!
//! So an unbounded sweep at startup can silently delay step 0 by minutes on a
//! host whose only sin is having run a lot of graphs. That is ROBOT-facing:
//! `/tmp` on a robot accumulates the same way, and a crash-looping node is
//! exactly the shape that fills it.
//!
//! **There are TWO startup sweeps in this system**, and bounding one alone
//! leaves the stall exactly where it was — a run that clears the CLI's sweep
//! walks straight into the transport's. `cerulion_cli_engine::ipc_cleanup`
//! DELEGATES to the functions here rather than carrying its own copy (the
//! rule being that two implementations of one policy is how they drift), so the
//! budget, the decision and the report have one definition.
//!
//! # Scope of the bound
//!
//! The budget is consulted BEFORE each dead node, and
//! `blocking_remove_stale_resources` has no cancellation, so the guarantee is
//! `budget + one attempt` — it stops a run from paying a pathological per-node
//! cost `N` times, it cannot make one attempt cheap. Nothing is lost by
//! deferring: a dead node is still dead next time, and the next run (or
//! `cerulion clean`, which is deliberately UNBOUNDED) picks it up.

use core::time::Duration;
use std::time::Instant;

use iceoryx2::config::Config;
use iceoryx2::node::{Node, NodeState};
use iceoryx2::prelude::CallbackProgression;

use super::CerService;

/// Wall budget for an advisory dead-node sweep on a STARTUP path.
///
/// Hygiene the operator did not ask for must not measurably delay the graph
/// they did. Two seconds is a bring-up cost nobody notices; the sweep it bounds
/// was MEASURED silently eating MINUTES before the first log line on a desk
/// with accumulated `/tmp` residue.
pub const STARTUP_DEAD_NODE_SWEEP_BUDGET: Duration = Duration::from_secs(2);

/// What a BOUNDED sweep did, and what it deliberately left standing.
///
/// `deferred` is the disclosure half: without it a caller cannot tell "this host is
/// clean" from "this host has 400 dead nodes and I stopped after two", and those
/// need different operator actions. It is a COUNT rather than a bool for the
/// same reason — "some were left" and "most were left" differ.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BoundedCleanup {
    /// Dead nodes whose stale resources were removed.
    pub cleanups: u64,
    /// Dead nodes that were attempted and refused (permissions, contention, …).
    pub failed_cleanups: u64,
    /// Dead nodes SEEN after the budget was spent, and therefore never
    /// attempted. They remain for the next run or for `cerulion clean`.
    pub deferred: u64,
}

impl BoundedCleanup {
    /// Whether the budget ran out with dead nodes still on the host.
    ///
    /// Derived rather than stored: a second field could disagree with the count
    /// it summarises, and there is exactly one condition an operator can act on.
    #[must_use]
    pub fn budget_exhausted(&self) -> bool {
        self.deferred > 0
    }
}

/// What the sweep should do with the next dead node it meets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweepStep {
    /// Budget remains — attempt the removal.
    Attempt,
    /// Budget spent — count it and leave it for the next run.
    Defer,
}

/// PURE: [`SweepStep`] from the wall already spent against the budget.
///
/// The comparison is `>=`, not `>`, so a ZERO budget defers EVERYTHING. That is
/// not a degenerate case to tolerate but the seam the tests drive: it makes the
/// `Defer` arm reachable on a HEALTHY host, where the alternative would be
/// waiting for a machine sick enough to exhaust a real budget.
#[must_use]
pub fn sweep_step(elapsed: Duration, budget: Duration) -> SweepStep {
    if elapsed >= budget {
        SweepStep::Defer
    } else {
        SweepStep::Attempt
    }
}

/// PURE-ish: drive a sequence of dead-node removals under a wall budget.
///
/// Generic over the CLOCK and the REMOVAL, and that is the whole reason it
/// exists as a separate function. On a HEALTHY host the real sweep meets no dead
/// nodes at all, so a test written against [`cleanup_dead_nodes_bounded`]
/// asserts `(0, 0)` whether or not the budget is honoured — a variant that
/// computes the budget and then ignores it passes such a test. Creating a
/// genuine dead node means spawning a process that opens an iceoryx2 node and
/// SIGKILLing it, in the SHARED namespace every other test on the machine is
/// using. This seam makes the policy observable without either.
///
/// The budget is consulted BEFORE each removal and the walk CONTINUES past it,
/// counting rather than attempting — so `deferred` is a true count instead of a
/// lower bound. That is affordable because the expensive half is the per-node
/// REMOVAL (which lists the whole shared-memory namespace), which is exactly
/// what the budget stops.
pub fn run_budgeted_sweep<T>(
    items: impl IntoIterator<Item = T>,
    budget: Duration,
    mut elapsed: impl FnMut() -> Duration,
    mut remove: impl FnMut(T) -> Result<(), ()>,
) -> BoundedCleanup {
    let mut acc = BoundedCleanup::default();
    for item in items {
        match sweep_step(elapsed(), budget) {
            SweepStep::Attempt => match remove(item) {
                Ok(()) => acc.cleanups += 1,
                Err(()) => acc.failed_cleanups += 1,
            },
            SweepStep::Defer => acc.deferred += 1,
        }
    }
    acc
}

/// Walk iceoryx2's node registry and remove dead nodes' stale resources, giving
/// up on the REMAINDER once `budget` of wall has been spent.
///
/// See this module's header for the measurement that made it necessary and for
/// the exact strength of the bound (`budget + one attempt`).
///
/// Panic containment: `Node::list` `unwrap()`s each registry filename, so a
/// stray or foreign file in the shared namespace must not take a best-effort
/// hygiene sweep — and the whole startup path — down with it. `AssertUnwindSafe`
/// is sound because on unwind nothing from inside the closure is observed:
/// everything is discarded and a zeroed report returned.
/// The iceoryx2 `Config` a dead-node sweep must mint its NODE from.
///
/// iceoryx2 carries `try_cleanup_dead_nodes` on `&Node`, so a sweep of a
/// namespace needs a node in it, and a node created with iceoryx2's defaults
/// sweeps that namespace ITSELF, on creation and again on destruction. Two
/// things go wrong if it does.
///
/// The first is a reporting defect: the implicit sweep does the work, so the
/// explicit call that follows finds nothing and reports `0` cleaned and `0`
/// refused for a namespace it just emptied. Anything built on those counts (the
/// classified report `cerulion clean` prints, the convergence check that
/// decides whether a second pass is needed) is then describing a sweep that did
/// not happen.
///
/// The second is the reason `TransportManager` disables the same three flags on
/// every node it builds: iceoryx2's liveness probe can judge a LIVE node dead
/// when the probing linkage unit is not the one that created it, so an implicit
/// reap can remove a running process's services. A sweep must be one deliberate
/// call whose result is reported, never a side effect of opening a namespace.
///
/// Both sweep entry points in the CLI engine and the transport's own liveliness
/// cleaner build their node from this.
#[must_use]
pub fn sweep_node_config(config: &Config) -> Config {
    crate::transport::disable_auto_dead_node_cleanup(config.clone())
}

pub fn cleanup_dead_nodes_bounded(config: &Config, budget: Duration) -> BoundedCleanup {
    let started = Instant::now();
    let swept = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        // Collect FIRST, sweep second. The walk reads only the node-registry
        // directory, which is the cheap half; separating it is what lets
        // `run_budgeted_sweep` be driven by hand oracles.
        // hot-path-alloc-ok: startup-only dead-node sweep — runs once at process
        // boot before any graph steps, never on the publish/receive path.
        let mut dead = Vec::new();
        let listed = Node::<CerService>::list(config, |state| {
            if let NodeState::Dead(view) = state {
                dead.push(view);
            }
            CallbackProgression::Continue
        });
        let acc = run_budgeted_sweep(
            dead,
            budget,
            || started.elapsed(),
            |view| {
                // `Duration::ZERO` is the per-node CONTENTION wait, NOT our
                // budget: it is what `try_cleanup_dead_nodes` passes, and it
                // means "if another process is already cleaning this node, skip
                // it" rather than "wait for it".
                view.blocking_remove_stale_resources(Duration::ZERO)
                    .map_err(|_| ())
            },
        );
        (acc, listed)
    }));
    match swept {
        Ok((acc, Ok(()))) => acc,
        Ok((acc, Err(e))) => {
            tracing::warn!(
                error = ?e,
                "iceoryx2 dead-node sweep could not finish listing the shared namespace — the \
                 advisory sweep is partial; the run continues"
            );
            acc
        }
        Err(payload) => {
            let panic_msg = payload
                .downcast_ref::<&str>()
                .map(|s| (*s).to_string()) // hot-path-alloc-ok: panic-payload extraction on the sweep's contained-unwind error path
                .or_else(|| payload.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "<non-string panic payload>".to_string()); // hot-path-alloc-ok: same contained-unwind error path — cold by construction
            tracing::warn!(
                panic = %panic_msg,
                "iceoryx2 dead-node sweep panicked while listing the shared namespace (likely a \
                 stray/foreign file in the iceoryx2 root) — skipping the advisory sweep; the run \
                 continues"
            );
            BoundedCleanup::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Hand oracle for [`sweep_step`] at BOTH sides of the boundary.
    ///
    /// The `elapsed == budget` row is what a `>` comparison in place of `>=`
    /// would flip, and the ZERO-budget row is what the behavioural deferral relies on.
    #[test]
    fn sweep_step_defers_exactly_at_and_past_the_budget() {
        let ms = Duration::from_millis;
        let cases = [
            (ms(0), ms(0), SweepStep::Defer),
            (ms(0), ms(1), SweepStep::Attempt),
            (ms(999), ms(1000), SweepStep::Attempt),
            (ms(1000), ms(1000), SweepStep::Defer),
            (ms(1001), ms(1000), SweepStep::Defer),
            (Duration::from_secs(3600), ms(1), SweepStep::Defer),
        ];
        for (elapsed, budget, want) in cases {
            assert_eq!(
                sweep_step(elapsed, budget),
                want,
                "elapsed={elapsed:?} budget={budget:?}"
            );
        }
    }

    /// `budget_exhausted` is DERIVED from the count, both directions.
    #[test]
    fn budget_exhausted_reads_the_deferred_count() {
        assert!(!BoundedCleanup::default().budget_exhausted());
        assert!(!BoundedCleanup {
            cleanups: 9,
            failed_cleanups: 3,
            deferred: 0
        }
        .budget_exhausted());
        assert!(BoundedCleanup {
            cleanups: 0,
            failed_cleanups: 0,
            deferred: 1
        }
        .budget_exhausted());
    }

    /// Drive [`run_budgeted_sweep`] with a HAND-FED clock over the four shapes a
    /// real sweep takes.
    ///
    /// This is the load-bearing arm: it is the only place the `Defer` path is
    /// reachable, because a healthy host has no dead nodes and the real sweep
    /// then reports zeros whatever the budget does.
    #[test]
    fn the_budget_stops_attempting_and_starts_counting() {
        fn stepped(values: Vec<u64>) -> impl FnMut() -> Duration {
            let mut i = 0;
            move || {
                let v = values.get(i).copied().unwrap_or(u64::MAX);
                i += 1;
                Duration::from_millis(v)
            }
        }

        // (1) A generous budget attempts everything; Ok/Err counted apart.
        let mut attempted = Vec::new();
        let report = run_budgeted_sweep(
            vec![1, 2, 3],
            Duration::from_millis(1000),
            stepped(vec![0, 0, 0]),
            |n| {
                attempted.push(n);
                if n == 2 {
                    Err(())
                } else {
                    Ok(())
                }
            },
        );
        assert_eq!(attempted, vec![1, 2, 3]);
        assert_eq!(
            (report.cleanups, report.failed_cleanups, report.deferred),
            (2, 1, 0)
        );
        assert!(!report.budget_exhausted());

        // (2) A ZERO budget attempts NOTHING and defers everything.
        let mut attempted = Vec::new();
        let report =
            run_budgeted_sweep(vec![1, 2, 3], Duration::ZERO, stepped(vec![0, 0, 0]), |n| {
                attempted.push(n);
                Ok(())
            });
        assert!(
            attempted.is_empty(),
            "a ZERO budget must not attempt a single removal, got {attempted:?}"
        );
        assert_eq!(
            (report.cleanups, report.failed_cleanups, report.deferred),
            (0, 0, 3)
        );
        assert!(report.budget_exhausted());

        // (3) A budget spent MID-WALK attempts the prefix and COUNTS the rest —
        //     the shape a real overloaded host takes.
        let mut attempted = Vec::new();
        let report = run_budgeted_sweep(
            vec![1, 2, 3, 4],
            Duration::from_millis(100),
            stepped(vec![0, 50, 100, 150]),
            |n| {
                attempted.push(n);
                Ok(())
            },
        );
        assert_eq!(
            attempted,
            vec![1, 2],
            "only the nodes reached before the budget was spent may be attempted"
        );
        assert_eq!(
            (report.cleanups, report.failed_cleanups, report.deferred),
            (2, 0, 2),
            "the remainder must be COUNTED, not silently skipped"
        );
        assert!(report.budget_exhausted());

        // (4) A clean host reports nothing at all — not "exhausted".
        let report =
            run_budgeted_sweep(
                Vec::<u8>::new(),
                Duration::ZERO,
                stepped(vec![]),
                |_| Ok(()),
            );
        assert_eq!(report, BoundedCleanup::default());
        assert!(!report.budget_exhausted());
    }

    /// The startup budget is small enough to be a bring-up cost nobody notices.
    ///
    /// A drift guard rather than a claim about a specific number: the point of
    /// the constant is that startup hygiene stays an order of magnitude below
    /// the seconds a bring-up handshake is allowed.
    #[test]
    fn the_startup_budget_stays_a_bring_up_cost() {
        assert!(STARTUP_DEAD_NODE_SWEEP_BUDGET >= Duration::from_millis(100));
        assert!(STARTUP_DEAD_NODE_SWEEP_BUDGET <= Duration::from_secs(5));
    }
}
