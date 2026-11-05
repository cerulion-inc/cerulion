// SPDX-License-Identifier: AGPL-3.0-only
//! Hosted runs: runs whose host serves its own description from memory.
//!
//! `graph run` publishes a run registry record and writes a run directory, and
//! the `runs` verb gathers both at serve time. An embedded host has neither, and
//! it must not fake them: the run directory root is `CERULION_HOME/runs` (or
//! `~/.cerulion/runs`) and the serve side refuses a directory outside it. So the
//! host puts its run here, on its [`NetworkManager`](super::network::NetworkManager),
//! and the `runs` verb folds these entries into the same reply as the gathered
//! ones.
//!
//! The one writer of a hosted run is the [`HostedRunGuard`] that
//! [`HostedRuns::host`] returns. Dropping the guard withdraws the run, so a run
//! cannot stay listed after its host lets go of it.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use super::cerulion_q::{format_run_id, RunEntry, RunEntryState};

/// One run a host serves from memory.
#[derive(Debug)]
pub struct HostedRun {
    /// The entry the `runs` verb serves. Its `state` is not read: the served
    /// state comes from [`Self::live`] at serve time.
    pub entry: RunEntry,
    /// Set while the run executes. The host clears it when the run stops for any
    /// reason; from then on the run is served as [`RunEntryState::Ending`] until
    /// its guard is dropped.
    pub live: Arc<AtomicBool>,
}

/// One hosted run plus the generation of the [`HostedRuns::host`] call that put
/// it there, so a replaced run's guard cannot withdraw its replacement.
#[derive(Debug)]
struct Slot {
    generation: u64,
    run: HostedRun,
}

#[derive(Debug, Default)]
struct Slots {
    next_generation: u64,
    runs: BTreeMap<u128, Slot>,
}

/// The runs one host serves from memory, keyed by run id.
///
/// Cheap to clone: every clone is a handle onto the same set. A
/// [`NetworkManager`](super::network::NetworkManager) owns one from its
/// construction and hands clones to its query surface, so a run hosted through
/// any clone is served by that surface.
#[derive(Debug, Clone, Default)]
pub struct HostedRuns {
    inner: Arc<Mutex<Slots>>,
}

/// The right to keep one hosted run listed. Dropping it withdraws the run.
#[must_use = "dropping the guard withdraws the run"]
#[derive(Debug)]
pub struct HostedRunGuard {
    runs: HostedRuns,
    run_id: u128,
    generation: u64,
}

impl HostedRuns {
    /// Host `run` under `run_id` until the returned guard drops.
    ///
    /// A run already hosted under `run_id` is replaced, and the guard of the
    /// replaced run no longer withdraws anything.
    ///
    /// `run_id` is the run's identity: the served entry carries its canonical
    /// text. An entry that names a different id is served under `run_id`, with a
    /// warning, because the guard and the `runs` dedup both key on `run_id`.
    pub fn host(&self, run_id: u128, mut run: HostedRun) -> HostedRunGuard {
        let canonical = format_run_id(run_id);
        if run.entry.run_id != canonical {
            tracing::warn!(
                run_id = %canonical,
                entry_run_id = %run.entry.run_id,
                "hosted run: the entry names a different run id; serving it under the \
                 hosted run id"
            );
            run.entry.run_id = canonical;
        }
        let mut slots = self.lock();
        slots.next_generation += 1;
        let generation = slots.next_generation;
        slots.runs.insert(run_id, Slot { generation, run });
        HostedRunGuard {
            // hot-path-alloc-ok: one Arc handle per hosted run, when the host starts it.
            runs: self.clone(),
            run_id,
            generation,
        }
    }

    // hot-path-alloc-ok-fn: one clone per hosted run per remote runs GET, on
    // the control plane; no message or sample passes through here.
    /// The entries to serve right now, in run id order, each with the state its
    /// `live` flag reads now.
    pub(crate) fn entries(&self) -> Vec<RunEntry> {
        self.lock()
            .runs
            .values()
            .map(|slot| {
                let mut entry = slot.run.entry.clone();
                entry.state = if slot.run.live.load(Ordering::Acquire) {
                    RunEntryState::Live
                } else {
                    RunEntryState::Ending
                };
                entry
            })
            .collect()
    }

    fn lock(&self) -> MutexGuard<'_, Slots> {
        self.inner.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Hold the slot's lock, so a test can prove a path never reads the slot:
    /// such a path completes while the guard is held.
    #[cfg(test)]
    pub(crate) fn hold_lock_for_test(&self) -> MutexGuard<'_, impl Sized> {
        self.lock()
    }
}

impl Drop for HostedRunGuard {
    fn drop(&mut self) {
        let mut slots = self.runs.lock();
        if slots
            .runs
            .get(&self.run_id)
            .is_some_and(|slot| slot.generation == self.generation)
        {
            slots.runs.remove(&self.run_id);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::run_registry::RunState;

    const RUN_A: u128 = 0x5d1c_0000_0000_0000_0000_0000_0000_9e02;
    const RUN_B: u128 = 0x0bee;
    const RUN_A_TEXT: &str = "0x5d1c0000000000000000000000009e02";
    const RUN_B_TEXT: &str = "0x00000000000000000000000000000bee";

    fn entry(run_id: u128, graph_name: &str, started_at_ns: u64) -> RunEntry {
        RunEntry::new(
            run_id,
            graph_name,
            started_at_ns,
            RunState::Live,
            "name: temi\n",
            "{\"version\":1}\n",
        )
        .expect("a well-formed entry")
    }

    fn run(run_id: u128, graph_name: &str, live: bool) -> (HostedRun, Arc<AtomicBool>) {
        let flag = Arc::new(AtomicBool::new(live));
        (
            HostedRun {
                entry: entry(run_id, graph_name, 1_762_264_931_000_000_000),
                live: Arc::clone(&flag),
            },
            flag,
        )
    }

    fn served(runs: &HostedRuns) -> Vec<(String, String, RunEntryState)> {
        runs.entries()
            .into_iter()
            .map(|e| (e.run_id, e.graph_name, e.state))
            .collect()
    }

    #[test]
    fn an_empty_slot_serves_nothing() {
        assert!(HostedRuns::default().entries().is_empty());
    }

    #[test]
    fn a_hosted_run_is_served_verbatim_while_live() {
        let runs = HostedRuns::default();
        let (hosted, _live) = run(RUN_A, "temi", true);
        let _guard = runs.host(RUN_A, hosted);

        assert_eq!(
            runs.entries(),
            vec![RunEntry {
                run_id: RUN_A_TEXT.to_string(),
                graph_name: "temi".to_string(),
                run_started_at_ns: 1_762_264_931_000_000_000,
                state: RunEntryState::Live,
                graph_yaml: "name: temi\n".to_string(),
                run_json: "{\"version\":1}\n".to_string(),
            }]
        );
    }

    #[test]
    fn a_run_whose_live_flag_clears_is_served_as_ending_at_the_next_read() {
        let runs = HostedRuns::default();
        let (hosted, live) = run(RUN_A, "temi", true);
        let _guard = runs.host(RUN_A, hosted);
        assert_eq!(served(&runs)[0].2, RunEntryState::Live);

        live.store(false, Ordering::Release);

        assert_eq!(
            served(&runs),
            vec![(
                RUN_A_TEXT.to_string(),
                "temi".to_string(),
                RunEntryState::Ending
            )]
        );
    }

    #[test]
    fn the_served_state_comes_from_the_live_flag_not_the_entry() {
        let runs = HostedRuns::default();
        let flag = Arc::new(AtomicBool::new(false));
        let mut hosted_entry = entry(RUN_A, "temi", 1);
        hosted_entry.state = RunEntryState::Live;
        let _guard = runs.host(
            RUN_A,
            HostedRun {
                entry: hosted_entry,
                live: Arc::clone(&flag),
            },
        );
        assert_eq!(served(&runs)[0].2, RunEntryState::Ending);
    }

    #[test]
    fn dropping_the_guard_withdraws_only_its_own_run() {
        let runs = HostedRuns::default();
        let (a, _live_a) = run(RUN_A, "temi", true);
        let (b, _live_b) = run(RUN_B, "other", true);
        let guard_a = runs.host(RUN_A, a);
        let _guard_b = runs.host(RUN_B, b);
        assert_eq!(served(&runs).len(), 2);

        drop(guard_a);

        assert_eq!(
            served(&runs),
            vec![(
                RUN_B_TEXT.to_string(),
                "other".to_string(),
                RunEntryState::Live
            )]
        );
    }

    #[test]
    fn hosting_a_same_id_run_replaces_it_and_the_old_guard_cannot_withdraw_the_new_one() {
        let runs = HostedRuns::default();
        let (first, _live_first) = run(RUN_A, "first", true);
        let (second, _live_second) = run(RUN_A, "second", true);
        let old_guard = runs.host(RUN_A, first);
        let new_guard = runs.host(RUN_A, second);
        assert_eq!(
            served(&runs),
            vec![(
                RUN_A_TEXT.to_string(),
                "second".to_string(),
                RunEntryState::Live
            )]
        );

        drop(old_guard);
        assert_eq!(served(&runs).len(), 1, "the replacement must stay listed");

        drop(new_guard);
        assert!(runs.entries().is_empty());
    }

    #[test]
    fn every_clone_is_a_handle_onto_the_same_set() {
        let runs = HostedRuns::default();
        let served_by = runs.clone();
        let (hosted, _live) = run(RUN_A, "temi", true);

        let guard = runs.host(RUN_A, hosted);
        assert_eq!(served(&served_by).len(), 1);

        drop(guard);
        assert!(served_by.entries().is_empty());
    }

    #[test]
    fn an_entry_naming_another_run_id_is_served_under_the_hosted_id() {
        let runs = HostedRuns::default();
        let flag = Arc::new(AtomicBool::new(true));
        let guard = runs.host(
            RUN_A,
            HostedRun {
                entry: entry(RUN_B, "temi", 1),
                live: flag,
            },
        );
        assert_eq!(served(&runs)[0].0, RUN_A_TEXT);

        drop(guard);
        assert!(
            runs.entries().is_empty(),
            "the guard withdraws the id the run is served under"
        );
    }

    #[test]
    fn entries_are_served_in_run_id_order() {
        let runs = HostedRuns::default();
        let (a, _live_a) = run(RUN_A, "a", true);
        let (b, _live_b) = run(RUN_B, "b", true);
        let _guard_a = runs.host(RUN_A, a);
        let _guard_b = runs.host(RUN_B, b);
        let ids: Vec<String> = runs.entries().into_iter().map(|e| e.run_id).collect();
        assert_eq!(ids, vec![RUN_B_TEXT.to_string(), RUN_A_TEXT.to_string()]);
    }

    #[test]
    fn the_guard_is_must_use() {
        let source = include_str!("hosted_runs.rs");
        let guard_decl = source
            .find("pub struct HostedRunGuard")
            .expect("the guard is declared in this file");
        let attributes = &source[..guard_decl];
        let attributes = &attributes[attributes.rfind("///").unwrap_or(0)..];
        assert!(
            attributes.contains("#[must_use = \"dropping the guard withdraws the run\"]"),
            "a guard dropped at once withdraws its run at once; the compiler must warn \
             about an unused guard, got attributes {attributes:?}"
        );
    }
}
