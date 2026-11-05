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

use super::cerulion_q::{format_run_id, RunEntry, RunEntryError, RunEntryState, UndescribableRun};
use super::run_registry::RunState;

/// One run a host serves from memory.
#[derive(Debug)]
pub struct HostedRun {
    /// The entry the `runs` verb serves, once [`HostedRuns::host`] has checked
    /// it again through [`RunEntry::new`]. Its `state` is not read: the served
    /// state comes from [`Self::live`] at serve time.
    pub entry: RunEntry,
    /// Set while the run executes. The host clears it when the run stops for any
    /// reason; from then on the run is served as [`RunEntryState::Ending`] until
    /// its guard is dropped.
    pub live: Arc<AtomicBool>,
}

/// One hosted run as the `runs` verb serves it, plus the generation of the
/// [`HostedRuns::host`] call that put it there, so a replaced run's guard cannot
/// withdraw its replacement.
#[derive(Debug)]
struct Slot {
    generation: u64,
    live: Arc<AtomicBool>,
    /// The entry minted again through [`RunEntry::new`], or the reason it could
    /// not be.
    served: Result<RunEntry, UndescribableRun>,
}

/// What the `runs` verb serves from a [`HostedRuns`] at one instant.
#[derive(Debug, Default, PartialEq, Eq)]
pub(crate) struct HostedServe {
    /// The describable runs, in run id order, each with the state its `live`
    /// flag reads now.
    pub(crate) entries: Vec<RunEntry>,
    /// The runs whose entry fails the checks every served entry meets, in run
    /// id order, each with the reason.
    pub(crate) undescribable: Vec<UndescribableRun>,
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
///
/// # The compiler refuses a guard that is dropped at once
///
/// ```compile_fail
/// #![deny(unused_must_use)]
/// use std::sync::{atomic::AtomicBool, Arc};
/// use cerulion_core::transport::cerulion_q::RunEntry;
/// use cerulion_core::transport::hosted_runs::{HostedRun, HostedRuns};
/// use cerulion_core::transport::run_registry::RunState;
/// let entry = RunEntry::new(0x1, "temi", 1, RunState::Live, "name: temi\n", "{}\n").unwrap();
/// let runs = HostedRuns::default();
/// // ERROR: the unused guard drops here, which withdraws the run at once.
/// runs.host(0x1, HostedRun { entry, live: Arc::new(AtomicBool::new(true)) });
/// ```
///
/// # The twin: the same code, holding the guard, compiles
///
/// A `compile_fail` block passes when compilation fails for any reason, a
/// missing import included, so the block above is paired with this one.
///
/// ```
/// #![deny(unused_must_use)]
/// use std::sync::{atomic::AtomicBool, Arc};
/// use cerulion_core::transport::cerulion_q::RunEntry;
/// use cerulion_core::transport::hosted_runs::{HostedRun, HostedRuns};
/// use cerulion_core::transport::run_registry::RunState;
/// let entry = RunEntry::new(0x1, "temi", 1, RunState::Live, "name: temi\n", "{}\n").unwrap();
/// let runs = HostedRuns::default();
/// let _guard = runs.host(0x1, HostedRun { entry, live: Arc::new(AtomicBool::new(true)) });
/// ```
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
    ///
    /// The entry is minted again through [`RunEntry::new`], so it meets the
    /// checks every served entry meets, however the host built it. An entry that
    /// fails them (an empty field, or an artifact above the served cap) is served
    /// as an undescribable run with the reason, never as an entry: a desk refuses
    /// the whole reply when one entry fails those checks.
    pub fn host(&self, run_id: u128, run: HostedRun) -> HostedRunGuard {
        let HostedRun { entry, live } = run;
        let RunEntry {
            run_id: entry_run_id,
            graph_name,
            run_started_at_ns,
            state: _,
            graph_yaml,
            run_json,
        } = entry;
        let canonical = format_run_id(run_id);
        if entry_run_id != canonical {
            tracing::warn!(
                run_id = %canonical,
                entry_run_id = %entry_run_id,
                "hosted run: the entry names a different run id; serving it under the \
                 hosted run id"
            );
        }
        // The state argument is not served: `served` reads it from `live`.
        let served = RunEntry::new(
            run_id,
            graph_name,
            run_started_at_ns,
            RunState::Live,
            graph_yaml,
            run_json,
        )
        .map_err(|error| {
            tracing::warn!(
                run_id = %canonical,
                error = %error,
                "hosted run: the entry fails the served-entry checks; the runs verb \
                 names it undescribable"
            );
            UndescribableRun {
                run_id: canonical,
                reason: refusal_reason(error),
            }
        });
        let mut slots = self.lock();
        slots.next_generation += 1;
        let generation = slots.next_generation;
        slots.runs.insert(
            run_id,
            Slot {
                generation,
                live,
                served,
            },
        );
        HostedRunGuard {
            // hot-path-alloc-ok: one Arc handle per hosted run, when the host starts it.
            runs: self.clone(),
            run_id,
            generation,
        }
    }

    // hot-path-alloc-ok-fn: one clone per hosted run per remote runs GET, on
    // the control plane; no message or sample passes through here.
    /// What to serve right now, in run id order: each describable run with the
    /// state its `live` flag reads now, and each undescribable run with its
    /// reason. One lock, so one host call cannot land between the two lists.
    pub(crate) fn served(&self) -> HostedServe {
        let slots = self.lock();
        let mut served = HostedServe::default();
        for slot in slots.runs.values() {
            match &slot.served {
                Ok(entry) => {
                    let mut entry = entry.clone();
                    entry.state = if slot.live.load(Ordering::Acquire) {
                        RunEntryState::Live
                    } else {
                        RunEntryState::Ending
                    };
                    served.entries.push(entry);
                }
                Err(withheld) => served.undescribable.push(withheld.clone()),
            }
        }
        served
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

// hot-path-alloc-ok-fn: once per refused hosted entry, when the host starts
// the run.
/// The wire reason for a hosted entry that fails the served-entry checks. The
/// checks' own text names a run directory or a run record, which a hosted run
/// does not have.
fn refusal_reason(error: RunEntryError) -> String {
    match error {
        RunEntryError::EmptyGraphName => "the hosted entry carries an empty graph name".to_string(),
        RunEntryError::EmptyGraphYaml => "the hosted entry carries an empty graph.yaml".to_string(),
        RunEntryError::EmptyRunJson => "the hosted entry carries an empty run.json".to_string(),
        RunEntryError::GraphYamlTooLong { .. } | RunEntryError::RunJsonTooLong { .. } => {
            format!("the hosted entry's {error}")
        }
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

    fn listed(runs: &HostedRuns) -> Vec<(String, String, RunEntryState)> {
        runs.served()
            .entries
            .into_iter()
            .map(|e| (e.run_id, e.graph_name, e.state))
            .collect()
    }

    fn by_hand(
        entry_run_id: &str,
        graph_name: &str,
        graph_yaml: impl Into<String>,
        run_json: impl Into<String>,
    ) -> HostedRun {
        HostedRun {
            entry: RunEntry {
                run_id: entry_run_id.to_string(),
                graph_name: graph_name.to_string(),
                run_started_at_ns: 1,
                state: RunEntryState::Live,
                graph_yaml: graph_yaml.into(),
                run_json: run_json.into(),
            },
            live: Arc::new(AtomicBool::new(true)),
        }
    }

    fn withheld(run_id_text: &str, reason: &str) -> UndescribableRun {
        UndescribableRun {
            run_id: run_id_text.to_string(),
            reason: reason.to_string(),
        }
    }

    #[test]
    fn an_empty_slot_serves_nothing() {
        assert_eq!(HostedRuns::default().served(), HostedServe::default());
    }

    #[test]
    fn a_hosted_run_is_served_verbatim_while_live() {
        let runs = HostedRuns::default();
        let (hosted, _live) = run(RUN_A, "temi", true);
        let _guard = runs.host(RUN_A, hosted);

        assert_eq!(
            runs.served().entries,
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
        assert_eq!(listed(&runs)[0].2, RunEntryState::Live);

        live.store(false, Ordering::Release);

        assert_eq!(
            listed(&runs),
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
        assert_eq!(listed(&runs)[0].2, RunEntryState::Ending);
    }

    #[test]
    fn dropping_the_guard_withdraws_only_its_own_run() {
        let runs = HostedRuns::default();
        let (a, _live_a) = run(RUN_A, "temi", true);
        let (b, _live_b) = run(RUN_B, "other", true);
        let guard_a = runs.host(RUN_A, a);
        let _guard_b = runs.host(RUN_B, b);
        assert_eq!(listed(&runs).len(), 2);

        drop(guard_a);

        assert_eq!(
            listed(&runs),
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
            listed(&runs),
            vec![(
                RUN_A_TEXT.to_string(),
                "second".to_string(),
                RunEntryState::Live
            )]
        );

        drop(old_guard);
        assert_eq!(listed(&runs).len(), 1, "the replacement must stay listed");

        drop(new_guard);
        assert!(runs.served().entries.is_empty());
    }

    #[test]
    fn every_clone_is_a_handle_onto_the_same_set() {
        let runs = HostedRuns::default();
        let served_by = runs.clone();
        let (hosted, _live) = run(RUN_A, "temi", true);

        let guard = runs.host(RUN_A, hosted);
        assert_eq!(listed(&served_by).len(), 1);

        drop(guard);
        assert!(served_by.served().entries.is_empty());
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
        assert_eq!(listed(&runs)[0].0, RUN_A_TEXT);

        drop(guard);
        assert!(
            runs.served().entries.is_empty(),
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
        let ids: Vec<String> = runs
            .served()
            .entries
            .into_iter()
            .map(|e| e.run_id)
            .collect();
        assert_eq!(ids, vec![RUN_B_TEXT.to_string(), RUN_A_TEXT.to_string()]);
    }

    #[test]
    fn an_entry_built_by_hand_that_new_would_refuse_is_named_undescribable_with_its_reason() {
        let over_cap = "y".repeat(256 * 1024 + 1);
        let cases: [(&str, String, String, &str); 5] = [
            (
                "",
                "name: temi\n".to_string(),
                "{}\n".to_string(),
                "the hosted entry carries an empty graph name",
            ),
            (
                "temi",
                String::new(),
                "{}\n".to_string(),
                "the hosted entry carries an empty graph.yaml",
            ),
            (
                "temi",
                "name: temi\n".to_string(),
                String::new(),
                "the hosted entry carries an empty run.json",
            ),
            (
                "temi",
                over_cap.clone(),
                "{}\n".to_string(),
                "the hosted entry's graph.yaml is 262145 bytes, above the 262144-byte served cap",
            ),
            (
                "temi",
                "name: temi\n".to_string(),
                over_cap,
                "the hosted entry's run.json is 262145 bytes, above the 262144-byte served cap",
            ),
        ];
        for (graph_name, graph_yaml, run_json, reason) in cases {
            let runs = HostedRuns::default();
            let _guard = runs.host(RUN_A, by_hand(RUN_A_TEXT, graph_name, graph_yaml, run_json));
            assert_eq!(
                runs.served(),
                HostedServe {
                    entries: Vec::new(),
                    undescribable: vec![withheld(RUN_A_TEXT, reason)],
                },
                "case: {reason}"
            );
        }
    }

    #[test]
    fn an_artifact_at_the_cap_built_by_hand_is_served() {
        let runs = HostedRuns::default();
        let _guard = runs.host(
            RUN_A,
            by_hand(RUN_A_TEXT, "temi", "y".repeat(256 * 1024), "{}\n"),
        );
        let served = runs.served();
        assert!(
            served.undescribable.is_empty(),
            "{:?}",
            served.undescribable
        );
        assert_eq!(
            served
                .entries
                .iter()
                .map(|e| (e.run_id.as_str(), e.graph_yaml.len()))
                .collect::<Vec<_>>(),
            vec![(RUN_A_TEXT, 262_144)]
        );
    }

    #[test]
    fn a_refused_entry_is_named_under_the_hosted_id() {
        let runs = HostedRuns::default();
        let _guard = runs.host(RUN_A, by_hand("0xBAD", "temi", "", "{}\n"));
        assert_eq!(
            runs.served().undescribable,
            vec![withheld(
                RUN_A_TEXT,
                "the hosted entry carries an empty graph.yaml"
            )]
        );
    }

    #[test]
    fn a_refused_run_does_not_hide_the_runs_beside_it() {
        let runs = HostedRuns::default();
        let _bad = runs.host(RUN_A, by_hand(RUN_A_TEXT, "temi", "", "{}\n"));
        let (good, _live) = run(RUN_B, "other", true);
        let _good = runs.host(RUN_B, good);
        assert_eq!(
            runs.served(),
            HostedServe {
                entries: vec![RunEntry {
                    run_id: RUN_B_TEXT.to_string(),
                    graph_name: "other".to_string(),
                    run_started_at_ns: 1_762_264_931_000_000_000,
                    state: RunEntryState::Live,
                    graph_yaml: "name: temi\n".to_string(),
                    run_json: "{\"version\":1}\n".to_string(),
                }],
                undescribable: vec![withheld(
                    RUN_A_TEXT,
                    "the hosted entry carries an empty graph.yaml"
                )],
            }
        );
    }

    #[test]
    fn a_refused_run_is_withdrawn_by_its_guard_and_a_good_replacement_is_served() {
        let runs = HostedRuns::default();
        let bad = runs.host(RUN_A, by_hand(RUN_A_TEXT, "temi", "", "{}\n"));
        assert_eq!(runs.served().undescribable.len(), 1);
        drop(bad);
        assert_eq!(runs.served(), HostedServe::default());

        let _bad = runs.host(RUN_A, by_hand(RUN_A_TEXT, "temi", "", "{}\n"));
        let (good, _live) = run(RUN_A, "temi", true);
        let _good = runs.host(RUN_A, good);
        let served = runs.served();
        assert!(
            served.undescribable.is_empty(),
            "{:?}",
            served.undescribable
        );
        assert_eq!(
            listed(&runs),
            vec![(
                RUN_A_TEXT.to_string(),
                "temi".to_string(),
                RunEntryState::Live
            )]
        );
    }
}
