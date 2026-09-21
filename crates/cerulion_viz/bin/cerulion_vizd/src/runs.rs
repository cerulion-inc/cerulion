// SPDX-License-Identifier: AGPL-3.0-only
//! The `runs` FOLD: this machine's live runs unioned with every
//! remote robot's, exactly as `discover` unions local iceoryx2 topics with the
//! remote catalog.
//!
//! # Why this module is PURE
//!
//! Everything below is arithmetic over answers somebody else already gathered. The
//! LOCAL arm's answer comes from `TransportManager::gather_live_runs` +
//! `run_artifacts::collect_run_entries`; the REMOTE arm's from
//! `cerulion_netd::NetdClient::query_runs`. Neither is reachable from a hermetic
//! test — one needs a live run registry on this machine, the other a robot on the
//! LAN — so the DECISIONS live here where hand-written oracles can drive every one
//! of them, and the daemon keeps only the plumbing.
//!
//! # Attribution, and why it is structural rather than careful
//!
//! Attribution is a property of the KEY an answer came back on, never of a merge.
//! A remote reply arrived on the explicit `cerulion_q/{robot}/runs` selector and is
//! attributed to that robot; a local gather crossed no such key and is attributed
//! to nothing. [`RunsOrigin`] makes that a TYPE rather than a convention: there is
//! no `robot: Option<String>` for a caller to fill in wrongly, so the phantom "this
//! machine" row that was removed from the sidebar is not constructible here.
//!
//! # Completeness is folded CONSERVATIVELY, and its inputs are already correct
//!
//! Each source's own verdict already carries its refusal (`RunsReply::refused` is
//! `not_established` by construction) and its withheld runs (`build_runs_reply`
//! demotes `Settled` when anything is undescribable). So the fold does NOT
//! re-derive either rule — a second copy of a policy is free to disagree with the
//! first — it only asks whether EVERY source settled, plus the three things no
//! single source can know: that nothing was unusable, that the remote arm ran at
//! all, and that discovery had converged.
//!
//! # One run, one row — a `run_id` is an identity, not a label
//!
//! A desk whose own `cerulion-netd` ANNOUNCES (a robot-side box someone opened
//! Studio on, or a dev machine standing in for both) sees each of its graphs twice:
//! once through the registry gather (`Local`) and once through the LAN, on the
//! `cerulion_q/{hostname}/runs` selector (`Robot(hostname)`). Both rows describe the
//! SAME run: `mint_run_id` is distinct per invocation by construction, so two
//! sources agreeing on a 128-bit id agree on the run. The fold keeps the first row
//! it sees — the local one, first-hand — and drops the echo, without collapsing
//! anything else: two robots running the same graph mint two ids and stay two rows,
//! and every SOURCE still reports in `sources` (the echo dedups ROWS, not answers).

use crate::protocol::{
    DiscoveryLabel, RunRow, RunsCompleteness, RunsSourceStatus, UndescribableRun,
    UnusableRunsAnswer,
};
use cerulion_core::RunsReply;

/// WHICH machine an answer came from — the attribution, as a type.
///
/// See the module docs: this exists so a local run cannot be stamped with a robot
/// name by a caller that has one to hand.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunsOrigin {
    /// THIS machine's run registry. Renders as `robot: None` — never this desk's
    /// hostname.
    Local,
    /// A remote robot, spelled as the desk QUERIED it (the announce identity in the
    /// explicit selector, already normalized by the gather).
    Robot(String),
}

impl RunsOrigin {
    /// The wire attribution: `None` for [`Self::Local`].
    ///
    /// The ONE place the mapping happens, so the rule has a single site to
    /// pin and a single site to mutate.
    #[must_use]
    pub fn wire(&self) -> Option<String> {
        match self {
            Self::Local => None,
            Self::Robot(robot) => Some(robot.clone()),
        }
    }
}

/// One source's contribution: where it came from, and what it said.
///
/// The reply's OWN `robot` field is deliberately ignored for attribution — see
/// [`RunsOrigin`]. It is still the right thing for the source to carry (a serve
/// self-attributes, and the gather normalizes it), but a fold that trusted it
/// would let a robot's self-description decide where its rows appear.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunsSourceAnswer {
    /// Where this answer came from.
    pub origin: RunsOrigin,
    /// What that source replied.
    pub reply: RunsReply,
}

/// What the REMOTE arm did.
///
/// Three states, and collapsing any two of them loses a remedy — see
/// [`RunsResponse`](crate::protocol::RunsResponse)'s type docs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RemoteArm {
    /// netd ran the query. `replies` is one per robot that answered USABLY;
    /// `unusable` is one per robot that answered with bytes this binary could not
    /// decode (the REDEPLOY signal); `silent` is one per robot that was ASKED and
    /// did not answer at all (the COVERAGE half); `discovery` is whether netd's LAN
    /// discovery had converged when it answered.
    Answered {
        /// One per robot that answered usably.
        replies: Vec<RunsReply>,
        /// One per robot whose answer could not be decoded.
        unusable: Vec<UnusableRunsAnswer>,
        /// One per robot that was asked and stayed silent.
        silent: Vec<String>,
        /// Whether netd's discovery had converged.
        discovery: DiscoveryLabel,
    },
    /// netd could not run the query at all (unreachable / not network-configured).
    /// The remote half of the answer is UNKNOWN — never empty-and-settled.
    Degraded(String),
    /// The remote arm was deliberately not run. Reachable only from a request that
    /// scoped itself to the LOCAL machine; there is no such request today, so this
    /// variant exists for the symmetry that makes [`fold_runs`] total rather than
    /// for a live caller.
    NotAsked,
}

/// What the LOCAL arm did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LocalArm {
    /// The registry gather ran; `reply` is its answer, assembled through the SAME
    /// `build_runs_reply` a robot's serve uses (so the dedup, the display sort and
    /// the undescribable demotion are one implementation, not two).
    Gathered(Box<RunsReply>),
    /// The gather could not be run (an SHM / config failure opening the registry
    /// service). Reported as a source that established nothing, never as "this
    /// machine is running nothing".
    Failed(String),
    /// Not asked — the request scoped itself to one robot. NOT a shortfall: a
    /// local run is attributed `robot: None` and no robot name can select one, so
    /// skipping the local gather is the request being honoured.
    NotAsked,
}

/// The folded answer, ready to become a
/// [`RunsResponse`](crate::protocol::RunsResponse).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunsFold {
    /// Every live run, local first then by robot, each source's runs in the order
    /// its reply carried them (start time, then identity) — one row per `run_id`,
    /// the first source to name an id keeping it (see the module docs).
    pub runs: Vec<RunRow>,
    /// The aggregate verdict — see [`fold_runs`].
    pub completeness: RunsCompleteness,
    /// Per-source diagnostics, in the same order as [`Self::runs`]'s sources.
    pub sources: Vec<RunsSourceStatus>,
    /// Robots that answered unusably (the redeploy signal).
    pub unusable: Vec<UnusableRunsAnswer>,
    /// Robots that were asked and stayed silent (the coverage shortfall).
    pub silent: Vec<String>,
    /// netd's discovery state, or `None` when the remote arm did not report one.
    pub discovery: Option<DiscoveryLabel>,
    /// Why the remote arm could not be run, when it could not.
    pub degraded: Option<String>,
}

/// Fold the two arms into one answer.
///
/// # The aggregate verdict
///
/// `Settled` — i.e. "an empty list really does mean nothing is running" — requires
/// ALL of:
///
/// * at least one source CONTRIBUTED (an answer nobody supplied evidence for is
///   not an absence claim, it is silence);
/// * every contributing source's own verdict is `Settled` (which already folds in
///   its refusal and its withheld runs — see the module docs);
/// * nothing was UNUSABLE (a robot whose bytes we could not decode may be running
///   anything);
/// * every asked robot ANSWERED — an announced robot that stayed SILENT is a
///   coverage hole, and its absence from `sources` is indistinguishable from a
///   robot nobody asked about, so an unqualified `Settled` would render its runs
///   as "nothing is running there";
/// * the remote arm RAN — a `Degraded` arm leaves every robot unknown, and a
///   `NotAsked` one was scoped to a single robot, so neither can license a claim
///   about "everywhere I can see";
/// * netd's discovery had CONVERGED, or a robot that has simply not been asked yet
///   would read as a robot that is not there (the discovery marker's thesis, on the surface a
///   person looks at).
///
/// When it is not `Settled` the counts are
/// [`RunsCompleteness::not_established`]'s no-claim zeros. They deliberately do NOT
/// sum the sources' writer counts: those are per-machine registry arithmetic, and
/// adding a robot's live-writer count to this desk's would produce a number that
/// describes no registry at all.
#[must_use]
pub fn fold_runs(local: LocalArm, remote: RemoteArm) -> RunsFold {
    let mut answers: Vec<RunsSourceAnswer> = Vec::new();
    let mut degraded = None;
    let mut local_failure = None;

    match local {
        LocalArm::Gathered(reply) => answers.push(RunsSourceAnswer {
            origin: RunsOrigin::Local,
            reply: *reply,
        }),
        // A failed local gather is a SOURCE that established nothing, not an
        // absent source: the machine is right here, and reporting silence where we
        // actually tried and failed would hide a real fault.
        LocalArm::Failed(reason) => local_failure = Some(reason),
        LocalArm::NotAsked => {}
    }

    let (unusable, silent, discovery, remote_ran) = match remote {
        RemoteArm::Answered {
            replies,
            unusable,
            silent,
            discovery,
        } => {
            for reply in replies {
                // Attribution comes from the KEY the gather asked on, which the
                // reply's `robot` carries AFTER normalization.
                let robot = reply.robot.clone();
                answers.push(RunsSourceAnswer {
                    origin: RunsOrigin::Robot(robot),
                    reply,
                });
            }
            (unusable, silent, Some(discovery), true)
        }
        RemoteArm::Degraded(reason) => {
            degraded = Some(reason);
            (Vec::new(), Vec::new(), None, false)
        }
        RemoteArm::NotAsked => (Vec::new(), Vec::new(), None, false),
    };

    let mut runs: Vec<RunRow> = Vec::new();
    let mut sources = Vec::new();
    if let Some(reason) = local_failure {
        sources.push(RunsSourceStatus {
            robot: None,
            completeness: RunsCompleteness::not_established(),
            error: Some(reason),
            undescribable: Vec::new(),
        });
    }
    for answer in answers {
        let robot = answer.origin.wire();
        for entry in answer.reply.runs {
            if runs.iter().any(|seen| seen.run_id == entry.run_id) {
                tracing::debug!(
                    run_id = %entry.run_id,
                    robot = ?robot,
                    "runs fold: a source echoed a run already listed — same run_id, one row"
                );
                continue;
            }
            runs.push(RunRow {
                robot: robot.clone(),
                run_id: entry.run_id,
                graph_name: entry.graph_name,
                run_started_at_ns: entry.run_started_at_ns,
                state: entry.state,
                graph_yaml: entry.graph_yaml,
                run_json: entry.run_json,
            });
        }
        sources.push(RunsSourceStatus {
            robot,
            completeness: answer.reply.completeness,
            error: answer.reply.error,
            undescribable: answer.reply.undescribable,
        });
    }

    let completeness = if sources_settle(&sources, &unusable, &silent, remote_ran, discovery) {
        RunsCompleteness::Settled
    } else {
        RunsCompleteness::not_established()
    };

    RunsFold {
        runs,
        completeness,
        sources,
        unusable,
        silent,
        discovery,
        degraded,
    }
}

/// The aggregate rule, extracted so it is drivable on its own — see [`fold_runs`]
/// for what each conjunct closes.
fn sources_settle(
    sources: &[RunsSourceStatus],
    unusable: &[UnusableRunsAnswer],
    silent: &[String],
    remote_ran: bool,
    discovery: Option<DiscoveryLabel>,
) -> bool {
    !sources.is_empty()
        && sources.iter().all(|s| s.completeness.is_settled())
        && unusable.is_empty()
        && silent.is_empty()
        && remote_ran
        && discovery == Some(DiscoveryLabel::Settled)
}

/// Assemble the LOCAL arm's reply from a registry gather's records.
///
/// It goes through `cerulion_core`'s own `build_runs_reply` — the assembler a
/// ROBOT's serve uses — so the local half of the fold and the remote half cannot
/// disagree about identity dedup, display order, or the demotion a withheld run
/// forces. The `robot` argument is a placeholder the fold never reads
/// ([`RunsOrigin::Local`] decides attribution), so it is spelled empty rather than
/// given this desk's hostname: a hostname sitting in a field nothing reads is one
/// refactor away from being read.
///
/// # The gather's verdict travels WHOLE, counts included
///
/// `completeness` is the registry gather's own `GatherCompleteness`, converted by
/// the total [`From`] impl `cerulion_core` already owns — NOT a boolean this
/// function re-expands. Reducing it to "settled or not" and rebuilding the other
/// side as [`RunsCompleteness::not_established`] threw away
/// `Incomplete { live_writers, writers_heard }`'s measured counts and replaced
/// them with that constructor's no-claim ZEROES, whose documented meaning is "no
/// writer arithmetic backs this verdict". So a desk that heard 2 of 3 live run
/// writers reported the same numbers as one that measured nothing at all — the
/// two states this desk's own local half can most usefully tell apart, since it is
/// the machine an operator can go and look at. Every REMOTE source's counts
/// already cross the wire intact; the local one was the only source losing them.
#[must_use]
pub fn local_reply(
    fold: cerulion_core::transport::run_artifacts::RunFold,
    completeness: RunsCompleteness,
) -> LocalArm {
    let undescribable: Vec<UndescribableRun> = fold
        .skipped
        .iter()
        .map(cerulion_core::transport::run_artifacts::SkippedRun::to_wire)
        .collect();
    LocalArm::Gathered(Box::new(
        cerulion_core::transport::cerulion_q::build_runs_reply(
            "",
            fold.entries,
            undescribable,
            completeness,
        ),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::transport::cerulion_q::{build_runs_reply, format_run_id};
    use cerulion_core::transport::run_artifacts::RunFold;
    use cerulion_core::transport::run_registry::RunState;
    use cerulion_core::RunEntry;

    /// A run entry with a hand-chosen identity + start time.
    fn entry(id: u128, graph: &str, started: u64) -> RunEntry {
        RunEntry::new(
            id,
            graph,
            started,
            RunState::Live,
            format!("name: {graph}\nnodes: []\n"),
            format!(r#"{{"run_id":"{}"}}"#, format_run_id(id)),
        )
        .expect("a hand-built entry is well formed")
    }

    /// A settled reply carrying `entries`, self-attributed to `robot`.
    fn settled(robot: &str, entries: Vec<RunEntry>) -> RunsReply {
        build_runs_reply(robot, entries, Vec::new(), RunsCompleteness::Settled)
    }

    /// A remote arm that answered cleanly with `replies` and nothing unusable.
    fn clean_remote(replies: Vec<RunsReply>) -> RemoteArm {
        RemoteArm::Answered {
            replies,
            unusable: Vec::new(),
            silent: Vec::new(),
            discovery: DiscoveryLabel::Settled,
        }
    }

    /// The `(robot, run_id)` pairs the fold produced, in order — the attribution
    /// oracle every arm below reads.
    fn attributed(fold: &RunsFold) -> Vec<(Option<&str>, &str)> {
        fold.runs
            .iter()
            .map(|r| (r.robot.as_deref(), r.run_id.as_str()))
            .collect()
    }

    // ── The fold matrix: local-only / remote-only / both / neither ─────────────

    /// LOCAL-ONLY — a desk running a graph with no robot on the LAN.
    ///
    /// **This is the attribution arm.** Every local row carries `robot: None`, and the
    /// obvious way to break this is a fold that had this desk's
    /// hostname to hand (it does — `discover` reads one) and stamped it here would
    /// mint the phantom "this machine" row that was removed from the sidebar, in a
    /// list whose entire purpose is telling you WHERE something runs.
    #[test]
    fn a_local_run_is_attributed_to_no_robot_at_all() {
        let local = LocalArm::Gathered(Box::new(settled(
            "",
            vec![entry(1, "perception", 100), entry(2, "control", 200)],
        )));
        let fold = fold_runs(local, clean_remote(Vec::new()));

        assert_eq!(
            attributed(&fold),
            vec![
                (None, format_run_id(1).as_str()),
                (None, format_run_id(2).as_str())
            ],
            "a LOCAL run carries no robot — never this desk's own hostname"
        );
        // …and the same rule at the ONE site that decides it, so a regression there is
        // caught even if a future fold stopped routing through it.
        assert_eq!(RunsOrigin::Local.wire(), None);
        assert_eq!(
            RunsOrigin::Robot("go2".into()).wire(),
            Some("go2".to_string())
        );

        assert_eq!(fold.sources.len(), 1);
        assert_eq!(fold.sources[0].robot, None);
        assert_eq!(
            fold.completeness,
            RunsCompleteness::Settled,
            "one settled local source, a settled LAN with nothing on it, nothing \
             unusable — an empty robot half is a real absence"
        );
    }

    /// REMOTE-ONLY — a desk running nothing, watching one robot.
    ///
    /// Attribution comes from the reply's (already-normalized) identity, so the row
    /// names the robot the desk QUERIED rather than whatever the robot called
    /// itself.
    #[test]
    fn a_remote_run_is_attributed_to_the_robot_that_answered() {
        let local = LocalArm::Gathered(Box::new(settled("", Vec::new())));
        let fold = fold_runs(
            local,
            clean_remote(vec![settled("go2", vec![entry(7, "nav", 500)])]),
        );

        assert_eq!(
            attributed(&fold),
            vec![(Some("go2"), format_run_id(7).as_str())]
        );
        assert_eq!(
            fold.sources
                .iter()
                .map(|s| s.robot.as_deref())
                .collect::<Vec<_>>(),
            vec![None, Some("go2")],
            "both sources are reported — the local one contributed an ANSWER even \
             though it contributed no runs, which is what makes its empty half a \
             claim rather than a silence"
        );
        assert_eq!(fold.completeness, RunsCompleteness::Settled);
    }

    /// BOTH — and the rows stay distinguishable.
    ///
    /// The local and remote runs deliberately share a GRAPH NAME here: on a desk
    /// developing against a robot that is the normal case, and a fold keyed on
    /// anything but the origin would merge or mis-attribute them.
    #[test]
    fn local_and_remote_runs_fold_into_one_list_without_losing_their_machines() {
        let local = LocalArm::Gathered(Box::new(settled("", vec![entry(1, "perception", 100)])));
        let fold = fold_runs(
            local,
            clean_remote(vec![
                settled("go2", vec![entry(2, "perception", 200)]),
                settled("spot", vec![entry(3, "perception", 300)]),
            ]),
        );

        assert_eq!(
            attributed(&fold),
            vec![
                (None, format_run_id(1).as_str()),
                (Some("go2"), format_run_id(2).as_str()),
                (Some("spot"), format_run_id(3).as_str()),
            ],
            "LOCAL first, then each robot — three runs of one graph name on three \
             machines stay three rows"
        );
        assert_eq!(fold.completeness, RunsCompleteness::Settled);
    }

    /// BOTH, where the "robot" IS this desk — a self-announcing netd echoes every
    /// local run back over the LAN under the hostname.
    ///
    /// Same `run_id` on both arms means the same run (`mint_run_id` is per
    /// invocation), so it is ONE row, attributed first-hand (`None`), while a
    /// robot's genuinely distinct run of the same graph name stays its own row. The
    /// failure modes: keep both (the picker lists "this machine" and "<hostname>" for one
    /// run, and switching between them is a no-op), or dedup on graph name (which
    /// would swallow `spot`'s run). `sources` is untouched — the echo is a
    /// duplicate ROW, not a duplicate ANSWER, and the hostname source still settled.
    #[test]
    fn a_run_echoed_by_this_desks_own_announce_is_listed_once_as_local() {
        let this_desk =
            LocalArm::Gathered(Box::new(settled("", vec![entry(1, "perception", 100)])));
        let fold = fold_runs(
            this_desk,
            clean_remote(vec![
                settled("desk-box", vec![entry(1, "perception", 100)]),
                settled("spot", vec![entry(3, "perception", 300)]),
            ]),
        );

        assert_eq!(
            attributed(&fold),
            vec![
                (None, format_run_id(1).as_str()),
                (Some("spot"), format_run_id(3).as_str()),
            ],
            "one run_id, one row — the LAN echo of this desk's own run is dropped and \
             the first-hand local row keeps its `None`; spot's distinct id survives"
        );
        assert_eq!(
            fold.sources
                .iter()
                .map(|s| s.robot.as_deref())
                .collect::<Vec<_>>(),
            vec![None, Some("desk-box"), Some("spot")],
            "every source still reports — dedup is over rows, never over answers"
        );
        assert_eq!(fold.completeness, RunsCompleteness::Settled);
    }

    /// Two ROBOTS echoing one id — a robot reachable under two announce names.
    /// First-seen wins there too, and nothing else moves.
    #[test]
    fn a_run_echoed_under_two_robot_names_is_listed_once_under_the_first() {
        let this_desk = LocalArm::Gathered(Box::new(settled("", Vec::new())));
        let fold = fold_runs(
            this_desk,
            clean_remote(vec![
                settled("go2", vec![entry(7, "nav", 500)]),
                settled(
                    "go2.local",
                    vec![entry(7, "nav", 500), entry(8, "arm", 600)],
                ),
            ]),
        );

        assert_eq!(
            attributed(&fold),
            vec![
                (Some("go2"), format_run_id(7).as_str()),
                (Some("go2.local"), format_run_id(8).as_str()),
            ]
        );
        assert_eq!(fold.sources.len(), 3);
    }

    /// ADVERSARIAL — an echo that DISAGREES with the first-hand row (a stale
    /// remote view carrying an older state and a different start time). The id
    /// is the identity; the first-hand row's metadata wins whole, never a merge,
    /// and nothing of the echo's payload leaks into the kept row.
    #[test]
    fn a_conflicting_echo_of_a_known_run_changes_nothing_about_the_kept_row() {
        let first_hand = entry(1, "perception", 100);
        let stale_echo = RunEntry::new(
            1,
            "perception-renamed",
            50,
            RunState::Ending,
            "name: perception-renamed\nnodes: [{ name: ghost }]\n".to_owned(),
            r#"{"run_id":"stale"}"#.to_owned(),
        )
        .expect("a hand-built entry is well formed");

        let fold = fold_runs(
            LocalArm::Gathered(Box::new(settled("", vec![first_hand.clone()]))),
            clean_remote(vec![settled("desk-box", vec![stale_echo])]),
        );

        assert_eq!(fold.runs.len(), 1);
        let kept = &fold.runs[0];
        assert_eq!(kept.robot, None);
        assert_eq!(kept.graph_name, first_hand.graph_name);
        assert_eq!(kept.run_started_at_ns, first_hand.run_started_at_ns);
        assert_eq!(kept.state, first_hand.state);
        assert_eq!(kept.graph_yaml, first_hand.graph_yaml);
        assert_eq!(kept.run_json, first_hand.run_json);
        assert_eq!(
            fold.sources.len(),
            2,
            "the disagreeing source still reports"
        );
        assert_eq!(fold.completeness, RunsCompleteness::Settled);
    }

    /// Determinism THROUGH the dedup branch: two folds of the same echoing
    /// inputs are bit-identical, and so is the dropped-echo choice.
    #[test]
    fn the_dedup_fold_is_deterministic() {
        let fold_echoing_inputs = || {
            fold_runs(
                LocalArm::Gathered(Box::new(settled("", vec![entry(1, "perception", 100)]))),
                clean_remote(vec![
                    settled("desk-box", vec![entry(1, "perception", 100)]),
                    settled("go2", vec![entry(7, "nav", 500)]),
                    settled("go2.local", vec![entry(7, "nav", 500)]),
                ]),
            )
        };
        let (a, b) = (fold_echoing_inputs(), fold_echoing_inputs());
        assert_eq!(a, b);
        assert_eq!(
            attributed(&a),
            vec![
                (None, format_run_id(1).as_str()),
                (Some("go2"), format_run_id(7).as_str()),
            ]
        );
    }

    /// NEITHER — the true empty, and the ONLY shape allowed to claim one.
    #[test]
    fn nothing_running_anywhere_is_a_settled_empty() {
        let fold = fold_runs(
            LocalArm::Gathered(Box::new(settled("", Vec::new()))),
            clean_remote(Vec::new()),
        );
        assert!(fold.runs.is_empty());
        assert_eq!(fold.completeness, RunsCompleteness::Settled);
        assert!(fold.unusable.is_empty());
        assert_eq!(fold.degraded, None);
        assert_eq!(fold.discovery, Some(DiscoveryLabel::Settled));
    }

    // ── The ways an answer is short, each with its own remedy ──────────────────

    /// A netd failure is `degraded` — the LOCAL half still stands, the remote half
    /// is UNKNOWN, and the verdict says so.
    ///
    /// The failure mode: swallow the error and report an empty-and-settled remote half.
    /// The local rows would look identical and the response would read as a
    /// confident "no robot is running anything" on a desk that could not ask.
    #[test]
    fn a_netd_failure_is_reported_as_degraded_and_never_as_an_empty_lan() {
        let local = LocalArm::Gathered(Box::new(settled("", vec![entry(1, "perception", 100)])));
        let fold = fold_runs(local, RemoteArm::Degraded("netd unreachable".into()));

        assert_eq!(fold.degraded.as_deref(), Some("netd unreachable"));
        assert_eq!(
            attributed(&fold),
            vec![(None, format_run_id(1).as_str())],
            "the local half of the answer is unaffected by a remote failure"
        );
        assert_ne!(
            fold.completeness,
            RunsCompleteness::Settled,
            "a desk that could not ask the LAN cannot claim the LAN is empty"
        );
        assert_eq!(
            fold.discovery, None,
            "and it reports no discovery state at all — UNKNOWN, never a claim"
        );
    }

    /// An UNUSABLE robot is surfaced as itself, and it demotes the verdict.
    ///
    /// This is the arm the whole B3→B4 hand-off exists for. Such a robot ANSWERED,
    /// so it is not a silence; it will re-serve the same undecodable bytes forever,
    /// so it is not a wait either. Note the discovery marker is `Settled` here on
    /// purpose — the one thing a renderer must NOT do is read this as "still
    /// discovering", and this fixture removes that excuse.
    #[test]
    fn a_robot_that_answered_unusably_is_surfaced_rather_than_folded_into_silence() {
        let fold = fold_runs(
            LocalArm::Gathered(Box::new(settled("", Vec::new()))),
            RemoteArm::Answered {
                replies: vec![settled("go2", vec![entry(1, "nav", 100)])],
                unusable: vec![UnusableRunsAnswer {
                    robot: "spot".into(),
                    reason: "runs reply version 2 is newer than this binary understands".into(),
                }],
                silent: Vec::new(),
                discovery: DiscoveryLabel::Settled,
            },
        );

        assert_eq!(fold.unusable.len(), 1);
        assert_eq!(fold.unusable[0].robot, "spot");
        assert!(
            fold.unusable[0].reason.contains("newer than this binary"),
            "the REASON travels — a bare count cannot be acted on"
        );
        assert_eq!(
            fold.discovery,
            Some(DiscoveryLabel::Settled),
            "discovery HAD converged, so nothing here may render as 'still \
             discovering' — the remedy is a redeploy"
        );
        assert_ne!(
            fold.completeness,
            RunsCompleteness::Settled,
            "a robot whose bytes we could not decode may be running anything, so \
             the answer is not an absence claim"
        );
        assert_eq!(
            attributed(&fold),
            vec![(Some("go2"), format_run_id(1).as_str())],
            "…while the robots that DID answer are still served"
        );
    }

    /// A robot that was ASKED and stayed SILENT is carried, and it FORBIDS the
    /// absence claim.
    ///
    /// **This is the coverage half, and the reason it cannot be left implicit.** A
    /// silent robot contributes no source, so its absence from `sources` is
    /// indistinguishable from a robot nobody asked about — which means a fold
    /// holding only replies and unusable answers reads a HALF-ANSWERED LAN
    /// identically to a fully-answered one and settles. That renders the silent
    /// robot's runs as "nothing is running there": the confident-empty class, one
    /// layer above the discovery latch that closes it on the netd side.
    ///
    /// Note the fixture holds discovery at SETTLED and nothing unusable, so the
    /// silence is the ONLY thing that can refuse the claim.
    #[test]
    fn a_robot_that_was_asked_and_stayed_silent_forbids_the_absence_claim() {
        let fold = fold_runs(
            LocalArm::Gathered(Box::new(settled("", Vec::new()))),
            RemoteArm::Answered {
                replies: vec![settled("go2", vec![entry(1, "nav", 100)])],
                unusable: Vec::new(),
                silent: vec!["spot".to_string()],
                discovery: DiscoveryLabel::Settled,
            },
        );

        assert_eq!(fold.silent, vec!["spot".to_string()]);
        assert!(
            fold.unusable.is_empty(),
            "a robot that said NOTHING is not a robot that said something \
             undecodable — the remedies differ (wait or upgrade vs redeploy)"
        );
        assert_eq!(
            fold.discovery,
            Some(DiscoveryLabel::Settled),
            "netd finished looking and asked this robot, so 'still discovering' \
             is not the correct rendering either"
        );
        assert_ne!(
            fold.completeness,
            RunsCompleteness::Settled,
            "the answer covers fewer machines than the question, so an empty list \
             cannot be read as 'nothing is running anywhere I can see'"
        );
        assert_eq!(
            attributed(&fold),
            vec![(Some("go2"), format_run_id(1).as_str())],
            "…while the robot that DID answer is still served in full"
        );
    }

    /// A robot that REFUSED this desk is a third thing again — an authorization
    /// remedy, carried per source, and never confusable with an absence.
    #[test]
    fn a_refusing_robot_carries_its_reason_and_forbids_the_absence_claim() {
        let fold = fold_runs(
            LocalArm::Gathered(Box::new(settled("", Vec::new()))),
            clean_remote(vec![RunsReply::refused("go2", "not paired with this desk")]),
        );

        let go2 = fold
            .sources
            .iter()
            .find(|s| s.robot.as_deref() == Some("go2"))
            .expect("the refusing robot is still a reported source");
        assert_eq!(go2.error.as_deref(), Some("not paired with this desk"));
        assert!(!go2.completeness.is_settled());
        assert!(fold.runs.is_empty());
        assert_ne!(fold.completeness, RunsCompleteness::Settled);
        assert!(
            fold.unusable.is_empty(),
            "a refusal is NOT an unusable answer — the remedy is a pairing grant, \
             not a redeploy, and collapsing them sends an operator to the wrong place"
        );
    }

    /// A run the source could not DESCRIBE rides through per source, and the
    /// assembler's demotion arrives with it.
    #[test]
    fn a_run_its_own_machine_could_not_describe_is_named_and_demotes_the_verdict() {
        let reply = build_runs_reply(
            "go2",
            vec![entry(1, "nav", 100)],
            vec![UndescribableRun {
                run_id: format_run_id(9),
                reason: "graph.yaml is not valid UTF-8".into(),
            }],
            // The GATHER settled — the shortfall is the unreadable directory, and
            // the assembler is what demotes for it.
            RunsCompleteness::Settled,
        );
        let fold = fold_runs(
            LocalArm::Gathered(Box::new(settled("", Vec::new()))),
            clean_remote(vec![reply]),
        );

        let go2 = fold
            .sources
            .iter()
            .find(|s| s.robot.as_deref() == Some("go2"))
            .expect("source");
        assert_eq!(go2.undescribable.len(), 1);
        assert_eq!(go2.undescribable[0].run_id, format_run_id(9));
        assert!(
            !go2.completeness.is_settled(),
            "the demotion comes from `build_runs_reply`, which owns that rule — the \
             fold deliberately does not re-derive it"
        );
        assert_ne!(fold.completeness, RunsCompleteness::Settled);
    }

    /// An UNCONVERGED discovery forbids the absence claim even when every source
    /// that answered settled — a robot may simply not have been asked yet.
    #[test]
    fn an_unconverged_discovery_forbids_the_absence_claim() {
        let fold = fold_runs(
            LocalArm::Gathered(Box::new(settled("", Vec::new()))),
            RemoteArm::Answered {
                replies: Vec::new(),
                unusable: Vec::new(),
                silent: Vec::new(),
                discovery: DiscoveryLabel::Discovering,
            },
        );
        assert_eq!(fold.discovery, Some(DiscoveryLabel::Discovering));
        assert_ne!(
            fold.completeness,
            RunsCompleteness::Settled,
            "netd has not finished looking, so an empty LAN proves nothing"
        );
    }

    /// A LOCAL gather that could not RUN is a source that established nothing —
    /// never an absent source, and never "this machine is running nothing".
    #[test]
    fn a_failed_local_gather_is_a_source_that_established_nothing() {
        let fold = fold_runs(
            LocalArm::Failed("shared memory unavailable".into()),
            clean_remote(Vec::new()),
        );
        assert_eq!(fold.sources.len(), 1);
        assert_eq!(fold.sources[0].robot, None);
        assert_eq!(
            fold.sources[0].error.as_deref(),
            Some("shared memory unavailable")
        );
        assert!(!fold.sources[0].completeness.is_settled());
        assert_ne!(fold.completeness, RunsCompleteness::Settled);
    }

    /// NOBODY contributed — the shape that must not read as an absence.
    ///
    /// A robot-scoped request whose robot never answered lands here: the local arm
    /// was deliberately skipped and the remote arm carried no replies. An empty
    /// `sources` is the discriminator, which is why the response carries the list
    /// rather than only a count.
    #[test]
    fn an_answer_no_source_contributed_to_is_silence_not_absence() {
        let fold = fold_runs(LocalArm::NotAsked, clean_remote(Vec::new()));
        assert!(fold.runs.is_empty());
        assert!(fold.sources.is_empty());
        assert_ne!(
            fold.completeness,
            RunsCompleteness::Settled,
            "no source supplied evidence, so there is nothing to read an empty list \
             against"
        );
    }

    /// A robot-scoped request runs NO local gather, and that is the attribution rule
    /// showing through rather than an omission.
    #[test]
    fn a_robot_scoped_request_contributes_no_local_rows() {
        let fold = fold_runs(
            LocalArm::NotAsked,
            clean_remote(vec![settled("go2", vec![entry(1, "nav", 100)])]),
        );
        assert_eq!(
            attributed(&fold),
            vec![(Some("go2"), format_run_id(1).as_str())]
        );
        assert!(
            fold.sources.iter().all(|s| s.robot.is_some()),
            "no local source appears at all — a local run is attributed to NO robot, \
             so no robot name could ever have selected one"
        );
    }

    /// The LOCAL gather's completeness travels WHOLE — its measured counts reach
    /// the source row rather than being re-expanded from a boolean.
    ///
    /// `local_reply` used to take `settled: bool` and rebuild the other side as
    /// [`RunsCompleteness::not_established`], whose counts are no-claim ZEROES
    /// meaning "no writer arithmetic backs this verdict". So a desk that HEARD 2 of
    /// 3 live run writers — a real, partial, measured picture — reported the same
    /// `(0, 0)` as one whose gather established nothing at all, and those are the
    /// two states this half can most usefully distinguish, since the local machine
    /// is the one an operator can go and look at. Every REMOTE source's counts
    /// already crossed the wire intact; the local one was the only source losing
    /// them.
    ///
    /// Driven through `local_reply` (not by hand) so the pin covers the seam the
    /// daemon actually calls, and the ANTI-TAUTOLOGY half is in the same body: a
    /// SETTLED gather must still settle, or "preserve the verdict" would be
    /// satisfied by never settling at all.
    #[test]
    fn the_local_gathers_measured_completeness_counts_reach_the_source_row() {
        let measured = RunsCompleteness::Incomplete {
            live_writers: 3,
            writers_heard: 2,
        };
        let fold = fold_runs(
            local_reply(RunFold::default(), measured),
            clean_remote(Vec::new()),
        );
        assert_eq!(fold.sources.len(), 1);
        assert_eq!(fold.sources[0].robot, None);
        assert_eq!(
            fold.sources[0].completeness, measured,
            "the registry's own arithmetic must survive — `not_established`'s \
             zeroes would report a partial gather as an unmeasured one"
        );
        assert_ne!(fold.completeness, RunsCompleteness::Settled);

        // ANTI-TAUTOLOGY: a settled gather still settles, so the arm above pins
        // the VERDICT travelling rather than a function that never settles.
        let fold = fold_runs(
            local_reply(RunFold::default(), RunsCompleteness::Settled),
            clean_remote(Vec::new()),
        );
        assert_eq!(fold.sources[0].completeness, RunsCompleteness::Settled);
        assert_eq!(fold.completeness, RunsCompleteness::Settled);
    }

    /// Determinism: the same inputs fold to the same answer, byte for byte.
    #[test]
    fn the_fold_is_deterministic() {
        let build = || {
            fold_runs(
                LocalArm::Gathered(Box::new(settled("", vec![entry(1, "perception", 100)]))),
                clean_remote(vec![settled("go2", vec![entry(2, "nav", 200)])]),
            )
        };
        assert_eq!(build(), build());
    }

    /// The aggregate rule, driven directly — one conjunct flipped at a time.
    ///
    /// The arms above each exercise it through a whole fold, which is the right
    /// shape for a behaviour but leaves a conjunct's INDEPENDENCE unproven: a rule
    /// that had silently collapsed two of its terms would still pass every one of
    /// them. Here each row differs from the settling row in exactly one place.
    #[test]
    fn every_conjunct_of_the_settle_rule_is_independently_load_bearing() {
        let settled_source = RunsSourceStatus {
            robot: None,
            completeness: RunsCompleteness::Settled,
            error: None,
            undescribable: Vec::new(),
        };
        let unsettled_source = RunsSourceStatus {
            completeness: RunsCompleteness::not_established(),
            ..settled_source.clone()
        };
        let bad = vec![UnusableRunsAnswer {
            robot: "spot".into(),
            reason: "skew".into(),
        }];

        // The one shape that settles.
        assert!(sources_settle(
            std::slice::from_ref(&settled_source),
            &[],
            &[],
            true,
            Some(DiscoveryLabel::Settled)
        ));
        // …and each single departure from it.
        assert!(
            !sources_settle(&[], &[], &[], true, Some(DiscoveryLabel::Settled)),
            "no source contributed"
        );
        assert!(
            !sources_settle(
                &[unsettled_source],
                &[],
                &[],
                true,
                Some(DiscoveryLabel::Settled)
            ),
            "a source did not settle"
        );
        assert!(
            !sources_settle(
                std::slice::from_ref(&settled_source),
                &bad,
                &[],
                true,
                Some(DiscoveryLabel::Settled)
            ),
            "a robot answered unusably"
        );
        assert!(
            !sources_settle(
                std::slice::from_ref(&settled_source),
                &[],
                &["spot".to_string()],
                true,
                Some(DiscoveryLabel::Settled)
            ),
            "an asked robot stayed silent"
        );
        assert!(
            !sources_settle(
                std::slice::from_ref(&settled_source),
                &[],
                &[],
                false,
                Some(DiscoveryLabel::Settled)
            ),
            "the remote arm never ran"
        );
        assert!(
            !sources_settle(
                std::slice::from_ref(&settled_source),
                &[],
                &[],
                true,
                Some(DiscoveryLabel::Discovering)
            ),
            "discovery had not converged"
        );
        assert!(
            !sources_settle(std::slice::from_ref(&settled_source), &[], &[], true, None),
            "discovery state UNKNOWN is not a licence either"
        );
    }
}
