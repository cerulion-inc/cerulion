// SPDX-License-Identifier: AGPL-3.0-only
//! The one always-on capture plane, over real POSIX SHM.
//!
//! The policy's DECISIONS are oracle-tested where they are pure
//! (`cerulion_core::flashback`'s in-module tests: the kill-switch table, the
//! cadence arithmetic, the projection, the gate's boundaries on both sides). This
//! file pins the thing those cannot see — that the decision is WIRED to a real
//! consequence:
//!
//! * a plain run really does create an armed word a peer can read, at a cadence
//!   derived from that graph's quantum, and a real state ring under the derived
//!   `(tag, rank)` name;
//! * `CERULION_FLASHBACK=off` really does leave the SHM namespace untouched — no
//!   word, no ring — and says so out loud;
//! * the RAM gate really does refuse, with a message an operator can act on.
//!
//! The oracle for every "did not arm" arm is the SHM OBJECT, not the return
//! value. A verdict that returned `None` while still creating the word would
//! satisfy an `is_none()` assertion and charge every robot for the feature it
//! just declined — which is the same class of bug
//! `a_failed_attach_creates_no_state_ring` was written for one layer down.
//!
//! `#[serial]` throughout: every test mutates process-global environment, and the
//! plane's whole policy is read from it.
#![cfg(unix)]

use cerulion_cli_engine::state_arm_attach::{admit_capture_plane, arm_capture_plane, PlaneRole};

/// PURE: only the roles that actually `fork(2)` are subject to the memory gate.
///
/// A one-line table, and it earns its place because BOTH directions of getting it
/// wrong shipped once and both are silent: gating a non-forking process disabled
/// capture for processes that were fine (the supervisor), and skipping a forking one
/// left the operator's ceiling unenforced on exactly the process it was set for (the
/// worker).
#[test]
fn only_the_roles_that_fork_are_gated() {
    assert!(
        PlaneRole::Monolith.pays_the_fork_cost(),
        "it anchors itself"
    );
    assert!(
        PlaneRole::Worker { rank: 0 }.pays_the_fork_cost(),
        "a worker is THE process that forks under a multi-process split"
    );
    assert!(
        !PlaneRole::Supervisor.pays_the_fork_cost(),
        "a supervisor holds the word and never anchors — it installs no state-ring \
         producer, and `take_anchor` returns at its first line without one"
    );
}

/// THE HEADLINE: a SUPERVISOR over the ceiling still arms the plane, so
/// its workers — which may all fit — are not silenced by a process that never forks.
///
/// The mirror of the worker arm, and the same test shape: the ceiling is
/// forced to 1 MiB and this process is put over it (`over_the_forced_ceiling`). A
/// monolith and a worker are refused at that same ceiling in the same body, so this
/// is a claim about the ROLE rather than about the ceiling failing to bite.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn a_supervisor_over_the_ceiling_still_arms_because_it_never_forks() {
    let _env = clean_env();
    let _low = over_the_forced_ceiling();
    let tag = unique_tag("supervisor_gate");

    // THE HEADLINE.
    let arm = arm_capture_plane(PlaneRole::Supervisor, &tag, Some(1_000_000));
    assert!(
        arm.is_some(),
        "a supervisor must arm the deployment's plane whatever ITS own memory is — it \
         holds the word and anchors nothing, and refusing here silences every worker"
    );
    assert!(plane_exists(&tag), "…and the word must really exist");
    assert!(
        !logs_contain("Flashback STATE capture is OFF"),
        "…with no refusal reported, because none applies"
    );
    // …and the ARM report must not price a copy this process never makes. Quoting a
    // supervisor's own resident memory as the cost of an anchor is the same
    // misleading-surface class as gating on it, one layer up in the log.
    assert!(
        logs_contain("holds the word and anchors nothing"),
        "a supervisor's arm report must say whose memory actually pays"
    );
    assert!(
        !logs_contain("copy of this process's private memory"),
        "…and must NOT borrow the monolith's wording, which would describe a copy that \
         never happens"
    );

    // ANTI-TAUTOLOGY, in the same body at the same ceiling: the two roles that DO
    // fork are refused. Without this the arm above would pass a gate that had
    // simply stopped working.
    assert!(
        admit_capture_plane(PlaneRole::Monolith, &tag, Some(1_000_000)).is_none(),
        "a monolith anchors itself, so the ceiling must still bite"
    );
    assert!(
        admit_capture_plane(PlaneRole::Worker { rank: 2 }, &tag, Some(1_000_000)).is_none(),
        "and so must a worker"
    );
}

/// THE ALWAYS-ON CONTRACT, behaviourally: a run whose DESCRIPTION cannot be
/// written still gets a real capture plane.
///
/// The run registry degrades run-descriptor creation on purpose — a read-only home or a
/// full disk yields one warn and `None`, and the graph runs. An arm tag
/// derived from the descriptor's `run_id` would make those hosts silently lose the
/// always-on black box because a bookkeeping file could not be
/// created. The identity is minted first and independently, so it survives.
///
/// Driven through the REAL failure the production path degrades over — a
/// `CERULION_HOME` that is a FILE, so every `create_dir_all` beneath it fails on
/// every platform (the idiom `run_dir`'s own tests use) — and the oracle is the
/// SHM OBJECT, as everywhere else in this file: a plane that returned `Some`
/// while creating nothing would pass a return-value assertion and still leave the
/// robot with no black box.
///
/// `graph_run`'s WIRING (that it derives the tag from the mint and mints before
/// it writes) is pinned structurally in `state_arm_adoption_test`; this is the
/// half that proves the pieces really compose over real SHM.
#[test]
#[serial_test::serial]
fn a_run_whose_descriptor_cannot_be_written_still_arms_its_plane() {
    let _env = clean_env();
    let tmp = tempfile::tempdir().expect("tempdir");
    // A FILE where the run root's parent must be a directory.
    let blocker = tmp.path().join("not_a_dir");
    std::fs::write(&blocker, b"x").expect("write blocker");
    let _home = EnvGuard::set("CERULION_HOME", blocker.to_str().expect("utf-8 path"));

    // The identity is minted FIRST — the whole point — and then the description
    // is attempted and fails.
    let run_id = cerulion_cli_engine::run_dir::mint_run_id();
    assert!(
        cerulion_cli_engine::run_dir::begin_run_descriptor(
            cerulion_cli_engine::run_dir::RunDescriptorSpec {
                chains: None,
                run_id,
                graph_name: "undescribable",
                run_started_at_ns: 1,
                network: cerulion_cli_engine::run_dir::NetworkPostureLabel::Off,
                partition: cerulion_cli_engine::run_dir::PartitionProvenance::Declared,
                process_groups: false,
                shm: Vec::new(),
                gating: cerulion_cli_engine::run_dir::GatingClock::Wall,
                graph_yaml: "name: undescribable\n".to_string(),
                env_json: b"{}".to_vec(),
                recorder_json: b"{}".to_vec(),
            }
        )
        .is_none(),
        "precondition: this run really is undescribable — otherwise the arm below \
         proves nothing"
    );

    // THE HEADLINE: the run is still IDENTIFIED, so it still has a plane.
    let (tag, source) = cerulion_cli_engine::state_arm_attach::state_arm_tag_for_process(Some(
        run_id,
    ))
    .expect("a run that could not be described is still a run, and must still have an arm tag");

    // …and it is THIS RUN's tag. "Some tag was returned" is far too weak on its
    // own: a resolver that ignored `run_id` and answered a constant, or one that
    // read an inherited `CERULION_STATE_ARM_TAG`, satisfies it — and then the
    // assertions below arm that tag and find that tag, so the whole test passes
    // with run-id derivation completely broken. Three independent properties,
    // because each catches a different way of being wrong:
    //
    // (1) PROVENANCE — it was DERIVED, not taken from an override. `clean_env`
    //     clears that variable, so this also fails loudly if the clearing ever
    //     stops working rather than degrading into a vacuous pass.
    assert_eq!(
        source,
        cerulion_cli_engine::state_arm_attach::ArmTagSource::DerivedFromRun,
        "the tag must be DERIVED from the run id — an explicit override resolving here \
         means this test is measuring somebody's environment, not the derivation"
    );
    // (2) CORRESPONDENCE — the run id is really IN the name, rendered as the 32
    //     hex digits the tag format uses. Hand-computed from the identity rather
    //     than compared against `state_arm_tag_for_run`, which would be the same
    //     function answering for itself.
    assert!(
        tag.contains(&format!("{run_id:032x}")),
        "the tag must carry THIS run's id: expected {run_id:032x} inside {tag:?} — a tag \
         that does not name the run cannot be found by a peer that knows only the run"
    );
    // (3) DISCRIMINATION — different runs get different names, and one run gets a
    //     stable one. A tag that embedded something random would pass (2) on a
    //     lucky substring but cannot pass both halves of this.
    let other_id = cerulion_cli_engine::run_dir::mint_run_id();
    assert_ne!(other_id, run_id, "precondition: two mints really differ");
    let (other_tag, _) =
        cerulion_cli_engine::state_arm_attach::state_arm_tag_for_process(Some(other_id))
            .expect("any run resolves a tag");
    assert_ne!(
        other_tag, tag,
        "two runs must not share one plane — a tag that ignored the run id would \
         collide every concurrent run on this machine onto one word"
    );
    assert_eq!(
        cerulion_cli_engine::state_arm_attach::state_arm_tag_for_process(Some(run_id))
            .expect("the same run resolves again")
            .0,
        tag,
        "…and one run must resolve the SAME name twice, or the recorder and the graph \
         would open two different words"
    );

    let arm = arm_capture_plane(PlaneRole::Monolith, &tag, Some(1_000_000))
        .expect("…and its plane must really arm");
    assert!(
        plane_exists(&tag),
        "the ORACLE: the arm word really exists in SHM — a run whose bookkeeping failed \
         must still have a black box"
    );
    drop(arm);
}

/// A supervisor is still subject to RUN-LEVEL policy: the kill switch stops it.
///
/// The boundary of the exemption above — it is about the MEMORY gate, which prices a
/// fork this process will not do, not about whether a supervisor may ignore an
/// operator who turned the feature off.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn the_kill_switch_still_stops_a_supervisor() {
    let _env = clean_env();
    let _off = EnvGuard::set(cerulion_core::flashback::FLASHBACK_ENV, "off");
    let tag = unique_tag("supervisor_killswitch");
    assert!(
        arm_capture_plane(PlaneRole::Supervisor, &tag, Some(1_000_000)).is_none(),
        "the kill switch is run-level policy and the supervisor is where it is decided"
    );
    assert!(!plane_exists(&tag), "and nothing is created");
}
use cerulion_core::state_arm::{state_arm_shm_name, MappedStateArm};

/// RAII: set (or clear) an env var for one test and restore it on the way out,
/// INCLUDING on a panic — a leaked `CERULION_FLASHBACK=off` would silently
/// disarm every sibling test in this binary and every assertion below is about
/// exactly that variable.
struct EnvGuard {
    key: &'static str,
    prior: Option<String>,
}

impl EnvGuard {
    fn set(key: &'static str, value: &str) -> Self {
        let prior = std::env::var(key).ok();
        std::env::set_var(key, value);
        Self { key, prior }
    }

    fn cleared(key: &'static str) -> Self {
        let prior = std::env::var(key).ok();
        std::env::remove_var(key);
        Self { key, prior }
    }
}

impl Drop for EnvGuard {
    fn drop(&mut self) {
        match &self.prior {
            Some(v) => std::env::set_var(self.key, v),
            None => std::env::remove_var(self.key),
        }
    }
}

/// PRECONDITION shared by every ARMING arm in this file: the machine has the
/// `CAPTURE_MEM_FLOOR_BYTES` (512 MiB) of headroom the gate's second arm requires.
/// A build machine running a Rust compile has it by a wide margin; a machine that
/// genuinely does not is one the gate is RIGHT to refuse on, so the failure would
/// be reporting a true fact rather than a flake.
///
/// A tag no other test or run can collide with.
fn unique_tag(label: &str) -> String {
    format!(
        "{label}_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("clock")
            .as_nanos()
    )
}

/// Does the arm word for `tag` exist in the SHM namespace?
///
/// THE oracle for every refusal arm: a plane that was declined must leave nothing
/// behind, and only the object can say so.
fn plane_exists(tag: &str) -> bool {
    cerulion_core::shm_ring::shm_object_exists(&state_arm_shm_name(tag))
}

/// The forced ceiling every REFUSAL arm in this file runs under, in mebibytes.
///
/// The smallest the knob can express, and therefore the smallest footprint this
/// process can be asked to exceed.
const FORCED_CEILING_MB: u64 = 1;

/// Anonymous memory this process holds for the rest of its life, so that
/// [`FORCED_CEILING_MB`] really is below its private-anon footprint.
///
/// Never freed, deliberately: the ballast has to be resident at the moment EVERY
/// refusal arm asks, and libtest chooses the order those arms run in.
static ANON_BALLAST: std::sync::OnceLock<Vec<u8>> = std::sync::OnceLock::new();

/// Eight times the forced ceiling — wide enough that no baseline or allocator
/// bookkeeping decides the comparison, small enough to be free on any runner.
const BALLAST_BYTES: usize = 8 * 1024 * 1024;

/// Force the state ceiling to [`FORCED_CEILING_MB`] AND make this process really
/// exceed it — the precondition every refusal arm here rests on, ESTABLISHED
/// rather than assumed.
///
/// # Why lowering the ceiling alone is not enough
///
/// These arms turn the ceiling DOWN instead of allocating past a real one, because
/// the gate's input is this process's own memory and staging a 673 MiB state would
/// be measuring the allocator. That much is right. What does not follow is the
/// claim that "turning the ceiling down to 1 MiB puts the
/// running test binary over it" — nothing establishes it, and it is FALSE on
/// Linux early in the binary's life: the gate prices `RssAnon`, PRIVATE ANONYMOUS
/// resident memory (`sample_anon_rss`), and a freshly-started test process has well
/// under 1 MiB of it. It climbs past the ceiling as siblings allocate, so whichever
/// refusal arm libtest schedules FIRST is admitted and fails, while every later
/// arm asserting the identical thing passes.
///
/// The failure is deterministic, not timing: it lands on the
/// alphabetically-first refusal arm each time, and a RENAME alone
/// (`an_owners_refusal…` → `a_monoliths_refusal…`, which sorts earlier) moves the
/// failure from `a_state_too_large_for_the_budget_refuses_loudly_and_prices_it` to
/// `a_monoliths_refusal_states_the_whole_run_is_unresimmable`. macOS never shows it
/// because `sample_anon_rss` there reports TOTAL resident memory, which is
/// megabytes before `main` runs — so the whole class is invisible on macOS.
///
/// The precondition is therefore MADE and then CHECKED. Every behavioural
/// assertion stands as written: this establishes what they rest on.
fn over_the_forced_ceiling() -> EnvGuard {
    // WRITTEN, not merely reserved, and with a NON-ZERO byte: `vec![0u8; n]` is
    // served by `alloc_zeroed`, which for a fresh mapping is the kernel's shared
    // zero page — reserved but not resident, and so not counted in `RssAnon`, which
    // is the one number this has to move. A non-zero fill memsets every page in.
    let ballast = ANON_BALLAST.get_or_init(|| vec![0xA5u8; BALLAST_BYTES]);
    assert_eq!(
        ballast.len(),
        BALLAST_BYTES,
        "the ballast must still be held — freeing it would return the pages"
    );

    // ATTRIBUTABLE: on a platform that can report the figure, fail HERE and say
    // which number was wrong, rather than letting a false precondition surface as a
    // mystified `is_none()` failure in whichever arm happened to run first.
    if let Some(anon) = cerulion_core::state_carrier::fork::sample_anon_rss() {
        assert!(
            anon > FORCED_CEILING_MB * 1024 * 1024,
            "PRECONDITION: this process must be OVER the forced {FORCED_CEILING_MB} MiB \
             ceiling for a refusal to mean anything, but its private-anon footprint \
             measures {anon} B"
        );
    }

    EnvGuard::set(
        cerulion_core::flashback::FLASHBACK_MAX_STATE_MB_ENV,
        &FORCED_CEILING_MB.to_string(),
    )
}

/// A clean environment for every variable that can decide this plane's identity
/// or policy, so a test states only what it varies and no inherited value can
/// decide an assertion.
///
/// `CERULION_STATE_ARM_TAG` is in this list for a reason worth stating: it is not
/// a knob like the other three, it is an OVERRIDE that wins outright
/// (`resolve_arm_tag` returns it before it ever looks at the run id). A developer
/// or CI job with one exported would silently redirect every DERIVED-identity
/// assertion in this file onto their tag — and those assertions would still PASS,
/// because each arms the tag it was handed and then checks that same tag. The
/// derivation could be entirely broken and nothing here would notice. Clearing it
/// is what makes "derived" mean derived.
fn clean_env() -> Vec<EnvGuard> {
    vec![
        EnvGuard::cleared(cerulion_core::flashback::FLASHBACK_ENV),
        EnvGuard::cleared(cerulion_core::flashback::FLASHBACK_CADENCE_MS_ENV),
        EnvGuard::cleared(cerulion_core::flashback::FLASHBACK_MAX_STATE_MB_ENV),
        EnvGuard::cleared(cerulion_cli_engine::state_arm_attach::STATE_ARM_TAG_ENV),
    ]
}

/// Assert EXACTLY one captured line carries `$marker`, AND that it was emitted at
/// `$level`.
///
/// The level is half the claim and a message-only predicate cannot see it.
/// `tracing_test` captures at TRACE, so a refusal demoted to `debug!` — invisible
/// at a robot's default filter, and therefore invisible to the operator it exists
/// for — still contains its own text and would satisfy `logs_contain`. Matching
/// the level as a whole whitespace TOKEN (rather than a substring) keeps the
/// rendered span name, which is the test function's own name, from deciding it.
///
/// A MACRO rather than a function because `logs_assert` is injected into the body
/// of a `#[traced_test]` fn and does not exist outside one.
macro_rules! assert_line_at {
    ($level:expr, $marker:expr, $why:expr) => {
        logs_assert(|lines: &[&str]| {
            let hits: Vec<&&str> = lines.iter().filter(|l| l.contains($marker)).collect();
            if hits.len() != 1 {
                return Err(format!(
                    "{}: expected exactly ONE line containing {:?}, got {} in:\n{}",
                    $why,
                    $marker,
                    hits.len(),
                    lines.join("\n")
                ));
            }
            let line = hits[0];
            // The level rides the line HEADER, before the fields; take whole
            // tokens so a field VALUE that happens to spell a level cannot
            // satisfy this.
            if !line.split_whitespace().any(|t| t == $level) {
                return Err(format!(
                    "{}: the line exists but not at {} — a demoted line is invisible at a \
                     robot's default filter:\n{}",
                    $why, $level, line
                ));
            }
            Ok(())
        });
    };
}

/// Assert a captured line carries `$marker` AND the structured field `$key=$value`
/// as a WHOLE WHITESPACE TOKEN.
///
/// The `has_field` lesson, and it is not hypothetical here: the warning
/// this is used on says "the only value that turns it off is `off`", and `"off"`
/// CONTAINS `"of"` — so a substring assertion for the offending value `of` is
/// satisfied by the message's own prose no matter what the field says. A token
/// match also separates `value=of` from `value=off`, which a prefix match cannot.
macro_rules! assert_field_at {
    ($level:expr, $marker:expr, $key:expr, $value:expr, $why:expr) => {
        logs_assert(|lines: &[&str]| {
            let want = format!("{}={}", $key, $value);
            let hits: Vec<&&str> = lines.iter().filter(|l| l.contains($marker)).collect();
            if hits.len() != 1 {
                return Err(format!(
                    "{}: expected exactly ONE line containing {:?}, got {} in:\n{}",
                    $why,
                    $marker,
                    hits.len(),
                    lines.join("\n")
                ));
            }
            let line = hits[0];
            if !line.split_whitespace().any(|t| t == $level) {
                return Err(format!(
                    "{}: the line is not at {}:\n{}",
                    $why, $level, line
                ));
            }
            // WHOLE TOKEN: `value=of` must not be satisfied by `value=off`, and
            // the field must not be satisfied by prose that merely spells it.
            if !line.split_whitespace().any(|t| t == want) {
                return Err(format!(
                    "{}: no whitespace token equals {:?} — the value is missing or \
                     misreported:\n{}",
                    $why, want, line
                ));
            }
            Ok(())
        });
    };
}

/// A worker runs the same gate, on its own memory,
/// and its refusal names the rank and the consequence.
///
/// # Why this arm exists at all
///
/// The gate prices a `fork(2)` of the CALLING process. Under a multi-process split the
/// process that forks is each worker, while the process that ARMS is the
/// supervisor — a few megabytes of spawn bookkeeping. Evaluating only at the
/// supervisor therefore measures the wrong process: a 1 MiB
/// supervisor can arm a plane that a 129 MiB worker then opens, under a 64 MiB
/// ceiling, i.e. the operator's limit silently unenforced on every process it was
/// set for.
///
/// The behavioural half — that the gate REACHES a worker at all — is what this
/// pins; the fact that the worker seam CALLS it is pinned structurally in
/// `state_arm_adoption_test`, because reproducing a real oversized worker means
/// spawning a supervisor and faulting in memory in a child.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn a_worker_over_the_ceiling_declines_for_itself_and_says_which_rank() {
    let _env = clean_env();
    let _low = over_the_forced_ceiling();
    let tag = unique_tag("worker_gate");

    assert!(
        admit_capture_plane(PlaneRole::Worker { rank: 3 }, &tag, Some(1_000_000)).is_none(),
        "a worker past the ceiling must decline for itself"
    );
    // The RANK, as a whole token: a refusal an operator cannot attribute to a rank
    // is unactionable on a deployment with a dozen of them.
    assert_field_at!(
        "WARN",
        "Flashback STATE capture is OFF",
        "rank",
        "Some(3)",
        "a worker's refusal must name WHICH rank declined"
    );
    assert!(
        logs_contain("its peers carry on"),
        "…and must say the run keeps going without it"
    );
    assert!(
        logs_contain("reported PARTIAL"),
        "…and what that costs: every anchor of the run becomes partial (§7.3)"
    );
}

/// The MONOLITH's refusal says something DIFFERENT, because a different thing is lost.
///
/// Paired with the arm above deliberately: one shared message for both roles would
/// tell a supervisor's operator that "peers carry on" when there are no peers, and
/// a worker's that the whole run is unresimmable when only one rank is missing.
/// Both are wrong in the direction that wastes the reader's time.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn a_monoliths_refusal_states_the_whole_run_is_unresimmable() {
    let _env = clean_env();
    let _low = over_the_forced_ceiling();
    let tag = unique_tag("owner_gate");

    assert!(admit_capture_plane(PlaneRole::Monolith, &tag, Some(1_000_000)).is_none());
    assert!(
        logs_contain("NO capture plane at all"),
        "a monolith's refusal is about the whole run"
    );
    assert!(
        logs_contain("NOT resimmable"),
        "…and names what a bag of it will lack"
    );
    assert!(
        !logs_contain("its peers carry on"),
        "…and must NOT borrow the worker's wording: there are no peers"
    );
}

/// ANTI-TAUTOLOGY for both: with a ceiling this process fits under, BOTH roles are
/// admitted and neither says anything.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn neither_role_is_refused_inside_the_ceiling() {
    let _env = clean_env();
    let _high = EnvGuard::set(
        cerulion_core::flashback::FLASHBACK_MAX_STATE_MB_ENV,
        "65536",
    );
    let tag = unique_tag("both_roles_ok");
    assert!(admit_capture_plane(PlaneRole::Monolith, &tag, Some(1_000_000)).is_some());
    assert!(admit_capture_plane(PlaneRole::Worker { rank: 1 }, &tag, Some(1_000_000)).is_some());
    assert!(
        !logs_contain("Flashback STATE capture is OFF"),
        "a healthy process must be silent — otherwise the 'exactly one line' oracles \
         above would be measuring noise"
    );
}

/// THE HEADLINE: a run nobody armed by hand comes up with a live capture plane, at
/// a cadence derived from its own quantum.
///
/// Every clause is a separate way this could ship inert. A word that exists but is
/// not ARMED makes `due()` false at every step and captures nothing; a cadence
/// derived from the wrong quantum anchors at the wrong logical rate; and a
/// `first_anchor_step` past 0 would throw away step 0, the free
/// and perfect anchor (recording and resim both build every node through
/// `Default::default()`).
#[test]
#[serial_test::serial]
fn a_plain_run_arms_a_live_plane_at_the_derived_cadence() {
    let _env = clean_env();
    let tag = unique_tag("plain");
    assert!(!plane_exists(&tag), "precondition: the name starts unused");

    // A 1 kHz graph: the quantum a `period_ms = 1` node produces.
    let arm = arm_capture_plane(PlaneRole::Monolith, &tag, Some(1_000_000))
        .expect("an ordinary run arms its plane");

    // Read through a PEER mapping — what a worker of this run really does — so
    // the assertion is on the cross-process seam, not on our own handle.
    let peer = MappedStateArm::open_unowned(&tag).expect("a peer opens the word");
    assert!(peer.is_armed(), "an un-armed word captures nothing");
    assert_eq!(
        peer.cadence_steps(),
        15_000,
        "15 s of logical time at a 1 ms quantum is 15 000 steps"
    );
    assert_eq!(
        peer.first_anchor_step(),
        0,
        "step 0 is the run's free and perfect anchor (§8.1)"
    );
    // The seam the boundary actually calls.
    assert!(peer.due(0), "step 0 is due");
    assert!(!peer.due(1), "a step between boundaries is not");
    assert!(peer.due(15_000), "and every cadence after it");

    drop(peer);
    drop(arm);
    assert!(
        !plane_exists(&tag),
        "the plane's name dies with the run that owns it"
    );
}

/// The cadence is DERIVED from the graph, not fixed — a 100 Hz graph anchors at
/// the same logical INTERVAL and a tenth of the step count.
///
/// Its own arm because it is the claim most easily satisfied by a constant: a
/// plane that hardcoded 15 000 steps passes the headline above and anchors 100×
/// too rarely here.
#[test]
#[serial_test::serial]
fn the_cadence_follows_the_graphs_own_quantum() {
    let _env = clean_env();
    let tag = unique_tag("cadence");
    let arm = arm_capture_plane(PlaneRole::Monolith, &tag, Some(10_000_000)).expect("arms");
    assert_eq!(arm.cadence_steps(), 1_500, "15 s at a 10 ms quantum");
    drop(arm);

    // A graph that declares NO timing at all uses the same 1 ms heartbeat floor
    // the gating clock does, so its cadence is a real number of ITS steps.
    let tag = unique_tag("cadence_none");
    let arm = arm_capture_plane(PlaneRole::Monolith, &tag, None).expect("arms");
    assert_eq!(arm.cadence_steps(), 15_000, "the 1 ms floor");
}

/// The cadence override is honoured — the one lever on the standing cost.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn the_cadence_override_is_honoured() {
    let _env = clean_env();
    let _set = EnvGuard::set(cerulion_core::flashback::FLASHBACK_CADENCE_MS_ENV, "60000");
    let tag = unique_tag("cadence_env");
    let arm = arm_capture_plane(PlaneRole::Monolith, &tag, Some(1_000_000)).expect("arms");
    assert_eq!(
        arm.cadence_steps(),
        60_000,
        "a minute of logical time at a 1 ms quantum — the operator's number, not the default"
    );
    // …and the ARM-TIME report carries the millisecond figure they set, beside the
    // step count the boundary uses. An operator checking whether their override took
    // effect reads the milliseconds; the steps mean nothing to them without the
    // graph's quantum in hand.
    assert_field_at!(
        "INFO",
        "capture plane ARMED",
        "cadence_ms",
        "60000",
        "the arm report must echo the cadence the operator actually set"
    );
    assert_field_at!(
        "INFO",
        "capture plane ARMED",
        "cadence_steps",
        "60000",
        "…alongside the step count the word really carries"
    );
}

/// THE KILL SWITCH: `CERULION_FLASHBACK=off` leaves the SHM namespace untouched,
/// and says so.
///
/// The `info!` is not decoration. An operator debugging a run that produced no
/// anchors has exactly one way to discover that they are the reason, and it is
/// this line in that run's own log — the plane is otherwise indistinguishable
/// from one the RAM gate refused or one whose word failed to create.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn the_kill_switch_arms_nothing_and_is_loud_about_it() {
    let _env = clean_env();
    let _off = EnvGuard::set(cerulion_core::flashback::FLASHBACK_ENV, "off");
    let tag = unique_tag("killswitch");

    assert!(
        arm_capture_plane(PlaneRole::Monolith, &tag, Some(1_000_000)).is_none(),
        "the kill switch must arm nothing"
    );
    // THE ORACLE: nothing was created. A verdict that returned `None` after
    // creating the word would pass the line above and charge the robot anyway.
    assert!(
        !plane_exists(&tag),
        "a disabled plane must leave NO arm word behind"
    );
    assert_line_at!(
        "INFO",
        "Flashback is OFF for this run",
        "an operator who turned it off must see that in the run's own log, at a level \
         a robot actually prints"
    );
    assert!(
        logs_contain("CERULION_FLASHBACK"),
        "…named, so they know which variable to unset"
    );
}

/// ANTI-TAUTOLOGY for the kill switch: with the variable UNSET, the same call
/// arms — so the test above is about the switch and not about the harness.
#[test]
#[serial_test::serial]
fn an_unset_kill_switch_arms_normally() {
    let _env = clean_env();
    let tag = unique_tag("killswitch_control");
    let arm = arm_capture_plane(PlaneRole::Monolith, &tag, Some(1_000_000));
    assert!(arm.is_some(), "no switch, no refusal");
    assert!(plane_exists(&tag), "…and the word really exists");
}

/// A value the switch cannot read leaves the plane ON and WARNS.
///
/// The safe direction and the loud one, together: a typo must not silently
/// disable a safety feature, and must not silently be ignored either. `of` is one
/// keystroke from `off`, which is exactly how an operator reaches this.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn an_unreadable_kill_switch_value_stays_on_and_warns() {
    let _env = clean_env();
    let _bad = EnvGuard::set(cerulion_core::flashback::FLASHBACK_ENV, "of");
    let tag = unique_tag("killswitch_typo");

    assert!(
        arm_capture_plane(PlaneRole::Monolith, &tag, Some(1_000_000)).is_some(),
        "an unreadable instruction must not disable the plane"
    );
    // The LEVEL, the message, AND the offending value — the last as a whole token,
    // because the warning's own prose names `off` and `"off".contains("of")`, so a
    // substring assertion here proves nothing at all (thread 3).
    assert_field_at!(
        "WARN",
        "unrecognised value",
        "value",
        "of",
        "a switch the machine could not read is a divergence between what the operator \
         believes and what is running — the warning must be loud AND must quote back \
         what they actually typed"
    );
}

/// THE RAM GATE: a ceiling this process cannot fit under refuses the plane, leaves
/// nothing behind, and prices the refusal.
///
/// Forced low rather than staged large: the gate's input is this process's own
/// resident anonymous memory, and a test that tried to ALLOCATE past the REAL
/// 673 MiB ceiling would be measuring the allocator. Turning the ceiling down to
/// 1 MiB drives the identical decision through the identical code path — and the
/// override is itself part of the contract, so exercising it is not a shortcut.
/// Being over that ceiling is then MADE TRUE rather than assumed, by
/// `over_the_forced_ceiling`; see its docs for how the bare assumption fails.
///
/// The message assertions are the point. A refusal an operator cannot act on
/// leaves them with a robot that has no black box and no idea why, so the line
/// must carry what was measured, what it would have cost, the ceiling, and the
/// variable that moves it.
#[test]
#[serial_test::serial]
#[tracing_test::traced_test]
fn a_state_too_large_for_the_budget_refuses_loudly_and_prices_it() {
    let _env = clean_env();
    let _low = over_the_forced_ceiling();
    let tag = unique_tag("ramgate");

    assert!(
        arm_capture_plane(PlaneRole::Monolith, &tag, Some(1_000_000)).is_none(),
        "a process past the ceiling must not arm"
    );
    assert!(
        !plane_exists(&tag),
        "a refused plane must leave NO arm word behind"
    );
    assert_line_at!(
        "WARN",
        "Flashback STATE capture is OFF",
        "the refusal must be legible as a refusal, at a level that survives a robot's \
         default filter — this is the only notice the operator ever gets"
    );
    assert!(
        logs_contain("stall_budget_ms=250"),
        "…carrying the budget it was measured against"
    );
    assert!(
        logs_contain("max_state_mb=1"),
        "…the ceiling in force, which is what the operator just set"
    );
    assert!(
        logs_contain("CERULION_FLASHBACK_MAX_STATE_MB"),
        "…and the variable that moves it — a refusal nobody can act on is barely \
         better than a silent one"
    );
    assert!(
        logs_contain("projected_stall_ms"),
        "…and the price, which is the number the decision was actually made on"
    );
}

/// ANTI-TAUTOLOGY for the gate: a ceiling this process fits under arms.
///
/// Without it, the arm above would be satisfied by a gate that refused
/// unconditionally — a real bug, and one whose only symptom on a
/// healthy robot is a black box that never records anything.
#[test]
#[serial_test::serial]
fn a_state_inside_the_budget_arms() {
    let _env = clean_env();
    // Well above any test binary's resident set, well below anything that would
    // trip the headroom arm.
    let _high = EnvGuard::set(
        cerulion_core::flashback::FLASHBACK_MAX_STATE_MB_ENV,
        "65536",
    );
    let tag = unique_tag("ramgate_control");
    // BOUND, not asserted-and-dropped: the handle owns the SHM NAME, so a
    // temporary would unlink the object before `plane_exists` could see it.
    let arm = arm_capture_plane(PlaneRole::Monolith, &tag, Some(1_000_000));
    assert!(arm.is_some(), "a process inside the ceiling must arm");
    assert!(plane_exists(&tag));
}

/// `CERULION_STATE_ARM_TAG` SURVIVES as the manual override, and
/// what it overrides is the NAME of the plane this run creates.
///
/// The pure precedence is oracle-tested in `state_arm_attach`'s own module; what
/// this adds is that the override reaches the SHM object — the half that decides
/// whether a bridged/rmw peer, which can only be told a name, can ever meet the
/// plane. It is the manual override on an always-on feature rather than half
/// of an opt-in pairing, which is why it is pinned here.
#[test]
#[serial_test::serial]
fn the_explicit_tag_override_names_the_plane_that_is_created() {
    let _env = clean_env();
    let explicit = unique_tag("explicit_override");
    let derived = cerulion_cli_engine::state_arm_attach::state_arm_tag_for_process(Some(0x2a));
    let _tagenv = EnvGuard::set(
        cerulion_cli_engine::state_arm_attach::STATE_ARM_TAG_ENV,
        &explicit,
    );

    let (resolved, source) =
        cerulion_cli_engine::state_arm_attach::state_arm_tag_for_process(Some(0x2a))
            .expect("a run with an explicit tag resolves one");
    assert_eq!(resolved, explicit, "the explicit tag wins over the derived");
    assert_eq!(
        source,
        cerulion_cli_engine::state_arm_attach::ArmTagSource::Explicit
    );

    let _arm = arm_capture_plane(PlaneRole::Monolith, &resolved, Some(1_000_000))
        .expect("arms under the given name");
    assert!(
        plane_exists(&explicit),
        "the plane must exist under the OPERATOR'S name — a peer told that name has \
         no other way to find it"
    );
    // …and NOT under the name it would have derived, which is the half that
    // proves the override actually diverted the creation.
    let derived_tag = derived
        .expect("a run derives a tag when nothing overrides")
        .0;
    assert_ne!(derived_tag, explicit, "precondition: the two names differ");
    assert!(
        !plane_exists(&derived_tag),
        "nothing may be created under the derived name once an operator named one"
    );
}
