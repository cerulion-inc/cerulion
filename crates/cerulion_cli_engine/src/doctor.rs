// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion doctor`: read-only checks of the host a graph runs on.
//!
//! Each check reads one fact about this machine (a limit, a mount, a file, a
//! daemon's state), decides `pass`, `info`, `warn`, `fail` or `skip` with a
//! pure function over that reading, and states why the fact matters to
//! Cerulion and, for `warn` and `fail`, the exact command or edit that fixes
//! it. No check needs root, and none writes a file or changes a sysctl or a
//! network setting; `limits.open-files` raises this process's own soft
//! open-file limit the way `graph run` does, to read the limit a graph gets.
//! Only a check marked as a network check sends anything off this machine,
//! and `--local` and `CERULION_NETWORK=off` skip those. The findings are
//! never part of the usage event the binary sends for every command.
//!
//! [`run`] starts every selected check on its own thread and waits for all of
//! them up to one deadline. A check that has not answered by then is
//! reported as `warn` (did not answer) and its thread is abandoned; the
//! process exits once the report is printed, so the abandoned thread ends
//! with it. Checks only read, so abandoning one leaves nothing half done.
//!
//! [`Report`] renders as the human table ([`Report::render_human`]) and as
//! JSON ([`Report::to_json`]), whose shape is versioned by
//! [`REPORT_VERSION`].

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::{Duration, Instant};

mod account;
mod cpu;
mod disk;
mod iceoryx2_config;
mod limits;
mod shm;

/// The version of the `--json` report shape. It changes only when a field is
/// removed or changes meaning; a new check or a new reading keeps it.
pub const REPORT_VERSION: u32 = 1;

/// The deadline [`run`] waits for the checks when `--timeout-ms` is not
/// given: twice [`crate::shm_state::SHM_STATE_REPORT_BUDGET`]. The slowest
/// check by construction is `shm.stale-nodes`, which walks the node registry
/// and then scans the state files under that budget; the factor of two gives
/// the walk as long as the scan.
pub const DEFAULT_TIMEOUT: Duration = crate::shm_state::SHM_STATE_REPORT_BUDGET.saturating_mul(2);

/// The outcome of one check.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Status {
    /// The reading is what Cerulion needs.
    Pass,
    /// A fact reported with no judgement (the host's CPU governor, say).
    Info,
    /// Cerulion works, but slower or less reliably in the way `why` names.
    Warn,
    /// A Cerulion feature does not work on this host as it is.
    Fail,
    /// The check does not apply on this platform, or was not run on request
    /// (`--local`, `CERULION_NETWORK=off`). The summary says which.
    Skip,
}

impl Status {
    /// The lowercase word the table and the JSON print.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Status::Pass => "pass",
            Status::Info => "info",
            Status::Warn => "warn",
            Status::Fail => "fail",
            Status::Skip => "skip",
        }
    }
}

/// One reading value a check reports beside its summary. JSON carries it as
/// the matching JSON type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reading {
    /// A count, a size in bytes or a limit.
    Int(u64),
    /// A path, a name or a raw value as read.
    Text(String),
    /// A yes or no fact.
    Bool(bool),
}

impl Reading {
    fn to_json(&self) -> serde_json::Value {
        match self {
            Reading::Int(v) => serde_json::Value::from(*v),
            Reading::Text(v) => serde_json::Value::from(v.as_str()),
            Reading::Bool(v) => serde_json::Value::from(*v),
        }
    }
}

/// What a check decided.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Outcome {
    /// The decision.
    pub status: Status,
    /// One line: what was read, in the terms the user acts on.
    pub summary: String,
    /// The exact fix, present for every `warn` and `fail`.
    pub fix: Option<String>,
    /// Named readings, for `--json`.
    pub readings: BTreeMap<&'static str, Reading>,
}

impl Outcome {
    /// An outcome with no fix and no readings.
    #[must_use]
    pub fn new(status: Status, summary: impl Into<String>) -> Self {
        Self {
            status,
            summary: summary.into(),
            fix: None,
            readings: BTreeMap::new(),
        }
    }

    /// The same outcome carrying `fix`.
    #[must_use]
    pub fn with_fix(mut self, fix: impl Into<String>) -> Self {
        self.fix = Some(fix.into());
        self
    }

    /// The same outcome carrying the reading `key`.
    #[must_use]
    pub fn reading(mut self, key: &'static str, value: Reading) -> Self {
        self.readings.insert(key, value);
        self
    }

    /// A `skip` for a check that does not apply on this platform.
    #[must_use]
    pub fn not_applicable(why: impl Into<String>) -> Self {
        Self::new(Status::Skip, format!("not applicable: {}", why.into()))
    }
}

/// What every check reads from the invocation rather than the host.
#[derive(Debug, Clone)]
pub struct Context {
    /// The directory the command runs in.
    pub cwd: PathBuf,
    /// The workspace root when the command runs inside one.
    pub workspace_root: Option<PathBuf>,
}

/// A check in the catalogue.
#[derive(Debug)]
pub struct Check {
    /// The stable id: `<area>.<name>`, never reused for another meaning.
    pub id: &'static str,
    /// Why the fact matters to Cerulion, in one or two sentences.
    pub why: &'static str,
    /// Whether the check sends anything off this machine. `--local` and
    /// `CERULION_NETWORK=off` skip these.
    pub network: bool,
    /// Read the host and decide.
    pub run: fn(&Context) -> Outcome,
}

/// Every check, in the order the report lists them.
pub static CATALOGUE: &[Check] = &[
    limits::OPEN_FILES,
    shm::SIZE,
    shm::STALE_NODES,
    iceoryx2_config::CWD_CONFIG,
    disk::RECORDINGS,
    account::LOGIN,
    cpu::GOVERNOR,
];

/// The ids of every check, in catalogue order (for `--check` completion).
#[must_use]
pub fn check_ids() -> Vec<&'static str> {
    CATALOGUE.iter().map(|c| c.id).collect()
}

/// One check's line in the report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Finding {
    /// [`Check::id`].
    pub id: &'static str,
    /// [`Check::why`].
    pub why: &'static str,
    /// Whether the check is a network check.
    pub network: bool,
    /// What the check decided.
    pub outcome: Outcome,
    /// How long the check took, or `None` when it did not answer in time.
    pub elapsed: Option<Duration>,
}

/// The whole report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Report {
    /// The findings, in catalogue order.
    pub findings: Vec<Finding>,
    /// The deadline every check ran under.
    pub timeout: Duration,
}

/// The process exit code for `findings`: 1 when any check failed, else 0. A
/// `warn` never changes the exit code.
#[must_use]
pub fn exit_code(findings: &[Finding]) -> u8 {
    if findings.iter().any(|f| f.outcome.status == Status::Fail) {
        1
    } else {
        0
    }
}

/// Why network checks are not run, or `None` when they are.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetworkGate {
    /// `--local` was given.
    LocalFlag,
    /// `CERULION_NETWORK` is set (to `off`, or to a value that fails closed).
    KillSwitch,
}

impl NetworkGate {
    fn skip_outcome(self) -> Outcome {
        Outcome::new(
            Status::Skip,
            match self {
                NetworkGate::LocalFlag => "not run: --local skips the checks that use the network",
                NetworkGate::KillSwitch => {
                    "not run: CERULION_NETWORK is set, which keeps this command local"
                }
            },
        )
    }
}

/// The network gate for this invocation: `--local` first, then the
/// `CERULION_NETWORK` kill switch parsed by the one parser every network
/// consumer uses ([`crate::graph_cmd::network_env_kill`]).
#[must_use]
pub fn network_gate(local_flag: bool) -> Option<NetworkGate> {
    if local_flag {
        Some(NetworkGate::LocalFlag)
    } else if crate::graph_cmd::network_env_kill().is_off() {
        Some(NetworkGate::KillSwitch)
    } else {
        None
    }
}

/// Why a `--check` selection was refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownCheck(pub String);

impl std::fmt::Display for UnknownCheck {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "no check is named `{}`; the checks are: {}",
            self.0,
            check_ids().join(", ")
        )
    }
}

/// The checks `selected` names, in catalogue order; every check when
/// `selected` is empty.
pub fn select(selected: &[String]) -> Result<Vec<&'static Check>, UnknownCheck> {
    if let Some(unknown) = selected
        .iter()
        .find(|s| !CATALOGUE.iter().any(|c| c.id == s.as_str()))
    {
        return Err(UnknownCheck(unknown.clone()));
    }
    Ok(CATALOGUE
        .iter()
        .filter(|c| selected.is_empty() || selected.iter().any(|s| s == c.id))
        .collect())
}

/// Run `checks` concurrently under one `timeout` and collect their findings
/// in the order given. A network check under `gate` is skipped without
/// running.
pub fn run(
    checks: &[&'static Check],
    ctx: &Context,
    gate: Option<NetworkGate>,
    timeout: Duration,
) -> Report {
    let (tx, rx) = mpsc::channel::<(usize, Outcome, Duration)>();
    let mut slots: Vec<Option<(Outcome, Duration)>> = vec![None; checks.len()];
    let mut pending = 0usize;
    for (index, check) in checks.iter().enumerate() {
        if let (true, Some(gate)) = (check.network, gate) {
            slots[index] = Some((gate.skip_outcome(), Duration::ZERO));
            continue;
        }
        let tx = tx.clone();
        let ctx = ctx.clone();
        let run = check.run;
        pending += 1;
        let id = check.id;
        std::thread::spawn(move || {
            let started = Instant::now();
            let outcome = std::panic::catch_unwind(|| run(&ctx))
                .unwrap_or_else(|payload| panicked(id, payload.as_ref()));
            // The receiver hangs up only after the deadline, when this result
            // is no longer wanted.
            let _ = tx.send((index, outcome, started.elapsed()));
        });
    }
    drop(tx);
    let deadline = Instant::now() + timeout;
    while pending > 0 {
        let left = deadline.saturating_duration_since(Instant::now());
        match rx.recv_timeout(left) {
            Ok((index, outcome, elapsed)) => {
                slots[index] = Some((outcome, elapsed));
                pending -= 1;
            }
            // A timeout: the unfilled slots are the checks that did not
            // answer. Every sender gone cannot leave a slot unfilled, since a
            // panic is caught and sent as an outcome.
            Err(_) => break,
        }
    }
    let findings = checks
        .iter()
        .zip(slots)
        .map(|(check, slot)| {
            let (outcome, elapsed) = match slot {
                Some((outcome, elapsed)) => (outcome, Some(elapsed)),
                None => (did_not_answer(check.id, timeout), None),
            };
            Finding {
                id: check.id,
                why: check.why,
                network: check.network,
                outcome,
                elapsed,
            }
        })
        .collect();
    Report { findings, timeout }
}

/// The outcome of a check that had not answered when the deadline passed.
/// The fix reruns that one check with twice the deadline.
fn did_not_answer(id: &str, timeout: Duration) -> Outcome {
    let ms = timeout.as_millis();
    Outcome::new(
        Status::Warn,
        format!("did not answer within {ms} ms, so nothing was read"),
    )
    .with_fix(format!(
        "run `cerulion doctor --check {id} --timeout-ms {}` to give it longer",
        ms.saturating_mul(2)
    ))
}

/// The outcome of a check that panicked, carrying the panic message.
fn panicked(id: &str, payload: &(dyn std::any::Any + Send)) -> Outcome {
    let message = payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "a panic with no message".to_string());
    Outcome::new(
        Status::Warn,
        format!("the check stopped with a panic, so nothing was read: {message}"),
    )
    .with_fix(format!(
        "rerun `cerulion doctor --check {id}`; a repeat is a Cerulion bug to report with that \
         message"
    ))
}

impl Report {
    /// The count of findings with each status, in [`Status`] order.
    #[must_use]
    pub fn counts(&self) -> BTreeMap<Status, usize> {
        let mut counts = BTreeMap::new();
        for status in [
            Status::Pass,
            Status::Info,
            Status::Warn,
            Status::Fail,
            Status::Skip,
        ] {
            counts.insert(status, 0);
        }
        for f in &self.findings {
            *counts.entry(f.outcome.status).or_insert(0) += 1;
        }
        counts
    }

    /// [`exit_code`] of this report's findings.
    #[must_use]
    pub fn exit_code(&self) -> u8 {
        exit_code(&self.findings)
    }

    /// The human table: one line per check, the `why` and `fix` under every
    /// `warn` and `fail`, and a closing count line.
    #[must_use]
    pub fn render_human(&self) -> String {
        let width = self.findings.iter().map(|f| f.id.len()).max().unwrap_or(0);
        let mut out = String::new();
        for f in &self.findings {
            out.push_str(&format!(
                "{:<4}  {:<width$}  {}\n",
                f.outcome.status.as_str().to_uppercase(),
                f.id,
                f.outcome.summary,
            ));
            if matches!(f.outcome.status, Status::Warn | Status::Fail) {
                let indent = " ".repeat(4 + 2 + width + 2);
                out.push_str(&format!("{indent}why: {}\n", f.why));
                if let Some(fix) = &f.outcome.fix {
                    out.push_str(&format!("{indent}fix: {fix}\n"));
                }
            }
        }
        let counts = self.counts();
        let parts: Vec<String> = counts
            .iter()
            .map(|(status, n)| format!("{n} {}", status.as_str()))
            .collect();
        out.push_str(&format!("\n{}\n", parts.join(", ")));
        out
    }

    /// The `--json` report.
    #[must_use]
    pub fn to_json(&self) -> serde_json::Value {
        let counts: serde_json::Map<String, serde_json::Value> = self
            .counts()
            .into_iter()
            .map(|(status, n)| (status.as_str().to_string(), serde_json::Value::from(n)))
            .collect();
        let checks: Vec<serde_json::Value> = self
            .findings
            .iter()
            .map(|f| {
                let readings: serde_json::Map<String, serde_json::Value> = f
                    .outcome
                    .readings
                    .iter()
                    .map(|(k, v)| ((*k).to_string(), v.to_json()))
                    .collect();
                serde_json::json!({
                    "id": f.id,
                    "status": f.outcome.status.as_str(),
                    "summary": f.outcome.summary,
                    "why": f.why,
                    "fix": f.outcome.fix,
                    "network": f.network,
                    "elapsed_ms": f.elapsed.map(|d| u64::try_from(d.as_millis()).unwrap_or(u64::MAX)),
                    "readings": readings,
                })
            })
            .collect();
        serde_json::json!({
            "report_version": REPORT_VERSION,
            "cerulion_version": env!("CARGO_PKG_VERSION"),
            "os": std::env::consts::OS,
            "arch": std::env::consts::ARCH,
            "timeout_ms": u64::try_from(self.timeout.as_millis()).unwrap_or(u64::MAX),
            "exit_code": self.exit_code(),
            "counts": counts,
            "checks": checks,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quick(_ctx: &Context) -> Outcome {
        Outcome::new(Status::Pass, "quick")
    }

    fn failing(_ctx: &Context) -> Outcome {
        Outcome::new(Status::Fail, "broken").with_fix("mend it")
    }

    fn slow(_ctx: &Context) -> Outcome {
        std::thread::sleep(Duration::from_secs(5));
        Outcome::new(Status::Pass, "slow")
    }

    fn panics(_ctx: &Context) -> Outcome {
        panic!("a check that panics");
    }

    fn network_ran(_ctx: &Context) -> Outcome {
        Outcome::new(Status::Pass, "network check ran")
    }

    static QUICK: Check = Check {
        id: "t.quick",
        why: "quick why",
        network: false,
        run: quick,
    };
    static FAILING: Check = Check {
        id: "t.failing",
        why: "failing why",
        network: false,
        run: failing,
    };
    static SLOW: Check = Check {
        id: "t.slow",
        why: "slow why",
        network: false,
        run: slow,
    };
    static PANICS: Check = Check {
        id: "t.panics",
        why: "panics why",
        network: false,
        run: panics,
    };
    static NETWORK: Check = Check {
        id: "t.network",
        why: "network why",
        network: true,
        run: network_ran,
    };

    fn ctx() -> Context {
        Context {
            cwd: PathBuf::from("/"),
            workspace_root: None,
        }
    }

    fn finding(id: &'static str, status: Status) -> Finding {
        Finding {
            id,
            why: "why",
            network: false,
            outcome: Outcome::new(status, "s"),
            elapsed: Some(Duration::ZERO),
        }
    }

    #[test]
    fn exit_code_is_one_only_when_a_check_failed() {
        let warn_only = [finding("a", Status::Warn), finding("b", Status::Skip)];
        assert_eq!(exit_code(&warn_only), 0);
        let failed = [finding("a", Status::Pass), finding("b", Status::Fail)];
        assert_eq!(exit_code(&failed), 1);
        assert_eq!(exit_code(&[]), 0);
    }

    #[test]
    fn select_keeps_catalogue_order_and_refuses_an_unknown_id() {
        let all = select(&[]).unwrap();
        assert_eq!(all.len(), CATALOGUE.len());
        let picked = select(&["account.login".to_string(), "limits.open-files".to_string()])
            .unwrap()
            .iter()
            .map(|c| c.id)
            .collect::<Vec<_>>();
        assert_eq!(picked, vec!["limits.open-files", "account.login"]);
        let err = select(&["no.such".to_string()]).unwrap_err();
        assert!(err
            .to_string()
            .starts_with("no check is named `no.such`; the checks are: limits.open-files"));
    }

    #[test]
    fn catalogue_ids_are_unique_and_shaped_area_dot_name() {
        let ids = check_ids();
        let unique: std::collections::BTreeSet<_> = ids.iter().collect();
        assert_eq!(unique.len(), ids.len());
        for id in ids {
            let (area, name) = id.split_once('.').expect("area.name");
            for part in [area, name] {
                assert!(
                    !part.is_empty()
                        && part
                            .chars()
                            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-'),
                    "{id}"
                );
            }
        }
    }

    #[test]
    fn run_keeps_order_and_reports_a_slow_check_as_did_not_answer() {
        let report = run(&[&SLOW, &QUICK], &ctx(), None, Duration::from_millis(200));
        assert_eq!(report.findings[0].id, "t.slow");
        assert_eq!(report.findings[0].outcome.status, Status::Warn);
        assert_eq!(report.findings[0].elapsed, None);
        assert_eq!(
            report.findings[0].outcome.summary,
            "did not answer within 200 ms, so nothing was read"
        );
        assert_eq!(
            report.findings[0].outcome.fix.as_deref(),
            Some("run `cerulion doctor --check t.slow --timeout-ms 400` to give it longer")
        );
        assert_eq!(report.findings[1].outcome.summary, "quick");
        assert!(report.findings[1].elapsed.is_some());
    }

    #[test]
    fn a_panicking_check_is_reported_with_its_message_and_the_rest_still_report() {
        let report = run(&[&PANICS, &QUICK], &ctx(), None, Duration::from_secs(5));
        assert_eq!(report.findings[0].outcome.status, Status::Warn);
        assert_eq!(
            report.findings[0].outcome.summary,
            "the check stopped with a panic, so nothing was read: a check that panics"
        );
        assert!(report.findings[0].elapsed.is_some());
        assert_eq!(report.findings[1].outcome.status, Status::Pass);
    }

    #[test]
    fn the_network_gate_skips_network_checks_without_running_them() {
        for (gate, needle) in [
            (NetworkGate::LocalFlag, "--local"),
            (NetworkGate::KillSwitch, "CERULION_NETWORK"),
        ] {
            let report = run(
                &[&NETWORK, &QUICK],
                &ctx(),
                Some(gate),
                Duration::from_secs(5),
            );
            assert_eq!(report.findings[0].outcome.status, Status::Skip);
            assert!(report.findings[0].outcome.summary.contains(needle));
            assert_eq!(report.findings[1].outcome.status, Status::Pass);
        }
        let ungated = run(&[&NETWORK], &ctx(), None, Duration::from_secs(5));
        assert_eq!(ungated.findings[0].outcome.summary, "network check ran");
    }

    #[test]
    fn the_local_flag_gates_the_network_whatever_the_environment_says() {
        assert_eq!(network_gate(true), Some(NetworkGate::LocalFlag));
    }

    #[test]
    fn the_human_table_shows_why_and_fix_only_under_warn_and_fail() {
        let report = run(&[&QUICK, &FAILING], &ctx(), None, Duration::from_secs(5));
        assert_eq!(
            report.render_human(),
            "PASS  t.quick    quick\n\
             FAIL  t.failing  broken\n\
             \x20                why: failing why\n\
             \x20                fix: mend it\n\
             \n\
             1 pass, 0 info, 0 warn, 1 fail, 0 skip\n"
        );
    }

    #[test]
    fn the_json_report_carries_every_field() {
        let mut report = run(&[&FAILING], &ctx(), None, Duration::from_millis(1500));
        report.findings[0].elapsed = Some(Duration::from_millis(7));
        report.findings[0].outcome = report.findings[0]
            .outcome
            .clone()
            .reading("n", Reading::Int(3))
            .reading("p", Reading::Text("/x".to_string()))
            .reading("b", Reading::Bool(true));
        let json = report.to_json();
        assert_eq!(json["report_version"], REPORT_VERSION);
        assert_eq!(json["cerulion_version"], env!("CARGO_PKG_VERSION"));
        assert_eq!(json["os"], std::env::consts::OS);
        assert_eq!(json["arch"], std::env::consts::ARCH);
        assert_eq!(json["timeout_ms"], 1500);
        assert_eq!(json["exit_code"], 1);
        assert_eq!(
            json["counts"],
            serde_json::json!({"pass": 0, "info": 0, "warn": 0, "fail": 1, "skip": 0})
        );
        assert_eq!(
            json["checks"],
            serde_json::json!([{
                "id": "t.failing",
                "status": "fail",
                "summary": "broken",
                "why": "failing why",
                "fix": "mend it",
                "network": false,
                "elapsed_ms": 7,
                "readings": {"b": true, "n": 3, "p": "/x"},
            }])
        );
    }

    #[test]
    fn a_check_that_did_not_answer_has_null_elapsed_in_json() {
        let report = run(&[&SLOW], &ctx(), None, Duration::from_millis(50));
        let json = report.to_json();
        assert_eq!(json["checks"][0]["elapsed_ms"], serde_json::Value::Null);
        assert_eq!(json["checks"][0]["status"], "warn");
    }

    #[test]
    fn the_default_timeout_is_twice_the_state_file_scan_budget() {
        assert_eq!(
            DEFAULT_TIMEOUT,
            crate::shm_state::SHM_STATE_REPORT_BUDGET * 2
        );
    }
}
