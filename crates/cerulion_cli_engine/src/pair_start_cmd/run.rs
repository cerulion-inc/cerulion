// SPDX-License-Identifier: AGPL-3.0-only
//! One run of `cerulion pair --start`: the refusals before the handoff, then
//! the loop that observes what [`Machine::wants`] asks for, takes the step
//! [`Machine::next_step`] decides and writes its lines. The serving host and
//! the process environment are injected, so a test drives a whole run with
//! scripted daemons and a temporary directory.

use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use cerulion_pairing::pair_code::{PairCodeNonce, PairCodeRequest};

use super::handoff;
use super::{
    armed_report, code_from_u32s, no_random_msg, outcome_report, serving_authority_msg,
    state_root_mismatch_msg, tries_failed_report, HoldRefusal, LockTry, Machine, NetdProbe,
    Observation, Pulled, Report, Step, ASKING_LINE, EXIT_REFUSED, HOLD_TTL_S, NO_STATE_ROOT_MSG,
    READY_WAIT, TICK, WAITING_NOTE,
};

/// A serving hold `cerulion-netd` granted.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HeldServing {
    /// The robot state root netd serves.
    pub root: PathBuf,
    /// When the hold lapses.
    pub expires: Instant,
}

/// What serves the robot while its code is armed.
pub trait ServingHost {
    /// Whether this machine may serve; the refusal when it may not.
    fn authority(&mut self) -> Result<(), &'static str>;
    /// Hold serving for `ttl_s` seconds; the hold lasts while the host lives.
    fn hold(&mut self, ttl_s: u64) -> Result<HeldServing, HoldRefusal>;
    /// Probe the serving daemon's status.
    fn probe(&mut self) -> NetdProbe;
}

/// The process a run lives in: its clocks, its sleep, its interrupt and its
/// random source.
pub trait StartEnv {
    /// A monotonic instant.
    fn now(&mut self) -> Instant;
    /// Seconds since the Unix epoch, for display only.
    fn unix_now_s(&mut self) -> u64;
    /// Sleep for `span`.
    fn sleep(&mut self, span: Duration);
    /// Whether an interrupt arrived.
    fn interrupted(&mut self) -> bool;
    /// One uniform random `u32`.
    fn draw_u32(&mut self) -> Result<u32, String>;
    /// A random nonce.
    fn nonce(&mut self) -> Result<[u8; 16], String>;
}

fn write_report(out: &mut impl Write, err: &mut impl Write, report: &Report) {
    // The exit code is the signal; a closed stream does not stop the run.
    let _ = out.write_all(report.stdout.as_bytes());
    let _ = out.flush();
    let _ = err.write_all(report.stderr.as_bytes());
    let _ = err.flush();
}

fn error_line(err: &mut impl Write, line: &str) {
    let _ = writeln!(err, "{line}");
    let _ = err.flush();
}

/// A request with a fresh nonce and code from `env`.
fn draw_request(env: &mut impl StartEnv) -> Result<PairCodeRequest, String> {
    let nonce = PairCodeNonce::from_bytes(env.nonce()?);
    let mut failure = None;
    let code = code_from_u32s(std::iter::from_fn(|| match env.draw_u32() {
        Ok(draw) => Some(draw),
        Err(e) => {
            failure = Some(e);
            None
        }
    }));
    let code = match (code, failure) {
        (Some(code), _) => code,
        (None, Some(e)) => return Err(e),
        (None, None) => return Err("every draw fell outside the accepted range".into()),
    };
    PairCodeRequest::new(nonce, code).map_err(|e| e.to_string())
}

/// Make one pairing code for the robot whose state root is `state_root` and
/// follow it to its end; the exit code. Lines go to `out` (state lines) and
/// `err` (lines for the user).
pub fn run(
    state_root: Option<&Path>,
    host: &mut impl ServingHost,
    env: &mut impl StartEnv,
    out: &mut impl Write,
    err: &mut impl Write,
) -> u8 {
    let Some(root) = state_root else {
        error_line(err, NO_STATE_ROOT_MSG);
        return EXIT_REFUSED;
    };
    if let Err(refusal) = host.authority() {
        error_line(err, &serving_authority_msg(refusal));
        return EXIT_REFUSED;
    }
    let request = match draw_request(env) {
        Ok(request) => request,
        Err(e) => {
            error_line(err, &no_random_msg(&e));
            return EXIT_REFUSED;
        }
    };
    error_line(err, ASKING_LINE);
    let held = match host.hold(HOLD_TTL_S) {
        Ok(held) => held,
        Err(refusal) => {
            error_line(err, &refusal.text());
            return refusal.exit_code();
        }
    };
    if held.root != root {
        error_line(err, &state_root_mismatch_msg(&held.root, root));
        return EXIT_REFUSED;
    }
    let start = env.now();
    let ready_wait = READY_WAIT.min(held.expires.saturating_duration_since(start));
    let mut machine = Machine::new(request.nonce(), ready_wait);
    let dir = root.join(super::REMOTED_SUBDIR);
    let mut lock: Option<File> = None;
    let mut locked = false;
    let mut pulled: Option<Pulled> = None;
    let mut fault: Option<String> = None;
    loop {
        let now = env.now().saturating_duration_since(start);
        let wants = machine.wants(now);
        let mut obs = Observation::at(now, env.unix_now_s());
        obs.interrupted = env.interrupted();
        obs.pulled = pulled.take();
        obs.fault = fault.take();
        if wants.dir {
            obs.dir = Some(handoff::dir_state(&dir));
        }
        if wants.lock {
            let tried = match &lock {
                Some(file) => handoff::try_lock(file),
                None => handoff::open_lock(&dir).and_then(|file| {
                    let taken = handoff::try_lock(&file);
                    lock = Some(file);
                    taken
                }),
            };
            match tried {
                Ok(taken) => {
                    locked = taken;
                    obs.lock = Some(if taken { LockTry::Taken } else { LockTry::Busy });
                }
                Err(e) => obs.fault = Some(e.to_string()),
            }
        }
        if wants.files {
            obs.request_present = Some(handoff::request_present(&dir));
            obs.result = Some(handoff::read_result(&dir));
        }
        if wants.probe {
            obs.probe = Some(host.probe());
        }
        match machine.next_step(&obs) {
            Step::Wait => env.sleep(TICK),
            Step::Note => error_line(err, WAITING_NOTE),
            Step::Leave | Step::LeaveAgain => {
                if let Err(e) = handoff::leave(&dir, &request) {
                    tracing::debug!(
                        path = %dir.display(),
                        error = %e,
                        "cerulion pair --start: could not leave the pair-code request"
                    );
                    fault = Some(e.to_string());
                }
            }
            Step::PullBack => pulled = Some(handoff::pull_back(&dir)),
            Step::ShowArmed(facts) => {
                let port = handoff::port(root, &facts.endpoint_id);
                write_report(out, err, &armed_report(request.code(), &facts, port));
            }
            Step::TriesFailed { from, to, tries } => {
                write_report(out, err, &tries_failed_report(from, to, tries));
            }
            Step::End(outcome) => {
                write_report(out, err, &outcome_report(&outcome, &dir));
                return outcome.exit_code();
            }
        }
        if locked && !machine.holds_lock() {
            // Closing the file releases the lock.
            lock = None;
            locked = false;
        }
    }
}
