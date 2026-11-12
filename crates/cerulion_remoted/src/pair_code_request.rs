// SPDX-License-Identifier: AGPL-3.0-only
//! Arming a pairing code from a request file on the robot, and writing what
//! the code comes to.
//!
//! A step on the robot, run as the user this daemon runs as, leaves a
//! [`REQUEST_FILE`] in the remoted state directory; its grammar, the result's
//! and the rule a writer follows are in [`cerulion_pairing::pair_code`].
//!
//! Before the daemon logs that it is listening, [`StartSweep::run`] removes
//! every file of the handoff in the directory
//! ([`cerulion_pairing::pair_code::is_handoff_file`]), whatever user the daemon
//! runs as. A request left before that removal, by this start or an earlier
//! run, is never armed; one left after the listening line is this run's to
//! arm. While the directory cannot be listed or a file left before the start
//! cannot be removed, no request is claimed: each poll tries the removal again
//! first, and the removal that then succeeds also removes every request and
//! staging file left in the meantime, unarmed and unanswered.
//!
//! A look for a request ([`PairCodeArm::apply_request`]):
//!
//! 1. It claims the request by renaming it to [`CLAIMED_FILE`], so a request
//!    written meanwhile waits for the next poll.
//! 2. It reads the claimed file through the owner-only checks: the directory
//!    and the file are owned by the user this daemon runs as (its effective
//!    user id) and give their group and others no permission bit, the file is
//!    a regular file with a single link of at most [`MAX_FILE_BYTES`] bytes,
//!    neither the directory nor the file is a symbolic link (each is opened
//!    without following one at its last component), and both are inspected
//!    through their open handles.
//! 3. It removes the claimed file. A request whose claimed file cannot be
//!    removed, or is gone before it is read, is refused, so no armed code stays
//!    on disk.
//! 4. It parses the request and arms its code under [`PAIR_CODE_POLICY`]: the
//!    code pairs one desk, within the window counted from the arm on the
//!    store's monotonic clock, with one budget of tries every desk shares
//!    ([`SharedCodePairSessions`]).
//! 5. It writes [`RESULT_FILE`], mode 0600, through [`RESULT_STAGING_FILE`] and
//!    a rename: `armed`, no try failed. The result repeats the request's nonce
//!    and names this robot's endpoint id, the window in seconds and the tries;
//!    it never carries the code. The result is built before the code is
//!    armed, so one that cannot be built arms nothing; when it cannot be
//!    written, the code is disarmed again, which leaves no code armed (the
//!    arm replaced an earlier one).
//!
//! A claimed request that arms no code is refused
//! ([`PairCodeRefusal`]): `unreadable` when the owner-only read or the removal
//! of its claimed file fails, `malformed` when it does not parse, and
//! `unconfirmed` when its result cannot be built or written. A refusal
//! disarms the code an earlier request armed, since the result answers the
//! request claimed last (see below for an end that code reached), and is
//! counted in the store ([`SharedCodePairSessions::requests_refused`]) and
//! logged once at warn ([`REFUSED_LINE`]). Its result says `refused` with the
//! reason, and with the request's nonce when it was parsed (`-` otherwise);
//! when that result cannot be written, the result file is removed, so no
//! result names a code that is gone (a removal that fails is logged at
//! error), and the next look writes it again. A refused result that cannot
//! be built (a policy with a window under one second, which only a test sets)
//! is not written, and the result file is removed.
//!
//! From then on [`PairCodeArm::report`] writes the result again when the
//! code's state changed since the last write: the failed count, then `paired`
//! with the desk's account and label, `exhausted` or `expired`. A window that
//! passes with no desk acting is written as `expired`. A write that fails is
//! retried on the next look.
//!
//! A newer request's code replaces the armed code and its nonce in the
//! result, and a refusal replaces the armed code too, once a right code's row
//! being saved is saved. When the replaced code reached an end the result
//! does not state yet, a desk's pairing included, that end is logged and
//! written first, as the replaced code's last result. A last result that
//! cannot be built or written, here or at the stop, is counted, logged
//! ([`LAST_WRITE_FAILED_LINE`]) and not written again, and the result file is
//! removed, so no result names a code that is gone.
//!
//! A claim, a removal of the handoff files or a write of the result that keeps
//! failing is logged at warn once and again at each tenfold count, and at
//! debug with a count in between; a claim or a result write that works again
//! after a suppressed repeat is logged at info, and so is the removal that
//! lets requests be armed. A failed try is logged at info, a pairing and an
//! expiry at info, and a code whose tries all failed at warn.
//! [`PairCodeArm::claim_failures`], [`PairCodeArm::removal_failures`] and
//! [`PairCodeArm::report_failures`] count every failure.
//!
//! [`PairCodeArm::stop_arming`] waits for a look or a write in progress to
//! end, then withdraws the armed code once a right code's row being saved is
//! saved, so no desk pairs with it afterwards, and writes what the result is
//! to say last: the code's own end when it reached one the result does not
//! state yet, `refused` with the reason `stopping` when it reached none, or a
//! refusal not yet written. After it returns, no further code is armed and no
//! result is written, and a look that starts later leaves a request where it
//! is. The armed code is held in memory only: a restart of the daemon disarms
//! it. Nothing here is reachable from the network.

use std::io::Write as _;
use std::os::unix::fs::OpenOptionsExt as _;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, MutexGuard, PoisonError};

use cerulion_core::transport::failure_regime_latch::{
    lock_regime_latch, FailureRegimeLatch, RegimeDecision,
};
use cerulion_pairing::pair_code::{
    is_handoff_file, PairCodeNonce, PairCodeRefusal, PairCodeRequest, PairCodeResult,
    PairCodeState, PairCodeStatus, CLAIMED_FILE, MAX_FILE_BYTES, PAIR_CODE_POLICY, REQUEST_FILE,
    RESULT_FILE, RESULT_STAGING_FILE,
};
use cerulion_pairing::pake::CeremonyConfig;
use cerulion_pairing::PairingError;

use crate::owner_only::{read_owner_only, ExpectedOwner};
use crate::pairing_verbs::{CodeSnapshot, SharedCodePairSessions};

/// The line logged at info, with `failed` and `tries`, when a try on the armed
/// code failed.
pub const TRY_FAILED_LINE: &str = "cerulion_remoted: a pairing try on the armed code failed";
/// The line logged at info, with the desk's `account` and `label`, when a desk
/// paired with the armed code.
pub const PAIRED_LINE: &str =
    "cerulion_remoted: a desk paired with the pairing code; it pairs no other desk";
/// The line logged at warn, with `failed`, `tries` and `total_failures`, when
/// every try on the armed code failed and no desk paired.
pub const EXHAUSTED_LINE: &str =
    "cerulion_remoted: the pairing code was used up by failed tries and no desk paired";
/// The line logged at info, with `failed` and `tries`, when the armed code's
/// window ended and no desk paired.
pub const EXPIRED_LINE: &str = "cerulion_remoted: the pairing code expired and no desk paired";
/// The line logged at warn, once for each claimed request that arms no code,
/// with the `refusal` token, the `reason` and `requests_refused`, the count so
/// far.
pub const REFUSED_LINE: &str = "cerulion_remoted: refused a pair-code request; no code is armed \
                                for it";
/// The line logged at info, with `failed`, `tries` and `requests_refused`,
/// when a stop of arming ([`PairCodeArm::stop_arming`]) withdrew a code no
/// desk paired with.
pub const WITHDRAWN_LINE: &str = "cerulion_remoted: the daemon stops; the armed pairing code is \
                                  withdrawn and no desk can pair with it";
/// The line logged at warn, and at debug while suppressed, with `error`,
/// `path` and `total_failures`, when the last result for a code the arm step
/// replaced or withdrew could not be written; the result file is removed.
pub const LAST_WRITE_FAILED_LINE: &str = "cerulion_remoted: could not write the last pair-code \
                                          result for a code that is replaced or withdrawn; the \
                                          result file is removed and not written again";
/// The refusal of a claimed request whose file could not be removed.
const CLAIMED_NOT_REMOVED: &str = "the claimed pair-code request could not be removed";
/// The refusal of a claimed request that was gone before it was read.
const CLAIMED_GONE: &str = "the claimed pair-code request was gone before it was read";

/// What one look for a request did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ArmOutcome {
    /// No request.
    Nothing,
    /// The request's code is armed and the result is written.
    Armed,
    /// The claimed request was refused as `refusal`: no code is armed for it,
    /// the code an earlier request armed is disarmed, and the result says
    /// `refused` (a write that fails is retried by [`PairCodeArm::report`]).
    /// Its claimed file is removed when it can be (a removal that fails is
    /// logged at warn).
    Refused {
        /// The reason the result names.
        refusal: PairCodeRefusal,
        /// Why, in full.
        reason: String,
    },
    /// The request was well formed and removed, but its code is not armed: its
    /// result could not be built or written. The code armed before it is
    /// disarmed too. When the result could not be written, it says `refused`
    /// with the reason `unconfirmed` once a write works, as for
    /// [`ArmOutcome::Refused`]; when it could not be built, neither can the
    /// refused one, so no result is written and the result file is removed.
    Unconfirmed {
        /// Why.
        reason: String,
    },
    /// The request could not be claimed; it is left for the next poll.
    ClaimFailed {
        /// Why.
        reason: String,
    },
    /// The directory could not be listed, or a file of the handoff left before
    /// requests were armed could not be removed; no request is claimed, and the
    /// next poll tries the removal again.
    LeftoversRemain {
        /// Why.
        reason: String,
    },
    /// Arming is stopped ([`PairCodeArm::stop_arming`]): no request is claimed.
    Stopping,
}

/// What one write of the result did ([`PairCodeArm::report`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReportOutcome {
    /// Nothing to write: no code this step armed and no refusal waits to be
    /// written, the state is already written, a newer arm replaced the code,
    /// or arming is stopped.
    Nothing,
    /// The result now states `status`.
    Written(PairCodeStatus),
    /// The result could not be written; the next look tries again.
    Failed {
        /// Why.
        reason: String,
    },
}

/// The removal of the handoff files a starting daemon runs before it logs that
/// it is listening ([`StartSweep::run`]); a [`PairCodeArm`] is built from it.
#[derive(Debug, PartialEq, Eq)]
pub struct StartSweep(Result<usize, String>);

impl StartSweep {
    /// Remove every file of the handoff in `dir`. A non-zero count of removed
    /// files is logged at info, and why not every one could be removed at warn.
    pub fn run(dir: &Path) -> StartSweep {
        let removed = remove_handoff_files(dir);
        match &removed {
            Ok(0) => {}
            Ok(removed) => tracing::info!(
                removed,
                "cerulion_remoted: removed the pair-code files left before this daemon reported \
                 listening; no code they carried is armed"
            ),
            Err(reason) => tracing::warn!(
                %reason,
                path = %dir.display(),
                "cerulion_remoted: could not remove every pair-code file left before this \
                 daemon reported listening; no request is armed until they are gone"
            ),
        }
        StartSweep(removed)
    }
}

/// Remove every file of the handoff in `dir`; how many were removed, or why
/// the directory could not be listed or a file in it removed. A file that
/// cannot be removed does not stop the removal of the others.
fn remove_handoff_files(dir: &Path) -> Result<usize, String> {
    let listing = |e: std::io::Error| format!("could not list the state directory: {e}");
    let mut removed = 0;
    let mut first_failure = None;
    for entry in std::fs::read_dir(dir).map_err(listing)? {
        let name = entry.map_err(listing)?.file_name();
        let Some(name) = name.to_str().filter(|name| is_handoff_file(name)) else {
            continue;
        };
        match std::fs::remove_file(dir.join(name)) {
            Ok(()) => removed += 1,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => {
                first_failure.get_or_insert_with(|| format!("could not remove {name}: {e}"));
            }
        }
    }
    first_failure.map_or(Ok(removed), Err)
}

/// The bytes of a claimed request to parse, from what reading it returned and
/// whether its file is gone from the disk. A read that was refused keeps its
/// reason.
fn claimed_bytes(read: Result<Option<Vec<u8>>, String>, removed: bool) -> Result<Vec<u8>, String> {
    match read {
        Err(reason) => Err(reason),
        Ok(_) if !removed => Err(CLAIMED_NOT_REMOVED.into()),
        Ok(None) => Err(CLAIMED_GONE.into()),
        Ok(Some(bytes)) => Ok(bytes),
    }
}

/// The code this step armed last: its request's nonce, its arm in the store,
/// and the generation and failed count its result last stated.
struct Tracked {
    nonce: PairCodeNonce,
    arm: u64,
    written: u64,
    failed: u8,
}

/// What the result is to say next, for the request this step claimed last.
enum Answer {
    /// The code its request armed, until the code's end is written.
    Armed(Tracked),
    /// Its request's refusal, while the result stating it is not written.
    Refused(PairCodeResult),
}

/// Arms pairing codes from request files and writes what they come to (see
/// the module docs).
pub struct PairCodeArm {
    dir: PathBuf,
    sessions: SharedCodePairSessions,
    endpoint_id: [u8; 32],
    owner_uid: u32,
    policy: CeremonyConfig,
    /// Every file of the handoff left before requests were armed is removed.
    leftovers_removed: AtomicBool,
    removal_failures: Mutex<FailureRegimeLatch>,
    claim_failures: Mutex<FailureRegimeLatch>,
    report_failures: Mutex<FailureRegimeLatch>,
    answer: Mutex<Option<Answer>>,
    /// Set by [`PairCodeArm::stop_arming`]; held across a look and a write.
    stopped: Mutex<bool>,
}

impl PairCodeArm {
    /// An arm step over the remoted state directory `dir`, arming into
    /// `sessions` for the robot whose endpoint id is `endpoint_id`, after the
    /// removal `start` ran over `dir`. Only a directory and files owned by
    /// `owner_uid`, the user this daemon runs as, are read.
    pub fn new(
        dir: impl Into<PathBuf>,
        sessions: SharedCodePairSessions,
        endpoint_id: [u8; 32],
        owner_uid: u32,
        start: StartSweep,
    ) -> Self {
        let mut removal_failures = FailureRegimeLatch::new();
        if start.0.is_err() {
            // The start's own warn line was this regime's loud line.
            let _ = removal_failures.on_failure();
        }
        PairCodeArm {
            dir: dir.into(),
            sessions,
            endpoint_id,
            owner_uid,
            policy: PAIR_CODE_POLICY,
            leftovers_removed: AtomicBool::new(start.0.is_ok()),
            removal_failures: Mutex::new(removal_failures),
            claim_failures: Mutex::new(FailureRegimeLatch::new()),
            report_failures: Mutex::new(FailureRegimeLatch::new()),
            answer: Mutex::new(None),
            stopped: Mutex::new(false),
        }
    }

    /// Every failed claim since this arm step was built.
    pub fn claim_failures(&self) -> u64 {
        lock_regime_latch(&self.claim_failures).total_failures()
    }

    /// Every failed removal of the handoff files left before requests were
    /// armed, the start's included.
    pub fn removal_failures(&self) -> u64 {
        lock_regime_latch(&self.removal_failures).total_failures()
    }

    /// Every failed write of the result after the first write for a request:
    /// a changed state, a refusal, or the last result of a code this step
    /// replaced or withdrew.
    pub fn report_failures(&self) -> u64 {
        lock_regime_latch(&self.report_failures).total_failures()
    }

    /// Arm the code of [`REQUEST_FILE`] if one is there (see the module docs).
    pub fn apply_request(&self) -> ArmOutcome {
        // Held across the claim, the arm and its result, so a stop waits for
        // them and a look after it claims nothing.
        let stopped = self.stop_flag();
        if *stopped {
            return ArmOutcome::Stopping;
        }
        if let Err(reason) = self.remove_leftovers() {
            return ArmOutcome::LeftoversRemain { reason };
        }
        let request = self.dir.join(REQUEST_FILE);
        if matches!(
            std::fs::symlink_metadata(&request),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound
        ) {
            self.claim_worked();
            return ArmOutcome::Nothing;
        }
        let claimed = self.dir.join(CLAIMED_FILE);
        match std::fs::rename(&request, &claimed) {
            Ok(()) => self.claim_worked(),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                self.claim_worked();
                return ArmOutcome::Nothing;
            }
            Err(e) => {
                self.claim_failed(&e);
                return ArmOutcome::ClaimFailed {
                    reason: e.to_string(),
                };
            }
        }
        let read = read_owner_only(
            &claimed,
            MAX_FILE_BYTES as u64,
            "the pair-code request",
            ExpectedOwner::Uid(self.owner_uid),
        );
        let removed = remove(&claimed);
        let (refusal, reason) = match claimed_bytes(read, removed) {
            Ok(bytes) => match PairCodeRequest::parse(&bytes) {
                Ok(request) => return self.arm(&request),
                Err(e) => (PairCodeRefusal::Malformed, e.to_string()),
            },
            Err(reason) => (PairCodeRefusal::Unreadable, reason),
        };
        self.refuse(None, refusal, &reason);
        ArmOutcome::Refused { refusal, reason }
    }

    /// Remove the handoff files left before requests were armed, unless that
    /// is already done.
    fn remove_leftovers(&self) -> Result<(), String> {
        if self.leftovers_removed.load(Ordering::Acquire) {
            return Ok(());
        }
        const MSG: &str = "cerulion_remoted: could not remove every pair-code file left before \
                           requests were armed; no request is armed until they are gone";
        match remove_handoff_files(&self.dir) {
            Ok(removed) => {
                self.leftovers_removed.store(true, Ordering::Release);
                let mut latch = lock_regime_latch(&self.removal_failures);
                let suppressed = latch.on_success().unwrap_or(0);
                let total_failures = latch.total_failures();
                drop(latch);
                tracing::info!(
                    removed,
                    suppressed,
                    total_failures,
                    "cerulion_remoted: removed the pair-code files left before requests were \
                     armed; no code they carried is armed, and requests are armed from now on"
                );
                Ok(())
            }
            Err(reason) => {
                let mut latch = lock_regime_latch(&self.removal_failures);
                let decision = latch.on_failure();
                let total_failures = latch.total_failures();
                drop(latch);
                let path = self.dir.display();
                match decision {
                    RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                        tracing::warn!(%reason, path = %path, total_failures, "{MSG}")
                    }
                    RegimeDecision::Suppressed { suppressed } => {
                        tracing::debug!(%reason, path = %path, total_failures, suppressed, "{MSG}")
                    }
                }
                Err(reason)
            }
        }
    }

    fn arm(&self, request: &PairCodeRequest) -> ArmOutcome {
        let result = match PairCodeResult::armed(request.nonce(), self.endpoint_id, &self.policy) {
            Ok(result) => result,
            Err(e) => {
                tracing::error!(
                    error = %e,
                    "cerulion_remoted: could not state the pair-code result; nothing is armed"
                );
                let reason = e.to_string();
                self.refuse(Some(request.nonce()), PairCodeRefusal::Unconfirmed, &reason);
                return ArmOutcome::Unconfirmed { reason };
            }
        };
        let armed = self.sessions.arm(request.code(), self.policy);
        self.settle_replaced(armed.replaced);
        match self.write_result(&result.render()) {
            Ok(()) => {
                *self.answer() = Some(Answer::Armed(Tracked {
                    nonce: request.nonce(),
                    arm: armed.armed.arm,
                    written: armed.armed.generation,
                    failed: 0,
                }));
                tracing::info!(
                    ttl_s = result.ttl_s(),
                    tries = result.tries(),
                    "cerulion_remoted: armed a pairing code from the request on this robot; it \
                     pairs one desk within ttl_s seconds of now, and every desk shares its tries"
                );
                ArmOutcome::Armed
            }
            Err(e) => {
                // The arm replaced any earlier code, so no code is left that
                // a result on disk could name.
                self.sessions.disarm();
                tracing::error!(
                    error = %e,
                    path = %self.dir.join(RESULT_FILE).display(),
                    "cerulion_remoted: could not write the pair-code result; the code is disarmed, \
                     an earlier armed code with it, and the result is to say the request was \
                     refused"
                );
                let reason = e.to_string();
                self.refuse(Some(request.nonce()), PairCodeRefusal::Unconfirmed, &reason);
                ArmOutcome::Unconfirmed { reason }
            }
        }
    }

    /// Refuse a claimed request, whose nonce is `nonce` when it was read, as
    /// `refusal`, `reason` saying why: disarm the code an earlier request
    /// armed, writing an end it reached that the result does not state yet,
    /// count the refusal in the store and log it, and answer `refused`. A
    /// refused result that cannot be written is kept for the next look, and
    /// the result file is removed meanwhile, so it names no code that is gone.
    fn refuse(&self, nonce: Option<PairCodeNonce>, refusal: PairCodeRefusal, reason: &str) {
        // The result answers the request claimed last, so a refused one
        // replaces the code an earlier one armed.
        let last = self.sessions.disarm();
        self.settle_replaced(last);
        let requests_refused = self.sessions.count_refused_request();
        tracing::warn!(
            refusal = %refusal.as_str(),
            %reason,
            requests_refused,
            "{REFUSED_LINE}"
        );
        let result = match PairCodeResult::refused(nonce, self.endpoint_id, &self.policy, refusal) {
            Ok(result) => result,
            Err(e) => {
                *self.answer() = None;
                tracing::error!(
                    error = %e,
                    "cerulion_remoted: could not state the refused pair-code result; the result \
                     file is removed"
                );
                self.clear_result();
                return;
            }
        };
        match self.write_result(&result.render()) {
            Ok(()) => *self.answer() = None,
            Err(e) => {
                self.report_failed(&e.to_string());
                self.clear_result();
                *self.answer() = Some(Answer::Refused(result));
            }
        }
    }

    /// Settle the code this step armed last once a newer request's arm or a
    /// refusal replaced or withdrew it, `last` being its state then: when it
    /// reached an end the result does not state yet ([`unwritten_end`]), log
    /// that end and write it as that code's last result. The answer is cleared
    /// either way, for the caller to set.
    fn settle_replaced(&self, last: Option<CodeSnapshot>) {
        let Some(Answer::Armed(t)) = self.answer().take() else {
            return;
        };
        let Some(last) = last.filter(|last| unwritten_end(t.arm, last)) else {
            return;
        };
        log_written(&last, t.failed);
        self.write_last(PairCodeResult::new(
            t.nonce,
            self.endpoint_id,
            last.ttl_s,
            last.tries,
            last.failed,
            last.state,
        ));
    }

    /// Write `result`, the last result for a code this step replaced or
    /// withdrew, which is not written again. A result that cannot be built or
    /// written is counted in [`PairCodeArm::report_failures`] and logged
    /// ([`LAST_WRITE_FAILED_LINE`]), and the result file is removed, so no
    /// result names a code that is gone.
    fn write_last(&self, result: Result<PairCodeResult, PairingError>) {
        let written = result.map_err(|e| e.to_string()).and_then(|result| {
            self.write_result(&result.render())
                .map_err(|e| e.to_string())
        });
        match written {
            Ok(()) => self.report_worked(),
            Err(reason) => {
                let (decision, total_failures) = self.count_report_failure();
                let path = self.dir.join(RESULT_FILE);
                match decision {
                    RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => tracing::warn!(
                        error = %reason,
                        path = %path.display(),
                        total_failures,
                        "{LAST_WRITE_FAILED_LINE}"
                    ),
                    RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                        error = %reason,
                        path = %path.display(),
                        total_failures,
                        suppressed,
                        "{LAST_WRITE_FAILED_LINE}"
                    ),
                }
                self.clear_result();
            }
        }
    }

    /// Remove [`RESULT_FILE`] when a result for the request claimed last
    /// cannot be written, so the file names no code that is gone. A removal
    /// that fails is logged at error: until a later write works, the file may
    /// name a code that is no longer armed.
    fn clear_result(&self) {
        let path = self.dir.join(RESULT_FILE);
        if let Err(e) = remove_file_if_present(&path) {
            tracing::error!(
                error = %e,
                path = %path.display(),
                "cerulion_remoted: could not remove the pair-code result after its write failed; \
                 until a write works it may name a code that is no longer armed"
            );
        }
    }

    /// Write what the result is to say for the request this step claimed
    /// last, when it changed since the last write: the armed code's new state,
    /// or the refusal a failed write left to write.
    pub fn report(&self) -> ReportOutcome {
        let stopped = self.stop_flag();
        if *stopped {
            return ReportOutcome::Nothing;
        }
        let mut answer = self.answer();
        let t = match answer.as_mut() {
            None => return ReportOutcome::Nothing,
            Some(Answer::Refused(result)) => {
                if let Err(e) = self.write_result(&result.render()) {
                    let reason = e.to_string();
                    self.report_failed(&reason);
                    return ReportOutcome::Failed { reason };
                }
                self.report_worked();
                *answer = None;
                return ReportOutcome::Written(PairCodeStatus::Refused);
            }
            Some(Answer::Armed(t)) => t,
        };
        let Some(snapshot) = self.sessions.snapshot().filter(|s| s.arm == t.arm) else {
            // A newer arm or a disarm replaced the code this step armed.
            *answer = None;
            return ReportOutcome::Nothing;
        };
        if snapshot.generation == t.written {
            return ReportOutcome::Nothing;
        }
        let result = PairCodeResult::new(
            t.nonce,
            self.endpoint_id,
            snapshot.ttl_s,
            snapshot.tries,
            snapshot.failed,
            snapshot.state.clone(),
        );
        let written = result.map_err(|e| e.to_string()).and_then(|result| {
            self.write_result(&result.render())
                .map_err(|e| e.to_string())
        });
        if let Err(reason) = written {
            self.report_failed(&reason);
            return ReportOutcome::Failed { reason };
        }
        self.report_worked();
        log_written(&snapshot, t.failed);
        let status = snapshot.state.status();
        t.written = snapshot.generation;
        t.failed = snapshot.failed;
        if status.is_terminal() {
            *answer = None;
        }
        ReportOutcome::Written(status)
    }

    /// Count one failed write of the result: how the regime logs it, and
    /// every failure so far.
    fn count_report_failure(&self) -> (RegimeDecision, u64) {
        let mut latch = lock_regime_latch(&self.report_failures);
        let decision = latch.on_failure();
        (decision, latch.total_failures())
    }

    fn report_failed(&self, reason: &str) {
        const MSG: &str = "cerulion_remoted: could not write the pair-code result; the next look \
                           tries again";
        let (decision, total_failures) = self.count_report_failure();
        let path = self.dir.join(RESULT_FILE);
        match decision {
            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                tracing::warn!(error = %reason, path = %path.display(), total_failures, "{MSG}")
            }
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                error = %reason,
                path = %path.display(),
                total_failures,
                suppressed,
                "{MSG}"
            ),
        }
    }

    fn report_worked(&self) {
        let mut latch = lock_regime_latch(&self.report_failures);
        if let Some(suppressed) = latch.on_success() {
            let total_failures = latch.total_failures();
            drop(latch);
            tracing::info!(
                suppressed,
                total_failures,
                "cerulion_remoted: the pair-code result can be written again"
            );
        }
    }

    /// Write `bytes` to [`RESULT_FILE`]: a new 0600 staging file, synced, then
    /// renamed over it. A failed write removes the staging file.
    fn write_result(&self, bytes: &[u8]) -> std::io::Result<()> {
        let staging = self.dir.join(RESULT_STAGING_FILE);
        remove_file_if_present(&staging)?;
        let written = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .custom_flags(libc::O_NOFOLLOW)
            .open(&staging)
            .and_then(|mut file| {
                file.write_all(bytes)?;
                file.sync_all()
            })
            .and_then(|()| std::fs::rename(&staging, self.dir.join(RESULT_FILE)));
        if written.is_err() {
            remove(&staging);
        }
        written
    }

    fn claim_failed(&self, error: &std::io::Error) {
        const MSG: &str = "cerulion_remoted: could not claim the pair-code request; it waits for \
                           the next poll";
        let mut latch = lock_regime_latch(&self.claim_failures);
        let decision = latch.on_failure();
        let total_failures = latch.total_failures();
        drop(latch);
        match decision {
            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                tracing::warn!(error = %error, total_failures, "{MSG}")
            }
            RegimeDecision::Suppressed { suppressed } => {
                tracing::debug!(error = %error, total_failures, suppressed, "{MSG}")
            }
        }
    }

    fn claim_worked(&self) {
        let mut latch = lock_regime_latch(&self.claim_failures);
        if let Some(suppressed) = latch.on_success() {
            let total_failures = latch.total_failures();
            drop(latch);
            tracing::info!(
                suppressed,
                total_failures,
                "cerulion_remoted: the pair-code request can be claimed again"
            );
        }
    }

    fn stop_flag(&self) -> MutexGuard<'_, bool> {
        self.stopped.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn answer(&self) -> MutexGuard<'_, Option<Answer>> {
        self.answer.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Arm no more codes: wait for a look or a write in progress to end, then
    /// withdraw the armed code, once a right code's row being saved is saved,
    /// and write what the result is to say last; from then on arm nothing and
    /// write no result, and a look that starts later leaves a request where it
    /// is. The result states the code's own end when it reached one the result
    /// does not state yet, a desk's pairing included; a code with no end yet
    /// is answered `refused` with the reason `stopping`; a refusal not yet
    /// written is written. A write that fails is counted, logged
    /// ([`LAST_WRITE_FAILED_LINE`]) and not tried again, and the result file
    /// is removed, so no result names a code that is gone.
    pub fn stop_arming(&self) {
        let mut stopped = self.stop_flag();
        if *stopped {
            return;
        }
        *stopped = true;
        let last = self.sessions.disarm();
        let Some(answer) = self.answer().take() else {
            return;
        };
        let result = match answer {
            Answer::Refused(result) => Ok(result),
            Answer::Armed(t) => {
                let Some(last) = last.filter(|last| last.arm == t.arm) else {
                    return;
                };
                let state = if unwritten_end(t.arm, &last) {
                    log_written(&last, t.failed);
                    last.state.clone()
                } else {
                    let requests_refused = self.sessions.count_refused_request();
                    tracing::info!(
                        failed = last.failed,
                        tries = last.tries,
                        requests_refused,
                        "{WITHDRAWN_LINE}"
                    );
                    PairCodeState::Refused(PairCodeRefusal::Stopping)
                };
                PairCodeResult::new(
                    t.nonce,
                    self.endpoint_id,
                    last.ttl_s,
                    last.tries,
                    last.failed,
                    state,
                )
            }
        };
        self.write_last(result);
    }

    /// Arm under `policy` in place of [`PAIR_CODE_POLICY`].
    #[cfg(test)]
    fn with_policy(mut self, policy: CeremonyConfig) -> Self {
        self.policy = policy;
        self
    }
}

/// Whether `last`, a code's state when the arm step replaced or withdrew it,
/// is an end of the code of arm `tracked` that the result does not state yet.
/// Writing a tracked code's end ends its tracking, so this is whether `last`
/// is that code's and has an end.
fn unwritten_end(tracked: u64, last: &CodeSnapshot) -> bool {
    last.arm == tracked && last.state.status().is_terminal()
}

/// Log what a written result recorded for the code in `snapshot`: a failed
/// try when more failed than `failed_before`, and the code's end.
fn log_written(snapshot: &CodeSnapshot, failed_before: u8) {
    let (failed, tries) = (snapshot.failed, snapshot.tries);
    if failed > failed_before {
        tracing::info!(failed, tries, "{TRY_FAILED_LINE}");
    }
    match &snapshot.state {
        PairCodeState::Armed | PairCodeState::Refused(_) => {}
        PairCodeState::Paired(desk) => tracing::info!(
            account = %hex::encode(desk.account()),
            label = %desk.label(),
            failed,
            tries,
            "{PAIRED_LINE}"
        ),
        PairCodeState::Exhausted => {
            tracing::warn!(failed, tries, total_failures = failed, "{EXHAUSTED_LINE}")
        }
        PairCodeState::Expired => tracing::info!(failed, tries, "{EXPIRED_LINE}"),
    }
}

/// Remove `path`; no file there is no error.
fn remove_file_if_present(path: &Path) -> std::io::Result<()> {
    match std::fs::remove_file(path) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
        removed => removed,
    }
}

/// Remove `path`; whether no file is left there. A failure is logged at warn.
fn remove(path: &Path) -> bool {
    match remove_file_if_present(path) {
        Ok(()) => true,
        Err(e) => {
            tracing::warn!(
                error = %e,
                path = %path.display(),
                "cerulion_remoted: could not remove a pair-code file"
            );
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::{MetadataExt as _, PermissionsExt as _};
    use std::sync::Arc;

    use cerud::error::CerudResult;
    use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey};
    use cerulion_pairing::pair_code::{PairedDesk, REQUEST_STAGING_PREFIX};
    use cerulion_pairing::pake::{CpaceConfirmed, CpaceInitiator, PakeIdentities};

    use crate::clock::MonotonicClock;
    use crate::pairing_verbs::{code_exhausted, code_expired};

    const T_ARM: u64 = 1_000_000_000_000;
    const SECOND: u64 = 1_000_000_000;
    const TTL_NS: u64 = 600 * SECOND;
    const RATE: u64 = 2 * SECOND;
    const EID: [u8; 32] = [0xE1; 32];
    const ROBOT_KEY: [u8; 32] = [0xB0; 32];
    const NONCE: PairCodeNonce = PairCodeNonce::from_bytes([7; 16]);
    const LATER_NONCE: PairCodeNonce = PairCodeNonce::from_bytes([8; 16]);
    const CODE: &str = "73915204";
    const LATER_CODE: &str = "50284617";
    const WRONG: &str = "11111111";

    struct Fixture {
        _root: tempfile::TempDir,
        dir: PathBuf,
        uid: u32,
        sessions: SharedCodePairSessions,
        clock: MonotonicClock,
        arm: PairCodeArm,
    }

    fn fixture() -> Fixture {
        let root = tempfile::tempdir().unwrap();
        let dir = root.path().join("remoted");
        std::fs::create_dir(&dir).unwrap();
        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o700)).unwrap();
        let uid = std::fs::metadata(&dir).unwrap().uid();
        let clock = MonotonicClock::fixed(T_ARM);
        let sessions = SharedCodePairSessions::with_clock(clock.clone());
        let arm = PairCodeArm::new(&dir, sessions.clone(), EID, uid, StartSweep::run(&dir));
        Fixture {
            _root: root,
            dir,
            uid,
            sessions,
            clock,
            arm,
        }
    }

    impl Fixture {
        /// An arm step over this fixture's directory and sessions, built from
        /// `start`.
        fn rebuilt(&self, start: StartSweep) -> PairCodeArm {
            PairCodeArm::new(&self.dir, self.sessions.clone(), EID, self.uid, start)
        }

        fn result(&self) -> String {
            String::from_utf8(std::fs::read(self.dir.join(RESULT_FILE)).unwrap()).unwrap()
        }
    }

    /// The result line for `nonce` with `status`, the policy's window and tries,
    /// `failed` failed tries and the `desk` fields.
    fn line(nonce: &str, status: &str, failed: u8, desk: &str) -> String {
        format!(
            "cerulion-pair-code-result/1 {nonce} {status} - {} 600 3 {failed} {desk}\n",
            "e1".repeat(32)
        )
    }

    /// The `refused` result line for `nonce` with the reason `reason` and
    /// `failed` failed tries.
    fn refused_line(nonce: &str, reason: &str, failed: u8) -> String {
        format!(
            "cerulion-pair-code-result/1 {nonce} refused {reason} {} 600 3 {failed} - -\n",
            "e1".repeat(32)
        )
    }
    const NONCE_HEX: &str = "07070707070707070707070707070707";

    fn set_mode(path: &Path, mode: u32) {
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(mode)).unwrap();
    }

    /// Gives its directory mode 0700 back when dropped.
    struct Listable(PathBuf);

    impl Drop for Listable {
        fn drop(&mut self) {
            let _ = std::fs::set_permissions(&self.0, std::fs::Permissions::from_mode(0o700));
        }
    }

    fn write_mode(path: &Path, bytes: &[u8], mode: u32) {
        std::fs::write(path, bytes).unwrap();
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(mode)).unwrap();
    }

    /// Root lists and opens a directory whatever its mode, so a test that
    /// needs one it cannot list cannot run as root. A skip passes, so with
    /// `CI` set it fails instead.
    fn skip_as_root(test: &str, uid: u32) -> bool {
        if uid != 0 {
            return false;
        }
        assert!(
            std::env::var_os("CI").is_none(),
            "{test} cannot run as root, which ignores the directory permissions it needs, \
             and CI is set: run the cerulion_remoted tests as a non-root user"
        );
        eprintln!("SKIPPING {test}: running as root, which ignores directory permissions");
        true
    }

    /// Leave a request as the step on the robot does: a staging file at
    /// `mode`, renamed into place.
    fn leave(dir: &Path, bytes: &[u8], mode: u32) {
        let staging = dir.join(format!("{REQUEST_STAGING_PREFIX}1.tmp"));
        write_mode(&staging, bytes, mode);
        std::fs::rename(&staging, dir.join(REQUEST_FILE)).unwrap();
    }

    fn request_line(code: &str) -> Vec<u8> {
        PairCodeRequest::new(NONCE, code).unwrap().render()
    }

    fn request_for(nonce: PairCodeNonce, code: &str) -> Vec<u8> {
        PairCodeRequest::new(nonce, code).unwrap().render()
    }

    fn names(dir: &Path) -> Vec<String> {
        let mut names: Vec<String> = std::fs::read_dir(dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        names.sort();
        names
    }

    /// A desk's whole ceremony with `code`, its row saved; a desk that sees
    /// a wrong code itself sends no finish and answers what it saw.
    fn pair(sessions: &SharedCodePairSessions, desk: [u8; 32], code: &str) -> CerudResult<()> {
        pair_with(sessions, desk, code, |_| Ok(()))
    }

    /// [`pair`] with the desk's row saved by `save`.
    fn pair_with(
        sessions: &SharedCodePairSessions,
        desk: [u8; 32],
        code: &str,
        save: impl FnOnce(&CpaceConfirmed) -> CerudResult<()>,
    ) -> CerudResult<()> {
        let initiator = CpaceInitiator::new(
            code,
            PakeIdentities::new(PublicKey(desk), PublicKey(ROBOT_KEY), Vec::new()),
        );
        let (attempt, msg1) = initiator.begin().unwrap();
        let (msg2, confirm) = sessions.start(desk, PublicKey(ROBOT_KEY), &msg1)?;
        let (_keys, initiator_confirm) = attempt
            .finish(&msg2, &confirm)
            .map_err(|e| cerud::error::CerudError::Verb(e.to_string()))?;
        sessions.finish_with(
            desk,
            &initiator_confirm,
            AccountId(desk),
            PrincipalKind::Human,
            "lab-desk",
            save,
        )
    }

    /// A try with a wrong code whose desk sees the mismatch itself, sends no
    /// finish and ends its session.
    fn wrong_try(sessions: &SharedCodePairSessions, desk: [u8; 32]) {
        let session = sessions.open_session(desk);
        let initiator = CpaceInitiator::new(
            WRONG,
            PakeIdentities::new(PublicKey(desk), PublicKey(ROBOT_KEY), Vec::new()),
        );
        let (_attempt, msg1) = initiator.begin().unwrap();
        sessions
            .start(desk, PublicKey(ROBOT_KEY), &msg1)
            .expect("an admitted try");
        drop(session);
    }

    fn assert_refused<T>(result: CerudResult<T>, needle: &str) {
        match result {
            Ok(_) => panic!("expected a refusal containing {needle:?}"),
            Err(e) => assert!(e.to_string().contains(needle), "{e}"),
        }
    }

    fn assert_nothing_armed(sessions: &SharedCodePairSessions) {
        assert_refused(
            sessions.start([9; 32], PublicKey(ROBOT_KEY), b"msg1"),
            "no code pairing is armed",
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_request_file_arms_the_code_it_carries() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);

        // Only the result is left, owner-only, and it is the oracle line: the
        // request's nonce, this robot's endpoint id, 600 s, 3 tries, none
        // failed, no desk.
        assert_eq!(names(&f.dir), vec![RESULT_FILE.to_string()]);
        let result = f.dir.join(RESULT_FILE);
        assert_eq!(std::fs::metadata(&result).unwrap().mode() & 0o777, 0o600);
        let bytes = std::fs::read(&result).unwrap();
        assert_eq!(
            String::from_utf8(bytes.clone()).unwrap(),
            line(NONCE_HEX, "armed", 0, "- -")
        );
        assert!(
            !bytes.windows(CODE.len()).any(|w| w == CODE.as_bytes()),
            "the result never carries the code"
        );
        assert_eq!(f.arm.report(), ReportOutcome::Nothing, "nothing changed");

        // DELIVERY: a desk that knows the code pairs, and the result says so.
        pair(&f.sessions, [4; 32], CODE).expect("the armed code pairs");
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Paired)
        );
        assert_eq!(
            f.result(),
            line(
                NONCE_HEX,
                "paired",
                0,
                &format!("{} lab-desk", "04".repeat(32))
            )
        );
        assert!(logs_contain(PAIRED_LINE));
        assert!(logs_contain(&format!("account={}", "04".repeat(32))));
        assert!(logs_contain("label=lab-desk"));
    }

    #[test]
    fn a_successful_pairing_disarms_the_code_a_request_armed() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        pair(&f.sessions, [4; 32], CODE).expect("the first desk pairs");
        f.clock.set(T_ARM + RATE);
        assert_refused(
            pair(&f.sessions, [5; 32], CODE),
            "this pairing code was already used by a desk",
        );
    }

    #[test]
    fn the_window_counts_from_the_arm() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        // A desk whose first contact is past the window is refused.
        f.clock.set(T_ARM + TTL_NS + 1);
        assert_refused(pair(&f.sessions, [4; 32], CODE), &code_expired());

        // A code armed later counts from its own arm: the last instant of its
        // window still pairs.
        let later = T_ARM + 10 * TTL_NS;
        f.clock.set(later);
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        f.clock.set(later + TTL_NS);
        pair(&f.sessions, [4; 32], CODE).expect("in-window desk pairs");
    }

    #[test]
    fn a_request_that_is_not_owner_only_or_well_formed_is_answered_refused_and_removed() {
        type Setup = fn(&Fixture);
        // Each refusal with the reason token its result line carries.
        const UNREADABLE: (PairCodeRefusal, &str) = (PairCodeRefusal::Unreadable, "unreadable");
        const MALFORMED: (PairCodeRefusal, &str) = (PairCodeRefusal::Malformed, "malformed");
        let cases: Vec<(&str, Setup, &str, (PairCodeRefusal, &str))> = vec![
            (
                "group-readable",
                |f| leave(&f.dir, &request_line(CODE), 0o640),
                "the pair-code request is not owner-only and owned by this robot's user",
                UNREADABLE,
            ),
            (
                "other-readable",
                |f| leave(&f.dir, &request_line(CODE), 0o604),
                "the pair-code request is not owner-only and owned by this robot's user",
                UNREADABLE,
            ),
            (
                "symlink",
                |f| {
                    write_mode(&f.dir.join("target"), &request_line(CODE), 0o600);
                    std::os::unix::fs::symlink(f.dir.join("target"), f.dir.join(REQUEST_FILE))
                        .unwrap();
                },
                "could not open the pair-code request: ",
                UNREADABLE,
            ),
            (
                "hard link",
                |f| {
                    write_mode(&f.dir.join("target"), &request_line(CODE), 0o600);
                    std::fs::hard_link(f.dir.join("target"), f.dir.join(REQUEST_FILE)).unwrap();
                },
                "the pair-code request is not a regular single-link file",
                UNREADABLE,
            ),
            (
                "directory open to its group",
                |f| {
                    leave(&f.dir, &request_line(CODE), 0o600);
                    std::fs::set_permissions(&f.dir, std::fs::Permissions::from_mode(0o750))
                        .unwrap();
                },
                "the state directory is not owner-only",
                UNREADABLE,
            ),
            (
                "oversized",
                |f| leave(&f.dir, &[b'A'; MAX_FILE_BYTES + 1], 0o600),
                "the pair-code request is 513 bytes, above the 512-byte bound",
                UNREADABLE,
            ),
            (
                "malformed",
                |f| {
                    leave(
                        &f.dir,
                        b"cerulion-pair-code-request/1 nonce 12345678\n",
                        0o600,
                    )
                },
                "pair-code file: the nonce is not 32 lowercase hex digits",
                MALFORMED,
            ),
            (
                "a code that is not 8 digits",
                |f| {
                    leave(
                        &f.dir,
                        format!(
                            "cerulion-pair-code-request/1 {} K7Q2M9XA\n",
                            "07".repeat(16)
                        )
                        .as_bytes(),
                        0o600,
                    )
                },
                "pair-code file: the code is not 8 ASCII digits",
                MALFORMED,
            ),
        ];
        for (label, setup, want, (refusal, token)) in cases {
            let f = fixture();
            setup(&f);
            match f.arm.apply_request() {
                ArmOutcome::Refused {
                    refusal: got,
                    reason,
                } => {
                    assert_eq!(got, refusal, "{label}");
                    assert!(reason.contains(want), "{label}: {reason}")
                }
                other => panic!("{label}: expected a refusal, got {other:?}"),
            }
            // Only the result is left of the handoff, and it answers the
            // request: refused, with no nonce the daemon read.
            let left: Vec<String> = names(&f.dir)
                .into_iter()
                .filter(|name| is_handoff_file(name))
                .collect();
            assert_eq!(left, vec![RESULT_FILE.to_string()], "{label}");
            assert_eq!(f.result(), refused_line("-", token, 0), "{label}");
            assert_eq!(f.sessions.requests_refused(), 1, "{label}");
            assert_nothing_armed(&f.sessions);
        }
    }

    #[test]
    fn a_request_owned_for_another_user_than_the_daemons_is_answered_unreadable() {
        // The directory and the file are this process's; the daemon runs as
        // another user, so neither is the daemon's own.
        let f = fixture();
        let arm = PairCodeArm::new(
            &f.dir,
            f.sessions.clone(),
            EID,
            f.uid.wrapping_add(1),
            StartSweep::run(&f.dir),
        );
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(
            arm.apply_request(),
            ArmOutcome::Refused {
                refusal: PairCodeRefusal::Unreadable,
                reason: "the state directory is not owned by the user this daemon runs as".into()
            }
        );
        assert_eq!(names(&f.dir), vec![RESULT_FILE.to_string()]);
        assert_eq!(f.result(), refused_line("-", "unreadable", 0));
        assert_nothing_armed(&f.sessions);
    }

    #[test]
    fn files_left_before_the_start_are_removed_and_arm_nothing() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        for name in [
            CLAIMED_FILE,
            RESULT_FILE,
            RESULT_STAGING_FILE,
            ".pair-code.request.42.tmp",
            "pair-code.request.withdrawn.42",
            "epoch.pending",
            "trust_store",
        ] {
            write_mode(&f.dir.join(name), &request_line(CODE), 0o600);
        }
        assert_eq!(StartSweep::run(&f.dir), StartSweep(Ok(6)));
        assert_eq!(names(&f.dir), vec!["epoch.pending", "trust_store"]);
        let arm = f.rebuilt(StartSweep(Ok(6)));
        assert_eq!(arm.apply_request(), ArmOutcome::Nothing);
        assert_nothing_armed(&f.sessions);
        assert_eq!(arm.removal_failures(), 0);
        assert_eq!(
            StartSweep::run(&f.dir),
            StartSweep(Ok(0)),
            "nothing is left"
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn while_the_directory_cannot_be_listed_no_request_is_armed() {
        const NOT_GONE: &str = "could not remove every pair-code file left before";
        let f = fixture();
        if skip_as_root(
            "while_the_directory_cannot_be_listed_no_request_is_armed",
            f.uid,
        ) {
            return;
        }
        // Dropped before the fixture, so its directory can be removed even
        // when an assertion below fails.
        let _listable = Listable(f.dir.clone());
        leave(&f.dir, &request_line(CODE), 0o600);
        // Write and search, no read: a request in it can still be renamed, but
        // the directory cannot be listed (or opened for reading).
        set_mode(&f.dir, 0o300);
        let start = StartSweep::run(&f.dir);
        let arm = f.rebuilt(start);
        let still_listing = |outcome: ArmOutcome| match outcome {
            ArmOutcome::LeftoversRemain { reason } => assert!(
                reason.starts_with("could not list the state directory: "),
                "{reason}"
            ),
            other => panic!("expected the leftovers to remain, got {other:?}"),
        };
        still_listing(arm.apply_request());
        // A request left after the start, in place of the first, and a
        // writer's staging file wait unclaimed too.
        leave(&f.dir, &request_line(LATER_CODE), 0o600);
        let staging = f.dir.join(format!("{REQUEST_STAGING_PREFIX}2.tmp"));
        write_mode(&staging, &request_line(LATER_CODE), 0o600);
        still_listing(arm.apply_request());
        assert_eq!(arm.removal_failures(), 3, "the start's and two polls'");
        assert_nothing_armed(&f.sessions);
        assert!(
            f.dir.join(REQUEST_FILE).exists() && staging.exists(),
            "the request is not claimed"
        );

        // Once the directory can be listed, the removal that works removes the
        // request left after the start and the staging file too, unarmed and
        // unanswered; a request left after that is armed.
        set_mode(&f.dir, 0o700);
        assert_eq!(arm.apply_request(), ArmOutcome::Nothing);
        assert_eq!(names(&f.dir), Vec::<String>::new());
        assert_nothing_armed(&f.sessions);
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(arm.apply_request(), ArmOutcome::Armed);
        assert_eq!(
            arm.removal_failures(),
            3,
            "a removal that works counts none"
        );
        logs_assert(|logs| {
            let warned = lines_with(logs, &["WARN", NOT_GONE]);
            if warned.len() != 1 || !warned[0].contains("before this daemon reported listening") {
                return Err(format!("warn lines: {warned:#?}"));
            }
            let debugged = lines_with(logs, &["DEBUG", NOT_GONE]);
            let expected = 2 * usize::from(cerulion_core::testing::debug_level_compiled_in());
            if debugged.len() != expected {
                return Err(format!("debug lines: {debugged:#?}"));
            }
            let removed = lines_with(
                logs,
                &[
                    "INFO",
                    "requests are armed from now on",
                    "removed=2",
                    "suppressed=2",
                    "total_failures=3",
                ],
            );
            if removed.len() != 1 {
                return Err(format!("removal lines: {removed:#?}"));
            }
            Ok(())
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_file_left_before_the_start_that_cannot_be_removed_holds_every_request() {
        let f = fixture();
        let stuck = f.dir.join("pair-code.request.withdrawn.1");
        std::fs::create_dir(&stuck).unwrap();
        write_mode(&f.dir.join(RESULT_FILE), b"left", 0o600);
        let start = StartSweep::run(&f.dir);
        match &start.0 {
            Err(reason) => assert!(
                reason.starts_with("could not remove pair-code.request.withdrawn.1: "),
                "{reason}"
            ),
            Ok(removed) => panic!("expected a failed removal, got {removed} removed"),
        }
        assert_eq!(
            names(&f.dir),
            vec!["pair-code.request.withdrawn.1"],
            "the other file is removed"
        );
        assert!(logs_contain(
            "could not remove every pair-code file left before this daemon reported listening"
        ));

        let arm = f.rebuilt(start);
        leave(&f.dir, &request_line(CODE), 0o600);
        assert!(matches!(
            arm.apply_request(),
            ArmOutcome::LeftoversRemain { .. }
        ));
        assert_nothing_armed(&f.sessions);

        std::fs::remove_dir(&stuck).unwrap();
        assert_eq!(
            arm.apply_request(),
            ArmOutcome::Nothing,
            "the request is removed"
        );
        assert_nothing_armed(&f.sessions);

        let gone = StartSweep::run(&f.dir.join("absent"));
        match &gone.0 {
            Err(reason) => assert!(
                reason.starts_with("could not list the state directory: "),
                "{reason}"
            ),
            Ok(removed) => panic!("expected a failed listing, got {removed} removed"),
        }
    }

    #[test]
    fn a_claimed_request_is_parsed_only_when_it_was_read_and_its_file_is_gone() {
        type Read = Result<Option<Vec<u8>>, String>;
        let read = |bytes: &[u8]| Ok(Some(bytes.to_vec()));
        let cases: Vec<(Read, bool, Result<Vec<u8>, String>)> = vec![
            (read(b"line"), true, Ok(b"line".to_vec())),
            (
                read(b"line"),
                false,
                Err("the claimed pair-code request could not be removed".into()),
            ),
            (
                Ok(None),
                true,
                Err("the claimed pair-code request was gone before it was read".into()),
            ),
            (
                Ok(None),
                false,
                Err("the claimed pair-code request could not be removed".into()),
            ),
            (Err("refused".into()), true, Err("refused".into())),
            (Err("refused".into()), false, Err("refused".into())),
        ];
        for (read, removed, want) in cases {
            let label = format!("{read:?} removed={removed}");
            assert_eq!(claimed_bytes(read, removed), want, "{label}");
        }
    }

    #[test]
    fn once_arming_is_stopped_a_request_is_left_and_nothing_is_armed() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        f.arm.stop_arming();
        assert_eq!(f.arm.apply_request(), ArmOutcome::Stopping);
        assert_eq!(names(&f.dir), vec![REQUEST_FILE.to_string()]);
        assert_nothing_armed(&f.sessions);
    }

    #[test]
    #[tracing_test::traced_test]
    fn the_stop_withdraws_a_code_no_desk_paired_with_and_the_result_says_refused_stopping() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        wrong_try(&f.sessions, [1; 32]);
        f.arm.stop_arming();
        // The result names the request's nonce, the failed try and why the
        // code no longer works.
        assert_eq!(f.result(), refused_line(NONCE_HEX, "stopping", 1));
        assert_eq!(f.sessions.requests_refused(), 1);
        // No desk can pair any more.
        f.clock.set(T_ARM + RATE);
        assert_refused(pair(&f.sessions, [4; 32], CODE), "no code pairing is armed");
        // Nothing more is written once the stop returned, and a second stop
        // is the first.
        assert_eq!(f.arm.report(), ReportOutcome::Nothing);
        f.arm.stop_arming();
        assert_eq!(f.result(), refused_line(NONCE_HEX, "stopping", 1));
        assert_eq!(f.sessions.requests_refused(), 1);
        logs_assert(|logs| {
            let withdrawn = lines_with(
                logs,
                &[
                    "INFO",
                    WITHDRAWN_LINE,
                    "failed=1",
                    "tries=3",
                    "requests_refused=1",
                ],
            );
            if withdrawn.len() != 1 {
                return Err(format!("withdrawal lines: {withdrawn:#?}"));
            }
            Ok(())
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn the_stop_writes_a_pairing_the_result_does_not_state_yet() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        // A desk pairs, and no look writes it before the stop.
        pair(&f.sessions, [4; 32], CODE).expect("the armed code pairs");
        f.arm.stop_arming();
        assert_eq!(
            f.result(),
            line(
                NONCE_HEX,
                "paired",
                0,
                &format!("{} lab-desk", "04".repeat(32))
            )
        );
        assert_eq!(f.sessions.requests_refused(), 0, "a pairing is no refusal");
        assert_eq!(f.sessions.snapshot(), None, "the store is disarmed");
        assert!(logs_contain(PAIRED_LINE));
        assert!(!logs_contain(WITHDRAWN_LINE));
    }

    #[test]
    fn the_stop_leaves_a_written_end_and_disarms_the_code() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        for (i, desk) in [[1; 32], [2; 32], [3; 32]].into_iter().enumerate() {
            f.clock.set(T_ARM + RATE * i as u64);
            wrong_try(&f.sessions, desk);
        }
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Exhausted)
        );
        f.arm.stop_arming();
        assert_eq!(f.result(), line(NONCE_HEX, "exhausted", 3, "- -"));
        assert_eq!(f.sessions.snapshot(), None);
        assert_eq!(f.sessions.requests_refused(), 0);
    }

    #[test]
    fn the_stop_writes_a_refusal_a_failed_write_left() {
        let f = fixture();
        let staging = f.dir.join(RESULT_STAGING_FILE);
        std::fs::create_dir(&staging).unwrap();
        std::fs::write(staging.join("x"), b"").unwrap();
        leave(&f.dir, &[b'A'; 8], 0o600);
        assert!(matches!(
            f.arm.apply_request(),
            ArmOutcome::Refused {
                refusal: PairCodeRefusal::Malformed,
                ..
            }
        ));
        assert!(!f.dir.join(RESULT_FILE).exists(), "the write failed");
        std::fs::remove_dir_all(&staging).unwrap();
        f.arm.stop_arming();
        assert_eq!(f.result(), refused_line("-", "malformed", 0));
        assert_eq!(f.arm.report_failures(), 1);
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_stop_whose_write_fails_counts_it_removes_the_result_and_still_withdraws_the_code() {
        const NOT_REMOVED: &str = "could not remove the pair-code result after its write failed";
        // A non-empty directory at the staging name refuses the write; one
        // put at the result's name refuses the rename and the removal too.
        for (blocked, removal_fails) in [(RESULT_STAGING_FILE, false), (RESULT_FILE, true)] {
            let f = fixture();
            leave(&f.dir, &request_line(CODE), 0o600);
            assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
            assert_eq!(f.result(), line(NONCE_HEX, "armed", 0, "- -"));
            let path = f.dir.join(blocked);
            remove_file_if_present(&path).unwrap();
            std::fs::create_dir(&path).unwrap();
            std::fs::write(path.join("x"), b"").unwrap();
            f.arm.stop_arming();
            assert_eq!(f.arm.report_failures(), 1, "{blocked}");
            assert_nothing_armed(&f.sessions);
            // No result is left that names the withdrawn code as armed.
            assert_eq!(names(&f.dir), vec![blocked.to_string()], "{blocked}");
            // The stop does not try again.
            std::fs::remove_dir_all(&path).unwrap();
            assert_eq!(f.arm.report(), ReportOutcome::Nothing, "{blocked}");
            assert_eq!(names(&f.dir), Vec::<String>::new(), "{blocked}");
            assert_eq!(logs_contain(NOT_REMOVED), removal_fails, "{blocked}");
        }
        logs_assert(|logs| {
            let last = lines_with(
                logs,
                &[
                    "WARN",
                    LAST_WRITE_FAILED_LINE,
                    "total_failures=1",
                    "pair-code.result",
                ],
            );
            let retried = lines_with(logs, &["the next look tries again"]);
            let not_removed = lines_with(logs, &["ERROR", NOT_REMOVED]);
            if last.len() != 2 || !retried.is_empty() || not_removed.len() != 1 {
                return Err(format!(
                    "last-write lines {last:#?}, retry lines {retried:#?}, removal lines \
                     {not_removed:#?}"
                ));
            }
            Ok(())
        });
    }

    #[test]
    fn a_stop_with_nothing_armed_writes_nothing() {
        let f = fixture();
        f.arm.stop_arming();
        assert_eq!(names(&f.dir), Vec::<String>::new());
        assert_eq!(f.sessions.requests_refused(), 0);
    }

    #[test]
    fn the_stop_withdraws_identically_on_two_runs() {
        fn run() -> (Vec<u8>, u64) {
            let f = fixture();
            leave(&f.dir, &request_line(CODE), 0o600);
            assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
            wrong_try(&f.sessions, [1; 32]);
            assert_eq!(
                f.arm.report(),
                ReportOutcome::Written(PairCodeStatus::Armed)
            );
            f.arm.stop_arming();
            (
                std::fs::read(f.dir.join(RESULT_FILE)).unwrap(),
                f.sessions.requests_refused(),
            )
        }
        let first = run();
        assert_eq!(first, run());
        assert_eq!(
            String::from_utf8(first.0).unwrap(),
            refused_line(NONCE_HEX, "stopping", 1)
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_result_that_cannot_be_written_disarms_the_code_and_is_answered_unconfirmed_once_it_can() {
        const NOT_REMOVED: &str = "could not remove the pair-code result after its write failed";
        // A non-empty directory at the result's name refuses the rename and
        // the removal of the result; one at the staging name refuses the
        // removal of a stale staging file.
        for (blocked, removal_fails) in [(RESULT_STAGING_FILE, false), (RESULT_FILE, true)] {
            let f = fixture();
            std::fs::create_dir(f.dir.join(blocked)).unwrap();
            std::fs::write(f.dir.join(blocked).join("x"), b"").unwrap();
            leave(&f.dir, &request_line(CODE), 0o600);
            assert!(
                matches!(f.arm.apply_request(), ArmOutcome::Unconfirmed { .. }),
                "{blocked}: the result write fails"
            );
            assert_nothing_armed(&f.sessions);
            assert_eq!(names(&f.dir), vec![blocked.to_string()], "{blocked}");
            assert!(
                f.dir.join(blocked).is_dir(),
                "{blocked}: the directory is left"
            );
            assert_eq!(f.sessions.requests_refused(), 1, "{blocked}");
            assert_eq!(f.arm.report_failures(), 1, "{blocked}: the refused write");
            // Each look tries the refusal again until it is written.
            assert!(
                matches!(f.arm.report(), ReportOutcome::Failed { .. }),
                "{blocked}"
            );
            assert_eq!(f.arm.report_failures(), 2, "{blocked}");
            std::fs::remove_dir_all(f.dir.join(blocked)).unwrap();
            assert_eq!(
                f.arm.report(),
                ReportOutcome::Written(PairCodeStatus::Refused),
                "{blocked}"
            );
            assert_eq!(
                f.result(),
                refused_line(NONCE_HEX, "unconfirmed", 0),
                "{blocked}"
            );
            assert_eq!(
                f.arm.report(),
                ReportOutcome::Nothing,
                "{blocked}: written once"
            );
            assert_eq!(
                logs_contain(NOT_REMOVED),
                removal_fails,
                "{blocked}: a removal that fails is logged at error"
            );
        }
        logs_assert(|logs| {
            let errors = lines_with(logs, &["ERROR", NOT_REMOVED, "pair-code.result"]);
            if errors.len() != 1 {
                return Err(format!("removal error lines: {errors:#?}"));
            }
            Ok(())
        });
    }

    #[test]
    fn a_newer_request_whose_result_cannot_be_written_leaves_no_code_and_answers_it_once_it_can() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        assert_eq!(f.result(), line(NONCE_HEX, "armed", 0, "- -"));
        // A non-empty directory at the staging name refuses the newer write.
        std::fs::create_dir(f.dir.join(RESULT_STAGING_FILE)).unwrap();
        std::fs::write(f.dir.join(RESULT_STAGING_FILE).join("x"), b"").unwrap();
        leave(&f.dir, &request_for(LATER_NONCE, LATER_CODE), 0o600);
        assert!(matches!(
            f.arm.apply_request(),
            ArmOutcome::Unconfirmed { .. }
        ));
        // Neither code pairs, and no result is left naming the earlier one
        // as armed.
        assert_nothing_armed(&f.sessions);
        assert_eq!(names(&f.dir), vec![RESULT_STAGING_FILE.to_string()]);
        assert!(matches!(f.arm.report(), ReportOutcome::Failed { .. }));
        // Once the result can be written it answers the newer request.
        std::fs::remove_dir_all(f.dir.join(RESULT_STAGING_FILE)).unwrap();
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Refused)
        );
        assert_eq!(
            f.result(),
            refused_line("08080808080808080808080808080808", "unconfirmed", 0)
        );
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_directory_at_the_request_name_is_refused_and_logged_when_it_cannot_be_removed() {
        let f = fixture();
        std::fs::create_dir(f.dir.join(REQUEST_FILE)).unwrap();
        assert_eq!(
            f.arm.apply_request(),
            ArmOutcome::Refused {
                refusal: PairCodeRefusal::Unreadable,
                reason: "the pair-code request is not a regular single-link file".into()
            }
        );
        assert_eq!(f.result(), refused_line("-", "unreadable", 0));
        assert_nothing_armed(&f.sessions);
        assert!(
            f.dir.join(CLAIMED_FILE).is_dir(),
            "a directory is not removed"
        );
        assert!(logs_contain("could not remove a pair-code file"));
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_policy_the_result_cannot_state_arms_nothing_and_writes_no_result() {
        let f = fixture();
        let arm = f
            .rebuilt(StartSweep::run(&f.dir))
            .with_policy(CeremonyConfig::new(3, 500_000_000).unwrap());
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(
            arm.apply_request(),
            ArmOutcome::Unconfirmed {
                reason: "pair-code file: the window is not a whole number of seconds from 1, \
                         without a leading zero"
                    .into()
            }
        );
        assert_nothing_armed(&f.sessions);
        assert_eq!(names(&f.dir), Vec::<String>::new());
        assert_eq!(f.sessions.requests_refused(), 1);
        assert!(logs_contain("could not state the refused pair-code result"));
    }

    #[test]
    #[tracing_test::traced_test]
    fn the_result_is_written_again_on_each_change_until_the_tries_are_used_up() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        assert_eq!(f.result(), line(NONCE_HEX, "armed", 0, "- -"));
        wrong_try(&f.sessions, [1; 32]);
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Armed)
        );
        assert_eq!(f.result(), line(NONCE_HEX, "armed", 1, "- -"));
        f.clock.set(T_ARM + RATE);
        wrong_try(&f.sessions, [2; 32]);
        f.clock.set(T_ARM + 2 * RATE);
        wrong_try(&f.sessions, [3; 32]);
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Exhausted)
        );
        assert_eq!(f.result(), line(NONCE_HEX, "exhausted", 3, "- -"));
        // The end is written once.
        assert_eq!(f.arm.report(), ReportOutcome::Nothing);
        f.clock.set(T_ARM + 3 * RATE);
        assert_refused(pair(&f.sessions, [4; 32], CODE), &code_exhausted(3));
        logs_assert(|logs| {
            let failed = lines_with(logs, &["INFO", TRY_FAILED_LINE]);
            if failed.len() != 2
                || !failed[0].contains("failed=1")
                || !failed[1].contains("failed=3")
            {
                return Err(format!("try lines: {failed:#?}"));
            }
            let used_up = lines_with(
                logs,
                &[
                    "WARN",
                    EXHAUSTED_LINE,
                    "failed=3",
                    "tries=3",
                    "total_failures=3",
                ],
            );
            if used_up.len() != 1 {
                return Err(format!("exhausted lines: {used_up:#?}"));
            }
            Ok(())
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_window_that_passes_with_no_desk_acting_is_written_expired() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        f.clock.set(T_ARM + TTL_NS);
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Nothing,
            "still in the window"
        );
        f.clock.set(T_ARM + TTL_NS + 1);
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Expired)
        );
        assert_eq!(f.result(), line(NONCE_HEX, "expired", 0, "- -"));
        assert!(logs_contain(EXPIRED_LINE));
        assert!(!logs_contain(TRY_FAILED_LINE));
    }

    #[test]
    fn a_start_that_first_finds_the_window_passed_has_the_result_written_expired() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        f.clock.set(T_ARM + TTL_NS + 1);
        // No look ran since the window passed: the refused start latches it.
        assert_refused(
            f.sessions.start([4; 32], PublicKey(ROBOT_KEY), b"msg1"),
            &code_expired(),
        );
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Expired)
        );
        assert_eq!(f.result(), line(NONCE_HEX, "expired", 0, "- -"));
    }

    #[test]
    fn a_session_end_that_first_finds_the_window_passed_has_the_result_written_expired() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        let session = f.sessions.open_session([4; 32]);
        f.clock.set(T_ARM + TTL_NS + 1);
        // No try is pending; the session's end latches the window's end.
        drop(session);
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Expired)
        );
        assert_eq!(f.result(), line(NONCE_HEX, "expired", 0, "- -"));
    }

    #[test]
    fn a_finish_with_a_wrong_tag_has_the_result_written_with_the_failed_try() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        let initiator = CpaceInitiator::new(
            WRONG,
            PakeIdentities::new(PublicKey([4; 32]), PublicKey(ROBOT_KEY), Vec::new()),
        );
        let (_attempt, msg1) = initiator.begin().unwrap();
        f.sessions
            .start([4; 32], PublicKey(ROBOT_KEY), &msg1)
            .expect("an admitted try");
        // The admitted try is written first, so only the finish can make the
        // next write.
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Armed)
        );
        assert_eq!(f.result(), line(NONCE_HEX, "armed", 0, "- -"));
        assert_refused(
            f.sessions.finish_with(
                [4; 32],
                &[0; 32],
                AccountId([4; 32]),
                PrincipalKind::Human,
                "d",
                |_| Ok(()),
            ),
            "confirmation failed",
        );
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Armed)
        );
        assert_eq!(f.result(), line(NONCE_HEX, "armed", 1, "- -"));
    }

    /// How a desk or the clock brings the armed code to one of its ends.
    type Reach = fn(&Fixture);

    /// Each end a code comes to, and how a desk or the clock reaches it.
    fn code_ends() -> [(PairCodeStatus, Reach); 3] {
        [
            (PairCodeStatus::Paired, |f| {
                pair(&f.sessions, [4; 32], CODE).unwrap()
            }),
            (PairCodeStatus::Exhausted, |f| {
                for (i, desk) in [[1; 32], [2; 32], [3; 32]].into_iter().enumerate() {
                    f.clock.set(T_ARM + RATE * i as u64);
                    wrong_try(&f.sessions, desk);
                }
            }),
            (PairCodeStatus::Expired, |f| f.clock.set(T_ARM + TTL_NS + 1)),
        ]
    }

    #[test]
    fn a_failed_write_of_an_end_is_retried_until_the_next_look_writes_it() {
        for (end, reach) in code_ends() {
            let f = fixture();
            leave(&f.dir, &request_line(CODE), 0o600);
            assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
            reach(&f);
            // A non-empty directory at the staging name refuses the write.
            let staging = f.dir.join(RESULT_STAGING_FILE);
            std::fs::create_dir(&staging).unwrap();
            std::fs::write(staging.join("x"), b"").unwrap();
            assert!(
                matches!(f.arm.report(), ReportOutcome::Failed { .. }),
                "{end:?}: the write of the end fails"
            );
            assert_eq!(f.arm.report_failures(), 1, "{end:?}");
            assert_eq!(
                f.result(),
                line(NONCE_HEX, "armed", 0, "- -"),
                "{end:?}: the end is not on disk"
            );
            std::fs::remove_dir_all(&staging).unwrap();
            assert_eq!(
                f.arm.report(),
                ReportOutcome::Written(end),
                "{end:?}: the next look writes the end"
            );
            assert_eq!(
                PairCodeResult::parse(f.result().as_bytes())
                    .unwrap()
                    .status(),
                end,
                "{end:?}: the end is on disk"
            );
        }
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_result_write_that_fails_is_counted_and_retried() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        wrong_try(&f.sessions, [1; 32]);
        // A non-empty directory at the staging name refuses each write.
        std::fs::create_dir(f.dir.join(RESULT_STAGING_FILE)).unwrap();
        std::fs::write(f.dir.join(RESULT_STAGING_FILE).join("x"), b"").unwrap();
        for _ in 0..2 {
            assert!(matches!(f.arm.report(), ReportOutcome::Failed { .. }));
        }
        assert_eq!(f.arm.report_failures(), 2);
        assert_eq!(f.result(), line(NONCE_HEX, "armed", 0, "- -"));
        std::fs::remove_dir_all(f.dir.join(RESULT_STAGING_FILE)).unwrap();
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Armed)
        );
        assert_eq!(f.result(), line(NONCE_HEX, "armed", 1, "- -"));
        assert!(logs_contain(
            "could not write the pair-code result; the next look tries again"
        ));
        assert!(logs_contain("the pair-code result can be written again"));
    }

    #[test]
    fn a_newer_request_replaces_the_code_and_its_nonce_in_the_result() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        leave(&f.dir, &request_for(LATER_NONCE, LATER_CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        assert_eq!(
            f.result(),
            line("08080808080808080808080808080808", "armed", 0, "- -")
        );
        // The replaced code no longer pairs: the desk sees a wrong code.
        let session = f.sessions.open_session([4; 32]);
        assert_refused(
            pair(&f.sessions, [4; 32], CODE),
            "pairing key confirmation failed",
        );
        drop(session);
        f.clock.set(T_ARM + RATE);
        pair(&f.sessions, [5; 32], LATER_CODE).expect("the newer code pairs");
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Paired)
        );
        assert_eq!(
            PairCodeResult::parse(f.result().as_bytes()).unwrap(),
            PairCodeResult::new(
                LATER_NONCE,
                EID,
                600,
                3,
                1,
                PairCodeState::Paired(PairedDesk::new([5; 32], "lab-desk").unwrap()),
            )
            .unwrap()
        );
        // A code armed by another than this step is not written.
        f.sessions.arm(CODE, PAIR_CODE_POLICY);
        assert_eq!(f.arm.report(), ReportOutcome::Nothing);
    }

    fn lines_with<'a>(logs: &'a [&'a str], needles: &[&str]) -> Vec<&'a str> {
        logs.iter()
            .copied()
            .filter(|line| needles.iter().all(|n| line.contains(n)))
            .collect()
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_claim_that_keeps_failing_is_counted_logged_once_and_recovers() {
        const WAITS: &str = "could not claim the pair-code request; it waits for the next poll";
        let f = fixture();
        // A non-empty directory at the claimed name refuses the claim rename.
        std::fs::create_dir(f.dir.join(CLAIMED_FILE)).unwrap();
        std::fs::write(f.dir.join(CLAIMED_FILE).join("x"), b"").unwrap();
        leave(&f.dir, &request_line(CODE), 0o600);
        for _ in 0..2 {
            assert!(matches!(
                f.arm.apply_request(),
                ArmOutcome::ClaimFailed { .. }
            ));
        }
        assert_eq!(f.arm.claim_failures(), 2);
        assert!(f.dir.join(REQUEST_FILE).exists(), "the request waits");
        assert_nothing_armed(&f.sessions);

        std::fs::remove_dir_all(f.dir.join(CLAIMED_FILE)).unwrap();
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        assert_eq!(f.arm.claim_failures(), 2, "a success counts no failure");
        logs_assert(|logs| {
            let warned = lines_with(logs, &["WARN", WAITS]);
            if warned.len() != 1 || !warned[0].contains("total_failures=1") {
                return Err(format!("warn lines: {warned:#?}"));
            }
            let debugged = lines_with(logs, &["DEBUG", WAITS]);
            let expected = usize::from(cerulion_core::testing::debug_level_compiled_in());
            if debugged.len() != expected {
                return Err(format!("debug lines: {debugged:#?}"));
            }
            let recovered = lines_with(
                logs,
                &[
                    "INFO",
                    "the pair-code request can be claimed again",
                    "suppressed=1",
                ],
            );
            if recovered.len() != 1 {
                return Err(format!("recovery lines: {recovered:#?}"));
            }
            Ok(())
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_refused_request_replaces_the_code_an_earlier_request_armed_and_is_logged_once() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        // A later request whose code is not 8 digits, its nonce well formed.
        leave(
            &f.dir,
            format!("cerulion-pair-code-request/1 {} 1234\n", "08".repeat(16)).as_bytes(),
            0o600,
        );
        assert!(matches!(
            f.arm.apply_request(),
            ArmOutcome::Refused {
                refusal: PairCodeRefusal::Malformed,
                ..
            }
        ));
        // The result answers the later request, and the earlier code no
        // longer pairs.
        assert_eq!(f.result(), refused_line("-", "malformed", 0));
        assert_nothing_armed(&f.sessions);
        assert_eq!(f.arm.report(), ReportOutcome::Nothing);
        // A request the owner-only read refuses is counted on.
        leave(&f.dir, &request_line(CODE), 0o640);
        assert!(matches!(
            f.arm.apply_request(),
            ArmOutcome::Refused {
                refusal: PairCodeRefusal::Unreadable,
                ..
            }
        ));
        assert_eq!(f.result(), refused_line("-", "unreadable", 0));
        assert_eq!(f.sessions.requests_refused(), 2);
        // A request armed after that is answered as armed again.
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        assert_eq!(f.result(), line(NONCE_HEX, "armed", 0, "- -"));
        logs_assert(|logs| {
            let refused = lines_with(logs, &["WARN", REFUSED_LINE]);
            if refused.len() != 2
                || !refused[0].contains("refusal=malformed")
                || !refused[0].contains("requests_refused=1")
                || !refused[1].contains("requests_refused=2")
                || !refused[1].contains("is not owner-only")
            {
                return Err(format!("refusal lines: {refused:#?}"));
            }
            Ok(())
        });
    }

    /// A request whose nonce does not parse.
    const MALFORMED_REQUEST: &[u8] = b"cerulion-pair-code-request/1 nonce 12345678\n";
    const LATER_NONCE_HEX: &str = "08080808080808080808080808080808";
    const ARMED_LINE: &str = "armed a pairing code from the request on this robot";

    /// The indices of the lines of `logs` that hold every one of `needles`.
    fn indices_with(logs: &[&str], needles: &[&str]) -> Vec<usize> {
        logs.iter()
            .enumerate()
            .filter(|(_, line)| needles.iter().all(|n| line.contains(n)))
            .map(|(i, _)| i)
            .collect()
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_refused_request_first_logs_and_writes_a_pairing_the_result_does_not_state_yet() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        // A desk pairs, and a request that does not parse is claimed before a
        // look writes the pairing.
        pair(&f.sessions, [4; 32], CODE).expect("the armed code pairs");
        leave(&f.dir, MALFORMED_REQUEST, 0o600);
        assert!(matches!(
            f.arm.apply_request(),
            ArmOutcome::Refused {
                refusal: PairCodeRefusal::Malformed,
                ..
            }
        ));
        // The result answers the refused request, every write worked, and
        // nothing is left to write.
        assert_eq!(f.result(), refused_line("-", "malformed", 0));
        assert_eq!(f.sessions.requests_refused(), 1);
        assert_eq!(f.arm.report_failures(), 0);
        assert_eq!(f.arm.report(), ReportOutcome::Nothing);
        assert_nothing_armed(&f.sessions);
        // The pairing is logged once, with its desk, before the refusal.
        let account = format!("account={}", "04".repeat(32));
        logs_assert(|logs| {
            let paired = indices_with(logs, &["INFO", PAIRED_LINE, &account, "label=lab-desk"]);
            let refused = indices_with(logs, &["WARN", REFUSED_LINE, "refusal=malformed"]);
            match (paired.as_slice(), refused.as_slice()) {
                ([p], [r]) if p < r => Ok(()),
                _ => Err(format!(
                    "pairing lines {paired:?}, refusal lines {refused:?}"
                )),
            }
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_newer_request_first_logs_and_writes_each_end_the_result_does_not_state_yet() {
        type Reach = fn(&Fixture);
        let ends: [(&str, Reach); 3] = [
            ("paired", |f| {
                pair(&f.sessions, [4; 32], CODE).expect("the armed code pairs")
            }),
            ("exhausted", |f| {
                for (i, desk) in [[1; 32], [2; 32], [3; 32]].into_iter().enumerate() {
                    f.clock.set(T_ARM + RATE * i as u64);
                    wrong_try(&f.sessions, desk);
                }
            }),
            ("expired", |f| f.clock.set(T_ARM + TTL_NS + 1)),
        ];
        for (label, reach) in ends {
            let f = fixture();
            leave(&f.dir, &request_line(CODE), 0o600);
            assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
            reach(&f);
            leave(&f.dir, &request_for(LATER_NONCE, LATER_CODE), 0o600);
            assert_eq!(f.arm.apply_request(), ArmOutcome::Armed, "{label}");
            assert_eq!(
                f.result(),
                line(LATER_NONCE_HEX, "armed", 0, "- -"),
                "{label}"
            );
            assert_eq!(f.arm.report_failures(), 0, "{label}");
            assert_eq!(f.arm.report(), ReportOutcome::Nothing, "{label}");
        }
        // Each end is logged once, between its code's arm and the newer arm.
        logs_assert(|logs| {
            let arms = indices_with(logs, &["INFO", ARMED_LINE]);
            let account = format!("account={}", "04".repeat(32));
            let ends = [
                indices_with(logs, &["INFO", PAIRED_LINE, &account]),
                indices_with(logs, &["WARN", EXHAUSTED_LINE, "failed=3"]),
                indices_with(logs, &["INFO", EXPIRED_LINE, "failed=0"]),
            ];
            if arms.len() != 6 {
                return Err(format!("arm lines: {arms:?}"));
            }
            for (i, end) in ends.iter().enumerate() {
                match end.as_slice() {
                    [e] if arms[2 * i] < *e && *e < arms[2 * i + 1] => {}
                    _ => return Err(format!("end {i} lines {end:?}, arm lines {arms:?}")),
                }
            }
            Ok(())
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn an_end_already_written_is_not_written_or_logged_again_when_its_code_is_replaced() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        pair(&f.sessions, [4; 32], CODE).expect("the armed code pairs");
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Paired)
        );
        leave(&f.dir, MALFORMED_REQUEST, 0o600);
        assert!(matches!(f.arm.apply_request(), ArmOutcome::Refused { .. }));
        assert_eq!(f.result(), refused_line("-", "malformed", 0));
        // A code with no end is replaced with nothing written or logged for it.
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        leave(&f.dir, &request_for(LATER_NONCE, LATER_CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        assert_eq!(f.result(), line(LATER_NONCE_HEX, "armed", 0, "- -"));
        logs_assert(|logs| {
            let paired = indices_with(logs, &[PAIRED_LINE]);
            let other_ends = indices_with(logs, &[EXHAUSTED_LINE]).len()
                + indices_with(logs, &[EXPIRED_LINE]).len()
                + indices_with(logs, &[TRY_FAILED_LINE]).len();
            if paired.len() != 1 || other_ends != 0 {
                return Err(format!(
                    "pairing lines {paired:?}, other end lines {other_ends}"
                ));
            }
            Ok(())
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_pairing_on_a_code_another_armed_is_not_written_under_this_steps_nonce() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        // A code armed by another than this step replaces this step's code,
        // and a desk pairs with it.
        f.sessions.arm(LATER_CODE, PAIR_CODE_POLICY);
        pair(&f.sessions, [5; 32], LATER_CODE).expect("the other code pairs");
        leave(&f.dir, MALFORMED_REQUEST, 0o600);
        assert!(matches!(f.arm.apply_request(), ArmOutcome::Refused { .. }));
        assert_eq!(f.result(), refused_line("-", "malformed", 0));
        assert_eq!(f.arm.report_failures(), 0);
        assert!(
            !logs_contain(PAIRED_LINE),
            "this step's code paired no desk"
        );
    }

    #[test]
    fn a_replaced_end_is_written_identically_on_two_runs() {
        fn run() -> (Vec<u8>, u64, u64) {
            let f = fixture();
            leave(&f.dir, &request_line(CODE), 0o600);
            assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
            wrong_try(&f.sessions, [1; 32]);
            f.clock.set(T_ARM + RATE);
            pair(&f.sessions, [4; 32], CODE).expect("the armed code pairs");
            leave(&f.dir, MALFORMED_REQUEST, 0o600);
            f.arm.apply_request();
            (
                std::fs::read(f.dir.join(RESULT_FILE)).unwrap(),
                f.sessions.requests_refused(),
                f.arm.report_failures(),
            )
        }
        let first = run();
        assert_eq!(first, run());
        assert_eq!(
            String::from_utf8(first.0).unwrap(),
            refused_line("-", "malformed", 0)
        );
        assert_eq!((first.1, first.2), (1, 0));
    }

    #[test]
    fn the_refused_path_writes_identically_on_two_runs() {
        /// The result file after each step of one script of refused requests
        /// (`None` while there is none), and the counters at its end.
        fn run() -> (Vec<Option<Vec<u8>>>, u64, u64, u64) {
            let f = fixture();
            let read = |f: &Fixture| std::fs::read(f.dir.join(RESULT_FILE)).ok();
            let mut results = Vec::new();
            // Unreadable: the request is open to its group.
            leave(&f.dir, &request_line(CODE), 0o640);
            f.arm.apply_request();
            results.push(read(&f));
            // Malformed: its nonce does not parse.
            leave(&f.dir, MALFORMED_REQUEST, 0o600);
            f.arm.apply_request();
            results.push(read(&f));
            // Unconfirmed: a non-empty directory at the staging name refuses
            // every write until it is removed.
            let staging = f.dir.join(RESULT_STAGING_FILE);
            std::fs::create_dir(&staging).unwrap();
            std::fs::write(staging.join("x"), b"").unwrap();
            leave(&f.dir, &request_for(LATER_NONCE, LATER_CODE), 0o600);
            f.arm.apply_request();
            results.push(read(&f));
            std::fs::remove_dir_all(&staging).unwrap();
            f.arm.report();
            results.push(read(&f));
            (
                results,
                f.sessions.requests_refused(),
                f.arm.report_failures(),
                f.arm.claim_failures(),
            )
        }
        let first = run();
        assert_eq!(first, run());
        let want = |nonce: &str, reason: &str| Some(refused_line(nonce, reason, 0).into_bytes());
        assert_eq!(
            first.0,
            vec![
                want("-", "unreadable"),
                want("-", "malformed"),
                None,
                want(LATER_NONCE_HEX, "unconfirmed"),
            ]
        );
        assert_eq!((first.1, first.2, first.3), (3, 1, 0));
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_replaced_codes_end_that_cannot_be_written_is_counted_logged_and_leaves_no_result() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        pair(&f.sessions, [4; 32], CODE).expect("the armed code pairs");
        // A non-empty directory at the staging name refuses every write.
        let staging = f.dir.join(RESULT_STAGING_FILE);
        std::fs::create_dir(&staging).unwrap();
        std::fs::write(staging.join("x"), b"").unwrap();
        leave(&f.dir, &request_for(LATER_NONCE, LATER_CODE), 0o600);
        assert!(matches!(
            f.arm.apply_request(),
            ArmOutcome::Unconfirmed { .. }
        ));
        // The pairing's write and the newer request's both failed: each is
        // counted, and no result is left that names the paired code as armed.
        assert_eq!(f.arm.report_failures(), 2);
        assert_eq!(names(&f.dir), vec![RESULT_STAGING_FILE.to_string()]);
        assert!(logs_contain(PAIRED_LINE), "the pairing is logged");
        // Once a write works the result answers the newer request.
        std::fs::remove_dir_all(&staging).unwrap();
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Refused)
        );
        assert_eq!(f.result(), refused_line(LATER_NONCE_HEX, "unconfirmed", 0));
        logs_assert(|logs| {
            let last = lines_with(logs, &["WARN", LAST_WRITE_FAILED_LINE, "total_failures=1"]);
            if last.len() != 1 {
                return Err(format!("last-write lines: {last:#?}"));
            }
            Ok(())
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_newer_request_claimed_while_a_rows_save_runs_waits_for_it_and_logs_the_pairing() {
        let f = fixture();
        leave(&f.dir, &request_line(CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        // Desk 4's right code: its row's save waits to be released.
        let (reached_tx, reached) = std::sync::mpsc::channel();
        let (release_tx, release) = std::sync::mpsc::channel::<()>();
        let saving = std::thread::spawn({
            let sessions = f.sessions.clone();
            move || {
                pair_with(&sessions, [4; 32], CODE, move |_| {
                    let _ = reached_tx.send(());
                    let _ = release.recv();
                    Ok(())
                })
            }
        });
        reached
            .recv_timeout(std::time::Duration::from_secs(10))
            .expect("the row's save started");
        let released = Arc::new(AtomicBool::new(false));
        let releaser = std::thread::spawn({
            let released = Arc::clone(&released);
            move || {
                std::thread::sleep(std::time::Duration::from_millis(300));
                released.store(true, Ordering::SeqCst);
                let _ = release_tx.send(());
            }
        });
        leave(&f.dir, &request_for(LATER_NONCE, LATER_CODE), 0o600);
        assert_eq!(f.arm.apply_request(), ArmOutcome::Armed);
        assert!(
            released.load(Ordering::SeqCst),
            "the newer code was armed while a row was saved"
        );
        saving.join().unwrap().expect("desk 4's row is saved");
        releaser.join().unwrap();
        assert_eq!(f.result(), line(LATER_NONCE_HEX, "armed", 0, "- -"));
        assert!(logs_contain(PAIRED_LINE));
        assert!(logs_contain(&format!("account={}", "04".repeat(32))));
        // The newer code is armed whole, and pairs another desk.
        f.clock.set(T_ARM + RATE);
        pair(&f.sessions, [5; 32], LATER_CODE).expect("the newer code pairs");
        assert_eq!(
            f.arm.report(),
            ReportOutcome::Written(PairCodeStatus::Paired)
        );
    }

    #[test]
    fn an_unwritten_end_is_the_tracked_codes_own_end() {
        let snapshot = |arm, state| CodeSnapshot {
            arm,
            generation: 9,
            state,
            tries: 3,
            failed: 0,
            ttl_s: 600,
        };
        let desk = PairedDesk::new([4; 32], "lab-desk").unwrap();
        let cases = [
            (
                "its own pairing",
                2,
                PairCodeState::Paired(desk.clone()),
                true,
            ),
            ("its own used-up tries", 2, PairCodeState::Exhausted, true),
            ("its own quiet window", 2, PairCodeState::Expired, true),
            ("its own code with no end", 2, PairCodeState::Armed, false),
            (
                "another arm's pairing",
                3,
                PairCodeState::Paired(desk),
                false,
            ),
            (
                "another arm's used-up tries",
                1,
                PairCodeState::Exhausted,
                false,
            ),
            ("another arm with no end", 3, PairCodeState::Armed, false),
        ];
        for (label, arm, state, want) in cases {
            assert_eq!(unwritten_end(2, &snapshot(arm, state)), want, "{label}");
        }
    }

    #[test]
    fn no_request_is_nothing_and_counts_no_failure() {
        let f = fixture();
        assert_eq!(f.arm.apply_request(), ArmOutcome::Nothing);
        assert_eq!(f.arm.claim_failures(), 0);
        assert_eq!(names(&f.dir), Vec::<String>::new());
    }
}
