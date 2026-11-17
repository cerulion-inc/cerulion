// SPDX-License-Identifier: AGPL-3.0-only
//! The robot side of code pairing: what each step of making one pairing code
//! decides, and every text it shows.
//!
//! The steps follow the writer's rule of [`cerulion_pairing::pair_code`]:
//!
//! 1. Wait, within the ready deadline, until `<state_root>/remoted` is a
//!    directory owned by this user that gives its group and others no
//!    permission bit. The directory is never created here: the serving daemon
//!    provisions it ([`DirState`]).
//! 2. Take the writers' exclusive lock on
//!    [`WRITER_LOCK_FILE`](cerulion_pairing::pair_code::WRITER_LOCK_FILE) in that
//!    directory ([`LockTry`]), polled every [`TICK`].
//! 3. With the lock held, remove the result and leave the request
//!    ([`Step::Leave`]). When the request is gone and no result naming its
//!    nonce shows within [`RELEAVE_AFTER`] of that, leave it again
//!    ([`Step::LeaveAgain`]).
//! 4. On the first result naming the request's nonce, or a refused one naming
//!    `-`, pull a request still on disk back first ([`Step::PullBack`]): with
//!    the lock held, a request there is this writer's own copy left again. Only
//!    then is the code shown ([`Step::ShowArmed`]) and the lock released, so a
//!    later writer replaces the code and this one reads a result naming
//!    another nonce.
//! 5. Follow the result every [`TICK`] and probe `cerulion-netd` every
//!    [`PROBE_EVERY`] until the code's end ([`Outcome`]).
//!
//! A result that is gone never ends a run: it is another writer's removal or
//! the daemon's own after a write that failed. The decisions and the texts
//! are pure: [`Machine::next_step`] decides from what the runner observed
//! ([`Observation`]), and the texts are rendered from fields. On Unix,
//! [`run`] is the runner: it does the file operations each step asks for,
//! holds serving through a [`ServingHost`] and lives in a [`StartEnv`].

use std::path::Path;
use std::time::Duration;

use cerulion_pairing::pair_code::{
    PairCodeNonce, PairCodeRefusal, PairCodeResult, PairCodeState, MAX_FILE_BYTES,
    PAIR_CODE_WINDOW, POLL_INTERVAL, REFUSED_STOPPING, RESULT_FILE,
};

/// Exit code: a desk paired with the code.
pub const EXIT_PAIRED: u8 = 0;
/// Exit code: refused on this robot before a code worked, or an answer
/// `cerulion-remoted` left that could not be read, also once the code is
/// shown.
pub const EXIT_REFUSED: u8 = 1;
/// Exit code: the code expired or was withdrawn, or a newer start replaced it.
pub const EXIT_ENDED_UNUSED: u8 = 2;
/// Exit code: `cerulion-netd` or `cerulion-remoted` could not be reached or
/// stopped.
pub const EXIT_UNREACHABLE: u8 = 3;
/// Exit code: failed tries used the code up.
pub const EXIT_USED_UP: u8 = 4;
/// Exit code: interrupted (Ctrl-C or a termination signal).
pub const EXIT_INTERRUPTED: u8 = 130;

/// How often the runner looks at the handoff files.
pub const TICK: Duration = Duration::from_millis(250);
/// The longest wait for the serving daemon to answer the request.
pub const READY_WAIT: Duration = Duration::from_secs(150);
/// When the waiting note is shown, counted from the granted hold.
pub const NOTE_AFTER: Duration = Duration::from_secs(2);
/// How long a request may be gone with no answer before it is left again:
/// two daemon polls.
pub const RELEAVE_AFTER: Duration = Duration::from_millis(2 * POLL_INTERVAL.as_millis() as u64);
/// How long an interrupted or timed-out handshake still waits for the answer
/// to a request the daemon may have claimed.
pub const CLOSING_WAIT: Duration = RELEAVE_AFTER;
/// How often an armed code's run probes `cerulion-netd`.
pub const PROBE_EVERY: Duration = Duration::from_secs(2);
/// How long a run that read its code's `stopping` result waits to tell which
/// daemon stopped.
pub const SETTLE: Duration = Duration::from_secs(2);
/// How often a settling run probes `cerulion-netd`.
pub const SETTLE_PROBE_EVERY: Duration = Duration::from_millis(500);
/// How long past the code's window a run waits for its end.
pub const END_MARGIN: Duration = Duration::from_secs(30);
/// The time the serving hold allows the daemon to arm the code once it
/// answers.
pub const ARM_ALLOWANCE: Duration = Duration::from_secs(10);
/// The serving hold asked of `cerulion-netd`: the ready wait, the arm, the
/// window and the end margin.
pub const HOLD_TTL_S: u64 = READY_WAIT.as_secs()
    + ARM_ALLOWANCE.as_secs()
    + PAIR_CODE_WINDOW.as_secs()
    + END_MARGIN.as_secs();
/// Draws are accepted below this bound, the largest multiple of 10^8 a `u32`
/// holds, so each code is equally likely.
pub const CODE_ACCEPT_BELOW: u32 = 4_200_000_000;
/// The most draws one code takes before the draw is given up.
pub const CODE_DRAWS: usize = 16;
const CODE_SPACE: u32 = 100_000_000;
const _: () = assert!(CODE_ACCEPT_BELOW.is_multiple_of(CODE_SPACE));
const _: () = assert!((CODE_ACCEPT_BELOW as u64) + (CODE_SPACE as u64) > u32::MAX as u64);
#[cfg(unix)]
const _: () = assert!(
    HOLD_TTL_S >= cerulion_netd::protocol::SERVING_HOLD_MIN_TTL_S
        && HOLD_TTL_S <= cerulion_netd::protocol::SERVING_HOLD_MAX_TTL_S
);

/// Every `cerulion` command a text of this module names.
pub const COMMANDS_NAMED: &[&str] = &[
    "cerulion pair --start",
    "cerulion login",
    "cerulion pair --eid <eid>",
];

/// The first line, before `cerulion-netd` is asked to serve.
pub const ASKING_LINE: &str = "Asking cerulion-netd to serve this robot for code pairing.";
/// Shown once while no answer has come [`NOTE_AFTER`] the granted hold.
pub const WAITING_NOTE: &str = "Waiting for cerulion-remoted to start serving. A first start on \
                                this robot registers it with your account and can take up to 2 \
                                minutes.";
/// The last of the armed lines.
pub const WAITING_FOR_DESK_LINE: &str = "Waiting for a desk. Ctrl-C stops waiting; the code keeps \
                                         working until it is used or expires.";
/// A newer start replaced the code.
pub const REPLACED_MSG: &str =
    "Error: a newer `cerulion pair --start` replaced this code, so it no longer works.";
/// A newer start withdrew the code and armed none for itself.
pub const WITHDRAWN_MSG: &str = "Error: a newer `cerulion pair --start` withdrew this code and \
                                 the robot armed none for it, so a desk that types this code is \
                                 told no code is armed. Run `cerulion pair --start` again for a \
                                 new code.";
/// The daemon withdrew the code after it was armed.
pub const DISARMED_MSG: &str = "Error: cerulion-remoted withdrew this code after it was armed, \
                                so it no longer works. Run `cerulion pair --start` again for a \
                                new code.";
/// `cerulion-netd` stopped while the code was armed.
pub const NETD_STOPPED_MSG: &str = "Error: cerulion-netd stopped while the code was armed, so the \
                                    code no longer works. Run `cerulion pair --start` again for a \
                                    new code.";
/// No end was read by the window plus [`END_MARGIN`].
pub const NO_END_MSG: &str = "Error: cerulion-remoted did not say how the code ended by the end \
                              of its window, so the code no longer works. Run `cerulion pair \
                              --start` again for a new code.";
/// No robot state root resolves.
pub const NO_STATE_ROOT_MSG: &str = "Error: no robot state root is available for this command, \
                                     so no code was made; set CERULION_STATE_ROOT, CERULION_HOME \
                                     or HOME.";
/// Interrupted before a code was armed.
pub const INTERRUPTED_BEFORE_ARM_LINE: &str = "Stopped waiting. No code was made.";
/// The last sentence of a refusal after which no code exists.
const NO_CODE: &str = "No code was made.";
/// What every end of a code that no longer works asks for.
const AGAIN: &str = "Run `cerulion pair --start` again for a new code.";
/// Why a handoff file longer than [`MAX_FILE_BYTES`] is not read.
const TOO_LONG: &str = "it is longer than 512 bytes";
const _: () = assert!(MAX_FILE_BYTES == 512, "TOO_LONG names MAX_FILE_BYTES");
/// The remedy of a `cerulion-netd` that cannot be found or started.
const INSTALL_NETD: &str =
    "Install cerulion-netd beside cerulion or set CERULION_NETD_BIN to its path.";
/// What every failed request to `cerulion-netd` starts with.
const COULD_NOT_ASK: &str = "Error: could not ask cerulion-netd to serve this robot";

/// `cerulion-remoted` stopped while the code was armed.
pub fn remoted_stopped_msg() -> String {
    format!("Error: {REFUSED_STOPPING}. {AGAIN}")
}

/// The daemon did not answer within `waited`.
pub fn never_ready_msg(waited: Duration) -> String {
    format!(
        "Error: cerulion-remoted did not start serving within {} s, so no code was made. The \
         cerulion-netd log says why it could not start cerulion-remoted; run `cerulion pair \
         --start` again once it can.",
        waited.as_secs()
    )
}

/// The daemon refused the request as `refusal` before a code worked.
pub fn refused_msg(refusal: PreArmRefusal) -> String {
    format!("Error: {}. {NO_CODE}", refusal.get().text())
}

/// `cerulion-netd` serves the robot state at `netd_root`, not at `cli_root`.
pub fn state_root_mismatch_msg(netd_root: &Path, cli_root: &Path) -> String {
    format!(
        "Error: cerulion-netd serves the robot state at {} but this command uses {}; run both \
         with the same CERULION_STATE_ROOT or CERULION_HOME. {NO_CODE}",
        netd_root.display(),
        cli_root.display()
    )
}

/// `dir` is not a private directory of this user.
pub fn state_dir_msg(dir: &Path) -> String {
    format!(
        "Error: the robot state directory {} is not a private directory of this user, so no \
         code was made: run `cerulion pair --start` as the user cerulion-netd runs as, and keep \
         the directory owner-only.",
        dir.display()
    )
}

/// The result at `path` could not be read: `error`.
pub fn unreadable_result_msg(path: &Path, error: &str) -> String {
    format!(
        "Error: could not read the answer cerulion-remoted left in {}: {error}. Use cerulion and \
         cerulion-remoted from the same release.",
        path.display()
    )
}

/// The request could not be left in `dir`: `error`.
pub fn handoff_failed_msg(dir: &Path, error: &str) -> String {
    format!(
        "Error: could not leave the pairing code request in {}: {error}. {NO_CODE}",
        dir.display()
    )
}

/// The random source failed: `error`.
pub fn no_random_msg(error: &str) -> String {
    format!("Error: could not draw a random pairing code: {error}. {NO_CODE}")
}

/// The machine lacks serving authority: `refusal` says why.
pub fn serving_authority_msg(refusal: &str) -> String {
    format!("Error: {refusal}. {NO_CODE}")
}

/// The refusal under the network kill switch, before `cerulion-netd` is
/// asked: `None` when `CERULION_NETWORK` (the run and query parse,
/// [`crate::graph_cmd::network_env_kill_from`]) is unset or empty.
pub fn network_refusal_from(network: Option<String>) -> Option<String> {
    crate::graph_cmd::network_env_kill_from(network)
        .refusal_label()
        .map(|label| {
            format!(
                "cerulion pair --start makes this robot serve pairing over the network, which \
                 {label} disables; unset CERULION_NETWORK to start code pairing"
            )
        })
}

/// A code of [`CODE_LEN`](cerulion_pairing::pair_code::CODE_LEN) digits from
/// uniform `u32` draws: the first draw below [`CODE_ACCEPT_BELOW`], reduced
/// modulo 10^8 and zero-padded; `None` when [`CODE_DRAWS`] draws give none.
pub fn code_from_u32s(draws: impl IntoIterator<Item = u32>) -> Option<String> {
    draws
        .into_iter()
        .take(CODE_DRAWS)
        .find(|draw| *draw < CODE_ACCEPT_BELOW)
        .map(|draw| format!("{:08}", draw % CODE_SPACE))
}

/// `unix_s` seconds after the epoch as `HH:MM` in UTC, rounded down.
pub fn utc_hh_mm(unix_s: u64) -> String {
    let day_s = unix_s % 86_400;
    format!("{:02}:{:02}", day_s / 3_600, day_s % 3_600 / 60)
}

/// A span of `s` seconds in words: whole minutes when it is a multiple of 60
/// and at least one, else seconds.
pub fn span_words(s: u64) -> String {
    if s >= 60 && s.is_multiple_of(60) {
        count_words(s / 60, "minute", "minutes")
    } else {
        count_words(s, "second", "seconds")
    }
}

/// `n` and the noun that agrees with it.
pub fn count_words(n: u64, one: &str, many: &str) -> String {
    format!("{n} {}", if n == 1 { one } else { many })
}

/// What the runner learned of the result file through its open handle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FileFacts {
    /// Whether it is a regular file.
    pub is_file: bool,
    /// Its link count.
    pub nlink: u64,
    /// Its owner.
    pub uid: u32,
    /// Its mode bits.
    pub mode: u32,
    /// Its length in bytes.
    pub len: u64,
}

/// Whether a result file the runner opened is to be read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Read it.
    Read,
    /// It was renamed over or removed after it was opened: look again later.
    Gone,
    /// It is not a file the daemon writes, for the reason given.
    Unreadable(&'static str),
}

/// The verdict on a result file whose handle shows `facts`, for a runner
/// whose effective user is `euid`. A link count of zero is a file renamed over
/// or removed since the open, so it is gone, not refused.
pub fn result_verdict(facts: &FileFacts, euid: u32) -> Verdict {
    if facts.nlink == 0 {
        Verdict::Gone
    } else if !facts.is_file {
        Verdict::Unreadable("it is not a regular file")
    } else if facts.nlink > 1 {
        Verdict::Unreadable("it has another hard link")
    } else if facts.uid != euid {
        Verdict::Unreadable("it is not owned by this user")
    } else if facts.mode & 0o077 != 0 {
        Verdict::Unreadable("it is not owner-only")
    } else if facts.len > MAX_FILE_BYTES as u64 {
        Verdict::Unreadable(TOO_LONG)
    } else {
        Verdict::Read
    }
}

/// The state of `<state_root>/remoted` as the runner found it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DirState {
    /// It does not exist yet.
    Missing,
    /// A directory owned by this user, owner-only.
    Ready,
    /// Anything else: another owner, a mode with group or other bits, not a
    /// directory, or a check that failed.
    NotPrivate,
}

/// What the runner's try of the writers' lock gave.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LockTry {
    /// The lock is held.
    Taken,
    /// Another writer holds it.
    Busy,
}

/// The result file as the runner read it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ResultSeen {
    /// No result is there, or it was replaced while read.
    Gone,
    /// A result line.
    Line(PairCodeResult),
    /// It cannot be read or does not parse, for the reason given.
    Unreadable(String),
}

/// What one `cerulion-netd` status probe gave.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NetdProbe {
    /// netd answered; `startup_error` is whether it reports that the remote
    /// server failed or stopped.
    Status {
        /// Whether netd reports a remote server startup error.
        startup_error: bool,
    },
    /// netd could not be reached.
    Unreachable,
}

/// How pulling the request back went.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pulled {
    /// It was taken back before the daemon claimed it.
    Back,
    /// It was gone, or could not be taken back: the daemon may have claimed it.
    Claimed,
}

/// What the runner observed since the last step. It fills the parts
/// [`Machine::wants`] asks for.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Observation {
    /// Time since the hold was granted, on a monotonic clock.
    pub now: Duration,
    /// Seconds since the Unix epoch, for display only.
    pub unix_s: u64,
    /// Whether an interrupt arrived.
    pub interrupted: bool,
    /// The remoted directory, when asked for.
    pub dir: Option<DirState>,
    /// The lock try, when asked for.
    pub lock: Option<LockTry>,
    /// Whether the request file is there, when the files are asked for.
    pub request_present: Option<bool>,
    /// The result, when the files are asked for.
    pub result: Option<ResultSeen>,
    /// The probe, when asked for.
    pub probe: Option<NetdProbe>,
    /// How the last [`Step::PullBack`] went.
    pub pulled: Option<Pulled>,
    /// Why the last lock try, [`Step::Leave`] or [`Step::LeaveAgain`] failed.
    pub fault: Option<String>,
}

impl Observation {
    /// An observation at `now` with nothing seen.
    pub fn at(now: Duration, unix_s: u64) -> Self {
        Observation {
            now,
            unix_s,
            interrupted: false,
            dir: None,
            lock: None,
            request_present: None,
            result: None,
            probe: None,
            pulled: None,
            fault: None,
        }
    }
}

/// What the runner is to observe before the next step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Wants {
    /// Check the remoted directory.
    pub dir: bool,
    /// Try the writers' lock.
    pub lock: bool,
    /// Look at the request and the result.
    pub files: bool,
    /// Probe `cerulion-netd`.
    pub probe: bool,
}

/// The facts the armed lines show, from the first result naming the
/// request's nonce.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ArmedFacts {
    /// The robot's endpoint id.
    pub endpoint_id: [u8; 32],
    /// The window in seconds.
    pub ttl_s: u64,
    /// The tries every desk shares.
    pub tries: u8,
    /// When the window ends, in seconds since the Unix epoch.
    pub expires_unix_s: u64,
}

/// What the runner does next.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Step {
    /// Sleep [`TICK`], then observe.
    Wait,
    /// Show [`WAITING_NOTE`].
    Note,
    /// Remove the result and leave the request.
    Leave,
    /// Remove the result and leave the request again.
    LeaveAgain,
    /// Take the request back, reporting how in [`Observation::pulled`].
    PullBack,
    /// Show the armed lines ([`armed_report`]).
    ShowArmed(ArmedFacts),
    /// Show a try-failed line for each count from `from` to `to`.
    TriesFailed {
        /// The first failed count not shown yet.
        from: u8,
        /// The last.
        to: u8,
        /// The tries every desk shares.
        tries: u8,
    },
    /// Show the outcome ([`outcome_report`]) and exit with its code.
    End(Outcome),
}

/// A refusal a request gets before its code is armed: any but `stopping`,
/// which withdraws a code that was armed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PreArmRefusal(PairCodeRefusal);

impl PreArmRefusal {
    /// `refusal`, unless it is `stopping`.
    pub fn new(refusal: PairCodeRefusal) -> Option<Self> {
        (!matches!(refusal, PairCodeRefusal::Stopping)).then_some(PreArmRefusal(refusal))
    }

    /// The refusal.
    pub fn get(self) -> PairCodeRefusal {
        self.0
    }
}

/// How a run ends.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// A desk paired.
    Paired {
        /// The desk's label token.
        label: String,
        /// The account it was paired as.
        account: [u8; 32],
    },
    /// Every try failed.
    Exhausted {
        /// Failed tries.
        failed: u8,
        /// Tries.
        tries: u8,
    },
    /// The window ended.
    Expired {
        /// Failed tries.
        failed: u8,
        /// Tries.
        tries: u8,
        /// The window in seconds.
        ttl_s: u64,
    },
    /// A newer start's code replaced this one.
    Replaced,
    /// A newer start withdrew this code and armed none.
    Withdrawn,
    /// The daemon withdrew this code after it was armed.
    Disarmed,
    /// `cerulion-remoted` stopped while the code was armed.
    RemotedStopped,
    /// `cerulion-netd` stopped while the code was armed.
    NetdStopped,
    /// No end was read by the window plus [`END_MARGIN`].
    NoEnd,
    /// No answer came within `waited`.
    NeverReady {
        /// The wait used.
        waited: Duration,
    },
    /// The daemon refused the request before its code was armed.
    Refused(PreArmRefusal),
    /// The remoted directory is not private to this user.
    StateDir,
    /// The result could not be read.
    UnreadableResult {
        /// Why.
        error: String,
    },
    /// The request could not be left.
    HandoffFailed {
        /// Why.
        error: String,
    },
    /// Interrupted before a code was armed.
    InterruptedBeforeArm,
    /// Interrupted while the code was armed.
    InterruptedArmed {
        /// When the code's window ends, in seconds since the Unix epoch.
        until_unix_s: u64,
    },
}

impl Outcome {
    /// The exit code of this end.
    pub fn exit_code(&self) -> u8 {
        match self {
            Outcome::Paired { .. } => EXIT_PAIRED,
            Outcome::Refused(_)
            | Outcome::StateDir
            | Outcome::UnreadableResult { .. }
            | Outcome::HandoffFailed { .. } => EXIT_REFUSED,
            Outcome::Expired { .. }
            | Outcome::Replaced
            | Outcome::Withdrawn
            | Outcome::Disarmed => EXIT_ENDED_UNUSED,
            Outcome::RemotedStopped
            | Outcome::NetdStopped
            | Outcome::NoEnd
            | Outcome::NeverReady { .. } => EXIT_UNREACHABLE,
            Outcome::Exhausted { .. } => EXIT_USED_UP,
            Outcome::InterruptedBeforeArm | Outcome::InterruptedArmed { .. } => EXIT_INTERRUPTED,
        }
    }
}

/// Lines for stdout and stderr, each ending in a newline.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Report {
    /// The state lines.
    pub stdout: String,
    /// The lines for the user.
    pub stderr: String,
}

impl Report {
    fn out(&mut self, line: &str) {
        self.stdout.push_str(line);
        self.stdout.push('\n');
    }

    fn err(&mut self, line: &str) {
        self.stderr.push_str(line);
        self.stderr.push('\n');
    }
}

/// The stdout line of an armed code; `port` renders `-` when unknown.
pub fn pair_code_stdout(code: &str, facts: &ArmedFacts, port: Option<u16>) -> String {
    format!(
        "pair-code: code={code} eid={} port={} expires_in_s={} tries={}",
        hex::encode(facts.endpoint_id),
        port.map_or_else(|| "-".to_string(), |p| p.to_string()),
        facts.ttl_s,
        facts.tries
    )
}

/// The stderr line naming the code.
pub fn code_line(code: &str) -> String {
    format!("Pairing code: {code}")
}

/// The stderr line naming when the code expires and its tries.
pub fn expiry_line(facts: &ArmedFacts) -> String {
    format!(
        "It pairs one desk and expires at {} UTC (in {}). All desks share {}.",
        utc_hh_mm(facts.expires_unix_s),
        span_words(facts.ttl_s),
        count_words(u64::from(facts.tries), "try", "tries")
    )
}

/// The stderr line telling the desk how to use the code: with the port when
/// it is known, so a desk with no internet can dial the robot directly.
pub fn desk_hint_line(endpoint_id: &[u8; 32], port: Option<u16>) -> String {
    let eid = hex::encode(endpoint_id);
    let hint = format!("On the desk, run `cerulion pair --eid {eid}` and type the code.");
    match port {
        Some(port) => {
            format!("{hint} Add `--addr <robot-ip>:{port}` when the desk has no internet.")
        }
        None => hint,
    }
}

/// The armed lines.
pub fn armed_report(code: &str, facts: &ArmedFacts, port: Option<u16>) -> Report {
    let mut report = Report::default();
    report.out(&pair_code_stdout(code, facts, port));
    report.err(&code_line(code));
    report.err(&expiry_line(facts));
    report.err(&desk_hint_line(&facts.endpoint_id, port));
    report.err(WAITING_FOR_DESK_LINE);
    report
}

/// The stdout line of one failed try.
pub fn try_failed_stdout(failed: u8, tries: u8) -> String {
    format!("try-failed: failed={failed} tries={tries}")
}

/// The stderr line of one failed try.
pub fn try_failed_line(failed: u8, tries: u8) -> String {
    format!("A pairing try failed: {failed} of {tries} tries failed.")
}

/// A try-failed pair of lines for each count from `from` to `to`.
pub fn tries_failed_report(from: u8, to: u8, tries: u8) -> Report {
    let mut report = Report::default();
    for failed in from..=to {
        report.out(&try_failed_stdout(failed, tries));
        report.err(&try_failed_line(failed, tries));
    }
    report
}

/// The stdout line of a paired code.
pub fn paired_stdout(label: &str, account: &[u8; 32]) -> String {
    format!("paired: desk={label} account={}", hex::encode(account))
}

/// The stderr line of a paired code.
pub fn paired_line(label: &str, account: &[u8; 32]) -> String {
    format!(
        "Paired with desk {label} (account {}). The code is used up.",
        hex::encode(account)
    )
}

/// The stdout line of a used-up code.
pub fn exhausted_stdout(failed: u8, tries: u8) -> String {
    format!("exhausted: failed={failed} tries={tries}")
}

/// The stderr line of a used-up code.
pub fn exhausted_line(failed: u8) -> String {
    format!(
        "The code was used up by {} and no desk paired. {AGAIN}",
        count_words(u64::from(failed), "failed try", "failed tries")
    )
}

/// The stdout line of an expired code.
pub fn expired_stdout(failed: u8, tries: u8) -> String {
    format!("expired: failed={failed} tries={tries}")
}

/// The stderr line of an expired code.
pub fn expired_line(ttl_s: u64) -> String {
    format!(
        "The code expired after {} and no desk paired. {AGAIN}",
        span_words(ttl_s)
    )
}

/// The stderr line of a run interrupted while its code is armed.
pub fn interrupted_armed_line(until_unix_s: u64) -> String {
    format!(
        "Stopped waiting. The code keeps working until {} UTC unless a desk uses it first.",
        utc_hh_mm(until_unix_s)
    )
}

/// The lines of `outcome`, for the remoted directory `dir`.
pub fn outcome_report(outcome: &Outcome, dir: &Path) -> Report {
    let mut report = Report::default();
    match outcome {
        Outcome::Paired { label, account } => {
            report.out(&paired_stdout(label, account));
            report.err(&paired_line(label, account));
        }
        Outcome::Exhausted { failed, tries } => {
            report.out(&exhausted_stdout(*failed, *tries));
            report.err(&exhausted_line(*failed));
        }
        Outcome::Expired {
            failed,
            tries,
            ttl_s,
        } => {
            report.out(&expired_stdout(*failed, *tries));
            report.err(&expired_line(*ttl_s));
        }
        Outcome::Replaced => report.err(REPLACED_MSG),
        Outcome::Withdrawn => report.err(WITHDRAWN_MSG),
        Outcome::Disarmed => report.err(DISARMED_MSG),
        Outcome::RemotedStopped => report.err(&remoted_stopped_msg()),
        Outcome::NetdStopped => report.err(NETD_STOPPED_MSG),
        Outcome::NoEnd => report.err(NO_END_MSG),
        Outcome::NeverReady { waited } => report.err(&never_ready_msg(*waited)),
        Outcome::Refused(refusal) => report.err(&refused_msg(*refusal)),
        Outcome::StateDir => report.err(&state_dir_msg(dir)),
        Outcome::UnreadableResult { error } => {
            report.err(&unreadable_result_msg(&dir.join(RESULT_FILE), error))
        }
        Outcome::HandoffFailed { error } => report.err(&handoff_failed_msg(dir, error)),
        Outcome::InterruptedBeforeArm => report.err(INTERRUPTED_BEFORE_ARM_LINE),
        Outcome::InterruptedArmed { until_unix_s } => {
            report.err(&interrupted_armed_line(*until_unix_s))
        }
    }
    report
}

/// The handshake: the lock is held and the request left.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Handshake {
    /// Since when the request has been seen gone with no answer.
    gone_since: Option<Duration>,
    /// Whether the request was seen gone since it was first left.
    claimed_seen: bool,
    /// The wall-clock second of the last look before the request was first
    /// seen gone that found it on disk, or of the look that left it. The
    /// daemon claims a request, and arms its code, no earlier, so a window
    /// counted from this second ends no later than the code's.
    arm_floor_unix_s: u64,
    /// How a pull-back went, once one ran.
    pulled: Option<Pulled>,
    /// Why a leave or leave again failed, kept through the pull-back it
    /// calls for.
    fault: Option<String>,
}

/// The armed code being followed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Armed {
    armed_at: Duration,
    ttl_s: u64,
    until_unix_s: u64,
    shown_failed: u8,
    startup_clear_seen: bool,
    next_probe: Duration,
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum Phase {
    WaitingForDir,
    Locking,
    Handshake(Handshake),
    /// Interrupted, or past the ready deadline, after the request was taken
    /// back: a request the daemon may have claimed is answered until `until`.
    /// An interrupt during the wait counts as one that closed it.
    Closing {
        until: Duration,
        interrupted: bool,
        arm_floor_unix_s: u64,
    },
    Armed(Armed),
    Settling {
        until: Duration,
        next_probe: Duration,
    },
    Done,
}

/// The decisions of one run (see the module docs).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Machine {
    nonce: PairCodeNonce,
    ready_deadline: Duration,
    note_shown: bool,
    phase: Phase,
    /// The end a step took, which every later step repeats.
    ended: Option<Outcome>,
}

/// What a result line says about a request whose nonce is `nonce`.
enum Answer<'a> {
    /// It names the nonce.
    Own(&'a PairCodeResult),
    /// A `refused` result naming `-`: the grammar gives `-` only to a
    /// refusal before an arm.
    Dash(PreArmRefusal),
    /// It names another nonce.
    Other(&'a PairCodeResult),
}

fn answer_to(nonce: PairCodeNonce, result: &PairCodeResult) -> Answer<'_> {
    match (result.nonce(), result.state()) {
        (Some(n), _) if n == nonce => Answer::Own(result),
        (None, PairCodeState::Refused(refusal)) => match PreArmRefusal::new(*refusal) {
            Some(refusal) => Answer::Dash(refusal),
            None => Answer::Other(result),
        },
        _ => Answer::Other(result),
    }
}

impl Machine {
    /// A run for the request whose nonce is `nonce`, waiting at most
    /// `ready_wait` from the granted hold for its answer.
    pub fn new(nonce: PairCodeNonce, ready_wait: Duration) -> Self {
        Machine {
            nonce,
            ready_deadline: ready_wait,
            note_shown: false,
            phase: Phase::WaitingForDir,
            ended: None,
        }
    }

    /// What the runner is to observe before calling [`Machine::next_step`].
    pub fn wants(&self, now: Duration) -> Wants {
        match &self.phase {
            Phase::WaitingForDir => Wants {
                dir: true,
                ..Wants::default()
            },
            Phase::Locking => Wants {
                lock: true,
                ..Wants::default()
            },
            Phase::Handshake(_) | Phase::Closing { .. } => Wants {
                files: true,
                ..Wants::default()
            },
            Phase::Armed(armed) => Wants {
                files: true,
                probe: now >= armed.next_probe,
                ..Wants::default()
            },
            Phase::Settling { next_probe, .. } => Wants {
                probe: now >= *next_probe,
                ..Wants::default()
            },
            Phase::Done => Wants::default(),
        }
    }

    /// Whether the runner is to hold the writers' lock now.
    pub fn holds_lock(&self) -> bool {
        matches!(self.phase, Phase::Handshake(_) | Phase::Closing { .. })
    }

    /// The next step from `obs`; once a step ended the run, that same end.
    pub fn next_step(&mut self, obs: &Observation) -> Step {
        if let Some(outcome) = &self.ended {
            return Step::End(outcome.clone());
        }
        let step = match std::mem::replace(&mut self.phase, Phase::Done) {
            Phase::WaitingForDir => self.waiting_for_dir(obs),
            Phase::Locking => self.locking(obs),
            Phase::Handshake(handshake) => self.handshake(handshake, obs),
            Phase::Closing {
                until,
                interrupted,
                arm_floor_unix_s,
            } => self.closing(until, interrupted, arm_floor_unix_s, obs),
            Phase::Armed(armed) => self.armed(armed, obs),
            Phase::Settling { until, next_probe } => self.settling(until, next_probe, obs),
            Phase::Done => Step::Wait,
        };
        if let Step::End(outcome) = &step {
            self.phase = Phase::Done;
            self.ended = Some(outcome.clone());
        }
        step
    }

    /// The step a pre-arm phase takes when nothing else is due: the note
    /// once, else a wait.
    fn note_or_wait(&mut self, now: Duration) -> Step {
        if !self.note_shown && now >= NOTE_AFTER {
            self.note_shown = true;
            Step::Note
        } else {
            Step::Wait
        }
    }

    fn never_ready(&self) -> Step {
        Step::End(Outcome::NeverReady {
            waited: self.ready_deadline,
        })
    }

    fn waiting_for_dir(&mut self, obs: &Observation) -> Step {
        self.phase = Phase::WaitingForDir;
        if obs.interrupted {
            return Step::End(Outcome::InterruptedBeforeArm);
        }
        match obs.dir {
            Some(DirState::NotPrivate) => return Step::End(Outcome::StateDir),
            Some(DirState::Ready) => {
                self.phase = Phase::Locking;
                return self.locking(obs);
            }
            Some(DirState::Missing) | None => {}
        }
        if obs.now >= self.ready_deadline {
            return self.never_ready();
        }
        self.note_or_wait(obs.now)
    }

    fn locking(&mut self, obs: &Observation) -> Step {
        self.phase = Phase::Locking;
        if obs.interrupted {
            return Step::End(Outcome::InterruptedBeforeArm);
        }
        if let Some(error) = &obs.fault {
            return Step::End(Outcome::HandoffFailed {
                error: error.clone(),
            });
        }
        if obs.lock == Some(LockTry::Taken) {
            self.phase = Phase::Handshake(Handshake {
                gone_since: None,
                claimed_seen: false,
                arm_floor_unix_s: obs.unix_s,
                pulled: None,
                fault: None,
            });
            return Step::Leave;
        }
        if obs.now >= self.ready_deadline {
            return self.never_ready();
        }
        self.note_or_wait(obs.now)
    }

    fn handshake(&mut self, mut hs: Handshake, obs: &Observation) -> Step {
        let present = obs.request_present == Some(true);
        let gone = obs.request_present == Some(false);
        // A request gone before any pull-back was claimed or swept; one gone
        // after a pull-back may be the pulled one.
        if hs.pulled.is_none() && obs.pulled.is_none() && gone {
            hs.claimed_seen = true;
        }
        if present && !hs.claimed_seen {
            hs.arm_floor_unix_s = obs.unix_s;
        }
        if let Some(pulled) = obs.pulled {
            hs.pulled = Some(pulled);
        }
        if hs.fault.is_none() {
            hs.fault.clone_from(&obs.fault);
        }
        let answer = match &obs.result {
            Some(ResultSeen::Line(result)) => match answer_to(self.nonce, result) {
                Answer::Other(_) => None,
                answer => Some(answer),
            },
            _ => None,
        };
        let unreadable = match &obs.result {
            Some(ResultSeen::Unreadable(error)) => Some(error.clone()),
            _ => None,
        };
        let ends = answer.is_some()
            || unreadable.is_some()
            || hs.fault.is_some()
            || obs.interrupted
            || obs.now >= self.ready_deadline;
        // Every way out of the handshake first takes back a request left
        // again, so the daemon arms no code nobody watches.
        if ends && present && hs.pulled.is_none() {
            self.phase = Phase::Handshake(hs);
            return Step::PullBack;
        }
        if let Some(answer) = answer {
            return match answer {
                Answer::Own(result) => self.own_first_answer(result, obs, hs.arm_floor_unix_s),
                Answer::Dash(refusal) => Step::End(Outcome::Refused(refusal)),
                Answer::Other(_) => Step::Wait,
            };
        }
        if let Some(error) = unreadable {
            return Step::End(Outcome::UnreadableResult { error });
        }
        // A failed leave ends the run at once, with no closing wait for a
        // copy seen claimed: no code it might arm is shown.
        if let Some(error) = hs.fault {
            return Step::End(Outcome::HandoffFailed { error });
        }
        if obs.interrupted || obs.now >= self.ready_deadline {
            let may_answer = hs.claimed_seen || hs.pulled == Some(Pulled::Claimed);
            if !may_answer {
                return if obs.interrupted {
                    Step::End(Outcome::InterruptedBeforeArm)
                } else {
                    self.never_ready()
                };
            }
            self.phase = Phase::Closing {
                until: obs.now + CLOSING_WAIT,
                interrupted: obs.interrupted,
                arm_floor_unix_s: hs.arm_floor_unix_s,
            };
            return Step::Wait;
        }
        if present {
            hs.gone_since = None;
        } else if gone {
            let since = *hs.gone_since.get_or_insert(obs.now);
            if obs.now.saturating_sub(since) >= RELEAVE_AFTER {
                hs.gone_since = None;
                self.phase = Phase::Handshake(hs);
                return Step::LeaveAgain;
            }
        }
        self.phase = Phase::Handshake(hs);
        self.note_or_wait(obs.now)
    }

    /// The first result naming the request's nonce, with no copy of the
    /// request left on disk. The window shown is counted from
    /// `arm_floor_unix_s` ([`Handshake`]), so it never ends after the code's.
    fn own_first_answer(
        &mut self,
        result: &PairCodeResult,
        obs: &Observation,
        arm_floor_unix_s: u64,
    ) -> Step {
        match result.state() {
            PairCodeState::Refused(refusal) => match PreArmRefusal::new(*refusal) {
                Some(refusal) => Step::End(Outcome::Refused(refusal)),
                None => {
                    self.phase = Phase::Settling {
                        until: obs.now + SETTLE,
                        next_probe: obs.now,
                    };
                    Step::Wait
                }
            },
            PairCodeState::Armed
            | PairCodeState::Paired(_)
            | PairCodeState::Exhausted
            | PairCodeState::Expired => {
                let facts = ArmedFacts {
                    endpoint_id: *result.endpoint_id(),
                    ttl_s: result.ttl_s(),
                    tries: result.tries(),
                    expires_unix_s: arm_floor_unix_s.saturating_add(result.ttl_s()),
                };
                self.phase = Phase::Armed(Armed {
                    armed_at: obs.now,
                    ttl_s: result.ttl_s(),
                    until_unix_s: facts.expires_unix_s,
                    shown_failed: 0,
                    startup_clear_seen: false,
                    next_probe: obs.now,
                });
                Step::ShowArmed(facts)
            }
        }
    }

    fn closing(
        &mut self,
        until: Duration,
        interrupted: bool,
        arm_floor_unix_s: u64,
        obs: &Observation,
    ) -> Step {
        let interrupted = interrupted || obs.interrupted;
        let refused = |refusal: PreArmRefusal| {
            Step::End(if interrupted {
                Outcome::InterruptedBeforeArm
            } else {
                Outcome::Refused(refusal)
            })
        };
        match &obs.result {
            Some(ResultSeen::Line(result)) => match answer_to(self.nonce, result) {
                Answer::Own(result) => {
                    return match result.refusal().and_then(PreArmRefusal::new) {
                        Some(refusal) => refused(refusal),
                        None => self.own_first_answer(result, obs, arm_floor_unix_s),
                    }
                }
                Answer::Dash(refusal) => return refused(refusal),
                Answer::Other(_) => {}
            },
            // An answer that cannot be read may be an armed code, so the
            // wait ends on it as the handshake does, never on "no code".
            Some(ResultSeen::Unreadable(error)) => {
                return Step::End(Outcome::UnreadableResult {
                    error: error.clone(),
                })
            }
            Some(ResultSeen::Gone) | None => {}
        }
        if obs.now >= until {
            return if interrupted {
                Step::End(Outcome::InterruptedBeforeArm)
            } else {
                self.never_ready()
            };
        }
        self.phase = Phase::Closing {
            until,
            interrupted,
            arm_floor_unix_s,
        };
        Step::Wait
    }

    fn armed(&mut self, mut armed: Armed, obs: &Observation) -> Step {
        let mut remoted_stopped = false;
        if let Some(NetdProbe::Status { startup_error }) = obs.probe {
            remoted_stopped = startup_error && armed.startup_clear_seen;
            armed.startup_clear_seen |= !startup_error;
        }
        if obs.probe.is_some() {
            armed.next_probe = obs.now + PROBE_EVERY;
        }
        self.phase = Phase::Armed(armed);
        // An end the look found comes before an interrupt in the same look,
        // so a code that ended is never said to keep working. An answer that
        // cannot be read is no end, so an interrupt in its look comes first.
        match &obs.result {
            Some(ResultSeen::Line(result)) => match answer_to(self.nonce, result) {
                Answer::Own(result) => {
                    if let Some(step) = self.own_state(result, obs) {
                        return step;
                    }
                }
                Answer::Dash(_) => return Step::End(Outcome::Withdrawn),
                Answer::Other(other) => {
                    return Step::End(match other.refusal().and_then(PreArmRefusal::new) {
                        Some(_) => Outcome::Withdrawn,
                        None => Outcome::Replaced,
                    })
                }
            },
            Some(ResultSeen::Unreadable(error)) if !obs.interrupted => {
                return Step::End(Outcome::UnreadableResult {
                    error: error.clone(),
                })
            }
            Some(ResultSeen::Unreadable(_) | ResultSeen::Gone) | None => {}
        }
        if remoted_stopped {
            return Step::End(Outcome::RemotedStopped);
        }
        let end = armed.armed_at + Duration::from_secs(armed.ttl_s) + END_MARGIN;
        if obs.now >= end {
            return Step::End(Outcome::NoEnd);
        }
        if obs.interrupted {
            return Step::End(Outcome::InterruptedArmed {
                until_unix_s: armed.until_unix_s,
            });
        }
        Step::Wait
    }

    /// The step a result naming the request's nonce calls for while its code
    /// is armed; `None` when it calls for none.
    fn own_state(&mut self, result: &PairCodeResult, obs: &Observation) -> Option<Step> {
        let Phase::Armed(armed) = &mut self.phase else {
            return None;
        };
        let (failed, tries) = (result.failed(), result.tries());
        // The failed tries the result itself does not announce.
        let unannounced = match result.state() {
            PairCodeState::Exhausted => failed.saturating_sub(1),
            _ => failed,
        };
        if failed < armed.shown_failed {
            armed.shown_failed = failed;
        }
        if unannounced > armed.shown_failed {
            let from = armed.shown_failed + 1;
            armed.shown_failed = unannounced;
            return Some(Step::TriesFailed {
                from,
                to: unannounced,
                tries,
            });
        }
        Some(match result.state() {
            PairCodeState::Armed => return None,
            PairCodeState::Paired(desk) => Step::End(Outcome::Paired {
                label: desk.label().to_string(),
                account: *desk.account(),
            }),
            PairCodeState::Exhausted => Step::End(Outcome::Exhausted { failed, tries }),
            PairCodeState::Expired => Step::End(Outcome::Expired {
                failed,
                tries,
                ttl_s: result.ttl_s(),
            }),
            PairCodeState::Refused(PairCodeRefusal::Stopping) => {
                self.phase = Phase::Settling {
                    until: obs.now + SETTLE,
                    next_probe: obs.now,
                };
                Step::Wait
            }
            PairCodeState::Refused(_) => Step::End(Outcome::Disarmed),
        })
    }

    fn settling(&mut self, until: Duration, next_probe: Duration, obs: &Observation) -> Step {
        match obs.probe {
            Some(NetdProbe::Unreachable) => return Step::End(Outcome::NetdStopped),
            Some(NetdProbe::Status {
                startup_error: true,
            }) => return Step::End(Outcome::RemotedStopped),
            Some(NetdProbe::Status {
                startup_error: false,
            })
            | None => {}
        }
        if obs.interrupted || obs.now >= until {
            return Step::End(Outcome::RemotedStopped);
        }
        let next_probe = if obs.probe.is_some() {
            obs.now + SETTLE_PROBE_EVERY
        } else {
            next_probe
        };
        self.phase = Phase::Settling { until, next_probe };
        Step::Wait
    }
}

/// The remoted state subdirectory of the robot state root
/// (`cerulion_remoted::config::REMOTED_SUBDIR`).
#[cfg(unix)]
const REMOTED_SUBDIR: &str = "remoted";

#[cfg(unix)]
mod handoff;
#[cfg(unix)]
mod netd;
#[cfg(unix)]
mod run;

#[cfg(unix)]
pub use hold::{hold_refusal, HoldRefusal};
#[cfg(unix)]
pub use netd::{resolve_state_root, NetdServing, ProcessEnv, PROBE_DEADLINE};
#[cfg(unix)]
pub use run::{run, HeldServing, ServingHost, StartEnv};

#[cfg(unix)]
mod hold {
    use std::path::{Path, PathBuf};

    use cerulion_netd::client::ClientError;
    use cerulion_netd::egress::{
        NO_STATE_ROOT, REMOTE_SERVING_UNAVAILABLE, STATE_ROOT_NOT_ABSOLUTE, STATE_ROOT_NOT_UTF8,
    };
    use cerulion_netd::protocol::{
        PROTOCOL_VERSION, SERVING_HOLD_MIN_DAEMON_VERSION, SERVING_HOLD_TTL_REFUSAL,
    };

    use super::{COULD_NOT_ASK, EXIT_REFUSED, EXIT_UNREACHABLE, INSTALL_NETD, NO_CODE};

    /// Why `cerulion-netd` did not hold serving, classified by the error's
    /// variant and whether it carries an OS error code. A netd error is a
    /// refusal only when its whole text equals one of netd's own refusal
    /// constants; no part of a text netd built is parsed.
    #[derive(Clone, Debug, PartialEq, Eq)]
    pub enum HoldRefusal {
        /// No `cerulion-netd` binary was found beside this one.
        NotFound {
            /// The directory looked in, when known.
            exe_dir: Option<PathBuf>,
        },
        /// The binary could not be started.
        CannotStart {
            /// The binary.
            bin: PathBuf,
            /// The OS error.
            os: String,
        },
        /// The control socket could not be reached.
        Unreachable {
            /// The socket.
            socket: PathBuf,
            /// The OS error.
            os: String,
        },
        /// netd was started but never opened its socket.
        NoSocket {
            /// The socket.
            socket: PathBuf,
            /// Its startup log.
            log: PathBuf,
        },
        /// The connection failed mid-exchange.
        ConnectionFailed {
            /// The OS error, when it is one.
            os: Option<String>,
        },
        /// netd refuses to serve this robot: one of its own refusals.
        WillNotServe {
            /// netd's refusal, verbatim.
            netd: String,
        },
        /// Any other netd error.
        NetdFailed {
            /// netd's error, its trailing period dropped.
            error: String,
        },
        /// netd is older than the hold verb.
        TooOld {
            /// The protocol netd speaks.
            daemon: u32,
        },
        /// netd's answer does not follow the protocol.
        NetdProtocol,
    }

    /// The refusals netd gives a hold, quoted verbatim.
    const NETD_REFUSALS: [&str; 6] = [
        REMOTE_SERVING_UNAVAILABLE,
        NO_STATE_ROOT,
        STATE_ROOT_NOT_ABSOLUTE,
        STATE_ROOT_NOT_UTF8,
        cerulion_netd::serving_login::REFUSAL,
        SERVING_HOLD_TTL_REFUSAL,
    ];

    /// Classify `error` from asking netd to hold serving. `daemon_protocol`
    /// is the protocol the connected netd announced, when a connection was
    /// made; `exe_dir` is the directory of this binary.
    pub fn hold_refusal(
        error: &ClientError,
        daemon_protocol: Option<u32>,
        exe_dir: Option<&Path>,
    ) -> HoldRefusal {
        match error {
            ClientError::Spawn { bin, source } => match source.raw_os_error() {
                None => HoldRefusal::NotFound {
                    exe_dir: exe_dir.map(Path::to_path_buf),
                },
                Some(_) => HoldRefusal::CannotStart {
                    bin: bin.clone(),
                    os: source.to_string(),
                },
            },
            ClientError::Connect { socket, source, .. } => match source.raw_os_error() {
                Some(_) => HoldRefusal::Unreachable {
                    socket: socket.clone(),
                    os: source.to_string(),
                },
                None => HoldRefusal::NoSocket {
                    socket: socket.clone(),
                    log: cerulion_netd::client::spawn_log_for(socket),
                },
            },
            ClientError::Io(e) => HoldRefusal::ConnectionFailed {
                os: e.raw_os_error().map(|_| e.to_string()),
            },
            ClientError::Netd { error, .. } if NETD_REFUSALS.contains(&error.as_str()) => {
                HoldRefusal::WillNotServe {
                    netd: error.clone(),
                }
            }
            ClientError::Netd { error, .. } => HoldRefusal::NetdFailed {
                error: error.strip_suffix('.').unwrap_or(error).to_string(),
            },
            ClientError::Protocol(_) => match daemon_protocol {
                Some(daemon) if daemon < SERVING_HOLD_MIN_DAEMON_VERSION => {
                    HoldRefusal::TooOld { daemon }
                }
                _ => HoldRefusal::NetdProtocol,
            },
            ClientError::MirrorLost(_) => HoldRefusal::NetdProtocol,
        }
    }

    impl HoldRefusal {
        /// The exit code of this refusal.
        pub fn exit_code(&self) -> u8 {
            match self {
                HoldRefusal::WillNotServe { .. }
                | HoldRefusal::TooOld { .. }
                | HoldRefusal::NetdProtocol => EXIT_REFUSED,
                HoldRefusal::NotFound { .. }
                | HoldRefusal::CannotStart { .. }
                | HoldRefusal::Unreachable { .. }
                | HoldRefusal::NoSocket { .. }
                | HoldRefusal::ConnectionFailed { .. }
                | HoldRefusal::NetdFailed { .. } => EXIT_UNREACHABLE,
            }
        }

        /// The line shown for this refusal.
        pub fn text(&self) -> String {
            match self {
                HoldRefusal::NotFound { exe_dir } => {
                    let place = exe_dir
                        .as_ref()
                        .map(|dir| format!(" in {}", dir.display()))
                        .unwrap_or_default();
                    format!(
                        "{COULD_NOT_ASK}: no cerulion-netd was found beside this cerulion{place}. \
                         {NO_CODE} {INSTALL_NETD}"
                    )
                }
                HoldRefusal::CannotStart { bin, os } => format!(
                    "{COULD_NOT_ASK}: could not start cerulion-netd from {}: {os}. {NO_CODE} \
                     {INSTALL_NETD}",
                    bin.display()
                ),
                HoldRefusal::Unreachable { socket, os } => format!(
                    "{COULD_NOT_ASK}: could not reach the cerulion-netd daemon at {}: {os}. \
                     {NO_CODE}",
                    socket.display()
                ),
                HoldRefusal::NoSocket { socket, log } => format!(
                    "{COULD_NOT_ASK}: cerulion-netd was started but did not open its control \
                     socket at {}; its startup output is in {}. {NO_CODE}",
                    socket.display(),
                    log.display()
                ),
                HoldRefusal::ConnectionFailed { os } => {
                    let why = os.as_ref().map(|os| format!(": {os}")).unwrap_or_default();
                    format!(
                        "{COULD_NOT_ASK}: the connection to cerulion-netd failed{why}. {NO_CODE}"
                    )
                }
                HoldRefusal::WillNotServe { netd } => {
                    format!("Error: cerulion-netd will not serve this robot: {netd}. {NO_CODE}")
                }
                HoldRefusal::NetdFailed { error } => {
                    format!("Error: cerulion-netd could not serve this robot: {error}. {NO_CODE}")
                }
                HoldRefusal::TooOld { daemon } => format!(
                    "{COULD_NOT_ASK}: the connected cerulion-netd speaks protocol v{daemon} but \
                     the 'hold_serving' verb needs protocol v{SERVING_HOLD_MIN_DAEMON_VERSION} \
                     (this client speaks v{PROTOCOL_VERSION}). Upgrade cerulion-netd to use \
                     this feature. Restart cerulion-netd after the upgrade."
                ),
                HoldRefusal::NetdProtocol => format!(
                    "{COULD_NOT_ASK}: its answer does not follow the protocol this cerulion \
                     speaks. Use cerulion and cerulion-netd from the same release and restart \
                     cerulion-netd. {NO_CODE}"
                ),
            }
        }
    }
}

#[cfg(test)]
mod tests;
