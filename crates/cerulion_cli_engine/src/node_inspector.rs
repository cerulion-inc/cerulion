// SPDX-License-Identifier: AGPL-3.0-only
//! How a validation pass reads a node library's `NodeInfo` — in this process,
//! or in a child that dies alone.
//!
//! `graph validate`'s network-ingress gate needs each consuming node's
//! declared input schema hashes, which only the node's cdylib knows
//! (`cerulion_node_info()`). Loading a library runs its constructors, and a
//! library that ABORTS on load takes the loading process with it: measured on
//! `cerulion-wsd`, a raw-FFI node whose `cerulion_node_info` called
//! `std::process::abort()` killed the standing daemon (SIGABRT), cut every
//! Studio session off mid-request and left its socket + pidfile behind. A
//! one-shot `cerulion graph validate` dying that way is the user's own
//! terminal; a daemon dying is everyone's.
//!
//! [`NodeInspector`] is the seam. [`InProcessInspector`] is what the CLI does
//! and always did; [`SubprocessInspector`] runs a helper program (the daemon's
//! own binary, `cerulion-wsd --inspect-node <lib>`) that loads the library and
//! prints the raw info JSON, and turns a crash, a non-zero exit, unparseable
//! output, a flood or a hang into a `CliError::Validation` the gate renders as
//! a FAILING `network ingress` check — never a dead daemon.

use std::ffi::OsString;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::graph::node::DylibNodeEntry;
use cerulion_core::{NodeEntry as _, NodeInfo};

use crate::error::{CliError, CliResult};

/// Read a node library's [`NodeInfo`]. `label` names the node in diagnostics.
pub trait NodeInspector: Send + Sync {
    fn inspect(&self, cdylib: &Path, label: &str) -> CliResult<NodeInfo>;
}

/// Load the library into THIS process (`DylibNodeEntry::load` + `info()`).
/// Right for a one-shot CLI; wrong for a standing daemon (see the module docs).
#[derive(Debug, Default, Clone, Copy)]
pub struct InProcessInspector;

impl NodeInspector for InProcessInspector {
    fn inspect(&self, cdylib: &Path, _label: &str) -> CliResult<NodeInfo> {
        Ok(DylibNodeEntry::load(cdylib)?.info()?)
    }
}

/// Default deadline for a helper inspection. A library's constructors run at
/// load; one that never returns must not wedge the caller's request.
pub const DEFAULT_INSPECT_TIMEOUT: Duration = Duration::from_secs(30);

/// Most bytes RETAINED from EACH of the helper's pipes. The info document is
/// a few kilobytes; a library that floods stdout or stderr at load must not
/// grow the caller's memory for the length of the deadline (measured: a 32 MiB
/// stream added ~34 MiB to the daemon's RSS). Past the cap the reader keeps
/// draining but discards — closing the pipe would kill the child with SIGPIPE
/// and turn a flood into a "crash" — until EOF or its deadline, and the
/// inspection fails naming the cap.
pub const MAX_INSPECT_OUTPUT_BYTES: usize = 1 << 20;

/// Load the library in a CHILD process and parse the info JSON it prints.
///
/// The child is `program prefix_args... <cdylib>`. Its WHOLE stdout is the
/// document `cerulion_node_info()` returned (any shape `serde_json` accepts,
/// pretty-printed included — the same parser as the in-process path); it
/// exits 0, or explains on stderr and exits non-zero. The helper keeps the
/// document channel clean itself: `cerulion-wsd --inspect-node` redirects
/// fd 1 to fd 2 before loading the library, so load-time chatter, whichever
/// stdio buffer it sits in, reaches stderr and never the document.
///
/// Everything else is reported, never propagated, under TWO bounds:
///
/// * the inspection deadline (`timeout`): the child runs in its OWN process
///   group; when the helper has exited (observed with `waitid(WNOWAIT)`, so
///   the zombie still pins the group id) or the deadline passes, the group is
///   killed as a whole — a descendant the library spawned cannot keep the
///   pipes open past the deadline (reproduced by review: a backgrounded
///   `sleep` held stdout open and the join waited for it) — and only then is
///   the child reaped, with the reap itself bounded;
/// * the collection grace ([`READER_GRACE`]) after the collector stops waiting
///   for the helper — the reap, the bounded attempt at it, or the ECHILD that
///   ended the wait: the collector takes whatever each reader has by then.
///   Each reader retains at most [`MAX_INSPECT_OUTPUT_BYTES`] and polls with
///   the deadline plus that grace as ITS bound, busy pipe or idle, so it
///   always returns and drops its descriptor — an escapee (`setsid`) that
///   keeps a pipe open costs that bound, never a leaked thread.
///
/// The collection grace is a flat bound and the collector cannot see WHY a
/// reader has not reported, so a note about an unfinished pipe states what was
/// OBSERVED and lists the possibilities rather than asserting one (see
/// `PipeEnd::Unfinished`, whose wording also depends on what the collector
/// knew about the helper — see `HelperState`). Failing a healthy library
/// therefore needs a runnable reader thread starved for the whole grace — the
/// pipe's last holder is already dead by then, so the reader only has to be
/// scheduled once to report EOF. That residual is accepted, not closed: the
/// bound stays flat, and the verdict names the unscheduled reader so a starved
/// runner produces a truthful failure rather than a misattributed one.
///
/// The exit status is reported first (a crash names its signal); a truncated
/// or held-open pipe rides along as a note, never in place of the cause. Only
/// the DOCUMENT channel — or a FLOODED stderr, a load-time defect in its own
/// right — can veto a clean exit; stderr that was merely held open or
/// unreadable is logged, not fatal — and its note rides along in the verdict
/// even when the document channel vetoed, because both pipes held open is the
/// corroborating signal for a forked survivor.
#[derive(Debug, Clone)]
pub struct SubprocessInspector {
    program: PathBuf,
    prefix_args: Vec<OsString>,
    timeout: Duration,
}

impl SubprocessInspector {
    pub fn new(program: PathBuf, prefix_args: Vec<OsString>, timeout: Duration) -> Self {
        Self {
            program,
            prefix_args,
            timeout,
        }
    }

    /// The helper program this inspector runs.
    pub fn program(&self) -> &Path {
        &self.program
    }
}

const POLL: Duration = Duration::from_millis(20);
/// How long the COLLECTOR waits, once it has stopped waiting for the helper —
/// at the reap, at the bounded attempt at it, or at the ECHILD that ended the
/// wait (where neither happens) — for the readers to hand over what they have
/// (the pipes close when their last holder dies, so on the ordinary path this
/// only covers scheduling); the readers themselves poll until the inspection
/// deadline plus this grace. A pipe that has not closed by then is reported as
/// the OBSERVATION it is — every arm that reaches the collector renders the
/// notes, including the ones where the helper was never reaped, and none of
/// them asserts a cause the collector cannot see.
pub const READER_GRACE: Duration = Duration::from_millis(500);
/// How long the reap after SIGKILL may take before the verdict stops waiting
/// (a helper stuck in uninterruptible sleep is reported, not waited for). Every
/// arm that reaps goes through [`reap_bounded`], so the bound holds on the
/// observed-exit path, the timeout path AND the wait-error path.
const REAP_BUDGET: Duration = Duration::from_secs(2);

/// Reap the child within [`REAP_BUDGET`]. `Ok(None)` is the budget expiring
/// with the helper still alive — blocked in the kernel, where SIGKILL cannot
/// reach it — never a silent success.
///
/// `Child::wait` is a BLOCKING `waitpid`, so a wedged helper would hold the
/// caller's request for as long as its driver does; every arm shares this one
/// loop rather than one arm keeping an unbounded wait the docs deny.
fn reap_bounded(
    child: &mut std::process::Child,
) -> std::io::Result<Option<std::process::ExitStatus>> {
    let reap_by = Instant::now() + REAP_BUDGET;
    loop {
        match child.try_wait() {
            Ok(Some(status)) => return Ok(Some(status)),
            Ok(None) if Instant::now() < reap_by => std::thread::sleep(POLL),
            Ok(None) => return Ok(None),
            Err(e) => return Err(e),
        }
    }
}

/// What one pipe reader collected.
struct PipeOutput {
    bytes: Vec<u8>,
    /// How the read ended: cleanly at EOF, or not.
    end: PipeEnd,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum PipeEnd {
    Eof,
    /// The cap was hit; `bytes` is the prefix that was kept and the rest was
    /// drained and discarded until EOF or the reader's deadline.
    Overflowed,
    /// The READER's own deadline (the inspection deadline plus
    /// [`READER_GRACE`]) passed with the pipe still open; `bytes` is what
    /// arrived before.
    HeldOpen,
    /// The COLLECTOR's grace passed with the reader still running; `bytes` is
    /// what had arrived by then. The collector observes only that the reader
    /// has not reported — it cannot tell a pipe still held from a reader not
    /// yet scheduled, nor WHICH of the possible holders is holding it — so the
    /// note lists the possibilities and asserts none.
    Unfinished,
    /// `read(2)` failed with this error; `bytes` is what arrived before.
    ReadError(String),
}

/// What the collector knew about the helper when it rendered its notes.
///
/// [`PipeEnd::Unfinished`] means "the reader had not reported"; who might
/// still hold the pipe depends on what became of the helper. Naming an escapee
/// on the path where the helper itself is the wedged holder sends the operator
/// hunting for a process that does not exist — and so does naming the helper
/// on the path where a foreign reaper already took it — so the state is passed
/// in rather than assumed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HelperState {
    /// An exit status was observed HERE and the group kill ran: the helper is
    /// gone, and whatever still holds a pipe outlived that kill.
    Reaped,
    /// The group kill ran and no exit status was observed — the reap was given
    /// up on (the helper is blocked in the kernel) or the reap itself failed
    /// with something other than ECHILD. The helper may well be the holder.
    Unreaped,
    /// Something ELSE in this process reaped the child (ECHILD, from the wait
    /// or from EITHER reap). The helper is gone, so it cannot be the holder —
    /// and the group kill is no help either: when the WAIT reported ECHILD the
    /// kill was SKIPPED (the freed pid is not ours to signal), and when a REAP
    /// reported it the kill had already run — but the foreign reap may have
    /// freed the leader pid before it landed, so nothing can be concluded from
    /// it either way. The state does not carry which of those happened — the
    /// arms know, but one variant serves both — and even the reap-error arms
    /// cannot see whether their kill landed before the pid was freed, so the
    /// note hedges rather than picking one.
    ReapedElsewhere,
}

impl PipeEnd {
    fn note(&self, pipe: &str, helper: HelperState) -> Option<String> {
        let what = if pipe == "stdout" {
            "the info document".to_string()
        } else {
            format!("{pipe} (load-time output)")
        };
        let grace = READER_GRACE.as_millis();
        match self {
            Self::Eof => None,
            Self::Overflowed => Some(format!(
                "{what} exceeded {MAX_INSPECT_OUTPUT_BYTES} bytes; only that prefix was kept"
            )),
            // Both bounds are rendered as the same observation ("had not
            // closed"), differing only in the anchor they are measured from,
            // so the poll-tick race between the two on the timeout path cannot
            // read as two different diagnoses of one condition.
            Self::HeldOpen => Some(format!(
                "{pipe} had not closed {grace} ms past the inspection deadline; its output is what arrived before"
            )),
            // None of the three claims a complete cause list: an in-group
            // member the SIGKILL could not end in time is a third holder, and
            // the collector cannot see any of them — hence "for example".
            Self::Unfinished => Some(match helper {
                HelperState::Reaped => format!(
                    "{pipe} had not closed {grace} ms after the helper was reaped (it may still be held — by a process outside its group, for example, or one the kill could not end in time — or the reader thread was not scheduled); its output is what arrived before"
                ),
                HelperState::Unreaped => format!(
                    "{pipe} had not closed {grace} ms after the helper's group was signalled (the helper itself may still hold it, for example, or the reader thread was not scheduled); its output is what arrived before"
                ),
                HelperState::ReapedElsewhere => format!(
                    "{pipe} had not closed {grace} ms after the helper was reaped by something else in this process (the helper is gone, and the group kill was skipped, or may have been aimed at a leader pid the foreign reap had already freed, so it proves nothing about the group: a descendant may still hold it, for example, or the reader thread was not scheduled); its output is what arrived before"
                ),
            }),
            Self::ReadError(e) => Some(format!("{pipe} could not be drained: {e}")),
        }
    }
}

/// SIGKILL was sent, [`REAP_BUDGET`] expired and the helper is STILL alive:
/// log it where the operator looks for the zombie (`Child`'s drop does not
/// wait, so it is the daemon's until the daemon exits) and return the same
/// facts as a verdict fragment for the requester.
///
/// Both reaping arms call this. They used to diverge: the wait-error arm
/// reaped bounded like everyone else but threw the outcome away, so a helper
/// still alive after the budget got no `error!`, no pid and no "left
/// un-reaped" — the one arm the earlier sweep missed.
fn report_unreaped(pid: u32, label: &str, cdylib: &Path, timeout: Duration) -> String {
    tracing::error!(
        pid,
        node = label,
        cdylib = %cdylib.display(),
        timeout = ?timeout,
        reap_budget = ?REAP_BUDGET,
        "node inspector: helper did not die of SIGKILL within the reap budget — blocked in the kernel, left un-reaped"
    );
    format!(
        "the inspector (pid {pid}) was sent SIGKILL and was still alive {REAP_BUDGET:?} later \
         (stuck in the kernel), so it is left un-reaped and the request gave up on it; the \
         inspection deadline is {timeout:?}"
    )
}

/// What a FAILED reap says about the helper. ECHILD means something else in
/// this process reaped the child between our wait and our reap, so the helper
/// is gone and cannot be the holder of anything; any other errno leaves it
/// un-reaped, where it may well be.
///
/// Both reap-error sites classify HERE. They used to decide separately, and
/// only one of them looked at the errno: the reap inside the wait-error arm
/// mapped every failure to [`HelperState::Unreaped`], so a helper a foreign
/// reaper had taken was still offered to the operator as the possible holder —
/// the one discrimination the earlier sweep missed, and a claim
/// `HelperState`'s own docs make about both arms.
fn helper_state_after_failed_reap(error: &std::io::Error) -> HelperState {
    if error.raw_os_error() == Some(libc_echild()) {
        HelperState::ReapedElsewhere
    } else {
        HelperState::Unreaped
    }
}

/// A reap that failed with an errno OTHER than ECHILD: the helper was neither
/// observed to exit nor reaped, and the errno is the only thing that says why.
/// Log it where the operator looks for the zombie.
///
/// The message is ARM-NEUTRAL and the errno is a FIELD (never spliced into the
/// message — see [`warn_unclean_pipe`]) because BOTH reap-error arms report
/// through it: the reap inside the wait-error arm, which also needs the fact
/// as a verdict fragment ([`report_failed_reap`]), and the main path's, whose
/// verdict already carries the errno for the requester but which used to leave
/// the OPERATOR with nothing — the same helper left un-reaped, logged on one
/// arm and silent on the other. The condition is the same on both, so it is
/// one message and one grep.
fn warn_failed_reap(pid: u32, label: &str, cdylib: &Path, error: &std::io::Error) {
    tracing::warn!(
        pid,
        node = label,
        cdylib = %cdylib.display(),
        error = %error,
        "node inspector: the reap failed"
    );
}

/// [`warn_failed_reap`] plus the same fact as a verdict fragment, for the arm
/// whose own verdict does not already carry the errno.
///
/// The sibling of [`report_unreaped`], which covers the other way that arm can
/// leave a helper un-reaped (the budget expiring).
fn report_failed_reap(pid: u32, label: &str, cdylib: &Path, error: &std::io::Error) -> String {
    warn_failed_reap(pid, label, cdylib, error);
    format!("reaping it failed: {error}")
}

/// The one `warn!` for a pipe that did not end cleanly and is NOT fatal. The
/// note is a FIELD, never part of the message: the repo-wide
/// `tracing_field_discipline_test` ratchet rejects a runtime value
/// interpolated into a message literal, and an operator greps by key.
///
/// Both arms that can observe a non-fatal stderr note call it — the clean exit
/// and the document-channel veto — so "a stderr merely held open is logged" is
/// true wherever it is observed, not only where the document was fine.
fn warn_unclean_pipe(label: &str, cdylib: &Path, note: &str) {
    tracing::warn!(
        node = label,
        cdylib = %cdylib.display(),
        pipe = "stderr",
        note = %note,
        "node inspector: a pipe did not end cleanly"
    );
}

/// The notes for both pipes, ready to append to a verdict (empty when both
/// ended cleanly).
fn notes_text(out: &PipeEnd, err: &PipeEnd, helper: HelperState) -> String {
    let notes: Vec<String> = [out.note("stdout", helper), err.note("stderr", helper)]
        .into_iter()
        .flatten()
        .collect();
    if notes.is_empty() {
        String::new()
    } else {
        format!("; {}", notes.join("; "))
    }
}

impl NodeInspector for SubprocessInspector {
    fn inspect(&self, cdylib: &Path, label: &str) -> CliResult<NodeInfo> {
        let mut command = Command::new(&self.program);
        command
            .args(&self.prefix_args)
            .arg(cdylib)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        #[cfg(unix)]
        {
            use std::os::unix::process::CommandExt as _;
            // Own process group (pgid == pid): killing it takes every descendant
            // that inherited the pipes with it.
            command.process_group(0);
        }
        let mut child = command.spawn().map_err(|e| {
            CliError::Validation(format!(
                "could not start the node inspector `{}` for node '{label}': {e}",
                self.program.display()
            ))
        })?;
        let pid = child.id();
        let deadline = Instant::now() + self.timeout;
        let reader_deadline = deadline + READER_GRACE;
        let stdout = spawn_reader(child.stdout.take(), reader_deadline);
        let stderr = spawn_reader(child.stderr.take(), reader_deadline);

        let exited = loop {
            match child_exited_unreaped(&mut child) {
                Ok(true) => break Ok(true),
                Ok(false) if Instant::now() < deadline => std::thread::sleep(POLL),
                Ok(false) => break Ok(false),
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => break Err(e),
            }
        };
        let exited = match exited {
            Ok(exited) => exited,
            Err(e) => {
                // ECHILD means the child was reaped elsewhere: its pid is no
                // longer ours to signal. Anything else: the leader is still ours.
                //
                // On ECHILD there is nothing left to reap either, and the reap
                // must be skipped for the same reason the kill is: the pid was
                // freed by the foreign reaper and the kernel may have handed it
                // to another child of THIS process, whose status a blocking
                // `child.wait()` would consume.
                let reaped_elsewhere = e.raw_os_error() == Some(libc_echild());
                let mut reap_clause = String::new();
                let helper = if reaped_elsewhere {
                    HelperState::ReapedElsewhere
                } else {
                    kill_group(pid, false);
                    // Bounded, like every other arm: `Child::wait` is a
                    // BLOCKING waitpid and the helper this arm exists for may
                    // be wedged in the kernel, which would hold the caller's
                    // request open for as long as the driver does. The outcome
                    // is REPORTED, not discarded — a helper still alive after
                    // the budget is a zombie the daemon now owns.
                    match reap_bounded(&mut child) {
                        Ok(Some(_)) => HelperState::Reaped,
                        Ok(None) => {
                            reap_clause =
                                format!("; {}", report_unreaped(pid, label, cdylib, self.timeout));
                            HelperState::Unreaped
                        }
                        // Classified by the SAME function as the sibling
                        // reap-error arm below, so the two cannot disagree
                        // about one errno again: this arm used to map EVERY
                        // reap failure to `Unreaped` — offering the helper as
                        // the possible holder even on ECHILD, where a foreign
                        // reaper has already taken it — and to log nothing.
                        Err(e) => {
                            let helper = helper_state_after_failed_reap(&e);
                            // A foreign reap is not a failure to report: the
                            // notes already say the helper is gone. Any other
                            // errno left it un-reaped, and is the only thing
                            // that says why.
                            if helper == HelperState::Unreaped {
                                reap_clause =
                                    format!("; {}", report_failed_reap(pid, label, cdylib, &e));
                            }
                            helper
                        }
                    }
                };
                let by = Instant::now() + READER_GRACE;
                let (out, err) = (collect_reader(stdout, by), collect_reader(stderr, by));
                // The notes say only what THIS arm knows about the helper
                // (READER_GRACE's contract: every arm that reaches the
                // collector renders its notes, and none asserts a cause it
                // cannot see).
                let notes = notes_text(&out.end, &err.end, helper);
                return Err(CliError::Validation(format!(
                    "waiting for the node inspector of '{label}' failed: {e}{}{}{}{}",
                    if reaped_elsewhere {
                        " (the child was reaped by something else in this process, so its process group was not signalled)"
                    } else {
                        ""
                    },
                    reap_clause,
                    notes,
                    stderr_clause(&bounded_excerpt(&err.bytes))
                )));
            }
        };
        // The helper is done (or overdue) and NOT yet reaped, so its pid still
        // pins the group id: kill the whole group now, then reap — bounded.
        kill_group(pid, exited);
        let status = match reap_bounded(&mut child) {
            Ok(status) => status,
            Err(e) => {
                // The sibling of the waitid-error arm above: the child's own
                // explanation is on a pipe we are about to drop, and it is
                // usually the only thing that names the cause.
                let by = Instant::now() + READER_GRACE;
                let (out, err) = (collect_reader(stdout, by), collect_reader(stderr, by));
                let helper = helper_state_after_failed_reap(&e);
                if helper == HelperState::Unreaped {
                    // The verdict below carries the errno for the REQUESTER;
                    // this is the OPERATOR's copy, under the same constant
                    // message the sibling reap-error arm logs. The helper is
                    // left un-reaped here exactly as it is there, and a
                    // condition that is the same on both arms must not be
                    // greppable on only one of them.
                    warn_failed_reap(pid, label, cdylib, &e);
                }
                let reaped_elsewhere = helper == HelperState::ReapedElsewhere;
                // The kill DID run here — but on ECHILD the foreign reap may
                // have freed the leader pid before it landed, and whether it
                // did is not observable, so the kill proves nothing about the
                // group either way. The helper itself is gone regardless.
                let notes = notes_text(&out.end, &err.end, helper);
                return Err(CliError::Validation(format!(
                    "reaping the node inspector of '{label}' failed: {e}{}{}{}",
                    if reaped_elsewhere {
                        " (the child was reaped by something else in this process)"
                    } else {
                        ""
                    },
                    notes,
                    stderr_clause(&bounded_excerpt(&err.bytes))
                )));
            }
        };
        // The grace is anchored where the collector stopped waiting for the
        // helper — here, the reap or the bounded attempt at it (see
        // [`READER_GRACE`]).
        let collect_by = Instant::now() + READER_GRACE;
        let out = collect_reader(stdout, collect_by);
        let err = collect_reader(stderr, collect_by);
        let err_text = bounded_excerpt(&err.bytes);
        let helper = if status.is_some() {
            HelperState::Reaped
        } else {
            HelperState::Unreaped
        };
        let notes_text = notes_text(&out.end, &err.end, helper);
        let Some(status) = status else {
            // The helper is the wedged holder of its own pipes here, and it is
            // never reaped. Both reaping arms report that through one helper so
            // they cannot diverge again.
            let unreaped = report_unreaped(pid, label, cdylib, self.timeout);
            return Err(CliError::Validation(format!(
                "node library `{}` for node '{label}': {unreaped}{}{}",
                cdylib.display(),
                notes_text,
                stderr_clause(&err_text)
            )));
        };
        if !exited {
            return Err(CliError::Validation(format!(
                "node library `{}` for node '{label}' did not finish inspection within {:?} — \
                 the inspector's process group was killed; a library whose constructors hang \
                 cannot be validated (or run){}{}",
                cdylib.display(),
                self.timeout,
                notes_text,
                stderr_clause(&err_text)
            )));
        }
        if !status.success() {
            // A failing child that said NOTHING on stderr has told us only a
            // number, and the document channel is then the only thing it said.
            // The general shape is any helper whose only words were on stdout.
            // One instance of it is `cerulion-wsd --inspect-node` reporting a
            // refusal it cannot route to fd 2 on fd 1, the one descriptor it
            // knows is open — reachable only if fd 2 is LOST MID-PROCESS,
            // since a spawn that closes fd 2 has it re-opened by the RUST
            // RUNTIME before `main` (documented in `cerulion_wsd`'s `main.rs`;
            // pinned by `cerulion_wsd/tests/inspect_channel_test.rs::a_closed_stderr_at_exec_never_reaches_the_refusal_because_the_runtime_reopens_it`).
            // Quote it, labelled as stdout so nobody reads it as a document,
            // and only when stderr had nothing: on every other failure stdout
            // is a truncated document and quoting it is noise.
            let out_text = if err_text.is_empty() {
                bounded_excerpt(&out.bytes)
            } else {
                String::new()
            };
            #[cfg(unix)]
            {
                use std::os::unix::process::ExitStatusExt as _;
                if let Some(signal) = status.signal() {
                    return Err(CliError::Validation(format!(
                        "node library `{}` for node '{label}' CRASHED while being inspected \
                         (signal {signal}{}) — the inspection ran in a child process, so the \
                         caller survived; fix the library's `cerulion_node_info()` / load-time \
                         constructors{}{}{}",
                        cdylib.display(),
                        signal_name(signal),
                        notes_text,
                        stderr_clause(&err_text),
                        stdout_clause(&out_text)
                    )));
                }
            }
            return Err(CliError::Validation(format!(
                "node inspector for '{label}' exited with {status}{}{}{}",
                notes_text,
                stderr_clause(&err_text),
                stdout_clause(&out_text)
            )));
        }
        // Only the DOCUMENT channel can veto a clean exit: a document that did
        // not end at EOF is a prefix and cannot be trusted; a flooded stderr is
        // a load-time defect in its own right. A stderr merely held open or
        // unreadable says nothing about the document, so it is logged below.
        if let Some(note) = out.end.note("stdout", helper) {
            // A stderr note is CORROBORATING evidence here — both pipes held
            // open is the signature of a `fork()` without `exec` — so it rides
            // along in the verdict and is logged, exactly as on the clean-exit
            // path below. Rendering only the stdout note left the fact that
            // stderr was held too in neither place.
            let err_note = err.end.note("stderr", helper);
            if let Some(err_note) = err_note.as_deref() {
                warn_unclean_pipe(label, cdylib, err_note);
            }
            return Err(CliError::Validation(format!(
                "node library `{}` for node '{label}' could not be inspected: {note}{}{}",
                cdylib.display(),
                err_note.map(|n| format!("; {n}")).unwrap_or_default(),
                stderr_clause(&err_text)
            )));
        }
        if err.end == PipeEnd::Overflowed {
            return Err(CliError::Validation(format!(
                "node library `{}` for node '{label}' could not be inspected: {}{}",
                cdylib.display(),
                err.end.note("stderr", helper).unwrap_or_default(),
                stderr_clause(&err_text)
            )));
        }
        if let Some(note) = err.end.note("stderr", helper) {
            warn_unclean_pipe(label, cdylib, &note);
        }
        if !err_text.is_empty() {
            // The CLI path shows these (a `DylibNodeEntry::load` warn, a
            // library's own diagnostics); the daemon path must not swallow them.
            tracing::warn!(
                node = label,
                cdylib = %cdylib.display(),
                stderr = %err_text,
                "node inspector: the helper wrote to stderr during a successful inspection"
            );
        }
        let json = String::from_utf8_lossy(&out.bytes);
        DylibNodeEntry::parse_info_json_labeled(json.trim(), label).map_err(|reason| {
            CliError::Validation(format!(
                "node library `{}` for node '{label}' returned unusable info JSON: {reason}{}",
                cdylib.display(),
                stderr_clause(&err_text)
            ))
        })
    }
}

fn stderr_clause(err_text: &str) -> String {
    if err_text.is_empty() {
        String::new()
    } else {
        format!(" (stderr: {err_text})")
    }
}

/// Quote the DOCUMENT channel, labelled, on a failing exit that left stderr
/// empty. Labelled because it is not a document then — it is whatever the
/// child managed to say on the only descriptor it had.
fn stdout_clause(out_text: &str) -> String {
    if out_text.is_empty() {
        String::new()
    } else {
        format!(" (stdout: {out_text})")
    }
}

/// Most bytes of a child pipe quoted in a verdict.
const EXCERPT_BYTES: usize = 1024;

/// The tail of one of the child's pipes, bounded and printable: a verdict
/// travels in a JSON error field and a log line, so a flood or binary noise
/// must not ride along verbatim. Shared by both channels — stderr always, and
/// stdout on a failing exit that said nothing on stderr.
fn bounded_excerpt(bytes: &[u8]) -> String {
    let tail = &bytes[bytes.len().saturating_sub(EXCERPT_BYTES)..];
    let text: String = String::from_utf8_lossy(tail)
        .chars()
        .map(|c| {
            if c.is_control() && c != '\n' && c != '\t' {
                '?'
            } else {
                c
            }
        })
        .collect();
    let text = text.trim().to_string();
    if bytes.len() > EXCERPT_BYTES && !text.is_empty() {
        format!("… {text}")
    } else {
        text
    }
}

#[cfg(unix)]
fn libc_echild() -> i32 {
    libc::ECHILD
}

#[cfg(not(unix))]
fn libc_echild() -> i32 {
    -1
}

/// Has the child exited, WITHOUT reaping it? A reaped pid can be recycled by
/// the kernel at once, and the process-group kill that follows must target
/// the group this child led — which the unreaped zombie keeps pinned.
#[cfg(unix)]
fn child_exited_unreaped(child: &mut std::process::Child) -> std::io::Result<bool> {
    // SAFETY: a zeroed siginfo_t is a valid out-parameter for waitid(2); WNOWAIT
    // leaves the child unreaped, WNOHANG makes the call non-blocking.
    let mut info: libc::siginfo_t = unsafe { std::mem::zeroed() };
    let rc = unsafe {
        libc::waitid(
            libc::P_PID,
            child.id() as libc::id_t,
            &mut info,
            libc::WEXITED | libc::WNOHANG | libc::WNOWAIT,
        )
    };
    if rc != 0 {
        return Err(std::io::Error::last_os_error());
    }
    // With WNOHANG, a still-running child leaves si_pid == 0.
    // SAFETY: si_pid is a plain field of the union-backed struct for a
    // successful WEXITED query.
    Ok(unsafe { info.si_pid() } != 0)
}

#[cfg(not(unix))]
fn child_exited_unreaped(child: &mut std::process::Child) -> std::io::Result<bool> {
    Ok(child.try_wait()?.is_some())
}

#[cfg(all(test, unix))]
thread_local! {
    /// Benign [`kill_group`] refusals (ESRCH, or EPERM after an observed exit)
    /// on THIS thread — `inspect` runs the kill on its caller's thread, so a
    /// test reads exactly its own.
    ///
    /// The healthy-inspection pin needs level-INDEPENDENT evidence that the
    /// benign arm really ran on macOS, or its "no refusal was warned" half is
    /// vacuous there. It cannot read the `debug!` below: `cerulion_core`
    /// enables `tracing/release_max_level_info` and feature unification carries
    /// that into this crate, so a `--release` test run compiles the line away
    /// and the pin would fail on correct code.
    static BENIGN_KILL_REFUSALS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// Kill the helper's whole process group (Unix; the helper was spawned as its
/// leader). After an OBSERVED exit the unreaped leader keeps the group alive
/// and the kill returns 0 — or, on macOS, EPERM when the zombie is its only
/// member; both are the clean outcome, as is ESRCH (the group is already
/// gone). On the timeout path any refusal other than ESRCH is logged, because
/// something in the group may still hold a pipe.
fn kill_group(pid: u32, exited: bool) {
    #[cfg(unix)]
    {
        // SAFETY: kill(2) on a negative pid addresses the process group this
        // child leads; the leader is unreaped (or overdue), so the id is ours.
        let rc = unsafe { libc::kill(-(pid as libc::pid_t), libc::SIGKILL) };
        if rc != 0 {
            let err = std::io::Error::last_os_error();
            let benign = match err.raw_os_error() {
                Some(libc::ESRCH) => true,
                Some(libc::EPERM) => exited,
                _ => false,
            };
            if benign {
                #[cfg(all(test, unix))]
                BENIGN_KILL_REFUSALS.with(|seen| seen.set(seen.get() + 1));
                tracing::debug!(pid, error = %err, "node inspector: process-group kill after exit");
            } else {
                tracing::warn!(pid, error = %err, "node inspector: process-group kill refused");
            }
        }
    }
    #[cfg(not(unix))]
    {
        let _ = (pid, exited);
    }
}

/// Read a pipe on a thread, retaining at most [`MAX_INSPECT_OUTPUT_BYTES`]
/// and bounded by `deadline` on a busy pipe as well as an idle one (each read
/// is preceded by `poll(2)` with the remaining time, and the deadline is
/// re-checked before every read). Past the cap the reader keeps draining and
/// discards. Progress is shared, so the collector can take what arrived even
/// if the reader is still running.
fn spawn_reader<R>(pipe: Option<R>, deadline: Instant) -> Reader
where
    R: Read + Send + PipeFd + 'static,
{
    let shared = Arc::new(Mutex::new(PipeOutput {
        bytes: Vec::new(),
        end: PipeEnd::Eof,
    }));
    let (done_tx, done_rx) = std::sync::mpsc::channel();
    let progress = Arc::clone(&shared);
    std::thread::spawn(move || {
        let end = match pipe {
            Some(mut pipe) => drain(&mut pipe, deadline, &progress),
            None => PipeEnd::Eof,
        };
        progress.lock().unwrap_or_else(|e| e.into_inner()).end = end;
        let _ = done_tx.send(());
    });
    Reader { shared, done_rx }
}

struct Reader {
    shared: Arc<Mutex<PipeOutput>>,
    done_rx: std::sync::mpsc::Receiver<()>,
}

fn drain<R: Read + PipeFd>(
    pipe: &mut R,
    deadline: Instant,
    progress: &Mutex<PipeOutput>,
) -> PipeEnd {
    let mut chunk = [0u8; 8192];
    let mut overflowed = false;
    loop {
        if Instant::now() >= deadline || !pipe.readable_before(deadline) {
            return if overflowed {
                PipeEnd::Overflowed
            } else {
                PipeEnd::HeldOpen
            };
        }
        match pipe.read(&mut chunk) {
            Ok(0) => {
                return if overflowed {
                    PipeEnd::Overflowed
                } else {
                    PipeEnd::Eof
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(e) => return PipeEnd::ReadError(e.to_string()),
            // Past the cap: keep the pipe flowing (the child must not die of
            // SIGPIPE on our account) but retain nothing more.
            Ok(_) if overflowed => {}
            Ok(n) => {
                let mut out = progress.lock().unwrap_or_else(|e| e.into_inner());
                let room = MAX_INSPECT_OUTPUT_BYTES.saturating_sub(out.bytes.len());
                if n > room {
                    out.bytes.extend_from_slice(&chunk[..room]);
                    overflowed = true;
                } else {
                    out.bytes.extend_from_slice(&chunk[..n]);
                }
            }
        }
    }
}

/// Wait for a pipe's data to be readable, or for `deadline`.
trait PipeFd {
    fn readable_before(&self, deadline: Instant) -> bool;
}

#[cfg(unix)]
impl<T: std::os::fd::AsRawFd> PipeFd for T {
    fn readable_before(&self, deadline: Instant) -> bool {
        let mut pfd = libc::pollfd {
            fd: self.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return false;
            }
            let ms = libc::c_int::try_from(remaining.as_millis()).unwrap_or(libc::c_int::MAX);
            // SAFETY: one valid pollfd for a descriptor this thread owns.
            let rc = unsafe { libc::poll(&mut pfd, 1, ms) };
            if rc > 0 {
                return true; // readable, or hung up (read returns 0)
            }
            if rc == 0 {
                return false;
            }
            let err = std::io::Error::last_os_error();
            if err.kind() != std::io::ErrorKind::Interrupted {
                return true; // let read(2) report the error
            }
        }
    }
}

#[cfg(not(unix))]
impl<T> PipeFd for T {
    fn readable_before(&self, _deadline: Instant) -> bool {
        true
    }
}

/// Take a reader's result by `by`. If the reader has not reported by then —
/// the pipe may still be held, or the thread may not yet have been scheduled;
/// the collector cannot tell — the bytes read so far are returned with
/// `end == Unfinished`, and it is `PipeEnd::note` that renders that
/// observation without asserting any of the possibilities.
fn collect_reader(reader: Reader, by: Instant) -> PipeOutput {
    let wait = by.saturating_duration_since(Instant::now());
    let finished = reader.done_rx.recv_timeout(wait).is_ok();
    let mut out = reader.shared.lock().unwrap_or_else(|e| e.into_inner());
    let bytes = std::mem::take(&mut out.bytes);
    let end = if finished {
        out.end.clone()
    } else {
        PipeEnd::Unfinished
    };
    PipeOutput { bytes, end }
}

#[cfg(unix)]
fn signal_name(signal: i32) -> &'static str {
    match signal {
        libc::SIGABRT => ", SIGABRT",
        libc::SIGSEGV => ", SIGSEGV",
        libc::SIGBUS => ", SIGBUS",
        libc::SIGILL => ", SIGILL",
        libc::SIGFPE => ", SIGFPE",
        libc::SIGKILL => ", SIGKILL",
        _ => "",
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use std::io::Write as _;

    /// `sh -c '<script>' sh <cdylib>` — the script sees the library path as `$1`.
    fn shell(script: &str, timeout: Duration) -> SubprocessInspector {
        SubprocessInspector::new(
            PathBuf::from("/bin/sh"),
            vec![
                OsString::from("-c"),
                OsString::from(script),
                OsString::from("sh"),
            ],
            timeout,
        )
    }
    const LIB: &str = "/nowhere/libcamera.so";

    const LEVELS: [&str; 5] = ["TRACE", "DEBUG", "INFO", "WARN", "ERROR"];

    /// The LEVEL token of one captured line, or `None` if its header carries
    /// none — the whole-token form (the discipline), not a substring
    /// test: `tracing-test` renders the test function's own name as the SPAN,
    /// so `line.contains("WARN")` would count a line whose span name, target or
    /// field VALUE happened to carry the token.
    fn line_level(line: &str) -> Option<&'static str> {
        let header = line.split(": ").next().unwrap_or(line);
        header
            .split_whitespace()
            .find_map(|token| LEVELS.into_iter().find(|level| *level == token))
    }

    /// Count captured lines carrying BOTH the level token and the marker. A
    /// message-only predicate would not catch `warn!` demoted to `debug!`,
    /// which is exactly the regression these pins exist to catch: `debug!` is
    /// compiled out of the daemon by `release_max_level_info`.
    fn count_at(lines: &[&str], level: &str, marker: &str) -> usize {
        lines
            .iter()
            .filter(|l| line_level(l) == Some(level) && l.contains(marker))
            .count()
    }

    /// A scratch file a helper hands something back through, plus the NONCE it
    /// must write beside the payload.
    struct Sentinel {
        path: PathBuf,
        nonce: String,
    }

    /// A per-invocation sentinel. The suffix carries the test binary's pid, a
    /// process-local counter AND the wall nanos, because the first two both
    /// restart at their old values on a rerun: a binary killed mid-arm (a CI
    /// job cancel) leaves `/tmp/cer-inspect-<tag>-<pid>-0` behind, and a later
    /// run handed the same pid would find the loop's readiness test already
    /// satisfied before its own escapee had called `setsid` — inverting the
    /// arm, and then handing [`reap_escapee`] a pid the kernel has long since
    /// given to something else.
    fn scratch_sentinel(tag: &str) -> Sentinel {
        static NEXT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
        let nonce = scratch_nonce(NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed));
        Sentinel {
            path: std::env::temp_dir().join(format!("cer-inspect-{tag}-{nonce}")),
            nonce,
        }
    }

    /// The nonce for invocation `n` of this process. `n` is taken as an
    /// argument so the ACROSS-RUN half can be pinned: a rerun repeats both the
    /// pid and the counter, so the wall stamp is the only thing that keeps two
    /// runs' sentinels apart, and holding `n` fixed is the only way to see it.
    fn scratch_nonce(n: u32) -> String {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0);
        format!("{}-{n}-{nanos}", std::process::id())
    }

    /// How long the escapee sleeps. It is out of the group by construction, so
    /// nothing but its own timer can end it, and the test binary must not
    /// leave a process behind for long — [`reap_escapee`] kills it by pid
    /// anyway, which is why the sentinel carries one, so the sleep is only the
    /// fallback for a reap that never runs. Orders past [`READER_GRACE`],
    /// which is what the arms actually measure.
    const ESCAPEE_SLEEP_SECS: u64 = 20;
    /// The wall the escapee arms assert. DECOUPLED from the sleep: the un-fixed
    /// cost is the escapee's own life, so a bound equal to the sleep would be
    /// separated from an unbounded collect by nothing but interpreter startup.
    /// Its value clears the worst healthy elapsed several times over and still
    /// leaves most of [`ESCAPEE_SLEEP_SECS`] as the unbounded collect's margin; that
    /// measurement is stated ONCE, on [`escapee_wall_message`], and the const
    /// assert below is what holds the ratio to the sleep.
    const ESCAPEE_WALL: Duration = Duration::from_secs(8);
    const _: () = assert!(
        ESCAPEE_WALL.as_secs() * 2 <= ESCAPEE_SLEEP_SECS,
        "the escapee arms' wall must stay at most half its sleep, or a collect that waits for \
         the escapee is separated from a healthy one by interpreter startup alone"
    );

    /// The inspection deadline both escapee arms pass to [`shell`]. NAMED
    /// because it is the OTHER bound on an un-fixed collect: the reader stops
    /// at this plus [`READER_GRACE`] whatever the escapee does, so the
    /// unbounded collect's cost is the SMALLER of that and [`ESCAPEE_SLEEP_SECS`] and
    /// the wall has to stay well under both. A literal at the call sites hid a
    /// third number from the two const asserts below and from the remedy
    /// [`escapee_wall_message`] prints.
    const ESCAPEE_INSPECT_TIMEOUT: Duration = Duration::from_secs(20);
    const _: () = assert!(
        ESCAPEE_WALL.as_millis() * 2 <= ESCAPEE_INSPECT_TIMEOUT.as_millis(),
        "the escapee arms' wall must stay at most half the reader's bound too, or a collect \
         that runs to the inspection deadline is separated from a healthy one by interpreter \
         startup alone"
    );
    const _: () = assert!(
        ESCAPEE_INSPECT_TIMEOUT.as_millis() + READER_GRACE.as_millis()
            >= ESCAPEE_SLEEP_SECS as u128 * 1_000,
        "an unbounded collect must cost about the escapee's whole sleep: if the reader's bound \
         (this timeout plus the grace) dropped below it, the arms would be measuring the \
         reader's deadline instead and the sleep would bound nothing"
    );

    /// Report an over-the-wall elapsed, ATTRIBUTABLY.
    ///
    /// The healthy path is two python startups plus the sentinel poll plus the
    /// grace, measured 0.81-2.44 s on a desktop with a sibling build running —
    /// so [`ESCAPEE_WALL`] is ~3.3x the worst of that; a cold or badly loaded
    /// runner could still cross it without anything being wrong. But the
    /// two causes are DISTINGUISHABLE: an unbounded collect returns only when
    /// the escapee's own timer closes the pipe or the reader's bound
    /// ([`ESCAPEE_INSPECT_TIMEOUT`] plus [`READER_GRACE`]) passes, and the
    /// const assert keeps that bound at or above [`ESCAPEE_SLEEP_SECS`], so it
    /// cannot come in under the sleep. Say which one happened instead of
    /// blaming the fix for the runner (the `flat_latency_test` /
    /// `cross_thread_rtt_test` pattern). Either way the arm still fails: the
    /// wall is a coarse SECOND guard, and what actually kills an
    /// unbounded collect is the VERDICT — such a collect either
    /// succeeds (the escapee's pipe closed before the reader's deadline) or
    /// fails on the HeldOpen note, never on the escapee note the arms assert.
    fn escapee_wall_message(elapsed: Duration) -> String {
        if elapsed >= Duration::from_secs(ESCAPEE_SLEEP_SECS) {
            format!(
                "the inspection WAITED for the escapee: {elapsed:?} is past its whole \
                 {ESCAPEE_SLEEP_SECS} s sleep, which only an unbounded collect can reach"
            )
        } else {
            format!(
                "over the {ESCAPEE_WALL:?} wall at {elapsed:?}, but UNDER the escapee's \
                 {ESCAPEE_SLEEP_SECS} s sleep — that is a slow box (two python startups plus \
                 the sentinel poll), not a collect that waited. The verdict assertions above \
                 already passed, so if this is reproducible, raise ESCAPEE_SLEEP_SECS, \
                 ESCAPEE_INSPECT_TIMEOUT and ESCAPEE_WALL together (the const asserts hold \
                 the ratios)"
            )
        }
    }

    /// A helper script whose backgrounded `python3` escapee calls `os.setsid()`,
    /// writes its own pid to `sentinel`, and only THEN lets the helper print
    /// `document` and exit.
    ///
    /// The escape must be OBSERVED, never raced. The earlier form exited ~3 ms
    /// after forking python while the inspector's next 20 ms poll fired the
    /// group kill, so whether the escapee was still in the group when SIGKILL
    /// landed came down to interpreter startup: measured, the escapee was
    /// killed BEFORE `setsid` on every run under a slow `python3` and survived
    /// on every run under a fast one. That makes the document arm red on a
    /// healthy box and the stderr arm green whether or not the behaviour it
    /// pins is present. Exit 7 — its own loud verdict — if the escape never
    /// happened, so a broken helper can never be read as a passing property.
    ///
    /// The probe is the CAPABILITY, not the binary. `command -v python3` is
    /// satisfied by a macOS Command-Line-Tools stub that cannot run, and by a
    /// sandbox where `setsid` raises — in both cases the escapee never holds
    /// the pipe, the inspection SUCCEEDS, and the document arm dies on
    /// `expect_err` with a panic that says nothing about python. Running the
    /// two calls the arm depends on turns that into the `exit 9` verdict.
    ///
    /// `redirect` is applied to the escapee: empty keeps BOTH inherited pipes,
    /// `>/dev/null` leaves it holding only stderr.
    ///
    /// The sentinel is REMOVED first and carries a per-invocation nonce beside
    /// the pid, so the loop can only ever observe THIS run's escapee and
    /// [`reap_escapee`] can only ever signal it (see [`scratch_sentinel`] for
    /// the stale-file shape both close).
    ///
    /// The escapee sleeps [`ESCAPEE_SLEEP_SECS`] — see that constant.
    ///
    /// The wait is capped at 200 iterations so `exit 7` can still WIN against
    /// the inspection deadline. Each iteration forks `sleep`, measured at
    /// ~33 ms rather than the nominal 10 ms on this platform, so 300 of them
    /// ran ~10 s — and against the 10 s deadline the arms first used, a helper
    /// whose escape never happened timed out instead of reporting `exit 7`,
    /// which is exactly how a broken helper gets read as an unrelated failure.
    /// 200 iterations (~6.6 s worst case here, ~2 s on Linux) against the
    /// arms' [`ESCAPEE_INSPECT_TIMEOUT`] leaves the specific verdict in front.
    /// The happy path never reaches the cap: python writes the sentinel in one
    /// to a few dozen iterations.
    ///
    /// (Caught by mutating this helper to make the sentinel unwritable and
    /// watching the arms report the DEADLINE rather than `exit 7`.)
    fn escapee_script(sentinel: &Sentinel, redirect: &str, document: &str) -> String {
        let s = sentinel.path.display();
        let nonce = &sentinel.nonce;
        let sleep = ESCAPEE_SLEEP_SECS;
        format!(
            "rm -f '{s}'; \
             python3 -c 'import os; os.setsid()' >/dev/null 2>&1 || exit 9; \
             python3 -c \"import os,time; os.setsid(); f=open('{s}','w'); \
             f.write('{nonce} '+str(os.getpid())); f.flush(); f.close(); \
             time.sleep({sleep})\" {redirect} & \
             i=0; while [ ! -s '{s}' ] && [ $i -lt 200 ]; do sleep 0.01; i=$((i+1)); done; \
             [ -s '{s}' ] || exit 7; \
             printf '%s' '{document}'; exit 0"
        )
    }

    /// Kill the escapee (it sleeps [`ESCAPEE_SLEEP_SECS`]) and remove the
    /// sentinel it reported its pid in. The helper cannot do this itself: it
    /// has to exit while the escapee still holds the pipe, which is the whole
    /// point of the arm.
    ///
    /// The kill is gated on the NONCE. A pid alone is not enough to signal on:
    /// a sentinel that outlived a killed test binary carries a pid the kernel
    /// is free to have handed to an unrelated process owned by the same user.
    fn reap_escapee(sentinel: &Sentinel) {
        if let Ok(text) = std::fs::read_to_string(&sentinel.path) {
            if let Some(pid) = escapee_pid_to_kill(&text, &sentinel.nonce) {
                // SAFETY: kill(2) on a pid THIS invocation's helper reported —
                // the nonce says the file is ours and was not left behind.
                unsafe { libc::kill(pid, libc::SIGKILL) };
            }
        }
        let _ = std::fs::remove_file(&sentinel.path);
    }

    /// The decision half of [`reap_escapee`], split out so it can be pinned
    /// without signalling anything: a sentinel yields a pid to SIGKILL only
    /// when its nonce is the one this invocation generated.
    fn escapee_pid_to_kill(text: &str, nonce: &str) -> Option<libc::pid_t> {
        let mut fields = text.split_whitespace();
        if fields.next()? != nonce {
            return None;
        }
        fields.next()?.parse::<libc::pid_t>().ok()
    }

    /// The fixture's two stale-file defences, pinned. Neither is observable
    /// from the arms themselves — they only bite on a rerun that inherits a
    /// leftover `/tmp` file — so without this they would ship unverified.
    ///
    /// The shape both close: a test binary SIGKILLed mid-arm (a CI job cancel)
    /// leaves the sentinel behind. Before, the path was pid + a counter that
    /// restarts at 0, so a rerun handed the same pid re-minted the same name;
    /// the helper's readiness test `[ -s <path> ]` was then satisfied on its
    /// FIRST iteration, before its own escapee had called `setsid`, and the
    /// escapee died with the group — inverting the arm and reporting it as a
    /// failure of the fix. `reap_escapee` then SIGKILLed the stale pid, which
    /// the kernel is free to have given to an unrelated process.
    #[test]
    fn the_escapee_fixture_cannot_be_fooled_by_a_sentinel_from_an_earlier_run() {
        let sentinel = scratch_sentinel("fixture-pin");
        let script = escapee_script(&sentinel, "", "{}");
        let path = sentinel.path.display().to_string();

        // (a) The script CLEARS the sentinel before anything reads it, and the
        //     clear precedes the wait loop that would otherwise see a leftover.
        assert!(
            script.starts_with(&format!("rm -f '{path}'; ")),
            "the sentinel must be removed before the loop can observe it: {script}"
        );
        assert!(
            script.find("rm -f").unwrap() < script.find("while [ ! -s").unwrap(),
            "the clear must precede the readiness loop: {script}"
        );
        // …and the escapee writes the nonce BESIDE its pid, or (b) below has
        // nothing to match on.
        assert!(
            script.contains(&format!("f.write('{} '+str(os.getpid()))", sentinel.nonce)),
            "the escapee must report the nonce with its pid: {script}"
        );
        // (c) The name is unique ACROSS RUNS, not merely within one. Holding
        //     the counter fixed is the only way to see that: a rerun repeats
        //     the pid and restarts the counter at 0, so a nonce built from
        //     those two alone re-mints the very path a killed run left behind.
        //     Bounded in seconds, and load can only DELAY a clock tick.
        let first = scratch_nonce(0);
        let until = Instant::now() + Duration::from_secs(5);
        let mut differs = false;
        while Instant::now() < until {
            if scratch_nonce(0) != first {
                differs = true;
                break;
            }
        }
        assert!(
            differs,
            "two runs with the same pid and the same counter still minted {first:?} — a \
             sentinel left behind by a killed run would be read as this run's own"
        );
        let again = scratch_sentinel("fixture-pin");
        assert_ne!(
            sentinel.path, again.path,
            "…and two sentinels within one run must not collide either"
        );
        assert_ne!(sentinel.nonce, again.nonce);

        // (b) The kill is gated on the nonce: hand oracles, no signal sent.
        let pid: libc::pid_t = 41234;
        assert_eq!(
            escapee_pid_to_kill(&format!("{} {pid}", sentinel.nonce), &sentinel.nonce),
            Some(pid),
            "this invocation's own escapee must still be reaped"
        );
        assert_eq!(
            escapee_pid_to_kill(&format!("{} {pid}", again.nonce), &sentinel.nonce),
            None,
            "a sentinel left by an EARLIER run names a pid the kernel may have recycled — it \
             must never be signalled"
        );
        assert_eq!(
            escapee_pid_to_kill(&format!("{pid}"), &sentinel.nonce),
            None
        );
        assert_eq!(escapee_pid_to_kill("", &sentinel.nonce), None);
        assert_eq!(
            escapee_pid_to_kill(&format!("{} notapid", sentinel.nonce), &sentinel.nonce),
            None
        );
    }

    #[test]
    fn a_crashing_inspection_is_a_validation_error_naming_the_signal_not_a_dead_caller() {
        let err = shell("kill -ABRT $$", Duration::from_secs(5))
            .inspect(Path::new(LIB), "camera")
            .expect_err("a crash must surface as an error");
        let msg = err.to_string();
        assert!(msg.contains("CRASHED") && msg.contains("SIGABRT"), "{msg}");
        assert!(
            msg.contains("libcamera.so") && msg.contains("'camera'"),
            "{msg}"
        );
    }

    #[test]
    fn a_failing_inspection_carries_the_childs_stderr() {
        let err = shell("echo no such symbol >&2; exit 3", Duration::from_secs(5))
            .inspect(Path::new(LIB), "camera")
            .expect_err("non-zero exit is an error");
        let msg = err.to_string();
        assert!(
            msg.contains("exit status: 3") && msg.contains("no such symbol"),
            "{msg}"
        );
    }

    /// A failing child whose only words were on STDOUT must still be quoted.
    /// Rendering only `exited with {status}` plus an empty stderr clause hands
    /// the operator a bare number for a failure whose explanation exists and is
    /// already in hand.
    ///
    /// The fixture is spelled as `cerulion-wsd --inspect-node`'s fd-2 refusal
    /// because that is the shape the wording was written for — but it is a
    /// NARROW instance: the RUST RUNTIME re-opens a closed fd 2 before `main`
    /// runs (`cerulion_wsd`'s `main.rs` documents that; the pin is
    /// `cerulion_wsd/tests/inspect_channel_test.rs::a_closed_stderr_at_exec_never_reaches_the_refusal_because_the_runtime_reopens_it`),
    /// so that refusal is reachable only if the descriptor is lost
    /// MID-PROCESS. The rule this arm actually defends is the general one —
    /// any helper whose only words were on stdout.
    #[test]
    fn a_failing_inspection_that_spoke_only_on_stdout_still_quotes_it() {
        let err = shell(
            "echo 'fd 2 is closed; refusing'; exit 1",
            Duration::from_secs(5),
        )
        .inspect(Path::new(LIB), "camera")
        .expect_err("non-zero exit is an error");
        let msg = err.to_string();
        assert!(msg.contains("exit status: 1"), "{msg}");
        assert!(
            msg.contains("fd 2 is closed; refusing"),
            "the child's only explanation must reach the operator: {msg}"
        );
        assert!(
            msg.contains("(stdout:"),
            "…and it must be LABELLED stdout, so nobody reads it as a document: {msg}"
        );
    }

    /// The other half of the rule: when stderr DID speak, stdout is a truncated
    /// document and quoting it is noise. Without this arm the rule above could
    /// ship as an unconditional dump of every failing child's stdout.
    #[test]
    fn a_failing_inspection_with_stderr_does_not_also_dump_its_partial_document() {
        let err = shell(
            r#"printf '%s' '{"inputs":[],"out'; echo 'the real reason' >&2; exit 4"#,
            Duration::from_secs(5),
        )
        .inspect(Path::new(LIB), "camera")
        .expect_err("non-zero exit is an error");
        let msg = err.to_string();
        assert!(
            msg.contains("the real reason") && !msg.contains("(stdout:"),
            "stderr spoke, so the truncated document must NOT ride along: {msg}"
        );
    }

    #[test]
    fn unusable_output_is_a_validation_error() {
        let err = shell("echo 'not json'", Duration::from_secs(5))
            .inspect(Path::new(LIB), "camera")
            .expect_err("garbage must not parse");
        assert!(err.to_string().contains("unusable info JSON"), "{err}");
    }

    /// Also the pin for `kill_group`'s benign-errno arm: on a HEALTHY
    /// inspection the group kill lands on a group whose only member is the
    /// unreaped zombie, which macOS refuses with EPERM (measured 30/30) and
    /// Linux allows (16/16). Treating that EPERM as a refusal puts one
    /// "process-group kill refused … a survivor may hold a pipe" in the log
    /// for every node library every Mac desk validates, burying the signal the
    /// warn exists to carry. The macOS half also asserts the benign arm really
    /// ran, so the absence assertion is not vacuous on the platform where it is
    /// the discriminator — read off [`BENIGN_KILL_REFUSALS`], not the `debug!`
    /// line, because `release_max_level_info` compiles that line out of a
    /// `--release` build and the pin would then fail on correct code.
    #[tracing_test::traced_test]
    #[test]
    fn a_healthy_inspection_parses_the_printed_document_with_the_shared_parser() {
        let script = r#"[ "$1" = /nowhere/libcamera.so ] || exit 9; printf '%s' '{"inputs":[{"name":"cmd","schema_hash":42,"trigger":true}],"outputs":[]}'"#;
        // A DELTA, not an absolute. `BENIGN_KILL_REFUSALS` is a
        // `thread_local!` and libtest spawns a fresh thread per test at every
        // concurrency level (it falls back to the caller's thread only when
        // that spawn fails with `WouldBlock`), so today the count starts at 0
        // here by construction. The delta is the guard for a harness that ever
        // REUSES threads, where a sibling arm's benign refusal would otherwise
        // satisfy this test's anti-vacuity half for it.
        let benign_before = BENIGN_KILL_REFUSALS.with(|seen| seen.get());
        let info = shell(script, Duration::from_secs(5))
            .inspect(Path::new(LIB), "camera")
            .expect("valid JSON parses");
        let cmd = info
            .input_meta()
            .iter()
            .find(|m| m.name == "cmd")
            .expect("input cmd");
        assert_eq!(cmd.schema_hash, 42);
        assert!(
            !logs_contain("process-group kill refused"),
            "a healthy inspection must not warn about its own group kill"
        );
        if cfg!(target_os = "macos") {
            let benign = BENIGN_KILL_REFUSALS.with(|seen| seen.get()) - benign_before;
            assert!(
                benign >= 1,
                "the ANTI-VACUITY half: on macOS the kill really is refused with EPERM, so the \
                 absence assertion above is only meaningful because the benign arm ran — it did \
                 not (delta {benign})"
            );
        }
    }

    /// A library's own load-time diagnostics reach the CLI user directly; the
    /// daemon path must not swallow them just because the inspection SUCCEEDED
    /// (the `warn!` is the whole of "logged, not fatal" in the struct doc and
    /// in docs/user-api.md).
    #[tracing_test::traced_test]
    #[test]
    fn stderr_on_a_successful_inspection_is_warned_not_swallowed() {
        let script = r#"echo careful >&2; printf '%s' '{"inputs":[],"outputs":[]}'"#;
        let info = shell(script, Duration::from_secs(5))
            .inspect(Path::new(LIB), "camera")
            .expect("stderr on a successful inspection is not a failure");
        assert!(info.input_names().is_empty());
        logs_assert(|lines: &[&str]| {
            let warns = count_at(
                lines,
                "WARN",
                "wrote to stderr during a successful inspection",
            );
            if warns != 1 {
                return Err(format!(
                    "expected exactly 1 WARN carrying the helper's stderr (a demoted line never \
                     reaches the daemon's log at all), got {warns}"
                ));
            }
            Ok(())
        });
        assert!(
            logs_contain("careful"),
            "the warn must carry what the helper actually said"
        );
    }

    /// The whole of stdout is the document, any JSON shape: a pretty-printed
    /// multi-line document parses exactly as it does in-process.
    #[test]
    fn a_multi_line_document_parses_like_the_in_process_path() {
        let script = "printf '%s\\n' '{' '  \"inputs\": [{\"name\": \"cmd\", \"schema_hash\": 7}],' '  \"outputs\": []' '}'";
        let info = shell(script, Duration::from_secs(5))
            .inspect(Path::new(LIB), "camera")
            .expect("pretty-printed JSON is a legal document");
        assert_eq!(info.input_meta()[0].schema_hash, 7);
    }

    /// The review reproduction: the helper exits 0 at once but a backgrounded
    /// descendant keeps stdout open. The join used to wait for it (1.2 s
    /// against a 100 ms deadline); the group kill now ends it. A 5 s deadline
    /// keeps the assertion about the FIX, not the runner's fork latency.
    #[test]
    fn a_descendant_holding_the_pipes_cannot_stretch_the_inspection_past_the_deadline() {
        let script = r#"sleep 30 & printf '%s' '{"inputs":[],"outputs":[]}'; exit 0"#;
        let started = Instant::now();
        let info = shell(script, Duration::from_secs(5))
            .inspect(Path::new(LIB), "camera")
            .expect("the helper's own output must parse");
        assert!(info.input_names().is_empty());
        assert!(
            started.elapsed() < Duration::from_secs(2),
            "the inspection waited for the descendant: {:?}",
            started.elapsed()
        );
    }

    #[test]
    fn a_flooding_document_is_rejected_at_the_cap_not_buffered_and_not_as_a_crash() {
        let script = r#"head -c 8000000 /dev/zero; printf '%s' '{"inputs":[],"outputs":[]}'"#;
        let err = shell(script, Duration::from_secs(10))
            .inspect(Path::new(LIB), "camera")
            .expect_err("a flood must be rejected");
        let msg = err.to_string();
        assert!(msg.contains("the info document exceeded"), "{msg}");
        assert!(msg.contains(&MAX_INSPECT_OUTPUT_BYTES.to_string()), "{msg}");
        assert!(
            !msg.contains("CRASHED") && !msg.contains("exited with"),
            "a flood must not be reported as a crash or a bad exit: {msg}"
        );
    }

    #[test]
    fn a_flooding_stderr_is_rejected_at_the_cap_too() {
        let script = r#"printf '%s' '{"inputs":[],"outputs":[]}'; head -c 8000000 /dev/zero >&2"#;
        let err = shell(script, Duration::from_secs(10))
            .inspect(Path::new(LIB), "camera")
            .expect_err("a stderr flood must be rejected");
        let msg = err.to_string();
        assert!(msg.contains("stderr (load-time output) exceeded"), "{msg}");
        assert!(
            !msg.contains("exited with") && !msg.contains("CRASHED"),
            "a flood must not be reported as a bad exit: {msg}"
        );
        assert!(
            msg.len() < 4096,
            "the stderr excerpt is bounded: {} bytes",
            msg.len()
        );
    }

    /// A crash that also flooded is reported as the crash, with the flood
    /// appended — the signal is the actionable fact.
    #[test]
    fn a_crash_after_a_flood_is_reported_as_the_crash() {
        let script = r#"head -c 8000000 /dev/zero >&2; kill -ABRT $$"#;
        let err = shell(script, Duration::from_secs(10))
            .inspect(Path::new(LIB), "camera")
            .expect_err("a crash must surface");
        let msg = err.to_string();
        assert!(msg.contains("CRASHED") && msg.contains("SIGABRT"), "{msg}");
        assert!(
            msg.contains("stderr (load-time output) exceeded"),
            "the flood rides along: {msg}"
        );
    }

    /// An escapee that left the process group (`setsid`, via python's
    /// `os.setsid` so the arm runs on macOS too) and holds BOTH pipes open
    /// cannot stretch the inspection: the collector returns READER_GRACE after
    /// the reap and the verdict names the held document channel — and the held
    /// stderr beside it, which is the corroborating half of the same signature.
    ///
    /// What actually decides here is the VERDICT, not the wall: an unbounded
    /// collect either SUCCEEDS (the escapee's pipe closed before the reader's
    /// own deadline) or fails on the HeldOpen note — never on the escapee note
    /// asserted below — so `expect_err` or the `outside its group` assertion
    /// fires first. The wall is a second, coarser guard, and its two sides are
    /// now decoupled:
    ///
    /// * healthy — TWO python startups (the `exit 9` capability probe, then
    ///   the escapee), the sentinel poll loop, and [`READER_GRACE`]. The whole
    ///   cost sits inside the timed window because the sentinel wait precedes
    ///   the helper's exit; the measured range for it lives in ONE place,
    ///   [`escapee_wall_message`], and is deliberately not restated here.
    /// * un-fixed — the SMALLER of the escapee's own sleep and the reader
    ///   deadline ([`ESCAPEE_INSPECT_TIMEOUT`] plus [`READER_GRACE`]), plus
    ///   those startups. The escapee is the pipe's last holder, but the reader
    ///   stops on its own bound whether or not the collector waits; a const
    ///   assert keeps the two within one [`READER_GRACE`] of each other and
    ///   the escapee's timer starts AFTER the startups, so which of them binds
    ///   comes down to that offset — usually the deadline.
    ///
    /// So the wall is [`ESCAPEE_WALL`] against BOTH bounds, held apart by the
    /// const asserts beside [`ESCAPEE_WALL`] (it stays at most half of each,
    /// so an unbounded collect is separated from a healthy one by far more
    /// than interpreter startup).
    /// Every number behind that — the measured healthy range and the margins
    /// it buys — is stated once, on [`escapee_wall_message`].
    #[tracing_test::traced_test]
    #[test]
    fn an_escaped_survivor_holding_stdout_is_reported_not_waited_for() {
        let sentinel = scratch_sentinel("stdout-escapee");
        let script = escapee_script(&sentinel, "", r#"{"inputs":[],"outputs":[]}"#);
        let started = Instant::now();
        let result = shell(&script, ESCAPEE_INSPECT_TIMEOUT).inspect(Path::new(LIB), "camera");
        let elapsed = started.elapsed();
        reap_escapee(&sentinel);
        let err = result.expect_err("a held-open document channel cannot be trusted");
        let msg = err.to_string();
        assert!(
            !msg.contains("exit status: 9"),
            "the arm needs python3 on this box: {msg}"
        );
        assert!(
            !msg.contains("exit status: 7"),
            "the escapee never reported its setsid, so nothing held the document channel: {msg}"
        );
        assert!(
            msg.contains("had not closed") && msg.contains("outside its group"),
            "{msg}"
        );
        assert!(
            msg.contains("the reader thread was not scheduled"),
            "the note lists the causes rather than asserting the escapee — USER_API promises \
             exactly this clause: {msg}"
        );
        // The escapee inherited BOTH pipes, so stderr was held too. That fact
        // is the corroborating signal for a forked survivor and must not be
        // dropped just because the DOCUMENT channel is what vetoed.
        assert!(
            msg.contains("stderr had not closed"),
            "a stderr held open by the same survivor must ride along in the verdict: {msg}"
        );
        logs_assert(|lines: &[&str]| {
            let warns = count_at(lines, "WARN", "stderr had not closed");
            if warns != 1 {
                return Err(format!(
                    "…and be LOGGED on this arm too (a demoted line never reaches the daemon's \
                     log at all), got {warns}"
                ));
            }
            Ok(())
        });
        assert!(elapsed < ESCAPEE_WALL, "{}", escapee_wall_message(elapsed));
    }

    /// An escapee holding only STDERR does not veto a good document — and the
    /// held-open stderr is LOGGED, which is the other half of that promise and
    /// the only thing standing between this arm and a vacuous pass.
    #[tracing_test::traced_test]
    #[test]
    fn an_escaped_survivor_holding_only_stderr_does_not_fail_a_good_document() {
        let sentinel = scratch_sentinel("stderr-escapee");
        let script = escapee_script(
            &sentinel,
            ">/dev/null",
            r#"{"inputs":[{"name":"cmd","schema_hash":3}],"outputs":[]}"#,
        );
        let started = Instant::now();
        let result = shell(&script, ESCAPEE_INSPECT_TIMEOUT).inspect(Path::new(LIB), "camera");
        let elapsed = started.elapsed();
        reap_escapee(&sentinel);
        let info = match result {
            Ok(info) => info,
            Err(e) => {
                let msg = e.to_string();
                assert!(
                    !msg.contains("exit status: 9"),
                    "the arm needs python3 on this box: {msg}"
                );
                assert!(
                    !msg.contains("exit status: 7"),
                    "the escapee never reported its setsid, so nothing held stderr: {msg}"
                );
                panic!("stderr held open is not the document's problem: {msg}");
            }
        };
        assert_eq!(info.input_meta()[0].schema_hash, 3);
        assert!(elapsed < ESCAPEE_WALL, "{}", escapee_wall_message(elapsed));
        logs_assert(|lines: &[&str]| {
            let warns = count_at(lines, "WARN", "stderr had not closed");
            if warns != 1 {
                return Err(format!(
                    "expected exactly 1 WARN naming the held-open stderr (not fatal, but never \
                     silent), got {warns}"
                ));
            }
            Ok(())
        });
    }

    /// The reader's OWN bound, on an idle pipe and on a busy one: the write
    /// end stays open in the test, so only the deadline can end the read.
    ///
    /// Each arm runs `drain` on its OWN thread and takes the verdict with a
    /// `recv_timeout`. Inline, dropping the `poll(2)` gate in `readable_before`
    /// — the regression this arm exists to catch — makes the idle `drain` block
    /// in `read(2)` on a held-open pipe forever; libtest has no per-test
    /// timeout, so the whole binary wedges until the CI job's cancel instead of
    /// printing a failing assertion. On a thread the same regression reads as a
    /// timed-out recv that names the arm.
    #[test]
    fn a_reader_returns_at_its_deadline_on_an_idle_and_on_a_busy_pipe() {
        // Idle: nobody writes.
        let (read_end, write_end) = os_pipe();
        let (tx, rx) = std::sync::mpsc::channel();
        let started = Instant::now();
        std::thread::spawn(move || {
            let mut read_end = read_end;
            let progress = Mutex::new(PipeOutput {
                bytes: Vec::new(),
                end: PipeEnd::Eof,
            });
            let end = drain(
                &mut read_end,
                Instant::now() + Duration::from_millis(300),
                &progress,
            );
            let _ = tx.send(end);
        });
        let end = rx
            .recv_timeout(Duration::from_secs(3))
            .expect("the reader must return on an IDLE held-open pipe: its poll(2) gate is bounded by the deadline");
        assert_eq!(end, PipeEnd::HeldOpen);
        assert!(
            started.elapsed() < Duration::from_secs(3),
            "{:?}",
            started.elapsed()
        );
        drop(write_end);

        // Busy: a writer floods for longer than the deadline. The verdict is a
        // throughput floor (the cap, 1 MiB, inside the deadline), so the window
        // is a whole second and the writer outlives it by seconds — a starved
        // writer must not be able to turn Overflowed into HeldOpen, and a
        // reader that ignores BOTH deadline checks must still fail on the
        // elapsed bound rather than sneak in under a short writer cap.
        let (read_end, mut write_end) = os_pipe();
        let writer = std::thread::spawn(move || {
            let buf = [b'x'; 65536];
            let until = Instant::now() + Duration::from_secs(5);
            while Instant::now() < until {
                if write_end.write_all(&buf).is_err() {
                    break;
                }
            }
        });
        let progress = Arc::new(Mutex::new(PipeOutput {
            bytes: Vec::new(),
            end: PipeEnd::Eof,
        }));
        let reader_progress = Arc::clone(&progress);
        let (tx, rx) = std::sync::mpsc::channel();
        let started = Instant::now();
        std::thread::spawn(move || {
            let mut read_end = read_end;
            let end = drain(
                &mut read_end,
                Instant::now() + Duration::from_secs(1),
                &reader_progress,
            );
            let _ = tx.send(end);
        });
        let end = rx
            .recv_timeout(Duration::from_secs(3))
            .expect("the reader must return on a BUSY pipe too: neither deadline check may be skipped while data keeps arriving");
        assert_eq!(
            end,
            PipeEnd::Overflowed,
            "a busy pipe past the cap ends as overflowed"
        );
        assert!(
            started.elapsed() < Duration::from_secs(3),
            "the reader did not honour its deadline on a busy pipe: {:?}",
            started.elapsed()
        );
        assert!(
            progress.lock().unwrap().bytes.len() <= MAX_INSPECT_OUTPUT_BYTES,
            "retention is capped"
        );
        let _ = writer.join();
    }

    /// `child_exited_unreaped` must observe WITHOUT reaping: the whole
    /// `waitid(WNOWAIT)` design is that the zombie still pins the group id when
    /// `kill_group` runs, so a reaping implementation would signal a pid the
    /// kernel is free to have recycled.
    ///
    /// The oracle asks the KERNEL, never the same `Child` a second time:
    /// `std::process::Child::try_wait` MEMOISES the status, so a
    /// `child_exited_unreaped` reverted to `child.try_wait()?.is_some()` answers
    /// every later query — `wait()` included — out of its own cache and passes
    /// an assertion phrased that way, having already reaped the pid.
    ///
    /// Measured on this platform: zombie ⇒ `(rc 0, si_pid == pid)`; reaped ⇒
    /// `(rc -1, ECHILD)`. Two notes on what this probe is NOT:
    ///
    /// * NOT `waitpid(WNOHANG | WNOWAIT)` (the shape the review proposed):
    ///   Darwin rejects `WNOWAIT` on `waitpid` with EINVAL — measured — so that
    ///   form fails on CORRECT code and would have to be `cfg`'d to Linux.
    /// * NOT corroborated with `kill(pid, 0)`: that spelling is a liveness
    ///   PREDICATE, and `shm_state`'s source walk requires the repo to hold
    ///   exactly one copy of it (a second copy drifts toward reading a live
    ///   namespace as free). It also could not have added a kill the `waitid`
    ///   probe misses — the variant this arm exists to catch reaps the child, which
    ///   the probe reports as ECHILD.
    #[test]
    fn exit_is_observed_without_reaping() {
        let mut child = Command::new("/bin/sh")
            .args(["-c", "exit 3"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .unwrap();
        let pid = child.id() as libc::pid_t;
        let until = Instant::now() + Duration::from_secs(5);
        while !child_exited_unreaped(&mut child).unwrap() {
            assert!(Instant::now() < until, "child never exited");
            std::thread::sleep(POLL);
        }
        // SAFETY: a zeroed siginfo_t out-parameter; a non-blocking,
        // non-consuming query for this test's own child.
        let (rc, seen) = unsafe {
            let mut info: libc::siginfo_t = std::mem::zeroed();
            let rc = libc::waitid(
                libc::P_PID,
                pid as libc::id_t,
                &mut info,
                libc::WEXITED | libc::WNOHANG | libc::WNOWAIT,
            );
            (rc, info.si_pid())
        };
        assert!(
            rc == 0 && seen == pid,
            "the kernel must still hold the zombie (waitid rc={rc}, si_pid={seen}): the \
             observation REAPED the child, so `kill_group` would signal a pid the kernel is \
             free to have recycled"
        );
        assert_eq!(child.wait().unwrap().code(), Some(3));
    }

    /// Hand-written oracles for every note an unfinished pipe can render, one
    /// per [`HelperState`]. No process: the wording IS the contract, and it is
    /// the only thing an operator gets.
    ///
    /// Each state knows different facts, and a note that asserts one the arm
    /// cannot have is worse than a note that names none — an operator told the
    /// helper may still hold the pipe, on the arm where a foreign reaper
    /// already took it, audits a process that does not exist. Full equality
    /// rather than `contains`, so a clause cannot be dropped silently: nothing
    /// else in this file asserted "the reader thread was not scheduled" until
    /// the stdout-escapee arm did, and docs/user-api.md promises that exact text.
    ///
    /// SCOPE: this pins the TEXT of each state. WHICH state each
    /// failure arm passes is not reachable from a test — all THREE routes to
    /// `ReapedElsewhere` (an ECHILD from the wait, and one from either reap)
    /// need a foreign reaper (a `wait(-1)` racing our own child), and a test
    /// that raced one would reap other tests' children. The errno half of that
    /// decision is pinned on its own, in
    /// `a_failed_reap_is_classified_by_its_errno_never_by_the_arm_that_saw_it`.
    #[test]
    fn the_unfinished_note_states_only_what_each_helper_state_knows() {
        let grace = READER_GRACE.as_millis();

        let reaped = PipeEnd::Unfinished
            .note("stdout", HelperState::Reaped)
            .expect("an unfinished pipe always has a note");
        assert_eq!(
            reaped,
            format!(
                "stdout had not closed {grace} ms after the helper was reaped (it may still be \
                 held — by a process outside its group, for example, or one the kill could not \
                 end in time — or the reader thread was not scheduled); its output is what \
                 arrived before"
            )
        );

        let unreaped = PipeEnd::Unfinished
            .note("stdout", HelperState::Unreaped)
            .expect("an unfinished pipe always has a note");
        assert_eq!(
            unreaped,
            format!(
                "stdout had not closed {grace} ms after the helper's group was signalled (the \
                 helper itself may still hold it, for example, or the reader thread was not \
                 scheduled); its output is what arrived before"
            )
        );

        let elsewhere = PipeEnd::Unfinished
            .note("stdout", HelperState::ReapedElsewhere)
            .expect("an unfinished pipe always has a note");
        assert_eq!(
            elsewhere,
            format!(
                "stdout had not closed {grace} ms after the helper was reaped by something else \
                 in this process (the helper is gone, and the group kill was skipped, or may \
                 have been aimed at a leader pid the foreign reap had already freed, so it \
                 proves nothing about the group: a descendant may still hold it, for example, \
                 or the reader thread was not scheduled); its output is what arrived before"
            )
        );

        // The negatives each state must NOT claim, spelled out so a future
        // rewording cannot reintroduce the self-contradiction these notes were
        // split to remove: the ECHILD verdict used to say "its process group
        // was not signalled" and, two clauses later, "after the helper's group
        // was signalled (the helper itself may still hold it)".
        for (state, note) in [
            (HelperState::Reaped, &reaped),
            (HelperState::ReapedElsewhere, &elsewhere),
        ] {
            assert!(
                !note.contains("the helper itself may still hold it"),
                "{state:?}: the helper is gone on this arm, so it cannot be offered as the \
                 holder: {note}"
            );
        }
        assert!(
            !elsewhere.contains("group was signalled ("),
            "ReapedElsewhere: the kill was skipped, or may have been aimed at a freed pid — \
             neither supports a flat 'the group was signalled': {elsewhere}"
        );
        // …and it must not swing the other way either. On the reap-error arms
        // the kill DID run, and whether the foreign reaper freed the leader pid
        // before or after it landed is not observable, so a note that states
        // the kill was aimed at a freed pid asserts a race it did not witness.
        assert!(
            elsewhere.contains("may have been aimed"),
            "ReapedElsewhere: the kill's effect is UNKNOWN on the reap-error arms, so the \
             note must hedge it rather than close the disjunction: {elsewhere}"
        );
        assert!(
            !unreaped.contains("was reaped"),
            "Unreaped: no exit status was observed, so nothing may claim a reap: {unreaped}"
        );
        for note in [&reaped, &unreaped, &elsewhere] {
            assert!(
                note.contains("for example"),
                "no note may claim a COMPLETE cause list — an in-group member the SIGKILL could \
                 not end in time is another holder the collector cannot see: {note}"
            );
        }

        // The pipe name is the caller's, and both notes travel together.
        assert!(PipeEnd::Unfinished
            .note("stderr", HelperState::Reaped)
            .expect("an unfinished pipe always has a note")
            .starts_with("stderr had not closed"));
        assert_eq!(PipeEnd::Eof.note("stdout", HelperState::Reaped), None);
        let both = notes_text(
            &PipeEnd::Unfinished,
            &PipeEnd::Unfinished,
            HelperState::ReapedElsewhere,
        );
        assert_eq!(
            both,
            format!(
                "; {elsewhere}; {}",
                elsewhere.replacen("stdout", "stderr", 1)
            )
        );
    }

    /// The shared report for "SIGKILL sent, budget expired, helper still
    /// alive". Both reaping arms call it; before the lift, the wait-error arm
    /// reaped bounded and threw the outcome away, so that arm's verdict carried
    /// no pid, no budget and no "left un-reaped", and its log line did not
    /// exist at all.
    ///
    /// SCOPE: this pins the report itself. Neither CALL SITE is reachable from
    /// a test — one needs a helper wedged in an uninterruptible syscall past
    /// [`REAP_BUDGET`], the other a `waitid(2)` errno that fixed flags and a
    /// stack `siginfo_t` cannot produce.
    #[tracing_test::traced_test]
    #[test]
    fn the_unreaped_report_names_the_pid_the_budget_and_the_deadline_at_error() {
        let fragment = report_unreaped(4242, "camera", Path::new(LIB), Duration::from_secs(30));
        assert_eq!(
            fragment,
            "the inspector (pid 4242) was sent SIGKILL and was still alive 2s later (stuck in \
             the kernel), so it is left un-reaped and the request gave up on it; the inspection \
             deadline is 30s"
        );
        logs_assert(|lines: &[&str]| {
            let errors = count_at(lines, "ERROR", "left un-reaped");
            if errors != 1 {
                return Err(format!(
                    "the zombie now belongs to the daemon, so it must be named where the \
                     operator looks for it — exactly one ERROR, got {errors}"
                ));
            }
            Ok(())
        });
        assert!(
            logs_contain("4242") && logs_contain("camera"),
            "the log line must carry the pid and the node, or it cannot be grepped for"
        );
    }

    /// The errno decides what a failed reap says about the helper — never the
    /// arm that saw it. Hand oracles, no process.
    ///
    /// Both reap-error sites route through one classifier, which is what stops
    /// them diverging again: the reap inside the wait-error arm used to map
    /// EVERY failure to `Unreaped` while its sibling discriminated ECHILD, so
    /// on one arm a helper a foreign reaper had already taken was still
    /// offered to the operator as the possible holder — while `HelperState`'s
    /// own docs claimed the discrimination for both.
    ///
    /// SCOPE: this pins the CLASSIFIER, not that each site calls it.
    /// Neither call site is reachable from a test — one needs a `waitid(2)`
    /// errno that fixed flags and a stack `siginfo_t` cannot produce, and both
    /// need a foreign reaper racing our own child, which a test cannot arrange
    /// without reaping other tests' children.
    #[test]
    fn a_failed_reap_is_classified_by_its_errno_never_by_the_arm_that_saw_it() {
        assert_eq!(
            helper_state_after_failed_reap(&std::io::Error::from_raw_os_error(libc_echild())),
            HelperState::ReapedElsewhere,
            "ECHILD is a foreign reaper: the helper is GONE, so offering it as the possible \
             holder sends the operator after a process that does not exist"
        );
        for errno in [libc::EINVAL, libc::EINTR, libc::EPERM, libc::EFAULT] {
            assert_eq!(
                helper_state_after_failed_reap(&std::io::Error::from_raw_os_error(errno)),
                HelperState::Unreaped,
                "errno {errno} left the helper un-reaped, where it may well be the holder"
            );
        }
        // An error carrying no errno at all is not ECHILD, and must fall to the
        // side that claims no reap rather than the one that claims a foreign
        // reaper nobody observed.
        assert_eq!(
            helper_state_after_failed_reap(&std::io::Error::other("no errno")),
            HelperState::Unreaped
        );
    }

    /// The report for a reap that failed with an errno of its own. Before it,
    /// the wait-error arm's reap threw that errno away: no verdict fragment,
    /// no log line, and a helper left un-reaped with nothing saying why —
    /// the twin of the discard `report_unreaped` closed one round earlier.
    ///
    /// The errno is a FIELD, so an operator can grep `error=`; the message is
    /// constant, which is also what the repo-wide `tracing_field_discipline`
    /// ratchet requires. WARN, not ERROR: the request already carries the
    /// failure in its verdict, and the helper is not (yet) known to be a
    /// zombie the daemon owns — that is `report_unreaped`'s louder claim.
    ///
    /// SCOPE: the call site is unreachable for the reason its sibling's is
    /// (a `waitid(2)` errno fixed flags cannot produce); this pins the report.
    #[tracing_test::traced_test]
    #[test]
    fn the_failed_reap_report_names_the_errno_in_a_field_at_warn() {
        let error = std::io::Error::from_raw_os_error(libc::EPERM);
        let fragment = report_failed_reap(4242, "camera", Path::new(LIB), &error);
        assert_eq!(fragment, format!("reaping it failed: {error}"));
        logs_assert(|lines: &[&str]| {
            let warns = count_at(lines, "WARN", FAILED_REAP_MESSAGE);
            if warns != 1 {
                return Err(format!(
                    "a reap that failed for a reason of its own must be logged where the \
                     operator looks — exactly one WARN, got {warns}"
                ));
            }
            let louder = count_at(lines, "ERROR", FAILED_REAP_MESSAGE);
            if louder != 0 {
                return Err(format!(
                    "…and not at ERROR, which is `report_unreaped`'s claim about a zombie the \
                     daemon now owns: got {louder}"
                ));
            }
            Ok(())
        });
        assert!(
            logs_contain(&format!("error={error}")),
            "the errno belongs in a FIELD, not spliced into the message: an operator greps \
             error="
        );
        assert!(
            logs_contain("4242") && logs_contain("camera"),
            "the line must carry the pid and the node, or it cannot be attributed"
        );
    }

    /// The message BOTH reap-error arms log under. It is asserted here rather
    /// than spelled into each `count_at` so that a message change cannot make
    /// the counts silently agree on zero.
    const FAILED_REAP_MESSAGE: &str = "node inspector: the reap failed";

    /// The reporter is ARM-NEUTRAL, which is what lets the main path share it.
    ///
    /// The main path's reap-error arm renders `reaping the node inspector of
    /// '<node>' failed: <errno>` for the REQUESTER and used to log nothing at
    /// all: the same helper left un-reaped as on the wait-error arm, WARNed
    /// there with the errno in a field and silent here. It now calls
    /// [`warn_failed_reap`] — but only a message that names no arm can serve
    /// both, or an operator asking "did a reap fail?" needs one grep per arm
    /// and gets no hits from the wording that does not match.
    ///
    /// SCOPE: this pins the REPORTER, not that the main path calls
    /// it. Neither reap-error call site is reachable from a test —
    /// `Child::try_wait` is `waitpid(WNOHANG)`, whose realistic errno is
    /// ECHILD (classified `ReapedElsewhere`, which reports nothing), and the
    /// sibling additionally needs a `waitid(2)` errno that fixed flags and a
    /// stack `siginfo_t` cannot produce. Deleting the new call site therefore
    /// fails nothing here; what it would restore is the asymmetry above.
    #[tracing_test::traced_test]
    #[test]
    fn the_failed_reap_warn_is_arm_neutral_so_both_arms_share_one_grep() {
        let error = std::io::Error::from_raw_os_error(libc::EPERM);
        warn_failed_reap(4242, "camera", Path::new(LIB), &error);
        logs_assert(|lines: &[&str]| {
            let warns = count_at(lines, "WARN", FAILED_REAP_MESSAGE);
            if warns != 1 {
                return Err(format!(
                    "the main path's reap-error arm logs through this reporter too, so it must \
                     emit exactly one WARN under the shared message, got {warns}"
                ));
            }
            Ok(())
        });
        assert!(
            !logs_contain("after a failed wait"),
            "the message must name NO arm: the main path's reap failure is not 'after a failed \
             wait', and an operator greps one string for both"
        );
        assert!(
            logs_contain(&format!("error={error}")),
            "the errno belongs in a FIELD on this arm too: an operator greps error="
        );
        assert!(
            logs_contain("4242") && logs_contain("camera"),
            "the line must carry the pid and the node, or it cannot be attributed"
        );
    }

    #[test]
    fn the_pipe_excerpt_is_bounded_and_printable() {
        let mut noise = vec![0u8; 5000];
        noise.extend_from_slice(b"real reason\n");
        let text = bounded_excerpt(&noise);
        assert!(text.starts_with("… "), "{text}");
        assert!(text.ends_with("real reason"), "{text}");
        assert!(text.len() <= EXCERPT_BYTES + 8, "{}", text.len());
        assert!(!text.contains('\0'), "control bytes are replaced");
        assert_eq!(bounded_excerpt(b"  short  "), "short");
    }

    #[test]
    fn a_hanging_inspection_is_killed_at_the_deadline() {
        let started = Instant::now();
        let err = shell("sleep 30", Duration::from_millis(300))
            .inspect(Path::new(LIB), "camera")
            .expect_err("a hang must time out");
        assert!(
            err.to_string().contains("did not finish inspection"),
            "{err}"
        );
        assert!(
            started.elapsed() < Duration::from_secs(10),
            "the child was not killed promptly"
        );
    }

    #[test]
    fn an_absent_inspector_program_is_reported_not_panicked() {
        let err = SubprocessInspector::new(
            PathBuf::from("/nonexistent/inspector"),
            vec![],
            Duration::from_secs(1),
        )
        .inspect(Path::new(LIB), "camera")
        .expect_err("missing program");
        assert!(
            err.to_string()
                .contains("could not start the node inspector"),
            "{err}"
        );
    }

    /// A raw pipe pair as owned `File`s (both ends are plain descriptors).
    fn os_pipe() -> (std::fs::File, std::fs::File) {
        use std::os::fd::FromRawFd as _;
        let mut fds = [0 as libc::c_int; 2];
        // SAFETY: pipe(2) into a 2-slot array.
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0, "pipe(2) failed");
        // SAFETY: both descriptors were just created and are owned here.
        unsafe {
            (
                std::fs::File::from_raw_fd(fds[0]),
                std::fs::File::from_raw_fd(fds[1]),
            )
        }
    }
}
