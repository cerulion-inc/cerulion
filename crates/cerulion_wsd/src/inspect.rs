//! The child half of `graph.validate`'s isolation (`cerulion-wsd --inspect-node`).
//!
//! The parent takes the child's WHOLE stdout as the info document, so nothing
//! the library prints while loading may reach that channel. The child keeps it
//! clean by construction: [`isolate_document_channel`] moves the original
//! stdout to a private, close-on-exec descriptor and points fd 1 at fd 2 BEFORE
//! the library is loaded, so anything its load-time code prints — Rust
//! `println!`, C `printf`, a buffered stream flushed at exit — lands on stderr,
//! and anything it `exec`s does not inherit the document channel.
//!
//! RESIDUAL — close-on-exec covers `exec`, not `fork`: a constructor that
//! `fork()`s and never execs inherits the document descriptor and holds the
//! parent's document pipe open after this process exits. Nothing here can
//! prevent that, and the parent does not guess: its verdict states only what it
//! OBSERVED — the pipe had not closed within the collector's grace after the
//! helper was REAPED, which is a different anchor from the inspection deadline
//! that ends a still-running read — and it names the possibilities without
//! asserting one:
//!
//! * a process outside the helper's group — a fork that also LEAVES the group,
//!   which is what glibc `daemon(3)` does with its `setsid`. The parent kills
//!   the helper's whole process group, so leaving it is what makes a survivor
//!   possible;
//! * one the group kill could not end in time — where a plain fork-and-sleep
//!   child belongs: it inherits the helper's pgid, so the group kill DOES
//!   reach it and it can only ever be this possibility, never the one above;
//! * or a reader thread that was not scheduled.
//!
//! USER_API's `graph.validate` paragraph describes that verdict.

use std::fs::File;
use std::io;
use std::os::fd::FromRawFd as _;

/// WHICH half of the isolation refused.
///
/// The caller reports the two on DIFFERENT descriptors, so the distinction is
/// a type rather than a substring of the message: only [`Self::StderrUnusable`]
/// means stderr cannot carry the explanation, and routing a refusal onto fd 1
/// in any other case writes a diagnostic straight into the parent's DOCUMENT
/// pipe — the one channel this module exists to keep clean.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IsolationRefusal {
    /// fd 2 is not open. Nothing may be written to stderr; fd 1 is still
    /// untouched (this is checked BEFORE the redirect) and is the one channel
    /// known open, so the caller reports there.
    StderrUnusable,
    /// The descriptor surgery itself failed — `F_DUPFD_CLOEXEC` on fd 1 (an
    /// exhausted fd table: EMFILE) or `dup2(2, 1)`. stderr is open and good;
    /// the explanation belongs there, and fd 1 is still the document channel.
    SurgeryFailed,
}

/// [`isolate_document_channel`]'s refusal: which half failed, plus the OS error
/// underneath it.
///
/// `Display` is the OS error's own text, and `From<IsolationError>` yields that
/// error, so a caller that only wants to propagate can `?` into an
/// [`io::Result`] and lose nothing but the discriminator.
#[derive(Debug)]
pub struct IsolationError {
    refusal: IsolationRefusal,
    source: io::Error,
}

impl IsolationError {
    /// Which half refused — the routing decision, never a message match.
    pub fn refusal(&self) -> IsolationRefusal {
        self.refusal
    }
}

impl std::fmt::Display for IsolationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Display::fmt(&self.source, f)
    }
}

impl std::error::Error for IsolationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.source)
    }
}

impl From<IsolationError> for io::Error {
    fn from(error: IsolationError) -> Self {
        error.source
    }
}

/// Move the original stdout to a private close-on-exec descriptor (returned
/// as an owned `File`) and redirect fd 1 to fd 2.
///
/// Refuses BEFORE touching fd 1 if fd 2 is not open: load-time output would
/// then have nowhere to go, and `dup2(2, 1)` would fail only AFTER the document
/// descriptor had already been taken — leaving the caller holding a document
/// handle it must not use, with no stderr to explain on. Refusing first leaves
/// both descriptors exactly as they were, so the caller still has fd 1 (the one
/// channel it knows is open) to report on.
///
/// The other two refusals leave stderr perfectly usable, which is why the
/// `Err` carries [`IsolationRefusal`]: see its docs for the routing rule.
pub fn isolate_document_channel() -> Result<File, IsolationError> {
    // SAFETY: fcntl F_GETFD on a descriptor number only reports whether it is
    // open; it changes nothing.
    if unsafe { libc::fcntl(2, libc::F_GETFD) } < 0 {
        return Err(IsolationError {
            refusal: IsolationRefusal::StderrUnusable,
            source: io::Error::other(
                "stderr (fd 2) is not open, so load-time output would have nowhere to go",
            ),
        });
    }
    // SAFETY: F_DUPFD_CLOEXEC returns a fresh descriptor >= 3 that we own,
    // marked close-on-exec so an exec'd grandchild cannot hold the document
    // channel open.
    let document_fd = unsafe { libc::fcntl(1, libc::F_DUPFD_CLOEXEC, 3) };
    if document_fd < 0 {
        return Err(IsolationError {
            refusal: IsolationRefusal::SurgeryFailed,
            source: io::Error::last_os_error(),
        });
    }
    // SAFETY: `document_fd` is the descriptor just returned; this File owns it.
    let document = unsafe { File::from_raw_fd(document_fd) };
    // SAFETY: dup2(2, 1) replaces fd 1 with a copy of fd 2, both open.
    if unsafe { libc::dup2(2, 1) } < 0 {
        return Err(IsolationError {
            refusal: IsolationRefusal::SurgeryFailed,
            source: io::Error::last_os_error(),
        });
    }
    Ok(document)
}
