// SPDX-License-Identifier: AGPL-3.0-only
//! The desk daemons' shared UDS control-socket primitives: preparing a just-ACCEPTED
//! stream and bounded, RESUMABLE writes over it.
//!
//! Both `cerulion-netd` and `cerulion_vizd` serve NDJSON control responses over a
//! `UnixStream`. On macOS the accepted socket INHERITS `O_NONBLOCK` from the
//! nonblocking listener (BSD `accept` inheritance; `set_read_timeout` /
//! `set_write_timeout` set `SO_RCVTIMEO`/`SO_SNDTIMEO` but do NOT clear the
//! nonblocking flag). MEASURED on macOS 26.0.1: a stream accepted from a
//! `set_nonblocking(true)` listener reports `O_NONBLOCK` set, and still reports it
//! set after `set_read_timeout(200ms)`. That one fact has TWO consequences:
//!
//! * **Writes:** once a response line exceeds the ~8 KiB UDS send buffer,
//!   a plain `writeln!` (== `write_all`) fills the buffer, the next `write` returns
//!   `WouldBlock` instantly, `write_all` ABORTS, and the daemon drops the
//!   connection after ~one send buffer — the client reads a truncated 8192-byte line
//!   then EOF ("EOF while parsing a string at line 1 column 8192"). A 75-topic
//!   catalog / demand-table reply is a real >8 KiB response.
//! * **Reads:** each daemon's per-connection read loop is PACED by its
//!   `read` blocking for `SO_RCVTIMEO`; on a nonblocking socket the read returns
//!   `EAGAIN` in microseconds instead, so the loop's `WouldBlock => continue` arm runs
//!   an unbounded tight loop. Measured on macOS: `cerulion-vizd` at
//!   99.4 % CPU and `cerulion-netd` at 198.9 % (two idle connection threads) with an
//!   EMPTY demand table and nothing being viewed. [`prepare_accepted_stream`] clears
//!   `O_NONBLOCK` at the ONE seam both daemons go through, so the timeouts they
//!   already set are what actually paces them.
//!
//! [`prepare_accepted_stream`] does NOT make the resume loop redundant: a
//! blocking socket with `SO_SNDTIMEO` returns a SHORT count (or `WouldBlock`/`TimedOut`
//! with none placed) when the budget expires mid-line, which is exactly what
//! `write_all_progress` resumes from — the classifier already covers both spellings.
//!
//! [`write_line_bounded`] fixes this ONCE for both daemons: it RESUMES the partial
//! write across a momentarily-full send buffer (polling writability), and aborts
//! only when NO forward progress happens within the caller's budget — the genuine
//! "consumer stopped reading" signal. It is LOGGING-AGNOSTIC: it returns a
//! [`UdsWriteOutcome`] the caller maps to its own log level + `bool` (netd logs a
//! plain `debug!`; vizd routes the timeout through its once-per-regime flood latch),
//! so neither daemon's log surface changes.
//!
//! The budget bounds the INTER-PROGRESS gap, NOT the total write time (by design): a
//! reader that keeps trickling — draining a little on every budget window — completes
//! the write however long that takes, because each byte placed RESETS the clock. Only
//! a genuinely stalled peer (no bytes placed for a whole budget window) is dropped.
//! Since the control-response threads are bounded in number and the caller picks the
//! budget, a slow-but-live consumer holding a thread is the intended trade (never
//! truncation); a truly wedged one is bounded by the budget.

use std::io::{self, Write};
use std::os::unix::io::{AsRawFd, RawFd};
use std::os::unix::net::UnixStream;
use std::time::{Duration, Instant};

/// How long each writability poll ([`poll_writable`]) waits before re-checking the
/// caller's no-progress budget inside [`write_all_progress`]'s resume loop. Short
/// enough to bound how long a genuinely-dead consumer pins the handler thread (the
/// budget is re-checked ~once per slice); long enough to avoid a busy poll. A live
/// reader's buffer drains well within one slice, so `poll` returns the instant space
/// frees — a healthy big-line write pays no per-slice latency.
const WRITE_POLL_SLICE: Duration = Duration::from_millis(100);

/// What to do after one `write` syscall, decided PURELY from its outcome + how long
/// forward progress has been stalled. Extracted so the progress-vs-no-progress
/// classification is unit-testable against oracle vectors with no real socket/clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WriteAction {
    /// Wrote `n` (> 0) bytes — advance the cursor and reset the progress clock.
    Advance(usize),
    /// `Interrupted` (EINTR) — retry the write immediately (not a stall).
    RetryNow,
    /// Send buffer momentarily full, but still within the no-progress budget —
    /// wait (bounded) for writability, then resume the partial write.
    WaitRetry,
    /// `write` returned `Ok(0)` — the peer is gone; no progress is possible.
    PeerClosed,
    /// Buffer full AND no forward progress for the whole budget — the consumer
    /// genuinely stopped reading. Drop the connection.
    NoProgressTimeout,
    /// Any other error — a normal disconnect (BrokenPipe/etc.). Drop.
    IoError,
}

/// Classify ONE `write` outcome (its byte count or [`io::ErrorKind`]) given how long
/// forward progress has been stalled and the no-progress `budget`. Pure — the
/// progress/no-progress decision lives here, not tangled with the socket loop.
fn classify_write_outcome(
    result: Result<usize, io::ErrorKind>,
    no_progress_elapsed: Duration,
    budget: Duration,
) -> WriteAction {
    match result {
        Ok(0) => WriteAction::PeerClosed,
        Ok(n) => WriteAction::Advance(n),
        Err(io::ErrorKind::Interrupted) => WriteAction::RetryNow,
        Err(io::ErrorKind::WouldBlock) | Err(io::ErrorKind::TimedOut) => {
            if no_progress_elapsed >= budget {
                WriteAction::NoProgressTimeout
            } else {
                WriteAction::WaitRetry
            }
        }
        Err(_) => WriteAction::IoError,
    }
}

/// Block (bounded by `slice`) until `fd` is writable, so the resume loop does not
/// busy-spin on a nonblocking socket's `WouldBlock`. Best-effort: `poll` errors /
/// `EINTR` / timeout all just return so the caller re-issues the `write`, which
/// re-classifies (advancing on space, re-checking the budget otherwise).
fn poll_writable(fd: RawFd, slice: Duration) {
    let mut pfd = libc::pollfd {
        fd,
        events: libc::POLLOUT,
        revents: 0,
    };
    let timeout_ms = slice.as_millis().min(i32::MAX as u128) as i32;
    // SAFETY: `pfd` is a valid, initialized single-element array; `poll` only reads
    // `fd`/`events` and writes `revents` (which we ignore). No pointer escapes.
    unsafe {
        libc::poll(&mut pfd, 1, timeout_ms);
    }
}

/// Why [`write_line_bounded`] stopped. `Complete` means every byte was delivered;
/// the other variants are drop-the-connection reasons the caller logs at its own
/// level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UdsWriteOutcome {
    /// The whole slice (line + newline) was delivered.
    Complete,
    /// No forward progress within the budget — the consumer stopped reading. Carries
    /// how far the failing segment got, for a `debug!`.
    NoProgressTimeout { written: usize, total: usize },
    /// `write` returned `Ok(0)` mid-write — the peer closed.
    PeerClosed { written: usize, total: usize },
    /// A normal disconnect (BrokenPipe / ConnectionReset / etc.).
    IoError,
}

impl UdsWriteOutcome {
    /// Whether the whole slice was delivered (the caller returns `true` / keeps the
    /// connection).
    pub fn is_complete(self) -> bool {
        matches!(self, UdsWriteOutcome::Complete)
    }
}

/// Write every byte of `bytes` to `stream`, RESUMING partial writes across a
/// momentarily-full send buffer (`WouldBlock`/`TimedOut`) and aborting ONLY when NO
/// forward progress happens within `budget` — that no-progress window is the genuine
/// "consumer stopped reading" signal. `pub(crate)` — the public surface is
/// exactly [`write_line_bounded`] + [`UdsWriteOutcome`]; this is exposed only to the
/// in-crate socketpair tests.
pub(crate) fn write_all_progress(
    stream: &mut UnixStream,
    bytes: &[u8],
    budget: Duration,
) -> UdsWriteOutcome {
    let fd = stream.as_raw_fd();
    let total = bytes.len();
    let mut written = 0usize;
    let mut last_progress = Instant::now();
    while written < total {
        let outcome = stream.write(&bytes[written..]).map_err(|e| e.kind());
        match classify_write_outcome(outcome, last_progress.elapsed(), budget) {
            WriteAction::Advance(n) => {
                written += n;
                last_progress = Instant::now();
            }
            WriteAction::RetryNow => {} // EINTR — loop and re-issue immediately.
            WriteAction::WaitRetry => poll_writable(fd, WRITE_POLL_SLICE),
            WriteAction::PeerClosed => return UdsWriteOutcome::PeerClosed { written, total },
            WriteAction::NoProgressTimeout => {
                return UdsWriteOutcome::NoProgressTimeout { written, total }
            }
            WriteAction::IoError => return UdsWriteOutcome::IoError,
        }
    }
    UdsWriteOutcome::Complete
}

/// Prepare a just-ACCEPTED control-socket stream for a daemon's timeout-PACED
/// read/write loop. The ONE seam both `cerulion-netd` and `cerulion_vizd`
/// go through, because two copies of this is how the class recurs.
///
/// Clears `O_NONBLOCK` FIRST, then installs the caller's timeouts. The clear is the
/// load-bearing line: on BSD/macOS an accepted socket inherits the listener's
/// nonblocking flag, and neither `set_read_timeout` nor `set_write_timeout` clears it
/// (they only set `SO_RCVTIMEO`/`SO_SNDTIMEO`), so a handler that assumes its `read`
/// blocks for the timeout instead spins on instant `EAGAIN` — see the module docs.
/// On Linux `accept(2)` does not inherit file status flags, so the accepted socket is
/// already blocking and this call is a no-op there: behaviour is IDENTICAL on both
/// platforms, which is the point.
///
/// Both timeouts are installed here rather than left to the caller so the ordering
/// (clear, then pace) cannot be got wrong at one site and right at the other. Any
/// step failing means the socket is unusable — the caller drops the connection.
pub fn prepare_accepted_stream(
    stream: &UnixStream,
    read_timeout: Duration,
    write_timeout: Duration,
) -> io::Result<()> {
    stream.set_nonblocking(false)?;
    stream.set_read_timeout(Some(read_timeout))?;
    stream.set_write_timeout(Some(write_timeout))?;
    Ok(())
}

/// Write one NDJSON `line` + its terminating newline to `stream`, resuming partial
/// writes so a line larger than the send buffer is never truncated.
/// Returns the [`UdsWriteOutcome`] the caller maps to its own log + `bool`. Small
/// lines take the exact same path with no added latency (they fit in one `write`).
pub fn write_line_bounded(
    stream: &mut UnixStream,
    line: &str,
    budget: Duration,
) -> UdsWriteOutcome {
    // Body then its terminating newline — two slices avoid allocating a joined
    // buffer (`writeln!` also writes in pieces). The newline is tiny but still
    // routed through the resume loop so a buffer boundary at end-of-body cannot drop
    // it.
    match write_all_progress(stream, line.as_bytes(), budget) {
        UdsWriteOutcome::Complete => write_all_progress(stream, b"\n", budget),
        stopped => stopped,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ----------------------------------------------------------------------
    // The pure write-outcome classifier (progress vs no-progress).
    // Oracle vectors — each row is (result, elapsed, budget) → expected action,
    // hand-written, never a self-compare.
    // ----------------------------------------------------------------------

    const BUDGET: Duration = Duration::from_secs(2);

    #[test]
    fn classify_ok_bytes_advances_and_ok_zero_is_peer_closed() {
        // A positive byte count advances by exactly that many bytes...
        assert_eq!(
            classify_write_outcome(Ok(1), Duration::ZERO, BUDGET),
            WriteAction::Advance(1)
        );
        assert_eq!(
            classify_write_outcome(Ok(8192), Duration::from_secs(10), BUDGET),
            WriteAction::Advance(8192),
            "Advance ignores the elapsed budget — progress is progress"
        );
        // ...while Ok(0) is a WriteZero-class peer-gone signal, never an advance.
        assert_eq!(
            classify_write_outcome(Ok(0), Duration::ZERO, BUDGET),
            WriteAction::PeerClosed
        );
    }

    #[test]
    fn classify_interrupted_retries_immediately_regardless_of_elapsed() {
        // EINTR is not a stall — retry now, never consult the budget.
        assert_eq!(
            classify_write_outcome(Err(io::ErrorKind::Interrupted), Duration::ZERO, BUDGET),
            WriteAction::RetryNow
        );
        assert_eq!(
            classify_write_outcome(
                Err(io::ErrorKind::Interrupted),
                Duration::from_secs(9999),
                BUDGET
            ),
            WriteAction::RetryNow,
            "EINTR never times out — the send buffer never entered the decision"
        );
    }

    #[test]
    fn classify_wouldblock_within_budget_waits_and_at_or_past_budget_aborts() {
        // The headline write contract: a full send buffer WAITS (resumes) while
        // there is still budget, and aborts ONLY once no progress has happened for
        // the whole budget window. Covers WouldBlock (macOS nonblocking) AND
        // TimedOut (Linux blocking + SO_SNDTIMEO).
        for kind in [io::ErrorKind::WouldBlock, io::ErrorKind::TimedOut] {
            assert_eq!(
                classify_write_outcome(Err(kind), Duration::ZERO, BUDGET),
                WriteAction::WaitRetry,
                "{kind:?}: fresh stall waits"
            );
            assert_eq!(
                classify_write_outcome(Err(kind), BUDGET - Duration::from_millis(1), BUDGET),
                WriteAction::WaitRetry,
                "{kind:?}: just under budget still waits"
            );
            assert_eq!(
                classify_write_outcome(Err(kind), BUDGET, BUDGET),
                WriteAction::NoProgressTimeout,
                "{kind:?}: exactly at the budget boundary aborts (>=)"
            );
            assert_eq!(
                classify_write_outcome(Err(kind), BUDGET + Duration::from_secs(1), BUDGET),
                WriteAction::NoProgressTimeout,
                "{kind:?}: past the budget aborts"
            );
        }
    }

    #[test]
    fn classify_other_errors_are_normal_disconnects() {
        for kind in [
            io::ErrorKind::BrokenPipe,
            io::ErrorKind::ConnectionReset,
            io::ErrorKind::NotConnected,
        ] {
            assert_eq!(
                classify_write_outcome(Err(kind), Duration::ZERO, BUDGET),
                WriteAction::IoError,
                "{kind:?} is a normal disconnect (drop, never resume)"
            );
        }
    }

    #[test]
    fn outcome_is_complete_only_for_complete() {
        assert!(UdsWriteOutcome::Complete.is_complete());
        assert!(!UdsWriteOutcome::NoProgressTimeout {
            written: 8192,
            total: 20000
        }
        .is_complete());
        assert!(!UdsWriteOutcome::PeerClosed {
            written: 0,
            total: 10
        }
        .is_complete());
        assert!(!UdsWriteOutcome::IoError.is_complete());
    }

    // ----------------------------------------------------------------------
    // The COMPOSED resume loop over a
    // real socketpair. Deterministic on EVERY platform (unlike the daemon e2e,
    // whose resume path only engages when a reply exceeds the platform send
    // buffer — 8 KiB on macOS but ~208 KiB on Linux). We force the resume path
    // with a small SO_SNDBUF/SO_RCVBUF + a nonblocking writer + a lagging reader,
    // so a modest payload spans MANY WouldBlock/poll cycles.
    // ----------------------------------------------------------------------

    use std::io::Read;

    /// Shrink a socket buffer via `setsockopt`. The kernel clamps to its own min
    /// and (on Linux) doubles the value; the exact size is never asserted — the point
    /// is "small enough that a >buffer payload triggers many resume cycles".
    fn set_sock_buf(fd: RawFd, opt: libc::c_int, bytes: libc::c_int) {
        let r = unsafe {
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                opt,
                &bytes as *const libc::c_int as *const libc::c_void,
                std::mem::size_of::<libc::c_int>() as libc::socklen_t,
            )
        };
        assert_eq!(
            r,
            0,
            "setsockopt(SO_*BUF) failed: {}",
            io::Error::last_os_error()
        );
    }

    /// A `(tx, rx)` socketpair: `tx` NONBLOCKING (forces the resume path on every
    /// platform, mirroring the macOS accept-inherited O_NONBLOCK) + both buffers
    /// shrunk to ~4 KiB so a >buffer payload spans many WouldBlock/poll cycles.
    fn small_buffer_pair() -> (UnixStream, UnixStream) {
        let (tx, rx) = UnixStream::pair().expect("socketpair");
        tx.set_nonblocking(true).expect("tx nonblocking");
        // Flow control on AF_UNIX stream is receiver-side on Linux (peer sk_rcvbuf)
        // and sender-side on macOS (SO_SNDBUF); shrink BOTH so it's small on either.
        set_sock_buf(tx.as_raw_fd(), libc::SO_SNDBUF, 4096);
        set_sock_buf(rx.as_raw_fd(), libc::SO_RCVBUF, 4096);
        (tx, rx)
    }

    /// A recognizable non-trivial byte pattern (a mis-ordered / dropped chunk in the
    /// resume is caught, unlike an all-`0xAB` fill).
    fn pattern(len: usize) -> Vec<u8> {
        (0..len).map(|i| (i % 251) as u8).collect()
    }

    #[test]
    fn resume_loop_completes_a_large_write_across_many_cycles_byte_exact() {
        // 256 KiB through a ~4 KiB buffer ⇒ ≥64 resume cycles are STRUCTURALLY
        // required — completing byte-exact IS the proof of "multiple WouldBlock/poll
        // cycles". A lagging reader (small reads + a tiny sleep) keeps
        // the writer's buffer full so it genuinely re-blocks each cycle.
        let (mut tx, rx) = small_buffer_pair();
        let payload = pattern(256 * 1024);
        let expected = payload.clone(); // hot-path-alloc-ok: test-only (socketpair unit test oracle copy)
        let want = expected.len();
        let reader = std::thread::spawn(move || {
            let mut rx = rx;
            let mut got = Vec::with_capacity(want); // hot-path-alloc-ok: test-only (reader-thread capture buffer)
            let mut buf = [0u8; 3072];
            while got.len() < want {
                match rx.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => {
                        got.extend_from_slice(&buf[..n]);
                        std::thread::sleep(Duration::from_millis(1)); // lag
                    }
                    Err(ref e) if e.kind() == io::ErrorKind::Interrupted => continue,
                    Err(_) => break,
                }
            }
            got
        });
        // Generous budget — this arm pins COMPLETION + byte-exactness, not timing.
        let outcome = write_all_progress(&mut tx, &payload, Duration::from_secs(30));
        assert_eq!(outcome, UdsWriteOutcome::Complete);
        drop(tx); // EOF so the reader stops even if it over-reads.
        let got = reader.join().expect("reader thread");
        assert_eq!(got, expected, "byte-exact across the whole resume");
    }

    #[test]
    fn resume_loop_budget_resets_on_progress_even_when_total_exceeds_budget() {
        // The budget bounds the INTER-PROGRESS gap, not total time (see the
        // module doc). A reader that trickles ~4 KiB every ~3 ms drains a 256 KiB
        // payload over ~64 cycles ⇒ total ≳ 192 ms, comfortably ABOVE the 150 ms
        // budget — yet each gap (~3 ms) is 50× UNDER it, so the write completes. A
        // "budget == total-write-time" bug would abort here.
        let (mut tx, rx) = small_buffer_pair();
        let payload = pattern(256 * 1024);
        let want = payload.len();
        let reader = std::thread::spawn(move || {
            let mut rx = rx;
            let mut got = 0usize;
            let mut buf = [0u8; 4096];
            while got < want {
                match rx.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => {
                        got += n;
                        std::thread::sleep(Duration::from_millis(3)); // steady trickle
                    }
                    Err(ref e) if e.kind() == io::ErrorKind::Interrupted => continue,
                    Err(_) => break,
                }
            }
        });
        let t0 = Instant::now();
        let outcome = write_all_progress(&mut tx, &payload, Duration::from_millis(150));
        let elapsed = t0.elapsed();
        assert_eq!(
            outcome,
            UdsWriteOutcome::Complete,
            "a steadily-trickling reader completes despite total time > budget"
        );
        assert!(
            elapsed > Duration::from_millis(150),
            "total write time ({elapsed:?}) must exceed the 150 ms budget — else this \
             does not distinguish per-gap from total budgeting"
        );
        drop(tx);
        reader.join().expect("reader thread");
    }

    #[test]
    fn resume_loop_trips_no_progress_when_the_peer_never_reads_within_the_budget() {
        // A never-reading peer fills the buffer then makes zero progress → the loop
        // aborts within ~the budget. `_rx` is held open (not dropped) so
        // this is a WEDGED reader, not a closed one.
        let (mut tx, _rx) = small_buffer_pair();
        let payload = pattern(256 * 1024);
        let budget = Duration::from_millis(300);
        let t0 = Instant::now();
        let outcome = write_all_progress(&mut tx, &payload, budget);
        let elapsed = t0.elapsed();
        match outcome {
            UdsWriteOutcome::NoProgressTimeout { written, total } => {
                assert!(written < total, "aborted mid-write: {written} of {total}");
                assert_eq!(total, payload.len());
            }
            other => panic!("expected NoProgressTimeout, got {other:?}"),
        }
        // Bounded: ≥ the budget, and well under a runaway (budget + a few poll slices).
        assert!(
            elapsed >= budget && elapsed < budget + Duration::from_secs(2),
            "abort must land near the budget, got {elapsed:?}"
        );
    }

    #[test]
    fn resume_loop_reports_a_drop_reason_when_the_reader_closes_mid_write() {
        // Portable arm: the reader accepts a few KiB then CLOSES → the
        // composed loop returns a drop reason (IoError via EPIPE, or PeerClosed),
        // never a hang or panic. Rust's runtime ignores SIGPIPE, so the write EPIPEs
        // rather than killing the process.
        let (mut tx, rx) = small_buffer_pair();
        let payload = pattern(512 * 1024);
        let reader = std::thread::spawn(move || {
            let mut rx = rx;
            let mut got = 0usize;
            let mut buf = [0u8; 2048];
            while got < 8 * 1024 {
                match rx.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => got += n,
                    Err(_) => break,
                }
            }
            // rx drops here → the read end closes mid-write.
        });
        let outcome = write_all_progress(&mut tx, &payload, Duration::from_secs(5));
        reader.join().expect("reader thread");
        assert!(
            matches!(
                outcome,
                UdsWriteOutcome::IoError | UdsWriteOutcome::PeerClosed { .. }
            ),
            "a mid-write reader close is a clean drop reason, got {outcome:?}"
        );
    }

    // ----------------------------------------------------------------------
    // The ACCEPTED-stream preparation, over a REAL nonblocking
    // `UnixListener` + a REAL `accept` — the exact shape both daemons run. A
    // socketpair cannot serve here: the whole property under test is what
    // `accept(2)` does with the LISTENER's flags.
    //
    // The load-bearing oracle is a LOWER bound on elapsed time. The failure mode
    // is "the reads came back too FAST" (a spin), so a floor can only be missed
    // by a genuine regression — a loaded runner makes reads SLOWER, never faster.
    // (The inverse of the loaded-runner trap, where a ceiling on work would
    // have failed open under load.)
    // ----------------------------------------------------------------------

    use std::os::unix::net::UnixListener;

    /// `O_NONBLOCK` state of `fd`, read straight from the kernel via `F_GETFL` —
    /// `UnixStream` exposes no accessor, and the flag is the entire subject here.
    fn is_nonblocking(fd: RawFd) -> bool {
        // SAFETY: `fd` is a live, owned socket descriptor; `F_GETFL` takes no
        // variadic argument and only READS the descriptor's status flags.
        let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
        assert!(
            flags >= 0,
            "fcntl(F_GETFL) failed: {}",
            io::Error::last_os_error()
        );
        flags & libc::O_NONBLOCK != 0
    }

    /// A bound NONBLOCKING `UnixListener` (the daemons' shape) + its socket path,
    /// on a unique pid+nanos path so the whole module stays parallel-safe.
    fn nonblocking_listener() -> (UnixListener, std::path::PathBuf) {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("clock")
            .subsec_nanos();
        // hot-path-alloc-ok: test-only socket-path construction (a `#[cfg(test)]`
        // helper); no publish/receive path can reach it.
        let path = std::env::temp_dir().join(format!("udsw-{}-{}.sock", std::process::id(), nanos));
        let _ = std::fs::remove_file(&path);
        let listener = UnixListener::bind(&path).expect("bind");
        listener
            .set_nonblocking(true)
            .expect("nonblocking listener");
        (listener, path)
    }

    /// Connect a client, then poll the nonblocking listener until it accepts —
    /// returning `(accepted, client)`. The client is RETURNED (not dropped) so the
    /// connection stays open and IDLE, which is what the reads below measure.
    fn accept_one(listener: &UnixListener, path: &std::path::Path) -> (UnixStream, UnixStream) {
        let client = UnixStream::connect(path).expect("connect");
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            match listener.accept() {
                Ok((s, _)) => return (s, client),
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                    assert!(Instant::now() < deadline, "accept never completed");
                    std::thread::sleep(Duration::from_millis(2));
                }
                Err(e) => panic!("accept: {e}"),
            }
        }
    }

    /// Drain-free idle reads: the connection carries no data, so every read either
    /// blocks for `SO_RCVTIMEO` (prepared) or returns `EAGAIN` at once (the bug).
    fn time_idle_reads(stream: &mut UnixStream, reads: usize) -> Duration {
        let mut buf = [0u8; 64];
        let t0 = Instant::now();
        for _ in 0..reads {
            match stream.read(&mut buf) {
                Ok(n) => panic!("an idle connection yielded {n} bytes"),
                Err(e) => assert!(
                    matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock
                            | io::ErrorKind::TimedOut
                            | io::ErrorKind::Interrupted
                    ),
                    "unexpected idle-read error: {e:?}"
                ),
            }
        }
        t0.elapsed()
    }

    const IDLE_TIMEOUT: Duration = Duration::from_millis(50);
    const IDLE_READS: usize = 4;

    #[test]
    fn a_prepared_accepted_stream_is_blocking_so_idle_reads_are_paced_by_the_timeout() {
        // THE pin (universal — macOS and Linux). After preparation the accepted
        // socket is BLOCKING, so the read loop both daemons run is paced by
        // SO_RCVTIMEO instead of spinning on instant EAGAIN.
        let (listener, path) = nonblocking_listener();
        let (mut accepted, _client) = accept_one(&listener, &path);

        prepare_accepted_stream(&accepted, IDLE_TIMEOUT, Duration::from_secs(2)).expect("prepare");
        assert!(
            !is_nonblocking(accepted.as_raw_fd()),
            "prepare_accepted_stream must clear O_NONBLOCK on the accepted socket"
        );

        let elapsed = time_idle_reads(&mut accepted, IDLE_READS);
        // Each of the 4 reads must block for ~IDLE_TIMEOUT. The floor drops one
        // whole read's worth (3 of 4) so timer granularity can never flake it,
        // while a spin — microseconds for all four — misses it by ~5 orders of
        // magnitude. MEASURED with the flag left set: 5.8 µs for TEN reads.
        let floor = IDLE_TIMEOUT * (IDLE_READS as u32 - 1);
        assert!(
            elapsed >= floor,
            "{IDLE_READS} idle reads took {elapsed:?}, expected at least {floor:?} \
             — the accepted socket is still nonblocking and the read loop is SPINNING"
        );

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn set_read_timeout_alone_never_clears_the_inherited_nonblocking_flag() {
        // WHY clearing the flag is a separate call and not "just set the timeouts".
        // Accept a stream, install ONLY the timeout,
        // and observe that the flag is exactly as `accept` left it.
        let (listener, path) = nonblocking_listener();
        let (accepted, _client) = accept_one(&listener, &path);

        let inherited = is_nonblocking(accepted.as_raw_fd());
        accepted
            .set_read_timeout(Some(IDLE_TIMEOUT))
            .expect("read timeout");
        assert_eq!(
            is_nonblocking(accepted.as_raw_fd()),
            inherited,
            "set_read_timeout sets SO_RCVTIMEO only — it must not change O_NONBLOCK \
             either way (it is not the remedy)"
        );

        // The inheritance itself is a PLATFORM fact, so the arm that proves this
        // module is not vacuous is cfg'd to the platform where the bug lives.
        // BSD/macOS: `accept` inherits the listener's flags, so the socket is
        // nonblocking and the idle reads return in microseconds — the spin,
        // reproduced. Linux: `accept(2)` does not inherit file status flags, so the
        // socket is already blocking and there is nothing to clear.
        #[cfg(target_os = "macos")]
        {
            // `mut` lives inside this arm only — the Linux half never reads the
            // stream, and a top-level `mut` binding is an unused_mut there.
            let mut accepted = accepted;
            assert!(
                inherited,
                "macOS accept(2) inherits the listener's O_NONBLOCK — if this ever \
                 stops being true, prepare_accepted_stream's rationale changed"
            );
            let elapsed = time_idle_reads(&mut accepted, IDLE_READS);
            // The budget is deliberately generous: this is the file's ONLY upper
            // bound on wall time, it runs on hosted macOS CI runners, where
            // whole-window preemptions happen, and it fails
            // CLOSED (a stall reads as spurious red, never a false green).
            //
            // The numbers are this TEST's `IDLE_TIMEOUT` (50 ms), NOT the daemons'
            // production `CONN_READ_TIMEOUT` (200 ms). Ceiling
            // = `IDLE_TIMEOUT * (IDLE_READS - 1)` = 150 ms, i.e. 3x one paced read.
            // A PREPARED socket would spend 4 x 50 = 200 ms here, so the ceiling
            // sits 1.33x below the blocking side — a modest separation, and the
            // reason it is safe is the OTHER side: the bug returns EAGAIN in
            // microseconds (MEASURED 5.792 µs for TEN reads), leaving ~25,000x of
            // headroom before a spin could ever reach 150 ms.
            let unprepared_ceiling = IDLE_TIMEOUT * (IDLE_READS as u32 - 1);
            assert!(
                elapsed < unprepared_ceiling,
                "an UNPREPARED accepted socket should return EAGAIN instantly \
                 (the bug), but {IDLE_READS} reads took {elapsed:?} \
                 (ceiling {unprepared_ceiling:?})"
            );
        }
        #[cfg(target_os = "linux")]
        assert!(
            !inherited,
            "Linux accept(2) does not inherit file status flags — the accepted \
             socket is expected to be blocking already"
        );

        let _ = std::fs::remove_file(&path);
    }

    /// How far ABOVE a requested `SO_RCVTIMEO`/`SO_SNDTIMEO` the kernel is
    /// allowed to store it. `setsockopt` takes a `timeval` but the kernel keeps
    /// the deadline in its own scheduler-tick units, so reading one back returns
    /// the request ROUNDED UP to that granularity — MEASURED on an aarch64
    /// Jetson (4 ms jiffies): a 70 ms request reads back as 72 ms.
    /// x86 CI and macOS happen to round to the requested value, so
    /// an exact-equality assertion would be green everywhere except there.
    ///
    /// 10 ms covers a 100 Hz kernel (the coarsest `CONFIG_HZ` in practice) with
    /// room to spare, and is far below the 60 ms gap between the two values
    /// asserted below — so the bands cannot overlap and a preparation that
    /// installed one timeout on both sockopts still fails.
    const TIMEOUT_TICK_SLACK: Duration = Duration::from_millis(10);

    /// The kernel's stored timeout must be the caller's request, allowing only
    /// the tick rounding above — never a different value, and never rounded DOWN
    /// (a shorter budget would cut a read/write off early).
    fn assert_timeout_within_tick_band(actual: Option<Duration>, requested: Duration, what: &str) {
        let actual = actual.unwrap_or_else(|| panic!("{what} timeout must be installed, not None"));
        assert!(
            actual >= requested && actual <= requested + TIMEOUT_TICK_SLACK,
            "the {what} timeout must be the one the caller asked for (allowing the \
             kernel's tick rounding): requested {requested:?}, read back {actual:?}, \
             band {requested:?}..={:?}",
            requested + TIMEOUT_TICK_SLACK
        );
    }

    #[test]
    fn preparation_installs_both_timeouts_not_just_the_read_one() {
        // The write timeout is the no-progress budget's kernel half; a
        // preparation that dropped it would leave a wedged consumer able to block
        // a handler thread forever.
        let (listener, path) = nonblocking_listener();
        let (accepted, _client) = accept_one(&listener, &path);

        // Two DISTINCT values, 60 ms apart: the write timeout cannot pass the
        // read timeout's band (or vice versa), so a preparation that installed
        // one value on both sockopts is still caught.
        let read_to = Duration::from_millis(70);
        let write_to = Duration::from_millis(130);
        prepare_accepted_stream(&accepted, read_to, write_to).expect("prepare");

        assert_timeout_within_tick_band(
            accepted.read_timeout().expect("read_timeout"),
            read_to,
            "read",
        );
        assert_timeout_within_tick_band(
            accepted.write_timeout().expect("write_timeout"),
            write_to,
            "write",
        );

        let _ = std::fs::remove_file(&path);
    }
}
