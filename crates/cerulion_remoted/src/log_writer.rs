// SPDX-License-Identifier: AGPL-3.0-only
//! The daemon's log writer: a bounded queue in front of stderr, written by one
//! thread of its own.
//!
//! The `cerulion-remoted` binary installs a [`LogWriter`] as the writer of its
//! `tracing_subscriber::fmt` subscriber ([`start`]); a host that embeds the
//! plane keeps its own subscriber. fmt formats each event on the thread that
//! logs it, with that event's timestamp. The writer copies the formatted line
//! into a buffer of its own and hands it to the queue, and the logging thread
//! goes on. One thread, `remoted-log`, takes the lines in the order they were
//! handed over and writes each to stderr. A write that blocks, on a full pipe
//! or a stalled journald, holds that thread and no other.
//!
//! The queue holds at most [`QUEUE_LINES`] lines and [`QUEUE_BYTES`] bytes. A
//! line that does not fit is dropped and counted, never waited for, and so is a
//! line whose write to stderr fails. [`lines_dropped`] reads the count since the
//! start; the wire plane's `status` reply carries it as `log_lines_dropped`.
//!
//! The writer thread reports drops through a [`FailureRegimeLatch`], one
//! dropped line being one failure. After each write, and when a drop wakes it
//! while it waits for a line, it reports the lines dropped since its last
//! report: one warn line naming how many (`dropped`) and the total
//! (`total_dropped`) when they open a regime or take the total across a power
//! of ten, and a debug line otherwise. Once the queue is empty, it ends an open
//! regime with one info line saying it caught up, with the regime's `dropped`
//! and the `total_dropped`; a regime of a single dropped line ends silently, as
//! the latch decides. These lines are formatted on the writer thread and
//! written straight to stderr, never queued. A report line whose own write
//! fails is not counted.
//!
//! The logging threads and the writer thread share one lock, the queue's, held
//! to push or pop one line, to count a drop, or to cut the queue at an exit,
//! and never across a format or a write. The queue reserves room for its lines,
//! up to [`QUEUE_LINES`], when it is built, so a push within that room does not
//! allocate until an exit cuts the queue.
//!
//! [`LogWriter::drain`] is an exit's wait for the queue.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::io::{self, Write};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant};

use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use tracing_subscriber::fmt::MakeWriter;

/// The most lines the queue holds: 4096. With [`QUEUE_BYTES`] it bounds what a
/// stderr that takes nothing can pin; a line of the daemon's is a few hundred
/// bytes, so the two bounds meet near 256 bytes a line, and this one caps the
/// per-line cost of short lines.
pub const QUEUE_LINES: usize = 4096;

/// The most bytes of formatted lines the queue holds: 1 MiB. A line longer
/// than this alone is always dropped and counted.
pub const QUEUE_BYTES: usize = 1 << 20;

/// Test-only: the environment variable that, in a build with the test seam,
/// sets a line bound of at most [`QUEUE_LINES`] for the binary's log queue
/// (`test_queue_lines`), from the moment the daemon has logged that it
/// listens (`shrink_queue_for_test`); the lines before that are queued under
/// the full bound. A build without the test seam never reads it.
#[cfg(feature = "test-seam")]
#[doc(hidden)]
pub const QUEUE_LINES_FOR_TEST_ENV: &str = "CERULION_REMOTED_LOG_QUEUE_LINES_FOR_TEST";

/// Test-only: the line bound a [`QUEUE_LINES_FOR_TEST_ENV`] value sets, which
/// can only shrink the queue. A value that parses as a positive `usize`,
/// whitespace around it trimmed, sets that many lines, clamped to
/// [`QUEUE_LINES`]: a larger one, up to `usize::MAX`, sets [`QUEUE_LINES`].
/// Any other value, zero among them, sets none, so the queue keeps
/// [`QUEUE_LINES`]; no value leaves it unbounded or with no room. Pure.
#[cfg(feature = "test-seam")]
pub(crate) fn test_queue_lines(value: &str) -> Option<usize> {
    let lines = value.trim().parse::<usize>().ok()?;
    (lines > 0).then(|| lines.min(QUEUE_LINES))
}

/// Test-only: apply the line bound [`QUEUE_LINES_FOR_TEST_ENV`] sets to the
/// writer [`start`] installed, if both are there. The daemon calls it right
/// after it logs that it listens, so a test that waits for that line never
/// loses it to that bound.
#[cfg(feature = "test-seam")]
pub(crate) fn shrink_queue_for_test() {
    let lines = std::env::var(QUEUE_LINES_FOR_TEST_ENV)
        .ok()
        .and_then(|value| test_queue_lines(&value));
    if let (Some(lines), Some(writer)) = (lines, INSTALLED.get()) {
        writer
            .inner
            .queue
            .test_lines
            .store(lines, Ordering::Release);
    }
}

/// The name of the writer thread.
const THREAD_NAME: &str = "remoted-log";

/// The newest queued lines an exit keeps when it discards the others: 8, so
/// up to that many of the last lines the queue took before the exit cut, such
/// as the stop thread's reason for the exit, come before the line that counts
/// the discarded ones.
pub const KEPT_AT_EXIT: usize = 8;

/// A queue's bounds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Limits {
    /// The most lines queued.
    pub lines: usize,
    /// The most bytes queued, over every queued line.
    pub bytes: usize,
}

impl Limits {
    /// [`QUEUE_LINES`] and [`QUEUE_BYTES`].
    pub const DEFAULT: Limits = Limits {
        lines: QUEUE_LINES,
        bytes: QUEUE_BYTES,
    };
}

/// The writer [`start`] installed, read by [`lines_dropped`].
static INSTALLED: OnceLock<LogWriter> = OnceLock::new();

thread_local! {
    /// While the writer thread formats a report line, the bytes of that line,
    /// which [`EventLine`] adds here instead of handing them to the queue.
    static REPORT: RefCell<Option<Vec<u8>>> = const { RefCell::new(None) };
}

/// The process's log writer: a `tracing_subscriber` [`MakeWriter`] whose
/// writers hand each formatted line to the queue. Clones share one queue. The
/// writer [`start`] built is kept for the process, so its thread runs until the
/// process exits; another writer's thread ends once every clone is dropped and
/// its queue is empty.
#[derive(Clone)]
pub struct LogWriter {
    inner: Arc<Inner>,
}

/// The queue, and the drop that tells the writer thread no line follows.
struct Inner {
    queue: Arc<Queue>,
}

impl Drop for Inner {
    fn drop(&mut self) {
        self.queue.lock().closed = true;
        self.queue.ready.notify_all();
    }
}

/// The lines handed over and not yet written, under the one lock the logging
/// threads and the writer thread share.
struct Queue {
    state: Mutex<State>,
    /// Notified when a line is queued, and when the last handle drops.
    ready: Condvar,
    /// Notified when a write ends.
    idle: Condvar,
    /// Lines dropped since the start: refused by a full queue, or lost to a
    /// failed write.
    dropped: AtomicU64,
    /// The value of `dropped` the writer thread last reported; stored by that
    /// thread only, before it clears `writing`.
    reported: AtomicU64,
    limits: Limits,
    /// Test-only: a line bound of at most [`QUEUE_LINES`], set once the daemon
    /// listens; `usize::MAX` for none.
    #[cfg(feature = "test-seam")]
    test_lines: std::sync::atomic::AtomicUsize,
}

struct State {
    lines: VecDeque<Vec<u8>>,
    /// The bytes of every line in `lines`.
    bytes: usize,
    /// A line is being written.
    writing: bool,
    /// Every handle is dropped.
    closed: bool,
}

impl Queue {
    fn new(limits: Limits) -> Self {
        Self {
            state: Mutex::new(State {
                lines: VecDeque::with_capacity(limits.lines.min(QUEUE_LINES)),
                bytes: 0,
                writing: false,
                closed: false,
            }),
            ready: Condvar::new(),
            idle: Condvar::new(),
            dropped: AtomicU64::new(0),
            reported: AtomicU64::new(0),
            limits,
            #[cfg(feature = "test-seam")]
            test_lines: std::sync::atomic::AtomicUsize::new(usize::MAX),
        }
    }

    /// The bounds in force: `limits`, with a test seam's line bound of at most
    /// [`QUEUE_LINES`] once one is set.
    fn bounds(&self) -> Limits {
        #[cfg(feature = "test-seam")]
        {
            let lines = self.test_lines.load(Ordering::Acquire);
            Limits {
                lines: self.limits.lines.min(lines),
                ..self.limits
            }
        }
        #[cfg(not(feature = "test-seam"))]
        {
            self.limits
        }
    }

    /// Whether lines were dropped that the writer thread has not reported.
    fn owes_report(&self) -> bool {
        self.dropped.load(Ordering::Acquire) != self.reported.load(Ordering::Acquire)
    }

    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Queue `line`, or drop and count it when it does not fit ([`admits`]);
    /// either way, wake the writer thread. A drop is counted under the lock,
    /// so a writer thread that waits for a line sees it or is woken by it.
    fn hand_over(&self, line: Vec<u8>) {
        let mut state = self.lock();
        let rejected = if admits(state.lines.len(), state.bytes, line.len(), self.bounds()) {
            state.bytes += line.len();
            state.lines.push_back(line);
            None
        } else {
            self.dropped.fetch_add(1, Ordering::AcqRel);
            Some(line)
        };
        drop(state);
        drop(rejected);
        self.ready.notify_one();
    }

    /// Wait until no line is queued, none is being written or reported, and
    /// no dropped line is left unreported, for at most what is left of
    /// `budget` since `since`. Returns whether that happened.
    fn wait_idle(&self, since: Instant, budget: Duration) -> bool {
        let mut state = self.lock();
        loop {
            if state.lines.is_empty() && !state.writing && !self.owes_report() {
                return true;
            }
            let left = budget.saturating_sub(since.elapsed());
            if left.is_zero() {
                return false;
            }
            state = self
                .idle
                .wait_timeout(state, left)
                .unwrap_or_else(PoisonError::into_inner)
                .0;
        }
    }
}

/// Whether a queue that holds `lines` lines of `bytes` bytes in all takes one
/// more line of `len` bytes under `limits`. Pure.
fn admits(lines: usize, bytes: usize, len: usize, limits: Limits) -> bool {
    lines < limits.lines && bytes.saturating_add(len) <= limits.bytes
}

/// Start the writer thread over stderr with `limits` and record the writer for
/// [`lines_dropped`]. The binary calls it once, before it installs its
/// subscriber.
///
/// # Errors
///
/// The error the writer thread's spawn returned.
pub fn start(limits: Limits) -> io::Result<LogWriter> {
    let writer = spawn(limits, Box::new(io::stderr()))?;
    let _ = INSTALLED.set(writer.clone());
    Ok(writer)
}

/// The lines the writer [`start`] installed has dropped since it started, or
/// `None` in a process that installed none.
pub fn lines_dropped() -> Option<u64> {
    INSTALLED.get().map(LogWriter::dropped)
}

/// A writer whose thread writes to `sink`.
fn spawn(limits: Limits, mut sink: Box<dyn Write + Send>) -> io::Result<LogWriter> {
    let queue = Arc::new(Queue::new(limits));
    let thread_queue = queue.clone();
    std::thread::Builder::new()
        .name(THREAD_NAME.into())
        .spawn(move || write_lines(&thread_queue, &mut *sink))?;
    Ok(LogWriter {
        inner: Arc::new(Inner { queue }),
    })
}

/// The writer thread: write each queued line to `sink` in order and report
/// drops after each write, or report drops a drop woke it for
/// ([`Reporter::after_write`]); return once the queue is closed and empty.
/// `writing` stays set until the report is written, so a drain waits for it.
fn write_lines(queue: &Queue, sink: &mut dyn Write) {
    let mut reporter = Reporter::default();
    loop {
        let line = {
            let mut state = queue.lock();
            loop {
                if let Some(line) = state.lines.pop_front() {
                    state.bytes -= line.len();
                    state.writing = true;
                    break Some(line);
                }
                if queue.owes_report() {
                    state.writing = true;
                    break None;
                }
                if state.closed {
                    return;
                }
                state = queue
                    .ready
                    .wait(state)
                    .unwrap_or_else(PoisonError::into_inner);
            }
        };
        if let Some(line) = line {
            if sink.write_all(&line).is_err() {
                queue.dropped.fetch_add(1, Ordering::AcqRel);
            }
        }
        let empty = queue.lock().lines.is_empty();
        reporter.after_write(queue, empty, sink);
        queue.lock().writing = false;
        queue.idle.notify_all();
    }
}

impl LogWriter {
    /// Lines dropped since this writer started.
    pub fn dropped(&self) -> u64 {
        self.inner.queue.dropped.load(Ordering::Acquire)
    }

    /// Wait for the queue to be written, for at most `grace`: until no line is
    /// queued, none is being written or reported and no dropped line is left
    /// unreported, for at most four fifths of `grace`. Then all but the newest
    /// [`KEPT_AT_EXIT`] queued lines are discarded, one error line naming how
    /// many (`discarded`), when there are any, is queued after those, and the
    /// wait goes on for the rest of `grace` or until the queue is written.
    /// Discarded lines are named in that line only, not in
    /// [`LogWriter::dropped`]. A write blocked in stderr holds the writer
    /// thread, not this wait.
    pub fn drain(&self, grace: Duration) -> Drained {
        let since = Instant::now();
        let queue = &self.inner.queue;
        if queue.wait_idle(since, grace / 5 * 4) {
            return Drained::Written;
        }
        let older = {
            let mut state = queue.lock();
            let kept_from = state.lines.len().saturating_sub(KEPT_AT_EXIT);
            let kept = state.lines.split_off(kept_from);
            state.bytes = kept.iter().map(Vec::len).sum();
            std::mem::replace(&mut state.lines, kept)
        };
        let discarded = u64::try_from(older.len()).unwrap_or(u64::MAX);
        drop(older);
        if discarded > 0 {
            tracing::error!(
                discarded,
                "cerulion_remoted: log lines still queued at the exit were discarded"
            );
        }
        let _ = queue.wait_idle(since, grace);
        Drained::Discarded { discarded }
    }
}

/// How [`LogWriter::drain`] ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Drained {
    /// No line was queued, none was being written and no dropped line was
    /// left unreported.
    Written,
    /// At four fifths of the grace, a write had not returned, lines were
    /// still queued or a dropped line was not yet reported; the `discarded`
    /// oldest queued lines were discarded.
    Discarded {
        /// The queued lines discarded.
        discarded: u64,
    },
}

impl<'a> MakeWriter<'a> for LogWriter {
    type Writer = EventLine<'a>;

    fn make_writer(&'a self) -> Self::Writer {
        EventLine {
            queue: &self.inner.queue,
            line: Vec::new(),
        }
    }
}

/// The writer of one event: it gathers the event's formatted bytes and, when
/// dropped, hands them to the queue as one line, or, on the writer thread
/// while it reports, adds them to the report.
pub struct EventLine<'a> {
    queue: &'a Queue,
    line: Vec<u8>,
}

impl Write for EventLine<'_> {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.line.extend_from_slice(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Drop for EventLine<'_> {
    fn drop(&mut self) {
        let line = std::mem::take(&mut self.line);
        if line.is_empty() {
            return;
        }
        let reporting = REPORT
            .try_with(|report| report.borrow().is_some())
            .unwrap_or(false);
        if reporting {
            REPORT.with(|report| {
                if let Some(report) = report.borrow_mut().as_mut() {
                    report.extend_from_slice(&line);
                }
            });
        } else {
            self.queue.hand_over(line);
        }
    }
}

/// What one batch of drops reports, by [`drop_report`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Report {
    /// The batch opened a regime: warn.
    Opened,
    /// The batch took the total across a power of ten: warn, with the
    /// failures the latch downgraded.
    StillDropping { suppressed: u64 },
    /// Neither: debug, with the failures the latch downgraded.
    Repeat { suppressed: u64 },
}

/// Feed `dropped` failures to `latch`, one per dropped line, and return what
/// the batch reports: [`Report::Opened`] when one opened the regime, else the
/// last [`Report::StillDropping`], else the last [`Report::Repeat`]. Pure but
/// for `latch`.
fn drop_report(latch: &mut FailureRegimeLatch, dropped: u64) -> Report {
    let mut report = Report::Repeat { suppressed: 0 };
    for _ in 0..dropped {
        report = match (report, latch.on_failure()) {
            (Report::Opened, _) | (_, RegimeDecision::Loud) => Report::Opened,
            (_, RegimeDecision::StillFailing { suppressed, .. }) => {
                Report::StillDropping { suppressed }
            }
            (still @ Report::StillDropping { .. }, RegimeDecision::Suppressed { .. }) => still,
            (Report::Repeat { .. }, RegimeDecision::Suppressed { suppressed }) => {
                Report::Repeat { suppressed }
            }
        };
    }
    report
}

/// The writer thread's drop regime.
#[derive(Default)]
struct Reporter {
    latch: FailureRegimeLatch,
    /// The lines dropped in the open regime.
    regime: u64,
}

impl Reporter {
    /// After a write: report the lines dropped since the last report, then,
    /// with the queue `empty` and no drop since that report, end an open
    /// regime.
    fn after_write(&mut self, queue: &Queue, empty: bool, sink: &mut dyn Write) {
        let total = queue.dropped.load(Ordering::Acquire);
        let dropped = total.saturating_sub(queue.reported.load(Ordering::Acquire));
        if dropped > 0 {
            queue.reported.store(total, Ordering::Release);
            self.regime = self.regime.saturating_add(dropped);
            let report = drop_report(&mut self.latch, dropped);
            write_report(sink, || match report {
                Report::Opened => tracing::warn!(
                    dropped,
                    total_dropped = total,
                    "cerulion_remoted: log lines were dropped: the log queue was full or a \
                     write to stderr failed"
                ),
                Report::StillDropping { suppressed } => tracing::warn!(
                    dropped,
                    total_dropped = total,
                    suppressed,
                    "cerulion_remoted: log lines were dropped: the log queue was full or a \
                     write to stderr failed"
                ),
                Report::Repeat { suppressed } => tracing::debug!(
                    dropped,
                    total_dropped = total,
                    suppressed,
                    "cerulion_remoted: log lines were dropped: the log queue was full or a \
                     write to stderr failed"
                ),
            });
        }
        if empty && self.latch.is_failing() && !queue.owes_report() {
            let regime = std::mem::take(&mut self.regime);
            if self.latch.on_success().is_some() {
                write_report(sink, || {
                    tracing::info!(
                        dropped = regime,
                        total_dropped = total,
                        "cerulion_remoted: the log writer caught up; the lines it dropped are \
                         counted in the wire status reply"
                    );
                });
            }
        }
    }
}

/// Run `log` with this thread's lines gathered as a report, then write the
/// report to `sink`.
fn write_report(sink: &mut dyn Write, log: impl FnOnce()) {
    REPORT.with(|report| *report.borrow_mut() = Some(Vec::new()));
    log();
    let report = REPORT
        .with(|report| report.borrow_mut().take())
        .unwrap_or_default();
    if !report.is_empty() {
        let _ = sink.write_all(&report);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A queue with no writer thread, and its writer.
    fn unstarted(limits: Limits) -> LogWriter {
        LogWriter {
            inner: Arc::new(Inner {
                queue: Arc::new(Queue::new(limits)),
            }),
        }
    }

    /// The lines a sink was written, and an optional gate it waits at, at most
    /// 20 s, before each write while the gate is shut.
    #[derive(Clone, Default)]
    struct Recorded {
        lines: Arc<Mutex<Vec<String>>>,
        gate: Arc<(Mutex<Gate>, Condvar)>,
        fail: bool,
    }

    #[derive(Default)]
    struct Gate {
        shut: bool,
        waiting: bool,
    }

    impl Recorded {
        fn shut(&self) {
            self.gate.0.lock().unwrap().shut = true;
        }

        fn open(&self) {
            self.gate.0.lock().unwrap().shut = false;
            self.gate.1.notify_all();
        }

        /// Wait, at most 20 s, until a write waits at the shut gate.
        fn until_a_write_waits(&self) {
            let (gate, wake) = &*self.gate;
            let gate = gate.lock().unwrap();
            let (gate, _) = wake
                .wait_timeout_while(gate, Duration::from_secs(20), |g| !g.waiting)
                .unwrap();
            assert!(gate.waiting, "no write reached the gate within 20 s");
        }

        fn lines(&self) -> Vec<String> {
            self.lines.lock().unwrap().clone()
        }
    }

    impl Write for Recorded {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            let (gate, wake) = &*self.gate;
            let mut held = gate.lock().unwrap();
            if held.shut {
                held.waiting = true;
                wake.notify_all();
                held = wake
                    .wait_timeout_while(held, Duration::from_secs(20), |g| g.shut)
                    .unwrap()
                    .0;
                held.waiting = false;
            }
            drop(held);
            if self.fail {
                return Err(io::Error::other("the sink refuses every write"));
            }
            self.lines
                .lock()
                .unwrap()
                .push(String::from_utf8_lossy(bytes).into_owned());
            Ok(bytes.len())
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    /// Hand `line` to `writer` as one event's bytes.
    fn log(writer: &LogWriter, line: &str) {
        let mut event = writer.make_writer();
        event.write_all(line.as_bytes()).unwrap();
    }

    const LONG: Duration = Duration::from_secs(20);

    #[test]
    fn a_line_fits_while_both_bounds_hold_and_is_refused_past_either() {
        let limits = Limits {
            lines: 2,
            bytes: 10,
        };
        assert!(
            admits(0, 0, 10, limits),
            "a line that fills the bytes exactly"
        );
        assert!(admits(1, 4, 6, limits));
        assert!(!admits(2, 0, 1, limits), "the line bound is reached");
        assert!(!admits(1, 5, 6, limits), "one byte past the byte bound");
        assert!(
            !admits(0, 0, 11, limits),
            "a line past the byte bound alone"
        );
        assert!(!admits(0, usize::MAX, 1, limits), "no overflow at the edge");
        assert!(admits(0, 0, 0, limits));
    }

    #[test]
    fn each_batch_of_drops_reports_what_the_regime_latch_decides() {
        let mut latch = FailureRegimeLatch::new();
        // Totals 1 to 3: the first opens the regime.
        assert_eq!(drop_report(&mut latch, 3), Report::Opened);
        // Totals 4 and 5: downgraded, 4 so far.
        assert_eq!(drop_report(&mut latch, 2), Report::Repeat { suppressed: 4 });
        // Totals 6 to 11: the 10th crosses a decade; the 11th does not undo it.
        assert_eq!(
            drop_report(&mut latch, 6),
            Report::StillDropping { suppressed: 8 }
        );
        assert_eq!(latch.total_failures(), 11);
        assert_eq!(latch.on_success(), Some(9));
        // A fresh regime is loud again, and one batch can open it and cross a
        // decade: opening wins.
        assert_eq!(drop_report(&mut latch, 100), Report::Opened);
        assert_eq!(latch.total_failures(), 111);
    }

    #[test]
    fn lines_are_written_in_the_order_they_were_handed_over() {
        for _run in 0..2 {
            let sink = Recorded::default();
            let writer = spawn_bare(Limits::DEFAULT, sink.clone());
            let expected: Vec<String> = (1..=10).map(|n| format!("line {n}\n")).collect();
            for line in &expected {
                log(&writer, line);
            }
            assert_eq!(writer.drain(LONG), Drained::Written);
            assert_eq!(sink.lines(), expected);
            assert_eq!(writer.dropped(), 0);
        }
    }

    #[test]
    fn lines_past_the_bounds_are_dropped_and_counted_while_a_write_blocks() {
        let sink = Recorded::default();
        sink.shut();
        let limits = Limits {
            lines: 2,
            bytes: 12,
        };
        let writer = spawn_bare(limits, sink.clone());
        log(&writer, "in flight\n");
        sink.until_a_write_waits();
        log(&writer, "queued 1\n");
        log(&writer, "dropped 1\n"); // 9 + 10 bytes: past the byte bound
        log(&writer, "q2\n");
        log(&writer, "dropped 2\n"); // past the line bound
        assert_eq!(writer.dropped(), 2);
        sink.open();
        assert_eq!(writer.drain(LONG), Drained::Written);
        let written: Vec<String> = sink
            .lines()
            .into_iter()
            .filter(|l| !l.contains("cerulion_remoted"))
            .collect();
        assert_eq!(written, ["in flight\n", "queued 1\n", "q2\n"]);
        assert_eq!(writer.dropped(), 2);
    }

    #[test]
    fn a_line_whose_write_fails_is_counted_as_dropped() {
        let sink = Recorded {
            fail: true,
            ..Recorded::default()
        };
        let writer = spawn_bare(Limits::DEFAULT, sink.clone());
        log(&writer, "a\n");
        log(&writer, "b\n");
        assert_eq!(writer.drain(LONG), Drained::Written);
        assert_eq!(writer.dropped(), 2);
        assert!(sink.lines().is_empty());
    }

    /// Keeps two `NoSubscriber` dispatchers alive for the rest of the
    /// process. Once both are registered, tracing-core's one-dispatcher path
    /// stays off: every later registration, of a callsite or of a dispatcher,
    /// computes and stores interest under the dispatchers' lock, over every
    /// live dispatcher. A callsite first reached on a thread with no
    /// subscriber is then cached as sometimes, not never, whenever a capture
    /// is live, so the capture is still asked whether its event is enabled.
    /// It must be called before a thread with no subscriber of its own first
    /// reaches a callsite a capture reads: `spawn_bare` calls it before it
    /// starts a writer thread, and `formatted_into` before it makes its
    /// subscriber, so before any thread its `f` starts.
    fn keep_two_dispatchers() {
        static KEPT: OnceLock<[tracing::Dispatch; 2]> = OnceLock::new();
        KEPT.get_or_init(|| {
            std::array::from_fn(|_| {
                tracing::Dispatch::new(tracing::subscriber::NoSubscriber::default())
            })
        });
    }

    /// A writer whose thread writes to `sink` with no subscriber of its own,
    /// started after `keep_two_dispatchers`.
    fn spawn_bare(limits: Limits, sink: Recorded) -> LogWriter {
        keep_two_dispatchers();
        spawn(limits, Box::new(sink)).unwrap()
    }

    /// `f` under a subscriber of this thread that formats into `writer`, with
    /// no time and no color, at debug, made after `keep_two_dispatchers`.
    fn formatted_into<R>(writer: &LogWriter, f: impl FnOnce() -> R) -> R {
        keep_two_dispatchers();
        let subscriber = tracing_subscriber::fmt()
            .with_writer(writer.clone())
            .with_ansi(false)
            .without_time()
            .with_target(false)
            .with_max_level(tracing::Level::DEBUG)
            .finish();
        tracing::subscriber::with_default(subscriber, f)
    }

    /// A writer whose thread writes to `sink` and formats its own report
    /// lines through a subscriber of that thread, so a test reads them.
    fn spawn_reporting(limits: Limits, mut sink: Recorded) -> LogWriter {
        let queue = Arc::new(Queue::new(limits));
        let thread_queue = queue.clone();
        // While the thread reports, REPORT takes its lines, so this queue
        // receives none of them.
        let format_into = unstarted(Limits::DEFAULT);
        std::thread::Builder::new()
            .name(THREAD_NAME.into())
            .spawn(move || {
                formatted_into(&format_into, || write_lines(&thread_queue, &mut sink));
            })
            .unwrap();
        LogWriter {
            inner: Arc::new(Inner { queue }),
        }
    }

    /// Set in the environment of the child process the test below runs alone
    /// in.
    const ALONE_ENV: &str = "CERULION_TEST_LOG_WRITER_ALONE";
    /// The full name of the test below, which the child runs with `--exact`.
    const ALONE_TEST: &str = "log_writer::tests::\
        a_callsite_first_reached_on_a_thread_with_no_subscriber_still_writes_its_line";

    /// Runs alone in a child process of this test binary, so no other test's
    /// dispatcher is live there: a thread with no subscriber reaches the
    /// callsite first, and the line is still written. The parent waits at
    /// most `LONG` for the child and kills it past that.
    #[test]
    fn a_callsite_first_reached_on_a_thread_with_no_subscriber_still_writes_its_line() {
        if std::env::var_os(ALONE_ENV).is_some() {
            let writer = unstarted(Limits::DEFAULT);
            let line =
                || tracing::warn!("cerulion_remoted: a line first reached with no subscriber");
            formatted_into(&writer, || {
                // A thread with no subscriber registers the callsite first.
                std::thread::spawn(line).join().unwrap();
                line();
            });
            let queued: Vec<Vec<u8>> = writer.inner.queue.lock().lines.iter().cloned().collect();
            assert_eq!(
                queued,
                [b" WARN cerulion_remoted: a line first reached with no subscriber\n".to_vec()],
                "the line was not written: its callsite was cached as never"
            );
            return;
        }
        /// The child, killed and reaped when dropped, so a panic here leaves
        /// no child running.
        struct Reap(std::process::Child);
        impl Drop for Reap {
            fn drop(&mut self) {
                let _ = self.0.kill();
                let _ = self.0.wait();
            }
        }
        let mut child = Reap(
            std::process::Command::new(std::env::current_exe().unwrap())
                .args(["--exact", ALONE_TEST, "--test-threads=1"])
                .env(ALONE_ENV, "1")
                .stdout(std::process::Stdio::piped())
                .stderr(std::process::Stdio::piped())
                .spawn()
                .unwrap(),
        );
        let until = Instant::now() + LONG;
        let status = loop {
            if let Some(status) = child.0.try_wait().unwrap() {
                break Some(status);
            }
            if Instant::now() >= until {
                child.0.kill().unwrap();
                child.0.wait().unwrap();
                break None;
            }
            std::thread::sleep(Duration::from_millis(5));
        };
        // The child has exited, so each pipe reads to its end.
        let (mut stdout, mut stderr) = (String::new(), String::new());
        io::Read::read_to_string(&mut child.0.stdout.take().unwrap(), &mut stdout).unwrap();
        io::Read::read_to_string(&mut child.0.stderr.take().unwrap(), &mut stderr).unwrap();
        let Some(status) = status else {
            panic!("the child ran past {LONG:?}\nstdout:\n{stdout}\nstderr:\n{stderr}");
        };
        assert!(
            status.success(),
            "the child failed with {status}\nstdout:\n{stdout}\nstderr:\n{stderr}"
        );
        assert!(
            stdout.contains("test result: ok. 1 passed"),
            "the child did not run exactly this test\nstdout:\n{stdout}\nstderr:\n{stderr}"
        );
    }

    #[test]
    fn a_drop_while_the_writer_waits_is_reported_with_no_further_line() {
        let sink = Recorded::default();
        let writer = spawn_reporting(Limits { lines: 4, bytes: 4 }, sink.clone());
        // One line written, then time for the writer thread to wait for the
        // next one.
        log(&writer, "ok\n");
        assert_eq!(writer.drain(LONG), Drained::Written);
        std::thread::sleep(Duration::from_millis(100));
        // Past the byte bound alone, into an empty queue.
        log(&writer, "too long\n");
        assert_eq!(writer.dropped(), 1);
        let until = Instant::now() + LONG;
        while sink.lines().len() < 2 {
            assert!(Instant::now() < until, "no drop report within {LONG:?}");
            std::thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(writer.drain(LONG), Drained::Written);
        // A regime of one line ends silently: no caught-up line.
        assert_eq!(
            sink.lines(),
            [
                "ok\n",
                " WARN cerulion_remoted: log lines were dropped: the log queue was full or a \
                 write to stderr failed dropped=1 total_dropped=1\n"
            ]
        );
    }

    #[test]
    fn a_drain_waits_for_the_drop_report_being_written() {
        let sink = Recorded::default();
        sink.shut();
        let writer = spawn_reporting(Limits { lines: 4, bytes: 4 }, sink.clone());
        log(&writer, "too long\n");
        // The writer thread holds its report at the shut gate.
        sink.until_a_write_waits();
        let grace = Duration::from_millis(200);
        assert_eq!(writer.drain(grace), Drained::Discarded { discarded: 0 });
        sink.open();
        assert_eq!(writer.drain(LONG), Drained::Written);
        assert_eq!(sink.lines().len(), 1, "{:?}", sink.lines());
    }

    #[test]
    fn a_drain_right_after_a_refused_line_returns_once_its_report_is_written() {
        let sink = Recorded::default();
        let writer = spawn_reporting(Limits { lines: 4, bytes: 4 }, sink.clone());
        // One line written, then time for the writer thread to wait for the
        // next one.
        log(&writer, "ok\n");
        assert_eq!(writer.drain(LONG), Drained::Written);
        std::thread::sleep(Duration::from_millis(100));
        // Refused into an empty queue, then drained at once, as an exit does.
        log(&writer, "too long\n");
        assert_eq!(writer.drain(LONG), Drained::Written);
        assert_eq!(
            sink.lines(),
            [
                "ok\n",
                " WARN cerulion_remoted: log lines were dropped: the log queue was full or a \
                 write to stderr failed dropped=1 total_dropped=1\n"
            ],
            "the drain returned before the drop was reported"
        );
    }

    #[test]
    fn a_drop_not_yet_reported_keeps_a_drain_waiting() {
        // No writer thread: the drop stays unreported for the whole grace.
        let writer = unstarted(Limits { lines: 4, bytes: 4 });
        log(&writer, "too long\n");
        assert_eq!(writer.dropped(), 1);
        let grace = Duration::from_millis(300);
        let since = Instant::now();
        assert_eq!(writer.drain(grace), Drained::Discarded { discarded: 0 });
        let took = since.elapsed();
        assert!(took >= grace, "returned after {took:?}, before its grace");
        assert!(
            took < grace + Duration::from_secs(5),
            "took {took:?}, past its grace {grace:?} plus 5 s"
        );
    }

    /// A sink that counts one more dropped line on `queue` at each write, as
    /// a logging thread whose line is refused while a report is written.
    struct DropsWhileWritten {
        queue: Arc<Queue>,
        lines: Vec<String>,
    }

    impl Write for DropsWhileWritten {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            self.queue.dropped.fetch_add(1, Ordering::AcqRel);
            self.lines.push(String::from_utf8_lossy(bytes).into_owned());
            Ok(bytes.len())
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn a_drop_while_its_report_is_written_keeps_the_regime_open() {
        let writer = unstarted(Limits::DEFAULT);
        let queue = writer.inner.queue.clone();
        let mut sink = DropsWhileWritten {
            queue: queue.clone(),
            lines: Vec::new(),
        };
        let mut reporter = Reporter::default();
        formatted_into(&writer, || {
            queue.dropped.store(3, Ordering::Release);
            reporter.after_write(&queue, true, &mut sink);
        });
        // The fourth drop came while the warn line was written: no caught-up
        // line for a regime that is still dropping.
        assert_eq!(
            sink.lines,
            [
                " WARN cerulion_remoted: log lines were dropped: the log queue was full or a \
              write to stderr failed dropped=3 total_dropped=3\n"
            ]
        );
        assert_eq!(queue.dropped.load(Ordering::Acquire), 4);
    }

    #[cfg(feature = "test-seam")]
    #[test]
    fn a_test_bound_set_later_applies_to_the_lines_after_it() {
        let writer = unstarted(Limits::DEFAULT);
        let queue = &writer.inner.queue;
        log(&writer, "before 1\n");
        log(&writer, "before 2\n");
        queue.test_lines.store(1, Ordering::Release);
        log(&writer, "after\n");
        assert_eq!(writer.dropped(), 1, "the bound applies from its store on");
        assert_eq!(queue.lock().lines.len(), 2, "lines queued before it stay");
    }

    #[cfg(feature = "test-seam")]
    #[test]
    fn the_test_seam_only_shrinks_the_queue_and_keeps_room_for_a_line() {
        assert_eq!(test_queue_lines("1"), Some(1));
        assert_eq!(test_queue_lines(" 8 "), Some(8));
        assert_eq!(test_queue_lines("4096"), Some(4096));
        assert_eq!(test_queue_lines("4097"), Some(4096));
        assert_eq!(test_queue_lines("1000000"), Some(4096));
        for refused in ["0", "-1", "", "x", "1.5", "18446744073709551616"] {
            assert_eq!(test_queue_lines(refused), None, "{refused}");
        }
    }

    #[test]
    fn drops_are_reported_once_written_and_the_regime_ends_when_the_queue_empties() {
        let writer = unstarted(Limits::DEFAULT);
        let queue = &writer.inner.queue;
        let mut sink = Recorded::default();
        let mut reporter = Reporter::default();
        formatted_into(&writer, || {
            queue.dropped.store(3, Ordering::Release);
            assert!(queue.owes_report());
            reporter.after_write(queue, false, &mut sink);
            assert!(!queue.owes_report());
            // No new drop, lines still queued: the regime stays open.
            reporter.after_write(queue, false, &mut sink);
            queue.dropped.store(5, Ordering::Release);
            reporter.after_write(queue, false, &mut sink);
            // The 10th drop crosses a decade; the line names the counter's 11.
            queue.dropped.store(11, Ordering::Release);
            reporter.after_write(queue, false, &mut sink);
            reporter.after_write(queue, true, &mut sink);
            // Healthy and empty: nothing more.
            reporter.after_write(queue, true, &mut sink);
            // Drops reported by a write that leaves the queue empty: the
            // regime opens and ends at once.
            queue.dropped.store(15, Ordering::Release);
            reporter.after_write(queue, true, &mut sink);
        });
        const DROPPED: &str = "cerulion_remoted: log lines were dropped: the log queue was full \
                               or a write to stderr failed";
        const CAUGHT_UP: &str = "cerulion_remoted: the log writer caught up; the lines it \
                                 dropped are counted in the wire status reply";
        assert_eq!(
            sink.lines(),
            [
                format!(" WARN {DROPPED} dropped=3 total_dropped=3\n"),
                format!("DEBUG {DROPPED} dropped=2 total_dropped=5 suppressed=4\n"),
                format!(" WARN {DROPPED} dropped=6 total_dropped=11 suppressed=8\n"),
                format!(" INFO {CAUGHT_UP} dropped=11 total_dropped=11\n"),
                format!(" WARN {DROPPED} dropped=4 total_dropped=15\n"),
                format!(" INFO {CAUGHT_UP} dropped=4 total_dropped=15\n"),
            ]
        );
        assert!(
            queue.lock().lines.is_empty(),
            "a report line is written by the reporter, never queued"
        );
    }

    #[test]
    fn a_regime_of_one_dropped_line_ends_silently() {
        let writer = unstarted(Limits::DEFAULT);
        let queue = &writer.inner.queue;
        let mut sink = Recorded::default();
        let mut reporter = Reporter::default();
        formatted_into(&writer, || {
            queue.dropped.store(1, Ordering::Release);
            reporter.after_write(queue, true, &mut sink);
            reporter.after_write(queue, true, &mut sink);
            queue.dropped.store(2, Ordering::Release);
            reporter.after_write(queue, true, &mut sink);
        });
        assert_eq!(
            sink.lines(),
            [
                " WARN cerulion_remoted: log lines were dropped: the log queue was full or a \
                 write to stderr failed dropped=1 total_dropped=1\n",
                " WARN cerulion_remoted: log lines were dropped: the log queue was full or a \
                 write to stderr failed dropped=1 total_dropped=2\n",
            ]
        );
    }

    #[test]
    fn the_drain_returns_at_once_when_nothing_is_queued() {
        let writer = spawn_bare(Limits::DEFAULT, Recorded::default());
        let since = Instant::now();
        assert_eq!(writer.drain(LONG), Drained::Written);
        assert!(since.elapsed() < Duration::from_secs(5));
    }

    #[test]
    fn a_drain_behind_a_blocked_write_discards_the_queue_and_queues_its_count() {
        let sink = Recorded::default();
        sink.shut();
        let writer = spawn_bare(Limits::DEFAULT, sink.clone());
        log(&writer, "in flight\n");
        sink.until_a_write_waits();
        let queued: Vec<String> = (1..=10).map(|n| format!("queued {n}\n")).collect();
        for line in &queued {
            log(&writer, line);
        }
        let grace = Duration::from_millis(500);
        let since = Instant::now();
        let drained = formatted_into(&writer, || writer.drain(grace));
        let took = since.elapsed();
        assert_eq!(drained, Drained::Discarded { discarded: 2 });
        assert!(took >= grace, "returned after {took:?}, before its grace");
        assert!(took < grace + Duration::from_secs(5), "took {took:?}");
        sink.open();
        assert_eq!(writer.drain(LONG), Drained::Written);
        // The two oldest are discarded; the newest eight and then the count
        // are written.
        let mut expected = vec!["in flight\n".to_string()];
        expected.extend(queued[2..].iter().cloned());
        expected.push(
            "ERROR cerulion_remoted: log lines still queued at the exit were discarded \
             discarded=2\n"
                .to_string(),
        );
        assert_eq!(sink.lines(), expected);
        assert_eq!(writer.dropped(), 0);
    }
}
