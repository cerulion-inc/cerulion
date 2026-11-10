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
//! dropped line being one failure. When a write returns and lines were dropped
//! since the last one, it writes one warn line naming how many (`dropped`) and
//! the total (`total_dropped`) when the drops open a regime or take the total
//! across a power of ten, and a debug line otherwise. When the queue is empty
//! again with no drop since, it writes one info line saying it caught up, with
//! the regime's `dropped` and the `total_dropped`; a regime of a single dropped
//! line ends silently, as the latch decides. These lines are formatted on the
//! writer thread and written straight to stderr, never queued. A report line
//! whose own write fails is not counted.
//!
//! The logging threads and the writer thread share one lock, the queue's, held
//! to push or pop one line and never across a format, a copy or a write.
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

/// The environment variable that sets, in a build with the test seam, the line
/// bound of the binary's log queue in place of [`QUEUE_LINES`]; a value that is
/// not a positive whole number keeps it.
#[cfg(feature = "test-seam")]
#[doc(hidden)]
pub const QUEUE_LINES_FOR_TEST_ENV: &str = "CERULION_REMOTED_LOG_QUEUE_LINES_FOR_TEST";

/// The name of the writer thread.
const THREAD_NAME: &str = "remoted-log";

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
/// writers hand each formatted line to the queue. Clones share one queue; the
/// writer thread ends once every clone is dropped and the queue is empty.
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
    limits: Limits,
}

#[derive(Default)]
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
            state: Mutex::new(State::default()),
            ready: Condvar::new(),
            idle: Condvar::new(),
            dropped: AtomicU64::new(0),
            limits,
        }
    }

    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Queue `line`, or drop and count it when it does not fit ([`admits`]).
    fn hand_over(&self, line: Vec<u8>) {
        let mut state = self.lock();
        if admits(state.lines.len(), state.bytes, line.len(), self.limits) {
            state.bytes += line.len();
            state.lines.push_back(line);
            drop(state);
            self.ready.notify_one();
        } else {
            drop(state);
            self.dropped.fetch_add(1, Ordering::AcqRel);
        }
    }

    /// Wait until no line is queued and none is being written, for at most
    /// what is left of `budget` since `since`. Returns whether that happened.
    fn wait_idle(&self, since: Instant, budget: Duration) -> bool {
        let mut state = self.lock();
        loop {
            if state.lines.is_empty() && !state.writing {
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

/// The writer thread: write each queued line to `sink` in order, then report
/// drops ([`Reporter::after_write`]); return once the queue is closed and
/// empty.
fn write_lines(queue: &Queue, sink: &mut dyn Write) {
    let mut reporter = Reporter::default();
    loop {
        let line = {
            let mut state = queue.lock();
            loop {
                if let Some(line) = state.lines.pop_front() {
                    state.bytes -= line.len();
                    state.writing = true;
                    break line;
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
        if sink.write_all(&line).is_err() {
            queue.dropped.fetch_add(1, Ordering::AcqRel);
        }
        drop(line);
        let empty = {
            let mut state = queue.lock();
            state.writing = false;
            state.lines.is_empty()
        };
        queue.idle.notify_all();
        reporter.after_write(queue, empty, sink);
    }
}

impl LogWriter {
    /// Lines dropped since this writer started.
    pub fn dropped(&self) -> u64 {
        self.inner.queue.dropped.load(Ordering::Acquire)
    }

    /// Wait for the queue to be written, for at most `grace`: until no line is
    /// queued and none is being written, for at most four fifths of `grace`.
    /// Lines still queued then are discarded and counted, one error line
    /// naming the count (`discarded`) is queued, and the wait goes on for the
    /// rest of `grace`. A write blocked in stderr holds the writer thread, not
    /// this wait.
    pub fn drain(&self, grace: Duration) -> Drained {
        let since = Instant::now();
        let queue = &self.inner.queue;
        if queue.wait_idle(since, grace / 5 * 4) {
            return Drained::Written;
        }
        let discarded = {
            let mut state = queue.lock();
            let discarded = state.lines.len();
            state.lines.clear();
            state.bytes = 0;
            discarded
        };
        let discarded = u64::try_from(discarded).unwrap_or(u64::MAX);
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
    /// No line was queued and none was being written.
    Written,
    /// At four fifths of the grace, a write had not returned or lines were
    /// still queued; `discarded` queued lines were discarded.
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
    /// The batch took the total across a power of ten: warn, with the latch's
    /// total and the failures it downgraded.
    StillDropping { total: u64, suppressed: u64 },
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
            (_, RegimeDecision::StillFailing { total, suppressed }) => {
                Report::StillDropping { total, suppressed }
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
    /// The drop count last reported.
    reported: u64,
    /// The lines dropped in the open regime.
    regime: u64,
}

impl Reporter {
    /// After a write: report the lines dropped since the last report, or,
    /// with none and the queue `empty`, the end of an open regime.
    fn after_write(&mut self, queue: &Queue, empty: bool, sink: &mut dyn Write) {
        let total = queue.dropped.load(Ordering::Acquire);
        let dropped = total.saturating_sub(self.reported);
        if dropped > 0 {
            self.reported = total;
            self.regime = self.regime.saturating_add(dropped);
            let report = drop_report(&mut self.latch, dropped);
            write_report(sink, || match report {
                Report::Opened => tracing::warn!(
                    dropped,
                    total_dropped = total,
                    "cerulion_remoted: log lines were dropped: the log queue was full or a \
                     write to stderr failed"
                ),
                Report::StillDropping { total, suppressed } => tracing::warn!(
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
        } else if empty && self.latch.is_failing() {
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
            Report::StillDropping {
                total: 10,
                suppressed: 8
            }
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
            let writer = spawn(Limits::DEFAULT, Box::new(sink.clone())).unwrap();
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
        let writer = spawn(limits, Box::new(sink.clone())).unwrap();
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
        let writer = spawn(Limits::DEFAULT, Box::new(sink.clone())).unwrap();
        log(&writer, "a\n");
        log(&writer, "b\n");
        assert_eq!(writer.drain(LONG), Drained::Written);
        assert_eq!(writer.dropped(), 2);
        assert!(sink.lines().is_empty());
    }

    /// `f` under a subscriber of this thread that formats into `writer`, with
    /// no time and no color, at debug.
    fn formatted_into<R>(writer: &LogWriter, f: impl FnOnce() -> R) -> R {
        let subscriber = tracing_subscriber::fmt()
            .with_writer(writer.clone())
            .with_ansi(false)
            .without_time()
            .with_target(false)
            .with_max_level(tracing::Level::DEBUG)
            .finish();
        tracing::subscriber::with_default(subscriber, f)
    }

    #[test]
    fn drops_are_reported_once_written_and_the_regime_ends_when_the_queue_empties() {
        let writer = unstarted(Limits::DEFAULT);
        let queue = &writer.inner.queue;
        let mut sink = Recorded::default();
        let mut reporter = Reporter::default();
        formatted_into(&writer, || {
            queue.dropped.store(3, Ordering::Release);
            reporter.after_write(queue, false, &mut sink);
            // No new drop, lines still queued: the regime stays open.
            reporter.after_write(queue, false, &mut sink);
            queue.dropped.store(5, Ordering::Release);
            reporter.after_write(queue, false, &mut sink);
            reporter.after_write(queue, true, &mut sink);
            // Healthy and empty: nothing more.
            reporter.after_write(queue, true, &mut sink);
        });
        assert_eq!(
            sink.lines(),
            [
                " WARN cerulion_remoted: log lines were dropped: the log queue was full or a \
                 write to stderr failed dropped=3 total_dropped=3\n",
                "DEBUG cerulion_remoted: log lines were dropped: the log queue was full or a \
                 write to stderr failed dropped=2 total_dropped=5 suppressed=4\n",
                " INFO cerulion_remoted: the log writer caught up; the lines it dropped are \
                 counted in the wire status reply dropped=5 total_dropped=5\n",
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
        let writer = spawn(Limits::DEFAULT, Box::new(Recorded::default())).unwrap();
        let since = Instant::now();
        assert_eq!(writer.drain(LONG), Drained::Written);
        assert!(since.elapsed() < Duration::from_secs(5));
    }

    #[test]
    fn a_drain_behind_a_blocked_write_discards_the_queue_and_queues_its_count() {
        let sink = Recorded::default();
        sink.shut();
        let writer = spawn(Limits::DEFAULT, Box::new(sink.clone())).unwrap();
        log(&writer, "in flight\n");
        sink.until_a_write_waits();
        log(&writer, "queued 1\n");
        log(&writer, "queued 2\n");
        let grace = Duration::from_millis(500);
        let since = Instant::now();
        let drained = formatted_into(&writer, || writer.drain(grace));
        let took = since.elapsed();
        assert_eq!(drained, Drained::Discarded { discarded: 2 });
        assert!(took >= grace, "returned after {took:?}, before its grace");
        assert!(took < grace + Duration::from_secs(5), "took {took:?}");
        sink.open();
        assert_eq!(writer.drain(LONG), Drained::Written);
        assert_eq!(
            sink.lines(),
            [
                "in flight\n",
                "ERROR cerulion_remoted: log lines still queued at the exit were discarded \
                 discarded=2\n",
            ]
        );
        assert_eq!(writer.dropped(), 0);
    }
}
