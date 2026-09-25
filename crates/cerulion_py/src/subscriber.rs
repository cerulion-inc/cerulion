// SPDX-License-Identifier: AGPL-3.0-only
//! `Subscriber` - owned zero-copy frame reads over a `CerulionSubscriber`.
//!
//! `receive()` blocks by `poll(2)`ing the subscriber's EXISTING event-
//! listener fd (no second wake source), in ≤100 ms slices so Python
//! signals stay responsive; the GIL is released only around the poll -
//! the `CerulionSubscriber` itself never leaves the attached thread.

use crate::errors::{map_transport_err, TransportError as PyTransportError};
use crate::frame::Frame;
use cerulion_core::clock::real_ns;
use cerulion_core::transport::failure_regime_latch::{
    lock_regime_latch, FailureRegimeLatch, RegimeDecision,
};
use cerulion_core::{CerulionSubscriber, TransportManager};
use pyo3::exceptions::PyRuntimeWarning;
use pyo3::prelude::*;
use std::ffi::CString;
use std::os::fd::RawFd;
use std::sync::Mutex;
use std::time::{Duration, Instant};

/// A subscriber on one topic with an iceoryx2 queue `depth` frames deep
/// (the core rejects 0 and >16).
#[pyclass(unsendable)]
pub struct Subscriber {
    sub: CerulionSubscriber,
    depth: usize,
    /// Flood suppression for `drain_event_notifications` failures -
    /// one per observing entity and condition (the shared latch's
    /// keying rule); `receive` is `&self`, so it lives behind a Mutex.
    drain_latch: Mutex<FailureRegimeLatch>,
}

impl Subscriber {
    /// `poll(2)` timeout for `slice`: whole milliseconds, floored at 1 so a
    /// sub-millisecond remainder blocks instead of degrading to a zero-timeout
    /// spin, capped at `i32::MAX`.
    fn poll_timeout_ms(slice: Duration) -> i32 {
        slice.as_millis().clamp(1, i32::MAX as u128) as i32
    }

    /// Pop one frame, mapping the sample into a `Frame`. Shared by
    /// `try_receive` and the `receive` loop.
    fn receive_one(&self) -> PyResult<Option<Frame>> {
        let res = self
            .sub
            .try_receive_one_owned()
            .map_err(map_transport_err)?;
        match res {
            Some(sample) => {
                // Capture receive time BEFORE anything else - the value
                // is observability, and the header parse below must not
                // skew it.
                let recv_ns = real_ns();
                let header = sample.wire_header().ok_or_else(|| {
                    PyTransportError::new_err("received a frame with a malformed wire header")
                })?;
                Ok(Some(Frame::new(sample, header, recv_ns)))
            }
            None => Ok(None),
        }
    }

    /// `poll(2)` the listener fd for up to `slice`. EINTR counts as an
    /// elapsed slice (the loop recomputes the deadline); any other errno
    /// surfaces as `TransportError` with the errno text.
    fn poll_fd(fd: RawFd, slice: Duration) -> PyResult<()> {
        let timeout_ms = Self::poll_timeout_ms(slice);
        let mut pfd = libc::pollfd {
            fd,
            events: libc::POLLIN,
            revents: 0,
        };
        // SAFETY: `pfd` is a valid pointer to one initialised pollfd and
        // nfds = 1; `fd` is a non-owning snapshot of the subscriber's
        // listener, alive for the whole call because `self` is.
        let rc = unsafe { libc::poll(&mut pfd, 1, timeout_ms) };
        if rc < 0 {
            let err = std::io::Error::last_os_error();
            if err.kind() != std::io::ErrorKind::Interrupted {
                return Err(PyTransportError::new_err(format!(
                    "event listener poll failed: {err}"
                )));
            }
        }
        Ok(())
    }
}

#[pymethods]
impl Subscriber {
    /// `Subscriber(topic, depth)` - the core validates `depth`
    /// (0 and >16 rejected → `TransportError`).
    #[new]
    fn new(topic: &str, depth: usize) -> PyResult<Self> {
        let mgr = TransportManager::get().map_err(map_transport_err)?;
        let sub = mgr
            .create_subscriber_with_buffers(topic, mgr.default_topic_config(), depth)
            .map_err(map_transport_err)?;
        Ok(Self {
            sub,
            depth,
            drain_latch: Mutex::new(FailureRegimeLatch::new()),
        })
    }

    /// Topic name this subscriber is bound to.
    #[getter]
    fn topic(&self) -> &str {
        self.sub.topic()
    }

    /// Receive-queue depth this subscriber was created with.
    #[getter]
    fn depth(&self) -> usize {
        self.depth
    }

    /// The service's `subscriber_max_borrowed_samples` budget - how many
    /// frames may be held (unreleased) at once before the next receive
    /// fails with `BorrowLimitExceeded`.
    #[getter]
    fn max_borrowed_samples(&self) -> usize {
        self.sub.max_borrowed_samples()
    }

    /// Pop at most one queued frame without blocking.
    fn try_receive(&self) -> PyResult<Option<Frame>> {
        self.receive_one()
    }

    /// Pop the next frame, blocking up to `timeout_ms` (None = forever,
    /// 0 = exactly one non-blocking try). Releases the GIL while parked.
    ///
    /// A `timeout_ms` whose deadline is unrepresentable on this clock
    /// (`Instant::now() + ms` overflows, e.g. `2**64-1`) is treated as
    /// unbounded - the wait then behaves like `receive(None)`. Each wait
    /// slice is rounded up to at least one millisecond, so a finite deadline
    /// may be overshot by less than one millisecond.
    ///
    /// # Errors
    ///
    /// `poll(2)` failures other than EINTR raise `TransportError`. The
    /// FIRST `drain_event_notifications()` failure emits a
    /// `RuntimeWarning` carrying the error text and marks the wake fd
    /// degraded - per the core's caller contract it only means the fd
    /// may stay readable with nobody able to drain it, so the rest of
    /// this call parks on bounded sleeps instead of polling that fd
    /// (a readable undrainable fd would spin forever otherwise). The
    /// frame probe is unaffected: queued samples are still found on
    /// every iteration. Warning emission itself can raise - under a
    /// `-W error` filter the `RuntimeWarning` surfaces as an exception.
    #[pyo3(signature = (timeout_ms=None))]
    fn receive(&self, py: Python<'_>, timeout_ms: Option<u64>) -> PyResult<Option<Frame>> {
        if timeout_ms == Some(0) {
            return self.receive_one();
        }
        // Unrepresentable deadlines degrade to unbounded rather than
        // wrapping to a far-past instant.
        let deadline =
            timeout_ms.and_then(|ms| Instant::now().checked_add(Duration::from_millis(ms)));
        // Set when `drain_event_notifications` fails once: the fd can no
        // longer be trusted drainable, so parking falls back to sleeping
        // the same bounded slice for the rest of this call.
        let mut fd_degraded = false;
        loop {
            // 1. A queued frame wins over the wait - and the drain below
            //    never precedes this try (a notify can describe a sample
            //    already taken; only data answers it).
            if let Some(frame) = self.receive_one()? {
                return Ok(Some(frame));
            }
            // 2. Deadline spent - report the miss.
            let slice = match deadline {
                Some(d) => {
                    let remaining = d.saturating_duration_since(Instant::now());
                    if remaining.is_zero() {
                        return Ok(None);
                    }
                    remaining.min(Duration::from_millis(100))
                }
                // Unbounded wait still slices at 100 ms so signals land.
                None => Duration::from_millis(100),
            };
            // 3. Park with the GIL off - on the subscriber's own event
            //    fd while it is drainable, else on a bounded sleep of
            //    the same slice (timeout pacing).
            if fd_degraded {
                py.detach(|| std::thread::sleep(slice));
            } else {
                let fd = self.sub.event_listener_fd();
                py.detach(|| Self::poll_fd(fd, slice))?;
            }
            // 4. Let KeyboardInterrupt propagate.
            py.check_signals()?;
            // 5. Drain the wake's notifications AFTER the poll fired -
            //    never before step 1's try, and never left to re-fire a
            //    level-triggered fd forever. The first drain error marks
            //    the fd degraded for the rest of this call (see # Errors).
            if !fd_degraded {
                match self.sub.drain_event_notifications() {
                    Err(e) => {
                        fd_degraded = true;
                        let decision = lock_regime_latch(&self.drain_latch).on_failure();
                        match decision {
                            // First-of-regime and each decade boundary
                            // warn; repeats are counted, not repeated.
                            RegimeDecision::Suppressed { suppressed } => {
                                tracing::debug!(
                                    suppressed,
                                    "event-notification drain still failing"
                                );
                            }
                            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                                let msg = CString::new(format!(
                                    "cerulion subscriber event-notification drain failed \
                                     ({e}); parking on sleeps for the rest of this receive()"
                                ))
                                .unwrap_or_else(|_| {
                                    c"cerulion subscriber event-notification drain failed"
                                        .to_owned()
                                });
                                PyErr::warn(py, &py.get_type::<PyRuntimeWarning>(), &msg, 1)?;
                            }
                        }
                    }
                    Ok(()) => {
                        if let Some(recovered) = lock_regime_latch(&self.drain_latch).on_success() {
                            tracing::info!(recovered, "event-notification drain recovered");
                        }
                    }
                }
            }
        }
    }
}
