// SPDX-License-Identifier: AGPL-3.0-only
//! `Frame` - an owned inbound sample with a READ-ONLY buffer export.
//!
//! The frame owns the `OwnedInboundSample`, so the SHM slot stays pinned
//! (borrowed, never overwritten) as long as the frame - or any buffer
//! exported from it - is alive. `release()` returns the slot early; with
//! live views the sample is parked with its subscriber and dropped at the
//! last `__releasebuffer__` on the owning thread, or at the subscriber's
//! next receive when that last view closed elsewhere (a memoryview/ndarray
//! keeps reading VALID pinned memory - nothing can invalidate a CPython
//! memoryview from the exporter side).

use crate::errors::{ReleasedFrame, TransportError as PyTransportError};
use cerulion_core::transport::subscriber::OwnedInboundSample;
use cerulion_core::WireHeader;
use pyo3::exceptions::{PyBufferError, PyRuntimeWarning};
use pyo3::ffi;
use pyo3::prelude::*;
use std::cell::{Cell, RefCell};
use std::ffi::CString;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;

static NEXT_THREAD_TOKEN: AtomicUsize = AtomicUsize::new(1);
thread_local! {
    static THREAD_TOKEN: Cell<usize> = const { Cell::new(0) };
}

/// Process-unique, never-reused token for the calling thread, assigned on
/// first use. No allocation: a const-initialised thread-local plus one
/// atomic counter (0 is "unassigned").
pub(crate) fn thread_token() -> usize {
    THREAD_TOKEN.with(|t| {
        if t.get() == 0 {
            t.set(NEXT_THREAD_TOKEN.fetch_add(1, Ordering::Relaxed));
        }
        t.get()
    })
}

/// Live buffer-export count shared by an exporter and its `Py_buffer`s.
///
/// Each export stores one `Arc` reference in `view.internal`
/// (exporter-private per the buffer protocol), so `__releasebuffer__` can
/// decrement the count on ANY thread without borrowing the unsendable
/// pyclass. `owner` is the exporter's creating thread: only there may the
/// SHM slot itself be dropped.
pub(crate) struct Exports {
    owner: usize,
    live: AtomicUsize,
    given_up: AtomicBool,
}

impl Exports {
    pub(crate) fn new() -> Arc<Self> {
        Arc::new(Self {
            owner: thread_token(),
            live: AtomicUsize::new(0),
            given_up: AtomicBool::new(false),
        })
    }

    /// Exports not yet released, on any thread.
    pub(crate) fn live(&self) -> usize {
        self.live.load(Ordering::Acquire)
    }

    /// Mark the slot as given up (released / discarded) by its owner.
    pub(crate) fn give_up(&self) {
        self.given_up.store(true, Ordering::Release);
    }

    /// Reuse an unshared counter for a new exporter on the same thread.
    fn reset(&mut self) {
        *self.live.get_mut() = 0;
        *self.given_up.get_mut() = false;
    }
}

/// Slot bookkeeping a `Subscriber` shares with every `Frame` it hands out.
///
/// `parked` holds samples of released frames whose views are still open,
/// so the slot returns as soon as the last view closes even while the
/// `Frame` object stays referenced. `pool` recycles export counters of
/// dropped frames, so a steady receive loop allocates none.
pub(crate) struct SlotHome {
    parked: Vec<(Arc<Exports>, OwnedInboundSample)>,
    pool: Vec<Arc<Exports>>,
}

impl SlotHome {
    /// Capacity for `borrowed` simultaneously held frames.
    pub(crate) fn shared(borrowed: usize) -> Rc<RefCell<Self>> {
        let capacity = borrowed.max(1);
        Rc::new(RefCell::new(Self {
            parked: Vec::with_capacity(capacity),
            pool: Vec::with_capacity(capacity),
        }))
    }

    /// Drop every parked sample whose views have all closed.
    pub(crate) fn reap(&mut self) {
        self.parked.retain(|(exports, _)| exports.live() > 0);
    }

    fn take_exports(&mut self) -> Arc<Exports> {
        if let Some(mut exports) = self.pool.pop() {
            if let Some(counter) = Arc::get_mut(&mut exports) {
                counter.reset();
                return exports;
            }
        }
        Exports::new()
    }
}

/// What `release_export` observed for one released export.
pub(crate) struct ExportRelease {
    pub(crate) on_owner: bool,
    pub(crate) live: usize,
    pub(crate) given_up: bool,
}

/// Count one export and store its reference in `view.internal`.
///
/// SAFETY: caller passes the `Py_buffer` the interpreter handed to
/// `__getbuffer__`, after a successful `PyBuffer_FillInfo`.
pub(crate) unsafe fn stamp_export(view: *mut ffi::Py_buffer, exports: &Arc<Exports>) {
    exports.live.fetch_add(1, Ordering::AcqRel);
    let raw = Arc::into_raw(Arc::clone(exports));
    unsafe { (*view).internal = raw.cast_mut().cast() };
}

/// Uncount the export stored in `view.internal`; `None` for a view this
/// exporter never stamped.
///
/// SAFETY: caller passes the `Py_buffer` the interpreter hands to
/// `__releasebuffer__` - the same struct `stamp_export` stamped, released
/// exactly once.
pub(crate) unsafe fn release_export(view: *mut ffi::Py_buffer) -> Option<ExportRelease> {
    let raw = unsafe { (*view).internal } as *const Exports;
    if raw.is_null() {
        return None;
    }
    unsafe { (*view).internal = std::ptr::null_mut() };
    // SAFETY: `raw` came from `Arc::into_raw` in `stamp_export` and is
    // consumed once, here.
    let exports = unsafe { Arc::from_raw(raw) };
    let live = exports
        .live
        .fetch_sub(1, Ordering::AcqRel)
        .saturating_sub(1);
    Some(ExportRelease {
        on_owner: exports.owner == thread_token(),
        live,
        given_up: exports.given_up.load(Ordering::Acquire),
    })
}

/// Emit the `RuntimeWarning` for the last `__releasebuffer__` of a given-up
/// slot that ran off its object's owning thread (or could not borrow it);
/// `returns` says when the slot comes back. `releasebuffer` cannot raise, so
/// the `Result` is ignored by callers.
pub(crate) fn warn_offthread_release(py: Python<'_>, class: &str, returns: &str) {
    let msg = CString::new(format!(
        "cerulion {class} buffer released off the owning thread after the slot was \
         given up; the shared-memory slot returns {returns}"
    ))
    .unwrap_or_else(|_| c"cerulion buffer released off the owning thread".to_owned());
    let _ = PyErr::warn(py, &py.get_type::<PyRuntimeWarning>(), &msg, 1);
}

/// One received wire frame (32-byte header + body).
///
/// A memoryview released on a foreign thread still uncounts its export;
/// if that was the last view of a released frame, the slot returns at the
/// subscriber's next receive.
///
/// Limitation (pyo3 `unsendable`): a `Frame` whose LAST Python reference
/// dies on a foreign thread is never dropped - pyo3's `can_drop` refuses,
/// writes an unraisable `PyRuntimeError` to stderr, and the value (and
/// its borrowed SHM slot) leaks until process exit.
#[pyclass(unsendable)]
pub struct Frame {
    sample: Option<OwnedInboundSample>,
    header: WireHeader,
    recv_ns: u64,
    exports: Arc<Exports>,
    home: Rc<RefCell<SlotHome>>,
    released: bool,
}

impl Frame {
    pub(crate) fn new(
        sample: OwnedInboundSample,
        header: WireHeader,
        recv_ns: u64,
        home: &Rc<RefCell<SlotHome>>,
    ) -> Self {
        let exports = home.borrow_mut().take_exports();
        Self {
            sample: Some(sample),
            header,
            recv_ns,
            exports,
            home: Rc::clone(home),
            released: false,
        }
    }

    /// Once released, drop the sample if no view is live, else park it
    /// with the subscriber until its last view closes (owner thread).
    fn reap(&mut self) {
        if !self.released {
            return;
        }
        let mut home = self.home.borrow_mut();
        if let Some(sample) = self.sample.take() {
            if self.exports.live() > 0 {
                home.parked.push((Arc::clone(&self.exports), sample));
            }
        }
        home.reap();
    }
}

impl Drop for Frame {
    fn drop(&mut self) {
        self.sample = None;
        if let Ok(mut home) = self.home.try_borrow_mut() {
            home.reap();
            if Arc::strong_count(&self.exports) == 1 && home.pool.len() < home.pool.capacity() {
                home.pool.push(Arc::clone(&self.exports));
            }
        }
    }
}

impl Frame {
    fn check_live(&self) -> PyResult<()> {
        if self.released {
            return Err(ReleasedFrame::new_err("frame already released"));
        }
        Ok(())
    }
}

#[pymethods]
impl Frame {
    /// Export the WHOLE frame `sample.payload()` (header + body,
    /// trailing slot capacity excluded by the core) as a READ-ONLY
    /// contiguous buffer - the subscriber-side SHM mapping is `r--s`,
    /// so a writable view would segfault.
    ///
    /// SAFETY: `view` is a valid `*mut ffi::Py_buffer` the interpreter
    /// handed us to fill; `PyBuffer_FillInfo` writes it and (on success)
    /// increfs `slf` into `view->obj`, so `self` - and therefore the
    /// `OwnedInboundSample` pinning the SHM slot - outlives the export.
    /// The `exports` counter additionally defers an early `release()`
    /// until the matching `__releasebuffer__`.
    unsafe fn __getbuffer__(
        slf: Bound<'_, Self>,
        view: *mut ffi::Py_buffer,
        flags: std::ffi::c_int,
    ) -> PyResult<()> {
        let this = slf.borrow();
        if this.released {
            return Err(ReleasedFrame::new_err("frame already released"));
        }
        let Some(sample) = this.sample.as_ref() else {
            return Err(PyTransportError::new_err("frame holds no sample"));
        };
        let payload = sample.payload();
        let rc = unsafe {
            ffi::PyBuffer_FillInfo(
                view,
                slf.as_ptr(),
                payload.as_ptr() as *mut std::ffi::c_void,
                payload.len() as ffi::Py_ssize_t,
                1,
                flags,
            )
        };
        if rc != 0 {
            return Err(PyErr::take(slf.py())
                .unwrap_or_else(|| PyBufferError::new_err("frame buffer export failed")));
        }
        unsafe { stamp_export(view, &this.exports) };
        Ok(())
    }

    /// SAFETY: called by the interpreter once per live export produced by
    /// `__getbuffer__`; `view` is the same pointer it filled there.
    ///
    /// CPython may run this on a FOREIGN thread (a memoryview handed to
    /// another `threading.Thread` releases there). The export count is
    /// atomic, so it is always decremented; only the owning thread borrows
    /// the unsendable pyclass to drop a released sample. Off the owner, a
    /// released frame's last export leaves the parked sample to the
    /// subscriber's next receive, with a `RuntimeWarning`.
    unsafe fn __releasebuffer__(slf: Bound<'_, Self>, view: *mut ffi::Py_buffer) {
        let Some(done) = (unsafe { release_export(view) }) else {
            return;
        };
        if done.live > 0 || !done.given_up {
            return;
        }
        if done.on_owner {
            if let Ok(mut this) = slf.try_borrow_mut() {
                this.reap();
                return;
            }
        }
        warn_offthread_release(slf.py(), "Frame", "at the subscriber's next receive");
    }

    /// Wire `schema_hash` field.
    #[getter]
    fn schema_hash(&self) -> PyResult<u64> {
        self.check_live()?;
        Ok(self.header.schema_hash)
    }

    /// Wire `sequence` field (per-publisher commit counter).
    #[getter]
    fn sequence(&self) -> PyResult<u32> {
        self.check_live()?;
        Ok(self.header.sequence)
    }

    /// Wire `timestamp_ns` field (publisher clock).
    #[getter]
    fn timestamp_ns(&self) -> PyResult<u64> {
        self.check_live()?;
        Ok(self.header.timestamp_ns)
    }

    /// Wire `total_size` field (header + body bytes).
    #[getter]
    fn total_size(&self) -> PyResult<u32> {
        self.check_live()?;
        Ok(self.header.total_size)
    }

    /// Receive timestamp in ns on the same monotonic clock `real_ns()`
    /// reports, captured the moment the frame was popped.
    #[getter]
    fn recv_ns(&self) -> PyResult<u64> {
        self.check_live()?;
        Ok(self.recv_ns)
    }

    /// True after `release()` (or a context-manager exit).
    #[getter]
    fn is_released(&self) -> bool {
        self.released
    }

    /// Return the SHM slot to the publisher pool. Idempotent. With live
    /// buffer views the return is deferred until the last view closes -
    /// the views keep reading valid pinned memory until then, but the
    /// borrow budget stays consumed.
    fn release(&mut self) {
        self.released = true;
        self.exports.give_up();
        self.reap();
    }
}
