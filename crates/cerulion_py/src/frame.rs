// SPDX-License-Identifier: AGPL-3.0-only
//! `Frame` - an owned inbound sample with a READ-ONLY buffer export.
//!
//! The frame owns the `OwnedInboundSample`, so the SHM slot stays pinned
//! (borrowed, never overwritten) as long as the frame - or any buffer
//! exported from it - is alive. `release()` returns the slot early; with
//! live views it takes effect at the last `__releasebuffer__` (a
//! memoryview/ndarray keeps reading VALID pinned memory - nothing can
//! invalidate a CPython memoryview from the exporter side).

use crate::errors::{ReleasedFrame, TransportError as PyTransportError};
use cerulion_core::transport::subscriber::OwnedInboundSample;
use cerulion_core::WireHeader;
use pyo3::exceptions::{PyBufferError, PyRuntimeWarning};
use pyo3::ffi;
use pyo3::prelude::*;
use std::cell::Cell;
use std::ffi::CString;
use std::sync::atomic::{AtomicUsize, Ordering};

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

/// Record the exporting thread in `view.internal` (exporter-private per
/// the buffer protocol; `PyBuffer_FillInfo` leaves it NULL).
///
/// SAFETY: caller passes the `Py_buffer` the interpreter handed to
/// `__getbuffer__`, after a successful `PyBuffer_FillInfo`.
pub(crate) unsafe fn stamp_export_owner(view: *mut ffi::Py_buffer) {
    unsafe { (*view).internal = thread_token() as *mut std::ffi::c_void };
}

/// True when the calling thread is the one that exported `view`.
///
/// SAFETY: caller passes the `Py_buffer` the interpreter hands to
/// `__releasebuffer__` - the same struct `stamp_export_owner` stamped.
pub(crate) unsafe fn export_released_on_owner(view: *mut ffi::Py_buffer) -> bool {
    unsafe { (*view).internal as usize == thread_token() }
}

/// Emit the `RuntimeWarning` for a `__releasebuffer__` that ran off its
/// object's owning thread (or could not borrow it). `releasebuffer`
/// cannot raise, so the `Result` is ignored by callers.
pub(crate) fn warn_offthread_release(py: Python<'_>, class: &str) {
    let msg = CString::new(format!(
        "cerulion {class} buffer released off the owning thread; \
         the shared-memory slot is returned only when the {class} itself is dropped"
    ))
    .unwrap_or_else(|_| c"cerulion buffer released off the owning thread".to_owned());
    let _ = PyErr::warn(py, &py.get_type::<PyRuntimeWarning>(), &msg, 1);
}

/// One received wire frame (32-byte header + body).
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
    exports: usize,
    released: bool,
}

impl Frame {
    pub(crate) fn new(sample: OwnedInboundSample, header: WireHeader, recv_ns: u64) -> Self {
        Self {
            sample: Some(sample),
            header,
            recv_ns,
            exports: 0,
            released: false,
        }
    }

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
        drop(this);
        let mut this = slf.borrow_mut();
        if rc != 0 {
            return Err(PyErr::take(slf.py())
                .unwrap_or_else(|| PyBufferError::new_err("frame buffer export failed")));
        }
        this.exports += 1;
        drop(this);
        unsafe { stamp_export_owner(view) };
        Ok(())
    }

    /// SAFETY: called by the interpreter once per live export produced by
    /// `__getbuffer__`; `view` is the same pointer it filled there. The
    /// release path reads its exporter-private `internal` stamp, then
    /// decrements bookkeeping and may trigger a deferred `release()` drop.
    ///
    /// CPython may run this on a FOREIGN thread (a memoryview handed to
    /// another `threading.Thread` releases there). Borrowing an
    /// unsendable pyclass off its owning thread panics in pyo3, so the
    /// borrow is gated on the per-export thread token; on a foreign thread
    /// bookkeeping is skipped with a `RuntimeWarning` and the SHM slot is
    /// returned when the `Frame` itself is dropped.
    /// (`releasebuffer` cannot raise: the warning's `Result` is ignored.)
    unsafe fn __releasebuffer__(slf: Bound<'_, Self>, view: *mut ffi::Py_buffer) {
        let on_owner = unsafe { export_released_on_owner(view) };
        let Some(mut this) = on_owner.then(|| slf.try_borrow_mut().ok()).flatten() else {
            warn_offthread_release(slf.py(), "Frame");
            return;
        };
        this.exports = this.exports.saturating_sub(1);
        if this.released && this.exports == 0 {
            this.sample = None;
        }
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
    /// buffer views the return is deferred to the last `__releasebuffer__`
    /// - the views keep reading valid pinned memory until then, but the
    /// borrow budget stays consumed.
    fn release(&mut self) {
        self.released = true;
        if self.exports == 0 {
            self.sample = None;
        }
    }
}
