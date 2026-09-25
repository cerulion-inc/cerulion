// SPDX-License-Identifier: AGPL-3.0-only
//! `Publisher` - raw wire-frame publishing over a `CerulionPublisher`.
//!
//! The core's raw-loan API does NOT stamp headers or sequences (that is
//! `pub(crate)` machinery the runtime owns), so this class keeps its own
//! `u32` counter - the exact precedent `rmw_cerulion` sets: take the
//! sequence at COMMIT (a failed loan burns none), rewrite
//! `WireHeader.sequence`, `write_to_buf` into `bytes_mut()[..32]` LAST so
//! every byte the consumer reads is initialised, then `send_raw_loan`,
//! `check_subscriber_events`, `notify_sent_sample` (Err → debug, never
//! fatal: the data is already committed to SHM).

use crate::errors::{map_transport_err, EncodeError};
use crate::frame::{release_export, stamp_export, warn_offthread_release, Exports};
use cerulion_core::clock::real_ns;
use cerulion_core::transport::publisher::RawShmLoan;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{CerulionPublisher, TransportManager};
use pyo3::buffer::PyBuffer;
use pyo3::exceptions::{PyBufferError, PyTypeError, PyValueError};
use pyo3::ffi;
use pyo3::prelude::*;
use std::sync::Arc;

/// A publisher on one topic. `max_payload_len` bounds every frame body;
/// the iceoryx2 slot is `WireHeader::SIZE + max_payload_len` (allocation
/// strategy Static - a larger loan fails at loan time).
#[pyclass(unsendable)]
pub struct Publisher {
    parked: Vec<(Arc<Exports>, RawShmLoan)>,
    publisher: CerulionPublisher,
    schema_hash: u64,
    max_payload_len: usize,
    next_sequence: u32,
}

impl Publisher {
    /// Return every parked loan whose views have all closed.
    fn reap(&mut self) {
        self.parked.retain(|(exports, _)| exports.live() > 0);
    }

    /// Commit `loan`: stamp the header (sequence taken HERE, at commit -
    /// a failed loan burned no number) and send. Shared by `publish` and
    /// `Loan::commit`.
    fn commit_loan(
        &mut self,
        mut loan: RawShmLoan,
        payload_len: usize,
        timestamp_ns: Option<u64>,
    ) -> PyResult<()> {
        let seq = self.next_sequence;
        let mut header =
            WireHeader::new(self.schema_hash, seq, timestamp_ns.unwrap_or_else(real_ns));
        header.total_size = (WireHeader::SIZE + payload_len) as u32;
        // Header written LAST: body bytes were placed first (or zero-
        // initialised by `loan()`), so no uninitialised range is visible.
        header.write_to_buf(&mut loan.bytes_mut()[..WireHeader::SIZE]);
        self.publisher
            .send_raw_loan(loan)
            .map_err(map_transport_err)?;
        self.next_sequence = seq.wrapping_add(1);
        self.publisher.check_subscriber_events();
        if let Err(e) = self.publisher.notify_sent_sample() {
            tracing::debug!(
                topic = %self.publisher.topic(),
                error = ?e,
                "notify failed"
            );
        }
        Ok(())
    }
}

#[pymethods]
impl Publisher {
    /// `Publisher(topic, schema_hash, max_payload_len)` - the transport
    /// must be `connect()`ed first (`NotInitialized` → `TransportError`).
    #[new]
    fn new(topic: &str, schema_hash: u64, max_payload_len: usize) -> PyResult<Self> {
        let slot_len = max_payload_len
            .checked_add(WireHeader::SIZE)
            .and_then(|n| u32::try_from(n).ok())
            .and_then(MaxSliceLen::try_new)
            .ok_or_else(|| {
                EncodeError::new_err(format!(
                    "max_payload_len {max_payload_len} cannot be represented on the wire \
                     (WireHeader::SIZE + len must fit in u32)"
                ))
            })?;
        let mgr = TransportManager::get().map_err(map_transport_err)?;
        let publisher = mgr
            .create_publisher(topic, slot_len, 0)
            .map_err(map_transport_err)?;
        Ok(Self {
            parked: Vec::with_capacity(2),
            publisher,
            schema_hash,
            max_payload_len,
            next_sequence: 0,
        })
    }

    /// Topic name this publisher is bound to.
    #[getter]
    fn topic(&self) -> &str {
        self.publisher.topic()
    }

    /// Schema hash stamped into every frame's wire header.
    #[getter]
    fn schema_hash(&self) -> u64 {
        self.schema_hash
    }

    /// Maximum payload bytes a single frame may carry.
    #[getter]
    fn max_payload_len(&self) -> usize {
        self.max_payload_len
    }

    /// Sequence number the NEXT committed frame will carry.
    #[getter]
    fn sequence(&self) -> u32 {
        self.next_sequence
    }

    /// Publish `payload` (any contiguous bytes-like object) as one wire
    /// frame - one copy from the Python buffer into the SHM loan.
    #[pyo3(signature = (payload, timestamp_ns=None))]
    fn publish(
        &mut self,
        py: Python<'_>,
        payload: PyBuffer<u8>,
        timestamp_ns: Option<u64>,
    ) -> PyResult<()> {
        let src = payload.as_slice(py).ok_or_else(|| {
            PyTypeError::new_err("payload must be a contiguous bytes-like object")
        })?;
        let len = src.len();
        self.reap();
        if len > self.max_payload_len {
            return Err(EncodeError::new_err(format!(
                "payload length {len} exceeds max_payload_len {}",
                self.max_payload_len
            )));
        }
        let mut loan = self
            .publisher
            .loan_raw_uninit(WireHeader::SIZE + len)
            .map_err(map_transport_err)?;
        for (dst, src) in loan.bytes_uninit_mut()[WireHeader::SIZE..]
            .iter_mut()
            .zip(src.iter())
        {
            dst.write(src.get());
        }
        for byte in &mut loan.bytes_uninit_mut()[..WireHeader::SIZE] {
            byte.write(0);
        }
        // SAFETY: the loan is exactly `WireHeader::SIZE + len` bytes; the
        // body loop wrote [32,32+len) and the zero-fill wrote [0,32),
        // covering every byte of the slot before `assume_init` (the real
        // header is stamped by `commit_loan` below).
        let loan = unsafe { loan.assume_init() };
        self.commit_loan(loan, len, timestamp_ns)
    }

    /// Loan a zero-initialised SHM slot for a `payload_len`-byte body.
    /// The slot is zero-initialised before it is handed to Python, so
    /// unwritten payload bytes are deterministic (never recycled garbage).
    fn loan(slf: &Bound<'_, Self>, payload_len: usize) -> PyResult<Loan> {
        let mut this = slf.borrow_mut();
        this.reap();
        if payload_len > this.max_payload_len {
            return Err(EncodeError::new_err(format!(
                "payload_len {payload_len} exceeds max_payload_len {}",
                this.max_payload_len
            )));
        }
        let mut loan = this
            .publisher
            .loan_raw_uninit(WireHeader::SIZE + payload_len)
            .map_err(map_transport_err)?;
        for byte in loan.bytes_uninit_mut() {
            byte.write(0);
        }
        // SAFETY: the zero-fill loop above wrote every byte of the slot.
        let loan = unsafe { loan.assume_init() };
        drop(this);
        Ok(Loan {
            publisher: slf.clone().unbind(),
            loan: Some(loan),
            payload_len,
            timestamp_ns: None,
            exports: Exports::new(),
            closed: false,
            pending_drop: false,
        })
    }
}

/// A live SHM loan handed to Python as a WRITABLE buffer over
/// `bytes_mut()[WireHeader::SIZE..]` (the payload region; the header is
/// stamped by `commit()`, never by the caller).
///
/// The buffer-export count keeps the loan alive while a view exists -
/// `commit()` refuses with `EncodeError` while exports are live (a view
/// over a sent slot would dangle); `discard()` with live exports parks
/// the slot with its publisher until the last `__releasebuffer__`.
#[pyclass(unsendable)]
pub struct Loan {
    publisher: Py<Publisher>,
    loan: Option<RawShmLoan>,
    payload_len: usize,
    timestamp_ns: Option<u64>,
    exports: Arc<Exports>,
    closed: bool,
    pending_drop: bool,
}

#[pymethods]
impl Loan {
    /// Export the payload region `bytes_mut()[WireHeader::SIZE..]` as a
    /// writable contiguous buffer (refuses once the loan is closed).
    ///
    /// SAFETY: `view` is a valid `*mut ffi::Py_buffer` the interpreter
    /// handed us to fill; `PyBuffer_FillInfo` writes it and (on success)
    /// increfs `slf` into `view->obj`, so `self` outlives the export. The
    /// exported pointer stays valid while the export is counted: the
    /// `exports` counter keeps the SHM loan alive until the matching
    /// `__releasebuffer__` (commit refuses while exports are live;
    /// discard defers via `pending_drop`).
    unsafe fn __getbuffer__(
        slf: Bound<'_, Self>,
        view: *mut ffi::Py_buffer,
        flags: std::ffi::c_int,
    ) -> PyResult<()> {
        let mut this = slf.borrow_mut();
        if this.closed {
            return Err(PyValueError::new_err("loan already committed or discarded"));
        }
        let Some(loan) = this.loan.as_mut() else {
            return Err(PyValueError::new_err("loan already committed or discarded"));
        };
        let body = &mut loan.bytes_mut()[WireHeader::SIZE..];
        let rc = unsafe {
            ffi::PyBuffer_FillInfo(
                view,
                slf.as_ptr(),
                body.as_mut_ptr().cast(),
                body.len() as ffi::Py_ssize_t,
                0,
                flags,
            )
        };
        if rc != 0 {
            return Err(PyErr::take(slf.py())
                .unwrap_or_else(|| PyBufferError::new_err("loan buffer export failed")));
        }
        unsafe { stamp_export(view, &this.exports) };
        Ok(())
    }

    /// SAFETY: called by the interpreter once per live export produced by
    /// `__getbuffer__`; `view` is the same pointer it filled there. The
    /// release path uncounts the export stored in its exporter-private
    /// `internal` field, and `ffi::PyBuffer_Release` (inside the runtime)
    /// drops the reference FillInfo added.
    ///
    /// Same foreign-thread rule as `Frame::__releasebuffer__` (see
    /// `frame.rs`): the atomic count is always decremented, so `commit()`
    /// works once every view is released on any thread. A discarded
    /// loan's slot is parked with its publisher; when its last view
    /// closes off the owning thread, it returns at the publisher's next
    /// `publish()` or `loan()`, with a `RuntimeWarning`.
    unsafe fn __releasebuffer__(slf: Bound<'_, Self>, view: *mut ffi::Py_buffer) {
        let Some(done) = (unsafe { release_export(view) }) else {
            return;
        };
        if done.live > 0 || !done.given_up {
            return;
        }
        if done.on_owner {
            if let Ok(mut this) = slf.try_borrow_mut() {
                if this.pending_drop {
                    this.loan = None;
                }
                if let Ok(mut publisher) = this.publisher.bind(slf.py()).try_borrow_mut() {
                    publisher.reap();
                    return;
                }
            }
        }
        warn_offthread_release(
            slf.py(),
            "Loan",
            "at the publisher's next publish() or loan()",
        );
    }

    /// Payload length in bytes (the writable region's length).
    #[getter]
    fn payload_len(&self) -> usize {
        self.payload_len
    }

    /// Optional explicit wire timestamp for the next commit.
    #[getter]
    fn timestamp_ns(&self) -> Option<u64> {
        self.timestamp_ns
    }

    #[setter]
    fn set_timestamp_ns(&mut self, value: Option<u64>) {
        self.timestamp_ns = value;
    }

    /// True while the loan is neither committed nor discarded.
    #[getter]
    fn is_open(&self) -> bool {
        !self.closed
    }

    /// Stamp the header and send the loan (closes it).
    fn commit(slf: Bound<'_, Self>, py: Python<'_>) -> PyResult<()> {
        let mut this = slf.borrow_mut();
        if this.closed {
            return Err(PyValueError::new_err("loan already committed or discarded"));
        }
        if this.exports.live() > 0 {
            return Err(EncodeError::new_err(
                "release loan.payload views before commit()",
            ));
        }
        let Some(loan) = this.loan.take() else {
            return Err(PyValueError::new_err("loan already committed or discarded"));
        };
        this.closed = true;
        let payload_len = this.payload_len;
        let timestamp_ns = this.timestamp_ns;
        let publisher = this.publisher.clone_ref(py);
        drop(this);
        let result = publisher
            .borrow_mut(py)
            .commit_loan(loan, payload_len, timestamp_ns);
        result
    }

    /// Release the SHM slot without sending (closes the loan). With a
    /// live view the slot is parked with the publisher and returns once
    /// the last view closes.
    fn discard(&mut self, py: Python<'_>) {
        if self.closed {
            return;
        }
        self.closed = true;
        self.pending_drop = true;
        self.exports.give_up();
        let Some(loan) = self.loan.take() else {
            return;
        };
        if self.exports.live() == 0 {
            return;
        }
        match self.publisher.bind(py).try_borrow_mut() {
            Ok(mut publisher) => publisher.parked.push((Arc::clone(&self.exports), loan)),
            Err(_) => self.loan = Some(loan),
        }
    }
}
