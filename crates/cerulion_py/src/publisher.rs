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
use crate::frame::{export_released_on_owner, stamp_export_owner, warn_offthread_release};
use crate::typed::PySchemaSet;
use cerulion_core::clock::real_ns;
use cerulion_core::dynamic::{FrameEncoder, FrameView};
use cerulion_core::transport::publisher::RawShmLoan;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{CerulionPublisher, TransportManager};
use pyo3::buffer::PyBuffer;
use pyo3::exceptions::{PyBufferError, PyTypeError, PyValueError};
use pyo3::ffi;
use pyo3::prelude::*;

/// A publisher on one topic. `max_payload_len` bounds every frame body;
/// the iceoryx2 slot is `WireHeader::SIZE + max_payload_len` (allocation
/// strategy Static - a larger loan fails at loan time).
#[pyclass(unsendable)]
pub struct Publisher {
    publisher: CerulionPublisher,
    schema_hash: u64,
    max_payload_len: usize,
    next_sequence: u32,
}

impl Publisher {
    /// Commit `loan`: stamp the header (sequence taken HERE, at commit -
    /// a failed loan burned no number) and send. Shared by `publish` and
    /// `Loan::commit`.
    fn commit_loan(
        &mut self,
        mut loan: RawShmLoan,
        payload_len: usize,
        timestamp_ns: Option<u64>,
        preserve_layout: bool,
    ) -> PyResult<()> {
        let seq = self.next_sequence;
        let mut header = if preserve_layout {
            WireHeader::read_from_buf(loan.bytes_mut()).ok_or_else(|| {
                EncodeError::new_err("typed loan does not contain a complete wire header")
            })?
        } else {
            let mut header =
                WireHeader::new(self.schema_hash, seq, timestamp_ns.unwrap_or_else(real_ns));
            header.total_size = (WireHeader::SIZE + payload_len) as u32;
            header
        };
        header.schema_hash = self.schema_hash;
        header.sequence = seq;
        header.timestamp_ns = timestamp_ns.unwrap_or_else(real_ns);
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
        let len = payload.len_bytes();
        if len > self.max_payload_len {
            return Err(EncodeError::new_err(format!(
                "payload length {len} exceeds max_payload_len {}",
                self.max_payload_len
            )));
        }
        let src = payload.as_slice(py).ok_or_else(|| {
            PyTypeError::new_err("payload must be a contiguous bytes-like object")
        })?;
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
        self.commit_loan(loan, len, timestamp_ns, false)
    }

    /// Loan a zero-initialised SHM slot for a `payload_len`-byte body.
    /// The slot is zero-initialised before it is handed to Python, so
    /// unwritten payload bytes are deterministic (never recycled garbage).
    fn loan(slf: &Bound<'_, Self>, payload_len: usize) -> PyResult<Loan> {
        let mut this = slf.borrow_mut();
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
            exports: 0,
            closed: false,
            pending_drop: false,
            variable_entries: None,
        })
    }

    fn loan_typed(
        slf: &Bound<'_, Self>,
        schemas: PyRef<'_, PySchemaSet>,
        name: &str,
        var_lens: Vec<usize>,
        timestamp_ns: Option<u64>,
    ) -> PyResult<Loan> {
        let mut this = slf.borrow_mut();
        let layout = schemas
            .inner
            .layout(name)
            .ok_or_else(|| EncodeError::new_err(format!("unknown schema {name}")))?;
        let encoder = FrameEncoder::new(layout)
            .map_err(|e| Python::attach(|py| crate::errors::map_dynamic_err(py, e)))?;
        let total = encoder
            .required_len(&var_lens)
            .map_err(|e| Python::attach(|py| crate::errors::map_dynamic_err(py, e)))?;
        let payload_len = total
            .checked_sub(WireHeader::SIZE)
            .ok_or_else(|| EncodeError::new_err("typed frame is shorter than its header"))?;
        if payload_len > this.max_payload_len {
            return Err(EncodeError::new_err(format!(
                "typed frame payload {payload_len} exceeds max_payload_len {}",
                this.max_payload_len
            )));
        }
        let mut raw = this
            .publisher
            .loan_raw_uninit(total)
            .map_err(map_transport_err)?;
        for byte in raw.bytes_uninit_mut() {
            byte.write(0);
        }
        // SAFETY: the loop above zero-initialised every byte of the slot.
        let mut loan = unsafe { raw.assume_init() };
        encoder
            .begin(
                loan.bytes_mut(),
                &var_lens,
                timestamp_ns.unwrap_or_else(real_ns),
            )
            .map_err(|e| Python::attach(|py| crate::errors::map_dynamic_err(py, e)))?;
        let entries = {
            let frame_bytes = loan.bytes_mut();
            let view = FrameView::with_layout(layout, frame_bytes)
                .map_err(|e| Python::attach(|py| crate::errors::map_dynamic_err(py, e)))?;
            let payload = view.payload();
            layout
                .variable_fields
                .iter()
                .map(|field| {
                    let bytes = view
                        .variable_field(&field.name)
                        .map_err(|e| Python::attach(|py| crate::errors::map_dynamic_err(py, e)))?;
                    let offset = (bytes.as_ptr() as usize)
                        .checked_sub(payload.as_ptr() as usize)
                        .ok_or_else(|| {
                            PyValueError::new_err(
                                "variable field slice lies outside the loan payload",
                            )
                        })?;
                    Ok((field.name.clone(), offset, bytes.len()))
                })
                .collect::<PyResult<Vec<_>>>()?
        };
        drop(this);
        Ok(Loan {
            publisher: slf.clone().unbind(),
            loan: Some(loan),
            payload_len,
            timestamp_ns,
            exports: 0,
            closed: false,
            pending_drop: false,
            variable_entries: Some(entries),
        })
    }

    fn publish_frame(
        &mut self,
        py: Python<'_>,
        frame: PyBuffer<u8>,
        timestamp_ns: Option<u64>,
    ) -> PyResult<()> {
        let src = frame
            .as_slice(py)
            .ok_or_else(|| PyTypeError::new_err("frame must be a contiguous bytes-like object"))?;
        if src.len() < WireHeader::SIZE {
            return Err(EncodeError::new_err(
                "frame is shorter than the wire header",
            ));
        }
        let header_bytes: [u8; WireHeader::SIZE] = std::array::from_fn(|index| src[index].get());
        let header = WireHeader::read_from_buf(&header_bytes)
            .ok_or_else(|| EncodeError::new_err("frame is shorter than the wire header"))?;
        let total = header.total_size as usize;
        if total < WireHeader::SIZE || total > src.len() {
            return Err(EncodeError::new_err(
                "frame total_size is outside its buffer",
            ));
        }
        let payload_len = total - WireHeader::SIZE;
        if payload_len > self.max_payload_len {
            return Err(EncodeError::new_err(format!(
                "frame payload {payload_len} exceeds max_payload_len {}",
                self.max_payload_len
            )));
        }
        let mut loan = self
            .publisher
            .loan_raw_uninit(total)
            .map_err(map_transport_err)?;
        for (dst, value) in loan.bytes_uninit_mut().iter_mut().zip(&src[..total]) {
            dst.write(value.get());
        }
        // SAFETY: every byte in the requested frame range was initialized above.
        let loan = unsafe { loan.assume_init() };
        self.commit_loan(loan, payload_len, timestamp_ns, true)
    }
}

/// A live SHM loan handed to Python as a WRITABLE buffer over
/// `bytes_mut()[WireHeader::SIZE..]` (the payload region; the header is
/// stamped by `commit()`, never by the caller).
///
/// The buffer-export count keeps the loan alive while a view exists -
/// `commit()` refuses with `EncodeError` while exports are live (a view
/// over a sent slot would dangle); `discard()` with live exports defers
/// the slot's return to the last `__releasebuffer__` (`pending_drop`).
#[pyclass(unsendable)]
pub struct Loan {
    publisher: Py<Publisher>,
    loan: Option<RawShmLoan>,
    payload_len: usize,
    timestamp_ns: Option<u64>,
    exports: usize,
    closed: bool,
    pending_drop: bool,
    variable_entries: Option<Vec<(String, usize, usize)>>,
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
        this.exports += 1;
        drop(this);
        unsafe { stamp_export_owner(view) };
        Ok(())
    }

    /// SAFETY: called by the interpreter once per live export produced by
    /// `__getbuffer__`; `view` is the same pointer it filled there. The
    /// release path reads its exporter-private `internal` stamp, decrements
    /// bookkeeping, and `ffi::PyBuffer_Release` (inside the runtime) drops
    /// the reference FillInfo added.
    ///
    /// Same foreign-thread rule as `Frame::__releasebuffer__` (see
    /// `frame.rs`): off the owning thread the borrow would panic, so
    /// bookkeeping is skipped with a `RuntimeWarning`.
    /// A skipped decrement leaves `exports > 0` - `commit()` keeps
    /// refusing - and the slot is returned when the `Loan` itself drops.
    unsafe fn __releasebuffer__(slf: Bound<'_, Self>, view: *mut ffi::Py_buffer) {
        let on_owner = unsafe { export_released_on_owner(view) };
        let Some(mut this) = on_owner.then(|| slf.try_borrow_mut().ok()).flatten() else {
            warn_offthread_release(slf.py(), "Loan");
            return;
        };
        this.exports = this.exports.saturating_sub(1);
        if this.pending_drop && this.exports == 0 {
            this.loan = None;
        }
    }

    /// Payload length in bytes (the writable region's length).
    #[getter]
    fn payload_len(&self) -> usize {
        self.payload_len
    }

    fn variable_entries(&self) -> Option<Vec<(String, usize, usize)>> {
        self.variable_entries.clone()
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
        if this.exports > 0 {
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
        let preserve_layout = this.variable_entries.is_some();
        let publisher = this.publisher.clone_ref(py);
        drop(this);
        let result =
            publisher
                .borrow_mut(py)
                .commit_loan(loan, payload_len, timestamp_ns, preserve_layout);
        result
    }

    /// Release the SHM slot without sending (closes the loan). A live
    /// view defers the return to the last `__releasebuffer__`.
    fn discard(&mut self) {
        if self.closed {
            return;
        }
        self.closed = true;
        if self.exports == 0 {
            self.loan = None;
        } else {
            self.pending_drop = true;
        }
    }
}
