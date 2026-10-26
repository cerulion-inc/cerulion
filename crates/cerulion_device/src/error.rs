// SPDX-License-Identifier: AGPL-3.0-only
//! The one error type of this crate.

use crate::backend::BackendKind;

/// Why a device operation refused or failed.
///
/// A backend that cannot run on this machine reports [`Self::LibraryNotFound`]
/// or [`Self::NoDevice`] from its `probe` (HIP also reports
/// [`Self::Unsupported`] for a runtime major whose attribute numbers were
/// not checked). Every later failure names the
/// backend and the call that failed, so a mid-run driver error is never
/// mistaken for an absent GPU.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum DeviceError {
    /// None of the library names the backend tries could be opened.
    #[error("{backend} backend: could not open any of {tried:?}: {reason}")]
    LibraryNotFound {
        /// The backend that looked for its library.
        backend: BackendKind,
        /// Every name passed to dlopen, in order.
        tried: Vec<&'static str>,
        /// The loader's message for the last name tried.
        reason: String,
    },
    /// The library opened but lacks a symbol the backend needs.
    #[error("{backend} backend: library {library} has no symbol {symbol}")]
    SymbolMissing {
        /// The backend whose table is incomplete.
        backend: BackendKind,
        /// The library that was opened.
        library: &'static str,
        /// The symbol that was not found.
        symbol: &'static str,
    },
    /// The driver loaded but reports no usable device.
    #[error("{backend} backend: no device ({detail})")]
    NoDevice {
        /// The backend that found no device.
        backend: BackendKind,
        /// What the driver reported.
        detail: String,
    },
    /// A driver call returned an error code.
    #[error("{backend} backend: {call} failed with code {code} ({message})")]
    Driver {
        /// The backend whose driver failed.
        backend: BackendKind,
        /// The driver entry point that failed.
        call: &'static str,
        /// The driver's numeric result.
        code: i64,
        /// The driver's description of `code`, or `"unknown"`.
        message: String,
    },
    /// The device cannot do what was asked (for example, register host
    /// memory read-only).
    #[error("{backend} backend: {what} is not supported on this device")]
    Unsupported {
        /// The backend that refused.
        backend: BackendKind,
        /// The capability that is missing.
        what: &'static str,
    },
    /// No mapping of this process contains the address.
    #[error("address {addr:#x} is not inside any mapping of this process")]
    NoMapping {
        /// The address that was looked up.
        addr: usize,
    },
    /// The process memory map could not be read.
    #[error("could not read the process memory map: {reason}")]
    MemoryMap {
        /// The operating system's message.
        reason: String,
    },
    /// The range is not inside a segment registered with the backend.
    #[error("range {addr:#x}+{len} is not inside a registered segment")]
    NotRegistered {
        /// Start of the range.
        addr: usize,
        /// Length of the range in bytes.
        len: usize,
    },
    /// A copy was given a source and destination of different lengths.
    #[error("copy source is {src} bytes but the destination is {dst} bytes")]
    LengthMismatch {
        /// Source length in bytes.
        src: usize,
        /// Destination length in bytes.
        dst: usize,
    },
    /// A write was asked through a segment registered read-only.
    #[error("range {addr:#x}+{len} is in a segment registered read-only")]
    ReadOnly {
        /// Start of the range.
        addr: usize,
        /// Length of the range in bytes.
        len: usize,
    },
}

/// Result alias for this crate.
pub type DeviceResult<T> = Result<T, DeviceError>;
