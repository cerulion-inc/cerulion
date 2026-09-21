// SPDX-License-Identifier: AGPL-3.0-only
//! `FillFrom` — producer abstraction for zero-copy variable-field writes.
//!
//! `FillFrom<T>` is what the codegen-emitted `fill_from_<field>` method on
//! `<Name>Shm` invokes to populate the SHM-loaned destination buffer. The
//! producer writes directly into the iceoryx2-managed region — no
//! intermediate `Vec`, no boundary memcpy.
//!
//! ```text
//!   user closure / driver shim          codegen `fill_from_<f>`
//!   ┌──────────────────────────┐        ┌────────────────────────────┐
//!   │ FnMut(&mut [T]) -> ...   │ ─────▶ │ loan_<f>(remaining)        │
//!   │      └─ writes T bytes ──┼───┐    │ src.fill_from(dst)?        │
//!   └──────────────────────────┘   │    │ commit cursor + offset to  │
//!                                  └──▶ │   actual `written` length  │
//!                                       └────────────────────────────┘
//! ```
//!
//! # Design choices
//!
//! - **Concrete error type.** `fill_from` returns
//!   `Result<usize, TransportError>` rather than a generic `Result<usize, E>`
//!   wrapper. Matches the standard Rust idiom (`std::io::Write::write`,
//!   `tokio::io`, `rclrs`) where one framework error type subsumes producer
//!   failures. Removes the associated-type / `Into` bound that an earlier
//!   draft carried.
//!
//! - **Generic over element type `T` (default `u8`).** Variable fields come
//!   in byte form (`String`, `Bytes`, `DynamicArray<u8>`, complex variable)
//!   AND typed-array form (`DynamicArray<f32>`, `DynamicArray<i32>`, ...).
//!   A single trait spans both — codegen instantiates it at the right `T`
//!   per field. `T = u8` is the default so the common closure form
//!   (`|buf| { … }` with no annotation) infers byte-orientation.
//!
//! - **Closure blanket impl + `SliceSource<&[T]>` wrapper.** Any
//!   `FnMut(&mut [T]) -> Result<usize, TransportError>` implements
//!   `FillFrom<T>` directly via a blanket. For raw `&[T]` slices, wrap
//!   with [`SliceSource::new`] — a bare `impl FillFrom<T> for &[T]`
//!   would collide with the closure blanket under Rust's coherence
//!   rules (even though no slice type actually implements `FnMut`).
//!   The two forms are shown under "In a node" below.
//!
//! - **No `&str` blanket.** `String` fields can be filled via the byte
//!   form (`fill_from` hands the producer raw `&mut [u8]`); the producer
//!   is responsible for valid UTF-8. The reader's existing
//!   `WireError::InvalidUtf8` covers misuse at read time.
//!
//! # In a node
//!
//! Inside `tick`, `self.<port>.<variable_field>.fill_from(producer)?` is the
//! no-copy way to write a variable-length field. The producer is handed the
//! loaned region and returns how many elements it wrote:
//!
//! ```rust
//! use cerulion_core::prelude::*;
//! use native_ros2_messages::sensor_msgs::Image;
//!
//! #[cerulion_node(period_ms = 33)]
//! #[derive(Default)]
//! struct CameraNode {
//!     #[output]
//!     image: Image,
//! }
//!
//! #[cerulion_node_impl]
//! impl CameraNode {
//!     fn tick(&mut self) -> Result<(), NodeError> {
//!         self.image.height = 2;
//!         self.image.width = 2;
//!         self.image.step = 6;
//!         self.image.header.frame_id = "camera";
//!         // Slice form: copy out of a buffer you already hold.
//!         self.image.encoding.fill_from(SliceSource::new(b"rgb8"))?;
//!         // Closure form: write straight into shared memory. `pixels` is
//!         // the remaining capacity of the slot, so take what you need.
//!         self.image.data.fill_from(|pixels: &mut [u8]| {
//!             let n = pixels.len().min(2 * 6);
//!             pixels[..n].fill(0);
//!             Ok(n)
//!         })?;
//!         Ok(())
//!     }
//! }
//! # fn main() {}
//! ```
//!
//! If the producer returns `Err`, the field is not marked written and the
//! frame is not published.

use crate::error::TransportError;

/// A producer that writes element data into a destination buffer.
///
/// Implementors:
/// - **Closures**: any `FnMut(&mut [T]) -> Result<usize, TransportError>`
///   implements `FillFrom<T>` via a blanket impl.
/// - **Slices**: wrap with [`SliceSource::new`] — `SliceSource<'a, T>`
///   implements `FillFrom<T>` when `T: Copy` and consumes the slice
///   progressively on repeated calls.
/// - **Driver shims**: a user-defined `struct V4l2Camera { ... }` with
///   `impl FillFrom for V4l2Camera` lets a driver hand bytes directly
///   into SHM without an intermediate buffer.
///
/// # The `T` parameter
///
/// `T` is the element type of the destination buffer. For byte-oriented
/// variable fields (`String`, `Bytes`, `DynamicArray<u8>`, complex
/// variable) this is `u8` (the default). For typed-array fields
/// (`DynamicArray<f32>`, ...) the codegen-emitted method binds `T` to
/// the schema-declared element type.
///
/// # Contract
///
/// `fill_from` must NOT write more than `dst.len()` elements. The
/// `Ok(written)` value is interpreted by the codegen call site as the
/// number of elements actually filled; the framework rewinds the SHM
/// cursor and truncates the offset-table entry accordingly. An impl that
/// returns `written > dst.len()` is unsound — codegen defensively
/// clamps via `min(written, dst.len())` before committing, and the
/// closure blanket impl carries a `debug_assert!(written <= dst.len())`
/// so lying producers blow up in debug/test builds.
///
/// # Object safety
///
/// `FillFrom` is **not** object-safe (the default `T = u8` plus a method
/// generic over `T` prevents `dyn FillFrom`). Use a generic bound
/// `<S: FillFrom>` or `<S: FillFrom<f32>>` instead of `Box<dyn …>`. If
/// you need runtime dispatch, store the closure form behind
/// `Box<dyn FnMut(&mut [u8]) -> Result<usize, TransportError>>` and pass
/// it as a closure (the blanket impl picks it up).
pub trait FillFrom<T = u8> {
    /// Write up to `dst.len()` elements into `dst`. Returns the number
    /// actually written (must be `<= dst.len()`).
    ///
    /// Producer errors are reported as [`TransportError`]. Use one of
    /// the existing variants if possible (e.g. `TransportError::NodeError`
    /// with a structured `node_id` + `reason`) so the error message
    /// reaching `tick`'s caller stays consistent with other transport
    /// failures.
    ///
    /// On `Err`, any bytes the producer wrote to `dst` before failing are
    /// unobservable to readers — the codegen call site does NOT commit
    /// the cursor or offset-table entry. The slot is reclaimed on the
    /// next loan; iceoryx2 does not zero-init so the partial bytes sit
    /// in an unreachable region until overwritten.
    ///
    /// Callers that consume `Ok(written)` as a length MUST clamp via
    /// `min(written, dst.len())` before indexing — the closure blanket
    /// impl carries a `debug_assert!` but does NOT clamp at runtime in
    /// release mode. The codegen call site is the enforcement
    /// boundary for the production path; downstream `FillFrom` callers
    /// outside of codegen MUST replicate the clamp.
    fn fill_from(&mut self, dst: &mut [T]) -> Result<usize, TransportError>;
}

// ============================================================
// Blanket impl: closures
// ============================================================

impl<F, T> FillFrom<T> for F
where
    F: FnMut(&mut [T]) -> Result<usize, TransportError>,
{
    #[inline]
    fn fill_from(&mut self, dst: &mut [T]) -> Result<usize, TransportError> {
        let dst_len = dst.len();
        let result = (self)(dst);
        if let Ok(written) = result {
            // Defense-in-depth: closure contract says
            // `written <= dst.len()`. Codegen at the call site clamps;
            // this debug_assert blows up tests when a
            // closure violates the contract, surfacing the bug at
            // source. Release-mode is a no-op.
            debug_assert!(
                written <= dst_len,
                "FillFrom closure returned written={written} > dst.len()={dst_len} — \
                 producer contract violation",
            );
        }
        result
    }
}

// ============================================================
// Wrapper for the &[T] slice case
// ============================================================
//
// A blanket `impl<T: Copy> FillFrom<T> for &[T]` would collide with the
// closure blanket (`FnMut` is a trait Rust considers potentially
// implemented by foreign types — coherence rules reject the dual
// blanket even though no slice type actually implements `FnMut`).
//
// `SliceSource` wraps a `&[T]` so the slice impl lives on a distinct
// concrete type. Construct via [`SliceSource::new`] and pass to
// `fill_from`:
//
// ```ignore
// self.image.data.fill_from(SliceSource::new(b"hello"))?;
// ```

/// Drainable wrapper around `&[T]` so it can be passed to a `FillFrom`
/// site without colliding with the closure blanket impl.
///
/// Each call to `fill_from` copies `min(self.remaining, dst.len())`
/// elements into `dst` and advances the internal cursor — repeated
/// calls drain the slice progressively. After `remaining == 0`,
/// subsequent calls return `Ok(0)`.
#[derive(Debug)]
pub struct SliceSource<'a, T> {
    remaining: &'a [T],
}

impl<'a, T> SliceSource<'a, T> {
    /// Wrap a `&[T]` for use as a `FillFrom<T>` producer.
    #[inline]
    pub fn new(slice: &'a [T]) -> Self {
        Self { remaining: slice }
    }

    /// Bytes (or elements) still un-consumed.
    #[inline]
    pub fn remaining(&self) -> &[T] {
        self.remaining
    }
}

impl<T: Copy> FillFrom<T> for SliceSource<'_, T> {
    #[inline]
    fn fill_from(&mut self, dst: &mut [T]) -> Result<usize, TransportError> {
        // Ordering is load-bearing: cursor advance MUST follow the
        // copy. If a panic ever slipped between the copy and the
        // advance, the next `fill_from` call would silently re-deliver
        // already-copied bytes — silent data corruption from the
        // consumer's view. Today `min` is total and `copy_from_slice`
        // cannot panic on equal-length slices, so no panic is
        // reachable, but the invariant is implicit; keep it that way.
        let n = self.remaining.len().min(dst.len());
        dst[..n].copy_from_slice(&self.remaining[..n]);
        self.remaining = &self.remaining[n..];
        Ok(n)
    }
}

// ============================================================
// Unit tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ----- Closure blanket impl -----

    #[test]
    fn closure_writes_bytes_and_returns_count() {
        let mut buf = [0u8; 8];
        let mut producer = |dst: &mut [u8]| {
            dst[..3].copy_from_slice(b"abc");
            Ok(3)
        };
        let written = producer.fill_from(&mut buf).unwrap();
        assert_eq!(written, 3);
        assert_eq!(&buf[..3], b"abc");
    }

    #[test]
    fn closure_zero_byte_write_is_ok() {
        let mut buf = [42u8; 4];
        let mut producer = |_dst: &mut [u8]| Ok(0);
        let written = producer.fill_from(&mut buf).unwrap();
        assert_eq!(written, 0);
        assert_eq!(buf, [42u8; 4], "dst untouched on Ok(0)");
    }

    #[test]
    fn closure_error_propagates_unchanged() {
        let mut buf = [0u8; 4];
        let mut producer = |_dst: &mut [u8]| {
            Err(TransportError::NodeError {
                node_id: "camera".into(),
                reason: "device disconnected".into(),
            })
        };
        let err = producer.fill_from(&mut buf).unwrap_err();
        match err {
            TransportError::NodeError { node_id, reason } => {
                assert_eq!(node_id, "camera");
                assert_eq!(reason, "device disconnected");
            }
            other => panic!("expected NodeError, got: {other:?}"),
        }
    }

    #[test]
    fn closure_writes_typed_elements() {
        // T = f32: closure infers from the &mut [f32] dst type.
        let mut buf = [0.0f32; 4];
        let mut producer = |dst: &mut [f32]| {
            dst[0] = 1.5;
            dst[1] = 2.5;
            Ok(2)
        };
        let written = producer.fill_from(&mut buf).unwrap();
        assert_eq!(written, 2);
        assert_eq!(buf[0], 1.5);
        assert_eq!(buf[1], 2.5);
        // Pin "Ok(n) means dst[..n] is written; dst[n..] is
        // producer-untouched" — symmetry with the SliceSource
        // long-dst test.
        assert_eq!(buf[2..], [0.0, 0.0]);
    }

    // These tests rely on `debug_assert!`
    // firing inside the closure blanket. `debug_assert!` is a no-op
    // in release mode, so `#[should_panic]` would fail with
    // "test did not panic as expected" on release-mode test runs
    // (CI's Latency Threshold job runs `cargo test --release`).
    // Gate on `debug_assertions` so they only run under debug.
    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "producer contract violation")]
    fn closure_lying_about_written_count_panics_in_debug() {
        // Defense-in-depth: closure contract is `written <= dst.len()`.
        // The closure blanket impl carries a debug_assert that fires
        // in debug/test builds when a closure lies. Release mode is a
        // no-op (codegen clamps).
        //
        // The `should_panic` expected substring matches the semantic
        // anchor ("producer contract violation") rather than the full
        // format string — refactors of the message that preserve
        // meaning don't break this test, but refactors that drop the
        // semantic anchor do.
        let mut buf = [0u8; 4];
        let mut producer = |_dst: &mut [u8]| Ok(usize::MAX);
        let _ = producer.fill_from(&mut buf);
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "producer contract violation")]
    fn closure_lying_about_written_count_off_by_one_panics_in_debug() {
        // Boundary-value twin of the `usize::MAX` test — `dst.len()+1`
        // is the off-by-one a real buggy producer is most likely to
        // hit (e.g. mis-tracking a 0-based index as 1-based). Pins
        // that the `debug_assert` catches both extreme and tight
        // contract violations.
        let mut buf = [0u8; 4];
        let mut producer = |dst: &mut [u8]| Ok(dst.len() + 1);
        let _ = producer.fill_from(&mut buf);
    }

    #[test]
    fn closure_returning_exactly_dst_len_is_ok() {
        // Inclusive bound check: `written == dst.len()` is the
        // permitted maximum. Kills the off-by-one mutation
        // (`debug_assert!(written < dst_len)` instead of `<=`) — that
        // mutation would silently pass every other test in this file.
        let mut buf = [0u8; 4];
        let mut producer = |dst: &mut [u8]| {
            dst.fill(0xff);
            Ok(dst.len())
        };
        let written = producer.fill_from(&mut buf).unwrap();
        assert_eq!(written, 4);
        assert_eq!(buf, [0xff; 4]);
    }

    // ----- SliceSource impl -----

    #[test]
    fn slice_source_copies_and_advances() {
        let mut src = SliceSource::new(b"hello".as_slice());
        let mut buf = [0u8; 5];
        let written = src.fill_from(&mut buf).unwrap();
        assert_eq!(written, 5);
        assert_eq!(&buf, b"hello");
        assert_eq!(src.remaining().len(), 0, "fully consumed");
    }

    #[test]
    fn slice_source_short_dst_partial_drain() {
        // dst smaller than src: copies dst.len(), advances by dst.len().
        let mut src = SliceSource::new(b"abcdef".as_slice());
        let mut buf = [0u8; 3];
        let written = src.fill_from(&mut buf).unwrap();
        assert_eq!(written, 3);
        assert_eq!(&buf, b"abc");
        assert_eq!(src.remaining(), b"def");
    }

    #[test]
    fn slice_source_long_dst_copies_only_remaining() {
        // dst larger than src: copies src.len(), src fully drained.
        let mut src = SliceSource::new(b"hi".as_slice());
        let mut buf = [0u8; 8];
        let written = src.fill_from(&mut buf).unwrap();
        assert_eq!(written, 2);
        assert_eq!(&buf[..2], b"hi");
        // Trailing bytes of dst untouched.
        assert_eq!(&buf[2..], &[0u8; 6]);
        assert_eq!(src.remaining().len(), 0);
    }

    #[test]
    fn slice_source_empty_src_returns_zero() {
        let mut src = SliceSource::new(b"".as_slice());
        let mut buf = [42u8; 4];
        let written = src.fill_from(&mut buf).unwrap();
        assert_eq!(written, 0);
        assert_eq!(buf, [42u8; 4], "dst untouched");
    }

    #[test]
    fn slice_source_chained_calls_drain_progressively() {
        let mut src = SliceSource::new(b"abcdef".as_slice());
        let mut buf = [0u8; 2];

        let n1 = src.fill_from(&mut buf).unwrap();
        assert_eq!(n1, 2);
        assert_eq!(&buf, b"ab");

        let n2 = src.fill_from(&mut buf).unwrap();
        assert_eq!(n2, 2);
        assert_eq!(&buf, b"cd");

        let n3 = src.fill_from(&mut buf).unwrap();
        assert_eq!(n3, 2);
        assert_eq!(&buf, b"ef");

        // Fourth call drains nothing.
        let n4 = src.fill_from(&mut buf).unwrap();
        assert_eq!(n4, 0);
    }

    #[test]
    fn slice_source_works_for_typed_elements() {
        let src_data = [1.5f32, 2.5, 3.5, 4.5];
        let mut src = SliceSource::new(src_data.as_slice());
        let mut buf = [0.0f32; 4];
        let written = src.fill_from(&mut buf).unwrap();
        assert_eq!(written, 4);
        assert_eq!(buf, src_data);
    }

    #[test]
    fn slice_source_works_for_zero_sized_dst() {
        // Empty dst: writes 0, does not advance source.
        let mut src = SliceSource::new(b"abc".as_slice());
        let mut buf: [u8; 0] = [];
        let written = src.fill_from(&mut buf).unwrap();
        assert_eq!(written, 0);
        assert_eq!(src.remaining(), b"abc", "source NOT consumed on empty dst");
    }

    #[test]
    fn slice_source_both_zero_returns_zero() {
        // Empty src + empty dst: min(0, 0) = 0, both no-op.
        let mut src: SliceSource<'_, u8> = SliceSource::new(&[]);
        let mut buf: [u8; 0] = [];
        let written = src.fill_from(&mut buf).unwrap();
        assert_eq!(written, 0);
        assert_eq!(src.remaining().len(), 0);
    }

    // ----- Trait object usability sanity check -----
    // `FillFrom` is not object-safe by design (default-T generics on
    // trait methods don't dispatch dynamically). Confirming via a
    // compile-time function that accepts impl FillFrom — proves the
    // trait is at least usable as a bound.

    fn _accepts_byte_filler<S: FillFrom>(_src: S) {}
    fn _accepts_f32_filler<S: FillFrom<f32>>(_src: S) {}

    #[test]
    fn trait_works_as_generic_bound() {
        _accepts_byte_filler(SliceSource::new(b"x".as_slice()));
        _accepts_byte_filler(|_buf: &mut [u8]| Ok(0));
        _accepts_f32_filler(SliceSource::new([0.0f32].as_slice()));
        _accepts_f32_filler(|_buf: &mut [f32]| Ok(0));
    }
}
