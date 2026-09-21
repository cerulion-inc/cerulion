// SPDX-License-Identifier: MIT OR Apache-2.0
//! # cerulion-wire
//!
//! A minimal, dependency-light reader/encoder for the **Cerulion wire
//! format**: the 32-byte header plus the offset-table framing that carries
//! every Cerulion message on the wire.
//!
//! This crate exists so **closed-source** applications (the Cerulion Studio
//! sidecar) can DECODE raw Cerulion frames without linking the AGPL
//! `cerulion_core`. Raw frame bytes are data; a byte-format reader is a
//! protocol boundary, not a derivative of the runtime, so this crate is
//! permissive-licensed (MIT OR Apache-2.0) while `cerulion_core` stays AGPL.
//!
//! ## Deliberate duplication
//!
//! The types here ([`WireHeader`], [`OffsetEntry`]) are a **byte-exact,
//! hand-maintained mirror** of `crates/cerulion_core/src/wire.rs`. This duplication
//! is intentional: it is what lets the permissive decode seam stand alone
//! with no `cerulion_core` dependency. **Byte-compatibility with
//! that file is pinned by `cerulion_core`'s lockstep test**
//! (`crates/cerulion_core/tests/wire_lockstep_test.rs`), which encodes frames with
//! the real writer and re-parses them with this crate; any drift in the
//! source-of-truth layout fails that test loudly.
//!
//! ## Scope: the STRUCTURAL layer only
//!
//! This crate decodes the wire STRUCTURE (the header, offset-table entry
//! iteration, the fixed section, and per-variable-field payload slices), all
//! driven by the self-describing header, with NO schema required. Interpreting
//! those bytes as TYPED fields (field names and element types from a `.msg`
//! schema) needs a schema and is not part of this crate; in `cerulion_core` that
//! is the frame walker's job.
//!
//! ## Wire layout
//!
//! ```text
//! [WireHeader (32 B)][fixed section][offset table (8·N B)][variable payload]
//! ```
//!
//! ## Example
//!
//! ```
//! use cerulion_wire::{Frame, WireHeader};
//!
//! # fn demo(frame_bytes: &[u8]) -> Result<(), cerulion_wire::WireError> {
//! let frame = Frame::parse(frame_bytes)?;
//! let hdr: &WireHeader = frame.header();
//! let _schema_tag = hdr.schema_hash;
//! let _fixed = frame.fixed_section();
//! for (i, entry) in frame.offset_entries().enumerate() {
//!     let _bytes = frame.variable_field(i).unwrap()?; // slice of variable field i
//!     let _ = entry.length;
//! }
//! # Ok(())
//! # }
//! ```

// Principle 12 (logging): library code never prints. It logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

mod error;
mod frame;
mod header;
mod offset;

pub use error::WireError;
pub use frame::Frame;
pub use header::WireHeader;
pub use offset::OffsetEntry;

// Byte-layout guards: these MUST match `cerulion_core::wire`. A mismatch here
// is a local compile error; a mismatch against the source of truth is caught
// by `cerulion_core`'s lockstep test.
const _: () = {
    assert!(
        core::mem::size_of::<WireHeader>() == 32,
        "WireHeader must be 32 bytes"
    );
    assert!(
        core::mem::size_of::<OffsetEntry>() == 8,
        "OffsetEntry must be 8 bytes"
    );
    assert!(
        WireHeader::SIZE == 32,
        "WireHeader::SIZE must equal size_of::<WireHeader>()"
    );
    assert!(
        OffsetEntry::SIZE == 8,
        "OffsetEntry::SIZE must equal size_of::<OffsetEntry>()"
    );
};
