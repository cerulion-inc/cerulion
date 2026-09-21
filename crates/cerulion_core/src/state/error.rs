//! The one error type the capture/restore encoders return.
//!
//! Every variant names the condition in the operator's vocabulary, and no
//! variant is reachable by a path that would otherwise have been silent: a
//! byte the decoder cannot make sense of is an error, never a guess.

use thiserror::Error;

/// Failure of a state capture or restore.
///
/// Capture-side variants are dominated by [`StateError::SinkFull`], which is
/// the ordinary, expected outcome of an inline capture that outgrows the
/// boundary budget — it means "this node joins the fork set", not "this node
/// is broken". Every other variant describes a real defect: contended state,
/// corrupt bytes, or a value the target type cannot hold.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum StateError {
    /// The sink refused a write because it would pass its capacity.
    ///
    /// On the inline carrier this is the routine overflow signal: the partial
    /// bytes are discarded, the sink's consumed capacity is charged to the
    /// boundary budget, and the node joins the fork set.
    #[error("state sink is full: the capture exceeds the boundary budget")]
    SinkFull,

    /// The cursor ran out of bytes mid-decode.
    #[error("state blob truncated: needed {needed} more byte(s), {remaining} remain")]
    Truncated {
        /// Bytes the decoder asked for.
        needed: usize,
        /// Bytes actually left in the blob.
        remaining: usize,
    },

    /// Bytes were left over after a decode that should have consumed the blob.
    #[error("state blob has {remaining} trailing byte(s) after decode")]
    TrailingBytes {
        /// Bytes left unconsumed.
        remaining: usize,
    },

    /// A container length does not fit the 32-bit on-the-wire length prefix.
    #[error("container length {len} exceeds the u32 length prefix")]
    LengthOverflow {
        /// The offending length.
        len: usize,
    },

    /// A container declared more elements than the decode's remaining budget.
    ///
    /// Every container count draws from ONE budget seeded at the blob's length
    /// (see [`StateCursor::element_budget`](super::StateCursor::element_budget)),
    /// so a decode's total element iterations are bounded by its input. A count
    /// above what is left is either corruption or a hostile blob.
    ///
    /// The budget is per-decode rather than per-count because the same tail
    /// bytes can justify many counts: `Vec<Vec<()>>` makes a per-count check
    /// quadratic in blob size.
    #[error(
        "container declares {declared} element(s) but this decode has budget for \
         {budget}; the count is not justified by the blob"
    )]
    CountUnjustified {
        /// The count word read from the blob.
        declared: usize,
        /// Elements the decode could still afford.
        budget: usize,
    },

    /// A map or set blob repeated a key or element.
    ///
    /// The canonical encoder emits from a live map or set, so it can never
    /// produce a duplicate — a repeat is corruption or a hostile blob. Refused
    /// rather than collapsed, because `insert` silently keeping the last
    /// writer makes the restored container hold FEWER entries than the blob
    /// declared: MEASURED, a two-entry blob restored one entry and re-captured
    /// to 12 bytes against the 20 it was decoded from. The state would differ
    /// from the recording with nothing reporting it.
    #[error("state blob repeats a `{type_name}` key or element; a canonical encoding never does")]
    DuplicateEntry {
        /// The container type being decoded.
        type_name: &'static str,
    },

    /// A `Duration`'s subsecond field is at or above one second.
    ///
    /// Refused rather than clamped: clamping makes two distinct byte strings
    /// (`5 s + 1_500_000_000 ns` and `5 s + 999_999_999 ns`) decode to one
    /// value, so a restored checkpoint would **re-capture to different bytes**
    /// than it was restored from — the canonical-form property the whole
    /// recording rests on. It would also apply a wrong timestamp silently.
    #[error(
        "state blob holds a noncanonical `Duration`: {nanos} subsecond nanos \
         (must be < 1_000_000_000)"
    )]
    NoncanonicalDuration {
        /// The offending subsecond field.
        nanos: u32,
    },

    /// A `SystemTime` encodes the epoch itself with the "before" sign.
    ///
    /// The same canonical-form rule as [`StateError::NoncanonicalDuration`],
    /// one level up: the encoder emits the epoch as `+0` (its `duration_since`
    /// returns `Ok` for an equal instant), so a `-0` on the wire is a second
    /// byte string for one value and is refused.
    #[error(
        "state blob holds a noncanonical `SystemTime`: the epoch itself is \
         encoded with the `before` sign"
    )]
    NoncanonicalEpochSign,

    /// A `String`/`PathBuf` payload was not valid UTF-8.
    #[error("state blob contains invalid UTF-8 for a `{type_name}` field")]
    InvalidUtf8 {
        /// The type being decoded.
        type_name: &'static str,
    },

    /// A `char` slot held a value that is not a Unicode scalar.
    #[error("state blob contains invalid char scalar 0x{value:08x}")]
    InvalidChar {
        /// The offending code point.
        value: u32,
    },

    /// A `bool` slot held a byte other than 0 or 1.
    #[error("state blob contains invalid bool byte 0x{value:02x} (expected 0 or 1)")]
    InvalidBool {
        /// The offending byte.
        value: u8,
    },

    /// An enum-like discriminant byte was outside the known set.
    #[error("state blob contains invalid `{type_name}` tag {tag}")]
    InvalidTag {
        /// The type being decoded.
        type_name: &'static str,
        /// The offending tag byte.
        tag: u8,
    },

    /// A `NonZero*` slot held zero.
    #[error("state blob contains zero for a `{type_name}` field")]
    ZeroForNonZero {
        /// The `NonZero*` type being decoded.
        type_name: &'static str,
    },

    /// A value recorded on one target does not fit the running target's width.
    ///
    /// `usize`/`isize` are encoded at a fixed 64 bits precisely so a recording
    /// is portable; a 64-bit value restored onto a 32-bit target can still be
    /// out of range, and that is reported rather than truncated.
    #[error("state blob value {value} does not fit `{type_name}` on this target")]
    OutOfRange {
        /// The target type.
        type_name: &'static str,
        /// The recorded value.
        value: i128,
    },

    /// A declared `Mutex`/`RwLock` in the state graph was held by someone else.
    ///
    /// Captures use `try_lock` and never block, so a contended
    /// lock is a reported miss for that node — never a hang, and never a
    /// silently reconstructed field.
    #[error("`{type_name}` in the state graph is locked by another holder; capture skipped for this node")]
    LockContended {
        /// The lock type that was contended.
        type_name: &'static str,
    },

    /// A path could not be represented as UTF-8.
    ///
    /// Paths are encoded as UTF-8 so a recording is portable across targets
    /// whose `OsStr` representations differ; a non-UTF-8 path is refused
    /// loudly rather than silently lossily transcoded.
    #[error("path is not valid UTF-8 and cannot be captured portably")]
    NonUtf8Path,

    /// A field whose type carries no runtime-constructible value was asked to
    /// decode one.
    ///
    /// The only inventory member in this class is `&'static str`: its value
    /// can be captured, but a `&'static str` cannot be minted from recorded
    /// bytes. Such a field restores only by verification — see
    /// [`StateError::ImmutableMismatch`].
    #[error("`{type_name}` can be captured but not reconstructed from a recording")]
    Unrestorable {
        /// The type that cannot be reconstructed.
        type_name: &'static str,
    },

    /// A `#[cerulion(serde)]` field's own `Serialize` refused.
    ///
    /// Named separately from the inventory's errors because the failure is in
    /// USER code the framework only calls: the remedy is that impl, not the
    /// recording.
    #[error("`{field}` could not be encoded by its own `Serialize` impl: {cause}")]
    SerdeEncode {
        /// The field that refused.
        field: &'static str,
        /// What the underlying serializer said, carried verbatim.
        ///
        /// The field alone cannot tell an operator WHY: a map with non-string
        /// keys, a value outside JSON's number domain and a `Serialize` impl
        /// that returned a custom error all reach this variant, and only the
        /// serializer's own message separates them.
        cause: String,
    },

    /// A `#[cerulion(serde)]` field's own `Deserialize` refused.
    ///
    /// The likeliest cause is a TYPE CHANGE since the recording: a
    /// `#[cerulion(serde)]` field folds only its name and the escape marker
    /// into `STATE_SHAPE`, so a retype is not caught by the shape check and
    /// surfaces here instead.
    #[error("`{field}` could not be decoded by its own `Deserialize` impl: {cause}")]
    SerdeDecode {
        /// The field that refused.
        field: &'static str,
        /// What the underlying deserializer said, carried verbatim.
        ///
        /// This is the variant's whole diagnostic value, and dropping it made
        /// two DIFFERENT faults with two DIFFERENT remedies identical: bytes
        /// the recording never wrote correctly (a truncated or corrupted blob,
        /// which serde reports as a syntax or EOF failure) versus a well-formed
        /// blob the running type refuses (a retype since the recording, which
        /// serde reports as a data failure naming the offending key or type).
        /// One says the bag is bad, the other says the CODE moved.
        cause: String,
    },

    /// A verify-only field's recorded value differs from the running one.
    ///
    /// Reported instead of silently keeping the running value, because a
    /// differing value means the recording genuinely captured different state
    /// and a resim would otherwise attribute the divergence to node logic.
    #[error("`{type_name}` field cannot be restored: recorded value differs from the running one")]
    ImmutableMismatch {
        /// The verify-only type.
        type_name: &'static str,
    },

    /// A node's `NodeEntry::capture_state` refused, carrying
    /// whatever it said.
    ///
    /// The boundary walk (`state_carrier::walk_inline` — named in prose, not
    /// linked: that module is `#[cfg(unix)]` while this one is portable, and a
    /// link would break the docs gate on a non-unix target) sees nodes
    /// through an ERASED view whose `capture` returns this type, while
    /// `capture_state` returns a `TransportError` — a cdylib's refusal arrives
    /// as its own error slot's text, and a macro node's arrives already
    /// formatted with the node's diagnostic label. Neither is a `StateError`,
    /// and neither should be flattened into one of the variants above, which
    /// name specific ENCODING faults this is not.
    ///
    /// # Why the lossy direction is safe
    ///
    /// The walk's overflow-versus-error split does NOT rest on recognising
    /// [`StateError::SinkFull`] through this variant: it also consults the
    /// sink's own `refused()` flag, so a capture that overran its grant is
    /// classified as an overflow whatever error text came back. This variant
    /// therefore only has to carry the operator's diagnostic, not a decision.
    #[error("node state capture refused: {detail}")]
    NodeCapture {
        /// What the node's own capture path reported, verbatim.
        detail: String,
    },
}
