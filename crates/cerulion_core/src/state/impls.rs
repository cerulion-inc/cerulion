//! The closed blanket-impl inventory.
//!
//! "The user writes an ordinary Rust struct" is only true up to this list, so
//! the list is published rather than discovered field-by-field at compile
//! time. Every group has an impl here, and every refused type
//! has *no* impl here — refusal is expressed by the type system, never by a
//! runtime error.

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};
use std::hash::{BuildHasher, Hash};
use std::marker::PhantomData;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, SocketAddrV4, SocketAddrV6};
use std::num::{
    NonZeroI128, NonZeroI16, NonZeroI32, NonZeroI64, NonZeroI8, NonZeroIsize, NonZeroU128,
    NonZeroU16, NonZeroU32, NonZeroU64, NonZeroU8, NonZeroUsize, Wrapping,
};
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::atomic::{
    AtomicBool, AtomicI16, AtomicI32, AtomicI64, AtomicI8, AtomicIsize, AtomicU16, AtomicU32,
    AtomicU64, AtomicU8, AtomicUsize, Ordering as AtomicOrdering,
};
use std::sync::{Arc, Mutex, RwLock, TryLockError};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use indexmap::{IndexMap, IndexSet};

use super::{CanonicalMapKey, CerulionState, StateCursor, StateError, StateShape, StateSink};

/// Width of a container's count prefix.
const LEN_PREFIX_BYTES: usize = 4;

/// `min` over `usize` in const position (`core::cmp::min` is not `const`).
const fn min_usize(a: usize, b: usize) -> usize {
    if a < b {
        a
    } else {
        b
    }
}

/// One second, in nanoseconds — the canonical bound on a `Duration`'s
/// subsecond field.
const NANOS_PER_SEC: u32 = 1_000_000_000;

/// Upper bound on how much a container's decoder pre-allocates from a
/// blob-supplied count.
///
/// Mirrors `FrameWalker`'s `ELEMENT_PREALLOC_CAP`: a count word is
/// blob-supplied, so reserving from it directly turns a four-byte edit into a
/// multi-gigabyte allocation. Growth past this point is amortized doubling
/// paid only as real elements decode.
///
/// The cap is **output-equivalent** — a hostile count fails at the first
/// element read either way — so no verdict-shaped assertion can see it, and on
/// a machine that overcommits its removal is not even loud. It is pinned by
/// `crates/cerulion_core/tests/state_prealloc_budget_test.rs`, which measures
/// ALLOCATION and therefore needs a binary of its own.
const ELEMENT_PREALLOC_CAP: usize = 256;

// ---------------------------------------------------------------------------
// shared helpers
// ---------------------------------------------------------------------------

/// Write a container's `u32` LE count prefix.
fn write_len(out: &mut dyn StateSink, n: usize) -> Result<(), StateError> {
    let n32 = u32::try_from(n).map_err(|_| StateError::LengthOverflow { len: n })?;
    out.write(&n32.to_le_bytes())?;
    Ok(())
}

/// How much to reserve up front for `n` blob-declared elements.
fn prealloc(n: usize) -> usize {
    n.min(ELEMENT_PREALLOC_CAP)
}

/// Refuse a sort-index build the sink provably cannot accept.
///
/// The canonical order for a hash-like container needs a `Vec<&K>` index built
/// **before** the first byte is written. Without this check, capturing a
/// 30-million-entry `HashMap` into a 64 KiB sink would allocate a ~240 MB index
/// on the node thread and only then discover it cannot fit — which is exactly
/// the unbounded cost `BoundedSink` exists to prevent.
///
/// The bound comes from the element type's own encoded-size floor
/// ([`CerulionState::MIN_ENCODED_BYTES`]), not from a flat one byte per entry.
/// A flat charge refuses state that FITS: a one-entry `HashSet<()>` encodes as
/// nothing but its four-byte count, so `entries + 4 > 4` latched an exactly
/// sized sink as full. The floor is the true minimum, so the guard now fires
/// only when the encoding provably cannot fit, and a zero floor (which implies
/// a container that can hold at most one entry, and so needs no scratch bound
/// at all) simply never fires.
///
/// Under-charging is the safe direction: this pre-check is an OPTIMIZATION —
/// it skips a sort index for a capture that was going to be refused anyway —
/// so a missed refusal costs work, while a wrong refusal costs correctness.
/// That is why [`CerulionState::MIN_ENCODED_BYTES`] defaults to zero.
///
/// A zero floor alone would let a hand-written impl lose the bound entirely (a
/// 30-million-entry map would build its ~240 MB index on the node thread and
/// only then meet a 64 KiB sink), so the guard carries a SECOND, floor-free
/// disjunct: more entries than the sink has bytes left can never justify an
/// index. That is sound for every zero-encoding type in the inventory because
/// each is single-valued, so a container of them holds at most one entry and
/// needs at least four bytes for its count anyway.
///
/// The refusal calls [`StateSink::refuse`] so it **latches and charges exactly
/// like a mid-write refusal**. Returning `SinkFull` having called `write` zero
/// times would otherwise leave the sink unrefused, so `consumed()` would report
/// zero and the boundary walk would hand the same budget to the next node —
/// an uncharged path through the "at most one node per boundary pays for a
/// refusal" invariant.
fn guard_sort_scratch(
    out: &mut dyn StateSink,
    entries: usize,
    min_bytes_per_entry: usize,
) -> Result<(), StateError> {
    if let Some(remaining) = out.remaining_hint() {
        let output_floor = entries
            .saturating_mul(min_bytes_per_entry)
            .saturating_add(LEN_PREFIX_BYTES);
        // The SECOND disjunct is independent of any declared floor, which is
        // what keeps a hand-written impl's `MIN_ENCODED_BYTES = 0` from
        // reintroducing the unbounded index: an index costs eight bytes per
        // entry to answer a question about a sink that has fewer bytes left
        // than the container has entries, so it can never be justified.
        let scratch_unjustifiable = entries > remaining;
        if output_floor > remaining || scratch_unjustifiable {
            out.refuse();
            return Err(StateError::SinkFull);
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// scalars
// ---------------------------------------------------------------------------

macro_rules! impl_le_scalar {
    ($($ty:ty => $name:literal),* $(,)?) => {$(
        impl CerulionState for $ty {
            const STATE_SHAPE: u64 = StateShape::of($name).finish();
            const INLINE_SAFE: bool = true;
            const MIN_ENCODED_BYTES: usize = std::mem::size_of::<$ty>();

            fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
                out.write(&self.to_le_bytes())?;
                Ok(())
            }

            fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
                Ok(<$ty>::from_le_bytes(
                    src.take_array::<{ std::mem::size_of::<$ty>() }>()?,
                ))
            }
        }
    )*};
}

impl_le_scalar! {
    i8 => "i8", i16 => "i16", i32 => "i32", i64 => "i64", i128 => "i128",
    u8 => "u8", u16 => "u16", u32 => "u32", u64 => "u64", u128 => "u128",
    f32 => "f32", f64 => "f64",
}

/// `usize`/`isize` ride a **fixed 64-bit** encoding so a recording made on one
/// target restores on another. A value that does not fit the running target is
/// [`StateError::OutOfRange`] — reported, never truncated.
macro_rules! impl_pointer_width_int {
    ($($ty:ty => $name:literal, $wide:ty),* $(,)?) => {$(
        impl CerulionState for $ty {
            const STATE_SHAPE: u64 = StateShape::of($name).finish();
            const INLINE_SAFE: bool = true;
            const MIN_ENCODED_BYTES: usize = 8;

            fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
                out.write(&(*self as $wide).to_le_bytes())?;
                Ok(())
            }

            fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
                let wide = <$wide>::from_le_bytes(src.take_array::<8>()?);
                <$ty>::try_from(wide).map_err(|_| StateError::OutOfRange {
                    type_name: $name,
                    value: wide as i128,
                })
            }
        }
    )*};
}

impl_pointer_width_int! {
    usize => "usize", u64,
    isize => "isize", i64,
}

impl CerulionState for bool {
    const STATE_SHAPE: u64 = StateShape::of("bool").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 1;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        out.write(&[u8::from(*self)])?;
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        read_bool(src)
    }
}

impl CerulionState for char {
    const STATE_SHAPE: u64 = StateShape::of("char").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 4;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        out.write(&(*self as u32).to_le_bytes())?;
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let value = u32::from_le_bytes(src.take_array::<4>()?);
        char::from_u32(value).ok_or(StateError::InvalidChar { value })
    }
}

impl CerulionState for () {
    const STATE_SHAPE: u64 = StateShape::of("()").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 0;

    fn cer_capture(&self, _out: &mut dyn StateSink) -> Result<(), StateError> {
        Ok(())
    }

    fn cer_read(_src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(())
    }
}

impl<T: ?Sized> CerulionState for PhantomData<T> {
    const STATE_SHAPE: u64 = StateShape::of("PhantomData").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 0;

    fn cer_capture(&self, _out: &mut dyn StateSink) -> Result<(), StateError> {
        Ok(())
    }

    fn cer_read(_src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(PhantomData)
    }
}

macro_rules! impl_non_zero {
    ($($ty:ty => $name:literal, $inner:ty),* $(,)?) => {$(
        impl CerulionState for $ty {
            const STATE_SHAPE: u64 = StateShape::of($name)
                .element(<$inner as CerulionState>::STATE_SHAPE)
                .finish();
            const INLINE_SAFE: bool = true;
            // Read off the DELEGATE rather than `size_of`, because the encoding
            // IS the delegate's: `NonZeroUsize` rides `usize`'s fixed 64-bit
            // form, so a `size_of` floor would read 4 on a 32-bit target for a
            // value that always costs 8.
            const MIN_ENCODED_BYTES: usize = <$inner as CerulionState>::MIN_ENCODED_BYTES;

            fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
                self.get().cer_capture(out)
            }

            fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
                let inner = <$inner as CerulionState>::cer_read(src)?;
                <$ty>::new(inner).ok_or(StateError::ZeroForNonZero { type_name: $name })
            }
        }
    )*};
}

impl_non_zero! {
    NonZeroI8 => "NonZeroI8", i8,
    NonZeroI16 => "NonZeroI16", i16,
    NonZeroI32 => "NonZeroI32", i32,
    NonZeroI64 => "NonZeroI64", i64,
    NonZeroI128 => "NonZeroI128", i128,
    NonZeroIsize => "NonZeroIsize", isize,
    NonZeroU8 => "NonZeroU8", u8,
    NonZeroU16 => "NonZeroU16", u16,
    NonZeroU32 => "NonZeroU32", u32,
    NonZeroU64 => "NonZeroU64", u64,
    NonZeroU128 => "NonZeroU128", u128,
    NonZeroUsize => "NonZeroUsize", usize,
}

impl<T: CerulionState> CerulionState for Wrapping<T> {
    const STATE_SHAPE: u64 = StateShape::of("Wrapping").element(T::STATE_SHAPE).finish();
    const INLINE_SAFE: bool = T::INLINE_SAFE;
    const MIN_ENCODED_BYTES: usize = T::MIN_ENCODED_BYTES;

    fn cer_probe(&self) -> bool {
        if Self::INLINE_SAFE {
            return true;
        }
        self.0.cer_probe()
    }

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        self.0.cer_capture(out)
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(Wrapping(T::cer_read(src)?))
    }

    fn cer_restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
        self.0.cer_restore(src)
    }
}

// ---------------------------------------------------------------------------
// text / bytes
// ---------------------------------------------------------------------------

/// Encode a UTF-8 payload as `u32` LE length + bytes.
fn write_str(out: &mut dyn StateSink, s: &str) -> Result<(), StateError> {
    write_len(out, s.len())?;
    out.write(s.as_bytes())?;
    Ok(())
}

/// Decode a `u32` LE length-prefixed UTF-8 payload.
fn read_str<'a>(src: &mut StateCursor<'a>, type_name: &'static str) -> Result<&'a str, StateError> {
    // A PAYLOAD length, not an iteration count: the `take` below bounds it, so
    // it must NOT draw the element budget. Billing it there made a
    // `(String, Vec<()>)` capture unrestorable by its own decoder.
    let len = src.read_payload_len()?;
    let bytes = src.take(len)?;
    std::str::from_utf8(bytes).map_err(|_| StateError::InvalidUtf8 { type_name })
}

impl CerulionState for String {
    const STATE_SHAPE: u64 = StateShape::of("String").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        write_str(out, self)
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(read_str(src, "String")?.to_owned())
    }
}

impl CerulionState for PathBuf {
    const STATE_SHAPE: u64 = StateShape::of("PathBuf").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        let s = self.to_str().ok_or(StateError::NonUtf8Path)?;
        write_str(out, s)
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(PathBuf::from(read_str(src, "PathBuf")?))
    }
}

/// `&'static str` is **capture-only**: its value is recorded, but a
/// `&'static str` cannot be minted from recorded bytes, so restore is a
/// *verification*. A recorded value that differs from the running one is
/// [`StateError::ImmutableMismatch`] rather than a silently kept running value
/// — a difference means the recording genuinely held different state, and
/// swallowing it would let a resim blame node logic for a divergence the
/// restore manufactured.
impl CerulionState for &'static str {
    const STATE_SHAPE: u64 = StateShape::of("&'static str").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        write_str(out, self)
    }

    fn cer_read(_src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Err(StateError::Unrestorable {
            type_name: "&'static str",
        })
    }

    fn cer_restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
        let recorded = read_str(src, "&'static str")?;
        if recorded == *self {
            Ok(())
        } else {
            Err(StateError::ImmutableMismatch {
                type_name: "&'static str",
            })
        }
    }
}

impl CerulionState for Cow<'static, str> {
    const STATE_SHAPE: u64 = StateShape::of("Cow<str>").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        write_str(out, self)
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(Cow::Owned(read_str(src, "Cow<str>")?.to_owned()))
    }
}

impl<T: CerulionState + Clone> CerulionState for Cow<'static, [T]> {
    const STATE_SHAPE: u64 = StateShape::of("Cow<[T]>").element(T::STATE_SHAPE).finish();
    const INLINE_SAFE: bool = T::INLINE_SAFE;
    const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

    fn cer_probe(&self) -> bool {
        if Self::INLINE_SAFE {
            return true;
        }
        self.iter().all(CerulionState::cer_probe)
    }

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        write_len(out, self.len())?;
        for element in self.iter() {
            element.cer_capture(out)?;
        }
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(Cow::Owned(Vec::<T>::cer_read(src)?))
    }
}

// ---------------------------------------------------------------------------
// wrappers
// ---------------------------------------------------------------------------

/// `Box`/`Rc`/`Arc` are transparent derefs.
///
/// Shared-ownership **identity** is not preserved: two `Arc` fields pointing at
/// one allocation restore as two allocations. This follows the framework's
/// general rule — the value is restored, never the identity.
macro_rules! impl_transparent_ptr {
    ($($ty:ident => $name:literal),* $(,)?) => {$(
        impl<T: CerulionState> CerulionState for $ty<T> {
            const STATE_SHAPE: u64 = StateShape::of($name).element(T::STATE_SHAPE).finish();
            const INLINE_SAFE: bool = T::INLINE_SAFE;
            const MIN_ENCODED_BYTES: usize = T::MIN_ENCODED_BYTES;

            fn cer_probe(&self) -> bool {
                if Self::INLINE_SAFE {
                    return true;
                }
                (**self).cer_probe()
            }

            fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
                (**self).cer_capture(out)
            }

            fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
                Ok($ty::new(T::cer_read(src)?))
            }
        }
    )*};
}

impl_transparent_ptr! {
    Box => "Box",
    Rc => "Rc",
    Arc => "Arc",
}

impl<T: CerulionState> CerulionState for Option<T> {
    const STATE_SHAPE: u64 = StateShape::of("Option").element(T::STATE_SHAPE).finish();
    const INLINE_SAFE: bool = T::INLINE_SAFE;
    const MIN_ENCODED_BYTES: usize = 1;

    fn cer_probe(&self) -> bool {
        if Self::INLINE_SAFE {
            return true;
        }
        self.as_ref().is_none_or(CerulionState::cer_probe)
    }

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        match self {
            None => {
                out.write(&[0])?;
                Ok(())
            }
            Some(value) => {
                out.write(&[1])?;
                value.cer_capture(out)
            }
        }
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        match src.take_array::<1>()?[0] {
            0 => Ok(None),
            1 => Ok(Some(T::cer_read(src)?)),
            tag => Err(StateError::InvalidTag {
                type_name: "Option",
                tag,
            }),
        }
    }
}

impl<T: CerulionState, E: CerulionState> CerulionState for Result<T, E> {
    const STATE_SHAPE: u64 = StateShape::of("Result")
        .element(T::STATE_SHAPE)
        .element(E::STATE_SHAPE)
        .finish();
    const INLINE_SAFE: bool = T::INLINE_SAFE && E::INLINE_SAFE;
    const MIN_ENCODED_BYTES: usize =
        1usize.saturating_add(min_usize(T::MIN_ENCODED_BYTES, E::MIN_ENCODED_BYTES));

    fn cer_probe(&self) -> bool {
        if Self::INLINE_SAFE {
            return true;
        }
        match self {
            Ok(value) => value.cer_probe(),
            Err(error) => error.cer_probe(),
        }
    }

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        match self {
            Ok(value) => {
                out.write(&[0])?;
                value.cer_capture(out)
            }
            Err(error) => {
                out.write(&[1])?;
                error.cer_capture(out)
            }
        }
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        match src.take_array::<1>()?[0] {
            0 => Ok(Ok(T::cer_read(src)?)),
            1 => Ok(Err(E::cer_read(src)?)),
            tag => Err(StateError::InvalidTag {
                type_name: "Result",
                tag,
            }),
        }
    }
}

// ---------------------------------------------------------------------------
// locks — captured through `try_lock`, NEVER a blocking `lock()`
// ---------------------------------------------------------------------------

/// A **poisoned** lock is not a **contended** lock.
///
/// `try_lock` distinguishes them, and this module keeps the distinction:
/// `WouldBlock` is [`StateError::LockContended`] (a real, retryable miss),
/// while a poisoned-but-free lock yields its guard and the value is captured
/// normally. Treating poisoning as contention would permanently disable
/// anchors for a node whose helper thread once panicked, and would report the
/// wrong cause while doing it.
impl<T: CerulionState> CerulionState for Mutex<T> {
    const STATE_SHAPE: u64 = StateShape::of("Mutex").element(T::STATE_SHAPE).finish();
    /// **Always `false`**, regardless of the inner type: a lock is exactly
    /// what the inline carrier must never touch on the node thread.
    const INLINE_SAFE: bool = false;
    const MIN_ENCODED_BYTES: usize = T::MIN_ENCODED_BYTES;

    fn cer_probe(&self) -> bool {
        match self.try_lock() {
            Ok(guard) => guard.cer_probe(),
            Err(TryLockError::Poisoned(poison)) => poison.into_inner().cer_probe(),
            Err(TryLockError::WouldBlock) => false,
        }
    }

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        match self.try_lock() {
            Ok(guard) => guard.cer_capture(out),
            Err(TryLockError::Poisoned(poison)) => poison.into_inner().cer_capture(out),
            Err(TryLockError::WouldBlock) => Err(StateError::LockContended { type_name: "Mutex" }),
        }
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(Mutex::new(T::cer_read(src)?))
    }

    fn cer_restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
        // `&mut self` proves exclusivity, so this cannot contend — and it
        // preserves the lock's identity rather than minting a new one.
        match self.get_mut() {
            Ok(inner) => inner.cer_restore(src),
            Err(poison) => poison.into_inner().cer_restore(src),
        }
    }
}

impl<T: CerulionState> CerulionState for RwLock<T> {
    const STATE_SHAPE: u64 = StateShape::of("RwLock").element(T::STATE_SHAPE).finish();
    /// **Always `false`** — see [`Mutex`]'s note.
    const INLINE_SAFE: bool = false;
    const MIN_ENCODED_BYTES: usize = T::MIN_ENCODED_BYTES;

    fn cer_probe(&self) -> bool {
        match self.try_read() {
            Ok(guard) => guard.cer_probe(),
            Err(TryLockError::Poisoned(poison)) => poison.into_inner().cer_probe(),
            Err(TryLockError::WouldBlock) => false,
        }
    }

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        match self.try_read() {
            Ok(guard) => guard.cer_capture(out),
            Err(TryLockError::Poisoned(poison)) => poison.into_inner().cer_capture(out),
            Err(TryLockError::WouldBlock) => Err(StateError::LockContended {
                type_name: "RwLock",
            }),
        }
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(RwLock::new(T::cer_read(src)?))
    }

    fn cer_restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
        match self.get_mut() {
            Ok(inner) => inner.cer_restore(src),
            Err(poison) => poison.into_inner().cer_restore(src),
        }
    }
}

// ---------------------------------------------------------------------------
// atomics — captured BY VALUE at the step boundary
// ---------------------------------------------------------------------------

/// An atomic is captured by **loading its value**, and that is sound at a step
/// boundary for a reason worth writing down.
///
/// A single atomic load is **never torn** — it is one indivisible machine
/// operation on every target Rust supports these types on, which is the whole
/// point of the type. So the value this reads is a value the counter really
/// held; it is never a half-written mixture of two writes the way a plain
/// `u64` written by another thread could be.
///
/// What it does NOT promise is *which* value. A helper thread incrementing the
/// counter concurrently means the capture sees either the old or the new one —
/// a valid reading either side of an instant the boundary cannot pin down more
/// precisely anyway. That is exactly the guarantee a counter wants and it is
/// why these rows are [`INLINE_SAFE`]: reading one takes **no lock**, so it
/// cannot block the node thread, which is the property the inline carrier's
/// byte bound rests on.
///
/// [`Ordering::Relaxed`] throughout, deliberately. The capture publishes
/// nothing and synchronizes nothing — it reads one number and writes it to a
/// sink — so a stronger ordering would buy no additional correctness while
/// costing a fence on the node thread at every boundary.
///
/// Restore goes through [`get_mut`](std::sync::atomic::AtomicU64::get_mut),
/// not `store`: `cer_restore` takes `&mut self`, which *proves* exclusive
/// access, so no atomic operation is needed at all.
///
/// # The `Arc<Atomic…>` residual, stated because this is where it bites
///
/// `Arc<T>` restores by **replacing** the allocation (see the transparent-ptr
/// note above — shared-ownership identity is not preserved). So restoring a
/// node's `Arc<AtomicU64>` leaves a helper thread that was handed a clone in
/// `init()` pointing at the OLD allocation, and the two silently diverge from
/// then on. That is inherited from `Arc`, not introduced here, but
/// `Arc<AtomicU64>` is by far the most common shape carrying an atomic, so the
/// warning belongs where a reader will meet it.
///
/// # Pointer-width atomics are NOT here
///
/// `AtomicUsize`/`AtomicIsize` are spelled out separately by
/// `impl_pointer_width_atomic!` below: this macro's `size_of` framing is the
/// TARGET's pointer width, which would put a target-dependent number of bytes
/// behind a target-independent `STATE_SHAPE`. Everything above applies to them
/// unchanged — only the framing differs.
///
/// [`INLINE_SAFE`]: CerulionState::INLINE_SAFE
macro_rules! impl_atomic {
    ($($ty:ty => $prim:ty, $name:literal),* $(,)?) => {$(
        impl CerulionState for $ty {
            const STATE_SHAPE: u64 = StateShape::of($name).finish();
            const INLINE_SAFE: bool = true;
            const MIN_ENCODED_BYTES: usize = std::mem::size_of::<$prim>();

            fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
                out.write(&self.load(AtomicOrdering::Relaxed).to_le_bytes())?;
                Ok(())
            }

            fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
                Ok(Self::new(<$prim>::from_le_bytes(
                    src.take_array::<{ std::mem::size_of::<$prim>() }>()?,
                )))
            }

            fn cer_restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
                // `&mut self` proves exclusivity, so no atomic op is needed.
                *self.get_mut() = <$prim>::from_le_bytes(
                    src.take_array::<{ std::mem::size_of::<$prim>() }>()?,
                );
                Ok(())
            }
        }
    )*};
}

impl_atomic! {
    AtomicI8 => i8, "AtomicI8",
    AtomicI16 => i16, "AtomicI16",
    AtomicI32 => i32, "AtomicI32",
    AtomicI64 => i64, "AtomicI64",
    AtomicU8 => u8, "AtomicU8",
    AtomicU16 => u16, "AtomicU16",
    AtomicU32 => u32, "AtomicU32",
    AtomicU64 => u64, "AtomicU64",
}

/// [`AtomicUsize`]/[`AtomicIsize`] ride the SAME fixed 64-bit encoding as
/// [`usize`]/[`isize`] — never `size_of`, which is the target's pointer width.
///
/// This is not symmetry for its own sake. `STATE_SHAPE` folds the type's NAME,
/// which is target-INDEPENDENT, so a pointer-width encoding would let a 32-bit
/// recording's shape check PASS on a 64-bit restore and then mis-frame every
/// byte after the atomic: the decoder consumes 8 where the writer wrote 4, so
/// the atomic absorbs the following field's bytes and the decode fails later
/// (or, in the other direction, leaves the tail shifted). Nothing on the wire
/// distinguishes the two, and the desk that replays is by design not the
/// machine that recorded (see `state_restore.rs`), so the skew is reachable.
///
/// A value the running target cannot hold is [`StateError::OutOfRange`] —
/// reported, never truncated — exactly as for the plain integers.
macro_rules! impl_pointer_width_atomic {
    ($($ty:ty => $prim:ty, $name:literal, $wide:ty),* $(,)?) => {$(
        impl CerulionState for $ty {
            const STATE_SHAPE: u64 = StateShape::of($name).finish();
            const INLINE_SAFE: bool = true;
            const MIN_ENCODED_BYTES: usize = 8;

            fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
                out.write(&(self.load(AtomicOrdering::Relaxed) as $wide).to_le_bytes())?;
                Ok(())
            }

            fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
                Ok(Self::new(read_pointer_width::<$prim, $wide>(src, $name)?))
            }

            fn cer_restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
                // Decode BEFORE the store: an out-of-range value must leave the
                // running atomic untouched rather than half-restored.
                let value = read_pointer_width::<$prim, $wide>(src, $name)?;
                // `&mut self` proves exclusivity, so no atomic op is needed.
                *self.get_mut() = value;
                Ok(())
            }
        }
    )*};
}

impl_pointer_width_atomic! {
    AtomicIsize => isize, "AtomicIsize", i64,
    AtomicUsize => usize, "AtomicUsize", u64,
}

// The one assertion that BITES on the target where the defect is observable.
// On a 64-bit host a pointer-width encoding and the portable one are
// byte-identical, so no test running here can tell them apart; compiled for a
// 32-bit target, a `size_of`-framed encoder declares a 4-byte floor and this
// stops the build instead of shipping a bag a 64-bit desk mis-frames.
const _: () = assert!(
    <AtomicUsize as CerulionState>::MIN_ENCODED_BYTES == 8
        && <AtomicIsize as CerulionState>::MIN_ENCODED_BYTES == 8,
    "pointer-width atomics must ride the portable 64-bit form on every target"
);

/// Decode one fixed 64-bit little-endian integer into a pointer-width type,
/// reporting a value the running target cannot hold rather than truncating it.
///
/// Shared by [`AtomicIsize`]/[`AtomicUsize`] so the two cannot disagree with
/// each other about the framing, and written to the same rule as
/// `impl_pointer_width_int!` so the atomic and the plain integer cannot
/// disagree either.
pub(super) fn read_pointer_width<T, W>(
    src: &mut StateCursor<'_>,
    type_name: &'static str,
) -> Result<T, StateError>
where
    T: TryFrom<W>,
    W: FromLeBytes + Into<i128>,
{
    let wide = W::from_le_bytes_8(src.take_array::<8>()?);
    T::try_from(wide).map_err(|_| StateError::OutOfRange {
        type_name,
        value: wide.into(),
    })
}

/// The 8-byte little-endian decode the two pointer-width atomics share.
///
/// A trait rather than a second macro arm because [`read_pointer_width`] is one
/// function over both signednesses, and `from_le_bytes` is an inherent method
/// no std trait exposes.
pub(super) trait FromLeBytes: Copy {
    fn from_le_bytes_8(bytes: [u8; 8]) -> Self;
}

impl FromLeBytes for i64 {
    fn from_le_bytes_8(bytes: [u8; 8]) -> Self {
        i64::from_le_bytes(bytes)
    }
}

impl FromLeBytes for u64 {
    fn from_le_bytes_8(bytes: [u8; 8]) -> Self {
        u64::from_le_bytes(bytes)
    }
}

/// `AtomicBool` rides the same one-byte `0`/`1` encoding as `bool`, so a
/// non-canonical byte is [`StateError::InvalidBool`] rather than a silent
/// coercion to `true`.
///
/// It is spelled out rather than folded into `impl_atomic!` because `bool`
/// has no `to_le_bytes`/`from_le_bytes` and its decode is validating.
impl CerulionState for AtomicBool {
    const STATE_SHAPE: u64 = StateShape::of("AtomicBool").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 1;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        out.write(&[u8::from(self.load(AtomicOrdering::Relaxed))])?;
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(AtomicBool::new(read_bool(src)?))
    }

    fn cer_restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
        *self.get_mut() = read_bool(src)?;
        Ok(())
    }
}

/// Decode one canonical `0`/`1` byte, shared by `bool` and [`AtomicBool`] so
/// the two can never disagree about what a non-canonical byte means.
fn read_bool(src: &mut StateCursor<'_>) -> Result<bool, StateError> {
    match src.take_array::<1>()?[0] {
        0 => Ok(false),
        1 => Ok(true),
        value => Err(StateError::InvalidBool { value }),
    }
}

// ---------------------------------------------------------------------------
// sequences
// ---------------------------------------------------------------------------

macro_rules! impl_linear_sequence {
    ($($ty:ident => $name:literal),* $(,)?) => {$(
        impl<T: CerulionState> CerulionState for $ty<T> {
            const STATE_SHAPE: u64 = StateShape::of($name).element(T::STATE_SHAPE).finish();
            const INLINE_SAFE: bool = T::INLINE_SAFE;
            const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

            fn cer_probe(&self) -> bool {
                if Self::INLINE_SAFE {
                    return true;
                }
                self.iter().all(CerulionState::cer_probe)
            }

            fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
                write_len(out, self.len())?;
                for element in self.iter() {
                    element.cer_capture(out)?;
                }
                Ok(())
            }

            fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
                let len = src.read_element_count()?;
                let mut out = $ty::with_capacity(prealloc(len));
                for _ in 0..len {
                    out.push_back_compat(T::cer_read(src)?);
                }
                Ok(out)
            }
        }
    )*};
}

/// One name for "append" across `Vec` and `VecDeque`, so the sequence impl is
/// written once.
trait PushBackCompat<T> {
    fn push_back_compat(&mut self, value: T);
}

impl<T> PushBackCompat<T> for Vec<T> {
    fn push_back_compat(&mut self, value: T) {
        self.push(value);
    }
}

impl<T> PushBackCompat<T> for VecDeque<T> {
    fn push_back_compat(&mut self, value: T) {
        self.push_back(value);
    }
}

impl_linear_sequence! {
    Vec => "Vec",
    VecDeque => "VecDeque",
}

impl<T: CerulionState, const N: usize> CerulionState for [T; N] {
    const STATE_SHAPE: u64 = StateShape::of("[T; N]")
        .count(N)
        .element(T::STATE_SHAPE)
        .finish();
    const INLINE_SAFE: bool = T::INLINE_SAFE;
    const MIN_ENCODED_BYTES: usize = N.saturating_mul(T::MIN_ENCODED_BYTES);

    fn cer_probe(&self) -> bool {
        if Self::INLINE_SAFE {
            return true;
        }
        self.iter().all(CerulionState::cer_probe)
    }

    /// No count prefix: `N` is structural and already folded into the shape.
    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        for element in self.iter() {
            element.cer_capture(out)?;
        }
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let mut decoded = Vec::with_capacity(N);
        for _ in 0..N {
            decoded.push(T::cer_read(src)?);
        }
        match <[T; N]>::try_from(decoded) {
            Ok(array) => Ok(array),
            // Unreachable: exactly `N` elements were pushed.
            Err(_) => Err(StateError::Truncated {
                needed: N,
                remaining: 0,
            }),
        }
    }

    fn cer_restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
        for element in self.iter_mut() {
            element.cer_restore(src)?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// tuples, to arity 12
// ---------------------------------------------------------------------------

macro_rules! impl_tuple {
    ($arity:literal; $($name:ident $idx:tt),+) => {
        impl<$($name: CerulionState),+> CerulionState for ($($name,)+) {
            const STATE_SHAPE: u64 = StateShape::of("tuple")
                .count($arity)
                $(.element($name::STATE_SHAPE))+
                .finish();
            const INLINE_SAFE: bool = true $(&& $name::INLINE_SAFE)+;
            const MIN_ENCODED_BYTES: usize = 0usize $(.saturating_add($name::MIN_ENCODED_BYTES))+;

            fn cer_probe(&self) -> bool {
                if Self::INLINE_SAFE {
                    return true;
                }
                true $(&& self.$idx.cer_probe())+
            }

            fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
                $(self.$idx.cer_capture(out)?;)+
                Ok(())
            }

            fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
                Ok(($($name::cer_read(src)?,)+))
            }

            fn cer_restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
                $(self.$idx.cer_restore(src)?;)+
                Ok(())
            }
        }
    };
}

impl_tuple!(1; A 0);
impl_tuple!(2; A 0, B 1);
impl_tuple!(3; A 0, B 1, C 2);
impl_tuple!(4; A 0, B 1, C 2, D 3);
impl_tuple!(5; A 0, B 1, C 2, D 3, E 4);
impl_tuple!(6; A 0, B 1, C 2, D 3, E 4, F 5);
impl_tuple!(7; A 0, B 1, C 2, D 3, E 4, F 5, G 6);
impl_tuple!(8; A 0, B 1, C 2, D 3, E 4, F 5, G 6, H 7);
impl_tuple!(9; A 0, B 1, C 2, D 3, E 4, F 5, G 6, H 7, I 8);
impl_tuple!(10; A 0, B 1, C 2, D 3, E 4, F 5, G 6, H 7, I 8, J 9);
impl_tuple!(11; A 0, B 1, C 2, D 3, E 4, F 5, G 6, H 7, I 8, J 9, K 10);
impl_tuple!(12; A 0, B 1, C 2, D 3, E 4, F 5, G 6, H 7, I 8, J 9, K 10, L 11);

// ---------------------------------------------------------------------------
// maps and sets
// ---------------------------------------------------------------------------

impl<K, V, S> CerulionState for HashMap<K, V, S>
where
    K: CerulionState + CanonicalMapKey + Hash + Eq,
    V: CerulionState,
    S: BuildHasher + Default,
{
    const STATE_SHAPE: u64 = StateShape::of("HashMap")
        .element(K::STATE_SHAPE)
        .element(V::STATE_SHAPE)
        .finish();
    const INLINE_SAFE: bool = K::INLINE_SAFE && V::INLINE_SAFE;
    const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

    fn cer_probe(&self) -> bool {
        if Self::INLINE_SAFE {
            return true;
        }
        self.iter()
            .all(|(key, value)| key.cer_probe() && value.cer_probe())
    }

    /// Emits in **sorted key order**, so a `HashMap`'s per-process
    /// `RandomState` iteration order never reaches the bag.
    ///
    /// The sort index is a `Vec<&K>` — 8 bytes per entry — rather than a
    /// buffer of encoded keys, which would land at roughly the size of the
    /// state itself.
    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        guard_sort_scratch(
            out,
            self.len(),
            K::MIN_ENCODED_BYTES.saturating_add(V::MIN_ENCODED_BYTES),
        )?;
        write_len(out, self.len())?;
        let mut keys: Vec<&K> = self.keys().collect();
        keys.sort_unstable_by(|a, b| a.canonical_cmp(b));
        for key in keys {
            key.cer_capture(out)?;
            let value = self.get(key).expect("key came from this map's own keys()");
            value.cer_capture(out)?;
        }
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let len = src.read_element_count()?;
        let mut out = HashMap::with_capacity_and_hasher(prealloc(len), S::default());
        for _ in 0..len {
            let key = K::cer_read(src)?;
            let value = V::cer_read(src)?;
            if out.insert(key, value).is_some() {
                return Err(StateError::DuplicateEntry {
                    type_name: "HashMap",
                });
            }
        }
        Ok(out)
    }
}

impl<T, S> CerulionState for HashSet<T, S>
where
    T: CerulionState + CanonicalMapKey + Hash + Eq,
    S: BuildHasher + Default,
{
    const STATE_SHAPE: u64 = StateShape::of("HashSet").element(T::STATE_SHAPE).finish();
    const INLINE_SAFE: bool = T::INLINE_SAFE;
    const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

    fn cer_probe(&self) -> bool {
        if Self::INLINE_SAFE {
            return true;
        }
        self.iter().all(CerulionState::cer_probe)
    }

    /// Emits in sorted order — see [`HashMap`]'s note.
    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        guard_sort_scratch(out, self.len(), T::MIN_ENCODED_BYTES)?;
        write_len(out, self.len())?;
        let mut elements: Vec<&T> = self.iter().collect();
        elements.sort_unstable_by(|a, b| a.canonical_cmp(b));
        for element in elements {
            element.cer_capture(out)?;
        }
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let len = src.read_element_count()?;
        let mut out = HashSet::with_capacity_and_hasher(prealloc(len), S::default());
        for _ in 0..len {
            if !out.insert(T::cer_read(src)?) {
                return Err(StateError::DuplicateEntry {
                    type_name: "HashSet",
                });
            }
        }
        Ok(out)
    }
}

impl<K: CerulionState + Ord, V: CerulionState> CerulionState for BTreeMap<K, V> {
    const STATE_SHAPE: u64 = StateShape::of("BTreeMap")
        .element(K::STATE_SHAPE)
        .element(V::STATE_SHAPE)
        .finish();
    const INLINE_SAFE: bool = K::INLINE_SAFE && V::INLINE_SAFE;
    const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

    fn cer_probe(&self) -> bool {
        if Self::INLINE_SAFE {
            return true;
        }
        self.iter()
            .all(|(key, value)| key.cer_probe() && value.cer_probe())
    }

    /// Already ordered — it sorts nothing and allocates no index.
    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        write_len(out, self.len())?;
        for (key, value) in self.iter() {
            key.cer_capture(out)?;
            value.cer_capture(out)?;
        }
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let len = src.read_element_count()?;
        let mut out = BTreeMap::new();
        for _ in 0..len {
            let key = K::cer_read(src)?;
            let value = V::cer_read(src)?;
            if out.insert(key, value).is_some() {
                return Err(StateError::DuplicateEntry {
                    type_name: "BTreeMap",
                });
            }
        }
        Ok(out)
    }
}

impl<T: CerulionState + Ord> CerulionState for BTreeSet<T> {
    const STATE_SHAPE: u64 = StateShape::of("BTreeSet").element(T::STATE_SHAPE).finish();
    const INLINE_SAFE: bool = T::INLINE_SAFE;
    const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

    fn cer_probe(&self) -> bool {
        if Self::INLINE_SAFE {
            return true;
        }
        self.iter().all(CerulionState::cer_probe)
    }

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        write_len(out, self.len())?;
        for element in self.iter() {
            element.cer_capture(out)?;
        }
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let len = src.read_element_count()?;
        let mut out = BTreeSet::new();
        for _ in 0..len {
            if !out.insert(T::cer_read(src)?) {
                return Err(StateError::DuplicateEntry {
                    type_name: "BTreeSet",
                });
            }
        }
        Ok(out)
    }
}

/// `IndexMap` is **insertion-ordered**, so it emits in its own order and
/// carries no `Ord` bound — it sorts nothing and pays no index.
impl<K, V, S> CerulionState for IndexMap<K, V, S>
where
    K: CerulionState + Hash + Eq,
    V: CerulionState,
    S: BuildHasher + Default,
{
    const STATE_SHAPE: u64 = StateShape::of("IndexMap")
        .element(K::STATE_SHAPE)
        .element(V::STATE_SHAPE)
        .finish();
    const INLINE_SAFE: bool = K::INLINE_SAFE && V::INLINE_SAFE;
    const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

    fn cer_probe(&self) -> bool {
        if Self::INLINE_SAFE {
            return true;
        }
        self.iter()
            .all(|(key, value)| key.cer_probe() && value.cer_probe())
    }

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        write_len(out, self.len())?;
        for (key, value) in self.iter() {
            key.cer_capture(out)?;
            value.cer_capture(out)?;
        }
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let len = src.read_element_count()?;
        let mut out = IndexMap::with_capacity_and_hasher(prealloc(len), S::default());
        for _ in 0..len {
            let key = K::cer_read(src)?;
            let value = V::cer_read(src)?;
            if out.insert(key, value).is_some() {
                return Err(StateError::DuplicateEntry {
                    type_name: "IndexMap",
                });
            }
        }
        Ok(out)
    }
}

/// `IndexSet` is insertion-ordered — see [`IndexMap`]'s note.
impl<T, S> CerulionState for IndexSet<T, S>
where
    T: CerulionState + Hash + Eq,
    S: BuildHasher + Default,
{
    const STATE_SHAPE: u64 = StateShape::of("IndexSet").element(T::STATE_SHAPE).finish();
    const INLINE_SAFE: bool = T::INLINE_SAFE;
    const MIN_ENCODED_BYTES: usize = LEN_PREFIX_BYTES;

    fn cer_probe(&self) -> bool {
        if Self::INLINE_SAFE {
            return true;
        }
        self.iter().all(CerulionState::cer_probe)
    }

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        write_len(out, self.len())?;
        for element in self.iter() {
            element.cer_capture(out)?;
        }
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let len = src.read_element_count()?;
        let mut out = IndexSet::with_capacity_and_hasher(prealloc(len), S::default());
        for _ in 0..len {
            if !out.insert(T::cer_read(src)?) {
                return Err(StateError::DuplicateEntry {
                    type_name: "IndexSet",
                });
            }
        }
        Ok(out)
    }
}

// ---------------------------------------------------------------------------
// `#[cerulion(unordered)]` — the landing point the derive emits calls to
// ---------------------------------------------------------------------------

/// Encode a map's entries in **iteration order**, skipping the canonical sort.
///
/// The escape hatch behind `#[cerulion(unordered)]`: a `HashMap` keyed by
/// something that cannot be `Ord` (a float grid coordinate, a pose-keyed
/// cache) is still **captured and restored** — only its byte-stability across
/// runs is given up, and the manifest records `order: unordered` for that
/// field so the loss is visible rather than inferred.
///
/// The bytes are otherwise identical to the ordered form, so a decoder needs
/// no knowledge of which form produced them.
pub fn capture_unordered_map<'a, K, V, I>(
    len: usize,
    entries: I,
    out: &mut dyn StateSink,
) -> Result<(), StateError>
where
    K: CerulionState + 'a,
    V: CerulionState + 'a,
    I: IntoIterator<Item = (&'a K, &'a V)>,
{
    write_len(out, len)?;
    for (key, value) in entries {
        key.cer_capture(out)?;
        value.cer_capture(out)?;
    }
    Ok(())
}

/// Decode the entries of a map written by [`capture_unordered_map`] (or by the
/// ordered path — the byte forms are the same).
///
/// Returns the entries rather than a container so the caller names the
/// container type: `Extend` is implemented twice over for `IndexMap` (owned and
/// by-reference), which makes a container-generic signature ambiguous at every
/// call site. The cost is one transient `Vec` on an escape-hatch restore path,
/// which already builds the whole container anyway.
///
/// # Duplicates are the CALLER's to refuse
///
/// `K` carries no `Eq`/`Hash`/`Ord` bound here, so this reader cannot detect a
/// repeated key the way the container decoders do
/// ([`StateError::DuplicateEntry`]). Whatever collects these entries owns that
/// check — see the unordered-readers residual in the [`crate::state`] module docs.
pub fn read_unordered_entries<K, V>(src: &mut StateCursor<'_>) -> Result<Vec<(K, V)>, StateError>
where
    K: CerulionState,
    V: CerulionState,
{
    let len = src.read_element_count()?;
    let mut out = Vec::with_capacity(prealloc(len));
    for _ in 0..len {
        let key = K::cer_read(src)?;
        let value = V::cer_read(src)?;
        out.push((key, value));
    }
    Ok(out)
}

/// Probe a map captured by [`capture_unordered_map`].
///
/// The derive emits this only when the key/value types are NOT `INLINE_SAFE`
/// — i.e. only when a lock really is reachable — so the O(n) walk never runs
/// on the lock-free node the short circuit exists for.
pub fn probe_unordered_map<'a, K, V, I>(entries: I) -> bool
where
    K: CerulionState + 'a,
    V: CerulionState + 'a,
    I: IntoIterator<Item = (&'a K, &'a V)>,
{
    entries
        .into_iter()
        .all(|(key, value)| key.cer_probe() && value.cer_probe())
}

/// Probe a set captured by [`capture_unordered_set`] — see
/// [`probe_unordered_map`].
pub fn probe_unordered_set<'a, T, I>(elements: I) -> bool
where
    T: CerulionState + 'a,
    I: IntoIterator<Item = &'a T>,
{
    elements.into_iter().all(CerulionState::cer_probe)
}

/// Encode a set's elements in iteration order — see [`capture_unordered_map`].
pub fn capture_unordered_set<'a, T, I>(
    len: usize,
    elements: I,
    out: &mut dyn StateSink,
) -> Result<(), StateError>
where
    T: CerulionState + 'a,
    I: IntoIterator<Item = &'a T>,
{
    write_len(out, len)?;
    for element in elements {
        element.cer_capture(out)?;
    }
    Ok(())
}

/// Decode the elements of a set written by [`capture_unordered_set`].
///
/// Returns the elements rather than a container, for the same reason as
/// [`read_unordered_entries`].
pub fn read_unordered_elements<T: CerulionState>(
    src: &mut StateCursor<'_>,
) -> Result<Vec<T>, StateError> {
    let len = src.read_element_count()?;
    let mut out = Vec::with_capacity(prealloc(len));
    for _ in 0..len {
        out.push(T::cer_read(src)?);
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// std misc
// ---------------------------------------------------------------------------

impl CerulionState for Duration {
    const STATE_SHAPE: u64 = StateShape::of("Duration").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 12;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        out.write(&self.as_secs().to_le_bytes())?;
        out.write(&self.subsec_nanos().to_le_bytes())?;
        Ok(())
    }

    /// A subsecond field at or above one second is **refused, never clamped**.
    ///
    /// Clamping would make `5 s + 1_500_000_000 ns` and `5 s + 999_999_999 ns`
    /// decode to one value, so a checkpoint restored from the first would
    /// **re-capture to the second's bytes** — the canonical-form property the
    /// recording rests on, broken silently, on a value the operator would then
    /// see as a wrong timestamp rather than as corruption.
    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let secs = u64::from_le_bytes(src.take_array::<8>()?);
        let nanos = u32::from_le_bytes(src.take_array::<4>()?);
        if nanos >= NANOS_PER_SEC {
            return Err(StateError::NoncanonicalDuration { nanos });
        }
        // Cannot panic: `Duration::new` only overflows on a carry out of the
        // nanos field, and the guard above proves there is none.
        Ok(Duration::new(secs, nanos))
    }
}

/// Encoded relative to the Unix epoch, with a sign tag, so it is meaningful in
/// another process — unlike `Instant`, which is refused for exactly that
/// reason.
impl CerulionState for SystemTime {
    const STATE_SHAPE: u64 = StateShape::of("SystemTime").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 13;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        let (tag, delta) = match self.duration_since(UNIX_EPOCH) {
            Ok(delta) => (0u8, delta),
            Err(before) => (1u8, before.duration()),
        };
        out.write(&[tag])?;
        delta.cer_capture(out)
    }

    /// A `-0` is refused, for the same reason a noncanonical `Duration` is.
    ///
    /// The encoder emits the epoch itself as `+0` — `duration_since` returns
    /// `Ok` for an equal instant — so a `before` tag over a zero delta is a
    /// SECOND byte string for one value, and accepting it would break the
    /// same restore-then-re-capture identity. Found by asking what else in
    /// this module carried the `Duration` defect's shape.
    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let tag = src.take_array::<1>()?[0];
        let delta = Duration::cer_read(src)?;
        match tag {
            0 => UNIX_EPOCH.checked_add(delta).ok_or(StateError::OutOfRange {
                type_name: "SystemTime",
                value: delta.as_secs() as i128,
            }),
            1 if delta.is_zero() => Err(StateError::NoncanonicalEpochSign),
            1 => UNIX_EPOCH.checked_sub(delta).ok_or(StateError::OutOfRange {
                type_name: "SystemTime",
                value: -(delta.as_secs() as i128),
            }),
            tag => Err(StateError::InvalidTag {
                type_name: "SystemTime",
                tag,
            }),
        }
    }
}

impl CerulionState for Ipv4Addr {
    const STATE_SHAPE: u64 = StateShape::of("Ipv4Addr").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 4;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        out.write(&self.octets())?;
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(Ipv4Addr::from(src.take_array::<4>()?))
    }
}

impl CerulionState for Ipv6Addr {
    const STATE_SHAPE: u64 = StateShape::of("Ipv6Addr").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 16;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        out.write(&self.octets())?;
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(Ipv6Addr::from(src.take_array::<16>()?))
    }
}

impl CerulionState for IpAddr {
    const STATE_SHAPE: u64 = StateShape::of("IpAddr")
        .element(<Ipv4Addr as CerulionState>::STATE_SHAPE)
        .element(<Ipv6Addr as CerulionState>::STATE_SHAPE)
        .finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 5;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        match self {
            IpAddr::V4(addr) => {
                out.write(&[4])?;
                addr.cer_capture(out)
            }
            IpAddr::V6(addr) => {
                out.write(&[6])?;
                addr.cer_capture(out)
            }
        }
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        match src.take_array::<1>()?[0] {
            4 => Ok(IpAddr::V4(Ipv4Addr::cer_read(src)?)),
            6 => Ok(IpAddr::V6(Ipv6Addr::cer_read(src)?)),
            tag => Err(StateError::InvalidTag {
                type_name: "IpAddr",
                tag,
            }),
        }
    }
}

impl CerulionState for SocketAddrV4 {
    const STATE_SHAPE: u64 = StateShape::of("SocketAddrV4")
        .element(<Ipv4Addr as CerulionState>::STATE_SHAPE)
        .finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 6;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        self.ip().cer_capture(out)?;
        out.write(&self.port().to_le_bytes())?;
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let ip = Ipv4Addr::cer_read(src)?;
        let port = u16::from_le_bytes(src.take_array::<2>()?);
        Ok(SocketAddrV4::new(ip, port))
    }
}

impl CerulionState for SocketAddrV6 {
    const STATE_SHAPE: u64 = StateShape::of("SocketAddrV6")
        .element(<Ipv6Addr as CerulionState>::STATE_SHAPE)
        .finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 26;

    /// `flowinfo` and `scope_id` are part of the address's identity and are
    /// carried, not dropped.
    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        self.ip().cer_capture(out)?;
        out.write(&self.port().to_le_bytes())?;
        out.write(&self.flowinfo().to_le_bytes())?;
        out.write(&self.scope_id().to_le_bytes())?;
        Ok(())
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        let ip = Ipv6Addr::cer_read(src)?;
        let port = u16::from_le_bytes(src.take_array::<2>()?);
        let flowinfo = u32::from_le_bytes(src.take_array::<4>()?);
        let scope_id = u32::from_le_bytes(src.take_array::<4>()?);
        Ok(SocketAddrV6::new(ip, port, flowinfo, scope_id))
    }
}

impl CerulionState for SocketAddr {
    const STATE_SHAPE: u64 = StateShape::of("SocketAddr")
        .element(<SocketAddrV4 as CerulionState>::STATE_SHAPE)
        .element(<SocketAddrV6 as CerulionState>::STATE_SHAPE)
        .finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 7;

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        match self {
            SocketAddr::V4(addr) => {
                out.write(&[4])?;
                addr.cer_capture(out)
            }
            SocketAddr::V6(addr) => {
                out.write(&[6])?;
                addr.cer_capture(out)
            }
        }
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        match src.take_array::<1>()?[0] {
            4 => Ok(SocketAddr::V4(SocketAddrV4::cer_read(src)?)),
            6 => Ok(SocketAddr::V6(SocketAddrV6::cer_read(src)?)),
            tag => Err(StateError::InvalidTag {
                type_name: "SocketAddr",
                tag,
            }),
        }
    }
}

// ---------------------------------------------------------------------------
// the `#[cerulion(serde)]` escape
// ---------------------------------------------------------------------------

/// Encode one `#[cerulion(serde)]` field: a `u32` LE length, then its JSON.
///
/// # Why this escape is a WEAKER citizen, stated rather than implied
///
/// It exists so an ordinary node holding a foreign math type (an
/// `nalgebra::Isometry3`, an `ndarray` view) does not have to choose between
/// forking that crate and losing its pose to `reconstruct` — the silent-loss
/// path that is dangerous. What it gives up, it gives up openly:
///
/// - **`INLINE_SAFE` is `false`.** The bytes come from a user-written
///   `Serialize`, which can take a lock, allocate, or run arbitrary control
///   flow, so the sink's BYTE bound stops implying a TIME bound. Such a
///   node takes the fork carrier.
/// - **The shape is `tokens`-kind.** A serde impl's bytes are not structurally
///   derivable, so the field folds only its name and the escape marker — which
///   means changing the field's TYPE does not bump `STATE_SHAPE`. The
///   mismatch surfaces as a decode error at restore rather than as a shape
///   refusal before it, which is later and less precise than the inventory's
///   guarantee.
/// - **Byte-stability is the field's own `Serialize`'s business.** A type that
///   serializes a `HashMap` in iteration order is not byte-deterministic, and
///   nothing here can make it so.
///
/// The length is read back with
/// [`StateCursor::read_payload_len`](super::StateCursor::read_payload_len) —
/// UNCHARGED against the element budget, correctly: the bytes are consumed
/// immediately by the `take` that follows, so the blob itself bounds them.
pub fn capture_serde_field<T>(
    value: &T,
    field: &'static str,
    out: &mut dyn StateSink,
) -> Result<(), StateError>
where
    T: serde::Serialize,
{
    let bytes = serde_json::to_vec(value).map_err(|err| StateError::SerdeEncode {
        field,
        // Allocates, deliberately: this is the error path of a cold,
        // opt-in escape, and the cause is the whole diagnostic.
        cause: err.to_string(),
    })?;
    write_len(out, bytes.len())?;
    out.write(&bytes)?;
    Ok(())
}

/// Decode one `#[cerulion(serde)]` field written by [`capture_serde_field`].
pub fn read_serde_field<T>(src: &mut StateCursor<'_>, field: &'static str) -> Result<T, StateError>
where
    T: serde::de::DeserializeOwned,
{
    let len = src.read_payload_len()?;
    let bytes = src.take(len)?;
    serde_json::from_slice(bytes).map_err(|err| StateError::SerdeDecode {
        field,
        // Allocates, deliberately: this is the error path of a cold,
        // opt-in escape, and the cause is the whole diagnostic.
        cause: err.to_string(),
    })
}
