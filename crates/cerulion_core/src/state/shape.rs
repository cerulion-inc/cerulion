//! `STATE_SHAPE` — the recursive **structural** identity of a state type.
//!
//! The recipe is deliberately the same shape as
//! [`MessageSchema::schema_hash`](crate::codegen::MessageSchema::schema_hash)
//! (hash recipe 3): FNV-1a 64 over a length-prefixed stream, folding each
//! member's own hash into the parent so a change *inside* a nested type bumps
//! the parent. It reuses that hash's constants rather than carrying a copy.
//!
//! # Why structural and not a token hash
//!
//! A proc macro sees only tokens, so a per-field token hash gets both
//! directions wrong: renaming `Cell` → `GridCell` changes the token
//! hash and would **refuse a byte-compatible restore**, while *adding a field
//! to a nested type does not change the token hash at all* — so incompatible
//! bytes are accepted and decode "successfully", after which a resim reports a
//! confident FAIL against a divergence the restore itself manufactured. An
//! associated const folds through the *real* type graph at compile time, at
//! zero runtime cost, and gets both right.

use crate::wire::{FNV_OFFSET, FNV_PRIME};

/// A `const`-foldable builder for a type's [`STATE_SHAPE`].
///
/// Every method is a `const fn` taking `self` by value, so an implementation
/// writes its shape as a single associated-const initializer that references
/// its members' shapes.
///
/// A node author does not write this. `#[derive(CerulionState)]` emits it for
/// a type a node holds, and a `#[cerulion_node]` struct gets its impl from the
/// macro (a hand-written impl beside a node is a conflicting-impl error). The
/// builder is public for the framework's own impls and for the rare type that
/// needs a hand-written `CerulionState`. This is what such an impl looks like:
///
/// ```
/// use cerulion_core::state::{CerulionState, StateShape};
///
/// struct Pose {
///     x: f64,
///     y: f64,
/// }
///
/// impl CerulionState for Pose {
///     const STATE_SHAPE: u64 = StateShape::of("Pose")
///         .field("x", <f64 as CerulionState>::STATE_SHAPE)
///         .field("y", <f64 as CerulionState>::STATE_SHAPE)
///         .finish();
///     # fn cer_capture(
///     #     &self,
///     #     out: &mut dyn cerulion_core::state::StateSink,
///     # ) -> Result<(), cerulion_core::state::StateError> {
///     #     self.x.cer_capture(out)?;
///     #     self.y.cer_capture(out)
///     # }
///     # fn cer_read(
///     #     src: &mut cerulion_core::state::StateCursor<'_>,
///     # ) -> Result<Self, cerulion_core::state::StateError> {
///     #     Ok(Pose { x: f64::cer_read(src)?, y: f64::cer_read(src)? })
///     # }
/// }
/// ```
///
/// [`STATE_SHAPE`]: crate::state::CerulionState::STATE_SHAPE
#[derive(Debug, Clone, Copy)]
pub struct StateShape {
    hash: u64,
}

impl StateShape {
    /// Start a shape seeded with the type's own name.
    ///
    /// The name is the *declared* identity — the `#[derive]`d struct's ident,
    /// or the container's name for a framework impl. It is length-prefixed so
    /// concatenation is unambiguous.
    pub const fn of(type_name: &str) -> Self {
        Self {
            hash: feed_str(FNV_OFFSET, type_name),
        }
    }

    /// Fold in a **named** member: a struct field, or an enum variant.
    ///
    /// Name-keying is what closes the transposition trap: under purely
    /// positional identity, swapping two same-typed `f64`s (`goal_x`,
    /// `goal_y`) restores silently transposed.
    pub const fn field(self, name: &str, shape: u64) -> Self {
        let hash = feed_str(self.hash, name);
        Self {
            hash: feed_u64(hash, shape),
        }
    }

    /// Fold in an **unnamed** member: a container's element, a tuple slot, a
    /// map's key or value.
    pub const fn element(self, shape: u64) -> Self {
        Self {
            hash: feed_u64(self.hash, shape),
        }
    }

    /// Fold in a structural count — an array's `N`, a tuple's arity.
    ///
    /// Load-bearing: without it `[u8; 4]` and `[u8; 8]` would share a shape,
    /// and a retype between them would restore silently short.
    pub const fn count(self, n: usize) -> Self {
        Self {
            hash: feed_u64(self.hash, n as u64),
        }
    }

    /// The folded 64-bit shape.
    pub const fn finish(self) -> u64 {
        self.hash
    }
}

/// Fold raw bytes into a running FNV-1a state.
const fn feed_bytes(mut hash: u64, bytes: &[u8]) -> u64 {
    let mut i = 0;
    while i < bytes.len() {
        hash ^= bytes[i] as u64;
        hash = hash.wrapping_mul(FNV_PRIME);
        i += 1;
    }
    hash
}

/// Fold a `u64` LE into a running FNV-1a state.
const fn feed_u64(hash: u64, value: u64) -> u64 {
    feed_bytes(hash, &value.to_le_bytes())
}

/// Fold a length-prefixed string into a running FNV-1a state.
const fn feed_str(hash: u64, s: &str) -> u64 {
    let bytes = s.as_bytes();
    let hash = feed_u64(hash, bytes.len() as u64);
    feed_bytes(hash, bytes)
}
