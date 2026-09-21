//! The state-capture core.
//!
//! One trait ([`CerulionState`]), one sink ([`StateSink`]/[`BoundedSink`]),
//! one reader ([`StateCursor`]), one compile-time identity ([`StateShape`]),
//! one error ([`StateError`]) — plus the closed blanket-impl inventory that
//! makes "the user writes an ordinary Rust struct" true.
//!
//! # For a node author
//!
//! A Flashback capture and a `cerulion bag play --resim` restart both put a
//! node back the way it was, which means capturing the node's own fields.
//! **A node struct needs nothing from this module**: `#[cerulion_node]` emits
//! the capture machinery itself, over the struct's non-port fields. What you
//! reach for is the derive, on the types your node HOLDS:
//!
//! ```
//! use cerulion_core::state::CerulionState;
//!
//! #[derive(CerulionState)]
//! struct Pose {
//!     x: f64,
//!     y: f64,
//! }
//! # fn needs_state<T: CerulionState>() {}
//! # needs_state::<Pose>();
//! ```
//!
//! It is also the fix the compiler names when a node field cannot be
//! captured. Most fields need nothing; the per-field escapes are for the
//! exceptional one:
//!
//! | Attribute | Meaning |
//! |---|---|
//! | `#[cerulion(reconstruct)]` | A HANDLE: not captured, and left untouched on restore. For something that must be rebuilt rather than restored (a connection, a device) |
//! | `#[cerulion(serde)]` | Capture through the field's own `Serialize` / `Deserialize` |
//! | `#[cerulion(unordered)]` | A hash-like container whose key has no total order, captured in iteration order |
//!
//! A resource type the framework already recognises (`File`, `TcpStream`,
//! `JoinHandle`, a `Box<dyn Trait>`, `Arc<TransportManager>`) is treated as
//! `reconstruct` without the attribute. An unknown key inside
//! `#[cerulion(...)]` is a compile error.
//!
//! The rest of this page is the machinery behind that derive: the trait, the
//! encoding and the limits. A node author does not implement any of it by
//! hand.
//!
//! # The three properties that make a capture cheap
//!
//! 1. **[`INLINE_SAFE`] turns a byte bound into a time bound.**
//!    [`BoundedSink`] bounds *bytes*. That is only a *time* bound if the
//!    encoder cannot take a lock, make a syscall, run user-defined control
//!    flow, or recurse into a cycle — which is exactly what
//!    [`INLINE_SAFE`] certifies, folded as an associated const through the
//!    real type graph at zero runtime cost. It defaults to **`false`**, so a
//!    hand-written impl cannot silently claim a proof it did not make.
//! 2. **[`cer_probe`] is total over the *declared* state, not just the node
//!    mutex.** It is generated from the same field list the encoder walks, so
//!    it sees exactly the locks the encoder would take, and it never blocks.
//! 3. **The canonical order never depends on a per-process `RandomState`.**
//!    Hash-like containers emit in sorted key order, so a `HashMap`'s
//!    iteration order never reaches the bag and `cerulion_bag`'s
//!    byte-determinism gate survives.
//!
//! # The encoding, in one place
//!
//! Little-endian throughout, matching the wire format. There is no per-value
//! type tag: the *shape* is carried once, out of band, by [`STATE_SHAPE`].
//!
//! | Group | Bytes |
//! |---|---|
//! | integers, floats | fixed width, LE (floats as their bit pattern, so NaN payloads are byte-exact) |
//! | `usize`/`isize` | **always 64-bit** LE, so a recording is portable; an out-of-range restore is [`StateError::OutOfRange`], never a truncation |
//! | `bool` | one byte, `0`/`1`; any other byte is [`StateError::InvalidBool`] |
//! | `char` | `u32` LE, validated as a Unicode scalar on read |
//! | `String`, `PathBuf`, `&'static str` | `u32` LE byte length + UTF-8 bytes |
//! | `Vec`, `VecDeque`, sets, maps | `u32` LE count + elements |
//! | `[T; N]`, tuples | elements back to back — the count is in the shape |
//! | `Option`, `Result` | one tag byte + the payload |
//! | `Box`, `Rc`, `Arc`, `Wrapping`, `NonZero*` | transparent |
//! | `Atomic*` | the loaded value, encoded exactly like the primitive it wraps (`AtomicBool` as one `0`/`1` byte) |
//! | `Mutex`, `RwLock` | transparent through `try_lock` — **never** a blocking `lock()` |
//! | `Duration` | `u64` secs + `u32` nanos |
//! | `SystemTime` | sign tag + `u64` secs + `u32` nanos, relative to the Unix epoch |
//! | `IpAddr`, `SocketAddr` | family tag + octets (+ port, flowinfo, scope id) |
//!
//! # What is deliberately NOT implemented
//!
//! `Cell`, `RefCell`, raw pointers, `fn` pointers, `File`/`RawFd` and
//! `Instant` carry **no impl**. `Cell`/`RefCell` have no lock to probe
//! and no way to prove exclusivity at the fork instant; `Instant` is an opaque
//! monotonic reading with no meaning in another process. Each is refused by
//! the type system, with [`CerulionState`]'s own
//! `#[diagnostic::on_unimplemented]` naming both real fixes.
//!
//! ```compile_fail
//! use std::cell::RefCell;
//! use cerulion_core::state::CerulionState;
//! fn needs_state<T: CerulionState>() {}
//! needs_state::<RefCell<u32>>();
//! ```
//!
//! ```compile_fail
//! use std::time::Instant;
//! use cerulion_core::state::CerulionState;
//! fn needs_state<T: CerulionState>() {}
//! needs_state::<Instant>();
//! ```
//!
//! # The derive refuses an uncapturable field
//!
//! `#[derive(CerulionState)]` names one `where`-clause obligation per captured
//! field, so a field whose type is not capturable is a compile error at THAT
//! FIELD carrying the message above. These two doctests are the CI-GATED half
//! of that pin — they assert the refusal HAPPENS. The RENDERING (and the
//! diagnostic COUNT) is pinned by the `#[ignore]`d
//! `tests/ui/type_error/state_derive_uncapturable_field.rs` fixture, which is
//! toolchain-fragile and therefore cannot gate every PR.
//!
//! ```compile_fail
//! use cerulion_core::state::CerulionState;
//! struct CudaContext;
//! #[derive(CerulionState)]
//! struct SlamNode { pose: f64, cuda: CudaContext }
//! fn needs_state<T: CerulionState>() {}
//! needs_state::<SlamNode>();
//! ```
//!
//! The ANTI-TAUTOLOGY twin: the SAME struct compiles once the field says what
//! it is, so the refusal above is about the field rather than about the derive
//! rejecting everything.
//!
//! ```
//! use cerulion_core::state::CerulionState;
//! struct CudaContext;
//! #[derive(CerulionState)]
//! struct SlamNode {
//!     pose: f64,
//!     #[cerulion(reconstruct)]
//!     cuda: CudaContext,
//! }
//! fn needs_state<T: CerulionState>() {}
//! needs_state::<SlamNode>();
//! ```
//!
//! # Residuals
//!
//! - **A zero-byte element type makes the CAPTURE walk O(n) in *iterations*
//!   even though it is O(1) in bytes.** A `Vec<()>` of a billion elements
//!   encodes four bytes and still loops a billion times. Bounded in practice
//!   by the node's own memory — it really holds a billion elements — and
//!   unrelated to the DECODE side, where the count is blob-supplied and is
//!   bounded by [`StateCursor::element_budget`].
//! - **A type may amplify decode work by a COMPILE-TIME constant.** A
//!   `Vec<[(); 1_000_000]>` iterates a million no-ops per element, because the
//!   array length lives in the program's own type rather than in the blob.
//!   Work stays linear in blob length times a constant the node chose.
//! - **Shared-ownership *identity* is not preserved.** Two `Arc` fields
//!   pointing at one allocation restore as two allocations.
//!   This follows the framework's general rule — the value is restored, never the identity.
//!   **Where this bites hardest is `Arc<Atomic…>`**, the
//!   dominant shape for a counter shared with a helper thread: restoring the
//!   node's field replaces the allocation, so a clone handed out in `init()`
//!   keeps writing to the old one and the two diverge silently from then on.
//!   Reachable only when the clone was shared BEFORE the restore, i.e. from
//!   `init()`; a node that spawns its helper in `tick()` is unaffected.
//! - **`cer_probe` walks the declared graph only.** It cannot see a `tracing`
//!   dispatcher lock, an allocator lock, or a lock reached through a
//!   hand-written impl.
//! - **A hand-written `cer_read` can loop outside the decode budget.** No trait
//!   signature can compel arbitrary code through it; the design makes the
//!   budgeted read the ergonomic one and states the contract on
//!   [`CerulionState::cer_read`]. The derive emits only budgeted code,
//!   so the exposure is hand-written impls — the same trust boundary as
//!   [`INLINE_SAFE`] and [`cer_probe`].
//! - **The `#[cerulion(unordered)]` readers cannot refuse duplicates.**
//!   [`read_unordered_entries`] carries no `Eq`/`Ord` bound on its key, so the
//!   [`StateError::DuplicateEntry`] check the container decoders perform is the
//!   collecting caller's job. The derive's generated reader performs it.
//! - **A zero-encoding element type that is NOT single-valued would be
//!   over-refused by the capture pre-check.** No such type exists in the
//!   inventory (`()`, `PhantomData`, `[T; 0]` each have one value), but a
//!   derived struct whose every field is `#[cerulion(reconstruct)]` would be
//!   one, and a large container of them would trip the floor-free disjunct.
//! - **A container length above `u32::MAX` is [`StateError::LengthOverflow`],
//!   and that arm has no test** — reaching it needs a four-billion-element
//!   container, so it is guarded rather than pinned.
//!
//! # `cer_probe`'s `INLINE_SAFE` short circuit
//!
//! Every composite's probe returns `true` immediately when its own
//! [`INLINE_SAFE`] is set, instead of walking. That is SOUND for the impls in
//! this module by construction — `INLINE_SAFE` is `false` for `Mutex` and
//! `RwLock` and folds with AND, so `true` means no lock is reachable — and it
//! is what keeps the probe O(1) for the overwhelmingly common lock-free node
//! rather than O(state size) on the node thread once per cadence, which would
//! be worse than the cost the whole design exists to avoid.
//!
//! It makes `INLINE_SAFE` load-bearing in a **second** place, which is worth
//! stating: a const wrongly set `true` would both run an encoder on the node
//! thread that must not run there AND skip the probe. That is one const being
//! wrong, not two failure modes — and inverting the short circuit fails three
//! `cer_probe` tests, so the direction is pinned. Deleting it entirely is
//! behaviour-preserving and is deliberately NOT pinned: it is an optimization.
//!
//! [`INLINE_SAFE`]: CerulionState::INLINE_SAFE
//! [`STATE_SHAPE`]: CerulionState::STATE_SHAPE
//! [`cer_probe`]: CerulionState::cer_probe

mod cursor;
mod error;
mod impls;
mod shape;
mod sink;
mod skip_cause;

#[cfg(test)]
mod tests;

pub use cursor::StateCursor;
pub use error::StateError;
pub use impls::{
    capture_serde_field, capture_unordered_map, capture_unordered_set, probe_unordered_map,
    probe_unordered_set, read_serde_field, read_unordered_elements, read_unordered_entries,
};

/// `#[derive(CerulionState)]` — see the [macro's own docs][macro@CerulionState].
///
/// Re-exported beside the trait so generated code can name ONE path
/// (`::cerulion_core::state::CerulionState`) for both, exactly as `serde` does
/// for `Serialize`.
pub use cerulion_macros::CerulionState;
pub use shape::StateShape;
pub use sink::{BoundedSink, SinkFull, StateSink, VecSink};
pub use skip_cause::SkipCause;

/// The per-boundary inline capture budget, in **bytes**.
///
/// # Where 64 KiB comes from
///
/// Derived, not chosen: the gating quantum is the graph's
/// `tightest_timing_ns` with a 1 ms floor, and a capture should consume a
/// small fraction of one quantum. At a deliberately conservative ~1 GB/s
/// encode-into-a-slice throughput, 5 % of 1 ms is ~50 KB, so 64 KiB.
///
/// It is expressed in **bytes, not time**, precisely so the boundary reads no
/// clock: `Instant::now()` at a step boundary is both a determinism hazard and,
/// on macOS under background QoS, unreliable (one run measured a
/// nominal 150 ms charged as 1100–1696 ms).
///
/// # The measured basis (2026-08-08)
///
/// The bench measured the **worst canonical shape** — a sorted `HashMap`,
/// which pays the `Vec<&K>` index build and a hash lookup per entry on top of
/// the encode — filling 64 KiB in:
///
/// | Machine | Fill time | Share of the 1 ms floor quantum |
/// |---|---|---|
/// | Jetson (aarch64) | 56.4 µs | **5.6 %** |
/// | M3 (macOS) | 21.5 µs | 2.2 % |
///
/// The Jetson figure is **13 % over the design's ≤ 5 % target**, and is
/// blessed as shipped: the 1 ms floor is the *minimum* gating quantum, typical
/// quanta are larger, and the overshoot is against the worst shape rather than
/// a representative one. **This constant is the tuning point** if the ≤ 5 %
/// target is later enforced strictly — halving it to 32 KiB would put the
/// Jetson's worst shape at ~2.8 %, at the cost of pushing more nodes onto the
/// fork carrier.
pub const CAPTURE_INLINE_BUDGET_BYTES: usize = 64 * 1024;

/// A key type whose ordering makes a hash-like container byte-deterministic.
///
/// Blanket-implemented for every [`Ord`] type. It exists only to own the
/// diagnostic: a `HashMap` keyed by something containing an `f64` — a float
/// grid coordinate, a pose-keyed cache — cannot be `Ord`, and rustc's own
/// "the trait bound `K: Ord` is not satisfied" says nothing about *why* a
/// recording needs it or what the user should do instead.
///
/// The message deliberately does **not** offer `#[cerulion(reconstruct)]`:
/// such a field is perfectly capturable, and steering the user to
/// `reconstruct` would trade a byte-stability property for silent state loss.
///
/// [`Ord`] is deliberately **not** a supertrait, and the ordering is exposed
/// as [`Self::canonical_cmp`] instead. With `Ord` as a supertrait rustc
/// reports the *supertrait* bound (`the trait bound `K: Ord` is not
/// satisfied`) and this message is never shown — verified by building the
/// failing case both ways.
#[diagnostic::on_unimplemented(
    message = "`{Self}` cannot be a Cerulion map/set key because it is not `Ord`",
    label = "this key type has no total order",
    note = "A recording must be byte-deterministic, so Cerulion emits hash-like containers in sorted key order. Without `Ord` there is no order to sort by, and the same logical state would serialize two different ways.",
    note = "fix 1: give the key a total order — usually by indexing with an integer cell instead of a float, which is generally what the code wanted",
    note = "fix 2: mark the field `#[cerulion(unordered)]` to emit in iteration order. The field is still captured and restored; only its byte-stability across runs is given up, and the manifest records `order: unordered` for it."
)]
pub trait CanonicalMapKey {
    /// The total order the canonical encoder sorts by.
    fn canonical_cmp(&self, other: &Self) -> std::cmp::Ordering;
}

// `do_not_recommend` keeps rustc from descending into this blanket impl and
// reporting the bare `K: Ord` bound instead of the message above — verified by
// building the failing case both with and without it.
#[diagnostic::do_not_recommend]
impl<K: Ord> CanonicalMapKey for K {
    fn canonical_cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.cmp(other)
    }
}

/// The obligation a field of an **enum variant** carries.
///
/// Blanket-implemented for every [`CerulionState`] type and carrying nothing
/// of its own: every item here forwards straight to the trait of the same
/// name. It exists only to own a diagnostic, exactly like [`CanonicalMapKey`],
/// because [`CerulionState`]'s own message is **false inside an enum**.
///
/// That message's handle fix is "mark the field `#[cerulion(reconstruct)]`" —
/// and the derive REFUSES that attribute on a variant's field. A user with
/// `enum Link { Connected(TcpStream) }` was therefore sent round a closed
/// loop: the `E0277` names an escape, and writing it is a hard error naming
/// the `E0277`. MEASURED both halves before this trait was written.
///
/// # Why the derive cannot just classify it, the way a struct field is
///
/// A struct field holding a recognised handle auto-classifies `reconstruct`
/// ([`resource`]-inventory). Doing the same inside a variant is
/// not a smaller version of the same feature — it is unimplementable, for two
/// independent reasons:
///
/// 1. **There is nothing to leave untouched.** `reconstruct` means "restore
///    walks past this field", which a struct can do because it restores field
///    by field IN PLACE. An enum restores by REPLACING the value (its variant
///    may change), so its read must MINT the variant — and a socket cannot be
///    minted from bytes. Defaulting one would fabricate state and, worse,
///    silently decide which variant is live.
/// 2. **The failure direction inverts.** The inventory is name-keyed, so a
///    user type sharing a name with an entry is a known false positive. On the
///    struct path that fails OPEN (silently reconstructed — the accepted,
///    documented cost). On the enum path it would have to fail CLOSED, so
///    `enum E { V(MyFile) }` with a perfectly capturable `MyFile` would STOP
///    COMPILING. A rule whose failure direction flips between the two paths is
///    worse than no rule.
///
/// So the real fix is the message, which is this trait's whole job.
///
/// [`resource`]: cerulion_macros::CerulionState
#[diagnostic::on_unimplemented(
    message = "`{Self}` cannot be part of a Cerulion node's state",
    label = "this enum variant's field cannot be captured for replay",
    note = "Cerulion snapshots node state so a recording made mid-run can be replayed. Every field must say what it is.",
    note = "`#[cerulion(reconstruct)]` is NOT available here — an enum restores by REPLACING the whole value, so reading one must MINT the variant, and a handle cannot be minted from bytes. This is the one place the advice for a struct field does not carry over.",
    note = "fix 1, if this is a HANDLE: hold it OUTSIDE the enum — a `#[cerulion(reconstruct)]` field on the struct that owns the enum, re-opened in `fn restored(&mut self)` — and leave the enum carrying only the capturable part",
    note = "fix 2, if this is real state: make it capturable — add `#[derive(CerulionState)]` to `{Self}`"
)]
pub trait CerulionVariantMember: Sized {
    /// Forwards [`CerulionState::STATE_SHAPE`].
    const VARIANT_STATE_SHAPE: u64;

    /// Forwards [`CerulionState::INLINE_SAFE`].
    const VARIANT_INLINE_SAFE: bool;

    /// Forwards [`CerulionState::cer_probe`].
    fn variant_cer_probe(&self) -> bool;

    /// Forwards [`CerulionState::cer_capture`].
    fn variant_cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError>;

    /// Forwards [`CerulionState::cer_read`].
    fn variant_cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError>;
}

// `CerulionState` is deliberately NOT a supertrait, and every item above is a
// forwarder rather than an inherited one, for the reason the `CanonicalMapKey`
// docs record two screens up: with the real trait as a supertrait rustc
// reports the SUPERTRAIT bound and the message above is never shown. MEASURED
// on `enum E { Connected(TcpStream) }` — the supertrait form rendered
// `CerulionState`'s note verbatim, the forwarding form renders this one.
//
// `do_not_recommend` then keeps rustc from descending into this blanket impl
// and reporting `T: CerulionState` instead — same reason, same measurement.
#[diagnostic::do_not_recommend]
impl<T: CerulionState> CerulionVariantMember for T {
    const VARIANT_STATE_SHAPE: u64 = <T as CerulionState>::STATE_SHAPE;
    const VARIANT_INLINE_SAFE: bool = <T as CerulionState>::INLINE_SAFE;

    fn variant_cer_probe(&self) -> bool {
        <T as CerulionState>::cer_probe(self)
    }

    fn variant_cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        <T as CerulionState>::cer_capture(self, out)
    }

    fn variant_cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        <T as CerulionState>::cer_read(src)
    }
}

/// A type Cerulion can snapshot and restore as part of a node's state.
///
/// See the [module docs](self) for the encoding, the inventory and the
/// residuals.
#[diagnostic::on_unimplemented(
    message = "`{Self}` cannot be part of a Cerulion node's state",
    label = "this field cannot be captured for replay",
    note = "Cerulion snapshots node state so a recording made mid-run can be replayed. Every field must say what it is.",
    note = "if this is a HANDLE that should be rebuilt rather than restored, mark the field `#[cerulion(reconstruct)]` and re-open it in `fn restored(&mut self)`",
    note = "if this is real state, make it capturable: add `#[derive(CerulionState)]` to `{Self}`"
)]
pub trait CerulionState: Sized {
    /// Recursive **structural** identity: member names plus each member
    /// type's own `STATE_SHAPE`, folded in declaration order.
    ///
    /// Built with [`StateShape`]; see that type for the recipe and for why a
    /// token hash cannot serve.
    const STATE_SHAPE: u64;

    /// `true` iff capturing this type is structurally incapable of blocking
    /// the node thread.
    ///
    /// Framework-generated walk only: no lock, no interior mutability, no
    /// user-written encoder. Folded as an **AND** through the real type graph,
    /// exactly like [`Self::STATE_SHAPE`]. This is what turns
    /// [`BoundedSink`]'s byte bound into a time bound with no clock.
    ///
    /// **Defaults to `false`**, deliberately: a hand-written impl cannot
    /// silently claim a proof it did not make, and the cost of being wrong in
    /// the safe direction is only that the node takes the fork carrier.
    const INLINE_SAFE: bool = false;

    /// A floor on how many bytes a value of this type encodes to.
    ///
    /// Used by the hash-like containers' pre-check, which must decide whether
    /// a capture can fit BEFORE building its sort index. Charging a flat one
    /// byte per entry instead would spuriously refuse an exactly-fitting
    /// container of a zero-encoding element type — a one-entry `HashSet<()>`
    /// encodes as nothing but its four-byte count.
    ///
    /// **Defaults to `0`**, which claims nothing: the pre-check exists to skip
    /// wasted scratch on a capture that was going to be refused anyway, so
    /// under-charging costs a missed optimization while over-charging would
    /// refuse state that fits. The safe default is therefore the low one —
    /// the opposite direction from [`Self::INLINE_SAFE`], and for the opposite
    /// reason. A nonzero default is not available: it would refuse an
    /// exactly-fitting container of a zero-encoding type, which is the bug this
    /// const exists to fix.
    ///
    /// # The contract for a hand-written impl
    ///
    /// Declare a TRUE floor — a value no encoding of the type can go under —
    /// or leave it at `0`. A floor that is too HIGH refuses state that fits; a
    /// floor of `0` only costs work.
    ///
    /// **The bound does not rest on it.** A hand impl that leaves it at `0`
    /// cannot reintroduce the unbounded sort index, because the pre-check
    /// carries a second, floor-free disjunct: a container with more entries
    /// than the sink has bytes left can never justify an index, whatever it
    /// claims about its element size. The floor is what lets a *legitimate*
    /// small container through, not what holds the ceiling.
    ///
    /// The blanket impls all declare real floors, pinned against actual
    /// smallest encodings by `min_encoded_bytes_is_a_true_floor_on_every_inventory_row`.
    /// The derive must sum its captured fields' floors; a
    /// `#[cerulion(reconstruct)]` field contributes nothing.
    const MIN_ENCODED_BYTES: usize = 0;

    /// Pre-fork lock probe — non-blocking, total over the declared state.
    ///
    /// `false` means some lock in this type graph is held **right now**, so
    /// the boundary skips the whole anchor rather than forking into a lock the
    /// child would hold forever (a fork child has one thread).
    ///
    /// **Defaults to `true`**, which is sound because the only types that can
    /// answer `false` are ones this module implements — a hand-written impl
    /// with a lock in it is covered by the fork carrier's progress watchdog,
    /// not by a claim it makes here.
    fn cer_probe(&self) -> bool {
        true
    }

    /// Encode this value into `out` in canonical order.
    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError>;

    /// Decode a fresh value from `src`.
    ///
    /// Required (rather than derived from [`Self::cer_restore`]) because
    /// container impls must *construct* elements, and demanding `Default` on
    /// every element type would refuse ordinary state — `Vec<NonZeroU32>`
    /// among it.
    ///
    /// # Contract for a hand-written impl
    ///
    /// **Anything blob-driven that you LOOP over must come from
    /// [`StateCursor::read_element_count`]**, which charges the decode-wide
    /// element budget. Use [`StateCursor::read_payload_len`] only for a length
    /// whose bytes you consume immediately with [`StateCursor::take`] — that
    /// one is bounded by the blob itself.
    ///
    /// This is a CONTRACT, not an enforcement, and the exact scope is worth
    /// stating: `cer_read` takes a cursor and can call [`StateCursor::take`] in
    /// a loop of its own devising, so no trait signature can compel it through
    /// the budget. What the design does instead is make the charged path the
    /// only ergonomic one — the budgeted read is a first-class cursor method,
    /// the two vocabularies are separate named methods, and the derive
    /// generates only budgeted code. The residual is therefore exactly the
    /// hand-written impl, the same trust boundary as [`Self::INLINE_SAFE`] and
    /// [`Self::cer_probe`], both of which likewise believe what an impl
    /// declares.
    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError>;

    /// Restore this value in place from `src`.
    ///
    /// The default assigns a freshly decoded value. Impls override it where
    /// in-place restore is materially better: a `Mutex` keeps its identity, an
    /// array avoids a temporary `Vec`, and, decisively, the derive
    /// must restore field by field so a `#[cerulion(reconstruct)]` field is
    /// left untouched.
    fn cer_restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
        *self = Self::cer_read(src)?;
        Ok(())
    }
}
