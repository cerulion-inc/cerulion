//! `#[derive(CerulionState)]` end to end.
//!
//! The macro crate's own tests can only read the TOKENS the derive emits. This
//! file is the proof they describe something real: every type here is derived
//! by the production macro, compiled by rustc, and driven against a
//! **hand-written byte oracle** — never against a second run of the derive.
//!
//! Pure: no transport, no iceoryx2, no shared-memory singleton, so the file is
//! parallel-safe and needs no `--test-threads=1`.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use cerulion_core::state::{CerulionState, StateCursor, StateError, StateShape, VecSink};

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

fn capture(value: &impl CerulionState) -> Vec<u8> {
    let mut sink = VecSink::new();
    value.cer_capture(&mut sink).expect("unbounded capture");
    sink.into_inner()
}

/// Read a type's `INLINE_SAFE` THROUGH a function.
///
/// Not indirection for its own sake: `assert!(T::INLINE_SAFE)` is a constant
/// assertion, which clippy's `assertions_on_constants` rejects — and it would
/// also be const-folded away rather than run. The sibling
/// `cerulion_core/src/state/tests.rs` uses the same helper for the same
/// reason.
fn inline_safe<T: CerulionState>() -> bool {
    T::INLINE_SAFE
}

fn read_exact<T: CerulionState>(bytes: &[u8]) -> T {
    let mut cursor = StateCursor::new(bytes);
    let value = T::cer_read(&mut cursor).expect("decode");
    cursor.finish().expect("blob fully consumed");
    value
}

/// A key with NO total order — the shape `#[cerulion(unordered)]` exists for.
///
/// It is `CerulionState` (so it can be captured) but deliberately not `Ord`,
/// so a plain `HashMap<FloatCell, _>` field would be refused by
/// `CanonicalMapKey`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, CerulionState)]
struct FloatCell {
    bits: u64,
}

// ---------------------------------------------------------------------------
// the ordinary struct
// ---------------------------------------------------------------------------

#[derive(Debug, Default, PartialEq, CerulionState)]
struct Pose {
    x: f64,
    y: f64,
}

#[derive(Debug, Default, PartialEq, CerulionState)]
struct Odom {
    pose: Pose,
    seq: u32,
    label: String,
}

#[test]
fn a_derived_struct_encodes_its_fields_back_to_back_in_declaration_order() {
    let odom = Odom {
        pose: Pose { x: 1.0, y: -2.0 },
        seq: 7,
        label: "hi".to_string(),
    };

    // HAND ORACLE: f64 bit patterns, then a u32, then a u32 length + UTF-8.
    let mut want = Vec::new();
    want.extend_from_slice(&1.0f64.to_le_bytes());
    want.extend_from_slice(&(-2.0f64).to_le_bytes());
    want.extend_from_slice(&7u32.to_le_bytes());
    want.extend_from_slice(&2u32.to_le_bytes());
    want.extend_from_slice(b"hi");

    assert_eq!(capture(&odom), want);
    assert_eq!(read_exact::<Odom>(&want), odom);
}

/// A field type that reports WHICH entry point the derive drove it through.
///
/// The values alone cannot tell `cer_restore` (field by field, in place) from
/// `*self = cer_read(..)?` (replace the whole struct) — both leave the same
/// numbers behind, which is why the test below needed one of these. This type
/// carries a `birthmark` that `cer_capture` never writes and `cer_read`
/// cannot know: a fresh value minted from bytes has no way to recover the one
/// the running value held, so an intact birthmark after a restore is proof
/// that the RUNNING value was mutated rather than replaced.
#[derive(Debug, PartialEq)]
struct PathProbe {
    value: u64,
    birthmark: u64,
}

impl PathProbe {
    /// What `cer_read` must stamp, being unable to do better.
    const MINTED: u64 = 0;
}

impl CerulionState for PathProbe {
    const STATE_SHAPE: u64 = StateShape::of("PathProbe").finish();
    const INLINE_SAFE: bool = true;
    const MIN_ENCODED_BYTES: usize = 8;

    fn cer_capture(&self, out: &mut dyn cerulion_core::state::StateSink) -> Result<(), StateError> {
        // The birthmark is deliberately NOT recorded: it is identity, not
        // state, so nothing in the blob can restore it.
        self.value.cer_capture(out)
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(Self {
            value: u64::cer_read(src)?,
            birthmark: Self::MINTED,
        })
    }

    fn cer_restore(&mut self, src: &mut StateCursor<'_>) -> Result<(), StateError> {
        self.value = u64::cer_read(src)?;
        // `birthmark` is untouched — this is the in-place path.
        Ok(())
    }
}

#[derive(CerulionState)]
struct Tracked {
    probe: PathProbe,
    seq: u32,
}

#[test]
fn a_derived_struct_restores_in_place_field_by_field() {
    let recorded = capture(&Odom {
        pose: Pose { x: 3.5, y: 4.5 },
        seq: 9,
        label: "back".to_string(),
    });
    let mut live = Odom::default();
    let mut cursor = StateCursor::new(&recorded);
    live.cer_restore(&mut cursor).expect("restore");
    cursor.finish().expect("blob fully consumed");

    assert_eq!(live.pose, Pose { x: 3.5, y: 4.5 });
    assert_eq!(live.seq, 9);
    assert_eq!(live.label, "back");

    // The values above pass under whole-value replacement too, so on their own
    // they do not test the name of this test. The probe field does: its
    // birthmark survives ONLY if the derive called the FIELD's `cer_restore`
    // on the running value. A `*self = Self::cer_read(src)?` would have gone
    // through `PathProbe::cer_read`, which stamps `MINTED`.
    let recorded = capture(&Tracked {
        probe: PathProbe {
            value: 77,
            birthmark: 0xC0FFEE,
        },
        seq: 5,
    });
    let mut live = Tracked {
        probe: PathProbe {
            value: 1,
            birthmark: 0xBEEF,
        },
        seq: 0,
    };
    let mut cursor = StateCursor::new(&recorded);
    live.cer_restore(&mut cursor).expect("restore");
    cursor.finish().expect("blob fully consumed");

    assert_eq!(live.probe.value, 77, "the field really was restored");
    assert_eq!(live.seq, 5);
    assert_eq!(
        live.probe.birthmark,
        0xBEEF,
        "the RUNNING field survived: it was mutated in place, not replaced by \
         one minted from the blob (which would read {})",
        PathProbe::MINTED
    );
    // Anti-tautology: the birthmark is a real discriminator, i.e. the read
    // path genuinely produces a different one. Without this the assertion
    // above would also hold for a probe whose two paths agreed.
    assert_eq!(
        read_exact::<Tracked>(&recorded).probe.birthmark,
        PathProbe::MINTED,
        "the read path must NOT be able to recover a birthmark"
    );
}

#[test]
fn what_the_in_place_restore_does_not_promise_is_allocation_reuse() {
    // Stated rather than assumed, because the sibling test's name invites the
    // stronger reading. The derive's guarantee is that each FIELD's own
    // `cer_restore` runs on the running value — nothing more. What that does
    // to the field's heap buffer is the FIELD TYPE's business, and no shipped
    // impl promises to reuse one: `String` and `Vec` do not override
    // `cer_restore` at all, so they take the trait default (`*self =
    // Self::cer_read(src)?`) and get a fresh allocation.
    //
    // Pinned so the claim cannot rot into the stronger one by accident: if a
    // future `String`/`Vec` impl DOES start reusing its buffer, this fails and
    // whoever made that true updates the contract deliberately.
    let recorded = capture(&"restored".to_string());

    let mut live = String::with_capacity(4096);
    live.push_str("running");
    let before = live.as_ptr();

    let mut cursor = StateCursor::new(&recorded);
    live.cer_restore(&mut cursor).expect("restore");
    cursor.finish().expect("blob fully consumed");

    assert_eq!(live, "restored");
    assert_ne!(
        live.as_ptr(),
        before,
        "a `String` field is REPLACED, not filled in place — if this ever \
         changes, the in-place guarantee has genuinely widened and should be \
         documented rather than discovered"
    );
}

#[test]
fn a_derived_shape_folds_field_names_so_a_transposition_cannot_pass() {
    // HAND-FOLDED oracle, using the same public builder the impl does.
    let want = StateShape::of("Pose")
        .field("x", <f64 as CerulionState>::STATE_SHAPE)
        .field("y", <f64 as CerulionState>::STATE_SHAPE)
        .finish();
    assert_eq!(<Pose as CerulionState>::STATE_SHAPE, want);

    // Two same-typed fields SWAPPED must not share a shape — the whole reason
    // the shape is keyed by name.
    #[derive(CerulionState)]
    struct Goal {
        goal_x: f64,
        goal_y: f64,
    }
    #[derive(CerulionState)]
    struct GoalSwapped {
        goal_y: f64,
        goal_x: f64,
    }
    assert_ne!(
        <Goal as CerulionState>::STATE_SHAPE,
        <GoalSwapped as CerulionState>::STATE_SHAPE
    );

    // A nested type's change bumps the PARENT's shape (recipe 3).
    assert_ne!(
        <Odom as CerulionState>::STATE_SHAPE,
        StateShape::of("Odom")
            .field("pose", 0)
            .field("seq", <u32 as CerulionState>::STATE_SHAPE)
            .field("label", <String as CerulionState>::STATE_SHAPE)
            .finish()
    );
}

#[test]
fn a_derived_struct_folds_inline_safe_and_the_encoded_floor() {
    assert!(inline_safe::<Odom>());
    // 8 + 8 (Pose) + 4 (u32) + 4 (String's length prefix)
    assert_eq!(<Odom as CerulionState>::MIN_ENCODED_BYTES, 24);
    assert!(Odom::default().cer_probe());

    // A lock anywhere in the graph folds it to false — the ANTI-TAUTOLOGY for
    // the `true` above.
    #[derive(Default, CerulionState)]
    struct Locked {
        shared: Arc<Mutex<Vec<u8>>>,
    }
    assert!(!inline_safe::<Locked>());
    // ... and the field is genuinely captured, not escaped.
    assert_eq!(capture(&Locked::default()), 0u32.to_le_bytes().to_vec());
}

// ---------------------------------------------------------------------------
// atomics through the derive (/ decision 52b)
// ---------------------------------------------------------------------------

#[derive(Default, CerulionState)]
struct TickStats {
    ticks: AtomicU64,
    drops: Arc<AtomicU64>,
}

#[test]
fn a_counter_struct_needs_no_escape_and_captures_its_counters() {
    let stats = TickStats::default();
    stats.ticks.store(11, Ordering::Relaxed);
    stats.drops.store(2, Ordering::Relaxed);

    let mut want = Vec::new();
    want.extend_from_slice(&11u64.to_le_bytes());
    want.extend_from_slice(&2u64.to_le_bytes());
    assert_eq!(capture(&stats), want);

    // Reading one takes no lock, so the whole struct stays inline-eligible.
    assert!(inline_safe::<TickStats>());

    let restored: TickStats = read_exact(&want);
    assert_eq!(restored.ticks.load(Ordering::Relaxed), 11);
    assert_eq!(restored.drops.load(Ordering::Relaxed), 2);
}

/// A captured FIELD and an EXTERNAL clone of one `Arc` come back as two
/// independent atomics.
///
/// Atomics are taken by value (by design), and the stated cost is
/// that `Arc` IDENTITY is not preserved — `Arc<T>`'s own row restores by
/// REPLACING the allocation, so a value captured through two aliasing handles
/// comes back through two separate ones. `impls.rs` says so in prose beside
/// the atomic rows; this is the test that stops that prose rotting into a
/// claim nobody checks.
///
/// The shape here is a field vs a handle held OUTSIDE the captured struct —
/// the shape a node whose `init()` gave a helper thread an `Arc<AtomicU64>`
/// clone is in. `TickStats` carries a single `Arc` field, so it cannot say
/// anything about two aliasing handles that are BOTH fields: that is the
/// sibling `two_fields_of_one_struct_sharing_one_arc_split_into_two_allocations`
/// below, which is a genuinely different question (nothing outside the struct
/// is holding the allocation alive, so an aliasing-preserving restore would be
/// entirely within the derive's power to implement).
///
/// The DEMONSTRATION is the point, not the verdict: it is not asserting that
/// splitting is desirable, it is fixing what the shipped behaviour IS so the
/// documented cost and the code cannot drift apart. A capture-time
/// `Arc::ptr_eq` aliasing guard is the NAMED future refinement
/// — if it ever lands, this test is where its arrival is announced,
/// because it will fail here first.
#[test]
fn a_captured_field_and_an_external_clone_of_one_arc_split_into_two_atomics() {
    let shared = Arc::new(AtomicU64::new(0));
    let stats = TickStats {
        ticks: AtomicU64::new(0),
        drops: Arc::clone(&shared),
    };
    // PRECONDITION, asserted rather than assumed: the two handles really do
    // alias one allocation before the round trip, so a post-restore split is
    // a change this test caused rather than a fixture that never aliased.
    assert!(
        Arc::ptr_eq(&stats.drops, &shared),
        "the fixture must genuinely share one allocation"
    );
    stats.drops.store(41, Ordering::Relaxed);

    let restored: TickStats = read_exact(&capture(&stats));

    // The VALUE survives — that is what capture-by-value buys.
    assert_eq!(restored.drops.load(Ordering::Relaxed), 41);

    // The IDENTITY does not. Writing through the ORIGINAL handle must not be
    // visible through the restored one, and vice versa: this is exactly the
    // shape a node whose `init()` handed an `Arc<AtomicU64>` clone to a helper
    // thread is in after a restore, and the reason the residual is documented
    // where `Arc<Atomic..>` readers will meet it.
    assert!(
        !Arc::ptr_eq(&restored.drops, &shared),
        "restore REPLACES the allocation — see the Arc<Atomic..> residual"
    );
    shared.store(999, Ordering::Relaxed);
    assert_eq!(
        restored.drops.load(Ordering::Relaxed),
        41,
        "the restored atomic must NOT see a write through the original handle"
    );
    restored.drops.store(7, Ordering::Relaxed);
    assert_eq!(
        shared.load(Ordering::Relaxed),
        999,
        "and the original must not see a write through the restored one"
    );
}

/// TWO FIELDS OF ONE STRUCT that share one `Arc` come back as two separate
/// allocations — through BOTH restore paths.
///
/// The sibling above pins a field against a handle held OUTSIDE the struct,
/// where nothing the derive could do would help: something else owns that
/// allocation. This is the case the derive genuinely could decide either way,
/// because both handles are inside the value it is walking — and it was
/// covered by nothing, since `TickStats` has one `Arc` field.
///
/// It matters because it is the shape of every "one counter, two readers"
/// struct — a `hits`/`misses` pair wired to one allocation at construction, a
/// stats block a node hands to two of its own helpers. Before a round trip
/// they are one number; afterwards they are two, and each carries the recorded
/// value independently. Anyone reading `impls.rs`'s "identity is not
/// preserved" residual would probably guess that; nothing checked it.
///
/// BOTH paths are driven on purpose:
///
/// - `cer_read` builds a fresh value, so the split is unsurprising there.
/// - `cer_restore` runs field-by-field over the RUNNING value, where the two
///   handles still alias while the walk is in progress. That is the path an
///   aliasing-preserving refinement (the `Arc::ptr_eq` guard) would
///   have to change, so it is where its arrival gets announced.
///
/// A DEMONSTRATION, like its sibling: it fixes the shipped behaviour so the
/// documented cost cannot drift, and it is not a claim that splitting is what
/// anyone would want.
#[test]
fn two_fields_of_one_struct_sharing_one_arc_split_into_two_allocations() {
    #[derive(Default, CerulionState)]
    struct SharedCounters {
        hits: Arc<AtomicU64>,
        misses: Arc<AtomicU64>,
    }

    // Built so that when the block ends the ONLY two handles to the
    // allocation are the two fields — nothing outside the struct is keeping
    // it alive, which is what separates this case from the sibling above.
    let live = {
        let one = Arc::new(AtomicU64::new(0));
        SharedCounters {
            hits: Arc::clone(&one),
            misses: one,
        }
    };

    // PRECONDITION, asserted rather than assumed: the two FIELDS really are
    // one allocation before the round trip, behaviourally as well as by
    // pointer — otherwise every "split" assertion below is about a fixture
    // that never aliased.
    assert!(
        Arc::ptr_eq(&live.hits, &live.misses),
        "the fixture's two fields must genuinely share one allocation"
    );
    live.hits.store(5, Ordering::Relaxed);
    assert_eq!(
        live.misses.load(Ordering::Relaxed),
        5,
        "a write through one field must be visible through the other BEFORE \
         the round trip — this is what the restore is about to take away"
    );

    let recorded = capture(&live);
    // 8 bytes twice: the aliasing is invisible to the encoder, which is the
    // root of everything below. HAND ORACLE.
    let mut want = Vec::new();
    want.extend_from_slice(&5u64.to_le_bytes());
    want.extend_from_slice(&5u64.to_le_bytes());
    assert_eq!(recorded, want, "one allocation is captured TWICE, by value");

    // --- path 1: `cer_read` (fresh value) ---
    let read: SharedCounters = read_exact(&recorded);
    assert_eq!(read.hits.load(Ordering::Relaxed), 5);
    assert_eq!(read.misses.load(Ordering::Relaxed), 5);
    assert!(
        !Arc::ptr_eq(&read.hits, &read.misses),
        "the two fields must come back as SEPARATE allocations"
    );

    // --- path 2: `cer_restore` (in place, over a value whose two fields are
    // STILL aliased as the walk begins) ---
    let mut restored = SharedCounters {
        hits: Arc::clone(&live.hits),
        misses: Arc::clone(&live.hits),
    };
    assert!(Arc::ptr_eq(&restored.hits, &restored.misses));
    // Move the shared allocation OFF the recorded value first, so that
    // "reads 5 afterwards" cannot be satisfied by a restore that did nothing
    // at all. With this, the value assertions and the identity assertion catch
    // different failures: a no-op restore fails the values, and an
    // identity-preserving one fails the split.
    live.hits.store(77, Ordering::Relaxed);
    assert_eq!(
        restored.hits.load(Ordering::Relaxed),
        77,
        "precondition: the target is genuinely off the recorded value"
    );

    let mut cursor = StateCursor::new(&recorded);
    restored.cer_restore(&mut cursor).expect("restore");
    cursor.finish().expect("blob fully consumed");

    assert_eq!(restored.hits.load(Ordering::Relaxed), 5);
    assert_eq!(restored.misses.load(Ordering::Relaxed), 5);
    assert!(
        !Arc::ptr_eq(&restored.hits, &restored.misses),
        "an in-place restore REPLACES each field's allocation, so an aliasing \
         pair does not survive it — the D2 residual, on the path a future \
         `Arc::ptr_eq` refinement would have to change"
    );
    // The behavioural half of the same claim, and the one a caller actually
    // feels: the two counters have stopped being one number.
    restored.hits.store(99, Ordering::Relaxed);
    assert_eq!(
        restored.misses.load(Ordering::Relaxed),
        5,
        "after restore, a write through one field is invisible through the other"
    );
    // ... and the allocation they used to share is untouched by either, still
    // carrying the value it was moved to before the restore.
    assert_eq!(
        live.hits.load(Ordering::Relaxed),
        77,
        "the pre-restore allocation must not see a write through either \
         restored field"
    );
}

// ---------------------------------------------------------------------------
// the reconstruct escape, declared AND inferred
// ---------------------------------------------------------------------------

/// Stands in for a resource: no `CerulionState`, no `Default` — exactly the
/// shape that makes a `Default`-bound `cer_read` the wrong design.
#[derive(Debug)]
struct DeviceHandle {
    _fd: i32,
}

#[derive(CerulionState)]
struct Slam {
    pose: Pose,
    #[cerulion(reconstruct)]
    device: DeviceHandle,
}

#[test]
fn a_reconstruct_field_is_neither_captured_nor_restored_and_the_shape_says_so() {
    let slam = Slam {
        pose: Pose { x: 1.0, y: 2.0 },
        device: DeviceHandle { _fd: 3 },
    };

    // Captures the POSE only — the handle contributes no bytes.
    let mut want = Vec::new();
    want.extend_from_slice(&1.0f64.to_le_bytes());
    want.extend_from_slice(&2.0f64.to_le_bytes());
    assert_eq!(capture(&slam), want);

    // The escape kind is in the shape, so ADDING the attribute is a real
    // change a recording can see.
    let with_escape = StateShape::of("Slam")
        .field("pose", <Pose as CerulionState>::STATE_SHAPE)
        .field("device", StateShape::of("cerulion::reconstruct").finish())
        .finish();
    assert_eq!(<Slam as CerulionState>::STATE_SHAPE, with_escape);

    // Restore leaves the handle UNTOUCHED — the decisive reason the derive
    // overrides `cer_restore` rather than replacing the whole value.
    let mut live = Slam {
        pose: Pose::default(),
        device: DeviceHandle { _fd: 42 },
    };
    let mut cursor = StateCursor::new(&want);
    live.cer_restore(&mut cursor).expect("restore");
    cursor.finish().expect("blob fully consumed");
    assert_eq!(live.pose, Pose { x: 1.0, y: 2.0 });
    assert_eq!(live.device._fd, 42, "the handle must survive the restore");

    // `cer_read` cannot construct one, and says so rather than inventing a
    // value. It is not a path the runtime takes.
    let mut cursor = StateCursor::new(&want);
    assert!(matches!(
        Slam::cer_read(&mut cursor),
        Err(StateError::Unrestorable { type_name: "Slam" })
    ));
}

/// The decision-52(c) shape: a handle the inventory recognises, no attribute.
#[derive(Default, CerulionState)]
struct Bridge {
    frames: u64,
    transport: Option<Arc<std::net::TcpStream>>,
    sink: Option<Box<dyn std::fmt::Debug + Send>>,
}

#[test]
fn a_recognised_resource_is_reconstructed_with_no_tag_at_all() {
    // `TcpStream` is in the curated inventory; `dyn Debug` is structural.
    // Neither field carries an attribute and the struct still compiles.
    let mut bridge = Bridge {
        frames: 5,
        ..Default::default()
    };
    assert_eq!(capture(&bridge), 5u64.to_le_bytes().to_vec());
    assert_eq!(<Bridge as CerulionState>::MIN_ENCODED_BYTES, 8);
    assert!(inline_safe::<Bridge>());

    // Both handles survive a restore untouched — which is also what proves
    // they were classified as resources rather than quietly captured.
    bridge.sink = Some(Box::new("live"));
    let recorded = capture(&Bridge {
        frames: 9,
        ..Default::default()
    });
    let mut cursor = StateCursor::new(&recorded);
    bridge.cer_restore(&mut cursor).expect("restore");
    cursor.finish().expect("blob fully consumed");
    assert_eq!(bridge.frames, 9);
    assert!(bridge.transport.is_none());
    assert!(
        bridge.sink.is_some(),
        "a recognised resource must be left for `restored()` to rebuild"
    );
}

// ---------------------------------------------------------------------------
// enums
// ---------------------------------------------------------------------------

#[derive(Debug, PartialEq, CerulionState)]
enum PadEvent {
    Disconnected,
    Button { id: u8, down: bool },
    Axis(u8, f32),
}

#[test]
fn an_enum_rides_a_one_byte_tag_then_its_members() {
    // HAND ORACLE per variant.
    assert_eq!(capture(&PadEvent::Disconnected), vec![0]);
    assert_eq!(
        capture(&PadEvent::Button { id: 3, down: true }),
        vec![1, 3, 1]
    );
    let mut want = vec![2, 9];
    want.extend_from_slice(&0.5f32.to_le_bytes());
    assert_eq!(capture(&PadEvent::Axis(9, 0.5)), want);

    assert_eq!(read_exact::<PadEvent>(&[0]), PadEvent::Disconnected);
    assert_eq!(
        read_exact::<PadEvent>(&[1, 3, 1]),
        PadEvent::Button { id: 3, down: true }
    );
    assert_eq!(read_exact::<PadEvent>(&want), PadEvent::Axis(9, 0.5));
}

#[test]
fn an_unknown_enum_tag_is_refused_by_name() {
    let mut cursor = StateCursor::new(&[9]);
    assert!(matches!(
        PadEvent::cer_read(&mut cursor),
        Err(StateError::InvalidTag {
            type_name: "PadEvent",
            tag: 9
        })
    ));
}

#[test]
fn an_enum_restores_by_replacing_the_value_so_its_variant_can_change() {
    let recorded = capture(&PadEvent::Axis(1, 2.0));
    let mut live = PadEvent::Disconnected;
    let mut cursor = StateCursor::new(&recorded);
    live.cer_restore(&mut cursor).expect("restore");
    assert_eq!(live, PadEvent::Axis(1, 2.0));
}

#[test]
fn an_enums_shape_folds_variant_names_arity_and_order() {
    #[derive(CerulionState)]
    enum Reordered {
        Button { id: u8, down: bool },
        Disconnected,
        Axis(u8, f32),
    }
    #[derive(CerulionState)]
    enum Widened {
        Disconnected,
        Button { id: u8, down: bool, held: bool },
        Axis(u8, f32),
    }
    assert_ne!(
        <PadEvent as CerulionState>::STATE_SHAPE,
        <Reordered as CerulionState>::STATE_SHAPE,
        "reordering variants changes the TAGS, so it must change the shape"
    );
    assert_ne!(
        <PadEvent as CerulionState>::STATE_SHAPE,
        <Widened as CerulionState>::STATE_SHAPE,
        "a member added to a variant must change the shape"
    );
}

#[test]
fn a_named_variants_field_names_are_folded_so_a_transposition_cannot_hide() {
    // The transposition trap, INSIDE an enum. A named variant captures and
    // reads its members in DECLARATION order, so swapping two same-typed ones
    // is a byte-compatible source edit — and under a purely positional fold
    // the shape does not move, which is exactly the silent mis-restore
    // `StateShape::field` exists to refuse (and the reason a tuple STRUCT is
    // refused outright). Same enum name, same variant name, same member types;
    // only the NAMES and their ORDER differ, so nothing else can explain a
    // difference in the shapes.
    #[derive(CerulionState)]
    enum Goal {
        Point { x: f64, y: f64 },
    }
    #[derive(CerulionState)]
    enum GoalSwapped {
        Point { y: f64, x: f64 },
    }
    #[derive(CerulionState)]
    enum GoalRenamed {
        Point { u: f64, v: f64 },
    }

    // A hand-written oracle, so this is not the derive agreeing with itself:
    // the variant's members fold through `.field(name, ..)` exactly as a
    // struct's do.
    let want = StateShape::of("Goal")
        .field(
            "Point",
            StateShape::of("Point")
                .field("x", <f64 as CerulionState>::STATE_SHAPE)
                .field("y", <f64 as CerulionState>::STATE_SHAPE)
                .count(2)
                .finish(),
        )
        .count(1)
        .finish();
    assert_eq!(<Goal as CerulionState>::STATE_SHAPE, want);

    assert_ne!(
        <Goal as CerulionState>::STATE_SHAPE,
        <GoalSwapped as CerulionState>::STATE_SHAPE,
        "swapping two same-typed NAMED variant members must change the shape: \
         they capture and read in declaration order, so a recording of one \
         restores into the other transposed"
    );
    assert_ne!(
        <Goal as CerulionState>::STATE_SHAPE,
        <GoalRenamed as CerulionState>::STATE_SHAPE,
        "renaming a named variant member must change the shape"
    );

    // The transposition is REAL, not merely conceivable: the two enums agree
    // byte for byte, so the shape is the only thing standing between a
    // recording of one and a wrong-field restore into the other.
    let bytes = capture(&Goal::Point { x: 1.0, y: 2.0 });
    let GoalSwapped::Point { x, y } = read_exact::<GoalSwapped>(&bytes);
    assert_eq!(
        (x, y),
        (2.0, 1.0),
        "the bytes really do decode transposed — which is what the shape must refuse"
    );
}

#[test]
fn a_tuple_variants_members_stay_positional_so_only_named_shapes_moved() {
    // The scope pin for the fold above, in the direction that costs bytes.
    // Folding names for NAMED variants changes `STATE_SHAPE` for those enums
    // and MUST NOT change it for anything else, so a unit-only and a
    // tuple-only enum are asserted against hand oracles built the old way —
    // `.element(..)`, no names. A regression that folded a positional index
    // (or a synthesised `"0"`/`"1"`) into a tuple variant fails here.
    #[derive(CerulionState)]
    enum UnitOnly {
        A,
        B,
    }
    #[derive(CerulionState)]
    enum TupleOnly {
        A(u32),
        B(f64, f64),
    }

    let unit_want = StateShape::of("UnitOnly")
        .field("A", StateShape::of("A").count(0).finish())
        .field("B", StateShape::of("B").count(0).finish())
        .count(2)
        .finish();
    assert_eq!(<UnitOnly as CerulionState>::STATE_SHAPE, unit_want);

    let tuple_want = StateShape::of("TupleOnly")
        .field(
            "A",
            StateShape::of("A")
                .element(<u32 as CerulionState>::STATE_SHAPE)
                .count(1)
                .finish(),
        )
        .field(
            "B",
            StateShape::of("B")
                .element(<f64 as CerulionState>::STATE_SHAPE)
                .element(<f64 as CerulionState>::STATE_SHAPE)
                .count(2)
                .finish(),
        )
        .count(2)
        .finish();
    assert_eq!(<TupleOnly as CerulionState>::STATE_SHAPE, tuple_want);
}

// ---------------------------------------------------------------------------
// the unordered escape
// ---------------------------------------------------------------------------

#[derive(Default, CerulionState)]
struct Grid {
    #[cerulion(unordered)]
    cells: HashMap<FloatCell, u8>,
}

#[test]
fn an_unordered_map_captures_a_key_that_has_no_total_order() {
    // Without the escape this field would not compile: `CanonicalMapKey`
    // requires `Ord`, which a float-derived cell cannot have.
    let mut grid = Grid::default();
    grid.cells.insert(FloatCell { bits: 7 }, 1);

    let mut want = Vec::new();
    want.extend_from_slice(&1u32.to_le_bytes());
    want.extend_from_slice(&7u64.to_le_bytes());
    want.push(1);
    assert_eq!(capture(&grid), want);

    let restored: Grid = read_exact(&want);
    assert_eq!(restored.cells.get(&FloatCell { bits: 7 }), Some(&1));
}

#[test]
fn an_unordered_map_refuses_a_duplicate_rather_than_silently_dropping_one() {
    // `read_unordered_entries` carries no `Eq` bound, so the refusal is the
    // COLLECTING caller's job — the derive's. Left to `insert`, the restored
    // map would hold FEWER entries than the blob declared and nothing would
    // report it.
    let mut hostile = Vec::new();
    hostile.extend_from_slice(&2u32.to_le_bytes());
    hostile.extend_from_slice(&7u64.to_le_bytes());
    hostile.push(1);
    hostile.extend_from_slice(&7u64.to_le_bytes());
    hostile.push(2);

    let mut cursor = StateCursor::new(&hostile);
    assert!(matches!(
        Grid::cer_read(&mut cursor),
        Err(StateError::DuplicateEntry {
            type_name: "HashMap"
        })
    ));

    // The same refusal on the in-place restore path.
    let mut live = Grid::default();
    let mut cursor = StateCursor::new(&hostile);
    assert!(matches!(
        live.cer_restore(&mut cursor),
        Err(StateError::DuplicateEntry { .. })
    ));
}

#[test]
fn an_unordered_field_has_its_own_shape_so_the_escape_is_visible() {
    #[derive(Default, CerulionState)]
    struct OrderedGrid {
        cells: HashMap<u64, u8>,
    }
    // Different key type, so this is not a strict A/B — what it pins is that
    // the unordered marker really reaches the shape.
    let want = StateShape::of("Grid")
        .field(
            "cells",
            StateShape::of("cerulion::unordered")
                .element(<FloatCell as CerulionState>::STATE_SHAPE)
                .element(<u8 as CerulionState>::STATE_SHAPE)
                .finish(),
        )
        .finish();
    assert_eq!(<Grid as CerulionState>::STATE_SHAPE, want);
    assert_ne!(
        <Grid as CerulionState>::STATE_SHAPE,
        <OrderedGrid as CerulionState>::STATE_SHAPE
    );
}

// ---------------------------------------------------------------------------
// the serde escape
// ---------------------------------------------------------------------------

/// Stands in for a foreign math type: `Serialize`/`Deserialize` but no
/// `CerulionState`, and not ours to change.
#[derive(Debug, Default, PartialEq, cerulion_core::serde::Serialize)]
#[serde(crate = "cerulion_core::serde")]
struct Isometry {
    t: [f64; 3],
}

impl<'de> cerulion_core::serde::Deserialize<'de> for Isometry {
    fn deserialize<D>(d: D) -> Result<Self, D::Error>
    where
        D: cerulion_core::serde::Deserializer<'de>,
    {
        #[derive(cerulion_core::serde::Deserialize)]
        #[serde(crate = "cerulion_core::serde")]
        struct Raw {
            t: [f64; 3],
        }
        Raw::deserialize(d).map(|r| Isometry { t: r.t })
    }
}

#[derive(Default, CerulionState)]
struct Localizer {
    #[cerulion(serde)]
    iso: Isometry,
}

#[test]
fn a_serde_field_round_trips_and_forfeits_the_inline_fast_path() {
    let mut node = Localizer::default();
    node.iso.t = [1.0, 2.0, 3.0];

    let bytes = capture(&node);
    // HAND ORACLE on the FRAMING (a u32 length prefix); the payload is the
    // field's own `Serialize`, which is deliberately not this module's
    // business.
    let declared = u32::from_le_bytes(bytes[..4].try_into().expect("prefix")) as usize;
    assert_eq!(declared, bytes.len() - 4);

    let restored: Localizer = read_exact(&bytes);
    assert_eq!(restored.iso.t, [1.0, 2.0, 3.0]);

    // A user-written encoder can block, so the byte bound stops being a
    // time bound and the node must take the fork carrier.
    assert!(!inline_safe::<Localizer>());
}

/// Frame a JSON payload the way `capture_serde_field` does, so these blobs are
/// well-formed AT THE FRAMING and the only thing under test is what the
/// field's own `Deserialize` makes of the bytes.
fn serde_field_blob(json: &[u8]) -> Vec<u8> {
    let mut blob = Vec::new();
    blob.extend_from_slice(&(json.len() as u32).to_le_bytes());
    blob.extend_from_slice(json);
    blob
}

#[test]
fn a_serde_field_reports_its_own_impls_refusal_rather_than_guessing() {
    // Truncated JSON: the field's `Deserialize` refuses, and the error names
    // the FIELD so an operator knows which one to look at.
    let blob = serde_field_blob(b"{ t");
    let mut cursor = StateCursor::new(&blob);
    assert!(matches!(
        Localizer::cer_read(&mut cursor),
        Err(StateError::SerdeDecode { field: "iso", .. })
    ));
}

#[test]
fn a_serde_refusal_separates_a_broken_recording_from_a_type_that_moved() {
    // The field alone cannot say WHICH fault this is, and the two have
    // opposite remedies: one says the bag is unusable, the other says the
    // node's own type changed since it was recorded. A `#[cerulion(serde)]`
    // field folds only its NAME into `STATE_SHAPE`, so a retype is invisible
    // to the shape check and lands here — which makes this the only place the
    // distinction can be drawn at all.
    // `Localizer` is deliberately not `Debug` (a node's state need not be), so
    // the error is taken by match rather than `expect_err`.
    let refusal = |blob: &[u8]| -> (&'static str, String) {
        let mut cursor = StateCursor::new(blob);
        match Localizer::cer_read(&mut cursor) {
            Ok(_) => panic!("this blob must not decode"),
            Err(StateError::SerdeDecode { field, cause }) => (field, cause),
            Err(other) => panic!("a serde field's refusal must be a serde error: {other}"),
        }
    };

    // A blob the recording never finished writing: valid JSON as far as it
    // goes, then nothing.
    let (field, corrupt_cause) = refusal(&serde_field_blob(br#"{"t":[1.0,2.0"#));
    assert_eq!(field, "iso");

    // Well-formed JSON the RUNNING type refuses: `t` was recorded as a string
    // by a version of this node that has since been retyped.
    let (field, retyped_cause) = refusal(&serde_field_blob(br#"{"t":"nope"}"#));
    assert_eq!(field, "iso");

    // THE assertion: the two faults are distinguishable. Dropping the cause
    // collapses them onto one string and an operator cannot tell which
    // remedy applies.
    assert_ne!(
        corrupt_cause, retyped_cause,
        "a corrupt recording and a retyped field must not read identically"
    );
    // ... and each says which it is, in the vocabulary serde itself uses.
    assert!(
        corrupt_cause.contains("EOF") || corrupt_cause.contains("eof"),
        "a truncated blob reads as an unexpected end of input: {corrupt_cause}"
    );
    assert!(
        retyped_cause.contains("string"),
        "a retyped field names the type it actually found: {retyped_cause}"
    );

    // The Display an operator sees carries it too — the struct field is no
    // use to somebody reading a log line.
    let rendered = StateError::SerdeDecode {
        field: "iso",
        cause: retyped_cause.clone(),
    }
    .to_string();
    assert!(rendered.contains("iso"), "{rendered}");
    assert!(rendered.contains(&retyped_cause), "{rendered}");
}

// ---------------------------------------------------------------------------
// generics
// ---------------------------------------------------------------------------

#[derive(Debug, Default, PartialEq, CerulionState)]
struct Holder<T> {
    value: T,
    count: u32,
}

#[test]
fn a_generic_struct_resolves_at_the_instantiation_not_at_the_definition() {
    // The alternative was measured: an unbounded `Foo<T>`
    // silently loses its payload for EVERY instantiation, including a
    // perfectly capturable one.
    let holder = Holder {
        value: 7u64,
        count: 2,
    };
    let mut want = Vec::new();
    want.extend_from_slice(&7u64.to_le_bytes());
    want.extend_from_slice(&2u32.to_le_bytes());
    assert_eq!(capture(&holder), want);
    assert_eq!(read_exact::<Holder<u64>>(&want), holder);

    // Two instantiations are DIFFERENT shapes, which a token hash could not
    // distinguish.
    assert_ne!(
        <Holder<u64> as CerulionState>::STATE_SHAPE,
        <Holder<Vec<u64>> as CerulionState>::STATE_SHAPE
    );
    // ... and `INLINE_SAFE` folds THROUGH the parameter.
    assert!(inline_safe::<Holder<u64>>());
    assert!(!inline_safe::<Holder<Mutex<u64>>>());
}

// ---------------------------------------------------------------------------
// determinism
// ---------------------------------------------------------------------------

#[test]
fn two_captures_of_one_value_are_byte_identical_and_equal_the_oracle() {
    let odom = Odom {
        pose: Pose { x: 0.25, y: 0.5 },
        seq: 3,
        label: "det".to_string(),
    };
    let mut oracle = Vec::new();
    oracle.extend_from_slice(&0.25f64.to_le_bytes());
    oracle.extend_from_slice(&0.5f64.to_le_bytes());
    oracle.extend_from_slice(&3u32.to_le_bytes());
    oracle.extend_from_slice(&3u32.to_le_bytes());
    oracle.extend_from_slice(b"det");

    let first = capture(&odom);
    let second = capture(&odom);
    assert_eq!(first, second);
    assert_eq!(
        first, oracle,
        "and both equal a HAND oracle, not each other"
    );
}
