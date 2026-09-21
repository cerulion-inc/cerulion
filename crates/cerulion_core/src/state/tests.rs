//! The state-capture core's test gate.
//!
//! Every assertion here is against a **hand-written oracle** — a byte vector
//! assembled by hand, a hand-folded FNV stream, an explicitly enumerated
//! `INLINE_SAFE` table — never against a second run of the code under test.
//!
//! Pure: no transport, no iceoryx2, no shared-memory singleton, so the whole
//! file is parallel-safe and needs no `--test-threads=1`.

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};
use std::num::{NonZeroU32, Wrapping};
use std::path::PathBuf;
use std::sync::atomic::{
    AtomicBool, AtomicI64, AtomicI8, AtomicIsize, AtomicU32, AtomicU64, AtomicU8, AtomicUsize,
    Ordering,
};
use std::sync::mpsc;
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use indexmap::{IndexMap, IndexSet};

use super::{
    capture_unordered_map, capture_unordered_set, read_unordered_elements, read_unordered_entries,
    BoundedSink, CerulionState, StateCursor, StateError, StateShape, StateSink, VecSink,
    CAPTURE_INLINE_BUDGET_BYTES,
};
use crate::wire::fnv1a_hash;

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

/// Canonical bytes for a value, through the unbounded sink.
fn capture(value: &impl CerulionState) -> Vec<u8> {
    let mut sink = VecSink::new();
    value
        .cer_capture(&mut sink)
        .expect("unbounded capture cannot fail");
    sink.into_inner()
}

/// Decode a value and assert the blob was fully consumed.
fn read_exact<T: CerulionState>(bytes: &[u8]) -> T {
    let mut cursor = StateCursor::new(bytes);
    let value = T::cer_read(&mut cursor).expect("decode");
    cursor.finish().expect("blob fully consumed");
    value
}

/// The nested-composite round-trip fixture's type, named so the round-trip
/// test can spell it twice without tripping `clippy::type_complexity`.
type NestedComposite = Vec<(String, Option<BTreeMap<u8, [u16; 2]>>)>;

/// A `u32` LE count prefix, for hand-built oracles.
fn count_bytes(n: u32) -> Vec<u8> {
    n.to_le_bytes().to_vec()
}

/// Read a type's `MIN_ENCODED_BYTES` through a function.
fn floor_of<T: CerulionState>() -> usize {
    T::MIN_ENCODED_BYTES
}

/// Read a type's `INLINE_SAFE` through a function.
///
/// The assertions below are all about compile-time constants, and
/// `assert!(SOME_CONST)` is what `clippy::assertions_on_constants` forbids —
/// but a `const` block would defeat the purpose, since the whole point is that
/// a WRONG constant must fail as a test, with a message naming the type,
/// rather than as a build error in the middle of an unrelated `cargo build`.
fn inline_safe<T: CerulionState>() -> bool {
    T::INLINE_SAFE
}

/// Element decodes performed by [`CountedUnit`], for the work oracle.
static ELEMENT_READS: AtomicU64 = AtomicU64::new(0);

/// A ZERO-encoding element that COUNTS its own decodes.
///
/// The work bound cannot be asserted with a wall — a wall tight enough to
/// separate linear from quadratic is also tight enough for a loaded runner to
/// invert — so the loop itself has to be observable.
#[derive(Debug, PartialEq, Eq)]
struct CountedUnit;

impl CerulionState for CountedUnit {
    const STATE_SHAPE: u64 = StateShape::of("CountedUnit").finish();

    fn cer_capture(&self, _out: &mut dyn StateSink) -> Result<(), StateError> {
        Ok(())
    }

    fn cer_read(_src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        ELEMENT_READS.fetch_add(1, Ordering::Relaxed);
        Ok(CountedUnit)
    }
}

/// A type with a HAND-WRITTEN impl that declares neither `INLINE_SAFE` nor
/// `cer_probe` — the risk-6b fixture.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
struct HandWritten(u32);

impl CerulionState for HandWritten {
    const STATE_SHAPE: u64 = StateShape::of("HandWritten")
        .field("0", <u32 as CerulionState>::STATE_SHAPE)
        .finish();

    fn cer_capture(&self, out: &mut dyn StateSink) -> Result<(), StateError> {
        self.0.cer_capture(out)
    }

    fn cer_read(src: &mut StateCursor<'_>) -> Result<Self, StateError> {
        Ok(HandWritten(u32::cer_read(src)?))
    }
}

// ---------------------------------------------------------------------------
// BoundedSink — the exact-boundary contract
// ---------------------------------------------------------------------------

#[test]
fn a_write_landing_exactly_at_the_boundary_is_accepted_and_one_byte_over_is_refused() {
    // AT the boundary: accepted, in one write.
    let mut arena = [0u8; 16];
    let mut sink = BoundedSink::with_capacity(&mut arena, 8);
    assert_eq!(sink.write(&[1, 2, 3, 4, 5, 6, 7, 8]), Ok(()));
    assert_eq!(sink.len(), 8);
    assert!(!sink.refused());
    assert_eq!(sink.written(), &[1, 2, 3, 4, 5, 6, 7, 8]);

    // ONE byte over: refused, and nothing is written.
    let mut arena = [0u8; 16];
    let mut sink = BoundedSink::with_capacity(&mut arena, 8);
    assert_eq!(
        sink.write(&[1, 2, 3, 4, 5, 6, 7, 8, 9]),
        Err(super::SinkFull)
    );
    assert_eq!(sink.len(), 0, "a refused write must write NOTHING");
    assert!(sink.refused());

    // AT the boundary across two writes: accepted.
    let mut arena = [0u8; 16];
    let mut sink = BoundedSink::with_capacity(&mut arena, 8);
    assert_eq!(sink.write(&[1, 2, 3, 4, 5]), Ok(()));
    assert_eq!(sink.write(&[6, 7, 8]), Ok(()));
    assert_eq!(sink.len(), 8);
    assert!(!sink.refused());

    // ONE byte over across two writes: the second is refused and the first
    // write's bytes survive untouched.
    let mut arena = [0u8; 16];
    let mut sink = BoundedSink::with_capacity(&mut arena, 8);
    assert_eq!(sink.write(&[1, 2, 3, 4, 5]), Ok(()));
    assert_eq!(sink.write(&[6, 7, 8, 9]), Err(super::SinkFull));
    assert_eq!(sink.len(), 5);
    assert_eq!(sink.written(), &[1, 2, 3, 4, 5]);
}

#[test]
fn a_zero_capacity_sink_accepts_an_empty_write_and_refuses_any_byte() {
    let mut arena = [0u8; 4];
    let mut sink = BoundedSink::with_capacity(&mut arena, 0);
    assert_eq!(sink.write(&[]), Ok(()), "0 <= 0 is at the boundary");
    assert!(!sink.refused());
    assert_eq!(sink.write(&[1]), Err(super::SinkFull));
    assert!(sink.refused());
}

#[test]
fn a_refused_sink_latches_so_a_later_small_write_cannot_land_after_a_hole() {
    let mut arena = [0u8; 16];
    let mut sink = BoundedSink::with_capacity(&mut arena, 8);
    assert_eq!(sink.write(&[1, 2, 3, 4]), Ok(()));
    // Refused: would need 9 of 8.
    assert_eq!(sink.write(&[5, 6, 7, 8, 9]), Err(super::SinkFull));
    // This one FITS in the remaining 4 bytes — and must still be refused,
    // because accepting it would emit a byte stream missing a field in the
    // middle that no decoder could detect.
    assert_eq!(sink.write(&[9]), Err(super::SinkFull));
    assert_eq!(sink.len(), 4);
    assert_eq!(sink.remaining_hint(), Some(0));
}

#[test]
fn an_aborted_sink_charges_its_whole_granted_capacity_to_the_shared_budget() {
    // Healthy sink: charged exactly what it wrote.
    let mut arena = [0u8; 128];
    let mut sink = BoundedSink::with_capacity(&mut arena, 100);
    assert_eq!(sink.write(&[0u8; 60]), Ok(()));
    assert_eq!(sink.consumed(), 60);
    assert_eq!(sink.len(), 60);

    // Aborted sink: charged its WHOLE capacity, not the 60 bytes it wrote.
    let mut arena = [0u8; 128];
    let mut sink = BoundedSink::with_capacity(&mut arena, 100);
    assert_eq!(sink.write(&[0u8; 60]), Ok(()));
    assert_eq!(sink.write(&[0u8; 60]), Err(super::SinkFull));
    assert_eq!(sink.len(), 60);
    assert_eq!(
        sink.consumed(),
        100,
        "an aborted sink charges its whole granted capacity, so at most ONE \
         node per boundary can pay for a refusal"
    );
}

#[test]
fn an_encoder_that_refuses_before_writing_charges_exactly_like_one_that_refuses_mid_write() {
    // A hash-like encoder consults `remaining_hint` and can refuse having
    // called `write` ZERO times (it must decide before building its sort
    // index). Such a refusal has to latch and charge like any other, or the
    // boundary walk hands the same budget to the next node — an UNCHARGED path
    // through "at most one node per boundary pays for a refusal".
    const CAP: usize = 103;
    let map: HashMap<u8, u8> = (0u8..100).map(|k| (k, k)).collect();

    let mut arena = [0u8; 256];
    let mut pre_check = BoundedSink::with_capacity(&mut arena, CAP);
    assert_eq!(map.cer_capture(&mut pre_check), Err(StateError::SinkFull));
    assert_eq!(pre_check.len(), 0, "it never wrote a byte");
    assert!(pre_check.refused());
    assert_eq!(pre_check.consumed(), CAP);
    assert_eq!(pre_check.remaining_hint(), Some(0));

    // The mid-write refusal, at the same capacity, for comparison.
    let mut arena = [0u8; 256];
    let mut mid_write = BoundedSink::with_capacity(&mut arena, CAP);
    assert_eq!(mid_write.write(&[0u8; 200]), Err(super::SinkFull));
    assert_eq!(
        pre_check.consumed(),
        mid_write.consumed(),
        "both refusals charge the boundary identically"
    );

    // And the latch holds: a later write that WOULD fit is still refused, so
    // the aborted encoding cannot be completed with a hole in it.
    assert_eq!(pre_check.write(&[1]), Err(super::SinkFull));
    assert_eq!(pre_check.len(), 0);
}

#[test]
fn the_shared_budget_bounds_the_whole_boundary_walk_however_many_nodes_overflow() {
    // Model the inline walk over an arena: node 0 overflows, so nodes 1 and 2
    // get a zero-byte sink and refuse at their first write.
    const BUDGET: usize = 64;
    let mut arena = [0u8; BUDGET];
    let mut used = 0usize;
    let mut carriers = Vec::new();

    for node in 0..3 {
        let mut sink = BoundedSink::with_capacity(&mut arena[used..], BUDGET - used);
        // Node 0 asks for more than the whole budget; the others ask for 4.
        let ask = if node == 0 { BUDGET + 1 } else { 4 };
        let outcome = sink.write(&vec![0u8; ask]);
        used += sink.consumed();
        carriers.push(outcome.is_ok());
    }

    assert_eq!(
        carriers,
        vec![false, false, false],
        "the first overflow pushes the rest of the boundary's nodes into the fork set"
    );
    assert_eq!(
        used, BUDGET,
        "the walk spends exactly one budget, never three"
    );
    assert!(used <= BUDGET);
}

#[test]
fn a_capacity_above_the_slice_clamps_to_the_slice() {
    let mut arena = [0u8; 4];
    let mut sink = BoundedSink::with_capacity(&mut arena, usize::MAX);
    assert_eq!(sink.capacity(), 4);
    assert_eq!(sink.write(&[1, 2, 3, 4]), Ok(()));
    assert_eq!(sink.write(&[5]), Err(super::SinkFull));
}

#[test]
fn a_sink_reports_its_remaining_capacity_and_an_unbounded_one_reports_none() {
    let mut arena = [0u8; 10];
    let mut sink = BoundedSink::new(&mut arena);
    assert_eq!(sink.remaining_hint(), Some(10));
    assert!(sink.is_empty());
    sink.write(&[0u8; 3]).expect("fits");
    assert_eq!(sink.remaining_hint(), Some(7));
    assert!(!sink.is_empty());

    let unbounded = VecSink::new();
    assert_eq!(
        unbounded.remaining_hint(),
        None,
        "the fork child's ring sink must not make encoders refuse early"
    );
}

#[test]
fn the_inline_budget_constant_is_the_documented_64_kib() {
    assert_eq!(CAPTURE_INLINE_BUDGET_BYTES, 65_536);
}

// ---------------------------------------------------------------------------
// the canonical sorted serializer
// ---------------------------------------------------------------------------

#[test]
fn the_sorted_map_encoding_matches_a_hand_built_byte_oracle() {
    let mut map: HashMap<u32, u32> = HashMap::new();
    map.insert(3, 30);
    map.insert(1, 10);
    map.insert(2, 20);

    let mut oracle = count_bytes(3);
    for (key, value) in [(1u32, 10u32), (2, 20), (3, 30)] {
        oracle.extend_from_slice(&key.to_le_bytes());
        oracle.extend_from_slice(&value.to_le_bytes());
    }

    assert_eq!(capture(&map), oracle);
}

#[test]
fn a_hash_map_encodes_byte_identically_however_its_iteration_order_falls() {
    // Every permutation of the same logical map. Each `HashMap::new()` mints a
    // fresh `RandomState`, so iteration order varies map to map.
    const KEYS: [u32; 4] = [7, 3, 9, 1];
    let permutations: Vec<Vec<u32>> = permutations_of(&KEYS);
    assert_eq!(permutations.len(), 24);

    let mut oracle = count_bytes(4);
    for key in [1u32, 3, 7, 9] {
        oracle.extend_from_slice(&key.to_le_bytes());
        oracle.extend_from_slice(&(key * 10).to_le_bytes());
    }

    let mut observed_iteration_orders: HashSet<Vec<u32>> = HashSet::new();
    for permutation in &permutations {
        let mut map: HashMap<u32, u32> = HashMap::new();
        for &key in permutation {
            map.insert(key, key * 10);
        }
        observed_iteration_orders.insert(map.keys().copied().collect());
        assert_eq!(
            capture(&map),
            oracle,
            "insertion order {permutation:?} must not reach the bag"
        );
    }

    // ANTI-VACUITY, and it is what makes the assertion above a PROOF rather
    // than a probability: at least two distinct iteration orders really
    // occurred, so at most one of them could have coincided with sorted order
    // — an unsorted encoder therefore MUST have failed at least one arm above.
    assert!(
        observed_iteration_orders.len() >= 2,
        "the test went vacuous: every map iterated in the same order, so an \
         unsorted encoder could have passed. Observed: {observed_iteration_orders:?}"
    );
}

/// Every permutation of a 4-element key set, by hand (no crate, no recursion
/// into the code under test).
fn permutations_of(keys: &[u32; 4]) -> Vec<Vec<u32>> {
    let mut out = Vec::new();
    for a in 0..4 {
        for b in 0..4 {
            for c in 0..4 {
                for d in 0..4 {
                    let idx = [a, b, c, d];
                    let mut seen = [false; 4];
                    if idx.iter().all(|&i| !std::mem::replace(&mut seen[i], true)) {
                        out.push(idx.iter().map(|&i| keys[i]).collect());
                    }
                }
            }
        }
    }
    out
}

#[test]
fn a_hash_set_encodes_in_sorted_order_however_it_iterates() {
    let mut oracle = count_bytes(4);
    for key in [1u32, 3, 7, 9] {
        oracle.extend_from_slice(&key.to_le_bytes());
    }

    let mut observed_iteration_orders: HashSet<Vec<u32>> = HashSet::new();
    for permutation in permutations_of(&[7, 3, 9, 1]) {
        let set: HashSet<u32> = permutation.iter().copied().collect();
        observed_iteration_orders.insert(set.iter().copied().collect());
        assert_eq!(capture(&set), oracle);
    }
    assert!(
        observed_iteration_orders.len() >= 2,
        "the test went vacuous: every set iterated in the same order"
    );
}

#[test]
fn a_btree_map_encodes_in_its_own_order_and_sorts_nothing() {
    let map: BTreeMap<u32, u32> = [(3, 30), (1, 10), (2, 20)].into_iter().collect();
    let mut oracle = count_bytes(3);
    for (key, value) in [(1u32, 10u32), (2, 20), (3, 30)] {
        oracle.extend_from_slice(&key.to_le_bytes());
        oracle.extend_from_slice(&value.to_le_bytes());
    }
    assert_eq!(capture(&map), oracle);

    let set: BTreeSet<u32> = [3, 1, 2].into_iter().collect();
    let mut set_oracle = count_bytes(3);
    for key in [1u32, 2, 3] {
        set_oracle.extend_from_slice(&key.to_le_bytes());
    }
    assert_eq!(capture(&set), set_oracle);
}

#[test]
fn an_index_map_encodes_in_insertion_order_which_is_its_own_determinism_contract() {
    let forward: IndexMap<u32, u32> = [(3, 30), (1, 10)].into_iter().collect();
    let reverse: IndexMap<u32, u32> = [(1, 10), (3, 30)].into_iter().collect();

    let mut forward_oracle = count_bytes(2);
    for (key, value) in [(3u32, 30u32), (1, 10)] {
        forward_oracle.extend_from_slice(&key.to_le_bytes());
        forward_oracle.extend_from_slice(&value.to_le_bytes());
    }
    assert_eq!(capture(&forward), forward_oracle);

    // Insertion-ordered, so a different insertion order IS different state as
    // far as the bytes are concerned. That is the documented contract: an
    // `IndexMap` sorts nothing and pays no index.
    assert_ne!(capture(&forward), capture(&reverse));

    let set: IndexSet<u32> = [9u32, 2].into_iter().collect();
    let mut set_oracle = count_bytes(2);
    set_oracle.extend_from_slice(&9u32.to_le_bytes());
    set_oracle.extend_from_slice(&2u32.to_le_bytes());
    assert_eq!(capture(&set), set_oracle);
}

#[test]
fn a_map_whose_entry_count_alone_exceeds_the_sink_refuses_before_building_a_sort_index() {
    let map: HashMap<u8, u8> = (0u8..100).map(|k| (k, k)).collect();

    // 4-byte count + 2 bytes per entry = 204. A sink with exactly that fits.
    let mut arena = [0u8; 256];
    let mut sink = BoundedSink::with_capacity(&mut arena, 204);
    assert_eq!(map.cer_capture(&mut sink), Ok(()));
    assert_eq!(sink.len(), 204);

    // The ANTI-TAUTOLOGY half: the guard keys on ENTRY COUNT, so it must not
    // refuse a map whose count fits even when the encoding is tight.
    let mut arena = [0u8; 256];
    let mut sink = BoundedSink::with_capacity(&mut arena, 203);
    assert_eq!(map.cer_capture(&mut sink), Err(StateError::SinkFull));

    // Entry count alone (100 + 4) past the capacity: refused up front.
    let mut arena = [0u8; 256];
    let mut sink = BoundedSink::with_capacity(&mut arena, 103);
    assert_eq!(map.cer_capture(&mut sink), Err(StateError::SinkFull));
    assert_eq!(
        sink.len(),
        0,
        "the guard runs before the count prefix is even written"
    );
}

#[test]
fn an_exactly_fitting_zero_byte_container_is_captured_not_spuriously_refused() {
    // A one-entry `HashSet<()>` encodes as NOTHING but its four-byte count, so
    // a four-byte sink fits it exactly. Charging a flat one byte per entry
    // made `1 + 4 > 4` and latched the sink full, refusing state that fits —
    // the guard now charges the element type's own encoded-size FLOOR.
    let set: HashSet<()> = [()].into_iter().collect();
    assert_eq!(capture(&set).len(), 4, "count prefix only");

    let mut arena = [0u8; 8];
    let mut sink = BoundedSink::with_capacity(&mut arena, 4);
    assert_eq!(set.cer_capture(&mut sink), Ok(()));
    assert_eq!(sink.len(), 4);
    assert!(!sink.refused());

    // The map twin, same shape.
    let map: HashMap<(), ()> = [((), ())].into_iter().collect();
    let mut arena = [0u8; 8];
    let mut sink = BoundedSink::with_capacity(&mut arena, 4);
    assert_eq!(map.cer_capture(&mut sink), Ok(()));
    assert_eq!(sink.len(), 4);

    // The floor-free scratch disjunct does not fire here: one entry against
    // four remaining bytes is not more entries than bytes.
    //
    // GENUINELY over still refuses, so the guard is still in place:
    // three bytes cannot hold a four-byte count.
    let mut arena = [0u8; 8];
    let mut sink = BoundedSink::with_capacity(&mut arena, 3);
    assert_eq!(set.cer_capture(&mut sink), Err(StateError::SinkFull));
    assert!(sink.refused());
    assert_eq!(
        sink.consumed(),
        3,
        "and it still charges its whole capacity"
    );
}

#[test]
fn the_two_guards_compose_on_the_shape_that_drives_both() {
    // The capture guard's floor and the decode budget meet on ONE value: a
    // one-entry zero-encoding container is both an exact fit for a four-byte
    // sink AND a count of a zero-encoding element type. Fixing either guard in
    // isolation could re-break the other, so this arm drives both halves of a
    // single round trip.
    let set: HashSet<()> = [()].into_iter().collect();

    let mut arena = [0u8; 4];
    let mut sink = BoundedSink::with_capacity(&mut arena, 4);
    set.cer_capture(&mut sink)
        .expect("capture must fit exactly");
    let bytes = sink.written().to_vec();
    assert_eq!(bytes.len(), 4);

    // ... and the bytes it produced must restore through the element budget,
    // whose seed is those same four bytes.
    let mut cursor = StateCursor::new(&bytes);
    assert_eq!(cursor.element_budget(), 4);
    let restored: HashSet<()> = CerulionState::cer_read(&mut cursor).expect("restore");
    assert_eq!(restored, set);
    cursor.finish().expect("consumed");
}

#[test]
fn the_unordered_escape_hatch_round_trips_and_emits_iteration_order() {
    let map: IndexMap<u32, u32> = [(9, 90), (2, 20)].into_iter().collect();
    let mut sink = VecSink::new();
    capture_unordered_map(map.len(), map.iter(), &mut sink).expect("capture");

    let mut oracle = count_bytes(2);
    for (key, value) in [(9u32, 90u32), (2, 20)] {
        oracle.extend_from_slice(&key.to_le_bytes());
        oracle.extend_from_slice(&value.to_le_bytes());
    }
    assert_eq!(sink.as_slice(), oracle.as_slice());

    let mut cursor = StateCursor::new(&oracle);
    let restored: IndexMap<u32, u32> = read_unordered_entries::<u32, u32>(&mut cursor)
        .expect("restore unordered map")
        .into_iter()
        .collect();
    assert_eq!(restored, map);
    cursor.finish().expect("consumed");

    let set: IndexSet<u32> = [5u32, 1].into_iter().collect();
    let mut sink = VecSink::new();
    capture_unordered_set(set.len(), set.iter(), &mut sink).expect("capture");

    // The BYTES, not just the membership: an encoder that sorted (or appended
    // anything) would still restore to an equal set.
    let mut set_oracle = count_bytes(2);
    for key in [5u32, 1] {
        set_oracle.extend_from_slice(&key.to_le_bytes());
    }
    assert_eq!(sink.as_slice(), set_oracle.as_slice());

    let mut cursor = StateCursor::new(sink.as_slice());
    let restored: IndexSet<u32> = read_unordered_elements::<u32>(&mut cursor)
        .expect("restore")
        .into_iter()
        .collect();
    assert_eq!(restored, set);
    cursor.finish().expect("consumed");
}

// ---------------------------------------------------------------------------
// the encoding, against hand oracles
// ---------------------------------------------------------------------------

#[test]
fn scalars_encode_little_endian_at_their_declared_width() {
    assert_eq!(capture(&0x1234_5678u32), vec![0x78, 0x56, 0x34, 0x12]);
    assert_eq!(capture(&(-2i16)), vec![0xfe, 0xff]);
    assert_eq!(capture(&1.0f64), 1.0f64.to_le_bytes().to_vec());
    assert_eq!(capture(&true), vec![1]);
    assert_eq!(capture(&false), vec![0]);
    assert_eq!(capture(&'A'), vec![0x41, 0, 0, 0]);
    assert_eq!(capture(&()), Vec::<u8>::new());
}

#[test]
fn a_float_keeps_its_exact_bit_pattern_so_a_nan_payload_survives() {
    let nan = f64::from_bits(0x7ff8_0000_dead_beef);
    let bytes = capture(&nan);
    assert_eq!(bytes, nan.to_le_bytes().to_vec());
    let restored: f64 = read_exact(&bytes);
    assert_eq!(restored.to_bits(), nan.to_bits());
}

#[test]
fn usize_rides_a_fixed_64_bit_encoding_so_a_recording_is_portable() {
    assert_eq!(capture(&7usize), 7u64.to_le_bytes().to_vec());
    assert_eq!(capture(&(-7isize)), (-7i64).to_le_bytes().to_vec());
    assert_eq!(capture(&7usize).len(), 8);
    assert_eq!(read_exact::<usize>(&capture(&usize::MAX)), usize::MAX);
}

#[test]
fn text_encodes_as_a_u32_length_prefix_plus_utf8() {
    let mut oracle = count_bytes(5);
    oracle.extend_from_slice(b"hello");
    assert_eq!(capture(&"hello".to_owned()), oracle);
    assert_eq!(capture(&PathBuf::from("hello")), oracle);
    assert_eq!(capture(&Cow::Borrowed("hello")), oracle);
    assert_eq!(read_exact::<String>(&oracle), "hello");
    assert_eq!(read_exact::<PathBuf>(&oracle), PathBuf::from("hello"));
}

#[test]
fn a_sequence_carries_a_count_while_an_array_and_a_tuple_do_not() {
    let mut oracle = count_bytes(3);
    for value in [1u16, 2, 3] {
        oracle.extend_from_slice(&value.to_le_bytes());
    }
    assert_eq!(capture(&vec![1u16, 2, 3]), oracle);
    assert_eq!(capture(&VecDeque::from(vec![1u16, 2, 3])), oracle);

    // No count: `N` and the arity are structural, carried by STATE_SHAPE.
    let mut array_oracle = Vec::new();
    for value in [1u16, 2, 3] {
        array_oracle.extend_from_slice(&value.to_le_bytes());
    }
    assert_eq!(capture(&[1u16, 2, 3]), array_oracle);
    assert_eq!(capture(&(1u16, 2u16, 3u16)), array_oracle);
}

#[test]
fn option_and_result_carry_a_one_byte_tag() {
    assert_eq!(capture(&None::<u16>), vec![0]);
    assert_eq!(capture(&Some(0x0201u16)), vec![1, 0x01, 0x02]);
    assert_eq!(capture(&Ok::<u16, u8>(0x0201)), vec![0, 0x01, 0x02]);
    assert_eq!(capture(&Err::<u16, u8>(9)), vec![1, 9]);
}

#[test]
fn wrappers_are_transparent() {
    let inner = capture(&0x0201u16);
    assert_eq!(capture(&Box::new(0x0201u16)), inner);
    assert_eq!(capture(&Arc::new(0x0201u16)), inner);
    assert_eq!(capture(&std::rc::Rc::new(0x0201u16)), inner);
    assert_eq!(capture(&Wrapping(0x0201u16)), inner);
    assert_eq!(capture(&Mutex::new(0x0201u16)), inner);
    assert_eq!(capture(&RwLock::new(0x0201u16)), inner);
    assert_eq!(
        capture(&NonZeroU32::new(0x0201).expect("nonzero")),
        capture(&0x0201u32)
    );
}

#[test]
fn std_misc_types_round_trip_against_hand_oracles() {
    let duration = Duration::new(5, 250);
    let mut oracle = 5u64.to_le_bytes().to_vec();
    oracle.extend_from_slice(&250u32.to_le_bytes());
    assert_eq!(capture(&duration), oracle);
    assert_eq!(read_exact::<Duration>(&oracle), duration);

    let after = UNIX_EPOCH + Duration::new(1_700_000_000, 42);
    let mut after_oracle = vec![0u8];
    after_oracle.extend_from_slice(&1_700_000_000u64.to_le_bytes());
    after_oracle.extend_from_slice(&42u32.to_le_bytes());
    assert_eq!(capture(&after), after_oracle);
    assert_eq!(read_exact::<SystemTime>(&after_oracle), after);

    let before = UNIX_EPOCH - Duration::new(3, 0);
    let mut before_oracle = vec![1u8];
    before_oracle.extend_from_slice(&3u64.to_le_bytes());
    before_oracle.extend_from_slice(&0u32.to_le_bytes());
    assert_eq!(capture(&before), before_oracle);
    assert_eq!(read_exact::<SystemTime>(&before_oracle), before);

    let addr: std::net::SocketAddr = "127.0.0.1:7683".parse().expect("addr");
    let mut addr_oracle = vec![4u8, 127, 0, 0, 1];
    addr_oracle.extend_from_slice(&7683u16.to_le_bytes());
    assert_eq!(capture(&addr), addr_oracle);
    assert_eq!(read_exact::<std::net::SocketAddr>(&addr_oracle), addr);

    // NONZERO flowinfo + scope_id: a zero fixture round trips even for an
    // encoder that drops both fields, so it would prove nothing.
    let v6 = std::net::SocketAddr::V6(std::net::SocketAddrV6::new(
        std::net::Ipv6Addr::LOCALHOST,
        7683,
        0x0102_0304,
        0x0506_0708,
    ));
    let decoded = read_exact::<std::net::SocketAddr>(&capture(&v6));
    assert_eq!(decoded, v6);
    match decoded {
        std::net::SocketAddr::V6(addr) => {
            assert_eq!(addr.flowinfo(), 0x0102_0304);
            assert_eq!(addr.scope_id(), 0x0506_0708);
        }
        other => panic!("expected a V6 address, got {other:?}"),
    }
}

#[test]
fn a_nested_composite_round_trips_through_the_cursor() {
    let value: NestedComposite = vec![
        (
            "alpha".to_owned(),
            Some([(1u8, [7u16, 8]), (2, [9, 10])].into_iter().collect()),
        ),
        ("beta".to_owned(), None),
    ];
    let restored: NestedComposite = read_exact(&capture(&value));
    assert_eq!(restored, value);
}

#[test]
fn cer_restore_writes_in_place_and_leaves_the_lock_identity_alone() {
    let mut target = Mutex::new(1u32);
    // POISON the target first: the observable difference between restoring in
    // place and assigning a freshly built `Mutex` is the lock's own state, and
    // the value alone cannot tell them apart.
    let _ = std::panic::catch_unwind(|| {
        let _guard = target.lock().expect("acquire");
        panic!("poison the restore target");
    });
    assert!(
        target.is_poisoned(),
        "the fixture really poisoned the target"
    );

    let bytes = capture(&Mutex::new(42u32));
    let mut cursor = StateCursor::new(&bytes);
    target.cer_restore(&mut cursor).expect("restore");
    assert_eq!(
        *target.lock().unwrap_or_else(|e| e.into_inner()),
        42,
        "the recorded value landed"
    );
    assert!(
        target.is_poisoned(),
        "restore writes THROUGH the lock; a fresh `Mutex` would have lost its state"
    );

    let mut array = [0u16; 3];
    let bytes = capture(&[4u16, 5, 6]);
    let mut cursor = StateCursor::new(&bytes);
    array.cer_restore(&mut cursor).expect("restore");
    assert_eq!(array, [4, 5, 6]);
}

// ---------------------------------------------------------------------------
// the decoder refuses rather than guesses
// ---------------------------------------------------------------------------

#[test]
fn a_truncated_blob_is_reported_never_short_read() {
    let mut cursor = StateCursor::new(&[1, 2, 3]);
    assert_eq!(
        u32::cer_read(&mut cursor),
        Err(StateError::Truncated {
            needed: 4,
            remaining: 3
        })
    );
}

#[test]
fn trailing_bytes_are_a_drift_signal_not_something_to_ignore() {
    let mut cursor = StateCursor::new(&[1, 0, 0, 0, 99]);
    assert_eq!(u32::cer_read(&mut cursor), Ok(1));
    assert_eq!(
        cursor.finish(),
        Err(StateError::TrailingBytes { remaining: 1 })
    );
}

#[test]
fn every_out_of_domain_byte_is_a_named_error() {
    assert_eq!(
        bool::cer_read(&mut StateCursor::new(&[2])),
        Err(StateError::InvalidBool { value: 2 })
    );
    assert_eq!(
        char::cer_read(&mut StateCursor::new(&0x0011_0000u32.to_le_bytes())),
        Err(StateError::InvalidChar { value: 0x0011_0000 })
    );
    assert_eq!(
        Option::<u8>::cer_read(&mut StateCursor::new(&[2, 0])),
        Err(StateError::InvalidTag {
            type_name: "Option",
            tag: 2
        })
    );
    assert_eq!(
        NonZeroU32::cer_read(&mut StateCursor::new(&0u32.to_le_bytes())),
        Err(StateError::ZeroForNonZero {
            type_name: "NonZeroU32"
        })
    );
    let mut invalid_utf8 = count_bytes(1);
    invalid_utf8.push(0xff);
    assert_eq!(
        String::cer_read(&mut StateCursor::new(&invalid_utf8)),
        Err(StateError::InvalidUtf8 {
            type_name: "String"
        })
    );
}

#[test]
fn a_hostile_count_word_is_refused_because_the_blob_cannot_justify_it() {
    // Four bytes claiming four billion elements over an empty payload. The
    // decoder refuses on the COUNT, before the loop — which is what bounds its
    // work by its input rather than by the count word.
    let blob = u32::MAX.to_le_bytes();
    assert_eq!(
        Vec::<u32>::cer_read(&mut StateCursor::new(&blob)),
        Err(StateError::CountUnjustified {
            declared: u32::MAX as usize,
            budget: 4
        })
    );
    // Same guard on every counted container family.
    assert!(matches!(
        HashMap::<u32, u32>::cer_read(&mut StateCursor::new(&blob)),
        Err(StateError::CountUnjustified { .. })
    ));
    assert!(matches!(
        BTreeSet::<u32>::cer_read(&mut StateCursor::new(&blob)),
        Err(StateError::CountUnjustified { .. })
    ));
    assert!(matches!(
        read_unordered_entries::<u32, u32>(&mut StateCursor::new(&blob)),
        Err(StateError::CountUnjustified { .. })
    ));
}

#[test]
fn a_zero_encoding_element_type_cannot_turn_four_bytes_into_billions_of_reads() {
    // The zero-progress shape: `Vec<()>` reads no bytes per element, so
    // without the count guard a four-byte blob drives `u32::MAX` iterations
    // with the cursor pinned at 4 — measured at 2_000_000 reads before the
    // fix. The count is now refused outright.
    let blob = u32::MAX.to_le_bytes();
    assert_eq!(
        Vec::<()>::cer_read(&mut StateCursor::new(&blob)),
        Err(StateError::CountUnjustified {
            declared: u32::MAX as usize,
            budget: 4
        })
    );
    assert_eq!(
        Vec::<std::marker::PhantomData<u8>>::cer_read(&mut StateCursor::new(&blob)),
        Err(StateError::CountUnjustified {
            declared: u32::MAX as usize,
            budget: 4
        })
    );

    // ANTI-TAUTOLOGY, and the named casualty's exact boundary. A four-byte
    // blob affords four elements, so a zero-encoding container may declare up
    // to that many and no more — the refusal is of an UNAFFORDABLE count, not
    // of the shape.
    let one = 1u32.to_le_bytes();
    assert_eq!(read_exact::<Vec<()>>(&one), vec![()]);
    let four = 4u32.to_le_bytes();
    assert_eq!(read_exact::<Vec<()>>(&four).len(), 4);
    let five = 5u32.to_le_bytes();
    assert_eq!(
        Vec::<()>::cer_read(&mut StateCursor::new(&five)),
        Err(StateError::CountUnjustified {
            declared: 5,
            budget: 4
        })
    );
    // A one-entry set of the same shape is likewise still restorable.
    assert_eq!(read_exact::<HashSet<()>>(&one).len(), 1);
}

#[test]
fn a_string_payload_length_is_not_billed_as_container_iterations() {
    // THE SHAPE, and it is a SELF-ROUND-TRIP failure: a `(String, Vec<()>)`
    // capture of 108 bytes had its 100 payload bytes billed against the element
    // budget, leaving 8 for a count of 9 — so the module could not restore its
    // own output. A payload length needs no budget: the `take` that follows is
    // bounded by the blob.
    let value: (String, Vec<()>) = ("x".repeat(100), vec![(); 9]);
    let bytes = capture(&value);
    assert_eq!(bytes.len(), 108, "the fixture is the measured one");
    assert_eq!(read_exact::<(String, Vec<()>)>(&bytes), value);

    // The budget is spent by ITERATIONS only, so a blob that is almost all
    // string payload still affords its containers.
    let heavy: (String, String, Vec<u8>) = ("a".repeat(500), "b".repeat(500), vec![7u8; 3]);
    assert_eq!(
        read_exact::<(String, String, Vec<u8>)>(&capture(&heavy)),
        heavy
    );

    // ... and the length is still BOUNDED, by the read that consumes it: a
    // string claiming more bytes than the blob holds is refused, not budgeted.
    let mut hostile = 9_999u32.to_le_bytes().to_vec();
    hostile.extend_from_slice(b"short");
    assert_eq!(
        String::cer_read(&mut StateCursor::new(&hostile)),
        Err(StateError::Truncated {
            needed: 9_999,
            remaining: 5
        })
    );
}

#[test]
fn a_repeated_map_key_or_set_element_is_refused_not_silently_collapsed() {
    // `insert` keeps the LAST writer, so a two-entry blob restored ONE entry
    // and re-captured to 12 bytes against the 20 it was decoded from — the
    // restored state differs from the recording with nothing reporting it.
    // The canonical encoder emits from a live map, so it can never produce a
    // duplicate: a repeat is corruption.
    let mut duplicate = count_bytes(2);
    for _ in 0..2 {
        duplicate.extend_from_slice(&7u32.to_le_bytes());
        duplicate.extend_from_slice(&1u32.to_le_bytes());
    }
    assert_eq!(
        HashMap::<u32, u32>::cer_read(&mut StateCursor::new(&duplicate)),
        Err(StateError::DuplicateEntry {
            type_name: "HashMap"
        })
    );
    assert_eq!(
        BTreeMap::<u32, u32>::cer_read(&mut StateCursor::new(&duplicate)),
        Err(StateError::DuplicateEntry {
            type_name: "BTreeMap"
        })
    );
    assert_eq!(
        IndexMap::<u32, u32>::cer_read(&mut StateCursor::new(&duplicate)),
        Err(StateError::DuplicateEntry {
            type_name: "IndexMap"
        })
    );

    let mut duplicate_set = count_bytes(2);
    duplicate_set.extend_from_slice(&7u32.to_le_bytes());
    duplicate_set.extend_from_slice(&7u32.to_le_bytes());
    for expected in ["HashSet", "BTreeSet", "IndexSet"] {
        let verdict = match expected {
            "HashSet" => HashSet::<u32>::cer_read(&mut StateCursor::new(&duplicate_set)).err(),
            "BTreeSet" => BTreeSet::<u32>::cer_read(&mut StateCursor::new(&duplicate_set)).err(),
            _ => IndexSet::<u32>::cer_read(&mut StateCursor::new(&duplicate_set)).err(),
        };
        assert_eq!(
            verdict,
            Some(StateError::DuplicateEntry {
                type_name: expected
            })
        );
    }

    // ANTI-TAUTOLOGY: ADJACENT DISTINCT keys are the ordinary canonical shape
    // and must decode — without this the refusal could be "two entries are
    // always rejected".
    let mut distinct = count_bytes(2);
    for key in [7u32, 8] {
        distinct.extend_from_slice(&key.to_le_bytes());
        distinct.extend_from_slice(&1u32.to_le_bytes());
    }
    let decoded: HashMap<u32, u32> = read_exact(&distinct);
    assert_eq!(decoded.len(), 2);
    // And a real round trip of every container family still works.
    let map: BTreeMap<u32, u32> = [(1, 10), (2, 20)].into_iter().collect();
    assert_eq!(read_exact::<BTreeMap<u32, u32>>(&capture(&map)), map);
    let set: HashSet<u32> = [3u32, 4].into_iter().collect();
    assert_eq!(read_exact::<HashSet<u32>>(&capture(&set)), set);
}

#[test]
fn a_zero_floor_cannot_lose_the_scratch_bound() {
    // The pre-check charges the element type's own floor, and a hand-written
    // impl may leave that at ZERO — which is the SAFE direction for the floor
    // itself (under-charging costs work, over-charging refuses state that
    // fits) but must not surrender the bound the guard exists for.
    assert_eq!(floor_of::<HandWritten>(), 0);

    // A container with more entries than the sink has bytes left can never
    // justify an index, whatever it claims about its element size. 300 entries
    // against a 64-byte sink: refused before any scratch is built.
    let big: HashSet<HandWritten> = (0..300).map(HandWritten).collect();
    let mut arena = [0u8; 128];
    let mut sink = BoundedSink::with_capacity(&mut arena, 64);
    assert_eq!(big.cer_capture(&mut sink), Err(StateError::SinkFull));
    assert!(sink.refused());
    assert_eq!(sink.len(), 0);

    // ANTI-TAUTOLOGY: the SAME zero-floor type in a container the sink CAN
    // hold is captured, so the disjunct is a bound and not a blanket refusal.
    let small: HashSet<HandWritten> = (0..4).map(HandWritten).collect();
    let mut arena = [0u8; 128];
    let mut sink = BoundedSink::with_capacity(&mut arena, 64);
    assert_eq!(small.cer_capture(&mut sink), Ok(()));
    assert_eq!(sink.len(), 4 + 4 * 4);
}

#[test]
fn nested_zero_byte_containers_cannot_make_decode_work_quadratic() {
    // THE SHAPE: `[N][c1][c2]..[cN]` as `Vec<Vec<Z>>`. Each inner count word
    // is four bytes, and under a PER-COUNT rule each one is justified against
    // the counts that FOLLOW it — so the same tail bytes justify count after
    // count and the claimed work is quadratic in blob size. MEASURED on this
    // exact blob under a per-count rule: the counts claim 7_998_000 elements over an
    // 8_004-byte blob, 999x the blob.
    const N: usize = 2000;
    let mut blob = (N as u32).to_le_bytes().to_vec();
    for i in 0..N {
        let justified_under_a_per_count_rule = (4 * (N - 1 - i) + 1) as u32;
        blob.extend_from_slice(&justified_under_a_per_count_rule.to_le_bytes());
    }

    // The ORACLE is a COUNT, not a wall: a wall assertion tight enough to
    // separate linear from quadratic is also tight enough for a loaded runner
    // to invert (a known flake class). `CountedUnit` bumps a counter per decode,
    // so the work itself is observable.
    ELEMENT_READS.store(0, Ordering::SeqCst);
    let verdict = Vec::<Vec<CountedUnit>>::cer_read(&mut StateCursor::new(&blob));
    let reads = ELEMENT_READS.load(Ordering::SeqCst);

    // THE WORK ORACLE FIRST: it names the actual defect, and it is the arm a
    // wall-based test could not provide.
    assert!(
        reads <= blob.len() as u64,
        "decode work must stay LINEAR in blob length: {reads} element reads over a \
         {}-byte blob (a per-count rule reaches ~7_998_000 here)",
        blob.len()
    );

    // Classified, never formatted: a decode that WRONGLY succeeds here holds
    // millions of elements, and `{verdict:?}` would render every one of them
    // into the panic message (measured at 166 KB in that case).
    let outcome = match &verdict {
        Ok(outer) => format!("Ok({} inner vectors)", outer.len()),
        Err(e) => format!("Err({e})"),
    };
    assert!(
        matches!(verdict, Err(StateError::CountUnjustified { .. })),
        "the outer count's draw must leave the inner ones nothing, got {outcome}"
    );
}

#[test]
fn a_legitimate_nested_container_still_round_trips() {
    // ANTI-TAUTOLOGY for the budget: nesting is not what is refused, an
    // unaffordable count is. Every level here is paid for by real bytes.
    let nested: Vec<Vec<u8>> = vec![vec![1, 2, 3], vec![], vec![4, 5], vec![6; 40]];
    assert_eq!(read_exact::<Vec<Vec<u8>>>(&capture(&nested)), nested);

    let deep: Vec<Vec<Vec<u16>>> = vec![vec![vec![1, 2], vec![3]], vec![], vec![vec![4]]];
    assert_eq!(read_exact::<Vec<Vec<Vec<u16>>>>(&capture(&deep)), deep);

    // And a nested container of a ZERO-encoding element is fine too, as long
    // as the blob affords it: 3 inner vectors of one `()` each.
    let zero_nested: Vec<Vec<()>> = vec![vec![()], vec![()], vec![()]];
    assert_eq!(
        read_exact::<Vec<Vec<()>>>(&capture(&zero_nested)),
        zero_nested
    );
}

#[test]
fn the_count_guard_is_loose_enough_that_real_state_never_trips_it() {
    // The guard binds only where a count exceeds the bytes that follow, so
    // every ordinary container has slack: `Vec<u32>` gets four bytes of budget
    // per element, `HashMap<u32, u32>` eight. The TIGHTEST real case is a
    // trailing `Vec<u8>`, which sits exactly AT the boundary — this is the
    // arm that would fail if the guard were tightened by one.
    let bytes: Vec<u8> = (0u8..200).collect();
    assert_eq!(read_exact::<Vec<u8>>(&capture(&bytes)), bytes);

    // And a container followed by SIBLING fields is paid for out of the same
    // decode-wide budget, which the blob's own length seeds.
    let pair: (Vec<u8>, [u8; 64]) = (vec![7u8; 100], [9u8; 64]);
    assert_eq!(read_exact::<(Vec<u8>, [u8; 64])>(&capture(&pair)), pair);

    // Several containers in one blob DRAW FROM ONE budget, which is the whole
    // point — and a legitimate blob still affords all of them.
    let many: (Vec<u8>, Vec<u8>, Vec<u8>) = (vec![1; 30], vec![2; 30], vec![3; 30]);
    assert_eq!(
        read_exact::<(Vec<u8>, Vec<u8>, Vec<u8>)>(&capture(&many)),
        many
    );
}

#[test]
fn a_noncanonical_duration_is_refused_rather_than_clamped() {
    // Both sides of the boundary. 999_999_999 ns is the largest canonical
    // subsecond field and MUST decode; 1_000_000_000 is the first
    // noncanonical one and MUST be refused.
    let mut at_boundary = 5u64.to_le_bytes().to_vec();
    at_boundary.extend_from_slice(&999_999_999u32.to_le_bytes());
    assert_eq!(
        read_exact::<Duration>(&at_boundary),
        Duration::new(5, 999_999_999)
    );

    let mut over = 5u64.to_le_bytes().to_vec();
    over.extend_from_slice(&1_000_000_000u32.to_le_bytes());
    assert_eq!(
        Duration::cer_read(&mut StateCursor::new(&over)),
        Err(StateError::NoncanonicalDuration {
            nanos: 1_000_000_000
        })
    );

    // THE REASON it is refused rather than clamped: clamping makes these two
    // DISTINCT byte strings decode to ONE value, so a checkpoint restored from
    // the second would re-capture to the first's bytes — the canonical-form
    // property the whole recording rests on.
    let mut far_over = 5u64.to_le_bytes().to_vec();
    far_over.extend_from_slice(&1_500_000_000u32.to_le_bytes());
    assert_ne!(far_over, at_boundary);
    assert_eq!(
        Duration::cer_read(&mut StateCursor::new(&far_over)),
        Err(StateError::NoncanonicalDuration {
            nanos: 1_500_000_000
        })
    );

    // `SystemTime` decodes through `Duration`, so it inherits the refusal.
    let mut nested = vec![0u8];
    nested.extend_from_slice(&far_over);
    assert_eq!(
        SystemTime::cer_read(&mut StateCursor::new(&nested)),
        Err(StateError::NoncanonicalDuration {
            nanos: 1_500_000_000
        })
    );
}

#[test]
fn a_system_time_encoding_the_epoch_with_the_before_sign_is_refused() {
    // The same canonical-form rule one level up, and the encoder's own choice
    // is what makes `-0` noncanonical: `duration_since` returns `Ok` for an
    // equal instant, so the epoch is emitted as `+0`.
    let plus_zero = capture(&UNIX_EPOCH);
    assert_eq!(plus_zero[0], 0, "the epoch is emitted with the after sign");
    assert_eq!(read_exact::<SystemTime>(&plus_zero), UNIX_EPOCH);

    let mut minus_zero = vec![1u8];
    minus_zero.extend_from_slice(&0u64.to_le_bytes());
    minus_zero.extend_from_slice(&0u32.to_le_bytes());
    assert_ne!(minus_zero, plus_zero);
    assert_eq!(
        SystemTime::cer_read(&mut StateCursor::new(&minus_zero)),
        Err(StateError::NoncanonicalEpochSign)
    );

    // ANTI-TAUTOLOGY: a NONZERO before-the-epoch time is perfectly canonical
    // and still round trips, so the arm above is not "the before sign is
    // rejected".
    let before = UNIX_EPOCH - Duration::new(3, 5);
    assert_eq!(capture(&before)[0], 1);
    assert_eq!(read_exact::<SystemTime>(&capture(&before)), before);
}

#[test]
fn a_capture_only_field_verifies_on_restore_and_never_silently_keeps_the_running_value() {
    let recorded = capture(&"idle");

    let mut matching: &'static str = "idle";
    assert_eq!(
        matching.cer_restore(&mut StateCursor::new(&recorded)),
        Ok(()),
        "an unchanged verify-only field restores cleanly"
    );

    let mut differing: &'static str = "driving";
    assert_eq!(
        differing.cer_restore(&mut StateCursor::new(&recorded)),
        Err(StateError::ImmutableMismatch {
            type_name: "&'static str"
        })
    );
    assert_eq!(differing, "driving", "the running value is left untouched");

    assert_eq!(
        <&'static str as CerulionState>::cer_read(&mut StateCursor::new(&recorded)),
        Err(StateError::Unrestorable {
            type_name: "&'static str"
        })
    );
}

// ---------------------------------------------------------------------------
// INLINE_SAFE — one arm per inventory row
// ---------------------------------------------------------------------------

#[test]
fn inline_safe_matches_the_published_inventory_row_by_row() {
    // scalars
    assert!(inline_safe::<bool>());
    assert!(inline_safe::<i8>());
    assert!(inline_safe::<i128>());
    assert!(inline_safe::<u8>());
    assert!(inline_safe::<u128>());
    assert!(inline_safe::<usize>());
    assert!(inline_safe::<isize>());
    assert!(inline_safe::<f32>());
    assert!(inline_safe::<f64>());
    assert!(inline_safe::<char>());
    assert!(inline_safe::<NonZeroU32>());
    assert!(inline_safe::<Wrapping<u32>>());
    assert!(inline_safe::<()>());
    assert!(inline_safe::<std::marker::PhantomData<u32>>());

    // text / bytes
    assert!(inline_safe::<String>());
    assert!(inline_safe::<&'static str>());
    assert!(inline_safe::<PathBuf>());
    assert!(inline_safe::<Cow<'static, str>>());
    assert!(inline_safe::<Cow<'static, [u8]>>());

    // wrappers — fold the inner type
    assert!(inline_safe::<Option<u32>>());
    assert!(inline_safe::<Result<u32, u8>>());
    assert!(inline_safe::<Box<u32>>());
    assert!(inline_safe::<std::rc::Rc<u32>>());
    assert!(inline_safe::<Arc<u32>>());

    // sequences
    assert!(inline_safe::<Vec<u32>>());
    assert!(inline_safe::<VecDeque<u32>>());
    assert!(inline_safe::<[u32; 4]>());
    assert!(inline_safe::<(u32, String)>());
    assert!(
        inline_safe::<(u8, u8, u8, u8, u8, u8, u8, u8, u8, u8, u8, u8)>(),
        "arity 12 is the inventory's declared limit"
    );

    // maps / sets
    assert!(inline_safe::<BTreeMap<u32, u32>>());
    assert!(inline_safe::<BTreeSet<u32>>());
    assert!(inline_safe::<HashMap<u32, u32>>());
    assert!(inline_safe::<HashSet<u32>>());
    assert!(inline_safe::<IndexMap<u32, u32>>());
    assert!(inline_safe::<IndexSet<u32>>());

    // std misc
    assert!(inline_safe::<Duration>());
    assert!(inline_safe::<SystemTime>());
    assert!(inline_safe::<std::net::IpAddr>());
    assert!(inline_safe::<std::net::SocketAddr>());

    // locks — the whole point of the const
    assert!(!inline_safe::<Mutex<u32>>());
    assert!(!inline_safe::<RwLock<u32>>());
}

#[test]
fn every_composite_folds_inline_safe_with_and_never_or() {
    // Single-parameter composites propagate.
    assert!(!inline_safe::<Vec<Mutex<u32>>>());
    assert!(!inline_safe::<VecDeque<Mutex<u32>>>());
    assert!(!inline_safe::<[Mutex<u32>; 2]>());
    assert!(!inline_safe::<Option<Mutex<u32>>>());
    assert!(!inline_safe::<Box<Mutex<u32>>>());
    assert!(!inline_safe::<Arc<Mutex<u32>>>());
    assert!(!inline_safe::<std::rc::Rc<RwLock<u32>>>());
    assert!(!inline_safe::<Wrapping<Mutex<u32>>>());
    // Sets keyed by a lock are uninhabitable (`Mutex` is neither `Hash` nor
    // `Ord`), so the set rows are exercised through the hand-written fixture,
    // whose `INLINE_SAFE` is also `false`.
    assert!(!inline_safe::<HashSet<HandWritten>>());
    assert!(!inline_safe::<BTreeSet<HandWritten>>());
    assert!(!inline_safe::<IndexSet<HandWritten>>());
    assert!(!inline_safe::<Cow<'static, [HandWritten]>>());

    // MULTI-parameter composites are where AND and OR differ. Each has ONE
    // unsafe slot, so an OR fold would report `true`.
    assert!(!inline_safe::<(u32, Mutex<u32>)>());
    assert!(!inline_safe::<(Mutex<u32>, u32)>());
    assert!(!inline_safe::<(u32, u32, Mutex<u32>)>());
    assert!(!inline_safe::<Result<u32, Mutex<u32>>>());
    assert!(!inline_safe::<Result<Mutex<u32>, u32>>());
    assert!(!inline_safe::<HashMap<u32, Mutex<u32>>>());
    assert!(!inline_safe::<BTreeMap<u32, Mutex<u32>>>());
    assert!(!inline_safe::<IndexMap<u32, Mutex<u32>>>());

    // Deeply nested still folds.
    assert!(!inline_safe::<Vec<HashMap<u32, Arc<Mutex<Vec<u8>>>>>>());
    assert!(inline_safe::<Vec<HashMap<u32, Arc<Vec<u8>>>>>());
}

// ---------------------------------------------------------------------------
// atomics
// ---------------------------------------------------------------------------

#[test]
fn an_atomic_encodes_exactly_like_the_primitive_it_wraps() {
    // Hand-built oracles: the whole point of the by-value capture is that the
    // bytes are the primitive's, so a recording is readable without knowing
    // the field happened to be atomic.
    assert_eq!(capture(&AtomicU64::new(0x0102_0304_0506_0708)), {
        let mut want = Vec::new();
        want.extend_from_slice(&[0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01]);
        want
    });
    assert_eq!(capture(&AtomicU32::new(7)), vec![7, 0, 0, 0]);
    assert_eq!(capture(&AtomicI8::new(-1)), vec![0xff]);
    assert_eq!(capture(&AtomicBool::new(true)), vec![1]);
    assert_eq!(capture(&AtomicBool::new(false)), vec![0]);

    // ... and byte-for-byte the same as the bare primitive, which is the
    // property that sentence claims.
    assert_eq!(capture(&AtomicU64::new(9)), capture(&9u64));
    assert_eq!(capture(&AtomicIsize::new(-3)), capture(&-3isize));
    assert_eq!(capture(&AtomicBool::new(true)), capture(&true));
}

#[test]
fn an_atomic_round_trips_through_both_read_and_in_place_restore() {
    let decoded: AtomicU64 = read_exact(&capture(&AtomicU64::new(42)));
    assert_eq!(decoded.load(Ordering::Relaxed), 42);

    // `cer_restore` is the path the derive uses field by field.
    let mut target = AtomicU64::new(0);
    let bytes = capture(&AtomicU64::new(1234));
    let mut cursor = StateCursor::new(&bytes);
    target.cer_restore(&mut cursor).expect("restore");
    cursor.finish().expect("blob fully consumed");
    assert_eq!(target.load(Ordering::Relaxed), 1234);

    let mut flag = AtomicBool::new(false);
    let bytes = capture(&AtomicBool::new(true));
    let mut cursor = StateCursor::new(&bytes);
    flag.cer_restore(&mut cursor).expect("restore");
    assert!(flag.load(Ordering::Relaxed));
}

#[test]
fn a_pointer_width_atomic_rides_the_portable_64_bit_form() {
    // Hand-built oracles, spelled to the byte rather than to `size_of`: an
    // `AtomicUsize` is EIGHT bytes little-endian on every target, because
    // `STATE_SHAPE` folds only the type NAME and so cannot tell a 32-bit
    // recording from a 64-bit one.
    assert_eq!(
        capture(&AtomicUsize::new(1)),
        vec![1, 0, 0, 0, 0, 0, 0, 0],
        "eight bytes, not the target's pointer width"
    );
    assert_eq!(
        capture(&AtomicIsize::new(-3)),
        vec![0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]
    );

    // ... and the same bytes the PLAIN pointer-width integer writes, which is
    // what lets one recording be read by both.
    assert_eq!(capture(&AtomicUsize::new(9)), capture(&9usize));
    assert_eq!(capture(&AtomicIsize::new(-3)), capture(&-3isize));

    let decoded: AtomicUsize = read_exact(&capture(&AtomicUsize::new(42)));
    assert_eq!(decoded.load(Ordering::Relaxed), 42);

    let mut target = AtomicIsize::new(0);
    let bytes = capture(&AtomicIsize::new(-1234));
    let mut cursor = StateCursor::new(&bytes);
    target.cer_restore(&mut cursor).expect("restore");
    cursor.finish().expect("blob fully consumed");
    assert_eq!(target.load(Ordering::Relaxed), -1234);
}

/// The cross-width decode, driven at a width THIS host can actually observe.
///
/// A 64-bit host cannot tell a portable encoding from a pointer-width one —
/// the two are byte-identical here — so the decode is driven through the same
/// `read_pointer_width` the atomics call, instantiated at `u32`/`i32` to stand
/// in for a 32-bit target's `usize`/`isize`. That makes the arm that matters
/// reachable: a recording written on a 64-bit robot carrying a value a 32-bit
/// desk cannot hold must be REPORTED, never silently truncated into a plausible
/// small number.
#[test]
fn a_pointer_width_decode_reports_a_value_the_target_cannot_hold() {
    use super::impls::read_pointer_width;

    // In range: a 64-bit writer's eight bytes are readable by the narrow side.
    let bytes = 7u64.to_le_bytes();
    let mut cursor = StateCursor::new(&bytes);
    assert_eq!(
        read_pointer_width::<u32, u64>(&mut cursor, "usize").expect("7 fits a 32-bit usize"),
        7u32
    );
    cursor.finish().expect("exactly eight bytes consumed");

    // Out of range: `OutOfRange` carrying the RECORDED value, so the operator
    // sees what was written rather than what a truncating cast would keep.
    let too_big = u64::from(u32::MAX) + 1;
    let bytes = too_big.to_le_bytes();
    let mut cursor = StateCursor::new(&bytes);
    assert_eq!(
        read_pointer_width::<u32, u64>(&mut cursor, "usize"),
        Err(StateError::OutOfRange {
            type_name: "usize",
            value: i128::from(too_big),
        }),
        "a truncating cast would have kept 0 and looked healthy"
    );

    // The signed half, negative-overflow direction.
    let too_small = i64::from(i32::MIN) - 1;
    let bytes = too_small.to_le_bytes();
    let mut cursor = StateCursor::new(&bytes);
    assert_eq!(
        read_pointer_width::<i32, i64>(&mut cursor, "isize"),
        Err(StateError::OutOfRange {
            type_name: "isize",
            value: i128::from(too_small),
        })
    );

    // Seven bytes is a truncated blob, never a short read that succeeds.
    let short = [0u8; 7];
    let mut cursor = StateCursor::new(&short);
    assert!(matches!(
        read_pointer_width::<u32, u64>(&mut cursor, "usize"),
        Err(StateError::Truncated { .. })
    ));
}

#[test]
fn an_atomic_bool_refuses_a_non_canonical_byte_exactly_like_bool() {
    // Two byte strings for one value would break the canonical-form property
    // the recording rests on, so `2` is refused rather than coerced to `true`.
    let mut cursor = StateCursor::new(&[2]);
    assert!(matches!(
        AtomicBool::cer_read(&mut cursor),
        Err(StateError::InvalidBool { value: 2 })
    ));
    // The bare `bool` answers identically — they share one decoder.
    let mut cursor = StateCursor::new(&[2]);
    assert!(matches!(
        bool::cer_read(&mut cursor),
        Err(StateError::InvalidBool { value: 2 })
    ));
}

#[test]
fn an_atomic_is_inline_safe_because_reading_one_takes_no_lock() {
    // A load cannot block the node thread, so the inline
    // carrier's byte bound is still a time bound.
    assert!(inline_safe::<AtomicU64>());
    assert!(inline_safe::<AtomicBool>());
    // ... and it folds through the wrappers a counter actually ships in.
    assert!(inline_safe::<Arc<AtomicU64>>());
    assert!(inline_safe::<Vec<AtomicU32>>());
    assert!(inline_safe::<Option<Arc<AtomicUsize>>>());
    // The ANTI-TAUTOLOGY half: a lock in the same shape still folds to false,
    // so `true` above is a real answer rather than a blanket one.
    assert!(!inline_safe::<Arc<Mutex<AtomicU64>>>());

    // The probe agrees — there is no lock to be held.
    assert!(AtomicU64::new(0).cer_probe());
}

#[test]
fn an_atomics_shape_is_distinct_from_the_primitive_it_wraps() {
    // The bytes are identical (above), so ONLY the shape distinguishes a
    // retype between them — and a retype IS a change to the declared state
    // graph, exactly as `Mutex<T>` differs from `T`.
    assert_ne!(
        <AtomicU64 as CerulionState>::STATE_SHAPE,
        <u64 as CerulionState>::STATE_SHAPE
    );
    assert_ne!(
        <AtomicBool as CerulionState>::STATE_SHAPE,
        <bool as CerulionState>::STATE_SHAPE
    );
    // Different widths are different shapes.
    assert_ne!(
        <AtomicU32 as CerulionState>::STATE_SHAPE,
        <AtomicU64 as CerulionState>::STATE_SHAPE
    );
    // Signedness too — the encodings are the same width, so nothing else
    // would catch a swap.
    assert_ne!(
        <AtomicI64 as CerulionState>::STATE_SHAPE,
        <AtomicU64 as CerulionState>::STATE_SHAPE
    );
    // Hand-folded oracle: the shape is the type NAME, nothing more.
    assert_eq!(
        <AtomicU64 as CerulionState>::STATE_SHAPE,
        StateShape::of("AtomicU64").finish()
    );
}

/// How long a concurrency test will wait for the helper thread to make
/// observable progress before giving up.
///
/// Deliberately enormous relative to the work: the writer is spinning, so the
/// condition is reached in microseconds on any machine that schedules it at
/// all. It is a WEDGE detector, not a timing assertion — nothing here is
/// bounded in units small enough for a loaded runner to invert (a known
/// flake class), and expiry is reported as a failure of the apparatus rather than
/// passed over.
const OVERLAP_DEADLINE: Duration = Duration::from_secs(30);

/// Read an `AtomicU64` back THROUGH `cer_capture`.
fn captured_u64(counter: &AtomicU64) -> u64 {
    u64::from_le_bytes(capture(counter).try_into().expect("8 bytes"))
}

#[test]
fn a_concurrently_incremented_counter_captures_a_value_it_really_held() {
    // The atomics' soundness claim, exercised rather than asserted: a helper
    // thread increments while the boundary captures, and every reading must
    // be a value the counter genuinely passed through — never a torn mixture.
    //
    // OVERLAP IS STRUCTURAL, not hoped for. The previous shape sent its ready
    // signal as the FIRST statement of the writer closure and then ran a fixed
    // 20_000 increments, so the writer was free to finish before the reader's
    // first capture — after which every reading is the final value, the
    // monotonic and in-range assertions hold vacuously, and the test asserts
    // nothing about concurrency at all. Here the writer spins until the reader
    // STOPS it, so it is provably live across every capture, and the reader
    // additionally has to WITNESS the counter moving before it is allowed to
    // conclude anything.
    let counter = Arc::new(AtomicU64::new(0));
    let stop = Arc::new(AtomicBool::new(false));

    let writer = {
        let counter = Arc::clone(&counter);
        let stop = Arc::clone(&stop);
        thread::spawn(move || {
            let mut written = 0u64;
            while !stop.load(Ordering::Relaxed) {
                counter.fetch_add(1, Ordering::Relaxed);
                written += 1;
            }
            written
        })
    };

    // Capture until the counter is SEEN to advance — the progress check. A
    // writer that had already finished (the old bug) can never satisfy it.
    let started = std::time::Instant::now();
    let mut readings = vec![captured_u64(&counter)];
    while *readings.last().expect("seeded") == readings[0] {
        assert!(
            started.elapsed() < OVERLAP_DEADLINE,
            "the writer made no observable progress in {OVERLAP_DEADLINE:?}: the \
             capture never overlapped it, so this test would prove nothing"
        );
        readings.push(captured_u64(&counter));
    }
    // Keep reading well past the first advance, so the window the oracle below
    // covers is a real stretch of concurrent writing rather than one edge.
    for _ in 0..500 {
        readings.push(captured_u64(&counter));
    }

    stop.store(true, Ordering::Relaxed);
    let written = writer.join().expect("writer");

    // The overlap the loop above waited for, restated as an assertion so the
    // property is checked rather than merely arranged.
    assert!(
        readings.iter().any(|v| *v > readings[0]),
        "no capture observed the counter advancing, so nothing raced: {readings:?}"
    );
    assert!(
        readings[0] < written,
        "the first capture already saw the writer's final value ({}, of {written}), \
         so the captures did not overlap the writing",
        readings[0]
    );

    // Every reading is in range and the sequence never goes backwards — a torn
    // read of a `fetch_add` sequence would land outside both.
    for pair in readings.windows(2) {
        assert!(
            pair[0] <= pair[1],
            "a counter reading went backwards: {} then {}",
            pair[0],
            pair[1]
        );
    }
    for value in &readings {
        assert!(
            *value <= written,
            "reading {value} exceeds the {written} increments ever made"
        );
    }
    // Once the writer is joined the counter is settled, so the capture is exact.
    assert_eq!(captured_u64(&counter), written);
}

#[test]
fn a_capture_racing_a_writer_that_flips_every_byte_never_returns_a_torn_value() {
    // The sibling above has the right shape (a shared atomic counter) but
    // is a weak tearing oracle: consecutive `fetch_add(1)` values differ in
    // one byte, so most torn mixtures of two of them are themselves plausible
    // in-range readings and slip past a monotonic-and-in-range check.
    //
    // This one is the EXACT oracle. The writer alternates between two values
    // that differ in EVERY bit, so any mixture of a half-written store — the
    // failure `AtomicU64::load` exists to make impossible, and what a
    // hand-written impl reading the 8 bytes directly would produce — is
    // neither sentinel and is caught by an equality check with no tolerance.
    const LOW: u64 = 0x0000_0000_0000_0000;
    const HIGH: u64 = 0xFFFF_FFFF_FFFF_FFFF;

    let cell = Arc::new(AtomicU64::new(LOW));
    let stop = Arc::new(AtomicBool::new(false));

    let writer = {
        let cell = Arc::clone(&cell);
        let stop = Arc::clone(&stop);
        thread::spawn(move || {
            while !stop.load(Ordering::Relaxed) {
                cell.store(HIGH, Ordering::Relaxed);
                cell.store(LOW, Ordering::Relaxed);
            }
        })
    };

    // Capture until BOTH sentinels have been seen. That is the overlap proof
    // and it is not satisfiable by luck or by a finished writer: only a writer
    // running CONCURRENTLY with the reader can show it two different values.
    let started = std::time::Instant::now();
    let (mut saw_low, mut saw_high) = (false, false);
    let mut captures = 0u64;
    while !(saw_low && saw_high) {
        assert!(
            started.elapsed() < OVERLAP_DEADLINE,
            "after {captures} captures in {OVERLAP_DEADLINE:?} only one sentinel was \
             ever seen (low: {saw_low}, high: {saw_high}) — the capture never \
             overlapped the writer, so this test would prove nothing"
        );
        let seen = captured_u64(&cell);
        captures += 1;
        // The tearing oracle: no tolerance, no range, no monotonicity. The
        // cell only ever HOLDS one of two values, so a capture returning
        // anything else read a half-written store.
        assert!(
            seen == LOW || seen == HIGH,
            "capture #{captures} returned {seen:#018x}, which the cell never held — \
             a torn read across the two sentinels"
        );
        saw_low |= seen == LOW;
        saw_high |= seen == HIGH;
    }

    stop.store(true, Ordering::Relaxed);
    writer.join().expect("writer");
}

#[test]
fn min_encoded_bytes_is_a_true_floor_on_every_inventory_row() {
    // The guard that consumes it must never OVER-charge, so each value is
    // checked against what the type really encodes to at its smallest.
    fn floor<T: CerulionState>() -> usize {
        T::MIN_ENCODED_BYTES
    }
    fn assert_is_a_floor<T: CerulionState>(smallest: &T) {
        let actual = capture(smallest).len();
        assert!(
            T::MIN_ENCODED_BYTES <= actual,
            "{}: declared floor {} exceeds an actual encoding of {actual}",
            std::any::type_name::<T>(),
            T::MIN_ENCODED_BYTES
        );
    }

    assert_eq!(floor::<u8>(), 1);
    assert_eq!(floor::<u128>(), 16);
    assert_eq!(floor::<usize>(), 8);
    assert_eq!(floor::<bool>(), 1);
    assert_eq!(floor::<char>(), 4);
    assert_eq!(floor::<()>(), 0);
    assert_eq!(floor::<std::marker::PhantomData<u8>>(), 0);
    assert_eq!(floor::<String>(), 4);
    assert_eq!(floor::<Vec<u8>>(), 4);
    assert_eq!(floor::<Option<u64>>(), 1);
    assert_eq!(floor::<Result<u64, u8>>(), 2, "tag + the cheaper arm");
    assert_eq!(floor::<[u16; 4]>(), 8);
    assert_eq!(floor::<(u8, u32)>(), 5);
    assert_eq!(floor::<Duration>(), 12);
    assert_eq!(floor::<SystemTime>(), 13);
    assert_eq!(floor::<HashMap<u32, u32>>(), 4);
    assert_eq!(floor::<Mutex<u64>>(), 8);
    assert_eq!(floor::<AtomicU64>(), 8);
    assert_eq!(floor::<AtomicU8>(), 1);
    assert_eq!(floor::<AtomicUsize>(), 8, "always the portable 64-bit form");
    assert_eq!(floor::<AtomicBool>(), 1);
    assert_eq!(
        floor::<HandWritten>(),
        0,
        "a hand impl claims NOTHING, so the guard under-charges rather than \
         refusing state that fits"
    );

    // Each declared floor is <= a real smallest encoding.
    assert_is_a_floor(&0u8);
    assert_is_a_floor(&0u128);
    assert_is_a_floor(&0usize);
    assert_is_a_floor(&false);
    assert_is_a_floor(&'a');
    assert_is_a_floor(&());
    assert_is_a_floor(&String::new());
    assert_is_a_floor(&Vec::<u8>::new());
    assert_is_a_floor(&None::<u64>);
    assert_is_a_floor(&Err::<u64, u8>(0));
    assert_is_a_floor(&[0u16; 4]);
    assert_is_a_floor(&(0u8, 0u32));
    assert_is_a_floor(&Duration::ZERO);
    assert_is_a_floor(&UNIX_EPOCH);
    assert_is_a_floor(&HashMap::<u32, u32>::new());
    assert_is_a_floor(&Mutex::new(0u64));
    assert_is_a_floor(&AtomicU64::new(0));
    assert_is_a_floor(&AtomicU8::new(0));
    assert_is_a_floor(&AtomicUsize::new(0));
    assert_is_a_floor(&AtomicBool::new(false));
}

#[test]
fn a_hand_written_impl_cannot_silently_claim_the_inline_fast_path() {
    // Risk 6b: the trait's default is `false`, so an impl that says nothing
    // gets the safe answer and takes the fork carrier.
    assert!(!inline_safe::<HandWritten>());
    // ... and the claim does not leak back through a composite.
    assert!(!inline_safe::<Vec<HandWritten>>());
    assert!(!inline_safe::<(u32, HandWritten)>());
    assert!(!inline_safe::<Option<HandWritten>>());
    // The default probe is still permissive, which is sound: a hand impl can
    // only fail to REPORT a lock, and that residual is the fork carrier's
    // progress watchdog, not this const.
    assert!(HandWritten(1).cer_probe());
}

// ---------------------------------------------------------------------------
// cer_probe
// ---------------------------------------------------------------------------

#[test]
fn cer_probe_reports_non_quiescent_over_a_held_lock_and_recovers_when_it_is_released() {
    let lock = Arc::new(Mutex::new(7u32));
    let (held_tx, held_rx) = mpsc::channel::<()>();
    let (release_tx, release_rx) = mpsc::channel::<()>();

    let holder = {
        let lock = Arc::clone(&lock);
        thread::spawn(move || {
            let guard = lock.lock().expect("acquire");
            // Rendezvous, never a sleep: the test proceeds only once the lock
            // is provably held, and the holder releases only once told to.
            held_tx.send(()).expect("signal held");
            release_rx.recv().expect("await release");
            drop(guard);
        })
    };

    held_rx.recv().expect("await held");
    assert!(
        !lock.cer_probe(),
        "a held lock in the declared state graph is NOT quiescent"
    );
    // The encoder agrees, and it never blocks: `try_lock`, never `lock`.
    let mut sink = VecSink::new();
    assert_eq!(
        lock.cer_capture(&mut sink),
        Err(StateError::LockContended { type_name: "Mutex" })
    );

    release_tx.send(()).expect("release");
    holder.join().expect("holder joins cleanly");

    assert!(lock.cer_probe(), "a free lock is quiescent");
    assert_eq!(capture(&*lock), capture(&7u32));
}

#[test]
fn cer_probe_walks_a_composite_all_the_way_to_a_held_lock() {
    let state: Arc<Vec<Mutex<u32>>> = Arc::new(vec![Mutex::new(1), Mutex::new(2)]);
    assert!(state.cer_probe());

    let (held_tx, held_rx) = mpsc::channel::<()>();
    let (release_tx, release_rx) = mpsc::channel::<()>();
    let holder = {
        let state = Arc::clone(&state);
        thread::spawn(move || {
            // The SECOND element, so the walk must reach past the first.
            let guard = state[1].lock().expect("acquire");
            held_tx.send(()).expect("signal held");
            release_rx.recv().expect("await release");
            drop(guard);
        })
    };

    held_rx.recv().expect("await held");
    assert!(!state.cer_probe());
    release_tx.send(()).expect("release");
    holder.join().expect("holder joins cleanly");
    assert!(state.cer_probe());
}

#[test]
fn a_read_lock_blocks_the_probe_only_for_a_writer_not_for_a_reader() {
    let lock = Arc::new(RwLock::new(5u32));
    let reader = lock.read().expect("read");
    assert!(
        lock.cer_probe(),
        "a concurrent READER does not stop a capture: capture only reads"
    );
    drop(reader);

    let (held_tx, held_rx) = mpsc::channel::<()>();
    let (release_tx, release_rx) = mpsc::channel::<()>();
    let holder = {
        let lock = Arc::clone(&lock);
        thread::spawn(move || {
            let guard = lock.write().expect("write");
            held_tx.send(()).expect("signal held");
            release_rx.recv().expect("await release");
            drop(guard);
        })
    };
    held_rx.recv().expect("await held");
    assert!(!lock.cer_probe(), "a held WRITER is not quiescent");
    let mut sink = VecSink::new();
    assert_eq!(
        lock.cer_capture(&mut sink),
        Err(StateError::LockContended {
            type_name: "RwLock"
        })
    );
    release_tx.send(()).expect("release");
    holder.join().expect("holder joins cleanly");
    assert!(lock.cer_probe());
}

#[test]
fn a_poisoned_but_free_lock_is_captured_not_reported_as_contended() {
    let lock = Mutex::new(11u32);
    let _ = std::panic::catch_unwind(|| {
        let _guard = lock.lock().expect("acquire");
        panic!("poison the mutex");
    });
    assert!(lock.is_poisoned(), "the fixture really poisoned the lock");

    // Poisoned is not contended: reporting it as contended would permanently
    // disable anchors for the node AND name the wrong cause.
    assert!(lock.cer_probe());
    assert_eq!(capture(&lock), capture(&11u32));
}

// ---------------------------------------------------------------------------
// STATE_SHAPE
// ---------------------------------------------------------------------------

#[test]
fn a_state_shape_matches_a_hand_folded_fnv_stream() {
    // Hand-assemble the byte stream the recipe declares, then hash it in one
    // shot — the builder must agree chunk for chunk.
    fn oracle_of(type_name: &str) -> Vec<u8> {
        let mut bytes = (type_name.len() as u64).to_le_bytes().to_vec();
        bytes.extend_from_slice(type_name.as_bytes());
        bytes
    }

    assert_eq!(
        <u32 as CerulionState>::STATE_SHAPE,
        fnv1a_hash(&oracle_of("u32"))
    );

    let mut vec_stream = oracle_of("Vec");
    vec_stream.extend_from_slice(&<u32 as CerulionState>::STATE_SHAPE.to_le_bytes());
    assert_eq!(
        <Vec<u32> as CerulionState>::STATE_SHAPE,
        fnv1a_hash(&vec_stream)
    );

    let mut named_stream = oracle_of("HandWritten");
    named_stream.extend_from_slice(&1u64.to_le_bytes());
    named_stream.extend_from_slice(b"0");
    named_stream.extend_from_slice(&<u32 as CerulionState>::STATE_SHAPE.to_le_bytes());
    assert_eq!(
        <HandWritten as CerulionState>::STATE_SHAPE,
        fnv1a_hash(&named_stream)
    );

    let mut array_stream = oracle_of("[T; N]");
    array_stream.extend_from_slice(&4u64.to_le_bytes());
    array_stream.extend_from_slice(&<u8 as CerulionState>::STATE_SHAPE.to_le_bytes());
    assert_eq!(
        <[u8; 4] as CerulionState>::STATE_SHAPE,
        fnv1a_hash(&array_stream)
    );
}

#[test]
fn a_state_shape_separates_every_edit_that_changes_the_bytes() {
    // Array length is structural.
    assert_ne!(
        <[u8; 4] as CerulionState>::STATE_SHAPE,
        <[u8; 8] as CerulionState>::STATE_SHAPE
    );
    // Element type.
    assert_ne!(
        <Vec<u8> as CerulionState>::STATE_SHAPE,
        <Vec<i8> as CerulionState>::STATE_SHAPE
    );
    // Container identity — a `HashMap` and a `BTreeMap` differ in ordering
    // semantics even though their bytes have the same shape.
    assert_ne!(
        <HashMap<u32, u32> as CerulionState>::STATE_SHAPE,
        <BTreeMap<u32, u32> as CerulionState>::STATE_SHAPE
    );
    assert_ne!(
        <Vec<u32> as CerulionState>::STATE_SHAPE,
        <VecDeque<u32> as CerulionState>::STATE_SHAPE
    );
    // Key/value order in a map.
    assert_ne!(
        <HashMap<u8, u32> as CerulionState>::STATE_SHAPE,
        <HashMap<u32, u8> as CerulionState>::STATE_SHAPE
    );
    // Tuple arity, even when the encodings coincide.
    assert_ne!(
        <(u8, u8) as CerulionState>::STATE_SHAPE,
        <(u8, u8, u8) as CerulionState>::STATE_SHAPE
    );
    // Field NAMES — the transposition trap: two same-typed fields
    // swapped must not share a shape.
    let goal = StateShape::of("Goal")
        .field("x", <f64 as CerulionState>::STATE_SHAPE)
        .field("y", <f64 as CerulionState>::STATE_SHAPE)
        .finish();
    let transposed = StateShape::of("Goal")
        .field("y", <f64 as CerulionState>::STATE_SHAPE)
        .field("x", <f64 as CerulionState>::STATE_SHAPE)
        .finish();
    assert_ne!(goal, transposed);
    // A nested type's DEFINITION reaches the parent (recipe 3's property).
    let parent_of_two = StateShape::of("Parent")
        .field("inner", StateShape::of("Inner").count(2).finish())
        .finish();
    let parent_of_three = StateShape::of("Parent")
        .field("inner", StateShape::of("Inner").count(3).finish())
        .finish();
    assert_ne!(parent_of_two, parent_of_three);
}

#[test]
fn a_state_shape_is_a_compile_time_constant() {
    // Usable in const position — the property that makes it zero-cost and
    // that lets the derive fold it through the real type graph.
    const SHAPE: u64 = <Vec<Option<u32>> as CerulionState>::STATE_SHAPE;
    const _: [(); 1] = [(); (SHAPE != 0) as usize];
    assert_eq!(SHAPE, <Vec<Option<u32>> as CerulionState>::STATE_SHAPE);
}
