//! — the generated corpus really derives `CerulionState`.
//!
//! The derive was added to `emit_snapshot_derives` in `cerulion_core`'s
//! codegen, and `cerulion_core` cannot test the result: `native_ros2_messages`
//! depends on IT, so the dependency only runs this way round. Without this
//! file the corpus half of the chunk could compile and still be inert — the
//! derive silently dropped from the emitted list, nothing failing until a node
//! that holds a ROS message as plain state stopped building.
//!
//! Every assertion is against a **hand-written byte oracle**, so it also pins
//! that the encoding a recording carries for a ROS type is the one this
//! module's docs describe (little-endian, fields in declaration order, `u32`
//! count prefixes) rather than whatever the derive happens to emit.
//!
//! Pure: no transport, no iceoryx2 — parallel-safe.

use cerulion_core::state::{CerulionState, StateCursor, StateShape, VecSink};
use native_ros2_messages::geometry_msgs::{TwistSnapshot, Vector3Snapshot};
use native_ros2_messages::std_msgs::StringSnapshot;

fn capture(value: &impl CerulionState) -> Vec<u8> {
    let mut sink = VecSink::new();
    value.cer_capture(&mut sink).expect("unbounded capture");
    sink.into_inner()
}

fn read_exact<T: CerulionState>(bytes: &[u8]) -> T {
    let mut cursor = StateCursor::new(bytes);
    let value = T::cer_read(&mut cursor).expect("decode");
    cursor.finish().expect("blob fully consumed");
    value
}

/// Read `INLINE_SAFE` through a function so the assertion is not a constant
/// one (clippy's `assertions_on_constants`, and it would be folded away).
fn inline_safe<T: CerulionState>() -> bool {
    T::INLINE_SAFE
}

#[test]
fn a_fixed_ros_snapshot_captures_its_fields_in_declaration_order() {
    let v = Vector3Snapshot {
        x: 1.0,
        y: 2.0,
        z: -3.0,
    };
    let mut want = Vec::new();
    want.extend_from_slice(&1.0f64.to_le_bytes());
    want.extend_from_slice(&2.0f64.to_le_bytes());
    want.extend_from_slice(&(-3.0f64).to_le_bytes());

    assert_eq!(capture(&v), want);
    assert_eq!(read_exact::<Vector3Snapshot>(&want), v);
}

#[test]
fn a_nested_ros_snapshot_folds_through_its_members() {
    // `Twist` is two nested `Vector3`s — the shape a bridge node holds as
    // plain state, and the case where a per-type hand impl would drift.
    let t = TwistSnapshot {
        linear: Vector3Snapshot {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        angular: Vector3Snapshot {
            x: 0.0,
            y: 0.0,
            z: 0.5,
        },
    };
    let mut want = Vec::new();
    for f in [1.0f64, 0.0, 0.0, 0.0, 0.0, 0.5] {
        want.extend_from_slice(&f.to_le_bytes());
    }
    assert_eq!(capture(&t), want);
    assert_eq!(read_exact::<TwistSnapshot>(&want), t);

    // A change INSIDE the nested type must bump the parent's shape (recipe 3,
    // which the derive inherits by folding the member's own STATE_SHAPE).
    //
    // Comparing `Twist`'s shape with `Vector3`'s does NOT test that, which is
    // what this assertion used to do: two differently-named types with
    // different field lists come out different under any scheme at all,
    // including one that folded nothing about its members. The invariant needs
    // the parent held FIXED while only the nested member moves.
    //
    // First, the recipe itself, against a hand-built oracle: the parent really
    // does fold the member's own shape rather than, say, the member's name.
    let folded = StateShape::of("TwistSnapshot")
        .field("linear", <Vector3Snapshot as CerulionState>::STATE_SHAPE)
        .field("angular", <Vector3Snapshot as CerulionState>::STATE_SHAPE)
        .finish();
    assert_eq!(<TwistSnapshot as CerulionState>::STATE_SHAPE, folded);

    // Then the twin that alters ONLY the nested member. Parent name, field
    // names, field order and arity are byte-identical to the oracle above, so
    // the perturbed member shape is the only variable — and the parent moves
    // with it. That is what makes a `Vector3` gaining a field a change a
    // recording of a `Twist` can see.
    const AS_IF_VECTOR3_CHANGED: u64 = <Vector3Snapshot as CerulionState>::STATE_SHAPE ^ 0x1;
    let perturbed = StateShape::of("TwistSnapshot")
        .field("linear", AS_IF_VECTOR3_CHANGED)
        .field("angular", AS_IF_VECTOR3_CHANGED)
        .finish();
    assert_ne!(
        <TwistSnapshot as CerulionState>::STATE_SHAPE,
        perturbed,
        "a change inside `Vector3` must reach `Twist`'s shape"
    );

    // The two assertions above are kept apart because each catches a
    // different failure the other misses:
    //
    //  - folding a constant, or the member's TYPE NAME, instead of the
    //    member's shape kills the equality — and the perturbation survives it,
    //    since it only ever compares against a hand-built different value;
    //  - a `StateShape::field` that IGNORES its shape argument kills the
    //    perturbation — and the equality survives it, because both sides are
    //    then built the same wrong way and still agree.
    //
    // Neither reaches the class the sibling test below is for, and BOTH of
    // them pass against a constant tuned to COINCIDE with the one nested
    // type these assertions check — the sibling is the only thing that
    // fails against such a constant.
}

#[test]
fn a_parents_shape_tracks_its_member_rather_than_a_baked_constant() {
    // Both assertions in the sibling above turn on ONE nested type. That is
    // enough to catch a fold of the WRONG value, and blind to a fold of a
    // RIGHT-TODAY value: an emission that bakes `Vector3Snapshot`'s current
    // shape as a literal agrees with a hand oracle that folds the same
    // constant, and only diverges once the member changes — by which time the
    // recording it silently accepted is already wrong.
    //
    // MEASURED, by hand-patching the derive to fold the literal
    // `0x7160eefea6f1ab0e` (`Vector3Snapshot::STATE_SHAPE` as of this commit)
    // for every nested member. The split IS this test's justification:
    //
    // | arm                                            | baked-constant defect |
    // |------------------------------------------------|-----------------------|
    // | sibling: parent equals its hand-folded oracle  | PASSES                |
    // | sibling: perturbed member shape differs        | PASSES                |
    // | this test                                      | **FAILS**             |
    //
    // Both assertion families below are killed INDEPENDENTLY under it, each
    // measured with the other neutralised: the per-parent oracles fire first
    // (a parent folding the literal cannot equal an oracle folding its own
    // member), and with those removed the cross-parent `assert_ne!` fires on
    // its own, both parents having collapsed to the identical baked number.
    //
    // A proc macro cannot evaluate an associated const, so the derive cannot
    // express that mutation today. This is a tripwire rather than a live
    // guard, and the reason it is worth carrying is that this crate's types
    // are GENERATED by a `build.rs`, which CAN compute a member's shape and
    // emit it as a literal — precisely the "precomputed constant" optimization
    // someone reaches for to avoid a deep const-fold chain.
    //
    // Two nested types are what it takes: with only one, a baked constant and
    // a real fold are the same number.
    mod narrow {
        use cerulion_core::state::CerulionState;
        #[derive(CerulionState)]
        pub struct Member {
            pub x: f64,
            pub y: f64,
            pub z: f64,
        }
        #[derive(CerulionState)]
        pub struct Parent {
            pub linear: Member,
            pub angular: Member,
        }
    }
    mod widened {
        use cerulion_core::state::CerulionState;
        #[derive(CerulionState)]
        pub struct Member {
            pub x: f64,
            pub y: f64,
            pub z: f64,
            pub w: f64,
        }
        #[derive(CerulionState)]
        pub struct Parent {
            pub linear: Member,
            pub angular: Member,
        }
    }

    // Each parent's shape TRACKS ITS OWN member. A baked literal is the same
    // number in both, so at most one of these two can hold under that defect.
    assert_eq!(
        <narrow::Parent as CerulionState>::STATE_SHAPE,
        StateShape::of("Parent")
            .field("linear", <narrow::Member as CerulionState>::STATE_SHAPE)
            .field("angular", <narrow::Member as CerulionState>::STATE_SHAPE)
            .finish(),
    );
    assert_eq!(
        <widened::Parent as CerulionState>::STATE_SHAPE,
        StateShape::of("Parent")
            .field("linear", <widened::Member as CerulionState>::STATE_SHAPE)
            .field("angular", <widened::Member as CerulionState>::STATE_SHAPE)
            .finish(),
    );

    // The headline, and the reason the two parents are declared identically:
    // same struct name, same field names, same order, same arity. The ONLY
    // difference in the whole comparison is one field inside `Member`, so
    // nothing else can account for the shapes differing — and nothing but a
    // real fold can make them differ at all.
    assert_ne!(
        <narrow::Parent as CerulionState>::STATE_SHAPE,
        <widened::Parent as CerulionState>::STATE_SHAPE,
        "the parents declare identical names, fields and order — only the \
         nested member gained a field, and that must move the parent's shape"
    );

    // Anti-tautology: the two members really are different types with
    // different shapes, so the assertion above is not satisfied by two
    // identical declarations that happen to be spelled twice.
    assert_ne!(
        <narrow::Member as CerulionState>::STATE_SHAPE,
        <widened::Member as CerulionState>::STATE_SHAPE,
    );
}

#[test]
fn a_variable_ros_snapshot_rides_a_length_prefix() {
    let s = StringSnapshot {
        data: "hello".to_string(),
    };
    let mut want = Vec::new();
    want.extend_from_slice(&5u32.to_le_bytes());
    want.extend_from_slice(b"hello");

    assert_eq!(capture(&s), want);
    assert_eq!(read_exact::<StringSnapshot>(&want), s);

    // The empty edge: a `u32` zero and nothing else.
    assert_eq!(
        capture(&StringSnapshot::default()),
        0u32.to_le_bytes().to_vec()
    );
}

#[test]
fn the_corpus_is_inline_eligible_so_a_node_holding_one_never_forks_for_it() {
    // The generated corpus is `INLINE_SAFE = true`.
    // A ROS message is plain data — no lock, no interior mutability — so a
    // node holding one stays on the fast carrier.
    assert!(inline_safe::<Vector3Snapshot>());
    assert!(inline_safe::<TwistSnapshot>());
    assert!(inline_safe::<StringSnapshot>());
}

#[test]
fn a_ros_snapshot_restores_in_place() {
    let recorded = capture(&Vector3Snapshot {
        x: 7.0,
        y: 8.0,
        z: 9.0,
    });
    let mut live = Vector3Snapshot::default();
    let mut cursor = StateCursor::new(&recorded);
    live.cer_restore(&mut cursor).expect("restore");
    cursor.finish().expect("blob fully consumed");
    assert_eq!(live.x, 7.0);
    assert_eq!(live.y, 8.0);
    assert_eq!(live.z, 9.0);
}
