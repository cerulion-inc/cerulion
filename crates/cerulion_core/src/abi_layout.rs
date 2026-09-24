// SPDX-License-Identifier: AGPL-3.0-only
//! The ABI LAYOUT PIN — a checked-in snapshot of every
//! `cerulion_core` struct that crosses the cdylib `init()` FFI, keyed to
//! [`crate::CERULION_ABI_VERSION`].
//!
//! # Why this exists
//!
//! `NodeContext` crosses the cdylib `init()` boundary as a raw `Box`
//! ([`crate::graph::node::DylibNodeEntry`]), and the cdylib links its OWN copy
//! of `cerulion_core`. Host and cdylib therefore compile the same struct
//! definitions TWICE, from possibly DIFFERENT source versions, and every field
//! either side reads is read at ITS OWN idea of the offset. That is the whole
//! reason [`crate::CERULION_ABI_VERSION`] exists: a struct in this closure
//! changes ⇒ the version bumps ⇒ a stale cdylib is REFUSED loudly at load
//! instead of reading garbage.
//!
//! Without this module, nothing enforces the first half of that sentence. A
//! field added to [`crate::read_outcome::StagedReadOutcome`] (`role`, say)
//! could ship with the ABI version unchanged and the whole test suite green; no
//! other test catches it. The two failure modes no other test sees
//! are exactly the two this module catches:
//!
//! - **M1** — a field is added to (or removed from, or retyped in) an
//!   FFI-crossing struct and the ABI version is left alone.
//! - **M2** — the ABI version is bumped and this snapshot is not re-taken, so
//!   the next M1 lands against a table that no longer describes anything.
//!
//! # The two halves
//!
//! Each covered struct is pinned twice, and the halves catch different things:
//!
//! 1. **Compile-time field-set pin.** `abi_pin_struct!` expands to an
//!    exhaustive destructuring pattern with NO `..` rest pattern, inside the
//!    module that defines the struct (private fields are visible only there).
//!    Adding or removing a field is then a COMPILE ERROR naming the struct and
//!    the field. This half is fully portable and catches the field-set change
//!    on every platform, including a change that leaves the size untouched
//!    because the new field fits in existing padding. That last case is not
//!    hypothetical and the runtime half's NUMBERS alone genuinely miss it —
//!    MEASURED on a faithful transliteration of `StagedReadOutcome` (same field
//!    types, same order, same reprs, same rustc 1.96): adding a trailing `u8`
//!    leaves size 32, align 8 and all five existing offsets (0/16/24/28/30)
//!    IDENTICAL, with the new field landing at 31 in the tail padding. What
//!    catches it is this half, and then the FIELD COUNT below — which is
//!    coupled to it, because the macro measures the same field list the
//!    destructuring pattern names, so restoring compilation means declaring the
//!    new field to the table too.
//! 2. **Runtime layout snapshot.** The same macro measures `size_of`,
//!    `align_of` and `offset_of!` per field and hands them here, where they
//!    are compared against [`EXPECTED`]. This half is what carries the
//!    *message*: it names the struct, the field, the expected and the measured
//!    value, and tells the reader to bump [`crate::CERULION_ABI_VERSION`] and
//!    re-snapshot.
//!
//! [`EXPECTED_ABI`] is asserted equal to [`crate::CERULION_ABI_VERSION`]
//! FIRST, which is the M2 half: a bump with no re-snapshot fails immediately.
//! The table is deliberately a SINGLE current-version snapshot rather than a
//! growing history — a history would let a bump slip past without anyone
//! re-measuring, which is the failure being prevented.
//!
//! # Re-snapshotting after a deliberate change
//!
//! ```text
//! cargo test -p cerulion_core --lib abi_layout::dump_measured_table -- --ignored --nocapture
//! ```
//!
//! prints the whole table in source form; paste it over [`EXPECTED`] and set
//! [`EXPECTED_ABI`] to the new [`crate::CERULION_ABI_VERSION`]. Run the pin
//! with `cargo test -p cerulion_core --lib abi_layout`.
//!
//! # Scope
//!
//! ## What is covered
//!
//! The closure of the `init()` `Box` — `NodeContext` and everything it owns,
//! by value or through a `cerulion_core`-owned `Arc`/`Vec` — restricted to
//! types DEFINED IN this crate:
//!
//! | Owner | Covered |
//! |---|---|
//! | the `Box` itself | `NodeContext`, `ShutdownSignal` |
//! | its two `IndexMap` values | `AnyPublisher`, `AnySubscriber` |
//! | those enums' payloads | `CerulionPublisher`, `CerulionSubscriber` |
//! | by value inside the subscriber | `SampleGate`, `DropOldestProbe`, `BlockProbe`, `BackpressureProbe`, `StreamObservation`, `FrozenSlot`, `ConsumeMode` |
//! | by value inside the publisher | `MaxSliceLen`, `AdaptiveSizer`, `NotifyDeliveryLatch`, `OutputDiscardLatch` |
//! | the v12/v16 read-outcome chain | `ReadOutcomeStage`, `StageInner`, `StagedReadOutcome`, `ReadOutcomeKind`, `ReadSiteRole`, `ReadStageRole` |
//! | the QoS store the node drains | `QosEventStore` |
//! | `NodeContext.transport` (`graph/node.rs`) | `TransportManager` |
//! | `CerulionPublisher.trace` | `PublishTrace`, `PublishTraceEntry`, `HistoryDepth`, `BagWriter`, `CurrentFile`, `BagRetention` |
//!
//! ## What is NOT covered, and why
//!
//! - **Third-party types held by value** (`iceoryx2`'s `Publisher`,
//!   `Subscriber`, `Listener`, `Notifier`, `Sample`, `PortFactory`,
//!   `UniquePublisherId`; `std`'s `Mutex`; `IndexMap`, `HashMap`). Their
//!   layouts are held by a DIFFERENT contract: one lockfile
//!   (`iceoryx2_version_lockstep_test`) plus one toolchain, since host and
//!   cdylib are built together on one machine. Pinning their sizes here would
//!   pin the platform, not the ABI.
//! - **`crate::doorbell::Doorbell`** (held by value as `Option<Doorbell>` in
//!   `CerulionPublisher`) — it has TWO `#[cfg]`-selected definitions with
//!   different field sets, so a portable field-set pin would need per-platform
//!   tables. Its parent's field set is pinned; its own is not.
//! - **The closure BELOW `TransportManager`.** The manager itself IS pinned
//!   (field set): `NodeContext.transport: Option<Arc<TransportManager>>` is
//!   read by CDYLIB-compiled code (that is the entire point: the manager was
//!   put in the context precisely so a cdylib would stop
//!   resolving its own `INSTANCE` static), so a cdylib method indexes those
//!   fields at ITS OWN offsets and a layout change without a bump is already
//!   UB on a stale cdylib. The pin makes an EXISTING obligation visible; the
//!   bump-per-transport-field cost is the cost of the design. What is NOT
//!   pinned is what those fields POINT AT (`NetworkManager` and its zenoh
//!   session, `TopicBridgeManager`, the registration/ingress channels, the
//!   mirror registry, iceoryx2's `Node`): every one sits behind an `Arc`,
//!   `Mutex` or a third-party handle, and the correct tier for the manager row
//!   is `FieldSetOnly` anyway, so absolute offsets could not be claimed for it
//!   whatever its members did.
//! - **The closure below `PublishTrace`'s `BagWriter`** — nothing, as it
//!   happens: `BagWriter`, `CurrentFile` and `BagRetention` are all pinned
//!   here too, so that branch is complete. `BufWriter<File>` itself is `std`
//!   and is held by the toolchain contract like every other `std` container.
//! - **`Arc<dyn Clock>`'s vtable.** A trait object's vtable is per linkage
//!   unit; nothing about it is a struct layout, and it is held by the
//!   same-source contract.
//!
//! ## Which entries carry absolute numbers
//!
//! [`Layout::Pinned`] rows carry `size`/`align`/`offset`s. A row is only
//! eligible when every one of its field types is a primitive, an atomic, a
//! pointer-shaped `std` container (`Box`/`Arc`/`Vec`/`String`/`Arc<str>`/
//! `Option` of those), a `HashMap`/`IndexMap` (whose own size does not depend
//! on `K`/`V`), or another `Pinned` type — i.e. a layout that is identical on
//! every 64-bit target we build. A `const` assertion below refuses a 32-bit
//! target outright rather than reporting a wall of confusing offset diffs.
//!
//! Everything embedding an `iceoryx2` port, a `std::sync::Mutex` or a
//! `Doorbell` is [`Layout::FieldSetOnly`]: its field NAMES and COUNT are
//! pinned (portable, and enough to fail M1 with the bump instruction), its
//! absolute offsets are not. The reason is first-party and checkable, not a
//! guess: `Doorbell` is `#[cfg]`-split in this very crate, `iceoryx2-pal-posix`
//! carries per-OS type modules (`src/macos/types.rs` vs the Linux ones), and
//! `std::sync::Mutex`'s inner lock is documented as platform-specific. CI runs
//! this suite on BOTH linux-x86_64 and macos-aarch64, so a table that pinned
//! those numbers from one desk would go red on the other for a reason that has
//! nothing to do with the ABI.
//!
//! ## Toolchain dependence
//!
//! `repr(Rust)` field order is not guaranteed stable across rustc releases.
//! Local rustc is 1.96; CI's test jobs track auto-updated `stable` (the MSRV
//! 1.93 job only `cargo check`s). No drift has been observed, and a future
//! rustc that reorders fields would fail this pin with an offset diff on an
//! unchanged tree — which is a *correct* signal to re-snapshot, not a false
//! alarm.
//!
//! The assumption this table used to rest on, "the same rustc compiles both
//! host and cdylib", is no longer merely assumed: [`crate::RUSTC_FINGERPRINT`]
//! (ABI v22) is compared across the cdylib FFI boundary at LOAD time
//! (`DylibNodeEntry::load`) and a host/cdylib pair built by different rustc
//! releases is refused before either side runs. That guard exists precisely
//! because this table's size and offset pin CANNOT see every divergence: two
//! rustc releases can agree on every struct size and offset here and still
//! encode a niche-holding `Option::None` with a different bit pattern (rustc
//! 1.97.0 changed it), a class of ABI break this static table is structurally
//! blind to, since it only measures size, align and offset, never bit
//! patterns.

use crate::CERULION_ABI_VERSION;

/// A 64-bit target is assumed by every [`Layout::Pinned`] row (pointer-shaped
/// `std` containers are what make those rows portable in the first place).
const _: () = assert!(
    core::mem::size_of::<usize>() == 8,
    "abi_layout: the pinned layout table describes 64-bit targets. On a 32-bit target \
     every pointer-shaped field moves; re-snapshot per target rather than reading the diff"
);

/// One measured field of one measured struct.
pub(crate) struct MeasuredField {
    pub(crate) name: &'static str,
    pub(crate) offset: usize,
}

/// One struct as MEASURED by the macro expansion in its defining module.
pub(crate) struct MeasuredStruct {
    pub(crate) name: &'static str,
    pub(crate) size: usize,
    pub(crate) align: usize,
    /// Empty for an enum (a variant carries no stable field offsets); the
    /// enum's variant SET is pinned by the exhaustive `match` the macro
    /// expands instead.
    pub(crate) fields: Vec<MeasuredField>,
}

/// Pin one struct: the compile-time exhaustive field-set half AND the runtime
/// measurement half, in one place so the two cannot drift apart.
///
/// Invoke it in the module that DEFINES the struct — private fields (and
/// `offset_of!` on them) are visible only there.
macro_rules! abi_pin_struct {
    ($ty:ident { $($field:ident),+ $(,)? }) => {{
        // COMPILE-TIME FIELD-SET PIN. Never called; it exists so that adding
        // or removing a field of `$ty` fails to compile, naming the struct and
        // the field. Deliberately NO `..` rest pattern.
        let _field_set_pin = |v: &$ty| {
            let $ty { $($field),+ } = v;
            $( let _ = $field; )+
        };
        $crate::abi_layout::MeasuredStruct {
            name: stringify!($ty),
            size: ::core::mem::size_of::<$ty>(),
            align: ::core::mem::align_of::<$ty>(),
            fields: vec![$(
                $crate::abi_layout::MeasuredField {
                    name: stringify!($field),
                    offset: ::core::mem::offset_of!($ty, $field),
                }
            ),+],
        }
    }};
    // Tuple-struct form. The binding name is written explicitly because a
    // destructuring pattern needs names while `offset_of!` needs indices:
    // `abi_pin_struct!(MaxSliceLen(len @ 0))`.
    ($ty:ident ( $($bind:ident @ $idx:tt),+ $(,)? )) => {{
        let _field_set_pin = |v: &$ty| {
            let $ty($($bind),+) = v;
            $( let _ = $bind; )+
        };
        $crate::abi_layout::MeasuredStruct {
            name: stringify!($ty),
            size: ::core::mem::size_of::<$ty>(),
            align: ::core::mem::align_of::<$ty>(),
            fields: vec![$(
                $crate::abi_layout::MeasuredField {
                    name: stringify!($idx),
                    offset: ::core::mem::offset_of!($ty, $idx),
                }
            ),+],
        }
    }};
}

/// Pin one enum: the compile-time exhaustive VARIANT-set half plus size and
/// align. A variant added or removed fails to compile, naming the enum.
macro_rules! abi_pin_enum {
    ($ty:ident { $($variant:pat_param),+ $(,)? }) => {{
        let _variant_set_pin = |v: &$ty| match v { $( $variant => {} ),+ };
        $crate::abi_layout::MeasuredStruct {
            name: stringify!($ty),
            size: ::core::mem::size_of::<$ty>(),
            align: ::core::mem::align_of::<$ty>(),
            fields: Vec::new(),
        }
    }};
}

pub(crate) use {abi_pin_enum, abi_pin_struct};

/// How much of a struct's layout this table claims.
enum Layout {
    /// `size`, `align` and every field `offset` are pinned exactly.
    Pinned {
        size: usize,
        align: usize,
        /// Parallel to [`Expected::fields`].
        offsets: &'static [usize],
    },
    /// Only the field NAMES and COUNT are pinned — the struct embeds a type
    /// whose size differs per platform (see the module docs). `reason` is
    /// printed on a mismatch so the next reader does not have to re-derive it.
    FieldSetOnly { reason: &'static str },
}

/// One row of the snapshot.
struct Expected {
    name: &'static str,
    /// Field names in declaration order (empty for an enum).
    fields: &'static [&'static str],
    layout: Layout,
}

/// The ABI version this table was snapshotted at.
///
/// Asserted equal to [`crate::CERULION_ABI_VERSION`] before anything else is
/// compared: a bump that did not re-take the snapshot is exactly as much of a
/// defect as a layout change that did not bump.
const EXPECTED_ABI: u32 = 23;

const IOX2_PORTS: &str =
    "embeds iceoryx2 port types by value, whose layouts come from per-OS `iceoryx2-pal-posix` \
     modules";
const IOX2_PORTS_AND_DOORBELL: &str =
    "embeds iceoryx2 port types by value AND `Option<crate::doorbell::Doorbell>`, which has two \
     `#[cfg]`-selected definitions";
const IOX2_SAMPLE: &str = "embeds an iceoryx2 `Sample` / `UniquePublisherId` by value";
const STD_MUTEX: &str = "embeds `std::sync::Mutex`, whose inner lock is platform-specific";
const IOX2_NODE_AND_MUTEXES: &str =
    "embeds an iceoryx2 `Node` by value AND six `std::sync::Mutex` fields";
const STD_FILE: &str =
    "reaches `std::fs::File` through `BufWriter`, whose representation is `#[cfg]`-selected by \
     target family";

static EXPECTED: &[Expected] = &[
    Expected {
        name: "NodeContext",
        fields: &[
            "publishers",
            "subscribers",
            "clock",
            "shutdown_signal",
            "env_snapshot",
            "qos_events",
            "node_id",
            "recon_logged",
            "transport",
        ],
        layout: Layout::Pinned {
            size: 224,
            align: 8,
            offsets: &[0, 72, 168, 184, 192, 200, 144, 216, 208],
        },
    },
    Expected {
        name: "ShutdownSignal",
        fields: &["inner"],
        layout: Layout::Pinned {
            size: 8,
            align: 8,
            offsets: &[0],
        },
    },
    Expected {
        name: "AnyPublisher",
        fields: &[],
        layout: Layout::FieldSetOnly {
            reason: IOX2_PORTS_AND_DOORBELL,
        },
    },
    Expected {
        name: "AnySubscriber",
        fields: &[],
        layout: Layout::FieldSetOnly { reason: IOX2_PORTS },
    },
    Expected {
        name: "CerulionPublisher",
        fields: &[
            "topic",
            "publisher",
            "notifier",
            "listener",
            "last_listener_count",
            "self_drains_armed",
            "sequence",
            "initial_sequence",
            "clock",
            "max_slice_len",
            "history_size",
            "provisioned_history_size",
            "sizer",
            "trace",
            "fault_inject_publish_raw_after",
            "fault_inject_send_overflow_frame_after",
            "fault_inject_send_raw_loan",
            "frames_dropped_overflow",
            "frames_dropped_invariant_violation",
            "frames_dropped_send_fail",
            "block_outstanding",
            "promise_within_last_publish_ns",
            "doorbell",
            "event_service",
            "notify_elision_expected",
            "notify_elided_count",
            "notify_gate_last_elided",
            "notify_on_publish_raw",
            "unannounced_publish",
            "resweep_notify_count",
            "notify_delivery_latch",
            "notify_undelivered_shared",
            "discard_signal",
            "replay_suppress",
            "fault_inject_loan_fail_next",
            "discard_latch",
            "output_discard_shared",
        ],
        layout: Layout::FieldSetOnly {
            reason: IOX2_PORTS_AND_DOORBELL,
        },
    },
    Expected {
        name: "CerulionSubscriber",
        fields: &[
            "topic",
            "subscriber",
            "listener",
            "notifier",
            "probe",
            "pending_backpressure_event",
            "fault_inject_receive_after",
            // v18: the cached effective borrow budget — see the
            // field doc and the v18 paragraph on `CERULION_ABI_VERSION`.
            "max_borrowed_samples",
            "max_publishers",
            "drain_scratch",
            "expect_within_last_data_ns",
            "per_set_sync_trigger",
            "frozen",
            "next_head",
            "block_slot_debt",
            "fault_inject_sync_next_arrived_err",
            "held_sample",
            "unified_bound",
            "unified_receive_warned",
            "read_stage",
            "multi_publisher_edge",
            "consume_mode",
            "held_head_reoffers",
            "held_head_warned",
            "served_sequence",
        ],
        layout: Layout::FieldSetOnly { reason: IOX2_PORTS },
    },
    Expected {
        name: "SampleGate",
        fields: &[
            "interval_ns",
            "interval_ms",
            "last_accepted_ns",
            "counters",
            "node_id",
            "input",
            "armed",
            "regime_started_ns",
            "regime_count",
            "buffer_capacity",
        ],
        layout: Layout::Pinned {
            size: 104,
            align: 8,
            offsets: &[56, 64, 0, 16, 24, 40, 96, 72, 80, 88],
        },
    },
    Expected {
        name: "DropOldestProbe",
        fields: &[
            "baselines",
            "max_baselines",
            "drain_counter",
            "counters",
            "node_id",
            "input",
            "armed",
            "backward_run",
            "regime_started_ns",
            "regime_count",
            "buffer_capacity",
        ],
        layout: Layout::Pinned {
            size: 120,
            align: 8,
            offsets: &[0, 64, 72, 24, 32, 48, 112, 80, 88, 96, 104],
        },
    },
    Expected {
        name: "BlockProbe",
        fields: &[
            "outstanding",
            "threshold",
            "counters",
            "input",
            "armed",
            "regime_started_ns",
            "regime_count",
            "buffer_capacity",
        ],
        layout: Layout::Pinned {
            size: 80,
            align: 8,
            offsets: &[0, 40, 16, 24, 72, 48, 56, 64],
        },
    },
    Expected {
        name: "StreamObservation",
        fields: &["id", "first_seq", "newest_seq", "newest_ts"],
        layout: Layout::FieldSetOnly {
            reason: IOX2_SAMPLE,
        },
    },
    // This row did NOT move when `BlockProbe` grew 72 -> 80.
    // An enum is sized by its LARGEST variant, and that is `DropOldestProbe`
    // (120) — so the 8-byte growth of a smaller variant is absorbed by padding
    // that already existed. Recorded because the obvious reading of "BlockProbe
    // grew" is that its enclosing enum grew too, and a future reader
    // re-snapshotting on that assumption would be chasing a number that is
    // correct.
    Expected {
        name: "BackpressureProbe",
        fields: &[],
        layout: Layout::Pinned {
            size: 120,
            align: 8,
            offsets: &[],
        },
    },
    Expected {
        name: "FrozenSlot",
        fields: &[],
        layout: Layout::FieldSetOnly {
            reason: IOX2_SAMPLE,
        },
    },
    Expected {
        name: "ConsumeMode",
        fields: &[],
        layout: Layout::Pinned {
            size: 1,
            align: 1,
            offsets: &[],
        },
    },
    Expected {
        name: "AdaptiveSizer",
        fields: &["window", "head", "recorded_count"],
        layout: Layout::Pinned {
            size: 68,
            align: 4,
            offsets: &[0, 64, 65],
        },
    },
    Expected {
        name: "NotifyDeliveryLatch",
        fields: &[
            "degraded",
            "suppressed",
            "total_undelivered",
            "pending_shortfall",
            "last_classified",
        ],
        layout: Layout::Pinned {
            size: 32,
            align: 8,
            offsets: &[24, 0, 8, 25, 16],
        },
    },
    Expected {
        name: "OutputDiscardLatch",
        fields: &["failing", "suppressed", "total_discards"],
        layout: Layout::Pinned {
            size: 24,
            align: 8,
            offsets: &[16, 0, 8],
        },
    },
    Expected {
        name: "MaxSliceLen",
        fields: &["0"],
        layout: Layout::Pinned {
            size: 4,
            align: 4,
            offsets: &[0],
        },
    },
    Expected {
        name: "ReadOutcomeStage",
        fields: &["input_idx", "role", "capacity", "armed", "pending", "inner"],
        layout: Layout::FieldSetOnly { reason: STD_MUTEX },
    },
    Expected {
        name: "StageInner",
        fields: &[
            "records",
            "fold_anchor",
            "last_served_seq",
            "dropped",
            "dropped_reported",
            "dropped_announced",
            "next_decade",
        ],
        // Re-snapshot (MEASURED via `dump_measured_table`, never
        // hand-computed): `fold_anchor: FoldAnchor` is a one-byte enum that
        // lands at offset 57, in what was tail padding — the struct stays 64
        // bytes and every other offset is unchanged. It carries the two facts a
        // run count asserts that no single record can hold: that the run is
        // ADJACENT (a drop at the rim breaks it) and that its provenance is its
        // own (a bare read never folds into an annotated pair's read half).
        //
        // The ABI VERSION is unchanged at 19, on the same rule applied to
        // `StagedReadOutcome` two entries down: the version already moved for
        // this struct family, and a later change to an FFI-closure struct in
        // the same release is a re-snapshot rather than a second bump. Host and
        // cdylib are built together from one tree, so what the version must
        // describe is the layout that SHIPS; bumping again would claim a
        // compatibility boundary against an intermediate state nobody ever
        // deployed.
        //
        // The NUMBER is 19, not 18: version 18 belongs to
        // `CerulionSubscriber::max_borrowed_samples`, which landed first with
        // that number, so this struct's entry took the next one — two different
        // structs across the same FFI cannot
        // share a version, or a cdylib built against one reads the other's fields
        // at the wrong offsets.
        layout: Layout::Pinned {
            size: 64,
            align: 8,
            offsets: &[0, 57, 24, 32, 40, 56, 48],
        },
    },
    Expected {
        name: "StagedReadOutcome",
        fields: &["kind", "served_seq", "popped", "token", "role", "run_count"],
        // Re-snapshot (MEASURED via `dump_measured_table`, never
        // hand-computed): `run_count: u32` grows the struct 32 -> 40 bytes and
        // moves `kind`/`role`. The ABI
        // VERSION is unchanged at 19 — it already moved for this family, and this
        // struct's entry is a re-snapshot of the same version, not a bump. (19
        // rather than 18: see the collision noted two entries
        // up.)
        layout: Layout::Pinned {
            size: 40,
            align: 8,
            offsets: &[32, 16, 24, 0, 34, 28],
        },
    },
    Expected {
        name: "ReadOutcomeKind",
        fields: &[],
        layout: Layout::Pinned {
            size: 2,
            align: 2,
            offsets: &[],
        },
    },
    Expected {
        name: "ReadSiteRole",
        fields: &[],
        layout: Layout::Pinned {
            size: 1,
            align: 1,
            offsets: &[],
        },
    },
    Expected {
        name: "ReadStageRole",
        fields: &[],
        layout: Layout::Pinned {
            size: 1,
            align: 1,
            offsets: &[],
        },
    },
    Expected {
        name: "QosEventStore",
        fields: &["expect", "promise", "liveliness"],
        layout: Layout::FieldSetOnly { reason: STD_MUTEX },
    },
    // ---- the two `Arc`-reached owners a cdylib's own code indexes --------
    Expected {
        name: "TransportManager",
        fields: &[
            "network",
            "node",
            "clock",
            "subscriber_buffer_size",
            "bridge_mgr",
            "network_watch_started",
            "degraded_warned",
            "dynamic_egress",
            "dynamic_egress_create",
            "ingress_build",
            "replay_sequence_seeds",
            "mirror_registry",
            "self_weak",
        ],
        layout: Layout::FieldSetOnly {
            reason: IOX2_NODE_AND_MUTEXES,
        },
    },
    Expected {
        name: "PublishTrace",
        fields: &["depth", "entries", "bag"],
        layout: Layout::FieldSetOnly { reason: STD_FILE },
    },
    Expected {
        name: "PublishTraceEntry",
        fields: &["topic", "sequence", "publish_time_ns", "schema_hash"],
        layout: Layout::Pinned {
            size: 40,
            align: 8,
            offsets: &[0, 32, 16, 24],
        },
    },
    Expected {
        name: "HistoryDepth",
        fields: &[],
        layout: Layout::Pinned {
            size: 24,
            align: 8,
            offsets: &[],
        },
    },
    Expected {
        name: "BagWriter",
        fields: &[
            "dir",
            "file_size_limit",
            "retention",
            "file_index",
            "current",
        ],
        layout: Layout::FieldSetOnly { reason: STD_FILE },
    },
    Expected {
        name: "CurrentFile",
        fields: &["path", "writer", "bytes_written"],
        layout: Layout::FieldSetOnly { reason: STD_FILE },
    },
    Expected {
        name: "BagRetention",
        fields: &["max_bytes", "max_duration"],
        layout: Layout::Pinned {
            size: 32,
            align: 8,
            offsets: &[0, 16],
        },
    },
];

/// Every FFI-crossing struct, measured in the module that defines it.
fn measured() -> Vec<MeasuredStruct> {
    let mut all = Vec::new();
    all.extend(crate::graph::node::abi_layout_pins());
    all.extend(crate::transport::publisher::abi_layout_pins());
    all.extend(crate::transport::subscriber::abi_layout_pins());
    all.extend(crate::transport::adaptive_sizer::abi_layout_pins());
    all.extend(crate::transport::notify_delivery_latch::abi_layout_pins());
    all.extend(crate::transport::output_discard_latch::abi_layout_pins());
    all.extend(crate::wire::abi_layout_pins());
    all.extend(crate::read_outcome::abi_layout_pins());
    all.extend(crate::scheduler::handle::abi_layout_pins());
    all.extend(crate::transport::abi_layout_pins());
    all.extend(crate::trace::publish::abi_layout_pins());
    all.extend(crate::trace::bag::abi_layout_pins());
    all
}

const BUMP_INSTRUCTION: &str = "\n\n  An FFI-crossing struct changed layout. `NodeContext` crosses the cdylib `init()` boundary \
     as a raw `Box`, so host and cdylib must agree on every offset in its closure: BUMP \
     `CERULION_ABI_VERSION` in cerulion_core/src/lib.rs (with a doc paragraph, as v13-v19 did) \
     AND re-snapshot this table — `cargo test -p cerulion_core --lib \
     abi_layout::dump_measured_table -- --ignored --nocapture` prints it in source form. If the \
     change is deliberate and the version was already bumped, only the re-snapshot is missing.";

#[test]
fn the_layout_table_is_keyed_to_the_current_abi_version() {
    assert_eq!(
        EXPECTED_ABI, CERULION_ABI_VERSION,
        "abi_layout: this table was snapshotted at ABI {EXPECTED_ABI} but \
         CERULION_ABI_VERSION is now {CERULION_ABI_VERSION}.\n\n  The ABI was bumped — re-snapshot \
         the layout table in cerulion_core/src/abi_layout.rs and set EXPECTED_ABI to \
         {CERULION_ABI_VERSION}. That re-snapshot IS the reminder the bump exists for: a table \
         left behind at an older version describes nothing, so the NEXT unbumped layout change \
         would sail through.\n  Re-snapshot with: cargo test -p cerulion_core --lib \
         abi_layout::dump_measured_table -- --ignored --nocapture"
    );
}

#[test]
fn every_ffi_crossing_struct_matches_its_pinned_layout() {
    let measured = measured();

    for m in &measured {
        let Some(expected) = EXPECTED.iter().find(|e| e.name == m.name) else {
            panic!(
                "abi_layout: `{}` is measured but has NO row in the snapshot table.\
                 {BUMP_INSTRUCTION}",
                m.name
            );
        };

        let measured_names: Vec<&str> = m.fields.iter().map(|f| f.name).collect();
        assert_eq!(
            measured_names, expected.fields,
            "abi_layout: `{}` field set drifted — expected {:?}, measured {:?}.\
             {BUMP_INSTRUCTION}",
            m.name, expected.fields, measured_names
        );

        match &expected.layout {
            Layout::FieldSetOnly { reason } => {
                // Absolute numbers are deliberately not claimed for this row
                // (it {reason}); the field set above is the pin. See the
                // module docs.
                let _ = reason;
            }
            Layout::Pinned {
                size,
                align,
                offsets,
            } => {
                assert_eq!(
                    offsets.len(),
                    expected.fields.len(),
                    "abi_layout: `{}`'s expected offsets and field names are different \
                     lengths — the table row is malformed, re-snapshot it.",
                    m.name
                );
                assert_eq!(
                    m.size, *size,
                    "abi_layout: `{}` size is {} bytes, expected {size}.\
                     {BUMP_INSTRUCTION}",
                    m.name, m.size
                );
                assert_eq!(
                    m.align, *align,
                    "abi_layout: `{}` alignment is {}, expected {align}.\
                     {BUMP_INSTRUCTION}",
                    m.name, m.align
                );
                for (field, want) in m.fields.iter().zip(offsets.iter()) {
                    assert_eq!(
                        field.offset, *want,
                        "abi_layout: `{}.{}` is at offset {}, expected {want}.\
                         {BUMP_INSTRUCTION}",
                        m.name, field.name, field.offset
                    );
                }
            }
        }
    }

    // No orphan rows: a struct deleted from the closure must leave the table.
    for e in EXPECTED {
        assert!(
            measured.iter().any(|m| m.name == e.name),
            "abi_layout: the table has a row for `{}` but nothing measures it — either \
             the struct left the FFI closure (drop the row) or its module's `abi_layout_pins()` \
             lost its entry (restore it; a silently unmeasured struct is an unpinned one).",
            e.name
        );
    }
}

/// The kind-6 wire discriminants are ABI surface too: the cdylib STAGES them
/// and the host PACKS them, so a renumbering that both sides agree on would
/// still corrupt every bag recorded across a version boundary.
#[test]
fn the_staged_wire_discriminants_are_pinned() {
    use crate::read_outcome::{ReadOutcomeKind, ReadSiteRole};

    assert_eq!(
        [
            ReadOutcomeKind::Served.wire(),
            ReadOutcomeKind::Held.wire(),
            ReadOutcomeKind::NoFrame.wire(),
            ReadOutcomeKind::DrainedBatch.wire(),
            ReadOutcomeKind::Decimated.wire(),
            ReadOutcomeKind::Truncated.wire(),
            ReadOutcomeKind::Producer.wire(),
        ],
        [1, 2, 3, 4, 5, 6, 7],
        "abi_layout: a `ReadOutcomeKind` wire value moved. These discriminants ARE the \
         kind-6 record's low-16 half; renumbering one silently re-labels every read in every bag \
         recorded before the change."
    );
    assert_eq!(
        [
            ReadSiteRole::Unstamped.wire(),
            ReadSiteRole::Drain.wire(),
            ReadSiteRole::Body.wire(),
            ReadSiteRole::Peek.wire(),
        ],
        [0, 1, 2, 3],
        "abi_layout: a `ReadSiteRole` wire value moved. `0` MUST stay `Unstamped` — a \
         format <= 4 bag carries zeroes in these bits, and any other meaning for `0` would make \
         every pre-roles record claim a call site nobody wrote. `3` is `Peek`, which \
         fills the two-bit field: a fifth variant needs a WIDER subfield and a `trace_format` bump."
    );
}

/// Print the measured table in source form, for re-snapshotting after a
/// deliberate ABI bump. `#[ignore]`d — it asserts nothing; it is the tool the
/// two failure messages above point at.
#[test]
#[ignore = "re-snapshot helper: prints the table, asserts nothing"]
fn dump_measured_table() {
    let mut out = String::new();
    out.push_str("static EXPECTED: &[Expected] = &[\n");
    for m in measured() {
        let pinned = EXPECTED
            .iter()
            .find(|e| e.name == m.name)
            .map(|e| matches!(e.layout, Layout::Pinned { .. }))
            .unwrap_or(true);
        out.push_str("    Expected {\n");
        out.push_str(&format!("        name: {:?},\n", m.name));
        let names: Vec<&str> = m.fields.iter().map(|f| f.name).collect();
        out.push_str(&format!("        fields: &{names:?},\n"));
        if pinned {
            let offsets: Vec<usize> = m.fields.iter().map(|f| f.offset).collect();
            out.push_str(&format!(
                "        layout: Layout::Pinned {{ size: {}, align: {}, offsets: &{:?} }},\n",
                m.size, m.align, offsets
            ));
        } else {
            out.push_str("        layout: Layout::FieldSetOnly { reason: /* keep */ },\n");
        }
        out.push_str("    },\n");
    }
    out.push_str("];\n");
    // The whole point of this helper is to put text in front of a human under
    // `--nocapture`; it is a test, not library code, so `println!` is correct.
    println!("{out}");
}
