// SPDX-License-Identifier: AGPL-3.0-only
//! The trace-specific layer over the generic [`crate::shm_ring`] SPSC ring.
//!
//! Where [`crate::shm_ring`] is a reusable "fixed-size opaque records in, zero-copy
//! byte slices out" ring (a later `logd` is meant to reuse it for a
//! second stream), THIS module fixes the record TYPE and the manifest CONTENT for
//! the deterministic record/replay trace:
//!
//! - [`TraceRingRecord`] — a 40-byte, endian-pinned, padding-free scheduler trace
//!   record (fire / departure / step-boundary events). Its byte layout is a FORMAT
//!   CONTRACT: the
//!   record IS the future on-disk bag trace payload, so it is hand-encoded
//!   little-endian (never transmuted) so the on-ring bytes are stable across
//!   architectures.
//! - The ring MANIFEST is the node-identity table ([`encode_manifest`] /
//!   [`decode_manifest`]): `node_idx` in a record indexes into it.
//!
//! [`TraceRingProducer::push`] is the interface the scheduler hook
//! wires: keep its signature `push(&mut self, &TraceRingRecord)` stable.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on
//! its `pub mod trace_ring;` declaration in `lib.rs`, so no redundant inner
//! `#![cfg(unix)]` is needed here (clippy 1.93 `duplicated_attributes`).

use crate::read_outcome::{ReadOutcomeKind, ReadSiteRole};
use crate::shm_ring::{
    prev_power_of_two, ShmRingConsumer, ShmRingError, ShmRingOwner, ShmRingProducer, ShmRingResult,
    MANIFEST_CAPACITY,
};

/// Record kind: a node fired at a DAG level. `node_idx` indexes the manifest;
/// `duration_ns` is the recorded tick duration.
pub const RECORD_TYPE_FIRE: u32 = 1;
/// Record kind: a cross-process DEPARTURE boundary (multi-process replay). For a
/// departure record `node_idx` carries the departed process RANK and `duration_ns`
/// is 0.
pub const RECORD_TYPE_DEPARTURE: u32 = 2;
/// Record kind: a per-STEP gating-clock boundary. Pushed ONCE
/// per scheduler step at `begin_step` when recording:
///
/// - `step` — the 0-based logical step index (`Scheduler::current_step`),
/// - `fire_time_ns` — the step's ADVANCED gating-clock value (the
///   `current_time_ns` every same-step publish stamps into its wire
///   `timestamp_ns`),
/// - `duration_ns` = 0, `node_idx` = 0, `global_level` = 0, `reserved` = 0
///   on the RING (rank lives in the ring header); when writing the record to
///   the BAG, `bagd` stamps the ring's rank into `reserved` (the
///   multi-process contract — rank 0 == the single-process 0, back-compatible).
///
/// This record is what makes the recorded clock trajectory REPLAYABLE:
/// a `Period` FIRE record's `fire_time_ns` is the period BOUNDARY,
/// which under wall-advance recording is strictly BELOW the step's advanced
/// clock — so FIRE records alone cannot reconstruct the per-step clock, and a
/// replay that re-advanced to `max(fire_time_ns)` would stamp different wire
/// timestamps than the recording (a false byte divergence). Replay re-advances
/// the gating clock to EXACTLY this record's value per step.
pub const RECORD_TYPE_STEP_BOUNDARY: u32 = 3;
// `record_type == 0` is INVALID (a zeroed slot). Kinds 4 (Keyframe)
// and 5 (NonDeterminism) stay RESERVED and MUST
// NOT be minted; kind 6 is minted below
// ([`RECORD_TYPE_READ_OUTCOME`]); values 7+ are RESERVED.

/// Record kind: a per-edge READ OUTCOME (`trace_format` 3) —
/// which frame(s) a consumer's input read actually served. Reuses the one
/// 40-byte [`TraceRingRecord`] layout with these field REINTERPRETATIONS
/// (a FORMAT CONTRACT — see the struct doc):
///
/// - `step` — the CONSUMER's logical step (unchanged meaning),
/// - `fire_time_ns` — the SERVED wire `sequence` as a `u64`
///   ([`READ_OUTCOME_NO_FRAME`] = no frame was served),
/// - `duration_ns` — packed aux via [`pack_read_outcome_aux`]: low 32 bits =
///   frames popped by this drain, high 32 bits = reserved 0 (the
///   re-offer-count reservation; see [`unpack_read_outcome_popped`]). The ONE
///   exception is the [`READ_OUTCOME_PRODUCER`] ANNOTATION kind, whose aux
///   slot is the FULL 64-bit producer token — which does not collide with the
///   reservation, because the format reserves the high 32 bits of a READ record's
///   aux (it is defined as a count on "the successor record"), and a
///   `Producer` record is not a read,
/// - `node_idx` — the CONSUMER node (manifest-indexed, unchanged meaning),
/// - `global_level` — packed via [`pack_read_outcome_meta`]: high 16 bits =
///   the input's index into the node's manifest input table
///   ([`encode_manifest_with_inputs`]), bits 14..16 = the READ-SITE
///   ROLE ([`ReadSiteRole`] — `0` on every `trace_format` <= 4 bag), bits
///   0..14 = the outcome kind ([`READ_OUTCOME_SERVED`] ..
///   [`READ_OUTCOME_DECIMATED`]),
/// - `reserved` — rank/discard exactly as for every other kind (0 on the
///   ring; `bagd` co-stamps the ring's header rank uniformly at drain time).
pub const RECORD_TYPE_READ_OUTCOME: u32 = 6;

/// Read-outcome kind: a fresh frame was served to the read (the low-16
/// half of the kind-6 `global_level` packing). The wire constants are
/// DERIVED from the portable [`ReadOutcomeKind`] enum's own discriminants
/// (one source; the in-module exhaustive-match drift guard
/// forces a new variant to mint its constant here).
pub const READ_OUTCOME_SERVED: u16 = ReadOutcomeKind::Served as u16;
/// Read-outcome kind: the HELD sample was replayed (no new
/// arrival); the served-seq slot carries the HELD frame's wire sequence.
pub const READ_OUTCOME_HELD: u16 = ReadOutcomeKind::Held as u16;
/// Read-outcome kind: no frame has EVER been delivered on this input —
/// the read observed nothing (served-seq slot = [`READ_OUTCOME_NO_FRAME`]).
pub const READ_OUTCOME_NONE: u16 = ReadOutcomeKind::NoFrame as u16;
/// Read-outcome kind: a trigger/batch drain — served-seq = the NEWEST
/// sequence in the batch, popped = the batch size.
pub const READ_OUTCOME_DRAINED_BATCH: u16 = ReadOutcomeKind::DrainedBatch as u16;
/// Read-outcome kind: the `sample(N)` gate popped frames but delivered
/// none — served-seq = the last-ACCEPTED sequence (or
/// [`READ_OUTCOME_NO_FRAME`] if nothing was ever accepted), popped > 0.
pub const READ_OUTCOME_DECIMATED: u16 = ReadOutcomeKind::Decimated as u16;
/// Annotation kind: the OVERFLOW MARKER, meaning "the records that
/// would have followed this position, in this merge window, were dropped at
/// the staging rim". Served-seq slot = [`READ_OUTCOME_NO_FRAME`]; the aux
/// word's low 32 bits carry the count dropped IN THIS WINDOW (a delta, never
/// the lifetime total) and the high 32 stay 0 — the re-offer-count reservation is
/// untouched.
pub const READ_OUTCOME_TRUNCATED: u16 = ReadOutcomeKind::Truncated as u16;
/// Annotation kind: the PRODUCER TOKEN for the NEXT read record
/// on the same `(node, input)`. Served-seq slot REPEATS the annotated read's
/// served sequence (a redundant join key); the aux word is the FULL 64-bit
/// token (see [`TraceRingRecord::read_outcome_producer`]).
pub const READ_OUTCOME_PRODUCER: u16 = ReadOutcomeKind::Producer as u16;

/// The "no frame" sentinel for a kind-6 record's served-seq slot
/// (`fire_time_ns`). A real wire sequence is a `u32`, so the two ranges can
/// never collide.
pub const READ_OUTCOME_NO_FRAME: u64 = u64::MAX;

/// The outcome-kind subfield of a kind-6 record's `global_level`
/// slot — bits 0..14. It NARROWED from 16 bits when the role bits took the top
/// of the kind half; the highest kind in use is 7
/// ([`READ_OUTCOME_PRODUCER`]) and `read_outcome_kind_from_wire` already
/// answers "unknown" for anything it does not recognise, so the narrowing
/// changes no reachable behaviour.
pub const READ_OUTCOME_KIND_MASK: u16 = 0x3FFF;

/// The bit position of the READ-SITE ROLE subfield (bits 14..16)
/// of a kind-6 record's `global_level` slot.
///
/// It sits at the TOP of the kind half rather than the bottom, and that is
/// what makes a role-0 record's meta word NUMERICALLY IDENTICAL to a pre-role
/// one: `input_idx` does not move and the kind stays in the low bits, so every
/// packed-value oracle written before roles existed still holds verbatim
/// (pinned by the in-module invariance oracle).
pub const READ_OUTCOME_ROLE_SHIFT: u32 = 14;

/// The two-bit mask of the read-site role subfield (post-shift).
///
/// All FOUR values are spoken for since the format-5 role definition that made `3` the
/// `Peek` site ([`ReadSiteRole`]), so widening this mask is a `trace_format`
/// change, not an additive edit.
pub const READ_OUTCOME_ROLE_MASK: u32 = 0b11;

/// The three constants above are ONE layout, and nothing else
/// says so.
///
/// They were declared independently, so a future narrowing (a third role value
/// wanting three bits, a wider kind field) could move one and leave the others
/// describing a layout that no longer exists — silently, because every packer
/// and every reader would still compile and would simply disagree about where
/// the role lives. The kind mask must be exactly the bits BELOW the role
/// shift, and the role subfield must reach the top of the 16-bit kind half:
///
/// * `KIND_MASK == (1 << ROLE_SHIFT) - 1` — no gap and no overlap between the
///   two fields;
/// * `ROLE_SHIFT + ROLE_MASK.count_ones() == 16` — the role occupies the top
///   of the half, so `input_idx` (bits 16..32) is untouched, which is what
///   keeps a role-0 word numerically identical to a pre-role one.
const _: () = assert!(
    READ_OUTCOME_KIND_MASK as u32 == (1u32 << READ_OUTCOME_ROLE_SHIFT) - 1,
    "the kind mask must be exactly the bits below the role shift"
);
const _: () = assert!(
    READ_OUTCOME_ROLE_SHIFT + READ_OUTCOME_ROLE_MASK.count_ones() == 16,
    "the role subfield must reach the top of the 16-bit kind half"
);

/// Pack a kind-6 record's `global_level` slot — the input index in
/// the HIGH 16 bits, the read-site role in bits 14..16, the outcome
/// kind in bits 0..14. Total inverse of [`unpack_read_outcome_meta_full`]
/// (pinned by the in-module oracle tests).
///
/// The kind is MASKED to [`READ_OUTCOME_KIND_MASK`] so a caller handing a
/// value the field cannot hold can never corrupt the role bits (it would
/// silently re-label the record's SITE, which is the one thing a reader
/// steers on). Production can never reach that — every kind is a
/// [`ReadOutcomeKind`] discriminant <= 7 — so it is a `debug_assert!` plus a
/// mask rather than a fallible signature.
#[inline]
pub fn pack_read_outcome_meta(input_idx: u16, outcome_kind: u16, role: ReadSiteRole) -> u32 {
    debug_assert!(
        outcome_kind & !READ_OUTCOME_KIND_MASK == 0,
        "a kind-6 outcome kind must fit the 14-bit field (bits 14..16 are the \
         read-site role); got {outcome_kind}"
    );
    (u32::from(input_idx) << 16)
        | ((u32::from(role.wire()) & READ_OUTCOME_ROLE_MASK) << READ_OUTCOME_ROLE_SHIFT)
        | u32::from(outcome_kind & READ_OUTCOME_KIND_MASK)
}

/// Unpack a kind-6 record's `global_level` slot into
/// `(input_idx, outcome_kind)`.
///
/// The role is deliberately NOT returned here: the overwhelming majority of
/// this function's callers want the addressing and the kind, and every one of
/// them predates roles. A reader that wants the role asks
/// [`read_site_role`]; a reader that wants the TOTAL inverse of
/// [`pack_read_outcome_meta`] asks [`unpack_read_outcome_meta_full`].
#[inline]
pub fn unpack_read_outcome_meta(meta: u32) -> (u16, u16) {
    ((meta >> 16) as u16, (meta as u16) & READ_OUTCOME_KIND_MASK)
}

/// Read a kind-6 record's READ-SITE ROLE out of its
/// `global_level` slot.
///
/// `0` (never written — every format <= 4 bag) decodes to
/// [`ReadSiteRole::Unstamped`], which is the pre-roles arm at every consumer.
/// `1`/`2`/`3` are `Drain`/`Body`/`Peek`.
#[inline]
pub fn read_site_role(meta: u32) -> ReadSiteRole {
    ReadSiteRole::from_wire(((meta >> READ_OUTCOME_ROLE_SHIFT) & READ_OUTCOME_ROLE_MASK) as u16)
}

/// Unpack a kind-6 record's `global_level` slot into
/// `(input_idx, outcome_kind, role)` — the TOTAL inverse of
/// [`pack_read_outcome_meta`].
#[inline]
pub fn unpack_read_outcome_meta_full(meta: u32) -> (u16, u16, ReadSiteRole) {
    let (input_idx, kind) = unpack_read_outcome_meta(meta);
    (input_idx, kind, read_site_role(meta))
}

/// Pack a kind-6 record's `duration_ns` slot — the popped count in
/// the LOW 32 bits, the high 32 bits reserved 0. Total inverse of
/// [`unpack_read_outcome_popped`] over the low half.
#[inline]
pub fn pack_read_outcome_aux(popped: u32) -> u64 {
    u64::from(popped)
}

/// Read the popped count (low 32 bits) out of a kind-6 record's
/// `duration_ns` slot. The high 32 bits are reserved 0 in format 3 and are
/// deliberately NOT validated here (a future format may assign them —
/// the pre-decided re-offer COUNT is exactly that reservation).
///
/// # The re-offer-count split, and why the producer token does not collide with it
///
/// The format reserves the aux word's high 32 bits as a re-offer count "on the
/// SUCCESSOR record" — i.e. it is a property of a READ record's aux, the
/// record that reports what a drain served. [`READ_OUTCOME_PRODUCER`] is an
/// ANNOTATION, not a read: it reports no drain, carries no popped count, and
/// its aux slot is the full 64-bit token
/// ([`TraceRingRecord::read_outcome_producer`]). So the two never contend for
/// the same bits — a `Producer` record's aux is never read through this
/// function, and a READ record's high 32 bits are still free for that count. A
/// future format that assigns them must therefore change the READ records
/// only; nothing about the token encoding narrows that decision.
#[inline]
pub fn unpack_read_outcome_popped(aux: u64) -> u32 {
    aux as u32
}

/// The two halves of a kind-6 record's aux word, as ONE value.
///
/// They travel together by construction — `popped` in the low 32 bits, the fold
/// `run_count` in the high 32 — so passing them as one argument is what the
/// wire already does. It also keeps the record constructor at seven parameters,
/// which is the repo's clippy ceiling; bundling the pair the encoding pairs is
/// the right way to stay under it rather than an `#[allow]` that says the
/// signature is fine when the lint says it is not.
/// The fields are PRIVATE and the only ways in are [`ReadRun::once`] and
/// [`ReadRun::folded`], which is the whole of the type's invariant: **a run is
/// at least one occurrence**. A literal `ReadRun { run_count: 0 }` was
/// constructible before and meant nothing — it packs to a zero high half, which
/// `unpack_read_outcome_run` reads back as `1`, so in memory the type had two
/// spellings of "one occurrence" and a fold taking a `0`-valued record to `1`
/// would have LOST an occurrence. The `0 == 1` translation now lives in exactly
/// one place, `unpack_read_outcome_run`, which is where a wire convention
/// belongs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReadRun {
    /// Frames this read popped (the low half; unchanged since kind 6 was minted).
    popped: u32,
    /// Consecutive byte-identical occurrences this record stands for; `>= 1` by
    /// construction.
    run_count: u32,
}

impl ReadRun {
    /// One occurrence — what every unfolded read carries.
    #[must_use]
    pub const fn once(popped: u32) -> Self {
        Self {
            popped,
            run_count: 1,
        }
    }

    /// A run of `run_count` occurrences. `0` is not a run: it SATURATES to one
    /// rather than being rejected, because the only caller is the recorder's
    /// own staging drain and a panic there would take down a recording to
    /// describe a record it could simply write correctly.
    #[must_use]
    pub const fn folded(popped: u32, run_count: u32) -> Self {
        Self {
            popped,
            run_count: if run_count == 0 { 1 } else { run_count },
        }
    }

    /// Frames this read popped.
    #[must_use]
    pub const fn popped(self) -> u32 {
        self.popped
    }

    /// Occurrences this record stands for — always `>= 1`.
    #[must_use]
    pub const fn run_count(self) -> u32 {
        self.run_count
    }
}

/// Pack a kind-6 record's `duration_ns` slot with BOTH halves —
/// the popped count in the low 32 bits and the RUN COUNT in the high 32.
///
/// The high half was reserved 0 through format 5 and was already NAMED for
/// exactly this use (see [`unpack_read_outcome_popped`]'s reservation note). Nothing
/// widens: `TRACE_RECORD_SIZE` stays 40 and the meta word, which has no spare
/// bits, is untouched.
///
/// `0 == 1 occurrence`, so every format-≤5 record — whose high bits are
/// structurally zero — decodes correctly as a single unfolded read. That
/// convention is what makes the section additive rather than breaking.
///
/// # A run of ONE is written in the unfolded shape
///
/// Writing a literal `1` into the high half of every unfolded
/// record, on the reasoning that an explicit count is a clearer statement of
/// intent, would be WRONG, for a reason the fold cannot see from inside its own
/// design: `CERULION_READ_LOG_FOLD=off` makes the recorder stamp `trace_format`
/// **5**, and a format-5 bag's high bits are supposed to be structurally zero —
/// so a fold-OFF run would write a `1 << 32` into every read of a bag that claimed a
/// format in which those bits do not exist. The bag would contradict its own stamp.
///
/// Writing `0` for a run of one avoids that WITHOUT consulting the switch, which
/// matters because the stamp is decided in one process (the supervisor's
/// `graph_cmd`) while the packing happens in another (each `graph run-worker`),
/// each reading the env through its own `OnceLock` — an encoding that asked the
/// switch would have had to keep two processes in agreement to stay correct.
/// The invariant is now positional and local: **a nonzero high half means a
/// REAL fold**, an unfolded record is byte-identical to what a recorder
/// without the fold writes, and the format number goes on saying what it always said —
/// which WRITER produced the bag.
#[inline]
pub fn pack_read_outcome_aux_run(popped: u32, run_count: u32) -> u64 {
    let high = if run_count <= 1 { 0 } else { run_count };
    u64::from(popped) | (u64::from(high) << 32)
}

/// Read the RUN COUNT out of a kind-6 record's `duration_ns`
/// slot — the inverse of [`pack_read_outcome_aux_run`]'s high half.
///
/// `0` decodes as `1`: a record that was never folded occupies exactly one
/// position, and so does every record written by a recorder without the fold.
///
/// NEVER call this on a [`READ_OUTCOME_PRODUCER`] annotation: its aux slot is
/// the full 64-bit token, so its "high half" is token bits. A folded PAIR
/// carries its count on the READ half only, which is why the
/// annotation never needs one.
#[inline]
pub fn unpack_read_outcome_run(aux: u64) -> u32 {
    match (aux >> 32) as u32 {
        0 => 1,
        n => n,
    }
}

/// The per-node tables a full manifest carries, as ONE value.
///
/// They are one thing — the manifest's additive sections, parallel to the node
/// table, always passed together — and bundling them is what keeps the degrade
/// ladder's widest entry point under the repo's 7-argument clippy ceiling without
/// an `#[allow]` that would say the signature is fine when the lint says
/// it is not.
///
/// `pub(crate)`: the only signature naming it is this module's own private
/// `create_with_inputs_publishers_and_capacities_or_degrade_marked`, and nothing
/// outside the crate names the type at all — so `pub` claimed a wider contract
/// than the bundle has.
#[derive(Clone, Copy)]
pub(crate) struct ManifestSections<'a> {
    /// The node-identity table every section is parallel to.
    pub node_ids: &'a [&'a str],
    /// Section 2: each node's ordered input names.
    pub node_inputs: &'a [&'a [&'a str]],
    /// Section 3: each node's `(output, publisher id)` pairs.
    pub node_publishers: &'a [&'a [(&'a str, u128)]],
    /// Section 4: each node's `(input_idx, role, capacity)` staging rows.
    pub node_capacities: &'a [&'a [(u16, u8, u32)]],
}

/// Which manifest SECTIONS a degrade rung had to drop.
///
/// A bit set in the ring header (`ShmRingConsumer::degraded_sections`), because it
/// has to survive the very thing it reports: the bottom rung drops the INPUT
/// section, so a marker inside the manifest is a marker that rung can drop.
///
/// `0` means "nothing dropped", which is also what a ring written without this word reads —
/// the conservative value, since a recorder without the degrade ladder never degrades silently in a
/// way a reader needs to know about.
///
/// # Why a reader cannot work this out for itself
///
/// An absent section does not say WHY it is absent. A rank with no wired inputs
/// and a rank whose input section was dropped decode to the SAME empty lists, and
/// they call for opposite answers: the first genuinely declares no staging
/// (`NoClaim`, compare normally), the second has lost what it declared
/// (`Unreadable`, stand down LOUDLY). Only the writer knows which happened, so
/// only the writer can say.
pub struct DegradedSections;

impl DegradedSections {
    /// The per-node INPUT section (section 2) was dropped.
    pub const INPUTS: u32 = 1 << 0;
    /// The per-node PUBLISHER section (section 3) was dropped.
    pub const PUBLISHERS: u32 = 1 << 1;
    /// The per-node CAPACITY section (section 4) was dropped.
    pub const CAPACITIES: u32 = 1 << 2;

    /// Whether anything that bears on READ-LOG staging was dropped — the inputs
    /// (which resolve a record's `input_idx` to a name) or the capacities (the
    /// rims a replay adopts). The publisher section is offline attribution only.
    #[must_use]
    pub const fn affects_read_log(sections: u32) -> bool {
        sections & (Self::INPUTS | Self::CAPACITIES) != 0
    }
}

/// The fixed size of a [`TraceRingRecord`] on the ring / on disk, in bytes.
pub const TRACE_RECORD_SIZE: u32 = 40;

/// The DISCARD marker — the high bit of a FIRE record's [`reserved`]
/// field, set by the scheduler ([`push_fire`]) when a live tick committed
/// **zero** of its loaned outputs (a pre-commit loan/borrow failure under SHM
/// pressure, or the all-defer discard). It co-exists with the multi-process
/// worker-rank stamp in the same 32-bit field: the rank occupies the low 31
/// bits ([`TRACE_RANK_MASK`]), the discard flag bit 31. Replay reads the flag
/// via [`is_discarded`](TraceRingRecord::is_discarded) and SUPPRESSES the marked
/// fire's outputs — the byte-identical mirror of the live discard (no phantom
/// seq is burned, so every later frame's wire `sequence` stays aligned).
///
/// Collision-freedom: a legal rank is `< 2^16 <=
/// TRACE_RANK_MASK`, so it never sets bit 31; `reserved = rank | maybe-bit`
/// therefore decodes uniquely. A discard-marked FIRE record can never equal the
/// departure sentinel `u32::MAX` (that would require `rank == 0x7FFF_FFFF`), so
/// the raw-`reserved` departure gate stays exact.
///
/// [`reserved`]: TraceRingRecord::reserved
/// [`push_fire`]: crate::scheduler
pub const TRACE_DISCARD_BIT: u32 = 1 << 31; // 0x8000_0000

/// The worker-rank subfield mask (bits `[0..31)`) of a FIRE record's
/// [`reserved`](TraceRingRecord::reserved) — everything below
/// [`TRACE_DISCARD_BIT`]. Every replay site that reads `reserved` as a rank goes
/// through [`rank`](TraceRingRecord::rank), which masks with this constant, so a
/// forgotten mask is structurally impossible.
pub const TRACE_RANK_MASK: u32 = 0x7FFF_FFFF; // bits [0..31)

/// The `u32::MAX` sentinel a multi-process supervisor stamps as the
/// rank of its own DEPARTURE ring — never a worker rank.
///
/// It was moved HERE from the two places that had grown their own copy
/// (`graph_cmd` mints it, `replay_cmd` re-declared it "to avoid cross-module
/// coupling"). It is not a module's private convention: it is a value written
/// into the wire form this module defines, and read back by every consumer of
/// that form — so a copy that drifted would make a writer and a reader disagree
/// about which records mean "a peer died".
///
/// It cannot collide with a legal rank (a rank is `< 2^16`) nor with a
/// discard-marked FIRE record, which would need `rank == 0x7FFF_FFFF` — see
/// [`TRACE_DISCARD_BIT`].
pub const DEPARTURE_RING_RANK: u32 = u32::MAX;

/// The rank whose `STEP_BOUNDARY` records ARE the recording's clock
/// trajectory — the one every reader of a boundary target walks.
///
/// It is 0, and it always was; what is new is that it has a NAME. `replay_cmd`
/// and `replay_engine` spell it as a bare `0` at half a dozen call sites
/// (`BoundaryCursor::for_rank(trace, 0)`, `first_recorded_boundary`,
/// `last_replayed_step`, the per-topic consistency cursor), each carrying its own
/// copy of the same justification: *"rank 0's boundary targets are the
/// authoritative clock trajectory — every graph publish stamps the SHARED gating
/// clock, and `validate_step_boundaries` phase 2 PINS the peer ranks' targets to
/// rank 0's on every shared step"*.
///
/// The value now has a SECOND kind of reader: the RECORDER, which must
/// measure the last boundary target its capture carries so the bag can state the
/// range a resume covers. A recorder-side literal `0` and a replay-side literal
/// `0` are two derivations of one fact, and the whole point of the covered range
/// is that the two sides answer identically; so the constant lives in the crate
/// both already depend on, beside the wire form it describes.
pub const AUTHORITATIVE_TRACE_RANK: u32 = 0;

/// **A record-level refusal `bag play --resim` applies to every trace
/// record — the SHARED condition, so a capture's verdict cannot drift from the
/// gate.**
///
/// `run_replay` walks the trace channel once and refuses on four things
/// (`replay_cmd.rs`, the loop at 628-722). A Flashback capture decides whether it
/// may claim `resimmable` by asking the SAME questions, and the whole point of
/// putting them here is that there is one copy: a refusal added to the replay
/// gate without a matching arm in the judge is exactly how a bag comes to be
/// stamped resimmable and then refused — the class this file's
/// [`TraceRingRecord::is_departure_boundary`] already closed for departures.
///
/// Departure is deliberately NOT a variant: `run_replay` hoists that check ABOVE
/// this one (pre-canonicalization) and the judge already carries it as
/// its own fact, so folding it in here would give one condition two homes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TraceRecordFault {
    /// The record's rank stamp names a rank the bag has no manifest for.
    ForeignRank {
        /// The masked rank the record carries.
        rank: u32,
        /// How many rank manifests exist.
        ranks_known: usize,
    },
    /// A manifest-indexed record's `node_idx` is beyond its OWN rank's manifest
    /// table.
    ///
    /// TWO record kinds are manifest-indexed and both land here: a FIRE record
    /// (the node that fired) and — since kind 6 was minted — a READ-OUTCOME record (the
    /// CONSUMER node whose read was logged). They share ONE variant because they
    /// share one fault: an index the rank's table cannot resolve, which is
    /// corruption whichever kind carried it. The kind rides `record_type` so the
    /// message can NAME which one, because that is what tells an operator where
    /// to look — and so the two renderers (this type's [`detail`](Self::detail)
    /// and `run_replay`'s own wording) cannot disagree about which record kinds
    /// are index-bearing.
    NodeIdxOutOfRange {
        /// The record's rank.
        rank: u32,
        /// The out-of-range index.
        node_idx: u32,
        /// How many node ids that rank's manifest lists.
        nodes: usize,
        /// WHICH manifest-indexed kind carried the bad index —
        /// [`RECORD_TYPE_FIRE`] or [`RECORD_TYPE_READ_OUTCOME`]. Render it with
        /// [`trace_record_noun`].
        record_type: u32,
    },
    /// `record_type` 0 — a zeroed slot, i.e. a corrupt recording.
    ZeroedRecord,
    /// A reserved or unknown `record_type` (3 is STEP_BOUNDARY; 4+ are reserved
    /// for future kinds).
    UnsupportedRecordType {
        /// The unrecognised kind.
        record_type: u32,
    },
}

impl TraceRecordFault {
    /// One sentence naming the fault, for a capture manifest's refusal reason.
    ///
    /// `run_replay` renders its own wording into its own error variants (it has
    /// the record INDEX and the manifest NAME, which this type deliberately does
    /// not carry); this is the capture side's rendering of the same fact.
    pub fn detail(&self) -> String {
        match self {
            Self::ForeignRank { rank, ranks_known } => format!(
                "a trace record is stamped with rank {rank}, but this capture carries worker \
                 trace manifests only for ranks 0..={}",
                ranks_known.saturating_sub(1)
            ),
            Self::NodeIdxOutOfRange {
                rank,
                node_idx,
                nodes,
                record_type,
            } => format!(
                "a scheduler-trace {} record references node_idx {node_idx} but rank {rank}'s \
                 manifest lists only {nodes} node id(s)",
                trace_record_noun(*record_type)
            ),
            Self::ZeroedRecord => {
                "a scheduler-trace record has record_type 0 — an invalid (zeroed) record, so \
                 the trace is corrupt"
                    .to_string()
            }
            Self::UnsupportedRecordType { record_type } => format!(
                "a scheduler-trace record has record_type {record_type}, a reserved kind this \
                 Cerulion version does not understand — the trace was written by a newer or \
                 foreign writer"
            ),
        }
    }
}

/// PURE: classify ONE trace record against the rank tables, in `run_replay`'s
/// own order (rank stamp first, then the record-type match).
///
/// `rank_node_counts` is indexed by RANK and holds that rank's manifest node
/// count — the same shape `run_replay` walks (`rank_tables[rank].1.len()`).
///
/// The caller MUST have already answered [`TraceRingRecord::is_departure_boundary`]
/// for this record: that gate is hoisted above this one and a departure record
/// reaching here would be classified on its `record_type` instead.
pub fn classify_trace_record(
    rec: &TraceRingRecord,
    rank_node_counts: &[usize],
) -> Option<TraceRecordFault> {
    // Rank BEFORE the type match, so a foreign-rank record is diagnosed as the
    // rank fault it is rather than through a table it does not belong to.
    // `.rank()` masks the discard bit, exactly as the gate does.
    let rank = rec.rank();
    let Some(nodes) = rank_node_counts.get(rank as usize).copied() else {
        return Some(TraceRecordFault::ForeignRank {
            rank,
            ranks_known: rank_node_counts.len(),
        });
    };
    match rec.record_type {
        // A READ-OUTCOME record's `node_idx` is the CONSUMER node and
        // is manifest-indexed exactly like a FIRE's, so it takes the SAME bounds
        // check — a stamp past the table is corruption whichever kind carried
        // it. Sharing the arm is what keeps a new index-bearing kind from
        // quietly skipping the check.
        RECORD_TYPE_FIRE | RECORD_TYPE_READ_OUTCOME if rec.node_idx as usize >= nodes => {
            Some(TraceRecordFault::NodeIdxOutOfRange {
                rank,
                node_idx: rec.node_idx,
                nodes,
                record_type: rec.record_type,
            })
        }
        RECORD_TYPE_FIRE | RECORD_TYPE_STEP_BOUNDARY | RECORD_TYPE_READ_OUTCOME => None,
        0 => Some(TraceRecordFault::ZeroedRecord),
        other => Some(TraceRecordFault::UnsupportedRecordType { record_type: other }),
    }
}

/// PURE: the FIRST worker rank a manifest set is missing, or `None` if the set
/// is contiguous `0..=max`.
///
/// `worker_ranks` must be SORTED ASCENDING, DEDUPED, and must already exclude
/// the [`DEPARTURE_RING_RANK`] sentinel — the three things `run_replay` does
/// before it asks, and the reason this takes a prepared slice rather than
/// re-deriving them: the sentinel is a supervisor artifact deliberately outside
/// the worker roster, and duplicate ranks are a DIFFERENT corruption with a
/// different message. An empty slice answers `None` (there is no roster to have
/// a hole in); the caller decides whether an empty roster is itself a refusal.
///
/// # Why it lives here rather than at either caller
///
/// `cerulion bag play --resim` REFUSES a bag whose worker manifests are not
/// contiguous from 0 (`replay_cmd::load_trace_manifests` →
/// `ReplayError::MultiRankManifestGap`), and a Flashback capture must answer the
/// SAME question before it may claim `resimmable`. It could not: the capture
/// side builds a rank-indexed table (`bagd::rank_node_counts`) that ZERO-FILLS a
/// gap rank, so a bag whose manifests are `{0, 7}` has every record resolve
/// against a table with six empty entries, the judge finds nothing to refuse,
/// and the stamp says `resimmable: true` — against a replay that refuses the bag
/// before it loads anything.
///
/// Zero-filling is CORRECT for the table's own purpose (a rank with no manifest
/// must read as ABSENT rather than as a rank with zero nodes — see that
/// function), so the answer is not to change it. It is to ask the contiguity
/// question SEPARATELY, in one place, exactly as
/// [`TraceRingRecord::is_departure_boundary`] and [`classify_trace_record`]
/// already are.
pub fn first_rank_manifest_gap(worker_ranks: &[u32]) -> Option<u32> {
    let max = *worker_ranks.last()?;
    (0..=max)
        .zip(worker_ranks.iter().copied())
        .find(|(expected, actual)| expected != actual)
        .map(|(expected, _)| expected)
}

/// The operator-facing NOUN for a manifest-indexed trace record kind.
///
/// Lives beside the classifier because BOTH renderers of
/// [`TraceRecordFault::NodeIdxOutOfRange`] — the capture side's
/// [`TraceRecordFault::detail`] and `run_replay`'s own error wording — must name
/// the same kind the same way; a second copy is how the two come to disagree.
/// Any other kind renders as the bare number, which is accurate: nothing else is
/// index-bearing, so reaching this with one is a bug rather than a diagnosis.
pub fn trace_record_noun(record_type: u32) -> String {
    match record_type {
        RECORD_TYPE_FIRE => "FIRE".to_string(),
        RECORD_TYPE_READ_OUTCOME => "READ-OUTCOME".to_string(),
        other => format!("record_type {other}"),
    }
}

/// WHY the shared record walk stopped — the refusal `run_replay` would raise on
/// the FIRST record it refuses, whichever KIND of refusal that is.
///
/// The gate (`replay_cmd.rs`) is ONE walk over the trace in FILE ORDER, and it
/// refuses per record: the departure boundary first, then
/// [`classify_trace_record`]. So which gap a bag reports is decided by WHICH
/// RECORD COMES FIRST, never by which KIND of problem it is — and a capture that
/// judged the two kinds in a fixed precedence would name a different gap from the
/// one `bag play --resim` names, on a bag carrying both. That is the exact drift
/// this shared vocabulary exists to prevent, so the walk reports the
/// first refusal of EITHER kind rather than collapsing one of them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TraceRecordRefusal {
    /// A DEPARTURE (fault) boundary — see
    /// [`TraceRingRecord::is_departure_boundary`]. `run_replay` refuses it with
    /// `DegradedRecordingDeparture`; a capture refuses it as fault replay.
    ///
    /// Carries no payload: the gate renders the record's INDEX and which of the
    /// two departure signals fired, neither of which this pure walk holds, and
    /// the capture side reports the departure COUNT it measured separately.
    Departure,
    /// A record-level fault beyond the departure gate — the four things
    /// [`classify_trace_record`] refuses.
    Fault(TraceRecordFault),
}

/// The FIRST record the replay gate would refuse, in FILE ORDER — or `None` if
/// it would refuse none of them.
///
/// This is the SAME predicate, asked in the same ORDER, as `run_replay`'s trace
/// walk: for each record the departure gate first, then
/// [`classify_trace_record`]; the first record that trips either one ends the
/// walk and IS the answer.
///
/// A departure therefore does not "outrank" a fault, nor the other way round —
/// whichever record comes first wins, because that is the one the gate reaches
/// first. A function that early-returned `None` at a
/// departure would make the capture side judge the two kinds by an AGGREGATE
/// precedence: a trace carrying a malformed record BEFORE a departure would advertise
/// fault replay while the gate refuses it at the earlier record with a different
/// error entirely — two named gaps for one bag.
pub fn first_trace_record_refusal<'a>(
    records: impl IntoIterator<Item = &'a TraceRingRecord>,
    rank_node_counts: &[usize],
) -> Option<TraceRecordRefusal> {
    for rec in records {
        if rec.is_departure_boundary() {
            return Some(TraceRecordRefusal::Departure);
        }
        if let Some(fault) = classify_trace_record(rec, rank_node_counts) {
            return Some(TraceRecordRefusal::Fault(fault));
        }
    }
    None
}

/// A single scheduler trace record — the unit written to the trace ring and (later)
/// the bag file.
///
/// `#[repr(C)]` with no trailing padding (3×`u64` + 4×`u32` = 40 bytes, 8-aligned),
/// but the on-ring bytes are produced by [`as_bytes`](Self::as_bytes) /
/// [`from_bytes`](Self::from_bytes) — an explicit little-endian encode, NOT a
/// transmute — so the layout is endian-pinned and padding-free by construction. The
/// `#[repr(C)]` + const-assert exist to keep the in-memory struct matched to the
/// 40-byte wire form.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct TraceRingRecord {
    /// Logical scheduler step.
    pub step: u64,
    /// Recorded fire time (gating clock nanos).
    pub fire_time_ns: u64,
    /// Recorded tick duration in nanos (0 for a departure record).
    pub duration_ns: u64,
    /// Index into the manifest node-id table (for a departure record: the departed
    /// process rank).
    pub node_idx: u32,
    /// The node's global DAG level. For a [`RECORD_TYPE_READ_OUTCOME`] record
    /// this slot is REINTERPRETED as the packed `(input_idx:16 | role:2 |
    /// outcome_kind:14)` word ([`pack_read_outcome_meta`]).
    pub global_level: u32,
    /// One of [`RECORD_TYPE_FIRE`] / [`RECORD_TYPE_DEPARTURE`] /
    /// [`RECORD_TYPE_STEP_BOUNDARY`] / [`RECORD_TYPE_READ_OUTCOME`]
    /// (0 invalid; 4/5 reserved for keyframe/nondeterminism; 7+ reserved).
    pub record_type: u32,
    /// 0 at mint — except a discard-marked FIRE, whose bit 31 carries
    /// [`TRACE_DISCARD_BIT`]. bagd co-stamps the owning worker
    /// rank into the low bits at bag-write time, uniformly for
    /// every kind. Read through [`rank`](Self::rank) /
    /// [`is_discarded`](Self::is_discarded), never raw.
    pub reserved: u32,
}

// Keep the in-memory struct locked to the 40-byte wire form.
const _: () = assert!(
    std::mem::size_of::<TraceRingRecord>() == TRACE_RECORD_SIZE as usize,
    "TraceRingRecord must be exactly 40 bytes"
);
const _: () = assert!(std::mem::align_of::<TraceRingRecord>() == 8);

impl TraceRingRecord {
    /// Encode to the 40-byte little-endian wire form (the format contract). Owned
    /// array, not a reference: the bytes are hand-built (never a transmute of the
    /// struct), so they are endian-independent and padding-free.
    pub fn as_bytes(&self) -> [u8; TRACE_RECORD_SIZE as usize] {
        let mut b = [0u8; TRACE_RECORD_SIZE as usize];
        b[0..8].copy_from_slice(&self.step.to_le_bytes());
        b[8..16].copy_from_slice(&self.fire_time_ns.to_le_bytes());
        b[16..24].copy_from_slice(&self.duration_ns.to_le_bytes());
        b[24..28].copy_from_slice(&self.node_idx.to_le_bytes());
        b[28..32].copy_from_slice(&self.global_level.to_le_bytes());
        b[32..36].copy_from_slice(&self.record_type.to_le_bytes());
        b[36..40].copy_from_slice(&self.reserved.to_le_bytes());
        b
    }

    /// The owning worker rank with the discard bit masked off — the
    /// low 31 bits ([`TRACE_RANK_MASK`]). Use this EVERYWHERE replay reads
    /// `reserved` as a rank (bounds checks, manifest indexing, per-rank demux),
    /// so the co-resident [`TRACE_DISCARD_BIT`] never misclassifies a marked
    /// fire as a foreign rank. A no-op on a legal rank (bit 31 already clear)
    /// and on STEP_BOUNDARY / departure records (their `reserved` never carries
    /// the discard bit). NOTE: the raw-`reserved` departure gate (`reserved ==
    /// u32::MAX`) must NOT go through this — it is evaluated before any rank
    /// extraction.
    #[inline]
    pub fn rank(&self) -> u32 {
        self.reserved & TRACE_RANK_MASK
    }

    /// True iff this is a FIRE record whose live tick committed **zero**
    /// of its loaned outputs (the [`TRACE_DISCARD_BIT`] is set). Gated on
    /// `record_type == RECORD_TYPE_FIRE` so a boundary/departure record that
    /// happens to carry a set bit (corrupt input) is never read as a discard —
    /// the marker is minted ONLY on FIRE records by the scheduler.
    #[inline]
    pub fn is_discarded(&self) -> bool {
        self.record_type == RECORD_TYPE_FIRE && (self.reserved & TRACE_DISCARD_BIT) != 0
    }

    /// Is this record a DEPARTURE (fault) boundary — a peer worker
    /// lost mid-run?
    ///
    /// Two signals, and BOTH are load-bearing: a `RECORD_TYPE_DEPARTURE` record,
    /// or ANY record stamped with the supervisor departure-ring sentinel rank
    /// ([`DEPARTURE_RING_RANK`]), which `bagd` writes into `reserved` for every
    /// record drained from that ring.
    ///
    /// # Why `reserved` is read RAW here, and must stay that way
    ///
    /// NOT through [`rank`](Self::rank). The mask clears the discard bit,
    /// and `u32::MAX & TRACE_RANK_MASK` is `0x7FFF_FFFF` — so a masked
    /// comparison against the sentinel can never match and the whole gate goes
    /// inert. This is the one place `reserved` is compared unmasked, and the
    /// masked-vs-raw distinction is deliberate.
    ///
    /// # Why it lives HERE rather than at each site that asks
    ///
    /// It is the predicate `cerulion bag play --resim` REFUSES a bag on
    /// (`ReplayError::DegradedRecordingDeparture`), and it is the predicate a
    /// Flashback capture must answer to decide whether it may claim
    /// `resimmable`. Two copies of one refusal rule is how the two answers drift
    /// — a capture stamped resimmable that resim then refuses is worse than a
    /// capture that never claimed it — so the rule is written once, in the crate
    /// both readers already depend on, and each of them calls it.
    #[inline]
    pub fn is_departure_boundary(&self) -> bool {
        self.record_type == RECORD_TYPE_DEPARTURE || self.reserved == DEPARTURE_RING_RANK
    }

    /// Build a [`RECORD_TYPE_READ_OUTCOME`] record — the ONE place
    /// the kind-6 field reinterpretation is assembled, so a producer cannot
    /// half-apply the packing. TYPED params: the outcome
    /// kind is the portable [`ReadOutcomeKind`] (its discriminants ARE the
    /// wire values), and `served_seq` is `Option<u32>` — the
    /// [`READ_OUTCOME_NO_FRAME`] sentinel is packed HERE, in the one assembly
    /// site, so no caller can hand a raw slot value that half-applies it.
    /// (A test crafting a foreign/corrupt kind builds the struct literally
    /// through the pub `pack_*` helpers.) `reserved` is written 0 (bagd
    /// co-stamps the rank at drain time, uniformly for every kind).
    ///
    /// `role` is the CALL-SITE role, threaded from the mint site
    /// as a compile-time constant. No default — every writer names one.
    /// `run_count` is how many CONSECUTIVE byte-identical
    /// occurrences this record stands for — `1` for an unfolded read. It rides
    /// the aux word's high 32 bits, which were reserved 0 through format 5 and
    /// already named for exactly this; a bag that folds stamps `trace_format` 6
    /// so an older reader refuses it rather than masking the count away and
    /// silently under-counting.
    pub fn read_outcome(
        step: u64,
        node_idx: u32,
        input_idx: u16,
        kind: ReadOutcomeKind,
        served_seq: Option<u32>,
        run: ReadRun,
        role: ReadSiteRole,
    ) -> Self {
        Self {
            step,
            fire_time_ns: served_seq.map(u64::from).unwrap_or(READ_OUTCOME_NO_FRAME),
            duration_ns: pack_read_outcome_aux_run(run.popped(), run.run_count()),
            node_idx,
            global_level: pack_read_outcome_meta(input_idx, kind.wire(), role),
            record_type: RECORD_TYPE_READ_OUTCOME,
            reserved: 0,
        }
    }

    /// Build the kind-6 OVERFLOW MARKER
    /// ([`READ_OUTCOME_TRUNCATED`]) — "the `dropped_in_window` records that
    /// would have followed this position were dropped at the staging rim".
    ///
    /// A MARKER, never a read: it carries no served sequence
    /// ([`READ_OUTCOME_NO_FRAME`]) and its aux word is the ordinary packing
    /// (low 32 = the count, high 32 = 0), so the re-offer-count reservation is untouched
    /// and a format-3 reader decodes every field it reads to the value the
    /// writer meant — it simply renders the kind as `unknown(6)`.
    ///
    /// `dropped_in_window` is a PER-WINDOW delta, not the stage's lifetime
    /// total: a marker carrying the lifetime total would re-report earlier
    /// windows' losses at every later drain, so an offline reader summing the
    /// markers would over-count. The lifetime total is
    /// `read_outcome::ReadOutcomeStage::dropped`.
    ///
    /// `role` is the OVERFLOWED STAGE's role, not a call-site
    /// role — DIAGNOSTIC ONLY. A consumer must NOT steer on it: a `Truncated`
    /// marker goes to BOTH injection queues and raises `TruncatedReadLog`
    /// under EVERY role value.
    pub fn read_outcome_truncated(
        step: u64,
        node_idx: u32,
        input_idx: u16,
        dropped_in_window: u32,
        role: ReadSiteRole,
    ) -> Self {
        Self {
            step,
            fire_time_ns: READ_OUTCOME_NO_FRAME,
            duration_ns: pack_read_outcome_aux(dropped_in_window),
            node_idx,
            global_level: pack_read_outcome_meta(input_idx, READ_OUTCOME_TRUNCATED, role),
            record_type: RECORD_TYPE_READ_OUTCOME,
            reserved: 0,
        }
    }

    /// Build the kind-6 PRODUCER ANNOTATION
    /// ([`READ_OUTCOME_PRODUCER`]) — it names the publisher whose frame the
    /// NEXT read record on the same `(node, input)` served.
    ///
    /// `served_seq` REPEATS the annotated read's served sequence (the
    /// redundant join key, so a reader that lost the ordering can still pair
    /// them); `token` is the 64-bit producer token and rides the FULL aux
    /// word.
    ///
    /// # Why the full 64 bits do not collide with the re-offer-count reservation
    ///
    /// The format pre-reserved the aux word's HIGH 32 bits as a re-offer COUNT —
    /// and it is explicitly a property of the SUCCESSOR **read** record's
    /// aux, i.e. of a record reporting what a drain served. This record
    /// reports no drain at all; it is an annotation whose whole payload is
    /// the token. So the reservation and the token are properties of
    /// DIFFERENT records and can never contend for the same bits (see
    /// [`unpack_read_outcome_popped`], which is never called on this kind).
    ///
    /// `role` is the CALL-SITE role of the read this annotation
    /// names — the pair is minted by ONE site, so both records carry the same
    /// role and a role-steering reader can never split them.
    pub fn read_outcome_producer(
        step: u64,
        node_idx: u32,
        input_idx: u16,
        served_seq: Option<u32>,
        token: u64,
        role: ReadSiteRole,
    ) -> Self {
        Self {
            step,
            fire_time_ns: served_seq.map(u64::from).unwrap_or(READ_OUTCOME_NO_FRAME),
            duration_ns: token,
            node_idx,
            global_level: pack_read_outcome_meta(input_idx, READ_OUTCOME_PRODUCER, role),
            record_type: RECORD_TYPE_READ_OUTCOME,
            reserved: 0,
        }
    }

    /// Decode from the 40-byte little-endian wire form. Total inverse of
    /// [`as_bytes`](Self::as_bytes).
    pub fn from_bytes(b: &[u8; TRACE_RECORD_SIZE as usize]) -> Self {
        Self {
            step: u64::from_le_bytes(b[0..8].try_into().unwrap()),
            fire_time_ns: u64::from_le_bytes(b[8..16].try_into().unwrap()),
            duration_ns: u64::from_le_bytes(b[16..24].try_into().unwrap()),
            node_idx: u32::from_le_bytes(b[24..28].try_into().unwrap()),
            global_level: u32::from_le_bytes(b[28..32].try_into().unwrap()),
            record_type: u32::from_le_bytes(b[32..36].try_into().unwrap()),
            reserved: u32::from_le_bytes(b[36..40].try_into().unwrap()),
        }
    }
}

/// Errors from the trace ring layer. Wraps [`ShmRingError`] and adds
/// manifest/record-type specific variants.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum TraceRingError {
    /// An error from the underlying generic ring (create/open/validation/overrun).
    #[error(transparent)]
    Ring(#[from] ShmRingError),
    /// The encoded node-id manifest exceeds the ring's manifest capacity.
    #[error("trace manifest too large: {needed} bytes exceeds MANIFEST_CAPACITY ({capacity}). Reduce the number/length of node ids.")]
    ManifestTooLarge {
        /// Encoded manifest size in bytes.
        needed: usize,
        /// The manifest capacity in bytes.
        capacity: usize,
    },
    /// A single node id exceeds the `u16` length prefix (65535 bytes).
    #[error("trace manifest node id at index {index} is {len} bytes, exceeding the u16 length limit (65535)")]
    NodeIdTooLong {
        /// The offending node's index.
        index: usize,
        /// Its UTF-8 byte length.
        len: usize,
    },
    /// `encode_manifest_with_inputs` was handed an input table whose
    /// length differs from the node table's — the per-node input lists index
    /// by node position, so a mismatch is a caller bug, never patched over.
    #[error("trace manifest input table has {inputs} entries but the node table has {nodes} — the two must be parallel (one input-name list per node)")]
    ManifestInputsMismatch {
        /// Node-id table length.
        nodes: usize,
        /// Input table length.
        inputs: usize,
    },
    /// The same caller bug, for the CAPACITY section. It used
    /// to be reported as [`Self::ManifestInputsMismatch`], so an operator
    /// reading "input table has N entries" went looking at a section that was
    /// fine — the sections are parallel to the node table INDEPENDENTLY, and a
    /// diagnostic that names the wrong one costs more than the variant it saved.
    #[error("trace manifest capacity table has {capacities} entries but the node table has {nodes} — the two must be parallel (one capacity-row list per node)")]
    ManifestCapacitiesMismatch {
        /// Node-id table length.
        nodes: usize,
        /// Capacity table length.
        capacities: usize,
    },
    /// The manifest content is malformed (truncated / invalid UTF-8 / an
    /// over-`u16`-limit input entry on the encode side).
    #[error("trace manifest decode failed: {reason}")]
    ManifestDecode {
        /// What was malformed.
        reason: String,
    },
    /// The opened ring's `record_size` is not [`TRACE_RECORD_SIZE`] — it is not a
    /// trace ring.
    #[error("trace ring record_size is {actual} but TraceRingRecord requires {expected}")]
    RecordSizeMismatch {
        /// The header's actual record size.
        actual: u32,
        /// [`TRACE_RECORD_SIZE`].
        expected: u32,
    },
}

/// Result alias for trace ring operations.
pub type TraceRingResult<T> = Result<T, TraceRingError>;

/// Encode a node-identity table to the ring manifest format:
/// `count: u32 (LE)`, then per node `len: u16 (LE)` + UTF-8 bytes. A node's array
/// index becomes its `node_idx`. Rejects a table that would exceed
/// [`MANIFEST_CAPACITY`] or a single id longer than a `u16`.
pub fn encode_manifest(node_ids: &[&str]) -> TraceRingResult<Vec<u8>> {
    let body: usize = node_ids.iter().map(|s| 2 + s.len()).sum();
    let mut out = Vec::with_capacity(4 + body);
    out.extend_from_slice(&(node_ids.len() as u32).to_le_bytes());
    for (i, id) in node_ids.iter().enumerate() {
        let bytes = id.as_bytes();
        if bytes.len() > u16::MAX as usize {
            return Err(TraceRingError::NodeIdTooLong {
                index: i,
                len: bytes.len(),
            });
        }
        out.extend_from_slice(&(bytes.len() as u16).to_le_bytes());
        out.extend_from_slice(bytes);
    }
    if out.len() > MANIFEST_CAPACITY {
        return Err(TraceRingError::ManifestTooLarge {
            needed: out.len(),
            capacity: MANIFEST_CAPACITY,
        });
    }
    Ok(out)
}

/// Decode a node-identity table produced by [`encode_manifest`]. The total inverse
/// (`decode_manifest(encode_manifest(x)) == x`).
///
/// Bytes AFTER the node table are deliberately IGNORED — that tolerance is what
/// let the additive per-node INPUT section
/// ([`encode_manifest_with_inputs`]) be appended without breaking older readers, and it
/// must be preserved for the next additive section.
pub fn decode_manifest(bytes: &[u8]) -> TraceRingResult<Vec<String>> {
    decode_node_table(bytes).map(|(names, _)| names)
}

/// The shared node-table decode: returns the names AND the cursor one past the
/// table, so [`decode_manifest_with_inputs`] can pick up the additive
/// input section from exactly where the table ended.
fn decode_node_table(bytes: &[u8]) -> TraceRingResult<(Vec<String>, usize)> {
    if bytes.len() < 4 {
        return Err(TraceRingError::ManifestDecode {
            reason: format!("manifest {} bytes < the 4-byte count header", bytes.len()),
        });
    }
    let count = u32::from_le_bytes(bytes[0..4].try_into().unwrap()) as usize;
    let mut cur = 4usize;
    // `count` comes from UNTRUSTED SHM bytes: reserving it verbatim would let a
    // hostile/corrupt header (e.g. count = u32::MAX) request a ~100 GB allocation
    // and abort via `handle_alloc_error` BEFORE the truncation guard in the loop
    // can fire. Cap the reserve by what the input could physically hold — each
    // entry needs at least its 2-byte length prefix — so a lying count still ends
    // in the loop's loud `ManifestDecode` truncation error, never an abort.
    let mut out = Vec::with_capacity(count.min(bytes.len().saturating_sub(4) / 2));
    for i in 0..count {
        if cur + 2 > bytes.len() {
            return Err(TraceRingError::ManifestDecode {
                reason: format!("truncated at node {i}: missing 2-byte length prefix"),
            });
        }
        let len = u16::from_le_bytes(bytes[cur..cur + 2].try_into().unwrap()) as usize;
        cur += 2;
        if cur + len > bytes.len() {
            return Err(TraceRingError::ManifestDecode {
                reason: format!(
                    "truncated at node {i}: declared {len} bytes but only {} remain",
                    bytes.len() - cur
                ),
            });
        }
        let s = std::str::from_utf8(&bytes[cur..cur + len]).map_err(|e| {
            TraceRingError::ManifestDecode {
                reason: format!("node {i} is not valid UTF-8: {e}"),
            }
        })?;
        out.push(s.to_string());
        cur += len;
    }
    Ok((out, cur))
}

/// Encode a node-identity table PLUS each node's ordered input-name
/// list — the offline resolver for a kind-6 record's `input_idx`.
///
/// Layout: the exact [`encode_manifest`] node table, then (ADDITIVELY) per node
/// in table order: `input_count: u16 (LE)`, then per input `len: u16 (LE)` +
/// UTF-8 bytes. An input's position in its node's list IS the `input_idx` a
/// [`RECORD_TYPE_READ_OUTCOME`] record carries. Back-compat by construction:
/// [`decode_manifest`] stops after the node table (trailing bytes ignored), and
/// [`decode_manifest_with_inputs`] treats an ABSENT section (a node-only
/// manifest) as empty lists.
///
/// `node_inputs` must be parallel to `node_ids` (one list per node, empty
/// allowed) — a length mismatch is refused loudly.
pub fn encode_manifest_with_inputs(
    node_ids: &[&str],
    node_inputs: &[&[&str]],
) -> TraceRingResult<Vec<u8>> {
    if node_ids.len() != node_inputs.len() {
        return Err(TraceRingError::ManifestInputsMismatch {
            nodes: node_ids.len(),
            inputs: node_inputs.len(),
        });
    }
    let mut out = encode_manifest(node_ids)?;
    for (i, inputs) in node_inputs.iter().enumerate() {
        if inputs.len() > u16::MAX as usize {
            return Err(TraceRingError::ManifestDecode {
                reason: format!(
                    "node {i} declares {} inputs, exceeding the u16 count limit",
                    inputs.len()
                ),
            });
        }
        out.extend_from_slice(&(inputs.len() as u16).to_le_bytes());
        for (j, name) in inputs.iter().enumerate() {
            let bytes = name.as_bytes();
            if bytes.len() > u16::MAX as usize {
                return Err(TraceRingError::ManifestDecode {
                    reason: format!(
                        "input name at node {i} input {j} is {} bytes, exceeding the u16 \
                         length limit (65535)",
                        bytes.len()
                    ),
                });
            }
            out.extend_from_slice(&(bytes.len() as u16).to_le_bytes());
            out.extend_from_slice(bytes);
        }
    }
    if out.len() > MANIFEST_CAPACITY {
        return Err(TraceRingError::ManifestTooLarge {
            needed: out.len(),
            capacity: MANIFEST_CAPACITY,
        });
    }
    Ok(out)
}

/// Decode a manifest's node table AND its additive per-node input
/// section. A manifest WITHOUT the section (every node-only ring) decodes to
/// one EMPTY input list per node — the serde-`#[serde(default)]` analogue for
/// this binary format — so a kind-6-less bag needs nothing. A manifest that
/// STARTS the section but truncates mid-way is refused loudly (it claimed the
/// section). Bytes AFTER a complete input section are ignored (room for the
/// next additive section, mirroring [`decode_manifest`]'s tolerance).
pub fn decode_manifest_with_inputs(
    bytes: &[u8],
) -> TraceRingResult<(Vec<String>, Vec<Vec<String>>)> {
    decode_inputs_section(bytes).map(|(names, inputs, _)| (names, inputs))
}

/// The shared node-table + input-section decode: returns the names, the
/// per-node input lists, AND the cursor one past the input section, so
/// [`decode_manifest_with_inputs_and_publishers`] can pick up the additive
/// publisher section from exactly where the input section ended (the
/// [`decode_node_table`] pattern, one section down).
#[allow(clippy::type_complexity)]
fn decode_inputs_section(bytes: &[u8]) -> TraceRingResult<(Vec<String>, Vec<Vec<String>>, usize)> {
    let (names, mut cur) = decode_node_table(bytes)?;
    if cur == bytes.len() {
        let empty = vec![Vec::new(); names.len()];
        return Ok((names, empty, cur));
    }
    let mut inputs: Vec<Vec<String>> = Vec::with_capacity(names.len());
    for i in 0..names.len() {
        if cur + 2 > bytes.len() {
            return Err(TraceRingError::ManifestDecode {
                reason: format!("input section truncated at node {i}: missing 2-byte count"),
            });
        }
        let count = u16::from_le_bytes(bytes[cur..cur + 2].try_into().unwrap()) as usize;
        cur += 2;
        let mut list = Vec::with_capacity(count.min(bytes.len().saturating_sub(cur) / 2));
        for j in 0..count {
            if cur + 2 > bytes.len() {
                return Err(TraceRingError::ManifestDecode {
                    reason: format!(
                        "input section truncated at node {i} input {j}: missing length prefix"
                    ),
                });
            }
            let len = u16::from_le_bytes(bytes[cur..cur + 2].try_into().unwrap()) as usize;
            cur += 2;
            if cur + len > bytes.len() {
                return Err(TraceRingError::ManifestDecode {
                    reason: format!(
                        "input section truncated at node {i} input {j}: declared {len} bytes \
                         but only {} remain",
                        bytes.len() - cur
                    ),
                });
            }
            let s = std::str::from_utf8(&bytes[cur..cur + len]).map_err(|e| {
                TraceRingError::ManifestDecode {
                    reason: format!("node {i} input {j} is not valid UTF-8: {e}"),
                }
            })?;
            list.push(s.to_string());
            cur += len;
        }
        inputs.push(list);
    }
    Ok((names, inputs, cur))
}

/// Encode a node table + its per-node INPUT section
/// ([`encode_manifest_with_inputs`]) + the additive per-node PUBLISHER
/// section — the offline resolver for a [`READ_OUTCOME_PRODUCER`]
/// annotation's token.
///
/// Layout: sections 1 and 2 EXACTLY as [`encode_manifest_with_inputs`] writes
/// them, then (ADDITIVELY) section 3, per node in table order:
/// `output_count: u16 (LE)`, then per output `id: u128 (LE, 16 bytes)` +
/// `len: u16 (LE)` + UTF-8 output name. The id is that output's publisher's
/// `iceoryx2::identifiers::UniquePublisherId::value()` — run-random, which is
/// exactly why it is TRANSLATED through this table rather than compared
/// across runs.
///
/// Back-compat by construction, at BOTH earlier readers:
/// [`decode_manifest`] stops after section 1 and
/// [`decode_manifest_with_inputs`] stops after a COMPLETE section 2 (its doc
/// reserves the next additive section — this is it), so an older reader is
/// byte-unaffected. `node_publishers` must be parallel to `node_ids` (one
/// list per node, empty allowed).
pub fn encode_manifest_with_inputs_and_publishers(
    node_ids: &[&str],
    node_inputs: &[&[&str]],
    node_publishers: &[&[(&str, u128)]],
) -> TraceRingResult<Vec<u8>> {
    if node_ids.len() != node_publishers.len() {
        return Err(TraceRingError::ManifestInputsMismatch {
            nodes: node_ids.len(),
            inputs: node_publishers.len(),
        });
    }
    let mut out = encode_manifest_with_inputs(node_ids, node_inputs)?;
    for (i, outputs) in node_publishers.iter().enumerate() {
        if outputs.len() > u16::MAX as usize {
            return Err(TraceRingError::ManifestDecode {
                reason: format!(
                    "node {i} declares {} outputs, exceeding the u16 count limit",
                    outputs.len()
                ),
            });
        }
        out.extend_from_slice(&(outputs.len() as u16).to_le_bytes());
        for (j, (name, id)) in outputs.iter().enumerate() {
            let bytes = name.as_bytes();
            if bytes.len() > u16::MAX as usize {
                return Err(TraceRingError::ManifestDecode {
                    reason: format!(
                        "output name at node {i} output {j} is {} bytes, exceeding the u16 \
                         length limit (65535)",
                        bytes.len()
                    ),
                });
            }
            out.extend_from_slice(&id.to_le_bytes());
            out.extend_from_slice(&(bytes.len() as u16).to_le_bytes());
            out.extend_from_slice(bytes);
        }
    }
    if out.len() > MANIFEST_CAPACITY {
        return Err(TraceRingError::ManifestTooLarge {
            needed: out.len(),
            capacity: MANIFEST_CAPACITY,
        });
    }
    Ok(out)
}

/// Decode a manifest's node table, its additive per-node INPUT
/// section, AND its additive per-node PUBLISHER section.
///
/// A manifest with no publisher section (every older ring, and every ring
/// whose graph declares no outputs) decodes to one EMPTY publisher list per
/// node — the same `#[serde(default)]` analogue [`decode_manifest_with_inputs`]
/// applies to the input section. A manifest that STARTS the section but
/// truncates mid-way is refused loudly (it claimed the section). Bytes AFTER
/// a complete publisher section are ignored (room for the next additive
/// section).
#[allow(clippy::type_complexity)]
pub fn decode_manifest_with_inputs_and_publishers(
    bytes: &[u8],
) -> TraceRingResult<(Vec<String>, Vec<Vec<String>>, Vec<Vec<(String, u128)>>)> {
    let (names, inputs, publishers, _cur) = decode_publishers_section(bytes)?;
    Ok((names, inputs, publishers))
}

/// The publisher section's decode, WITH the cursor, so the next
/// additive section can chain off it exactly as this one chains off
/// [`decode_inputs_section`]. Extracted verbatim from
/// [`decode_manifest_with_inputs_and_publishers`], which is now a thin delegate
/// — the byte grammar is unchanged.
#[allow(clippy::type_complexity)]
fn decode_publishers_section(
    bytes: &[u8],
) -> TraceRingResult<(
    Vec<String>,
    Vec<Vec<String>>,
    Vec<Vec<(String, u128)>>,
    usize,
)> {
    let (names, inputs, mut cur) = decode_inputs_section(bytes)?;
    if cur == bytes.len() {
        let empty = vec![Vec::new(); names.len()];
        return Ok((names, inputs, empty, cur));
    }
    let mut publishers: Vec<Vec<(String, u128)>> = Vec::with_capacity(names.len());
    for i in 0..names.len() {
        if cur + 2 > bytes.len() {
            return Err(TraceRingError::ManifestDecode {
                reason: format!("publisher section truncated at node {i}: missing 2-byte count"),
            });
        }
        let count = u16::from_le_bytes(bytes[cur..cur + 2].try_into().unwrap()) as usize;
        cur += 2;
        // Each entry needs at least 16 (id) + 2 (len prefix) bytes, so cap
        // the reserve by what the input could physically hold (the
        // `decode_node_table` hostile-count discipline).
        let mut list = Vec::with_capacity(count.min(bytes.len().saturating_sub(cur) / 18));
        for j in 0..count {
            if cur + 18 > bytes.len() {
                return Err(TraceRingError::ManifestDecode {
                    reason: format!(
                        "publisher section truncated at node {i} output {j}: missing the \
                         16-byte id + length prefix"
                    ),
                });
            }
            let id = u128::from_le_bytes(bytes[cur..cur + 16].try_into().unwrap());
            cur += 16;
            let len = u16::from_le_bytes(bytes[cur..cur + 2].try_into().unwrap()) as usize;
            cur += 2;
            if cur + len > bytes.len() {
                return Err(TraceRingError::ManifestDecode {
                    reason: format!(
                        "publisher section truncated at node {i} output {j}: declared {len} \
                         bytes but only {} remain",
                        bytes.len() - cur
                    ),
                });
            }
            let s = std::str::from_utf8(&bytes[cur..cur + len]).map_err(|e| {
                TraceRingError::ManifestDecode {
                    reason: format!("node {i} output {j} is not valid UTF-8: {e}"),
                }
            })?;
            list.push((s.to_string(), id));
            cur += len;
        }
        publishers.push(list);
    }
    Ok((names, inputs, publishers, cur))
}

/// The capacity section's row-KEY rule, in ONE place.
///
/// The key is `(input_idx, role)` — an input under the Separate or legacy-`Sync`
/// discipline contributes two stages that SHARE an index — and every reader
/// builds a map on it, so a duplicate silently keeps whichever row landed last:
/// a rim the recording may never have used. Both ends refuse one, and both ends
/// refuse it through this function. Separate
/// implementations (a `HashSet` on the encode side, an O(n^2) scan on the
/// decode side) would be how one end's rule drifts from the other's.
///
/// `Err` is the whole diagnostic, ready to return.
fn reject_duplicate_capacity_keys(
    node_idx: usize,
    rows: impl IntoIterator<Item = (u16, u8)>,
) -> TraceRingResult<()> {
    let mut seen = std::collections::HashSet::new();
    for (input_idx, role) in rows {
        if !seen.insert((input_idx, role)) {
            return Err(TraceRingError::ManifestDecode {
                reason: format!(
                    "capacity section at node {node_idx} declares (input_idx {input_idx}, \
                     role {role}) twice; the pair is the row KEY, so the table is unreadable \
                     rather than last-one-wins"
                ),
            });
        }
    }
    Ok(())
}

/// SECTION 4 — the per-node READ-LOG STAGE CAPACITY table, the
/// fourth additive section, written after a complete publisher section.
///
/// Each row is `(input_idx, role, capacity)` and is keyed on the PAIR, never on
/// position: an input under the Separate or legacy-`Sync` discipline carries TWO
/// stages that deliberately share an `input_idx`, so the vector's length is not
/// the input count and the tail is not in input order. Row layout is 7 bytes —
/// `u16` index, `u8` role, `u32` capacity —
/// all little-endian, matching every other section.
///
/// The capacity is what the RECORDING armed its stage at, and replay ADOPTS it
/// so the two sides truncate identically. That is why it travels at
/// all: the capacity is derived per edge from the graph AND from
/// the loaded node's freeze capability, so it is no longer a constant either
/// side can assume.
pub fn encode_manifest_with_inputs_publishers_and_capacities(
    node_ids: &[&str],
    node_inputs: &[&[&str]],
    node_publishers: &[&[(&str, u128)]],
    node_capacities: &[&[(u16, u8, u32)]],
) -> TraceRingResult<Vec<u8>> {
    if node_ids.len() != node_capacities.len() {
        return Err(TraceRingError::ManifestCapacitiesMismatch {
            nodes: node_ids.len(),
            capacities: node_capacities.len(),
        });
    }
    let mut out =
        encode_manifest_with_inputs_and_publishers(node_ids, node_inputs, node_publishers)?;
    for (i, rows) in node_capacities.iter().enumerate() {
        if rows.len() > u16::MAX as usize {
            return Err(TraceRingError::ManifestDecode {
                reason: format!(
                    "node {i} declares {} capacity rows, exceeding the u16 count limit",
                    rows.len()
                ),
            });
        }
        // The row KEY is `(input_idx, role)` and the
        // encoder cannot emit a duplicate — the SAME rule the decoder applies,
        // through the same function.
        reject_duplicate_capacity_keys(i, rows.iter().map(|(idx, role, _)| (*idx, *role)))?;
        out.extend_from_slice(&(rows.len() as u16).to_le_bytes());
        for (input_idx, role, capacity) in rows.iter() {
            out.extend_from_slice(&input_idx.to_le_bytes());
            out.push(*role);
            out.extend_from_slice(&capacity.to_le_bytes());
        }
    }
    if out.len() > MANIFEST_CAPACITY {
        return Err(TraceRingError::ManifestTooLarge {
            needed: out.len(),
            capacity: MANIFEST_CAPACITY,
        });
    }
    Ok(out)
}

/// Decode the node table plus the input, publisher AND capacity
/// sections.
///
/// A manifest with no capacity section decodes to one
/// EMPTY row list per node — the same `#[serde(default)]` analogue the input and
/// publisher sections take. A manifest that STARTS the section but truncates
/// mid-way is refused loudly (it claimed the section). Bytes AFTER a complete
/// capacity section are ignored, which is what keeps room for section 5.
#[allow(clippy::type_complexity)]
pub fn decode_manifest_with_inputs_publishers_and_capacities(
    bytes: &[u8],
) -> TraceRingResult<(
    Vec<String>,
    Vec<Vec<String>>,
    Vec<Vec<(String, u128)>>,
    Vec<Vec<(u16, u8, u32)>>,
)> {
    let (names, inputs, publishers, mut cur) = decode_publishers_section(bytes)?;
    if cur == bytes.len() {
        let empty = vec![Vec::new(); names.len()];
        return Ok((names, inputs, publishers, empty));
    }
    // 7 bytes per row: u16 index + u8 role + u32 capacity.
    const ROW_BYTES: usize = 7;
    let mut caps: Vec<Vec<(u16, u8, u32)>> = Vec::with_capacity(names.len());
    for i in 0..names.len() {
        if cur + 2 > bytes.len() {
            return Err(TraceRingError::ManifestDecode {
                reason: format!("capacity section truncated at node {i}: missing 2-byte count"),
            });
        }
        let count = u16::from_le_bytes(bytes[cur..cur + 2].try_into().unwrap()) as usize;
        cur += 2;
        // The `decode_node_table` hostile-count discipline: never reserve more
        // than the remaining bytes could physically hold.
        let mut rows = Vec::with_capacity(count.min(bytes.len().saturating_sub(cur) / ROW_BYTES));
        for j in 0..count {
            if cur + ROW_BYTES > bytes.len() {
                return Err(TraceRingError::ManifestDecode {
                    reason: format!(
                        "capacity section truncated at node {i} row {j}: needs {ROW_BYTES} \
                         bytes but only {} remain",
                        bytes.len() - cur
                    ),
                });
            }
            let input_idx = u16::from_le_bytes(bytes[cur..cur + 2].try_into().unwrap());
            let role = bytes[cur + 2];
            let capacity = u32::from_le_bytes(bytes[cur + 3..cur + 7].try_into().unwrap());
            // The reader half — the SAME rule as the encoder's,
            // through the same function. Checked against the rows read
            // SO FAR plus this one, which is what the encoder's set-insert does.
            reject_duplicate_capacity_keys(
                i,
                rows.iter()
                    .map(|(idx, r, _)| (*idx, *r))
                    .chain(std::iter::once((input_idx, role))),
            )?;
            rows.push((input_idx, role, capacity));
            cur += ROW_BYTES;
        }
        caps.push(rows);
    }
    Ok((names, inputs, publishers, caps))
}

/// Default trace-ring data-region budget (64 MiB) — deliberately OVERSIZED so an
/// overrun means the recorder has been dead/stalled for a very long time.
pub const DEFAULT_TRACE_RING_BYTES: usize = 64 * 1024 * 1024;

/// The default capacity in records: [`DEFAULT_TRACE_RING_BYTES`] / [`TRACE_RECORD_SIZE`],
/// rounded DOWN to a power of two (the ring capacity must be a power of two). ≈1M
/// records (`2^20`).
pub fn default_capacity_records() -> u32 {
    let raw = (DEFAULT_TRACE_RING_BYTES / TRACE_RECORD_SIZE as usize) as u64;
    prev_power_of_two(raw) as u32
}

/// The APPARENT bytes ONE trace ring of `capacity_records` reserves
/// — header (control + 64 KiB manifest) plus `capacity_records × 40`.
///
/// At [`default_capacity_records`] that is `65_600 + 2^20 × 40` = 42,008,640 B
/// = 40.06 MiB per rank, which is the figure the `/dev/shm` free-space gate
/// compares against and the figure the always-on cost is quoted in.
///
/// It delegates to [`crate::shm_ring::apparent_segment_bytes`] rather than
/// restating the arithmetic, so a header-layout change moves both.
#[must_use]
pub fn apparent_ring_bytes(capacity_records: u32) -> u64 {
    crate::shm_ring::apparent_segment_bytes(TRACE_RECORD_SIZE, capacity_records)
}

// ===========================================================================
// Owner / producer / consumer wrappers
// ===========================================================================

/// The trace-ring owner: creates the SHM segment sized for [`TraceRingRecord`]s,
/// writes the node-id manifest, and mints the single [`TraceRingProducer`].
#[derive(Debug)]
#[must_use = "the owner shm_unlinks the ring name on drop — bind it to a named local"]
pub struct TraceRingOwner {
    inner: ShmRingOwner,
    node_ids: Vec<String>,
}

impl TraceRingOwner {
    /// Create a trace ring for `tag` holding `capacity_records` [`TraceRingRecord`]s,
    /// tagged with the producer `rank`, with `node_ids` as the manifest table.
    ///
    /// The manifest carries NO per-node input section (the base form —
    /// the right call for input-less rings like the supervisor
    /// departure ring). A recording that emits [`RECORD_TYPE_READ_OUTCOME`]
    /// records should use [`Self::create_with_inputs`] so `input_idx` resolves
    /// to a name offline.
    pub fn create(
        tag: &str,
        capacity_records: u32,
        rank: u32,
        node_ids: &[&str],
    ) -> TraceRingResult<Self> {
        let manifest = encode_manifest(node_ids)?;
        Self::create_inner(tag, capacity_records, rank, node_ids, manifest)
    }

    /// [`Self::create`] with the additive per-node INPUT-name section
    /// ([`encode_manifest_with_inputs`]) — `node_inputs` parallel to `node_ids`
    /// (one ordered input-name list per node; the list position IS the
    /// `input_idx` a kind-6 record carries).
    pub fn create_with_inputs(
        tag: &str,
        capacity_records: u32,
        rank: u32,
        node_ids: &[&str],
        node_inputs: &[&[&str]],
    ) -> TraceRingResult<Self> {
        let manifest = encode_manifest_with_inputs(node_ids, node_inputs)?;
        Self::create_inner(tag, capacity_records, rank, node_ids, manifest)
    }

    /// The public entry points. Each starts the ladder with an
    /// EMPTY dropped-set; the `_marked` bodies accumulate on the way down so the
    /// ring is created already marked (see `create_with_policy_and_degraded`).
    pub fn create_with_inputs_or_degrade(
        tag: &str,
        capacity_records: u32,
        rank: u32,
        node_ids: &[&str],
        node_inputs: &[&[&str]],
    ) -> TraceRingResult<Self> {
        Self::create_with_inputs_or_degrade_marked(
            tag,
            capacity_records,
            rank,
            node_ids,
            node_inputs,
            0,
        )
    }

    pub fn create_with_inputs_and_publishers_or_degrade(
        tag: &str,
        capacity_records: u32,
        rank: u32,
        node_ids: &[&str],
        node_inputs: &[&[&str]],
        node_publishers: &[&[(&str, u128)]],
    ) -> TraceRingResult<Self> {
        Self::create_with_inputs_and_publishers_or_degrade_marked(
            tag,
            capacity_records,
            rank,
            node_ids,
            node_inputs,
            node_publishers,
            0,
        )
    }

    pub fn create_with_inputs_publishers_and_capacities_or_degrade(
        tag: &str,
        capacity_records: u32,
        rank: u32,
        node_ids: &[&str],
        node_inputs: &[&[&str]],
        node_publishers: &[&[(&str, u128)]],
        node_capacities: &[&[(u16, u8, u32)]],
    ) -> TraceRingResult<Self> {
        Self::create_with_inputs_publishers_and_capacities_or_degrade_marked(
            tag,
            capacity_records,
            rank,
            ManifestSections {
                node_ids,
                node_inputs,
                node_publishers,
                node_capacities,
            },
            0,
        )
    }

    /// The compatibility entry point: [`Self::create_with_inputs`]
    /// with the never-block-degrade-loudly posture. The additive input-name
    /// section can push a manifest whose NODE table alone fit comfortably
    /// past [`MANIFEST_CAPACITY`] — under the strict form that FAILS recording
    /// at startup ([`TraceRingError::ManifestTooLarge`]) for a graph that
    /// records fine with a node-only manifest. This entry point tries the inputs-bearing
    /// manifest first; if (and only if) that encoding exceeds the budget, it
    /// falls back to the node-only manifest ([`Self::create`]'s form) with ONE
    /// loud `warn!` naming what was dropped and the consequence. The degrade
    /// is sound end-to-end through EXISTING machinery: a bag whose manifest
    /// carries no input section decodes to empty per-node input lists
    /// ([`decode_manifest_with_inputs`]), which is exactly the shape the
    /// replay read-log verifier's Disabled arm already stands down loudly for
    /// — the recording itself is complete; only the offline input-name
    /// resolution is lost. A node-only manifest that STILL exceeds the budget
    /// propagates the error (there is nothing left to
    /// drop), and every non-budget error (e.g. the parallel-tables mismatch)
    /// propagates untouched.
    fn create_with_inputs_or_degrade_marked(
        tag: &str,
        capacity_records: u32,
        rank: u32,
        node_ids: &[&str],
        node_inputs: &[&[&str]],
        dropped: u32,
    ) -> TraceRingResult<Self> {
        match encode_manifest_with_inputs(node_ids, node_inputs) {
            // The SUCCESS path carries `dropped` too: this rung's own encoding fit,
            // but a rung ABOVE may already have dropped its section and passed its
            // bit down. Calling the unmarked `create_inner` here threw those bits
            // away, so a ladder that fell exactly one step reported nothing.
            Ok(manifest) => Self::create_inner_degraded(
                tag,
                capacity_records,
                rank,
                node_ids,
                manifest,
                dropped,
            ),
            Err(TraceRingError::ManifestTooLarge { needed, capacity }) => {
                // The node-only form must itself fit — `encode_manifest`
                // re-checks the budget, and a failure HERE propagates BEFORE
                // the warn (no "recording proceeds" claim for a run that then
                // fails anyway).
                let manifest = encode_manifest(node_ids)?;
                tracing::warn!(
                    ring = %tag,
                    needed_bytes = needed,
                    capacity_bytes = capacity,
                    "read-log input names omitted from the trace-ring manifest: the \
                     inputs-bearing encoding exceeds the manifest budget (the `needed_bytes` \
                     and `capacity_bytes` fields carry the two sizes); recording proceeds \
                     with the node-only manifest. `bagd` stamps the `read_log_capacity` \
                     SENTINEL for this rank on the strength of the ring's DEGRADED marker \
                     (the input section is gone, so the marker is the only thing left that \
                     knows this rank HAD stages), and sentinel-present + table-absent is \
                     what the replay reads as UNREADABLE — so the read-log verifier stands \
                     down LOUDLY for this bag rather than deriving its own rims in silence"
                );
                // The marker, written AT CREATE. Without it this
                // rung is INDISTINGUISHABLE from a genuinely stageless rank — both
                // decode to empty inputs AND empty capacities — and the recorder's
                // discriminator reads it as "declares nothing", which arms every
                // stage at the replay's derived rim with no warn. The marker keeps it
                // `Unreadable` and loud, which is what the warn above
                // promises.
                //
                // `dropped` arrives from the rungs ABOVE: each adds its own bit on
                // the way down, so the ring is created once with the full set rather
                // than marked afterwards, when a consumer could already have opened
                // it and read "nothing dropped".
                Self::create_inner_degraded(
                    tag,
                    capacity_records,
                    rank,
                    node_ids,
                    manifest,
                    dropped
                        | DegradedSections::INPUTS
                        | DegradedSections::PUBLISHERS
                        | DegradedSections::CAPACITIES,
                )
            }
            Err(e) => Err(e),
        }
    }

    /// [`Self::create_with_inputs_and_publishers_or_degrade`] plus
    /// the additive per-node CAPACITY section (section 4) — the per-stage
    /// staging rims the replay ADOPTS so both sides truncate identically.
    ///
    /// The DEGRADE LADDER gains its fourth rung and keeps its posture: try the
    /// full 4-section manifest; if (and ONLY if) it exceeds
    /// [`MANIFEST_CAPACITY`], fall back to the publishers-bearing form with one
    /// loud `warn!` naming what was dropped and the consequence; that form runs
    /// its own degrade, and so on down to the node-only manifest.
    ///
    /// # How the fallback stays sound, stated as the mechanism it now is
    ///
    /// A manifest with no capacity section decodes to empty row lists, and that
    /// is NOT on its own enough to make the replay refuse: an empty row list is
    /// also what a STAGELESS rank has, and those two must be answered
    /// differently. What carries the refusal is the RECORDER: `bagd` writes the
    /// `read_log_capacity` SENTINEL for any rank that declares staging —
    /// including one whose rows this rung dropped, told apart by its INPUT
    /// section, which survives here — and sentinel-present + table-absent is the
    /// pair the replay reads as `Unreadable` and stands down loudly on.
    ///
    /// The refusal does NOT come from "existing machinery" reading the empty
    /// rows: that holds only while the scalar is unconditional, and once it
    /// rides the same condition as the table a degraded bag declares NOTHING
    /// and every stage arms at the replay's own derived rim in silence. The
    /// sentinel is written for every staging rank for exactly that reason.
    fn create_with_inputs_publishers_and_capacities_or_degrade_marked(
        tag: &str,
        capacity_records: u32,
        rank: u32,
        sections: ManifestSections<'_>,
        dropped: u32,
    ) -> TraceRingResult<Self> {
        let ManifestSections {
            node_ids,
            node_inputs,
            node_publishers,
            node_capacities,
        } = sections;
        match encode_manifest_with_inputs_publishers_and_capacities(
            node_ids,
            node_inputs,
            node_publishers,
            node_capacities,
        ) {
            // The SUCCESS path carries `dropped` too: this rung's own encoding fit,
            // but a rung ABOVE may already have dropped its section and passed its
            // bit down. Calling the unmarked `create_inner` here threw those bits
            // away, so a ladder that fell exactly one step reported nothing.
            Ok(manifest) => Self::create_inner_degraded(
                tag,
                capacity_records,
                rank,
                node_ids,
                manifest,
                dropped,
            ),
            Err(TraceRingError::ManifestTooLarge { needed, capacity }) => {
                tracing::warn!(
                    ring = %tag,
                    needed_bytes = needed,
                    capacity_bytes = capacity,
                    "read-log STAGE CAPACITIES omitted from the trace-ring manifest: the \
                     capacities-bearing encoding exceeds the manifest budget (see \
                     `needed_bytes`/`capacity_bytes`); recording proceeds without the \
                     capacity table. `bagd` still stamps the `read_log_capacity` SENTINEL \
                     for this rank (it has INPUTS, so it declares staging even with its rows \
                     gone), and sentinel-present + table-absent is what the replay reads as \
                     UNREADABLE — so the read-log verifier stands down LOUDLY for this bag \
                     rather than adopting a staging rim the bag never declared. \
                     This retries the PUBLISHERS-bearing encoding, which is smaller but not \
                     guaranteed to fit — if it does not, the next warn says so."
                );
                // This rung's bit travels DOWN, so whichever rung
                // finally creates the ring creates it already marked.
                Self::create_with_inputs_and_publishers_or_degrade_marked(
                    tag,
                    capacity_records,
                    rank,
                    node_ids,
                    node_inputs,
                    node_publishers,
                    dropped | DegradedSections::CAPACITIES,
                )
            }
            Err(e) => Err(e),
        }
    }

    /// [`Self::create_with_inputs_or_degrade`] plus the additive
    /// per-node PUBLISHER section
    /// ([`encode_manifest_with_inputs_and_publishers`]) — the offline resolver
    /// for a [`READ_OUTCOME_PRODUCER`] annotation's token.
    ///
    /// The DEGRADE LADDER is the same posture, one rung longer, and it is the
    /// R10 answer for the third section: try the full 3-section manifest; if
    /// (and ONLY if) it exceeds [`MANIFEST_CAPACITY`], fall back to the
    /// inputs-bearing form with one loud `warn!` naming exactly what was
    /// dropped and the consequence; that form runs its OWN degrade to the
    /// node-only manifest. Both fallbacks are sound end-to-end through
    /// EXISTING machinery: a manifest with no publisher section decodes to
    /// empty publisher lists, which is precisely the shape the replay-side
    /// token resolver already stands the affected EDGE down for (`foreign`,
    /// loudly) — the recording itself is complete; only the offline
    /// producer-name resolution is lost. Every non-budget error propagates
    /// untouched.
    fn create_with_inputs_and_publishers_or_degrade_marked(
        tag: &str,
        capacity_records: u32,
        rank: u32,
        node_ids: &[&str],
        node_inputs: &[&[&str]],
        node_publishers: &[&[(&str, u128)]],
        dropped: u32,
    ) -> TraceRingResult<Self> {
        match encode_manifest_with_inputs_and_publishers(node_ids, node_inputs, node_publishers) {
            // The SUCCESS path carries `dropped` too: this rung's own encoding fit,
            // but a rung ABOVE may already have dropped its section and passed its
            // bit down. Calling the unmarked `create_inner` here threw those bits
            // away, so a ladder that fell exactly one step reported nothing.
            Ok(manifest) => Self::create_inner_degraded(
                tag,
                capacity_records,
                rank,
                node_ids,
                manifest,
                dropped,
            ),
            Err(TraceRingError::ManifestTooLarge { needed, capacity }) => {
                tracing::warn!(
                    ring = %tag,
                    needed_bytes = needed,
                    capacity_bytes = capacity,
                    "read-log PRODUCER names omitted from the trace-ring manifest: the \
                     publishers-bearing encoding exceeds the manifest budget (see \
                     `needed_bytes`/`capacity_bytes`); recording proceeds without the \
                     publisher table, and the \
                     replay read-log verifier will stand every multi-publisher edge down \
                     loudly (its producer tokens resolve to `foreign`). This retries the \
                     INPUTS-only encoding, which is smaller but not guaranteed to fit — if it \
                     does not, the next warn says so and the whole read log degrades to \
                     `input_idx` placeholders."
                );
                Self::create_with_inputs_or_degrade_marked(
                    tag,
                    capacity_records,
                    rank,
                    node_ids,
                    node_inputs,
                    dropped | DegradedSections::PUBLISHERS,
                )
            }
            Err(e) => Err(e),
        }
    }

    /// The shared body of [`Self::create`] / [`Self::create_with_inputs`]: one
    /// SHM-create + fork-exclusion path, so the two manifest forms cannot
    /// drift in anything but the encoded bytes.
    fn create_inner(
        tag: &str,
        capacity_records: u32,
        rank: u32,
        node_ids: &[&str],
        manifest: Vec<u8>,
    ) -> TraceRingResult<Self> {
        Self::create_inner_degraded(tag, capacity_records, rank, node_ids, manifest, 0)
    }

    /// [`Self::create_inner`] carrying the degrade ladder's accumulated
    /// [`DegradedSections`] bits, so the ring is created ALREADY MARKED — before
    /// `MAGIC` publishes the header to any consumer.
    fn create_inner_degraded(
        tag: &str,
        capacity_records: u32,
        rank: u32,
        node_ids: &[&str],
        manifest: Vec<u8>,
        degraded: u32,
    ) -> TraceRingResult<Self> {
        let inner = ShmRingOwner::create_degraded(
            tag,
            TRACE_RECORD_SIZE,
            capacity_records,
            rank,
            &manifest,
            degraded,
        )?;
        // Fork exclusion, applied AT BIRTH: a capture `fork` child
        // inherits a fully ARMED producer over this ring, and a stray write there is
        // a corruption class pointed straight at the LIVE bag. Never fatal — a
        // failure degrades to the structural argument and logs (see `exclude_at_birth`).
        let (ptr, len) = inner.mapping();
        // SAFETY: the region `inner` just mapped and exclusively owns.
        unsafe {
            crate::state_carrier::exclude_at_birth(
                ptr,
                len,
                crate::state_carrier::ForkExcludedMapping::TraceRing,
            );
        }
        Ok(Self {
            inner,
            node_ids: node_ids.iter().map(|s| s.to_string()).collect(),
        })
    }

    /// Mint the single [`TraceRingProducer`] (`None` if already minted).
    pub fn producer(&mut self) -> Option<TraceRingProducer> {
        self.inner
            .producer()
            .map(|inner| TraceRingProducer { inner })
    }

    /// The owner's mapped region, as `(base, len)`.
    ///
    /// The carrier excludes this ring from `fork`
    /// inheritance at BIRTH (see [`TraceRingOwner::create`]), and a test needs the same
    /// bounds to prove it. Delegates to the generic ring — ONE source for the region.
    pub fn mapping(&self) -> (*mut std::ffi::c_void, usize) {
        self.inner.mapping()
    }

    /// The POSIX SHM object name — hand this to the recorder process.
    pub fn name(&self) -> &str {
        self.inner.name()
    }

    /// The producer rank.
    pub fn rank(&self) -> u32 {
        self.inner.rank()
    }

    /// The create-generation.
    pub fn generation(&self) -> u64 {
        self.inner.generation()
    }

    /// The capacity in records.
    pub fn capacity(&self) -> u32 {
        self.inner.capacity()
    }

    /// The node-id manifest table.
    pub fn node_ids(&self) -> &[String] {
        &self.node_ids
    }
}

/// The wait-free trace-ring producer: the interface the scheduler hook
/// wires. Keep the [`push`](Self::push) signature stable.
#[derive(Debug)]
#[must_use = "a producer with no pushes records nothing"]
pub struct TraceRingProducer {
    inner: ShmRingProducer,
}

impl TraceRingProducer {
    /// Publish one trace record. WAIT-FREE: encodes the record onto a 40-byte stack
    /// buffer and calls the underlying wait-free [`ShmRingProducer::push`] (no
    /// alloc, no lock, no syscall, no clock read).
    pub fn push(&mut self, record: &TraceRingRecord) {
        let bytes = record.as_bytes();
        self.inner.push(&bytes);
    }

    /// Total records published so far.
    pub fn pushed(&self) -> u64 {
        self.inner.pushed()
    }
}

/// The PARTIAL-HEAD-STEP rule for a mid-run attach, as a pure,
/// reusable drain-side state machine.
///
/// # The problem it closes
///
/// [`TraceRingConsumer::open_at_live`] lands the read cursor wherever the
/// producer happens to be — which is **mid-step**. The first records such a
/// consumer drains can therefore be `FIRE`s belonging to a step whose
/// [`RECORD_TYPE_STEP_BOUNDARY`] was committed before the attach. A trace whose
/// head step is partial makes a downstream step skeleton wrong at record 1: the
/// fires exist with no boundary to hang them on, and the boundary is the record
/// that carries the step's gating-clock value (see
/// [`RECORD_TYPE_STEP_BOUNDARY`]'s doc — replay re-advances the clock to exactly
/// that value per step).
///
/// # The rule
///
/// DISCARD records until the first `STEP_BOUNDARY`; ADMIT that boundary and
/// everything after it. The admitted boundary's `step` is the first COMPLETE
/// step, reported by [`first_step_recorded`](Self::first_step_recorded) — the
/// value a manifest publishes so nobody mistakes a mid-run trace for one that
/// began at step 0.
///
/// # Scope — this gate is for a ring that CARRIES boundaries
///
/// The rule is defined over the SCHEDULER trace ring, whose stream interleaves
/// `STEP_BOUNDARY` with `FIRE`. A ring that carries no boundary record at all —
/// the supervisor DEPARTURE ring is exactly that — would have an armed
/// gate discard **everything, forever**. Do not arm one over such a ring:
/// whether a mid-run recorder should read a departure ring from live, and what
/// it must then say about departures it can never have seen, is a caller policy
/// decision, not a property of this gate.
///
/// Use [`passthrough`](Self::passthrough) for an ordinary at-zero attach so ONE
/// call site serves both attach modes: it admits every record and still reports
/// `first_step_recorded` from the first boundary it sees.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HeadStepGate {
    /// False while the leading partial step is still being discarded.
    open: bool,
    /// How many leading records were discarded for want of a boundary.
    discarded: u64,
    /// The `step` of the first ADMITTED `STEP_BOUNDARY` — the first complete step.
    first_step_recorded: Option<u64>,
}

impl HeadStepGate {
    /// A gate ARMED for a mid-run attach: discards records until the first
    /// [`RECORD_TYPE_STEP_BOUNDARY`], which is itself admitted.
    pub fn armed() -> Self {
        Self {
            open: false,
            discarded: 0,
            first_step_recorded: None,
        }
    }

    /// A gate that admits everything — for an at-zero attach, where record 0 is
    /// already a step head. It still reports
    /// [`first_step_recorded`](Self::first_step_recorded), so a caller needs no
    /// second code path for the two attach modes.
    pub fn passthrough() -> Self {
        Self {
            open: true,
            discarded: 0,
            first_step_recorded: None,
        }
    }

    /// Offer one record to the gate in ring order. Returns `true` if it is part
    /// of the recorded trace, `false` if it belongs to the discarded partial head
    /// step.
    pub fn admit(&mut self, record: &TraceRingRecord) -> bool {
        if !self.open {
            if record.record_type != RECORD_TYPE_STEP_BOUNDARY {
                self.discarded += 1;
                return false;
            }
            // The boundary that opens the gate is ADMITTED — it is the head of
            // the first complete step, not part of the partial one.
            self.open = true;
        }
        if record.record_type == RECORD_TYPE_STEP_BOUNDARY && self.first_step_recorded.is_none() {
            self.first_step_recorded = Some(record.step);
        }
        true
    }

    /// Whether the gate has seen its first boundary (or was built
    /// [`passthrough`](Self::passthrough)) and is now admitting records.
    pub fn is_open(&self) -> bool {
        self.open
    }

    /// How many leading records were discarded as the partial head step.
    pub fn discarded(&self) -> u64 {
        self.discarded
    }

    /// The `step` of the first complete step in the recorded trace — `None`
    /// until a `STEP_BOUNDARY` has been admitted.
    pub fn first_step_recorded(&self) -> Option<u64> {
        self.first_step_recorded
    }

    /// Offer a RAW drained span — the two halves
    /// [`drain_slices`](TraceRingConsumer::drain_slices) returns, in ring order —
    /// and report how many LEADING records must be discarded.
    ///
    /// It exists because the production recorder does not decode. `bagd` drains
    /// rings zero-copy (`drain_slices` → `writev` → `commit`) and never builds a
    /// `Vec<TraceRingRecord>`, so [`drain_gated`](TraceRingConsumer::drain_gated)
    /// — which is defined over decoded records — cannot serve it. What the
    /// zero-copy path CAN do is start its span later, which is exactly a leading
    /// skip.
    ///
    /// **The rule is not restated here.** Every record is routed through
    /// [`admit`](Self::admit), so the discard-until-the-first-boundary decision
    /// and the `first_step_recorded` stamp live in ONE function; this method only
    /// converts its per-record answer into a count.
    ///
    /// **Its validity rests on `admit` being MONOTONE** — once the gate opens it
    /// never closes, so the admitted records are a SUFFIX and "skip the leading
    /// N" is a faithful encoding of the whole verdict. That is a property of
    /// `admit` rather than an assumption made here, and it is pinned by
    /// `admit_is_monotone_so_a_leading_skip_is_a_faithful_encoding`.
    ///
    /// A partial trailing record (a span whose length is not a whole multiple of
    /// [`TRACE_RECORD_SIZE`]) is IGNORED rather than decoded — the caller's own
    /// record arithmetic governs how much it writes, and a gate must not invent a
    /// record the caller will not see.
    ///
    /// Returns the number of leading records to drop. `0` means the span is
    /// admitted whole — the steady state once the gate is open, and always true
    /// of a [`passthrough`](Self::passthrough) gate.
    pub fn skip_leading(&mut self, a: &[u8], b: &[u8]) -> usize {
        const RS: usize = TRACE_RECORD_SIZE as usize;
        // The fast path is the one that runs for the whole life of a recording:
        // an open gate admits everything and needs to decode nothing at all.
        if self.open {
            // The gate still owes `first_step_recorded` if it has never seen a
            // boundary — a passthrough gate on its first span, say — so it walks
            // only until it has one.
            if self.first_step_recorded.is_none() {
                for chunk in a.as_chunks::<RS>().0.iter().chain(b.as_chunks::<RS>().0) {
                    let rec = TraceRingRecord::from_bytes(chunk);
                    self.admit(&rec);
                    if self.first_step_recorded.is_some() {
                        break;
                    }
                }
            }
            return 0;
        }
        let mut skipped = 0usize;
        for chunk in a.as_chunks::<RS>().0.iter().chain(b.as_chunks::<RS>().0) {
            let rec = TraceRingRecord::from_bytes(chunk);
            if self.admit(&rec) {
                return skipped;
            }
            skipped += 1;
        }
        // Every record in the span belonged to the partial head step. The gate
        // stays armed, so the next span is judged from where this one left off.
        skipped
    }
}

/// The trace-ring consumer — decodes [`TraceRingRecord`]s (for tests/tools) and
/// passes through the zero-copy [`drain_slices`](Self::drain_slices) /
/// [`commit`](Self::commit) for the future `bagd` `writev` path.
#[derive(Debug)]
#[must_use = "a consumer that is never drained reads nothing"]
pub struct TraceRingConsumer {
    inner: ShmRingConsumer,
    node_ids: Vec<String>,
    /// Each node's ordered input-name table (parallel to
    /// `node_ids`; the list position is a kind-6 record's `input_idx`).
    /// One EMPTY list per node for a manifest with no input section.
    input_names: Vec<Vec<String>>,
    /// Each node's `(output name, publisher id)` table (parallel
    /// to `node_ids`) — the offline resolver for a [`READ_OUTCOME_PRODUCER`]
    /// annotation's token. One EMPTY list per node for a manifest with no
    /// publisher section (every older ring, and every degraded one).
    publisher_ids: Vec<Vec<(String, u128)>>,
    /// Each node's `(input_idx, role, capacity)` staging-rim rows
    /// (parallel to `node_ids`) — what the replay ADOPTS so both sides truncate
    /// identically. One EMPTY list per node for a manifest with no capacity
    /// section (every older ring, and every degraded one). The row is keyed
    /// on `(input_idx, role)`, never on position.
    stage_capacities: Vec<Vec<(u16, u8, u32)>>,
}

impl TraceRingConsumer {
    /// STRICT-open a trace ring by its object name. Validates the record size is
    /// [`TRACE_RECORD_SIZE`] and decodes the node-id manifest.
    pub fn open(shm_name: &str) -> TraceRingResult<Self> {
        Self::from_inner(ShmRingConsumer::open(shm_name)?)
    }

    /// STRICT-open a trace ring **at the producer's CURRENT write cursor** — the
    /// mid-run attach seam. See
    /// [`ShmRingConsumer::open_at_live`] for the cursor semantics and what the
    /// attach instant does and does not claim.
    ///
    /// Node identity is UNAFFECTED by attaching late: the manifest is written
    /// once at create and read here at open, so a `node_idx` in a record drained
    /// after a mid-run attach resolves through [`node_ids`](Self::node_ids)
    /// exactly as it would for a consumer that had been attached from record 0.
    ///
    /// The records this consumer first sees can be the TAIL of a step whose
    /// [`RECORD_TYPE_STEP_BOUNDARY`] was committed before the attach — a step
    /// skeleton that begins mid-step. Pair this with a
    /// [`HeadStepGate::armed`] and [`drain_gated`](Self::drain_gated) to drop
    /// that partial head step.
    pub fn open_at_live(shm_name: &str) -> TraceRingResult<Self> {
        Self::from_inner(ShmRingConsumer::open_at_live(shm_name)?)
    }

    /// The shared tail of [`open`](Self::open) / [`open_at_live`](Self::open_at_live):
    /// validate the record size and decode the node-id manifest. ONE body, so the
    /// two entry points cannot drift in what they validate.
    fn from_inner(inner: ShmRingConsumer) -> TraceRingResult<Self> {
        if inner.record_size() != TRACE_RECORD_SIZE {
            return Err(TraceRingError::RecordSizeMismatch {
                actual: inner.record_size(),
                expected: TRACE_RECORD_SIZE,
            });
        }
        // One decode serves all four tables: a manifest with no input
        // section yields empty per-node input lists, one with no publisher
        // section empty publisher lists, and one with no capacity section
        // empty capacity rows.
        let (node_ids, input_names, publisher_ids, stage_capacities) =
            decode_manifest_with_inputs_publishers_and_capacities(inner.manifest())?;
        Ok(Self {
            inner,
            node_ids,
            input_names,
            publisher_ids,
            stage_capacities,
        })
    }

    /// Decode-drain all currently-available records into `out`, then commit. Returns
    /// the number of records appended. Errors
    /// [`TraceRingError::Ring`]`(`[`ShmRingError::Overrun`]`)` on a lap (a
    /// convenience for tests/tools; the zero-copy path uses
    /// [`drain_slices`](Self::drain_slices) + [`commit`](Self::commit)).
    ///
    /// On `Err`, `out` is UNCHANGED: records decoded before a torn-drain `commit`
    /// failure may have been overwritten mid-read (garbage), so they are truncated
    /// away rather than handed to the caller.
    pub fn drain(&mut self, out: &mut Vec<TraceRingRecord>) -> TraceRingResult<usize> {
        self.drain_inner(out, None, || {})
    }

    /// [`drain`](Self::drain) with the partial-head-step rule applied: every
    /// decoded record is offered to `gate`, and only ADMITTED records are pushed
    /// to `out`. Returns the number of records **admitted** (not the number
    /// consumed off the ring — the discarded head is consumed and committed like
    /// any other record, because it was really read).
    ///
    /// Pair with [`open_at_live`](Self::open_at_live) + [`HeadStepGate::armed`]
    /// for a mid-run attach; an at-zero attach can pass a
    /// [`HeadStepGate::passthrough`] through the same call site and still learn
    /// [`first_step_recorded`](HeadStepGate::first_step_recorded).
    ///
    /// **`Ok(0)` is AMBIGUOUS and the caller must disambiguate it.** It means
    /// "nothing was RECORDED", which covers two very different states: the ring
    /// was idle, or a batch was consumed and every record in it belonged to the
    /// discarded partial head — the ORDINARY first drain of a mid-run attach.
    /// The return value cannot tell them apart, and deliberately does not try:
    /// the gate already carries the distinction, so widening the return type
    /// would give one caller a second way to ask the same question.
    /// [`HeadStepGate::discarded`] is the disambiguator (it GREW iff records
    /// were destroyed), and [`read_cursor`](Self::read_cursor) is the
    /// independent cross-check (it advanced by what was consumed, admitted or
    /// not — pinned by `a_drain_that_admits_nothing_still_commits_what_it_consumed`).
    /// A caller reporting coverage MUST read the gate, or a recording that
    /// silently began mid-step looks byte-identical to one that began at step 0.
    ///
    /// The gate is state carried ACROSS calls, never re-armed at entry: a
    /// looping caller passes the same gate every drain, and a batch arriving
    /// after the gate opened is admitted whole even when it contains no
    /// boundary of its own.
    ///
    /// Torn-drain semantics match [`drain`](Self::drain) exactly: on `Err` the
    /// records appended by THIS call are truncated away **and the gate is rolled
    /// back to its EXACT pre-call state** (restored, not re-armed), so a failed
    /// drain leaves neither half-trusted records nor a gate that believes it saw
    /// a boundary in bytes the producer may have overwritten mid-read.
    pub fn drain_gated(
        &mut self,
        gate: &mut HeadStepGate,
        out: &mut Vec<TraceRingRecord>,
    ) -> TraceRingResult<usize> {
        self.drain_inner(out, Some(gate), || {})
    }

    /// TEST SEAM: [`drain`](Self::drain) with a `pre_commit` hook that runs AFTER
    /// the decode/push into `out` and BEFORE the internal `commit`. Exists SOLELY
    /// to pin the truncate-on-torn-commit branch deterministically — a lap landing
    /// in that window cannot be forced single-threaded through the public API (the
    /// producer would have to push mid-`drain`), so the hook stands in for the
    /// racing producer. Zero production impact: `drain` routes through the same
    /// `drain_inner` with a no-op closure that inlines away.
    #[cfg(any(test, feature = "test-helpers"))]
    #[doc(hidden)]
    pub fn drain_with_pre_commit_hook_for_test(
        &mut self,
        out: &mut Vec<TraceRingRecord>,
        pre_commit: impl FnOnce(),
    ) -> TraceRingResult<usize> {
        self.drain_inner(out, None, pre_commit)
    }

    /// TEST SEAM: [`drain_gated`](Self::drain_gated) with the same `pre_commit`
    /// hook as `drain_with_pre_commit_hook_for_test`, so the GATE-ROLLBACK half
    /// of the torn-drain branch is pinnable single-threaded. Same zero
    /// production impact — one `drain_inner`, one no-op closure.
    #[cfg(any(test, feature = "test-helpers"))]
    #[doc(hidden)]
    pub fn drain_gated_with_pre_commit_hook_for_test(
        &mut self,
        gate: &mut HeadStepGate,
        out: &mut Vec<TraceRingRecord>,
        pre_commit: impl FnOnce(),
    ) -> TraceRingResult<usize> {
        self.drain_inner(out, Some(gate), pre_commit)
    }

    /// The shared body of [`drain`](Self::drain) / [`drain_gated`](Self::drain_gated) /
    /// `drain_with_pre_commit_hook_for_test`: drain-slices → decode/gate/push →
    /// hook → commit (truncating `out` — and rolling the gate back — on a
    /// torn-drain commit failure).
    ///
    /// The commit count is the number of records CONSUMED off the ring, never
    /// the number admitted: a gate-discarded head record was really read, and
    /// committing less than was drained would re-serve it forever.
    fn drain_inner(
        &mut self,
        out: &mut Vec<TraceRingRecord>,
        mut gate: Option<&mut HeadStepGate>,
        pre_commit: impl FnOnce(),
    ) -> TraceRingResult<usize> {
        const RS: usize = TRACE_RECORD_SIZE as usize;
        let original_len = out.len();
        let gate_before: Option<HeadStepGate> = gate.as_deref().copied();
        // Scope the borrowed slices so `commit` (a `&mut self` call) can run after.
        let consumed = {
            let (a, b) = self.inner.drain_slices()?;
            for chunk in a.as_chunks::<RS>().0.iter().chain(b.as_chunks::<RS>().0) {
                let record = TraceRingRecord::from_bytes(chunk);
                let admitted = match gate.as_deref_mut() {
                    Some(g) => g.admit(&record),
                    None => true,
                };
                if admitted {
                    out.push(record);
                }
            }
            (a.len() + b.len()) / RS
        };
        pre_commit();
        if let Err(e) = self.inner.commit(consumed as u64) {
            // Torn drain: the decoded records may be garbage — don't leak them,
            // and don't let the gate keep a boundary it may never have seen.
            out.truncate(original_len);
            if let (Some(g), Some(before)) = (gate, gate_before) {
                *g = before;
            }
            return Err(e.into());
        }
        Ok(out.len() - original_len)
    }

    /// Zero-copy passthrough: the unread region as up-to-2 raw byte slices into the
    /// mapping (for `bagd`'s `writev`). Pair with [`commit`](Self::commit).
    ///
    /// DELIBERATELY ring-level (`ShmRingResult`, unlike the typed methods above):
    /// this is the raw `writev` fast path, a 1:1 passthrough to
    /// [`ShmRingConsumer::drain_slices`] with no trace-layer semantics added.
    pub fn drain_slices(&mut self) -> ShmRingResult<(&[u8], &[u8])> {
        self.inner.drain_slices()
    }

    /// Zero-copy passthrough: advance the read cursor, re-validating no torn drain.
    ///
    /// DELIBERATELY ring-level (`ShmRingResult`) — the raw partner of
    /// [`drain_slices`](Self::drain_slices); see its note.
    pub fn commit(&mut self, n_records: u64) -> ShmRingResult<()> {
        self.inner.commit(n_records)
    }

    /// Unread record count (`> capacity` signals a lap).
    pub fn available(&self) -> u64 {
        self.inner.available()
    }

    /// The consumer's local read cursor (records consumed + committed).
    pub fn read_cursor(&self) -> u64 {
        self.inner.read_cursor()
    }

    /// The read cursor as PUBLISHED in the ring header — which on a trace ring
    /// is ALWAYS the create-time zero, because a trace ring is
    /// `OverrunPolicy::FailLoud` and publishing is gated on `Backpressure`
    /// (`shm_ring` module doc contract 7).
    ///
    /// That divergence from [`read_cursor`](Self::read_cursor) is the
    /// OBSERVABLE behind the multi-reader contract (Principle #3): it is
    /// how a test can show that N readers of one trace ring cannot advance each
    /// other's position, rather than merely showing that today they happen not
    /// to. Pinned by
    /// `shm_ring_test::two_failloud_consumers_each_see_the_whole_stream`.
    pub fn published_read_cursor(&self) -> u64 {
        self.inner.published_read_cursor()
    }

    /// The decoded node-id manifest table.
    pub fn node_ids(&self) -> &[String] {
        &self.node_ids
    }

    /// Each node's ordered input-name table, parallel to
    /// [`node_ids`](Self::node_ids) — a kind-6 record's `input_idx` indexes
    /// its node's list. Empty lists for a ring with no input section.
    pub fn input_names(&self) -> &[Vec<String>] {
        &self.input_names
    }

    /// Each node's `(output name, publisher id)` table, parallel
    /// to [`node_ids`](Self::node_ids) — the offline resolver for a
    /// [`READ_OUTCOME_PRODUCER`] annotation's token. Empty lists for an older
    /// ring (no publisher section) or one whose manifest degraded past the
    /// budget.
    pub fn publisher_ids(&self) -> &[Vec<(String, u128)>] {
        &self.publisher_ids
    }

    /// Each node's `(input_idx, role, capacity)` staging-rim rows,
    /// parallel to [`Self::node_ids`]. EMPTY per node on a manifest with no
    /// capacity section — which the bag stamp renders as an absent table, and
    /// the replay side treats as "this bag declares no rims" rather than
    /// adopting one it was never told.
    pub fn stage_capacities(&self) -> &[Vec<(u16, u8, u32)>] {
        &self.stage_capacities
    }

    /// Which manifest sections the WRITER's degrade ladder
    /// dropped ([`DegradedSections`]; `0` on every ring written without the ladder and on every
    /// ring that degraded nothing).
    ///
    /// The recorder needs this because an EMPTY section cannot say why it is
    /// empty: a stageless rank and a rank whose sections were dropped decode
    /// identically, and they call for opposite answers from a replay.
    pub fn degraded_sections(&self) -> u32 {
        self.inner.degraded_sections()
    }

    /// The producer rank from the header.
    pub fn rank(&self) -> u32 {
        self.inner.rank()
    }

    /// The create-generation from the header (restart detection).
    pub fn generation(&self) -> u64 {
        self.inner.generation()
    }

    /// The per-record size from the header (always [`TRACE_RECORD_SIZE`] here).
    pub fn record_size(&self) -> u32 {
        self.inner.record_size()
    }

    /// The object name this consumer opened.
    pub fn name(&self) -> &str {
        self.inner.name()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_size_is_forty() {
        assert_eq!(std::mem::size_of::<TraceRingRecord>(), 40);
        assert_eq!(TRACE_RECORD_SIZE, 40);
    }

    /// The SHARED worker-roster contiguity rule, against a hand
    /// oracle.
    ///
    /// `bag play --resim` refuses a holed roster (`MultiRankManifestGap`) and a
    /// Flashback capture must refuse to claim `resimmable` on the same one, so
    /// this is a shared refusal predicate and its boundaries are pinned here
    /// rather than at either caller.
    #[test]
    fn the_worker_roster_gap_rule_answers_its_hand_oracle() {
        // Contiguous from 0 — every shipping shape.
        assert_eq!(first_rank_manifest_gap(&[0]), None);
        assert_eq!(first_rank_manifest_gap(&[0, 1, 2, 3]), None);
        // EMPTY is not a hole: there is no roster to have one in, and the
        // callers refuse an empty manifest set separately and for a different
        // reason.
        assert_eq!(first_rank_manifest_gap(&[]), None);
        // The FIRST missing rank is what the error names, so a roster with two
        // holes reports the earlier one.
        assert_eq!(first_rank_manifest_gap(&[0, 2]), Some(1));
        assert_eq!(first_rank_manifest_gap(&[0, 2, 4]), Some(1));
        assert_eq!(first_rank_manifest_gap(&[0, 1, 3]), Some(2));
        // A roster that does not START at 0 is missing rank 0 — the
        // shape a zero-filled table reads as "rank 0 has no nodes" rather than
        // as absent.
        assert_eq!(first_rank_manifest_gap(&[1]), Some(0));
        assert_eq!(first_rank_manifest_gap(&[2, 3]), Some(0));
        // The DEPARTURE sentinel is the caller's to exclude, and this is what
        // happens if one forgets: the rule reports every worker rank in between
        // as missing rather than silently tolerating it. Documented so the
        // precondition reads as load-bearing rather than as a formality.
        assert_eq!(first_rank_manifest_gap(&[0, DEPARTURE_RING_RANK]), Some(1));
    }

    /// The SHARED departure predicate — both of its signals, and the
    /// two shapes that must NOT trip it.
    ///
    /// It is a shared refusal rule (`bag play --resim` refuses a bag on it; a
    /// Flashback capture refuses to claim `resimmable` on it), so its boundary
    /// is pinned here rather than at either caller.
    #[test]
    fn the_departure_predicate_answers_both_signals_and_neither_false_positive() {
        let base = TraceRingRecord {
            step: 9,
            fire_time_ns: 1,
            duration_ns: 0,
            node_idx: 0,
            global_level: 0,
            record_type: RECORD_TYPE_FIRE,
            reserved: 0,
        };

        // Signal 1: the record KIND.
        let by_kind = TraceRingRecord {
            record_type: RECORD_TYPE_DEPARTURE,
            ..base
        };
        assert!(by_kind.is_departure_boundary());

        // Signal 2: the supervisor departure-RING sentinel, on a record of any
        // kind (bagd stamps the ring's rank into `reserved` for every record it
        // drains from that ring, boundaries included).
        for record_type in [RECORD_TYPE_FIRE, RECORD_TYPE_STEP_BOUNDARY] {
            let by_rank = TraceRingRecord {
                record_type,
                reserved: DEPARTURE_RING_RANK,
                ..base
            };
            assert!(
                by_rank.is_departure_boundary(),
                "a record from the departure ring is a departure whatever its kind"
            );
        }

        // NOT a departure: an ordinary worker record.
        assert!(!base.is_departure_boundary());
        assert!(!TraceRingRecord {
            record_type: RECORD_TYPE_STEP_BOUNDARY,
            reserved: 3,
            ..base
        }
        .is_departure_boundary());

        // NOT a departure: a DISCARD-marked FIRE record from the
        // highest legal rank. This is the arm the raw-vs-masked rule exists for
        // — and it is asserted in BOTH directions, because a masked comparison
        // would make the sentinel unreachable while a careless widening would
        // make every discard-marked record read as a fault.
        let discarded = TraceRingRecord {
            record_type: RECORD_TYPE_FIRE,
            reserved: 5 | TRACE_DISCARD_BIT,
            ..base
        };
        assert!(discarded.is_discarded());
        assert!(
            !discarded.is_departure_boundary(),
            "a discard-marked fire is not a peer departure"
        );
        // The masked reading of the sentinel is NOT the sentinel — which is why
        // the predicate must read `reserved` raw.
        assert_ne!(DEPARTURE_RING_RANK & TRACE_RANK_MASK, DEPARTURE_RING_RANK);
    }

    #[test]
    fn record_byte_layout_oracle() {
        // Hand-built record with distinct, byte-recognisable field values.
        let r = TraceRingRecord {
            step: 0x0102_0304_0506_0708,
            fire_time_ns: 0x1112_1314_1516_1718,
            duration_ns: 0x2122_2324_2526_2728,
            node_idx: 0x3132_3334,
            global_level: 0x4142_4344,
            record_type: RECORD_TYPE_DEPARTURE, // 2
            reserved: 0,
        };
        // Little-endian, field order per the format contract.
        let expected: [u8; 40] = [
            0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, // step
            0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, // fire_time_ns
            0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21, // duration_ns
            0x34, 0x33, 0x32, 0x31, // node_idx
            0x44, 0x43, 0x42, 0x41, // global_level
            0x02, 0x00, 0x00, 0x00, // record_type = 2
            0x00, 0x00, 0x00, 0x00, // reserved
        ];
        assert_eq!(
            r.as_bytes(),
            expected,
            "record byte layout is a format contract"
        );
    }

    // The discard-marker consts + accessors. Oracle-vector decode over
    // {rank} × {discarded, not} plus the departure sentinel and STEP_BOUNDARY,
    // asserting exact decode and non-collision with the rank stamp.
    #[test]
    fn discard_marker_consts_are_disjoint_and_cover_reserved() {
        assert_eq!(TRACE_DISCARD_BIT, 0x8000_0000);
        assert_eq!(TRACE_RANK_MASK, 0x7FFF_FFFF);
        // The two subfields partition the whole u32 with no overlap.
        assert_eq!(TRACE_DISCARD_BIT & TRACE_RANK_MASK, 0, "disjoint");
        assert_eq!(
            TRACE_DISCARD_BIT | TRACE_RANK_MASK,
            u32::MAX,
            "cover all 32 bits"
        );
    }

    #[test]
    fn rank_and_is_discarded_decode_oracle() {
        // {rank 0..K} × {discarded, not}: rank() strips the bit, is_discarded()
        // reports it — for FIRE records only.
        for &rank in &[0u32, 1, 7, 5, 65_535, 0x7FFF_FFFE] {
            // Undiscarded FIRE record: reserved == rank, bit clear.
            let plain = TraceRingRecord {
                reserved: rank,
                record_type: RECORD_TYPE_FIRE,
                ..TraceRingRecord::default()
            };
            assert_eq!(plain.rank(), rank, "rank() decodes an undiscarded rank");
            assert!(!plain.is_discarded(), "no bit ⇒ not discarded");

            // Discard-marked FIRE record: reserved == rank | DISCARD_BIT.
            let marked = TraceRingRecord {
                reserved: rank | TRACE_DISCARD_BIT,
                record_type: RECORD_TYPE_FIRE,
                ..TraceRingRecord::default()
            };
            assert_eq!(marked.rank(), rank, "rank() strips the discard bit");
            assert!(marked.is_discarded(), "bit set on a FIRE ⇒ discarded");
            // No legal rank+bit collides with the departure sentinel.
            assert_ne!(
                marked.reserved,
                u32::MAX,
                "rank {rank} | DISCARD_BIT can never equal the departure sentinel"
            );
        }
    }

    #[test]
    fn is_discarded_gates_on_fire_record_type() {
        // A STEP_BOUNDARY or DEPARTURE record with bit 31 set (corrupt input) is
        // NOT read as a discard — is_discarded() gates on RECORD_TYPE_FIRE.
        for rt in [RECORD_TYPE_STEP_BOUNDARY, RECORD_TYPE_DEPARTURE, 0, 99] {
            let r = TraceRingRecord {
                reserved: 3 | TRACE_DISCARD_BIT,
                record_type: rt,
                ..TraceRingRecord::default()
            };
            assert!(
                !r.is_discarded(),
                "record_type {rt} never reports discarded (bit is FIRE-only)"
            );
            // rank() still strips the bit uniformly (harmless no-op on boundaries).
            assert_eq!(r.rank(), 3);
        }
    }

    #[test]
    fn departure_sentinel_is_not_a_fire_discard() {
        // u32::MAX (the departure sentinel) on a FIRE record: rank()
        // strips bit 31 to 0x7FFF_FFFF and is_discarded() is true — but a REAL
        // departure record is RECORD_TYPE_DEPARTURE and is caught by the
        // raw-`reserved == u32::MAX` gate BEFORE any rank/discard extraction (the
        // gate stays raw). This oracle only pins the decode.
        let fire = TraceRingRecord {
            reserved: u32::MAX,
            record_type: RECORD_TYPE_FIRE,
            ..TraceRingRecord::default()
        };
        assert_eq!(fire.rank(), 0x7FFF_FFFF);
        assert!(fire.is_discarded());
    }

    /// The shared record walk reports the FIRST refusing
    /// record in FILE ORDER — never a fixed precedence between the two kinds.
    ///
    /// The gate is one walk that refuses per record (departure first, then
    /// `classify_trace_record`), so on a trace carrying BOTH a departure and a
    /// record-level fault the gap a bag reports is decided by which record comes
    /// first. Both directions are driven here from ONE record vector reordered,
    /// so the oracle cannot be satisfied by a walk that always answers the same
    /// kind — and the departure-only / fault-only / clean arms sit in the same
    /// body so an all-`None` or an all-`Some` walk fails too.
    #[test]
    fn the_first_refusing_record_in_file_order_is_the_one_reported() {
        // One rank declaring one node, so `node_idx` 0 is in range.
        let ranks = [1usize];
        let ok = boundary(1);
        let dep = departure(2);
        let alien = TraceRingRecord {
            step: 3,
            record_type: 7,
            ..TraceRingRecord::default()
        };
        let unsupported = || {
            Some(TraceRecordRefusal::Fault(
                TraceRecordFault::UnsupportedRecordType { record_type: 7 },
            ))
        };

        assert_eq!(
            first_trace_record_refusal([&ok], &ranks),
            None,
            "a well-formed trace refuses nothing"
        );
        assert_eq!(
            first_trace_record_refusal([&ok, &dep], &ranks),
            Some(TraceRecordRefusal::Departure),
        );
        assert_eq!(
            first_trace_record_refusal([&ok, &alien], &ranks),
            unsupported()
        );

        // THE pin: the same two refusing records, both orders. Whichever is
        // FIRST is the answer — an early return at a departure would make the
        // fault-then-departure order report nothing at all, so the judge would fall
        // through to its aggregate departure arm and name a gap the gate would not.
        assert_eq!(
            first_trace_record_refusal([&ok, &alien, &dep], &ranks),
            unsupported(),
            "the fault comes first in file order, so the fault is the refusal"
        );
        assert_eq!(
            first_trace_record_refusal([&ok, &dep, &alien], &ranks),
            Some(TraceRecordRefusal::Departure),
            "the departure comes first in file order, so the departure is the refusal"
        );
    }

    #[test]
    fn record_roundtrip_is_identity() {
        let r = TraceRingRecord {
            step: 42,
            fire_time_ns: 1_000_000,
            duration_ns: 7,
            node_idx: 3,
            global_level: 1,
            record_type: RECORD_TYPE_FIRE,
            reserved: 0,
        };
        assert_eq!(TraceRingRecord::from_bytes(&r.as_bytes()), r);
    }

    // =======================================================================
    // The kind-6 READ-OUTCOME record + the additive
    // manifest input section. Every packing assertion is against HAND-BUILT
    // byte/word oracles — never a pack/unpack self-compare alone.
    // =======================================================================

    #[test]
    fn read_outcome_kind_is_distinct_and_record_stays_forty_bytes() {
        // Drift guard: kind 6 collides with NO existing kind, sits past the
        // 4/5 reservation, and the record layout is still the 40-byte contract.
        assert_eq!(RECORD_TYPE_READ_OUTCOME, 6);
        for existing in [
            RECORD_TYPE_FIRE,
            RECORD_TYPE_DEPARTURE,
            RECORD_TYPE_STEP_BOUNDARY,
            4, // reserved: keyframe
            5, // reserved: nondeterminism
        ] {
            assert_ne!(RECORD_TYPE_READ_OUTCOME, existing);
        }
        assert_eq!(std::mem::size_of::<TraceRingRecord>(), 40);
        assert_eq!(TRACE_RECORD_SIZE, 40);
    }

    /// The drift guard for the TWO annotation kinds: they sit
    /// in the FREE outcome-kind space (5 of 65,536 were used), never in the
    /// RECORD-type space, which is what keeps an annotated bag format-3-clean.
    #[test]
    fn annotation_kinds_live_in_the_outcome_space_not_the_record_space() {
        // The RECORD type is unchanged — a format-3 reader's `record_type`
        // gate (kinds >= 7 are UnsupportedRecordType) never sees them.
        assert_eq!(
            TraceRingRecord::read_outcome_truncated(1, 0, 0, 3, ReadSiteRole::Drain).record_type,
            RECORD_TYPE_READ_OUTCOME
        );
        assert_eq!(
            TraceRingRecord::read_outcome_producer(1, 0, 0, Some(9), 0xDEAD, ReadSiteRole::Body)
                .record_type,
            RECORD_TYPE_READ_OUTCOME
        );
        // The OUTCOME kinds are new and collide with nothing.
        assert_eq!(READ_OUTCOME_TRUNCATED, 6);
        assert_eq!(READ_OUTCOME_PRODUCER, 7);
        for existing in [
            READ_OUTCOME_SERVED,
            READ_OUTCOME_HELD,
            READ_OUTCOME_NONE,
            READ_OUTCOME_DRAINED_BATCH,
            READ_OUTCOME_DECIMATED,
        ] {
            assert_ne!(READ_OUTCOME_TRUNCATED, existing);
            assert_ne!(READ_OUTCOME_PRODUCER, existing);
        }
    }

    /// The OVERFLOW MARKER as a HAND-BUILT 40-byte LE array,
    /// never a pack/unpack self-compare.
    #[test]
    fn read_outcome_truncated_byte_oracle() {
        let rec = TraceRingRecord::read_outcome_truncated(
            0x0000_0000_0000_002A, // step 42
            7,                     // node_idx
            2,                     // input_idx
            5,                     // dropped in THIS window
            // The ROLE-0 vector — the invariance claim. A record
            // whose role bits were never written must pack the SAME word a
            // pre-role writer produced, byte for byte. The role-BEARING vectors
            // are the sibling test below.
            ReadSiteRole::Unstamped,
        );
        let mut want = [0u8; 40];
        want[0..8].copy_from_slice(&42u64.to_le_bytes()); // step
                                                          // fire_time_ns = the no-frame sentinel (a marker serves nothing).
        want[8..16].copy_from_slice(&u64::MAX.to_le_bytes());
        // duration_ns = the ORDINARY aux packing: low 32 = the count, high 32
        // = 0 (the re-offer-count reservation untouched).
        want[16..24].copy_from_slice(&5u64.to_le_bytes());
        want[24..28].copy_from_slice(&7u32.to_le_bytes()); // node_idx
                                                           // global_level = input_idx HIGH 16 | role 0 | kind = 0x0002_0006
                                                           // — the PRE-ROLE word, unchanged (a marker's role is
                                                           // `Unstamped` here, so bits 14..16 are zero).
        want[28..32].copy_from_slice(&0x0002_0006u32.to_le_bytes());
        want[32..36].copy_from_slice(&6u32.to_le_bytes()); // record_type 6
        want[36..40].copy_from_slice(&0u32.to_le_bytes()); // reserved
        assert_eq!(rec.as_bytes(), want);
        // And the round trip, so the oracle pins the DECODE too.
        assert_eq!(TraceRingRecord::from_bytes(&want), rec);
        // The aux really reads back as a count through the ordinary unpacker.
        assert_eq!(unpack_read_outcome_popped(rec.duration_ns), 5);
        assert_eq!(
            unpack_read_outcome_meta(rec.global_level),
            (2, READ_OUTCOME_TRUNCATED)
        );
        assert_eq!(read_site_role(rec.global_level), ReadSiteRole::Unstamped);
    }

    /// The OVERFLOW MARKER carrying a real (STAGE) role, as
    /// hand-built words. Everything OUTSIDE the meta word must be identical to
    /// the role-0 oracle above — the role is two bits in one field, not a
    /// re-layout.
    #[test]
    fn read_outcome_truncated_role_byte_oracle() {
        for (role, want_meta) in [
            (ReadSiteRole::Drain, 0x0002_4006u32),
            (ReadSiteRole::Body, 0x0002_8006u32),
        ] {
            let rec = TraceRingRecord::read_outcome_truncated(42, 7, 2, 5, role);
            let mut want = [0u8; 40];
            want[0..8].copy_from_slice(&42u64.to_le_bytes());
            want[8..16].copy_from_slice(&u64::MAX.to_le_bytes());
            want[16..24].copy_from_slice(&5u64.to_le_bytes());
            want[24..28].copy_from_slice(&7u32.to_le_bytes());
            want[28..32].copy_from_slice(&want_meta.to_le_bytes());
            want[32..36].copy_from_slice(&6u32.to_le_bytes());
            want[36..40].copy_from_slice(&0u32.to_le_bytes());
            assert_eq!(rec.as_bytes(), want, "marker role {role:?}");
            assert_eq!(TraceRingRecord::from_bytes(&want), rec);
            // The kind and the input index are UNMOVED by the role.
            assert_eq!(
                unpack_read_outcome_meta(rec.global_level),
                (2, READ_OUTCOME_TRUNCATED)
            );
            assert_eq!(read_site_role(rec.global_level), role);
            // The aux word is still the ordinary count packing (the re-offer-count
            // reservation untouched — the role never touches aux).
            assert_eq!(unpack_read_outcome_popped(rec.duration_ns), 5);
        }
    }

    /// The PRODUCER annotation as a HAND-BUILT 40-byte LE
    /// array. The whole point is the aux word: the FULL 64 bits are the
    /// token, which is why the high half is NOT zero here (and why the re-offer-count
    /// reservation, a property of a READ record's aux, cannot collide).
    #[test]
    fn read_outcome_producer_byte_oracle() {
        const TOKEN: u64 = 0x0123_4567_89AB_CDEF;
        let rec = TraceRingRecord::read_outcome_producer(
            9,          // step
            1,          // node_idx
            3,          // input_idx
            Some(1234), // the annotated read's served seq (join key)
            TOKEN,
            // Role-0, so the hand oracle below is the pre-role word.
            ReadSiteRole::Unstamped,
        );
        let mut want = [0u8; 40];
        want[0..8].copy_from_slice(&9u64.to_le_bytes());
        want[8..16].copy_from_slice(&1234u64.to_le_bytes()); // served seq
        want[16..24].copy_from_slice(&TOKEN.to_le_bytes()); // FULL 64-bit aux
        want[24..28].copy_from_slice(&1u32.to_le_bytes());
        want[28..32].copy_from_slice(&0x0003_0007u32.to_le_bytes());
        want[32..36].copy_from_slice(&6u32.to_le_bytes());
        want[36..40].copy_from_slice(&0u32.to_le_bytes());
        assert_eq!(rec.as_bytes(), want);
        assert_eq!(TraceRingRecord::from_bytes(&want), rec);
        // The high half really is part of the token (dropping it in
        // `pack_read_outcome_aux` would make this line read 0).
        assert_eq!(rec.duration_ns >> 32, 0x0123_4567);
        // A no-frame producer annotation still packs the sentinel.
        let none =
            TraceRingRecord::read_outcome_producer(9, 1, 3, None, TOKEN, ReadSiteRole::Unstamped);
        assert_eq!(none.fire_time_ns, READ_OUTCOME_NO_FRAME);
        assert_eq!(none.duration_ns, TOKEN);
    }

    /// ANTI-TAUTOLOGY (the format-3-clean claim): an OLD
    /// reader's record-level machinery accepts both annotations and decodes
    /// every field it reads to the value the writer meant. Nothing here is
    /// new code: `classify_trace_record` and the `unpack_*` helpers are the
    /// format-3 surface verbatim.
    ///
    /// Driven under a STAMPED role as well as an unstamped one.
    /// The role rides bits 14..16 of the KIND half, so it can only be a
    /// no-change for the record-type gate and for `input_idx` — asserting that
    /// rather than assuming it is what stops a future role encoding from
    /// silently moving either. (A real role-bearing bag stamps `trace_format` 5, so a
    /// format-3 binary refuses it up front and never reaches this machinery;
    /// what is pinned here is that the RECORD-level surface is unmoved.)
    #[test]
    fn format_three_reader_decodes_both_annotations_without_refusal() {
        let nodes = 4usize;
        for rec in [
            TraceRingRecord::read_outcome_truncated(1, 2, 0, 11, ReadSiteRole::Unstamped),
            TraceRingRecord::read_outcome_producer(
                1,
                2,
                0,
                Some(77),
                0xFFFF_0000_FFFF_0000,
                ReadSiteRole::Unstamped,
            ),
            TraceRingRecord::read_outcome_truncated(1, 2, 0, 11, ReadSiteRole::Drain),
            TraceRingRecord::read_outcome_producer(
                1,
                2,
                0,
                Some(77),
                0xFFFF_0000_FFFF_0000,
                ReadSiteRole::Body,
            ),
        ] {
            // (a) the record-type gate passes (kind 6, node_idx in range) —
            // no `UnsupportedRecordType`, no `NodeIdxOutOfRange`.
            assert_eq!(
                classify_trace_record(&rec, &[nodes]),
                None,
                "a format-3 reader must not REFUSE an annotation record"
            );
            // (b) every field it reads decodes to what the writer meant.
            let (input_idx, _kind) = unpack_read_outcome_meta(rec.global_level);
            assert_eq!(input_idx, 0);
            assert_eq!(rec.step, 1);
            assert_eq!(rec.node_idx, 2);
            // (c) the round trip is byte-exact through the shared codec.
            assert_eq!(TraceRingRecord::from_bytes(&rec.as_bytes()), rec);
        }
    }

    /// The ROLE-0 BYTE-INVARIANCE oracle, and the whole
    /// enforcement of the claim that "role 0 is what old bags already
    /// contain": every value here was written before roles existed, and every
    /// one of them must survive the signature change BYTE FOR BYTE. If one
    /// shifts, the claim is false and every archived bag reads differently.
    #[test]
    fn read_outcome_meta_packing_oracle() {
        // Hand oracles: input_idx HIGH 16, outcome kind LOW 16 — the PRE-ROLE
        // words, unchanged.
        assert_eq!(
            pack_read_outcome_meta(0, READ_OUTCOME_SERVED, ReadSiteRole::Unstamped),
            0x0000_0001
        );
        assert_eq!(
            pack_read_outcome_meta(1, READ_OUTCOME_HELD, ReadSiteRole::Unstamped),
            0x0001_0002
        );
        assert_eq!(
            pack_read_outcome_meta(2, READ_OUTCOME_NONE, ReadSiteRole::Unstamped),
            0x0002_0003
        );
        assert_eq!(
            pack_read_outcome_meta(0x00AB, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Unstamped),
            0x00AB_0004
        );
        assert_eq!(
            pack_read_outcome_meta(u16::MAX, READ_OUTCOME_DECIMATED, ReadSiteRole::Unstamped),
            0xFFFF_0005
        );
        // Total inverse over the boundary values.
        for (idx, kind) in [
            (0u16, READ_OUTCOME_SERVED),
            (1, READ_OUTCOME_HELD),
            (0x00AB, READ_OUTCOME_DRAINED_BATCH),
            (u16::MAX, READ_OUTCOME_DECIMATED),
        ] {
            assert_eq!(
                unpack_read_outcome_meta(pack_read_outcome_meta(
                    idx,
                    kind,
                    ReadSiteRole::Unstamped
                )),
                (idx, kind)
            );
        }
    }

    /// The ROLE-BEARING words, as HAND-BUILT oracles — the other
    /// half of the invariance claim. `input_idx` does not move, the kind stays
    /// in the low bits, and the role occupies EXACTLY bits 14..16, so the
    /// expected words are the pre-role ones plus `0x4000` (drain) / `0x8000`
    /// (body).
    ///
    /// A role encoding that used bits 16..18 would collide with `input_idx`
    /// and fails here on the first line.
    #[test]
    fn read_outcome_meta_role_packing_oracle() {
        assert_eq!(
            pack_read_outcome_meta(0, READ_OUTCOME_SERVED, ReadSiteRole::Drain),
            0x0000_4001
        );
        assert_eq!(
            pack_read_outcome_meta(0, READ_OUTCOME_SERVED, ReadSiteRole::Body),
            0x0000_8001
        );
        assert_eq!(
            pack_read_outcome_meta(2, READ_OUTCOME_TRUNCATED, ReadSiteRole::Drain),
            0x0002_4006
        );
        assert_eq!(
            pack_read_outcome_meta(2, READ_OUTCOME_TRUNCATED, ReadSiteRole::Body),
            0x0002_8006
        );
        assert_eq!(
            pack_read_outcome_meta(u16::MAX, READ_OUTCOME_DECIMATED, ReadSiteRole::Body),
            0xFFFF_8005
        );

        // The FULL inverse, over both halves and every role.
        for idx in [0u16, 1, 0x00AB, u16::MAX] {
            for kind in [
                READ_OUTCOME_SERVED,
                READ_OUTCOME_DRAINED_BATCH,
                READ_OUTCOME_TRUNCATED,
                READ_OUTCOME_PRODUCER,
            ] {
                for role in [
                    ReadSiteRole::Unstamped,
                    ReadSiteRole::Drain,
                    ReadSiteRole::Body,
                    ReadSiteRole::Peek,
                ] {
                    let meta = pack_read_outcome_meta(idx, kind, role);
                    assert_eq!(unpack_read_outcome_meta_full(meta), (idx, kind, role));
                    // And the 2-tuple reader still answers the same
                    // `(idx, kind)` on a ROLE-BEARING word — a reader that
                    // forgot the kind mask reads `kind | 0x4000` here.
                    assert_eq!(unpack_read_outcome_meta(meta), (idx, kind));
                    assert_eq!(read_site_role(meta), role);
                }
            }
        }
    }

    /// The STAGE role's wire codec, round-tripped per
    /// variant — and the SKEW between the two role numberings, stated.
    ///
    /// No test anywhere put a `Drain` STAGE byte on the wire: every capacity-row
    /// fixture in the tree used role byte `0`, so `ReadStageRole::wire()`
    /// returning `0` for BOTH variants would have broken production recording
    /// and failed nothing. The skew half matters for the same reason it is
    /// const-asserted in `read_outcome`: `ReadSiteRole` numbers overlapping
    /// words differently, so a site byte read as a stage byte decodes
    /// `Unstamped` into `Body` in silence.
    #[test]
    fn the_stage_role_wire_codec_round_trips_and_the_two_numberings_are_stated() {
        use crate::read_outcome::{ReadSiteRole, ReadStageRole};
        // Hand-written values, not `role.wire()` compared with itself.
        assert_eq!(ReadStageRole::Body.wire(), 0);
        assert_eq!(ReadStageRole::Drain.wire(), 1);
        for role in [ReadStageRole::Body, ReadStageRole::Drain] {
            assert_eq!(
                ReadStageRole::from_wire(role.wire()),
                Some(role),
                "{role:?} must survive the wire"
            );
        }
        assert_eq!(
            ReadStageRole::from_wire(2),
            None,
            "an unknown byte is refused, never guessed"
        );
        // The SKEW, as it really is: the two enums agree on `Drain` and
        // disagree on `Body`, because a site role needs a zero `Unstamped`.
        assert_eq!(ReadSiteRole::Unstamped as u8, ReadStageRole::Body.wire());
        assert_eq!(ReadSiteRole::Drain as u8, ReadStageRole::Drain.wire());
        assert_ne!(ReadSiteRole::Body as u8, ReadStageRole::Body.wire());
    }

    /// Wire value `3` is the `Peek` SITE (with the format-5 role definition), not
    /// UNSTAMPED, and this arm is where that is pinned. The whole two-bit
    /// field is now spoken for, so the oracle is the FULL map, hand-written:
    /// a reader that masked to one bit, or that kept `3 => Unstamped`, fails
    /// here.
    #[test]
    fn every_wire_role_value_names_a_site() {
        // Built by hand at the bit level, so the decode is checked against the
        // WIRE rather than against the packer that produced it.
        let meta = (u32::from(1u16) << 16) | (0b11 << READ_OUTCOME_ROLE_SHIFT) | 0x0004;
        assert_eq!(read_site_role(meta), ReadSiteRole::Peek);
        assert_eq!(
            unpack_read_outcome_meta(meta),
            (1, READ_OUTCOME_DRAINED_BATCH),
            "the kind half must still decode under a peek role"
        );
        assert_eq!(ReadSiteRole::from_wire(0), ReadSiteRole::Unstamped);
        assert_eq!(ReadSiteRole::from_wire(1), ReadSiteRole::Drain);
        assert_eq!(ReadSiteRole::from_wire(2), ReadSiteRole::Body);
        assert_eq!(ReadSiteRole::from_wire(3), ReadSiteRole::Peek);
        assert!(!ReadSiteRole::Unstamped.is_stamped());
        assert!(ReadSiteRole::Drain.is_stamped());
        assert!(ReadSiteRole::Body.is_stamped());
        assert!(
            ReadSiteRole::Peek.is_stamped(),
            "a peek NAMES a site, so it is believed like any other stamped role \
             — what excludes it from the sync fold is that it is not the HEAD, \
             not that the wire failed to name it"
        );
    }

    /// The kind half is 14 bits, and a value that does not fit is
    /// MASKED rather than allowed to bleed into the role bits — silently
    /// re-labelling a record's SITE is the one corruption a reader steers on.
    ///
    /// Unreachable from production (every kind is a `ReadOutcomeKind`
    /// discriminant <= 7); the release-build behaviour is what is pinned here,
    /// since the debug build asserts first.
    #[test]
    #[cfg(not(debug_assertions))]
    fn an_over_wide_kind_cannot_corrupt_the_role_bits() {
        let meta = pack_read_outcome_meta(1, 0x4004, ReadSiteRole::Drain);
        assert_eq!(read_site_role(meta), ReadSiteRole::Drain);
        assert_eq!(unpack_read_outcome_meta(meta), (1, 0x0004));
    }

    /// The DEBUG half of the same contract, and the one CI actually runs: every
    /// package test in `ci.yml` runs in the debug profile (the only release job
    /// is the push-to-main latency lane), so the release arm above executes on
    /// no pull request and the 14-bit field would be pinned by nothing on the path a
    /// contributor sees.
    ///
    /// SCOPE, because the two arms cover different things and neither
    /// covers the other. THIS arm pins the DEBUG contract: an over-wide kind is
    /// REFUSED loudly rather than silently re-labelling a record's site. It
    /// does NOT pin the mask — deleting `& READ_OUTCOME_KIND_MASK` leaves the
    /// `debug_assert!` untouched, so this test still panics and still passes.
    /// The mask is a RELEASE-only safety net (in debug the assert fires first,
    /// so no debug call can reach the masking with bits 14..16 set), which is
    /// why its own pin is the `cfg(not(debug_assertions))` arm above and stays
    /// there. Run it with `cargo test -p cerulion_core --release --lib
    /// trace_ring`.
    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "must fit the 14-bit field")]
    fn an_over_wide_kind_is_refused_loudly_in_a_debug_build() {
        let _ = pack_read_outcome_meta(1, 0x4004, ReadSiteRole::Drain);
    }

    #[test]
    fn read_outcome_aux_packing_oracle() {
        // Hand oracles: popped LOW 32, high 32 reserved 0.
        assert_eq!(pack_read_outcome_aux(0), 0u64);
        assert_eq!(pack_read_outcome_aux(3), 3u64);
        assert_eq!(pack_read_outcome_aux(u32::MAX), 0x0000_0000_FFFF_FFFF);
        assert_eq!(unpack_read_outcome_popped(3), 3);
        assert_eq!(unpack_read_outcome_popped(0x0000_0000_FFFF_FFFF), u32::MAX);
        // A future format-4 high half must not corrupt the popped read.
        assert_eq!(unpack_read_outcome_popped(0xDEAD_BEEF_0000_0007), 7);
    }

    #[test]
    fn read_outcome_record_byte_oracle_per_kind() {
        // The full 40-byte wire form of one kind-6 record per outcome kind,
        // against hand-written byte oracles (the `record_byte_layout_oracle`
        // discipline). served_seq/popped vary per kind to keep each oracle
        // byte-distinct.
        let cases: [(ReadOutcomeKind, Option<u32>, u32); 5] = [
            (ReadOutcomeKind::Served, Some(7), 1),
            (ReadOutcomeKind::Held, Some(7), 0),
            (ReadOutcomeKind::NoFrame, None, 0),
            (ReadOutcomeKind::DrainedBatch, Some(0x0102_0304), 3),
            (ReadOutcomeKind::Decimated, Some(5), 2),
        ];
        for (kind, seq, popped) in cases {
            // The expected slot value — None packs the NO_FRAME sentinel (the
            // contract: packed INSIDE the assembly site).
            let seq_slot = seq.map(u64::from).unwrap_or(READ_OUTCOME_NO_FRAME);
            // Role-0 — the byte oracle below is the pre-role word.
            let r = TraceRingRecord::read_outcome(
                9,
                2,
                1,
                kind,
                seq,
                ReadRun::once(popped),
                ReadSiteRole::Unstamped,
            );
            let mut expected = [0u8; 40];
            expected[0..8].copy_from_slice(&9u64.to_le_bytes()); // step
            expected[8..16].copy_from_slice(&seq_slot.to_le_bytes()); // served seq
                                                                      // The aux word is BOTH halves — `popped` low, the fold
                                                                      // `run_count` high. A run of ONE writes the
                                                                      // UNFOLDED shape, a structurally ZERO high half, so an unfolded
                                                                      // record stays byte-identical to what every earlier recorder wrote
                                                                      // and a nonzero high half means a REAL fold. A literal `1`
                                                                      // here would make a `CERULION_READ_LOG_FOLD=off` run — which stamps
                                                                      // `trace_format` 5, a format in which those bits do not exist —
                                                                      // contradict its own stamp on every record.
            expected[16..24].copy_from_slice(&u64::from(popped).to_le_bytes()); // aux
            expected[24..28].copy_from_slice(&2u32.to_le_bytes()); // node_idx
            expected[28..32]
                .copy_from_slice(&(0x0001_0000u32 | u32::from(kind.wire())).to_le_bytes());
            expected[32..36].copy_from_slice(&6u32.to_le_bytes()); // kind 6
            expected[36..40].copy_from_slice(&0u32.to_le_bytes()); // reserved
            assert_eq!(
                r.as_bytes(),
                expected,
                "kind-6 byte layout is a format contract (outcome kind {kind:?})"
            );
            // Decode round-trip + field reinterpretation readback.
            let back = TraceRingRecord::from_bytes(&r.as_bytes());
            assert_eq!(back, r);
            assert_eq!(back.record_type, RECORD_TYPE_READ_OUTCOME);
            assert_eq!(
                unpack_read_outcome_meta(back.global_level),
                (1, kind.wire())
            );
            assert_eq!(unpack_read_outcome_popped(back.duration_ns), popped);
            assert_eq!(back.fire_time_ns, seq_slot);
            // A kind-6 record is never a FIRE discard and carries rank 0 on
            // the ring (bagd co-stamps at drain time, uniformly).
            assert!(!back.is_discarded());
            assert_eq!(back.rank(), 0);
        }
    }

    /// The `READ_OUTCOME_*` wire constants are DERIVED from
    /// the portable enum's discriminants; this EXHAUSTIVE match is the drift
    /// guard — adding a `ReadOutcomeKind` variant fails to COMPILE here until
    /// its wire constant is minted, and the asserts pin the pairing.
    #[test]
    fn read_outcome_wire_constants_cover_every_kind_exhaustively() {
        for kind in [
            ReadOutcomeKind::Served,
            ReadOutcomeKind::Held,
            ReadOutcomeKind::NoFrame,
            ReadOutcomeKind::DrainedBatch,
            ReadOutcomeKind::Decimated,
            ReadOutcomeKind::Truncated,
            ReadOutcomeKind::Producer,
        ] {
            let constant = match kind {
                ReadOutcomeKind::Served => READ_OUTCOME_SERVED,
                ReadOutcomeKind::Held => READ_OUTCOME_HELD,
                ReadOutcomeKind::NoFrame => READ_OUTCOME_NONE,
                ReadOutcomeKind::DrainedBatch => READ_OUTCOME_DRAINED_BATCH,
                ReadOutcomeKind::Decimated => READ_OUTCOME_DECIMATED,
                ReadOutcomeKind::Truncated => READ_OUTCOME_TRUNCATED,
                ReadOutcomeKind::Producer => READ_OUTCOME_PRODUCER,
            };
            assert_eq!(kind.wire(), constant);
        }
    }

    #[test]
    fn manifest_with_inputs_byte_oracle_and_roundtrip() {
        // ["cam","fuse"] with inputs [] / ["img","imu"] → the node table
        // exactly as `encode_manifest` writes it, then the additive section.
        let enc =
            encode_manifest_with_inputs(&["cam", "fuse"], &[&[], &["img", "imu"]]).expect("encode");
        let expected: Vec<u8> = vec![
            0x02, 0x00, 0x00, 0x00, // node count = 2
            0x03, 0x00, b'c', b'a', b'm', // "cam"
            0x04, 0x00, b'f', b'u', b's', b'e', // "fuse"
            0x00, 0x00, // cam: 0 inputs
            0x02, 0x00, // fuse: 2 inputs
            0x03, 0x00, b'i', b'm', b'g', // "img"
            0x03, 0x00, b'i', b'm', b'u', // "imu"
        ];
        assert_eq!(
            enc, expected,
            "input-section byte layout is a format contract"
        );
        let (names, inputs) = decode_manifest_with_inputs(&enc).expect("decode");
        assert_eq!(names, vec!["cam".to_string(), "fuse".to_string()]);
        assert_eq!(
            inputs,
            vec![
                Vec::<String>::new(),
                vec!["img".to_string(), "imu".to_string()]
            ]
        );
    }

    /// The additive PUBLISHER section (section 3): byte
    /// oracle, total round trip, and the two back-compat directions.
    #[test]
    fn manifest_section_three_round_trips_and_pre_1289_decodes_empty() {
        // ["cam","fuse"] · inputs [] / ["img"] · publishers [("img", 0x0102…)]
        // / [] — sections 1 and 2 EXACTLY as `encode_manifest_with_inputs`
        // writes them, then section 3.
        const CAM_ID: u128 = 0x0102_0304_0506_0708_090A_0B0C_0D0E_0F10;
        let enc = encode_manifest_with_inputs_and_publishers(
            &["cam", "fuse"],
            &[&[], &["img"]],
            &[&[("img", CAM_ID)], &[]],
        )
        .expect("encode");
        let mut expected: Vec<u8> = vec![
            0x02, 0x00, 0x00, 0x00, // node count = 2
            0x03, 0x00, b'c', b'a', b'm', // "cam"
            0x04, 0x00, b'f', b'u', b's', b'e', // "fuse"
            0x00, 0x00, // cam: 0 inputs
            0x01, 0x00, // fuse: 1 input
            0x03, 0x00, b'i', b'm', b'g', // "img"
            0x01, 0x00, // cam: 1 output
        ];
        expected.extend_from_slice(&CAM_ID.to_le_bytes()); // the 16-byte id
        expected.extend_from_slice(&[0x03, 0x00, b'i', b'm', b'g']); // "img"
        expected.extend_from_slice(&[0x00, 0x00]); // fuse: 0 outputs
        assert_eq!(
            enc, expected,
            "publisher-section byte layout is a format contract"
        );

        let (names, inputs, publishers) =
            decode_manifest_with_inputs_and_publishers(&enc).expect("decode");
        assert_eq!(names, vec!["cam".to_string(), "fuse".to_string()]);
        assert_eq!(inputs, vec![Vec::<String>::new(), vec!["img".to_string()]]);
        assert_eq!(
            publishers,
            vec![vec![("img".to_string(), CAM_ID)], Vec::new()]
        );

        // BACK-COMPAT (a): an older manifest (node table only) and a
        // later one (node table + inputs) BOTH decode to empty publisher
        // lists — the `#[serde(default)]` analogue, one section down.
        for older in [
            encode_manifest(&["cam", "fuse"]).expect("node-only"),
            encode_manifest_with_inputs(&["cam", "fuse"], &[&[], &["img"]]).expect("inputs-only"),
        ] {
            let (n, _i, p) = decode_manifest_with_inputs_and_publishers(&older).expect("decode");
            assert_eq!(n.len(), 2);
            assert_eq!(
                p,
                vec![Vec::<(String, u128)>::new(), Vec::new()],
                "a manifest with no publisher section decodes to EMPTY lists, never an error"
            );
        }

        // BACK-COMPAT (b): the two EARLIER readers over the NEW manifest see
        // exactly their own sections — the trailing publisher section is the
        // tolerated additive region `decode_manifest_with_inputs` reserved.
        assert_eq!(
            decode_manifest(&enc).expect("node-only decode"),
            vec!["cam".to_string(), "fuse".to_string()]
        );
        let (n2, i2) = decode_manifest_with_inputs(&enc).expect("inputs-only decode");
        assert_eq!(n2, vec!["cam".to_string(), "fuse".to_string()]);
        assert_eq!(i2, vec![Vec::<String>::new(), vec!["img".to_string()]]);

        // FAIL-CLOSED: a manifest that STARTS section 3 but truncates
        // mid-way is corrupt — refused loudly, never patched to empty.
        for cut in [enc.len() - 1, enc.len() - 10, enc.len() - 20] {
            let err = decode_manifest_with_inputs_and_publishers(&enc[..cut])
                .expect_err("a truncated publisher section must refuse");
            assert!(
                matches!(err, TraceRingError::ManifestDecode { .. }),
                "expected a loud ManifestDecode, got {err:?}"
            );
        }

        // The parallel-tables contract is refused loudly, like section 2's.
        let err = encode_manifest_with_inputs_and_publishers(&["a", "b"], &[&[], &[]], &[&[]])
            .expect_err("mismatched publisher table must refuse");
        assert!(matches!(
            err,
            TraceRingError::ManifestInputsMismatch {
                nodes: 2,
                inputs: 1
            }
        ));
    }

    /// The manifest DEGRADE ladder, one rung longer. A
    /// publisher section that will not fit costs the PUBLISHER names and
    /// nothing else — the recording still starts, and the input section
    /// survives.
    #[test]
    #[tracing_test::traced_test]
    fn oversized_publisher_section_degrades_to_the_inputs_manifest() {
        // One node, one modest input, and enough outputs to blow the 64 KiB
        // manifest budget on the publisher section ALONE (each entry costs 16
        // id bytes + 2 length bytes + the name).
        let long_output: String = "o".repeat(1000);
        let outputs: Vec<(&str, u128)> =
            (0..80).map(|i| (long_output.as_str(), i as u128)).collect();
        // Precondition: the 3-section encoding really is over budget while
        // the 2-section one is not (else the test proves nothing).
        assert!(matches!(
            encode_manifest_with_inputs_and_publishers(&["n"], &[&["i"]], &[outputs.as_slice()]),
            Err(TraceRingError::ManifestTooLarge { .. })
        ));
        assert!(encode_manifest_with_inputs(&["n"], &[&["i"]]).is_ok());

        let tag = format!("pub_degrade_{}", std::process::id());
        let owner = TraceRingOwner::create_with_inputs_and_publishers_or_degrade(
            &tag,
            64,
            0,
            &["n"],
            &[&["i"]],
            &[outputs.as_slice()],
        )
        .expect("the degrade must not fail the recording");
        assert!(
            logs_contain("read-log PRODUCER names omitted"),
            "the degrade warns loudly, naming what was dropped"
        );
        let consumer = TraceRingConsumer::open(owner.name()).expect("open");
        assert_eq!(consumer.node_ids(), &["n".to_string()]);
        assert_eq!(
            consumer.input_names(),
            &[vec!["i".to_string()]],
            "the INPUT section survives — only the publisher section was dropped"
        );
        assert_eq!(
            consumer.publisher_ids(),
            &[Vec::<(String, u128)>::new()],
            "the dropped publisher section decodes to empty lists (every token \
             resolves `foreign` at replay, loudly)"
        );
    }

    #[test]
    fn manifest_without_input_section_decodes_to_empty_lists() {
        // The serde-default analogue: an older manifest (node table
        // only) decodes with one EMPTY input list per node — old rings need
        // nothing, and a kind-6-less bag carries no input table.
        let old = encode_manifest(&["a", "b"]).expect("encode old form");
        let (names, inputs) = decode_manifest_with_inputs(&old).expect("decode");
        assert_eq!(names, vec!["a".to_string(), "b".to_string()]);
        assert_eq!(inputs, vec![Vec::<String>::new(), Vec::<String>::new()]);
    }

    #[test]
    fn old_decoder_ignores_the_additive_input_section() {
        // Back-compat the OTHER way: a node-table-only reader (`decode_manifest`)
        // over a NEW manifest sees exactly the node table — the trailing
        // input section is the tolerated additive region.
        let enc = encode_manifest_with_inputs(&["a", "b"], &[&["x"], &["y", "z"]]).expect("encode");
        assert_eq!(
            decode_manifest(&enc).expect("old decode"),
            vec!["a".to_string(), "b".to_string()]
        );
    }

    #[test]
    fn manifest_input_section_truncation_fails_closed() {
        // A manifest that STARTS the input section but truncates mid-way is
        // corrupt — refused loudly, never silently patched to empty.
        let full = encode_manifest_with_inputs(&["a"], &[&["long_input_name"]]).expect("encode");
        for cut in [full.len() - 1, full.len() - 8] {
            let err = decode_manifest_with_inputs(&full[..cut]).expect_err("must refuse");
            match err {
                TraceRingError::ManifestDecode { reason } => {
                    assert!(
                        reason.contains("input section truncated"),
                        "got reason: {reason}"
                    );
                }
                other => panic!("expected ManifestDecode, got {other:?}"),
            }
        }
    }

    #[test]
    fn manifest_inputs_length_mismatch_is_refused() {
        let err = encode_manifest_with_inputs(&["a", "b"], &[&["x"]])
            .expect_err("parallel-table mismatch must refuse");
        match err {
            TraceRingError::ManifestInputsMismatch { nodes, inputs } => {
                assert_eq!((nodes, inputs), (2, 1));
            }
            other => panic!("expected ManifestInputsMismatch, got {other:?}"),
        }
    }

    /// The BOTTOM degrade rung marks every section it dropped.
    ///
    /// The rung that drops the INPUT section is the one no reader can recognise:
    /// its manifest decodes to empty inputs AND empty capacities, which is exactly
    /// what a genuinely stageless rank decodes to. The marker is the only thing
    /// that tells them apart, and it lives in the ring HEADER because a marker in
    /// the manifest is a marker this rung can drop.
    ///
    /// Driven through a REAL ring: the inputs are made big enough that the
    /// inputs-bearing encoding cannot fit, which is the only way to reach the rung.
    #[test]
    fn the_bottom_degrade_rung_marks_every_section_it_dropped() {
        let tag = format!("p2b_bottom_{}", std::process::id());
        // One node whose input names cannot fit the manifest budget, so the
        // inputs-bearing encode fails and the ladder falls to the node-only form.
        let big: Vec<String> = (0..4096).map(|i| format!("input_name_{i:040}")).collect();
        let refs: Vec<&str> = big.iter().map(String::as_str).collect();
        let owner = TraceRingOwner::create_with_inputs_or_degrade(
            &tag,
            default_capacity_records(),
            0,
            &["consumer"],
            &[&refs],
        )
        .expect("the node-only form fits, so the ladder degrades rather than failing");
        let name = owner.name().to_string();
        let consumer = TraceRingConsumer::open(&name).expect("open");

        // PREMISE: the rung really fired — the input section is GONE.
        assert_eq!(
            consumer.input_names(),
            &[Vec::<String>::new()],
            "the inputs-bearing encoding must have been dropped, or this arm is \
             testing the happy path"
        );

        // THE PIN: and the header says so, naming all three sections.
        let marker = consumer.degraded_sections();
        assert_ne!(
            marker, 0,
            "a rank that LOST its sections must say so — without the marker it is \
             byte-indistinguishable from a stageless rank and the replay derives \
             its own rims in silence"
        );
        assert!(
            marker & DegradedSections::INPUTS != 0,
            "the INPUT section was dropped: {marker:#b}"
        );
        assert!(
            DegradedSections::affects_read_log(marker),
            "…and that is a read-log section, so the recorder must declare staging"
        );

        // ANTI-TAUTOLOGY: a ring that degraded NOTHING reads 0, so the marker is
        // about the degrade rather than about every ring this code creates.
        let tag2 = format!("p2b_clean_{}", std::process::id());
        let clean = TraceRingOwner::create_with_inputs_or_degrade(
            &tag2,
            default_capacity_records(),
            0,
            &["consumer"],
            &[&["inp"]],
        )
        .expect("a small manifest fits");
        let clean_name = clean.name().to_string();
        let clean_consumer = TraceRingConsumer::open(&clean_name).expect("open");
        assert_eq!(
            clean_consumer.degraded_sections(),
            0,
            "nothing was dropped, so nothing is marked"
        );
        assert_eq!(clean_consumer.input_names(), &[vec!["inp".to_string()]]);
    }

    /// EVERY rung marks what it dropped, and the bits ACCUMULATE
    /// when the ladder falls more than one step.
    ///
    /// The bottom rung has its own arm above. This covers the two upper rungs and
    /// the accumulation, which is the part a post-create `fetch_or` got right by
    /// accident and a create-time parameter has to get right on purpose: each rung
    /// passes its bit DOWN, so whichever rung finally creates the ring creates it
    /// with the whole set.
    #[test]
    fn every_degrade_rung_marks_its_own_section_and_the_bits_accumulate() {
        let pid = std::process::id();
        // A capacity table too big for the budget, with small inputs/publishers:
        // the CAPACITIES rung fires and the one below it does not.
        let many_rows: Vec<(u16, u8, u32)> = (0..20_000).map(|i| (i as u16, 0, 64)).collect();
        let owner = TraceRingOwner::create_with_inputs_publishers_and_capacities_or_degrade(
            &format!("p2b_caps_{pid}"),
            default_capacity_records(),
            0,
            &["consumer"],
            &[&["inp"]],
            &[&[]],
            &[&many_rows],
        )
        .expect("the publishers-bearing form fits, so the ladder degrades one step");
        let c = TraceRingConsumer::open(owner.name()).expect("open");
        assert_eq!(
            c.degraded_sections(),
            DegradedSections::CAPACITIES,
            "exactly the capacity section was dropped — the INPUT section survives \
             (and is what the recorder's discriminator reads): {:?}",
            c.input_names()
        );
        assert_eq!(
            c.input_names(),
            &[vec!["inp".to_string()]],
            "the rung below did not fire"
        );
        assert!(DegradedSections::affects_read_log(c.degraded_sections()));

        // Now make BOTH the capacities and the publishers too big, with inputs
        // small enough to survive: two rungs fire, and BOTH bits must be set.
        let many_pubs: Vec<(String, u128)> = (0..4_000)
            .map(|i| (format!("out_{i:060}"), i as u128))
            .collect();
        let pub_refs: Vec<(&str, u128)> = many_pubs.iter().map(|(n, i)| (n.as_str(), *i)).collect();
        let owner = TraceRingOwner::create_with_inputs_publishers_and_capacities_or_degrade(
            &format!("p2b_accum_{pid}"),
            default_capacity_records(),
            0,
            &["consumer"],
            &[&["inp"]],
            &[&pub_refs],
            &[&many_rows],
        )
        .expect("the inputs-bearing form fits, so the ladder degrades two steps");
        let c = TraceRingConsumer::open(owner.name()).expect("open");
        let marker = c.degraded_sections();
        assert_eq!(
            marker,
            DegradedSections::CAPACITIES | DegradedSections::PUBLISHERS,
            "TWO rungs fired, so TWO bits are set — an outer rung that OVERWROTE \
             instead of accumulating would report only its own: {marker:#b}"
        );
        assert_eq!(
            c.input_names(),
            &[vec!["inp".to_string()]],
            "…and the input section still survives"
        );
    }

    /// A PUBLISHERS-ONLY degrade declares nothing about staging.
    ///
    /// The marker is read for what it MEANS, not as a boolean, and this is the
    /// scenario that distinguishes the two on a REAL ring. The publisher section is
    /// offline producer attribution: losing it costs token resolution, not staging
    /// rims. A recorder that treated any degrade as "this rank has stages" would
    /// make a genuinely STAGELESS rank start claiming a staging regime it never had,
    /// and stand the whole read log down for nothing.
    #[test]
    fn a_publishers_only_degrade_says_nothing_about_the_read_log() {
        // Publishers too big, inputs and capacities small: the PUBLISHERS rung fires
        // and the rung below it does not.
        let many_pubs: Vec<(String, u128)> = (0..4_000)
            .map(|i| (format!("out_{i:060}"), i as u128))
            .collect();
        let pub_refs: Vec<(&str, u128)> = many_pubs.iter().map(|(n, i)| (n.as_str(), *i)).collect();
        let owner = TraceRingOwner::create_with_inputs_and_publishers_or_degrade(
            &format!("f5_pubs_{}", std::process::id()),
            default_capacity_records(),
            0,
            &["consumer"],
            &[&["inp"]],
            &[&pub_refs],
        )
        .expect("the inputs-bearing form fits, so the ladder degrades one step");
        let c = TraceRingConsumer::open(owner.name()).expect("open");

        // PREMISE: the rung really fired — the publisher section is gone…
        assert_eq!(
            c.publisher_ids(),
            &[Vec::<(String, u128)>::new()],
            "the publishers-bearing encoding must have been dropped"
        );
        // …and the INPUT section survived, so nothing about staging was lost.
        assert_eq!(c.input_names(), &[vec!["inp".to_string()]]);

        // THE PIN: marked, but NOT as a read-log matter.
        assert_eq!(
            c.degraded_sections(),
            DegradedSections::PUBLISHERS,
            "exactly the publisher section is marked"
        );
        assert!(
            !DegradedSections::affects_read_log(c.degraded_sections()),
            "a publishers-only degrade must NOT make the recorder declare staging — \
             that would stand a stageless rank's read log down for a loss that has \
             nothing to do with it"
        );
    }

    /// A CAPACITY-table length mismatch names the CAPACITY
    /// section, not the input one.
    ///
    /// The check existed and reported itself as `ManifestInputsMismatch`, so an
    /// operator read "input table has N entries but the node table has M" and went
    /// looking at a section that was fine. The sections are parallel to the node
    /// table INDEPENDENTLY; a diagnostic that names the wrong one costs more than
    /// the variant it saved. This arm pins the variant.
    #[test]
    fn manifest_capacities_length_mismatch_names_the_capacity_section() {
        let rows: &[&[(u16, u8, u32)]] = &[&[(0, 0, 22)]];
        let err = encode_manifest_with_inputs_publishers_and_capacities(
            &["a", "b"],
            &[&[], &[]],
            &[&[], &[]],
            rows, // ONE list for TWO nodes
        )
        .expect_err("a capacity table that is not parallel to the node table must refuse");
        match err {
            TraceRingError::ManifestCapacitiesMismatch { nodes, capacities } => {
                assert_eq!((nodes, capacities), (2, 1));
                // …and the rendered message says CAPACITY, which is the whole point
                // of the variant.
                let text = format!("{err}");
                assert!(
                    text.contains("capacity table"),
                    "the message must name the section that is wrong: {text}"
                );
                assert!(
                    !text.contains("input table"),
                    "…and must not send the reader to the input section: {text}"
                );
            }
            other => panic!("expected ManifestCapacitiesMismatch, got {other:?}"),
        }
        // The ANTI-TAUTOLOGY half: an INPUT-table mismatch still reports itself as
        // one, so the new variant did not capture its sibling's case.
        let err = encode_manifest_with_inputs_publishers_and_capacities(
            &["a", "b"],
            &[&[]],
            &[&[], &[]],
            &[&[], &[]],
        )
        .expect_err("an input-table mismatch must still refuse");
        assert!(
            matches!(err, TraceRingError::ManifestInputsMismatch { .. }),
            "got {err:?}"
        );
    }

    #[test]
    fn manifest_empty_roundtrip_and_oracle() {
        let enc = encode_manifest(&[]).expect("encode empty");
        assert_eq!(enc, vec![0, 0, 0, 0], "empty table = count 0");
        assert_eq!(decode_manifest(&enc).expect("decode"), Vec::<String>::new());
    }

    #[test]
    fn manifest_small_byte_oracle() {
        // ["a","bc"] → count 2, then (1,"a"), (2,"bc").
        let enc = encode_manifest(&["a", "bc"]).expect("encode");
        let expected: Vec<u8> = vec![
            0x02, 0x00, 0x00, 0x00, // count = 2
            0x01, 0x00, b'a', // "a"
            0x02, 0x00, b'b', b'c', // "bc"
        ];
        assert_eq!(enc, expected, "manifest byte layout is a format contract");
        assert_eq!(
            decode_manifest(&enc).expect("decode"),
            vec!["a".to_string(), "bc".to_string()]
        );
    }

    #[test]
    fn manifest_roundtrips_single_many_and_non_ascii() {
        for ids in [
            vec!["only"],
            vec!["camera", "imu", "lidar", "fusion"],
            vec!["café", "日本語", "emoji_🚀"],
        ] {
            let enc = encode_manifest(&ids).expect("encode");
            let dec = decode_manifest(&enc).expect("decode");
            let expect: Vec<String> = ids.iter().map(|s| s.to_string()).collect();
            assert_eq!(dec, expect, "decode(encode(x)) == x for {ids:?}");
        }
    }

    #[test]
    fn manifest_oversize_is_rejected() {
        // Many long ids so the encoded size exceeds MANIFEST_CAPACITY (64 KiB).
        let big = "x".repeat(1000);
        let ids: Vec<&str> = (0..100).map(|_| big.as_str()).collect();
        let err = encode_manifest(&ids).expect_err("must reject oversize");
        match err {
            TraceRingError::ManifestTooLarge { needed, capacity } => {
                assert!(needed > capacity, "needed {needed} > capacity {capacity}");
                assert_eq!(capacity, MANIFEST_CAPACITY);
            }
            other => panic!("expected ManifestTooLarge, got {other:?}"),
        }
    }

    /// `create_with_inputs_or_degrade`: the
    /// never-block-degrade-loudly posture. Input names sized to blow the
    /// 64 KiB manifest budget while the NODE table alone fits comfortably:
    /// the strict form REFUSES (the startup failure this entry point
    /// exists to remove — a graph whose node-only manifest recorded fine
    /// before input names existed must not fail recording because they were added),
    /// the degrade form RECORDS — one loud warn — and the ring's manifest
    /// decodes NODE-ONLY (names intact, EMPTY input lists: exactly the shape
    /// the replay read-log verifier's Disabled arm stands down loudly for).
    #[test]
    #[tracing_test::traced_test]
    fn oversized_input_section_degrades_to_a_node_only_manifest_with_one_warn() {
        let tag = format!("degrade_{}", std::process::id());
        let node_ids = ["alpha", "beta"];
        // Two ~33 KiB input names on one node: the input section alone is
        // ~66 KiB > MANIFEST_CAPACITY (64 KiB); the node table is ~20 bytes.
        let big_a = "a".repeat(33_000);
        let big_b = "b".repeat(33_000);
        let alpha_inputs: Vec<&str> = vec![big_a.as_str(), big_b.as_str()];
        let beta_inputs: Vec<&str> = Vec::new();
        let inputs: Vec<&[&str]> = vec![&alpha_inputs, &beta_inputs];

        // The strict form refuses — the compatibility regression this entry
        // point absorbs. The encode fails BEFORE any SHM create, so the tag
        // stays free for the degrade attempt below.
        let err = TraceRingOwner::create_with_inputs(&tag, 64, 0, &node_ids, &inputs)
            .expect_err("the strict form must refuse the oversized manifest");
        assert!(
            matches!(err, TraceRingError::ManifestTooLarge { .. }),
            "the refusal is the budget error: {err:?}"
        );

        // The degrade form records.
        let owner = TraceRingOwner::create_with_inputs_or_degrade(&tag, 64, 0, &node_ids, &inputs)
            .expect("the degrade form must create the ring, not refuse");
        assert!(
            logs_contain("read-log input names omitted"),
            "the degrade warns loudly, naming what was dropped"
        );

        // The ring's manifest is NODE-ONLY: names intact, input lists empty.
        let consumer = TraceRingConsumer::open(owner.name()).expect("open the degraded ring");
        assert_eq!(
            consumer.node_ids(),
            &["alpha".to_string(), "beta".to_string()],
            "the node table survives the degrade intact"
        );
        assert_eq!(
            consumer.input_names(),
            &[Vec::<String>::new(), Vec::<String>::new()],
            "kind-6 records resolve against EMPTY input lists (the verifier's Disabled shape)"
        );
    }

    /// Control for the degrade arm: a manifest that FITS keeps its inputs
    /// through `create_with_inputs_or_degrade` (behavior identical to the
    /// strict form) and warns NOTHING.
    #[test]
    #[tracing_test::traced_test]
    fn fitting_manifest_keeps_its_inputs_through_the_degrade_entry_point() {
        let tag = format!("degrade_ctl_{}", std::process::id());
        let node_ids = ["alpha", "beta"];
        let alpha_inputs: Vec<&str> = vec!["cam", "imu"];
        let beta_inputs: Vec<&str> = Vec::new();
        let inputs: Vec<&[&str]> = vec![&alpha_inputs, &beta_inputs];

        let owner = TraceRingOwner::create_with_inputs_or_degrade(&tag, 64, 0, &node_ids, &inputs)
            .expect("a fitting manifest creates normally");
        assert!(
            !logs_contain("read-log input names omitted"),
            "no degrade happened, so no degrade warn may fire"
        );
        let consumer = TraceRingConsumer::open(owner.name()).expect("open the ring");
        assert_eq!(
            consumer.node_ids(),
            &["alpha".to_string(), "beta".to_string()]
        );
        assert_eq!(
            consumer.input_names(),
            &[
                vec!["cam".to_string(), "imu".to_string()],
                Vec::<String>::new()
            ],
            "the input section survives intact when it fits"
        );
    }

    #[test]
    fn manifest_decode_rejects_truncated_and_bad_utf8() {
        // Too short for the 4-byte count.
        assert!(decode_manifest(&[0, 0]).is_err());
        // count=1 but no length prefix.
        assert!(decode_manifest(&[1, 0, 0, 0]).is_err());
        // count=1, len=5, but only 2 bytes follow.
        assert!(decode_manifest(&[1, 0, 0, 0, 5, 0, b'a', b'b']).is_err());
        // count=1, len=2, invalid UTF-8 bytes (0xFF 0xFE).
        assert!(decode_manifest(&[1, 0, 0, 0, 2, 0, 0xFF, 0xFE]).is_err());
    }

    /// Regression pin: a hostile/corrupt count (u32::MAX from
    /// untrusted SHM bytes) with an empty body must error INSTANTLY at node 0 —
    /// without the cap, `Vec::with_capacity(u32::MAX)` requests a ~100 GB reserve and
    /// aborts via `handle_alloc_error` BEFORE the loop's truncation guard can
    /// fire. The reserve is capped by what the input could physically hold
    /// (≥ 2 bytes per entry).
    #[test]
    fn manifest_decode_hostile_count_errs_instead_of_aborting() {
        // count = u32::MAX, no body.
        let err = decode_manifest(&[0xFF, 0xFF, 0xFF, 0xFF]).expect_err("hostile count must err");
        match err {
            TraceRingError::ManifestDecode { reason } => {
                assert!(
                    reason.contains("truncated at node 0"),
                    "errs at node 0 (no body at all): {reason}"
                );
            }
            other => panic!("expected ManifestDecode, got {other:?}"),
        }

        // Boundary control: a count that lies only SLIGHTLY (count=3, body holds
        // 2 entries) still errs at exactly node 2 — the loop's existing
        // truncation behavior is preserved by the capped reserve.
        let mut bytes = vec![3u8, 0, 0, 0]; // count = 3
        bytes.extend_from_slice(&[1, 0, b'a']); // node 0: "a"
        bytes.extend_from_slice(&[1, 0, b'b']); // node 1: "b"
        let err = decode_manifest(&bytes).expect_err("count=3 with 2 entries must err");
        match err {
            TraceRingError::ManifestDecode { reason } => {
                assert!(
                    reason.contains("truncated at node 2"),
                    "errs at the first missing node (2): {reason}"
                );
            }
            other => panic!("expected ManifestDecode, got {other:?}"),
        }
    }

    // =======================================================================
    // The partial-head-step rule (pure oracle vectors).
    //
    // Each arm drives a HAND-WRITTEN record sequence through the gate and
    // asserts the admit verdict PER RECORD plus the three observables
    // (is_open / discarded / first_step_recorded) — never a self-compare.
    // =======================================================================

    /// A `FIRE` record for `step` (the fields the gate does not read are left at
    /// their defaults — the gate keys only on `record_type` and `step`).
    fn fire(step: u64) -> TraceRingRecord {
        TraceRingRecord {
            step,
            record_type: RECORD_TYPE_FIRE,
            ..TraceRingRecord::default()
        }
    }

    /// A `STEP_BOUNDARY` record for `step`.
    fn boundary(step: u64) -> TraceRingRecord {
        TraceRingRecord {
            step,
            record_type: RECORD_TYPE_STEP_BOUNDARY,
            ..TraceRingRecord::default()
        }
    }

    /// A `DEPARTURE` record (never a step head).
    fn departure(step: u64) -> TraceRingRecord {
        TraceRingRecord {
            step,
            record_type: RECORD_TYPE_DEPARTURE,
            ..TraceRingRecord::default()
        }
    }

    /// Run `records` through `gate`, returning the per-record admit verdicts.
    fn verdicts(gate: &mut HeadStepGate, records: &[TraceRingRecord]) -> Vec<bool> {
        records.iter().map(|r| gate.admit(r)).collect()
    }

    #[test]
    fn an_armed_gate_discards_the_headless_fires_and_admits_the_boundary_that_opens_it() {
        // A mid-run attach lands mid-step 4: two of step 4's FIREs arrive with
        // no boundary, then step 5 begins.
        let mut gate = HeadStepGate::armed();
        assert!(!gate.is_open(), "an armed gate starts closed");
        let got = verdicts(
            &mut gate,
            &[fire(4), fire(4), boundary(5), fire(5), fire(5), boundary(6)],
        );
        assert_eq!(
            got,
            vec![false, false, true, true, true, true],
            "discard until the first boundary; the boundary itself is ADMITTED"
        );
        assert_eq!(gate.discarded(), 2, "exactly the two headless fires");
        assert_eq!(
            gate.first_step_recorded(),
            Some(5),
            "the first COMPLETE step is the one the admitted boundary names"
        );
        assert!(gate.is_open());
    }

    #[test]
    fn an_armed_gate_that_attaches_exactly_at_a_boundary_discards_nothing() {
        // The boundary case: the attach cursor lands on a step head.
        let mut gate = HeadStepGate::armed();
        let got = verdicts(&mut gate, &[boundary(9), fire(9), fire(9)]);
        assert_eq!(got, vec![true, true, true], "nothing to discard");
        assert_eq!(gate.discarded(), 0);
        assert_eq!(gate.first_step_recorded(), Some(9));
    }

    #[test]
    fn a_passthrough_gate_admits_a_headless_fire_that_an_armed_gate_discards() {
        // The SAME vector through both gates — the only difference is the
        // constructor, so this is the arm that separates the two modes.
        let vector = [fire(4), boundary(5), fire(5)];

        let mut pass = HeadStepGate::passthrough();
        assert!(pass.is_open(), "a passthrough gate starts open");
        assert_eq!(
            verdicts(&mut pass, &vector),
            vec![true, true, true],
            "an at-zero attach admits every record"
        );
        assert_eq!(pass.discarded(), 0);
        assert_eq!(
            pass.first_step_recorded(),
            Some(5),
            "passthrough still dates the first boundary it sees"
        );

        let mut armed = HeadStepGate::armed();
        assert_eq!(
            verdicts(&mut armed, &vector),
            vec![false, true, true],
            "the armed gate drops the headless fire the passthrough gate keeps"
        );
        assert_eq!(armed.discarded(), 1);
    }

    #[test]
    fn an_armed_gate_over_a_boundary_less_stream_admits_nothing_and_claims_no_step() {
        // The DEPARTURE-ring hazard, pinned rather than described: a ring that
        // carries no STEP_BOUNDARY leaves an armed gate closed forever. A caller
        // must not arm one over such a ring (see the `HeadStepGate` doc's Scope section).
        let mut gate = HeadStepGate::armed();
        let got = verdicts(&mut gate, &[fire(1), departure(1), fire(2), departure(2)]);
        assert_eq!(
            got,
            vec![false; 4],
            "no boundary ⇒ nothing is ever admitted"
        );
        assert_eq!(gate.discarded(), 4);
        assert_eq!(
            gate.first_step_recorded(),
            None,
            "no boundary was admitted, so no step may be claimed complete"
        );
        assert!(!gate.is_open());
    }

    #[test]
    fn the_first_complete_step_is_the_first_admitted_boundary_not_a_later_one() {
        let mut gate = HeadStepGate::armed();
        let got = verdicts(&mut gate, &[fire(6), boundary(7), fire(7), boundary(8)]);
        assert_eq!(got, vec![false, true, true, true]);
        assert_eq!(
            gate.first_step_recorded(),
            Some(7),
            "later boundaries never move the first-complete-step report"
        );
    }

    // =======================================================================
    // `skip_leading` — the gate over a RAW drained span.
    //
    // `bagd` drains rings zero-copy, so the production recorder never builds a
    // `Vec<TraceRingRecord>` and cannot use `drain_gated`. These arms pin that
    // the byte-facing form agrees with the record-facing one, and the
    // MONOTONICITY that makes a leading-skip count a faithful encoding at all.
    // =======================================================================

    /// Encode `records` into the two halves `drain_slices` would return, split
    /// at `at` records — so an arm can put the boundary in the SECOND half and
    /// prove the walk really crosses the seam.
    fn span(records: &[TraceRingRecord], at: usize) -> (Vec<u8>, Vec<u8>) {
        let bytes: Vec<u8> = records.iter().flat_map(|r| r.as_bytes()).collect();
        let cut = at * TRACE_RECORD_SIZE as usize;
        (bytes[..cut].to_vec(), bytes[cut..].to_vec())
    }

    #[test]
    fn skip_leading_counts_the_headless_prefix_across_both_span_halves() {
        // The wrap-around shape: the partial head step is split across the
        // ring's two drained halves, and the opening boundary lands in the
        // SECOND — so a walk that stopped at the first half would under-count.
        let mut gate = HeadStepGate::armed();
        let records = [fire(4), fire(4), fire(4), boundary(5), fire(5)];
        let (a, b) = span(&records, 2);
        assert_eq!(
            gate.skip_leading(&a, &b),
            3,
            "three headless fires lead the span, one of them past the seam"
        );
        assert_eq!(gate.discarded(), 3);
        assert_eq!(gate.first_step_recorded(), Some(5));
        assert!(gate.is_open());
    }

    #[test]
    fn skip_leading_agrees_with_the_record_facing_gate_on_the_same_stream() {
        // The two forms must not drift: one rule, two shapes. Driven over a
        // stream whose verdicts are known by hand (the record-facing arms above), with the
        // BYTE form's answer checked against the RECORD form's own count.
        let records = [fire(4), fire(4), boundary(5), fire(5), boundary(6)];

        let mut by_record = HeadStepGate::armed();
        let verdicts = verdicts(&mut by_record, &records);
        assert_eq!(
            verdicts,
            vec![false, false, true, true, true],
            "hand oracle: the two headless fires go, the rest stay"
        );

        let mut by_bytes = HeadStepGate::armed();
        let (a, b) = span(&records, records.len());
        let skipped = by_bytes.skip_leading(&a, &b);
        assert_eq!(
            skipped,
            verdicts.iter().filter(|v| !**v).count(),
            "the byte form skips exactly the records the record form refused"
        );
        assert_eq!(by_bytes.discarded(), by_record.discarded());
        assert_eq!(
            by_bytes.first_step_recorded(),
            by_record.first_step_recorded()
        );
        assert_eq!(by_bytes.is_open(), by_record.is_open());
    }

    #[test]
    fn admit_is_monotone_so_a_leading_skip_is_a_faithful_encoding() {
        // THE property `skip_leading` rests on: the admitted set is a SUFFIX.
        // If `admit` could ever return false AFTER returning true, "skip the
        // leading N" would silently keep records the gate had refused, and no
        // other arm in this file would notice.
        let mut gate = HeadStepGate::armed();
        let stream = [
            fire(1),
            departure(1),
            boundary(2),
            fire(2),
            departure(2),
            fire(2),
            boundary(3),
            fire(3),
        ];
        let mut seen_admit = false;
        for record in &stream {
            let admitted = gate.admit(record);
            if admitted {
                seen_admit = true;
            } else {
                assert!(
                    !seen_admit,
                    "admit REFUSED {record:?} after already admitting — the admitted \
                     set is not a suffix, so `skip_leading`'s count would keep \
                     records the gate rejected"
                );
            }
        }
        assert!(
            seen_admit,
            "the stream must reach an admitted record at all"
        );
    }

    #[test]
    fn skip_leading_over_a_span_with_no_boundary_skips_it_whole_and_stays_armed() {
        // The first drain of a mid-run attach that lands deep inside a long
        // step: nothing is admitted, and the gate must carry its state into the
        // NEXT span rather than resetting.
        let mut gate = HeadStepGate::armed();
        let (a, b) = span(&[fire(4), fire(4)], 1);
        assert_eq!(gate.skip_leading(&a, &b), 2, "the whole span is the head");
        assert!(!gate.is_open(), "still waiting for a boundary");
        assert_eq!(gate.discarded(), 2);

        let (a2, b2) = span(&[fire(4), boundary(5), fire(5)], 0);
        assert_eq!(
            gate.skip_leading(&a2, &b2),
            1,
            "the next span resumes where the last left off — one more headless fire"
        );
        assert_eq!(gate.discarded(), 3, "the discard count ACCUMULATES");
        assert_eq!(gate.first_step_recorded(), Some(5));
    }

    #[test]
    fn skip_leading_on_an_open_gate_skips_nothing_and_still_learns_its_first_step() {
        // The steady state, and the passthrough (departure-ring) case in one
        // body: an open gate admits every span whole, and a passthrough gate
        // that has never seen a boundary still reports one when it arrives.
        let mut open = HeadStepGate::armed();
        let (a, b) = span(&[boundary(5), fire(5)], 1);
        assert_eq!(open.skip_leading(&a, &b), 0);
        let (a2, b2) = span(&[fire(5), fire(5), boundary(6)], 2);
        assert_eq!(open.skip_leading(&a2, &b2), 0, "an open gate never skips");
        assert_eq!(
            open.first_step_recorded(),
            Some(5),
            "and never re-stamps its first complete step"
        );

        let mut pass = HeadStepGate::passthrough();
        let (a3, b3) = span(&[fire(9), boundary(10), fire(10)], 1);
        assert_eq!(
            pass.skip_leading(&a3, &b3),
            0,
            "a passthrough gate keeps the headless fire — that is what it is for"
        );
        assert_eq!(pass.discarded(), 0);
        assert_eq!(
            pass.first_step_recorded(),
            Some(10),
            "it still reports the first boundary it saw"
        );
    }

    #[test]
    fn skip_leading_ignores_a_partial_trailing_record() {
        // A span whose length is not a whole multiple of the record size: the
        // caller's own record arithmetic governs what it writes, so the gate
        // must not decode — or count — a record the caller will never see.
        let mut gate = HeadStepGate::armed();
        let (a, _) = span(&[fire(4), boundary(5)], 2);
        let truncated = &a[..a.len() - 7];
        assert_eq!(
            gate.skip_leading(truncated, &[]),
            1,
            "only the ONE whole record is judged"
        );
        assert!(
            !gate.is_open(),
            "the half-written boundary must not open the gate"
        );
    }

    #[test]
    fn default_capacity_is_power_of_two_and_fits_budget() {
        let cap = default_capacity_records();
        assert!(
            cap.is_power_of_two(),
            "capacity {cap} must be a power of two"
        );
        assert_eq!(cap, 1 << 20, "64 MiB / 40 B rounded down to 2^20");
        // The rounded-down data region fits within the budget.
        assert!((cap as usize) * TRACE_RECORD_SIZE as usize <= DEFAULT_TRACE_RING_BYTES);
    }

    #[test]
    fn prev_power_of_two_edges() {
        assert_eq!(prev_power_of_two(0), 0);
        assert_eq!(prev_power_of_two(1), 1);
        assert_eq!(prev_power_of_two(2), 2);
        assert_eq!(prev_power_of_two(3), 2);
        assert_eq!(prev_power_of_two(1_677_721), 1 << 20);
    }
    /// The aux word carries BOTH halves, and the low half is
    /// byte-unchanged so every pre-existing `popped` oracle still holds.
    #[test]
    fn the_run_count_rides_the_aux_high_half_and_popped_is_byte_unchanged() {
        // Hand-built words: the oracle is arithmetic, not the packer's own
        // expression (a packer bug and a reader bug would otherwise cancel).
        assert_eq!(pack_read_outcome_aux_run(3, 7), 3 | (7u64 << 32));
        assert_eq!(pack_read_outcome_aux_run(u32::MAX, u32::MAX), u64::MAX);
        // A run of ONE writes the legacy shape (a
        // structurally zero high half), so an unfolded record is byte-identical
        // to what every earlier recorder wrote, a nonzero high half means a REAL
        // fold, and a `CERULION_READ_LOG_FOLD=off` run (which stamps
        // `trace_format` 5, a format in which those bits do not exist) cannot
        // contradict its own stamp.
        assert_eq!(pack_read_outcome_aux_run(0, 1), 0);
        assert_eq!(pack_read_outcome_aux_run(9, 1), 9);
        assert_eq!(
            pack_read_outcome_aux_run(9, 0),
            pack_read_outcome_aux_run(9, 1),
            "`0` and `1` are the same one occurrence, and are written the same way"
        );
        assert_eq!(pack_read_outcome_aux_run(9, 2), 9 | (2u64 << 32));

        // The LOW half is exactly what it always was.
        for popped in [0u32, 1, 3, u32::MAX] {
            for run in [0u32, 1, 2, u32::MAX] {
                let aux = pack_read_outcome_aux_run(popped, run);
                assert_eq!(
                    unpack_read_outcome_popped(aux),
                    popped,
                    "the popped half is untouched by the count"
                );
            }
        }
        // And a legacy word (high half structurally zero) still reads
        // its popped count AND decodes as ONE occurrence.
        let legacy = pack_read_outcome_aux(5);
        assert_eq!(unpack_read_outcome_popped(legacy), 5);
        assert_eq!(
            unpack_read_outcome_run(legacy),
            1,
            "0 == 1: a format-<=5 record stands for exactly one read, which is \
             what makes the high half an ADDITIVE section rather than a break"
        );
    }

    /// The `0 == 1` convention, stated on its own because it is what every
    /// older bag depends on.
    #[test]
    fn a_zero_run_count_decodes_as_one() {
        assert_eq!(unpack_read_outcome_run(0), 1);
        assert_eq!(
            unpack_read_outcome_run(12345),
            1,
            "low bits are popped, not a count"
        );
        assert_eq!(unpack_read_outcome_run(1u64 << 32), 1);
        assert_eq!(unpack_read_outcome_run(9u64 << 32), 9);
    }

    /// The manifest's fourth additive section — the per-stage
    /// staging rims the replay ADOPTS — round-trips, is optional, and refuses a
    /// truncation loudly.
    ///
    /// The rows are keyed on `(input_idx, role)`, never on position, so the
    /// oracle deliberately gives ONE node two rows that SHARE an input index
    /// (the Separate/legacy-`Sync` shape, where a body and a drain stage sit on
    /// the same input) and another node zero rows.
    #[test]
    fn manifest_capacity_section_round_trips_and_is_optional() {
        const CAM_ID: u128 = 0x1234_5678_9abc_def0_1122_3344_5566_7788;
        let rows_fuse: &[(u16, u8, u32)] = &[(0, 0, 22), (0, 1, 130), (1, 0, 4096)];
        let enc = encode_manifest_with_inputs_publishers_and_capacities(
            &["cam", "fuse"],
            &[&[], &["img", "imu"]],
            &[&[("img", CAM_ID)], &[]],
            &[&[], rows_fuse],
        )
        .expect("encode");
        let (names, inputs, publishers, caps) =
            decode_manifest_with_inputs_publishers_and_capacities(&enc).expect("decode");
        assert_eq!(names, vec!["cam".to_string(), "fuse".to_string()]);
        assert_eq!(
            inputs,
            vec![
                Vec::<String>::new(),
                vec!["img".to_string(), "imu".to_string()]
            ]
        );
        assert_eq!(
            publishers,
            vec![vec![("img".to_string(), CAM_ID)], Vec::new()]
        );
        assert_eq!(
            caps,
            vec![Vec::new(), rows_fuse.to_vec()],
            "the rows survive verbatim, INCLUDING the two that share input_idx 0 — a decoder \
             that keyed on position instead of on (input_idx, role) would collapse them"
        );

        // A DUPLICATE `(input_idx, role)` is refused
        // at BOTH binary ends. The row key is the pair, and every reader builds
        // a map on it, so a duplicate silently picks whichever row landed last
        // — a rim the recording may never have used. Neither guard had a test.
        let dup: &[(u16, u8, u32)] = &[(0, 0, 22), (0, 0, 99)];
        let err = encode_manifest_with_inputs_publishers_and_capacities(
            &["cam", "fuse"],
            &[&[], &["img", "imu"]],
            &[&[("img", CAM_ID)], &[]],
            &[&[], dup],
        )
        .expect_err("the ENCODER must refuse a duplicate row key");
        let text = format!("{err}");
        assert!(
            text.contains("twice"),
            "the refusal names the condition: {text}"
        );
        // The DECODER refuses one too — the encoder cannot produce this, so the
        // bytes are built by hand (a corrupt or hand-edited ring).
        let valid = encode_manifest_with_inputs_publishers_and_capacities(
            &["cam", "fuse"],
            &[&[], &["img", "imu"]],
            &[&[("img", CAM_ID)], &[]],
            &[&[], &[(0, 0, 22), (0, 1, 130)]],
        )
        .expect("encode the valid twin");
        let mut corrupt = valid.clone();
        // The last row's ROLE byte, rewritten to collide with the row before it.
        // Located by SEARCH rather than by a hardcoded offset, so a section
        // layout change fails this test loudly instead of mutating a random
        // byte: the valid tail is `(0,1,130)`, the collision is `(0,0,130)`.
        let tail: [u8; 7] = {
            let mut b = [0u8; 7];
            b[0..2].copy_from_slice(&0u16.to_le_bytes());
            b[2] = 1;
            b[3..7].copy_from_slice(&130u32.to_le_bytes());
            b
        };
        let at = corrupt
            .windows(7)
            .rposition(|w| w == tail)
            .expect("the valid tail row is in the encoded bytes");
        corrupt[at + 2] = 0;
        let err = decode_manifest_with_inputs_publishers_and_capacities(&corrupt)
            .expect_err("the DECODER must refuse a duplicate row key");
        assert!(
            format!("{err}").contains("twice"),
            "the refusal names the condition: {err}"
        );

        // BACK-COMPAT: every older manifest decodes to empty capacity rows —
        // the `#[serde(default)]` analogue, one section further down.
        for older in [
            encode_manifest(&["cam", "fuse"]).expect("node-only"),
            encode_manifest_with_inputs(&["cam", "fuse"], &[&[], &["img"]]).expect("inputs-only"),
            encode_manifest_with_inputs_and_publishers(
                &["cam", "fuse"],
                &[&[], &["img"]],
                &[&[("img", CAM_ID)], &[]],
            )
            .expect("publishers-only"),
        ] {
            let (n, _i, _p, c) =
                decode_manifest_with_inputs_publishers_and_capacities(&older).expect("decode");
            assert_eq!(n.len(), 2);
            assert_eq!(
                c,
                vec![Vec::new(), Vec::new()],
                "a manifest with no capacity section declares NO rims — which is not the same \
                 as declaring a rim of zero"
            );
        }

        // A manifest that STARTS the section and truncates mid-row is refused:
        // it claimed the section, so a silent empty would be a lie.
        let mut torn = enc.clone();
        torn.truncate(enc.len() - 3);
        let err = decode_manifest_with_inputs_publishers_and_capacities(&torn)
            .expect_err("a torn capacity section must be refused");
        assert!(
            format!("{err}").contains("capacity section truncated"),
            "the error names the section: {err}"
        );

        // And the old 3-section decoder still reads a 4-section manifest — the
        // trailing bytes are ignored, which is what keeps section 5 possible.
        let (n, _i, p) = decode_manifest_with_inputs_and_publishers(&enc).expect("decode");
        assert_eq!(n.len(), 2);
        assert_eq!(p, vec![vec![("img".to_string(), CAM_ID)], Vec::new()]);
    }
}
