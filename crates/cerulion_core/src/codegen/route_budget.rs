// SPDX-License-Identifier: AGPL-3.0-only
//! The per-route SLICE budget a dynamically-created ingress route
//! (a `ros2 attach` bridge `RawIngressRoute`) should ask for, derived from the
//! route's own SCHEMA.
//!
//! # Why a route's slice decides how many frames it can absorb
//!
//! An ingress route has a slice-aware receive-queue depth
//! ([`crate::transport::ingress_route_buffer_depth`]):
//!
//! ```text
//! depth = clamp( 64 MiB / max_slice_len, 16, 1024 )
//! ```
//!
//! and every `ros2 attach` route inherits `dds_bridge`'s BRIDGE-WIDE
//! `DEFAULT_RAW_MAX_SLICE_LEN` = 1 MiB, because the generated bridge config
//! never emits `max_slice_len:`. So every route — a 157-byte `/tf` frame and a
//! 46 KiB point cloud alike — lands on the SAME depth 64, which absorbs 38.6 ms
//! at 1.66 kHz.
//!
//! The Go2 verification run makes that number matter: two OUTLIER drive
//! passes (`max_pass_duration_us` = 107,898, two drain-gap-histogram entries in
//! the 100–250 ms bucket) exceeded 38.6 ms while the recorder staged and wrote a
//! burst of large frames across 102 taps, and a ~1.66 kHz topic committed 170+
//! frames into a 64-frame queue. Those frames were reclaimed in SHM at commit
//! (drop-oldest) before the drain reached the tap: unrecoverable by
//! construction, because the queue depth IS the loss boundary.
//!
//! The depth is CREATE-time and immutable (iceoryx2 pins
//! `subscriber_max_buffer_size` at service creation), so the only lever is the
//! slice the route is created with — and the only thing that knows a route's
//! size is its schema. That is this module.
//!
//! # The rule, and why each half is safe
//!
//! [`route_slice_budget`] returns a byte budget for one schema:
//!
//! * **A recursively-FIXED schema** (no offset-table fields) has a wire frame of
//!   EXACTLY `WireHeader::SIZE + fixed_size` bytes — `CdrCodec::decode` builds
//!   the header, appends the fixed section, and there is no variable payload to
//!   append. So the budget is that number, exactly, with no headroom, and it
//!   cannot be exceeded by any frame of that type. There is no guess here at
//!   all.
//!
//! * **A VARIABLE schema** has no bound this module can compute — a `string` or
//!   an unbounded array is unbounded on the wire — so it takes the repo's OWN
//!   per-schema budget, the [`variable_schema_max_slice_len`] tier table that
//!   already decides what a GRAPH-DECLARED output of that schema reserves.
//!   Every `nav_msgs/Odometry` topic written by a Cerulion node is capped at
//!   TIER_TINY (16 KiB) today; a bridged `nav_msgs/Odometry` asking for the
//!   same 16 KiB is therefore not a new judgement, it is the judgement the repo
//!   already ships, applied to a route that was ignoring it. (The tier audit moved
//!   Odometry SMALL -> TINY: its wire size is near-constant at ~1 KB — 688 B of
//!   fixed section including two 288-B covariance matrices, plus two frame
//!   strings — so the tier it inherits here got deeper without this module
//!   changing at all. That is the intended shape: the table is the judgement,
//!   this module only applies it.)
//!
//! # This is a CEILING to be taken with `min`, never a value to be adopted
//!
//! The caller composes it as `min(configured_default, budget)`, so the budget
//! can only ever SHRINK a route below the bridge-wide default — never grow one.
//! That matters because the tier table is sized for a type's worst case and is
//! frequently LARGER than the bridge default (`sensor_msgs/PointCloud2` is
//! TIER_HUGE = 128 MiB, `visualization_msgs/MarkerArray` is TIER_LARGE = 16 MiB):
//! adopting the tier directly would make those routes' queues SHALLOWER than
//! they are today, which is the opposite of the fix.
//!
//! An explicitly configured `max_slice_len:` wins outright and is not narrowed —
//! same precedence as everywhere else in the repo (explicit YAML > schema
//! budget > default).
//!
//! # What this reaches on the Go2, and what it still does not
//!
//! Both Go2 frame-loss victims are covered:
//!
//! | topic | type | tier | slice | ingress depth | absorbs @1.66 kHz |
//! |---|---|---|---|---|---|
//! | `/state_estimation` | `nav_msgs/Odometry` | TINY | 16 KiB | 1024 | 617 ms |
//! | `/tf` | `tf2_msgs/TFMessage` | SMALL | 256 KiB | 256 | 154 ms |
//! | `/utlidar/cloud` | `sensor_msgs/PointCloud2` | HUGE | 1 MiB (default) | 64 | 38.6 ms |
//!
//! `/state_estimation` gained its extra 4x when the later tier audit
//! moved `nav_msgs/Odometry` SMALL -> TINY. Nothing in this module
//! changed for it — the row moved because the table it reads did.
//!
//! `/tf` reaches it only because `tf2_msgs/TFMessage` was also moved
//! MEDIUM → SMALL in the tier table (a deliberate decision): at MEDIUM its 4 MiB was
//! ABOVE the bridge default, so the `min` left it at 1 MiB and depth 64, and it
//! MEASURED 12.67 % / 13.15 % loss under the burst bench. That move is a change
//! to a SHARED table — it re-sizes every graph-declared `/tf` publisher too, and
//! the reasoning + the breakage boundary it introduces live beside the arm in
//! `variable_schema_max_slice_len`.
//!
//! The residual that remains: a variable schema whose tier is still at or above
//! the bridge default gets no improvement — TIER_HUGE and TIER_LARGE types, i.e.
//! exactly the large-payload rows that are inherently low-rate. That is the
//! intended shape, not an oversight: a 46 KiB point cloud at a few Hz cannot
//! overflow 64 frames, and giving it a deeper queue would cost real SHM for
//! nothing.

use crate::codegen::layout::WireLayout;
use crate::codegen::variable_schema_max_slice_len;
use crate::wire::{MaxSliceLen, WireHeader};

/// The per-schema slice budget for a dynamically-created ingress route.
///
/// See the module docs for the two arms and for why the result is a CEILING to
/// be composed with `min`, never a value to adopt.
///
/// Returns a raw `usize` rather than a [`MaxSliceLen`] because the number is an
/// input to a `min`, not a provisioning decision; [`route_slice_budget_capped`]
/// is the composed form callers actually want.
pub fn route_slice_budget(layout: &WireLayout) -> usize {
    if layout.variable_fields.is_empty() {
        // EXACT: `CdrCodec::decode` emits header + fixed section and nothing
        // else for a schema with no offset-table fields. `MaxSliceLen`'s own
        // floor is `WireHeader::SIZE`, and `fixed_size` is 0 only for a
        // fieldless schema (`std_msgs/Empty`), so this is always constructible.
        WireHeader::SIZE + layout.fixed_size
    } else {
        variable_schema_max_slice_len(&layout.qualified_name)
    }
}

/// The composed rule: narrow `configured` to this schema's budget, never
/// widening it.
///
/// `configured` is the slice the route would get today — the bridge-wide
/// default. The result is `min(configured, budget)`, floored at
/// `WireHeader::SIZE` by [`MaxSliceLen`]'s own invariant.
pub fn route_slice_budget_capped(layout: &WireLayout, configured: MaxSliceLen) -> MaxSliceLen {
    let budget = route_slice_budget(layout);
    // Saturating: a budget above `u32::MAX` cannot narrow a `MaxSliceLen`
    // anyway, and one below the 32-byte floor is impossible (the fixed arm is
    // `>= WireHeader::SIZE` by construction and every tier is >= 16 KiB) — but
    // `min` with the configured value keeps the result constructible without
    // relying on that.
    let narrowed = budget.min(configured.get() as usize) as u32;
    MaxSliceLen::try_new(narrowed).unwrap_or(configured)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::layout::{FieldLayout, VariableFieldLayout};
    use crate::codegen::FieldType;

    /// A hand-built layout — never a real one resolved from the corpus, so the
    /// oracles below are arithmetic on numbers this test states itself.
    fn layout(qualified_name: &str, fixed_size: usize, n_variable: usize) -> WireLayout {
        WireLayout {
            qualified_name: qualified_name.to_string(),
            schema_hash: 0,
            fixed_size,
            fixed_align: 8,
            fixed_fields: Vec::<FieldLayout>::new(),
            variable_fields: (0..n_variable)
                .map(|i| VariableFieldLayout {
                    name: format!("v{i}"),
                    field_type: FieldType::DynamicArray {
                        element_type: Box::new(FieldType::U8),
                    },
                })
                .collect(),
        }
    }

    /// A FIXED schema's budget is EXACTLY its frame size — header plus fixed
    /// section, no headroom.
    ///
    /// Hand oracle: 125 is chosen so `32 + 125 = 157`, the `/tf` frame size
    /// the module docs quote, which is what makes the depth consequence below concrete.
    #[test]
    fn a_fixed_schema_budget_is_exactly_header_plus_fixed_section() {
        assert_eq!(route_slice_budget(&layout("demo/Fixed", 125, 0)), 157);
        // The degenerate end: a fieldless schema is a bare header.
        assert_eq!(
            route_slice_budget(&layout("std_msgs/Empty", 0, 0)),
            WireHeader::SIZE
        );
        // And a large fixed schema is still exact — no tier, no rounding.
        assert_eq!(
            route_slice_budget(&layout("demo/BigFixed", 8_000, 0)),
            8_032
        );
    }

    /// A VARIABLE schema takes the repo's own per-schema tier, not a computed
    /// size — and the tier is a property of the NAME, so two layouts with
    /// identical geometry and different names get different budgets.
    ///
    /// Hand oracles are the tier constants themselves (`variable_schema_max_slice_len`
    /// is the source of truth for graph-declared outputs of the same schemas).
    #[test]
    fn a_variable_schema_budget_is_its_declared_tier() {
        // TIER_TINY: `/state_estimation` on the Go2. The tier audit moved
        // `nav_msgs/Odometry` SMALL -> TINY (its wire size is near-constant at
        // ~1 KB), which takes this route from depth 256 to the 1024 cap. Pinned
        // as a HAND value rather than re-blessed: the number is what a bridged
        // odometry route can absorb, and it is also what a graph-declared
        // Odometry publisher can fit in one message.
        assert_eq!(
            route_slice_budget(&layout("nav_msgs/Odometry", 800, 2)),
            16 * 1024
        );
        // TIER_SMALL: `/tf` on the Go2, moved MEDIUM -> SMALL by a deliberate
        // decision. Pinned as a hand value because the number is what
        // takes `/tf` from depth 64 to depth 256, AND it is what a graph-declared
        // `/tf` publisher can now fit in one message.
        assert_eq!(
            route_slice_budget(&layout("tf2_msgs/TFMessage", 0, 1)),
            256 * 1024
        );
        // TIER_MEDIUM is still reachable — the retier moved ONE type, not the
        // tier. Without this row a mutation that emptied the medium arm entirely
        // would pass.
        assert_eq!(
            route_slice_budget(&layout("nav_msgs/Path", 0, 2)),
            4 * 1024 * 1024
        );
        // TIER_HUGE: the type the module docs name as keeping the conservative slice.
        assert_eq!(
            route_slice_budget(&layout("sensor_msgs/PointCloud2", 40, 3)),
            128 * 1024 * 1024
        );
        // TIER_TINY — the shape most bridged sensor rows have.
        assert_eq!(
            route_slice_budget(&layout("sensor_msgs/Imu", 300, 1)),
            16 * 1024
        );
        // THE NAME IS THE INPUT, not the geometry: this layout is byte-for-byte
        // the `nav_msgs/Odometry` shape asserted above (fixed 800, 2 variable
        // fields) and must nonetheless get a DIFFERENT budget, because
        // `nav_msgs/Path` is a different arm of the table. A budget computed
        // from `fixed_size` would return the same number for both.
        //
        // Note: `sensor_msgs/Imu` shares Odometry's TIER_TINY tier, so using it
        // here would silently collapse the discriminator to a self-comparison. Any
        // replacement here must be a schema whose tier DIFFERS from the row it
        // is contrasted with.
        assert_eq!(
            route_slice_budget(&layout("nav_msgs/Path", 800, 2)),
            4 * 1024 * 1024
        );
    }

    /// The composed rule NARROWS and never widens.
    ///
    /// The widening half is the load-bearing one: every tier from MEDIUM up is
    /// larger than the bridge's 1 MiB default, so a rule that adopted the budget
    /// would make a marker-array or point-cloud route's queue SHALLOWER than it
    /// is today.
    #[test]
    fn the_capped_rule_narrows_but_never_widens() {
        const BRIDGE_DEFAULT: MaxSliceLen = MaxSliceLen::const_new(1 << 20);

        // Narrows: a fixed 157-byte schema.
        assert_eq!(
            route_slice_budget_capped(&layout("demo/Fixed", 125, 0), BRIDGE_DEFAULT).get(),
            157
        );
        // Narrows: both Go2 victims are below the default —
        // `/state_estimation` at TIER_TINY (the tier audit moved Odometry SMALL ->
        // TINY) and `/tf` at TIER_SMALL (a deliberate move). They are
        // asserted at DIFFERENT numbers on purpose: the narrowing rule must
        // carry each route's own tier, not a single "below the default" bucket.
        assert_eq!(
            route_slice_budget_capped(&layout("nav_msgs/Odometry", 800, 2), BRIDGE_DEFAULT).get(),
            16 * 1024
        );
        assert_eq!(
            route_slice_budget_capped(&layout("tf2_msgs/TFMessage", 0, 1), BRIDGE_DEFAULT).get(),
            256 * 1024
        );
        // Does NOT widen: TIER_MEDIUM is above the default.
        assert_eq!(
            route_slice_budget_capped(&layout("nav_msgs/Path", 0, 2), BRIDGE_DEFAULT).get(),
            1 << 20
        );
        // Does NOT widen: TIER_HUGE is far above the default.
        assert_eq!(
            route_slice_budget_capped(&layout("sensor_msgs/PointCloud2", 40, 3), BRIDGE_DEFAULT)
                .get(),
            1 << 20
        );
        // An EXPLICITLY configured small slice is not widened by a big tier
        // either — `min` is symmetric, so the caller's choice survives.
        assert_eq!(
            route_slice_budget_capped(
                &layout("sensor_msgs/PointCloud2", 40, 3),
                MaxSliceLen::const_new(64 * 1024)
            )
            .get(),
            64 * 1024
        );
    }

    /// THE claim this module rests on, checked against the real encoder rather
    /// than reasoned about: a FIXED schema's decoded frame is EXACTLY the
    /// budget, so a route sized to it can never fail a publish.
    ///
    /// This is the arm that would catch the dangerous failure mode. The fixed
    /// budget carries no headroom on purpose, so if `CdrCodec::decode` appended
    /// so much as one alignment byte beyond `WireHeader::SIZE + fixed_size`,
    /// every frame on every fixed route would exceed its slice and be DROPPED at
    /// `publish_raw` (a loan of exactly `frame.len()` against a `Static` pool
    /// that cannot grow) — trading a queue-overflow loss for a total one.
    ///
    /// Driven over three shapes with different alignment stories, each decoded
    /// from a hand-built CDR body: two `f64`s (8-aligned, no padding), a
    /// `bool` + `f64` (the parser inserts 7 bytes of padding before the f64),
    /// and a `bool` alone (a 1-byte fixed section).
    #[test]
    fn a_fixed_schemas_real_decoded_frame_is_exactly_the_budget() {
        use crate::codegen::layout::LayoutResolver;
        use crate::codegen::{parse_rosmsg, CdrCodec, CdrEndianness};

        // (name, .msg text, CDR body little-endian)
        let cases: [(&str, &str, Vec<u8>); 3] = [
            ("Pair", "float64 a\nfloat64 b\n", {
                let mut v = 1.5f64.to_le_bytes().to_vec();
                v.extend_from_slice(&2.5f64.to_le_bytes());
                v
            }),
            ("Padded", "bool flag\nfloat64 v\n", {
                // CDR aligns the f64 to 8 after the 1-byte bool.
                let mut v = vec![1u8, 0, 0, 0, 0, 0, 0, 0];
                v.extend_from_slice(&3.25f64.to_le_bytes());
                v
            }),
            ("Tiny", "bool flag\n", vec![1u8]),
        ];

        for (name, text, body) in cases {
            let schema = parse_rosmsg(text, name, Some("demo")).expect("parse");
            let (mut resolver, _) = LayoutResolver::new(vec![schema.clone()]);
            let layout = resolver.layout_of(&format!("demo/{name}")).expect("layout");
            let (codec, _) = CdrCodec::new(vec![schema]);
            let frame = codec
                .decode(&format!("demo/{name}"), CdrEndianness::Little, &body, 0, 0)
                .expect("decode");

            assert!(
                layout.variable_fields.is_empty(),
                "{name} must be a FIXED schema for this arm to say anything"
            );
            assert_eq!(
                frame.len(),
                route_slice_budget(&layout),
                "{name}: the budget must be the frame size EXACTLY — a budget \
                 under the real frame drops every frame on the route"
            );
        }
    }

    /// The result is always a constructible `MaxSliceLen` — including at the
    /// 32-byte floor, where a fieldless fixed schema lands.
    #[test]
    fn the_capped_rule_is_constructible_at_the_floor() {
        let got = route_slice_budget_capped(
            &layout("std_msgs/Empty", 0, 0),
            MaxSliceLen::const_new(1 << 20),
        );
        assert_eq!(got.get() as usize, WireHeader::SIZE);
    }
}
