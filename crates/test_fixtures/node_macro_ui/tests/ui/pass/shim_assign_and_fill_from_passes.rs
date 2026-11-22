#![allow(unexpected_cfgs)]
//! Pass case: the schema-blind shim rewrite end-to-end at
//! compile level. EVERY simple `self.<port>.<field> = expr` on an output
//! port — fixed scalar, variable string, variable bytes, complex nested —
//! lowers to the uniform codegen shim `__cer_<port>.__cer_assign_<field>(…)?`
//! and MUST compile without any `#[output(...)]` field list; likewise
//! `self.<port>.<field>.fill_from(…)` lowers to
//! `__cer_<port>.__cer_fill_from_<field>(…)` (codegen resolves the
//! simple-vs-complex delegate). A helper method gets the same treatment
//! (the per-tick proxy is threaded in as a trailing parameter).

use cerulion_core::prelude::*;
use native_ros2_messages::autoware_perception_msgs::PredictedObject;
use native_ros2_messages::sensor_msgs::{Image, Imu};

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct ShimAssignNode {
    #[output]
    image: Image,
    frame: u32,
}

#[cerulion_node_impl]
impl ShimAssignNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.frame += 1;
        // Fixed fields — `__cer_assign_height(&(self.frame))?` etc.
        self.image.height = self.frame;
        self.image.width = 1920;
        self.image.step = 5760;
        self.image.is_bigendian = 0;
        // Variable string — `__cer_assign_encoding("rgb8")?` (str literal
        // passes through unborrowed).
        self.image.encoding = "rgb8";
        // Complex nested — `__cer_assign_header(&[][..])?` (delegates to
        // `set_header_bytes`).
        self.image.header = &[][..];
        // Variable bytes via fill_from in a HELPER — the helper body gets
        // the same rewrite + the proxy threaded as a trailing parameter.
        self.populate_data()?;
        Ok(())
    }

    fn populate_data(&mut self) -> Result<(), NodeError> {
        // `__cer_fill_from_data(closure)` — no `#[output(...)]` list needed.
        self.image.data.fill_from(|buf: &mut [u8]| {
            buf[..2].copy_from_slice(&[0xAB, 0xCD]);
            Ok(2)
        })?;
        // …and the bytes-assign form on the same field (last write wins).
        self.image.data = b"\x01\x02\x03";
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// The RECURSIVE nested-write rewrite (>= 3 segments past the
// port) lowers to the `__cer_with_nested_*` closure chain — fixed AND
// complex-variable nested, any depth, plus nested `fill_from` and the
// `with_<f>` closure alternative. Each field here is written by EXACTLY ONE
// mechanism (no staged-vs-whole-field conflict), so it doubles as a
// canonical usage reference.
// ---------------------------------------------------------------------------
#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct NestedShimPassNode {
    #[output]
    imu: Imu,
    #[output]
    image: Image,
    #[output]
    image2: Image,
}

#[cerulion_node_impl]
impl NestedShimPassNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // 3-segment FIXED nested: orientation is an inline fixed substruct
        // (QuaternionShm), so this lowers to
        // `__cer_imu.__cer_with_nested_orientation(|__cer_v| __cer_v.__cer_assign_x(&(1.5)))?`.
        self.imu.orientation.x = 1.5;
        self.imu.orientation.w = 1.0;
        // Same-port RHS on a nested fixed path — pins the RHS hoist compiles
        // (the closure would otherwise capture `__cer_imu` while it is
        // mutably borrowed → E0502).
        self.imu.orientation.y = self.imu.orientation.w * 2.0;
        // imu's only variable field is `header`; write it via the staged
        // sugar (single mechanism) to satisfy the publish gate.
        self.imu.header.frame_id = "imu0";

        // Variable-nested sugar (staged): frame_id, depth-2 stamp.sec, and a
        // `with_<f>` closure — all STAGED writes to the SAME `header` field,
        // which accumulate (resume) with no conflict.
        self.image.header.frame_id = "cam0";
        self.image.header.stamp.sec = 5; // depth-2: header.stamp.sec
        self.image.with_header(|h| {
            h.stamp.nanosec = 7;
            Ok(())
        })?;
        self.image.encoding = "rgb8";
        self.image.data = b"\x00";

        // Nested fill_from: `self.<port>.<f1>…<leaf>.fill_from(args)` →
        // `__cer_image2.__cer_with_nested_header(|__cer_v| __cer_v.__cer_fill_from_frame_id(args))`
        // (no trailing `?` — the user's own `?` wraps it). Different child
        // field (frame_id) from the depth-2 assign below, so both stage
        // cleanly.
        self.image2.header.frame_id.fill_from(|buf: &mut [u8]| {
            buf[..4].copy_from_slice(b"cam1");
            Ok(4)
        })?;
        self.image2.header.stamp.sec = 9;
        self.image2.encoding = "rgb8";
        self.image2.data = b"\x00";
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// KEYWORD leaf under a COMPLEX-nested field. Real native schema:
// `autoware_perception_msgs::PredictedObject.shape` is complex-nested (Shape
// has a variable `footprint`), and `Shape.type` (u8) is a Rust keyword field.
// `self.obj.shape.r#type = 2` lowers to
// `__cer_obj.__cer_with_nested_shape(|__cer_v| __cer_v.__cer_assign_type(&(2)))?`.
// COMPILING is the pin: the `r#` on `r#type` MUST be stripped or
// the macro panics building `__cer_assign_r#type`; `__cer_with_nested_shape`
// must exist on the complex-nested writer or this is an E0599.
// ---------------------------------------------------------------------------
#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct KeywordNestedLeafNode {
    #[output]
    obj: PredictedObject,
}

#[cerulion_node_impl]
impl KeywordNestedLeafNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.obj.shape.r#type = 2;
        Ok(())
    }
}

fn main() {
    // Reference the generated entries so the expansions are exercised.
    let _entry = ShimAssignNodeEntry::new();
    let _nested = NestedShimPassNodeEntry::new();
    let _kw = KeywordNestedLeafNodeEntry::new();
}
