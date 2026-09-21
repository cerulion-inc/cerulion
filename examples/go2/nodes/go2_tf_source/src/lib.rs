// SPDX-License-Identifier: AGPL-3.0-only
//! Go2 TF broadcaster.
//!
//! A small periodic node that publishes standard `tf2_msgs/TFMessage`
//! transforms, so this workspace and a robot running a ROS 2
//! `robot_state_publisher` feed the SAME desk-side viz contract.
//! It writes the transforms with the pure [`go2_tf`] element
//! codec (`set_transforms_bytes(&encode_tf_transforms(..))`) — the exact bytes
//! the DESK decodes, and byte-identical to the zenoh-link test
//! (`cerulion_core/tests/network_tf_e2e_test.rs`).
//!
//! Nothing renders on the robot. This node ships RAW
//! frames; `cerulion-netd` mirrors a demanded topic into desk-local shared
//! memory and `cerulion-vizd` decodes + renders on the user's machine
//! (`cerulion viz --robot go2`). This crate therefore has ZERO rerun edges even
//! under `-e normal,dev` — the `rerun-leanness` CI job enforces it, and the viz
//! e2e lives desk-side at
//! `cerulion_viz/lib/cerulion_viz/tests/tf_source_e2e_test.rs`.
//!
//! # What it publishes
//!
//! - `/tf` — the DYNAMIC `odom → base` transform.
//! - `/tf_static` — the STATIC mount table (`base → lidar`, `base → camera`).
//!
//! # odom → base is an IDENTITY stub
//!
//! The pose data is already bridged: `dds_bridge` projects the firmware's
//! `/sportmodestate` (`unitree_go/SportModeState`, vendored in
//! `lib/cerulion_go2_dds`) onto `/go2/odom` as `nav_msgs/Odometry` (pose +
//! twist; the quaternion reordered from Unitree's `[w, x, y, z]`). This node
//! does not consume it: the `SportModeState` byte layout is not validated
//! against samples from a robot (`examples/go2/README.md`,
//! "Validate on your robot"). Rather than fabricate motion (Principle #13),
//! this node publishes an IDENTITY `odom` to `base`: the robot sits at the odom
//! origin. This gives the DESK viewer a COMPLETE, correct transform tree:
//! the lidar cloud and camera still render posed via the static
//! mounts; only the base's motion within odom is absent. To derive the
//! transform, add `#[input] odom` wired to the absolute `/go2/odom` (the
//! producer-less absolute-source class) and replace
//! [`go2_tf::identity_odom_base`] with the derived transform.
//!
//! # Publish cadence
//!
//! `period_ms = 100` (10 Hz — a low rate; a real odom feed runs faster). The
//! macro path arms EVERY declared output on every `Ok` tick, so BOTH
//! `/tf` and `/tf_static` publish each fire. Re-broadcasting `/tf_static` at
//! 10 Hz is a strict superset of "publish once at startup" — the static
//! transforms never change, so the sink's `log_static` re-logs identical
//! bytes (idempotent). This is the simplest correct shape given the every-output-arms
//! constraint (teleop_mux does the same) — no conditional-publish surface is
//! needed.

// P12 (the AGENTS.md logging rule): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use cerulion_core::TransportError;
use go2_tf::{
    encode_tf_transforms, encode_tf_transforms_into, identity_odom_base, static_mounts, TfTransform,
};
use native_ros2_messages::builtin_interfaces::Time;
use native_ros2_messages::tf2_msgs::TFMessage;

/// Fixed geometry is prepared once. Each tick only changes the dynamic stamp
/// and reuses the encoder buffer; frame names and static mounts never allocate
/// on the publication path. Reconstructing this cache is deterministic: it
/// contains only the fixed demo geometry and scratch bytes, not robot state.
#[derive(Debug)]
struct TfPublishBuffers {
    dynamic: [TfTransform; 1],
    dynamic_bytes: Vec<u8>,
    static_bytes: Vec<u8>,
}

impl Default for TfPublishBuffers {
    fn default() -> Self {
        let dynamic = [identity_odom_base(0, 0)];
        Self {
            dynamic_bytes: encode_tf_transforms(&dynamic),
            dynamic,
            static_bytes: encode_tf_transforms(&static_mounts()),
        }
    }
}

impl TfPublishBuffers {
    fn stamp(&mut self, sec: i32, nanosec: u32) {
        self.dynamic[0].stamp_sec = sec;
        self.dynamic[0].stamp_nanosec = nanosec;
        encode_tf_transforms_into(&self.dynamic, &mut self.dynamic_bytes);
    }
}

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
pub struct Go2TfSource {
    #[cerulion(reconstruct)]
    buffers: TfPublishBuffers,
    /// The dynamic `odom → base` transform.
    #[output]
    tf: TFMessage,
    /// The static mount table (`base → lidar`, `base → camera`).
    #[output]
    tf_static: TFMessage,
}

#[cerulion_node_impl]
impl Go2TfSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        // The transform's OWN stamp = the node clock (deterministic — the wire
        // stamp the sink uses for its timeline comes from the transport at loan
        // time; both derive from the same clock, so replay is bit-identical).
        // The ns→(sec, nanosec) split is the built-in `Time::from_ns` helper,
        // fed the NODE clock, never a wall clock.
        let stamp = Time::from_ns(self.now_ns());

        // Dynamic: odom → base (identity stub — see the module docs).
        self.buffers.stamp(stamp.sec, stamp.nanosec);
        self.tf
            .set_transforms_bytes(&self.buffers.dynamic_bytes)
            .map_err(to_node_err)?;

        // Static: the sensor mounts (re-broadcast every tick — harmless, see
        // the module docs' cadence note).
        self.tf_static
            .set_transforms_bytes(&self.buffers.static_bytes)
            .map_err(to_node_err)?;

        Ok(())
    }
}

fn to_node_err(e: TransportError) -> NodeError {
    NodeError::Logic(e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamping_reuses_storage_and_preserves_static_geometry() {
        let mut buffers = TfPublishBuffers::default();
        let dynamic_ptr = buffers.dynamic_bytes.as_ptr();
        let static_ptr = buffers.static_bytes.as_ptr();
        let static_bytes = buffers.static_bytes.clone();
        for (sec, nanosec) in [(0, 0), (7, 42), (1, 500_000_000)] {
            buffers.stamp(sec, nanosec);
            assert_eq!(buffers.dynamic_bytes.as_ptr(), dynamic_ptr);
            assert_eq!(buffers.static_bytes.as_ptr(), static_ptr);
            assert_eq!(buffers.static_bytes, static_bytes);
            let decoded = go2_tf::decode_tf_transforms(&buffers.dynamic_bytes).unwrap();
            assert_eq!(decoded.len(), 1);
            assert_eq!(decoded[0].frame_id, "odom");
            assert_eq!(decoded[0].child_frame_id, "base");
            assert_eq!(decoded[0].translation, [0.0, 0.0, 0.0]);
            assert_eq!(decoded[0].rotation, [0.0, 0.0, 0.0, 1.0]);
            assert_eq!(
                (decoded[0].stamp_sec, decoded[0].stamp_nanosec),
                (sec, nanosec)
            );
        }
    }
}
