// SPDX-License-Identifier: AGPL-3.0-only
//! PURE struct→schema mapping plans — the testable half of the tick.
//!
//! Each `plan_*` fn turns a decoded `cerulion_go2_dds` struct into a plain
//! plan-of-values struct; the node's tick applies the plan to the output
//! port's loaned SHM slot (thin assignments, no logic). Splitting plan from
//! apply keeps every mapping decision (field routing, the quaternion reorder,
//! the packed `fields` encoding) oracle-testable with no transport.
//!
//! # The packed `fields` encoding (the cerulion_viz contract)
//!
//! `PointCloud2.fields` is a `DynamicArray<Nested>` — opaque bytes on the
//! Cerulion wire (no framework element framing). [`encode_point_fields`] writes
//! the DEFINED packed layout that `cerulion_viz::pointcloud::parse_point_fields`
//! decodes (its docs name a driver as the producer — THIS is that
//! producer):
//!
//! ```text
//! record := [name_len: u32 LE][name: name_len bytes][offset: u32 LE][datatype: u8][count: u32 LE]
//! blob   := record*        (records back-to-back, NO count prefix, NO alignment)
//! ```
//!
//! The two fns are a cross-repo-file MIRROR: a layout change here MUST be
//! mirrored in `cerulion_viz/lib/cerulion_viz/src/pointcloud.rs` (each side pins the
//! bytes with its own hand oracle, so a one-sided edit fails a test).
//!
//! # The odometry projection (SportModeState → nav_msgs/Odometry)
//!
//! `unitree_go/SportModeState` has no native Cerulion schema (custom-schema
//! wiring for node crates is the auto-schema chain). The bridge publishes the state's
//! POSE + TWIST projection as `nav_msgs/Odometry` — the same topic shape
//! the ROS 2 `go2_ros2_sdk` publishes, and exactly what `go2_tf_source` needs
//! to replace its odom→base identity stub. Foot forces,
//! gait/mode etc. are NOT bridged by this projection (full fidelity rides
//! the auto-schema chain).
//!
//! **Covariances (consumer semantics — read carefully before fusing):** the
//! pose/twist covariances are left at the loaned slot's ZERO default because
//! SportModeState carries none. An all-zero covariance is NOT "unknown" to a
//! ROS pose-fusing consumer — a Kalman/EKF reads a zero-variance pose as
//! MAXIMUM confidence (perfectly certain), so it would trust this odometry
//! absolutely. Real covariance handling (or the ROS "unknown" convention —
//! `covariance[0] = -1`) MUST be added before this odometry is fused.
//!
//! **Quaternion order**: Unitree `IMUState.quaternion` is `[w, x, y, z]`
//! (the `cerulion_go2_dds` struct doc, verified against the unitree_ros2 IDL; ROS
//! `geometry_msgs/Quaternion` fields are x/y/z/w) — [`plan_odom`] does the
//! reorder, pinned by a hand oracle below. The w-first convention is not
//! validated against live SportModeState bytes: confirm it on your robot.

use cerulion_go2_dds::messages::{PointCloud2, PointField, SportModeState};

/// The values the tick writes into the `sensor_msgs/PointCloud2` output port
/// (the `data` blob itself stays on the decoded sample — the tick copies it
/// into a size-aware SHM loan via `set_data`, no intermediate copy here).
#[derive(Debug, Clone, PartialEq)]
pub struct CloudPlan {
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub frame_id: String,
    pub height: u32,
    pub width: u32,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub is_dense: bool,
    /// The packed `fields` blob (see the module docs).
    pub fields_blob: Vec<u8>,
}

/// The values the tick writes into the `nav_msgs/Odometry` output port.
#[derive(Debug, Clone, PartialEq)]
pub struct OdomPlan {
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    /// `pose.pose.position` (metres, f64 — widened from the f32 state).
    pub position: [f64; 3],
    /// `pose.pose.orientation` as ROS field order `[x, y, z, w]` — REORDERED
    /// from Unitree's `[w, x, y, z]` (see the module docs).
    pub orientation_xyzw: [f64; 4],
    /// `twist.twist.linear` (body-frame velocity).
    pub linear: [f64; 3],
    /// `twist.twist.angular.z` (yaw rate; x/y stay 0).
    pub yaw_speed: f64,
}

/// Encode `PointField` descriptors into the packed `fields` blob (the
/// cerulion_viz `parse_point_fields` contract — see the module docs). An empty
/// slice encodes to an empty blob (which the sink treats as "infer from
/// point_step", loudly).
pub fn encode_point_fields(fields: &[PointField]) -> Vec<u8> {
    let mut blob = Vec::new();
    for f in fields {
        blob.extend_from_slice(&(f.name.len() as u32).to_le_bytes());
        blob.extend_from_slice(f.name.as_bytes());
        blob.extend_from_slice(&f.offset.to_le_bytes());
        blob.push(f.datatype);
        blob.extend_from_slice(&f.count.to_le_bytes());
    }
    blob
}

/// Plan the PointCloud2 pass-through: geometry/flags verbatim, header stamp +
/// frame_id from the DDS header (sensor time — deterministic replay input:
/// the tick is a pure function of the drained sample), `fields` re-encoded
/// into the packed blob.
pub fn plan_cloud(c: &PointCloud2) -> CloudPlan {
    CloudPlan {
        stamp_sec: c.header.stamp.sec,
        stamp_nanosec: c.header.stamp.nanosec,
        frame_id: c.header.frame_id.clone(),
        height: c.height,
        width: c.width,
        is_bigendian: c.is_bigendian,
        point_step: c.point_step,
        row_step: c.row_step,
        is_dense: c.is_dense,
        fields_blob: encode_point_fields(&c.fields),
    }
}

/// Plan the SportModeState → Odometry projection (see the module docs).
pub fn plan_odom(s: &SportModeState) -> OdomPlan {
    let q = s.imu_state.quaternion; // Unitree [w, x, y, z]
    OdomPlan {
        stamp_sec: s.stamp.sec,
        stamp_nanosec: s.stamp.nanosec,
        position: [
            s.position[0] as f64,
            s.position[1] as f64,
            s.position[2] as f64,
        ],
        orientation_xyzw: [q[1] as f64, q[2] as f64, q[3] as f64, q[0] as f64],
        linear: [
            s.velocity[0] as f64,
            s.velocity[1] as f64,
            s.velocity[2] as f64,
        ],
        yaw_speed: s.yaw_speed as f64,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_go2_dds::messages::{Header, ImuState, Time};

    /// Hand-build one packed field record (an INDEPENDENT oracle — mirrors the
    /// byte layout documented in cerulion_viz::pointcloud::parse_point_fields, so
    /// this test + cerulion_viz's own parser tests pin the contract from both
    /// sides).
    fn oracle_record(name: &str, offset: u32, datatype: u8, count: u32) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&(name.len() as u32).to_le_bytes());
        v.extend_from_slice(name.as_bytes());
        v.extend_from_slice(&offset.to_le_bytes());
        v.push(datatype);
        v.extend_from_slice(&count.to_le_bytes());
        v
    }

    fn xyz_fields() -> Vec<PointField> {
        [("x", 0u32), ("y", 4), ("z", 8)]
            .into_iter()
            .map(|(n, off)| PointField {
                name: n.to_string(),
                offset: off,
                datatype: 7, // FLOAT32
                count: 1,
            })
            .collect()
    }

    #[test]
    fn encode_point_fields_matches_hand_oracle_bytes() {
        let mut oracle = Vec::new();
        oracle.extend(oracle_record("x", 0, 7, 1));
        oracle.extend(oracle_record("y", 4, 7, 1));
        oracle.extend(oracle_record("z", 8, 7, 1));
        assert_eq!(encode_point_fields(&xyz_fields()), oracle);
        // Fully hand-computable check on one record: name_len(4) + "x"(1) +
        // offset(4) + datatype(1) + count(4) = 14 bytes per single-char field.
        assert_eq!(oracle_record("x", 0, 7, 1).len(), 14);
        // Empty in → empty blob (the sink's loud-inference arm).
        assert!(encode_point_fields(&[]).is_empty());
    }

    #[test]
    fn cloud_plan_passes_geometry_through_and_reencodes_fields() {
        let c = PointCloud2 {
            header: Header {
                stamp: Time {
                    sec: 12,
                    nanosec: 34,
                },
                frame_id: "lidar".to_string(),
            },
            height: 1,
            width: 2,
            fields: xyz_fields(),
            is_bigendian: false,
            point_step: 12,
            row_step: 24,
            data: vec![0u8; 24],
            is_dense: true,
        };
        let plan = plan_cloud(&c);
        // Hand-oracle blob (NOT `encode_point_fields` — the fn under test — so
        // this is not a self-compare; same bytes as
        // `encode_point_fields_matches_hand_oracle_bytes` pins independently).
        let mut oracle_blob = Vec::new();
        oracle_blob.extend(oracle_record("x", 0, 7, 1));
        oracle_blob.extend(oracle_record("y", 4, 7, 1));
        oracle_blob.extend(oracle_record("z", 8, 7, 1));
        assert_eq!(
            plan,
            CloudPlan {
                stamp_sec: 12,
                stamp_nanosec: 34,
                frame_id: "lidar".to_string(),
                height: 1,
                width: 2,
                is_bigendian: false,
                point_step: 12,
                row_step: 24,
                is_dense: true,
                fields_blob: oracle_blob,
            }
        );
    }

    #[test]
    fn odom_plan_reorders_the_unitree_quaternion_and_widens_to_f64() {
        // DISTINCT quaternion components so a wrong reorder shows a wrong
        // VALUE, never a coincidental pass: Unitree [w,x,y,z] = [0.5, 0.1,
        // 0.2, 0.3] must land as ROS [x,y,z,w] = [0.1, 0.2, 0.3, 0.5].
        let s = SportModeState {
            stamp: Time {
                sec: 100,
                nanosec: 250_000_000,
            },
            imu_state: ImuState {
                quaternion: [0.5, 0.1, 0.2, 0.3],
                ..Default::default()
            },
            position: [1.5, -2.5, 0.25],
            velocity: [0.5, 0.0, 0.0],
            yaw_speed: 0.25,
            ..Default::default()
        };
        let plan = plan_odom(&s);
        assert_eq!(
            plan,
            OdomPlan {
                stamp_sec: 100,
                stamp_nanosec: 250_000_000,
                position: [1.5, -2.5, 0.25],
                // The REORDER pin (hand oracle, exact f32→f64-widenable values).
                orientation_xyzw: [0.1f32 as f64, 0.2f32 as f64, 0.3f32 as f64, 0.5f32 as f64],
                linear: [0.5, 0.0, 0.0],
                yaw_speed: 0.25,
            }
        );
    }
}
