// SPDX-License-Identifier: AGPL-3.0-only
//! ROS 2 / Unitree message definitions for the Go2 DDS bridge (v1 set).
//!
//! These are plain `serde` structs whose field ORDER and TYPES exactly match
//! the source `.msg` IDL. That is the whole contract: `ros2-client`'s `Message`
//! marker has a blanket `Serialize + DeserializeOwned` bound, and the
//! [`cdr`](crate::cdr) codecs (and typed DDS pub/sub in the node crates) serialize
//! them over CDR. A field-order or type mismatch does NOT error — it silently
//! mis-decodes — so every struct below is field-verified against its IDL with
//! the source URL cited inline. The oracle-vector tests in [`crate::cdr`] pin
//! the wire bytes against hand-built buffers.
//!
//! ## CDR type-mapping rules (why the Rust types are what they are)
//!
//! - ROS `floatN`/`intN`/`uintN`/`bool`/`string` -> the obvious Rust scalar /
//!   `String`.
//! - A ROS **fixed** array `T[N]` -> a Rust array `[T; N]` (CDR writes the `N`
//!   elements with NO length prefix). Using a `Vec` here would inject a
//!   spurious `u32` count and corrupt the wire layout — so fixed arrays are
//!   arrays, deliberately.
//! - A ROS **dynamic** array `T[]` -> a Rust `Vec<T>` (CDR writes a `u32`
//!   element count then the elements).
//! - A nested message -> a nested struct; CDR flattens its fields inline (no
//!   wrapper bytes), with normal per-field alignment.

use serde::{Deserialize, Serialize};

// ===========================================================================
// builtin_interfaces / std_msgs (shared)
// ===========================================================================

/// `builtin_interfaces/msg/Time` (a.k.a. Unitree `unitree_go/msg/TimeSpec`,
/// same two fields). Verified against
/// <https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/TimeSpec.msg>
/// (`int32 sec` / `uint32 nanosec`).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

/// `std_msgs/msg/Header` — `builtin_interfaces/Time stamp` + `string frame_id`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

// ===========================================================================
// sensor_msgs/PointCloud2
// ===========================================================================

/// `sensor_msgs/msg/PointField`. Matches `native_ros2_messages`'
/// `msg/sensor_msgs/PointField.msg`: `string name`, `uint32 offset`,
/// `uint8 datatype`, `uint32 count`. `datatype == 7` is `FLOAT32`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

/// ROS 2 `sensor_msgs/PointField` datatype code for `FLOAT32`.
pub const POINT_FIELD_FLOAT32: u8 = 7;

/// `sensor_msgs/msg/PointCloud2`. Matches `native_ros2_messages`'
/// `msg/sensor_msgs/PointCloud2.msg` field-for-field:
/// `Header header` / `uint32 height` / `uint32 width` /
/// `PointField[] fields` / `bool is_bigendian` / `uint32 point_step` /
/// `uint32 row_step` / `uint8[] data` / `bool is_dense`.
///
/// `fields` and `data` are dynamic arrays -> `Vec` (each gets a CDR `u32`
/// count). `data` is the packed point blob; decode it with
/// [`crate::cdr::decode_xyz_points`] once the message itself is CDR-decoded.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PointCloud2 {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}

// ===========================================================================
// geometry_msgs/Twist (internal command type)
// ===========================================================================

/// `geometry_msgs/msg/Vector3` — three `float64`. Matches
/// `native_ros2_messages`' `msg/geometry_msgs/Vector3.msg`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// `geometry_msgs/msg/Twist` — `Vector3 linear` + `Vector3 angular`. Matches
/// `native_ros2_messages`' `msg/geometry_msgs/Twist.msg`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

// ===========================================================================
// unitree_go/SportModeState
// ===========================================================================
//
// Field-verified 2026-07-08 against the public Unitree IDL:
//   SportModeState: https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/SportModeState.msg
//   IMUState:       https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/IMUState.msg
//   TimeSpec:       https://github.com/unitreerobotics/unitree_ros2/blob/master/cyclonedds_ws/src/unitree/unitree_go/msg/TimeSpec.msg

/// `unitree_go/msg/IMUState`:
/// `float32[4] quaternion` / `float32[3] gyroscope` /
/// `float32[3] accelerometer` / `float32[3] rpy` / `int8 temperature`.
/// The four `float32[N]` are FIXED arrays -> Rust arrays (no CDR count).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ImuState {
    /// Orientation quaternion `[w, x, y, z]` (Unitree order — the on-robot
    /// convention; the sink is responsible for any reordering).
    pub quaternion: [f32; 4],
    pub gyroscope: [f32; 3],
    pub accelerometer: [f32; 3],
    pub rpy: [f32; 3],
    pub temperature: i8,
}

/// `unitree_go/msg/SportModeState` — the Go2 high-level body state. Field order
/// is load-bearing (see the module note); verified against the IDL cited above.
///
/// LIMITATION: this layout is validated structurally (body-size +
/// field-offset oracle tests in [`crate::cdr`]) and against the cited IDL, but
/// is NOT validated against real DDS bytes from a Go2 — the
/// live-peer e2e uses a stock ROS 2 Humble peer, which carries no
/// `unitree_go`/`unitree_api` msgs. Cross-check it against bytes from your
/// robot before you rely on it.
///
/// IDL field walk:
///
/// ```text
/// TimeSpec       stamp
/// uint32         error_code
/// IMUState       imu_state
/// uint8          mode
/// float32        progress
/// uint8          gait_type
/// float32        foot_raise_height
/// float32[3]     position
/// float32        body_height
/// float32[3]     velocity
/// float32        yaw_speed
/// float32[4]     range_obstacle
/// int16[4]       foot_force
/// float32[12]    foot_position_body
/// float32[12]    foot_speed_body
/// ```
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SportModeState {
    pub stamp: Time,
    pub error_code: u32,
    pub imu_state: ImuState,
    pub mode: u8,
    pub progress: f32,
    pub gait_type: u8,
    pub foot_raise_height: f32,
    pub position: [f32; 3],
    pub body_height: f32,
    pub velocity: [f32; 3],
    pub yaw_speed: f32,
    pub range_obstacle: [f32; 4],
    pub foot_force: [i16; 4],
    pub foot_position_body: [f32; 12],
    pub foot_speed_body: [f32; 12],
}

// ===========================================================================
// unitree_api/Request
// ===========================================================================
//
// Field-verified 2026-07-08 against:
//   https://github.com/unitreerobotics/unitree_ros2/tree/master/cyclonedds_ws/src/unitree/unitree_api/msg
//   Request.msg:         RequestHeader header / string parameter / uint8[] binary
//   RequestHeader.msg:   RequestIdentity identity / RequestLease lease / RequestPolicy policy
//   RequestIdentity.msg: int64 id / int64 api_id
//   RequestLease.msg:    int64 id
//   RequestPolicy.msg:   int32 priority / bool noreply

/// `unitree_api/msg/RequestIdentity` — `int64 id` / `int64 api_id`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RequestIdentity {
    pub id: i64,
    pub api_id: i64,
}

/// `unitree_api/msg/RequestLease` — `int64 id`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RequestLease {
    pub id: i64,
}

/// `unitree_api/msg/RequestPolicy` — `int32 priority` / `bool noreply`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RequestPolicy {
    pub priority: i32,
    pub noreply: bool,
}

/// `unitree_api/msg/RequestHeader` — nested identity / lease / policy.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RequestHeader {
    pub identity: RequestIdentity,
    pub lease: RequestLease,
    pub policy: RequestPolicy,
}

/// `unitree_api/msg/Request` — the sport-mode command envelope. The command
/// payload rides in `parameter` as a JSON string (e.g. api_id 1008 "Move" ->
/// `{"x":vx,"y":vy,"z":vyaw}`); `binary` is an unused-in-v1 `uint8[]`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Request {
    pub header: RequestHeader,
    pub parameter: String,
    pub binary: Vec<u8>,
}

/// Unitree sport-mode API id for the `Move` command (velocity in `parameter`).
pub const SPORT_API_ID_MOVE: i64 = 1008;
