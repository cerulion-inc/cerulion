// SPDX-License-Identifier: AGPL-3.0-only
//! The codec registry — the PURE seam between config `ros_type` strings and
//! the `cerulion_go2_dds` codec set.
//!
//! The typed path supports exactly the `cerulion_go2_dds` message set; each type owns one
//! FIXED output port on the [`crate::DdsBridge`] node (the `#[cerulion_node]`
//! macro declares ports at compile time, so a config-mapped bridge binds
//! runtime mappings onto a fixed port set — any OTHER schema-resolvable type
//! takes the raw-generic path in [`crate::generic`], which creates its
//! publisher at run time instead of using a hand-mapped port).
//!
//! Two dispatch surfaces:
//! - [`RosType::parse`] / [`RosType::port_name`] — config-time resolution
//!   (unknown type = the loud [`crate::config::ConfigError::UnknownRosType`]).
//! - [`decode_sample`] — the bytes→struct seam over the PURE codecs
//!   (`cerulion_go2_dds::cdr`). The production pump uses TYPED ros2-client
//!   subscriptions, which deserialize into the same structs via the
//!   same serde-CDR engine (wire-identical — see lib/cerulion_go2_dds docs);
//!   this fn is the engine-equivalence seam for byte-level tests and any
//!   raw-payload path.

use cerulion_go2_dds::cdr::{
    decode_point_cloud2, decode_request, decode_sport_mode_state, decode_twist, CdrError,
};
use cerulion_go2_dds::messages::{PointCloud2, Request, SportModeState, Twist};
use serde::{Deserialize, Serialize};

/// The typed ROS types — the `cerulion_go2_dds` codec set. Order is the FIXED
/// port order (drain determinism keys off it: [`crate::queue::SampleQueue`]
/// drains ports in this declaration order).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RosType {
    /// `sensor_msgs/PointCloud2` → port `cloud` (schema
    /// `sensor_msgs/PointCloud2`).
    PointCloud2,
    /// `unitree_go/SportModeState` → port `odom` (schema `nav_msgs/Odometry`
    /// — a documented PROJECTION: pose + twist; see `crate::mapping`).
    SportModeState,
    /// `geometry_msgs/Twist` → port `twist` (schema
    /// `geometry_msgs/TwistStamped` — stamped so the output-discard gate has a
    /// variable field; see the lib.rs partial-fire note).
    Twist,
    /// `unitree_api/Request` → port `request_json` (schema `std_msgs/String`
    /// carrying the request's `parameter` JSON — an observability projection).
    Request,
}

/// All supported types in fixed port order.
pub const ALL_ROS_TYPES: [RosType; 4] = [
    RosType::PointCloud2,
    RosType::SportModeState,
    RosType::Twist,
    RosType::Request,
];

impl RosType {
    /// Resolve a config `ros_type` string (`pkg/Type`, exact match).
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "sensor_msgs/PointCloud2" => Self::PointCloud2,
            "unitree_go/SportModeState" => Self::SportModeState,
            "geometry_msgs/Twist" => Self::Twist,
            "unitree_api/Request" => Self::Request,
            _ => return None,
        })
    }

    /// The canonical `pkg/Type` string (the inverse of [`Self::parse`]).
    pub const fn ros_type_str(self) -> &'static str {
        match self {
            Self::PointCloud2 => "sensor_msgs/PointCloud2",
            Self::SportModeState => "unitree_go/SportModeState",
            Self::Twist => "geometry_msgs/Twist",
            Self::Request => "unitree_api/Request",
        }
    }

    /// The `(package, type)` pair for ros2-client's `MessageTypeName`.
    pub fn pkg_and_type(self) -> (&'static str, &'static str) {
        let s = self.ros_type_str();
        let (pkg, ty) = s.split_once('/').expect("ros_type_str is always pkg/Type");
        (pkg, ty)
    }

    /// The node output port this type publishes on (a fixed port — see the
    /// module docs).
    pub const fn port_name(self) -> &'static str {
        match self {
            Self::PointCloud2 => "cloud",
            Self::SportModeState => "odom",
            Self::Twist => "twist",
            Self::Request => "request_json",
        }
    }

    /// The Cerulion schema the port publishes (what the graph YAML declares).
    pub const fn cerulion_schema(self) -> &'static str {
        match self {
            Self::PointCloud2 => "sensor_msgs/PointCloud2",
            Self::SportModeState => "nav_msgs/Odometry",
            Self::Twist => "geometry_msgs/TwistStamped",
            Self::Request => "std_msgs/String",
        }
    }

    /// The fixed queue-slot index (port order).
    pub const fn slot(self) -> usize {
        match self {
            Self::PointCloud2 => 0,
            Self::SportModeState => 1,
            Self::Twist => 2,
            Self::Request => 3,
        }
    }

    /// Human-readable supported-set list for error messages.
    pub fn supported_list() -> String {
        ALL_ROS_TYPES
            .iter()
            .map(|t| t.ros_type_str())
            .collect::<Vec<_>>()
            .join(", ")
    }
}

/// One decoded DDS sample, tagged by its type (and hence its port).
///
/// `Serialize`/`Deserialize` are here so [`crate::queue::SampleQueue`]
/// can carry its in-flight slots through the `#[cerulion(serde)]` escape. The
/// alternative is `#[derive(CerulionState)]`, which would need the same on the
/// four payload types — and those live in `cerulion_go2_dds`, a pure DDS-interop
/// lib with no `cerulion_core` dependency and no reason to acquire one. They
/// already derive serde, so this is the cheaper half of the same choice.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum BridgeSample {
    Cloud(PointCloud2),
    Sport(SportModeState),
    Twist(Twist),
    Request(Request),
}

impl BridgeSample {
    /// The type (and hence port/slot) this sample belongs to.
    pub fn ros_type(&self) -> RosType {
        match self {
            Self::Cloud(_) => RosType::PointCloud2,
            Self::Sport(_) => RosType::SportModeState,
            Self::Twist(_) => RosType::Twist,
            Self::Request(_) => RosType::Request,
        }
    }
}

/// Decode a full CDR DDS payload (encapsulation header + body) into a tagged
/// sample via the `cerulion_go2_dds` PURE codecs. The bytes→struct seam (see the module
/// docs for how this relates to the typed production path).
pub fn decode_sample(ros_type: RosType, payload: &[u8]) -> Result<BridgeSample, CdrError> {
    Ok(match ros_type {
        RosType::PointCloud2 => BridgeSample::Cloud(decode_point_cloud2(payload)?),
        RosType::SportModeState => BridgeSample::Sport(decode_sport_mode_state(payload)?),
        RosType::Twist => BridgeSample::Twist(decode_twist(payload)?),
        RosType::Request => BridgeSample::Request(decode_request(payload)?),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_go2_dds::cdr::{
        encode_point_cloud2, encode_request, encode_sport_mode_state, encode_twist,
    };
    use cerulion_go2_dds::messages::{
        Header, ImuState, PointField, RequestHeader, RequestIdentity, RequestLease, RequestPolicy,
        Time, Vector3,
    };

    #[test]
    fn parse_round_trips_every_supported_type() {
        for t in ALL_ROS_TYPES {
            assert_eq!(RosType::parse(t.ros_type_str()), Some(t));
        }
        // Near-misses are None (exact match — no case forgiveness, no msg/
        // infix: the config must spell the canonical pkg/Type form).
        for s in [
            "sensor_msgs/pointcloud2",
            "sensor_msgs/msg/PointCloud2",
            "PointCloud2",
            "",
        ] {
            assert_eq!(RosType::parse(s), None, "{s:?} must not resolve");
        }
    }

    #[test]
    fn port_schema_and_slot_tables_are_consistent() {
        // Slots are the 0..4 port order, ports unique, pkg/type splits clean.
        let mut seen_ports = Vec::new();
        for (i, t) in ALL_ROS_TYPES.iter().enumerate() {
            assert_eq!(t.slot(), i, "slot order == declaration order");
            assert!(!seen_ports.contains(&t.port_name()), "ports unique");
            seen_ports.push(t.port_name());
            let (pkg, ty) = t.pkg_and_type();
            assert_eq!(format!("{pkg}/{ty}"), t.ros_type_str());
        }
        assert_eq!(RosType::PointCloud2.port_name(), "cloud");
        assert_eq!(
            RosType::SportModeState.cerulion_schema(),
            "nav_msgs/Odometry"
        );
        assert_eq!(
            RosType::Twist.cerulion_schema(),
            "geometry_msgs/TwistStamped"
        );
    }

    #[test]
    fn supported_list_names_all_four() {
        let l = RosType::supported_list();
        for t in ALL_ROS_TYPES {
            assert!(l.contains(t.ros_type_str()), "{l}");
        }
    }

    #[test]
    fn decode_sample_dispatches_to_the_chunk1_codec() {
        // Encode via the `cerulion_go2_dds` codecs (whose bytes are themselves pinned to
        // hand oracles in lib/cerulion_go2_dds), then dispatch-decode: the
        // registry must route each type to ITS codec and tag the sample.
        let twist = Twist {
            linear: Vector3 {
                x: 0.5,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.25,
            },
        };
        let payload = encode_twist(&twist).unwrap();
        let sample = decode_sample(RosType::Twist, &payload).expect("twist decodes");
        assert_eq!(sample.ros_type(), RosType::Twist);
        assert_eq!(sample, BridgeSample::Twist(twist));

        let req = Request {
            header: RequestHeader {
                identity: RequestIdentity {
                    id: 7,
                    api_id: 1008,
                },
                lease: RequestLease { id: 0 },
                policy: RequestPolicy {
                    priority: 0,
                    noreply: false,
                },
            },
            parameter: r#"{"x":0.1,"y":0.0,"z":0.0}"#.to_string(),
            binary: Vec::new(),
        };
        let payload = encode_request(&req).unwrap();
        let sample = decode_sample(RosType::Request, &payload).expect("request decodes");
        assert_eq!(sample, BridgeSample::Request(req));

        // A WRONG-type dispatch on those bytes must not panic: it either errs
        // or mis-decodes into a struct — the registry's job is routing, and
        // the config layer guarantees the route; this pins totality only.
        let _ = decode_sample(RosType::PointCloud2, &payload);
    }

    #[test]
    fn decode_sample_positive_pointcloud2_and_sportmodestate() {
        // The PointCloud2 and SportModeState arms, POSITIVELY
        // pinned: encode a hand-built struct via its
        // `cerulion_go2_dds` codec (itself byte-oracle-anchored in lib/cerulion_go2_dds),
        // dispatch-decode, and assert the EXACT tagged BridgeSample.

        // PointCloud2: the (1,2,3)/(4,5,6) LE-f32 two-point cloud.
        let pc = PointCloud2 {
            header: Header {
                stamp: Time {
                    sec: 12,
                    nanosec: 34,
                },
                frame_id: "lidar".to_string(),
            },
            height: 1,
            width: 2,
            fields: [("x", 0u32), ("y", 4), ("z", 8)]
                .into_iter()
                .map(|(n, off)| PointField {
                    name: n.to_string(),
                    offset: off,
                    datatype: 7, // FLOAT32
                    count: 1,
                })
                .collect(),
            is_bigendian: false,
            point_step: 12,
            row_step: 24,
            data: vec![
                0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, // (1.0, 2.0, 3.0)
                0, 0, 128, 64, 0, 0, 160, 64, 0, 0, 192, 64, // (4.0, 5.0, 6.0)
            ],
            is_dense: true,
        };
        let payload = encode_point_cloud2(&pc).expect("encode cloud");
        let sample = decode_sample(RosType::PointCloud2, &payload).expect("cloud decodes");
        assert_eq!(sample.ros_type(), RosType::PointCloud2);
        assert_eq!(sample, BridgeSample::Cloud(pc));

        // SportModeState: distinct fields so a mis-route shows a wrong value.
        let s = SportModeState {
            stamp: Time {
                sec: 100,
                nanosec: 250_000_000,
            },
            error_code: 7,
            imu_state: ImuState {
                quaternion: [0.5, 0.1, 0.2, 0.3],
                ..Default::default()
            },
            position: [1.5, -2.5, 0.25],
            velocity: [0.5, 0.0, 0.0],
            yaw_speed: 0.25,
            ..Default::default()
        };
        let payload = encode_sport_mode_state(&s).expect("encode sport");
        let sample = decode_sample(RosType::SportModeState, &payload).expect("sport decodes");
        assert_eq!(sample.ros_type(), RosType::SportModeState);
        assert_eq!(sample, BridgeSample::Sport(s));
    }
}
