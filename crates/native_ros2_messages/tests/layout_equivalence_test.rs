// SPDX-License-Identifier: AGPL-3.0-only
//! Equivalence proof between the RUNTIME wire-layout
//! calculator (`cerulion_core::codegen::layout::LayoutResolver`) and the
//! COMPILE-TIME `#[repr(C)]` layout rustc gives the generated structs.
//!
//! This is the linchpin for rmw_cerulion: the rmw bridge flattens C
//! messages at runtime using `LayoutResolver`; native Cerulion nodes
//! read the same topics through the generated structs. A single byte of
//! divergence means cross-runtime misreads under a matching schema
//! hash — so this test is EXHAUSTIVE over all 254 vendored schemas
//! (fixed sizes + schema hashes) plus `offset_of!` spot checks on the
//! trickiest layouts (nested composition, padding, large arrays).
//!
//! The schema set is re-parsed from the same `msg/` tree the build
//! script used — same pipeline, independent execution.

use cerulion_core::codegen::layout::LayoutResolver;
use cerulion_core::codegen::parse_rosmsg;
use cerulion_core::message::ShmMessage;
use std::fs;
use std::mem::offset_of;
use std::path::Path;

fn build_resolver() -> LayoutResolver {
    let msg_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("msg");
    let mut schemas = Vec::new();
    let mut packages: Vec<_> = fs::read_dir(&msg_dir)
        .expect("msg dir")
        .filter_map(|e| e.ok())
        .filter(|e| e.path().is_dir())
        .collect();
    packages.sort_by_key(|e| e.file_name());
    for pkg_entry in packages {
        let pkg = pkg_entry.file_name().to_string_lossy().replace('-', "_");
        let mut files: Vec<_> = fs::read_dir(pkg_entry.path())
            .expect("pkg dir")
            .filter_map(|e| e.ok())
            .filter(|e| e.path().extension().is_some_and(|x| x == "msg"))
            .collect();
        files.sort_by_key(|e| e.file_name());
        for f in files {
            let name = f
                .path()
                .file_stem()
                .expect("stem")
                .to_string_lossy()
                .to_string();
            let content = fs::read_to_string(f.path()).expect("read msg");
            schemas.push(parse_rosmsg(&content, &name, Some(&pkg)).expect("parse"));
        }
    }
    // Guard the EXHAUSTIVE claim: a newly vendored .msg must extend the
    // assert_layout_matches list below, or this fails loudly instead of
    // the headline silently decaying.
    assert_eq!(
        schemas.len(),
        254,
        "vendored .msg count changed — extend the exhaustive layout list below"
    );
    let (resolver, warnings) = LayoutResolver::new(schemas);
    assert!(warnings.is_empty(), "resolution warnings: {warnings:?}");
    resolver
}

macro_rules! assert_layout_matches {
    ($resolver:ident, $pkg:ident, $name:ident) => {{
        let qualified = concat!(stringify!($pkg), "/", stringify!($name));
        let layout = $resolver
            .layout_of(qualified)
            .unwrap_or_else(|| panic!("no layout for {}", qualified));
        assert_eq!(
            layout.fixed_size,
            <native_ros2_messages::$pkg::$name as ShmMessage>::WIRE_FIXED_SIZE,
            "fixed-section size mismatch for {}",
            qualified
        );
        assert_eq!(
            layout.schema_hash,
            <native_ros2_messages::$pkg::$name as ShmMessage>::SCHEMA_HASH,
            "schema hash mismatch for {}",
            qualified
        );
        assert_eq!(
            layout.variable_fields.len(),
            <native_ros2_messages::$pkg::$name as ShmMessage>::VARIABLE_FIELD_COUNT,
            "variable-field count mismatch for {}",
            qualified
        );
    }};
}

/// EXHAUSTIVE: computed fixed size + hash + variable count equal the
/// generated constants for every vendored schema.
#[test]
fn computed_layout_matches_generated_constants_for_all_schemas() {
    let mut r = build_resolver();

    assert_layout_matches!(r, builtin_interfaces, Time);
    assert_layout_matches!(r, builtin_interfaces, Duration);
    assert_layout_matches!(r, std_msgs, Bool);
    assert_layout_matches!(r, std_msgs, Byte);
    assert_layout_matches!(r, std_msgs, Char);
    assert_layout_matches!(r, std_msgs, ColorRGBA);
    assert_layout_matches!(r, std_msgs, Empty);
    assert_layout_matches!(r, std_msgs, Float32);
    assert_layout_matches!(r, std_msgs, Float64);
    assert_layout_matches!(r, std_msgs, Int8);
    assert_layout_matches!(r, std_msgs, Int16);
    assert_layout_matches!(r, std_msgs, Int32);
    assert_layout_matches!(r, std_msgs, Int64);
    assert_layout_matches!(r, std_msgs, UInt8);
    assert_layout_matches!(r, std_msgs, UInt16);
    assert_layout_matches!(r, std_msgs, UInt32);
    assert_layout_matches!(r, std_msgs, UInt64);
    assert_layout_matches!(r, geometry_msgs, Point);
    assert_layout_matches!(r, geometry_msgs, Point32);
    assert_layout_matches!(r, geometry_msgs, Vector3);
    assert_layout_matches!(r, geometry_msgs, Quaternion);
    assert_layout_matches!(r, geometry_msgs, Pose2D);
    assert_layout_matches!(r, geometry_msgs, Accel);
    assert_layout_matches!(r, geometry_msgs, AccelWithCovariance);
    assert_layout_matches!(r, geometry_msgs, Inertia);
    assert_layout_matches!(r, geometry_msgs, Pose);
    assert_layout_matches!(r, geometry_msgs, PoseWithCovariance);
    assert_layout_matches!(r, geometry_msgs, Transform);
    assert_layout_matches!(r, geometry_msgs, Twist);
    assert_layout_matches!(r, geometry_msgs, TwistWithCovariance);
    assert_layout_matches!(r, geometry_msgs, Wrench);
    assert_layout_matches!(r, sensor_msgs, JoyFeedback);
    assert_layout_matches!(r, sensor_msgs, NavSatStatus);
    assert_layout_matches!(r, sensor_msgs, RegionOfInterest);
    assert_layout_matches!(r, nav_msgs, MapMetaData);
    assert_layout_matches!(r, visualization_msgs, UVCoordinate);
    assert_layout_matches!(r, shape_msgs, MeshTriangle);
    assert_layout_matches!(r, shape_msgs, Plane);
    assert_layout_matches!(r, statistics_msgs, StatisticDataPoint);
    assert_layout_matches!(r, statistics_msgs, StatisticDataType);
    assert_layout_matches!(r, action_msgs, GoalInfo);
    assert_layout_matches!(r, action_msgs, GoalStatus);
    assert_layout_matches!(r, moveit_msgs, CartesianPoint);
    assert_layout_matches!(r, moveit_msgs, CartesianTrajectoryPoint);
    assert_layout_matches!(r, moveit_msgs, ConstraintEvalResult);
    assert_layout_matches!(r, moveit_msgs, CostSource);
    assert_layout_matches!(r, moveit_msgs, MoveItErrorCodes);
    assert_layout_matches!(r, moveit_msgs, OrientedBoundingBox);
    assert_layout_matches!(r, moveit_msgs, PlanningSceneComponents);
    assert_layout_matches!(r, control_msgs, CANopenState);
    assert_layout_matches!(r, control_msgs, GripperCommand);
    assert_layout_matches!(r, control_msgs, SpeedScalingFactor);
    assert_layout_matches!(r, std_msgs, MultiArrayDimension);
    assert_layout_matches!(r, std_msgs, MultiArrayLayout);
    assert_layout_matches!(r, std_msgs, Header);
    assert_layout_matches!(r, std_msgs, String);
    assert_layout_matches!(r, std_msgs, Int8MultiArray);
    assert_layout_matches!(r, std_msgs, Int16MultiArray);
    assert_layout_matches!(r, std_msgs, Int32MultiArray);
    assert_layout_matches!(r, std_msgs, Int64MultiArray);
    assert_layout_matches!(r, std_msgs, UInt8MultiArray);
    assert_layout_matches!(r, std_msgs, UInt16MultiArray);
    assert_layout_matches!(r, std_msgs, UInt32MultiArray);
    assert_layout_matches!(r, std_msgs, UInt64MultiArray);
    assert_layout_matches!(r, std_msgs, Float32MultiArray);
    assert_layout_matches!(r, std_msgs, Float64MultiArray);
    assert_layout_matches!(r, std_msgs, ByteMultiArray);
    assert_layout_matches!(r, geometry_msgs, AccelStamped);
    assert_layout_matches!(r, geometry_msgs, AccelWithCovarianceStamped);
    assert_layout_matches!(r, geometry_msgs, InertiaStamped);
    assert_layout_matches!(r, geometry_msgs, PointStamped);
    assert_layout_matches!(r, geometry_msgs, Polygon);
    assert_layout_matches!(r, geometry_msgs, PolygonInstance);
    assert_layout_matches!(r, geometry_msgs, PolygonInstanceStamped);
    assert_layout_matches!(r, geometry_msgs, PolygonStamped);
    assert_layout_matches!(r, geometry_msgs, PoseArray);
    assert_layout_matches!(r, geometry_msgs, PoseStamped);
    assert_layout_matches!(r, geometry_msgs, PoseWithCovarianceStamped);
    assert_layout_matches!(r, geometry_msgs, QuaternionStamped);
    assert_layout_matches!(r, geometry_msgs, TransformStamped);
    assert_layout_matches!(r, geometry_msgs, TwistStamped);
    assert_layout_matches!(r, geometry_msgs, TwistWithCovarianceStamped);
    assert_layout_matches!(r, geometry_msgs, Vector3Stamped);
    assert_layout_matches!(r, geometry_msgs, VelocityStamped);
    assert_layout_matches!(r, geometry_msgs, WrenchStamped);
    assert_layout_matches!(r, sensor_msgs, BatteryState);
    assert_layout_matches!(r, sensor_msgs, CameraInfo);
    assert_layout_matches!(r, sensor_msgs, ChannelFloat32);
    assert_layout_matches!(r, sensor_msgs, CompressedImage);
    assert_layout_matches!(r, sensor_msgs, FluidPressure);
    assert_layout_matches!(r, sensor_msgs, Illuminance);
    assert_layout_matches!(r, sensor_msgs, Image);
    assert_layout_matches!(r, sensor_msgs, Imu);
    assert_layout_matches!(r, sensor_msgs, JointState);
    assert_layout_matches!(r, sensor_msgs, Joy);
    assert_layout_matches!(r, sensor_msgs, JoyFeedbackArray);
    assert_layout_matches!(r, sensor_msgs, LaserEcho);
    assert_layout_matches!(r, sensor_msgs, LaserScan);
    assert_layout_matches!(r, sensor_msgs, MagneticField);
    assert_layout_matches!(r, sensor_msgs, MultiDOFJointState);
    assert_layout_matches!(r, sensor_msgs, MultiEchoLaserScan);
    assert_layout_matches!(r, sensor_msgs, NavSatFix);
    assert_layout_matches!(r, sensor_msgs, PointCloud);
    assert_layout_matches!(r, sensor_msgs, PointCloud2);
    assert_layout_matches!(r, sensor_msgs, PointField);
    assert_layout_matches!(r, sensor_msgs, Range);
    assert_layout_matches!(r, sensor_msgs, RelativeHumidity);
    assert_layout_matches!(r, sensor_msgs, Temperature);
    assert_layout_matches!(r, sensor_msgs, TimeReference);
    assert_layout_matches!(r, nav_msgs, GridCells);
    assert_layout_matches!(r, nav_msgs, OccupancyGrid);
    assert_layout_matches!(r, nav_msgs, Odometry);
    assert_layout_matches!(r, nav_msgs, Path);
    assert_layout_matches!(r, visualization_msgs, ImageMarker);
    assert_layout_matches!(r, visualization_msgs, InteractiveMarker);
    assert_layout_matches!(r, visualization_msgs, InteractiveMarkerControl);
    assert_layout_matches!(r, visualization_msgs, InteractiveMarkerFeedback);
    assert_layout_matches!(r, visualization_msgs, InteractiveMarkerInit);
    assert_layout_matches!(r, visualization_msgs, InteractiveMarkerPose);
    assert_layout_matches!(r, visualization_msgs, InteractiveMarkerUpdate);
    assert_layout_matches!(r, visualization_msgs, Marker);
    assert_layout_matches!(r, visualization_msgs, MarkerArray);
    assert_layout_matches!(r, visualization_msgs, MenuEntry);
    assert_layout_matches!(r, visualization_msgs, MeshFile);
    assert_layout_matches!(r, trajectory_msgs, JointTrajectory);
    assert_layout_matches!(r, trajectory_msgs, JointTrajectoryPoint);
    assert_layout_matches!(r, trajectory_msgs, MultiDOFJointTrajectory);
    assert_layout_matches!(r, trajectory_msgs, MultiDOFJointTrajectoryPoint);
    assert_layout_matches!(r, shape_msgs, Mesh);
    assert_layout_matches!(r, shape_msgs, SolidPrimitive);
    assert_layout_matches!(r, diagnostic_msgs, DiagnosticArray);
    assert_layout_matches!(r, diagnostic_msgs, DiagnosticStatus);
    assert_layout_matches!(r, diagnostic_msgs, KeyValue);
    assert_layout_matches!(r, statistics_msgs, MetricsMessage);
    assert_layout_matches!(r, action_msgs, GoalStatusArray);
    assert_layout_matches!(r, tf2_msgs, TFMessage);
    assert_layout_matches!(r, tf2_msgs, TF2Error);
    assert_layout_matches!(r, moveit_msgs, AllowedCollisionEntry);
    assert_layout_matches!(r, moveit_msgs, AllowedCollisionMatrix);
    assert_layout_matches!(r, moveit_msgs, AttachedCollisionObject);
    assert_layout_matches!(r, moveit_msgs, BoundingVolume);
    assert_layout_matches!(r, moveit_msgs, CartesianTrajectory);
    assert_layout_matches!(r, moveit_msgs, CollisionObject);
    assert_layout_matches!(r, moveit_msgs, Constraints);
    assert_layout_matches!(r, moveit_msgs, ContactInformation);
    assert_layout_matches!(r, moveit_msgs, DisplayRobotState);
    assert_layout_matches!(r, moveit_msgs, DisplayTrajectory);
    assert_layout_matches!(r, moveit_msgs, GenericTrajectory);
    assert_layout_matches!(r, moveit_msgs, Grasp);
    assert_layout_matches!(r, moveit_msgs, GripperTranslation);
    assert_layout_matches!(r, moveit_msgs, JointConstraint);
    assert_layout_matches!(r, moveit_msgs, JointLimits);
    assert_layout_matches!(r, moveit_msgs, KinematicSolverInfo);
    assert_layout_matches!(r, moveit_msgs, LinkPadding);
    assert_layout_matches!(r, moveit_msgs, LinkScale);
    assert_layout_matches!(r, moveit_msgs, MotionPlanDetailedResponse);
    assert_layout_matches!(r, moveit_msgs, MotionPlanRequest);
    assert_layout_matches!(r, moveit_msgs, MotionPlanResponse);
    assert_layout_matches!(r, moveit_msgs, MotionSequenceItem);
    assert_layout_matches!(r, moveit_msgs, MotionSequenceRequest);
    assert_layout_matches!(r, moveit_msgs, MotionSequenceResponse);
    assert_layout_matches!(r, moveit_msgs, ObjectColor);
    assert_layout_matches!(r, moveit_msgs, OrientationConstraint);
    assert_layout_matches!(r, moveit_msgs, PlaceLocation);
    assert_layout_matches!(r, moveit_msgs, PlannerInterfaceDescription);
    assert_layout_matches!(r, moveit_msgs, PlannerParams);
    assert_layout_matches!(r, moveit_msgs, PlanningOptions);
    assert_layout_matches!(r, moveit_msgs, PlanningScene);
    assert_layout_matches!(r, moveit_msgs, PlanningSceneWorld);
    assert_layout_matches!(r, moveit_msgs, PositionConstraint);
    assert_layout_matches!(r, moveit_msgs, PositionIKRequest);
    assert_layout_matches!(r, moveit_msgs, RobotState);
    assert_layout_matches!(r, moveit_msgs, RobotTrajectory);
    assert_layout_matches!(r, moveit_msgs, TrajectoryConstraints);
    assert_layout_matches!(r, moveit_msgs, VisibilityConstraint);
    assert_layout_matches!(r, moveit_msgs, WorkspaceParameters);
    assert_layout_matches!(r, control_msgs, AdmittanceControllerState);
    assert_layout_matches!(r, control_msgs, BatteryStateArray);
    assert_layout_matches!(r, control_msgs, DynamicInterfaceGroupValues);
    assert_layout_matches!(r, control_msgs, DynamicInterfaceValues);
    assert_layout_matches!(r, control_msgs, DynamicJointState);
    assert_layout_matches!(r, control_msgs, EtherCATState);
    assert_layout_matches!(r, control_msgs, Float64Values);
    assert_layout_matches!(r, control_msgs, GenericHardwareState);
    assert_layout_matches!(r, control_msgs, HardwareDeviceDiagnostics);
    assert_layout_matches!(r, control_msgs, HardwareDeviceStatus);
    assert_layout_matches!(r, control_msgs, HardwareDiagnostics);
    assert_layout_matches!(r, control_msgs, HardwareStatus);
    assert_layout_matches!(r, control_msgs, InterfaceValue);
    assert_layout_matches!(r, control_msgs, JointCommand);
    assert_layout_matches!(r, control_msgs, JointComponentTolerance);
    assert_layout_matches!(r, control_msgs, JointControllerState);
    assert_layout_matches!(r, control_msgs, JointJog);
    assert_layout_matches!(r, control_msgs, JointTolerance);
    assert_layout_matches!(r, control_msgs, JointTrajectoryControllerState);
    assert_layout_matches!(r, control_msgs, JointWrenchTrajectory);
    assert_layout_matches!(r, control_msgs, JointWrenchTrajectoryPoint);
    assert_layout_matches!(r, control_msgs, Keys);
    assert_layout_matches!(r, control_msgs, MecanumDriveControllerState);
    assert_layout_matches!(r, control_msgs, MotionArgument);
    assert_layout_matches!(r, control_msgs, MotionPrimitive);
    assert_layout_matches!(r, control_msgs, MotionPrimitiveSequence);
    assert_layout_matches!(r, control_msgs, MultiDOFCommand);
    assert_layout_matches!(r, control_msgs, MultiDOFStateStamped);
    assert_layout_matches!(r, control_msgs, PidState);
    assert_layout_matches!(r, control_msgs, SingleDOFState);
    assert_layout_matches!(r, control_msgs, SingleDOFStateStamped);
    assert_layout_matches!(r, control_msgs, SteeringControllerCommand);
    assert_layout_matches!(r, control_msgs, SteeringControllerStatus);
    assert_layout_matches!(r, control_msgs, VDA5050SafetyState);
    assert_layout_matches!(r, control_msgs, VDA5050State);
    assert_layout_matches!(r, control_msgs, WrenchFramed);
    assert_layout_matches!(r, octomap_msgs, Octomap);
    assert_layout_matches!(r, octomap_msgs, OctomapWithPose);
    assert_layout_matches!(r, object_recognition_msgs, ObjectInformation);
    assert_layout_matches!(r, object_recognition_msgs, ObjectType);
    assert_layout_matches!(r, object_recognition_msgs, RecognizedObject);
    assert_layout_matches!(r, object_recognition_msgs, RecognizedObjectArray);
    assert_layout_matches!(r, object_recognition_msgs, Table);
    assert_layout_matches!(r, object_recognition_msgs, TableArray);

    // Vendored perception packages
    // (grid_map_msgs, radar_msgs, unique_identifier_msgs, vision_msgs).
    assert_layout_matches!(r, grid_map_msgs, GridMap);
    assert_layout_matches!(r, grid_map_msgs, GridMapInfo);
    assert_layout_matches!(r, radar_msgs, RadarReturn);
    assert_layout_matches!(r, radar_msgs, RadarScan);
    assert_layout_matches!(r, radar_msgs, RadarTrack);
    assert_layout_matches!(r, radar_msgs, RadarTracks);
    assert_layout_matches!(r, unique_identifier_msgs, UUID);
    assert_layout_matches!(r, vision_msgs, BoundingBox2D);
    assert_layout_matches!(r, vision_msgs, BoundingBox2DArray);
    assert_layout_matches!(r, vision_msgs, BoundingBox3D);
    assert_layout_matches!(r, vision_msgs, BoundingBox3DArray);
    assert_layout_matches!(r, vision_msgs, Classification);
    assert_layout_matches!(r, vision_msgs, Detection2D);
    assert_layout_matches!(r, vision_msgs, Detection2DArray);
    assert_layout_matches!(r, vision_msgs, Detection3D);
    assert_layout_matches!(r, vision_msgs, Detection3DArray);
    assert_layout_matches!(r, vision_msgs, LabelInfo);
    assert_layout_matches!(r, vision_msgs, ObjectHypothesis);
    assert_layout_matches!(r, vision_msgs, ObjectHypothesisWithPose);
    assert_layout_matches!(r, vision_msgs, Point2D);
    assert_layout_matches!(r, vision_msgs, Pose2D);
    assert_layout_matches!(r, vision_msgs, VisionClass);
    assert_layout_matches!(r, vision_msgs, VisionInfo);

    // Vendored Autoware perception + planning
    // packages (faithful reconstructions of autoware_perception_msgs /
    // autoware_planning_msgs).
    assert_layout_matches!(r, autoware_perception_msgs, ObjectClassification);
    assert_layout_matches!(r, autoware_perception_msgs, Shape);
    assert_layout_matches!(r, autoware_perception_msgs, PredictedPath);
    assert_layout_matches!(r, autoware_perception_msgs, PredictedObjectKinematics);
    assert_layout_matches!(r, autoware_perception_msgs, PredictedObject);
    assert_layout_matches!(r, autoware_perception_msgs, PredictedObjects);
    assert_layout_matches!(r, autoware_planning_msgs, TrajectoryPoint);
    assert_layout_matches!(r, autoware_planning_msgs, Trajectory);
    assert_layout_matches!(r, autoware_planning_msgs, LaneletPrimitive);
    assert_layout_matches!(r, autoware_planning_msgs, LaneletSegment);
    assert_layout_matches!(r, autoware_planning_msgs, LaneletRoute);
}

/// Field-offset spot checks via `offset_of!` on the trickiest layouts.
#[test]
fn field_offsets_match_repr_c_spot_checks() {
    use native_ros2_messages::action_msgs::GoalInfoShm;
    use native_ros2_messages::geometry_msgs::{PoseShm, PoseWithCovarianceShm, TransformShm};
    use native_ros2_messages::radar_msgs::RadarTrackShm;
    use native_ros2_messages::sensor_msgs::ImageFixedSection;

    let mut r = build_resolver();

    // Pose: position (Point, 24 B) @ 0, orientation (Quaternion) @ 24.
    let pose = r.layout_of("geometry_msgs/Pose").expect("pose");
    assert_eq!(pose.fixed_fields[0].offset, offset_of!(PoseShm, position));
    assert_eq!(
        pose.fixed_fields[1].offset,
        offset_of!(PoseShm, orientation)
    );

    // Transform: translation (Vector3) @ 0, rotation (Quaternion) @ 24.
    let tf = r.layout_of("geometry_msgs/Transform").expect("tf");
    assert_eq!(
        tf.fixed_fields[0].offset,
        offset_of!(TransformShm, translation)
    );
    assert_eq!(
        tf.fixed_fields[1].offset,
        offset_of!(TransformShm, rotation)
    );

    // PoseWithCovariance: pose @ 0 (56 B), covariance f64[36] @ 56.
    let pwc = r
        .layout_of("geometry_msgs/PoseWithCovariance")
        .expect("pwc");
    assert_eq!(
        pwc.fixed_fields[0].offset,
        offset_of!(PoseWithCovarianceShm, pose)
    );
    assert_eq!(
        pwc.fixed_fields[1].offset,
        offset_of!(PoseWithCovarianceShm, covariance)
    );

    // GoalInfo: goal_id (unique_identifier_msgs/UUID — a nested FIXED message
    // wrapping uint8[16], so still 16 bytes / align 1) @ 0, stamp (Time:
    // i32+u32) @ 16. A bare `uint8[16]` field has the same offsets as
    // upstream's UUID, so these asserts cannot tell the two apart; the
    // field TYPE is pinned by `upstream_drift_test`.
    let gi = r.layout_of("action_msgs/GoalInfo").expect("goalinfo");
    assert_eq!(gi.fixed_fields[0].offset, offset_of!(GoalInfoShm, goal_id));
    assert_eq!(gi.fixed_fields[1].offset, offset_of!(GoalInfoShm, stamp));

    // Image fixed section: height/width/is_bigendian/step with padding
    // around the u8.
    let img = r.layout_of("sensor_msgs/Image").expect("image");
    let by_name = |layout: &cerulion_core::codegen::layout::WireLayout, n: &str| {
        layout
            .fixed_fields
            .iter()
            .find(|f| f.name == n)
            .unwrap_or_else(|| panic!("missing field {n}"))
            .offset
    };
    assert_eq!(
        by_name(&img, "height"),
        offset_of!(ImageFixedSection, height)
    );
    assert_eq!(by_name(&img, "width"), offset_of!(ImageFixedSection, width));
    assert_eq!(
        by_name(&img, "is_bigendian"),
        offset_of!(ImageFixedSection, is_bigendian)
    );
    assert_eq!(by_name(&img, "step"), offset_of!(ImageFixedSection, step));

    // RadarTrack: the trickiest complex-fixed layout —
    // inlines unique_identifier_msgs/UUID (u8[16], align 1) +
    // geometry_msgs/Point (3×f64) + three Vector3, then a u16 and four
    // float32[6] arrays. Spot-check the nested-inline composition AND the
    // u16 → [f32; 6] alignment transition (the u16 forces 2 B of padding
    // before the align-4 covariance arrays).
    let rt = r.layout_of("radar_msgs/RadarTrack").expect("radartrack");
    assert_eq!(by_name(&rt, "uuid"), offset_of!(RadarTrackShm, uuid));
    assert_eq!(
        by_name(&rt, "position"),
        offset_of!(RadarTrackShm, position)
    );
    assert_eq!(by_name(&rt, "size"), offset_of!(RadarTrackShm, size));
    assert_eq!(
        by_name(&rt, "classification"),
        offset_of!(RadarTrackShm, classification)
    );
    assert_eq!(
        by_name(&rt, "position_covariance"),
        offset_of!(RadarTrackShm, position_covariance)
    );
    assert_eq!(
        by_name(&rt, "size_covariance"),
        offset_of!(RadarTrackShm, size_covariance)
    );
}

/// `FixedArray<Nested-fixed>` stride vs a hand-written `#[repr(C)]`
/// oracle — no vendored .msg exercises arrays of message types, so this
/// pins the path against rustc directly.
#[test]
fn fixed_array_of_nested_matches_repr_c_oracle() {
    use cerulion_core::codegen::{FieldDef, FieldType, MessageSchema};
    use native_ros2_messages::geometry_msgs::PointShm;

    #[repr(C)]
    struct Oracle {
        corners: [PointShm; 3],
        tag: u8,
    }

    let mut point = MessageSchema::new_in_package("Point", "geometry_msgs");
    for f in ["x", "y", "z"] {
        point.add_field(FieldDef::new(f, FieldType::F64));
    }
    let mut corners = MessageSchema::new_in_package("Corners", "test_msgs");
    corners.add_field(FieldDef::new(
        "corners",
        FieldType::FixedArray {
            element_type: Box::new(FieldType::Nested {
                schema_name: "Point".into(),
                package: Some("geometry_msgs".into()),
                fixed: None,
            }),
            length: 3,
        },
    ));
    corners.add_field(FieldDef::new("tag", FieldType::U8));

    let (mut resolver, warnings) = LayoutResolver::new(vec![point, corners]);
    assert!(warnings.is_empty(), "{warnings:?}");
    let layout = resolver.layout_of("test_msgs/Corners").expect("layout");

    assert_eq!(layout.fixed_size, std::mem::size_of::<Oracle>());
    assert_eq!(layout.fixed_fields[0].offset, offset_of!(Oracle, corners));
    assert_eq!(
        layout.fixed_fields[0].size,
        std::mem::size_of::<[PointShm; 3]>()
    );
    assert_eq!(layout.fixed_fields[1].offset, offset_of!(Oracle, tag));
}
