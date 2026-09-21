// SPDX-License-Identifier: AGPL-3.0-only
//! Tests that error messages contain actionable context and suggested fixes.

use cerulion_core::error::TransportError;

#[test]
fn test_node_creation_error_contains_context_and_suggestion() {
    let err = TransportError::NodeCreation {
        node_name: "camera_node".to_string(),
        reason: "permission denied".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("camera_node"), "should contain node name");
    assert!(msg.contains("permission denied"), "should contain reason");
    assert!(
        msg.contains("shared memory permissions"),
        "should suggest checking permissions"
    );
}

#[test]
fn test_publisher_creation_error_contains_topic_and_suggestion() {
    let err = TransportError::PublisherCreation {
        topic: "camera/image".to_string(),
        reason: "service already exists".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("camera/image"), "should contain topic name");
    assert!(msg.contains("graph YAML"), "should suggest checking YAML");
}

#[test]
fn test_subscriber_creation_error_contains_topic_and_suggestion() {
    let err = TransportError::SubscriberCreation {
        topic: "imu/data".to_string(),
        reason: "service not found".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("imu/data"), "should contain topic name");
    assert!(
        msg.contains("publisher for this topic"),
        "should suggest checking publisher"
    );
}

#[test]
fn test_loan_error_contains_topic_and_buffer_suggestion() {
    let err = TransportError::Loan {
        topic: "lidar/points".to_string(),
        reason: "out of memory".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("lidar/points"), "should contain topic name");
    assert!(
        msg.contains("max_slice_len"),
        "should suggest increasing buffer"
    );
}

#[test]
fn test_publish_error_contains_topic() {
    let err = TransportError::Publish {
        topic: "camera/image".to_string(),
        reason: "send failed".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("camera/image"), "should contain topic name");
    assert!(
        msg.contains("subscribers are draining"),
        "should suggest checking subscribers"
    );
}

#[test]
fn test_receive_error_contains_topic() {
    let err = TransportError::Receive {
        topic: "camera/image".to_string(),
        reason: "timeout".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("camera/image"), "should contain topic name");
    assert!(
        msg.contains("publisher exists"),
        "should suggest checking publisher"
    );
}

#[test]
fn test_topic_not_found_contains_suggestion() {
    let err = TransportError::TopicNotFound {
        topic: "sensor/temp".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("sensor/temp"), "should contain topic name");
    assert!(msg.contains("graph YAML"), "should suggest checking YAML");
}

#[test]
fn test_not_initialized_contains_suggestion() {
    let err = TransportError::NotInitialized;
    let msg = err.to_string();
    assert!(
        msg.contains("TransportManager::init()"),
        "should suggest calling init"
    );
}

#[test]
fn test_duplicate_node_contains_id_and_suggestion() {
    let err = TransportError::DuplicateNode {
        node_id: "camera_1".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("camera_1"), "should contain node id");
    assert!(
        msg.contains("unique ID"),
        "should explain uniqueness requirement"
    );
}

#[test]
fn test_node_not_found_contains_id_and_suggestion() {
    let err = TransportError::NodeNotFound {
        node_id: "detector_1".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("detector_1"), "should contain node id");
    assert!(msg.contains("graph YAML"), "should suggest checking YAML");
}

#[test]
fn test_graph_parse_error_contains_suggestion() {
    let err = TransportError::GraphParseError {
        reason: "expected ':' at line 5".to_string(),
    };
    let msg = err.to_string();
    assert!(
        msg.contains("expected ':' at line 5"),
        "should contain parse reason"
    );
    assert!(
        msg.contains("YAML syntax"),
        "should suggest checking YAML syntax"
    );
}

#[test]
fn test_session_creation_error_contains_suggestion() {
    let err = TransportError::SessionCreation {
        reason: "connection refused".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("connection refused"), "should contain reason");
    assert!(
        msg.contains("network connectivity"),
        "should suggest checking network"
    );
}

#[test]
fn test_schema_hash_mismatch_contains_both_hashes_and_topic() {
    let err = TransportError::SchemaHashMismatch {
        topic: "camera/image".to_string(),
        expected_schema: "sensor_msgs::Image".to_string(),
        expected_hash: 0xABCD_1234_5678_9012,
        actual_schema: "std_msgs::Header".to_string(),
        actual_hash: 0x1234_5678_9ABC_DEF0,
    };
    let msg = err.to_string();
    assert!(msg.contains("camera/image"), "should contain topic name");
    assert!(
        msg.contains("sensor_msgs::Image"),
        "should contain expected schema"
    );
    assert!(
        msg.contains("ABCD123456789012"),
        "should contain expected hash in hex"
    );
    assert!(
        msg.contains("std_msgs::Header"),
        "should contain actual schema"
    );
    assert!(msg.contains("12345678"), "should contain actual hash");
    assert!(msg.contains("graph YAML"), "should suggest checking YAML");
}

#[test]
fn test_schema_hash_mismatch_display_format() {
    let err = TransportError::SchemaHashMismatch {
        topic: "test/topic".to_string(),
        expected_schema: "SchemaA".to_string(),
        expected_hash: 0xFF,
        actual_schema: "SchemaB".to_string(),
        actual_hash: 0xAA,
    };
    let msg = err.to_string();
    // Verify hex formatting with zero-padding
    assert!(
        msg.contains("00000000000000FF"),
        "expected hash should be zero-padded to 16 hex digits, got: {}",
        msg
    );
    assert!(
        msg.contains("00000000000000AA"),
        "actual hash should be zero-padded to 16 hex digits, got: {}",
        msg
    );
}

#[test]
fn test_buffer_too_small_contains_both_sizes_and_topic() {
    let err = TransportError::BufferTooSmall {
        topic: "lidar/pointcloud".to_string(),
        needed: 1_048_576,
        available: 65_536,
    };
    let msg = err.to_string();
    assert!(
        msg.contains("lidar/pointcloud"),
        "should contain topic name"
    );
    assert!(msg.contains("1048576"), "should contain needed size");
    assert!(msg.contains("65536"), "should contain available size");
    assert!(
        msg.contains("max_slice_len"),
        "should suggest increasing buffer"
    );
}

#[test]
fn test_buffer_too_small_edge_case_zero_available() {
    let err = TransportError::BufferTooSmall {
        topic: "test/topic".to_string(),
        needed: 32,
        available: 0,
    };
    let msg = err.to_string();
    assert!(msg.contains("32"), "should contain needed size");
    assert!(msg.contains("0 bytes"), "should contain zero available");
}

// ============================================================
// Error variants for the SHM-backed proxy API.
//
// These cover the `OutputProxy<T>` / `InputView<T>` error paths added with
// the SHM-backed API. The spec called the variant `ProxyBufferTooSmall`
// and the actual variant was later brought in line; see
// `cerulion_core/src/error.rs`.
// ============================================================

#[test]
fn test_buffer_too_small_proxy_contains_both_sizes_and_suggestion() {
    let err = TransportError::ProxyBufferTooSmall {
        requested: 1024,
        available: 512,
    };
    let msg = err.to_string();
    assert!(msg.contains("1024"), "should contain requested size");
    assert!(msg.contains("512"), "should contain available size");
    assert!(
        msg.contains("max_slice_len"),
        "should suggest increasing max_slice_len"
    );
}

#[test]
fn test_buffer_too_small_proxy_zero_available() {
    // Edge case: cursor already at the end of the loaned buffer.
    let err = TransportError::ProxyBufferTooSmall {
        requested: 8,
        available: 0,
    };
    let msg = err.to_string();
    assert!(msg.contains("8"), "should contain requested size");
    assert!(msg.contains("0"), "should contain zero available");
}

#[test]
fn test_missing_variable_field_contains_field_name() {
    let err = TransportError::MissingVariableField { field: "data" };
    let msg = err.to_string();
    assert!(msg.contains("data"), "should contain the field name");
    assert!(
        msg.contains("set_") || msg.contains("loan_") || msg.contains("push_"),
        "should suggest one of the setter prefixes; got: {msg}"
    );
}

#[test]
fn test_missing_variable_field_with_static_field_name() {
    // The variant's `field` is `&'static str` — verify the formatter
    // accepts a static string slice and renders it verbatim.
    static FIELD: &str = "encoding";
    let err = TransportError::MissingVariableField { field: FIELD };
    let msg = err.to_string();
    assert!(msg.contains("encoding"), "should contain 'encoding'");
}

#[test]
fn test_loan_capacity_contains_topic_and_suggestion() {
    let err = TransportError::LoanCapacity {
        topic: "camera/image".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("camera/image"), "should contain topic name");
    // `LoanCapacity` has TWO producers, and the caller cannot tell them
    // apart from the error alone, so the Display must name both remedies.
    // Assert each independently — a future Display refactor that drops one
    // silently leaves half the callers with no actionable text.
    // (Formerly the second producer was the in-process heap backend's
    // OOM, remediated by max_slice_len / process memory. That backend is
    // deleted; the live second producer is the publisher's simultaneous-loan
    // budget.)
    assert!(
        msg.contains("subscribers") || msg.contains("slot count"),
        "should suggest sample-pool remediation (subscribers / slot count); got: {msg}"
    );
    assert!(
        msg.contains("publisher_max_loaned_samples") || msg.contains("held loan"),
        "should suggest loan-budget remediation (publisher_max_loaned_samples / \
         held loan); got: {msg}"
    );
}

#[test]
fn test_push_after_non_tail_contains_field_name_and_remediation() {
    // This user error was once reported as `TransportError::Internal`;
    // the audit replaced it with a dedicated `PushAfterNonTail` variant so the
    // user can pattern-match on the error and the message names the field.
    let err = TransportError::PushAfterNonTail { field: "ranges" };
    let msg = err.to_string();
    assert!(msg.contains("ranges"), "should contain the field name");
    assert!(
        msg.contains("set_ranges") || msg.contains("loan_ranges"),
        "should suggest the typed alternatives; got: {msg}"
    );
}

#[test]
fn test_max_slice_len_required_contains_topic_and_suggestion() {
    let err = TransportError::MaxSliceLenRequired {
        topic: "lidar/pointcloud".to_string(),
    };
    let msg = err.to_string();
    assert!(
        msg.contains("lidar/pointcloud"),
        "should contain topic name"
    );
    assert!(
        msg.contains("max_slice_len"),
        "should mention the missing config knob"
    );
    assert!(
        msg.contains("graph YAML") || msg.contains("create_publisher"),
        "should point to where to set it; got: {msg}"
    );
}

#[test]
fn test_schema_mismatch_contains_both_hashes() {
    let err = TransportError::SchemaMismatch {
        topic: "control/cmd_vel".to_string(),
        expected_hash: 0x0000_0000_0000_0ABC,
        actual_hash: 0x0000_0000_0000_0DEF,
    };
    let msg = err.to_string();
    assert!(msg.contains("control/cmd_vel"), "should contain topic");
    // Hex-formatted with zero padding to 16 digits.
    assert!(
        msg.contains("0000000000000ABC"),
        "should contain expected hash, got: {msg}"
    );
    assert!(
        msg.contains("0000000000000DEF"),
        "should contain actual hash, got: {msg}"
    );
}

#[test]
fn test_schema_mismatch_suggests_rebuild() {
    // The layout-sensitive hash means the usual cause of
    // a mismatch is rebuilding only one side after a schema edit. Both
    // schema-mismatch variants must carry the actionable rebuild hint
    // (mirrors the CLI's `cerulion node build <type>` stale-cdylib advisory).
    let hash_only = TransportError::SchemaMismatch {
        topic: "control/cmd_vel".to_string(),
        expected_hash: 0xABC,
        actual_hash: 0xDEF,
    }
    .to_string();
    assert!(
        hash_only.contains("cerulion node build"),
        "hash-only SchemaMismatch should suggest a rebuild; got: {hash_only}"
    );
    assert!(
        hash_only.contains("BOTH"),
        "rebuild hint must stress rebuilding BOTH sides; got: {hash_only}"
    );

    let with_names = TransportError::SchemaHashMismatch {
        topic: "control/cmd_vel".to_string(),
        expected_schema: "geometry_msgs/Twist".to_string(),
        expected_hash: 0xABC,
        actual_schema: "geometry_msgs/Twist".to_string(),
        actual_hash: 0xDEF,
    }
    .to_string();
    assert!(
        with_names.contains("cerulion node build"),
        "named SchemaHashMismatch should suggest a rebuild; got: {with_names}"
    );
}

#[test]
fn test_schema_mismatch_display_format_is_uppercase_hex() {
    let err = TransportError::SchemaMismatch {
        topic: "t".to_string(),
        expected_hash: 0xff,
        actual_hash: 0xaa,
    };
    let msg = err.to_string();
    // `:016X` formatting → uppercase, 16-digit zero-padded.
    assert!(
        msg.contains("00000000000000FF"),
        "expected hash should be uppercase 16-digit hex, got: {msg}"
    );
    assert!(
        msg.contains("00000000000000AA"),
        "actual hash should be uppercase 16-digit hex, got: {msg}"
    );
}

#[test]
fn test_internal_error_suggests_bug_report() {
    let err = TransportError::Internal {
        reason: "mutex poisoned".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("mutex poisoned"), "should contain reason");
    assert!(
        msg.contains("bug in Cerulion"),
        "should indicate this is a bug"
    );
    assert!(msg.contains("report"), "should suggest reporting");
}

#[test]
fn test_node_error_contains_node_id() {
    let err = TransportError::NodeError {
        node_id: "fusion_node".to_string(),
        reason: "tick panicked".to_string(),
    };
    let msg = err.to_string();
    assert!(msg.contains("fusion_node"), "should contain node id");
    assert!(msg.contains("tick panicked"), "should contain reason");
}

#[test]
fn test_scheduler_error_contains_reason() {
    let err = TransportError::SchedulerError {
        reason: "sync source 'camera' not found".to_string(),
    };
    let msg = err.to_string();
    assert!(
        msg.contains("sync source 'camera' not found"),
        "should contain full reason"
    );
}

#[test]
fn test_graph_error_contains_reason() {
    let err = TransportError::GraphError {
        reason: "node 'detector' references unknown topic 'camera/image'".to_string(),
    };
    let msg = err.to_string();
    assert!(
        msg.contains("node 'detector' references unknown topic 'camera/image'"),
        "should contain full reason"
    );
}
