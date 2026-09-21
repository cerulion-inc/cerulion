// SPDX-License-Identifier: AGPL-3.0-only
//! Tests for proxy-buffer sizing errors and YAML `max_slice_len` parsing
//! (rewritten for the SHM-backed `loan_proxy` API).
//!
//! Verifies:
//! - Omitting `max_slice_len` from YAML parses as `None` (graph runtime
//!   then falls back to `DEFAULT_MAX_SLICE_LEN`).
//! - `loan_<field>` / `set_<field>` / `push_<field>` setters on a variable
//!   schema return `Err(ProxyBufferTooSmall)` synchronously when the request
//!   would exceed the loaned `max_slice_len` buffer.
//! - The proxy is not poisoned by a single failed setter — fields that fit
//!   can still be written after a setter that returned `ProxyBufferTooSmall`.
//! - A variable-schema publisher created with a `max_slice_len` smaller than
//!   `WireHeader + WIRE_FIXED_SIZE + 8 * VARIABLE_FIELD_COUNT` returns
//!   `Err(MaxSliceLenRequired)` from `loan_proxy::<T>()`.

use cerulion_core::wire::MaxSliceLen;

use cerulion_core::error::TransportError;
use cerulion_core::graph::config::{OutputDef, DEFAULT_MAX_SLICE_LEN};
use cerulion_core::graph::parse_graph;
use cerulion_core::testing::TestTransport;
use native_ros2_messages::std_msgs::String as RosString;

// ============================================================
// YAML parsing: omitting max_slice_len defaults to None
// ============================================================

#[test]
fn test_omitting_max_slice_len_parses_as_none() {
    let yaml = r#"
name: test
nodes:
  - id: pub1
    type: test_pub
    outputs:
      - name: data
        schema: test/Data
"#;
    let config = parse_graph(yaml).unwrap();
    assert_eq!(config.nodes[0].outputs[0].max_slice_len, None);
}

#[test]
fn test_explicit_max_slice_len_parses_as_some() {
    let yaml = r#"
name: test
nodes:
  - id: pub1
    type: test_pub
    outputs:
      - name: data
        schema: test/Data
        max_slice_len: 2048
"#;
    let config = parse_graph(yaml).unwrap();
    assert_eq!(config.nodes[0].outputs[0].max_slice_len, Some(2048));
}

#[test]
fn test_default_max_slice_len_constant() {
    // Tier-3 of the 3-tier max_slice_len resolution
    // ladder is 128 MiB (launch bump, from 16 MiB — free because
    // iceoryx2 Static pools are lazy/demand-paged). Tier-1 (explicit YAML)
    // wins; tier-2 (`<T as ShmMessage>::MAX_SLICE_LEN`) is consulted second;
    // this constant is the final fallback when neither is available.
    assert_eq!(DEFAULT_MAX_SLICE_LEN, 128 * 1024 * 1024);
}

#[test]
fn test_output_def_default_has_none_max_slice_len() {
    let yaml = r#"
name: data
schema: test/Data
"#;
    let def: OutputDef = serde_yaml::from_str(yaml).unwrap();
    assert_eq!(def.max_slice_len, None);
}

// ============================================================
// MaxSliceLenRequired: variable-schema publisher with too-small buffer.
//
// `String` (std_msgs::String) is a variable schema with one variable field
// (`data`). The minimum loanable buffer is:
//   WireHeader (32) + WIRE_FIXED_SIZE (0) + 8 * VARIABLE_FIELD_COUNT (8) = 40.
// Anything smaller must reject `loan_proxy::<RosString>()` with
// MaxSliceLenRequired. (For fixed schemas the equivalent error is
// BufferTooSmall — covered by the existing output_proxy_test.)
// ============================================================

#[test]
fn test_loan_proxy_undersized_variable_schema_errors_max_slice_len_required() {
    // 32 = WireHeader::SIZE (smallest legal MaxSliceLen, header-only slot).
    // std_msgs::String needs header (32) + offset table for 1 variable field (8) = 40 bytes
    // before any payload bytes — so 32 < 40 → loan_proxy must return MaxSliceLenRequired.
    // The newtype rejects anything < WireHeader::SIZE at construction (compile-time),
    // so this test now pins the *runtime* lower bound (schema-specific) rather than the
    // type-system lower bound.
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher("test/undersized_var", MaxSliceLen::const_new(32), 0);
    let result = publisher.loan_proxy::<RosString>();
    match result {
        Err(TransportError::MaxSliceLenRequired { topic }) => {
            assert_eq!(topic, "test/undersized_var");
        }
        Err(other) => panic!("expected MaxSliceLenRequired, got: {:?}", other),
        Ok(_) => panic!("expected MaxSliceLenRequired, got Ok"),
    }
}

// ============================================================
// ProxyBufferTooSmall: setter requested more bytes than the loaned slot.
//
// The proxy holds a max_slice_len-sized SHM region. Once the WireHeader
// (32 B) and the offset table (8 B per variable field) are subtracted the
// remainder is what `set_data`/`loan_data`/`push_data` can fill. Going
// past that returns ProxyBufferTooSmall synchronously — no allocation, no
// publish.
// ============================================================

// Overflow-spill update: setters now spill to a heap fallback on recoverable
// overflow (cursor + bytes > self.len but <= max_capacity). The
// `ProxyBufferTooSmall` variant is no longer produced by the codegen-
// emitted setters — `PayloadTooLarge` replaces it for the no-rescue case
// (cursor + bytes > max_capacity). The three tests below were updated to
// pin the new contract.

#[test]
fn test_set_data_oversized_returns_payload_too_large() {
    // 64 bytes total: 32 (header) + 8 (offset table) → 24 bytes max payload.
    // max_capacity = 64 - 32 = 32 (post-header ceiling). A 1 KiB payload
    // exceeds even max_capacity → PayloadTooLarge (no rescue possible).
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher("test/oversize", MaxSliceLen::const_new(64), 0);
    let mut proxy = publisher.loan_proxy::<RosString>().expect("loan_proxy");

    let big_payload: String = "x".repeat(1024);
    let result = proxy.set_data(&big_payload);
    match result {
        Err(TransportError::PayloadTooLarge {
            topic,
            requested,
            max,
        }) => {
            assert_eq!(topic, "test/oversize");
            assert!(requested >= 1024, "requested includes the 1 KiB write");
            assert_eq!(
                max,
                64 - 32,
                "max_capacity = max_slice_len - WireHeader::SIZE"
            );
        }
        Err(other) => panic!("expected PayloadTooLarge, got: {:?}", other),
        Ok(()) => panic!("expected PayloadTooLarge, got Ok"),
    }
}

#[test]
fn test_loan_data_oversized_returns_payload_too_large() {
    let tt = TestTransport::with_buffer_size(16);
    let mut publisher = tt.publisher("test/loan_oversize", MaxSliceLen::const_new(64), 0);
    let mut proxy = publisher.loan_proxy::<RosString>().expect("loan_proxy");

    // String's `data` is a `Bytes` field → emits `loan_data(n) -> &mut [u8]`.
    // 2048 > max_capacity (32) → PayloadTooLarge.
    let result = proxy.loan_data(2048);
    match result {
        Err(TransportError::PayloadTooLarge {
            topic,
            requested,
            max,
        }) => {
            assert_eq!(topic, "test/loan_oversize");
            assert!(requested >= 2048);
            assert_eq!(max, 64 - 32);
        }
        Err(other) => panic!("expected PayloadTooLarge, got: {:?}", other),
        Ok(_) => panic!("expected PayloadTooLarge, got Ok"),
    }
}

#[test]
fn test_proxy_not_poisoned_by_failed_setter() {
    // After a setter returns PayloadTooLarge the proxy must still accept
    // a smaller value — failed setters do not move the cursor or mark the
    // field written, so a subsequent in-bounds call succeeds. The overflow
    // spill path is NOT triggered when PayloadTooLarge fires (the
    // ceiling itself is too small), so the proxy stays on the original
    // SHM loan.
    let tt = TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher("test/recover", MaxSliceLen::const_new(128), 0);
    let mut subscriber = tt.subscriber("test/recover");
    {
        let mut proxy = publisher.loan_proxy::<RosString>().expect("loan_proxy");

        // First: oversized (4 KiB > max_capacity=96) → fails with PayloadTooLarge.
        let huge = "y".repeat(4096);
        assert!(matches!(
            proxy.set_data(&huge),
            Err(TransportError::PayloadTooLarge { .. })
        ));

        // Second: small payload that fits → succeeds.
        proxy
            .set_data("ok")
            .expect("small payload should fit after failed setter");
        // proxy drops here, publishes the small payload.
    }

    let observed = subscriber
        .try_view::<RosString, _>(|view| view.data().expect("utf-8").to_owned())
        .expect("try_view");
    assert_eq!(
        observed.as_deref(),
        Some("ok"),
        "publisher should have emitted the second (in-bounds) payload"
    );
}

#[test]
fn test_set_data_exact_fit_succeeds() {
    // Sanity check the available-bytes accounting: ask for exactly the
    // remainder after header + offset table and confirm publish succeeds.
    // 32 (header) + 8 (offset table) + 16 (payload room) = 56-byte buffer.
    let buffer_size: u32 = 56;
    let tt = TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher("test/exact_fit", MaxSliceLen::const_new(buffer_size), 0);
    let mut subscriber = tt.subscriber("test/exact_fit");
    {
        let mut proxy = publisher.loan_proxy::<RosString>().expect("loan_proxy");
        let payload = "x".repeat(16);
        proxy
            .set_data(&payload)
            .expect("payload exactly fits the loan");
    }

    let observed = subscriber
        .try_view::<RosString, _>(|view| view.data().expect("utf-8").len())
        .expect("try_view");
    assert_eq!(observed, Some(16));
}

/// Pin that the publisher constructor accepts the type-level
/// ceiling `u32::MAX` and exposes it via the `max_slice_len()`
/// accessor unchanged. We do NOT call `loan_proxy` here — a u32::MAX
/// loan would allocate 4 GiB on the heap and is unrelated to the
/// floor-comparison invariant under test. The point is to prove the
/// `u32` API at the boundary supports the wire-format ceiling; the
/// allocation path is exercised by other tests at sane sizes.
#[test]
fn test_publisher_accepts_u32_max_at_ceiling() {
    let tt = TestTransport::with_buffer_size(16);
    let publisher = tt.publisher("test/u32_max_ceiling", MaxSliceLen::const_new(u32::MAX), 0);
    assert_eq!(
        publisher.max_slice_len().get(),
        u32::MAX,
        "max_slice_len accessor must round-trip u32::MAX unchanged",
    );
}
