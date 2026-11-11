// SPDX-License-Identifier: AGPL-3.0-only
//! Stable-only wire facts layered on the common legacy metadata emitter.

use std::ffi::{CStr, CString};

use crate::error::TransportResult;
use crate::node_abi::LoanRequestV1;

/// Preserve common params, triggers and QoS while adding checked wire facts.
// hot-path-alloc-ok-fn: bootstrap builds immutable stable metadata once per library.
pub fn metadata(
    base: &CStr,
    inputs: &[LoanRequestV1],
    outputs: &[LoanRequestV1],
) -> TransportResult<CString> {
    metadata_with_accumulation(base, inputs, outputs, &[])
}

/// Add explicit accumulation limits without changing common legacy metadata.
// hot-path-alloc-ok-fn: checked cold bootstrap metadata, once per library.
pub fn metadata_with_accumulation(
    base: &CStr,
    inputs: &[LoanRequestV1],
    outputs: &[LoanRequestV1],
    caps: &[Option<u32>],
) -> TransportResult<CString> {
    if !caps.is_empty() && caps.len() != inputs.len() {
        return Err(super::argument_error(
            "accumulation limits differ from input count",
        ));
    }
    let mut value: serde_json::Value = serde_json::from_slice(base.to_bytes())
        .map_err(|error| super::argument_error(&error.to_string()))?;
    for (direction, facts) in [("inputs", inputs), ("outputs", outputs)] {
        let ports = value
            .get_mut(direction)
            .and_then(serde_json::Value::as_array_mut)
            .ok_or_else(|| super::argument_error("common metadata port array is missing"))?;
        if ports.len() != facts.len() {
            return Err(super::argument_error(
                "wire facts differ from the common port count",
            ));
        }
        for (index, (port, facts)) in ports.iter_mut().zip(facts).enumerate() {
            let port = port
                .as_object_mut()
                .ok_or_else(|| super::argument_error("common port metadata is not an object"))?;
            if port.get("schema_hash").and_then(serde_json::Value::as_u64)
                != Some(facts.schema_hash)
            {
                return Err(super::argument_error(
                    "wire facts differ from the common schema identity",
                ));
            }
            port.insert("wire_fixed_size".into(), facts.wire_fixed_size.into());
            port.insert("variable_count".into(), facts.variable_count.into());
            port.insert("alignment".into(), facts.alignment.into());
            if direction == "inputs" {
                if let Some(Some(cap)) = caps.get(index) {
                    if *cap == 0 {
                        return Err(super::argument_error("accumulation limit is zero"));
                    }
                    port.insert("accumulate_cap".into(), (*cap).into());
                }
            }
        }
    }
    let bytes =
        serde_json::to_vec(&value).map_err(|error| super::argument_error(&error.to_string()))?;
    CString::new(bytes).map_err(|error| super::argument_error(&error.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn facts(schema_hash: u64) -> LoanRequestV1 {
        LoanRequestV1 {
            schema_hash,
            minimum_capacity: 64,
            wire_fixed_size: 24,
            variable_count: 0,
            alignment: 8,
        }
    }

    #[test]
    fn stable_facts_preserve_common_metadata_and_leave_the_legacy_bytes_untouched() {
        let base = c"{\"inputs\":[{\"name\":\"input\",\"schema_hash\":7,\"trigger\":true}],\"outputs\":[{\"name\":\"output\",\"schema_hash\":9}],\"params\":[{\"name\":\"scale\",\"default\":2}],\"policy\":{\"kind\":\"data_trigger\",\"input\":\"input\"}}";
        for _ in 0..2 {
            let enriched = metadata(base, &[facts(7)], &[facts(9)]).unwrap();
            let value: serde_json::Value = serde_json::from_slice(enriched.to_bytes()).unwrap();
            assert_eq!(value["inputs"][0]["trigger"], true);
            assert_eq!(value["inputs"][0]["wire_fixed_size"], 24);
            assert_eq!(value["outputs"][0]["alignment"], 8);
            assert_eq!(value["outputs"][0]["variable_count"], 0);
            assert_eq!(value["params"][0]["default"], 2);
            assert_eq!(value["policy"]["input"], "input");
        }
        assert!(!base
            .to_bytes()
            .windows(9)
            .any(|bytes| bytes == b"alignment"));
    }

    #[test]
    fn malformed_shape_count_and_schema_are_refused() {
        for base in [
            c"x",
            c"{}",
            c"{\"inputs\":[],\"outputs\":[]}",
            c"{\"inputs\":[3],\"outputs\":[]}",
            c"{\"inputs\":[{\"schema_hash\":8}],\"outputs\":[]}",
        ] {
            assert!(metadata(base, &[facts(7)], &[]).is_err());
        }
    }
}
