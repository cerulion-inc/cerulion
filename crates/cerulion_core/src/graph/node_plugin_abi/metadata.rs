// SPDX-License-Identifier: AGPL-3.0-only
//! Stable-only wire facts layered on the common legacy metadata emitter.

use std::ffi::{CStr, CString};

use crate::error::TransportResult;
use crate::node_abi::LoanRequestV1;

/// Preserve common params, triggers and QoS with wire facts and declared inline safety.
// hot-path-alloc-ok-fn: bootstrap builds immutable stable metadata once per library.
pub fn metadata(
    base: &CStr,
    inputs: &[LoanRequestV1],
    outputs: &[LoanRequestV1],
    accumulate_caps: &[Option<usize>],
    inline_safe: bool,
) -> TransportResult<CString> {
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
        for (port, facts) in ports.iter_mut().zip(facts) {
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
        }
    }
    let input_ports = value["inputs"]
        .as_array_mut()
        .ok_or_else(|| super::argument_error("input metadata array is missing"))?;
    if input_ports.len() != accumulate_caps.len() {
        return Err(super::argument_error(
            "accumulate capacities differ from input count",
        ));
    }
    for (port, capacity) in input_ports.iter_mut().zip(accumulate_caps) {
        port.as_object_mut()
            .ok_or_else(|| super::argument_error("input metadata is not an object"))?
            .insert("accumulate_cap".into(), (*capacity).into());
    }
    value
        .as_object_mut()
        .ok_or_else(|| super::argument_error("common metadata is not an object"))?
        .insert("inline_safe".into(), inline_safe.into());
    let bytes =
        serde_json::to_vec(&value).map_err(|error| super::argument_error(&error.to_string()))?;
    CString::new(bytes).map_err(|error| super::argument_error(&error.to_string()))
}

/// Add explicit raw author schema facts to the source metadata name arrays.
/// Input/output order and policy are preserved; no Rust owner crosses C.
// hot-path-alloc-ok-fn: raw bootstrap normalizes metadata only once per library
pub fn raw_metadata(
    base: &CStr,
    inputs: &[LoanRequestV1],
    outputs: &[LoanRequestV1],
) -> TransportResult<CString> {
    let mut value: serde_json::Value = serde_json::from_slice(base.to_bytes())
        .map_err(|error| super::argument_error(&error.to_string()))?;
    for (direction, facts) in [("inputs", inputs), ("outputs", outputs)] {
        let ports = value
            .get_mut(direction)
            .and_then(serde_json::Value::as_array_mut)
            .ok_or_else(|| super::argument_error("raw metadata port array is missing"))?;
        if ports.len() != facts.len() {
            return Err(super::argument_error(
                "raw wire facts differ from declared port count",
            ));
        }
        for (port, fact) in ports.iter_mut().zip(facts) {
            if let Some(name) = port.as_str() {
                *port = serde_json::json!({ "name": name });
            }
            port.as_object_mut()
                .ok_or_else(|| super::argument_error("raw port metadata is not a name or object"))?
                .insert("schema_hash".into(), fact.schema_hash.into());
        }
    }
    let bytes =
        serde_json::to_vec(&value).map_err(|error| super::argument_error(&error.to_string()))?;
    let normalized =
        CString::new(bytes).map_err(|error| super::argument_error(&error.to_string()))?;
    metadata(
        &normalized,
        inputs,
        outputs,
        &vec![None; inputs.len()],
        false,
    )
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
            let enriched = metadata(base, &[facts(7)], &[facts(9)], &[None], false).unwrap();
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
    fn inline_safe_metadata_is_the_declared_bool_and_preserves_common_bytes() {
        let base = c"{\"inputs\":[],\"outputs\":[],\"policy\":{\"period_ms\":1}}";
        for inline in [false, true] {
            let bytes = metadata(base, &[], &[], &[], inline).unwrap();
            let value: serde_json::Value = serde_json::from_slice(bytes.to_bytes()).unwrap();
            assert_eq!(value["inline_safe"], serde_json::Value::Bool(inline));
            assert_eq!(value["policy"]["period_ms"], 1);
        }
        assert!(!base
            .to_bytes()
            .windows(11)
            .any(|bytes| bytes == b"inline_safe"));
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
            assert!(metadata(base, &[facts(7)], &[], &[None], false).is_err());
        }
    }
}
