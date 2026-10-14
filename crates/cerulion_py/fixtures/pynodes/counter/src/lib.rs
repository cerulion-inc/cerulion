// SPDX-License-Identifier: AGPL-3.0-only
static INFO_BYTES: &[u8] = b"{\"inputs\":[{\"name\":\"inp\",\"schema_hash\":15583965953746584896,\"depth\":1}],\"outputs\":[{\"name\":\"out\",\"schema_hash\":15583965953746584896,\"max_slice_len_default\":40,\"promise_within_ms\":null,\"wire_fixed_size\":8}],\"policy\":{\"period_ms\":10}}\0";
// CERULION:PORT_SCHEMAS {"inputs":{"inp":"Probe"},"outputs":{"out":"Probe"}}

cerulion_pynode::export_node! {
    module: "node",
    sys_path: [env!("CARGO_MANIFEST_DIR")],
    info: INFO_BYTES
}
