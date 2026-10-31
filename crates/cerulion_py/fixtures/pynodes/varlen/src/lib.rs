// SPDX-License-Identifier: AGPL-3.0-only
static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[{\"name\":\"out\",\"schema_hash\":3245531966109359144,\"max_slice_len_default\":96,\"promise_within_ms\":null,\"wire_fixed_size\":0}],\"policy\":{\"period_ms\":10}}\0";
// CERULION:PORT_SCHEMAS {"outputs":{"out":"Samples"}}

cerulion_pynode::export_node! {
    module: "node",
    sys_path: [env!("CARGO_MANIFEST_DIR")],
    info: INFO_BYTES
}
