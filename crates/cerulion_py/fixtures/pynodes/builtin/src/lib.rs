// SPDX-License-Identifier: AGPL-3.0-only
static INFO_BYTES: &[u8] = b"{\"inputs\":[],\"outputs\":[{\"name\":\"out\",\"schema_hash\":15293913555552287199,\"max_slice_len_default\":56,\"promise_within_ms\":null,\"wire_fixed_size\":24}],\"policy\":{\"period_ms\":1}}\0";
// CERULION:PORT_SCHEMAS {"outputs":{"out":"geometry_msgs/Vector3"}}

cerulion_pynode::export_node! {
    module: "node",
    sys_path: [env!("CARGO_MANIFEST_DIR")],
    info: INFO_BYTES
}
