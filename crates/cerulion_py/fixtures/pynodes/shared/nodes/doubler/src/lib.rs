// SPDX-License-Identifier: AGPL-3.0-only
static INFO_BYTES: &[u8] = b"{\"inputs\":[{\"name\":\"inp\",\"schema_hash\":9549963880961276720}],\"outputs\":[{\"name\":\"out\",\"schema_hash\":9549963880961276720,\"max_slice_len_default\":44,\"promise_within_ms\":null,\"wire_fixed_size\":4}],\"policy\":{\"period_ms\":10}}\0";
// CERULION:PORT_SCHEMAS {"inputs":{"inp":"UInt32Probe"},"outputs":{"out":"UInt32Probe"}}

cerulion_pynode::export_node! {
    module: "node",
    sys_path: [env!("CARGO_MANIFEST_DIR")],
    info: INFO_BYTES
}
