// SPDX-License-Identifier: AGPL-3.0-only
static INFO_BYTES: &[u8] = b"{\"inputs\":[{\"name\":\"inp\",\"schema_hash\":5093796653891464803}],\"outputs\":[{\"name\":\"out\",\"schema_hash\":5093796653891464803,\"max_slice_len_default\":36,\"promise_within_ms\":null,\"wire_fixed_size\":4},{\"name\":\"out2\",\"schema_hash\":5093796653891464803,\"max_slice_len_default\":36,\"promise_within_ms\":null,\"wire_fixed_size\":4}],\"policy\":{\"period_ms\":10}}\0";
// CERULION:PORT_SCHEMAS {"inputs":{"inp":"Probe"},"outputs":{"out":"Probe","out2":"Probe"}}

cerulion_pynode::export_node! {
    module: "node",
    sys_path: [env!("CARGO_MANIFEST_DIR")],
    info: INFO_BYTES
}
