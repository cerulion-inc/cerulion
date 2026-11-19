// SPDX-License-Identifier: AGPL-3.0-only
//! Prints the linked node's policy and outputs, read from its macro entry.

fn main() {
    let info = test_node_links_host_app::linked_node()
        .info()
        .expect("node info");
    println!(
        "policy={:?} outputs={:?}",
        info.policy(),
        info.output_names()
    );
}
