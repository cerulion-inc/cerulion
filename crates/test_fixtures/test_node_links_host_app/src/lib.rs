// SPDX-License-Identifier: AGPL-3.0-only
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]
//! An app library that links the `test_node_links_timer` node crate as a
//! plain library and constructs the node through its macro entry.

use cerulion_core::graph::node::NodeEntry;

/// Builds the linked node through its macro entry, as an app runtime does.
pub fn linked_node() -> Box<dyn NodeEntry> {
    Box::new(test_node_links_timer::TestNodeLinksTimerNodeEntry::new())
}

/// Keeps `linked_node` reachable from the cdylib's export table.
#[no_mangle]
pub extern "C" fn host_app_linked_node_count() -> usize {
    let entry = linked_node();
    entry
        .info()
        .map(|info| info.output_names().len())
        .unwrap_or(0)
}
