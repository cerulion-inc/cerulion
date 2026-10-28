// SPDX-License-Identifier: AGPL-3.0-only
//! The read-identity harness's node sources build. Every node type the box
//! oracles run is created, its source written and built with `node build`, so
//! a source the node macros refuse (the determinism lint runs at that build)
//! fails here on any host instead of at a box run. Nothing is loaded and no
//! graph runs.

#![cfg(unix)]

#[allow(dead_code)]
mod det_scaffold;
#[allow(dead_code)]
mod mp_support;
#[allow(dead_code)]
mod read_identity;

#[test]
fn every_read_identity_node_type_builds() {
    let ws = read_identity::workspace();
    for (name, cdylib) in read_identity::built_cdylibs(ws) {
        assert!(
            cdylib.is_file(),
            "RI_HARNESS: node type {name} built no library at {}",
            cdylib.display()
        );
    }
}
