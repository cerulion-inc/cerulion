// SPDX-License-Identifier: AGPL-3.0-only
//! The vendored corpus, walked off disk, for the corpus tests.
//!
//! One entry per message the build generates: every `msg/<pkg>/<Name>.msg`,
//! and the two parts of every `srv/<pkg>/<Name>.srv` as `<Name>Request` and
//! `<Name>Response`. The walk is independent of `build.rs` (it reads the
//! files, not the generated registry), so a test comparing the two still
//! compares two separate enumerations.

use cerulion_core::codegen::{split_rossrv, srv_request_name, srv_response_name};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// `(pkg, Name) -> .msg text` for the vendored corpus, in sorted order.
pub fn vendored_sources() -> BTreeMap<(String, String), String> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    corpus_sources(&root.join("msg"), &root.join("srv"))
}

/// `(pkg, Name) -> .msg text` for the `.msg` files under `msg_root/<pkg>/`
/// and the `.srv` parts under `srv_root/<pkg>/` (the two roots may be the
/// same directory). A missing root contributes nothing. A `.srv` part that
/// has the name of a `.msg` in its package panics, as in `build.rs`.
pub fn corpus_sources(msg_root: &Path, srv_root: &Path) -> BTreeMap<(String, String), String> {
    let mut out = BTreeMap::new();
    for (pkg, name, path) in files_with_extension(msg_root, "msg") {
        let text = read(&path);
        out.insert((pkg, name), text);
    }
    for (pkg, name, path) in files_with_extension(srv_root, "srv") {
        let text = read(&path);
        let parts =
            split_rossrv(&text, &path.display().to_string()).unwrap_or_else(|e| panic!("{e}"));
        for (part, body) in [
            (srv_request_name(&name), parts.request),
            (srv_response_name(&name), parts.response),
        ] {
            let key = (pkg.clone(), part);
            assert!(
                !out.contains_key(&key),
                "{}: service part {}/{} collides with a .msg of the same name",
                path.display(),
                key.0,
                key.1
            );
            out.insert(key, body.to_string());
        }
    }
    out
}

fn read(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

/// `(pkg, stem, path)` of every `<root>/<pkg>/*.<ext>`, with `build.rs`'s
/// package-name sanitization applied.
fn files_with_extension(root: &Path, ext: &str) -> Vec<(String, String, PathBuf)> {
    let Ok(entries) = std::fs::read_dir(root) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for pkg_dir in entries.flatten().map(|e| e.path()).filter(|p| p.is_dir()) {
        let pkg = pkg_dir
            .file_name()
            .expect("package dir name")
            .to_string_lossy()
            .replace('-', "_");
        for path in std::fs::read_dir(&pkg_dir)
            .unwrap_or_else(|e| panic!("read {}: {e}", pkg_dir.display()))
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x == ext))
        {
            let stem = path
                .file_stem()
                .expect("file stem")
                .to_string_lossy()
                .to_string();
            out.push((pkg.clone(), stem, path));
        }
    }
    out.sort();
    out
}
