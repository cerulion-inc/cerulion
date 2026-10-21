// SPDX-License-Identifier: AGPL-3.0-only
//! `CERULION_HOME` moves every per-machine file the CLI resolves, and with it
//! unset every one of them lands in `~/.cerulion`.
//!
//! One test function, so the process environment it sets is never shared with
//! a concurrently running test in this binary.
//!
//! The source walk at the end refuses shipped code with a `".cerulion"`
//! literal within six lines after a `home_dir` or `"HOME"` mention, outside an
//! allowed list. The listed mirrors sit in crates without `cerulion_pairing`
//! and repeat its rule; the path assertions pin each of them. The listed
//! exemptions are not config files: they keep their own location rules.

use std::path::{Path, PathBuf};

use cerulion_cli_engine::{account_cmd, auth, connect_cmd, peer_cache, ros2_cmd};

/// Every resolver under test, as (name, resolved path).
fn resolved() -> Vec<(&'static str, Option<PathBuf>)> {
    let layout = auth::identity_layout();
    let mut out = vec![
        ("config dir", auth::cerulion_config_dir()),
        ("auth.json", auth::auth_json_path()),
        ("device.cert", auth::device_cert_path()),
        (
            "identity/device.key",
            layout.as_ref().map(|l| l.device_key_path()),
        ),
        ("desk.key", connect_cmd::desk_key_path()),
        ("robots.toml", connect_cmd::robots_toml_path()),
        ("peers.json", peer_cache::default_cache_path()),
        (
            "runs",
            cerulion_core::transport::run_registry::run_dir_root(),
        ),
        ("epochs", account_cmd::resolve_epoch_cache_path("go2")),
        (
            "ros2",
            ros2_cmd::ament_prefix_path(Path::new("/opt/build/release")).map(|p| {
                p.parent()
                    .expect("a staging prefix has a parent")
                    .to_path_buf()
            }),
        ),
        (
            "epochs next to the connect key",
            epoch_dir_next_to(connect_cmd::desk_key_path().as_deref()),
        ),
    ];
    #[cfg(feature = "telemetry")]
    out.push((
        "telemetry.json",
        cerulion_telemetry::consent::file_path().ok(),
    ));
    out
}

/// The `epochs/` directory `cerulion-connectd` and `cerulion-netd` derive from
/// a key file path when `CERULION_EPOCH_DIR` is unset.
fn epoch_dir_next_to(key_file: Option<&Path>) -> Option<PathBuf> {
    account_cmd::resolve_epoch_dir(None, key_file)
}

fn expected(dir: &Path) -> Vec<(&'static str, Option<PathBuf>)> {
    let mut out = vec![
        ("config dir", Some(dir.to_path_buf())),
        ("auth.json", Some(dir.join("auth.json"))),
        ("device.cert", Some(dir.join("device.cert"))),
        ("identity/device.key", Some(dir.join("identity/device.key"))),
        ("desk.key", Some(dir.join("desk.key"))),
        ("robots.toml", Some(dir.join("robots.toml"))),
        ("peers.json", Some(dir.join("peers.json"))),
        ("runs", Some(dir.join("runs"))),
        ("epochs", Some(dir.join("epochs/go2.epoch"))),
        ("ros2", Some(dir.join("ros2"))),
        ("epochs next to the connect key", Some(dir.join("epochs"))),
    ];
    #[cfg(feature = "telemetry")]
    out.push(("telemetry.json", Some(dir.join("telemetry.json"))));
    out
}

#[test]
fn cerulion_home_moves_every_machine_file_and_unset_means_dot_cerulion() {
    let home = tempfile::tempdir().unwrap();
    let rig = tempfile::tempdir().unwrap();
    // This is the only test in this binary, so no other test reads the
    // environment while it changes.
    std::env::set_var("HOME", home.path());
    std::env::remove_var("CERULION_EPOCH_DIR");

    // Overridden: everything moves into CERULION_HOME, verbatim.
    std::env::set_var("CERULION_HOME", rig.path());
    assert_eq!(resolved(), expected(rig.path()));

    // The machine key written through the one writer lands there, and nothing
    // appears under the home directory.
    let layout = auth::identity_layout().unwrap();
    let (key, _) = layout.load_or_create_machine_key().unwrap();
    assert_eq!(
        std::fs::read(rig.path().join("identity/device.key")).unwrap(),
        key.seed()
    );
    assert_eq!(
        std::fs::read(rig.path().join("desk.key")).unwrap(),
        key.seed()
    );
    assert!(!home.path().join(".cerulion").exists());

    // Empty counts as unset.
    std::env::set_var("CERULION_HOME", "");
    assert_eq!(resolved(), expected(&home.path().join(".cerulion")));

    // Unset: everything under ~/.cerulion.
    std::env::remove_var("CERULION_HOME");
    assert_eq!(resolved(), expected(&home.path().join(".cerulion")));

    no_new_home_dir_resolver();
}

/// Files that join the home directory with `".cerulion"` themselves.
const ALLOWED: &[&str] = &[
    // Mirrors of `cerulion_pairing::identity::config_dir_from` in crates
    // without that dependency, pinned by the assertions above.
    "crates/cerulion_core/src/transport/run_registry.rs",
    "crates/cerulion_discovery/src/peer_cache.rs",
    "crates/cerulion_telemetry/src/consent.rs",
    // Not config: daemon sockets prefer XDG_RUNTIME_DIR and a per-daemon
    // variable before `$HOME/.cerulion`.
    "crates/cerulion_hygiene/src/lib.rs",
    // Not config: the downloaded decoder cache, moved by its own variable.
    "crates/cerulion_viz/lib/cerulion_viz/src/video_decode.rs",
];

fn no_new_home_dir_resolver() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut found: Vec<String> = Vec::new();
    let mut files = Vec::new();
    collect_rs(&root.join("crates"), &mut files);
    assert!(
        files.len() > 100,
        "the walk found only {} files",
        files.len()
    );
    for file in files {
        let text = std::fs::read_to_string(&file).unwrap();
        if joins_home_with_dot_cerulion(&text) {
            let rel = file
                .strip_prefix(&root)
                .unwrap()
                .to_string_lossy()
                .replace('\\', "/");
            found.push(rel);
        }
    }
    found.sort();
    let mut want: Vec<String> = ALLOWED.iter().map(|s| (*s).to_string()).collect();
    want.sort();
    assert_eq!(
        found, want,
        "a home directory + \".cerulion\" resolver outside the allowed list: resolve the config \
         directory through cerulion_pairing::identity::config_dir (or the CLI's \
         auth::cerulion_config_dir) instead; an allowed entry that no longer matches must leave \
         the list"
    );
}

/// Whether the non-test part of `text` (everything before its first
/// `#[cfg(test)]` module) has a `".cerulion"` literal on a line that, or one of
/// the six lines before it, names `home_dir` or `"HOME"`.
fn joins_home_with_dot_cerulion(text: &str) -> bool {
    let shipped = match text.find("#[cfg(test)]\nmod ") {
        Some(at) => &text[..at],
        None => text,
    };
    let lines: Vec<&str> = shipped.lines().collect();
    lines.iter().enumerate().any(|(i, line)| {
        line.contains("\".cerulion\"")
            && lines[i.saturating_sub(6)..=i]
                .iter()
                .any(|l| l.contains("home_dir") || l.contains("\"HOME\""))
    })
}

fn collect_rs(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        let name = path.file_name().unwrap().to_string_lossy();
        if path.is_dir() {
            if name == "target" || name.starts_with('.') {
                continue;
            }
            collect_rs(&path, out);
        } else if name.ends_with(".rs") && path.components().any(|c| c.as_os_str() == "src") {
            out.push(path);
        }
    }
}
