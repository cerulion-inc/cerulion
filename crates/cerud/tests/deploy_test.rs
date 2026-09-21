// SPDX-License-Identifier: AGPL-3.0-only
//! Deploy-bundle manifest digest verification + symlink-plan oracles.

use std::collections::BTreeMap;
use std::path::Path;

use cerud::deploy::{
    compute_bundle_hash, plan_deploy, plan_rollback, validate_bundle_relpath, verify_bundle,
    verify_bundle_hash, verify_file_digest, BundleFile, BundleFileRole, BundleLayout,
    BundleManifest, InstalledBundle, MANIFEST_FORMAT_VERSION,
};
use cerud::error::CerudError;
use cerud::hash::sha256_hex;

fn manifest_with(files: Vec<BundleFile>) -> BundleManifest {
    let mut schema_hashes = BTreeMap::new();
    schema_hashes.insert("/tf".to_string(), 0x1234_5678_9abc_def0u64);
    let mut m = BundleManifest {
        format_version: MANIFEST_FORMAT_VERSION,
        bundle_hash: String::new(),
        cerulion_version: "0.1.0".to_string(),
        target_glibc: Some("2.31".to_string()),
        schema_hashes,
        files,
    };
    m.bundle_hash = compute_bundle_hash(&m);
    m
}

#[test]
fn sha256_hex_matches_known_answer_vectors() {
    // NIST FIPS-180 known-answer tests — a real external oracle, not a
    // self-compare. Pins the digest primitive the whole crate depends on.
    assert_eq!(
        sha256_hex(b""),
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    );
    assert_eq!(
        sha256_hex(b"abc"),
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
    );
}

#[test]
fn validate_bundle_relpath_refuses_traversal() {
    // Safe relative paths pass.
    validate_bundle_relpath("graphs/perception.yaml").unwrap();
    validate_bundle_relpath("node.so").unwrap();
    validate_bundle_relpath("./a.yaml").unwrap();
    // Hostile paths are refused.
    for bad in [
        "",
        "..",
        "../../etc/passwd",
        "a/../../b",
        "/etc/passwd",
        "/",
    ] {
        match validate_bundle_relpath(bad) {
            Err(CerudError::Manifest(msg)) => assert!(msg.contains("unsafe"), "{bad}: {msg}"),
            other => panic!("expected Manifest error for {bad:?}, got {other:?}"),
        }
    }
}

#[test]
fn verify_refuses_a_hostile_manifest_path_before_touching_the_fs() {
    let dir = tempfile::tempdir().unwrap();
    let bundle_dir = dir.path();
    // A malicious manifest tries to read outside the bundle dir.
    let hostile = BundleFile {
        path: "../../../../etc/passwd".to_string(),
        sha256: sha256_hex(b"whatever"),
        role: BundleFileRole::Other,
    };
    match verify_file_digest(bundle_dir, &hostile) {
        Err(CerudError::Manifest(msg)) => assert!(msg.contains("unsafe")),
        other => panic!("expected Manifest refusal, got {other:?}"),
    }
    // And through the whole-bundle path (validate_paths runs first).
    let m = manifest_with(vec![hostile]);
    match verify_bundle(bundle_dir, &m) {
        Err(CerudError::Manifest(_)) => {}
        other => panic!("expected Manifest refusal, got {other:?}"),
    }
}

#[cfg(unix)]
#[test]
fn verify_file_digest_refuses_a_symlink_bundle_file() {
    // A content-addressed bundle must contain no symlinks —
    // a symlink component would escape the bundle dir on the digest read. Even
    // though the symlink's TARGET content would hash-match, the symlink type is
    // refused BEFORE the read.
    let dir = tempfile::tempdir().unwrap();
    let bundle_dir = dir.path();
    let outside = tempfile::tempdir().unwrap();
    let secret = outside.path().join("secret");
    std::fs::write(&secret, b"outside-the-bundle content").unwrap();

    // A symlink inside the bundle dir pointing OUTSIDE it.
    let link = bundle_dir.join("node.so");
    std::os::unix::fs::symlink(&secret, &link).unwrap();

    let file = BundleFile {
        path: "node.so".to_string(),
        sha256: sha256_hex(b"outside-the-bundle content"), // would match if read
        role: BundleFileRole::Cdylib,
    };
    match verify_file_digest(bundle_dir, &file) {
        Err(CerudError::Manifest(msg)) => assert!(msg.contains("symlink"), "msg: {msg}"),
        other => panic!("expected a symlink refusal, got {other:?}"),
    }
}

#[test]
fn bundle_hash_is_stable_and_self_verifies() {
    let m = manifest_with(vec![BundleFile {
        path: "graphs/perception.yaml".to_string(),
        sha256: sha256_hex(b"graph yaml bytes"),
        role: BundleFileRole::GraphYaml,
    }]);
    verify_bundle_hash(&m).unwrap();
    // Recompute is deterministic.
    assert_eq!(compute_bundle_hash(&m), m.bundle_hash);
}

#[test]
fn bundle_hash_is_file_order_independent() {
    let f1 = BundleFile {
        path: "a.yaml".to_string(),
        sha256: sha256_hex(b"a"),
        role: BundleFileRole::GraphYaml,
    };
    let f2 = BundleFile {
        path: "b.so".to_string(),
        sha256: sha256_hex(b"b"),
        role: BundleFileRole::Cdylib,
    };
    let m1 = manifest_with(vec![f1.clone(), f2.clone()]);
    let m2 = manifest_with(vec![f2, f1]);
    // The content address is order-independent (files sorted internally).
    assert_eq!(m1.bundle_hash, m2.bundle_hash);
}

#[test]
fn tampering_manifest_metadata_breaks_the_content_address() {
    let mut m = manifest_with(vec![BundleFile {
        path: "graphs/x.yaml".to_string(),
        sha256: sha256_hex(b"x"),
        role: BundleFileRole::GraphYaml,
    }]);
    // Alter the cerulion_version WITHOUT recomputing bundle_hash.
    m.cerulion_version = "9.9.9".to_string();
    match verify_bundle_hash(&m) {
        Err(CerudError::Manifest(msg)) => assert!(msg.contains("bundle_hash mismatch")),
        other => panic!("expected Manifest mismatch, got {other:?}"),
    }
}

#[test]
fn file_digest_verification_matches_on_disk_bytes() {
    let dir = tempfile::tempdir().unwrap();
    let bundle_dir = dir.path();
    let bytes = b"the real graph yaml content";
    std::fs::write(bundle_dir.join("graph.yaml"), bytes).unwrap();

    let good = BundleFile {
        path: "graph.yaml".to_string(),
        sha256: sha256_hex(bytes),
        role: BundleFileRole::GraphYaml,
    };
    verify_file_digest(bundle_dir, &good).unwrap();

    // A wrong digest is caught.
    let bad = BundleFile {
        path: "graph.yaml".to_string(),
        sha256: sha256_hex(b"different bytes"),
        role: BundleFileRole::GraphYaml,
    };
    match verify_file_digest(bundle_dir, &bad) {
        Err(CerudError::DigestMismatch { path, .. }) => assert_eq!(path, "graph.yaml"),
        other => panic!("expected DigestMismatch, got {other:?}"),
    }
}

#[test]
fn verify_bundle_checks_both_content_address_and_files() {
    let dir = tempfile::tempdir().unwrap();
    let bundle_dir = dir.path();
    let bytes = b"cdylib bytes here";
    std::fs::write(bundle_dir.join("node.so"), bytes).unwrap();

    let m = manifest_with(vec![BundleFile {
        path: "node.so".to_string(),
        sha256: sha256_hex(bytes),
        role: BundleFileRole::Cdylib,
    }]);
    verify_bundle(bundle_dir, &m).unwrap();
}

// ─────────────────────────────── symlink plan ──────────────────────────────

#[test]
fn bundle_layout_paths() {
    let layout = BundleLayout::new("/var/lib/cerulion", 3);
    assert_eq!(layout.bundles_dir(), Path::new("/var/lib/cerulion/bundles"));
    assert_eq!(
        layout.bundle_dir("abc123"),
        Path::new("/var/lib/cerulion/bundles/abc123")
    );
    assert_eq!(
        layout.current_link(),
        Path::new("/var/lib/cerulion/current")
    );
}

#[test]
fn plan_deploy_points_current_and_prunes_to_keep_last() {
    let layout = BundleLayout::new("/var/lib/cerulion", 2);
    let installed = vec![
        InstalledBundle {
            hash: "A".to_string(),
            installed_at_ns: 100,
        },
        InstalledBundle {
            hash: "B".to_string(),
            installed_at_ns: 200,
        },
    ];
    let new = InstalledBundle {
        hash: "C".to_string(),
        installed_at_ns: 300,
    };
    let plan = plan_deploy(&layout, &installed, Some("B"), &new);

    assert_eq!(plan.target_hash, "C");
    assert_eq!(plan.target_dir, Path::new("/var/lib/cerulion/bundles/C"));
    assert_eq!(plan.current_link, Path::new("/var/lib/cerulion/current"));
    assert_eq!(plan.previous_hash, Some("B".to_string()));
    // keep_last = 2 → retain {C, B}; prune the oldest (A).
    assert_eq!(plan.prune, vec!["A".to_string()]);
}

#[test]
fn plan_deploy_protects_the_rollback_anchor_beyond_keep_last() {
    // keep_last = 1 would retain only the new bundle, but the previous
    // `current` (the rollback anchor) is protected from pruning.
    let layout = BundleLayout::new("/var/lib/cerulion", 1);
    let installed = vec![
        InstalledBundle {
            hash: "A".to_string(),
            installed_at_ns: 100,
        },
        InstalledBundle {
            hash: "B".to_string(),
            installed_at_ns: 200,
        },
    ];
    let new = InstalledBundle {
        hash: "C".to_string(),
        installed_at_ns: 300,
    };
    let plan = plan_deploy(&layout, &installed, Some("B"), &new);
    // A is pruned; B (the rollback anchor) survives; C is the new target.
    assert_eq!(plan.prune, vec!["A".to_string()]);
    assert_eq!(plan.previous_hash, Some("B".to_string()));
}

#[test]
fn plan_deploy_prunes_multiple_bundles_beyond_keep_last() {
    let layout = BundleLayout::new("/var/lib/cerulion", 1);
    let installed = vec![
        InstalledBundle {
            hash: "A".to_string(),
            installed_at_ns: 100,
        },
        InstalledBundle {
            hash: "B".to_string(),
            installed_at_ns: 200,
        },
        InstalledBundle {
            hash: "C".to_string(),
            installed_at_ns: 300,
        },
    ];
    let new = InstalledBundle {
        hash: "D".to_string(),
        installed_at_ns: 400,
    };
    let plan = plan_deploy(&layout, &installed, Some("C"), &new);
    // keep_last=1 retains D; C is the protected rollback anchor; A and B are
    // BOTH pruned (multi-prune).
    let mut pruned = plan.prune.clone();
    pruned.sort();
    assert_eq!(pruned, vec!["A".to_string(), "B".to_string()]);
    assert!(
        !plan.prune.contains(&"C".to_string()),
        "current is protected"
    );
    assert!(
        !plan.prune.contains(&"D".to_string()),
        "new is never pruned"
    );
}

#[test]
fn plan_deploy_is_deterministic_under_equal_install_times() {
    // Equal install times must still yield a STABLE plan (hash tie-break), so
    // two identical calls produce byte-identical prune lists.
    let layout = BundleLayout::new("/var/lib/cerulion", 2);
    let installed = vec![
        InstalledBundle {
            hash: "aaa".to_string(),
            installed_at_ns: 500,
        },
        InstalledBundle {
            hash: "bbb".to_string(),
            installed_at_ns: 500,
        },
        InstalledBundle {
            hash: "ccc".to_string(),
            installed_at_ns: 500,
        },
    ];
    let new = InstalledBundle {
        hash: "ddd".to_string(),
        installed_at_ns: 500,
    };
    let p1 = plan_deploy(&layout, &installed, Some("ccc"), &new);
    let p2 = plan_deploy(&layout, &installed, Some("ccc"), &new);
    assert_eq!(p1, p2, "plan must be deterministic under equal times");
    // The new bundle and the current rollback anchor are never pruned.
    assert!(!p1.prune.contains(&"ddd".to_string()));
    assert!(!p1.prune.contains(&"ccc".to_string()));
}

#[test]
fn plan_rollback_repoints_to_an_installed_bundle() {
    let layout = BundleLayout::new("/var/lib/cerulion", 3);
    let installed = vec![
        InstalledBundle {
            hash: "A".to_string(),
            installed_at_ns: 100,
        },
        InstalledBundle {
            hash: "B".to_string(),
            installed_at_ns: 200,
        },
    ];
    let plan = plan_rollback(&layout, &installed, Some("B"), "A").unwrap();
    assert_eq!(plan.target_hash, "A");
    assert_eq!(plan.previous_hash, Some("B".to_string()));
    assert!(plan.prune.is_empty(), "rollback prunes nothing");
}

#[test]
fn plan_rollback_refuses_a_non_installed_target() {
    let layout = BundleLayout::default();
    let installed = vec![InstalledBundle {
        hash: "A".to_string(),
        installed_at_ns: 100,
    }];
    match plan_rollback(&layout, &installed, Some("A"), "Z") {
        Err(CerudError::Manifest(msg)) => assert!(msg.contains("not among the installed")),
        other => panic!("expected Manifest error, got {other:?}"),
    }
}
