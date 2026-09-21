// SPDX-License-Identifier: AGPL-3.0-only
//! Inventory verb: pure-parser oracles + absent-fact-shape (absent = None) tests.

use cerud::verbs::inventory::{
    parse_cuda_version_txt, parse_l4t_release, parse_ldd_version, probe_inventory, Inventory,
};
use cerud::verbs::{InventoryVerb, VerbHandler};

#[test]
fn parse_ldd_version_oracles() {
    // Real `ldd --version` first lines from Debian/Ubuntu.
    assert_eq!(
        parse_ldd_version("ldd (Ubuntu GLIBC 2.31-0ubuntu9.9) 2.31"),
        Some("2.31".to_string())
    );
    assert_eq!(
        parse_ldd_version("ldd (Debian GLIBC 2.36-9+deb12u4) 2.36"),
        Some("2.36".to_string())
    );
    // No trailing version token → None (never fabricate one).
    assert_eq!(parse_ldd_version("ldd unknown build"), None);
    assert_eq!(parse_ldd_version(""), None);
}

#[test]
fn parse_cuda_version_txt_oracle() {
    assert_eq!(
        parse_cuda_version_txt("CUDA Version 12.2.140"),
        Some("12.2.140".to_string())
    );
    assert_eq!(
        parse_cuda_version_txt("CUDA Version 11.4.315\n"),
        Some("11.4.315".to_string())
    );
    assert_eq!(parse_cuda_version_txt("no version here"), None);
}

#[test]
fn parse_l4t_release_oracle() {
    // Real `/etc/nv_tegra_release` first line (JetPack 5.x on Orin).
    let content = "# R35 (release), REVISION: 4.1, GCID: 33958178, BOARD: t186ref, EABI: aarch64, DATE: Tue Aug  1 19:57:35 UTC 2023\n";
    assert_eq!(parse_l4t_release(content), Some("35.4.1".to_string()));

    // Major only when the revision is missing/odd.
    let major_only = "# R32 (release), BOARD: t210ref";
    assert_eq!(parse_l4t_release(major_only), Some("32".to_string()));

    // Not a Tegra release string.
    assert_eq!(parse_l4t_release("some other file"), None);
}

#[test]
fn inventory_absent_probes_serialize_as_null_never_fabricated() {
    // Hand-built inventory with every optional probe absent.
    let inv = Inventory {
        arch: "aarch64".to_string(),
        os: "linux".to_string(),
        kernel: None,
        glibc: None,
        cuda: None,
        jetpack_l4t: None,
        disk_free_bytes: None,
        cerulion_version: "0.1.0".to_string(),
    };
    let v = serde_json::to_value(&inv).unwrap();
    // Absent facts are an EXPLICIT JSON null — not a fabricated
    // default and not an OMITTED key. `serde_json` indexing (`v["kernel"]`)
    // returns `Null` for BOTH a present null AND a missing key, so it cannot
    // tell the two apart; assert the key is PRESENT in the object first, then
    // that its value is null. (A `#[serde(skip_serializing_if)]` regression
    // that dropped the key would slip past a bare `is_null()`.)
    let obj = v
        .as_object()
        .expect("inventory serializes to a JSON object");
    for key in ["kernel", "glibc", "cuda", "jetpack_l4t", "disk_free_bytes"] {
        assert!(
            obj.contains_key(key),
            "absent fact '{key}' must serialize as a present key"
        );
        assert!(
            obj[key].is_null(),
            "absent fact '{key}' must be an explicit null"
        );
    }
    assert_eq!(v["arch"], "aarch64");
    assert_eq!(v["cerulion_version"], "0.1.0");

    // Roundtrips.
    let back: Inventory = serde_json::from_value(v).unwrap();
    assert_eq!(back, inv);
}

#[test]
fn probe_inventory_reports_always_present_facts() {
    let inv = probe_inventory();
    // arch / os / cerulion_version are always determinable on this host.
    assert!(!inv.arch.is_empty());
    assert!(!inv.os.is_empty());
    assert!(!inv.cerulion_version.is_empty());
    // Disk free on `/` is available on any test host.
    assert!(inv.disk_free_bytes.map(|b| b > 0).unwrap_or(false));
}

#[test]
fn inventory_verb_executes_to_a_json_object() {
    let verb = InventoryVerb;
    assert_eq!(verb.name(), "inventory");
    // A read-only probe declares is_mutating = false.
    assert!(!verb.is_mutating());
    let result = verb.execute(&serde_json::json!({})).unwrap();
    assert!(result.is_object());
    assert!(result["arch"].is_string());
    assert!(result["cerulion_version"].is_string());
}
