// SPDX-License-Identifier: AGPL-3.0-only
//! The LOCAL half of schema acquisition: end-to-end pins for [`LocalAmentAcquirer`]
//! driven through the REAL `cerulion ros2 attach` ladder
//! (`ros_cmd::ros_attach_with_acquirer`) over a FAKE ament install tree.
//!
//! No DDS peer — a hand-built endpoint list is injected through the
//! `cerulion_dds::DdsDiscovery` seam; the acquirer is a REAL
//! [`LocalAmentAcquirer`] over a tempdir install. Every assertion is against a
//! HAND oracle (byte-verbatim `.msg`, the exact report marker). Parallel-safe
//! (tempdirs only; no DDS one-per-process slot) — NO `#[serial]`.

use std::net::IpAddr;
use std::path::Path;
use std::time::Duration;

use cerulion_cli_engine::error::CliResult;
use cerulion_cli_engine::local_harvest::LocalAmentAcquirer;
use cerulion_cli_engine::ros_cmd::{
    self, AttachAcquirers, AttachOutcome, AttachSchemaChain, RosAttachOptions,
};
use cerulion_dds::{
    DdsDiscovery, DdsError, DiscoveredEndpoint, DiscoveredQos, DiscoveryParams, DiscoveryResult,
    EndpointKind, QosDurability, QosReliability, SchemaAcquirer,
};

// ─────────────────────────── Fixtures / seams ──────────────────────────────

/// A `DdsDiscovery` replaying a hand-built endpoint list.
struct FakeDiscovery {
    endpoints: Vec<DiscoveredEndpoint>,
}
impl DdsDiscovery for FakeDiscovery {
    fn discover(&self, _params: &DiscoveryParams) -> Result<DiscoveryResult, DdsError> {
        Ok(DiscoveryResult {
            endpoints: self.endpoints.clone(),
            own_endpoints_hidden: 0,
            // Unused by the pure engine logic (the wire rung
            // is a live acquirer wired in the binary).
            nodes: Vec::new(),
            participants: Vec::new(),
        })
    }
}

const BE_VOL: DiscoveredQos = DiscoveredQos {
    reliability: QosReliability::BestEffort,
    durability: QosDurability::Volatile,
};

fn iface() -> IpAddr {
    "192.168.123.18".parse().unwrap()
}

fn opts(dry_run: bool, assume_yes: bool) -> RosAttachOptions {
    RosAttachOptions {
        iface: iface(),
        domain_id: 0,
        window: Duration::from_secs(5),
        dry_run,
        assume_yes,
        graph_name: "attach".to_string(),
        topic_prefix: None,
        robot_name: None,
    }
}

fn panic_confirm(_preview: &str) -> CliResult<bool> {
    panic!("the confirm provider must not be invoked on this path")
}

/// One writer publishing `acme_msgs/Widget` (mangled discovery form) — the
/// acquisition candidate (no built-in / typed registry resolves it).
fn widget_writer() -> FakeDiscovery {
    FakeDiscovery {
        endpoints: vec![DiscoveredEndpoint {
            dds_topic: "rt/widget".to_string(),
            type_name: "acme_msgs::msg::dds_::Widget_".to_string(),
            qos: BE_VOL,
            kind: EndpointKind::Writer,
            // Unused by the local ament rung path under test.
            type_hash: None,
            writer_guid: None,
        }],
    }
}

/// Write `<prefix>/share/<pkg>/msg/<ty>.msg` = `text`, and (when `in_index`)
/// append `msg/<ty>.msg` to the pkg's ament index marker — a FAKE ament tree.
fn write_ament_msg(prefix: &Path, pkg: &str, ty: &str, text: &str, in_index: bool) {
    let msg_dir = prefix.join("share").join(pkg).join("msg");
    std::fs::create_dir_all(&msg_dir).unwrap();
    std::fs::write(msg_dir.join(format!("{ty}.msg")), text).unwrap();
    if in_index {
        let idx_dir = prefix
            .join("share")
            .join("ament_index")
            .join("resource_index")
            .join("rosidl_interfaces");
        std::fs::create_dir_all(&idx_dir).unwrap();
        let marker = idx_dir.join(pkg);
        let mut content = std::fs::read_to_string(&marker).unwrap_or_default();
        content.push_str(&format!("msg/{ty}.msg\n"));
        std::fs::write(&marker, content).unwrap();
    }
}

/// Write a workspace store `.msg` (`schemas/<pkg>/msg/<Type>.msg`).
fn write_store_msg(root: &Path, pkg: &str, ty: &str, text: &str) {
    let dir = root.join("schemas").join(pkg).join("msg");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(format!("{ty}.msg")), text).unwrap();
}

fn read(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap()
}

const WIDGET_MSG: &str = "int32 id\nfloat64 value\n";

// ─────────────────────────────── TEST 9 ────────────────────────────────────

/// End-to-end through `ros_attach_with_acquirer` with a REAL
/// `LocalAmentAcquirer` over a fake install: the previously-UNRESOLVABLE topic
/// flips to RESOLVABLE with the "(schema acquired — …)" marker; `--yes`
/// materializes the store file BYTE-IDENTICAL to the install file; dry-run
/// writes nothing.
#[test]
fn test_e2e_local_ament_acquire_flips_topic_and_materializes_byte_verbatim() {
    // The fake ROS install carrying acme_msgs/Widget.
    let install = tempfile::tempdir().unwrap();
    write_ament_msg(install.path(), "acme_msgs", "Widget", WIDGET_MSG, true);
    let acquirer = LocalAmentAcquirer::new(vec![install.path().to_path_buf()]);

    // Dry-run: the report flips + names the would-be file, writing NOTHING.
    {
        let ws = tempfile::tempdir().unwrap();
        let mut confirm = panic_confirm;
        let report = ros_cmd::ros_attach_with_acquirer(
            &widget_writer(),
            &acquirer,
            ws.path(),
            &opts(true, false),
            true,
            &mut confirm,
        )
        .expect("dry run");

        assert_eq!(report.mappings.len(), 1);
        assert_eq!(report.mappings[0].ros_type, "acme_msgs/Widget");
        // Fix D2: the acquired note AUGMENTS the route marker (both the HOW and
        // the not-yet-written state).
        assert!(
            report.report.contains(
                "(bridged via the generic codec; schema acquired — writes \
                 schemas/acme_msgs/msg/Widget.msg on consent)"
            ),
            "{}",
            report.report
        );
        // The rung provenance (paren-free per fix D4) appears in the ACQUIRED
        // SCHEMAS section (fix D3).
        assert!(
            report.report.contains("local ROS install via ament index"),
            "{}",
            report.report
        );
        // Never-mutate floor: no schemas/ dir was created.
        assert!(!ws.path().join("schemas").exists());
        assert!(matches!(report.outcome, AttachOutcome::DryRun));
    }

    // --yes: materializes the store file byte-identical to the install file.
    {
        let ws = tempfile::tempdir().unwrap();
        let mut confirm = panic_confirm;
        let report = ros_cmd::ros_attach_with_acquirer(
            &widget_writer(),
            &acquirer,
            ws.path(),
            &opts(false, true),
            false,
            &mut confirm,
        )
        .expect("assume-yes write");
        assert!(matches!(report.outcome, AttachOutcome::Written { .. }));

        let dest = ws
            .path()
            .join("schemas")
            .join("acme_msgs")
            .join("msg")
            .join("Widget.msg");
        // BYTE-IDENTICAL to the fake install's own file (hand oracle).
        assert_eq!(read(&dest), WIDGET_MSG);
        let install_file = install
            .path()
            .join("share")
            .join("acme_msgs")
            .join("msg")
            .join("Widget.msg");
        assert_eq!(read(&dest), read(&install_file));

        // The materialized store immediately resolves the type with NO acquirer.
        let chain = AttachSchemaChain::from_workspace(ws.path());
        assert!(chain.resolves("acme_msgs/Widget"));

        match &report.outcome {
            AttachOutcome::Written { schema_writes, .. } => {
                assert_eq!(schema_writes.len(), 1);
                assert_eq!(schema_writes[0].path, dest);
                assert!(schema_writes[0].backup.is_none());
            }
            other => panic!("expected Written, got {other:?}"),
        }
    }
}

// ─────────────────────────────── TEST 11 ───────────────────────────────────

/// STORE-CLOBBER decision: the acquirer includes every non-built
/// -in dep it harvested (here a nested dep that ALSO lives in the workspace
/// store with DIFFERENT bytes), and the DRIVER's existing staging filter omits
/// any closure member already resolvable via the store — so the committed store
/// file is NEVER clobbered by the re-acquired twin. Only the genuinely-new
/// requested type is written.
#[test]
fn test_store_dep_is_not_clobbered_by_reacquired_twin() {
    // The workspace store already carries dep_pkg/DepType (the COMMITTED bytes).
    const STORE_DEP: &str = "int32 v\n";
    const AMENT_DEP: &str = "int32 v\nint32 w\n"; // a DIFFERENT (twin) definition
    let ws = tempfile::tempdir().unwrap();
    write_store_msg(ws.path(), "dep_pkg", "DepType", STORE_DEP);

    // The fake install carries Widget (references dep_pkg/DepType) AND its own
    // dep_pkg/DepType with the DIFFERENT bytes.
    let install = tempfile::tempdir().unwrap();
    write_ament_msg(
        install.path(),
        "acme_msgs",
        "Widget",
        "int32 id\ndep_pkg/DepType dep\n",
        true,
    );
    write_ament_msg(install.path(), "dep_pkg", "DepType", AMENT_DEP, true);
    let acquirer = LocalAmentAcquirer::new(vec![install.path().to_path_buf()]);

    // Sanity: the acquirer's bundle DOES include both (it has no store
    // knowledge) — the never-clobber guarantee is the driver's, not the
    // acquirer's.
    let bundle = acquirer.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
    match &bundle[0].outcome {
        cerulion_dds::AcquisitionOutcome::Acquired(s) => {
            let names: Vec<String> = s.closure.iter().map(|m| m.qualified_name()).collect();
            assert_eq!(
                names,
                vec![
                    "acme_msgs/Widget".to_string(),
                    "dep_pkg/DepType".to_string()
                ],
                "acquirer includes the harvested store-twin dep"
            );
        }
        other => panic!("expected Acquired, got {other:?}"),
    }

    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_writer(),
        &acquirer,
        ws.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("assume-yes write");

    let widget_dest = ws
        .path()
        .join("schemas")
        .join("acme_msgs")
        .join("msg")
        .join("Widget.msg");
    let dep_dest = ws
        .path()
        .join("schemas")
        .join("dep_pkg")
        .join("msg")
        .join("DepType.msg");

    // The committed store dep is UNCHANGED (never clobbered by the twin).
    assert_eq!(
        read(&dep_dest),
        STORE_DEP,
        "store dep must not be clobbered"
    );
    // Only Widget (the genuinely-new type) was written; the dep was NOT.
    match &report.outcome {
        AttachOutcome::Written { schema_writes, .. } => {
            let written: Vec<_> = schema_writes.iter().map(|w| w.path.clone()).collect();
            assert!(
                written.contains(&widget_dest),
                "Widget must be written: {written:?}"
            );
            assert!(
                !written.contains(&dep_dest),
                "store-resolved dep must NOT be re-written: {written:?}"
            );
            assert_eq!(schema_writes.len(), 1);
        }
        other => panic!("expected Written, got {other:?}"),
    }
    // Widget itself was written verbatim.
    assert_eq!(read(&widget_dest), "int32 id\ndep_pkg/DepType dep\n");
}

// ─────────────────────────────── Production factory ────────────────────────

/// The production factory ([`AttachAcquirers`]) composes a ONE-rung ladder
/// whose first (only, today) rung is the LOCAL ament harvest (the wire rung
/// prepends). Behavioral, not a downcast: a fake install carrying
/// `acme_msgs/Widget` is acquired VIA the composed chain and the outcome's rung
/// is `LocalAment`.
#[test]
fn test_attach_acquirers_first_rung_is_local_ament() {
    let install = tempfile::tempdir().unwrap();
    write_ament_msg(install.path(), "acme_msgs", "Widget", WIDGET_MSG, true);
    let prefixes = vec![install.path().to_path_buf()];
    let acquirers = AttachAcquirers::with_local_ament(LocalAmentAcquirer::new(prefixes));
    assert_eq!(
        acquirers.rungs().len(),
        1,
        "one rung today (the wire rung prepends at 4b)"
    );

    let chain = acquirers.chain();
    let out = chain.acquire(&["acme_msgs/Widget".to_string()], &DiscoveryResult::empty());
    assert_eq!(out.len(), 1);
    match &out[0].outcome {
        cerulion_dds::AcquisitionOutcome::Acquired(s) => {
            assert_eq!(s.rung, cerulion_dds::AcquisitionRung::LocalAment);
            assert_eq!(s.closure[0].qualified_name(), "acme_msgs/Widget");
        }
        other => panic!("expected Acquired via LocalAment, got {other:?}"),
    }
}

/// End-to-end config-points-at-real-files pin: a full `--yes` acquire run writes
/// the acquired `.msg` into `schemas/` AND emits a bridge config whose
/// `msg_dirs:` points at the store now holding it — and that store dir really
/// resolves the type.
#[test]
fn test_e2e_attach_writes_msg_dirs_pointing_at_materialized_store() {
    let install = tempfile::tempdir().unwrap();
    write_ament_msg(install.path(), "acme_msgs", "Widget", WIDGET_MSG, true);
    let acquirer = LocalAmentAcquirer::new(vec![install.path().to_path_buf()]);

    let ws = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_writer(),
        &acquirer,
        ws.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("assume-yes write");
    assert!(matches!(report.outcome, AttachOutcome::Written { .. }));

    // The acquired schema landed in the store.
    let dest = ws
        .path()
        .join("schemas")
        .join("acme_msgs")
        .join("msg")
        .join("Widget.msg");
    assert_eq!(read(&dest), WIDGET_MSG);

    // The written bridge config points msg_dirs at the store dir — as
    // `../schemas`, relative to the CONFIG FILE's directory (supplement A: the
    // config lives in graphs/, and the bridge joins relative entries to its
    // config path at load, so the run works from any CWD).
    let config_path = ws.path().join("graphs").join("attach.bridge.yaml");
    let config = read(&config_path);
    assert!(
        config.contains("\nmsg_dirs:\n"),
        "config emits msg_dirs: {config}"
    );
    assert!(
        config.contains("\n  - ../schemas\n"),
        "msg_dirs names the store dir relative to the config file: {config}"
    );

    // config-points-at-REAL-files, resolved EXACTLY as the bridge does: the
    // emitted relative entry joined to the config file's parent directory
    // reaches the store that holds the type — CWD-independent (the engine-seam
    // half of the supplement-A doomed-subdir fix; the bridge-side join is
    // pinned by dds_bridge's `relative_msg_dirs_resolve_against_the_config_file_dir`).
    let resolved = config_path
        .parent()
        .expect("config path has a parent")
        .join("../schemas");
    let store = cerulion_cli_engine::schema_store::SchemaStore::load(&resolved);
    assert!(
        store.resolves("acme_msgs/Widget"),
        "the msg_dirs target, resolved config-relative, really holds the type"
    );
}

/// Mirror-duty (engine side): a type resolvable via the workspace-store attach
/// PREDICATE (`AttachSchemaChain::resolves` — what attach gates on) is ALSO
/// codec-loadable via the SAME `parse_rosmsg` + `CdrCodec::new` core the bridge's
/// `bridge_codec_with_store` wraps — over ONE tempdir store. The bridge-side
/// wrapper is pinned by the examples/go2 `store_only_type_is_known_by_bridge_codec`
/// test; together they close the cross-crate agreement.
#[test]
fn test_mirror_duty_engine_predicate_and_codec_gate_agree_on_store() {
    let ws = tempfile::tempdir().unwrap();
    write_store_msg(ws.path(), "acme_msgs", "Widget", WIDGET_MSG);
    let store_dir = ws.path().join("schemas");

    // Attach predicate: the store resolves the type (not a built-in).
    let chain = AttachSchemaChain::from_workspace(ws.path());

    // Bridge gate (same core): a CdrCodec over the SAME store dir knows it.
    let store = cerulion_cli_engine::schema_store::SchemaStore::load(&store_dir);
    let schemas: Vec<_> = store.iter().map(|(_, s)| s.schema.clone()).collect();
    let (codec, _warnings) = cerulion_core::codegen::CdrCodec::new(schemas);

    // Agreement over a store-present type and a truly-absent one (no built-in
    // dimension — keeps the two sides exactly comparable).
    for (ty, expected) in [("acme_msgs/Widget", true), ("acme_msgs/Ghost", false)] {
        assert_eq!(chain.resolves(ty), expected, "attach predicate for {ty}");
        assert_eq!(codec.knows(ty), expected, "bridge codec gate for {ty}");
    }
}

/// The `store_non_empty()` OR-arm of msg_dirs
/// emission. A RE-ATTACH over a workspace whose store ALREADY holds the type
/// acquires NOTHING this run (empty `to_write`), so ONLY `store_non_empty()` can
/// emit `msg_dirs:`. Pins that the written config STILL carries it (a mutation
/// dropping the OR-arm ships a config WITHOUT `msg_dirs:` → `graph run` dies at
/// `UnknownRosType` AFTER consent — the exact post-consent class this arm
/// exists to kill), AND that it is BYTE-IDENTICAL to the acquisition-run config
/// (the re-attach idempotence pin).
#[test]
fn test_reattach_over_prepopulated_store_still_writes_msg_dirs() {
    use cerulion_dds::NoopAcquirer;

    // Run 1 (acquisition): a real LocalAmentAcquirer materializes Widget into the
    // store and writes a config carrying msg_dirs (via the `to_write` arm).
    let install = tempfile::tempdir().unwrap();
    write_ament_msg(install.path(), "acme_msgs", "Widget", WIDGET_MSG, true);
    let acquirer = LocalAmentAcquirer::new(vec![install.path().to_path_buf()]);
    let ws1 = tempfile::tempdir().unwrap();
    let mut confirm = panic_confirm;
    ros_cmd::ros_attach_with_acquirer(
        &widget_writer(),
        &acquirer,
        ws1.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("run 1: acquisition write");
    let config1 = read(&ws1.path().join("graphs").join("attach.bridge.yaml"));

    // Run 2 (re-attach): a DIFFERENT workspace whose store is PRE-SEEDED with
    // Widget, attached with the NoopAcquirer (nothing acquired → to_write empty),
    // so the OR-arm is `store_non_empty()` ALONE.
    let ws2 = tempfile::tempdir().unwrap();
    write_store_msg(ws2.path(), "acme_msgs", "Widget", WIDGET_MSG);
    // Sanity: the pre-seeded store resolves the type with NO acquirer.
    assert!(AttachSchemaChain::from_workspace(ws2.path()).resolves("acme_msgs/Widget"));
    let report = ros_cmd::ros_attach_with_acquirer(
        &widget_writer(),
        &NoopAcquirer,
        ws2.path(),
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("run 2: re-attach write");
    // This run acquired NOTHING — the OR-arm cannot be the `to_write` arm.
    match &report.outcome {
        AttachOutcome::Written { schema_writes, .. } => assert!(
            schema_writes.is_empty(),
            "a re-attach over a pre-seeded store acquires nothing: {schema_writes:?}"
        ),
        other => panic!("expected Written, got {other:?}"),
    }
    let config2 = read(&ws2.path().join("graphs").join("attach.bridge.yaml"));

    // The re-attach config STILL carries msg_dirs (the OR-arm pin).
    assert!(
        config2.contains("\nmsg_dirs:\n"),
        "re-attach config must still emit msg_dirs: {config2}"
    );
    assert!(
        config2.contains("\n  - ../schemas\n"),
        "msg_dirs must name the store dir (config-file-relative): {config2}"
    );

    // Idempotence: byte-identical to the acquisition-run config.
    assert_eq!(
        config1, config2,
        "the re-attach config must be byte-identical to the acquisition config"
    );
}

/// A RETARGETED pin.
/// `AttachAcquirers::from_env` — the wire-less constructor this test used to
/// pin — was DELETED so `production()` is the only wire-bearing composition
/// root a main.rs edit can reach. The surviving pin is the same regression
/// class on the surviving root: `production` composes the FULL two-rung ladder
/// (wire + env-derived local ament) — a regression dropping either rung would
/// leave attach unable to acquire schemas from that source. Shape-only (no env
/// mutation — parallel-safe); ladder ORDER is pinned by
/// `ros_attach_test::test_production_acquirers_prepend_wire_rung`, the
/// behavioral local rung by `test_attach_acquirers_first_rung_is_local_ament`.
#[test]
fn test_production_composes_two_rungs() {
    let acquirers = AttachAcquirers::production(Box::new(cerulion_dds::NoopAcquirer));
    assert_eq!(
        acquirers.rungs().len(),
        2,
        "production composes wire + the env-derived local ament rung"
    );
}
