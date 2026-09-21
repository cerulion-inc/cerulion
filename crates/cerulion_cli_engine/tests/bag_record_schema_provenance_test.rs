// SPDX-License-Identifier: AGPL-3.0-only
//! `bag record` stamps a recording's SCHEMA PROVENANCE, end to end over
//! the production `cerulion_bagd` recorder and REAL iceoryx2.
//!
//! The defect this pins: an attach-mode tap sees a wire frame, which carries a
//! `schema_hash` and NO name, so without provenance every channel `bag record` writes is named
//! `"unknown"` and the bag carries no schema text at all. Played on a desk that
//! never compiled the type, the hash resolves to nothing, `walk_by_hash` refuses
//! at the hash gate, and the viewer renders an empty scene while reporting
//! success.
//!
//! The oracles are HAND-WRITTEN and never compare the pipeline to itself:
//!
//! - the type name (`acme/Widget`) is one the test chose and wrote to disk; the
//!   recorder can only produce it by resolving the wire hash through the
//!   workspace store, so a broken join yields `"unknown"`, not a near miss;
//! - the doc text is compared BYTE-for-byte against the exact strings written to
//!   the `.msg` files, so a re-serialised or normalised copy fails;
//! - the wire hash the publisher stamps is computed independently in this file
//!   from the `.msg` text via the codegen recipe — the same value a real
//!   generated type stamps — rather than being read back out of the recorder;
//! - and the ANTI-TAUTOLOGY control records the SAME topic with no workspace,
//!   which must still say `"unknown"` and write no attachment. Without it every
//!   assertion below would pass a recorder that hard-coded the answer.
//!
//! Parallel-safe: per-test SHM roots + per-test topic names, no `#[serial]`.

#![cfg(unix)]

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_cli_engine::bag_cmd::{self, RecordOptions};
use cerulion_core::codegen::{parse_rosmsg, resolve_fixed_nested, MessageSchema};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};

/// A vendor type no desk compiles: nests a CUSTOM (`acme/Sub`) and a BUILT-IN
/// (`std_msgs/Header`). The built-in is what proves the omission rule — the bag
/// must ship `acme/Sub`'s text and NOT `std_msgs/Header`'s.
const WIDGET_MSG: &str = "# fixture: a type the desk never compiled\n\
                          std_msgs/Header header\n\
                          acme/Sub sub\n\
                          int32 id\n";
const SUB_MSG: &str = "# the nested custom the closure must pull in\nfloat64 value\n";

fn unique() -> u64 {
    static N: AtomicU64 = AtomicU64::new(0);
    (std::process::id() as u64) << 20 | N.fetch_add(1, Ordering::Relaxed)
}

fn manager(tag: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("{tag}_{}", unique()),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test")
}

/// Scaffold `<ws>/schemas/acme/msg/{Widget,Sub}.msg`: the `.msg` store the
/// schema-resolution ladder materialises acquired vendor types into, and the
/// store `bag record` resolves against.
fn workspace_with_vendor_schemas(root: &std::path::Path) -> std::path::PathBuf {
    let msg_dir = root.join("schemas").join("acme").join("msg");
    std::fs::create_dir_all(&msg_dir).expect("create the .msg store");
    std::fs::write(msg_dir.join("Widget.msg"), WIDGET_MSG).expect("write Widget.msg");
    std::fs::write(msg_dir.join("Sub.msg"), SUB_MSG).expect("write Sub.msg");
    root.join("schemas")
}

/// The wire `schema_hash` a real `acme/Widget` publisher stamps.
///
/// Computed here from the `.msg` TEXT through the canonical codegen recipe
/// (`parse_rosmsg` → `resolve_fixed_nested` over the full corpus →
/// `MessageSchema::schema_hash`) — the same path a generated type's
/// `SCHEMA_HASH` const comes from. It is deliberately NOT read back out of the
/// recorder or out of `build_schema_docs`: this value is the INPUT the publisher
/// puts on the wire, and the recorder's job is to resolve it.
fn widget_wire_hash() -> u64 {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    schemas.push(parse_rosmsg(WIDGET_MSG, "Widget", Some("acme")).expect("parse Widget"));
    schemas.push(parse_rosmsg(SUB_MSG, "Sub", Some("acme")).expect("parse Sub"));
    let _ = resolve_fixed_nested(&mut schemas);
    schemas
        .iter()
        .find(|s| s.qualified_name() == "acme/Widget")
        .expect("Widget is in the set")
        .schema_hash()
}

/// The ordinary frame payload — small, because these tests are about the schema
/// ATTACHMENT and not about bytes.
const PAYLOAD_LEN: usize = 16;

/// The payload the ROTATION test uses instead.
///
/// Rotation must be driven by DATA VOLUME, not by wall-clock luck. At
/// [`PAYLOAD_LEN`] a frame is 48 bytes, so crossing the 4 KiB cap ONCE takes ~85
/// frames — and the producer only manages that inside an 800 ms window if it
/// really achieves its nominal ~200 Hz. On a loaded runner it does not: too
/// few frames arrive to roll even once, and the test fails at
/// its own `files.len() > 1` precondition with a single file. (It is a
/// PRECONDITION rather than a wrong answer — the test refuses to pass vacuously
/// — but a precondition that a busy runner can miss makes the test unreliable.)
///
/// At ~1 KiB a frame, four or five frames fill a file, so the same window rolls
/// several times even when the producer is starved to a fraction of its rate.
/// Bounded by the publisher's `MaxSliceLen::const_new(1024)`, hence 960 + a
/// 32-byte header = 992.
const ROTATION_PAYLOAD_LEN: usize = 960;

fn frame_sized(hash: u64, seq: u32, payload_len: usize) -> Vec<u8> {
    let payload: Vec<u8> = (0..payload_len)
        .map(|i| (i as u8).wrapping_add(seq as u8))
        .collect();
    let total = WireHeader::SIZE + payload.len();
    let header = WireHeader {
        schema_hash: hash,
        total_size: total as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: 1_000_000_000 + u64::from(seq) * 10_000_000,
    };
    let mut buf = vec![0u8; total];
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    buf[WireHeader::SIZE..].copy_from_slice(&payload);
    buf
}

/// Record `topic` (carrying `hash`-stamped frames) for a short window, with or
/// without a workspace, and return the finalized bag path.
fn record_one_topic(
    out: std::path::PathBuf,
    topic: String,
    hash: u64,
    schemas_dir: Option<std::path::PathBuf>,
) -> std::path::PathBuf {
    record_one_topic_capped(out, topic, hash, schemas_dir, None).0
}

/// [`record_one_topic`] with an optional size cap, returning every file the
/// recorder produced (rotation appends `.1`, `.2`, …).
fn record_one_topic_capped(
    out: std::path::PathBuf,
    topic: String,
    hash: u64,
    schemas_dir: Option<std::path::PathBuf>,
    max_bag_size: Option<u64>,
) -> (std::path::PathBuf, Vec<std::path::PathBuf>) {
    record_one_topic_full(out, topic, hash, schemas_dir, max_bag_size, false)
}

/// [`record_one_topic_capped`], with control over WHICH writer path bagd takes.
///
/// bagd has two, and they create bag files through DIFFERENT code: the INLINE
/// writer (`Recorder::open_file`, used when a tap is not recording-provisioned)
/// and the dedicated WRITER THREAD (`WriterCore::create` / `WriterCore::rotate`,
/// used when every tap's `subscriber_max_borrowed_samples` reaches
/// `RECORDING_SUBSCRIBER_MAX_BORROWED`). Both must stamp the provenance, and a
/// test that only ever drives one leaves the other's four call sites unpinned —
/// with a single-path test, emptying the catalog on the
/// THREADED rotation path leaves the whole file green.
///
/// `recording_provisioned` provisions the topic's service at the borrow the
/// threaded path requires, which is how a real recording-provisioned graph topic
/// is set up.
fn record_one_topic_full(
    out: std::path::PathBuf,
    topic: String,
    hash: u64,
    schemas_dir: Option<std::path::PathBuf>,
    max_bag_size: Option<u64>,
    recording_provisioned: bool,
) -> (std::path::PathBuf, Vec<std::path::PathBuf>) {
    let robot = manager("rec");
    let mut publisher = if recording_provisioned {
        let mut cfg = robot.default_topic_config();
        cfg.subscriber_max_borrowed_samples =
            Some(cerulion_core::transport::RECORDING_SUBSCRIBER_MAX_BORROWED);
        robot
            .create_publisher_with_topic_config(&topic, MaxSliceLen::const_new(1024), 0, cfg)
            .expect("recording-provisioned publisher")
    } else {
        robot
            .create_publisher(&topic, MaxSliceLen::const_new(1024), 0)
            .expect("publisher")
    };

    // A CAPPED run is a rotation run, and rotation must be reached by DATA
    // VOLUME rather than by the producer hitting its nominal rate — see
    // `ROTATION_PAYLOAD_LEN`. Only the rotation test passes a cap, so this keys
    // the bigger frame on exactly the condition that needs it and leaves every
    // other test's bytes unchanged.
    let payload_len = if max_bag_size.is_some() {
        ROTATION_PAYLOAD_LEN
    } else {
        PAYLOAD_LEN
    };

    // A data-only tap gets no late-joiner history, so the producer must run
    // THROUGH the recording window — the taps arm inside `bag_record`.
    let publishing = Arc::new(AtomicBool::new(true));
    let flag = Arc::clone(&publishing);
    let producer = std::thread::spawn(move || {
        let mut seq = 0u32;
        while flag.load(Ordering::Relaxed) && seq < 400 {
            let _ = publisher.publish_raw(&frame_sized(hash, seq, payload_len));
            seq += 1;
            std::thread::sleep(Duration::from_millis(5));
        }
    });

    let summary = bag_cmd::bag_record_with_manager(
        robot,
        RecordOptions {
            topics: vec![topic],
            out: out.clone(),
            duration: Some(Duration::from_millis(800)),
            schema_wait: Duration::from_millis(400),
            schemas_dir,
            max_bag_size,
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut Vec::new(),
    )
    .expect("record");
    publishing.store(false, Ordering::Relaxed);
    producer.join().expect("producer thread");
    assert!(
        summary.messages > 0,
        "the recorder captured nothing — the provenance assertions would be vacuous"
    );
    (out, summary.bag_paths)
}

/// THE headline: a recording of a vendor-typed topic NAMES its channel and
/// carries the type's verbatim text plus its custom closure.
#[test]
fn a_vendor_typed_recording_names_its_channel_and_carries_the_schema_text() {
    let id = unique();
    let topic = format!("/widget/{id}");
    let hash = widget_wire_hash();
    let ws = tempfile::tempdir().expect("tempdir");
    let schemas_dir = workspace_with_vendor_schemas(ws.path());
    let out = ws.path().join("provenance.mcap");

    record_one_topic(out.clone(), topic.clone(), hash, Some(schemas_dir));

    let reader = cerulion_bag::BagReader::open(&out).expect("open");
    let channel = reader
        .channels()
        .expect("channels")
        .into_iter()
        .find(|c| c.topic == topic)
        .expect("the recorded channel");

    // (1) The wire hash is recorded exactly — that is what makes the bag playable.
    assert_eq!(
        channel.descriptor.expect("cerulion descriptor").schema_hash,
        hash
    );
    // (2) THE FIX: the channel is NAMED. The wire carried no name; the recorder
    //     resolved the hash through the workspace store to get it.
    assert_eq!(
        channel.schema_name, "acme/Widget",
        "an attach-mode channel whose hash this machine can resolve must record \
         the REAL type name, not the \"unknown\" placeholder"
    );

    let catalog = reader
        .schema_catalog()
        .expect("the bag must carry its schema provenance");

    // (3) The TEXT is there, byte-for-byte as written to disk — a desk decodes
    //     from these bytes, so a normalised or re-serialised copy is a defect.
    let widget = catalog
        .docs
        .iter()
        .find(|d| d.qualified == "acme/Widget")
        .expect("the recorded type's own doc");
    assert_eq!(widget.text, WIDGET_MSG);

    // (4) The CLOSURE came with it — the nested custom is not a channel of the
    //     bag, so nothing but the closure walk could have pulled it in.
    let sub = catalog
        .docs
        .iter()
        .find(|d| d.qualified == "acme/Sub")
        .expect("the nested custom type must ride along");
    assert_eq!(sub.text, SUB_MSG);
    assert!(
        widget.deps.iter().any(|d| d == "acme/Sub"),
        "the root doc must name its custom dep so the walk is closure-complete"
    );

    // (5) The BUILT-IN it nests is deliberately NOT shipped — every reader
    //     compiled `std_msgs/Header` in, so shipping it would be dead weight.
    assert!(
        !catalog
            .docs
            .iter()
            .any(|d| d.qualified.starts_with("std_msgs/")),
        "built-in types must not be shipped; got {:?}",
        catalog
            .docs
            .iter()
            .map(|d| &d.qualified)
            .collect::<Vec<_>>()
    );

    // (6) Only the type actually recorded is bound — the workspace holds two
    //     types, and a recording of one must not claim the other's hash.
    assert_eq!(catalog.hashes.len(), 1, "one recorded channel, one binding");
    assert_eq!(catalog.name_for_hash(hash), Some("acme/Widget"));
}

/// EVERY rotation file carries the provenance, not just the first.
///
/// bagd rolls to a new file at the size cap, and each file is meant to be
/// self-contained (that is why the ring manifests and the metadata attachments
/// are re-written per file). A recording whose provenance rode only file 0 would
/// leave every later file exactly as undecodable as before — and rotation is the
/// normal state of a long recording, which is precisely when nobody re-reads
/// file 0.
#[test]
fn every_rotation_file_carries_the_provenance() {
    // BOTH writer paths. They create files through different code, and a
    // single-path test is blind: emptying the catalog on the THREADED
    // rotation path leaves every assertion of such a test green, because an
    // un-provisioned tap records INLINE.
    rotation_carries_provenance(false, "inline");
    rotation_carries_provenance(true, "threaded");
}

fn rotation_carries_provenance(recording_provisioned: bool, path_name: &str) {
    let id = unique();
    let topic = format!("/rot/{id}");
    let hash = widget_wire_hash();
    let ws = tempfile::tempdir().expect("tempdir");
    let schemas_dir = workspace_with_vendor_schemas(ws.path());
    let out = ws.path().join("rot.mcap");

    // A cap small enough that a few ~1 KiB frames fill a file — so the roll is
    // reached by BYTES, not by the producer achieving its nominal rate inside
    // the window (see `ROTATION_PAYLOAD_LEN`).
    let (_first, files) = record_one_topic_full(
        out,
        topic.clone(),
        hash,
        Some(schemas_dir),
        Some(4 * 1024),
        recording_provisioned,
    );
    assert!(
        files.len() > 1,
        "[{path_name}] PRECONDITION: the cap must have actually rotated, or this test \
         proves nothing about later files (got {files:?})"
    );

    for path in &files {
        let reader = cerulion_bag::BagReader::open(path).unwrap_or_else(|e| {
            panic!("open {}: {e}", path.display());
        });
        let channel = reader
            .channels()
            .expect("channels")
            .into_iter()
            .find(|c| c.topic == topic)
            .unwrap_or_else(|| panic!("no channel for {topic} in {}", path.display()));
        assert_eq!(
            channel.schema_name,
            "acme/Widget",
            "[{path_name}] every rotation file names its channels: {}",
            path.display()
        );
        let catalog = reader.schema_catalog().unwrap_or_else(|| {
            panic!(
                "[{path_name}] every rotation file must be self-describing; {} carries \
                 no provenance",
                path.display()
            )
        });
        assert!(
            catalog.docs.iter().any(|d| d.qualified == "acme/Widget"),
            "[{path_name}] the definition must ride EVERY file: {}",
            path.display()
        );
        assert_eq!(catalog.name_for_hash(hash), Some("acme/Widget"));
    }
}

/// THE CLOSED LOOP: a bag recorded where the type IS known, then read where it
/// is NOT, resolves anyway — off the bag's own records.
///
/// This is the whole point of bag-carried schemas in one test. `bag info` is given
/// `schemas_dir: None`, i.e. the brand-new-laptop case: nothing but the built-in
/// corpus. The row must still name `acme/Widget` and the banner must say the
/// definition came from the bag — where before it said the hash resolved to
/// nothing and a viewer would render nothing.
#[test]
fn a_bag_recorded_where_the_type_is_known_reads_on_a_machine_where_it_is_not() {
    let id = unique();
    let topic = format!("/loop/{id}");
    let hash = widget_wire_hash();
    let ws = tempfile::tempdir().expect("tempdir");
    let schemas_dir = workspace_with_vendor_schemas(ws.path());
    let out = ws.path().join("loop.mcap");

    record_one_topic(out.clone(), topic.clone(), hash, Some(schemas_dir));

    // Read it as a machine with NO workspace: built-ins only.
    let report = bag_cmd::bag_info(&out, None).expect("bag info");
    assert!(
        report.contains("acme/Widget"),
        "the vendor type must be named on a machine that never compiled it:\n{report}"
    );
    assert!(
        report.contains("from the bag's own schema records"),
        "the row must say WHERE the definition came from:\n{report}"
    );
    assert!(
        report.contains("the BAG carries its definition"),
        "the banner must report the type as renderable, not as a dead end:\n{report}"
    );
    assert!(
        !report.contains("resolves to NOTHING"),
        "this is exactly the claim bag-carried schemas make false:\n{report}"
    );
}

/// ANTI-TAUTOLOGY: the identical recording of the identical VENDOR hash, with no
/// workspace, must degrade without fabricating — `"unknown"` and no attachment — proving
/// the workspace store is what produced the name and the text above, and not
/// something hard-coded into the recorder.
///
/// Note the scope precisely: this is NOT "a workspace-less recording never
/// records provenance". Built-in bindings are resolved unconditionally (see
/// `an_all_builtin_recording_is_named_but_ships_no_text`), because a wire frame
/// carries no name and `"unknown"` is what every standard MCAP reader would
/// otherwise display. What a workspace-less machine cannot do is resolve a
/// CUSTOM type — and this pins that it then says so instead of guessing.
#[test]
fn the_same_recording_without_a_workspace_stays_honestly_unknown() {
    let id = unique();
    let topic = format!("/nows/{id}");
    let hash = widget_wire_hash();
    let dir = tempfile::tempdir().expect("tempdir");
    let out = dir.path().join("bare.mcap");

    record_one_topic(out.clone(), topic.clone(), hash, None);

    let reader = cerulion_bag::BagReader::open(&out).expect("open");
    let channel = reader
        .channels()
        .expect("channels")
        .into_iter()
        .find(|c| c.topic == topic)
        .expect("the recorded channel");
    assert_eq!(
        channel.descriptor.expect("cerulion descriptor").schema_hash,
        hash,
        "the hash is on the wire, so it is recorded either way"
    );
    assert_eq!(
        channel.schema_name, "unknown",
        "with nothing to resolve against, the recorder must say so rather than guess"
    );
    assert!(
        reader.schema_catalog().is_none(),
        "nothing about this recording resolved, so there is no attachment at all — \
         the pruned closure of an unresolvable hash is empty, and an empty catalog \
         writes no bytes"
    );
}

/// A recording whose types are all BUILT-IN gains no attachment: every reader
/// already compiled them, so there is nothing to say. The channel is still
/// NAMED, though — the binding costs nothing and is what turns an `"unknown"`
/// row into a readable one.
#[test]
fn an_all_builtin_recording_is_named_but_ships_no_text() {
    let id = unique();
    let topic = format!("/builtin/{id}");
    let hash = <native_ros2_messages::geometry_msgs::Vector3 as cerulion_core::message::ShmMessage>::SCHEMA_HASH;
    let ws = tempfile::tempdir().expect("tempdir");
    let schemas_dir = workspace_with_vendor_schemas(ws.path());
    let out = ws.path().join("builtin.mcap");

    record_one_topic(out.clone(), topic.clone(), hash, Some(schemas_dir));

    let reader = cerulion_bag::BagReader::open(&out).expect("open");
    let channel = reader
        .channels()
        .expect("channels")
        .into_iter()
        .find(|c| c.topic == topic)
        .expect("the recorded channel");
    assert_eq!(
        channel.schema_name, "geometry_msgs/Vector3",
        "a built-in hash resolves to its real name too"
    );
    let catalog = reader
        .schema_catalog()
        .expect("the binding alone is still provenance worth recording");
    assert!(
        catalog.docs.is_empty(),
        "a built-in's text is dead weight in the bag; got {:?}",
        catalog
            .docs
            .iter()
            .map(|d| &d.qualified)
            .collect::<Vec<_>>()
    );
    assert_eq!(catalog.name_for_hash(hash), Some("geometry_msgs/Vector3"));
}
