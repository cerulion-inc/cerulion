// SPDX-License-Identifier: AGPL-3.0-only
//! The per-channel PROVISIONING metadata, end to end through a
//! REAL bag file, read back BOTH by our `BagReader` and by the independent
//! `mcap` crate.
//!
//! Two properties, and the second is the one that makes the extension safe to
//! ship:
//!
//! 1. **It round-trips.** A declared tuple comes back field for field, against a
//!    hand-written oracle (each field a DISTINCT value, so a swapped key cannot
//!    pass).
//! 2. **It is additive in BOTH directions.** A bag whose caller declares nothing
//!    is BYTE-IDENTICAL to one written by the pre-provisioning encoder, and a NEW
//!    reader on such a bag reads "not recorded" rather than a fabricated
//!    default. The byte-identity half is asserted against a hand-built oracle
//!    byte sequence for the Channel record — not against a second run of the
//!    same writer, which would be a self-compare.

use std::path::{Path, PathBuf};

use cerulion_bag::{BagReader, BagWriter, BagWriterConfig, ChannelProvisioning, TopicSchema};

fn tempdir() -> PathBuf {
    static C: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = C.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let name = format!(
        "cerulion_bag_prov_{}_{}_{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos(),
        n
    );
    let p = std::env::temp_dir().join(name);
    std::fs::create_dir_all(&p).unwrap();
    p
}

fn topics() -> Vec<TopicSchema> {
    vec![
        TopicSchema {
            topic: "/camera/image".into(),
            schema_name: "sensor_msgs/Image".into(),
            schema_hash: 0x1111_2222_3333_4444,
            wire_fixed_size: 64,
        },
        TopicSchema {
            topic: "/imu".into(),
            schema_name: "sensor_msgs/Imu".into(),
            schema_hash: 0x5555_6666_7777_8888,
            wire_fixed_size: 320,
        },
    ]
}

fn write_bag(dir: &Path, config: BagWriterConfig) -> PathBuf {
    let path = dir.join("b.mcap");
    let mut w = BagWriter::create(&path, config, &topics()).unwrap();
    w.write_message("/camera/image", 1, 10, 10, &[&[0xAAu8; 8]])
        .unwrap();
    w.write_message("/imu", 1, 11, 11, &[&[0xBBu8; 8]]).unwrap();
    w.finalize().unwrap();
    path
}

/// The headline: a declared tuple survives the file and comes back exact.
#[test]
fn a_declared_provisioning_tuple_round_trips_through_a_real_bag() {
    let dir = tempdir();
    let mut config = BagWriterConfig::default();
    // Hand oracle: DISTINCT values per field and per topic, so neither a key
    // swap nor a topic swap can pass.
    config.provisioning.insert(
        "/camera/image".to_string(),
        ChannelProvisioning {
            buffer_depth: Some(4096),
            max_slice_len: Some(2_097_152),
            history_depth: Some(3),
            latched: Some(false),
        },
    );
    config.provisioning.insert(
        "/imu".to_string(),
        ChannelProvisioning {
            buffer_depth: Some(16),
            max_slice_len: Some(320),
            history_depth: Some(1),
            latched: Some(true),
        },
    );
    let path = write_bag(&dir, config);

    let reader = BagReader::open(&path).unwrap();
    let channels = reader.channels().unwrap();

    let cam = channels
        .iter()
        .find(|c| c.topic == "/camera/image")
        .expect("the camera channel must exist");
    assert_eq!(
        cam.provisioning,
        ChannelProvisioning {
            buffer_depth: Some(4096),
            max_slice_len: Some(2_097_152),
            history_depth: Some(3),
            latched: Some(false),
        }
    );

    let imu = channels
        .iter()
        .find(|c| c.topic == "/imu")
        .expect("the imu channel must exist");
    assert_eq!(
        imu.provisioning,
        ChannelProvisioning {
            buffer_depth: Some(16),
            max_slice_len: Some(320),
            history_depth: Some(1),
            latched: Some(true),
        }
    );

    // The RESERVED channels declare nothing — they have no producer to describe.
    for c in &channels {
        if c.topic.starts_with("__cerulion/") {
            assert!(
                c.provisioning.is_empty(),
                "reserved channel {} must carry no provisioning, got {:?}",
                c.topic,
                c.provisioning
            );
        }
    }

    // The INDEPENDENT oracle: the upstream `mcap` crate must see the same map,
    // which is what proves we emitted a standard MCAP metadata map and not a
    // private encoding our own reader happens to agree with.
    let bytes = std::fs::read(&path).unwrap();
    let summary = mcap::Summary::read(&bytes).unwrap().unwrap();
    let cam_meta = summary
        .channels
        .values()
        .find(|c| c.topic == "/camera/image")
        .expect("mcap must see the camera channel")
        .metadata
        .clone();
    assert_eq!(
        cam_meta.get("cerulion.max_slice_len").map(String::as_str),
        Some("2097152")
    );
    assert_eq!(
        cam_meta.get("cerulion.latched").map(String::as_str),
        Some("false")
    );
    assert_eq!(cam_meta.len(), 4, "four declared fields, four keys");
}

/// ADDITIVE IN BOTH DIRECTIONS — the property that makes this shippable.
///
/// A caller that declares NOTHING must produce the byte sequence the
/// pre-provisioning encoder produced. Asserted on the FILE BYTES against a
/// hand-assembled Channel record, so it cannot be satisfied by two runs of the
/// same (wrong) writer agreeing with each other.
#[test]
fn a_bag_that_declares_no_provisioning_is_byte_identical_to_a_pre_981_bag() {
    let dir = tempdir();
    let path = write_bag(&dir, BagWriterConfig::default());
    let bytes = std::fs::read(&path).unwrap();

    // Hand-assemble the Channel record the OLDER (pre-provisioning) encoder wrote for
    // channel 0 (`/camera/image`, sorted first): opcode 0x04, u64 content
    // length, id u16, schema_id u16, topic string, message_encoding string,
    // then a u32 ZERO for the empty metadata map.
    let topic = "/camera/image";
    let enc = "cerulion";
    let content_len: u64 = (2 + 2 + 4 + topic.len() + 4 + enc.len() + 4) as u64;
    let mut oracle: Vec<u8> = Vec::new();
    oracle.push(0x04); // op::CHANNEL (MCAP: Header 1, Footer 2, Schema 3, Channel 4)
    oracle.extend_from_slice(&content_len.to_le_bytes());
    // Channel ids are assigned by SORTED TOPIC name (user topics first), so
    // `/camera/image` is 0; schema ids by SORTED SCHEMA IDENTITY starting at 1,
    // which puts `sensor_msgs/Image` FIFTH behind the FOUR reserved schemas
    // (`cerulion.FrameProducers`, `cerulion.NonDeterminism`,
    // `cerulion.SchedulerTrace`, `cerulion.State`).
    //
    // The state channel moved it from 3 to 4 and the producer-label channel
    // moved it from 4 to 5, each by registering another reserved channel. That
    // is a real, deliberate change to the byte layout of every bag (the
    // reserved channels are registered unconditionally, so the id assignment
    // stays a function of the registration alone) and it is invisible to every
    // reader, which resolves schemas through the Channel record's `schema_id`
    // and never through a literal.
    oracle.extend_from_slice(&0u16.to_le_bytes());
    oracle.extend_from_slice(&5u16.to_le_bytes());
    oracle.extend_from_slice(&(topic.len() as u32).to_le_bytes());
    oracle.extend_from_slice(topic.as_bytes());
    oracle.extend_from_slice(&(enc.len() as u32).to_le_bytes());
    oracle.extend_from_slice(enc.as_bytes());
    oracle.extend_from_slice(&0u32.to_le_bytes()); // EMPTY metadata map

    assert!(
        bytes.windows(oracle.len()).any(|w| w == oracle.as_slice()),
        "a no-provisioning bag must carry the pre-provisioning Channel record byte for byte \
         (empty metadata map); the hand-assembled oracle was not found in the file"
    );

    // And a NEW reader on it reads NOT RECORDED, never a fabricated default.
    let reader = BagReader::open(&path).unwrap();
    for c in reader.channels().unwrap() {
        assert_eq!(
            c.provisioning,
            ChannelProvisioning::default(),
            "channel {} must read as not-recorded",
            c.topic
        );
        assert!(c.provisioning.is_empty());
    }
}

/// A partially-declared tuple carries only what it knows — the absent fields
/// must not become zeros.
#[test]
fn a_partially_declared_channel_reads_back_its_absent_fields_as_not_recorded() {
    let dir = tempdir();
    let mut config = BagWriterConfig::default();
    config.provisioning.insert(
        "/imu".to_string(),
        ChannelProvisioning {
            max_slice_len: Some(321),
            ..Default::default()
        },
    );
    let path = write_bag(&dir, config);

    let reader = BagReader::open(&path).unwrap();
    let channels = reader.channels().unwrap();
    let imu = channels.iter().find(|c| c.topic == "/imu").unwrap();
    assert_eq!(imu.provisioning.max_slice_len, Some(321));
    assert_eq!(
        imu.provisioning.buffer_depth, None,
        "an undeclared field must read as NOT RECORDED, never 0"
    );
    assert_eq!(imu.provisioning.history_depth, None);
    assert_eq!(imu.provisioning.latched, None);

    // The sibling topic was never mentioned at all.
    let cam = channels
        .iter()
        .find(|c| c.topic == "/camera/image")
        .unwrap();
    assert!(cam.provisioning.is_empty());
}

/// A provisioning key in the reserved `__cerulion/`
/// namespace is REFUSED, and a normal key on the same config still stamps.
///
/// The lookup that builds each channel's metadata runs over EVERY
/// registration — the auto-registered reserved channels included — while
/// the pre-existing reserved-prefix guard walks only the caller's `topics`
/// slice. So a reserved key was stamped onto a reserved Channel record,
/// contradicting `ChannelInfo::metadata`'s "EMPTY for every reserved channel"
/// and breaking those channels' byte-identity with older bags.
///
/// Fail-closed rather than trusted: no shipping caller populates this map
/// today (`cerulion_bagd` leaves it at `Default`), so nothing REACHES the
/// defect. The play-fidelity writer is the named future caller, and a guard is
/// what makes that arrival safe rather than a review it has to pass.
///
/// The ANTI-TAUTOLOGY half is in this body deliberately: without it, a
/// variant that blanket-refuses any provisioning would pass every assertion
/// above. The boundary is pinned too — `__cerulion_not_a_namespace` does NOT
/// start with `__cerulion/` and must be ACCEPTED, so the guard is the prefix
/// rule rather than a substring sweep.
///
/// BOTH constructors are driven: `create` and `with_sink` are separate public
/// entry points, and a guard on one is not a guard on the other.
#[test]
fn a_reserved_prefix_provisioning_key_is_refused_while_a_normal_key_still_stamps() {
    use cerulion_bag::{BagError, FileSink, NONDETERMINISM_TOPIC, SCHEDULER_TRACE_TOPIC};

    let dir = tempdir();

    for reserved in [SCHEDULER_TRACE_TOPIC, NONDETERMINISM_TOPIC] {
        let mut config = BagWriterConfig::default();
        config.provisioning.insert(
            reserved.to_string(),
            ChannelProvisioning {
                buffer_depth: Some(7),
                ..Default::default()
            },
        );

        // `create` — the production path. It validates BEFORE opening the
        // file, so a refused config must also leave no stray bag behind.
        let path = dir.join(format!("refused_{}.mcap", reserved.replace('/', "_")));
        let err = match BagWriter::create(&path, config.clone(), &topics()) {
            Ok(_) => panic!("a reserved provisioning key ({reserved}) must not be accepted"),
            Err(e) => e,
        };
        assert!(
            matches!(&err, BagError::ReservedProvisioningKey { topic } if topic == reserved),
            "a reserved provisioning key must be refused by its own variant, got {err:?}"
        );
        assert!(
            !path.exists(),
            "a refused config must not leave a stray bag file behind"
        );

        // `with_sink` — the sink-injection entry point, guarded separately.
        let sink_path = dir.join(format!("sink_{}.mcap", reserved.replace('/', "_")));
        let sink = FileSink::create(&sink_path).unwrap();
        let err = match BagWriter::with_sink(sink, config, &topics()) {
            Ok(_) => panic!("the sink entry point must refuse {reserved} too"),
            Err(e) => e,
        };
        assert!(
            matches!(&err, BagError::ReservedProvisioningKey { topic } if topic == reserved),
            "the sink entry point must carry the same guard, got {err:?}"
        );
    }

    // ANTI-TAUTOLOGY: a normal key on the same shape still round-trips, so the
    // guard rejects reserved keys rather than provisioning as such.
    let mut ok = BagWriterConfig::default();
    ok.provisioning.insert(
        "/imu".to_string(),
        ChannelProvisioning {
            buffer_depth: Some(7),
            ..Default::default()
        },
    );
    // BOUNDARY: adjacent to the prefix but not in it — `__cerulion_x` has no
    // `/`, so it is an ordinary (if odd) user-namespace key.
    ok.provisioning.insert(
        "__cerulion_not_a_namespace".to_string(),
        ChannelProvisioning {
            latched: Some(true),
            ..Default::default()
        },
    );
    let path = write_bag(&dir, ok);
    let reader = BagReader::open(&path).unwrap();
    let channels = reader.channels().unwrap();
    let imu = channels.iter().find(|c| c.topic == "/imu").unwrap();
    assert_eq!(
        imu.provisioning.buffer_depth,
        Some(7),
        "a normal provisioning key must still stamp"
    );

    // And the reserved channels really are the ones that were at risk: they
    // exist in every bag, and carry nothing.
    for reserved in [SCHEDULER_TRACE_TOPIC, NONDETERMINISM_TOPIC] {
        let ch = channels
            .iter()
            .find(|c| c.topic == reserved)
            .unwrap_or_else(|| panic!("{reserved} must be auto-registered"));
        assert!(
            ch.provisioning.is_empty(),
            "{reserved} must carry NO provisioning, got {:?}",
            ch.provisioning
        );
    }
}
