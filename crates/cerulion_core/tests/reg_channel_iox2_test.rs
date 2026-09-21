// SPDX-License-Identifier: AGPL-3.0-only
//! The cross-process runtime-egress REGISTRATION CHANNEL, e2e.
//!
//! A network-free WORKER `TransportManager` (`W`) — a `ros2 attach` robot's
//! dds_bridge process, deliberately with NO zenoh session — calls
//! [`TransportManager::register_dynamic_egress_topic`] for each runtime raw
//! route; the record crosses the reserved `/__cerulion/gateway_topics` SHM
//! control service to a separate GATEWAY (`G`) whose `drive_once` drains it and
//! feeds every record through `register_runtime_topic`, so the route
//! becomes announced + demand-grantable.
//!
//! # Test shape
//!
//! `W` and `G` share ONE per-test SHM root (two in-process managers modelling
//! two processes on one machine — crib `gateway_iox2_test`). Delivery is driven
//! DETERMINISTICALLY: the worker's republish is fired inline via the sync seam
//! ([`TransportManager::republish_dynamic_egress_for_test`]) and the gateway is
//! polled with [`GatewayRuntime::drive_once`], so no test depends on the
//! background republish thread's timing (extra thread republishes are harmless —
//! `register_runtime_topic` is idempotent). Hand oracles throughout — never a
//! self-compare. Per-test SHM roots + unique topics ⇒ parallel-safe, no
//! `#[serial]`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::testing::{count_at_exclusively, debug_level_compiled_in, line_level};
use cerulion_core::transport::gateway::{
    GatewayEgressPolicy, GatewayPlan, GatewayRuntime, RESERVED_TOPIC_PREFIX,
};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::reg_channel::{
    encode_record, force_control_open_collisions_for_test,
    forced_control_open_collisions_remaining_for_test, kill_pump_thread_for_test,
    panic_on_next_pump_send_for_test, panic_on_next_pump_sends_for_test,
    panic_on_next_register_send_for_test, suppressed_panic_hook_reports_for_test,
    CONTROL_OPEN_ATTEMPTS, MAX_REG_TOPIC_LEN, REGISTER_SEND_PANIC_MSG, REG_CHANNEL_MAX_WRITERS,
};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::TransportError;
use tracing_test::traced_test;

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_id() -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{nanos}_{id}")
}

/// A network-free WORKER manager on `root` (a graph/worker process — no session).
fn worker_manager(tag: &str, root: &iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("rc_w_{tag}_{}", unique_id()),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init worker manager")
}

/// A GATEWAY on `root` with an AllowAll, empty-announce plan (the runtime
/// registrations arrive over the control channel, not the boot plan).
fn gateway_on(tag: &str, root: &iceoryx2::config::Config) -> GatewayRuntime {
    let g = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("rc_g_{tag}_{}", unique_id()),
            network: Some(NetworkConfig {
                robot_identity: Some("rctest".to_string()),
                ..NetworkConfig::default()
            }),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init gateway manager");
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![],
        ingress: vec![],
    };
    GatewayRuntime::new(g, plan).expect("gateway boot")
}

/// Deterministically settle the channel: republish the worker's whole set inline,
/// then drive the gateway, `passes` times. Bounded (no thread timing). One pass
/// suffices in practice (queue depth ≫ any test burst); a few give slack for
/// in-process connection establishment.
fn settle(worker: &TransportManager, gateway: &mut GatewayRuntime, passes: usize) {
    for _ in 0..passes {
        worker.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive_once");
    }
}

/// The gateway's runtime-registered set, sorted (hand-oracle friendly).
fn registered_set(gateway: &GatewayRuntime) -> Vec<String> {
    gateway
        .runtime_registered_topics()
        .expect("runtime_registered_topics")
}

// ---------------------------------------------------------------------------
// (a) HEADLINE cross-manager e2e.
// ---------------------------------------------------------------------------

/// A network-free worker registers `/rt/cloud`; the gateway's next drive pass
/// registers + announces it and stores its advertised schema hash. Hand oracles.
#[test]
fn headline_worker_registration_reaches_gateway_announced_with_hash() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("a", &root); // gateway boots first (reader creates the service)
    let worker = worker_manager("a", &root);
    let topic = format!("/rt/cloud/{}", unique_id());
    const HASH: u64 = 0xABCD_1234_5678_9F01;

    // Not registered / not announced before anything.
    assert!(
        !gateway.is_runtime_registered(&topic).expect("pre reg"),
        "topic must not be registered before the worker registers it"
    );
    assert!(
        gateway.runtime_registration_active(),
        "reader opened at boot"
    );

    // Worker registers (network-free) → genuinely new locally.
    assert!(
        worker
            .register_dynamic_egress_topic(&topic, HASH)
            .expect("register"),
        "a fresh registration returns Ok(true)"
    );

    settle(&worker, &mut gateway, 3);

    // The gateway registered + announced it, and stored the advertised hash.
    assert_eq!(
        registered_set(&gateway),
        vec![topic.clone()],
        "the runtime topic is registered at the gateway"
    );
    assert!(gateway.is_runtime_registered(&topic).expect("is reg"));
    assert_eq!(
        gateway.runtime_topic_schema_hash(&topic),
        Some(HASH),
        "the advertised schema hash is stored observably (item 5)"
    );
    // Discovery truth: announced on the network.
    assert!(
        gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&topic),
        "the runtime topic is announced (discovery truth)"
    );
    // No malformed / no rejected on the happy path.
    assert_eq!(gateway.registration_malformed_count(), 0);
    assert_eq!(gateway.registration_rejected_count(), 0);
}

// ---------------------------------------------------------------------------
// (b) ORDERING pin — the bring-up race. Registrations pushed BEFORE the gateway
//     exists/drains must NOT be lost.
// ---------------------------------------------------------------------------

/// The worker registers N topics BEFORE the gateway process exists; once the
/// gateway boots + drains, ALL N land (exact set oracle, zero loss). This is the
/// bring-up race the periodic-republication design closes.
#[test]
fn ordering_registrations_before_gateway_boots_are_not_lost() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let worker = worker_manager("b", &root);

    // Worker registers N routes with NO gateway in existence yet.
    let base = unique_id();
    let mut want: Vec<String> = (0..8).map(|i| format!("/rt/b/{base}/route{i}")).collect();
    for (i, t) in want.iter().enumerate() {
        assert!(
            worker
                .register_dynamic_egress_topic(t, 100 + i as u64)
                .expect("register"),
            "each is a fresh registration"
        );
    }
    assert_eq!(
        worker.dynamic_egress_record_count_for_test(),
        want.len(),
        "the worker retains every registration in its authoritative set"
    );

    // NOW the gateway boots (its reader connects to the worker's live publisher)
    // and drains — every pre-boot registration must arrive.
    let mut gateway = gateway_on("b", &root);
    settle(&worker, &mut gateway, 3);

    let mut got = registered_set(&gateway);
    got.sort();
    want.sort();
    assert_eq!(got, want, "ALL pre-boot registrations landed — zero loss");
    // Each advertised hash survived too (hand oracle on the map).
    for (i, t) in want.iter().enumerate() {
        // want was sorted, so recompute the hash from the topic's suffix index.
        let idx: usize = t
            .rsplit("route")
            .next()
            .and_then(|s| s.parse().ok())
            .expect("route index");
        assert_eq!(
            gateway.runtime_topic_schema_hash(t),
            Some(100 + idx as u64),
            "hash for {t}"
        );
        let _ = i;
    }
    assert_eq!(gateway.registration_malformed_count(), 0);
    assert_eq!(gateway.registration_rejected_count(), 0);
}

// ---------------------------------------------------------------------------
// (c) BURST — the real robot's ~90-route shape all lands.
// ---------------------------------------------------------------------------

/// A ~90-registration burst (the dds_bridge shape) all lands at the gateway
/// (exact count + spot-checked names/hashes). Exercises the queue-depth headroom.
#[test]
fn burst_ninety_registrations_all_land() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("c", &root);
    let worker = worker_manager("c", &root);

    const N: usize = 90;
    let base = unique_id();
    let topics: Vec<String> = (0..N).map(|i| format!("/rt/c/{base}/t{i:03}")).collect();
    for (i, t) in topics.iter().enumerate() {
        worker
            .register_dynamic_egress_topic(t, i as u64)
            .expect("register");
    }

    settle(&worker, &mut gateway, 3);

    assert_eq!(
        gateway.runtime_registered_count().expect("count"),
        N,
        "all {N} registrations landed"
    );
    // Spot-check a few names + hashes against the oracle.
    for &i in &[0usize, 1, 45, 88, 89] {
        assert!(
            gateway.is_runtime_registered(&topics[i]).expect("is reg"),
            "topic {} registered",
            topics[i]
        );
        assert_eq!(
            gateway.runtime_topic_schema_hash(&topics[i]),
            Some(i as u64),
            "hash for {}",
            topics[i]
        );
    }
    assert_eq!(gateway.registration_malformed_count(), 0);
    assert_eq!(gateway.registration_rejected_count(), 0);
}

// ---------------------------------------------------------------------------
// (d) RESERVED-NAME refused at BOTH seams.
// ---------------------------------------------------------------------------

/// The public API rejects a reserved-namespace topic (defense in depth) — both
/// the bare `/__cerulion` token and a child. Nothing reaches the wire.
#[test]
fn reserved_name_refused_at_public_api() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let worker = worker_manager("d1", &root);

    for name in [
        RESERVED_TOPIC_PREFIX.to_string(),
        format!("{RESERVED_TOPIC_PREFIX}/evil"),
        format!("{RESERVED_TOPIC_PREFIX}/gateway_topics"),
    ] {
        let err = worker
            .register_dynamic_egress_topic(&name, 0)
            .expect_err("reserved name must be refused");
        assert!(
            matches!(err, TransportError::InvalidTransportConfig { .. }),
            "reserved-name refusal is InvalidTransportConfig, got {err:?}"
        );
        assert!(
            format!("{err}").contains(RESERVED_TOPIC_PREFIX),
            "the error names the reserved namespace"
        );
    }
    // A non-child sibling (`/__cerulionx`) is a DIFFERENT topic, NOT reserved.
    assert!(worker
        .register_dynamic_egress_topic("/__cerulionx", 0)
        .is_ok());
    // Empty topic is also refused.
    assert!(matches!(
        worker.register_dynamic_egress_topic("", 0),
        Err(TransportError::InvalidTransportConfig { .. })
    ));
}

/// A HOSTILE reserved-name record crafted on the wire (bypassing the public API
/// guard) is refused by the gateway drain — counted + the drain SURVIVES (a
/// valid record sent alongside still registers). Hand oracle. Regression guard:
/// a REFUSAL (`InvalidTransportConfig`) bumps `reg_rejected` (NOT
/// `reg_announce_deferred`) AND emits the loud "will NOT be announced or made
/// demandable" warn — the discrimination's rejection arm stays intact.
#[traced_test]
#[test]
fn reserved_name_record_on_the_wire_is_refused_by_the_drain() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("d2", &root);
    let worker = worker_manager("d2", &root);

    // A well-behaved registration + a hostile reserved-name record on the wire.
    let good = format!("/rt/d2/{}", unique_id());
    worker
        .register_dynamic_egress_topic(&good, 7)
        .expect("register good");
    let hostile =
        encode_record(&format!("{RESERVED_TOPIC_PREFIX}/evil"), 0).expect("encode hostile");

    // Send the hostile record, then settle (the worker's republish re-sends the
    // good one each pass; the hostile one is sent once here).
    assert!(
        worker
            .dynamic_egress_send_raw_for_test(&hostile)
            .expect("send raw"),
        "the raw hostile record is sent onto the control wire"
    );
    settle(&worker, &mut gateway, 3);

    // The good topic registered; the hostile one was refused + counted; the
    // drain survived (the good topic is still there).
    assert_eq!(
        registered_set(&gateway),
        vec![good.clone()],
        "only the valid topic registered — the reserved-name record was refused"
    );
    assert!(
        !gateway
            .is_runtime_registered(&format!("{RESERVED_TOPIC_PREFIX}/evil"))
            .expect("is reg"),
        "the reserved-name topic must NEVER be registered"
    );
    assert!(
        gateway.registration_rejected_count() >= 1,
        "the reserved-name record is counted as rejected"
    );
    assert_eq!(
        gateway.registration_malformed_count(),
        0,
        "a reserved-name record decodes fine — it is REJECTED, not malformed"
    );
    // A REFUSAL is NOT a deferred announce — the deferred counter stays 0.
    assert_eq!(
        gateway.registration_announce_deferred_count(),
        0,
        "a reserved-name refusal is a rejection, never a deferred announce"
    );
    // The refusal arm still emits the loud "will NOT be announced or made
    // demandable" warn (accurate for a refused topic — it is neither).
    logs_assert(|lines: &[&str]| {
        let refusal_warns = lines
            .iter()
            .filter(|l| {
                line_level(l) == Some("WARN")
                    && l.contains("will NOT be announced or made demandable")
            })
            .count();
        if refusal_warns < 1 {
            return Err(format!(
                "expected the reserved-name refusal warn, got {refusal_warns}"
            ));
        }
        Ok(())
    });
}

// ---------------------------------------------------------------------------
// (e) IDEMPOTENT re-push.
// ---------------------------------------------------------------------------

/// Registering the same topic twice (raw + canonical spellings) yields exactly
/// ONE registration at the gateway. Hand oracle.
#[test]
fn idempotent_re_push_registers_once() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("e", &root);
    let worker = worker_manager("e", &root);

    let suffix = unique_id();
    let canonical = format!("/rt/e/{suffix}");
    let raw = format!("rt/e/{suffix}"); // same canonical topic, no leading slash

    assert!(
        worker
            .register_dynamic_egress_topic(&canonical, 5)
            .expect("first"),
        "first registration is new"
    );
    assert!(
        !worker
            .register_dynamic_egress_topic(&canonical, 5)
            .expect("dup"),
        "a canonical re-register returns Ok(false)"
    );
    assert!(
        !worker
            .register_dynamic_egress_topic(&raw, 5)
            .expect("raw dup"),
        "the raw spelling of the same canonical topic is also a dup"
    );
    // The worker's authoritative set holds exactly ONE record.
    assert_eq!(worker.dynamic_egress_record_count_for_test(), 1);

    settle(&worker, &mut gateway, 3);

    assert_eq!(
        registered_set(&gateway),
        vec![canonical.clone()],
        "exactly one registration at the gateway despite repeated pushes"
    );
    assert_eq!(gateway.runtime_registered_count().expect("count"), 1);
}

/// Re-registering a topic with a DIFFERENT `schema_hash` is LAST-WRITE-WINS
/// on BOTH sides — the worker's authoritative map holds the NEW hash immediately,
/// and the gateway's topic→hash table converges to the NEW hash after the next
/// drain (the drain's Ok arm always overwrites). Exactly one registration
/// throughout. Hand oracles on the stored hash value each side.
#[test]
fn hash_conflict_is_last_write_wins_on_both_sides() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("k", &root);
    let worker = worker_manager("k", &root);
    let topic = format!("/rt/k/{}", unique_id());
    const H1: u64 = 0x1111_2222_3333_4444;
    const H2: u64 = 0xAAAA_BBBB_CCCC_DDDD;

    // Register with H1 → both sides store H1.
    assert!(
        worker
            .register_dynamic_egress_topic(&topic, H1)
            .expect("first registration"),
        "the first registration is new"
    );
    settle(&worker, &mut gateway, 3);
    assert_eq!(
        worker.dynamic_egress_record_hash_for_test(&topic),
        Some(H1),
        "worker map holds H1 after the first register"
    );
    assert_eq!(
        gateway.runtime_topic_schema_hash(&topic),
        Some(H1),
        "gateway table holds H1 after the first drain"
    );

    // Re-register the SAME topic with a DIFFERENT hash H2 (idempotent topic →
    // Ok(false)) — worker-side LWW is immediate (insert overwrites the value).
    assert!(
        !worker
            .register_dynamic_egress_topic(&topic, H2)
            .expect("re-register with a new hash"),
        "the topic is not new — only its advertised hash changed"
    );
    assert_eq!(
        worker.dynamic_egress_record_hash_for_test(&topic),
        Some(H2),
        "worker map LAST-WRITE-WINS: it now holds H2"
    );
    assert_eq!(
        worker.dynamic_egress_record_count_for_test(),
        1,
        "still exactly one authoritative record (the hash changed, not the topic)"
    );

    // Gateway-side converges to H2 on the next drain (Ok arm overwrites).
    settle(&worker, &mut gateway, 3);
    assert_eq!(
        gateway.runtime_topic_schema_hash(&topic),
        Some(H2),
        "gateway table LAST-WRITE-WINS: it updates to H2 after the next drain"
    );
    assert_eq!(
        gateway.runtime_registered_count().expect("count"),
        1,
        "exactly one registration at the gateway throughout the hash change"
    );
    assert_eq!(gateway.registration_malformed_count(), 0);
    assert_eq!(gateway.registration_rejected_count(), 0);
}

// ---------------------------------------------------------------------------
// (f) MALFORMED record — counted + warn-once + drain continues.
// ---------------------------------------------------------------------------

/// Malformed records on the wire (truncated header + bad magic) are counted and
/// dropped; the drain SURVIVES (a valid record sent alongside still registers).
/// The warn fires exactly ONCE (first malformed), repeats at debug. Hand oracle.
#[traced_test]
#[test]
fn malformed_records_counted_warn_once_and_drain_continues() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("f", &root);
    let worker = worker_manager("f", &root);

    let good = format!("/rt/f/{}", unique_id());
    worker
        .register_dynamic_egress_topic(&good, 3)
        .expect("register good");

    // Two DISTINCT malformed frames on the wire: truncated (shorter than the
    // header) + bad magic (right length, wrong opening bytes).
    assert!(worker
        .dynamic_egress_send_raw_for_test(&[0xCE, 0x21, 0x01])
        .expect("send truncated"));
    let mut bad_magic = encode_record("/whatever", 1).expect("encode");
    bad_magic[0] = 0x00; // corrupt the magic
    assert!(worker
        .dynamic_egress_send_raw_for_test(&bad_magic)
        .expect("send bad magic"));

    settle(&worker, &mut gateway, 3);

    // Both malformed records counted; the good topic still registered.
    assert_eq!(
        gateway.registration_malformed_count(),
        2,
        "both malformed records are counted"
    );
    assert_eq!(
        registered_set(&gateway),
        vec![good.clone()],
        "the drain survived the malformed records — the valid topic registered"
    );
    assert_eq!(
        gateway.registration_rejected_count(),
        0,
        "a malformed record is dropped at decode — never reaches register (not 'rejected')"
    );

    // Warn-once: exactly ONE loud malformed warn; the second malformed downgrades
    // to a debug 'repeat'.
    logs_assert(|lines: &[&str]| {
        // Level-free twin: a suppressed malformed-record repeat must never be LOUD — the half of the
        // contract that survives `release_max_level_info`, where the gated
        // DEBUG count reads 0.
        for level in ["WARN", "INFO", "ERROR"] {
            let loud = lines
                .iter()
                .filter(|l| {
                    line_level(l) == Some(level) && (l.contains("malformed control record (repeat"))
                })
                .count();
            if loud != 0 {
                return Err(format!(
                    "a suppressed malformed-record repeat was emitted at {level} ({loud} line(s))"
                ));
            }
        }
        // The loud head, matched WITH its level token AND against the level-free
        // total of the same marker: a head demoted to INFO/ERROR is not the one
        // loud warn this pins, and neither is a second copy of it at another
        // level.
        let warns =
            count_at_exclusively(lines, "WARN", &["dropped a malformed control record on"])?;
        let repeats = count_at_exclusively(lines, "DEBUG", &["malformed control record (repeat"])?;
        if warns != 1 {
            return Err(format!(
                "expected exactly 1 loud malformed warn, got {warns}"
            ));
        }
        if debug_level_compiled_in() && repeats < 1 {
            return Err(format!(
                "expected >= 1 debug 'repeat' for the 2nd+ malformed record, got {repeats}"
            ));
        }
        Ok(())
    });
}

// ---------------------------------------------------------------------------
// (g) DETERMINISM — two runs of the same scenario yield the same oracle.
// ---------------------------------------------------------------------------

/// The registered set + stored hashes are identical across two independent runs
/// of the same registration scenario (deterministic — the authoritative set is a
/// sorted map, delivery is idempotent). Not a self-compare: both runs are also
/// checked against a HAND oracle.
#[test]
fn determinism_two_runs_same_registered_set_and_hashes() {
    fn run_once(tag: &str) -> Vec<(String, Option<u64>)> {
        let root = cerulion_core::testing::iceoryx_test_config();
        let mut gateway = gateway_on(tag, &root);
        let worker = worker_manager(tag, &root);
        // A FIXED scenario (topics independent of unique_id so both runs match).
        let topics = ["/rt/g/alpha", "/rt/g/beta", "/rt/g/gamma"];
        for (i, t) in topics.iter().enumerate() {
            worker
                .register_dynamic_egress_topic(t, (i as u64 + 1) * 11)
                .expect("register");
        }
        settle(&worker, &mut gateway, 3);
        registered_set(&gateway)
            .into_iter()
            .map(|t| {
                let h = gateway.runtime_topic_schema_hash(&t);
                (t, h)
            })
            .collect()
    }

    let run1 = run_once("g1");
    let run2 = run_once("g2");
    // Hand oracle (sorted set + hashes) — anchors BOTH runs, so equality is a
    // real cross-check, not a self-compare.
    let oracle = vec![
        ("/rt/g/alpha".to_string(), Some(11)),
        ("/rt/g/beta".to_string(), Some(22)),
        ("/rt/g/gamma".to_string(), Some(33)),
    ];
    assert_eq!(run1, oracle, "run 1 matches the hand oracle");
    assert_eq!(run2, oracle, "run 2 matches the hand oracle");
    assert_eq!(run1, run2, "the two runs are byte-identical");
}

// ---------------------------------------------------------------------------
// (h) The writer-side LENGTH GUARD (Principle #6: no silent zombie).
// ---------------------------------------------------------------------------

/// An over-long CANONICAL topic is refused LOUDLY at the public API,
/// naming the limit, and NOTHING enters the authoritative set (no permanent
/// un-encodable zombie that reports Ok(true) then re-fails on every republish).
/// Canonicalization prepends '/', so a 512-byte no-slash name becomes a 513-byte
/// canonical one — both the leading-slash-513 and the no-slash-512 forms exceed
/// the {MAX_REG_TOPIC_LEN}-byte bound. Hand oracle on the count (== 0).
#[test]
fn over_long_topic_refused_at_api_and_never_queued() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let worker = worker_manager("h1", &root);

    // A leading-slash topic whose canonical length is MAX_REG_TOPIC_LEN + 1.
    let over_canonical = format!("/{}", "x".repeat(MAX_REG_TOPIC_LEN));
    assert_eq!(over_canonical.len(), MAX_REG_TOPIC_LEN + 1);
    let err = worker
        .register_dynamic_egress_topic(&over_canonical, 1)
        .expect_err("an over-long topic must be refused");
    assert!(
        matches!(err, TransportError::InvalidTransportConfig { .. }),
        "the length refusal is InvalidTransportConfig, got {err:?}"
    );
    let msg = format!("{err}");
    assert!(
        msg.contains(&MAX_REG_TOPIC_LEN.to_string()),
        "the error names the {MAX_REG_TOPIC_LEN}-byte limit: {msg}"
    );
    // No zombie: the over-long name never entered the authoritative map.
    assert_eq!(
        worker.dynamic_egress_record_count_for_test(),
        0,
        "an over-long topic must NOT enter the authoritative set (no zombie)"
    );

    // A no-slash topic of exactly MAX_REG_TOPIC_LEN bytes canonicalizes to
    // MAX_REG_TOPIC_LEN + 1 and is ALSO refused — the '/' prepend pushes it over.
    let over_noslash = "y".repeat(MAX_REG_TOPIC_LEN);
    assert!(
        matches!(
            worker.register_dynamic_egress_topic(&over_noslash, 2),
            Err(TransportError::InvalidTransportConfig { .. })
        ),
        "a {MAX_REG_TOPIC_LEN}-byte no-slash name canonicalizes to {}-byte and is refused",
        MAX_REG_TOPIC_LEN + 1
    );
    assert_eq!(
        worker.dynamic_egress_record_count_for_test(),
        0,
        "still no zombie after the second refused registration"
    );
}

/// The EXACT-boundary topic — canonical length == MAX_REG_TOPIC_LEN — is
/// accepted and transmits end-to-end (registered + announced + hash stored at the
/// gateway). Pins that the guard rejects `> limit`, not `>= limit`. Hand oracles.
#[test]
fn boundary_length_topic_registers_and_transmits_e2e() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("h2", &root);
    let worker = worker_manager("h2", &root);

    // '/' + (MAX_REG_TOPIC_LEN - 1) chars == exactly MAX_REG_TOPIC_LEN canonical.
    let exact = format!("/{}", "z".repeat(MAX_REG_TOPIC_LEN - 1));
    assert_eq!(exact.len(), MAX_REG_TOPIC_LEN);
    const HASH: u64 = 0x0BAD_C0DE_F00D_1234;

    assert!(
        worker
            .register_dynamic_egress_topic(&exact, HASH)
            .expect("the exact-boundary topic registers"),
        "canonical length == MAX_REG_TOPIC_LEN is accepted (guard is > limit, not >=)"
    );
    assert_eq!(
        worker.dynamic_egress_record_count_for_test(),
        1,
        "the boundary topic entered the authoritative set"
    );

    settle(&worker, &mut gateway, 3);

    // Transmits e2e: registered + announced + hash stored at the gateway.
    assert_eq!(registered_set(&gateway), vec![exact.clone()]);
    assert!(gateway.is_runtime_registered(&exact).expect("is reg"));
    assert_eq!(gateway.runtime_topic_schema_hash(&exact), Some(HASH));
    assert!(
        gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&exact),
        "the boundary topic is announced (discovery truth)"
    );
    assert_eq!(gateway.registration_malformed_count(), 0);
    assert_eq!(gateway.registration_rejected_count(), 0);
}

// ---------------------------------------------------------------------------
// (i) The writer-side pump-liveness accessor (Principle #3).
// ---------------------------------------------------------------------------

/// `TransportManager::registration_pump_active()` is `false` before the
/// first registration (the channel + pump are lazily created) and `true` after
/// (the first register starts the background republish belt) — the writer-side
/// symmetry to the gateway's `runtime_registration_active`. Hand oracle.
#[test]
fn registration_pump_active_reflects_lazy_start() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let worker = worker_manager("i", &root);

    assert!(
        !worker.registration_pump_active(),
        "no pump before the first registration (channel is lazily created)"
    );
    let topic = format!("/rt/i/{}", unique_id());
    worker
        .register_dynamic_egress_topic(&topic, 1)
        .expect("register");
    assert!(
        worker.registration_pump_active(),
        "the first registration started the background republish pump"
    );
}

// ---------------------------------------------------------------------------
// (j) A transient ANNOUNCE failure at the drain is a DEFERRAL, not a
//     rejection; the topic stays demand-grantable and self-heals.
// ---------------------------------------------------------------------------

/// A LEGITIMATE runtime topic whose gateway-side announce hits a transient
/// failure at the drain bumps `reg_announce_deferred` (NOT `reg_rejected`), emits
/// NO false "will NOT be announced or made demandable" warn, keeps the topic
/// demand-grantable (flag RETAINED), and self-HEALS on a later drain (the announce
/// is re-attempted idempotently). Driven by the fire-once
/// `fault_inject_announce_egress_once` seam. Hand oracles on every count.
#[traced_test]
#[test]
fn announce_failure_at_drain_defers_not_rejects_and_heals() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("j", &root);
    let worker = worker_manager("j", &root);
    let topic = format!("/rt/j/{}", unique_id());

    worker
        .register_dynamic_egress_topic(&topic, 0xFEED)
        .expect("register");

    // Arm the fire-once announce fault: the NEXT announce_egress_topic fails. The
    // gateway's boot plan is empty (no boot announce), so the fault fires on the
    // runtime topic's announce when the drain applies it.
    gateway
        .manager()
        .network()
        .expect("network")
        .fault_inject_announce_egress_once();

    // Drive until the record is drained and the announce fault fires (connection
    // may take a pass or two). BOUNDED — a wedge fails loudly, never hangs.
    let mut deferred = false;
    for _ in 0..30 {
        worker.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive");
        if gateway.registration_announce_deferred_count() >= 1 {
            deferred = true;
            break;
        }
    }
    assert!(
        deferred,
        "the announce failure at the drain bumped reg_announce_deferred"
    );
    // It is a DEFERRAL, not a rejection, and not malformed.
    assert_eq!(
        gateway.registration_rejected_count(),
        0,
        "a legit topic's announce blip is NOT a rejection"
    );
    assert_eq!(gateway.registration_malformed_count(), 0);
    // The demand flag is RETAINED (grantable) despite the deferred announce.
    assert!(
        gateway.is_runtime_registered(&topic).expect("is reg"),
        "the topic is demand-grantable (the flag is retained, not rolled back)"
    );
    // NB: we do NOT assert the topic is transiently un-announced here — the
    // background republish belt can place a DUPLICATE record in the same drain
    // queue, so the fire-once fault (deferred, count 1) and the idempotent re-
    // attempt (announce succeeds) can both land in ONE drive pass. The
    // contract is the counter discrimination + no-false-warn + eventual heal,
    // all asserted below; the exact instant of announce is not part of it.

    // No FALSE refusal warn for the legit topic (that text is reserved for a true
    // rejection). The accurate registered-but-unannounced warn is allowed.
    logs_assert(|lines: &[&str]| {
        let false_refusal = lines
            .iter()
            .filter(|l| {
                l.contains("will NOT be announced or made demandable") && l.contains(&topic)
            })
            .count();
        if false_refusal != 0 {
            return Err(format!(
                "a deferred legit topic must NOT get the refusal warn, got {false_refusal}"
            ));
        }
        Ok(())
    });

    // HEAL: the fault is fire-once (already cleared). Keep driving — the worker
    // re-sends the record each republish and the gateway re-attempts the announce
    // idempotently, so discovery truth is restored.
    let mut healed = false;
    for _ in 0..30 {
        worker.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive");
        if gateway
            .manager()
            .network()
            .expect("network")
            .announced_topics()
            .contains(&topic)
        {
            healed = true;
            break;
        }
    }
    assert!(
        healed,
        "the announce self-heals on a later drain (idempotent re-attempt)"
    );
    // Exactly ONE deferred blip — the Ok heal path stores the hash without
    // bumping the counter again, and never a rejection.
    assert_eq!(
        gateway.registration_announce_deferred_count(),
        1,
        "exactly one deferred blip — the heal (Ok path) does not re-count it"
    );
    assert_eq!(gateway.registration_rejected_count(), 0);
    // The advertised hash is now stored (the heal's Ok drain stored it).
    assert_eq!(
        gateway.runtime_topic_schema_hash(&topic),
        Some(0xFEED),
        "the heal's successful drain stored the advertised hash"
    );
}

// ---------------------------------------------------------------------------
// A panicking registration poisons NOTHING.
// ---------------------------------------------------------------------------

/// A registration whose live SEND panics — the seam sits inside the one
/// panic-capable step, AFTER the record is retained and the pump is armed —
/// leaves the writer fully usable: the record is retained and republishable,
/// the pump is live, and the NEXT registration on the same manager succeeds.
/// Nothing is poisoned because no lock is held across the send.
///
/// Were the manager's lazy-slot guard held across the WHOLE
/// registration, the poison would turn every later registration in the
/// process into the poisoned-lock error (every later publisher local-only);
/// and a retain, send, THEN arm-the-pump order would leave a first-ever
/// registration's record with no pump — never republished to a gateway that
/// boots later. Both halves are pinned on a FRESH worker (first-ever
/// registration ⇒ the arm ordering is observable), with the panic payload
/// asserted so the failure is attributable to the injection. Hand oracles.
#[test]
fn a_panicking_send_poisons_nothing_and_the_next_registration_succeeds() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("pp", &root);
    let worker = worker_manager("pp", &root);
    let first = format!("/rt/pp/first/{}", unique_id());
    let second = format!("/rt/pp/second/{}", unique_id());
    assert!(
        !worker.registration_pump_active(),
        "fresh worker: no pump yet (precondition for the arm-ordering half)"
    );

    panic_on_next_register_send_for_test(&first);
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        worker.register_dynamic_egress_topic(&first, 0xA1)
    }));
    let payload = outcome.expect_err("the armed seam must panic the first registration");
    let message = payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
        .unwrap_or_default();
    assert!(
        message.contains(REGISTER_SEND_PANIC_MSG),
        "the panic is the injected one, not something else: {message}"
    );

    // The pump was armed and the record retained BEFORE the panic-capable send.
    assert!(
        worker.registration_pump_active(),
        "the pump is armed before the send — a retained record is never left pump-less"
    );
    assert_eq!(
        worker.dynamic_egress_record_hash_for_test(&first),
        Some(0xA1),
        "the record was retained before the send"
    );

    // The plane is usable: the next registration succeeds (nothing poisoned).
    assert!(
        worker
            .register_dynamic_egress_topic(&second, 0xB2)
            .expect("the next registration must succeed — no lock was poisoned"),
        "a genuinely new topic"
    );

    // Both reach the gateway — the first ONLY via the republish belt, since its
    // live send never happened.
    settle(&worker, &mut gateway, 3);
    assert_eq!(gateway.runtime_topic_schema_hash(&first), Some(0xA1));
    assert_eq!(gateway.runtime_topic_schema_hash(&second), Some(0xB2));
    assert_eq!(gateway.registration_malformed_count(), 0);
    assert_eq!(gateway.registration_rejected_count(), 0);
}

/// N FIRST registrations racing on an empty channel slot open exactly ONE
/// control publisher: without
/// serialization, every racer that observed the empty slot would open its OWN
/// control publisher before one won the install — and the control service
/// caps writers at `REG_CHANNEL_MAX_WRITERS`, so a concurrent burst larger
/// than the cap would exhaust it and the losers' topics stay local-only.
/// Creation is double-checked under a dedicated creation mutex.
///
/// A barrier releases all racers at once (the shape that maximizes an
/// unserialized fan-out) while an observer thread polls the service's LIVE
/// publisher count throughout: the maximum it ever sees must be 1 (under the
/// mutex, exactly one open ever happens — deterministic on this isolated
/// root), the count is exactly 1 after the race, and every racer's topic is
/// registered with its hash. Hand oracles.
///
/// Reverting to unserialized creation fans the racers
/// into concurrent opens and the observer's maximum exceeds 1.
#[test]
fn racing_first_registrations_open_exactly_one_control_publisher() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let worker = worker_manager("race", &root);
    const RACERS: usize = 8;

    // Pin the control SERVICE in existence for the whole test: iceoryx2
    // removes a service once its last port and factory drop, and its
    // `open_or_create` errors `HangsInCreation` when it races another party's
    // mid-creation — so without the keepalive, the observer's polling and the
    // racers' opens race the very first creation this test orchestrates (a
    // harness artifact, not the property under test). The publisher-port
    // fan-out IS the property, and the keepalive holds ZERO ports — the
    // assert is the 0-publisher baseline.
    let _service_keepalive = worker
        .dynamic_egress_control_service_keepalive_for_test()
        .expect("pin the control service for the test's lifetime");
    assert_eq!(
        worker
            .dynamic_egress_control_publisher_count_for_test()
            .expect("baseline publisher count"),
        0,
        "no control publisher exists before any registration"
    );
    assert_eq!(
        worker.dynamic_egress_channel_creations_for_test(),
        0,
        "no control channel has been created on this manager yet"
    );

    let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let max_seen = Arc::new(AtomicU64::new(0));
    let observer = {
        let worker = Arc::clone(&worker);
        let stop = Arc::clone(&stop);
        let max_seen = Arc::clone(&max_seen);
        std::thread::spawn(move || {
            while !stop.load(Ordering::SeqCst) {
                if let Ok(count) = worker.dynamic_egress_control_publisher_count_for_test() {
                    max_seen.fetch_max(count as u64, Ordering::SeqCst);
                }
                std::thread::yield_now();
            }
        })
    };

    let barrier = Arc::new(std::sync::Barrier::new(RACERS));
    let racers: Vec<_> = (0..RACERS)
        .map(|i| {
            let worker = Arc::clone(&worker);
            let barrier = Arc::clone(&barrier);
            std::thread::spawn(move || {
                let topic = format!("/rt/race/t{i}");
                barrier.wait();
                worker.register_dynamic_egress_topic(&topic, i as u64)
            })
        })
        .collect();
    for (i, handle) in racers.into_iter().enumerate() {
        assert!(
            handle
                .join()
                .expect("racer must not panic")
                .expect("every racing registration succeeds"),
            "racer {i} registered a genuinely new topic"
        );
    }
    stop.store(true, Ordering::SeqCst);
    observer.join().expect("observer");

    // THE oracle — CREATIONS, not live handles: a counter bumped on every
    // control-channel open inside the create arm, keyed to THIS manager (so
    // sibling tests' own first registrations in this parallel binary cannot
    // bleed into it). Deterministic either way: serialized creation opens exactly once,
    // and an unserialized implementation that opened several and dropped
    // the losers before any sample could be taken would still have COUNTED
    // every open — the sampled live maximum below is only the secondary
    // check.
    assert_eq!(
        worker.dynamic_egress_channel_creations_for_test(),
        1,
        "exactly ONE control channel (and so one control publisher) was ever \
         created for this manager — creation is serialized (unserialized, every \
         racer that saw the empty slot would create its own)"
    );

    // Secondary: the live view. One deterministic final sample (the winner's
    // publisher is still alive) folds a guaranteed floor into the observer's
    // maximum even if the OS starved the observer thread through the race.
    let final_count = worker
        .dynamic_egress_control_publisher_count_for_test()
        .expect("publisher count after the race") as u64;
    assert_eq!(
        final_count, 1,
        "exactly one live control publisher after the race"
    );
    assert_eq!(
        max_seen.load(Ordering::SeqCst).max(final_count),
        1,
        "the observer never saw more than one live control publisher"
    );
    for i in 0..RACERS {
        assert_eq!(
            worker.dynamic_egress_record_hash_for_test(&format!("/rt/race/t{i}")),
            Some(i as u64),
            "racer {i}'s topic is registered with its hash"
        );
    }
}

// ---------------------------------------------------------------------------
// Pump-thread panic: containment, accurate liveness, stay-down.
// ---------------------------------------------------------------------------

/// Containment pin: a panic surfacing from the PUMP's republish
/// tick (injected via the topic-keyed pump-send seam, AFTER the real loan +
/// send) is CONTAINED — the tick dies, the belt survives. Registered with NO
/// gateway in existence, so the immediate live send has no subscriber and
/// the ONLY way the topic can reach the gateway booted below is the
/// background pump belt itself — delivery there proves the belt RESUMED
/// after the panic (this test never drives the inline republish seam).
///
/// Removing the `catch_unwind` in `pump_tick` (calling
/// `inner.republish()` bare) lets the injected panic kill the thread before
/// the containment counter can move — the first bounded wait times out.
#[test]
fn a_pump_tick_panic_is_contained_the_belt_survives_and_resumes() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let worker = worker_manager("pk1", &root);
    let topic = format!("/rt/pk1/{}", unique_id());

    worker
        .register_dynamic_egress_topic(&topic, 0xC1)
        .expect("register");
    assert!(worker.registration_pump_active(), "pump started live");
    assert_eq!(worker.dynamic_egress_pump_tick_panics_for_test(), 0);

    // Arm: the next PUMP republish of this topic panics inside the tick body.
    panic_on_next_pump_send_for_test(&topic);

    // Bounded wait for the pump to take (and contain) the injected panic.
    let deadline = Instant::now() + Duration::from_secs(15);
    while worker.dynamic_egress_pump_tick_panics_for_test() == 0 {
        assert!(
            Instant::now() < deadline,
            "the armed pump tick never panicked-and-was-contained — with the \
             catch_unwind removed the thread dies before the counter moves"
        );
        std::thread::sleep(Duration::from_millis(25));
    }
    assert_eq!(
        worker.dynamic_egress_pump_tick_panics_for_test(),
        1,
        "the seam is fire-once — exactly one contained tick panic"
    );
    assert!(
        worker.registration_pump_active(),
        "the belt SURVIVES a contained tick panic — active stays true"
    );

    // The belt RESUMES: a gateway booting only NOW receives the topic via the
    // background pump alone.
    let mut gateway = gateway_on("pk1", &root);
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        gateway.drive_once().expect("drive");
        if gateway.is_runtime_registered(&topic).expect("is reg") {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "the pump never delivered after the contained panic — the belt did not resume"
        );
        std::thread::sleep(Duration::from_millis(25));
    }
    assert_eq!(gateway.runtime_topic_schema_hash(&topic), Some(0xC1));
    assert!(
        worker.registration_pump_active(),
        "still live after the heal"
    );
    assert_eq!(
        worker.dynamic_egress_pump_tick_panics_for_test(),
        1,
        "no further panics — the next clean tick closed the regime"
    );
}

/// Liveness pin: a panic that ESCAPES the per-tick containment (the
/// death seam fires in the pump scaffolding, OUTSIDE the catch) kills the
/// thread — and `registration_pump_active()` must flip FALSE, because
/// liveness is a flag the thread body clears by RAII on any exit, never the
/// handle's mere existence. Also pins the documented STAY-DOWN contract: a
/// later register still succeeds (nothing poisoned, the live-send plane is
/// intact) but does NOT silently respawn the dead pump.
///
/// Making `PumpLiveGuard::drop` a no-op (or reverting the accessor
/// to `is_some()`) leaves the flag never flipping — the bounded wait times out.
#[test]
fn a_dead_pump_thread_reads_inactive_and_stays_down() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let worker = worker_manager("pk2", &root);
    let topic = format!("/rt/pk2/{}", unique_id());
    worker
        .register_dynamic_egress_topic(&topic, 0xC2)
        .expect("register");
    assert!(worker.registration_pump_active(), "pump started live");

    // Arm the death seam: the next pump ITERATION of the channel holding this
    // topic panics OUTSIDE the per-tick containment — the thread dies.
    kill_pump_thread_for_test(&topic);

    let deadline = Instant::now() + Duration::from_secs(15);
    while worker.registration_pump_active() {
        assert!(
            Instant::now() < deadline,
            "registration_pump_active still reads true after the pump thread \
             died — the observable is lying (the earlier hole)"
        );
        std::thread::sleep(Duration::from_millis(25));
    }

    // A thread DEATH is not a contained tick panic.
    assert_eq!(
        worker.dynamic_egress_pump_tick_panics_for_test(),
        0,
        "an escaped panic must not count as a contained tick panic"
    );

    // STAY-DOWN: a later register still works but does not respawn the pump.
    let second = format!("/rt/pk2b/{}", unique_id());
    assert!(
        worker
            .register_dynamic_egress_topic(&second, 0xC3)
            .expect("a register after the pump death must still succeed"),
        "a genuinely new topic"
    );
    assert!(
        !worker.registration_pump_active(),
        "stay-down contract: a later register does not silently respawn the dead pump"
    );
    std::thread::sleep(Duration::from_millis(300));
    assert!(
        !worker.registration_pump_active(),
        "and no delayed respawn either"
    );
}

// ---------------------------------------------------------------------------
// The control service's cross-process first-creation collision.
// ---------------------------------------------------------------------------

/// Retry pin: ONE forced `HangsInCreation` on the first open attempt
/// (the node-name-keyed injection seam — the real collision window needs a
/// peer stalled past iceoryx2's 500 ms creation wait, which no in-process
/// test can stage deterministically; the seam substitutes only the ORIGIN of
/// the genuine error value, and the classification/retry/heal code runs
/// unchanged) heals on the bounded re-attempt: the registration succeeds,
/// the channel is fully live, and records flow e2e.
///
/// Removing the retry (surfacing the collision on the first
/// attempt) makes the register call error instead of healing.
#[test]
fn a_transient_creation_collision_is_retried_and_heals() {
    let root = cerulion_core::testing::iceoryx_test_config();
    // Built by hand (not `worker_manager`) so the test knows the manager's
    // exact node name — the seam is node-name-keyed for parallel-binary
    // confinement.
    let node_name = format!("rc_cc1_{}", unique_id());
    let worker = TransportManager::init_for_test(
        TransportConfig {
            node_name: node_name.clone(),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init worker manager");
    let topic = format!("/rt/cc1/{}", unique_id());

    force_control_open_collisions_for_test(&node_name, 1);
    assert!(
        worker
            .register_dynamic_egress_topic(&topic, 0xD1)
            .expect("the registration succeeds THROUGH one forced collision — the retry healed it"),
        "a genuinely new topic"
    );
    assert_eq!(
        forced_control_open_collisions_remaining_for_test(&node_name),
        0,
        "the forced collision was consumed — the first attempt really failed \
         and the re-attempt did the real open"
    );
    assert!(
        worker.registration_pump_active(),
        "the channel is fully live after the heal"
    );
    assert_eq!(
        worker.dynamic_egress_channel_creations_for_test(),
        1,
        "exactly one channel creation despite the retried open"
    );

    // e2e: the healed channel really carries records to a gateway.
    let mut gateway = gateway_on("cc1", &root);
    settle(&worker, &mut gateway, 3);
    assert_eq!(gateway.runtime_topic_schema_hash(&topic), Some(0xD1));
}

/// Exhaustion pin: forcing MORE collisions than the bound surfaces
/// the classified hard error — naming the service, BOTH boot roles (the
/// gateway's READER, a worker's WRITER), the exhausted bound, the diagnosis
/// (a peer died mid-creation) and the stale-artifact remedy — after exactly
/// `CONTROL_OPEN_ATTEMPTS` attempts, and the failure is NOT sticky: once the
/// environment heals, the next register lazily re-attempts and succeeds.
#[test]
fn an_exhausted_creation_collision_is_a_loud_actionable_error_and_not_sticky() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let node_name = format!("rc_cc2_{}", unique_id());
    let worker = TransportManager::init_for_test(
        TransportConfig {
            node_name: node_name.clone(),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init worker manager");
    let topic = format!("/rt/cc2/{}", unique_id());

    // One MORE than the bound: exhaustion must stop at the bound, so exactly
    // one forced collision is left over afterwards.
    force_control_open_collisions_for_test(&node_name, CONTROL_OPEN_ATTEMPTS + 1);
    let err = worker
        .register_dynamic_egress_topic(&topic, 0xD2)
        .expect_err("exhausting the collision bound must surface a hard error");
    let msg = format!("{err}");
    for needle in [
        "/__cerulion/gateway_topics",
        "HANGS IN CREATION",
        &CONTROL_OPEN_ATTEMPTS.to_string(),
        "READER",
        "WRITER",
        "gateway",
        "worker",
        "DIED mid-creation",
        "stale iceoryx2 shared-memory artifacts",
    ] {
        assert!(
            msg.contains(needle),
            "the exhaustion error must carry '{needle}' — got: {msg}"
        );
    }
    assert_eq!(
        forced_control_open_collisions_remaining_for_test(&node_name),
        1,
        "exactly CONTROL_OPEN_ATTEMPTS attempts consumed forced collisions — \
         the retry ladder is bounded, never one attempt more"
    );

    // NOT STICKY: disarm the seam; the next register lazily re-creates the
    // channel and succeeds (the exhausted failure queued/poisoned nothing).
    force_control_open_collisions_for_test(&node_name, 0);
    assert!(
        worker
            .register_dynamic_egress_topic(&topic, 0xD2)
            .expect("a clean retry succeeds after the environment heals"),
        "the topic is genuinely new — the failed attempt queued nothing"
    );
    assert_eq!(worker.dynamic_egress_channel_creations_for_test(), 1);
    let mut gateway = gateway_on("cc2", &root);
    settle(&worker, &mut gateway, 3);
    assert_eq!(gateway.runtime_topic_schema_hash(&topic), Some(0xD2));
}

/// The REAL-TRANSPORT race arm: a worker writer's first
/// registration and a reader-shaped service open race a COLD root's very
/// first creation, with NO `ControlServiceKeepalive` pinning the service —
/// the keepalive is how the pre-existing race pin DODGES this window; here
/// the window is the subject. Scope: the deterministic coverage of
/// the `HangsInCreation` classification + retry is the seam-driven pair
/// above (the true collision needs a creator stalled past iceoryx2's 500 ms
/// creation window, which a healthy in-process peer never is); this arm pins
/// that concurrent cold-boot first creations ALL SUCCEED through the
/// production open path — iceoryx2's internal AlreadyExists/IsBeingCreated
/// retry composed with our bounded collision retry — round after round.
#[test]
fn racing_cold_boot_first_creations_all_succeed_without_keepalive() {
    for round in 0..4u64 {
        // FRESH root each round: a genuine first creation every time.
        let root = cerulion_core::testing::iceoryx_test_config();
        let writer = worker_manager(&format!("ccr_w{round}"), &root);
        let opener = worker_manager(&format!("ccr_o{round}"), &root);
        let topic = format!("/rt/ccr/{round}/{}", unique_id());
        let barrier = Arc::new(std::sync::Barrier::new(2));

        let write = {
            let writer = Arc::clone(&writer);
            let barrier = Arc::clone(&barrier);
            let topic = topic.clone();
            std::thread::spawn(move || {
                barrier.wait();
                writer.register_dynamic_egress_topic(&topic, round)
            })
        };
        let open = {
            let opener = Arc::clone(&opener);
            let barrier = Arc::clone(&barrier);
            std::thread::spawn(move || {
                barrier.wait();
                // The reader-shaped boot: opens the control service through
                // the SAME open_or_create helper the gateway's
                // RegistrationReader uses (the probe opens the factory only).
                opener.dynamic_egress_control_publisher_count_for_test()
            })
        };

        let newly = write
            .join()
            .expect("writer thread must not panic")
            .expect("the racing writer registration succeeds");
        assert!(newly, "a fresh topic each round");
        let count = open
            .join()
            .expect("opener thread must not panic")
            .expect("the racing reader-shaped open succeeds");
        assert!(
            count <= 1,
            "round {round}: at most the writer's one control publisher exists"
        );
        assert_eq!(
            writer.dynamic_egress_record_hash_for_test(&topic),
            Some(round),
            "round {round}: the registration landed in the authoritative set"
        );
    }
}

// ---------------------------------------------------------------------------
// The panic HOOK stays bounded for contained pump panics.
// ---------------------------------------------------------------------------

/// Counter pin: N contained pump-tick panics engage the wrapping
/// hook's SUPPRESSED arm N times — each of those would otherwise have been
/// one default-hook `thread ... panicked at ...` stderr line per 250 ms tick,
/// unbounded under a persistent send-panic (the disk-fill class,
/// re-opened on a channel the regime latch does not control; measured at
/// exactly one line per contained panic without the hook). The counter is
/// process-global (the hook is), so under this binary's PARALLEL tests the
/// assertion is a `>=` DELTA anchored to this test's own contained-panic
/// count, never an absolute.
///
/// Making the wrapping hook ignore the suppress flag (always
/// delegating) makes the delta read 0.
#[test]
fn contained_pump_panics_are_swallowed_by_the_hook_not_printed() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let worker = worker_manager("pk3", &root);
    let topic = format!("/rt/pk3/{}", unique_id());
    worker
        .register_dynamic_egress_topic(&topic, 0xC2)
        .expect("register");

    let hook_baseline = suppressed_panic_hook_reports_for_test();
    // A PERSISTENT regime — 3 ticks — because one contained panic cannot
    // distinguish "printed once" from "prints per tick, unbounded".
    panic_on_next_pump_sends_for_test(&topic, 3);

    let deadline = Instant::now() + Duration::from_secs(20);
    while worker.dynamic_egress_pump_tick_panics_for_test() < 3 {
        assert!(
            Instant::now() < deadline,
            "the 3-tick persistent panic regime never completed"
        );
        std::thread::sleep(Duration::from_millis(25));
    }
    assert_eq!(
        worker.dynamic_egress_pump_tick_panics_for_test(),
        3,
        "the counted seam disarms at zero — exactly 3 contained panics"
    );

    let delta = suppressed_panic_hook_reports_for_test() - hook_baseline;
    assert!(
        delta >= 3,
        "each contained tick panic must take the wrapping hook's SUPPRESSED arm \
         (the default-hook stderr line swallowed): suppressed delta {delta} < 3 — \
         with the suppression deleted this reads 0 and every contained panic \
         printed an unbounded per-tick stderr line"
    );
    assert!(
        worker.registration_pump_active(),
        "the belt survives the persistent regime (the containment contract is untouched)"
    );
}

/// Stderr pin, subprocess-based — the only way to observe what the
/// REAL default hook actually prints. The child process drives a PERSISTENT
/// contained-panic regime (4 pump ticks), then panics an ordinary
/// UN-suppressed thread as the control. The parent asserts the child's
/// stderr carries EXACTLY ONE `panicked at` line — the control thread's —
/// which proves both halves at once:
///
/// - contained pump panics print NOTHING (without the hook: one line per tick, and a
///   count-only oracle could not see a variant that both counts AND
///   delegates);
/// - the wrapper DELEGATES for every other thread — a normal panic still
///   reaches the previous hook and prints, so libtest / the state-carrier
///   fork hook / a user hook chain through untouched.
#[test]
fn the_default_hook_is_silent_for_contained_pump_panics_and_loud_for_others() {
    use std::io::Read as _;
    use std::process::{Command, Stdio};

    let exe = std::env::current_exe().expect("current_exe");
    let mut child = Command::new(exe)
        .args([
            "--exact",
            "subprocess_child_pump_panic_storm_probe",
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env("CER_REG_CHANNEL_CHILD", "1")
        // Determinism: a backtrace would add stderr lines (none of them match
        // "panicked at", but keep the capture minimal and stable).
        .env("RUST_BACKTRACE", "0")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn subprocess child");

    // Drain both pipes on threads so a filling pipe can never wedge the child,
    // and bound the wait so a wedged child fails loudly instead of hanging.
    let mut out_pipe = child.stdout.take().expect("child stdout");
    let mut err_pipe = child.stderr.take().expect("child stderr");
    let out_thread = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = out_pipe.read_to_string(&mut s);
        s
    });
    let err_thread = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = err_pipe.read_to_string(&mut s);
        s
    });
    let deadline = Instant::now() + Duration::from_secs(90);
    let status = loop {
        if let Some(status) = child.try_wait().expect("try_wait") {
            break status;
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            panic!("subprocess child did not finish within the bound");
        }
        std::thread::sleep(Duration::from_millis(50));
    };
    let stdout = out_thread.join().expect("stdout drain");
    let stderr = err_thread.join().expect("stderr drain");

    assert!(
        status.success(),
        "child failed (status {status:?})\n--- child stdout ---\n{stdout}\n--- child stderr ---\n{stderr}"
    );
    assert!(
        stdout.contains("CHILD-PANICS=4"),
        "child must report 4 contained panics; stdout:\n{stdout}"
    );

    let hook_lines: Vec<&str> = stderr
        .lines()
        .filter(|l| l.contains("panicked at"))
        .collect();
    assert!(
        !stderr
            .lines()
            .any(|l| l.contains("panicked at") && l.contains("cer-reg-republish")),
        "a CONTAINED pump panic reached the default hook — the per-tick stderr \
         flood is back; stderr:\n{stderr}"
    );
    assert_eq!(
        hook_lines.len(),
        1,
        "exactly ONE default-hook line — the un-suppressed control thread's \
         (0 = the wrapper stopped delegating for ordinary threads; >1 = a \
         contained pump panic printed); lines: {hook_lines:?}\nstderr:\n{stderr}"
    );
    assert!(
        hook_lines[0].contains("cer-ctl-panic"),
        "the one printed line must be the control thread's: {:?}",
        hook_lines[0]
    );
}

/// The subprocess body for the test above — env-gated so a blanket
/// `-- --ignored` sweep cannot start a panic storm outside the driving
/// parent's controlled capture.
#[test]
#[ignore = "subprocess child entrypoint (CER_REG_CHANNEL_CHILD=1) — driven by the_default_hook_is_silent_..."]
fn subprocess_child_pump_panic_storm_probe() {
    if std::env::var("CER_REG_CHANNEL_CHILD").as_deref() != Ok("1") {
        return;
    }
    let root = cerulion_core::testing::iceoryx_test_config();
    let worker = worker_manager("pk3c", &root);
    let topic = format!("/rt/pk3c/{}", unique_id());
    worker
        .register_dynamic_egress_topic(&topic, 0xC3)
        .expect("register");

    panic_on_next_pump_sends_for_test(&topic, 4);
    let deadline = Instant::now() + Duration::from_secs(30);
    while worker.dynamic_egress_pump_tick_panics_for_test() < 4 {
        assert!(
            Instant::now() < deadline,
            "child: the 4-tick persistent panic regime never completed"
        );
        std::thread::sleep(Duration::from_millis(25));
    }

    // Control: an ordinary, UN-suppressed thread's panic must still reach the
    // previous hook and print. Named so the parent can attribute the one line.
    let ctl = std::thread::Builder::new()
        .name("cer-ctl-panic".to_string())
        .spawn(|| panic!("control: un-suppressed thread panic must print"))
        .expect("spawn control thread");
    assert!(ctl.join().is_err(), "the control thread panicked");

    println!(
        "CHILD-PANICS={} CHILD-SUPPRESSED={}",
        worker.dynamic_egress_pump_tick_panics_for_test(),
        suppressed_panic_hook_reports_for_test()
    );
}

// ---------------------------------------------------------------------------
// (m) The writer-slot ARITHMETIC — registration is per-PROCESS.
// ---------------------------------------------------------------------------

/// The control service's `max_publishers = REG_CHANNEL_MAX_WRITERS`
/// budget is consumed per PROCESS, never per registered topic or per rmw
/// publisher: ONE worker registering MORE topics than the whole writer cap
/// still holds exactly ONE control publisher (and every topic is SERVED — all
/// land at the gateway), and three workers hold exactly three. This is the
/// arithmetic that makes 64 slots mean "64 concurrently-registering PROCESSES
/// per machine" (a nav2/MoveIt process with dozens of rmw publishers consumes ONE
/// slot), not "64 rmw publishers per machine". Hand oracles.
///
/// Making
/// `TransportManager::dynamic_egress_channel` open a fresh RETAINED channel
/// per registration (the per-publisher shape this arithmetic forbids) fails
/// this test — the registrations past the writer cap error out of slots and
/// the publisher count reads far above 1.
#[test]
fn one_control_publisher_per_process_regardless_of_topic_count() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("m", &root);
    let worker = worker_manager("m1", &root);

    // MORE topics than the whole writer cap: a per-publisher registration
    // shape would exhaust the service's publisher slots at registration
    // `REG_CHANNEL_MAX_WRITERS + 1`; per-process registration never can.
    // Pin the PROMISED cap value itself, independently of the derivation below:
    // TOPICS_PER_WORKER is derived FROM the cap, so without this a cap bump to
    // 128 would silently re-derive the burst and every assertion would keep
    // passing — the "64 slots" promise this test exists to pin would be gone.
    const {
        assert!(
            REG_CHANNEL_MAX_WRITERS == 64,
            "the reg-channel writer cap is documented as 64 per machine; a deliberate \
             cap change must update the docs and this pin together"
        )
    };
    const TOPICS_PER_WORKER: usize = REG_CHANNEL_MAX_WRITERS + 6;
    const {
        assert!(
            TOPICS_PER_WORKER > REG_CHANNEL_MAX_WRITERS,
            "the burst must exceed the writer cap or the arithmetic is untested"
        )
    };
    let base = unique_id();
    let mut expected: Vec<String> = Vec::new();
    for i in 0..TOPICS_PER_WORKER {
        let topic = format!("/regch/w1/{base}/t{i:03}");
        assert!(
            worker
                .register_dynamic_egress_topic(&topic, i as u64)
                .expect("register"),
            "registration {i} succeeds — past the writer cap included"
        );
        expected.push(topic);
    }
    assert_eq!(
        worker
            .dynamic_egress_control_publisher_count_for_test()
            .expect("publisher count"),
        1,
        "one process = ONE control publisher, however many topics it registers"
    );

    // Two more "processes" (managers on the same root): one slot each.
    let w2 = worker_manager("m2", &root);
    let w3 = worker_manager("m3", &root);
    for (w, tag) in [(&w2, "w2"), (&w3, "w3")] {
        let topic = format!("/regch/{tag}/{base}");
        assert!(
            w.register_dynamic_egress_topic(&topic, 7)
                .expect("register"),
            "each sibling process registers its own topic"
        );
        expected.push(topic);
    }
    assert_eq!(
        worker
            .dynamic_egress_control_publisher_count_for_test()
            .expect("publisher count"),
        3,
        "three registering processes consume exactly three writer slots"
    );

    // And the whole set is SERVED: every topic from every process reaches the
    // gateway (the cap bounds processes, never the topics they announce).
    for _ in 0..4 {
        worker.republish_dynamic_egress_for_test();
        w2.republish_dynamic_egress_for_test();
        w3.republish_dynamic_egress_for_test();
        gateway.drive_once().expect("drive_once");
    }
    expected.sort();
    assert_eq!(
        registered_set(&gateway),
        expected,
        "every topic from all three processes is registered at the gateway"
    );
    assert_eq!(gateway.registration_malformed_count(), 0);
    assert_eq!(gateway.registration_rejected_count(), 0);
}

// ---------------------------------------------------------------------------
// (g) REAL-OS-PROCESS capacity pin: the
// in-process test above models processes with `init_for_test` managers, which
// are documented NON-singleton instances — a regression that broke
// process-wide manager sharing would still pass it. This arm spawns TWO real
// child PROCESSES (self-re-exec, crib `barrier_level_gate_subprocess` /
// `shm_ring_test`), each holding ONE manager and registering its own topic
// over the SAME root, and pins from the parent: both topics are SERVED at the
// gateway, the control service's LIVE publisher count reads EXACTLY 2 while
// both children are alive (one writer slot per OS process), and the slots
// DRAIN back to 0 once the children DROP their managers and exit — release
// is a MANAGER-DROP property, which these children exercise by construction
// (mgr drops when run() returns, before process::exit). The PRODUCTION shape
// — the OnceLock-retained singleton that is NEVER dropped — is pinned by the
// second test below: a child that leaks its manager (mem::forget, modelling
// the singleton) and exits leaves its port REGISTERED (stale state until
// iceoryx2's dead-node cleanup), exactly what the module docs claim.
// Runs on macOS and Linux (real POSIX SHM everywhere; no #[ignore]).
// ---------------------------------------------------------------------------

/// Path to the serialized shared iceoryx2 `Config` (same root as the parent).
const ENV_REGCH_CONFIG: &str = "CER_REGCH_SUBPROC_CONFIG";
/// The one topic this child registers.
const ENV_REGCH_TOPIC: &str = "CER_REGCH_SUBPROC_TOPIC";
/// Stop-file path: the child republishes until it appears, then exits 0.
const ENV_REGCH_STOP: &str = "CER_REGCH_SUBPROC_STOP";
/// When set, the child LEAKS its manager before exiting (mem::forget) —
/// modelling the production OnceLock singleton, which is never dropped.
const ENV_REGCH_LEAK: &str = "CER_REGCH_SUBPROC_LEAK";

/// Child entrypoint — re-invoked as a subprocess via `current_exe`. In a
/// normal suite run the env is absent and this is a no-op pass (the
/// barrier-subprocess pattern).
///
/// `process::exit` is repo-banned; a subprocess ENTRYPOINT is the one sanctioned
/// use (its exit STATUS is the child->parent protocol — crib the narrow allow on
/// `barrier_level_gate_subprocess_iox2_test`'s entrypoint, not a file-wide one).
#[allow(clippy::disallowed_methods)]
#[test]
fn regch_subprocess_child_entrypoint() {
    let config_path = match std::env::var(ENV_REGCH_CONFIG) {
        Ok(p) => p,
        Err(_) => return, // not the child invocation
    };
    let run = || -> Result<(), Box<dyn std::error::Error>> {
        let topic = std::env::var(ENV_REGCH_TOPIC)?;
        let stop = std::env::var(ENV_REGCH_STOP)?;
        let ix: iceoryx2::config::Config =
            serde_json::from_str(&std::fs::read_to_string(&config_path)?)?;
        let mgr = worker_manager("subproc", &ix);
        if !mgr.register_dynamic_egress_topic(&topic, 7)? {
            return Err("registration reported already-known on a fresh root".into());
        }
        // Keep the process (and its ONE control publisher) alive until the
        // parent has asserted, republishing so the record is drainable
        // whenever the parent drives. Bounded: ~30s then fail loudly.
        for _ in 0..600 {
            mgr.republish_dynamic_egress_for_test();
            if std::path::Path::new(&stop).exists() {
                if std::env::var(ENV_REGCH_LEAK).is_ok() {
                    // Production shape: the singleton is never dropped —
                    // exit with the manager (and its control publisher)
                    // still alive, so the port outlives the process.
                    std::mem::forget(mgr);
                }
                return Ok(());
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        Err("stop file never appeared within the bound".into())
    };
    match run() {
        Ok(()) => std::process::exit(0),
        Err(e) => {
            eprintln!("[regch subproc] FAILED: {e}");
            std::process::exit(2);
        }
    }
}

/// RAII kill+reap so a parent panic never orphans a republishing child.
struct RegchChildGuard {
    child: std::process::Child,
    reaped: bool,
}

impl Drop for RegchChildGuard {
    fn drop(&mut self) {
        if !self.reaped {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

#[test]
fn each_real_os_process_consumes_exactly_one_writer_slot() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("mp", &root);
    // Read-only probe manager: registers nothing, so it holds no control
    // publisher (the count probe opens only the port FACTORY — inert).
    let probe = worker_manager("probe", &root);
    assert_eq!(
        probe
            .dynamic_egress_control_publisher_count_for_test()
            .expect("baseline count"),
        0,
        "no process has registered yet — zero writer slots consumed"
    );

    let dir = std::env::temp_dir().join(format!("cer_regch_mp_{}", unique_id()));
    std::fs::create_dir_all(&dir).expect("mk temp dir");
    let config_path = dir.join("ix_config.json");
    std::fs::write(
        &config_path,
        serde_json::to_string(&root).expect("serialize iceoryx2 config"),
    )
    .expect("write config");
    let stop_path = dir.join("stop");

    let base = unique_id();
    let exe = std::env::current_exe().expect("current_exe");
    let topics: Vec<String> = (1..=2).map(|i| format!("/regch/proc{i}/{base}")).collect();
    let mut children: Vec<RegchChildGuard> = topics
        .iter()
        .map(|topic| {
            let child = std::process::Command::new(&exe)
                .args([
                    "--exact",
                    "regch_subprocess_child_entrypoint",
                    "--nocapture",
                ])
                .env(ENV_REGCH_CONFIG, &config_path)
                .env(ENV_REGCH_TOPIC, topic)
                .env(ENV_REGCH_STOP, &stop_path)
                .spawn()
                .expect("spawn child");
            RegchChildGuard {
                child,
                reaped: false,
            }
        })
        .collect();

    // Drive until BOTH child topics are served at the gateway (bounded ~30s).
    let mut expected = topics.clone();
    expected.sort();
    let mut served = false;
    for _ in 0..300 {
        gateway.drive_once().expect("drive_once");
        let mut got = registered_set(&gateway);
        got.retain(|t| expected.contains(t));
        if got.len() == expected.len() {
            served = true;
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    assert!(
        served,
        "both real-process registrations must reach the gateway; got {:?}",
        registered_set(&gateway)
    );

    // THE PIN: two live OS processes = EXACTLY two control publishers.
    assert_eq!(
        probe
            .dynamic_egress_control_publisher_count_for_test()
            .expect("live count"),
        2,
        "one writer slot per OS process while both children are alive"
    );

    // Release the children; both must exit cleanly (0 = registered + held).
    std::fs::write(&stop_path, b"done").expect("write stop file");
    for g in &mut children {
        let status = g.child.wait().expect("wait child");
        g.reaped = true;
        assert!(status.success(), "child exited {status:?}");
    }

    // Teardown pin: a slot frees WITH its process (bounded poll — iceoryx2
    // reclaims a cleanly-exited node's ports on the next service interaction).
    let mut drained = 0;
    for _ in 0..100 {
        drained = probe
            .dynamic_egress_control_publisher_count_for_test()
            .expect("drained count");
        if drained == 0 {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    assert_eq!(
        drained, 0,
        "writer slots drain to zero once the children DROP their managers and exit (manager-drop release; the production singleton shape is pinned by the leak test below)"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

/// The PRODUCTION lifecycle pin: a process whose manager is retained in the
/// `OnceLock` singleton (modelled by `mem::forget`) exits WITHOUT releasing
/// its writer slot — the port survives as stale state until iceoryx2's
/// dead-node cleanup, exactly as the module docs (and the drop-property doc
/// on `REG_CHANNEL_MAX_WRITERS`) state. One observation after the confirmed
/// exit suffices to prove staleness; how long cleanup takes is deliberately
/// NOT asserted (it is iceoryx2's business, and an interaction-triggered
/// sweep may reclaim at any later point).
#[test]
fn a_leaked_singleton_manager_leaves_its_slot_stale_past_process_exit() {
    let root = cerulion_core::testing::iceoryx_test_config();
    let mut gateway = gateway_on("mpl", &root);
    let probe = worker_manager("probe_l", &root);
    assert_eq!(
        probe
            .dynamic_egress_control_publisher_count_for_test()
            .expect("baseline count"),
        0,
        "no writer yet"
    );

    let dir = std::env::temp_dir().join(format!("cer_regch_leak_{}", unique_id()));
    std::fs::create_dir_all(&dir).expect("mk temp dir");
    let config_path = dir.join("ix_config.json");
    std::fs::write(
        &config_path,
        serde_json::to_string(&root).expect("serialize iceoryx2 config"),
    )
    .expect("write config");
    let stop_path = dir.join("stop");
    let topic = format!("/regch/leak/{}", unique_id());

    let exe = std::env::current_exe().expect("current_exe");
    let child = std::process::Command::new(&exe)
        .args([
            "--exact",
            "regch_subprocess_child_entrypoint",
            "--nocapture",
        ])
        .env(ENV_REGCH_CONFIG, &config_path)
        .env(ENV_REGCH_TOPIC, &topic)
        .env(ENV_REGCH_STOP, &stop_path)
        .env(ENV_REGCH_LEAK, "1")
        .spawn()
        .expect("spawn child");
    let mut guard = RegchChildGuard {
        child,
        reaped: false,
    };

    // Wait until the registration is SERVED (the child is provably up).
    let mut served = false;
    for _ in 0..300 {
        gateway.drive_once().expect("drive_once");
        if registered_set(&gateway).contains(&topic) {
            served = true;
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    assert!(served, "the leaking child's registration must be served");
    assert_eq!(
        probe
            .dynamic_egress_control_publisher_count_for_test()
            .expect("live count"),
        1,
        "one live writer while the child runs"
    );

    // Release the child; it forgets its manager and exits 0.
    std::fs::write(&stop_path, b"done").expect("write stop file");
    let status = guard.child.wait().expect("wait child");
    guard.reaped = true;
    assert!(status.success(), "child exited {status:?}");

    // THE PIN: the port SURVIVES the process — the count still reads 1 after
    // the exit is confirmed. (A manager-drop exit reads 0 here — that is the
    // sibling test — so this observation is the discriminator.)
    assert_eq!(
        probe
            .dynamic_egress_control_publisher_count_for_test()
            .expect("post-exit count"),
        1,
        "a leaked (singleton-shaped) manager's writer slot is STALE past process exit — release is a drop property, not an exit property"
    );

    let _ = std::fs::remove_dir_all(&dir);
}
