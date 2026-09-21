// SPDX-License-Identifier: AGPL-3.0-only
//! Cross-machine data-plane A/B DIAGNOSTIC harness (needs two machines, `#[ignore]`).
//!
//! A two-role, env-driven tool for answering ONE question on a real two-machine
//! link: **do a robot's frames actually reach a desk, and if not, where do they
//! stop?** It faithfully models the production desk↔robot zenoh 1.8 asymmetric
//! peer link (desk DIALS, robot LISTENS; scouting OFF) and isolates the DATA
//! route by flipping the robot gateway's egress flag ON directly — modelling
//! "demand already crossed via the query path" — so the demand plane is
//! held constant and only the data route varies.
//!
//! # Status — what this harness has measured (NOT a live defect)
//!
//! Two results, both first-party:
//!
//! 1. **This harness, a Mac desk ↔ an x86 robot-role machine (bare `session.put`
//!    egress): frames FLOW.** Desk received 894 (fresh link), 790 (re-cross
//!    after a netd-restart-equivalent disconnect/reconnect), 696 (robot
//!    scouting ON). The full mechanism — demand crossing via the query
//!    inversion, data delivery, and restart re-cross — works end to end on a
//!    homogeneous current-zenoh two-machine link.
//! 2. **A "frames=0 at the desk" symptom can come from a STALE robot binary**,
//!    not a transport defect. A robot rebuilt from the current source
//!    streams to the desk over this same bare-put path.
//!
//! So this file is NOT a reproduction of a live bug — it is the retained
//! DIAGNOSTIC for that class: when a desk sees no frames from a robot, run it
//! to localize the break (producer / SHM tap / wire route / re-inject) instead
//! of guessing. It also revalidates a real robot — point `HARNESS_PEER` at
//! the robot's network locator.
//!
//! The link shape it models is deliberate, not incidental: the wire-proven
//! zenoh 1.8 peer asymmetry
//! says a dialer's declarations do not reach the accepter, which is exactly why
//! the DEMAND plane needed the queryable inversion. Whether the desk's
//! `cerulion/{topic}` data-subscriber declaration crosses on any given link is
//! precisely what this harness measures rather than assumes — on the
//! link measured above it demonstrably did.
//!
//! # Roles (env `HARNESS_ROLE`)
//!
//! `robot` — run on the producing machine (accepter/producer/listener).
//! Producer P + gateway G share one SHM root; G LISTENS on
//! `HARNESS_LISTEN` (default `tcp/0.0.0.0:7683`), announces `HARNESS_TOPIC`,
//! egress flag forced ON. Publishes Vector3 at ~20 Hz and drives egress.
//!
//! `desk` — run on the consuming machine (dialer/consumer). CONNECTS to
//! `HARNESS_PEER` (required; the robot gateway locator, e.g. `tcp/<robot-ip>:7683`), `register_ingress_topic`
//! (declares the demand token + the `cerulion/{topic}` subscriber + re-inject
//! publisher), then counts frames RE-INJECTED (`ingress_stats().frames`) and
//! delivered to a local subscriber.
//!
//! # Reading the output
//!
//! The two counters are independent evidence and must be read as a pair:
//!
//! - robot `forwarded` — frames the gateway handed to zenoh. This is a
//!   ROBOT-LOCAL counter, **not** delivery evidence: a bare `session.put`
//!   succeeds locally even when the computed route is empty, so `forwarded`
//!   climbs whether or not anything is listening.
//! - desk `sub_received` / `ingress_stats` — frames that actually crossed.
//!
//! Both climbing ⇒ the data plane is healthy end to end. Desk `0` while the
//! robot's `forwarded` climbs ⇒ the break is on the wire/route (or in
//! re-inject), NOT in the producer or the SHM tap — that is the localization
//! this harness exists to provide.
//!
//! Run (on the robot machine):
//! `HARNESS_ROLE=robot cargo test -p cerulion_core --test cross_machine_data_plane_harness cross_machine_data_plane -- --ignored --nocapture`
//!
//! Run (on the desk machine):
//! `HARNESS_ROLE=desk HARNESS_PEER=tcp/<robot-ip>:7683 cargo test -p cerulion_core --test cross_machine_data_plane_harness cross_machine_data_plane -- --ignored --nocapture`

use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::gateway::{GatewayEgressPolicy, GatewayPlan, GatewayRuntime};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;

fn env_or(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

/// Scouting toggle: `HARNESS_SCOUT=on` enables multicast+gossip scouting (models
/// the production permissive gateway, which scouts the LAN); anything else keeps
/// scouting OFF (the faithful connect-only asymmetric-link model).
fn scout_on() -> bool {
    env_or("HARNESS_SCOUT", "off") == "on"
}

#[test]
#[ignore = "cross-machine harness; run explicitly with HARNESS_ROLE set"]
fn cross_machine_data_plane() {
    let role = env_or("HARNESS_ROLE", "");
    let topic = env_or("HARNESS_TOPIC", "/odom");
    let secs: u64 = env_or("HARNESS_SECS", "30").parse().expect("HARNESS_SECS");
    match role.as_str() {
        "robot" => run_robot(&topic, secs),
        "desk" => run_desk(&topic, secs),
        other => panic!(
            "set HARNESS_ROLE=robot (on the robot) or HARNESS_ROLE=desk (on the desk); got {other:?}"
        ),
    }
}

/// Accepter/producer/listener: producer P + gateway G on one SHM root, egress
/// flag forced ON, publishing Vector3 at ~20 Hz.
fn run_robot(topic: &str, secs: u64) {
    let listen = env_or("HARNESS_LISTEN", "tcp/0.0.0.0:7683");
    println!("[robot] listen={listen} topic={topic} secs={secs}");
    let root = cerulion_core::testing::iceoryx_test_config();

    let p = TransportManager::init_for_test(
        TransportConfig {
            node_name: "p".to_string(),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init P");

    let g = TransportManager::init_for_test(
        TransportConfig {
            node_name: "g".to_string(),
            network: Some(NetworkConfig {
                listen_endpoints: vec![listen.clone()],
                robot_identity: Some("bot".to_string()),
                multicast_scouting: scout_on(),
                gossip_scouting: scout_on(),
                ..Default::default()
            }),
            ..Default::default()
        },
        root.clone(),
    )
    .expect("init G");

    let mut publisher = p
        .create_publisher_simple(topic, MaxSliceLen::const_new(256))
        .expect("P publisher");

    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![topic.to_string()],
        ingress: vec![],
    };
    let mut gateway = GatewayRuntime::new(g, plan).expect("gateway boot (binds listen endpoint)");

    // Force the egress flag ON — models "demand crossed via the query
    // path, egress enabled". The ONLY remaining variable is the data route.
    gateway
        .manager()
        .bridge_manager()
        .enable_bridge(topic)
        .expect("enable egress bridge");
    println!("[robot] egress bridge enabled; gateway session listening. Publishing @20Hz...");

    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut n: u64 = 0;
    let mut last_print = Instant::now();
    while Instant::now() < deadline {
        {
            let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
            proxy.x = n as f64;
            proxy.y = (n % 100) as f64;
            proxy.z = -(n as f64);
        }
        n += 1;
        // drive_once attaches the tap (first ON pass) then drains+forwards.
        let _ = gateway.drive_once().expect("gateway drive_once");
        if last_print.elapsed() >= Duration::from_secs(1) {
            println!(
                "[robot] published={} forwarded[{}]={} taps={:?} attach_fail={}",
                n,
                topic,
                gateway.forwarded_count(topic),
                gateway.active_tap_topics(),
                gateway.attach_failure_count(topic),
            );
            last_print = Instant::now();
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    println!(
        "[robot] DONE published={} forwarded[{}]={} \
         (robot-local counter — a put succeeds locally even with an empty route, \
         so read this against the desk's sub_received)",
        n,
        topic,
        gateway.forwarded_count(topic),
    );
}

/// Dialer/consumer: connect to the robot, register the ingress mirror, count
/// re-injected + delivered frames.
fn run_desk(topic: &str, secs: u64) {
    let peer = std::env::var("HARNESS_PEER").unwrap_or_else(|_| {
        panic!("HARNESS_PEER must be set to the robot gateway locator, e.g. HARNESS_PEER=tcp/<robot-ip>:7683")
    });
    println!("[desk] connect={peer} topic={topic} secs={secs}");
    let root = cerulion_core::testing::iceoryx_test_config();

    let b = Arc::new(
        TransportManager::init_for_test(
            TransportConfig {
                node_name: "desk".to_string(),
                network: Some(NetworkConfig {
                    connect_endpoints: vec![peer.clone()],
                    multicast_scouting: scout_on(),
                    gossip_scouting: scout_on(),
                    ..Default::default()
                }),
                ..Default::default()
            },
            root,
        )
        .expect("init desk"),
    );

    let sub = b.create_subscriber(topic).expect("desk subscriber");
    // HARNESS_HASH (hex, e.g. 0x697ef87e3ead41a1) = the topic's REAL wire
    // schema hash so re-inject + local delivery is exercised; absent = the
    // Vector3 placeholder (mismatch-drop counting still proves wire delivery).
    // A PRESENT-but-unparseable value panics loudly (an operator typo is an
    // error, not a default); only an ABSENT var takes the placeholder.
    let expected_hash = match std::env::var("HARNESS_HASH") {
        Ok(v) => {
            let h = u64::from_str_radix(v.trim().trim_start_matches("0x"), 16)
                .unwrap_or_else(|e| panic!("HARNESS_HASH {v:?} is not parseable hex: {e}"));
            println!("[desk] expected schema hash 0x{h:016x} (from HARNESS_HASH)");
            h
        }
        Err(_) => {
            let h = Vector3::SCHEMA_HASH;
            println!(
                "[desk] expected schema hash 0x{h:016x} (placeholder — HARNESS_HASH unset; \
                 a real-schema topic will show schema_mismatch_drops, which still proves \
                 wire delivery)"
            );
            h
        }
    };
    b.register_ingress_topic(topic, expected_hash, MaxSliceLen::const_new(65536))
        .expect("register_ingress_topic");
    println!("[desk] ingress registered (demand token + cerulion{topic} subscriber declared).");

    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut received: u64 = 0;
    let mut last_x = f64::NAN;
    let mut last_print = Instant::now();
    while Instant::now() < deadline {
        sub.try_receive(|msg| {
            received += 1;
            let p = msg.payload();
            if p.len() >= 8 {
                last_x = f64::from_le_bytes(p[0..8].try_into().unwrap());
            }
        })
        .expect("desk try_receive");
        if last_print.elapsed() >= Duration::from_secs(1) {
            let stats = b.network().and_then(|nm| nm.ingress_stats(topic));
            println!("[desk] sub_received={received} last_x={last_x} ingress_stats={stats:?}");
            last_print = Instant::now();
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    let stats = b.network().and_then(|nm| nm.ingress_stats(topic));
    println!(
        "[desk] FINAL sub_received={received} ingress_stats={stats:?} \
         (>0 == the data plane is healthy end to end; 0 while the robot's \
         forwarded climbs localizes the break to the wire/route or re-inject)"
    );
}
