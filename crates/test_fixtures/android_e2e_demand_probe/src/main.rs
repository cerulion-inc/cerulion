// SPDX-License-Identifier: AGPL-3.0-only
//! `android-e2e-demand-probe` - the Android emulator e2e's direct data demand.
//!
//! `cerulion topic echo` reads a robot's catalog before it demands data, so a
//! paired app that refuses its catalog never receives a data demand from it.
//! This probe skips the catalog: it boots an ingress-only gateway for ONE topic
//! with the schema it is told to expect, connected to ONE endpoint with
//! scouting off, and lets the gateway's demand-GET loop demand the topic for a
//! bounded window. With `--listen` it listens on the endpoint and dials
//! nothing instead, so the only link is the one the app dials. It reports who
//! dialed the link (`dialer`, `probe` or `app`), the demand acks it counted and
//! the robots that
//! sent them (`ack_peers`, sorted and joined by `,`, empty when no ack came),
//! the acks from a peer it did not name because its peer set was full
//! (`ack_peer_overflow`) or the name was too long to store (`ack_peer_overlong`),
//! the frames that reached its ingress from the network (`wire_frames`,
//! whether re-injected or dropped by validation), each drop bucket by name
//! (`schema_mismatch_drops` when the schema hash was not the expected one,
//! `decode_errors`, `reinject_failure_drops`, `lan_refused_drops`), and the
//! frames its local subscriber received (`frames`). The probe's own LAN access is set to open, so its ingress never
//! drops a frame on its own posture.
//!
//! Usage: `android-e2e-demand-probe [--listen] <endpoint> <topic> <schema> <window-secs>`
//!
//! `<schema>` is `std_msgs/String` or `std_msgs/Int32`. It prints ONE fact line
//! and exits 0 when the window completed:
//!
//! ```text
//! demand_probe topic=/app/heartbeat schema=std_msgs/String window_s=10 dialer=probe passes=5 enabled_acks=0 refused_acks=5 wire_header_refused_acks=0 ack_peers=android-app ack_peer_overflow=0 ack_peer_overlong=0 schema_mismatch_drops=0 decode_errors=0 reinject_failure_drops=0 lan_refused_drops=0 wire_frames=0 frames=0
//! ```
//!
//! The verdict is the caller's (`tools/scripts/android_e2e_probe_verdict.sh`):
//! every ack must come from the app; a paired app must answer with refused
//! acks only and send no frame; an open one grants and delivers, and drops
//! every frame when the expected schema is not the topic's; an open app that
//! dialed the probe grants and delivers over that link. A bad argument or a
//! setup failure returns an error (exit code 1) and prints no fact line.

use std::error::Error;
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::demand_authorizer::LanAccess;
use cerulion_core::transport::gateway::{
    GatewayEgressPolicy, GatewayIngressEntry, GatewayPlan, GatewayRuntime,
};
use cerulion_core::transport::network::{IngressStats, NetworkConfig};
use cerulion_core::transport::{TransportConfig, TransportManager};
use native_ros2_messages::std_msgs::{Int32, String as RosString};

/// The identity this probe announces. It never serves anything.
const PROBE_IDENTITY: &str = "android-e2e-demand-probe";
/// How often the probe drains its subscriber during the window.
const POLL: Duration = Duration::from_millis(50);

/// The schemas the probe can expect, by name. The e2e demands the heartbeat
/// (`std_msgs/String`) under its own schema and under `std_msgs/Int32`, which it
/// does not carry.
const SCHEMAS: &[(&str, u64)] = &[
    ("std_msgs/String", RosString::SCHEMA_HASH),
    ("std_msgs/Int32", Int32::SCHEMA_HASH),
];

/// The probe's arguments: `--listen` and the four positional ones.
#[derive(Debug, PartialEq)]
struct Args {
    /// Listen on `endpoint` and dial nothing, instead of dialing it.
    listen: bool,
    endpoint: String,
    topic: String,
    schema: &'static str,
    schema_hash: u64,
    window: Duration,
}

fn parse_args(argv: impl Iterator<Item = String>) -> Result<Args, String> {
    let usage =
        "usage: android-e2e-demand-probe [--listen] <endpoint> <topic> <schema> <window-secs>";
    let mut argv = argv.peekable();
    let listen = argv.next_if(|arg| arg == "--listen").is_some();
    let (Some(endpoint), Some(topic), Some(schema), Some(window), None) = (
        argv.next(),
        argv.next(),
        argv.next(),
        argv.next(),
        argv.next(),
    ) else {
        return Err(usage.to_string());
    };
    if !topic.starts_with('/') {
        return Err(format!("topic '{topic}' is not canonical (no leading '/')"));
    }
    let Some(&(schema, schema_hash)) = SCHEMAS.iter().find(|(name, _)| *name == schema) else {
        return Err(format!(
            "schema '{schema}' is not one this probe knows (std_msgs/String, std_msgs/Int32)"
        ));
    };
    let secs: u64 = window
        .parse()
        .map_err(|_| format!("window '{window}' is not a whole number of seconds"))?;
    if secs == 0 {
        return Err("window must be at least one second".to_string());
    }
    Ok(Args {
        listen,
        endpoint,
        topic,
        schema,
        schema_hash,
        window: Duration::from_secs(secs),
    })
}

// This binary's stdout IS its report: the e2e parses the one fact line.
#[allow(clippy::print_stdout)]
fn main() -> Result<(), Box<dyn Error>> {
    let args = parse_args(std::env::args().skip(1))?;
    let manager = TransportManager::init(TransportConfig {
        node_name: "android_e2e_demand_probe".to_string(),
        network: Some(NetworkConfig {
            connect_endpoints: if args.listen {
                vec![]
            } else {
                vec![args.endpoint.clone()]
            },
            listen_endpoints: if args.listen {
                vec![args.endpoint.clone()]
            } else {
                vec![]
            },
            robot_identity: Some(PROBE_IDENTITY.to_string()),
            ..Default::default()
        }),
        ..Default::default()
    })?;
    let network = manager
        .network()
        .ok_or("the probe's transport has no network")?;
    network.set_lan_access(LanAccess::Open);
    // The zenoh session opens lazily, on its first use. Open it here, so an
    // endpoint it cannot take fails with this message before the probe
    // builds anything on it.
    network
        .session()
        .map_err(|error| format!("the probe's session did not open: {error}"))?;
    let subscriber = manager.create_subscriber(&args.topic)?;
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec![]),
        announce: vec![],
        ingress: vec![GatewayIngressEntry {
            topic: args.topic.clone(),
            schema_hash: args.schema_hash,
        }],
    };
    let gateway = GatewayRuntime::new(manager, plan)?;

    let deadline = Instant::now() + args.window;
    let mut frames = 0usize;
    while Instant::now() < deadline {
        frames += subscriber.try_receive(|_| {})?;
        std::thread::sleep(POLL);
    }
    frames += subscriber.try_receive(|_| {})?;
    // Every frame the ingress callback received lands in exactly one of these
    // five counters, so their sum is every frame that crossed the network. The
    // pattern names every field with no `..`: a counter added to
    // `IngressStats` stops this build until the sum counts it.
    let IngressStats {
        frames: reinjected,
        schema_mismatch_drops,
        decode_errors,
        reinject_failure_drops,
        lan_refused_drops,
    } = gateway
        .manager()
        .network()
        .and_then(|network| network.ingress_stats(&args.topic))
        .ok_or("the probe's ingress bridge is not registered")?;
    let wire_frames = reinjected
        + schema_mismatch_drops
        + decode_errors
        + reinject_failure_drops
        + lan_refused_drops;

    println!(
        "demand_probe topic={} schema={} window_s={} dialer={} passes={} enabled_acks={} \
         refused_acks={} \
         wire_header_refused_acks={} ack_peers={} ack_peer_overflow={} ack_peer_overlong={} \
         schema_mismatch_drops={schema_mismatch_drops} \
         decode_errors={decode_errors} reinject_failure_drops={reinject_failure_drops} \
         lan_refused_drops={lan_refused_drops} wire_frames={wire_frames} frames={frames}",
        args.topic,
        args.schema,
        args.window.as_secs(),
        if args.listen { "app" } else { "probe" },
        gateway.demand_get_pass_count(),
        gateway.demand_enabled_ack_count(),
        gateway.demand_refused_ack_count(),
        gateway.demand_wire_header_refused_ack_count(),
        gateway.demand_ack_peers().join(","),
        gateway.demand_ack_peer_overflow_count(),
        gateway.demand_ack_peer_overlong_count(),
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Result<Args, String> {
        parse_args(args.iter().map(|arg| arg.to_string()))
    }

    #[test]
    fn the_probe_dials_its_endpoint_unless_told_to_listen() {
        let dial = parse(&[
            "tcp/127.0.0.1:7683",
            "/app/heartbeat",
            "std_msgs/String",
            "10",
        ])
        .expect("dial arguments");
        assert!(!dial.listen);
        assert_eq!(dial.endpoint, "tcp/127.0.0.1:7683");
        let listen = parse(&[
            "--listen",
            "tcp/127.0.0.1:7690",
            "/app/heartbeat",
            "std_msgs/Int32",
            "15",
        ])
        .expect("listen arguments");
        assert_eq!(
            listen,
            Args {
                listen: true,
                endpoint: "tcp/127.0.0.1:7690".to_string(),
                topic: "/app/heartbeat".to_string(),
                schema: "std_msgs/Int32",
                schema_hash: Int32::SCHEMA_HASH,
                window: Duration::from_secs(15),
            }
        );
    }

    #[test]
    fn listen_is_a_leading_flag_and_takes_the_four_positional_arguments() {
        let usage =
            "usage: android-e2e-demand-probe [--listen] <endpoint> <topic> <schema> <window-secs>";
        for args in [
            &["--listen"][..],
            &[
                "--listen",
                "tcp/127.0.0.1:7690",
                "/app/heartbeat",
                "std_msgs/String",
            ],
            &[
                "--listen",
                "tcp/127.0.0.1:7690",
                "/app/heartbeat",
                "std_msgs/String",
                "10",
                "extra",
            ],
        ] {
            assert_eq!(parse(args).unwrap_err(), usage, "{args:?}");
        }
        // Anywhere but first, `--listen` is a positional argument: here the
        // window, which is not a number.
        assert_eq!(
            parse(&[
                "tcp/127.0.0.1:7683",
                "/app/heartbeat",
                "std_msgs/String",
                "--listen"
            ])
            .unwrap_err(),
            "window '--listen' is not a whole number of seconds"
        );
    }
}
