// SPDX-License-Identifier: AGPL-3.0-only
//! `android-e2e-demand-probe` - the Android emulator e2e's direct data demand.
//!
//! `cerulion topic echo` reads a robot's catalog before it demands data, so a
//! paired app that refuses its catalog never receives a data demand from it.
//! This probe skips the catalog: it boots an ingress-only gateway for ONE topic
//! with a known schema (`std_msgs/String`), connected to ONE endpoint with
//! scouting off, and lets the gateway's demand-GET loop demand the topic for a
//! bounded window. It reports the frames that reached its ingress from the
//! network (`wire_frames`, whether re-injected or dropped by validation) and the
//! frames its local subscriber received (`frames`). The probe's own LAN access is
//! set to open, so its ingress never drops a frame on its own posture.
//!
//! Usage: `android-e2e-demand-probe <connect-endpoint> <topic> <window-secs>`
//!
//! It prints ONE fact line and exits 0 when the window completed:
//!
//! ```text
//! demand_probe topic=/app/heartbeat window_s=10 passes=5 enabled_acks=0 refused_acks=5 wire_header_refused_acks=0 wire_frames=0 frames=0
//! ```
//!
//! The verdict is the caller's: a paired robot must answer with refused acks
//! only and send no frame; an open one grants and delivers. Setup failures
//! return an error (exit code 1).

use std::error::Error;
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::demand_authorizer::LanAccess;
use cerulion_core::transport::gateway::{
    GatewayEgressPolicy, GatewayIngressEntry, GatewayPlan, GatewayRuntime,
};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};
use native_ros2_messages::std_msgs::String as RosString;

/// The identity this probe announces. It never serves anything.
const PROBE_IDENTITY: &str = "android-e2e-demand-probe";
/// How often the probe drains its subscriber during the window.
const POLL: Duration = Duration::from_millis(50);

/// The probe's three positional arguments.
struct Args {
    endpoint: String,
    topic: String,
    window: Duration,
}

fn parse_args(mut argv: impl Iterator<Item = String>) -> Result<Args, String> {
    let usage = "usage: android-e2e-demand-probe <connect-endpoint> <topic> <window-secs>";
    let (Some(endpoint), Some(topic), Some(window), None) =
        (argv.next(), argv.next(), argv.next(), argv.next())
    else {
        return Err(usage.to_string());
    };
    if !topic.starts_with('/') {
        return Err(format!("topic '{topic}' is not canonical (no leading '/')"));
    }
    let secs: u64 = window
        .parse()
        .map_err(|_| format!("window '{window}' is not a whole number of seconds"))?;
    if secs == 0 {
        return Err("window must be at least one second".to_string());
    }
    Ok(Args {
        endpoint,
        topic,
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
            connect_endpoints: vec![args.endpoint.clone()],
            robot_identity: Some(PROBE_IDENTITY.to_string()),
            ..Default::default()
        }),
        ..Default::default()
    })?;
    manager
        .network()
        .ok_or("the probe's transport has no network")?
        .set_lan_access(LanAccess::Open);
    let subscriber = manager.create_subscriber(&args.topic)?;
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowList(vec![]),
        announce: vec![],
        ingress: vec![GatewayIngressEntry {
            topic: args.topic.clone(),
            schema_hash: RosString::SCHEMA_HASH,
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
    // five counters, so their sum is every frame that crossed the network.
    let ingress = gateway
        .manager()
        .network()
        .and_then(|network| network.ingress_stats(&args.topic))
        .ok_or("the probe's ingress bridge is not registered")?;
    let wire_frames = ingress.frames
        + ingress.schema_mismatch_drops
        + ingress.decode_errors
        + ingress.reinject_failure_drops
        + ingress.lan_refused_drops;

    println!(
        "demand_probe topic={} window_s={} passes={} enabled_acks={} refused_acks={} \
         wire_header_refused_acks={} wire_frames={wire_frames} frames={frames}",
        args.topic,
        args.window.as_secs(),
        gateway.demand_get_pass_count(),
        gateway.demand_enabled_ack_count(),
        gateway.demand_refused_ack_count(),
        gateway.demand_wire_header_refused_ack_count(),
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Result<Args, String> {
        parse_args(args.iter().map(|s| (*s).to_string()))
    }

    #[test]
    fn three_arguments_parse_into_endpoint_topic_and_window() {
        let args = parse(&["tcp/127.0.0.1:7683", "/app/heartbeat", "10"]).expect("parses");
        assert_eq!(args.endpoint, "tcp/127.0.0.1:7683");
        assert_eq!(args.topic, "/app/heartbeat");
        assert_eq!(args.window, Duration::from_secs(10));
    }

    #[test]
    fn a_missing_or_extra_argument_is_the_usage_error() {
        let usage = "usage: android-e2e-demand-probe <connect-endpoint> <topic> <window-secs>";
        assert_eq!(
            parse(&["tcp/127.0.0.1:7683", "/t"]).err().as_deref(),
            Some(usage)
        );
        assert_eq!(
            parse(&["tcp/127.0.0.1:7683", "/t", "1", "x"])
                .err()
                .as_deref(),
            Some(usage)
        );
    }

    #[test]
    fn a_relative_topic_is_refused() {
        assert_eq!(
            parse(&["tcp/127.0.0.1:7683", "app/heartbeat", "1"])
                .err()
                .as_deref(),
            Some("topic 'app/heartbeat' is not canonical (no leading '/')")
        );
    }

    #[test]
    fn a_zero_or_non_numeric_window_is_refused() {
        assert_eq!(
            parse(&["tcp/127.0.0.1:7683", "/t", "0"]).err().as_deref(),
            Some("window must be at least one second")
        );
        assert_eq!(
            parse(&["tcp/127.0.0.1:7683", "/t", "ten"]).err().as_deref(),
            Some("window 'ten' is not a whole number of seconds")
        );
    }
}
