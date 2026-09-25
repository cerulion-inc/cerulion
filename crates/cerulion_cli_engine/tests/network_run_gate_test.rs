// SPDX-License-Identifier: AGPL-3.0-only
//! PURE tests for the `graph run` network DECISION —
//! [`resolve_run_network`] (the single funnel from a graph's `network:` block
//! or the permissive default to the separate GATEWAY process). Networked
//! multi-process is first-class, so run SHAPE is not an input to the
//! decision.
//!
//! No iceoryx2, no zenoh, no transport. The tests DO read/write process env
//! (the `CERULION_NETWORK` / `CERULION_GATEWAY_PORT` knobs `resolve_run_network`
//! consults), so a file-local `env_lock()` mutex serializes every test in this
//! binary — `resolve_run_network` reads `CERULION_NETWORK` on EVERY call, so an
//! unlocked env-mutating test could poison a concurrent decision.
//!
//! The wired build behavior (egress watch, ingress registration, the gateway
//! e2e) lives in `cerulion_core`'s gateway/network tests.

use std::sync::{Mutex, MutexGuard, OnceLock};

use cerulion_cli_engine::graph_cmd::{
    network_env_kill, remote_network_suppressed, resolve_gateway_base_port, resolve_robot_identity,
    resolve_run_network, NetworkEnvKill, RunNetwork, TimeSource, GATEWAY_WELL_KNOWN_PORT,
    NETWORK_OFF_LOCAL_ONLY_NOTICE, NETWORK_OFF_REDUNDANT_NOTICE,
};
use cerulion_core::graph::config::{GraphConfig, NetworkBlock, NetworkMode, NodeDef};
use cerulion_core::transport::network::ZenohMode;
use tracing_test::traced_test;

/// Serialize every test in this binary — they read/write the process env the
/// decision consults. Poison-tolerant (a panicking test must not wedge the rest).
fn env_lock() -> MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    LOCK.get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|e| e.into_inner())
}

/// RAII: remove an env var on drop so a set-in-test knob never leaks.
struct EnvGuard(&'static str);
impl Drop for EnvGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.0);
    }
}

/// One-node graph carrying the given `network:` block.
fn graph_with(network: Option<NetworkBlock>) -> GraphConfig {
    GraphConfig {
        execution: None,
        name: None,
        identity: "netgate".to_string(),
        prefix: "ng".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "n".to_string(),
            node_type: "t".to_string(),
            inputs: vec![],
            outputs: vec![],
        }],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Vec::new(),
        level_assignments: None,
        network,
    }
}

/// An enabled block with BOTH egress and ingress declared.
fn block_with_ingress() -> NetworkBlock {
    NetworkBlock {
        mode: NetworkMode::Peer,
        connect: vec!["tcp/192.168.123.99:7447".to_string()],
        listen: vec!["tcp/0.0.0.0:7447".to_string()],
        egress: vec!["/ng/n/out".to_string()],
        ingress: vec!["/remote/cmd".to_string()],
    }
}

/// An enabled EGRESS-ONLY block (no ingress).
fn egress_only_block() -> NetworkBlock {
    NetworkBlock {
        mode: NetworkMode::Peer,
        connect: vec![],
        listen: vec!["tcp/0.0.0.0:7447".to_string()],
        egress: vec!["/ng/n/out".to_string()],
        ingress: vec![],
    }
}

// ==========================================================================
// Kill switch (`--network off` + the `CERULION_NETWORK=off` env knob).
// ==========================================================================

/// `--network off` + enabled block ⇒ `Off` AND the loud kill-switch warn.
#[test]
#[traced_test]
fn network_off_forces_off_and_warns() {
    let _lock = env_lock();
    let config = graph_with(Some(block_with_ingress()));
    let decision = resolve_run_network(&config, true, TimeSource::Real, false).unwrap();
    assert!(matches!(decision, RunNetwork::Off));
    assert!(logs_contain("network disabled by --network off"));
    assert!(logs_contain("will NOT cross the machine boundary"));
}

/// `--network off` + no block + REAL clock ⇒
/// `Off` + the LOCAL-ONLY warn CONFIRMING the kill-switch acted (it suppressed
/// the permissive default — the flag is NOT inert anymore).
#[test]
#[traced_test]
fn network_off_on_blockless_real_clock_confirms_suppression() {
    let _lock = env_lock();
    let decision = resolve_run_network(&graph_with(None), true, TimeSource::Real, false).unwrap();
    assert!(matches!(decision, RunNetwork::Off));
    assert!(
        logs_contain(NETWORK_OFF_LOCAL_ONLY_NOTICE),
        "the blockless real-clock kill-switch must confirm it SUPPRESSED the permissive default"
    );
    assert!(
        !logs_contain("redundant"),
        "the real-clock arm must not claim the flag was redundant"
    );
}

/// The GENUINELY redundant arm: `--network off` + no block + a replay-class
/// clock. The run is network-inert either way, and
/// the info notice says exactly that.
#[test]
#[traced_test]
fn network_off_on_virtual_clock_is_genuinely_redundant() {
    let _lock = env_lock();
    let decision =
        resolve_run_network(&graph_with(None), true, TimeSource::Virtual, false).unwrap();
    assert!(matches!(decision, RunNetwork::Off));
    assert!(
        logs_contain(NETWORK_OFF_REDUNDANT_NOTICE),
        "the replay-class arm must state the flag is redundant (network-inert by design)"
    );
    assert!(
        !logs_contain(NETWORK_OFF_LOCAL_ONLY_NOTICE),
        "the replay-class arm must not claim the flag suppressed the permissive default"
    );
}

/// The `CERULION_NETWORK=off` env knob == `--network off` for
/// engine-level callers that cannot pass the flag.
#[test]
fn env_knob_off_forces_off() {
    let _lock = env_lock();
    std::env::set_var("CERULION_NETWORK", "off");
    let _g = EnvGuard("CERULION_NETWORK");
    // Flag ABSENT + real clock + no block would be Permissive; the env knob
    // overrides to Off.
    let decision = resolve_run_network(&graph_with(None), false, TimeSource::Real, false).unwrap();
    assert!(
        matches!(decision, RunNetwork::Off),
        "CERULION_NETWORK=off must force Off even with the flag absent"
    );
}

/// The env knob is trimmed + case-insensitive:
/// "OFF", "Off", " off " all engage the kill-switch (no fail-open on casing).
#[test]
fn env_knob_off_is_trimmed_and_case_insensitive() {
    let _lock = env_lock();
    for v in ["OFF", "Off", " off ", "\toFF\t"] {
        std::env::set_var("CERULION_NETWORK", v);
        let _g = EnvGuard("CERULION_NETWORK");
        let decision =
            resolve_run_network(&graph_with(None), false, TimeSource::Real, false).unwrap();
        assert!(
            matches!(decision, RunNetwork::Off),
            "CERULION_NETWORK={v:?} must engage the kill-switch (trim + case-insensitive)"
        );
    }
}

/// Any OTHER non-empty env value FAILS CLOSED:
/// a loud warn naming the accepted value, then treated as `off` (an operator
/// who set the kill-switch env at all intended local-only; a typo must never
/// fail open to permissive).
#[test]
#[traced_test]
fn env_knob_unrecognized_value_fails_closed_with_warn() {
    let _lock = env_lock();
    std::env::set_var("CERULION_NETWORK", "on");
    let _g = EnvGuard("CERULION_NETWORK");
    let decision = resolve_run_network(&graph_with(None), false, TimeSource::Real, false).unwrap();
    assert!(
        matches!(decision, RunNetwork::Off),
        "an unrecognized CERULION_NETWORK value must FAIL CLOSED (treated as off)"
    );
    assert!(
        logs_contain("CERULION_NETWORK is set to an unrecognized value")
            && logs_contain("the only accepted value is `off`"),
        "the fail-closed arm must warn loudly, naming the accepted value"
    );
}

/// An EMPTY env value is treated as unset:
/// silent, no kill-switch (the conventional "unset" idiom).
#[test]
fn env_knob_empty_value_is_unset() {
    let _lock = env_lock();
    std::env::set_var("CERULION_NETWORK", "");
    let _g = EnvGuard("CERULION_NETWORK");
    let decision = resolve_run_network(&graph_with(None), false, TimeSource::Real, false).unwrap();
    assert!(
        matches!(decision, RunNetwork::Permissive(_)),
        "an empty CERULION_NETWORK must behave as unset (permissive default stands)"
    );
}

// ==========================================================================
// Replay-class clocks ⇒ Inert (silent).
// ==========================================================================

#[test]
fn virtual_clock_is_inert_with_block() {
    let _lock = env_lock();
    let config = graph_with(Some(block_with_ingress()));
    let decision = resolve_run_network(&config, false, TimeSource::Virtual, false).unwrap();
    assert!(matches!(decision, RunNetwork::Inert));
}

#[test]
fn virtual_clock_is_inert_without_block() {
    let _lock = env_lock();
    let decision =
        resolve_run_network(&graph_with(None), false, TimeSource::Virtual, false).unwrap();
    assert!(matches!(decision, RunNetwork::Inert));
}

#[test]
fn external_clock_is_inert() {
    let _lock = env_lock();
    let decision =
        resolve_run_network(&graph_with(None), false, TimeSource::External, false).unwrap();
    assert!(matches!(decision, RunNetwork::Inert));
}

// ==========================================================================
// Real clock + explicit enabled block ⇒ Strict verbatim.
// ==========================================================================

#[test]
fn enabled_block_real_clock_is_strict_verbatim() {
    let _lock = env_lock();
    let config = graph_with(Some(block_with_ingress()));
    let RunNetwork::Strict(net) =
        resolve_run_network(&config, false, TimeSource::Real, false).unwrap()
    else {
        panic!("an enabled block under the real clock must be Strict");
    };
    assert_eq!(net.mode, ZenohMode::Peer);
    assert_eq!(
        net.connect_endpoints,
        vec!["tcp/192.168.123.99:7447".to_string()]
    );
    assert_eq!(net.listen_endpoints, vec!["tcp/0.0.0.0:7447".to_string()]);
    // Scouting stays OFF for an explicit block (the transport default).
    assert!(!net.multicast_scouting && !net.gossip_scouting);
}

/// `--record` KEEPS the network: an EGRESS-ONLY block records fine
/// (Strict, networked, announces + egress stay ON).
#[test]
fn record_egress_only_block_is_strict() {
    let _lock = env_lock();
    let config = graph_with(Some(egress_only_block()));
    let decision = resolve_run_network(&config, false, TimeSource::Real, true).unwrap();
    assert!(
        matches!(decision, RunNetwork::Strict(_)),
        "record + an egress-only network block must stay networked (Strict)"
    );
}

/// `--record` + a block that declares INGRESS is REFUSED loudly,
/// naming the topic + BOTH workarounds.
#[test]
fn record_ingress_block_is_refused() {
    let _lock = env_lock();
    let config = graph_with(Some(block_with_ingress()));
    let err = resolve_run_network(&config, false, TimeSource::Real, true)
        .expect_err("record + declared ingress must be refused")
        .to_string();
    assert!(
        err.contains("netgate")
            && err.contains("/remote/cmd")
            && err.contains("--record")
            && err.contains("--network off")
            && err.contains("not yet replay-faithful"),
        "the refusal must name the graph, the ingress topic, both workarounds, and the cause; \
         got: {err}"
    );
}

// ==========================================================================
// Real clock + no/disabled block ⇒ Permissive (synth config).
// ==========================================================================

#[test]
fn no_block_real_clock_is_permissive_synth() {
    let _lock = env_lock();
    let RunNetwork::Permissive(net) =
        resolve_run_network(&graph_with(None), false, TimeSource::Real, false).unwrap()
    else {
        panic!("a no-block real-clock run must be Permissive");
    };
    assert_eq!(net.mode, ZenohMode::Peer);
    assert!(
        net.multicast_scouting && net.gossip_scouting,
        "the permissive synth must enable scouting (unpaired robots LAN-discoverable)"
    );
    assert!(
        net.connect_endpoints.is_empty(),
        "the permissive synth has no connect endpoints"
    );
    assert_eq!(
        net.listen_endpoints,
        vec![format!("tcp/[::]:{GATEWAY_WELL_KNOWN_PORT}")],
        "the permissive synth listens on the well-known gateway port"
    );
}

/// `--record` + no block ⇒ Permissive (permissive has no ingress, so
/// the record + no-block combination is always fine).
#[test]
fn record_no_block_is_permissive() {
    let _lock = env_lock();
    let decision = resolve_run_network(&graph_with(None), false, TimeSource::Real, true).unwrap();
    assert!(matches!(decision, RunNetwork::Permissive(_)));
}

/// A `mode: disabled` block behaves like no block ⇒ Permissive.
#[test]
fn disabled_block_is_permissive() {
    let _lock = env_lock();
    let block = NetworkBlock {
        mode: NetworkMode::Disabled,
        ..NetworkBlock::default()
    };
    let decision =
        resolve_run_network(&graph_with(Some(block)), false, TimeSource::Real, false).unwrap();
    assert!(matches!(decision, RunNetwork::Permissive(_)));
}

// ==========================================================================
// Run SHAPE is NOT an input (networked multi-process is never rejected).
// ==========================================================================

/// A `process_groups:` graph with an enabled block under the real clock is
/// NOT refused (networked multi-process is first-class) — the decision is the
/// SAME Strict a monolith would get.
#[test]
fn enabled_network_plus_process_groups_is_strict_not_refused() {
    let _lock = env_lock();
    let mut config = graph_with(Some(egress_only_block()));
    config.process_groups = [
        ("g0".to_string(), vec!["n".to_string()]),
        ("g1".to_string(), vec![]),
    ]
    .into_iter()
    .collect();
    let decision = resolve_run_network(&config, false, TimeSource::Real, false).unwrap();
    assert!(
        matches!(decision, RunNetwork::Strict(_)),
        "networked multi-process is first-class — the block must NOT be refused"
    );
}

// ==========================================================================
// Entry-point contracts (decision: no entry-point gating).
// ==========================================================================

/// `node run`'s temp single-node graph — the EXACT shape `node run`
/// builds (one node, `standalone` prefix, NO `network:` block) — is Permissive
/// under the real clock. `node run` resolves through the SAME decision fn as
/// `graph run` (its `--network off` clap flag maps to the same `network_off`
/// input), so this row is the node-run permissive pin.
#[test]
fn node_run_temp_graph_shape_is_permissive() {
    let _lock = env_lock();
    let config = GraphConfig {
        execution: None,
        name: None,
        identity: "__temp_talker".to_string(),
        prefix: "standalone".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "talker".to_string(),
            node_type: "talker".to_string(),
            inputs: vec![],
            outputs: vec![],
        }],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Vec::new(),
        level_assignments: None,
        network: None,
    };
    let decision = resolve_run_network(&config, false, TimeSource::Real, false).unwrap();
    assert!(
        matches!(decision, RunNetwork::Permissive(_)),
        "node run's temp graph must ride the permissive default (no entry-point gating)"
    );
    // And its kill-switch (`node run --network off`) suppresses it.
    let off = resolve_run_network(&config, true, TimeSource::Real, false).unwrap();
    assert!(matches!(off, RunNetwork::Off));
}

/// `ros2 attach`'s bridge graph (the FLAGSHIP automagic surface — a
/// generated graph, no `network:` block, real clock, single-process monolith)
/// is Permissive: attach a robot and it is network-viewable with zero config.
/// Run shape is not an input, so the monolith-forced attach run resolves
/// identically to a direct `graph run`.
#[test]
fn ros_attach_bridge_graph_shape_is_permissive() {
    let _lock = env_lock();
    let mut config = graph_with(None);
    config.identity = "go2_bridge".to_string();
    config.nodes.push(NodeDef {
        fuse: None,
        ros2: None,
        id: "viz".to_string(),
        node_type: "cerulion_viz".to_string(),
        inputs: vec![],
        outputs: vec![],
    });
    let decision = resolve_run_network(&config, false, TimeSource::Real, false).unwrap();
    assert!(
        matches!(decision, RunNetwork::Permissive(_)),
        "ros2 attach's bridge graph must ride the permissive default"
    );
}

// NOTE (`graph profile` local-only): profile is the one deliberate exception —
// a bounded MEASUREMENT run that never calls `resolve_run_network`; its
// enforcement point is the hardcoded `network: None` on the profiler's own
// `TransportManager::init` (documented at both sites). There is no pure seam
// to pin that from here without a live profiling run; the profile e2e suite
// (`graph_profile_iox2_test.rs`, serial iceoryx2) is where a behavioral pin
// would live.

// ==========================================================================
// The POSTURE pins, both directions.
//
// These are what make the unknown-key refusal load-bearing rather than cosmetic. Every other
// arm in this file builds its `GraphConfig` IN MEMORY, so none of them can
// see the failure: the block never reaches `resolve_run_network` at all
// when a misspelled top-level key leaves `network: None` at PARSE time and
// the decision then answers with its no-block default.
// ==========================================================================

/// FAIL-OPEN. `netwrok:` must never resolve to `RunNetwork::Permissive`.
///
/// If it did, that would be the sharp end of the failure: the block vanishes,
/// the robot falls to the permissive default — every produced topic announced
/// to the LAN, multicast and gossip scouting on, listening on the gateway
/// port — and the only line printed insists there is "no `network:` block",
/// which reads as normal to anyone not already suspecting a typo and reads as
/// WRONG to the author looking straight at the block they wrote.
///
/// The pin is on the PARSE, because that is where the refusal lives: a document that
/// cannot be parsed never reaches a posture decision at all.
#[test]
fn a_misspelled_network_key_never_resolves_to_permissive() {
    let typo = r#"
prefix: ng
nodes:
  - id: n
    type: t
netwrok:
  mode: peer
  connect:
    - tcp/192.168.123.99:7447
"#;
    let err = cerulion_core::graph::parse_graph_raw(typo)
        .expect_err("a misspelled `network:` key must be REFUSED, not silently dropped")
        .to_string();
    assert!(
        err.contains("netwrok"),
        "the refusal must NAME the offending key; got: {err}"
    );
}

/// ANTI-TAUTOLOGY for the arm above, and the half that proves the posture
/// machinery still works: the CORRECT spelling parses AND tightens to Strict.
/// Without this, "a typo is refused" is satisfied by a parser that refuses
/// every document, and the pin would say nothing about the posture at all.
#[test]
fn the_correct_network_key_parses_and_tightens_to_strict() {
    let _lock = env_lock();
    let good = r#"
prefix: ng
nodes:
  - id: n
    type: t
network:
  mode: peer
  connect:
    - tcp/192.168.123.99:7447
"#;
    let config = cerulion_core::graph::parse_graph_raw(good).expect("the correct spelling parses");
    assert!(config.network.is_some(), "the block survived the parse");

    let decision = resolve_run_network(&config, false, TimeSource::Real, false).unwrap();
    assert!(
        matches!(decision, RunNetwork::Strict(_)),
        "an enabled block must TIGHTEN to Strict — the whole point of writing one"
    );
}

/// FAIL-CLOSED. `egres:` inside an otherwise-valid `mode: peer` block must
/// never resolve to a silent deny-all.
///
/// An accepted typo leaves `egress` at its `#[serde(default)]` empty, so the gateway
/// installs an `AllowList([])` with an empty announce set and the graph
/// exports NOTHING — in total silence, with every instinct pointing at the
/// network rather than at a one-letter YAML error. Rule 5 cannot catch it
/// (it fires only when `mode == Disabled`) and the egress loop iterates an
/// empty list.
#[test]
fn a_misspelled_egress_key_never_resolves_to_a_silent_deny_all() {
    let typo = r#"
prefix: ng
nodes:
  - id: n
    type: t
    outputs:
      - name: out
        schema: geometry_msgs/Vector3
network:
  mode: peer
  listen:
    - tcp/0.0.0.0:7447
  egres:
    - /ng/n/out
"#;
    let err = cerulion_core::graph::parse_graph_raw(typo)
        .expect_err("a misspelled `egress:` key must be REFUSED, not silently defaulted")
        .to_string();
    assert!(
        err.contains("egres"),
        "the refusal must NAME the offending key; got: {err}"
    );

    // ANTI-TAUTOLOGY, in the same body so the pair cannot drift apart: the
    // CORRECT spelling parses and the topic really lands in the allow-list.
    let good = typo.replace("egres:", "egress:");
    let config = cerulion_core::graph::parse_graph_raw(&good).expect("the correct spelling parses");
    let net = config.network.as_ref().expect("block present");
    assert_eq!(
        net.egress,
        vec!["/ng/n/out".to_string()],
        "the declared egress topic must survive the parse"
    );
}

// ==========================================================================
// Accessors + the port env override.
// ==========================================================================

#[test]
fn runnetwork_accessors() {
    let _lock = env_lock();
    let strict = resolve_run_network(
        &graph_with(Some(egress_only_block())),
        false,
        TimeSource::Real,
        false,
    )
    .unwrap();
    assert_eq!(strict.posture(), cerulion_core::NetworkPosture::Strict);
    assert!(strict.network_config().is_some());
    assert!(!strict.is_permissive());

    let permissive =
        resolve_run_network(&graph_with(None), false, TimeSource::Real, false).unwrap();
    assert_eq!(
        permissive.posture(),
        cerulion_core::NetworkPosture::PermissiveDefault
    );
    assert!(permissive.is_permissive());
    assert!(permissive.network_config().is_some());

    let off = resolve_run_network(&graph_with(None), true, TimeSource::Real, false).unwrap();
    assert!(off.network_config().is_none());
    assert!(!off.is_permissive());
}

/// The permissive base port: default well-known, env override parsed, garbage
/// rejected loudly.
#[test]
fn gateway_base_port_env_override_parses_strictly() {
    let _lock = env_lock();
    // Unset ⇒ the well-known default.
    std::env::remove_var("CERULION_GATEWAY_PORT");
    assert_eq!(
        resolve_gateway_base_port().unwrap(),
        GATEWAY_WELL_KNOWN_PORT
    );

    // A valid override wins.
    std::env::set_var("CERULION_GATEWAY_PORT", "9123");
    let _g = EnvGuard("CERULION_GATEWAY_PORT");
    assert_eq!(resolve_gateway_base_port().unwrap(), 9123);

    // Garbage is a LOUD Err, never a silent fallback.
    std::env::set_var("CERULION_GATEWAY_PORT", "not-a-port");
    let err = resolve_gateway_base_port()
        .expect_err("a non-u16 port must be rejected loudly")
        .to_string();
    assert!(
        err.contains("CERULION_GATEWAY_PORT") && err.contains("valid TCP port"),
        "the rejection must name the env var + the constraint; got: {err}"
    );
}

// ─── The SHARED CERULION_NETWORK kill-switch parse ──────────────────────────
//
// The schema-info remote tier + topic-echo remote decode-seed must never skip
// the parse and ignore the kill-switch. They
// consult the SAME `network_env_kill` / `remote_network_suppressed`
// `resolve_run_network` uses — pinned here so the ONE env-parse can never drift.

/// `network_env_kill` — the single env-parse: unset/empty ⇒ `Unset`; `off`
/// (trimmed, ASCII case-insensitive) ⇒ `Off`; any other non-empty value ⇒
/// `Unrecognized` (carrying the ORIGINAL string) + `is_off()`. Hand oracle.
#[test]
fn network_env_kill_parse_oracle() {
    let _lock = env_lock();
    let _g = EnvGuard("CERULION_NETWORK");

    std::env::remove_var("CERULION_NETWORK");
    assert_eq!(network_env_kill(), NetworkEnvKill::Unset);
    assert!(!network_env_kill().is_off());

    std::env::set_var("CERULION_NETWORK", "");
    assert_eq!(network_env_kill(), NetworkEnvKill::Unset);

    std::env::set_var("CERULION_NETWORK", "off");
    assert_eq!(network_env_kill(), NetworkEnvKill::Off);
    assert!(network_env_kill().is_off());

    // Trimmed + case-insensitive.
    std::env::set_var("CERULION_NETWORK", "  OFF ");
    assert_eq!(network_env_kill(), NetworkEnvKill::Off);

    // Fails CLOSED, carrying the original value for the loud warn.
    std::env::set_var("CERULION_NETWORK", "on");
    assert_eq!(
        network_env_kill(),
        NetworkEnvKill::Unrecognized("on".to_string())
    );
    assert!(network_env_kill().is_off());
}

/// `remote_network_suppressed` — the discovery-side gate (schema info / topic
/// echo): unset ⇒ NOT suppressed; `off` ⇒ suppressed; an unrecognized value ⇒
/// suppressed AND a LOUD fail-closed warn (traced). This is the exact gate that
/// short-circuits the remote fetch BEFORE any session opens.
#[traced_test]
#[test]
fn remote_network_suppressed_honors_the_kill_switch_and_warns_on_garbage() {
    let _lock = env_lock();
    let _g = EnvGuard("CERULION_NETWORK");

    std::env::remove_var("CERULION_NETWORK");
    assert!(!remote_network_suppressed());

    std::env::set_var("CERULION_NETWORK", "OfF");
    assert!(remote_network_suppressed());

    std::env::set_var("CERULION_NETWORK", "yes-please");
    assert!(remote_network_suppressed());
    // The fail-closed warn names the offending value.
    assert!(logs_contain("unrecognized value") && logs_contain("yes-please"));
}

// ─── The robot NETWORK IDENTITY resolver ────────────────────────────────
//
// Identity (announce keys' robot chunk + `cerulion_q/{robot}/**` + the mDNS
// instance/TXT — ONE resolver) DEFAULTS to the machine hostname, DECOUPLED from
// the graph prefix (which drives topic naming). The escape hatch is the
// `CERULION_ROBOT_IDENTITY` env var. A `prefix: go2` graph on host
// `ubuntu` announces as `ubuntu`, never `go2`: identity is the hostname uniformly.

/// `resolve_robot_identity` — the hostname default (via `default_prefix`),
/// DECOUPLED from the graph prefix; a trimmed non-empty `CERULION_ROBOT_IDENTITY`
/// wins; an empty override falls back to the hostname. Hand oracle.
#[test]
fn robot_identity_defaults_to_hostname_and_override_wins() {
    let _lock = env_lock();
    let _g = EnvGuard("CERULION_ROBOT_IDENTITY");
    // `graph_with(None)` has name "netgate", prefix "ng" — the identity must NOT
    // be the prefix.
    let config = graph_with(None);
    let hostname = cerulion_core::graph::default_prefix(config.identity());

    // Default: the machine hostname, NOT the graph prefix.
    std::env::remove_var("CERULION_ROBOT_IDENTITY");
    assert_eq!(resolve_robot_identity(&config), hostname);
    assert_ne!(
        resolve_robot_identity(&config),
        config.prefix,
        "identity is DECOUPLED from the graph prefix (coupling them is a bug)"
    );

    // A trimmed, non-empty override WINS (the fleet / stock-image escape hatch).
    std::env::set_var("CERULION_ROBOT_IDENTITY", "  fleet-bot-7  ");
    assert_eq!(resolve_robot_identity(&config), "fleet-bot-7");

    // An empty/whitespace override has NO effect (falls back to the hostname).
    std::env::set_var("CERULION_ROBOT_IDENTITY", "   ");
    assert_eq!(resolve_robot_identity(&config), hostname);
}

/// Decision oracle: `GraphConfig::network_transport_config()` stamps the
/// robot identity from the SAME resolver as the CLI (`resolve_robot_identity`)
/// — the hostname, or the `CERULION_ROBOT_IDENTITY` override — NEVER the graph
/// prefix. This is the programmatic-embedder funnel the two network e2e tests
/// exercise; stamping `Some(prefix)` here would make a `prefix: go2`
/// graph announce as `go2`. Hand oracle, env-serialized.
#[test]
fn network_transport_config_stamps_identity_not_the_graph_prefix() {
    let _lock = env_lock();
    let _g = EnvGuard("CERULION_ROBOT_IDENTITY");
    // `graph_with(..)` has prefix "ng" — the stamped identity must NOT be it.
    let config = graph_with(Some(block_with_ingress()));
    let hostname = cerulion_core::graph::default_prefix(config.identity());

    // Unset ⇒ the hostname, DECOUPLED from the prefix, and equal to the CLI
    // resolver (the ONE-resolver-agrees-by-construction pin).
    std::env::remove_var("CERULION_ROBOT_IDENTITY");
    let cfg = config
        .network_transport_config()
        .expect("enabled block maps to Some");
    assert_eq!(cfg.robot_identity.as_deref(), Some(hostname.as_str()));
    assert_ne!(
        cfg.robot_identity.as_deref(),
        Some(config.prefix.as_str()),
        "identity is DECOUPLED from the graph prefix (coupling them is a bug)"
    );
    assert_eq!(
        cfg.robot_identity.as_deref(),
        Some(resolve_robot_identity(&config).as_str()),
        "the config funnel and the CLI resolver share ONE identity source"
    );

    // A trimmed, non-empty override WINS — still never the prefix.
    std::env::set_var("CERULION_ROBOT_IDENTITY", "  fleet-bot-7  ");
    let cfg = config
        .network_transport_config()
        .expect("enabled block maps to Some");
    assert_eq!(cfg.robot_identity.as_deref(), Some("fleet-bot-7"));
    assert_ne!(cfg.robot_identity.as_deref(), Some(config.prefix.as_str()));
}
