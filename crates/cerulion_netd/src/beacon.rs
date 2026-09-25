// SPDX-License-Identifier: AGPL-3.0-only
//! Netd's `_cerulion._tcp` mDNS BEACON: the advertise the netd fold lost.
//!
//! # What was broken
//!
//! mDNS is the primary robot-discovery rung ("mDNS finds robots; zenoh
//! connects and does everything after"), and the advertise lived in the
//! GATEWAY — the standalone `cerulion graph run-gateway` child
//! that a permissive `graph run` spawned. The netd fold then converged that plane
//! into the ONE per-computer `cerulion-netd`: a permissive run now pushes its
//! egress plan into netd over `register_egress` and spawns **no `run-gateway`
//! child at all** (pinned by `network_gateway_e2e_test`'s "the netd default path
//! must spawn NO `run-gateway` child"). The advertise call went with the child.
//!
//! MEASURED on a live desk: a robot on the desk's own /24, actively serving 86 topics
//! via `graph run attach --single-process`, answered NOTHING to
//! `dns-sd -B _cerulion._tcp local.` and logged zero mdns lines. The beacon was
//! not misconfigured — nothing on the netd-hosted plane reached it.
//!
//! # What advertises, and when
//!
//! The beacon is raised when netd's embedded egress gateway BOOTS
//! ([`crate::egress::GatewayEgressPlane`] — on the first `register_egress`, or,
//! at DAEMON START on a LISTEN-configured machine via
//! `boot_standing_gateway`) — the instant this machine becomes a producer on the
//! LAN. Either boot opens the shared zenoh session with its listener bound, so
//! the SRV port the beacon publishes is a port something is really listening on
//! (which is exactly why the start-time raise rides the gateway boot rather than
//! preceding it: the session is LAZY, and a beacon raised before any boot would
//! advertise an unbound port). It is held for the plane's LIFETIME (not the
//! gateway thread's): a drive thread that dies and re-boots did not change the
//! session or the port, so the beacon must not flap; the robot leaves
//! `_cerulion._tcp` when netd exits.
//!
//! # Why a DESK does not advertise
//!
//! The gate is [`cerulion_mdns::srv_port_from_listen_endpoints`] over the shared
//! session's own `NetworkConfig` — and that is not a formality. netd stamps
//! `robot_identity` (the hostname) at init on EVERY machine (so the
//! egress plane can announce), so identity alone cannot tell a robot from a desk.
//! The LISTEN endpoint can: a robot is given one (`CERULION_NETD_LISTEN=tcp/0.0.0.0:7683`
//! — see the serving-topology note), a desk netd is not. A desk with no
//! bound port has nothing a peer could dial, so advertising one would publish a
//! beacon that resolves to a closed port AND would list every laptop on the LAN
//! as a robot in `cerulion topic list`'s ROBOTS section. `None` is therefore a
//! REFUSAL, reported loudly, never a default port.
//!
//! # Failure posture
//!
//! Additive overlay, as specified: every non-advertising outcome
//! is a log line and netd keeps serving. An advertise that FAILS warns; a machine
//! with no dialable listen endpoint, and one running LOCAL-ONLY, each say so at
//! `info!`.
//!
//! That level is deliberate and was a polish fix. `cerulion_netd`'s
//! `Cargo.toml` sets `tracing/release_max_level_info`, so a `debug!` there does
//! not exist in a robot's RELEASE build at ANY `RUST_LOG` setting — and the
//! no-listen-port line is the ONE line that explains the measured symptom (a serving
//! robot invisible to the `_cerulion._tcp` rung).
//! [`crate::beacon::GatewayBeacon::decision`] is the only other window and it has
//! no production consumer, so the log IS the operator's evidence. It is a
//! decide-ONCE line, so `info!` cannot flood; it is deliberately not a `warn!`,
//! because EVERY desk takes this arm and a desk being silent here is correct. The
//! message therefore reads correctly on both shapes — see
//! [`crate::beacon::GatewayBeacon::ensure_raised`].
//!
//! The LOCAL-ONLY arm carries the same level for the same reason, and its scope
//! is stated rather than implied: it is not reachable through today's shipped
//! boot at all, because [`crate::egress::GatewayEgressPlane`] raises the beacon
//! only AFTER `GatewayRuntime::new_embedded`, which REFUSES a manager with no
//! network transport — so a `CERULION_NETD_NETWORK=off` daemon fails the gateway
//! boot long before the beacon is consulted. `info!` is therefore free (the line
//! cannot print, let alone flood) and buys the one thing that matters: if a
//! future caller — or a hand-built [`GatewayBeacon`](crate::beacon::GatewayBeacon) — ever
//! does reach it, an
//! operator on a release build gets evidence instead of silence. `debug!` there
//! was the only arm whose outcome the "every non-advertising outcome is a log
//! line" sentence above did not actually cover in a shipping binary.
//!
//! The SUPPRESSED arm stays at `debug!` deliberately: it is the test double's,
//! reachable only through
//! [`GatewayBeacon::without_mdns_for_test`](crate::beacon::GatewayBeacon::without_mdns_for_test),
//! and a line
//! about a test seam has no operator on the other end.

use std::sync::{Mutex, PoisonError};

use cerulion_core::transport::network::NetworkConfig;
use cerulion_mdns::{MdnsAdvertiseGuard, MdnsError};

/// What the shared session's network config implies about advertising this
/// machine over `_cerulion._tcp`. PURE — oracle-tested.
///
/// Every non-[`Advertise`](BeaconDecision::Advertise) arm names WHICH premise was
/// missing rather than collapsing to a bare `None`, because the arms mean
/// different things to an operator: a desk SHOULD be silent, while a robot that
/// meant to serve and set no listen endpoint is a misconfiguration the log has to
/// distinguish.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BeaconDecision {
    /// Advertise this robot at `port` (the shared session's bound zenoh listen
    /// port — the SRV record) under `robot` (the TXT `robot=` identity).
    Advertise {
        /// The announce identity (hostname / `CERULION_ROBOT_IDENTITY`).
        robot: String,
        /// The bound zenoh listen port peers dial.
        port: u16,
    },
    /// netd is running LOCAL-ONLY (`CERULION_NETD_NETWORK=off`) — there is no
    /// session and nothing to dial.
    NoNetwork,
    /// The shared config carries no announce identity, so the beacon would have
    /// no `robot=` value. Unreachable through `network_config_from_env` (which
    /// always stamps one); kept because the plane takes whatever config the
    /// manager was built with.
    NoIdentity,
    /// No dialable `tcp/` listen endpoint — the DESK shape (and a robot that
    /// forgot `CERULION_NETD_LISTEN`). Carries the endpoints that were offered so
    /// the log can show what it looked at.
    NoListenPort {
        /// The listen endpoints the config did carry (empty on a desk).
        listen: Vec<String>,
    },
}

/// Decide whether this machine's shared session should raise the
/// `_cerulion._tcp` beacon. PURE — no I/O, no env, no mDNS.
///
/// `cfg` is the shared `TransportManager`'s network config (`None` when netd is
/// LOCAL-ONLY). The identity is checked BEFORE the port so a network-configured
/// but identity-less config reports the premise it actually lacks.
pub fn plan_gateway_beacon(cfg: Option<&NetworkConfig>) -> BeaconDecision {
    let Some(cfg) = cfg else {
        return BeaconDecision::NoNetwork;
    };
    let Some(robot) = cfg.robot_identity.as_deref().filter(|r| !r.is_empty()) else {
        return BeaconDecision::NoIdentity;
    };
    match cerulion_mdns::srv_port_from_listen_endpoints(&cfg.listen_endpoints) {
        Some(port) => BeaconDecision::Advertise {
            robot: robot.to_string(),
            port,
        },
        None => BeaconDecision::NoListenPort {
            listen: cfg.listen_endpoints.clone(),
        },
    }
}

/// WHERE a [`GatewayBeacon`]'s advertise step lands. Chosen at CONSTRUCTION, so
/// the production call chain carries no runtime switch a variant could hide in.
///
/// The test arm exists because `ensure_raised` is driven by the REAL production
/// plane in `egress_plane_iox2_test`, and its robot-shaped fixtures decide
/// `Advertise` — which, unsuppressed, publishes a genuine, resolvable
/// `_cerulion._tcp` record on the operator's LAN for the length of the run
/// (MEASURED: `cargo test -p cerulion_netd` put `egpdesk` on two interfaces,
/// where a concurrent `cerulion topic list` renders it as a live ROBOT and
/// caches its dead ephemeral port in `peers.json` for the 7-day TTL). The
/// DECISION path is identical on both arms, so every oracle in those tests still
/// binds.
#[derive(Default)]
enum BeaconAdvertiser {
    /// PRODUCTION: register a real service on the LAN. `Default` so that
    /// `GatewayBeacon::default()`/`new()` cannot silently acquire the
    /// suppressed arm.
    #[default]
    Mdns,
    /// TEST ONLY: take the decision, reach no multicast socket. Reachable ONLY
    /// through [`GatewayBeacon::without_mdns_for_test`].
    Suppressed,
}

impl BeaconAdvertiser {
    /// Perform the advertise. `Ok(None)` means it was deliberately NOT performed
    /// (the [`Suppressed`](BeaconAdvertiser::Suppressed) arm) — never a failure.
    fn advertise(&self, robot: &str, port: u16) -> Result<Option<MdnsAdvertiseGuard>, MdnsError> {
        match self {
            Self::Mdns => cerulion_mdns::advertise_gateway(robot, port).map(Some),
            Self::Suppressed => Ok(None),
        }
    }

    /// Whether this advertiser reaches the real mDNS socket.
    fn is_real(&self) -> bool {
        matches!(self, Self::Mdns)
    }
}

/// netd's live `_cerulion._tcp` advertisement, raised AT MOST ONCE per daemon.
///
/// Holds the [`MdnsAdvertiseGuard`] for the daemon's lifetime; dropping this
/// (netd exit) multicasts the mDNS goodbye and withdraws the robot. Idempotent by
/// construction: a re-boot of the egress gateway drive thread must not flap the
/// beacon, so [`ensure_raised`](GatewayBeacon::ensure_raised) is a no-op once the
/// guard exists.
#[derive(Default)]
pub struct GatewayBeacon {
    state: Mutex<BeaconState>,
    /// Where the advertise lands — [`BeaconAdvertiser::Mdns`] on every
    /// production path. See [`GatewayBeacon::advertiser_is_real`].
    advertiser: BeaconAdvertiser,
}

/// The beacon's one-shot state. A machine that decided NOT to advertise records
/// that too, so the decision (and its log line) is taken once per daemon rather
/// than on every egress registration.
#[derive(Default)]
struct BeaconState {
    /// The decision this daemon took, `None` until the first
    /// [`GatewayBeacon::ensure_raised`]. Recorded — not just acted on — because
    /// it is the ONE observable that proves the beacon was CONSULTED at all: a
    /// desk and a netd that never asked both report `is_advertising() == false`,
    /// so without this a deleted call site is invisible. See
    /// [`GatewayBeacon::decision`].
    decision: Option<BeaconDecision>,
    /// The live mDNS registration, `Some` only on the advertising path. Its
    /// `Drop` multicasts the goodbye packet.
    guard: Option<MdnsAdvertiseGuard>,
}

impl GatewayBeacon {
    /// Refresh an existing advertisement after automatic robot startup.
    pub fn refresh_robot_facts(&self, expected_eid: &str) -> Result<bool, MdnsError> {
        let mut state = self.state.lock().unwrap_or_else(PoisonError::into_inner);
        match state.guard.as_mut() {
            Some(guard) => guard.refresh_robot_facts(expected_eid),
            None => Ok(false),
        }
    }

    /// A beacon that has not yet been raised. Advertises for REAL when its
    /// decision says to.
    pub fn new() -> Self {
        Self::default()
    }

    /// TEST ONLY: a beacon that takes every decision exactly as
    /// [`new`](Self::new) does, and then does NOT reach the multicast socket.
    ///
    /// For a suite that drives the REAL egress plane over robot-shaped configs,
    /// where an unsuppressed run publishes a live `_cerulion._tcp` record on
    /// whatever LAN the developer or CI runner is on. Nothing in `src/` may call
    /// this — a production plane built with it would take every decision, log
    /// every line and advertise NOTHING, which is precisely the defect.
    /// [`advertiser_is_real`](Self::advertiser_is_real) is the observable that
    /// makes that substitution detectable, and
    /// `a_desk_shaped_netd_boots_its_gateway_and_advertises_nothing`
    /// asserts it over the PRODUCTION constructor.
    #[doc(hidden)]
    pub fn without_mdns_for_test() -> Self {
        Self {
            state: Mutex::default(),
            advertiser: BeaconAdvertiser::Suppressed,
        }
    }

    /// Principle #3: whether this beacon's advertise step reaches the real mDNS
    /// socket (`true` on every production path).
    ///
    /// Derived from the ONE field that also picks the advertise, so it cannot
    /// drift from what the beacon would actually do. It is what lets a test
    /// prove the PRODUCTION constructor is still wired in without opening a
    /// socket: a desk declines before the advertiser is ever called, so the
    /// desk arm can assert this is `true` while publishing nothing.
    pub fn advertiser_is_real(&self) -> bool {
        self.advertiser.is_real()
    }

    /// Raise the beacon for `cfg` if this machine is a dialable gateway. Called
    /// on every egress-gateway boot; the FIRST call decides and every later one
    /// is a no-op, so a gateway drive thread that dies and re-boots does not
    /// re-register (or re-log) anything.
    ///
    /// NEVER fails: an advertise error is a loud `warn!` and netd keeps serving —
    /// the beacon is an additive discovery overlay, never load-bearing.
    /// Returns whether THIS call raised it, for the Principle-#3 observable and
    /// the tests.
    pub fn ensure_raised(&self, cfg: Option<&NetworkConfig>) -> bool {
        let mut state = self.state.lock().unwrap_or_else(PoisonError::into_inner);
        if state.decision.is_some() {
            return false;
        }
        let decision = plan_gateway_beacon(cfg);
        state.decision = Some(decision.clone());
        match decision {
            BeaconDecision::Advertise { robot, port } => {
                match self.advertiser.advertise(&robot, port) {
                    Ok(Some(g)) => {
                        tracing::info!(
                            robot = %robot,
                            port,
                            fullname = %g.fullname(),
                            "cerulion-netd is advertising this machine's gateway over \
                             mDNS (_cerulion._tcp) — the PRIMARY discovery rung. A desk \
                             browsing `_cerulion._tcp` (or running `cerulion topic list`) now \
                             finds this robot."
                        );
                        state.guard = Some(g);
                        true
                    }
                    // Unreachable in production: `BeaconAdvertiser::Mdns` never
                    // yields `None`. Only `without_mdns_for_test` installs the
                    // arm that does, and `advertiser_is_real()` is what proves
                    // production still installs the other one.
                    Ok(None) => {
                        tracing::debug!(
                            robot = %robot,
                            port,
                            "the mDNS advertise step is SUPPRESSED on this beacon (the \
                             test double) — the decision was taken and recorded, no record was \
                             published"
                        );
                        false
                    }
                    Err(e) => {
                        tracing::warn!(
                            robot = %robot,
                            port,
                            error = %e,
                            "cerulion-netd could NOT advertise over mDNS — this robot \
                             will not appear on the `_cerulion._tcp` discovery rung; a desk can \
                             still reach it with `cerulion topic list --connect tcp/<host>:<port>`. \
                             netd keeps serving."
                        );
                        false
                    }
                }
            }
            BeaconDecision::NoNetwork => {
                // `info!`, NOT `debug!`, for the SAME reason as the arm below:
                // `release_max_level_info` compiles a `debug!` out of a robot's
                // release build, so at `debug!` this outcome has no log line at
                // all in a shipping binary — while the module doc promises that
                // every non-advertising outcome does. Decided once per daemon, so
                // it cannot flood.
                //
                // SCOPE: this arm is not reachable through today's
                // shipped boot. `GatewayEgressPlane::ensure_gateway_booted`
                // raises the beacon only after `GatewayRuntime::new_embedded`,
                // which REFUSES a network-less manager, so a LOCAL-ONLY daemon
                // never gets here. That makes `info!` free rather than noisy, and
                // it is the level at which a future caller — or a hand-built
                // `GatewayBeacon` — would actually be heard.
                tracing::info!(
                    "cerulion-netd is LOCAL-ONLY (no zenoh session, e.g. \
                     CERULION_NETD_NETWORK=off) — it is NOT advertising this machine over mDNS \
                     (`_cerulion._tcp`), because there is no session for a peer to dial. This is \
                     the operator's own kill-switch, so on a machine that is MEANT to serve \
                     topics to other machines, unset it and restart cerulion-netd."
                );
                false
            }
            BeaconDecision::NoIdentity => {
                tracing::warn!(
                    "cerulion-netd has a network session but NO announce identity, so \
                     the `_cerulion._tcp` beacon has no robot= value and is SKIPPED — set \
                     CERULION_ROBOT_IDENTITY (or fix the host's hostname) to be discoverable"
                );
                false
            }
            BeaconDecision::NoListenPort { listen } => {
                // The DESK shape, and the normal case on most machines: nothing
                // is bound, so there is no port to publish.
                //
                // `info!`, NOT `debug!`: `release_max_level_info` compiles a
                // `debug!` out of a robot's release build entirely, so at
                // `debug!` this line does not exist at any RUST_LOG — and it is
                // the ONE piece of evidence for the measured symptom (a serving
                // robot invisible to the PRIMARY discovery rung), since
                // `mdns_beacon_decision()` has no production consumer. Decided
                // once per daemon, so it cannot flood.
                //
                // NOT `warn!`: every desk on the fleet takes this arm and a
                // silent desk is CORRECT, so the message has to read truthfully
                // on both shapes — benign for a desk, remedy for a robot.
                tracing::info!(
                    listen = ?listen,
                    "cerulion-netd is NOT advertising this machine over mDNS \
                     (`_cerulion._tcp`) — it has no dialable tcp/ listen endpoint, so there is no \
                     port a peer could dial. On a DESK this is correct and expected: a desk is \
                     not a robot and must not appear in `cerulion topic list`'s ROBOTS section. \
                     On a machine that is MEANT to serve topics to other machines, this line is \
                     WHY it is undiscoverable — give cerulion-netd a listen locator \
                     (CERULION_NETD_LISTEN=tcp/0.0.0.0:7683) and restart it."
                );
                false
            }
        }
    }

    /// Principle #3: whether the beacon is currently advertising. `false` on a
    /// desk, on a LOCAL-ONLY netd, and before the first egress registration.
    pub fn is_advertising(&self) -> bool {
        self.state
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .guard
            .is_some()
    }

    /// Principle #3: the registered service fullname
    /// (`<robot>._cerulion._tcp.local.`) while advertising, else `None`. The
    /// observable a test asserts the beacon's IDENTITY on without touching the
    /// LAN.
    pub fn advertised_fullname(&self) -> Option<String> {
        self.state
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .guard
            .as_ref()
            .map(|g| g.fullname().to_string())
    }

    /// Principle #3: the decision this daemon took about its beacon, `None`
    /// until something ASKS (the first egress-gateway boot).
    ///
    /// This is the observable that makes the WIRING checkable, and it is not
    /// redundant with [`is_advertising`](Self::is_advertising): a desk that
    /// correctly declined and a netd whose call site was deleted BOTH report
    /// `is_advertising() == false`, so only "was a decision recorded, and which
    /// one" separates them. A caller that never consults the beacon leaves this
    /// `None` forever.
    ///
    /// It is also what makes a ROBOT-shaped adoption test possible without
    /// touching the LAN: the decision is recorded BEFORE the advertise is
    /// attempted, so a beacon built by
    /// [`without_mdns_for_test`](Self::without_mdns_for_test) reports exactly
    /// the same `Advertise { .. }` a real one would.
    pub fn decision(&self) -> Option<BeaconDecision> {
        self.state
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .decision
            .clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A network config shaped like the one `network_config_from_env` builds.
    fn cfg(identity: Option<&str>, listen: &[&str]) -> NetworkConfig {
        NetworkConfig {
            multicast_scouting: true,
            gossip_scouting: true,
            listen_endpoints: listen.iter().map(|s| s.to_string()).collect(),
            robot_identity: identity.map(str::to_string),
            ..NetworkConfig::default()
        }
    }

    /// THE oracle, both directions in ONE body: the ROBOT shape advertises its
    /// bound port under its hostname identity, and the DESK shape — same
    /// identity, no listen endpoint, which is exactly what `network_config_from_env`
    /// builds on a laptop — advertises NOTHING.
    ///
    /// The pairing is the point. netd stamps `robot_identity` on EVERY machine,
    /// so a gate keyed on identity would make every desk on the LAN
    /// announce itself as a robot; only the listen endpoint separates them.
    #[test]
    fn a_robot_with_a_bound_listener_advertises_and_a_desk_does_not() {
        assert_eq!(
            plan_gateway_beacon(Some(&cfg(Some("go2"), &["tcp/0.0.0.0:7683"]))),
            BeaconDecision::Advertise {
                robot: "go2".to_string(),
                port: 7683,
            },
            "a robot with CERULION_NETD_LISTEN advertises its bound port"
        );
        // The DESK: identity present (netd always stamps it), NO listen endpoint.
        assert_eq!(
            plan_gateway_beacon(Some(&cfg(Some("desk-macbook"), &[]))),
            BeaconDecision::NoListenPort { listen: vec![] },
            "a desk netd carries an identity but no listener — it must NOT advertise"
        );
    }

    /// The remaining refusal arms, each naming the premise it lacks. A bare
    /// `None` would collapse "this is a desk" into "this robot is misconfigured",
    /// and the two need different log lines.
    #[test]
    fn every_missing_premise_is_named_rather_than_collapsed() {
        // LOCAL-ONLY netd (CERULION_NETD_NETWORK=off) — no session at all.
        assert_eq!(plan_gateway_beacon(None), BeaconDecision::NoNetwork);
        // Network + listener but no announce identity: nothing to put in robot=.
        assert_eq!(
            plan_gateway_beacon(Some(&cfg(None, &["tcp/0.0.0.0:7683"]))),
            BeaconDecision::NoIdentity
        );
        // An EMPTY identity is treated as absent, not advertised as `robot=`.
        assert_eq!(
            plan_gateway_beacon(Some(&cfg(Some(""), &["tcp/0.0.0.0:7683"]))),
            BeaconDecision::NoIdentity
        );
        // A listener that is not dialable tcp/ carries the endpoints it saw, so
        // the operator's log shows what was rejected.
        assert_eq!(
            plan_gateway_beacon(Some(&cfg(Some("go2"), &["udp/0.0.0.0:7683"]))),
            BeaconDecision::NoListenPort {
                listen: vec!["udp/0.0.0.0:7683".to_string()],
            }
        );
    }

    /// The IPv6 + multi-endpoint forms a real `CERULION_NETD_LISTEN` carries.
    /// (The port RULE itself is oracle-tested in `cerulion_mdns`; this pins that
    /// the netd decision reads the shared rule rather than a second copy.)
    #[test]
    fn the_advertised_port_comes_from_the_first_dialable_tcp_endpoint() {
        assert_eq!(
            plan_gateway_beacon(Some(&cfg(Some("go2"), &["tcp/[::]:7683"]))),
            BeaconDecision::Advertise {
                robot: "go2".to_string(),
                port: 7683,
            }
        );
        assert_eq!(
            plan_gateway_beacon(Some(&cfg(
                Some("go2"),
                &["udp/0.0.0.0:1", "tcp/192.0.2.43:7001", "tcp/0.0.0.0:9999"],
            ))),
            BeaconDecision::Advertise {
                robot: "go2".to_string(),
                port: 7001,
            }
        );
    }

    /// A desk-shaped beacon settles WITHOUT touching mDNS, and stays settled —
    /// the drive-thread re-boot path calls `ensure_raised` again and must not
    /// re-decide (or re-log) anything.
    ///
    /// Deliberately the NON-advertising shape ON THE PRODUCTION CONSTRUCTOR: a
    /// desk declines before the advertiser is reached, so `GatewayBeacon::new()`
    /// is safe here and this arm doubles as proof that the real advertiser
    /// publishes nothing when the decision says not to. The ADVERTISING side is
    /// covered by the `#[ignore]`d live-multicast arm (it needs real multicast) in
    /// `cerulion_cli_engine/tests/mdns_live_test.rs`.
    #[test]
    fn a_declined_beacon_is_decided_once_and_never_advertises() {
        let beacon = GatewayBeacon::new();
        assert!(
            beacon.advertiser_is_real(),
            "the production constructor must install the REAL advertiser"
        );
        let desk = cfg(Some("desk"), &[]);
        // NOTHING has asked yet — the state that a deleted call site leaves
        // behind forever, and the reason `decision()` exists beside
        // `is_advertising()` (both are `false`/`None`-ish on a desk).
        assert_eq!(
            beacon.decision(),
            None,
            "un-consulted beacon has no decision"
        );
        assert!(
            !beacon.ensure_raised(Some(&desk)),
            "a desk raises nothing on the first call"
        );
        assert_eq!(
            beacon.decision(),
            Some(BeaconDecision::NoListenPort { listen: vec![] }),
            "the decision is RECORDED, so a caller that asked is distinguishable \
             from one that never did"
        );
        assert!(!beacon.is_advertising());
        assert_eq!(beacon.advertised_fullname(), None);
        // A gateway re-boot re-calls it: still nothing, still settled.
        assert!(!beacon.ensure_raised(Some(&desk)));
        // Even handed a ROBOT config later, a settled beacon does not re-decide:
        // the session's listen endpoints are immutable at init, so a later
        // divergent config would be a lie about the same session — and a beacon
        // that flapped would make the robot appear and vanish on every desk.
        assert!(!beacon.ensure_raised(Some(&cfg(Some("go2"), &["tcp/0.0.0.0:7683"]))));
        assert!(!beacon.is_advertising());
        assert_eq!(
            beacon.decision(),
            Some(BeaconDecision::NoListenPort { listen: vec![] }),
            "the FIRST decision stands — a re-boot must not re-decide"
        );
    }

    /// The SUPPRESSED advertiser takes the decision and publishes nothing — the
    /// property that lets `egress_plane_iox2_test` drive robot-shaped configs
    /// through the REAL plane without putting a record on the operator's LAN.
    ///
    /// Both halves matter and are asserted in ONE body. If the decision path
    /// diverged, every `mdns_beacon_decision()` oracle in that suite would be
    /// measuring the double instead of the production rule; if suppression did
    /// not hold, the suite is back to advertising `egpdesk` on every run. The
    /// PRODUCTION-constructor twin is the arm above (`advertiser_is_real()`
    /// there, `false` here), so a swap in either direction is visible.
    #[test]
    fn a_suppressed_beacon_takes_the_same_decision_and_publishes_nothing() {
        let robot = cfg(Some("go2"), &["tcp/0.0.0.0:7683"]);
        // The oracle is the PRODUCTION rule, computed independently — so this
        // cannot be satisfied by a double that decides something of its own.
        assert_eq!(
            plan_gateway_beacon(Some(&robot)),
            BeaconDecision::Advertise {
                robot: "go2".to_string(),
                port: 7683,
            }
        );

        let beacon = GatewayBeacon::without_mdns_for_test();
        assert!(
            !beacon.advertiser_is_real(),
            "the test constructor must NOT install the real advertiser"
        );
        assert!(
            !beacon.ensure_raised(Some(&robot)),
            "a suppressed advertise raises nothing, so it reports raising nothing"
        );
        assert_eq!(
            beacon.decision(),
            Some(BeaconDecision::Advertise {
                robot: "go2".to_string(),
                port: 7683,
            }),
            "the DECISION is identical to the production rule's — that is what \
             keeps the adoption oracles meaningful"
        );
        // Nothing was published, so there is no guard and no fullname.
        assert!(!beacon.is_advertising());
        assert_eq!(beacon.advertised_fullname(), None);
        // And it is still decide-ONCE.
        assert!(!beacon.ensure_raised(Some(&robot)));
    }

    /// Match a captured line's LEVEL as a whole whitespace token.
    ///
    /// A bare `contains("INFO")` would also match the span name (`tracing-test`
    /// renders the test function's own name into every line) or a field value.
    /// Same discipline as `query.rs`'s `line_level` and
    /// `rmw_publish_reject_test`'s.
    fn line_level(line: &str) -> Option<&str> {
        line.split_whitespace()
            .find(|t| matches!(*t, "TRACE" | "DEBUG" | "INFO" | "WARN" | "ERROR"))
    }

    /// The no-listen-port refusal is emitted at `info!` — the ONE level at which
    /// it survives a robot's RELEASE build.
    ///
    /// `cerulion_netd/Cargo.toml` sets `tracing/release_max_level_info`, so a
    /// `debug!` here is compiled OUT of the shipping binary and the operator has
    /// no evidence at ANY `RUST_LOG` — which is the measured symptom reproducing
    /// silently, since `mdns_beacon_decision()` has no production consumer. It
    /// must also not be a `warn!`: EVERY desk on the fleet takes this arm and a
    /// silent desk is correct.
    ///
    /// The message is asserted to carry BOTH readings, because this one line is
    /// read by two populations: a desk operator who must not be alarmed, and a
    /// robot operator who needs the remedy.
    #[test]
    #[tracing_test::traced_test]
    fn the_no_listen_port_refusal_is_logged_at_info_so_a_release_build_keeps_it() {
        let beacon = GatewayBeacon::new();
        assert!(!beacon.ensure_raised(Some(&cfg(Some("desk"), &[]))));

        logs_assert(|lines: &[&str]| {
            let at = |level: &str| {
                lines
                    .iter()
                    .filter(|l| {
                        line_level(l) == Some(level) && l.contains("no dialable tcp/ listen")
                    })
                    .count()
            };
            let (info, debug, warn) = (at("INFO"), at("DEBUG"), at("WARN"));
            if (info, debug, warn) != (1, 0, 0) {
                return Err(format!(
                    "the no-listen-port refusal must be exactly one INFO line \
                     (release_max_level_info compiles DEBUG out of a robot's build, \
                     and WARN would alarm every desk); got \
                     (INFO {info}, DEBUG {debug}, WARN {warn})\n{}",
                    lines.join("\n")
                ));
            }
            let line = lines
                .iter()
                .find(|l| l.contains("no dialable tcp/ listen"))
                .expect("the line counted above must be findable");
            for needle in [
                // The desk half: this is expected, not a fault.
                "On a DESK this is correct",
                // The robot half: the remedy, spelled out.
                "CERULION_NETD_LISTEN=tcp/0.0.0.0:7683",
            ] {
                if !line.contains(needle) {
                    return Err(format!("refusal line is missing {needle:?}: {line}"));
                }
            }
            Ok(())
        });
    }

    /// The LOCAL-ONLY refusal is emitted at `info!` too — the module doc promises
    /// that EVERY non-advertising outcome is a log line, and under
    /// `release_max_level_info` a `debug!` is not one in a shipping binary.
    ///
    /// Its OWN arm rather than an extension of the sibling above, so a failure can
    /// be attributed: the two conditions are separate `tracing` call sites, and
    /// an unpinned arm was measured to be free to become a `debug!`.
    ///
    /// Scope, stated because the arm's value depends on it: this outcome is
    /// unreachable through `GatewayEgressPlane` today (the gateway boot refuses a
    /// network-less manager first), so what is pinned is the beacon's own
    /// contract, not a line an operator will see on a `CERULION_NETD_NETWORK=off`
    /// desk.
    #[test]
    #[tracing_test::traced_test]
    fn the_local_only_refusal_is_logged_at_info_so_a_release_build_keeps_it() {
        let beacon = GatewayBeacon::new();
        assert!(!beacon.ensure_raised(None));
        assert_eq!(beacon.decision(), Some(BeaconDecision::NoNetwork));

        logs_assert(|lines: &[&str]| {
            let at = |level: &str| {
                lines
                    .iter()
                    .filter(|l| line_level(l) == Some(level) && l.contains("LOCAL-ONLY"))
                    .count()
            };
            let (info, debug) = (at("INFO"), at("DEBUG"));
            if (info, debug) != (1, 0) {
                return Err(format!(
                    "the LOCAL-ONLY refusal must be exactly one INFO line \
                     (release_max_level_info compiles DEBUG out of a robot's build); got \
                     (INFO {info}, DEBUG {debug})\n{}",
                    lines.join("\n")
                ));
            }
            let line = lines
                .iter()
                .find(|l| l.contains("LOCAL-ONLY"))
                .expect("the line counted above must be findable");
            // It has to say what is NOT happening, and name the operator's own
            // switch — a bare "local-only" states a config value back at someone
            // who set it, and explains nothing about mDNS.
            for needle in ["NOT advertising", "CERULION_NETD_NETWORK=off"] {
                if !line.contains(needle) {
                    return Err(format!("local-only line is missing {needle:?}: {line}"));
                }
            }
            Ok(())
        });
    }
}
