// SPDX-License-Identifier: AGPL-3.0-only
//! The pure [`PairingAuthorizer`] — the deny-by-default accept gate.
//!
//! It implements `cerud::authz::Authorizer`, so cerud's ops server re-checks
//! every verb through it. On each decision it:
//!
//! 1. requires the caller be **transport-authenticated** with a parseable
//!    32-byte device key (the iroh transport yields `verified(remote_id_hex)`);
//! 2. maps that device key to an account via the [`DeviceAccountIndex`] side-map
//!    (established pairings never re-run the full chain per connect; the
//!    access row is truth, [`TrustStore::is_allowed`]);
//! 3. applies the per-verb policy below.
//!
//! ## The verb → capability table (the sacred security surface)
//!
//! | Verb(s) | Class | Requirement |
//! |---|---|---|
//! | `claim` / `pair` / `present-grant` / `code-pair` | **Bootstrap** | Exempt from the access list — self-gates in the handler (chassis secret / a valid cert chain / an owner-signed grant / a `CpaceConfirmed`). Admissible for any authed key; on an UNCLAIMED robot only `claim` is admissible. |
//! | `engage-estop` | **E-stop floor** | Any PAIRED account, regardless of scope (`cerud::lease` — any paired session, always wins). |
//! | `inventory` / `log-tail` | Normal | `CAP_OBSERVE` (role ≥ `VIEWER`) — read-only observation. |
//! | `restart` / `deploy` | Normal | Role ≥ `OPERATOR` (a lifecycle mutation; no fine-grained cap models it, so the role IS the gate). |
//! | `teleop` | Normal | `CAP_TELEOP` AND role ≥ `OPERATOR` — actuation. |
//! | *any other* | — | **DENY** (deny-by-default; an unclassified verb is refused, never fail-open — a new mutating verb must be classified here). |
//!
//! "Role ≥ X" means numerically ≤ X's value — a lower `Role` value is higher
//! privilege (`OWNER=1 < OPERATOR=2 < VIEWER=3`).
//!
//! ## Accept-time plane routing
//!
//! [`PairingAuthorizer::classify_accept`] is the accept gate for the two ALPNs:
//! the ops plane admits a paired key to the full surface and an unpaired /
//! unclaimed key to the bootstrap surface only; the wire plane refuses an
//! unpaired peer outright (catalog/demand require a paired `CAP_OBSERVE` account
//! — the `cerulion_q` marker).

use cerud::authz::{Authorizer, AuthzDecision};
use cerud::transport::CallerIdentity;
use cerulion_core::transport::demand_authorizer::{
    DemandAuthorizer, DemandDecision, DemandSubject,
};
use cerulion_pairing::format::{Role, Scope};
use cerulion_pairing::verify::TrustStore;

use crate::device_index::DeviceAccountIndex;
use crate::trust::{KeyAccess, SharedTrust};
use crate::verbs;

/// The security classification of an ops verb (the sacred table above).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VerbClass {
    /// Bootstrap verbs: exempt from the access-list check (self-gate in the
    /// handler). On an unclaimed robot only `claim` is admissible.
    Bootstrap,
    /// The e-stop permission floor: any paired account, regardless of scope.
    EstopFloor,
    /// A normal verb, gated on the access list AND this requirement.
    Normal(AccessRequirement),
}

/// The access requirement for a normal verb: ALL `required_caps` bits must be
/// present in the account's scope AND its role must be at least as privileged as
/// `max_role` (numerically ≤ — lower [`Role`] value = higher privilege).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct AccessRequirement {
    required_caps: u64,
    max_role: Role,
}

/// Classify a verb. `None` = unclassified → deny-by-default: a new verb
/// (especially a mutating one) is refused until it is classified here, never
/// fail-open.
fn classify_verb(verb: &str) -> Option<VerbClass> {
    Some(match verb {
        verbs::CLAIM
        | verbs::PAIR
        | verbs::PRESENT_GRANT
        | verbs::CODE_PAIR_START
        | verbs::CODE_PAIR_FINISH => VerbClass::Bootstrap,
        verbs::ENGAGE_ESTOP => VerbClass::EstopFloor,
        verbs::INVENTORY | verbs::LOG_TAIL => VerbClass::Normal(AccessRequirement {
            required_caps: Scope::CAP_OBSERVE,
            max_role: Role::VIEWER,
        }),
        verbs::RESTART | verbs::DEPLOY => VerbClass::Normal(AccessRequirement {
            required_caps: 0,
            max_role: Role::OPERATOR,
        }),
        verbs::TELEOP => VerbClass::Normal(AccessRequirement {
            required_caps: Scope::CAP_TELEOP,
            max_role: Role::OPERATOR,
        }),
        _ => return None,
    })
}

/// The accept-time routing decision for one incoming connection (per ALPN).
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "decision", rename_all = "snake_case")]
pub enum AcceptDecision {
    /// Ops plane, full surface (a paired, allowed key). Per-verb authz still runs
    /// downstream in the ops server.
    OpsAdmit,
    /// Ops plane, bootstrap surface only (an unpaired key or an unclaimed robot).
    /// The ops server admits only the self-gating bootstrap verbs.
    OpsBootstrapOnly,
    /// Wire plane admitted (a paired `CAP_OBSERVE` account).
    WireAdmit,
    /// Refused outright (wire plane, unpaired/unclaimed) — close, serve nothing.
    Refuse {
        /// The diagnosable reason.
        reason: String,
    },
    /// An ALPN the daemon does not serve — close.
    UnknownAlpn {
        /// The negotiated ALPN bytes.
        alpn: Vec<u8>,
    },
}

/// The deny-by-default accept gate over the LIVE trust store + the side-map.
///
/// The reads here never mutate; they observe the SAME [`SharedTrust`] the pairing
/// bootstrap verbs mutate, so a `claim` / `pair` / `code-pair` takes
/// effect at the very next accept WITHOUT a daemon restart (the always-on
/// contract). Every decision reads the claimed flag + the key's access state under
/// ONE lock ([`SharedTrust::snapshot_for_key`]) so it never observes a torn view.
#[derive(Debug)]
pub struct PairingAuthorizer {
    shared: SharedTrust,
}

impl PairingAuthorizer {
    /// Build the authorizer over an owned trust store + device→account side-map,
    /// wrapping them in a fresh read-only [`SharedTrust`] (empty MAC key — the
    /// pure-authz shape that never persists). Preserved for the decision-matrix
    /// tests; the daemon uses [`PairingAuthorizer::from_shared`] so the authorizer
    /// and the pairing verbs share ONE live handle.
    pub fn new(store: TrustStore, index: DeviceAccountIndex) -> Self {
        PairingAuthorizer::from_shared(SharedTrust::new(store, index, Vec::new()))
    }

    /// Build the authorizer over an existing LIVE [`SharedTrust`] handle — the
    /// daemon path, so the accept gate reads the state the pairing verbs write.
    pub fn from_shared(shared: SharedTrust) -> Self {
        PairingAuthorizer { shared }
    }

    /// Authorize one `(caller, verb)`. The args-free core of the `Authorizer`
    /// trait impl (the policy is verb-based; there is no args-based policy).
    pub fn authorize_verb(&self, caller: &CallerIdentity, verb: &str) -> AuthzDecision {
        // 1. Classify the verb. Unclassified → deny-by-default.
        let class = match classify_verb(verb) {
            Some(c) => c,
            None => {
                return AuthzDecision::deny(format!(
                    "verb '{verb}' is not classified by the PairingAuthorizer; \
                     deny-by-default (add it to the verb → capability table)"
                ))
            }
        };

        // 2. Require a transport-authenticated, parseable 32-byte device key.
        let device_key = match authed_device_key(caller) {
            Some(k) => k,
            None => {
                return AuthzDecision::deny(format!(
                    "caller '{}' is not transport-authenticated (or its id is not a \
                     32-byte device key); the remote plane admits only TLS-authed \
                     iroh peers",
                    caller.id
                ))
            }
        };

        // 3. One torn-free read of (claimed, access) for this key.
        let (claimed, access) = self.shared.snapshot_for_key(&device_key);

        // 4. UNCLAIMED robot: ONLY `claim` is admissible.
        if !claimed {
            return if matches!(class, VerbClass::Bootstrap) && verb == verbs::CLAIM {
                AuthzDecision::Allow
            } else {
                AuthzDecision::deny(format!(
                    "robot is UNCLAIMED: only the `claim` bootstrap verb is admissible \
                     (verb '{verb}' refused)"
                ))
            };
        }

        // 5. Claimed robot.
        match class {
            // Bootstrap verbs self-gate in the handler; exempt from the access
            // list. (`claim` on a claimed robot is allowed here but rejected by
            // its handler — already claimed.)
            VerbClass::Bootstrap => AuthzDecision::Allow,

            // E-stop floor: any PAIRED account, regardless of scope.
            VerbClass::EstopFloor => match access {
                KeyAccess::Allowed(_) => AuthzDecision::Allow,
                KeyAccess::NotAllowed => AuthzDecision::deny(
                    "e-stop floor: the device key's account is not on the access list \
                     (revoked, removed, or its owner-signed grant expired)"
                        .to_string(),
                ),
                // This specific DEVICE key was revoked (the account's
                // other devices may still be allowed) — denied even at the e-stop floor.
                KeyAccess::DeviceRevoked => AuthzDecision::deny(
                    "e-stop floor: this device key is revoked by the current access-list \
                     epoch — its account's other devices may still be \
                     allowed, but this desk was cut"
                        .to_string(),
                ),
                KeyAccess::Unpaired => AuthzDecision::deny(
                    "e-stop floor requires a PAIRED account; this device key is unpaired \
                     (only bootstrap verbs are admissible)"
                        .to_string(),
                ),
            },

            // Normal verb: paired AND meets the capability + role requirement.
            VerbClass::Normal(req) => match access {
                KeyAccess::Unpaired => AuthzDecision::deny(format!(
                    "device key is unpaired: verb '{verb}' requires a paired account \
                     (only bootstrap verbs are admissible)"
                )),
                KeyAccess::NotAllowed => AuthzDecision::deny(format!(
                    "the account for verb '{verb}' is not on the access list \
                     (revoked, removed, or its owner-signed grant expired)"
                )),
                // The DEVICE key is epoch-revoked, independent of its
                // account row.
                KeyAccess::DeviceRevoked => AuthzDecision::deny(format!(
                    "the device key for verb '{verb}' is revoked by the current \
                     access-list epoch — this desk was cut"
                )),
                KeyAccess::Allowed(scope) => {
                    let caps_ok = (scope.caps & req.required_caps) == req.required_caps;
                    let role_ok = scope.role.0 <= req.max_role.0;
                    if caps_ok && role_ok {
                        AuthzDecision::Allow
                    } else {
                        AuthzDecision::deny(format!(
                            "scope (role {}, caps {:#06x}) lacks the requirement for \
                             verb '{verb}' (needs caps {:#06x} and role value <= {})",
                            scope.role.0, scope.caps, req.required_caps, req.max_role.0
                        ))
                    }
                }
            },
        }
    }

    /// The accept-time plane-routing decision for an incoming connection.
    pub fn classify_accept(&self, alpn: &[u8], remote_id: &[u8; 32]) -> AcceptDecision {
        self.classify_accept_with_enrollment_hint(alpn, remote_id).0
    }

    /// The enrollment hint and refusal must describe the same trust snapshot.
    pub(crate) fn classify_accept_with_enrollment_hint(
        &self,
        alpn: &[u8],
        remote_id: &[u8; 32],
    ) -> (AcceptDecision, bool) {
        let (claimed, access) = self.shared.snapshot_for_key(remote_id);
        let unpaired =
            alpn == cerulion_link::alpn::WIRE && claimed && matches!(access, KeyAccess::Unpaired);
        let decision = if alpn == cerulion_link::alpn::OPS {
            // Ops plane: per-verb authz runs downstream in the ops server. At
            // accept we route by reachability — a paired+allowed key gets the
            // full surface; anyone else reaches only the bootstrap surface.
            if claimed && matches!(access, KeyAccess::Allowed(_)) {
                AcceptDecision::OpsAdmit
            } else {
                AcceptDecision::OpsBootstrapOnly
            }
        } else if alpn == cerulion_link::alpn::WIRE {
            // Wire plane: catalog/demand REQUIRE a paired CAP_OBSERVE account
            // (the cerulion_q marker). Unpaired → refuse outright.
            match wire_admittable(claimed, access) {
                Ok(()) => AcceptDecision::WireAdmit,
                Err(reason) => AcceptDecision::Refuse { reason },
            }
        } else {
            AcceptDecision::UnknownAlpn {
                alpn: alpn.to_vec(),
            }
        };
        (decision, unpaired)
    }

    /// The live shared-trust handle backing this authorizer, so the daemon can
    /// hand the SAME handle to the pairing bootstrap verbs (they mutate what the
    /// accept gate reads).
    pub fn shared(&self) -> &SharedTrust {
        &self.shared
    }
}

/// Whether a key may be admitted to the wire plane (paired + `CAP_OBSERVE`) given
/// a torn-free `(claimed, access)` read.
fn wire_admittable(claimed: bool, access: KeyAccess) -> Result<(), String> {
    if !claimed {
        return Err(
            "wire plane refused: robot is UNCLAIMED (catalog/demand require a \
             paired CAP_OBSERVE account)"
                .to_string(),
        );
    }
    match access {
        KeyAccess::Unpaired => Err(
            "wire plane refused: unpaired device key (catalog/demand require a paired \
             CAP_OBSERVE account)"
                .to_string(),
        ),
        KeyAccess::NotAllowed => Err(
            "wire plane refused: the account is not on the access list (revoked, \
             removed, or its owner-signed grant expired)"
                .to_string(),
        ),
        // This DEVICE key is epoch-revoked (its account's other devices
        // may still be admitted, but this desk was cut). The sweep re-checks
        // this predicate, so an established stream is EVICTED when a synced epoch
        // flips it.
        KeyAccess::DeviceRevoked => Err(
            "wire plane refused: this device key is revoked by the current access-list \
             epoch — this desk was cut"
                .to_string(),
        ),
        KeyAccess::Allowed(scope) => {
            if (scope.caps & Scope::CAP_OBSERVE) == 0 {
                Err(
                    "wire plane refused: the account lacks CAP_OBSERVE (catalog/demand \
                     require it)"
                        .to_string(),
                )
            } else {
                Ok(())
            }
        }
    }
}

impl Authorizer for PairingAuthorizer {
    fn authorize(
        &self,
        caller: &CallerIdentity,
        verb: &str,
        _args: &serde_json::Value,
    ) -> AuthzDecision {
        self.authorize_verb(caller, verb)
    }
}

/// An `is_allowed`-backed implementation of the
/// [`DemandAuthorizer`] seam — the LIVE per-demand enforcement predicate, INSTALLED on
/// the robot's serving demand plane.
///
/// ## Two composed enforcement points
///
/// 1. The connection-**ACCEPT** gate ([`PairingAuthorizer::classify_accept`] → the
///    private `wire_admittable` → the [`SharedTrust::snapshot_for_key`] /
///    [`cerulion_pairing::verify::TrustStore::is_allowed`] read) refuses an unpaired /
///    revoked / EXPIRED-owner-grant peer at CONNECT (whole-plane admission).
/// 2. **Demand plane:** this `DemandAuthorizer` impl is installed on the robot's serving
///    demand plane (`cerulion_remoted::wire::serve_wire_connection`), adding **per-topic
///    granularity** at each `Demand` AND **mid-session eviction** — a periodic sweep
///    re-checks every established tap against the LIVE trust state, so an owner-revoke /
///    grant-expiry that lands AFTER accept drops the already-streaming topic. Both read
///    the SAME live [`SharedTrust`], so a `present-grant` / pairing / revoke is honored
///    with no re-plumbing.
///
/// ## Why it is NOT installed at netd's desk-side WAN demand site
///
/// netd is the DESK: its WAN demand call (`cerulion_netd::iroh_plane`'s
/// `DemandSubject::Wan`) is a client-side pre-flight, not the robot's authoritative
/// gate. Its subject carries netd's OWN desk key (the demander), but netd holds no
/// robot access list — the ROBOT enforces. So netd's `build_demand_authorizer()` stays
/// the deny-nothing [`cerulion_core::transport::demand_authorizer::AllowAllAuthorizer`];
/// this `is_allowed` predicate runs on the ROBOT plane, keyed by the connection's
/// authenticated remote peer key (never the dial target).
///
/// - **WAN** (`iroh-wan`): admit iff the subject's demander key maps to an account on
///   the access list with `CAP_OBSERVE` (the wire-plane requirement — a demand streams a
///   topic, which is observation) — the SAME gate `classify_accept` applies at the wire
///   ALPN.
/// - **LAN** (`zenoh-lan`): the zenoh LAN demand plane carries NO authenticated identity
///   today (`DemandSubject::Lan { locator: None }`), so there is no account to check
///   `is_allowed` against; the LAN arm stays permissive. A paired LAN identity
///   is not carried here yet.
impl DemandAuthorizer for PairingAuthorizer {
    fn authorize_demand(&self, subject: &DemandSubject, topic: &str) -> DemandDecision {
        match subject {
            DemandSubject::Wan { demander_key } => {
                // This impl is INSTALLED on the ROBOT serving demand plane
                // (`cerulion_remoted::wire::serve_wire_connection`), keyed by the
                // connection's authenticated remote peer key — the demanding desk. That
                // plane owns the robot-side `TrustStore` (the SAME live `SharedTrust`
                // this authorizer reads), gates each `Demand` per topic, and re-checks
                // on a periodic sweep so a mid-session owner-revoke DROPS the already-
                // established stream. It is deliberately NOT installed at netd's
                // desk-side WAN pre-flight (`iroh_plane::ensure_async`), which threads
                // netd's own desk key but stays deny-nothing — the robot is the
                // authoritative enforcement point.
                let (claimed, access) = self.shared.snapshot_for_key(demander_key);
                match wire_admittable(claimed, access) {
                    Ok(()) => DemandDecision::Allow,
                    // hot-path-alloc-ok: a denied DEMAND is a control-plane refusal
                    // (never the per-frame publish/receive path); the loud reason names
                    // the plane + topic for the refusal breadcrumb.
                    Err(reason) => DemandDecision::deny(format!(
                        "iroh-wan demand for topic '{topic}' refused: {reason}"
                    )),
                }
            }
            // No LAN identity to resolve yet — permissive (see the type doc).
            DemandSubject::Lan { .. } => DemandDecision::Allow,
            // `DemandSubject` is `#[non_exhaustive]`: a future plane variant this
            // predicate has not classified is fail-CLOSED (deny), never fail-open — a
            // new demand plane must be explicitly gated here before it can serve.
            // hot-path-alloc-ok: control-plane refusal, never the per-frame path.
            other => DemandDecision::deny(format!(
                "{} demand for topic '{topic}' refused: unclassified demand plane \
                 (add it to the DemandAuthorizer) — fail-closed",
                other.plane_label()
            )),
        }
    }
}

/// Extract the transport-authenticated 32-byte device key from a caller, or
/// `None` if the caller is not authenticated OR its id is not a 32-byte hex key.
fn authed_device_key(caller: &CallerIdentity) -> Option<[u8; 32]> {
    if !caller.authenticated {
        return None;
    }
    let bytes = hex::decode(&caller.id).ok()?;
    bytes.try_into().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classify_verb_covers_the_sacred_table_and_denies_the_rest() {
        // Bootstrap.
        assert!(matches!(
            classify_verb(verbs::CLAIM),
            Some(VerbClass::Bootstrap)
        ));
        assert!(matches!(
            classify_verb(verbs::PAIR),
            Some(VerbClass::Bootstrap)
        ));
        assert!(matches!(
            classify_verb(verbs::PRESENT_GRANT),
            Some(VerbClass::Bootstrap)
        ));
        assert!(matches!(
            classify_verb(verbs::CODE_PAIR_START),
            Some(VerbClass::Bootstrap)
        ));
        assert!(matches!(
            classify_verb(verbs::CODE_PAIR_FINISH),
            Some(VerbClass::Bootstrap)
        ));
        // E-stop floor.
        assert!(matches!(
            classify_verb(verbs::ENGAGE_ESTOP),
            Some(VerbClass::EstopFloor)
        ));
        // Observe-class.
        for v in [verbs::INVENTORY, verbs::LOG_TAIL] {
            assert_eq!(
                classify_verb(v),
                Some(VerbClass::Normal(AccessRequirement {
                    required_caps: Scope::CAP_OBSERVE,
                    max_role: Role::VIEWER,
                })),
                "{v} should be observe-class"
            );
        }
        // Operator-role class.
        for v in [verbs::RESTART, verbs::DEPLOY] {
            assert_eq!(
                classify_verb(v),
                Some(VerbClass::Normal(AccessRequirement {
                    required_caps: 0,
                    max_role: Role::OPERATOR,
                })),
                "{v} should be operator-role class"
            );
        }
        // Teleop class.
        assert_eq!(
            classify_verb(verbs::TELEOP),
            Some(VerbClass::Normal(AccessRequirement {
                required_caps: Scope::CAP_TELEOP,
                max_role: Role::OPERATOR,
            }))
        );
        // Deny-by-default for anything else.
        assert_eq!(classify_verb("frobnicate"), None);
        assert_eq!(classify_verb(""), None);
        assert_eq!(classify_verb("restart-now"), None);
    }

    #[test]
    fn authed_device_key_requires_authentication_and_a_32_byte_hex_id() {
        let key = [7u8; 32];
        let hexed = hex::encode(key);
        assert_eq!(
            authed_device_key(&CallerIdentity::verified(&hexed)),
            Some(key)
        );
        // Unauthenticated (dev UDS) → None even with a valid-looking id.
        assert_eq!(
            authed_device_key(&CallerIdentity {
                id: hexed.clone(),
                authenticated: false,
            }),
            None
        );
        // Authenticated but not a 32-byte key → None.
        assert_eq!(
            authed_device_key(&CallerIdentity::verified("local-dev")),
            None
        );
        assert_eq!(authed_device_key(&CallerIdentity::verified("dead")), None); // 2 bytes
        assert_eq!(
            authed_device_key(&CallerIdentity::verified(hex::encode([1u8; 33]))),
            None // 33 bytes
        );
    }
}
