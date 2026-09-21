// SPDX-License-Identifier: AGPL-3.0-only
//! The ONE demand-authorization SEAM — the single gate BOTH network
//! demand planes (the zenoh LAN plane and the iroh WAN plane) pass through before a
//! topic's frames are served.
//!
//! # Why this exists (the access decision)
//!
//! Topic egress allow-lists cease to be authorization. The access decision
//! (the account-scoped access model)
//! makes the **account / pairing grant** the access boundary: a robot has an owner
//! account that grants per-account access, and BOTH demand planes must consult the
//! SAME `is_allowed(account)` check. The zenoh LAN demand path is UN-gated today; the
//! iroh WAN plane already gates on `cerulion_remoted::PairingAuthorizer` at the
//! `cerulion/wire/1` accept. This module installs ONE trait so the LAN plane gains a gate and
//! both planes route through one seam, and the account-grant authorizer plugs ONE
//! `is_allowed`-backed implementation into it without re-plumbing.
//!
//! # This module is PLUMBING only (deny-nothing default)
//!
//! The seam ownership is split: **this module is the plumbing, `cerulion_remoted`
//! supplies the predicate.** So the default implementation ([`AllowAllAuthorizer`])
//! DENIES NOTHING — it returns [`DemandDecision::Allow`] for every subject. That makes
//! every gated call site byte-identical to today's behavior (the anti-regression
//! contract); the change is purely that a gate now EXISTS to be swapped. The
//! `is_allowed`-backed predicate is `cerulion_remoted::PairingAuthorizer`'s
//! implementation of this trait, which **is INSTALLED on the robot's serving demand
//! plane** (`cerulion_remoted::wire`), the authoritative per-demand enforcement point.
//!
//! # Who carries the identity — the DEMANDER, at the enforcement point
//!
//! The seam is enforced where the demand's DEMANDER is cryptographically
//! authenticated:
//!
//! - The **iroh WAN plane**'s subject is [`DemandSubject::Wan`], carrying the
//!   **demander's** 32-byte ed25519 device key (the party asking for the
//!   topic, NEVER the dial target). The ROBOT serving plane
//!   (`cerulion_remoted::wire::serve_wire_connection`) threads the connection's
//!   authenticated remote peer key (`Connection::remote_id`) and installs
//!   `PairingAuthorizer` here, so an owner-signed grant / pairing is honored per topic
//!   with no re-plumbing, plus a mid-session sweep evicts a revoked demander's live
//!   stream. netd's desk-side WAN pre-flight (`cerulion_netd::iroh_plane`) threads its
//!   OWN desk key (netd is the demander) but stays deny-nothing — the robot enforces.
//! - The **zenoh LAN plane**'s demand token (a `cerulion_lv` liveliness Put / a
//!   `cerulion_q` demand GET) carries NO authenticated identity today — at most a
//!   network locator. So its subject is [`DemandSubject::Lan`], carrying an optional
//!   locator; an account folds in once the LAN plane carries a paired identity.
//!
//! One trait, two subject variants, one implementation installed at the robot.

use std::fmt;

/// The authenticated identity of the party a demand is authorized FOR, modeled per
/// plane (the two demand planes carry different identity at the gate — see the module
/// docs). Kept deliberately free of account types: this module is plumbing, and the
/// account-grant authorizer owns the `AccountId` resolution.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum DemandSubject {
    /// The zenoh LAN demand plane. The inbound demand (a `cerulion_lv` liveliness Put
    /// or a `cerulion_q` demand GET) carries NO authenticated identity today — at most
    /// the demanding peer's network locator. `None` is the "no identity yet"
    /// case the LAN plane passes today; an account is folded in once the LAN plane carries one.
    Lan {
        /// The demanding peer's network locator, if the plane surfaces one. `None`
        /// today (the zenoh demand callbacks do not thread the querier's identity yet).
        locator: Option<String>,
    },
    /// The iroh WAN demand plane. The demand rides a `cerulion/wire/1` connection whose
    /// TLS layer mutually-authenticates a 32-byte ed25519 device key. The
    /// subject carries the **demander's** identity (the party asking for the topic), so
    /// an `is_allowed`-backed authorizer resolves the RIGHT account — NEVER the dial
    /// target's. The authoritative per-demand enforcement point is the ROBOT serving
    /// plane (`cerulion_remoted::wire`); netd's desk-side pre-flight threads its OWN
    /// desk key (netd is the demander) but stays deny-nothing.
    Wan {
        /// The authenticated 32-byte ed25519 device key of the party DEMANDING the
        /// topic. On the robot serving plane this is the connection's
        /// authenticated remote peer key (`Connection::remote_id`) — the demanding
        /// desk; netd's desk-side pre-flight passes its own desk device key. Either
        /// resolves to an account via the device→account side-map + `is_allowed`.
        demander_key: [u8; 32],
    },
}

impl DemandSubject {
    /// A stable, log-friendly label for the plane this subject belongs to — used in the
    /// refusal breadcrumbs so a denied demand names WHICH plane refused it.
    pub fn plane_label(&self) -> &'static str {
        match self {
            DemandSubject::Lan { .. } => "zenoh-lan",
            DemandSubject::Wan { .. } => "iroh-wan",
        }
    }
}

impl fmt::Display for DemandSubject {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DemandSubject::Lan { locator } => match locator {
                Some(l) => write!(f, "zenoh-lan peer @ {l}"),
                None => write!(f, "zenoh-lan peer (no authenticated identity yet)"),
            },
            // Only the first 8 bytes of the key are rendered — enough to disambiguate
            // in a log without dumping the whole key material.
            DemandSubject::Wan { demander_key } => {
                write!(f, "iroh-wan device 0x")?;
                for b in &demander_key[..8] {
                    write!(f, "{b:02x}")?;
                }
                write!(f, "…")
            }
        }
    }
}

/// The verdict of a [`DemandAuthorizer`]: admit the demand, or refuse it with a loud,
/// human-readable reason (surfaced in the refusal breadcrumb + the caller's error).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DemandDecision {
    /// The demand is authorized — the plane proceeds to serve the topic.
    Allow,
    /// The demand is refused — the plane must NOT serve the topic. `reason` is a loud,
    /// user-facing explanation (never a silent drop — Principle #3 / the "loud over
    /// silent" house rule).
    Deny {
        /// A human-readable explanation of the refusal.
        reason: String,
    },
}

impl DemandDecision {
    /// Construct a refusal from any string-like reason.
    pub fn deny(reason: impl Into<String>) -> Self {
        DemandDecision::Deny {
            reason: reason.into(),
        }
    }

    /// Whether this decision admits the demand.
    pub fn is_allowed(&self) -> bool {
        matches!(self, DemandDecision::Allow)
    }
}

/// The ONE demand-authorization seam. An implementation decides whether a demand from
/// `subject` for `topic` is authorized. BOTH network demand planes consult the SAME
/// trait object, so a single implementation (the account-grant authorizer)
/// gates the whole machine's network access.
///
/// `Send + Sync`: shared behind an `Arc<dyn DemandAuthorizer>` across the demand
/// callbacks / plane threads.
pub trait DemandAuthorizer: Send + Sync {
    /// Authorize (or refuse) a demand from `subject` for `topic`. The default
    /// [`AllowAllAuthorizer`] admits everything (the deny-nothing plumbing); the account-grant
    /// implementation resolves the subject to an account and returns `is_allowed`.
    fn authorize_demand(&self, subject: &DemandSubject, topic: &str) -> DemandDecision;
}

/// The DEFAULT authorizer — deny-nothing. Admits every demand on every plane, so a
/// gated call site is byte-identical to an un-gated one. An account-grant authorizer
/// replaces this where installed; otherwise the seam exists but withholds nothing
/// (matching the "post-pairing = allow-all topics by default" interim, and the
/// permissive-LAN default that the account gate later tightens).
#[derive(Debug, Default, Clone, Copy)]
pub struct AllowAllAuthorizer;

impl DemandAuthorizer for AllowAllAuthorizer {
    fn authorize_demand(&self, _subject: &DemandSubject, _topic: &str) -> DemandDecision {
        DemandDecision::Allow
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    /// A hand oracle authorizer that DENIES exactly one named topic (any plane) — the
    /// stub the real `is_allowed` authorizer stands in for. NOT a self-compare:
    /// the tests assert against fixed expected verdicts, never against a second run of
    /// the same closure.
    struct DenyTopic(&'static str);
    impl DemandAuthorizer for DenyTopic {
        fn authorize_demand(&self, subject: &DemandSubject, topic: &str) -> DemandDecision {
            if topic == self.0 {
                // hot-path-alloc-ok: test-only hand-oracle authorizer; deny-message
                // formatting runs per refused DEMAND (control plane), never on the
                // per-frame publish/receive path.
                DemandDecision::deny(format!(
                    "{} not authorized for topic {topic}",
                    subject.plane_label()
                ))
            } else {
                DemandDecision::Allow
            }
        }
    }

    fn lan() -> DemandSubject {
        DemandSubject::Lan { locator: None }
    }
    fn lan_at(l: &str) -> DemandSubject {
        DemandSubject::Lan {
            // hot-path-alloc-ok: test-only subject builder.
            locator: Some(l.to_string()),
        }
    }
    fn wan() -> DemandSubject {
        DemandSubject::Wan {
            demander_key: [0xab; 32],
        }
    }

    #[test]
    fn allow_all_admits_every_subject_and_topic() {
        let auth = AllowAllAuthorizer;
        // Deny-nothing on BOTH plane variants + arbitrary topics.
        assert_eq!(auth.authorize_demand(&lan(), "/a"), DemandDecision::Allow);
        assert_eq!(
            auth.authorize_demand(&lan_at("tcp/1.2.3.4:7683"), "/b"),
            DemandDecision::Allow
        );
        assert_eq!(auth.authorize_demand(&wan(), "/c"), DemandDecision::Allow);
        assert!(auth.authorize_demand(&wan(), "/tf").is_allowed());
    }

    #[test]
    fn allow_all_is_the_default() {
        // The Default impl must be the deny-nothing authorizer (the seam's contract).
        let auth = AllowAllAuthorizer;
        assert!(auth.authorize_demand(&lan(), "/anything").is_allowed());
    }

    #[test]
    fn deny_verdict_carries_a_reason_naming_the_plane() {
        let auth = DenyTopic("/secret");
        // The denied topic refuses on EITHER plane, and the reason names the plane
        // (so the refusal breadcrumb is attributable).
        match auth.authorize_demand(&lan(), "/secret") {
            DemandDecision::Deny { reason } => {
                assert!(reason.contains("zenoh-lan"), "reason={reason}");
                assert!(reason.contains("/secret"), "reason={reason}");
            }
            other => panic!("expected Deny, got {other:?}"),
        }
        match auth.authorize_demand(&wan(), "/secret") {
            DemandDecision::Deny { reason } => {
                assert!(reason.contains("iroh-wan"), "reason={reason}")
            }
            other => panic!("expected Deny, got {other:?}"),
        }
    }

    #[test]
    fn deny_authorizer_admits_untargeted_topics() {
        // Anti-tautology: a topic the stub does NOT target is still admitted, on both
        // planes — the gate is selective, not a blanket deny.
        let auth = DenyTopic("/secret");
        assert!(auth.authorize_demand(&lan(), "/public").is_allowed());
        assert!(auth.authorize_demand(&wan(), "/public").is_allowed());
    }

    #[test]
    fn plane_label_and_display_disambiguate_subjects() {
        assert_eq!(lan().plane_label(), "zenoh-lan");
        assert_eq!(wan().plane_label(), "iroh-wan");
        // Display renders a locator when present, and only a key PREFIX for WAN (never
        // the whole key material).
        assert!(lan_at("tcp/1.2.3.4:7683")
            .to_string() // hot-path-alloc-ok: test-only Display assert.
            .contains("1.2.3.4:7683"));
        // hot-path-alloc-ok: test-only Display assert.
        assert!(lan().to_string().contains("no authenticated identity"));
        // First 8 bytes are the rendered prefix; byte 8 is 0xff and must NOT appear.
        let mut key = [0xffu8; 32];
        key[..8].copy_from_slice(&[0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]);
        let s = DemandSubject::Wan { demander_key: key }.to_string(); // hot-path-alloc-ok: test-only Display assert.
        assert!(s.contains("0123456789abcdef"), "display={s}");
        assert!(
            !s.contains("ffff"),
            "must render only the 8-byte prefix: {s}"
        );
    }

    #[test]
    fn authorizer_is_object_safe_and_shareable() {
        // The seam is used as `Arc<dyn DemandAuthorizer>` across threads — pin that it
        // is object-safe + Send + Sync (a compile-time contract exercised at runtime).
        let auth: Arc<dyn DemandAuthorizer> = Arc::new(AllowAllAuthorizer);
        let handle = std::thread::spawn(move || auth.authorize_demand(&wan(), "/x").is_allowed());
        assert!(handle.join().unwrap());
    }
}
