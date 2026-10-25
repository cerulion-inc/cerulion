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
//! `cerulion/wire/2` accept. This module installs ONE trait so the LAN plane gains a gate and
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

use serde::{Deserialize, Serialize};

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
    /// The iroh WAN demand plane. The demand rides a `cerulion/wire/2` connection whose
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

    /// The flood latches a host logs this authorizer's LAN refusals through,
    /// when it keeps them. The default keeps none, so every refusal is
    /// logged loudly; [`LanAccessAuthorizer`] keeps one latch per request
    /// kind, because a paired machine refuses every LAN request.
    fn lan_refusal_latches(&self) -> Option<&LanRefusalLatches> {
        None
    }
}

/// One [`FailureRegimeLatch`](super::failure_regime_latch::FailureRegimeLatch)
/// per kind of LAN request ("demand", "catalog GET", ...), so repeated
/// refusals of one kind log one loud line while a first refusal of another
/// kind is still loud.
///
/// Each kind also remembers the resource its latest refusal named. Only an
/// authorized request for THAT resource closes the regime: an inner
/// authorizer may deny one topic and allow another, and an allowed topic
/// must not re-arm the loud path for the denied one or log a false
/// recovery. The state stays bounded: one latch and one resource name per
/// request kind, never one per topic.
#[derive(Debug, Default)]
pub struct LanRefusalLatches {
    latches: std::sync::Mutex<std::collections::BTreeMap<&'static str, RefusalRegime>>,
}

/// One request kind's refusal latch and the resource its latest refusal
/// named.
#[derive(Debug, Default)]
struct RefusalRegime {
    latch: super::failure_regime_latch::FailureRegimeLatch,
    refused_resource: String,
}

impl LanRefusalLatches {
    /// Record one refused `request` for `resource` and return how to log it.
    pub fn on_refusal(
        &self,
        request: &'static str,
        resource: &str,
    ) -> super::failure_regime_latch::RegimeDecision {
        let mut latches = self
            .latches
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let regime = latches.entry(request).or_default();
        if regime.refused_resource != resource {
            regime.refused_resource.clear();
            // hot-path-alloc-ok: a refused LAN request is a control-plane
            // reply, never the per-frame publish/receive path.
            regime.refused_resource.push_str(resource);
        }
        regime.latch.on_failure()
    }

    /// Record one authorized `request` for `resource`. It closes the
    /// regime only when `resource` is the one the latest refusal named, and
    /// returns `Some(suppressed)` when that closed regime counted repeats
    /// (log recovery once). An authorization for any other resource leaves
    /// the regime open and returns `None`.
    pub fn on_authorized(&self, request: &'static str, resource: &str) -> Option<u64> {
        self.latches
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .get_mut(request)
            .filter(|regime| regime.refused_resource == resource)
            .and_then(|regime| regime.latch.on_success())
    }

    /// Every refusal of `request` recorded so far.
    pub fn total_refusals(&self, request: &'static str) -> u64 {
        self.latches
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .get(request)
            .map_or(0, |regime| regime.latch.total_failures())
    }
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

/// The environment variable that selects a serving process's [`LanAccess`] posture.
pub const LAN_ACCESS_ENV: &str = "CERULION_LAN_ACCESS";

/// Who may demand this machine's topics over the zenoh LAN plane.
///
/// The LAN plane carries no paired identity, so the choice is binary: serve every
/// LAN peer ([`LanAccess::Open`]) or refuse them all and leave topic access to the
/// paired remote plane ([`LanAccess::Paired`]). Android devices default to
/// `Paired`; every other platform defaults to `Open`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LanAccess {
    /// Every LAN peer may enumerate and demand topics.
    Open,
    /// LAN demands, catalog, schema and run queries are refused, and LAN ingress
    /// delivers nothing.
    Paired,
}

impl LanAccess {
    /// The posture used when nothing selects one: `Paired` on Android, else `Open`.
    pub const fn platform_default() -> Self {
        if cfg!(target_os = "android") {
            LanAccess::Paired
        } else {
            LanAccess::Open
        }
    }

    /// Parse a posture value. An unset, empty or whitespace-only value selects
    /// [`Self::platform_default`]; `open` and `paired` match after trimming, ASCII
    /// case-insensitively. Any other value is returned as the error.
    pub fn parse(raw: Option<&str>) -> Result<Self, String> {
        let Some(value) = raw.map(str::trim).filter(|value| !value.is_empty()) else {
            return Ok(Self::platform_default());
        };
        if value.eq_ignore_ascii_case("open") {
            Ok(LanAccess::Open)
        } else if value.eq_ignore_ascii_case("paired") {
            Ok(LanAccess::Paired)
        } else {
            // hot-path-alloc-ok: posture parsing runs once at session setup.
            Err(value.to_string())
        }
    }

    /// Resolve a posture value, failing CLOSED: an unrecognized value selects
    /// `Paired` with a loud warning naming it.
    pub fn resolve(raw: Option<&str>) -> Self {
        Self::parse(raw).unwrap_or_else(|value| {
            tracing::warn!(
                var = LAN_ACCESS_ENV,
                value = %value,
                "unrecognized LAN access posture (expected open or paired); refusing unpaired \
                 LAN peers"
            );
            LanAccess::Paired
        })
    }

    /// Resolve the posture from [`LAN_ACCESS_ENV`] (see [`Self::resolve`]).
    pub fn from_env() -> Self {
        Self::from_env_result(std::env::var(LAN_ACCESS_ENV))
    }

    fn from_env_result(raw: Result<String, std::env::VarError>) -> Self {
        match raw {
            Ok(value) => Self::resolve(Some(&value)),
            Err(std::env::VarError::NotPresent) => Self::resolve(None),
            Err(std::env::VarError::NotUnicode(_)) => {
                tracing::warn!(
                    var = LAN_ACCESS_ENV,
                    "value is not valid Unicode; refusing unpaired LAN peers"
                );
                Self::Paired
            }
        }
    }

    /// The posture's configuration spelling.
    pub const fn as_str(self) -> &'static str {
        match self {
            LanAccess::Open => "open",
            LanAccess::Paired => "paired",
        }
    }
}

/// A [`DemandAuthorizer`] that applies a [`LanAccess`] posture in front of an inner
/// authorizer. Under [`LanAccess::Paired`] every [`DemandSubject::Lan`] demand is
/// refused; every other subject, and every subject under [`LanAccess::Open`], gets
/// the inner authorizer's verdict.
pub struct LanAccessAuthorizer {
    access: LanAccess,
    inner: std::sync::Arc<dyn DemandAuthorizer>,
    refusals: LanRefusalLatches,
}

impl LanAccessAuthorizer {
    /// Apply `access` in front of `inner`.
    pub fn new(access: LanAccess, inner: std::sync::Arc<dyn DemandAuthorizer>) -> Self {
        Self {
            access,
            inner,
            refusals: LanRefusalLatches::default(),
        }
    }

    /// The posture this authorizer applies.
    pub fn access(&self) -> LanAccess {
        self.access
    }
}

impl DemandAuthorizer for LanAccessAuthorizer {
    fn authorize_demand(&self, subject: &DemandSubject, topic: &str) -> DemandDecision {
        match (self.access, subject) {
            // hot-path-alloc-ok: a refused demand is a control-plane reply, never the
            // per-frame publish/receive path.
            (LanAccess::Paired, DemandSubject::Lan { .. }) => DemandDecision::deny(format!(
                "zenoh-lan request for '{topic}' refused: LAN access is paired-only and a \
                 LAN peer carries no paired identity; use the paired remote plane, or set \
                 the LAN access posture to open for local development"
            )),
            _ => self.inner.authorize_demand(subject, topic),
        }
    }

    fn lan_refusal_latches(&self) -> Option<&LanRefusalLatches> {
        Some(&self.refusals)
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

    #[test]
    fn lan_access_parses_both_postures_and_defaults_per_platform() {
        assert_eq!(LanAccess::parse(Some("open")), Ok(LanAccess::Open));
        assert_eq!(LanAccess::parse(Some(" PAIRED ")), Ok(LanAccess::Paired));
        assert_eq!(LanAccess::parse(Some("Open")), Ok(LanAccess::Open));
        let expected = if cfg!(target_os = "android") {
            LanAccess::Paired
        } else {
            LanAccess::Open
        };
        assert_eq!(LanAccess::parse(None), Ok(expected));
        assert_eq!(LanAccess::parse(Some("  ")), Ok(expected));
        assert_eq!(LanAccess::platform_default(), expected);
        assert_eq!(LanAccess::Open.as_str(), "open");
        assert_eq!(LanAccess::Paired.as_str(), "paired");
    }

    #[test]
    fn lan_access_fails_closed_on_an_unknown_value() {
        assert_eq!(LanAccess::parse(Some("yes")), Err("yes".to_string()));
        assert_eq!(LanAccess::resolve(Some("yes")), LanAccess::Paired);
        assert_eq!(LanAccess::resolve(Some("off")), LanAccess::Paired);
    }

    #[test]
    fn lan_access_from_env_result_uses_the_platform_default_when_unset() {
        assert_eq!(
            LanAccess::from_env_result(Err(std::env::VarError::NotPresent)),
            LanAccess::platform_default()
        );
    }

    #[test]
    fn lan_access_from_env_result_parses_open() {
        assert_eq!(
            LanAccess::from_env_result(Ok("open".to_string())),
            LanAccess::Open
        );
    }

    #[cfg(unix)]
    #[test]
    fn lan_access_from_env_result_fails_closed_on_non_unicode() {
        use std::os::unix::ffi::OsStringExt;

        let value = std::ffi::OsString::from_vec(vec![0xff]);
        assert_eq!(
            LanAccess::from_env_result(Err(std::env::VarError::NotUnicode(value))),
            LanAccess::Paired
        );
    }

    #[test]
    fn paired_lan_access_refuses_every_lan_request() {
        let gate = LanAccessAuthorizer::new(LanAccess::Paired, Arc::new(AllowAllAuthorizer));
        assert_eq!(gate.access(), LanAccess::Paired);
        for subject in [lan(), lan_at("tcp/10.0.0.1:7683")] {
            match gate.authorize_demand(&subject, "/app/heartbeat") {
                DemandDecision::Deny { reason } => assert_eq!(
                    reason,
                    "zenoh-lan request for '/app/heartbeat' refused: LAN access is \
                     paired-only and a LAN peer carries no paired identity; use the paired \
                     remote plane, or set the LAN access posture to open for local development"
                ),
                DemandDecision::Allow => panic!("paired LAN access admitted {subject}"),
            }
        }
    }

    #[test]
    fn paired_lan_access_defers_wan_demands_to_the_inner_authorizer() {
        let allow = LanAccessAuthorizer::new(LanAccess::Paired, Arc::new(AllowAllAuthorizer));
        assert_eq!(allow.authorize_demand(&wan(), "/t"), DemandDecision::Allow);
        let deny = LanAccessAuthorizer::new(LanAccess::Paired, Arc::new(DenyTopic("/t")));
        assert!(!deny.authorize_demand(&wan(), "/t").is_allowed());
        assert!(deny.authorize_demand(&wan(), "/other").is_allowed());
    }

    #[test]
    fn open_lan_access_defers_every_demand_to_the_inner_authorizer() {
        let gate = LanAccessAuthorizer::new(LanAccess::Open, Arc::new(DenyTopic("/t")));
        assert_eq!(gate.access(), LanAccess::Open);
        assert!(gate.authorize_demand(&lan(), "/other").is_allowed());
        assert!(!gate.authorize_demand(&lan(), "/t").is_allowed());
        assert!(!gate.authorize_demand(&wan(), "/t").is_allowed());
    }
}
