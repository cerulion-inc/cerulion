// SPDX-License-Identifier: MIT OR Apache-2.0
//! Access-list rows and ownership state — the durable, robot-local source of
//! truth for who may access the robot. Established pairings live here and never
//! rot offline; certs only gate *new* pairings.

use serde::{Deserialize, Serialize};

use crate::format::{AccountId, PrincipalKind, Scope};

/// How an access-list row was established.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum PairingSource {
    /// Established by verifying the offline certificate chain (strong path).
    StrongChain,
    /// Established by the CPace fallback ceremony (code pairing).
    CodePaired,
    /// The owner row, written by the first **physical-possession** claim (the
    /// chassis-secret [`crate::verify::TrustStore::claim`]). Since the owner grant landed this
    /// is the factory-reset / recovery path — the primary owner claim is
    /// [`PairingSource::OwnerGrant`].
    Claim,
    /// The owner row, written by a **login-gated owner grant**. The
    /// installer logs in at install, the account service issues an `OWNER_FULL`
    /// grant for the installer's account, and
    /// [`crate::verify::TrustStore::claim_by_owner_grant`] verifies that grant's
    /// offline chain and records the owner. Distinct from [`PairingSource::Claim`]
    /// (physical possession) so the owner row's provenance stays accurate.
    OwnerGrant,
    /// A guest row established from a **desk-carried, owner-signed access grant**.
    /// The robot's owner signs a [`crate::format::SignedAccessGrant`]
    /// for a subject account; the desk carries it and presents it at dial time, and
    /// [`crate::verify::TrustStore::establish_by_owner_grant`] verifies its offline
    /// chain against the robot's OWN owner and writes this row — no cloud contact.
    /// Distinct from [`PairingSource::OwnerGrant`] (the login-gated OWNER claim) so
    /// the provenance of a guest admitted by the owner stays accurate and revocable.
    OwnerSignedGrant,
}

/// One durable access-list entry.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct AccessRow {
    /// The account this row grants access to.
    pub account: AccountId,
    /// The scope granted.
    pub scope: Scope,
    /// Human vs. machine.
    pub principal_kind: PrincipalKind,
    /// How the row was established.
    pub source: PairingSource,
    /// A human-visible name (code-paired rows are visible/named/revocable).
    pub name: Option<String>,
    /// When the row was added (Unix ns).
    pub added_at_ns: u64,
    /// The row's finite access expiry (Unix ns), if any. An
    /// owner-signed-grant row ([`PairingSource::OwnerSignedGrant`]) carries the
    /// grant's `not_after_ns` here so the owner's time-boxed access is honored
    /// OFFLINE at every access check — an expired grant DENIES (see
    /// [`crate::verify::TrustStore::is_allowed`]) without an explicit revoke. Every
    /// OTHER pairing source (`Claim`/`OwnerGrant`/`StrongChain`/`CodePaired`) carries
    /// `None` — an established pairing never rots offline (an unbounded grant, whose
    /// `not_after_ns == u64::MAX`, likewise maps to `None`).
    pub expires_at_ns: Option<u64>,
    /// Whether the row has been revoked (by epoch or owner "revoke-now").
    pub revoked: bool,
}

/// Ownership: robots ship unclaimed; the first physical-possession pairing writes
/// the owner account.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum OwnershipState {
    /// No owner yet — an empty owner slot at ship time / after factory reset.
    Unclaimed,
    /// Owned by the given account.
    Claimed(AccountId),
}

impl OwnershipState {
    /// The owner account, if claimed.
    pub fn owner(&self) -> Option<AccountId> {
        match self {
            OwnershipState::Unclaimed => None,
            OwnershipState::Claimed(a) => Some(*a),
        }
    }
}

/// The result of applying a pushed revocation epoch.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EpochOutcome {
    /// The epoch was newer and was applied.
    Applied {
        /// How many previously-active rows this epoch newly revoked.
        newly_revoked: usize,
    },
    /// The epoch was not newer than the current one; nothing changed.
    NotNewer {
        /// The current (retained) epoch number.
        current: u64,
    },
}
