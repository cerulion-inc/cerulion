// SPDX-License-Identifier: MIT OR Apache-2.0
//! Anti-rollback decisions for team statements and the local clock. Pure: the
//! caller passes the high-water marks it has persisted and stores the new ones
//! after [`RollbackDecision::Apply`].

use super::statement::{SignedTeamStatusList, SignedTeamTrustBundle, Verified};
use super::TeamId;
use crate::PairingError;

/// Whether a verified statement replaces what the node holds.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RollbackDecision {
    /// Newer than the held one: install it and raise the high-water mark.
    Apply,
    /// The same position as the held one: nothing to do.
    NoChange,
}

/// The newest trust bundle a node has installed for a team.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct BundleHighWater {
    /// The team.
    pub team_id: TeamId,
    /// The installed generation.
    pub generation: u32,
    /// The installed bundle's issuance time, Unix nanoseconds.
    pub issued_at_ns: u64,
}

/// The newest status list a node has applied for a team.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct StatusHighWater {
    /// The team.
    pub team_id: TeamId,
    /// The applied list sequence.
    pub seq: u64,
}

/// Decide on a verified bundle against the held mark (`None` when the node
/// holds no bundle for any team yet).
///
/// Ordered by (generation, issued_at): a higher generation applies; at the
/// same generation a later issuance applies (the service reissues a bundle
/// when a robot joins), the same issuance is [`RollbackDecision::NoChange`],
/// and anything lower is refused. A bundle for another team is refused.
pub fn decide_bundle(
    held: Option<&BundleHighWater>,
    candidate: &Verified<SignedTeamTrustBundle>,
) -> Result<RollbackDecision, PairingError> {
    let b = &candidate.get().bundle;
    let Some(held) = held else {
        return Ok(RollbackDecision::Apply);
    };
    if held.team_id != b.team_id {
        return Err(PairingError::TeamMismatch);
    }
    let offered = (b.generation, b.issued_at_ns);
    let have = (held.generation, held.issued_at_ns);
    match offered.cmp(&have) {
        core::cmp::Ordering::Greater => Ok(RollbackDecision::Apply),
        core::cmp::Ordering::Equal => Ok(RollbackDecision::NoChange),
        core::cmp::Ordering::Less if b.generation < held.generation => {
            Err(PairingError::TeamRollback {
                what: "bundle generation",
                held: held.generation as u64,
                offered: b.generation as u64,
            })
        }
        core::cmp::Ordering::Less => Err(PairingError::TeamRollback {
            what: "bundle issuance time",
            held: held.issued_at_ns,
            offered: b.issued_at_ns,
        }),
    }
}

/// Decide on a verified status list against the held mark (`None` when the
/// node holds no list yet): a higher sequence applies, the same sequence is
/// [`RollbackDecision::NoChange`], a lower one is refused, and a list for
/// another team is refused.
pub fn decide_status_list(
    held: Option<&StatusHighWater>,
    candidate: &Verified<SignedTeamStatusList>,
) -> Result<RollbackDecision, PairingError> {
    let l = &candidate.get().list;
    let Some(held) = held else {
        return Ok(RollbackDecision::Apply);
    };
    if held.team_id != l.team_id {
        return Err(PairingError::TeamMismatch);
    }
    match l.seq.cmp(&held.seq) {
        core::cmp::Ordering::Greater => Ok(RollbackDecision::Apply),
        core::cmp::Ordering::Equal => Ok(RollbackDecision::NoChange),
        core::cmp::Ordering::Less => Err(PairingError::TeamRollback {
            what: "status list sequence",
            held: held.seq,
            offered: l.seq,
        }),
    }
}

/// Refuse a clock below the larger of the persisted clock high-water mark and
/// the binary's build time (all Unix nanoseconds). After `Ok`, the caller may
/// raise its persisted mark to `now_ns`.
pub fn check_clock(
    now_ns: u64,
    persisted_high_water_ns: u64,
    build_time_ns: u64,
) -> Result<(), PairingError> {
    let floor = persisted_high_water_ns.max(build_time_ns);
    if now_ns < floor {
        return Err(PairingError::RollbackDetected {
            now_ns,
            high_water_ns: floor,
        });
    }
    Ok(())
}
