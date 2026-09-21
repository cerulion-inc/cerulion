// SPDX-License-Identifier: AGPL-3.0-only
//! The PUBLIC beacon-facts file — the endpoint facts only
//! `remoted` can know, published for the gateway's mDNS TXT enrichment.
//!
//! ## The contract
//!
//! `remoted` owns the ONE iroh endpoint; the LAN gateway (a SEPARATE process,
//! in `cerulion_cli_engine`) owns the mDNS `_cerulion._tcp` beacon. The gateway
//! cannot see the endpoint (it does not link iroh — that would pull ~390 crates
//! into the plain `cargo build`, R10), so `remoted` writes the three facts only
//! IT can know — the endpoint's public key (`eid`), its bound UDP port
//! (`iroh_port`), and whether the robot is still claimable — to a small PUBLIC
//! JSON file the gateway reads at advertise time.
//!
//! ## On-disk shape (PUBLIC — no MAC)
//!
//! ```json
//! { "version": 1, "eid": "<64-hex EndpointId>", "iroh_port": 41234, "claimable": "1" }
//! ```
//!
//! It carries NO secrets (the `eid` IS the public key; ports + the claimable
//! flag are public), so — unlike the trust store + device index — it is NOT
//! MAC-authenticated. A tampered facts file can only make the mDNS TXT advertise
//! a wrong `eid`/port (a hint); the actual dial still TLS-authenticates against
//! the real key, and mDNS TXT is itself unauthenticated on the LAN. `claimable`
//! is a STRING `"0"`/`"1"` so the gateway passes it into the TXT record
//! verbatim.
//!
//! **The reader lives in `cerulion_cli_engine::mdns_discovery`** (it cannot link
//! this crate — iroh), so the two sides share this shape by convention; keep
//! them in lockstep (both pin it with the identical literal-JSON oracle).
//!
//! ## Lifecycle
//!
//! [`write_at_startup`] publishes the facts once the endpoint is bound (a
//! failure is a loud warn, never fatal — the facts are an additive overlay).
//! [`refresh`] is the SEAM the claim / pair verb handlers call after
//! mutating ownership: it rewrites the FILE (so `claimable` becomes `"0"` on
//! disk). NOTE — this does NOT by itself flip the ALREADY-ADVERTISED mDNS TXT:
//! the gateway reads the facts file ONCE at advertise time and registers a
//! static `ServiceInfo`, so a discovering peer sees the updated `claimable` only
//! after the gateway next (re)starts and re-reads the file. Making a live claim
//! flip the advertised TXT (the gateway re-reading + re-registering the
//! `ServiceInfo`) would belong in the claim handler and is NOT wired
//! here. Writes are atomic (`.tmp` then `rename`) so a reader never observes a
//! torn/partial file.

use std::path::{Path, PathBuf};

use cerulion_link::Endpoint;
use cerulion_pairing::verify::OwnershipState;
use serde::{Deserialize, Serialize};

use crate::error::RemotedError;

/// The on-disk format version (bumped on a breaking layout change). The reader
/// tolerates an unknown version best-effort (the facts are additive).
pub const BEACON_FACTS_VERSION: u16 = 1;

/// The public endpoint facts published for mDNS TXT enrichment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BeaconFacts {
    /// The endpoint's public key, hex-encoded (the mDNS `eid=` TXT value).
    pub eid: String,
    /// The endpoint's bound UDP port (the mDNS `iroh_port=` TXT value — DISTINCT
    /// from the zenoh SRV/TCP port).
    pub iroh_port: u16,
    /// The mDNS `claimable=` TXT value: `"1"` (unclaimed) / `"0"` (claimed).
    pub claimable: String,
}

/// The serde view of the on-disk file (adds the `version` frame).
#[derive(Debug, Serialize, Deserialize)]
struct OnDisk {
    version: u16,
    eid: String,
    iroh_port: u16,
    claimable: String,
}

impl BeaconFacts {
    /// Assemble the facts from the endpoint's public key bytes + its bound UDP
    /// port + the current ownership state. Pure.
    pub fn from_parts(endpoint_id: &[u8; 32], iroh_port: u16, ownership: OwnershipState) -> Self {
        BeaconFacts {
            eid: hex::encode(endpoint_id),
            iroh_port,
            claimable: claimable_flag(ownership).to_string(),
        }
    }

    /// Atomically write the facts to `path` (`.tmp` then `rename`, so a reader
    /// never sees a torn file). A failure is a [`RemotedError::BeaconFacts`]
    /// carrying the path + the failed operation.
    ///
    /// # Concurrency
    ///
    /// The temp file has a FIXED `.tmp` sibling name (mirroring the trust store +
    /// device index atomic-write convention), so this is NOT safe to call
    /// concurrently on the same `path`. That holds today: the startup write is
    /// one-shot, and the claim/pair handlers that call [`refresh`] are
    /// serialized through the `&mut TrustStore` mutation they follow, so at most
    /// one facts write is ever in flight.
    pub fn write_atomically(&self, path: &Path) -> Result<(), RemotedError> {
        let on_disk = OnDisk {
            version: BEACON_FACTS_VERSION,
            eid: self.eid.clone(),
            iroh_port: self.iroh_port,
            claimable: self.claimable.clone(),
        };
        let body = serde_json::to_vec_pretty(&on_disk)
            .map_err(|e| RemotedError::BeaconFacts(format!("serialize: {e}")))?;
        let tmp = tmp_path(path);
        std::fs::write(&tmp, &body).map_err(|e| {
            RemotedError::BeaconFacts(format!(
                "{}: writing the beacon facts failed: {e}",
                tmp.display()
            ))
        })?;
        std::fs::rename(&tmp, path).map_err(|e| {
            RemotedError::BeaconFacts(format!(
                "{}: renaming the beacon facts into place failed: {e}",
                path.display()
            ))
        })?;
        Ok(())
    }

    /// Read + parse a facts file (the reader half — exercised by the round-trip
    /// tests; the production reader is the gateway's, in `cerulion_cli_engine`).
    pub fn load(path: &Path) -> Result<Self, RemotedError> {
        let bytes = std::fs::read(path).map_err(|e| {
            RemotedError::BeaconFacts(format!(
                "{}: reading the beacon facts failed: {e}",
                path.display()
            ))
        })?;
        let on_disk: OnDisk = serde_json::from_slice(&bytes).map_err(|e| {
            RemotedError::BeaconFacts(format!(
                "{}: parsing the beacon facts failed: {e}",
                path.display()
            ))
        })?;
        Ok(BeaconFacts {
            eid: on_disk.eid,
            iroh_port: on_disk.iroh_port,
            claimable: on_disk.claimable,
        })
    }
}

/// The mDNS `claimable` TXT value for an ownership state: `"1"` when the robot
/// is UNCLAIMED (a fresh robot advertises itself claimable), `"0"` once CLAIMED.
/// Pure so it is oracle-testable.
pub fn claimable_flag(ownership: OwnershipState) -> &'static str {
    match ownership {
        OwnershipState::Unclaimed => "1",
        OwnershipState::Claimed(_) => "0",
    }
}

/// Recompute + atomically rewrite the beacon-facts FILE — the SEAM the claim /
/// pair verb handlers call after mutating ownership so the on-disk
/// `claimable` fact tracks the new state. Reads the endpoint's bound UDP port
/// (via [`cerulion_link::bound_port`]) and public key; a returned `Err` lets the
/// caller decide how loud to be ([`write_at_startup`] demotes it to a warn).
///
/// This updates the file only. The gateway reads it ONCE at advertise time, so
/// the ALREADY-ADVERTISED mDNS TXT does not change until the gateway next
/// (re)starts — driving a live re-advertise off a mid-session claim is
/// not wired here (see the module docs).
pub fn refresh(
    beacon_facts_file: &Path,
    endpoint: &Endpoint,
    ownership: OwnershipState,
) -> Result<(), RemotedError> {
    let port = cerulion_link::bound_port(endpoint).ok_or_else(|| {
        RemotedError::BeaconFacts(
            "the endpoint reports no bound UDP port; cannot publish the iroh_port beacon fact \
             (the LAN direct-dial hint)"
                .to_string(),
        )
    })?;
    let facts = BeaconFacts::from_parts(endpoint.id().as_bytes(), port, ownership);
    facts.write_atomically(beacon_facts_file)?;
    tracing::info!(
        eid = %facts.eid,
        iroh_port = facts.iroh_port,
        claimable = %facts.claimable,
        path = %beacon_facts_file.display(),
        "cerulion_remoted: published beacon facts for mDNS TXT enrichment (eid/iroh_port/claimable)"
    );
    Ok(())
}

/// Best-effort startup publish of the beacon facts. Calls [`refresh`]; a failure
/// is a LOUD `warn!` (the mDNS TXT enrichment is omitted this run) but NEVER
/// fatal — the facts are an additive overlay, and the remote plane serves
/// regardless. Called once after the endpoint binds.
pub fn write_at_startup(beacon_facts_file: &Path, endpoint: &Endpoint, ownership: OwnershipState) {
    if let Err(e) = refresh(beacon_facts_file, endpoint, ownership) {
        tracing::warn!(
            error = %e,
            path = %beacon_facts_file.display(),
            "cerulion_remoted: failed to publish beacon facts; the gateway's mDNS TXT \
             enrichment (eid/iroh_port/claimable) will be OMITTED this run. Additive + \
             non-fatal — the remote plane serves regardless."
        );
    }
}

/// Sibling temp path for atomic writes: append `.tmp` to the file name (mirrors
/// the trust store + device index atomic-write convention).
fn tmp_path(path: &Path) -> PathBuf {
    let mut name = path.file_name().unwrap_or_default().to_os_string();
    name.push(".tmp");
    path.with_file_name(name)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_pairing::format::AccountId;

    #[test]
    fn claimable_flag_is_1_unclaimed_and_0_claimed() {
        // Hand oracle: an unclaimed robot advertises claimable="1"; a claimed
        // one "0" (regardless of WHICH account claimed it).
        assert_eq!(claimable_flag(OwnershipState::Unclaimed), "1");
        assert_eq!(
            claimable_flag(OwnershipState::Claimed(AccountId([9u8; 32]))),
            "0"
        );
    }

    #[test]
    fn from_parts_stamps_eid_port_and_flips_claimable_with_ownership() {
        // eid = hex of the key; port carried verbatim; claimable derives from
        // ownership (the flip that the claim verb will trigger via `refresh`).
        let key = [1u8; 32];
        let unclaimed = BeaconFacts::from_parts(&key, 41234, OwnershipState::Unclaimed);
        assert_eq!(unclaimed.eid, hex::encode(key));
        assert_eq!(unclaimed.iroh_port, 41234);
        assert_eq!(unclaimed.claimable, "1");

        let claimed =
            BeaconFacts::from_parts(&key, 41234, OwnershipState::Claimed(AccountId([2u8; 32])));
        // Only claimable flips; eid + port are identical.
        assert_eq!(claimed.eid, unclaimed.eid);
        assert_eq!(claimed.iroh_port, unclaimed.iroh_port);
        assert_eq!(claimed.claimable, "0");
    }
}
