// SPDX-License-Identifier: AGPL-3.0-only
//! One-per-process DDS discovery participant — a generalized copy of
//! `examples/go2/lib/cerulion_go2_dds/src/participant.rs` (the working
//! precedent), parameterized on domain + `only_networks` for any robot.
//!
//! # One participant per process (Principle #8 analog)
//!
//! A process-global `AtomicBool` guards a single LIVE participant (released on
//! `Drop`, so build → drop → rebuild is fine). A second [`DiscoveryParticipant::new`]
//! while one is live returns [`DdsError::ParticipantAlreadyExists`] rather than
//! duplicating discovery traffic.
//!
//! # `only_networks` is REQUIRED on a multi-homed host
//!
//! CycloneDDS (all versions) DROPS fragmented builtin (SPDP/SEDP) discovery
//! data, and rustdds fragments a builtin sample once it exceeds ~1.4 KB — which
//! a many-interface host reaches because rustdds advertises EVERY local address
//! as a unicast locator. Restricting to the robot-LAN interface via
//! `with_only_networks` keeps the discovery payload under the threshold. See
//! `cerulion_go2_dds::participant` for the full research trail.

use std::net::IpAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use ros2_client::rustdds::discovery::{DiscoveredReaderData, DiscoveredWriterData};
use ros2_client::{Context, Node, NodeName, NodeOptions};

use crate::DdsError;

/// The ROS 2 node namespace the discovery node is created under.
pub const NODE_NAMESPACE: &str = "/cerulion_attach";

/// The default SPDP participant lease duration this discovery participant (and
/// the wire acquirer, which builds one) advertises. A SHORT lease so a
/// SIGKILLed/crashed `cerulion graph run attach` process's orphaned DDS readers
/// age out on the robot's writers in seconds rather than the stock rustdds
/// ~50 s (`5 * SPDP_PUBLISH_PERIOD`) — the ghost-reader window implicated in the
/// lidar subscription wedge. Floor rationale: the rustdds discovery event loop
/// is single-threaded and the SPDP announce period derives as `lease / 5` (2 s
/// here), so a much shorter lease risks FALSE lease-timeout evictions of a
/// healthy-but-busy peer.
pub const DEFAULT_LEASE_DURATION: Duration = Duration::from_secs(10);

// ─────────────────────────── One-per-process guard ─────────────────────────

/// Process-global "a participant is live" flag. `AtomicBool` (not `Once`)
/// deliberately: the slot must be RELEASABLE on `Drop` and via the test seam.
static PARTICIPANT_CLAIMED: AtomicBool = AtomicBool::new(false);

/// RAII claim on the single-participant slot; releases on `Drop`.
#[derive(Debug)]
struct ParticipantClaim;

fn claim_participant_slot() -> Result<ParticipantClaim, DdsError> {
    if PARTICIPANT_CLAIMED.swap(true, Ordering::SeqCst) {
        Err(DdsError::ParticipantAlreadyExists)
    } else {
        Ok(ParticipantClaim)
    }
}

impl Drop for ParticipantClaim {
    fn drop(&mut self) {
        PARTICIPANT_CLAIMED.store(false, Ordering::SeqCst);
    }
}

/// Test-only force-release of the process-global participant slot (models a
/// panic-leaked claim). `#[doc(hidden)]` + the `_for_test` name keep it out of
/// production API.
#[doc(hidden)]
pub fn reset_participant_slot_for_test() {
    PARTICIPANT_CLAIMED.store(false, Ordering::SeqCst);
}

// ─────────────────────────── Participant wrapper ───────────────────────────

/// A single-per-process DDS discovery participant. Wraps the ros2-client
/// [`Context`] (the rustdds `DomainParticipant`) and hands out discovery
/// [`Node`]s.
pub struct DiscoveryParticipant {
    context: Context,
    // Released on Drop -> frees the process slot. Held only for its Drop.
    _claim: ParticipantClaim,
}

impl DiscoveryParticipant {
    /// Create the process participant on `domain_id`, restricting rustdds to
    /// `only_networks` when non-empty (the multi-homed-discovery fix) and
    /// advertising the [`DEFAULT_LEASE_DURATION`] (10 s) SPDP lease. Errors with
    /// [`DdsError::ParticipantAlreadyExists`] if one is already live. Use
    /// [`Self::new_with_lease`] to override the lease.
    pub fn new(domain_id: u16, only_networks: &[IpAddr]) -> Result<Self, DdsError> {
        Self::new_with_lease(domain_id, only_networks, DEFAULT_LEASE_DURATION)
    }

    /// Like [`Self::new`] but with an explicit SPDP participant lease
    /// duration. Keep the lease at least a few seconds; see
    /// [`DEFAULT_LEASE_DURATION`] for the single-threaded-event-loop floor.
    pub fn new_with_lease(
        domain_id: u16,
        only_networks: &[IpAddr],
        lease_duration: Duration,
    ) -> Result<Self, DdsError> {
        // Claim FIRST so two concurrent constructors cannot both build; any
        // `?`-return below drops the claim and frees the slot (RAII).
        let claim = claim_participant_slot()?;

        if only_networks.is_empty() {
            tracing::warn!(
                "cerulion_dds: no only_networks restriction — rustdds will advertise EVERY local \
                 interface as a DDS locator. On a multi-homed host that bloats SPDP/SEDP past \
                 ~1.4 KB and CycloneDDS DROPS the fragmented discovery data (discovery silently \
                 fails). Pass --iface <robot-LAN-IP>."
            );
        }

        // Build via `DomainParticipantBuilder` on BOTH the restricted and
        // unrestricted paths (unlike the earlier `Context::with_options`
        // shortcut for the empty case) so the SPDP participant lease is set
        // regardless — stock `ContextOptions` exposes no lease knob. The empty
        // case matches what `Context::with_options` does internally, plus
        // `.participant_lease_duration(..)`.
        let mut builder = ros2_client::rustdds::DomainParticipantBuilder::new(domain_id)
            .participant_lease_duration(lease_duration);
        if !only_networks.is_empty() {
            builder = builder.with_only_networks(only_networks.iter().copied());
        }
        let participant = builder.build().map_err(|e| DdsError::ParticipantBuild {
            domain_id,
            only_networks: only_networks.to_vec(),
            cause: format!("{e:?}"),
        })?;
        let context =
            Context::from_domain_participant(participant).map_err(|e| DdsError::ContextBuild {
                domain_id,
                cause: format!("{e:?}"),
            })?;

        tracing::info!(
            domain = domain_id,
            only_networks = ?only_networks,
            lease_secs = lease_duration.as_secs_f64(),
            "cerulion_dds: DDS discovery participant created"
        );
        Ok(Self {
            context,
            _claim: claim,
        })
    }

    /// This participant's GUID — the STRUCTURAL identity every endpoint it
    /// creates shares a `prefix` with. The live-P0 self-endpoint filter
    /// matches discovered endpoint GUID prefixes against this one (never a
    /// name-prefix heuristic): our own discovery node's parameter-service
    /// readers/writers must not show up in the report as robot topics.
    /// Returns the whole [`GUID`](ros2_client::rustdds::GUID) because rustdds
    /// does not re-export `GuidPrefix` at its root (`structure` is
    /// `pub(crate)`); callers compare the pub `prefix` fields directly.
    pub fn guid(&self) -> ros2_client::rustdds::GUID {
        use ros2_client::rustdds::RTPSEntity as _;
        self.context.domain_participant().guid()
    }

    /// Snapshot every REMOTE writer (publisher) endpoint this participant's
    /// internal DiscoveryDB has seen — cloned `DiscoveredWriterData`, each
    /// carrying the endpoint's SEDP `USER_DATA` (where the REP-2011 RIHS01 type
    /// hash rides).
    ///
    /// This is the LOSS-PROOF endpoint harvest (the wire rung): the
    /// DiscoveryDB is maintained by rustdds's own discovery threads, so it
    /// reflects every endpoint SEDP observed during the window regardless of the
    /// bounded ros2-client status channel (`async_channel::bounded(8)`, which
    /// drops the SEDP burst on a large graph). rustdds 0.14.2 provides this
    /// accessor and the endpoint `USER_DATA` it exposes; the published
    /// cerulion-rustdds fork is retained only for its participant lease
    /// duration builder knob.
    pub fn discovered_writers(&self) -> Vec<DiscoveredWriterData> {
        self.context.domain_participant().discovered_writers()
    }

    /// Snapshot every REMOTE reader (subscriber) endpoint — the reader-side twin
    /// of [`DiscoveryParticipant::discovered_writers`] (the wire rung falls back
    /// to a robot-side subscriber's hash when no publisher carried one). This
    /// is the same loss-proof, `USER_DATA`-bearing snapshot supplied by
    /// rustdds 0.14.2.
    pub fn discovered_readers(&self) -> Vec<DiscoveredReaderData> {
        self.context.domain_participant().discovered_readers()
    }

    /// Create the discovery ROS 2 [`Node`] under [`NODE_NAMESPACE`], with
    /// rosout logging disabled (keeps the discovery node off `/rosout`).
    pub fn create_node(&self, node_name: &str) -> Result<Node, DdsError> {
        let name = NodeName::new(NODE_NAMESPACE, node_name).map_err(|e| DdsError::NodeName {
            name: node_name.to_string(),
            cause: format!("{e:?}"),
        })?;
        self.context
            .new_node(name, NodeOptions::new().enable_rosout(false))
            .map_err(|e| DdsError::NodeCreate {
                name: node_name.to_string(),
                cause: format!("{e:?}"),
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The one-per-process guard lifecycle, folded into ONE test body so it is
    /// the SOLE toucher of the process-global `PARTICIPANT_CLAIMED` — hence
    /// parallel-safe without `#[serial]`. (The real-participant construction
    /// that would also touch it is box/robot-only.)
    #[test]
    fn single_participant_slot_guard_lifecycle() {
        reset_participant_slot_for_test();
        let first = claim_participant_slot().expect("first claim");
        assert_eq!(
            claim_participant_slot().unwrap_err(),
            DdsError::ParticipantAlreadyExists
        );
        drop(first);
        let second = claim_participant_slot().expect("claim after drop");
        reset_participant_slot_for_test();
        let third = claim_participant_slot().expect("claim after reset seam");
        drop(second);
        drop(third);
        reset_participant_slot_for_test();
    }

    /// The discovery participant's SPDP lease defaults to the 10 s SHORT
    /// value (not the stock ~50 s). CI-safe pure value pin — `new`/`new_with_lease`
    /// thread it into `DomainParticipantBuilder::participant_lease_duration`, and
    /// the wire acquirer inherits it via `DiscoveryParticipant::new`. The lease
    /// actually reaching the wire is pinned by the fork's SPDP serialize-seam
    /// test + the live-peer ageout test (real participants need a live DDS network).
    #[test]
    fn default_lease_duration_is_ten_seconds() {
        assert_eq!(DEFAULT_LEASE_DURATION, Duration::from_secs(10));
    }
}
