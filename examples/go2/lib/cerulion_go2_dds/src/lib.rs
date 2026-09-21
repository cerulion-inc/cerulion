// SPDX-License-Identifier: AGPL-3.0-only
//! Go2 DDS interop support — a ONE-per-process
//! `ros2-client`/`rustdds` participant wrapper + PURE CDR codecs for the v1
//! message set. This is a `lib/*` support crate (like `go2_tf` / `cerulion_viz`),
//! NOT a Cerulion node; the DDS ingress/egress node crates live under `nodes/`.
//!
//! # Interop constraints (baked into this crate)
//!
//! Interop with a CycloneDDS peer imposes these constraints (they
//! are restated in `graphs/go2.bridge.yaml`'s BRINGUP notes):
//!
//! 1. **`with_only_networks` is REQUIRED on a multi-homed host.** CycloneDDS
//!    (ALL versions, by design) DROPS fragmented builtin SPDP/SEDP discovery
//!    data — it logs `DATAFRAG ... fragmented builtin data not yet supported`
//!    and there is NO receiver-side fix. rustdds fragments a builtin sample
//!    once it exceeds ~1.4 KB, which a many-interface host reaches because
//!    rustdds advertises every local address as a unicast locator. Restricting
//!    to the robot-LAN interface (via
//!    [`participant::ParticipantConfig::only_networks`] ->
//!    `DomainParticipantBuilder::with_only_networks`) shrinks the discovery
//!    payload back under the fragmentation threshold. Leave it unset ONLY on a
//!    single-interface host.
//! 2. **Humble GID.** The Go2 firmware + on-robot ROS 2 are Humble-era, whose
//!    ROS 2 `Gid` predates the Iron 16-byte format. ros2-client's distro
//!    feature chain selects it; this crate defaults to the `humble` feature
//!    (`default-features = false` on ros2-client + `default = ["humble"]`).
//!    Wrong GID => `ros2 node list` empty + services broken (topic data may
//!    still flow). For a Jazzy+ peer: `--no-default-features --features jazzy`.
//! 3. **Pure serde-CDR codecs.** DDS pub/sub is typed (ros2-client serializes
//!    the message structs over CDR implicitly), but the [`cdr`] module also
//!    exposes the encode/decode as PURE functions using the SAME `cdr-encoding`
//!    engine (little-endian XCDR1) so the wire bytes are oracle-testable in
//!    isolation and available for direct bytes<->struct bridging in the node crates.
//!
//! # Layout
//!
//! - [`participant`] — [`Go2Participant`] (the one-per-process guard),
//!   [`participant::ParticipantConfig`], QoS helpers, [`participant::DdsError`].
//! - [`messages`] — field-verified serde structs for the v1 set (PointCloud2,
//!   Twist, SportModeState, Request) with source citations.
//! - [`cdr`] — pure CDR codecs + PointCloud2 point-data extraction, with
//!   hand-built oracle-vector tests (including the adversarial hostile-input
//!   regression guards — robot traffic is untrusted input).
//!
//! # Known limitations
//!
//! - The Unitree types ([`messages::SportModeState`], [`messages::Request`])
//!   are validated structurally and against the cited unitree_ros2 IDL, but
//!   are NOT validated against real DDS bytes captured from a
//!   Go2 — the live-peer e2e uses a stock ROS 2 Humble peer, which carries no
//!   `unitree_go`/`unitree_api` msgs. Cross-check them against bytes from your robot
//!   (`examples/go2/README.md`, "Validate on your robot").
//! - [`cdr::decode_xyz_points`] refuses ROW-PADDED organized clouds
//!   (`height > 1` with `row_step != width * point_step`) — empty result +
//!   `tracing::debug!`, never a silent mis-read of row padding as points.
//!   Dense organized and unorganized clouds (the Go2 lidar shape) decode
//!   normally, honoring `is_bigendian`.

// P12 (the repo's logging policy): library code never prints — it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics, and
// applied at the crate root rather than in `[workspace.lints]` because that
// table cannot distinguish a lib target from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

pub mod cdr;
pub mod messages;
pub mod participant;

// Re-export the whole ros2-client so the node crates reach any version-specific type
// (Topic, subscription, status streams) without this crate spelling paths that
// drift across ros2-client/rustdds releases.
pub use ros2_client;

// Curated top-level re-exports for the common surface.
pub use participant::{
    best_effort_qos, reliable_volatile_qos, DdsError, Go2Participant, ParticipantConfig,
    GO2_DEFAULT_DOMAIN,
};
