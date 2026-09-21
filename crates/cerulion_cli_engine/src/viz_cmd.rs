// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion viz [TOPIC…]` — the `$CERULION_RERUN_URL` mirror the `cerulion viz`
//! verb shares with the daemon.
//!
//! # The verb is a `cerulion-vizd` CLIENT
//!
//! `cerulion viz` no longer compiles a graph per run. It is a THIN CLIENT of the
//! long-lived `cerulion-vizd` DAEMON: it ensures the daemon is running and sends
//! one `attach` per topic over the daemon's NDJSON control socket. The daemon owns
//! the runtime taps, the schema-generic decode→archetype→log dispatch, AND the
//! hosted Rerun proxy a viewer connects to. The client half — socket-path
//! resolution, `find_vizd_binary`, the request-line builders, the connection, and
//! `ensure_daemon` — lives in the rerun-free [`crate::viz_client`] module; THIS
//! module keeps only the `$CERULION_RERUN_URL` resolution ([`RERUN_URL_ENV`]).
//!
//! The `cerulion` CLI STILL never links `rerun` (the daemon does; the CLI speaks
//! its socket — the same decoupling `ros2 attach` uses), so this engine module
//! MIRRORS the [`RERUN_URL_ENV`] constant rather than depending on `cerulion_viz`.
//!
//! # The viewer is Cerulion Studio
//!
//! `cerulion-vizd` hosts the Rerun gRPC message proxy and advertises the bound
//! `rerun_url` in its connect banner. Cerulion Studio connects to that same daemon
//! and renders the scene, so the verb starts no viewer itself: it attaches the
//! topics, names the ones that landed, and points the user at Studio. The daemon
//! holds the scene, so Studio can connect at any time, before or after a run.
//!
//! # What still lives here (kept), and what the daemon owns now
//!
//! Kept: the `$CERULION_RERUN_URL` resolution. The topic
//! discovery + schema resolution the verb used to do at graph-generation time is
//! now the DAEMON's job (its `discover` control method walks live iceoryx2
//! services + resolves schemas); the verb asks the daemon for it.
//!
//! # There is no graph-embedded Rerun sink node
//!
//! This module's interactive graph-generation path
//! (`materialize_rerun_sink` / `generate_viz_graph` / `generate_remote_viz_graph`
//! / `generate_viz_anchor_source`) was deleted first, leaving the `rerun_sink`
//! NodeEntry crate standing, on the stated grounds that it was still wanted for
//! `graph run --record`. **That justification was false.** Recording is
//! implemented by `cerulion_bagd` draining data-only taps into `cerulion_bag`'s
//! MCAP — neither crate ever referenced `rerun_sink`, and the sink declares only
//! inputs, so it could not contribute a single frame to a bag. Its one remaining
//! caller was `ros2 attach`'s robot-side staging, which has since been removed (the
//! decision: no Rerun work runs on the robot). With no caller left, the
//! crate was deleted under the repo's dead-code policy.
//!
//! Visualization is DESK-side end to end: the desk demands a topic, `cerulion-netd`
//! re-injects it into desk-local SHM, and `cerulion-vizd` decodes + renders it on
//! the user's machine. The shared `cerulion_viz` dispatch/worker library now backs
//! exactly that one consumer.

/// The viewer-endpoint override env var, honored by the daemon's stream layer.
///
/// MIRRORS `cerulion_viz::stream::ADDR_ENV` (that crate pulls `rerun` and is NOT a
/// dependency of this engine — see the module docs — so the mirror cannot be a
/// direct cross-crate `assert_eq!`). Both sides pin the SAME literal string against
/// their own oracle: this side by the `rerun_url_env_is_the_pinned_literal` test,
/// the daemon side by `cerulion_viz::stream`'s `addr_env_is_the_pinned_literal` —
/// so a rename on EITHER side fails its literal pin and the drift is caught.
pub const RERUN_URL_ENV: &str = "CERULION_RERUN_URL";

// ─────────────────────────────────────────────────────────────────────────────
// `cerulion viz --robot NAME [TOPIC[=SCHEMA]…]` (remote-robot viz)
//
// The remote arm is now a DAEMON concern, not a graph-generation one. The verb
// passes the robot NAME + each topic (optionally pinned `TOPIC=SCHEMA`) to
// `cerulion-vizd`'s `attach` (see [`crate::viz_client`]); the daemon declares
// gateway INGRESS for the topic (re-injecting the robot's frames into desk-local
// SHM) and taps the local mirror by the identical local tap path. A remote topic
// carries no schema on the wire, but a schema pin is NO LONGER required: the daemon
// resolves each topic's ROS type from the robot's served CATALOG and
// fetches any custom type's schema over the network. A pin is still accepted (it
// skips the catalog lookup for a type the desk already knows). A topic the robot
// genuinely does not serve is a hard error, never a silent skip. The daemon is
// network-configured by default (scouting ON); only a kill-switched
// (CERULION_VIZD_NETWORK=off) daemon surfaces the not-network-configured residual.
// The old graph-generation remote path (`generate_remote_viz_graph` /
// `plan_viz_inputs` / the `viz_anchor` node) was DELETED with the rest of the
// per-run graph machinery.
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_workspace_is_a_loud_error_naming_workspace_creation() {
        // The verb requires a workspace, discovered by the binary via
        // `CerulionWorkspace::discover` (the SAME precondition every workspace
        // verb uses — `ros2 attach`, `graph run`, `node run`). Pin the contract
        // viz relies on: discovery from a non-workspace dir errors LOUDLY,
        // naming a workspace-creation command.
        let tmp = tempfile::tempdir().unwrap();
        let err = crate::workspace::CerulionWorkspace::discover(tmp.path()).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("Workspace not found"), "got: {msg}");
        assert!(msg.contains("cerulion workspace"), "got: {msg}");
    }

    #[test]
    fn rerun_url_env_is_the_pinned_literal() {
        // This engine cannot depend on `cerulion_viz` (it pulls rerun),
        // so the mirror with `cerulion_viz::stream::ADDR_ENV` cannot be a direct
        // cross-crate assert. Both sides instead pin the SAME literal against their
        // own oracle; the daemon side is `cerulion_viz::stream`'s
        // `addr_env_is_the_pinned_literal`. If either literal is renamed, its own
        // pin fails → the drift is caught.
        assert_eq!(RERUN_URL_ENV, "CERULION_RERUN_URL");
    }
}
