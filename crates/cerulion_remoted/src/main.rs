// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-remoted` — the robot-level remote-plane daemon binary.
//!
//! Thin clap wrapper: resolve the config (state-root path conventions, the relay
//! seam, the `--network off` kill-switch), then run the always-on accept loop
//! with Ctrl-C graceful shutdown. All logic lives in the `cerulion_remoted`
//! library.

use std::path::PathBuf;
use std::process::ExitCode;

use clap::Parser;

use cerulion_link::RelayConfig;
use cerulion_remoted::{resolve_network_disabled_loud, run, RemotedConfig, RemotedError};

/// The robot-level daemon that owns the ONE iroh endpoint and multiplexes the
/// wire + ops ALPNs, gating every accept through a deny-by-default authorizer.
#[derive(Parser, Debug)]
#[command(name = "cerulion-remoted", version, about, long_about = None)]
struct Cli {
    /// On-robot state root (default: cerud's `/var/lib/cerulion`). remoted's
    /// files live under `<root>/remoted/`.
    ///
    /// PREFER setting this via the CERULION_STATE_ROOT env: the LAN gateway (a
    /// SEPARATE process that publishes the mDNS beacon) resolves the state root
    /// from that env / the default to find remoted's beacon-facts file — it
    /// cannot see this flag. A non-default `--state-root` passed WITHOUT the
    /// matching env means the gateway won't find the facts, so the mDNS TXT
    /// enrichment (eid/iroh_port/claimable) is silently omitted (the beacon
    /// still advertises; direct-dial-by-key just falls back to discovery).
    #[arg(long, env = "CERULION_STATE_ROOT")]
    state_root: Option<PathBuf>,

    /// Internal registration writer used by the automatic bootstrap worker.
    #[arg(long, hide = true, conflicts_with_all = ["key_file", "store_file", "store_mac_key_file", "index_file"])]
    provision_bundle: Option<PathBuf>,

    /// Override the device-key file (32 raw bytes; public half = EndpointId).
    #[arg(long)]
    key_file: Option<PathBuf>,

    /// Override the trust-store file.
    #[arg(long)]
    store_file: Option<PathBuf>,

    /// Override the trust-store MAC-key file (skeleton seam — production reads
    /// this from firmware secure storage).
    #[arg(long)]
    store_mac_key_file: Option<PathBuf>,

    /// Override the device→account side-map file.
    #[arg(long)]
    index_file: Option<PathBuf>,

    // NOTE: there is deliberately NO `--beacon-facts-file` override, unlike the
    // four file flags above. Those are remoted-PRIVATE files (only remoted reads
    // them), so a per-file override is harmless. `beacon_facts_file` is a
    // CROSS-PROCESS HANDOFF file — the SEPARATE LAN gateway process also reads it,
    // resolving its path from CERULION_STATE_ROOT / the default (it cannot see any
    // remoted CLI flag). A standalone `--beacon-facts-file` only remoted knew
    // about would make remoted write to /custom while the gateway reads the
    // default, silently omitting the mDNS TXT enrichment with no error either
    // side. The ONLY correct redirect for this file is `--state-root` /
    // `CERULION_STATE_ROOT`, which BOTH processes honor — so `beacon_facts_file`
    // stays derived from the state root in `RemotedConfig::from_state_root` and is
    // never independently overridable.
    /// Self-hosted relay URL (prod). Default: n0 public relays.
    #[arg(long, env = "CERULION_RELAY_URL")]
    relay_url: Option<String>,

    /// Disable all relays (LAN / direct-dial only) — for tests / air-gap.
    #[arg(long)]
    relay_disabled: bool,

    /// Kill-switch: pass `off` to refuse to serve (R8 parity with the gateway).
    /// `CERULION_NETWORK=off` also engages it (either mechanism can trip it).
    #[arg(long)]
    network: Option<String>,
}

impl Cli {
    fn into_config(self) -> Result<RemotedConfig, RemotedError> {
        // The kill-switch is the OR of the flag and the env (either trips it).
        // The loud variant WARNS on an unrecognized value (a mistyped `--network
        // of` must never be a silently-ineffective kill switch) while keeping the
        // safe network-ON default.
        let network_disabled = resolve_network_disabled_loud(
            self.network.as_deref(),
            std::env::var("CERULION_NETWORK").ok().as_deref(),
        );
        let relay = resolve_relay(self.relay_url.as_deref(), self.relay_disabled)?;
        let state_root = self
            .state_root
            .unwrap_or_else(|| PathBuf::from(cerud::constants::DEFAULT_STATE_ROOT));
        let mut config = RemotedConfig::from_state_root(&state_root, relay, network_disabled);
        if let Some(p) = self.key_file {
            config.key_file = p;
        }
        if let Some(p) = self.store_file {
            config.store_file = p;
        }
        if let Some(p) = self.store_mac_key_file {
            config.store_mac_key_file = p;
        }
        if let Some(p) = self.index_file {
            config.index_file = p;
        }
        // No `beacon_facts_file` override here on purpose — it is a cross-process
        // handoff file redirected only via --state-root/CERULION_STATE_ROOT (see
        // the field-absence note on the `Cli` struct); it stays derived from the
        // resolved state root by `RemotedConfig::from_state_root`.
        Ok(config)
    }
}

/// Resolve the relay posture: `--relay-disabled` wins; else a `--relay-url`
/// (or its `CERULION_RELAY_URL` env, already merged by clap) becomes a
/// self-hosted `Custom` relay; else the n0 public relays.
///
/// DELEGATES to the ONE shared `cerulion_link::RelayConfig::resolve` precedence
/// helper (also linked by the desk client) so the parse/precedence logic is
/// single-sourced. The env is passed as the `env_url` slot because clap's
/// `env = "CERULION_RELAY_URL"` on `--relay-url` already merged it (never
/// double-read); `remoted` has no config file, so `config_url` is `None`.
fn resolve_relay(
    relay_url: Option<&str>,
    relay_disabled: bool,
) -> Result<RelayConfig, RemotedError> {
    RelayConfig::resolve(relay_disabled, relay_url, None)
        .map_err(|e| RemotedError::Config(e.to_string()))
}

fn init_tracing() {
    use tracing_subscriber::EnvFilter;
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    // Log to STDERR (a daemon's stdout stays clean); mirrors the workspace's
    // stdout↔stderr discipline (MEMORY: fmt::layer defaults to stdout).
    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .init();
}

#[tokio::main]
async fn main() -> ExitCode {
    init_tracing();
    // Apply `IOX2_LOG_LEVEL` to THIS binary's `iceoryx2-log` static
    // (a per-linked-copy static — see `init_iceoryx_log_level`), before the SHM
    // tap path can touch iceoryx2.
    cerulion_core::iceoryx_logger::init_iceoryx_log_level_from_env();
    let mut cli = Cli::parse();
    if let Some(registration_bundle) = cli.provision_bundle.take() {
        let state_root = cli
            .state_root
            .take()
            .unwrap_or_else(|| PathBuf::from(cerud::constants::DEFAULT_STATE_ROOT));
        return provision_registered_robot(state_root, registration_bundle);
    }
    let config = match cli.into_config() {
        Ok(c) => c,
        Err(e) => {
            tracing::error!(error = %e, "cerulion_remoted: configuration error");
            return ExitCode::FAILURE;
        }
    };
    match run(config, async {
        let _ = tokio::signal::ctrl_c().await;
    })
    .await
    {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            tracing::error!(error = %e, "cerulion_remoted: failed");
            ExitCode::FAILURE
        }
    }
}

fn provision_registered_robot(state_root: PathBuf, registration_bundle: PathBuf) -> ExitCode {
    use cerulion_remoted::provision::{provision, ProvisionConfig};
    use std::io::Write;

    let config = ProvisionConfig {
        state_root,
        registration_bundle,
    };
    let now_ns = cerulion_remoted::RemotedClock::wall().now_ns();
    match provision(&config, now_ns) {
        Ok(registered) => {
            let output = serde_json::json!({
                "version": 1,
                "robot_id": hex::encode(registered.robot_id.0),
                "endpoint_id": hex::encode(registered.endpoint_id.0),
                "owner_account": hex::encode(registered.owner_account.0),
            });
            let mut stdout = std::io::stdout().lock();
            if serde_json::to_writer(&mut stdout, &output).is_err()
                || stdout.write_all(b"\n").is_err()
                || stdout.flush().is_err()
            {
                tracing::error!(
                    "registered robot state is durable, but the result could not be written"
                );
                return ExitCode::FAILURE;
            }
            ExitCode::SUCCESS
        }
        Err(error) => {
            tracing::error!(error = %error, "automatic robot provisioning failed");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    #[test]
    fn automatic_writer_argument_is_hidden_and_parses_as_a_path() {
        let help = Cli::command().render_long_help().to_string();
        assert!(!help.contains("--provision-bundle"));
        let cli = Cli::try_parse_from([
            "cerulion-remoted",
            "--state-root",
            "/state",
            "--provision-bundle",
            "/state/registration.json",
        ])
        .unwrap();
        assert_eq!(
            cli.provision_bundle,
            Some(PathBuf::from("/state/registration.json"))
        );
    }

    #[test]
    fn automatic_writer_cannot_silently_ignore_individual_state_overrides() {
        for argument in [
            "--key-file",
            "--store-file",
            "--store-mac-key-file",
            "--index-file",
        ] {
            assert!(Cli::try_parse_from([
                "cerulion-remoted",
                "--provision-bundle",
                "/state/registration.json",
                argument,
                "/different-state",
            ])
            .is_err());
        }
    }
}
