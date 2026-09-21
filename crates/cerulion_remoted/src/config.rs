// SPDX-License-Identifier: AGPL-3.0-only
//! Daemon configuration: the on-disk path conventions, the relay seam, and the
//! `--network off` / `CERULION_NETWORK=off` kill-switch.

use std::path::{Path, PathBuf};

use cerulion_link::RelayConfig;

/// The `remoted`-owned subdirectory under the state root. Namespaced so its
/// files never collide with cerud's `bundles/` + `current` under the same root.
pub const REMOTED_SUBDIR: &str = "remoted";
/// The 32-byte ed25519 device secret (raw bytes). Its public half is the
/// endpoint's [`EndpointId`](cerulion_link::EndpointId).
pub const DEVICE_KEY_FILENAME: &str = "device_key";
/// The MAC'd, anti-rollback trust store (`cerulion_pairing`).
pub const TRUST_STORE_FILENAME: &str = "trust_store";
/// The trust-store MAC key. **Skeleton seam** — production reads this from
/// firmware secure storage (`cerulion_pairing` holds no key-management policy);
/// this file path is the dev/skeleton stand-in.
pub const TRUST_STORE_MAC_KEY_FILENAME: &str = "trust_store.mac_key";
/// The `device_key → account` side-map (R1), a small human-readable JSON index.
pub const DEVICE_INDEX_FILENAME: &str = "device_index.json";
/// The ops-plane hash-chained audit receipt log (one per robot, appended by every
/// ops verb the daemon serves — pairing/claim/e-stop/deploy/…).
pub const RECEIPT_FILENAME: &str = "receipts.log";
/// The `log-tail` allow-list root (under the state root). A `log-tail` request is
/// confined to this subtree.
pub const LOGS_SUBDIR: &str = "logs";
/// The PUBLIC beacon-facts file: `{version, eid, iroh_port, claimable}`
/// — the endpoint facts (public key hex, bound UDP port, claimable flag) only
/// `remoted` can know, published for the gateway's mDNS TXT enrichment. NO
/// secrets, so (unlike the trust store + device index) it carries no MAC.
pub const BEACON_FACTS_FILENAME: &str = "beacon_facts.json";

/// The resolved daemon configuration: where each artifact lives, the relay
/// posture, and whether the network kill-switch is engaged.
#[derive(Debug, Clone)]
pub struct RemotedConfig {
    /// The 32-byte device secret file.
    pub key_file: PathBuf,
    /// The trust-store file.
    pub store_file: PathBuf,
    /// The trust-store MAC key file (skeleton seam — see the constant docs).
    pub store_mac_key_file: PathBuf,
    /// The device→account side-map file.
    pub index_file: PathBuf,
    /// The ops-plane audit receipt log file.
    pub receipt_file: PathBuf,
    /// The `log-tail` allow-list root.
    pub log_root: PathBuf,
    /// The PUBLIC beacon-facts file the daemon writes at startup +
    /// refreshes after a claim, read by the gateway for mDNS TXT enrichment.
    pub beacon_facts_file: PathBuf,
    /// The relay posture (default: n0 public relays, by design).
    pub relay: RelayConfig,
    /// Whether `--network off` / `CERULION_NETWORK=off` is engaged (R8): the
    /// daemon refuses to serve, logs loudly, and exits cleanly.
    pub network_disabled: bool,
}

impl RemotedConfig {
    /// Derive the default file layout under a state root (`<root>/remoted/…`).
    pub fn from_state_root(
        state_root: impl AsRef<Path>,
        relay: RelayConfig,
        network_disabled: bool,
    ) -> Self {
        let dir = state_root.as_ref().join(REMOTED_SUBDIR);
        RemotedConfig {
            key_file: dir.join(DEVICE_KEY_FILENAME),
            store_file: dir.join(TRUST_STORE_FILENAME),
            store_mac_key_file: dir.join(TRUST_STORE_MAC_KEY_FILENAME),
            index_file: dir.join(DEVICE_INDEX_FILENAME),
            receipt_file: dir.join(RECEIPT_FILENAME),
            log_root: dir.join(LOGS_SUBDIR),
            beacon_facts_file: dir.join(BEACON_FACTS_FILENAME),
            relay,
            network_disabled,
        }
    }
}

/// The classification of a single raw kill-switch value (`--network` /
/// `CERULION_NETWORK`), after trimming + lowercasing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetworkSetting {
    /// Recognized "on" (or unset / empty) — the default: serve.
    On,
    /// Recognized "off" — engage the kill switch: refuse to serve.
    Off,
    /// A non-empty value that is neither `on` nor `off` (e.g. `of` / `disabled` /
    /// `false` / `0`). The SAFE interpretation is network-ON, but this must be
    /// surfaced LOUDLY (a silently-ignored kill switch on an internet-facing
    /// daemon is a footgun) — see [`resolve_network_disabled_loud`].
    Unrecognized,
}

/// Classify a raw kill-switch value (trimmed + case-insensitive). `None` and an
/// empty/whitespace-only value are the unset default ([`NetworkSetting::On`], no
/// warning — an empty env var reads as unset). Pure so it is oracle-testable.
pub fn classify_network(raw: Option<&str>) -> NetworkSetting {
    match raw.map(|s| s.trim().to_ascii_lowercase()).as_deref() {
        None | Some("") | Some("on") => NetworkSetting::On,
        Some("off") => NetworkSetting::Off,
        Some(_) => NetworkSetting::Unrecognized,
    }
}

/// Whether a single raw kill-switch value disables the network.
///
/// Off iff [`classify_network`] classifies it as [`NetworkSetting::Off`] (the
/// value, **trimmed and lowercased**, is exactly `"off"` — so a stray `"OFF"` /
/// `" off "` still kills the network reliably). Every other value (including
/// `None`, empty, and unrecognized garbage) leaves the network ON — the
/// always-on decision. Pure so it is oracle-testable. Mirrors the
/// gateway's `resolve_run_network` kill-switch parse. See
/// [`resolve_network_disabled_loud`] for the variant that WARNS on unrecognized
/// values instead of silently defaulting.
pub fn network_disabled(raw: Option<&str>) -> bool {
    matches!(classify_network(raw), NetworkSetting::Off)
}

/// Resolve the kill-switch from BOTH the `--network` flag and the
/// `CERULION_NETWORK` env: EITHER saying `off` disables the network (a kill
/// switch — either mechanism can trip it, mirroring the gateway).
pub fn resolve_network_disabled(flag: Option<&str>, env: Option<&str>) -> bool {
    network_disabled(flag) || network_disabled(env)
}

/// Resolve the kill-switch AND emit a LOUD `warn!` for any UNRECOGNIZED value.
///
/// Recognized values are (trimmed, case-insensitive) `on` / `off`, plus
/// unset/empty. Anything else keeps the safe default (network on:
/// garbage means the safe default) but must never be silently ignored: a
/// mistyped `--network of` on an internet-facing daemon is a silently-ineffective
/// kill switch. The binary calls THIS at config resolution; the pure classifier
/// keeps `config.rs` logging-free elsewhere.
pub fn resolve_network_disabled_loud(flag: Option<&str>, env: Option<&str>) -> bool {
    warn_if_unrecognized("--network", flag);
    warn_if_unrecognized("CERULION_NETWORK", env);
    resolve_network_disabled(flag, env)
}

/// Warn loudly (once) if `raw` is a non-empty, unrecognized kill-switch value.
fn warn_if_unrecognized(source: &str, raw: Option<&str>) {
    if matches!(classify_network(raw), NetworkSetting::Unrecognized) {
        tracing::warn!(
            source,
            value = raw.unwrap_or_default(),
            "cerulion_remoted: unrecognized {source} value; expected 'on' or 'off'. Leaving the \
             network ON (the SAFE default) — pass 'off' to disable the remote plane. A mistyped \
             kill-switch value is NOT silently honored as 'off'.",
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn network_disabled_only_matches_off_case_and_whitespace_insensitively() {
        // Hand oracle: (raw, disabled?).
        let cases: &[(Option<&str>, bool)] = &[
            (None, false),
            (Some("off"), true),
            (Some("OFF"), true),
            (Some("Off"), true),
            (Some(" off "), true),
            (Some("\toff\n"), true),
            (Some("on"), false),
            (Some(""), false),
            (Some("offx"), false),
            (Some("0"), false),
            (Some("false"), false),
        ];
        for (raw, expected) in cases {
            assert_eq!(
                network_disabled(*raw),
                *expected,
                "network_disabled({raw:?}) should be {expected}"
            );
        }
    }

    #[test]
    fn classify_network_covers_on_off_and_unrecognized() {
        use NetworkSetting::*;
        // Hand oracle: (raw, classification). Only `on`/`off` (+ unset/empty) are
        // recognized; everything else is Unrecognized (warned, network ON).
        let cases: &[(Option<&str>, NetworkSetting)] = &[
            (None, On),
            (Some(""), On),
            (Some("   "), On),
            (Some("on"), On),
            (Some("ON"), On),
            (Some(" On "), On),
            (Some("off"), Off),
            (Some("OFF"), Off),
            (Some("\toff\n"), Off),
            // The footgun class: a silently-ignored kill switch on the old code.
            (Some("of"), Unrecognized),
            (Some("disabled"), Unrecognized),
            (Some("false"), Unrecognized),
            (Some("0"), Unrecognized),
            (Some("offx"), Unrecognized),
            (Some("no"), Unrecognized),
        ];
        for (raw, expected) in cases {
            assert_eq!(
                classify_network(*raw),
                *expected,
                "classify_network({raw:?}) should be {expected:?}"
            );
        }
    }

    #[test]
    fn loud_resolver_agrees_with_the_plain_resolver_on_the_bool() {
        // The loud variant only ADDS a warn on unrecognized values; the disable
        // decision is identical to the plain resolver (the safe default holds).
        let cases: &[(Option<&str>, Option<&str>)] = &[
            (None, None),
            (Some("off"), None),
            (None, Some("off")),
            (Some("of"), None),        // unrecognized flag → still ON
            (None, Some("disabled")),  // unrecognized env → still ON
            (Some("off"), Some("of")), // off wins
            (Some("on"), Some("on")),
        ];
        for (flag, env) in cases {
            assert_eq!(
                resolve_network_disabled_loud(*flag, *env),
                resolve_network_disabled(*flag, *env),
                "loud vs plain disagree for flag={flag:?} env={env:?}"
            );
        }
    }

    #[test]
    fn resolve_kill_switch_is_the_or_of_flag_and_env() {
        // Either the flag OR the env saying "off" disables (a kill switch).
        assert!(!resolve_network_disabled(None, None));
        assert!(resolve_network_disabled(Some("off"), None), "flag off");
        assert!(resolve_network_disabled(None, Some("off")), "env off");
        assert!(
            resolve_network_disabled(Some("on"), Some("off")),
            "env off overrides an on flag (kill switch)"
        );
        assert!(
            resolve_network_disabled(Some("off"), Some("on")),
            "flag off overrides an on env (kill switch)"
        );
        assert!(
            !resolve_network_disabled(Some("on"), Some("on")),
            "neither off => network on"
        );
    }

    #[test]
    fn from_state_root_derives_the_namespaced_layout() {
        let cfg = RemotedConfig::from_state_root("/var/lib/cerulion", RelayConfig::Disabled, false);
        assert!(cfg.key_file.ends_with("remoted/device_key"));
        assert!(cfg.store_file.ends_with("remoted/trust_store"));
        assert!(cfg
            .store_mac_key_file
            .ends_with("remoted/trust_store.mac_key"));
        assert!(cfg.index_file.ends_with("remoted/device_index.json"));
        assert!(cfg.receipt_file.ends_with("remoted/receipts.log"));
        assert!(cfg.log_root.ends_with("remoted/logs"));
        assert!(cfg.beacon_facts_file.ends_with("remoted/beacon_facts.json"));
        assert!(matches!(cfg.relay, RelayConfig::Disabled));
        assert!(!cfg.network_disabled);
    }
}
