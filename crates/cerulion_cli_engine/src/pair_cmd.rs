// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion pair` verb's PURE resolution + spawn logic.
//!
//! `cerulion pair <robot>` pairs THIS desk with a robot: it runs the CPace
//! code-pairing ceremony over the robot's ops plane so the robot access-lists the
//! desk's device key, then pins `name → eid` into `~/.cerulion/robots.toml` so
//! `cerulion connect <robot>` afterwards works with no flags. Like `cerulion
//! connect`, the `cerulion` CLI stays iroh-free: it RESOLVES the robot eid + the
//! desk key path, builds the exact `cerulion-connectd pair` argv, and SPAWNS that
//! iroh-linking sibling (which owns the dial + the ceremony).
//!
//! This module is PURE + oracle-tested for the load-bearing seams: the
//! [`resolve_pair_target_with`] eid/name ladder, the [`build_pair_argv`] byte
//! oracle, and the [`merge_robots_toml`] round-trip (preserve existing entries +
//! the name-collision update). The mDNS browse + the process spawn + the file
//! writes are the thin impure wrappers.
//!
//! ## Target-resolution ladder (a positional NAME → eid)
//!
//! 1. `--eid HEX` explicit (mutually exclusive with a positional).
//! 2. a positional 64-char-hex IS the eid.
//! 3. a positional NAME → mDNS TXT `eid=` (the robot's advertised endpoint id).
//! 4. a positional NAME → `~/.cerulion/robots.toml` `[robots]` table.
//! 5. otherwise a not-found error.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Duration;

use crate::connect_cmd::{is_hex_eid, normalize_eid, parse_robots_toml, robots_toml_path};
use crate::error::{CliError, CliResult};

/// The bounded mDNS browse window for a `cerulion pair <name>` eid lookup — long
/// enough for a robot on the LAN to answer, short enough to fall through to
/// `robots.toml` promptly (mirrors the discovery ladder's sub-2s ceiling).
const PAIR_MDNS_BUDGET: Duration = Duration::from_millis(1500);

/// The raw `cerulion pair` CLI flags (the clap-parsed surface). Resolved into a
/// [`PairPlan`] by [`plan`].
#[derive(Clone, Default)]
pub struct PairArgs {
    /// The positional ROBOT: a 64-char-hex eid, OR a name resolved via mDNS
    /// `eid=` / `~/.cerulion/robots.toml`. Omit when using `--eid`.
    pub robot: Option<String>,
    /// The explicit `--eid` (overrides a positional; no name is pinned).
    pub eid: Option<String>,
    /// Direct `ip:port` addresses (`--addr`, repeatable).
    pub addrs: Vec<String>,
    /// The short pairing code (`--code`). When omitted, `cerulion-connectd pair`
    /// reads it from stdin (a TTY prompt for a human, a piped line for Studio).
    pub code: Option<String>,
    /// The account (64-char hex) this pairing is FOR (`--account`). When omitted,
    /// the desk derives a self-account from its device key.
    pub account: Option<String>,
    /// The access-list label the robot stores (`--label`). When omitted, the desk
    /// hostname is used.
    pub label: Option<String>,
    /// The desk device key file (`--key-file`). When omitted, the well-known
    /// `~/.cerulion/desk.key` is used (created by `cerulion-connectd pair` if
    /// absent).
    pub key_file: Option<PathBuf>,
    /// Self-hosted relay URL (`--relay-url`).
    pub relay_url: Option<String>,
    /// Disable relays (`--relay-disabled`).
    pub relay_disabled: bool,
}

impl std::fmt::Debug for PairArgs {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PairArgs")
            .field("robot", &self.robot)
            .field("eid", &self.eid)
            .field("addrs", &self.addrs)
            .field("code", &"[REDACTED]")
            .field("account", &self.account)
            .field("label", &self.label)
            .field("key_file", &self.key_file)
            .field("relay_url", &self.relay_url)
            .field("relay_disabled", &self.relay_disabled)
            .finish()
    }
}

/// A resolved pairing target: the robot eid (hex) + the name to pin on success
/// (`None` when the user passed a raw eid — nothing to pin).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PairTarget {
    /// The robot's iroh endpoint id (64-char hex).
    pub eid: String,
    /// The name to pin `name → eid` into `robots.toml` on success (`None` for a
    /// raw-eid target).
    pub name: Option<String>,
}

/// The resolved spawn plan: the `cerulion-connectd` binary + its `pair` argv, plus
/// the eid + optional name used to pin `robots.toml` after a successful pairing.
#[derive(Clone, PartialEq, Eq)]
pub struct PairPlan {
    /// The resolved `cerulion-connectd` binary path.
    pub bin: PathBuf,
    /// The argv (after the binary) to pass to it (leads with `pair`).
    pub argv: Vec<String>,
    /// The name to pin on success (`None` for a raw-eid target).
    pub name: Option<String>,
    /// The resolved robot eid (pinned as `name → eid`).
    pub eid: String,
}

impl std::fmt::Debug for PairPlan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PairPlan")
            .field("bin", &self.bin)
            .field("argv", &"[REDACTED]")
            .field("name", &self.name)
            .field("eid", &self.eid)
            .finish()
    }
}

// ── target resolution (pure ladder + production wrapper) ──────────────────────

/// Resolve the pairing target from the positional + `--eid` against an injected
/// mDNS lookup and a `robots.toml` map. PURE (both sources are injected).
///
/// Precedence: `--eid` (explicit) wins; else a 64-char-hex positional IS the eid;
/// else a positional NAME resolves via `mdns_lookup` first (the robot's live
/// advertised eid), else the `robots` map (a previously-pinned eid). A positional
/// AND `--eid` together is ambiguous (error); neither is a missing-target error.
///
/// EVERY resolved eid is NORMALIZED to canonical bare-lowercase-hex via
/// `normalize_eid` (an ed25519 endpoint id is case-insensitive hex, and the mDNS
/// rung's validator is lowercase-only). So an uppercase positional / `--eid` /
/// mDNS `eid=` all resolve identically, and a `robots.toml` pin stores lowercase —
/// no rung disagrees on case.
pub fn resolve_pair_target_with(
    positional: Option<&str>,
    eid_flag: Option<&str>,
    mdns_lookup: impl Fn(&str) -> Option<String>,
    robots: &BTreeMap<String, String>,
) -> CliResult<PairTarget> {
    match (positional, eid_flag) {
        (Some(_), Some(_)) => Err(CliError::Validation(
            "pass EITHER a positional robot (name or eid) OR `--eid`, not both".to_string(),
        )),
        (None, Some(eid)) => Ok(PairTarget {
            eid: normalize_eid(eid),
            name: None,
        }),
        (Some(pos), None) => {
            if is_hex_eid(pos) {
                return Ok(PairTarget {
                    eid: normalize_eid(pos),
                    name: None,
                });
            }
            // A NAME: mDNS eid= (live) first, then robots.toml (pinned).
            if let Some(eid) = mdns_lookup(pos) {
                return Ok(PairTarget {
                    eid: normalize_eid(&eid),
                    name: Some(pos.to_string()),
                });
            }
            if let Some(eid) = robots.get(pos) {
                return Ok(PairTarget {
                    eid: normalize_eid(eid),
                    name: Some(pos.to_string()),
                });
            }
            Err(CliError::Validation(format!(
                "unknown robot '{pos}': it did not answer mDNS with an `eid=` record and is not \
                 pinned in ~/.cerulion/robots.toml. Make sure the robot is on and reachable, or \
                 pass its endpoint id directly with `--eid <64-hex>` (from the robot operator)."
            )))
        }
        (None, None) => Err(CliError::Validation(
            "specify a robot: a positional name/eid or `--eid <64-hex>` (see \
             `cerulion pair --help`)"
                .to_string(),
        )),
    }
}

/// Production target resolution: [`resolve_pair_target_with`] over a real bounded
/// mDNS browse + the real `~/.cerulion/robots.toml`.
pub fn resolve_pair_target(args: &PairArgs) -> CliResult<PairTarget> {
    let robots = load_robots();
    resolve_pair_target_with(
        args.robot.as_deref(),
        args.eid.as_deref(),
        |name| crate::mdns_discovery::resolve_robot_eid(name, PAIR_MDNS_BUDGET),
        &robots,
    )
}

/// Load the `~/.cerulion/robots.toml` name→eid map (empty if the file is absent
/// or unreadable — a missing file is normal).
fn load_robots() -> BTreeMap<String, String> {
    let Some(path) = robots_toml_path() else {
        return BTreeMap::new();
    };
    match std::fs::read_to_string(&path) {
        Ok(text) => parse_robots_toml(&text),
        Err(_) => BTreeMap::new(),
    }
}

/// The desk hostname — the default access-list label the robot stores (so the
/// owner recognizes this desk). Reuses the platform-wide identity resolver
/// (`CERULION_ROBOT_IDENTITY` override, else the machine hostname).
fn desk_label_default() -> String {
    cerulion_core::graph::robot_identity_from_env()
}

/// Resolve the desk key PATH: the explicit `--key-file`, else the well-known
/// `~/.cerulion/desk.key`. A missing home directory with no explicit path is a
/// loud error. `cerulion-connectd pair` CREATES the key at this path if absent.
fn resolve_desk_key_path(explicit: Option<PathBuf>) -> CliResult<PathBuf> {
    if let Some(p) = explicit {
        return Ok(p);
    }
    crate::connect_cmd::desk_key_path().ok_or_else(|| {
        CliError::Validation(
            "no home directory to resolve ~/.cerulion/desk.key — pass `--key-file <path>` \
             for the desk device key"
                .to_string(),
        )
    })
}

// ── argv construction (pure, byte-oracle-tested) ──────────────────────────────

/// Build the exact `cerulion-connectd pair` argv (after the binary) for a resolved
/// target + key path + label + the flags. PURE — the byte-exact argv oracle-tested.
///
/// The argv leads with the `pair` SUBCOMMAND. `--label` is always passed (the
/// resolved default or the user override); `--robot-name` is passed only when a
/// name is known (a friendly STDOUT display); `--code` / `--account` are passed
/// only when the user supplied them.
pub fn build_pair_argv(
    target: &PairTarget,
    key_path: &std::path::Path,
    label: &str,
    args: &PairArgs,
) -> Vec<String> {
    let mut v = vec![
        "pair".to_string(),
        "--eid".to_string(),
        target.eid.clone(),
        "--key-file".to_string(),
        key_path.display().to_string(),
    ];
    for a in &args.addrs {
        v.push("--addr".to_string());
        v.push(a.clone());
    }
    if let Some(code) = &args.code {
        v.push("--code".to_string());
        v.push(code.clone());
    }
    if let Some(account) = &args.account {
        v.push("--account".to_string());
        v.push(account.clone());
    }
    v.push("--label".to_string());
    v.push(label.to_string());
    if let Some(name) = &target.name {
        v.push("--robot-name".to_string());
        v.push(name.clone());
    }
    if let Some(u) = &args.relay_url {
        v.push("--relay-url".to_string());
        v.push(u.clone());
    }
    if args.relay_disabled {
        v.push("--relay-disabled".to_string());
    }
    v
}

/// Warn (loudly) when the pairing code is passed via `--code`: it is then visible
/// in process listings (`ps`) for the ceremony window, and the code authorizes
/// durable enrollment. stdin (the default) keeps it off the argv. Returns whether
/// it warned (for the oracle test).
fn warn_if_code_in_argv(code: Option<&str>) -> bool {
    if code.is_some() {
        tracing::warn!(
            "cerulion pair: the pairing code passed via --code is VISIBLE in process listings \
             (`ps`) for the ceremony window; prefer the interactive prompt or piping it on stdin"
        );
        true
    } else {
        false
    }
}

/// Resolve the full spawn plan: the target (eid + name) + the argv + the binary.
pub fn plan(args: &PairArgs) -> CliResult<PairPlan> {
    warn_if_code_in_argv(args.code.as_deref());
    let target = resolve_pair_target(args)?;
    let key_path = resolve_desk_key_path(args.key_file.clone())?;
    let label = args.label.clone().unwrap_or_else(desk_label_default);
    let argv = build_pair_argv(&target, &key_path, &label, args);
    let bin = crate::connect_cmd::resolve_connectd_bin()?;
    Ok(PairPlan {
        bin,
        argv,
        name: target.name,
        eid: target.eid,
    })
}

// ── robots.toml pinning (pure merge + atomic write) ───────────────────────────

/// Merge `name → eid` into an existing `robots.toml` document, **preserving
/// every other byte** — foreign top-level keys, comments and the author's own
/// key order included — and return the rewritten document. PURE — oracle-tested.
///
/// **Why the whole document and not a typed view.** A merge
/// that deserializes into a struct carrying ONLY
/// `robots` and re-serializes THAT makes `toml::to_string` emit a document
/// built from the one key the struct knows — and every other top-level key a
/// user has put in the file is DELETED by the next successful `cerulion pair`.
/// That is destruction, on a file the module's own no-clobber
/// discipline protects two functions away (a malformed document is a loud
/// refusal, an unreadable one likewise), and on a surface `connect_cmd`
/// explicitly tells users to hand-edit.
///
/// Editing the WHOLE PARSED DOCUMENT rather than a typed view is what makes
/// the claim true: a `toml::Table` holds every key the file had, so one this
/// code has never heard of survives BECAUSE nothing rewrote it — the same
/// argument the `nodes:` splice makes for `graphs/*.yaml`.
///
/// **RESIDUAL, stated because it is not free:** the document is re-SERIALIZED,
/// so COMMENTS are still lost and key order follows `toml`'s map rather than
/// the author's. `toml_edit` would preserve both, but it does
/// not resolve against this tree (it needs `toml_writer ^1.1.2` while `toml`
/// 0.9.11 pins 1.0.6), and bumping a dependency shared with every other `toml`
/// user is not worth comment fidelity on a machine-written pairing store. What
/// matters most — a key the next `cerulion pair` DELETES — is
/// closed either way.
///
/// A STRICT parse (unlike the lenient read-path [`parse_robots_toml`]): a
/// MALFORMED existing document is a LOUD error, never silently clobbered. An
/// empty / absent document yields a one-entry table. Re-pinning an existing
/// name updates its eid in place. A `robots` key of the WRONG SHAPE — a scalar
/// where a table belongs, which a hand-edit can produce — is refused rather
/// than overwritten, for the same reason a malformed document is.
pub fn merge_robots_toml(existing: &str, name: &str, eid: &str) -> Result<String, String> {
    let mut doc: toml::Table = existing.parse().map_err(|e| {
        format!("~/.cerulion/robots.toml is malformed ({e}); fix it or connect with `--eid`")
    })?;

    // An absent `[robots]` is the normal first-pin case. `or_insert_with`
    // leaves an existing table — and everything already in it — untouched.
    let robots = doc
        .entry("robots")
        .or_insert_with(|| toml::Value::Table(toml::Table::new()));

    // A `robots` that is not a table is a hand-edit mistake, and overwriting it
    // would destroy whatever the user meant to write there.
    let table = robots.as_table_mut().ok_or_else(|| {
        "~/.cerulion/robots.toml has a `robots` key that is not a table; fix it or connect \
         with `--eid`"
            .to_string()
    })?;

    // The ENTRIES must satisfy the READER's contract too.
    //
    // A typed merge would deserialize into `BTreeMap<String, String>` and so
    // refuse a non-string entry (`go2 = 123`) before anything is
    // written. Editing the document keeps every foreign key — which is the
    // point — but it does not type-check the one table we DO own, and
    // that is not a harmless relaxation: `connect_cmd::parse_robots_toml`
    // deserializes the same `BTreeMap<String, String>`, so ONE non-string
    // entry makes the whole read fail and yield an EMPTY map. Writing that
    // file would strand every pin in it, the newly-added one included — the
    // pairing would report success and `cerulion connect <name>` would then
    // find nothing.
    //
    // Refused rather than dropped, on this module's own no-clobber discipline:
    // a malformed document and an unreadable one are both loud refusals here,
    // and silently deleting an entry the user hand-wrote is worse than either.
    if let Some((bad_key, bad_value)) = table
        .iter()
        .find(|(_, value)| !value.is_str())
        .map(|(k, v)| (k.to_string(), v.type_str()))
    {
        return Err(format!(
            "~/.cerulion/robots.toml has a `[robots]` entry that is not a string — `{bad_key}` \
             is a {bad_value}, and every entry must be a robot's eid (a quoted hex string). \
             `cerulion connect` cannot read the file at all while it is there, so pinning would \
             strand every entry including this one. Fix that line, or connect with `--eid`."
        ));
    }

    table.insert(name.to_string(), toml::Value::String(eid.to_string()));

    toml::to_string(&doc).map_err(|e| format!("serializing robots.toml failed: {e}"))
}

/// Pin `name → eid` into `~/.cerulion/robots.toml` (read → merge → atomic write).
/// Preserves every existing entry. A best-effort convenience — the CALLER treats
/// a failure as non-fatal (the pairing already succeeded).
pub fn pin_robot(name: &str, eid: &str) -> Result<(), String> {
    let path = robots_toml_path()
        .ok_or_else(|| "no home directory to write ~/.cerulion/robots.toml".to_string())?;
    pin_robot_at(&path, name, eid)
}

/// The path-injected core of [`pin_robot`] (read → merge → atomic write), so the
/// read/merge/write orchestration is testable over a temp file without touching
/// the real `~/.cerulion` or mutating `$HOME`.
fn pin_robot_at(path: &std::path::Path, name: &str, eid: &str) -> Result<(), String> {
    // Distinguish "no file yet" (start from an empty doc — the normal first-pin
    // case) from ANY OTHER read error (permission denied, disk error). An
    // UNREADABLE existing file is a loud refusal, NEVER a clobber: nothing can
    // preserve what was not read (the malformed-file no-clobber discipline
    // extends to unreadable — conflating them with `unwrap_or_default()` would
    // erase every existing pin).
    let existing = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => String::new(),
        Err(e) => {
            return Err(format!(
                "cannot read {} to preserve its existing pins ({e}); refusing to overwrite it",
                path.display()
            ))
        }
    };
    let merged = merge_robots_toml(&existing, name, eid)?;
    write_string_atomically(path, &merged)
}

/// Atomically write `content` to `path`, crash-durably: parent created 0700 →
/// unique temp `create_new` 0600 → fsync → `rename` (atomic on POSIX). Delegates
/// to [`crate::auth::atomic_write_secret`] so `robots.toml` gets the SAME
/// contract as the auth secrets (a crash between write and rename cannot
/// lose the pinned robots); `robots.toml` landing at 0600 is fine — it is
/// per-user CLI state.
fn write_string_atomically(path: &std::path::Path, content: &str) -> Result<(), String> {
    crate::auth::atomic_write_secret(path, content.as_bytes())
        .map_err(|e| format!("writing {}: {e}", path.display()))
}

// ── spawn + pin ───────────────────────────────────────────────────────────────

/// Spawn `cerulion-connectd pair` (inheriting stdio, forwarding a directed
/// signal), and on a successful pairing (exit 0) pin `name → eid` into
/// `robots.toml`. Returns the child's exit code (the pairing outcome contract).
///
/// A `robots.toml` pin failure is NON-fatal: the pairing already succeeded (the
/// robot trusts the desk), so it is a loud `warn!` with the `--eid` fallback, not
/// a changed exit code.
pub fn spawn_and_pair(plan: &PairPlan, running: Arc<AtomicBool>) -> CliResult<i32> {
    // Reuse the connect spawner's battle-tested inherit-stdio + signal-forward +
    // reap loop — the plan shape (bin + argv) is identical.
    let connect_plan = crate::connect_cmd::ConnectPlan {
        bin: plan.bin.clone(),
        argv: plan.argv.clone(),
    };
    let code = crate::connect_cmd::spawn_and_wait(&connect_plan, running)?;
    finalize_pairing(code, plan.name.as_deref(), &plan.eid, &pin_robot);
    Ok(code)
}

/// Decide + apply the post-ceremony `robots.toml` pin. Pins `name → eid` (and
/// prints the `pinned:` state line) ONLY on a GENUINE exit 0 — the connectd `pair`
/// exit contract's "the robot wrote a durable access-list row".
///
/// ANY other exit code — a robot refusal (2), an unreachable dial (3), a wrong
/// code (4), OR a signal-KILLED connectd that `connect_cmd::exit_code_of` maps to
/// `128 + signo` (e.g. Ctrl-C at the code prompt → 130, NEVER a
/// synthesized 0) — leaves `robots.toml` UNTOUCHED, prints no `pinned:` line, and
/// emits a loud "did NOT complete" note. `pin` is injected so the pin-only-on-0
/// gate is unit-testable with a spy (no real `~/.cerulion` write).
// P12 exception: `pinned_line` is a machine-parseable state line on STDOUT —
// this crate's OUTPUT PROTOCOL, not logging. A Studio driver parses it; moving
// it to `tracing` would put it on stderr behind RUST_LOG and break the parser.
// The narration of the same step already goes through `tracing::warn!` below.
#[allow(clippy::print_stdout)]
fn finalize_pairing(
    code: i32,
    name: Option<&str>,
    eid: &str,
    pin: &dyn Fn(&str, &str) -> Result<(), String>,
) {
    if code != 0 {
        tracing::warn!(
            exit_code = code,
            eid = %eid,
            "cerulion pair: did NOT complete (exit {code}) — the robot did not confirm pairing; \
             ~/.cerulion/robots.toml was NOT pinned"
        );
        return;
    }
    match name {
        Some(name) => match pin(name, eid) {
            Ok(()) => println!("{}", pinned_line(name, eid)),
            Err(e) => tracing::warn!(
                error = %e,
                name = %name,
                eid = %eid,
                "cerulion pair: paired OK, but pinning ~/.cerulion/robots.toml failed — \
                 `cerulion connect {name}` may not resolve; connect with `--eid {eid}` meanwhile"
            ),
        },
        None => tracing::info!(
            eid = %eid,
            "cerulion pair: paired OK (no robot name to pin — reconnect with \
             `cerulion connect --eid {eid}`)"
        ),
    }
}

/// The `pinned:` STDOUT state line (the Studio machine contract) — exact format
/// pinned by an oracle test so a driver can parse it byte-stably.
fn pinned_line(name: &str, eid: &str) -> String {
    format!("pinned: name={name} eid={eid}")
}

#[cfg(test)]
mod tests {
    use super::*;

    // Exactly 64 hex chars.
    const HEX64: &str = "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
    const HEX64_B: &str = "ffeeddccbbaa99887766554433221100ffeeddccbbaa99887766554433221100";

    fn robots(pairs: &[(&str, &str)]) -> BTreeMap<String, String> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect()
    }

    /// The eid/name ladder: --eid wins (no name), a hex positional IS the eid (no
    /// name), a NAME resolves via mDNS FIRST then robots.toml (with the name), and
    /// the error arms.
    #[test]
    fn resolve_pair_target_ladder() {
        let no_mdns = |_: &str| None;
        let r = robots(&[("go2", HEX64)]);

        // --eid wins → eid, no name to pin.
        assert_eq!(
            resolve_pair_target_with(None, Some(HEX64), no_mdns, &r).unwrap(),
            PairTarget {
                eid: HEX64.to_string(),
                name: None
            }
        );
        // A hex positional IS the eid → no name.
        assert_eq!(
            resolve_pair_target_with(Some(HEX64), None, no_mdns, &r).unwrap(),
            PairTarget {
                eid: HEX64.to_string(),
                name: None
            }
        );
        // A NAME resolves via robots.toml (mDNS silent) → carries the name.
        assert_eq!(
            resolve_pair_target_with(Some("go2"), None, no_mdns, &r).unwrap(),
            PairTarget {
                eid: HEX64.to_string(),
                name: Some("go2".to_string())
            }
        );
        // mDNS WINS over robots.toml (the live advertised eid beats a stale pin).
        let mdns_hit = |n: &str| (n == "go2").then(|| HEX64_B.to_string());
        assert_eq!(
            resolve_pair_target_with(Some("go2"), None, mdns_hit, &r).unwrap(),
            PairTarget {
                eid: HEX64_B.to_string(),
                name: Some("go2".to_string())
            },
            "mDNS eid= takes precedence over a stale robots.toml pin"
        );
        // Both positional AND --eid → ambiguous error.
        let e = resolve_pair_target_with(Some("go2"), Some(HEX64), no_mdns, &r).unwrap_err();
        assert!(e.to_string().contains("not both"), "err: {e}");
        // Unknown name (no mDNS, no robots) → error naming both sources + --eid.
        let e = resolve_pair_target_with(Some("nope"), None, no_mdns, &robots(&[])).unwrap_err();
        assert!(e.to_string().contains("robots.toml"), "err: {e}");
        assert!(e.to_string().contains("--eid"), "err: {e}");
        assert!(e.to_string().contains("nope"), "err names the robot: {e}");
        // Neither → missing-target error.
        let e = resolve_pair_target_with(None, None, no_mdns, &r).unwrap_err();
        assert!(e.to_string().contains("specify a robot"), "err: {e}");
    }

    /// No case asymmetry between rungs: an uppercase positional,
    /// `--eid`, AND mDNS `eid=` all resolve to the SAME canonical lowercase eid
    /// (hand oracle = the lowercased eid), so no rung disagrees on case.
    #[test]
    fn resolve_pair_target_normalizes_eid_case_across_rungs() {
        let upper = HEX64.to_uppercase();
        let no_mdns = |_: &str| None;
        let empty = robots(&[]);

        // Uppercase --eid → lowercased eid, no name.
        assert_eq!(
            resolve_pair_target_with(None, Some(&upper), no_mdns, &empty).unwrap(),
            PairTarget {
                eid: HEX64.to_string(),
                name: None
            }
        );
        // Uppercase 64-hex positional → lowercased eid, no name.
        assert_eq!(
            resolve_pair_target_with(Some(&upper), None, no_mdns, &empty).unwrap(),
            PairTarget {
                eid: HEX64.to_string(),
                name: None
            }
        );
        // Uppercase mDNS eid= for a name → lowercased eid + the name (without the
        // lowercasing this fell through to robots.toml and died as "unknown robot").
        let mdns_upper = {
            let u = upper.clone();
            move |n: &str| (n == "go2").then(|| u.clone())
        };
        assert_eq!(
            resolve_pair_target_with(Some("go2"), None, mdns_upper, &empty).unwrap(),
            PairTarget {
                eid: HEX64.to_string(),
                name: Some("go2".to_string())
            }
        );
        // An uppercase-pinned robots.toml value also resolves to lowercase.
        let r_upper = robots(&[("go2", &upper)]);
        assert_eq!(
            resolve_pair_target_with(Some("go2"), None, no_mdns, &r_upper).unwrap(),
            PairTarget {
                eid: HEX64.to_string(),
                name: Some("go2".to_string())
            }
        );
    }

    /// The full `pair` argv byte oracle.
    #[test]
    fn build_pair_argv_full_oracle() {
        let target = PairTarget {
            eid: HEX64.to_string(),
            name: Some("go2".to_string()),
        };
        let args = PairArgs {
            addrs: vec!["192.168.1.20:7842".into(), "[::1]:9000".into()],
            code: Some("SWAN-42".into()),
            account: Some(HEX64_B.into()),
            relay_url: Some("https://relay.example".into()),
            relay_disabled: false,
            ..Default::default()
        };
        let argv = build_pair_argv(
            &target,
            std::path::Path::new("/keys/desk.key"),
            "my-desk",
            &args,
        );
        assert_eq!(
            argv,
            vec![
                "pair".to_string(),
                "--eid".into(),
                HEX64.to_string(),
                "--key-file".into(),
                "/keys/desk.key".into(),
                "--addr".into(),
                "192.168.1.20:7842".into(),
                "--addr".into(),
                "[::1]:9000".into(),
                "--code".into(),
                "SWAN-42".into(),
                "--account".into(),
                HEX64_B.into(),
                "--label".into(),
                "my-desk".into(),
                "--robot-name".into(),
                "go2".into(),
                "--relay-url".into(),
                "https://relay.example".into(),
            ]
        );
    }

    /// The minimal argv (raw-eid target — no name, no code/account/addrs/relay):
    /// `pair --eid <e> --key-file <p> --label <l>`.
    #[test]
    fn build_pair_argv_minimal() {
        let target = PairTarget {
            eid: HEX64.to_string(),
            name: None,
        };
        let args = PairArgs {
            relay_disabled: true,
            ..Default::default()
        };
        let argv = build_pair_argv(&target, std::path::Path::new("/k.key"), "desk", &args);
        assert_eq!(
            argv,
            vec![
                "pair".to_string(),
                "--eid".into(),
                HEX64.to_string(),
                "--key-file".into(),
                "/k.key".into(),
                "--label".into(),
                "desk".into(),
                "--relay-disabled".into(),
            ],
            "no --robot-name (raw eid), no --code/--account/--addr; --relay-disabled trails"
        );
    }

    /// `merge_robots_toml`: pins a new entry, PRESERVES existing entries, and
    /// UPDATES an existing name in place — round-tripped back through a lenient
    /// parse (hand oracle, never a self-compare of the serialized string).
    #[test]
    fn merge_robots_toml_preserves_and_updates() {
        // Absent/empty → a one-entry table.
        let out = merge_robots_toml("", "go2", HEX64).unwrap();
        let m = parse_robots_toml(&out);
        assert_eq!(m.get("go2").map(String::as_str), Some(HEX64));
        assert_eq!(m.len(), 1);

        // Existing entries are PRESERVED; the new one is added.
        let existing = "[robots]\nspot = \"aabb\"\norin = \"ccdd\"\n";
        let out = merge_robots_toml(existing, "go2", HEX64).unwrap();
        let m = parse_robots_toml(&out);
        assert_eq!(
            m.get("spot").map(String::as_str),
            Some("aabb"),
            "spot preserved"
        );
        assert_eq!(
            m.get("orin").map(String::as_str),
            Some("ccdd"),
            "orin preserved"
        );
        assert_eq!(m.get("go2").map(String::as_str), Some(HEX64), "go2 added");
        assert_eq!(m.len(), 3);

        // Re-pinning an existing name UPDATES its eid (a stale entry heals).
        let existing = format!("[robots]\ngo2 = \"{HEX64}\"\n");
        let out = merge_robots_toml(&existing, "go2", HEX64_B).unwrap();
        let m = parse_robots_toml(&out);
        assert_eq!(
            m.get("go2").map(String::as_str),
            Some(HEX64_B),
            "go2 updated in place"
        );
        assert_eq!(m.len(), 1);

        // A MALFORMED existing document is a LOUD error — NOT silently clobbered.
        let err = merge_robots_toml("this is : not [[[toml", "go2", HEX64).unwrap_err();
        assert!(err.contains("malformed"), "err: {err}");
    }

    /// A FOREIGN TOP-LEVEL key survives a pin.
    ///
    /// The preservation arm above feeds only siblings INSIDE
    /// `[robots]`, which even a typed round-trip preserves perfectly — so it
    /// cannot see the total destruction a typed merge causes: a merge that
    /// deserializes into a struct carrying only `robots` and re-serializes
    /// THAT drops every other top-level key from the document the next
    /// successful `cerulion pair` writes.
    ///
    /// Asserted on the RE-PARSED DOCUMENT rather than on the text, so the
    /// claim is about what survives semantically and does not turn on
    /// serializer formatting.
    #[test]
    fn a_foreign_top_level_key_survives_a_pin() {
        let existing = "\
[robots]
spot = \"aabb\"

[desk]
name = \"desk-mac\"
theme = \"dark\"

[experimental]
enabled = true
";
        let out = merge_robots_toml(existing, "go2", HEX64).unwrap();
        let doc: toml::Table = out.parse().expect("the merged document re-parses");

        // The pin landed, and the sibling inside `[robots]` is still there
        // (the property the pre-existing arm covers).
        let robots = doc["robots"].as_table().expect("`robots` is a table");
        assert_eq!(robots["go2"].as_str(), Some(HEX64));
        assert_eq!(robots["spot"].as_str(), Some("aabb"));

        // THE PIN: keys this code has never heard of survive, values included.
        let desk = doc
            .get("desk")
            .unwrap_or_else(|| panic!("`[desk]` must survive a pin; got:\n{out}"))
            .as_table()
            .expect("`desk` is a table");
        assert_eq!(desk["name"].as_str(), Some("desk-mac"));
        assert_eq!(desk["theme"].as_str(), Some("dark"));

        let experimental = doc
            .get("experimental")
            .unwrap_or_else(|| panic!("`[experimental]` must survive a pin; got:\n{out}"))
            .as_table()
            .expect("`experimental` is a table");
        assert_eq!(experimental["enabled"].as_bool(), Some(true));

        // A SECOND pin over the merged output preserves them again — the
        // property that matters is that repeated pinning converges rather than
        // eroding the file one key at a time.
        let out2 = merge_robots_toml(&out, "orin", HEX64_B).unwrap();
        let doc2: toml::Table = out2.parse().unwrap();
        assert!(doc2.contains_key("desk"), "got:\n{out2}");
        assert!(doc2.contains_key("experimental"), "got:\n{out2}");
        assert_eq!(
            doc2["robots"].as_table().unwrap()["orin"].as_str(),
            Some(HEX64_B)
        );
    }

    /// A NON-STRING `[robots]` entry is REFUSED, because
    /// writing it would strand every pin in the file.
    ///
    /// A typed merge would refuse it as a side effect of
    /// deserializing `BTreeMap<String, String>`; editing the document keeps
    /// foreign keys (the point) but does not type-check the one table we
    /// own. It is not a harmless relaxation — `parse_robots_toml`
    /// deserializes that same map, so ONE non-string entry makes the read
    /// yield an EMPTY map, and a pair that "succeeded" would leave
    /// `cerulion connect <name>` finding nothing.
    #[test]
    fn a_non_string_robots_entry_is_refused_before_anything_is_written() {
        // The integer shape, plus the two other JSON-ish scalars a
        // hand-edit produces.
        for (bad, kind) in [
            ("go2 = 123\n", "integer"),
            ("go2 = true\n", "boolean"),
            ("go2 = [\"aabb\"]\n", "array"),
        ] {
            let existing = format!("[robots]\nspot = \"aabb\"\n{bad}");
            let err = merge_robots_toml(&existing, "orin", HEX64).unwrap_err();
            assert!(
                err.contains("go2") && err.contains(kind),
                "the refusal must name the entry and its type, got: {err}"
            );
            assert!(err.contains("--eid"), "and the escape hatch, got: {err}");
        }

        // ANTI-TAUTOLOGY, in the same body: the identical document with the
        // entry corrected merges, so the refusal is attributable to the type
        // rather than to the merge refusing everything.
        let ok = merge_robots_toml("[robots]\nspot = \"aabb\"\ngo2 = \"ccdd\"\n", "orin", HEX64)
            .expect("a well-typed table must merge");
        let doc: toml::Table = ok.parse().unwrap();
        let robots = doc["robots"].as_table().unwrap();
        assert_eq!(robots["orin"].as_str(), Some(HEX64));
        assert_eq!(robots["go2"].as_str(), Some("ccdd"));

        // THE REASON, demonstrated rather than asserted: the reader really
        // does lose the whole file to one bad entry.
        assert!(
            parse_robots_toml("[robots]\nspot = \"aabb\"\ngo2 = 123\n").is_empty(),
            "one non-string entry must make the READ yield an empty map — the \
             fact the refusal above exists to prevent"
        );
    }

    /// A `robots` key of the WRONG SHAPE is REFUSED, not overwritten — the
    /// same no-clobber discipline the malformed and unreadable arms follow. A
    /// hand-edit can produce it (`robots = "go2"`), and silently replacing it
    /// with a table would destroy what the user meant to write.
    #[test]
    fn a_robots_key_that_is_not_a_table_is_refused_not_overwritten() {
        let err = merge_robots_toml("robots = \"go2\"\n", "go2", HEX64).unwrap_err();
        assert!(
            err.contains("not a table"),
            "the refusal must name the shape problem, got: {err}"
        );
        assert!(err.contains("--eid"), "and the escape hatch, got: {err}");
    }

    /// `pin_robot_at` end-to-end over a temp file (parent dirs created): a fresh
    /// pin writes the entry, then a second pin PRESERVES the first (atomic
    /// read→merge→write). No `$HOME` mutation — the path is injected, so it is
    /// parallel-safe and never touches the real `~/.cerulion`.
    #[test]
    fn pin_robot_at_writes_and_preserves() {
        let dir = tempfile::tempdir().unwrap();
        // A nested path proves the parent dirs are created.
        let path = dir.path().join(".cerulion").join("robots.toml");

        pin_robot_at(&path, "go2", HEX64).unwrap();
        pin_robot_at(&path, "spot", HEX64_B).unwrap();

        let text = std::fs::read_to_string(&path).unwrap();
        let m = parse_robots_toml(&text);
        assert_eq!(m.get("go2").map(String::as_str), Some(HEX64));
        assert_eq!(m.get("spot").map(String::as_str), Some(HEX64_B));
        assert_eq!(m.len(), 2, "both pins survive (merge-preserve)");

        // A malformed existing file refuses (no clobber), leaving it intact.
        std::fs::write(&path, "garbage : [[[ not toml").unwrap();
        let err = pin_robot_at(&path, "orin", HEX64).unwrap_err();
        assert!(err.contains("malformed"), "err: {err}");
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            "garbage : [[[ not toml",
            "a malformed robots.toml is NEVER clobbered by a pin"
        );
    }

    /// An unreadable robots.toml (permission denied) is a
    /// loud refusal, NEVER a clobber — an unreadable file must not be conflated
    /// with not-found (which would erase every existing pin). A NotFound path
    /// still pins (the normal first-pin case).
    #[cfg(unix)]
    #[test]
    fn pin_robot_at_refuses_unreadable_file_and_never_clobbers() {
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();

        // NotFound arm: an absent file still pins (start-empty).
        let missing = dir.path().join("fresh.toml");
        pin_robot_at(&missing, "go2", HEX64).expect("a not-yet-existing file pins");
        assert_eq!(
            parse_robots_toml(&std::fs::read_to_string(&missing).unwrap()).len(),
            1
        );

        // Unreadable arm: chmod 000 an existing file with real content → Err, and
        // the file's bytes are UNTOUCHED (never clobbered).
        let locked = dir.path().join("locked.toml");
        let original = format!("[robots]\nspot = \"{HEX64_B}\"\n");
        std::fs::write(&locked, &original).unwrap();
        std::fs::set_permissions(&locked, std::fs::Permissions::from_mode(0o000)).unwrap();

        // Skip the assertion if the test runs as root (0o000 is still readable) —
        // never a false pass, never a false failure.
        if std::fs::read_to_string(&locked).is_err() {
            let err = pin_robot_at(&locked, "go2", HEX64).unwrap_err();
            assert!(err.contains("refusing to overwrite"), "err: {err}");
            // Restore perms to read the bytes back — they must be the ORIGINAL.
            std::fs::set_permissions(&locked, std::fs::Permissions::from_mode(0o644)).unwrap();
            assert_eq!(
                std::fs::read_to_string(&locked).unwrap(),
                original,
                "an unreadable robots.toml is NEVER clobbered"
            );
        } else {
            std::fs::set_permissions(&locked, std::fs::Permissions::from_mode(0o644)).unwrap();
        }
    }

    /// False-pairing guard: `finalize_pairing` pins `name → eid` ONLY on a
    /// GENUINE exit 0. A signal-killed connectd (130), a refusal (2), an
    /// unreachable dial (3), or a wrong code (4) must NOT touch robots.toml. A spy
    /// pin records every call so the gate is proven without a real `~/.cerulion`
    /// write. (The signal→130 mapping is pinned in `connect_cmd`.)
    #[test]
    fn finalize_pairing_pins_only_on_genuine_exit_zero() {
        use std::cell::RefCell;
        let calls: RefCell<Vec<(String, String)>> = RefCell::new(Vec::new());
        let spy = |n: &str, e: &str| -> Result<(), String> {
            calls.borrow_mut().push((n.to_string(), e.to_string()));
            Ok(())
        };

        // Every non-zero outcome — INCLUDING the signal-killed 130 (a Ctrl-C at the
        // code prompt) — must NOT pin.
        for code in [130, 2, 3, 4, 1] {
            finalize_pairing(code, Some("go2"), HEX64, &spy);
            assert!(
                calls.borrow().is_empty(),
                "exit {code} must NOT pin robots.toml (only a genuine 0 pins)"
            );
        }

        // A genuine 0 pins exactly once.
        finalize_pairing(0, Some("go2"), HEX64, &spy);
        assert_eq!(
            &*calls.borrow(),
            &[("go2".to_string(), HEX64.to_string())],
            "a genuine exit 0 pins name→eid exactly once"
        );

        // A genuine 0 with NO name pins nothing more (nothing to pin — no error).
        finalize_pairing(0, None, HEX64, &spy);
        assert_eq!(
            calls.borrow().len(),
            1,
            "a no-name success pins nothing (reconnect with --eid)"
        );
    }

    /// The `pinned:` STDOUT state line has the exact Studio-contract format (hand
    /// oracle — a literal expected string, never a self-compare).
    #[test]
    fn pinned_line_has_exact_format() {
        assert_eq!(
            pinned_line("go2", "deadbeef"),
            "pinned: name=go2 eid=deadbeef"
        );
    }

    /// The `--code` security warn fires ONLY when a code is passed via the flag
    /// (the decision gate) — stdin is the off-argv default.
    #[test]
    fn warn_if_code_in_argv_warns_only_when_code_present() {
        assert!(
            warn_if_code_in_argv(Some("SWAN-42")),
            "--code present → warn"
        );
        assert!(!warn_if_code_in_argv(None), "no --code → no warn");
    }

    /// The `--code` warn actually EMITS the process-listing caution (traced).
    #[tracing_test::traced_test]
    #[test]
    fn warn_if_code_in_argv_emits_the_process_listing_caution() {
        assert!(warn_if_code_in_argv(Some("SWAN-42")));
        assert!(logs_contain("VISIBLE in process listings"));
    }
}

#[cfg(test)]
mod debug_redaction_tests {
    use super::*;
    #[test]
    fn pair_args_debug_never_prints_the_enrollment_code() {
        let args = PairArgs {
            code: Some("PAIR-SENSITIVE-53".into()),
            ..PairArgs::default()
        };
        assert_eq!(format!("{args:?}"), "PairArgs { robot: None, eid: None, addrs: [], code: \"[REDACTED]\", account: None, label: None, key_file: None, relay_url: None, relay_disabled: false }");
        let pretty = format!("{args:#?}");
        assert!(pretty.contains("[REDACTED]"));
        assert!(!pretty.contains("PAIR-SENSITIVE-53"));
    }
    #[test]
    fn pair_plan_debug_keeps_target_and_redacts_complete_argv() {
        let plan = PairPlan {
            bin: PathBuf::from("connectd"),
            argv: vec!["pair".into(), "--code".into(), "PAIR-SENSITIVE-61".into()],
            name: Some("robot-public-vector".into()),
            eid: "public-endpoint-vector".into(),
        };
        assert_eq!(format!("{plan:?}"), "PairPlan { bin: \"connectd\", argv: \"[REDACTED]\", name: Some(\"robot-public-vector\"), eid: \"public-endpoint-vector\" }");
        let pretty = format!("{plan:#?}");
        assert!(pretty.contains("[REDACTED]"));
        assert!(!pretty.contains("PAIR-SENSITIVE-61"));
    }
}
