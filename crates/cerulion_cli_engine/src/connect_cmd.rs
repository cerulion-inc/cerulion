// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion connect` verb's PURE resolution logic.
//!
//! `cerulion connect` makes a REMOTE robot's topics appear as LOCAL desk SHM
//! topics ("remote = local"). The `cerulion` CLI stays iroh-free: it RESOLVES the
//! robot address + the demand set into a `cerulion-connectd` argv and SPAWNS that
//! sibling binary (which owns the iroh half), exactly as the CLI spawns the
//! network gateway / `vizd` — the license + build-graph boundary is a subprocess.
//!
//! This module is PURE + oracle-tested: it builds the exact argv, resolves the
//! robot eid (explicit `--eid`, a 64-char-hex positional, or a name pinned in
//! `~/.cerulion/robots.toml`), and locates the `cerulion-connectd` binary
//! (`CERULION_CONNECTD_BIN` override, else the sibling of the running `cerulion`
//! exe). The spawn/stream/signal wiring lives in the `cerulion` binary.
//!
//! ## Name resolution
//!
//! A positional ROBOT name resolves via `~/.cerulion/robots.toml` — which
//! `cerulion pair <robot>` WRITES on a successful pairing, so a paired robot's
//! name just works here with no flags. (`cerulion pair` additionally resolves a
//! name LIVE from the robot's mDNS TXT `eid=` record —
//! [`crate::mdns_discovery::resolve_robot_eid`]; `connect` uses the pinned
//! `robots.toml` entry the pairing left behind.)

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::{Child, ExitStatus};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::error::{CliError, CliResult};

/// How often the spawn wait-loop polls the child + the shutdown flag.
const CONNECT_POLL_INTERVAL: Duration = Duration::from_millis(50);

/// Grace window for `cerulion-connectd` to shut down after a forwarded SIGINT,
/// before the SIGKILL backstop (mirrors the gateway's `GATEWAY_SHUTDOWN_GRACE`).
const CONNECTD_SHUTDOWN_GRACE: Duration = Duration::from_secs(5);

/// The `cerulion-connectd` binary name (+ the platform exe suffix).
fn connectd_bin_name() -> String {
    format!("cerulion-connectd{}", std::env::consts::EXE_SUFFIX)
}

/// The raw `cerulion connect` CLI flags (the clap-parsed surface). Resolved into a
/// [`ConnectPlan`] by [`plan`].
#[derive(Debug, Clone, Default)]
pub struct ConnectArgs {
    /// The positional ROBOT: a 64-char-hex eid OR a `~/.cerulion/robots.toml` name.
    pub robot: Option<String>,
    /// The explicit `--eid` (overrides a positional name).
    pub eid: Option<String>,
    /// Direct `ip:port` addresses (`--addr`, repeatable).
    pub addrs: Vec<String>,
    /// Topics to demand (`--topic`, repeatable).
    pub topics: Vec<String>,
    /// Demand every catalog topic (`--all`).
    pub all: bool,
    /// The desk device key file (`--key-file`).
    pub key_file: Option<PathBuf>,
    /// The schema-materialization directory (`--schemas-dir`).
    pub schemas_dir: Option<PathBuf>,
    /// Self-hosted relay URL (`--relay-url`).
    pub relay_url: Option<String>,
    /// Disable relays (`--relay-disabled`).
    pub relay_disabled: bool,
    /// The `off` kill-switch (`--network`).
    pub network: Option<String>,
}

/// The resolved spawn plan: the `cerulion-connectd` binary + its argv.
#[derive(Clone, PartialEq, Eq)]
pub struct ConnectPlan {
    /// The resolved `cerulion-connectd` binary path.
    pub bin: PathBuf,
    /// The argv (after the binary) to pass to it.
    pub argv: Vec<String>,
}

impl std::fmt::Debug for ConnectPlan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Pairing reuses this spawn plan, so argv may contain an enrollment code.
        f.debug_struct("ConnectPlan")
            .field("bin", &self.bin)
            .field("argv", &"[REDACTED]")
            .finish()
    }
}

/// Whether `s` looks like a 64-char-hex endpoint id (tolerating a `0x` prefix +
/// surrounding whitespace) — the discriminator between a positional eid and a
/// robots.toml name.
pub fn is_hex_eid(s: &str) -> bool {
    let t = s.trim();
    let t = t.strip_prefix("0x").unwrap_or(t);
    t.len() == 64 && t.bytes().all(|b| b.is_ascii_hexdigit())
}

/// Normalize a candidate eid to its canonical bare-lowercase-hex form: trim
/// surrounding whitespace, strip a `0x` prefix, and lowercase. An ed25519 endpoint
/// id is case-insensitive hex; canonicalizing at this ONE shared seam makes BOTH
/// `cerulion connect` and `cerulion pair` agree on case for every rung (positional
/// / `--eid` / mDNS `eid=` / `robots.toml`), and makes a `robots.toml` pin store
/// the lowercase form (so non-normalized values never leak into logs/comparisons).
/// PURE — oracle-tested.
pub fn normalize_eid(s: &str) -> String {
    let t = s.trim();
    t.strip_prefix("0x").unwrap_or(t).to_ascii_lowercase()
}

/// Parse the `[robots]` table of `~/.cerulion/robots.toml` into name → eid-hex.
/// A malformed document warns and yields an empty map (never a hard failure — the
/// user can still pass `--eid`). PURE.
///
/// **A top-level key that is a NEAR MISS of `robots` warns.**
/// A hand-typed `[robot]` table parses clean and yields an EMPTY map, so a
/// desk with pins in it behaves exactly like one with none.
///
/// A WARN and not `deny_unknown_fields`: this is a NEVER-BRICKS store, listed
/// as such in `config_deny_unknown_fields_test.rs`'s own "deliberately NOT on
/// the list" inventory, and refusing the document would break the discipline
/// this very function states — a malformed file yields an empty map so the
/// user can still pass `--eid`.
pub fn parse_robots_toml(text: &str) -> BTreeMap<String, String> {
    #[derive(serde::Deserialize, Default)]
    struct RobotsDoc {
        #[serde(default)]
        robots: BTreeMap<String, String>,
    }
    match toml::from_str::<RobotsDoc>(text) {
        Ok(doc) => {
            // Reported only on a document that PARSED — the sibling rule in
            // `hostname_peers::parse_config_peers`, which this call site
            // follows for the same reason. Syntactically
            // valid TOML can still fail `RobotsDoc` (`[robot]` beside a
            // non-string `robots` entry), and there the near-miss line adds
            // nothing an operator can act on: their pins are being ignored for
            // a reason they have already been told, and a second line about a
            // different key reads like a second fault.
            warn_on_near_miss_robots_keys(text);
            doc.robots
        }
        Err(e) => {
            tracing::warn!(error = %e, "failed to parse ~/.cerulion/robots.toml; ignoring it");
            BTreeMap::new()
        }
    }
}

/// The keys `~/.cerulion/robots.toml` is READ for.
const ROBOTS_KNOWN_KEYS: &[&str] = &["robots"];

/// Warn once per near-miss top-level key. Silent on a document that will not
/// parse — [`parse_robots_toml`] reports that itself.
fn warn_on_near_miss_robots_keys(text: &str) {
    let Ok(doc) = text.parse::<toml::Table>() else {
        return;
    };
    for (found, expected) in
        crate::near_miss::near_miss_keys(doc.keys().map(String::as_str), ROBOTS_KNOWN_KEYS)
    {
        tracing::warn!(
            found = %found,
            expected = %expected,
            "a top-level key in ~/.cerulion/robots.toml looks like a misspelling (see `found` \
             and `expected`) — it is being IGNORED, so any robot pinned under it is invisible \
             to `cerulion connect`. Rename the key to the expected spelling."
        );
    }
}

/// The `~/.cerulion/robots.toml` path (None when there is no home directory).
pub fn robots_toml_path() -> Option<PathBuf> {
    Some(dirs::home_dir()?.join(".cerulion").join("robots.toml"))
}

/// The well-known desk device key path `~/.cerulion/desk.key` (None when there is
/// no home directory). This is the persistent desk identity `cerulion pair`
/// creates and `cerulion connect` defaults `--key-file` to when it exists — so a
/// paired desk connects with NO flags.
pub fn desk_key_path() -> Option<PathBuf> {
    Some(dirs::home_dir()?.join(".cerulion").join("desk.key"))
}

/// The default `--key-file` to hand `cerulion-connectd connect` when the user
/// passed none: the well-known `~/.cerulion/desk.key` IF it exists (a paired
/// desk), else `None` (an unpaired desk → ephemeral key → the robot refuses it,
/// which is the correct "pair first" signal). A `Some` here is logged at info so
/// the auto-selection is never silent.
fn default_connect_key_file() -> Option<PathBuf> {
    let path = desk_key_path()?;
    if path.exists() {
        tracing::info!(
            path = %path.display(),
            "cerulion connect: using the paired desk key ~/.cerulion/desk.key (pass --key-file to override)"
        );
        Some(path)
    } else {
        None
    }
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

/// Resolve the robot eid-hex from the positional + `--eid` flag against a
/// `robots.toml` map. PURE (the map is injected for testing).
///
/// Precedence: `--eid` (explicit) wins; else a 64-char-hex positional IS the eid;
/// else a positional name is looked up in `robots`. A positional AND `--eid`
/// together is ambiguous (error); neither is a missing-target error.
///
/// Every resolved eid is canonicalized via [`normalize_eid`] — the SAME shared
/// seam `cerulion pair` uses — so an uppercase / `0x`-prefixed `--eid` or
/// `robots.toml` value comes out bare-lowercase (the two verbs never disagree on
/// case, and no non-normalized value leaks downstream).
pub fn resolve_eid_with(
    positional: Option<&str>,
    eid_flag: Option<&str>,
    robots: &BTreeMap<String, String>,
) -> CliResult<String> {
    match (positional, eid_flag) {
        (Some(_), Some(_)) => Err(CliError::Validation(
            "pass EITHER a positional robot (name or eid) OR `--eid`, not both".to_string(),
        )),
        (None, Some(eid)) => Ok(normalize_eid(eid)),
        (Some(pos), None) => {
            if is_hex_eid(pos) {
                Ok(normalize_eid(pos))
            } else if let Some(eid) = robots.get(pos) {
                Ok(normalize_eid(eid))
            } else {
                Err(CliError::Validation(format!(
                    "unknown robot '{pos}': not a 64-char-hex endpoint id and not pinned in \
                     ~/.cerulion/robots.toml. Pair it first with `cerulion pair {pos}` (which \
                     pins it), pass `--eid <hex>`, or add a `[robots]` entry \
                     (`{pos} = \"<64-hex-eid>\"`)."
                )))
            }
        }
        (None, None) => Err(CliError::Validation(
            "specify a robot: a positional name/eid or `--eid <64-hex>` (see \
             `cerulion connect --help`)"
                .to_string(),
        )),
    }
}

/// The DESK-VERIFIED name for the robot being dialed, or `None` when this
/// desk has none. PURE (the `robots` map is injected for testing).
///
/// This — never the name the ROBOT reports about itself — is what keys the desk's
/// revocation-epoch cache (`<robot>.epoch`). A peer-reported key would let the dialed
/// robot choose which cached artifact it is handed, i.e. ask for another robot's signed
/// epoch (whose revoked account/device ids are not its business). The epoch cache is
/// filed under "the name this desk knows the robot by", and `~/.cerulion/robots.toml`
/// is exactly that record — written by `cerulion pair`, which pins the name→eid binding
/// the dial then authenticates cryptographically.
///
/// Two ways a name is desk-verified:
///
/// 1. the user NAMED the robot (a non-hex positional), which `resolve_eid_with`
///    resolved through `robots.toml` — the name and the dialed eid are bound by the
///    desk's own record;
/// 2. the user passed a raw eid, and exactly ONE `robots.toml` entry maps to it (a
///    reverse lookup of the desk's own record).
///
/// `None` when the desk has no record for the eid, or when SEVERAL names map to it
/// (ambiguous — picking one would be a guess about which cache file was meant). The
/// caller then pushes no epoch and says so loudly, rather than trusting peer-supplied
/// text to select a file.
pub fn resolve_robot_name_with(
    positional: Option<&str>,
    resolved_eid: &str,
    robots: &BTreeMap<String, String>,
) -> Option<String> {
    let candidate = robot_name_candidate(positional, resolved_eid, robots)?;
    // Two guards on the CHOSEN name, both of which would otherwise be worse than
    // pushing nothing. Either one ⇒ no `--robot-name`, so the
    // session pushes no epoch and says so, exactly like an unpinned robot.
    if !is_argv_safe_name(&candidate) {
        // The name is ESCAPED for the log (`escape_debug`): the very characters this
        // guard rejects are the ones that would repaint a terminal or forge a second log
        // record if rendered raw — printing them verbatim in the diagnostic ABOUT them
        // would be self-defeating.
        tracing::warn!(
            eid = %resolved_eid,
            name = %candidate.escape_debug(),
            "cerulion connect: this robot's name cannot be passed as a `--robot-name` VALUE — it \
             either starts with '-' (the sibling binary's parser would read it as a flag) or \
             carries a NUL / control character (a NUL cannot cross execve at all), and both abort \
             the whole connect. Pushing NO revocation epoch on this dial rather than \
             failing the dial. Rename the ~/.cerulion/robots.toml entry so it starts with a \
             regular character and contains no control characters."
        );
        return None;
    }
    if let Some(other) = colliding_cache_name(&candidate, resolved_eid, robots) {
        tracing::warn!(
            eid = %resolved_eid,
            name = %candidate,
            collides_with = %other,
            cache_file = %cerulion_pairing::verify::epoch_cache_file_name(&candidate),
            "cerulion connect: two ~/.cerulion/robots.toml names for DIFFERENT robots resolve to \
             the SAME <robot>.epoch cache file (the file name folds path separators, and a \
             case-insensitive filesystem folds case), so one robot's signed epoch would be read \
             for the other. Pushing NO revocation epoch on this dial. Rename one of the \
             two entries so their cache files differ."
        );
        return None;
    }
    Some(candidate)
}

/// The name the desk would key by, BEFORE the safety guards: the named positional, else
/// a UNIQUE reverse lookup of the dialed eid. `None` when the desk has no record, or when
/// several names map to the eid (ambiguous — picking one would be a guess).
fn robot_name_candidate(
    positional: Option<&str>,
    resolved_eid: &str,
    robots: &BTreeMap<String, String>,
) -> Option<String> {
    // 1. A named positional IS the desk's name for this robot. Trimmed to match the
    //    tolerance `is_hex_eid` / `normalize_eid` apply to the SAME argument, so a
    //    padded hex positional is classified as an eid rather than taken for a name.
    //    (A padded NAME cannot reach here: `resolve_eid_with` looks names up VERBATIM
    //    and errors first — see `resolve_robot_name_is_desk_verified_or_none`.)
    if let Some(pos) = positional.map(str::trim).filter(|p| !p.is_empty()) {
        if !is_hex_eid(pos) {
            return Some(pos.to_string());
        }
    }
    // 2. Reverse-look the dialed eid up in the desk's own pin file.
    let mut matches = robots
        .iter()
        .filter(|(_, eid)| normalize_eid(eid) == resolved_eid)
        .map(|(name, _)| name.clone());
    let first = matches.next()?;
    match matches.next() {
        None => Some(first),
        Some(second) => {
            tracing::warn!(
                eid = %resolved_eid,
                names = %format!("{first}, {second}, …"),
                "cerulion connect: several ~/.cerulion/robots.toml names point at this robot, so \
                 the desk cannot tell which <robot>.epoch cache is meant — pushing NO revocation \
                 epoch on this dial. Dial it by the name whose cache you want, or \
                 remove the duplicate entries."
            );
            None
        }
    }
}

/// Whether `name` can be passed as a clap VALUE (`--robot-name <name>`).
///
/// `~/.cerulion/robots.toml` is a plain TOML file whose keys are unconstrained, so a name
/// can carry things a shell argument never could. Two classes are refused — both because
/// they ABORT THE WHOLE `cerulion connect` VERB rather than merely skipping the epoch
/// push:
///
/// 1. **A leading `-`** — the sibling binary's parser reads the value as a FLAG, fails
///    argument parsing, and takes the verb down with it.
/// 2. **NUL and other control characters** — a NUL cannot cross
///    `execve` at all: `Command::arg` rejects the argument and the spawn fails, again
///    aborting the verb. The other C0/DEL controls do cross, but land as terminal escapes
///    and forged newlines in the child's own diagnostics and in `ps` output — the same
///    untrusted-text class the desk sanitizes everywhere it renders peer names. Refusing
///    them here keeps the failure a SKIPPED EPOCH PUSH (the caller falls back to no
///    `--robot-name`, and the dial still works) instead of a dead verb.
///
/// Pure — oracle-tested.
fn is_argv_safe_name(name: &str) -> bool {
    !name.starts_with('-') && !name.chars().any(|c| c.is_control())
}

/// The FIRST other pinned name (for a DIFFERENT robot) whose `<robot>.epoch` cache file
/// would COLLIDE with `name`'s, or `None` when the cache key is unambiguous.
///
/// Two ways distinct names collapse onto one file: the shared file-name rule replaces path
/// separators (`a/b` and `a:b` both become `a_b.epoch` — a collision on EVERY filesystem),
/// and a case-insensitive/normalizing filesystem (macOS APFS by default, Windows) folds
/// `Go2.epoch` onto `go2.epoch`. Either way the desk would read one robot's signed epoch
/// as the other's. The comparison folds case, so the answer is the same on every platform
/// (a config that silently works on Linux and mis-delivers on a Mac is worse than a loud
/// refusal on both). Pure — oracle-tested.
///
/// # Scope (limits)
///
/// This is a `cerulion connect` guard, and it detects exactly two collision mechanisms:
///
/// - **Where**: only names pinned in the SAME `~/.cerulion/robots.toml` this dial reads.
///   A cache file written by some other means, or a robot whose entry is absent, is not
///   compared against — there is nothing here to compare it to.
/// - **What**: separator-folding (deterministic, every platform) and ASCII/Unicode
///   case-folding via `to_lowercase`. It does NOT model a filesystem's full normalization:
///   APFS additionally normalizes Unicode (NFD/NFC), so two names differing only in
///   composition (`é` as one code point vs `e` + U+0301) fold to one file on a Mac and
///   are NOT caught here. That residual is accepted rather than half-modelled — a
///   platform-dependent guard would be exactly the "works on Linux, mis-delivers on a
///   Mac" failure this function exists to prevent, just moved one layer down.
/// - **Who**: `cerulion connect` only. The netd WAN plane keys its cache by the demand's
///   robot name through the same shared file-name rule but does not run this check, and
///   the account-page cache WRITER does not either.
fn colliding_cache_name(
    name: &str,
    resolved_eid: &str,
    robots: &BTreeMap<String, String>,
) -> Option<String> {
    let key = |n: &str| cerulion_pairing::verify::epoch_cache_file_name(n).to_lowercase();
    let mine = key(name);
    robots
        .iter()
        .find(|(other, eid)| {
            other.as_str() != name && normalize_eid(eid) != resolved_eid && key(other) == mine
        })
        .map(|(other, _)| other.clone())
}

/// Build the exact `cerulion-connectd` argv (after the binary) for a resolved
/// `eid` + the flags. PURE — the byte-exact argv oracle-tested.
///
/// The argv leads with the `connect` SUBCOMMAND — `cerulion-connectd` dispatches
/// `connect` (this re-inject client) vs `pair` (the CPace pairing ceremony).
///
/// `robot_name` is the DESK-VERIFIED name from [`resolve_robot_name_with`] (`None` when
/// this desk has no record for the eid). It is passed as `--robot-name` and is what the
/// session keys its revocation-epoch cache by — never the name the robot reports about
/// itself.
pub fn build_connectd_argv(eid: &str, robot_name: Option<&str>, args: &ConnectArgs) -> Vec<String> {
    let mut v = vec!["connect".to_string(), "--eid".to_string(), eid.to_string()];
    if let Some(name) = robot_name {
        v.push("--robot-name".to_string());
        v.push(name.to_string());
    }
    for a in &args.addrs {
        v.push("--addr".to_string());
        v.push(a.clone());
    }
    for t in &args.topics {
        v.push("--topic".to_string());
        v.push(t.clone());
    }
    if args.all {
        v.push("--all".to_string());
    }
    if let Some(k) = &args.key_file {
        v.push("--key-file".to_string());
        v.push(k.display().to_string());
    }
    if let Some(d) = &args.schemas_dir {
        v.push("--schemas-dir".to_string());
        v.push(d.display().to_string());
    }
    if let Some(u) = &args.relay_url {
        v.push("--relay-url".to_string());
        v.push(u.clone());
    }
    if args.relay_disabled {
        v.push("--relay-disabled".to_string());
    }
    if let Some(n) = &args.network {
        v.push("--network".to_string());
        v.push(n.clone());
    }
    v
}

/// Resolve the `cerulion-connectd` binary from an explicit override or the
/// directories holding the running `cerulion` exe. PURE (inputs injected).
///
/// `env_override` is `CERULION_CONNECTD_BIN` (must exist if set); else
/// `cerulion-connectd` beside `exe_dir`, then beside `resolved_dir`. A missing
/// binary is a LOUD error naming every directory tried and the build command (it
/// is NOT in the default build — it links iroh).
///
/// # Why the second sibling rung exists
///
/// `std::env::current_exe()` is not canonicalised on macOS — a binary invoked
/// through a symlink reports the SYMLINK — so a symlinked `cerulion` looked only
/// in the link's directory and hard-errored, naming a directory nobody had reason
/// to suspect. That is the symlinked-binary class; the sweep that fixed `cerulion-netd` and
/// `cerulion-vizd` did not reach this verb.
///
/// Scoped deliberately to the SIBLING rungs. The env rung keeps `exists()` and
/// keeps REQUIRING existence — connectd diverges from netd there on purpose (netd
/// takes its override verbatim so a wrong one fails naming itself; connectd
/// refuses up front), and that arm is pinned by `resolve_connectd_bin_oracle`.
/// Mechanically "harmonising" it would break a deliberate difference.
///
/// `resolved_dir` is deduped against `exe_dir`: they agree on Linux
/// (`/proc/self/exe` is already resolved) and on any un-symlinked install, so the
/// rung can only ever ADD a directory the running binary genuinely came from.
pub fn resolve_connectd_bin_from_dirs(
    env_override: Option<PathBuf>,
    exe_dir: &Path,
    resolved_dir: Option<&Path>,
    exists: &dyn Fn(&Path) -> bool,
) -> CliResult<PathBuf> {
    if let Some(p) = env_override {
        return if exists(&p) {
            Ok(p)
        } else {
            Err(CliError::Validation(format!(
                "CERULION_CONNECTD_BIN points to '{}', which does not exist",
                p.display()
            )))
        };
    }
    let mut tried: Vec<PathBuf> = Vec::new();
    for dir in [Some(exe_dir), resolved_dir].into_iter().flatten() {
        let candidate = dir.join(connectd_bin_name());
        if tried.contains(&candidate) {
            continue;
        }
        if exists(&candidate) {
            return Ok(candidate);
        }
        tried.push(candidate);
    }
    let looked = tried
        .iter()
        .enumerate()
        .map(|(i, p)| format!("({}) {}", i + 1, p.display()))
        .collect::<Vec<_>>()
        .join(" ");
    Err(CliError::Validation(format!(
        "`cerulion-connectd` was not found next to the `cerulion` binary (looked at {looked}). \
         The release packages place it beside `cerulion`; it is NOT part of a `cargo install` \
         or of the default source build (it links iroh). From a checkout of the Cerulion \
         source tree build it with:\n    \
         cargo build -p cerulion_connectd\nor set CERULION_CONNECTD_BIN to its path."
    )))
}

/// Production binary resolution: [`resolve_connectd_bin_from_dirs`] over
/// `CERULION_CONNECTD_BIN` + `std::env::current_exe()` + its canonicalised form.
pub fn resolve_connectd_bin() -> CliResult<PathBuf> {
    let env_override = std::env::var_os("CERULION_CONNECTD_BIN").map(PathBuf::from);
    let exe = std::env::current_exe()?;
    let exe_dir = exe.parent().ok_or_else(|| {
        CliError::Validation(
            "could not determine the directory of the running `cerulion` binary to locate \
             `cerulion-connectd`; set CERULION_CONNECTD_BIN"
                .to_string(),
        )
    })?;
    // The second sibling rung's input. `.ok()` because a canonicalize failure is the
    // ABSENCE of a rung, never a reason to stop looking.
    let resolved = std::fs::canonicalize(&exe).ok();
    let resolved_dir = resolved.as_deref().and_then(Path::parent);
    resolve_connectd_bin_from_dirs(env_override, exe_dir, resolved_dir, &|p| p.exists())
}

/// Resolve the full spawn plan: the robot eid + the argv + the binary path.
///
/// When the user passed no `--key-file`, the paired desk key
/// `~/.cerulion/desk.key` is used automatically IF it exists (see
/// `default_connect_key_file`) — so a desk paired via `cerulion pair` connects
/// with no flags.
pub fn plan(args: &ConnectArgs) -> CliResult<ConnectPlan> {
    // ONE read of `robots.toml` per plan. If `resolve_eid` and
    // `resolve_robot_name` each called `load_robots`, a near-miss key
    // in that file would be reported TWICE on every affected connection — against
    // the one-warning behaviour the parser's own doc promises. Reading once is
    // also the consistent shape: both answers should come from the same file
    // contents.
    let robots = load_robots();
    let eid = resolve_eid_with(args.robot.as_deref(), args.eid.as_deref(), &robots)?;
    // The name the DESK knows this robot by (from the positional the user
    // typed, or a reverse lookup of the dialed eid in `robots.toml`) — the key the
    // session's revocation-epoch cache lookup uses, so the dialed robot can never
    // choose which cached artifact it is handed.
    let robot_name = resolve_robot_name_with(args.robot.as_deref(), &eid, &robots);
    let mut args = args.clone();
    if args.key_file.is_none() {
        args.key_file = default_connect_key_file();
    }
    let argv = build_connectd_argv(&eid, robot_name.as_deref(), &args);
    let bin = resolve_connectd_bin()?;
    Ok(ConnectPlan { bin, argv })
}

/// Spawn `cerulion-connectd` (inheriting stdio) and wait for it, FORWARDING a
/// termination signal to the child + reaping it (f4).
///
/// `running` is the flag `setup_ctrlc_handler` flips to `false` on
/// SIGINT/SIGTERM/SIGHUP. The wait-loop polls the child (natural exit / robot
/// disconnect / a terminal Ctrl-C the child also received) AND the flag; when the
/// flag flips (a DIRECTED `kill <cerulion-pid>` / systemd SIGTERM the child did
/// NOT receive), it forwards SIGINT to the child for a graceful shutdown, then
/// reaps it — so the parent never hangs and the child is never orphaned. Returns
/// the child's exit code — see `exit_code_of`: a signal-KILLED child maps to
/// `128 + signo` (NEVER a synthesized 0), so a keystroke-killed child never reads
/// as a clean success (a graceful signal handler still exits 0 on its own).
pub fn spawn_and_wait(plan: &ConnectPlan, running: Arc<AtomicBool>) -> CliResult<i32> {
    let mut cmd = std::process::Command::new(&plan.bin);
    cmd.args(&plan.argv);
    // Make the forwarded shutdown SIGINT
    // ([`forward_shutdown_to_child`]) DELIVERABLE regardless of the session this
    // process runs in. WHY, and which launchers do this, lives in ONE place now —
    // [`crate::child_signals`] — because `ros2_graph::Ros2Children::spawn` needs
    // exactly the same thing for exactly the same reason, and shipped without it.
    // Note it closes BOTH halves (an inherited SIG_IGN and an inherited BLOCK), not
    // just the disposition this comment originally described.
    //
    // What is specific to THIS site is the exit-code contract: a child that dies by
    // the forwarded SIGINT maps to 130, where the SIGKILL backstop would map to 137
    // — the exact failure the two `spawn_and_wait` signal tests hit under the gate's
    // detached context. A handler-owning `cerulion-connectd` re-installs its
    // `ctrl_c` handler over SIG_DFL cleanly, and an un-handled child dies by
    // SIGINT → 128+2. This NEVER weakens the non-zero guard.
    crate::child_signals::make_sigint_deliverable(&mut cmd);
    let mut child = cmd.spawn().map_err(|e| {
        // The "build it" remedy is right for ENOENT and WRONG for anything else —
        // including a `pre_exec` refusal from `make_sigint_deliverable`, which
        // would otherwise send the operator to rebuild a binary that is already
        // there. `ros2_graph::Ros2Children::spawn` splits the same two cases.
        CliError::Validation(if e.kind() == std::io::ErrorKind::NotFound {
            format!(
                "failed to spawn '{}': {e}. The release packages place `cerulion-connectd` \
                 beside `cerulion`; from a checkout of the Cerulion source tree build it with \
                 `cargo build -p cerulion_connectd`, or set CERULION_CONNECTD_BIN.",
                plan.bin.display()
            )
        } else {
            format!("failed to spawn '{}': {e}", plan.bin.display())
        })
    })?;
    loop {
        match child.try_wait() {
            // Natural exit (robot disconnect / stdin-EOF / a terminal Ctrl-C the
            // child also handled) — forward its exit code.
            Ok(Some(status)) => return Ok(exit_code_of(status)),
            Ok(None) => {}
            Err(e) => {
                return Err(CliError::Validation(format!(
                    "waiting on cerulion-connectd failed: {e}"
                )))
            }
        }
        if !running.load(Ordering::SeqCst) {
            // A directed SIGINT/SIGTERM/SIGHUP reached `cerulion` (the child did
            // not get it) — forward + reap so nothing is orphaned. A missing final
            // status (the wait errored) maps to a nonzero (1), NEVER a synthesized
            // 0 — the child's fate is unknown, which is not a clean success.
            let status = forward_shutdown_to_child(&mut child);
            return Ok(status.map(exit_code_of).unwrap_or(1));
        }
        std::thread::sleep(CONNECT_POLL_INTERVAL);
    }
}

/// Map a child `ExitStatus` to an `i32` exit code the CLI forwards.
///
/// A child that EXITED carries its code. A signal-TERMINATED child carries NO
/// code — it is mapped to the Unix convention `128 + signo` (SIGINT → 130,
/// SIGKILL → 137), NEVER a synthesized 0. This is the false-pairing
/// guard: a keystroke-killed `cerulion-connectd` must not read as a clean 0,
/// because `pair_cmd::finalize_pairing` pins `robots.toml` ONLY on a genuine 0.
/// (A child with its own graceful signal handler exits 0 on its own — that is a
/// real code, not this path.)
fn exit_code_of(status: ExitStatus) -> i32 {
    // `ExitStatus` is `Copy`, so the fallback closure re-uses it freely.
    status.code().unwrap_or_else(|| signal_exit_code(status))
}

/// The `128 + signo` exit code for a signal-terminated child.
#[cfg(unix)]
fn signal_exit_code(status: ExitStatus) -> i32 {
    use std::os::unix::process::ExitStatusExt;
    status.signal().map(|s| 128 + s).unwrap_or(1)
}

/// Non-Unix: no signal number available — a generic nonzero (never 0).
#[cfg(not(unix))]
fn signal_exit_code(_status: ExitStatus) -> i32 {
    1
}

/// Forward a graceful shutdown to the child + reap it, returning its final status
/// (mirrors `graph_cmd::GatewayChild::forward_shutdown`).
///
/// Unix: SIGINT (the signal `cerulion-connectd`'s `ctrl_c` handler honors) for a
/// [`CONNECTD_SHUTDOWN_GRACE`] window, then a SIGKILL backstop. Non-Unix: `kill`
/// (there is no `libc::kill`).
#[cfg(unix)]
fn forward_shutdown_to_child(child: &mut Child) -> Option<ExitStatus> {
    let pid = child.id();
    // SAFETY: FFI `kill` of OUR OWN un-reaped child pid with SIGINT. The child is
    // owned + un-reaped (nothing calls `wait` on it until the poll loop below), so it
    // lingers (as a zombie if already exited) holding its pid until we reap it —
    // the kernel cannot recycle the number under us. A just-exited race yields
    // ESRCH, which is benign (we reap it via `try_wait` below).
    let ret = unsafe { libc::kill(pid as libc::pid_t, libc::SIGINT) };
    if ret != 0 {
        let err = std::io::Error::last_os_error();
        if err.raw_os_error() != Some(libc::ESRCH) {
            tracing::warn!(
                pid,
                error = %err,
                "kill(SIGINT) to cerulion-connectd FAILED — falling back to the SIGKILL backstop"
            );
        }
    }
    // Poll for graceful exit within the grace window.
    let deadline = Instant::now() + CONNECTD_SHUTDOWN_GRACE;
    while Instant::now() < deadline {
        match child.try_wait() {
            Ok(Some(status)) => return Some(status),
            Ok(None) => std::thread::sleep(CONNECT_POLL_INTERVAL),
            Err(_) => break,
        }
    }
    // Grace expired (or the poll errored): SIGKILL + reap.
    let _ = child.kill();
    child.wait().ok()
}

/// Non-Unix: no `libc::kill` — SIGKILL + reap is the only teardown.
#[cfg(not(unix))]
fn forward_shutdown_to_child(child: &mut Child) -> Option<ExitStatus> {
    let _ = child.kill();
    child.wait().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn robots(pairs: &[(&str, &str)]) -> BTreeMap<String, String> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect()
    }

    // Exactly 64 hex chars (32 + 32).
    const HEX64: &str = "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";

    /// A hand-typed `[robot]` table is REPORTED, and the read
    /// still degrades to an empty map rather than refusing (the never-bricks
    /// posture — the user can still pass `--eid`).
    #[tracing_test::traced_test]
    #[test]
    fn a_misspelled_robots_table_warns_and_the_read_still_degrades() {
        let m = parse_robots_toml("[robot]\ngo2 = \"aabb\"\n");
        assert!(m.is_empty(), "a misspelled table still yields no pins");
        assert!(
            logs_contain("looks like a misspelling"),
            "a near-miss key must be reported"
        );
        assert!(logs_contain("found=robot"), "the offending key is named");
        assert!(logs_contain("expected=robots"), "and the intended one");
        assert!(logs_contain("WARN"), "at WARN, not below it");
    }

    /// ANTI-TAUTOLOGY: the correct spelling reads its pins and says nothing,
    /// and a foreign key stays legal and silent.
    #[tracing_test::traced_test]
    #[test]
    fn a_correct_robots_table_and_a_foreign_key_are_both_silent() {
        let m = parse_robots_toml("[robots]\ngo2 = \"aabb\"\n\n[desk]\ntheme = \"dark\"\n");
        assert_eq!(m.get("go2").map(String::as_str), Some("aabb"));
        assert!(
            !logs_contain("looks like a misspelling"),
            "neither `robots` nor `desk` may be reported as a misspelling"
        );
    }

    #[test]
    fn is_hex_eid_oracle() {
        assert_eq!(HEX64.len(), 64, "the test fixture must be a 64-char hex id");
        assert!(is_hex_eid(HEX64));
        assert!(is_hex_eid(&format!("0x{HEX64}")));
        assert!(is_hex_eid(&format!("  {HEX64}  ")));
        assert!(!is_hex_eid("robo1"));
        assert!(!is_hex_eid("aabbcc")); // too short
        assert!(!is_hex_eid(&format!("{HEX64}ff")), "too long"); // 66
                                                                 // A 64-char string with non-hex chars ('zz' prefix) is NOT an eid.
        assert!(!is_hex_eid(
            "zz112233445566778899aabbccddeeff00112233445566778899aabbccddeeff"
        ));
    }

    #[test]
    fn resolve_eid_precedence() {
        let r = robots(&[("robo1", HEX64)]);
        // --eid wins.
        assert_eq!(resolve_eid_with(None, Some(HEX64), &r).unwrap(), HEX64);
        // A hex positional IS the eid.
        assert_eq!(resolve_eid_with(Some(HEX64), None, &r).unwrap(), HEX64);
        // A name resolves via robots.toml.
        assert_eq!(resolve_eid_with(Some("robo1"), None, &r).unwrap(), HEX64);
        // Both → ambiguous error.
        let e = resolve_eid_with(Some("robo1"), Some(HEX64), &r).unwrap_err();
        assert!(e.to_string().contains("not both"), "err: {e}");
        // Unknown name → error naming robots.toml.
        let e = resolve_eid_with(Some("nope"), None, &r).unwrap_err();
        assert!(e.to_string().contains("robots.toml"), "err: {e}");
        assert!(e.to_string().contains("nope"), "err names the robot: {e}");
        // Neither → missing-target error.
        let e = resolve_eid_with(None, None, &r).unwrap_err();
        assert!(e.to_string().contains("specify a robot"), "err: {e}");
    }

    /// `normalize_eid` (the shared canonicalization seam for `connect` + `pair`):
    /// trims, strips `0x`, lowercases (hand oracle).
    #[test]
    fn normalize_eid_oracle() {
        assert_eq!(normalize_eid("DEADBEEF"), "deadbeef");
        assert_eq!(normalize_eid("  0xDeAdBeEf  "), "deadbeef");
        assert_eq!(
            normalize_eid("deadbeef"),
            "deadbeef",
            "already-canonical unchanged"
        );
    }

    /// `connect`'s resolver returns canonical lowercase for
    /// an uppercase / `0x`-prefixed `--eid`, a hex positional, AND a `robots.toml`
    /// value — the same canonicalization `pair` applies (hand oracle = the
    /// lowercased eid).
    #[test]
    fn resolve_eid_with_normalizes_case() {
        let upper = HEX64.to_uppercase();
        // Uppercase --eid → lowercased.
        assert_eq!(
            resolve_eid_with(None, Some(&upper), &robots(&[])).unwrap(),
            HEX64
        );
        // `0x`-prefixed --eid → bare lowercase.
        assert_eq!(
            resolve_eid_with(None, Some(&format!("0x{upper}")), &robots(&[])).unwrap(),
            HEX64
        );
        // Uppercase 64-hex positional → lowercased.
        assert_eq!(
            resolve_eid_with(Some(&upper), None, &robots(&[])).unwrap(),
            HEX64
        );
        // An uppercase robots.toml value → lowercased (non-normalized never leaks).
        assert_eq!(
            resolve_eid_with(Some("go2"), None, &robots(&[("go2", &upper)])).unwrap(),
            HEX64
        );
    }

    /// `plan` reads `robots.toml` exactly ONCE.
    ///
    /// It called `resolve_eid` and `resolve_robot_name`, and each of those
    /// called `load_robots` — so the file was read twice and a near-miss key
    /// in it was reported TWICE on every affected connection, against the
    /// one-warning behaviour `parse_robots_toml`'s own doc promises. `plan`
    /// now reads once and hands the map to both `_with` forms.
    ///
    /// STRUCTURAL, deliberately: the doubling is only observable through
    /// `plan`, which reads the real `~/.cerulion/robots.toml`, and a test that
    /// wrote to the user's HOME to count log lines would be worse than the bug.
    /// The two resolvers are individually correct, so what needs pinning is
    /// the call SHAPE.
    #[test]
    fn plan_reads_robots_toml_exactly_once() {
        let src = include_str!("connect_cmd.rs");
        let body = enclosing_fn_body(src, "pub fn plan(args: &ConnectArgs)");
        assert_eq!(
            body.matches("load_robots()").count(),
            1,
            "plan must read robots.toml once and share the map\n{body}"
        );
        // And it must reach the resolvers through their injectable forms —
        // calling the wrappers back would re-read the file.
        assert!(
            body.contains("resolve_eid_with(") && body.contains("resolve_robot_name_with("),
            "plan must call the map-taking resolvers\n{body}"
        );
    }

    /// ANTI-TAUTOLOGY for the walk above: the extractor must isolate ONE
    /// function, or its count is over the whole file and means nothing.
    #[test]
    fn the_body_extractor_isolates_one_function() {
        let src = "fn a() {\n  load_robots();\n}\nfn b() {\n  let y = 2;\n}\n";
        let b = enclosing_fn_body(src, "fn b()");
        assert!(b.contains("let y"));
        assert!(!b.contains("load_robots"));
    }

    /// The brace-matched body of the function whose signature starts with
    /// `signature`.
    fn enclosing_fn_body<'a>(src: &'a str, signature: &str) -> &'a str {
        let start = src
            .find(signature)
            .unwrap_or_else(|| panic!("no function matching {signature:?}"));
        let open = start + src[start..].find('{').expect("a body");
        let mut depth = 0usize;
        for (offset, ch) in src[open..].char_indices() {
            match ch {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        return &src[open..=open + offset];
                    }
                }
                _ => {}
            }
        }
        panic!("unbalanced body for {signature:?}");
    }

    /// A `robots.toml` that will not DESERIALIZE
    /// gets exactly ONE warn — the parse one — never a near-miss line on top.
    ///
    /// The sibling rule in `hostname_peers::parse_config_peers` was fixed
    /// first and THIS call site was missed: the scan ran before
    /// the typed parse, so syntactically valid TOML that fails `RobotsDoc`
    /// (`[robot]` beside a non-string `robots` entry) produced two lines — one
    /// saying the pins were ignored, and a second about a different key, which
    /// reads like a second fault when the operator has already been told the
    /// actionable thing.
    ///
    /// The oracle matches the LEVEL TOKEN as well as the message: a `debug!`
    /// regression would leave the map empty exactly as the silence did.
    #[tracing_test::traced_test]
    #[test]
    fn a_wrongly_shaped_robots_table_gets_one_warn_not_two() {
        // Syntactically VALID TOML, so the near-miss walk parses it happily —
        // the type failure is downstream, which is what made the second line
        // reachable.
        let pins = parse_robots_toml("[robots]\ngo2 = 123\n[robot]\norin = \"aabb\"\n");
        assert!(pins.is_empty(), "a non-string robot entry yields no pins");
        assert!(
            logs_contain("failed to parse ~/.cerulion/robots.toml"),
            "the parse failure is the ONE warn this document earns"
        );
        assert!(
            !logs_contain("looks like a misspelling"),
            "and the near-miss line must not ride on top of it"
        );

        // ANTI-TAUTOLOGY: the same near-miss key in a document that DOES
        // deserialize is still reported — the gate suppresses a redundant
        // line, not the feature.
        let pins = parse_robots_toml("[robots]\ngo2 = \"aabb\"\n[robot]\norin = \"ccdd\"\n");
        assert_eq!(pins.get("go2").map(String::as_str), Some("aabb"));
        assert!(
            logs_contain("looks like a misspelling"),
            "a parseable document still reports its near-miss key"
        );
        assert!(logs_contain("found=robot"), "the offending key is named");
        assert!(logs_contain("expected=robots"), "and the intended one");
        assert!(logs_contain("WARN"), "at WARN, not below it");
    }

    #[test]
    fn parse_robots_toml_oracle() {
        let text = "[robots]\nrobo1 = \"aabb\"\ngo2 = \"ccdd\"\n";
        let m = parse_robots_toml(text);
        assert_eq!(m.get("robo1").map(String::as_str), Some("aabb"));
        assert_eq!(m.get("go2").map(String::as_str), Some("ccdd"));
        // No table → empty.
        assert!(parse_robots_toml("").is_empty());
        // Malformed → empty (warned, never a hard failure).
        assert!(parse_robots_toml("this is : not toml [[[").is_empty());
    }

    /// The cache key is the name the DESK knows the
    /// robot by — the positional the user typed, or a UNIQUE reverse lookup of the
    /// dialed eid in the desk's own `robots.toml`. Never the peer's self-report, and
    /// never a guess when the desk's record is ambiguous or absent. Hand oracles.
    #[test]
    fn resolve_robot_name_is_desk_verified_or_none() {
        let robots: BTreeMap<String, String> = [
            ("go2".to_string(), HEX64.to_string()),
            ("orin".to_string(), "b".repeat(64)),
        ]
        .into_iter()
        .collect();

        // 1. A NAMED positional is the desk's name (it is what resolved the eid).
        assert_eq!(
            resolve_robot_name_with(Some("go2"), HEX64, &robots),
            Some("go2".to_string())
        );
        // The positional is trimmed the way `is_hex_eid` / `normalize_eid` trim the SAME
        // argument, so a PADDED HEX positional is classified as an eid (the reachable
        // case) and falls through to the reverse lookup rather than being taken for a
        // robots.toml name:
        assert_eq!(
            resolve_robot_name_with(Some(&format!("  {HEX64}  ")), HEX64, &robots),
            Some("go2".to_string()),
            "a padded hex positional is an EID, resolved by reverse lookup — not a name"
        );
        // A padded NAME is DEFENSE IN DEPTH, not a reachable path: `resolve_eid_with`
        // looks a positional name up VERBATIM (`robots.get(pos)`, no trim), so
        // `cerulion connect "  go2  "` fails eid resolution and `plan` never reaches
        // here. Pinned so the two functions' tolerance cannot silently diverge in the
        // OTHER direction (this one being laxer is harmless; the reverse would key the
        // cache off untrimmed text).
        assert!(resolve_eid_with(Some("  go2  "), None, &robots).is_err());
        assert_eq!(
            resolve_robot_name_with(Some("  go2  "), HEX64, &robots),
            Some("go2".to_string())
        );

        // 2. A raw eid (positional or `--eid`) reverse-looks-up the desk's pin file.
        assert_eq!(
            resolve_robot_name_with(Some(HEX64), HEX64, &robots),
            Some("go2".to_string())
        );
        assert_eq!(
            resolve_robot_name_with(None, HEX64, &robots),
            Some("go2".to_string())
        );
        // The reverse lookup is case/prefix-insensitive on the STORED value, because
        // both sides normalize (an uppercase `robots.toml` pin still matches).
        let upper: BTreeMap<String, String> = [("go2".to_string(), HEX64.to_uppercase())]
            .into_iter()
            .collect();
        assert_eq!(
            resolve_robot_name_with(None, HEX64, &upper),
            Some("go2".to_string())
        );

        // 3. No record for the dialed eid ⇒ None (the session pushes nothing rather
        //    than trusting the peer's self-report to pick a file).
        assert_eq!(
            resolve_robot_name_with(None, &"c".repeat(64), &robots),
            None
        );
        assert_eq!(resolve_robot_name_with(None, HEX64, &BTreeMap::new()), None);

        // 4. AMBIGUOUS (two names pinned to one eid) ⇒ None: picking one would be a
        //    guess about which cache file the operator meant.
        let dup: BTreeMap<String, String> = [
            ("go2".to_string(), HEX64.to_string()),
            ("go2-lab".to_string(), HEX64.to_string()),
        ]
        .into_iter()
        .collect();
        assert_eq!(resolve_robot_name_with(None, HEX64, &dup), None);
        // …but an explicitly NAMED positional still wins over the ambiguity (the user
        // said which one).
        assert_eq!(
            resolve_robot_name_with(Some("go2-lab"), HEX64, &dup),
            Some("go2-lab".to_string())
        );
    }

    /// A `robots.toml` name that would BREAK the spawn, or that
    /// would make two robots share ONE epoch-cache file, yields NO desk-verified name —
    /// so the session pushes nothing (a policy no-op) instead of aborting the verb or
    /// delivering the wrong robot's signed epoch. Hand oracles.
    #[test]
    fn a_hostile_or_colliding_robots_toml_name_is_not_a_desk_verified_name() {
        // (a) A leading dash: `--robot-name -evil` is parsed as a FLAG by the sibling
        //     binary, which aborts argument parsing and takes the WHOLE verb down. The
        //     epoch push is expendable; the dial is not.
        let dashed: BTreeMap<String, String> = [("-evil".to_string(), HEX64.to_string())]
            .into_iter()
            .collect();
        assert_eq!(resolve_robot_name_with(None, HEX64, &dashed), None);
        // …via the positional spelling too (`cerulion connect -- -evil`).
        assert_eq!(resolve_robot_name_with(Some("-evil"), HEX64, &dashed), None);
        // An INTERIOR dash is fine — only a LEADING one is a flag.
        let inner: BTreeMap<String, String> = [("go2-lab".to_string(), HEX64.to_string())]
            .into_iter()
            .collect();
        assert_eq!(
            resolve_robot_name_with(None, HEX64, &inner),
            Some("go2-lab".to_string())
        );

        // (a2) NUL and the other control characters are the SAME
        //      verb-abort class. A NUL cannot cross `execve` at all — `Command::arg`
        //      refuses it and the spawn fails — and ESC/CR/newline/BEL land as terminal
        //      escapes and forged lines in the child's diagnostics and in `ps`. Each is
        //      refused, so the epoch push is skipped and the DIAL still works.
        for hostile in [
            "go2\u{0}evil", // NUL — the execve killer
            "go2\u{1b}[2J", // ESC screen-clear
            "go2\nmore",    // forged newline
            "go2\revil",    // CR overwrite
            "go2\u{7}",     // BEL
            "go2\u{7f}",    // DEL
            "\u{0}",        // the degenerate all-control name
        ] {
            let m: BTreeMap<String, String> = [(hostile.to_string(), HEX64.to_string())]
                .into_iter()
                .collect();
            assert_eq!(
                resolve_robot_name_with(None, HEX64, &m),
                None,
                "a control character in {hostile:?} must not reach the child's argv"
            );
            // …and not via the explicit positional spelling either.
            assert_eq!(resolve_robot_name_with(Some(hostile), HEX64, &m), None);
        }
        // ANTI-TAUTOLOGY: ordinary punctuation and non-ASCII are NOT control characters
        // and still resolve — the guard is the control class, not a name whitelist.
        let ok: BTreeMap<String, String> = [("naïve robot.2 🤖".to_string(), HEX64.to_string())]
            .into_iter()
            .collect();
        assert_eq!(
            resolve_robot_name_with(None, HEX64, &ok),
            Some("naïve robot.2 🤖".to_string())
        );

        // (b) CASE-FOLD collision: on a case-insensitive filesystem (macOS APFS default,
        //     Windows) `Go2.epoch` and `go2.epoch` are ONE file, so robot A's signed epoch
        //     would be read for robot B. Refused on every platform, so a config cannot
        //     work on Linux and mis-deliver on a Mac.
        let other_eid = "b".repeat(64);
        let case_dup: BTreeMap<String, String> = [
            ("go2".to_string(), HEX64.to_string()),
            ("GO2".to_string(), other_eid.clone()),
        ]
        .into_iter()
        .collect();
        assert_eq!(resolve_robot_name_with(None, HEX64, &case_dup), None);
        assert_eq!(
            resolve_robot_name_with(None, &other_eid, &case_dup),
            None,
            "BOTH sides of the collision are refused — neither can claim the shared file"
        );
        // An explicitly NAMED positional does not escape it either (the collision is
        // about which FILE the cache lives in, not how the name was chosen).
        assert_eq!(resolve_robot_name_with(Some("go2"), HEX64, &case_dup), None);

        // (c) SEPARATOR-FOLD collision: the shared file-name rule replaces `/` `\` `:`
        //     with `_`, so these two names are one file on EVERY filesystem.
        let sep_dup: BTreeMap<String, String> = [
            ("lab/go2".to_string(), HEX64.to_string()),
            ("lab:go2".to_string(), other_eid.clone()),
        ]
        .into_iter()
        .collect();
        assert_eq!(resolve_robot_name_with(None, HEX64, &sep_dup), None);
        // Hand oracle for WHY: both fold to the same cache file.
        assert_eq!(
            cerulion_pairing::verify::epoch_cache_file_name("lab/go2"),
            cerulion_pairing::verify::epoch_cache_file_name("lab:go2")
        );

        // (d) ANTI-TAUTOLOGY: two names for the SAME robot are the pre-existing AMBIGUITY
        //     case (also None), while two genuinely distinct names for distinct robots
        //     resolve normally — so the guard above is the COLLISION, not a blanket
        //     refusal of multi-robot pin files.
        let clean: BTreeMap<String, String> = [
            ("go2".to_string(), HEX64.to_string()),
            ("orin".to_string(), other_eid.clone()),
        ]
        .into_iter()
        .collect();
        assert_eq!(
            resolve_robot_name_with(None, HEX64, &clean),
            Some("go2".to_string())
        );
        assert_eq!(
            resolve_robot_name_with(None, &other_eid, &clean),
            Some("orin".to_string())
        );
    }

    #[test]
    fn build_connectd_argv_full_oracle() {
        let args = ConnectArgs {
            robot: None,
            eid: None,
            addrs: vec!["192.168.1.20:7842".into(), "[::1]:9000".into()],
            topics: vec!["/imu".into(), "/tf".into()],
            all: false,
            key_file: Some(PathBuf::from("/keys/desk.key")),
            schemas_dir: Some(PathBuf::from("/ws/schemas")),
            relay_url: Some("https://relay.example".into()),
            relay_disabled: false,
            network: Some("off".into()),
        };
        let argv = build_connectd_argv(HEX64, Some("go2"), &args);
        assert_eq!(
            argv,
            vec![
                "connect".to_string(),
                "--eid".into(),
                HEX64.to_string(),
                "--robot-name".into(),
                "go2".into(),
                "--addr".into(),
                "192.168.1.20:7842".into(),
                "--addr".into(),
                "[::1]:9000".into(),
                "--topic".into(),
                "/imu".into(),
                "--topic".into(),
                "/tf".into(),
                "--key-file".into(),
                "/keys/desk.key".into(),
                "--schemas-dir".into(),
                "/ws/schemas".into(),
                "--relay-url".into(),
                "https://relay.example".into(),
                "--network".into(),
                "off".into(),
            ]
        );
    }

    #[test]
    fn build_connectd_argv_minimal_and_all() {
        // Zero topics + no --all: the discoverable catalog-only default (no
        // --topic, no --all in the argv).
        let args = ConnectArgs {
            eid: Some(HEX64.into()),
            ..Default::default()
        };
        // No desk-verified name ⇒ NO `--robot-name` (the session then pushes no
        // epoch rather than keying the cache off the peer's self-report).
        assert_eq!(
            build_connectd_argv(HEX64, None, &args),
            vec!["connect".to_string(), "--eid".into(), HEX64.to_string()]
        );
        // --all + --relay-disabled.
        let args = ConnectArgs {
            all: true,
            relay_disabled: true,
            ..Default::default()
        };
        assert_eq!(
            build_connectd_argv(HEX64, Some("go2"), &args),
            vec![
                "connect".to_string(),
                "--eid".into(),
                HEX64.to_string(),
                "--robot-name".into(),
                "go2".into(),
                "--all".into(),
                "--relay-disabled".into(),
            ]
        );
    }

    /// f4: `spawn_and_wait` forwards the child's natural exit code (no signal).
    #[cfg(unix)]
    #[test]
    fn spawn_and_wait_forwards_child_exit_code() {
        let plan = ConnectPlan {
            bin: PathBuf::from("sh"),
            argv: vec!["-c".to_string(), "exit 7".to_string()],
        };
        let running = Arc::new(AtomicBool::new(true));
        assert_eq!(spawn_and_wait(&plan, running).unwrap(), 7);
    }

    /// f4 HEADLINE: a directed signal (the `running` flag
    /// flips, as `setup_ctrlc_handler` does on SIGINT/SIGTERM/SIGHUP) makes
    /// `spawn_and_wait` FORWARD SIGINT to the child + REAP it PROMPTLY — it must NOT
    /// block for the child's full lifetime. The child is a 30s `sleep`; a hard 10s
    /// bound catches a regression to the old `.status()` (which would block the
    /// full 30s and the recv_timeout would fire).
    #[cfg(unix)]
    #[test]
    fn spawn_and_wait_forwards_signal_and_reaps_without_hanging() {
        let plan = ConnectPlan {
            bin: PathBuf::from("sleep"),
            argv: vec!["30".to_string()],
        };
        let running = Arc::new(AtomicBool::new(true));
        let running_flip = running.clone();
        // Flip the flag shortly after spawn (a directed SIGTERM to `cerulion` only).
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(200));
            running_flip.store(false, Ordering::SeqCst);
        });
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(spawn_and_wait(&plan, running));
        });
        let result = rx.recv_timeout(Duration::from_secs(10)).expect(
            "spawn_and_wait must return PROMPTLY after the signal — not block for the child's full \
             30s (f4 regression: reverting the forward/poll to a blocking .status() times out here)",
        );
        // The forwarded SIGINT KILLS the un-handled `sleep` child → 128 + SIGINT(2)
        // = 130 (a signal-killed child NEVER reads as a clean 0).
        assert_eq!(
            result.unwrap(),
            130,
            "a signal-killed child maps to 128 + SIGINT(2) = 130, never a synthesized 0"
        );
    }

    /// Signal-mapping oracle: a child that dies by a signal maps to
    /// `128 + signo` (SIGINT → 130, SIGKILL → 137), NEVER a synthesized 0 — so a
    /// keystroke-killed connectd never reads as a clean success (the false-pairing
    /// guard `pair_cmd::finalize_pairing` relies on).
    #[cfg(unix)]
    #[test]
    fn spawn_and_wait_signal_killed_child_maps_to_128_plus_signo() {
        let sh = |script: &str| ConnectPlan {
            bin: PathBuf::from("sh"),
            argv: vec!["-c".to_string(), script.to_string()],
        };
        assert_eq!(
            spawn_and_wait(&sh("kill -INT $$"), Arc::new(AtomicBool::new(true))).unwrap(),
            130,
            "self-SIGINT → 128 + 2 = 130"
        );
        assert_eq!(
            spawn_and_wait(&sh("kill -KILL $$"), Arc::new(AtomicBool::new(true))).unwrap(),
            137,
            "self-SIGKILL → 128 + 9 = 137"
        );
        // A clean exit still forwards its real code (anti-tautology).
        assert_eq!(
            spawn_and_wait(&sh("exit 0"), Arc::new(AtomicBool::new(true))).unwrap(),
            0,
            "a genuine clean exit is still 0"
        );
    }

    /// The DETERMINISTIC reproduction of the
    /// gate's detached-context flake, on ANY box — no `setsid` / no controlling
    /// terminal needed. A no-TTY / backgrounded / service-manager launcher leaves
    /// SIGINT at SIG_IGN in this process; a naively-spawned child INHERITS that
    /// disposition (SIG_IGN SURVIVES `execve`, POSIX), so a `sleep` that installs no
    /// handler would IGNORE the forwarded shutdown SIGINT, the grace would expire,
    /// and the SIGKILL backstop would map to 137 — NOT the intended graceful SIGINT
    /// (130). The `pre_exec` SIG_DFL reset in `spawn_and_wait` makes the forwarded
    /// SIGINT REACH the child.
    ///
    /// This flips the whole thing to reproduce that context HERE: set SIGINT to
    /// SIG_IGN process-wide (restored by the RAII guard), spawn, forward, and assert
    /// the child STILL dies by SIGINT → 130. Deleting the `pre_exec`
    /// reset yields 137 (the exact gate failure). `#[serial]` because the SIGINT
    /// disposition is process-global.
    #[cfg(unix)]
    #[serial_test::serial]
    #[test]
    fn spawn_and_wait_forwards_sigint_even_when_parent_ignores_it() {
        // Restore the prior SIGINT disposition on drop (panic-safe), so no other
        // test in this binary inherits the simulated "ignored" state.
        struct SigintDispositionGuard(libc::sighandler_t);
        impl Drop for SigintDispositionGuard {
            fn drop(&mut self) {
                // SAFETY: restore the exact disposition captured below.
                unsafe {
                    libc::signal(libc::SIGINT, self.0);
                }
            }
        }
        // Simulate the detached/no-TTY launcher: SIGINT ignored process-wide.
        // SAFETY: a single disposition change, undone by the guard.
        let _guard = SigintDispositionGuard(unsafe { libc::signal(libc::SIGINT, libc::SIG_IGN) });

        let plan = ConnectPlan {
            bin: PathBuf::from("sleep"),
            argv: vec!["30".to_string()],
        };
        let running = Arc::new(AtomicBool::new(true));
        let running_flip = running.clone();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(200));
            running_flip.store(false, Ordering::SeqCst);
        });
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(spawn_and_wait(&plan, running));
        });
        // Bounded: the reset makes SIGINT kill `sleep` immediately, so the forward
        // returns promptly. Without the reset the child ignores SIGINT
        // and the 5s grace → SIGKILL still returns within this bound as 137 (a FAIL,
        // not a hang).
        let result = rx.recv_timeout(Duration::from_secs(10)).expect(
            "spawn_and_wait must return after the forwarded SIGINT reaps the child (or the grace \
             backstop fires) — never hang",
        );
        assert_eq!(
            result.unwrap(),
            130,
            "With SIGINT IGNORED in the parent (the detached-launcher context), the \
             pre_exec SIG_DFL reset must still let the forwarded SIGINT KILL the child → 130; \
             removing the reset regresses to 137 (the exact gate flake)"
        );
    }

    #[test]
    fn resolve_connectd_bin_oracle() {
        let exe_dir = Path::new("/opt/cerulion/bin");
        // Env override that exists.
        let over = PathBuf::from("/custom/cerulion-connectd");
        let exists_over = |p: &Path| p == over.as_path();
        assert_eq!(
            resolve_connectd_bin_from_dirs(Some(over.clone()), exe_dir, None, &exists_over)
                .unwrap(),
            over
        );
        // Env override that does NOT exist → loud error.
        let none_exists = |_: &Path| false;
        let e = resolve_connectd_bin_from_dirs(Some(over.clone()), exe_dir, None, &none_exists)
            .unwrap_err();
        assert!(e.to_string().contains("CERULION_CONNECTD_BIN"), "err: {e}");
        assert!(e.to_string().contains("does not exist"), "err: {e}");
        // No override, sibling exists.
        let sibling = exe_dir.join(connectd_bin_name());
        let sib = sibling.clone();
        let exists_sib = move |p: &Path| p == sib.as_path();
        assert_eq!(
            resolve_connectd_bin_from_dirs(None, exe_dir, None, &exists_sib).unwrap(),
            sibling
        );
        // No override, sibling missing → error naming the build command.
        let e = resolve_connectd_bin_from_dirs(None, exe_dir, None, &none_exists).unwrap_err();
        assert!(
            e.to_string().contains("cargo build -p cerulion_connectd"),
            "err names the build command: {e}"
        );
        assert!(e.to_string().contains("iroh"), "err explains why: {e}");
    }

    /// A symlinked `cerulion` finds connectd beside its REAL self.
    ///
    /// `std::env::current_exe()` is not canonicalised on macOS (measured), so a
    /// two-rung ladder that looks only in the symlink's directory hard-errors
    /// naming a directory nobody had reason to suspect — the same class as the
    /// netd/vizd launch sites, on two live verbs (`connect`, `pair`).
    ///
    /// Dropping `resolved_dir` from the rung list fails this test.
    #[test]
    fn a_symlinked_cli_finds_connectd_beside_its_resolved_binary() {
        let link_dir = Path::new("/w/somewhere/bin");
        let real_dir = Path::new("/w/cerulion/target/release");
        let real = real_dir.join(connectd_bin_name());

        // ONLY the real checkout holds the daemon.
        let r = real.clone();
        let only_real = move |p: &Path| p == r.as_path();
        assert_eq!(
            resolve_connectd_bin_from_dirs(None, link_dir, Some(real_dir), &only_real).unwrap(),
            real,
            "a cerulion reached through a symlink must find connectd beside its real self"
        );

        // ANTI-TAUTOLOGY: rung 1 still WINS when it holds a daemon, so rung 2 is a
        // fallback and not a blanket redirection.
        let link_sibling = link_dir.join(connectd_bin_name());
        let both: Vec<PathBuf> = vec![link_sibling.clone(), real.clone()];
        let exists_both = move |p: &Path| both.iter().any(|q| q == p);
        assert_eq!(
            resolve_connectd_bin_from_dirs(None, link_dir, Some(real_dir), &exists_both).unwrap(),
            link_sibling,
            "the invoked directory outranks the resolved one when it holds a daemon"
        );

        // With NEITHER present the failure names BOTH directories, numbered — the
        // original complaint was being told about one unsuspected directory.
        let none_exists = |_: &Path| false;
        let e = resolve_connectd_bin_from_dirs(None, link_dir, Some(real_dir), &none_exists)
            .unwrap_err()
            .to_string();
        assert!(e.contains("(1) /w/somewhere/bin/"), "names rung 1: {e}");
        assert!(
            e.contains("(2) /w/cerulion/target/release/"),
            "names rung 2: {e}"
        );
        assert!(
            e.contains("cargo build -p cerulion_connectd") && e.contains("CERULION_CONNECTD_BIN"),
            "keeps both remedies: {e}"
        );

        // An ALREADY-RESOLVED exe (the Linux shape) contributes no duplicate rung.
        let e = resolve_connectd_bin_from_dirs(None, real_dir, Some(real_dir), &none_exists)
            .unwrap_err()
            .to_string();
        assert!(e.contains("(1) "), "one rung is still numbered: {e}");
        assert!(
            !e.contains("(2) "),
            "an already-resolved exe must not report the same directory twice: {e}"
        );

        // The ENV rung is DELIBERATELY unchanged: connectd requires its override to
        // exist (netd takes its verbatim). Re-asserted here so a future "harmonise
        // the ladders" edit has to confront the divergence rather than assume it.
        let over = PathBuf::from("/custom/cerulion-connectd");
        let e = resolve_connectd_bin_from_dirs(Some(over), link_dir, Some(real_dir), &none_exists)
            .unwrap_err()
            .to_string();
        assert!(
            e.contains("CERULION_CONNECTD_BIN") && e.contains("does not exist"),
            "a missing override still refuses up front, and does NOT fall through to \
             the sibling rungs: {e}"
        );
    }

    /// Serializes the env-mutating tests in this module against EVERY other
    /// env-mutating lib test in the crate — see [`crate::test_env`]. This module's
    /// `HOME`-mutating `plan` pin and `account_cmd`'s `HOME`-mutating cache-path pin
    /// once held two DIFFERENT private mutexes and raced 12/12 under
    /// `--test-threads=2`.
    fn env_lock() -> std::sync::MutexGuard<'static, ()> {
        crate::test_env::env_lock()
    }

    /// Snapshots the named env vars and RESTORES them on drop (panic-safe).
    struct EnvSnapshot(Vec<(&'static str, Option<String>)>);
    impl EnvSnapshot {
        fn take(keys: &[&'static str]) -> Self {
            EnvSnapshot(keys.iter().map(|k| (*k, std::env::var(k).ok())).collect())
        }
    }
    impl Drop for EnvSnapshot {
        fn drop(&mut self) {
            for (k, v) in &self.0 {
                match v {
                    Some(v) => std::env::set_var(k, v),
                    None => std::env::remove_var(k),
                }
            }
        }
    }

    /// [`plan`] (where the epoch-cache keying decision is actually
    /// MADE and handed to the sibling binary) is pinned end to end, not just its
    /// injected-map helper. `plan` reads the REAL `~/.cerulion/robots.toml` (via a
    /// tempdir `HOME`) and resolves the REAL binary (via `CERULION_CONNECTD_BIN`), so the
    /// composition `resolve_eid` → `resolve_robot_name` → `build_connectd_argv` is
    /// exercised exactly as the CLI runs it.
    ///
    /// The security-relevant assertion is the NEGATIVE one: a dial of a robot this desk
    /// has NO record for must emit NO `--robot-name`, so the session cannot fall back to
    /// the robot's self-reported identity to select a cached epoch.
    ///
    /// Env-mutating (`HOME` + `CERULION_CONNECTD_BIN`) → serialized on [`env_lock`].
    #[test]
    fn plan_keys_the_epoch_cache_by_the_desk_verified_name_or_by_nothing() {
        let _lock = env_lock();
        let _snap = EnvSnapshot::take(&["HOME", "CERULION_CONNECTD_BIN"]);

        let home = tempfile::tempdir().unwrap();
        std::env::set_var("HOME", home.path());
        // A real (empty) file standing in for the sibling binary, so `resolve_connectd_bin`
        // succeeds without a build.
        let bin = home.path().join("cerulion-connectd-stub");
        std::fs::write(&bin, b"").unwrap();
        std::env::set_var("CERULION_CONNECTD_BIN", &bin);

        let other_eid = "b".repeat(64);
        std::fs::create_dir_all(home.path().join(".cerulion")).unwrap();
        std::fs::write(
            home.path().join(".cerulion").join("robots.toml"),
            format!("[robots]\ngo2 = \"{HEX64}\"\norin = \"{other_eid}\"\n"),
        )
        .unwrap();

        let name_flag = |argv: &[String]| -> Option<String> {
            argv.iter()
                .position(|a| a == "--robot-name")
                .and_then(|i| argv.get(i + 1).cloned())
        };

        // (a) A NAMED dial: the desk's own name keys the cache.
        let p = plan(&ConnectArgs {
            robot: Some("go2".into()),
            ..Default::default()
        })
        .unwrap();
        assert_eq!(p.bin, bin);
        assert_eq!(name_flag(&p.argv).as_deref(), Some("go2"));
        assert_eq!(p.argv[..3], ["connect", "--eid", HEX64]);

        // (b) A RAW-EID dial of a PINNED robot: the reverse lookup supplies the name.
        let p = plan(&ConnectArgs {
            eid: Some(other_eid.clone()),
            ..Default::default()
        })
        .unwrap();
        assert_eq!(name_flag(&p.argv).as_deref(), Some("orin"));

        // (c) THE SECURITY PIN — a raw-eid dial of a robot this desk has NO record for
        //     emits NO `--robot-name`, so the session has no name to key a cache by and
        //     can never fall back to the robot's self-report.
        let p = plan(&ConnectArgs {
            eid: Some("c".repeat(64)),
            ..Default::default()
        })
        .unwrap();
        assert_eq!(name_flag(&p.argv), None);
        assert!(!p.argv.iter().any(|a| a == "--robot-name"));

        // (d) A pin file whose two names COLLIDE onto one epoch-cache file also yields no
        //     `--robot-name`: better no push than the wrong robot's epoch.
        std::fs::write(
            home.path().join(".cerulion").join("robots.toml"),
            format!("[robots]\ngo2 = \"{HEX64}\"\nGO2 = \"{other_eid}\"\n"),
        )
        .unwrap();
        let p = plan(&ConnectArgs {
            robot: Some("go2".into()),
            ..Default::default()
        })
        .unwrap();
        assert_eq!(
            name_flag(&p.argv),
            None,
            "a colliding cache key is not used"
        );
        // …and the dial itself still proceeds (the epoch push is expendable, the dial is
        // not): the eid resolved and the argv is otherwise intact.
        assert_eq!(p.argv[..3], ["connect", "--eid", HEX64]);
    }
}

#[cfg(test)]
mod debug_redaction_tests {
    use super::*;
    #[test]
    fn connect_plan_debug_never_prints_pairing_argv() {
        let plan = ConnectPlan {
            bin: PathBuf::from("connectd"),
            argv: vec!["pair".into(), "--code".into(), "PAIR-SENSITIVE-71".into()],
        };
        assert_eq!(
            format!("{plan:?}"),
            "ConnectPlan { bin: \"connectd\", argv: \"[REDACTED]\" }"
        );
        let pretty = format!("{plan:#?}");
        assert!(pretty.contains("[REDACTED]"));
        assert!(!pretty.contains("PAIR-SENSITIVE-71"));
    }
}
