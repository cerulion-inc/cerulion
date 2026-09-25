// SPDX-License-Identifier: AGPL-3.0-only
//! The CLI's LOCAL account state (`~/.cerulion/auth.json`) + the
//! runtime login gate.
//!
//! This module is the **network-free** half of login: it owns the on-disk auth
//! state and the pure gate classifier that every command consults. The
//! HTTP-touching device-code login FLOW lives in [`crate::login_cmd`]; this
//! module never dials the account service.
//!
//! ## The state ([`AuthState`], `~/.cerulion/auth.json`)
//!
//! ```json
//! { "account_id": "<base64url>", "session_token": "…", "refresh_token": "…",
//!   "expires_at_ns": 0, "logged_in_ever": true }
//! ```
//!
//! `logged_in_ever` is the **durable never-bricks marker**: once a
//! machine has EVER logged in, it operates locally + on the LAN
//! forever, even offline with an expired session — there is **no offline
//! horizon**. The file is written **atomically** (temp + rename) and **chmod
//! 600** (it holds bearer secrets).
//!
//! ## The gate ([`local_gate`] / [`wan_gate`])
//!
//! | Situation | Gate |
//! |---|---|
//! | never logged in (`logged_in_ever == false` / no file) | **refuse** — auto-trigger login. The ONLY hard local block; fires ONCE. |
//! | logged-in-ever, session valid | proceed (local, zero network) |
//! | logged-in-ever, session EXPIRED, offline | **proceed** for local + LAN + established-remote (no horizon) |
//! | logged-in-ever, WAN / relay / account-service call | **server-side** gate — refuses on an expired session until refresh |
//!
//! [`local_gate`] answers the first three rows (a pure, zero-network,
//! microsecond-cheap `stat` + parse); [`wan_gate`] answers the fourth (the
//! cloud call must refresh a stale session first — enforced server-side for
//! real, this is only the client-side hint).
//!
//! ## Config-dir isolation
//!
//! The cerulion dir resolves from `CERULION_HOME` (used verbatim as the config
//! dir) when set, else `dirs::home_dir()/.cerulion`. Tests and CI jobs seed a
//! valid [`AuthState`] into an isolated `CERULION_HOME` via [`seed_logged_in_at`]
//! (or `tools/ci/seed_test_login.sh` for a shell harness) so the gate reads
//! provisioned state rather than a real login. That is the sanctioned shape and
//! the only one: there is no bypass tier, no way to turn the gate off, and the
//! gate always reads LOCAL state. Provisioning that state is not a bypass, and
//! the seam is not a secret: `auth.json` is a plain file, and the shell script
//! writes the same bytes with no binary at all.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};

/// This machine's install-determined role: a **Studio** install
/// (website download →
/// first-run login) is a `Desk`; a **standalone CLI** install (install-time login)
/// is a `Robot` ("any computer with the standalone CLI is a robot"). Persisted on
/// [`AuthState`] so the ONE binary picks its role BY CONFIG (the rule: "one
/// binary, role by config, nothing desk-specific baked in") rather than re-deriving
/// it every run.
///
/// **The WRITER is the install funnel — not implemented here (future work).**
/// [`register_robot`](crate::robot_cmd::register_robot) (the `POST /v1/robots`
/// install-funnel entry) is deliberately DECOUPLED from the login flow (see its
/// module docs: "a plain login is a DESK … the trigger that distinguishes a robot
/// install from a desk install (an install-time `role`) is an install-funnel
/// concern"), so NO production path stamps a role yet. This enum + the
/// [`AuthState::role`] field are the forward/back-compatible groundwork the funnel
/// writes into: the field round-trips through `auth.json`, [`run_login`](crate::login_cmd::run_login)
/// / [`refresh_session_if_stale`](crate::login_cmd::refresh_session_if_stale) carry
/// a set role forward non-destructively across re-logins, and [`AuthState::machine_role`]
/// resolves an unmarked machine to [`MachineRole::Desk`] (the documented default).
///
/// Serialized lowercase (`"desk"` / `"robot"`) — the exact shape a newer install
/// funnel writes and that the forward-compat parse pins.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MachineRole {
    /// A **desk**: a Studio install, OR the resolved default for any machine that
    /// never registered itself as a robot. Consumes remote robots' topics; owns no
    /// robot row (login_cmd: "a desk (Studio ⊃ CLI) signs in but owns no robot").
    Desk,
    /// A **robot**: a standalone CLI install — the data source other desks consume.
    Robot,
}

/// The durable local account state persisted at `~/.cerulion/auth.json`.
///
/// Holds bearer secrets — always written 0600 via [`write_to`].
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthState {
    /// The cloud `AccountId`, `base64url`-encoded (matches `/v1/me`'s
    /// `account_id`). Opaque to the CLI — it identifies the logged-in account.
    pub account_id: String,
    /// The opaque session token (the WAN-gate credential; short-lived).
    pub session_token: String,
    /// The opaque refresh token (rotates the session while online).
    pub refresh_token: String,
    /// Session expiry, Unix nanoseconds. `now >= expires_at_ns` ⇒ the session is
    /// stale (local ops still proceed; cloud ops must refresh first).
    pub expires_at_ns: u64,
    /// The never-bricks marker: set `true` the first time this machine ever
    /// logs in and NEVER cleared. Its presence is the local-capability gate.
    pub logged_in_ever: bool,
    /// This machine's install-determined [`MachineRole`] (— "role by
    /// config"). `None` = **unmarked**: a legacy `auth.json` written before this
    /// field existed, a machine whose install never stamped a role, OR a role VALUE
    /// this binary does not recognize (see below). An unmarked machine resolves to
    /// [`MachineRole::Desk`] via [`Self::machine_role`] (the documented default — "a
    /// plain login is a DESK").
    ///
    /// **Never-bricks — unknown KEYS *and* unknown VALUES are tolerated.**
    /// `#[serde(default)]` keeps a file with the `role` key ABSENT parseable, and (no
    /// `deny_unknown_fields`) a file with EXTRA keys parseable. Critically,
    /// `deserialize_tolerant_role` maps an UNRECOGNIZED `role` value (a future
    /// `"operator"`, a foreign string, or any non-string shape a newer binary wrote)
    /// to `None` + a `tracing::warn!` at parse time — instead of failing the WHOLE
    /// `AuthState` parse, which would surface as [`LoadedAuth::Corrupt`] → a
    /// never-logged-in gate refusal → a BRICKED login. `skip_serializing_if` keeps an
    /// unmarked file BYTE-IDENTICAL to the pre-field shape.
    ///
    /// **Trade-off (documented):** an unrecognized value is NOT preserved — it reads
    /// as `None`, so the next [`run_login`](crate::login_cmd::run_login) /
    /// [`refresh_session_if_stale`](crate::login_cmd::refresh_session_if_stale)
    /// carry-forward writes it back OMITTED (the foreign value is DROPPED). Chosen
    /// deliberately over a `#[serde(other)] Unknown` variant so the write side never
    /// clobbers a future role string with a placeholder like `"unknown"` — an unmarked
    /// file stays clean. The WRITER (the install funnel) is future work — see
    /// [`MachineRole`].
    #[serde(
        default,
        deserialize_with = "deserialize_tolerant_role",
        skip_serializing_if = "Option::is_none"
    )]
    pub role: Option<MachineRole>,
}

impl std::fmt::Debug for AuthState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AuthState")
            .field("account_id", &self.account_id)
            .field("session_token", &"[REDACTED]")
            .field("refresh_token", &"[REDACTED]")
            .field("expires_at_ns", &self.expires_at_ns)
            .field("logged_in_ever", &self.logged_in_ever)
            .field("role", &self.role)
            .finish()
    }
}

impl AuthState {
    /// Whether the session token is still fresh at `now_ns` (a cloud call may use
    /// it without refreshing).
    pub fn session_is_valid(&self, now_ns: u64) -> bool {
        now_ns < self.expires_at_ns
    }

    /// This machine's RESOLVED [`MachineRole`]: the stamped [`Self::role`], or
    /// [`MachineRole::Desk`] when unmarked (`None`) — the documented default ("a
    /// plain login is a DESK"; see [`register_robot`](crate::robot_cmd::register_robot)'s
    /// module docs). The role-by-config seam a C6-body consumer (netd role selection)
    /// reads, so an unmarked / legacy machine behaves as a desk rather than crashing
    /// or re-deriving the role every run.
    pub fn machine_role(&self) -> MachineRole {
        self.role.unwrap_or(MachineRole::Desk)
    }
}

/// Tolerant deserializer for [`AuthState::role`]: map an UNRECOGNIZED role value (a
/// future/foreign string, or any non-string JSON shape) to `None` + a loud
/// `tracing::warn!`, instead of failing the whole [`AuthState`] parse. This is the
/// never-bricks contract for unknown VALUES — the sibling of the no-`deny_unknown_fields`
/// tolerance for unknown KEYS: a newer binary that stamps `role: "operator"` must NOT
/// brick an older binary's login (a parse failure surfaces as [`LoadedAuth::Corrupt`]
/// → a never-logged-in gate refusal — see [`local_gate`]).
///
/// The loudness lives HERE because this is the ONLY seam holding the foreign value:
/// [`load_from`] / [`load`] only see the final `AuthState`, where a tolerated value is
/// Present with `role: None` — indistinguishable after the fact from a genuinely absent
/// role. The unrecognized value is DROPPED, not preserved (see [`AuthState::role`] for
/// the trade-off). `auth.json` is always serde_json, so matching against
/// [`serde_json::Value`] tolerates every JSON shape (string / number / object / …), not
/// only unrecognized strings — a non-string role never bricks either. Recognized values
/// stay in lockstep with [`MachineRole`]'s `rename_all = "lowercase"` serialization
/// (drift-guarded by `role_round_trips_all_states_and_unmarked_is_omitted`).
fn deserialize_tolerant_role<'de, D>(deserializer: D) -> Result<Option<MachineRole>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    // `#[serde(default)]` handles a MISSING key (this fn is not called); a PRESENT key
    // (incl. explicit null) lands here. Decode to a generic JSON value so any shape is
    // tolerated rather than erroring the parse.
    let raw = Option::<serde_json::Value>::deserialize(deserializer)?;
    Ok(match raw {
        None | Some(serde_json::Value::Null) => None,
        Some(serde_json::Value::String(ref s)) if s == "desk" => Some(MachineRole::Desk),
        Some(serde_json::Value::String(ref s)) if s == "robot" => Some(MachineRole::Robot),
        Some(other) => {
            tracing::warn!(
                role = %other,
                "~/.cerulion/auth.json has an unrecognized `role` value — treating this \
                 machine as unmarked (it resolves to Desk). A re-login DROPS the foreign \
                 value (it is not preserved). The machine is NOT bricked."
            );
            None
        }
    })
}

/// The outcome of reading `auth.json`.
#[derive(Debug)]
pub enum LoadedAuth {
    /// A parseable state was read.
    Present(AuthState),
    /// No `auth.json` exists (never logged in on this machine).
    Absent,
    /// `auth.json` exists but could not be parsed. Carries the parse error for a
    /// LOUD log at the boundary. Treated as never-logged-in by the gate
    /// (`corrupt ⇒ re-login`) — NEVER deleted here (a fresh login overwrites it
    /// atomically), NEVER a crash.
    Corrupt(String),
}

impl LoadedAuth {
    /// Borrow the parsed state when present (Absent/Corrupt ⇒ `None`).
    pub fn state(&self) -> Option<&AuthState> {
        match self {
            LoadedAuth::Present(s) => Some(s),
            LoadedAuth::Absent | LoadedAuth::Corrupt(_) => None,
        }
    }
}

/// The local-capability gate decision. Pure over the loaded
/// state + the current time — zero network, microsecond-cheap.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalGate {
    /// Row 1: never logged in (no file, corrupt, or `logged_in_ever ==
    /// false`). The ONLY hard local block — the caller refuses + auto-triggers
    /// the device-code login. Fires ONCE (the next run reads the written state).
    RefuseNeverLoggedIn,
    /// Row 2: logged-in-ever with a still-valid session. Proceed, zero network.
    ProceedValidSession,
    /// Row 3: logged-in-ever with an EXPIRED session. Proceed anyway for
    /// local + LAN + established-remote operation — no offline horizon. Only NEW
    /// cross-machine pairings (needing a fresh cert) defer to the next refresh.
    ProceedExpiredLocalForever,
}

impl LocalGate {
    /// Whether the command may proceed on the LOCAL/LAN plane.
    pub fn may_proceed(self) -> bool {
        matches!(
            self,
            LocalGate::ProceedValidSession | LocalGate::ProceedExpiredLocalForever
        )
    }
}

/// The WAN / relay / account-service gate decision. The cloud
/// enforces this server-side for real; this is the client-side hint that a
/// stale session must be refreshed before a cloud call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WanGate {
    /// The session is valid — a cloud call may use it directly.
    Allow,
    /// The session is expired — refresh (`/v1/auth/refresh`) before the cloud
    /// call, or the server refuses it. Local operation is unaffected.
    RefreshRequired,
}

/// Classify the LOCAL gate from the loaded state + the current time.
/// Pure — no I/O, no network.
pub fn local_gate(loaded: &LoadedAuth, now_ns: u64) -> LocalGate {
    match loaded.state() {
        // Absent OR corrupt OR an explicit `logged_in_ever == false` ⇒ never
        // logged in. Corrupt state is deliberately folded here (corrupt ⇒
        // re-login) — the LOUD warn is emitted where the file is read.
        None => LocalGate::RefuseNeverLoggedIn,
        Some(state) if !state.logged_in_ever => LocalGate::RefuseNeverLoggedIn,
        Some(state) if state.session_is_valid(now_ns) => LocalGate::ProceedValidSession,
        // logged-in-ever + expired ⇒ proceed locally FOREVER (no horizon).
        Some(_) => LocalGate::ProceedExpiredLocalForever,
    }
}

/// Classify the WAN gate from a present state + the current time.
/// Only meaningful for a logged-in-ever account (a never-logged-in machine
/// has no session to present).
pub fn wan_gate(state: &AuthState, now_ns: u64) -> WanGate {
    if state.session_is_valid(now_ns) {
        WanGate::Allow
    } else {
        WanGate::RefreshRequired
    }
}

// ===========================================================================
// Path resolution
// ===========================================================================

/// The Cerulion config directory: `CERULION_HOME` verbatim when set (the test /
/// deployment isolation knob), else `~/.cerulion`. `None` only when there is no
/// home directory AND no override.
pub fn cerulion_config_dir() -> Option<PathBuf> {
    if let Some(home) = std::env::var_os("CERULION_HOME") {
        if !home.is_empty() {
            return Some(PathBuf::from(home));
        }
    }
    Some(dirs::home_dir()?.join(".cerulion"))
}

/// `~/.cerulion/auth.json` (env-aware; `None` when no config dir resolves).
pub fn auth_json_path() -> Option<PathBuf> {
    Some(cerulion_config_dir()?.join("auth.json"))
}

/// `~/.cerulion/device.cert` — the cached `SignedDeviceCert` (base64url blob).
pub fn device_cert_path() -> Option<PathBuf> {
    Some(cerulion_config_dir()?.join("device.cert"))
}

/// `~/.cerulion/desk.key` — the 32-byte ed25519 device seed. Shared with the
/// `cerulion connect`/`pair` desk identity (Studio ⊃ CLI: one device key per
/// machine).
pub fn device_key_path() -> Option<PathBuf> {
    Some(cerulion_config_dir()?.join("desk.key"))
}

/// The current Unix time in nanoseconds (saturating; monotone-enough for TTL
/// comparisons).
pub fn now_unix_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

// ===========================================================================
// Read / write
// ===========================================================================

/// Read + parse `auth.json` from an explicit path. A missing file ⇒ [`Absent`];
/// a present-but-unparseable file ⇒ [`Corrupt`] (never a crash, never a delete).
///
/// [`Absent`]: LoadedAuth::Absent
/// [`Corrupt`]: LoadedAuth::Corrupt
pub fn load_from(path: &Path) -> LoadedAuth {
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return LoadedAuth::Absent,
        Err(e) => return LoadedAuth::Corrupt(format!("read {}: {e}", path.display())),
    };
    match serde_json::from_slice::<AuthState>(&bytes) {
        Ok(state) => LoadedAuth::Present(state),
        Err(e) => LoadedAuth::Corrupt(format!("parse {}: {e}", path.display())),
    }
}

/// Read the env-resolved `auth.json`, emitting a LOUD `warn!` on a corrupt file
/// (the "corrupt ⇒ re-login" contract — the corruption is surfaced, the file is
/// left in place for a fresh login to overwrite atomically). No config dir ⇒
/// [`Absent`] (treated as never-logged-in).
///
/// [`Absent`]: LoadedAuth::Absent
pub fn load() -> LoadedAuth {
    let Some(path) = auth_json_path() else {
        return LoadedAuth::Absent;
    };
    let loaded = load_from(&path);
    if let LoadedAuth::Corrupt(reason) = &loaded {
        tracing::warn!(
            path = %path.display(),
            reason = %reason,
            "~/.cerulion/auth.json is corrupt — treating this machine as never-logged-in \
             (a fresh `cerulion login` will overwrite it). The file is NOT deleted."
        );
    }
    loaded
}

/// Write `auth.json` to an explicit path **atomically** and **owner-only**
/// (0600 file inside a 0700 dir on Unix; unique-per-process staging temp, never
/// following a symlink — see `atomic_write_secret`), holding the store lock
/// ([`with_store_lock`]) for the write.
///
/// A caller that READS the store and then writes a value derived from what it
/// read (`login`'s carry-the-role, the stale-session refresh) must wrap BOTH in
/// one [`with_store_lock`] instead of relying on this: the lock here serializes
/// the publication, not the read-modify-write around it.
pub fn write_to(path: &Path, state: &AuthState) -> std::io::Result<()> {
    let json = serde_json::to_vec_pretty(state)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    with_store_lock(path, || atomic_write_secret(path, &json))
}

/// Write the cert to a hidden staging sibling of `path` WITHOUT publishing it, so
/// the caller learns whether the path is writable before it commits anything a
/// consumer can see. [`StagedCert::commit`] is the publication.
///
/// This is what makes a multi-consumer cache update all-or-nothing: a switch
/// caches the cert at every path it emptied, and staging every one of them before
/// `auth.json` is published means the store is only published when all of them CAN
/// be, leaving the commits as renames within a directory that has already accepted
/// a create + fsync.
///
/// Call it holding the store lock ([`with_store_lock`]): stage and commit inside the
/// same critical section that publishes the `auth.json` the cert belongs to. The cert
/// and the store are ONE binding, and a cert cached under a lock of its own names an
/// account the store has not been told about yet.
pub fn stage_device_cert_at(path: &Path, cert_b64: &str) -> std::io::Result<StagedCert> {
    stage_secret(path, cert_b64.as_bytes())
}

/// Env var: `cerulion-netd`'s explicit device-cert path — a copy of that crate's
/// `wan::DEVICE_CERT_ENV`, because the CLI links netd with `default-features = false`
/// (the iroh-leanness rule) and the const lives behind its `wan` feature. Pinned to
/// netd's own source by `netd_device_cert_env_matches_netds_const`.
pub const NETD_DEVICE_CERT_ENV: &str = "CERULION_NETD_DEVICE_CERT";

/// Env var: `cerulion-netd`'s explicit desk-key path (its `wan::DESK_KEY_ENV`), whose
/// SIBLING `device.cert` netd reads when [`NETD_DEVICE_CERT_ENV`] is unset. Same
/// duplication, same pin.
pub const NETD_DESK_KEY_ENV: &str = "CERULION_NETD_DESK_KEY";

/// Every cert file an account switch on this machine invalidates: `~/.cerulion/
/// device.cert`, plus the paths `cerulion-netd` resolves when its env overrides move
/// them (explicit cert path first, else the sibling of its desk key).
///
/// netd resolves the cert itself, from ITS environment, so clearing only the CLI's
/// path leaves a relocated cache authoritative: the WAN registry keeps presenting the
/// previous account after the switch. Deduplicated, because the ordinary case is that
/// all three name the same file.
fn stale_device_cert_paths() -> Result<Vec<PathBuf>, String> {
    let env = |name: &str| std::env::var(name).ok();
    stale_device_cert_paths_from(
        device_cert_path(),
        env(NETD_DEVICE_CERT_ENV).as_deref(),
        env(NETD_DESK_KEY_ENV).as_deref(),
    )
}

/// [`stale_device_cert_paths`]'s decision, with the environment passed in. Pure —
/// oracle-tested, and deliberately the same shape as netd's own
/// `wan::device_cert_path_from`, which is the resolution being mirrored.
fn stale_device_cert_paths_from(
    own: Option<PathBuf>,
    netd_cert: Option<&str>,
    netd_desk_key: Option<&str>,
) -> Result<Vec<PathBuf>, String> {
    let mut paths: Vec<PathBuf> = Vec::new();
    let mut push = |p: PathBuf| {
        if !paths.contains(&p) {
            paths.push(p);
        }
    };
    if let Some(p) = own {
        push(p);
    }
    // Each variable is read EXACTLY as `cerulion-netd` reads its own: the cert
    // path trimmed (netd's `device_cert_path_from` trims, so `"  "` is unset
    // there), the desk key VERBATIM (netd filters only on empty, so `"  "` is a
    // configured path there, and a relative one). Normalising them the same way
    // would make the CLI call a value unset that netd resolves to a file.
    fn trimmed(v: Option<&str>) -> Option<&str> {
        v.map(str::trim).filter(|s| !s.is_empty())
    }
    fn verbatim(v: Option<&str>) -> Option<&str> {
        v.filter(|s| !s.is_empty())
    }
    let cert = match trimmed(netd_cert) {
        Some(explicit) => Some(relocated(NETD_DEVICE_CERT_ENV, PathBuf::from(explicit))?),
        None => None,
    };
    // The desk key is checked even when an explicit cert path makes it irrelevant
    // HERE: it is netd's WAN identity, this is the one place the CLI resolves
    // netd's environment, and a relative one is as unusable to netd's key as to
    // its cert — refusing only the path this function happens to need would let a
    // login succeed against an identity netd resolves somewhere else entirely.
    let key = match verbatim(netd_desk_key).map(PathBuf::from) {
        Some(key) => Some(relocated(NETD_DESK_KEY_ENV, key)?),
        None => None,
    };
    match cert {
        Some(explicit) => push(explicit),
        // netd's own fallback: `device.cert` NEXT TO the desk key, which an override
        // can move out of `~/.cerulion` entirely.
        None => {
            if let Some(key) = key {
                push(match key.parent() {
                    Some(dir) if !dir.as_os_str().is_empty() => dir.join("device.cert"),
                    _ => PathBuf::from("device.cert"),
                });
            }
        }
    }
    Ok(paths)
}

/// Resolve the cert paths for their VALIDITY alone, discarding them.
///
/// Every login calls this, including the ones that clear nothing (a same-account
/// re-login, an identity-only service with no cert cached): the refusal a relative
/// `CERULION_NETD_*` override earns is a property of the CONFIGURATION, not of the
/// branch a particular login happens to take, and a login that skipped the check
/// would report a binding netd resolves somewhere else entirely.
pub fn check_netd_cert_paths() -> Result<(), String> {
    stale_device_cert_paths().map(|_| ())
}

/// A `CERULION_NETD_*` path is only usable if it is ABSOLUTE: the CLI and
/// `cerulion-netd` are separate processes with separate working directories, and a
/// relative override names a different file in each. Writing the CLI's one would
/// report a cached binding netd cannot see, so a relative override is refused by
/// name instead — one rule, applied where the path is resolved, so the clear and
/// the cache can never disagree about which file is netd's.
fn relocated(var: &str, path: PathBuf) -> Result<PathBuf, String> {
    if path.is_absolute() {
        return Ok(path);
    }
    Err(format!(
        "{var} is a relative path ({}), which `cerulion-netd` resolves against ITS working \
         directory, not this command's — set it to an absolute path",
        path.display()
    ))
}

/// Take every cached device cert this machine's consumers would read OUT OF THE
/// WAY, handing back the move so the caller can undo it — or survive dying
/// mid-login.
///
/// A login that resolves its account WITHOUT being issued a cert must call this:
/// [`crate::device_binding::resolve_device_binding`] treats a locally valid cert
/// as cryptographic truth about which account this machine is bound to, so a cert
/// left behind by an earlier login keeps naming THAT account while `auth.json`
/// names the new one. Absent is success — the point is the file not existing at
/// the path a consumer reads, not this call having deleted anything.
///
/// It clears `~/.cerulion/device.cert` AND the paths `cerulion-netd`'s own env
/// overrides ([`NETD_DEVICE_CERT_ENV`], [`NETD_DESK_KEY_ENV`]) relocate the cache
/// to: netd resolves the cert from its environment, so clearing one path leaves a
/// relocated copy authoritative for device binding.
///
/// # It is a RENAME, and that is the crash story
///
/// Each entry is renamed to a hidden sibling — `.device.cert.superseded.<owner>`,
/// where `<owner>` is the account `auth.json` named when the clear ran — instead
/// of being unlinked. Three properties follow, and each of them is a bug fixed:
///
/// 1. **An interrupted login does not lose the previous binding.** The clear and
///    the `auth.json` publication are separately durable, so a kill between them
///    is unavoidable; with an unlink it left the prior session live with its cert
///    deleted. The aside survives that kill, and the next login's
///    [`recover_superseded_device_certs`] renames it back — because the store
///    still names `<owner>` — putting the desk back exactly where it was.
/// 2. **Nothing is read, so nothing can block or refuse.** A `rename` does not
///    care whether the entry is a regular file, a symlink (dangling or not), a
///    FIFO with no writer, or a file this process cannot open: the previous
///    snapshot-then-unlink had to read the bytes to have a way back, which made an
///    unreadable entry unclearable and a FIFO override able to hang `cerulion
///    login` forever.
/// 3. **A symlink stays a symlink.** The rename moves the LINK, so putting it back
///    restores the link, not a regular file holding whatever it resolved to at
///    snapshot time — that would have frozen one account's cert at a path netd
///    rotates.
///
/// The aside is a hidden sibling, in the same directory as the entry (so the
/// rename is atomic and cannot cross a filesystem) and at a name no consumer
/// resolves.
///
/// # Guarantees
///
/// Every path is attempted even after one refuses — a switch that cleared some
/// consumers and not others is worse than either end — and the error names the
/// entries still in place, which is the recovery target. The clear is
/// ALL-OR-NOTHING: a failure moves back the ones already set aside, so a caller
/// that refuses its login leaves every consumer reading the cert it read before.
/// A move-back that itself fails is named in the error too — that file is the one
/// the operator must replace.
///
/// **Call it holding the store lock** ([`with_store_lock`]), inside the same
/// critical section that publishes the `auth.json` this clear is part of: the
/// owner tag it stamps is read from that store, and a login on another process
/// landing between the two would be rolled back by the wrong recovery decision.
///
/// The move it returns is a HANDLE, not a receipt: a caller that publishes
/// something else afterwards undoes the clear with [`ClearedCerts::put_back`] when
/// its own write fails, and drops the asides with [`ClearedCerts::discard`] once
/// the publication makes them meaningless — which is why the certs go first, see
/// `login_cmd`.
pub fn clear_device_cert(owner: Option<&str>) -> Result<ClearedCerts, ClearCertError> {
    let paths = stale_device_cert_paths().map_err(|message| ClearCertError {
        message,
        unrestored: Vec::new(),
    })?;
    if paths.is_empty() {
        return Err(ClearCertError {
            message: "no home directory (set CERULION_HOME) to locate ~/.cerulion/device.cert"
                .to_string(),
            unrestored: Vec::new(),
        });
    }
    let tag = superseded_owner_tag(owner);
    let mut moved: Vec<(PathBuf, PathBuf)> = Vec::new();
    // Every path is attempted even after one refuses: returning at the first
    // failure would leave the caller aborting a login with SOME consumers' caches
    // already gone and others still naming the previous account — the mixed state
    // is worse than either end, and the paths are independent. The error names the
    // entries that are still there, since that is the recovery target and the
    // ordinary case is that they are not `~/.cerulion/device.cert`.
    let mut failures: Vec<String> = Vec::new();
    for path in paths {
        let Some(aside) = superseded_path(&path, &tag) else {
            failures.push(format!(
                "{} (has no parent directory to hold its superseded copy)",
                path.display()
            ));
            continue;
        };
        // A leftover aside from an earlier crashed login at this same owner is
        // replaced, not honoured: the entry being moved aside NOW is the newer
        // one. (Unix `rename` replaces silently; the explicit removal is what
        // makes that true on platforms whose `rename` refuses an existing
        // destination.)
        let _ = std::fs::remove_file(&aside);
        match std::fs::rename(&path, &aside) {
            Ok(()) => moved.push((path, aside)),
            // Nothing there (or no directory at all). Absent is the goal, so it is
            // not a failure.
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => failures.push(format!("{} ({e})", path.display())),
        }
    }
    if failures.is_empty() {
        return Ok(ClearedCerts { moved });
    }
    let mut unrestored = Vec::new();
    for (path, aside) in &moved {
        if let Err(e) = std::fs::rename(aside, path) {
            failures.push(format!("{} (could not be put back: {e})", path.display()));
            unrestored.push(path.display().to_string());
        }
    }
    Err(ClearCertError {
        message: failures.join("; "),
        unrestored,
    })
}

/// Finish the job an interrupted login left half-done: put back, or drop, the
/// `.superseded.<owner>` asides [`clear_device_cert`] renamed out of the way.
///
/// The clear and the `auth.json` publication cannot be fused, so a login can die
/// between them. This is the other half of making that survivable, and the OWNER
/// TAG is what makes the decision unambiguous — for each aside found beside a cert
/// path a consumer reads:
///
/// * the live path already holds a cert ⇒ the aside is superseded for real, and is
///   dropped;
/// * the live path is empty and the tag is the account `auth.json` names NOW ⇒ the
///   login that moved it aside never published anything, so the session on disk is
///   still that account's and its binding is put back;
/// * the live path is empty and the tag names some OTHER account ⇒ the login DID
///   publish (or the store moved on some other way), so putting it back would bind
///   this desk to an account the store no longer names: it is dropped, leaving the
///   desk uncached, which is what a fresh machine is and what the next certifying
///   login fixes.
///
/// Never the cause of a failed login: an aside that cannot be acted on is logged
/// and left, since it is at a name no consumer resolves. **Call it holding the
/// store lock** ([`with_store_lock`]), before the login's own clear, so the store
/// it reads is the one the clear will replace.
pub fn recover_superseded_device_certs() {
    let Ok(paths) = stale_device_cert_paths() else {
        // A relative override is refused by the login itself, with a message that
        // names the variable; there is nothing to recover under a path this
        // process cannot agree with netd about.
        return;
    };
    let owner = auth_json_path()
        .map(|p| load_from(&p))
        .and_then(|loaded| loaded.state().map(|s| s.account_id.clone()));
    let want = superseded_owner_tag(owner.as_deref());
    for path in paths {
        let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) else {
            continue;
        };
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
            continue;
        };
        let prefix = format!(".{name}{SUPERSEDED_INFIX}");
        let Ok(entries) = std::fs::read_dir(parent) else {
            continue;
        };
        let live_is_empty = std::fs::symlink_metadata(&path).is_err();
        let mut restored = false;
        for entry in entries.flatten() {
            let Ok(found) = entry.file_name().into_string() else {
                continue;
            };
            let Some(tag) = found.strip_prefix(&prefix) else {
                continue;
            };
            let aside = parent.join(&found);
            if live_is_empty && !restored && tag == want {
                match std::fs::rename(&aside, &path) {
                    Ok(()) => {
                        restored = true;
                        tracing::info!(
                            path = %path.display(),
                            "put back the device cert an interrupted `cerulion login` had \
                             moved aside: this machine's `auth.json` still names the account \
                             it certifies, so the binding it was signed in with is restored"
                        );
                    }
                    Err(e) => tracing::warn!(
                        path = %path.display(),
                        error = %e,
                        "a device cert an interrupted `cerulion login` moved aside could not be \
                         put back — this desk has no cached device binding until the next \
                         certifying login"
                    ),
                }
                continue;
            }
            if let Err(e) = std::fs::remove_file(&aside) {
                tracing::debug!(
                    path = %aside.display(),
                    error = %e,
                    "a superseded device cert copy could not be dropped"
                );
            }
        }
    }
}

/// Finish the OTHER half an interrupted login can leave undone: give a cert this
/// machine has ALREADY VERIFIED to the consumers that ended up without one.
///
/// The cert is cached at one path per consumer and nothing fuses the renames, so
/// a kill during the commits (or a netd override pointed somewhere new after a
/// login, which is the same shape without a crash) leaves some paths holding the
/// cert and others empty — a consumer that resolves no device binding on a desk
/// that is signed in.
///
/// What may be propagated is the caller's whole responsibility, and it is why this
/// takes the bytes rather than reading them from a sibling: the only safe source
/// is a cert whose identity has been ESTABLISHED — decoded, attesting this
/// machine's device key, and naming the account the store names
/// ([`crate::device_binding::resolve_device_binding`], its one caller). Copying
/// whatever a sibling path happened to hold would put the PREVIOUS account's cert
/// back after a switch that cleared only the paths that switch could see, which is
/// the misbinding [`clear_device_cert`] exists to prevent.
///
/// The account the cert binds is taken as an argument and re-checked against the
/// store INSIDE the lock, because the caller's check cannot hold: between it and
/// this call another process can complete a switch — clearing every consumer path
/// and publishing a new `auth.json` — and this repair would then fill the paths it
/// just emptied with the cert of the account the desk signed out of. Under the
/// lock the store is authoritative, so a switch that landed first wins and this
/// writes nothing.
///
/// Otherwise deliberately narrow: only an ABSENT path is written — anything
/// present, of any kind, is left exactly as it is, so a symlink netd rotates is
/// never replaced — and a failure is logged, never returned, because this is
/// repair and the caller's own work is what must succeed or fail.
pub fn cache_verified_device_cert_at_absent_consumers(cert_b64: &str, cert_account: &str) {
    let Some(auth_path) = auth_json_path() else {
        return;
    };
    let _ = with_store_lock(&auth_path, || {
        match load().state() {
            Some(state) if state.account_id == cert_account => {}
            other => {
                tracing::info!(
                    cert_account = %cert_account,
                    store_account = other.map(|s| s.account_id.clone()).unwrap_or_default(),
                    "skipped caching the verified device cert: the store no longer names the \
                     account it binds — a login switched accounts while this command was \
                     resolving, and its own cert publication owns those paths now"
                );
                return Ok(());
            }
        }
        for path in absent_device_cert_consumer_paths() {
            match stage_secret(&path, cert_b64.as_bytes()).and_then(StagedCert::commit) {
                Ok(()) => tracing::info!(
                    path = %path.display(),
                    "cached the verified device cert for a consumer that had none — a login \
                     publishes them one at a time, and a consumer left empty resolves no \
                     device binding on a desk that is signed in"
                ),
                Err(e) => tracing::warn!(
                    path = %path.display(),
                    error = %e,
                    "a consumer with no cached device cert could not be given the verified \
                     one — it has no device binding until `cerulion login` runs again with \
                     that path writable"
                ),
            }
        }
        Ok::<(), std::io::Error>(())
    });
}

/// The device-cert consumer paths that currently hold NOTHING: the targets a
/// certifying login caches to beyond the ones it cleared, and the ones the repair
/// above fills.
///
/// Absent means absent: `symlink_metadata` does not follow, so a dangling symlink
/// counts as PRESENT and is left to the ordinary replace path rather than being
/// written through. A resolution failure yields none — a relative override has
/// already been refused by then, and a target list is not the place to fail a
/// second time.
pub fn absent_device_cert_consumer_paths() -> Vec<PathBuf> {
    stale_device_cert_paths()
        .unwrap_or_default()
        .into_iter()
        .filter(|p| {
            std::fs::symlink_metadata(p)
                .err()
                .is_some_and(|e| e.kind() == std::io::ErrorKind::NotFound)
        })
        .collect()
}

/// The device-cert consumer paths other than this CLI's own — the ones a
/// `CERULION_NETD_*` override relocates `cerulion-netd`'s cache to.
///
/// A read that finds nothing at the CLI's own path consults these before
/// reporting no binding: they hold a cert for the SAME machine, so a login that
/// died between its per-consumer commits (or one that never reached this path at
/// all) can leave the binding sitting at netd's copy alone. What may be believed
/// about what is there is the reader's business — it is verified before it is
/// used, exactly as the CLI's own copy is.
pub fn relocated_device_cert_consumer_paths() -> Vec<PathBuf> {
    let own = device_cert_path();
    stale_device_cert_paths()
        .unwrap_or_default()
        .into_iter()
        .filter(|p| Some(p) != own.as_ref())
        .collect()
}

/// Put back or drop the asides an interrupted login left, under the store lock.
///
/// This is what a READER of the binding calls — [`crate::device_binding`] — so a
/// login killed between moving a cert aside and publishing its `auth.json` is
/// repaired by the next command that needs the binding rather than only by the
/// next login. Which way an aside goes is decided by the account tag in its name,
/// never by its contents.
pub fn recover_interrupted_device_cert_state() {
    let Some(auth_path) = auth_json_path() else {
        return;
    };
    let _ = with_store_lock(&auth_path, || {
        recover_superseded_device_certs();
        Ok::<(), std::io::Error>(())
    });
}

/// The separator between a superseded cert's base name and the account tag that
/// says whose it is. Its own constant because the writer and the recovery scan
/// have to agree on it exactly.
const SUPERSEDED_INFIX: &str = ".superseded.";

/// The account whose cert an aside holds, as a filename-safe tag: everything
/// outside `[A-Za-z0-9._-]` becomes `_`, bounded so no id can push the name past a
/// filesystem's limit. `None` (no store, or a corrupt one) is `unknown`, which
/// recovery matches against the same "no account named" store — the state a
/// never-logged-in machine is in.
///
/// Collisions between two DIFFERENT ids that sanitise the same way would only
/// mis-restore a cert to the account that overwrote it, and account ids are
/// hex/uuid-shaped in practice, so nothing is escaped: the tag says whose it is,
/// it is not a key anything is looked up by.
fn superseded_owner_tag(owner: Option<&str>) -> String {
    let Some(owner) = owner.map(str::trim).filter(|s| !s.is_empty()) else {
        return "unknown".to_string();
    };
    owner
        .chars()
        .take(64)
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '.' || c == '_' || c == '-' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

/// The hidden sibling `path`'s cert is moved to. `None` when `path` has no
/// directory component to hold it — a relative bare filename, which the netd
/// overrides are already refused for.
fn superseded_path(path: &Path, tag: &str) -> Option<PathBuf> {
    let parent = path.parent().filter(|p| !p.as_os_str().is_empty())?;
    let name = path.file_name()?.to_str()?;
    Some(parent.join(format!(".{name}{SUPERSEDED_INFIX}{tag}")))
}

/// The certs [`clear_device_cert`] moved aside, in the form needed to put them
/// back.
///
/// Held by the caller for as long as its own commit can still fail: the clear runs
/// BEFORE the new `auth.json` is published (so a crash in between cannot leave the
/// new account beside the old account's cert), which means a failed publish is the
/// case that has to undo the clear — and a SUCCESSFUL one is the case that drops
/// the asides.
#[must_use = "a clear the caller neither undoes nor discards is a binding left in limbo"]
pub struct ClearedCerts {
    moved: Vec<(PathBuf, PathBuf)>,
}

impl ClearedCerts {
    /// Whether anything was actually there — for the log line, not for control
    /// flow: absent is success either way.
    #[must_use]
    pub fn any(&self) -> bool {
        !self.moved.is_empty()
    }

    /// The paths a cert was actually cleared FROM: the consumers this clear left
    /// with no binding at all.
    ///
    /// A login that was issued a cert of its own must cache it at every one of
    /// them, not just `~/.cerulion/device.cert` — `cerulion-netd` resolves the
    /// cert from ITS environment, so a switch that clears a relocated cache and
    /// writes only the CLI's path leaves netd reading nothing and the desk with no
    /// WAN binding after a login that reported success.
    #[must_use]
    pub fn cleared_paths(&self) -> Vec<&Path> {
        self.moved.iter().map(|(p, _)| p.as_path()).collect()
    }

    /// Put every cleared cert back exactly as it was — the same bytes, or the same
    /// symlink, since the undo is the reverse rename. `Err` names the paths that
    /// could NOT be put back: those bindings are gone and the caller must say so
    /// rather than claim the previous sign-in survived intact.
    pub fn put_back(&self) -> Result<(), Vec<String>> {
        let mut lost = Vec::new();
        for (path, aside) in &self.moved {
            if let Err(e) = std::fs::rename(aside, path) {
                lost.push(format!("{} ({e})", path.display()));
            }
        }
        if lost.is_empty() {
            Ok(())
        } else {
            Err(lost)
        }
    }

    /// Drop the asides: the new `auth.json` is published, so the certs they hold
    /// name an account this machine is no longer signed in to and there is nothing
    /// left to roll back to.
    ///
    /// Best effort on purpose — an aside that survives is a stale secret at a name
    /// no consumer resolves, and the next login's
    /// [`recover_superseded_device_certs`] drops it: failing a published login over
    /// it would report a sign-in that happened as a failure.
    pub fn discard(&self) {
        for (_, aside) in &self.moved {
            if let Err(e) = std::fs::remove_file(aside) {
                if e.kind() != std::io::ErrorKind::NotFound {
                    tracing::debug!(
                        path = %aside.display(),
                        error = %e,
                        "a superseded device cert copy could not be dropped; the next \
                         `cerulion login` will drop it"
                    );
                }
            }
        }
    }
}

/// A clear that refused: `message` names the entries still present (the recovery
/// target), and [`ClearCertError::unrestored`] the ones that were moved aside and
/// could not be moved back — a binding that is gone, which reads differently to
/// the user than a login that changed nothing.
#[derive(Debug)]
pub struct ClearCertError {
    message: String,
    unrestored: Vec<String>,
}

impl ClearCertError {
    /// Paths whose cert was cleared and could not be restored.
    #[must_use]
    pub fn unrestored(&self) -> &[String] {
        &self.unrestored
    }

    /// For tests of the callers' messages: the double-I/O-failure shape needs two
    /// injected failures at once, which no filesystem this suite can build gives
    /// it, so the message logic is pinned against a constructed error instead.
    #[cfg(test)]
    #[must_use]
    pub(crate) fn for_test(message: &str, unrestored: Vec<String>) -> Self {
        Self {
            message: message.to_string(),
            unrestored,
        }
    }
}

impl std::fmt::Display for ClearCertError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ClearCertError {}

// ===========================================================================
// The store lock — shared with Cerulion Studio
// ===========================================================================

/// The lock file every writer of `auth.json` holds while it reads, modifies and
/// publishes that store — this CLI **and** Cerulion Studio, which writes the same
/// file on a desk where both are installed.
///
/// The name is Studio's and is deliberately not re-spelled CLI-side: exclusion
/// exists only between processes that lock the SAME path, so this string is a
/// cross-process protocol rather than a local detail. Changing it here without
/// changing it there silently un-serializes the two writers.
///
/// `flock` on a sibling rather than on `auth.json` itself, because the store is
/// REPLACED by a rename: a lock held on its inode protects a file that is about
/// to stop being the store. A separate, never-renamed file is a stable thing to
/// serialize on, and being empty it can be locked without opening a credential
/// for writing.
pub const STORE_LOCK_FILE: &str = ".studio-auth.lock";

/// Where [`STORE_LOCK_FILE`] sits for a given `auth.json` path.
#[must_use]
pub fn store_lock_path(auth_path: &Path) -> PathBuf {
    auth_path.with_file_name(STORE_LOCK_FILE)
}

/// How long a writer waits for a peer to finish, and how often it retries.
///
/// Bounded rather than blocking: `cerulion login` runs in the foreground of a
/// terminal and Studio's writers run on the frame that handled a click, so a
/// blocking `flock` behind a wedged peer would hang rather than report. A real
/// read-modify-write of a small JSON file is microseconds, so a second is orders
/// of magnitude of headroom.
const LOCK_WAIT: std::time::Duration = std::time::Duration::from_millis(1_000);
const LOCK_POLL: std::time::Duration = std::time::Duration::from_millis(10);

/// A held store lock: released when dropped, and by the KERNEL if this process
/// dies holding it — which is what makes a crashed writer unable to wedge the
/// store for the next login.
#[cfg(unix)]
#[must_use = "the lock is released the moment this value drops"]
struct StoreLock {
    /// Kept solely to HOLD the `flock` (it is owned by the open file
    /// DESCRIPTION), never read.
    _file: std::fs::File,
}

/// Non-Unix: no `flock`, nothing held. `~/.cerulion` is a Unix path story and a
/// fake lock claiming exclusion would be worse than none.
#[cfg(not(unix))]
struct StoreLock;

#[cfg(unix)]
fn hold_store_lock(auth_path: &Path) -> std::io::Result<StoreLock> {
    use std::os::unix::fs::OpenOptionsExt;

    let path = store_lock_path(auth_path);
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            create_secret_dir(parent)?;
        }
    }
    // `truncate(false)`: the file's content is irrelevant and a peer may have it
    // open — the lock lives in the kernel, not in the bytes.
    let file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .mode(0o600)
        .open(&path)?;

    let deadline = std::time::Instant::now() + LOCK_WAIT;
    loop {
        match flock_exclusive_nonblocking(&file) {
            Ok(()) => return Ok(StoreLock { _file: file }),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                if std::time::Instant::now() >= deadline {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::WouldBlock,
                        format!(
                            "another Cerulion process is writing {} and did not finish within \
                             {}ms",
                            auth_path.display(),
                            LOCK_WAIT.as_millis()
                        ),
                    ));
                }
                std::thread::sleep(LOCK_POLL);
            }
            Err(e) => return Err(e),
        }
    }
}

/// `flock(LOCK_EX | LOCK_NB)` on `f`, reporting a contended lock as
/// [`std::io::ErrorKind::WouldBlock`].
///
/// The same two-line `libc` call as [`crate::run_lock`]'s liveness probe, for
/// the same reason (the kernel releases it on SIGKILL); not shared with it
/// because that module's `RunLock` also mints and unlinks the artifact, which a
/// long-lived credential-store lock must never do.
#[cfg(unix)]
fn flock_exclusive_nonblocking(f: &std::fs::File) -> std::io::Result<()> {
    use std::os::unix::io::AsRawFd;
    // SAFETY: `f` is a live, open file owned by the caller for the whole call,
    // so its raw fd is valid; `flock` only reads it.
    let rc = unsafe { libc::flock(f.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) };
    if rc != 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}

#[cfg(not(unix))]
fn hold_store_lock(_auth_path: &Path) -> std::io::Result<StoreLock> {
    Ok(StoreLock)
}

// The lock paths this THREAD already holds, so a nested `with_store_lock` (a
// read-modify-write whose inner `write_to` locks again) runs under the lock it
// is already inside instead of contending with itself. `flock` contends across
// open file DESCRIPTIONS, including two in one process, so without this the
// inner acquisition would burn `LOCK_WAIT` and then fail — a self-deadlock that
// reports.
//
// Thread-scoped rather than process-global: a second thread genuinely IS a
// second writer and must serialize against this one through the kernel.
thread_local! {
    static HELD_LOCKS: std::cell::RefCell<Vec<PathBuf>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

/// Run `write` with the credential store's lock held, so a read-modify-write of
/// `auth.json` is not interleaved with another writer's.
///
/// This is the CLI half of the protocol Studio's `desk_login::with_store_lock`
/// implements: both open [`STORE_LOCK_FILE`] beside the store and take an
/// exclusive `flock` for the whole read-compare-publish. Without it, the two
/// processes exclude each other only by comparing the file before the rename —
/// and a peer landing between the compare and the rename is overwritten
/// wholesale (a `cerulion login` losing its session to Studio's refresh, a
/// sign-out undone by a rotation that read the store a moment earlier).
///
/// Re-entrant within one thread: wrapping a `load()` + [`write_to`] pair is the
/// intended use even though `write_to` locks too.
///
/// # Errors
///
/// [`std::io::ErrorKind::WouldBlock`] when a peer holds the lock for longer than
/// the bounded wait, plus whatever `write` itself returns. A caller that cannot
/// proceed without publishing must surface it: refusing to write is the point.
pub fn with_store_lock<T>(
    auth_path: &Path,
    write: impl FnOnce() -> std::io::Result<T>,
) -> std::io::Result<T> {
    let lock_path = store_lock_path(auth_path);
    if HELD_LOCKS.with(|held| held.borrow().contains(&lock_path)) {
        return write();
    }
    let held = hold_store_lock(auth_path)?;
    HELD_LOCKS.with(|h| h.borrow_mut().push(lock_path.clone()));
    // Released even if `write` panics — the guard drops during the unwind, and
    // the kernel drops the `flock` with the file.
    struct Release(PathBuf);
    impl Drop for Release {
        fn drop(&mut self) {
            HELD_LOCKS.with(|h| {
                let mut h = h.borrow_mut();
                if let Some(i) = h.iter().rposition(|p| *p == self.0) {
                    h.remove(i);
                }
            });
        }
    }
    let _release = Release(lock_path);
    let out = write();
    drop(held);
    out
}

/// Atomically write `bytes` to `final_path` as an **owner-only secret**:
///
/// - the parent directory is created **0700** on Unix (not the umask default
///   0755) so the credential filenames are not world-listable,
/// - the staging temp is a **unique-per-process** sibling (`.<name>.<pid>.tmp`)
///   so two concurrent writers NEVER share a staging inode (no torn temp),
/// - a stale temp is unlinked (removing a symlink, NEVER following it) and the
///   temp is created with `create_new` + mode 0600 — so a pre-existing/symlinked
///   temp can neither redirect the write nor leave the file at umask perms,
/// - the temp is fsynced then `rename`d over `final_path` (atomic replace).
///
/// `pub(crate)` so the other `~/.cerulion` writers (`robots.toml` in `pair_cmd`,
/// `peers.json` in `peer_cache`) get the SAME crash-durable, atomic write instead
/// of a bare `fs::write` + `rename` (no fsync, no `create_new`). Those files are
/// per-user CLI state, so landing at 0600 is fine — the stronger contract, for
/// free.
pub(crate) fn atomic_write_secret(final_path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    stage_secret(final_path, bytes)?.commit()
}

/// [`atomic_write_secret`]'s first half: everything up to (not including) the
/// rename that publishes it. Split out so a caller writing SEVERAL secrets can
/// learn that all of them are writable before any of them is visible.
fn stage_secret(final_path: &Path, bytes: &[u8]) -> std::io::Result<StagedCert> {
    if let Some(parent) = final_path.parent() {
        if !parent.as_os_str().is_empty() {
            create_secret_dir(parent)?;
        }
    }
    let tmp = unique_temp_path(final_path);
    // Remove any stale temp FIRST (unlinks a symlink rather than following it,
    // and clears a crashed prior write's leftover) so `create_new` succeeds.
    match std::fs::remove_file(&tmp) {
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => return Err(e),
    }
    write_new_secret_file(&tmp, bytes)?;
    Ok(StagedCert {
        tmp,
        final_path: final_path.to_path_buf(),
        committed: false,
    })
}

/// A secret written to a hidden staging sibling and not yet published. Dropping it
/// unpublished unlinks the staging file, so an abandoned multi-path update leaves
/// no litter for the next writer's `create_new` to trip over.
#[must_use = "a staged secret nothing commits is a write that never happened"]
pub struct StagedCert {
    tmp: PathBuf,
    final_path: PathBuf,
    committed: bool,
}

impl StagedCert {
    /// The path this will publish to, for the caller's own log lines.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.final_path
    }

    /// Publish it: `rename` the staged file over the destination (atomic replace),
    /// then flush the directory entry.
    pub fn commit(mut self) -> std::io::Result<()> {
        std::fs::rename(&self.tmp, &self.final_path)?;
        self.committed = true;
        // The bytes are fsynced; the DIRECTORY ENTRY the rename created is not, so
        // a power cut in the next moments can lose the replacement and leave the
        // prior credential (or nothing) in place. Best effort on purpose: a
        // directory that refuses `fsync` (some network and virtualised mounts do)
        // has still taken the rename, and failing the write there would refuse a
        // login that worked.
        if let Some(parent) = self.final_path.parent() {
            if !parent.as_os_str().is_empty() {
                if let Err(e) = std::fs::File::open(parent).and_then(|d| d.sync_all()) {
                    tracing::debug!(
                        path = %parent.display(),
                        error = %e,
                        "the credential directory could not be flushed after the rename"
                    );
                }
            }
        }
        Ok(())
    }
}

impl Drop for StagedCert {
    fn drop(&mut self) {
        // A committed staging file was RENAMED away, so there is nothing to
        // remove; an uncommitted one leaves the destination untouched, so all this
        // removes is the hidden temp.
        if self.committed {
            return;
        }
        if let Err(e) = std::fs::remove_file(&self.tmp) {
            if e.kind() != std::io::ErrorKind::NotFound {
                tracing::debug!(
                    path = %self.tmp.display(),
                    error = %e,
                    "an uncommitted staging file could not be removed"
                );
            }
        }
    }
}

/// A unique-per-process staging sibling of `final_path`: `.<name>.<pid>.tmp` in
/// the same directory (so the rename is atomic on the target filesystem, and two
/// processes never collide on the temp inode).
fn unique_temp_path(final_path: &Path) -> PathBuf {
    let name = final_path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| "cerulion".to_string());
    let tmp_name = format!(".{name}.{}.tmp", std::process::id());
    match final_path.parent() {
        Some(p) if !p.as_os_str().is_empty() => p.join(tmp_name),
        _ => PathBuf::from(tmp_name),
    }
}

/// Create `dir` (and parents) owner-only — 0700 on Unix. Idempotent (an existing
/// dir is left untouched, mode included; a pre-existing HOME is never chmod'd).
///
/// `pub(crate)` so the other `~/.cerulion` writers (the device-key seed path in
/// `login_cmd`, `robots.toml` in `pair_cmd`, `peers.json` in `peer_cache`) create
/// the shared secret dir at 0700 too — a plain `create_dir_all` there would leave
/// `~/.cerulion` world-listable (0755) whenever it runs before any secret write.
pub(crate) fn create_secret_dir(dir: &Path) -> std::io::Result<()> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::DirBuilderExt;
        std::fs::DirBuilder::new()
            .recursive(true)
            .mode(0o700)
            .create(dir)
    }
    #[cfg(not(unix))]
    {
        std::fs::create_dir_all(dir)
    }
}

/// Create `path` with `create_new` (0600 on Unix) and write `bytes`, fsyncing.
/// `create_new` fails if the path exists — belt-and-suspenders vs the unlink in
/// [`atomic_write_secret`] and a racing writer, and it NEVER follows a symlink at
/// that path (the kernel refuses `O_CREAT|O_EXCL` on an existing symlink).
#[cfg(unix)]
fn write_new_secret_file(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    use std::io::Write;
    use std::os::unix::fs::OpenOptionsExt;
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(path)?;
    f.write_all(bytes)?;
    f.sync_all()
}

/// Non-Unix: no POSIX mode bits (the file still holds secrets — a Windows ACL
/// story is future work). `create_new` still refuses to follow/overwrite an
/// existing path.
#[cfg(not(unix))]
fn write_new_secret_file(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    use std::io::Write;
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)?;
    f.write_all(bytes)?;
    f.sync_all()
}

// ===========================================================================
// Test and CI seeding.
//
// Deliberately not behind `cfg(test)` or a feature: a cfg gate here would guard
// nothing. `auth.json` is a plain JSON file in a directory the caller chooses
// with `CERULION_HOME`, and the sibling shell script writes the identical bytes
// with no Cerulion code involved. Every crate whose tests run the binary needs
// this, and two of them sit BELOW the CLI engine in the dependency graph, where
// a gated function would force dependency cycles to reach something that
// guards nothing.
//
// The rule this must not break: seeding is not a switch. Writing local state
// that says this machine signed in is the same act a real sign-in performs, and
// the gate reads it the same way.
// ===========================================================================

/// Seed a **valid, logged-in-ever** [`AuthState`] into `cerulion_dir` (treated
/// as the `.cerulion` config dir directly — set `CERULION_HOME` to it so the
/// gate reads it). Writes `auth.json` with a far-future session expiry through
/// the same [`write_to`] every real sign-in uses, so the format cannot drift
/// away from the one the gate parses.
///
/// For a shell harness that cannot call Rust, `tools/ci/seed_test_login.sh <dir>`
/// writes the same file; a unit test below runs that script and requires the
/// result to load as a state the gate accepts.
///
/// Hidden from the docs because it is provisioning for tests and CI, not part of
/// the product surface.
#[doc(hidden)]
pub fn seed_logged_in_at(cerulion_dir: &Path, account_id: &str) -> std::io::Result<()> {
    let state = AuthState {
        account_id: account_id.to_string(),
        session_token: "seed-session".to_string(),
        refresh_token: "seed-refresh".to_string(),
        // Far-future expiry so the seeded session reads as valid for decades.
        expires_at_ns: u64::MAX,
        logged_in_ever: true,
        // A seeded generic login is role-unmarked (resolves to Desk) — the seam
        // provisions a passing gate, not a robot.
        role: None,
    };
    write_to(&cerulion_dir.join("auth.json"), &state)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(logged_in_ever: bool, expires_at_ns: u64) -> AuthState {
        AuthState {
            account_id: "acct-abc".to_string(),
            session_token: "sess".to_string(),
            refresh_token: "refr".to_string(),
            expires_at_ns,
            logged_in_ever,
            role: None,
        }
    }

    // --- the CI seeding script agrees with the type it has to parse as ------

    /// `tools/ci/seed_test_login.sh` is the second writer of `auth.json` (the
    /// first is [`write_to`], which every real sign-in and [`seed_logged_in_at`]
    /// go through). A shell script cannot share a serde derive, so the only thing
    /// keeping the two in step is this test: it RUNS the script and requires the
    /// file it leaves behind to load as an [`AuthState`] the gate lets through.
    ///
    /// Running it, rather than reading its text, is the point. A test that
    /// scraped the heredoc would keep passing if the script stopped writing the
    /// file, wrote it to the wrong name, or failed outright.
    #[cfg(unix)]
    #[test]
    fn the_ci_seed_script_writes_a_state_the_gate_accepts() {
        let script = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../tools/ci/seed_test_login.sh")
            .canonicalize()
            .expect("tools/ci/seed_test_login.sh is missing");
        let dir = tempfile::tempdir().unwrap();
        let seeded = dir.path().join("cerulion-home");

        let out = std::process::Command::new("sh")
            .arg(&script)
            .arg(&seeded)
            .output()
            .expect("run the seed script");
        assert!(
            out.status.success(),
            "the seed script failed: {}",
            String::from_utf8_lossy(&out.stderr)
        );

        let loaded = load_from(&seeded.join("auth.json"));
        let state = match &loaded {
            LoadedAuth::Present(s) => s,
            other => panic!("the script's auth.json must parse as AuthState: {other:?}"),
        };
        assert!(
            state.logged_in_ever,
            "the marker the gate reads must be set: {state:?}"
        );
        assert_eq!(
            local_gate(&loaded, now_unix_ns()),
            LocalGate::ProceedValidSession,
            "a seeded home must let a command through"
        );
    }

    /// The two writers must not drift into producing states the gate treats
    /// differently. They deliberately carry different account ids (the script's
    /// is fixed, the function's is the caller's), so the comparison is over every
    /// field the gate actually consults.
    #[cfg(unix)]
    #[test]
    fn the_seed_script_and_the_seed_function_agree_on_what_the_gate_reads() {
        let script = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../tools/ci/seed_test_login.sh")
            .canonicalize()
            .expect("tools/ci/seed_test_login.sh is missing");
        let dir = tempfile::tempdir().unwrap();
        let by_script = dir.path().join("script");
        let by_fn = dir.path().join("fn");

        assert!(std::process::Command::new("sh")
            .arg(&script)
            .arg(&by_script)
            .status()
            .expect("run the seed script")
            .success());
        std::fs::create_dir_all(&by_fn).unwrap();
        seed_logged_in_at(&by_fn, "seeded-test-account").unwrap();

        let a = match load_from(&by_script.join("auth.json")) {
            LoadedAuth::Present(s) => s,
            other => panic!("script state: {other:?}"),
        };
        let b = match load_from(&by_fn.join("auth.json")) {
            LoadedAuth::Present(s) => s,
            other => panic!("function state: {other:?}"),
        };
        assert_eq!(a.logged_in_ever, b.logged_in_ever);
        assert_eq!(a.expires_at_ns, b.expires_at_ns);
        assert_eq!(a.role, b.role);
        assert_eq!(a.machine_role(), b.machine_role());
    }

    // --- stale device certs an account switch invalidates -------------------

    /// The env-var names duplicated here must be netd's own: its `wan` module is
    /// behind a feature the CLI must not enable (iroh leanness), so the copy is
    /// pinned against netd's source instead of its symbol.
    #[test]
    fn netd_cert_env_names_match_netds_own_source() {
        let wan = include_str!("../../cerulion_netd/src/wan.rs");
        assert!(
            wan.contains(&format!(
                "pub const DEVICE_CERT_ENV: &str = \"{NETD_DEVICE_CERT_ENV}\""
            )),
            "netd renamed its device-cert env var; NETD_DEVICE_CERT_ENV is now clearing nothing"
        );
        assert!(
            wan.contains(&format!(
                "pub const DESK_KEY_ENV: &str = \"{NETD_DESK_KEY_ENV}\""
            )),
            "netd renamed its desk-key env var; the sibling cert would be missed"
        );
    }

    #[test]
    fn stale_cert_paths_are_every_path_a_consumer_reads() {
        let own = || Some(PathBuf::from("/home/x/.cerulion/device.cert"));

        // The ordinary case: netd unconfigured ⇒ the CLI's own path, once.
        assert_eq!(
            stale_device_cert_paths_from(own(), None, None),
            Ok(vec![PathBuf::from("/home/x/.cerulion/device.cert")])
        );

        // An explicit netd cert path is a SECOND file to clear — leaving it is the
        // account confusion this exists to prevent.
        assert_eq!(
            stale_device_cert_paths_from(own(), Some("/srv/certs/desk.cert"), None),
            Ok(vec![
                PathBuf::from("/home/x/.cerulion/device.cert"),
                PathBuf::from("/srv/certs/desk.cert"),
            ])
        );

        // No explicit cert ⇒ netd reads the SIBLING of its desk key.
        assert_eq!(
            stale_device_cert_paths_from(own(), None, Some("/srv/keys/desk.key")),
            Ok(vec![
                PathBuf::from("/home/x/.cerulion/device.cert"),
                PathBuf::from("/srv/keys/device.cert"),
            ])
        );

        // An explicit cert WINS over the desk-key sibling, as it does in netd.
        assert_eq!(
            stale_device_cert_paths_from(own(), Some("/srv/certs/a.cert"), Some("/srv/keys/b.key")),
            Ok(vec![
                PathBuf::from("/home/x/.cerulion/device.cert"),
                PathBuf::from("/srv/certs/a.cert"),
            ])
        );

        // Blank/whitespace is unset (netd trims too), and naming the same file
        // twice is one removal, not two.
        assert_eq!(
            stale_device_cert_paths_from(own(), Some("  "), None),
            Ok(vec![PathBuf::from("/home/x/.cerulion/device.cert")])
        );
        assert_eq!(
            stale_device_cert_paths_from(own(), Some("/home/x/.cerulion/device.cert"), None),
            Ok(vec![PathBuf::from("/home/x/.cerulion/device.cert")])
        );

        // No home AND no netd override ⇒ nothing to clear, which `clear_device_cert`
        // reports as an error rather than a silent success.
        assert_eq!(stale_device_cert_paths_from(None, None, None), Ok(vec![]));
    }

    /// The two variables' whitespace rules are netd's, not a rule of our own:
    /// `cerulion_netd::wan` trims the explicit CERT value before using it, and takes
    /// the DESK KEY verbatim whenever it is non-empty. So a whitespace-only cert path
    /// is unset for both of us, and a whitespace-only key path is a real (relative)
    /// path netd will resolve in its own directory — which is refused, not ignored.
    /// A CLI that trimmed both would report "nothing configured" for a desk whose netd
    /// is reading a file named " ".
    #[test]
    fn the_whitespace_rules_are_the_ones_netd_applies_to_each_variable() {
        assert_eq!(
            stale_device_cert_paths_from(None, Some(" \t "), None),
            Ok(vec![]),
            "a whitespace-only cert override is unset, as netd reads it"
        );

        let refused = stale_device_cert_paths_from(None, None, Some("  "))
            .expect_err("a whitespace-only desk key is a relative path to netd, not unset");
        assert!(
            refused.contains(NETD_DESK_KEY_ENV) && refused.contains("absolute"),
            "the refusal names the variable and what it needs: {refused}"
        );
    }

    /// A RELATIVE `CERULION_NETD_*` path names a different file in the CLI's
    /// working directory than in netd's, so neither the clear nor the cache can
    /// know which file netd reads. Resolving it against the CLI's own cwd would
    /// report a cached binding netd cannot see, so it is refused by name — the
    /// same rule at the one place both the clear and the cache resolve paths.
    #[test]
    fn a_relative_netd_cert_override_is_refused_by_name() {
        let own = || Some(PathBuf::from("/home/x/.cerulion/device.cert"));

        for (cert, key, var) in [
            (Some("certs/desk.cert"), None, NETD_DEVICE_CERT_ENV),
            (Some("./desk.cert"), None, NETD_DEVICE_CERT_ENV),
            (None, Some("keys/desk.key"), NETD_DESK_KEY_ENV),
            // A bare filename is relative too: its sibling `device.cert` would be
            // resolved in whatever directory `cerulion login` happened to run in.
            (None, Some("desk.key"), NETD_DESK_KEY_ENV),
            // An explicit cert path makes the desk key irrelevant to THIS
            // resolution, and it is still refused: it is netd's WAN identity,
            // this is the one place the CLI reads netd's environment, and a
            // login that passed here would report a binding against a key netd
            // resolves in another directory entirely.
            (
                Some("/srv/certs/desk.cert"),
                Some("keys/desk.key"),
                NETD_DESK_KEY_ENV,
            ),
        ] {
            let refused = stale_device_cert_paths_from(own(), cert, key)
                .expect_err("a relative netd override is not resolvable by this process");
            assert!(
                refused.contains(var) && refused.contains("absolute"),
                "the refusal names the variable and what it needs: {refused}"
            );
        }

        // An absolute desk key still resolves its sibling, unchanged.
        assert_eq!(
            stale_device_cert_paths_from(None, None, Some("/srv/keys/desk.key")),
            Ok(vec![PathBuf::from("/srv/keys/device.cert")])
        );
    }

    /// An uncommitted staging file leaves the destination untouched and takes its
    /// own temp with it, so a multi-path update that abandons half its stagings
    /// publishes nothing and litters nothing.
    #[test]
    fn an_uncommitted_staging_publishes_nothing_and_leaves_no_temp() {
        let dir = tempfile::tempdir().unwrap();
        let target = dir.path().join("device.cert");
        std::fs::write(&target, "the-previous-cert").unwrap();

        let staged = stage_device_cert_at(&target, "the-new-cert").unwrap();
        assert_eq!(staged.path(), target.as_path());
        assert_eq!(
            std::fs::read_to_string(&target).unwrap(),
            "the-previous-cert",
            "staging publishes nothing"
        );
        drop(staged);

        assert_eq!(
            std::fs::read_to_string(&target).unwrap(),
            "the-previous-cert"
        );
        let leftovers: Vec<String> = std::fs::read_dir(dir.path())
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .filter(|n| n != "device.cert")
            .collect();
        assert!(
            leftovers.is_empty(),
            "a dropped staging removes its own temp: {leftovers:?}"
        );

        // And a committed one publishes exactly the staged bytes.
        stage_device_cert_at(&target, "the-new-cert")
            .unwrap()
            .commit()
            .unwrap();
        assert_eq!(std::fs::read_to_string(&target).unwrap(), "the-new-cert");
    }

    // --- gate matrix — pure oracle vectors -----------------------------------

    #[test]
    fn gate_never_logged_in_absent_file_refuses() {
        // Row 1: no auth.json at all ⇒ hard refuse.
        assert_eq!(
            local_gate(&LoadedAuth::Absent, 1_000),
            LocalGate::RefuseNeverLoggedIn
        );
    }

    #[test]
    fn gate_corrupt_file_is_never_logged_in() {
        // Row 1: a corrupt file is folded into never-logged-in (corrupt ⇒
        // re-login), NEVER a proceed.
        assert_eq!(
            local_gate(&LoadedAuth::Corrupt("bad json".into()), 1_000),
            LocalGate::RefuseNeverLoggedIn
        );
    }

    #[test]
    fn gate_logged_in_ever_false_refuses_even_with_valid_session() {
        // A state with a valid session but logged_in_ever==false is still a
        // refuse — the durable marker is the gate, not the token.
        let s = state(false, u64::MAX);
        assert_eq!(
            local_gate(&LoadedAuth::Present(s), 1_000),
            LocalGate::RefuseNeverLoggedIn
        );
    }

    #[test]
    fn gate_valid_session_proceeds_zero_network() {
        // Row 2: logged-in-ever + valid session ⇒ proceed.
        let s = state(true, 2_000);
        assert_eq!(
            local_gate(&LoadedAuth::Present(s), 1_000),
            LocalGate::ProceedValidSession
        );
    }

    #[test]
    fn gate_expired_session_proceeds_locally_forever() {
        // Row 3: logged-in-ever + EXPIRED session ⇒ proceed locally, no
        // horizon. now (5_000) >= expires (2_000).
        let s = state(true, 2_000);
        let g = local_gate(&LoadedAuth::Present(s), 5_000);
        assert_eq!(g, LocalGate::ProceedExpiredLocalForever);
        assert!(
            g.may_proceed(),
            "expired-but-logged-in must proceed locally"
        );
    }

    #[test]
    fn gate_expiry_boundary_is_exclusive() {
        // now == expires_at is EXPIRED (validity is `now < expires`), so a
        // logged-in machine still proceeds locally (row 3), never refuses.
        let s = state(true, 2_000);
        assert_eq!(
            local_gate(&LoadedAuth::Present(s), 2_000),
            LocalGate::ProceedExpiredLocalForever
        );
    }

    #[test]
    fn wan_gate_valid_allows_expired_requires_refresh() {
        // Row 4: the cloud call is Allow while valid, RefreshRequired once stale.
        let s = state(true, 2_000);
        assert_eq!(wan_gate(&s, 1_000), WanGate::Allow);
        assert_eq!(wan_gate(&s, 2_000), WanGate::RefreshRequired);
        assert_eq!(wan_gate(&s, 9_000), WanGate::RefreshRequired);
    }

    // --- round-trip + corrupt handling ---------------------------------------

    #[test]
    fn write_then_load_round_trips() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        let s = state(true, 42);
        write_to(&path, &s).unwrap();
        match load_from(&path) {
            LoadedAuth::Present(got) => assert_eq!(got, s),
            other => panic!("expected Present, got {other:?}"),
        }
    }

    #[test]
    fn write_is_chmod_600_on_unix() {
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("auth.json");
            write_to(&path, &state(true, 1)).unwrap();
            let mode = std::fs::metadata(&path).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode, 0o600, "auth.json must be 0600 (it holds secrets)");
        }
    }

    #[test]
    fn missing_file_is_absent() {
        let dir = tempfile::tempdir().unwrap();
        assert!(matches!(
            load_from(&dir.path().join("nope.json")),
            LoadedAuth::Absent
        ));
    }

    #[test]
    fn garbage_file_is_corrupt_not_a_crash() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        std::fs::write(&path, b"{ this is not valid json ]").unwrap();
        match load_from(&path) {
            LoadedAuth::Corrupt(reason) => assert!(reason.contains("parse")),
            other => panic!("expected Corrupt, got {other:?}"),
        }
        // The corrupt file is NOT deleted (re-login overwrites it).
        assert!(path.exists(), "corrupt auth.json must be left in place");
    }

    #[test]
    fn role_field_parses_into_machine_role() {
        // The install-time robot-vs-desk `role` marker is now a REAL
        // optional field (promoted from the forward-compat "future field").
        // The exact lowercase wire shape a newer install funnel writes (`"role":
        // "robot"`) parses INTO `Some(MachineRole::Robot)` — a real round-trip from a
        // hand-written JSON body (not a struct-literal echo), and the resolver reports
        // Robot.
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        std::fs::write(
            &path,
            br#"{"account_id":"acct-x","session_token":"s","refresh_token":"r",
                 "expires_at_ns":123,"logged_in_ever":true,"role":"robot"}"#,
        )
        .unwrap();
        match load_from(&path) {
            LoadedAuth::Present(s) => {
                assert!(s.logged_in_ever);
                assert_eq!(s.account_id, "acct-x");
                assert_eq!(s.role, Some(MachineRole::Robot));
                assert_eq!(s.machine_role(), MachineRole::Robot);
            }
            other => panic!("the `role` field must parse into MachineRole, got {other:?}"),
        }
    }

    #[test]
    fn a_genuinely_unknown_field_is_still_tolerated() {
        // The no-`deny_unknown_fields` never-bricks contract, re-pinned on a field
        // that is STILL unknown (now that `role` is a real field): an auth.json a
        // future binary writes with some NEW key must parse (the unknown key ignored,
        // never a crash / Corrupt). Guards against a future `deny_unknown_fields`
        // regressing the never-bricks read of a newer-binary file.
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        std::fs::write(
            &path,
            br#"{"account_id":"acct-y","session_token":"s","refresh_token":"r",
                 "expires_at_ns":7,"logged_in_ever":true,"future_field":{"nested":42}}"#,
        )
        .unwrap();
        match load_from(&path) {
            LoadedAuth::Present(s) => {
                assert!(s.logged_in_ever);
                assert_eq!(s.account_id, "acct-y");
                // The unknown field is ignored; role stays unmarked → Desk.
                assert_eq!(s.role, None);
                assert_eq!(s.machine_role(), MachineRole::Desk);
            }
            other => panic!("an unknown field must be tolerated, got {other:?}"),
        }
    }

    #[test]
    fn role_round_trips_all_states_and_unmarked_is_omitted() {
        // Every role state survives a write→load round-trip; an unmarked (None) file
        // is byte-identical to the pre-field shape (the `role` key is OMITTED, not
        // written as null — `skip_serializing_if`), so an unmarked machine's auth.json
        // reads identically on an old binary.
        let dir = tempfile::tempdir().unwrap();
        for (role, expect) in [
            (Some(MachineRole::Desk), MachineRole::Desk),
            (Some(MachineRole::Robot), MachineRole::Robot),
            (None, MachineRole::Desk),
        ] {
            let path = dir.path().join(format!("auth-{role:?}.json"));
            let s = AuthState {
                role,
                ..state(true, 1)
            };
            write_to(&path, &s).unwrap();
            match load_from(&path) {
                LoadedAuth::Present(got) => {
                    assert_eq!(got, s, "round-trip preserves the whole state incl. role");
                    assert_eq!(got.role, role);
                    assert_eq!(got.machine_role(), expect);
                }
                other => panic!("expected Present, got {other:?}"),
            }
            // The unmarked file omits the `role` key entirely (byte-shape parity).
            let raw = std::fs::read_to_string(&path).unwrap();
            assert_eq!(
                raw.contains("\"role\""),
                role.is_some(),
                "the `role` key is present iff a role is stamped (unmarked ⇒ omitted): {raw}"
            );
        }
    }

    #[test]
    fn legacy_auth_json_without_a_role_key_resolves_desk() {
        // A legacy auth.json (no `role` key at all) parses with `role: None` (serde
        // default) and resolves to Desk — the never-bricks read of an OLD file by a
        // NEW binary (the mirror of the forward-compat test above).
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        std::fs::write(
            &path,
            br#"{"account_id":"legacy","session_token":"s","refresh_token":"r",
                 "expires_at_ns":9,"logged_in_ever":true}"#,
        )
        .unwrap();
        match load_from(&path) {
            LoadedAuth::Present(s) => {
                assert_eq!(s.role, None, "a legacy file has no role ⇒ None");
                assert_eq!(s.machine_role(), MachineRole::Desk);
            }
            other => panic!("a legacy (no-role) auth.json must parse, got {other:?}"),
        }
    }

    #[test]
    fn an_unknown_role_value_is_tolerated_and_login_does_not_brick() {
        // Never bricks for unknown values: a role string
        // THIS binary does not recognize (a future "operator" a newer binary wrote)
        // must NOT fail the whole AuthState parse. Failing it would go Corrupt →
        // never-logged-in gate refusal → a bricked login. Instead it degrades to Present
        // with role None (→ Desk), and the gate PROCEEDS.
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        std::fs::write(
            &path,
            br#"{"account_id":"acct-op","session_token":"s","refresh_token":"r",
                 "expires_at_ns":18446744073709551615,"logged_in_ever":true,"role":"operator"}"#,
        )
        .unwrap();
        match load_from(&path) {
            LoadedAuth::Present(s) => {
                assert_eq!(s.role, None, "an unrecognized role value degrades to None");
                assert_eq!(s.machine_role(), MachineRole::Desk);
                // The login gate PROCEEDS on the foreign-role file — it is NOT treated
                // as never-logged-in (which a Corrupt parse would have caused).
                assert_eq!(
                    local_gate(&LoadedAuth::Present(s), 0),
                    LocalGate::ProceedValidSession,
                    "a foreign role must never brick the login gate"
                );
            }
            other => panic!("an unknown role value must be tolerated (Present), got {other:?}"),
        }
    }

    #[test]
    fn a_non_string_role_value_is_tolerated_too() {
        // Robustness beyond strings: a role written as a NUMBER, OBJECT, or explicit
        // null (a structured future role, or junk) also degrades to None rather than
        // erroring the parse — every JSON shape is tolerated, so nothing bricks.
        let dir = tempfile::tempdir().unwrap();
        for body in [
            &br#"{"account_id":"a","session_token":"s","refresh_token":"r","expires_at_ns":9,"logged_in_ever":true,"role":42}"#[..],
            &br#"{"account_id":"a","session_token":"s","refresh_token":"r","expires_at_ns":9,"logged_in_ever":true,"role":{"kind":"op"}}"#[..],
            &br#"{"account_id":"a","session_token":"s","refresh_token":"r","expires_at_ns":9,"logged_in_ever":true,"role":null}"#[..],
        ] {
            let path = dir.path().join("auth.json");
            std::fs::write(&path, body).unwrap();
            match load_from(&path) {
                LoadedAuth::Present(s) => {
                    assert_eq!(
                        s.role,
                        None,
                        "a non-(desk|robot) role shape ⇒ None: {}",
                        String::from_utf8_lossy(body)
                    );
                    assert_eq!(s.machine_role(), MachineRole::Desk);
                }
                other => panic!("a non-string role must be tolerated, got {other:?}"),
            }
        }
    }

    #[test]
    fn an_unknown_role_value_is_dropped_on_the_next_write() {
        // The documented TRADE-OFF: an unrecognized value is NOT preserved. Parse a
        // foreign-role file (role → None), write the state back (what the carry-forward
        // does), and the `role` key is OMITTED — the foreign "operator" is dropped, NOT
        // round-tripped and NOT rewritten as a placeholder like "unknown".
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        std::fs::write(
            &path,
            br#"{"account_id":"acct-op","session_token":"s","refresh_token":"r",
                 "expires_at_ns":9,"logged_in_ever":true,"role":"operator"}"#,
        )
        .unwrap();
        let state = match load_from(&path) {
            LoadedAuth::Present(s) => s,
            other => panic!("expected Present, got {other:?}"),
        };
        write_to(&path, &state).unwrap();
        let raw = std::fs::read_to_string(&path).unwrap();
        assert!(
            !raw.contains("\"role\""),
            "the dropped foreign role must NOT be written back (key omitted): {raw}"
        );
        assert!(
            !raw.contains("operator"),
            "the foreign value must be gone (never a placeholder): {raw}"
        );
    }

    #[test]
    fn seed_helper_writes_valid_logged_in_state() {
        let dir = tempfile::tempdir().unwrap();
        seed_logged_in_at(dir.path(), "acct-seed").unwrap();
        match load_from(&dir.path().join("auth.json")) {
            LoadedAuth::Present(s) => {
                assert!(s.logged_in_ever);
                assert_eq!(s.account_id, "acct-seed");
                assert_eq!(
                    local_gate(&LoadedAuth::Present(s), now_unix_ns()),
                    LocalGate::ProceedValidSession
                );
            }
            other => panic!("expected Present, got {other:?}"),
        }
    }

    #[test]
    fn atomic_write_overwrites_existing() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        write_to(&path, &state(true, 1)).unwrap();
        write_to(&path, &state(true, 999)).unwrap();
        match load_from(&path) {
            LoadedAuth::Present(s) => assert_eq!(s.expires_at_ns, 999),
            other => panic!("expected Present, got {other:?}"),
        }
        // No leftover staging temp (the unique-per-pid sibling is renamed away).
        assert!(!super::unique_temp_path(&path).exists());
    }

    // --- secret-write hardening ----------

    #[cfg(unix)]
    #[test]
    fn pre_existing_0666_temp_does_not_leak_perms() {
        // A co-user plants the staging temp as a 0666 regular file. The write
        // must UNLINK it and re-create 0600 (create(true) would have kept 0666).
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        let tmp = super::unique_temp_path(&path);
        std::fs::write(&tmp, b"planted").unwrap();
        std::fs::set_permissions(&tmp, std::fs::Permissions::from_mode(0o666)).unwrap();
        write_to(&path, &state(true, 7)).unwrap();
        let mode = std::fs::metadata(&path).unwrap().permissions().mode() & 0o777;
        assert_eq!(
            mode, 0o600,
            "the planted 0666 temp must not leak into auth.json"
        );
        assert!(!tmp.exists(), "the staging temp is renamed away");
    }

    #[cfg(unix)]
    #[test]
    fn symlinked_temp_is_not_followed() {
        // A co-user points the staging temp at a victim file. The write must
        // UNLINK the symlink (not follow it) — the victim stays untouched and
        // auth.json lands 0600 at its real path.
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        let victim = dir.path().join("victim.txt");
        std::fs::write(&victim, b"do-not-touch").unwrap();
        let tmp = super::unique_temp_path(&path);
        std::os::unix::fs::symlink(&victim, &tmp).unwrap();
        write_to(&path, &state(true, 9)).unwrap();
        assert_eq!(
            std::fs::read(&victim).unwrap(),
            b"do-not-touch",
            "the symlink target must NOT be written through"
        );
        let mode = std::fs::metadata(&path).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode, 0o600);
        // The real auth.json parses back (it did not clobber the victim).
        assert!(matches!(load_from(&path), LoadedAuth::Present(_)));
    }

    #[cfg(unix)]
    #[test]
    fn fresh_config_dir_is_created_0700() {
        // The first login on a fresh box creates ~/.cerulion at 0700 (not the
        // umask default 0755) so the credential filenames are not world-listable.
        use std::os::unix::fs::PermissionsExt;
        let base = tempfile::tempdir().unwrap();
        let cfg_dir = base.path().join("nested").join(".cerulion");
        let path = cfg_dir.join("auth.json");
        write_to(&path, &state(true, 3)).unwrap();
        let mode = std::fs::metadata(&cfg_dir).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode, 0o700, "~/.cerulion must be owner-only (0700)");
    }

    // --- the store lock shared with Studio -----------------------------------

    /// The lock is a CROSS-PROCESS protocol, so its spelling and location are the
    /// contract: Studio's `desk_login::STORE_LOCK_FILE` is this string, beside
    /// `auth.json`. A rename on either side serializes nothing while looking like
    /// it does, which is why this is pinned by name rather than left to the
    /// implementation.
    #[test]
    fn the_store_lock_is_studios_file_beside_the_store() {
        assert_eq!(STORE_LOCK_FILE, ".studio-auth.lock");
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        assert_eq!(store_lock_path(&path), dir.path().join(".studio-auth.lock"));
    }

    /// A write mints the lock file owner-only, in the owner-only config dir — the
    /// lock must not be the one artifact in `~/.cerulion` that a co-user can open.
    #[cfg(unix)]
    #[test]
    fn writing_the_store_mints_the_lock_file_0600() {
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(".cerulion").join("auth.json");
        write_to(&path, &state(true, 5)).unwrap();
        let lock = store_lock_path(&path);
        assert!(lock.exists(), "the write must leave the lock file behind");
        let mode = std::fs::metadata(&lock).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode, 0o600);
    }

    /// The point of the whole change: while a PEER holds the lock (Studio
    /// rotating a refreshed session, a second `cerulion`), this process does not
    /// publish over it — it waits, then REPORTS. A peer's open file description
    /// is what a second process presents, so holding one here is the same
    /// contention.
    ///
    /// Anti-tautology: the identical call succeeds the moment the peer lets go,
    /// so the refusal is about the lock rather than about the path.
    #[cfg(unix)]
    #[test]
    fn a_peer_holding_the_lock_makes_a_write_report_rather_than_publish() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        write_to(&path, &state(true, 1)).unwrap();

        let peer = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(store_lock_path(&path))
            .unwrap();
        super::flock_exclusive_nonblocking(&peer).unwrap();

        let err = write_to(&path, &state(true, 2)).expect_err("a held lock must refuse the write");
        assert_eq!(err.kind(), std::io::ErrorKind::WouldBlock);
        let LoadedAuth::Present(kept) = load_from(&path) else {
            panic!("the store must still parse");
        };
        assert_eq!(
            kept.expires_at_ns, 1,
            "the refused write must not have touched the store"
        );

        drop(peer);
        write_to(&path, &state(true, 2)).expect("the released lock must let the write through");
        let LoadedAuth::Present(now) = load_from(&path) else {
            panic!("the store must still parse");
        };
        assert_eq!(now.expires_at_ns, 2);
    }

    /// A read-modify-write wraps `load` + `write_to` in ONE lock, and `write_to`
    /// locks too — so the nested acquisition must run under the lock it is already
    /// inside instead of contending with itself. Without the re-entrancy the
    /// inner acquisition spends `LOCK_WAIT` and then returns `WouldBlock`, so
    /// the nested write's own result reports a self-deadlock; what the ledger
    /// assertion adds is that the shortcut was AVAILABLE, read directly rather
    /// than inferred from how long the call took.
    #[test]
    fn a_nested_lock_runs_under_the_one_it_is_already_inside() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        write_to(&path, &state(true, 11)).unwrap();

        let mut ledger_inside = false;
        let carried = with_store_lock(&path, || {
            ledger_inside = HELD_LOCKS.with(|h| h.borrow().contains(&store_lock_path(&path)));
            let prior = load_from(&path).state().map(|s| s.expires_at_ns);
            write_to(&path, &state(true, prior.unwrap_or(0) + 1)).map(|()| prior)
        })
        .expect("a nested write must not contend with its own lock");
        assert_eq!(carried, Some(11));
        assert!(
            ledger_inside,
            "the outer lock must be on this thread's ledger, which is the only \
             thing that lets the nested acquisition run under it"
        );
        assert!(
            HELD_LOCKS.with(|h| h.borrow().is_empty()),
            "the outer call must clear the ledger on the way out"
        );
        let LoadedAuth::Present(now) = load_from(&path) else {
            panic!("the store must still parse");
        };
        assert_eq!(now.expires_at_ns, 12);
        // …and the lock is free again once the outer call returns.
        write_to(&path, &state(true, 13)).expect("the lock must be released on the way out");
    }

    /// A panic inside the critical section must not leave the store locked for
    /// the rest of the process — the next writer would then wait a second and
    /// refuse, turning one failure into every subsequent one.
    ///
    /// The release is OBSERVED, never timed. A ceiling on a path that should
    /// not wait at all can only be tripped by a machine that is slower than
    /// expected, and it observes nothing a stalled writer's own error does not
    /// already report. Both halves of the release are read directly instead:
    /// this thread's re-entrancy ledger must be empty, and a SIBLING thread
    /// (which cannot use that ledger) must be able to take the kernel lock.
    #[test]
    fn a_panicking_write_releases_the_lock() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            with_store_lock(&path, || -> std::io::Result<()> {
                panic!("the write blew up");
            })
        }));
        assert!(panicked.is_err(), "the panic must propagate");

        // Half one: the re-entrancy ledger is THREAD-LOCAL, so an entry the
        // unwind failed to remove would let every later write on this thread
        // take the re-entrant shortcut and skip the kernel lock entirely. A
        // same-thread writer therefore cannot see that leak at all.
        assert!(
            HELD_LOCKS.with(|h| h.borrow().is_empty()),
            "the unwind must clear this thread's re-entrancy ledger"
        );

        // Half two: a SIBLING thread has no ledger entry, so it has to take the
        // flock for real. A lock the unwind left held answers WouldBlock and
        // this write reports it.
        let sibling_path = path.clone();
        std::thread::spawn(move || write_to(&sibling_path, &state(true, 4)))
            .join()
            .expect("the sibling writer thread")
            .expect("the next writer must not be locked out");
        let LoadedAuth::Present(now) = load_from(&path) else {
            panic!("the store must still parse");
        };
        assert_eq!(now.expires_at_ns, 4, "the sibling's write landed intact");
    }

    /// Two threads are two writers and must serialize through the KERNEL (the
    /// re-entrancy guard is thread-scoped, so it must not let a sibling in). The
    /// holder sleeps inside the section, and the second writer's wait is the
    /// evidence it was excluded.
    #[cfg(unix)]
    #[test]
    fn a_second_thread_waits_for_the_lock_rather_than_interleaving() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("auth.json");
        let held = std::time::Duration::from_millis(150);

        let inside = std::sync::Arc::new(std::sync::Barrier::new(2));
        let holder_path = path.clone();
        let holder_inside = std::sync::Arc::clone(&inside);
        let holder = std::thread::spawn(move || {
            with_store_lock(&holder_path, || {
                holder_inside.wait();
                std::thread::sleep(held);
                write_to(&holder_path, &state(true, 1))
            })
            .expect("the holder writes");
        });

        inside.wait();
        let started = std::time::Instant::now();
        write_to(&path, &state(true, 2)).expect("the waiter publishes once the holder is done");
        let waited = started.elapsed();
        holder.join().expect("the holder thread");

        assert!(
            waited >= held / 2,
            "the second writer must have waited for the lock, not walked in (waited {waited:?})"
        );
        let LoadedAuth::Present(now) = load_from(&path) else {
            panic!("the store must still parse");
        };
        assert_eq!(
            now.expires_at_ns, 2,
            "the waiter's write is the last one and must be intact"
        );
    }
}

#[cfg(test)]
mod debug_redaction_tests {
    use super::*;
    #[test]
    fn auth_state_debug_keeps_public_facts_and_redacts_both_bearers() {
        let state = AuthState {
            account_id: "account-public-vector".into(),
            session_token: "session-sensitive-vector-7f".into(),
            refresh_token: "refresh-sensitive-vector-9b".into(),
            expires_at_ns: 42,
            logged_in_ever: true,
            role: Some(MachineRole::Desk),
        };
        assert_eq!(format!("{state:?}"), "AuthState { account_id: \"account-public-vector\", session_token: \"[REDACTED]\", refresh_token: \"[REDACTED]\", expires_at_ns: 42, logged_in_ever: true, role: Some(Desk) }");
        let pretty = format!("{state:#?}");
        assert!(pretty.contains("[REDACTED]"));
        assert!(!pretty.contains("session-sensitive-vector-7f"));
        assert!(!pretty.contains("refresh-sensitive-vector-9b"));
    }
}
