// SPDX-License-Identifier: AGPL-3.0-only
//! The iroh WAN plane's robot REGISTRY + the dual-plane PICKER.
//!
//! netd owns THE one mirror per `(robot, topic)` for BOTH transport planes — zenoh
//! on the LAN, iroh over the WAN (the decided dual-plane, one-endpoint model). A
//! robot is ONE identity reachable over two transports; this module answers the two
//! questions that fold the iroh engine in cleanly:
//!
//! 1. **Where does the WAN dial config come from?** [`WanRegistry`] maps a robot
//!    NAME (the same string a demand carries, e.g. `ubuntu`) to its iroh
//!    [`EndpointId`] + optional direct LAN socket addresses, alongside the ONE desk
//!    device seed (its iroh identity — the pairing/trust key the robot access-lists)
//!    and the relay posture. It is built at startup from the environment
//!    ([`WanRegistry::from_env`]) or dependency-injected in tests.
//!
//! 2. **Which plane serves a given demand?** [`pick_plane`] is a PURE, deterministic
//!    decision: a robot present in the WAN registry (the operator configured a WAN
//!    iroh endpoint for it) is reached over IROH; every other robot defaults to the
//!    ZENOH LAN plane. The legacy router keeps a static registry and re-picks on
//!    release. The account controller owns separate mutable membership and must
//!    pin each demand's route until release; registry failures never select LAN.
//!
//! # The plane-picker's scope
//!
//! The design ideal is "LAN-zenoh-IF-reachable, else iroh" — a preference for
//! the LAN even when a robot ALSO has a WAN endpoint. Realizing the "if-reachable"
//! probe needs a live zenoh-announce query surface, which the picker does not use.
//! So the picker is the faithful realization of "else iroh": a robot the
//! operator gave a WAN endpoint for IS the not-on-the-LAN / use-iroh case; anything
//! else stays on the zenoh default. The automatic LAN-reachability
//! preference (prefer zenoh even for a WAN-registered robot when it is also present
//! on the LAN) is not implemented.
//!
//! Everything here is `#[cfg(feature = "wan")]` — `wan` is DEFAULT-ON (the shipped
//! netd includes the iroh WAN plane; `--no-default-features` is the lean opt-out).
//! netd is kept out of `default-members` so a plain `cargo build` stays iroh-free
//! (see `Cargo.toml`).

use std::collections::HashMap;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock, RwLock};

use cerulion_pairing::verify::OwnerCertificatePresentationWire;

use cerulion_link::{EndpointId, RelayConfig};

use crate::registry::TopicKey;

/// Env var naming the desk's WAN robots: a `;`-separated list of
/// `name=eid[@ip:port[,ip:port...]]` entries (see [`parse_robots_spec`]). A robot
/// listed here is reachable over the iroh WAN plane; the demand's `robot` string
/// must match a `name`. Unset / empty ⇒ no WAN robots (every demand routes to the
/// zenoh LAN plane — netd behaves exactly as a LAN-only daemon).
pub const WAN_ROBOTS_ENV: &str = "CERULION_NETD_WAN_ROBOTS";

/// Env var: path to the desk's 32-byte ed25519 device key file (the desk's iroh
/// identity + the pairing key the robot access-lists). Unset ⇒ an EPHEMERAL desk
/// key is generated — which an un-paired robot REFUSES (the iroh dial is refused
/// LOUDLY at demand time), so a real WAN deployment points this at
/// `~/.cerulion/desk.key` (the SAME key `cerulion pair` creates). Mirrors
/// `cerulion-connectd connect --key-file`.
pub const DESK_KEY_ENV: &str = "CERULION_NETD_DESK_KEY";

/// Env var: set to a non-empty value to DISABLE all iroh relays (LAN direct-dial
/// only — for a locked-down / air-gapped desk, or tests). Mirrors
/// `cerulion-connectd connect --relay-disabled`. When unset, the relay posture is
/// resolved from [`RELAY_URL_ENV`] (n0 public relays by default) —
/// [`RelayConfig::resolve_from_env`].
pub const RELAY_DISABLED_ENV: &str = "CERULION_NETD_RELAY_DISABLED";

/// Env var: explicit path to the desk's cached device cert — the
/// `base64url(postcard(SignedDeviceCert))` blob `cerulion login` writes to
/// `~/.cerulion/device.cert`. netd resolves the logged-in cloud ACCOUNT
/// from it (bound to the desk device key per A3's proof-of-possession model) so a WAN
/// dial can present an account-bound identity. Unset ⇒ the cert is taken from the
/// SIBLING `device.cert` next to the [`DESK_KEY_ENV`] key file (both live in
/// `~/.cerulion`); with an ephemeral desk key (no key file) there is no cert and no
/// account (WAN dials present none — never bricks; A5 gates the WAN plane on it).
pub const DEVICE_CERT_ENV: &str = "CERULION_NETD_DEVICE_CERT";

/// Env var: explicit path to the desk's revocation-epoch cache DIRECTORY — where an
/// online sync (Studio / the account page / a CLI) writes
/// `<robot>.epoch` = `base64url(postcard(EpochSyncWire))` after fetching
/// `GET /v1/robots/{id}/access`. netd PUSHES the cached epoch to each robot
/// it dials, which is how a revocation reaches the robot at all (the desk-push
/// sync, by design). Unset ⇒ the `epochs/` directory NEXT TO the [`DESK_KEY_ENV`] key
/// file (both under `~/.cerulion`); with an ephemeral desk key (no key file) there is
/// no cache and no push — dials proceed exactly as before (never bricks).
///
/// A compile-time ALIAS of the SHARED [`cerulion_pairing::verify::EPOCH_DIR_ENV`] —
/// NOT a netd-scoped name. The epoch cache is a desk-wide artifact location that
/// `cerulion connect` and the account-page cache writer resolve too, and an override
/// only ONE path honored would relocate the cache for that path alone: the writer
/// reports "cached", the other reader reports "nothing cached", and revocations stop
/// travelling with no symptom at either end.
pub const EPOCH_DIR_ENV: &str = cerulion_pairing::verify::EPOCH_DIR_ENV;

/// The relay-URL env var the WAN plane reads (via
/// [`RelayConfig::resolve_from_env`]) when [`RELAY_DISABLED_ENV`] is not engaged. It
/// is the SHARED `cerulion_link` relay env (`cerulion connect` reads the same one),
/// re-exported here as a compile-time ALIAS of [`cerulion_link::CERULION_RELAY_URL_ENV`]
/// so the two CANNOT drift — and so the lean-build misuse warn (`main.rs`) can
/// enumerate the FULL set of env vars the wan build consumes without linking
/// `cerulion_link`.
pub const RELAY_URL_ENV: &str = cerulion_link::CERULION_RELAY_URL_ENV;

/// One WAN robot's iroh dial parameters. The desk seed + relay are shared across
/// all robots (they are the DESK's identity + posture), so they live on the
/// [`WanRegistry`], not here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WanRobot {
    /// The robot's iroh endpoint id (== its 32-byte device key's public half — the
    /// pinned identity the dial authenticates against, preserving the pairing's
    /// trust machinery: a mis-keyed robot fails the TLS peer-identity check).
    pub eid: EndpointId,
    /// Optional direct `ip:port` socket addresses for a LAN direct-dial. Empty ⇒
    /// resolve the eid via relay/discovery (requires relays enabled).
    pub direct_addrs: Vec<SocketAddr>,
}

/// Which transport plane serves a demand. The daemon's
/// [`MirrorPlane`](crate::mirror::MirrorPlane) implementation (`DualMirrorPlane`)
/// delegates to the zenoh
/// [`GatewayMirrorPlane`](crate::mirror::GatewayMirrorPlane) or the iroh
/// `IrohMirrorPlane` per this choice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Plane {
    /// The zenoh LAN plane (`register_ingress_topic`) — the default for any robot
    /// with no configured WAN endpoint.
    Zenoh,
    /// The iroh WAN plane (dial `cerulion/wire/1` + re-inject) — for a robot present
    /// in the [`WanRegistry`].
    Iroh,
}

/// The raw env INPUTS the desk-path resolvers need, captured by
/// [`WanRegistry::from_env`] and consumed LAZILY at first use.
///
/// Capturing the (cheap) env strings at `from_env` — while deferring the (not cheap)
/// filesystem + crypto work they feed — keeps the env-read TIMING at construction
/// (a later `set_var` cannot retroactively change a registry's answer) while taking the
/// work itself off the daemon's boot critical path. See [`WanRegistry::account`] /
/// [`WanRegistry::epoch_cache_path`] for the deferral contract.
///
/// `pub` (with named fields) so a TEST can hand a dependency-injected registry the
/// SAME inputs [`WanRegistry::from_env`] captures — see
/// [`with_desk_paths`](WanRegistry::with_desk_paths) — and therefore exercise the REAL
/// lazy resolvers (`resolve_account_from`, the shared epoch-dir rule) without touching
/// the process environment. A named struct rather than three positional `Option`s
/// because two of them are identically typed and a swapped pair would be silent.
#[derive(Debug, Clone, Default)]
pub struct DeskPathInputs {
    /// [`DESK_KEY_ENV`]'s value (the desk key file both sibling artifacts hang off).
    pub key_file: Option<PathBuf>,
    /// [`DEVICE_CERT_ENV`]'s value (an explicit device-cert path override).
    pub device_cert_env: Option<String>,
    /// [`EPOCH_DIR_ENV`]'s value (the DESK-WIDE epoch-cache directory override).
    pub epoch_dir_env: Option<String>,
}

/// The desk's WAN robot registry: robot name → its iroh dial params, plus the ONE
/// desk device seed + relay posture shared across every WAN dial, and
/// the logged-in account the dial presents.
///
/// # Boot cost
///
/// The two DESK-PATH facts — the logged-in account (read + key-matched from the cached
/// device cert) and the revocation-epoch cache directory — are resolved LAZILY, on
/// first use, NOT at [`from_env`]. Both are DIAL-time facts: a netd that never dials
/// over the WAN never needs either, and `NetdClient::connect_or_spawn` waits on this
/// daemon's socket, so every millisecond spent resolving them before the socket binds
/// is a millisecond a desk consumer blocks. Their `OnceLock`s make the resolution
/// happen exactly once per process (the same cardinality a boot-time resolution has),
/// and every log line they emit reads the same as it would at boot — just
/// emitted at first use rather than at startup.
///
/// "First use" is a REAL production point for both, not a hypothetical one (without
/// a production caller of [`account`](WanRegistry::account), its
/// device-cert diagnostics would move to *never*):
///
/// | fact | production first-use point |
/// |---|---|
/// | [`account`](WanRegistry::account) | `IrohMirrorPlane::dial_robot` (via `resolve_dial_account`), before that dial's network I/O |
/// | [`epoch_cache_path`](WanRegistry::epoch_cache_path) | `IrohMirrorPlane::push_epoch_inner`, on the same dial |
///
/// [`account_resolved`](WanRegistry::account_resolved) is the observable that pins the
/// account half of that table (`IrohMirrorPlane::desk_account_resolved` delegates to
/// it); the pins are `from_env_leaves_the_account_cell_unresolved_until_first_use` here and
/// `the_desk_account_is_resolved_at_the_dial_not_before` in `wan_plane_iroh_test.rs`.
///
/// [`from_env`]: WanRegistry::from_env
#[derive(Clone)]
pub struct WanRegistry {
    robots: Arc<RwLock<HashMap<String, WanRobot>>>,
    desk_seed: [u8; 32],
    relay: RelayConfig,
    /// The logged-in cloud account (32 bytes) the WAN dial presents, bound to
    /// `desk_seed`'s device key per A3's PoP model. `None` when no
    /// device cert binds this desk (never logged in / ephemeral key).
    ///
    /// LAZY: resolved from the cached device cert on the first
    /// [`account`](WanRegistry::account) call, from the [`DeskPathInputs`]
    /// [`from_env`] captured; pre-seeded by [`with_account`] on the DI path.
    ///
    /// [`from_env`]: WanRegistry::from_env
    /// [`with_account`]: WanRegistry::with_account
    account: OnceLock<Option<[u8; 32]>>,
    /// The desk's revocation-epoch cache directory (`<robot>.epoch` files).
    /// `None` when this desk has no cache location (ephemeral key + no explicit env) ⇒
    /// no epoch is pushed and every dial behaves exactly as before.
    ///
    /// LAZY: resolved through the SHARED
    /// [`cerulion_wireclient::epoch::resolve_epoch_dir`] rule on the first
    /// [`epoch_cache_path`](WanRegistry::epoch_cache_path) call (i.e. the first WAN
    /// dial), from the [`DeskPathInputs`] [`from_env`] captured; pre-seeded by
    /// [`with_epoch_dir`] on the DI path.
    ///
    /// [`from_env`]: WanRegistry::from_env
    /// [`with_epoch_dir`]: WanRegistry::with_epoch_dir
    epoch_dir: OnceLock<Option<PathBuf>>,
    /// The captured env inputs the two lazy resolvers read. `None` on the
    /// [`new`](WanRegistry::new) (dependency-injection) path — which resolves both
    /// facts to `None` exactly as it did before, so a DI registry never touches the
    /// ambient environment or filesystem.
    desk_paths: Option<DeskPathInputs>,
    owner_certificate: Arc<RwLock<Option<Arc<OwnerCertificatePresentationWire>>>>,
}

impl std::fmt::Debug for WanRegistry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let public =
            cerulion_pairing::client::DeviceIdentity::from_seed(&self.desk_seed).public_key();
        f.write_str("WanRegistry { desk_seed: \"[REDACTED]\", desk_public_key: \"")?;
        for byte in public.0 {
            write!(f, "{byte:02x}")?;
        }
        f.write_str("\" }")
    }
}

impl std::fmt::Debug for WanRegistry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let public =
            cerulion_pairing::client::DeviceIdentity::from_seed(&self.desk_seed).public_key();
        f.write_str("WanRegistry { desk_seed: \"[REDACTED]\", desk_public_key: \"")?;
        for byte in public.0 {
            write!(f, "{byte:02x}")?;
        }
        f.write_str("\" }")
    }
}

impl WanRegistry {
    /// Build a registry directly (the test / dependency-injection entry). The
    /// account defaults to `None` — set it with [`with_account`] (the production
    /// [`from_env`] resolves it from the cached device cert).
    ///
    /// A registry built this way carries NO captured env inputs, so its lazy
    /// account / epoch-dir resolvers yield `None` without reading the ambient
    /// environment or filesystem — the DI path stays hermetic.
    ///
    /// [`with_account`]: WanRegistry::with_account
    /// [`from_env`]: WanRegistry::from_env
    pub fn new(robots: HashMap<String, WanRobot>, desk_seed: [u8; 32], relay: RelayConfig) -> Self {
        Self {
            robots: Arc::new(RwLock::new(robots)),
            desk_seed,
            relay,
            account: OnceLock::new(),
            epoch_dir: OnceLock::new(),
            desk_paths: None,
            owner_certificate: Arc::new(RwLock::new(None)),
        }
    }

    /// Replace the login proof used for first-demand owner admission.
    ///
    /// Clears the previous proof before validating its replacement. The account
    /// and transport key are immutable for this registry; changing either requires
    /// a new registry and plane. The caller must invalidate active connections
    /// before changing the login account.
    pub fn replace_owner_certificate(
        &self,
        proof: Option<Arc<OwnerCertificatePresentationWire>>,
    ) -> Result<(), String> {
        let mut current = self
            .owner_certificate
            .write()
            .map_err(|_| "owner certificate state is poisoned".to_string())?;
        *current = None;
        if let Some(proof) = proof {
            let device = &proof.device_cert.cert;
            let key =
                cerulion_pairing::client::DeviceIdentity::from_seed(&self.desk_seed).public_key();
            if device.device_key != key || Some(device.account.0) != self.account() {
                return Err(
                    "owner certificate does not match the current desk key and account".into(),
                );
            }
            *current = Some(proof);
        }
        Ok(())
    }

    /// Snapshot the injected login proof without reading ambient files.
    pub fn owner_certificate(
        &self,
    ) -> Result<Option<Arc<OwnerCertificatePresentationWire>>, String> {
        let proof = self
            .owner_certificate
            .read()
            .map_err(|_| "owner certificate state is poisoned".to_string())?
            .clone();
        if let Some(proof) = &proof {
            let key =
                cerulion_pairing::client::DeviceIdentity::from_seed(&self.desk_seed).public_key();
            if proof.device_cert.cert.device_key != key
                || Some(proof.device_cert.cert.account.0) != self.account()
            {
                return Err(
                    "owner certificate does not match the current desk key and account".into(),
                );
            }
        }
        Ok(proof)
    }

    /// Attach the logged-in account the WAN dial presents — bound to the
    /// desk device key. Tests inject one directly; the production [`from_env`] resolves
    /// it lazily from the cached device cert instead.
    ///
    /// PRE-SEEDS the lazy cell, so an injected account is authoritative and the
    /// device-cert resolver never runs.
    ///
    /// [`from_env`]: WanRegistry::from_env
    pub fn with_account(mut self, account: Option<[u8; 32]>) -> Self {
        self.account = OnceLock::new();
        let _ = self.account.set(account);
        self
    }

    /// Attach the desk's revocation-epoch cache directory — where each
    /// dialed robot's `<robot>.epoch` artifact is read from before the push. Tests
    /// inject a tempdir directly; the production [`from_env`] resolves it lazily from
    /// the env / the sibling `epochs/` dir instead.
    ///
    /// PRE-SEEDS the lazy cell, so an injected directory is authoritative and the
    /// env-backed resolver never runs.
    ///
    /// [`from_env`]: WanRegistry::from_env
    pub fn with_epoch_dir(mut self, epoch_dir: Option<PathBuf>) -> Self {
        self.epoch_dir = OnceLock::new();
        let _ = self.epoch_dir.set(epoch_dir);
        self
    }

    /// Attach the DESK-PATH env inputs a [`from_env`] registry captures, WITHOUT
    /// pre-seeding either lazy cell — so a dependency-injected registry runs the REAL
    /// resolvers (`resolve_account_from`'s cert read + I1 key match; the shared
    /// epoch-dir rule) on first use, exactly as production does.
    ///
    /// This is the seam [`with_account`] / [`with_epoch_dir`] deliberately are NOT:
    /// those two pre-seed an ANSWER (the resolver never runs), which is right for a
    /// test that only needs the carried value, and wrong for a test that needs to prove
    /// the resolution HAPPENS — and happens at the first dial, not before. Pairs with
    /// [`account_resolved`](WanRegistry::account_resolved).
    ///
    /// # Composition
    ///
    /// This builder PANICS if either lazy cell has already been pre-seeded by
    /// [`with_account`] / [`with_epoch_dir`]. The two orders are not equivalent —
    /// `with_desk_paths(..).with_epoch_dir(..)` keeps the injected directory, while
    /// `with_epoch_dir(..).with_desk_paths(..)` would DISCARD it and re-arm the
    /// env/filesystem-backed resolver — and a DI seam whose whole purpose is hermetic,
    /// parallel-safe tests must not resolve that ambiguity silently in favour of the
    /// ambient environment. (The other reading treats the reset as a feature: "a later
    /// `with_desk_paths` overrides an earlier pre-seed rather than being silently
    /// ignored". Both readings are order-dependent; refusing the ambiguous composition
    /// is the strict one.) Call `with_desk_paths` FIRST, or not at all.
    ///
    /// The reset is still performed (the cells must be unresolved for the real
    /// resolvers to run); the panic only rejects the case where the reset would
    /// DESTROY a caller's injected answer.
    ///
    /// # `desk_seed` and `inputs.key_file` are deliberately independent
    ///
    /// Production's [`from_env`] DERIVES `desk_seed` from `desk_paths.key_file`
    /// (`resolve_desk_seed`), but this seam does NOT re-derive it, and must not: here
    /// `key_file` serves as the PATH ANCHOR for the sibling `device.cert` (and the
    /// shared epoch-dir rule) and is explicitly allowed not to exist, whereas
    /// `resolve_desk_seed` would fail on a missing file. The caller therefore supplies
    /// the seed the cert was minted for. This encodes nothing production cannot reach:
    /// writing that seed into `key_file` yields the identical `(seed, key_file)` pair,
    /// and a MISMATCHED pair produces a NEGATIVE — `resolve_account_from` runs the real
    /// I1 key match against `desk_seed` and answers `None` — never a false resolution.
    ///
    /// [`from_env`]: WanRegistry::from_env
    /// [`with_account`]: WanRegistry::with_account
    /// [`with_epoch_dir`]: WanRegistry::with_epoch_dir
    pub fn with_desk_paths(mut self, inputs: DeskPathInputs) -> Self {
        assert!(
            self.account.get().is_none(),
            "WanRegistry::with_desk_paths would DISCARD the account pre-seeded by \
             with_account (the two are alternative resolution strategies) — call \
             with_desk_paths FIRST, or drop the with_account call"
        );
        assert!(
            self.epoch_dir.get().is_none(),
            "WanRegistry::with_desk_paths would DISCARD the epoch dir pre-seeded by \
             with_epoch_dir and re-arm the env-backed resolver (a hermetic test would \
             silently start reading the ambient environment) — call with_desk_paths \
             FIRST, or drop the with_epoch_dir call"
        );
        self.account = OnceLock::new();
        self.epoch_dir = OnceLock::new();
        self.desk_paths = Some(inputs);
        self
    }

    /// The desk's revocation-epoch cache DIRECTORY, resolving it on first use
    /// through the SHARED [`cerulion_wireclient::epoch::resolve_epoch_dir`]
    /// rule — the same rule, the same inputs, the same one-per-process breadcrumb as
    /// a boot-time resolution; only the TIMING differs.
    fn epoch_dir(&self) -> Option<&Path> {
        self.epoch_dir
            .get_or_init(|| {
                let dir = self.desk_paths.as_ref().and_then(|inputs| {
                    cerulion_wireclient::epoch::resolve_epoch_dir(
                        inputs.epoch_dir_env.as_deref(),
                        inputs.key_file.as_deref(),
                    )
                });
                // The breadcrumb — deferred to the first
                // WAN dial (the only place the answer can matter). A DI registry
                // (`desk_paths == None`) stays silent: it was never env-resolved, so
                // reporting on env resolution would be a lie.
                if self.desk_paths.is_some() {
                    log_epoch_dir_resolution(dir.as_deref());
                }
                dir
            })
            .as_deref()
    }

    /// The desk's revocation-epoch cache path for `robot`, or `None` when this desk
    /// has no cache directory (⇒ nothing to push; the dial proceeds unchanged).
    ///
    /// `robot` is TRIMMED to match [`WanRegistry::get`] / [`WanRegistry::is_wan_robot`]:
    /// without it a key carrying surrounding whitespace would dial fine but resolve
    /// `"go2 .epoch"` and silently push nothing.
    ///
    /// The join itself is the SHARED keying rule
    /// ([`cerulion_wireclient::epoch::epoch_cache_path`] → the one function in
    /// `cerulion_pairing` the WRITER and the `cerulion connect` session also call), so
    /// this path and the one the other desk path looks for can never diverge.
    ///
    /// The DIRECTORY half is resolved on the FIRST call (the first WAN dial),
    /// not at [`from_env`](WanRegistry::from_env) — see [`WanRegistry`]'s boot-cost
    /// note. The resolution rule and its breadcrumb are unchanged.
    pub fn epoch_cache_path(&self, robot: &str) -> Option<PathBuf> {
        self.epoch_dir()
            .map(|d| cerulion_wireclient::epoch::epoch_cache_path(d, robot.trim()))
    }

    /// The WAN dial params for `robot`, if it has a configured WAN endpoint.
    pub fn get(&self, robot: &str) -> Result<Option<WanRobot>, String> {
        self.robots
            .read()
            .map(|robots| robots.get(robot.trim()).cloned())
            .map_err(|_| "WAN robot membership lock is poisoned".into())
    }

    /// Whether `robot` is reachable over the iroh WAN plane (has a configured WAN
    /// endpoint). The [`pick_plane`] selector.
    pub fn is_wan_robot(&self, robot: &str) -> Result<bool, String> {
        self.robots
            .read()
            .map(|robots| robots.contains_key(robot.trim()))
            .map_err(|_| "WAN robot membership lock is poisoned".into())
    }

    pub(crate) fn membership_snapshot(&self) -> Result<HashMap<String, WanRobot>, String> {
        self.robots
            .read()
            .map(|robots| robots.clone())
            .map_err(|_| "WAN robot membership lock is poisoned".into())
    }

    /// Called only by the account plane while holding its Iroh state lock.
    pub(crate) fn replace_membership(
        &self,
        robots: HashMap<String, WanRobot>,
    ) -> Result<(), String> {
        let mut current = self
            .robots
            .write()
            .map_err(|_| "WAN robot membership lock is poisoned".to_owned())?;
        *current = robots;
        Ok(())
    }

    /// The desk's 32-byte ed25519 device seed (its iroh identity for every WAN dial).
    pub fn desk_seed(&self) -> [u8; 32] {
        self.desk_seed
    }

    /// The relay posture for every WAN dial.
    pub fn relay(&self) -> &RelayConfig {
        &self.relay
    }

    /// The logged-in cloud account (32 bytes) the WAN dial presents, bound to the
    /// desk device key, or `None` when no device cert binds this desk. An owner
    /// certificate injected through [`Self::replace_owner_certificate`] must
    /// match this account before the first-demand admission path can use it.
    ///
    /// Resolved on the FIRST call, not at
    /// [`from_env`](WanRegistry::from_env) — see [`WanRegistry`]'s boot-cost note. The
    /// cert read, the I1 key match, and every classifying log line are unchanged; only
    /// the TIMING moved (the answer is a fact about a WAN DIAL, and a netd that never
    /// dials never needs it).
    ///
    /// The production first caller is `IrohMirrorPlane::dial_robot` (through its
    /// `resolve_dial_account`), which runs BEFORE the dial's network I/O so a
    /// device-cert misconfiguration is classified above the catalog-gate refusal it
    /// causes. **Keep it that way**: if this method ever loses its production caller
    /// again, the classifying `info!`/`warn!` lines below become unreachable and a
    /// stale/foreign cert goes silent. [`account_resolved`](WanRegistry::account_resolved)
    /// is the observable that pins it.
    pub fn account(&self) -> Option<[u8; 32]> {
        *self.account.get_or_init(|| {
            self.desk_paths
                .as_ref()
                .and_then(|inputs| resolve_account_from(&self.desk_seed, inputs))
        })
    }

    /// Whether this registry's lazy account cell has been RESOLVED yet — i.e. whether
    /// [`account`](WanRegistry::account) has been called on it (or an answer was
    /// pre-seeded by [`with_account`](WanRegistry::with_account)).
    ///
    /// This is the deferral's observable (Principle #3): the cell is the thing
    /// that holds the cost — the device-cert read, the postcard decode and the ed25519
    /// derivation all live behind it — so its state, not a caller-side flag, is what
    /// says whether that work has happened in this process. `false` on a freshly
    /// [`from_env`](WanRegistry::from_env)-built registry is the boot-cost contract; it
    /// flipping only at the first WAN dial is what makes "first use" a real production
    /// point rather than "never".
    ///
    /// It reports the CELL, not the cert: a `with_account`-seeded (DI) registry reads
    /// `true` having read no cert, and a registry whose `desk_paths` are `None` reads
    /// `true` after a resolution that short-circuited to `None`. Use it to pin WHEN the
    /// resolution happens; use [`account`](WanRegistry::account)'s value to pin WHAT it
    /// resolved.
    pub fn account_resolved(&self) -> bool {
        self.account.get().is_some()
    }

    /// The number of configured WAN robots (diagnostics / tests).
    pub fn robot_count(&self) -> Result<usize, String> {
        self.robots
            .read()
            .map(|robots| robots.len())
            .map_err(|_| "WAN robot membership lock is poisoned".into())
    }

    /// Resolve the WAN registry from the process environment:
    /// - robots ← [`WAN_ROBOTS_ENV`] (parsed by [`parse_robots_spec`]; unset ⇒ none);
    /// - desk seed ← [`DESK_KEY_ENV`] (a 32-byte key file; unset ⇒ ephemeral —
    ///   which an un-paired robot refuses, so a real deployment sets it);
    /// - relay ← [`RELAY_DISABLED_ENV`] + `CERULION_RELAY_URL`.
    ///
    /// A malformed robots spec / unreadable key / bad relay URL is a LOUD `Err`
    /// (`main.rs` surfaces it) — never a silent empty registry.
    ///
    /// The two DESK-PATH facts (the logged-in account behind the cached device
    /// cert, and the revocation-epoch cache directory) are NOT resolved here. Their env
    /// inputs are captured (cheap) and the resolution itself — a file read + an ed25519
    /// key derivation for the account, the shared directory rule for the epoch cache —
    /// is deferred to first use. `cerulion-netd` is spawned by a desk consumer that
    /// BLOCKS until this daemon's control socket binds, so boot-path work is latency a
    /// user pays for; both facts are only ever needed by a WAN dial.
    pub fn from_env() -> Result<Self, String> {
        // Distinguish "unset" (normal → no WAN robots) from "set but non-UTF-8" (a
        // set-but-garbled config must never silently no-op — the silent-inertness
        // class). `Err(_)` would collapse both into an empty registry.
        let robots = match std::env::var(WAN_ROBOTS_ENV) {
            Ok(raw) => parse_robots_spec(&raw)?,
            Err(std::env::VarError::NotPresent) => HashMap::new(),
            Err(std::env::VarError::NotUnicode(v)) => {
                return Err(format!(
                    "{WAN_ROBOTS_ENV} is set but contains non-UTF-8 bytes ({v:?}) — refusing to \
                     silently route all demands to zenoh; fix or unset it"
                ));
            }
        };
        let key_file: Option<PathBuf> = std::env::var(DESK_KEY_ENV)
            .ok()
            .filter(|s| !s.is_empty())
            .map(PathBuf::from);
        let desk_seed = cerulion_wireclient::config::resolve_desk_seed(key_file.as_deref())
            .map_err(|e| format!("{DESK_KEY_ENV}: {e}"))?;
        let relay_disabled = std::env::var(RELAY_DISABLED_ENV)
            .ok()
            .is_some_and(|v| !v.trim().is_empty());
        let relay = RelayConfig::resolve_from_env(relay_disabled, None)
            .map_err(|e| format!("relay config ({RELAY_URL_ENV}): {e}"))?;
        // Capture (don't resolve) the desk-path env inputs. The account's cert
        // read + key derivation and the epoch-cache directory rule run on first use —
        // see the fn docs. `env::var` is nanoseconds; the work behind it is not.
        let desk_paths = DeskPathInputs {
            key_file: key_file.clone(),
            device_cert_env: std::env::var(DEVICE_CERT_ENV).ok(),
            epoch_dir_env: std::env::var(EPOCH_DIR_ENV).ok(),
        };
        if !robots.is_empty() {
            // Report WHICH device-cert file the first dial will
            // read, not a `device_cert_configured` bool. That bool was pure path
            // composition — true whenever `DEVICE_CERT_ENV` was set OR a key file
            // existed, i.e. exactly `!ephemeral_desk_key || <env set>` — so it carried
            // no information the same log line did not already have, while its NAME
            // implied a cert had been found (nothing is read here; the cert may not
            // exist). The PATH is a genuine boot-time fact, costs no I/O, and is the
            // thing an operator debugging "why does the robot refuse me" needs.
            // `resolve_account_from`'s three classifying lines still report the ACTUAL
            // outcome at the first dial.
            let device_cert = device_cert_path_from(
                desk_paths.device_cert_env.as_deref(),
                desk_paths.key_file.as_deref(),
            )
            .map(|p| p.display().to_string())
            .unwrap_or_else(|| format!("(none — ephemeral desk key and {DEVICE_CERT_ENV} unset)"));
            tracing::info!(
                wan_robots = robots.len(),
                relay_disabled,
                ephemeral_desk_key = key_file_absent(),
                %device_cert,
                "cerulion-netd: iroh WAN plane configured — {WAN_ROBOTS_ENV} lists {} robot(s)",
                robots.len()
            );
        }
        Ok(Self {
            robots: Arc::new(RwLock::new(robots)),
            desk_seed,
            relay,
            account: OnceLock::new(),
            epoch_dir: OnceLock::new(),
            desk_paths: Some(desk_paths),
            owner_certificate: Arc::new(RwLock::new(None)),
        })
    }
}

/// The epoch-cache-directory breadcrumb, emitted once per process when the
/// directory is first resolved (the resolution is off the boot path, at the
/// first WAN dial).
fn log_epoch_dir_resolution(dir: Option<&Path>) {
    match dir {
        Some(d) => tracing::info!(
            epoch_dir = %d.display(),
            "cerulion-netd: WAN dials PUSH the desk's cached revocation epoch when \
             a <robot>.epoch artifact is present"
        ),
        None => tracing::info!(
            "cerulion-netd: WAN dials push NO revocation epoch — no cache directory \
             (ephemeral desk key + {EPOCH_DIR_ENV} unset). Point {DESK_KEY_ENV} at \
             ~/.cerulion/desk.key so the sibling epochs/ dir resolves."
        ),
    }
}

/// Resolve the desk device cert path: an explicit `explicit`
/// ([`DEVICE_CERT_ENV`]) wins; otherwise the SIBLING `device.cert` next to the desk
/// `key_file`. `None` when neither is available (an ephemeral desk key with no
/// explicit cert env — there is no bound account). Pure — oracle-tested.
fn device_cert_path_from(explicit: Option<&str>, key_file: Option<&Path>) -> Option<PathBuf> {
    if let Some(p) = explicit.map(str::trim).filter(|s| !s.is_empty()) {
        return Some(PathBuf::from(p));
    }
    key_file.map(|kf| match kf.parent() {
        Some(dir) if !dir.as_os_str().is_empty() => dir.join("device.cert"),
        // A bare `desk.key` (no parent component) ⇒ `device.cert` in the same
        // (current) directory.
        _ => PathBuf::from("device.cert"),
    })
}

// The epoch-cache DIRECTORY resolver is NOT duplicated here. netd resolves it through
// the SHARED `cerulion_wireclient::epoch::resolve_epoch_dir` (homed in
// `cerulion_pairing` so the iroh-free cache WRITER reaches it too) — deliberately
// parallel to [`device_cert_path_from`] in shape, but shared in CODE, because the
// `cerulion connect` path resolves the same directory and a second implementation
// would silently disagree the moment [`EPOCH_DIR_ENV`] relocated the cache.

/// Resolve the logged-in account for the WAN registry from the cached device cert:
/// read [`DEVICE_CERT_ENV`] / the sibling cert, verify it binds
/// `desk_seed`'s device key, and return the account bytes. NEVER bricks — every
/// non-resolution path yields `None` with a LOUD log classifying WHY (info for the
/// benign no-cert cases, warn for a real misconfiguration), so a WAN dial presents
/// no account rather than a WRONG one.
///
/// Takes the env inputs [`WanRegistry::from_env`] captured rather than
/// reading the environment itself, so this — the file read + postcard decode + ed25519
/// key derivation — runs at first use (off the daemon's boot critical path) while the
/// env is still read at `from_env` time. Every log line is unchanged.
fn resolve_account_from(desk_seed: &[u8; 32], inputs: &DeskPathInputs) -> Option<[u8; 32]> {
    let explicit = inputs.device_cert_env.clone();
    let Some(cert_path) = device_cert_path_from(explicit.as_deref(), inputs.key_file.as_deref())
    else {
        tracing::info!(
            "cerulion-netd: WAN dials present NO account — no device cert path (ephemeral desk key \
             + {DEVICE_CERT_ENV} unset). Point {DESK_KEY_ENV} at ~/.cerulion/desk.key and run \
             `cerulion login` to bind one."
        );
        return None;
    };
    match cerulion_wireclient::config::resolve_desk_account(desk_seed, &cert_path) {
        Ok(Some(account)) => {
            tracing::info!(
                cert = %cert_path.display(),
                "cerulion-netd: WAN dials present the logged-in account bound to the desk device key"
            );
            Some(account)
        }
        Ok(None) => {
            tracing::info!(
                cert = %cert_path.display(),
                "cerulion-netd: no device cert cached — WAN dials present NO account until \
                 `cerulion login` writes one (A5 gates the WAN plane on it)"
            );
            None
        }
        Err(e) => {
            tracing::warn!(
                cert = %cert_path.display(),
                error = %e,
                "cerulion-netd: could not resolve the desk account from the cached device cert (a \
                 stale/foreign cert, or {DESK_KEY_ENV} points at a different key than the one that \
                 logged in) — WAN dials present NO account rather than a WRONG one"
            );
            None
        }
    }
}

/// Whether the desk key env is absent (⇒ an ephemeral, un-paired desk identity —
/// surfaced in the config breadcrumb so a WAN demand that is refused for being
/// unpaired is diagnosable).
fn key_file_absent() -> bool {
    std::env::var(DESK_KEY_ENV)
        .ok()
        .filter(|s| !s.is_empty())
        .is_none()
}

/// Parse a [`WAN_ROBOTS_ENV`] spec: a `;`-separated list of
/// `name=eid[@ip:port[,ip:port...]]` entries. Whitespace around entries / `=` /
/// addresses is tolerated; empty entries (a trailing `;`) are dropped. The `eid` is
/// validated as a real ed25519 endpoint id (via the SAME
/// [`cerulion_wireclient::config::parse_eid`] the `connect` CLI uses), and each addr
/// as an `ip:port`. A duplicate name, a missing `=`, a blank name, an empty eid, or
/// a bad eid/addr is a LOUD `Err` naming the offending entry. Pure — oracle-tested.
pub fn parse_robots_spec(raw: &str) -> Result<HashMap<String, WanRobot>, String> {
    let mut robots = HashMap::new();
    for entry in raw.split(';') {
        let entry = entry.trim();
        if entry.is_empty() {
            continue;
        }
        let (name, rest) = entry.split_once('=').ok_or_else(|| {
            format!("WAN robot entry '{entry}' is missing '=' (expected name=eid[@ip:port,...])")
        })?;
        let name = name.trim();
        if name.is_empty() {
            return Err(format!("WAN robot entry '{entry}' has an empty name"));
        }
        // Split the value into `eid` and the optional `@addr,addr,...` tail.
        let (eid_str, addr_str) = match rest.split_once('@') {
            Some((eid, addrs)) => (eid.trim(), Some(addrs)),
            None => (rest.trim(), None),
        };
        if eid_str.is_empty() {
            return Err(format!("WAN robot '{name}' has an empty endpoint id"));
        }
        let eid = cerulion_wireclient::config::parse_eid(eid_str)
            .map_err(|e| format!("WAN robot '{name}': {e}"))?;
        let direct_addrs = match addr_str {
            None => Vec::new(),
            Some(addrs) => {
                let list: Vec<String> = addrs
                    .split(',')
                    .map(str::trim)
                    .filter(|s| !s.is_empty())
                    .map(str::to_string)
                    .collect();
                cerulion_wireclient::config::parse_addrs(&list)
                    .map_err(|e| format!("WAN robot '{name}': {e}"))?
            }
        };
        if robots
            .insert(name.to_string(), WanRobot { eid, direct_addrs })
            .is_some()
        {
            return Err(format!(
                "WAN robot '{name}' is listed more than once in {WAN_ROBOTS_ENV}"
            ));
        }
    }
    Ok(robots)
}

/// The dual-plane PICKER: a robot present in the WAN registry is reached over IROH;
/// every other robot defaults to the ZENOH LAN plane. Pure + deterministic over
/// `(key, registry snapshot)`. The legacy router's registry is static; mutable
/// account membership requires the controller to retain per-topic assignments.
/// Oracle-tested.
pub fn pick_plane(key: &TopicKey, registry: &WanRegistry) -> Result<Plane, String> {
    if registry.is_wan_robot(&key.robot)? {
        Ok(Plane::Iroh)
    } else {
        Ok(Plane::Zenoh)
    }
}

/// Refusal used when remoted owns this serving machine's single WAN endpoint.
pub const SERVING_MACHINE_WAN_REFUSAL: &str = "this machine serves the WAN plane; consuming other robots over the WAN from a serving machine is not supported in this version; use the LAN plane";

enum WanConsumer {
    Desk(Box<crate::iroh_plane::IrohMirrorPlane>),
    ServingMachine,
}

/// The DUAL-plane mirror: composes the zenoh LAN plane
/// ([`GatewayMirrorPlane`](crate::mirror::GatewayMirrorPlane)) and the iroh WAN plane
/// ([`IrohMirrorPlane`](crate::iroh_plane::IrohMirrorPlane)) behind ONE
/// [`MirrorPlane`](crate::mirror::MirrorPlane), routing every demand via [`pick_plane`] so netd owns THE one
/// mirror per `(robot, topic)` across both — the decided dual-plane, one-endpoint
/// model. Both planes share the desk's ONE [`TransportManager`](cerulion_core::TransportManager)
/// (the single SHM mirror target), and the picker guarantees exactly ONE plane
/// serves a given topic, so the single-writer mirror slot is never double-claimed.
///
/// `release_mirror` RE-PICKS rather than remembering an assignment: the WAN registry
/// is static for netd's lifetime, so a key's release routes to the SAME plane its
/// ensure did — the ensure/release pair is symmetric with zero bookkeeping (and no
/// assignment map to drift out of sync with the daemon's registry).
pub struct DualMirrorPlane {
    zenoh: crate::mirror::GatewayMirrorPlane,
    iroh: WanConsumer,
    registry: std::sync::Arc<WanRegistry>,
}

impl DualMirrorPlane {
    /// Compose the two planes over a shared WAN `registry` (the picker's input). The
    /// two planes must already wrap the SAME desk transport manager (the one mirror
    /// target).
    pub fn new(
        zenoh: crate::mirror::GatewayMirrorPlane,
        iroh: crate::iroh_plane::IrohMirrorPlane,
        registry: std::sync::Arc<WanRegistry>,
    ) -> Self {
        Self {
            zenoh,
            // hot-path-alloc-ok: daemon construction stores this client once.
            iroh: WanConsumer::Desk(Box::new(iroh)),
            registry,
        }
    }

    /// Build a serving machine's mirror router without a WAN client runtime.
    ///
    /// The sibling remoted owns this machine's endpoint. Registered WAN targets
    /// remain WAN targets and fail explicitly at demand, rather than falling back
    /// to LAN. No IrohMirrorPlane, Tokio runtime, or outgoing endpoint is created.
    pub fn for_serving_machine(
        zenoh: crate::mirror::GatewayMirrorPlane,
        registry: Arc<WanRegistry>,
    ) -> Self {
        Self {
            zenoh,
            iroh: WanConsumer::ServingMachine,
            registry,
        }
    }

    /// The plane `key` routes to (diagnostics / tests) — the [`pick_plane`] result.
    pub fn plane_for(&self, key: &TopicKey) -> Result<Plane, String> {
        pick_plane(key, &self.registry)
    }
}

impl crate::mirror::MirrorPlane for DualMirrorPlane {
    /// The picker is pure over `(key, registry)` and this registry is fixed for the
    /// daemon's lifetime, so the plane it names here is the one that ensured the
    /// mirror, not a fresh guess: the same argument that lets `release_mirror`
    /// re-pick instead of remembering.
    fn serving_plane(&self, key: &TopicKey) -> Option<crate::protocol::ServingPlane> {
        match pick_plane(key, &self.registry).ok()? {
            Plane::Zenoh => Some(crate::protocol::ServingPlane::Zenoh),
            Plane::Iroh => Some(crate::protocol::ServingPlane::Iroh),
        }
    }

    fn ensure_mirror(
        &self,
        key: &TopicKey,
        schema_hash: u64,
    ) -> Result<(), crate::mirror::MirrorError> {
        match pick_plane(key, &self.registry).map_err(|reason| {
            crate::mirror::MirrorError::Iroh {
                key: key.clone(),
                reason,
            }
        })? {
            Plane::Zenoh => self.zenoh.ensure_mirror(key, schema_hash),
            Plane::Iroh => match &self.iroh {
                WanConsumer::Desk(iroh) => iroh.ensure_mirror(key, schema_hash),
                WanConsumer::ServingMachine => Err(crate::mirror::MirrorError::Iroh {
                    key: key.clone(),
                    reason: SERVING_MACHINE_WAN_REFUSAL.to_owned(),
                }),
            },
        }
    }

    fn release_mirror(&self, key: &TopicKey) -> crate::mirror::MirrorRelease {
        match pick_plane(key, &self.registry) {
            Ok(Plane::Zenoh) => self.zenoh.release_mirror(key),
            Ok(Plane::Iroh) => match &self.iroh {
                WanConsumer::Desk(iroh) => iroh.release_mirror(key),
                WanConsumer::ServingMachine => crate::mirror::MirrorRelease::Retired,
            },
            Err(error) => {
                tracing::error!(robot = %key.robot, topic = %key.topic, error = %error,
                    "could not select the mirror plane for release");
                crate::mirror::MirrorRelease::Lingering
            }
        }
    }
}

#[cfg(test)]
#[path = "wan_membership_tests.rs"]
mod membership_tests;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_debug_redacts_the_seed_and_only_exposes_the_public_key() {
        // RFC 8032 test vector 1 provides an independent public-key oracle.
        let seed_hex = "9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60";
        let seed: [u8; 32] = hex::decode(seed_hex).unwrap().try_into().unwrap();
        let public_hex = "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a";
        let registry = WanRegistry::new(HashMap::new(), seed, RelayConfig::Disabled);
        let compact = format!("{registry:?}");
        assert_eq!(
            compact,
            format!(
                "WanRegistry {{ desk_seed: \"[REDACTED]\", desk_public_key: \"{public_hex}\" }}"
            )
        );
        for rendered in [compact, format!("{registry:#?}")] {
            assert!(rendered.contains("[REDACTED]"));
            assert!(rendered.contains(public_hex));
            assert!(!rendered.contains(seed_hex));
            assert!(!rendered.contains(&format!("{seed:?}")));
            assert!(!rendered.contains(&format!("{seed:#?}")));
        }
    }

    /// A real ed25519 endpoint id (hex) derived from a seed — the SAME derivation
    /// the desk + robot use, so `parse_eid` accepts it. NOT a self-compare: we
    /// assert the parsed registry eid equals this independently-derived key.
    fn real_eid_hex(seed: [u8; 32]) -> (String, [u8; 32]) {
        let public = cerulion_pairing::client::DeviceIdentity::from_seed(&seed)
            .public_key()
            .0;
        (hex::encode(public), public)
    }

    #[test]
    fn parse_robots_spec_single_robot_with_addrs() {
        let (hex, pub_bytes) = real_eid_hex([3u8; 32]);
        let spec = format!("ubuntu={hex}@192.168.1.20:7842,192.168.1.20:7843");
        let robots = parse_robots_spec(&spec).expect("parses");
        assert_eq!(robots.len(), 1);
        let r = robots.get("ubuntu").expect("ubuntu present");
        assert_eq!(r.eid.as_bytes(), &pub_bytes, "eid == the derived key");
        assert_eq!(
            r.direct_addrs,
            vec![
                "192.168.1.20:7842".parse::<SocketAddr>().unwrap(),
                "192.168.1.20:7843".parse::<SocketAddr>().unwrap(),
            ]
        );
    }

    #[test]
    fn parse_robots_spec_multiple_and_no_addrs_and_whitespace() {
        let (hex_a, pub_a) = real_eid_hex([4u8; 32]);
        let (hex_b, pub_b) = real_eid_hex([5u8; 32]);
        // Whitespace around entries + `=`, a trailing `;`, one robot with no addrs.
        let spec = format!("  go2 = {hex_a}@10.0.0.9:7683 ;  ubuntu={hex_b} ; ");
        let robots = parse_robots_spec(&spec).expect("parses");
        assert_eq!(robots.len(), 2);
        assert_eq!(robots.get("go2").unwrap().eid.as_bytes(), &pub_a);
        assert_eq!(
            robots.get("go2").unwrap().direct_addrs,
            vec!["10.0.0.9:7683".parse::<SocketAddr>().unwrap()]
        );
        let ubuntu = robots.get("ubuntu").unwrap();
        assert_eq!(ubuntu.eid.as_bytes(), &pub_b);
        assert!(
            ubuntu.direct_addrs.is_empty(),
            "no @addr ⇒ relay/discovery dial"
        );
    }

    #[test]
    fn parse_robots_spec_empty_and_blank_yield_no_robots() {
        assert!(parse_robots_spec("").expect("empty ok").is_empty());
        assert!(parse_robots_spec("   ").expect("blank ok").is_empty());
        assert!(parse_robots_spec(" ; ; ").expect("only-seps ok").is_empty());
    }

    #[test]
    fn parse_robots_spec_rejects_malformed_entries_loudly() {
        let (hex, _) = real_eid_hex([6u8; 32]);
        // Missing '='.
        let err = parse_robots_spec(&format!("ubuntu{hex}")).unwrap_err();
        assert!(err.contains("missing '='"), "err: {err}");
        // Empty name.
        let err = parse_robots_spec(&format!("={hex}")).unwrap_err();
        assert!(err.contains("empty name"), "err: {err}");
        // Empty eid.
        let err = parse_robots_spec("ubuntu=").unwrap_err();
        assert!(err.contains("empty endpoint id"), "err: {err}");
        // Bad eid (not 64 hex chars → not a valid key).
        let err = parse_robots_spec("ubuntu=deadbeef").unwrap_err();
        assert!(err.contains("ubuntu"), "names the robot: {err}");
        // Bad addr.
        let err = parse_robots_spec(&format!("ubuntu={hex}@not-an-addr")).unwrap_err();
        assert!(
            err.contains("ubuntu") && err.contains("not-an-addr"),
            "err: {err}"
        );
        // Duplicate name.
        let dup = format!("ubuntu={hex};ubuntu={hex}");
        let err = parse_robots_spec(&dup).unwrap_err();
        assert!(err.contains("more than once"), "err: {err}");
    }

    /// `pick_plane`: a WAN-registered robot → Iroh; anything else → Zenoh (the
    /// default). Hand oracle.
    #[test]
    fn pick_plane_routes_wan_robots_to_iroh_others_to_zenoh() {
        let (hex, _) = real_eid_hex([7u8; 32]);
        let robots = parse_robots_spec(&format!("ubuntu={hex}")).unwrap();
        let registry = WanRegistry::new(robots, [1u8; 32], RelayConfig::Disabled);

        // The WAN-registered robot → iroh.
        assert_eq!(
            pick_plane(&TopicKey::new("ubuntu", "/utlidar/robot_odom"), &registry).unwrap(),
            Plane::Iroh
        );
        // A robot NOT in the registry → the zenoh LAN default.
        assert_eq!(
            pick_plane(&TopicKey::new("lan-bot", "/tf"), &registry).unwrap(),
            Plane::Zenoh
        );
        // Topic does not matter — the robot identity selects the plane.
        assert_eq!(
            pick_plane(&TopicKey::new("ubuntu", "/tf"), &registry).unwrap(),
            Plane::Iroh
        );
        assert!(registry.is_wan_robot("ubuntu").unwrap());
        assert!(!registry.is_wan_robot("lan-bot").unwrap());
        // An empty registry routes everything to zenoh (a LAN-only daemon).
        let empty = WanRegistry::new(HashMap::new(), [1u8; 32], RelayConfig::Disabled);
        assert_eq!(
            pick_plane(&TopicKey::new("ubuntu", "/tf"), &empty).unwrap(),
            Plane::Zenoh
        );
        assert_eq!(empty.robot_count().unwrap(), 0);
    }

    /// `RELAY_URL_ENV` is a compile-time alias of `cerulion_link`'s const (so they
    /// cannot drift) AND is the exact string the wan build reads.
    #[test]
    fn relay_url_env_aliases_the_link_const() {
        assert_eq!(RELAY_URL_ENV, cerulion_link::CERULION_RELAY_URL_ENV);
        assert_eq!(RELAY_URL_ENV, "CERULION_RELAY_URL");
    }

    /// A SET-but-non-UTF-8 `CERULION_NETD_WAN_ROBOTS` is a LOUD error, NEVER a silent
    /// empty registry (the silent-inertness class). Unix-only: a
    /// non-UTF-8 env value needs raw bytes. Mutates process env → the ONLY wan test
    /// touching `WAN_ROBOTS_ENV`, with an RAII removal guard (net.rs convention).
    #[cfg(unix)]
    #[test]
    fn from_env_rejects_non_utf8_wan_robots_loudly() {
        use std::ffi::OsStr;
        use std::os::unix::ffi::OsStrExt;
        let _lock = env_lock();
        struct Guard;
        impl Drop for Guard {
            fn drop(&mut self) {
                std::env::remove_var(WAN_ROBOTS_ENV);
            }
        }
        let _g = Guard;
        // "f" followed by invalid UTF-8 bytes — `from_env` errs at the WAN_ROBOTS
        // match (its first read) BEFORE touching the other env vars.
        std::env::set_var(WAN_ROBOTS_ENV, OsStr::from_bytes(&[0x66, 0xFF, 0xFE]));
        let err = WanRegistry::from_env()
            .expect_err("non-UTF-8 WAN_ROBOTS must error, not silently yield an empty registry");
        assert!(
            err.contains(WAN_ROBOTS_ENV) && err.contains("non-UTF-8"),
            "the error names the var + the cause: {err}"
        );
    }

    // --- The WAN dial config carries the logged-in account -----------------

    /// The env-mutating `from_env` tests share the process env → serialize them (and
    /// the non-UTF-8 test above).
    ///
    /// This is a thin delegate to the CRATE-WIDE
    /// [`crate::test_env::env_lock`]. A file-local mutex would serialize
    /// `wan.rs` against itself and against nothing else — while `hygiene.rs` and
    /// `net.rs` mutate the same process env in the SAME test binary.
    /// See `test_env`'s module docs for why var-name disjointness is not a defense.
    fn env_lock() -> std::sync::MutexGuard<'static, ()> {
        crate::test_env::env_lock()
    }

    /// Snapshots the named env vars on construction and RESTORES them on drop (so a
    /// test can freely set/remove them without leaking to siblings).
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

    /// The desk device public key derived from `seed` (the SAME derivation
    /// `resolve_desk_account` uses).
    fn desk_public_key(seed: &[u8; 32]) -> [u8; 32] {
        cerulion_pairing::client::DeviceIdentity::from_seed(seed)
            .public_key()
            .0
    }

    /// A cached cert blob (`base64url(postcard(SignedDeviceCert))`, the shape
    /// `cerulion login` writes) binding `device_key` → `account`. A zero signature —
    /// the account resolver does NOT verify the signature chain (I1 key match only).
    fn make_cert_b64(device_key: [u8; 32], account: [u8; 32]) -> String {
        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        use cerulion_pairing::format::{
            AccountId, DeviceCert, PrincipalKind, PublicKey, Scope, Signature, SignedDeviceCert,
            Validity, FORMAT_VERSION,
        };
        let cert = DeviceCert {
            version: FORMAT_VERSION,
            device_key: PublicKey(device_key),
            account: AccountId(account),
            principal_kind: PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity: Validity {
                not_before_ns: 0,
                not_after_ns: u64::MAX,
            },
            issued_at_ns: 0,
            issuer_key: PublicKey([0xAB; 32]),
        };
        let signed = SignedDeviceCert {
            cert,
            signature: Signature([0u8; 64]),
        };
        URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&signed).unwrap())
    }

    /// The netd HALF of the three-party
    /// agreement pin, driven through netd's REAL PRODUCTION env read
    /// ([`WanRegistry::from_env`], which is what the daemon actually runs), so it must
    /// equal — byte for byte — the literal oracle the other two halves are anchored to
    /// (`cerulion_wireclient::epoch::both_desk_paths_resolve_one_cache_path` for
    /// `cerulion connect`, `cerulion_cli_engine::account_cmd::
    /// the_writer_resolves_the_shared_epoch_cache_path` for the cache WRITER).
    ///
    /// Were netd to honor a netd-SCOPED `CERULION_NETD_EPOCH_DIR` that the connect path
    /// ignores, with each path hand-rolling its own join, relocating the cache would make an
    /// epoch cached by one path invisible to the other, silently. The two paths share the
    /// code; a test that INJECTED the resolved directory into
    /// the registry and asserted the registry re-joined it would be tautological in its
    /// loop and never execute netd's own env read at all. This drives `from_env`.
    ///
    /// Env-mutating → serialized on the file-local [`env_lock`].
    #[test]
    fn netd_from_env_resolves_the_shared_epoch_cache_path() {
        use cerulion_wireclient::epoch::{epoch_cache_path, resolve_epoch_dir, EPOCH_DIR_ENV};
        let _lock = env_lock();
        let _snap = EnvSnapshot::take(&[
            DESK_KEY_ENV,
            DEVICE_CERT_ENV,
            WAN_ROBOTS_ENV,
            RELAY_DISABLED_ENV,
            RELAY_URL_ENV,
            EPOCH_DIR_ENV,
        ]);

        // netd's env name IS the shared desk-wide one (not a netd-scoped alias).
        assert_eq!(EPOCH_DIR_ENV, super::EPOCH_DIR_ENV);
        assert_eq!(EPOCH_DIR_ENV, "CERULION_EPOCH_DIR");

        // Hermetic + network-free: no WAN robots, relays disabled, no device cert.
        std::env::remove_var(WAN_ROBOTS_ENV);
        std::env::remove_var(DEVICE_CERT_ENV);
        std::env::set_var(RELAY_DISABLED_ENV, "1");

        let dir = tempfile::tempdir().unwrap();
        let cerulion = dir.path().join(".cerulion");
        std::fs::create_dir_all(&cerulion).unwrap();
        let key_path = cerulion.join("desk.key");
        std::fs::write(&key_path, [3u8; 32]).unwrap();

        // (A) A desk key + NO override ⇒ the sibling `epochs/` dir next to it, i.e. the
        //     shared `<desk dir>/epochs/<robot>.epoch` — hand-composed, so a change to
        //     the shared rule cannot silently drag this side along.
        std::env::set_var(DESK_KEY_ENV, &key_path);
        std::env::remove_var(EPOCH_DIR_ENV);
        let expected = cerulion.join("epochs").join("go2.epoch");
        assert_eq!(
            WanRegistry::from_env().unwrap().epoch_cache_path("go2"),
            Some(expected.clone()),
            "netd's PRODUCTION env read must resolve the shared cache path"
        );

        // (B) The DESK-WIDE override relocates netd's lookup too (ONE var,
        //     honored by all three parties, never a netd-private one).
        let relocated = dir.path().join("elsewhere").join("epochs");
        std::env::set_var(EPOCH_DIR_ENV, &relocated);
        assert_eq!(
            WanRegistry::from_env().unwrap().epoch_cache_path("go2"),
            Some(relocated.join("go2.epoch"))
        );
        // …and it is the SAME composition the other two parties perform.
        assert_eq!(
            WanRegistry::from_env().unwrap().epoch_cache_path("go2"),
            resolve_epoch_dir(relocated.to_str(), Some(&key_path))
                .map(|d| epoch_cache_path(&d, "go2")),
        );

        // (C) A blank override is NOT an override.
        std::env::set_var(EPOCH_DIR_ENV, "   ");
        assert_eq!(
            WanRegistry::from_env().unwrap().epoch_cache_path("go2"),
            Some(expected)
        );

        // (D) An EPHEMERAL desk key (no DESK_KEY_ENV) + no override ⇒ no cache dir at
        //     all ⇒ None (nothing pushed; the dial is byte-unchanged).
        std::env::remove_var(DESK_KEY_ENV);
        std::env::remove_var(EPOCH_DIR_ENV);
        assert_eq!(
            WanRegistry::from_env().unwrap().epoch_cache_path("go2"),
            None
        );

        // (E) …but an explicit override alone is enough, even with an ephemeral key.
        std::env::set_var(EPOCH_DIR_ENV, &relocated);
        assert_eq!(
            WanRegistry::from_env().unwrap().epoch_cache_path("go2"),
            Some(relocated.join("go2.epoch"))
        );
    }

    /// The `robot` argument is TRIMMED and the JOIN is the shared keying rule — the
    /// registry-level half (the env resolution is
    /// [`netd_from_env_resolves_the_shared_epoch_cache_path`]). Hand oracles, no env.
    #[test]
    fn netd_epoch_cache_path_trims_and_uses_the_shared_join() {
        use cerulion_wireclient::epoch::epoch_cache_path;
        let epochs = PathBuf::from("/desk/.cerulion/epochs");
        let reg = WanRegistry::new(HashMap::new(), [7u8; 32], RelayConfig::Disabled)
            .with_epoch_dir(Some(epochs.clone()));
        // Hand oracle for the shared convention (NOT read back off the helper).
        assert_eq!(
            reg.epoch_cache_path("go2"),
            Some(epochs.join("go2.epoch")),
            "netd must place its lookup where the writer put the artifact"
        );
        // A key carrying surrounding whitespace dials fine, so the lookup must trim it
        // too (or it would resolve `go2 .epoch` and silently push nothing).
        assert_eq!(
            reg.epoch_cache_path("  go2  "),
            Some(epochs.join("go2.epoch"))
        );
        // The traversal-safety rewrite applies here as well.
        assert_eq!(
            reg.epoch_cache_path("../../etc/passwd"),
            Some(epochs.join(".._.._etc_passwd.epoch"))
        );
        // And it is the SAME shared join, not a coincidentally equal hand-roll.
        assert_eq!(
            reg.epoch_cache_path("go2"),
            Some(epoch_cache_path(&epochs, "go2"))
        );
        // No cache dir ⇒ None (nothing pushed, the dial is byte-unchanged).
        let reg = WanRegistry::new(HashMap::new(), [7u8; 32], RelayConfig::Disabled);
        assert_eq!(reg.epoch_cache_path("go2"), None);
    }

    /// `device_cert_path_from`: an explicit path wins; otherwise the SIBLING
    /// `device.cert` next to the desk key; an ephemeral key with no explicit env ⇒
    /// `None`. Hand oracle — no env, no files.
    #[test]
    fn device_cert_path_from_oracle() {
        // Explicit override wins (even with a key file present).
        assert_eq!(
            device_cert_path_from(
                Some("/etc/cerulion/device.cert"),
                Some(Path::new("/home/x/.cerulion/desk.key"))
            ),
            Some(PathBuf::from("/etc/cerulion/device.cert"))
        );
        // A blank/whitespace explicit value is ignored → falls to the sibling.
        assert_eq!(
            device_cert_path_from(Some("  "), Some(Path::new("/home/x/.cerulion/desk.key"))),
            Some(PathBuf::from("/home/x/.cerulion/device.cert"))
        );
        // No explicit, a key file ⇒ the sibling device.cert.
        assert_eq!(
            device_cert_path_from(None, Some(Path::new("/home/x/.cerulion/desk.key"))),
            Some(PathBuf::from("/home/x/.cerulion/device.cert"))
        );
        // A bare key filename (no parent) ⇒ device.cert in the current directory.
        assert_eq!(
            device_cert_path_from(None, Some(Path::new("desk.key"))),
            Some(PathBuf::from("device.cert"))
        );
        // Ephemeral desk key (no key file) + no explicit env ⇒ no cert, no account.
        assert_eq!(device_cert_path_from(None, None), None);
    }

    /// `from_env` resolves the WAN account from the cached device cert:
    /// the sibling cert bound to the desk key is CARRIED; a foreign / absent cert
    /// yields NO account (never a wrong one); an explicit `CERULION_NETD_DEVICE_CERT`
    /// wins; an ephemeral desk key carries none. Folded into ONE body so the ordered
    /// env mutations never race (env is process-global). Hand oracles.
    #[test]
    fn from_env_resolves_the_account_from_the_device_cert() {
        let _lock = env_lock();
        let _snap = EnvSnapshot::take(&[
            DESK_KEY_ENV,
            DEVICE_CERT_ENV,
            WAN_ROBOTS_ENV,
            RELAY_DISABLED_ENV,
            RELAY_URL_ENV,
        ]);
        // Hermetic + network-free: no WAN robots, relays disabled.
        std::env::remove_var(WAN_ROBOTS_ENV);
        std::env::set_var(RELAY_DISABLED_ENV, "1");

        let dir = tempfile::tempdir().unwrap();
        let seed = [3u8; 32];
        let key_path = dir.path().join("desk.key");
        std::fs::write(&key_path, seed).unwrap();
        let cert_path = dir.path().join("device.cert");
        let account = [0x55u8; 32];

        std::env::set_var(DESK_KEY_ENV, &key_path);
        std::env::remove_var(DEVICE_CERT_ENV);

        // (A) The sibling device.cert bound to THIS desk key ⇒ the account is carried.
        std::fs::write(&cert_path, make_cert_b64(desk_public_key(&seed), account)).unwrap();
        assert_eq!(
            WanRegistry::from_env().unwrap().account(),
            Some(account),
            "the sibling device.cert account is carried onto the WAN dial config"
        );

        // (B) A FOREIGN cert (bound to a different key — e.g. the desk-key env points
        //     at a different key than the one that logged in) ⇒ NO account (never a
        //     wrong one), and the registry still builds (never bricks).
        std::fs::write(
            &cert_path,
            make_cert_b64(desk_public_key(&[9u8; 32]), account),
        )
        .unwrap();
        assert_eq!(WanRegistry::from_env().unwrap().account(), None);

        // (C) Absent cert ⇒ NO account (never fabricated).
        std::fs::remove_file(&cert_path).unwrap();
        assert_eq!(WanRegistry::from_env().unwrap().account(), None);

        // (D) An explicit CERULION_NETD_DEVICE_CERT at a NON-sibling path wins.
        let explicit = dir.path().join("explicit").join("cert.b64");
        std::fs::create_dir_all(explicit.parent().unwrap()).unwrap();
        std::fs::write(&explicit, make_cert_b64(desk_public_key(&seed), account)).unwrap();
        std::env::set_var(DEVICE_CERT_ENV, &explicit);
        assert_eq!(
            WanRegistry::from_env().unwrap().account(),
            Some(account),
            "an explicit CERULION_NETD_DEVICE_CERT overrides the sibling"
        );
        std::env::remove_var(DEVICE_CERT_ENV);

        // (E) An ephemeral desk key (no DESK_KEY_ENV) + no explicit cert ⇒ no account.
        std::env::remove_var(DESK_KEY_ENV);
        assert_eq!(WanRegistry::from_env().unwrap().account(), None);
    }

    /// The OBSERVABLE half of the deferral:
    /// [`WanRegistry::account_resolved`] must report the cell's real state against the
    /// REAL production constructor.
    ///
    /// The deferral BEHAVIOR is already owned by
    /// [`account_resolution_is_deferred_off_the_boot_path_to_first_use`] below (a
    /// cert-appears-after-build oracle that needs no observable). What this adds is that
    /// `account_resolved` — which `IrohMirrorPlane::desk_account_resolved` delegates to,
    /// and on which the loopback-dial e2e
    /// (`wan_plane_iroh_test::the_desk_account_is_resolved_at_the_dial_not_before`) rests
    /// — is WIRED to that same cell rather than to some caller-side flag. Without this,
    /// a broken observable would leave the e2e passing vacuously.
    ///
    /// MUTATIONS: an eager `resolve_account_from` in `from_env` fails arm (1); an
    /// observable hardcoded to `false` (or read off a flag nothing sets) fails arm (2);
    /// a resolver that never reads the cert fails arm (2)'s value assert.
    #[test]
    fn from_env_leaves_the_account_cell_unresolved_until_first_use() {
        let _lock = env_lock();
        let _snap = EnvSnapshot::take(&[
            DESK_KEY_ENV,
            DEVICE_CERT_ENV,
            WAN_ROBOTS_ENV,
            RELAY_DISABLED_ENV,
            RELAY_URL_ENV,
        ]);
        std::env::remove_var(WAN_ROBOTS_ENV);
        std::env::remove_var(DEVICE_CERT_ENV);
        std::env::set_var(RELAY_DISABLED_ENV, "1");

        let dir = tempfile::tempdir().unwrap();
        let seed = [7u8; 32];
        let key_path = dir.path().join("desk.key");
        std::fs::write(&key_path, seed).unwrap();
        let account = [0x5Au8; 32];
        std::fs::write(
            dir.path().join("device.cert"),
            make_cert_b64(desk_public_key(&seed), account),
        )
        .unwrap();
        std::env::set_var(DESK_KEY_ENV, &key_path);

        let reg = WanRegistry::from_env().unwrap();

        // (1) Boot did NOT touch the cert — the cell is untouched.
        assert!(
            !reg.account_resolved(),
            "`from_env` must not resolve the account (the cert read + ed25519 \
             derivation belong to the first WAN dial, not to netd's boot path)"
        );

        // (2) The first ask resolves it — to the REAL cert's account, so the resolver
        //     genuinely ran (a hardcoded observable, or a resolver that never reads the
        //     cert, fails here).
        assert_eq!(reg.account(), Some(account));
        assert!(
            reg.account_resolved(),
            "the first `account()` call must initialize the cell"
        );
    }

    /// `with_account` sets the carried account; `new` defaults it to `None`.
    #[test]
    fn with_account_sets_and_new_defaults_none() {
        let reg = WanRegistry::new(HashMap::new(), [1u8; 32], RelayConfig::Disabled);
        assert_eq!(reg.account(), None, "new() carries no account");
        let acct = [0xEEu8; 32];
        assert_eq!(reg.with_account(Some(acct)).account(), Some(acct));
    }

    /// Composing `with_desk_paths` AFTER a pre-seeding builder is
    /// REFUSED loudly instead of silently discarding the injected answer.
    ///
    /// `with_desk_paths` must reset both lazy cells (the real resolvers have to run),
    /// so `with_epoch_dir(Some(tempdir)).with_desk_paths(..)` would drop the hermetic
    /// directory and re-arm the env/filesystem-backed resolver — inside a test file
    /// whose whole contract is "parallel-safe, no ambient env". The failure mode is
    /// silent and order-dependent (the reverse order keeps the injection), which is
    /// exactly why it is refused.
    ///
    /// Env-free (no resolver runs — the panic fires before any cell is read), but it
    /// DOES swap the process-global panic HOOK (to keep the by-design panics quiet),
    /// which is the same class of process-global state [`crate::test_env`] exists to
    /// serialize — so it takes that lock too, and restores the hook EXPLICITLY before
    /// the first assert that can unwind (see below).
    #[test]
    fn with_desk_paths_after_a_pre_seed_is_refused_not_silently_clobbered() {
        /// The panic payload as text (a bare `assert!` message is a `&'static str`;
        /// a formatted one is a `String` — accept either so the assert below tests
        /// the MESSAGE, not the payload type).
        fn panic_text(err: &(dyn std::any::Any + Send)) -> String {
            err.downcast_ref::<String>()
                .cloned()
                .or_else(|| err.downcast_ref::<&'static str>().map(|s| s.to_string()))
                .unwrap_or_else(|| "<non-string panic payload>".to_string())
        }

        /// Leak BACKSTOP for the process-global panic hook. The PRIMARY restore is the
        /// explicit `drop(hook_guard)` below, placed before the first assert that can
        /// unwind; this `Drop` only covers an unexpected panic between the install and
        /// that point.
        ///
        /// Why the `thread::panicking()` check is load-bearing:
        /// [`std::panic::set_hook`] PANICS when called from a panicking thread, and std
        /// raises that as a NON-UNWINDING panic, i.e. `abort()`. A `Drop` that restored
        /// unconditionally would therefore turn any unwind through this scope into SIGABRT,
        /// and the abort lands on precisely the path
        /// this test guards (revert `with_desk_paths`'s `assert!`s ⇒ the `expect_err`s
        /// panic ⇒ EXIT=134), losing the failing test's name AND every sibling's result
        /// — strictly worse than the leaked hook it would prevent. On the unwind path
        /// this deliberately LEAKS the no-op hook rather than kill the binary.
        ///
        /// A `Drop`-based restore cannot recover a test's own failure message either: it is
        /// swallowed because the no-op hook is installed AT panic time (the hook runs
        /// before unwinding begins). What
        /// recovers it is the ORDER below — catch, restore, then assert.
        type PanicHook = Box<dyn Fn(&std::panic::PanicHookInfo<'_>) + Sync + Send + 'static>;
        struct PanicHookGuard(Option<PanicHook>);
        impl Drop for PanicHookGuard {
            fn drop(&mut self) {
                if std::thread::panicking() {
                    return;
                }
                if let Some(prev) = self.0.take() {
                    std::panic::set_hook(prev);
                }
            }
        }

        // The hook is process-global: hold the crate-wide lock so a concurrent
        // env/global-state test cannot have its own panic message swallowed by our
        // no-op hook. (Nothing below takes this lock, so no re-entrant deadlock.)
        let _lock = env_lock();

        let dir = PathBuf::from("/tmp/hermetic-epochs");
        let inputs = DeskPathInputs {
            key_file: Some(PathBuf::from("/tmp/desk.key")),
            ..Default::default()
        };
        // These arms panic BY DESIGN — silence the default hook so the expected
        // panics do not print scary backtraces in a passing run.
        let hook_guard = {
            let prev = std::panic::take_hook();
            std::panic::set_hook(Box::new(|_| {}));
            PanicHookGuard(Some(prev))
        };

        // (a) epoch-dir pre-seed then desk paths → PANIC naming both builders.
        let seeded = WanRegistry::new(HashMap::new(), [7u8; 32], RelayConfig::Disabled)
            .with_epoch_dir(Some(dir.clone()));
        let inputs_a = inputs.clone();
        let caught_a = std::panic::catch_unwind(move || seeded.with_desk_paths(inputs_a));

        // (b) account pre-seed then desk paths → likewise.
        let seeded = WanRegistry::new(HashMap::new(), [7u8; 32], RelayConfig::Disabled)
            .with_account(Some([9u8; 32]));
        let inputs_b = inputs.clone();
        let caught_b = std::panic::catch_unwind(move || seeded.with_desk_paths(inputs_b));

        // Restore the hook HERE — before ANY assert that can unwind. Both
        // `catch_unwind` results are already bound, so every check below (the
        // `expect_err`s included, which are exactly what the guarded mutation flips)
        // panics with the REAL hook installed and libtest prints the failure normally.
        drop(hook_guard);

        let msg_a = panic_text(
            caught_a
                .expect_err("composing over a with_epoch_dir pre-seed must be refused")
                .as_ref(),
        );
        let msg_b = panic_text(
            caught_b
                .expect_err("composing over a with_account pre-seed must be refused")
                .as_ref(),
        );

        assert!(
            msg_a.contains("with_desk_paths") && msg_a.contains("with_epoch_dir"),
            "the panic names both builders + the fix: {msg_a}"
        );
        assert!(
            msg_b.contains("with_desk_paths") && msg_b.contains("with_account"),
            "the panic names both builders + the fix: {msg_b}"
        );

        // (c) ANTI-TAUTOLOGY: the SUPPORTED order composes fine, and the later
        //     `with_epoch_dir` injection WINS (the hermetic directory survives).
        let reg = WanRegistry::new(HashMap::new(), [7u8; 32], RelayConfig::Disabled)
            .with_desk_paths(inputs)
            .with_epoch_dir(Some(dir.clone()));
        assert_eq!(
            reg.epoch_dir(),
            Some(dir.as_path()),
            "with_desk_paths THEN with_epoch_dir keeps the injected directory"
        );

        // (d) And `with_desk_paths` on a fresh registry (the production DI shape) is
        //     accepted — the panic guards composition, not the builder itself.
        let reg = WanRegistry::new(HashMap::new(), [7u8; 32], RelayConfig::Disabled)
            .with_desk_paths(DeskPathInputs::default());
        assert!(
            !reg.account_resolved(),
            "a fresh with_desk_paths registry leaves the account cell unresolved"
        );
    }

    /// [`WanRegistry::from_env`] does NOT read the device cert — the file read
    /// and key match happen on the FIRST [`WanRegistry::account`] call, off the daemon's
    /// boot critical path (a desk consumer BLOCKS on netd's socket binding, so boot-path
    /// work is latency a user pays for).
    ///
    /// Observable, non-tautological oracle: build the registry while the cert does NOT
    /// exist, THEN write a valid one, THEN ask. Eager (boot-time) resolution answers
    /// `None` — it already looked and found nothing. Lazy resolution answers
    /// `Some(account)`. So this pin FAILS the moment the resolution moves back into
    /// `from_env`, and it also states the user-visible consequence: a desk that runs
    /// `cerulion login` AFTER netd started gets its account on the next dial, with no
    /// daemon restart.
    ///
    /// Anti-tautology control folded in: a registry built the same way whose cert never
    /// appears still answers `None` (the lazy resolver really runs and really finds
    /// nothing — it does not just echo whatever was written last).
    ///
    /// Env-mutating → serialized on the file-local [`env_lock`].
    #[test]
    fn account_resolution_is_deferred_off_the_boot_path_to_first_use() {
        let _lock = env_lock();
        let _snap = EnvSnapshot::take(&[
            DESK_KEY_ENV,
            DEVICE_CERT_ENV,
            WAN_ROBOTS_ENV,
            RELAY_DISABLED_ENV,
            RELAY_URL_ENV,
        ]);
        std::env::remove_var(WAN_ROBOTS_ENV);
        std::env::remove_var(DEVICE_CERT_ENV);
        std::env::set_var(RELAY_DISABLED_ENV, "1");

        let dir = tempfile::tempdir().unwrap();
        let seed = [3u8; 32];
        let key_path = dir.path().join("desk.key");
        std::fs::write(&key_path, seed).unwrap();
        let cert_path = dir.path().join("device.cert");
        let account = [0x77u8; 32];
        std::env::set_var(DESK_KEY_ENV, &key_path);

        // (1) THE DEFERRAL PIN. No cert exists at build time…
        assert!(!cert_path.exists());
        let deferred = WanRegistry::from_env().unwrap();
        // …it appears afterwards…
        std::fs::write(&cert_path, make_cert_b64(desk_public_key(&seed), account)).unwrap();
        // …and the FIRST `account()` call is what reads it.
        assert_eq!(
            deferred.account(),
            Some(account),
            "the cert must be read at FIRST USE, not at from_env — an eager \
             resolution would have answered None"
        );
        // Resolved exactly once: the answer is now frozen even if the cert changes.
        std::fs::remove_file(&cert_path).unwrap();
        assert_eq!(
            deferred.account(),
            Some(account),
            "the OnceLock resolves once per process (the same cardinality boot-time \
             resolution had)"
        );

        // (2) ANTI-TAUTOLOGY CONTROL: same construction, cert never appears ⇒ None.
        let never = WanRegistry::from_env().unwrap();
        assert!(!cert_path.exists());
        assert_eq!(
            never.account(),
            None,
            "the lazy resolver really runs and really finds nothing"
        );
    }

    /// The epoch-cache half: a DI registry built with
    /// [`WanRegistry::new`] resolves its desk-path facts to `None` WITHOUT consulting
    /// the ambient environment — so a test (or an embedder) never picks up a stray
    /// `CERULION_EPOCH_DIR` from the machine it runs on.
    ///
    /// The env is deliberately SET to a value a leaking resolver would return.
    #[test]
    fn a_di_registry_never_consults_the_ambient_environment() {
        let _lock = env_lock();
        let _snap = EnvSnapshot::take(&[EPOCH_DIR_ENV, DEVICE_CERT_ENV, DESK_KEY_ENV]);
        std::env::set_var(EPOCH_DIR_ENV, "/leaked/epochs");

        let reg = WanRegistry::new(HashMap::new(), [1u8; 32], RelayConfig::Disabled);
        assert_eq!(
            reg.epoch_cache_path("go2"),
            None,
            "a DI registry must not inherit the ambient {EPOCH_DIR_ENV}"
        );
        assert_eq!(reg.account(), None, "…nor resolve an ambient device cert");

        // …and an INJECTED directory still wins over the ambient env (the DI path is
        // authoritative, not merely absent).
        let injected = PathBuf::from("/injected/epochs");
        let reg = WanRegistry::new(HashMap::new(), [1u8; 32], RelayConfig::Disabled)
            .with_epoch_dir(Some(injected.clone()));
        assert_eq!(
            reg.epoch_cache_path("go2"),
            Some(injected.join("go2.epoch"))
        );
    }
}
