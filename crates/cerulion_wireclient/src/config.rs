// SPDX-License-Identifier: AGPL-3.0-only
//! The resolved connect parameters + the pure input parsers.
//!
//! The binary parses CLI flags into raw strings; these helpers turn them into
//! the typed [`ConnectConfig`] the worker consumes. Every parser is PURE and
//! oracle-tested — the only file-reading helpers are [`resolve_desk_seed`] (an
//! optional key file) and [`resolve_desk_account`] (the cached device cert,
//! A4), and both split a pure core ([`parse_device_cert`] for the latter)
//! from the I/O so the decision logic stays oracle-tested.

use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::time::Duration;

use cerulion_link::{EndpointId, RelayConfig};

use crate::error::{ConnectError, ConnectResult};

/// Default bound on each robot control/preamble read (f1 — bounded waits
/// everywhere; an admit-then-stall robot must not hang the pre-shutdown setup).
pub const DEFAULT_ROBOT_TIMEOUT: Duration = Duration::from_secs(30);

/// Which of the robot's topics to demand.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DemandSet {
    /// Demand nothing — fetch + print the catalog, then idle (the discoverable
    /// default: zero `--topic` and no `--all`).
    CatalogOnly,
    /// Demand exactly these topics (`--topic /x` repeated).
    Named(Vec<String>),
    /// Demand EVERY topic the catalog lists (`--all`).
    All,
}

impl DemandSet {
    /// Resolve the demand set from the CLI flags: `--all` wins; else the named
    /// topics if any; else the catalog-only default.
    pub fn from_flags(topics: Vec<String>, all: bool) -> Self {
        if all {
            DemandSet::All
        } else if topics.is_empty() {
            DemandSet::CatalogOnly
        } else {
            DemandSet::Named(topics)
        }
    }
}

/// The fully-resolved parameters for one `cerulion connect` session.
#[derive(Clone)]
pub struct ConnectConfig {
    /// The robot's iroh endpoint id (== its device key's public half).
    pub robot_eid: EndpointId,
    /// Optional direct socket addresses for the robot (LAN direct-dial); empty ⇒
    /// resolve the eid via relay/discovery.
    pub direct_addrs: Vec<SocketAddr>,
    /// Which topics to demand.
    pub demand: DemandSet,
    /// The desk's 32-byte ed25519 device seed (its iroh identity + pairing seed).
    pub desk_seed: [u8; 32],
    /// The iroh relay configuration.
    pub relay: RelayConfig,
    /// Where to materialize fetched `.msg`/YAML schemas so `topic echo` / `viz`
    /// can decode a never-seen type. `None` ⇒ skip schema materialization.
    pub schemas_dir: Option<PathBuf>,
    /// Bound on each robot control/preamble read (f1). The binary uses
    /// [`DEFAULT_ROBOT_TIMEOUT`]; tests can shorten it.
    pub robot_timeout: Duration,
    /// The desk's revocation-epoch cache DIRECTORY (`<robot>.epoch` files).
    /// The session PUSHES the cached epoch for the robot it dials — UNCONDITIONALLY:
    /// revocation propagation is not something a user opts
    /// into, and a flag would let someone dial a robot while silently withholding a
    /// revocation they are holding. `None` (an ephemeral desk key with no key file)
    /// simply means there is no cache to read — a no-op, never a refusal.
    ///
    /// The binary fills this from `epoch::resolve_epoch_dir_from_env`; tests inject a
    /// tempdir.
    pub epoch_dir: Option<PathBuf>,
    /// The name THIS DESK knows the dialed robot by — the key its
    /// revocation-epoch cache lookup uses (`<robot_name>.epoch` inside [`Self::epoch_dir`]).
    ///
    /// It is deliberately NOT the robot's own reported catalog identity. The desk dials
    /// an eid it chose and cryptographically authenticates; the cache is filed under the
    /// desk's own name for that robot (`cerulion pair` pins the name→eid binding in
    /// `~/.cerulion/robots.toml`). Keying by peer-supplied text would let the dialed
    /// robot select WHICH cached artifact it receives — e.g. another robot's signed
    /// epoch, whose revoked account/device ids are not its business.
    ///
    /// `None` when this desk has no verified name for the eid (a raw `--eid` dial of an
    /// unpinned robot, or an ambiguous reverse lookup): the session then pushes NOTHING
    /// and says so loudly — it never falls back to the peer's word.
    pub robot_name: Option<String>,
}

impl std::fmt::Debug for ConnectConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let public =
            cerulion_pairing::client::DeviceIdentity::from_seed(&self.desk_seed).public_key();
        f.write_str("ConnectConfig { desk_seed: [REDACTED], desk_eid: \"")?;
        for byte in public.0 {
            write!(f, "{byte:02x}")?;
        }
        f.write_str("\" }")
    }
}

/// Parse a robot endpoint id from its 64-char hex form (the `--eid` flag).
///
/// The iroh `EndpointId` is an ed25519 public key; its canonical hex is 64
/// lowercase hex chars (32 bytes). A `0x` prefix and surrounding whitespace are
/// tolerated. A wrong length / non-hex / non-canonical key is a LOUD error naming
/// the value.
pub fn parse_eid(raw: &str) -> ConnectResult<EndpointId> {
    let trimmed = raw.trim();
    let hexed = trimmed.strip_prefix("0x").unwrap_or(trimmed);
    let bytes = hex::decode(hexed).map_err(|e| {
        ConnectError::Eid(format!(
            "'{raw}' is not valid hex ({e}); expected 64 hex chars (the robot's 32-byte device key)"
        ))
    })?;
    let arr: [u8; 32] = bytes.as_slice().try_into().map_err(|_| {
        ConnectError::Eid(format!(
            "'{raw}' decoded to {} bytes, but a robot endpoint id is exactly 32 bytes (64 hex chars)",
            bytes.len()
        ))
    })?;
    // `from_bytes` rejects a non-canonical / not-on-curve key — a real ed25519
    // public-key validity check, not just a length gate.
    EndpointId::from_bytes(&arr)
        .map_err(|e| ConnectError::Eid(format!("'{raw}' is not a valid ed25519 endpoint id: {e}")))
}

/// Parse an optional `--account` hex string into a 32-byte account id.
///
/// `None` (flag omitted) yields `None` — the `pair` ceremony then derives a
/// self-account from the desk device key. A present value MUST be 64 hex chars
/// (32 bytes); a `0x` prefix + surrounding whitespace are tolerated. A wrong
/// length / non-hex is a LOUD error naming the value (never a silent default).
pub fn parse_account(raw: Option<&str>) -> ConnectResult<Option<[u8; 32]>> {
    let Some(raw) = raw else {
        return Ok(None);
    };
    let trimmed = raw.trim();
    let hexed = trimmed.strip_prefix("0x").unwrap_or(trimmed);
    let bytes = hex::decode(hexed).map_err(|e| {
        ConnectError::DeskKey(format!(
            "--account '{raw}' is not valid hex ({e}); expected 64 hex chars (a 32-byte account id)"
        ))
    })?;
    let arr: [u8; 32] = bytes.as_slice().try_into().map_err(|_| {
        ConnectError::DeskKey(format!(
            "--account '{raw}' decoded to {} bytes, but an account id is exactly 32 bytes (64 hex chars)",
            bytes.len()
        ))
    })?;
    Ok(Some(arr))
}

/// Parse the repeatable `--addr ip:port` direct socket addresses.
pub fn parse_addrs(raw: &[String]) -> ConnectResult<Vec<SocketAddr>> {
    raw.iter()
        .map(|s| {
            s.trim().parse::<SocketAddr>().map_err(|e| {
                ConnectError::Addr(format!(
                    "'{s}' is not an ip:port socket address ({e}); e.g. 192.168.1.20:7842"
                ))
            })
        })
        .collect()
}

/// Resolve the desk's 32-byte device seed: from `key_file` if given (exactly 32
/// raw bytes), else a freshly-generated EPHEMERAL seed.
///
/// An ephemeral desk identity is UNPAIRED — an un-paired robot refuses it (the
/// intended v1 default: pair the desk first). A persisted key file gives the
/// desk a STABLE identity the robot can put on its access list.
pub fn resolve_desk_seed(key_file: Option<&Path>) -> ConnectResult<[u8; 32]> {
    match key_file {
        Some(path) => load_desk_seed(path),
        None => ephemeral_seed(),
    }
}

/// Resolve the desk seed for PAIRING: load `path` if it exists (exactly 32
/// bytes), else CREATE a fresh 32-byte key there (0600, parent dirs created).
///
/// This is the persistent-identity path `cerulion pair` uses — an ephemeral key
/// the robot access-lists would vanish on exit, so pairing always writes a stable
/// key. An EXISTING key is NEVER overwritten (the create is `create_new`, which
/// also closes the check-then-create race). A wrong-size existing file is a LOUD
/// error (never silently regenerated over a real key).
pub fn resolve_or_create_desk_seed(path: &Path) -> ConnectResult<[u8; 32]> {
    if path.exists() {
        let seed = load_desk_seed(path)?;
        tracing::info!(path = %path.display(), "cerulion pair: using the existing desk device key");
        return Ok(seed);
    }
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).map_err(|e| {
                ConnectError::DeskKey(format!("creating {}: {e}", parent.display()))
            })?;
        }
    }
    let mut seed = [0u8; 32];
    getrandom::getrandom(&mut seed)
        .map_err(|e| ConnectError::KeyGen(format!("getrandom failed: {e}")))?;
    write_new_desk_key(path, &seed)?;
    tracing::info!(path = %path.display(), "cerulion pair: created a new desk device key (0600)");
    Ok(seed)
}

/// Create `path` with mode 0600 and write the 32-byte `seed`. `create_new` fails
/// if the file already exists — NEVER overwrites (belt-and-suspenders vs the
/// `exists()` check + a racing writer).
#[cfg(unix)]
fn write_new_desk_key(path: &Path, seed: &[u8; 32]) -> ConnectResult<()> {
    use std::io::Write;
    use std::os::unix::fs::OpenOptionsExt;
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(path)
        .map_err(|e| ConnectError::DeskKey(format!("creating {}: {e}", path.display())))?;
    f.write_all(seed)
        .map_err(|e| ConnectError::DeskKey(format!("writing {}: {e}", path.display())))
}

/// Non-Unix: no POSIX mode bits; `create_new` still guards against overwrite.
#[cfg(not(unix))]
fn write_new_desk_key(path: &Path, seed: &[u8; 32]) -> ConnectResult<()> {
    use std::io::Write;
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|e| ConnectError::DeskKey(format!("creating {}: {e}", path.display())))?;
    f.write_all(seed)
        .map_err(|e| ConnectError::DeskKey(format!("writing {}: {e}", path.display())))
}

/// Load a 32-byte raw device seed from `path` (exactly 32 bytes).
fn load_desk_seed(path: &Path) -> ConnectResult<[u8; 32]> {
    let bytes = std::fs::read(path)
        .map_err(|e| ConnectError::DeskKey(format!("{}: {e}", path.display())))?;
    let len = bytes.len();
    bytes.try_into().map_err(|_| {
        ConnectError::DeskKey(format!(
            "{} must be exactly 32 raw bytes (an ed25519 device seed), got {len}",
            path.display()
        ))
    })
}

/// Generate a fresh ephemeral 32-byte seed from OS entropy.
fn ephemeral_seed() -> ConnectResult<[u8; 32]> {
    let mut seed = [0u8; 32];
    getrandom::getrandom(&mut seed)
        .map_err(|e| ConnectError::KeyGen(format!("getrandom failed: {e}")))?;
    Ok(seed)
}

// ===========================================================================
// A4 — resolve the desk's cloud ACCOUNT from the cached device cert.
//
// `cerulion login` writes `~/.cerulion/device.cert` = `base64url(postcard(
// SignedDeviceCert))`, the cert the account service issues at login binding the
// desk's DEVICE key → its cloud account (A3's proof-of-possession model). The desk
// (transport) identity and the account thus travel together on-disk. These helpers
// are the READ half of that binding: given the desk's device SEED (its transport
// identity) they resolve the account the cached cert binds it to — so the netd WAN
// dial config presents an account-bound identity. There is NO second account file:
// the device cert is the ONE source, written once by `cerulion login`.
// ===========================================================================

/// Decode a cached device cert (`base64url(postcard(SignedDeviceCert))`, the exact
/// shape `cerulion login` writes) and return the account it binds `desk_public_key`
/// to (pairing A4). Delegates the I1 binding check to the shared
/// [`cerulion_pairing::format::SignedDeviceCert::account_for_device_key`] — the SAME
/// verifier `cerulion_cli_engine::device_binding` uses, so the desk CLI and the netd
/// WAN plane can never derive a DIFFERENT account from one cert.
///
/// Loud errors (never a silent/fabricated account): a non-base64url blob, a blob
/// that is not a `SignedDeviceCert`, or a cert that attests a DIFFERENT device key
/// than the desk holds (I1 — a stale/foreign cert). Pure — oracle-tested, no I/O.
pub fn parse_device_cert(cert_b64: &str, desk_public_key: &[u8; 32]) -> ConnectResult<[u8; 32]> {
    use base64::engine::general_purpose::URL_SAFE_NO_PAD;
    use base64::Engine;
    use cerulion_pairing::format::{PublicKey, SignedDeviceCert};

    let bytes = URL_SAFE_NO_PAD
        .decode(cert_b64.trim().as_bytes())
        .map_err(|e| ConnectError::DeviceCert(format!("not base64url: {e}")))?;
    let signed: SignedDeviceCert = postcard::from_bytes(&bytes).map_err(|e| {
        ConnectError::DeviceCert(format!(
            "did not decode as a SignedDeviceCert: {e} (re-run `cerulion login` to refresh it)"
        ))
    })?;
    let account = signed
        .account_for_device_key(&PublicKey(*desk_public_key))
        .map_err(|_| {
            ConnectError::DeviceCert(
                "attests a DIFFERENT device key than the desk holds (a stale or foreign cert) — \
                 re-run `cerulion login` to refresh it"
                    .into(),
            )
        })?;
    Ok(account.0)
}

/// Resolve the desk's cloud account for `desk_seed` from the cached device cert at
/// `cert_path` (pairing A4). Derives the desk's device public key from the seed (the
/// SAME `DeviceIdentity::from_seed` derivation the pairing layer uses) and reads the
/// cert bound to it:
///
/// - `Ok(Some(account))` — the cert exists and binds THIS desk key (the account bytes);
/// - `Ok(None)` — no cert is cached (the desk has not completed a login that issued
///   one) — the caller presents no account (never bricks; A5 gates the WAN plane);
/// - `Err(..)` — the cert exists but is corrupt OR binds a DIFFERENT key (a loud
///   misconfiguration; the caller warns and presents no account, never a WRONG one).
pub fn resolve_desk_account(
    desk_seed: &[u8; 32],
    cert_path: &Path,
) -> ConnectResult<Option<[u8; 32]>> {
    let cert_b64 = match std::fs::read_to_string(cert_path) {
        Ok(s) => s,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => {
            return Err(ConnectError::DeviceCert(format!(
                "reading {}: {e}",
                cert_path.display()
            )))
        }
    };
    let desk_public_key = cerulion_pairing::client::DeviceIdentity::from_seed(desk_seed)
        .public_key()
        .0;
    parse_device_cert(&cert_b64, &desk_public_key).map(Some)
}

// ===========================================================================
// A5 — the desk-side owner-signed grant CARRIAGE.
//
// The decided model: the robot's OWNER signs an access grant for a subject
// account; the DESK carries it and PRESENTS it at dial time; the robot verifies it
// OFFLINE against its own owner. This is the desk's READ + COMBINE half:
//
//   - the owner hands the subject a GRANT BUNDLE = { intermediate, owner_cert,
//     access_grant }, stored at `~/.cerulion/grants/<robot>.grant` (base64url(
//     postcard(..)), the SAME on-disk convention as A4's `device.cert`);
//   - the desk holds its OWN `device.cert` (its subject cert, A4);
//   - at dial time the desk COMBINES the bundle + its subject cert into the shared
//     `OwnerGrantPresentationWire` and hands the hex-encoded postcard blob to the
//     robot's `present-grant` ops verb.
//
// OWNER SHORT-CIRCUIT: the robot's OWNER needs NO grant — its device key is already
// bound to the owner account (OWNER_FULL) at claim, so the accept gate admits it.
// The owner's desk simply has no `<robot>.grant` file → `resolve_owner_grant`
// returns `Ok(None)` → the desk presents nothing (never bricks). Everything is
// offline; no online call on any dial/present path.
// ===========================================================================

/// The owner-provided half of an owner-signed grant, carried on the desk. Combined
/// at present time with the desk's OWN subject cert (`device.cert`) to form the full
/// [`cerulion_pairing::verify::OwnerGrantPresentationWire`] the `present-grant` verb
/// consumes. Serialized as `base64url(postcard(OwnerGrantBundle))` on disk (the A4
/// `device.cert` convention).
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct OwnerGrantBundle {
    /// The root-signed intermediate certificate.
    pub intermediate: cerulion_pairing::format::SignedIntermediateCert,
    /// The OWNER's device cert (binds the grant's signing key to the owner account).
    pub owner_cert: cerulion_pairing::format::SignedDeviceCert,
    /// The owner-signed access grant (subject account + robot + scope).
    pub access_grant: cerulion_pairing::format::SignedAccessGrant,
}

impl OwnerGrantBundle {
    /// Decode a bundle from its on-disk `base64url(postcard(..))` form. Loud errors
    /// (never fail-open) on a non-base64url blob or a malformed bundle.
    pub fn from_b64(bundle_b64: &str) -> ConnectResult<Self> {
        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        let bytes = URL_SAFE_NO_PAD
            .decode(bundle_b64.trim().as_bytes())
            .map_err(|e| ConnectError::OwnerGrant(format!("not base64url: {e}")))?;
        postcard::from_bytes(&bytes).map_err(|e| {
            ConnectError::OwnerGrant(format!(
                "did not decode as an owner-grant bundle: {e} (re-request the grant from the robot owner)"
            ))
        })
    }

    /// Encode to the on-disk `base64url(postcard(..))` form (the shape the owner's
    /// team page / CLI writes; used by the desk-side round-trip tests).
    pub fn to_b64(&self) -> ConnectResult<String> {
        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        let bytes = postcard::to_stdvec(self).map_err(|e| {
            ConnectError::OwnerGrant(format!("encoding the grant bundle failed: {e}"))
        })?;
        Ok(URL_SAFE_NO_PAD.encode(bytes))
    }

    /// Combine this owner-provided bundle with the desk's OWN subject cert into the
    /// shared presentation wire, then hex-encode its postcard blob — the exact
    /// `grant_postcard` value the robot's `present-grant` ops verb decodes.
    ///
    /// Sanity-binds (never a chain verify — that is the robot's job) the grant's
    /// `subject` to the subject cert's account: a bundle granting a DIFFERENT account
    /// than the desk's own cert is a loud [`ConnectError::OwnerGrant`], so the desk
    /// never presents a grant that could not admit it.
    pub fn to_present_grant_blob(
        &self,
        subject_cert: cerulion_pairing::format::SignedDeviceCert,
    ) -> ConnectResult<String> {
        let grant_subject = self.access_grant.grant.subject;
        let cert_account = subject_cert.cert.account;
        if grant_subject != cert_account {
            return Err(ConnectError::OwnerGrant(format!(
                "grants access to account {} but the desk's own device cert is account {} — \
                 the grant was issued for a DIFFERENT account (re-request a grant for this desk's account)",
                hex::encode(grant_subject.0),
                hex::encode(cert_account.0)
            )));
        }
        let wire = cerulion_pairing::verify::OwnerGrantPresentationWire {
            intermediate: self.intermediate.clone(),
            owner_cert: self.owner_cert.clone(),
            subject_cert,
            access_grant: self.access_grant.clone(),
        };
        let bytes = wire.to_postcard().map_err(|e| {
            ConnectError::OwnerGrant(format!("encoding the presentation failed: {e}"))
        })?;
        Ok(hex::encode(bytes))
    }
}

/// Load the desk's OWN subject cert (`~/.cerulion/device.cert`, the exact A4 shape
/// `base64url(postcard(SignedDeviceCert))`) as a full `SignedDeviceCert`, asserting
/// it attests THIS desk key. Reuses the shared `account_for_device_key` binding
/// so the cert the desk presents can never bind a different key than its account
/// resolution derived. Pure — oracle-tested.
pub fn parse_desk_subject_cert(
    cert_b64: &str,
    desk_public_key: &[u8; 32],
) -> ConnectResult<cerulion_pairing::format::SignedDeviceCert> {
    use base64::engine::general_purpose::URL_SAFE_NO_PAD;
    use base64::Engine;
    use cerulion_pairing::format::{PublicKey, SignedDeviceCert};

    let bytes = URL_SAFE_NO_PAD
        .decode(cert_b64.trim().as_bytes())
        .map_err(|e| ConnectError::DeviceCert(format!("not base64url: {e}")))?;
    let signed: SignedDeviceCert = postcard::from_bytes(&bytes).map_err(|e| {
        ConnectError::DeviceCert(format!(
            "did not decode as a SignedDeviceCert: {e} (re-run `cerulion login` to refresh it)"
        ))
    })?;
    // I1: the cert must attest THIS desk key (never present a stale/foreign cert).
    signed
        .account_for_device_key(&PublicKey(*desk_public_key))
        .map_err(|_| {
            ConnectError::DeviceCert(
                "attests a DIFFERENT device key than the desk holds (a stale or foreign cert) — \
                 re-run `cerulion login` to refresh it"
                    .into(),
            )
        })?;
    Ok(signed)
}

/// Resolve the desk's `present-grant` blob for a robot (pairing A5). Loads the cached
/// owner-grant bundle at `grant_bundle_path`, combines it with the desk's own subject
/// cert at `device_cert_path`, and returns the hex-encoded presentation blob:
///
/// - `Ok(None)` — no `<robot>.grant` bundle is cached: the desk presents no grant.
///   This is the OWNER short-circuit (the owner needs none — its claim already admits
///   it) AND the not-yet-granted guest (the robot will refuse it at accept, with its reason).
/// - `Ok(Some(blob))` — the bundle exists, binds this desk's account, and combined
///   with the desk's subject cert into the presentation the `present-grant` verb takes.
/// - `Err(..)` — the bundle exists but is corrupt / grants a different account, or the
///   desk has no cached `device.cert` to combine it with (a loud misconfiguration; the
///   caller surfaces it rather than presenting a broken grant).
///
/// Fully offline — no network I/O on any path (never bricks).
pub fn resolve_owner_grant(
    desk_seed: &[u8; 32],
    device_cert_path: &Path,
    grant_bundle_path: &Path,
) -> ConnectResult<Option<String>> {
    // No cached bundle → the owner short-circuit / no-grant path: present nothing.
    let bundle_b64 = match std::fs::read_to_string(grant_bundle_path) {
        Ok(s) => s,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => {
            return Err(ConnectError::OwnerGrant(format!(
                "reading {}: {e}",
                grant_bundle_path.display()
            )))
        }
    };
    let bundle = OwnerGrantBundle::from_b64(&bundle_b64)?;

    // The desk MUST have its own subject cert to combine with the bundle.
    let cert_b64 = std::fs::read_to_string(device_cert_path).map_err(|e| {
        if e.kind() == std::io::ErrorKind::NotFound {
            ConnectError::OwnerGrant(format!(
                "a grant bundle is cached at {} but no desk device cert is at {} — \
                 run `cerulion login` first (the grant is presented WITH the desk's own cert)",
                grant_bundle_path.display(),
                device_cert_path.display()
            ))
        } else {
            ConnectError::OwnerGrant(format!("reading {}: {e}", device_cert_path.display()))
        }
    })?;
    let desk_public_key = cerulion_pairing::client::DeviceIdentity::from_seed(desk_seed)
        .public_key()
        .0;
    let subject_cert = parse_desk_subject_cert(&cert_b64, &desk_public_key)?;
    bundle.to_present_grant_blob(subject_cert).map(Some)
}

// ===========================================================================
// The desk-side revocation-EPOCH carriage.
//
// The decided model: revocations reach a robot because the DESKS that
// talk to it PUSH the latest signed epoch on connect. This is the desk's READ half
// (the exact sibling of A5's `resolve_owner_grant`):
//
//   - something online (Studio / the account page / a CLI sync) fetches
//     `GET /v1/robots/{id}/access` from the account service and caches its
//     `{ intermediate, signed_epoch }` pair at `~/.cerulion/epochs/<robot>.epoch`
//     as `base64url(postcard(EpochSyncWire))` — the SAME on-disk convention as A4's
//     `device.cert` and A5's `<robot>.grant`;
//   - at dial time the desk reads that cache and hands the hex-encoded postcard
//     blob to the robot's `sync_epoch` wire verb.
//
// NEVER BLOCKS THE CONNECTION (issue point 3). No cache ⇒ `Ok(None)` ⇒ push
// nothing. A corrupt cache ⇒ a loud `Err` the CALLER logs and then proceeds with
// the dial anyway — a desk that cannot deliver a revocation must still be able to
// reach the robot (the never-bricks-offline floor). Everything here is offline: no
// network I/O on any dial path, so an accountd outage costs nothing.
//
// The desk is a COURIER, not a trust anchor — there is deliberately NO desk-side
// signature check here. The robot re-verifies the intermediate against its own root
// set plus the epoch's signature, issuer, robot id, and monotonic floor
// (`TrustStore::apply_epoch`). A desk-side check would be duplicated policy that
// could only ever be MORE permissive than the robot's, never less.
// ===========================================================================

/// The desk's on-disk epoch cache filename for `robot` (inside the epochs dir):
/// `<robot>.epoch`, mirroring A5's `<robot>.grant`.
///
/// A verbatim re-export of [`cerulion_pairing::verify::epoch_cache_file_name`], which
/// is where the convention LIVES: the cache's WRITER
/// (`cerulion_cli_engine::account_cmd`) must stay iroh-free and therefore cannot link
/// this crate, so writer and reader share the one function in `cerulion_pairing`
/// rather than duplicating a name that could silently drift (a drifted name reads as
/// "nothing cached" forever, and revocations never travel).
pub use cerulion_pairing::verify::epoch_cache_file_name;

// The epoch-cache DIRECTORY resolver lives in `cerulion_pairing::verify`
// (`resolve_epoch_dir` + `EPOCH_DIR_ENV`), re-exported by [`crate::epoch`]. It is NOT
// duplicated here: `cerulion-netd`'s WAN plane and this crate's `cerulion connect`
// session must resolve the SAME directory from the SAME inputs (including the env
// override), or an epoch cached by one path is invisible to the other with no symptom
// at either end. The push itself remains unconditional — the env override relocates
// the cache, it can never disable a push.

/// Encode an epoch-sync artifact to its on-disk `base64url(postcard(..))` form —
/// what an online sync (Studio / the account page / a CLI) writes into the desk's
/// epoch cache after fetching `GET /v1/robots/{id}/access`.
pub fn encode_epoch_cache(wire: &cerulion_pairing::verify::EpochSyncWire) -> ConnectResult<String> {
    use base64::engine::general_purpose::URL_SAFE_NO_PAD;
    use base64::Engine;
    let bytes = wire
        .to_postcard()
        .map_err(|e| ConnectError::EpochSync(format!("encoding the epoch artifact failed: {e}")))?;
    Ok(URL_SAFE_NO_PAD.encode(bytes))
}

/// Decode a cached epoch artifact from its on-disk `base64url(postcard(..))` form.
/// Loud errors (never fail-open) on a non-base64url blob or a malformed artifact —
/// a silently-skipped corrupt cache would look exactly like a delivered revocation.
pub fn decode_epoch_cache(
    cache_b64: &str,
) -> ConnectResult<cerulion_pairing::verify::EpochSyncWire> {
    use base64::engine::general_purpose::URL_SAFE_NO_PAD;
    use base64::Engine;
    let bytes = URL_SAFE_NO_PAD
        .decode(cache_b64.trim().as_bytes())
        .map_err(|e| ConnectError::EpochSync(format!("not base64url: {e}")))?;
    cerulion_pairing::verify::EpochSyncWire::from_postcard(&bytes).map_err(|e| {
        ConnectError::EpochSync(format!(
            "did not decode as an epoch-sync artifact: {e} (re-sync it from the account service)"
        ))
    })
}

/// Resolve the desk's `sync_epoch` blob for a robot. Reads the cached
/// artifact at `epoch_cache_path` and returns the hex-encoded postcard blob the
/// robot's `sync_epoch` wire verb decodes:
///
/// - `Ok(None)` — nothing cached: the desk pushes no epoch. The NORMAL state for a
///   desk that has never synced this robot (and for every guest desk, since
///   `GET /v1/robots/{id}/access` is owner-only). The robot keeps whatever epoch it
///   already has; nothing is weakened.
/// - `Ok(Some(blob))` — the cached artifact, ready to hand to the verb.
/// - `Err(..)` — the cache exists but is unreadable / corrupt. LOUD, so an operator
///   sees that this desk is not carrying revocations; the caller logs it and dials
///   ANYWAY (never block the connection on epoch freshness).
///
/// Fully offline — no network I/O on any path.
pub fn resolve_epoch_sync(epoch_cache_path: &Path) -> ConnectResult<Option<String>> {
    let cache_b64 = match std::fs::read_to_string(epoch_cache_path) {
        Ok(s) => s,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => {
            return Err(ConnectError::EpochSync(format!(
                "reading {}: {e}",
                epoch_cache_path.display()
            )))
        }
    };
    let wire = decode_epoch_cache(&cache_b64)?;
    let bytes = wire.to_postcard().map_err(|e| {
        ConnectError::EpochSync(format!("re-encoding the cached epoch artifact failed: {e}"))
    })?;
    Ok(Some(hex::encode(bytes)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn connect_config_debug_redacts_seed_and_reports_public_identity() {
        // RFC 8032, section 7.1, test vector 1: independent public-key oracle.
        let seed = [
            0x9d, 0x61, 0xb1, 0x9d, 0xef, 0xfd, 0x5a, 0x60, 0xba, 0x84, 0x4a, 0xf4, 0x92, 0xec,
            0x2c, 0xc4, 0x44, 0x49, 0xc5, 0x69, 0x7b, 0x32, 0x69, 0x19, 0x70, 0x3b, 0xac, 0x03,
            0x1c, 0xae, 0x7f, 0x60,
        ];
        let public_hex = "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a";
        let secret_hex = "9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60";
        let public_bytes: [u8; 32] = hex::decode(public_hex).unwrap().try_into().unwrap();
        let config = ConnectConfig {
            robot_eid: EndpointId::from_bytes(&public_bytes).unwrap(),
            direct_addrs: Vec::new(),
            demand: DemandSet::CatalogOnly,
            desk_seed: seed,
            relay: RelayConfig::Disabled,
            schemas_dir: None,
            robot_timeout: Duration::from_secs(1),
            epoch_dir: None,
            robot_name: None,
        };
        let compact = format!("{config:?}");
        let pretty = format!("{config:#?}");
        assert_eq!(
            compact,
            format!("ConnectConfig {{ desk_seed: [REDACTED], desk_eid: \"{public_hex}\" }}")
        );
        for output in [compact, pretty] {
            assert!(output.contains("[REDACTED]"));
            assert!(output.contains(public_hex));
            assert!(!output.contains(secret_hex));
            assert!(!output.contains(&format!("{seed:?}")));
            assert!(!output.contains(&format!("{seed:#?}")));
        }
    }

    /// `--all` wins over named topics; named topics beat the catalog-only default;
    /// empty + no `--all` is catalog-only. Hand oracle.
    #[test]
    fn demand_set_resolution() {
        assert_eq!(
            DemandSet::from_flags(vec!["/a".into()], true),
            DemandSet::All,
            "--all wins even with named topics"
        );
        assert_eq!(
            DemandSet::from_flags(vec!["/a".into(), "/b".into()], false),
            DemandSet::Named(vec!["/a".into(), "/b".into()])
        );
        assert_eq!(
            DemandSet::from_flags(vec![], false),
            DemandSet::CatalogOnly,
            "zero topics + no --all is the discoverable catalog-only default"
        );
        assert_eq!(
            DemandSet::from_flags(vec![], true),
            DemandSet::All,
            "--all with no named topics is still All"
        );
    }

    /// A round-trip: a real 32-byte key hex-encoded parses back to the SAME
    /// endpoint id (derived via the pairing seed → public key equivalence).
    #[test]
    fn parse_eid_round_trips_a_real_key() {
        // Derive a real public key from a seed (the SAME derivation the desk +
        // robot use), hex-encode it, and parse it back.
        let seed = [7u8; 32];
        let public = cerulion_pairing::client::DeviceIdentity::from_seed(&seed)
            .public_key()
            .0;
        let hexed = hex::encode(public);
        let eid = parse_eid(&hexed).expect("a real key parses");
        assert_eq!(eid.as_bytes(), &public, "parsed eid == the derived key");

        // A `0x` prefix + surrounding whitespace are tolerated.
        let eid2 = parse_eid(&format!("  0x{hexed}  ")).expect("prefix+ws tolerated");
        assert_eq!(eid2.as_bytes(), &public);
    }

    /// Wrong length / non-hex are LOUD errors naming the value.
    #[test]
    fn parse_eid_rejects_bad_input() {
        // Too short (16 hex chars = 8 bytes).
        let err = parse_eid("deadbeefdeadbeef").unwrap_err();
        assert!(matches!(err, ConnectError::Eid(_)));
        assert!(err.to_string().contains("32 bytes"), "err: {err}");
        // Non-hex.
        let err = parse_eid("nothexnothexnothex").unwrap_err();
        assert!(matches!(err, ConnectError::Eid(_)));
        assert!(err.to_string().contains("not valid hex"), "err: {err}");
        // Empty.
        assert!(matches!(parse_eid(""), Err(ConnectError::Eid(_))));
    }

    /// Direct addresses parse (v4 + v6); a bad one is a LOUD error naming it.
    #[test]
    fn parse_addrs_oracle() {
        let addrs = parse_addrs(&["192.168.1.20:7842".to_string(), "[::1]:9000".to_string()])
            .expect("valid addrs");
        assert_eq!(addrs.len(), 2);
        assert_eq!(addrs[0], "192.168.1.20:7842".parse::<SocketAddr>().unwrap());

        let err = parse_addrs(&["not-an-addr".to_string()]).unwrap_err();
        assert!(matches!(err, ConnectError::Addr(_)));
        assert!(err.to_string().contains("not-an-addr"), "err: {err}");
        // A missing port is rejected.
        assert!(parse_addrs(&["192.168.1.20".to_string()]).is_err());
    }

    /// A 32-byte key file loads its exact bytes; a wrong-size file is a LOUD
    /// error; an absent file (with a path) errors (never fabricated).
    #[test]
    fn resolve_desk_seed_from_file() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("desk.key");
        let seed = [42u8; 32];
        std::fs::write(&path, seed).unwrap();
        assert_eq!(resolve_desk_seed(Some(&path)).unwrap(), seed);

        // Wrong size.
        let short = dir.path().join("short.key");
        std::fs::write(&short, [1u8; 16]).unwrap();
        let err = resolve_desk_seed(Some(&short)).unwrap_err();
        assert!(matches!(err, ConnectError::DeskKey(_)));
        assert!(err.to_string().contains("32 raw bytes"), "err: {err}");

        // Absent (with a path) → error (never fabricated).
        let missing = dir.path().join("nope.key");
        assert!(matches!(
            resolve_desk_seed(Some(&missing)),
            Err(ConnectError::DeskKey(_))
        ));
    }

    /// `--account`: omitted → `None` (self-account derived later); a 64-hex value
    /// → the 32 bytes (0x-prefix + whitespace tolerated); wrong length / non-hex →
    /// a LOUD error naming the value.
    #[test]
    fn parse_account_oracle() {
        assert_eq!(parse_account(None).unwrap(), None, "omitted → self-account");
        let hex64 = "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";
        // Hand oracle: the 16-byte pattern 00 11 22 … ff, twice (NOT a self-compare
        // of `hex::decode`).
        let half: [u8; 16] = [
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
            0xee, 0xff,
        ];
        let mut expect = [0u8; 32];
        expect[..16].copy_from_slice(&half);
        expect[16..].copy_from_slice(&half);
        assert_eq!(parse_account(Some(hex64)).unwrap(), Some(expect));
        // 0x-prefix + surrounding whitespace tolerated.
        assert_eq!(
            parse_account(Some(&format!("  0x{hex64}  "))).unwrap(),
            Some(expect)
        );
        // Wrong length → loud error naming the value.
        let err = parse_account(Some("deadbeef")).unwrap_err();
        assert!(matches!(err, ConnectError::DeskKey(_)));
        assert!(err.to_string().contains("32 bytes"), "err: {err}");
        // Non-hex → loud error.
        let err = parse_account(Some("nothexnothexnothex")).unwrap_err();
        assert!(err.to_string().contains("not valid hex"), "err: {err}");
    }

    /// `resolve_or_create_desk_seed`: an ABSENT path is CREATED (32 bytes, mode
    /// 0600, non-zero, parent dirs made); an EXISTING key is loaded, NEVER
    /// regenerated (same bytes); a wrong-size existing file is a LOUD error.
    #[test]
    fn resolve_or_create_desk_seed_creates_0600_and_never_overwrites() {
        let dir = tempfile::tempdir().unwrap();
        // A nested path proves the parent dirs are created.
        let path = dir.path().join(".cerulion").join("desk.key");

        // 1. Absent → created.
        let s1 = resolve_or_create_desk_seed(&path).expect("create");
        assert!(path.exists(), "the key file must be created");
        let meta = std::fs::metadata(&path).unwrap();
        assert_eq!(meta.len(), 32, "a desk key is exactly 32 bytes");
        assert_ne!(s1, [0u8; 32], "a created key is real entropy, not zeros");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            assert_eq!(
                meta.permissions().mode() & 0o777,
                0o600,
                "a desk key must be created 0600 (owner-only)"
            );
        }

        // 2. Present → the SAME bytes, never regenerated (never overwritten).
        let s2 = resolve_or_create_desk_seed(&path).expect("load existing");
        assert_eq!(s1, s2, "an existing desk key is loaded, NEVER re-created");

        // 3. A wrong-size existing file → a LOUD error (never a silent regenerate).
        let bad = dir.path().join("bad.key");
        std::fs::write(&bad, [1u8; 16]).unwrap();
        let err = resolve_or_create_desk_seed(&bad).unwrap_err();
        assert!(matches!(err, ConnectError::DeskKey(_)), "err: {err}");
        assert!(err.to_string().contains("32 raw bytes"), "err: {err}");
    }

    /// An ephemeral seed (no key file) is non-zero and differs between calls
    /// (real OS entropy, not a fixed placeholder).
    #[test]
    fn resolve_desk_seed_ephemeral_is_random() {
        let a = resolve_desk_seed(None).expect("ephemeral a");
        let b = resolve_desk_seed(None).expect("ephemeral b");
        assert_ne!(a, [0u8; 32], "an ephemeral seed is not all-zero");
        assert_ne!(a, b, "two ephemeral seeds differ");
    }

    // --- pairing A4: device-cert → account resolution ------------------------

    /// The desk public key derived from `seed` (the SAME derivation the pairing
    /// layer + `resolve_desk_account` use).
    fn desk_public_key(seed: &[u8; 32]) -> [u8; 32] {
        cerulion_pairing::client::DeviceIdentity::from_seed(seed)
            .public_key()
            .0
    }

    /// Build a cached cert blob (`base64url(postcard(SignedDeviceCert))`, the EXACT
    /// shape `cerulion login` / `cerulion_cli_engine::device_binding` write + read)
    /// binding `device_key` → `account`. A throwaway issuer — the account resolver
    /// does NOT check the signature chain (that is the robot's job at pairing time).
    fn make_cert_b64(device_key: [u8; 32], account: [u8; 32]) -> String {
        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        use cerulion_pairing::format::{
            AccountId, DeviceCert, PrincipalKind, PublicKey, Scope, Validity, FORMAT_VERSION,
        };
        let issuer = cerulion_pairing::client::DeviceIdentity::from_seed(&[0xAB; 32]);
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
            issuer_key: issuer.public_key(),
        }
        .sign(&ed25519_dalek::SigningKey::from_bytes(&[0xAB; 32]));
        URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&cert).unwrap())
    }

    /// `parse_device_cert`: a cert for THIS desk key resolves to the exact account
    /// bytes; a cert for a DIFFERENT key is a loud I1 refusal; garbage/non-cert
    /// blobs are loud errors (never a fabricated account). Hand oracles.
    #[test]
    fn parse_device_cert_oracle() {
        let seed = [7u8; 32];
        let key = desk_public_key(&seed);
        let account = [0x42u8; 32];
        let cert_b64 = make_cert_b64(key, account);

        // Matching key → the exact account bytes put in (NOT a self-compare).
        assert_eq!(parse_device_cert(&cert_b64, &key).unwrap(), account);

        // A cert that binds a DIFFERENT device key → I1 refusal naming the mismatch.
        let other_key = desk_public_key(&[9u8; 32]);
        let foreign = make_cert_b64(other_key, account);
        let err = parse_device_cert(&foreign, &key).unwrap_err();
        assert!(matches!(err, ConnectError::DeviceCert(_)));
        assert!(
            err.to_string().contains("DIFFERENT device key"),
            "err: {err}"
        );

        // Non-base64url → loud error.
        let err = parse_device_cert("not base64url !!!", &key).unwrap_err();
        assert!(err.to_string().contains("base64url"), "err: {err}");

        // Valid base64url whose bytes are NOT a SignedDeviceCert → loud decode error.
        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        let junk = URL_SAFE_NO_PAD.encode(b"not a postcard SignedDeviceCert");
        let err = parse_device_cert(&junk, &key).unwrap_err();
        assert!(err.to_string().contains("SignedDeviceCert"), "err: {err}");
    }

    /// `resolve_desk_account`: an ABSENT cert file → `Ok(None)` (no fabricated
    /// account); a present cert bound to this desk key → `Ok(Some(account))`; a
    /// present cert bound to a FOREIGN key → a loud `Err` (never a wrong account).
    #[test]
    fn resolve_desk_account_oracle() {
        let dir = tempfile::tempdir().unwrap();
        let seed = [3u8; 32];
        let key = desk_public_key(&seed);
        let account = [0x77u8; 32];

        // 1. Absent cert file → Ok(None) (never logged in / no cert cached).
        let missing = dir.path().join("device.cert");
        assert_eq!(resolve_desk_account(&seed, &missing).unwrap(), None);

        // 2. Present cert bound to THIS desk key → Some(account) — the closed
        //    login→netd loop (the file shape is exactly what `cerulion login` writes).
        std::fs::write(&missing, make_cert_b64(key, account)).unwrap();
        assert_eq!(
            resolve_desk_account(&seed, &missing).unwrap(),
            Some(account)
        );

        // 3. Present cert bound to a DIFFERENT key (operator pointed the desk key
        //    env at a different key than the one that logged in) → loud Err, so the
        //    caller presents NO account rather than a wrong one.
        let foreign = dir.path().join("foreign.cert");
        std::fs::write(
            &foreign,
            make_cert_b64(desk_public_key(&[9u8; 32]), account),
        )
        .unwrap();
        let err = resolve_desk_account(&seed, &foreign).unwrap_err();
        assert!(matches!(err, ConnectError::DeviceCert(_)), "err: {err}");
    }

    // --- pairing A5: owner-signed grant carriage -----------------------------

    use cerulion_pairing::format::{
        AccessGrant, AccountId, IntermediateCert, PublicKey, RobotId, Role, Scope,
        SignedAccessGrant, SignedDeviceCert, SignedIntermediateCert,
    };
    use ed25519_dalek::SigningKey;

    const A5_ISSUED: u64 = 500_000_000_000;

    fn a5_sk(seed: u8) -> SigningKey {
        SigningKey::from_bytes(&[seed; 32])
    }
    fn a5_pk(k: &SigningKey) -> PublicKey {
        PublicKey(k.verifying_key().to_bytes())
    }
    fn a5_wide() -> cerulion_pairing::format::Validity {
        cerulion_pairing::format::Validity {
            not_before_ns: 0,
            not_after_ns: u64::MAX,
        }
    }
    fn a5_op_scope() -> Scope {
        Scope {
            role: Role::OPERATOR,
            caps: Scope::CAP_TELEOP | Scope::CAP_OBSERVE,
        }
    }

    fn a5_intermediate() -> SignedIntermediateCert {
        IntermediateCert {
            version: cerulion_pairing::format::FORMAT_VERSION,
            intermediate_key: a5_pk(&a5_sk(10)),
            validity: a5_wide(),
            issued_at_ns: A5_ISSUED,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&a5_sk(1)])
    }

    /// The owner's device cert: account 0x01, device key sk(30), issued by the
    /// intermediate (sk(10)).
    fn a5_owner_cert() -> SignedDeviceCert {
        cerulion_pairing::format::DeviceCert {
            version: cerulion_pairing::format::FORMAT_VERSION,
            device_key: a5_pk(&a5_sk(30)),
            account: AccountId([0x01; 32]),
            principal_kind: cerulion_pairing::format::PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity: a5_wide(),
            issued_at_ns: A5_ISSUED,
            issuer_key: a5_pk(&a5_sk(10)),
        }
        .sign(&a5_sk(10))
    }

    /// A bundle granting `subject_account` the operator scope on robot 0x0B, signed by
    /// owner (sk(30)) / account 0x01.
    fn a5_bundle(subject_account: [u8; 32]) -> OwnerGrantBundle {
        let access_grant: SignedAccessGrant = AccessGrant {
            version: cerulion_pairing::format::FORMAT_VERSION,
            subject: AccountId(subject_account),
            robot: RobotId([0x0B; 32]),
            scope: a5_op_scope(),
            principal_kind: cerulion_pairing::format::PrincipalKind::Human,
            validity: a5_wide(),
            issued_at_ns: A5_ISSUED,
            owner: AccountId([0x01; 32]),
            owner_device_key: a5_pk(&a5_sk(30)),
        }
        .sign(&a5_sk(30));
        OwnerGrantBundle {
            intermediate: a5_intermediate(),
            owner_cert: a5_owner_cert(),
            access_grant,
        }
    }

    /// The desk's subject cert (`device.cert`) binding `desk_key` → `account`,
    /// base64url-postcard-encoded — the exact A4 shape.
    fn a5_subject_cert_b64(desk_key: [u8; 32], account: [u8; 32]) -> String {
        make_cert_b64(desk_key, account)
    }

    #[test]
    fn owner_grant_bundle_round_trips_through_b64() {
        let bundle = a5_bundle([0x0A; 32]);
        let b64 = bundle.to_b64().unwrap();
        let back = OwnerGrantBundle::from_b64(&b64).unwrap();
        assert_eq!(
            back, bundle,
            "the owner-grant bundle must survive the on-disk round-trip"
        );
        // Bad base64url + non-postcard bytes are LOUD (never fail-open).
        assert!(OwnerGrantBundle::from_b64("not base64url !!!")
            .unwrap_err()
            .to_string()
            .contains("base64url"));
        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        let junk = URL_SAFE_NO_PAD.encode(b"not a bundle");
        assert!(OwnerGrantBundle::from_b64(&junk)
            .unwrap_err()
            .to_string()
            .contains("did not decode"));
    }

    #[test]
    fn to_present_grant_blob_combines_and_decodes_on_the_robot_side() {
        // The desk holds a subject cert binding its key → account 0x0A; the bundle
        // grants 0x0A. The combined blob decodes back to the SAME four fields via the
        // shared wire shape (byte-compat with the robot's `present-grant` verb).
        let seed = [3u8; 32];
        let desk_key = desk_public_key(&seed);
        let subject_account = [0x0A; 32];
        let subject_cert =
            parse_desk_subject_cert(&a5_subject_cert_b64(desk_key, subject_account), &desk_key)
                .unwrap();
        let bundle = a5_bundle(subject_account);
        let blob = bundle.to_present_grant_blob(subject_cert.clone()).unwrap();

        // Decode the way the robot's verb does — the shared pairing wire shape.
        let bytes = hex::decode(&blob).unwrap();
        let pres =
            cerulion_pairing::verify::OwnerGrantPresentationWire::from_postcard(&bytes).unwrap();
        // Hand oracle: the four fields are exactly what the desk combined.
        assert_eq!(pres.subject_cert, subject_cert);
        assert_eq!(pres.owner_cert, a5_owner_cert());
        assert_eq!(pres.access_grant.grant.subject, AccountId(subject_account));
        assert_eq!(pres.access_grant.grant.robot, RobotId([0x0B; 32]));
    }

    #[test]
    fn to_present_grant_blob_refuses_a_bundle_for_a_different_account() {
        // The bundle grants 0x0C but the desk's cert is account 0x0A → loud refusal,
        // so the desk never presents a grant that could not admit it.
        let seed = [3u8; 32];
        let desk_key = desk_public_key(&seed);
        let subject_cert =
            parse_desk_subject_cert(&a5_subject_cert_b64(desk_key, [0x0A; 32]), &desk_key).unwrap();
        let bundle = a5_bundle([0x0C; 32]); // a DIFFERENT account
        let err = bundle.to_present_grant_blob(subject_cert).unwrap_err();
        assert!(matches!(err, ConnectError::OwnerGrant(_)), "err: {err}");
        assert!(err.to_string().contains("DIFFERENT account"), "err: {err}");
    }

    #[test]
    fn resolve_owner_grant_oracle() {
        let dir = tempfile::tempdir().unwrap();
        let seed = [3u8; 32];
        let desk_key = desk_public_key(&seed);
        let subject_account = [0x0A; 32];
        let cert_path = dir.path().join("device.cert");
        let grant_path = dir.path().join("go2.grant");

        // 1. Absent bundle → Ok(None): the OWNER short-circuit / no-grant path.
        assert_eq!(
            resolve_owner_grant(&seed, &cert_path, &grant_path).unwrap(),
            None
        );

        // 2. Bundle present + a matching device cert → Some(blob) that decodes on the
        //    robot side to the desk's subject cert.
        std::fs::write(&grant_path, a5_bundle(subject_account).to_b64().unwrap()).unwrap();
        std::fs::write(&cert_path, a5_subject_cert_b64(desk_key, subject_account)).unwrap();
        let blob = resolve_owner_grant(&seed, &cert_path, &grant_path)
            .unwrap()
            .expect("a cached bundle + cert yields a present blob");
        let bytes = hex::decode(&blob).unwrap();
        let pres =
            cerulion_pairing::verify::OwnerGrantPresentationWire::from_postcard(&bytes).unwrap();
        assert_eq!(pres.access_grant.grant.subject, AccountId(subject_account));

        // 3. Bundle present but NO device cert → loud Err (present nothing broken).
        std::fs::remove_file(&cert_path).unwrap();
        let err = resolve_owner_grant(&seed, &cert_path, &grant_path).unwrap_err();
        assert!(matches!(err, ConnectError::OwnerGrant(_)), "err: {err}");
        assert!(err.to_string().contains("cerulion login"), "err: {err}");

        // 4. Corrupt bundle → loud Err.
        std::fs::write(&grant_path, "not-a-bundle").unwrap();
        assert!(matches!(
            resolve_owner_grant(&seed, &cert_path, &grant_path),
            Err(ConnectError::OwnerGrant(_))
        ));
    }

    // ---- The desk-side epoch carriage ----------------------------

    /// A cached epoch-sync artifact: the A5 intermediate (so it chains to the same
    /// root fixture) + a genuine signed epoch `n` for robot 0x0B revoking
    /// `revoked_account` and `revoked_device`.
    fn artifact(
        n: u64,
        revoked_account: [u8; 32],
        revoked_device: PublicKey,
    ) -> cerulion_pairing::verify::EpochSyncWire {
        let epoch = cerulion_pairing::format::AccessListEpoch {
            version: cerulion_pairing::format::FORMAT_VERSION,
            robot: RobotId([0x0B; 32]),
            epoch: n,
            revoked_accounts: vec![AccountId(revoked_account)],
            revoked_devices: vec![revoked_device],
            issued_at_ns: A5_ISSUED,
            issuer_key: a5_pk(&a5_sk(10)),
        }
        .sign(&a5_sk(10));
        cerulion_pairing::verify::EpochSyncWire::new(a5_intermediate(), epoch)
    }

    #[test]
    fn epoch_cache_round_trips_through_b64_and_refuses_junk_loudly() {
        let art = artifact(4, [0x0C; 32], a5_pk(&a5_sk(31)));
        let b64 = encode_epoch_cache(&art).unwrap();
        assert_eq!(
            decode_epoch_cache(&b64).unwrap(),
            art,
            "the epoch artifact must survive the on-disk round-trip"
        );
        // Bad base64url + non-postcard bytes are LOUD (never fail-open — a silently
        // skipped corrupt cache looks identical to a delivered revocation).
        assert!(decode_epoch_cache("not base64url !!!")
            .unwrap_err()
            .to_string()
            .contains("base64url"));
        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        let junk = URL_SAFE_NO_PAD.encode(b"not an epoch artifact");
        assert!(decode_epoch_cache(&junk)
            .unwrap_err()
            .to_string()
            .contains("did not decode"));
    }

    // The epoch-cache DIRECTORY resolver's oracle lives with the resolver, in
    // `cerulion_pairing::verify` (`resolve_epoch_dir_is_the_one_desk_wide_rule`), and
    // the two-path AGREEMENT pin — netd's registry vs this crate's connect session
    // resolving the SAME path from the SAME inputs — is
    // `cerulion_wireclient::epoch::both_desk_paths_resolve_one_cache_path`.

    #[test]
    fn resolve_epoch_sync_oracle() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(epoch_cache_file_name("go2"));
        assert_eq!(path.file_name().unwrap(), "go2.epoch");
        // Traversal-safe: separators and dot-only names can never escape the dir.
        assert_eq!(
            epoch_cache_file_name("../../etc/passwd"),
            ".._.._etc_passwd.epoch"
        );
        assert_eq!(epoch_cache_file_name(".."), "__.epoch");
        assert_eq!(epoch_cache_file_name("  go2  "), "go2.epoch");
        // Empty / whitespace-only names must NOT become the hidden dotfile `.epoch`
        // (the vacuous-`all` arm): they get a sentinel.
        assert_eq!(epoch_cache_file_name(""), "_unnamed_.epoch");
        assert_eq!(epoch_cache_file_name("   "), "_unnamed_.epoch");

        // 1. Absent cache → Ok(None): the desk pushes nothing. The normal state for a
        //    never-synced desk and for every guest (the access endpoint is owner-only).
        assert_eq!(resolve_epoch_sync(&path).unwrap(), None);

        // 2. Cached → Some(blob) that decodes ROBOT-SIDE through the shared pairing
        //    wire shape, carrying the exact epoch/robot/revocation sets written.
        let revoked_device = a5_pk(&a5_sk(31));
        let art = artifact(9, [0x0C; 32], revoked_device);
        std::fs::write(&path, encode_epoch_cache(&art).unwrap()).unwrap();
        let blob = resolve_epoch_sync(&path)
            .unwrap()
            .expect("a cached artifact yields a push blob");
        let decoded = cerulion_pairing::verify::EpochSyncWire::from_postcard(
            &hex::decode(&blob).expect("the blob is hex — the wire verb hex-decodes it"),
        )
        .expect("the blob decodes as the shared wire shape");
        // Hand oracle: every carried field, not a self-compare against `art` alone.
        assert_eq!(decoded.signed_epoch.epoch_data.epoch, 9);
        assert_eq!(
            decoded.signed_epoch.epoch_data.robot,
            RobotId([0x0B; 32]),
            "the epoch must stay bound to its robot (the robot refuses a foreign one)"
        );
        assert_eq!(
            decoded.signed_epoch.epoch_data.revoked_accounts,
            vec![AccountId([0x0C; 32])]
        );
        assert_eq!(
            decoded.signed_epoch.epoch_data.revoked_devices,
            vec![revoked_device]
        );
        assert_eq!(decoded, art, "and the artifact survives whole");

        // 3. Corrupt cache → loud Err (the caller logs it and dials ANYWAY). BOTH
        //    corruption shapes are distinguished, so an operator can tell a mangled
        //    file from a stale-format one.
        //    (a) not base64url at all.
        std::fs::write(&path, "not-an-epoch-artifact").unwrap();
        let err = resolve_epoch_sync(&path).unwrap_err();
        assert!(matches!(err, ConnectError::EpochSync(_)), "err: {err}");
        assert!(
            err.to_string().contains("base64url"),
            "a mangled file must be named as such: {err}"
        );
        //    (b) valid base64url whose bytes are not an epoch artifact → the
        //        re-sync hint.
        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        std::fs::write(&path, URL_SAFE_NO_PAD.encode(b"not an epoch artifact")).unwrap();
        let err = resolve_epoch_sync(&path).unwrap_err();
        assert!(matches!(err, ConnectError::EpochSync(_)), "err: {err}");
        assert!(
            err.to_string().contains("account service"),
            "the error must name the fix: {err}"
        );
    }

    #[test]
    fn resolve_epoch_sync_is_deterministic_across_reads() {
        // Two reads of the same cache produce the SAME blob — the push is a pure
        // function of the cached bytes (no clock, no entropy, no network).
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(epoch_cache_file_name("orin"));
        std::fs::write(
            &path,
            encode_epoch_cache(&artifact(2, [0x0C; 32], a5_pk(&a5_sk(31)))).unwrap(),
        )
        .unwrap();
        assert_eq!(
            resolve_epoch_sync(&path).unwrap(),
            resolve_epoch_sync(&path).unwrap()
        );
    }
}
