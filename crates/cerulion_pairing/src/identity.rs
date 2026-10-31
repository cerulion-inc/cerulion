// SPDX-License-Identifier: MIT OR Apache-2.0
//! The machine identity on disk: one config directory rule, one machine key,
//! and the MAC'd high-water file.
//!
//! ## Layout
//!
//! ```text
//! <config>/                      CERULION_HOME verbatim when set and non-empty, else ~/.cerulion
//!   desk.key                     legacy name of the machine key (a hard link to, or equal copy of, identity/device.key)
//!   identity/                    0700
//!     device.key                 the 32-byte ed25519 seed, 0600
//!     active_team                the team directory name this machine opens sessions with, one line
//!     <team>/                    0700, one directory per team (one DNS label)
//!       high_water               clock, bundle generation and status sequence floors, MAC'd
//!       leaf.der                 this machine's team leaf certificate, DER, 0600
//!       bundle                   the team trust bundle, postcard `SignedTeamTrustBundle`, 0600
//! ```
//!
//! ## Team credentials
//!
//! [`IdentityLayout::write_team_credentials`] writes a team's leaf and bundle
//! (each replaced atomically) and [`IdentityLayout::set_active_team`] names
//! the team. [`IdentityLayout::load_team_credentials`] reads the active team
//! back and accepts it only when the bundle verifies against one of the root
//! sets it is given, the leaf satisfies the team profile and is signed by the
//! bundle's current issuing CA, the leaf certifies this machine's key, and a
//! leaf other than a robot's has not passed its notAfter. With no
//! `active_team` file it returns `None`. It reads no high-water: whether the
//! bundle's generation is newer than one this machine already accepted is the
//! caller's `team::rollback` decision.
//!
//! ## The one machine key
//!
//! [`IdentityLayout::load_or_create_machine_key`] is the only writer of the
//! machine key. It reads `identity/device.key`; when only the legacy
//! `desk.key` exists it adopts that file byte for byte (a hard link, so the
//! key every robot already trusts is kept); when neither exists it creates a
//! fresh key. It then makes sure `desk.key` names the same key, so readers of
//! the legacy path (`CERULION_NETD_DESK_KEY` set to it, older binaries) keep
//! working. Two files holding different keys are refused with
//! [`PairingError::MachineKeyConflict`] and neither is touched. A key file of
//! the wrong size is refused with [`PairingError::MachineKeyMalformed`] and
//! never regenerated in place.
//!
//! ## The high-water file
//!
//! ```text
//! magic "CERHW001" (8) | clock_unix_s u64-LE (8) | bundle_generation u32-LE (4)
//!   | status_seq u64-LE (8) | HMAC-SHA256 tag (32)          = 60 bytes
//! mac_key = HMAC-SHA256(key = machine seed, "cerulion-identity-high-water:v1")
//! tag     = HMAC-SHA256(mac_key, the first 28 bytes)
//! ```
//!
//! The length is checked first, then the tag (constant time), then the magic.
//! [`IdentityLayout::raise_high_water`] holds an exclusive lock on a sibling
//! `high_water.lock` across read, fieldwise maximum and atomic replace, so
//! concurrent raises from several processes never lower a floor.

use std::ffi::OsStr;
use std::fs::File;
use std::io::{ErrorKind, Write as _};
use std::path::{Path, PathBuf};

use ed25519_dalek::SigningKey;
use zeroize::Zeroize;

use crate::crypto;
use crate::error::PairingError;

/// The environment variable that overrides the config directory, used verbatim.
pub const CERULION_HOME_ENV: &str = "CERULION_HOME";
/// The config directory under the home directory when [`CERULION_HOME_ENV`] is unset.
pub const DEFAULT_CONFIG_DIR_NAME: &str = ".cerulion";
/// The identity subdirectory of the config directory.
pub const IDENTITY_SUBDIR: &str = "identity";
/// The machine key file name inside [`IDENTITY_SUBDIR`].
pub const DEVICE_KEY_FILENAME: &str = "device.key";
/// The legacy machine key file name directly under the config directory.
pub const LEGACY_DESK_KEY_FILENAME: &str = "desk.key";
/// The high-water file name inside a team directory.
pub const HIGH_WATER_FILENAME: &str = "high_water";
/// The team leaf certificate file name inside a team directory.
pub const LEAF_FILENAME: &str = "leaf.der";
/// The team trust bundle file name inside a team directory.
pub const BUNDLE_FILENAME: &str = "bundle";
/// The file inside [`IDENTITY_SUBDIR`] naming the active team.
pub const ACTIVE_TEAM_FILENAME: &str = "active_team";
/// The length of an encoded high-water file.
pub const HIGH_WATER_LEN: usize = HIGH_WATER_FRAMED_LEN + 32;

const SEED_LEN: usize = 32;
const HIGH_WATER_MAGIC: [u8; 8] = *b"CERHW001";
const HIGH_WATER_FRAMED_LEN: usize = 8 + 8 + 4 + 8;
const HIGH_WATER_MAC_DOMAIN: &[u8] = b"cerulion-identity-high-water:v1";

// ===========================================================================
// Config directory
// ===========================================================================

/// The config directory rule with its inputs passed in: `cerulion_home`
/// verbatim when it is set and non-empty, else `home_dir/.cerulion`, else `None`.
pub fn config_dir_from(cerulion_home: Option<&OsStr>, home_dir: Option<&Path>) -> Option<PathBuf> {
    if let Some(home) = cerulion_home.filter(|h| !h.is_empty()) {
        return Some(PathBuf::from(home));
    }
    home_dir.map(|h| h.join(DEFAULT_CONFIG_DIR_NAME))
}

/// The config directory of this process: [`config_dir_from`] over
/// `CERULION_HOME` and the user's home directory.
pub fn config_dir() -> Option<PathBuf> {
    config_dir_from(
        std::env::var_os(CERULION_HOME_ENV).as_deref(),
        dirs::home_dir().as_deref(),
    )
}

// ===========================================================================
// Layout
// ===========================================================================

/// A team directory name: one DNS label (1 to 63 of `a-z`, `0-9`, `-`, not
/// starting or ending with `-`), so it is safe as a single path component.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct TeamDirName(String);

impl TeamDirName {
    /// Validate `name`.
    ///
    /// # Errors
    ///
    /// [`PairingError::InvalidTeamName`] when `name` is not one DNS label.
    pub fn new(name: &str) -> Result<Self, PairingError> {
        let bytes = name.as_bytes();
        let label_char = |b: &u8| b.is_ascii_lowercase() || b.is_ascii_digit() || *b == b'-';
        let valid = (1..=63).contains(&bytes.len())
            && bytes.iter().all(label_char)
            && bytes.first() != Some(&b'-')
            && bytes.last() != Some(&b'-');
        if valid {
            Ok(TeamDirName(name.to_owned()))
        } else {
            Err(PairingError::InvalidTeamName(name.to_owned()))
        }
    }

    /// The name.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Every machine identity path, derived from one config directory.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IdentityLayout {
    config_dir: PathBuf,
}

/// Where a loaded machine key came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyOrigin {
    /// The canonical key file already existed.
    Existing,
    /// Only the legacy `desk.key` existed; it is now the canonical key too.
    AdoptedLegacy,
    /// No key existed; a fresh one was written.
    Created,
}

impl IdentityLayout {
    /// The layout under `config_dir`.
    pub fn at(config_dir: impl Into<PathBuf>) -> Self {
        IdentityLayout {
            config_dir: config_dir.into(),
        }
    }

    /// The layout under this process's [`config_dir`].
    ///
    /// # Errors
    ///
    /// [`PairingError::NoConfigDir`] when no config directory resolves.
    pub fn resolve() -> Result<Self, PairingError> {
        config_dir().map(Self::at).ok_or(PairingError::NoConfigDir)
    }

    /// The config directory.
    pub fn config_dir(&self) -> &Path {
        &self.config_dir
    }

    /// `<config>/identity`.
    pub fn identity_dir(&self) -> PathBuf {
        self.config_dir.join(IDENTITY_SUBDIR)
    }

    /// `<config>/identity/device.key`, the canonical machine key.
    pub fn device_key_path(&self) -> PathBuf {
        self.identity_dir().join(DEVICE_KEY_FILENAME)
    }

    /// `<config>/desk.key`, the legacy name of the machine key.
    pub fn legacy_desk_key_path(&self) -> PathBuf {
        self.config_dir.join(LEGACY_DESK_KEY_FILENAME)
    }

    /// The file a reader that never writes takes the machine key from: the
    /// canonical `identity/device.key` when that name exists (any file type,
    /// so the reader's own checks see it), else the legacy `desk.key`. The
    /// writer, [`IdentityLayout::load_or_create_machine_key`], keeps the two
    /// names on one key and refuses a conflict.
    pub fn machine_key_read_path(&self) -> PathBuf {
        let canonical = self.device_key_path();
        if std::fs::symlink_metadata(&canonical).is_ok() {
            canonical
        } else {
            self.legacy_desk_key_path()
        }
    }

    /// `<config>/identity/<team>`.
    pub fn team_dir(&self, team: &TeamDirName) -> PathBuf {
        self.identity_dir().join(team.as_str())
    }

    /// `<config>/identity/<team>/high_water`.
    pub fn high_water_path(&self, team: &TeamDirName) -> PathBuf {
        self.team_dir(team).join(HIGH_WATER_FILENAME)
    }

    /// `<config>/identity/<team>/leaf.der`.
    pub fn leaf_path(&self, team: &TeamDirName) -> PathBuf {
        self.team_dir(team).join(LEAF_FILENAME)
    }

    /// `<config>/identity/<team>/bundle`.
    pub fn bundle_path(&self, team: &TeamDirName) -> PathBuf {
        self.team_dir(team).join(BUNDLE_FILENAME)
    }

    /// `<config>/identity/active_team`.
    pub fn active_team_path(&self) -> PathBuf {
        self.identity_dir().join(ACTIVE_TEAM_FILENAME)
    }

    /// Load the machine key, creating it when no key exists (see the module
    /// docs for adoption, the legacy alias and the refusals).
    ///
    /// # Errors
    ///
    /// [`PairingError::MachineKeyConflict`], [`PairingError::MachineKeyMalformed`]
    /// or [`PairingError::IdentityIo`].
    pub fn load_or_create_machine_key(&self) -> Result<(MachineKey, KeyOrigin), PairingError> {
        if let Some(found) = self.load_existing()? {
            return Ok(found);
        }
        create_secret_dir(&self.identity_dir())?;
        let canonical = self.device_key_path();
        let (key, origin) = load_or_create_key_file(&canonical)?;
        // A racing creator may have won with a different legacy file: re-check.
        self.ensure_legacy_alias(&key)?;
        Ok((key, origin))
    }

    /// Load the machine key without generating one: `Ok(None)` when neither the
    /// canonical nor the legacy file exists. A legacy-only key is adopted, and
    /// a missing `desk.key` alias is written, so this may create
    /// `identity/` and the second name of an existing key.
    ///
    /// # Errors
    ///
    /// As [`IdentityLayout::load_or_create_machine_key`].
    pub fn load_machine_key(&self) -> Result<Option<MachineKey>, PairingError> {
        Ok(self.load_existing()?.map(|(k, _)| k))
    }

    /// The shared read path: canonical and legacy reconciled, `None` when
    /// neither exists.
    fn load_existing(&self) -> Result<Option<(MachineKey, KeyOrigin)>, PairingError> {
        let canonical = self.device_key_path();
        let legacy = self.legacy_desk_key_path();
        let canonical_key = read_key_file_if_present(&canonical)?;
        match canonical_key {
            Some(key) => {
                self.ensure_legacy_alias(&key)?;
                Ok(Some((key, KeyOrigin::Existing)))
            }
            None => {
                let Some(legacy_key) = read_key_file_if_present(&legacy)? else {
                    return Ok(None);
                };
                create_secret_dir(&self.identity_dir())?;
                match link_or_copy(&legacy, &canonical, &legacy_key) {
                    Ok(()) => {
                        restrict_to_owner(&canonical)?;
                        Ok(Some((legacy_key, KeyOrigin::AdoptedLegacy)))
                    }
                    // Another process adopted or created it first: reconcile.
                    Err(PairingError::IdentityIo { source, .. })
                        if source.kind() == ErrorKind::AlreadyExists =>
                    {
                        let key = read_key_file(&canonical)?;
                        self.ensure_legacy_alias(&key)?;
                        Ok(Some((key, KeyOrigin::Existing)))
                    }
                    Err(e) => Err(e),
                }
            }
        }
    }

    /// Make `desk.key` name `key`: create the alias when absent, refuse when it
    /// holds a different key.
    fn ensure_legacy_alias(&self, key: &MachineKey) -> Result<(), PairingError> {
        let canonical = self.device_key_path();
        let legacy = self.legacy_desk_key_path();
        let check = |found: MachineKey| {
            if crypto::ct_eq(found.seed(), key.seed()) {
                Ok(())
            } else {
                Err(PairingError::MachineKeyConflict {
                    canonical: canonical.clone(),
                    legacy: legacy.clone(),
                })
            }
        };
        if let Some(found) = read_key_file_if_present(&legacy)? {
            return check(found);
        }
        match link_or_copy(&canonical, &legacy, key) {
            Ok(()) => Ok(()),
            Err(PairingError::IdentityIo { source, .. })
                if source.kind() == ErrorKind::AlreadyExists =>
            {
                check(read_key_file(&legacy)?)
            }
            Err(e) => Err(e),
        }
    }

    /// Read the team's high-water floors; all zero when the file is absent.
    ///
    /// # Errors
    ///
    /// [`PairingError::HighWaterTampered`], [`PairingError::HighWaterCorrupt`]
    /// or [`PairingError::IdentityIo`].
    pub fn load_high_water(
        &self,
        team: &TeamDirName,
        key: &MachineKey,
    ) -> Result<HighWater, PairingError> {
        let path = self.high_water_path(team);
        match std::fs::read(&path) {
            Ok(bytes) => decode_high_water(&bytes, key),
            Err(e) if e.kind() == ErrorKind::NotFound => Ok(HighWater::default()),
            Err(e) => Err(io_err("reading", &path, e)),
        }
    }

    /// Raise the team's floors to the fieldwise maximum of the stored value and
    /// `candidate`, and return the result. Writes only when a floor rises.
    ///
    /// # Errors
    ///
    /// As [`IdentityLayout::load_high_water`]; a tampered or malformed file is
    /// never overwritten.
    pub fn raise_high_water(
        &self,
        team: &TeamDirName,
        key: &MachineKey,
        candidate: HighWater,
    ) -> Result<HighWater, PairingError> {
        let dir = self.team_dir(team);
        create_secret_dir(&dir)?;
        let lock_path = dir.join(format!("{HIGH_WATER_FILENAME}.lock"));
        let lock = open_private(&lock_path, false)?;
        lock.lock().map_err(|e| io_err("locking", &lock_path, e))?;
        let current = self.load_high_water(team, key)?;
        let raised = current.raised_to(candidate);
        if raised != current {
            write_private_atomic(
                &self.high_water_path(team),
                &encode_high_water(&raised, key),
            )?;
        }
        // Dropping `lock` closes the file, which releases the lock.
        drop(lock);
        Ok(raised)
    }
}

// ===========================================================================
// Team credentials
// ===========================================================================

/// A machine's team credentials as [`IdentityLayout::load_team_credentials`]
/// accepted them.
#[derive(Debug)]
pub struct TeamCredentials {
    /// The team directory name.
    pub team: TeamDirName,
    /// This machine's key.
    pub key: MachineKey,
    /// The leaf certificate, DER.
    pub leaf_der: Vec<u8>,
    /// What the leaf names.
    pub leaf: crate::team::LeafFacts,
    /// The verified team trust bundle.
    pub bundle: crate::team::Verified<crate::team::SignedTeamTrustBundle>,
}

impl IdentityLayout {
    /// Write `team`'s leaf, then its bundle, each replaced atomically,
    /// creating the team directory (0700) when it is missing. Nothing is
    /// checked here; the reader checks, and refuses a leaf that is not of the
    /// bundle's current generation, so a crash between the two writes leaves
    /// credentials that fail to load rather than mismatched ones that load.
    ///
    /// # Errors
    ///
    /// [`PairingError::IdentityIo`], or [`PairingError::TeamCredentials`] for
    /// a bundle that does not encode.
    pub fn write_team_credentials(
        &self,
        team: &TeamDirName,
        leaf_der: &[u8],
        bundle: &crate::team::SignedTeamTrustBundle,
    ) -> Result<(), PairingError> {
        create_secret_dir(&self.team_dir(team))?;
        let encoded = postcard::to_allocvec(bundle).map_err(|_| PairingError::TeamCredentials {
            path: self.bundle_path(team),
            reason: "the bundle does not encode".into(),
        })?;
        write_private_atomic(&self.leaf_path(team), leaf_der)?;
        write_private_atomic(&self.bundle_path(team), &encoded)
    }

    /// Name `team` as the team this machine opens sessions with.
    ///
    /// # Errors
    ///
    /// [`PairingError::IdentityIo`].
    pub fn set_active_team(&self, team: &TeamDirName) -> Result<(), PairingError> {
        create_secret_dir(&self.identity_dir())?;
        write_private_atomic(
            &self.active_team_path(),
            format!("{}\n", team.as_str()).as_bytes(),
        )
    }

    /// The active team, or `None` when no `active_team` file exists.
    ///
    /// # Errors
    ///
    /// [`PairingError::IdentityIo`], or [`PairingError::InvalidTeamName`] for a
    /// file that does not hold one team directory name.
    pub fn active_team(&self) -> Result<Option<TeamDirName>, PairingError> {
        let path = self.active_team_path();
        match std::fs::read_to_string(&path) {
            Ok(text) => TeamDirName::new(text.trim_end_matches('\n')).map(Some),
            Err(e) if e.kind() == ErrorKind::NotFound => Ok(None),
            Err(e) => Err(io_err("reading", &path, e)),
        }
    }

    /// Load and check the active team's credentials (see the module docs).
    /// `now_ns` is the time the bundle's intermediate and a non-robot leaf's
    /// notAfter are checked at.
    ///
    /// # Errors
    ///
    /// [`PairingError::TeamCredentials`] naming the file and what is wrong
    /// with it, [`PairingError::IdentityIo`], or the machine key errors of
    /// [`IdentityLayout::load_machine_key`].
    pub fn load_team_credentials(
        &self,
        root_sets: &[crate::format::RootSet],
        now_ns: u64,
    ) -> Result<Option<TeamCredentials>, PairingError> {
        use crate::team::{check_team_ca, verify_leaf_issued_by, verify_team_trust_bundle};
        let Some(team) = self.active_team()? else {
            return Ok(None);
        };
        let refuse = |path: PathBuf, reason: String| PairingError::TeamCredentials { path, reason };
        let key = self.load_machine_key()?.ok_or_else(|| {
            refuse(
                self.device_key_path(),
                "there is no machine key; the team leaf certifies a key this machine no longer has"
                    .into(),
            )
        })?;
        let read = |path: PathBuf| -> Result<Vec<u8>, PairingError> {
            std::fs::read(&path).map_err(|e| {
                if e.kind() == ErrorKind::NotFound {
                    refuse(
                        path.clone(),
                        format!(
                            "missing; the active team is '{}', so sign in to it again",
                            team.as_str()
                        ),
                    )
                } else {
                    io_err("reading", &path, e)
                }
            })
        };
        let bundle_path = self.bundle_path(&team);
        let signed: crate::team::SignedTeamTrustBundle =
            postcard::from_bytes(&read(bundle_path.clone())?).map_err(|_| {
                refuse(
                    bundle_path.clone(),
                    "does not decode as a team trust bundle".into(),
                )
            })?;
        if signed.bundle.team_short.as_str() != team.as_str() {
            return Err(refuse(
                bundle_path,
                format!(
                    "names team '{}', not the active team '{}'",
                    signed.bundle.team_short,
                    team.as_str()
                ),
            ));
        }
        let mut failures = Vec::new();
        let mut verified = None;
        for roots in root_sets {
            match verify_team_trust_bundle(roots, &signed, now_ns) {
                Ok(v) => {
                    verified = Some(v);
                    break;
                }
                Err(e) => failures.push(e.to_string()),
            }
        }
        let bundle = verified.ok_or_else(|| {
            refuse(
                bundle_path.clone(),
                if failures.is_empty() {
                    "cannot be checked: this build trusts no root set".into()
                } else {
                    format!(
                        "does not verify against a trusted root set ({} tried: {})",
                        failures.len(),
                        failures.join("; ")
                    )
                },
            )
        })?;
        let leaf_path = self.leaf_path(&team);
        let leaf_der = read(leaf_path.clone())?;
        let ca = check_team_ca(&bundle.get().bundle.issuing_ca_der)
            .map_err(|e| refuse(bundle_path, format!("its issuing CA: {e}")))?;
        let leaf = verify_leaf_issued_by(&leaf_der, &ca).map_err(|e| {
            refuse(
                leaf_path.clone(),
                format!("is not a leaf of the bundle's current generation: {e}"),
            )
        })?;
        if leaf.device_key.0 != key.public_key() {
            return Err(refuse(
                leaf_path,
                format!(
                    "certifies another machine's key; this machine's key is {}",
                    key.short_fingerprint()
                ),
            ));
        }
        // A robot leaf ends only by revocation, never by a clock; every other
        // role's leaf has a notAfter, compared with `now_ns` here.
        if leaf.role != crate::team::TeamRole::Robot
            && now_ns / 1_000_000_000 > leaf.not_after_unix_secs
        {
            return Err(refuse(
                leaf_path,
                format!(
                    "expired at {} (Unix seconds {}); sign in to the team again",
                    unix_date(leaf.not_after_unix_secs),
                    leaf.not_after_unix_secs
                ),
            ));
        }
        Ok(Some(TeamCredentials {
            team,
            key,
            leaf_der,
            leaf,
            bundle,
        }))
    }
}

/// `YYYY-MM-DD` for Unix seconds (proleptic Gregorian, UTC).
pub fn unix_date(unix_secs: u64) -> String {
    let z = (unix_secs / 86_400) as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!("{y:04}-{m:02}-{d:02}")
}

// ===========================================================================
// The machine key
// ===========================================================================

/// The machine's 32-byte ed25519 seed. Its `Debug` prints only the short
/// fingerprint, and the seed is zeroed on drop.
pub struct MachineKey {
    seed: [u8; SEED_LEN],
}

impl MachineKey {
    /// Wrap a seed.
    pub fn from_seed(seed: [u8; SEED_LEN]) -> Self {
        MachineKey { seed }
    }

    /// The seed bytes.
    pub fn seed(&self) -> &[u8; SEED_LEN] {
        &self.seed
    }

    /// The ed25519 public key (also the iroh endpoint id).
    pub fn public_key(&self) -> [u8; 32] {
        SigningKey::from_bytes(&self.seed)
            .verifying_key()
            .to_bytes()
    }

    /// SHA-256 of the public key: the device fingerprint.
    pub fn fingerprint(&self) -> [u8; 32] {
        use sha2::{Digest, Sha256};
        Sha256::digest(self.public_key()).into()
    }

    /// The first 3 fingerprint bytes as 6 lowercase hex digits, for display.
    pub fn short_fingerprint(&self) -> String {
        crypto::to_hex(&self.fingerprint()[..3])
    }
}

impl std::fmt::Debug for MachineKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MachineKey")
            .field("fingerprint", &self.short_fingerprint())
            .finish_non_exhaustive()
    }
}

impl Drop for MachineKey {
    fn drop(&mut self) {
        self.seed.zeroize();
    }
}

/// Read a key file that must hold exactly 32 bytes.
///
/// # Errors
///
/// [`PairingError::IdentityIo`] (including a missing file) or
/// [`PairingError::MachineKeyMalformed`].
pub fn read_key_file(path: &Path) -> Result<MachineKey, PairingError> {
    let mut bytes = std::fs::read(path).map_err(|e| io_err("reading", path, e))?;
    let len = bytes.len();
    let seed: Result<[u8; SEED_LEN], _> = bytes.as_slice().try_into();
    bytes.zeroize();
    match seed {
        Ok(seed) => Ok(MachineKey::from_seed(seed)),
        Err(_) => Err(PairingError::MachineKeyMalformed {
            path: path.to_path_buf(),
            len,
        }),
    }
}

fn read_key_file_if_present(path: &Path) -> Result<Option<MachineKey>, PairingError> {
    match read_key_file(path) {
        Ok(k) => Ok(Some(k)),
        Err(PairingError::IdentityIo { source, .. }) if source.kind() == ErrorKind::NotFound => {
            Ok(None)
        }
        Err(e) => Err(e),
    }
}

/// Load the key at an explicit `path`, creating a fresh one (file 0600, parent
/// directory 0700 when created) when it does not exist. An existing file is
/// never overwritten.
///
/// # Errors
///
/// [`PairingError::MachineKeyMalformed`] or [`PairingError::IdentityIo`].
pub fn load_or_create_key_file(path: &Path) -> Result<(MachineKey, KeyOrigin), PairingError> {
    if let Some(key) = read_key_file_if_present(path)? {
        return Ok((key, KeyOrigin::Existing));
    }
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        create_secret_dir(parent)?;
    }
    let mut seed = [0u8; SEED_LEN];
    getrandom::fill(&mut seed).map_err(|e| {
        io_err(
            "generating a key for",
            path,
            std::io::Error::other(e.to_string()),
        )
    })?;
    let key = MachineKey::from_seed(seed);
    seed.zeroize();
    match write_new_private(path, key.seed()) {
        Ok(()) => Ok((key, KeyOrigin::Created)),
        // A racing creator won: use its key.
        Err(PairingError::IdentityIo { source, .. })
            if source.kind() == ErrorKind::AlreadyExists =>
        {
            Ok((read_key_file(path)?, KeyOrigin::Existing))
        }
        Err(e) => Err(e),
    }
}

/// Make `to` name the key at `from`: a hard link, or a private copy of `key`
/// when `from` is a symbolic link (a hard link to a relative symbolic link
/// would resolve against the other directory) or the file system has no hard
/// links. `AlreadyExists` when `to` exists.
fn link_or_copy(from: &Path, to: &Path, key: &MachineKey) -> Result<(), PairingError> {
    if std::fs::symlink_metadata(from).is_ok_and(|m| m.file_type().is_symlink()) {
        return write_new_private(to, key.seed());
    }
    match std::fs::hard_link(from, to) {
        Ok(()) => sync_parent(to),
        Err(e) if e.kind() == ErrorKind::AlreadyExists => Err(io_err("linking", to, e)),
        Err(e) => {
            tracing::debug!(
                path = %to.display(),
                error = %e,
                "hard link refused; writing a private copy of the machine key"
            );
            write_new_private(to, key.seed())
        }
    }
}

// ===========================================================================
// High water
// ===========================================================================

/// Floors that only rise: the latest trusted wall clock, the newest team bundle
/// generation and the newest team status list sequence this machine accepted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HighWater {
    /// Unix seconds.
    pub clock_unix_s: u64,
    /// Team trust bundle generation.
    pub bundle_generation: u32,
    /// Team status list sequence.
    pub status_seq: u64,
}

impl HighWater {
    /// The fieldwise maximum of `self` and `other`.
    #[must_use]
    pub fn raised_to(self, other: HighWater) -> HighWater {
        HighWater {
            clock_unix_s: self.clock_unix_s.max(other.clock_unix_s),
            bundle_generation: self.bundle_generation.max(other.bundle_generation),
            status_seq: self.status_seq.max(other.status_seq),
        }
    }
}

fn high_water_mac_key(key: &MachineKey) -> [u8; 32] {
    crypto::hmac_sha256(key.seed(), HIGH_WATER_MAC_DOMAIN)
}

/// Encode `hw` with its MAC under `key` (layout in the module docs).
pub fn encode_high_water(hw: &HighWater, key: &MachineKey) -> Vec<u8> {
    let mut out = Vec::with_capacity(HIGH_WATER_LEN);
    out.extend_from_slice(&HIGH_WATER_MAGIC);
    out.extend_from_slice(&hw.clock_unix_s.to_le_bytes());
    out.extend_from_slice(&hw.bundle_generation.to_le_bytes());
    out.extend_from_slice(&hw.status_seq.to_le_bytes());
    let mut mac_key = high_water_mac_key(key);
    let tag = crypto::hmac_sha256(&mac_key, &out);
    mac_key.zeroize();
    out.extend_from_slice(&tag);
    out
}

/// Decode and verify a high-water file.
///
/// # Errors
///
/// [`PairingError::HighWaterCorrupt`] for a wrong length or magic,
/// [`PairingError::HighWaterTampered`] when the MAC does not verify.
pub fn decode_high_water(bytes: &[u8], key: &MachineKey) -> Result<HighWater, PairingError> {
    if bytes.len() != HIGH_WATER_LEN {
        return Err(PairingError::HighWaterCorrupt("wrong length"));
    }
    let (framed, tag) = bytes.split_at(HIGH_WATER_FRAMED_LEN);
    let mut mac_key = high_water_mac_key(key);
    let expected = crypto::hmac_sha256(&mac_key, framed);
    mac_key.zeroize();
    if !crypto::ct_eq(&expected, tag) {
        return Err(PairingError::HighWaterTampered);
    }
    if framed[..8] != HIGH_WATER_MAGIC {
        return Err(PairingError::HighWaterCorrupt("bad magic"));
    }
    let u64_at = |at: usize| {
        let mut b = [0u8; 8];
        b.copy_from_slice(&framed[at..at + 8]);
        u64::from_le_bytes(b)
    };
    let mut generation = [0u8; 4];
    generation.copy_from_slice(&framed[16..20]);
    Ok(HighWater {
        clock_unix_s: u64_at(8),
        bundle_generation: u32::from_le_bytes(generation),
        status_seq: u64_at(20),
    })
}

// ===========================================================================
// Private files
// ===========================================================================

pub(crate) fn io_err(op: &'static str, path: &Path, source: std::io::Error) -> PairingError {
    PairingError::IdentityIo {
        op,
        path: path.to_path_buf(),
        source,
    }
}

/// Create `dir` and any missing parents with mode 0700 on Unix. Existing
/// directories keep their mode.
pub(crate) fn create_secret_dir(dir: &Path) -> Result<(), PairingError> {
    let mut builder = std::fs::DirBuilder::new();
    builder.recursive(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::DirBuilderExt;
        builder.mode(0o700);
    }
    builder
        .create(dir)
        .map_err(|e| io_err("creating directory", dir, e))
}

/// Open `path` for writing with mode 0600 on Unix; `create_new` refuses an
/// existing file, otherwise an existing file is opened as is.
fn open_private(path: &Path, create_new: bool) -> Result<File, PairingError> {
    let mut opts = std::fs::OpenOptions::new();
    opts.write(true);
    if create_new {
        opts.create_new(true);
    } else {
        opts.create(true).truncate(false);
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        opts.mode(0o600);
    }
    opts.open(path).map_err(|e| io_err("creating", path, e))
}

/// Clear the group and other permission bits of an adopted key on Unix (a
/// hard link shares them with the legacy name).
fn restrict_to_owner(path: &Path) -> Result<(), PairingError> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mode = std::fs::metadata(path)
            .map_err(|e| io_err("reading", path, e))?
            .permissions()
            .mode();
        if mode & 0o077 != 0 {
            std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))
                .map_err(|e| io_err("restricting", path, e))?;
            tracing::info!(
                path = %path.display(),
                "restricted the adopted machine key to its owner (0600)"
            );
        }
    }
    #[cfg(not(unix))]
    let _ = path;
    Ok(())
}

/// A sibling of `path` no other writer uses: pid plus a per-process counter.
fn unique_sibling(path: &Path, tag: &str) -> PathBuf {
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let mut name = std::ffi::OsString::from(".");
    name.push(path.file_name().unwrap_or_default());
    name.push(format!(
        ".{tag}.{}.{}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    path.with_file_name(name)
}

/// Write `bytes` to a fresh 0600 sibling and fsync it. A sibling name that
/// already exists (left by a crashed writer that had the same pid) is skipped,
/// so `AlreadyExists` from the callers only ever means the target exists.
fn write_sibling(path: &Path, tag: &str, bytes: &[u8]) -> Result<PathBuf, PairingError> {
    const ATTEMPTS: usize = 1024;
    let mut attempt = 0;
    let (tmp, mut f) = loop {
        let tmp = unique_sibling(path, tag);
        match open_private(&tmp, true) {
            Ok(f) => break (tmp, f),
            Err(PairingError::IdentityIo { source, .. })
                if source.kind() == ErrorKind::AlreadyExists && attempt + 1 < ATTEMPTS =>
            {
                attempt += 1;
            }
            Err(e) => return Err(e),
        }
    };
    let written = f
        .write_all(bytes)
        .map_err(|e| io_err("writing", &tmp, e))
        .and_then(|()| f.sync_all().map_err(|e| io_err("syncing", &tmp, e)));
    match written {
        Ok(()) => Ok(tmp),
        Err(e) => {
            remove_quietly(&tmp);
            Err(e)
        }
    }
}

/// Remove a temporary file this module created; a failure leaves only a
/// dot-named sibling behind and is logged.
fn remove_quietly(path: &Path) {
    if let Err(e) = std::fs::remove_file(path) {
        if e.kind() != ErrorKind::NotFound {
            tracing::debug!(path = %path.display(), error = %e, "could not remove a temporary file");
        }
    }
}

/// fsync the directory holding `path` on Unix, so a rename or link into it
/// survives a crash.
fn sync_parent(path: &Path) -> Result<(), PairingError> {
    #[cfg(unix)]
    if let Some(dir) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        File::open(dir)
            .and_then(|d| d.sync_all())
            .map_err(|e| io_err("syncing directory", dir, e))?;
    }
    #[cfg(not(unix))]
    let _ = path;
    Ok(())
}

/// Create `path` holding `bytes`, never replacing an existing file
/// (`AlreadyExists`), and never visible with fewer than all of `bytes`: the
/// bytes go to a synced sibling that is then hard-linked into place. Where the
/// file system has no hard links, `create_new` on `path` itself is the fallback.
pub(crate) fn write_new_private(path: &Path, bytes: &[u8]) -> Result<(), PairingError> {
    let tmp = write_sibling(path, "new", bytes)?;
    let linked = std::fs::hard_link(&tmp, path);
    remove_quietly(&tmp);
    match linked {
        Ok(()) => sync_parent(path),
        Err(e) if e.kind() == ErrorKind::AlreadyExists => Err(io_err("creating", path, e)),
        Err(e) => {
            tracing::debug!(
                path = %path.display(),
                error = %e,
                "hard link refused; creating the file in place"
            );
            let mut f = open_private(path, true)?;
            f.write_all(bytes).map_err(|e| io_err("writing", path, e))?;
            f.sync_all().map_err(|e| io_err("syncing", path, e))?;
            sync_parent(path)
        }
    }
}

/// Replace `path` with `bytes` atomically: a synced 0600 sibling renamed over
/// it, then the directory synced.
pub(crate) fn write_private_atomic(path: &Path, bytes: &[u8]) -> Result<(), PairingError> {
    let tmp = write_sibling(path, "tmp", bytes)?;
    if let Err(e) = std::fs::rename(&tmp, path) {
        remove_quietly(&tmp);
        return Err(io_err("replacing", path, e));
    }
    sync_parent(path)
}
