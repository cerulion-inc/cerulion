// SPDX-License-Identifier: AGPL-3.0-only
//! The per-robot Jev key, read from a file only its owner can read.
//!
//! The file holds one key and nothing else (surrounding whitespace is
//! ignored). It is refused when its mode lets the group or others read or
//! write it, when it holds more than one line or a `=` (an environment file
//! is not a key file), or when the key is not 8 to 512 visible ASCII
//! characters. The key is read again for every ask, so a replaced file
//! takes effect on the next look.
//!
//! [`Credential`] prints as `Credential(..)` and never writes its key to a
//! log, an answer frame or a receipt; the client sends it only in the
//! `Authorization` header, marked sensitive.

use std::path::{Path, PathBuf};

use crate::eye::JEV_KEY_FILE_ENV;

/// The file name of the default credential, in the receipt log's directory.
pub const DEFAULT_KEY_FILE: &str = "jev.key";

/// The largest key file read, bytes.
const MAX_KEY_FILE_BYTES: u64 = 4_096;

/// A Jev key. Its `Debug` form hides the key.
#[derive(Clone, PartialEq, Eq)]
pub struct Credential(String);

impl std::fmt::Debug for Credential {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Credential(..)")
    }
}

impl Credential {
    /// The key, for the `Authorization` header only.
    pub(crate) fn expose(&self) -> &str {
        &self.0
    }
}

/// Why no key was read. None of these carry any of the file's content.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CredentialProblem {
    /// No file at the path.
    Missing,
    /// The file could not be read.
    Unreadable,
    /// The group or others can read or write the file.
    Mode,
    /// The content is not one key.
    Format,
}

impl CredentialProblem {
    /// The `detail` an answer frame carries.
    pub fn code(self) -> &'static str {
        match self {
            Self::Missing => "credential_missing",
            Self::Unreadable => "credential_unreadable",
            Self::Mode => "credential_mode",
            Self::Format => "credential_format",
        }
    }
}

/// Where the credential file is: [`JEV_KEY_FILE_ENV`] when set, else the
/// manifest's `credential_file` (relative to `manifest_dir`), else
/// [`DEFAULT_KEY_FILE`] in `state_dir`.
pub fn credential_path(
    env_value: Option<&str>,
    manifest_file: Option<&Path>,
    manifest_dir: &Path,
    state_dir: &Path,
) -> PathBuf {
    if let Some(path) = env_value.filter(|v| !v.trim().is_empty()) {
        return PathBuf::from(path);
    }
    match manifest_file {
        Some(file) => manifest_dir.join(file),
        None => state_dir.join(DEFAULT_KEY_FILE),
    }
}

/// [`credential_path`] reading [`JEV_KEY_FILE_ENV`] from the environment.
pub fn credential_path_from_env(
    manifest_file: Option<&Path>,
    manifest_dir: &Path,
    state_dir: &Path,
) -> PathBuf {
    let env = std::env::var(JEV_KEY_FILE_ENV).ok();
    credential_path(env.as_deref(), manifest_file, manifest_dir, state_dir)
}

/// Read the key at `path`.
///
/// # Errors
///
/// The [`CredentialProblem`] that applies.
pub fn read_credential(path: &Path) -> Result<Credential, CredentialProblem> {
    let metadata = match std::fs::metadata(path) {
        Ok(m) => m,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Err(CredentialProblem::Missing)
        }
        Err(_) => return Err(CredentialProblem::Unreadable),
    };
    if !metadata.is_file() || metadata.len() > MAX_KEY_FILE_BYTES {
        return Err(CredentialProblem::Format);
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        if metadata.permissions().mode() & 0o077 != 0 {
            return Err(CredentialProblem::Mode);
        }
    }
    let text = std::fs::read_to_string(path).map_err(|_| CredentialProblem::Unreadable)?;
    let key = text.trim();
    let visible = key.bytes().all(|b| b.is_ascii_graphic() && b != b'=');
    if !(8..=512).contains(&key.len()) || !visible {
        return Err(CredentialProblem::Format);
    }
    Ok(Credential(key.to_owned()))
}

/// Write `key` to `path` with mode 0600, for tests and hand provisioning.
///
/// # Errors
///
/// The I/O error.
pub fn write_credential(path: &Path, key: &str) -> std::io::Result<()> {
    use std::io::Write;
    let mut options = std::fs::OpenOptions::new();
    options.write(true).create(true).truncate(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    let mut file = options.open(path)?;
    file.write_all(key.as_bytes())?;
    file.write_all(b"\n")?;
    file.sync_all()
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::{
        credential_path, read_credential, write_credential, CredentialProblem, DEFAULT_KEY_FILE,
    };

    #[test]
    fn the_env_names_the_file_first_then_the_manifest_then_the_state_dir() {
        let m = Path::new("/m");
        let s = Path::new("/s");
        assert_eq!(
            credential_path(Some("/e/k"), Some(Path::new("k")), m, s),
            Path::new("/e/k")
        );
        assert_eq!(
            credential_path(None, Some(Path::new("k")), m, s),
            Path::new("/m/k")
        );
        assert_eq!(
            credential_path(Some(" "), None, m, s),
            s.join(DEFAULT_KEY_FILE)
        );
    }

    #[test]
    fn only_a_private_file_with_one_key_is_read() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("jev.key");
        assert_eq!(read_credential(&path), Err(CredentialProblem::Missing));
        write_credential(&path, "test-only-0123456789").expect("write");
        let key = read_credential(&path).expect("read");
        assert_eq!(key.expose(), "test-only-0123456789");
        assert_eq!(format!("{key:?}"), "Credential(..)");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o640)).expect("chmod");
            assert_eq!(read_credential(&path), Err(CredentialProblem::Mode));
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600)).expect("chmod");
        }
        std::fs::write(&path, "OPENROUTER_API_KEY=test-only-0123456789\n").expect("write");
        assert_eq!(read_credential(&path), Err(CredentialProblem::Format));
        std::fs::write(&path, "sk-one\nsk-two-0123456789\n").expect("write");
        assert_eq!(read_credential(&path), Err(CredentialProblem::Format));
        std::fs::write(&path, "short\n").expect("write");
        assert_eq!(read_credential(&path), Err(CredentialProblem::Format));
    }
}
