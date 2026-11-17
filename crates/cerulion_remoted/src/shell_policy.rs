// SPDX-License-Identifier: AGPL-3.0-only
//! Durable owner policy and the immutable robot-local shell prohibition.

use std::fs::{self, OpenOptions};
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use cerud::error::{CerudError, CerudResult};
use cerud::verbs::VerbHandler;
use serde::{Deserialize, Serialize};

static POLICY_NONCE: AtomicU64 = AtomicU64::new(0);

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct StoredPolicy {
    enabled: bool,
}

/// Owner settings shared by the policy verb and every live session's authorizer.
#[derive(Debug)]
pub struct ShellPolicy {
    path: PathBuf,
    enabled: AtomicBool,
    hard_off: bool,
    writer: Mutex<()>,
}

impl ShellPolicy {
    /// Load policy. A missing file uses the default; every other read failure disables access.
    pub fn load(path: PathBuf, hard_off: bool) -> Arc<Self> {
        let enabled = match read_policy(&path) {
            Ok(enabled) => enabled,
            Err(error) => {
                tracing::warn!(path = %path.display(), error = %error, "remote shell policy could not be read; shell and exec disabled");
                false
            }
        };
        Arc::new(Self {
            path,
            enabled: AtomicBool::new(enabled),
            hard_off,
            writer: Mutex::new(()),
        })
    }

    /// The stored owner setting, before the robot-local prohibition is applied.
    pub fn enabled(&self) -> bool {
        self.enabled.load(Ordering::Acquire)
    }

    /// Whether the daemon was configured to prohibit all remote shell access.
    pub fn hard_off(&self) -> bool {
        self.hard_off
    }

    /// Return the effective policy without disk I/O on the reauthorization path.
    pub fn allowed(&self) -> bool {
        !self.hard_off && self.enabled()
    }

    /// Durably store the owner setting before publishing it to live sessions.
    pub fn set_enabled(&self, enabled: bool) -> io::Result<()> {
        if enabled && self.hard_off {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "remote shell is disabled by the robot-local configuration",
            ));
        }
        let _guard = self.writer.lock().map_err(|_| {
            self.enabled.store(false, Ordering::Release);
            io::Error::other("remote shell policy lock poisoned")
        })?;
        let result = self.persist(enabled);
        self.enabled
            .store(result.is_ok() && enabled, Ordering::Release);
        result
    }

    fn persist(&self, enabled: bool) -> io::Result<()> {
        let parent = self
            .path
            .parent()
            .ok_or_else(|| io::Error::other("remote shell policy has no parent directory"))?;
        let mut builder = fs::DirBuilder::new();
        builder.recursive(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::DirBuilderExt;
            builder.mode(0o700);
        }
        builder.create(parent)?;
        let nonce = POLICY_NONCE.fetch_add(1, Ordering::Relaxed);
        let temporary = parent.join(format!(
            ".remote-shell-policy-{}-{nonce}.tmp",
            std::process::id()
        ));
        let mut options = OpenOptions::new();
        options.write(true).create_new(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        let mut file = options.open(&temporary)?;
        let result = (|| {
            serde_json::to_writer(&mut file, &StoredPolicy { enabled })?;
            file.write_all(b"\n")?;
            file.sync_all()?;
            fs::rename(&temporary, &self.path)?;
            // Persist both the rename and every newly created ancestor entry.
            // A failed directory sync cannot leave live authorization enabled.
            let directory = fs::canonicalize(parent)?;
            for ancestor in directory.ancestors() {
                fs::File::open(ancestor)?.sync_all()?;
            }
            Ok::<_, io::Error>(())
        })();
        if result.is_err() {
            let _ = fs::remove_file(&temporary);
        }
        result
    }
}

fn read_policy(path: &Path) -> io::Result<bool> {
    read_policy_with(path, open_policy)
}

fn open_policy(path: &Path) -> io::Result<fs::File> {
    let mut options = OpenOptions::new();
    options.read(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        // A replacement symlink or FIFO must not redirect or block this read.
        options.custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK);
    }
    options.open(path)
}

fn read_policy_with(
    path: &Path,
    open: impl FnOnce(&Path) -> io::Result<fs::File>,
) -> io::Result<bool> {
    let metadata = match fs::symlink_metadata(path) {
        Ok(metadata) => metadata,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(true),
        Err(error) => return Err(error),
    };
    validate_policy_metadata(&metadata)?;
    let file = open(path)?;
    let opened = file.metadata()?;
    validate_policy_metadata(&opened)?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        if (metadata.dev(), metadata.ino()) != (opened.dev(), opened.ino()) {
            return Err(io::Error::other(
                "remote shell policy changed while opening",
            ));
        }
    }
    let mut bytes = Vec::new();
    file.take(4097).read_to_end(&mut bytes)?;
    if bytes.len() > 4096 {
        return Err(io::Error::other(
            "remote shell policy exceeds its size limit",
        ));
    }
    let policy: StoredPolicy = serde_json::from_slice(&bytes)?;
    Ok(policy.enabled)
}

fn validate_policy_metadata(metadata: &fs::Metadata) -> io::Result<()> {
    if !metadata.is_file() || metadata.len() > 4096 {
        return Err(io::Error::other(
            "remote shell policy must be a small regular file",
        ));
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        if metadata.permissions().mode() & 0o777 != 0o600 {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "remote shell policy must have mode 0600",
            ));
        }
    }
    Ok(())
}

/// Owner-only `remote-shell-policy` get/set handler.
pub struct ShellPolicyVerb {
    policy: Arc<ShellPolicy>,
}

impl ShellPolicyVerb {
    /// Use the same live policy handle as session authorization.
    pub fn new(policy: Arc<ShellPolicy>) -> Self {
        Self { policy }
    }
}

impl VerbHandler for ShellPolicyVerb {
    fn name(&self) -> &'static str {
        crate::verbs::REMOTE_SHELL_POLICY
    }

    fn execute(&self, args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        #[derive(Deserialize)]
        #[serde(deny_unknown_fields)]
        struct Args {
            enabled: Option<bool>,
        }
        let args: Args = serde_json::from_value(args.clone())
            .map_err(|error| CerudError::Verb(error.to_string()))?;
        if let Some(enabled) = args.enabled {
            self.policy.set_enabled(enabled).map_err(CerudError::Io)?;
        }
        Ok(
            serde_json::json!({"enabled": self.policy.enabled(), "hard_off": self.policy.hard_off(), "effective": self.policy.allowed()}),
        )
    }

    fn is_mutating(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_policy_defaults_on_and_durable_off_survives_reload() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("policy.json");
        let policy = ShellPolicy::load(path.clone(), false);
        assert!(policy.allowed());
        policy.set_enabled(false).unwrap();
        assert!(!policy.allowed());
        assert!(!ShellPolicy::load(path.clone(), false).allowed());
        policy.set_enabled(true).unwrap();
        assert!(ShellPolicy::load(path, false).allowed());
    }

    #[test]
    fn failed_off_before_temporary_file_creation_disables_live_access() {
        let dir = tempfile::tempdir().unwrap();
        let parent = dir.path().join("state");
        let policy = ShellPolicy::load(parent.join("policy.json"), false);
        assert!(policy.allowed());
        std::fs::write(&parent, b"not a directory").unwrap();
        assert!(policy.set_enabled(false).is_err());
        assert!(!policy.allowed());
    }

    #[test]
    fn off_in_a_new_nested_state_directory_survives_reload() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("state/remoted/policy.json");
        let policy = ShellPolicy::load(path.clone(), false);
        assert!(policy.allowed());
        policy.set_enabled(false).unwrap();
        assert!(!ShellPolicy::load(path, false).allowed());
    }

    #[test]
    fn hard_off_cannot_be_overridden_by_owner_setting() {
        let dir = tempfile::tempdir().unwrap();
        let policy = ShellPolicy::load(dir.path().join("policy.json"), true);
        assert!(!policy.allowed());
        assert_eq!(
            policy.set_enabled(true).unwrap_err().kind(),
            io::ErrorKind::PermissionDenied
        );
        assert!(!policy.allowed());
    }

    #[test]
    fn corrupt_and_symlinked_policy_fail_closed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("policy.json");
        fs::write(&path, b"broken").unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&path, fs::Permissions::from_mode(0o600)).unwrap();
        }
        assert!(!ShellPolicy::load(path.clone(), false).allowed());
        #[cfg(unix)]
        {
            let link = dir.path().join("link.json");
            std::os::unix::fs::symlink(&path, &link).unwrap();
            assert!(!ShellPolicy::load(link, false).allowed());
        }
    }

    #[test]
    fn failed_policy_write_never_publishes_on() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("policy.json");
        let policy = ShellPolicy::load(path.clone(), false);
        policy.set_enabled(false).unwrap();
        fs::remove_file(&path).unwrap();
        fs::create_dir(&path).unwrap();
        assert!(policy.set_enabled(true).is_err());
        assert!(!policy.allowed());
    }

    #[test]
    fn invalid_policy_shapes_and_oversized_files_disable_access() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("policy.json");
        let policy = ShellPolicy::load(path.clone(), false);
        policy.set_enabled(true).unwrap();
        for bytes in [
            b"".as_slice(),
            br#"{}"#,
            br#"{"enabled":null}"#,
            br#"{"enabled":"true"}"#,
            br#"{"enabled":true,"extra":false}"#,
            br#"{"enabled":true,"enabled":false}"#,
        ] {
            fs::write(&path, bytes).unwrap();
            assert!(!ShellPolicy::load(path.clone(), false).allowed());
        }
        let mut oversized = br#"{"enabled":true}"#.to_vec();
        oversized.resize(4097, b' ');
        fs::write(&path, oversized).unwrap();
        assert!(!ShellPolicy::load(path, false).allowed());
    }

    #[test]
    fn vanished_policy_after_metadata_does_not_default_on() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("policy.json");
        ShellPolicy::load(path.clone(), false)
            .set_enabled(false)
            .unwrap();
        let result = read_policy_with(&path, |path| {
            fs::remove_file(path)?;
            open_policy(path)
        });
        assert_eq!(result.unwrap_err().kind(), io::ErrorKind::NotFound);
    }

    #[cfg(unix)]
    #[test]
    fn replaced_regular_or_symlinked_policy_is_refused() {
        for symlink in [false, true] {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("policy.json");
            let replacement = dir.path().join("replacement.json");
            ShellPolicy::load(path.clone(), false)
                .set_enabled(false)
                .unwrap();
            ShellPolicy::load(replacement.clone(), false)
                .set_enabled(true)
                .unwrap();
            let result = read_policy_with(&path, |path| {
                if symlink {
                    fs::remove_file(path)?;
                    std::os::unix::fs::symlink(&replacement, path)?;
                } else {
                    fs::rename(&replacement, path)?;
                }
                open_policy(path)
            });
            assert!(result.is_err());
        }
    }

    #[cfg(unix)]
    #[test]
    fn descriptor_permissions_are_checked_after_open() {
        use std::os::unix::fs::PermissionsExt;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("policy.json");
        ShellPolicy::load(path.clone(), false)
            .set_enabled(true)
            .unwrap();
        let result = read_policy_with(&path, |path| {
            let file = open_policy(path)?;
            file.set_permissions(fs::Permissions::from_mode(0o644))?;
            Ok(file)
        });
        assert_eq!(result.unwrap_err().kind(), io::ErrorKind::PermissionDenied);
    }

    #[test]
    fn poisoned_writer_disables_the_live_policy() {
        let dir = tempfile::tempdir().unwrap();
        let policy = ShellPolicy::load(dir.path().join("policy.json"), false);
        let writer = Arc::clone(&policy);
        assert!(std::thread::spawn(move || {
            let _guard = writer.writer.lock().unwrap();
            panic!("poison the policy writer");
        })
        .join()
        .is_err());
        assert!(policy.set_enabled(true).is_err());
        assert!(!policy.allowed());
    }

    #[test]
    fn policy_handler_reports_effective_state_and_rejects_unknown_arguments() {
        let dir = tempfile::tempdir().unwrap();
        let policy = ShellPolicy::load(dir.path().join("policy.json"), false);
        let handler = ShellPolicyVerb::new(Arc::clone(&policy));
        assert_eq!(handler.name(), "remote-shell-policy");
        assert!(handler.is_mutating());
        assert_eq!(
            handler.execute(&serde_json::json!({})).unwrap(),
            serde_json::json!({"enabled": true, "hard_off": false, "effective": true})
        );
        assert_eq!(
            handler
                .execute(&serde_json::json!({"enabled": false}))
                .unwrap(),
            serde_json::json!({"enabled": false, "hard_off": false, "effective": false})
        );
        for args in [
            serde_json::json!({"enabled": "true"}),
            serde_json::json!({"extra": true}),
        ] {
            assert!(handler.execute(&args).is_err());
            assert!(!policy.allowed());
        }
    }
}
