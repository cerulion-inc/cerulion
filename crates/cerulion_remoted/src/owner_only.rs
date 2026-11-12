// SPDX-License-Identifier: AGPL-3.0-only
//! Reading a file another process on the robot leaves in the remoted state
//! directory for the daemon: the fetched revocation epoch and the pair-code
//! request.
//!
//! [`read_owner_only`] opens the directory without following a symbolic link at
//! its last component and inspects it through that handle, then opens the file
//! the same way and inspects it through its handle, then checks that the
//! directory's path still names the directory it inspected. The decision over
//! what it learned is [`owner_only_verdict`], which is pure.

use std::io::Read as _;
use std::os::unix::fs::{MetadataExt as _, OpenOptionsExt as _};
use std::path::Path;

/// Whose files [`read_owner_only`] accepts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ExpectedOwner {
    /// The user that owns the directory, who must own the file too.
    Directory,
    /// This user, who must own both the directory and the file.
    Uid(u32),
}

/// What [`read_owner_only`] learned about the directory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct DirFacts {
    /// The opened handle is a directory.
    pub(crate) is_dir: bool,
    /// Its mode bits.
    pub(crate) mode: u32,
    /// Its owner.
    pub(crate) uid: u32,
    /// After the file was opened, the directory's path still named the opened
    /// directory (the same device and inode).
    pub(crate) unchanged: bool,
}

/// What [`read_owner_only`] learned about the file, through its handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct FileFacts {
    /// It is a regular file.
    pub(crate) is_file: bool,
    /// Its link count.
    pub(crate) nlink: u64,
    /// Its owner.
    pub(crate) uid: u32,
    /// Its mode bits.
    pub(crate) mode: u32,
    /// Its length in bytes.
    pub(crate) len: u64,
}

/// Why [`owner_only_verdict`] refused a file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum OwnerOnlyRefusal {
    /// The directory's path names another directory than the one inspected.
    DirectoryReplaced,
    /// The directory is not a directory, or gives its group or others a
    /// permission bit.
    DirectoryNotOwnerOnly,
    /// The directory is not the expected owner's.
    DirectoryNotOwned,
    /// The file is not a regular file with a single link.
    NotRegularSingleLink,
    /// The file gives its group or others a permission bit, or is not the
    /// expected owner's.
    NotOwnerOnly,
    /// The file is longer than the bound.
    TooLarge {
        /// Its length in bytes.
        len: u64,
    },
}

impl OwnerOnlyRefusal {
    /// The refusal as a reason naming the file as `what`.
    fn describe(self, what: &str, max_bytes: u64) -> String {
        match self {
            OwnerOnlyRefusal::DirectoryReplaced => {
                format!("the state directory was replaced while {what} was opened")
            }
            OwnerOnlyRefusal::DirectoryNotOwnerOnly => {
                "the state directory is not owner-only".into()
            }
            OwnerOnlyRefusal::DirectoryNotOwned => {
                "the state directory is not owned by the user this daemon runs as".into()
            }
            OwnerOnlyRefusal::NotRegularSingleLink => {
                format!("{what} is not a regular single-link file")
            }
            OwnerOnlyRefusal::NotOwnerOnly => {
                format!("{what} is not owner-only and owned by this robot's user")
            }
            OwnerOnlyRefusal::TooLarge { len } => {
                format!("{what} is {len} bytes, above the {max_bytes}-byte bound")
            }
        }
    }
}

/// Accept a file when the directory is unchanged, a directory owned by the
/// expected owner with no group or other permission bits, and the file is a
/// regular single-link file of at most `max_bytes` bytes owned by the expected
/// owner with no group or other permission bits.
pub(crate) fn owner_only_verdict(
    dir: &DirFacts,
    file: &FileFacts,
    owner: ExpectedOwner,
    max_bytes: u64,
) -> Result<(), OwnerOnlyRefusal> {
    let owner = match owner {
        ExpectedOwner::Directory => dir.uid,
        ExpectedOwner::Uid(uid) => uid,
    };
    if !dir.unchanged {
        return Err(OwnerOnlyRefusal::DirectoryReplaced);
    }
    if !dir.is_dir || dir.mode & 0o077 != 0 {
        return Err(OwnerOnlyRefusal::DirectoryNotOwnerOnly);
    }
    if dir.uid != owner {
        return Err(OwnerOnlyRefusal::DirectoryNotOwned);
    }
    if !file.is_file || file.nlink != 1 {
        return Err(OwnerOnlyRefusal::NotRegularSingleLink);
    }
    if file.uid != owner || file.mode & 0o077 != 0 {
        return Err(OwnerOnlyRefusal::NotOwnerOnly);
    }
    if file.len > max_bytes {
        return Err(OwnerOnlyRefusal::TooLarge { len: file.len });
    }
    Ok(())
}

/// Read `path` when [`owner_only_verdict`] accepts it and its directory, read
/// as the module docs describe; `Ok(None)` when there is no file at `path`.
/// `what` names the file in a refusal. The bytes read are bounded by
/// `max_bytes`, and a file that grew past it after its length was read is
/// refused.
pub(crate) fn read_owner_only(
    path: &Path,
    max_bytes: u64,
    what: &str,
    owner: ExpectedOwner,
) -> Result<Option<Vec<u8>>, String> {
    let dir_path = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let dir = match std::fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_DIRECTORY | libc::O_NOFOLLOW)
        .open(dir_path)
    {
        Ok(dir) => dir,
        // A symbolic link or a non-directory at the directory's path.
        Err(e) if matches!(e.raw_os_error(), Some(libc::ELOOP | libc::ENOTDIR)) => {
            return Err(OwnerOnlyRefusal::DirectoryNotOwnerOnly.describe(what, max_bytes));
        }
        Err(e) => return Err(format!("could not inspect the state directory: {e}")),
    };
    let dir_meta = dir
        .metadata()
        .map_err(|e| format!("could not inspect the state directory: {e}"))?;
    let file = match std::fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
        .open(path)
    {
        Ok(file) => file,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(format!("could not open {what}: {e}")),
    };
    let meta = file
        .metadata()
        .map_err(|e| format!("could not inspect {what}: {e}"))?;
    let at_path = std::fs::symlink_metadata(dir_path)
        .map_err(|e| format!("could not inspect the state directory: {e}"))?;
    let dir_facts = DirFacts {
        is_dir: dir_meta.is_dir(),
        mode: dir_meta.mode(),
        uid: dir_meta.uid(),
        unchanged: at_path.dev() == dir_meta.dev() && at_path.ino() == dir_meta.ino(),
    };
    let file_facts = FileFacts {
        is_file: meta.is_file(),
        nlink: meta.nlink(),
        uid: meta.uid(),
        mode: meta.mode(),
        len: meta.len(),
    };
    let verdict = |file_facts: &FileFacts| {
        owner_only_verdict(&dir_facts, file_facts, owner, max_bytes)
            .map_err(|refusal| refusal.describe(what, max_bytes))
    };
    verdict(&file_facts)?;
    let mut bytes = Vec::new();
    file.take(max_bytes + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| format!("could not read {what}: {e}"))?;
    verdict(&FileFacts {
        len: bytes.len() as u64,
        ..file_facts
    })?;
    Ok(Some(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::PermissionsExt as _;

    const OWNER: u32 = 501;
    const OTHER: u32 = 502;
    const MAX: u64 = 256;

    fn dir() -> DirFacts {
        DirFacts {
            is_dir: true,
            mode: 0o40700,
            uid: OWNER,
            unchanged: true,
        }
    }

    fn file() -> FileFacts {
        FileFacts {
            is_file: true,
            nlink: 1,
            uid: OWNER,
            mode: 0o100600,
            len: MAX,
        }
    }

    #[test]
    fn owner_only_verdict_matches_the_oracle_cases() {
        use OwnerOnlyRefusal as R;
        let by_dir = ExpectedOwner::Directory;
        let by_uid = ExpectedOwner::Uid(OWNER);
        let cases: Vec<(DirFacts, FileFacts, ExpectedOwner, Result<(), R>)> = vec![
            (dir(), file(), by_dir, Ok(())),
            (dir(), file(), by_uid, Ok(())),
            (
                dir(),
                FileFacts {
                    mode: 0o100400,
                    ..file()
                },
                by_uid,
                Ok(()),
            ),
            (
                DirFacts {
                    unchanged: false,
                    ..dir()
                },
                file(),
                by_uid,
                Err(R::DirectoryReplaced),
            ),
            (
                DirFacts {
                    is_dir: false,
                    ..dir()
                },
                file(),
                by_uid,
                Err(R::DirectoryNotOwnerOnly),
            ),
            (
                DirFacts {
                    mode: 0o40750,
                    ..dir()
                },
                file(),
                by_uid,
                Err(R::DirectoryNotOwnerOnly),
            ),
            (
                DirFacts {
                    mode: 0o40701,
                    ..dir()
                },
                file(),
                by_dir,
                Err(R::DirectoryNotOwnerOnly),
            ),
            // A directory owned by someone else than the daemon's user.
            (
                DirFacts {
                    uid: OTHER,
                    ..dir()
                },
                FileFacts {
                    uid: OTHER,
                    ..file()
                },
                by_uid,
                Err(R::DirectoryNotOwned),
            ),
            // A directory that is neither owner-only nor the daemon's user's
            // is refused for its mode.
            (
                DirFacts {
                    uid: OTHER,
                    mode: 0o40750,
                    ..dir()
                },
                file(),
                by_uid,
                Err(R::DirectoryNotOwnerOnly),
            ),
            // The same pair is accepted when the directory's owner is the rule.
            (
                DirFacts {
                    uid: OTHER,
                    ..dir()
                },
                FileFacts {
                    uid: OTHER,
                    ..file()
                },
                by_dir,
                Ok(()),
            ),
            (
                dir(),
                FileFacts {
                    is_file: false,
                    ..file()
                },
                by_uid,
                Err(R::NotRegularSingleLink),
            ),
            (
                dir(),
                FileFacts { nlink: 2, ..file() },
                by_uid,
                Err(R::NotRegularSingleLink),
            ),
            (
                dir(),
                FileFacts { nlink: 0, ..file() },
                by_uid,
                Err(R::NotRegularSingleLink),
            ),
            // A file owned by another user than the directory's and the daemon's.
            (
                dir(),
                FileFacts {
                    uid: OTHER,
                    ..file()
                },
                by_uid,
                Err(R::NotOwnerOnly),
            ),
            (
                dir(),
                FileFacts {
                    uid: OTHER,
                    ..file()
                },
                by_dir,
                Err(R::NotOwnerOnly),
            ),
            (
                dir(),
                FileFacts {
                    mode: 0o100640,
                    ..file()
                },
                by_uid,
                Err(R::NotOwnerOnly),
            ),
            (
                dir(),
                FileFacts {
                    mode: 0o100604,
                    ..file()
                },
                by_uid,
                Err(R::NotOwnerOnly),
            ),
            (
                dir(),
                FileFacts {
                    len: MAX + 1,
                    ..file()
                },
                by_uid,
                Err(R::TooLarge { len: MAX + 1 }),
            ),
        ];
        for (dir, file, owner, want) in cases {
            assert_eq!(
                owner_only_verdict(&dir, &file, owner, MAX),
                want,
                "{dir:?} {file:?} {owner:?}"
            );
        }
    }

    #[test]
    fn each_refusal_names_the_file_as_the_caller_does() {
        let what = "the fetched epoch";
        let cases = [
            (
                OwnerOnlyRefusal::DirectoryReplaced,
                "the state directory was replaced while the fetched epoch was opened",
            ),
            (
                OwnerOnlyRefusal::DirectoryNotOwnerOnly,
                "the state directory is not owner-only",
            ),
            (
                OwnerOnlyRefusal::DirectoryNotOwned,
                "the state directory is not owned by the user this daemon runs as",
            ),
            (
                OwnerOnlyRefusal::NotRegularSingleLink,
                "the fetched epoch is not a regular single-link file",
            ),
            (
                OwnerOnlyRefusal::NotOwnerOnly,
                "the fetched epoch is not owner-only and owned by this robot's user",
            ),
            (
                OwnerOnlyRefusal::TooLarge { len: 1_048_577 },
                "the fetched epoch is 1048577 bytes, above the 1048576-byte bound",
            ),
        ];
        for (refusal, want) in cases {
            assert_eq!(refusal.describe(what, 1_048_576), want);
        }
    }

    fn private_dir(root: &Path) -> std::path::PathBuf {
        let dir = root.join("remoted");
        std::fs::create_dir(&dir).unwrap();
        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o700)).unwrap();
        dir
    }

    fn write_mode(path: &Path, bytes: &[u8], mode: u32) {
        std::fs::write(path, bytes).unwrap();
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(mode)).unwrap();
    }

    #[test]
    fn an_owner_only_file_is_read_and_a_missing_one_is_none() {
        let root = tempfile::tempdir().unwrap();
        let dir = private_dir(root.path());
        let path = dir.join("f");
        assert_eq!(
            read_owner_only(&path, MAX, "the file", ExpectedOwner::Directory),
            Ok(None)
        );
        write_mode(&path, b"payload", 0o600);
        let uid = std::fs::metadata(&dir).unwrap().uid();
        for owner in [ExpectedOwner::Directory, ExpectedOwner::Uid(uid)] {
            assert_eq!(
                read_owner_only(&path, MAX, "the file", owner),
                Ok(Some(b"payload".to_vec()))
            );
        }
    }

    #[test]
    fn a_symlinked_or_missing_state_directory_is_refused() {
        let root = tempfile::tempdir().unwrap();
        let dir = private_dir(root.path());
        write_mode(&dir.join("f"), b"payload", 0o600);
        let link = root.path().join("link");
        std::os::unix::fs::symlink(&dir, &link).unwrap();
        assert_eq!(
            read_owner_only(&link.join("f"), MAX, "the file", ExpectedOwner::Directory),
            Err("the state directory is not owner-only".to_string())
        );
        let missing = read_owner_only(
            &root.path().join("absent").join("f"),
            MAX,
            "the file",
            ExpectedOwner::Directory,
        )
        .unwrap_err();
        assert!(
            missing.starts_with("could not inspect the state directory: "),
            "{missing}"
        );
    }

    #[test]
    fn a_symlinked_file_is_refused_at_the_open() {
        let root = tempfile::tempdir().unwrap();
        let dir = private_dir(root.path());
        write_mode(&dir.join("target"), b"payload", 0o600);
        std::os::unix::fs::symlink(dir.join("target"), dir.join("f")).unwrap();
        let refused =
            read_owner_only(&dir.join("f"), MAX, "the file", ExpectedOwner::Directory).unwrap_err();
        assert!(
            refused.starts_with("could not open the file: "),
            "{refused}"
        );
    }

    #[test]
    fn a_file_at_the_bound_is_read_and_one_past_it_is_refused() {
        let root = tempfile::tempdir().unwrap();
        let dir = private_dir(root.path());
        let path = dir.join("f");
        write_mode(&path, &[b'x'; MAX as usize], 0o600);
        assert_eq!(
            read_owner_only(&path, MAX, "the file", ExpectedOwner::Directory)
                .unwrap()
                .map(|b| b.len()),
            Some(MAX as usize)
        );
        write_mode(&path, &[b'x'; MAX as usize + 1], 0o600);
        assert_eq!(
            read_owner_only(&path, MAX, "the file", ExpectedOwner::Directory),
            Err("the file is 257 bytes, above the 256-byte bound".to_string())
        );
    }
}
