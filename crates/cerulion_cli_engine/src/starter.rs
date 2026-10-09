// SPDX-License-Identifier: AGPL-3.0-only
//! Complete example source shipped inside the CLI, with no network retrieval.

use std::path::{Component, Path, PathBuf};
use std::time::{Duration, SystemTime};

use crate::error::{CliError, CliResult};
use crate::workspace::{scaffold_workspace, CerulionWorkspace};

const STAGING_PREFIX: &str = ".cerulion-starter-";
const STAGING_SUFFIX: &str = ".tmp";
/// Written into every container this module creates, beside the payload, and
/// holding the container's own directory name. The sweep deletes a populated
/// directory only when its marker names it, so a directory that merely carries
/// a container's name (a workspace a user renamed, a hand-made folder), even
/// one holding a file by this name, is kept.
const STAGING_MARKER: &str = "cerulion-starter-staging";
/// A genuine marker is one container name plus a newline; anything larger is
/// not ours and is never even read.
const STAGING_MARKER_MAX_LEN: u64 = 256;
/// A staging container older than this was abandoned by a killed run:
/// population writes a handful of small files, so a live creator never holds
/// its container anywhere near this long.
const STALE_STAGING_AGE: Duration = Duration::from_secs(10 * 60);

/// A starter's source is compiled into the same package as its CLI.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Starter {
    /// Synthetic laser scanner and data-triggered safety controller.
    ObstacleAvoidance,
}

impl Starter {
    /// Stable name used by the CLI and the provenance manifest.
    pub fn name(self) -> &'static str {
        match self {
            Self::ObstacleAvoidance => "obstacle_avoidance",
        }
    }
}

const OBSTACLE_FILES: &[(&str, &str)] = &[
    (
        "graphs/obstacle_avoidance.yaml",
        include_str!("starters/obstacle_avoidance/graphs/obstacle_avoidance.yaml"),
    ),
    (
        "nodes/laser_scanner/Cargo.toml",
        include_str!("starters/obstacle_avoidance/nodes/laser_scanner/Cargo.toml.txt"),
    ),
    (
        "nodes/laser_scanner/src/lib.rs",
        include_str!("starters/obstacle_avoidance/nodes/laser_scanner/src/lib.rs"),
    ),
    (
        "nodes/laser_scanner/src/tests.rs",
        include_str!("starters/obstacle_avoidance/nodes/laser_scanner/src/tests.rs"),
    ),
    (
        "nodes/safety_controller/Cargo.toml",
        include_str!("starters/obstacle_avoidance/nodes/safety_controller/Cargo.toml.txt"),
    ),
    (
        "nodes/safety_controller/src/lib.rs",
        include_str!("starters/obstacle_avoidance/nodes/safety_controller/src/lib.rs"),
    ),
    (
        "nodes/safety_controller/src/tests.rs",
        include_str!("starters/obstacle_avoidance/nodes/safety_controller/src/tests.rs"),
    ),
    (
        "README.md",
        include_str!("starters/obstacle_avoidance/README.md"),
    ),
];

/// Create a complete starter workspace without replacing any existing path.
/// Source retrieval is offline. Dependencies follow the installed binary's
/// checkout-or-exact-registry selection, just like an empty workspace.
pub fn workspace_create_with_starter(
    parent: &Path,
    name: &str,
    starter: Starter,
) -> CliResult<CerulionWorkspace> {
    let ws = create_with(parent, name, |staging| {
        let ws = scaffold_workspace(staging)?;
        match starter {
            Starter::ObstacleAvoidance => {
                for &(relative, contents) in OBSTACLE_FILES {
                    let path = staging.join(relative);
                    if let Some(directory) = path.parent() {
                        std::fs::create_dir_all(directory)?;
                    }
                    std::fs::write(path, contents)?;
                }
            }
        }
        let provenance = format!(
            "starter = {:?}\ncli_version = {:?}\nrustc_release = {:?}\nrustc_fingerprint = {:?}\n",
            starter.name(),
            env!("CARGO_PKG_VERSION"),
            cerulion_core::RUSTC_RELEASE,
            cerulion_core::RUSTC_FINGERPRINT,
        );
        std::fs::write(staging.join("starter.toml"), provenance)?;
        Ok(ws)
    })?;
    tracing::info!(workspace = %name, path = %ws.root.display(), starter = starter.name(), "workspace created");
    Ok(ws)
}

fn create_with(
    parent: &Path,
    name: &str,
    populate: impl FnOnce(&Path) -> CliResult<CerulionWorkspace>,
) -> CliResult<CerulionWorkspace> {
    let mut components = Path::new(name).components();
    if !matches!(components.next(), Some(Component::Normal(_))) || components.next().is_some() {
        return Err(CliError::Validation(
            "Starter workspace name must be a single directory name, without a path.".to_owned(),
        ));
    }
    refuse_reserved_name(Path::new(name))?;
    let destination = parent.join(name);
    match std::fs::symlink_metadata(&destination) {
        Ok(_) => return Err(existing(&destination)),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => return Err(error.into()),
    }
    sweep_stale_staging(parent, SystemTime::now());
    let mut nonce = [0_u8; 16];
    getrandom::fill(&mut nonce).map_err(|error| std::io::Error::other(error.to_string()))?;
    let staging = Staging::create(parent.join(staging_name(u128::from_le_bytes(nonce))))?;
    let payload = staging.path.join("workspace");
    // The private outer container protects population; the payload inherits
    // the same umask-governed permissions as an ordinary workspace.
    std::fs::create_dir(&payload)?;
    let mut ws = populate(&payload)?;
    publish_directory(&payload, &destination)
        .map_err(|error| publish_error(parent, &destination, error))?;
    ws.root = destination.clone();
    ws.graphs_dir = destination.join("graphs");
    ws.nodes_dir = destination.join("nodes");
    ws.schemas_dir = destination.join("schemas");
    Ok(ws)
}

fn existing(path: &Path) -> CliError {
    CliError::WorkspaceExists {
        path: path.display().to_string(),
    }
}

/// The user's error for a failed publication. `AlreadyExists` is a competing
/// creator that won the destination. `EINVAL`, `ENOTSUP` and `EOPNOTSUPP` are
/// how a filesystem without a no-replace rename answers (some network, FUSE
/// and overlay filesystems on Linux; SMB and FAT volumes on macOS), so they
/// name the requirement instead of surfacing a bare "Invalid argument" after
/// a complete population. Every other error propagates unchanged.
fn publish_error(parent: &Path, destination: &Path, error: std::io::Error) -> CliError {
    if error.kind() == std::io::ErrorKind::AlreadyExists {
        return existing(destination);
    }
    if error
        .raw_os_error()
        .is_some_and(rename_noreplace_unsupported)
    {
        return CliError::Validation(format!(
            "The filesystem holding {} does not support atomic no-replace publication, \
             which a starter requires so that a competing workspace is never replaced. \
             Create the workspace under a local filesystem parent, then move it.",
            parent.display()
        ));
    }
    error.into()
}

#[cfg(unix)]
fn rename_noreplace_unsupported(code: i32) -> bool {
    use rustix::io::Errno;
    [Errno::INVAL, Errno::NOTSUP, Errno::OPNOTSUPP]
        .iter()
        .any(|errno| errno.raw_os_error() == code)
}

#[cfg(not(unix))]
fn rename_noreplace_unsupported(_code: i32) -> bool {
    false
}

fn staging_name(nonce: u128) -> String {
    format!("{STAGING_PREFIX}{nonce:032x}{STAGING_SUFFIX}")
}

/// Exactly the names [`staging_name`] produces: the prefix, 32 lowercase hex
/// digits, the suffix.
fn is_staging_name(name: &str) -> bool {
    name.strip_prefix(STAGING_PREFIX)
        .and_then(|rest| rest.strip_suffix(STAGING_SUFFIX))
        .is_some_and(|hex| {
            hex.len() == 32
                && hex
                    .bytes()
                    .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
        })
}

/// Refuse a workspace path any component of which is a staging container
/// name, on every creation route (`workspace create` with or without a
/// starter, `workspace init`). A workspace under such a path would sit where
/// the sweep looks for abandoned containers; refusing the path keeps the
/// sweep's territory free of user data by construction, and the marker check
/// in [`sweep_stale_staging`] protects whatever reaches that territory by
/// other means (a rename, a hand-made folder).
pub(crate) fn refuse_reserved_name(path: &Path) -> CliResult<()> {
    let reserved = path.components().any(|component| {
        matches!(component, Component::Normal(part) if part.to_str().is_some_and(is_staging_name))
    });
    if reserved {
        return Err(CliError::Validation(format!(
            "Workspace path '{}' contains a name reserved for starter staging directories; choose another name.",
            path.display()
        )));
    }
    Ok(())
}

/// The marker authenticates a container only when it is a regular file, no
/// larger than [`STAGING_MARKER_MAX_LEN`], whose content is the container's
/// own directory name. A renamed workspace that happens to hold a file by the
/// marker's name does not name the container it sits in, so it never passes.
fn marker_names_container(container: &Path, name: &str) -> bool {
    let marker = container.join(STAGING_MARKER);
    let Ok(metadata) = std::fs::symlink_metadata(&marker) else {
        return false;
    };
    if !metadata.is_file() || metadata.len() > STAGING_MARKER_MAX_LEN {
        return false;
    }
    std::fs::read_to_string(&marker).is_ok_and(|content| content.trim_end() == name)
}

/// Remove staging containers an earlier run left behind. The `Drop` guard
/// cleans up every failure that unwinds, but a kill (SIGKILL, or a Ctrl+C
/// delivered before any handler runs) ends the process without it, so the
/// next starter creation in the same parent sweeps them. A candidate is a
/// directory (never a symlink) whose name [`is_staging_name`] and whose
/// modification time is at least [`STALE_STAGING_AGE`] before `now` (a
/// younger one may belong to a concurrent creator still populating it). A
/// candidate whose marker names it ([`marker_names_container`]) is removed
/// with its contents. Any other candidate is removed only if it is EMPTY,
/// through `remove_dir`, which refuses a directory holding anything: that
/// reclaims the container a kill between the mkdir and the marker write
/// leaves, and can never delete data. Sweeping is best effort; a container
/// that cannot be read or removed is reported and left alone.
fn sweep_stale_staging(parent: &Path, now: SystemTime) {
    let Ok(entries) = std::fs::read_dir(parent) else {
        return;
    };
    for entry in entries.flatten() {
        let Some(name) = entry.file_name().to_str().map(str::to_owned) else {
            continue;
        };
        if !is_staging_name(&name) {
            continue;
        }
        let path = entry.path();
        let Ok(metadata) = std::fs::symlink_metadata(&path) else {
            continue;
        };
        let stale = metadata.is_dir()
            && metadata
                .modified()
                .ok()
                .and_then(|modified| now.duration_since(modified).ok())
                .is_some_and(|age| age >= STALE_STAGING_AGE);
        if !stale {
            continue;
        }
        if marker_names_container(&path, &name) {
            match std::fs::remove_dir_all(&path) {
                Ok(()) => {
                    tracing::info!(path = %path.display(), "removed a stale starter staging directory")
                }
                Err(error) => tracing::warn!(
                    path = %path.display(),
                    error = %error,
                    "could not remove a stale starter staging directory"
                ),
            }
        } else {
            match std::fs::remove_dir(&path) {
                Ok(()) => tracing::info!(
                    path = %path.display(),
                    "removed an empty stale starter staging directory"
                ),
                Err(error) => tracing::debug!(
                    path = %path.display(),
                    error = %error,
                    "kept a directory with a starter staging name that is not an owned container"
                ),
            }
        }
    }
}

/// Same-filesystem atomic publication refuses files, directories and symlinks,
/// even when a concurrent creator wins after the initial existence check.
#[cfg(any(target_os = "linux", target_os = "macos"))]
fn publish_directory(staging: &Path, destination: &Path) -> std::io::Result<()> {
    rustix::fs::renameat_with(
        rustix::fs::CWD,
        staging,
        rustix::fs::CWD,
        destination,
        rustix::fs::RenameFlags::NOREPLACE,
    )
    .map_err(Into::into)
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
fn publish_directory(_staging: &Path, _destination: &Path) -> std::io::Result<()> {
    Err(std::io::Error::new(
        std::io::ErrorKind::Unsupported,
        "Starter workspace publication requires Linux or macOS.",
    ))
}

struct Staging {
    path: PathBuf,
}

impl Staging {
    fn create(path: PathBuf) -> std::io::Result<Self> {
        let mut builder = std::fs::DirBuilder::new();
        #[cfg(unix)]
        {
            use std::os::unix::fs::DirBuilderExt;
            builder.mode(0o700);
        }
        builder.create(&path)?;
        // Own the container before writing into it, so a failed marker write
        // still removes the directory on drop. A kill in between leaves an
        // empty directory, which the sweep reclaims without a marker.
        let staging = Self { path };
        let name = staging
            .path
            .file_name()
            .and_then(|name| name.to_str())
            .ok_or_else(|| std::io::Error::other("staging container path has no file name"))?;
        std::fs::write(staging.path.join(STAGING_MARKER), format!("{name}\n"))?;
        Ok(staging)
    }
}

impl Drop for Staging {
    fn drop(&mut self) {
        if let Err(error) = std::fs::remove_dir_all(&self.path) {
            if error.kind() != std::io::ErrorKind::NotFound {
                tracing::warn!(path = %self.path.display(), error = %error, "could not remove starter staging directory");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn released_documentation_routes_do_not_invoke_unavailable_starter_commands() {
        let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        for (relative, path) in [
            ("README.md", crate_dir.join("../../README.md")),
            (
                "docs/tutorials/01-getting-started.md",
                crate_dir.join("../../docs/tutorials/01-getting-started.md"),
            ),
        ] {
            let document = std::fs::read_to_string(path).unwrap();
            let (_, routes) = document.split_once("### Released CLI 1.0.0").unwrap();
            let (released, starter) = routes.split_once("### CLI builds with --starter").unwrap();
            assert!(released.contains("git clone --depth 1 --branch v1.0.0"));
            assert!(released.contains("cd cerulion-starter-source/examples/obstacle_avoidance"));
            for block in released.split("```bash").skip(1) {
                let commands = block.split_once("```").unwrap().0;
                assert!(
                    !commands.contains("--starter"),
                    "unavailable flag in released route: {relative}"
                );
            }
            let (capability_guard, commands) = starter.split_once("```bash").unwrap();
            assert!(capability_guard.contains(
                "Use this route only when `cerulion workspace create --help` lists `--starter`"
            ));
            assert!(commands.contains("--starter obstacle_avoidance"));
        }
    }

    #[test]
    fn complete_starter_matches_canonical_sources_and_exact_dependencies() {
        let root = tempfile::tempdir().unwrap();
        let ws =
            workspace_create_with_starter(root.path(), "demo", Starter::ObstacleAvoidance).unwrap();
        let canonical =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/obstacle_avoidance");
        fn source_paths(
            root: &Path,
            current: &Path,
            paths: &mut std::collections::BTreeSet<String>,
        ) {
            for entry in std::fs::read_dir(current).unwrap() {
                let path = entry.unwrap().path();
                if path.is_dir() {
                    source_paths(root, &path, paths);
                } else if matches!(
                    path.extension().and_then(|part| part.to_str()),
                    Some("rs" | "toml" | "yaml")
                ) {
                    paths.insert(
                        path.strip_prefix(root)
                            .unwrap()
                            .to_str()
                            .unwrap()
                            .replace('\\', "/"),
                    );
                }
            }
        }
        let mut required = std::collections::BTreeSet::new();
        for directory in ["nodes", "graphs"] {
            source_paths(&canonical, &canonical.join(directory), &mut required);
        }
        let bundled = OBSTACLE_FILES
            .iter()
            .filter(|(path, _)| *path != "README.md")
            .map(|(path, _)| path.to_string())
            .collect();
        assert_eq!(
            required, bundled,
            "every canonical source and graph file must be bundled"
        );
        for &(path, contents) in OBSTACLE_FILES {
            assert_eq!(
                std::fs::read_to_string(ws.root.join(path)).unwrap(),
                contents
            );
            if path != "README.md" {
                assert_eq!(
                    std::fs::read_to_string(canonical.join(path)).unwrap(),
                    contents,
                    "bundled source drift: {path}"
                );
            }
        }
        let provenance: toml::Table = std::fs::read_to_string(ws.root.join("starter.toml"))
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(provenance["starter"].as_str(), Some("obstacle_avoidance"));
        assert_eq!(
            provenance["cli_version"].as_str(),
            Some(env!("CARGO_PKG_VERSION"))
        );
        assert_eq!(
            provenance["rustc_fingerprint"].as_str(),
            Some(cerulion_core::RUSTC_FINGERPRINT)
        );
        assert!(ws.graphs_dir.join("obstacle_avoidance.yaml").is_file());
        assert!(ws.schemas_dir.is_dir());
        assert_eq!(std::fs::read_dir(root.path()).unwrap().count(), 1);
    }

    #[test]
    fn publish_errors_map_collisions_and_unsupported_filesystems_and_pass_the_rest() {
        let parent = Path::new("/parent");
        let destination = parent.join("demo");
        let collision = std::io::Error::from(std::io::ErrorKind::AlreadyExists);
        assert!(matches!(
            publish_error(parent, &destination, collision),
            CliError::WorkspaceExists { path } if path == "/parent/demo"
        ));
        #[cfg(unix)]
        for errno in [
            rustix::io::Errno::INVAL,
            rustix::io::Errno::NOTSUP,
            rustix::io::Errno::OPNOTSUPP,
        ] {
            let unsupported = std::io::Error::from_raw_os_error(errno.raw_os_error());
            let error = publish_error(parent, &destination, unsupported);
            assert!(
                matches!(&error, CliError::Validation(_)),
                "{errno:?}: {error}"
            );
            let message = error.to_string();
            assert!(message.contains("/parent"), "{message}");
            assert!(
                message.contains("atomic no-replace publication"),
                "{message}"
            );
            assert!(message.contains("local filesystem parent"), "{message}");
        }
        #[cfg(unix)]
        {
            let denied =
                std::io::Error::from_raw_os_error(rustix::io::Errno::ACCESS.raw_os_error());
            assert!(matches!(
                publish_error(parent, &destination, denied),
                CliError::Io(error) if error.kind() == std::io::ErrorKind::PermissionDenied
            ));
        }
        let missing = std::io::Error::from(std::io::ErrorKind::NotFound);
        assert!(matches!(
            publish_error(parent, &destination, missing),
            CliError::Io(error) if error.kind() == std::io::ErrorKind::NotFound
        ));
    }

    #[test]
    fn staging_names_round_trip_and_reject_every_hand_written_lookalike() {
        let name = staging_name(0x0123_4567_89ab_cdef_0123_4567_89ab_cdef);
        assert_eq!(
            name,
            ".cerulion-starter-0123456789abcdef0123456789abcdef.tmp"
        );
        assert!(is_staging_name(&name));
        assert!(is_staging_name(&staging_name(0)));
        assert!(is_staging_name(&staging_name(u128::MAX)));
        for lookalike in [
            "",
            ".cerulion-starter-.tmp",
            ".cerulion-starter-0123456789abcdef.tmp",
            ".cerulion-starter-0123456789ABCDEF0123456789ABCDEF.tmp",
            ".cerulion-starter-0123456789abcdef0123456789abcdeg.tmp",
            ".cerulion-starter-0123456789abcdef0123456789abcdef.tmp.bak",
            "cerulion-starter-0123456789abcdef0123456789abcdef.tmp",
            ".cerulion-starter-0123456789abcdef0123456789abcdef",
            "demo",
        ] {
            assert!(!is_staging_name(lookalike), "{lookalike}");
        }
    }

    /// A container exactly as a killed run leaves it: created by this module
    /// (so it carries the marker), a half-written payload inside, and the
    /// guard never ran.
    fn abandon_container(parent: &Path, nonce: u128, age: Duration) -> PathBuf {
        let staging = Staging::create(parent.join(staging_name(nonce))).unwrap();
        std::fs::create_dir(staging.path.join("workspace")).unwrap();
        std::fs::write(staging.path.join("workspace/partial"), "killed mid-write").unwrap();
        std::fs::File::open(&staging.path)
            .unwrap()
            .set_modified(SystemTime::now() - age)
            .unwrap();
        let path = staging.path.clone();
        std::mem::forget(staging);
        path
    }

    #[test]
    fn sweep_removes_only_stale_marked_staging_directories() {
        let parent = tempfile::tempdir().unwrap();
        let now = SystemTime::now();
        let old = STALE_STAGING_AGE + Duration::from_secs(60);
        let stale = abandon_container(parent.path(), 1, old);
        let young = abandon_container(parent.path(), 2, Duration::ZERO);
        let aged = |path: &Path| {
            std::fs::File::open(path)
                .unwrap()
                .set_modified(now - old)
                .unwrap();
        };
        // A workspace someone renamed to a container name: old, right name,
        // no marker.
        let renamed = parent.path().join(staging_name(3));
        std::fs::create_dir(&renamed).unwrap();
        std::fs::write(renamed.join("Cargo.toml"), "user source").unwrap();
        aged(&renamed);
        // The marker as a directory, not a file, proves nothing.
        let fake_marker = parent.path().join(staging_name(4));
        std::fs::create_dir_all(fake_marker.join(STAGING_MARKER)).unwrap();
        aged(&fake_marker);
        // A renamed workspace holding a file by the marker's name whose content
        // is not this directory's name: still user data.
        let forged = parent.path().join(staging_name(10));
        std::fs::create_dir(&forged).unwrap();
        std::fs::write(forged.join(STAGING_MARKER), "user source").unwrap();
        std::fs::write(forged.join("Cargo.toml"), "user source").unwrap();
        aged(&forged);
        // A marker naming ANOTHER container does not authenticate this one.
        let mismatched = parent.path().join(staging_name(11));
        std::fs::create_dir(&mismatched).unwrap();
        std::fs::write(
            mismatched.join(STAGING_MARKER),
            format!("{}\n", staging_name(1)),
        )
        .unwrap();
        std::fs::write(mismatched.join("Cargo.toml"), "user source").unwrap();
        aged(&mismatched);
        // A kill between the container mkdir and its marker write: empty, so
        // reclaimed without a marker.
        let empty_unmarked = parent.path().join(staging_name(12));
        std::fs::create_dir(&empty_unmarked).unwrap();
        aged(&empty_unmarked);
        let lookalike = parent.path().join(".cerulion-starter-notes.tmp");
        std::fs::create_dir(&lookalike).unwrap();
        let file = parent.path().join(staging_name(5));
        std::fs::write(&file, "user source").unwrap();
        let user_dir = parent.path().join("demo");
        std::fs::create_dir(&user_dir).unwrap();
        std::fs::write(user_dir.join("keep"), "user source").unwrap();
        #[cfg(unix)]
        let link = {
            let link = parent.path().join(staging_name(6));
            std::os::unix::fs::symlink(&user_dir, &link).unwrap();
            link
        };
        sweep_stale_staging(parent.path(), now);
        assert!(!stale.exists(), "old, named, marked: swept");
        assert!(young.join(STAGING_MARKER).is_file(), "young: kept");
        assert_eq!(
            std::fs::read_to_string(renamed.join("Cargo.toml")).unwrap(),
            "user source",
            "no marker: kept"
        );
        assert!(fake_marker.join(STAGING_MARKER).is_dir());
        assert_eq!(
            std::fs::read_to_string(forged.join("Cargo.toml")).unwrap(),
            "user source",
            "marker with foreign content: kept"
        );
        assert_eq!(
            std::fs::read_to_string(forged.join(STAGING_MARKER)).unwrap(),
            "user source"
        );
        assert_eq!(
            std::fs::read_to_string(mismatched.join("Cargo.toml")).unwrap(),
            "user source",
            "marker naming another container: kept"
        );
        assert!(!empty_unmarked.exists(), "empty, old, named: reclaimed");
        assert!(lookalike.is_dir());
        assert_eq!(std::fs::read_to_string(&file).unwrap(), "user source");
        assert_eq!(
            std::fs::read_to_string(user_dir.join("keep")).unwrap(),
            "user source"
        );
        #[cfg(unix)]
        assert_eq!(std::fs::read_link(link).unwrap(), user_dir);
        std::fs::remove_dir_all(young).unwrap();
    }

    #[test]
    fn starter_creation_sweeps_a_container_an_earlier_kill_left_behind() {
        let parent = tempfile::tempdir().unwrap();
        let abandoned = abandon_container(
            parent.path(),
            7,
            STALE_STAGING_AGE + Duration::from_secs(60),
        );
        let fresh = abandon_container(parent.path(), 8, Duration::ZERO);
        workspace_create_with_starter(parent.path(), "demo", Starter::ObstacleAvoidance).unwrap();
        assert!(!abandoned.exists());
        assert!(fresh.is_dir(), "a container still being populated is kept");
        assert!(parent.path().join("demo/starter.toml").is_file());
        assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 2);
        std::fs::remove_dir_all(fresh).unwrap();
    }

    #[test]
    fn container_names_are_refused_as_workspace_names_and_never_swept_as_workspaces() {
        let parent = tempfile::tempdir().unwrap();
        let reserved = staging_name(9);
        for name in [
            reserved.clone(),
            format!("nested/{reserved}"),
            format!("{reserved}/child"),
        ] {
            let error = crate::workspace::workspace_create(parent.path(), &name).unwrap_err();
            assert!(
                error.to_string().contains("reserved for starter staging"),
                "{name}"
            );
            let error = crate::workspace::workspace_init(&parent.path().join(&name)).unwrap_err();
            assert!(
                error.to_string().contains("reserved for starter staging"),
                "init {name}"
            );
        }
        let error =
            workspace_create_with_starter(parent.path(), &reserved, Starter::ObstacleAvoidance)
                .unwrap_err();
        assert!(error.to_string().contains("reserved for starter staging"));
        assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 0);
        // A completed workspace that reached the reserved name anyway (an
        // older CLI, a rename) survives a later starter creation beside it.
        let moved = crate::workspace::workspace_create(parent.path(), "to_move").unwrap();
        let manifest = std::fs::read_to_string(moved.root.join("Cargo.toml")).unwrap();
        let renamed = parent.path().join(&reserved);
        std::fs::rename(&moved.root, &renamed).unwrap();
        std::fs::File::open(&renamed)
            .unwrap()
            .set_modified(SystemTime::now() - STALE_STAGING_AGE - Duration::from_secs(60))
            .unwrap();
        workspace_create_with_starter(parent.path(), "demo", Starter::ObstacleAvoidance).unwrap();
        assert_eq!(
            std::fs::read_to_string(renamed.join("Cargo.toml")).unwrap(),
            manifest
        );
        assert!(renamed.join("graphs").is_dir());
        assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 2);
    }

    #[test]
    fn staging_collision_preserves_tree_this_call_did_not_create() {
        let parent = tempfile::tempdir().unwrap();
        let path = parent.path().join("staging");
        std::fs::create_dir(&path).unwrap();
        std::fs::write(path.join("keep"), "user source").unwrap();
        assert!(Staging::create(path.clone()).is_err());
        assert_eq!(
            std::fs::read_to_string(path.join("keep")).unwrap(),
            "user source"
        );
    }

    #[test]
    fn successful_publication_cleans_container_and_preserves_destination() {
        let parent = tempfile::tempdir().unwrap();
        let path = parent.path().join("staging");
        let staging = Staging::create(path.clone()).unwrap();
        let payload = path.join("workspace");
        std::fs::create_dir(&payload).unwrap();
        std::fs::write(payload.join("keep"), "complete user source").unwrap();
        let destination = parent.path().join("published");
        publish_directory(&payload, &destination).unwrap();
        drop(staging);
        assert!(!path.exists());
        assert_eq!(
            std::fs::read_to_string(destination.join("keep")).unwrap(),
            "complete user source"
        );
        assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 1);
    }

    #[test]
    fn publication_failure_leaves_no_final_workspace_or_staging_tree() {
        let parent = tempfile::tempdir().unwrap();
        let error = create_with(parent.path(), "demo", |staging| {
            let ws = scaffold_workspace(staging)?;
            std::fs::remove_dir_all(staging)?;
            Ok(ws)
        })
        .unwrap_err();
        assert!(matches!(error, CliError::Io(_)));
        assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 0);
    }

    #[test]
    fn repeated_starter_creation_has_identical_source_and_metadata() {
        let parent = tempfile::tempdir().unwrap();
        let one = workspace_create_with_starter(parent.path(), "one", Starter::ObstacleAvoidance)
            .unwrap();
        let two = workspace_create_with_starter(parent.path(), "two", Starter::ObstacleAvoidance)
            .unwrap();
        for path in [
            "Cargo.toml",
            ".cargo/config.toml",
            ".gitignore",
            "starter.toml",
        ]
        .into_iter()
        .chain(OBSTACLE_FILES.iter().map(|(path, _)| *path))
        {
            assert_eq!(
                std::fs::read(one.root.join(path)).unwrap(),
                std::fs::read(two.root.join(path)).unwrap(),
                "nondeterministic artifact: {path}"
            );
        }
    }

    #[test]
    fn rejects_existing_files_and_directories_without_writing() {
        for directory in [false, true] {
            let parent = tempfile::tempdir().unwrap();
            let destination = parent.path().join("demo");
            if directory {
                std::fs::create_dir(&destination).unwrap();
                std::fs::write(destination.join("keep"), "user source").unwrap();
            } else {
                std::fs::write(&destination, "user source").unwrap();
            }
            assert!(matches!(
                workspace_create_with_starter(parent.path(), "demo", Starter::ObstacleAvoidance),
                Err(CliError::WorkspaceExists { .. })
            ));
            let original = if directory {
                destination.join("keep")
            } else {
                destination
            };
            assert_eq!(std::fs::read_to_string(original).unwrap(), "user source");
            assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 1);
        }
    }

    #[test]
    fn invalid_names_are_refused_before_creating_anything() {
        let parent = tempfile::tempdir().unwrap();
        for name in ["", ".", "..", "../escape", "/absolute", "nested/demo"] {
            let error =
                workspace_create_with_starter(parent.path(), name, Starter::ObstacleAvoidance)
                    .unwrap_err();
            assert!(error.to_string().contains("single directory name"));
            assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 0);
        }
    }

    #[test]
    fn failed_population_leaves_no_workspace_or_staging_tree() {
        let parent = tempfile::tempdir().unwrap();
        let error = create_with(parent.path(), "demo", |staging| {
            std::fs::write(staging.join("partial"), "incomplete source")?;
            Err(std::io::Error::new(std::io::ErrorKind::WriteZero, "injected write failure").into())
        })
        .unwrap_err();
        assert!(error.to_string().contains("injected write failure"));
        assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 0);
    }

    #[test]
    fn concurrent_directory_creation_wins_without_replacement() {
        let parent = tempfile::tempdir().unwrap();
        let destination = parent.path().join("demo");
        let error = create_with(parent.path(), "demo", |staging| {
            let ws = scaffold_workspace(staging)?;
            // An empty directory matters: ordinary rename would replace it.
            std::fs::create_dir(&destination)?;
            Ok(ws)
        })
        .unwrap_err();
        assert!(matches!(error, CliError::WorkspaceExists { .. }));
        assert_eq!(std::fs::read_dir(destination).unwrap().count(), 0);
        assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 1);
    }

    #[test]
    fn ordinary_workspace_creation_during_staging_is_preserved() {
        let parent = tempfile::tempdir().unwrap();
        let destination = parent.path().join("demo");
        let mut original = String::new();
        let error = create_with(parent.path(), "demo", |staging| {
            let ws = scaffold_workspace(staging)?;
            let ordinary = crate::workspace::workspace_create(parent.path(), "demo")?;
            let manifest = ordinary.root.join("Cargo.toml");
            original = std::fs::read_to_string(&manifest)?;
            original.push_str("\n# learner's ordinary workspace\n");
            std::fs::write(manifest, &original)?;
            Ok(ws)
        })
        .unwrap_err();
        assert!(matches!(error, CliError::WorkspaceExists { .. }));
        assert_eq!(
            std::fs::read_to_string(destination.join("Cargo.toml")).unwrap(),
            original
        );
        assert!(destination.join("graphs").is_dir());
        assert!(!destination.join("starter.toml").exists());
        assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 1);
    }

    #[cfg(unix)]
    #[test]
    fn dangling_symlink_is_preserved_before_and_during_publication() {
        for concurrent in [false, true] {
            let parent = tempfile::tempdir().unwrap();
            let destination = parent.path().join("demo");
            let error = if concurrent {
                create_with(parent.path(), "demo", |staging| {
                    let ws = scaffold_workspace(staging)?;
                    std::os::unix::fs::symlink("user-choice", &destination)?;
                    Ok(ws)
                })
                .unwrap_err()
            } else {
                std::os::unix::fs::symlink("user-choice", &destination).unwrap();
                workspace_create_with_starter(parent.path(), "demo", Starter::ObstacleAvoidance)
                    .unwrap_err()
            };
            assert!(matches!(error, CliError::WorkspaceExists { .. }));
            assert_eq!(
                std::fs::read_link(destination).unwrap(),
                Path::new("user-choice")
            );
            assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 1);
        }
    }
}
