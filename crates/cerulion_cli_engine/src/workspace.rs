// SPDX-License-Identifier: AGPL-3.0-only
//! Workspace discovery, creation, and initialization.
//!
//! A Cerulion workspace is a directory containing:
//! - `Cargo.toml` with `[workspace]`
//! - `graphs/` directory for graph YAML files
//! - `nodes/` directory for node crate sources
//! - `schemas/` directory for schema YAML files

use std::io::Write;
use std::path::{Path, PathBuf};

use crate::error::{CliError, CliResult};

/// Discovered Cerulion workspace.
#[derive(Debug)]
pub struct CerulionWorkspace {
    pub root: PathBuf,
    pub graphs_dir: PathBuf,
    pub nodes_dir: PathBuf,
    pub schemas_dir: PathBuf,
    /// Where the scaffolded root `Cargo.toml` points its `cerulion_core` /
    /// `native_ros2_messages` dependencies. `Some` only for a workspace this
    /// call just created (`workspace create` / `workspace init`); `None` for a
    /// workspace found by [`CerulionWorkspace::discover`], whose manifest the
    /// user may have edited since.
    pub dependency_source: Option<DependencySource>,
}

/// The dependency source `workspace create` chose for the generated root
/// manifest. The decision keys on where the running `cerulion` BINARY lives
/// (or was built), never on the current directory — see the crate-private
/// `find_cerulion_base` — and the CLI prints it beside the created path so a
/// miss on a checkout-built binary is never silent.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DependencySource {
    /// Exact-pinned published crates (`cerulion_core = "=X.Y.Z"`).
    Registry {
        /// The pinned version — the installed CLI's own `CARGO_PKG_VERSION`.
        version: String,
    },
    /// Absolute `path` dependencies into a Cerulion source checkout.
    Checkout {
        /// The checkout root the paths point into.
        base: PathBuf,
    },
}

impl std::fmt::Display for DependencySource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Registry { version } => write!(
                f,
                "published crates from crates.io, pinned to ={version} (the installed CLI version)"
            ),
            Self::Checkout { base } => write!(
                f,
                "source checkout at {} (absolute path dependencies)",
                base.display()
            ),
        }
    }
}

impl CerulionWorkspace {
    /// Discover a workspace by walking upward from `start_dir`.
    ///
    /// Looks for a directory containing both `Cargo.toml` with `[workspace]`
    /// and a `graphs/` directory.
    pub fn discover(start_dir: &Path) -> CliResult<Self> {
        let mut current = start_dir.to_path_buf();
        loop {
            let cargo_toml = current.join("Cargo.toml");
            let graphs_dir = current.join("graphs");
            if cargo_toml.exists() && graphs_dir.is_dir() {
                let content = std::fs::read_to_string(&cargo_toml)?;
                if content.contains("[workspace]") {
                    return Ok(Self {
                        graphs_dir: current.join("graphs"),
                        nodes_dir: current.join("nodes"),
                        schemas_dir: current.join("schemas"),
                        root: current,
                        dependency_source: None,
                    });
                }
            }
            if !current.pop() {
                return Err(CliError::WorkspaceNotFound {
                    start: start_dir.display().to_string(),
                });
            }
        }
    }
}

/// Create a new workspace at `parent_dir/{name}/`.
pub fn workspace_create(parent_dir: &Path, name: &str) -> CliResult<CerulionWorkspace> {
    let root = parent_dir.join(name);
    if root.exists() {
        return Err(CliError::WorkspaceExists {
            path: root.display().to_string(),
        });
    }
    scaffold_workspace(&root, name)
}

/// Initialize a workspace at the given path (default: current directory).
pub fn workspace_init(location: &Path) -> CliResult<CerulionWorkspace> {
    let cargo_toml = location.join("Cargo.toml");
    if cargo_toml.exists() {
        let content = std::fs::read_to_string(&cargo_toml)?;
        if content.contains("[workspace]") {
            return Err(CliError::WorkspaceExists {
                path: location.display().to_string(),
            });
        }
    }
    let name = location
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("cerulion_ws");
    scaffold_workspace(location, name)
}

/// Locate the Cerulion source checkout root. Three tiers, in order:
///
/// 1. **Walk-up from `current_exe()`** ([`exe_walk_up_candidate`]): when
///    installed via `cargo install --path cerulion_cli`, the binary lives at
///    `<checkout>/target/{debug,release}/cerulion`; walking up finds
///    the repo root containing `crates/cerulion_core/Cargo.toml`.
/// 2. **Build-time repo path** ([`compile_time_base_candidate`]): an ambient
///    `CARGO_TARGET_DIR` moves the binary (or a test binary calling the
///    engine in-process, e.g. cli_e2e under `cargo test`) OUTSIDE the repo,
///    so the walk-up fails even though the repo is right where it was at
///    build time — and the generated workspace `Cargo.toml` would fall back
///    to broken relative paths, killing every `node_build` in it. The engine
///    crate's baked `CARGO_MANIFEST_DIR` parent is the repo root at BUILD
///    time; it is valid whenever the binary runs on the machine it was built
///    on (developer machines, CI, tests) and is guarded by a runtime existence check.
///    Same `CARGO_TARGET_DIR` hazard class as cdylib resolution.
/// 3. **`None`**: the caller (`scaffold_workspace`) generates exact-pinned
///    registry dependencies for the installed-user path.
pub(crate) fn find_cerulion_base() -> Option<PathBuf> {
    if let Some(base) = exe_walk_up_candidate() {
        return Some(base);
    }
    if let Some(base) = compile_time_base_candidate() {
        tracing::info!(
            base = %base.display(),
            "located the Cerulion source checkout via the build-time repo path (CARGO_MANIFEST_DIR) — \
             the current_exe walk-up failed (e.g. an ambient CARGO_TARGET_DIR moved \
             the binary outside the repo)"
        );
        return Some(base);
    }
    None
}

/// Tier 1: walk up from the CLI binary's location (see [`find_cerulion_base`]).
fn exe_walk_up_candidate() -> Option<PathBuf> {
    let exe = match std::env::current_exe() {
        Ok(p) => p,
        Err(e) => {
            tracing::debug!(error = %e, "could not determine CLI binary path");
            return None;
        }
    };
    // Resolve symlinks so walk-up traverses the real directory tree.
    let exe = exe.canonicalize().unwrap_or(exe);
    let mut dir = exe.parent()?;
    // Walk up to 5 levels: covers target/{debug,release}/cerulion (3 levels)
    // plus cross-compilation dirs like target/<triple>/release/ (5 levels).
    for _ in 0..5 {
        if dir.join("crates/cerulion_core/Cargo.toml").exists() {
            return Some(dir.to_path_buf());
        }
        dir = dir.parent()?;
    }
    tracing::debug!(exe = %exe.display(), "walked 5 levels from binary without finding cerulion_core");
    None
}

/// Tier 2: the repo root baked in at BUILD time — the engine crate's
/// `CARGO_MANIFEST_DIR` grandparent (`<repo>/crates/cerulion_cli_engine` → the
/// repo root). Returned only if `<candidate>/crates/cerulion_core/Cargo.toml` exists
/// at RUNTIME: registry-installed binaries carry a baked path into a cargo
/// registry cache with no `crates/cerulion_core` subtree, so for them this tier
/// safely falls through instead of generating a bogus path.
fn compile_time_base_candidate() -> Option<PathBuf> {
    base_candidate_if_repo(Path::new(env!("CARGO_MANIFEST_DIR")).parent()?.parent()?)
}

/// The tier-2 runtime existence guard: `candidate` counts as a repo root iff
/// `<candidate>/crates/cerulion_core/Cargo.toml` exists. Split out so the guard is
/// testable against arbitrary paths (including the negative, non-repo case).
fn base_candidate_if_repo(candidate: &Path) -> Option<PathBuf> {
    if candidate.join("crates/cerulion_core/Cargo.toml").exists() {
        Some(candidate.to_path_buf())
    } else {
        None
    }
}

/// Create the workspace directory structure and files.
fn scaffold_workspace(root: &Path, name: &str) -> CliResult<CerulionWorkspace> {
    let graphs_dir = root.join("graphs");
    let nodes_dir = root.join("nodes");
    let schemas_dir = root.join("schemas");

    std::fs::create_dir_all(&graphs_dir)?;
    std::fs::create_dir_all(&nodes_dir)?;
    std::fs::create_dir_all(&schemas_dir)?;
    let gitignore_path = root.join(".gitignore");
    let mut gitignore = match std::fs::read_to_string(&gitignore_path) {
        Ok(contents) => contents,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => String::new(),
        Err(error) => return Err(error.into()),
    };
    if !gitignore.lines().any(|line| line.trim() == ".cerulion/") {
        if !gitignore.is_empty() && !gitignore.ends_with('\n') {
            gitignore.push('\n');
        }
        gitignore.push_str(".cerulion/\n");
    }
    std::fs::write(gitignore_path, gitignore)?;

    // Workspace Cargo.toml — use absolute paths when a source checkout is found,
    // otherwise use exact-pinned registry dependencies.
    let base = find_cerulion_base();
    if base.is_none() {
        tracing::info!(
            version = env!("CARGO_PKG_VERSION"),
            "no Cerulion source checkout was found; workspace uses published crates"
        );
    }
    let dependency_source = match &base {
        Some(base) => DependencySource::Checkout { base: base.clone() },
        None => DependencySource::Registry {
            version: env!("CARGO_PKG_VERSION").to_string(),
        },
    };
    let cargo_toml = workspace_manifest(base.as_deref(), env!("CARGO_PKG_VERSION"));
    std::fs::write(root.join("Cargo.toml"), cargo_toml)?;
    write_workspace_toolchain(
        root,
        cerulion_core::RUSTC_RELEASE,
        cerulion_core::RUSTC_FINGERPRINT,
        installed_rustup_compiler,
    )?;

    // .cargo/config.toml
    let cargo_dir = root.join(".cargo");
    std::fs::create_dir_all(&cargo_dir)?;
    std::fs::write(
        cargo_dir.join("config.toml"),
        r#"[env]
IOX2_LOG_LEVEL = "error"
RUST_LOG = { value = "warn", force = false }
"#,
    )?;

    tracing::info!(workspace = %name, path = %root.display(), "workspace created");

    Ok(CerulionWorkspace {
        root: root.to_path_buf(),
        graphs_dir,
        nodes_dir,
        schemas_dir,
        dependency_source: Some(dependency_source),
    })
}

/// Select a locally verified stable compiler without changing rustup defaults or
/// replacing an explicit project choice. A release name alone cannot identify a
/// custom compiler, and nightly releases lack their toolchain's date.
fn write_workspace_toolchain(
    root: &Path,
    release: &str,
    fingerprint: &str,
    probe: impl FnOnce(&str) -> Result<String, String>,
) -> CliResult<()> {
    for name in ["rust-toolchain", "rust-toolchain.toml"] {
        match std::fs::symlink_metadata(root.join(name)) {
            Ok(_) => {
                tracing::warn!(
                    rustc = release,
                    file = name,
                    "existing workspace Rust toolchain retained; node builds must match the Cerulion compiler"
                );
                return Ok(());
            }
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => return Err(error.into()),
        }
    }
    let Some(manifest) = stable_toolchain_manifest(release) else {
        tracing::warn!(
            rustc = release,
            "compiler is not a stable release; select the same compiler explicitly when building nodes"
        );
        return Ok(());
    };
    let compiler = match probe(release) {
        Ok(compiler) => compiler,
        Err(error) => {
            tracing::warn!(
                rustc = release,
                error = %error,
                "could not verify the installed rustup compiler; select the Cerulion compiler explicitly when building nodes"
            );
            return Ok(());
        }
    };
    if !toolchain_fingerprint_matches(release, fingerprint, &compiler) {
        tracing::warn!(
            rustc = release,
            fingerprint,
            "installed rustup compiler does not match Cerulion; ambient compiler selection retained"
        );
        return Ok(());
    }
    let path = root.join("rust-toolchain.toml");
    if !publish_toolchain_manifest(&path, |file| file.write_all(manifest.as_bytes()))? {
        tracing::warn!(
            path = %path.display(),
            rustc = release,
            "workspace Rust toolchain appeared during creation; existing choice retained"
        );
    }
    Ok(())
}

/// Publish complete bytes without replacing a concurrently created choice.
/// A failed write leaves only the sibling staging file to remove, never a
/// partial toolchain file that rustup or a later invocation could select.
fn publish_toolchain_manifest(
    path: &Path,
    write: impl FnOnce(&mut std::fs::File) -> std::io::Result<()>,
) -> std::io::Result<bool> {
    let mut nonce = [0_u8; 16];
    getrandom::fill(&mut nonce).map_err(|error| std::io::Error::other(error.to_string()))?;
    let staging = path.with_file_name(format!(
        ".rust-toolchain-{:032x}.tmp",
        u128::from_le_bytes(nonce)
    ));
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&staging)?;
    let written = write(&mut file).and_then(|()| file.sync_all());
    drop(file);
    // A sibling is on the same filesystem. Unlike rename, hard_link atomically
    // refuses existing files, directories, and symlinks. Unsupported filesystems
    // return an error rather than falling back to an overwriting publication.
    let published = written.and_then(|()| match std::fs::hard_link(&staging, path) {
        Ok(()) => Ok(true),
        Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => Ok(false),
        Err(error) => Err(error),
    });
    if let Err(error) = std::fs::remove_file(&staging) {
        if published.is_ok() {
            return Err(error);
        }
        tracing::warn!(
            path = %staging.display(),
            error = %error,
            "failed to remove toolchain staging file after publication failed"
        );
    }
    published
}

fn installed_rustup_compiler(release: &str) -> Result<String, String> {
    // `rustup run` never installs a missing toolchain without `--install`.
    // Workspace creation must remain offline and preserve installed defaults.
    let output = std::process::Command::new("rustup")
        .args(["run", release, "rustc", "-vV"])
        .output()
        .map_err(|error| error.to_string())?;
    if !output.status.success() {
        return Err(String::from_utf8_lossy(&output.stderr).trim().to_owned());
    }
    String::from_utf8(output.stdout).map_err(|error| error.to_string())
}

fn toolchain_fingerprint_matches(release: &str, fingerprint: &str, compiler: &str) -> bool {
    let mut actual_release = None;
    let mut actual_commit = None;
    for line in compiler.lines() {
        if let Some(value) = line.strip_prefix("release: ") {
            if actual_release.replace(value).is_some() {
                return false;
            }
        } else if let Some(value) = line.strip_prefix("commit-hash: ") {
            if actual_commit.replace(value).is_some() {
                return false;
            }
        }
    }
    let Some(commit) = actual_commit else {
        return false;
    };
    actual_release == Some(release)
        && commit.len() == 40
        && commit.bytes().all(|byte| byte.is_ascii_hexdigit())
        && fingerprint == format!("{release} ({commit})")
}

fn stable_toolchain_manifest(release: &str) -> Option<String> {
    let parts: Vec<_> = release.split('.').collect();
    if parts.len() != 3
        || parts
            .iter()
            .any(|part| part.is_empty() || !part.bytes().all(|byte| byte.is_ascii_digit()))
    {
        return None;
    }
    Some(format!(
        "[toolchain]\nchannel = \"{release}\"\nprofile = \"minimal\"\n"
    ))
}

fn workspace_manifest(base: Option<&Path>, version: &str) -> String {
    if let Some(base) = base {
        let core_path = base
            .join("crates/cerulion_core")
            .display()
            .to_string()
            .replace('\\', "/");
        let msgs_path = base
            .join("crates/native_ros2_messages")
            .display()
            .to_string()
            .replace('\\', "/");
        format!(
            r#"[workspace]
members = ["nodes/*"]
resolver = "2"

[workspace.dependencies]
cerulion_core = {{ path = "{core_path}" }}
native_ros2_messages = {{ path = "{msgs_path}" }}
"#
        )
    } else {
        format!(
            r#"[workspace]
members = ["nodes/*"]
resolver = "2"

[workspace.dependencies]
cerulion_core = "={version}"
native_ros2_messages = "={version}"
"#
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OFFICIAL_FINGERPRINT: &str = "1.93.0 (1111111111111111111111111111111111111111)";
    const OFFICIAL_COMPILER: &str =
        "release: 1.93.0\ncommit-hash: 1111111111111111111111111111111111111111\n";

    fn write_verified_toolchain(root: &Path, release: &str) -> CliResult<()> {
        write_workspace_toolchain(root, release, OFFICIAL_FINGERPRINT, |_| {
            Ok(OFFICIAL_COMPILER.to_owned())
        })
    }

    #[test]
    fn custom_stable_compiler_does_not_select_the_official_compiler() {
        let root = tempfile::tempdir().unwrap();
        write_workspace_toolchain(
            root.path(),
            "1.93.0",
            "1.93.0 (2222222222222222222222222222222222222222)",
            |_| Ok(OFFICIAL_COMPILER.to_owned()),
        )
        .unwrap();
        assert!(!root.path().join("rust-toolchain.toml").exists());
    }

    #[test]
    fn unavailable_or_malformed_rustup_compilers_preserve_ambient_selection() {
        for compiler in [
            Err("rustup is unavailable".to_owned()),
            Ok(String::new()),
            Ok("release: 1.93.0\ncommit-hash: unknown\n".to_owned()),
            Ok("release: 1.93.0\ncommit-hash: 11111111\n".to_owned()),
            Ok(OFFICIAL_COMPILER.replace("1.93.0", "1.94.0")),
            Ok(format!("{OFFICIAL_COMPILER}release: 1.93.0\n")),
            Ok(format!(
                "{OFFICIAL_COMPILER}commit-hash: 1111111111111111111111111111111111111111\n"
            )),
        ] {
            let root = tempfile::tempdir().unwrap();
            write_workspace_toolchain(root.path(), "1.93.0", OFFICIAL_FINGERPRINT, |_| compiler)
                .unwrap();
            assert!(!root.path().join("rust-toolchain.toml").exists());
        }
    }

    #[test]
    fn hashless_host_compiler_is_not_pinned() {
        let root = tempfile::tempdir().unwrap();
        write_workspace_toolchain(root.path(), "1.93.0", "1.93.0 (unknown)", |_| {
            Ok(OFFICIAL_COMPILER.to_owned())
        })
        .unwrap();
        assert!(!root.path().join("rust-toolchain.toml").exists());
    }

    #[test]
    fn stable_workspace_toolchain_is_exact_and_deterministic() {
        let expected = "[toolchain]\nchannel = \"1.93.0\"\nprofile = \"minimal\"\n";
        for _ in 0..2 {
            let root = tempfile::tempdir().unwrap();
            write_verified_toolchain(root.path(), "1.93.0").unwrap();
            assert_eq!(
                std::fs::read_to_string(root.path().join("rust-toolchain.toml")).unwrap(),
                expected
            );
        }
    }

    #[test]
    fn explicit_workspace_toolchain_choices_are_preserved() {
        for name in ["rust-toolchain", "rust-toolchain.toml"] {
            let root = tempfile::tempdir().unwrap();
            let original = "[toolchain]\nchannel = \"1.97.1\"\nprofile = \"complete\"\n";
            std::fs::write(root.path().join(name), original).unwrap();
            write_workspace_toolchain(root.path(), "1.93.0", OFFICIAL_FINGERPRINT, |_| {
                panic!("existing choice must be preserved without probing rustup")
            })
            .unwrap();
            assert_eq!(
                std::fs::read_to_string(root.path().join(name)).unwrap(),
                original
            );
            if name == "rust-toolchain" {
                assert!(!root.path().join("rust-toolchain.toml").exists());
            }
        }
    }

    #[cfg(unix)]
    #[test]
    fn dangling_workspace_toolchain_symlink_is_not_replaced() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("rust-toolchain.toml");
        std::os::unix::fs::symlink("missing-toolchain", &path).unwrap();
        write_workspace_toolchain(root.path(), "1.93.0", OFFICIAL_FINGERPRINT, |_| {
            panic!("existing symlink must be preserved without probing rustup")
        })
        .unwrap();
        assert_eq!(
            std::fs::read_link(path).unwrap(),
            Path::new("missing-toolchain")
        );
    }

    #[test]
    fn ambiguous_or_malformed_compiler_releases_do_not_create_overrides() {
        for release in [
            "1.99.0-nightly",
            "1.99.0-beta.1",
            "stable",
            "",
            "1.93",
            "1..0",
            "1.93.0\n",
            "1.93.0\"",
            "1.93.0.1",
        ] {
            let root = tempfile::tempdir().unwrap();
            write_workspace_toolchain(root.path(), release, OFFICIAL_FINGERPRINT, |_| {
                panic!("invalid release must not probe rustup")
            })
            .unwrap();
            assert!(
                !root.path().join("rust-toolchain.toml").exists(),
                "{release:?}"
            );
        }
    }

    #[test]
    fn workspace_toolchain_write_errors_propagate() {
        let root = tempfile::tempdir().unwrap();
        let missing = root.path().join("missing");
        assert!(write_verified_toolchain(&missing, "1.93.0").is_err());
        let file = root.path().join("file");
        std::fs::write(&file, "not a directory").unwrap();
        assert!(write_verified_toolchain(&file, "1.93.0").is_err());
    }

    #[test]
    fn failed_toolchain_writes_leave_no_manifest_and_can_be_retried() {
        for prefix in ["", "[toolchain]\nchannel = \""] {
            let root = tempfile::tempdir().unwrap();
            let path = root.path().join("rust-toolchain.toml");
            let error = publish_toolchain_manifest(&path, |file| {
                file.write_all(prefix.as_bytes())?;
                assert!(!path.exists(), "incomplete toolchain must stay unpublished");
                Err(std::io::Error::new(
                    std::io::ErrorKind::WriteZero,
                    "injected toolchain write failure",
                ))
            })
            .unwrap_err();
            assert_eq!(error.kind(), std::io::ErrorKind::WriteZero);
            assert!(!path.exists());
            assert_eq!(std::fs::read_dir(root.path()).unwrap().count(), 0);

            // Retry the pin writer, not whole-workspace creation: that operation
            // separately refuses an already existing root or Cargo manifest.
            write_verified_toolchain(root.path(), "1.93.0").unwrap();
            assert_eq!(
                std::fs::read_to_string(path).unwrap(),
                "[toolchain]\nchannel = \"1.93.0\"\nprofile = \"minimal\"\n"
            );
            assert_eq!(std::fs::read_dir(root.path()).unwrap().count(), 1);
        }
    }

    #[test]
    fn concurrent_toolchain_choice_survives_successful_and_failed_staging() {
        for fail_write in [false, true] {
            let root = tempfile::tempdir().unwrap();
            let path = root.path().join("rust-toolchain.toml");
            let winner = "[toolchain]\nchannel = \"1.97.1\"\nprofile = \"complete\"\n";
            let result = publish_toolchain_manifest(&path, |file| {
                file.write_all(b"[toolchain]\nchannel = \"")?;
                assert!(!path.exists(), "incomplete toolchain must stay unpublished");
                std::fs::write(&path, winner)?;
                if fail_write {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::WriteZero,
                        "injected toolchain write failure",
                    ));
                }
                file.write_all(b"1.93.0\"\nprofile = \"minimal\"\n")
            });
            if fail_write {
                assert_eq!(result.unwrap_err().kind(), std::io::ErrorKind::WriteZero);
            } else {
                assert!(!result.unwrap(), "a concurrent choice must win publication");
            }
            assert_eq!(std::fs::read_to_string(path).unwrap(), winner);
            assert_eq!(std::fs::read_dir(root.path()).unwrap().count(), 1);
        }
    }

    #[cfg(unix)]
    #[test]
    fn concurrent_dangling_toolchain_symlink_is_not_replaced() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("rust-toolchain.toml");
        let published = publish_toolchain_manifest(&path, |file| {
            std::os::unix::fs::symlink("chosen-toolchain", &path)?;
            file.write_all(b"[toolchain]\nchannel = \"1.93.0\"\nprofile = \"minimal\"\n")
        })
        .unwrap();
        assert!(!published);
        assert_eq!(
            std::fs::read_link(path).unwrap(),
            Path::new("chosen-toolchain")
        );
        assert_eq!(std::fs::read_dir(root.path()).unwrap().count(), 1);
    }

    #[test]
    fn test_workspace_create_structure() {
        let tmp = tempfile::tempdir().unwrap();
        let ws = workspace_create(tmp.path(), "my_robot").unwrap();

        assert!(ws.root.join("Cargo.toml").exists());
        assert!(ws.graphs_dir.is_dir());
        assert!(ws.nodes_dir.is_dir());
        assert!(ws.schemas_dir.is_dir());
        assert!(ws.root.join(".cargo/config.toml").exists());
        assert_eq!(
            std::fs::read_to_string(ws.root.join(".gitignore")).unwrap(),
            ".cerulion/\n"
        );
    }

    #[test]
    fn test_workspace_cargo_toml_content() {
        let tmp = tempfile::tempdir().unwrap();
        let ws = workspace_create(tmp.path(), "test_ws").unwrap();

        let content = std::fs::read_to_string(ws.root.join("Cargo.toml")).unwrap();
        assert!(content.contains("[workspace]"));
        assert!(content.contains("resolver = \"2\""));
        assert!(content.contains("members = [\"nodes/*\"]"));
        assert!(content.contains("cerulion_core"));
    }

    #[test]
    fn test_workspace_init() {
        let tmp = tempfile::tempdir().unwrap();
        let ws = workspace_init(tmp.path()).unwrap();

        assert!(ws.root.join("Cargo.toml").exists());
        assert!(ws.graphs_dir.is_dir());
        assert!(ws.nodes_dir.is_dir());
        assert!(ws.schemas_dir.is_dir());
    }

    #[test]
    fn test_workspace_create_already_exists() {
        let tmp = tempfile::tempdir().unwrap();
        workspace_create(tmp.path(), "my_robot").unwrap();
        let result = workspace_create(tmp.path(), "my_robot");
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("already exists"));
    }

    #[test]
    fn test_workspace_init_already_exists() {
        let tmp = tempfile::tempdir().unwrap();
        workspace_init(tmp.path()).unwrap();
        let result = workspace_init(tmp.path());
        assert!(result.is_err());
    }

    #[test]
    fn test_workspace_discover() {
        let tmp = tempfile::tempdir().unwrap();
        let ws = workspace_create(tmp.path(), "my_robot").unwrap();

        // Create a subdirectory to search from
        let sub = ws.nodes_dir.join("camera");
        std::fs::create_dir_all(&sub).unwrap();

        let found = CerulionWorkspace::discover(&sub).unwrap();
        assert_eq!(found.root, ws.root);
    }

    #[test]
    fn test_find_cerulion_base_returns_some_in_repo() {
        // When running via `cargo test`, the binary is inside the repo's
        // target/ directory, so find_cerulion_base() should succeed.
        let base = find_cerulion_base();
        assert!(
            base.is_some(),
            "find_cerulion_base() should find the repo root when run from within the repo"
        );
        let base = base.unwrap();
        assert!(base.join("crates/cerulion_core/Cargo.toml").exists());
    }

    #[test]
    fn test_compile_time_base_candidate_returns_some_in_repo() {
        // In-repo test run: the engine crate's CARGO_MANIFEST_DIR parent IS
        // the repo root, so the runtime existence guard holds by construction.
        let base = compile_time_base_candidate();
        assert!(
            base.is_some(),
            "compile_time_base_candidate() should resolve when tests run in-repo"
        );
        let base = base.unwrap();
        assert!(base.join("crates/cerulion_core/Cargo.toml").exists());
    }

    #[test]
    fn test_base_candidate_guard_rejects_non_repo_dir() {
        // NEGATIVE guard pin: a directory with no cerulion_core/Cargo.toml
        // (the registry-installed-binary case) must fall through to None —
        // an inverted guard would fabricate a bogus repo root.
        let tmp = tempfile::tempdir().unwrap();
        assert!(
            base_candidate_if_repo(tmp.path()).is_none(),
            "tier-2 guard must reject a dir without cerulion_core/Cargo.toml"
        );
    }

    #[test]
    fn test_workspace_cargo_toml_uses_absolute_paths() {
        let tmp = tempfile::tempdir().unwrap();
        let ws = workspace_create(tmp.path(), "remote_ws").unwrap();

        let content = std::fs::read_to_string(ws.root.join("Cargo.toml")).unwrap();
        // When a source checkout is found, paths should be absolute (start with /)
        if find_cerulion_base().is_some() {
            assert!(
                content.contains("path = \"/"),
                "expected absolute paths in Cargo.toml, got:\n{content}"
            );
        }
    }

    #[test]
    fn test_workspace_manifest_uses_absolute_paths_for_checkout() {
        let manifest = workspace_manifest(Some(Path::new("/tmp/cerulion-src")), "0.0.1-alpha");

        assert!(manifest
            .contains(r#"cerulion_core = { path = "/tmp/cerulion-src/crates/cerulion_core" }"#));
        assert!(manifest.contains(
            r#"native_ros2_messages = { path = "/tmp/cerulion-src/crates/native_ros2_messages" }"#
        ));
    }

    #[test]
    fn test_workspace_manifest_uses_exact_registry_pins_without_paths() {
        let manifest = workspace_manifest(None, "0.0.1-alpha");

        assert!(manifest.contains(r#"cerulion_core = "=0.0.1-alpha""#));
        assert!(manifest.contains(r#"native_ros2_messages = "=0.0.1-alpha""#));
        assert!(!manifest.contains("path ="));
    }

    #[test]
    fn test_workspace_discover_not_found() {
        let tmp = tempfile::tempdir().unwrap();
        let result = CerulionWorkspace::discover(tmp.path());
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("not found"));
    }
}
