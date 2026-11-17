// SPDX-License-Identifier: AGPL-3.0-only
//! Workspace discovery, creation, and initialization.
//!
//! A Cerulion workspace is a directory containing:
//! - `Cargo.toml` with `[workspace]`
//! - `graphs/` directory for graph YAML files
//! - `nodes/` directory for node crate sources
//! - `schemas/` directory for schema YAML files

use std::path::{Path, PathBuf};

use crate::error::{CliError, CliResult};

const ICEORYX2_FORK_GIT: &str = "https://github.com/cerulion-inc/iceoryx2";
const ICEORYX2_FORK_REV: &str = "282f479b5c2ae607a461c0f4bc6c70f452d1c604";
/// Keep these names aligned with the root Cargo.toml pin; a test enforces it.
const ICEORYX2_FORK_CRATES: &[&str] = &[
    "iceoryx2",
    "iceoryx2-bb-concurrency",
    "iceoryx2-bb-container",
    "iceoryx2-bb-derive-macros",
    "iceoryx2-bb-elementary",
    "iceoryx2-bb-elementary-traits",
    "iceoryx2-bb-flatbuffers",
    "iceoryx2-bb-linux",
    "iceoryx2-bb-lock-free",
    "iceoryx2-bb-loggers",
    "iceoryx2-bb-memory",
    "iceoryx2-bb-posix",
    "iceoryx2-bb-print",
    "iceoryx2-bb-system-types",
    "iceoryx2-cal",
    "iceoryx2-log",
    "iceoryx2-log-types",
    "iceoryx2-pal-concurrency-sync",
    "iceoryx2-pal-configuration",
    "iceoryx2-pal-os-api",
    "iceoryx2-pal-posix",
    "iceoryx2-pal-print",
];

/// The build-environment variable that joins a workspace's `schemas/` to the
/// generated message crate: `native_ros2_messages`' build script reads the
/// workspace root from it and generates the workspace's `.msg` and YAML types
/// beside the built-ins. The workspace scaffold sets it in `.cargo/config.toml`
/// (`[env]`, relative to the workspace root), which cargo passes to every
/// build script, so `cargo build`, `cerulion node build` and an editor's
/// rust-analyzer see the same types.
pub const WORKSPACE_MSGS_ENV: &str = "CERULION_WORKSPACE_MSGS";

/// Whether the workspace at `root` generates its messages into the message
/// crate: its `.cargo/config.toml` parses and sets [`WORKSPACE_MSGS_ENV`] under
/// `[env]`. A missing or unparseable file is `false`.
pub fn workspace_generates_messages(root: &Path) -> bool {
    std::fs::read_to_string(root.join(".cargo").join("config.toml"))
        .ok()
        .and_then(|text| text.parse::<toml::Table>().ok())
        .and_then(|table| table.get("env").and_then(|env| env.as_table()).cloned())
        .is_some_and(|env| env.contains_key(WORKSPACE_MSGS_ENV))
}

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
/// (or was built), never on the current directory: the crate-private
/// `find_cerulion_root` finds the source checkout the binary came from, and the
/// CLI prints the chosen source beside the created path so a miss on a
/// checkout-built binary is never silent.
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
    let ws = create_new_workspace(&root)?;
    tracing::info!(workspace = %name, path = %ws.root.display(), "workspace created");
    Ok(ws)
}

fn create_new_workspace(root: &Path) -> CliResult<CerulionWorkspace> {
    // Reserve the final directory before writing anything into it. A starter
    // may have published it since workspace_create's advisory absence check.
    if let Some(parent) = root.parent() {
        std::fs::create_dir_all(parent)?;
    }
    match std::fs::create_dir(root) {
        Ok(()) => {}
        Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {
            return Err(CliError::WorkspaceExists {
                path: root.display().to_string(),
            });
        }
        Err(error) => return Err(error.into()),
    }
    scaffold_workspace(root)
}

/// Initialize a workspace at the given path (default: current directory).
pub fn workspace_init(location: &Path) -> CliResult<CerulionWorkspace> {
    let location_exists = location.exists();
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
    let ws = initialize_workspace(location, location_exists)?;
    tracing::info!(workspace = %name, path = %ws.root.display(), "workspace created");
    Ok(ws)
}

fn initialize_workspace(location: &Path, location_exists: bool) -> CliResult<CerulionWorkspace> {
    if location_exists {
        scaffold_workspace(location)
    } else {
        create_new_workspace(location)
    }
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
///    crate's baked `CARGO_MANIFEST_DIR` grandparent is the repo root at BUILD
///    time; it is valid whenever the binary runs on the machine it was built
///    on (developer machines, CI, tests) and is guarded by a runtime existence check.
///    Same `CARGO_TARGET_DIR` hazard class as cdylib resolution.
/// 3. **`None`**: the caller (`scaffold_workspace`) generates exact-pinned
///    registry dependencies for the installed-user path.
pub(crate) fn find_cerulion_root() -> Option<PathBuf> {
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

/// Tier 1: walk up from the CLI binary's location (see [`find_cerulion_root`]).
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
pub(crate) fn scaffold_workspace(root: &Path) -> CliResult<CerulionWorkspace> {
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
    let base = find_cerulion_root();
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

    // .cargo/config.toml
    let cargo_dir = root.join(".cargo");
    std::fs::create_dir_all(&cargo_dir)?;
    std::fs::write(
        cargo_dir.join("config.toml"),
        format!(
            r#"[env]
IOX2_LOG_LEVEL = "error"
RUST_LOG = {{ value = "warn", force = false }}
# The workspace's schemas/ (.msg packages and YAML) generate into the message
# crate beside the built-in ROS 2 types.
{WORKSPACE_MSGS_ENV} = {{ value = ".", relative = true }}
"#
        ),
    )?;

    Ok(CerulionWorkspace {
        root: root.to_path_buf(),
        graphs_dir,
        nodes_dir,
        schemas_dir,
        dependency_source: Some(dependency_source),
    })
}

fn workspace_manifest(base: Option<&Path>, version: &str) -> String {
    let manifest = if let Some(base) = base {
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
    };
    format!("{manifest}\n{}", iceoryx2_fork_patch())
}

fn iceoryx2_fork_patch() -> String {
    let entries = ICEORYX2_FORK_CRATES
        .iter()
        .map(|name| {
            format!("{name} = {{ git = \"{ICEORYX2_FORK_GIT}\", rev = \"{ICEORYX2_FORK_REV}\" }}")
        })
        .collect::<Vec<_>>()
        .join("\n");
    format!(
        "# Node libraries link the same iceoryx2 build as the cerulion CLI. Android\n\
         # shared memory needs it; keep every entry on one revision.\n\
         [patch.crates-io]\n\
         {entries}"
    )
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;

    #[test]
    fn test_workspace_create_structure() {
        let tmp = tempfile::tempdir().unwrap();
        let ws = workspace_create(tmp.path(), "my_robot").unwrap();

        assert!(ws.root.join("Cargo.toml").exists());
        assert!(ws.graphs_dir.is_dir());
        assert!(ws.nodes_dir.is_dir());
        assert!(ws.schemas_dir.is_dir());
        assert!(ws.root.join(".cargo/config.toml").exists());
        assert!(
            workspace_generates_messages(&ws.root),
            "a new workspace generates its schemas into the message crate"
        );
        assert_eq!(
            std::fs::read_to_string(ws.root.join(".gitignore")).unwrap(),
            ".cerulion/\n"
        );
    }

    #[test]
    fn workspace_messages_need_the_env_key_under_env() {
        let tmp = tempfile::tempdir().unwrap();
        let cargo = tmp.path().join(".cargo");
        assert!(!workspace_generates_messages(tmp.path()), "no config file");
        std::fs::create_dir_all(&cargo).unwrap();
        for (text, expected) in [
            ("[env]\nIOX2_LOG_LEVEL = \"error\"\n", false),
            ("[build]\nCERULION_WORKSPACE_MSGS = \".\"\n", false),
            ("# CERULION_WORKSPACE_MSGS = \".\"\n[env]\n", false),
            ("[env\nCERULION_WORKSPACE_MSGS = \".\"\n", false),
            (
                "[env]\nCERULION_WORKSPACE_MSGS = { value = \".\", relative = true }\n",
                true,
            ),
            ("[env]\nCERULION_WORKSPACE_MSGS = \"/abs/ws\"\n", true),
        ] {
            std::fs::write(cargo.join("config.toml"), text).unwrap();
            assert_eq!(
                workspace_generates_messages(tmp.path()),
                expected,
                "{text:?}"
            );
        }
    }

    #[test]
    fn the_message_crate_reads_the_variable_the_scaffold_sets() {
        // The two spellings live in two crates; this pins them together.
        let build_rs = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../native_ros2_messages/build.rs");
        let text = std::fs::read_to_string(&build_rs).unwrap();
        assert!(
            text.contains(&format!(
                "const WORKSPACE_ENV: &str = \"{WORKSPACE_MSGS_ENV}\";"
            )),
            "{} must read {WORKSPACE_MSGS_ENV}",
            build_rs.display()
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

    /// Any Rust 1.95 or newer builds loadable nodes, so a new workspace names
    /// no compiler: Cargo and rustup select one as they would anywhere else.
    #[test]
    fn workspace_create_and_init_write_no_rust_toolchain_file() {
        let tmp = tempfile::tempdir().unwrap();
        let created = workspace_create(tmp.path(), "created").unwrap();
        let initialized = workspace_init(&tmp.path().join("initialized")).unwrap();
        let in_place = tmp.path().join("in_place");
        std::fs::create_dir(&in_place).unwrap();
        let in_place = workspace_init(&in_place).unwrap();
        for ws in [created, initialized, in_place] {
            for name in ["rust-toolchain", "rust-toolchain.toml"] {
                assert!(
                    std::fs::symlink_metadata(ws.root.join(name)).is_err(),
                    "{} wrote {name}",
                    ws.root.display()
                );
            }
        }
    }

    /// `workspace init` in a directory that already chose a compiler leaves
    /// that choice byte for byte, in either file name.
    #[test]
    fn workspace_init_leaves_an_existing_rust_toolchain_file_untouched() {
        let original = "[toolchain]\nchannel = \"1.97.1\"\nprofile = \"complete\"\n";
        for name in ["rust-toolchain", "rust-toolchain.toml"] {
            let tmp = tempfile::tempdir().unwrap();
            std::fs::write(tmp.path().join(name), original).unwrap();
            workspace_init(tmp.path()).unwrap();
            assert_eq!(
                std::fs::read_to_string(tmp.path().join(name)).unwrap(),
                original
            );
            let other = if name == "rust-toolchain" {
                "rust-toolchain.toml"
            } else {
                "rust-toolchain"
            };
            assert!(std::fs::symlink_metadata(tmp.path().join(other)).is_err());
        }
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
    fn a_starter_published_after_bare_creation_precheck_is_preserved() {
        use crate::starter::{workspace_create_with_starter, Starter};
        let parent = tempfile::tempdir().unwrap();
        let destination = parent.path().join("shared");
        // Pause ordinary creation after its advisory absence check, then let
        // the starter publish the destination before ordinary scaffolding.
        assert!(!destination.exists());
        let starter =
            workspace_create_with_starter(parent.path(), "shared", Starter::ObstacleAvoidance)
                .unwrap();
        let manifest = starter.root.join("Cargo.toml");
        let mut original = std::fs::read_to_string(&manifest).unwrap();
        original.push_str("\n# learner's published workspace\n");
        std::fs::write(&manifest, &original).unwrap();
        let before = snapshot(&destination);
        let result = create_new_workspace(&destination);
        assert_eq!(std::fs::read_to_string(manifest).unwrap(), original);
        assert!(matches!(result, Err(CliError::WorkspaceExists { .. })));
        assert_eq!(snapshot(&destination), before);
    }

    fn snapshot(root: &Path) -> std::collections::BTreeMap<PathBuf, Vec<u8>> {
        let mut files = std::collections::BTreeMap::new();
        let mut directories = vec![root.to_path_buf()];
        while let Some(directory) = directories.pop() {
            for entry in std::fs::read_dir(directory).unwrap() {
                let path = entry.unwrap().path();
                if path.is_dir() {
                    directories.push(path);
                } else {
                    files.insert(
                        path.strip_prefix(root).unwrap().to_path_buf(),
                        std::fs::read(path).unwrap(),
                    );
                }
            }
        }
        files
    }

    #[test]
    fn a_starter_published_after_absent_init_precheck_is_preserved() {
        use crate::starter::{workspace_create_with_starter, Starter};
        let parent = tempfile::tempdir().unwrap();
        let destination = parent.path().join("shared");
        // Pause initialization after it observes an absent location, then let
        // starter publication win before the actual init continuation.
        let location_exists = destination.exists();
        assert!(!location_exists);
        let starter =
            workspace_create_with_starter(parent.path(), "shared", Starter::ObstacleAvoidance)
                .unwrap();
        let manifest = starter.root.join("Cargo.toml");
        let mut original = std::fs::read_to_string(&manifest).unwrap();
        original.push_str("\n# learner's published workspace\n");
        std::fs::write(&manifest, &original).unwrap();
        let before = snapshot(&destination);
        let result = initialize_workspace(&destination, location_exists);
        assert_eq!(std::fs::read_to_string(manifest).unwrap(), original);
        assert!(matches!(result, Err(CliError::WorkspaceExists { .. })));
        assert_eq!(snapshot(&destination), before);
    }

    #[test]
    fn initialization_of_an_absent_nested_folder_still_creates_a_workspace() {
        let parent = tempfile::tempdir().unwrap();
        let location = parent.path().join("missing/robot");
        let workspace = workspace_init(&location).unwrap();
        assert_eq!(workspace.root, location);
        assert!(workspace.root.join("Cargo.toml").is_file());
        assert!(workspace.graphs_dir.is_dir());
    }

    #[test]
    fn ordinary_creation_retains_nested_name_and_missing_parent_support() {
        let parent = tempfile::tempdir().unwrap();
        let missing = parent.path().join("missing");
        let workspace = workspace_create(&missing, "nested/robot").unwrap();
        assert_eq!(workspace.root, missing.join("nested/robot"));
        assert!(workspace.root.join("Cargo.toml").exists());
        assert!(workspace.graphs_dir.is_dir());
    }

    #[test]
    fn ordinary_creation_preserves_a_non_directory_parent_on_error() {
        let parent = tempfile::tempdir().unwrap();
        let existing_file = parent.path().join("file");
        std::fs::write(&existing_file, b"user source").unwrap();
        assert!(matches!(
            workspace_create(&existing_file, "robot"),
            Err(CliError::Io(_))
        ));
        assert_eq!(std::fs::read(existing_file).unwrap(), b"user source");
        assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 1);
    }

    #[test]
    #[cfg(unix)]
    fn ordinary_creation_preserves_a_dangling_destination_symlink() {
        let parent = tempfile::tempdir().unwrap();
        let target = parent.path().join("missing");
        let destination = parent.path().join("robot");
        std::os::unix::fs::symlink(&target, &destination).unwrap();
        assert!(matches!(
            workspace_create(parent.path(), "robot"),
            Err(CliError::WorkspaceExists { .. })
        ));
        assert_eq!(std::fs::read_link(destination).unwrap(), target);
        assert!(!target.exists());
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
    fn test_find_cerulion_root_returns_some_in_repo() {
        // When running via `cargo test`, the binary is inside the repo's
        // target/ directory, so find_cerulion_root() should succeed.
        let base = find_cerulion_root();
        assert!(
            base.is_some(),
            "find_cerulion_root() should find the repo root when run from within the repo"
        );
        let base = base.unwrap();
        assert!(base.join("crates/cerulion_core/Cargo.toml").exists());
    }

    #[test]
    fn test_compile_time_base_candidate_returns_some_in_repo() {
        // In-repo test run: the engine crate's CARGO_MANIFEST_DIR grandparent IS
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
        if find_cerulion_root().is_some() {
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
    fn generated_manifests_patch_the_whole_iceoryx2_family_at_the_root_pin() {
        fn iceoryx2_patch(manifest: &str) -> BTreeMap<String, (String, String)> {
            let document: toml::Value = toml::from_str(manifest).unwrap();
            document
                .get("patch")
                .and_then(|patch| patch.get("crates-io"))
                .and_then(toml::Value::as_table)
                .unwrap()
                .iter()
                .filter(|(name, _)| name.starts_with("iceoryx2"))
                .map(|(name, entry)| {
                    let entry = entry.as_table().unwrap();
                    (
                        name.clone(),
                        (
                            entry
                                .get("git")
                                .and_then(toml::Value::as_str)
                                .unwrap()
                                .to_owned(),
                            entry
                                .get("rev")
                                .and_then(toml::Value::as_str)
                                .unwrap()
                                .to_owned(),
                        ),
                    )
                })
                .collect()
        }

        let root_manifest =
            std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../Cargo.toml"))
                .unwrap();
        let root_patch = iceoryx2_patch(&root_manifest);
        assert!(!root_patch.is_empty());

        let checkout_manifest =
            workspace_manifest(Some(Path::new("/tmp/cerulion-src")), "0.0.1-alpha");
        let registry_manifest = workspace_manifest(None, "0.0.1-alpha");
        let example_manifest = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/basic_timer/Cargo.toml"
        ))
        .unwrap();

        // The Android sample's app runtime crate is its own workspace too, so
        // it carries the same block: a patch does not reach across workspaces.
        let android_runtime_manifest = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../cerulion_android/android/runtime/Cargo.toml"
        ))
        .unwrap();

        assert_eq!(iceoryx2_patch(&checkout_manifest), root_patch);
        assert_eq!(iceoryx2_patch(&registry_manifest), root_patch);
        assert_eq!(iceoryx2_patch(&example_manifest), root_patch);
        assert_eq!(iceoryx2_patch(&android_runtime_manifest), root_patch);
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
