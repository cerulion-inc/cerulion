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

const ICEORYX2_FORK_GIT: &str = "https://github.com/cerulion-inc/iceoryx2";
const ICEORYX2_FORK_REV: &str = "8657bc53bd0b544dd40fc9ce27e6cdb90c312b51";
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
/// crate: the value of [`WORKSPACE_MSGS_ENV`] the build script sees names
/// THIS workspace. That value is the inherited process variable unless the
/// workspace's `.cargo/config.toml` sets the key under `[env]` with
/// `force = true`; a non-forced entry applies only when nothing is inherited.
/// Without a usable entry (no file, no `[env]` key, an unparseable file) the
/// inherited value alone decides, as it does for the build script.
///
/// An empty value generates nothing. A value that names another workspace
/// generates THAT workspace's types, so it does not count. A relative value
/// resolves the way the build script reads it: from the config with
/// `relative = true`, cargo anchors it at the workspace root; anywhere else
/// (an inherited value, a config entry without `relative = true`) the build
/// script reads it from its own package directory, the `native_ros2_messages`
/// path dependency in the workspace manifest. A relative value with no such
/// path dependency (a registry or git source) does not count.
pub fn workspace_generates_messages(root: &Path) -> bool {
    workspace_generates_messages_with(root, std::env::var_os(WORKSPACE_MSGS_ENV).as_deref())
}

/// [`workspace_generates_messages`] with the inherited value of
/// [`WORKSPACE_MSGS_ENV`] passed in, so a test does not mutate the process
/// environment.
fn workspace_generates_messages_with(root: &Path, inherited: Option<&std::ffi::OsStr>) -> bool {
    let Some(path) = effective_workspace_msgs(root, inherited) else {
        return false;
    };
    match (path.canonicalize(), root.canonicalize()) {
        (Ok(named), Ok(this)) => named == this,
        _ => false,
    }
}

/// The workspace's `.cargo/config.toml` `[env]` entry for
/// [`WORKSPACE_MSGS_ENV`], in cargo's string or `{ value, relative, force }`
/// table form.
struct ConfiguredMsgs {
    value: String,
    /// Cargo joins the value to the directory that holds `.cargo/`.
    relative: bool,
    /// The entry replaces an inherited value instead of yielding to it.
    force: bool,
}

impl ConfiguredMsgs {
    /// The entry in the workspace at `root`, or `None` without a usable one
    /// (no file, an unparseable file, no `[env]` key, a value of another type).
    fn read(root: &Path) -> Option<Self> {
        let text = std::fs::read_to_string(root.join(".cargo").join("config.toml")).ok()?;
        let table = text.parse::<toml::Table>().ok()?;
        match table.get("env")?.as_table()?.get(WORKSPACE_MSGS_ENV)? {
            toml::Value::String(value) => Some(Self {
                value: value.clone(),
                relative: false,
                force: false,
            }),
            toml::Value::Table(entry) => {
                let flag = |key: &str| entry.get(key).and_then(|flag| flag.as_bool()) == Some(true);
                Some(Self {
                    value: entry
                        .get("value")
                        .and_then(|value| value.as_str())
                        .unwrap_or("")
                        .to_owned(),
                    relative: flag("relative"),
                    force: flag("force"),
                })
            }
            _ => None,
        }
    }

    /// The directory the build script reads for this entry in the workspace
    /// at `root`, or `None` for an empty value or a relative value it cannot
    /// resolve.
    fn path(&self, root: &Path) -> Option<PathBuf> {
        if self.value.is_empty() {
            None
        } else if self.relative {
            Some(root.join(&self.value))
        } else {
            resolve_from_message_crate(root, Path::new(&self.value))
        }
    }
}

/// The directory the build script reads from [`WORKSPACE_MSGS_ENV`] for a
/// build in the workspace at `root`, or `None` when the value it sees is
/// unset, empty, or relative to a package directory this workspace does not
/// name.
fn effective_workspace_msgs(root: &Path, inherited: Option<&std::ffi::OsStr>) -> Option<PathBuf> {
    match (ConfiguredMsgs::read(root), inherited) {
        (Some(entry), _) if entry.force => entry.path(root),
        (_, Some(value)) if value.is_empty() => None,
        (_, Some(value)) => resolve_from_message_crate(root, Path::new(value)),
        (Some(entry), None) => entry.path(root),
        (None, None) => None,
    }
}

/// `value` as the `native_ros2_messages` build script reads it: cargo runs a
/// build script in its package directory, so a relative value resolves from
/// there. An absolute value is returned as is. A relative value resolves
/// only when the workspace manifest at `root` gives the message crate a
/// `path` source (`[workspace.dependencies]` or `[dependencies]`); for a
/// registry or git source it is `None`.
fn resolve_from_message_crate(root: &Path, value: &Path) -> Option<PathBuf> {
    if value.is_absolute() {
        return Some(value.to_path_buf());
    }
    Some(message_crate_dir(root)?.join(value))
}

/// The package directory of the `native_ros2_messages` path dependency that
/// the workspace manifest at `root` declares, or `None` without one.
fn message_crate_dir(root: &Path) -> Option<PathBuf> {
    let text = std::fs::read_to_string(root.join("Cargo.toml")).ok()?;
    let manifest = text.parse::<toml::Table>().ok()?;
    let declared = |deps: Option<&toml::Value>| {
        deps?
            .as_table()?
            .get("native_ros2_messages")?
            .as_table()?
            .get("path")?
            .as_str()
            .map(str::to_owned)
    };
    let path = declared(
        manifest
            .get("workspace")
            .and_then(|workspace| workspace.get("dependencies")),
    )
    .or_else(|| declared(manifest.get("dependencies")))?;
    Some(root.join(path))
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
        let this = tmp.path().display().to_string();
        assert!(
            !workspace_generates_messages_with(tmp.path(), None),
            "no config file"
        );
        std::fs::create_dir_all(&cargo).unwrap();
        for (text, expected) in [
            ("[env]\nIOX2_LOG_LEVEL = \"error\"\n".to_owned(), false),
            (
                "[build]\nCERULION_WORKSPACE_MSGS = \".\"\n".to_owned(),
                false,
            ),
            (
                "# CERULION_WORKSPACE_MSGS = \".\"\n[env]\n".to_owned(),
                false,
            ),
            ("[env\nCERULION_WORKSPACE_MSGS = \".\"\n".to_owned(), false),
            (
                "[env]\nCERULION_WORKSPACE_MSGS = { value = \".\", relative = true }\n".to_owned(),
                true,
            ),
            (format!("[env]\nCERULION_WORKSPACE_MSGS = {this:?}\n"), true),
            // Another workspace's root generates that workspace's types.
            (
                "[env]\nCERULION_WORKSPACE_MSGS = \"/abs/ws\"\n".to_owned(),
                false,
            ),
            // Without `relative = true` cargo passes "." unchanged and the
            // build script reads it from its own package directory; this
            // workspace has no manifest that names one.
            ("[env]\nCERULION_WORKSPACE_MSGS = \".\"\n".to_owned(), false),
        ] {
            std::fs::write(cargo.join("config.toml"), &text).unwrap();
            assert_eq!(
                workspace_generates_messages_with(tmp.path(), None),
                expected,
                "{text:?}"
            );
        }
    }

    #[test]
    fn an_inherited_value_must_name_this_workspace() {
        // The build script reads the inherited variable whether or not the
        // workspace has a `.cargo/config.toml`, and generates the workspace
        // the value names.
        let tmp = tempfile::tempdir().unwrap();
        let other = tempfile::tempdir().unwrap();
        assert!(workspace_generates_messages_with(
            tmp.path(),
            Some(tmp.path().as_os_str())
        ));
        assert!(!workspace_generates_messages_with(
            tmp.path(),
            Some(other.path().as_os_str())
        ));
        assert!(!workspace_generates_messages_with(
            tmp.path(),
            Some(std::ffi::OsStr::new("/other/ws"))
        ));
        assert!(!workspace_generates_messages_with(
            tmp.path(),
            Some(std::ffi::OsStr::new("."))
        ));
        assert!(!workspace_generates_messages_with(
            tmp.path(),
            Some(std::ffi::OsStr::new(""))
        ));
        assert!(!workspace_generates_messages_with(tmp.path(), None));
    }

    #[test]
    fn workspace_messages_follow_the_value_the_build_script_sees() {
        // Cargo lets an inherited variable override a non-forced `[env]`
        // entry; the build script ignores an empty value.
        let tmp = tempfile::tempdir().unwrap();
        let other_ws = tempfile::tempdir().unwrap();
        let cargo = tmp.path().join(".cargo");
        std::fs::create_dir_all(&cargo).unwrap();
        let empty = Some(std::ffi::OsStr::new(""));
        let this = Some(tmp.path().as_os_str());
        let other = Some(other_ws.path().as_os_str());
        let relative = "[env]\nCERULION_WORKSPACE_MSGS = { value = \".\", relative = true }\n";
        let forced =
            "[env]\nCERULION_WORKSPACE_MSGS = { value = \".\", relative = true, force = true }\n";
        for (text, inherited, expected) in [
            (relative, None, true),
            (relative, empty, false),
            (relative, this, true),
            // The inherited value wins over a non-forced entry, and names
            // another workspace: cargo would generate THAT one's types.
            (relative, other, false),
            (forced, empty, true),
            (forced, other, true),
            ("[env]\nCERULION_WORKSPACE_MSGS = \"\"\n", None, false),
            ("[env]\nCERULION_WORKSPACE_MSGS = \"\"\n", this, true),
            ("[env]\nCERULION_WORKSPACE_MSGS = \"\"\n", other, false),
            (
                "[env]\nCERULION_WORKSPACE_MSGS = { value = \"\", force = true }\n",
                this,
                false,
            ),
            ("[env]\nCERULION_WORKSPACE_MSGS = 1\n", None, false),
            ("[env]\nCERULION_WORKSPACE_MSGS = 1\n", this, true),
            ("[env]\nIOX2_LOG_LEVEL = \"error\"\n", this, true),
            ("[env]\nIOX2_LOG_LEVEL = \"error\"\n", other, false),
            ("[env]\nIOX2_LOG_LEVEL = \"error\"\n", empty, false),
            ("[build]\njobs = 4\n", this, true),
            ("[build]\njobs = 4\n", other, false),
        ] {
            std::fs::write(cargo.join("config.toml"), text).unwrap();
            assert_eq!(
                workspace_generates_messages_with(tmp.path(), inherited),
                expected,
                "{text:?} with inherited {inherited:?}"
            );
        }
    }

    #[test]
    fn a_relative_value_resolves_from_the_message_crate_directory() {
        // Cargo runs the build script in the message crate's package
        // directory, so an inherited value or a config entry without
        // `relative = true` resolves from there.
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        std::fs::create_dir_all(root.join("vendor/msgs")).unwrap();
        std::fs::create_dir_all(root.join(".cargo")).unwrap();
        let back = Some(std::ffi::OsStr::new("../.."));
        let here = Some(std::ffi::OsStr::new("."));
        let path_dep =
            "[workspace]\n[workspace.dependencies]\nnative_ros2_messages = { path = \"vendor/msgs\" }\n";
        let direct_dep = "[package]\nname = \"ws\"\n[dependencies]\nnative_ros2_messages = { path = \"vendor/msgs\" }\n";
        let registry = "[workspace]\n[workspace.dependencies]\nnative_ros2_messages = \"=0.0.1\"\n";
        let unanchored = "[env]\nCERULION_WORKSPACE_MSGS = \"../..\"\n";
        let forced_unanchored =
            "[env]\nCERULION_WORKSPACE_MSGS = { value = \"../..\", force = true }\n";
        for (manifest, config, inherited, expected) in [
            (path_dep, "", back, true),
            (direct_dep, "", back, true),
            // "." from the message crate is the message crate, not the root.
            (path_dep, "", here, false),
            (registry, "", back, false),
            (path_dep, unanchored, None, true),
            (registry, unanchored, None, false),
            (path_dep, forced_unanchored, here, true),
            (path_dep, unanchored, here, false),
        ] {
            std::fs::write(root.join("Cargo.toml"), manifest).unwrap();
            std::fs::write(root.join(".cargo/config.toml"), config).unwrap();
            assert_eq!(
                workspace_generates_messages_with(root, inherited),
                expected,
                "{manifest:?} {config:?} with inherited {inherited:?}"
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

        assert_eq!(iceoryx2_patch(&checkout_manifest), root_patch);
        assert_eq!(iceoryx2_patch(&registry_manifest), root_patch);
        assert_eq!(iceoryx2_patch(&example_manifest), root_patch);
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
