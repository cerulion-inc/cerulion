// SPDX-License-Identifier: AGPL-3.0-only
//! Rust for the verbs that build nodes: whether a Rust 1.95 or newer is there,
//! and, on a terminal, the offer to install it.
//!
//! `cerulion node build` compiles a node with the user's own `cargo`, and any
//! Rust at or above the workspace MSRV builds a node the CLI loads. So the
//! question is only whether one is there. [`rust_readiness`] answers it from
//! what the probe found; it is pure, and pinned by hand-written vectors, as is
//! the release parsing behind it ([`RustVersion::from_rustc_verbose`]).
//!
//! The check never fails a verb by itself. A declined offer, or a run off a
//! terminal, prints the command that installs Rust and lets the verb reach its
//! own error (for `node build`, the error and exit code a build without the
//! check gives). It never asks off a terminal.

use std::ffi::{OsStr, OsString};
use std::io::{BufRead, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use crate::viz_client::is_executable_file;

/// The oldest Rust that builds nodes: the workspace MSRV, `rust-version` in the
/// root `Cargo.toml`. A unit test pins it to `CARGO_PKG_RUST_VERSION`, and
/// `tools/scripts/test_install_rust.sh` pins the installer's copy.
pub const RUST_FLOOR: RustVersion = RustVersion::new(1, 95, 0);

/// The question the offer asks on stderr, before it reads one line of stdin.
pub const INSTALL_PROMPT: &str = concat!(
    "Rust ",
    env!("CARGO_PKG_RUST_VERSION"),
    " or newer is needed to build nodes. Install it now? [Y/n] "
);

/// The first line printed off a terminal, before the install command.
pub const RUST_NEEDED: &str = concat!(
    "Rust ",
    env!("CARGO_PKG_RUST_VERSION"),
    " or newer is needed to build nodes."
);

/// Printed with every rustup route: the step that selects stable.
pub const SELECT_STABLE: &str = "Then select it with `rustup default stable`, or for one build \
     with `RUSTUP_TOOLCHAIN=stable cerulion node build <node>`.";

/// After an install that still left no usable Rust.
pub const STILL_NOT_READY: &str = concat!(
    "Rust ",
    env!("CARGO_PKG_RUST_VERSION"),
    " or newer is still not ready to build nodes."
);

/// The name of the helper the Debian package and the Homebrew formula install.
pub const INSTALL_HELPER: &str = "cerulion-install-rust";

/// The official rustup installer, for a user to run by hand.
pub const RUSTUP_INIT_COMMAND: &str = "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
     | sh -s -- -y --profile minimal --default-toolchain stable";

/// How the CLI downloads the official rustup installer: the same arguments as
/// `tools/scripts/install_rust.sh`, followed by the output path.
const RUSTUP_INIT_DOWNLOAD: [&str; 14] = [
    "--proto",
    "=https",
    "--tlsv1.2",
    "-fsSL",
    "--retry",
    "5",
    "--retry-delay",
    "5",
    "--connect-timeout",
    "30",
    "--max-time",
    "120",
    "https://sh.rustup.rs",
    "-o",
];

/// How the CLI runs the downloaded installer: the same arguments as
/// `tools/scripts/install_rust.sh`. Shell startup files are left alone.
const RUSTUP_INIT_ARGS: [&str; 6] = [
    "-y",
    "--profile",
    "minimal",
    "--default-toolchain",
    "stable",
    "--no-modify-path",
];

/// How the CLI adds stable beside the toolchains an existing rustup manages.
const RUSTUP_ADD_STABLE: [&str; 6] = [
    "toolchain",
    "install",
    "stable",
    "--profile",
    "minimal",
    "--no-self-update",
];

/// A `rustc` release, as `MAJOR.MINOR.PATCH`. Ordering is numeric, field by
/// field (`1.100.0` is newer than `1.95.0`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RustVersion {
    major: u32,
    minor: u32,
    patch: u32,
}

impl RustVersion {
    pub const fn new(major: u32, minor: u32, patch: u32) -> Self {
        Self {
            major,
            minor,
            patch,
        }
    }

    /// The release in the output of `rustc -vV`: its one `release: ` line,
    /// read as `tools/scripts/install_rust.sh` reads it. `None` when there is
    /// no such line, more than one, or one that [`RustVersion::parse_release`]
    /// refuses.
    pub fn from_rustc_verbose(output: &str) -> Option<Self> {
        let mut releases = output
            .lines()
            .filter_map(|line| line.strip_prefix("release: "));
        let release = releases.next()?;
        if releases.next().is_some() {
            return None;
        }
        Self::parse_release(release)
    }

    /// One release value, `MAJOR.MINOR.PATCH` with each part decimal digits.
    /// A pre-release or build suffix (`1.96.0-nightly`, `1.95.0-beta.2`,
    /// `1.95.0+distro`) is dropped, as Cargo drops it when it checks a
    /// package's `rust-version`, so a nightly of a release counts as that
    /// release.
    pub fn parse_release(release: &str) -> Option<Self> {
        let core = release
            .split_once(['-', '+'])
            .map_or(release, |(core, _)| core);
        let mut parts = core.split('.');
        let mut next = || -> Option<u32> {
            let part = parts.next()?;
            if part.is_empty() || !part.bytes().all(|byte| byte.is_ascii_digit()) {
                return None;
            }
            part.parse().ok()
        };
        let version = Self::new(next()?, next()?, next()?);
        if parts.next().is_some() {
            return None;
        }
        Some(version)
    }
}

impl std::fmt::Display for RustVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

/// What the probe found, for the `cargo` a node build runs.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RustFound {
    /// No `cargo` on `PATH`, and none in Cargo's home.
    Missing,
    /// `cargo` is only in this Cargo home `bin` directory, which is not on
    /// `PATH`.
    OffPath(PathBuf),
    /// `cargo` is on `PATH`, and the compiler Cargo uses reported this release.
    Release(RustVersion),
    /// As [`RustFound::Release`], selected by this toolchain file in the
    /// probed directory or above it, which rustup obeys over its default.
    Pinned(RustVersion, PathBuf),
    /// `cargo` is on `PATH` through rustup, which has no default toolchain, so
    /// the compiler runs nothing.
    NoDefault,
    /// `cargo` is on `PATH`, and the compiler reported no release this can
    /// read for another reason: a toolchain file naming a toolchain that is not
    /// installed, or a broken install. Cargo reports it better than this can.
    Unreadable,
}

/// What a verb does about Rust before it goes on.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RustAction {
    /// Go on: Rust is ready, or nothing here does better than Cargo's own error.
    Proceed,
    /// Ask on the terminal whether to install Rust now.
    Offer,
    /// Print the command that installs Rust; the verb then reaches its own error.
    PrintInstall,
    /// Print how to put this Cargo home `bin` directory on `PATH`. Rust is
    /// installed; the shell does not find it.
    PrintPath(PathBuf),
    /// Print how to upgrade this release; no rustup was found to add stable
    /// beside it.
    PrintUpgrade(RustVersion),
    /// Print that this toolchain file selects this older release. Installing
    /// stable would not change what the file selects.
    PrintPinned(RustVersion, PathBuf),
}

/// The decision, from what the probe found. `has_rustup` is whether a rustup
/// is on `PATH` or in Cargo's home; `interactive` is whether stdin and stderr
/// are both terminals, the only case that may ask.
pub fn rust_readiness(
    found: RustFound,
    floor: RustVersion,
    has_rustup: bool,
    interactive: bool,
) -> RustAction {
    match found {
        RustFound::Release(release) | RustFound::Pinned(release, _) if release >= floor => {
            RustAction::Proceed
        }
        RustFound::Pinned(release, file) => RustAction::PrintPinned(release, file),
        RustFound::Unreadable => RustAction::Proceed,
        RustFound::OffPath(dir) => RustAction::PrintPath(dir),
        RustFound::Release(release) if !has_rustup => RustAction::PrintUpgrade(release),
        RustFound::Missing | RustFound::NoDefault | RustFound::Release(_) => {
            if interactive {
                RustAction::Offer
            } else {
                RustAction::PrintInstall
            }
        }
    }
}

/// Whether the probe found a Rust at or above `floor`.
fn is_ready(found: &RustFound, floor: RustVersion) -> bool {
    matches!(found, RustFound::Release(release) | RustFound::Pinned(release, _) if *release >= floor)
}

/// Whether an answer to the `[Y/n]` prompt agrees to install. Enter alone
/// agrees, as the capital `Y` says; `None` (end of input) does not, so a
/// closed stdin never installs anything. Anything but yes or Enter declines.
pub fn answer_is_yes(answer: Option<&str>) -> bool {
    answer.map(str::trim).is_some_and(|answer| {
        answer.is_empty() || answer.eq_ignore_ascii_case("y") || answer.eq_ignore_ascii_case("yes")
    })
}

/// `text` as one POSIX shell word: unchanged when every byte is safe there,
/// else single-quoted.
pub fn shell_word(text: &str) -> String {
    let safe = !text.is_empty()
        && text
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"/._-+=:,@%".contains(&byte));
    if safe {
        text.to_owned()
    } else {
        format!("'{}'", text.replace('\'', r"'\''"))
    }
}

/// The line that puts `dir` first on `PATH`, for a shell startup file.
pub fn export_path_line(dir: &Path) -> String {
    let dir = dir.display().to_string();
    if shell_word(&dir) == dir {
        format!("export PATH=\"{dir}:$PATH\"")
    } else {
        format!("export PATH={}:\"$PATH\"", shell_word(&dir))
    }
}

/// An executable the check found, and whether `PATH` finds it by name.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Program {
    pub path: PathBuf,
    pub on_path: bool,
}

impl Program {
    /// How a user types it: the bare name when `PATH` finds it, else its path.
    fn spelled(&self, name: &str) -> String {
        if self.on_path {
            name.to_owned()
        } else {
            shell_word(&self.path.display().to_string())
        }
    }
}

/// How Rust gets installed after a yes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InstallRoute {
    /// `cerulion-install-rust`, which the Debian package and the Homebrew
    /// formula install: `tools/scripts/install_rust.sh`.
    Helper(Program),
    /// Stable added beside the toolchains this rustup already manages. Its
    /// default is set only when it has none.
    Rustup(Program),
    /// The official rustup installer from `https://sh.rustup.rs`.
    RustupInit,
}

/// The route: the helper when there is one, else the rustup already here,
/// else the official installer.
pub fn install_route(helper: Option<Program>, rustup: Option<Program>) -> InstallRoute {
    match (helper, rustup) {
        (Some(helper), _) => InstallRoute::Helper(helper),
        (None, Some(rustup)) => InstallRoute::Rustup(rustup),
        (None, None) => InstallRoute::RustupInit,
    }
}

impl InstallRoute {
    /// The command a user runs to take this route by hand.
    pub fn command(&self) -> String {
        match self {
            Self::Helper(helper) => helper.spelled(INSTALL_HELPER),
            Self::Rustup(rustup) => format!(
                "{} toolchain install stable --profile minimal",
                rustup.spelled("rustup")
            ),
            Self::RustupInit => RUSTUP_INIT_COMMAND.to_owned(),
        }
    }

    /// A short name for the route, for logs.
    fn name(&self) -> &'static str {
        match self {
            Self::Helper(_) => "helper",
            Self::Rustup(_) => "rustup",
            Self::RustupInit => "rustup-init",
        }
    }
}

/// The machine a check runs against: the probe and the installer. Production
/// fills it from this process; the tests hand-write it.
struct Machine<'a> {
    /// Probe the `cargo` a build runs; `Some(dir)` puts `dir` at the front of
    /// `PATH` for the search and for the `rustc` it runs.
    probe: &'a mut dyn FnMut(Option<&Path>) -> RustFound,
    /// Run the route after a yes; `Ok(false)` is an installer that exited
    /// with a failure, which printed its own reason.
    install: &'a mut dyn FnMut(&InstallRoute) -> std::io::Result<bool>,
    route: InstallRoute,
    has_rustup: bool,
    /// Cargo's home `bin` directory, where the helper and the official
    /// installer place Cargo.
    cargo_dir: Option<PathBuf>,
    /// Whether `cargo_dir` is already an entry of `PATH`.
    cargo_dir_on_path: bool,
}

/// Check for Rust before a verb that builds nodes, or that sets up a workspace
/// for them, and offer to install it when stdin and stderr are both
/// terminals. Everything it prints goes to stderr, so stdout keeps carrying the
/// verb's result alone.
///
/// `dir` is where the compiler is probed, so a workspace's toolchain file
/// applies. Returns the Cargo home `bin` directory a build that follows must
/// run `cargo` from (see
/// [`node_build_with_cargo_dir`](crate::node_cmd::node_build_with_cargo_dir)),
/// when this call installed Rust there and `PATH` does not find it yet, and
/// `None` otherwise. It never fails the verb: a terminal it cannot read or
/// write is logged and treated as a no.
pub fn check_rust(dir: &Path) -> Option<PathBuf> {
    use std::io::IsTerminal as _;
    // A device cross-builds its nodes on a host; `node build` says so itself.
    if cfg!(target_os = "android") {
        return None;
    }
    // Without PATH the system's default search list applies, which this
    // cannot see; Cargo's own spawn reports what is missing.
    let Some(path) = std::env::var_os("PATH") else {
        tracing::debug!("PATH is unset; Rust readiness not checked");
        return None;
    };
    let interactive = std::io::stdin().is_terminal() && std::io::stderr().is_terminal();
    let cargo_dir = cargo_home_bin();
    let cargo_dir_on_path = cargo_dir
        .as_deref()
        .is_some_and(|cargo_dir| std::env::split_paths(&path).any(|entry| entry == cargo_dir));
    let rustup = find_program("rustup", &path, cargo_dir.as_deref());
    let helper = find_helper(&path);
    let has_rustup = rustup.is_some();
    let probe_dir = cargo_dir.clone();
    let rustup_path = rustup.as_ref().map(|rustup| rustup.path.clone());
    let mut probe = |front: Option<&Path>| {
        probe_cargo(
            dir,
            &path,
            front,
            probe_dir.as_deref(),
            rustup_path.as_deref(),
        )
    };
    let mut install = |route: &InstallRoute| run_install(route);
    let checked = check_rust_with(
        &mut Machine {
            probe: &mut probe,
            install: &mut install,
            route: install_route(helper, rustup),
            has_rustup,
            cargo_dir,
            cargo_dir_on_path,
        },
        interactive,
        &mut std::io::stdin().lock(),
        &mut std::io::stderr(),
    );
    checked.unwrap_or_else(|error| {
        tracing::warn!(error = %error, "could not use the terminal for the Rust check");
        None
    })
}

fn check_rust_with(
    machine: &mut Machine<'_>,
    interactive: bool,
    input: &mut dyn BufRead,
    out: &mut dyn Write,
) -> std::io::Result<Option<PathBuf>> {
    let found = (machine.probe)(None);
    tracing::debug!(found = ?found, "Rust readiness probed");
    match rust_readiness(found, RUST_FLOOR, machine.has_rustup, interactive) {
        RustAction::Proceed => return Ok(None),
        RustAction::PrintPath(dir) => {
            let shown = dir.display();
            writeln!(
                out,
                "{RUST_NEEDED} Cargo is installed in {shown}, which is not on your PATH."
            )?;
            writeln!(
                out,
                "Add it to your PATH, for example in your shell startup file: {}",
                export_path_line(&dir)
            )?;
            return Ok(None);
        }
        RustAction::PrintUpgrade(release) => {
            writeln!(out, "{RUST_NEEDED} The rustc Cargo uses reports {release}.")?;
            writeln!(
                out,
                "rustup does not manage that Rust: upgrade it with the tool that installed it, \
                 or install rustup from https://rustup.rs."
            )?;
            return Ok(None);
        }
        RustAction::PrintPinned(release, file) => {
            writeln!(
                out,
                "{RUST_NEEDED} The toolchain file {} selects Rust {release}; make it name \
                 `stable`, or remove it.",
                file.display()
            )?;
            return Ok(None);
        }
        RustAction::PrintInstall => {
            writeln!(out, "{RUST_NEEDED}")?;
            write_install_command(out, &machine.route)?;
            return Ok(None);
        }
        RustAction::Offer => {}
    }

    write!(out, "{INSTALL_PROMPT}")?;
    out.flush()?;
    let mut line = String::new();
    let answer = match input.read_line(&mut line)? {
        0 => None,
        _ => Some(line.as_str()),
    };
    if !answer_is_yes(answer) {
        tracing::info!(route = machine.route.name(), "Rust install declined");
        write_install_command(out, &machine.route)?;
        return Ok(None);
    }

    tracing::info!(
        route = machine.route.name(),
        "installing Rust after consent"
    );
    match (machine.install)(&machine.route) {
        Ok(true) => {}
        Ok(false) => {
            tracing::warn!(route = machine.route.name(), "Rust installer failed");
            writeln!(
                out,
                "Rust was not installed. To try again, run: {}",
                machine.route.command()
            )?;
            return Ok(None);
        }
        Err(error) => {
            tracing::warn!(route = machine.route.name(), error = %error, "Rust installer did not start");
            writeln!(
                out,
                "Rust was not installed: {error}. To try again, run: {}",
                machine.route.command()
            )?;
            return Ok(None);
        }
    }

    // The helper and the official installer place Cargo in Cargo's home,
    // which a fresh machine does not have on PATH yet. Check PATH first, then
    // with Cargo's home in front.
    if is_ready(&(machine.probe)(None), RUST_FLOOR) {
        return Ok(None);
    }
    if let Some(dir) = machine.cargo_dir.clone() {
        if is_ready(&(machine.probe)(Some(&dir)), RUST_FLOOR) {
            let place = if machine.cargo_dir_on_path {
                "which comes after another Rust on your PATH. Put it first"
            } else {
                "which is not on your PATH yet. Add it to your PATH"
            };
            writeln!(
                out,
                "Cargo is installed in {}, {place}: {}",
                dir.display(),
                export_path_line(&dir)
            )?;
            return Ok(Some(dir));
        }
    }
    writeln!(out, "{STILL_NOT_READY}")?;
    if matches!(machine.route, InstallRoute::Rustup(_)) {
        writeln!(out, "{SELECT_STABLE}")?;
    }
    Ok(None)
}

fn write_install_command(out: &mut dyn Write, route: &InstallRoute) -> std::io::Result<()> {
    writeln!(out, "To install it, run: {}", route.command())?;
    if matches!(route, InstallRoute::Rustup(_)) {
        writeln!(out, "{SELECT_STABLE}")?;
    }
    Ok(())
}

/// `PATH` with `front` first and empty entries dropped, for a build that runs
/// the `cargo` installed there. `None` when `front` cannot be placed on a
/// `PATH` (it holds the separator).
pub fn path_with_front(front: &Path, path: &OsStr) -> Option<OsString> {
    let rest = std::env::split_paths(path).filter(|dir| !dir.as_os_str().is_empty());
    std::env::join_paths(std::iter::once(front.to_path_buf()).chain(rest)).ok()
}

/// Cargo's home `bin` directory: `$CARGO_HOME/bin`, else `$HOME/.cargo/bin`.
fn cargo_home_bin() -> Option<PathBuf> {
    let non_empty = |name: &str| std::env::var_os(name).filter(|value| !value.is_empty());
    let home = non_empty("CARGO_HOME")
        .map(PathBuf::from)
        .or_else(|| non_empty("HOME").map(|home| PathBuf::from(home).join(".cargo")))?;
    Some(home.join("bin"))
}

fn find_on(path: &OsStr, name: &str) -> Option<PathBuf> {
    std::env::split_paths(path)
        .filter(|dir| !dir.as_os_str().is_empty())
        .map(|dir| dir.join(name))
        .find(|candidate| is_executable_file(candidate))
}

/// `name` on `PATH`, else in `fallback_dir`.
fn find_program(name: &str, path: &OsStr, fallback_dir: Option<&Path>) -> Option<Program> {
    if let Some(found) = find_on(path, name) {
        return Some(Program {
            path: found,
            on_path: true,
        });
    }
    let candidate = fallback_dir?.join(name);
    is_executable_file(&candidate).then_some(Program {
        path: candidate,
        on_path: false,
    })
}

/// `cerulion-install-rust` on `PATH`, else beside this executable (as it is
/// installed, then with symlinks resolved).
fn find_helper(path: &OsStr) -> Option<Program> {
    if let Some(found) = find_on(path, INSTALL_HELPER) {
        return Some(Program {
            path: found,
            on_path: true,
        });
    }
    let exe = std::env::current_exe().ok()?;
    let resolved = std::fs::canonicalize(&exe).ok();
    [Some(exe), resolved]
        .into_iter()
        .flatten()
        .filter_map(|exe| exe.parent().map(|dir| dir.join(INSTALL_HELPER)))
        .find(|candidate| is_executable_file(candidate))
        .map(|path| Program {
            path,
            on_path: false,
        })
}

/// The compiler Cargo runs, by Cargo's own precedence: `RUSTC`, then
/// `CARGO_BUILD_RUSTC`, else `rustc` on `PATH`. A `build.rustc` in a Cargo
/// config file is not read.
fn cargo_rustc() -> OsString {
    ["RUSTC", "CARGO_BUILD_RUSTC"]
        .into_iter()
        .find_map(|name| std::env::var_os(name).filter(|value| !value.is_empty()))
        .unwrap_or_else(|| OsString::from("rustc"))
}

/// Find the `cargo` a build runs and ask the compiler Cargo uses for its
/// release, run in `dir` so a toolchain file there applies. With `front`, the
/// search and the compiler see `front` first on `PATH`. A rustup proxy runs
/// with its automatic install of a missing toolchain turned off
/// (`RUSTUP_AUTO_INSTALL=0`).
fn probe_cargo(
    dir: &Path,
    path: &OsStr,
    front: Option<&Path>,
    cargo_dir: Option<&Path>,
    rustup: Option<&Path>,
) -> RustFound {
    let search = match front {
        Some(front) => match path_with_front(front, path) {
            Some(search) => search,
            None => return RustFound::Missing,
        },
        None => path.to_owned(),
    };
    if find_on(&search, "cargo").is_none() {
        return match cargo_dir {
            Some(cargo_dir) if front.is_none() && is_executable_file(&cargo_dir.join("cargo")) => {
                RustFound::OffPath(cargo_dir.to_path_buf())
            }
            _ => RustFound::Missing,
        };
    }
    let mut command = Command::new(cargo_rustc());
    command
        .arg("-vV")
        .current_dir(dir)
        .env("RUSTUP_AUTO_INSTALL", "0")
        .stdin(Stdio::null())
        .stderr(Stdio::null());
    if front.is_some() {
        command.env("PATH", &search);
    }
    let release = command
        .output()
        .ok()
        .filter(|output| output.status.success())
        .and_then(|output| String::from_utf8(output.stdout).ok())
        .and_then(|text| RustVersion::from_rustc_verbose(&text));
    match (release, rustup) {
        (Some(release), Some(_)) => {
            match selecting_toolchain_file(dir, std::env::var_os("RUSTUP_TOOLCHAIN").as_deref()) {
                Some(file) => RustFound::Pinned(release, file),
                None => RustFound::Release(release),
            }
        }
        (Some(release), None) => RustFound::Release(release),
        (None, Some(rustup)) if !rustup_has_default(rustup) => RustFound::NoDefault,
        (None, _) => RustFound::Unreadable,
    }
}

/// The toolchain file rustup obeys in `dir`: `rust-toolchain`, then
/// `rust-toolchain.toml`, in `dir` and then each directory above it. `None`
/// when `rustup_toolchain` (the `RUSTUP_TOOLCHAIN` value) is set, which rustup
/// obeys over any file.
fn selecting_toolchain_file(dir: &Path, rustup_toolchain: Option<&OsStr>) -> Option<PathBuf> {
    if rustup_toolchain.is_some_and(|value| !value.is_empty()) {
        return None;
    }
    dir.ancestors()
        .flat_map(|dir| [dir.join("rust-toolchain"), dir.join("rust-toolchain.toml")])
        .find(|file| file.is_file())
}

/// Whether this rustup has a default toolchain: `rustup default` fails when
/// none is set.
fn rustup_has_default(rustup: &Path) -> bool {
    Command::new(rustup)
        .arg("default")
        .env("RUSTUP_AUTO_INSTALL", "0")
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .is_ok_and(|status| status.success())
}

/// Run a route. The installers print their progress to stderr, so stdout
/// keeps carrying only the verb's result.
fn run_install(route: &InstallRoute) -> std::io::Result<bool> {
    match route {
        InstallRoute::Helper(helper) => run(Command::new(&helper.path)),
        InstallRoute::Rustup(rustup) => {
            let mut add = Command::new(&rustup.path);
            add.args(RUSTUP_ADD_STABLE);
            if !run(add)? {
                return Ok(false);
            }
            // Only a missing default becomes stable.
            if rustup_has_default(&rustup.path) {
                return Ok(true);
            }
            let mut select = Command::new(&rustup.path);
            select.args(["default", "stable"]);
            run(select)
        }
        InstallRoute::RustupInit => run_rustup_init(),
    }
}

fn run(mut command: Command) -> std::io::Result<bool> {
    Ok(command.stdout(std::io::stderr()).status()?.success())
}

/// Download the official installer into a private directory and run it.
fn run_rustup_init() -> std::io::Result<bool> {
    let mut nonce = [0_u8; 16];
    getrandom::fill(&mut nonce).map_err(|error| std::io::Error::other(error.to_string()))?;
    let dir = std::env::temp_dir().join(format!(
        "cerulion-rustup-init-{:032x}",
        u128::from_le_bytes(nonce)
    ));
    let mut builder = std::fs::DirBuilder::new();
    #[cfg(unix)]
    std::os::unix::fs::DirBuilderExt::mode(&mut builder, 0o700);
    builder.create(&dir)?;
    let result = download_and_run_rustup_init(&dir.join("rustup-init.sh"));
    if let Err(error) = std::fs::remove_dir_all(&dir) {
        tracing::warn!(
            path = %dir.display(),
            error = %error,
            "could not remove the downloaded rustup installer"
        );
    }
    result
}

fn download_and_run_rustup_init(script: &Path) -> std::io::Result<bool> {
    let mut download = Command::new("curl");
    download.args(RUSTUP_INIT_DOWNLOAD).arg(script);
    if !run(download)? {
        return Ok(false);
    }
    let mut init = Command::new("sh");
    init.arg(script).args(RUSTUP_INIT_ARGS);
    run(init)
}

#[cfg(test)]
mod tests {
    use super::*;

    const V1_80: RustVersion = RustVersion::new(1, 80, 0);
    const V1_94: RustVersion = RustVersion::new(1, 94, 9);
    const V1_98: RustVersion = RustVersion::new(1, 98, 1);
    const HOME_BIN: &str = "/home/u/.cargo/bin";

    /// The floor tracks the manifest, and the literal is re-confirmed here by
    /// hand, so an MSRV bump trips this test and the prompt is reread.
    #[test]
    fn floor_is_the_workspace_rust_version() {
        let manifest = format!("{}.0", env!("CARGO_PKG_RUST_VERSION"));
        assert_eq!(RustVersion::parse_release(&manifest), Some(RUST_FLOOR));
        assert_eq!(env!("CARGO_PKG_RUST_VERSION"), "1.95");
        assert_eq!(RUST_FLOOR, RustVersion::new(1, 95, 0));
        assert_eq!(RUST_FLOOR.to_string(), "1.95.0");
        assert_eq!(
            INSTALL_PROMPT,
            "Rust 1.95 or newer is needed to build nodes. Install it now? [Y/n] "
        );
        assert_eq!(RUST_NEEDED, "Rust 1.95 or newer is needed to build nodes.");
    }

    /// The `release:` value of `rustc -vV` across channels and builds.
    #[test]
    fn release_values_parse_as_cargo_reads_them() {
        let vectors: &[(&str, Option<RustVersion>)] = &[
            ("1.95.0", Some(RustVersion::new(1, 95, 0))),
            ("1.98.1", Some(RustVersion::new(1, 98, 1))),
            ("1.100.0", Some(RustVersion::new(1, 100, 0))),
            ("2.0.0", Some(RustVersion::new(2, 0, 0))),
            ("1.96.0-nightly", Some(RustVersion::new(1, 96, 0))),
            ("1.95.0-beta.2", Some(RustVersion::new(1, 95, 0))),
            ("1.95.0-dev", Some(RustVersion::new(1, 95, 0))),
            ("1.95.0+distro", Some(RustVersion::new(1, 95, 0))),
            ("1.95.0-beta+build.7", Some(RustVersion::new(1, 95, 0))),
            ("01.095.00", Some(RustVersion::new(1, 95, 0))),
            ("1.95", None),
            ("1", None),
            ("1.95.0.1", None),
            ("1..0", None),
            (".95.0", None),
            ("1.95.", None),
            ("", None),
            ("-nightly", None),
            ("x.y.z", None),
            ("1.95.x", None),
            ("1.95.0 ", None),
            (" 1.95.0", None),
            ("+1.95.0", None),
            ("1.-95.0", None),
            ("4294967296.0.0", None),
            ("1.95.0\u{663}", None),
        ];
        for &(release, expected) in vectors {
            assert_eq!(RustVersion::parse_release(release), expected, "{release:?}");
        }
    }

    #[test]
    fn rustc_verbose_output_yields_its_one_release_line() {
        let nightly = "rustc 1.96.0-nightly (0123456789 2026-10-01)\n\
                       binary: rustc\n\
                       commit-hash: 0123456789abcdef0123456789abcdef01234567\n\
                       commit-date: 2026-10-01\n\
                       host: aarch64-apple-darwin\n\
                       release: 1.96.0-nightly\n\
                       LLVM version: 21.1.2\n";
        assert_eq!(
            RustVersion::from_rustc_verbose(nightly),
            Some(RustVersion::new(1, 96, 0))
        );
        // Read as tools/scripts/install_rust.sh reads it: `release: ` and the
        // value, exactly once. A CRLF line end also reads, as `str::lines`
        // splits it.
        let vectors: &[(&str, Option<RustVersion>)] = &[
            ("release: 1.95.0\n", Some(RUST_FLOOR)),
            ("release: 1.98.1", Some(V1_98)),
            ("release: 1.98.1\r\n", Some(V1_98)),
            ("binary: rustc\nrelease: 1.80.0\n", Some(V1_80)),
            ("release:1.98.1\n", None),
            ("release:  1.98.1\n", None),
            ("release: 1.98.1 \n", None),
            ("", None),
            ("rustc 1.98.1 (48a229cea 2026-09-01)\n", None),
            ("release: \n", None),
            ("release: 1.98.1\nrelease: 1.98.1\n", None),
            ("release: 1.98.1\nrelease: 1.80.0\n", None),
            ("  release: 1.98.1\n", None),
            ("Release: 1.98.1\n", None),
            ("release: stable\n", None),
        ];
        for &(output, expected) in vectors {
            assert_eq!(
                RustVersion::from_rustc_verbose(output),
                expected,
                "{output:?}"
            );
        }
    }

    #[test]
    fn versions_order_numerically() {
        let mut versions = [
            RustVersion::new(1, 100, 0),
            RustVersion::new(1, 9, 0),
            RustVersion::new(2, 0, 0),
            RustVersion::new(1, 95, 1),
            RustVersion::new(1, 95, 0),
        ];
        versions.sort();
        assert_eq!(
            versions,
            [
                RustVersion::new(1, 9, 0),
                RustVersion::new(1, 95, 0),
                RustVersion::new(1, 95, 1),
                RustVersion::new(1, 100, 0),
                RustVersion::new(2, 0, 0),
            ]
        );
    }

    /// Every input combination, written out by hand.
    #[test]
    fn readiness_decides_every_case() {
        use RustAction::*;
        let off_path = || RustFound::OffPath(PathBuf::from(HOME_BIN));
        let print_path = || PrintPath(PathBuf::from(HOME_BIN));
        let vectors: Vec<(RustFound, bool, bool, RustAction)> = vec![
            // found, has_rustup, interactive, action
            (RustFound::Release(RUST_FLOOR), false, false, Proceed),
            (RustFound::Release(RUST_FLOOR), true, true, Proceed),
            (RustFound::Release(V1_98), false, true, Proceed),
            (RustFound::Release(V1_98), true, false, Proceed),
            (RustFound::Missing, false, false, PrintInstall),
            (RustFound::Missing, false, true, Offer),
            (RustFound::Missing, true, false, PrintInstall),
            (RustFound::Missing, true, true, Offer),
            (off_path(), false, false, print_path()),
            (off_path(), false, true, print_path()),
            (off_path(), true, false, print_path()),
            (off_path(), true, true, print_path()),
            (RustFound::Release(V1_94), true, false, PrintInstall),
            (RustFound::Release(V1_94), true, true, Offer),
            (RustFound::Release(V1_80), true, true, Offer),
            (RustFound::Release(V1_94), false, false, PrintUpgrade(V1_94)),
            (RustFound::Release(V1_80), false, true, PrintUpgrade(V1_80)),
            (RustFound::NoDefault, false, false, PrintInstall),
            (RustFound::NoDefault, false, true, Offer),
            (RustFound::NoDefault, true, false, PrintInstall),
            (RustFound::NoDefault, true, true, Offer),
            (
                RustFound::Pinned(RUST_FLOOR, PathBuf::from("/w/rust-toolchain.toml")),
                true,
                true,
                Proceed,
            ),
            (
                RustFound::Pinned(V1_80, PathBuf::from("/w/rust-toolchain.toml")),
                true,
                true,
                PrintPinned(V1_80, PathBuf::from("/w/rust-toolchain.toml")),
            ),
            (
                RustFound::Pinned(V1_94, PathBuf::from("/w/rust-toolchain")),
                true,
                false,
                PrintPinned(V1_94, PathBuf::from("/w/rust-toolchain")),
            ),
            (
                RustFound::Pinned(V1_94, PathBuf::from("/w/rust-toolchain")),
                false,
                false,
                PrintPinned(V1_94, PathBuf::from("/w/rust-toolchain")),
            ),
            (RustFound::Unreadable, false, false, Proceed),
            (RustFound::Unreadable, false, true, Proceed),
            (RustFound::Unreadable, true, false, Proceed),
            (RustFound::Unreadable, true, true, Proceed),
        ];
        for (found, has_rustup, interactive, expected) in vectors {
            let label = format!("{found:?} rustup={has_rustup} interactive={interactive}");
            assert_eq!(
                rust_readiness(found, RUST_FLOOR, has_rustup, interactive),
                expected,
                "{label}"
            );
        }
        // The floor is an input, not a constant baked into the decision.
        assert_eq!(
            rust_readiness(
                RustFound::Release(V1_94),
                RustVersion::new(1, 94, 0),
                false,
                false
            ),
            Proceed
        );
        assert_eq!(
            rust_readiness(
                RustFound::Release(V1_98),
                RustVersion::new(1, 99, 0),
                true,
                false
            ),
            PrintInstall
        );
    }

    #[test]
    fn only_yes_or_enter_agrees() {
        for answer in ["", "\n", "\r\n", "y\n", "Y\n", "yes\n", "YES", " Yes \n"] {
            assert!(answer_is_yes(Some(answer)), "{answer:?}");
        }
        for answer in ["n\n", "no\n", "N", "nope\n", "yep\n", "y y\n", "1\n", "q\n"] {
            assert!(!answer_is_yes(Some(answer)), "{answer:?}");
        }
        assert!(!answer_is_yes(None), "end of input never installs");
    }

    #[test]
    fn printed_paths_survive_a_shell() {
        let vectors = [
            ("/home/u/.cargo/bin", "/home/u/.cargo/bin"),
            ("/opt/a-b_c/1.2+x@y%z=w,v:u", "/opt/a-b_c/1.2+x@y%z=w,v:u"),
            ("/Users/Jo Doe/.cargo/bin", "'/Users/Jo Doe/.cargo/bin'"),
            ("/tmp/it's", r"'/tmp/it'\''s'"),
            ("/tmp/$HOME/`x`\"", "'/tmp/$HOME/`x`\"'"),
            ("", "''"),
        ];
        for (text, expected) in vectors {
            assert_eq!(shell_word(text), expected, "{text:?}");
        }
        assert_eq!(
            export_path_line(Path::new("/home/u/.cargo/bin")),
            "export PATH=\"/home/u/.cargo/bin:$PATH\""
        );
        assert_eq!(
            export_path_line(Path::new("/Users/Jo Doe/.cargo/bin")),
            "export PATH='/Users/Jo Doe/.cargo/bin':\"$PATH\""
        );
    }

    fn program(path: &str, on_path: bool) -> Program {
        Program {
            path: PathBuf::from(path),
            on_path,
        }
    }

    #[test]
    fn route_prefers_the_helper_then_rustup_then_the_official_installer() {
        let helper = program("/usr/bin/cerulion-install-rust", true);
        let rustup = program("/home/u/.cargo/bin/rustup", false);
        assert_eq!(
            install_route(Some(helper.clone()), Some(rustup.clone())),
            InstallRoute::Helper(helper.clone())
        );
        assert_eq!(
            install_route(Some(helper.clone()), None),
            InstallRoute::Helper(helper)
        );
        assert_eq!(
            install_route(None, Some(rustup.clone())),
            InstallRoute::Rustup(rustup)
        );
        assert_eq!(install_route(None, None), InstallRoute::RustupInit);
    }

    #[test]
    fn route_commands_are_what_a_user_types() {
        let vectors = [
            (
                InstallRoute::Helper(program("/usr/bin/cerulion-install-rust", true)),
                "cerulion-install-rust",
            ),
            (
                InstallRoute::Helper(program("/opt/cerulion/bin/cerulion-install-rust", false)),
                "/opt/cerulion/bin/cerulion-install-rust",
            ),
            (
                InstallRoute::Helper(program("/opt/my apps/cerulion-install-rust", false)),
                "'/opt/my apps/cerulion-install-rust'",
            ),
            (
                InstallRoute::Rustup(program("/usr/local/bin/rustup", true)),
                "rustup toolchain install stable --profile minimal",
            ),
            (
                InstallRoute::Rustup(program("/home/u/.cargo/bin/rustup", false)),
                "/home/u/.cargo/bin/rustup toolchain install stable --profile minimal",
            ),
            (
                InstallRoute::RustupInit,
                "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y \
                 --profile minimal --default-toolchain stable",
            ),
        ];
        for (route, expected) in vectors {
            assert_eq!(route.command(), expected, "{route:?}");
        }
    }

    /// The CLI's own run of the official installer matches the release
    /// installer's, argument for argument.
    #[test]
    fn rustup_init_arguments_match_the_release_installer() {
        assert_eq!(
            RUSTUP_INIT_DOWNLOAD.join(" "),
            "--proto =https --tlsv1.2 -fsSL --retry 5 --retry-delay 5 --connect-timeout 30 \
             --max-time 120 https://sh.rustup.rs -o"
        );
        assert_eq!(
            RUSTUP_INIT_ARGS.join(" "),
            "-y --profile minimal --default-toolchain stable --no-modify-path"
        );
        assert_eq!(
            RUSTUP_ADD_STABLE.join(" "),
            "toolchain install stable --profile minimal --no-self-update"
        );
        let script = std::fs::read_to_string(
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/scripts/install_rust.sh"),
        )
        .unwrap();
        assert!(script.contains(
            "curl --proto '=https' --tlsv1.2 -fsSL --retry 5 --retry-delay 5 \\\n        \
             --connect-timeout 30 --max-time 120 https://sh.rustup.rs -o"
        ));
        assert!(script.contains(
            "sh \"$temp/rustup-init.sh\" -y --profile minimal --default-toolchain stable \
             --no-modify-path"
        ));
        assert!(script
            .contains("\"$rustup\" toolchain install stable --profile minimal --no-self-update"));
        assert!(script.contains("\nrust_floor=1.95\n"));
    }

    #[test]
    fn path_with_front_puts_the_directory_first_and_drops_empty_entries() {
        let path = std::env::join_paths(["/usr/bin", "/bin"]).unwrap();
        let joined = path_with_front(Path::new(HOME_BIN), &path).unwrap();
        assert_eq!(
            std::env::split_paths(&joined).collect::<Vec<_>>(),
            [
                PathBuf::from(HOME_BIN),
                PathBuf::from("/usr/bin"),
                PathBuf::from("/bin"),
            ]
        );
        #[cfg(unix)]
        {
            for empty in ["", ":", "/usr/bin::/bin:"] {
                let joined = path_with_front(Path::new(HOME_BIN), OsStr::new(empty)).unwrap();
                assert!(
                    std::env::split_paths(&joined).all(|dir| !dir.as_os_str().is_empty()),
                    "{empty:?} left an empty entry: {joined:?}"
                );
            }
            assert_eq!(path_with_front(Path::new("/odd:dir"), &path), None);
        }
    }

    /// A hand-written machine: the probe answers from a script of results, the
    /// installer records every call.
    struct Scripted {
        probes: Vec<(Option<PathBuf>, RustFound)>,
        probe_calls: Vec<Option<PathBuf>>,
        installs: Vec<InstallRoute>,
        install_result: fn() -> std::io::Result<bool>,
        cargo_dir: Option<PathBuf>,
        cargo_dir_on_path: bool,
    }

    impl Scripted {
        fn new(probes: &[(Option<&str>, RustFound)]) -> Self {
            Self {
                probes: probes
                    .iter()
                    .rev()
                    .map(|(front, found)| (front.map(PathBuf::from), found.clone()))
                    .collect(),
                probe_calls: Vec::new(),
                installs: Vec::new(),
                install_result: || Ok(true),
                cargo_dir: Some(PathBuf::from(HOME_BIN)),
                cargo_dir_on_path: false,
            }
        }

        fn run(
            &mut self,
            route: InstallRoute,
            has_rustup: bool,
            interactive: bool,
            input: &mut dyn BufRead,
        ) -> (std::io::Result<Option<PathBuf>>, String) {
            let mut out = Vec::new();
            let probes = &mut self.probes;
            let probe_calls = &mut self.probe_calls;
            let installs = &mut self.installs;
            let install_result = self.install_result;
            let mut probe = |front: Option<&Path>| {
                probe_calls.push(front.map(Path::to_path_buf));
                let (expected_front, found) = probes.pop().expect("an unscripted probe ran");
                assert_eq!(front.map(Path::to_path_buf), expected_front);
                found
            };
            let mut install = |route: &InstallRoute| {
                installs.push(route.clone());
                install_result()
            };
            let result = check_rust_with(
                &mut Machine {
                    probe: &mut probe,
                    install: &mut install,
                    route,
                    has_rustup,
                    cargo_dir: self.cargo_dir.clone(),
                    cargo_dir_on_path: self.cargo_dir_on_path,
                },
                interactive,
                input,
                &mut out,
            );
            assert!(self.probes.is_empty(), "a scripted probe never ran");
            (result, String::from_utf8(out).unwrap())
        }

        fn answer(
            &mut self,
            route: InstallRoute,
            has_rustup: bool,
            interactive: bool,
            answer: &str,
        ) -> (std::io::Result<Option<PathBuf>>, String) {
            let mut input = std::io::Cursor::new(answer.as_bytes().to_vec());
            self.run(route, has_rustup, interactive, &mut input)
        }
    }

    fn helper_route() -> InstallRoute {
        InstallRoute::Helper(program("/usr/bin/cerulion-install-rust", true))
    }

    fn rustup_route() -> InstallRoute {
        InstallRoute::Rustup(program("/usr/local/bin/rustup", true))
    }

    #[test]
    fn ready_or_unreadable_rust_prints_nothing_and_installs_nothing() {
        for found in [RustFound::Release(V1_98), RustFound::Unreadable] {
            let mut machine = Scripted::new(&[(None, found.clone())]);
            let (result, out) = machine.answer(helper_route(), true, true, "y\n");
            assert_eq!(result.unwrap(), None, "{found:?}");
            assert_eq!(out, "", "{found:?}");
            assert!(machine.installs.is_empty(), "{found:?}");
        }
    }

    /// Off a terminal nothing is asked and nothing installed, whatever stdin
    /// holds.
    #[test]
    fn off_a_terminal_prints_the_command_and_never_asks() {
        let mut machine = Scripted::new(&[(None, RustFound::Missing)]);
        let (result, out) = machine.answer(helper_route(), false, false, "y\n");
        assert_eq!(result.unwrap(), None);
        assert_eq!(
            out,
            "Rust 1.95 or newer is needed to build nodes.\n\
             To install it, run: cerulion-install-rust\n"
        );
        assert!(machine.installs.is_empty());
    }

    #[test]
    fn off_a_terminal_without_a_helper_names_the_official_installer() {
        let mut machine = Scripted::new(&[(None, RustFound::Missing)]);
        let (_, out) = machine.answer(InstallRoute::RustupInit, false, false, "");
        assert_eq!(
            out,
            "Rust 1.95 or newer is needed to build nodes.\n\
             To install it, run: curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
             sh -s -- -y --profile minimal --default-toolchain stable\n"
        );
    }

    #[test]
    fn an_older_or_unselected_rust_under_rustup_off_a_terminal_names_both_steps() {
        for found in [RustFound::Release(V1_80), RustFound::NoDefault] {
            let mut machine = Scripted::new(&[(None, found.clone())]);
            let (_, out) = machine.answer(rustup_route(), true, false, "");
            assert_eq!(
                out,
                "Rust 1.95 or newer is needed to build nodes.\n\
                 To install it, run: rustup toolchain install stable --profile minimal\n\
                 Then select it with `rustup default stable`, or for one build with \
                 `RUSTUP_TOOLCHAIN=stable cerulion node build <node>`.\n",
                "{found:?}"
            );
            assert!(machine.installs.is_empty());
        }
    }

    #[test]
    fn an_older_rust_without_rustup_is_told_how_to_upgrade_and_not_offered() {
        let mut machine = Scripted::new(&[(None, RustFound::Release(V1_80))]);
        let (_, out) = machine.answer(InstallRoute::RustupInit, false, true, "y\n");
        assert_eq!(
            out,
            "Rust 1.95 or newer is needed to build nodes. The rustc Cargo uses reports 1.80.0.\n\
             rustup does not manage that Rust: upgrade it with the tool that installed it, or \
             install rustup from https://rustup.rs.\n"
        );
        assert!(machine.installs.is_empty());
    }

    #[test]
    fn cargo_off_path_is_told_to_fix_path_not_to_reinstall() {
        let mut machine = Scripted::new(&[(
            None,
            RustFound::OffPath(PathBuf::from("/Users/Jo Doe/.cargo/bin")),
        )]);
        let (result, out) = machine.answer(helper_route(), true, true, "y\n");
        assert_eq!(result.unwrap(), None);
        assert_eq!(
            out,
            "Rust 1.95 or newer is needed to build nodes. Cargo is installed in \
             /Users/Jo Doe/.cargo/bin, which is not on your PATH.\n\
             Add it to your PATH, for example in your shell startup file: \
             export PATH='/Users/Jo Doe/.cargo/bin':\"$PATH\"\n"
        );
        assert!(machine.installs.is_empty());
    }

    /// A toolchain file that selects an older Rust is named, and nothing is
    /// offered: stable beside it would not change what the file selects.
    #[test]
    fn a_toolchain_file_selecting_an_older_rust_is_named_not_offered() {
        let file = PathBuf::from("/w/demo/rust-toolchain.toml");
        let mut machine = Scripted::new(&[(None, RustFound::Pinned(V1_80, file))]);
        let (result, out) = machine.answer(rustup_route(), true, true, "y\n");
        assert_eq!(result.unwrap(), None);
        assert_eq!(
            out,
            "Rust 1.95 or newer is needed to build nodes. The toolchain file \
             /w/demo/rust-toolchain.toml selects Rust 1.80.0; make it name `stable`, or \
             remove it.\n"
        );
        assert!(machine.installs.is_empty());
    }

    /// After an install, a ready Rust selected by a toolchain file counts.
    #[test]
    fn a_yes_that_leaves_a_pinned_ready_rust_continues() {
        let file = PathBuf::from("/w/demo/rust-toolchain.toml");
        let mut machine = Scripted::new(&[
            (None, RustFound::NoDefault),
            (None, RustFound::Pinned(V1_98, file)),
        ]);
        let (result, out) = machine.answer(rustup_route(), true, true, "y\n");
        assert_eq!(result.unwrap(), None);
        assert_eq!(out, INSTALL_PROMPT);
    }

    /// Cargo's home already on PATH, behind another Rust: the line says to put
    /// it first, not that it is missing.
    #[test]
    fn a_yes_that_finds_cargo_home_behind_another_rust_says_put_it_first() {
        let mut machine = Scripted::new(&[
            (None, RustFound::Release(V1_80)),
            (None, RustFound::Release(V1_80)),
            (Some(HOME_BIN), RustFound::Release(V1_98)),
        ]);
        machine.cargo_dir_on_path = true;
        let (result, out) = machine.answer(rustup_route(), true, true, "y\n");
        assert_eq!(result.unwrap(), Some(PathBuf::from(HOME_BIN)));
        assert_eq!(
            out,
            format!(
                "{INSTALL_PROMPT}Cargo is installed in /home/u/.cargo/bin, which comes after \
                 another Rust on your PATH. Put it first: \
                 export PATH=\"/home/u/.cargo/bin:$PATH\"\n"
            )
        );
    }

    #[test]
    fn selecting_toolchain_file_walks_up_and_prefers_the_legacy_name() {
        let tmp = tempfile::tempdir().unwrap();
        let inner = tmp.path().join("a").join("b");
        std::fs::create_dir_all(&inner).unwrap();
        assert_eq!(selecting_toolchain_file(&inner, None), None);
        let toml = tmp.path().join("a").join("rust-toolchain.toml");
        std::fs::write(&toml, "[toolchain]\nchannel = \"1.80.0\"\n").unwrap();
        assert_eq!(selecting_toolchain_file(&inner, None), Some(toml.clone()));
        assert_eq!(
            selecting_toolchain_file(&inner, Some(OsStr::new(""))),
            Some(toml),
            "an empty RUSTUP_TOOLCHAIN selects nothing"
        );
        let legacy = tmp.path().join("a").join("rust-toolchain");
        std::fs::write(&legacy, "1.80.0\n").unwrap();
        assert_eq!(selecting_toolchain_file(&inner, None), Some(legacy));
        let nearer = inner.join("rust-toolchain.toml");
        std::fs::write(&nearer, "[toolchain]\nchannel = \"stable\"\n").unwrap();
        assert_eq!(selecting_toolchain_file(&inner, None), Some(nearer));
        assert_eq!(
            selecting_toolchain_file(&inner, Some(OsStr::new("1.98.1"))),
            None,
            "RUSTUP_TOOLCHAIN overrides every file"
        );
    }

    #[test]
    fn a_no_prints_the_command_and_installs_nothing() {
        for answer in ["n\n", "no\n", "later\n", ""] {
            let mut machine = Scripted::new(&[(None, RustFound::Missing)]);
            let (result, out) = machine.answer(helper_route(), false, true, answer);
            assert_eq!(result.unwrap(), None);
            assert_eq!(
                out,
                "Rust 1.95 or newer is needed to build nodes. Install it now? [Y/n] \
                 To install it, run: cerulion-install-rust\n",
                "{answer:?}"
            );
            assert!(machine.installs.is_empty(), "{answer:?}");
        }
    }

    #[test]
    fn a_yes_installs_once_and_continues_when_path_finds_rust() {
        for answer in ["\n", "y\n", "YES\n"] {
            let mut machine = Scripted::new(&[
                (None, RustFound::Missing),
                (None, RustFound::Release(V1_98)),
            ]);
            let (result, out) = machine.answer(helper_route(), false, true, answer);
            assert_eq!(result.unwrap(), None);
            assert_eq!(out, INSTALL_PROMPT, "{answer:?}");
            assert_eq!(machine.installs, [helper_route()]);
        }
    }

    /// A fresh machine: the installer put Rust in Cargo's home, which is not
    /// on PATH. The build that follows runs it from there.
    #[test]
    fn a_yes_that_installs_off_path_hands_the_build_cargo_home() {
        let mut machine = Scripted::new(&[
            (None, RustFound::Missing),
            (None, RustFound::Missing),
            (Some(HOME_BIN), RustFound::Release(V1_98)),
        ]);
        let (result, out) = machine.answer(InstallRoute::RustupInit, false, true, "y\n");
        assert_eq!(result.unwrap(), Some(PathBuf::from(HOME_BIN)));
        assert_eq!(
            out,
            "Rust 1.95 or newer is needed to build nodes. Install it now? [Y/n] \
             Cargo is installed in /home/u/.cargo/bin, which is not on your PATH yet. \
             Add it to your PATH: export PATH=\"/home/u/.cargo/bin:$PATH\"\n"
        );
        assert_eq!(machine.installs, [InstallRoute::RustupInit]);
        assert_eq!(
            machine.probe_calls,
            [None, None, Some(PathBuf::from(HOME_BIN))]
        );
    }

    #[test]
    fn an_install_that_leaves_rust_unready_says_so() {
        let mut machine = Scripted::new(&[
            (None, RustFound::Release(V1_80)),
            (None, RustFound::Release(V1_80)),
            (Some(HOME_BIN), RustFound::Release(V1_80)),
        ]);
        let (result, out) = machine.answer(rustup_route(), true, true, "y\n");
        assert_eq!(result.unwrap(), None);
        assert_eq!(
            out,
            format!("{INSTALL_PROMPT}{STILL_NOT_READY}\n{SELECT_STABLE}\n")
        );
        assert_eq!(machine.installs, [rustup_route()]);

        // The helper names its own next step, so no selection line follows,
        // and without a Cargo home there is nowhere else to look.
        let mut machine = Scripted::new(&[(None, RustFound::Missing), (None, RustFound::Missing)]);
        machine.cargo_dir = None;
        let (result, out) = machine.answer(helper_route(), false, true, "\n");
        assert_eq!(result.unwrap(), None);
        assert_eq!(out, format!("{INSTALL_PROMPT}{STILL_NOT_READY}\n"));
        assert_eq!(machine.probe_calls, [None, None]);
    }

    #[test]
    fn a_failed_install_says_how_to_retry_and_probes_no_further() {
        type InstallResult = fn() -> std::io::Result<bool>;
        let vectors: [(InstallResult, &str); 2] = [
            (|| Ok(false), "Rust was not installed. To try again, run: "),
            (
                || Err(std::io::Error::new(std::io::ErrorKind::NotFound, "no curl")),
                "Rust was not installed: no curl. To try again, run: ",
            ),
        ];
        for (install_result, said) in vectors {
            let mut machine = Scripted::new(&[(None, RustFound::Missing)]);
            machine.install_result = install_result;
            let (outcome, out) = machine.answer(InstallRoute::RustupInit, false, true, "y\n");
            assert_eq!(outcome.unwrap(), None);
            assert_eq!(
                out,
                format!("{INSTALL_PROMPT}{said}{RUSTUP_INIT_COMMAND}\n")
            );
            assert_eq!(machine.probe_calls, [None]);
        }
    }

    /// Two runs from the same machine state print the same bytes and make the
    /// same calls, and both match the hand-written oracle.
    #[test]
    fn the_offer_is_deterministic() {
        let run = || {
            let mut machine = Scripted::new(&[
                (None, RustFound::NoDefault),
                (None, RustFound::Missing),
                (Some(HOME_BIN), RustFound::Release(V1_98)),
            ]);
            let (result, out) = machine.answer(helper_route(), true, true, "\n");
            (result.unwrap(), out, machine.installs, machine.probe_calls)
        };
        let expected = (
            Some(PathBuf::from(HOME_BIN)),
            format!(
                "{INSTALL_PROMPT}Cargo is installed in /home/u/.cargo/bin, which is not on your \
                 PATH yet. Add it to your PATH: export PATH=\"/home/u/.cargo/bin:$PATH\"\n"
            ),
            vec![helper_route()],
            vec![None, None, Some(PathBuf::from(HOME_BIN))],
        );
        assert_eq!(run(), expected);
        assert_eq!(run(), expected);
    }

    #[test]
    fn a_terminal_that_cannot_be_written_or_read_installs_nothing() {
        struct Closed;
        impl Write for Closed {
            fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
                Err(std::io::Error::from(std::io::ErrorKind::BrokenPipe))
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        let mut found = vec![RustFound::Missing];
        let mut probe = |_: Option<&Path>| found.pop().unwrap();
        let mut install = |_: &InstallRoute| -> std::io::Result<bool> {
            panic!("nothing installs when the prompt cannot be shown")
        };
        let result = check_rust_with(
            &mut Machine {
                probe: &mut probe,
                install: &mut install,
                route: helper_route(),
                has_rustup: false,
                cargo_dir: None,
                cargo_dir_on_path: false,
            },
            true,
            &mut std::io::Cursor::new(b"y\n".to_vec()),
            &mut Closed,
        );
        assert_eq!(result.unwrap_err().kind(), std::io::ErrorKind::BrokenPipe);

        // An answer that is not UTF-8 is an error, never a yes.
        let mut machine = Scripted::new(&[(None, RustFound::Missing)]);
        let mut invalid = std::io::Cursor::new(vec![b'y', 0xff, b'\n']);
        let (result, out) = machine.run(helper_route(), false, true, &mut invalid);
        assert_eq!(result.unwrap_err().kind(), std::io::ErrorKind::InvalidData);
        assert_eq!(out, INSTALL_PROMPT);
        assert!(machine.installs.is_empty());
    }
}
