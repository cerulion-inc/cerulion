// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion ros2 run` / `cerulion ros2 launch` — env orchestration +
//! transparent `exec()` for stock ROS 2 entry points.
//!
//! The ROS-Compat thesis is "swap the command, not the stack": a ROS 2 user
//! adopts Cerulion transport by prefixing one word —
//!
//! ```text
//! ros2 launch demo.launch.py              →  cerulion ros2 launch demo.launch.py
//! ros2 run demo_nodes_cpp talker          →  cerulion ros2 run demo_nodes_cpp talker
//! ```
//!
//! Both verbs are verbatim pass-through wrappers. This is by design (the
//! verb split): everything after `run` / `launch` is forwarded to the native
//! `ros2 <verb>` untouched — hyphenated tokens, the package form, and any
//! future native flag work by construction, because this module never parses
//! the forwarded arguments. ONE exception, and it is leading-position-only:
//! Cerulion's own [`ADOPT_TAKE_FLAG`] (`--adopt-take`) is split
//! off the FRONT of the args before forwarding; from the first
//! non-Cerulion token on, the verbatim guarantee holds unchanged — and
//! since the launcher-refusal rule that flag is refused here (exit
//! 69): `ros2` spawns the node as a grandchild with `close_fds=True`, so
//! the descriptor-bound heap hook cannot reach it. The verbs' ENTIRE job
//! is:
//!
//! 1. validate the runtime (the rmw cdylib the child needs exists),
//! 2. assemble the child environment (`RMW_IMPLEMENTATION=rmw_cerulion`, an
//!    `LD_LIBRARY_PATH` prepend of the Cerulion lib dir, an
//!    `AMENT_PREFIX_PATH` prepend of a minimal ament prefix so ROS 2's rmw
//!    discovery can dlopen `librmw_cerulion.so`, and — when it is shipped
//!    beside the binary — an `LD_PRELOAD` of the Cerulion heap hook), and
//! 3. `exec()` `ros2 <verb> [args...]`, replacing the `cerulion` process
//!    image so stdio, signals, and the exit code propagate through the
//!    kernel with zero relay code.
//!
//! The same staged environment is what `cerulion graph run` gives each
//! `ros2:` graph entry's child process — [`stage_base_child_env`] is the ONE
//! shared seam, so the paths cannot drift.
//!
//! **Heap-hook auto-injection** (by design): when
//! [`HEAPHOOK_FILENAME`] exists in the lib dir, it is prepended to the
//! child's `LD_PRELOAD` automatically, with ONE `info!` naming the injected
//! path. `CERULION_ROS2_PRELOAD=off` (or `=none`) disables ALL preload
//! injection; any other value names an explicit `.so` that STACKS ahead of
//! the hook (`user.so : hook : ambient`, the user's first — a missing explicit
//! path is a loud [`EXIT_UNAVAILABLE`] error, never a silent drop).
//!
//! Exit contract (small, sysexits-aligned; owned here like `replay_cmd`
//! owns its `EXIT_*` consts):
//!
//! | code | meaning |
//! |------|---------|
//! | —    | success is unreachable — a successful `exec()` BECOMES `ros2`, inheriting its exit code |
//! | 2    | usage error (clap's own: a missing/unknown `cerulion ros2` subcommand; or `CERULION_RMW_ADOPT_TAKE` set in the environment without `--adopt-take`) |
//! | 69   | required Cerulion library missing, or `--adopt-take` given (`EX_UNAVAILABLE`) |
//! | 127  | `ros2` not found on `PATH` |
//! | 1    | any other pre-exec failure |
//!
//! There is deliberately NO argument validation beyond that: a bad package
//! name, a missing launch file, a wrong flag are all `ros2`'s own errors,
//! reported by `ros2` itself after the exec — verbatim forwarding means the
//! native tool owns its argument surface.

use std::path::{Path, PathBuf};

use crate::error::{CliError, CliResult};

/// Usage error. Clap owns one case (a missing/unknown `cerulion ros2`
/// subcommand); [`classify`] owns the other, an ambient
/// [`ADOPT_TAKE_CHILD_ENV`] with no flag. The forwarded
/// ARGUMENTS are still never parsed, so nothing argument-shaped lands here.
pub const EXIT_USAGE: u8 = 2;
/// A required Cerulion library is missing (`EX_UNAVAILABLE` from sysexits).
pub const EXIT_UNAVAILABLE: u8 = 69;
/// The real `ros2` executable was not found on `PATH` (shell convention).
pub const EXIT_ROS2_NOT_FOUND: u8 = 127;
/// Any other pre-exec failure.
pub const EXIT_OTHER: u8 = 1;

/// The rmw cdylib beside the `cerulion` binary: the Linux packages install
/// it there, and in a checkout `cargo` drops it there (`rmw_cerulion`
/// declares `crate-type = ["cdylib", ...]`).
pub const RMW_LIB_FILENAME: &str = "librmw_cerulion.so";

/// The Cerulion heap hook, AUTO-prepended to every ros2 child's `LD_PRELOAD`
/// when this file exists in the lib dir (by design). The library ships as
/// the `cerulion_heaphook` crate (`cargo build -p cerulion_heaphook` drops it
/// beside the binary): it interposes the `malloc` family and exports a versioned
/// handshake the rmw uses to fill an unbounded message field into a loan slot.
/// Preloading it alone is transparent — while no borrow window is armed it
/// forwards to the real allocator; the rmw's windowed-borrow path
/// arms windows for unbounded-type borrows when the handshake is
/// Active. `CERULION_ROS2_PRELOAD=off` is the kill switch.
pub const HEAPHOOK_FILENAME: &str = "libcerulion_heaphook.so";

/// Env var overriding the `current_exe()`-relative lib-dir resolution.
pub const LIB_DIR_ENV: &str = "CERULION_LIB_DIR";

/// The `--adopt-take` launcher flag — Cerulion's
/// OWN flag, recognized only as a LEADING token right after
/// `cerulion ros2 run|launch` (before the first forwarded argument), so the
/// verbatim pass-through guarantee for ros2's surface is untouched: the flag
/// anywhere later is forwarded to `ros2`, which rejects it loudly.
///
/// **Both verbs refuse it**: `ros2` is a Python
/// CLI that spawns the node as a further subprocess with `close_fds=True`,
/// so the inherited descriptor the validated hook rides never reaches the
/// node — see `refuse_adopt_take_under_the_ros2_cli`. Adoption is armed by
/// launching the node executable DIRECTLY with the hook preloaded and
/// [`ADOPT_TAKE_CHILD_ENV`]`=1`.
///
/// The gate behind the flag — child [`ADOPT_TAKE_CHILD_ENV`]=1 plus a
/// hard fail unless the host is Linux/GNU ([`HostAbi`]), the heap hook is
/// actually staged into `LD_PRELOAD`, AND some staged file inspects as the
/// BUILT hook ([`inspect_heaphook_file`] — the lib_dir copy or the file
/// `CERULION_ROS2_PRELOAD` names, not suppressed by the
/// [`ROS2_PRELOAD_ENV`] kill switch) — is RETAINED in
/// `build_plan_with_adopt_gate`; only a two-hop-safe binding scheme
/// can make it reachable again.
pub const ADOPT_TAKE_FLAG: &str = "--adopt-take";

/// The env var `rmw_cerulion` reads at subscription CREATE
/// (`CERULION_RMW_ADOPT_TAKE=1` arms the adopt-take gate; the rmw
/// still requires the Active heap-hook handshake).
///
/// No Cerulion launcher sets it (the launcher-refusal rule: see
/// [`ADOPT_TAKE_FLAG`]), so nothing here guarantees that handshake is even
/// possible — the only way it is set is a direct launch the user staged,
/// where the guarantee is theirs. [`stage_base_child_env`] still REFUSES an
/// ambient value on every Cerulion-spawned child, so a launcher-staged
/// child never carries it.
pub const ADOPT_TAKE_CHILD_ENV: &str = "CERULION_RMW_ADOPT_TAKE";

/// Preload control for the ros2 child:
/// - unset → AUTO: prepend [`HEAPHOOK_FILENAME`] iff it exists in the lib
///   dir (one `info!` names the injected path);
/// - `off` / `none` → NO preload injection at all;
/// - any other value → that `.so` STACKS ahead of the hook (user first, hook
///   kept, ambient kept; a missing path is a loud [`EXIT_UNAVAILABLE`] error);
/// - set but empty → a loud error (never a silent guess).
pub const ROS2_PRELOAD_ENV: &str = "CERULION_ROS2_PRELOAD";

// Message prefixes shared by the error constructors and `classify` so the
// classification can never drift from the strings it classifies (they are
// the SINGLE source of truth for both sides).
const LIB_MISSING_PREFIX: &str = "required Cerulion library missing";
const PRELOAD_MISSING_PREFIX: &str = "CERULION_ROS2_PRELOAD names a missing library";
const ADOPT_TAKE_HOOK_MISSING_PREFIX: &str = "--adopt-take requires the Cerulion heap hook";
const ADOPT_TAKE_HOST_UNSUPPORTED_PREFIX: &str = "--adopt-take requires a Linux/GNU host";
const ADOPT_TAKE_PRELOAD_ORDER_PREFIX: &str =
    "--adopt-take requires the Cerulion heap hook to win malloc";
const ADOPT_TAKE_ENV_CONFLICT_PREFIX: &str = "CERULION_RMW_ADOPT_TAKE is set without --adopt-take";
const ADOPT_TAKE_ROS2_CLI_PREFIX: &str =
    "--adopt-take cannot reach a node launched by `ros2 run` / `ros2 launch`";
const PRELOAD_CHANGED_PREFIX: &str = "a validated library changed before launch";

/// The host ABI a launch runs on, as `--adopt-take`'s platform gate sees
/// it. The heap hook interposes glibc's `malloc`/`free` through
/// `LD_PRELOAD`, so a Linux/GNU host is the only one on which an accepted
/// ELF ever resolves in the child; the launcher refuses anywhere else
/// before it inspects a file (otherwise macOS or a
/// musl host could "accept" a copied ELF hook and launch on the copy path
/// behind a flag that promised adoption). `os` is `std::env::consts::OS`;
/// `env` is the C library flavour the binary was built against
/// (`cfg!(target_env)` — `"gnu"`, `"musl"`, or `""` when the target
/// declares none, e.g. macOS).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HostAbi {
    pub os: &'static str,
    pub env: &'static str,
}

impl HostAbi {
    /// The host this binary is running on.
    pub fn current() -> Self {
        Self {
            os: std::env::consts::OS,
            env: if cfg!(target_env = "gnu") {
                "gnu"
            } else if cfg!(target_env = "musl") {
                "musl"
            } else {
                ""
            },
        }
    }

    /// The one host `--adopt-take` runs on.
    pub const LINUX_GNU: Self = Self {
        os: "linux",
        env: "gnu",
    };

    /// Can the hook's interposer work here at all?
    pub fn supports_adopt_take(self) -> bool {
        self == Self::LINUX_GNU
    }

    fn describe(self) -> String {
        if self.env.is_empty() {
            format!("{} (no GNU C library)", self.os)
        } else {
            format!("{}/{}", self.os, self.env)
        }
    }
}

/// The handshake symbol the rmw resolves FIRST from the preloaded hook
/// (`rmw_cerulion::heaphook` looks it up by this exact name) — the first
/// entry of [`HEAPHOOK_REQUIRED_EXPORTS`], kept as its own name for the
/// version-handshake prose. Never checked as text (an earlier byte scan
/// accepted any file that merely mentioned the name); never alone (a
/// hook exporting only the handshake is an older or incomplete build
/// the rmw would refuse at resolution and silently copy behind).
pub const HEAPHOOK_HANDSHAKE_SYMBOL: &str = "cerulion_heaphook_version";

/// EVERY dynamic export `rmw_cerulion::heaphook` resolves from the
/// preloaded hook — the rmw resolves ALL of them or degrades to the copy
/// path (`active_hook()` returning `Some` IS that proof), so this is the
/// set the `--adopt-take` gate must see DEFINED in the file's `.dynsym`
/// before it promises adoption. Declared in the rmw's own order (the
/// handshake pair, the borrow-window entries, the take-side
/// three). The launcher cannot link the rmw (a cdylib with ROS bindings)
/// nor the hook crate, so the list lives here and is pinned by a test that
/// walks BOTH `crates/rmw_cerulion/src/heaphook.rs` (the `c"…"` names it
/// `dlsym`s — must equal this set exactly) and
/// `cerulion_heaphook/src/exports.rs` (must export each): a symbol added
/// or renamed on either side fails that pin, not the launcher.
pub const HEAPHOOK_REQUIRED_EXPORTS: &[&str] = &[
    "cerulion_heaphook_version",
    "cerulion_heaphook_status",
    "cerulion_heaphook_arm_window",
    "cerulion_heaphook_disarm_window",
    "cerulion_heaphook_window_escape",
    "cerulion_heaphook_window_range_test",
    "cerulion_heaphook_retire_slot",
    "cerulion_heaphook_counter",
    "cerulion_heaphook_register_segment",
    "cerulion_heaphook_unregister_segment",
    "cerulion_heaphook_set_release_callback",
];

/// The most the inspector will read: a built hook is a few hundred KiB, so
/// anything past this is not it — and never a multi-gigabyte read because
/// an env var pointed the launcher at the wrong file.
/// `pub` so the boundary pin derives the ceiling instead of
/// hardcoding it — a hardcoded 64 MiB in a test silently stops testing the
/// boundary the moment this constant moves.
pub const HEAPHOOK_INSPECT_LIMIT: u64 = 64 * 1024 * 1024;

// ELF64 constants the inspector needs (`elf.h` values). ELF64 little-endian
// shared objects for THIS host's machine are the ONLY shape the hook ships;
// every other shape is refused loudly rather than parsed on a guess.
const ELF_MAGIC: &[u8; 4] = b"\x7fELF";
const ELFCLASS64: u8 = 2;
const ELFDATA2LSB: u8 = 1;
const ET_DYN: u16 = 3;
const SHN_UNDEF: u16 = 0;
const STB_GLOBAL: u8 = 1;
const STB_WEAK: u8 = 2;
const STT_OBJECT: u8 = 1;
const STT_FUNC: u8 = 2;
/// `STT_GNU_IFUNC` — a function whose address is chosen at load time by a
/// resolver. It is counted as a function export
/// like any other, because that is what the dynamic loader does with it. An
/// allocator exporting `malloc` as an IFUNC (the shape an optimized build
/// picks to select a CPU-specific implementation) was invisible to this scan,
/// so it could sit ahead of the hook, win `malloc` resolution, and degrade
/// adoption to the copy path behind a flag that promised it — the exact
/// outcome the preload-order guard exists to refuse.
const STT_GNU_IFUNC: u8 = 10;
const PT_LOAD: u32 = 1;
const PT_DYNAMIC: u32 = 2;
// The dynamic-array tags the loader itself reads to find a shared object's
// symbols. `d_tag` values from the gABI; `DT_GNU_HASH` is the GNU
// extension every modern toolchain emits (often INSTEAD of `DT_HASH`).
const DT_NULL: u64 = 0;
const DT_HASH: u64 = 4;
const DT_STRTAB: u64 = 5;
const DT_SYMTAB: u64 = 6;
const DT_STRSZ: u64 = 10;
const DT_SYMENT: u64 = 11;
const DT_GNU_HASH: u64 = 0x6fff_fef5;
const ELF64_DYN_SIZE: usize = 16;
const ELF64_EHDR_SIZE: usize = 64;
const ELF64_PHDR_SIZE: usize = 56;
const ELF64_SYM_SIZE: usize = 24;
const ELF64_HASH_HEADER_SIZE: u64 = 8;
const ELF64_GNU_HASH_HEADER_SIZE: u64 = 16;

/// The `e_machine` this host's dynamic loader will map, from
/// `std::env::consts::ARCH` (`elf.h` `EM_*` values), with the machine's
/// name for the refusal line. `None` for a host the launcher has no
/// mapping for — refused loudly by [`inspect_heaphook_file`] rather than
/// waved through (a valid ELF64 hook built for
/// another architecture must not pass inspection: the loader drops it
/// and the child runs the copy path).
pub fn host_elf_machine() -> Option<(u16, &'static str)> {
    match std::env::consts::ARCH {
        "x86_64" => Some((62, "x86-64")),
        "aarch64" => Some((183, "AArch64")),
        "riscv64" => Some((243, "RISC-V")),
        "powerpc64" => Some((21, "PowerPC64")),
        "s390x" => Some((22, "S/390")),
        "loongarch64" => Some((258, "LoongArch")),
        _ => None,
    }
}

/// The name of an `e_machine` value for a refusal line (the machines the
/// host mapping knows plus "unknown").
fn elf_machine_name(machine: u16) -> &'static str {
    match machine {
        62 => "x86-64",
        183 => "AArch64",
        243 => "RISC-V",
        21 => "PowerPC64",
        22 => "S/390",
        258 => "LoongArch",
        _ => "an unknown machine",
    }
}

/// Why `<lib_dir>/libcerulion_heaphook.so` is not the built hook — one
/// variant per shape, so the refusal names what the operator is looking at
/// (`CERULION_ROS2_PRELOAD` naming the hook's
/// own path while the file is absent is NOT "staged": accepting it lets
/// `--adopt-take` launch a child that runs the copy path behind a
/// flag that promised adoption. The check is a real dynamic-symbol
/// check over the
/// full export set and the host's machine, never a byte scan).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HookFileProblem {
    /// Nothing at the path.
    Absent,
    /// A directory, socket, device — not a file the loader can map.
    NotARegularFile,
    /// The file could not be read (permissions, I/O); carries the OS reason.
    Unreadable(String),
    /// Larger than the inspect ceiling (`HEAPHOOK_INSPECT_LIMIT`, 64 MiB) —
    /// not the hook, and not read.
    TooLarge(u64),
    /// No `\x7fELF` magic: empty, or not a Linux shared object at all.
    NotAnElfSharedObject,
    /// An ELF the preload cannot be: not ELF64, not little-endian, or not
    /// `ET_DYN` (an executable or relocatable object). Carries what was
    /// found.
    UnsupportedElfShape(String),
    /// This host's architecture has no `e_machine` mapping in the launcher
    /// ([`host_elf_machine`]) — refused rather than waved through; carries
    /// the host's `ARCH`.
    UnsupportedHost(&'static str),
    /// A valid ELF64 shared object built for ANOTHER machine: `found` is
    /// the file's `e_machine`, `expected` this host's.
    WrongMachine { found: u16, expected: u16 },
    /// ELF64 headers that do not describe the bytes present (a table or
    /// section past the end of the file, a wrong entry size, an
    /// unterminated string table). Carries which structure.
    MalformedElf(&'static str),
    /// No `PT_LOAD` program header — the dynamic loader has nothing to
    /// map (a section-header-only object).
    NoLoadableSegment,
    /// `.dynsym`/`.dynstr` lie outside every `PT_LOAD` segment — the loader
    /// would map the object without its symbol table.
    DynamicTablesNotLoadable,
    /// No `.dynsym` at all — the object exports nothing (static, stripped
    /// of its dynamic table, or a header-only stub).
    NoDynamicSymbolTable,
    /// Not every entry of [`HEAPHOOK_REQUIRED_EXPORTS`] is a DEFINED
    /// global/weak function-or-object export: `missing` names each one
    /// that is not (in the required order), and `imported_only` the subset
    /// of those that appear in `.dynsym` only as `SHN_UNDEF` — a consumer
    /// of the hook, or an older/incomplete build of it.
    IncompleteExports {
        missing: Vec<&'static str>,
        imported_only: Vec<&'static str>,
    },
}

impl std::fmt::Display for HookFileProblem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Absent => f.write_str("does not exist"),
            Self::NotARegularFile => f.write_str("is not a regular file"),
            Self::Unreadable(reason) => write!(f, "could not be read ({reason})"),
            Self::TooLarge(len) => {
                write!(f, "is {len} bytes — far larger than any built hook")
            }
            Self::NotAnElfSharedObject => f.write_str(
                "is not an ELF shared object (no ELF magic: an empty file, or not a Linux `.so`)",
            ),
            Self::UnsupportedElfShape(found) => write!(
                f,
                "is an ELF the preload cannot be ({found}; the hook is an ELF64 little-endian \
                 shared object)"
            ),
            Self::UnsupportedHost(arch) => write!(
                f,
                "cannot be checked against this host: the launcher has no ELF machine mapping \
                 for `{arch}` (x86_64, aarch64, riscv64, powerpc64, s390x and loongarch64 are \
                 mapped)"
            ),
            Self::WrongMachine { found, expected } => write!(
                f,
                "is built for {} (e_machine={found}) but this host is {} ({}, e_machine={expected}) \
                 — the loader would drop it",
                elf_machine_name(*found),
                std::env::consts::ARCH,
                elf_machine_name(*expected),
            ),
            Self::MalformedElf(what) => {
                write!(f, "is a malformed ELF ({what}) — not the built hook")
            }
            Self::NoLoadableSegment => f.write_str(
                "has no PT_LOAD program header — the dynamic loader cannot map it",
            ),
            Self::DynamicTablesNotLoadable => f.write_str(
                "keeps its `.dynsym`/`.dynstr` outside every PT_LOAD segment — the loader would \
                 map it without its symbol table",
            ),
            Self::NoDynamicSymbolTable => f.write_str(
                "has no dynamic symbol table (`.dynsym`) — it exports nothing, so it is not the \
                 built hook",
            ),
            Self::IncompleteExports {
                missing,
                imported_only,
            } => {
                write!(
                    f,
                    "does not export {} of the {} entry points the rmw resolves as defined \
                     global/weak functions or objects — missing: {}",
                    missing.len(),
                    HEAPHOOK_REQUIRED_EXPORTS.len(),
                    missing.join(", "),
                )?;
                if !imported_only.is_empty() {
                    write!(
                        f,
                        "; of those it only imports (undefined in its `.dynsym`, the mark of a \
                         consumer of the hook): {}",
                        imported_only.join(", ")
                    )?;
                }
                f.write_str(" — an older or incomplete build, or not the hook at all")
            }
        }
    }
}

/// Every hook-file inspection this process has performed. The
/// order oracle — "the host gate runs before any
/// inspection" — needs an observable that a discarded inspection cannot
/// hide, so [`inspect_heaphook_file`] counts itself at entry, whichever
/// caller reached it. Read through [`hook_inspections_for_test`] as a
/// DELTA around one call (tests in a binary share it).
static HOOK_INSPECTIONS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// The running count of [`inspect_heaphook_file`] calls in this process —
/// the `*_for_test` observable pattern (`unified_binding_count_for_test`).
/// Monotone; read a delta around the call under test.
#[doc(hidden)]
pub fn hook_inspections_for_test() -> usize {
    HOOK_INSPECTIONS.load(std::sync::atomic::Ordering::Relaxed)
}

/// Is the file at `path` the BUILT Cerulion heap hook? Regular, readable,
/// bounded in size, ELF-magic'd, an ELF64 little-endian shared object for
/// THIS host's machine, and EXPORTING every entry of
/// [`HEAPHOOK_REQUIRED_EXPORTS`] from its dynamic symbol table (defined,
/// global or weak, a function or object) — each checked in that order so
/// the first problem named is the outermost one. Follows symlinks
/// (`metadata`, as the dynamic loader does). Pure over the filesystem; no
/// `dlopen`, no env beyond the host's `ARCH`.
pub fn inspect_heaphook_file(path: &Path) -> Result<(), HookFileProblem> {
    inspect_heaphook_file_for_machine(path, host_elf_machine())
}

/// [`inspect_heaphook_file`] with the host's `e_machine` mapping handed in
/// — the seam the UNMAPPED-host refusal is tested through on a mapped desk
/// (a real cross-target run is not how that
/// arm is exercised). `None` is what [`host_elf_machine`] returns on a host
/// the launcher has no row for, and it is refused as `UnsupportedHost`
/// before any ELF parse — before the header is even measured, so a 4-byte
/// file gets that answer too. Production goes through the
/// wrapper above.
#[doc(hidden)]
pub fn inspect_heaphook_file_for_machine(
    path: &Path,
    host: Option<(u16, &'static str)>,
) -> Result<(), HookFileProblem> {
    HOOK_INSPECTIONS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    // Open once, fstat that descriptor, read that descriptor. A stat of
    // the path followed by a read of the path would be two resolutions of
    // an environment-controlled name.
    let (bytes, _meta, _file) = open_inspect_bounded(path)?;
    if bytes.len() < ELF_MAGIC.len() || &bytes[..ELF_MAGIC.len()] != ELF_MAGIC {
        return Err(HookFileProblem::NotAnElfSharedObject);
    }
    elf64_exports(&bytes, HEAPHOOK_REQUIRED_EXPORTS, host)
}

/// Bounds-checked little-endian reads; `None` = the field is not inside the
/// buffer, which the caller reports as a malformed structure (the inspector
/// never indexes past what it read).
fn le_u16(bytes: &[u8], at: usize) -> Option<u16> {
    let s = bytes.get(at..at.checked_add(2)?)?;
    Some(u16::from_le_bytes([s[0], s[1]]))
}
fn le_u32(bytes: &[u8], at: usize) -> Option<u32> {
    let s = bytes.get(at..at.checked_add(4)?)?;
    Some(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}
fn le_u64(bytes: &[u8], at: usize) -> Option<u64> {
    let s = bytes.get(at..at.checked_add(8)?)?;
    Some(u64::from_le_bytes([
        s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7],
    ]))
}
/// The slice `[offset, offset + len)` of an ELF64 file, or `None` when the
/// header-declared range is not inside the bytes read.
fn le_range(bytes: &[u8], offset: u64, len: u64) -> Option<&[u8]> {
    let offset = usize::try_from(offset).ok()?;
    let len = usize::try_from(len).ok()?;
    bytes.get(offset..offset.checked_add(len)?)
}

/// Check an ELF64 little-endian shared object for THIS host's machine
/// against `required`: follow its PROGRAM headers to `PT_DYNAMIC`, read
/// `DT_SYMTAB`/`DT_STRTAB`/`DT_STRSZ` and the hash table that gives
/// `.dynsym` its length — the same route `ld.so` takes, and the reason a
/// STRIPPED object (no section headers at all) is read correctly rather
/// than reported as having no symbols —
/// and classify each required
/// name the way `dlsym` would see it — only a DEFINED (`st_shndx !=
/// SHN_UNDEF`) entry with `STB_GLOBAL`/`STB_WEAK` binding and
/// `STT_FUNC`/`STT_OBJECT` type, named EXACTLY (NUL-terminated in
/// `.dynstr`, so a longer name sharing the prefix is a different symbol),
/// counts as exported. The ELF magic is the caller's check; everything else
/// about the shape is refused or reported here, and every read is
/// bounds-checked against the buffer.
fn elf64_scan_exports(
    bytes: &[u8],
    names: &[&str],
    host: Option<(u16, &'static str)>,
) -> Result<(Vec<bool>, Vec<bool>), HookFileProblem> {
    use HookFileProblem::{
        DynamicTablesNotLoadable, MalformedElf, NoDynamicSymbolTable, NoLoadableSegment,
        UnsupportedElfShape, UnsupportedHost, WrongMachine,
    };
    // The host mapping is resolved before the header is even measured:
    // a host the launcher has no row for is
    // refused as `UnsupportedHost` ahead of any parse, so a 4-byte file
    // and a complete hook get the same answer there.
    let (expected_machine, _) = host.ok_or(UnsupportedHost(std::env::consts::ARCH))?;
    if bytes.len() < ELF64_EHDR_SIZE {
        return Err(MalformedElf("shorter than an ELF64 header"));
    }
    let (class, data) = (bytes[4], bytes[5]);
    if class != ELFCLASS64 || data != ELFDATA2LSB {
        return Err(UnsupportedElfShape(format!(
            "EI_CLASS={class} EI_DATA={data}, not ELF64 (2) little-endian (1)"
        )));
    }
    let e_type = le_u16(bytes, 0x10).ok_or(MalformedElf("no e_type"))?;
    if e_type != ET_DYN {
        return Err(UnsupportedElfShape(format!(
            "e_type={e_type}, not ET_DYN ({ET_DYN}) — not a shared object the loader can preload"
        )));
    }
    let e_machine = le_u16(bytes, 0x12).ok_or(MalformedElf("no e_machine"))?;
    if e_machine != expected_machine {
        return Err(WrongMachine {
            found: e_machine,
            expected: expected_machine,
        });
    }
    // The PROGRAM headers first — they are what the dynamic loader maps
    // (a section-header-only object with the
    // right `.dynsym` must not pass: ld.so has nothing to map in it). At least
    // one PT_LOAD, each inside the file; the dynamic tables must sit inside
    // one of them (checked once they are located below).
    let e_phoff = le_u64(bytes, 0x20).ok_or(MalformedElf("no e_phoff"))?;
    let e_phentsize = le_u16(bytes, 0x36).ok_or(MalformedElf("no e_phentsize"))?;
    let e_phnum = le_u16(bytes, 0x38).ok_or(MalformedElf("no e_phnum"))?;
    if e_phnum == 0 {
        return Err(NoLoadableSegment);
    }
    if usize::from(e_phentsize) != ELF64_PHDR_SIZE {
        return Err(MalformedElf("program header entry size is not 56"));
    }
    let phdrs = le_range(bytes, e_phoff, u64::from(e_phnum) * ELF64_PHDR_SIZE as u64).ok_or(
        MalformedElf("program header table runs past the end of the file"),
    )?;
    let mut loads: Vec<Elf64Load> = Vec::new();
    let mut dynamic: Option<(u64, u64)> = None;
    for phdr in phdrs.as_chunks::<ELF64_PHDR_SIZE>().0 {
        let p_type = le_u32(phdr, 0);
        if p_type != Some(PT_LOAD) && p_type != Some(PT_DYNAMIC) {
            continue;
        }
        let p_offset = le_u64(phdr, 0x08).ok_or(MalformedElf("no p_offset"))?;
        let p_vaddr = le_u64(phdr, 0x10).ok_or(MalformedElf("no p_vaddr"))?;
        let p_filesz = le_u64(phdr, 0x20).ok_or(MalformedElf("no p_filesz"))?;
        let p_memsz = le_u64(phdr, 0x28).ok_or(MalformedElf("no p_memsz"))?;
        if p_type == Some(PT_DYNAMIC) {
            if le_range(bytes, p_offset, p_filesz).is_none() {
                return Err(MalformedElf(
                    "the PT_DYNAMIC segment runs past the end of the file",
                ));
            }
            dynamic = Some((p_offset, p_filesz));
            continue;
        }
        if le_range(bytes, p_offset, p_filesz).is_none() {
            return Err(MalformedElf(
                "a PT_LOAD segment runs past the end of the file",
            ));
        }
        // A segment whose memory image is smaller than its
        // FILE image is malformed — the loader maps `p_memsz` bytes, so
        // the excess file bytes are never in the image at all.
        if p_memsz < p_filesz {
            return Err(MalformedElf(
                "a PT_LOAD segment declares less memory than file image",
            ));
        }
        loads.push(Elf64Load {
            offset: p_offset,
            vaddr: p_vaddr,
            filesz: p_filesz,
            memsz: p_memsz,
        });
    }
    if loads.is_empty() {
        return Err(NoLoadableSegment);
    }
    // The tables come from PT_DYNAMIC, not
    // from the section headers. `strip` removes the section header table
    // outright while leaving PT_DYNAMIC, `.dynsym` and `.dynstr` intact —
    // and a stripped `.so` is the ORDINARY shape for a packaged allocator
    // (jemalloc, tcmalloc). Reading section headers therefore answered
    // "no dynamic symbol table" for exactly the files the malloc-order
    // guard exists to catch, and `preload_allocator_exports` turned that
    // into an EMPTY allocator list: a foreign allocator staged ahead of
    // the hook was waved through. This reader now asks what ld.so asks.
    let (dyn_off, dyn_size) = dynamic.ok_or(NoDynamicSymbolTable)?;
    let dynamic_entries = le_range(bytes, dyn_off, dyn_size).ok_or(MalformedElf(
        "the PT_DYNAMIC segment runs past the end of the file",
    ))?;
    let mut symtab_vaddr: Option<u64> = None;
    let mut strtab_vaddr: Option<u64> = None;
    let mut strsz: Option<u64> = None;
    let mut syment: Option<u64> = None;
    let mut hash_vaddr: Option<u64> = None;
    let mut gnu_hash_vaddr: Option<u64> = None;
    for entry in dynamic_entries.as_chunks::<ELF64_DYN_SIZE>().0 {
        let d_tag = le_u64(entry, 0).ok_or(MalformedElf("no d_tag"))?;
        let d_val = le_u64(entry, 8).ok_or(MalformedElf("no d_un"))?;
        match d_tag {
            DT_NULL => break,
            DT_SYMTAB => symtab_vaddr = Some(d_val),
            DT_STRTAB => strtab_vaddr = Some(d_val),
            DT_STRSZ => strsz = Some(d_val),
            DT_SYMENT => syment = Some(d_val),
            DT_HASH => hash_vaddr = Some(d_val),
            DT_GNU_HASH => gnu_hash_vaddr = Some(d_val),
            _ => {}
        }
    }
    // A `.so` with no DT_SYMTAB/DT_STRTAB resolves nothing through the
    // loader; it has no dynamic symbol table in the only sense that
    // matters here.
    let (symtab_vaddr, strtab_vaddr) = match (symtab_vaddr, strtab_vaddr) {
        (Some(sym), Some(str_)) => (sym, str_),
        _ => return Err(NoDynamicSymbolTable),
    };
    if syment.unwrap_or(ELF64_SYM_SIZE as u64) != ELF64_SYM_SIZE as u64 {
        return Err(MalformedElf("DT_SYMENT is not 24"));
    }
    // Every dynamic-table address is a VIRTUAL address: translate through
    // the PT_LOAD segments, which is also what makes "the tables are
    // inside the mapped image" structural rather than a separate check —
    // an address outside every PT_LOAD has no file offset at all.
    // The complete range must lie inside
    // one PT_LOAD segment's file image, not just its start address. A
    // table whose start is mapped and whose tail runs past `p_filesz`
    // would otherwise be classified from bytes the loader never maps —
    // the reader would answer a question about a file region that does
    // not exist at run time, which is the same "read what ld.so reads"
    // rule the PT_DYNAMIC rewrite was for, applied to the far end.
    // Both bounds, not whichever is expected to be tighter:
    // the range must lie inside the segment's FILE image, because those
    // are the bytes being read, AND inside its MEMORY image, because that
    // is what the loader maps. A well-formed segment has
    // `memsz >= filesz` so the file bound binds first; the memory bound
    // is what keeps that an OBSERVATION rather than an assumption.
    let to_offset = |vaddr: u64, size: u64| -> Option<u64> {
        loads.iter().find_map(|load| {
            let file_end = load.vaddr.checked_add(load.filesz)?;
            let mem_end = load.vaddr.checked_add(load.memsz)?;
            let wanted_end = vaddr.checked_add(size)?;
            if vaddr < load.vaddr || wanted_end > file_end || wanted_end > mem_end {
                return None;
            }
            Some(load.offset.saturating_add(vaddr - load.vaddr))
        })
    };
    let dynstr_size = strsz.ok_or(MalformedElf("no DT_STRSZ"))?;
    let dynstr_off = to_offset(strtab_vaddr, dynstr_size).ok_or(DynamicTablesNotLoadable)?;
    // `.dynsym` carries no length in the dynamic array — the loader learns
    // it from the hash table, so this reader does too. No hash table at
    // all is a refusal, never a silent "exports nothing".
    let symbol_count = elf64_dynsym_count(bytes, &to_offset, hash_vaddr, gnu_hash_vaddr)?;
    let sh_size = symbol_count
        .checked_mul(ELF64_SYM_SIZE as u64)
        .ok_or(MalformedElf(".dynsym symbol count overflows"))?;
    // The symbol table's extent is only known once the hash table has
    // given the count, so its mapped-range check happens here.
    let sh_offset = to_offset(symtab_vaddr, sh_size).ok_or(DynamicTablesNotLoadable)?;
    // The complete-range checks above already required each
    // table to sit inside a PT_LOAD's FILE image, and a PT_LOAD that
    // overruns the file was refused before that — so under correct code
    // these two reads cannot fail, and they keep their OWN messages
    // precisely because of that. A build where either message appears is
    // a build whose mapped-range check let a table through, which is what
    // `inspect_heaphook_file_classifies_each_problem` detects: the
    // fixtures expect the not-loadable verdict, and a start-address-only
    // check turns that into one of these instead.
    let symbols = le_range(bytes, sh_offset, sh_size)
        .ok_or(MalformedElf(".dynsym runs past the end of the file"))?;
    let dynstr = le_range(bytes, dynstr_off, dynstr_size)
        .ok_or(MalformedElf(".dynstr runs past the end of the file"))?;
    let (entries, remainder) = symbols.as_chunks::<ELF64_SYM_SIZE>();
    if !remainder.is_empty() {
        return Err(MalformedElf(
            ".dynsym size is not a whole number of entries",
        ));
    }

    let mut exported = vec![false; names.len()];
    let mut imported = vec![false; names.len()];
    for sym in entries {
        let st_name = le_u32(sym, 0).ok_or(MalformedElf("no st_name"))?;
        let st_info = sym[4];
        let st_shndx = le_u16(sym, 6).ok_or(MalformedElf("no st_shndx"))?;
        let tail = usize::try_from(st_name)
            .ok()
            .and_then(|at| dynstr.get(at..))
            .ok_or(MalformedElf("a .dynsym name offset runs past .dynstr"))?;
        let end = tail
            .iter()
            .position(|b| *b == 0)
            .ok_or(MalformedElf(".dynstr is not NUL-terminated"))?;
        let Some(slot) = names
            .iter()
            .position(|name| name.as_bytes() == &tail[..end])
        else {
            continue;
        };
        let (bind, kind) = (st_info >> 4, st_info & 0xf);
        if !matches!(bind, STB_GLOBAL | STB_WEAK)
            || !matches!(kind, STT_FUNC | STT_OBJECT | STT_GNU_IFUNC)
        {
            // A local, a section symbol, a NOTYPE marker — not what `dlsym`
            // resolves a call through.
            continue;
        }
        if st_shndx == SHN_UNDEF {
            imported[slot] = true;
            continue;
        }
        exported[slot] = true;
    }
    Ok((exported, imported))
}

/// Does an ELF64 shared object EXPORT every one of `required` as a defined
/// global/weak function-or-object? The hook-shaped consumer of
/// [`elf64_scan_exports`]: an incomplete set is reported with every missing
/// name (and which of those are present only as imports).
fn elf64_exports(
    bytes: &[u8],
    required: &[&'static str],
    host: Option<(u16, &'static str)>,
) -> Result<(), HookFileProblem> {
    let (exported, imported) = elf64_scan_exports(bytes, required, host)?;
    let missing: Vec<&'static str> = required
        .iter()
        .zip(&exported)
        .filter(|(_, exported)| !**exported)
        .map(|(name, _)| *name)
        .collect();
    if missing.is_empty() {
        return Ok(());
    }
    let imported_only = required
        .iter()
        .zip(exported.iter().zip(&imported))
        .filter(|(_, (exported, imported))| !**exported && **imported)
        .map(|(name, _)| *name)
        .collect();
    Err(HookFileProblem::IncompleteExports {
        missing,
        imported_only,
    })
}

/// The C allocator entry points an interposer wins by exporting them. A
/// preload staged AHEAD of the hook that defines any of these wins
/// `malloc` resolution instead of the hook (the dynamic loader resolves
/// symbols left to right across the preload list), and the rmw's
/// handshake then reads `DegradeForeignAllocator` and runs the copy path
/// — behind a flag that promised adoption (the same silent-degrade class
/// the hook inspection refuses).
pub const MALLOC_FAMILY_EXPORTS: &[&str] = &[
    "malloc",
    "free",
    "calloc",
    "realloc",
    "posix_memalign",
    "aligned_alloc",
    "memalign",
    "valloc",
    "pvalloc",
];

/// The export/import classification for an arbitrary name set — the seam
/// the real-artifact test arm reads the built hook's dynamic symbols
/// through, so "what this launcher counts as exported" and "what the test
/// asserts the shipped `.so` exports" are ONE implementation rather than
/// two that can drift. Mirrors `hook_inspections_for_test`'s convention.
pub fn scan_exports_for_test(
    bytes: &[u8],
    names: &[&str],
) -> Result<(Vec<bool>, Vec<bool>), HookFileProblem> {
    elf64_scan_exports(bytes, names, host_elf_machine())
}

/// One PT_LOAD segment, as the dynamic-table reader needs it: a virtual
/// address range, where it lives in the file, and how much of it the
/// loader maps.
///
/// Both extents are kept. Dropping
/// `p_memsz` on the argument that `memsz >= filesz` makes the file image
/// the tighter bound is wrong — that is true of a well-formed segment, and precisely what
/// a malformed one violates. A PT_LOAD declaring `p_memsz < p_filesz` has
/// file bytes beyond its own memory image, so bounding tables by `filesz`
/// alone would classify a symbol table from bytes the loader never maps.
/// Such a segment is refused outright, and every table range must
/// satisfy both bounds.
struct Elf64Load {
    offset: u64,
    vaddr: u64,
    filesz: u64,
    memsz: u64,
}

/// How many entries `.dynsym` holds. The dynamic array carries no size for
/// it — the loader derives the count from the hash table, so this does the
/// same: `DT_HASH`'s `nchain` IS the symbol count, and `DT_GNU_HASH` needs
/// its last chain walked. An object with NEITHER hash table exports
/// nothing the loader could resolve by name, and is refused rather than
/// reported as "no allocator": every real shared object has one, so its
/// absence means the file is not what it claims, not that it is harmless.
fn elf64_dynsym_count(
    bytes: &[u8],
    to_offset: &dyn Fn(u64, u64) -> Option<u64>,
    hash_vaddr: Option<u64>,
    gnu_hash_vaddr: Option<u64>,
) -> Result<u64, HookFileProblem> {
    use HookFileProblem::{DynamicTablesNotLoadable, MalformedElf, NoDynamicSymbolTable};
    // DT_HASH first: its second word is the count outright.
    if let Some(vaddr) = hash_vaddr {
        let off = to_offset(vaddr, ELF64_HASH_HEADER_SIZE).ok_or(DynamicTablesNotLoadable)?;
        let head = le_range(bytes, off, ELF64_HASH_HEADER_SIZE)
            .ok_or(MalformedElf("DT_HASH runs past the end of the file"))?;
        return le_u32(head, 4)
            .map(u64::from)
            .ok_or(MalformedElf("DT_HASH has no nchain"));
    }
    let Some(vaddr) = gnu_hash_vaddr else {
        return Err(NoDynamicSymbolTable);
    };
    let off = to_offset(vaddr, ELF64_GNU_HASH_HEADER_SIZE).ok_or(DynamicTablesNotLoadable)?;
    let head = le_range(bytes, off, ELF64_GNU_HASH_HEADER_SIZE)
        .ok_or(MalformedElf("DT_GNU_HASH runs past the end of the file"))?;
    let nbuckets = u64::from(le_u32(head, 0).ok_or(MalformedElf("DT_GNU_HASH has no nbuckets"))?);
    let symoffset = u64::from(le_u32(head, 4).ok_or(MalformedElf("DT_GNU_HASH has no symoffset"))?);
    let bloom_size =
        u64::from(le_u32(head, 8).ok_or(MalformedElf("DT_GNU_HASH has no maskwords"))?);
    if nbuckets == 0 {
        // No buckets at all: every symbol is below `symoffset`, i.e. none
        // is hashed. `symoffset` is still the table length.
        return Ok(symoffset);
    }
    let buckets_off = off
        .checked_add(ELF64_GNU_HASH_HEADER_SIZE)
        .and_then(|a| a.checked_add(bloom_size.checked_mul(8)?))
        .ok_or(MalformedElf("DT_GNU_HASH bloom filter overflows"))?;
    // The bucket table's own extent, against the MAPPED image as well as
    // the file (every dynamic table's complete range).
    let buckets_size = nbuckets.saturating_mul(4);
    let buckets_vaddr = vaddr
        .checked_add(ELF64_GNU_HASH_HEADER_SIZE)
        .and_then(|a| a.checked_add(bloom_size.checked_mul(8)?))
        .ok_or(MalformedElf("DT_GNU_HASH bloom filter overflows"))?;
    if to_offset(buckets_vaddr, buckets_size).is_none() {
        return Err(DynamicTablesNotLoadable);
    }
    let buckets = le_range(bytes, buckets_off, buckets_size).ok_or(MalformedElf(
        "DT_GNU_HASH buckets run past the end of the file",
    ))?;
    let last = buckets
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| u32::from_le_bytes(*b))
        .max()
        .map(u64::from)
        .unwrap_or(0);
    if last < symoffset {
        return Ok(symoffset);
    }
    // Walk that bucket's chain to its terminator (low bit set); the index
    // it ends on is the highest symbol in the table.
    let chain_off = buckets_off
        .checked_add(
            nbuckets
                .checked_mul(4)
                .ok_or(MalformedElf("DT_GNU_HASH bucket table overflows"))?,
        )
        .ok_or(MalformedElf("DT_GNU_HASH chain table overflows"))?;
    let mut index = last - symoffset;
    loop {
        let at = chain_off
            .checked_add(
                index
                    .checked_mul(4)
                    .ok_or(MalformedElf("DT_GNU_HASH chain index overflows"))?,
            )
            .ok_or(MalformedElf("DT_GNU_HASH chain index overflows"))?;
        let word = le_range(bytes, at, 4)
            .and_then(|w| le_u32(w, 0))
            .ok_or(MalformedElf(
                "DT_GNU_HASH chain runs past the end of the file",
            ))?;
        index += 1;
        if word & 1 == 1 {
            break;
        }
    }
    symoffset
        .checked_add(index)
        .ok_or(MalformedElf("DT_GNU_HASH symbol count overflows"))
}

/// Which of [`MALLOC_FAMILY_EXPORTS`] the shared object at `path` DEFINES
/// as global/weak exports — i.e. whether, preloaded ahead of the hook, it
/// would win `malloc`.
///
/// The failure cases are split in two, because
/// collapsing them lets a stripped allocator through. An entry the
/// loader CANNOT MAP — absent, unreadable, not an ELF64 shared object for
/// this host — is dropped by ld.so and cannot win anything, so it correctly
/// reports NO allocator. An entry the loader WOULD map but whose exports
/// this launcher could not enumerate is a DIFFERENT answer: "unknown", and
/// serving "unknown" as "no allocator" is precisely the silent degrade the
/// guard exists to prevent. That case is an `Err` the caller refuses on.
/// Reuses the hook inspector's parser, so what "exports" means is the same
/// on both questions.
pub fn preload_allocator_exports(path: &Path) -> Result<Vec<&'static str>, HookFileProblem> {
    // One open, one fstat, read that descriptor (see
    // `open_inspect_bounded`). An absent / unreadable / non-regular entry is
    // correctly "no allocator" here.
    //
    // An over-limit one is not, and that verdict passes through:
    // the size ceiling is THIS launcher's inspection-read budget, not a loader
    // limitation — `ld.so` maps a >64 MiB `.so` perfectly well. Reporting such
    // an entry as "no allocator" is exactly the "unknown served as no" degrade
    // the doc above says this split exists to prevent: an unstripped allocator
    // build (a debug `libtcmalloc`, say) staged ahead of the hook would be
    // waved through, win `malloc` left-to-right, and silently put the run on
    // the copy path behind a flag that promised adoption.
    let (bytes, _meta, _file) = match open_inspect_bounded(path) {
        Ok(v) => v,
        Err(e @ HookFileProblem::TooLarge(_)) => return Err(e),
        Err(_) => return Ok(Vec::new()),
    };
    if bytes.len() < ELF_MAGIC.len() || &bytes[..ELF_MAGIC.len()] != ELF_MAGIC {
        return Ok(Vec::new());
    }
    match elf64_scan_exports(&bytes, MALLOC_FAMILY_EXPORTS, host_elf_machine()) {
        Ok((exported, _)) => Ok(MALLOC_FAMILY_EXPORTS
            .iter()
            .zip(&exported)
            .filter(|(_, exported)| **exported)
            .map(|(name, _)| *name)
            .collect()),
        // Shapes ld.so itself will not map: it cannot win `malloc`.
        Err(
            problem @ (HookFileProblem::UnsupportedElfShape(_)
            | HookFileProblem::WrongMachine { .. }
            | HookFileProblem::NoLoadableSegment),
        ) => {
            tracing::debug!(
                path = %path.display(),
                problem = %problem,
                "preload entry is not a loadable shared object for this host — it cannot \
                 win malloc, so it is not treated as an allocator"
            );
            Ok(Vec::new())
        }
        // Everything else means its exports could not be READ. Unknown is
        // not "none".
        Err(problem) => Err(problem),
    }
}

/// The native `ros2` verb a `cerulion ros2` invocation passes through to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ros2NativeVerb {
    /// `cerulion ros2 run …` → `ros2 run …`.
    Run,
    /// `cerulion ros2 launch …` → `ros2 launch …`.
    Launch,
}

impl Ros2NativeVerb {
    /// The verb token handed to the native `ros2`.
    pub fn as_str(self) -> &'static str {
        match self {
            Ros2NativeVerb::Run => "run",
            Ros2NativeVerb::Launch => "launch",
        }
    }
}

/// The identity of a file the launcher INSPECTED and then hands to the
/// child — the TOCTOU guard.
///
/// `metadata` FOLLOWS symlinks, deliberately: so does `ld.so`, and a
/// symlinked hook is legitimate (the inspector has an arm for it). The
/// property that matters is therefore not "this path is not a symlink"
/// but "the object this path resolved to at read time is the object
/// still there at exec" — a symlink re-pointed between the two
/// changes the resolved inode and is caught by exactly the same check,
/// while a legitimate symlink keeps working. `O_NOFOLLOW` would refuse the
/// legitimate one and still say nothing about the window.
#[derive(Debug, Clone)]
pub struct InspectedFile {
    /// The path as it will appear in the child's environment.
    pub path: PathBuf,
    /// `(dev, ino, size, mtime_ns)` of the OPENED OBJECT the launcher
    /// read, or `None` when the path could not be opened at plan time.
    ///
    /// `None` is a recorded state, not an
    /// omission. Dropping an unidentifiable path from
    /// the list entirely would let a file that vanished — or one that appeared
    /// between plan and exec — pass `verify_inspected` unexamined and
    /// be loaded by ld.so having been validated by nothing. An entry
    /// that was absent must still be absent at exec; one that was present
    /// must still be the same object.
    pub identity: Option<FileId>,
    /// The OPEN descriptor the launcher validated, when this entry is
    /// bound to the launch by fd rather than by name (descriptor
    /// binding). `path` is then `/proc/self/fd/<N>`, the child inherits
    /// this descriptor across `exec`, and `verify_inspected` identifies
    /// it with `fstat` — no name is resolved a second time, so there is
    /// no window for a substitution at all.
    pub fd: Option<std::sync::Arc<std::fs::File>>,
}

/// The fully-assembled exec plan: program + verbatim args + the env pairs to
/// OVERLAY onto the inherited environment (nothing is removed from it).
#[derive(Debug, Clone)]
pub struct Ros2Plan {
    /// The program to exec (always `"ros2"` in production; tests point it at
    /// a fixture script to prove env staging + exit-code transparency).
    pub program: String,
    /// The native verb token followed by the forwarded args, verbatim.
    pub args: Vec<String>,
    /// Env pairs to overlay onto the child environment.
    pub env: Vec<(String, String)>,
    /// TOCTOU guard: every file the launcher
    /// inspected and then names to the child, with the identity it had
    /// when inspected. The launcher validates by READING these paths and
    /// then hands ld.so the path STRINGS, so a swap between the two
    /// launches an object nothing validated. [`exec_ros2`] re-verifies
    /// immediately before `exec` and refuses on any change.
    ///
    /// The window is closed, not narrowed. The
    /// validated hook is handed to the child through the descriptor the
    /// launcher inspected — `LD_PRELOAD` names `/proc/self/fd/<N>`, the
    /// fd survives `exec`, and the loader resolves that to the inherited
    /// descriptor — so no name is resolved a second time and nothing can
    /// substitute a different file. Entries that are NOT fd-bound (ambient
    /// preloads the launcher does not validate, and the rmw library, which
    /// rides `LD_LIBRARY_PATH` as a directory and so has no descriptor to
    /// bind) keep the identity re-check, which narrows their window
    /// without closing it. The price of the closed one: an operator
    /// reading the child's environment sees `/proc/self/fd/<N>` where the
    /// hook's path would otherwise be.
    pub inspected: Vec<InspectedFile>,
}

/// Resolve the Cerulion lib dir: `$CERULION_LIB_DIR` (absolutized) when set,
/// else the directory of the running `cerulion` binary (`current_exe()`),
/// which is where `cargo` drops the cdylibs.
pub fn resolve_lib_dir() -> CliResult<PathBuf> {
    if let Some(raw) = std::env::var_os(LIB_DIR_ENV) {
        if raw.is_empty() {
            return Err(CliError::Validation(format!(
                "{LIB_DIR_ENV} is set but empty — unset it to use the default (the `cerulion` \
                 binary's directory) or point it at the directory containing {RMW_LIB_FILENAME}"
            )));
        }
        return Ok(std::path::absolute(PathBuf::from(raw))?);
    }
    let exe = std::env::current_exe()?;
    match exe.parent() {
        Some(dir) => Ok(dir.to_path_buf()),
        None => Err(CliError::Validation(format!(
            "cannot resolve the Cerulion lib dir: `{}` has no parent directory — set \
             {LIB_DIR_ENV} to the directory containing {RMW_LIB_FILENAME}",
            exe.display()
        ))),
    }
}

/// Resolve (and if needed CREATE) the minimal ament prefix whose
/// `lib/librmw_cerulion.so` lets ROS 2's rmw discovery dlopen the Cerulion
/// rmw. It is the prefix shape the latency suite's ROS 2 runner stages: a prefix dir
/// prepended to `AMENT_PREFIX_PATH` whose `lib/` carries the cdylib.
///
/// Two shapes:
/// - `lib_dir` already named `.../lib` (an installed layout): its PARENT is
///   the prefix — nothing is written.
/// - anything else (the cargo `target/<profile>/` layout): a per-lib-dir
///   staging prefix under `~/.cerulion/ros2/` whose `lib/` holds a SYMLINK to
///   the real cdylib. Keyed by a hash of `lib_dir` so two Cerulion builds
///   never fight over one link; the symlink is refreshed when stale, so the
///   prefix always tracks the freshly-built library. This is the module's one
///   deliberate side effect — it runs in the dispatch, never in the pure
///   plan builder.
pub fn stage_ament_prefix(lib_dir: &Path) -> CliResult<PathBuf> {
    let state_dir = dirs::home_dir()
        .ok_or_else(|| {
            CliError::Validation(
                "cannot resolve a home directory for the ament staging prefix — set HOME"
                    .to_string(),
            )
        })?
        .join(".cerulion")
        .join("ros2");
    stage_ament_prefix_under(&state_dir, lib_dir)
}

/// The testable core of [`stage_ament_prefix`]: `state_dir` is where staging
/// prefixes live (`~/.cerulion/ros2` in production; a tempdir in tests).
/// Is `lib_dir` already the `lib/` of an ament-shaped prefix
/// (`<prefix>/lib/librmw_cerulion.so`)? Then nothing needs staging.
fn is_installed_layout(lib_dir: &Path) -> bool {
    lib_dir.file_name().is_some_and(|n| n == "lib") && lib_dir.parent().is_some()
}

/// WHERE [`stage_ament_prefix_under`] puts the prefix — pure, creates
/// nothing, touches no filesystem.
///
/// Split out for the launcher-refusal rule: the `--adopt-take` refusal names
/// this path in its direct-launch recipe and must be decidable BEFORE any
/// staging happens, so the two cannot be the same function. Both call it,
/// so the path the refusal prints is the path the staging would create.
fn ament_prefix_path_under(state_dir: &Path, lib_dir: &Path) -> PathBuf {
    if is_installed_layout(lib_dir) {
        if let Some(prefix) = lib_dir.parent() {
            return prefix.to_path_buf();
        }
    }
    let key = fnv1a64(lib_dir.as_os_str().as_encoded_bytes());
    state_dir.join(format!("prefix-{key:016x}"))
}

/// `ament_prefix_path_under` (private) against the real state dir — `None`
/// when no
/// home directory resolves, which is the ONE thing `stage_ament_prefix`
/// errors on before doing any work. The refusal degrades to naming the
/// shape rather than failing, because a user with an unusable HOME still
/// deserves the refusal's answer rather than a staging error.
pub fn ament_prefix_path(lib_dir: &Path) -> Option<PathBuf> {
    let state_dir = dirs::home_dir()?.join(".cerulion").join("ros2");
    Some(ament_prefix_path_under(&state_dir, lib_dir))
}

pub fn stage_ament_prefix_under(state_dir: &Path, lib_dir: &Path) -> CliResult<PathBuf> {
    if is_installed_layout(lib_dir) {
        // Installed layout: `<prefix>/lib/librmw_cerulion.so` — the prefix
        // already has the ament shape, so no staging is needed.
        return Ok(ament_prefix_path_under(state_dir, lib_dir));
    }
    let prefix = ament_prefix_path_under(state_dir, lib_dir);
    let staged_lib = prefix.join("lib");
    let link = staged_lib.join(RMW_LIB_FILENAME);
    let target = lib_dir.join(RMW_LIB_FILENAME);
    let stage_err = |e: std::io::Error| {
        CliError::Validation(format!(
            "failed to stage the minimal ament prefix at `{}`: {e}",
            prefix.display()
        ))
    };
    std::fs::create_dir_all(&staged_lib).map_err(stage_err)?;
    match std::fs::read_link(&link) {
        Ok(existing) if existing == target => return Ok(prefix),
        Ok(_) => {
            // Stale link from a different lib dir landing on the same key
            // (practically unreachable given the hash, but never serve a
            // wrong target silently) — refresh it.
            std::fs::remove_file(&link).map_err(stage_err)?;
        }
        Err(_) => {
            // Not a symlink: either absent (the common first run) or some
            // stray regular file — clear the way. A failed remove of an
            // absent path is fine; symlink creation below reports loudly.
            let _ = std::fs::remove_file(&link);
        }
    }
    #[cfg(unix)]
    std::os::unix::fs::symlink(&target, &link).map_err(stage_err)?;
    #[cfg(not(unix))]
    return Err(CliError::Validation(
        "`cerulion ros2` is only supported on Unix platforms (LD_PRELOAD + rmw_cerulion \
         require the Unix dynamic loader and exec() semantics)"
            .to_string(),
    ));
    #[cfg(unix)]
    Ok(prefix)
}

/// THE production entry for `cerulion ros2 run|launch` — resolve the lib
/// dir, decide the [`ADOPT_TAKE_FLAG`] refusal, and only THEN stage
/// anything and build the plan.
///
/// The order is the point, and it is why this lives in the engine rather
/// than in the binary's dispatch: a dispatch that
/// calls [`stage_ament_prefix`] first resolves HOME and
/// CREATES `~/.cerulion/ros2/prefix-<hash>/lib/librmw_cerulion.so`. So a
/// refused `--adopt-take` could leave staging state behind, or fail with a
/// "cannot resolve a home directory" error instead of the
/// host-independent exit 69 — contradicting the refusal's own promise that
/// it answers before anything is touched. An ordering rule enforced at the
/// call site is an ordering rule waiting to be got wrong at the next call
/// site; here it is the only order there is.
///
/// [`resolve_lib_dir`] still runs first, deliberately: it only READS (an
/// env var, else `current_exe`), creates nothing, and the refusal's recipe
/// names paths under the lib dir. If it fails, `--adopt-take` reports that
/// failure rather than the refusal — a genuinely prior fact, with no
/// filesystem left behind either way.
#[cfg(unix)]
pub fn plan_ros2_passthrough(verb: Ros2NativeVerb, args: &[String]) -> CliResult<Ros2Plan> {
    let lib_dir = resolve_lib_dir()?;
    refuse_adopt_take_if_requested(args, &lib_dir)?;
    let ament_prefix = stage_ament_prefix(&lib_dir)?;
    build_ros2_passthrough_plan(verb, args, &lib_dir, &ament_prefix)
}

/// The launcher-refusal rule, decidable with nothing staged: `Err` iff `args`
/// carries a LEADING [`ADOPT_TAKE_FLAG`]. Separate from
/// [`build_ros2_passthrough_plan_for_host`]'s own copy of the same refusal
/// so the plan builder keeps refusing on its own terms (defence in depth,
/// and what the engine's arms drive) while production can answer earlier.
/// Both render the IDENTICAL message: the prefix path comes from
/// [`ament_prefix_path`], which is what the staging would have produced.
pub fn refuse_adopt_take_if_requested(args: &[String], lib_dir: &Path) -> CliResult<()> {
    if split_adopt_take_flag(args).0 {
        let prefix = ament_prefix_path(lib_dir);
        return Err(refuse_adopt_take_under_the_ros2_cli(
            lib_dir,
            prefix.as_deref(),
        ));
    }
    Ok(())
}

/// Build the pass-through exec plan for `cerulion ros2 <verb> [args...]`.
/// The forwarded `args` are NEVER parsed — they land in the plan verbatim
/// after the native verb token. Validation is env-only (it never
/// degrades silently): `<lib_dir>/librmw_cerulion.so` missing →
/// [`EXIT_UNAVAILABLE`]; a preload misconfiguration per
/// [`ROS2_PRELOAD_ENV`]'s contract.
pub fn build_ros2_passthrough_plan(
    verb: Ros2NativeVerb,
    args: &[String],
    lib_dir: &Path,
    ament_prefix: &Path,
) -> CliResult<Ros2Plan> {
    build_ros2_passthrough_plan_for_host(verb, args, lib_dir, ament_prefix, HostAbi::current())
}

/// [`build_ros2_passthrough_plan`] with the host ABI handed in — the seam
/// the platform gate is tested through (a macOS desk can drive the
/// Linux/GNU acceptance arms and the refusal arm alike). Production calls
/// the wrapper above with [`HostAbi::current`].
///
/// This is where [`ADOPT_TAKE_FLAG`] is
/// REFUSED for `cerulion ros2 run` / `ros2 launch` — see
/// `refuse_adopt_take_under_the_ros2_cli` below. Named in prose, not
/// intra-doc-linked: a private item resolves and warns under
/// `private_intra_doc_links`, a `test-seams`-gated one does not resolve at
/// all with the feature off (`broken_intra_doc_links`), and the docs gate
/// denies both. The gate itself is RETAINED, one call away in
/// `build_plan_with_adopt_gate`, which is what a two-hop-safe binding
/// scheme will re-enable.
pub fn build_ros2_passthrough_plan_for_host(
    verb: Ros2NativeVerb,
    args: &[String],
    lib_dir: &Path,
    ament_prefix: &Path,
    host: HostAbi,
) -> CliResult<Ros2Plan> {
    if split_adopt_take_flag(args).0 {
        return Err(refuse_adopt_take_under_the_ros2_cli(
            lib_dir,
            Some(ament_prefix),
        ));
    }
    build_plan_with_adopt_gate(verb, args, lib_dir, ament_prefix, host)
}

/// The retained `--adopt-take` launch gate, reached without the
/// two-hop launcher refusal — the seam the retained gate's arms in
/// `tests/ros2_cmd_test.rs` drive, so the gate cannot rot while no
/// two-hop-safe binding scheme exists.
///
/// Gated on `test-seams` for the reason that feature exists (see
/// `Cargo.toml`): a release binary linking this library must carry no
/// one-call way around a refusal. The BODY is not gated — it is the same
/// private function every copy-path plan goes through — so the compiler
/// still sees every helper it calls as live in a bare build.
#[cfg(feature = "test-seams")]
pub fn build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
    verb: Ros2NativeVerb,
    args: &[String],
    lib_dir: &Path,
    ament_prefix: &Path,
    host: HostAbi,
) -> CliResult<Ros2Plan> {
    build_plan_with_adopt_gate(verb, args, lib_dir, ament_prefix, host)
}

/// POSIX-shell-quote one interpolated path for the direct-launch recipe.
///
/// The recipe is printed to be pasted, so a lib dir
/// or home containing a space SPLITS into two words, and one containing a
/// metacharacter can do worse than split. Quoting is what makes "runnable"
/// true on exactly the paths that need it most — a macOS
/// `~/Library/Application Support/...`, a Windows-style share mounted with
/// spaces, a user directory with an apostrophe.
///
/// Single quotes, because inside them every byte is literal in POSIX sh;
/// an embedded `'` closes, escapes and reopens (`'\''`). Shell-safe paths
/// are left bare so the common case stays readable. The `${VAR:+:$VAR}`
/// suffixes are deliberately NOT quoted by this — they must still expand,
/// and adjacent quoted and unquoted parts concatenate into one word.
fn shell_quote(value: &str) -> String {
    let safe = !value.is_empty()
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"_-./:=@+,".contains(&b));
    if safe {
        value.to_string()
    } else {
        format!("'{}'", value.replace('\'', "'\\''"))
    }
}

/// The launcher-refusal rule on `--adopt-take`'s two-hop gap: these
/// two verbs `exec()` the `ros2` PYTHON CLI, which spawns the node as a
/// FURTHER subprocess with `close_fds=True` — measured in a ROS 2
/// Jazzy container, NOT reproduced by any test in this tree: a
/// descriptor whose `FD_CLOEXEC` was cleared reads `EBADF` in the
/// grandchild, and survives only under `close_fds=False`. The tests here
/// need no ROS 2 install, so re-measuring it takes a ROS 2
/// container. The descriptor binding hands the validated
/// hook over as `/proc/self/fd/<N>` and NEVER by name (that is what closes
/// the substitution window, and it stands), so the node's loader finds a
/// descriptor that is closed in its own table, `ld.so` drops the entry, the
/// hook does not load, and every take is served by COPY — while this
/// launcher has already reported success.
///
/// So the flag is refused here, for both verbs, on EVERY host, until a
/// two-hop-safe binding scheme exists. Host-independent deliberately: the
/// platform gate ([`HostAbi::supports_adopt_take`]) answers "can adoption
/// run on this machine at all", which is a weaker claim than "this verb
/// cannot deliver the hook to the node it launches" — the latter is true
/// on Linux/GNU too, so it must not hide behind a check that passes there.
/// The message carries the Linux/GNU-only caveat instead, so the remedy it
/// names is accurate on a macOS desk as well.
///
/// Why not fall back to naming the hook's path for the `ros2` case: that
/// re-opens exactly the substitution window the descriptor binding closed (the launcher
/// validates bytes it read, and a path is resolved a second time by the
/// child), which descriptor binding exists to forbid.
/// Why the direct-launch recipe cannot be expressed for these paths, or
/// `None` when it can.
///
/// [`shell_quote`] buys shell tokenization and
/// nothing more. The variables the recipe sets are LISTS the loader splits
/// again, by its own rules, after the shell has handed them over intact —
/// glibc's `handle_preload_list` is `strsep(&list, " :")`, so `LD_PRELOAD`
/// breaks on a SPACE as well as a colon, and `LD_LIBRARY_PATH` /
/// `AMENT_PREFIX_PATH` are colon-separated. This module already knows that
/// rule: the staging side splits on `[':', ' ']` for exactly this
/// reason.
///
/// So a hook at `/tmp/lib dir/libcerulion_heaphook.so` yields a recipe that
/// pastes cleanly, starts the node, and silently runs the COPY path —
/// adoption never arms, and the message claimed it would. That is the
/// remedy-that-cannot-work class this whole refusal exists to kill, so the
/// recipe is WITHHELD rather than printed with a caveat.
fn recipe_undeliverable(hook: &Path, lib: &Path, ament: &RecipeAment) -> Option<String> {
    let ament_path: Option<&Path> = match ament {
        RecipeAment::Staged(p) => Some(p),
        // A synthetic placeholder, not a path — always renderable.
        RecipeAment::Unresolvable => None,
    };
    for (var, value, delimiters, why) in [
        (
            "LD_PRELOAD",
            Some(hook),
            [' ', ':'].as_slice(),
            "glibc splits LD_PRELOAD on spaces AND colons (`strsep(&list, \" :\")`)",
        ),
        (
            "LD_LIBRARY_PATH",
            Some(lib),
            [':'].as_slice(),
            "LD_LIBRARY_PATH is a colon-separated list",
        ),
        (
            "AMENT_PREFIX_PATH",
            ament_path,
            [':'].as_slice(),
            "AMENT_PREFIX_PATH is a colon-separated list",
        ),
    ] {
        let Some(path) = value else {
            continue;
        };
        // The recipe is text, and `Display` for a path
        // replaces bytes that are not valid UTF-8 with U+FFFD. Rendering one
        // would hand the reader a command naming a location that does not
        // exist — the same silent-wrong-remedy class as the delimiter case,
        // so it takes the same branch rather than growing a byte-quoting
        // machinery the message could not express anyway.
        let Some(value) = path.to_str() else {
            return Some(format!(
                "the path this verb would put in {var} is not valid UTF-8 (it renders as \
                 `{}`, with the invalid bytes replaced) — a recipe is TEXT, so printing it \
                 would name a location that does not exist. Move the library to a path whose \
                 bytes are valid UTF-8, with no space or colon in it either, and run this \
                 verb again",
                path.display()
            ));
        };
        if let Some(bad) = value.chars().find(|c| delimiters.contains(c)) {
            let named = if bad == ' ' {
                "a space".to_string()
            } else {
                format!("a `{bad}`")
            };
            return Some(format!(
                "`{value}` contains {named}, and {why} — so that path CANNOT be carried in \
                 {var} at all. Quoting does not help: the shell hands the variable through \
                 intact and the LOADER splits its value. The node would start WITHOUT the \
                 hook and every take would be served by COPY — the silent degrade this \
                 refusal exists to prevent — so no recipe is printed. Move the library to a \
                 path with no space or colon in it (build or copy it beside the `cerulion` \
                 binary in a delimiter-free directory, or point {ROS2_PRELOAD_ENV} at one) \
                 and run this verb again"
            ));
        }
    }
    None
}

/// What the recipe would put in `AMENT_PREFIX_PATH`: the prefix the staging
/// would produce, or — when no home directory resolves — a placeholder that
/// is a description rather than a path.
enum RecipeAment<'a> {
    Staged(&'a Path),
    Unresolvable,
}

fn refuse_adopt_take_under_the_ros2_cli(lib_dir: &Path, ament_prefix: Option<&Path>) -> CliError {
    let hook_path = lib_dir.join(HEAPHOOK_FILENAME);
    let ament_kind = match ament_prefix {
        Some(p) => RecipeAment::Staged(p),
        // No home directory resolves, so the prefix has no path to name.
        // Naming the shape beats failing with a staging error: the refusal's
        // answer does not depend on HOME.
        None => RecipeAment::Unresolvable,
    };
    let hook = hook_path.display().to_string();
    let lib = lib_dir.display().to_string();
    let ament = match ament_kind {
        RecipeAment::Staged(p) => p.display().to_string(),
        RecipeAment::Unresolvable => "<a prefix whose lib/ holds librmw_cerulion.so>".to_string(),
    };
    let remedy = match recipe_undeliverable(&hook_path, lib_dir, &ament_kind) {
        Some(why) => format!(
            "Launching the node executable DIRECTLY is normally the way to arm adoption — one \
             hop keeps the preload — but NOT from where these files live: {why}. Until then \
             the copy path is all that is available here, which is what dropping \
             {ADOPT_TAKE_FLAG} asks for explicitly."
        ),
        None => format!(
            "Launch the node executable DIRECTLY instead of through `ros2 run`/`ros2 launch` \
             — one hop keeps the preload, so adoption can arm there: \
             `LD_PRELOAD={hook_q}${{LD_PRELOAD:+:$LD_PRELOAD}} \
             LD_LIBRARY_PATH={lib_q}${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}} \
             AMENT_PREFIX_PATH={ament_q}${{AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}} \
             RMW_IMPLEMENTATION=rmw_cerulion {ADOPT_TAKE_CHILD_ENV}=1 \
             <install>/lib/<pkg>/<executable>` — run it in a shell that has already sourced \
             your ROS 2 setup: each path var is PREPENDED to the sourced value, exactly as \
             this verb stages it, because replacing them would drop the distro's own ament \
             index and libraries. The hook is named by PATH here (a path survives the spawn; \
             a descriptor does not). Build what that names if it is missing: `cargo build -p \
             cerulion_heaphook -p rmw_cerulion`, or point LD_PRELOAD at the hook \
             {ROS2_PRELOAD_ENV} names if it lives elsewhere. The AMENT_PREFIX_PATH entry is \
             the minimal prefix these verbs stage; THIS refusal creates nothing, so if it \
             does not exist yet, run the same verb once WITHOUT {ADOPT_TAKE_FLAG} to create \
             it — or point AMENT_PREFIX_PATH at any prefix whose `lib/` holds \
             {RMW_LIB_FILENAME}.",
            hook_q = shell_quote(&hook),
            lib_q = shell_quote(&lib),
            ament_q = shell_quote(&ament),
        ),
    };
    CliError::Validation(format!(
        "{ADOPT_TAKE_ROS2_CLI_PREFIX}: `ros2` is a Python CLI that spawns the node as a \
         FURTHER subprocess with close_fds=True, so the validated heap hook — handed over as \
         an inherited descriptor (`/proc/self/fd/<N>`), never by name — is already closed when \
         the node's loader reads LD_PRELOAD; ld.so drops the entry, the hook never loads, and \
         every take would be served by COPY behind a flag that promised adoption, after this \
         launcher had reported success. {remedy} Linux/GNU only, as {ADOPT_TAKE_FLAG} always \
         was — the hook interposes glibc's malloc/free. Or drop {ADOPT_TAKE_FLAG} to run \
         these verbs on the copy path."
    ))
}

/// The pass-through plan builder including the `--adopt-take`
/// launch gate (host support → staged-preload check → hook inspection →
/// preload-order scan → descriptor binding).
///
/// Production reaches this only on the COPY path: its one PRODUCTION
/// caller, [`build_ros2_passthrough_plan_for_host`], refuses `--adopt-take` before
/// delegating (the launcher-refusal rule above), so the `if adopt_take` body
/// below is unreachable from either verb. It stays because the rule is
/// "refuse until a two-hop-safe binding scheme exists", not "delete the
/// binding"; the tests reach it through
/// `build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test`.
fn build_plan_with_adopt_gate(
    verb: Ros2NativeVerb,
    args: &[String],
    lib_dir: &Path,
    ament_prefix: &Path,
    host: HostAbi,
) -> CliResult<Ros2Plan> {
    // Cerulion's own leading flag is split off BEFORE anything is
    // forwarded; everything after the first non-Cerulion token stays
    // verbatim (see `split_adopt_take_flag`).
    let (adopt_take, forwarded) = split_adopt_take_flag(args);
    if adopt_take && !host.supports_adopt_take() {
        // The platform gate comes first — before any file is looked at.
        // The hook interposes glibc's
        // malloc/free via LD_PRELOAD, so on any other host an ELF that
        // inspects clean still resolves nothing in the child.
        return Err(CliError::Validation(format!(
            "{ADOPT_TAKE_HOST_UNSUPPORTED_PREFIX}: this host is {} — the Cerulion heap hook \
             interposes glibc's malloc/free through LD_PRELOAD, so adoption can only run on \
             linux/gnu; drop {ADOPT_TAKE_FLAG} to run on the copy path",
            host.describe()
        )));
    }
    let mut env = stage_base_child_env(
        lib_dir,
        ament_prefix,
        if adopt_take {
            AdoptTakeGate::Ran
        } else {
            AdoptTakeGate::NotRun
        },
    )?;
    // Identities captured from the SAME opened objects the adopt gate
    // validated. Empty when the flag is off — there is then no
    // content validation to bind, and the plan falls back to a plain stat
    // of each staged path.
    let mut adopt_identities: Vec<(PathBuf, Option<FileId>)> = Vec::new();
    // The descriptor-bound hook entry, when the adopt gate validated one.
    let mut hook_fd: Option<InspectedFile> = None;
    if adopt_take {
        // The flag DEMANDS the preload: hard-fail when NOTHING landed in
        // the staged LD_PRELOAD — because the hook is not built beside the
        // binary and nothing was named, or because the kill switch
        // suppressed injection. Starting anyway would run the copy path
        // behind a flag that promised adoption (the rmw would warn once
        // and degrade — visibly, but the launcher can refuse BEFORE the
        // robot is running on the wrong path).
        let hook_path = lib_dir.join(HEAPHOOK_FILENAME);
        let staged: Vec<PathBuf> = env
            .iter()
            .find(|(k, _)| k == "LD_PRELOAD")
            .map(|(_, v)| {
                // Split on space as
                // well as colon, because that is what the loader does —
                // glibc's `handle_preload_list` is `strsep(&list, " :")`,
                // and `ld.so(8)` says the items "can be separated by spaces
                // or colons". This guard's whole job is that the entries it
                // inspects are the entries `ld.so` will load; a value like
                // `/tmp/a b.so` parsed as ONE path here but as TWO by the
                // loader is precisely the divergence that lets an
                // un-inspected library win `malloc` ahead of the hook.
                v.split([':', ' '])
                    .filter(|entry| !entry.is_empty())
                    .map(PathBuf::from)
                    .collect()
            })
            .unwrap_or_default();
        if staged.is_empty() {
            return Err(CliError::Validation(format!(
                "{ADOPT_TAKE_HOOK_MISSING_PREFIX}: `{}` is not staged into the child's \
                 LD_PRELOAD — build it (`cargo build -p cerulion_heaphook`) so \
                 {HEAPHOOK_FILENAME} sits beside the `cerulion` binary, and do not \
                 combine {ROS2_PRELOAD_ENV}=off with {ADOPT_TAKE_FLAG} (the flag \
                 requires the preload; drop the flag to run on the copy path)",
                hook_path.display()
            )));
        }
        // A path being in LD_PRELOAD is the STAGING half; some staged FILE
        // must also be the built hook. The
        // hook is whichever staged entry INSPECTS as it — the lib_dir copy,
        // or a valid hook {ROS2_PRELOAD_ENV} named anywhere else (the
        // explicit path is the hook the loader will
        // map; refusing it broke the explicit-preload contract). Entries
        // are inspected in LD_PRELOAD order, first-wins like the loader's
        // own symbol resolution; a user's unrelated `.so` stacked ahead is
        // simply not it. When none is, every staged entry is named with
        // WHY it is not the hook — a path the dynamic loader cannot map is
        // dropped by ld.so with a message nobody reads, and the child would
        // run the copy path behind a flag that promised adoption.
        // Every staged entry is opened
        // once here, and both the hook question and the allocator
        // question are answered from those bytes — with the identity of
        // the object they came from. Reading each file
        // for validation and then `stat`ing the path again to record an
        // identity would let a replacement landing between them become the
        // recorded identity, and the exec-time re-check would confirm the
        // attacker's file.
        let reads: Vec<InspectedRead> = staged
            .iter()
            .map(|entry| (entry.clone(), read_inspected(entry)))
            .collect();
        let mut problems: Vec<String> = Vec::with_capacity(staged.len());
        let accepted = reads.iter().position(|(entry, read)| {
            let verdict = match read {
                Ok((bytes, _, _)) => inspect_heaphook_bytes(bytes, host_elf_machine()),
                Err(problem) => Err(problem.clone()),
            };
            match verdict {
                Ok(()) => true,
                Err(problem) => {
                    problems.push(format!("`{}` {problem}", entry.display()));
                    false
                }
            }
        });
        let Some(hook_index) = accepted else {
            return Err(CliError::Validation(format!(
                "{ADOPT_TAKE_HOOK_MISSING_PREFIX}: no staged preload is the built hook — {} — \
                 rebuild it (`cargo build -p cerulion_heaphook`) so the built \
                 {HEAPHOOK_FILENAME} sits beside the `cerulion` binary (or is the file \
                 {ROS2_PRELOAD_ENV} names), or drop {ADOPT_TAKE_FLAG} to run on the copy \
                 path — a preload the loader cannot map would silently run the copy path \
                 behind a flag that promised adoption",
                problems.join("; ")
            )));
        };
        // A valid hook in the list is necessary, not sufficient:
        // the dynamic loader resolves symbols left to
        // right across the preload list, so an allocator staged ahead of
        // the hook wins `malloc`, the rmw's handshake reads
        // `DegradeForeignAllocator`, and the child runs the copy path behind
        // a flag that promised adoption — the same silent-degrade class the
        // hook inspection closes on other edges. Every entry
        // BEFORE the hook is scanned for a malloc-family export; an entry
        // AFTER it is fine, because the hook already won.
        for (ahead_index, ahead) in staged[..hook_index].iter().enumerate() {
            // The launcher inspects files, but
            // ld.so resolves a bare soname (no `/`) through the library
            // search path and expands `$LIB`/`$ORIGIN`/`$PLATFORM` tokens —
            // the object that actually loads ahead of the hook may not be
            // the one at this path (a same-named file in the launcher's
            // cwd, say). Refuse rather than inspect the wrong file.
            let raw = ahead.to_string_lossy();
            let bare = !ahead.has_root() && ahead.components().count() == 1;
            let dollar = raw.contains('$');
            if bare || dollar {
                // Each reason carries its own why and remedy. A
                // bare soname is SEARCHED for; a `$` may be EXPANDED — and
                // telling someone who wrote `/opt/ros/$LIB/libfoo.so` to
                // "give an absolute path" names a property that entry
                // already has. The `$` arm describes the LAUNCHER's rule
                // rather than ld.so's, deliberately: the guard refuses ANY
                // `$`, a literal one in a legal filename (`libfoo$1.so`)
                // included, so a remedy phrased as "expand the token"
                // would be unfollowable for that user. Refusing the
                // literal case too is the conservative choice — the
                // launcher cannot tell a token from a filename without
                // reimplementing ld.so's expansion.
                let (as_what, why, remedy) = if bare {
                    (
                        "a bare soname",
                        "ld.so resolves it through the library search path",
                        "give an absolute path",
                    )
                } else {
                    (
                        "a path containing `$`",
                        "ld.so expands `$LIB`/`$ORIGIN`/`$PLATFORM` and their `${…}` forms at \
                         load time",
                        "give a path with no `$` in it — the expanded one, if the `$` is a token",
                    )
                };
                return Err(CliError::Validation(format!(
                    "{ADOPT_TAKE_PRELOAD_ORDER_PREFIX}: `{raw}` is staged AHEAD of the hook as \
                     {as_what} — {why}, so the launcher cannot inspect the object that will \
                     actually load ahead of the hook; {remedy}, stage it after the hook, or \
                     drop it (or drop {ADOPT_TAKE_FLAG} to run on the copy path)"
                )));
            }
            // An entry whose exports could
            // not be enumerated is refused, never waved through. The guard
            // answers "does this win malloc ahead of the hook?"; a reader
            // that could not tell must not answer "no".
            let allocator = match &reads[ahead_index].1 {
                // A shape ld.so cannot map is dropped by the loader and
                // cannot win `malloc` — the same split
                // `preload_allocator_exports` documents, over the bytes
                // already read.
                // `TooLarge` is not
                // in this bucket. The ceiling is the launcher's own read budget, not
                // something `ld.so` refuses to map, so a too-large entry is
                // an UNKNOWN — refused below — never a "no allocator".
                Err(HookFileProblem::Absent | HookFileProblem::NotARegularFile) => Ok(Vec::new()),
                Err(problem) => Err(problem.clone()),
                Ok((bytes, _, _)) => allocator_exports_bytes(bytes),
            }
            .map_err(|problem| {
                CliError::Validation(format!(
                    "{ADOPT_TAKE_PRELOAD_ORDER_PREFIX}: `{}` is staged AHEAD of the hook in \
                     LD_PRELOAD and the launcher cannot read its exported symbols — {problem} \
                     — so it cannot rule out that this entry wins `malloc` and degrades the \
                     hook's handshake to the copy path behind a flag that promised adoption; \
                     stage it AFTER the hook (the ambient LD_PRELOAD is stacked after the \
                     hook, {ROS2_PRELOAD_ENV} before it) or drop it, or drop \
                     {ADOPT_TAKE_FLAG} to run on the copy path",
                    ahead.display()
                ))
            })?;
            if !allocator.is_empty() {
                return Err(CliError::Validation(format!(
                    "{ADOPT_TAKE_PRELOAD_ORDER_PREFIX}: `{}` is staged AHEAD of the hook in \
                     LD_PRELOAD and exports {} — the dynamic loader resolves symbols left \
                     to right, so that allocator wins `malloc` and the hook's handshake \
                     degrades to the copy path (DegradeForeignAllocator) behind a flag that \
                     promised adoption; stage it AFTER the hook (the ambient LD_PRELOAD is \
                     stacked after the hook, {ROS2_PRELOAD_ENV} before it) or drop it, or \
                     drop {ADOPT_TAKE_FLAG} to run on the copy path",
                    ahead.display(),
                    allocator.join(", ")
                )));
            }
        }
        adopt_identities = reads
            .iter()
            .map(|(path, read)| (path.clone(), read.as_ref().ok().map(|(_, id, _)| *id)))
            .collect();
        // The validated hook is handed to the
        // child through the SAME DESCRIPTOR that was inspected, not by
        // name. `LD_PRELOAD` names `/proc/self/fd/<N>`, the fd is kept
        // open across `exec` (FD_CLOEXEC cleared in `exec_ros2`), and the
        // child's loader resolves that path to the inherited descriptor —
        // the exact inode this launcher read and validated. There is then
        // no window at all: nothing between here and the child's `dlopen`
        // can substitute a different file, because no name is resolved a
        // second time. Rounds 20 and 21 only NARROWED the window; this
        // closes it.
        //
        // The price, stated plainly: an operator inspecting the child's
        // environment sees `/proc/self/fd/<N>` where the hook's path used
        // to be. `cerulion ros2 --adopt-take` is Linux/GNU-only already
        // (the platform gate above), so `/proc` is always there.
        if let Some((_, Ok((_, identity, file)))) = reads.get(hook_index) {
            let (fd_path, retained) = bind_hook_by_descriptor(file).map_err(|e| {
                CliError::Validation(format!(
                    "{ADOPT_TAKE_HOOK_MISSING_PREFIX}: the validated hook could not be \
                     bound to this launch by descriptor ({e}) — without that binding the \
                     child would resolve the hook by NAME again, which is the window this \
                     launcher closes; re-run, or drop {ADOPT_TAKE_FLAG} to run on the copy \
                     path"
                ))
            })?;
            let mut staged_paths: Vec<String> =
                staged.iter().map(|p| p.display().to_string()).collect();
            staged_paths[hook_index] = fd_path.display().to_string();
            if let Some((_, value)) = env.iter_mut().find(|(k, _)| k == "LD_PRELOAD") {
                *value = staged_paths.join(":");
            }
            hook_fd = Some(InspectedFile {
                path: fd_path,
                identity: Some(*identity),
                fd: Some(retained),
            });
        }
        env.push((ADOPT_TAKE_CHILD_ENV.to_string(), "1".to_string()));
    }
    let mut full_args = Vec::with_capacity(1 + forwarded.len());
    full_args.push(verb.as_str().to_string());
    full_args.extend(forwarded.iter().cloned());
    // TOCTOU guard: record what every file the
    // launcher validated resolved to, so `exec_ros2` can refuse if it is
    // no longer the same object. The sweep covers every path handed to
    // the child that the launcher made a claim about: each staged
    // LD_PRELOAD entry (content-inspected) and the rmw `.so` (whose
    // existence is the claim `stage_base_child_env` makes and whose
    // directory it puts on LD_LIBRARY_PATH).
    // Nothing is omitted. A path that
    // could not be identified is recorded with `None`, which
    // `verify_inspected` requires to still be unidentifiable — so a file
    // that VANISHED cannot be silently dropped from the list, and one
    // that APPEARS between plan and exec is refused rather than loaded
    // unvalidated.
    let mut inspected: Vec<InspectedFile> = Vec::new();
    for (key, value) in &env {
        if key == "LD_PRELOAD" {
            // The same loader-faithful split as the staging
            // side — the exec-time re-verification must enumerate exactly
            // the entries `ld.so` will load, or an entry hidden behind a
            // space is never identity-checked at all.
            for entry in value.split([':', ' ']).filter(|e| !e.is_empty()) {
                let path = PathBuf::from(entry);
                // The descriptor-bound hook carries its own entry: its
                // identity comes from the OPEN fd, and re-`stat`ing
                // `/proc/self/fd/<N>` would be the very name resolution
                // the binding exists to avoid.
                if hook_fd.as_ref().is_some_and(|h| h.path == path) {
                    continue;
                }
                let identity = adopt_identities
                    .iter()
                    .find(|(p, _)| p == &path)
                    .map(|(_, id)| *id)
                    .unwrap_or_else(|| file_identity(&path));
                inspected.push(InspectedFile {
                    path,
                    identity,
                    fd: None,
                });
            }
        }
    }
    inspected.extend(hook_fd);
    let rmw_lib = lib_dir.join(RMW_LIB_FILENAME);
    let rmw_identity = file_identity(&rmw_lib);
    inspected.push(InspectedFile {
        path: rmw_lib,
        identity: rmw_identity,
        fd: None,
    });
    Ok(Ros2Plan {
        program: "ros2".to_string(),
        args: full_args,
        env,
        inspected,
    })
}

/// `(dev, ino, size, mtime_ns)` of what `path` resolves to, or `None` when
/// it cannot be stat'd. This is the identity the
/// launcher pins between inspecting a file and handing its NAME to the
/// child. Follows symlinks, as `ld.so` does — see [`InspectedFile`].
/// Has the caller run the `--adopt-take` launch gate (host support, heap-hook
/// inspection, preload order) for the child it is staging?
///
/// The parameter exists so
/// [`stage_base_child_env`] — the ONE function every Cerulion-spawned ROS 2
/// child passes through — can refuse an ambient `CERULION_RMW_ADOPT_TAKE`
/// on the paths where no gate ran. A `bool` would read as
/// `stage_base_child_env(dir, prefix, false)` at the call site, which says
/// nothing about WHICH claim is false; naming the two states makes the
/// graph path's answer legible where it is written.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdoptTakeGate {
    /// The caller ran the flag's checks and will arm the child itself.
    ///
    /// Production-unreachable under the launcher-refusal rule (see
    /// [`ADOPT_TAKE_FLAG`]): the only constructor is the retained gate, and
    /// the flag that reaches it is refused before delegation, so no production
    /// path arms a child. Retained with the gate it belongs to. Read it as
    /// "a gate ran" when auditing who may arm a child unvalidated — today,
    /// nobody.
    Ran,
    /// No gate ran on this path (a `ros2:` graph entry has no flag to
    /// pass), so an ambient arming value must be refused rather than
    /// inherited.
    NotRun,
}

/// Bind the validated hook to this launch by descriptor,
/// returning the `/proc/self/fd/<N>` path to hand the child and the
/// descriptor the plan RETAINS.
///
/// The clone happens first and the
/// path names the CLONE. Reading `as_raw_fd()` off the
/// INSPECTED file — which lives in the builder's `reads` vector and is closed
/// when that vector drops at the end of the plan builder — while separately
/// retaining a `try_clone()`, which is a `dup` and therefore a DIFFERENT
/// number, is wrong: `exec_ros2` would clear `FD_CLOEXEC` on the clone while
/// `LD_PRELOAD` still named the closed original, so the child's loader would find
/// nothing there (or, worse, whatever an unrelated `open` had since been
/// given that number), drop the preload with a warning nobody reads, and
/// run the copy path — the exact silent degrade behind a flag promising
/// adoption that this whole gate exists to prevent.
///
/// Unix-only: `/proc/self/fd` and
/// `AsRawFd` are Unix concepts, and the non-Unix build must still compile —
/// [`exec_ros2`] has a non-Unix stub that refuses the verb outright, and this
/// mirrors it rather than leaking a Unix-only import into the
/// platform-independent plan builder.
#[cfg(unix)]
fn bind_hook_by_descriptor(
    file: &std::fs::File,
) -> std::io::Result<(PathBuf, std::sync::Arc<std::fs::File>)> {
    use std::os::unix::io::AsRawFd;
    let retained = std::sync::Arc::new(file.try_clone()?);
    let fd = retained.as_raw_fd();
    Ok((PathBuf::from(format!("/proc/self/fd/{fd}")), retained))
}

/// Non-Unix stub (see [`exec_ros2`]): there is no descriptor path to hand a
/// child, and the verb itself is refused on this platform.
#[cfg(not(unix))]
fn bind_hook_by_descriptor(
    _file: &std::fs::File,
) -> std::io::Result<(PathBuf, std::sync::Arc<std::fs::File>)> {
    Err(std::io::Error::new(
        std::io::ErrorKind::Unsupported,
        "descriptor binding needs /proc/self/fd, which is Unix-only",
    ))
}

/// The FULL modification time in nanoseconds — seconds AND the sub-second
/// remainder.
///
/// `mtime_nsec()` ALONE is not enough: on Linux it is only
/// `st_mtim.tv_nsec` — the
/// remainder within the second, range `0..1_000_000_000`. Without the whole-second
/// component, on any filesystem with 1-second mtime
/// granularity it is permanently `0` and the field contributes NOTHING.
/// That field is precisely what the exec-time re-check leans on to catch an
/// in-place, same-size rewrite of the hook (same `dev`, `ino` and `size`,
/// only the time differs) — the attack this identity exists to refuse. An
/// `i128` because the product overflows `i64` shortly after the year 2262,
/// and an identity that wraps is worse than one that is merely coarse.
#[cfg(unix)]
fn mtime_ns(meta: &std::fs::Metadata) -> i128 {
    use std::os::unix::fs::MetadataExt;
    meta.mtime() as i128 * 1_000_000_000 + meta.mtime_nsec() as i128
}

/// `(dev, ino, size, mtime_ns)` for a path. `pub` so the
/// tests assert against the launcher's own definition instead of rebuilding
/// the tuple by hand — five hand-built copies are five chances to carry a
/// different mtime component from the launcher's own.
#[cfg(unix)]
pub fn file_identity(path: &Path) -> Option<FileId> {
    use std::os::unix::fs::MetadataExt;
    let meta = std::fs::metadata(path).ok()?;
    Some((meta.dev(), meta.ino(), meta.size(), mtime_ns(&meta)))
}

/// Non-Unix stub: the verb itself is Unix-only (see [`exec_ros2`]).
#[cfg(not(unix))]
pub fn file_identity(_path: &Path) -> Option<FileId> {
    None
}

/// The identity of an OPEN descriptor (`fstat`), for an entry bound to
/// the launch by fd (descriptor binding). No path is resolved, so
/// this answers about the object the launcher actually read.
#[cfg(unix)]
fn fd_identity(file: &std::fs::File) -> Option<FileId> {
    use std::os::unix::fs::MetadataExt;
    let meta = file.metadata().ok()?;
    Some((meta.dev(), meta.ino(), meta.size(), mtime_ns(&meta)))
}

/// Non-Unix stub (see [`exec_ros2`]).
#[cfg(not(unix))]
fn fd_identity(_file: &std::fs::File) -> Option<FileId> {
    None
}

/// `(dev, ino, size, mtime_ns)` — what a path resolved to when the
/// launcher read it. Named because it travels through the plan and three
/// signatures.
pub type FileId = (u64, u64, u64, i128);

/// One staged preload entry: its path, and either the bytes the launcher
/// validated with the identity of the object they came from, or why it
/// could not be read.
type InspectedRead = (
    PathBuf,
    Result<(Vec<u8>, FileId, std::fs::File), HookFileProblem>,
);

/// Open a file ONCE, identify the OPENED OBJECT, and read it.
///
/// Validating a path by
/// reading it and then `stat`ing the path again to record an identity lets a
/// replacement landing between the two become the recorded identity — the
/// re-check at exec would then confirm the attacker's file rather than the
/// validated one. `metadata()` on an OPEN handle is `fstat(fd)`, which
/// describes the object the bytes came from and nothing else, so the bytes
/// that are validated and the identity that is pinned are the same file by
/// construction.
///
/// The read is BOUNDED: at most the inspect ceiling is read from the file.
///
/// The fstat refuses a file that is over
/// the inspect limit, but `meta.len()` is a SNAPSHOT and these paths come from
/// `LD_PRELOAD` — ambient or `CERULION_ROS2_PRELOAD`, both
/// environment-controlled. A plain `std::fs::read` (or `read_to_end`) chases
/// the moving EOF of a file that GROWS after the stat and allocates without
/// limit, so a launch could be OOM-ed or hung before the node ever starts. The
/// stat check stays — it refuses an already-oversized file without reading a
/// byte; the bounded read is the half that survives the file changing mid-read.
///
/// The read asks for `LIMIT + 1` and treats a full buffer as over-limit: a read that
/// stops AT the limit cannot tell a conforming file from a truncated view of a
/// larger one.
///
/// One open, one fstat, and the
/// bytes read from THAT descriptor — the path is never resolved twice.
///
/// Stat-ing the path and then
/// reading the path is two resolutions of an environment-controlled name, so a
/// replacement between them is what gets read, and a FIFO swapped in blocks
/// the launcher on the second open. This is the one helper `read_inspected`
/// and the two sibling inspectors share.
///
/// `O_NONBLOCK` is load-bearing and not a style choice: a plain `O_RDONLY`
/// open of a writer-less FIFO blocks forever per POSIX, and it happens BEFORE
/// the `is_file()` refusal below, so `cerulion ros2 run --adopt-take` would
/// hang with no output at all. The flag returns immediately for a FIFO, the
/// fstat then sees a non-regular file and refuses it, and on a regular file it
/// has no effect on subsequent reads.
fn open_inspect_bounded(
    path: &Path,
) -> Result<(Vec<u8>, std::fs::Metadata, std::fs::File), HookFileProblem> {
    let mut file = open_nonblocking(path)?;
    // Test seam: lets an arm act in the window between the open and the read
    // — swap the path to another file, or to a FIFO — which is the only way to
    // drive this TOCTOU deterministically instead of racing it. Inert unless a
    // test arms it; one relaxed load per inspection, and an inspection happens
    // a handful of times per launch.
    run_after_open_hook();
    let meta = file
        .metadata()
        .map_err(|e| HookFileProblem::Unreadable(e.to_string()))?;
    if !meta.is_file() {
        return Err(HookFileProblem::NotARegularFile);
    }
    if meta.len() > HEAPHOOK_INSPECT_LIMIT {
        return Err(HookFileProblem::TooLarge(meta.len()));
    }
    let ceiling = HEAPHOOK_INSPECT_LIMIT.saturating_add(1);
    let bytes = read_bounded_from(&mut file, ceiling)
        .map_err(|e| HookFileProblem::Unreadable(e.to_string()))?;
    if bytes.len() as u64 > HEAPHOOK_INSPECT_LIMIT {
        // Grew past the ceiling after the fstat — reported as the ceiling,
        // since any length printed would already be stale.
        return Err(HookFileProblem::TooLarge(ceiling));
    }
    Ok((bytes, meta, file))
}

#[cfg(unix)]
fn open_nonblocking(path: &Path) -> Result<std::fs::File, HookFileProblem> {
    use std::os::unix::fs::OpenOptionsExt;
    std::fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NONBLOCK)
        .open(path)
        .map_err(|e| match e.kind() {
            std::io::ErrorKind::NotFound => HookFileProblem::Absent,
            _ => HookFileProblem::Unreadable(e.to_string()),
        })
}

#[cfg(not(unix))]
fn open_nonblocking(path: &Path) -> Result<std::fs::File, HookFileProblem> {
    std::fs::File::open(path).map_err(|e| match e.kind() {
        std::io::ErrorKind::NotFound => HookFileProblem::Absent,
        _ => HookFileProblem::Unreadable(e.to_string()),
    })
}

static AFTER_OPEN_HOOK_ARMED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);
static AFTER_OPEN_HOOK: std::sync::Mutex<Option<Box<dyn Fn() + Send + Sync>>> =
    std::sync::Mutex::new(None);

fn run_after_open_hook() {
    if !AFTER_OPEN_HOOK_ARMED.load(std::sync::atomic::Ordering::Relaxed) {
        return;
    }
    if let Ok(hook) = AFTER_OPEN_HOOK.lock() {
        if let Some(f) = hook.as_ref() {
            f();
        }
    }
}

/// Arm the after-open seam. The returned guard disarms on drop, so
/// an arm that panics cannot leave it set for the rest of the binary.
#[doc(hidden)]
pub fn arm_after_open_hook_for_test(f: Box<dyn Fn() + Send + Sync>) -> AfterOpenHookGuard {
    *AFTER_OPEN_HOOK.lock().expect("after-open hook") = Some(f);
    AFTER_OPEN_HOOK_ARMED.store(true, std::sync::atomic::Ordering::Relaxed);
    AfterOpenHookGuard(())
}

/// RAII disarm for [`arm_after_open_hook_for_test`].
#[doc(hidden)]
#[must_use = "the seam is armed only while the guard is held"]
pub struct AfterOpenHookGuard(());

impl Drop for AfterOpenHookGuard {
    fn drop(&mut self) {
        AFTER_OPEN_HOOK_ARMED.store(false, std::sync::atomic::Ordering::Relaxed);
        if let Ok(mut hook) = AFTER_OPEN_HOOK.lock() {
            *hook = None;
        }
    }
}

/// The bound itself, over any reader.
///
/// Split out from the reading half so
/// a test can observe what the read PULLS, not just what it returns. Checking
/// the returned length cannot tell this apart from an unbounded `read_to_end`
/// followed by a truncation — which returns exactly the same bytes while
/// allocating the whole growing file, i.e. while doing the precise thing the
/// bound exists to prevent. `take` is what makes the difference observable,
/// and it is only observable through the SOURCE.
fn read_bounded_from<R: std::io::Read>(src: R, ceiling: u64) -> std::io::Result<Vec<u8>> {
    use std::io::Read;
    let mut bytes = Vec::new();
    src.take(ceiling).read_to_end(&mut bytes)?;
    Ok(bytes)
}

/// The reader-side bound, for tests — the seam that makes the number of bytes
/// pulled observable. Same implementation the three inspectors
/// reach through `open_inspect_bounded`, driven over a counting source.
pub fn read_bounded_from_for_test<R: std::io::Read>(
    src: R,
    ceiling: u64,
) -> std::io::Result<Vec<u8>> {
    read_bounded_from(src, ceiling)
}

#[cfg(unix)]
fn read_inspected(path: &Path) -> Result<(Vec<u8>, FileId, std::fs::File), HookFileProblem> {
    use std::os::unix::fs::MetadataExt;
    // The open, the fstat and the read all live in
    // `open_inspect_bounded`, shared with the two sibling inspectors, so no
    // caller resolves the path twice. What stays here is this caller's extra
    // obligation: the recorded IDENTITY must come from the same descriptor the
    // bytes did, so it is taken from that fstat, never from a
    // second `stat` of the name.
    let (bytes, meta, file) = open_inspect_bounded(path)?;
    Ok((
        bytes,
        (meta.dev(), meta.ino(), meta.size(), mtime_ns(&meta)),
        file,
    ))
}

/// Non-Unix stub: the verb itself is Unix-only (see [`exec_ros2`]).
#[cfg(not(unix))]
fn read_inspected(_path: &Path) -> Result<(Vec<u8>, FileId, std::fs::File), HookFileProblem> {
    Err(HookFileProblem::Unreadable(
        "`cerulion ros2` is Unix-only".to_string(),
    ))
}

/// The hook inspection over BYTES already read — so a caller that has
/// opened the file once (see `read_inspected`) does not read it again,
/// which is what let a replacement slip between the two reads.
fn inspect_heaphook_bytes(
    bytes: &[u8],
    host: Option<(u16, &'static str)>,
) -> Result<(), HookFileProblem> {
    HOOK_INSPECTIONS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    if bytes.len() < ELF_MAGIC.len() || &bytes[..ELF_MAGIC.len()] != ELF_MAGIC {
        return Err(HookFileProblem::NotAnElfSharedObject);
    }
    elf64_exports(bytes, HEAPHOOK_REQUIRED_EXPORTS, host)
}

/// The allocator scan over BYTES already read — the sibling of
/// [`inspect_heaphook_bytes`], with the same mappable/enumerable split
/// [`preload_allocator_exports`] documents.
fn allocator_exports_bytes(bytes: &[u8]) -> Result<Vec<&'static str>, HookFileProblem> {
    if bytes.len() < ELF_MAGIC.len() || &bytes[..ELF_MAGIC.len()] != ELF_MAGIC {
        return Ok(Vec::new());
    }
    match elf64_scan_exports(bytes, MALLOC_FAMILY_EXPORTS, host_elf_machine()) {
        Ok((exported, _)) => Ok(MALLOC_FAMILY_EXPORTS
            .iter()
            .zip(&exported)
            .filter(|(_, exported)| **exported)
            .map(|(name, _)| *name)
            .collect()),
        Err(
            HookFileProblem::UnsupportedElfShape(_)
            | HookFileProblem::WrongMachine { .. }
            | HookFileProblem::NoLoadableSegment,
        ) => Ok(Vec::new()),
        Err(problem) => Err(problem),
    }
}

/// TOCTOU guard: re-verify every inspected file
/// still resolves to the object the launcher validated. A file that has
/// been swapped — or has vanished — since inspection is refused rather
/// than handed to `ld.so`, because the validation the launcher printed no
/// longer describes what would load.
fn verify_inspected(plan: &Ros2Plan) -> Result<(), CliError> {
    for file in &plan.inspected {
        // An fd-bound entry is identified through the descriptor
        // — `fstat`, never a re-`open` of the name. That is the point of
        // the binding: the object cannot have changed, because nothing
        // resolves a path again between here and the child.
        let now = match &file.fd {
            Some(handle) => fd_identity(handle),
            None => file_identity(&file.path),
        };
        if now == file.identity {
            continue;
        }
        let what = match (file.identity, now) {
            (Some(_), None) => "it no longer exists",
            (None, Some(_)) => {
                "it did not exist when the launch was validated, so nothing \
                                checked what is there now"
            }
            _ => "different inode, size or mtime",
        };
        return Err(CliError::Validation(format!(
            "{PRELOAD_CHANGED_PREFIX}: `{}` is not the file this launch validated ({what}) — \
             it was replaced between inspection and exec, so what ld.so would map is not \
             what was checked; re-run the command, and if this repeats, something else is \
             rewriting the file while you launch",
            file.path.display()
        )));
    }
    Ok(())
}

/// Split Cerulion's own [`ADOPT_TAKE_FLAG`] off the front of the forwarded
/// args. ONLY leading occurrences are consumed — the moment a
/// non-`--adopt-take` token is seen, the remainder is forwarded verbatim,
/// preserving the pass-through guarantee (a later `--adopt-take` belongs to
/// ros2's argument surface and ros2 rejects it loudly, never silently).
/// Repeats are idempotent.
pub fn split_adopt_take_flag(args: &[String]) -> (bool, Vec<String>) {
    let leading = args
        .iter()
        .take_while(|a| a.as_str() == ADOPT_TAKE_FLAG)
        .count();
    (leading > 0, args[leading..].to_vec())
}

/// How the child's `LD_PRELOAD` is composed — the pure decision behind
/// [`stage_base_child_env`]'s preload arm, split out so the matrix is
/// oracle-testable without an env dance.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PreloadDecision {
    /// No preload is staged (no hook present and nothing asked for, or the
    /// kill switch) — the child's `LD_PRELOAD` is whatever the ambient env
    /// already says, untouched.
    None,
    /// The `.so` paths to prepend, IN ORDER, ahead of the ambient
    /// `LD_PRELOAD` — `.bashrc`-style stacking (by design): an explicit
    /// [`ROS2_PRELOAD_ENV`] value is an ADDITION, never a replacement, and
    /// the USER's `.so` comes FIRST (first-wins is theirs), then the
    /// auto-detected heap hook when present, then the ambient value.
    Inject(Vec<PathBuf>),
}

/// The preload matrix (by design): `env_value` is
/// [`ROS2_PRELOAD_ENV`]'s raw value (`None` = unset), `hook_exists` whether
/// `<lib_dir>/libcerulion_heaphook.so` exists.
///
/// - unset + hook → `Inject([hook])`; unset + no hook → `None`;
/// - `off` / `none` → `None` even when the hook exists (the kill switch
///   suppresses ALL injection);
/// - empty → a loud error (a set-but-empty control is a mistake, never a
///   silent guess);
/// - an explicit value STACKS: `Inject([explicit, hook])` when the hook is
///   present, `Inject([explicit])` otherwise — the user's `.so` first, the
///   hook kept, the ambient kept (the caller checks EVERY path's existence
///   loudly); an explicit value that IS the hook's path is listed once.
pub fn decide_preload(
    env_value: Option<&std::ffi::OsStr>,
    lib_dir: &Path,
    hook_exists: bool,
) -> CliResult<PreloadDecision> {
    let hook = || lib_dir.join(HEAPHOOK_FILENAME);
    match env_value {
        None => Ok(if hook_exists {
            PreloadDecision::Inject(vec![hook()])
        } else {
            PreloadDecision::None
        }),
        Some(raw) if raw.is_empty() => Err(CliError::Validation(format!(
            "{ROS2_PRELOAD_ENV} is set but empty — unset it for the default (auto-inject \
             {HEAPHOOK_FILENAME} when present), set it to `off` to disable preload injection, \
             or point it at the `.so` to preload into the ros2 child"
        ))),
        Some(raw) if raw == "off" || raw == "none" => Ok(PreloadDecision::None),
        // An explicit value that IS the hook's own path is listed ONCE —
        // stacking it ahead of itself would stage the same object twice.
        Some(raw) => Ok(PreloadDecision::Inject(
            if hook_exists && Path::new(raw) != hook() {
                vec![PathBuf::from(raw), hook()]
            } else {
                vec![PathBuf::from(raw)]
            },
        )),
    }
}

/// The env pairs EVERY Cerulion-launched ROS 2 child gets — shared verbatim
/// between `cerulion ros2 run` / `ros2 launch` (which `exec()`) and
/// `cerulion graph run`'s `ros2:` graph entries (which spawn supervised
/// children), so every path stages the identical transport environment by
/// construction.
///
/// - `RMW_IMPLEMENTATION=rmw_cerulion`;
/// - `LD_LIBRARY_PATH` = `lib_dir` prepended to the ambient value;
/// - `AMENT_PREFIX_PATH` = `ament_prefix` prepended to the ambient value;
/// - `LD_PRELOAD` per [`decide_preload`]: the heap hook auto-injected when
///   present (ONE `info!` naming the path), an explicit
///   [`ROS2_PRELOAD_ENV`] value instead when set, nothing under the
///   `off`/`none` kill switch.
///
/// Requires `<lib_dir>/librmw_cerulion.so` to exist — a loud
/// [`EXIT_UNAVAILABLE`] error with remediation otherwise
/// ([`missing_rmw_lib_message`]).
pub fn stage_base_child_env(
    lib_dir: &Path,
    ament_prefix: &Path,
    gate: AdoptTakeGate,
) -> CliResult<Vec<(String, String)>> {
    // The rmw arms adoption from
    // `CERULION_RMW_ADOPT_TAKE` alone, so an ambient arming value in the
    // launcher's environment would arm the child without the host, hook and
    // preload-order checks the flag runs. This refusal does not live in
    // `build_ros2_passthrough_plan_for_host`, which is only ONE of the paths
    // that stages a child: a `ros2:` graph entry reaches this function
    // through `ros2_graph::stage_ros2_child_env`, has no flag to pass, and
    // would otherwise inherit the variable unchecked — a safety gate bypassed by
    // choosing the other launch path. It belongs HERE, in the ONE function
    // every Cerulion-spawned ROS 2 child goes through, so a future launch
    // path cannot miss it by construction. `0` and empty disarm the rmw and
    // pass; with the flag the launcher sets its own `1` after its checks.
    if matches!(gate, AdoptTakeGate::NotRun) {
        if let Some(value) = std::env::var_os(ADOPT_TAKE_CHILD_ENV) {
            if !value.is_empty() && value != "0" {
                return Err(CliError::Validation(format!(
                    "{ADOPT_TAKE_ENV_CONFLICT_PREFIX}: {ADOPT_TAKE_CHILD_ENV}={} is set in the \
                     environment but nothing here validated a heap hook for it, so it is \
                     refused rather than inherited. NO Cerulion launcher arms adoption for a \
                     child any more: {ADOPT_TAKE_FLAG} is itself refused under `ros2 \
                     run`/`ros2 launch` (the `ros2` CLI closes the inherited descriptor the \
                     hook rides), and a `ros2:` graph entry never had a flag to pass. Unset the \
                     variable, or launch the node executable DIRECTLY — outside any Cerulion \
                     launcher — with the hook preloaded and this variable set",
                    value.to_string_lossy()
                )));
            }
        }
    }
    let rmw_lib = lib_dir.join(RMW_LIB_FILENAME);
    if !rmw_lib.exists() {
        return Err(CliError::Validation(missing_rmw_lib_message(
            HostAbi::current().os,
            &rmw_lib,
        )));
    }
    let mut env: Vec<(String, String)> = Vec::new();
    env.push(("RMW_IMPLEMENTATION".to_string(), "rmw_cerulion".to_string()));
    env.push((
        "LD_LIBRARY_PATH".to_string(),
        prepend_path_var(&lib_dir.display().to_string(), "LD_LIBRARY_PATH"),
    ));
    env.push((
        "AMENT_PREFIX_PATH".to_string(),
        prepend_path_var(&ament_prefix.display().to_string(), "AMENT_PREFIX_PATH"),
    ));
    let hook_path = lib_dir.join(HEAPHOOK_FILENAME);
    let hook_exists = hook_path.exists();
    match decide_preload(
        std::env::var_os(ROS2_PRELOAD_ENV).as_deref(),
        lib_dir,
        hook_exists,
    )? {
        PreloadDecision::None => {}
        PreloadDecision::Inject(sos) => {
            // EVERY entry is existence-checked — the explicit one AND the
            // hook. The auto-injected hook is in the list because
            // `hook_exists` said so, but an explicit value can NAME the
            // hook's own path while the file is absent. The old
            // `!= hook_path` exemption then
            // staged a missing preload — and the `--adopt-take` gate, seeing
            // the hook's path in LD_PRELOAD, accepted adoption for a child
            // that would run the copy path. A missing path is the loud
            // 69-class error, never a silent drop, whichever entry it is;
            // the hook's own path gets the hook's build remedy.
            for so in sos.iter() {
                if !so.exists() {
                    let remedy = if *so == hook_path {
                        format!(
                            "it is the Cerulion heap hook's own path and the file is absent — \
                             build it (`cargo build -p cerulion_heaphook`) so {HEAPHOOK_FILENAME} \
                             sits beside the `cerulion` binary, or unset {ROS2_PRELOAD_ENV} for \
                             the default (auto-inject it when present)"
                        )
                    } else {
                        format!(
                            "build/install it, set {ROS2_PRELOAD_ENV}=off to disable preload \
                             injection, or unset it for the default (auto-inject \
                             {HEAPHOOK_FILENAME} when present)"
                        )
                    };
                    return Err(CliError::Validation(format!(
                        "{PRELOAD_MISSING_PREFIX}: `{}` — {remedy}",
                        so.display()
                    )));
                }
            }
            let stacked = sos
                .iter()
                .map(|so| so.display().to_string())
                .collect::<Vec<_>>()
                .join(":");
            let composed = prepend_path_var(&stacked, "LD_PRELOAD");
            // Nothing hidden (by design): whenever the hook is
            // auto-included, ONE info line reports the FULL composed list —
            // the user's explicit .so, the hook, and the ambient value alike.
            if sos.contains(&hook_path) {
                tracing::info!(
                    ld_preload = %composed,
                    "auto-injecting the Cerulion heap hook into the ros2 child's LD_PRELOAD \
                     (full composed list shown; CERULION_ROS2_PRELOAD=off disables)"
                );
            }
            env.push(("LD_PRELOAD".to_string(), composed));
        }
    }
    Ok(env)
}

/// `exec()` the plan — replaces the `cerulion` process image with the real
/// `ros2`, so stdin/stdout/stderr stream untouched and signals + exit code
/// propagate through the kernel with ZERO relay code. Returns ONLY on
/// failure (`ErrorKind::NotFound` = "ros2 not on PATH", classified by the
/// caller via [`classify`] → [`EXIT_ROS2_NOT_FOUND`]).
///
/// Deliberately installs NO signal handler: after `exec()` the real `ros2`
/// owns the process group and SIGINT — a handler installed here would be both
/// discarded by `exec()` and a transparency violation.
#[cfg(unix)]
pub fn exec_ros2(plan: &Ros2Plan) -> CliError {
    use std::os::unix::process::CommandExt;
    // TOCTOU guard: the last thing before the
    // image is replaced. See `Ros2Plan::inspected` for what this does and
    // does not close.
    if let Err(e) = verify_inspected(plan) {
        return e;
    }
    // The child must inherit the descriptors
    // whose paths it is being handed, so their close-on-exec bit comes
    // off — last, after verification, and only for entries the launcher
    // itself opened and validated.
    for file in &plan.inspected {
        let Some(handle) = &file.fd else {
            continue;
        };
        let fd = std::os::unix::io::AsRawFd::as_raw_fd(handle.as_ref());
        // SAFETY: `fd` is owned by `handle`, which outlives this call.
        let flags = unsafe { libc::fcntl(fd, libc::F_GETFD) };
        if flags < 0 || unsafe { libc::fcntl(fd, libc::F_SETFD, flags & !libc::FD_CLOEXEC) } < 0 {
            return CliError::Validation(format!(
                "{PRELOAD_CHANGED_PREFIX}: the validated hook's descriptor could not be kept \
                 open across exec ({}) — the child would then resolve `{}` by name, which is \
                 the substitution window this launcher closes; re-run, or drop \
                 {ADOPT_TAKE_FLAG} to run on the copy path",
                std::io::Error::last_os_error(),
                file.path.display()
            ));
        }
    }
    let mut cmd = std::process::Command::new(&plan.program);
    cmd.args(&plan.args);
    for (key, value) in &plan.env {
        cmd.env(key, value);
    }
    cmd.exec().into()
}

/// Non-Unix stub (mirrors the `bagd` platform stub): the verb depends on the
/// Unix dynamic loader and `exec()` semantics.
#[cfg(not(unix))]
pub fn exec_ros2(_plan: &Ros2Plan) -> CliError {
    CliError::Validation(
        "`cerulion ros2` is only supported on Unix platforms (LD_PRELOAD + rmw_cerulion \
         require the Unix dynamic loader and exec() semantics)"
            .to_string(),
    )
}

/// The refusal for a missing `librmw_cerulion.so`, keyed on the HOST OS so
/// the remedy is one the user can take. The Linux packages (the Debian
/// package, the installer archive, Homebrew on Linux) install the library
/// beside the `cerulion` binary, so a Linux user reinstalls a package and is
/// never sent to cargo. ROS 2 Jazzy publishes no macOS binaries, so a macOS
/// user is told in one sentence that the verbs need a Linux machine. Any
/// other host takes the Linux wording: it is the route that exists.
///
/// `host_os` is `std::env::consts::OS` in production (through
/// [`HostAbi::current`]) and a literal in tests, so both branches are pinned
/// on every host. Both keep `LIB_MISSING_PREFIX` (a private constant), which is what
/// [`classify`] maps to [`EXIT_UNAVAILABLE`].
pub fn missing_rmw_lib_message(host_os: &str, rmw_lib: &Path) -> String {
    if host_os == "macos" {
        format!(
            "{LIB_MISSING_PREFIX}: {}: ROS 2 Jazzy has no macOS binaries, so `cerulion ros2 run` \
             and `cerulion ros2 launch` need a Linux machine, where the Cerulion Linux packages \
             install {RMW_LIB_FILENAME} beside the `cerulion` binary",
            rmw_lib.display()
        )
    } else {
        format!(
            "{LIB_MISSING_PREFIX}: {}: the Cerulion Linux packages install {RMW_LIB_FILENAME} \
             beside the `cerulion` binary, so reinstall Cerulion (the Debian package, the \
             installer archive or Homebrew on Linux) or set {LIB_DIR_ENV} to the directory \
             that contains it",
            rmw_lib.display()
        )
    }
}

/// Map an error from [`resolve_lib_dir`] / [`stage_ament_prefix`] /
/// [`build_ros2_passthrough_plan`] / [`exec_ros2`] to the verb's exit
/// contract. Local to this verb — the mapping is verb-specific
/// (`replay_cmd` owns its own `EXIT_*` consts; this follows that precedent
/// rather than touching the shared [`CliError`]).
pub fn classify(err: &CliError) -> u8 {
    match err {
        // The only Io error the dispatch classifies post-plan is exec()'s:
        // NotFound = the real `ros2` is not on PATH. (Staging failures are
        // wrapped into `Validation` precisely so an unrelated NotFound can
        // never land here.)
        CliError::Io(e) if e.kind() == std::io::ErrorKind::NotFound => EXIT_ROS2_NOT_FOUND,
        CliError::Validation(msg)
            if msg.starts_with(LIB_MISSING_PREFIX)
                || msg.starts_with(PRELOAD_MISSING_PREFIX)
                || msg.starts_with(ADOPT_TAKE_HOOK_MISSING_PREFIX)
                || msg.starts_with(ADOPT_TAKE_HOST_UNSUPPORTED_PREFIX)
                || msg.starts_with(ADOPT_TAKE_PRELOAD_ORDER_PREFIX)
                || msg.starts_with(ADOPT_TAKE_ROS2_CLI_PREFIX)
                || msg.starts_with(PRELOAD_CHANGED_PREFIX) =>
        {
            EXIT_UNAVAILABLE
        }
        CliError::Validation(msg) if msg.starts_with(ADOPT_TAKE_ENV_CONFLICT_PREFIX) => EXIT_USAGE,
        _ => EXIT_OTHER,
    }
}

/// Prepend `new_entry` to the ambient value of `var_name` with the Unix `:`
/// separator; a missing/empty ambient value yields `new_entry` alone (no
/// trailing separator).
fn prepend_path_var(new_entry: &str, var_name: &str) -> String {
    match std::env::var(var_name) {
        Ok(existing) if !existing.is_empty() => format!("{new_entry}:{existing}"),
        _ => new_entry.to_string(),
    }
}

/// FNV-1a 64 over raw bytes — keys the per-lib-dir staging prefix so two
/// Cerulion builds never share (and race on) one symlink.
fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        hash ^= u64::from(b);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}
