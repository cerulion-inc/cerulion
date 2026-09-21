// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion ros2 run` / `ros2 launch` engine oracles: the pass-through plan
//! builder (`build_ros2_passthrough_plan` / `stage_base_child_env`), the
//! heap-hook preload matrix (`decide_preload` + env composition), the
//! ament-prefix staging, the exit-code classification, and a spawn-based
//! env-application check against a fixture script.
//!
//! Tests that READ or WRITE the ambient env (`LD_LIBRARY_PATH`,
//! `AMENT_PREFIX_PATH`, `LD_PRELOAD`, `CERULION_ROS2_PRELOAD`,
//! `CERULION_LIB_DIR`) are `#[serial]` — the env is process-global. The
//! pure `decide_preload` matrix, staging and classification tests touch no
//! env and stay parallel.
//!
//! The REAL-binary exec-transparency e2e (exit-code inheritance through
//! `exec()` on BOTH verbs, verbatim forwarding of hyphenated tokens, the
//! {69, 127} contract over the shipped `cerulion`) lives in
//! `cerulion_cli/tests/ros2_run_e2e_test.rs`.

use std::path::{Path, PathBuf};

use cerulion_cli_engine::error::CliError;
use cerulion_cli_engine::ros2_cmd::{
    arm_after_open_hook_for_test, build_ros2_passthrough_plan,
    build_ros2_passthrough_plan_for_host,
    build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test, classify, decide_preload,
    file_identity, hook_inspections_for_test, host_elf_machine, inspect_heaphook_file,
    inspect_heaphook_file_for_machine, missing_rmw_lib_message, preload_allocator_exports,
    read_bounded_from_for_test, resolve_lib_dir, split_adopt_take_flag, stage_ament_prefix_under,
    stage_base_child_env, AdoptTakeGate, HookFileProblem, HostAbi, PreloadDecision, Ros2NativeVerb,
    ADOPT_TAKE_CHILD_ENV, ADOPT_TAKE_FLAG, EXIT_OTHER, EXIT_ROS2_NOT_FOUND, EXIT_UNAVAILABLE,
    EXIT_USAGE, HEAPHOOK_FILENAME, HEAPHOOK_HANDSHAKE_SYMBOL, HEAPHOOK_INSPECT_LIMIT,
    HEAPHOOK_REQUIRED_EXPORTS, LIB_DIR_ENV, MALLOC_FAMILY_EXPORTS, RMW_LIB_FILENAME,
    ROS2_PRELOAD_ENV,
};
use serial_test::serial;
use std::os::unix::fs::FileTypeExt;

// The built-artifact arm is Linux-only (the hook is an ELF `.so`), so its
// seam is imported there and nowhere else — an unconditional `use` is an
// unused import on every other host, which this crate denies.
#[cfg(target_os = "linux")]
use cerulion_cli_engine::ros2_cmd::scan_exports_for_test;

/// The distinctive TAIL of `PRELOAD_MISSING_PREFIX`, the engine's generic
/// missing-preload refusal (the const is private to the engine because
/// `classify` is its only other reader). Arms that must prove a LATER
/// guard fired assert this is ABSENT — and an absence assertion against a
/// literal goes vacuous if the production string is reworded, so every use
/// of it is PAIRED, in the same test body, with a POSITIVE assertion over
/// a refusal that really is the missing-preload one: a rewording fails
/// that half loudly instead of quietly excusing the other.
const PRELOAD_MISSING_SUBSTR: &str = "names a missing library";

/// The sentence only the NOTHING-STAGED `--adopt-take` refusal emits, and
/// the one only the LATER "some staged file must inspect as the hook"
/// refusal emits. The two share `ADOPT_TAKE_HOOK_MISSING_PREFIX` and one
/// exit code, so an arm that names neither cannot say WHICH answered — and
/// the NOTHING-STAGED guard was then individually deletable with the file
/// green, because its owner arms' assertions are all satisfied by the
/// later refusal answering with an empty problem list. (The later guard is
/// not deletable: its own arm's `expect_err` fails outright. These pins
/// say which answered, in both directions.) Each const is asserted
/// POSITIVELY in the same body as its negative uses wherever the arm can
/// produce both, so a rewording fails loudly instead of quietly excusing
/// the negatives.
const NOTHING_STAGED_SUBSTR: &str = "is not staged into the child's LD_PRELOAD";
const NOT_THE_HOOK_SUBSTR: &str = "no staged preload is the built hook";

/// The clause both branches of the preload-ORDER guard carry, and nothing
/// else does — the positive/negative pair that tells an order refusal
/// apart from the generic existence check.
const ORDER_GUARD_SUBSTR: &str = "staged AHEAD of the hook";

/// Panic-safe env override: restores (or removes) the prior value on drop.
struct EnvVarGuard {
    key: &'static str,
    prior: Option<std::ffi::OsString>,
}

impl EnvVarGuard {
    fn set(key: &'static str, value: &str) -> Self {
        let prior = std::env::var_os(key);
        std::env::set_var(key, value);
        Self { key, prior }
    }

    fn unset(key: &'static str) -> Self {
        let prior = std::env::var_os(key);
        std::env::remove_var(key);
        Self { key, prior }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        match &self.prior {
            Some(v) => std::env::set_var(self.key, v),
            None => std::env::remove_var(self.key),
        }
    }
}

// ── A hand-assembled ELF64 shared object (the inspector's input shape) ─────
//
// `elf.h` values the generator writes; the inspector's own copies are
// private, so the fixture carries them independently (a drift on either side
// fails the arms, which is the point).
const FX_ELFCLASS64: u8 = 2;
const FX_ELFDATA2LSB: u8 = 1;
const FX_ET_DYN: u16 = 3;
const FX_SHT_PROGBITS: u32 = 1;
const FX_SHT_STRTAB: u32 = 3;
const FX_SHT_DYNSYM: u32 = 11;
const FX_SHDR_SIZE: usize = 64;
const FX_PT_DYNAMIC: u32 = 2;
const FX_DT_NULL: u64 = 0;
const FX_DT_HASH: u64 = 4;
const FX_DT_GNU_HASH: u64 = 0x6fff_fef5;
const FX_DT_STRTAB: u64 = 5;
const FX_DT_SYMTAB: u64 = 6;
const FX_DT_STRSZ: u64 = 10;
const FX_DT_SYMENT: u64 = 11;
const FX_SHN_UNDEF: u16 = 0;
const FX_STB_LOCAL: u8 = 0;
const FX_STB_GLOBAL: u8 = 1;
const FX_STB_WEAK: u8 = 2;
const FX_STT_NOTYPE: u8 = 0;
const FX_STT_OBJECT: u8 = 1;
const FX_STT_FUNC: u8 = 2;
/// `STT_GNU_IFUNC` — an indirect function, resolved at load time.
const FX_STT_GNU_IFUNC: u8 = 10;

/// The `e_machine` this test binary's host MUST map to — hand-written from
/// `elf.h` (`EM_X86_64` 62, `EM_AARCH64` 183, `EM_RISCV` 243, `EM_PPC64`
/// 21, `EM_S390` 22, `EM_LOONGARCH` 258), keyed on `cfg!(target_arch)`,
/// NEVER via the engine's `host_elf_machine()` (a fixture that stamps
/// what the inspector consults lets a wrong
/// row in that mapping agree with itself on both sides). This table is
/// the oracle; the engine's mapping is what gets checked against it.
fn oracle_host_machine() -> Option<(u16, &'static str)> {
    if cfg!(target_arch = "x86_64") {
        Some((62, "x86-64"))
    } else if cfg!(target_arch = "aarch64") {
        Some((183, "AArch64"))
    } else if cfg!(target_arch = "riscv64") {
        Some((243, "RISC-V"))
    } else if cfg!(target_arch = "powerpc64") {
        Some((21, "PowerPC64"))
    } else if cfg!(target_arch = "s390x") {
        Some((22, "S/390"))
    } else if cfg!(target_arch = "loongarch64") {
        Some((258, "LoongArch"))
    } else {
        // An unmapped target is a SKIP for the
        // fixture arms (`skip_unless_host_mapped!`), never a panic before
        // the inspector runs — and the production mapping returning `None`
        // for it is what `an_unmapped_host_is_refused_before_any_elf_parse`
        // pins through the seam.
        None
    }
}

/// Every fixture-dependent arm opens with this: on a target the oracle has
/// no row for, the arm SKIPS loudly by name instead of panicking inside the
/// generator. The unsupported-host refusal itself is pinned separately.
macro_rules! skip_unless_host_mapped {
    ($arm:expr) => {
        if oracle_host_machine().is_none() {
            eprintln!(
                "SKIP {}: no elf.h oracle row for target_arch {} — the fixture arms need one; \
                 the unsupported-host refusal is pinned by \
                 `an_unmapped_host_is_refused_before_any_elf_parse`",
                $arm,
                std::env::consts::ARCH
            );
            return;
        }
    };
}

/// The OTHER of the two machines the launcher's hosts run on, as a literal
/// from the same `elf.h` table: an x86-64 host gets an AArch64 stamp and
/// every other host an x86-64 one.
fn oracle_other_machine() -> Option<(u16, &'static str)> {
    oracle_host_machine().map(|(host, _)| {
        if host == 62 {
            (183, "AArch64")
        } else {
            (62, "x86-64")
        }
    })
}

/// One `.dynsym` entry the generator emits.
struct FixtureSym {
    name: &'static str,
    bind: u8,
    kind: u8,
    /// `true` ⇒ `st_shndx` = the `.text` section (a DEFINED export);
    /// `false` ⇒ `SHN_UNDEF` (an import).
    defined: bool,
}

fn put_u16(out: &mut Vec<u8>, v: u16) {
    out.extend_from_slice(&v.to_le_bytes());
}
fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}
fn put_u64(out: &mut Vec<u8>, v: u64) {
    out.extend_from_slice(&v.to_le_bytes());
}

/// A REAL minimal ELF64 little-endian shared object, hand-assembled, with
/// the `.dynsym`/`.dynstr` pair the inspector parses — the same structures
/// `ld.so`/`dlsym` walk. No program headers (the inspector never needs
/// them; the arms are about the symbol table). Layout, every offset fixed
/// by construction:
///
/// ```text
/// 0x00  ELF header, 64 B: magic, ELFCLASS64, ELFDATA2LSB, e_type=ET_DYN,
///       e_phoff=0x40 e_phentsize=56 e_phnum=1,
///       e_shoff → the section table, e_shentsize=64, e_shnum=6, e_shstrndx=5
/// 0x40  ONE program header, 56 B: PT_LOAD, R+X, p_offset=0, p_filesz =
///       p_memsz = everything up to the section table (so the bodies — the
///       dynamic tables included — are what the loader maps)
/// 0x78  [1] .text     SHT_PROGBITS  4 bytes — what a DEFINED symbol points at
///       [2] .rodata   SHT_PROGBITS  caller bytes — where the "a string is not
///                                   a symbol" arm plants the name
///       [3] .dynstr   SHT_STRTAB    "\0" + each symbol name + "\0"
///       [4] .dynsym   SHT_DYNSYM    link=3, entsize=24: the null symbol, then
///                                   one Elf64_Sym per `syms` entry
///                                   (st_name → .dynstr, st_info = bind<<4 | type,
///                                   st_shndx = 1 when defined, SHN_UNDEF otherwise)
///       [5] .shstrtab SHT_STRTAB    section names
///       section header table, 6 × 64 B (index 0 is the null header)
/// ```
/// Which hash table the fixture carries. `.dynsym` has no length in the
/// dynamic array, so the loader takes its count from one of these — and
/// modern toolchains default to `--hash-style=gnu`, which means the GNU
/// walker is the path a REAL packaged allocator exercises.
#[derive(Clone, Copy, PartialEq, Eq)]
enum FxHashStyle {
    Sysv,
    Gnu,
}

fn minimal_elf64_so(syms: &[FixtureSym], rodata: &[u8]) -> Vec<u8> {
    minimal_elf64_so_with_hash(syms, rodata, FxHashStyle::Sysv)
}

fn minimal_elf64_so_with_hash(syms: &[FixtureSym], rodata: &[u8], style: FxHashStyle) -> Vec<u8> {
    // Bodies.
    let text = [0xc3u8, 0, 0, 0]; // `ret` + padding
    let mut dynstr = vec![0u8];
    let mut name_offsets = Vec::with_capacity(syms.len());
    for sym in syms {
        name_offsets.push(dynstr.len() as u32);
        dynstr.extend_from_slice(sym.name.as_bytes());
        dynstr.push(0);
    }
    let mut dynsym = vec![0u8; 24]; // the mandatory null symbol
    for (sym, name_off) in syms.iter().zip(name_offsets) {
        put_u32(&mut dynsym, name_off);
        dynsym.push((sym.bind << 4) | (sym.kind & 0xf));
        dynsym.push(0); // st_other
        put_u16(&mut dynsym, if sym.defined { 1 } else { FX_SHN_UNDEF });
        put_u64(&mut dynsym, 0x78); // st_value: the .text body
        put_u64(&mut dynsym, 4); // st_size
    }
    let shstrtab = b"\0.text\0.rodata\0.dynstr\0.dynsym\0.shstrtab\0".to_vec();
    let sh_name = |section: &str| -> u32 {
        let needle = format!("{section}\0");
        shstrtab
            .windows(needle.len())
            .position(|w| w == needle.as_bytes())
            .expect("section name present") as u32
    };

    // A DT_HASH table, because `.dynsym` carries no length in
    // the dynamic array and the loader (and the inspector) takes the
    // count from `nchain`. One bucket is enough — nothing here hashes.
    let nsyms = (syms.len() + 1) as u32; // the mandatory null symbol first
    let mut hash: Vec<u8> = Vec::new();
    match style {
        FxHashStyle::Sysv => {
            put_u32(&mut hash, 1); // nbucket
            put_u32(&mut hash, nsyms); // nchain == the symbol count
            put_u32(&mut hash, 0); // bucket[0]
            for _ in 0..nsyms {
                put_u32(&mut hash, 0); // chain[i]
            }
        }
        FxHashStyle::Gnu => {
            // One bucket holding every hashed symbol; the count is
            // recovered by walking that bucket's chain to its terminator
            // (low bit set), which is the only way to learn it.
            put_u32(&mut hash, 1); // nbuckets
            put_u32(&mut hash, 1); // symoffset: symbol 0 is the null entry
            put_u32(&mut hash, 1); // bloom_size (maskwords)
            put_u32(&mut hash, 0); // bloom_shift
            put_u64(&mut hash, u64::MAX); // bloom[0]: matches everything
            put_u32(&mut hash, 1); // buckets[0] = the first hashed symbol
            for i in 1..nsyms {
                // chain entries are hash values; the LAST carries the
                // terminator bit.
                put_u32(&mut hash, if i == nsyms - 1 { 1 } else { 0 });
            }
        }
    }

    // Offsets: bodies laid out back to back after the header, then the table.
    // vaddr == file offset throughout (one PT_LOAD at vaddr 0), so the
    // dynamic array's virtual addresses are the fixture's own offsets.
    let phdr_off = 0x40u64;
    let text_off = phdr_off + 2 * 56; // PT_LOAD + PT_DYNAMIC
    let rodata_off = text_off + text.len() as u64;
    let dynstr_off = rodata_off + rodata.len() as u64;
    let dynsym_off = dynstr_off + dynstr.len() as u64;
    let hash_off = dynsym_off + dynsym.len() as u64;
    let dynamic_off = (hash_off + hash.len() as u64 + 7) & !7;
    let mut dynamic: Vec<u8> = Vec::new();
    for (tag, val) in [
        (
            match style {
                FxHashStyle::Sysv => FX_DT_HASH,
                FxHashStyle::Gnu => FX_DT_GNU_HASH,
            },
            hash_off,
        ),
        (FX_DT_STRTAB, dynstr_off),
        (FX_DT_SYMTAB, dynsym_off),
        (FX_DT_STRSZ, dynstr.len() as u64),
        (FX_DT_SYMENT, 24u64),
        (FX_DT_NULL, 0),
    ] {
        put_u64(&mut dynamic, tag);
        put_u64(&mut dynamic, val);
    }
    let shstrtab_off = dynamic_off + dynamic.len() as u64;
    let shoff = shstrtab_off + shstrtab.len() as u64;
    let shoff = (shoff + 7) & !7; // 8-align the table (cosmetic; real linkers do)

    let mut out = Vec::new();
    // ELF header.
    out.extend_from_slice(b"\x7fELF");
    out.push(FX_ELFCLASS64);
    out.push(FX_ELFDATA2LSB);
    out.push(1); // EI_VERSION
    out.push(0); // EI_OSABI = SYSV
    out.extend_from_slice(&[0u8; 8]); // EI_ABIVERSION + padding
    put_u16(&mut out, FX_ET_DYN);
    // e_machine: THIS host's, from the independent elf.h oracle table — the
    // inspector refuses any other, and the stamp must not come
    // from the mapping under test.
    put_u16(
        &mut out,
        oracle_host_machine()
            .expect("fixture arms skip on an unmapped host before building one")
            .0,
    );
    put_u32(&mut out, 1); // e_version
    put_u64(&mut out, 0); // e_entry
    put_u64(&mut out, phdr_off); // e_phoff: PT_LOAD + PT_DYNAMIC after the header
    put_u64(&mut out, shoff);
    put_u32(&mut out, 0); // e_flags
    put_u16(&mut out, 64); // e_ehsize
    put_u16(&mut out, 56); // e_phentsize
    put_u16(&mut out, 2); // e_phnum
    put_u16(&mut out, FX_SHDR_SIZE as u16); // e_shentsize
    put_u16(&mut out, 6); // e_shnum
    put_u16(&mut out, 5); // e_shstrndx
    assert_eq!(out.len(), 0x40);
    // The one PT_LOAD: file range [0, shoff) mapped at vaddr 0, R+X.
    put_u32(&mut out, 1); // p_type = PT_LOAD
    put_u32(&mut out, 5); // p_flags = R | X
    put_u64(&mut out, 0); // p_offset
    put_u64(&mut out, 0); // p_vaddr
    put_u64(&mut out, 0); // p_paddr
    put_u64(&mut out, shoff); // p_filesz
    put_u64(&mut out, shoff); // p_memsz
    put_u64(&mut out, 0x1000); // p_align
                               // PT_DYNAMIC: what the loader — and the inspector —
                               // follows to the symbol and string tables. A stripped object keeps
                               // this and loses the section headers, which is the whole point.
    put_u32(&mut out, FX_PT_DYNAMIC);
    put_u32(&mut out, 6); // p_flags = R | W
    put_u64(&mut out, dynamic_off); // p_offset
    put_u64(&mut out, dynamic_off); // p_vaddr (identity mapping)
    put_u64(&mut out, dynamic_off); // p_paddr
    put_u64(&mut out, dynamic.len() as u64); // p_filesz
    put_u64(&mut out, dynamic.len() as u64); // p_memsz
    put_u64(&mut out, 8); // p_align
    assert_eq!(out.len() as u64, text_off);
    // Bodies.
    out.extend_from_slice(&text);
    out.extend_from_slice(rodata);
    out.extend_from_slice(&dynstr);
    out.extend_from_slice(&dynsym);
    out.extend_from_slice(&hash);
    out.resize(dynamic_off as usize, 0); // the 8-alignment gap
    out.extend_from_slice(&dynamic);
    out.extend_from_slice(&shstrtab);
    out.resize(shoff as usize, 0);
    // Section headers: (name, type, offset, size, link, entsize).
    let mut shdr = |name: u32, kind: u32, off: u64, size: u64, link: u32, entsize: u64| {
        put_u32(&mut out, name);
        put_u32(&mut out, kind);
        put_u64(&mut out, 0); // sh_flags
        put_u64(&mut out, 0); // sh_addr
        put_u64(&mut out, off);
        put_u64(&mut out, size);
        put_u32(&mut out, link);
        put_u32(&mut out, 0); // sh_info
        put_u64(&mut out, 1); // sh_addralign
        put_u64(&mut out, entsize);
    };
    shdr(0, 0, 0, 0, 0, 0); // [0] null
    shdr(
        sh_name(".text"),
        FX_SHT_PROGBITS,
        text_off,
        text.len() as u64,
        0,
        0,
    );
    shdr(
        sh_name(".rodata"),
        FX_SHT_PROGBITS,
        rodata_off,
        rodata.len() as u64,
        0,
        0,
    );
    shdr(
        sh_name(".dynstr"),
        FX_SHT_STRTAB,
        dynstr_off,
        dynstr.len() as u64,
        0,
        0,
    );
    shdr(
        sh_name(".dynsym"),
        FX_SHT_DYNSYM,
        dynsym_off,
        dynsym.len() as u64,
        3,
        24,
    );
    shdr(
        sh_name(".shstrtab"),
        FX_SHT_STRTAB,
        shstrtab_off,
        shstrtab.len() as u64,
        0,
        0,
    );
    out
}

/// A defined global FUNC export of `name` — the shape every real hook
/// entry point has.
fn exported(name: &'static str) -> FixtureSym {
    FixtureSym {
        name,
        bind: FX_STB_GLOBAL,
        kind: FX_STT_FUNC,
        defined: true,
    }
}

/// The faithful stand-in for the BUILT hook: a real ELF64 shared object for
/// this host EXPORTING every entry point the rmw resolves
/// (`HEAPHOOK_REQUIRED_EXPORTS`, all defined global functions), plus one
/// export the rmw does not look up, the way the built hook does.
fn hook_fixture_bytes() -> Vec<u8> {
    let mut syms: Vec<FixtureSym> = HEAPHOOK_REQUIRED_EXPORTS
        .iter()
        .copied()
        .map(exported)
        .collect();
    syms.push(exported("cerulion_heaphook_abi"));
    minimal_elf64_so(&syms, b"")
}

/// `e_shoff` read out of a generated fixture. Derived rather than
/// hard-coded so a generator that grows a section cannot silently move an
/// offset a comment goes on describing.
fn shdr_table_offset(bytes: &[u8]) -> usize {
    u64::from_le_bytes(
        bytes
            .get(0x28..0x30)
            .expect("an ELF64 header carries e_shoff")
            .try_into()
            .expect("e_shoff is 8 bytes"),
    ) as usize
}

/// STRIP the fixture: drop the section header table and the header fields
/// that describe it, exactly as `strip(1)` leaves a shared object — the
/// shape a packaged allocator ships in, and the one that made the
/// section-header reader answer "no dynamic symbol table".
fn strip_section_headers(bytes: &[u8]) -> Vec<u8> {
    let shoff = shdr_table_offset(bytes);
    let mut out = bytes[..shoff].to_vec();
    out[0x28..0x30].copy_from_slice(&0u64.to_le_bytes()); // e_shoff
    out[0x3a..0x3c].copy_from_slice(&0u16.to_le_bytes()); // e_shentsize
    out[0x3c..0x3e].copy_from_slice(&0u16.to_le_bytes()); // e_shnum
    out[0x3e..0x40].copy_from_slice(&0u16.to_le_bytes()); // e_shstrndx
    out
}

/// The file offset of a `d_tag`'s VALUE slot in the fixture's PT_DYNAMIC
/// array. Walks the program headers independently of the production
/// reader, so a fixture patched through here and a parse that disagrees
/// are two separate opinions rather than one.
fn dyn_tag_value_offset(bytes: &[u8], tag: u64) -> usize {
    let phoff = u64::from_le_bytes(bytes[0x20..0x28].try_into().expect("e_phoff")) as usize;
    let phnum = u16::from_le_bytes(bytes[0x38..0x3a].try_into().expect("e_phnum")) as usize;
    let (dyn_off, dyn_len) = (0..phnum)
        .map(|i| phoff + i * 56)
        .find_map(|at| {
            let p_type = u32::from_le_bytes(bytes[at..at + 4].try_into().expect("p_type"));
            (p_type == FX_PT_DYNAMIC).then(|| {
                (
                    u64::from_le_bytes(bytes[at + 8..at + 16].try_into().expect("p_offset"))
                        as usize,
                    u64::from_le_bytes(bytes[at + 32..at + 40].try_into().expect("p_filesz"))
                        as usize,
                )
            })
        })
        .expect("the fixture carries a PT_DYNAMIC segment");
    (0..dyn_len / 16)
        .map(|i| dyn_off + i * 16)
        .find(|&at| u64::from_le_bytes(bytes[at..at + 8].try_into().expect("d_tag")) == tag)
        .map(|at| at + 8)
        .unwrap_or_else(|| panic!("the fixture's dynamic array carries no tag {tag}"))
}

fn write_hook_fixture(path: &std::path::Path) {
    std::fs::write(path, hook_fixture_bytes()).expect("write hook fixture");
}

/// An OLDER / incomplete build: a real shared object exporting ONLY the
/// version handshake (none of the window or take-side entry
/// points). A gate that accepts this leaves the rmw to
/// refuse it at resolution and copy.
fn elf_exporting_only_the_handshake() -> Vec<u8> {
    minimal_elf64_so(&[exported(HEAPHOOK_HANDSHAKE_SYMBOL)], b"")
}

/// The complete fixture re-stamped for the OTHER of the two machines the
/// launcher's hosts run on (x86-64 ⇄ AArch64) — a valid hook for the wrong
/// architecture.
fn elf_for_another_machine() -> Vec<u8> {
    let mut bytes = hook_fixture_bytes();
    bytes[0x12..0x14].copy_from_slice(
        &oracle_other_machine()
            .expect("fixture arms skip on an unmapped host before building one")
            .0
            .to_le_bytes(),
    );
    bytes
}

/// An ELF64 shared object that merely MENTIONS the handshake symbol's name
/// as a `.rodata` string — a byte-scan check would accept this.
fn elf_with_the_name_only_in_rodata() -> Vec<u8> {
    let mut rodata = HEAPHOOK_HANDSHAKE_SYMBOL.as_bytes().to_vec();
    rodata.push(0);
    minimal_elf64_so(&[exported("unrelated_export")], &rodata)
}

/// An ELF64 shared object that IMPORTS the handshake symbol (`SHN_UNDEF`) —
/// a consumer of the hook.
fn elf_that_imports_the_symbol() -> Vec<u8> {
    minimal_elf64_so(
        &[FixtureSym {
            name: HEAPHOOK_HANDSHAKE_SYMBOL,
            bind: FX_STB_GLOBAL,
            kind: FX_STT_FUNC,
            defined: false,
        }],
        b"",
    )
}

/// A FOREIGN allocator: a real shared object for this host exporting the
/// C allocator family — what a jemalloc/tcmalloc-style preload looks like
/// to the dynamic loader.
fn foreign_allocator_so() -> Vec<u8> {
    minimal_elf64_so(
        &[
            exported("malloc"),
            exported("free"),
            exported("calloc"),
            exported("realloc"),
        ],
        b"",
    )
}

/// An allocator that exports its family as `STT_GNU_IFUNC` — the shape an
/// optimized build picks so a resolver can select a CPU-specific
/// implementation at load time. To `ld.so` it is a function export like any
/// other, and it wins `malloc` the same way.
fn ifunc_allocator_so() -> Vec<u8> {
    minimal_elf64_so(
        &[
            FixtureSym {
                name: "malloc",
                bind: FX_STB_GLOBAL,
                kind: FX_STT_GNU_IFUNC,
                defined: true,
            },
            FixtureSym {
                name: "free",
                bind: FX_STB_GLOBAL,
                kind: FX_STT_GNU_IFUNC,
                defined: true,
            },
        ],
        b"",
    )
}

/// A harmless library: a real shared object exporting something that is
/// not an allocator entry point.
fn harmless_library_so() -> Vec<u8> {
    minimal_elf64_so(&[exported("libfoo_init"), exported("libfoo_run")], b"")
}

/// The `IncompleteExports` a file that exports exactly `exported_names`
/// (and imports `imported_only`) must produce: `missing` is the required
/// list minus the exports, in the required order.
fn incomplete(exported_names: &[&str], imported_only: &[&'static str]) -> HookFileProblem {
    HookFileProblem::IncompleteExports {
        missing: HEAPHOOK_REQUIRED_EXPORTS
            .iter()
            .copied()
            .filter(|name| !exported_names.contains(name))
            .collect(),
        imported_only: imported_only.to_vec(),
    }
}

/// A tempdir carrying a fixture lib dir (with `librmw_cerulion.so`) and an
/// ament prefix dir — the happy-path scaffolding.
struct Fixture {
    _root: tempfile::TempDir,
    lib_dir: PathBuf,
    ament_prefix: PathBuf,
}

fn fixture() -> Fixture {
    let root = tempfile::tempdir().expect("tempdir");
    let lib_dir = root.path().join("libs");
    std::fs::create_dir_all(&lib_dir).expect("mkdir libs");
    std::fs::write(lib_dir.join(RMW_LIB_FILENAME), b"not a real so").expect("write rmw fixture");
    let ament_prefix = root.path().join("prefix");
    std::fs::create_dir_all(&ament_prefix).expect("mkdir prefix");
    Fixture {
        lib_dir,
        ament_prefix,
        _root: root,
    }
}

fn args(list: &[&str]) -> Vec<String> {
    list.iter().map(|s| s.to_string()).collect()
}

fn env_value<'a>(env: &'a [(String, String)], key: &str) -> Option<&'a str> {
    env.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
}

// ── Pass-through plan oracles (env-touching → #[serial]) ────────────────────

/// The headline oracle: known ambient values in, EXACT env pairs out —
/// `rmw_cerulion` selected, lib dir prepended to `LD_LIBRARY_PATH`, ament
/// prefix prepended to `AMENT_PREFIX_PATH` (order + `:` separator pinned),
/// NO `LD_PRELOAD` (no hook present, nothing asked for one) — and the argv
/// is the native verb token + the forwarded args VERBATIM.
#[test]
#[serial]
fn run_plan_stages_rmw_env_and_forwards_args_verbatim() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _l = EnvVarGuard::set("LD_LIBRARY_PATH", "/ambient/libs");
    let _a = EnvVarGuard::set("AMENT_PREFIX_PATH", "/ambient/ros");
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    let plan = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &args(&[
            "demo_nodes_cpp",
            "talker",
            "--ros-args",
            "-r",
            "chatter:=c2",
        ]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("happy plan must build");

    assert_eq!(plan.program, "ros2");
    assert_eq!(
        plan.args,
        [
            "run",
            "demo_nodes_cpp",
            "talker",
            "--ros-args",
            "-r",
            "chatter:=c2"
        ]
    );
    assert_eq!(
        env_value(&plan.env, "RMW_IMPLEMENTATION"),
        Some("rmw_cerulion")
    );
    assert_eq!(
        env_value(&plan.env, "LD_LIBRARY_PATH"),
        Some(format!("{}:/ambient/libs", f.lib_dir.display()).as_str())
    );
    assert_eq!(
        env_value(&plan.env, "AMENT_PREFIX_PATH"),
        Some(format!("{}:/ambient/ros", f.ament_prefix.display()).as_str())
    );
    assert!(
        env_value(&plan.env, "LD_PRELOAD").is_none(),
        "no preload without a hook or an explicit ask"
    );
}

/// The launch verb: same env, `launch` token first, and the forwarded args
/// are NEVER parsed or validated — hyphenated tokens and a nonexistent file
/// pass through untouched (`ros2 launch` owns its own argument errors).
#[test]
#[serial]
fn launch_plan_forwards_hyphenated_and_unvalidated_args() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);

    let plan = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Launch,
        &args(&[
            "--show-args",
            "/nonexistent/demo.launch.py",
            "use_rviz:=false",
        ]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("arguments are never validated by the wrapper");
    assert_eq!(
        plan.args,
        [
            "launch",
            "--show-args",
            "/nonexistent/demo.launch.py",
            "use_rviz:=false"
        ]
    );
}

/// Absent ambient values: the prepend yields the new entry ALONE — no
/// trailing separator.
#[test]
#[serial]
fn prepend_with_no_ambient_value_has_no_trailing_separator() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _l = EnvVarGuard::unset("LD_LIBRARY_PATH");
    let _a = EnvVarGuard::unset("AMENT_PREFIX_PATH");

    let plan = build_ros2_passthrough_plan(Ros2NativeVerb::Run, &[], &f.lib_dir, &f.ament_prefix)
        .expect("happy plan must build");
    assert_eq!(
        env_value(&plan.env, "LD_LIBRARY_PATH"),
        Some(f.lib_dir.display().to_string().as_str())
    );
    assert_eq!(
        env_value(&plan.env, "AMENT_PREFIX_PATH"),
        Some(f.ament_prefix.display().to_string().as_str())
    );
}

/// A missing `librmw_cerulion.so` is a LOUD typed error naming the file and
/// the HOST's remedy, classified 69 (`EX_UNAVAILABLE`), never a silent
/// fall-through to the stock rmw. The plan builder must emit exactly
/// `missing_rmw_lib_message` for this host and the path it looked at, so the
/// call site cannot drift from the pure function the branch pins below test.
#[test]
#[serial]
fn missing_rmw_lib_is_loud_exit_69_with_remediation() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let rmw_lib = f.lib_dir.join(RMW_LIB_FILENAME);
    std::fs::remove_file(&rmw_lib).expect("remove rmw fixture");

    let err = build_ros2_passthrough_plan(Ros2NativeVerb::Run, &[], &f.lib_dir, &f.ament_prefix)
        .expect_err("a missing rmw lib must refuse");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(msg.contains(RMW_LIB_FILENAME), "names the file: {msg}");
    assert_eq!(
        msg,
        missing_rmw_lib_message(std::env::consts::OS, &rmw_lib),
        "the refusal is the host-keyed message for the path it looked at"
    );
    assert!(
        !msg.contains("cargo build"),
        "an installed user is never sent to cargo: {msg}"
    );
}

/// The missing-library remedy is the HOST's, pinned on both branches on every
/// host through the OS-string parameter: a Linux user is told the Linux
/// packages carry the library and to reinstall one (the override env var
/// still named), a macOS user is told in ONE sentence that ROS 2 Jazzy has no
/// macOS binaries and the verbs need a Linux machine, neither is sent to
/// cargo, and both keep the prefix `classify` maps to exit 69. Any other host
/// takes the Linux wording, the route that exists.
#[test]
fn missing_rmw_lib_message_names_a_route_the_host_can_take() {
    let rmw_lib = Path::new("/opt/cerulion/bin").join(RMW_LIB_FILENAME);

    let linux = missing_rmw_lib_message("linux", &rmw_lib);
    assert!(linux.contains(&rmw_lib.display().to_string()), "{linux}");
    assert!(linux.contains("reinstall Cerulion"), "{linux}");
    assert!(linux.contains("Debian package"), "{linux}");
    assert!(linux.contains(LIB_DIR_ENV), "names the override: {linux}");
    assert!(!linux.contains("cargo"), "{linux}");
    assert!(!linux.contains("macOS"), "{linux}");
    assert_eq!(
        classify(&CliError::Validation(linux.clone())),
        EXIT_UNAVAILABLE
    );

    let macos = missing_rmw_lib_message("macos", &rmw_lib);
    assert!(macos.contains(&rmw_lib.display().to_string()), "{macos}");
    assert!(
        macos.contains("ROS 2 Jazzy has no macOS binaries"),
        "{macos}"
    );
    assert!(macos.contains("need a Linux machine"), "{macos}");
    assert!(!macos.contains("cargo"), "{macos}");
    assert!(!macos.contains("reinstall"), "{macos}");
    assert!(
        !macos.contains(". "),
        "the macOS refusal is one sentence: {macos}"
    );
    assert_eq!(
        classify(&CliError::Validation(macos.clone())),
        EXIT_UNAVAILABLE
    );

    assert_ne!(linux, macos, "the two hosts get different remedies");
    assert_eq!(
        missing_rmw_lib_message("freebsd", &rmw_lib),
        linux,
        "an unlisted host takes the Linux wording"
    );
}

// ── The preload matrix (pure — no env, parallel-safe) ───────────────────────

/// `decide_preload` against the decided matrix, hand oracles: unset
/// auto-injects the hook when present; `off`/`none` kill ALL injection even
/// with the hook there; an explicit value STACKS (user's `.so` FIRST, hook
/// kept when present); set-but-empty is loud.
#[test]
fn preload_decision_matrix_follows_the_ruling() {
    use std::ffi::OsStr;
    let lib = Path::new("/libs");
    let hook = PathBuf::from("/libs").join(HEAPHOOK_FILENAME);

    // unset → AUTO: hook when present, nothing otherwise.
    assert_eq!(
        decide_preload(None, lib, true).expect("auto"),
        PreloadDecision::Inject(vec![hook.clone()])
    );
    assert_eq!(
        decide_preload(None, lib, false).expect("auto absent"),
        PreloadDecision::None
    );
    // The kill switch suppresses injection EVEN when the hook exists.
    for switch in ["off", "none"] {
        assert_eq!(
            decide_preload(Some(OsStr::new(switch)), lib, true).expect(switch),
            PreloadDecision::None,
            "{switch} must disable injection"
        );
    }
    // An explicit value that IS the hook's own path is listed ONCE.
    assert_eq!(
        decide_preload(Some(hook.as_os_str()), lib, true).expect("explicit == hook"),
        PreloadDecision::Inject(vec![hook.clone()]),
        "the hook's own path is never stacked ahead of itself"
    );
    // An explicit value STACKS: user's .so FIRST, then the hook when present
    // (the ORDER is the decision — first-wins is the user's).
    assert_eq!(
        decide_preload(Some(OsStr::new("/x/mine.so")), lib, true).expect("explicit+hook"),
        PreloadDecision::Inject(vec![PathBuf::from("/x/mine.so"), hook.clone()])
    );
    assert_eq!(
        decide_preload(Some(OsStr::new("/x/mine.so")), lib, false).expect("explicit, no hook"),
        PreloadDecision::Inject(vec![PathBuf::from("/x/mine.so")])
    );
    // Set-but-empty is a loud error naming the choices, classified 1.
    let err = decide_preload(Some(OsStr::new("")), lib, true).expect_err("empty is loud");
    assert_eq!(classify(&err), EXIT_OTHER);
    let msg = err.to_string();
    assert!(msg.contains(ROS2_PRELOAD_ENV), "{msg}");
    assert!(msg.contains("off"), "names the kill switch: {msg}");
}

// ── Preload env composition (env-touching → #[serial]) ──────────────────────

/// AUTO-INJECT: the heap hook present in the lib dir is prepended to the
/// ambient `LD_PRELOAD` with no env var set at all.
#[test]
#[serial]
fn heaphook_present_is_auto_injected_before_ambient_preload() {
    let f = fixture();
    let hook = f.lib_dir.join(HEAPHOOK_FILENAME);
    std::fs::write(&hook, b"not a real so").expect("write hook fixture");
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::set("LD_PRELOAD", "/existing.so");

    let plan = build_ros2_passthrough_plan(Ros2NativeVerb::Run, &[], &f.lib_dir, &f.ament_prefix)
        .expect("happy plan must build");
    assert_eq!(
        env_value(&plan.env, "LD_PRELOAD"),
        Some(format!("{}:/existing.so", hook.display()).as_str())
    );
}

/// The kill switch: `CERULION_ROS2_PRELOAD=off` stages NO preload even with
/// the hook sitting right there (`=none` is pinned by the pure matrix).
#[test]
#[serial]
fn preload_off_suppresses_injection_even_with_the_hook_present() {
    let f = fixture();
    std::fs::write(f.lib_dir.join(HEAPHOOK_FILENAME), b"so").expect("write hook fixture");
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, "off");
    let _d = EnvVarGuard::set("LD_PRELOAD", "/existing.so");

    let plan = build_ros2_passthrough_plan(Ros2NativeVerb::Run, &[], &f.lib_dir, &f.ament_prefix)
        .expect("happy plan must build");
    assert!(
        env_value(&plan.env, "LD_PRELOAD").is_none(),
        "the kill switch must suppress ALL preload injection"
    );
}

/// An explicit `CERULION_ROS2_PRELOAD` value STACKS `.bashrc`-style: the
/// user's `.so` FIRST, then the hook (present here), then the ambient
/// `LD_PRELOAD` — order + separator pinned; a variant that puts the hook
/// first fails this.
#[test]
#[serial]
fn explicit_preload_stacks_user_first_then_hook_then_ambient() {
    skip_unless_host_mapped!("explicit_preload_stacks_user_first_then_hook_then_ambient");
    let f = fixture();
    let hook = f.lib_dir.join(HEAPHOOK_FILENAME);
    write_hook_fixture(&hook);
    let explicit = f.lib_dir.join("libcerulion_custom.so");
    std::fs::write(&explicit, b"so").expect("write explicit fixture");
    let explicit_str = explicit.display().to_string();
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &explicit_str);
    let _d = EnvVarGuard::set("LD_PRELOAD", "/existing.so");

    let plan = build_ros2_passthrough_plan(Ros2NativeVerb::Run, &[], &f.lib_dir, &f.ament_prefix)
        .expect("happy plan must build");
    assert_eq!(
        env_value(&plan.env, "LD_PRELOAD"),
        Some(format!("{explicit_str}:{}:/existing.so", hook.display()).as_str()),
        "user's .so first, the hook kept, the ambient kept"
    );
}

/// An explicit value with NO hook present: user's `.so` then the ambient.
#[test]
#[serial]
fn explicit_preload_without_hook_prepends_ambient_only() {
    let f = fixture();
    let explicit = f.lib_dir.join("libcerulion_custom.so");
    std::fs::write(&explicit, b"so").expect("write explicit fixture");
    let explicit_str = explicit.display().to_string();
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &explicit_str);
    let _d = EnvVarGuard::set("LD_PRELOAD", "/existing.so");

    let plan = build_ros2_passthrough_plan(Ros2NativeVerb::Run, &[], &f.lib_dir, &f.ament_prefix)
        .expect("happy plan must build");
    assert_eq!(
        env_value(&plan.env, "LD_PRELOAD"),
        Some(format!("{explicit_str}:/existing.so").as_str())
    );
}

// ── `--adopt-take` ─────────────────────────────
//
// TWO entries below, deliberately:
//   * `build_ros2_passthrough_plan` / `_for_host` — what `cerulion ros2
//     run|launch` really call. Since the launcher-refusal rule these REFUSE the
//     flag, so every arm driving them with it asserts the refusal.
//   * `build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test` —
//     the RETAINED gate (host support, staged preload, hook
//     inspection, preload order, descriptor binding). The function behind it
//     is the production copy-path builder; only its `if adopt_take` BRANCH
//     is reachable nowhere but through this `test-seams` seam.
//
//     Those arms are design-preservation pins, not regression pins. Read
//     EVERY sentence in them — flat present tense included — as a claim
//     about the retained gate, never about what `cerulion ros2 run|launch`
//     does today: since the launcher-refusal rule neither verb reaches that
//     branch at all, so an arm saying the flag "sets CERULION_RMW_ADOPT_TAKE
//     for the child", or that two refusals "are the SAME refusal", is
//     describing the gate as it will behave when a two-hop-safe binding
//     scheme re-enables it. The same caveat covers the descriptor and
//     plan-identity arms under "── Classification ──" and the inspector arms
//     under "── Spawn-based env application ──", which name the verb in the
//     present tense from outside this section.

/// The pure leading-token split: only LEADING `--adopt-take` tokens are
/// Cerulion's; from the first other token on, everything (a later
/// `--adopt-take` included) is forwarded verbatim — the pass-through
/// guarantee for ros2's own surface.
#[test]
fn split_adopt_take_flag_consumes_leading_tokens_only() {
    let flag = ADOPT_TAKE_FLAG;
    assert_eq!(split_adopt_take_flag(&args(&[])), (false, args(&[])));
    assert_eq!(split_adopt_take_flag(&args(&[flag])), (true, args(&[])));
    assert_eq!(
        split_adopt_take_flag(&args(&[flag, flag, "pkg", "exe"])),
        (true, args(&["pkg", "exe"])),
        "repeats are idempotent"
    );
    assert_eq!(
        split_adopt_take_flag(&args(&["pkg", flag])),
        (false, args(&["pkg", flag])),
        "a non-leading flag belongs to ros2's surface, verbatim"
    );
    assert_eq!(
        split_adopt_take_flag(&args(&[flag, "pkg", "exe", flag])),
        (true, args(&["pkg", "exe", flag])),
        "the trailing occurrence stays forwarded"
    );
}

/// The headline: `cerulion ros2 run` /
/// `ros2 launch` REFUSE `--adopt-take` outright. `ros2` is a Python CLI
/// that spawns the node as a further subprocess with `close_fds=True`, so
/// the inherited descriptor the validated hook rides (the binding
/// stands) is closed before the node's loader reads `LD_PRELOAD`;
/// every take would be served by copy after this launcher had reported
/// success.
///
/// The refusal is HOST-INDEPENDENT (both hosts asserted here) and lands
/// BEFORE any file is looked at. Be exact about which assertion does which
/// job: `hook_inspections_for_test` not moving is an ORDERING pin only —
/// this fixture stages no hook, so the retained gate and the platform gate
/// would each also refuse without inspecting anything, and the counter
/// reads 0 on all three paths. What separates "refused by the decision" from
/// "refused because nothing was staged" is the absence assertion on the
/// staged-preload sentence, and that one is PAIRED, in this body, with a
/// positive: the RETAINED gate, driven over the identical arguments,
/// really does answer with that sentence. So a rewording fails the
/// positive half loudly instead of quietly excusing the negative. (The
/// DISCRIMINATING inspection-counter pin, over a fixture that stages a real
/// hook, lives in `the_default_entry_point_refuses_adoption_*`.)
#[test]
#[serial]
fn adopt_take_is_refused_under_the_ros2_cli_on_every_host_before_any_inspection() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    for host in [
        HostAbi::LINUX_GNU,
        HostAbi {
            os: "macos",
            env: "",
        },
    ] {
        for verb in [Ros2NativeVerb::Run, Ros2NativeVerb::Launch] {
            let before = hook_inspections_for_test();
            let err = build_ros2_passthrough_plan_for_host(
                verb,
                &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
                &f.lib_dir,
                &f.ament_prefix,
                host,
            )
            .expect_err("both verbs must refuse --adopt-take");
            assert_eq!(
                hook_inspections_for_test() - before,
                0,
                "refused before any file is inspected ({host:?})"
            );
            assert_eq!(classify(&err), EXIT_UNAVAILABLE, "exit 69 ({host:?})");
            let msg = err.to_string();
            for token in [
                ADOPT_TAKE_FLAG,
                "ros2 run",
                "ros2 launch",
                "close_fds=True",
                "LD_PRELOAD",
                "DIRECTLY",
                ADOPT_TAKE_CHILD_ENV,
                "copy path",
                // The direct-launch recipe must PREPEND each path var to the
                // sourced value, exactly as `prepend_path_var` stages it. A
                // recipe that ASSIGNS them reads as runnable and is not: in
                // the sourced ROS 2 shell it is printed in it would drop the
                // distro's own ament index and libraries. That is the
                // remedy-that-cannot-work class this refusal exists to kill,
                // so the composition is pinned, not just the var names.
                "${LD_PRELOAD:+:$LD_PRELOAD}",
                "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}",
                "${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}",
            ] {
                assert!(msg.contains(token), "refusal must name `{token}`: {msg}");
            }
            // Each var is pinned to ITS OWN value, not merely to the value
            // being present somewhere: `fixture()` gives lib_dir and
            // ament_prefix distinct paths, so a recipe that SWAPPED them
            // (ament prefix on LD_LIBRARY_PATH, lib dir on
            // AMENT_PREFIX_PATH) still contains both strings and would pass
            // a presence check while telling the user something unrunnable.
            for (var, value) in [
                (
                    "LD_PRELOAD",
                    f.lib_dir.join(HEAPHOOK_FILENAME).display().to_string(),
                ),
                ("LD_LIBRARY_PATH", f.lib_dir.display().to_string()),
                ("AMENT_PREFIX_PATH", f.ament_prefix.display().to_string()),
            ] {
                let assignment = format!("{var}={value}");
                assert!(
                    msg.contains(&assignment),
                    "the direct-launch recipe must set `{assignment}`: {msg}"
                );
            }
            assert!(
                !msg.contains(NOTHING_STAGED_SUBSTR),
                "the adopt refusal answers, not the staged-preload guard: {msg}"
            );
        }
    }
    // The PAIRED positive: the retained gate, same arguments, same empty
    // fixture — it is the guard that owns the sentence asserted absent
    // above, so a rewording is loud here.
    let gated = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("no hook is staged, so the retained gate refuses too");
    assert!(
        gated.to_string().contains(NOTHING_STAGED_SUBSTR),
        "the retained gate still owns the staged-preload refusal: {gated}"
    );
}

/// A minimal POSIX word splitter: whitespace separates words EXCEPT inside
/// single quotes, and a backslash outside quotes escapes the next byte (so
/// the `'\''` idiom reads as one literal quote). Enough to answer the only
/// question asked of it — does this line tokenize the way the message
/// intends? — without pulling in a shell.
fn shell_words(line: &str) -> Vec<String> {
    let mut words = Vec::new();
    let mut word = String::new();
    let mut started = false;
    let mut quoted = false;
    let mut chars = line.chars();
    while let Some(c) = chars.next() {
        match c {
            '\'' => {
                quoted = !quoted;
                started = true;
            }
            '\\' if !quoted => {
                if let Some(next) = chars.next() {
                    word.push(next);
                    started = true;
                }
            }
            c if c.is_whitespace() && !quoted => {
                if started {
                    words.push(std::mem::take(&mut word));
                    started = false;
                }
            }
            c => {
                word.push(c);
                started = true;
            }
        }
    }
    if started {
        words.push(word);
    }
    words
}

/// Like [`shell_words`], but keeps each word's RAW text — quotes included.
///
/// `shell_words` erases quote boundaries and
/// treats `$` as ordinary text, so it cannot tell `'path'${VAR:+:$VAR}`
/// (the suffix EXPANDS) from `'path${VAR:+:$VAR}'` (the suffix is literal).
/// Both tokenize to the same word, so the very property the prepend
/// guarantees would be pinned by nothing. The raw text is what the two structural
/// and behavioural checks below need.
fn shell_words_raw(line: &str) -> Vec<String> {
    let mut words = Vec::new();
    let mut word = String::new();
    let mut started = false;
    let mut quoted = false;
    let mut chars = line.chars();
    while let Some(c) = chars.next() {
        match c {
            '\'' => {
                quoted = !quoted;
                word.push(c);
                started = true;
            }
            '\\' if !quoted => {
                word.push(c);
                if let Some(next) = chars.next() {
                    word.push(next);
                }
                started = true;
            }
            c if c.is_whitespace() && !quoted => {
                if started {
                    words.push(std::mem::take(&mut word));
                    started = false;
                }
            }
            c => {
                word.push(c);
                started = true;
            }
        }
    }
    if started {
        words.push(word);
    }
    words
}

/// Every interpolated path in the printed
/// direct-launch recipe must be POSIX-shell-quoted. The recipe exists to be
/// PASTED, so a path carrying a shell metacharacter otherwise breaks the
/// line when a reader runs it.
///
/// The fixture uses an apostrophe and a `$` and DELIBERATELY no space or
/// colon: those two are not a quoting problem
/// at all but an undeliverable one — glibc re-splits the variables on them
/// after the shell is done — and that case is the sibling arm below. What is
/// left here is the genuine quoting class.
///
/// The oracle is the COMPLETE tokenized word list, not a substring and not a
/// count: validating only the three path assignments plus a
/// length leaves `RMW_IMPLEMENTATION=rmw_cerulion` and
/// `CERULION_RMW_ADOPT_TAKE=1` unpinned, so changing either still passes.
///
/// The `${VAR:+:$VAR}` suffixes must stay OUTSIDE the quotes — adjacent
/// quoted and unquoted parts concatenate into one word — or they would print
/// literally instead of expanding, which the whole-word oracle also catches.
#[test]
#[serial]
fn the_direct_launch_recipe_quotes_every_path_it_interpolates() {
    let root = tempfile::tempdir().expect("tempdir");
    // No space and no colon anywhere — those are the loader's delimiters and
    // belong to the sibling arm. An apostrophe, a `$` and a `;` are the
    // shell's problem and this arm's subject.
    let lib_dir = root.path().join("ament's$lib;dir");
    let ament_prefix = root.path().join("prefix's$home");
    std::fs::create_dir_all(&lib_dir).expect("mkdir lib");
    std::fs::create_dir_all(&ament_prefix).expect("mkdir prefix");
    std::fs::write(lib_dir.join(RMW_LIB_FILENAME), b"not a real so").expect("write rmw");
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    let err = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &lib_dir,
        &ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("the flag is refused");
    let msg = err.to_string();

    // Tokenize the RECIPE, not the whole message: the surrounding prose is
    // English full of apostrophes ("the node's loader", "glibc's
    // malloc/free"), and a word splitter reads every one of those as a
    // quote. The recipe is the backtick-delimited run starting at the first
    // assignment — the exact span a reader would select and paste.
    let start = msg
        .find("`LD_PRELOAD=")
        .expect("the refusal carries a direct-launch recipe")
        + 1;
    let end = start
        + msg[start..]
            .find('`')
            .expect("the recipe is backtick-delimited");
    let recipe = &msg[start..end];

    let expected: Vec<String> = vec![
        format!(
            "LD_PRELOAD={}${{LD_PRELOAD:+:$LD_PRELOAD}}",
            lib_dir.join(HEAPHOOK_FILENAME).display()
        ),
        format!(
            "LD_LIBRARY_PATH={}${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}}",
            lib_dir.display()
        ),
        format!(
            "AMENT_PREFIX_PATH={}${{AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}}",
            ament_prefix.display()
        ),
        "RMW_IMPLEMENTATION=rmw_cerulion".to_string(),
        format!("{ADOPT_TAKE_CHILD_ENV}=1"),
        "<install>/lib/<pkg>/<executable>".to_string(),
    ];
    assert_eq!(
        shell_words(recipe),
        expected,
        "the recipe must tokenize to EXACTLY these words: {recipe}"
    );

    // The word list above CANNOT see where the quotes were — `shell_words`
    // consumes them — so `'path'${VAR:+:$VAR}` and `'path${VAR:+:$VAR}'`
    // are identical to it, and the prepend property is unpinned by it
    // alone. Two checks that can see it, on the RAW text:
    let raw = shell_words_raw(recipe);
    for (var, path) in [
        (
            "LD_PRELOAD",
            lib_dir.join(HEAPHOOK_FILENAME).display().to_string(),
        ),
        ("LD_LIBRARY_PATH", lib_dir.display().to_string()),
        ("AMENT_PREFIX_PATH", ament_prefix.display().to_string()),
    ] {
        let word = raw
            .iter()
            .find(|w| w.starts_with(&format!("{var}=")))
            .unwrap_or_else(|| panic!("the recipe assigns {var}: {recipe}"));

        // STRUCTURE: the expansion must begin AFTER the closing quote.
        let close = word
            .rfind('\'')
            .unwrap_or_else(|| panic!("{var}'s value is quoted in this fixture: {word}"));
        let expansion = word
            .find("${")
            .unwrap_or_else(|| panic!("{var} carries a prepend suffix: {word}"));
        assert!(
            close < expansion,
            "{var}: the `${{…}}` suffix must sit OUTSIDE the quotes, or it is printed \
             literally instead of expanding: {word}"
        );

        // BEHAVIOUR, which is the claim itself: hand the rendered assignment
        // to a real `sh` and read back what it set, with the variable unset
        // and then set. A literal suffix fails both.
        for (ambient, want) in [
            (None, path.clone()),
            (Some("/ambient.so"), format!("{path}:/ambient.so")),
        ] {
            let mut cmd = std::process::Command::new("sh");
            cmd.arg("-c")
                .arg(format!("{word}; printf %s \"${{{var}}}\""));
            match ambient {
                Some(v) => cmd.env(var, v),
                None => cmd.env_remove(var),
            };
            let out = cmd.output().expect("run sh");
            assert!(out.status.success(), "sh failed on: {word}");
            assert_eq!(
                String::from_utf8_lossy(&out.stdout),
                want,
                "{var} with ambient {ambient:?}: the rendered assignment must PREPEND to the \
                 sourced value — got the left side, expected the right, from: {word}"
            );
        }
    }
}

/// Shell quoting buys tokenization by the SHELL
/// and nothing more. `LD_PRELOAD` is re-split by the LOADER — glibc's
/// `handle_preload_list` is `strsep(&list, " :")`, space as well as colon —
/// and `LD_LIBRARY_PATH` / `AMENT_PREFIX_PATH` are colon-separated lists. A
/// hook at `/tmp/lib dir/libcerulion_heaphook.so` therefore yields a recipe
/// that pastes cleanly, starts the node, and SILENTLY runs the copy path.
///
/// So the recipe is WITHHELD rather than printed with a caveat: the refusal
/// says plainly that adoption cannot be armed from that location, why, and
/// what to do. Three assertions carry that: the explanation is present, the
/// promise is NOT, and — the one that matters — no `LD_PRELOAD=` assignment
/// is emitted at all, so there is nothing for a reader to paste and have
/// quietly fail.
///
/// Both delimiters and both colon-separated variables are driven; the
/// sibling arm above is the anti-tautology half (a delimiter-free path DOES
/// get a runnable recipe), so this is not "the recipe is never printed".
#[test]
#[serial]
fn a_path_the_loader_cannot_carry_gets_no_recipe_at_all() {
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let root = tempfile::tempdir().expect("tempdir");

    for (label, lib_name, ament_name, var) in [
        ("space in the hook path", "lib dir", "prefix", "LD_PRELOAD"),
        ("colon in the hook path", "lib:dir", "prefix", "LD_PRELOAD"),
        (
            "colon in the ament prefix",
            "libs",
            "pre:fix",
            "AMENT_PREFIX_PATH",
        ),
    ] {
        let lib_dir = root.path().join(lib_name);
        let ament_prefix = root.path().join(ament_name);
        std::fs::create_dir_all(&lib_dir).expect("mkdir lib");
        std::fs::create_dir_all(&ament_prefix).expect("mkdir prefix");
        std::fs::write(lib_dir.join(RMW_LIB_FILENAME), b"not a real so").expect("write rmw");

        let err = build_ros2_passthrough_plan_for_host(
            Ros2NativeVerb::Run,
            &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
            &lib_dir,
            &ament_prefix,
            HostAbi::LINUX_GNU,
        )
        .expect_err("the flag is refused");
        let msg = err.to_string();

        assert_eq!(classify(&err), EXIT_UNAVAILABLE, "{label}");
        assert!(
            msg.contains(&format!("CANNOT be carried in {var}")),
            "{label}: the refusal must say the path cannot be carried: {msg}"
        );
        assert!(
            msg.contains("no space or colon"),
            "{label}: and name the fix: {msg}"
        );
        // THE assertion: nothing pasteable is offered.
        assert!(
            !msg.contains("LD_PRELOAD="),
            "{label}: no recipe may be emitted for a path the loader cannot carry: {msg}"
        );
        assert!(
            !msg.contains("adoption can arm there"),
            "{label}: and the promise must not be made: {msg}"
        );
    }
}

/// Refused on the same code path as the delimiter case:
/// a recipe is TEXT, and `Display` for a path replaces bytes that are not
/// valid UTF-8 with U+FFFD — so rendering one would hand the reader a
/// command naming a location that does not exist. Same silent-wrong-remedy
/// class, same answer: say what cannot be done and why, print no recipe.
///
/// Unix-only, because that is where a path can hold arbitrary bytes.
#[cfg(unix)]
#[test]
#[serial]
fn a_path_that_is_not_utf8_gets_no_recipe_either() {
    use std::os::unix::ffi::OsStrExt;

    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let root = tempfile::tempdir().expect("tempdir");
    // A lone 0xFF byte is valid in a POSIX path and invalid in UTF-8.
    let lib_dir = root.path().join(std::ffi::OsStr::from_bytes(b"lib\xffdir"));
    let ament_prefix = root.path().join("prefix");
    // Nothing is CREATED: the refusal is decided before any file is looked
    // at, which is the decision this change implements — and it is what makes this
    // arm portable, since macOS refuses to create a non-UTF-8 filename at
    // all ("Illegal byte sequence"). The path only has to be NAMEABLE.
    assert!(
        lib_dir.to_str().is_none(),
        "precondition: the fixture path really is not UTF-8"
    );

    let err = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &lib_dir,
        &ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("the flag is refused");
    let msg = err.to_string();

    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    assert!(
        msg.contains("is not valid UTF-8"),
        "the refusal must say why no recipe is printed: {msg}"
    );
    assert!(!msg.contains("LD_PRELOAD="), "and print none: {msg}");
    assert!(
        !msg.contains("adoption can arm there"),
        "and make no promise it cannot keep: {msg}"
    );
}

/// The anti-tautology half of the decision: the refusal is the LEADING FLAG,
/// not the verb and not the token.
///
/// Two legs, both through the PRODUCTION entry. (a) The identical launch
/// WITHOUT `--adopt-take` still builds its copy-path plan and arms nothing.
/// (b) A NON-LEADING `--adopt-take` is still FORWARDED VERBATIM — it is
/// `ros2`'s argument surface from the first non-Cerulion token on, which is
/// the pass-through guarantee both verbs are built around, so the refusal
/// must key on `split_adopt_take_flag`, never on "the token appears
/// somewhere". Leg (b) is what fails a refusal written as
/// `args.iter().any(|a| a == ADOPT_TAKE_FLAG)`, which leg (a) and every
/// refusal arm would happily pass.
#[test]
#[serial]
fn the_ros2_cli_refusal_is_the_leading_flag_not_the_verb_or_the_token() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let _a = EnvVarGuard::unset(ADOPT_TAKE_CHILD_ENV);
    for verb in [Ros2NativeVerb::Run, Ros2NativeVerb::Launch] {
        let plan = build_ros2_passthrough_plan_for_host(
            verb,
            &args(&["pkg", "exe"]),
            &f.lib_dir,
            &f.ament_prefix,
            HostAbi::LINUX_GNU,
        )
        .expect("the copy path is untouched by the adopt refusal");
        assert_eq!(plan.args, vec![verb.as_str(), "pkg", "exe"]);
        assert!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV).is_none());

        let plan = build_ros2_passthrough_plan_for_host(
            verb,
            &args(&["pkg", "exe", ADOPT_TAKE_FLAG]),
            &f.lib_dir,
            &f.ament_prefix,
            HostAbi::LINUX_GNU,
        )
        .expect("a non-leading --adopt-take belongs to ros2's surface, verbatim");
        assert_eq!(
            plan.args,
            vec![verb.as_str(), "pkg", "exe", ADOPT_TAKE_FLAG],
            "the trailing occurrence must reach ros2 untouched"
        );
        assert!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV).is_none());
    }
}

/// The happy path: a leading `--adopt-take` with the hook staged sets
/// CERULION_RMW_ADOPT_TAKE=1 for the child, keeps the hook in LD_PRELOAD,
/// and forwards the remaining args verbatim (a trailing `--adopt-take`
/// included — ros2 owns it).
#[test]
#[serial]
fn adopt_take_flag_sets_child_env_and_forwards_the_rest_verbatim() {
    skip_unless_host_mapped!("adopt_take_flag_sets_child_env_and_forwards_the_rest_verbatim");
    let f = fixture();
    let hook = f.lib_dir.join(HEAPHOOK_FILENAME);
    write_hook_fixture(&hook);
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "demo_nodes_cpp", "talker", ADOPT_TAKE_FLAG]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("hook staged ⇒ the flag must build a plan");
    assert_eq!(
        plan.args,
        ["run", "demo_nodes_cpp", "talker", ADOPT_TAKE_FLAG],
        "leading flag consumed, trailing flag forwarded verbatim"
    );
    assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));
    // Decision: the validated hook is handed to the child
    // by DESCRIPTOR, so its entry is `/proc/self/fd/<N>` rather than its
    // name — that is what closes the substitution window. That the
    // descriptor really identifies THIS hook is pinned by
    // `the_validated_hook_is_handed_to_the_child_by_descriptor`.
    assert!(
        env_value(&plan.env, "LD_PRELOAD").is_some_and(|v| v.starts_with("/proc/self/fd/")),
        "the hook is staged by descriptor: {:?}",
        env_value(&plan.env, "LD_PRELOAD")
    );
    assert!(hook.exists(), "and the validated file is still there");
}

/// Without the flag, the child env must NOT carry the adopt var (catches a
/// variant that pushes it unconditionally).
#[test]
#[serial]
fn no_flag_stages_no_adopt_env() {
    let f = fixture();
    std::fs::write(f.lib_dir.join(HEAPHOOK_FILENAME), b"so").expect("write hook fixture");
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    let plan = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &args(&["demo_nodes_cpp", "talker"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("happy plan must build");
    assert!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV).is_none());
}

/// `--adopt-take` with NO hook beside the binary is a loud exit-69 refusal
/// naming the hook, the build command and the flag — the launcher path
/// cannot start mis-configured (the rmw's env-without-preload degrade warn
/// is unreachable from here).
#[test]
#[serial]
fn adopt_take_without_the_hook_is_loud_exit_69() {
    let f = fixture(); // no hook written
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Launch,
        &args(&[ADOPT_TAKE_FLAG, "demo.launch.py"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("no hook ⇒ the flag must refuse");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(msg.contains(HEAPHOOK_FILENAME), "names the hook: {msg}");
    assert!(
        msg.contains("cargo build -p cerulion_heaphook"),
        "names the build remedy: {msg}"
    );
    assert!(msg.contains(ADOPT_TAKE_FLAG), "names the flag: {msg}");
    // The nothing-staged
    // refusal and the LATER "no staged preload is the built hook" one share
    // one prefix, and `classify` maps both to the same exit code — so every
    // assertion above is equally satisfied by the later check, and deleting
    // the nothing-staged guard would leave this arm green (its `position()` over an
    // empty list yields `None` and the later arm answers with an empty
    // problem list). Pin the sentence only THIS refusal emits, and pin that
    // the later one did not answer in its place.
    assert!(
        msg.contains(NOTHING_STAGED_SUBSTR),
        "the nothing-staged guard refused: {msg}"
    );
    assert!(
        !msg.contains(NOT_THE_HOOK_SUBSTR),
        "...and not the later hook-inspection refusal: {msg}"
    );
    // The same-body POSITIVE anchor for `NOT_THE_HOOK_SUBSTR`. Its other
    // anchor lives in an arm gated on `skip_unless_host_mapped!`, so on a
    // host the elf.h oracle has no row for, the three `!contains` uses of
    // this const would run unanchored and a rewording would slip through.
    // Non-ELF bytes are refused before any machine check, so this leg
    // needs no fixture and no gate.
    std::fs::write(f.lib_dir.join(HEAPHOOK_FILENAME), b"not a real so").expect("write");
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Launch,
        &args(&[ADOPT_TAKE_FLAG, "demo.launch.py"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("a staged non-hook file must refuse too");
    assert!(
        err.to_string().contains(NOT_THE_HOOK_SUBSTR),
        "the sentence the siblings assert ABSENT is really still emitted: {err}"
    );
}

/// `--adopt-take` combined with the `CERULION_ROS2_PRELOAD=off` kill switch
/// is the SAME refusal: the flag demands the preload, and the two settings
/// contradict — refused loudly, never a silent copy-path run behind a flag
/// that promised adoption.
#[test]
#[serial]
fn adopt_take_with_preload_kill_switch_is_loud_exit_69() {
    let f = fixture();
    std::fs::write(f.lib_dir.join(HEAPHOOK_FILENAME), b"so").expect("write hook fixture");
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, "off");
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("kill switch + flag ⇒ refuse");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(
        msg.contains(ROS2_PRELOAD_ENV),
        "names the kill switch: {msg}"
    );
    // Both refusals name `CERULION_ROS2_PRELOAD`, so
    // the assertion above discriminates nothing — the same two-sided pin as
    // the sibling arm above. The `=off` half independently catches reverting
    // the guard rather than being a statement about this run: the
    // nothing-staged message carries `CERULION_ROS2_PRELOAD=off` in every
    // case, switch set or not, while the later hook-inspection message
    // names the variable but never `=off`.
    assert!(
        msg.contains(&format!("{ROS2_PRELOAD_ENV}=off")),
        "the nothing-staged wording, which alone carries the switch value: {msg}"
    );
    assert!(
        msg.contains(NOTHING_STAGED_SUBSTR),
        "the nothing-staged guard refused: {msg}"
    );
    assert!(
        !msg.contains(NOT_THE_HOOK_SUBSTR),
        "...and not the later hook-inspection refusal: {msg}"
    );
}

/// The flag composes with an explicit stacked preload: user's `.so` first,
/// the hook kept — and the adopt env still set.
#[test]
#[serial]
fn adopt_take_composes_with_an_explicit_stacked_preload() {
    skip_unless_host_mapped!("adopt_take_composes_with_an_explicit_stacked_preload");
    let f = fixture();
    let hook = f.lib_dir.join(HEAPHOOK_FILENAME);
    write_hook_fixture(&hook);
    let explicit = f.lib_dir.join("libcerulion_custom.so");
    std::fs::write(&explicit, b"so").expect("write explicit fixture");
    let explicit_str = explicit.display().to_string();
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &explicit_str);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("stacked preload keeps the hook ⇒ the flag must build");
    assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));
    // Decision: the explicit `.so` keeps its NAME — the
    // launcher validates only the hook — while the hook itself is handed
    // over by descriptor, so the stacking ORDER is what this pins.
    assert!(
        env_value(&plan.env, "LD_PRELOAD")
            .is_some_and(|v| v.starts_with(&format!("{explicit_str}:/proc/self/fd/"))),
        "explicit first by name, hook second by descriptor: {:?}",
        env_value(&plan.env, "LD_PRELOAD")
    );
    assert!(hook.exists(), "and the validated hook is still there");
}

/// `CERULION_ROS2_PRELOAD` naming the hook's
/// OWN path while the file is ABSENT. Were the explicit entry exempt
/// from the existence check ("the hook is in the list because it exists"),
/// the missing path would be staged and `--adopt-take`, finding the hook's
/// path in LD_PRELOAD, would accept adoption for a child that would run the
/// copy path — the silent degrade the flag exists to refuse. Instead: a loud
/// exit-69 refusal naming the path and the hook's build remedy.
#[test]
#[serial]
fn adopt_take_with_preload_naming_an_absent_hook_is_loud_exit_69() {
    let f = fixture(); // no hook written
    let hook_str = f.lib_dir.join(HEAPHOOK_FILENAME).display().to_string();
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &hook_str);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("an absent hook named by the env must refuse adoption");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(msg.contains(&hook_str), "names the path: {msg}");
    assert!(
        msg.contains("cargo build -p cerulion_heaphook"),
        "names the hook's build remedy: {msg}"
    );
    // The LATER hook-inspection refusal also names
    // this path (folded into its problem list as "does not exist") and the
    // same build remedy, so both assertions above survive deleting the
    // per-entry existence check. Pin WHICH refusal answered.
    assert!(
        msg.contains(PRELOAD_MISSING_SUBSTR),
        "the per-entry existence check refused: {msg}"
    );
    assert!(
        !msg.contains(NOT_THE_HOOK_SUBSTR),
        "...not the later hook-inspection refusal: {msg}"
    );
}

/// The same env value with the file present and inspecting as the BUILT
/// hook (ELF magic + the handshake symbol) is ACCEPTED: adoption armed, and
/// the hook staged ONCE (an explicit value that IS the hook's path is not
/// stacked ahead of itself).
#[test]
#[serial]
fn adopt_take_with_preload_naming_the_real_hook_is_accepted() {
    skip_unless_host_mapped!("adopt_take_with_preload_naming_the_real_hook_is_accepted");
    let f = fixture();
    let hook = f.lib_dir.join(HEAPHOOK_FILENAME);
    write_hook_fixture(&hook);
    let hook_str = hook.display().to_string();
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &hook_str);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("the built hook named explicitly must be accepted");
    assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));
    // Decision: by DESCRIPTOR, not by the name — the
    // validation and the load refer to one inode, which is what
    // closes the substitution window. `hook_str` still names the file that was
    // validated; the descriptor identifying it is pinned by
    // `the_validated_hook_is_handed_to_the_child_by_descriptor`.
    assert!(
        env_value(&plan.env, "LD_PRELOAD").is_some_and(|v| v.starts_with("/proc/self/fd/")),
        "staged by descriptor, exactly once: {:?}",
        env_value(&plan.env, "LD_PRELOAD")
    );
}

/// The same env value naming a file that is NOT the hook — arbitrary bytes
/// (no ELF magic), then an ELF-shaped file without the handshake symbol —
/// is refused loudly, each time naming the path, WHY it is not the hook,
/// and the rebuild remedy. The path is present and staged, so only the
/// file inspection can refuse here (a variant that drops it passes this
/// arm's inputs straight through to a plan).
#[test]
#[serial]
fn adopt_take_with_preload_naming_a_non_hook_file_is_loud_exit_69() {
    skip_unless_host_mapped!("adopt_take_with_preload_naming_a_non_hook_file_is_loud_exit_69");
    let f = fixture();
    let hook = f.lib_dir.join(HEAPHOOK_FILENAME);
    let hook_str = hook.display().to_string();
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &hook_str);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    for (bytes, why) in [
        (b"not a real so".to_vec(), "is not an ELF shared object"),
        (elf_with_the_name_only_in_rodata(), "does not export"),
        (elf_that_imports_the_symbol(), "only imports"),
        (
            elf_exporting_only_the_handshake(),
            "cerulion_heaphook_set_release_callback",
        ),
        (elf_for_another_machine(), "but this host is"),
    ] {
        std::fs::write(&hook, &bytes).expect("write non-hook fixture");
        let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
            Ros2NativeVerb::Run,
            &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
            &f.lib_dir,
            &f.ament_prefix,
            HostAbi::LINUX_GNU,
        )
        .expect_err("a non-hook file at the hook's path must refuse adoption");
        assert_eq!(classify(&err), EXIT_UNAVAILABLE);
        let msg = err.to_string();
        assert!(msg.contains(&hook_str), "names the path: {msg}");
        assert!(msg.contains(why), "names why ({why}): {msg}");
        assert!(
            msg.contains("cargo build -p cerulion_heaphook"),
            "names the build remedy: {msg}"
        );
        // The POSITIVE anchor for `NOT_THE_HOOK_SUBSTR`: this arm is the
        // one that owns the hook-inspection refusal, so its siblings may
        // assert that sentence's ABSENCE without the assertion going
        // vacuous behind a rewording.
        assert!(
            msg.contains(NOT_THE_HOOK_SUBSTR),
            "the hook-inspection refusal answered: {msg}"
        );
        assert!(
            !msg.contains(NOTHING_STAGED_SUBSTR),
            "...not the nothing-staged guard, which had a path to work with: {msg}"
        );
    }
}

/// The NON-adopt half of the same hole: an explicit preload naming the
/// hook's own ABSENT path is the loud exit-69 missing-library error the
/// preload contract promises for EVERY explicit path — never a staged preload
/// the dynamic loader would drop with a message nobody reads.
#[test]
#[serial]
fn explicit_preload_naming_the_absent_hook_path_is_loud_exit_69() {
    let f = fixture(); // no hook written
    let hook_str = f.lib_dir.join(HEAPHOOK_FILENAME).display().to_string();
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &hook_str);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    let err = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &args(&["pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("the hook's own absent path is a missing explicit library");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(msg.contains(&hook_str), "names the path: {msg}");
    assert!(msg.contains(ROS2_PRELOAD_ENV), "names the var: {msg}");
    assert!(
        msg.contains("cargo build -p cerulion_heaphook"),
        "the hook's own path gets the hook's remedy: {msg}"
    );
}

/// The pure inspector, one problem per shape, hand oracles (no env) —
/// absent, a directory, an empty file, arbitrary bytes, then the ELF
/// shapes a real symbol check must tell apart: the name only as a
/// `.rodata` string (a byte-scan check's false accept), the name only
/// as an IMPORT, the name as a PREFIX of a longer export, a LOCAL or a
/// NOTYPE entry with the name, ONLY the handshake exported (a
/// handshake-only gate's false accept), a header-only object with no `.dynsym`, the OTHER
/// machine, ELF32 / big-endian / `ET_EXEC` shapes, two malformed tables —
/// and the faithful fixture (Ok), reached through a symlink too
/// (`metadata` follows it, as the dynamic loader does). Each refusal's
/// message names something the operator can check with `nm -D` or
/// `readelf -h`.
#[test]
#[serial]
fn inspect_heaphook_file_classifies_each_problem() {
    skip_unless_host_mapped!("inspect_heaphook_file_classifies_each_problem");
    let root = tempfile::tempdir().expect("tempdir");
    let p = |name: &str| root.path().join(name);
    let check = |name: &str, bytes: &[u8], want: Result<(), HookFileProblem>| {
        std::fs::write(p(name), bytes).expect("write");
        assert_eq!(inspect_heaphook_file(&p(name)), want, "{name}");
    };
    assert_eq!(
        inspect_heaphook_file(&p("absent.so")),
        Err(HookFileProblem::Absent)
    );
    std::fs::create_dir(p("dir.so")).expect("mkdir");
    assert_eq!(
        inspect_heaphook_file(&p("dir.so")),
        Err(HookFileProblem::NotARegularFile)
    );
    check("empty.so", b"", Err(HookFileProblem::NotAnElfSharedObject));
    check(
        "text.so",
        b"not a real so",
        Err(HookFileProblem::NotAnElfSharedObject),
    );
    // The name-as-bytes class: the name present as BYTES but not as an export.
    check(
        "rodata.so",
        &elf_with_the_name_only_in_rodata(),
        Err(incomplete(&[], &[])),
    );
    check(
        "import.so",
        &elf_that_imports_the_symbol(),
        Err(incomplete(&[], &[HEAPHOOK_HANDSHAKE_SYMBOL])),
    );
    let one = |name: &'static str, bind: u8, kind: u8| {
        minimal_elf64_so(
            &[FixtureSym {
                name,
                bind,
                kind,
                defined: true,
            }],
            b"",
        )
    };
    check(
        "prefix.so",
        &one("cerulion_heaphook_version2", FX_STB_GLOBAL, FX_STT_FUNC),
        Err(incomplete(&[], &[])),
    );
    check(
        "local.so",
        &one(HEAPHOOK_HANDSHAKE_SYMBOL, FX_STB_LOCAL, FX_STT_FUNC),
        Err(incomplete(&[], &[])),
    );
    check(
        "notype.so",
        &one(HEAPHOOK_HANDSHAKE_SYMBOL, FX_STB_GLOBAL, FX_STT_NOTYPE),
        Err(incomplete(&[], &[])),
    );
    // The older-hook class: a real, well-formed hook that is OLDER — only the
    // handshake exported. Every other required entry point is named.
    check(
        "versiononly.so",
        &elf_exporting_only_the_handshake(),
        Err(incomplete(&[HEAPHOOK_HANDSHAKE_SYMBOL], &[])),
    );
    // Accepted bindings/types for the WHOLE set: weak and object exports
    // count, as `dlsym` resolves them.
    let all_as = |bind: u8, kind: u8| {
        let syms: Vec<FixtureSym> = HEAPHOOK_REQUIRED_EXPORTS
            .iter()
            .copied()
            .map(|name| FixtureSym {
                name,
                bind,
                kind,
                defined: true,
            })
            .collect();
        minimal_elf64_so(&syms, b"")
    };
    check("weak.so", &all_as(FX_STB_WEAK, FX_STT_FUNC), Ok(()));
    check("object.so", &all_as(FX_STB_GLOBAL, FX_STT_OBJECT), Ok(()));
    // A header-only stub: it claims one program header and has none —
    // malformed at the loader's first read.
    let mut header_only = hook_fixture_bytes();
    header_only.truncate(0x40);
    check(
        "headeronly.so",
        &header_only,
        Err(HookFileProblem::MalformedElf(
            "program header table runs past the end of the file",
        )),
    );
    // No PT_LOAD at all — a section-header-only object with the
    // right `.dynsym` that ld.so could never map (a section-header reader would accept it).
    let mut no_phdr = hook_fixture_bytes();
    no_phdr[0x38..0x3a].copy_from_slice(&0u16.to_le_bytes()); // e_phnum = 0
    check(
        "nophdr.so",
        &no_phdr,
        Err(HookFileProblem::NoLoadableSegment),
    );
    // A PT_LOAD that maps only the header and .text, leaving the
    // dynamic tables unmapped.
    let mut not_loaded = hook_fixture_bytes();
    not_loaded[0x40 + 0x20..0x40 + 0x28].copy_from_slice(&0x78u64.to_le_bytes()); // p_filesz
    not_loaded[0x40 + 0x28..0x40 + 0x30].copy_from_slice(&0x78u64.to_le_bytes()); // p_memsz
    check(
        "notloaded.so",
        &not_loaded,
        Err(HookFileProblem::DynamicTablesNotLoadable),
    );
    // e_shnum = 0 is NOT a refusal — that is a stripped
    // object, which ld.so loads and resolves through PT_DYNAMIC, and
    // refusing it would be a false refusal of the user's own artifact. The
    // exports-nothing refusal is driven by the dynamic array itself:
    // no DT_SYMTAB means nothing the loader could resolve by name.
    let mut no_sections = hook_fixture_bytes();
    no_sections[0x3c..0x3e].copy_from_slice(&0u16.to_le_bytes()); // e_shnum = 0
    check("nosections.so", &no_sections, Ok(()));
    let mut no_symtab = hook_fixture_bytes();
    let symtab_tag_at = dyn_tag_value_offset(&no_symtab, FX_DT_SYMTAB) - 8;
    no_symtab[symtab_tag_at..symtab_tag_at + 8].copy_from_slice(&FX_DT_NULL.to_le_bytes());
    check(
        "nosymtab.so",
        &no_symtab,
        Err(HookFileProblem::NoDynamicSymbolTable),
    );
    // ...and so is a shared object with no PT_DYNAMIC at all: it carries
    // no dynamic section, so the loader can resolve nothing from it.
    let mut no_dynamic = hook_fixture_bytes();
    no_dynamic[0x38..0x3a].copy_from_slice(&1u16.to_le_bytes()); // e_phnum = 1 (drop PT_DYNAMIC)
    check(
        "nodynamic.so",
        &no_dynamic,
        Err(HookFileProblem::NoDynamicSymbolTable),
    );
    // A dynamic array naming a symbol table but NO hash table cannot say
    // how long `.dynsym` is; that is a refusal, never a silent empty.
    let mut no_hash = hook_fixture_bytes();
    let hash_tag_at = dyn_tag_value_offset(&no_hash, FX_DT_HASH) - 8;
    no_hash[hash_tag_at..hash_tag_at + 8].copy_from_slice(&FX_DT_STRSZ.to_le_bytes());
    check(
        "nohash.so",
        &no_hash,
        Err(HookFileProblem::NoDynamicSymbolTable),
    );
    // The OTHER machine: a valid hook the loader would drop. Both numbers
    // are the elf.h literals, not the engine's mapping.
    let (host_machine, _) = oracle_host_machine().expect("skipped above");
    let (other_machine, _) = oracle_other_machine().expect("skipped above");
    check(
        "othermachine.so",
        &elf_for_another_machine(),
        Err(HookFileProblem::WrongMachine {
            found: other_machine,
            expected: host_machine,
        }),
    );
    // Shapes the preload cannot be, each refused loudly with what was found.
    let mut elf32 = hook_fixture_bytes();
    elf32[4] = 1; // ELFCLASS32
    assert!(matches!(
        {
            std::fs::write(p("elf32.so"), &elf32).expect("write");
            inspect_heaphook_file(&p("elf32.so"))
        },
        Err(HookFileProblem::UnsupportedElfShape(found)) if found.contains("EI_CLASS=1")
    ));
    let mut big_endian = hook_fixture_bytes();
    big_endian[5] = 2; // ELFDATA2MSB
    assert!(matches!(
        {
            std::fs::write(p("be.so"), &big_endian).expect("write");
            inspect_heaphook_file(&p("be.so"))
        },
        Err(HookFileProblem::UnsupportedElfShape(found)) if found.contains("EI_DATA=2")
    ));
    let mut executable = hook_fixture_bytes();
    executable[0x10] = 2; // ET_EXEC
    assert!(matches!(
        {
            std::fs::write(p("exec.so"), &executable).expect("write");
            inspect_heaphook_file(&p("exec.so"))
        },
        Err(HookFileProblem::UnsupportedElfShape(found)) if found.contains("e_type=2")
    ));
    // Malformed: a section table past the end, a truncated file, and a
    // `.dynsym` whose own header runs past the end.
    // A bogus `e_shoff` is IGNORED, deliberately — ld.so
    // never reads the section header table, so neither does this reader,
    // and a stripped-then-relinked object with a stale field must not be
    // refused for a value nothing consults. The equivalent bound in the
    // world the reader DOES live in is PT_DYNAMIC's own extent.
    let mut bad_shoff = hook_fixture_bytes();
    bad_shoff[0x28..0x30].copy_from_slice(&0x7fff_ffffu64.to_le_bytes());
    check("badshoff.so", &bad_shoff, Ok(()));
    let mut bad_dynamic = hook_fixture_bytes();
    let phdr_off =
        u64::from_le_bytes(bad_dynamic[0x20..0x28].try_into().expect("e_phoff")) as usize;
    let dyn_phdr = phdr_off + 56; // [1] PT_DYNAMIC
    bad_dynamic[dyn_phdr + 0x20..dyn_phdr + 0x28].copy_from_slice(&0x7fff_ffffu64.to_le_bytes()); // p_filesz
    check(
        "baddynamic.so",
        &bad_dynamic,
        Err(HookFileProblem::MalformedElf(
            "the PT_DYNAMIC segment runs past the end of the file",
        )),
    );
    // This was a `matches!(.., MalformedElf(_))`
    // wildcard, which EVERY one of the inspector's 28 distinct
    // `MalformedElf` messages satisfies — and the narrative claimed it cut
    // "inside .dynsym" while the generator maps `p_filesz` over the whole
    // file up to the section table, so the PT_LOAD bound is what actually
    // refuses and the deeper checks were pinned by nothing. Exact oracle,
    // and the `.dynsym` bound gets its own fixtures below.
    let mut truncated = hook_fixture_bytes();
    // Derived from the file's OWN `e_shoff`, not from a literal section
    // count: a generator that grows a section would otherwise silently
    // move this cut into the section header table while the comment went
    // on claiming otherwise.
    let cut = shdr_table_offset(&truncated) - 8; // inside the last body (.shstrtab)
    truncated.truncate(cut);
    check(
        "truncated.so",
        &truncated,
        Err(HookFileProblem::MalformedElf(
            "a PT_LOAD segment runs past the end of the file",
        )),
    );
    // These two fixtures patch the DYNAMIC tables, not section headers. The
    // reader follows PT_DYNAMIC — the route ld.so takes, and the only
    // one a stripped object still has — so the bound is reached through
    // the dynamic tables: a symbol COUNT (from the hash table)
    // whose array overruns the file, and a DT_SYMTAB address outside every
    // PT_LOAD. Patching section headers would change nothing at all,
    // which is precisely why the fixtures do not.
    let mut dynsym_past_end = hook_fixture_bytes();
    let nchain_at = dyn_tag_value_offset(&dynsym_past_end, FX_DT_HASH);
    let hash_off = u64::from_le_bytes(
        dynsym_past_end[nchain_at..nchain_at + 8]
            .try_into()
            .expect("DT_HASH value"),
    ) as usize;
    dynsym_past_end[hash_off + 4..hash_off + 8].copy_from_slice(&0x0010_0000u32.to_le_bytes());
    // The COMPLETE range is checked against
    // the PT_LOAD segment, so a symbol count whose array overruns is
    // refused as NOT LOADABLE — the table would extend past what ld.so
    // maps. Without that check this reaches the file-length read instead,
    // classifying a symbol table from bytes the loader never maps.
    check(
        "dynsym_past_end.so",
        &dynsym_past_end,
        Err(HookFileProblem::DynamicTablesNotLoadable),
    );
    // A PT_LOAD whose MEMORY image is
    // smaller than its FILE image is refused. Dropping `p_memsz`
    // on the argument that `memsz >= filesz` makes the file bound the
    // tighter one assumes exactly the property this shape violates,
    // and bounding tables by `filesz` alone would then classify a symbol
    // table from bytes the loader never maps.
    let mut memsz_short = hook_fixture_bytes();
    let phdr_off =
        u64::from_le_bytes(memsz_short[0x20..0x28].try_into().expect("e_phoff")) as usize;
    let filesz = u64::from_le_bytes(
        memsz_short[phdr_off + 0x20..phdr_off + 0x28]
            .try_into()
            .expect("p_filesz"),
    );
    memsz_short[phdr_off + 0x28..phdr_off + 0x30].copy_from_slice(&(filesz - 1).to_le_bytes());
    check(
        "memsz_short.so",
        &memsz_short,
        Err(HookFileProblem::MalformedElf(
            "a PT_LOAD segment declares less memory than file image",
        )),
    );
    // ...and the anti-tautology control: the SAME fixture with a memory
    // image at least as large as its file image is accepted, so the
    // refusal above is about that relation and not about touching
    // `p_memsz` at all.
    let mut memsz_equal = hook_fixture_bytes();
    memsz_equal[phdr_off + 0x28..phdr_off + 0x30].copy_from_slice(&filesz.to_le_bytes());
    check("memsz_equal.so", &memsz_equal, Ok(()));
    let mut memsz_larger = hook_fixture_bytes();
    memsz_larger[phdr_off + 0x28..phdr_off + 0x30]
        .copy_from_slice(&(filesz + 0x1000).to_le_bytes());
    check("memsz_larger.so", &memsz_larger, Ok(()));
    let mut dynsym_unmapped = hook_fixture_bytes();
    let symtab_at = dyn_tag_value_offset(&dynsym_unmapped, FX_DT_SYMTAB);
    dynsym_unmapped[symtab_at..symtab_at + 8].copy_from_slice(&0x8000_0000u64.to_le_bytes());
    check(
        "dynsym_unmapped.so",
        &dynsym_unmapped,
        Err(HookFileProblem::DynamicTablesNotLoadable),
    );
    // The `.dynstr` bound, reached the same way: DT_STRSZ is the string
    // table's declared length, so an oversized one overruns the file.
    let mut dynstr_past_end = hook_fixture_bytes();
    let strsz_at = dyn_tag_value_offset(&dynstr_past_end, FX_DT_STRSZ);
    let overrun = dynstr_past_end.len() as u64;
    dynstr_past_end[strsz_at..strsz_at + 8].copy_from_slice(&overrun.to_le_bytes());
    check(
        "dynstr_past_end.so",
        &dynstr_past_end,
        Err(HookFileProblem::DynamicTablesNotLoadable),
    );
    // The refusals name what to look for.
    let versiononly = incomplete(&[HEAPHOOK_HANDSHAKE_SYMBOL], &[]).to_string();
    for name in HEAPHOOK_REQUIRED_EXPORTS.iter().copied().skip(1) {
        assert!(versiononly.contains(name), "names `{name}`: {versiononly}");
    }
    assert!(
        !versiononly.contains(&format!("{HEAPHOOK_HANDSHAKE_SYMBOL},"))
            && !versiononly.ends_with(HEAPHOOK_HANDSHAKE_SYMBOL),
        "does not list the one export it HAS as missing: {versiononly}"
    );
    let wrong = HookFileProblem::WrongMachine {
        found: other_machine,
        expected: host_machine,
    }
    .to_string();
    assert!(
        wrong.contains(std::env::consts::ARCH) && wrong.contains("this host is"),
        "names the host: {wrong}"
    );
    // An unmapped host (unreachable through the inspector on a mapped one —
    // the refusal it would produce is pinned by its message).
    let unmapped = HookFileProblem::UnsupportedHost("mips64").to_string();
    assert!(
        unmapped.contains("mips64") && unmapped.contains("x86_64") && unmapped.contains("aarch64"),
        "names the host arch and the mapped ones: {unmapped}"
    );
    // The faithful fixture, directly and through a symlink.
    write_hook_fixture(&p("hook.so"));
    assert_eq!(inspect_heaphook_file(&p("hook.so")), Ok(()));
    #[cfg(unix)]
    {
        std::os::unix::fs::symlink(p("hook.so"), p("link.so")).expect("symlink");
        assert_eq!(
            inspect_heaphook_file(&p("link.so")),
            Ok(()),
            "a symlink to the built hook is the built hook"
        );
    }
}

/// An OLDER / incomplete hook — a real,
/// well-formed shared object exporting only the version handshake — is
/// refused by `--adopt-take` naming every missing entry point and the
/// rebuild remedy; the same object with the complete set is accepted.
/// Through the real gate (env naming the hook's path).
#[test]
#[serial]
fn adopt_take_refuses_an_incomplete_hook_naming_the_missing_exports() {
    skip_unless_host_mapped!("adopt_take_refuses_an_incomplete_hook_naming_the_missing_exports");
    let f = fixture();
    let hook = f.lib_dir.join(HEAPHOOK_FILENAME);
    let hook_str = hook.display().to_string();
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &hook_str);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    std::fs::write(&hook, elf_exporting_only_the_handshake()).expect("write");
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("a hook exporting only the handshake must refuse adoption");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    for name in HEAPHOOK_REQUIRED_EXPORTS.iter().copied().skip(1) {
        assert!(msg.contains(name), "names the missing `{name}`: {msg}");
    }
    assert!(
        msg.contains("cargo build -p cerulion_heaphook"),
        "names the rebuild remedy: {msg}"
    );

    write_hook_fixture(&hook);
    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("the complete export set is accepted");
    assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));
}

/// A valid ELF64 hook built for the OTHER
/// machine is refused naming both machines and this host, through the
/// real gate.
#[test]
#[serial]
fn adopt_take_refuses_a_hook_for_another_machine_naming_both() {
    skip_unless_host_mapped!("adopt_take_refuses_a_hook_for_another_machine_naming_both");
    let f = fixture();
    let hook = f.lib_dir.join(HEAPHOOK_FILENAME);
    let hook_str = hook.display().to_string();
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &hook_str);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    std::fs::write(&hook, elf_for_another_machine()).expect("write");
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("a hook for another machine must refuse adoption");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    let (_, host_name) = oracle_host_machine().expect("skipped above");
    let (_, other_name) = oracle_other_machine().expect("skipped above");
    assert!(msg.contains(other_name), "names the file's machine: {msg}");
    assert!(msg.contains(host_name), "names the host's machine: {msg}");
    assert!(
        msg.contains(std::env::consts::ARCH) && msg.contains("this host is"),
        "names this host: {msg}"
    );
}

/// The engine's `host_elf_machine()` must agree with the hand-written
/// `elf.h` oracle table for the host this binary runs on — number AND name.
/// This is the pin the fixtures alone cannot give: with a fixture stamping
/// what the inspector consults, a wrong row passes both sides.
/// Swapping two rows of the engine's mapping fails HERE (and the
/// accepted-fixture arms, which stamp the literal).
#[test]
fn host_elf_machine_matches_the_elf_h_table() {
    // On a host neither side maps this degrades to `None == None` — a
    // genuine "both unmapped", not a drift.
    assert_eq!(
        host_elf_machine(),
        oracle_host_machine(),
        "host_elf_machine() drifted from the elf.h table for target_arch {}",
        std::env::consts::ARCH
    );
}

/// The platform gate (its oracle sharpened by two
/// later reviews): on any host but Linux/GNU, `--adopt-take`
/// is refused BEFORE a file is looked at. Two observables carry the ORDER:
/// the non-Linux/GNU cases stage a deliberately MALFORMED hook (a gate that
/// ran after inspection would report the ELF refusal, so the host refusal
/// — asserted by the literal prefix the launcher documents — proves the
/// gate answered first), AND the process-wide inspection counter must not
/// move at all across those calls (an inspect-then-DISCARD order
/// still returns the host error, and only the counter sees it). Deltas are
/// read around each call; every direct caller of the inspector in this
/// binary is `#[serial]`, so the delta is exact. Then the anti-tautology
/// halves: the same malformed file on Linux/GNU IS inspected (delta 1) and
/// refused; a valid hook on Linux/GNU is inspected once and accepted; the
/// copy path is neither gated nor inspected (delta 0).
#[test]
#[serial]
fn adopt_take_refuses_a_non_linux_gnu_host_before_inspecting_anything() {
    skip_unless_host_mapped!("adopt_take_refuses_a_non_linux_gnu_host_before_inspecting_anything");
    const HOST_GATE_PREFIX: &str = "--adopt-take requires a Linux/GNU host";
    let f = fixture();
    let hook = f.lib_dir.join(HEAPHOOK_FILENAME);
    // Malformed on purpose: inspection would refuse THIS file loudly, so a
    // host refusal proves the gate ran first.
    std::fs::write(&hook, b"not a real so").expect("write malformed hook");
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    for host in [
        HostAbi {
            os: "macos",
            env: "",
        },
        HostAbi {
            os: "linux",
            env: "musl",
        },
        HostAbi {
            os: "freebsd",
            env: "",
        },
    ] {
        let before = hook_inspections_for_test();
        let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
            Ros2NativeVerb::Run,
            &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
            &f.lib_dir,
            &f.ament_prefix,
            host,
        )
        .expect_err("a non-Linux/GNU host must refuse adoption");
        assert_eq!(
            hook_inspections_for_test() - before,
            0,
            "a non-Linux/GNU host must refuse WITHOUT inspecting any file ({host:?})"
        );
        assert_eq!(classify(&err), EXIT_UNAVAILABLE);
        let msg = err.to_string();
        assert!(
            msg.starts_with(HOST_GATE_PREFIX),
            "the refusal is the HOST gate's, before any inspection: {msg}"
        );
        assert!(
            !msg.contains("is not an ELF shared object") && !msg.contains("no staged preload"),
            "the malformed file must not have been inspected: {msg}"
        );
        assert!(msg.contains(host.os), "names the host os: {msg}");
        assert!(
            msg.contains("linux/gnu"),
            "names the one supported host: {msg}"
        );
        assert!(
            msg.contains(ADOPT_TAKE_FLAG),
            "names the flag to drop: {msg}"
        );
    }
    // The malformed file IS inspected — once — and refused when the host is
    // right: the anti-tautology half, inspection runs, just after the gate.
    let before = hook_inspections_for_test();
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("Linux/GNU inspects the malformed file and refuses it");
    assert_eq!(
        hook_inspections_for_test() - before,
        1,
        "Linux/GNU inspects the one staged entry exactly once"
    );
    assert!(
        err.to_string().contains("is not an ELF shared object"),
        "on Linux/GNU the malformed file is what is refused: {err}"
    );
    // The same staging with a VALID hook on Linux/GNU: inspected once,
    // accepted.
    write_hook_fixture(&hook);
    let before = hook_inspections_for_test();
    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("Linux/GNU accepts the staged hook");
    assert_eq!(hook_inspections_for_test() - before, 1);
    assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));
    // A launch WITHOUT the flag is neither gated nor inspected.
    std::fs::write(&hook, b"not a real so").expect("write malformed hook");
    let before = hook_inspections_for_test();
    let plan = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &args(&["pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi {
            os: "macos",
            env: "",
        },
    )
    .expect("the copy path is not host-gated");
    assert_eq!(
        hook_inspections_for_test() - before,
        0,
        "the copy path inspects nothing"
    );
    assert!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV).is_none());
}

/// The PRODUCTION entry point — the one `cerulion ros2 run|launch` really
/// call — refuses `--adopt-take` on whatever host this test runs on, on
/// BOTH verbs. Its sibling above drives the same refusal through the
/// host-injecting seam; this arm is what proves the wrapper `main.rs` calls
/// carries it too, so the decision cannot be live only behind a seam — and
/// that half needs no ELF oracle, so it runs above the skip, on every arch.
///
/// The second half stages a REAL hook, which is what makes the inspection
/// counter DISCRIMINATING here (unlike the sibling, where nothing is
/// staged and every candidate refusal also inspects nothing): a variant that
/// dropped the refusal and delegated would inspect the staged hook, so `0`
/// is a claim only the decision satisfies. The RETAINED gate is then asserted
/// over the same fixture and still gates on the RUNNING host — accepted on
/// Linux/GNU, refused naming the host anywhere else. Scope: the wrapper's
/// `HostAbi::current()` argument is no longer OBSERVABLE (it is read only
/// inside the adopt branch, which the refusal precedes), so a variant
/// hardcoding `LINUX_GNU` in the wrapper survives; what is pinned is that
/// the retained gate still reads the host it is handed.
#[test]
#[serial]
fn the_default_entry_point_refuses_adoption_while_the_retained_gate_still_host_gates() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    for verb in [Ros2NativeVerb::Run, Ros2NativeVerb::Launch] {
        let err = build_ros2_passthrough_plan(
            verb,
            &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
            &f.lib_dir,
            &f.ament_prefix,
        )
        .expect_err("the production entry refuses --adopt-take");
        assert_eq!(classify(&err), EXIT_UNAVAILABLE, "verb {verb:?}");
        assert!(
            err.to_string().contains("close_fds=True"),
            "names the cause ({verb:?}): {err}"
        );
    }

    skip_unless_host_mapped!(
        "the_default_entry_point_refuses_adoption_while_the_retained_gate_still_host_gates"
    );
    write_hook_fixture(&f.lib_dir.join(HEAPHOOK_FILENAME));
    let before = hook_inspections_for_test();
    let err = build_ros2_passthrough_plan(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
    )
    .expect_err("the production entry refuses --adopt-take");
    assert_eq!(
        hook_inspections_for_test() - before,
        0,
        "refused before the STAGED hook is inspected — so it is the adopt refusal, \
         not a hook verdict"
    );
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);

    // The retained gate over the SAME fixture: still host-gated.
    let before = hook_inspections_for_test();
    let result = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::current(),
    );
    let inspections = hook_inspections_for_test() - before;
    // The runtime verdict must agree with the compile target it was built
    // for (one side is a runtime value, so this is not a constant assert).
    assert_eq!(
        HostAbi::current().supports_adopt_take(),
        cfg!(all(target_os = "linux", target_env = "gnu")),
        "supports_adopt_take must mean linux/gnu"
    );
    if HostAbi::current().supports_adopt_take() {
        let plan = result.expect("Linux/GNU accepts");
        assert_eq!(inspections, 1, "the one staged hook was inspected");
        assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));
    } else {
        let err = result.expect_err("a non-Linux/GNU desk must refuse");
        assert_eq!(inspections, 0, "refused before any inspection on this desk");
        assert_eq!(classify(&err), EXIT_UNAVAILABLE);
        assert!(
            err.to_string().contains(std::env::consts::OS),
            "names this host: {err}"
        );
    }
}

/// A VALID hook staged
/// explicitly from OUTSIDE lib_dir (`CERULION_ROS2_PRELOAD`), with no
/// hook beside the binary, is ACCEPTED — the path the env names IS the
/// hook the loader maps; a gate keyed on
/// `lib_dir/libcerulion_heaphook.so` alone would refuse it. Also: an explicit non-hook `.so`
/// stacked AHEAD of a lib_dir hook still accepts (first-wins finds the
/// hook second), and an explicit file that is NOT the hook with no lib_dir
/// hook refuses naming that file and why.
#[test]
#[serial]
fn adopt_take_accepts_a_valid_hook_staged_explicitly_from_outside_lib_dir() {
    skip_unless_host_mapped!(
        "adopt_take_accepts_a_valid_hook_staged_explicitly_from_outside_lib_dir"
    );
    let f = fixture(); // no lib_dir hook
    let elsewhere = tempfile::tempdir().expect("tempdir");
    let hook = elsewhere.path().join("hook-build-42.so");
    write_hook_fixture(&hook);
    let hook_str = hook.display().to_string();
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &hook_str);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    let before = hook_inspections_for_test();
    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("a valid hook named explicitly is the hook");
    assert_eq!(
        hook_inspections_for_test() - before,
        1,
        "one staged entry, inspected once"
    );
    assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));
    // Decision: by DESCRIPTOR, not by the name — the
    // validation and the load refer to one inode, which is what
    // closes the substitution window. `hook_str` still names the file that was
    // validated; the descriptor identifying it is pinned by
    // `the_validated_hook_is_handed_to_the_child_by_descriptor`.
    assert!(
        env_value(&plan.env, "LD_PRELOAD").is_some_and(|v| v.starts_with("/proc/self/fd/")),
        "the explicit hook is staged, by descriptor: {:?}",
        env_value(&plan.env, "LD_PRELOAD")
    );

    // A non-hook explicit `.so` AHEAD of a lib_dir hook: still accepted.
    let other = elsewhere.path().join("libsanitizer.so");
    std::fs::write(&other, b"not the hook").expect("write");
    write_hook_fixture(&f.lib_dir.join(HEAPHOOK_FILENAME));
    let other_str = other.display().to_string();
    let _p2 = EnvVarGuard::set(ROS2_PRELOAD_ENV, &other_str);
    let before = hook_inspections_for_test();
    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("the lib_dir hook behind an unrelated explicit .so is found");
    assert_eq!(
        hook_inspections_for_test() - before,
        2,
        "first-wins: the unrelated .so, then the hook — two inspections, then stop"
    );
    assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));

    // The same non-hook explicit `.so` with NO lib_dir hook: refused,
    // naming the file and why.
    std::fs::remove_file(f.lib_dir.join(HEAPHOOK_FILENAME)).expect("remove");
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("no staged entry is the hook");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(msg.contains(&other_str), "names the staged file: {msg}");
    assert!(
        msg.contains("is not an ELF shared object"),
        "names why: {msg}"
    );
}

/// A valid hook in the staged
/// list is necessary, not sufficient. The dynamic loader resolves symbols
/// left to right across LD_PRELOAD, so an allocator staged AHEAD of the
/// hook wins `malloc`, the rmw's handshake reads `DegradeForeignAllocator`,
/// and the child would run the copy path behind `--adopt-take` — the same
/// silent-degrade class the other edges refuse. Arms:
/// `foreign.so:hook.so` (the explicit value stacks BEFORE the hook) ⇒
/// refused naming the entry, its allocator exports and the reorder/drop
/// remedy; `hook.so:foreign.so` (the ambient LD_PRELOAD stacks AFTER the
/// hook) ⇒ accepted; a real non-allocator library ahead ⇒ accepted; and
/// the pure scan: the foreign object reports its four exports, the
/// harmless one and the hook fixture report none, a non-ELF and an absent
/// path report none.
#[test]
#[serial]
fn adopt_take_refuses_a_foreign_allocator_staged_ahead_of_the_hook() {
    skip_unless_host_mapped!("adopt_take_refuses_a_foreign_allocator_staged_ahead_of_the_hook");
    const ORDER_PREFIX: &str = "--adopt-take requires the Cerulion heap hook to win malloc";
    let f = fixture();
    write_hook_fixture(&f.lib_dir.join(HEAPHOOK_FILENAME));
    let elsewhere = tempfile::tempdir().expect("tempdir");
    let foreign = elsewhere.path().join("libjemalloc.so");
    std::fs::write(&foreign, foreign_allocator_so()).expect("write");
    let harmless = elsewhere.path().join("libfoo.so");
    std::fs::write(&harmless, harmless_library_so()).expect("write");
    let foreign_str = foreign.display().to_string();

    // The pure scan, first.
    assert_eq!(
        preload_allocator_exports(&foreign).expect("a well-formed allocator is readable"),
        vec!["malloc", "free", "calloc", "realloc"],
        "the foreign allocator reports exactly what it exports, in table order"
    );
    assert_eq!(preload_allocator_exports(&harmless), Ok(Vec::new()));
    assert_eq!(
        preload_allocator_exports(&f.lib_dir.join(HEAPHOOK_FILENAME)),
        Ok(Vec::new())
    );
    let not_elf = elsewhere.path().join("text.so");
    std::fs::write(&not_elf, b"not a real so").expect("write");
    assert_eq!(
        preload_allocator_exports(&not_elf),
        Ok(Vec::new()),
        "a file the loader cannot map wins nothing"
    );
    assert_eq!(
        preload_allocator_exports(&elsewhere.path().join("absent.so")),
        Ok(Vec::new())
    );
    assert!(
        MALLOC_FAMILY_EXPORTS.contains(&"malloc") && MALLOC_FAMILY_EXPORTS.contains(&"free"),
        "the family names the two entry points every allocator interposes"
    );

    // foreign.so AHEAD of the hook (the explicit value stacks first): refused.
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &foreign_str);
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("an allocator staged ahead of the hook must refuse adoption");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(msg.starts_with(ORDER_PREFIX), "the order refusal: {msg}");
    assert!(
        msg.contains(&foreign_str),
        "names the offending entry: {msg}"
    );
    assert!(msg.contains("malloc"), "names the winning export: {msg}");
    assert!(msg.contains("left to right"), "states why: {msg}");
    assert!(msg.contains(ADOPT_TAKE_FLAG), "names the flag: {msg}");
    drop(_p);

    // hook.so:foreign.so (the ambient value stacks AFTER the hook): accepted.
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d2 = EnvVarGuard::set("LD_PRELOAD", &foreign_str);
    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("an allocator AFTER the hook cannot win malloc");
    assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));
    let staged = env_value(&plan.env, "LD_PRELOAD").expect("staged");
    assert!(
        // Decision: the hook leads, by descriptor.
        staged.starts_with("/proc/self/fd/") && staged.ends_with(&foreign_str),
        "hook first, foreign after: {staged}"
    );
    drop(_d2);
    drop(_p);

    // A real non-allocator library ahead of the hook: accepted.
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &harmless.display().to_string());
    let _d3 = EnvVarGuard::unset("LD_PRELOAD");
    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("a non-allocator ahead of the hook takes nothing from it");
    assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));
}

/// A STRIPPED shared object — no section
/// header table at all, PT_DYNAMIC/`.dynsym`/`.dynstr` intact — is read
/// through its dynamic tables, exactly as ld.so reads it. This is the
/// shape a packaged allocator ships in; a section-header reader
/// answers `NoDynamicSymbolTable` for it, which `preload_allocator_exports`
/// would turn into an EMPTY allocator list: a stripped jemalloc staged ahead
/// of the hook would be waved straight through the malloc-order guard.
///
/// Both hash styles are driven, because the count of `.dynsym` entries is
/// not in the dynamic array and must come from one of them — and modern
/// toolchains default to `--hash-style=gnu`, so the GNU walker is the path
/// a real allocator takes.
#[test]
#[serial]
fn a_stripped_shared_object_is_read_through_its_dynamic_tables() {
    skip_unless_host_mapped!("a_stripped_shared_object_is_read_through_its_dynamic_tables");
    let root = tempfile::tempdir().expect("tempdir");
    for (style, tag) in [(FxHashStyle::Sysv, "sysv"), (FxHashStyle::Gnu, "gnu")] {
        let allocator = minimal_elf64_so_with_hash(
            &[
                exported("malloc"),
                exported("free"),
                exported("calloc"),
                exported("realloc"),
            ],
            b"",
            style,
        );
        // Anti-tautology: UNstripped, the same bytes report the same four
        // exports, so the stripped answer below is about the SECTION
        // HEADERS being gone and nothing else.
        let whole = root.path().join(format!("whole_{tag}.so"));
        std::fs::write(&whole, &allocator).expect("write");
        assert_eq!(
            preload_allocator_exports(&whole).expect("a well-formed object is readable"),
            vec!["malloc", "free", "calloc", "realloc"],
            "{tag}: the unstripped object reports its allocator exports"
        );
        let stripped_bytes = strip_section_headers(&allocator);
        assert!(
            stripped_bytes.len() < allocator.len(),
            "{tag}: stripping must actually remove the section header table"
        );
        assert_eq!(
            u16::from_le_bytes(stripped_bytes[0x3c..0x3e].try_into().expect("e_shnum")),
            0,
            "{tag}: and leave the header saying so"
        );
        let stripped = root.path().join(format!("stripped_{tag}.so"));
        std::fs::write(&stripped, &stripped_bytes).expect("write");
        assert_eq!(
            preload_allocator_exports(&stripped).expect("a stripped object is still readable"),
            vec!["malloc", "free", "calloc", "realloc"],
            "{tag}: a stripped allocator must still be seen as an allocator"
        );
        // The same for the HOOK question — one reader, both users: a
        // stripped hook is a valid hook, and refusing it was a false
        // refusal of the user's own artifact.
        let hook = strip_section_headers(&hook_fixture_bytes());
        let hook_path = root.path().join(format!("hook_{tag}.so"));
        std::fs::write(&hook_path, &hook).expect("write");
        assert_eq!(
            inspect_heaphook_file(&hook_path),
            Ok(()),
            "{tag}: a stripped hook still inspects clean"
        );
    }
}

/// An entry the loader WOULD map but whose
/// exports the launcher cannot enumerate is UNKNOWN, and unknown is not
/// "no allocator". The two failure classes are told apart: a shape ld.so
/// itself drops reports no allocator (it cannot win `malloc`), while an
/// unreadable-but-loadable object is an `Err` the plan builder refuses on.
#[test]
#[serial]
fn an_unreadable_preload_is_refused_rather_than_reported_harmless() {
    skip_unless_host_mapped!("an_unreadable_preload_is_refused_rather_than_reported_harmless");
    let root = tempfile::tempdir().expect("tempdir");
    // A loadable object whose dynamic array names no symbol table: ld.so
    // maps it happily, and nothing here can say what it exports.
    let mut blind = foreign_allocator_so();
    let symtab_at = dyn_tag_value_offset(&blind, FX_DT_SYMTAB);
    blind[symtab_at - 8..symtab_at].copy_from_slice(&FX_DT_NULL.to_le_bytes());
    let blind_path = root.path().join("blind.so");
    std::fs::write(&blind_path, &blind).expect("write");
    assert_eq!(
        preload_allocator_exports(&blind_path),
        Err(HookFileProblem::NoDynamicSymbolTable),
        "unknown exports must be an Err, never an empty allocator list"
    );
    // A shape ld.so drops: not this host's machine. It cannot win malloc,
    // so it is correctly reported as no allocator — the distinction the
    // stripped-object fix rests on.
    let wrong = root.path().join("wrong_machine.so");
    std::fs::write(&wrong, elf_for_another_machine()).expect("write");
    assert_eq!(
        preload_allocator_exports(&wrong),
        Ok(Vec::new()),
        "a file the loader will not map cannot win malloc"
    );

    // And at the PLAN level: staged AHEAD of the hook, the unreadable
    // entry is a loud exit-69 refusal naming it and why.
    let f = fixture();
    write_hook_fixture(&f.lib_dir.join(HEAPHOOK_FILENAME));
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &blind_path.display().to_string());
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("an entry whose exports cannot be read must not be waved through");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(
        msg.contains(ORDER_GUARD_SUBSTR),
        "the ORDER guard refused: {msg}"
    );
    assert!(
        msg.contains("cannot read its exported symbols"),
        "names why it could not decide: {msg}"
    );
    assert!(msg.contains("blind.so"), "names the entry: {msg}");
}

/// A host the launcher has no `e_machine`
/// row for is refused as `UnsupportedHost` BEFORE any ELF parse — driven
/// through the `inspect_heaphook_file_for_machine` seam with `None`, which
/// is exactly what `host_elf_machine()` returns there, so this runs on
/// every desk and never needs a real cross-target build. The file is a
/// hand-written 64-byte ELF64/LE/ET_DYN header (no generator, no oracle
/// row needed), so the arm is cfg-independent; the refusal names the host
/// arch and the mapped ones, and it still counts as an inspection.
#[test]
#[serial]
fn an_unmapped_host_is_refused_before_any_elf_parse() {
    // The file is FOUR bytes — the ELF magic
    // and nothing else — so any implementation that measures or parses the
    // header before consulting the host mapping answers `MalformedElf`,
    // and only a host check that precedes every parse answers
    // `UnsupportedHost`. The mapped-host control on the SAME bytes reaches
    // the parse error, which is the anti-tautology half.
    let root = tempfile::tempdir().expect("tempdir");
    let path = root.path().join("hook.so");
    std::fs::write(&path, b"\x7fELF").expect("write");
    let before = hook_inspections_for_test();
    assert_eq!(
        inspect_heaphook_file_for_machine(&path, None),
        Err(HookFileProblem::UnsupportedHost(std::env::consts::ARCH)),
        "no host mapping ⇒ UnsupportedHost BEFORE the header is even measured"
    );
    assert_eq!(
        hook_inspections_for_test() - before,
        1,
        "it is an inspection"
    );
    let text = HookFileProblem::UnsupportedHost(std::env::consts::ARCH).to_string();
    assert!(
        text.contains(std::env::consts::ARCH)
            && text.contains("x86_64")
            && text.contains("aarch64"),
        "names the host arch and the mapped ones: {text}"
    );
    assert_eq!(
        inspect_heaphook_file_for_machine(&path, Some((0xffff, "any"))),
        Err(HookFileProblem::MalformedElf(
            "shorter than an ELF64 header"
        )),
        "with a mapping the same bytes reach the parse and fail it"
    );
}

/// Restores the process cwd on drop (the bare-soname arm must run with the
/// launcher's cwd pointed at a directory holding the bare name).
struct CwdGuard(std::path::PathBuf);
impl CwdGuard {
    fn enter(dir: &std::path::Path) -> Self {
        let prior = std::env::current_dir().expect("cwd");
        std::env::set_current_dir(dir).expect("chdir");
        Self(prior)
    }
}
impl Drop for CwdGuard {
    fn drop(&mut self) {
        let _ = std::env::set_current_dir(&self.0);
    }
}

/// An explicit preload staged AHEAD of the hook
/// as a BARE soname (no `/`) or a path with a loader token is refused for
/// `--adopt-take`: ld.so resolves those through the library search path /
/// token expansion, so the object that really loads ahead of the hook may
/// not be the file the launcher can see. The bare arm plants a harmless
/// library under the bare name in the launcher's cwd (the only way a bare
/// name passes the existence check), so bareness is the ONLY reason it is
/// refused — the SAME file by ABSOLUTE path is accepted in the next
/// block, so bareness is provably the only reason, and a variant that
/// drops the check accepts the bare one too. To keep the `$` legs from
/// going vacuous, they plant that same library behind a directory
/// literally NAMED `$LIB` (then `${ORIGIN}`), so the entry EXISTS and the
/// generic missing-preload check cannot answer for the guard — which is
/// also the dangerous shape, the launcher able to stat a file ld.so will
/// never map there. A `$` path with nothing behind it stays covered
/// separately, and the two refusals are asserted apart.
#[test]
#[serial]
fn adopt_take_refuses_a_bare_soname_or_loader_token_staged_ahead_of_the_hook() {
    skip_unless_host_mapped!(
        "adopt_take_refuses_a_bare_soname_or_loader_token_staged_ahead_of_the_hook"
    );
    let f = fixture();
    write_hook_fixture(&f.lib_dir.join(HEAPHOOK_FILENAME));
    let cwd = tempfile::tempdir().expect("tempdir");
    // This arm discriminates on `$`, so its own scratch path must be
    // `$`-free or the acceptance control below refuses for the wrong
    // reason and points at the wrong thing.
    assert!(
        !cwd.path().to_string_lossy().contains('$'),
        "TMPDIR must be $-free for this arm: {}",
        cwd.path().display()
    );
    std::fs::write(cwd.path().join("libfoo.so"), harmless_library_so()).expect("write");
    let _cwd = CwdGuard::enter(cwd.path());
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, "libfoo.so");
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("a bare soname ahead of the hook cannot be inspected");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(
        msg.contains(ORDER_GUARD_SUBSTR),
        "the ORDER guard refused: {msg}"
    );
    assert!(msg.contains("bare soname"), "names why: {msg}");
    assert!(msg.contains("libfoo.so"), "names the entry: {msg}");
    assert!(msg.contains("absolute path"), "names the remedy: {msg}");
    // The SAME library by absolute path is inspected and, being harmless,
    // accepted — bareness was the only problem.
    let abs = cwd.path().join("libfoo.so").display().to_string();
    let _p2 = EnvVarGuard::set(ROS2_PRELOAD_ENV, &abs);
    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("the same harmless library by absolute path is fine");
    assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));
    // The `$` arm has to REACH
    // the order guard. `$LIB` is a perfectly legal directory NAME, so a
    // literal `<cwd>/$LIB/libfoo.so` exists on disk and the generic
    // missing-preload existence check — which runs FIRST — cannot answer
    // in the guard's place. That is also exactly the hazard: the launcher
    // stats a real file while ld.so expands the token and maps whatever
    // sits at the expanded path ahead of the hook — a different object, or
    // (as here) nothing at all. The library planted there is the SAME
    // harmless one accepted by absolute path just above, so the `$` is the
    // ONLY reason this is refused: a variant that drops
    // `raw.contains('$')` scans it, finds no allocator, and accepts.
    //
    // BOTH spellings are driven: ld.so expands `${…}` as readily as the
    // bare form, so a narrowing to the three NAMED tokens — which passes
    // the `$LIB` leg — must still be caught.
    for token in ["$LIB", "${ORIGIN}"] {
        let token_dir = cwd.path().join(token);
        std::fs::create_dir_all(&token_dir)
            .unwrap_or_else(|e| panic!("mkdir a directory literally named {token}: {e}"));
        let token_path = token_dir.join("libfoo.so");
        std::fs::write(&token_path, harmless_library_so()).expect("write");
        assert!(
            token_path.exists(),
            "the {token} path must exist LITERALLY, or the missing-preload check answers instead"
        );
        let staged = token_path.display().to_string();
        let _p3 = EnvVarGuard::set(ROS2_PRELOAD_ENV, &staged);
        let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
            Ros2NativeVerb::Run,
            &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
            &f.lib_dir,
            &f.ament_prefix,
            HostAbi::LINUX_GNU,
        )
        .expect_err("a `$` path ahead of the hook cannot be inspected");
        assert_eq!(classify(&err), EXIT_UNAVAILABLE);
        let msg = err.to_string();
        assert!(
            msg.contains(ORDER_GUARD_SUBSTR),
            "the ORDER guard refused: {msg}"
        );
        assert!(msg.contains("path containing `$`"), "names why: {msg}");
        // The ENTRY, not the token: the production `why` clause names
        // `$LIB`/`$ORIGIN`/`$PLATFORM` literally, so asserting a bare
        // token here would be satisfied by the explanation whatever
        // `{raw}` rendered — a vacuity of exactly the kind this arm exists
        // to close.
        assert!(msg.contains(&staged), "names the staged entry: {msg}");
        assert!(msg.contains("expands"), "names the loader behaviour: {msg}");
        assert!(
            msg.contains("no `$` in it"),
            "the remedy is `$`-specific, not the bare arm's `absolute path`: {msg}"
        );
        assert!(
            !msg.contains(PRELOAD_MISSING_SUBSTR),
            "the ORDER guard refused, not the generic missing-preload check: {msg}"
        );
    }
    // The generic missing-path behaviour stays covered SEPARATELY, and the
    // two are told APART: a `$` path with no literal file behind it is the
    // missing-preload error and never reaches the order guard.
    let _p4 = EnvVarGuard::set(ROS2_PRELOAD_ENV, "$ORIGIN/libabsent.so");
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("a token path with nothing behind it is a missing preload");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(
        msg.contains(PRELOAD_MISSING_SUBSTR) && msg.contains("$ORIGIN/libabsent.so"),
        "the generic existence check names the entry: {msg}"
    );
    assert!(
        !msg.contains(ORDER_GUARD_SUBSTR),
        "it is the missing-preload error, not the order guard's: {msg}"
    );
}

/// The rmw arms adoption from
/// `CERULION_RMW_ADOPT_TAKE` alone, so an AMBIENT arming value in the
/// launcher's environment would arm the child WITHOUT the flag's host,
/// hook and preload-order checks — the launcher reporting a copy-path
/// launch while the child adopts or degrades on its own. Without the flag
/// an arming value is a usage conflict (exit 2) naming the variable and
/// the flag; `0` and empty disarm the rmw and pass; WITH the flag the
/// launcher's own `1` and its checks apply.
#[test]
#[serial]
fn an_ambient_adopt_take_env_without_the_flag_is_a_usage_conflict() {
    skip_unless_host_mapped!("an_ambient_adopt_take_env_without_the_flag_is_a_usage_conflict");
    let f = fixture();
    write_hook_fixture(&f.lib_dir.join(HEAPHOOK_FILENAME));
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let _a = EnvVarGuard::set(ADOPT_TAKE_CHILD_ENV, "1");
    let err = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Run,
        &args(&["pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("an ambient arming value without the flag is refused");
    assert_eq!(classify(&err), EXIT_USAGE);
    let msg = err.to_string();
    assert!(
        msg.contains(ADOPT_TAKE_CHILD_ENV) && msg.contains(ADOPT_TAKE_FLAG),
        "{msg}"
    );
    // The REMEDY, paired positive + negative: since the launcher-refusal rule
    // the flag is refused on every path, so a message pointing the reader
    // back at it is a dead end. The old wording ("…or launch through
    // `cerulion ros2 run|launch --adopt-take`") satisfies the two substring
    // checks above, which is why the remedy needs its own pin.
    assert!(
        msg.contains("DIRECTLY"),
        "the remedy must be the direct launch: {msg}"
    );
    assert!(
        !msg.contains("launch through `cerulion ros2"),
        "and must not send the reader back to a verb that always refuses: {msg}"
    );
    drop(_a);
    for disarmed in ["0", ""] {
        let _a = EnvVarGuard::set(ADOPT_TAKE_CHILD_ENV, disarmed);
        let plan = build_ros2_passthrough_plan_for_host(
            Ros2NativeVerb::Run,
            &args(&["pkg", "exe"]),
            &f.lib_dir,
            &f.ament_prefix,
            HostAbi::LINUX_GNU,
        )
        .expect("a disarming value is not a conflict");
        assert!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV).is_none());
    }
    let _a = EnvVarGuard::set(ADOPT_TAKE_CHILD_ENV, "1");
    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("with the flag the launcher's own checks apply and it arms");
    assert_eq!(env_value(&plan.env, ADOPT_TAKE_CHILD_ENV), Some("1"));
}

/// The ambient-value refusal covers the
/// GRAPH path too, the path with no other gate.
///
/// The flag's host/hook/preload-order checks live on
/// `build_ros2_passthrough_plan_for_host` — but a
/// `ros2:` graph entry never reaches that function. It stages its child
/// through `stage_base_child_env` directly, has no flag to pass, and
/// would otherwise inherit `CERULION_RMW_ADOPT_TAKE` unchecked: the child would
/// arm adoption with none of the checks having run, on a host that may not
/// support it and with no heap hook staged. Choosing the other launch verb
/// would bypass a safety gate.
///
/// Driven through the graph path's own entry point, so it fails if the gate
/// is ever moved back behind the passthrough builder.
#[test]
#[serial]
fn a_graph_child_refuses_an_ambient_adopt_take_env() {
    let f = fixture();
    write_hook_fixture(&f.lib_dir.join(HEAPHOOK_FILENAME));
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");

    // The graph path answers `NotRun` — it has no flag to pass.
    let _a = EnvVarGuard::set(ADOPT_TAKE_CHILD_ENV, "1");
    let err = stage_base_child_env(&f.lib_dir, &f.ament_prefix, AdoptTakeGate::NotRun)
        .expect_err("a graph child must not inherit an ambient arming value");
    assert_eq!(classify(&err), EXIT_USAGE);
    let msg = err.to_string();
    assert!(
        msg.contains(ADOPT_TAKE_CHILD_ENV) && msg.contains(ADOPT_TAKE_FLAG),
        "the refusal names the variable and the flag: {msg}"
    );
    // The REMEDY, paired positive + negative: since the launcher-refusal rule
    // the flag is refused on every path, so a message pointing the reader
    // back at it is a dead end. The old wording ("…or launch through
    // `cerulion ros2 run|launch --adopt-take`") satisfies the two substring
    // checks above, which is why the remedy needs its own pin.
    assert!(
        msg.contains("DIRECTLY"),
        "the remedy must be the direct launch: {msg}"
    );
    assert!(
        !msg.contains("launch through `cerulion ros2"),
        "and must not send the reader back to a verb that always refuses: {msg}"
    );

    // The disarming values still pass — this gate must not break a normal
    // graph run whose environment happens to carry the variable.
    drop(_a);
    for disarmed in ["0", ""] {
        let _a = EnvVarGuard::set(ADOPT_TAKE_CHILD_ENV, disarmed);
        let env = stage_base_child_env(&f.lib_dir, &f.ament_prefix, AdoptTakeGate::NotRun)
            .expect("a disarming value is not a conflict");
        assert!(env_value(&env, ADOPT_TAKE_CHILD_ENV).is_none());
    }

    // ANTI-TAUTOLOGY: the same ambient value with the gate reported as RUN
    // is accepted — the refusal keys on the gate, not on the variable, so a
    // stager that refused unconditionally would break `--adopt-take` itself.
    let _a = EnvVarGuard::set(ADOPT_TAKE_CHILD_ENV, "1");
    stage_base_child_env(&f.lib_dir, &f.ament_prefix, AdoptTakeGate::Ran)
        .expect("with the launch gate run, the launcher arms the child itself");
}

/// An allocator exporting its family as
/// `STT_GNU_IFUNC` is still an allocator, and must be refused ahead of the
/// hook.
///
/// A scan accepting only `STT_FUNC` and `STT_OBJECT` is blind to an IFUNC export —
/// the shape an optimized build picks so a resolver can choose a
/// CPU-specific implementation. `ld.so` resolves an
/// IFUNC exactly as it resolves a plain function, so such a library staged
/// ahead of the hook would win `malloc` while the guard reported no allocator at
/// all, and adoption would degrade to the copy path behind a flag that promised
/// it. That is precisely the outcome this guard exists to refuse.
#[test]
#[serial]
fn an_ifunc_allocator_staged_ahead_of_the_hook_is_refused() {
    skip_unless_host_mapped!("an_ifunc_allocator_staged_ahead_of_the_hook_is_refused");
    const ORDER_PREFIX: &str = "--adopt-take requires the Cerulion heap hook to win malloc";
    let f = fixture();
    write_hook_fixture(&f.lib_dir.join(HEAPHOOK_FILENAME));
    let elsewhere = tempfile::tempdir().expect("tempdir");
    let ifunc = elsewhere.path().join("libifunc_malloc.so");
    std::fs::write(&ifunc, ifunc_allocator_so()).expect("write");

    // The pure scan first: the family is REPORTED, not silently empty.
    assert_eq!(
        preload_allocator_exports(&ifunc).expect("a well-formed IFUNC allocator is readable"),
        vec!["malloc", "free"],
        "an IFUNC export is a function export — the loader treats it as one"
    );

    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &ifunc.display().to_string());
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("an IFUNC allocator ahead of the hook must be refused");
    let msg = err.to_string();
    assert!(
        msg.contains(ORDER_PREFIX) && msg.contains("libifunc_malloc.so"),
        "the order guard names it: {msg}"
    );

    // ANTI-TAUTOLOGY: the same IFUNC shape on a NON-allocator name is not an
    // allocator, so the refusal keys on the symbol family and not on the
    // symbol type.
    let benign = elsewhere.path().join("libifunc_benign.so");
    std::fs::write(
        &benign,
        minimal_elf64_so(
            &[FixtureSym {
                name: "libfoo_run",
                bind: FX_STB_GLOBAL,
                kind: FX_STT_GNU_IFUNC,
                defined: true,
            }],
            b"",
        ),
    )
    .expect("write");
    assert_eq!(preload_allocator_exports(&benign), Ok(Vec::new()));
}

/// The preload list is split the way
/// `ld.so` splits it — on SPACES as well as colons.
///
/// glibc's `handle_preload_list` is `strsep(&list, " :")`, and `ld.so(8)`
/// says the items "can be separated by spaces or colons". This guard's whole
/// job is that the entries it inspects are the entries the loader will load,
/// so a colon-only split is a divergence with a name.
///
/// The reachable shape is a path that CONTAINS a space and EXISTS — the
/// launcher existence-checks the staged value, so a merely-bogus spaced
/// string is refused earlier as a missing path. Here `<dir>/liba.so extra.so`
/// is a real file, so it passes that check, and a colon-only split inspects it as ONE
/// harmless entry — while `ld.so` loads `<dir>/liba.so` (a real allocator)
/// and `extra.so`, and the allocator wins `malloc` ahead of the hook,
/// uninspected.
#[test]
#[serial]
fn a_spaced_preload_path_is_split_the_way_the_loader_splits_it() {
    skip_unless_host_mapped!("a_spaced_preload_path_is_split_the_way_the_loader_splits_it");
    const ORDER_PREFIX: &str = "--adopt-take requires the Cerulion heap hook to win malloc";
    let f = fixture();
    write_hook_fixture(&f.lib_dir.join(HEAPHOOK_FILENAME));
    let elsewhere = tempfile::tempdir().expect("tempdir");

    // The token `ld.so` would load FIRST: a real allocator.
    let first_token = elsewhere.path().join("liba.so");
    std::fs::write(&first_token, foreign_allocator_so()).expect("write");
    // The staged value itself: a REAL file whose path contains a space, so
    // the launcher's existence check passes and the entry is staged.
    let spaced = elsewhere.path().join("liba.so extra.so");
    std::fs::write(&spaced, harmless_library_so()).expect("write");
    assert!(spaced.exists(), "the staged path must really exist");

    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &spaced.display().to_string());
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("an allocator hidden behind a space must still be refused");
    let msg = err.to_string();
    assert!(
        msg.contains(ORDER_PREFIX),
        "the ORDER guard must be what refuses (not the missing-path arm): {msg}"
    );
    // THE DISCRIMINATOR. Both order-family refusals share that prefix, and
    // `liba.so` is a SUBSTRING of the un-split path — so neither of those
    // alone can tell the split apart from the colon-only one; the older
    // arms pass without this discriminator, which is why this arm exists.
    // Pin the ALLOCATOR arm specifically, and pin that the entry it names is
    // the FIRST TOKEN and not the whole spaced string.
    assert!(
        msg.contains("exports"),
        "the allocator arm must be what refuses — a 'cannot read its exported symbols' \
         refusal names the same file and proves nothing about the split: {msg}"
    );
    assert!(
        msg.contains(&format!("`{}`", first_token.display())),
        "it names the FIRST space-delimited token, which is what the loader loads: {msg}"
    );
    assert!(
        !msg.contains("liba.so extra.so"),
        "and NOT the un-split whole — naming that is exactly the colon-only behaviour: {msg}"
    );
}

/// An entry too large for THIS
/// launcher's read budget is UNKNOWN, never "no allocator".
///
/// `HEAPHOOK_INSPECT_LIMIT` is our own inspection ceiling, not something
/// `ld.so` refuses — the loader maps a >64 MiB `.so` perfectly well. Folding
/// `TooLarge` into the cannot-map bucket meant an unstripped allocator build
/// (a debug `libtcmalloc`, say) staged ahead of the hook was waved through as
/// carrying no allocator, won `malloc` left-to-right, and put the run on the
/// copy path behind a flag that promised adoption — the exact silent degrade
/// this guard's own doc comment says it exists to prevent.
///
/// The oversized file is SPARSE (`set_len`), so this costs no I/O: the size
/// is read from the metadata and the bytes are never touched.
#[test]
#[serial]
fn an_oversized_preload_entry_is_unknown_not_no_allocator() {
    let root = tempfile::tempdir().expect("tempdir");
    let big = root.path().join("libhuge.so");
    let f = std::fs::File::create(&big).expect("create");
    f.set_len(HEAPHOOK_INSPECT_LIMIT + 1).expect("sparse grow");
    drop(f);
    match preload_allocator_exports(&big) {
        Err(HookFileProblem::TooLarge(len)) => {
            assert_eq!(len, HEAPHOOK_INSPECT_LIMIT + 1, "the size is reported")
        }
        other => panic!("an unreadable-because-too-large entry must be UNKNOWN, got {other:?}"),
    }

    // ANTI-TAUTOLOGY, and the boundary from the other side: exactly AT the
    // ceiling is still inspectable, so it must NOT report TooLarge. (It is
    // not an ELF, so the correct answer is "no allocator".)
    let at = root.path().join("libat.so");
    let f = std::fs::File::create(&at).expect("create");
    f.set_len(HEAPHOOK_INSPECT_LIMIT).expect("sparse grow");
    drop(f);
    assert_eq!(
        preload_allocator_exports(&at).expect("at the ceiling is inspectable"),
        Vec::<&'static str>::new(),
        "a non-ELF file at the ceiling exports no allocator"
    );

    // ...and the SAME misclassification existed on the staged-AHEAD path,
    // which is where it actually bites: an oversized entry ahead of the hook
    // in LD_PRELOAD must REFUSE the launch, not be waved through as carrying
    // no allocator.
    let f = fixture();
    write_hook_fixture(&f.lib_dir.join(HEAPHOOK_FILENAME));
    let ahead = f.lib_dir.join("libahead.so");
    let fh = std::fs::File::create(&ahead).expect("create");
    fh.set_len(HEAPHOOK_INSPECT_LIMIT + 1).expect("sparse grow");
    drop(fh);
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &ahead.display().to_string());
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let err = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect_err("an entry the launcher cannot enumerate must refuse, never be waved through");
    let msg = err.to_string();
    assert!(
        msg.contains("libahead.so"),
        "the refusal names the offending entry: {msg}"
    );
}

/// A FIFO in `LD_PRELOAD` must not
/// hang the launcher.
///
/// The inspection opens each staged entry BEFORE it can know the entry is
/// not a regular file — deliberately, because the identity check requires the bytes
/// and the identity to come from one descriptor. A plain `O_RDONLY` open of
/// a writer-less FIFO blocks forever per POSIX, and `LD_PRELOAD` is
/// environment-controlled, so `cerulion ros2 run --adopt-take …` would hang
/// with no output at all. `O_NONBLOCK` returns immediately and the `fstat`
/// then refuses it as non-regular.
///
/// The oracle is TERMINATION, so the work runs on a thread with a deadline —
/// a hang has no value to assert on, only a wall to exceed.
#[test]
#[serial]
fn a_fifo_staged_in_ld_preload_does_not_hang_the_launcher() {
    let f = fixture();
    write_hook_fixture(&f.lib_dir.join(HEAPHOOK_FILENAME));
    let fifo = f.lib_dir.join("libfifo.so");
    let c = std::ffi::CString::new(fifo.as_os_str().as_encoded_bytes()).expect("cstring");
    // SAFETY: a fresh path inside this test's own tempdir.
    assert_eq!(unsafe { libc::mkfifo(c.as_ptr(), 0o644) }, 0, "mkfifo");
    assert!(
        std::fs::symlink_metadata(&fifo)
            .expect("stat")
            .file_type()
            .is_fifo(),
        "precondition: the staged entry really is a FIFO with no writer"
    );

    let (tx, rx) = std::sync::mpsc::channel();
    let lib_dir = f.lib_dir.clone();
    let ament = f.ament_prefix.clone();
    let fifo_s = fifo.display().to_string();
    std::thread::spawn(move || {
        let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, &fifo_s);
        let _d = EnvVarGuard::unset("LD_PRELOAD");
        let r = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
            Ros2NativeVerb::Run,
            &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
            &lib_dir,
            &ament,
            HostAbi::LINUX_GNU,
        );
        let _ = tx.send(r.is_ok());
    });
    rx.recv_timeout(std::time::Duration::from_secs(20))
        .expect("the launcher must not block on a writer-less FIFO in LD_PRELOAD");
}

/// The recorded identity must change
/// when only the WHOLE-SECOND mtime changes.
///
/// A `FileId` carrying `mtime_nsec()` alone — the sub-second remainder — never
/// sees the whole-second component. On a filesystem with 1-second
/// granularity that field is permanently 0 and contributes nothing, and an
/// in-place same-size rewrite of the hook landing in a different second has
/// the same `dev`, `ino`, `size` AND the same recorded time: the exec-time
/// re-check waves it through. That rewrite is the attack the identity exists
/// to catch.
///
/// Same file, same size, same inode — only the second moves.
#[test]
#[serial]
fn the_recorded_identity_changes_when_only_the_whole_second_mtime_moves() {
    let root = tempfile::tempdir().expect("tempdir");
    let path = root.path().join("libsame.so");
    std::fs::write(&path, b"same size, same inode, different second").expect("write");
    let before = file_identity(&path).expect("identity");

    let file = std::fs::OpenOptions::new()
        .write(true)
        .open(&path)
        .expect("open for set_times");
    let meta = std::fs::metadata(&path).expect("stat");
    let moved = meta.modified().expect("mtime") + std::time::Duration::from_secs(1);
    file.set_times(std::fs::FileTimes::new().set_modified(moved))
        .expect("set_times");
    drop(file);

    let after = file_identity(&path).expect("identity");
    assert_eq!(
        (before.0, before.1, before.2),
        (after.0, after.1, after.2),
        "precondition: dev, ino and size are unchanged — the second is the ONLY difference"
    );
    assert_ne!(
        before, after,
        "a one-second mtime move must change the identity; with only the sub-second \
         remainder recorded it does not, and an in-place same-size rewrite passes the \
         exec-time re-check"
    );
}

/// The launcher's required export list is pinned to BOTH sides it stands
/// in for: it must equal, as a set, every `c"cerulion_heaphook_…"` name
/// `rmw_cerulion/src/heaphook.rs` resolves by `dlsym` (a symbol the rmw
/// starts resolving that the launcher does not require would re-open the
/// older-hook hole; one the launcher requires that the rmw never resolves
/// would refuse a working hook), and every entry must be exported by
/// `cerulion_heaphook/src/exports.rs`. The handshake symbol is the list's
/// first entry. A rename or addition on either side fails HERE.
#[test]
fn the_required_export_list_matches_what_the_rmw_resolves_and_the_hook_exports() {
    use std::collections::BTreeSet;
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
    let rmw = std::fs::read_to_string(root.join("rmw_cerulion/src/heaphook.rs"))
        .expect("read rmw_cerulion/src/heaphook.rs");
    // Every `c"cerulion_heaphook_<name>"` literal — the exact spelling the
    // rmw hands to `dlsym`.
    let mut resolved = BTreeSet::new();
    let mut rest = rmw.as_str();
    while let Some(at) = rest.find("c\"cerulion_heaphook_") {
        let literal = &rest[at + 2..];
        let end = literal.find('"').expect("a closed c-string literal");
        resolved.insert(literal[..end].to_string());
        rest = &literal[end..];
    }
    let required: BTreeSet<String> = HEAPHOOK_REQUIRED_EXPORTS
        .iter()
        .map(|name| name.to_string())
        .collect();
    assert_eq!(
        required, resolved,
        "HEAPHOOK_REQUIRED_EXPORTS must equal the set of symbols the rmw resolves \
         (left: the launcher's list; right: rmw_cerulion/src/heaphook.rs)"
    );
    assert_eq!(
        HEAPHOOK_REQUIRED_EXPORTS[0], HEAPHOOK_HANDSHAKE_SYMBOL,
        "the handshake symbol is the list's first entry"
    );
    let hook = std::fs::read_to_string(root.join("cerulion_heaphook/src/exports.rs"))
        .expect("read cerulion_heaphook/src/exports.rs");
    // `fn {name}(` is satisfied by a function
    // that is not EXPORTED at all. Rust mangles a plain `pub extern "C"
    // fn`, so dropping `#[no_mangle]` leaves this file unchanged in every
    // way this assertion could see while the C symbol `dlsym` resolves
    // disappears — and the hand-built fixtures cannot catch it either,
    // since they derive their symbol names from this same list. Pin the
    // whole export SHAPE: the attribute immediately above the definition.
    let lines: Vec<&str> = hook.lines().collect();
    for name in HEAPHOOK_REQUIRED_EXPORTS {
        let needle = format!("extern \"C\" fn {name}(");
        let at = lines
            .iter()
            .position(|l| !l.trim_start().starts_with("//") && l.contains(&needle))
            .unwrap_or_else(|| {
                panic!("the hook must define `{name}` as an `extern \"C\"` function")
            });
        // Walk back over the definition's own attribute block (attributes
        // and doc comments only — the first line that is neither ends it).
        let no_mangle = lines[..at].iter().rev().take_while(|l| {
            let t = l.trim_start();
            t.starts_with('#') || t.starts_with("///") || t.starts_with("//") || t.is_empty()
        });
        assert!(
            no_mangle.clone().any(|l| l.trim() == "#[no_mangle]"),
            "`{name}` must carry #[no_mangle] on its own attribute block — without it Rust \
             MANGLES the symbol, the rmw's dlsym for `{name}` finds nothing, and this file \
             still reads exactly as if it exported it"
        );
    }
}

/// The REAL built hook — Linux only (the hook is an ELF `.so`). The
/// artifact is a REQUIREMENT, not a skip: returning
/// normally when neither `target/debug` nor `target/release`
/// holds the artifact would let a missing or broken build pass without
/// touching the production `.so` at all, and taking whichever profile
/// exists FIRST would let a stale opposite-profile artifact stand in for
/// the one under test. It selects the artifact for the profile this
/// test is RUNNING in (honouring `CARGO_TARGET_DIR`) and fails loudly,
/// naming the profile-specific build command, when it is absent. Linux CI
/// runs `cargo build --workspace` before every test step, so the artifact
/// is there; a bare `cargo test` on a fresh checkout is told what to run.
///
/// It also asserts the EXPORTED symbols directly (the
/// other half of the same requirement): the source-shape oracle above cannot see a
/// linker or build-script change that drops a symbol, and the hand-built
/// fixtures derive their names from the same list the launcher checks, so
/// the real artifact's dynamic symbol table is the only independent
/// witness that what ships is what `dlsym` will find.
#[cfg(target_os = "linux")]
#[test]
#[serial]
fn the_real_built_hook_exports_every_required_symbol() {
    let profile = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };
    let target = std::env::var_os("CARGO_TARGET_DIR").map(std::path::PathBuf::from);
    let target = target.unwrap_or_else(|| {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("target")
    });
    let real = target.join(profile).join(HEAPHOOK_FILENAME);
    assert!(
        real.exists(),
        "no built {HEAPHOOK_FILENAME} for the `{profile}` profile at {} — run \
         `cargo build{} -p cerulion_heaphook`. This arm is the ONLY check that the \
         shipped artifact, rather than a hand-built fixture, exports what the rmw \
         resolves; skipping it when the file is missing made a broken build look green.",
        real.display(),
        if profile == "release" {
            " --release"
        } else {
            ""
        }
    );
    assert_eq!(
        inspect_heaphook_file(&real),
        Ok(()),
        "the real hook at {} must inspect clean",
        real.display()
    );
    // Read its dynamic symbols directly: every required name must be
    // DEFINED (not merely imported), which is what `#[no_mangle]` buys and
    // what the source oracle can only infer.
    let bytes = std::fs::read(&real).expect("read the built hook");
    let (exported, imported) = scan_exports_for_test(&bytes, HEAPHOOK_REQUIRED_EXPORTS)
        .expect("the built hook's dynamic tables must be readable");
    for (name, (is_exported, is_imported)) in HEAPHOOK_REQUIRED_EXPORTS
        .iter()
        .zip(exported.iter().zip(imported.iter()))
    {
        assert!(
            *is_exported,
            "the built hook at {} does not EXPORT `{name}` (imported-only: {is_imported}) — \
             `dlsym` in the rmw will find nothing for it",
            real.display()
        );
    }
}

/// An explicit preload naming a MISSING library is a loud exit-69 error
/// naming the var, the kill switch and the hook default — never a silent
/// drop.
#[test]
#[serial]
fn explicit_preload_naming_missing_library_is_loud_exit_69() {
    let f = fixture();
    let _p = EnvVarGuard::set(ROS2_PRELOAD_ENV, "/nonexistent/hook.so");

    let err = build_ros2_passthrough_plan(Ros2NativeVerb::Run, &[], &f.lib_dir, &f.ament_prefix)
        .expect_err("a missing preload must refuse");
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(msg.contains(ROS2_PRELOAD_ENV), "names the var: {msg}");
    assert!(msg.contains("off"), "names the kill switch: {msg}");
    assert!(msg.contains(HEAPHOOK_FILENAME), "names the default: {msg}");
}

/// `CERULION_LIB_DIR` overrides the `current_exe()`-relative resolution
/// (absolutized); an EMPTY override is a loud error rather than a silent
/// fall-through.
#[test]
#[serial]
fn lib_dir_env_overrides_and_empty_is_loud() {
    let f = fixture();
    let lib_str = f.lib_dir.display().to_string();
    {
        let _g = EnvVarGuard::set(LIB_DIR_ENV, &lib_str);
        let resolved = resolve_lib_dir().expect("override must resolve");
        assert_eq!(resolved, f.lib_dir);
    }
    {
        let _g = EnvVarGuard::set(LIB_DIR_ENV, "");
        let err = resolve_lib_dir().expect_err("empty override must refuse");
        assert_eq!(classify(&err), EXIT_OTHER);
        assert!(err.to_string().contains(LIB_DIR_ENV));
    }
    {
        let _g = EnvVarGuard::unset(LIB_DIR_ENV);
        let resolved = resolve_lib_dir().expect("default must resolve");
        // The default is the test binary's own directory.
        let exe_dir = std::env::current_exe()
            .expect("current_exe")
            .parent()
            .expect("parent")
            .to_path_buf();
        assert_eq!(resolved, exe_dir);
    }
}

// ── Ament-prefix staging (own state dir → parallel-safe) ────────────────────

/// Cargo-layout lib dir: a per-lib-dir prefix is staged under the state dir
/// whose `lib/librmw_cerulion.so` is a symlink to the real cdylib;
/// re-staging is idempotent; two lib dirs get two distinct prefixes.
#[cfg(unix)]
#[test]
fn staging_creates_symlinked_prefix_idempotently() {
    let root = tempfile::tempdir().expect("tempdir");
    let state = root.path().join("state");
    let lib_a = root.path().join("target-a");
    let lib_b = root.path().join("target-b");
    for d in [&lib_a, &lib_b] {
        std::fs::create_dir_all(d).expect("mkdir");
        std::fs::write(d.join(RMW_LIB_FILENAME), b"so").expect("write");
    }

    let prefix_a = stage_ament_prefix_under(&state, &lib_a).expect("stage a");
    let link = prefix_a.join("lib").join(RMW_LIB_FILENAME);
    assert_eq!(
        std::fs::read_link(&link).expect("must be a symlink"),
        lib_a.join(RMW_LIB_FILENAME)
    );

    // Idempotent: same prefix, link still correct.
    let again = stage_ament_prefix_under(&state, &lib_a).expect("stage a again");
    assert_eq!(again, prefix_a);
    assert_eq!(
        std::fs::read_link(&link).expect("still a symlink"),
        lib_a.join(RMW_LIB_FILENAME)
    );

    // A different lib dir stages a DIFFERENT prefix (no shared symlink to
    // race on between two Cerulion builds).
    let prefix_b = stage_ament_prefix_under(&state, &lib_b).expect("stage b");
    assert_ne!(prefix_b, prefix_a);
}

/// A stale link (wrong target) is refreshed, never served silently.
#[cfg(unix)]
#[test]
fn staging_refreshes_a_stale_symlink() {
    let root = tempfile::tempdir().expect("tempdir");
    let state = root.path().join("state");
    let lib = root.path().join("target");
    std::fs::create_dir_all(&lib).expect("mkdir");
    std::fs::write(lib.join(RMW_LIB_FILENAME), b"so").expect("write");

    let prefix = stage_ament_prefix_under(&state, &lib).expect("stage");
    let link = prefix.join("lib").join(RMW_LIB_FILENAME);
    std::fs::remove_file(&link).expect("remove link");
    std::os::unix::fs::symlink("/wrong/target.so", &link).expect("plant stale link");

    let again = stage_ament_prefix_under(&state, &lib).expect("re-stage");
    assert_eq!(again, prefix);
    assert_eq!(
        std::fs::read_link(&link).expect("refreshed symlink"),
        lib.join(RMW_LIB_FILENAME)
    );
}

/// An installed layout (`<prefix>/lib`) IS an ament prefix already — its
/// parent is returned and nothing is written under the state dir.
#[test]
fn installed_lib_layout_uses_parent_prefix_without_staging() {
    let root = tempfile::tempdir().expect("tempdir");
    let state = root.path().join("state");
    let install = root.path().join("install");
    let lib = install.join("lib");
    std::fs::create_dir_all(&lib).expect("mkdir");
    std::fs::write(lib.join(RMW_LIB_FILENAME), b"so").expect("write");

    let prefix = stage_ament_prefix_under(&state, &lib).expect("resolve");
    assert_eq!(prefix, install);
    assert!(
        !state.exists(),
        "no staging dir is created for an installed layout"
    );
}

// ── Classification ──────────────────────────────────────────────────────────

/// The exec-failure classification: `NotFound` = ros2 not on PATH (127);
/// anything else unclassified = 1.
#[test]
fn classify_maps_notfound_to_127_and_unknown_to_1() {
    let not_found = CliError::Io(std::io::Error::from(std::io::ErrorKind::NotFound));
    assert_eq!(classify(&not_found), EXIT_ROS2_NOT_FOUND);
    let other = CliError::Validation("something unrelated".to_string());
    assert_eq!(classify(&other), EXIT_OTHER);
}

/// `exec_ros2` on a guaranteed-nonexistent program returns (never replaces
/// the test process) and classifies 127.
#[cfg(unix)]
#[test]
fn exec_of_nonexistent_program_returns_notfound() {
    use cerulion_cli_engine::ros2_cmd::{exec_ros2, Ros2Plan};
    let plan = Ros2Plan {
        program: "/nonexistent/cerulion-test/ros2".to_string(),
        args: vec!["run".to_string()],
        env: vec![],
        inspected: vec![],
    };
    let err = exec_ros2(&plan);
    assert_eq!(classify(&err), EXIT_ROS2_NOT_FOUND);
}

/// A TOCTOU: the launcher validates hook
/// paths by READING them and then hands ld.so the path STRINGS, so a swap
/// between inspection and exec launches an object nothing validated.
/// `exec_ros2` re-verifies every inspected file's identity immediately
/// before replacing the image and refuses on change.
///
/// The swap is the realistic one: the SAME path, a DIFFERENT file — a
/// symlink re-point or an atomic rename — which a check keyed on the path
/// string alone cannot see. `metadata` follows symlinks deliberately (so
/// does ld.so, and a symlinked hook is legitimate), so the pin is
/// identity, not "is a symlink".
///
/// The plan carries TWO entries and it is the
/// SECOND that is mutated, so a `verify_inspected` that checked only the
/// first — or stopped at the first match — fails here. A
/// single-entry arm could not tell those apart.
#[test]
#[serial]
fn a_validated_library_replaced_before_exec_is_refused() {
    use cerulion_cli_engine::ros2_cmd::{exec_ros2, InspectedFile, Ros2Plan};
    let root = tempfile::tempdir().expect("tempdir");
    // The launcher's OWN definition, not a hand-rebuilt tuple — a
    // local copy is a place for the identity to silently keep an old shape
    // after the real one is fixed (which is exactly what happened to the
    // mtime component).
    let ident = cerulion_cli_engine::ros2_cmd::file_identity;
    let first = root.path().join("libfirst.so");
    let second = root.path().join("libsecond.so");
    std::fs::write(&first, b"the first file the launcher validated").expect("write");
    std::fs::write(&second, b"the second file the launcher validated").expect("write");
    let entries = vec![
        InspectedFile {
            path: first.clone(),
            identity: ident(&first),
            fd: None,
        },
        InspectedFile {
            path: second.clone(),
            identity: ident(&second),
            fd: None,
        },
    ];
    let plan = |inspected: Vec<InspectedFile>| Ros2Plan {
        program: "/nonexistent/cerulion-test/ros2".to_string(),
        args: vec!["run".to_string()],
        env: vec![],
        inspected,
    };
    // Unchanged: the exec is attempted, so the failure is the missing
    // program — the anti-tautology half, proving the check is not simply
    // refusing everything.
    let err = exec_ros2(&plan(entries.clone()));
    assert_eq!(
        classify(&err),
        EXIT_ROS2_NOT_FOUND,
        "unchanged files must not block the exec: {err}"
    );
    // The SECOND entry swapped: same path, different inode.
    let replacement = root.path().join("other.so");
    std::fs::write(&replacement, b"a different file entirely").expect("write");
    std::fs::rename(&replacement, &second).expect("rename over the second target");
    let err = exec_ros2(&plan(entries.clone()));
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    let msg = err.to_string();
    assert!(
        msg.contains("is not the file this launch validated"),
        "names the condition: {msg}"
    );
    assert!(
        msg.contains("libsecond.so") && !msg.contains("libfirst.so"),
        "names the SECOND file, which is the one that changed: {msg}"
    );
    // The SECOND entry removed between inspection and exec.
    std::fs::remove_file(&second).expect("remove");
    let err = exec_ros2(&plan(entries));
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    assert!(
        err.to_string().contains("it no longer exists"),
        "a vanished file is named as vanished: {err}"
    );
    // A path recorded as ABSENT must still
    // be absent. Were such a path OMITTED from the plan
    // entirely, a file appearing between plan and exec would be loaded
    // having been validated by nothing.
    let appears = root.path().join("libappears.so");
    let absent_entry = vec![InspectedFile {
        path: appears.clone(),
        identity: None,
        fd: None,
    }];
    let err = exec_ros2(&plan(absent_entry.clone()));
    assert_eq!(
        classify(&err),
        EXIT_ROS2_NOT_FOUND,
        "still absent ⇒ the exec proceeds: {err}"
    );
    std::fs::write(&appears, b"a library that appeared after validation").expect("write");
    let err = exec_ros2(&plan(absent_entry));
    assert_eq!(classify(&err), EXIT_UNAVAILABLE);
    assert!(
        err.to_string()
            .contains("did not exist when the launch was validated"),
        "a file that appeared is named as such: {err}"
    );
}

/// Decision: the validated hook is handed to the child
/// through the DESCRIPTOR the launcher inspected, not by name. That is
/// what closes the window identity re-verification alone only narrows: `LD_PRELOAD`
/// names `/proc/self/fd/<N>`, the fd survives `exec`, and the child's
/// loader resolves that to the inherited descriptor — the exact inode
/// this launcher read.
///
/// The arm swaps the hook's PATH between plan and exec, which is the
/// attack rounds 20/21 could only narrow, and requires two things of the
/// result: the child is still pointed at a descriptor (not at the
/// attacker's name), and `verify_inspected` still passes, because the
/// descriptor's identity is unchanged — the swap cannot reach it. A
/// path-keyed check would have refused here; an fd-keyed one correctly
/// does not care, because the substituted file is not what will load.
#[test]
#[serial]
fn the_validated_hook_is_handed_to_the_child_by_descriptor() {
    use cerulion_cli_engine::ros2_cmd::exec_ros2;
    skip_unless_host_mapped!("the_validated_hook_is_handed_to_the_child_by_descriptor");
    let f = fixture();
    let hook = f.lib_dir.join(HEAPHOOK_FILENAME);
    write_hook_fixture(&hook);
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("the built hook is accepted");

    // The child is pointed at a DESCRIPTOR, not at the hook's name.
    let preload = env_value(&plan.env, "LD_PRELOAD").expect("LD_PRELOAD is staged");
    assert!(
        preload.starts_with("/proc/self/fd/"),
        "the hook must be staged by descriptor: {preload}"
    );
    assert!(
        !preload.contains(HEAPHOOK_FILENAME),
        "and its NAME must not be what the child resolves: {preload}"
    );
    let bound = plan
        .inspected
        .iter()
        .find(|i| i.fd.is_some())
        .expect("the descriptor-bound entry must be recorded");
    assert_eq!(
        bound.path.display().to_string(),
        preload,
        "the recorded entry is the one the child is handed"
    );
    // Its identity is the HOOK's, read from the open descriptor.
    let hook_id = file_identity(&hook).expect("stat the hook");
    assert_eq!(
        bound.identity,
        Some(hook_id),
        "the descriptor identifies the inode the launcher validated"
    );

    // The NUMBER must be the
    // descriptor the plan RETAINS. The builder's inspected `File` lives in a
    // vector that drops when the builder returns, so naming ITS number hands
    // the child a closed — possibly reused — fd, ld.so drops the preload with
    // a warning nobody reads, and the run silently falls back to the copy
    // path behind a flag that promised adoption. Asserting the `/proc/self/fd/`
    // prefix cannot see that; asserting the number can.
    use std::os::unix::io::AsRawFd;
    let retained = bound
        .fd
        .as_ref()
        .expect("the fd-bound entry carries the retained descriptor");
    let staged_fd: i32 = preload
        .rsplit('/')
        .next()
        .and_then(|n| n.parse().ok())
        .unwrap_or_else(|| panic!("LD_PRELOAD must end in the fd number: {preload}"));
    assert_eq!(
        staged_fd,
        retained.as_raw_fd(),
        "the number in LD_PRELOAD must be the descriptor the plan retains, not the \
         builder-local one it closed: {preload}"
    );
    // ...and that number is still OPEN, and still the validated inode. This
    // is the half a same-number assertion alone cannot give: `fstat` on the
    // bare number goes through the process fd table exactly as the child's
    // loader will, so a number that has been closed fails here.
    let mut st: libc::stat = unsafe { std::mem::zeroed() };
    // SAFETY: `staged_fd` is read back from the plan; `st` is a live local.
    let rc = unsafe { libc::fstat(staged_fd, &mut st) };
    assert_eq!(
        rc,
        0,
        "the staged fd {staged_fd} must still be open at exec time (errno {})",
        std::io::Error::last_os_error()
    );
    {
        use std::os::unix::fs::MetadataExt;
        let hooked = std::fs::metadata(&hook).expect("stat the hook");
        assert_eq!(
            (st.st_dev as u64, st.st_ino),
            (hooked.dev(), hooked.ino()),
            "and it must still resolve to the inode the launcher validated"
        );
    }

    // THE PIN: swap the file at the hook's PATH between plan and exec.
    // A path-keyed check refuses here; the descriptor cannot be reached
    // by a rename, so the launch proceeds — and what it proceeds with is
    // still the validated inode.
    let replacement = f.lib_dir.join("impostor.so");
    std::fs::write(&replacement, b"an impostor at the hook's path").expect("write");
    std::fs::rename(&replacement, &hook).expect("rename over the hook");
    let after = file_identity(&hook).expect("stat the impostor");
    assert_ne!(
        (after.0, after.1),
        (hook_id.0, hook_id.1),
        "the swap must really have changed the inode behind that name"
    );
    let err = exec_ros2(&plan);
    assert_eq!(
        classify(&err),
        EXIT_ROS2_NOT_FOUND,
        "the swap cannot reach the descriptor, so the launch is not refused — it fails only \
         on the missing `ros2` program: {err}"
    );
    // ...and the entry still identifies the ORIGINAL inode, because the
    // fd never followed the name.
    assert_eq!(
        bound.identity,
        Some(hook_id),
        "the descriptor still names the validated inode after the swap"
    );
}

/// The plan CARRIES the identities — the check above is inert if
/// the builder records nothing. Every staged preload entry and the rmw
/// `.so` (existence-checked by the staging, directory handed to the child
/// on LD_LIBRARY_PATH) must appear.
///
/// The recorded IDENTITY is asserted
/// against the real file's metadata, not just the path. A builder that
/// recorded the right paths with wrong, stale, or defaulted identities
/// would pass a path-only arm while `verify_inspected`
/// refused every launch — or, worse, matched a file it never read.
#[test]
#[serial]
fn the_plan_records_the_identity_of_every_file_it_hands_the_child() {
    skip_unless_host_mapped!("the_plan_records_the_identity_of_every_file_it_hands_the_child");
    let f = fixture();
    let hook = f.lib_dir.join(HEAPHOOK_FILENAME);
    write_hook_fixture(&hook);
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _d = EnvVarGuard::unset("LD_PRELOAD");
    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("the built hook is accepted");
    // Decision: the HOOK is recorded under its
    // descriptor path, so it is found by the entry that CARRIES an fd —
    // and its identity is still the hook inode's, read from that open
    // descriptor rather than by re-resolving a name.
    let bound = plan
        .inspected
        .iter()
        .find(|i| i.fd.is_some())
        .expect("the descriptor-bound hook must be recorded");
    {
        assert_eq!(
            bound.identity,
            file_identity(&hook),
            "the descriptor identifies the hook the launcher validated"
        );
        assert!(
            bound.path.starts_with("/proc/self/fd/"),
            "and it is handed to the child by that descriptor: {}",
            bound.path.display()
        );
    }
    // The rmw library is staged by NAME (only the hook is fd-bound), so it
    // is still found by path and identity-pinned the same way.
    let rmw_lib = f.lib_dir.join(RMW_LIB_FILENAME);
    let entry = plan
        .inspected
        .iter()
        .find(|i| i.path == rmw_lib)
        .unwrap_or_else(|| {
            panic!(
                "{} must be identity-pinned; got {:?}",
                rmw_lib.display(),
                plan.inspected.iter().map(|i| &i.path).collect::<Vec<_>>()
            )
        });
    {
        assert_eq!(
            entry.identity,
            file_identity(&rmw_lib),
            "the recorded identity must be the REAL file's, for {}",
            rmw_lib.display()
        );
    }
    // An entry the launcher could NOT
    // identify must still be RECORDED, with `None`. An ambient LD_PRELOAD
    // entry is not existence-checked — ld.so simply drops a path that is
    // not there — so this is the shape that reaches identity capture
    // unidentifiable. Omitting it would let a file APPEARING between
    // plan and exec be loaded having been validated by nothing.
    let ghost = f.lib_dir.join("libghost-never-created.so");
    let _amb = EnvVarGuard::set("LD_PRELOAD", &ghost.display().to_string());
    let plan = build_ros2_passthrough_plan_skipping_the_two_hop_refusal_for_test(
        Ros2NativeVerb::Run,
        &args(&[ADOPT_TAKE_FLAG, "pkg", "exe"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("an ambient entry ld.so would drop must not refuse the launch");
    let recorded = plan
        .inspected
        .iter()
        .find(|i| i.path == ghost)
        .unwrap_or_else(|| {
            panic!(
                "a path that could not be identified must still be RECORDED; got {:?}",
                plan.inspected.iter().map(|i| &i.path).collect::<Vec<_>>()
            )
        });
    assert_eq!(
        recorded.identity, None,
        "and recorded as unidentifiable, so exec refuses if it appears"
    );
}

// ── Spawn-based env application (unix fixture script) ───────────────────────

/// Write an executable shell script into `dir` and return its path.
#[cfg(unix)]
fn write_script(dir: &Path, name: &str, body: &str) -> PathBuf {
    use std::os::unix::fs::PermissionsExt;
    let path = dir.join(name);
    std::fs::write(&path, body).expect("write script");
    let mut perms = std::fs::metadata(&path).expect("meta").permissions();
    perms.set_mode(0o755);
    std::fs::set_permissions(&path, perms).expect("chmod");
    path
}

/// The plan's env pairs really reach the child (spawn-based; the exec()-path
/// exit-code inheritance is pinned by the real-binary e2e in
/// `cerulion_cli`): a fixture script run with the plan's program/args/env
/// sees `RMW_IMPLEMENTATION` and the staged `AMENT_PREFIX_PATH`, receives
/// `launch <args>` verbatim (hyphenated token included), and its exit code
/// comes back unchanged.
#[cfg(unix)]
#[test]
#[serial]
fn plan_env_reaches_the_child_and_exit_code_propagates() {
    let f = fixture();
    let _p = EnvVarGuard::unset(ROS2_PRELOAD_ENV);
    let _a = EnvVarGuard::unset("AMENT_PREFIX_PATH");
    let script = write_script(
        f.lib_dir.as_path(),
        "fake_ros2.sh",
        "#!/bin/sh\necho \"rmw=$RMW_IMPLEMENTATION\"\necho \"ament=$AMENT_PREFIX_PATH\"\necho \"argv=$*\"\nexit 42\n",
    );

    let mut plan = build_ros2_passthrough_plan_for_host(
        Ros2NativeVerb::Launch,
        &args(&["--show-args", "demo.launch.py"]),
        &f.lib_dir,
        &f.ament_prefix,
        HostAbi::LINUX_GNU,
    )
    .expect("happy plan must build");
    plan.program = script.display().to_string();

    let mut cmd = std::process::Command::new(&plan.program);
    cmd.args(&plan.args);
    for (k, v) in &plan.env {
        cmd.env(k, v);
    }
    let out = cmd.output().expect("spawn fixture script");
    assert_eq!(out.status.code(), Some(42), "exit code propagates");
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.contains("rmw=rmw_cerulion"), "stdout: {stdout}");
    assert!(
        stdout.contains(&format!("ament={}", f.ament_prefix.display())),
        "stdout: {stdout}"
    );
    assert!(
        stdout.contains("argv=launch --show-args demo.launch.py"),
        "stdout: {stdout}"
    );
}

/// The preload inspectors bound their
/// READ at the descriptor, not at the size they just stat-ed.
///
/// `meta.len()` is a snapshot, and these paths come from `LD_PRELOAD` — ambient
/// or `CERULION_ROS2_PRELOAD`, both environment-controlled — so a file that
/// GROWS after the stat would make `std::fs::read` / `read_to_end` chase the moving
/// EOF and allocate without limit, OOM-ing or hanging
/// `cerulion ros2 run --adopt-take` before the node ever launched. Identity
/// re-verification closes the TOCTOU on the file's IDENTITY, not this one, on its SIZE.
///
/// Pinned on the BOUND itself rather than by racing a growing file against the
/// stat→read window. A racing arm was written first and discarded: it never
/// observed the window in 20 s of continuous toggling, so it could only ever
/// have failed for the wrong reason or passed without testing anything. What
/// the inspectors guarantee is a read that stops at a ceiling regardless of
/// what the stat said, and that is exactly what this drives — against the same
/// implementation the three inspectors call, not a copy of it.
#[test]
fn the_preload_read_stops_at_its_ceiling_whatever_the_file_turns_out_to_be() {
    let root = tempfile::tempdir().expect("tempdir");
    let path = root.path().join("bigger-than-it-said.so");
    // 4 KiB on disk; every caller passes `LIMIT + 1`, so the shape under test
    // is "the file is larger than the ceiling handed to the read".
    std::fs::write(&path, vec![0xABu8; 4096]).expect("seed");

    let bytes =
        read_bounded_from_for_test(std::fs::File::open(&path).expect("open"), 512).expect("read");
    assert_eq!(
        bytes.len(),
        512,
        "the read must stop AT the ceiling — an unbounded read returns the whole file, \
         which is what lets a preload that grew after its size check allocate without limit"
    );
    assert!(
        bytes.iter().all(|b| *b == 0xAB),
        "…and it must be the file's own leading bytes, not a truncated-elsewhere buffer"
    );

    // Under the ceiling, nothing is truncated: the bound must not silently
    // shorten a conforming file.
    let short = root.path().join("small.so");
    std::fs::write(&short, vec![0xCDu8; 100]).expect("seed short");
    assert_eq!(
        read_bounded_from_for_test(std::fs::File::open(&short).expect("open"), 512)
            .expect("read short")
            .len(),
        100,
        "a file under the ceiling is read whole"
    );

    // `LIMIT + 1` is what lets a caller tell "exactly at the limit" from
    // "over it": a read that stopped at the limit returns the same length for
    // both.
    let at_limit = root.path().join("at-limit.so");
    std::fs::write(&at_limit, vec![0xEFu8; 512]).expect("seed at-limit");
    assert_eq!(
        read_bounded_from_for_test(std::fs::File::open(&at_limit).expect("open"), 513)
            .expect("read at-limit")
            .len(),
        512,
        "a file exactly at the limit reads whole and is distinguishable from an over-limit \
         one, which fills the ceiling"
    );
}

/// A reader that records how many bytes were actually PULLED from the source.
struct CountingReader<R> {
    inner: R,
    pulled: std::sync::Arc<std::sync::atomic::AtomicU64>,
}

impl<R: std::io::Read> std::io::Read for CountingReader<R> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let n = self.inner.read(buf)?;
        self.pulled
            .fetch_add(n as u64, std::sync::atomic::Ordering::Relaxed);
        Ok(n)
    }
}

/// The bound is pinned on what the read
/// PULLS, not on what it returns.
///
/// Checking only the returned length and its prefix bytes is not enough: an
/// unbounded `read_to_end` followed by a `truncate(ceiling)` satisfies that exactly
/// — same length, same bytes — while allocating the whole growing file, i.e.
/// while doing the one thing the bound exists to prevent. The returned buffer
/// simply cannot distinguish those two implementations; only the SOURCE can.
///
/// So the source counts. A `repeat` source is capped well above the ceiling, a
/// bounded read must stop at the ceiling and leave the rest unread, and an
/// unbounded one drains the cap. The sibling arm
/// (`the_preload_read_stops_at_its_ceiling_whatever_the_file_turns_out_to_be`)
/// keeps the length, prefix and exact-boundary checks; this one adds the claim
/// those cannot make.
#[test]
fn the_preload_read_never_pulls_more_than_its_ceiling() {
    const CEILING: u64 = 512;
    // Well above the ceiling, so "stopped at the bound" and "drained the
    // source" are far apart, and finite so an unbounded read TERMINATES and
    // FAILS rather than hanging the suite.
    const AVAILABLE: u64 = CEILING * 4;

    let pulled = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
    let src = CountingReader {
        inner: std::io::Read::take(std::io::repeat(0xABu8), AVAILABLE),
        pulled: std::sync::Arc::clone(&pulled),
    };

    let bytes = read_bounded_from_for_test(src, CEILING).expect("read");
    let pulled = pulled.load(std::sync::atomic::Ordering::Relaxed);

    assert_eq!(
        bytes.len() as u64,
        CEILING,
        "the read returns at most the ceiling"
    );
    assert!(
        pulled <= CEILING,
        "the read must STOP at its ceiling, not read everything and truncate: it pulled \
         {pulled} bytes of the {AVAILABLE} available for a {CEILING}-byte ceiling. Returning \
         the right bytes is not the property under test — a `read_to_end` followed by a \
         truncate returns exactly these bytes while allocating the whole file, which is the \
         OOM this bound exists to prevent."
    );
}

/// A TOCTOU: every preload read comes
/// from the descriptor the fstat ran on, never from a second resolution of the
/// path.
///
/// `read_inspected` holds this by construction, and the sibling inspectors must
/// too: stat-ing the path and then reading the path is two resolutions of an
/// environment-controlled name, and a bounded read built on that shape lets
/// a replacement between the two be what
/// gets inspected.
///
/// Driven through the PRODUCTION path (`preload_allocator_exports`) with a
/// seam that acts in the window between the open and the read — the only way
/// to make this deterministic rather than a race against a swapper thread.
///
/// (a) the path is swapped to a DIFFERENT regular file after the open: the
/// ORIGINAL file's bytes must be the ones inspected. The two files are chosen
/// so the answer differs — the original exports nothing (not an ELF at all),
/// the replacement is a real allocator-exporting ELF — so reading the wrong
/// one is visible in the verdict, not merely in bytes nobody checks.
#[test]
#[serial]
fn a_preload_path_swapped_after_the_open_is_inspected_as_it_was_opened() {
    let root = tempfile::tempdir().expect("tempdir");
    let path = root.path().join("swapped.so");
    let decoy = root.path().join("decoy.so");
    // ORIGINAL: not an ELF ⇒ correctly "no allocator".
    std::fs::write(&path, b"not an elf").expect("seed original");
    // REPLACEMENT: a real ELF exporting the malloc family ⇒ a different answer.
    std::fs::write(&decoy, foreign_allocator_so()).expect("seed decoy");

    // The swap's OUTCOME is recorded
    // and asserted. Ignoring it would make this arm pass whenever the swap FAILED —
    // the inspector would read the untouched original and report no exports,
    // which is the expected answer, so a rename that never happened would look
    // exactly like a guard that worked.
    //
    // Recorded rather than `expect`ed INSIDE the closure: the seam runs while
    // `run_after_open_hook` holds the hook mutex, so a panic there would poison
    // it and leave the seam armed for the rest of the binary.
    let swap_err: std::sync::Arc<std::sync::Mutex<Option<String>>> =
        std::sync::Arc::new(std::sync::Mutex::new(Some("the seam never ran".into())));
    let swap_from = decoy.clone();
    let swap_to = path.clone();
    let seam_err = std::sync::Arc::clone(&swap_err);
    let _seam = arm_after_open_hook_for_test(Box::new(move || {
        // RENAME, not copy. A copy writes through to the SAME inode, which the
        // open descriptor would legitimately see — that is not a TOCTOU, it is
        // the file changing, so a copy would not exercise this arm for
        // exactly that reason. A rename repoints the NAME at a different inode while
        // the descriptor keeps the original, which is the hazard under test.
        let outcome = match std::fs::rename(&swap_from, &swap_to) {
            Ok(()) => None,
            Err(e) => Some(format!("rename failed: {e}")),
        };
        *seam_err.lock().expect("swap outcome") = outcome;
    }));

    let verdict = preload_allocator_exports(&path).expect("inspection");

    // The swap must have HAPPENED, or the rest of this arm proves nothing.
    if let Some(why) = swap_err.lock().expect("swap outcome").as_ref() {
        panic!(
            "the decoy was never swapped in ({why}) — without it this arm reads the \
                untouched original and passes for the wrong reason"
        );
    }
    assert_eq!(
        std::fs::read(&path).expect("read back the path"),
        foreign_allocator_so(),
        "…and the NAME must now resolve to the decoy: that is the state whose inspection \
         this arm is about"
    );
    assert!(
        verdict.is_empty(),
        "the bytes must come from the descriptor that was fstat-ed, not from a second \
         resolution of the path: the file was replaced after the open with an \
         allocator-exporting ELF and the inspector reported {verdict:?}"
    );
}

/// (b) the path is swapped to a FIFO after the open: the launcher must not
/// block, and must still inspect the original. A re-open by path here is worse
/// than wrong — a writer-less FIFO blocks forever per POSIX, so
/// `cerulion ros2 run --adopt-take` hangs with no output at all.
#[test]
#[serial]
#[cfg(unix)]
fn a_preload_path_swapped_to_a_fifo_after_the_open_neither_blocks_nor_misreads() {
    let root = tempfile::tempdir().expect("tempdir");
    let path = root.path().join("becomes-a-fifo.so");
    std::fs::write(&path, foreign_allocator_so()).expect("seed original");

    // The FIFO must not outlive the test body. `TempDir`'s cleanup walks the
    // directory, and opening a writer-less FIFO blocks forever — leaving it
    // there would hang the whole suite holding
    // the desk lock. The unlink rides a Drop guard so a FAILING assertion
    // cannot skip it.
    struct UnlinkOnDrop(PathBuf);
    impl Drop for UnlinkOnDrop {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }
    let _fifo_cleanup = UnlinkOnDrop(path.clone());

    // Both syscalls are checked, and
    // the resulting file TYPE is asserted. Ignoring them would make this arm pass
    // whenever the swap failed — the original stayed in place, the inspector
    // read it, and the expected answer came back from a test that had exercised
    // nothing. Recorded rather than `expect`ed inside the closure for the same
    // mutex-poisoning reason as its sibling.
    let fifo_err: std::sync::Arc<std::sync::Mutex<Option<String>>> =
        std::sync::Arc::new(std::sync::Mutex::new(Some("the seam never ran".into())));
    let fifo_at = path.clone();
    let seam_err = std::sync::Arc::clone(&fifo_err);
    let _seam = arm_after_open_hook_for_test(Box::new(move || {
        let outcome = std::fs::remove_file(&fifo_at)
            .err()
            .map(|e| format!("remove failed: {e}"))
            .or_else(|| {
                let c =
                    std::ffi::CString::new(fifo_at.as_os_str().as_encoded_bytes()).expect("cstr");
                // SAFETY: a NUL-terminated path into a tempdir this test owns.
                let rc = unsafe { libc::mkfifo(c.as_ptr(), 0o644) };
                (rc != 0).then(|| format!("mkfifo failed: {}", std::io::Error::last_os_error()))
            });
        *seam_err.lock().expect("fifo outcome") = outcome;
    }));

    let verdict = preload_allocator_exports(&path).expect("inspection");

    if let Some(why) = fifo_err.lock().expect("fifo outcome").as_ref() {
        panic!(
            "the FIFO was never swapped in ({why}) — without it this arm reads the \
                untouched original and passes for the wrong reason"
        );
    }
    assert!(
        std::fs::symlink_metadata(&path)
            .expect("stat the path")
            .file_type()
            .is_fifo(),
        "…and the NAME must now resolve to a FIFO: that is the state whose inspection this \
         arm is about"
    );
    assert!(
        !verdict.is_empty(),
        "the ORIGINAL file's exports must still be reported — a re-open by path would find \
         the FIFO that replaced it, and a writer-less FIFO blocks forever, hanging the \
         launcher with no output. got {verdict:?}"
    );
}
