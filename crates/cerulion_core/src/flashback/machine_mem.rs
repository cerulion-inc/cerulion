// SPDX-License-Identifier: AGPL-3.0-only
//! **Effective RAM**: the one machine
//! number the Flashback window cap is a fraction of.
//!
//! The window cap is not a constant: it is
//! `clamp(effRAM / 16, 320 MiB, 8 GiB)`, so a Jetson keeps roughly the static-floor plane
//! while an AGX or a desk gets a window proportional to the machine it is running
//! on. That is only correct if "the machine" means the memory THIS PROCESS may
//! actually use, which on a containerised robot is not the host's `MemTotal`.
//!
//! # effRAM = the TIGHTER of the machine's total and the cgroup ceiling
//!
//! Without the cgroup term a container budgets against its host and the cap is
//! wrong by the container ratio — a 2 GiB container on a 64 GiB machine would size
//! its window at 4 GiB, i.e. twice the memory it is allowed to touch at all. The
//! rule is therefore `min(machine total, cgroup ceiling)`, with each term optional
//! and an absent term simply not participating.
//!
//! The cgroup ceiling is itself a minimum, over everything that binds this process
//! at once: every directory from its own cgroup up to the mount point,
//! and on cgroup v2 `memory.high` beside `memory.max`. A limit on a PARENT is not
//! advice a child can exceed, and `memory.high` is not advice `memory.max`
//! overrides — whichever is smallest is the one an allocation meets first. Reading
//! only the leaf's `memory.max` would grant a throttled or
//! parent-capped container a window larger than its real share, so the budget
//! would think it fits while the kernel reclaims underneath it.
//!
//! # Everything decidable is PURE
//!
//! The four parses ([`parse_mem_total_kib`], [`parse_cgroup_limit`],
//! [`parse_own_cgroup_path`], [`resolve_cgroup_dir`]) and the choice between the
//! two sources ([`effective_ram`]) take strings and numbers and return decisions,
//! so every branch — including the ones no healthy desk can reach (a cgroup v1
//! sentinel, a literal `max`, a Jetson-shaped `MemTotal`, a container whose cgroup
//! mount is rooted at its own cgroup) — is oracle-testable with no `/proc`, no
//! container and no second machine.
//!
//! The cgroup LADDER is decidable on the same terms, and for the same reason:
//! [`cgroup_limit_from_sources`] takes the two `/proc` bodies plus a function that
//! reads a limit file, so the v2-before-v1 order, the ancestor walk and the
//! `Unlimited`-stops / `Unknown`-continues rule are all exercised against a
//! hand-built filesystem. Only [`sample_effective_ram`] and [`sample_cgroup_limit`]
//! touch the OS, and they are the thin layer the oracles do not cover.
//!
//! None of the parses are `#[cfg(target_os = "linux")]`, deliberately and for the
//! reason [`crate::state_carrier::fork::parse_rss_anon_kib`] states: gating them
//! would make their oracles unrunnable on the platform this repo is authored on,
//! and the defect class they exist to prevent — a reader whose NAME says one thing
//! and whose BODY reads another — is exactly the kind that survives when nobody
//! can execute the check.
//!
//! # What is deliberately NOT read
//!
//! * **swap.** `memory.swap.max` is a separate budget and would only ever RAISE
//!   the figure, which is the wrong direction for a ceiling.
//! * **cgroup v1's `memory.soft_limit_in_bytes`.** It is not v2's `memory.high`
//!   under another name — see `CgroupHierarchy::limit_files`.
//! * **Ancestors ABOVE the mount point** — the one binding limit this does NOT
//!   see, stated plainly because the failure direction is the unfavourable one.
//!   Such a cgroup binds, but it is not visible: in the host-namespace container
//!   shape the cgroup filesystem is rooted at the container's own cgroup, so there
//!   is no path to a parent's limit file at all. The chain therefore stops where
//!   the filesystem does, and a container capped ONLY above its own mount still
//!   sizes its window from the machine, exactly as a leaf-only read would,
//!   bounded by the 8 GiB ceiling and evicted rather than fatal.
//!
//!   The alternative is worse, which is why this is the boundary: synthesising a
//!   path above the mount point would address whatever else is mounted there, and
//!   a ceiling read off an unrelated directory is a confidently WRONG number where
//!   this one is merely an absent term. Reading no limit degrades to the
//!   machine total; reading somebody else's does not.

use std::borrow::Cow;

/// Which source the effective figure came from.
///
/// Carried rather than inferred because the cap-provenance line an operator reads
/// has to name it: "4 GiB = machine 64 GiB / 16" and "128 MiB = container 2 GiB /
/// 16 (floored to 320 MiB)" are different sentences with different remedies, and
/// a bare number lets an operator conclude the wrong one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RamBasis {
    /// The machine's own total — `MemTotal` on Linux, `hw.memsize` on macOS. No
    /// container limit applied, either because there is none or because it is
    /// looser than the machine (which is not a limit at all).
    Machine,
    /// The cgroup ceiling binding this process, which is TIGHTER than the
    /// machine's total.
    ///
    /// "Binding" rather than "its own": the figure is the minimum over every
    /// directory from this process's cgroup up to the mount point, and on cgroup v2
    /// over `memory.high` as well as `memory.max`. Which of those produced it is
    /// deliberately NOT carried — the operator-facing remedy is the same sentence
    /// either way ("this container's share is raised by the container"), and a
    /// basis that named the file would have to be kept accurate against a chain
    /// whose tightest link changes with the machine's load.
    Cgroup,
}

/// The memory this process may actually use, and where that figure came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EffectiveRam {
    /// The figure, in bytes.
    pub bytes: u64,
    /// Which of the two sources won.
    pub basis: RamBasis,
}

/// What a cgroup memory-limit file said — or, once a whole ancestor chain of them
/// has been folded (`fold_cgroup_chain`), what the chain said.
///
/// ONE type for both scopes because they are one lattice: a chain's ceiling is a
/// cgroup limit, folded with `min`, and every distinction that matters at a single
/// file matters identically for the chain. A second enum shaped exactly like this
/// one would have to restate all three doc paragraphs and could drift from them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CgroupLimit {
    /// A real byte limit — for a chain, the TIGHTEST one found anywhere on it.
    Bytes(u64),
    /// No limit: cgroup v2's literal `max`, or cgroup v1's huge sentinel. For a
    /// chain: something on it answered, and nothing named a number.
    ///
    /// Distinct from [`Unknown`](Self::Unknown) on purpose. "There is no limit" is
    /// a POSITIVE answer — it says the machine total is the right figure — while
    /// "I could not read it" says nothing at all. Folding them looks harmless
    /// for a single file (both leave the machine total standing) and is not
    /// once anything has to act on which happened.
    ///
    /// The ladder in [`cgroup_limit_from_sources`] is the thing that acts on it:
    /// a positively unlimited v2 hierarchy ENDS the search, while an
    /// unreadable one falls through to v1.
    Unlimited,
    /// The file was absent, empty, or held something this build cannot read. For a
    /// chain: nothing on it could be read at all.
    Unknown,
}

/// Divisor-free floor above which a cgroup limit is read as "no limit".
///
/// cgroup v1 spells unlimited as `PAGE_COUNTER_MAX * PAGE_SIZE`, i.e. `i64::MAX`
/// rounded DOWN to a page multiple — which makes the literal value page-size
/// dependent: 9 223 372 036 854 771 712 on a 4 KiB kernel, 9 223 372 036 854 710 272
/// on a 64 KiB one (both shipping ARM configurations). So the rule is a FLOOR
/// rather than an equality against one spelling of the sentinel.
///
/// 2^62 is 4 EiB. It sits four orders of magnitude below every sentinel spelling
/// and four orders above any limit a machine could enforce, so the band between
/// "a real limit" and "the sentinel" is not one anything can land in by accident.
pub const CGROUP_UNLIMITED_FLOOR: u64 = 1 << 62;

/// PURE: total memory in KIBIBYTES out of a `/proc/meminfo` body, or `None` when
/// the field is absent or malformed.
///
/// The UNIT is checked rather than assumed, exactly as
/// [`crate::state_carrier::fork::parse_rss_anon_kib`] does: reading `kB` as bytes
/// would under-report by 1024x, and at this gate an under-reported machine means a
/// window cap 1024x too small — silently, on every robot.
pub fn parse_mem_total_kib(meminfo: &str) -> Option<u64> {
    for line in meminfo.lines() {
        let Some(rest) = line.strip_prefix("MemTotal:") else {
            continue;
        };
        let mut fields = rest.split_whitespace();
        let value: u64 = fields.next()?.parse().ok()?;
        if fields.next()? != "kB" {
            return None;
        }
        return Some(value);
    }
    None
}

/// PURE: read a cgroup memory-limit file's body.
///
/// Handles all four shapes one can hold: a number, cgroup v2's literal `max`,
/// cgroup v1's huge sentinel (see [`CGROUP_UNLIMITED_FLOOR`]), and anything else.
///
/// A limit of ZERO reads as [`Unknown`](CgroupLimit::Unknown) rather than as
/// `Bytes(0)`: a cgroup in which every allocation fails is not a machine anything
/// runs on, so the value is far more likely to be a file this build does not
/// understand than a budget somebody set — and treating it as a real figure would
/// mean sizing a window against zero bytes.
pub fn parse_cgroup_limit(raw: &str) -> CgroupLimit {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return CgroupLimit::Unknown;
    }
    if trimmed.eq_ignore_ascii_case("max") {
        return CgroupLimit::Unlimited;
    }
    match trimmed.parse::<u64>() {
        Ok(0) => CgroupLimit::Unknown,
        Ok(bytes) if bytes >= CGROUP_UNLIMITED_FLOOR => CgroupLimit::Unlimited,
        Ok(bytes) => CgroupLimit::Bytes(bytes),
        Err(_) => CgroupLimit::Unknown,
    }
}

/// PURE: this process's own cgroup path for `controller`, out of a
/// `/proc/self/cgroup` body.
///
/// The file is one record per line, `hierarchy-ID:controller-list:cgroup-path`.
/// cgroup v2 writes exactly one line whose hierarchy is `0` and whose controller
/// list is EMPTY (`0::/payload`), so a v2 lookup passes `""`. cgroup v1 writes one
/// line per hierarchy with a comma-separated controller list
/// (`4:cpu,cpuacct,memory:/payload`), so a v1 lookup passes `"memory"`.
///
/// The controller is matched as a WHOLE comma-separated token, never as a
/// substring: v1 hierarchies can be NAMED (`5:name=systemd:/…`), and a substring
/// match would let an unrelated hierarchy answer for the memory controller and
/// point the read at a file that does not exist.
///
/// The path is the REMAINDER of the line rather than the third field, because a
/// cgroup path may itself contain a colon.
pub fn parse_own_cgroup_path<'a>(proc_self_cgroup: &'a str, controller: &str) -> Option<&'a str> {
    for line in proc_self_cgroup.lines() {
        let mut parts = line.splitn(3, ':');
        // Every field is `else { continue }` rather than `?`: a malformed line must
        // SKIP, and a `?` here returns from the whole function, so one junk line
        // ahead of the real record would answer `None` for a process that is very
        // much in a cgroup — a silently unbudgeted container.
        let (Some(_hierarchy), Some(controllers), Some(path)) =
            (parts.next(), parts.next(), parts.next())
        else {
            continue;
        };
        let matches = if controller.is_empty() {
            controllers.is_empty()
        } else {
            controllers.split(',').any(|c| c == controller)
        };
        if matches && path.starts_with('/') {
            return Some(path);
        }
    }
    None
}

/// Which cgroup hierarchy a limit lookup is for.
///
/// The two differ on BOTH sides of the resolution — the filesystem type the mount
/// carries and the controller `/proc/self/cgroup` names it under — so they are one
/// enum rather than two string arguments a caller could pair up wrongly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CgroupHierarchy {
    /// cgroup v2's unified hierarchy: a `cgroup2` mount, named by the single
    /// `/proc/self/cgroup` line whose controller list is EMPTY (`0::/payload`).
    V2,
    /// cgroup v1's `memory` controller: a `cgroup` mount whose SUPER OPTIONS carry
    /// `memory`, named by the `/proc/self/cgroup` line listing that controller.
    V1Memory,
}

impl CgroupHierarchy {
    /// The `/proc/self/mountinfo` filesystem type this hierarchy is mounted as.
    fn fstype(self) -> &'static str {
        match self {
            Self::V2 => "cgroup2",
            Self::V1Memory => "cgroup",
        }
    }

    /// The controller token [`parse_own_cgroup_path`] matches on.
    fn controller(self) -> &'static str {
        match self {
            // v2 writes an EMPTY controller list; see `parse_own_cgroup_path`.
            Self::V2 => "",
            Self::V1Memory => "memory",
        }
    }

    /// The super option a v1 mount must carry to BE this controller's mount.
    ///
    /// `None` for v2: the filesystem type already identifies it, and a unified
    /// mount lists no controllers in its options.
    fn required_super_option(self) -> Option<&'static str> {
        match self {
            Self::V2 => None,
            Self::V1Memory => Some("memory"),
        }
    }

    /// Where this hierarchy is mounted IF it is mounted at the hierarchy root —
    /// the assumption the pre-mountinfo code made unconditionally, kept as the
    /// fallback for a `/proc/self/mountinfo` that cannot be read at all.
    fn root_mounted_base(self) -> &'static str {
        match self {
            Self::V2 => "/sys/fs/cgroup",
            Self::V1Memory => "/sys/fs/cgroup/memory",
        }
    }

    /// Every file in one cgroup directory that names a ceiling this process must
    /// respect. ALL of them bind, so the directory's ceiling is their minimum.
    ///
    /// v2 reads `memory.high` beside `memory.max` because `memory.high` binds
    /// FIRST: it is the throttle, not the killer — past it the kernel
    /// reclaims aggressively and stalls the allocator rather than OOM-killing. For
    /// a ROLLING WINDOW that distinction does not help. A window sized above the
    /// throttle is a standing residency the kernel spends every cycle trying to
    /// take back, so the effective ceiling is the tighter of the two, and a
    /// `memory.high` set below `memory.max` is a deliberate operator statement
    /// about what this container may actually keep resident.
    ///
    /// v1 reads its hard limit and NOT `memory.soft_limit_in_bytes`. The v1 soft
    /// limit is not v2's `memory.high` under another name: it is a best-effort
    /// reclaim PRIORITY consulted only when the machine as a whole is under
    /// pressure, with no throttle and no guarantee, and distributions ship it set
    /// on cgroups nobody intended to cap. Reading it would shrink windows against
    /// a number the kernel does not enforce.
    fn limit_files(self) -> &'static [&'static str] {
        match self {
            Self::V2 => &["memory.max", "memory.high"],
            Self::V1Memory => &["memory.limit_in_bytes"],
        }
    }
}

/// PURE: undo `/proc/self/mountinfo`'s `\ooo` escaping of a path field.
///
/// The kernel writes these fields through `seq_escape(…, " \t\n\\")`, so a mount
/// path containing a space arrives as `\040`, a tab as `\011` and a literal
/// backslash as `\134`. `/proc/self/cgroup` is written by a different code path
/// that escapes NOTHING, so the two spellings of one path do not compare equal
/// until this runs — and a container whose cgroup path holds a space would fail
/// every mount match and silently lose its limit.
///
/// Decoding is byte-wise rather than char-wise because the escaping is: a
/// non-ASCII path has its multi-byte characters written raw and only the four
/// special bytes escaped, so substituting bytes reproduces the original exactly.
///
/// Three rules keep a malformed field from becoming a wrong path:
///
/// * The escape is EXACTLY three octal digits. `\04x`, `\04` at the end of the
///   field, and `\` alone all keep their backslash literally, because that is
///   what a `\` the kernel did not write means.
/// * `\400`..`\777` name no byte, so they are left literal too.
/// * A decode whose bytes are not UTF-8 is discarded for the ORIGINAL field. The
///   kernel cannot produce that from a real path, and a lossy replacement would
///   invent a path that matches nothing; falling back leaves the field as it
///   arrived rather than adding a new failure.
fn decode_mountinfo_escapes(field: &str) -> Cow<'_, str> {
    // The overwhelmingly common case: no escape, no allocation, no scan past the
    // one `memchr`-shaped pass `contains` already does.
    if !field.contains('\\') {
        return Cow::Borrowed(field);
    }
    let raw = field.as_bytes();
    let mut out = Vec::with_capacity(raw.len());
    let mut i = 0;
    while i < raw.len() {
        // `i + 3 < raw.len()` — three digits must FOLLOW the backslash, so the
        // last of them sits at `i + 3` and must be in bounds.
        if raw[i] == b'\\' && i + 3 < raw.len() {
            let digits = &raw[i + 1..i + 4];
            if digits.iter().all(|b| matches!(b, b'0'..=b'7')) {
                let value = digits
                    .iter()
                    .fold(0u32, |acc, b| acc * 8 + u32::from(b - b'0'));
                if let Ok(byte) = u8::try_from(value) {
                    out.push(byte);
                    i += 4;
                    continue;
                }
            }
        }
        out.push(raw[i]);
        i += 1;
    }
    String::from_utf8(out).map_or(Cow::Borrowed(field), Cow::Owned)
}

/// One `/proc/self/mountinfo` record, reduced to the four fields this resolution
/// needs.
struct MountInfoEntry<'a> {
    /// Field 4 — the directory INSIDE the filesystem that forms the root of this
    /// mount. `/` on a host mount; the container's own cgroup on cgroup-v1 Docker
    /// with a host cgroup namespace, which is the whole reason this parse exists.
    ///
    /// [Octal-decoded](decode_mountinfo_escapes), so it is comparable with the
    /// UNescaped path `/proc/self/cgroup` carries. `Cow` because the escape is
    /// rare enough that the common case should not allocate.
    root: Cow<'a, str>,
    /// Field 5 — where that root is attached in THIS process's view. Octal-decoded
    /// for the same reason as [`root`](Self::root).
    mount_point: Cow<'a, str>,
    /// The filesystem type (the first field after the ` - ` separator).
    fstype: &'a str,
    /// The super options (the third field after the separator) — where a cgroup v1
    /// mount names its controller.
    super_options: &'a str,
}

/// PURE: one `/proc/self/mountinfo` line, or `None` when it is not one.
///
/// The format is positional AROUND a separator rather than at fixed indices:
///
/// ```text
/// 36 35 98:0 /mnt1 /mnt2 rw,noatime master:1 - ext3 /dev/root rw,errors=continue
/// (1)(2)(3)   (4)   (5)     (6)        (7)  (8) (9)   (10)          (11)
/// ```
///
/// Field (7) is ZERO OR MORE optional `tag:value` fields, so (9) onwards can only
/// be found by scanning for the bare `-` at (8) — indexing from the front past
/// field 6, or from the back, both break on real kernels (a shared mount carries
/// `shared:N`, a slave carries `master:N`, an unbindable one carries neither).
///
/// A line this cannot read answers `None`, which the caller SKIPS — never a `?`
/// that abandons the whole file. That is the bug `parse_own_cgroup_path` already
/// records: one junk line ahead of the real record must not leave a container
/// silently unbudgeted.
///
/// Fields 4 and 5 are [octal-decoded](decode_mountinfo_escapes); the fields after
/// the separator are NOT, and deliberately. The kernel escapes every one of them,
/// but the two this resolution COMPARES are the paths — and only those two are
/// compared against text (`/proc/self/cgroup`'s) that carries no escaping at all.
/// The others are matched against `cgroup2`/`cgroup` and a controller name, none
/// of which can contain a character the kernel escapes, so decoding them would be
/// motion with no case behind it.
fn parse_mountinfo_line(line: &str) -> Option<MountInfoEntry<'_>> {
    let mut fields = line.split_whitespace();
    let _mount_id = fields.next()?;
    let _parent_id = fields.next()?;
    let _major_minor = fields.next()?;
    // Splitting on whitespace is sound BECAUSE of the escaping: a path holding a
    // space reaches us as `\040`, so a field never splits in two.
    let root = decode_mountinfo_escapes(fields.next()?);
    let mount_point = decode_mountinfo_escapes(fields.next()?);
    let _mount_options = fields.next()?;
    // Scan the optional fields for the separator. `by_ref` so the iterator keeps
    // its position for the post-separator fields below.
    for token in fields.by_ref() {
        if token == "-" {
            let fstype = fields.next()?;
            let _mount_source = fields.next()?;
            let super_options = fields.next()?;
            return Some(MountInfoEntry {
                root,
                mount_point,
                fstype,
                super_options,
            });
        }
    }
    None
}

/// PURE: subtract a mount's ROOT from a cgroup path, or `None` when the path is
/// not inside that root.
///
/// This is the whole of the resolution, and the three cases are the three shapes a real
/// robot produces:
///
/// * `root == "/"` — a host mount. The remainder is the whole cgroup path, which
///   is what the naive join assumed and why that join worked on a desk.
/// * `root == cgroup_path` — cgroup-v1 Docker with a HOST cgroup namespace, the
///   norm on the Jetson L4T / Ubuntu 20.04 class this feature targets. The mount
///   is rooted AT the container's own cgroup, so the limit file sits directly in
///   the mount point and the remainder is EMPTY.
/// * `root` is a strict ANCESTOR — the remainder is what is left below it.
///
/// Anything else (the mount root is BELOW us, or beside us) means this process's
/// own cgroup directory is not visible through this mount at all, which is a
/// correct `None` rather than a path to somebody else's limit.
fn cgroup_path_under_root<'a>(cgroup_path: &'a str, root: &str) -> Option<&'a str> {
    // Trailing slashes are cosmetic on both sides; `/` normalises to the empty
    // string, which makes the host-mount case fall out of the general rule below
    // instead of needing an arm of its own.
    let cgroup = cgroup_path.trim_end_matches('/');
    let root = root.trim_end_matches('/');
    if cgroup == root {
        return Some("");
    }
    let rest = cgroup.strip_prefix(root)?;
    // A PATH-COMPONENT boundary, not a string prefix: `/docker-other` starts with
    // `/docker` and is a different cgroup.
    rest.starts_with('/').then_some(rest)
}

/// PURE: render a slash-trimmed directory back into a path a `read_to_string` can
/// open — which is only ever a question about the ROOT.
///
/// [`cgroup_path_under_root`] normalises `/` to the empty string so the host-mount
/// case falls out of its general rule, and both [`resolve_cgroup_scope`] and
/// [`cgroup_ancestor_chain`] have to undo that before handing a path to anyone.
/// They MUST undo it the same way: the chain walk terminates by comparing its
/// cursor against the mount point, so two spellings of the root would walk past the
/// boundary and read limit files belonging to another mount. One function rather
/// than two closures is what makes that agreement structural.
fn render_cgroup_dir(dir: &str) -> String {
    if dir.is_empty() {
        "/".to_string()
    } else {
        dir.to_string()
    }
}

/// PURE: the directory holding this process's cgroup limit file, from the two
/// `/proc` bodies that decide it.
///
/// # Why `/sys/fs/cgroup` + the cgroup path is NOT the answer
///
/// The naive join is correct only when the cgroup filesystem is mounted at the
/// HIERARCHY ROOT. Under cgroup v1 with a host cgroup namespace — the norm for
/// Docker on the Jetson L4T / Ubuntu 20.04 class this feature exists for — the
/// runtime mounts the container's OWN cgroup as the mount root, so the container
/// reads `/sys/fs/cgroup/memory/memory.limit_in_bytes` while `/proc/self/cgroup`
/// still names `/docker/<id>`. The join then addresses
/// `/sys/fs/cgroup/memory/docker/<id>/…`, which does not exist.
///
/// The failure was SILENT and in the wrong direction: the read failed, the limit
/// read as absent, and [`effective_ram`] let the HOST's total stand — so a 2 GiB
/// container on a 64 GiB machine sized its window from 64 GiB. This is the
/// canonical mountinfo resolution (the OpenJDK `osContainer` algorithm): find the
/// mount, subtract its root, and address the remainder under its mount point.
///
/// Every failure — an unparseable line, no such mount, a cgroup path outside the
/// mount's root — answers `None`, which leaves the machine total standing. That is
/// the same degradation the whole module already documents, never a zero.
///
/// The FIRST resolvable mount wins. That is not "the first cgroup2 line": on a
/// hybrid host (v2 at `/sys/fs/cgroup/unified`, v1 controllers beside it) or any
/// box with several cgroup mounts, a mount whose root does not contain this
/// process's cgroup is skipped rather than answered with.
///
/// # An UNREADABLE mount table is its own answer, and it rides the `Option`
///
/// `mountinfo: None` means `/proc/self/mountinfo` could not be read at all, and
/// falls back to the ROOT-MOUNTED assumption this resolution replaced — nothing is
/// known about the mount, the naive join is the best available guess, and no
/// environment ends up worse than it was. It is deliberately NOT the same as a
/// READABLE table that yields no answer: there the mount table was consulted and
/// says this process's cgroup is not reachable that way, and retrying the naive
/// join is the one case that could produce a WRONG number rather than no number
/// (with a mount rooted at `/docker/abc`, `/sys/fs/cgroup/docker/abc/…` addresses
/// a DIFFERENT cgroup if one happens to exist). A limit read off somebody else's
/// cgroup is worse than none, so that case reports none.
///
/// The absence is in the TYPE rather than in a sentinel string, for the reason
/// `resolve_positive_override` records one module over: a caller cannot recover
/// "unreadable" from a marker value by accident.
pub fn resolve_cgroup_dir(
    mountinfo: Option<&str>,
    proc_self_cgroup: &str,
    hierarchy: CgroupHierarchy,
) -> Option<String> {
    resolve_cgroup_scope(mountinfo, proc_self_cgroup, hierarchy).map(|scope| scope.dir)
}

/// This process's cgroup directory TOGETHER WITH the mount point it was reached
/// through.
///
/// The mount point is not decoration: it is the BOUNDARY of the ancestor walk
/// ([`cgroup_ancestor_chain`]). Above it there is no cgroup filesystem, so a
/// parent cgroup above the mount point is not merely unread — it is not visible to
/// this process at all, and inventing a path for it would address whatever
/// happened to be mounted there instead.
struct CgroupScope {
    /// The directory holding this process's own limit files.
    dir: String,
    /// Where the filesystem carrying `dir` is attached in this process's view.
    mount_point: String,
}

/// PURE: [`resolve_cgroup_dir`], plus the mount point that bounds the ancestor
/// walk. The two are resolved together because they come from ONE mountinfo
/// record, and re-finding the record to ask the second question could pick a
/// different line.
fn resolve_cgroup_scope(
    mountinfo: Option<&str>,
    proc_self_cgroup: &str,
    hierarchy: CgroupHierarchy,
) -> Option<CgroupScope> {
    let cgroup_path = parse_own_cgroup_path(proc_self_cgroup, hierarchy.controller())?;
    let Some(mountinfo) = mountinfo else {
        // The ROOT-MOUNTED guess. Its mount point is the assumed base, which is
        // also the correct walk boundary: the guess claims the hierarchy root is
        // mounted there and nothing above it belongs to this hierarchy.
        return Some(CgroupScope {
            dir: format!(
                "{}{}",
                hierarchy.root_mounted_base(),
                cgroup_path.trim_end_matches('/')
            ),
            mount_point: hierarchy.root_mounted_base().to_string(),
        });
    };
    for entry in mountinfo.lines().filter_map(parse_mountinfo_line) {
        if entry.fstype != hierarchy.fstype() {
            continue;
        }
        // The controller is matched as a WHOLE comma-separated token, for the
        // reason `parse_own_cgroup_path` gives: a substring match would let an
        // unrelated option answer for the memory controller.
        if let Some(required) = hierarchy.required_super_option() {
            if !entry.super_options.split(',').any(|o| o == required) {
                continue;
            }
        }
        let Some(rest) = cgroup_path_under_root(cgroup_path, &entry.root) else {
            continue;
        };
        let base = entry.mount_point.trim_end_matches('/');
        return Some(CgroupScope {
            dir: render_cgroup_dir(&format!("{base}{rest}")),
            mount_point: render_cgroup_dir(base),
        });
    }
    None
}

/// PURE: `dir` and every cgroup directory above it, up to and including
/// `mount_point` — the chain whose limits ALL bind this process.
///
/// A cgroup v2 limit set on a PARENT applies to every descendant, so the ceiling
/// that actually binds is the minimum down the whole chain and not the leaf's own
/// number. Walking is a pure string operation, which is what keeps it
/// oracle-testable without a container.
///
/// The walk STOPS at the mount point, and that is a correctness rule rather than a
/// budget. Under the host-namespace shape the mount is rooted at the container's
/// own cgroup, so the chain is one entry long — which is accurate: from inside that
/// container the ancestors genuinely are not readable, and a path synthesised for
/// one would address a directory belonging to some other mount.
///
/// A `dir` that is not under `mount_point` cannot be walked without leaving the
/// filesystem, so it answers with itself alone rather than climbing out.
fn cgroup_ancestor_chain(dir: &str, mount_point: &str) -> Vec<String> {
    let boundary = mount_point.trim_end_matches('/');
    let leaf = dir.trim_end_matches('/');
    if cgroup_path_under_root(leaf, boundary).is_none() {
        return vec![render_cgroup_dir(leaf)];
    }
    let mut chain = Vec::new();
    let mut cur = leaf;
    loop {
        chain.push(render_cgroup_dir(cur));
        if cur == boundary {
            return chain;
        }
        match cur.rfind('/') {
            // `/foo` — the parent is the root, which normalises to the empty
            // string exactly as `cgroup_path_under_root` treats it.
            Some(0) => cur = "",
            Some(i) => cur = &cur[..i],
            // Unreachable while `cur` is under `boundary` (a path under a boundary
            // always holds the separator that put it there), but a `return` rather
            // than a `panic!`: a reader must degrade, never abort the process it is
            // measuring.
            None => return chain,
        }
    }
}

/// PURE: which of the two sources decides, and what it says.
///
/// The cgroup wins only when it is STRICTLY TIGHTER. A limit at or above the
/// machine's total is not a limit — it is the runtime writing down a number it
/// never intends to enforce — and reporting it as the basis would tell an operator
/// their container decided a cap the machine decided.
///
/// Either input may be absent and the other still answers; both absent is `None`,
/// which the caller turns into the static floor (see
/// [`super::window_cap_from_eff_ram`]) rather than into a refusal.
pub fn effective_ram(
    machine_total: Option<u64>,
    cgroup_limit: Option<u64>,
) -> Option<EffectiveRam> {
    match (machine_total, cgroup_limit) {
        (Some(machine), Some(cgroup)) if cgroup < machine => Some(EffectiveRam {
            bytes: cgroup,
            basis: RamBasis::Cgroup,
        }),
        (Some(machine), _) => Some(EffectiveRam {
            bytes: machine,
            basis: RamBasis::Machine,
        }),
        (None, Some(cgroup)) => Some(EffectiveRam {
            bytes: cgroup,
            basis: RamBasis::Cgroup,
        }),
        (None, None) => None,
    }
}

/// The machine's own total memory in bytes, or `None` where this platform cannot
/// say.
///
/// Linux reads `MemTotal` from `/proc/meminfo` — the kernel's own figure, which is
/// already NET of firmware carve-outs, and that matters here: an "8 GB" Orin NX
/// really reports ~7.4 GiB, so a cap derived from the marketing number would be
/// ~10 % too generous on exactly the most memory-pressured class. macOS reads
/// `hw.memsize`, which is the installed total.
pub fn sample_machine_total() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        let raw = std::fs::read_to_string("/proc/meminfo").ok()?;
        parse_mem_total_kib(&raw).map(|kib| kib.saturating_mul(1024))
    }
    #[cfg(target_vendor = "apple")]
    {
        let mut value: u64 = 0;
        let mut len = std::mem::size_of::<u64>();
        // SAFETY: `sysctlbyname` fills `value` with at most `len` bytes and updates
        // `len`; the two null arguments are the documented "read, do not write"
        // form. The name is a NUL-terminated literal.
        let rc = unsafe {
            libc::sysctlbyname(
                c"hw.memsize".as_ptr(),
                &mut value as *mut u64 as *mut libc::c_void,
                &mut len,
                std::ptr::null_mut(),
                0,
            )
        };
        if rc != 0 || len != std::mem::size_of::<u64>() || value == 0 {
            return None;
        }
        Some(value)
    }
    #[cfg(not(any(target_os = "linux", target_vendor = "apple")))]
    {
        None
    }
}

/// PURE: fold one chain's readings into the ceiling that binds — the same
/// [`CgroupLimit`] lattice a single file answers in, which is why the ladder below
/// matches on it exactly as the one-file version did.
///
/// MINIMUM, because every limit on the chain applies simultaneously — a parent's
/// cap is not advice a child can exceed, and `memory.high` is not advice
/// `memory.max` overrides. Whichever is smallest is the one the process meets
/// first.
///
/// An `Unlimited` anywhere makes the chain ANSWERED without naming a number, which
/// is what lets a positively unlimited v2 hierarchy stop the search while an
/// unreadable one falls through. A single `Bytes` outranks any number of
/// `Unlimited`s: "no limit here" never loosens a limit set elsewhere.
///
/// The fold direction is also the SAFE one, which is worth stating because it is
/// the opposite of the module's other degradations. Everywhere else an unreadable
/// figure leaves the machine total standing and risks a window too LARGE; here a
/// spurious extra limit can only make the window smaller, and the derivation's
/// floor ([`super::FLASHBACK_WINDOW_FLOOR_BYTES`]) puts a hard bound under how
/// much smaller — at worst the static 320 MiB.
fn fold_cgroup_chain(readings: impl IntoIterator<Item = CgroupLimit>) -> CgroupLimit {
    let mut tightest: Option<u64> = None;
    let mut answered = false;
    for reading in readings {
        match reading {
            CgroupLimit::Bytes(bytes) => {
                answered = true;
                tightest = Some(tightest.map_or(bytes, |cur: u64| cur.min(bytes)));
            }
            CgroupLimit::Unlimited => answered = true,
            CgroupLimit::Unknown => {}
        }
    }
    match (tightest, answered) {
        (Some(bytes), _) => CgroupLimit::Bytes(bytes),
        (None, true) => CgroupLimit::Unlimited,
        (None, false) => CgroupLimit::Unknown,
    }
}

/// Read every limit file on one hierarchy's ancestor chain and fold them.
///
/// A file that cannot be read is [`CgroupLimit::Unknown`] — the same answer an
/// empty one gives — so a chain whose upper reaches are unreadable still reports
/// whatever its readable part said.
fn read_cgroup_chain(
    scope: &CgroupScope,
    hierarchy: CgroupHierarchy,
    read_limit_file: &impl Fn(&str) -> Option<String>,
) -> CgroupLimit {
    let readings = cgroup_ancestor_chain(&scope.dir, &scope.mount_point)
        .into_iter()
        .flat_map(|dir| {
            hierarchy.limit_files().iter().map(move |file| {
                // `dir` may be the root `/`, whose trailing slash is the only one
                // the join must not double.
                let base = dir.trim_end_matches('/');
                let path = format!("{base}/{file}");
                read_limit_file(&path).map_or(CgroupLimit::Unknown, |raw| parse_cgroup_limit(&raw))
            })
        });
    fold_cgroup_chain(readings)
}

/// PURE given its reader: the cgroup ceiling binding this process, from the two
/// `/proc` bodies that locate it plus a function that reads a limit file.
///
/// The reader is an ARGUMENT rather than `std::fs` inline for the reason the whole
/// module is built on: the ladder below — v2 before v1, the whole ancestor chain
/// folded to its minimum rather than the leaf alone, `Unlimited` stopping the
/// search where `Unknown` does not — is the part that can be wrong, and with the
/// filesystem injected every branch of it is oracle-testable on a laptop, with no
/// container and no second machine. It also pins the PATHS, which is the defect
/// class this module exists to prevent: a reader whose name says one thing and
/// whose body reads another.
///
/// Note the two are different KINDS of ordering. v2-before-v1 is a precedence: the
/// first hierarchy to answer wins. Within a hierarchy there is none — the chain is
/// read leaf-first only so the read log is legible, and `min` does not care.
///
/// v2 is tried first because it is what every current distribution boots, and
/// because a v1 lookup on a v2 host finds no `memory` controller line at all — so
/// the order costs one absent-file read on a legacy box and nothing anywhere else.
/// A v2 hierarchy that positively reports NO limit stops the search; one that
/// cannot be read at all falls through, because "no such file" is what a legacy
/// v1-only box looks like from up here.
///
/// EVERY failure — no such file, an unreadable body, a sentinel — degrades to
/// `None`, i.e. "no container term", which leaves the machine total standing. That
/// is the direction that keeps a robot's black box: the alternative reading of an
/// unreadable limit is zero, and a zero effRAM would clamp every machine to the
/// floor.
pub fn cgroup_limit_from_sources(
    mountinfo: Option<&str>,
    proc_self_cgroup: &str,
    read_limit_file: impl Fn(&str) -> Option<String>,
) -> Option<u64> {
    // cgroup v2: one line, `0::<path>`, under the unified hierarchy.
    if let Some(scope) = resolve_cgroup_scope(mountinfo, proc_self_cgroup, CgroupHierarchy::V2) {
        match read_cgroup_chain(&scope, CgroupHierarchy::V2, &read_limit_file) {
            CgroupLimit::Bytes(bytes) => return Some(bytes),
            // A POSITIVE "no limit" — stop here rather than falling through to v1,
            // whose files cannot exist on a unified host anyway.
            CgroupLimit::Unlimited => return None,
            CgroupLimit::Unknown => {}
        }
    }
    // cgroup v1: the `memory` controller's own hierarchy.
    let scope = resolve_cgroup_scope(mountinfo, proc_self_cgroup, CgroupHierarchy::V1Memory)?;
    match read_cgroup_chain(&scope, CgroupHierarchy::V1Memory, &read_limit_file) {
        CgroupLimit::Bytes(bytes) => Some(bytes),
        CgroupLimit::Unlimited | CgroupLimit::Unknown => None,
    }
}

/// The cgroup memory ceiling binding this process in bytes, or `None` when there
/// is none, it cannot be read, or the platform has no cgroups.
///
/// The decision is [`cgroup_limit_from_sources`]; this is the thin layer that
/// hands it the real `/proc` and the real filesystem.
///
/// `/proc/self/mountinfo` is read ONCE and threaded through both attempts — the
/// two lookups ask different questions of the same snapshot, and re-reading would
/// let a mount table change between them.
pub fn sample_cgroup_limit() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        let own = std::fs::read_to_string("/proc/self/cgroup").ok()?;
        let mountinfo = std::fs::read_to_string("/proc/self/mountinfo").ok();
        cgroup_limit_from_sources(mountinfo.as_deref(), &own, |path| {
            std::fs::read_to_string(path).ok()
        })
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}

/// The memory this process may actually use — [`sample_machine_total`] and
/// [`sample_cgroup_limit`] folded by [`effective_ram`].
///
/// `None` means neither source could be read. Callers turn that into the
/// static floor, never into a refusal: an unreadable machine must leave the plane
/// exactly as capable as the static default makes it.
pub fn sample_effective_ram() -> Option<EffectiveRam> {
    effective_ram(sample_machine_total(), sample_cgroup_limit())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `MemTotal` out of real `/proc/meminfo` shapes, including the one that
    /// matters: a Jetson's, which is NOT the marketing figure.
    #[test]
    fn mem_total_is_read_with_its_unit_checked() {
        // An 8 GB Orin NX reports ~7.4 GiB after carve-outs. The whole reason the
        // fraction reads the kernel's figure rather than a nameplate.
        let jetson = "MemTotal:        7736516 kB\nMemFree:          123456 kB\n";
        assert_eq!(parse_mem_total_kib(jetson), Some(7_736_516));
        assert!(
            parse_mem_total_kib(jetson).unwrap() * 1024 < 8 * 1024 * 1024 * 1024,
            "anti-tautology: an '8 GB' Jetson really is under 8 GiB, which is why \
             the derivation reads this file rather than a nameplate"
        );
        // The label is not the first line, and the value is tab-padded.
        assert_eq!(
            parse_mem_total_kib("SwapTotal: 0 kB\nMemTotal:\t 16384 kB\n"),
            Some(16_384)
        );
        // No field at all.
        assert_eq!(parse_mem_total_kib("MemFree: 100 kB\n"), None);
        // THE UNIT IS CHECKED. A body claiming bytes is refused rather than read
        // as kibibytes — the 1024x under-report that would silently cap every
        // machine at the floor.
        assert_eq!(parse_mem_total_kib("MemTotal: 16384 B\n"), None);
        assert_eq!(parse_mem_total_kib("MemTotal: 16384\n"), None);
        // Not a number.
        assert_eq!(parse_mem_total_kib("MemTotal: lots kB\n"), None);
        // A PREFIX of the label must not answer for it.
        assert_eq!(parse_mem_total_kib("MemTotalHuge: 8 kB\n"), None);
    }

    /// Every shape a cgroup limit file holds, on both hierarchy versions.
    #[test]
    fn a_cgroup_limit_reads_max_the_v1_sentinel_and_real_numbers_apart() {
        // A real container limit — 2 GiB.
        assert_eq!(
            parse_cgroup_limit("2147483648\n"),
            CgroupLimit::Bytes(2 * 1024 * 1024 * 1024)
        );
        // cgroup v2's literal, with the newline a `read_to_string` really carries.
        assert_eq!(parse_cgroup_limit("max\n"), CgroupLimit::Unlimited);
        assert_eq!(parse_cgroup_limit(" MAX "), CgroupLimit::Unlimited);
        // cgroup v1's sentinel, in BOTH shipping page-size spellings — this is why
        // the rule is a floor and not an equality.
        assert_eq!(
            parse_cgroup_limit("9223372036854771712\n"),
            CgroupLimit::Unlimited,
            "the 4 KiB-page sentinel"
        );
        assert_eq!(
            parse_cgroup_limit("9223372036854710272\n"),
            CgroupLimit::Unlimited,
            "the 64 KiB-page sentinel — a different number, the same meaning"
        );
        // THE FLOOR, both sides. Below it is a limit; at it is the sentinel band.
        assert_eq!(
            parse_cgroup_limit(&(CGROUP_UNLIMITED_FLOOR - 1).to_string()),
            CgroupLimit::Bytes(CGROUP_UNLIMITED_FLOOR - 1)
        );
        assert_eq!(
            parse_cgroup_limit(&CGROUP_UNLIMITED_FLOOR.to_string()),
            CgroupLimit::Unlimited
        );
        // UNKNOWN is not UNLIMITED: an empty or unreadable file says nothing.
        assert_eq!(parse_cgroup_limit(""), CgroupLimit::Unknown);
        assert_eq!(parse_cgroup_limit("   \n"), CgroupLimit::Unknown);
        assert_eq!(parse_cgroup_limit("unbounded"), CgroupLimit::Unknown);
        assert_eq!(parse_cgroup_limit("-1"), CgroupLimit::Unknown);
        // A zero limit is a file this build does not understand, not a machine
        // with no memory — see the doc.
        assert_eq!(parse_cgroup_limit("0\n"), CgroupLimit::Unknown);
    }

    /// This process's own cgroup path, on both hierarchy versions.
    #[test]
    fn the_own_cgroup_path_is_found_per_hierarchy_version() {
        // cgroup v2 — one line, empty controller list. What a container writes.
        let v2 = "0::/kubepods/burstable/pod123/abc\n";
        assert_eq!(
            parse_own_cgroup_path(v2, ""),
            Some("/kubepods/burstable/pod123/abc")
        );
        // …and a v1 lookup finds nothing there, which is what makes trying v2
        // first free rather than merely tidy.
        assert_eq!(parse_own_cgroup_path(v2, "memory"), None);

        // cgroup v1 — one line per hierarchy, comma-separated controllers, and a
        // NAMED hierarchy alongside.
        let v1 = "12:pids:/docker/abc\n\
                  4:cpu,cpuacct,memory:/docker/abc\n\
                  1:name=systemd:/docker/abc\n";
        assert_eq!(parse_own_cgroup_path(v1, "memory"), Some("/docker/abc"));
        // The controller is a WHOLE token. `emory` is a substring of `memory` and
        // must not answer for it; nor must a named hierarchy.
        assert_eq!(parse_own_cgroup_path(v1, "emory"), None);
        assert_eq!(parse_own_cgroup_path(v1, "systemd"), None);
        // A host at the unified root.
        assert_eq!(parse_own_cgroup_path("0::/\n", ""), Some("/"));
        // A path containing a colon survives, because the path is the REMAINDER.
        assert_eq!(
            parse_own_cgroup_path("0::/machine.slice/x:y\n", ""),
            Some("/machine.slice/x:y")
        );
        // Malformed lines are skipped rather than answered with.
        assert_eq!(parse_own_cgroup_path("nonsense\n0::/ok\n", ""), Some("/ok"));
        // A relative path is not a cgroup path.
        assert_eq!(parse_own_cgroup_path("0::relative\n", ""), None);
        assert_eq!(parse_own_cgroup_path("", ""), None);
    }

    /// A `/proc/self/mountinfo` line, built the way the kernel writes one.
    ///
    /// `optional` is the field group that makes indexing impossible — a shared
    /// mount carries `shared:N`, a slave `master:N`, an unbindable one NEITHER —
    /// so every vector below states its own.
    fn mount_line(
        root: &str,
        mount_point: &str,
        optional: &str,
        fstype: &str,
        sup: &str,
    ) -> String {
        format!("31 25 0:26 {root} {mount_point} rw,nosuid{optional} - {fstype} cgroup {sup}")
    }

    /// The mount-aware resolution, on the four shapes a real robot produces.
    ///
    /// This is the mount-aware fix: the naive `/sys/fs/cgroup` + cgroup-path join is
    /// correct only when the filesystem is mounted at the HIERARCHY ROOT, and on
    /// cgroup-v1 Docker with a host cgroup namespace — the norm on the Jetson L4T
    /// class this feature targets — it is not. Every vector is hand-written; the
    /// oracle is the path a `read_to_string` would actually find.
    #[test]
    fn the_cgroup_directory_is_resolved_through_the_mount_that_carries_it() {
        // (1) cgroup v2, PRIVATE cgroup namespace — the modern desk. Mount root is
        // `/`, so the remainder is the whole path and the answer equals the naive
        // join. This is the shape that made the naive join look correct.
        let v2_private = mount_line("/", "/sys/fs/cgroup", " shared:9", "cgroup2", "rw");
        assert_eq!(
            resolve_cgroup_dir(Some(&v2_private), "0::/\n", CgroupHierarchy::V2).as_deref(),
            Some("/sys/fs/cgroup"),
            "a host at the unified root reads the mount point itself"
        );
        assert_eq!(
            resolve_cgroup_dir(
                Some(&v2_private),
                "0::/user.slice/session-3.scope\n",
                CgroupHierarchy::V2
            )
            .as_deref(),
            Some("/sys/fs/cgroup/user.slice/session-3.scope"),
        );

        // (2) THE DEFECT: cgroup v2 with a HOST cgroup namespace. The mount is
        // rooted AT the container's own cgroup, so the limit file sits directly in
        // the mount point — while the naive join addresses
        // `/sys/fs/cgroup/docker/abc/…`, which does not exist, so the limit read as
        // absent and the HOST's total stood.
        let v2_host_ns = mount_line("/docker/abc", "/sys/fs/cgroup", "", "cgroup2", "rw");
        assert_eq!(
            resolve_cgroup_dir(Some(&v2_host_ns), "0::/docker/abc\n", CgroupHierarchy::V2)
                .as_deref(),
            Some("/sys/fs/cgroup"),
            "rooted at our own cgroup ⇒ the mount point IS the directory"
        );

        // (3) The mount root is a strict ANCESTOR: the remainder is what is left
        // below it, NOT the whole path.
        let v2_ancestor = mount_line("/docker", "/sys/fs/cgroup", " master:4", "cgroup2", "rw");
        assert_eq!(
            resolve_cgroup_dir(
                Some(&v2_ancestor),
                "0::/docker/abc/nested\n",
                CgroupHierarchy::V2
            )
            .as_deref(),
            Some("/sys/fs/cgroup/abc/nested"),
        );

        // (4) cgroup v1's `memory` controller, BOTH ways. The controller is named
        // in the SUPER OPTIONS, and the mount point is the controller's own
        // subdirectory rather than `/sys/fs/cgroup`.
        let v1_cgroup = "12:pids:/docker/abc\n4:cpu,cpuacct,memory:/docker/abc\n";
        let v1_root = mount_line("/", "/sys/fs/cgroup/memory", "", "cgroup", "rw,memory");
        assert_eq!(
            resolve_cgroup_dir(Some(&v1_root), v1_cgroup, CgroupHierarchy::V1Memory).as_deref(),
            Some("/sys/fs/cgroup/memory/docker/abc"),
        );
        let v1_host_ns = mount_line(
            "/docker/abc",
            "/sys/fs/cgroup/memory",
            "",
            "cgroup",
            "rw,memory",
        );
        assert_eq!(
            resolve_cgroup_dir(Some(&v1_host_ns), v1_cgroup, CgroupHierarchy::V1Memory).as_deref(),
            Some("/sys/fs/cgroup/memory"),
            "THE Jetson/L4T Docker shape — the one the naive join could never read"
        );

        // A cgroup mount is not ENOUGH: a v1 mount for another controller must not
        // answer for `memory`, and the option is matched as a WHOLE token so
        // `memoryfoo` cannot either.
        let v1_wrong = mount_line("/", "/sys/fs/cgroup/pids", "", "cgroup", "rw,pids");
        assert_eq!(
            resolve_cgroup_dir(Some(&v1_wrong), v1_cgroup, CgroupHierarchy::V1Memory),
            None
        );
        let v1_substring = mount_line(
            "/",
            "/sys/fs/cgroup/memoryfoo",
            "",
            "cgroup",
            "rw,memoryfoo",
        );
        assert_eq!(
            resolve_cgroup_dir(Some(&v1_substring), v1_cgroup, CgroupHierarchy::V1Memory),
            None
        );
        // …and the two hierarchies never answer for each other.
        assert_eq!(
            resolve_cgroup_dir(Some(&v1_root), v1_cgroup, CgroupHierarchy::V2),
            None
        );
        assert_eq!(
            resolve_cgroup_dir(Some(&v2_private), "0::/\n", CgroupHierarchy::V1Memory),
            None
        );
    }

    /// The failure arms — each answers `None`, which leaves the machine total
    /// standing, and none of them abandons the rest of the file.
    #[test]
    fn an_unresolvable_mount_table_degrades_instead_of_guessing() {
        let v1_cgroup = "4:memory:/docker/abc\n";

        // NO cgroup mount at all: a table full of ordinary filesystems.
        let no_cgroup = format!(
            "{}\n{}",
            mount_line("/", "/", " shared:1", "ext4", "rw"),
            mount_line("/", "/proc", " shared:2", "proc", "rw"),
        );
        assert_eq!(
            resolve_cgroup_dir(Some(&no_cgroup), "0::/\n", CgroupHierarchy::V2),
            None
        );
        assert_eq!(
            resolve_cgroup_dir(Some(""), "0::/\n", CgroupHierarchy::V2),
            None
        );

        // A MALFORMED LINE MID-FILE IS SKIPPED, not fatal. This is the bug class
        // `parse_own_cgroup_path` already records: a `?` here would answer `None`
        // for a process that is very much in a cgroup — a silently unbudgeted
        // container. The real mount sits AFTER the junk, so a parser that stopped
        // would never reach it.
        let junk_then_real = format!(
            "nonsense\n31 25 0:26 / /sys/fs/cgroup rw\n\n{}",
            mount_line("/", "/sys/fs/cgroup", "", "cgroup2", "rw"),
        );
        assert_eq!(
            resolve_cgroup_dir(Some(&junk_then_real), "0::/pod\n", CgroupHierarchy::V2).as_deref(),
            Some("/sys/fs/cgroup/pod"),
        );

        // Our cgroup is OUTSIDE the mount's root — the mount is rooted BELOW us, or
        // beside us. Our own directory is not visible through it, so there is no
        // valid answer; a path-COMPONENT boundary is what separates `/docker` from
        // `/docker-other`.
        let below = mount_line("/docker/abc/deep", "/sys/fs/cgroup", "", "cgroup2", "rw");
        assert_eq!(
            resolve_cgroup_dir(Some(&below), "0::/docker/abc\n", CgroupHierarchy::V2),
            None
        );
        let beside = mount_line("/docker", "/sys/fs/cgroup", "", "cgroup2", "rw");
        assert_eq!(
            resolve_cgroup_dir(Some(&beside), "0::/docker-other\n", CgroupHierarchy::V2),
            None
        );

        // A mount table that names no controller line for us stops before it looks
        // at the mounts at all.
        assert_eq!(
            resolve_cgroup_dir(Some(&no_cgroup), v1_cgroup, CgroupHierarchy::V2),
            None
        );

        // SEVERAL cgroup mounts: the first RESOLVABLE one wins, not the first of
        // the right type. A hybrid host mounts cgroup2 somewhere the container's
        // cgroup does not live.
        let hybrid = format!(
            "{}\n{}",
            mount_line("/other", "/sys/fs/cgroup/unified", "", "cgroup2", "rw"),
            mount_line("/", "/sys/fs/cgroup", "", "cgroup2", "rw"),
        );
        assert_eq!(
            resolve_cgroup_dir(Some(&hybrid), "0::/pod\n", CgroupHierarchy::V2).as_deref(),
            Some("/sys/fs/cgroup/pod"),
        );
    }

    /// The ` - ` separator is found by SCANNING, because the optional fields before
    /// it vary in count on real kernels.
    #[test]
    fn the_mountinfo_separator_is_found_rather_than_indexed() {
        // ZERO optional fields, ONE, and TWO — the same mount, the same answer.
        for optional in ["", " shared:9", " shared:9 master:4"] {
            let line = mount_line("/", "/sys/fs/cgroup", optional, "cgroup2", "rw");
            assert_eq!(
                resolve_cgroup_dir(Some(&line), "0::/pod\n", CgroupHierarchy::V2).as_deref(),
                Some("/sys/fs/cgroup/pod"),
                "optional fields {optional:?} must not move the separator's meaning"
            );
        }
        // The MOUNT POINT may itself be a lone `-`-free path with hyphens; the
        // separator is a BARE `-` token, so a hyphenated name is not one.
        let hyphenated = mount_line("/", "/sys/fs/cgroup-v2", "", "cgroup2", "rw");
        assert_eq!(
            resolve_cgroup_dir(Some(&hyphenated), "0::/pod\n", CgroupHierarchy::V2).as_deref(),
            Some("/sys/fs/cgroup-v2/pod"),
        );
        // A line that never reaches its separator is malformed, and skipped.
        let truncated = "31 25 0:26 / /sys/fs/cgroup rw shared:9";
        assert_eq!(
            resolve_cgroup_dir(Some(truncated), "0::/pod\n", CgroupHierarchy::V2),
            None
        );
        // …as is one whose post-separator fields are short.
        let short_tail = "31 25 0:26 / /sys/fs/cgroup rw - cgroup2";
        assert_eq!(
            resolve_cgroup_dir(Some(short_tail), "0::/pod\n", CgroupHierarchy::V2),
            None
        );
    }

    /// An UNREADABLE mount table falls back to the root-mounted assumption — and
    /// that is NOT the same as a readable one with no answer.
    ///
    /// The distinction is the whole of the fallback's scope: with nothing known
    /// about the mount, the naive join is the best available guess and no
    /// environment is worse than it was; with the table CONSULTED and silent,
    /// retrying it could address a DIFFERENT cgroup's limit file.
    #[test]
    fn an_unreadable_mount_table_falls_back_to_the_root_mounted_assumption() {
        assert_eq!(
            resolve_cgroup_dir(None, "0::/docker/abc\n", CgroupHierarchy::V2).as_deref(),
            Some("/sys/fs/cgroup/docker/abc"),
        );
        assert_eq!(
            resolve_cgroup_dir(None, "0::/\n", CgroupHierarchy::V2).as_deref(),
            Some("/sys/fs/cgroup"),
            "the trailing slash of the root cgroup is normalised away"
        );
        assert_eq!(
            resolve_cgroup_dir(None, "4:memory:/docker/abc\n", CgroupHierarchy::V1Memory)
                .as_deref(),
            Some("/sys/fs/cgroup/memory/docker/abc"),
            "the v1 base is the CONTROLLER's directory, not the hierarchy root"
        );
        // Still `None` when this process is in no such hierarchy at all — the
        // fallback guesses a MOUNT, never a cgroup.
        assert_eq!(
            resolve_cgroup_dir(None, "0::/\n", CgroupHierarchy::V1Memory),
            None
        );

        // The CONTRAST, in the same body: a READABLE table with no cgroup mount
        // answers `None` rather than the guess above.
        let no_cgroup = mount_line("/", "/", " shared:1", "ext4", "rw");
        assert_eq!(
            resolve_cgroup_dir(Some(&no_cgroup), "0::/docker/abc\n", CgroupHierarchy::V2),
            None,
            "a consulted mount table that says nothing must not be re-guessed"
        );
    }

    /// `/proc/self/mountinfo`'s `\ooo` escaping, undone — and the malformed shapes
    /// that must stay literal rather than becoming a wrong path.
    #[test]
    fn mountinfo_octal_escapes_are_decoded_and_malformed_ones_stay_literal() {
        // The four bytes the kernel escapes, which is the whole live set.
        assert_eq!(
            decode_mountinfo_escapes(r"/sys/fs/my\040cgroup"),
            "/sys/fs/my cgroup"
        );
        assert_eq!(decode_mountinfo_escapes(r"/a\011b"), "/a\tb");
        assert_eq!(decode_mountinfo_escapes(r"/a\012b"), "/a\nb");
        assert_eq!(decode_mountinfo_escapes(r"/a\134b"), r"/a\b");
        // Several in one field, and adjacent.
        assert_eq!(decode_mountinfo_escapes(r"/x\040y\040z\040"), "/x y z ");
        assert_eq!(decode_mountinfo_escapes(r"/\040\040"), "/  ");

        // NO escape ⇒ borrowed, not rebuilt. The common case must not allocate.
        assert!(matches!(
            decode_mountinfo_escapes("/sys/fs/cgroup"),
            Cow::Borrowed(_)
        ));
        assert!(matches!(
            decode_mountinfo_escapes(r"/a\040b"),
            Cow::Owned(_)
        ));

        // MALFORMED: each keeps its backslash rather than inventing a byte, and
        // none of them panics. A `\` the kernel did not write is a `\`.
        assert_eq!(decode_mountinfo_escapes(r"/a\04b"), r"/a\04b", "two digits");
        assert_eq!(
            decode_mountinfo_escapes(r"/a\0x1b"),
            r"/a\0x1b",
            "not octal"
        );
        assert_eq!(
            decode_mountinfo_escapes(r"/a\09b"),
            r"/a\09b",
            "9 is not octal"
        );
        assert_eq!(
            decode_mountinfo_escapes(r"/a\04"),
            r"/a\04",
            "truncated at the end"
        );
        assert_eq!(
            decode_mountinfo_escapes(r"/a\"),
            r"/a\",
            "a lone trailing backslash"
        );
        assert_eq!(
            decode_mountinfo_escapes(r"\\040"),
            r"\ ",
            "the escape of a backslash is not the start of another"
        );
        // `\400`..`\777` name no byte. Left literal rather than truncated to one.
        assert_eq!(decode_mountinfo_escapes(r"/a\400b"), r"/a\400b");
        assert_eq!(decode_mountinfo_escapes(r"/a\777b"), r"/a\777b");
        // Decoding is BYTE-wise, so a multi-byte character survives beside an
        // escape: `é` is written raw, only the space is escaped. `\303\251` is
        // that same `é` spelt as two escapes, which must decode to one character.
        assert_eq!(decode_mountinfo_escapes(r"/caf\303\251\040x"), "/café x");
        assert_eq!(decode_mountinfo_escapes("/café\\040x"), "/café x");

        // A decode whose bytes are not TEXT is discarded for the original: `\377`
        // is a byte no UTF-8 path can hold on its own. The kernel cannot produce
        // this from a real path, and a lossy replacement would invent a path that
        // matches nothing — so the field stays exactly as it arrived, the safe
        // answer rather than a new failure.
        assert_eq!(decode_mountinfo_escapes(r"/a\377b"), r"/a\377b");
    }

    /// THE DEFECT this pins: a cgroup mount whose path holds a space.
    ///
    /// `/proc/self/mountinfo` escapes it; `/proc/self/cgroup` does not. Without the
    /// decode the two spellings of one path never compare equal, the resolution
    /// answers `None`, and a container that has deliberately been given a limit
    /// silently sizes its window from the HOST's RAM.
    #[test]
    fn a_mount_path_containing_a_space_still_matches_its_cgroup_path() {
        // (a) the ROOT field carries the space — the host-namespace container
        // whose own cgroup name has one. This is the arm that fails outright
        // without the decode.
        let escaped_root = mount_line(
            r"/machine.slice/my\040unit",
            "/sys/fs/cgroup",
            "",
            "cgroup2",
            "rw",
        );
        assert_eq!(
            resolve_cgroup_dir(
                Some(&escaped_root),
                "0::/machine.slice/my unit\n",
                CgroupHierarchy::V2
            )
            .as_deref(),
            Some("/sys/fs/cgroup"),
            "rooted at our own cgroup — the escaped root must compare equal to the \
             unescaped path `/proc/self/cgroup` carries"
        );

        // (b) the MOUNT POINT field carries the space. This one MATCHED before the
        // fix (the root is `/`), but resolved to a directory spelt `\040`, which
        // no `read_to_string` could ever open — the same silent fallback by a
        // different route.
        let escaped_mount_point =
            mount_line("/", r"/sys/fs/cgroup\040alt", " shared:9", "cgroup2", "rw");
        assert_eq!(
            resolve_cgroup_dir(Some(&escaped_mount_point), "0::/pod\n", CgroupHierarchy::V2)
                .as_deref(),
            Some("/sys/fs/cgroup alt/pod"),
        );

        // A MALFORMED escape is not decoded, so it stays a literal `\04` — which
        // matches nothing and degrades to `None`. Safe, and above all not a panic.
        let malformed = mount_line(
            r"/machine.slice/my\04unit",
            "/sys/fs/cgroup",
            "",
            "cgroup2",
            "rw",
        );
        assert_eq!(
            resolve_cgroup_dir(
                Some(&malformed),
                "0::/machine.slice/my unit\n",
                CgroupHierarchy::V2
            ),
            None
        );

        // ANTI-TAUTOLOGY: decoding must not soften the path-COMPONENT boundary.
        // `/docker\040a` and `/docker a-other` are still different cgroups.
        let beside = mount_line(r"/docker\040a", "/sys/fs/cgroup", "", "cgroup2", "rw");
        assert_eq!(
            resolve_cgroup_dir(Some(&beside), "0::/docker a-other\n", CgroupHierarchy::V2),
            None
        );
    }

    /// The ancestor chain: every directory whose limits bind, and no more.
    #[test]
    fn the_ancestor_chain_walks_up_to_the_mount_point_and_stops_there() {
        // The ordinary nested shape — leaf first, mount point last.
        assert_eq!(
            cgroup_ancestor_chain("/sys/fs/cgroup/kubepods/burstable/pod1", "/sys/fs/cgroup"),
            vec![
                "/sys/fs/cgroup/kubepods/burstable/pod1",
                "/sys/fs/cgroup/kubepods/burstable",
                "/sys/fs/cgroup/kubepods",
                "/sys/fs/cgroup",
            ]
        );
        // THE HOST-NAMESPACE SHAPE: the mount is rooted at our own cgroup, so the
        // leaf IS the mount point and there is exactly one directory to read. The
        // ancestors bind but are genuinely not visible from inside.
        assert_eq!(
            cgroup_ancestor_chain("/sys/fs/cgroup", "/sys/fs/cgroup"),
            vec!["/sys/fs/cgroup"]
        );
        // A mount point of `/` — the chain still terminates, at the root.
        assert_eq!(cgroup_ancestor_chain("/a/b", "/"), vec!["/a/b", "/a", "/"]);
        assert_eq!(cgroup_ancestor_chain("/", "/"), vec!["/"]);
        // Trailing slashes are cosmetic on both sides.
        assert_eq!(
            cgroup_ancestor_chain("/sys/fs/cgroup/pod/", "/sys/fs/cgroup/"),
            vec!["/sys/fs/cgroup/pod", "/sys/fs/cgroup"]
        );
        // A leaf that is not under the boundary cannot be walked without leaving
        // the filesystem, so it answers with itself ALONE — never by climbing out
        // into whatever else is mounted above.
        assert_eq!(
            cgroup_ancestor_chain("/elsewhere/pod", "/sys/fs/cgroup"),
            vec!["/elsewhere/pod"]
        );
        // …and a COMPONENT boundary decides that, not a string prefix.
        assert_eq!(
            cgroup_ancestor_chain("/sys/fs/cgroup-other/pod", "/sys/fs/cgroup"),
            vec!["/sys/fs/cgroup-other/pod"]
        );
    }

    /// The fold: the tightest limit on the chain binds, and "no limit" is not
    /// "could not read".
    #[test]
    fn a_chain_folds_to_its_tightest_limit_and_keeps_unlimited_apart_from_unknown() {
        use CgroupLimit::{Bytes, Unknown, Unlimited};

        // MINIMUM, and the position of the winner does not matter.
        assert_eq!(
            fold_cgroup_chain([Bytes(8 * GIB), Bytes(2 * GIB), Bytes(4 * GIB)]),
            Bytes(2 * GIB)
        );
        assert_eq!(
            fold_cgroup_chain([Bytes(2 * GIB), Bytes(8 * GIB)]),
            Bytes(2 * GIB)
        );
        // An `Unlimited` beside a real limit does not loosen it.
        assert_eq!(
            fold_cgroup_chain([Unlimited, Bytes(2 * GIB), Unlimited]),
            Bytes(2 * GIB)
        );
        // Nor does an unreadable one.
        assert_eq!(
            fold_cgroup_chain([Unknown, Bytes(2 * GIB), Unknown]),
            Bytes(2 * GIB)
        );
        // ANSWERED, with no number: this hierarchy really says "no limit", which
        // is what stops the search rather than continuing to v1.
        assert_eq!(
            fold_cgroup_chain([Unlimited, Unknown, Unlimited]),
            Unlimited
        );
        // NOTHING answered — a legacy box with no v2 files at all.
        assert_eq!(fold_cgroup_chain([Unknown, Unknown]), Unknown);
        assert_eq!(fold_cgroup_chain([]), Unknown);
        // Equal limits fold to that limit rather than to nothing.
        assert_eq!(
            fold_cgroup_chain([Bytes(2 * GIB), Bytes(2 * GIB)]),
            Bytes(2 * GIB)
        );
        // A SINGLE reading folds to itself — the chain of length one every
        // host-namespace container walks, and the shape that must behave
        // exactly as a one-file read does.
        assert_eq!(fold_cgroup_chain([Bytes(2 * GIB)]), Bytes(2 * GIB));
        assert_eq!(fold_cgroup_chain([Unlimited]), Unlimited);
        assert_eq!(fold_cgroup_chain([Unknown]), Unknown);
    }

    /// A hand-built cgroup filesystem.
    ///
    /// Only the paths a vector names exist, and every read is RECORDED — so a test
    /// can pin not just the answer but WHICH files the ladder asked for, which is
    /// the half a value assertion cannot see (a walk that climbed above its mount
    /// point would still return the right number on most vectors).
    struct FakeCgroupFs {
        files: std::collections::HashMap<String, String>,
        reads: std::cell::RefCell<Vec<String>>,
    }

    impl FakeCgroupFs {
        fn new(files: &[(&str, &str)]) -> Self {
            Self {
                files: files
                    .iter()
                    .map(|(p, b)| ((*p).to_string(), (*b).to_string()))
                    .collect(),
                reads: std::cell::RefCell::new(Vec::new()),
            }
        }

        fn reader(&self) -> impl Fn(&str) -> Option<String> + '_ {
            move |path: &str| {
                self.reads.borrow_mut().push(path.to_string());
                self.files.get(path).cloned()
            }
        }

        /// Every path asked for, in order.
        fn reads(&self) -> Vec<String> {
            self.reads.borrow().clone()
        }
    }

    const GIB: u64 = 1024 * 1024 * 1024;

    /// A cgroup-v2 mount in the modern PRIVATE-namespace shape: rooted at `/`,
    /// attached at `/sys/fs/cgroup`, so the whole ancestor chain is visible.
    fn v2_private_mount() -> String {
        mount_line("/", "/sys/fs/cgroup", " shared:9", "cgroup2", "rw")
    }

    /// THE DEFECT THIS PINS, both halves: a limit on an ANCESTOR and a `memory.high`
    /// throttle each bind this process, and reading only the leaf's `memory.max`
    /// misses both.
    #[test]
    fn an_ancestor_cap_and_a_memory_high_throttle_each_bind_the_window() {
        let mounts = v2_private_mount();
        let own = "0::/kubepods/burstable/pod123\n";

        // (a) ANCESTOR. The leaf is uncapped — which is exactly what a Kubernetes
        // pod's own cgroup looks like when the limit sits on the QoS parent — so
        // reading the leaf alone answered "no limit" and let the HOST's total
        // stand.
        let ancestor = FakeCgroupFs::new(&[
            (
                "/sys/fs/cgroup/kubepods/burstable/pod123/memory.max",
                "max\n",
            ),
            (
                "/sys/fs/cgroup/kubepods/burstable/memory.max",
                "2147483648\n",
            ),
        ]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&mounts), own, ancestor.reader()),
            Some(2 * GIB),
            "a cap on the parent binds this process too"
        );

        // (b) MEMORY.HIGH below max. The throttle is what an allocation meets
        // first, so a window sized from `memory.max` is one the kernel spends
        // every cycle reclaiming.
        let throttled = FakeCgroupFs::new(&[
            (
                "/sys/fs/cgroup/kubepods/burstable/pod123/memory.max",
                "8589934592\n",
            ),
            (
                "/sys/fs/cgroup/kubepods/burstable/pod123/memory.high",
                "2147483648\n",
            ),
        ]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&mounts), own, throttled.reader()),
            Some(2 * GIB),
            "memory.high below memory.max is the ceiling that binds"
        );

        // (c) …and ABOVE max it is not a limit at all, so `memory.max` keeps the
        // answer. The min is a min in BOTH directions — this is the arm that
        // stops the fold from simply preferring `memory.high`.
        let loose_high = FakeCgroupFs::new(&[
            (
                "/sys/fs/cgroup/kubepods/burstable/pod123/memory.max",
                "2147483648\n",
            ),
            (
                "/sys/fs/cgroup/kubepods/burstable/pod123/memory.high",
                "8589934592\n",
            ),
        ]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&mounts), own, loose_high.reader()),
            Some(2 * GIB)
        );

        // (d) The LEAF still wins when it is the tightest — the ancestor walk adds
        // limits, it does not replace the one that already worked.
        let leaf_tightest = FakeCgroupFs::new(&[
            (
                "/sys/fs/cgroup/kubepods/burstable/pod123/memory.max",
                "1073741824\n",
            ),
            (
                "/sys/fs/cgroup/kubepods/burstable/memory.max",
                "2147483648\n",
            ),
            ("/sys/fs/cgroup/kubepods/memory.max", "4294967296\n"),
        ]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&mounts), own, leaf_tightest.reader()),
            Some(GIB)
        );

        // (e) The chain really is walked to the mount point: a cap three levels up,
        // with everything below it unset (ABSENT, not `max` — the shape a plain
        // `systemd` slice produces).
        let far_ancestor = FakeCgroupFs::new(&[("/sys/fs/cgroup/memory.max", "2147483648\n")]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&mounts), own, far_ancestor.reader()),
            Some(2 * GIB)
        );

        // (f) BOTH HALVES AT ONCE — the arm
        // neither half of the chain read passes alone. A throttle set on a PARENT slice is
        // where systemd's `DefaultMemoryHigh` and a Kubernetes QoS class both put
        // it: reading only the leaf never opens the directory, and reading only
        // `memory.max` never opens the file.
        let ancestor_high = FakeCgroupFs::new(&[
            (
                "/sys/fs/cgroup/kubepods/burstable/pod123/memory.max",
                "max\n",
            ),
            ("/sys/fs/cgroup/kubepods/memory.high", "2147483648\n"),
        ]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&mounts), own, ancestor_high.reader()),
            Some(2 * GIB),
            "a memory.high two directories up binds this process"
        );
    }

    /// WHICH files the ladder reads — the half an answer cannot show.
    #[test]
    fn the_ladder_reads_the_chain_it_claims_to_and_stops_at_the_mount_point() {
        // The HOST-NAMESPACE shape: the mount is rooted at our own cgroup, so
        // there is exactly one directory. A walk that climbed anyway would read
        // `/sys/fs/memory.max` and `/memory.max` — directories belonging to some
        // other mount entirely.
        let host_ns = mount_line("/docker/abc", "/sys/fs/cgroup", "", "cgroup2", "rw");
        let fs = FakeCgroupFs::new(&[("/sys/fs/cgroup/memory.max", "2147483648\n")]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&host_ns), "0::/docker/abc\n", fs.reader()),
            Some(2 * GIB)
        );
        assert_eq!(
            fs.reads(),
            vec!["/sys/fs/cgroup/memory.max", "/sys/fs/cgroup/memory.high"],
            "one directory, both v2 files, and nothing above the mount point"
        );

        // The nested shape reads BOTH files in EVERY directory up to the mount
        // point, in leaf-first order.
        let nested = FakeCgroupFs::new(&[("/sys/fs/cgroup/a/memory.max", "2147483648\n")]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&v2_private_mount()), "0::/a/b\n", nested.reader()),
            Some(2 * GIB)
        );
        assert_eq!(
            nested.reads(),
            vec![
                "/sys/fs/cgroup/a/b/memory.max",
                "/sys/fs/cgroup/a/b/memory.high",
                "/sys/fs/cgroup/a/memory.max",
                "/sys/fs/cgroup/a/memory.high",
                "/sys/fs/cgroup/memory.max",
                "/sys/fs/cgroup/memory.high",
            ],
        );
    }

    /// The v2-before-v1 ladder: a POSITIVE "no limit" stops it, an unreadable
    /// hierarchy does not.
    #[test]
    fn an_unlimited_v2_hierarchy_stops_the_ladder_and_an_unreadable_one_falls_through() {
        let v1_cgroup = "0::/docker/abc\n12:pids:/docker/abc\n4:cpu,memory:/docker/abc\n";
        let mounts = format!(
            "{}\n{}",
            v2_private_mount(),
            mount_line("/", "/sys/fs/cgroup/memory", "", "cgroup", "rw,memory"),
        );

        // (a) v2 says "no limit" — and it is BELIEVED. v1's file exists in this
        // vector and must NOT be consulted: on a unified host it cannot be a real
        // budget, and reading it would let a stale v1 tree override the live
        // answer.
        let unlimited_v2 = FakeCgroupFs::new(&[
            ("/sys/fs/cgroup/docker/abc/memory.max", "max\n"),
            ("/sys/fs/cgroup/docker/memory.max", "max\n"),
            ("/sys/fs/cgroup/memory.max", "max\n"),
            (
                "/sys/fs/cgroup/memory/docker/abc/memory.limit_in_bytes",
                "2147483648\n",
            ),
        ]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&mounts), v1_cgroup, unlimited_v2.reader()),
            None
        );
        assert!(
            !unlimited_v2
                .reads()
                .iter()
                .any(|p| p.contains("memory.limit_in_bytes")),
            "a positively unlimited v2 hierarchy must stop the ladder, not fall \
             through: {:?}",
            unlimited_v2.reads()
        );

        // (b) v2 is UNREADABLE — no files at all, the legacy-box shape. That says
        // nothing, so v1 still gets its turn, ancestors included.
        let v1_only = FakeCgroupFs::new(&[(
            "/sys/fs/cgroup/memory/docker/memory.limit_in_bytes",
            "2147483648\n",
        )]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&mounts), v1_cgroup, v1_only.reader()),
            Some(2 * GIB),
            "a v1 ancestor's hard limit binds exactly as a v2 one does"
        );

        // WHICH files v1 opened, exactly: its hard limit in every directory up to
        // the mount point and NOTHING else. An `all(!contains("soft_limit"))` would
        // pass a reader that also opened `memory.high` — a v2 file with no meaning
        // on a v1 hierarchy — so the pin is the whole vector. The v2 attempt's own
        // reads are filtered out by prefix; `/sys/fs/cgroup/memory.max` is not
        // under `/sys/fs/cgroup/memory/`.
        let v1_reads: Vec<String> = v1_only
            .reads()
            .into_iter()
            .filter(|p| p.starts_with("/sys/fs/cgroup/memory/"))
            .collect();
        assert_eq!(
            v1_reads,
            vec![
                "/sys/fs/cgroup/memory/docker/abc/memory.limit_in_bytes",
                "/sys/fs/cgroup/memory/docker/memory.limit_in_bytes",
                "/sys/fs/cgroup/memory/memory.limit_in_bytes",
            ],
            "v1 reads its hard limit and no other file — not the soft limit (a \
             reclaim priority the kernel does not enforce as a ceiling), and not \
             v2's memory.high"
        );

        // (c) v1's sentinel is still read as "no limit" through the chain.
        let v1_unlimited = FakeCgroupFs::new(&[(
            "/sys/fs/cgroup/memory/docker/abc/memory.limit_in_bytes",
            "9223372036854771712\n",
        )]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&mounts), v1_cgroup, v1_unlimited.reader()),
            None
        );
    }

    /// ANTI-TAUTOLOGY: an uncapped machine must still report its own RAM.
    ///
    /// Every arm above asserts that a limit is FOUND, and a reader that invented
    /// limits would pass all of them. This is the other side: with nothing capping
    /// this process, the cgroup term must contribute nothing at all and the
    /// machine's total must stand — which is the shape every desk and every
    /// uncontainerised robot runs in.
    #[test]
    fn an_uncapped_chain_contributes_no_term_and_the_machine_total_stands() {
        let mounts = v2_private_mount();

        // Nothing is capped: no limit file exists anywhere on the chain.
        let bare = FakeCgroupFs::new(&[]);
        assert_eq!(
            cgroup_limit_from_sources(Some(&mounts), "0::/user.slice\n", bare.reader()),
            None
        );
        assert!(
            !bare.reads().is_empty(),
            "the reader really was consulted — an empty read log would make the \
             `None` above vacuous"
        );
        assert_eq!(
            effective_ram(
                Some(64 * GIB),
                cgroup_limit_from_sources(Some(&mounts), "0::/user.slice\n", bare.reader())
            ),
            Some(EffectiveRam {
                bytes: 64 * GIB,
                basis: RamBasis::Machine
            }),
            "an uncapped machine sizes its window from its own RAM"
        );

        // …and the POSITIVE control, in the same body: put a cap on the chain and
        // the very same fold reports it, with the basis flipping to the cgroup.
        let capped =
            FakeCgroupFs::new(&[("/sys/fs/cgroup/user.slice/memory.high", "2147483648\n")]);
        assert_eq!(
            effective_ram(
                Some(64 * GIB),
                cgroup_limit_from_sources(Some(&mounts), "0::/user.slice\n", capped.reader())
            ),
            Some(EffectiveRam {
                bytes: 2 * GIB,
                basis: RamBasis::Cgroup
            }),
        );

        // A hierarchy this process is not in at all contributes nothing either,
        // and asks for no file.
        let no_hierarchy = FakeCgroupFs::new(&[]);
        assert_eq!(
            cgroup_limit_from_sources(Some(""), "0::/pod\n", no_hierarchy.reader()),
            None,
            "a readable mount table with no cgroup mount resolves nothing"
        );
        assert!(no_hierarchy.reads().is_empty());
    }

    /// An UNREADABLE mount table still walks a chain — under the root-mounted
    /// guess, whose assumed base is also its correct walk boundary.
    #[test]
    fn the_root_mounted_fallback_walks_the_chain_below_its_assumed_base() {
        let fs = FakeCgroupFs::new(&[("/sys/fs/cgroup/docker/memory.max", "2147483648\n")]);
        assert_eq!(
            cgroup_limit_from_sources(None, "0::/docker/abc\n", fs.reader()),
            Some(2 * GIB)
        );
        assert_eq!(
            fs.reads(),
            vec![
                "/sys/fs/cgroup/docker/abc/memory.max",
                "/sys/fs/cgroup/docker/abc/memory.high",
                "/sys/fs/cgroup/docker/memory.max",
                "/sys/fs/cgroup/docker/memory.high",
                "/sys/fs/cgroup/memory.max",
                "/sys/fs/cgroup/memory.high",
            ],
            "the guess claims the hierarchy root is at /sys/fs/cgroup, so the walk \
             stops there and never reads /sys/fs or /"
        );
    }

    /// The fold: the tighter source wins, and it says which one it was.
    #[test]
    fn the_effective_figure_is_the_tighter_source_and_names_itself() {
        // A container on a big machine — the case the cgroup term exists for.
        assert_eq!(
            effective_ram(Some(64 * GIB), Some(2 * GIB)),
            Some(EffectiveRam {
                bytes: 2 * GIB,
                basis: RamBasis::Cgroup
            })
        );
        // An unlimited (or absent) container reads as no term at all.
        assert_eq!(
            effective_ram(Some(64 * GIB), None),
            Some(EffectiveRam {
                bytes: 64 * GIB,
                basis: RamBasis::Machine
            })
        );
        // A limit LOOSER than the machine is not a limit, and must not be reported
        // as the basis — an operator would go looking for a container that is not
        // deciding anything.
        assert_eq!(
            effective_ram(Some(8 * GIB), Some(64 * GIB)),
            Some(EffectiveRam {
                bytes: 8 * GIB,
                basis: RamBasis::Machine
            })
        );
        // EQUAL is not TIGHTER — the machine keeps the attribution.
        assert_eq!(
            effective_ram(Some(8 * GIB), Some(8 * GIB)),
            Some(EffectiveRam {
                bytes: 8 * GIB,
                basis: RamBasis::Machine
            })
        );
        // One byte tighter IS tighter. The boundary, both sides.
        assert_eq!(
            effective_ram(Some(8 * GIB), Some(8 * GIB - 1)),
            Some(EffectiveRam {
                bytes: 8 * GIB - 1,
                basis: RamBasis::Cgroup
            })
        );
        // A machine that cannot report its total but IS in a cgroup still answers.
        assert_eq!(
            effective_ram(None, Some(2 * GIB)),
            Some(EffectiveRam {
                bytes: 2 * GIB,
                basis: RamBasis::Cgroup
            })
        );
        // Neither: the caller falls back to the static floor.
        assert_eq!(effective_ram(None, None), None);
    }
}
