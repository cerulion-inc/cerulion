//! The cfg audit — nothing PORTABLE in `cerulion_core` may name a
//! `#[cfg(unix)]`-only module, in code OR in an intra-doc link.
//!
//! # Why a walk, and why it needs two arms
//!
//! `cerulion_core` is supposed to compile for non-unix targets (multi-process ships
//! Windows the monolith fallback, which implies the crate builds there), and a
//! real `cargo check --target x86_64-pc-windows-msvc` is impossible from a mac
//! host — MEASURED: `iceoryx2-pal-posix`'s build script runs bindgen against
//! `src/c/posix.h` and dies with `fatal error: 'errno.h' file not found`. So
//! the property has no build gate in CI, and without this walk it could only
//! be probed by hand.
//!
//! A hand probe that configures the unix-only MODULES out and
//! builds the crate is not faithful: it leaves the `unix` predicate
//! itself TRUE, so a reference that IS correctly `#[cfg(unix)]`-gated stays
//! compiled while the module it names does not. It therefore cannot tell a
//! gated reference from an ungated one: its 8 `crate::trace_ring` errors
//! read as "named unconditionally" when every one of them is gated.
//! MEASURED with the predicate made false instead (`#[cfg(unix)]` -> never,
//! `#[cfg(not(unix))]` -> always): ZERO errors.
//!
//! The class that CAN break is invisible to any `cargo check` probe: a portable
//! item carrying an intra-doc link into a unix-only module makes
//! `RUSTDOCFLAGS=-D warnings cargo doc` — CI's Documentation job — fail on a
//! non-unix target with `rustdoc::broken_intra_doc_links`. A comment-STRIPPED
//! walk is blind to that class by construction, which is why this file walks
//! twice:
//!
//! * [`code_occurrences`] over the comment-blanked view — a real reference.
//! * [`doc_link_occurrences`] over the raw view — an intra-doc LINK, in any of
//!   its spellings: the shortcut ``[`crate::x::Y`]``, the reference definition
//!   ``[`Y`]: crate::x::Y``, and the EXPLICIT DESTINATION
//!   ``[text](crate::x::Y)``. The last is the easiest to miss; see
//!   [`is_link_destination_prefix`].
//!
//! A comment that merely NAMES a unix-only module in prose is deliberately
//! fine (this module's own docs do it, and so does `lib.rs`) — only a link is
//! flagged, because only a link is resolved by rustdoc. An AUTOLINK
//! (`<crate::x>`) is likewise NOT flagged: MEASURED, rustdoc does not resolve
//! that shape as an intra-doc link.
//!
//! # Why this is not just "run the docs gate on a non-unix target"
//!
//! Because a docs run can only see the links rustdoc DOCUMENTS. Of three
//! such links this walk refuses, exactly ONE hangs off a public item
//! (`scheduler::TraceEntry::discarded`) and it alone fails the docs gate —
//! MEASURED: with `unix` configured false, restoring all three links yields
//! exactly one unresolved link. The other two hang off PRIVATE items
//! (`struct ScheduledNode`, `mod skip_cause`), so a green docs run says nothing
//! about them; they break the day somebody writes `pub`, on a change that
//! touched no link.
//!
//! This walk reads SOURCE, so visibility is irrelevant to it and all three are
//! caught, each revert failing it on its own. That is the
//! coverage a non-unix docs run cannot give even if CI had a Windows lane.
//!
//! # What is NOT claimed
//!
//! This pins the portability of REFERENCES to unix-only modules. It does not
//! claim `cerulion_core` BUILDS on Windows, and no test in this repo does —
//! the bindgen constraint above still makes a real `cargo check --target
//! x86_64-pc-windows-msvc` impossible from a mac host.
//!
//! A Windows-shape probe depends on ONE thing outside this walk:
//! configuring the crate for that shape (`unix` and every
//! `target_os` arm false, `windows` true) without `windows-sys` yields exactly 12 errors and every
//! one of them is a MISSING DEPENDENCY — 4 × `E0433 cannot find module or
//! crate windows_sys` plus the 8 type errors cascading from the two
//! `#[cfg(windows)]` arms those leave unfinished, all in `clock.rs`. ZERO
//! mention `trace_ring`.
//!
//! So `cerulion_core/Cargo.toml` carries
//! a `[target.'cfg(windows)'.dependencies]` section declaring `windows-sys`
//! with exactly the three features those arms' module paths require. Verified
//! at dependency-resolution level (the dep resolves for the Windows target and
//! is absent from the host tree) plus a feature-correctness probe crate that
//! compiles a transliteration of both arms with `--target
//! x86_64-pc-windows-msvc`; windows-sys is pure declarations, so that check
//! needs no linker. A full Windows build/CI lane is out of scope, so
//! "those 12 errors do not occur" is the level at which that is established — not a
//! Windows build.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

// ---------------------------------------------------------------------------
// source view helpers
// ---------------------------------------------------------------------------

/// Blank every comment, string literal and char literal, PRESERVING line
/// structure and byte offsets within a line (each removed byte becomes a
/// space). Line-preserving because every caller correlates the blanked view
/// with raw line numbers.
///
/// Literals are blanked as well as comments: the gate-region scan below reads
/// `{` / `}` / `;` out of this view, and `tracing` format strings in these
/// files are full of braces (`"tag `{ring_tag}`"`), which would otherwise
/// corrupt every region boundary after them.
fn blank_noncode(src: &str) -> String {
    let b: Vec<char> = src.chars().collect();
    let mut out: Vec<char> = Vec::with_capacity(b.len());
    let mut i = 0usize;
    let keep = |out: &mut Vec<char>, c: char| out.push(if c == '\n' { '\n' } else { ' ' });

    while i < b.len() {
        // line comment
        if b[i] == '/' && i + 1 < b.len() && b[i + 1] == '/' {
            while i < b.len() && b[i] != '\n' {
                keep(&mut out, b[i]);
                i += 1;
            }
            continue;
        }
        // block comment (Rust's NEST)
        if b[i] == '/' && i + 1 < b.len() && b[i + 1] == '*' {
            let mut depth = 0usize;
            while i < b.len() {
                if b[i] == '/' && i + 1 < b.len() && b[i + 1] == '*' {
                    depth += 1;
                    keep(&mut out, b[i]);
                    keep(&mut out, b[i + 1]);
                    i += 2;
                    continue;
                }
                if b[i] == '*' && i + 1 < b.len() && b[i + 1] == '/' {
                    depth -= 1;
                    keep(&mut out, b[i]);
                    keep(&mut out, b[i + 1]);
                    i += 2;
                    if depth == 0 {
                        break;
                    }
                    continue;
                }
                keep(&mut out, b[i]);
                i += 1;
            }
            continue;
        }
        // raw string: r"..." / r#"..."# / br#"..."#
        if b[i] == 'r' || (b[i] == 'b' && i + 1 < b.len() && b[i + 1] == 'r') {
            let start = i;
            let mut j = if b[i] == 'b' { i + 2 } else { i + 1 };
            let mut hashes = 0usize;
            while j < b.len() && b[j] == '#' {
                hashes += 1;
                j += 1;
            }
            if j < b.len() && b[j] == '"' {
                // consume the prefix verbatim (it is code), then blank the body
                out.extend(b[start..j].iter().copied());
                keep(&mut out, b[j]);
                i = j + 1;
                loop {
                    if i >= b.len() {
                        break;
                    }
                    if b[i] == '"' {
                        let mut h = 0usize;
                        while h < hashes && i + 1 + h < b.len() && b[i + 1 + h] == '#' {
                            h += 1;
                        }
                        if h == hashes {
                            for _ in 0..=hashes {
                                keep(&mut out, ' ');
                            }
                            i += hashes + 1;
                            break;
                        }
                    }
                    keep(&mut out, b[i]);
                    i += 1;
                }
                continue;
            }
        }
        // normal string
        if b[i] == '"' {
            keep(&mut out, b[i]);
            i += 1;
            while i < b.len() {
                if b[i] == '\\' && i + 1 < b.len() {
                    keep(&mut out, b[i]);
                    keep(&mut out, b[i + 1]);
                    i += 2;
                    continue;
                }
                if b[i] == '"' {
                    keep(&mut out, b[i]);
                    i += 1;
                    break;
                }
                keep(&mut out, b[i]);
                i += 1;
            }
            continue;
        }
        // char literal — only the short shapes, so lifetimes ('a) survive
        if b[i] == '\'' {
            let esc = i + 1 < b.len() && b[i + 1] == '\\';
            let close = if esc { None } else { i.checked_add(2) };
            if let Some(c) = close {
                if c < b.len() && b[c] == '\'' {
                    keep(&mut out, b[i]);
                    keep(&mut out, b[i + 1]);
                    keep(&mut out, b[c]);
                    i = c + 1;
                    continue;
                }
            }
            if esc {
                let mut j = i + 2;
                while j < b.len() && b[j] != '\'' && b[j] != '\n' {
                    j += 1;
                }
                if j < b.len() && b[j] == '\'' {
                    for &c in &b[i..=j] {
                        keep(&mut out, c);
                    }
                    i = j + 1;
                    continue;
                }
            }
        }
        out.push(b[i]);
        i += 1;
    }
    out.into_iter().collect()
}

fn indent_of(line: &str) -> usize {
    line.len() - line.trim_start().len()
}

fn is_attr_or_doc(line: &str) -> bool {
    let t = line.trim_start();
    t.starts_with("#[") || t.starts_with("//")
}

/// A `cfg(...)` predicate, parsed structurally.
///
/// Deliberately a tiny AST rather than a substring test. A substring test
/// is WRONG on three shapes the crate actually contains, all of
/// the same class — a disjunction one of whose branches reaches a non-unix
/// target, excused because SOME substring in it looked unix-implying:
///
/// ```text
///   any(all(unix, feature = "x"), windows)   `all(unix,` matched
///   any(target_os = "linux", test)           `target_os="linux"` matched
///   any(test, target_os = "linux")           same
/// ```
///
/// Each compiles on Windows via its other branch, so a portable reference under
/// one was silently EXCUSED — exactly the class this walk exists to catch.
#[derive(Debug, Clone, PartialEq, Eq)]
enum CfgExpr {
    /// `unix`, `windows`, `test`, `target_os = "linux"`, `feature = "x"`, …
    Atom {
        key: String,
        value: Option<String>,
    },
    Not(Box<CfgExpr>),
    All(Vec<CfgExpr>),
    Any(Vec<CfgExpr>),
}

/// Atoms that PROVE `unix`. Deliberately CONSERVATIVE: an unlisted atom fails
/// closed (the reference gets flagged), which is the safe direction — a false
/// positive is a loud test failure, a false negative is a silently broken
/// non-unix build.
///
/// `target_vendor = "apple"` is included because every Apple target is
/// Darwin-based, and the crate really does gate on it
/// (`any(target_os = "linux", target_vendor = "apple")`); omitting it would
/// turn that shape into a false positive.
const UNIX_IMPLYING_ATOMS: &[(&str, Option<&str>)] = &[
    ("unix", None),
    ("target_family", Some("unix")),
    ("target_vendor", Some("apple")),
    ("target_os", Some("linux")),
    ("target_os", Some("macos")),
    ("target_os", Some("ios")),
    ("target_os", Some("android")),
    ("target_os", Some("freebsd")),
    ("target_os", Some("openbsd")),
    ("target_os", Some("netbsd")),
    ("target_os", Some("dragonfly")),
    ("target_os", Some("solaris")),
    ("target_os", Some("illumos")),
];

/// Can we PROVE that every satisfiable assignment of this expression implies
/// `unix`? Sound but deliberately INCOMPLETE — anything unproven is `false`.
///
/// * `All` — one proving conjunct is enough (`all(unix, feature="x")` ⇒ unix).
/// * `Any` — EVERY branch must prove it, else some branch reaches a non-unix
///   target and the item is compiled there. This is the arm the substring rule
///   got wrong.
/// * `Not` — never proves it. `not(windows)` is the interesting case and it is
///   deliberately REFUSED: `wasm32-unknown-unknown` and `target_os = "none"`
///   are neither windows nor unix, so a `#[cfg(not(windows))]` item really is
///   compiled where the unix-only modules do not exist. Accepting it would be
///   unsound, so under fail-closed it does not count.
fn proves_unix(e: &CfgExpr) -> bool {
    match e {
        CfgExpr::Atom { key, value } => UNIX_IMPLYING_ATOMS
            .iter()
            .any(|(k, v)| k == key && *v == value.as_deref()),
        CfgExpr::All(items) => items.iter().any(proves_unix),
        CfgExpr::Any(items) => items.iter().all(proves_unix),
        CfgExpr::Not(_) => false,
    }
}

struct CfgParser<'a> {
    b: &'a [u8],
    i: usize,
}

impl<'a> CfgParser<'a> {
    fn new(s: &'a str) -> Self {
        Self {
            b: s.as_bytes(),
            i: 0,
        }
    }

    fn skip_ws(&mut self) {
        while self.i < self.b.len() && (self.b[self.i] as char).is_whitespace() {
            self.i += 1;
        }
    }

    fn peek(&mut self) -> Option<u8> {
        self.skip_ws();
        self.b.get(self.i).copied()
    }

    fn eat(&mut self, c: u8) -> bool {
        if self.peek() == Some(c) {
            self.i += 1;
            true
        } else {
            false
        }
    }

    fn ident(&mut self) -> Option<String> {
        self.skip_ws();
        let start = self.i;
        while self.i < self.b.len() {
            let c = self.b[self.i] as char;
            if c.is_ascii_alphanumeric() || c == '_' {
                self.i += 1;
            } else {
                break;
            }
        }
        if self.i == start {
            None
        } else {
            Some(String::from_utf8_lossy(&self.b[start..self.i]).into_owned())
        }
    }

    fn string(&mut self) -> Option<String> {
        if !self.eat(b'"') {
            return None;
        }
        let start = self.i;
        while self.i < self.b.len() && self.b[self.i] != b'"' {
            self.i += 1;
        }
        if self.i >= self.b.len() {
            return None;
        }
        let s = String::from_utf8_lossy(&self.b[start..self.i]).into_owned();
        self.i += 1;
        Some(s)
    }

    /// Anything unrecognised — an unknown combinator, a wrong arity, an empty
    /// `all()`/`any()`, an unterminated expression — yields `None`, which the
    /// caller treats as NOT unix-implying.
    fn expr(&mut self) -> Option<CfgExpr> {
        let key = self.ident()?;
        if self.peek() == Some(b'(') {
            self.i += 1;
            let mut items = Vec::new();
            loop {
                if self.peek() == Some(b')') {
                    break;
                }
                items.push(self.expr()?);
                if !self.eat(b',') {
                    break;
                }
            }
            if !self.eat(b')') {
                return None;
            }
            return match (key.as_str(), items.len()) {
                ("not", 1) => Some(CfgExpr::Not(Box::new(items.pop()?))),
                ("all", n) if n > 0 => Some(CfgExpr::All(items)),
                ("any", n) if n > 0 => Some(CfgExpr::Any(items)),
                _ => None,
            };
        }
        if self.peek() == Some(b'=') {
            self.i += 1;
            let value = self.string()?;
            return Some(CfgExpr::Atom {
                key,
                value: Some(value),
            });
        }
        Some(CfgExpr::Atom { key, value: None })
    }
}

/// Parse a single-line outer `#[cfg(...)]` attribute.
///
/// `#[cfg_attr(...)]` is REJECTED on purpose: it conditionally applies an
/// attribute and gates NOTHING, so it must never excuse a reference. It is
/// rejected structurally — `ident()` reads `cfg_attr` as one identifier — and
/// the crate has nine of them.
///
/// A cfg attribute split across lines also yields `None`. `gated_regions`
/// inspects one line at a time, so an unparseable line simply gates nothing,
/// which is the fail-closed direction.
fn parse_cfg_attr(attr: &str) -> Option<CfgExpr> {
    let inner = attr.trim_start().strip_prefix("#[")?;
    let mut p = CfgParser::new(inner);
    if p.ident()? != "cfg" {
        return None;
    }
    if !p.eat(b'(') {
        return None;
    }
    let e = p.expr()?;
    if !p.eat(b')') || !p.eat(b']') {
        return None;
    }
    Some(e)
}

/// Does this attribute line gate its item to targets where the unix-only
/// modules EXIST?
///
/// FAIL-CLOSED: true only when the cfg expression PARSES and every satisfiable
/// branch of it is provably unix. `#[cfg(unix)]` is not the only spelling that
/// qualifies — `target_os = "linux"` implies it, and `transport/shm_guard.rs`
/// really does reach `crate::state_carrier` from inside such an item — but an
/// expression that merely CONTAINS a unix-implying atom does not (see
/// `CfgExpr`).
fn is_unix_implying_gate(attr: &str) -> bool {
    parse_cfg_attr(attr)
        .map(|e| proves_unix(&e))
        .unwrap_or(false)
}

/// Inclusive `[start, end]` line ranges (0-based) governed by a `#[cfg(unix)]`
/// attribute: the whole attribute/doc block plus the item it applies to.
///
/// The item's extent is read by INDENTATION rather than by brace arithmetic:
/// the tree is `cargo fmt`-enforced, so an item opened at indent `I` closes on
/// a line whose trimmed text starts with `}` at that same indent. A
/// `;`-terminated item (`pub mod trace_ring;`) ends on its own line.
fn gated_regions(raw: &[&str], blanked: &[&str]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    for i in 0..raw.len() {
        // The PREDICATE is read from the RAW line: `blank_noncode` blanks string
        // literals, and `#[cfg(target_os = "linux")]` carries its predicate
        // INSIDE one — blanked it reads `#[cfg(target_os = "")]` and matches
        // nothing. The blanked line is still consulted, but only to establish
        // that the attribute is real code rather than a commented-out one.
        if blanked[i].trim().is_empty() || !is_unix_implying_gate(raw[i].trim()) {
            continue;
        }
        // up: the whole attribute/doc block this gate belongs to
        let mut start = i;
        while start > 0 && is_attr_or_doc(raw[start - 1]) {
            start -= 1;
        }
        // down: the item signature
        let mut s = i + 1;
        while s < raw.len() && (is_attr_or_doc(raw[s]) || raw[s].trim().is_empty()) {
            s += 1;
        }
        if s >= raw.len() {
            continue;
        }
        let indent = indent_of(raw[s]);

        // braced or `;`-terminated?
        let mut k = s;
        let mut braced = false;
        while k < raw.len() {
            let bt = blanked[k].trim_end();
            if bt.contains('{') {
                braced = true;
                break;
            }
            if bt.trim_end().ends_with(';') {
                break;
            }
            k += 1;
        }
        let end = if braced {
            let mut e = k + 1;
            let mut found = k;
            while e < raw.len() {
                if blanked[e].trim_start().starts_with('}') && indent_of(raw[e]) == indent {
                    found = e;
                    break;
                }
                e += 1;
            }
            found
        } else {
            k.min(raw.len() - 1)
        };
        out.push((start, end.max(s)));
    }
    out
}

fn in_gated_region(regions: &[(usize, usize)], line: usize) -> bool {
    regions.iter().any(|&(a, b)| line >= a && line <= b)
}

// ---------------------------------------------------------------------------
// occurrence finders
// ---------------------------------------------------------------------------

/// A real code reference to `crate::<module>` (comments blanked away).
fn code_occurrences(blanked: &[&str], module: &str) -> Vec<usize> {
    let needle = format!("crate::{module}");
    blanked
        .iter()
        .enumerate()
        .filter(|(_, l)| l.contains(&needle))
        .map(|(i, _)| i)
        .collect()
}

/// Is the text immediately BEFORE a path occurrence rustdoc link syntax?
///
/// The decoration between a link's opening syntax and its destination is
/// stripped first — backticks (``[`crate::x`]``), whitespace (`[text]( crate::x )`,
/// `[ref]: crate::x`) and a pointy-bracket destination (`[text](<crate::x>)`) —
/// then what remains must END with one of the three openers:
///
/// ```text
///   [     shortcut / bare bracket    [`crate::x::Y`]   [crate::x::Y]
///   ](    EXPLICIT DESTINATION       [text](crate::x::Y)
///   ]:    reference definition       [ref]: crate::x::Y
/// ```
///
/// `](` is the prefix easiest to miss: an explicit-destination
/// link puts `](` before the path, which matches neither of the other two
/// prefix checks, so without it a portable doc comment could link a unix-only module and
/// break non-unix rustdoc while this walk stayed green. MEASURED first-party
/// (`unix` configured false): that shape really does fail the docs gate with
/// `error: unresolved link to 'crate::trace_ring::TRACE_DISCARD_BIT'`.
///
/// A COLLAPSED reference (`[text][ref]` + `[ref]: crate::x`) needs no arm of its
/// own — measured, rustdoc reports it against the reference DEFINITION line,
/// which `]:` already covers.
fn is_link_destination_prefix(before: &str) -> bool {
    let b = before.trim_end_matches(|c: char| c == '`' || c == '<' || c.is_whitespace());
    b.ends_with('[') || b.ends_with("](") || b.ends_with("]:")
}

/// An intra-doc LINK to `crate::<module>` — the class `cargo check` cannot see
/// and a comment-stripped walk discards. See [`is_link_destination_prefix`] for
/// the shapes.
///
/// A bare prose mention (``  `crate::x`  `` with no brackets) is NOT a link and
/// is deliberately allowed. So is an AUTOLINK (`<crate::x>`): MEASURED — with
/// `unix` configured false, `<crate::trace_ring::TRACE_DISCARD_BIT>` on a `pub`
/// item produced NO error, and the CONTROL (two explicit-destination links on
/// the same item, reported as two errors) proves rustdoc does not stop at the
/// first, so that silence is a real non-resolution rather than a masked one.
///
/// EVERY occurrence on the line is examined, not just the first: a line can
/// mention the path in prose and then link it.
fn doc_link_occurrences(raw: &[&str], module: &str) -> Vec<usize> {
    let path = format!("crate::{module}");
    let mut out = Vec::new();
    for (i, line) in raw.iter().enumerate() {
        let t = line.trim_start();
        if !t.starts_with("///") && !t.starts_with("//!") {
            continue;
        }
        let mut from = 0usize;
        while let Some(rel) = line[from..].find(&path) {
            let p = from + rel;
            if is_link_destination_prefix(&line[..p]) {
                out.push(i);
                break;
            }
            from = p + path.len();
        }
    }
    out
}

// ---------------------------------------------------------------------------
// crate layout
// ---------------------------------------------------------------------------

fn src_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("src")
}

/// DERIVE the unix-only module set from `lib.rs` rather than hand-listing it —
/// a hand list reproduces the very failure mode this file exists for (a new
/// unix-only module joins the audit by construction).
fn unix_only_modules(lib_rs: &str) -> BTreeSet<String> {
    let lines: Vec<&str> = lib_rs.lines().collect();
    let mut out = BTreeSet::new();
    for (i, l) in lines.iter().enumerate() {
        if l.trim() != "#[cfg(unix)]" {
            continue;
        }
        for cand in lines.iter().skip(i + 1).take(4) {
            let t = cand.trim();
            if t.starts_with("#[") {
                continue;
            }
            if let Some(rest) = t
                .strip_prefix("pub mod ")
                .or_else(|| t.strip_prefix("mod "))
            {
                if let Some(name) = rest.strip_suffix(';') {
                    out.insert(name.trim().to_string());
                }
            }
            break;
        }
    }
    out
}

fn rust_files(dir: &Path, out: &mut Vec<PathBuf>) {
    let mut entries: Vec<_> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display()))
        .filter_map(Result::ok)
        .map(|e| e.path())
        .collect();
    entries.sort();
    for p in entries {
        if p.is_dir() {
            rust_files(&p, out);
        } else if p.extension().and_then(|e| e.to_str()) == Some("rs") {
            out.push(p);
        }
    }
}

/// The files that ARE the unix-only modules — a reference from inside one is
/// fine, because the file itself is not compiled on a non-unix target.
fn is_unix_only_file(rel: &Path, modules: &BTreeSet<String>) -> bool {
    let s = rel.to_string_lossy().replace('\\', "/");
    modules
        .iter()
        .any(|m| s == format!("{m}.rs") || s.starts_with(&format!("{m}/")))
}

struct Finding {
    file: String,
    line: usize,
    module: String,
    kind: &'static str,
    text: String,
}

fn audit() -> (Vec<Finding>, BTreeSet<String>, usize) {
    let src = src_dir();
    let lib = std::fs::read_to_string(src.join("lib.rs")).expect("read lib.rs");
    let modules = unix_only_modules(&lib);
    assert!(
        !modules.is_empty(),
        "ANTI-TAUTOLOGY: derived NO unix-only modules from lib.rs — the walk would \
         then vacuously pass. Has the `#[cfg(unix)] pub mod X;` shape changed?"
    );

    let mut files = Vec::new();
    rust_files(&src, &mut files);

    let mut findings = Vec::new();
    let mut gated_seen = 0usize;
    for f in &files {
        let rel = f.strip_prefix(&src).expect("under src").to_path_buf();
        if is_unix_only_file(&rel, &modules) {
            continue;
        }
        let text = std::fs::read_to_string(f).unwrap_or_else(|e| panic!("read {f:?}: {e}"));
        let blanked_owned = blank_noncode(&text);
        let raw: Vec<&str> = text.lines().collect();
        let blanked: Vec<&str> = blanked_owned.lines().collect();
        assert_eq!(
            raw.len(),
            blanked.len(),
            "blank_noncode must preserve line structure ({})",
            rel.display()
        );
        let regions = gated_regions(&raw, &blanked);

        for m in &modules {
            for (kind, lines) in [
                ("code", code_occurrences(&blanked, m)),
                ("doc-link", doc_link_occurrences(&raw, m)),
            ] {
                for line in lines {
                    if in_gated_region(&regions, line) {
                        gated_seen += 1;
                    } else {
                        findings.push(Finding {
                            file: rel.display().to_string(),
                            line: line + 1,
                            module: m.clone(),
                            kind,
                            text: raw[line].trim().to_string(),
                        });
                    }
                }
            }
        }
    }
    (findings, modules, gated_seen)
}

// ---------------------------------------------------------------------------
// the gate
// ---------------------------------------------------------------------------

#[test]
fn no_portable_item_references_a_unix_only_module() {
    let (findings, modules, _) = audit();
    assert!(
        findings.is_empty(),
        "cfg audit: {} PORTABLE reference(s) to a `#[cfg(unix)]`-only module \
         ({:?}).\n\nEach breaks a non-unix build: a `code` finding fails \
         `cargo check`, a `doc-link` finding fails CI's Documentation job \
         (`RUSTDOCFLAGS=-D warnings cargo doc`) with \
         `rustdoc::broken_intra_doc_links`.\n\nFix: gate the enclosing item \
         with `#[cfg(unix)]`, or — for a doc link — drop the link and name the \
         item in plain backticks (a prose mention is not resolved by rustdoc \
         and is deliberately allowed).\n\n{}",
        findings.len(),
        modules,
        findings
            .iter()
            .map(|f| format!(
                "  {}:{} [{}] {} -> {}",
                f.file, f.line, f.kind, f.module, f.text
            ))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

#[test]
fn the_walk_reaches_the_references_it_is_supposed_to_be_judging() {
    // ANTI-TAUTOLOGY. Every assertion above is negative, so a walk that read
    // no files, derived no modules, or classified every line as ungoverned
    // would pass it. This pins that the walk really does find — and really
    // does CLEAR — the gated production references.
    let (_, modules, gated_seen) = audit();
    assert!(
        modules.contains("trace_ring"),
        "expected `trace_ring` among the derived unix-only modules, got {modules:?}"
    );
    assert!(
        gated_seen >= 10,
        "expected the walk to clear the gated trace-ring references \
         (~12 in scheduler/mod.rs + graph/runtime.rs), cleared {gated_seen}"
    );
}

// ---------------------------------------------------------------------------
// oracles for the machinery itself
// ---------------------------------------------------------------------------

#[test]
fn blank_noncode_removes_only_noncode_and_keeps_lines() {
    let src = "let a = 1; // crate::trace_ring\n\
               /* crate::trace_ring /* nested */ still */ let b = 2;\n\
               let s = \"crate::trace_ring {\";\n\
               let r = r#\"crate::trace_ring }\"#;\n\
               let c = '{';\n\
               fn f<'a>(x: &'a str) {}\n";
    let out = blank_noncode(src);
    assert_eq!(
        src.lines().count(),
        out.lines().count(),
        "line structure must survive"
    );
    assert!(
        !out.contains("crate::trace_ring"),
        "every non-code occurrence must be blanked, got:\n{out}"
    );
    for keep in ["let a = 1;", "let b = 2;", "let s =", "let r =", "fn f<'a>"] {
        assert!(out.contains(keep), "code `{keep}` must survive:\n{out}");
    }
    // the brace-bearing literals are what would corrupt region ends
    let braces = out.chars().filter(|&c| c == '{' || c == '}').count();
    assert_eq!(
        braces, 2,
        "only the real fn-body braces may survive, got {braces}:\n{out}"
    );
}

#[test]
fn a_gate_whose_predicate_lives_in_a_string_literal_is_still_recognised() {
    // REGRESSION. `blank_noncode` blanks string literals (it must — `tracing`
    // format strings are full of braces and the region scan reads braces), so
    // reading the gate predicate off the BLANKED line sees
    // `#[cfg(target_os = "")]` and recognises nothing. That is not academic:
    // `transport/shm_guard.rs` reaches `crate::state_carrier` from inside a
    // `#[cfg(target_os = "linux")]` fn, and this walk called it a defect until
    // the predicate was read from the raw line.
    for gate in [
        "#[cfg(unix)]",
        "#[cfg(target_os = \"linux\")]",
        "#[cfg(target_os = \"macos\")]",
        "#[cfg(all(unix, not(target_os = \"macos\")))]",
    ] {
        assert!(is_unix_implying_gate(gate), "{gate} must be unix-implying");
    }
    for open in [
        "#[cfg(not(unix))]",
        "#[cfg(not(target_os = \"linux\"))]",
        "#[cfg(not(any(target_os = \"linux\", target_os = \"macos\")))]",
        "#[cfg(windows)]",
        "#[cfg(feature = \"test-helpers\")]",
        "#[derive(Debug)]",
    ] {
        assert!(
            !is_unix_implying_gate(open),
            "{open} is reachable off unix and must NOT excuse a reference"
        );
    }

    // …and end to end through `gated_regions`, over a real string-bearing gate.
    let src = "\
#[cfg(target_os = \"linux\")]
fn linux_only() {
    let p = crate::state_carrier::thing();
}
";
    let raw: Vec<&str> = src.lines().collect();
    let blanked_owned = blank_noncode(src);
    let blanked: Vec<&str> = blanked_owned.lines().collect();
    let regions = gated_regions(&raw, &blanked);
    assert!(
        in_gated_region(&regions, 2),
        "the body of a `target_os = \"linux\"` fn must be gated, regions={regions:?}"
    );
}

#[test]
fn a_nested_any_gate_whose_other_branch_reaches_windows_does_not_excuse_a_reference() {
    // THE regression, in its own test so a revert is attributable to it rather
    // than to whichever table row happens to be checked first.
    //
    // `all(unix, ...)` appears INSIDE this expression, which is what a
    // substring rule matches on — but the `windows` branch compiles the item on
    // Windows, where the unix-only modules do not exist. So under a substring
    // rule a portable reference under this gate is silently EXCUSED: exactly the
    // class the walk exists to catch, walking straight through it.
    const NESTED_ANY: &str = "#[cfg(any(all(unix, feature = \"x\"), windows))]";

    assert!(
        !is_unix_implying_gate(NESTED_ANY),
        "{NESTED_ANY} compiles on Windows via its second branch, so it must NOT \
         excuse a reference to a unix-only module"
    );

    // ANTI-TAUTOLOGY: the same expression with its non-unix branch replaced by
    // a unix-implying one MUST still be excused, so the assertion above cannot
    // be satisfied by a classifier that simply refuses every `any(...)`.
    const ALL_BRANCHES_UNIX: &str =
        "#[cfg(any(all(unix, feature = \"x\"), target_os = \"linux\"))]";
    assert!(
        is_unix_implying_gate(ALL_BRANCHES_UNIX),
        "{ALL_BRANCHES_UNIX} has no branch that reaches a non-unix target and \
         must still excuse a reference"
    );

    // …and END TO END through `gated_regions`, which is what the walk consults:
    // a portable reference under the nested-any gate must be FLAGGED, while the
    // all-unix twin beside it stays excused.
    let src = "\
#[cfg(any(all(unix, feature = \"x\"), windows))]
fn compiled_on_windows_too() {
    let p = crate::trace_ring::TRACE_DISCARD_BIT;
}

#[cfg(any(all(unix, feature = \"x\"), target_os = \"linux\"))]
fn unix_only_either_way() {
    let q = crate::trace_ring::TRACE_DISCARD_BIT;
}
";
    let raw: Vec<&str> = src.lines().collect();
    let blanked_owned = blank_noncode(src);
    let blanked: Vec<&str> = blanked_owned.lines().collect();
    let regions = gated_regions(&raw, &blanked);
    let at = |needle: &str| -> usize {
        raw.iter()
            .position(|l| l.contains(needle))
            .unwrap_or_else(|| panic!("fixture line {needle:?} not found"))
    };
    assert!(
        !in_gated_region(&regions, at("let p =")),
        "a reference under `any(all(unix, ..), windows)` reaches Windows and \
         must NOT be excused, regions={regions:?}"
    );
    assert!(
        in_gated_region(&regions, at("let q =")),
        "a reference under `any(all(unix, ..), target_os = \"linux\")` is \
         unix-only on every branch and must stay excused, regions={regions:?}"
    );
}

#[test]
fn a_gate_excuses_a_reference_only_when_every_branch_provably_implies_unix() {
    // The classification is FAIL-CLOSED: a cfg expression excuses a reference
    // only when we can PROVE every satisfiable branch implies `unix`.
    //
    // The rule this replaces was a substring test, and it was wrong on the
    // whole disjunction class — an `any(...)` one of whose branches reaches a
    // non-unix target, excused because some substring in it looked
    // unix-implying. All three shapes below occur in `cerulion_core/src`
    // TODAY, so this is not a hypothetical hardening.
    let proves = [
        "#[cfg(unix)]",
        "#[cfg(target_os = \"linux\")]",
        "#[cfg(target_os = \"macos\")]",
        "#[cfg(target_vendor = \"apple\")]",
        "#[cfg(target_family = \"unix\")]",
        // a conjunction needs only ONE proving conjunct
        "#[cfg(all(unix, feature = \"x\"))]",
        "#[cfg(all(unix, not(target_os = \"macos\")))]",
        "#[cfg(all(target_os = \"linux\", target_arch = \"aarch64\"))]",
        // a disjunction is fine when EVERY branch proves it
        "#[cfg(any(target_os = \"linux\", target_vendor = \"apple\"))]",
        "#[cfg(any(unix, all(target_os = \"macos\", test)))]",
        // whitespace and a trailing comment are not part of the predicate
        "  #[cfg( all ( unix , feature = \"x\" ) )]  ",
        "#[cfg(unix)] // a trailing comment",
    ];
    for gate in proves {
        assert!(
            is_unix_implying_gate(gate),
            "{gate} provably implies unix and must excuse a reference"
        );
    }

    let does_not_prove = [
        // the disjunction class is pinned on its OWN, above — these are its
        // two siblings that live in `cerulion_core/src` today (`test` is
        // satisfiable on Windows)
        "#[cfg(any(target_os = \"linux\", test))]",
        "#[cfg(any(test, target_os = \"linux\"))]",
        // NEGATION never proves unix — `not(windows)` is the interesting one:
        // wasm32-unknown-unknown and `target_os = "none"` are neither windows
        // nor unix, so such an item really is compiled where the modules are
        // absent. Refused deliberately rather than accepted as "probably unix".
        "#[cfg(not(windows))]",
        "#[cfg(not(unix))]",
        "#[cfg(not(target_os = \"linux\"))]",
        "#[cfg(not(any(target_os = \"linux\", target_os = \"macos\")))]",
        // plainly non-unix / target-agnostic
        "#[cfg(windows)]",
        "#[cfg(test)]",
        "#[cfg(feature = \"test-helpers\")]",
        "#[cfg(any(unix, windows))]",
        // `cfg_attr` gates NOTHING — it conditionally applies an ATTRIBUTE, so
        // it must never excuse a reference. The crate has nine.
        "#[cfg_attr(unix, allow(dead_code))]",
        "#[cfg_attr(all(target_os = \"linux\", target_arch = \"aarch64\"), allow(dead_code))]",
        // …and the ARITY-FREE form, which is what makes the NAME rule
        // checkable on its own, apart from arity. MEASURED: with the name check
        // widened to accept `cfg_attr`, both real spellings above are STILL
        // refused — a real `cfg_attr` carries a second argument, so the parse
        // hits `,` where it needs `)` and fails closed on arity, not on the
        // name. That makes the name check output-equivalent on every shape the
        // crate contains, so without this row it could be deleted with the
        // suite green. Not valid Rust, deliberately: the point is to isolate
        // "only `cfg` gates an item" from the trailing-syntax check that
        // happens to imply it today.
        "#[cfg_attr(unix)]",
        // FAIL-CLOSED on anything unparseable: an unknown combinator, a wrong
        // arity, an empty list, a truncated multi-line attribute, garbage.
        "#[cfg(some_unknown_combinator(unix))]",
        "#[cfg(not(unix, windows))]",
        "#[cfg(all())]",
        "#[cfg(any())]",
        "#[cfg(not(any(",
        "#[cfg(unix",
        "#[cfg(unix))]]extra",
        "#[derive(Debug)]",
        "not an attribute at all",
        "",
    ];
    for gate in does_not_prove {
        assert!(
            !is_unix_implying_gate(gate),
            "{gate:?} is NOT provably unix-only and must NOT excuse a reference"
        );
    }

    // `all(windows, unix)` is unsatisfiable, so "every satisfiable branch
    // implies unix" is vacuously true and the prover says so. Asserted
    // explicitly rather than left ambiguous — it is excused above, and that is
    // sound (an item that is compiled nowhere can reference anything).
    assert!(is_unix_implying_gate("#[cfg(all(windows, unix))]"));
}

#[test]
fn an_explicit_destination_link_is_detected_like_any_other_link() {
    // THE regression, in its own test so a revert is attributable to it.
    //
    // A markdown link may carry its destination EXPLICITLY — `[text](dest)` —
    // which puts `](` before the path rather than `[` or `]:`. Both original
    // prefix checks missed it, so a portable doc comment could link a unix-only
    // module and break non-unix rustdoc while this walk stayed green.
    //
    // MEASURED first-party rather than assumed (with the `unix` predicate
    // configured false, on `TraceEntry::discarded`, the one carrier rustdoc
    // reaches): this shape fails the docs gate with
    //   error: unresolved link to `crate::trace_ring::TRACE_DISCARD_BIT`
    const EXPLICIT: &str = "/// see [the trace ring](crate::trace_ring::TraceRingProducer) here";
    assert_eq!(
        doc_link_occurrences(&[EXPLICIT], "trace_ring"),
        vec![0],
        "an explicit-destination link is a rustdoc link and must be flagged"
    );

    // ANTI-TAUTOLOGY: the same line with the path in PROSE instead of as a
    // destination must NOT be flagged, so the assertion above cannot be
    // satisfied by a detector that simply flags every doc line naming the path.
    const PROSE: &str = "/// see [the trace ring](https://example.invalid) — `crate::trace_ring`";
    assert_eq!(
        doc_link_occurrences(&[PROSE], "trace_ring"),
        Vec::<usize>::new(),
        "a prose mention beside an unrelated link must NOT be flagged"
    );

    // …and END TO END through the walk's own region logic: a PORTABLE item
    // carrying the link is a finding, while the same link under a `#[cfg(unix)]`
    // gate is not.
    let src = "\
/// links [the ring](crate::trace_ring::TRACE_DISCARD_BIT)
pub struct Portable {
    f: u32,
}

/// links [the ring](crate::trace_ring::TRACE_DISCARD_BIT)
#[cfg(unix)]
pub struct Gated {
    f: u32,
}
";
    let raw: Vec<&str> = src.lines().collect();
    let blanked_owned = blank_noncode(src);
    let blanked: Vec<&str> = blanked_owned.lines().collect();
    let regions = gated_regions(&raw, &blanked);
    let links = doc_link_occurrences(&raw, "trace_ring");
    assert_eq!(
        links.len(),
        2,
        "both links must be seen, regions={regions:?}"
    );
    assert!(
        !in_gated_region(&regions, links[0]),
        "the PORTABLE item's explicit-destination link must be flagged, regions={regions:?}"
    );
    assert!(
        in_gated_region(&regions, links[1]),
        "the GATED item's identical link must be excused, regions={regions:?}"
    );
}

#[test]
fn doc_link_detection_separates_a_link_from_a_prose_mention() {
    let raw = vec![
        "/// see [`crate::trace_ring::X`] for detail", // 0 link (shortcut)
        "/// see `crate::trace_ring` in prose",        // 1 NOT a link
        "//! [`Y`]: crate::trace_ring::Y",             // 2 link (ref def)
        "// crate::trace_ring in a plain comment",     // 3 not a doc line
        "    let p: crate::trace_ring::P = q;",        // 4 code, not doc
        "/// and [crate::trace_ring::Z] bare-bracket", // 5 link (bare bracket)
        "/// and [text](crate::trace_ring::W) too",    // 6 link (explicit dest)
        "/// and [t](`crate::trace_ring::V`) too",     // 7 link (backticked dest)
        "/// and [t](<crate::trace_ring::U>) too",     // 8 link (pointy dest)
        "/// and [t]( crate::trace_ring::T ) too",     // 9 link (spaced dest)
        // an AUTOLINK is NOT a rustdoc intra-doc link — MEASURED (see
        // `doc_link_occurrences`): rustdoc reports nothing for this shape, and
        // the control proves it reports EVERY link it does resolve.
        "/// bare autolink <crate::trace_ring::S>", // 10 NOT a link
        // prose FIRST, link SECOND on one line — the whole line must be
        // scanned, not just the first occurrence
        "/// `crate::trace_ring` then [x](crate::trace_ring::R)", // 11 link
    ];
    assert_eq!(
        doc_link_occurrences(&raw, "trace_ring"),
        vec![0, 2, 5, 6, 7, 8, 9, 11],
        "every LINK spelling must be flagged and nothing else"
    );
    let blanked_owned = blank_noncode(&raw.join("\n"));
    let blanked: Vec<&str> = blanked_owned.lines().collect();
    assert_eq!(
        code_occurrences(&blanked, "trace_ring"),
        vec![4],
        "only the real code line may be flagged as a code reference"
    );
}

#[test]
fn gated_regions_cover_the_block_the_item_and_nothing_after_it() {
    let raw_s = "\
struct Portable {
    /// links [`crate::trace_ring::BIT`]
    f: u32,
}

/// doc above the gate
#[cfg(unix)]
struct Gated {
    p: crate::trace_ring::P,
}

#[cfg(unix)]
pub mod thing;

impl Portable {
    /// doc
    #[cfg(unix)]
    pub fn m(&self) -> crate::trace_ring::P {
        todo!()
    }

    pub fn portable(&self) -> u32 {
        0
    }
}
";
    let raw: Vec<&str> = raw_s.lines().collect();
    let blanked_owned = blank_noncode(raw_s);
    let blanked: Vec<&str> = blanked_owned.lines().collect();
    let regions = gated_regions(&raw, &blanked);

    // Look lines up BY CONTENT — a hardcoded index table is its own bug source
    // (an off-by-one table reports the
    // implementation as broken when the indices are).
    let at = |needle: &str| -> usize {
        raw.iter()
            .position(|l| l.contains(needle))
            .unwrap_or_else(|| panic!("fixture line {needle:?} not found"))
    };
    let gated = |needle: &str| in_gated_region(&regions, at(needle));

    assert!(
        !gated("links [`crate::trace_ring::BIT`]"),
        "a portable item's doc link must not be excused, regions={regions:?}"
    );
    // the gate's own doc block, the gate, the signature, the body, the close
    for needle in [
        "/// doc above the gate",
        "#[cfg(unix)]",
        "struct Gated {",
        "p: crate::trace_ring::P,",
    ] {
        assert!(
            gated(needle),
            "{needle:?} must be gated, regions={regions:?}"
        );
    }
    // the `;`-terminated item covers itself and does NOT swallow what follows
    assert!(gated("pub mod thing;"), "regions={regions:?}");
    assert!(
        !gated("impl Portable {"),
        "a `;` item must not swallow the following impl, regions={regions:?}"
    );
    // a gated METHOD inside an UNGATED impl gates itself, not its sibling
    for needle in ["pub fn m(&self)", "todo!()"] {
        assert!(
            gated(needle),
            "{needle:?} must be gated, regions={regions:?}"
        );
    }
    assert!(
        !gated("pub fn portable(&self)"),
        "the portable sibling method must NOT inherit the gate, regions={regions:?}"
    );
    assert!(
        !gated("struct Portable {"),
        "the portable struct must not be gated, regions={regions:?}"
    );
}
