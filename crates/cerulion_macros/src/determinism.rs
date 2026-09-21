// SPDX-License-Identifier: AGPL-3.0-only
//! The canonical banned-non-deterministic-symbol table.
//!
//! This is the **single source of truth** for the node-level determinism
//! lints. The two halves of the determinism lint consume it:
//!
//! - the **macro half** (`#[cerulion_node_impl]`, this crate) walks every
//!   user method body and emits a `compile_error!` for each `Deny` symbol it
//!   finds (see `impl_macro.rs`). **The macro half emits DENY errors only**
//!   — `Warn` rows are not surfaced by the macro (stable Rust gives
//!   proc-macros no warn-level diagnostic API; see "Why the macro DENIES but
//!   cannot WARN" below).
//! - the **core half** (deferred — collides with the `NodeContext`
//!   rewrite) will read the `Warn` rows to print a CLI warning at graph load.
//!   That half is responsible for BOTH the warn detection (re-walking node
//!   bodies at the CLI, which is where the node source is actually available)
//!   AND its own warn-surfacing metadata mechanism + the `--strict-determinism`
//!   gate. The macro half does NOT record warn keys for it — there is no
//!   cross-process channel from the proc-macro to the separate graph-load CLI
//!   process (the proc-macro registry is process-local to the proc-macro DLL,
//!   so anything written there is invisible to graph load). See the
//!   "Warn-surfacing is deferred" note below.
//!
//! The list is the canonical determinism POLICY (which symbols are banned) —
//! its items are `pub` (crate-visible) so the macro half can match against them.
//! It is NOT cross-crate-importable, and CANNOT be made so: a `proc-macro = true`
//! crate may export only its macros, so this module stays private (`mod`, not
//! `pub mod` — the latter is a hard compile error) and the deferred core half
//! CANNOT `use cerulion_macros::determinism::BANNED`. Part 2 relocates this
//! table to a shared non-proc-macro crate (consumable by both this macro and
//! `cerulion_cli_engine`) so the CLI half matches its re-walk of node source
//! against the SAME list rather than re-deriving it.
//!
//! # Warn-surfacing is deferred (no process-local warn recording)
//!
//! Writing surviving warn keys into the proc-macro registry
//! (`record_determinism_warns`) so the deferred CLI half could surface them
//! would be dead code: the registry lives in the proc-macro process, which
//! exits when `cargo build` finishes; the separate
//! `cerulion graph run` / `cerulion node info` process can never read it. So
//! the macro half now **emits deny errors only** and does NOT detect or record
//! warn-class symbols. The deferred core half owns warn detection (it re-walks
//! node source via `cerulion_cli_engine::node_metadata::parse_node_metadata`,
//! which already reads `nodes/<type>/src/lib.rs`) and warn-surfacing.
//!
//! # The DENY set is the unambiguous killers; live-IO is NOT a fs blocklist
//!
//! The DENY set is intentionally **exactly the four symbols that have no
//! legitimate in-tick use** and unambiguously void bit-for-bit replay:
//! `Instant::now`, `SystemTime::now`, `tokio::time::Instant::now` (caught via
//! the shared `["Instant", "now"]` tail), and `thread::spawn`. Reading the
//! live clock or spawning an unmanaged thread is never the right thing inside
//! a deterministic tick — there is a framework alternative for each
//! (`ctx.clock()`, the scheduler) — so denying them is unambiguous.
//!
//! **`fs::read_dir` is a WARN (IO-class), NOT a deny.** Live-filesystem IO
//! determinism is owned by the `#[cerulion_node(uses_live_io)]` declaration
//! plus replay verification — NOT by a syntactic fs blocklist. A partial
//! blocklist (deny `read_dir` but ignore `File::open`, `TcpStream::connect`,
//! `recv`, mmap, …) would overstate what the lint actually catches: it would
//! imply "the macro guarantees no live IO" when it can only see a handful of
//! syntactic shapes. So `read_dir` is surfaced as a warn (a nudge toward
//! declaring `uses_live_io`), and the declaration + replay own the real
//! completeness guarantee.
//!
//! # Why the macro DENIES but cannot WARN
//!
//! Stable Rust gives proc-macros no warn-level diagnostic API — the only
//! compile-time signal a proc-macro can raise is a hard `compile_error!`.
//! So the macro half:
//!
//! - **Deny** → emit `compile_error!` (a real compile failure).
//! - **Warn** → NOT surfaced by the macro at all. The macro cannot emit a
//!   warn diagnostic on stable Rust, and it has no usable channel to hand
//!   warn keys to the separate graph-load process (the proc-macro registry is
//!   process-local — see "Warn-surfacing is deferred" above). So warn
//!   detection + surfacing is entirely the deferred core half's job; the macro
//!   half stays silent for warn-class symbols. The `Warn` rows below exist as
//!   the canonical table the core half will consume, not as anything the macro
//!   acts on today.
//!
//! # IO-class symbols and `uses_live_io`
//!
//! Orthogonal to severity, a row may be **IO-class** ([`BannedSymbol::io`]).
//! Today the only IO-class row is the `fs::read_dir` warn. The
//! `#[cerulion_node(uses_live_io)]` declaration is the live-IO marker: it both
//! suppresses IO-class rows AND declares (in the node SOURCE, where the
//! deferred CLI half reads it via `parse_node_metadata`) that the node performs
//! live IO, which the CLI pairs with replay verification. Since the macro half
//! is deny-only and today's only IO-class row is a warn, `uses_live_io`
//! currently affects only the deferred core half's warn path — but the gate is
//! `io`, so it would suppress an IO-class deny too if one were ever added
//! (which the const contract below forbids). Non-IO rows (whether deny or
//! warn) are suppressed only by the blanket
//! `#[cerulion_node(allow_non_deterministic)]`. (`allow_non_deterministic`
//! suppresses everything.)
//!
//! # Matching contract — strict last-two-segment
//!
//! A call matches a row when the **last two path segments** of the callee
//! equal the row's [`BannedSymbol::segments`]. So both the fully-qualified
//! `std::time::Instant::now()` and the unqualified `Instant::now()` (under
//! a `use std::time::Instant;`) match the row `["Instant", "now"]`. This
//! deliberately accepts a false-positive risk: a user type literally named
//! `Instant` with a `now()` method would also match. The suppression attrs
//! (`#[cerulion_node(allow_non_deterministic)]` /
//! `#[cerulion_node(uses_live_io)]`) are the escape hatch for that case.
//!
//! Receiver method calls are NOT matched — `self.timer.now()` is a method
//! call on a *value*, not a free/associated path call. The macro half's
//! `DeterminismLintVisitor` (`impl_macro.rs`) only consults this table for
//! `Expr::Call` whose callee is a path (e.g. `Instant::now()`,
//! `thread::spawn(...)`) — including such calls hidden inside macro arguments,
//! which `visit_macro` re-parses — never for `Expr::MethodCall` on a value
//! receiver. (And the macro half consults DENY rows only; see "Why the macro
//! DENIES but cannot WARN" above.)

/// Lint **severity** for a banned symbol. Orthogonal to IO-class
/// (see [`BannedSymbol::io`]): severity decides whether the macro emits a
/// `compile_error!`; IO-class decides which opt-out suppresses the row.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LintClass {
    /// Hard deny. The macro emits `compile_error!`. Suppressed only by the
    /// blanket `#[cerulion_node(allow_non_deterministic)]` — unless the row
    /// is also IO-class, in which case `#[cerulion_node(uses_live_io)]`
    /// suppresses it too. (No IO-class deny rows exist today; the deny set is
    /// the four time/thread killers.)
    Deny,
    /// Warn-class. NOT surfaced by the macro half at all (stable Rust has no
    /// proc-macro warn API, and there is no cross-process channel to the
    /// graph-load CLI — see the module's "Warn-surfacing is deferred" note).
    /// Warn detection + surfacing is owned by the deferred core half, which
    /// re-walks node source at the CLI; suppression there is by
    /// `#[cerulion_node(allow_non_deterministic)]`, or (if also IO-class,
    /// [`BannedSymbol::io`]) `#[cerulion_node(uses_live_io)]`. These rows exist
    /// as the canonical table that half will consume.
    Warn,
}

impl LintClass {
    /// `true` if the symbol produces a hard compile error in the macro half
    /// (i.e. `Deny`). `Warn` returns `false` — it is not surfaced by the macro
    /// (warn-surfacing is the deferred CLI/core half; see module docs).
    ///
    /// `const` so the compile-time `BANNED`-table contract check (the
    /// `const _: ()` block below) can call it in const context.
    pub const fn is_deny(self) -> bool {
        matches!(self, LintClass::Deny)
    }
}

/// One banned non-deterministic symbol: how to match it, how severe it is,
/// and the user-facing remediation message.
#[derive(Debug, Clone, Copy)]
pub struct BannedSymbol {
    /// The path segments matched **strict last-two** against a callee path
    /// (e.g. `["Instant", "now"]`). For free functions matched as a
    /// two-segment tail (`thread::spawn`), this is `["thread", "spawn"]`.
    pub segments: &'static [&'static str],
    /// Severity (deny vs warn). Orthogonal to [`Self::io`].
    pub class: LintClass,
    /// `true` if this symbol performs **live IO** whose determinism is owned
    /// by the `#[cerulion_node(uses_live_io)]` declaration + replay, not by a
    /// syntactic blocklist. IO-class rows are suppressed by `uses_live_io`
    /// (in addition to the blanket `allow_non_deterministic`). The only
    /// IO-class row today is the `fs::read_dir` warn.
    pub io: bool,
    /// Stable identifier for the symbol, used as the key the deferred core
    /// half will surface for warn-class rows (and asserted by this module's
    /// unit tests, which pin the exact table contents). Distinct from
    /// `segments` so a future symbol matched by multiple shapes still has one
    /// key.
    ///
    /// `#[allow(dead_code)]`: read by the table-contract unit tests and the
    /// deferred core half, but not by the macro half's deny path
    /// (which uses `message`, not `key`). Kept because `key` is part of the
    /// canonical table this module exports as the single source of truth for
    /// both halves — see the module docs.
    #[allow(dead_code)]
    pub key: &'static str,
    /// The user-facing remediation message embedded in the `compile_error!`
    /// (deny rows) or surfaced by the CLI (warn rows). **User-facing
    /// contract — keep actionable.**
    pub message: &'static str,
}

/// The canonical banned-symbol table.
///
/// The DENY set is exactly the four unambiguous killers (no legit in-tick
/// use): `Instant::now`, `SystemTime::now`, `tokio::time::Instant::now` (via
/// the shared `["Instant", "now"]` tail), and `thread::spawn`. Everything
/// else is a WARN. `fs::read_dir` is the sole IO-class WARN (`io: true`) —
/// live-IO completeness is owned by `uses_live_io` + replay, NOT by this
/// table (see module docs).
///
/// Ordering is deny-first, then warn — purely for readability; the matcher
/// scans the whole slice and the first match wins, so no two DENY rows share
/// a `segments` tail (the duplicated tokio `Instant::now` tail is covered by
/// a single match — see that row's note).
pub const BANNED: &[BannedSymbol] = &[
    // ----- Deny: the four unambiguous killers (no IO-class deny rows) -----
    BannedSymbol {
        segments: &["Instant", "now"],
        class: LintClass::Deny,
        io: false,
        key: "std::time::Instant::now",
        message: "`Instant::now()` reads the live monotonic clock and voids bit-for-bit \
                  replay determinism. Read time through the node's own clock accessor \
                  instead: `self.now_ns()` (nanoseconds from the clock the scheduler is \
                  running on, so it replays identically). If you need real wall time \
                  regardless of replay, `self.real_ns()` says so explicitly. If this node \
                  is genuinely allowed to be non-deterministic, opt out with \
                  `#[cerulion_node(allow_non_deterministic)]`.",
    },
    BannedSymbol {
        segments: &["SystemTime", "now"],
        class: LintClass::Deny,
        io: false,
        key: "std::time::SystemTime::now",
        message: "`SystemTime::now()` reads the live system clock and voids bit-for-bit \
                  replay determinism. If you are measuring elapsed time, read it through \
                  the node's own clock accessor instead: `self.now_ns()` (nanoseconds from \
                  the clock the scheduler is running on, so it replays identically), or \
                  `self.real_ns()` for a duration you need measured against the hardware \
                  clock on this machine. Note that neither is a calendar clock — both count \
                  from an arbitrary origin, so neither can be converted to a date. If you \
                  genuinely need wall-clock date-and-time, that is inherently \
                  non-deterministic and there is no replay-safe way to read it: keep \
                  `SystemTime::now()` and declare the node with \
                  `#[cerulion_node(allow_non_deterministic)]`.",
    },
    // NOTE: `tokio::time::Instant::now` has the SAME last-two tail
    // `["Instant", "now"]` as the std row above, so the matcher already
    // catches it — we do NOT add a duplicate-tail row (the matcher's
    // first-match-wins would make it dead). The std row's message names the
    // std path; the tokio variant produces an identical compile error, which
    // is acceptable (both remediate to `self.now_ns()`).
    BannedSymbol {
        segments: &["thread", "spawn"],
        class: LintClass::Deny,
        io: false,
        key: "std::thread::spawn",
        message: "`thread::spawn` creates an unmanaged thread whose interleaving is not \
                  reproducible, voiding bit-for-bit replay determinism. Express concurrency \
                  through the scheduler instead — split the work across nodes, which the \
                  runtime already runs in parallel where the graph allows it. If this node \
                  is genuinely allowed to be non-deterministic, opt out with \
                  `#[cerulion_node(allow_non_deterministic)]`.",
    },
    // ----- Warn (record-only in the macro; CLI surfaces later) -----
    // `fs::read_dir` is the sole IO-class warn (`io: true`): live-filesystem
    // IO determinism is owned by `uses_live_io` + replay, not by a partial
    // fs blocklist, so it is a NUDGE to declare `uses_live_io`, not a deny.
    BannedSymbol {
        segments: &["fs", "read_dir"],
        class: LintClass::Warn,
        io: true,
        key: "std::fs::read_dir",
        message: "`fs::read_dir` performs live filesystem IO whose result (entry order, \
                  contents) is not reproducible across runs. If this node genuinely performs \
                  live IO, declare it with `#[cerulion_node(uses_live_io)]` so the framework \
                  records it for replay verification; otherwise pre-stage the inputs and read \
                  them through the graph. (This lint flags one common call, not every form of \
                  live IO — the `uses_live_io` declaration is what makes a node's IO explicit.)",
    },
    BannedSymbol {
        segments: &["env", "var"],
        class: LintClass::Warn,
        io: false,
        key: "std::env::var",
        message: "`std::env::var` reads the live process environment, which is not captured \
                  in the replay trace. Read it through the environment snapshot the runtime \
                  took at build time instead: `ctx.env_str(\"NAME\", \"default\")`, or \
                  `ctx.env(\"NAME\", default)` for a value that parses (both on the \
                  `&NodeContext` your `init` receives). Store what you read on the node so \
                  `tick` can use it.",
    },
    BannedSymbol {
        segments: &["rand", "thread_rng"],
        class: LintClass::Warn,
        io: false,
        key: "rand::thread_rng",
        message: "`rand::thread_rng()` is seeded from the OS per thread, so it draws a \
                  different sequence every run and voids replay determinism. Cerulion does \
                  not provide a seeded RNG yet: construct one yourself from a fixed seed you \
                  own (for example `rand::rngs::StdRng::seed_from_u64(seed)`), keep it on the \
                  node so the sequence is a function of node state, and take the seed from \
                  configuration rather than the environment.",
    },
    BannedSymbol {
        segments: &["rand", "random"],
        class: LintClass::Warn,
        io: false,
        key: "rand::random",
        message: "`rand::random()` draws from the thread-local RNG, which is seeded from the \
                  OS, so it yields a different sequence every run and voids replay \
                  determinism. Cerulion does not provide a seeded RNG yet: construct one \
                  yourself from a fixed seed you own (for example \
                  `rand::rngs::StdRng::seed_from_u64(seed)`), keep it on the node so the \
                  sequence is a function of node state, and take the seed from configuration \
                  rather than the environment.",
    },
    BannedSymbol {
        segments: &["thread", "sleep"],
        class: LintClass::Warn,
        io: false,
        key: "std::thread::sleep",
        message: "`thread::sleep` blocks on the live wall clock, coupling node timing to real \
                  time and undermining deterministic re-execution — it also stalls every \
                  other node the scheduler runs on this thread. Drive timing through the \
                  scheduler instead: `#[cerulion_node(period_ms = N)]` to fire on a period, \
                  or `throttle_ms` to cap how often a node fires.",
    },
    BannedSymbol {
        segments: &["process", "id"],
        class: LintClass::Warn,
        io: false,
        key: "std::process::id",
        message: "`std::process::id()` returns a per-run process id that is not reproducible \
                  across runs. Avoid making node output depend on it — if you need a stable \
                  identity, use the node's own id from the graph.",
    },
    BannedSymbol {
        // `std::thread::current().id()` — the `.id()` here is a method call on
        // the value returned by `current()`, NOT a path-tail call, so the
        // strict last-two matcher (`["Thread", "id"]`) does not catch the live
        // call shape. This warn row exists for the deferred core half (which
        // owns warn detection + surfacing); the macro half is deny-only and
        // does not detect it. The `["Thread", "id"]` tail is the canonical key
        // the core half will key off when it re-walks node source.
        segments: &["Thread", "id"],
        class: LintClass::Warn,
        io: false,
        key: "std::thread::current().id",
        message: "`std::thread::current().id()` returns a per-run thread id that is not \
                  reproducible across runs — the scheduler is also free to run a node on a \
                  different thread from one run to the next. Avoid making node output depend \
                  on it; if you need a stable identity, use the node's own id from the graph.",
    },
];

/// Compile-time contract check over [`BANNED`]. Makes two
/// contract-invalid states UNREPRESENTABLE — they fail the build, not a
/// runtime test:
///
/// 1. **Every row is a strict two-segment tail.** The matcher
///    ([`match_deny`] / [`match_warn`]) only ever compares a 2-segment
///    `tail`, so a row with a different `segments.len()` could never match
///    (silently dead). Asserting `== 2` makes a mis-sized row a build error.
/// 2. **No IO-class row is a `Deny`.** IO-class severity is owned by
///    `#[cerulion_node(uses_live_io)]` + replay (see module docs), so an
///    `io: true` row MUST be `Warn`. `BannedSymbol { io: true, class: Deny }`
///    is representable in the struct but contract-invalid; this asserts it
///    away at compile time.
///
/// A `const fn` loop with `assert!` runs in const context (stable since Rust
/// 1.79's const-eval `assert!` / loop support) at zero runtime cost — the
/// build fails if any row violates the contract. The runtime
/// `deny_set_is_exactly_the_four_killers` / `read_dir_is_the_only_io_class_row`
/// tests below still pin the EXACT table contents (a stronger, table-specific
/// claim); this const check pins the per-row STRUCTURAL invariant for every
/// present and future row.
const _: () = {
    const fn assert_banned_table_invariants() {
        let mut i = 0;
        while i < BANNED.len() {
            let sym = &BANNED[i];
            // Invariant 1: strict two-segment tail (else the matcher can
            // never reach this row).
            assert!(
                sym.segments.len() == 2,
                "BANNED contract: every row must have exactly two path \
                 segments (the matcher only compares a 2-segment tail)"
            );
            // Invariant 2: io-class implies warn (io severity is owned by
            // `uses_live_io` + replay, never a hard deny).
            assert!(
                !(sym.io && sym.class.is_deny()),
                "BANNED contract: an IO-class row (`io: true`) must be \
                 `Warn`, not `Deny` — live-IO severity is owned by \
                 `uses_live_io` + replay, not a syntactic deny"
            );
            i += 1;
        }
    }
    assert_banned_table_invariants();
};

/// Look up the banned-symbol row whose **strict last-two** segments match
/// `tail` (a slice of the callee path's segment idents). Returns the first
/// matching `Deny` row if any (those drive compile errors); a caller wanting
/// warn-class rows should iterate [`BANNED`] directly or call [`match_warn`].
///
/// `tail` must be the LAST segments of the callee path (the matcher in
/// `impl_macro.rs` slices off the last two). A row matches only when its
/// `segments` length equals `tail` length and every segment is equal.
pub fn match_deny(tail: &[String]) -> Option<&'static BannedSymbol> {
    BANNED.iter().find(|sym| {
        sym.class.is_deny() && sym.segments.len() == tail.len() && segments_eq(sym.segments, tail)
    })
}

/// Look up the warn-class row matching `tail`, if any. Provided for the
/// deferred core half (which will re-walk node source at the CLI and surface
/// warn-class hits at graph load); the macro half does NOT call this — it emits
/// deny errors only (see module docs).
///
/// `#[allow(dead_code)]`: exercised by this module's unit tests and reserved
/// for the deferred core half. It is the warn-side twin of
/// [`match_deny`] and stays here so the matching recipe lives in ONE place for
/// both halves; the macro half intentionally does not call it.
#[allow(dead_code)]
pub fn match_warn(tail: &[String]) -> Option<&'static BannedSymbol> {
    BANNED.iter().find(|sym| {
        sym.class == LintClass::Warn
            && sym.segments.len() == tail.len()
            && segments_eq(sym.segments, tail)
    })
}

fn segments_eq(expected: &[&'static str], actual: &[String]) -> bool {
    expected.len() == actual.len() && expected.iter().zip(actual).all(|(e, a)| *e == a.as_str())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn class_is_deny_reports_correctly() {
        assert!(LintClass::Deny.is_deny());
        assert!(!LintClass::Warn.is_deny());
    }

    #[test]
    fn instant_now_matches_deny_strict_tail() {
        let tail = vec!["Instant".to_string(), "now".to_string()];
        let m = match_deny(&tail).expect("Instant::now must match a deny row");
        assert_eq!(m.class, LintClass::Deny);
        assert!(!m.io, "Instant::now is not IO-class");
        assert_eq!(m.key, "std::time::Instant::now");
    }

    #[test]
    fn fs_read_dir_is_io_class_warn_not_deny() {
        // Revision: `fs::read_dir` is now a WARN (IO-class), NOT a
        // deny. It must NOT match a deny row, MUST match a warn row, and MUST
        // carry `io: true` so `uses_live_io` suppresses it.
        let tail = vec!["fs".to_string(), "read_dir".to_string()];
        assert!(
            match_deny(&tail).is_none(),
            "fs::read_dir is warn-class, must not match deny"
        );
        let w = match_warn(&tail).expect("fs::read_dir must match a warn row");
        assert_eq!(w.class, LintClass::Warn);
        assert!(
            w.io,
            "fs::read_dir must be IO-class so uses_live_io suppresses it"
        );
        assert_eq!(w.key, "std::fs::read_dir");
    }

    #[test]
    fn read_dir_is_the_only_io_class_row() {
        // Pins the locked contract: today exactly one IO-class row exists.
        // A future IO-class addition is an explicit decision, not an
        // accident — this test forces it to be acknowledged.
        let io_keys: Vec<&str> = BANNED.iter().filter(|s| s.io).map(|s| s.key).collect();
        assert_eq!(io_keys, vec!["std::fs::read_dir"]);
    }

    #[test]
    fn deny_set_is_exactly_the_four_killers() {
        // Locked DENY set: the unambiguous time/thread killers only. The
        // tokio `Instant::now` is caught via the shared std `Instant::now`
        // tail, so it carries no separate row (hence 3 rows, 4 symbols).
        let deny_keys: Vec<&str> = BANNED
            .iter()
            .filter(|s| s.class.is_deny())
            .map(|s| s.key)
            .collect();
        assert_eq!(
            deny_keys,
            vec![
                "std::time::Instant::now",
                "std::time::SystemTime::now",
                "std::thread::spawn",
            ],
            "DENY set must be exactly the unambiguous killers (tokio Instant::now \
             shares the std Instant::now tail); read_dir is a warn"
        );
        // And no deny row is IO-class (the deny set carries no fs/socket IO).
        assert!(
            BANNED.iter().filter(|s| s.class.is_deny()).all(|s| !s.io),
            "no deny row may be IO-class"
        );
    }

    #[test]
    fn env_var_is_warn_not_deny() {
        let tail = vec!["env".to_string(), "var".to_string()];
        assert!(
            match_deny(&tail).is_none(),
            "env::var is warn-class, must not match deny"
        );
        let w = match_warn(&tail).expect("env::var must match a warn row");
        assert_eq!(w.key, "std::env::var");
    }

    #[test]
    fn unknown_symbol_matches_nothing() {
        let tail = vec!["my_module".to_string(), "compute".to_string()];
        assert!(match_deny(&tail).is_none());
        assert!(match_warn(&tail).is_none());
    }

    #[test]
    fn single_segment_tail_matches_nothing() {
        // All rows are two-segment; a one-segment tail can't match.
        let tail = vec!["now".to_string()];
        assert!(match_deny(&tail).is_none());
        assert!(match_warn(&tail).is_none());
    }

    #[test]
    fn no_two_deny_rows_share_a_tail() {
        // The matcher is first-match-wins; two deny rows with the same tail
        // would make the second dead. (The tokio Instant::now tail is
        // covered by the std row WITHOUT a duplicate row by design.)
        let deny: Vec<_> = BANNED.iter().filter(|s| s.class.is_deny()).collect();
        for i in 0..deny.len() {
            for j in (i + 1)..deny.len() {
                assert_ne!(
                    deny[i].segments, deny[j].segments,
                    "duplicate deny tail: {} vs {}",
                    deny[i].key, deny[j].key
                );
            }
        }
    }
}
