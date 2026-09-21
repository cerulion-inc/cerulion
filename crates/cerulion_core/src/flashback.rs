// SPDX-License-Identifier: AGPL-3.0-only
//! The one always-on capture plane's policy: the
//! kill switch, the anchor cadence, and the arm-time RAM projection that refuses
//! to arm a process whose state is too big to checkpoint affordably.
//!
//! Everything here is PURE: it takes numbers and strings and returns decisions.
//! The two OS reads it depends on
//! ([`sample_anon_rss`](crate::state_carrier::fork::sample_anon_rss) and
//! [`sample_mem_available`](crate::state_carrier::fork::sample_mem_available))
//! happen at the
//! call site and arrive as `Option<u64>`, so every branch below — including the
//! ones a healthy desk can never reach — is oracle-testable with no `/proc`, no
//! SHM and no second process.
//!
//! # What the always-on plane changed, and why this module exists at all
//!
//! Before it, node-state capture was OPT-IN: a recorder created the arm word
//! (`cerulion bagd --state-arm <tag>`), the graph opened it by name, and the two
//! halves had to be paired by hand BEFORE the graph started. Nothing on the
//! `graph run --record` path minted a tag, so in practice nothing was ever armed
//! and no recording was resimmable.
//!
//! That was collapsed into one plane that every serving graph runs, so that
//! **every recording is resimmable with no flag** and the Flashback retention
//! (the rolling window a later PR lands) has anchors to hold. An always-on plane
//! cannot ask permission, so what replaces the opt-in is:
//!
//! 1. an opt-OUT — [`FLASHBACK_ENV`], the `CERULION_TOPIC_LIVENESS=off` posture:
//!    read once, honoured for the process's life, loud when it is off; and
//! 2. an AUTOMATIC safety — the arm-time projection below, so a robot whose state
//!    is too large to fork cheaply is told the price and left un-armed rather than
//!    silently made to pay it every cadence.
//!
//! # The cost this gate is pricing (the measured basis)
//!
//! A checkpoint anchor `fork(2)`s the worker. Two costs follow, and they differ by
//! ~50×:
//!
//! * the **fork** itself — MEASURED at 0.70–0.85 ms per GiB of private anon on the
//!   Jetson (L4T 5.15, THP=`always`, the shipping default) and ~2.3–2.5 ms/GiB on
//!   an M3 Mac, with a 150–270 µs floor. `MAP_SHARED` mappings (the big iceoryx2
//!   pools) are ≈ free at fork.
//! * the post-fork **CoW smear** — the parent re-faulting the pages it writes after
//!   the child took its snapshot. MEASURED: a dense 256 MiB slab front-loads its
//!   ENTIRE copy into the first post-fork tick — 91.6–92.8 ms, 11.4–11.6× that
//!   node's baseline tick — after which p99 returns to baseline.
//!
//! The smear is FRONT-LOADED rather than spread, which is what makes cadence the
//! lever: the whole cost lands once per anchor, so halving the anchor rate halves
//! the duty. It is also what makes a per-anchor gate insufficient on its own — a
//! process big enough to smear for a second will smear for a second at EVERY
//! cadence, and telling its operator once at arm time is worth more than declining
//! quietly forever.
//!
//! # What is deliberately NOT projected here
//!
//! The full arm-time projection is a SUM: the
//! ring's two in-flight anchors (`2 × Σ state_bytes`), the parent's CoW spike
//! (`≤ anon_RSS`), the child's sort index, and the frame window. Only the CoW
//! spike is knowable at arm time from inside the process, and it is the dominant
//! term for exactly the nodes this mechanism exists for (a node that
//! rewrites its map every tick saturates the bound within one tick). So the gate
//! projects THAT and says so; the ring reservation is REPORTED beside it rather
//! than folded in, because it is an SHM reservation whose sizing belongs to the
//! Flashback retention PR, and the frame window does not exist yet.
//!
//! NOTE: this module is compiled only on Unix — the `#[cfg(unix)]` gate lives on
//! its `pub mod flashback;` declaration in `lib.rs` — because everything it gates
//! is.

// The two halves of the RETENTION the module docs above promise —
// the rule→action seam that decides WHEN a capture happens, and the dashcam
// contract that decides which captures a robot keeps. Both are PURE for the same
// reason this file is, and both live beside it rather than in the recorder because
// the recorder is only one of their callers: the monitors-UI work reaches
// `trigger` by constructing a cause, and the CLI reaches `retention` to render a
// directory.
// The `/__cerulion/flashback` TRIGGER CHANNEL — how an observer
// in one process reaches the recorder in another.
//
// A SIBLING of `trigger` and `retention` rather than a `transport` module, and
// the cfg guard is what settled it: the channel's whole vocabulary is
// `trigger`'s (`CaptureRequest` in, `SuppressReason` back), so a portable
// `transport::flashback_channel` referenced this `#[cfg(unix)]`-only module from
// outside it — which breaks a non-unix `cargo check` on the `use` and CI's
// Documentation job on the doc link. Living HERE makes it unix-only
// structurally, by the same rule that already covers its two siblings, instead
// of by an attribute somebody has to remember.
// The machine number the window cap is a fraction of. A
// SIBLING here rather than a top-level `cerulion_core::machine_mem`, for the same
// structural reason `channel` gives: everything that reads it is `#[cfg(unix)]`,
// and living inside this module makes that true by construction instead of by an
// attribute somebody has to remember.
pub mod channel;
// The producer fault-injection seam. A SIBLING here for the same
// structural reason `channel` gives — every consumer is `#[cfg(unix)]` — and
// always-on (never feature-gated) because one of those consumers lives in
// `cerulion_cli_engine`, whose test feature is independent of this crate's.
pub mod fault_injection;
pub mod machine_mem;
pub mod resim;
pub mod retention;
pub mod switch;
pub mod trigger;

/// The kill switch: set to `off` to disable the whole capture plane.
///
/// The [`crate::transport::liveness`] posture, deliberately: an env var read ONCE
/// (nothing re-reads it mid-run, so a run's behaviour cannot change under it), a
/// single value that means off, and a LOUD line when it is honoured — an operator
/// who turned a feature off must be able to see, in the log of the run they are
/// debugging, that they did.
pub const FLASHBACK_ENV: &str = "CERULION_FLASHBACK";

/// Override the anchor cadence, in MILLISECONDS of logical time.
///
/// The cadence is the one real lever on the standing cost (see the module docs:
/// the smear is front-loaded, so the duty is `smear / cadence`), which is why it
/// is reachable at all. It is stated in wall-ish milliseconds because that is the
/// unit an operator can reason about, and converted to STEPS at arm time by
/// [`cadence_steps`] — the cadence itself is never a wall timer.
pub const FLASHBACK_CADENCE_MS_ENV: &str = "CERULION_FLASHBACK_CADENCE_MS";

/// Override the arm-time state-size ceiling, in MEBIBYTES.
///
/// Tunable, unlike [`crate::state_carrier::fork::CAPTURE_MEM_FLOOR_BYTES`], and the
/// difference is not an inconsistency: the floor protects the LIVE GRAPH from an
/// anchor that would take the machine's last memory, and turning it down on the
/// box where it matters most is exactly the foot-gun that doc refuses. This
/// ceiling protects the OPERATOR from a standing cost they may knowingly accept —
/// a big-state robot whose duty cycle genuinely has room for a 1 s smear every
/// 15 s can say so, and the alternative is not "safer", it is "no black box".
pub const FLASHBACK_MAX_STATE_MB_ENV: &str = "CERULION_FLASHBACK_MAX_STATE_MB";

/// The default anchor cadence: 15 s of logical time.
///
/// Against the measured basis it is a duty cycle: the
/// worst-case state this gate admits smears for [`FLASHBACK_STALL_BUDGET_MS`], so
/// the plane's standing cost is at most `STALL_BUDGET / CADENCE` of one node's
/// thread — 250 ms in 15 s, 1.7 %, once per cadence and front-loaded rather than
/// smeared across it.
///
/// It is also the window arithmetic's input: a Flashback covering `[T−15s, T+15s]`
/// needs an anchor at or before `T−15s`, and with cadence `C` the newest such
/// anchor is at worst `T−15s−C` old, so the frame window the retention PR holds
/// must span `15 s + C`. `C = 15 s` makes that 30 s — the number the retention
/// design sizes the ~155 MB frame window from.
pub const DEFAULT_FLASHBACK_CADENCE_MS: u64 = 15_000;

/// The largest first-tick CoW smear the always-on plane may impose on a node
/// thread, in milliseconds.
///
/// A quarter of a second is a long time on a robot, and that is the point: this is
/// the pain the plane is ALLOWED to cause, once per cadence, in exchange for being
/// a black box nobody had to arm. A 1 kHz control loop misses ~250 ticks; its
/// catch-up burst is what the `Period` catch-up clamp exists to bound.
pub const FLASHBACK_STALL_BUDGET_MS: u64 = 250;

/// Milliseconds of first-tick stall per GiB of private anonymous memory.
///
/// DERIVED from the measured basis rather than picked: a dense 256 MiB slab
/// front-loads its whole copy into the first post-fork tick at 91.6–92.8 ms, i.e.
/// 366.4–371.2 ms/GiB, to which the fork itself adds 0.70–0.85 ms/GiB on the same
/// box. Rounded UP to 380 so the projection errs toward refusing — the direction
/// that costs a big robot its black box, never a big robot its control loop.
///
/// Measured on the Jetson under THP=`always` (the shipping L4T default), which is
/// the unfavourable setting: with transparent huge pages every CoW fault copies
/// 2 MiB instead of 4 KiB. A box with THP `never` will smear LESS than this
/// predicts, so the number is an upper bound there too.
pub const FLASHBACK_STALL_MS_PER_GIB: u64 = 380;

/// Bytes in a GiB — the unit [`FLASHBACK_STALL_MS_PER_GIB`] is quoted in.
const BYTES_PER_GIB: u64 = 1024 * 1024 * 1024;

/// The default state-size ceiling: the largest private-anon footprint whose
/// projected first-tick smear still fits [`FLASHBACK_STALL_BUDGET_MS`].
///
/// COMPUTED from the two constants above rather than written down, so the
/// derivation cannot rot: change the budget or the measured rate and the ceiling
/// follows. At the shipped values that is 250/380 GiB ≈ 673 MiB.
pub const fn default_max_state_bytes() -> u64 {
    // Both operands are small compile-time constants; the multiply cannot overflow
    // a u64 (250 × 2^30).
    FLASHBACK_STALL_BUDGET_MS * BYTES_PER_GIB / FLASHBACK_STALL_MS_PER_GIB
}

// The derivation is the doc's whole claim, so it is pinned at COMPILE time in both
// directions: the ceiling must project a stall AT the budget, and one byte more
// must not. (Integer division makes the first inequality `<=`.)
const _: () = assert!(default_max_state_bytes() > 0);
const _: () = assert!(
    default_max_state_bytes() * FLASHBACK_STALL_MS_PER_GIB / BYTES_PER_GIB
        <= FLASHBACK_STALL_BUDGET_MS
);

/// What [`FLASHBACK_ENV`] said.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlaneSwitch {
    /// Unset, empty, or explicitly on. The plane runs.
    On,
    /// `off` (any case, surrounding whitespace trimmed). The plane does not run.
    Off,
    /// A value this build does not recognise — REPORTED, never guessed at.
    ///
    /// The plane still runs (an unreadable instruction must not silently disable a
    /// safety feature), but the caller warns with the offending text. Folding this
    /// into [`On`](Self::On) would mean `CERULION_FLASHBACK=of` silently left the
    /// plane armed with the operator believing otherwise, which is the class of
    /// silent inference this codebase refuses.
    Unrecognized(String),
}

/// PURE: read the kill switch.
///
/// Unset and empty are both [`On`](PlaneSwitch::On) — an env var set to `""` is
/// what a shell script that computed no value leaves behind, and it is not an
/// instruction. `off`, `0` and `false` are all [`Off`](PlaneSwitch::Off) because an
/// operator reaching for a kill switch reaches for whichever of those they know;
/// `on`, `1` and `true` are the explicit affirmative. Anything else is
/// [`Unrecognized`](PlaneSwitch::Unrecognized).
///
/// # The absent arm is KILL-SWITCH semantics, and it does not travel
///
/// `None` ⇒ `On` is right for [`FLASHBACK_ENV`]: one variable governing whether a
/// safety feature runs at all, which SHIPS on, so an operator who has never heard
/// of it gets it. Absence there means "nobody reached for the kill switch".
///
/// It is WRONG for a PREFERENCE switch, whose default is written down per row —
/// [`TriggerSwitch::default_on`](switch::TriggerSwitch::default_on) is `off` for
/// one of its eight rows. Absence there means "the operator expressed no
/// preference", which must resolve to that row's own default. Handing an unset
/// per-row variable to this function forces it on, which makes the
/// `silent` demotion inert (measured: a healthy Go2
/// captured 82 MB eleven seconds after boot).
///
/// So a per-row caller decides absent and empty BEFORE calling this, and only a
/// value an operator really typed reaches it —
/// [`TriggerPosture::resolve`](switch::TriggerPosture::resolve) is the worked
/// example. This function stays as it is: re-pointing its absent arm would silently
/// disable Flashback on every robot that has never set `CERULION_FLASHBACK`.
pub fn parse_plane_switch(raw: Option<&str>) -> PlaneSwitch {
    let Some(value) = raw else {
        return PlaneSwitch::On;
    };
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return PlaneSwitch::On;
    }
    match trimmed.to_ascii_lowercase().as_str() {
        "off" | "0" | "false" | "no" => PlaneSwitch::Off,
        "on" | "1" | "true" | "yes" => PlaneSwitch::On,
        _ => PlaneSwitch::Unrecognized(trimmed.to_string()),
    }
}

/// PURE: a numeric env override — `None` when the operator supplied no usable one,
/// plus the reason it was not honoured.
///
/// # `Option`, never a sentinel value
///
/// The presence of an override and its VALUE are different facts, and folding them
/// into one number means some legal value has to stand in for "absent". This
/// function returned `default` for both, so a caller could not tell an operator who
/// typed the default from one who typed nothing — and the first attempt at the
/// caller's own version of this used `u64::MAX` as its sentinel, which made
/// `CERULION_FLASHBACK_MAX_STATE_MB=18446744073709551615` — a value that PARSES,
/// and the largest one anybody can express — silently mean "no override" and
/// restore the default. An operator asking for no practical ceiling got the
/// tightest one instead: the exact inversion of what they typed, with nothing said.
///
/// So the ABSENCE is carried in the type. `default` is taken ONLY to render the
/// complaint text; no arm returns it, so no caller can recover a value from a
/// sentinel by accident.
///
/// A `Some` complaint is what the caller warns with. Returning it alongside the
/// value rather than as a `Result` is deliberate: an unparseable knob must never
/// stop a robot running its graph, and must never be silently ignored either.
///
/// # `default_for_message` is a `Display`, not a number
///
/// It renders THE VALUE THAT WILL BE IN FORCE, and for one of the two callers that
/// value is not expressible in the unit the knob is typed in: the state ceiling is a
/// DERIVATION (706 409 094 B = 673.68 MiB), so quoting it as a whole number of
/// mebibytes told an operator "using the default 673" while 673.68 MiB was what
/// actually applied — and an operator who then typed `673`, believing they were
/// restating the default, would have TIGHTENED the ceiling by 717 446 B. Taking a
/// `Display` lets each caller render its own default exactly, without this function
/// having to know that one of them is not a round number of its own unit.
// `pub(crate)` rather than private since `retention` resolves two
// knobs of its own and must do it through THIS parser, not a second copy. The
// absence-in-the-type rule and the quote-the-default-in-force rule below were both
// paid for once (see the doc); a sibling module re-deriving them would re-open the
// exact defects they close.
pub(crate) fn resolve_positive_override(
    raw: Option<&str>,
    default_for_message: &dyn std::fmt::Display,
    unit: &str,
) -> (Option<u64>, Option<String>) {
    let Some(value) = raw else {
        return (None, None);
    };
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return (None, None);
    }
    match trimmed.parse::<u64>() {
        Ok(0) => (
            None,
            Some(format!(
                "0 {unit} is not a usable value (it would make the plane meaningless); \
                 using the default {default_for_message}"
            )),
        ),
        Ok(v) => (Some(v), None),
        Err(e) => (
            None,
            Some(format!(
                "{trimmed:?} is not a whole number of {unit} ({e}); using the default \
                 {default_for_message}"
            )),
        ),
    }
}

/// How far back the rolling FRAME window reaches, in milliseconds.
///
/// DERIVED, and the derivation is the whole window arithmetic: a Flashback
/// covering `[T−post, T+post]` needs an anchor at or before `T−post`, and with
/// cadence `C` the newest such anchor is at worst `T−post−C` old — so the window
/// must span `post + C`. At the shipped 15 s post window and 15 s cadence that is
/// 30 s, the number the ~155 MB standing window was priced from.
///
/// Written as the SUM rather than as `30_000`, so moving either input moves this
/// with it. A literal here would silently under-span the moment the cadence
/// changed, and the failure would be a capture whose oldest anchor is younger than
/// its oldest frame — resimmable from a point AFTER the thing that went wrong.
pub const DEFAULT_FLASHBACK_WINDOW_MS: u64 =
    crate::flashback::trigger::DEFAULT_FLASHBACK_POST_WINDOW_MS + DEFAULT_FLASHBACK_CADENCE_MS;

// The claim above, at COMPILE time: the window must reach back at least as far as
// one post window plus one cadence, or a capture cannot be anchored at its own
// start.
const _: () = assert!(
    DEFAULT_FLASHBACK_WINDOW_MS
        >= crate::flashback::trigger::DEFAULT_FLASHBACK_POST_WINDOW_MS
            + DEFAULT_FLASHBACK_CADENCE_MS
);

/// Override [`DEFAULT_FLASHBACK_WINDOW_MS`].
pub const FLASHBACK_WINDOW_MS_ENV: &str = "CERULION_FLASHBACK_WINDOW_MS";

/// The hard BYTE ceiling on the rolling window, in mebibytes.
///
/// # Why a second cap, when the span already bounds it
///
/// The span bounds the window in TIME and says nothing about bytes: 30 s of a
/// Go2-scale robot is ~155 MB, and 30 s of a robot streaming four 4K cameras is
/// not. Without this the standing cost of an always-on plane would be set by
/// whatever the busiest graph on the machine happens to publish, which is the one
/// number an operator cannot predict when they decide to leave it on.
///
/// So the span is the PROMISE and this is the BACKSTOP: past it the oldest frames
/// go even though they are inside the span, and a capture that loses frames this
/// way SAYS so rather than quietly covering less than it claims.
///
/// # This is the floor, not the cap
///
/// As a fixed cap it is 2× the priced Go2 window, one number an
/// operator can predict. What a fixed number cannot do is scale: 320 MiB holds
/// the promised 30 s only up to ~11 MB/s aggregate, so a Go2 (5.1 MB/s, measured)
/// fits at 2.2× margin while a humanoid at realistic payloads (~230 MB/s) gets
/// ~1.5 s and a raw four-camera construct ~0.4 s — on a desk with 128 GB of RAM
/// sitting idle.
///
/// The cap is therefore DERIVED ([`window_cap_from_eff_ram`]) and this constant is
/// its LOWER bound: the static number is the floor, so the derivation can
/// only ever RAISE a window, never shrink one. That is also what makes the
/// degradation safe — a machine whose memory cannot be read lands exactly here,
/// i.e. on the static default.
pub const DEFAULT_FLASHBACK_WINDOW_MAX_MB: u64 = 320;

/// Override the derived window cap ([`window_cap_from_eff_ram`]), in mebibytes.
///
/// An explicit value WINS over the derivation — the operator knows something the
/// machine's RAM does not say. It also drags the anchor ceiling with it (see
/// [`resolve_anchor_max_bytes`]).
pub const FLASHBACK_WINDOW_MAX_MB_ENV: &str = "CERULION_FLASHBACK_WINDOW_MAX_MB";

/// The fraction of effective RAM the frame window may occupy.
///
/// 1/16 = 6.25 % for the window, and the plane is 2× that (the anchor follows the window) — 12.5 % of
/// the machine, whatever the machine is. The COST is stated rather than implied:
/// on an 8 GB Orin NX (real `MemTotal` ~7.4 GiB) the plane grows from the static
/// 640 MiB to ~930 MiB, i.e. 12.5 % of the machine against 7.8 % — the most
/// memory-pressured class pays MORE, not the same, which is the price of one
/// formula holding everywhere.
pub const FLASHBACK_WINDOW_RAM_DIVISOR: u64 = 16;

/// The floor the derivation clamps to — the static default, in bytes.
///
/// DERIVED from [`DEFAULT_FLASHBACK_WINDOW_MAX_MB`] rather than written again, so
/// the two cannot drift into an operator reading one figure and getting another.
pub const FLASHBACK_WINDOW_FLOOR_BYTES: u64 = DEFAULT_FLASHBACK_WINDOW_MAX_MB * 1024 * 1024;

/// The ceiling the derivation clamps to.
///
/// 8 GiB, reached at 128 GB of effective RAM. It exists because the fraction is a
/// SHARE and a share of a very large machine stops being a proportionate standing
/// cost and starts being an unbounded one: a desk with 512 GB would otherwise hold
/// a 32 GiB window (64 GiB of plane) for a graph nobody asked to record. Past this
/// point the operator's own override is the way up — a window that big is a
/// deliberate act, not a default.
pub const FLASHBACK_WINDOW_CEILING_BYTES: u64 = 8 * 1024 * 1024 * 1024;

// The clamp must be a clamp. A floor at or above the ceiling would make
// `clamp` panic, and a floor that is not the static default would make "only ever
// raises a window" false — both are properties of the constants and nothing else, so both are
// checked at COMPILE time.
const _: () = assert!(FLASHBACK_WINDOW_FLOOR_BYTES < FLASHBACK_WINDOW_CEILING_BYTES);
const _: () = assert!(FLASHBACK_WINDOW_RAM_DIVISOR > 0);

/// Where a resolved byte ceiling came from.
///
/// Carried out of the resolvers rather than re-derived by whoever renders it,
/// because ONE spawn line has to explain the cap ("window cap
/// 4 GiB = machine 64 GiB / 16") and a bare number cannot produce that sentence.
/// The variants are the four possible answers, and they are kept apart because their
/// REMEDIES differ: an env cap is changed by editing the environment, a derived
/// one by the machine it runs on, a floored one is already at the static default, and
/// an unknown-RAM one means a read failed and is worth an operator's attention.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CapBasis {
    /// The operator's own env override. It wins over every derivation.
    Env,
    /// Derived from this machine's effective RAM.
    Ram {
        /// What [`machine_mem::sample_effective_ram`] reported.
        eff_ram_bytes: u64,
        /// Which source that figure came from — the machine, or a tighter cgroup.
        ram_basis: machine_mem::RamBasis,
        /// Whether the clamp moved the derived figure, and which way.
        clamp: CapClamp,
    },
    /// Neither the machine total nor a cgroup limit could be read, so the
    /// static floor applies.
    ///
    /// NOT a refusal and not a degradation of behaviour: the floor IS the
    /// static default, so a machine this build cannot
    /// measure keeps exactly that window. It is reported because a read
    /// failure on a platform that should be able to answer is worth knowing about.
    UnknownRam,
    /// DERIVED-EQUAL to the resolved window cap — the anchor's default.
    Window,
}

/// Whether the clamp bound, and on which side.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CapClamp {
    /// `effRAM / divisor` fell inside the band and stands as computed.
    Unclamped,
    /// It fell BELOW [`FLASHBACK_WINDOW_FLOOR_BYTES`], so the floor applies.
    Floor,
    /// It rose ABOVE [`FLASHBACK_WINDOW_CEILING_BYTES`], so the ceiling applies.
    Ceiling,
}

/// A byte ceiling and the reason it is that number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResolvedCap {
    /// The ceiling in force, in bytes.
    pub bytes: u64,
    /// Where it came from.
    pub basis: CapBasis,
}

/// PURE: the RAM derivation — `clamp(effRAM / 16, 320 MiB, 8 GiB)`.
///
/// `None` effective RAM lands on the FLOOR, never on zero and never on a refusal:
/// the whole degradation ladder (a cgroup that cannot be read falls back to the
/// machine total, a machine total that cannot be read falls back to here) is built
/// so that a measurement failure leaves the plane exactly as capable as the
/// static default makes it. Failing the other way would mean a robot losing its
/// black box because `/proc` moved.
pub fn window_cap_from_eff_ram(eff_ram: Option<machine_mem::EffectiveRam>) -> ResolvedCap {
    let Some(eff_ram) = eff_ram else {
        return ResolvedCap {
            bytes: FLASHBACK_WINDOW_FLOOR_BYTES,
            basis: CapBasis::UnknownRam,
        };
    };
    let share = eff_ram.bytes / FLASHBACK_WINDOW_RAM_DIVISOR;
    let clamp = if share < FLASHBACK_WINDOW_FLOOR_BYTES {
        CapClamp::Floor
    } else if share > FLASHBACK_WINDOW_CEILING_BYTES {
        CapClamp::Ceiling
    } else {
        CapClamp::Unclamped
    };
    ResolvedCap {
        bytes: share.clamp(FLASHBACK_WINDOW_FLOOR_BYTES, FLASHBACK_WINDOW_CEILING_BYTES),
        basis: CapBasis::Ram {
            eff_ram_bytes: eff_ram.bytes,
            ram_basis: eff_ram.basis,
            clamp,
        },
    }
}

/// PURE: the rolling window's span in MILLISECONDS, from
/// [`FLASHBACK_WINDOW_MS_ENV`].
pub fn resolve_window_ms(raw: Option<&str>) -> (u64, Option<String>) {
    let (value, complaint) =
        resolve_positive_override(raw, &DEFAULT_FLASHBACK_WINDOW_MS, "milliseconds");
    (value.unwrap_or(DEFAULT_FLASHBACK_WINDOW_MS), complaint)
}

/// PURE: the rolling window's byte ceiling — [`FLASHBACK_WINDOW_MAX_MB_ENV`] over
/// the RAM derivation.
///
/// The env WINS: the derivation is a default, and an operator who sized
/// their window by hand knows something `MemTotal` does not. Absent one, the
/// answer is [`window_cap_from_eff_ram`], which is why the effective RAM is an
/// ARGUMENT rather than sampled here — the derivation is then one function with no
/// hidden input, oracle-testable across every machine shape without owning one.
///
/// SATURATING on the multiply, for the same reason [`resolve_max_state_bytes`] is:
/// an astronomical ask means "no practical ceiling", which is exactly what an
/// operator typing one means.
pub fn resolve_window_max_bytes(
    raw: Option<&str>,
    eff_ram: Option<machine_mem::EffectiveRam>,
) -> (ResolvedCap, Option<String>) {
    let derived = window_cap_from_eff_ram(eff_ram);
    // The complaint quotes THE VALUE THAT WILL BE IN FORCE, which is the derived
    // figure rather than a constant — `effRAM / 16` is not a whole number of
    // mebibytes, and the `DefaultStateCeiling` lesson is that quoting a
    // rounded default invites an operator to restate it and tighten the ceiling.
    let (value, complaint) =
        resolve_positive_override(raw, &MebibyteCeiling(derived.bytes), "mebibytes");
    let Some(mb) = value else {
        return (derived, complaint);
    };
    (
        ResolvedCap {
            bytes: mb.saturating_mul(1024 * 1024),
            basis: CapBasis::Env,
        },
        complaint,
    )
}

/// Override the ANCHOR retention's byte ceiling, in mebibytes.
///
/// # Why there is no `DEFAULT_FLASHBACK_ANCHOR_MAX_MB`
///
/// A CONST equal to the window's would mean the
/// equality the plane is priced on ("one standing cost, priced once") holds
/// only while nobody touches either number. Raising the window env would move the
/// window and leave the anchor at 320 MiB, so the plane an operator thought they
/// had doubled would not have: a silent divergence.
///
/// The equality is REAL and at RUNTIME instead: the anchor's default is the
/// RESOLVED window (see [`resolve_anchor_max_bytes`]), so an env-overridden window
/// DRAGS the anchor with it and a derived window drags it too. A const cannot
/// express that, so there is none kept as a fallback nothing
/// resolves through — one would be a second, quieter answer to the same
/// question.
pub const FLASHBACK_ANCHOR_MAX_MB_ENV: &str = "CERULION_FLASHBACK_ANCHOR_MAX_MB";

/// PURE: the anchor retention's byte ceiling — [`FLASHBACK_ANCHOR_MAX_MB_ENV`] over
/// the RESOLVED window.
///
/// # The drag, and the one thing that stops it
///
/// `window_bytes` is the window cap **after** its own env and derivation, so the
/// coupling is transitive by construction: raise the window and the anchor follows;
/// let the machine size the window and the anchor is sized by the same machine.
/// That is what makes "the plane is 2× the window" a fact rather than a comment.
///
/// The ONE thing that stops it is the anchor's OWN env being set explicitly, which
/// wins outright — the held-fixed rule this whole design keeps: every existing knob
/// keeps working, and an operator who states a number gets that number.
///
/// # What it BUYS, and what it costs when it bites
///
/// A checkpoint is bounded only by the arm-time state gate (~673 MiB of private
/// anon PER RANK), so a big-state robot's checkpoints genuinely may not fit. Past
/// this ceiling the oldest checkpoints go.
///
/// **What that costs is fixed by one rule.** Such a robot does NOT degrade to a
/// frames-only Flashback — still a black box, not a resumable
/// one. The rule: "STRICTLY NEVER FRAMES-ONLY. Every Flashback
/// capture must be resimmable." A plane that cannot hold one whole generation
/// raises a standing alarm and REFUSES to finalize a capture, rather than writing
/// a dashcam clip wearing the Flashback name. This ceiling is therefore a
/// STARTING point, not the operative bound: once a generation has been measured,
/// [`split_plane_budget`] re-splits the plane anchor-first and the reserve is
/// sized from the demand.
///
/// Scaling the window scales this too, which is precisely why the coupling is
/// worth keeping: on a big machine the anchor side stops being the binding limit
/// at the same rate the frame side does.
///
/// SATURATING on the multiply, for the reason every other byte knob here is: an
/// astronomical ask means "no practical ceiling", which is what an operator typing
/// one means.
pub fn resolve_anchor_max_bytes(
    raw: Option<&str>,
    window_bytes: u64,
) -> (ResolvedCap, Option<String>) {
    let (value, complaint) =
        resolve_positive_override(raw, &MebibyteCeiling(window_bytes), "mebibytes");
    let Some(mb) = value else {
        // The DRAG: no explicit anchor env, so the anchor IS the window.
        return (
            ResolvedCap {
                bytes: window_bytes,
                basis: CapBasis::Window,
            },
            complaint,
        );
    };
    (
        ResolvedCap {
            bytes: mb.saturating_mul(1024 * 1024),
            basis: CapBasis::Env,
        },
        complaint,
    )
}

/// The small frame budget the anchor reserve may never take ("the frame window
/// gets the remainder above a small floor").
///
/// 64 MiB — the house number, and deliberately modest: its job is to stop a
/// giant-state robot's reserve from collapsing the window to nothing, not to
/// defend a useful span. A robot whose state is that big pays for it in WINDOW
/// SECONDS, which is the visible, smooth axis the cost is meant to land
/// on, and this floor is the point past which "fewer seconds" would become "no
/// black box at all".
///
/// Deliberately NOT its own env knob. The operator-facing axes are already the
/// window cap, the window span and the topic-exclude list, and a fourth knob
/// whose only effect is to trade the same bytes between the same two consumers
/// would be a way to reach the same states with more ways to get them wrong.
pub const DEFAULT_FLASHBACK_FRAMES_FLOOR_MB: u64 = 64;

/// [`DEFAULT_FLASHBACK_FRAMES_FLOOR_MB`] in bytes.
pub const FLASHBACK_FRAMES_FLOOR_BYTES: u64 = DEFAULT_FLASHBACK_FRAMES_FLOOR_MB * 1024 * 1024;

/// How many checkpoint generations the anchor reserve asks for when they fit.
///
/// Three by design, and the ladder degrades 3 → 2 → 1 before the
/// frames give anything, which is what "anchor-first" means.
pub const FLASHBACK_ANCHOR_GENERATIONS_TARGET: u8 = 3;

const _: () = assert!(
    FLASHBACK_ANCHOR_GENERATIONS_TARGET >= 1,
    "a target below one generation would make the alarm arm on every robot"
);
const _: () = assert!(
    FLASHBACK_FRAMES_FLOOR_BYTES < FLASHBACK_WINDOW_FLOOR_BYTES,
    "the frames floor must fit inside the smallest window this build will ever \
     derive, or the split would refuse a machine the window cap already accepted"
);

/// What the plane's budget was split into, and on what evidence.
///
/// An ENUM rather than a bare `generations: u8`, because zero would have to mean
/// two different things: "no generation fits" (the standing alarm) and "nothing
/// has been measured yet" (an ordinary robot in its first cadence). That is the
/// absence-as-sentinel trap `resolve_positive_override` already refuses one
/// screen up, and here the two states have OPPOSITE operator meanings: one is a
/// standing alarm, the other is silence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneSplitVerdict {
    /// No complete generation has been measured yet, so the default caps
    /// stand unchanged.
    ///
    /// This is every robot's state until its first checkpoint completes, and it
    /// is deliberately the DEFAULT split rather than a guess: the reserve is
    /// sized from MEASURED state, and inventing a demand before anything has
    /// been measured would be fabricated data.
    Unmeasured,
    /// `generations` complete generations are reserved, and the frames keep the
    /// remainder above the floor.
    Reserved {
        /// 1, 2 or 3 — never 0; see the enum docs.
        generations: u8,
    },
    /// Not even ONE generation fits beside the frames floor.
    ///
    /// The standing-alarm condition, and the state in which a capture that
    /// cannot resim is REFUSED rather than written as a dashcam clip wearing the
    /// Flashback name.
    BelowOneGeneration {
        /// How much bigger the plane would have to be to hold one generation
        /// plus the frames floor. What the remedy is computed from.
        shortfall_bytes: u64,
    },
}

/// The plane's byte budget, split anchor-first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlaneSplit {
    /// The anchor retention's ceiling.
    pub anchor_bytes: u64,
    /// The frame window's ceiling.
    pub frames_bytes: u64,
    /// What that split is, and on what evidence.
    pub verdict: PlaneSplitVerdict,
}

impl PlaneSplit {
    /// Is this the state the plane raises a standing alarm for?
    pub fn below_one_generation(&self) -> bool {
        matches!(self.verdict, PlaneSplitVerdict::BelowOneGeneration { .. })
    }
}

/// PURE: split the plane's budget anchor-first.
///
/// # Anchor first, and why it is the whole decision
///
/// An anchor cap DERIVED from the window's, with the two left to
/// bite independently, means that on a robot whose state does not fit, the anchor
/// retention simply drops checkpoints and the capture degrades to frames-only.
/// The anchor-first rule reverses the priority: the anchor reserves what measured state
/// demands (three generations when they fit, degrading 3 → 2 → 1 before the
/// frames give anything), and the frame window gets the remainder above
/// [`FLASHBACK_FRAMES_FLOOR_BYTES`]. The cost of big state therefore lands on
/// WINDOW SECONDS — a visible, smooth, per-robot axis the spawn projection and
/// the achieved-span surfaces already report, and one an operator buys back with
/// the duration knobs or by excluding camera topics.
///
/// # The plane is the SUM, and an explicit anchor env still wins
///
/// `plane = window + anchor`, which under the window's drag is 2× the window. The
/// reserve is capped by `anchor_cap` when its basis is [`CapBasis::Env`]: an
/// operator who states a number gets that number, which is the held-fixed rule
/// every knob in this module keeps. When the anchor cap is DERIVED, the reserve
/// may take as much of the plane as the ladder asks for.
///
/// # Arithmetic at the measured points
///
/// A 16 GB NX gives a plane of ~1.9 GiB (2 × the 961 MiB derived window). Against
/// a 1.2 GiB generation that is ONE generation plus ~700 MiB of frames — which is
/// exactly what that machine is sized to hold. Three generations are
/// AGX-class. A real attach-class robot (a Go2: kilobytes of state) reserves
/// three generations out of the noise and pays nothing.
pub fn split_plane_budget(
    window_bytes: u64,
    anchor_cap: ResolvedCap,
    measured_generation_bytes: Option<u64>,
    frames_floor_bytes: u64,
) -> PlaneSplit {
    let plane = window_bytes.saturating_add(anchor_cap.bytes);

    // A measurement of ZERO is not a measurement: a complete generation always
    // carries at least one record, so this is a caller that has nothing to say.
    // Treated as unmeasured rather than as "everything fits", which would report
    // three reserved generations of nothing.
    let Some(generation) = measured_generation_bytes.filter(|g| *g > 0) else {
        return PlaneSplit {
            anchor_bytes: anchor_cap.bytes,
            frames_bytes: window_bytes,
            verdict: PlaneSplitVerdict::Unmeasured,
        };
    };

    // An EXPLICIT anchor env is a ceiling on the reserve; a derived one is not.
    let reserve_ceiling = match anchor_cap.basis {
        CapBasis::Env => anchor_cap.bytes,
        _ => plane,
    };

    let mut n = FLASHBACK_ANCHOR_GENERATIONS_TARGET;
    while n >= 1 {
        let reserve = generation.saturating_mul(u64::from(n));
        if reserve <= reserve_ceiling && reserve.saturating_add(frames_floor_bytes) <= plane {
            return PlaneSplit {
                anchor_bytes: reserve,
                frames_bytes: plane - reserve,
                verdict: PlaneSplitVerdict::Reserved { generations: n },
            };
        }
        n -= 1;
    }

    // Below one generation. The plane KEEPS RUNNING and holds what it can
    // by design; the alarm and the capture refusal are what make that
    // visible to the operator, not a wider reserve nobody can afford.
    let anchor_bytes = reserve_ceiling.min(plane.saturating_sub(frames_floor_bytes));
    PlaneSplit {
        anchor_bytes,
        frames_bytes: plane - anchor_bytes,
        verdict: PlaneSplitVerdict::BelowOneGeneration {
            shortfall_bytes: generation
                .saturating_add(frames_floor_bytes)
                .saturating_sub(plane),
        },
    }
}

/// The knob an operator must raise to restore ONE generation, and to what.
///
/// A STRUCTURED answer rather than a sentence, because the alarm has to print an
/// exact value an operator can paste and a test has to be able to check the
/// arithmetic without matching prose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GenerationRemedy {
    /// The env var to set — [`FLASHBACK_ANCHOR_MAX_MB_ENV`] when the anchor cap
    /// is already explicit, [`FLASHBACK_WINDOW_MAX_MB_ENV`] otherwise.
    pub knob: &'static str,
    /// The value to set it to, in whole mebibytes, rounded UP — a value rounded
    /// down would be a remedy that does not work.
    ///
    /// `None` when NOTHING has been observed about this robot's state: the knob
    /// is still named, with no number. **A zero is never rendered** — see the
    /// function docs.
    pub mebibytes: Option<u64>,
    /// What that number is evidence of.
    pub basis: RemedyBasis,
}

/// What a [`GenerationRemedy`]'s value was derived from.
///
/// Carried rather than inferred from `mebibytes.is_some()`, because the two
/// present cases mean different things to an operator: one is a measurement of a
/// whole generation, the other a FLOOR from an anchor that never completed, and
/// a remedy that presented a floor as a measurement would under-state the ask on
/// exactly the robot least able to absorb a second wrong answer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RemedyBasis {
    /// A whole generation was measured. The value restores one.
    Measured,
    /// No generation ever completed; the value is a FLOOR read off an anchor the
    /// ceiling refused — the plane needs AT LEAST this much.
    PartialAnchorFloor,
    /// Nothing was observed at all. The knob is named with no number.
    Unknown,
}

/// PURE: what would restore one generation, given the measurement that did not fit.
///
/// # Which knob, and why it depends on the basis
///
/// When the anchor cap is EXPLICIT, that number is the binding ceiling and the
/// operator already chose it — so the remedy is to raise THAT, to one whole
/// generation. When it is DERIVED, the anchor is dragged by the window
/// (`plane = 2 × window`), so the knob that moves the plane is the WINDOW's, and
/// the value must satisfy `2W >= generation + floor`, i.e. `W >= (G + floor) / 2`.
///
/// Naming the wrong knob is the failure this exists to prevent: it is the
/// per-verb rule applied to an env var — a remedy that names a knob the
/// reaching path does not honour prints an instruction that does nothing.
///
/// # A zero is never rendered
///
/// Deriving the value from `measured_generation_bytes.unwrap_or(0)` is wrong,
/// because the plane is REACHABLE with nothing measured: the harvester's in-flight
/// ceiling can refuse every anchor before ONE ever completes, which is precisely
/// the shape a too-small machine is in. That would print
/// `CERULION_FLASHBACK_ANCHOR_MAX_MB=0` — a confidently WRONG knob value, on the
/// one robot that needs the right one, in the one message whose entire content
/// is the number.
///
/// So the demand is taken from the best evidence available, in order:
///
/// 1. a MEASURED whole generation ⇒ [`RemedyBasis::Measured`];
/// 2. else the largest anchor-shaped thing the ceiling REFUSED — an anchor that
///    had buffered N bytes is at least N bytes ⇒
///    [`RemedyBasis::PartialAnchorFloor`], rendered as "at least N MiB";
/// 3. else nothing is known ⇒ [`RemedyBasis::Unknown`], `mebibytes: None`, and
///    the knob is named WITHOUT a number.
///
/// A zero in either input is NOT a measurement (a complete generation carries at
/// least one record; a refusal that held nothing of its own reports zero bytes),
/// so both are filtered rather than believed — which is what makes arm 3
/// reachable and case (a) unreachable.
pub fn generation_remedy(
    measured_generation_bytes: Option<u64>,
    refused_bytes_floor: Option<u64>,
    frames_floor_bytes: u64,
    anchor_cap: ResolvedCap,
) -> GenerationRemedy {
    const MIB: u64 = 1024 * 1024;
    let knob = match anchor_cap.basis {
        CapBasis::Env => FLASHBACK_ANCHOR_MAX_MB_ENV,
        _ => FLASHBACK_WINDOW_MAX_MB_ENV,
    };
    let (demand, basis) = match (
        measured_generation_bytes.filter(|g| *g > 0),
        refused_bytes_floor.filter(|b| *b > 0),
    ) {
        (Some(g), _) => (g, RemedyBasis::Measured),
        (None, Some(b)) => (b, RemedyBasis::PartialAnchorFloor),
        (None, None) => {
            return GenerationRemedy {
                knob,
                mebibytes: None,
                basis: RemedyBasis::Unknown,
            }
        }
    };
    // `demand > 0` here by construction, so `div_ceil` cannot yield 0 and the
    // rendered value is always a number that means something.
    let mebibytes = match anchor_cap.basis {
        CapBasis::Env => demand.div_ceil(MIB),
        // ceil((G + floor) / 2) in bytes, then ceil to whole MiB — rounding up
        // at BOTH steps, because either rounding down yields a value that still
        // does not fit.
        _ => demand
            .saturating_add(frames_floor_bytes)
            .div_ceil(2)
            .div_ceil(MIB),
    };
    GenerationRemedy {
        knob,
        mebibytes: Some(mebibytes),
        basis,
    }
}

/// One produced topic's DECLARED cost, as the spawn-time projection reads it.
///
/// DECLARED is the operative word and the reason this is a report rather than a
/// policy input: both numbers come from what a graph SAYS, and they were measured
/// erring in BOTH directions inside one real graph — `max_slice_len` 23×
/// over on a `JointState` and 128× under on a right-camera chain. Sizing a cap
/// from that is unsound; telling an operator what their own
/// declarations imply, and letting them compare it with reality, is sound.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeclaredTopicRate {
    /// The declared slot size — header plus payload — in bytes.
    pub bytes_per_frame: u64,
    /// The declared publish rate in MILLIHERTZ, matching
    /// [`crate::graph::PartitionCosts::edge_rate_mhz`]'s unit so no conversion
    /// happens at the call site.
    pub millihertz: u64,
}

/// PURE: what a set of declared topics offers the window, in bytes per second.
///
/// # The millihertz scale is divided out ONCE, at the END
///
/// Dividing per topic TRUNCATES each term, and the truncation compounds in the
/// one direction that matters: N topics each offering under a byte per second
/// contribute nothing at all, so a fleet of sub-unit publishers sums to 0 — which
/// the projection reads as "reach unknown / this cap holds forever" rather than as
/// the real, finite offer. Two 1-byte 500 mHz topics really do offer one byte a
/// second, and a per-topic fold reports zero. Overstating the reach is the
/// failure mode this reporting exists to remove, so the products are
/// aggregated first and the scale removed from the total.
///
/// Saturating throughout, and the direction is deliberate: an absurd declaration
/// reports an absurd rate — which renders as "this cap holds ~0 s" and sends the
/// operator to look at the declaration — rather than wrapping into a small,
/// plausible, wrong number. With the divide moved to the end, the saturation
/// CEILING is `u64::MAX / 1000` rather than `u64::MAX`: the accumulator pins at
/// `u64::MAX` and is then scaled. Still absurd, still in the safe direction — a
/// rate that large renders as a reach of ~0 s either way.
pub fn offered_bytes_per_second(topics: &[DeclaredTopicRate]) -> u64 {
    topics.iter().fold(0u64, |acc, t| {
        acc.saturating_add(t.bytes_per_frame.saturating_mul(t.millihertz))
    }) / 1_000
}

/// What a cap buys at a given offered rate — the spawn-time projection line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WindowReach {
    /// The offered aggregate that produced it.
    pub offered_bytes_per_second: u64,
    /// How far back the cap reaches at that rate, in milliseconds.
    ///
    /// `None` when nothing is offered: a cap divided by a zero rate reaches
    /// forever, and "forever" is not a projection — it is the absence of one. An
    /// arm that prints `∞` would be read as a promise.
    pub reach_ms: Option<u64>,
    /// The span the window PROMISES, for the comparison the line exists to make.
    pub span_ms: u64,
}

impl WindowReach {
    /// Whether the cap covers the promised span.
    ///
    /// An unknown reach is NOT "holds" — the projection made no claim, and reading
    /// silence as success is the failure mode this reporting exists to
    /// remove.
    pub fn holds_full_span(&self) -> bool {
        self.reach_ms.is_some_and(|reach| reach >= self.span_ms)
    }
}

/// PURE: the spawn-time projection — "declared topics project ~X MB/s; this cap
/// holds ~Y s of the N s window".
///
/// The arithmetic is deliberately trivial (`cap / rate`) because the ESTIMATE is
/// not in the arithmetic, it is in the inputs — see [`DeclaredTopicRate`]. Keeping
/// it here rather than at the call site means the one place that decides what a
/// cap buys is the same place that decides what the cap is.
pub fn project_window_reach(
    cap_bytes: u64,
    offered_bytes_per_second: u64,
    span_ms: u64,
) -> WindowReach {
    WindowReach {
        offered_bytes_per_second,
        reach_ms: (offered_bytes_per_second > 0)
            .then(|| cap_bytes.saturating_mul(1_000) / offered_bytes_per_second),
        span_ms,
    }
}

/// The hard BYTE ceiling on the rolling SCHEDULER-TRACE retention, in mebibytes.
///
/// # Why it is a literal rather than derived from the frame window's
///
/// The anchor retention derives its ceiling from the frame window's because the
/// two are one standing cost an operator was quoted once. This one is a
/// DIFFERENT order of magnitude and would be misleading folded into that number:
/// a trace record is 40 bytes and one is pushed per node fire plus one per step,
/// so a 1 kHz graph with 10 firing nodes produces ~11 000 records/s ≈ 440 KB/s,
/// i.e. **~13 MB** across the shipped 30 s window — under 5 % of the frame
/// window's 320 MiB ceiling.
///
/// 64 MiB is therefore ~5× the priced figure: generous enough that an ordinary
/// robot never reaches it, small enough that it is still a BACKSTOP rather than
/// a licence. Past it the oldest records go, exactly as the frame window's does,
/// and a capture that loses its own resume boundary that way reports itself
/// NOT resimmable rather than shipping a trace that begins in the middle of a
/// step.
pub const DEFAULT_FLASHBACK_TRACE_MAX_MB: u64 = 64;

/// Topics the Flashback window does not hold.
///
/// Comma-separated canonical topic names, each either exact or a trailing-`*`
/// prefix. The lever for buying window seconds back on a robot
/// whose state is big: trimming camera topics buys seconds directly, and it is
/// the one axis that works when the duration knobs have already been spent.
///
/// # It governs the WINDOW, never a recording
///
/// An explicit `cerulion bag record` (or `graph run --record`) still records
/// every topic it was asked to. The operator asking for a recording of a camera
/// topic and the operator excluding it from the always-on black box are making
/// two different decisions, and collapsing them would let a memory knob silently
/// shrink a recording somebody asked for.
pub const FLASHBACK_EXCLUDE_TOPICS_ENV: &str = "CERULION_FLASHBACK_EXCLUDE_TOPICS";

/// A parsed [`FLASHBACK_EXCLUDE_TOPICS_ENV`] list.
///
/// Ordered and de-duplicated at parse time, so the set a status feed renders and
/// the set a capture manifest records are the same list in the same order on
/// every run — a set that rendered in hash order would make two identical robots
/// look different.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ExcludeTopics {
    /// Exact canonical names.
    exact: Vec<String>,
    /// Prefixes, each written `foo*` and stored WITHOUT the star.
    prefixes: Vec<String>,
}

impl ExcludeTopics {
    /// Is this topic excluded from the window?
    pub fn excludes(&self, topic: &str) -> bool {
        self.exact.iter().any(|e| e == topic)
            || self.prefixes.iter().any(|p| topic.starts_with(p.as_str()))
    }

    /// Nothing is excluded — the ordinary robot.
    pub fn is_empty(&self) -> bool {
        self.exact.is_empty() && self.prefixes.is_empty()
    }

    /// How many patterns it carries.
    pub fn len(&self) -> usize {
        self.exact.len() + self.prefixes.len()
    }

    /// Every pattern as written, exact ones first then prefixes (with their
    /// stars restored) — what a status feed and a capture manifest render.
    pub fn patterns(&self) -> Vec<String> {
        let mut out = self.exact.clone();
        out.extend(self.prefixes.iter().map(|p| format!("{p}*")));
        out
    }
}

/// PURE: parse [`FLASHBACK_EXCLUDE_TOPICS_ENV`].
///
/// # What is refused, and why each refusal is LOUD rather than silent
///
/// Every rejected entry is DROPPED and named in the complaint, because the
/// failure mode of a silently-misparsed exclude list is the opposite of obvious:
/// the operator believes a heavy topic is out of the window, the window keeps
/// holding it, and the symptom is a span that will not grow for a reason nothing
/// reports. The entries refused are:
///
/// * A bare `*`. It excludes EVERY topic, which is not a memory knob — it turns
///   the black box off — and there is already a switch that says so
///   ([`FLASHBACK_ENV`]). Accepting it would let a robot ship with Flashback
///   silently disabled by something that reads like a tuning parameter.
/// * A `*` anywhere but the END. Only a trailing star is a prefix; an interior
///   one is a glob this does not implement, and matching it literally would
///   silently exclude nothing.
/// * An empty entry, which a trailing comma produces. Dropped WITHOUT a
///   complaint — `a,b,` is an ordinary way to write a list and treating it as an
///   error would be noise.
///
/// Duplicates are folded silently: stating a topic twice is not a mistake worth
/// a line.
pub fn parse_exclude_topics(raw: Option<&str>) -> (ExcludeTopics, Option<String>) {
    let mut out = ExcludeTopics::default();
    let mut refused: Vec<String> = Vec::new();
    for entry in raw.unwrap_or("").split(',') {
        let entry = entry.trim();
        if entry.is_empty() {
            continue;
        }
        if entry == "*" {
            refused.push(format!(
                "{entry:?} would exclude every topic — set {FLASHBACK_ENV}=off to turn the \
                 plane off instead"
            ));
            continue;
        }
        match entry.strip_suffix('*') {
            Some(prefix) if !prefix.contains('*') => {
                let prefix = prefix.to_string();
                if !out.prefixes.contains(&prefix) {
                    out.prefixes.push(prefix);
                }
            }
            // A star that is not the last character, or several stars.
            _ if entry.contains('*') => refused.push(format!(
                "{entry:?} is not a pattern this knob understands — only a TRAILING \
                 '*' is a prefix"
            )),
            _ => {
                let exact = entry.to_string();
                if !out.exact.contains(&exact) {
                    out.exact.push(exact);
                }
            }
        }
    }
    let complaint = (!refused.is_empty()).then(|| {
        format!(
            "{FLASHBACK_EXCLUDE_TOPICS_ENV}: ignoring {} entr{} — {}",
            refused.len(),
            if refused.len() == 1 { "y" } else { "ies" },
            refused.join("; ")
        )
    });
    (out, complaint)
}

/// Override [`DEFAULT_FLASHBACK_TRACE_MAX_MB`].
pub const FLASHBACK_TRACE_MAX_MB_ENV: &str = "CERULION_FLASHBACK_TRACE_MAX_MB";

/// PURE: the trace retention's byte ceiling, from
/// [`FLASHBACK_TRACE_MAX_MB_ENV`].
///
/// SATURATING on the multiply, for the reason every other byte knob here is: an
/// astronomical ask means "no practical ceiling", which is what an operator
/// typing one means.
pub fn resolve_trace_max_bytes(raw: Option<&str>) -> (u64, Option<String>) {
    let (value, complaint) =
        resolve_positive_override(raw, &DEFAULT_FLASHBACK_TRACE_MAX_MB, "mebibytes");
    let bytes = value
        .unwrap_or(DEFAULT_FLASHBACK_TRACE_MAX_MB)
        .saturating_mul(1024 * 1024);
    (bytes, complaint)
}

/// The per-topic byte budget a
/// window-only recorder's tap queue is sized to, in mebibytes.
///
/// # 64 MiB — one number, two rules
///
/// It is the SAME figure already shipped as the per-route ingress
/// budget (`transport::ingress_route_buffer_depth`), and deliberately so: both
/// answer "how much SHM may ONE topic's queue be worth?", and a robot whose
/// operator has internalised one number should not discover a second one with a
/// different value doing the same job. What differs is the DIVISOR — the ingress
/// rule prices the nominal slice (an APPARENT-reservation budget), while this one
/// prices the REAL slot layout, because what it bounds is RESIDENT pinning. See
/// `transport::flashback_tap_buffer_depth`.
///
/// # Why it is a knob at all
///
/// The held-fixed rule says an explicit env
/// wins. The budget is a property of the MACHINE the window stands on, not of
/// Cerulion: a desk with 128 GB can afford a deeper standing tap than a Jetson,
/// and an operator who has decided that should not have to choose between the
/// shipped number and turning Flashback off entirely.
pub const DEFAULT_FLASHBACK_TAP_BUDGET_MB: u64 = 64;

/// Override [`DEFAULT_FLASHBACK_TAP_BUDGET_MB`].
pub const FLASHBACK_TAP_BUDGET_MB_ENV: &str = "CERULION_FLASHBACK_TAP_BUDGET_MB";

/// PURE: the per-topic window-only tap budget in BYTES, from
/// [`FLASHBACK_TAP_BUDGET_MB_ENV`].
///
/// Resolved through the SAME parser and reported through the same complaint seam
/// as every other Flashback knob — so a `0`, an unparseable value and an absent
/// one behave here exactly as they do for the window span and the byte ceilings,
/// and the absence-in-the-type rule is paid for once rather than re-derived.
///
/// SATURATING on the multiply, for the reason every byte knob here is: an
/// astronomical ask means "no practical ceiling", and the depth rule then simply
/// lands on the topic's own service ceiling — which is the unbudgeted
/// behaviour, i.e. exactly what an operator asking for no budget means.
pub fn resolve_tap_budget_bytes(raw: Option<&str>) -> (u64, Option<String>) {
    let (value, complaint) =
        resolve_positive_override(raw, &DEFAULT_FLASHBACK_TAP_BUDGET_MB, "mebibytes");
    let bytes = value
        .unwrap_or(DEFAULT_FLASHBACK_TAP_BUDGET_MB)
        .saturating_mul(1024 * 1024);
    (bytes, complaint)
}

/// Override the per-rank STATE RING's size, in MEBIBYTES.
///
/// The state ring's sizing knob. It is
/// a knob rather than a constant because the right size is a property of the
/// ROBOT: the ring must hold the anchors a capture's pre-window needs
/// (`window / cadence + 1` of them) and an anchor is bounded only by the
/// arm-time gate, so a big-state robot legitimately needs more than the shipped
/// 64 MiB while a tiny one is paying for space it can never fill. An operator
/// who raises [`FLASHBACK_MAX_STATE_MB_ENV`] almost always has to raise this
/// with it.
pub const FLASHBACK_STATE_RING_MB_ENV: &str = "CERULION_FLASHBACK_STATE_RING_MB";

/// The shipped per-rank ring size in MEBIBYTES — the same quantity
/// [`crate::state_ring::DEFAULT_STATE_RING_BYTES`] names, expressed in the unit
/// the knob is typed in.
///
/// DERIVED from that constant rather than written as `64`, so the two cannot
/// drift into an operator reading one number and getting another.
pub const DEFAULT_FLASHBACK_STATE_RING_MB: u64 =
    (crate::state_ring::DEFAULT_STATE_RING_BYTES / (1024 * 1024)) as u64;

/// PURE: the per-rank state ring's size in BYTES, from
/// [`FLASHBACK_STATE_RING_MB_ENV`].
///
/// Saturating on the multiply for the same reason every other byte knob here is:
/// an astronomical ask means "no practical ceiling", and the ring creation
/// refuses what it cannot allocate LOUDLY rather than silently shrinking.
pub fn resolve_state_ring_bytes(raw: Option<&str>) -> (u64, Option<String>) {
    let (value, complaint) =
        resolve_positive_override(raw, &DEFAULT_FLASHBACK_STATE_RING_MB, "mebibytes");
    let bytes = value
        .unwrap_or(DEFAULT_FLASHBACK_STATE_RING_MB)
        .saturating_mul(1024 * 1024);
    (bytes, complaint)
}

/// PURE: the anchor cadence in MILLISECONDS, from [`FLASHBACK_CADENCE_MS_ENV`].
pub fn resolve_cadence_ms(raw: Option<&str>) -> (u64, Option<String>) {
    // The cadence default IS a whole number of its own unit, so it renders as
    // itself — unlike the state ceiling below.
    let (value, complaint) =
        resolve_positive_override(raw, &DEFAULT_FLASHBACK_CADENCE_MS, "milliseconds");
    (value.unwrap_or(DEFAULT_FLASHBACK_CADENCE_MS), complaint)
}

/// Renders a BYTE ceiling as the operator-facing figure it really is: fractional
/// mebibytes AND the exact byte count.
///
/// Every default in this module is a DERIVATION — the state ceiling from a stall
/// budget, the window cap from effective RAM — and none of them is a whole number
/// of the unit its knob is typed in. Quoting one as whole mebibytes told an
/// operator "using the default 673" while 673.68 MiB applied, so an operator who
/// restated what they were just told would have TIGHTENED the ceiling by 717 446 B.
///
/// ONE type for every such default (generalised from the state
/// ceiling's own), because a second copy of this rendering is a second place for
/// that defect to come back. It borrows the value rather than computing it so the
/// complaint path allocates nothing on the ordinary (no-complaint) call.
pub(crate) struct MebibyteCeiling(pub(crate) u64);

impl std::fmt::Display for MebibyteCeiling {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        const MIB: u64 = 1024 * 1024;
        let bytes = self.0;
        // Two decimal places, in integer arithmetic: the point of this type is that
        // the figure is NOT a whole number of mebibytes, so truncating it back to
        // one would reintroduce the very misreport it exists to fix. The exact byte
        // count rides alongside, because 673.68 is still a rounding and the bytes
        // are the number the gate actually compares against.
        write!(
            f,
            "{}.{:02} mebibytes ({bytes} bytes)",
            bytes / MIB,
            (bytes % MIB) * 100 / MIB
        )
    }
}

/// [`MebibyteCeiling`] over [`default_max_state_bytes`] — the arm-time state
/// ceiling's own rendering, kept as a named type because that is the default the
/// misreport above was found on.
struct DefaultStateCeiling;

impl std::fmt::Display for DefaultStateCeiling {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        MebibyteCeiling(default_max_state_bytes()).fmt(f)
    }
}

/// PURE: the arm-time state ceiling in BYTES, from [`FLASHBACK_MAX_STATE_MB_ENV`]
/// (which is stated in mebibytes).
///
/// # The DEFAULT is not round-tripped through mebibytes
///
/// [`default_max_state_bytes`] is a byte count DERIVED from two constants, and it
/// is not a whole number of MiB — at the shipped values it is 706 409 094 B,
/// i.e. 673.68 MiB. Converting it to whole MiB and back would shave 717 446 B
/// (0.68 MiB) off the ceiling, and every state in the resulting band projects a
/// 249 ms stall — INSIDE the 250 ms budget — while being refused. The band is
/// narrow but the refusal it causes is total (that robot gets no state capture at
/// all), so the default is passed through untouched and only an OPERATOR'S
/// override, which really is stated in mebibytes, is scaled.
///
/// The unit split is deliberate rather than an inconsistency: the knob is a number
/// a human types, and MiB is the unit they think in; the default is a derivation,
/// and rounding a derivation to a friendlier unit changes the answer.
pub fn resolve_max_state_bytes(raw: Option<&str>) -> (u64, Option<String>) {
    // ABSENCE arrives as `None`, never as a magic number — see
    // `resolve_positive_override`. A `u64::MAX` sentinel would make the largest
    // value an operator can type mean its own opposite.
    //
    // The complaint renders the FULL-PRECISION default (`DefaultStateCeiling`), not
    // a whole-MiB copy of it: the fallback below returns every one of those bytes,
    // so quoting a rounded figure would describe a ceiling that is not the one in
    // force — and it would be rounded DOWN, i.e. an operator restating what they
    // were just told would tighten it.
    let (mb, complaint) = resolve_positive_override(raw, &DefaultStateCeiling, "mebibytes");
    let Some(mb) = mb else {
        // No usable override — INCLUDING the degraded cases (a `0`, an unparseable
        // value), which must land on the same full-precision default as an absent
        // one rather than on a quietly-shaved copy of it.
        return (default_max_state_bytes(), complaint);
    };
    // SATURATING, and that is the correct reading of an enormous ask: a ceiling of
    // `u64::MAX` bytes is "no practical ceiling", which is exactly what an operator
    // typing an astronomical number means. The headroom arm of the gate still
    // applies, so this disables the SIZE budget rather than the safety.
    (mb.saturating_mul(1024 * 1024), complaint)
}

/// PURE: the cadence in STEPS — `cadence_ms / gating_quantum`, rounded UP
/// and floored at 1.
///
/// STEPS, never a wall duration, and that is not stylistic: macOS background-QoS
/// timer coalescing charges a nominal 150 ms as 1100–1696 ms, so a
/// wall-timed cadence would be unreliable on the dev platform and non-deterministic
/// everywhere. A step count derived ONCE at arm time is the same on every replay of
/// the same graph, which is what keeps Principle #7 intact.
///
/// Rounded UP so a cadence shorter than one step still means "every step" rather
/// than "every step, twice"; floored at 1 so a zero or absurd quantum cannot make
/// the modulo in [`crate::state_arm::cadence_due`] divide by zero or anchor on
/// every boundary forever.
pub fn cadence_steps(cadence_ms: u64, quantum_ns: u64) -> u64 {
    if quantum_ns == 0 {
        return 1;
    }
    let cadence_ns = cadence_ms.saturating_mul(1_000_000);
    cadence_ns.div_ceil(quantum_ns).max(1)
}

/// PURE: the first-tick stall `state_bytes` of private anon projects to.
///
/// Saturating, so an absurd input reports an absurd stall rather than wrapping into
/// a small one — the single arithmetic slip here would ADMIT the one process this
/// gate exists to refuse.
pub fn projected_stall_ms(state_bytes: u64) -> u64 {
    state_bytes
        .saturating_mul(FLASHBACK_STALL_MS_PER_GIB)
        .saturating_div(BYTES_PER_GIB)
}

/// Everything the arm-time report states, priced.
///
/// Carried as a struct rather than passed as four arguments because it is also the
/// LOG LINE: an operator who is refused must be told the number, the threshold and
/// the override in one place, and one that is admitted should be able to see what
/// the plane is going to cost before it costs it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmProjection {
    /// This process's resident anonymous memory — the sound upper bound on what a
    /// capture child can cost (see [`crate::state_carrier::fork::sample_anon_rss`]).
    /// `None` where the platform cannot say.
    pub state_bytes: Option<u64>,
    /// What the kernel believes it can hand out. `None` where the platform cannot
    /// say.
    pub mem_available: Option<u64>,
    /// The ceiling in force, after [`FLASHBACK_MAX_STATE_MB_ENV`].
    pub max_state_bytes: u64,
    /// The SHM the per-rank state ring reserves. Reported, never gated on — see the
    /// module docs.
    pub ring_bytes: u64,
    /// The cadence this plane would anchor at, in steps — what the arm word
    /// actually carries.
    pub cadence_steps: u64,
    /// The same cadence in MILLISECONDS of logical time: what the operator either
    /// set or inherited as the default.
    ///
    /// Carried ALONGSIDE the step count rather than derived from it, because the
    /// two answer different questions and neither substitutes: an operator checking
    /// whether their `CERULION_FLASHBACK_CADENCE_MS` took effect reads this one,
    /// while the step count is the number the boundary compares against and the
    /// only one that means anything on a graph whose quantum is unusual.
    pub cadence_ms: u64,
}

impl ArmProjection {
    /// The projected first-tick stall, or `None` when the state size is unknown.
    pub fn stall_ms(&self) -> Option<u64> {
        self.state_bytes.map(projected_stall_ms)
    }
}

/// Why an arm-time projection refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmRefusal {
    /// The state is bigger than the ceiling, so every anchor would stall a node
    /// thread for longer than [`FLASHBACK_STALL_BUDGET_MS`].
    StateTooLarge,
    /// There is not enough memory for the FIRST anchor, so the per-anchor gate
    /// would decline every one of them anyway.
    ///
    /// Refusing here rather than letting that happen is the difference between a
    /// robot that is TOLD its black box is off and one that quietly reserves a ring,
    /// runs the whole plane, and never captures anything.
    NoHeadroom,
}

/// PURE: the arm-time gate. `None` = arm.
///
/// # `None` inputs ARM, and that is the same decision the per-anchor gate makes
///
/// A platform that cannot report its own memory would otherwise never arm at all —
/// a cost-derived refusal by another name, and on the one platform (macOS) where
/// the numbers are hardest to read. Both reads fail OPEN, exactly as
/// [`crate::state_carrier::fork::memory_verdict`] does for `mem_available`, and the
/// per-anchor gate remains as the backstop: it re-reads both numbers every cadence
/// and declines an anchor it cannot afford, so an over-optimistic arm costs skipped
/// anchors rather than a wedged robot.
///
/// # Order matters, because the refusal is a MESSAGE
///
/// A process can trip both arms at once (a giant map on a full machine). The size
/// arm is reported first because it names something the operator can act on — the
/// state is too big, here is the ceiling, here is the override — while "no
/// headroom" is a property of the machine at this instant and says nothing about
/// what to change.
pub fn arm_verdict(projection: &ArmProjection, mem_floor_bytes: u64) -> Option<ArmRefusal> {
    if let Some(bytes) = projection.state_bytes {
        if bytes > projection.max_state_bytes {
            return Some(ArmRefusal::StateTooLarge);
        }
        // The FIRST anchor's reservation is this same bound (the per-anchor gate seeds the
        // projection from anon RSS when nothing has been observed yet), so asking
        // the per-anchor question here asks exactly what the boundary will ask.
        if let Some(available) = projection.mem_available {
            if available.saturating_sub(bytes) < mem_floor_bytes {
                return Some(ArmRefusal::NoHeadroom);
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The kill switch's whole table, hand-written.
    ///
    /// The `Unrecognized` rows are the load-bearing ones: a typo'd kill switch must
    /// be REPORTED, because the operator's belief ("I turned it off") and the
    /// machine's behaviour ("it is on") have diverged, and nothing else in the
    /// system will ever tell them.
    #[test]
    fn the_kill_switch_reads_off_on_and_neither() {
        assert_eq!(parse_plane_switch(None), PlaneSwitch::On, "unset");
        assert_eq!(parse_plane_switch(Some("")), PlaneSwitch::On, "empty");
        assert_eq!(parse_plane_switch(Some("   ")), PlaneSwitch::On, "blank");
        for off in ["off", "OFF", "Off", " off ", "0", "false", "no"] {
            assert_eq!(parse_plane_switch(Some(off)), PlaneSwitch::Off, "{off:?}");
        }
        for on in ["on", "ON", "1", "true", "yes"] {
            assert_eq!(parse_plane_switch(Some(on)), PlaneSwitch::On, "{on:?}");
        }
        // A near-miss is not a guess. `of` is one keystroke from `off`.
        assert_eq!(
            parse_plane_switch(Some("of")),
            PlaneSwitch::Unrecognized("of".to_string())
        );
        assert_eq!(
            parse_plane_switch(Some(" disabled\n")),
            PlaneSwitch::Unrecognized("disabled".to_string()),
            "trimmed for the report, not accepted"
        );
    }

    /// A numeric override is honoured, and every way of failing to give one
    /// DEGRADES to the default WITH a complaint — never silently, never fatally.
    #[test]
    fn a_numeric_override_is_honoured_or_complained_about() {
        assert_eq!(
            resolve_cadence_ms(None),
            (DEFAULT_FLASHBACK_CADENCE_MS, None)
        );
        assert_eq!(
            resolve_cadence_ms(Some("")),
            (DEFAULT_FLASHBACK_CADENCE_MS, None)
        );
        assert_eq!(resolve_cadence_ms(Some(" 30000 ")), (30_000, None));

        let (value, complaint) = resolve_cadence_ms(Some("0"));
        assert_eq!(value, DEFAULT_FLASHBACK_CADENCE_MS, "0 is refused");
        assert!(
            complaint
                .as_deref()
                .is_some_and(|c| c.contains("0 milliseconds")),
            "a refused 0 must say so: {complaint:?}"
        );

        let (value, complaint) = resolve_cadence_ms(Some("soon"));
        assert_eq!(value, DEFAULT_FLASHBACK_CADENCE_MS);
        assert!(
            complaint.as_deref().is_some_and(|c| c.contains("soon")),
            "the offending text must reach the operator: {complaint:?}"
        );

        // The size knob is stated in MEBIBYTES and lands in BYTES — a unit slip here
        // would move the ceiling by six orders of magnitude in silence.
        assert_eq!(resolve_max_state_bytes(Some("32")).0, 32 * 1024 * 1024);
        // …and the DEFAULT is passed through at FULL PRECISION, never round-tripped
        // through whole mebibytes: the derived ceiling is 673.68 MiB, so a
        // round-trip shaves 717 446 B off it and refuses a band of states that
        // project INSIDE the budget. Both sides of that band are pinned below.
        assert_eq!(
            resolve_max_state_bytes(None).0,
            default_max_state_bytes(),
            "the default must not be rounded — it is a derivation, not a typed number"
        );
        assert_ne!(
            default_max_state_bytes() % (1024 * 1024),
            0,
            "anti-tautology: the assertion above only bites because the derived \
             ceiling is NOT a whole number of MiB"
        );
        // A DEGRADED override lands on the same full-precision default, not on a
        // shaved copy of it.
        for degraded in ["0", "soon"] {
            assert_eq!(
                resolve_max_state_bytes(Some(degraded)).0,
                default_max_state_bytes(),
                "{degraded:?} must fall back to the FULL default"
            );
        }

        // THE SENTINEL BOUNDARY. `u64::MAX` PARSES and is the largest value anybody
        // can express, so a magic-number "absent" marker makes it mean its own
        // opposite: an operator asking for no practical ceiling silently gets the
        // tightest one. Both sides pinned — the maximum is HONOURED, and it is
        // distinguishable from an absent override.
        let (max_override, complaint) = resolve_max_state_bytes(Some("18446744073709551615"));
        assert_eq!(
            complaint, None,
            "u64::MAX mebibytes parses cleanly — there is nothing to complain about"
        );
        assert_eq!(
            max_override,
            u64::MAX,
            "the largest expressible override must be HONOURED (saturated to a byte \
             ceiling nothing can reach), never read as `absent`"
        );
        assert_ne!(
            max_override,
            default_max_state_bytes(),
            "…and must be distinguishable from having set nothing at all"
        );
        // The neighbours either side of it behave the same way, so the fix is not
        // itself a special case at one value.
        assert_eq!(
            resolve_max_state_bytes(Some("18446744073709551614")).0,
            u64::MAX,
            "one below the maximum also saturates, and is also not `absent`"
        );
        // A value that does NOT saturate round-trips exactly, which is what proves
        // the saturation above is the ceiling and not a swallowed conversion.
        assert_eq!(
            resolve_max_state_bytes(Some("4096")).0,
            4096 * 1024 * 1024,
            "an ordinary override is scaled exactly"
        );
    }

    /// A degraded ceiling's complaint must quote the default ACTUALLY IN FORCE.
    ///
    /// The degraded paths fall back to `default_max_state_bytes()` at full
    /// precision, and that value is not a whole number of mebibytes. Rendering it
    /// as one would tell the operator "using the default 673" while 673.68 MiB
    /// (706 409 094 B) is what applies — a figure 717 446 B TIGHTER than the
    /// truth, and one an operator who believed they were restating the default
    /// would then set. At the default constants the line reads
    /// "using the default 673.68 mebibytes (706409094 bytes)".
    ///
    /// The value and the text are asserted together, because the defect is
    /// the two disagreeing while each looks reasonable alone.
    #[test]
    fn a_degraded_ceiling_complaint_quotes_the_default_actually_in_force() {
        const MIB: u64 = 1024 * 1024;
        let bytes = default_max_state_bytes();
        // ANTI-TAUTOLOGY: every assertion below only bites because the derived
        // ceiling is NOT a whole number of MiB. If it ever becomes one, the
        // misreport this pins is unreachable and this test must be re-thought
        // rather than silently passing.
        assert_ne!(
            bytes % MIB,
            0,
            "the ceiling is a derivation, and its not being round is the point"
        );

        for degraded in ["0", "not-a-number", ""] {
            let (value, complaint) = resolve_max_state_bytes(Some(degraded));
            assert_eq!(
                value, bytes,
                "{degraded:?} must fall back at FULL precision"
            );
            let Some(complaint) = complaint else {
                // An EMPTY value is "the operator computed nothing", not a typo, so
                // it degrades silently — the one row here with no text to check.
                assert_eq!(degraded, "", "only an empty value degrades silently");
                continue;
            };
            // The EXACT figure the gate will compare against.
            assert!(
                complaint.contains(&format!("{bytes} bytes")),
                "the complaint must quote the ceiling in force: {complaint}"
            );
            // …and in the unit the knob is typed in, WITH its fraction. Truncating
            // to whole mebibytes is the regression: the digits are followed by a
            // decimal point, never by the end of the number.
            //
            // The FRACTION's DIGITS, not merely a decimal point: a
            // renderer that keeps the point and zeroes what follows — `673.00` —
            // satisfies a `contains("673.")` while naming the same tighter ceiling
            // this arm exists to forbid.
            assert!(
                complaint.contains(&format!(
                    "{}.{:02} mebibytes",
                    bytes / MIB,
                    (bytes % MIB) * 100 / MIB
                )),
                "the mebibyte figure must keep its fraction — a whole-MiB default \
                 names a ceiling tighter than the one applied: {complaint}"
            );
            assert!(
                complaint.contains("mebibytes"),
                "…named in the operator's unit: {complaint}"
            );
        }
    }

    /// The cadence arithmetic, at its boundaries.
    #[test]
    fn the_cadence_is_a_step_count_rounded_up_and_floored_at_one() {
        // A 1 kHz graph (1 ms quantum) at the shipped 15 s cadence.
        assert_eq!(cadence_steps(15_000, 1_000_000), 15_000);
        // A 100 Hz graph anchors a tenth as often in steps, and the SAME often in
        // logical time — which is the whole reason the cadence is derived.
        assert_eq!(cadence_steps(15_000, 10_000_000), 1_500);
        // Rounded UP: 15 s at a 4 s quantum is 3.75 steps ⇒ 4, never 3.
        assert_eq!(cadence_steps(15_000, 4_000_000_000), 4);
        // Floored at 1, both ways it can be reached.
        assert_eq!(cadence_steps(1, 1_000_000_000), 1, "cadence below one step");
        assert_eq!(cadence_steps(15_000, 0), 1, "a zero quantum cannot divide");
    }

    /// The stall projection, against the measurement it is derived from.
    ///
    /// The anchor row is the MEASURED one: a dense 256 MiB slab cost 91.6–92.8 ms.
    /// A projection that did not land in that band would mean the constant no
    /// longer describes the machine it was measured on.
    #[test]
    fn the_stall_projection_reproduces_the_measured_basis() {
        let measured_slab = 256 * 1024 * 1024;
        let projected = projected_stall_ms(measured_slab);
        assert!(
            (91..=96).contains(&projected),
            "256 MiB was MEASURED at 91.6-92.8 ms; the constant projects {projected} ms"
        );
        assert_eq!(projected_stall_ms(0), 0);
        assert_eq!(
            projected_stall_ms(BYTES_PER_GIB),
            FLASHBACK_STALL_MS_PER_GIB
        );
        // Saturating rather than wrapping: the one slip that would ADMIT the process
        // this gate exists to refuse.
        assert!(projected_stall_ms(u64::MAX) > FLASHBACK_STALL_BUDGET_MS);
    }

    /// The ceiling is DERIVED from the budget and the rate, on both sides.
    #[test]
    fn the_default_ceiling_is_the_largest_state_inside_the_stall_budget() {
        let ceiling = default_max_state_bytes();
        assert!(
            projected_stall_ms(ceiling) <= FLASHBACK_STALL_BUDGET_MS,
            "the ceiling itself must fit the budget"
        );
        // …and a state materially past it does not. The discriminator is 8 MiB
        // rather than 1 byte because the projection's resolution is one whole
        // millisecond — `BYTES_PER_GIB / FLASHBACK_STALL_MS_PER_GIB` ≈ 2.8 MiB of
        // state per projected ms — so a byte, or even a mebibyte, is INSIDE the
        // integer division's own rounding. The claim being pinned is that the
        // ceiling sits at the budget's edge, not that it is the exact last byte.
        assert!(
            projected_stall_ms(ceiling + 8 * 1024 * 1024) > FLASHBACK_STALL_BUDGET_MS,
            "a state 8 MiB past the ceiling must project past the budget"
        );

        // THE SHAVED BAND, pinned on both sides. A ceiling round-tripped through
        // whole mebibytes lands at `floor(ceiling / MiB) * MiB`, and every state
        // between that and the real ceiling projects INSIDE the budget yet would be
        // refused. This is the arm that fails if the rounding comes back.
        let rounded = (ceiling / (1024 * 1024)) * (1024 * 1024);
        assert!(
            rounded < ceiling,
            "precondition: the rounding really shaves"
        );
        let (in_force, _) = resolve_max_state_bytes(None);
        for probe in [rounded, rounded + 1, ceiling] {
            assert!(
                projected_stall_ms(probe) <= FLASHBACK_STALL_BUDGET_MS,
                "{probe} projects {} ms, inside the {FLASHBACK_STALL_BUDGET_MS} ms budget",
                projected_stall_ms(probe)
            );
            assert!(
                probe <= in_force,
                "…so the ceiling in force must ADMIT it: {probe} > {in_force}"
            );
        }
    }

    /// A helper so each verdict arm below states only what it varies.
    fn projection(state_bytes: Option<u64>, mem_available: Option<u64>) -> ArmProjection {
        ArmProjection {
            state_bytes,
            mem_available,
            max_state_bytes: default_max_state_bytes(),
            ring_bytes: 64 * 1024 * 1024,
            cadence_steps: 15_000,
            cadence_ms: DEFAULT_FLASHBACK_CADENCE_MS,
        }
    }

    /// The gate's decision table — every arm, and the boundary pinned on BOTH sides.
    #[test]
    fn the_arm_gate_refuses_a_giant_state_and_a_full_machine_and_nothing_else() {
        let floor = 512 * 1024 * 1024;
        let ceiling = default_max_state_bytes();
        let roomy = Some(8 * 1024 * 1024 * 1024);

        // An ordinary robot: 64 MiB of state on a machine with room.
        assert_eq!(
            arm_verdict(&projection(Some(64 * 1024 * 1024), roomy), floor),
            None
        );
        // THE BOUNDARY, both sides. `>` not `>=`: a state exactly at the ceiling
        // projects a stall exactly at the budget, which is the budget being MET.
        assert_eq!(arm_verdict(&projection(Some(ceiling), roomy), floor), None);
        assert_eq!(
            arm_verdict(&projection(Some(ceiling + 1), roomy), floor),
            Some(ArmRefusal::StateTooLarge)
        );
        // A small state on a machine with nothing left: the first anchor would be
        // declined by the per-anchor gate, so arming buys a ring and no anchors.
        assert_eq!(
            arm_verdict(&projection(Some(64 * 1024 * 1024), Some(floor)), floor),
            Some(ArmRefusal::NoHeadroom)
        );
        // …and its boundary: available − state == floor is enough.
        assert_eq!(
            arm_verdict(
                &projection(Some(64 * 1024 * 1024), Some(floor + 64 * 1024 * 1024)),
                floor
            ),
            None
        );
        assert_eq!(
            arm_verdict(
                &projection(Some(64 * 1024 * 1024), Some(floor + 64 * 1024 * 1024 - 1)),
                floor
            ),
            Some(ArmRefusal::NoHeadroom)
        );
        // BOTH arms trip: the actionable one is reported.
        assert_eq!(
            arm_verdict(&projection(Some(ceiling + 1), Some(floor)), floor),
            Some(ArmRefusal::StateTooLarge)
        );
    }

    /// Unknown numbers ARM — the fail-OPEN half, which is the same decision
    /// `memory_verdict` makes and for the same reason.
    #[test]
    fn a_platform_that_cannot_say_still_arms() {
        let floor = 512 * 1024 * 1024;
        assert_eq!(arm_verdict(&projection(None, None), floor), None);
        assert_eq!(
            arm_verdict(&projection(None, Some(0)), floor),
            None,
            "an unknown state size cannot be judged against a known machine"
        );
        assert_eq!(
            arm_verdict(&projection(Some(1024), None), floor),
            None,
            "a known small state on an unreadable machine still arms"
        );
        // …and the projection reports the unknown rather than a fabricated 0.
        assert_eq!(projection(None, None).stall_ms(), None);
        assert_eq!(projection(Some(BYTES_PER_GIB), None).stall_ms(), Some(380));
    }

    /// The env var NAMES are a cross-process contract — an operator's shell sets
    /// them and three binaries read them, so a rename in one place is a switch that
    /// silently stops working.
    #[test]
    fn the_env_var_names_are_the_documented_ones() {
        assert_eq!(FLASHBACK_ENV, "CERULION_FLASHBACK");
        assert_eq!(FLASHBACK_CADENCE_MS_ENV, "CERULION_FLASHBACK_CADENCE_MS");
        assert_eq!(
            FLASHBACK_MAX_STATE_MB_ENV,
            "CERULION_FLASHBACK_MAX_STATE_MB"
        );
        assert_eq!(FLASHBACK_WINDOW_MS_ENV, "CERULION_FLASHBACK_WINDOW_MS");
        assert_eq!(
            FLASHBACK_STATE_RING_MB_ENV,
            "CERULION_FLASHBACK_STATE_RING_MB"
        );
        assert_eq!(
            FLASHBACK_WINDOW_MAX_MB_ENV,
            "CERULION_FLASHBACK_WINDOW_MAX_MB"
        );
        assert_eq!(
            FLASHBACK_ANCHOR_MAX_MB_ENV,
            "CERULION_FLASHBACK_ANCHOR_MAX_MB"
        );
        assert_eq!(
            FLASHBACK_TAP_BUDGET_MB_ENV,
            "CERULION_FLASHBACK_TAP_BUDGET_MB"
        );
    }

    /// The per-topic tap budget resolves through the shared
    /// parser — so its degraded arms behave EXACTLY as every other Flashback
    /// byte knob's do, rather than growing their own dialect.
    ///
    /// The default is asserted against `DEFAULT_FLASHBACK_TAP_BUDGET_MB` AND
    /// against the ingress rule's own 64 MiB: "one number, two rules" is a claim
    /// the doc makes, and a claim in a doc that no test reads is a claim that
    /// drifts.
    #[test]
    fn the_tap_budget_resolves_through_the_shared_parser() {
        // Absent / empty: the default, in bytes, with nothing to complain about.
        assert_eq!(
            resolve_tap_budget_bytes(None),
            (DEFAULT_FLASHBACK_TAP_BUDGET_MB * 1024 * 1024, None)
        );
        assert_eq!(
            resolve_tap_budget_bytes(Some("")),
            (DEFAULT_FLASHBACK_TAP_BUDGET_MB * 1024 * 1024, None)
        );
        // The shipped default IS the ingress budget — the doc's claim,
        // as an assertion.
        assert_eq!(
            DEFAULT_FLASHBACK_TAP_BUDGET_MB * 1024 * 1024,
            64 * 1024 * 1024
        );

        // An override wins, whitespace and all (the parser trims).
        assert_eq!(resolve_tap_budget_bytes(Some(" 128 ")).0, 128 * 1024 * 1024);
        assert_eq!(resolve_tap_budget_bytes(Some("1")).0, 1024 * 1024);

        // ZERO is refused LOUDLY and falls back — never silently disables the
        // tap by asking for a zero-slot queue.
        let (value, complaint) = resolve_tap_budget_bytes(Some("0"));
        assert_eq!(value, DEFAULT_FLASHBACK_TAP_BUDGET_MB * 1024 * 1024);
        let complaint = complaint.expect("a 0 budget must be reported, not absorbed");
        assert!(complaint.contains("0 mebibytes"), "got {complaint:?}");

        // Unparseable is refused LOUDLY, quoting the offender and the default in
        // force. The default here IS a whole number of its own unit, so it
        // renders as itself.
        let (value, complaint) = resolve_tap_budget_bytes(Some("lots"));
        assert_eq!(value, DEFAULT_FLASHBACK_TAP_BUDGET_MB * 1024 * 1024);
        let complaint = complaint.expect("an unparseable budget must be reported");
        assert!(complaint.contains("\"lots\""), "got {complaint:?}");
        assert!(
            complaint.contains(&format!("default {DEFAULT_FLASHBACK_TAP_BUDGET_MB}")),
            "the complaint must quote the ceiling actually in force; got {complaint:?}"
        );

        // An astronomical ask SATURATES rather than wrapping into a tiny budget
        // (which would make every tap two slots deep — the inversion of what the
        // operator typed).
        assert_eq!(
            resolve_tap_budget_bytes(Some(&u64::MAX.to_string())).0,
            u64::MAX
        );
    }

    /// The anchor is DERIVED-EQUAL to the RESOLVED window, so an
    /// env-overridden window DRAGS it — unless the anchor's own env says otherwise.
    ///
    /// All FOUR combinations, because the rule is a two-input table and testing
    /// three of its corners is how a table ships with one wrong answer. An
    /// anchor on its own const passes the "neither set" corner and gets the
    /// window-set-anchor-absent corner exactly backwards: the anchor stays on its
    /// own const while the operator believes they have moved both halves of one
    /// priced cost.
    #[test]
    fn the_anchor_is_dragged_by_the_window_unless_its_own_env_says_otherwise() {
        const MIB: u64 = 1024 * 1024;

        // (1) NEITHER set: the anchor is the window, whatever the window turned out
        // to be — here a derived one, so the drag is transitive through the
        // derivation and not merely through the env.
        let window = window_cap_from_eff_ram(Some(machine_mem::EffectiveRam {
            bytes: 64 * 1024 * MIB,
            basis: machine_mem::RamBasis::Machine,
        }));
        assert_eq!(window.bytes, 4 * 1024 * MIB, "64 GiB / 16");
        let (anchor, complaint) = resolve_anchor_max_bytes(None, window.bytes);
        assert_eq!(
            anchor,
            ResolvedCap {
                bytes: window.bytes,
                basis: CapBasis::Window
            },
            "a derived window drags the anchor with it"
        );
        assert_eq!(complaint, None);

        // (2) WINDOW set, anchor absent — THE headline. The window's own env has
        // already been folded into `window.bytes` by `resolve_window_max_bytes`, so
        // the anchor follows it without ever reading that env itself.
        let (window_env, _) = resolve_window_max_bytes(Some("2048"), None);
        assert_eq!(
            window_env,
            ResolvedCap {
                bytes: 2048 * MIB,
                basis: CapBasis::Env
            }
        );
        assert_eq!(
            resolve_anchor_max_bytes(None, window_env.bytes).0,
            ResolvedCap {
                bytes: 2048 * MIB,
                basis: CapBasis::Window
            },
            "raising the window must raise the anchor — one standing cost, priced once"
        );

        // (3) BOTH set: each is exactly what its operator typed. An explicit value
        // is never a suggestion.
        assert_eq!(
            resolve_anchor_max_bytes(Some("48"), window_env.bytes).0,
            ResolvedCap {
                bytes: 48 * MIB,
                basis: CapBasis::Env
            },
            "an explicit anchor wins outright — the held-fixed rule"
        );

        // (4) ANCHOR set, window absent: same answer, and the window's value does
        // not leak into it.
        let (window_default, _) = resolve_window_max_bytes(None, None);
        assert_eq!(
            resolve_anchor_max_bytes(Some("48"), window_default.bytes).0,
            ResolvedCap {
                bytes: 48 * MIB,
                basis: CapBasis::Env
            }
        );
        assert_ne!(
            48 * MIB,
            window_default.bytes,
            "anti-tautology: corner (4) only bites because the explicit anchor and \
             the window default are DIFFERENT numbers"
        );

        // A ZERO is refused WITH the value in force quoted, exactly as its
        // siblings — and the value in force is the DRAGGED one, not a constant.
        let (anchor, complaint) = resolve_anchor_max_bytes(Some("0"), window_env.bytes);
        assert_eq!(anchor.bytes, window_env.bytes);
        assert_eq!(anchor.basis, CapBasis::Window);
        let complaint = complaint.expect("a refused 0 must say so");
        assert!(complaint.contains("mebibytes"), "{complaint}");
        assert!(
            complaint.contains(&format!("{} bytes", window_env.bytes)),
            "the complaint must quote the DRAGGED ceiling, not a constant: {complaint}"
        );

        // An astronomical ask SATURATES rather than wrapping into a tiny one —
        // the inversion `resolve_positive_override`'s own docs exist to prevent.
        assert_eq!(
            resolve_anchor_max_bytes(Some(&u64::MAX.to_string()), window_env.bytes)
                .0
                .bytes,
            u64::MAX
        );
    }

    /// `clamp(effRAM / 16, 320 MiB, 8 GiB)`, on the machine shapes the
    /// constants are sized against and on BOTH clamp edges.
    #[test]
    fn the_window_cap_is_a_clamped_fraction_of_effective_ram() {
        const MIB: u64 = 1024 * 1024;
        const GIB: u64 = 1024 * MIB;

        let derived = |bytes: u64, basis: machine_mem::RamBasis| {
            window_cap_from_eff_ram(Some(machine_mem::EffectiveRam { bytes, basis }))
        };

        // An "8 GB" Orin NX's REAL MemTotal, which is what the sizing is done
        // against: ~7.4 GiB ⇒ ~473 MiB, not the marketing 512.
        let nx = derived(7_736_516 * 1024, machine_mem::RamBasis::Machine);
        assert_eq!(nx.bytes, 7_736_516 * 1024 / 16);
        assert!(
            (460 * MIB..480 * MIB).contains(&nx.bytes),
            "the NX lands in the ~465-473 MiB band: {}",
            nx.bytes
        );
        assert!(
            matches!(
                nx.basis,
                CapBasis::Ram {
                    clamp: CapClamp::Unclamped,
                    ..
                }
            ),
            "an NX is inside the band, not on either edge"
        );

        // An AGX: 64 GiB ⇒ 4 GiB, comfortably inside the band.
        assert_eq!(
            derived(64 * GIB, machine_mem::RamBasis::Machine).bytes,
            4 * GIB
        );

        // A desk: 128 GiB ⇒ 8 GiB EXACTLY, which is the ceiling — so the ceiling is
        // reached rather than crossed, and the clamp reports UNCLAMPED.
        let desk = derived(128 * GIB, machine_mem::RamBasis::Machine);
        assert_eq!(desk.bytes, FLASHBACK_WINDOW_CEILING_BYTES);
        assert!(matches!(
            desk.basis,
            CapBasis::Ram {
                clamp: CapClamp::Unclamped,
                ..
            }
        ));

        // THE CEILING EDGE, both sides. `divisor` bytes more is one byte over.
        let over = derived(
            128 * GIB + FLASHBACK_WINDOW_RAM_DIVISOR,
            machine_mem::RamBasis::Machine,
        );
        assert_eq!(over.bytes, FLASHBACK_WINDOW_CEILING_BYTES, "clamped down");
        assert!(matches!(
            over.basis,
            CapBasis::Ram {
                clamp: CapClamp::Ceiling,
                ..
            }
        ));

        // THE FLOOR EDGE, both sides. 320 MiB × 16 = 5 GiB is exactly the floor.
        let at_floor = derived(
            FLASHBACK_WINDOW_FLOOR_BYTES * FLASHBACK_WINDOW_RAM_DIVISOR,
            machine_mem::RamBasis::Machine,
        );
        assert_eq!(at_floor.bytes, FLASHBACK_WINDOW_FLOOR_BYTES);
        assert!(
            matches!(
                at_floor.basis,
                CapBasis::Ram {
                    clamp: CapClamp::Unclamped,
                    ..
                }
            ),
            "AT the floor is not BELOW it"
        );
        let under = derived(
            FLASHBACK_WINDOW_FLOOR_BYTES * FLASHBACK_WINDOW_RAM_DIVISOR
                - FLASHBACK_WINDOW_RAM_DIVISOR,
            machine_mem::RamBasis::Machine,
        );
        assert_eq!(under.bytes, FLASHBACK_WINDOW_FLOOR_BYTES, "clamped up");
        assert!(matches!(
            under.basis,
            CapBasis::Ram {
                clamp: CapClamp::Floor,
                ..
            }
        ));

        // A Pi-class board: 4 GiB ⇒ 256 MiB, which the floor lifts back to the static
        // 320 MiB — the priced Go2 window survives on a small board.
        let pi = derived(4 * GIB, machine_mem::RamBasis::Machine);
        assert_eq!(pi.bytes, FLASHBACK_WINDOW_FLOOR_BYTES);
        assert_eq!(
            pi.bytes,
            DEFAULT_FLASHBACK_WINDOW_MAX_MB * MIB,
            "the floor IS the static default"
        );

        // A CONTAINER decides, and says so — the whole reason effRAM has a cgroup
        // term. The basis travels through the derivation so the spawn line can name
        // it; a container-sized cap reported as "the machine" sends an operator to
        // the wrong place.
        let container = derived(2 * GIB, machine_mem::RamBasis::Cgroup);
        assert_eq!(
            container.basis,
            CapBasis::Ram {
                eff_ram_bytes: 2 * GIB,
                ram_basis: machine_mem::RamBasis::Cgroup,
                clamp: CapClamp::Floor,
            }
        );

        // A degenerate machine cannot underflow or wrap — it lands on the floor.
        assert_eq!(
            derived(0, machine_mem::RamBasis::Machine).bytes,
            FLASHBACK_WINDOW_FLOOR_BYTES
        );
        // …and an absurd one cannot overflow past the ceiling.
        assert_eq!(
            derived(u64::MAX, machine_mem::RamBasis::Machine).bytes,
            FLASHBACK_WINDOW_CEILING_BYTES
        );
    }

    /// An unreadable machine keeps EXACTLY the static default
    /// window — never less, and never a refusal.
    ///
    /// The degradation ladder's whole promise: `effective_ram` answering `None`
    /// (no `/proc`, no cgroup, an unsupported platform) must fail TOWARD the static
    /// default. A derivation that read an unknown machine as zero would clamp
    /// every such machine to the floor too — the same number, by accident rather than
    /// by rule — which is why the basis is asserted as well as the value.
    #[test]
    fn an_unreadable_machine_lands_on_todays_number_and_says_so() {
        let cap = window_cap_from_eff_ram(None);
        assert_eq!(cap.bytes, FLASHBACK_WINDOW_FLOOR_BYTES);
        assert_eq!(
            cap.bytes,
            DEFAULT_FLASHBACK_WINDOW_MAX_MB * 1024 * 1024,
            "the fallback is the SHIPPED static default, not a new number"
        );
        assert_eq!(
            cap.basis,
            CapBasis::UnknownRam,
            "…and it is REPORTED as a read failure rather than passed off as a \
             derivation, because those have different remedies"
        );
        // The derivation can never SHRINK a window: the floor is the static default, so
        // every machine shape lands at or above it.
        for bytes in [0, 1, 1024, 4 * 1024 * 1024 * 1024, u64::MAX] {
            let derived = window_cap_from_eff_ram(Some(machine_mem::EffectiveRam {
                bytes,
                basis: machine_mem::RamBasis::Machine,
            }));
            assert!(
                derived.bytes >= FLASHBACK_WINDOW_FLOOR_BYTES,
                "effRAM {bytes} derived {} — below the static default",
                derived.bytes
            );
        }
    }

    /// The spawn-time projection: what a cap buys at a declared rate.
    #[test]
    fn the_spawn_projection_states_what_a_cap_buys() {
        const MIB: u64 = 1024 * 1024;

        // The measured Go2: 5.1 MB/s against the static 320 MiB reaches ~65 s, i.e.
        // the full 30 s promise at better than 2x margin (2.18x).
        let go2 = project_window_reach(320 * MIB, 5_100_000, DEFAULT_FLASHBACK_WINDOW_MS);
        assert_eq!(go2.reach_ms, Some(320 * MIB * 1_000 / 5_100_000));
        assert!(go2.holds_full_span());
        assert!(
            go2.reach_ms.unwrap() >= 2 * DEFAULT_FLASHBACK_WINDOW_MS,
            "the priced margin: {:?}",
            go2.reach_ms
        );

        // The humanoid at realistic payloads: ~230 MB/s against the same cap
        // reaches ~1.5 s — the shortfall the reporting exists to state out loud.
        let humanoid = project_window_reach(320 * MIB, 230_000_000, DEFAULT_FLASHBACK_WINDOW_MS);
        assert_eq!(humanoid.reach_ms, Some(1_458));
        assert!(!humanoid.holds_full_span());

        // …and the SAME robot on a 64 GiB AGX, whose derived 4 GiB cap turns 1.5 s
        // into ~18.7 s. This pair is why the cap is derived from RAM.
        let agx = project_window_reach(4 * 1024 * MIB, 230_000_000, DEFAULT_FLASHBACK_WINDOW_MS);
        assert_eq!(agx.reach_ms, Some(18_673));
        assert!(!agx.holds_full_span(), "even 4 GiB is short of 30 s here");

        // EXACTLY the span holds — a `>` would report the boundary case as a
        // shortfall and send an operator chasing a cap that is already right.
        let exact = project_window_reach(30_000_000, 1_000_000, 30_000);
        assert_eq!(exact.reach_ms, Some(30_000));
        assert!(exact.holds_full_span());
        let short = project_window_reach(29_999_999, 1_000_000, 30_000);
        assert_eq!(short.reach_ms, Some(29_999));
        assert!(!short.holds_full_span());

        // NOTHING OFFERED is not "reaches forever": the projection made no claim,
        // and `holds_full_span` must not read that silence as success.
        let silent = project_window_reach(320 * MIB, 0, DEFAULT_FLASHBACK_WINDOW_MS);
        assert_eq!(silent.reach_ms, None);
        assert!(!silent.holds_full_span());

        // The aggregate itself, against a hand sum: one 16 MiB frame at 10 Hz plus
        // forty 1 KiB frames at 100 Hz.
        let declared = [
            DeclaredTopicRate {
                bytes_per_frame: 16 * MIB,
                millihertz: 10_000,
            },
            DeclaredTopicRate {
                bytes_per_frame: 1024,
                millihertz: 100_000,
            },
        ];
        assert_eq!(
            offered_bytes_per_second(&declared),
            16 * MIB * 10 + 1024 * 100
        );
        assert_eq!(offered_bytes_per_second(&[]), 0);
        // A sub-hertz topic contributes its real share… and a rate BELOW one
        // millihertz-second truly is zero, which is the documented truncation
        // rather than a surprise.
        assert_eq!(
            offered_bytes_per_second(&[DeclaredTopicRate {
                bytes_per_frame: 2_000,
                millihertz: 500,
            }]),
            1_000,
            "2 KB at 0.5 Hz is 1 KB/s"
        );
        // THE SCALE IS DIVIDED OUT ONCE, AT THE END. Two topics each offering
        // half a byte a second offer ONE together; a per-topic divide truncates
        // both to zero and reports a graph that publishes nothing — which the
        // projection then renders as an unknown reach rather than a finite one.
        // The under-report is the dangerous direction, so this is the vector the
        // fix exists for.
        let half_a_byte = DeclaredTopicRate {
            bytes_per_frame: 1,
            millihertz: 500,
        };
        assert_eq!(
            offered_bytes_per_second(&[half_a_byte]),
            0,
            "one sub-unit topic really does floor to zero — the truncation is real"
        );
        assert_eq!(
            offered_bytes_per_second(&[half_a_byte, half_a_byte]),
            1,
            "…but two of them offer a whole byte a second, and the total must say so"
        );
        // The aggregate keeps the remainders a per-topic divide would throw away.
        // Hand-computed: `u64::MAX / 4` = 4_611_686_018_427_387_903, whose own
        // divide loses 903; two of them sum to 9_223_372_036_854_775_806, which is
        // ONE MORE than twice the truncated single — the recovered remainder, and
        // the reason this pair is asserted rather than `2 * single`.
        let big = DeclaredTopicRate {
            bytes_per_frame: u64::MAX / 4,
            millihertz: 1,
        };
        assert_eq!(offered_bytes_per_second(&[big]), 4_611_686_018_427_387);
        assert_eq!(
            offered_bytes_per_second(&[big, big]),
            9_223_372_036_854_775,
            "two large topics SUM (and recover the truncated remainder), never wrap"
        );
        // An absurd declaration SATURATES rather than wrapping into a small,
        // plausible, wrong number — which is the direction that matters, because a
        // wrapped total reads as a healthy robot.
        //
        // With the divide at the END there are still two overflow sites, and the
        // CEILING is `u64::MAX / 1000` at both: the per-topic MULTIPLY pins at
        // `u64::MAX`, the accumulator pins there too, and the scale is removed
        // from the pinned total. Hand-computed, not read off the implementation.
        let absurd = DeclaredTopicRate {
            bytes_per_frame: u64::MAX,
            millihertz: u64::MAX,
        };
        assert_eq!(offered_bytes_per_second(&[absurd]), u64::MAX / 1_000);
        assert_eq!(
            offered_bytes_per_second(&[absurd, absurd]),
            u64::MAX / 1_000,
            "the accumulator saturates, so a second absurd topic adds nothing — it \
             does NOT wrap into a small, plausible number"
        );
        assert_eq!(
            offered_bytes_per_second(&vec![absurd; 1_001]),
            u64::MAX / 1_000,
            "…and a thousand more of them stay at the same ceiling"
        );
    }

    /// The window SPAN is arithmetic over its inputs, and the assertion is the
    /// arithmetic rather than the number: a `assert_eq!(.., 30_000)` would pass
    /// against a hardcoded literal that had stopped following its inputs, which is
    /// the exact rot this derivation exists to prevent.
    #[test]
    fn the_window_span_is_derived_from_the_post_window_and_the_cadence() {
        assert_eq!(
            DEFAULT_FLASHBACK_WINDOW_MS,
            crate::flashback::trigger::DEFAULT_FLASHBACK_POST_WINDOW_MS
                + DEFAULT_FLASHBACK_CADENCE_MS
        );
        // …and, separately, that the default inputs really do produce the 30 s the
        // ~155 MB window figure is priced from. Both halves are needed: the first
        // pins the RULE, this one pins that the rule's current answer is the one
        // the memory figures were costed against.
        assert_eq!(DEFAULT_FLASHBACK_WINDOW_MS, 30_000);
    }

    /// The ring knob's DEFAULT is derived from the ring's own constant, so an
    /// operator reading `DEFAULT_STATE_RING_BYTES` and an operator reading this
    /// knob's complaint text cannot be told two different numbers.
    #[test]
    fn the_state_ring_knob_is_derived_from_the_rings_own_default() {
        assert_eq!(
            DEFAULT_FLASHBACK_STATE_RING_MB * 1024 * 1024,
            crate::state_ring::DEFAULT_STATE_RING_BYTES as u64
        );
        assert_eq!(
            resolve_state_ring_bytes(None).0,
            crate::state_ring::DEFAULT_STATE_RING_BYTES as u64
        );
        assert_eq!(resolve_state_ring_bytes(Some("256")).0, 256 * 1024 * 1024);
        let (bytes, complaint) = resolve_state_ring_bytes(Some("0"));
        assert_eq!(bytes, crate::state_ring::DEFAULT_STATE_RING_BYTES as u64);
        assert!(complaint.is_some_and(|c| c.contains("64")));
    }

    #[test]
    fn the_window_knobs_resolve_through_the_shared_parser() {
        const MIB: u64 = 1024 * 1024;
        // A JETSON-shaped machine, and the shape is the point: its derived cap is
        // 472.19 MiB — NOT a whole number of mebibytes — which is the only
        // condition under which the complaint's FRACTION is observable at all. A
        // round machine (64 GiB ⇒ exactly 4096 MiB) makes the whole-MiB round-trip
        // and the full-precision rendering indistinguishable, so the original
        // defect this arm inherits would be unpinned. It is also inside both clamp
        // edges, so the "env wins" arms below provably decide.
        let machine = Some(machine_mem::EffectiveRam {
            bytes: 7_736_516 * 1024,
            basis: machine_mem::RamBasis::Machine,
        });

        // Absent → the DERIVED default, silently.
        assert_eq!(resolve_window_ms(None), (DEFAULT_FLASHBACK_WINDOW_MS, None));
        let (cap, complaint) = resolve_window_max_bytes(None, machine);
        assert_eq!(cap, window_cap_from_eff_ram(machine));
        assert_eq!(complaint, None);
        assert_eq!(cap.bytes, 7_736_516 * 1024 / 16);
        assert_ne!(
            cap.bytes % MIB,
            0,
            "anti-tautology: the fraction assertions below only bite because this \
             machine's derived cap is NOT a whole number of mebibytes"
        );

        // A real override, in the unit the knob is typed in — and it WINS over the
        // derivation, which the differing values make observable.
        assert_eq!(resolve_window_ms(Some("45000")).0, 45_000);
        assert_eq!(
            resolve_window_max_bytes(Some("64"), machine).0,
            ResolvedCap {
                bytes: 64 * MIB,
                basis: CapBasis::Env
            },
            "an explicit window is the operator's number, not a suggestion the \
             machine may overrule"
        );

        // A ZERO is refused with a complaint and the default stays in force — the
        // shared parser's contract, which is why these knobs go through it rather
        // than parsing for themselves.
        let (ms, complaint) = resolve_window_ms(Some("0"));
        assert_eq!(ms, DEFAULT_FLASHBACK_WINDOW_MS);
        assert!(
            complaint.is_some_and(|c| c.contains("30000")),
            "the complaint must quote the default actually in force"
        );
        let (cap, complaint) = resolve_window_max_bytes(Some("nonsense"), machine);
        assert_eq!(cap, window_cap_from_eff_ram(machine));
        let complaint = complaint.expect("an unreadable value must say so");
        assert!(
            complaint.contains(&format!("{} bytes", cap.bytes)),
            "the complaint must quote the DERIVED ceiling in force, not the static \
             constant, because the number moves per \
             machine: {complaint}"
        );
        // …and in the unit the knob is TYPED in, WITH its fraction. Truncating to
        // whole mebibytes is the state-ceiling regression, and here it would be worse
        // than it was there: an operator restating "472" would cut 209 152 B off a
        // ceiling the machine had just derived for them.
        assert!(
            complaint.contains(&format!(
                "{}.{:02} mebibytes",
                cap.bytes / MIB,
                (cap.bytes % MIB) * 100 / MIB
            )),
            "the mebibyte figure must keep its fraction: {complaint}"
        );

        // An astronomical ask means "no practical ceiling" — it must SATURATE
        // rather than wrap into a tiny one, which would be the opposite of what
        // the operator asked for.
        assert_eq!(
            resolve_window_max_bytes(Some(&u64::MAX.to_string()), machine)
                .0
                .bytes,
            u64::MAX
        );
    }

    /// Anchor-first: the anchor reserves what measured state demands, and
    /// the frames get the remainder above a small floor.
    ///
    /// The ladder is driven at every rung against HAND-COMPUTED bytes, so the
    /// degradation 3 → 2 → 1 → alarm is pinned as a sequence rather than at one
    /// point. Every vector shares one plane, so the only variable is the
    /// generation size — which is what the ladder keys on.
    #[test]
    fn the_anchor_reserves_generations_before_the_frames_give_anything() {
        const MIB: u64 = 1024 * 1024;
        // The default shape: a derived anchor cap equal to the window, so the
        // plane is 2× the window.
        let window = 320 * MIB;
        let derived = ResolvedCap {
            bytes: window,
            basis: CapBasis::Window,
        };
        let floor = 64 * MIB;
        let plane = 640 * MIB;

        // THREE fit: a real attach-class robot (a Go2 carries kilobytes of
        // state) reserves its three generations out of the noise.
        let split = split_plane_budget(window, derived, Some(10 * MIB), floor);
        assert_eq!(
            split,
            PlaneSplit {
                anchor_bytes: 30 * MIB,
                frames_bytes: plane - 30 * MIB,
                verdict: PlaneSplitVerdict::Reserved { generations: 3 },
            },
            "three generations of a small state must fit, and the frames keep the rest"
        );

        // TWO: 3G would not leave the floor, 2G does.
        let split = split_plane_budget(window, derived, Some(200 * MIB), floor);
        assert_eq!(
            split,
            PlaneSplit {
                anchor_bytes: 400 * MIB,
                frames_bytes: 240 * MIB,
                verdict: PlaneSplitVerdict::Reserved { generations: 2 },
            },
            "the reserve degrades 3 → 2 before the frame window gives anything"
        );

        // ONE.
        let split = split_plane_budget(window, derived, Some(300 * MIB), floor);
        assert_eq!(
            split,
            PlaneSplit {
                anchor_bytes: 300 * MIB,
                frames_bytes: 340 * MIB,
                verdict: PlaneSplitVerdict::Reserved { generations: 1 },
            },
        );

        // BELOW one: the alarm condition. The plane still holds what it can, and
        // the frames keep exactly their floor.
        let split = split_plane_budget(window, derived, Some(600 * MIB), floor);
        assert_eq!(
            split,
            PlaneSplit {
                anchor_bytes: 576 * MIB,
                frames_bytes: floor,
                verdict: PlaneSplitVerdict::BelowOneGeneration {
                    shortfall_bytes: 24 * MIB
                },
            },
            "below one generation the plane keeps running and the frames keep their floor"
        );
        assert!(split.below_one_generation());
    }

    /// The arithmetic at the measured point.
    ///
    /// A 16 GB NX derives a 961 MiB window, so its plane is ~1.9 GiB. Against a
    /// 1.2 GiB generation that machine is sized to hold one generation plus
    /// ~700 MiB of frames — this reproduces that number rather than restating it,
    /// so a change to the ladder that still passes the rungs above cannot quietly
    /// move the machine the sizing is anchored on.
    #[test]
    fn a_sixteen_gigabyte_machine_holds_one_generation_and_seven_hundred_megabytes_of_frames() {
        const MIB: u64 = 1024 * 1024;
        let window = 961 * MIB;
        let derived = ResolvedCap {
            bytes: window,
            basis: CapBasis::Window,
        };
        // 1.2 GiB, to the mebibyte.
        let generation = 1229 * MIB;
        let split = split_plane_budget(
            window,
            derived,
            Some(generation),
            FLASHBACK_FRAMES_FLOOR_BYTES,
        );
        assert_eq!(
            split.verdict,
            PlaneSplitVerdict::Reserved { generations: 1 },
            "the NX holds exactly one generation — three are AGX-class"
        );
        assert_eq!(split.anchor_bytes, generation);
        assert_eq!(
            split.frames_bytes / MIB,
            693,
            "…and ~700 MiB of frames, which is the figure the sizing is anchored on"
        );
    }

    /// An EXPLICIT anchor env is a CEILING on the reserve — the held-fixed rule.
    ///
    /// The anti-tautology half is in the same body: the identical measurement
    /// against a DERIVED cap of the same size reserves three generations, so the
    /// clamp is what the `Env` basis buys and not an artefact of the numbers.
    #[test]
    fn an_explicit_anchor_env_caps_the_reserve_while_a_derived_one_does_not() {
        const MIB: u64 = 1024 * 1024;
        let window = 320 * MIB;
        let generation = 50 * MIB;
        let floor = 64 * MIB;

        let explicit = ResolvedCap {
            bytes: 100 * MIB,
            basis: CapBasis::Env,
        };
        let split = split_plane_budget(window, explicit, Some(generation), floor);
        assert_eq!(
            split.verdict,
            PlaneSplitVerdict::Reserved { generations: 2 },
            "an operator who states a number gets that number: 3 × 50 MiB would \
             exceed the 100 MiB they asked for"
        );
        assert_eq!(split.anchor_bytes, 100 * MIB);

        let derived_same_size = ResolvedCap {
            bytes: 100 * MIB,
            basis: CapBasis::Window,
        };
        let split = split_plane_budget(window, derived_same_size, Some(generation), floor);
        assert_eq!(
            split.verdict,
            PlaneSplitVerdict::Reserved { generations: 3 },
            "ANTI-TAUTOLOGY: the same bytes with a DERIVED basis are not a ceiling, \
             so the clamp above is the basis and not the arithmetic"
        );
    }

    /// Nothing measured yet leaves the default caps exactly as they are.
    ///
    /// The boundary that matters is `Some(0)`: a complete generation always
    /// carries at least one record, so a zero is a caller with nothing to say —
    /// and reading it as a measurement would report three reserved generations
    /// of nothing while the frames kept the whole plane.
    #[test]
    fn an_unmeasured_plane_keeps_the_shipped_split_and_a_zero_is_not_a_measurement() {
        const MIB: u64 = 1024 * 1024;
        let window = 320 * MIB;
        let anchor = ResolvedCap {
            bytes: 200 * MIB,
            basis: CapBasis::Env,
        };
        let unmeasured_split = PlaneSplit {
            anchor_bytes: 200 * MIB,
            frames_bytes: 320 * MIB,
            verdict: PlaneSplitVerdict::Unmeasured,
        };
        assert_eq!(
            split_plane_budget(window, anchor, None, FLASHBACK_FRAMES_FLOOR_BYTES),
            unmeasured_split,
            "before the first checkpoint the caps stand as resolved"
        );
        assert_eq!(
            split_plane_budget(window, anchor, Some(0), FLASHBACK_FRAMES_FLOOR_BYTES),
            unmeasured_split,
            "a zero-byte generation is not a measurement"
        );
    }

    /// The alarm names the knob that actually MOVES the plane, and a value that
    /// actually fits.
    ///
    /// Which knob depends on the anchor cap's basis, and naming the wrong one
    /// prints an instruction that does nothing — the per-verb rule
    /// applied to an env var. The value is rounded UP at both steps, so the
    /// boundary vectors are the ones that would expose a `/` where a `div_ceil`
    /// belongs.
    #[test]
    fn the_generation_remedy_names_the_knob_that_moves_the_plane() {
        const MIB: u64 = 1024 * 1024;
        let floor = 64 * MIB;

        // EXPLICIT anchor cap: that number is the binding ceiling, so raise it to
        // one whole generation.
        let explicit = ResolvedCap {
            bytes: 8 * MIB,
            basis: CapBasis::Env,
        };
        assert_eq!(
            generation_remedy(Some(600 * MIB), None, floor, explicit),
            GenerationRemedy {
                knob: FLASHBACK_ANCHOR_MAX_MB_ENV,
                mebibytes: Some(600),
                basis: RemedyBasis::Measured,
            }
        );

        // DERIVED: the anchor is dragged by the window, so the window knob is
        // what moves the plane — and it must satisfy 2W ≥ G + floor.
        let derived = ResolvedCap {
            bytes: 320 * MIB,
            basis: CapBasis::Window,
        };
        assert_eq!(
            generation_remedy(Some(600 * MIB), None, floor, derived),
            GenerationRemedy {
                knob: FLASHBACK_WINDOW_MAX_MB_ENV,
                mebibytes: Some(332),
                basis: RemedyBasis::Measured,
            },
            "(600 + 64) / 2 = 332 MiB of window, which is 664 MiB of plane"
        );
        // …and that value really does fit, checked through the split itself
        // rather than by restating the arithmetic.
        let restored = split_plane_budget(
            332 * MIB,
            ResolvedCap {
                bytes: 332 * MIB,
                basis: CapBasis::Window,
            },
            Some(600 * MIB),
            floor,
        );
        assert_eq!(
            restored.verdict,
            PlaneSplitVerdict::Reserved { generations: 1 },
            "the remedy must RESTORE one generation, or it is not a remedy"
        );

        // ROUNDING, both steps: one byte over a mebibyte must round UP, or the
        // printed value is a number that still does not fit.
        assert_eq!(
            generation_remedy(Some(600 * MIB + 1), None, floor, explicit).mebibytes,
            Some(601)
        );
        assert_eq!(
            generation_remedy(Some(1), None, 1, derived).mebibytes,
            Some(1),
            "a sub-mebibyte plane still needs a whole mebibyte asked for"
        );
    }

    /// The topic-exclude knob parses what it documents
    /// and REFUSES what it does not, loudly.
    ///
    /// Driven against hand-written inputs at every arm of the grammar in one
    /// body, because the failure mode of a misparse is silent: the operator
    /// believes a heavy topic is out of the window, the window keeps holding it,
    /// and the only symptom is a span that will not grow.
    #[test]
    fn the_exclude_list_parses_its_grammar_and_refuses_the_rest_loudly() {
        // Absent and empty are the ordinary robot: nothing excluded, nothing said.
        for raw in [None, Some(""), Some("   "), Some(",,")] {
            let (ex, complaint) = parse_exclude_topics(raw);
            assert!(ex.is_empty(), "{raw:?} must exclude nothing");
            assert_eq!(complaint, None, "{raw:?} must say nothing");
        }

        // Exact names, a trailing-star prefix, whitespace, a trailing comma and a
        // duplicate — all in one list.
        let (ex, complaint) = parse_exclude_topics(Some(" /cam/left , /cam/* , /cam/left ,"));
        assert_eq!(complaint, None, "none of those is a refusal");
        assert_eq!(
            ex.patterns(),
            vec!["/cam/left".to_string(), "/cam/*".to_string()],
            "exact first, then prefixes with their stars restored — and the \
             duplicate folded silently"
        );
        assert_eq!(ex.len(), 2);

        // What it matches, and — the half that matters — what it does NOT.
        assert!(ex.excludes("/cam/left"), "the exact name");
        assert!(ex.excludes("/cam/right"), "the prefix");
        assert!(ex.excludes("/cam/"), "the prefix, at its own boundary");
        assert!(
            !ex.excludes("/lidar/points"),
            "ANTI-TAUTOLOGY: an unlisted topic must survive, or every arm above \
             is satisfied by a list that excludes everything"
        );
        assert!(
            !ex.excludes("/cam"),
            "a PREFIX of the prefix is not a match — `/cam*` would be, and is a \
             different thing to write"
        );

        // A bare star turns the black box OFF, which is not what a memory knob
        // may do silently. Refused, named, and pointed at the switch that means
        // it.
        let (ex, complaint) = parse_exclude_topics(Some("*"));
        assert!(ex.is_empty(), "the bare star is DROPPED, not honoured");
        let complaint = complaint.expect("a bare star must be refused loudly");
        assert!(
            complaint.contains(FLASHBACK_ENV),
            "…and must name the switch that really turns the plane off: {complaint}"
        );

        // A star anywhere but the end is a glob this does not implement.
        let (ex, complaint) = parse_exclude_topics(Some("/a*/b,/c**,/ok"));
        assert_eq!(
            ex.patterns(),
            vec!["/ok".to_string()],
            "the two malformed entries are dropped and the good one survives"
        );
        let complaint = complaint.expect("malformed entries must be named");
        assert!(complaint.contains("/a*/b"), "{complaint}");
        assert!(complaint.contains("/c**"), "{complaint}");
        assert!(
            complaint.contains("2 entries"),
            "the count must be right and plural: {complaint}"
        );

        // Singular, because an operator reading "1 entries" learns the message
        // was not written for them.
        let (_, complaint) = parse_exclude_topics(Some("/a*/b"));
        let complaint = complaint.expect("one refusal");
        assert!(complaint.contains("1 entry"), "{complaint}");
    }

    /// The remedy NEVER renders a zero, and says what its
    /// number is evidence OF.
    ///
    /// The rendered value is the entire content of a capture refusal, and the
    /// plane is reachable with NOTHING measured — the harvester's in-flight
    /// ceiling can refuse every anchor before one ever completes, which is
    /// exactly the shape a too-small machine is in. Deriving the value from an
    /// absent measurement printed `…ANCHOR_MAX_MB=0`: a confidently wrong knob
    /// value, on the robot that needs the right one.
    ///
    /// Every arm asserts the BASIS as well as the number, because "600" as a
    /// measurement and "600" as a floor are different instructions.
    #[test]
    fn an_unmeasured_generation_never_renders_a_zero_remedy() {
        const MIB: u64 = 1024 * 1024;
        let floor = 64 * MIB;
        let explicit = ResolvedCap {
            bytes: 8 * MIB,
            basis: CapBasis::Env,
        };

        // THE DEFECT: no measurement, but the ceiling refused a 40 MiB anchor.
        // The remedy must be a FLOOR from that, never a zero.
        let remedy = generation_remedy(None, Some(40 * MIB), floor, explicit);
        assert_eq!(
            remedy,
            GenerationRemedy {
                knob: FLASHBACK_ANCHOR_MAX_MB_ENV,
                mebibytes: Some(40),
                basis: RemedyBasis::PartialAnchorFloor,
            },
            "a refused anchor of N bytes proves the plane needs at least N — never \
             `…ANCHOR_MAX_MB=0`"
        );
        assert!(
            remedy.mebibytes.is_some_and(|m| m >= 40),
            "the value must be at or above the refused size, never below it"
        );

        // The DERIVED-cap twin: the same floor, through the window knob.
        let derived = ResolvedCap {
            bytes: 320 * MIB,
            basis: CapBasis::Window,
        };
        let remedy = generation_remedy(None, Some(40 * MIB), floor, derived);
        assert_eq!(
            remedy,
            GenerationRemedy {
                knob: FLASHBACK_WINDOW_MAX_MB_ENV,
                // (40 + 64) / 2 = 52 MiB of window ⇒ 104 MiB of plane.
                mebibytes: Some(52),
                basis: RemedyBasis::PartialAnchorFloor,
            },
            "the floor still routes through the knob that MOVES the plane"
        );

        // NOTHING observed: the knob is named, with NO number. Never a zero.
        for cap in [explicit, derived] {
            let remedy = generation_remedy(None, None, floor, cap);
            assert_eq!(remedy.mebibytes, None, "a zero is never rendered");
            assert_eq!(remedy.basis, RemedyBasis::Unknown);
            assert!(
                !remedy.knob.is_empty(),
                "a remedy with no knob is not a remedy — the operator still needs \
                 to know WHICH variable to raise"
            );
        }

        // A ZERO in either input is NOT a measurement. Both are filtered, which
        // is what makes the Unknown arm reachable at all — and, without the
        // second filter, a refusal that held nothing of its own (the commonest
        // shape: a first record refused while a sibling holds the whole budget)
        // would be believed as a zero-byte demand.
        assert_eq!(
            generation_remedy(Some(0), Some(0), floor, explicit).basis,
            RemedyBasis::Unknown,
            "a zero generation and a zero refusal are both absences"
        );
        assert_eq!(
            generation_remedy(Some(0), Some(40 * MIB), floor, explicit).basis,
            RemedyBasis::PartialAnchorFloor,
            "a zero measurement falls through to the floor rather than winning"
        );

        // ANTI-TAUTOLOGY: a real measurement still OUTRANKS a floor, and is
        // labelled as the measurement it is.
        let remedy = generation_remedy(Some(600 * MIB), Some(40 * MIB), floor, explicit);
        assert_eq!(remedy.mebibytes, Some(600));
        assert_eq!(remedy.basis, RemedyBasis::Measured);
    }

    /// The shipped values are the documented ones.
    ///
    /// Their RELATIONSHIP — the floor fitting inside the smallest window this
    /// build will ever derive — is guarded at COMPILE time beside the constants
    /// themselves, which is strictly stronger than an arm here: a violation
    /// fails the build on every target rather than waiting for a test run. This
    /// arm pins the numbers, so a change to either has to be deliberate in both
    /// places.
    #[test]
    fn the_frames_floor_is_the_documented_number() {
        assert_eq!(DEFAULT_FLASHBACK_FRAMES_FLOOR_MB, 64);
        assert_eq!(FLASHBACK_FRAMES_FLOOR_BYTES, 64 * 1024 * 1024);
        assert_eq!(FLASHBACK_ANCHOR_GENERATIONS_TARGET, 3);
    }
}
