// SPDX-License-Identifier: AGPL-3.0-only
//! The DASHCAM CONTRACT — which flashbacks a robot keeps, and
//! which it overwrites.
//!
//! PURE: it takes a listing and the caps and returns a plan. No filesystem, no
//! clock, no `std::fs` — so every branch, including the ones a healthy robot never
//! reaches, is oracle-testable with hand-written vectors.
//!
//! # Why a plan rather than a sweep
//!
//! Deleting a robot's recordings is the one operation here that cannot be undone,
//! and the decision has three interacting rules (two caps and an escape). Splitting
//! the DECISION from the DELETION means the decision is testable without a
//! temporary directory, and the deletion is a loop with no policy in it — the same
//! split `plan_discovery` and `plan_connect_set` already use for the same reason.
//!
//! # The escape is an EXCLUSION, never a reprieve
//!
//! A pinned capture is removed from the eviction candidates entirely: it is not
//! evicted last, it is not evicted when the caps are badly exceeded, and it is not
//! evicted when it is the only thing over the cap. That is what makes the pin worth
//! anything — an operator who pinned the capture of the incident they are
//! debugging must not lose it to a robot that kept running.
//!
//! The cost is that pins can put a directory permanently over its caps, so the
//! plan REPORTS that state ([`RetentionPlan::pinned_over_cap`]) rather than
//! resolving it. A caller warns; nothing deletes a pin.
//!
//! # Eviction is diversity-preserving
//!
//! Strict oldest-first has a failure mode the caps cannot see: it defends the
//! directory's SIZE and says nothing about its COMPOSITION. A robot with one
//! chatty cause — a duty-cycled route stalling several times a day, a graph
//! relaunching in a loop — mints captures of that ONE condition steadily, and
//! oldest-first faithfully rotates the single capture of the real incident out
//! from under it while keeping a dozen copies of the noise.
//!
//! So the eviction candidate is the oldest unpinned capture of the MOST POPULOUS
//! CAUSE CLASS, ties broken by the globally-oldest candidate and then by name.
//! On a diverse directory — every class holding one capture — every class is
//! equally populous and the tie rule picks the global oldest, so the plan
//! DEGENERATES EXACTLY to oldest-first and no operator's mental model changes
//! until a flood actually starts.
//!
//! **Config-free by construction.** The alternatives were per-cause QUOTAS (a new
//! number, and a wrong one silently drops a real repeat) and severity-tier
//! eviction (an a-priori ranking of which incidents matter — configuration in a
//! trench coat). This rule introduces no number at all: it reads the composition
//! it is handed.
//!
//! **The class is the [`TriggerKind`], not the `(kind, subject)` regime.** The
//! flood shapes this exists for are per-SUBJECT by nature — a duty-cycle flood
//! raises one cause per topic, and a relaunch loop one per group — so keying the
//! class on the subject would give every flood member its own class and the rule
//! would do nothing at all on the exact shape it is for. What an operator scanning
//! the directory needs preserved is "I still have a worker-death capture and an
//! e-stop capture beside all this monitor noise", which is the KIND.

use std::collections::BTreeMap;

use super::trigger::TriggerKind;

/// The default ceiling on the flashback directory, in BYTES.
///
/// The design's proposed number. Against the ~155 MB-class capture it is about a
/// dozen captures, so on a robot that is faulting steadily the SIZE cap binds
/// before the count cap — which is the right way round, since a robot with small
/// state and few topics should get more captures rather than the same number of
/// smaller ones.
pub const DEFAULT_FLASHBACK_MAX_BYTES: u64 = 2 * 1024 * 1024 * 1024;

/// The default ceiling on the NUMBER of flashbacks kept.
///
/// The second cap exists because the first cannot bound inode churn or a listing's
/// readability: a robot producing hundreds of tiny captures would sit inside the
/// byte cap while making the v1 status surface — the directory listing — useless.
pub const DEFAULT_FLASHBACK_MAX_CAPTURES: u32 = 20;

/// Override [`DEFAULT_FLASHBACK_MAX_BYTES`], in MEBIBYTES.
///
/// Stated in mebibytes for the same reason
/// [`FLASHBACK_MAX_STATE_MB_ENV`](super::FLASHBACK_MAX_STATE_MB_ENV) is: it is a
/// number a human types, and MiB is the unit they think in.
pub const FLASHBACK_MAX_MB_ENV: &str = "CERULION_FLASHBACK_MAX_MB";

/// Override [`DEFAULT_FLASHBACK_MAX_CAPTURES`].
pub const FLASHBACK_MAX_CAPTURES_ENV: &str = "CERULION_FLASHBACK_MAX_CAPTURES";

/// Override where flashbacks land.
///
/// The default is `recordings/flashbacks/`, resolved relative to the
/// directory the graph was run from — a workspace-relative path, exactly like
/// `graphs/` and `nodes/`.
pub const FLASHBACK_DIR_ENV: &str = "CERULION_FLASHBACK_DIR";

/// Decision: this is where a flashback lands, relative to the workspace.
pub const DEFAULT_FLASHBACK_DIR: &str = "recordings/flashbacks";

/// One capture on disk.
///
/// # Where each field comes from, stated so the sweep cannot drift from the plan
///
/// This type is the WHOLE interface between the pure policy and the directory it
/// governs, and every field is a plain scalar — so a sweep that filled one from
/// the wrong source would produce a perfectly well-typed plan that evicts the
/// wrong file. The provenance is therefore part of the contract rather than an
/// implementation detail of whichever function builds it:
///
/// | field | source |
/// |---|---|
/// | `name` | the directory entry's `file_name()` |
/// | `bytes` | its `metadata().len()` |
/// | `created_ns` | the capture's DURABLE stamp — see below |
/// | `pinned` | the pin marker beside the capture |
/// | `cause` | the CAUSE marker beside the capture |
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CaptureEntry {
    /// The file name. Also the tie-break for two captures with one timestamp, so
    /// a plan is deterministic on any filesystem's listing order.
    pub name: String,
    /// Its size on disk.
    pub bytes: u64,
    /// When it was captured, as a DURABLE nanosecond stamp — one that keeps its
    /// ordering across a reboot.
    ///
    /// # NOT the monotonic clock the trigger gate runs on
    ///
    /// [`trigger`](super::trigger) is deliberately monotonic-only: it compares
    /// instants within one process's life and never renders a time of day. This
    /// field cannot be the same reading, and the difference is not stylistic — a
    /// monotonic clock RESTARTS NEAR ZERO at boot, while this directory OUTLIVES
    /// the robot's uptime. Filled from a monotonic source, every capture written
    /// after a reboot would sort BEFORE every capture written before it, so
    /// `plan_retention` would faithfully evict the NEWEST flashbacks first and
    /// keep the stale ones — the dashcam contract inverted, silently, and only on
    /// the robots that have been up long enough to matter.
    ///
    /// So it is a wall-clock-derived stamp (the file's modification time, or the
    /// capture's own recorded creation time — both survive a reboot). The cost is
    /// the ordinary wall-clock one: a clock stepped backwards can misorder
    /// captures across the step. That is a bounded, self-healing mis-ordering of
    /// eviction PRIORITY, which is the strictly smaller failure — and the pin
    /// escape is what protects anything an operator could not afford to lose.
    pub created_ns: u64,
    /// Excluded from eviction. See the module docs.
    pub pinned: bool,
    /// The capture's PRIMARY cause kind, its eviction class.
    ///
    /// PRIMARY means the cause of the request that STARTED the capture, which is
    /// [`FinishedCapture::causes`](super::trigger::FinishedCapture::causes)' first
    /// entry. A capture can carry up to
    /// [`TriggerPolicy::max_causes`](super::trigger::TriggerPolicy::max_causes)
    /// of them and something has to attribute the bag to one class; the one that
    /// OPENED it is the only choice that does not depend on how many times a
    /// monitor happened to re-evaluate before the window closed.
    ///
    /// `None` is a genuine UNKNOWN, not a default: a capture written by an
    /// older recorder, or one whose marker could not be written, genuinely
    /// has no recorded class. Unknown captures form ONE class of their own —
    /// they are indistinguishable from each other, so treating them as one group
    /// is what the evidence supports, and it keeps a directory of them behaving
    /// exactly as oldest-first did.
    pub cause: Option<TriggerKind>,
}

/// The two caps, whichever binds first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RetentionCaps {
    /// See [`DEFAULT_FLASHBACK_MAX_BYTES`].
    pub max_bytes: u64,
    /// See [`DEFAULT_FLASHBACK_MAX_CAPTURES`].
    pub max_captures: u32,
}

impl Default for RetentionCaps {
    fn default() -> Self {
        Self {
            max_bytes: DEFAULT_FLASHBACK_MAX_BYTES,
            max_captures: DEFAULT_FLASHBACK_MAX_CAPTURES,
        }
    }
}

/// What a sweep should do.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct RetentionPlan {
    /// Names to delete, in the order a caller should delete in.
    ///
    /// Each entry is the oldest candidate of the most populous cause class AT
    /// THAT POINT in the plan, so the sequence is oldest-first
    /// WITHIN a class and interleaves across classes as the flood is drained. On
    /// a diverse directory it is exactly oldest-first.
    ///
    /// An interrupted sweep therefore leaves the newest captures of every class
    /// rather than an arbitrary subset — the property the old strict ordering
    /// bought, now stated per class.
    pub evict: Vec<String>,
    /// Bytes that remain after the plan is carried out.
    pub retained_bytes: u64,
    /// Captures that remain.
    pub retained_count: u32,
    /// The caps are still exceeded and NOTHING further may be evicted, because
    /// everything left is pinned.
    ///
    /// Reported rather than resolved: the alternative is deleting a pin, which is
    /// the one thing the escape exists to forbid. A caller WARNS — an operator who
    /// pinned more than their cap allows has to be told, or the directory grows
    /// silently forever.
    pub pinned_over_cap: bool,
}

/// PURE: decide which captures to evict.
///
/// Rules, in force together:
///
/// 1. **Pinned captures are never candidates.** Not last, not under pressure,
///    never.
/// 2. **Oldest unpinned first**, ties broken by name so the plan does not depend
///    on a directory listing's order.
/// 3. **Both caps apply**, and eviction stops the moment BOTH are satisfied —
///    so a directory over one cap and under the other evicts only what that one
///    cap needs.
///
/// 4. **The candidate is the oldest of the MOST POPULOUS cause class** (decision
///    110-E) — see the module docs. On a diverse directory this is exactly rule 2.
///
/// A cap of zero is honoured LITERALLY here — "keep nothing unpinned" — because
/// this function is TOTAL and must have an answer for every input its own type
/// admits. That is NOT the same as saying an operator can ask for it: the env
/// path ([`resolve_caps`]) REFUSES a zero and says why, so the only way to reach
/// this arm is a caller that constructed [`RetentionCaps`] by hand. The two
/// layers are deliberately different and the doc used to claim only the first,
/// which read as a promise the knob does not keep.
pub fn plan_retention(entries: &[CaptureEntry], caps: RetentionCaps) -> RetentionPlan {
    let mut retained_bytes: u64 = entries.iter().map(|e| e.bytes).sum();
    let mut retained_count: u32 = entries.len() as u32;

    // The candidate order IS the eviction order, and it is derived rather than
    // taken from the caller: a directory listing is unordered on every filesystem
    // this ships on, so sorting here is what makes two robots with the same
    // captures produce the same plan.
    let mut candidates: Vec<&CaptureEntry> = entries.iter().filter(|e| !e.pinned).collect();
    candidates.sort_by(|a, b| {
        a.created_ns
            .cmp(&b.created_ns)
            .then_with(|| a.name.cmp(&b.name))
    });

    let over = |bytes: u64, count: u32| bytes > caps.max_bytes || count > caps.max_captures;

    // Population PER CLASS, over the candidates only — a pinned capture is not a
    // candidate, so it must not make its class look populous and draw evictions
    // onto the siblings it is not protecting.
    let mut population: BTreeMap<Option<TriggerKind>, usize> = BTreeMap::new();
    for candidate in &candidates {
        *population.entry(candidate.cause).or_insert(0) += 1;
    }

    let mut evict = Vec::new();
    // `candidates` is sorted oldest-first, so the FIRST candidate of a class is
    // that class's oldest and the walk below reads populations off one map
    // instead of re-sorting per round.
    let mut remaining: Vec<&CaptureEntry> = candidates;
    while over(retained_bytes, retained_count) {
        // The most populous class, ties broken by whichever holds the globally
        // oldest candidate — which is what makes a diverse directory (every class
        // holding one) degenerate exactly to oldest-first.
        let Some(pos) = remaining
            .iter()
            .enumerate()
            .max_by_key(|(idx, entry)| {
                let pop = population.get(&entry.cause).copied().unwrap_or(0);
                // `max_by_key` keeps the LAST maximum, so the index is negated to
                // make the EARLIEST (oldest) candidate win a tie. Without it the
                // rule would evict the NEWEST capture of a tied class — the
                // dashcam contract inverted inside the class.
                (pop, std::cmp::Reverse(*idx))
            })
            .map(|(idx, _)| idx)
        else {
            break;
        };
        let candidate = remaining.remove(pos);
        if let Some(count) = population.get_mut(&candidate.cause) {
            *count = count.saturating_sub(1);
        }
        evict.push(candidate.name.clone());
        retained_bytes = retained_bytes.saturating_sub(candidate.bytes);
        retained_count = retained_count.saturating_sub(1);
    }

    RetentionPlan {
        evict,
        retained_bytes,
        retained_count,
        pinned_over_cap: over(retained_bytes, retained_count),
    }
}

/// PURE: the caps in force, from the two environment overrides.
///
/// Each returns its default plus an optional COMPLAINT — never a hard failure. An
/// unparseable retention knob must not stop a robot capturing, and must not be
/// silently ignored either; that is the same contract
/// [`resolve_cadence_ms`](super::resolve_cadence_ms) has, through the same helper.
pub fn resolve_caps(
    max_mb: Option<&str>,
    max_captures: Option<&str>,
) -> (RetentionCaps, Vec<String>) {
    let mut complaints = Vec::new();
    let default_mb = DEFAULT_FLASHBACK_MAX_BYTES / (1024 * 1024);

    let (mb, complaint) = super::resolve_positive_override(max_mb, &default_mb, "mebibytes");
    if let Some(c) = complaint {
        complaints.push(with_zero_remedy(max_mb, c));
    }
    // SATURATING: an astronomical ask means "no practical ceiling", which is
    // exactly what an operator typing one means — the same reading
    // `resolve_max_state_bytes` gives its own knob.
    let max_bytes = mb
        .map(|v| v.saturating_mul(1024 * 1024))
        .unwrap_or(DEFAULT_FLASHBACK_MAX_BYTES);

    let (count, complaint) =
        super::resolve_positive_override(max_captures, &DEFAULT_FLASHBACK_MAX_CAPTURES, "captures");
    if let Some(c) = complaint {
        complaints.push(with_zero_remedy(max_captures, c));
    }
    let max_captures = count
        .map(|v| u32::try_from(v).unwrap_or(u32::MAX))
        .unwrap_or(DEFAULT_FLASHBACK_MAX_CAPTURES);

    (
        RetentionCaps {
            max_bytes,
            max_captures,
        },
        complaints,
    )
}

/// PURE: name the thing an operator who typed `0` actually wanted.
///
/// # Why a zero is REFUSED rather than honoured, and why that needs a sentence
///
/// A retention cap of zero asks the robot to keep capturing ~155 MB bags and
/// delete each one immediately — all of the standing cost, none of the benefit —
/// so it is a typo or a misunderstanding essentially every time. Refusing it also
/// keeps this knob consistent with its two siblings
/// ([`FLASHBACK_CADENCE_MS_ENV`](super::FLASHBACK_CADENCE_MS_ENV),
/// [`FLASHBACK_MAX_STATE_MB_ENV`](super::FLASHBACK_MAX_STATE_MB_ENV)), which
/// refuse a zero through the SAME shared parser.
///
/// But a bare "0 is not usable, using the default" leaves that operator with a
/// robot doing the opposite of what they asked and no idea what to type instead —
/// and what they almost certainly wanted, [`FLASHBACK_ENV`](super::FLASHBACK_ENV)
/// `=off`, turns the whole plane off and costs nothing. Naming it is the
/// difference between a refusal and an actionable one (the house rule: loud AND
/// precise).
///
/// Appended by the CALLER rather than pushed into the shared parser, because the
/// remedy is knob-specific: the kill switch is not the answer for a bad cadence.
fn with_zero_remedy(raw: Option<&str>, complaint: String) -> String {
    if raw.map(str::trim) == Some("0") {
        format!(
            "{complaint}. To keep no flashbacks at all, turn the plane off with \
             {}=off rather than capturing and immediately deleting them",
            super::FLASHBACK_ENV
        )
    } else {
        complaint
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A capture with NO recorded class — the class-less shape, which is what the
    /// oldest-first arms are about, so they assert the oldest-first rule
    /// alone (one class ⇒ the class walk degenerates to it).
    fn entry(name: &str, bytes: u64, created_ns: u64, pinned: bool) -> CaptureEntry {
        CaptureEntry {
            name: name.to_string(),
            bytes,
            created_ns,
            pinned,
            cause: None,
        }
    }

    fn caps(max_bytes: u64, max_captures: u32) -> RetentionCaps {
        RetentionCaps {
            max_bytes,
            max_captures,
        }
    }

    /// The shipped numbers, which the docs above reason about.
    #[test]
    fn the_shipped_caps_are_the_documented_ones() {
        let d = RetentionCaps::default();
        assert_eq!(d.max_bytes, 2 * 1024 * 1024 * 1024);
        assert_eq!(d.max_captures, 20);
        assert_eq!(FLASHBACK_MAX_MB_ENV, "CERULION_FLASHBACK_MAX_MB");
        assert_eq!(
            FLASHBACK_MAX_CAPTURES_ENV,
            "CERULION_FLASHBACK_MAX_CAPTURES"
        );
        assert_eq!(FLASHBACK_DIR_ENV, "CERULION_FLASHBACK_DIR");
        assert_eq!(DEFAULT_FLASHBACK_DIR, "recordings/flashbacks");
    }

    /// A directory inside both caps is left alone. The anti-tautology control for
    /// every arm below: without it, a planner that evicts everything passes them.
    #[test]
    fn a_directory_inside_both_caps_evicts_nothing() {
        let entries = vec![entry("a", 10, 1, false), entry("b", 10, 2, false)];
        let plan = plan_retention(&entries, caps(100, 10));
        assert_eq!(plan.evict, Vec::<String>::new());
        assert_eq!((plan.retained_bytes, plan.retained_count), (20, 2));
        assert!(!plan.pinned_over_cap);
    }

    /// The COUNT cap, oldest-first, stopping the moment it is satisfied.
    #[test]
    fn the_count_cap_evicts_the_oldest_and_stops() {
        let entries = vec![
            entry("newest", 1, 30, false),
            entry("oldest", 1, 10, false),
            entry("middle", 1, 20, false),
        ];
        let plan = plan_retention(&entries, caps(u64::MAX, 2));
        assert_eq!(
            plan.evict,
            vec!["oldest"],
            "exactly one over, exactly one out"
        );
        assert_eq!(plan.retained_count, 2);
    }

    /// The BYTE cap, which can evict several — and the two caps compose, each
    /// stopping the loop only when BOTH are satisfied.
    #[test]
    fn the_byte_cap_evicts_until_it_fits_and_composes_with_the_count_cap() {
        let entries = vec![
            entry("a", 100, 10, false),
            entry("b", 100, 20, false),
            entry("c", 100, 30, false),
            entry("d", 100, 40, false),
        ];
        // 400 bytes against a 150-byte cap: evicting stops only when the REMAINDER
        // fits, so three go and 100 stays — not "two, because 400/150 is under 3".
        let plan = plan_retention(&entries, caps(150, 100));
        assert_eq!(plan.evict, vec!["a", "b", "c"]);
        assert_eq!((plan.retained_bytes, plan.retained_count), (100, 1));

        // Now a cap pair where the COUNT is the binding one even though bytes fit.
        let plan = plan_retention(&entries, caps(u64::MAX, 1));
        assert_eq!(plan.evict, vec!["a", "b", "c"]);
        assert_eq!(plan.retained_count, 1);
    }

    /// THE escape: a pinned capture is not a candidate, even when it is the oldest
    /// and even when the directory is far over its cap.
    #[test]
    fn a_pinned_capture_is_never_evicted_even_when_it_is_the_oldest() {
        let entries = vec![
            entry("pinned-oldest", 100, 10, true),
            entry("b", 100, 20, false),
            entry("c", 100, 30, false),
        ];
        // A cap that ONE eviction satisfies, so the choice of victim is the whole
        // observation: the oldest capture is pinned, and the second-oldest goes
        // instead. A planner that merely evicted "oldest first" would take the pin.
        let plan = plan_retention(&entries, caps(250, 100));
        assert_eq!(
            plan.evict,
            vec!["b"],
            "the oldest UNPINNED goes; the pin is not a candidate"
        );
        assert_eq!((plan.retained_bytes, plan.retained_count), (200, 2));
        assert!(!plan.pinned_over_cap, "one eviction was enough");
    }

    /// …and when the pins ALONE exceed the caps, the plan says so rather than
    /// deleting one. This is the state a caller warns about.
    #[test]
    fn pins_alone_over_the_cap_are_reported_and_never_deleted() {
        let entries = vec![
            entry("p1", 100, 10, true),
            entry("p2", 100, 20, true),
            entry("u", 100, 30, false),
        ];
        let plan = plan_retention(&entries, caps(150, 100));
        assert_eq!(
            plan.evict,
            vec!["u"],
            "only the unpinned one is a candidate"
        );
        assert_eq!(plan.retained_bytes, 200);
        assert!(
            plan.pinned_over_cap,
            "over the cap with nothing left to evict — reported, never resolved"
        );
    }

    /// A single capture larger than the whole cap: everything else goes, it stays,
    /// and the plan reports that the cap is still exceeded. Evicting it would
    /// leave the directory empty AND still not satisfy the cap.
    #[test]
    fn one_capture_larger_than_the_cap_leaves_an_honest_over_cap_report() {
        let entries = vec![
            entry("huge", 1_000, 10, false),
            entry("small", 10, 20, false),
        ];
        let plan = plan_retention(&entries, caps(100, 100));
        assert_eq!(
            plan.evict,
            vec!["huge"],
            "oldest-first, and it is the oldest"
        );
        assert_eq!(plan.retained_bytes, 10);
        assert!(!plan.pinned_over_cap);
    }

    /// A capture written AFTER a reboot must not be evicted before one written
    /// before it.
    ///
    /// This is the arm that fails if `created_ns` is ever filled from a MONOTONIC
    /// reading: such a clock restarts near zero at boot, so the post-reboot
    /// capture would carry the SMALLER stamp, sort first, and be evicted while
    /// the stale pre-reboot one survives — the dashcam contract inverted. The
    /// planner cannot see where the number came from, so what is pinned here is
    /// the CONSEQUENCE: given a durable stamp the older capture goes, and the
    /// values are the shapes a real reboot produces (a large uptime-era stamp
    /// against a small post-boot one).
    #[test]
    fn a_post_reboot_capture_outlives_an_older_one_when_the_stamp_is_durable() {
        // Durable (wall-derived): the pre-reboot capture is genuinely older.
        let durable = vec![
            entry("before-reboot", 50, 1_700_000_000_000_000_000, false),
            entry("after-reboot", 50, 1_700_000_600_000_000_000, false),
        ];
        assert_eq!(
            plan_retention(&durable, caps(50, 100)).evict,
            vec!["before-reboot"],
            "the older capture is the candidate"
        );
        // The SAME two captures stamped from a monotonic clock — 3 h of uptime,
        // then a reboot — invert, which is what the field's doc forbids.
        let monotonic = vec![
            entry("before-reboot", 50, 10_800_000_000_000, false),
            entry("after-reboot", 50, 2_000_000_000, false),
        ];
        assert_eq!(
            plan_retention(&monotonic, caps(50, 100)).evict,
            vec!["after-reboot"],
            "documented consequence of a monotonic stamp: the NEWEST is evicted"
        );
    }

    /// Ties are broken by NAME, so two captures stamped in the same nanosecond
    /// produce one plan rather than whichever the filesystem listed first.
    #[test]
    fn a_timestamp_tie_is_broken_by_name_so_the_plan_is_deterministic() {
        let forward = vec![
            entry("aaa", 10, 5, false),
            entry("bbb", 10, 5, false),
            entry("ccc", 10, 5, false),
        ];
        let mut reversed = forward.clone();
        reversed.reverse();
        let oracle = vec!["aaa".to_string()];
        assert_eq!(plan_retention(&forward, caps(25, 100)).evict, oracle);
        assert_eq!(
            plan_retention(&reversed, caps(25, 100)).evict,
            oracle,
            "listing order must not change the plan"
        );
    }

    /// A zero cap keeps nothing unpinned — honoured literally, and the boundary
    /// that proves the comparison is `>` rather than `>=` (a directory holding
    /// EXACTLY the cap is not over it).
    #[test]
    fn the_caps_are_thresholds_pinned_on_both_sides() {
        let entries = vec![entry("a", 50, 1, false), entry("b", 50, 2, false)];
        // Exactly at the byte cap: nothing goes.
        assert!(plan_retention(&entries, caps(100, 100)).evict.is_empty());
        // One byte under: the oldest goes.
        assert_eq!(plan_retention(&entries, caps(99, 100)).evict, vec!["a"]);
        // Exactly at the count cap: nothing goes.
        assert!(plan_retention(&entries, caps(u64::MAX, 2)).evict.is_empty());
        assert_eq!(plan_retention(&entries, caps(u64::MAX, 1)).evict, vec!["a"]);
        // Zero: everything unpinned goes.
        let plan = plan_retention(&entries, caps(0, 0));
        assert_eq!(plan.evict, vec!["a", "b"]);
        assert_eq!((plan.retained_bytes, plan.retained_count), (0, 0));
        assert!(!plan.pinned_over_cap);
    }

    /// An empty directory plans nothing and claims nothing.
    #[test]
    fn an_empty_directory_plans_nothing() {
        assert_eq!(plan_retention(&[], caps(0, 0)), RetentionPlan::default());
    }

    /// The knobs are honoured or complained about — never silently, never fatally.
    #[test]
    fn a_retention_override_is_honoured_or_complained_about() {
        let (c, complaints) = resolve_caps(None, None);
        assert_eq!(c, RetentionCaps::default());
        assert!(complaints.is_empty());

        let (c, complaints) = resolve_caps(Some("512"), Some("5"));
        assert_eq!(c.max_bytes, 512 * 1024 * 1024, "MEBIBYTES land in BYTES");
        assert_eq!(c.max_captures, 5);
        assert!(complaints.is_empty());

        // Both degrade to their defaults WITH the offending text.
        let (c, complaints) = resolve_caps(Some("soon"), Some("0"));
        assert_eq!(c, RetentionCaps::default());
        assert_eq!(complaints.len(), 2, "each bad knob complains once");
        assert!(complaints[0].contains("soon"), "{complaints:?}");
        assert!(complaints[1].contains("0 captures"), "{complaints:?}");

        // An astronomical ask is "no practical ceiling", not its own opposite —
        // the sentinel lesson from the plane's other knobs, applied to this one too.
        let (c, _) = resolve_caps(Some("18446744073709551615"), Some("4294967296"));
        assert_eq!(c.max_bytes, u64::MAX);
        assert_eq!(c.max_captures, u32::MAX);
    }

    /// An explicit `0` is REFUSED, LOUDLY, and the refusal names what the operator
    /// almost certainly wanted.
    ///
    /// The two layers genuinely differ and the doc now says so: [`plan_retention`]
    /// honours a zero literally (it is total), while the ENV path refuses it —
    /// keeping a retention knob consistent with its two siblings, which refuse a
    /// zero through the same shared parser. What must never happen is the middle
    /// case: an explicit `0` SILENTLY becoming the default, leaving a robot doing
    /// the opposite of what was typed with nothing said.
    ///
    /// Asserted on BOTH knobs, because they are two call sites and only one of
    /// them was exercised by the arm above.
    #[test]
    fn an_explicit_zero_cap_is_refused_loudly_and_names_the_kill_switch() {
        for (mb, captures, unit) in [
            (Some("0"), None, "mebibytes"),
            (None, Some("0"), "captures"),
        ] {
            let (caps, complaints) = resolve_caps(mb, captures);
            assert_eq!(
                caps,
                RetentionCaps::default(),
                "a refused zero falls back to the default"
            );
            assert_eq!(complaints.len(), 1, "…and it is NOT silent: {complaints:?}");
            let complaint = &complaints[0];
            // It says what was refused, in the operator's own unit…
            assert!(
                complaint.contains(&format!("0 {unit}")),
                "the refusal must quote what was typed: {complaint}"
            );
            // …and names the switch that really does mean "keep nothing".
            assert!(
                complaint.contains("CERULION_FLASHBACK=off"),
                "the refusal must be ACTIONABLE: {complaint}"
            );
        }
        // ANTI-TAUTOLOGY: the kill-switch remedy is scoped to a ZERO. An
        // unparseable value is a different mistake with a different fix, and
        // stapling "turn the feature off" onto every complaint would make the
        // sentence meaningless.
        let (_, complaints) = resolve_caps(Some("soon"), None);
        assert_eq!(complaints.len(), 1);
        assert!(
            !complaints[0].contains("CERULION_FLASHBACK=off"),
            "a typo is not a request to disable the plane: {}",
            complaints[0]
        );
        // …and a HEALTHY value complains about nothing at all.
        assert!(resolve_caps(Some("512"), Some("5")).1.is_empty());
    }

    /// The two layers' documented split, pinned as BEHAVIOUR rather than prose.
    ///
    /// `plan_retention` is TOTAL and honours a zero; `resolve_caps` refuses one.
    /// The doc said only the first, which read as a promise the knob does not
    /// keep — so both halves are asserted here, in one body, where they cannot
    /// drift apart unnoticed.
    #[test]
    fn a_zero_is_honoured_by_the_pure_planner_and_refused_by_the_env_knob() {
        let entries = vec![entry("a", 50, 1, false), entry("b", 50, 2, true)];
        // HAND-CONSTRUCTED caps: total, literal, and it evicts the unpinned one
        // while the pin survives (the escape outranks even a zero cap).
        let plan = plan_retention(&entries, caps(0, 0));
        assert_eq!(plan.evict, vec!["a"]);
        assert_eq!(plan.retained_count, 1, "the pin is never a candidate");
        assert!(plan.pinned_over_cap, "…and the plan says it is still over");
        // The ENV knob refuses the same number.
        assert_eq!(
            resolve_caps(Some("0"), Some("0")).0,
            RetentionCaps::default()
        );
    }

    // ---------------------------------------------------------------------
    // Diversity-preserving eviction.
    // ---------------------------------------------------------------------

    fn caused(name: &str, bytes: u64, created_ns: u64, cause: Option<TriggerKind>) -> CaptureEntry {
        CaptureEntry {
            name: name.to_string(),
            bytes,
            created_ns,
            pinned: false,
            cause,
        }
    }

    /// THE RULE: a chatty cause cannot rotate the one real incident out.
    ///
    /// Six captures, five of them one noisy monitor class and one a worker death,
    /// with the WORKER DEATH THE OLDEST — so strict oldest-first evicts exactly
    /// the capture an operator kept the directory for. The oracle is the EVICTED
    /// LIST, not merely "the incident survived": a rule that evicted everything
    /// except the incident would satisfy a survival check while destroying the
    /// context around it.
    #[test]
    fn a_chatty_cause_cannot_evict_the_one_capture_of_another() {
        let monitor = Some(TriggerKind::MonitorVerdict);
        let fault = Some(TriggerKind::ProcessFault);
        let entries = vec![
            caused("worker_death", 10, 100, fault),
            caused("noise_a", 10, 200, monitor),
            caused("noise_b", 10, 300, monitor),
            caused("noise_c", 10, 400, monitor),
            caused("noise_d", 10, 500, monitor),
            caused("noise_e", 10, 600, monitor),
        ];
        // Room for four: two must go.
        let plan = plan_retention(
            &entries,
            RetentionCaps {
                max_bytes: 40,
                max_captures: 100,
            },
        );
        assert_eq!(
            plan.evict,
            vec!["noise_a".to_string(), "noise_b".to_string()],
            "the two oldest of the MOST POPULOUS class, never the global oldest"
        );
        assert_eq!(plan.retained_count, 4);
        assert_eq!(plan.retained_bytes, 40);
        assert!(!plan.pinned_over_cap);
    }

    /// …and on a DIVERSE directory the rule degenerates EXACTLY to oldest-first,
    /// so nothing an operator already understands changes until a flood starts.
    ///
    /// This is the arm that stops the fix from being a behaviour change nobody
    /// asked for: every class holds one capture, so every class is equally
    /// populous and the tie rule has to fall through to the global oldest.
    #[test]
    fn a_diverse_directory_evicts_oldest_first_exactly_as_before() {
        let entries = vec![
            caused("a", 10, 100, Some(TriggerKind::ProcessFault)),
            caused("b", 10, 200, Some(TriggerKind::EStop)),
            caused("c", 10, 300, Some(TriggerKind::Manual)),
            caused("d", 10, 400, Some(TriggerKind::RunVanished)),
        ];
        let plan = plan_retention(
            &entries,
            RetentionCaps {
                max_bytes: 20,
                max_captures: 100,
            },
        );
        assert_eq!(plan.evict, vec!["a".to_string(), "b".to_string()]);
    }

    /// UNKNOWN captures are ONE class, and a directory of them behaves exactly as
    /// oldest-first did.
    ///
    /// The back-compat arm: every capture written before the cause marker existed carries no
    /// marker, so a robot upgrading in place has a directory of `None`s and must
    /// see no change at all.
    #[test]
    fn captures_with_no_recorded_class_form_one_class_and_evict_oldest_first() {
        let entries = vec![
            caused("a", 10, 100, None),
            caused("b", 10, 200, None),
            caused("c", 10, 300, None),
        ];
        let plan = plan_retention(
            &entries,
            RetentionCaps {
                max_bytes: 10,
                max_captures: 100,
            },
        );
        assert_eq!(plan.evict, vec!["a".to_string(), "b".to_string()]);
        // …and UNKNOWN is a class of its OWN, never folded into a known one: a
        // class-less capture is not evidence about any kind.
        let mixed = vec![
            caused("old_unknown", 10, 100, None),
            caused("m1", 10, 200, Some(TriggerKind::MonitorVerdict)),
            caused("m2", 10, 300, Some(TriggerKind::MonitorVerdict)),
        ];
        let plan = plan_retention(
            &mixed,
            RetentionCaps {
                max_bytes: 20,
                max_captures: 100,
            },
        );
        assert_eq!(
            plan.evict,
            vec!["m1".to_string()],
            "the populous MonitorVerdict class yields, not the lone unknown"
        );
    }

    /// A PIN does not make its class look populous.
    ///
    /// Pinned captures are not candidates, so counting them would draw evictions
    /// onto the siblings a pin is not protecting — the escape working against the
    /// captures around it.
    #[test]
    fn a_pinned_capture_does_not_make_its_class_a_target() {
        let monitor = Some(TriggerKind::MonitorVerdict);
        let fault = Some(TriggerKind::ProcessFault);
        let mut entries = vec![
            caused("pinned_m1", 10, 100, monitor),
            caused("pinned_m2", 10, 150, monitor),
            caused("pinned_m3", 10, 175, monitor),
            caused("live_monitor", 10, 200, monitor),
            caused("fault_a", 10, 300, fault),
            caused("fault_b", 10, 400, fault),
        ];
        for e in entries.iter_mut().take(3) {
            e.pinned = true;
        }
        let plan = plan_retention(
            &entries,
            RetentionCaps {
                max_bytes: 50,
                max_captures: 100,
            },
        );
        // Candidates are 1 monitor + 2 faults, so ProcessFault is the populous
        // class. Counting the three pins would have made MonitorVerdict populous
        // (4 vs 2) and evicted the one live monitor capture instead.
        assert_eq!(plan.evict, vec!["fault_a".to_string()]);
    }

    /// Within one class the plan is still OLDEST-first, and the sequence stays
    /// interleaved as the flood drains.
    ///
    /// Pinned as the full EVICTION SEQUENCE rather than a set, because the
    /// ordering is a contract an interrupted sweep depends on — a caller that
    /// stops halfway must be left with the newest of every class.
    #[test]
    fn the_eviction_sequence_drains_the_flood_and_stays_oldest_first_per_class() {
        let monitor = Some(TriggerKind::MonitorVerdict);
        let fault = Some(TriggerKind::ProcessFault);
        let entries = vec![
            caused("m1", 10, 100, monitor),
            caused("m2", 10, 200, monitor),
            caused("m3", 10, 300, monitor),
            caused("f1", 10, 400, fault),
            caused("f2", 10, 500, fault),
        ];
        let plan = plan_retention(
            &entries,
            RetentionCaps {
                max_bytes: 0,
                max_captures: 100,
            },
        );
        assert_eq!(
            plan.evict,
            vec![
                // MonitorVerdict is 3 vs 2, oldest first…
                "m1".to_string(),
                "m2".to_string(),
                // …now 1 vs 2, so the fault class yields…
                "f1".to_string(),
                // …now 1 vs 1, tie falls through to the global oldest…
                "m3".to_string(),
                "f2".to_string(),
            ],
            "the plan drains the flood and only then touches the diverse remainder"
        );
        assert_eq!(plan.retained_count, 0);
    }

    /// The NAME tie-break survives, within a class.
    ///
    /// Two captures of one class sharing a timestamp must still produce one plan
    /// on any filesystem's listing order — the property rule 2 already had, now
    /// asserted under the class walk.
    #[test]
    fn two_captures_of_one_class_sharing_a_stamp_still_break_by_name() {
        let k = Some(TriggerKind::Manual);
        let forward = vec![
            caused("aaa", 10, 100, k),
            caused("bbb", 10, 100, k),
            caused("ccc", 10, 100, k),
        ];
        let mut reversed = forward.clone();
        reversed.reverse();
        let caps = RetentionCaps {
            max_bytes: 10,
            max_captures: 100,
        };
        assert_eq!(
            plan_retention(&forward, caps).evict,
            vec!["aaa".to_string(), "bbb".to_string()]
        );
        assert_eq!(
            plan_retention(&reversed, caps).evict,
            plan_retention(&forward, caps).evict,
            "a directory listing's order must not change the plan"
        );
    }
}
