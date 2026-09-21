// SPDX-License-Identifier: AGPL-3.0-only
//! The LOUD, actionable diagnosis for a wire `schema_hash` no
//! local schema resolves — and the flood-latched reporting layer over it.
//!
//! # The defect
//!
//! Cerulion resolves message layout BY HASH.
//! [`FrameWalker::walk_by_hash`](super::FrameWalker::walk_by_hash) refuses an
//! unrecognized `schema_hash` at the hash gate, **before framing is ever
//! consulted** — so a frame whose type this build does not hold, or holds under
//! a DIFFERENT definition, is dropped. Every consumer of that refusal treated it
//! as "nothing to render": the topic drew nothing and the user was told nothing.
//!
//! That is the exact silent-death class the upstream-drift gate exists to close, and the
//! corpus-side fixes do not close it on their own:
//!
//! * ** /3** moved 48 vendored types onto the qualified spelling a
//!   stock ROS 2 robot uses. The `schema_hash` IS the compatibility token and
//!   nothing on the wire signals which spelling a peer carries, so a desk and a
//!   robot deployed across that change compute different hashes for the same
//!   message. Unlike its skew — which degrades to a loud
//!   `NestedArrayOpaque` text fallback — a hash skew was **silent**.
//! * The **Jazzy `control_msgs/PidState` / `SteeringControllerStatus` residual**
//!   (see `crates/native_ros2_messages/tests/upstream_drift_test.rs`) is a permanent,
//!   known field-NAME divergence against a stock Jazzy arm. The upstream-drift
//!   gate cannot flag it by design; a live operator hits it as silence.
//! * A robot type **never compiled on this desk** — the "any ROS 2 robot we have
//!   never seen" story — lands here too, and its remedy is completely different
//!   (acquire the definition, don't rebuild).
//!
//! Three causes, three remedies, one indistinguishable silence. This module
//! makes the refusal say WHICH one it is.
//!
//! # The candidate check (why a NAME turns a hash into an instruction)
//!
//! The wire carries a hash and nothing else, so a refusal alone can only ever
//! say "0x…". But on every path that matters a type NAME is available from a
//! SECOND source — a remote topic's `CatalogEntry::schema_name`, an attach's
//! pinned type, a bag's schema catalog — and given a name,
//! [`FrameWalker::schema_hash_for`](super::FrameWalker::schema_hash_for) is one
//! `O(log n)` lookup. That single lookup splits the silence in two:
//!
//! * we hold that name at a **different** hash ⇒ the two builds disagree about
//!   the message DEFINITION ⇒ **rebuild the stale side**;
//! * we hold no schema for that name at all ⇒ **acquire the definition**.
//!
//! [`diagnose_unknown_hash`] is that decision, kept PURE (it takes the looked-up
//! hash, not the walker) so it is oracle-testable over hand vectors;
//! [`diagnose_unknown_hash_with_walker`] is the one-line convenience call sites
//! use.
//!
//! # Placement
//!
//! Here in `codegen` rather than beside the latch in `transport`, because the
//! condition is a DECODE fact about the walker's schema set — the module needs
//! [`FrameWalker`], and no transport type appears in it. The
//! suppression policy is imported from
//! [`FailureRegimeLatch`]
//! — the repo's ONE flood-suppression state machine — exactly as
//! `transport::frame_drop_latch` and `rmw_cerulion`'s decode/publish reporters
//! do, so the running total logs under the SAME `total_failures=` key
//! and an open regime re-announces itself at each decade of that total.
//!
//! # Why the latch, and why recovery is real here
//!
//! A hash skew fails EVERY frame until somebody redeploys, so a bare per-frame
//! `warn!` is the disk-fill class (~100 lines/s on a 100 Hz topic). But
//! the earlier sink went to the other extreme — warn ONCE per distinct hash,
//! forever, with no counter and no re-announcement — so an operator who missed
//! the line had no way back to it and no way to ask "how bad is this?".
//!
//! Recovery is genuinely observable: the viz daemon SWAPS its walker when a
//! robot's `.msg` closure is seeded, so a topic that could not
//! be decoded a second ago can start decoding. [`report_schema_hash_resolved`]
//! reports that once and re-arms.

use crate::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

use super::FrameWalker;

/// WHICH observer is speaking. Logged as `vantage=` on every line.
///
/// Two reporters can see the SAME undecodable topic in ONE process: inside
/// `cerulion-vizd` the render worker runs in-process with the resolver and both
/// are handed the same frames. They do NOT have the same information — the
/// resolver holds the catalog / attach-pinned type name and can say "version
/// skew", while the render worker receives only frames plus a `FrameWalker` and
/// can only ever say "unidentified".
///
/// Without this field those two lines read as one condition contradicting
/// itself. With it they read as what they are: two observers at different
/// vantages, the NAMED one authoritative. It is also why the `Unidentified`
/// wording claims only what the reporting vantage knows — a reporter that cannot
/// name a type must not assert that nothing can.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosisVantage {
    /// The viz render path (`cerulion_viz`'s sink), which holds only frames and
    /// a walker.
    Render,
    /// A daemon's schema-RESOLUTION path, which can hold an out-of-band type
    /// name (a catalog entry, an attach pin) and so can reach a NAMED verdict.
    Resolver,
    /// A one-shot CLI observer (`cerulion topic echo`).
    Cli,
}

impl DiagnosisVantage {
    /// The stable `vantage=` token.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Render => "render",
            Self::Resolver => "resolver",
            Self::Cli => "cli",
        }
    }
}

/// A type NAME for the refused frame, sourced from OUTSIDE the wire (a remote
/// topic's catalog entry, an attach's pinned type, a bag's schema catalog),
/// together with the hash the LOCAL schema set gives that name.
///
/// `local_hash` is `None` when the local set holds no schema under `name` at
/// all — which is itself the diagnosis (the type was never compiled here).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnknownHashCandidate<'a> {
    /// The qualified ROS type name (`pkg/Type`) something OTHER than the wire
    /// says this topic carries.
    pub name: &'a str,
    /// The `schema_hash` the LOCAL schema set computes for `name`, or `None`
    /// when it holds no schema under that name.
    pub local_hash: Option<u64>,
}

/// Why a frame's `schema_hash` resolved to no local schema — the closed set of
/// answers, each with a DIFFERENT remedy.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UnknownHashDiagnosis {
    /// We hold a schema under this exact type name, at a DIFFERENT hash: the
    /// two builds disagree about the message definition. Remedy: rebuild and
    /// redeploy the stale side.
    VersionSkew {
        /// The qualified type name both ends agree the topic carries.
        name: String,
        /// The hash THIS build computes for that name.
        local_hash: u64,
        /// The hash the frame actually carried.
        wire_hash: u64,
    },
    /// The type is NAMED but this build holds no schema for it at all. Remedy:
    /// acquire the definition (`cerulion ros2 attach`, a bag's recorded schema).
    TypeNotCompiled {
        /// The qualified type name the catalog / attach pin supplied.
        name: String,
        /// The hash the frame carried.
        wire_hash: u64,
    },
    /// Nothing but the hash: no candidate name was available, or the one that
    /// was supplied carried no discriminating evidence (see
    /// [`diagnose_unknown_hash`]).
    Unidentified {
        /// The hash the frame carried.
        wire_hash: u64,
    },
}

impl UnknownHashDiagnosis {
    /// The stable machine token for this diagnosis — the `kind=` log field AND
    /// the wire `reason` a viz client renders a row's state from.
    ///
    /// A CLOSED set of snake_case tokens, deliberately stable across releases:
    /// a client switches on it, so renaming one is a wire change.
    pub const fn reason_code(&self) -> &'static str {
        match self {
            Self::VersionSkew { .. } => "schema_version_skew",
            Self::TypeNotCompiled { .. } => "schema_not_compiled",
            Self::Unidentified { .. } => "schema_unidentified",
        }
    }

    /// The hash the frame carried — present in every arm, since it is the one
    /// fact the wire always supplies.
    pub const fn wire_hash(&self) -> u64 {
        match self {
            Self::VersionSkew { wire_hash, .. }
            | Self::TypeNotCompiled { wire_hash, .. }
            | Self::Unidentified { wire_hash } => *wire_hash,
        }
    }

    /// The candidate type name, when one was available.
    pub fn schema_name(&self) -> Option<&str> {
        match self {
            Self::VersionSkew { name, .. } | Self::TypeNotCompiled { name, .. } => Some(name),
            Self::Unidentified { .. } => None,
        }
    }

    /// The hash THIS build computes for [`Self::schema_name`] — `Some` only on
    /// a version skew, where the PAIR is the diagnosis.
    pub const fn local_hash(&self) -> Option<u64> {
        match self {
            Self::VersionSkew { local_hash, .. } => Some(*local_hash),
            Self::TypeNotCompiled { .. } | Self::Unidentified { .. } => None,
        }
    }

    /// One self-contained human sentence: what went wrong, and what to DO.
    ///
    /// This is the string the viz daemon puts on the wire for a topic's
    /// undecodable row, so a non-technical user reading a Studio sidebar gets
    /// the same instruction an operator reading the log gets. It repeats the
    /// hashes rather than assuming the reader also has the structured fields.
    pub fn detail(&self) -> String {
        match self {
            Self::VersionSkew {
                name,
                local_hash,
                wire_hash,
            } => format!(
                "schema version skew: this build holds `{name}` and hashes it to \
                 0x{local_hash:016X}, but the wire carries 0x{wire_hash:016X} — the two \
                 ends disagree about that message's definition (a field added, removed, \
                 retyped or renamed). Rebuild and redeploy whichever side is stale so \
                 both carry the same schemas."
            ),
            Self::TypeNotCompiled { name, wire_hash } => format!(
                "schema not compiled here: the wire carries 0x{wire_hash:016X} for type \
                 `{name}`, and this build holds no schema under that name. Acquire the \
                 producer's definition — `cerulion ros2 attach` materializes the robot's \
                 `.msg` closure under `schemas/`, and a recorded bag carries its own \
                 schema text — then re-run."
            ),
            Self::Unidentified { wire_hash } => format!(
                "schema unidentified: no schema in this build's set matches the wire hash \
                 0x{wire_hash:016X}, and NOTHING AVAILABLE AT THIS VANTAGE names the type \
                 (the wire carries a hash and no name). Run `cerulion topic info` against \
                 this topic to name it, or acquire the producer's `.msg` closure \
                 (`cerulion ros2 attach`) and re-run."
            ),
        }
    }
}

/// Classify an unresolved wire hash from the hash and an optional out-of-band
/// candidate name. PURE — the caller does the `name → local hash` lookup, so
/// this decision is oracle-testable over hand vectors.
///
/// # The contradictory input, and why it degrades rather than lies
///
/// A caller reaches this only because a walker REFUSED `wire_hash`, which means
/// no name in that walker's set hashes to it. A candidate whose `local_hash`
/// nonetheless EQUALS `wire_hash` therefore contradicts its own premise — the
/// two facts came from different sets (a stale snapshot, a walker swapped in
/// between). That name is not evidence of a skew, so it is not reported as one:
/// the diagnosis degrades to [`UnknownHashDiagnosis::Unidentified`], which
/// claims only what the hash itself supports.
pub fn diagnose_unknown_hash(
    wire_hash: u64,
    candidate: Option<UnknownHashCandidate<'_>>,
) -> UnknownHashDiagnosis {
    match candidate {
        Some(UnknownHashCandidate {
            name,
            local_hash: Some(local),
        }) if local != wire_hash => UnknownHashDiagnosis::VersionSkew {
            name: name.to_string(),
            local_hash: local,
            wire_hash,
        },
        // `local == wire_hash`: the premise is contradicted — see the doc above.
        Some(UnknownHashCandidate {
            local_hash: Some(_),
            ..
        }) => UnknownHashDiagnosis::Unidentified { wire_hash },
        Some(UnknownHashCandidate {
            name,
            local_hash: None,
        }) => UnknownHashDiagnosis::TypeNotCompiled {
            name: name.to_string(),
            wire_hash,
        },
        None => UnknownHashDiagnosis::Unidentified { wire_hash },
    }
}

/// [`diagnose_unknown_hash`] with the `name → local hash` lookup done against
/// `walker` — the one-line form call sites use.
///
/// `candidate_name` is whatever OUT-OF-BAND source the caller has (a catalog
/// entry's `schema_name`, an attach's pinned type); `None` when it has none,
/// which is accurate rather than a guess.
pub fn diagnose_unknown_hash_with_walker(
    walker: &FrameWalker,
    wire_hash: u64,
    candidate_name: Option<&str>,
) -> UnknownHashDiagnosis {
    let candidate = candidate_name.map(|name| UnknownHashCandidate {
        name,
        local_hash: walker.schema_hash_for(name),
    });
    diagnose_unknown_hash(wire_hash, candidate)
}

/// Expand one decision under a fixed extra-field list. `tracing` field keys must
/// be literals, so the per-variant field set cannot be a runtime value — the
/// reporter matches on the diagnosis and invokes this once per variant, which
/// keeps each message text in exactly one place.
macro_rules! emit_unknown_hash {
    (
        $topic:expr, $vantage:expr, $code:expr, $hash:expr, $total:expr, $decision:expr,
        $loud:expr, $still:expr, [$($extra:tt)*]
    ) => {
        match $decision {
            RegimeDecision::Loud => tracing::warn!(
                topic = %$topic,
                vantage = %$vantage,
                kind = %$code,
                schema_hash = format_args!("0x{:016X}", $hash),
                $($extra)*
                total_failures = $total,
                $loud
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                topic = %$topic,
                vantage = %$vantage,
                kind = %$code,
                schema_hash = format_args!("0x{:016X}", $hash),
                $($extra)*
                suppressed,
                total_failures = total,
                $still
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                topic = %$topic,
                vantage = %$vantage,
                kind = %$code,
                schema_hash = format_args!("0x{:016X}", $hash),
                $($extra)*
                suppressed,
                total_failures = $total,
                "undecodable frame suppressed (regime still open)"
            ),
        }
    };
}

/// Report ONE frame dropped because its `schema_hash` resolved to no local
/// schema. Loud on the first of a regime and at each decade of the running
/// total, `debug!` in between.
///
/// `topic` is logged under `topic=` — operators grep by key, and this condition
/// is always about a topic (there is no service twin to discriminate against, so
/// unlike [`FrameDropSite`](crate::transport::frame_drop_latch::FrameDropSite)
/// there is deliberately no site enum here; a one-variant enum would
/// discriminate nothing). `kind=` carries
/// [`UnknownHashDiagnosis::reason_code`], so one grep separates a version skew
/// from a missing type.
pub fn report_unknown_schema_hash(
    latch: &mut FailureRegimeLatch,
    vantage: DiagnosisVantage,
    topic: &str,
    diagnosis: &UnknownHashDiagnosis,
) {
    let decision = latch.on_failure();
    let total = latch.total_failures();
    let code = diagnosis.reason_code();
    let hash = diagnosis.wire_hash();
    let vantage = vantage.as_str();
    match diagnosis {
        UnknownHashDiagnosis::VersionSkew {
            name, local_hash, ..
        } => emit_unknown_hash!(
            topic,
            vantage,
            code,
            hash,
            total,
            decision,
            "dropping a frame this build cannot decode — it holds this topic's \
             type at a DIFFERENT schema hash, so the two ends disagree about the message \
             definition. Rebuild and redeploy whichever side is stale so both carry the \
             same schemas. Repeats are suppressed to debug until a decodable frame arrives.",
            "STILL dropping every frame on this topic — the two ends' schema \
             hashes for this type still disagree and the running total has crossed \
             another decade. Rebuild and redeploy whichever side is stale.",
            [
                schema = %name,
                local_schema_hash = format_args!("0x{:016X}", local_hash),
            ]
        ),
        UnknownHashDiagnosis::TypeNotCompiled { name, .. } => emit_unknown_hash!(
            topic,
            vantage,
            code,
            hash,
            total,
            decision,
            "dropping a frame this build cannot decode — its type is not \
             compiled here, so no local schema matches the wire hash. Acquire the \
             producer's definition (`cerulion ros2 attach` materializes the robot's `.msg` \
             closure under `schemas/`; a recorded bag carries its own schema text) and \
             re-run. Repeats are suppressed to debug until a decodable frame arrives.",
            "STILL dropping every frame on this topic — its type is still not \
             compiled here and the running total has crossed another decade. Acquire the \
             producer's definition and re-run.",
            [schema = %name,]
        ),
        UnknownHashDiagnosis::Unidentified { .. } => emit_unknown_hash!(
            topic,
            vantage,
            code,
            hash,
            total,
            decision,
            "dropping a frame this build cannot decode — no local schema matches \
             the wire hash and nothing names its type. Run `cerulion topic info` against \
             this topic to name it, or acquire the producer's `.msg` closure (`cerulion \
             ros2 attach`) and re-run. Repeats are suppressed to debug until a decodable \
             frame arrives.",
            "STILL dropping every frame on this topic — no local schema matches \
             the wire hash and the running total has crossed another decade. Name the \
             type with `cerulion topic info`, or acquire its `.msg` closure.",
            []
        ),
    }
}

/// Report a frame that DID resolve, closing any open undecodable regime.
///
/// Recovery is real, not theoretical: the viz daemon swaps its walker whenever a
/// robot's `.msg` closure is seeded, so a topic that could not be decoded a
/// moment ago starts decoding. Emits one `info!` iff the closed regime actually
/// suppressed something; a healthy consumer pays one predictable branch.
pub fn report_schema_hash_resolved(
    latch: &mut FailureRegimeLatch,
    vantage: DiagnosisVantage,
    topic: &str,
) {
    if let Some(suppressed) = latch.on_success() {
        tracing::info!(
            topic = %topic,
            vantage = %vantage.as_str(),
            suppressed_count = suppressed,
            total_failures = latch.total_failures(),
            "frames on this topic decode again"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::debug_lines_expected;
    use tracing_test::traced_test;

    const WIRE: u64 = 0x0102_0304_0506_0708;
    const LOCAL: u64 = 0x1111_2222_3333_4444;
    const NAME: &str = "visualization_msgs/Marker";

    // ---- the pure classifier, against hand vectors --------------------------

    #[test]
    fn a_name_we_hold_at_a_different_hash_is_a_version_skew() {
        let d = diagnose_unknown_hash(
            WIRE,
            Some(UnknownHashCandidate {
                name: NAME,
                local_hash: Some(LOCAL),
            }),
        );
        assert_eq!(
            d,
            UnknownHashDiagnosis::VersionSkew {
                name: NAME.to_string(),
                local_hash: LOCAL,
                wire_hash: WIRE,
            }
        );
        assert_eq!(d.reason_code(), "schema_version_skew");
        assert_eq!(d.schema_name(), Some(NAME));
        assert_eq!(d.local_hash(), Some(LOCAL));
        assert_eq!(d.wire_hash(), WIRE);
        // The remedy is REBUILD, and BOTH hashes are in the sentence — the pair
        // is what tells an operator which end is stale.
        let detail = d.detail();
        assert!(detail.contains("Rebuild and redeploy"), "{detail}");
        assert!(detail.contains("0x1111222233334444"), "{detail}");
        assert!(detail.contains("0x0102030405060708"), "{detail}");
        assert!(detail.contains(NAME), "{detail}");
    }

    #[test]
    fn a_named_type_we_hold_no_schema_for_is_not_compiled_here() {
        let d = diagnose_unknown_hash(
            WIRE,
            Some(UnknownHashCandidate {
                name: NAME,
                local_hash: None,
            }),
        );
        assert_eq!(
            d,
            UnknownHashDiagnosis::TypeNotCompiled {
                name: NAME.to_string(),
                wire_hash: WIRE,
            }
        );
        assert_eq!(d.reason_code(), "schema_not_compiled");
        assert_eq!(d.schema_name(), Some(NAME));
        // No local hash exists, so none is claimed.
        assert_eq!(d.local_hash(), None);
        // The remedy is ACQUIRE, never rebuild — a rebuild of a type this build
        // never had cannot help, and sending an operator to do it is the wrong
        // instruction, not merely a vague one.
        let detail = d.detail();
        assert!(detail.contains("cerulion ros2 attach"), "{detail}");
        assert!(!detail.contains("Rebuild"), "{detail}");
    }

    #[test]
    fn no_candidate_name_leaves_the_hash_unidentified() {
        let d = diagnose_unknown_hash(WIRE, None);
        assert_eq!(d, UnknownHashDiagnosis::Unidentified { wire_hash: WIRE });
        assert_eq!(d.reason_code(), "schema_unidentified");
        assert_eq!(d.schema_name(), None);
        assert_eq!(d.local_hash(), None);
        let detail = d.detail();
        assert!(detail.contains("cerulion topic info"), "{detail}");
        // ROLL-IN 2: it must claim only what the REPORTING VANTAGE knows. The
        // sink emits this arm from a process whose resolver half CAN name the
        // type (an unmapped remote-attach type), so "nothing names the type"
        // was affirmatively false there.
        assert!(detail.contains("AT THIS VANTAGE"), "{detail}");
        assert!(!detail.contains("nothing names the type"), "{detail}");
    }

    /// The contradictory input: a candidate whose local hash EQUALS the refused
    /// wire hash. A walker that hashes the name to `wire_hash` would have
    /// resolved it, so the two facts came from different sets — the name is not
    /// evidence of a skew and must not be reported as one.
    #[test]
    fn a_candidate_matching_the_wire_hash_is_not_reported_as_a_skew() {
        let d = diagnose_unknown_hash(
            WIRE,
            Some(UnknownHashCandidate {
                name: NAME,
                local_hash: Some(WIRE),
            }),
        );
        assert_eq!(d, UnknownHashDiagnosis::Unidentified { wire_hash: WIRE });
        // Specifically NOT a skew claim, and it names no schema it cannot stand
        // behind.
        assert_eq!(d.reason_code(), "schema_unidentified");
        assert_eq!(d.schema_name(), None);
    }

    /// The walker-backed convenience must produce EXACTLY what the pure fn does
    /// for each of the three real shapes, with the lookup done for it. Driven
    /// over the real built-in corpus so `schema_hash_for` is the production
    /// lookup, not a stub.
    #[test]
    fn the_walker_form_agrees_with_the_pure_form_on_every_shape() {
        let schemas = crate::codegen::parse_rosmsg(
            "float64 x\nfloat64 y\nfloat64 z\n",
            "Vector3",
            Some("geometry_msgs"),
        )
        .expect("fixture schema parses");
        let (walker, _) = FrameWalker::new(vec![schemas]);
        let known = walker
            .schema_hash_for("geometry_msgs/Vector3")
            .expect("fixture is known");

        // (a) a name the walker holds, at a different hash ⇒ skew.
        let wire = known ^ 0xFFFF;
        assert_eq!(
            diagnose_unknown_hash_with_walker(&walker, wire, Some("geometry_msgs/Vector3")),
            UnknownHashDiagnosis::VersionSkew {
                name: "geometry_msgs/Vector3".to_string(),
                local_hash: known,
                wire_hash: wire,
            }
        );
        // (b) a name the walker does NOT hold ⇒ not compiled.
        assert_eq!(
            diagnose_unknown_hash_with_walker(&walker, wire, Some("pkg/Absent")),
            UnknownHashDiagnosis::TypeNotCompiled {
                name: "pkg/Absent".to_string(),
                wire_hash: wire,
            }
        );
        // (c) no name at all ⇒ unidentified.
        assert_eq!(
            diagnose_unknown_hash_with_walker(&walker, wire, None),
            UnknownHashDiagnosis::Unidentified { wire_hash: wire }
        );
    }

    /// The four tokens a client switches on are a CLOSED, stable set. Spelled
    /// out literally so a rename is a deliberate wire change rather than a
    /// silent one that leaves a Studio sidebar rendering an unknown state.
    #[test]
    fn the_reason_codes_are_the_declared_closed_set() {
        assert_eq!(
            UnknownHashDiagnosis::VersionSkew {
                name: String::new(),
                local_hash: 0,
                wire_hash: 0
            }
            .reason_code(),
            "schema_version_skew"
        );
        assert_eq!(
            UnknownHashDiagnosis::TypeNotCompiled {
                name: String::new(),
                wire_hash: 0
            }
            .reason_code(),
            "schema_not_compiled"
        );
        assert_eq!(
            UnknownHashDiagnosis::Unidentified { wire_hash: 0 }.reason_code(),
            "schema_unidentified"
        );
    }

    // ---- the reporting layer ------------------------------------------------

    /// Whole whitespace-separated `key=value` token match (the lesson:
    /// a bare `contains` makes `suppressed=5` match `suppressed=50`, and any
    /// field name a prefix of another).
    fn has_field(line: &str, key: &str, value: &str) -> bool {
        let want = format!("{key}={value}");
        line.split_whitespace().any(|t| t == want)
    }

    /// The level token, read as a whole token out of the line header — never as
    /// a bare substring, since `tracing-test` renders the SPAN NAME (this test
    /// function's own name) into every line.
    fn line_level(line: &str) -> Option<&str> {
        line.split_whitespace()
            .find(|t| matches!(*t, "TRACE" | "DEBUG" | "INFO" | "WARN" | "ERROR"))
    }

    fn lines_at<'a>(logs: &'a [&'a str], level: &str, needle: &str) -> Vec<&'a str> {
        logs.iter()
            .filter(|l| line_level(l) == Some(level) && l.contains(needle))
            .copied()
            .collect()
    }

    /// UNCONDITIONAL, level-independent: a suppressed repeat must never be LOUD.
    ///
    /// This is the half of the suppression contract that survives
    /// `release_max_level_info`. See `crate::testing::debug_lines_expected` for why the
    /// DEBUG count cannot carry it alone.
    fn suppressed_never_loud(logs: &[&str], needle: &str) -> Result<(), String> {
        for level in ["WARN", "INFO", "ERROR"] {
            let loud = lines_at(logs, level, needle);
            if !loud.is_empty() {
                return Err(format!(
                    "a suppressed repeat was emitted at {level} ({} line(s)): {loud:?}",
                    loud.len()
                ));
            }
        }
        Ok(())
    }

    #[traced_test]
    #[test]
    fn a_skew_regime_is_loud_once_then_suppressed_then_recovers_and_re_arms() {
        let mut latch = FailureRegimeLatch::new();
        let d = UnknownHashDiagnosis::VersionSkew {
            name: NAME.to_string(),
            local_hash: LOCAL,
            wire_hash: WIRE,
        };
        for _ in 0..6 {
            report_unknown_schema_hash(&mut latch, DiagnosisVantage::Render, "/viz/marker", &d);
        }
        assert_eq!(latch.total_failures(), 6);

        logs_assert(|logs: &[&str]| {
            let loud = lines_at(logs, "WARN", "dropping a frame");
            if loud.len() != 1 {
                return Err(format!(
                    "expected 1 WARN head, got {}: {loud:?}",
                    loud.len()
                ));
            }
            let head = loud[0];
            // The head carries the FULL diagnosis: which topic, which type,
            // and BOTH hashes — an operator must not need a second line.
            for (k, v) in [
                ("topic", "/viz/marker"),
                ("kind", "schema_version_skew"),
                ("schema_hash", "0x0102030405060708"),
                ("local_schema_hash", "0x1111222233334444"),
                ("schema", NAME),
                ("total_failures", "1"),
            ] {
                if !has_field(head, k, v) {
                    return Err(format!("head missing {k}={v}: {head}"));
                }
            }
            suppressed_never_loud(logs, "undecodable frame suppressed")?;
            // Through the shared EXCLUSIVE matcher:
            // a local level filter counting DEBUG lines only would let a copy of
            // the marker at TRACE beside the expected DEBUG repeats pass.
            let quiet = crate::testing::lines_at_exclusively(
                logs,
                "DEBUG",
                &["undecodable frame suppressed"],
            )?;
            let want_quiet = debug_lines_expected(5);
            if quiet.len() != want_quiet {
                return Err(format!(
                    "expected {want_quiet} DEBUG suppressed repeats, got {}",
                    quiet.len()
                ));
            }
            // The suppressed line still carries the KEY fields, so a debug-level
            // capture is a usable window rather than a bare count. (Nothing to
            // read in release, where the arm is compiled out.)
            if let Some(last) = quiet.last() {
                if !has_field(last, "kind", "schema_version_skew")
                    || !has_field(last, "topic", "/viz/marker")
                {
                    return Err(format!("suppressed line lost its keys: {last}"));
                }
            }
            Ok(())
        });

        // Recovery reports the SUPPRESSED count (not the total) exactly once,
        // and does NOT reset the unconditional total.
        report_schema_hash_resolved(&mut latch, DiagnosisVantage::Render, "/viz/marker");
        assert_eq!(latch.total_failures(), 6);
        logs_assert(|logs: &[&str]| {
            let rec = lines_at(logs, "INFO", "frames on this topic decode again");
            if rec.len() != 1 {
                return Err(format!("expected 1 INFO recovery, got {}", rec.len()));
            }
            if !has_field(rec[0], "suppressed_count", "5")
                || !has_field(rec[0], "topic", "/viz/marker")
            {
                return Err(format!("recovery line wrong: {}", rec[0]));
            }
            Ok(())
        });

        // A fresh regime is LOUD again.
        report_unknown_schema_hash(&mut latch, DiagnosisVantage::Render, "/viz/marker", &d);
        logs_assert(|logs: &[&str]| {
            let loud = lines_at(logs, "WARN", "dropping a frame");
            if loud.len() != 2 {
                return Err(format!("expected a re-armed head, got {}", loud.len()));
            }
            Ok(())
        });
        assert_eq!(latch.total_failures(), 7);
    }

    /// The decade re-announcement, at `WARN`, carrying the running total — the
    /// arm that matters most here, because the viz sink's counter is not
    /// reachable through any client verb, so the LOG is the operator's only
    /// window onto "how bad has this got?".
    #[traced_test]
    #[test]
    fn an_open_regime_re_announces_at_each_decade_at_warn() {
        let mut latch = FailureRegimeLatch::new();
        let d = UnknownHashDiagnosis::TypeNotCompiled {
            name: NAME.to_string(),
            wire_hash: WIRE,
        };
        for _ in 0..10 {
            report_unknown_schema_hash(&mut latch, DiagnosisVantage::Render, "/viz/marker", &d);
        }
        assert_eq!(latch.total_failures(), 10);
        logs_assert(|logs: &[&str]| {
            let head = lines_at(logs, "WARN", "dropping a frame");
            let again = lines_at(logs, "WARN", "STILL dropping every frame");
            let quiet = crate::testing::lines_at_exclusively(
                logs,
                "DEBUG",
                &["undecodable frame suppressed"],
            )?;
            suppressed_never_loud(logs, "undecodable frame suppressed")?;
            let want_quiet = debug_lines_expected(8);
            if (head.len(), again.len(), quiet.len()) != (1, 1, want_quiet) {
                return Err(format!(
                    "expected (1 head, 1 decade, {want_quiet} suppressed), got ({}, {}, {})",
                    head.len(),
                    again.len(),
                    quiet.len()
                ));
            }
            // The re-announcement exists for the operator who MISSED the head,
            // so it must not serve a thinner field set than the head did.
            for (k, v) in [
                ("topic", "/viz/marker"),
                ("kind", "schema_not_compiled"),
                ("schema", NAME),
                ("schema_hash", "0x0102030405060708"),
                ("total_failures", "10"),
                ("suppressed", "8"),
            ] {
                if !has_field(again[0], k, v) {
                    return Err(format!("decade line missing {k}={v}: {}", again[0]));
                }
            }
            // A DEBUG-level line must never be mistaken for the re-announcement.
            if !lines_at(logs, "DEBUG", "STILL dropping every frame").is_empty() {
                return Err("the decade re-announcement emitted at DEBUG".to_string());
            }
            Ok(())
        });
    }

    /// The `Unidentified` arm has its own two `tracing` call sites, so it needs
    /// its own decade drive — an unpinned `StillFailing` is free to become a
    /// `debug!` (the lesson).
    #[traced_test]
    #[test]
    fn the_unidentified_arm_is_loud_at_the_head_and_at_the_decade() {
        let mut latch = FailureRegimeLatch::new();
        let d = UnknownHashDiagnosis::Unidentified { wire_hash: WIRE };
        for _ in 0..10 {
            report_unknown_schema_hash(&mut latch, DiagnosisVantage::Cli, "/desk/mystery", &d);
        }
        logs_assert(|logs: &[&str]| {
            let head = lines_at(logs, "WARN", "dropping a frame");
            let again = lines_at(logs, "WARN", "STILL dropping every frame");
            if (head.len(), again.len()) != (1, 1) {
                return Err(format!(
                    "expected 1 head + 1 decade at WARN, got ({}, {})",
                    head.len(),
                    again.len()
                ));
            }
            for line in [head[0], again[0]] {
                if !has_field(line, "kind", "schema_unidentified")
                    || !has_field(line, "topic", "/desk/mystery")
                {
                    return Err(format!("unidentified line lost its keys: {line}"));
                }
                // It names NO schema — the whole point of this arm is that it
                // has none, and a fabricated one would be worse than silence.
                if line.contains("schema=") {
                    return Err(format!("unidentified line claimed a schema: {line}"));
                }
            }
            Ok(())
        });
    }

    /// Two reporters, one process, one topic — the
    /// shape `cerulion-vizd` really produces, since its render worker runs
    /// in-process with its resolver and both are handed the same frames.
    ///
    /// They reach DIFFERENT verdicts because they hold different information,
    /// and that is correct. What was WRONG was that the render side's line said
    /// "nothing names the type" in a process that was naming it on the very next
    /// line — an affirmatively false claim about the world, made by a reporter
    /// that could only speak for itself.
    ///
    /// The fix is not to silence either line: it is to make each one say who is
    /// speaking (`vantage=`) and claim only what that vantage knows. An operator
    /// grepping one topic then reads two observations, not a contradiction, and
    /// the NAMED one is visibly the authoritative half.
    #[traced_test]
    #[test]
    fn two_vantages_on_one_topic_are_labelled_and_neither_overclaims() {
        let mut render = FailureRegimeLatch::new();
        let mut resolver = FailureRegimeLatch::new();
        report_unknown_schema_hash(
            &mut render,
            DiagnosisVantage::Render,
            "/go2/lowstate",
            &UnknownHashDiagnosis::Unidentified { wire_hash: WIRE },
        );
        report_unknown_schema_hash(
            &mut resolver,
            DiagnosisVantage::Resolver,
            "/go2/lowstate",
            &UnknownHashDiagnosis::VersionSkew {
                name: NAME.to_string(),
                local_hash: LOCAL,
                wire_hash: WIRE,
            },
        );

        logs_assert(|logs: &[&str]| {
            let heads = lines_at(logs, "WARN", "dropping a frame");
            if heads.len() != 2 {
                return Err(format!("expected both heads, got {}", heads.len()));
            }
            let render_line = heads
                .iter()
                .find(|l| has_field(l, "vantage", "render"))
                .ok_or("no render-vantage line")?;
            let resolver_line = heads
                .iter()
                .find(|l| has_field(l, "vantage", "resolver"))
                .ok_or("no resolver-vantage line")?;
            // Both name the SAME topic under the SAME key — one grep, both
            // observations. (The two spellings that used to differ are aligned
            // at the sink; see `route_key_topic`'s caller.)
            for l in [render_line, resolver_line] {
                if !has_field(l, "topic", "/go2/lowstate") {
                    return Err(format!("line lost the topic key: {l}"));
                }
            }
            // Each says only what it can substantiate.
            if !has_field(render_line, "kind", "schema_unidentified")
                || render_line.contains("schema=")
            {
                return Err(format!("render line overclaimed: {render_line}"));
            }
            if render_line.contains("nothing names the type") {
                return Err(format!(
                    "the render vantage asserted a fact about the whole process: {render_line}"
                ));
            }
            if !has_field(resolver_line, "kind", "schema_version_skew")
                || !has_field(resolver_line, "schema", NAME)
            {
                return Err(format!("resolver line lost its verdict: {resolver_line}"));
            }
            Ok(())
        });
    }

    /// ANTI-TAUTOLOGY: a consumer that never refuses a frame logs NOTHING and
    /// counts zero. Without this, every "exactly N" arm above would also pass a
    /// reporter that fired on success.
    #[traced_test]
    #[test]
    fn a_consumer_that_decodes_everything_is_silent_and_counts_zero() {
        let mut latch = FailureRegimeLatch::new();
        for _ in 0..50 {
            report_schema_hash_resolved(&mut latch, DiagnosisVantage::Render, "/viz/ok");
        }
        assert_eq!(latch.total_failures(), 0);
        logs_assert(|logs: &[&str]| {
            // A forbidden line is forbidden at every level, so this predicate is
            // deliberately level-free — and it is PAIRED with the positive count
            // assertion above, which fails first if the capture itself broke.
            let noisy: Vec<_> = logs
                .iter()
                .filter(|l| l.contains("dropping a frame"))
                .collect();
            if !noisy.is_empty() {
                return Err(format!("a healthy consumer logged: {noisy:?}"));
            }
            Ok(())
        });
    }
}
