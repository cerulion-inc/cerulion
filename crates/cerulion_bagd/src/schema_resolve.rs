// SPDX-License-Identifier: AGPL-3.0-only
//! **RESOLVE-OR-DEMAND**: name every channel this bag can name,
//! and say plainly which ones it could not.
//!
//! # The problem
//!
//! The wire carries a **hash and no name**. An attach-mode tap therefore records
//! `schema_name = "unknown"` and `wire_fixed_size = 0` — an observability-grade
//! channel that renders nothing on any machine, forever, because MCAP channels
//! are immutable after bag creation. Half of it is already closed: a hash the
//! local corpus knows gets its NAME back. What was left was everything the
//! local machine has never seen, which on the flagship `ros2 attach` robot is
//! most of the ~71 dynamically-registered bridge routes the recorder was taught
//! to discover in the first place.
//!
//! This is not a reason to constrain what gets
//! recorded: Cerulion can fetch a full schema from anywhere, for
//! anything, local or networked, so the recorder demands the schema
//! through Cerulion's own paths when it cannot find it locally.
//!
//! # The ladder
//!
//! Run PER TOPIC, on a background resolver started at arm time — never on the
//! drive loop:
//!
//! | Rung | Source | Answers |
//! |---|---|---|
//! | **R0** | Declaration (`--topics-json`) | name + hash + size, from the graph |
//! | **R1** | Local corpus (built-ins + `--schema-catalog` docs) | hash → name + size |
//! | **R2** | The LOCAL machine's own catalog, via netd | a local-FOREIGN workspace's type |
//! | **R3** | A remote robot's catalog + schema, via netd | a type this machine never had |
//! | **R5** | Hash only, loudly labelled | nothing |
//!
//! **R4 (the DDS wire rung) is deliberately NOT invoked at record time.**
//! Its acquirer is reachable only from `cerulion ros2 attach`, and on that one
//! path the acquisition has ALREADY happened — it materialised `.msg` files into
//! `schemas/<pkg>/msg/`, which R1 loads. Wiring a live DDS acquirer into the
//! recorder would be a second acquisition path for zero extra coverage. Recorded
//! here as a deliberate non-inclusion, not an oversight.
//!
//! **R2 and R3 are ONE mechanism, and that is the real shape.** There is no
//! public local catalog-plane query in this repo: `catalog_serve_decision` and
//! `build_catalog_reply_from_bridge` are private and take a `zenoh::query::Query`,
//! so "ask the local catalog" without a session does not exist — and by
//! design the recorder never opens one. What does reach a
//! local-foreign workspace is netd's own CATALOG FAN-OUT (the un-scoped catalog
//! query, which this crate issues through the non-spawning verb), which
//! answers from every reachable gateway INCLUDING this machine's. So the rung is
//! the same demand, and the two are told apart by WHO ANSWERED: a reply from
//! this machine's own robot identity is [`SchemaSource::LocalCatalog`], anything
//! else is [`SchemaSource::Demanded`]. That is "reuse, don't invent", and it
//! keeps `LocalCatalog` a variant real code can produce rather than a label
//! nothing can reach.
//!
//! # Verify, never trust
//!
//! Any answer from R2/R3 is externally supplied, so **nothing a peer says about
//! identity is believed**. `SchemaReply` carries text, and `CatalogEntry` carries
//! a `schema_hash` the peer chose; this module uses NEITHER as evidence. It
//! parses the served text, runs the repo's own `resolve_fixed_nested` over the
//! doc PLUS its closure PLUS the built-in corpus, recomputes the recipe-3
//! [`MessageSchema::schema_hash`], and accepts the answer only if that
//! recomputed hash equals the hash observed **on the wire, locally**.
//!
//! A mismatch is REFUSED — the channel records `Unresolved` and keeps the name
//! `"unknown"`, never the wrong name. A channel that lies about its own type is
//! worse than one that admits it does not know: replay trusts the descriptor.
//!
//! The same verification is what closes the `wire_fixed_size` gap: a doc whose
//! hash matches is, by construction, the schema on the wire, so its fixed size
//! is a DERIVATION rather than another thing to acquire.
//!
//! # Budget
//!
//! `ensure_writer` **never blocks on the resolver**. It reads the latest
//! published answer set — an `Arc` swap, so the critical section is a pointer
//! copy and cannot be extended by anything the resolver is doing — and creates
//! the bag on its own schedule. The budget
//! ([`BagdConfig::schema_demand`](crate::BagdConfig::schema_demand)) therefore
//! bounds how long the resolver keeps TRYING; it can never bound, delay, or
//! extend bag creation, which is a structural property of the split rather than
//! a timing one.
//!
//! Round trips use the NON-waiting, NON-SPAWNING netd verbs
//! (`query_catalog_no_respawn` / `query_schema_no_respawn`): never the
//! `_converged` first-contact loop, whose 10–15 s desk-interactive wait a
//! recorder must not inherit, and never the plain verbs, whose
//! first-request retry reconnects through `connect_or_spawn_at` and would START
//! a daemon on the machine being recorded. A cold plane simply yields R5.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::codegen::{
    composed_overflow_indices, parse_rosmsg, resolve_fixed_nested, MessageSchema,
};
use cerulion_core::{CatalogReply, SchemaDoc, SchemaEncoding, SchemaReply};

/// WHERE a recorded channel's schema descriptor came from.
///
/// Carried per channel in `record_coverage.json` so a reader never has to guess
/// whether a name is the graph's own declaration, something this machine already
/// had, or something a peer supplied and the recorder then VERIFIED against the
/// wire.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SchemaSource {
    /// R0 — the caller declared it (`--topics-json`, resolved from the graph's
    /// own outputs). The graph is the source of truth for its own ports.
    Declared,
    /// R1 — this machine's schema corpus knew the wire hash (built-in types plus
    /// whatever `--schema-catalog` carried).
    LocalCorpus,
    /// R2 — the catalog plane answered from THIS machine's own robot identity: a
    /// local producer belonging to a workspace this recorder is not part of.
    LocalCatalog,
    /// R3 — a peer answered, and the served text's recomputed hash MATCHED the
    /// hash on the wire. `robot` is who answered.
    Demanded {
        /// The robot whose catalog + schema served this type.
        robot: String,
    },
    /// R5 — nothing could name it, or an answer was REFUSED by the hash check.
    /// The channel carries the wire hash with the name `"unknown"`.
    Unresolved,
}

/// The bag's REPLAY GRADE, over descriptor completeness.
///
/// Distinct from `RecordCoverage::all_channels_exact`, which measures TAP MODE
/// and keeps its v1 meaning — a fully-NAMED attach channel is still not "exact".
/// Redefining a shipped field under the same name is the misleading-name trap
/// this repo rejects, so the old field is deprecated in place and this is the
/// one to read.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReplayGrade {
    /// Every channel carries a schema name AND a definition a reader can obtain.
    Full,
    /// Some channels are describable and some are not.
    Partial,
    /// NO channel is describable — the bag renders nowhere.
    Observability,
}

/// One channel's descriptor state, as [`ReplayGrade`] counts it.
///
/// A NAME alone is not a descriptor. The whole mechanism of the bag's schema
/// closure is that a bag ships the TEXT of the custom types it references so
/// it "renders on a machine that never compiled them"; a channel named from a
/// peer's served doc whose text never reached the bag reads, on any other
/// machine, exactly as the hash-only `"unknown"` channel did: `bag info`
/// prints "[hash resolves to nothing here]" and `bag play` offers the viewer
/// zero docs. Grading such a bag `Full` would put the word "replay" on a bag
/// that replays nowhere, so the definition is a CONJUNCT rather than an
/// afterthought.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelDescriptor {
    /// The channel carries a schema NAME — any rung but
    /// [`SchemaSource::Unresolved`].
    pub named: bool,
    /// A reader can obtain the DEFINITION behind that name: either it is a type
    /// every reader compiles in (a vendored ROS 2 built-in, which the bag
    /// deliberately does NOT ship text for), or the bag's own schema closure
    /// carries its text.
    pub definition_reachable: bool,
}

impl ChannelDescriptor {
    /// A channel is replay-grade only when BOTH halves hold.
    pub fn is_complete(&self) -> bool {
        self.named && self.definition_reachable
    }
}

impl ReplayGrade {
    /// Classify from the per-channel descriptor states. PURE.
    ///
    /// An EMPTY channel set grades [`Observability`](Self::Observability): a bag
    /// with nothing in it names nothing, and claiming `Full` for it would be a
    /// vacuous truth on the one field a reader uses to decide whether the bag is
    /// worth replaying.
    pub fn classify(channels: impl IntoIterator<Item = ChannelDescriptor>) -> Self {
        let mut complete = 0usize;
        let mut total = 0usize;
        for c in channels {
            total += 1;
            if c.is_complete() {
                complete += 1;
            }
        }
        if total == 0 || complete == 0 {
            ReplayGrade::Observability
        } else if complete == total {
            ReplayGrade::Full
        } else {
            ReplayGrade::Partial
        }
    }
}

/// Build the corpus a READER of this bag would have: the ROS 2 types every
/// reader compiles in, plus exactly the docs the bag ships.
///
/// This is what turns "the bag carries a doc under that name" — a question that
/// cannot see a definition arriving from the wrong machine — into "this closure
/// reproduces this channel's recorded identity", which is the only claim
/// `replay_grade` is entitled to make.
///
/// Built-ins go FIRST, so the dedupe keeps the VENDORED definition on a name a
/// shipped doc also declares — which mirrors what a reader gets: a reader
/// resolves a vendored name from its own compiled corpus, and `bag_cmd`'s
/// `bag_doc_is_viewable` says so directly by returning `false` for any
/// built-in-named doc, whatever the bag carries under it.
///
/// SCOPE, and it is load-bearing: [`schemas_from_docs`] parses `.msg` text only,
/// because the workspace YAML parser lives in `cerulion_cli_engine`, which
/// depends on this crate. So a YAML-encoded doc contributes NOTHING here, and a
/// caller must read its absence as "not checked" rather than "not describable"
/// — see the `describes` predicate in `resolve_recording_schemas`.
pub(crate) fn corpus_from_bag_closure(docs: &[SchemaDoc]) -> ResolvedCorpus {
    let mut schemas = builtin_schemas();
    schemas.extend(schemas_from_docs(docs));
    ResolvedCorpus::build(schemas)
}

/// Every qualified name in the ROS 2 corpus this binary compiled in.
///
/// A channel named with one of these needs no text in the bag: every reader
/// links the same generated types, and `bag_cmd`'s `bag_doc_is_viewable`
/// refuses a built-in-named doc outright for that reason. It is therefore the
/// one class where "no doc in the bag" is not "renders nowhere" — and, read the
/// other way, the one class whose text must never TRAVEL, which is what the
/// closure walk in `promoted_served_closure` uses this set for.
pub(crate) fn builtin_qualified_names() -> BTreeSet<String> {
    native_ros2_messages::BUILTIN_MSGS
        .iter()
        .map(|(package, name, _)| format!("{package}/{name}"))
        .collect()
}

// ===========================================================================
// The resolved corpus — parse once, index by RECOMPUTED hash
// ===========================================================================

/// A set of schemas resolved as ONE closure, indexed both ways.
///
/// Resolution over the whole set is not optional: `parse_rosmsg` leaves every
/// `Nested` reference `fixed: None`, and a schema hashed in that state diverges
/// from the wire for any type carrying a fixed-resolvable nested field (both the
/// `wire_fixed_size` term and the folded target hash are wrong). So a doc is
/// only ever hashed alongside its closure and the built-in corpus.
///
/// # ONE definition per qualified name, decided BEFORE resolution
///
/// A closure holding two definitions of one qualified name is not a corpus, it
/// is two corpora in a trench coat — and the two halves of this type must not
/// disagree about which one wins. With first-wins INDEXES (`or_insert`)
/// while `resolve_fixed_nested` is LATER-wins (`resolve.rs`'s own warning says
/// "later definition wins in resolution"), and recipe 3 folding a FIXED-resolved
/// nested target's hash AND size into its parent, one duplicate rewrites the
/// recomputed identity of every parent that nests it while the index still
/// reports the first definition for the duplicated name itself. MEASURED: a
/// single peer-served redefinition of one leaf type made this machine's OWN
/// `name_for_wire_hash` stop naming the parent it had named a moment earlier
/// (`Some(("p/Root", 32))` → `None`) — a remote machine silently degrading a
/// first-party rung, which INVERTS the ladder's own "a LOCAL answer outranks a
/// networked one" precedence.
///
/// So [`build`](Self::build) DEDUPLICATES by qualified name — first-wins,
/// matching the index — BEFORE resolving, which makes the two rules ONE rule
/// and the corpus internally consistent by construction. Callers order the
/// vector so the definition they want to keep comes FIRST; `run_resolver` puts
/// this machine's own schemas ahead of every served one, so a peer can never
/// take a name from us.
#[derive(Debug, Default, Clone)]
pub(crate) struct ResolvedCorpus {
    /// qualified name → (recipe-3 hash, wire fixed size)
    by_name: BTreeMap<String, (u64, u32)>,
    /// recipe-3 hash → (qualified name, wire fixed size) — the size is carried
    /// HERE, off the hash-matched schema itself, rather than looked back up
    /// through `by_name`. Reading it through the name index means reading a
    /// DIFFERENT schema's size whenever two entries share a qualified name, and
    /// that number is stamped into an MCAP channel descriptor that is immutable
    /// after bag creation. The dedupe above already makes the two indexes agree;
    /// this makes the read structurally incapable of disagreeing.
    by_hash: BTreeMap<u64, (String, u32)>,
    /// Qualified names whose LATER definitions were dropped by the dedupe. A
    /// served doc landing here was refused a name this machine already holds, so
    /// a subsequent hash mismatch on it is OUR shadowing rather than the peer
    /// serving a different type — and saying so is the difference between an
    /// actionable message and one that blames the wrong machine.
    shadowed: BTreeSet<String>,
}

impl ResolvedCorpus {
    /// Resolve `schemas` as one closure and index them.
    ///
    /// Duplicate qualified names are collapsed FIRST-WINS before resolution, so
    /// the index and `resolve_fixed_nested` can never disagree about which
    /// definition a nested reference means (see the type docs).
    pub(crate) fn build(schemas: Vec<MessageSchema>) -> Self {
        let mut seen: BTreeSet<String> = BTreeSet::new();
        let mut shadowed: BTreeSet<String> = BTreeSet::new();
        let mut deduped: Vec<MessageSchema> = Vec::with_capacity(schemas.len());
        for s in schemas {
            let qualified = s.qualified_name();
            if !seen.insert(qualified.clone()) {
                shadowed.insert(qualified);
                continue;
            }
            deduped.push(s);
        }
        if !shadowed.is_empty() {
            tracing::debug!(
                shadowed = ?shadowed,
                "bagd schema corpus: a later definition of these qualified names was DROPPED — \
                 this machine's own definition wins, and a served doc under one of these names \
                 can only be verified against ours"
            );
        }
        let mut schemas = deduped;
        // Preflight composed overflows before
        // resolution — `resolve_fixed_nested` PANICS materializing a
        // composed-overflow definition referenced as a fixed nested target
        // (it sizes every fixed target through the panicking
        // `wire_fixed_size`), and a served doc is UNTRUSTED input, so one
        // hostile closure from a peer could crash the recorder. The probe is
        // core's (the ONE mirror of the resolver's lookup + fixedness rules,
        // shared with the CLI engine's preflight); run to a fixpoint because a
        // drop can flip a bare reference's resolution. Removal is plain
        // index removal: the dedupe above leaves ONE definition per qualified
        // name, so the CLI engine's winner-sweep policy has nothing to sweep.
        loop {
            let overflow = composed_overflow_indices(&schemas);
            if overflow.is_empty() {
                break;
            }
            for &idx in overflow.iter().rev() {
                tracing::warn!(
                    schema = %schemas[idx].qualified_name(),
                    "bagd schema corpus: COMPOSED fixed section (after fixed-nested inlining) \
                     overflows — dropped before resolution (hostile or corrupt definition; \
                     the rest still resolve)"
                );
                schemas.remove(idx);
            }
        }
        for w in resolve_fixed_nested(&mut schemas) {
            tracing::debug!(warning = %w, "bagd schema resolution");
        }
        let mut by_name = BTreeMap::new();
        let mut by_hash: BTreeMap<u64, (String, u32)> = BTreeMap::new();
        for s in &schemas {
            // A fixed section the wire cannot carry is never indexed — and never
            // dressed as `u32::MAX` (a fallback value would be an affirmatively wrong
            // size stamped into an immutable MCAP channel descriptor). Checked
            // BEFORE hashing: `schema_hash`/`wire_fixed_size` panic on a hostile
            // `FixedArray` length, and a served doc is untrusted input. The
            // ceiling is core's ONE wire rule on the FRAME PREFIX (the 32-byte
            // header, the fixed section and the offset table — the schema is
            // resolved here, so its table is counted exactly), not a bare
            // `u32::try_from` of the fixed section, which admitted a section of
            // exactly `u32::MAX` bytes no frame could carry.
            let offset_table_entries = s.variable_field_count();
            let size = match s.checked_wire_fixed_size() {
                Ok(size)
                    if cerulion_core::wire::frame_prefix_exceeds_wire(
                        size,
                        offset_table_entries,
                    ) =>
                {
                    tracing::warn!(
                        schema = %s.qualified_name(),
                        fixed_size = size,
                        offset_table_entries,
                        "bagd schema corpus: frame prefix (the 32-byte header, the fixed \
                         section and the offset table) exceeds the u32 wire total_size — \
                         unrepresentable on the wire, not indexed (hostile or corrupt \
                         definition; the rest still resolve)"
                    );
                    continue;
                }
                // Exact by construction: the whole prefix fits the u32
                // `total_size`, so the fixed section alone cannot exceed it.
                Ok(size) => size as u32,
                Err(e) => {
                    tracing::warn!(
                        schema = %s.qualified_name(),
                        error = %e,
                        "bagd schema corpus: fixed-section arithmetic overflows — not \
                         indexed (hostile or corrupt definition; the rest still resolve)"
                    );
                    continue;
                }
            };
            let qualified = s.qualified_name();
            let hash = s.schema_hash();
            by_name.entry(qualified.clone()).or_insert((hash, size));
            by_hash.entry(hash).or_insert((qualified, size));
        }
        Self {
            by_name,
            by_hash,
            shadowed,
        }
    }

    /// The recomputed hash + fixed size for a qualified name, if this corpus
    /// holds it.
    pub(crate) fn identity_of(&self, qualified: &str) -> Option<(u64, u32)> {
        self.by_name.get(qualified).copied()
    }

    /// Was a LATER definition of `qualified` dropped in favour of the one this
    /// corpus holds? See [`ResolvedCorpus::shadowed`].
    pub(crate) fn is_shadowed(&self, qualified: &str) -> bool {
        self.shadowed.contains(qualified)
    }

    /// The qualified name whose RECOMPUTED hash equals `wire_hash`, with THAT
    /// schema's own fixed size.
    ///
    /// This is the local-corpus rung and simultaneously its verification: a name
    /// comes back only because the schema this machine holds under it hashes to
    /// what the wire actually carries.
    pub(crate) fn name_for_wire_hash(&self, wire_hash: u64) -> Option<(&str, u32)> {
        let (name, size) = self.by_hash.get(&wire_hash)?;
        Some((name.as_str(), *size))
    }

    pub(crate) fn len(&self) -> usize {
        self.by_name.len()
    }
}

/// Parse the built-in ROS 2 corpus this binary was compiled against.
///
/// `cerulion_cli_engine::schema_cmd::parse_builtin_schemas` is the same six
/// lines, but it is `pub(crate)` there and `cerulion_cli_engine` depends on THIS
/// crate — importing it would be a cyclic package edge. So the loop is
/// re-written over the public `BUILTIN_MSGS` const rather than the mechanism
/// being re-invented.
///
/// A built-in that does not parse is SKIPPED with a warn rather than panicking:
/// the recorder's job is to record, and one unparseable vendored message must
/// not take a recording down.
pub(crate) fn builtin_schemas() -> Vec<MessageSchema> {
    let mut out = Vec::with_capacity(native_ros2_messages::BUILTIN_MSGS.len());
    for (package, name, text) in native_ros2_messages::BUILTIN_MSGS {
        match parse_rosmsg(text, name, Some(package)) {
            Ok(s) => out.push(s),
            Err(e) => tracing::warn!(
                package, name, error = ?e,
                "bagd could not parse a vendored built-in message — it will not be available \
                 for schema resolution in this recording"
            ),
        }
    }
    out
}

/// Parse served [`SchemaDoc`]s into schemas, skipping (loudly, at debug) any
/// that will not parse. Only `Msg` docs are understood here — a `Yaml` doc needs
/// `cerulion_cli_engine`'s workspace parser, which this crate cannot reach.
fn schemas_from_docs(docs: &[SchemaDoc]) -> Vec<MessageSchema> {
    let mut out = Vec::with_capacity(docs.len());
    for doc in docs {
        match doc.encoding {
            SchemaEncoding::Msg => {
                let (pkg, ty) = match doc.qualified.split_once('/') {
                    Some((p, t)) => (Some(p), t),
                    None => (None, doc.qualified.as_str()),
                };
                match parse_rosmsg(&doc.text, ty, pkg) {
                    Ok(s) => out.push(s),
                    Err(e) => tracing::debug!(
                        qualified = %doc.qualified, error = ?e,
                        "bagd could not parse a served schema doc"
                    ),
                }
            }
            SchemaEncoding::Yaml => tracing::debug!(
                qualified = %doc.qualified,
                "bagd cannot parse a YAML schema doc (that parser lives in the CLI engine, \
                 which depends on this crate) — this type stays unresolved"
            ),
        }
    }
    out
}

// ===========================================================================
// The PURE decisions
// ===========================================================================

/// A verified schema identity for one channel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ResolvedSchema {
    /// The qualified schema name (`pkg/Type`).
    pub(crate) schema_name: String,
    /// The schema's fixed wire size, DERIVED from the verified schema.
    pub(crate) wire_fixed_size: u32,
    /// Which rung produced it.
    pub(crate) source: SchemaSource,
}

/// Why a served answer was refused. Every variant means the channel keeps its
/// `"unknown"` name — never a guess.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum VerifyRefusal {
    /// The recorder never saw a frame, so there is no wire hash to check
    /// against. An unverifiable answer is not an answer.
    NoWireHash,
    /// The served docs did not parse, or did not contain the requested type.
    NotInServedDocs,
    /// The served text describes a DIFFERENT type from the one on the wire.
    HashMismatch {
        /// What the served text actually hashes to, resolved as one closure.
        recomputed: u64,
        /// What the frames on this topic actually carry.
        wire: u64,
    },
    /// This machine already holds a definition under that qualified name, so
    /// the served one was dropped by the corpus dedupe and could only ever be
    /// checked against OURS. A distinct arm because the remedy is distinct: the
    /// peer is not serving the wrong type, we are refusing to let it rename a
    /// type we define.
    ShadowedByLocalDefinition {
        /// What OUR definition of that name hashes to.
        local: u64,
        /// What the frames on this topic actually carry.
        wire: u64,
    },
}

/// **Verify, never trust.** PURE.
///
/// `served_name` is what a peer said serves this topic and `corpus` is the
/// parse of what it sent (plus its closure plus the built-ins). The answer is
/// accepted ONLY when the text's own recomputed hash equals the hash this
/// recorder observed on the wire — so a hostile, stale, or merely mismatched
/// peer cannot rename a channel, and the peer's own claimed `schema_hash` is
/// never consulted.
pub(crate) fn verify_served_schema(
    served_name: &str,
    corpus: &ResolvedCorpus,
    wire_hash: u64,
    source: SchemaSource,
) -> Result<ResolvedSchema, VerifyRefusal> {
    if wire_hash == 0 {
        return Err(VerifyRefusal::NoWireHash);
    }
    let Some((recomputed, wire_fixed_size)) = corpus.identity_of(served_name) else {
        return Err(VerifyRefusal::NotInServedDocs);
    };
    if recomputed != wire_hash {
        return Err(if corpus.is_shadowed(served_name) {
            VerifyRefusal::ShadowedByLocalDefinition {
                local: recomputed,
                wire: wire_hash,
            }
        } else {
            VerifyRefusal::HashMismatch {
                recomputed,
                wire: wire_hash,
            }
        });
    }
    Ok(ResolvedSchema {
        schema_name: served_name.to_string(),
        wire_fixed_size,
        source,
    })
}

/// What one channel's ladder decision is made from.
#[derive(Debug, Clone)]
pub(crate) struct LadderEvidence<'a> {
    /// R0: the tap carries an EXACT descriptor from the graph.
    pub(crate) declared: bool,
    /// The schema hash observed on the wire (`0` = the topic never spoke).
    pub(crate) wire_hash: u64,
    /// R1/R2: a LOCAL answer — this machine's corpus, or its own catalog.
    /// Already verified (a corpus hit is a hash match by construction).
    pub(crate) local: Option<&'a ResolvedSchema>,
    /// R3: a verified answer from a peer.
    pub(crate) demanded: Option<&'a ResolvedSchema>,
}

/// The ladder's verdict for one channel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct LadderOutcome {
    /// The name to stamp — `None` leaves the channel's existing name alone
    /// (a declared channel keeps the graph's, an unresolved one keeps
    /// `"unknown"`).
    pub(crate) schema_name: Option<String>,
    /// The fixed size to stamp, when the rung derived one.
    pub(crate) wire_fixed_size: Option<u32>,
    /// Which rung answered.
    pub(crate) source: SchemaSource,
}

/// **The ladder.** PURE, syscall-free, oracle-tested.
///
/// PRECEDENCE IS THE POINT, and it runs strictly downhill: the graph's own
/// declaration outranks everything (Principle #5 — a hash collision or a stale
/// peer must never RE-NAME a port the graph already named), a LOCAL answer
/// outranks a networked one (this machine's own corpus is first-party evidence
/// and costs nothing), and only then does a peer get to answer.
pub(crate) fn resolve_channel_schema(ev: &LadderEvidence<'_>) -> LadderOutcome {
    if ev.declared {
        return LadderOutcome {
            schema_name: None,
            wire_fixed_size: None,
            source: SchemaSource::Declared,
        };
    }
    // A topic that never spoke has NO wire hash, so there is nothing any answer
    // could be checked against. Stamping a name here would be believing a peer
    // outright — precisely what verify-never-trust forbids — so an un-declared, un-heard topic
    // is Unresolved however loudly the network volunteers a type for it.
    if ev.wire_hash == 0 {
        return LadderOutcome {
            schema_name: None,
            wire_fixed_size: None,
            source: SchemaSource::Unresolved,
        };
    }
    if let Some(hit) = ev.local {
        return LadderOutcome {
            schema_name: Some(hit.schema_name.clone()),
            wire_fixed_size: Some(hit.wire_fixed_size),
            source: hit.source.clone(),
        };
    }
    if let Some(hit) = ev.demanded {
        return LadderOutcome {
            schema_name: Some(hit.schema_name.clone()),
            wire_fixed_size: Some(hit.wire_fixed_size),
            source: hit.source.clone(),
        };
    }
    LadderOutcome {
        schema_name: None,
        wire_fixed_size: None,
        source: SchemaSource::Unresolved,
    }
}

// ===========================================================================
// The background resolver
// ===========================================================================

/// A peer's CLAIM about one topic, before verification.
///
/// Deliberately NOT a resolved schema: the resolver never decides whether an
/// answer is true, because it does not know the wire hash — only the recorder
/// does, and only once a frame has arrived. Carrying the claim (and its closure,
/// folded into [`ResolverAnswers::corpus`]) keeps the expensive parse off the
/// drive loop while leaving the DECISION at the single seam that can make it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct DemandCandidate {
    /// The qualified type name the catalog said serves this topic.
    pub(crate) schema_name: String,
    /// Which rung the answer came from — [`SchemaSource::LocalCatalog`] when
    /// this machine's own identity answered, else [`SchemaSource::Demanded`].
    pub(crate) source: SchemaSource,
}

/// What the resolver has published so far, swapped in whole.
///
/// Published as an `Arc` so `ensure_writer` holds the lock only long enough to
/// clone a pointer: the drive loop can never be delayed by anything the resolver
/// is doing, which is what makes "the demand budget cannot extend bag creation"
/// a STRUCTURAL property rather than a timing hope.
#[derive(Debug, Default)]
pub(crate) struct ResolverAnswers {
    /// THIS MACHINE's own schemas only — the built-in corpus plus whatever
    /// `--schema-catalog` carried — resolved as one set.
    ///
    /// Kept SEPARATE from [`corpus`](Self::corpus) for two reasons that are the
    /// same reason twice. (a) PROVENANCE: the ladder's local rung reads this,
    /// so a name that comes back really is one this machine already had, and
    /// [`SchemaSource::Demanded`] stays REACHABLE. Reading the merged corpus
    /// instead made `local` hit on every successfully-demanded type — the
    /// served closure is folded in, so the hash it verifies against is by
    /// construction present — and local-beats-remote precedence then stamped
    /// `LocalCorpus` on 100 % of peer-supplied identities, i.e. the one field
    /// added so a reader "never has to guess whether a name is something this
    /// machine already had, or something a peer supplied" answered the wrong
    /// one every time. (b) INTEGRITY: a served doc can no longer take part in
    /// resolving THIS machine's own types, so a peer cannot degrade a
    /// first-party answer (see [`ResolvedCorpus`]).
    pub(crate) local_corpus: ResolvedCorpus,
    /// Everything: [`local_corpus`](Self::local_corpus) PLUS every served
    /// closure, resolved as ONE set — a served doc nesting `geometry_msgs/Pose`
    /// only hashes correctly alongside it. The VERIFICATION runs against this,
    /// and nothing else does.
    pub(crate) corpus: ResolvedCorpus,
    /// topic → an UNVERIFIED peer claim.
    pub(crate) candidates: BTreeMap<String, DemandCandidate>,
    /// Every doc a peer served, by qualified name — the TEXT, which the corpus
    /// deliberately does not keep (it holds identities, not sources).
    ///
    /// Carried because a name without a definition renders nowhere: the whole
    /// mechanism of the schema closure is that the bag ships the text of the
    /// custom types it references, and a type acquired from a peer is exactly
    /// the case where this machine has none to ship.
    /// `resolve_recording_schemas` folds the docs of every VERIFIED promotion
    /// into the bag's schema closure.
    pub(crate) served_docs: BTreeMap<String, SchemaDoc>,
}

/// The netd-facing seam, so the resolver is drivable without a daemon.
pub(crate) trait SchemaOracle: Send {
    /// The fan-out catalog: every reachable robot, including this machine.
    fn catalog(&mut self) -> Result<Vec<CatalogReply>, String>;
    /// One robot's schema text for a qualified type name.
    fn schema(&mut self, robot: &str, requested: &str) -> Result<Vec<SchemaReply>, String>;
}

/// What the resolver needs to run.
pub(crate) struct ResolverInputs {
    /// The demand budget — how long the resolver keeps TRYING. Zero means the
    /// networked rungs are off entirely (the local corpus is still built, so R1
    /// still upgrades descriptors).
    pub(crate) budget: Duration,
    /// Schema docs this machine already has (`--schema-catalog`).
    pub(crate) local_docs: Vec<SchemaDoc>,
    /// This machine's own robot identity — a catalog answer bearing it is
    /// [`SchemaSource::LocalCatalog`], not a remote demand.
    pub(crate) local_identity: String,
    /// Opens the netd seam. An `Err` means the networked rungs are unavailable
    /// and the ladder falls through to R5, which is a NORMAL outcome on a robot
    /// with no daemon running — never an error the recording reports.
    pub(crate) open_oracle: Box<dyn FnOnce() -> Result<Box<dyn SchemaOracle>, String> + Send>,
}

/// A handle on the background resolver.
pub(crate) struct SchemaResolver {
    answers: Arc<Mutex<Arc<ResolverAnswers>>>,
    topics_tx: Option<Sender<String>>,
}

impl SchemaResolver {
    /// Start the resolver over `topics`, returning IMMEDIATELY.
    pub(crate) fn start(inputs: ResolverInputs, topics: Vec<String>) -> Self {
        let answers = Arc::new(Mutex::new(Arc::new(ResolverAnswers::default())));
        let (topics_tx, topics_rx) = std::sync::mpsc::channel::<String>();
        let shared = Arc::clone(&answers);
        if let Err(e) = std::thread::Builder::new()
            .name("bagd-schema-resolver".to_string())
            .spawn(move || run_resolver(inputs, topics, topics_rx, shared))
        {
            tracing::warn!(
                error = %e,
                "bagd could not start the schema resolver thread — channels this machine \
                 cannot name from the graph will record hash-only"
            );
        }
        Self {
            answers,
            topics_tx: Some(topics_tx),
        }
    }

    /// The latest published answers. Holds the lock for a pointer clone ONLY.
    pub(crate) fn answers(&self) -> Arc<ResolverAnswers> {
        match self.answers.lock() {
            Ok(g) => Arc::clone(&g),
            // A poisoned resolver is a DIAGNOSTIC failure; it must never wedge
            // the recording it is only describing.
            Err(p) => Arc::clone(&p.into_inner()),
        }
    }

    /// Tell the resolver about a topic discovered AFTER arm time.
    ///
    /// Load-bearing, not a refinement: the dynamically-registered producers
    /// that live discovery exists to record are found by the RESCAN, so a
    /// resolver that only ever saw the arm-time set would be inert on exactly
    /// the path this feature is for. A closed channel (the resolver finished)
    /// is a silent no-op.
    pub(crate) fn note_topic(&self, topic: &str) {
        if let Some(tx) = &self.topics_tx {
            let _ = tx.send(topic.to_string());
        }
    }
}

impl Drop for SchemaResolver {
    fn drop(&mut self) {
        // Dropping the sender is how the worker learns nothing more is coming;
        // it also exits on its own budget. Deliberately NOT joined: a recorder
        // must never wait on a diagnostic thread at teardown, and the worker
        // holds nothing the process needs back.
        self.topics_tx = None;
    }
}

fn run_resolver(
    inputs: ResolverInputs,
    seed_topics: Vec<String>,
    topics_rx: Receiver<String>,
    shared: Arc<Mutex<Arc<ResolverAnswers>>>,
) {
    let deadline = Instant::now() + inputs.budget;

    // Step 1 — the LOCAL corpus, parsed ONCE and published immediately. This is
    // R1, and it needs no network at all: a machine with no netd still gets its
    // `"unknown"` channels named AND their `wire_fixed_size` derived.
    let builtins = builtin_schemas();
    let mut local_schemas = builtins.clone();
    local_schemas.extend(schemas_from_docs(&inputs.local_docs));
    let local_corpus = ResolvedCorpus::build(local_schemas);
    tracing::debug!(
        schemas = local_corpus.len(),
        "bagd schema resolver: local corpus ready"
    );
    publish(
        &shared,
        local_corpus.clone(),
        local_corpus.clone(),
        BTreeMap::new(),
        BTreeMap::new(),
    );

    if inputs.budget.is_zero() {
        return;
    }

    // Step 2 — open the netd seam. An absent netd is a NORMAL outcome: the
    // recorder never starts one (see `NetdClient::connect_existing`).
    let mut oracle = match (inputs.open_oracle)() {
        Ok(o) => o,
        Err(e) => {
            tracing::debug!(
                reason = %e,
                "bagd schema resolver: no reachable cerulion-netd, so the demand rungs are \
                 unavailable — channels this machine's own corpus cannot name will record \
                 hash-only (see record_coverage.json's schema_source)"
            );
            return;
        }
    };

    // Step 3 — a catalog fan-out names the types, then one schema query per
    // still-UNANSWERED topic. Every answer's CLOSURE is folded into the
    // published corpus, which is what the stamp-time hash check runs against.
    //
    // **The loop RE-ASKS, and that is what makes the budget mean what its flag
    // says.** Retiring a topic the instant it is LOOKED UP — inserting `asked`
    // before the catalog reply is even consulted — means a topic the
    // first catalog did not name is never demanded again, and the rest of the
    // budget is spent asleep. That is the wrong shape for this rung twice over:
    // the resolver arms BEFORE the taps-ready → GO handshake releases the graph,
    // which is the coldest netd's discovery plane is ever going to be, and the
    // dynamically-registered producers that live discovery exists to record
    // arrive later still (`note_topic`). So a topic leaves `todo` only when it
    // has been ANSWERED; anything else is retried on the next round, paced by
    // `RESOLVER_RETRY_INTERVAL` so the retry can never become a spin against the
    // one daemon this machine shares.
    let mut wanted: BTreeSet<String> = seed_topics.into_iter().collect();
    let mut candidates: BTreeMap<String, DemandCandidate> = BTreeMap::new();
    let mut served_docs: BTreeMap<String, SchemaDoc> = BTreeMap::new();
    let mut answered: BTreeSet<String> = BTreeSet::new();
    let mut rounds = 0usize;

    while Instant::now() < deadline {
        wanted.extend(topics_rx.try_iter());
        let todo: Vec<String> = wanted
            .iter()
            .filter(|t| !answered.contains(*t))
            .cloned()
            .collect();
        if todo.is_empty() {
            let left = deadline.saturating_duration_since(Instant::now());
            std::thread::sleep(RESOLVER_IDLE_POLL.min(left));
            continue;
        }

        rounds += 1;
        let catalogs = match oracle.catalog() {
            Ok(c) => c,
            Err(e) => {
                // NOT a return: a catalog query can fail for reasons that pass
                // (a daemon restarting, a plane mid-gather). Retrying is what
                // the budget is FOR — and the sleep below is what keeps a
                // permanently-broken daemon from becoming a hot loop.
                tracing::debug!(error = %e, "bagd schema resolver: catalog query failed");
                sleep_until_next_round(deadline);
                continue;
            }
        };
        // topic → (robot, qualified type name). First answer wins; a topic two
        // robots both claim is deliberately not adjudicated here, because the
        // stamp-time hash check is what decides whether an answer is USABLE.
        let mut serves: BTreeMap<String, (String, String)> = BTreeMap::new();
        for reply in &catalogs {
            for entry in &reply.entries {
                if let Some(name) = &entry.schema_name {
                    serves
                        .entry(entry.topic.clone())
                        .or_insert((reply.robot.clone(), name.clone()));
                }
            }
        }

        let mut gained = false;
        for topic in todo {
            if Instant::now() >= deadline {
                break;
            }
            // NOT retired here — only an ANSWER retires a topic (see above).
            let Some((robot, name)) = serves.get(&topic) else {
                continue;
            };
            let docs = match oracle.schema(robot, name) {
                Ok(replies) => replies
                    .into_iter()
                    .flat_map(|r| r.docs.into_iter())
                    .collect::<Vec<_>>(),
                Err(e) => {
                    tracing::debug!(topic = %topic, robot = %robot, error = %e,
                        "bagd schema resolver: schema query failed");
                    continue;
                }
            };
            if docs.is_empty() {
                continue;
            }
            for doc in docs {
                served_docs.insert(doc.qualified.clone(), doc);
            }
            candidates.insert(
                topic.clone(),
                DemandCandidate {
                    schema_name: name.clone(),
                    source: if robot == &inputs.local_identity {
                        SchemaSource::LocalCatalog
                    } else {
                        SchemaSource::Demanded {
                            robot: robot.clone(),
                        }
                    },
                },
            );
            // Marked ANSWERED on receipt, not on acceptance — and that is a
            // limit of this layer, not an oversight. Verification needs the WIRE
            // hash, which only the recorder has, and only once a frame has
            // arrived; the resolver never learns whether the doc it fetched was
            // ultimately stamped or refused. Re-asking anyway would be inert on
            // the shape that matters: `serves` is built first-wins over a sorted
            // reply set, so a second round returns the SAME robot's SAME doc.
            // RESIDUAL, disclosed rather than papered over: if the answer is
            // refused at stamp time, this run will not go looking for a
            // different robot's version of that type.
            answered.insert(topic);
            gained = true;
        }

        if gained {
            // Rebuild ONCE per round, not per topic: `resolve_fixed_nested` must
            // see the whole set anyway, and re-parsing 250-odd built-ins per
            // topic would make a 71-topic robot pay ~18k parses for nothing.
            //
            // THIS MACHINE'S schemas go in FIRST, so the dedupe keeps ours on a
            // name collision and a peer can never take a name we define.
            let mut merged = builtins.clone();
            merged.extend(schemas_from_docs(&inputs.local_docs));
            merged.extend(schemas_from_docs(
                &served_docs.values().cloned().collect::<Vec<_>>(),
            ));
            publish(
                &shared,
                local_corpus.clone(),
                ResolvedCorpus::build(merged),
                candidates.clone(),
                served_docs.clone(),
            );
        }
        sleep_until_next_round(deadline);
    }
    tracing::debug!(
        candidates = candidates.len(),
        rounds,
        unanswered = wanted.len().saturating_sub(answered.len()),
        "bagd schema resolver: budget spent"
    );
}

/// Pace one retry round against the remaining budget.
///
/// The recorder shares ONE `cerulion-netd` with every other consumer on the
/// machine, so a re-asking loop must be a PACED loop: without this a topic no
/// robot serves would put the resolver into a tight catalog round trip for the
/// whole budget.
fn sleep_until_next_round(deadline: Instant) {
    let left = deadline.saturating_duration_since(Instant::now());
    if !left.is_zero() {
        std::thread::sleep(RESOLVER_RETRY_INTERVAL.min(left));
    }
}

fn publish(
    shared: &Arc<Mutex<Arc<ResolverAnswers>>>,
    local_corpus: ResolvedCorpus,
    corpus: ResolvedCorpus,
    candidates: BTreeMap<String, DemandCandidate>,
    served_docs: BTreeMap<String, SchemaDoc>,
) {
    let next = Arc::new(ResolverAnswers {
        local_corpus,
        corpus,
        candidates,
        served_docs,
    });
    match shared.lock() {
        Ok(mut g) => *g = next,
        Err(p) => *p.into_inner() = next,
    }
}

/// How long the resolver sleeps when it has nothing to ask about.
const RESOLVER_IDLE_POLL: Duration = Duration::from_millis(50);

/// How long the resolver waits between RETRY rounds for topics nothing has
/// answered yet. See [`sleep_until_next_round`].
pub(crate) const RESOLVER_RETRY_INTERVAL: Duration = Duration::from_millis(250);

// ===========================================================================
// The PRODUCTION oracle
// ===========================================================================

/// The shipping [`SchemaOracle`]: `cerulion-netd`'s query plane over its UDS
/// control seam.
///
/// **The recorder opens no network session of its own.** It speaks to the one
/// per-computer daemon that already owns the machine's zenoh session, exactly
/// as `topic echo` does.
///
/// TWO verb properties are load-bearing here, and the second was learned the
/// hard way:
///
/// * NON-WAITING — `query_*`, never their `_converged` first-contact siblings,
///   whose 10–15 s desk-interactive wait a recorder must not inherit.
/// * NON-SPAWNING — `query_*_no_respawn`, never the plain verbs. The plain ones
///   carry the first-request retry arm, which calls `reconnect` →
///   `connect_or_spawn_at`; `first_request` is `next_id == 1`, always true of
///   this oracle's first query, so `connect_existing`'s whole "never spawns,
///   never waits" contract evaporated on the round trip it was bought for.
///   MEASURED: a daemon started and the call blocked 10.06 s.
pub(crate) struct NetdOracle {
    client: cerulion_netd::NetdClient,
}

impl NetdOracle {
    /// Connect to a netd that is ALREADY RUNNING.
    ///
    /// Never `connect_or_spawn`: a recorder must not start a network daemon on a
    /// machine that deliberately has none, and must not block for that daemon's
    /// readiness ceiling inside its own arm-time window. No daemon simply means
    /// this rung is unavailable.
    pub(crate) fn connect() -> Result<Box<dyn SchemaOracle>, String> {
        cerulion_netd::NetdClient::connect_existing()
            .map(|client| Box::new(Self { client }) as Box<dyn SchemaOracle>)
            .map_err(|e| e.to_string())
    }
}

impl SchemaOracle for NetdOracle {
    fn catalog(&mut self) -> Result<Vec<CatalogReply>, String> {
        self.client
            .query_catalog_no_respawn(None)
            .map_err(|e| e.to_string())
    }

    fn schema(&mut self, robot: &str, requested: &str) -> Result<Vec<SchemaReply>, String> {
        self.client
            .query_schema_no_respawn(Some(robot), requested)
            .map_err(|e| e.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hit(name: &str, size: u32, source: SchemaSource) -> ResolvedSchema {
        ResolvedSchema {
            schema_name: name.to_string(),
            wire_fixed_size: size,
            source,
        }
    }

    // -----------------------------------------------------------------------
    // C4-T1 — the LADDER, one vector per rung plus the precedence cases.
    // -----------------------------------------------------------------------

    /// PRECEDENCE is the property, so every vector that CAN have two answers
    /// has two, and the oracle names which one must win.
    #[test]
    fn the_ladder_answers_from_the_highest_rung_that_can() {
        let local = hit("pkg/Local", 16, SchemaSource::LocalCorpus);
        let cat = hit("pkg/Cat", 24, SchemaSource::LocalCatalog);
        let dem = hit(
            "pkg/Remote",
            32,
            SchemaSource::Demanded {
                robot: "go2".to_string(),
            },
        );

        // R0 alone.
        let out = resolve_channel_schema(&LadderEvidence {
            declared: true,
            wire_hash: 7,
            local: None,
            demanded: None,
        });
        assert_eq!(out.source, SchemaSource::Declared);
        assert_eq!(out.schema_name, None, "a declared channel keeps its name");
        assert_eq!(out.wire_fixed_size, None);

        // R1 alone.
        let out = resolve_channel_schema(&LadderEvidence {
            declared: false,
            wire_hash: 7,
            local: Some(&local),
            demanded: None,
        });
        assert_eq!(out.source, SchemaSource::LocalCorpus);
        assert_eq!(out.schema_name.as_deref(), Some("pkg/Local"));
        assert_eq!(out.wire_fixed_size, Some(16));

        // R2 alone (this machine's own catalog answered).
        let out = resolve_channel_schema(&LadderEvidence {
            declared: false,
            wire_hash: 7,
            local: None,
            demanded: Some(&cat),
        });
        assert_eq!(out.source, SchemaSource::LocalCatalog);
        assert_eq!(out.schema_name.as_deref(), Some("pkg/Cat"));
        assert_eq!(out.wire_fixed_size, Some(24));

        // R3 alone.
        let out = resolve_channel_schema(&LadderEvidence {
            declared: false,
            wire_hash: 7,
            local: None,
            demanded: Some(&dem),
        });
        assert_eq!(
            out.source,
            SchemaSource::Demanded {
                robot: "go2".to_string()
            }
        );
        assert_eq!(out.schema_name.as_deref(), Some("pkg/Remote"));
        assert_eq!(out.wire_fixed_size, Some(32));

        // TWO RUNGS ABLE: the local answer must win. This is the arm the
        // "precedence inverted" variant fails — a peer must never outrank this
        // machine's own first-party corpus.
        let out = resolve_channel_schema(&LadderEvidence {
            declared: false,
            wire_hash: 7,
            local: Some(&local),
            demanded: Some(&dem),
        });
        assert_eq!(out.source, SchemaSource::LocalCorpus);
        assert_eq!(out.schema_name.as_deref(), Some("pkg/Local"));
        assert_eq!(out.wire_fixed_size, Some(16));

        // THREE RUNGS ABLE: the graph's own declaration outranks both, and does
        // not overwrite the descriptor it already carries (Principle #5).
        let out = resolve_channel_schema(&LadderEvidence {
            declared: true,
            wire_hash: 7,
            local: Some(&local),
            demanded: Some(&dem),
        });
        assert_eq!(out.source, SchemaSource::Declared);
        assert_eq!(out.schema_name, None);
        assert_eq!(out.wire_fixed_size, None);

        // NONE able → R5.
        let out = resolve_channel_schema(&LadderEvidence {
            declared: false,
            wire_hash: 7,
            local: None,
            demanded: None,
        });
        assert_eq!(out.source, SchemaSource::Unresolved);
        assert_eq!(out.schema_name, None);
        assert_eq!(out.wire_fixed_size, None);
    }

    /// A topic that never spoke has nothing to check an answer AGAINST, so no
    /// rung below the declaration may name it however confidently the network
    /// volunteers a type.
    #[test]
    fn a_topic_that_never_spoke_is_unresolved_even_when_a_peer_volunteers_a_type() {
        let dem = hit(
            "pkg/Remote",
            32,
            SchemaSource::Demanded {
                robot: "go2".to_string(),
            },
        );
        let out = resolve_channel_schema(&LadderEvidence {
            declared: false,
            wire_hash: 0,
            local: None,
            demanded: Some(&dem),
        });
        assert_eq!(out.source, SchemaSource::Unresolved);
        assert_eq!(out.schema_name, None);

        // …but a DECLARED channel is still declared: its descriptor came from
        // the graph, not from anything on the wire.
        let out = resolve_channel_schema(&LadderEvidence {
            declared: true,
            wire_hash: 0,
            local: None,
            demanded: None,
        });
        assert_eq!(out.source, SchemaSource::Declared);
    }

    // -----------------------------------------------------------------------
    // C4-T2 — VERIFY, NEVER TRUST.
    // -----------------------------------------------------------------------

    /// Build a corpus holding exactly one hand-written type, so its recomputed
    /// hash is a real recipe-3 hash of real text rather than a literal.
    fn one_type_corpus() -> (ResolvedCorpus, u64, u32) {
        let schema = parse_rosmsg(
            "float64 x\nfloat64 y\nfloat64 z\n",
            "Probe",
            Some("probe_msgs"),
        )
        .expect("the fixture must parse");
        let corpus = ResolvedCorpus::build(vec![schema]);
        let (hash, size) = corpus
            .identity_of("probe_msgs/Probe")
            .expect("the fixture must be indexed");
        (corpus, hash, size)
    }

    /// The headline refusal: a served doc whose recomputed hash DISAGREES with
    /// the wire is refused, and the refusal names both numbers.
    #[test]
    fn a_served_doc_whose_hash_disagrees_with_the_wire_is_refused() {
        let (corpus, real_hash, size) = one_type_corpus();

        // Matching: accepted, and the fixed size is DERIVED (never guessed).
        let ok = verify_served_schema(
            "probe_msgs/Probe",
            &corpus,
            real_hash,
            SchemaSource::LocalCatalog,
        )
        .expect("a doc that hashes to the wire hash IS the type on the wire");
        assert_eq!(ok.schema_name, "probe_msgs/Probe");
        assert_eq!(ok.wire_fixed_size, size);
        assert_eq!(ok.source, SchemaSource::LocalCatalog);

        // Disagreeing: REFUSED, with both hashes carried so the refusal is
        // attributable rather than a shrug.
        let wire = real_hash ^ 0x1;
        match verify_served_schema("probe_msgs/Probe", &corpus, wire, SchemaSource::LocalCorpus) {
            Err(VerifyRefusal::HashMismatch {
                recomputed,
                wire: w,
            }) => {
                assert_eq!(recomputed, real_hash);
                assert_eq!(w, wire);
            }
            other => panic!("a mismatched doc must be REFUSED, got {other:?}"),
        }

        // A name the served docs did not contain is not an answer either.
        assert_eq!(
            verify_served_schema(
                "probe_msgs/NotServed",
                &corpus,
                real_hash,
                SchemaSource::LocalCorpus
            ),
            Err(VerifyRefusal::NotInServedDocs)
        );

        // No wire hash → nothing to verify against.
        assert_eq!(
            verify_served_schema("probe_msgs/Probe", &corpus, 0, SchemaSource::LocalCorpus),
            Err(VerifyRefusal::NoWireHash)
        );
    }

    /// The verification is only worth anything if the recomputation is RIGHT,
    /// and "right" means agreeing with what a real publisher puts on the wire.
    ///
    /// This is the one arm with an EXTERNAL oracle: `geometry_msgs/Twist`'s
    /// compiled `SCHEMA_HASH` is what `build.rs` derived through the generator,
    /// and it is what a live publisher stamps into every frame's header. If this
    /// module's parse + resolve + hash agrees with it, a demanded doc can be
    /// checked against a real wire hash; if it does not, every demand would be
    /// falsely refused.
    ///
    /// Twist is chosen deliberately: it nests two FIXED `Vector3`s, so its hash
    /// folds their target hashes and its `wire_fixed_size` counts them. A corpus
    /// built WITHOUT running `resolve_fixed_nested` over the whole set leaves
    /// those `fixed: None` and gets both terms wrong — which is exactly the
    /// failure mode the closure requirement exists to prevent.
    #[test]
    fn the_recomputed_hash_matches_what_a_real_publisher_puts_on_the_wire() {
        use cerulion_core::message::ShmMessage;
        let corpus = ResolvedCorpus::build(builtin_schemas());

        let (hash, size) = corpus
            .identity_of("geometry_msgs/Twist")
            .expect("the built-in corpus must hold geometry_msgs/Twist");
        assert_eq!(
            hash,
            native_ros2_messages::geometry_msgs::Twist::SCHEMA_HASH,
            "the recomputed recipe-3 hash must equal the compiled type's — a nested-FIXED \
             type mis-hashes when the corpus is not resolved as one closure"
        );
        assert_eq!(
            size as usize,
            native_ros2_messages::geometry_msgs::Twist::WIRE_FIXED_SIZE
        );

        // And the reverse index agrees, which is what the local-corpus rung uses.
        let (name, back_size) = corpus
            .name_for_wire_hash(native_ros2_messages::geometry_msgs::Twist::SCHEMA_HASH)
            .expect("a wire hash from a real publisher must resolve to its name");
        assert_eq!(name, "geometry_msgs/Twist");
        assert_eq!(back_size, size);

        // A VARIABLE-shaped type too, so the arm is not accidentally scoped to
        // the all-fixed case.
        let (shash, _) = corpus
            .identity_of("std_msgs/String")
            .expect("the built-in corpus must hold std_msgs/String");
        assert_eq!(shash, native_ros2_messages::std_msgs::String::SCHEMA_HASH);

        // A hash nothing serves resolves to NOTHING — the anti-tautology half,
        // without which "resolves to its name" is satisfied by a lookup that
        // returns something for everything.
        assert_eq!(corpus.name_for_wire_hash(0xDEAD_BEEF_DEAD_BEEF), None);
    }

    // -----------------------------------------------------------------------
    // The corpus is ONE definition per name, and this machine's wins.
    // -----------------------------------------------------------------------

    fn parse(pkg: &str, name: &str, text: &str) -> MessageSchema {
        parse_rosmsg(text, name, Some(pkg)).expect("fixture parses")
    }

    /// A COMPOSED-overflow definition referenced as a FIXED nested
    /// target (the shape: `Inner` ≈ 4 GiB is representable, `Outer =
    /// Inner[2^33]` overflows only after inlining, `Wrapper` nests `Outer`)
    /// would otherwise PANIC `build` inside `resolve_fixed_nested` — a peer-served
    /// closure could crash the recorder. The preflight drops `Outer` before
    /// resolution; `Wrapper` survives with the ref variable (its checked
    /// single-definition hash), and the representable `Inner` indexes with
    /// its exact size.
    #[test]
    fn a_composed_overflow_nested_target_is_preflighted_before_resolution() {
        // Without the preflight this PANICS (schema.rs: FixedArray size overflows usize).
        let corpus = ResolvedCorpus::build(vec![
            parse("cer15", "Inner", "float64[536870907] v\n"),
            parse("cer15", "Outer", "Inner[8589934592] arr\n"),
            parse("cer15", "Wrapper", "Outer o\nfloat64 w\n"),
            parse("cer15", "Probe", "float64 x\n"),
        ]);
        assert_eq!(
            corpus.identity_of("cer15/Outer"),
            None,
            "the composed overflow is dropped before resolution, never indexed"
        );
        let (_, inner_size) = corpus
            .identity_of("cer15/Inner")
            .expect("the representable member indexes");
        assert_eq!(inner_size, 536_870_907 * 8);
        // Wrapper survives with its ref conservatively variable: hash oracle
        // = the definition hashed alone (nothing resolves its ref).
        let (wrapper_hash, wrapper_size) = corpus
            .identity_of("cer15/Wrapper")
            .expect("the referencing definition survives");
        let alone = parse("cer15", "Wrapper", "Outer o\nfloat64 w\n");
        assert_eq!(wrapper_hash, alone.schema_hash());
        assert_eq!(wrapper_size, 8, "only the trailing f64 is fixed");
        assert!(corpus.identity_of("cer15/Probe").is_some());
    }

    /// A definition whose fixed section the wire cannot carry is NOT indexed
    /// — never served as `u32::MAX` (an `unwrap_or` fallback), which
    /// would have stamped an affirmatively wrong size into an immutable MCAP
    /// channel descriptor. `Huge` (2^64 − 8 bytes) passes the checked recipe
    /// and fails the u32 conversion; `Hostile` (2^61 × 8 = 2^64) overflows
    /// the checked recipe itself — its `wire_fixed_size()` would PANIC
    /// the corpus build. The sane sibling still indexes with its real size.
    #[test]
    fn an_unrepresentable_fixed_section_is_never_dressed_as_u32_max() {
        let corpus = ResolvedCorpus::build(vec![
            parse("cer14", "Huge", "float64[2305843009213693951] v\n"),
            parse("cer14", "Hostile", "float64[2305843009213693952] v\n"),
            parse("cer14", "Probe", "float64 x\n"),
        ]);
        assert_eq!(
            corpus.identity_of("cer14/Huge"),
            None,
            "an over-u32 fixed section is unindexed, not saturated"
        );
        assert_eq!(
            corpus.identity_of("cer14/Hostile"),
            None,
            "an overflowing declaration is unindexed, not a panic"
        );
        let (_, size) = corpus
            .identity_of("cer14/Probe")
            .expect("the sane sibling still indexes");
        assert_eq!(size, 8, "the sibling's real fixed size");
    }

    /// The corpus ceiling is the FRAME PREFIX — the 32-byte
    /// header, the fixed section and one 8-byte offset entry per variable
    /// field — not a bare `u32::try_from` of the fixed section (which
    /// indexed a section of exactly `u32::MAX` bytes, a descriptor no frame
    /// can carry). Both sides of the boundary, on the RESOLVED corpus:
    /// `At` (`u32::MAX − 32`) and `VarAt` (`u32::MAX − 40`, one string)
    /// index with their exact sizes; one byte past each, the
    /// extreme `Max`, and `WrapOver` (a `uint8` run whose inlined 8-byte
    /// `Small` lands one byte past through padding) do not; `WrapAt` does.
    #[test]
    fn the_corpus_ceiling_counts_the_header_and_the_offset_table_on_both_sides() {
        const AT: usize = u32::MAX as usize - 32;
        const VAR_AT: usize = AT - 8;
        let corpus = ResolvedCorpus::build(vec![
            parse("cer26", "At", &format!("uint8[{AT}] a\n")),
            parse("cer26", "Over", &format!("uint8[{}] a\n", AT + 1)),
            parse("cer26", "Max", &format!("uint8[{}] a\n", u32::MAX)),
            parse("cer26", "VarAt", &format!("uint8[{VAR_AT}] a\nstring s\n")),
            parse(
                "cer26",
                "VarOver",
                &format!("uint8[{}] a\nstring s\n", VAR_AT + 1),
            ),
            parse("cer26", "Small", "uint64 v\n"),
            parse(
                "cer26",
                "WrapAt",
                &format!("uint8[{}] a\nSmall s\n", AT - 16),
            ),
            parse(
                "cer26",
                "WrapOver",
                &format!("uint8[{}] a\nSmall s\n", AT - 8),
            ),
            // A nested target that stays VARIABLE after resolution (it has
            // a string): its reference is a real offset entry on the
            // resolved corpus — the exact count, not the declared floor.
            parse("cer26", "Blob", "string s\n"),
            parse(
                "cer26",
                "VarNestAt",
                &format!("uint8[{VAR_AT}] a\nBlob b\n"),
            ),
            parse(
                "cer26",
                "VarNestOver",
                &format!("uint8[{}] a\nBlob b\n", VAR_AT + 1),
            ),
            // Exactly 2^32: the guard-less `as u32` would record a descriptor
            // of 0 — a silently EMPTY channel, worse than a large wrong one.
            parse("cer26", "Wrap32", "uint8[4294967296] a\n"),
        ]);
        let size_of = |name: &str| corpus.identity_of(name).map(|(_, size)| size);
        assert_eq!(size_of("cer26/At"), Some(u32::MAX - 32), "at the ceiling");
        assert_eq!(size_of("cer26/Over"), None, "one byte past");
        assert_eq!(size_of("cer26/Max"), None, "the extreme");
        assert_eq!(
            size_of("cer26/VarAt"),
            Some(u32::MAX - 40),
            "one offset entry"
        );
        assert_eq!(size_of("cer26/VarOver"), None, "one byte past, one entry");
        // AT − 16 = …247; the u64 lands at …248 and ends …256 (= AT − 7).
        assert_eq!(
            size_of("cer26/WrapAt"),
            Some(u32::MAX - 39),
            "inlined, fits"
        );
        // AT − 8 = …255; the u64 lands at …256 and ends …264 (= AT + 1).
        assert_eq!(size_of("cer26/WrapOver"), None, "inlined, one past");
        assert_eq!(
            size_of("cer26/VarNestAt"),
            Some(u32::MAX - 40),
            "a still-variable nested reference is one offset entry: at the ceiling"
        );
        assert_eq!(
            size_of("cer26/VarNestOver"),
            None,
            "a still-variable nested reference is one offset entry: one past (the \
             declared floor would have counted it as none)"
        );
        assert_eq!(
            size_of("cer26/Wrap32"),
            None,
            "2^32 is refused, never wrapped to 0"
        );
    }

    /// A peer serving a divergent definition of a name THIS MACHINE defines must
    /// not be able to change what our own types hash to.
    ///
    /// A first-wins index while `resolve_fixed_nested` is later-wins, with
    /// recipe 3 folding a nested target's hash and size into its parent, lets one
    /// served redefinition of a leaf rewrite the identity of every parent
    /// nesting it while the index still reports OUR leaf. MEASURED with a
    /// first-wins index: `name_for_wire_hash(root)` went `Some(("p/Root", 32))` →
    /// `None`, i.e. a remote machine silently switched off this machine's own
    /// first-party rung, INVERTING the ladder's local-beats-remote precedence.
    #[test]
    fn a_served_doc_cannot_change_what_this_machines_own_types_hash_to() {
        let leaf_v1 = "float64 a\nfloat64 b\n";
        let leaf_v2 = "float64 a\nfloat64 b\nfloat64 c\nfloat64 d\n";
        let root_text = "p/Leaf leaf\n";

        // What this machine holds on its own.
        let local = ResolvedCorpus::build(vec![
            parse("p", "Leaf", leaf_v1),
            parse("p", "Root", root_text),
        ]);
        let (root_hash, root_size) = local
            .identity_of("p/Root")
            .expect("precondition: this machine can identify its own root type");
        assert_eq!(
            local.name_for_wire_hash(root_hash),
            Some(("p/Root", root_size)),
            "precondition: the local rung names it before any peer speaks"
        );

        // Now a peer serves a DIVERGENT `p/Leaf`, appended after ours exactly as
        // `run_resolver` orders the merged set.
        let merged = ResolvedCorpus::build(vec![
            parse("p", "Leaf", leaf_v1),
            parse("p", "Root", root_text),
            parse("p", "Leaf", leaf_v2),
        ]);
        assert_eq!(
            merged.identity_of("p/Root"),
            Some((root_hash, root_size)),
            "OUR root must hash to exactly what it hashed to before the peer spoke"
        );
        assert_eq!(
            merged.name_for_wire_hash(root_hash),
            Some(("p/Root", root_size)),
            "and it must still be resolvable from the wire hash — this is the rung a peer \
             must not be able to switch off"
        );
        assert!(
            merged.is_shadowed("p/Leaf"),
            "the corpus must RECORD that it dropped a later definition, so a refusal on that \
             name can name the real cause instead of blaming the peer's text"
        );
        assert!(
            !merged.is_shadowed("p/Root"),
            "the anti-tautology half: a name with no duplicate is not shadowed"
        );
    }

    /// A name-colliding corpus must never stamp a size that belongs to a
    /// DIFFERENT schema.
    ///
    /// A `name_for_wire_hash` that found the NAME through the hash index and
    /// then read the SIZE back out of the NAME index would serve a different schema's
    /// number whenever two entries share a qualified name, stamped into an MCAP
    /// channel descriptor that is immutable after bag creation. A field left
    /// at 0 (meaning unknown) is an absence, so that would
    /// replace an absence with a false measurement.
    #[test]
    fn the_fixed_size_never_comes_from_a_schema_that_merely_shares_a_name() {
        let first = parse("p", "T", "float64 a\nfloat64 b\nfloat64 c\nfloat64 d\n");
        let second = parse("p", "T", "float64 a\n");
        let (h1, h2) = (first.schema_hash(), second.schema_hash());
        let (s1, s2) = (
            first.wire_fixed_size() as u32,
            second.wire_fixed_size() as u32,
        );
        assert_ne!(h1, h2, "precondition: the two definitions really differ");
        assert_ne!(
            s1, s2,
            "precondition: and differ in SIZE, which is the point"
        );

        let corpus = ResolvedCorpus::build(vec![first, second]);
        assert_eq!(
            corpus.name_for_wire_hash(h1),
            Some(("p/T", s1)),
            "the definition the corpus kept resolves to its OWN size"
        );
        // The dropped definition is not resolvable at all — the fail-CLOSED
        // outcome. What must never happen is `Some(("p/T", s1))` here: a name
        // and a size that describe a schema this wire is not carrying.
        match corpus.name_for_wire_hash(h2) {
            None => {}
            Some((name, size)) => assert_eq!(
                (name, size),
                ("p/T", s2),
                "a hash may resolve to its OWN schema or to nothing — never to a namesake's size"
            ),
        }

        // THE READ PATH, pinned over a state `build` can no longer produce.
        //
        // Stated plainly because it bounds what this half is worth: with the
        // dedupe above, `by_name[by_hash[h]]` is always the same schema, so
        // reading the size back through the NAME index is OUTPUT-EQUIVALENT and
        // a variant that does so survives every behavioural arm.
        // Carrying the size in the hash index is what makes the divergence
        // structurally unrepresentable rather than merely absent, so a future
        // change that relaxes the dedupe cannot silently restore a wrong size in
        // an immutable channel descriptor. The indexes are private, so the only
        // way to hold the read to that promise is to hand it the divergence.
        let divergent = ResolvedCorpus {
            by_name: [("p/T".to_string(), (h1, s1))].into_iter().collect(),
            by_hash: [(h2, ("p/T".to_string(), s2))].into_iter().collect(),
            shadowed: BTreeSet::new(),
        };
        assert_eq!(
            divergent.name_for_wire_hash(h2),
            Some(("p/T", s2)),
            "the size must come from the schema the HASH matched, never from a namesake the \
             name index happens to hold"
        );
    }

    /// A served doc refused because WE shadow its name gets its own reason.
    ///
    /// The generic mismatch message says "the text describes a DIFFERENT type
    /// from the one on this topic's wire", which blames the peer for a corpus
    /// artifact: the peer may be entirely right, and we simply cannot hold two
    /// definitions of one qualified name in one closure.
    #[test]
    fn a_served_name_this_machine_shadows_is_refused_with_the_real_reason() {
        let ours = parse("p", "T", "float64 a\n");
        let theirs = parse("p", "T", "float64 a\nfloat64 b\n");
        let (our_hash, their_hash) = (ours.schema_hash(), theirs.schema_hash());
        let corpus = ResolvedCorpus::build(vec![ours, theirs]);

        assert_eq!(
            verify_served_schema(
                "p/T",
                &corpus,
                their_hash,
                SchemaSource::Demanded {
                    robot: "go2".to_string()
                }
            ),
            Err(VerifyRefusal::ShadowedByLocalDefinition {
                local: our_hash,
                wire: their_hash,
            }),
            "the refusal must name OUR definition as the thing it checked against"
        );
        // The anti-tautology half: an unshadowed name that simply disagrees is
        // still the ordinary mismatch, so the new arm is not swallowing it.
        let solo = ResolvedCorpus::build(vec![parse("q", "U", "float64 a\n")]);
        let solo_hash = solo.identity_of("q/U").expect("indexed").0;
        assert_eq!(
            verify_served_schema("q/U", &solo, solo_hash ^ 0x1, SchemaSource::LocalCatalog),
            Err(VerifyRefusal::HashMismatch {
                recomputed: solo_hash,
                wire: solo_hash ^ 0x1,
            })
        );
    }

    /// The recorder's oracle uses the NON-SPAWNING verbs.
    ///
    /// STRUCTURAL, because the behavioural kill is unreachable from here: the
    /// respawn arm needs a daemon that serves its `Hello` and then hangs up, and
    /// every bagd arm drives either a daemon that answers or one that never
    /// speaks — so reverting this one call site leaves the whole bagd suite
    /// green (RUN, not assumed). The verb's own behaviour is pinned in
    /// `cerulion_netd`'s `the_no_respawn_query_verbs_do_not_reconnect_on_an_early_close`;
    /// this pins that the recorder ADOPTS it, which is the half that can ship
    /// inert.
    ///
    /// Scoped to the `impl` BLOCK rather than the file: the module docs NAME the
    /// plain verbs in order to explain why they are not used, and a whole-file
    /// `contains` would be satisfied by that prose.
    #[test]
    fn the_recorders_oracle_calls_the_verbs_that_never_start_a_daemon() {
        let src = include_str!("schema_resolve.rs");
        let start = src
            .find("impl SchemaOracle for NetdOracle {")
            .expect("the production oracle impl must exist");
        let body = &src[start..];
        let end = body
            .find("\n}\n")
            .expect("the impl block must terminate at column 0");
        let body = &body[..end];

        for verb in ["query_catalog_no_respawn", "query_schema_no_respawn"] {
            assert!(
                body.contains(verb),
                "the recorder must call `{verb}` — the plain sibling carries the \
                 first-request retry, which reconnects through `connect_or_spawn_at` and so \
                 STARTS a daemon on the machine being recorded. impl body:\n{body}"
            );
        }
        for forbidden in ["query_catalog(", "query_schema("] {
            assert!(
                !body.contains(forbidden),
                "`{forbidden}` must not appear in the recorder's oracle: `first_request` is \
                 `next_id == 1`, always true of its FIRST query, so that arm is live on \
                 exactly the round trip `connect_existing` was bought for (MEASURED: a daemon \
                 started, the call blocked 10.06 s). impl body:\n{body}"
            );
        }
        // ANTI-TAUTOLOGY: the extraction really found the impl and not an empty
        // slice, so the negative assertions above are not vacuous.
        assert!(
            body.contains("fn catalog(") && body.contains("fn schema("),
            "the extracted block must be the oracle impl; got:\n{body}"
        );
    }

    // -----------------------------------------------------------------------
    // The budget bounds how long the resolver keeps TRYING, not how long it waits.
    // -----------------------------------------------------------------------

    /// An oracle whose catalog is EMPTY until the Nth call — the cold plane the
    /// resolver arms against, since it starts before the taps-ready → GO
    /// handshake releases the graph.
    struct ColdThenWarm {
        calls: Arc<std::sync::atomic::AtomicUsize>,
        warm_after: usize,
        topic: String,
    }

    impl SchemaOracle for ColdThenWarm {
        fn catalog(&mut self) -> Result<Vec<CatalogReply>, String> {
            let n = self.calls.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            if n < self.warm_after {
                return Ok(Vec::new());
            }
            Ok(vec![CatalogReply {
                version: cerulion_core::transport::cerulion_q::CATALOG_WIRE_VERSION,
                robot: "go2".to_string(),
                entries: vec![cerulion_core::CatalogEntry {
                    topic: self.topic.clone(),
                    schema_hash: None,
                    schema_name: Some("p/Late".to_string()),
                    provenance: cerulion_core::CatalogProvenance::Runtime,
                    producer_count: Some(1),
                    liveness: None,
                }],
                error: None,
            }])
        }

        fn schema(&mut self, robot: &str, requested: &str) -> Result<Vec<SchemaReply>, String> {
            Ok(vec![SchemaReply::found(
                robot,
                requested,
                vec![SchemaDoc {
                    qualified: "p/Late".to_string(),
                    encoding: SchemaEncoding::Msg,
                    text: "float64 a\n".to_string(),
                    deps: Vec::new(),
                }],
            )])
        }
    }

    /// A topic the FIRST catalog does not name must be asked again.
    ///
    /// A resolver that retires a topic the instant it is LOOKED UP —
    /// inserting `asked` before the catalog reply is even consulted — lets one
    /// empty answer retire it for the run, and the rest of the budget is spent
    /// asleep. That is the wrong shape twice over on this path: the resolver
    /// arms BEFORE the taps-ready → GO handshake releases the graph, which is
    /// the coldest netd's discovery plane is ever going to be, and the
    /// dynamically-registered producers that live discovery exists to record
    /// arrive later still. So `--schema-demand-timeout-ms` bounded how long the
    /// resolver WAITED, not how many times it tried, while its own doc said the
    /// opposite.
    #[test]
    fn a_topic_the_first_catalog_cannot_name_is_asked_again() {
        let calls = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let topic = "/late".to_string();
        let oracle_calls = Arc::clone(&calls);
        let oracle_topic = topic.clone();
        let resolver = SchemaResolver::start(
            ResolverInputs {
                // Comfortably more than three rounds at the retry pace, and
                // bounded so a broken loop ends the test rather than hanging it.
                budget: Duration::from_secs(3),
                local_docs: Vec::new(),
                local_identity: "desk".to_string(),
                open_oracle: Box::new(move || {
                    Ok(Box::new(ColdThenWarm {
                        calls: oracle_calls,
                        warm_after: 2,
                        topic: oracle_topic,
                    }) as Box<dyn SchemaOracle>)
                }),
            },
            vec![topic.clone()],
        );

        // A CONDITION, not a wall bet: wait until the candidate appears, so load
        // delays this arm instead of failing it.
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut answers = resolver.answers();
        while Instant::now() < deadline && !answers.candidates.contains_key(&topic) {
            std::thread::sleep(Duration::from_millis(10));
            answers = resolver.answers();
        }

        assert!(
            answers.candidates.contains_key(&topic),
            "a topic the first catalog could not name must be asked again while budget \
             remains; the resolver made {} catalog call(s) and produced {:?}",
            calls.load(std::sync::atomic::Ordering::SeqCst),
            answers.candidates.keys().collect::<Vec<_>>()
        );
        assert!(
            calls.load(std::sync::atomic::Ordering::SeqCst) >= 3,
            "precondition: the answer must have arrived on a LATER round — otherwise this arm \
             passes against a one-shot loop"
        );
        assert!(
            answers.served_docs.contains_key("p/Late"),
            "and the served text is kept, so it can reach the bag's schema closure"
        );
    }

    /// …and the retry is PACED. A topic nothing ever serves must not turn the
    /// resolver into a tight round-trip loop against the one daemon this machine
    /// shares with every other consumer.
    #[test]
    fn a_topic_nothing_serves_is_retried_at_a_pace_not_in_a_spin() {
        let calls = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let oracle_calls = Arc::clone(&calls);
        let budget = Duration::from_millis(900);
        let resolver = SchemaResolver::start(
            ResolverInputs {
                budget,
                local_docs: Vec::new(),
                local_identity: "desk".to_string(),
                open_oracle: Box::new(move || {
                    Ok(Box::new(ColdThenWarm {
                        calls: oracle_calls,
                        // Never warms: every catalog is empty.
                        warm_after: usize::MAX,
                        topic: "/never".to_string(),
                    }) as Box<dyn SchemaOracle>)
                }),
            },
            vec!["/orphan".to_string()],
        );
        std::thread::sleep(budget + Duration::from_millis(400));
        let n = calls.load(std::sync::atomic::Ordering::SeqCst);
        // A CEILING is the only side load can move the safe way here: a slow
        // runner makes FEWER round trips, never more. The bound is generous
        // (the pace admits ~4 in this budget) and still orders of magnitude
        // under what an unpaced loop produces.
        assert!(
            n <= 40,
            "the retry must be paced by RESOLVER_RETRY_INTERVAL; {n} catalog round trips in \
             {budget:?} is a spin against a shared daemon"
        );
        // …and it really did retry, so the ceiling is not passing vacuously.
        assert!(
            n >= 2,
            "precondition: the loop must have retried at all; got {n}"
        );
        drop(resolver);
    }

    // -----------------------------------------------------------------------
    // ReplayGrade
    // -----------------------------------------------------------------------

    /// A channel that is named AND whose definition a reader can obtain.
    fn complete() -> ChannelDescriptor {
        ChannelDescriptor {
            named: true,
            definition_reachable: true,
        }
    }

    /// A channel nothing could name.
    fn unnamed() -> ChannelDescriptor {
        ChannelDescriptor {
            named: false,
            definition_reachable: false,
        }
    }

    /// The shape `definition_reachable` must not paper over: a NAME
    /// arrived (from a peer, or from a hash binding carrying no text) but the
    /// bag ships no definition for it.
    fn named_but_textless() -> ChannelDescriptor {
        ChannelDescriptor {
            named: true,
            definition_reachable: false,
        }
    }

    #[test]
    fn the_replay_grade_reports_descriptor_completeness_over_the_whole_channel_set() {
        assert_eq!(
            ReplayGrade::classify([complete(), complete(), complete()]),
            ReplayGrade::Full,
            "every channel describable ⇒ Full, whichever rung named it"
        );
        assert_eq!(
            ReplayGrade::classify([complete(), unnamed()]),
            ReplayGrade::Partial
        );
        assert_eq!(
            ReplayGrade::classify([unnamed(), unnamed()]),
            ReplayGrade::Observability
        );
        // A bag with no channels names nothing; claiming `Full` for it would be
        // a vacuous truth on the field a reader uses to decide whether the bag
        // is worth replaying.
        assert_eq!(
            ReplayGrade::classify(std::iter::empty()),
            ReplayGrade::Observability
        );
    }

    /// THE arm that separates a name from a descriptor.
    ///
    /// A channel promoted by the demand rung whose served TEXT never reached the
    /// bag reads, on every machine but the recorder, exactly as the hash-only
    /// `"unknown"` channel it replaced. Counting it toward `Full` would put the
    /// word "replay" on a bag that replays nowhere — so the definition is a
    /// CONJUNCT, and this arm is what fails if it ever becomes a comment.
    #[test]
    fn a_named_channel_whose_definition_the_bag_does_not_carry_is_not_replay_grade() {
        assert_eq!(
            ReplayGrade::classify([named_but_textless()]),
            ReplayGrade::Observability,
            "a name with no obtainable definition describes nothing"
        );
        assert_eq!(
            ReplayGrade::classify([complete(), named_but_textless()]),
            ReplayGrade::Partial,
            "one describable channel beside one that is not ⇒ Partial, never Full"
        );
        // The anti-tautology half: the SAME channel with its definition present
        // grades `Full`, so the arm is measuring the definition and not simply
        // refusing everything.
        assert_eq!(
            ReplayGrade::classify([complete(), complete()]),
            ReplayGrade::Full
        );
    }

    /// The built-in corpus is the ONE class where "no doc in the bag" is not
    /// "renders nowhere" — every reader links the same generated types, which is
    /// why their text is deliberately omitted. A `definition_reachable`
    /// predicate that did not know this would grade every ordinary ROS 2
    /// recording `Observability`.
    #[test]
    fn the_builtin_name_set_is_the_corpus_this_binary_actually_compiled() {
        let names = builtin_qualified_names();
        assert!(
            names.contains("geometry_msgs/Twist"),
            "a vendored type must be recognised as reader-resolvable without text"
        );
        assert!(names.contains("std_msgs/String"));
        assert!(
            !names.contains("peerpkg/Widget"),
            "a type only a peer has must NOT be treated as reader-resolvable"
        );
        assert_eq!(
            names.len(),
            native_ros2_messages::BUILTIN_MSGS.len(),
            "every vendored message contributes exactly one qualified name"
        );
    }
}
