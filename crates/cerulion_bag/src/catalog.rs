// SPDX-License-Identifier: AGPL-3.0-only
//! The bag's SCHEMA-PROVENANCE attachment — the verbatim `.msg`/YAML
//! text of every CUSTOM type the recording's frames are stamped with, so a bag
//! describes its own contents on a machine that never compiled them.
//!
//! # Why the attachment and not the MCAP Schema record
//!
//! An MCAP Schema record's `data` blob is per-CHANNEL, and it is the wrong shape
//! for a schema CLOSURE: a `unitree_go/LowState` frame needs
//! `unitree_go/BmsState`, `unitree_go/MotorState`, … — types that are NOT
//! channels of the bag and therefore have no Schema record of their own. Putting
//! the closure in the per-channel blob would duplicate every shared nested type
//! once per channel that references it.
//!
//! So the closure rides ONE bag-level attachment, [`SCHEMA_DOCS_ATTACHMENT`],
//! exactly like the `__cerulion/recorder.json` host-identity attachment
//! and bagd's `__cerulion/record_health.json`. Consequences that
//! matter:
//!
//! - The Schema record bytes are BYTE-UNCHANGED, so the 18-byte
//!   [`SchemaDescriptor`](crate::SchemaDescriptor) contract, its
//!   `descriptor_version`, and every existing reader keep working in BOTH
//!   directions — an old `cerulion` reads a new bag and a new `cerulion` reads
//!   an old one, neither degraded beyond the missing text.
//! - A recorder that resolves NOTHING — no docs and no bindings — writes no
//!   attachment at all, and its bag is byte-identical to a pre-attachment
//!   recording. Note what that is NOT: a bag whose topics are all BUILT-IN
//!   still gets an attachment, because [`BagSchemaCatalog::closure_for_hashes`]
//!   keeps a hash→name BINDING for every recorded hash even where it ships no
//!   text (see its doc — that binding is the whole point for a reader trying to
//!   name a channel). Built-ins-only does not mean no attachment:
//!   `cerulion_cli/tests/graph_record_e2e_test.rs`'s
//!   `record_e2e_a_schema_less_workspace_ships_bindings_but_no_text` asserts as
//!   much over a real recording.
//!
//! # What is served, and what is deliberately NOT
//!
//! BUILT-IN types (`sensor_msgs/*`, `geometry_msgs/*`, …) are OMITTED. Every
//! Cerulion binary compiles `native_ros2_messages::BUILTIN_MSGS` in, so shipping
//! them would be bytes nobody reads; the network schema serving already
//! applies the same rule (`cerulion_cli_engine::schema_serve`), and this
//! attachment reuses that module's [`SchemaDoc`] vocabulary verbatim rather than
//! inventing a second one.
//!
//! The `hashes` table is what makes the whole thing work for an ATTACH-mode bag:
//! `cerulion bag record` taps already-running publishers, and a wire frame
//! carries a `schema_hash` and NO name, so every channel it writes is named
//! `"unknown"`. A recorder that can resolve that hash locally stamps the real
//! qualified name into the channel AND records the binding here, so a reader can
//! recover the name even for a type it does not have the text of.

use std::collections::{BTreeMap, BTreeSet};

use cerulion_core::{SchemaDoc, SchemaHashName};
use serde::{Deserialize, Serialize};

use crate::error::{BagError, BagResult};

/// The bag-level attachment carrying [`BagSchemaCatalog`] as JSON.
pub const SCHEMA_DOCS_ATTACHMENT: &str = "__cerulion/schemas.json";

/// The attachment's `media_type`.
pub const SCHEMA_DOCS_MEDIA_TYPE: &str = "application/json";

/// The current [`BagSchemaCatalog::version`].
pub const SCHEMA_CATALOG_VERSION: u32 = 1;

/// The schema provenance a bag carries about ITSELF: the custom-type closure its
/// frames need, plus the hash→name bindings the recorder resolved.
///
/// Both lists are kept in a deterministic order (`docs` by `qualified`, `hashes`
/// by `schema_hash`) so two recordings of the same input produce byte-identical
/// attachment bytes — the bag writer's determinism contract extends here.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BagSchemaCatalog {
    /// The catalog wire version ([`SCHEMA_CATALOG_VERSION`]).
    pub version: u32,
    /// The CUSTOM-type docs (built-ins omitted — every reader has them), each
    /// carrying the qualified names of the other docs it references (`deps`), so
    /// the set is closure-complete and the walk never dangles.
    #[serde(default)]
    pub docs: Vec<SchemaDoc>,
    /// `schema_hash` → qualified name bindings, so a reader can NAME a frame's
    /// type even when it holds no text for it (an attach-mode bag records
    /// `"unknown"` in the channel, because the wire carries no name).
    #[serde(default)]
    pub hashes: Vec<SchemaHashName>,
}

impl BagSchemaCatalog {
    /// Build a catalog, stamping the current [`SCHEMA_CATALOG_VERSION`] and
    /// normalising both lists into their deterministic order.
    pub fn new(docs: Vec<SchemaDoc>, hashes: Vec<SchemaHashName>) -> Self {
        let mut c = Self {
            version: SCHEMA_CATALOG_VERSION,
            docs,
            hashes,
        };
        c.normalize();
        c
    }

    /// An empty catalog — nothing to record. Never written to a bag (see
    /// [`is_empty`](Self::is_empty)).
    pub fn empty() -> Self {
        Self {
            version: SCHEMA_CATALOG_VERSION,
            docs: Vec::new(),
            hashes: Vec::new(),
        }
    }

    /// True when there is nothing to say. The writer skips the attachment
    /// entirely in that case, so an all-built-in bag stays byte-identical to a
    /// pre-attachment recording.
    pub fn is_empty(&self) -> bool {
        self.docs.is_empty() && self.hashes.is_empty()
    }

    /// Sort both lists and drop duplicates (last wins on a repeated key), so the
    /// encoded bytes depend only on the CONTENT, never on the order a caller
    /// happened to accumulate it in.
    fn normalize(&mut self) {
        let mut docs: BTreeMap<String, SchemaDoc> = BTreeMap::new();
        for d in self.docs.drain(..) {
            docs.insert(d.qualified.clone(), d);
        }
        self.docs = docs.into_values().collect();
        let mut hashes: BTreeMap<u64, SchemaHashName> = BTreeMap::new();
        for h in self.hashes.drain(..) {
            hashes.insert(h.schema_hash, h);
        }
        self.hashes = hashes.into_values().collect();
    }

    /// The qualified name bound to `schema_hash`, if the recorder resolved one.
    ///
    /// A LINEAR scan, deliberately: the fields are `pub` (serde), so a caller can
    /// build an unsorted value by hand and a binary search would then answer
    /// `None` for a binding that is right there. The list is one entry per
    /// recorded channel — tens, walked once per channel at open time, never per
    /// frame — so the linear lookup costs nothing worth having a hazard for.
    pub fn name_for_hash(&self, schema_hash: u64) -> Option<&str> {
        self.hashes
            .iter()
            .find(|h| h.schema_hash == schema_hash)
            .map(|h| h.qualified.as_str())
    }

    /// The sub-catalog needed by exactly `used` — the docs for those hashes plus
    /// the transitive `deps` closure of each, and only the hash bindings that
    /// name a type in `used`.
    ///
    /// This is what keeps a bag's attachment proportional to the bag: a robot
    /// whose workspace holds 85 `unitree_go` types but whose recording touches
    /// four of them ships four plus their closure, not all 85.
    ///
    /// A hash with no binding contributes nothing (there is no name to look a
    /// doc up by), and a bound name with no doc contributes its BINDING only —
    /// that is the built-in case, and it is exactly what lets a reader NAME a
    /// channel whose text it already has compiled in.
    pub fn closure_for_hashes(&self, used: impl IntoIterator<Item = u64>) -> Self {
        let by_qualified: BTreeMap<String, SchemaDoc> = self
            .docs
            .iter()
            .map(|d| (d.qualified.clone(), d.clone()))
            .collect();
        let mut wanted_hashes: Vec<SchemaHashName> = Vec::new();
        let mut queue: Vec<String> = Vec::new();
        let mut seen: BTreeSet<String> = BTreeSet::new();
        for hash in used {
            let Some(name) = self.name_for_hash(hash) else {
                continue;
            };
            wanted_hashes.push(SchemaHashName {
                schema_hash: hash,
                qualified: name.to_string(),
            });
            if seen.insert(name.to_string()) {
                queue.push(name.to_string());
            }
        }
        // Transitive `deps` walk over the docs we actually have. A dep naming a
        // type absent from `docs` is skipped — the served set is closure-complete
        // by construction, so this can only happen on a hand-edited bag, and the
        // correct answer there is "that type is undecodable", never a guess.
        let mut docs: Vec<SchemaDoc> = Vec::new();
        while let Some(q) = queue.pop() {
            let Some(doc) = by_qualified.get(&q) else {
                continue;
            };
            for dep in &doc.deps {
                if seen.insert(dep.clone()) {
                    queue.push(dep.clone());
                }
            }
            docs.push(doc.clone());
        }
        Self::new(docs, wanted_hashes)
    }

    /// Encode to the attachment's JSON bytes.
    ///
    /// NORMALIZES first, and that is load-bearing rather than tidy: the fields
    /// are `pub` (serde), so a caller can push docs in any order — or mutate
    /// them after [`new`](Self::new) — and the bag writer's byte-determinism
    /// contract says the file depends on CONTENT, never on the order a caller
    /// accumulated it in. Encoding `self` verbatim would leak that order into
    /// the bag, which is exactly what the integration determinism pin caught.
    /// One clone per bag (cold — written once at finalize).
    pub fn encode(&self) -> BagResult<Vec<u8>> {
        let mut normalized = self.clone();
        normalized.normalize();
        serde_json::to_vec(&normalized).map_err(|e| BagError::SchemaCatalog {
            reason: format!("could not encode it: {e}"),
        })
    }

    /// Decode from the attachment's JSON bytes. An unknown (FUTURE) `version` is
    /// rejected rather than half-read: the fields a newer writer added would be
    /// silently dropped, and a reader claiming provenance it did not understand
    /// is worse than one saying it cannot.
    pub fn decode(bytes: &[u8]) -> BagResult<Self> {
        let mut c: Self = serde_json::from_slice(bytes).map_err(|e| BagError::SchemaCatalog {
            reason: format!("could not decode it: {e}"),
        })?;
        if c.version > SCHEMA_CATALOG_VERSION {
            return Err(BagError::SchemaCatalog {
                reason: format!(
                    "schema catalog version {} is newer than this build reads (v{SCHEMA_CATALOG_VERSION}) \
                     — upgrade cerulion to read this bag's schema provenance",
                    c.version
                ),
            });
        }
        c.normalize();
        Ok(c)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::SchemaEncoding;

    fn doc(q: &str, text: &str, deps: &[&str]) -> SchemaDoc {
        SchemaDoc {
            qualified: q.to_string(),
            encoding: SchemaEncoding::Msg,
            text: text.to_string(),
            deps: deps.iter().map(|d| d.to_string()).collect(),
        }
    }

    fn bind(hash: u64, q: &str) -> SchemaHashName {
        SchemaHashName {
            schema_hash: hash,
            qualified: q.to_string(),
        }
    }

    /// The catalog a `unitree_go`-style robot would hold: a root with two nested
    /// customs, one of which nests a third, plus an unrelated type.
    fn store() -> BagSchemaCatalog {
        BagSchemaCatalog::new(
            vec![
                doc(
                    "go/LowState",
                    "go/Bms bms\ngo/Motor[20] motor\n",
                    &["go/Bms", "go/Motor"],
                ),
                doc("go/Bms", "uint8 soc\ngo/Cell[10] cell\n", &["go/Cell"]),
                doc("go/Cell", "uint16 mv\n", &[]),
                doc("go/Motor", "float32 q\n", &[]),
                doc("go/Unrelated", "int32 x\n", &[]),
            ],
            vec![
                bind(0x11, "go/LowState"),
                bind(0x22, "go/Unrelated"),
                bind(0x33, "sensor_msgs/Image"),
            ],
        )
    }

    #[test]
    fn a_round_trip_is_the_identity_and_the_bytes_are_order_independent() {
        let c = store();
        let decoded = BagSchemaCatalog::decode(&c.encode().unwrap()).unwrap();
        assert_eq!(decoded, c);
        assert_eq!(decoded.version, SCHEMA_CATALOG_VERSION);

        // The SAME content accumulated in a different order encodes to the SAME
        // bytes — the determinism contract the bag writer already promises.
        let mut shuffled_docs = c.docs.clone();
        shuffled_docs.reverse();
        let mut shuffled_hashes = c.hashes.clone();
        shuffled_hashes.reverse();
        let shuffled = BagSchemaCatalog::new(shuffled_docs, shuffled_hashes);
        assert_eq!(
            shuffled.encode().unwrap(),
            c.encode().unwrap(),
            "encoded bytes must depend on content, not accumulation order"
        );
    }

    #[test]
    fn the_closure_walk_prunes_to_exactly_what_the_used_hashes_need() {
        // Hand oracle: /lowstate's hash pulls LowState + Bms + Motor + Cell (Bms's
        // own dep, reached only transitively) and NOTHING else. `go/Unrelated` is
        // in the store but no recorded channel is stamped with it.
        let pruned = store().closure_for_hashes([0x11]);
        let got: Vec<&str> = pruned.docs.iter().map(|d| d.qualified.as_str()).collect();
        assert_eq!(got, vec!["go/Bms", "go/Cell", "go/LowState", "go/Motor"]);
        assert_eq!(pruned.hashes, vec![bind(0x11, "go/LowState")]);

        // Two roots union their closures; the shared type appears ONCE.
        let both = store().closure_for_hashes([0x11, 0x22]);
        let got: Vec<&str> = both.docs.iter().map(|d| d.qualified.as_str()).collect();
        assert_eq!(
            got,
            vec![
                "go/Bms",
                "go/Cell",
                "go/LowState",
                "go/Motor",
                "go/Unrelated"
            ]
        );
    }

    #[test]
    fn a_builtin_hash_contributes_its_binding_but_no_doc() {
        // 0x33 binds to sensor_msgs/Image, which the store deliberately holds NO
        // doc for (built-ins are omitted). The binding must survive — it is what
        // recovers the NAME of an attach-mode channel recorded as "unknown" —
        // while the doc list stays empty.
        let pruned = store().closure_for_hashes([0x33]);
        assert!(pruned.docs.is_empty(), "no text is shipped for a built-in");
        assert_eq!(pruned.hashes, vec![bind(0x33, "sensor_msgs/Image")]);
        assert_eq!(pruned.name_for_hash(0x33), Some("sensor_msgs/Image"));
        assert!(!pruned.is_empty(), "a binding alone is still provenance");
    }

    #[test]
    fn an_unbound_hash_contributes_nothing_and_an_empty_catalog_is_empty() {
        let pruned = store().closure_for_hashes([0xDEAD_BEEF]);
        assert!(pruned.is_empty(), "an unresolvable hash records nothing");
        assert_eq!(pruned.name_for_hash(0xDEAD_BEEF), None);
        assert!(BagSchemaCatalog::empty().is_empty());
        assert!(store().closure_for_hashes([]).is_empty());
    }

    #[test]
    fn a_dangling_dep_is_skipped_rather_than_guessed() {
        // A hand-edited bag whose root names a dep it does not carry: the walk
        // must yield the types it HAS, never fabricate the missing one.
        let c = BagSchemaCatalog::new(
            vec![doc("go/Root", "go/Gone g\n", &["go/Gone"])],
            vec![bind(1, "go/Root")],
        );
        let pruned = c.closure_for_hashes([1]);
        let got: Vec<&str> = pruned.docs.iter().map(|d| d.qualified.as_str()).collect();
        assert_eq!(got, vec!["go/Root"]);
    }

    #[test]
    fn a_dep_cycle_terminates() {
        // Cerulion schemas cannot legally recurse, but a hand-edited bag can say
        // they do; the walk must not spin.
        let c = BagSchemaCatalog::new(
            vec![
                doc("a/A", "a/B b\n", &["a/B"]),
                doc("a/B", "a/A a\n", &["a/A"]),
            ],
            vec![bind(1, "a/A")],
        );
        let pruned = c.closure_for_hashes([1]);
        let got: Vec<&str> = pruned.docs.iter().map(|d| d.qualified.as_str()).collect();
        assert_eq!(got, vec!["a/A", "a/B"]);
    }

    #[test]
    fn a_future_version_is_refused_rather_than_half_read() {
        let mut c = store();
        c.version = SCHEMA_CATALOG_VERSION + 1;
        let bytes = serde_json::to_vec(&c).unwrap();
        let err = BagSchemaCatalog::decode(&bytes).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("newer than this build"), "got: {msg}");
        // Garbage is refused too (never a silent empty catalog).
        assert!(BagSchemaCatalog::decode(b"not json").is_err());
    }

    #[test]
    fn a_duplicate_key_keeps_one_entry() {
        let c = BagSchemaCatalog::new(
            vec![doc("a/A", "first\n", &[]), doc("a/A", "second\n", &[])],
            vec![bind(1, "a/A"), bind(1, "a/B")],
        );
        assert_eq!(c.docs.len(), 1);
        assert_eq!(c.docs[0].text, "second\n");
        assert_eq!(c.hashes.len(), 1);
        assert_eq!(c.name_for_hash(1), Some("a/B"));
    }
}
