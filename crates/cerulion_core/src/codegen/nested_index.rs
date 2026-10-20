// SPDX-License-Identifier: AGPL-3.0-only
//! The one nested-reference binding ladder, on schema identity.
//!
//! A schema's identity is its own `(package, name)` pair, never the text of
//! its qualified name: a package-less schema NAMED `pkg2/Leaf` is
//! `(None, "pkg2/Leaf")`. It is neither in package `pkg2` nor a schema named
//! `Leaf`, even though its qualified text equals that of `(Some("pkg2"),
//! "Leaf")`. Binding on the text would let such a schema take the place of
//! the packaged one (a wrong layout under a reference) or make a reference
//! the resolver bound look unresolvable to a later reader.
//!
//! [`NestedIndex`] is the registry the fixedness resolver, the layout
//! resolver and the frame walker all bind through, so the three can never
//! disagree on what a reference points at. The ladder follows rosidl:
//!
//! 1. a qualified reference (`pkg/Msg`) binds `(Some(pkg), Msg)` exactly;
//! 2. an unqualified one binds the parent's own package first (the parent's
//!    `package` field, `None` for a package-less parent);
//! 3. then bare `Header` binds `std_msgs/Header` (rosidl's legacy case);
//! 4. then the global bare namespace, on each schema's own `name`, iff
//!    exactly one schema carries it (rosidl rejects this step, so the
//!    fixedness resolver warns when it is taken).
//!
//! Duplicate identities resolve "later wins" on every step.
//!
//! Lookups take `&str` keys and do not allocate.

use super::schema::MessageSchema;
use std::collections::BTreeMap;

/// A bound reference: the target's index in the indexed schema slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Bound {
    /// Index of the target schema.
    pub(crate) idx: usize,
    /// The reference bound through the global bare namespace (step 4),
    /// a binding rosidl would reject.
    pub(crate) cross_package_bare: bool,
}

/// Why a reference did not bind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Miss {
    /// No schema carries the referenced identity or bare name.
    Unknown,
    /// An unqualified name that more than one schema carries.
    Ambiguous {
        /// How many schemas carry the bare name.
        candidates: usize,
    },
}

/// Identity registry over one schema slice (see the module docs).
#[derive(Debug, Clone, Default)]
pub(crate) struct NestedIndex {
    /// `package` → `name` → index, for packaged schemas.
    packaged: BTreeMap<String, BTreeMap<String, usize>>,
    /// `name` → index, for package-less schemas.
    loose: BTreeMap<String, usize>,
    /// Each schema's own `name` → every active index carrying it.
    by_bare: BTreeMap<String, Vec<usize>>,
}

impl NestedIndex {
    /// Index `schemas` by identity, later duplicates winning.
    pub(crate) fn from_schemas(schemas: &[MessageSchema]) -> Self {
        let mut index = Self::default();
        for (idx, schema) in schemas.iter().enumerate() {
            index.insert(schema, idx);
        }
        index
    }

    /// Register `schema` at `idx`. Returns the index it shadows when its
    /// identity was already registered (the later definition wins on every
    /// ladder step, the bare namespace included, so a duplicate never
    /// counts twice as an ambiguity).
    pub(crate) fn insert(&mut self, schema: &MessageSchema, idx: usize) -> Option<usize> {
        let slot = match &schema.package {
            Some(pkg) => self
                .packaged
                .entry(pkg.clone())
                .or_default()
                .insert(schema.name.clone(), idx),
            None => self.loose.insert(schema.name.clone(), idx),
        };
        let bare = self.by_bare.entry(schema.name.clone()).or_default();
        match slot {
            Some(prev) => {
                for entry in bare.iter_mut().filter(|e| **e == prev) {
                    *entry = idx;
                }
            }
            None => bare.push(idx),
        }
        slot
    }

    /// The index registered for exactly this identity.
    pub(crate) fn get(&self, package: Option<&str>, name: &str) -> Option<usize> {
        match package {
            Some(pkg) => self.packaged.get(pkg)?.get(name).copied(),
            None => self.loose.get(name).copied(),
        }
    }

    /// True when `schema`, registered at `idx`, is the active (not
    /// shadowed) definition of its identity.
    pub(crate) fn is_active(&self, schema: &MessageSchema, idx: usize) -> bool {
        self.get(schema.package.as_deref(), &schema.name) == Some(idx)
    }

    /// Bind a nested reference `reference_pkg`/`name` written inside a
    /// schema whose own package is `parent_package` (see the module docs
    /// for the ladder).
    pub(crate) fn bind(
        &self,
        reference_pkg: Option<&str>,
        name: &str,
        parent_package: Option<&str>,
    ) -> Result<Bound, Miss> {
        let exact = |idx| Bound {
            idx,
            cross_package_bare: false,
        };
        if let Some(pkg) = reference_pkg {
            return self.get(Some(pkg), name).map(exact).ok_or(Miss::Unknown);
        }
        if let Some(idx) = self.get(parent_package, name) {
            return Ok(exact(idx));
        }
        if name == "Header" {
            if let Some(idx) = self.get(Some("std_msgs"), "Header") {
                return Ok(exact(idx));
            }
        }
        match self.by_bare.get(name).map(Vec::as_slice) {
            Some([idx]) => Ok(Bound {
                idx: *idx,
                cross_package_bare: true,
            }),
            Some(many) if many.len() > 1 => Err(Miss::Ambiguous {
                candidates: many.len(),
            }),
            _ => Err(Miss::Unknown),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn loose(name: &str) -> MessageSchema {
        MessageSchema::new(name)
    }

    fn packaged(pkg: &str, name: &str) -> MessageSchema {
        MessageSchema::new_in_package(name, pkg)
    }

    fn bound(idx: usize, cross_package_bare: bool) -> Result<Bound, Miss> {
        Ok(Bound {
            idx,
            cross_package_bare,
        })
    }

    /// The identity rule itself: a package-less `pkg2/Leaf` is neither in
    /// package `pkg2` nor a bare `Leaf`, and it still binds by its own name.
    #[test]
    fn a_slash_named_package_less_schema_binds_only_on_its_own_name() {
        let index = NestedIndex::from_schemas(&[loose("pkg2/Leaf")]);
        assert_eq!(index.bind(Some("pkg2"), "Leaf", None), Err(Miss::Unknown));
        assert_eq!(index.bind(None, "Leaf", None), Err(Miss::Unknown));
        assert_eq!(index.bind(None, "Leaf", Some("pkg2")), Err(Miss::Unknown));
        assert_eq!(index.bind(None, "pkg2/Leaf", None), bound(0, false));
    }

    /// Beside a real `pkg2/Leaf`, the two identities stay apart and a bare
    /// `Leaf` has exactly one candidate (the packaged one).
    #[test]
    fn a_qualified_text_twin_is_a_separate_identity() {
        let index = NestedIndex::from_schemas(&[loose("pkg2/Leaf"), packaged("pkg2", "Leaf")]);
        assert_eq!(index.bind(Some("pkg2"), "Leaf", None), bound(1, false));
        assert_eq!(index.bind(None, "Leaf", Some("pkg2")), bound(1, false));
        assert_eq!(index.bind(None, "Leaf", None), bound(1, true));
        assert_eq!(index.bind(None, "pkg2/Leaf", None), bound(0, false));
    }

    #[test]
    fn the_ladder_prefers_same_package_then_header_then_unique_bare() {
        let index = NestedIndex::from_schemas(&[
            packaged("a", "Leaf"),
            packaged("b", "Leaf"),
            packaged("std_msgs", "Header"),
            packaged("c", "Only"),
        ]);
        assert_eq!(index.bind(None, "Leaf", Some("b")), bound(1, false));
        assert_eq!(
            index.bind(None, "Leaf", Some("z")),
            Err(Miss::Ambiguous { candidates: 2 })
        );
        assert_eq!(index.bind(None, "Header", Some("z")), bound(2, false));
        assert_eq!(index.bind(None, "Only", None), bound(3, true));
        assert_eq!(index.bind(Some("z"), "Only", None), Err(Miss::Unknown));
    }

    /// A duplicate identity: the later index wins on every step and the
    /// bare namespace counts it once.
    #[test]
    fn a_duplicate_identity_resolves_later_wins_and_counts_once() {
        let leaf = packaged("a", "Leaf");
        let mut index = NestedIndex::default();
        assert_eq!(index.insert(&leaf, 0), None);
        assert_eq!(index.insert(&leaf, 1), Some(0));
        assert_eq!(index.bind(Some("a"), "Leaf", None), bound(1, false));
        assert_eq!(index.bind(None, "Leaf", None), bound(1, true));
        assert!(index.is_active(&leaf, 1));
        assert!(!index.is_active(&leaf, 0));
    }
}
