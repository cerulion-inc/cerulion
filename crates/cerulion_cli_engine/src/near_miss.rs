// SPDX-License-Identifier: AGPL-3.0-only
//! The near-miss key warn, for hand-authored config files
//! whose FOREIGN sibling keys must stay legal.
//!
//! Two mechanisms cover unknown keys in this tree, and which one applies is a
//! property of the FILE rather than a preference:
//!
//! * `#[serde(deny_unknown_fields)]` where the document is ours end to end —
//!   graph YAML, the cost snapshot, the replay tolerance
//!   document. A typo there is a HARD parse error.
//! * this module where it is not. `~/.cerulion/config.toml` is named for
//!   general configuration and exactly ONE key is read from it today;
//!   `~/.cerulion/robots.toml` is a NEVER-BRICKS store, named as such in
//!   `crates/cerulion_core/tests/config_deny_unknown_fields_test.rs`'s own
//!   "deliberately NOT on the list" inventory alongside `auth` and
//!   `peers.json`. Denying an unknown key in either would refuse a file over
//!   a key we simply do not consume — and on `robots.toml` it would break the
//!   discipline `connect_cmd` states one function away, where a malformed
//!   document warns and yields an empty map so the user can still pass
//!   `--eid`.
//!
//! What the silence cost was not hypothetical: `peer = [...]` or `Peers =
//! [...]` in `config.toml` parses clean and yields NO peers, so the discovery
//! ladder's hostname rung goes quiet with nothing said; `[robot]` in
//! `robots.toml` yields an empty map the same way.
//!
//! The predicate is `system_deps::near_miss_cerulion_keys`' generalized: a key
//! is a near miss of a known one when it is within [`MAX_EDITS`] single-character
//! edits of it. That module keeps its own `contains`-based half (a SUFFIXED key
//! like `cerulion-deps` is many edits from `cerulion` but obviously means it),
//! which is specific to a namespaced `package.metadata` table and does not
//! generalize to a two-letter key like `peers`.

/// Edit distance at which a key is reported as a probable misspelling. Two is
/// `near_miss_cerulion_keys`' bound and the same reasoning applies: one covers
/// a dropped/added/wrong letter, two covers a transposition plus a slip, and
/// three starts matching unrelated short keys.
pub(crate) const MAX_EDITS: usize = 2;

/// Every key in `actual` that is a near miss of some key in `known`, paired
/// with the key it probably meant. An EXACT match is never a near miss.
///
/// Deterministic: `actual`'s order is preserved and, where a key is close to
/// several known ones, the FIRST in `known` wins — so a diagnostic does not
/// depend on hash order.
pub(crate) fn near_miss_keys<'a>(
    actual: impl IntoIterator<Item = &'a str>,
    known: &[&'a str],
) -> Vec<(String, &'a str)> {
    actual
        .into_iter()
        .filter(|k| !known.contains(k))
        .filter_map(|k| {
            known
                .iter()
                .find(|want| levenshtein_at_most(k, want, MAX_EDITS))
                .map(|want| (k.to_string(), *want))
        })
        .collect()
}

/// `true` when `a` and `b` are within `max` single-character edits
/// (insert/delete/substitute). Plain full-matrix Levenshtein over `char`s with
/// a length-difference short circuit — the inputs are config keys, so the cost
/// never matters and a clever bounded variant would only add ways to be wrong.
///
/// Shared with [`crate::system_deps::near_miss_cerulion_keys`] rather than
/// written a second time (the rule).
pub(crate) fn levenshtein_at_most(a: &str, b: &str, max: usize) -> bool {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    if a.len().abs_diff(b.len()) > max {
        return false;
    }
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    let mut cur = vec![0usize; b.len() + 1];
    for (i, ca) in a.iter().enumerate() {
        cur[0] = i + 1;
        for (j, cb) in b.iter().enumerate() {
            let sub = prev[j] + usize::from(ca != cb);
            cur[j + 1] = sub.min(prev[j + 1] + 1).min(cur[j] + 1);
        }
        std::mem::swap(&mut prev, &mut cur);
    }
    prev[b.len()] <= max
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Hand vectors on BOTH sides. The interesting failure is a matcher that
    /// is too EAGER — every unrelated key it claims is a misspelling is a
    /// warning an operator has to dismiss, which is how a diagnostic stops
    /// being read.
    #[test]
    fn a_near_miss_is_reported_and_an_unrelated_key_is_not() {
        // The issue's own named spellings for `peers`, plus a transposition.
        assert_eq!(
            near_miss_keys(["peer"], &["peers"]),
            vec![("peer".to_string(), "peers")]
        );
        assert_eq!(
            near_miss_keys(["Peers"], &["peers"]),
            vec![("Peers".to_string(), "peers")]
        );
        assert_eq!(
            near_miss_keys(["peerz"], &["peers"]),
            vec![("peerz".to_string(), "peers")]
        );
        assert_eq!(
            near_miss_keys(["pers"], &["peers"]),
            vec![("pers".to_string(), "peers")]
        );
        // The issue's named spelling for `robots`.
        assert_eq!(
            near_miss_keys(["robot"], &["robots"]),
            vec![("robot".to_string(), "robots")]
        );

        // An EXACT key is never a near miss — the whole point is that the
        // correct spelling stays silent.
        assert!(near_miss_keys(["peers"], &["peers"]).is_empty());

        // Genuinely foreign keys stay legal AND silent. These are the reason
        // this file uses a warn rather than `deny_unknown_fields` at all, so a
        // matcher that flagged them would defeat the choice.
        for foreign in ["theme", "editor", "log_level", "network", "aliases"] {
            assert!(
                near_miss_keys([foreign], &["peers", "robots"]).is_empty(),
                "`{foreign}` is a foreign key, not a misspelling"
            );
        }
    }

    /// Determinism: input order preserved, and a key close to several known
    /// ones resolves to the first in `known` rather than to hash order.
    #[test]
    fn near_miss_output_is_deterministic() {
        let known = ["peers", "beers"];
        assert_eq!(
            near_miss_keys(["zeers", "aaaa", "peerz"], &known),
            vec![
                ("zeers".to_string(), "peers"),
                ("peerz".to_string(), "peers"),
            ]
        );
    }

    /// The distance bound, pinned on both sides: at `MAX_EDITS` a key is a
    /// near miss, one edit further it is not.
    #[test]
    fn the_edit_bound_is_pinned_on_both_sides() {
        assert_eq!(MAX_EDITS, 2, "the vectors below are written for 2 edits");
        // ONE edit (a dropped letter).
        assert!(levenshtein_at_most("pers", "peers", MAX_EDITS));
        // Exactly TWO (two dropped letters) — the boundary, INSIDE.
        assert!(levenshtein_at_most("prs", "peers", MAX_EDITS));
        // THREE (three substitutions, same length so the short circuit below
        // is not what refuses it) — the boundary, OUTSIDE.
        assert!(!levenshtein_at_most("xyzrs", "peers", MAX_EDITS));
        // The length short circuit refuses before the matrix runs.
        assert!(!levenshtein_at_most("p", "peers", MAX_EDITS));
        // Identity, at distance zero.
        assert!(levenshtein_at_most("peers", "peers", 0));
    }
}
