// SPDX-License-Identifier: AGPL-3.0-only
//! The buffer's name table: every registered frame id stored once, and a
//! fixed-size open-addressed index from a name to its frame slot.
//!
//! Both are sized at construction and never grow: registering a name copies
//! it into spare capacity and fills one index slot, and finding a name
//! hashes it and probes the index. The hash is FNV-1a, fixed and unseeded,
//! so the index layout is the same on every run.

use std::ops::Range;

/// An unused index slot.
const EMPTY: u32 = u32::MAX;

#[derive(Debug)]
pub(crate) struct NameTable {
    /// Every registered name, back to back. Its capacity is at least the
    /// name byte budget, which is never exceeded, so a push never
    /// reallocates.
    bytes: String,
    /// The name byte budget: what `bytes` may hold. `String::with_capacity`
    /// may round its capacity up, so the budget is kept on its own.
    budget: usize,
    /// The byte range in `bytes` of each registered frame, by slot.
    spans: Vec<Range<usize>>,
    /// Open-addressed, linear probing; a power of two at least twice the
    /// frame capacity, so a probe always meets an empty slot.
    index: Vec<u32>,
}

impl NameTable {
    // hot-path-alloc-ok-fn: construction, once per buffer; it sizes the
    // name bytes, the spans and the index so that registering and finding
    // a name never allocate.
    pub(crate) fn with_capacity(frames: usize, name_bytes: usize) -> Self {
        let slots = frames
            .checked_mul(2)
            .and_then(usize::checked_next_power_of_two)
            .expect("the frame capacity fits the name index");
        Self {
            bytes: String::with_capacity(name_bytes),
            budget: name_bytes,
            spans: Vec::with_capacity(frames),
            index: vec![EMPTY; slots],
        }
    }

    /// The name byte budget.
    pub(crate) fn budget(&self) -> usize {
        self.budget
    }

    /// The name bytes not yet used.
    pub(crate) fn spare(&self) -> usize {
        self.budget - self.bytes.len()
    }

    /// How many names are registered.
    pub(crate) fn len(&self) -> usize {
        self.spans.len()
    }

    /// The name of frame slot `slot`.
    pub(crate) fn name(&self, slot: usize) -> &str {
        let span = &self.spans[slot];
        &self.bytes[span.start..span.end]
    }

    /// The frame slot holding `name`, if one does.
    pub(crate) fn find(&self, name: &str) -> Option<usize> {
        let mask = self.index.len() - 1;
        let mut i = hash(name) & mask;
        loop {
            match self.index[i] {
                EMPTY => return None,
                slot if self.name(slot as usize) == name => return Some(slot as usize),
                _ => i = (i + 1) & mask,
            }
        }
    }

    /// Records `name` as the next frame slot and returns it. The caller has
    /// checked that the name is absent, that a frame slot is free and that
    /// the name fits in [`Self::spare`].
    pub(crate) fn register(&mut self, name: &str) -> usize {
        debug_assert!(name.len() <= self.spare() && self.spans.len() < self.spans.capacity());
        let slot = self.spans.len();
        let start = self.bytes.len();
        self.bytes.push_str(name);
        self.spans.push(start..self.bytes.len());
        let mask = self.index.len() - 1;
        let mut i = hash(name) & mask;
        while self.index[i] != EMPTY {
            i = (i + 1) & mask;
        }
        // At most the frame capacity, which the constructor bounds by u32.
        self.index[i] = slot as u32;
        slot
    }
}

/// 64-bit FNV-1a, folded to `usize`.
fn hash(name: &str) -> usize {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in name.as_bytes() {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h as usize
}

#[cfg(test)]
mod tests {
    use super::hash;

    /// The name index hashes with 64-bit FNV-1a: the published vectors for
    /// the empty string (the offset basis) and `a`, and `odom` worked from
    /// the same definition. A changed fold (`|` or `&` in place of `^`)
    /// still builds a working table, so only the value pins the function
    /// the probe sequence depends on.
    #[test]
    fn the_name_hash_is_fnv1a_64() {
        assert_eq!(hash(""), 0xcbf2_9ce4_8422_2325_u64 as usize);
        assert_eq!(hash("a"), 0xaf63_dc4c_8601_ec8c_u64 as usize);
        assert_eq!(hash("odom"), 0x5d77_84b4_9bb7_c03a_u64 as usize);
    }
}
