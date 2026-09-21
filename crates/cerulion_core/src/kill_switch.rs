// SPDX-License-Identifier: AGPL-3.0-only
//! The ONE env kill-switch GRAMMAR shared by every Cerulion
//! `CERULION_*` boolean switch — `0` disables, unset/empty/`1` enable, and
//! anything else keeps the default ON while flagging the caller's loud warn.
//!
//! The grammar lived in [`crate::os_sync`] until the
//! credit plane's `CERULION_CREDIT_WAKE` became its FIRST cross-platform
//! consumer: direction-B producer wake is not a macOS feature, so a grammar
//! parked behind `#[cfg(target_os = "macos")]` did not compile on Linux at all
//! — the primary robot target. It is a PURE function with no platform
//! dependency, so the fix is structural rather than a second copy: it lives
//! here, in a platform-neutral module, and `os_sync` re-exports it under its
//! own name for the two macOS switches that predate this move.
//!
//! One definition is the point. Four switches parse through this fn —
//! `CERULION_BARRIER_OS_SYNC` ([`crate::barrier`]), `CERULION_PARK_OS_SYNC`
//! ([`crate::monitor_wait`]), `CERULION_CREDIT_WAKE` and
//! `CERULION_CREDIT_OS_SYNC` ([`crate::credit`]) — so they cannot drift in
//! what they ACCEPT. Each consumer still owns its OWN resolve-and-warn
//! wrapper, because the warn has to name ITS variable: a switch whose
//! diagnostic points at a sibling's env name is the misleading-surface class
//! this repo rejects.
//!
//! Pure, and pinned on EVERY platform. The oracle below is the one place the
//! grammar is fixed; while it sat in the macOS-only module it ran on macOS
//! alone, so a Linux-only regression in what these switches accept was
//! untestable by construction.

/// Pure parse of a `CERULION_*` kill-switch env value → `(disabled,
/// was_garbage)`.
///
/// unset / empty / `"1"` → `(false, false)` (the feature stays ON, the
/// default); `"0"` → `(true, false)` (the explicit kill switch); anything else
/// → `(false, true)` (keep the default ON, flagged for the caller's loud
/// warn).
///
/// Matching is EXACT — no trimming and no case folding, the same discipline as
/// the barrier's `parse_barrier_spin_us`. `" 0"` and `"00"` are garbage, not a
/// disable: a switch that guessed at a near-miss would silently turn a feature
/// off on a typo, and an operator who meant `0` gets told so by the warn.
pub(crate) fn parse_kill_switch(raw: Option<&str>) -> (bool, bool) {
    match raw {
        None | Some("") | Some("1") => (false, false),
        Some("0") => (true, false),
        Some(_) => (false, true),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Moved here from `os_sync`: the shared kill-switch
    /// GRAMMAR. Pure oracle over `(disabled, was_garbage)` — no env mutation.
    ///
    /// All four consumers' resolve wrappers parse through THIS fn, so this is
    /// the one place the grammar is pinned. It runs on every platform now; in
    /// `os_sync` it was macOS-gated along with the module, which left the
    /// grammar that governs a Linux-first switch (`CERULION_CREDIT_WAKE`)
    /// unpinned on Linux.
    #[test]
    fn kill_switch_parse_oracle() {
        for (raw, disabled, garbage) in [
            (None, false, false),
            (Some(""), false, false),
            (Some("1"), false, false),
            (Some("0"), true, false),
            (Some("2"), false, true),
            (Some("true"), false, true),
            (Some("off"), false, true),
            (Some(" 0"), false, true), // EXACT match — no trim
            (Some("00"), false, true),
        ] {
            assert_eq!(
                parse_kill_switch(raw),
                (disabled, garbage),
                "parse_kill_switch({raw:?}) must be ({disabled}, {garbage})"
            );
        }
    }
}
