// SPDX-License-Identifier: AGPL-3.0-only
//! Named oracle mutants. Under the `oracle-mutants` feature, [`active`] is
//! `true` for the one name `CERULION_ORACLE_MUTANT` holds, and the site that
//! asks breaks its rule; without the feature it is a constant `false`.

/// The variable that names the active mutant.
pub const ORACLE_MUTANT_ENV: &str = "CERULION_ORACLE_MUTANT";

/// Is the mutant `name` active in this process?
#[cfg(feature = "oracle-mutants")]
pub fn active(name: &str) -> bool {
    std::env::var(ORACLE_MUTANT_ENV).is_ok_and(|v| v == name)
}

/// Is the mutant `name` active in this process? Always `false` without the
/// `oracle-mutants` feature.
#[cfg(not(feature = "oracle-mutants"))]
#[inline(always)]
pub fn active(_name: &str) -> bool {
    false
}
