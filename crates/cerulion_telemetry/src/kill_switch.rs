// SPDX-License-Identifier: AGPL-3.0-only
//! The network kill switch as telemetry reads it.
//!
//! The switch keeps Cerulion service traffic off: robots, relays, hosted
//! Cerulion services and usage telemetry. A surface sends no usage event while
//! it is engaged. A value engages it when it is not blank: `off` after
//! trimming, in any case, and any other value, which fails closed. That is a
//! copy of the blank decision in the parse the CLI's run and query entry
//! points use for `CERULION_NETWORK`; a `cerulion_cli` unit test holds the two
//! equal.

/// The network kill switch every Cerulion surface reads.
pub const NETWORK_ENV: &str = "CERULION_NETWORK";

/// Whether the kill-switch value `value` (`None` when the variable is unset)
/// engages the switch.
pub fn engaged(value: Option<&str>) -> bool {
    value.is_some_and(|value| !value.trim().is_empty())
}

/// [`engaged`] over the process variable `name`. A value that is not Unicode
/// reads as unset, as it does for the CLI.
pub fn engaged_in_env(name: &str) -> bool {
    engaged(std::env::var(name).ok().as_deref())
}

#[cfg(test)]
mod tests {
    use super::engaged;

    #[test]
    fn any_value_that_is_not_blank_engages_the_switch() {
        for value in ["off", "OFF", " Off ", "on", "typo", "0", "false"] {
            assert!(engaged(Some(value)), "{value:?}");
        }
        for value in [None, Some(""), Some("   "), Some("\t\n")] {
            assert!(!engaged(value), "{value:?}");
        }
    }
}
