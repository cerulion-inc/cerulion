// SPDX-License-Identifier: AGPL-3.0-only
//! Messages as JSON or YAML text: [`cerulion_msgio::text`], shared with
//! `cerulion-remoted`, plus the conversion of an engine error into its
//! exit-code class.

pub use cerulion_msgio::text::*;

impl From<crate::error::CliError> for VerbError {
    fn from(e: crate::error::CliError) -> Self {
        VerbError::Internal(crate::error::render_user_error(&e))
    }
}
