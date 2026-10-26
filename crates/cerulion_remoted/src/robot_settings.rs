// SPDX-License-Identifier: AGPL-3.0-only
//! Owner-controlled robot settings served on the ops plane.
//!
//! `cloud-access` reads or sets the allow-from-cloud switch kept in the MAC'd
//! trust store. The authorizer classifies it owner-role only; the handler also
//! requires the caller's own row to be an owner-account, non-cloud row
//! ([`SharedTrust::set_allow_cloud`]).

use cerud::error::{CerudError, CerudResult};
use cerud::transport::CallerIdentity;
use cerud::verbs::VerbHandler;

use crate::pairing_verbs::caller_device_key;
use crate::trust::SharedTrust;
use crate::verbs;

/// `cloud-access`: `{"allow": bool}` sets the switch; `{}` reads it. The reply
/// is `{"allow_cloud": bool}` with the value now in force.
pub struct CloudAccessVerb {
    shared: SharedTrust,
}

impl CloudAccessVerb {
    /// A handler over the live trust state.
    pub fn new(shared: SharedTrust) -> Self {
        Self { shared }
    }
}

impl VerbHandler for CloudAccessVerb {
    fn name(&self) -> &'static str {
        verbs::CLOUD_ACCESS
    }
    fn is_mutating(&self) -> bool {
        true // `allow` persists the switch; intent-receipt-before-effect
    }
    fn execute(&self, _args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        Err(CerudError::Verb(format!(
            "verb '{}' requires a transport-authenticated caller",
            verbs::CLOUD_ACCESS
        )))
    }
    fn execute_with_caller(
        &self,
        caller: &CallerIdentity,
        args: &serde_json::Value,
    ) -> CerudResult<serde_json::Value> {
        let device_key = caller_device_key(caller)?;
        match args.get("allow") {
            None => {}
            Some(serde_json::Value::Bool(allow)) => self
                .shared
                .set_allow_cloud(&device_key, *allow)
                .map_err(|e| CerudError::Verb(format!("cloud-access refused: {e}")))?,
            Some(_) => {
                return Err(CerudError::Verb(
                    "cloud-access: 'allow' must be true or false".into(),
                ))
            }
        }
        Ok(serde_json::json!({ "allow_cloud": self.shared.allow_cloud() }))
    }
}
