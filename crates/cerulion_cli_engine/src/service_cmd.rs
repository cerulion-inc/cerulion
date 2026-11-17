// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion service call`: send one request to a service and print its
//! response as one JSON document.
//!
//! The call itself is [`cerulion_msgio::service`] (the protocol
//! `rmw_cerulion` serves ROS 2 services with); this module resolves the
//! service type against the workspace the way every schema surface of the
//! CLI does: the built-in corpus, the `.msg` store and `schemas/*.yaml`,
//! preflighted, plus the store's `schemas/<pkg>/srv/<Name>.srv`.
//!
//! The server must run on this machine: the request and reply topics are
//! local shared memory.

use std::path::Path;
use std::sync::atomic::AtomicBool;

use cerulion_core::codegen::FrameWalker;

pub use cerulion_msgio::service::{
    check_service_name, CallRequest, ServiceTypes, DEFAULT_CALL_TIMEOUT, MEMBER_PLACEHOLDER,
};

use crate::message_text::VerbError;

/// Resolve the service type `type_name` from the workspace store, laid out
/// over the workspace's resolution set.
pub fn resolve_service_types(
    type_name: &str,
    schemas_dir: Option<&Path>,
) -> Result<ServiceTypes, VerbError> {
    let base = schemas_dir
        .map(|dir| crate::schema_cmd::resolution_schema_set(Some(dir)))
        .unwrap_or_default();
    cerulion_msgio::service::resolve_service_types(type_name, schemas_dir, base, |mut schemas| {
        crate::schema_cmd::preflight_resolution_set(&mut schemas, "service call schema set");
        let (walker, warnings) = FrameWalker::new(schemas);
        for w in warnings {
            tracing::debug!(warning = %w, "service call walker: resolution warning");
        }
        Ok(walker)
    })
}

/// Call the service once and return its response as one line of JSON (see
/// [`cerulion_msgio::service::service_call`] for the error classes).
pub fn service_call(
    req: &CallRequest<'_>,
    schemas_dir: Option<&Path>,
    running: &AtomicBool,
) -> Result<String, VerbError> {
    check_service_name(req.service)?;
    let types = resolve_service_types(req.type_name, schemas_dir)?;
    cerulion_msgio::service::service_call(req, &types, running)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_workspace_resolution_and_the_checked_store_resolution_agree_on_the_hashes() {
        let dir = tempfile::tempdir().expect("tempdir");
        let srv = dir.path().join("demo_msgs/srv");
        std::fs::create_dir_all(&srv).unwrap();
        std::fs::create_dir_all(dir.path().join("demo_msgs/msg")).unwrap();
        std::fs::write(dir.path().join("demo_msgs/msg/Item.msg"), "string name\n").unwrap();
        std::fs::write(srv.join("List.srv"), "---\nItem[] items\n").unwrap();
        let engine = resolve_service_types("demo_msgs/srv/List", Some(dir.path())).unwrap();
        let store = cerulion_msgio::service::resolve_store_service_types(
            "demo_msgs/srv/List",
            Some(dir.path()),
        )
        .unwrap();
        for part in ["demo_msgs/List_Request", "demo_msgs/List_Response"] {
            assert!(engine.walker.schema_hash_for(part).is_some(), "{part}");
            assert_eq!(
                engine.walker.schema_hash_for(part),
                store.walker.schema_hash_for(part),
                "{part}"
            );
        }
    }
}
