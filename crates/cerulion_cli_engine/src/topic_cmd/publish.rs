// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion topic pub`: publish a message given as JSON or YAML text, the
//! verb a ROS 2 user knows as `ros2 topic pub`.
//!
//! The type resolves as `topic echo` and `schema info` resolve one: the
//! built-in ROS 2 corpus, then the workspace `.msg` store and
//! `schemas/*.yaml`, then (outside `--local`) a robot on the LAN that serves
//! it. The text becomes one wire frame ([`crate::message_text`]), published
//! into local shared memory. Outside `--local` a topic whose name starts
//! with `/` is also registered with `cerulion-netd`, which announces it on
//! the LAN; when a remote peer demands it, netd's tap subscribes to it and
//! forwards the frames, and that tap counts as one of the topic's
//! subscribers.

use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::time::Duration;

use cerulion_core::codegen::FrameWalker;

pub use cerulion_msgio::publish::{
    publish_period, PubReport, DEFAULT_REPEAT_RATE_HZ, DEFAULT_WAIT_SUBSCRIBERS,
};

use super::{local_walker_from_workspace, TopicScope};
use crate::message_text::{qualified_type_name, InterfaceKind, VerbError};

/// What `topic pub` was asked to do.
#[derive(Debug, Clone)]
pub struct PubRequest<'a> {
    /// The topic, as ROS spells it (`/cmd_vel`).
    pub topic: &'a str,
    /// `pkg/Type`, `pkg/msg/Type` or a bare workspace `Type`.
    pub type_name: &'a str,
    /// The message as a JSON or YAML mapping.
    pub message: &'a str,
    /// How many frames to publish (at least 1).
    pub times: u64,
    /// The publish rate in Hz; `None` is [`DEFAULT_REPEAT_RATE_HZ`].
    pub rate_hz: Option<f64>,
    /// How long to wait for a first subscriber before publishing.
    pub wait_subscribers: Duration,
    /// Local only, or local plus the LAN.
    pub scope: TopicScope,
}

/// A walker that knows the message type `name`: the local corpus, else (in
/// automatic scope) the closure a robot on the LAN serves for it.
pub fn resolve_message_walker(
    name: &str,
    schemas_dir: Option<&Path>,
    scope: TopicScope,
) -> Result<FrameWalker, VerbError> {
    let walker = local_walker_from_workspace(schemas_dir);
    if walker.knows(name) {
        return Ok(walker);
    }
    if !scope.local_only() && name.contains('/') {
        let opts = super::RemoteTopicsOptions {
            connect: vec![],
            listen: vec![],
            scouting: true,
        };
        if let Ok(super::RemoteSchemaFetch::Found(reply)) = super::fetch_remote_schema(name, &opts)
        {
            let remote = super::seed_framewalker(&reply.docs);
            if remote.knows(name) {
                return Ok(remote);
            }
        }
    }
    Err(VerbError::Usage(format!(
        "unknown message type '{name}': it is not a built-in ROS 2 type, not in this \
         workspace's schemas/ (a .msg file goes in schemas/<pkg>/msg/<Type>.msg){}",
        if scope.local_only() {
            ""
        } else {
            ", and no robot on the LAN served it"
        }
    )))
}

/// Publish `req.message` on `req.topic` ([`cerulion_msgio::publish`]) and
/// report what was sent. Outside `--local` a `/`-rooted topic is registered
/// with `cerulion-netd` while it publishes.
pub fn topic_pub(
    req: &PubRequest<'_>,
    schemas_dir: Option<&Path>,
    running: &AtomicBool,
) -> Result<PubReport, VerbError> {
    publish_period(req.times, req.rate_hz)?;
    let name = qualified_type_name(req.type_name, InterfaceKind::Message)?;
    let walker = resolve_message_walker(&name, schemas_dir, req.scope)?;
    let core = cerulion_msgio::publish::PubRequest {
        topic: req.topic,
        message: req.message,
        times: req.times,
        rate_hz: req.rate_hz,
        wait_subscribers: req.wait_subscribers,
    };
    let scope = req.scope;
    let _exclusion = if scope.local_only() {
        Some(
            cerulion_core::TransportManager::get_or_init()
                .and_then(|manager| manager.exclude_remote_topic(req.topic))
                .map_err(|error| VerbError::Unavailable(error.to_string()))?,
        )
    } else {
        None
    };
    cerulion_msgio::publish::publish_message(&core, &walker, &name, running, |schema_hash| {
        if scope.local_only() {
            None
        } else if req.topic.starts_with('/') {
            register_egress(req.topic, &name, schema_hash)
        } else {
            tracing::warn!(
                topic = %req.topic,
                "the topic name does not start with '/', so it is published locally only"
            );
            None
        }
    })
}

/// Register `topic` with `cerulion-netd` as this machine's egress, so the
/// shared gateway announces it and serves a remote demand for it. The
/// registration lasts while the returned client is held. A failure is a
/// warning: the local publish goes ahead.
#[cfg(unix)]
fn register_egress(
    topic: &str,
    schema_name: &str,
    schema_hash: u64,
) -> Option<cerulion_netd::NetdClient> {
    use cerulion_core::{
        GatewayEgressPolicy, GatewayPlan, SchemaHashName, SchemaServing, TopicSchema,
    };
    if cerulion_core::LanAccess::from_env() == cerulion_core::LanAccess::Paired {
        tracing::warn!(
            topic = %topic,
            "paired LAN access is required, so the topic is published locally only"
        );
        return None;
    }
    let canonical = topic.to_string();
    let plan = GatewayPlan {
        egress_policy: GatewayEgressPolicy::AllowAll,
        announce: vec![canonical.clone()],
        ingress: vec![],
    };
    let serving = SchemaServing {
        topic_schemas: vec![TopicSchema {
            topic: canonical.clone(),
            schema_name: schema_name.to_string(),
        }],
        schema_docs: vec![],
        schema_hashes: vec![SchemaHashName {
            schema_hash,
            qualified: schema_name.to_string(),
        }],
        served_topics: vec![canonical],
    };
    let registered = cerulion_netd::NetdClient::connect_or_spawn().and_then(|mut client| {
        client.register_egress(&plan, &serving, None)?;
        Ok(client)
    });
    match registered {
        Ok(client) => Some(client),
        Err(e) => {
            tracing::warn!(
                topic = %topic,
                error = %e,
                "could not register the topic with cerulion-netd; it is published locally only"
            );
            None
        }
    }
}

/// Non-Unix: there is no `cerulion-netd`, so the topic is local only.
#[cfg(not(unix))]
fn register_egress(_topic: &str, _schema_name: &str, _schema_hash: u64) -> Option<()> {
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_unknown_type_under_local_scope_is_a_usage_error() {
        let err = resolve_message_walker("nope_msgs/Nope", None, TopicScope::Local)
            .err()
            .unwrap();
        assert_eq!(err.exit_code(), crate::message_text::EXIT_USAGE);
        assert!(err.to_string().contains("nope_msgs/Nope"), "{err}");
    }
}
