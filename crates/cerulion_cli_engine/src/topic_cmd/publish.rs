// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion topic pub`: publish a message given as JSON or YAML text, the
//! verb a ROS 2 user knows as `ros2 topic pub`.
//!
//! The type resolves as `topic echo` and `schema info` resolve one: the
//! built-in ROS 2 corpus, then the workspace `.msg` store and
//! `schemas/*.yaml`, then (outside `--local`) a robot on the LAN that serves
//! it. The text becomes one wire frame ([`crate::message_text`]), published
//! into local shared memory. Outside `--local` a topic whose name starts
//! with `/` is also registered with `cerulion-netd` under paired access by
//! default; when an authorized remote peer demands it, the tap subscribes and
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
    /// Local only, or local plus authorized remote access.
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
/// shared gateway serves an authorized remote demand for it. The
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
    let requested = cerulion_core::LanAccess::from_env();
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
        let access = register_topic_plan(&mut client, &plan, &serving, requested)?;
        Ok((client, access))
    });
    match registered {
        Ok((client, access)) => {
            if access == Some(cerulion_core::LanAccess::Open) {
                use crate::graph_cmd::PERMISSIVE_EGRESS_NOTICE;
                tracing::warn!(topic = %topic, "{PERMISSIVE_EGRESS_NOTICE}");
            }
            Some(client)
        }
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

/// Check the serving process before registering any exportable topic.
#[cfg(unix)]
fn register_topic_plan(
    client: &mut cerulion_netd::NetdClient,
    plan: &cerulion_core::GatewayPlan,
    serving: &cerulion_core::SchemaServing,
    requested: cerulion_core::LanAccess,
) -> Result<Option<cerulion_core::LanAccess>, cerulion_netd::ClientError> {
    let reported = client.lan_access()?;
    if !crate::graph_cmd::netd_posture_acceptable(reported, requested) {
        return Err(cerulion_netd::ClientError::Protocol(
            "paired topic access requires a daemon reporting paired access; restart cerulion-netd with paired access".into(),
        ));
    }
    let response = client.register_egress(plan, serving, None)?;
    if !crate::graph_cmd::netd_posture_acceptable(response.lan_access, requested) {
        return Err(cerulion_netd::ClientError::Protocol(
            "cerulion-netd no longer reports the required paired topic access".into(),
        ));
    }
    Ok(response.lan_access)
}

/// Non-Unix: there is no `cerulion-netd`, so the topic is local only.
#[cfg(not(unix))]
fn register_egress(_topic: &str, _schema_name: &str, _schema_hash: u64) -> Option<()> {
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(unix)]
    #[test]
    fn topic_egress_checks_the_daemon_posture_before_registration() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::Arc;

        use cerulion_core::{GatewayPlan, LanAccess, SchemaServing};
        use cerulion_netd::egress::{EgressError, EgressId, EgressPlane};
        use cerulion_netd::mirror::{MirrorError, MirrorPlane, MirrorRelease};
        use cerulion_netd::registry::TopicKey;

        struct NoMirror;
        impl MirrorPlane for NoMirror {
            fn ensure_mirror(&self, _: &TopicKey, _: u64) -> Result<(), MirrorError> {
                panic!("topic publication must not demand a mirror")
            }
            fn release_mirror(&self, _: &TopicKey) -> MirrorRelease {
                panic!("topic publication must not release a mirror")
            }
        }

        struct CountingEgress {
            posture: Option<LanAccess>,
            registrations: AtomicUsize,
            open_after_registration: bool,
        }
        impl EgressPlane for CountingEgress {
            fn register_egress(
                &self,
                _: EgressId,
                plan: &GatewayPlan,
                _: &SchemaServing,
                _: Option<&str>,
            ) -> Result<bool, EgressError> {
                assert_eq!(plan.announce, ["/paired/publish"]);
                self.registrations.fetch_add(1, Ordering::SeqCst);
                Ok(true)
            }
            fn lan_access(&self) -> Option<LanAccess> {
                if self.open_after_registration && self.registrations.load(Ordering::SeqCst) > 0 {
                    Some(LanAccess::Open)
                } else {
                    self.posture
                }
            }
            fn release_egress(&self, _: EgressId) {}
        }

        for (reported, requested, allowed, opens) in [
            (Some(LanAccess::Paired), LanAccess::Paired, true, false),
            (Some(LanAccess::Open), LanAccess::Paired, false, false),
            (None, LanAccess::Paired, false, false),
            (Some(LanAccess::Paired), LanAccess::Open, true, false),
            (Some(LanAccess::Open), LanAccess::Open, true, false),
            (None, LanAccess::Open, true, false),
            (Some(LanAccess::Paired), LanAccess::Paired, false, true),
        ] {
            let root = tempfile::tempdir().expect("test root");
            let socket = root.path().join("netd.sock");
            let plane = Arc::new(CountingEgress {
                posture: reported,
                registrations: AtomicUsize::new(0),
                open_after_registration: opens,
            });
            let mut daemon = cerulion_netd::daemon::start_with_egress(
                socket.clone(),
                Arc::new(NoMirror),
                Arc::clone(&plane) as Arc<dyn EgressPlane>,
                cerulion_netd::daemon::NetdConfig::default(),
            )
            .expect("daemon starts");
            let mut client =
                cerulion_netd::NetdClient::connect_existing_at(socket).expect("client connects");
            let plan = GatewayPlan {
                egress_policy: cerulion_core::GatewayEgressPolicy::AllowAll,
                announce: vec!["/paired/publish".into()],
                ingress: vec![],
            };
            for attempt in 1..=2 {
                let result =
                    register_topic_plan(&mut client, &plan, &SchemaServing::default(), requested);
                assert_eq!(result.is_ok(), allowed, "{reported:?}/{requested:?}");
                if let Err(error) = result {
                    let expected = if opens && attempt == 1 {
                        "cerulion-netd protocol error: cerulion-netd no longer reports the required paired topic access"
                    } else {
                        "cerulion-netd protocol error: paired topic access requires a daemon reporting paired access; restart cerulion-netd with paired access"
                    };
                    assert_eq!(error.to_string(), expected);
                }
                assert_eq!(
                    plane.registrations.load(Ordering::SeqCst),
                    if opens {
                        1
                    } else if allowed {
                        attempt
                    } else {
                        0
                    },
                    "a refused posture must register no topic"
                );
            }
            drop(client);
            daemon.shutdown();
        }
    }

    #[test]
    fn an_unknown_type_under_local_scope_is_a_usage_error() {
        let err = resolve_message_walker("nope_msgs/Nope", None, TopicScope::Local)
            .err()
            .unwrap();
        assert_eq!(err.exit_code(), crate::message_text::EXIT_USAGE);
        assert!(err.to_string().contains("nope_msgs/Nope"), "{err}");
    }
}
