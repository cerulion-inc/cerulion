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
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use cerulion_core::codegen::FrameWalker;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::ReinjectOutcome;

use super::{local_walker_from_workspace, TopicScope};
use crate::message_text::{message_frame, qualified_type_name, InterfaceKind, VerbError};

/// The default `--wait-subscribers`: how long `topic pub` waits for a
/// subscriber before it publishes.
pub const DEFAULT_WAIT_SUBSCRIBERS: Duration = Duration::from_secs(2);

/// The rate `--times N` (N > 1) publishes at when `--rate` is not given.
pub const DEFAULT_REPEAT_RATE_HZ: f64 = 1.0;

/// How long the publisher stays open after its last frame, so a subscriber
/// (netd's network tap among them) drains it before the publisher goes.
const LINGER: Duration = Duration::from_millis(500);

/// How often the subscriber wait checks the topic's subscriber count.
const SUBSCRIBER_POLL: Duration = Duration::from_millis(20);

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

/// What `topic pub` did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PubReport {
    /// The topic published to.
    pub topic: String,
    /// The type's Cerulion name (`pkg/Type`).
    pub type_name: String,
    /// Frames published.
    pub frames: u64,
    /// Subscribers the last frame was delivered to.
    pub subscribers: usize,
}

impl PubReport {
    /// The one line `topic pub` prints.
    pub fn line(&self) -> String {
        format!(
            "published: topic={} type={} frames={} subscribers={}",
            self.topic, self.type_name, self.frames, self.subscribers
        )
    }
}

/// The interval between frames for `times` frames at `rate_hz`; zero for a
/// single frame. A rate that is not a finite positive number is a usage
/// error.
pub fn publish_period(times: u64, rate_hz: Option<f64>) -> Result<Duration, VerbError> {
    let rate = rate_hz.unwrap_or(DEFAULT_REPEAT_RATE_HZ);
    if !(rate.is_finite() && rate > 0.0) {
        return Err(VerbError::Usage(format!(
            "--rate must be a positive number of Hz, got {rate}"
        )));
    }
    if times == 0 {
        return Err(VerbError::Usage("--times must be at least 1".to_string()));
    }
    if times == 1 {
        return Ok(Duration::ZERO);
    }
    Duration::try_from_secs_f64(1.0 / rate)
        .map_err(|_| VerbError::Usage(format!("--rate {rate} gives no usable interval")))
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

/// Publish `req.message` on `req.topic` and report what was sent.
///
/// The message is encoded before anything is opened, so a bad value fails
/// with no side effect. The verb then waits up to `req.wait_subscribers` for
/// a subscriber, publishes `req.times` frames at the requested rate (each
/// with the next sequence number and the transport clock's stamp), and keeps
/// the publisher open [`LINGER`] longer. With no subscriber after the wait
/// it publishes anyway and reports `subscribers=0`. Ctrl-C stops between
/// frames and reports the frames sent; before the first frame it is an
/// error.
pub fn topic_pub(
    req: &PubRequest<'_>,
    schemas_dir: Option<&Path>,
    running: &AtomicBool,
) -> Result<PubReport, VerbError> {
    let period = publish_period(req.times, req.rate_hz)?;
    let name = qualified_type_name(req.type_name, InterfaceKind::Message)?;
    let walker = resolve_message_walker(&name, schemas_dir, req.scope)?;
    let mut frame = message_frame(&walker, &name, req.message, 0)?;
    let mut header = WireHeader::read_from_buf(&frame)
        .ok_or_else(|| VerbError::Encode("the encoded frame has no header".to_string()))?;
    let slot = u32::try_from(frame.len())
        .ok()
        .and_then(MaxSliceLen::try_new)
        .ok_or_else(|| {
            VerbError::Encode(format!(
                "a {}-byte frame passes the u32 size field",
                frame.len()
            ))
        })?;

    let transport = cerulion_core::TransportManager::get_or_init()?;
    let injector = transport.create_ingress_injector(req.topic, header.schema_hash, slot)?;
    let _egress = if req.scope.local_only() {
        None
    } else if req.topic.starts_with('/') {
        register_egress(req.topic, &name, header.schema_hash)
    } else {
        tracing::warn!(
            topic = %req.topic,
            "the topic name does not start with '/', so it is published locally only"
        );
        None
    };

    let deadline = Instant::now() + req.wait_subscribers;
    while transport.topic_subscriber_count(req.topic) == 0
        && Instant::now() < deadline
        && running.load(Ordering::Relaxed)
    {
        std::thread::sleep(SUBSCRIBER_POLL); // ALLOW: a one-shot CLI wait for a first subscriber, bounded by --wait-subscribers
    }

    let mut frames = 0u64;
    let mut subscribers = 0usize;
    let mut next = Instant::now();
    while frames < req.times && running.load(Ordering::Relaxed) {
        if frames > 0 {
            next += period;
            if !sleep_until(next, running) {
                break;
            }
        }
        header.sequence = header.sequence.wrapping_add(1);
        header.timestamp_ns = transport.clock().now_ns();
        header.write_to_buf(&mut frame[..WireHeader::SIZE]);
        match injector.reinject_raw(&frame) {
            ReinjectOutcome::Injected { recipients } => {
                frames += 1;
                subscribers = recipients;
            }
            other => {
                return Err(VerbError::Cli(crate::error::CliError::Validation(format!(
                    "publishing frame {} on '{}' failed: {other:?}",
                    frames + 1,
                    req.topic
                ))))
            }
        }
    }
    if frames == 0 {
        return Err(VerbError::Cli(crate::error::CliError::Validation(format!(
            "interrupted before any frame was published on '{}'",
            req.topic
        ))));
    }
    sleep_until(Instant::now() + LINGER, running);
    Ok(PubReport {
        topic: req.topic.to_string(),
        type_name: name,
        frames,
        subscribers,
    })
}

/// Sleep until `at`, in slices that re-check `running`; false if it cleared.
fn sleep_until(at: Instant, running: &AtomicBool) -> bool {
    loop {
        if !running.load(Ordering::Relaxed) {
            return false;
        }
        let now = Instant::now();
        if now >= at {
            return true;
        }
        std::thread::sleep((at - now).min(Duration::from_millis(50))); // ALLOW: paces --rate and the linger in a one-shot CLI verb
    }
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
    fn one_frame_has_no_period_and_repeats_default_to_one_hz() {
        assert_eq!(publish_period(1, None).unwrap(), Duration::ZERO);
        assert_eq!(publish_period(1, Some(50.0)).unwrap(), Duration::ZERO);
        assert_eq!(publish_period(3, None).unwrap(), Duration::from_secs(1));
        assert_eq!(
            publish_period(3, Some(4.0)).unwrap(),
            Duration::from_millis(250)
        );
    }

    #[test]
    fn a_rate_that_is_not_a_positive_number_and_zero_times_are_usage_errors() {
        for rate in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let err = publish_period(2, Some(rate)).unwrap_err();
            assert_eq!(err.exit_code(), crate::message_text::EXIT_USAGE, "{rate}");
        }
        assert_eq!(
            publish_period(0, None).unwrap_err().exit_code(),
            crate::message_text::EXIT_USAGE
        );
    }

    #[test]
    fn the_report_line_is_the_documented_shape() {
        let report = PubReport {
            topic: "/cmd_vel".to_string(),
            type_name: "geometry_msgs/Twist".to_string(),
            frames: 3,
            subscribers: 1,
        };
        assert_eq!(
            report.line(),
            "published: topic=/cmd_vel type=geometry_msgs/Twist frames=3 subscribers=1"
        );
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
