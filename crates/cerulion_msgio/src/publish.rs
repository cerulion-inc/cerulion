// SPDX-License-Identifier: AGPL-3.0-only
//! Publish frames of one message given as JSON or YAML text on a local
//! shared-memory topic: the engine of `cerulion topic pub` and the
//! robot-side `topic-publish` verb.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use cerulion_core::codegen::FrameWalker;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{FullQueueSend, ReinjectOutcome};

use crate::text::{message_frame, VerbError};

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

/// One publish: what to send, where, how often.
#[derive(Debug, Clone)]
pub struct PubRequest<'a> {
    /// The topic, as ROS spells it (`/cmd_vel`).
    pub topic: &'a str,
    /// The message as a JSON or YAML mapping.
    pub message: &'a str,
    /// How many frames to publish (at least 1).
    pub times: u64,
    /// The publish rate in Hz; `None` is [`DEFAULT_REPEAT_RATE_HZ`].
    pub rate_hz: Option<f64>,
    /// How long to wait for a first subscriber before publishing.
    pub wait_subscribers: Duration,
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
    /// Deliveries skipped: for each frame, the subscribers attached when it
    /// was sent less the ones it was delivered to. A subscriber is skipped
    /// when its queue is full on a topic created without safe overflow.
    pub skipped: u64,
}

impl PubReport {
    /// The one line `topic pub` prints, with what was skipped after it when
    /// a delivery was.
    pub fn line(&self) -> String {
        let line = format!(
            "published: topic={} type={} frames={} subscribers={}",
            self.topic, self.type_name, self.frames, self.subscribers
        );
        match self.skipped_note() {
            Some(note) => format!("{line}; {note}"),
            None => line,
        }
    }

    /// What was skipped, in words; `None` when every delivery was made.
    pub fn skipped_note(&self) -> Option<String> {
        match self.skipped {
            0 => None,
            1 => Some("1 delivery was skipped because a subscriber's queue was full".to_string()),
            n => Some(format!(
                "{n} deliveries were skipped because a subscriber's queue was full"
            )),
        }
    }
}

/// The interval between frames for `times` frames at `rate_hz`; zero for a
/// single frame. A rate that is not a finite positive number, or whose
/// interval rounds to zero nanoseconds, is a usage error.
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
        .ok()
        .filter(|period| !period.is_zero())
        .ok_or_else(|| VerbError::Usage(format!("--rate {rate} gives no usable interval")))
}

/// How long `times` frames take at `period`, counted as `times` whole
/// periods; [`Duration::MAX`] when that does not fit a `Duration`.
pub fn frames_span(period: Duration, times: u64) -> Duration {
    Duration::try_from_secs_f64(period.as_secs_f64() * times as f64).unwrap_or(Duration::MAX)
}

/// Publish `req.message`, encoded as the type `name` (`pkg/Type`, a name
/// `walker` knows), on `req.topic`, and report what was sent.
///
/// The message is encoded before anything is opened, so a bad value fails
/// with no side effect. `on_open` runs once the publisher is open, with the
/// type's schema hash, and what it returns is held until the publish ends
/// (the CLI registers the topic with `cerulion-netd` there). The call then
/// waits up to `req.wait_subscribers` for a subscriber, publishes
/// `req.times` frames at the requested rate (each with the next sequence
/// number and the transport clock's stamp), and keeps the publisher open
/// half a second longer. With no subscriber after the wait it publishes anyway
/// and reports `subscribers=0`. A cleared `running` stops between frames and
/// reports the frames sent; before the first frame it is an error.
///
/// The publisher is built with [`FullQueueSend::Discard`], so a frame's send
/// returns without waiting on a subscriber whose queue is full (a topic
/// created without safe overflow) and skips that subscriber. Each frame's
/// skipped subscribers, the topic's subscriber count read before and after
/// the send (the smaller of the two) less the subscribers the frame reached,
/// add up in [`PubReport::skipped`].
pub fn publish_message<G>(
    req: &PubRequest<'_>,
    walker: &FrameWalker,
    name: &str,
    running: &AtomicBool,
    on_open: impl FnOnce(u64) -> G,
) -> Result<PubReport, VerbError> {
    publish_message_gated(req, walker, name, running, on_open, |send| send())
}

/// [`publish_message`] with `send_gate` around each frame's send: the
/// publish passes every frame's send to `send_gate` as a closure, and the
/// frame goes out only if `send_gate` runs it. A gate that returns without
/// running it stops the publish there as a cleared `running` does, with no
/// linger: the frames sent before it are reported, and with none sent it is
/// an error.
pub fn publish_message_gated<G>(
    req: &PubRequest<'_>,
    walker: &FrameWalker,
    name: &str,
    running: &AtomicBool,
    on_open: impl FnOnce(u64) -> G,
    mut send_gate: impl FnMut(&mut dyn FnMut()),
) -> Result<PubReport, VerbError> {
    let period = publish_period(req.times, req.rate_hz)?;
    let mut frame = message_frame(walker, name, req.message, 0)?;
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
    let injector = transport.create_ingress_injector_with(
        req.topic,
        header.schema_hash,
        slot,
        FullQueueSend::Discard,
    )?;
    let _held = on_open(header.schema_hash);

    let deadline = Instant::now() + req.wait_subscribers;
    while transport.topic_subscriber_count(req.topic) == 0
        && Instant::now() < deadline
        && running.load(Ordering::Relaxed)
    {
        std::thread::sleep(SUBSCRIBER_POLL); // ALLOW: a one-shot wait for a first subscriber, bounded by --wait-subscribers
    }

    let mut frames = 0u64;
    let mut subscribers = 0usize;
    let mut skipped = 0u64;
    let mut gated = false;
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
        let mut sent = None;
        send_gate(&mut || sent = Some(reinject_counted(&injector, &frame)));
        match sent {
            None => {
                gated = true;
                break;
            }
            Some((ReinjectOutcome::Injected { recipients }, attached)) => {
                frames += 1;
                subscribers = recipients;
                skipped += attached.saturating_sub(recipients) as u64;
            }
            Some((other, _)) => {
                return Err(VerbError::Internal(format!(
                    "publishing frame {} on '{}' failed: {other:?}",
                    frames + 1,
                    req.topic
                )))
            }
        }
    }
    if frames == 0 {
        return Err(VerbError::Internal(format!(
            "interrupted before any frame was published on '{}'",
            req.topic
        )));
    }
    if !gated {
        sleep_until(Instant::now() + LINGER, running);
    }
    Ok(PubReport {
        topic: req.topic.to_string(),
        type_name: name.to_string(),
        frames,
        subscribers,
        skipped,
    })
}

/// Re-inject `frame` and return the outcome with the topic's subscriber
/// count, the smaller of its reads before and after the send, so a
/// subscriber that attaches or leaves during the send is not counted as
/// skipped.
fn reinject_counted(
    injector: &cerulion_core::IngressInjector,
    frame: &[u8],
) -> (ReinjectOutcome, usize) {
    let before = injector.subscriber_count().unwrap_or(0);
    let outcome = injector.reinject_raw(frame);
    let after = injector.subscriber_count().unwrap_or(0);
    (outcome, before.min(after))
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
            assert_eq!(err.exit_code(), crate::text::EXIT_USAGE, "{rate}");
        }
        assert_eq!(
            publish_period(0, None).unwrap_err().exit_code(),
            crate::text::EXIT_USAGE
        );
    }

    #[test]
    fn a_rate_whose_interval_rounds_to_zero_is_a_usage_error() {
        let err = publish_period(2, Some(1e10)).unwrap_err();
        assert_eq!(err.exit_code(), crate::text::EXIT_USAGE);
        assert_eq!(
            publish_period(2, Some(1e9)).unwrap(),
            Duration::from_nanos(1)
        );
    }

    #[test]
    fn the_frames_span_counts_every_frame_and_saturates() {
        assert_eq!(
            frames_span(Duration::from_millis(250), 3),
            Duration::from_millis(750)
        );
        assert_eq!(frames_span(Duration::ZERO, 1), Duration::ZERO);
        assert!(frames_span(Duration::from_nanos(1), u64::MAX) > Duration::from_secs(1 << 33));
        assert_eq!(frames_span(Duration::MAX, 2), Duration::MAX);
    }

    #[test]
    fn the_report_line_is_the_documented_shape() {
        let mut report = PubReport {
            topic: "/cmd_vel".to_string(),
            type_name: "geometry_msgs/Twist".to_string(),
            frames: 3,
            subscribers: 1,
            skipped: 0,
        };
        assert_eq!(
            report.line(),
            "published: topic=/cmd_vel type=geometry_msgs/Twist frames=3 subscribers=1"
        );
        report.skipped = 1;
        assert_eq!(
            report.line(),
            "published: topic=/cmd_vel type=geometry_msgs/Twist frames=3 subscribers=1; 1 \
             delivery was skipped because a subscriber's queue was full"
        );
        report.skipped = 2;
        assert_eq!(
            report.skipped_note().unwrap(),
            "2 deliveries were skipped because a subscriber's queue was full"
        );
    }
}
