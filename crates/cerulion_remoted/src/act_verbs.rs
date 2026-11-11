// SPDX-License-Identifier: AGPL-3.0-only
//! The act verbs: publish a message, call a service, snapshot a camera topic
//! and read one message, on this robot's local shared memory, for a paired
//! peer over the ops plane.
//!
//! | verb | caps | args | result |
//! |---|---|---|---|
//! | `topic-publish` | `CAP_TELEOP` | `topic`, `type`, `message`, `times`?, `rate_hz`?, `wait_subscribers_s`? | `topic`, `type`, `frames`, `subscribers`, `skipped`, `line` |
//! | `service-call` | `CAP_TELEOP` | `service`, `type`, `request`?, `timeout_s`? | `response`, `response_text` (the same response as one JSON line, fields in declaration order) |
//! | `topic-snapshot` | `CAP_OBSERVE` | `topic`, `timeout_s`?, `quality`? | `jpeg_base64`, `width`, `height`, `encoding`, `stamp_ns`, `bytes` |
//! | `topic-echo-once` | `CAP_OBSERVE` | `topic`, `timeout_s`? | `type`, `stamp_ns`, `sequence`, `message` |
//!
//! `message` and `request` are a JSON object, or a string holding JSON or
//! YAML. Types resolve from the built-in ROS 2 corpus and the `.msg`/`.srv`
//! store under the directory [`SCHEMAS_DIR_ENV`] names (the workspace
//! `schemas/` layout: `<pkg>/msg/*.msg`, `<pkg>/srv/*.srv`). The operations
//! are [`cerulion_msgio`]'s, the same ones `cerulion topic pub`,
//! `service call` and `topic snapshot` run on a desk.
//!
//! `topic-publish` and `service-call` check only the e-stop, not the control
//! lease: each is refused while the e-stop latch (`cerud::lease::ControlLease`)
//! is engaged or an engage is counted on the [`EstopLease`], a running
//! publish or call stops at the next check (every 10 ms) after either, and
//! each `topic-publish` frame and the `service-call` request are sent only
//! while the verb holds the lease's lock and reads, under it, the latch clear
//! and no engage counted. The publishers those sends go through are built
//! with [`cerulion_core::FullQueueSend::Discard`], so a send skips a
//! subscriber whose queue is full instead of waiting on it. An engage counts
//! itself in before it waits for the lease's lock, so it waits for at most
//! the one send that holds the lock: after the count, each running act verb
//! takes the lock at most once more, to read the count, and sends nothing. A
//! skipped delivery is reported: `skipped` counts them in a `topic-publish`
//! result and its `line` says so, and a `service-call` request that no
//! server's queue took fails as unavailable naming the full request queue. A
//! publish still running
//! [`PUBLISH_DEADLINE`] after it started is stopped too. A `message` or
//! `request` longer than [`MAX_MESSAGE_TEXT_BYTES`] is refused. A topic under
//! the reserved `/__cerulion` namespace is refused.
//!
//! A failure is `CerudError::Verb` with a message that starts with the
//! error's class (`usage: `, `encode: `, `unavailable: `, `internal: `,
//! [`cerulion_msgio::text::VerbError::to_classed`]), from which a desk
//! recovers the CLI exit code.

use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerud::lease::EstopState;
use cerud::verbs::VerbHandler;
use cerud::{CerudError, CerudResult};
use cerulion_msgio::text::{frame_to_json, qualified_type_name, InterfaceKind, VerbError};
use cerulion_msgio::{publish, service, snapshot};
use serde_json::{json, Value};

use crate::estop_lease::EstopLease;

/// Register the four act verbs in `verbs`; `topic-publish` and
/// `service-call` check the e-stop on `lease`, the robot's one shared control
/// lease and the count of the engages that wait for its lock. Every daemon ops
/// constructor registers them through this function, with a clone of the
/// `EstopLease` its `engage-estop` verb engages through; a library host
/// registers refusals under the same names instead.
pub fn register(verbs: &mut cerud::verbs::VerbRegistry, lease: EstopLease) {
    verbs
        .register(Box::new(TopicPublishVerb::new(lease.clone())))
        .register(Box::new(ServiceCallVerb::new(lease)))
        .register(Box::new(TopicSnapshotVerb))
        .register(Box::new(TopicEchoOnceVerb));
}

/// Publish a message on a topic.
pub const TOPIC_PUBLISH: &str = "topic-publish";
/// Call a service once.
pub const SERVICE_CALL: &str = "service-call";
/// The newest frame of a camera topic as a JPEG.
pub const TOPIC_SNAPSHOT: &str = "topic-snapshot";
/// One message of a topic, decoded.
pub const TOPIC_ECHO_ONCE: &str = "topic-echo-once";

/// The environment variable naming the schema directory the act verbs
/// resolve message and service types from.
pub const SCHEMAS_DIR_ENV: &str = "CERULION_REMOTED_SCHEMAS_DIR";

/// How often a running publish checks the e-stop.
const ESTOP_POLL: Duration = Duration::from_millis(10);

/// The default `timeout_s` of `topic-snapshot` and `topic-echo-once`.
const DEFAULT_READ_TIMEOUT: Duration = Duration::from_secs(5);

/// The largest result the verbs return, as serialized JSON: the 16 MiB ops
/// frame cap less room for the response envelope.
const MAX_RESULT_BYTES: usize = 12 * 1024 * 1024;

/// The largest frame `topic-echo-once` decodes: its JSON form can be several
/// times the frame's size, and the result is refused past
/// [`MAX_RESULT_BYTES`].
const MAX_ECHO_FRAME_BYTES: usize = MAX_RESULT_BYTES / 8;

/// The longest single timeout a caller may ask for.
const MAX_TIMEOUT: Duration = Duration::from_secs(60);

/// The longest a `topic-publish` may run in all (subscriber wait plus
/// frames), inside the ops session's 120 s deadline.
const MAX_PUBLISH_SPAN: Duration = Duration::from_secs(90);

/// When the watcher stops a `topic-publish` that is still running: the 90 s
/// span the request was checked against plus room for the encode and the
/// linger, inside the ops session's 120 s deadline. A frame
/// that takes longer to send than its period delays every later one, so the
/// span check alone does not bound the run.
pub const PUBLISH_DEADLINE: Duration = Duration::from_secs(100);

/// The longest `message` or `request` text the verbs read (1 MiB). The
/// encode holds the parsed text, the tree with omitted fields filled in and
/// that tree's re-serialized text at once, so the cap bounds that memory
/// below what the 16 MiB ops frame would allow.
pub const MAX_MESSAGE_TEXT_BYTES: usize = 1 << 20;

fn remote_lease(
    topic: &str,
) -> CerudResult<cerulion_core::transport::egress_scope::RemoteTopicLease> {
    cerulion_core::TransportManager::get_or_init()
        .and_then(|manager| manager.acquire_remote_topic_lease(topic))
        .map_err(|error| verb_error(VerbError::Unavailable(error.to_string())))
}

fn schemas_dir() -> Option<PathBuf> {
    std::env::var_os(SCHEMAS_DIR_ENV)
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
}

/// A verb failure as the ops error carrying its class.
fn verb_error(e: VerbError) -> CerudError {
    CerudError::Verb(e.to_classed())
}

fn usage(message: String) -> CerudError {
    verb_error(VerbError::Usage(message))
}

fn str_arg<'a>(args: &'a Value, key: &str) -> CerudResult<&'a str> {
    args.get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| usage(format!("missing string argument `{key}`")))
}

/// A message argument: a JSON object (used as JSON text) or a string of
/// JSON or YAML, at most [`MAX_MESSAGE_TEXT_BYTES`] long; absent is `{}`.
fn text_arg(args: &Value, key: &str) -> CerudResult<String> {
    let text = match args.get(key) {
        None | Some(Value::Null) => "{}".to_string(),
        Some(Value::String(s)) => s.clone(),
        Some(v @ Value::Object(_)) => v.to_string(),
        Some(_) => {
            return Err(usage(format!(
                "`{key}` must be a JSON object or a JSON/YAML string"
            )))
        }
    };
    if text.len() > MAX_MESSAGE_TEXT_BYTES {
        return Err(usage(format!(
            "`{key}` is {} bytes of text; the act verbs read at most {MAX_MESSAGE_TEXT_BYTES}",
            text.len()
        )));
    }
    Ok(text)
}

fn seconds_arg(args: &Value, key: &str, default: Duration) -> CerudResult<Duration> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v
            .as_f64()
            .and_then(|s| Duration::try_from_secs_f64(s).ok())
            .filter(|d| *d <= MAX_TIMEOUT)
            .ok_or_else(|| {
                usage(format!(
                    "`{key}` must be a number of seconds from 0 to {}",
                    MAX_TIMEOUT.as_secs()
                ))
            }),
    }
}

/// The result checked against [`MAX_RESULT_BYTES`].
fn bounded(result: Value) -> CerudResult<Value> {
    let len = result.to_string().len();
    if len > MAX_RESULT_BYTES {
        return Err(verb_error(VerbError::Encode(format!(
            "the result is {len} bytes, over the {MAX_RESULT_BYTES}-byte limit of one ops reply"
        ))));
    }
    Ok(result)
}

/// The flag a read verb's waits poll; it stays set, so a wait ends at its
/// timeout.
fn running() -> AtomicBool {
    AtomicBool::new(true)
}

/// Sets its flag when dropped, panics included.
struct SetOnDrop<'a>(&'a AtomicBool);

impl Drop for SetOnDrop<'_> {
    fn drop(&mut self) {
        self.0.store(true, std::sync::atomic::Ordering::Relaxed);
    }
}

/// Why [`watched`] cleared the running flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stop {
    /// The e-stop latch engaged.
    Estop,
    /// The deadline passed.
    Deadline,
}

/// Run `op` with a running flag that a watcher thread clears when, given
/// one, `deadline` passes or the e-stop on `lease` engages, checked every
/// [`ESTOP_POLL`] in that order. The watcher reads the clock first and the
/// e-stop through [`estop_reads_engaged`], which does not wait for the
/// lease's lock, so a deadline stop never waits on the lease. The watcher
/// ends with `op`. Returns what `op` returned and why the flag was cleared,
/// if it was. With the e-stop engaged or an engage counted when this is
/// called, `op` runs with the flag already cleared and no watcher.
fn watched<R>(
    lease: &EstopLease,
    verb: &'static str,
    deadline: Option<Instant>,
    op: impl FnOnce(&AtomicBool) -> R,
) -> (R, Option<Stop>) {
    if refuse_under_estop(lease, verb).is_err() {
        return (op(&AtomicBool::new(false)), Some(Stop::Estop));
    }
    let running = Arc::new(AtomicBool::new(true));
    let done = Arc::new(AtomicBool::new(false));
    let watcher = {
        let (running, done, lease) = (running.clone(), done.clone(), lease.clone());
        std::thread::spawn(move || {
            while !done.load(std::sync::atomic::Ordering::Relaxed) {
                let stop = if deadline.is_some_and(|at| Instant::now() >= at) {
                    Some(Stop::Deadline)
                } else if estop_reads_engaged(&lease) {
                    Some(Stop::Estop)
                } else {
                    None
                };
                if stop.is_some() {
                    running.store(false, std::sync::atomic::Ordering::Relaxed);
                    return stop;
                }
                std::thread::sleep(ESTOP_POLL); // ALLOW: e-stop watch for one act verb, ends with it
            }
            None
        })
    };
    let out = {
        let _stop = SetOnDrop(&done);
        op(&running)
    };
    let stop = watcher.join().unwrap_or(Some(Stop::Estop));
    (out, stop)
}

/// The e-stop on `lease` read without waiting for its lock: true when an
/// engage is counted, or when the latch reads engaged under `try_lock` or the
/// lock is poisoned; false when the latch reads clear or another thread
/// holds the lock. A send holding the lock reads the latch and the count
/// itself under it, and the next [`ESTOP_POLL`] reads them again.
fn estop_reads_engaged(lease: &EstopLease) -> bool {
    if lease.engage_pending() {
        return true;
    }
    match lease.try_lock() {
        Ok(lease) => !matches!(lease.estop(), EstopState::Clear),
        Err(std::sync::TryLockError::Poisoned(_)) => true,
        Err(std::sync::TryLockError::WouldBlock) => false,
    }
}

/// Refuse `verb` while the robot's e-stop holds ([`EstopLease::holds`]): the
/// latch is engaged or an engage is counted. A counted engage refuses without
/// taking the lease's lock.
fn refuse_under_estop(lease: &EstopLease, verb: &str) -> CerudResult<()> {
    if lease.holds() {
        return Err(estop_refusal(verb));
    }
    Ok(())
}

/// The error [`refuse_under_estop`] answers for `verb`:
/// `unavailable: the robot's e-stop is engaged, so <verb> is refused`.
pub(crate) fn estop_refusal(verb: &str) -> CerudError {
    verb_error(VerbError::Unavailable(format!(
        "the robot's e-stop is engaged, so {verb} is refused"
    )))
}

/// Run `send` while holding `lease`'s lock, if under that lock its e-stop
/// latch reads clear and no engage is counted; true when `send` ran. The
/// latch read and `send` share the one lock, so an e-stop engaged before the
/// lock is taken keeps `send` from running. An engage counts itself in
/// before it waits for the lock ([`EstopLease::engage`]), so once it waits,
/// every later call here sends nothing and the engage waits for at most the
/// `send` that holds the lock. Each `send` is one frame or request publish
/// through a publisher built with [`cerulion_core::FullQueueSend::Discard`],
/// which skips a subscriber whose queue is full instead of waiting on it.
fn send_unless_estop(lease: &EstopLease, send: &mut dyn FnMut()) -> bool {
    match lease.lock() {
        Ok(latch) if matches!(latch.estop(), EstopState::Clear) && !lease.engage_pending() => {
            send();
            true
        }
        _ => false,
    }
}

/// Refuse a topic in the reserved control-plane namespace.
fn refuse_reserved(topic: &str) -> CerudResult<()> {
    let canonical = if topic.starts_with('/') {
        topic.to_string()
    } else {
        format!("/{topic}")
    };
    if canonical == cerulion_core::transport::gateway::RESERVED_TOPIC_PREFIX
        || canonical.starts_with(&format!(
            "{}/",
            cerulion_core::transport::gateway::RESERVED_TOPIC_PREFIX
        ))
    {
        return Err(usage(format!(
            "'{}' is in Cerulion's reserved control-plane namespace",
            topic.escape_debug()
        )));
    }
    Ok(())
}

/// `topic-publish`: publish frames of one message on a local topic. Refused
/// while the e-stop is engaged; each frame goes out only with the latch read
/// clear and no engage counted under the lease's lock at its send, so an
/// e-stop engaged mid-publish stops it before the next frame. The frame's
/// send holds that lock and cannot wait on a full queue: its publisher is
/// built with [`cerulion_core::FullQueueSend::Discard`] and skips a
/// subscriber whose queue is full. An engage counted before it waits for
/// the lock stops the publish at its next frame, so the engage waits for at
/// most the one send in flight. The result's `skipped` counts the skipped
/// deliveries and its `line` says so, as does the error of a publish the
/// e-stop stopped.
pub struct TopicPublishVerb {
    lease: EstopLease,
}

impl TopicPublishVerb {
    /// The verb, checking the e-stop on `lease`. An `Arc<Mutex<ControlLease>>`
    /// becomes an [`EstopLease`] with a count of its own.
    pub fn new(lease: impl Into<EstopLease>) -> Self {
        Self {
            lease: lease.into(),
        }
    }
}

impl VerbHandler for TopicPublishVerb {
    fn name(&self) -> &'static str {
        TOPIC_PUBLISH
    }

    fn is_mutating(&self) -> bool {
        true
    }

    fn execute(&self, args: &Value) -> CerudResult<Value> {
        let topic = str_arg(args, "topic")?;
        let _remote_lease = remote_lease(topic)?;
        refuse_reserved(topic)?;
        let type_arg = str_arg(args, "type")?;
        let message = text_arg(args, "message")?;
        let times = match args.get("times") {
            None | Some(Value::Null) => 1,
            Some(v) => v
                .as_u64()
                .filter(|n| *n >= 1)
                .ok_or_else(|| usage("`times` must be an integer of at least 1".to_string()))?,
        };
        let rate_hz = match args.get("rate_hz") {
            None | Some(Value::Null) => None,
            Some(v) => Some(
                v.as_f64()
                    .ok_or_else(|| usage("`rate_hz` must be a number".to_string()))?,
            ),
        };
        let wait = seconds_arg(
            args,
            "wait_subscribers_s",
            publish::DEFAULT_WAIT_SUBSCRIBERS,
        )?;
        let period = publish::publish_period(times, rate_hz).map_err(verb_error)?;
        let frames_span = publish::frames_span(period, times);
        if wait.saturating_add(frames_span) > MAX_PUBLISH_SPAN {
            return Err(usage(format!(
                "the subscriber wait plus {times} frames at that rate take longer than {} s",
                MAX_PUBLISH_SPAN.as_secs()
            )));
        }
        refuse_under_estop(&self.lease, TOPIC_PUBLISH)?;
        let name = qualified_type_name(type_arg, InterfaceKind::Message).map_err(verb_error)?;
        let walker =
            cerulion_msgio::checked_walker(cerulion_msgio::store_schemas(schemas_dir().as_deref()))
                .map_err(verb_error)?;
        if !walker.knows(&name) {
            return Err(usage(format!(
                "unknown message type '{name}' on this robot: not a built-in ROS 2 type and not \
                 in the {SCHEMAS_DIR_ENV} store"
            )));
        }
        let req = publish::PubRequest {
            topic,
            message: &message,
            times,
            rate_hz,
            wait_subscribers: wait,
        };
        // The publish loop polls the running flag in the subscriber wait,
        // between frames and in the linger, and sends each frame through
        // send_unless_estop.
        let deadline = Instant::now() + PUBLISH_DEADLINE;
        let mut estop_at_send = false;
        let (report, stop) = watched(&self.lease, TOPIC_PUBLISH, Some(deadline), |running| {
            publish::publish_message_gated(
                &req,
                &walker,
                &name,
                running,
                |_| (),
                |send| estop_at_send = !send_unless_estop(&self.lease, send),
            )
        });
        let stop = if estop_at_send {
            Some(Stop::Estop)
        } else {
            stop
        };
        let sent = report.as_ref().map_or(0, |r| r.frames);
        let skipped = report
            .as_ref()
            .ok()
            .and_then(publish::PubReport::skipped_note)
            .map(|note| format!("; {note}"))
            .unwrap_or_default();
        match stop {
            Some(Stop::Estop) => {
                return Err(verb_error(VerbError::Unavailable(format!(
                    "the robot's e-stop engaged after {sent} of {times} frames{skipped}"
                ))))
            }
            Some(Stop::Deadline) => {
                return Err(verb_error(VerbError::Internal(format!(
                    "the publish was still running {} s after it started and was stopped after \
                     {sent} of {times} frames{skipped}",
                    PUBLISH_DEADLINE.as_secs()
                ))))
            }
            None => {}
        }
        let report = report.map_err(verb_error)?;
        Ok(json!({
            "topic": report.topic,
            "type": report.type_name,
            "frames": report.frames,
            "subscribers": report.subscribers,
            "skipped": report.skipped,
            "line": report.line(),
        }))
    }
}

/// `service-call`: call a service on this robot once. Refused while the
/// e-stop is engaged; the request goes out only with the latch read clear
/// and no engage counted under the lease's lock at the send, and an e-stop
/// engaged during the server or response wait ends the call as unavailable.
/// The send holds that lock and cannot wait on a full queue: the client's
/// request publisher is built with [`cerulion_core::FullQueueSend::Discard`]
/// and skips a server whose request queue is full, so engaging the e-stop
/// waits for at most the one send. A request that no server's queue took
/// fails as unavailable naming the full request queue.
pub struct ServiceCallVerb {
    lease: EstopLease,
}

impl ServiceCallVerb {
    /// The verb, checking the e-stop on `lease`. An `Arc<Mutex<ControlLease>>`
    /// becomes an [`EstopLease`] with a count of its own.
    pub fn new(lease: impl Into<EstopLease>) -> Self {
        Self {
            lease: lease.into(),
        }
    }
}

impl VerbHandler for ServiceCallVerb {
    fn name(&self) -> &'static str {
        SERVICE_CALL
    }

    fn is_mutating(&self) -> bool {
        true
    }

    fn execute(&self, args: &Value) -> CerudResult<Value> {
        let service_name = str_arg(args, "service")?;
        let type_arg = str_arg(args, "type")?;
        let request = text_arg(args, "request")?;
        let timeout = seconds_arg(args, "timeout_s", service::DEFAULT_CALL_TIMEOUT)?;
        service::check_service_name(service_name).map_err(verb_error)?;
        let _request_lease = remote_lease(&cerulion_core::transport::service::request_topic(
            service_name,
        ))?;
        refuse_under_estop(&self.lease, SERVICE_CALL)?;
        let types = service::resolve_store_service_types(type_arg, schemas_dir().as_deref())
            .map_err(verb_error)?;
        let req = service::CallRequest {
            service: service_name,
            type_name: type_arg,
            request: &request,
            timeout,
        };
        let mut estop_at_send = false;
        let (call, stop) = watched(&self.lease, SERVICE_CALL, None, |running| {
            service::service_call_gated(&req, &types, running, |send| {
                estop_at_send = !send_unless_estop(&self.lease, send);
            })
        });
        let stop = if estop_at_send {
            Some(Stop::Estop)
        } else {
            stop
        };
        let text = match (call, stop) {
            (Err(e), Some(Stop::Estop)) => {
                return Err(verb_error(VerbError::Unavailable(format!(
                    "the robot's e-stop engaged, so {SERVICE_CALL} stopped without a response: {e}"
                ))))
            }
            (call, _) => call.map_err(verb_error)?,
        };
        let response: Value = serde_json::from_str(&text)
            .map_err(|e| verb_error(VerbError::Encode(format!("the response is not JSON: {e}"))))?;
        bounded(json!({ "response": response, "response_text": text }))
    }
}

/// `topic-snapshot`: the newest frame of a local camera topic as a JPEG.
pub struct TopicSnapshotVerb;

impl VerbHandler for TopicSnapshotVerb {
    fn name(&self) -> &'static str {
        TOPIC_SNAPSHOT
    }

    fn is_mutating(&self) -> bool {
        false
    }

    fn execute(&self, args: &Value) -> CerudResult<Value> {
        let topic = str_arg(args, "topic")?;
        let _remote_lease = remote_lease(topic)?;
        let timeout = seconds_arg(args, "timeout_s", DEFAULT_READ_TIMEOUT)?;
        let quality = match args.get("quality") {
            None | Some(Value::Null) => snapshot::DEFAULT_JPEG_QUALITY,
            Some(v) => v
                .as_u64()
                .filter(|q| (1..=100).contains(q))
                .map(|q| q as u8)
                .ok_or_else(|| usage("`quality` must be an integer from 1 to 100".to_string()))?,
        };
        let walker =
            cerulion_msgio::checked_walker(cerulion_msgio::store_schemas(schemas_dir().as_deref()))
                .map_err(verb_error)?;
        let frame = newest_frame(topic, timeout)?;
        let (jpeg, width, height, encoding, stamp_ns) =
            snapshot::frame_to_jpeg(&walker, &frame, quality).map_err(verb_error)?;
        bounded(json!({
            "jpeg_base64": snapshot::base64_encode(&jpeg),
            "width": width,
            "height": height,
            "encoding": encoding,
            "stamp_ns": stamp_ns,
            "bytes": jpeg.len(),
        }))
    }
}

/// `topic-echo-once`: one message of a local topic, decoded to JSON.
pub struct TopicEchoOnceVerb;

impl VerbHandler for TopicEchoOnceVerb {
    fn name(&self) -> &'static str {
        TOPIC_ECHO_ONCE
    }

    fn is_mutating(&self) -> bool {
        false
    }

    fn execute(&self, args: &Value) -> CerudResult<Value> {
        let topic = str_arg(args, "topic")?;
        let _remote_lease = remote_lease(topic)?;
        let timeout = seconds_arg(args, "timeout_s", DEFAULT_READ_TIMEOUT)?;
        let walker =
            cerulion_msgio::checked_walker(cerulion_msgio::store_schemas(schemas_dir().as_deref()))
                .map_err(verb_error)?;
        let frame = newest_frame(topic, timeout)?;
        if frame.len() > MAX_ECHO_FRAME_BYTES {
            return Err(verb_error(VerbError::Encode(format!(
                "the frame is {} bytes; topic-echo-once decodes frames up to {MAX_ECHO_FRAME_BYTES} \
                 bytes",
                frame.len()
            ))));
        }
        echo_result(&walker, &frame)
    }
}

/// The `topic-echo-once` result for one whole frame.
fn echo_result(walker: &cerulion_core::codegen::FrameWalker, frame: &[u8]) -> CerudResult<Value> {
    let header = cerulion_core::wire::WireHeader::read_from_buf(frame)
        .ok_or_else(|| verb_error(VerbError::Encode("the frame has no header".to_string())))?;
    let type_name = walker
        .schema_name_for_hash(header.schema_hash)
        .ok_or_else(|| {
            verb_error(VerbError::Encode(format!(
                "no type on this robot has the frame's schema hash 0x{:016x}",
                header.schema_hash
            )))
        })?
        .to_string();
    let message = frame_to_json(walker, frame).map_err(verb_error)?;
    let message: Value = serde_json::from_str(&message)
        .map_err(|e| verb_error(VerbError::Encode(format!("the message is not JSON: {e}"))))?;
    bounded(json!({
        "type": type_name,
        "stamp_ns": header.timestamp_ns,
        "sequence": header.sequence,
        "message": message,
    }))
}

/// The newest frame of the local topic within `timeout`.
fn newest_frame(topic: &str, timeout: Duration) -> CerudResult<Vec<u8>> {
    let running = running();
    let deadline = Instant::now() + timeout;
    let (_transport, subscriber) =
        snapshot::open_local_topic(topic, deadline, &running).map_err(verb_error)?;
    snapshot::take_newest_frame(&subscriber, deadline, &running)
        .map_err(verb_error)?
        .ok_or_else(|| {
            verb_error(VerbError::Unavailable(format!(
                "no frame on '{}' within {} s",
                topic.escape_debug(),
                timeout.as_secs_f64()
            )))
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerud::lease::ControlLease;
    use std::sync::atomic::Ordering;
    use std::sync::{mpsc, Mutex};

    fn lease() -> EstopLease {
        EstopLease::new(Arc::new(Mutex::new(ControlLease::new(1_000_000_000))))
    }

    #[test]
    fn publish_and_call_are_refused_under_an_engaged_estop_and_reserved_topics_always() {
        let l = lease();
        l.lock().unwrap().engage_estop("test");
        let err = TopicPublishVerb::new(l.clone())
            .execute(&json!({"topic": "/x", "type": "std_msgs/String"}))
            .unwrap_err();
        assert_eq!(class_of(err), "unavailable");
        let err = ServiceCallVerb::new(l)
            .execute(&json!({"service": "/s", "type": "std_srvs/srv/Trigger"}))
            .unwrap_err();
        assert_eq!(class_of(err), "unavailable");
        for topic in ["/__cerulion/gateway_topics", "__cerulion/x", "/__cerulion"] {
            let err = TopicPublishVerb::new(lease())
                .execute(&json!({"topic": topic, "type": "std_msgs/String"}))
                .unwrap_err();
            assert_eq!(class_of(err), "usage", "{topic}");
        }
    }

    fn class_of(e: CerudError) -> String {
        match e {
            CerudError::Verb(m) => m.split(':').next().unwrap_or("").to_string(),
            other => panic!("not a verb error: {other:?}"),
        }
    }

    #[test]
    fn the_mutating_verbs_are_publish_and_call() {
        assert!(TopicPublishVerb::new(lease()).is_mutating());
        assert!(ServiceCallVerb::new(lease()).is_mutating());
        assert!(!TopicSnapshotVerb.is_mutating());
        assert!(!TopicEchoOnceVerb.is_mutating());
        let names = [
            TopicPublishVerb::new(lease()).name(),
            ServiceCallVerb::new(lease()).name(),
            TopicSnapshotVerb.name(),
            TopicEchoOnceVerb.name(),
        ];
        assert_eq!(
            names,
            [
                "topic-publish",
                "service-call",
                "topic-snapshot",
                "topic-echo-once"
            ]
        );
    }

    #[test]
    fn missing_and_malformed_arguments_are_usage_errors() {
        let publish = TopicPublishVerb::new(lease());
        let call = ServiceCallVerb::new(lease());
        for (verb, args) in [
            (&publish as &dyn VerbHandler, json!({})),
            (
                &publish,
                json!({"topic": "/x", "type": "std_msgs/String", "times": 0}),
            ),
            (
                &publish,
                json!({"topic": "/x", "type": "std_msgs/String", "times": 100, "rate_hz": 1}),
            ),
            (
                &publish,
                json!({"topic": "/x", "type": "std_msgs/String", "message": 5}),
            ),
            (
                &publish,
                json!({"topic": "/x", "type": "std_msgs/String", "times": 2, "rate_hz": 1e10}),
            ),
            (
                &publish,
                json!({"topic": "/x", "type": "std_msgs/String", "times": u64::MAX,
                       "rate_hz": 1e9}),
            ),
            (
                &publish,
                json!({"topic": "/x", "type": "std_msgs/String",
                       "message": "x".repeat(MAX_MESSAGE_TEXT_BYTES + 1)}),
            ),
            (
                &call,
                json!({"service": "/s", "type": "std_srvs/srv/Trigger",
                       "request": "x".repeat(MAX_MESSAGE_TEXT_BYTES + 1)}),
            ),
            (&publish, json!({"topic": "/x", "type": "nope_msgs/Nope"})),
            (&call, json!({"service": "/s"})),
            (
                &call,
                json!({"service": "/", "type": "std_srvs/srv/Trigger"}),
            ),
            (
                &call,
                json!({"service": "/s", "type": "a/srv/B", "timeout_s": -1}),
            ),
            (&TopicSnapshotVerb, json!({"topic": "/c", "quality": 0})),
            (&TopicEchoOnceVerb, json!({"timeout_s": 1})),
        ] {
            let err = verb.execute(&args).unwrap_err();
            assert_eq!(class_of(err), "usage", "{} {args}", verb.name());
        }
    }

    #[test]
    fn the_watcher_clears_the_flag_at_the_deadline_and_at_an_estop() {
        let hold = |running: &AtomicBool| {
            let start = Instant::now();
            while running.load(std::sync::atomic::Ordering::Relaxed)
                && start.elapsed() < Duration::from_secs(5)
            {
                std::thread::sleep(Duration::from_millis(5));
            }
            start.elapsed()
        };
        let deadline = Instant::now() + Duration::from_millis(50);
        let (held, stop) = watched(&lease(), TOPIC_PUBLISH, Some(deadline), hold);
        assert_eq!(stop, Some(Stop::Deadline));
        assert!(held < Duration::from_secs(4), "held {held:?}");
        let engaged = lease();
        let (held, stop) = watched(&engaged, TOPIC_PUBLISH, None, |running| {
            engaged.lock().unwrap().engage_estop("test");
            hold(running)
        });
        assert_eq!(stop, Some(Stop::Estop));
        assert!(held < Duration::from_secs(4), "held {held:?}");
        // Engaged before the call: the operation starts with the flag
        // already cleared.
        let (set_at_start, stop) = watched(&engaged, TOPIC_PUBLISH, None, |running| {
            running.load(std::sync::atomic::Ordering::Relaxed)
        });
        assert!(!set_at_start);
        assert_eq!(stop, Some(Stop::Estop));
        let (_, stop) = watched(&lease(), TOPIC_PUBLISH, None, |_| ());
        assert_eq!(stop, None);
    }

    #[test]
    fn a_deadline_stop_does_not_wait_for_the_lease_lock() {
        // The operation holds the lease's lock until the flag clears or 5 s
        // pass, as a send that does not return would.
        let l = lease();
        let deadline = Instant::now() + Duration::from_millis(50);
        let (held, stop) = watched(&l, TOPIC_PUBLISH, Some(deadline), |running| {
            let _lock = l.lock().unwrap();
            let start = Instant::now();
            while running.load(std::sync::atomic::Ordering::Relaxed)
                && start.elapsed() < Duration::from_secs(5)
            {
                std::thread::sleep(Duration::from_millis(5));
            }
            start.elapsed()
        });
        assert_eq!(stop, Some(Stop::Deadline));
        assert!(
            held < Duration::from_secs(4),
            "the watcher waited for the lease's lock before reading the deadline: held {held:?}"
        );
    }

    #[test]
    fn the_send_runs_under_the_lease_lock_and_not_under_an_engaged_latch() {
        let l = lease();
        let mut lock_held_during_send = None;
        let ran = send_unless_estop(&l, &mut || {
            lock_held_during_send = Some(matches!(
                l.try_lock(),
                Err(std::sync::TryLockError::WouldBlock)
            ));
        });
        assert!(ran);
        assert_eq!(
            lock_held_during_send,
            Some(true),
            "the send ran with the lease's lock released"
        );
        l.lock().unwrap().engage_estop("test");
        let mut sent = false;
        assert!(!send_unless_estop(&l, &mut || sent = true));
        assert!(!sent);
    }

    /// Sends back to back through [`send_unless_estop`] on another thread,
    /// each send busy for 20 us under the lease's lock, runs `engage` once
    /// the sends have started, and returns how many sends started while an
    /// engage was counted. The sender stops when the gate refuses, at the
    /// second such send, or after 2 s.
    fn sends_while_an_engage_waits(l: &EstopLease, engage: impl FnOnce()) -> u32 {
        let sending = Arc::new(AtomicBool::new(false));
        let sender = {
            let (l, sending) = (l.clone(), sending.clone());
            std::thread::spawn(move || {
                let start = Instant::now();
                let mut during = 0u32;
                while during < 2 && start.elapsed() < Duration::from_secs(2) {
                    let ran = send_unless_estop(&l, &mut || {
                        if l.engage_pending() {
                            during += 1;
                        }
                        sending.store(true, Ordering::SeqCst);
                        let until = Instant::now() + Duration::from_micros(20);
                        while Instant::now() < until {
                            std::hint::spin_loop();
                        }
                    });
                    if !ran {
                        break;
                    }
                }
                during
            })
        };
        while !sending.load(Ordering::SeqCst) {
            std::thread::yield_now();
        }
        engage();
        sender.join().unwrap()
    }

    /// A capture ask that asks for nothing.
    struct NoCapture;

    impl crate::flashback::EstopCaptureAsk for NoCapture {
        fn ask(&self, _by: &str) {}
    }

    #[test]
    fn an_engage_waits_for_at_most_the_send_holding_the_lease_lock() {
        // The lease is a std Mutex, which lets a sender that unlocks and
        // locks again at once take it ahead of the waiting engage; a count
        // the gate did not read would let sends run while the engage waits.
        for trial in 0..20 {
            let l = lease();
            let during = sends_while_an_engage_waits(&l, || {
                l.engage("test");
            });
            assert!(
                during <= 1,
                "trial {trial}: {during} sends started while the engage waited for the lock"
            );
            assert!(!matches!(l.lock().unwrap().estop(), EstopState::Clear));
        }
        for trial in 0..20 {
            let l = lease();
            let verb = crate::pairing_verbs::EngageEstopVerb::with_capture_ask(
                l.clone(),
                Arc::new(NoCapture),
            );
            let during = sends_while_an_engage_waits(&l, || {
                verb.execute_with_caller(
                    &cerud::transport::CallerIdentity::verified("test"),
                    &json!({}),
                )
                .unwrap();
            });
            assert!(
                during <= 1,
                "trial {trial}: {during} sends started while engage-estop waited for the lock"
            );
            assert!(!matches!(l.lock().unwrap().estop(), EstopState::Clear));
        }
    }

    /// Holds `l`'s lock, starts `engage` on another thread, and returns
    /// whether the engage read as counted within 2 s, with `check` run while
    /// it is counted and the lock is held. The lock is then released and the
    /// engage joined.
    fn while_an_engage_waits(
        l: &EstopLease,
        engage: impl FnOnce() + Send + 'static,
        check: impl FnOnce(),
    ) -> bool {
        let held = l.lock().unwrap();
        let engager = std::thread::spawn(engage);
        let until = Instant::now() + Duration::from_secs(2);
        while !l.engage_pending() && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(1));
        }
        let counted = l.engage_pending();
        if counted {
            check();
        }
        drop(held);
        engager.join().unwrap();
        counted
    }

    #[test]
    fn an_engage_is_counted_while_it_waits_for_the_lease_lock_and_after_it_is_not() {
        let l = lease();
        let engaging = l.clone();
        assert!(
            while_an_engage_waits(
                &l,
                move || {
                    let _ = engaging.engage("test");
                },
                || ()
            ),
            "EstopLease::engage did not count itself in while it waited for the lock"
        );
        assert!(!l.engage_pending(), "the engage was not counted out");
        assert!(!matches!(l.lock().unwrap().estop(), EstopState::Clear));

        let l = lease();
        let verb =
            crate::pairing_verbs::EngageEstopVerb::with_capture_ask(l.clone(), Arc::new(NoCapture));
        let engage = move || {
            verb.execute_with_caller(
                &cerud::transport::CallerIdentity::verified("test"),
                &json!({}),
            )
            .unwrap();
        };
        assert!(
            while_an_engage_waits(&l, engage, || ()),
            "engage-estop did not count itself in while it waited for the lock"
        );
        assert!(!l.engage_pending(), "engage-estop was not counted out");
        assert!(!matches!(l.lock().unwrap().estop(), EstopState::Clear));
    }

    #[test]
    fn a_counted_engage_reads_as_engaged_without_the_lease_lock() {
        let l = lease();
        let engaging = l.clone();
        let (tx, rx) = mpsc::channel();
        let refusing = l.clone();
        let counted = while_an_engage_waits(
            &l,
            move || {
                let _ = engaging.engage("test");
            },
            || {
                assert!(
                    estop_reads_engaged(&l),
                    "the watcher's read missed the counted engage"
                );
                std::thread::spawn(move || {
                    let _ = tx.send(refuse_under_estop(&refusing, TOPIC_PUBLISH).is_err());
                });
                assert_eq!(
                    rx.recv_timeout(Duration::from_secs(2)),
                    Ok(true),
                    "the refusal waited for the lease's lock instead of reading the counted engage"
                );
            },
        );
        assert!(counted);
    }

    #[test]
    fn a_bad_message_value_is_an_encode_error_before_anything_is_opened() {
        let err = TopicPublishVerb::new(lease())
            .execute(
                &json!({"topic": "/c1_act/bad", "type": "std_msgs/msg/UInt8",
                             "message": {"data": 256}}),
            )
            .unwrap_err();
        assert_eq!(class_of(err), "encode");
    }

    #[test]
    fn an_echo_result_names_the_type_and_carries_the_message() {
        let walker = cerulion_msgio::checked_walker(cerulion_msgio::builtin_schemas()).unwrap();
        let mut frame =
            cerulion_msgio::text::message_frame(&walker, "std_msgs/String", "data: hi", 9).unwrap();
        let mut header = cerulion_core::wire::WireHeader::read_from_buf(&frame).unwrap();
        header.sequence = 4;
        header.write_to_buf(&mut frame);
        assert_eq!(
            echo_result(&walker, &frame).unwrap(),
            json!({"type": "std_msgs/String", "stamp_ns": 9, "sequence": 4,
                   "message": {"data": "hi"}})
        );
    }
}
