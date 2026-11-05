// SPDX-License-Identifier: AGPL-3.0-only
//! The `rcp-eye` thread: asks in, answers out, one receipt per answer.
//!
//! The thread owns its transport ports (they stay on the thread that made
//! them): a subscriber on the ask topic, a subscriber on the frame topic and
//! a publisher on the answer topic. It is separate from the journal thread:
//! no look runs on it, and a request on the network holds nothing the
//! journal or the tools take. Its receipts go to the tools' receipt log and
//! take the same lock as the e-stop's intent receipt, which is written
//! before the e-stop frame is queued, so one eye receipt (one `sync_data`)
//! can delay an e-stop frame. The ask bound ([`limits::ASK_BURST`] asks at
//! once, [`limits::ASKS_PER_SECOND`] sustained) caps how many receipts the
//! eye writes; an ask over it is dropped unanswered and unreceipted, and
//! the gate's own deadline makes Jev absent.
//!
//! An ask's deadline runs from its wire timestamp, read against
//! [`cerulion_core::real_ns`], the host's monotonic clock that a live graph
//! stamps its frames with ([`ask_sent`]). An ask already past its deadline
//! when the thread reads it, or stamped more than [`MAX_WIRE_LEAD`] ahead of
//! that clock, is stale ([`EyeCore::stale`]): it gets no request, no answer
//! and no receipt, and is logged once, then counted. A wrist frame stamped
//! more than [`limits::TIMEOUT_MS`] behind that clock or [`MAX_WIRE_LEAD`]
//! ahead when the thread reads it is not held ([`frame_fresh`]).
//! `cerulion bag play` and `bag play --resim` publish asks and frames with
//! their recorded wire timestamps, so a played recording's asks are stale
//! here and its frames are never sent. So are the asks of a graph whose
//! clock does not follow the host's: a virtual or external time source, or a
//! run after a pause longer than the deadline, as the run clock stands still
//! while the run is paused.
//!
//! Each pass of its loop, it keeps the newest [`FRAMES_HELD`] frames, reads
//! every ask in arrival order, drops the stale asks and the asks over the ask
//! bound ([`EyeCore::intake`]) and hands each other ask to [`EyeCore::admit`], which
//! answers at once (off, no key, a bad ask, a frame that breaks the shape, a
//! limit) or yields a request. A request runs as a task on the daemon's
//! runtime and reports back over a channel; the thread writes the receipt,
//! then publishes the answer. An answer whose receipt could not be written
//! is not published, so the gate's own timeout makes Jev absent. An ask
//! naming a frame not yet held waits up to [`FRAME_WAIT`] for it.
//!
//! A request that misses its deadline is answered `timeout` at the deadline.
//! It stays on the network (and in the in-flight count) until
//! [`limits::LATE_GRACE`] after the thread read the ask; a reply in that window is
//! logged as late with a `late` receipt and never published, and at its end
//! the request is cancelled. A reply the thread takes from the channel at or
//! after its deadline is handled the same way ([`Completed::taken_at`]): the
//! ask is answered `timeout` and the reply is receipted `late`.
//!
//! The data notice is checked at spawn and again before the first request
//! of the run leaves the robot, so a key placed after the daemon started
//! still gets its line before any frame is sent. A client that cannot be
//! built (no system roots, say) leaves the eye running with every look
//! answered `unavailable`.

use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::{CerulionPublisher, CerulionSubscriber, TransportManager};
use native_ros2_messages::sensor_msgs::CompressedImage;
use native_ros2_messages::std_msgs::String as RosString;
use serde_json::Value;

use crate::eye::answer::{
    ask_id_of, classify, parse_ask, unread, unsent, Answer, AskProblem, ImageInfo, Prepared, Route,
    ThirdParty,
};
use crate::eye::client::{placeholder, request_body, request_state, sha256_hex, JevClient};
use crate::eye::config::{endpoint, EyesConfig, JevConfig};
use crate::eye::credential::{credential_path, read_credential, Credential};
use crate::eye::limits::{self, check_image, InFlight, InFlightGuard, TokenBucket};
use crate::eye::switch::{self, JevStatus, NoticeLevel, REGION_NOTICE};
use crate::eye::verdict::Outcome;
use crate::eye::{EYE_CALLER, JEV_VERB};
use crate::journal::drain_stamped_strings;
use crate::tools::ReceiptSink;
use crate::RcpdError;

/// Frames kept from the frame topic, newest last.
pub const FRAMES_HELD: usize = 8;

/// How long an ask waits for the frame it names to arrive.
pub const FRAME_WAIT: Duration = Duration::from_millis(100);

/// The largest answer frame published, bytes.
pub const MAX_ANSWER_BYTES: u32 = 64 * 1024;

/// The largest frame topic message the subscriber reads, bytes.
const MAX_FRAME_MESSAGE_BYTES: usize = 4 * 1024 * 1024;

/// How long a 429 without `Retry-After` holds requests back.
pub const DEFAULT_BACKOFF: Duration = Duration::from_secs(1);

/// The longest a `Retry-After` holds requests back.
pub const MAX_BACKOFF: Duration = Duration::from_secs(30);

/// How long the loop sleeps when nothing happened.
const IDLE: Duration = Duration::from_millis(1);

/// How far an ask's wire timestamp may run ahead of the eye's clock and
/// still be read as now; further ahead, the ask is stale.
pub const MAX_WIRE_LEAD: Duration = Duration::from_secs(1);

/// When an ask stamped `wire_ns` was published, on the `Instant` clock:
/// `now` less the ask's age by its wire timestamp against `now_ns`, the
/// [`cerulion_core::real_ns`] reading taken with `now`. A wire time at most
/// [`MAX_WIRE_LEAD`] ahead of `now_ns` reads as `now`. `None` for one
/// further ahead, or older than the `Instant` clock reaches: such an ask is
/// stale.
pub fn ask_sent(wire_ns: u64, now_ns: u64, now: Instant) -> Option<Instant> {
    match now_ns.checked_sub(wire_ns) {
        Some(age) => now.checked_sub(Duration::from_nanos(age)),
        None => (Duration::from_nanos(wire_ns - now_ns) <= MAX_WIRE_LEAD).then_some(now),
    }
}

/// Whether a wrist frame stamped `wire_ns` is held when the thread reads it
/// at `now_ns` ([`cerulion_core::real_ns`]): at most [`limits::TIMEOUT_MS`]
/// behind and at most [`MAX_WIRE_LEAD`] ahead. A frame `bag play` or a resim
/// publishes with its recorded wire timestamp is not held, so it is never
/// sent for an ask.
pub fn frame_fresh(wire_ns: u64, now_ns: u64) -> bool {
    match now_ns.checked_sub(wire_ns) {
        Some(age) => Duration::from_nanos(age) <= Duration::from_millis(limits::TIMEOUT_MS),
        None => Duration::from_nanos(wire_ns - now_ns) <= MAX_WIRE_LEAD,
    }
}

/// One ask the thread has read and not yet settled.
#[derive(Debug, Clone)]
struct Pending {
    text: String,
    sent: Instant,
    received: Instant,
}

/// The eye's configuration, resolved against the filesystem.
#[derive(Debug, Clone)]
pub struct EyeSettings {
    /// The `eyes.jev` section; `None` answers every ask `disabled`.
    pub jev: Option<JevConfig>,
    /// The ask topic.
    pub ask_topic: String,
    /// The answer topic.
    pub answer_topic: String,
    /// The System One endpoint.
    pub endpoint: Option<url::Url>,
    /// The credential file.
    pub credential_path: PathBuf,
    /// The receipt log's directory: switch, notice marker and status file.
    pub state_dir: PathBuf,
    /// The notice level of the configured region.
    pub notice: NoticeLevel,
}

impl EyeSettings {
    /// Resolve the manifest's `eyes` section. `key_file_env` is the value of
    /// [`crate::eye::JEV_KEY_FILE_ENV`].
    ///
    /// # Errors
    ///
    /// The reason the section cannot be served.
    pub fn resolve(
        eyes: &EyesConfig,
        manifest_dir: &Path,
        state_dir: &Path,
        key_file_env: Option<&str>,
    ) -> Result<Self, String> {
        let jev = eyes.jev.clone();
        let endpoint = jev.as_ref().map(|j| endpoint(&j.base_url)).transpose()?;
        let credential_path = credential_path(
            key_file_env,
            jev.as_ref().and_then(|j| j.credential_file.as_deref()),
            manifest_dir,
            state_dir,
        );
        let notice = switch::notice_level(
            jev.as_ref().and_then(|j| j.region.as_deref()),
            REGION_NOTICE,
        );
        Ok(Self {
            ask_topic: jev
                .as_ref()
                .map_or_else(|| String::from("/rcp/eye/ask"), |j| j.ask_topic.clone()),
            answer_topic: jev.as_ref().map_or_else(
                || String::from("/rcp/eye/answer"),
                |j| j.answer_topic.clone(),
            ),
            jev,
            endpoint,
            credential_path,
            state_dir: state_dir.to_path_buf(),
            notice,
        })
    }

    fn route(&self) -> Route {
        Route {
            endpoint: self
                .endpoint
                .as_ref()
                .map(url::Url::to_string)
                .unwrap_or_default(),
            model: self
                .jev
                .as_ref()
                .map(|j| j.model.clone())
                .unwrap_or_default(),
        }
    }

    /// Whether the owner's switch and the notice level leave Jev on.
    pub fn switched_on(&self) -> bool {
        self.jev.is_some() && switch::switched_on(switch::read_switch(&self.state_dir), self.notice)
    }

    /// The status file's view of now, without the counters.
    pub fn status(&self) -> JevStatus {
        let credential = match read_credential(&self.credential_path) {
            Ok(_) => String::from("present"),
            Err(problem) => problem.code().to_owned(),
        };
        let switched = self.switched_on();
        JevStatus {
            on: switched && credential == "present",
            switch: switch::read_switch(&self.state_dir).as_str().to_owned(),
            credential,
            route: self.route().endpoint,
            model: self.route().model,
            notice: self.notice.as_str().to_owned(),
            day: switch::utc_day(SystemTime::now()),
            ..JevStatus::default()
        }
    }
}

/// One frame held from the frame topic.
#[derive(Debug, Clone)]
pub struct HeldFrame {
    /// The frame's wire sequence number.
    pub seq: u32,
    /// The JPEG bytes, or the size of a frame too large to send.
    pub data: Result<Vec<u8>, usize>,
}

/// The newest [`FRAMES_HELD`] frames.
#[derive(Debug, Clone, Default)]
pub struct FrameStore {
    frames: VecDeque<HeldFrame>,
}

impl FrameStore {
    /// Keep the frame with `seq`, dropping the oldest past [`FRAMES_HELD`].
    pub fn push(&mut self, seq: u32, data: &[u8]) {
        self.frames.retain(|f| f.seq != seq);
        let data = if data.len() > limits::MAX_IMAGE_BYTES {
            Err(data.len())
        } else {
            Ok(data.to_vec())
        };
        self.frames.push_back(HeldFrame { seq, data });
        while self.frames.len() > FRAMES_HELD {
            self.frames.pop_front();
        }
    }

    /// The frame with `seq`.
    pub fn get(&self, seq: u32) -> Option<&HeldFrame> {
        self.frames.iter().rev().find(|f| f.seq == seq)
    }
}

/// A request ready for the network.
#[derive(Debug)]
pub struct Dispatch {
    /// What the answer frame records.
    pub prepared: Prepared,
    /// The body.
    pub body: Vec<u8>,
    /// The key.
    pub key: Credential,
    /// The in-flight slot, held until the request leaves the network.
    pub guard: InFlightGuard,
    /// When the thread read the ask.
    pub received: Instant,
    /// When the ask's answer is due: its wire time plus its budget.
    pub deadline: Instant,
}

/// What [`EyeCore::admit`] makes of one ask.
#[derive(Debug)]
pub enum Admit {
    /// Answered without the network.
    Answer(Box<Answer>),
    /// The frame is not held yet; ask again later.
    Wait,
    /// Send this request.
    Send(Box<Dispatch>),
    /// No usable `ask_id`: nothing to answer.
    Drop(String),
}

/// What a request task reports.
#[derive(Debug)]
pub enum Completed {
    /// The answer to publish, and a `Retry-After` from a 429.
    Answer {
        /// The answer frame.
        answer: Box<Answer>,
        /// How long the route asked to wait.
        retry_after: Option<Duration>,
        /// For an answer from a reply, the ask's deadline: taken at or after
        /// it, the answer is `timeout` and the reply late
        /// ([`Completed::taken_at`]). `None` for the `timeout` answer given
        /// at the deadline.
        deadline: Option<Instant>,
    },
    /// A reply after the deadline: logged, receipted `late`, not published.
    /// A 429 or spend-limit reply still holds requests back.
    Late {
        /// What the reply would have answered.
        answer: Box<Answer>,
        /// How long the route asked to wait.
        retry_after: Option<Duration>,
    },
    /// No reply within the grace after the deadline.
    Abandoned {
        /// The ask's id.
        ask_id: Value,
    },
}

impl Completed {
    /// What the eye thread acts on when it takes this report at `now`. An
    /// answer from a reply taken at or after its deadline becomes the ask's
    /// `timeout` answer, followed by the reply as [`Completed::Late`]; any
    /// other report is itself.
    pub fn taken_at(self, now: Instant) -> Vec<Self> {
        match self {
            Self::Answer {
                answer,
                retry_after,
                deadline: Some(deadline),
            } if now >= deadline => vec![
                Self::Answer {
                    answer: Box::new(answer.timed_out()),
                    retry_after: None,
                    deadline: None,
                },
                Self::Late {
                    answer,
                    retry_after,
                },
            ],
            other => vec![other],
        }
    }
}

/// The admission rules, without the transport.
#[derive(Debug)]
pub struct EyeCore {
    settings: Arc<EyeSettings>,
    route: Route,
    bucket: TokenBucket,
    asks: TokenBucket,
    in_flight: InFlight,
    backoff_until: Option<Instant>,
    quota_until: Option<Instant>,
}

/// How long from `wall` to the next 00:00 UTC.
pub fn until_utc_midnight(wall: SystemTime) -> Duration {
    let secs = wall
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    Duration::from_secs(86_400 - secs % 86_400)
}

fn millis(d: Duration) -> u64 {
    u64::try_from(d.as_millis()).unwrap_or(u64::MAX)
}

impl EyeCore {
    /// The rules for `settings`, with a full bucket at `now`.
    pub fn new(settings: Arc<EyeSettings>, now: Instant) -> Self {
        let route = settings.route();
        Self {
            settings,
            route,
            bucket: TokenBucket::new(now),
            asks: TokenBucket::for_asks(now),
            in_flight: InFlight::default(),
            backoff_until: None,
            quota_until: None,
        }
    }

    /// The settings.
    pub fn settings(&self) -> &EyeSettings {
        &self.settings
    }

    /// The in-flight count.
    pub fn in_flight(&self) -> &InFlight {
        &self.in_flight
    }

    /// The time an ask with `deadline_ms` has from its wire time: its own
    /// deadline, at most the configured `timeout_ms` ([`limits::TIMEOUT_MS`]
    /// without an `eyes.jev` section).
    fn budget(&self, deadline_ms: Option<u64>) -> Duration {
        let cap = self
            .settings
            .jev
            .as_ref()
            .map_or(limits::TIMEOUT_MS, |j| j.timeout_ms);
        Duration::from_millis(deadline_ms.unwrap_or(cap).min(cap))
    }

    /// Whether the ask frame `text`, published at `sent` ([`ask_sent`]) and
    /// read at `received`, is stale: `sent` is `None`, or `received` is at
    /// or past its deadline, `sent` plus its budget. The budget comes from
    /// the ask's `deadline_ms` when that is a positive integer. A stale ask
    /// gets no request, no answer and no receipt, whether Jev is on, off or
    /// unconfigured.
    pub fn stale(&self, text: &str, sent: Option<Instant>, received: Instant) -> bool {
        let Some(sent) = sent else {
            return true;
        };
        let deadline_ms = serde_json::from_str::<Value>(text)
            .ok()
            .and_then(|v| v.get("deadline_ms").and_then(Value::as_u64))
            .filter(|ms| *ms > 0);
        received >= sent + self.budget(deadline_ms)
    }

    /// Whether an ask arriving at `now` is within the ask bound: a bucket of
    /// [`limits::ASK_BURST`] refilled at [`limits::ASKS_PER_SECOND`]. An ask
    /// outside it gets no answer and no receipt.
    pub fn intake(&mut self, now: Instant) -> bool {
        self.asks.try_take(now)
    }

    /// Note what an answer says about the route. A `rate_limited` answer
    /// holds requests back for its `Retry-After` (1 s without one, at most
    /// [`MAX_BACKOFF`]): asks until then are answered `rate_limited`. A
    /// `quota` answer holds them back until 00:00 UTC by `wall`: asks until
    /// then are answered `quota`. Returns whether a quota hold began.
    pub fn note(
        &mut self,
        outcome: Outcome,
        retry_after: Option<Duration>,
        now: Instant,
        wall: SystemTime,
    ) -> bool {
        match outcome {
            Outcome::RateLimited => {
                let wait = retry_after.unwrap_or(DEFAULT_BACKOFF).min(MAX_BACKOFF);
                self.backoff_until = Some(now + wait);
                false
            }
            Outcome::Quota => {
                let began = !self.quota_until.is_some_and(|until| now < until);
                self.quota_until = Some(now + until_utc_midnight(wall));
                began
            }
            _ => false,
        }
    }

    fn bare(&self, ask_id: Value, text: &str, outcome: Outcome, detail: &str) -> Box<Answer> {
        Box::new(Answer {
            ask_id,
            outcome,
            detail: Some(detail.to_owned()),
            question_set: None,
            frame: None,
            ask_sha256: sha256_hex(text.as_bytes()),
            route: self.route.endpoint.clone(),
            model_requested: self.route.model.clone(),
            model_answered: None,
            request_sha256: None,
            image_sha256: None,
            image: None,
            questions: None,
            state: None,
            answers: None,
            usage: None,
            latency_ms: None,
            provenance: ThirdParty,
        })
    }

    /// Decide what to do with the ask frame `text`, published at `sent`
    /// ([`ask_sent`]) and read at `received`, at `now`, with `frames` held.
    /// The thread drops a stale ask ([`Self::stale`]) before this. The
    /// deadline is `sent` plus the ask's budget, and the frame wait runs from
    /// `received`. Checks run in this order, all before any network call: the
    /// ask's id; the switch (`disabled`); the ask's shape (`malformed`); the
    /// key (`no_credential`); a 429 backoff (`rate_limited`); a spend-limit
    /// hold (`quota`); the deadline (`timeout`); the frame (wait, then
    /// `refused`); the image and token shape (`refused`); the in-flight and
    /// rate limits (`throttled`).
    pub fn admit(
        &mut self,
        text: &str,
        sent: Instant,
        received: Instant,
        now: Instant,
        frames: &FrameStore,
    ) -> Admit {
        let Some(config) = self.settings.jev.as_ref() else {
            return match ask_id_of(text) {
                Some(id) => Admit::Answer(self.bare(id, text, Outcome::Disabled, "no_eyes_jev")),
                None => Admit::Drop(String::from("no usable ask_id")),
            };
        };
        let on = self.settings.switched_on();
        let ask = match parse_ask(text, config) {
            Ok(ask) => ask,
            Err(AskProblem::NoId(why)) => return Admit::Drop(why),
            Err(problem) => {
                let outcome = if on {
                    Outcome::Malformed
                } else {
                    Outcome::Disabled
                };
                return match unread(text, &problem, &self.route, outcome) {
                    Some(answer) => Admit::Answer(Box::new(answer)),
                    None => Admit::Drop(String::from("unreadable ask")),
                };
            }
        };
        let answer = |outcome: Outcome, detail: &str| {
            Admit::Answer(Box::new(unsent(&ask, &self.route, outcome, Some(detail))))
        };
        if !on {
            return answer(Outcome::Disabled, "switched_off");
        }
        let key = match read_credential(&self.settings.credential_path) {
            Ok(key) => key,
            Err(problem) => return answer(Outcome::NoCredential, problem.code()),
        };
        if self.backoff_until.is_some_and(|until| now < until) {
            return answer(Outcome::RateLimited, "backing_off");
        }
        if self.quota_until.is_some_and(|until| now < until) {
            return answer(Outcome::Quota, "daily_limit");
        }
        let deadline = sent + self.budget(ask.deadline_ms);
        if now >= deadline {
            return answer(Outcome::Timeout, "deadline_before_send");
        }
        let Some(held) = frames.get(ask.frame.seq) else {
            if now < received + FRAME_WAIT {
                return Admit::Wait;
            }
            return answer(Outcome::Refused, "frame_not_held");
        };
        let image = match &held.data {
            Ok(bytes) => bytes,
            Err(_) => return answer(Outcome::Refused, limits::ShapeRefusal::TooManyBytes.code()),
        };
        let size = match check_image(image) {
            Ok(size) => size,
            Err(refusal) => return answer(Outcome::Refused, refusal.code()),
        };
        let Some(set) = config.question_set(&ask.question_set) else {
            return answer(Outcome::Malformed, "unknown_question_set");
        };
        let image_sha256 = sha256_hex(image);
        let state = request_state(&set.state, &placeholder(&image_sha256), &ask.numbers);
        let questions = serde_json::to_value(&set.questions).unwrap_or(Value::Null);
        let text_bytes = state.to_string().len() + questions.to_string().len() + config.model.len();
        if limits::estimate_input_tokens(text_bytes, size.width, size.height)
            > limits::MAX_INPUT_TOKENS
        {
            return answer(Outcome::Refused, limits::ShapeRefusal::TooManyTokens.code());
        }
        let Some(guard) = self.in_flight.try_acquire() else {
            return answer(Outcome::Throttled, "in_flight");
        };
        if !self.bucket.try_take(now) {
            return answer(Outcome::Throttled, "rate");
        }
        let body = request_body(&config.model, &state, &questions, image);
        let (expected, choices) = Prepared::expectations(set);
        let prepared = Prepared {
            route: self.route.endpoint.clone(),
            model: config.model.clone(),
            request_sha256: sha256_hex(&body),
            image_sha256,
            image: ImageInfo::new(size, image.len()),
            questions,
            state,
            expected,
            choices,
            visible: set
                .visible
                .as_ref()
                .map(|v| (v.question.clone(), v.threshold)),
            ask,
        };
        Admit::Send(Box::new(Dispatch {
            prepared,
            body,
            key,
            guard,
            received,
            deadline,
        }))
    }
}

/// Send one request and report on `done`: the reply's answer with the
/// deadline, or a `timeout` at the deadline followed by a late reply or an
/// abandonment.
pub async fn request(client: Arc<JevClient>, dispatch: Dispatch, done: mpsc::Sender<Completed>) {
    let Dispatch {
        prepared,
        body,
        key,
        guard,
        received,
        deadline,
    } = dispatch;
    let _guard = guard;
    let call = client.post(body, &key);
    tokio::pin!(call);
    tokio::select! {
        reply = &mut call => {
            let latency = millis(received.elapsed());
            let retry_after = reply.as_ref().ok().filter(|r| r.status == 429).and_then(|r| r.retry_after);
            let answer = classify(&prepared, reply.as_ref().map_err(|e| *e), latency);
            // When both branches are ready, `select!` may take this one after
            // the deadline; `Completed::taken_at` answers it `timeout`.
            let _ = done.send(Completed::Answer { answer: Box::new(answer), retry_after, deadline: Some(deadline) });
        }
        () = tokio::time::sleep_until(tokio::time::Instant::from_std(deadline)) => {
            let latency = millis(received.elapsed());
            let timeout = prepared.answer(Outcome::Timeout, Some("deadline"), Some(latency));
            let _ = done.send(Completed::Answer { answer: Box::new(timeout), retry_after: None, deadline: None });
            let left = limits::LATE_GRACE.saturating_sub(received.elapsed());
            match tokio::time::timeout(left, &mut call).await {
                Ok(reply) => {
                    let latency = millis(received.elapsed());
                    let retry_after = reply.as_ref().ok().filter(|r| r.status == 429).and_then(|r| r.retry_after);
                    let late = classify(&prepared, reply.as_ref().map_err(|e| *e), latency);
                    let _ = done.send(Completed::Late { answer: Box::new(late), retry_after });
                }
                Err(_) => {
                    let _ = done.send(Completed::Abandoned { ask_id: prepared.ask.ask_id.clone() });
                }
            }
        }
    }
}

/// The running eye thread. Dropping it stops the thread.
#[derive(Debug)]
pub struct EyeHost {
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Drop for EyeHost {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

struct Ports {
    asks: CerulionSubscriber,
    frames: Option<CerulionSubscriber>,
    answers: CerulionPublisher,
}

impl Ports {
    fn open(manager: &TransportManager, settings: &EyeSettings) -> Result<Self, RcpdError> {
        let answers = manager.create_publisher_simple(
            &settings.answer_topic,
            MaxSliceLen::const_new(MAX_ANSWER_BYTES),
        )?;
        let asks = manager.create_subscriber(&settings.ask_topic)?;
        let frames = match &settings.jev {
            Some(jev) => Some(manager.create_subscriber(&jev.frame_topic)?),
            None => None,
        };
        Ok(Self {
            asks,
            frames,
            answers,
        })
    }
}

/// The counters behind the status file.
#[derive(Debug, Default)]
struct Counters {
    day: String,
    asks: u64,
    looks: u64,
    limit_reached: bool,
    last_outcome: Option<Outcome>,
}

struct EyeLoop {
    core: EyeCore,
    client: Option<Arc<JevClient>>,
    ports: Ports,
    receipts: ReceiptSink,
    runtime: tokio::runtime::Handle,
    store: FrameStore,
    pending: VecDeque<Pending>,
    done_tx: mpsc::Sender<Completed>,
    done_rx: mpsc::Receiver<Completed>,
    counters: Counters,
    /// Frames that could not be read: loud once, then counted.
    frame_failures: FailureRegimeLatch,
    /// Answers absent for a refused shape, another model or a malformed
    /// field: loud once, then counted, until an answer is `ok`.
    absent_answers: FailureRegimeLatch,
    /// Asks dropped over the ask bound: loud once, then counted, until an
    /// ask is within it.
    ask_flood: FailureRegimeLatch,
    /// Asks dropped as stale: loud once, then counted, until an ask is not.
    stale_asks: FailureRegimeLatch,
    /// Wrist frames dropped for their wire time: loud once, then counted.
    stale_frames: FailureRegimeLatch,
    /// Whether this run's data notice was checked with Jev on.
    notice_given: bool,
    /// The release the route last answered as.
    release: Option<String>,
}

/// Log one failure through `latch`: the regime's first and each decade at
/// warn, the rest at debug.
fn log_failure(latch: &mut FailureRegimeLatch, what: &'static str, detail: &str) {
    match latch.on_failure() {
        RegimeDecision::Loud => tracing::warn!(failure = what, detail, "rcp-eye failure"),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            failure = what,
            detail,
            total_failures = total,
            suppressed,
            "rcp-eye failure"
        ),
        RegimeDecision::Suppressed { suppressed } => {
            tracing::debug!(failure = what, detail, suppressed, "rcp-eye failure");
        }
    }
}

/// The client for `endpoint`, built by `build`. `None` without an endpoint,
/// and `None` with one error line when `build` fails: every look is then
/// answered `unavailable` (`no_client`).
fn client_for(
    endpoint: Option<&url::Url>,
    build: impl FnOnce(url::Url) -> Result<JevClient, String>,
) -> Option<Arc<JevClient>> {
    match build(endpoint?.clone()) {
        Ok(client) => Some(Arc::new(client)),
        Err(reason) => {
            tracing::error!(
                error = %reason,
                "Jev client not built; every look is answered unavailable"
            );
            None
        }
    }
}

/// Log the data notice [`switch::startup_notice`] gives for `settings`,
/// with Jev `on` or not.
fn give_notice(settings: &EyeSettings, on: bool) {
    let host = settings
        .endpoint
        .as_ref()
        .and_then(|u| u.host_str().map(str::to_owned))
        .unwrap_or_default();
    match switch::startup_notice(&settings.state_dir, settings.notice, on, &host) {
        Some((true, line)) => tracing::warn!(notice = %line, "Jev data notice"),
        Some((false, line)) => tracing::info!(notice = %line, "Jev data notice"),
        None => {}
    }
}

impl EyeHost {
    /// Open the ports and start the thread; returns once the ports exist.
    /// Prints the data notice the region's level calls for. A client that
    /// cannot be built is logged, and every look is answered `unavailable`.
    ///
    /// # Errors
    ///
    /// [`RcpdError::Transport`] when a port cannot be created,
    /// [`RcpdError::ObserverSpawn`] when the thread cannot start.
    pub fn spawn(
        manager: Arc<TransportManager>,
        settings: EyeSettings,
        receipts: ReceiptSink,
        runtime: tokio::runtime::Handle,
    ) -> Result<Self, RcpdError> {
        let client = client_for(settings.endpoint.as_ref(), JevClient::new);
        let mut notice_given = false;
        if settings.jev.is_some() {
            let status = settings.status();
            give_notice(&settings, status.on);
            notice_given = status.on;
            tracing::info!(
                on = status.on,
                credential = %status.credential,
                route = %status.route,
                model = %status.model,
                "Jev eye ready"
            );
        }
        let stop = Arc::new(AtomicBool::new(false));
        let thread_stop = Arc::clone(&stop);
        let (ready_tx, ready_rx) = mpsc::sync_channel(1);
        let thread = std::thread::Builder::new()
            .name(String::from("rcp-eye"))
            .spawn(move || {
                let ports = match Ports::open(&manager, &settings) {
                    Ok(ports) => {
                        let _ = ready_tx.send(Ok(()));
                        ports
                    }
                    Err(e) => {
                        let _ = ready_tx.send(Err(e));
                        return;
                    }
                };
                let (done_tx, done_rx) = mpsc::channel();
                let core = EyeCore::new(Arc::new(settings), Instant::now());
                let mut eye = EyeLoop {
                    core,
                    client,
                    ports,
                    receipts,
                    runtime,
                    store: FrameStore::default(),
                    pending: VecDeque::new(),
                    done_tx,
                    done_rx,
                    counters: Counters::default(),
                    frame_failures: FailureRegimeLatch::new(),
                    absent_answers: FailureRegimeLatch::new(),
                    ask_flood: FailureRegimeLatch::new(),
                    stale_asks: FailureRegimeLatch::new(),
                    stale_frames: FailureRegimeLatch::new(),
                    notice_given,
                    release: None,
                };
                eye.run(&thread_stop);
            })
            .map_err(RcpdError::ObserverSpawn)?;
        match ready_rx.recv() {
            Ok(Ok(())) => Ok(Self {
                stop,
                thread: Some(thread),
            }),
            Ok(Err(e)) => Err(e),
            Err(_) => Err(RcpdError::Session {
                reason: String::from("the eye thread exited before opening its ports"),
            }),
        }
    }
}

impl EyeLoop {
    fn run(&mut self, stop: &AtomicBool) {
        while !stop.load(Ordering::SeqCst) {
            let mut busy = self.read_frames();
            let now = Instant::now();
            let now_ns = cerulion_core::real_ns();
            for (text, wire_ns) in
                drain_stamped_strings(&self.ports.asks, &self.core.settings().ask_topic)
            {
                busy = true;
                let sent = ask_sent(wire_ns, now_ns, now);
                if self.core.stale(&text, sent, now) {
                    log_failure(
                        &mut self.stale_asks,
                        "dropped a stale ask",
                        "its wire time is past its deadline or ahead of this host's clock: \
                         no request, no answer, no receipt",
                    );
                    continue;
                }
                if let Some(suppressed) = self.stale_asks.on_success() {
                    tracing::info!(suppressed, "asks within their deadline again");
                }
                let Some(sent) = sent else { continue };
                if self.core.intake(now) {
                    if let Some(suppressed) = self.ask_flood.on_success() {
                        tracing::info!(suppressed, "asks within the ask bound again");
                    }
                    self.pending.push_back(Pending {
                        text,
                        sent,
                        received: now,
                    });
                } else {
                    log_failure(
                        &mut self.ask_flood,
                        "dropped an ask over the ask bound",
                        "unanswered, no receipt",
                    );
                }
            }
            busy |= self.admit_pending();
            while let Ok(done) = self.done_rx.try_recv() {
                self.completed(done);
                busy = true;
            }
            if !busy {
                std::thread::sleep(IDLE);
            }
        }
    }

    fn read_frames(&mut self) -> bool {
        let Some(frames) = &self.ports.frames else {
            return false;
        };
        let store = &mut self.store;
        let latch = &mut self.frame_failures;
        let stale = &mut self.stale_frames;
        let now_ns = cerulion_core::real_ns();
        let mut any = false;
        let read = frames.try_receive(|message| {
            any = true;
            let dropped = |latch: &mut FailureRegimeLatch, detail: &str| {
                log_failure(latch, "dropped a wrist frame the eye cannot read", detail);
            };
            if message.header().schema_hash != CompressedImage::SCHEMA_HASH {
                dropped(latch, "not sensor_msgs/CompressedImage");
                return;
            }
            if !frame_fresh(message.header().timestamp_ns, now_ns) {
                log_failure(
                    stale,
                    "dropped a wrist frame stamped outside this host's clock",
                    "not held, never sent",
                );
                return;
            }
            let payload = message.payload();
            let min = CompressedImage::WIRE_FIXED_SIZE + 8 * CompressedImage::VARIABLE_FIELD_COUNT;
            if payload.len() > MAX_FRAME_MESSAGE_BYTES
                || payload.len() < min
                || payload.as_ptr().align_offset(8) != 0
            {
                dropped(latch, "payload size or alignment");
                return;
            }
            let seq = message.header().sequence;
            // The store copies the image bytes only when they fit the shape.
            let read = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                store.push(seq, CompressedImage::build_reader(payload).data());
            }));
            match read {
                Ok(()) => {
                    if let Some(suppressed) = latch.on_success() {
                        tracing::info!(suppressed, "wrist frames readable again");
                    }
                }
                Err(_) => dropped(latch, "the reader refused the payload"),
            }
        });
        if let Err(e) = read {
            tracing::warn!(error = %e, "frame receive failed");
        }
        any
    }

    fn admit_pending(&mut self) -> bool {
        let mut any = false;
        let mut waiting = VecDeque::new();
        while let Some(ask) = self.pending.pop_front() {
            match self.core.admit(
                &ask.text,
                ask.sent,
                ask.received,
                Instant::now(),
                &self.store,
            ) {
                Admit::Answer(answer) => {
                    self.finish(*answer);
                    any = true;
                }
                Admit::Wait => waiting.push_back(ask),
                Admit::Send(dispatch) => {
                    any = true;
                    match self.client.as_ref().map(Arc::clone) {
                        Some(client) => {
                            self.counters_today().looks += 1;
                            if !self.notice_given {
                                give_notice(self.core.settings(), true);
                                self.notice_given = true;
                            }
                            let done = self.done_tx.clone();
                            self.runtime.spawn(request(client, *dispatch, done));
                        }
                        None => {
                            let answer = dispatch.prepared.answer(
                                Outcome::Unavailable,
                                Some("no_client"),
                                None,
                            );
                            self.finish(answer);
                        }
                    }
                }
                Admit::Drop(why) => {
                    any = true;
                    tracing::warn!(reason = %why, "dropped an ask with no usable ask_id");
                }
            }
        }
        self.pending = waiting;
        any
    }

    fn completed(&mut self, done: Completed) {
        for done in done.taken_at(Instant::now()) {
            self.settle(done);
        }
    }

    fn settle(&mut self, done: Completed) {
        match done {
            Completed::Answer {
                answer,
                retry_after,
                ..
            } => {
                let held = self.core.note(
                    answer.outcome,
                    retry_after,
                    Instant::now(),
                    SystemTime::now(),
                );
                if held {
                    tracing::info!(
                        "Jev reached its daily limit on this robot; off until 00:00 UTC"
                    );
                }
                self.finish(*answer);
            }
            Completed::Late {
                answer,
                retry_after,
            } => {
                // A late 429 or spend-limit reply still holds the route.
                let held = self.core.note(
                    answer.outcome,
                    retry_after,
                    Instant::now(),
                    SystemTime::now(),
                );
                if held {
                    tracing::info!(
                        "Jev reached its daily limit on this robot; off until 00:00 UTC"
                    );
                }
                tracing::info!(
                    ask_id = %answer.ask_id,
                    latency_ms = answer.latency_ms.unwrap_or(0),
                    outcome = %answer.outcome,
                    "late: a Jev answer arrived after its deadline and was dropped"
                );
                self.receipts.record(
                    EYE_CALLER,
                    JEV_VERB,
                    answer.receipt_digest(),
                    cerud::receipt::ReceiptOutcome::Error(String::from("late")),
                );
            }
            Completed::Abandoned { ask_id } => {
                tracing::warn!(%ask_id, "a Jev request had no reply within the grace after its deadline");
            }
        }
    }

    fn counters_today(&mut self) -> &mut Counters {
        let day = switch::utc_day(SystemTime::now());
        if self.counters.day != day {
            self.counters = Counters {
                day,
                ..Counters::default()
            };
        }
        &mut self.counters
    }

    /// Receipt, then publish, then the status file.
    fn finish(&mut self, mut answer: Answer) {
        let mut text = answer.to_json();
        if text.len() > MAX_ANSWER_BYTES as usize - 1024 {
            answer.outcome = Outcome::Malformed;
            answer.detail = Some(String::from("answer_too_large"));
            answer.answers = None;
            text = answer.to_json();
        }
        match answer.outcome {
            Outcome::Refused | Outcome::WrongModel | Outcome::Malformed => {
                let detail = format!(
                    "{} {}",
                    answer.outcome,
                    answer.detail.as_deref().unwrap_or("")
                );
                log_failure(&mut self.absent_answers, "Jev answer absent", &detail);
            }
            Outcome::Ok => {
                if let Some(suppressed) = self.absent_answers.on_success() {
                    tracing::info!(suppressed, "Jev answers again");
                }
                if answer.model_answered.is_some() && answer.model_answered != self.release {
                    tracing::info!(
                        model_answered = answer.model_answered.as_deref().unwrap_or(""),
                        "Jev answers as this release"
                    );
                    self.release.clone_from(&answer.model_answered);
                }
            }
            _ => {}
        }
        if answer
            .usage
            .as_ref()
            .is_some_and(|u| u.input_tokens > limits::MAX_INPUT_TOKENS)
        {
            tracing::warn!(
                ask_id = %answer.ask_id,
                input_tokens = answer.usage.as_ref().map_or(0, |u| u.input_tokens),
                "a Jev request used more input tokens than the request shape allows"
            );
        }
        let receipt = self.receipts.record(
            EYE_CALLER,
            JEV_VERB,
            answer.receipt_digest(),
            answer.receipt_outcome(),
        );
        if receipt.is_none() {
            tracing::error!(ask_id = %answer.ask_id, "answer not published: its receipt could not be written");
            return;
        }
        match self.ports.answers.loan_proxy::<RosString>() {
            // The frame is published when the proxy drops.
            Ok(mut proxy) => {
                if let Err(e) = proxy.set_data(&text) {
                    tracing::error!(error = %e, "answer not published");
                }
            }
            Err(e) => tracing::error!(error = %e, "answer not published"),
        }
        let outcome = answer.outcome;
        let counters = self.counters_today();
        counters.asks += 1;
        counters.limit_reached |= outcome.is_limit();
        counters.last_outcome = Some(outcome);
        let mut status = self.core.settings().status();
        status.day.clone_from(&self.counters.day);
        status.asks_today = self.counters.asks;
        status.looks_today = self.counters.looks;
        status.limit_reached_today = self.counters.limit_reached;
        status.last_outcome = self.counters.last_outcome.map(|o| o.as_str().to_owned());
        if let Err(e) = switch::write_status(&self.core.settings().state_dir, &status) {
            tracing::debug!(error = %e, "status file not written");
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use super::{ask_sent, client_for, frame_fresh, FrameStore, MAX_WIRE_LEAD};
    use crate::eye::limits::{MAX_IMAGE_BYTES, TIMEOUT_MS};

    #[test]
    fn an_asks_wire_time_maps_onto_the_instant_clock() {
        let now = Instant::now();
        let now_ns = 5_000_000_000_000;
        let lead = u64::try_from(MAX_WIRE_LEAD.as_nanos()).expect("lead");
        // Stamped 300 ms ago.
        assert_eq!(
            ask_sent(now_ns - 300_000_000, now_ns, now),
            Some(now - Duration::from_millis(300))
        );
        assert_eq!(ask_sent(now_ns, now_ns, now), Some(now));
        // Ahead by at most the lead reads as now; further ahead, never.
        assert_eq!(ask_sent(now_ns + lead, now_ns, now), Some(now));
        assert_eq!(ask_sent(now_ns + lead + 1, now_ns, now), None);
        // An hour old: an hour before now.
        let hour = Duration::from_secs(3_600);
        assert_eq!(
            ask_sent(now_ns - 3_600_000_000_000, now_ns, now),
            now.checked_sub(hour)
        );
    }

    #[test]
    fn a_frame_is_held_only_within_the_timeout_behind_and_the_lead_ahead() {
        let now_ns = 5_000_000_000_000;
        let timeout = TIMEOUT_MS * 1_000_000;
        let lead = u64::try_from(MAX_WIRE_LEAD.as_nanos()).expect("lead");
        assert!(frame_fresh(now_ns, now_ns));
        assert!(frame_fresh(now_ns - timeout, now_ns));
        assert!(!frame_fresh(now_ns - timeout - 1, now_ns));
        assert!(frame_fresh(now_ns + lead, now_ns));
        assert!(!frame_fresh(now_ns + lead + 1, now_ns));
        assert!(!frame_fresh(0, now_ns));
    }

    #[test]
    fn a_frame_of_exactly_the_byte_limit_is_held_and_one_byte_more_is_not() {
        let mut store = FrameStore::default();
        store.push(1, &vec![0x20_u8; MAX_IMAGE_BYTES]);
        store.push(2, &vec![0x20_u8; MAX_IMAGE_BYTES + 1]);
        let held = store.get(1).expect("frame 1");
        assert_eq!(held.data.as_ref().map(Vec::len), Ok(200_000));
        let over = store.get(2).expect("frame 2");
        assert_eq!(over.data, Err(200_001));
    }

    #[test]
    fn a_client_that_cannot_be_built_leaves_the_eye_without_one() {
        let endpoint = url::Url::parse("https://openrouter.ai/api/v1/systemone").expect("url");
        let built = client_for(Some(&endpoint), |_| {
            Err(String::from(
                "No CA certificates were loaded from the system",
            ))
        });
        assert!(built.is_none());
        assert!(client_for(None, |_| unreachable!("no endpoint, no build")).is_none());
    }
}
