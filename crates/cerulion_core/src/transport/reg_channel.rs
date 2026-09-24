// SPDX-License-Identifier: AGPL-3.0-only
//! The cross-process runtime-egress REGISTRATION CHANNEL.
//!
//! # Why this exists
//!
//! The gateway core has [`GatewayRuntime::register_runtime_topic`](super::gateway::GatewayRuntime::register_runtime_topic)
//! — but that entry point is only callable INSIDE the gateway process. A
//! `cerulion ros2 attach` robot's dds_bridge creates ~90 raw-route SHM publishers
//! (`/utlidar/cloud`, …) at RUNTIME, and those publishers live in the GRAPH /
//! WORKER process, which is deliberately NETWORK-FREE (the gateway is a
//! separate process owning the robot's one zenoh session). This module is the
//! bridge across that process boundary: a network-free worker calls
//! [`TransportManager::register_dynamic_egress_topic`](super::TransportManager::register_dynamic_egress_topic),
//! which pushes a `(topic, schema_hash)` record over a reserved SHM control
//! service; the gateway drains that service each drive pass and feeds every
//! record through `register_runtime_topic`, so the raw route becomes both
//! ANNOUNCED (discoverable) AND demand-grantable exactly like a YAML-declared
//! producer.
//!
//! # The control service
//!
//! One iceoryx2 pub/sub service at the fixed name
//! [`REG_CHANNEL_SERVICE_NAME`] (`/__cerulion/gateway_topics`, under the
//! reserved [`RESERVED_TOPIC_PREFIX`](super::gateway::RESERVED_TOPIC_PREFIX)).
//! It is a FRAMEWORK control service, NOT a graph topic, so it deliberately does
//! NOT ride the graph-topic conventions:
//!
//! - provisioned for MANY writer processes ([`REG_CHANNEL_MAX_WRITERS`]) — the
//!   opposite of a graph topic's `max_publishers = 1` single-writer rule — plus
//!   a small reader headroom ([`REG_CHANNEL_MAX_READERS`]);
//! - a bounded record size ([`MAX_REG_RECORD_LEN`]);
//! - NO event service (no wake): the gateway POLLS it every `drive_once`, so a
//!   listener/notifier would be dead weight;
//! - NO `/data` suffix on the service name, so it is invisible to
//!   `cerulion topic list` (which enumerates `*/data` services) — see the
//!   record/replay-inertness note below.
//!
//! # The ordering race and its solution: PERIODIC RE-PUBLICATION
//!
//! The bring-up race: the dds_bridge opens its raw routes when the graph starts,
//! and the gateway process starts at roughly the same time — it may open/drain
//! the control service only AFTER many registrations were already pushed. No
//! registration may be lost to this race.
//!
//! iceoryx2 native late-joiner history does NOT close this race on its own: a
//! publisher pushes history into a subscriber connection only inside
//! `Publisher::update_connections` (driven by `send`), so a FIRE-AND-FORGET
//! publisher that goes quiet after its last registration — exactly the
//! dds_bridge shape — would never deliver its buffered records to a gateway that
//! connects afterwards (verified against vendored iceoryx2 0.9.1
//! `port/publisher.rs::deliver_sample_history`, called only on a NEW connection
//! during a `send`-driven `update_connections`).
//!
//! So the channel keeps the AUTHORITATIVE registration set in an UNBOUNDED
//! process-local map (`RegChannelInner::records`) and RE-PUBLISHES the whole
//! set periodically ([`REG_REPUBLISH_INTERVAL`]) from a background thread. Every
//! send is a LIVE send, so whenever the gateway's subscriber connects it receives
//! the full set within one interval — regardless of which process started first,
//! and self-healing across a gateway restart or a transient queue overflow (a
//! dropped record simply re-arrives on the next republish, and
//! `register_runtime_topic` is idempotent). The publisher is ALSO fired once
//! immediately on each `RegistrationChannel::register` so the common
//! gateway-already-up path pays no interval latency.
//!
//! ## Capacity — what happens at registration 101?
//!
//! Nothing bad. The record map is unbounded, so registrations 101, 1000, … all
//! land — there is NO history-ring eviction (the class of bug a fixed
//! `history_size = N` would have at registration N+1). The only bounded resource
//! is the reader's per-pass queue ([`REG_CHANNEL_QUEUE_DEPTH`], 256), which
//! comfortably exceeds the real robot's ~90-route burst, so in practice the whole
//! set lands in ONE drain pass. A single-instant burst LARGER than the queue
//! overflows it (iceoryx2 `drop_oldest`), but the republish ROTATES its send
//! order each round (`RegChannelInner::republish`), so overflow never
//! perpetually starves the same topics — every registration lands within
//! ⌈N / queue_depth⌉ republishes, and `register_runtime_topic` is idempotent, so
//! the gateway's set still converges to the full N for any N.
//!
//! ## Capacity — the writer-slot arithmetic
//!
//! The bounded resource on the WRITER side is the control service's publisher
//! slots ([`REG_CHANNEL_MAX_WRITERS`]), and its unit is PROCESSES: each
//! process holds exactly ONE control publisher no matter how many topics it
//! registers (`TransportManager::dynamic_egress_channel` serializes creation
//! and every `register_dynamic_egress_topic` in the process shares the one
//! channel; on the rmw path the `TransportManager` is itself a process
//! singleton, so a nav2/MoveIt process with dozens of `rmw_create_publisher`s
//! consumes ONE slot). Process #65's registration is REFUSED loudly (the
//! control publisher create fails; the rmw caller's process-wide latch warns;
//! local pub/sub is unaffected and that process's topics stay local-only),
//! and the slot frees when a registered process exits. See
//! [`REG_CHANNEL_MAX_WRITERS`] for why the cap is deliberately NOT raised as
//! belt-and-braces.
//!
//! # No retire / withdraw — registration is PRESENCE-BASED
//!
//! There is deliberately NO unregister primitive on either side of this
//! channel: the writer is insert + periodic republish (nothing removes a
//! record), and the gateway's `register_runtime_topic` has no inverse and no
//! expiry. A destroyed producer's record therefore OUTLIVES it — the
//! registering process republishes the topic until IT exits, and a gateway
//! that heard it keeps the topic announced + demand-grantable until the
//! GATEWAY restarts. (`rmw_destroy_publisher` states the same model at its
//! call site; a removal primitive added here must be wired there and update
//! both.)
//!
//! That surface stays TRUE without a withdraw, which is why one is not
//! built: the served catalog row carries the truth about a dead route —
//! `CatalogEntry::producer_count` probes the LIVE iceoryx2 port count at
//! serve time (`Some(0)` once the producer is gone), and
//! `CatalogEntry::liveness` reports what the robot's data-flow
//! observer actually saw (a destroyed producer's row ages out of `streaming`
//! and cannot return to it; a never-produced row reads `no_data`) — so a desk
//! dims the row rather than rendering a phantom live stream, and a re-created
//! producer on the same name (the routine rclcpp restart shape) lands
//! straight back on the standing record with zero re-announce latency. A
//! withdraw would buy only the removal of a correctly-annotated row, at the
//! price of a retraction protocol (ordering against the periodic republish,
//! loss against a gateway that missed the retraction, re-add races) —
//! rejected until someone finds a user-visible lie the liveness annotation
//! cannot cover. Pinned by `rmw_canonical_names_test::
//! a_subscription_registers_nothing_and_a_destroyed_publishers_registration_outlives_it`
//! (the process-side record) and `gateway_iox2_test::
//! a_destroyed_producers_topic_stays_cataloged_with_honest_liveness`
//! (the desk-visible surface).
//!
//! # Pump-thread failure containment
//!
//! The republish belt must not die quietly. Two layers:
//!
//! - **Per-tick containment**: the pump thread wraps each republish tick in
//!   `catch_unwind` (`pump_tick`), so a panic surfacing from the send path
//!   (the tick's only panic-capable work — an iceoryx2 loan + send) kills the
//!   TICK, not the belt. The panic is counted unconditionally
//!   (`RegChannelInner::pump_tick_panics`, Principle #3), reported through the
//!   shared [`FailureRegimeLatch`] flood discipline (loud first `error!`,
//!   `debug!` repeats carrying the suppressed count, decade re-announcements,
//!   ONE recovery `info!` on the next clean tick), and the next interval
//!   re-attempts the whole set — the belt is idempotent re-publication, so a
//!   torn tick re-heals by construction, and a pump that panics EVERY tick
//!   cannot flood the log. The flood's OTHER channel is closed too: the
//!   process panic HOOK runs before `catch_unwind` sees the panic, so the
//!   default hook printed one `thread ... panicked at ...` stderr line per
//!   contained tick — a `Once`-installed wrapping hook now swallows exactly
//!   the pump's contained-region reports (thread-local, RAII-scoped) and
//!   delegates for every other thread, re-serving the captured `file:line`
//!   on the latch's own bounded lines (see the hook-suppression section).
//! - **True liveness**: the thread body holds an RAII `PumpLiveGuard`
//!   that clears the pump's live flag on ANY exit — a normal teardown return
//!   or a panic that escaped the per-tick containment — so
//!   `RegistrationChannel::registration_pump_active` can never report a
//!   dead pump as live (an accessor that read the HANDLE's
//!   existence would, since a handle survives the thread it once named).
//!
//! A pump whose THREAD genuinely died stays DOWN, loudly — later `register`
//! calls do NOT respawn it (see `RegistrationChannel::ensure_pump_running`
//! for the rationale). Each `register` still live-sends its record, so an
//! already-up gateway keeps receiving; the degraded state is observable via
//! `registration_pump_active()`.
//!
//! # The cross-process first-creation collision
//!
//! The control service is opened `open_or_create` by BOTH sides at boot: the
//! gateway process's `RegistrationReader` and each worker process's
//! `RegistrationChannel`. On a cold machine those boot simultaneously, and
//! iceoryx2's `open_or_create` has a window where the loser observes the
//! winner's creation IN PROGRESS: it waits up to `global.creation_timeout`
//! (500 ms default) for the static storage to finalize and then surfaces
//! `HangsInCreation` (verified against vendored iceoryx2 0.9.1
//! `service/builder/mod.rs::is_service_available` —
//! `InitializationNotYetFinalized` maps to `HangsInCreation`, and the variant
//! is NOT in `open_or_create_impl`'s internal retry set, so it propagates to
//! this module rather than being retried upstream).
//!
//! The collision is transient BY CONSTRUCTION iff the creating peer is ALIVE:
//! creating this tiny service is microseconds of work, so a live creator
//! finalizes within one `creation_timeout` window and a re-attempt succeeds.
//! `open_control_service` therefore re-attempts a `HangsInCreation` up to
//! [`CONTROL_OPEN_ATTEMPTS`] total attempts ([`CONTROL_OPEN_RETRY_DELAY`]
//! apart; each attempt already carries iceoryx2's own 500 ms wait) — bounded,
//! never a spin, with a `warn!` per re-attempt and on a heal (at most
//! `CONTROL_OPEN_ATTEMPTS` lines per open, and the open runs once per process
//! boot, so no flood is constructible and no regime latch is needed here).
//! Exhausting the bound means the creator DIED mid-creation (stale artifacts
//! no retry can heal) and is a hard, actionable error naming the service,
//! both roles, and the stale-artifact remedy.
//!
//! # Record/replay inertness
//!
//! `/__cerulion/gateway_topics` is inert to record + replay BY CONSTRUCTION, not
//! by a filter added here:
//!
//! - **Recording (`bagd`)** taps only the topics in its explicit tap plan
//!   (graph-produced topics / `--topics-json`); it never auto-discovers all
//!   iceoryx2 services. The control service is not a graph topic, so it is never
//!   tapped. `bagd` additionally refuses any user topic under the bag layer's
//!   `__cerulion/` reserved prefix (the `cerulion_bag` crate).
//! - **Replay** reads bag CHANNELS excluding `__cerulion/*`, and the control
//!   service was never recorded, so no such channel exists.
//! - **`cerulion topic list`** enumerates `*/data` services; the control
//!   service's name has no `/data` suffix, so it never appears (pinned by
//!   `reg_channel_service_name_has_no_data_suffix` in the unit tests below).

use std::cell::{Cell, RefCell};
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, Once};
use std::thread::JoinHandle;
use std::time::Duration;

use iceoryx2::node::Node;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
// The `PortFactory` TRAIT (anonymous import) provides `dynamic_config()` for
// the test-only publisher-count probe below; gated with the probe so the bare
// build carries no unused import.
#[cfg(any(test, feature = "test-helpers"))]
use iceoryx2::prelude::PortFactory as _;
use iceoryx2::prelude::ServiceName;
use iceoryx2::service::builder::publish_subscribe::{
    PublishSubscribeCreateError, PublishSubscribeOpenError, PublishSubscribeOpenOrCreateError,
};
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;

use super::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use super::CerService;
use crate::error::{TransportError, TransportResult};

/// The fixed iceoryx2 service name of the runtime-registration control
/// channel — canonical `/__cerulion/gateway_topics`, under the reserved
/// [`RESERVED_TOPIC_PREFIX`](super::gateway::RESERVED_TOPIC_PREFIX). Deliberately
/// carries NO `/data` suffix (unlike graph topic services `{topic}/data`), so
/// `cerulion topic list` — which enumerates `*/data` services — never surfaces
/// it. Both the writer (`RegistrationChannel`) and the reader
/// (`RegistrationReader`) build their service from this ONE constant, so their
/// static configs cannot drift.
pub const REG_CHANNEL_SERVICE_NAME: &str = "/__cerulion/gateway_topics";

/// `max_publishers` the control service is provisioned with. The
/// unit is PROCESSES, not topics or rmw publishers: each process
/// holds exactly ONE control publisher however many topics it registers —
/// `TransportManager` lazily opens one `RegistrationChannel` per process and
/// serializes its creation, so a multi-process graph consumes one slot per
/// supervisor/worker and a nav2/MoveIt rclcpp process with dozens of rmw
/// publishers consumes ONE (pinned by
/// `racing_first_registrations_open_exactly_one_control_publisher` and
/// `one_control_publisher_per_process_regardless_of_topic_count` in
/// `reg_channel_iox2_test`). 64 slots therefore bound
/// CONCURRENTLY-REGISTERING PROCESSES per box; process #65 is refused loudly
/// (the control publisher create fails → the caller's flood latch warns;
/// local pub/sub is unaffected, that process's topics stay local-only).
/// SLOT RELEASE is a DROP property, not an exit property: dropping the
/// owning `TransportManager` releases the publisher port immediately
/// (pinned by `each_real_os_process_consumes_exactly_one_writer_slot`'s
/// drain arm, whose children drop their managers before exiting) — but a
/// PRODUCTION process retains its manager in the `INSTANCE` `OnceLock`
/// singleton, which is never dropped at process exit, so an rmw/worker
/// process leaves its port registered as stale state until iceoryx2's
/// explicit dead-node cleanup sweep runs WHETHER it exits gracefully or
/// crashes. Registrations can therefore stay refused for that window
/// despite the process being gone.
///
/// DELIBERATELY NOT RAISED as belt-and-braces: iceoryx2 treats the
/// builder's `max_publishers` as create-time provisioning AND an open-time
/// MINIMUM requirement (`verify_number_of_publishers` — opening an existing
/// service provisioned below the opener's constant fails with
/// `DoesNotSupportRequestedAmountOfPublishers`). Both sides build from THIS
/// one constant, so raising it would make every NEW binary refuse to open the
/// service a still-running OLD binary created — and the long-lived reader
/// (`cerulion-netd`'s embedded gateway, spawn-once across CLI upgrades) holds
/// the service in existence, turning routine version skew into a process-#1
/// refusal: strictly worse than the process-#65 one. A safe raise needs the
/// two-phase open(genuine-floor)-then-create(raised) shape (the
/// `open_topic_services` precedent) — deliberately not built until a real
/// deployment approaches 64 concurrently-registering processes.
pub const REG_CHANNEL_MAX_WRITERS: usize = 64;

/// `max_subscribers` the control service is provisioned with — the
/// gateway is the sole reader, plus a little headroom for tests / a second
/// introspecting reader.
pub const REG_CHANNEL_MAX_READERS: usize = 4;

/// `subscriber_max_buffer_size` the control service is provisioned with
/// — the per-pass reader queue. Comfortably exceeds the real robot's ~90-route
/// registration burst so the whole set lands in a single drain pass; a larger
/// burst overflows (`drop_oldest`) but self-heals via periodic re-publication
/// (see the module docs' capacity note).
pub const REG_CHANNEL_QUEUE_DEPTH: usize = 256;

/// How often the background republish thread re-sends the full
/// registration set (the belt that closes the bring-up race regardless of which
/// process started first). 250 ms is far below any human-perceptible bring-up
/// latency and negligible SHM traffic for a ~90-record set.
pub const REG_REPUBLISH_INTERVAL: Duration = Duration::from_millis(250);

/// Total bounded attempts `open_control_service` makes while
/// `open_or_create` keeps surfacing the cross-process first-creation COLLISION
/// (`HangsInCreation`) — 1 initial + 2 re-attempts. Each attempt already
/// carries iceoryx2's own internal wait for a mid-creation peer
/// (`global.creation_timeout`, 500 ms default), so the worst-case wall on the
/// unhealable (dead-creator) path is ~3 × 500 ms + 2 × [`CONTROL_OPEN_RETRY_DELAY`]
/// — bounded and loud, never a spin. A LIVE creator finalizes this tiny
/// service in microseconds, so one re-attempt is already generous; the second
/// covers a creator descheduled through a whole window on a saturated cold
/// boot. See the module docs' collision section.
pub const CONTROL_OPEN_ATTEMPTS: usize = 3;

/// Pause between control-service open re-attempts (see
/// [`CONTROL_OPEN_ATTEMPTS`]).
pub const CONTROL_OPEN_RETRY_DELAY: Duration = Duration::from_millis(50);

/// The record magic (`0xCE`, `0x21`). A frame
/// that does not open with this is not one of ours and is rejected as malformed
/// (defense in depth on a dedicated service that should never carry foreign
/// frames).
const REG_MAGIC: [u8; 2] = [0xCE, 0x21];

/// The record wire-format version. Bumped only on an incompatible
/// record layout change; a mismatched version is rejected as malformed.
const REG_VERSION: u8 = 1;

/// The fixed record header length — `magic(2) + version(1) +
/// schema_hash(8) + topic_len(2)`. The topic bytes follow.
pub const REG_HEADER_LEN: usize = 2 + 1 + 8 + 2;

/// The maximum registrable topic length (bytes). Bounds the record
/// size (and thus the control publisher's loaned slot). Far above any realistic
/// canonical topic name.
pub const MAX_REG_TOPIC_LEN: usize = 512;

/// The maximum record length — the control publisher's
/// `initial_max_slice_len`. A record is `REG_HEADER_LEN + topic.len()` bytes.
pub const MAX_REG_RECORD_LEN: usize = REG_HEADER_LEN + MAX_REG_TOPIC_LEN;

/// A decoded registration record — a canonical egress topic plus the
/// wire `schema_hash` the producing process advertised for it. The hash is
/// carried for the gateway's topic→schema table (future probe/diagnostics);
/// egress forwards frames verbatim, so it is stored + exposed, never used to
/// gate delivery.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegRecord {
    /// Canonical (leading-slash) egress topic the producing process registered.
    pub topic: String,
    /// The wire `schema_hash` (recipe-3) the producer advertised for the topic.
    pub schema_hash: u64,
}

/// Why a registration record failed to encode or decode. Every variant
/// is a distinct, testable reason (oracle-vector coverage in the unit tests);
/// the `RegistrationReader` drain maps ANY decode error to a counted +
/// warn-once + continue (a poisoned/hostile record must never wedge the drain).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegRecordError {
    /// The topic is empty (no topic name to register).
    EmptyTopic,
    /// The topic exceeds [`MAX_REG_TOPIC_LEN`] bytes.
    TopicTooLong { len: usize },
    /// The frame is shorter than [`REG_HEADER_LEN`] (truncated header).
    Truncated { len: usize },
    /// The leading magic bytes are not `REG_MAGIC` (not a Cerulion record).
    BadMagic,
    /// The version byte is not `REG_VERSION`.
    BadVersion { found: u8 },
    /// The declared topic length disagrees with the frame length.
    LengthMismatch {
        declared: usize,
        actual_payload: usize,
    },
    /// The topic bytes are not valid UTF-8.
    InvalidUtf8,
}

impl std::fmt::Display for RegRecordError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RegRecordError::EmptyTopic => write!(f, "empty topic name"),
            RegRecordError::TopicTooLong { len } => {
                write!(
                    f,
                    "topic length {len} exceeds the {MAX_REG_TOPIC_LEN}-byte limit"
                )
            }
            RegRecordError::Truncated { len } => {
                write!(
                    f,
                    "record is {len} bytes, shorter than the {REG_HEADER_LEN}-byte header"
                )
            }
            RegRecordError::BadMagic => {
                write!(f, "bad record magic (not a Cerulion registration frame)")
            }
            RegRecordError::BadVersion { found } => {
                write!(
                    f,
                    "unsupported record version {found} (expected {REG_VERSION})"
                )
            }
            RegRecordError::LengthMismatch {
                declared,
                actual_payload,
            } => write!(
                f,
                "record declares a {declared}-byte topic but carries {actual_payload} topic bytes"
            ),
            RegRecordError::InvalidUtf8 => write!(f, "topic bytes are not valid UTF-8"),
        }
    }
}

/// Encode a `(topic, schema_hash)` registration record for the control
/// wire — `magic(2) ++ version(1) ++ schema_hash(8, LE) ++ topic_len(2, LE) ++
/// topic_bytes`. Pure (no transport); the exact inverse of [`decode_record`].
///
/// # Errors
///
/// - [`RegRecordError::EmptyTopic`] for an empty topic.
/// - [`RegRecordError::TopicTooLong`] if the topic exceeds [`MAX_REG_TOPIC_LEN`].
pub fn encode_record(topic: &str, schema_hash: u64) -> Result<Vec<u8>, RegRecordError> {
    if topic.is_empty() {
        return Err(RegRecordError::EmptyTopic);
    }
    if topic.len() > MAX_REG_TOPIC_LEN {
        return Err(RegRecordError::TopicTooLong { len: topic.len() });
    }
    let mut buf = Vec::with_capacity(REG_HEADER_LEN + topic.len());
    buf.extend_from_slice(&REG_MAGIC);
    buf.push(REG_VERSION);
    buf.extend_from_slice(&schema_hash.to_le_bytes());
    buf.extend_from_slice(&(topic.len() as u16).to_le_bytes());
    buf.extend_from_slice(topic.as_bytes());
    Ok(buf)
}

/// Decode a control-wire registration record — the exact inverse of
/// [`encode_record`]. Pure (no transport). Validates the magic, version, the
/// declared-vs-actual topic length, and UTF-8 so a truncated / hostile / foreign
/// frame is rejected with a precise reason rather than silently misread.
///
/// # Errors
///
/// Any of these [`RegRecordError`] variants (each on its own condition — see the
/// variant docs):
///
/// - [`RegRecordError::Truncated`] — fewer bytes than [`REG_HEADER_LEN`].
/// - [`RegRecordError::BadMagic`] — the leading bytes are not the record magic.
/// - [`RegRecordError::BadVersion`] — the version byte is not `REG_VERSION`.
/// - [`RegRecordError::TopicTooLong`] — the frame DECLARES a topic length above
///   [`MAX_REG_TOPIC_LEN`] (the decode-side analogue of the encode-side check).
/// - [`RegRecordError::LengthMismatch`] — the declared topic length disagrees
///   with the actual payload length.
/// - [`RegRecordError::InvalidUtf8`] — the topic bytes are not valid UTF-8.
/// - [`RegRecordError::EmptyTopic`] — the topic bytes decode to an empty string.
pub fn decode_record(bytes: &[u8]) -> Result<RegRecord, RegRecordError> {
    if bytes.len() < REG_HEADER_LEN {
        return Err(RegRecordError::Truncated { len: bytes.len() });
    }
    if bytes[0..2] != REG_MAGIC {
        return Err(RegRecordError::BadMagic);
    }
    if bytes[2] != REG_VERSION {
        return Err(RegRecordError::BadVersion { found: bytes[2] });
    }
    // Fixed-offset reads (the length checks above make these infallible).
    let schema_hash = u64::from_le_bytes(bytes[3..11].try_into().expect("8 bytes"));
    let declared = u16::from_le_bytes(bytes[11..13].try_into().expect("2 bytes")) as usize;
    if declared > MAX_REG_TOPIC_LEN {
        return Err(RegRecordError::TopicTooLong { len: declared });
    }
    let actual_payload = bytes.len() - REG_HEADER_LEN;
    if declared != actual_payload {
        return Err(RegRecordError::LengthMismatch {
            declared,
            actual_payload,
        });
    }
    let topic =
        std::str::from_utf8(&bytes[REG_HEADER_LEN..]).map_err(|_| RegRecordError::InvalidUtf8)?;
    if topic.is_empty() {
        return Err(RegRecordError::EmptyTopic);
    }
    Ok(RegRecord {
        topic: topic.to_string(),
        schema_hash,
    })
}

/// Open the ONE control pub/sub service (`open_or_create`, so whichever
/// side — worker publisher or gateway subscriber — arrives first CREATES it and
/// the other OPENS it). Both sides call THIS helper, so the static config
/// (buffer / writer slots / reader slots; NO event service, NO history) is
/// identical and `open_or_create` is always compatible.
///
/// The one failure `open_or_create` cannot absorb on its own is the
/// cross-process first-creation COLLISION — `HangsInCreation`, surfaced when
/// this opener observed the OTHER side's creation in progress and iceoryx2's
/// internal `creation_timeout` wait expired. That arm (and ONLY that arm — see
/// [`is_creation_collision`]) is re-attempted up to [`CONTROL_OPEN_ATTEMPTS`]
/// times, [`CONTROL_OPEN_RETRY_DELAY`] apart, then classified into a loud,
/// actionable hard error naming the service, both roles, and the
/// stale-artifact remedy. Every other failure surfaces immediately.
fn open_control_service(
    node: &Node<CerService>,
) -> TransportResult<PortFactory<CerService, [u8], ()>> {
    let name: ServiceName =
        REG_CHANNEL_SERVICE_NAME
            .try_into()
            .map_err(|e| TransportError::Internal {
                reason: format!(
                    "registration channel: invalid control service name \
                 '{REG_CHANNEL_SERVICE_NAME}': {e:?}"
                ),
            })?;
    let mut attempt = 0usize;
    loop {
        attempt += 1;
        // The injection seam substitutes a forced collision for the
        // REAL open's result (test builds only): the classification, retry
        // and error-formatting code below run UNCHANGED on the genuine
        // `HangsInCreation` value — only its ORIGIN is synthetic, because the
        // real window needs a peer stalled > 500 ms mid-create, which no
        // in-process test can stage deterministically.
        let opened = match maybe_forced_creation_collision(node) {
            Some(forced) => Err(forced),
            None => try_open_control_service(node, &name),
        };
        match opened {
            Ok(factory) => {
                if attempt > 1 {
                    tracing::warn!(
                        service = REG_CHANNEL_SERVICE_NAME,
                        attempts = attempt,
                        "registration channel: the control service's first-creation \
                         collision healed on a re-attempt (a peer was mid-creation)"
                    );
                }
                return Ok(factory);
            }
            Err(e) if is_creation_collision(&e) => {
                if attempt < CONTROL_OPEN_ATTEMPTS {
                    tracing::warn!(
                        service = REG_CHANNEL_SERVICE_NAME,
                        attempt = attempt,
                        error = ?e,
                        "registration channel: the control service hangs in creation \
                         (a peer process is mid-creation, or died there) — \
                         re-attempting"
                    );
                    std::thread::sleep(CONTROL_OPEN_RETRY_DELAY);
                    continue;
                }
                return Err(TransportError::Internal {
                    reason: format!(
                        "registration channel: the control service \
                         '{REG_CHANNEL_SERVICE_NAME}' still HANGS IN CREATION after \
                         {CONTROL_OPEN_ATTEMPTS} open attempts ({e:?}). Both sides open \
                         this service open_or_create at boot — the gateway process's \
                         registration READER and each worker process's registration \
                         WRITER — so a cold-machine boot can collide with the other \
                         side's first creation; a LIVE creator finalizes within \
                         iceoryx2's creation window (global.creation_timeout, 500 ms \
                         default) and a re-attempt heals it, which was tried. \
                         Exhausting the bound means a process DIED mid-creation and \
                         left stale service artifacts no retry can heal: remove the \
                         stale iceoryx2 shared-memory artifacts (or reboot), then \
                         restart the gateway and workers"
                    ),
                });
            }
            Err(e) => {
                return Err(TransportError::Internal {
                    reason: format!(
                        "registration channel: could not open the control service \
                         '{REG_CHANNEL_SERVICE_NAME}': {e:?}"
                    ),
                });
            }
        }
    }
}

/// One REAL `open_or_create` attempt on the control service — no retry, no
/// error mapping. The single point the loop above calls, so the
/// static config stays defined exactly once.
fn try_open_control_service(
    node: &Node<CerService>,
    name: &ServiceName,
) -> Result<PortFactory<CerService, [u8], ()>, PublishSubscribeOpenOrCreateError> {
    node.service_builder(name)
        .publish_subscribe::<[u8]>()
        .subscriber_max_buffer_size(REG_CHANNEL_QUEUE_DEPTH)
        .max_subscribers(REG_CHANNEL_MAX_READERS)
        .max_publishers(REG_CHANNEL_MAX_WRITERS)
        .open_or_create()
}

/// Is `e` the cross-process first-creation COLLISION — iceoryx2's
/// `HangsInCreation`, on either the open or the create side of
/// `open_or_create`? Exactly these two arms buy a bounded re-attempt; every
/// other failure (type skew, corruption, port-cap mismatch, `SystemInFlux` —
/// iceoryx2's OWN retry-exhausted verdict, which re-retrying here would
/// silently double) surfaces immediately. PURE — oracle-tested below with an
/// exhaustive-match sweep, so an iceoryx2 upgrade that adds a variant fails
/// to compile in the test rather than silently changing the retry set.
fn is_creation_collision(e: &PublishSubscribeOpenOrCreateError) -> bool {
    matches!(
        e,
        PublishSubscribeOpenOrCreateError::PublishSubscribeOpenError(
            PublishSubscribeOpenError::HangsInCreation
        ) | PublishSubscribeOpenOrCreateError::PublishSubscribeCreateError(
            PublishSubscribeCreateError::HangsInCreation
        )
    )
}

/// Injection seam (test builds only): force the next `count` control
/// open attempts BY THE NAMED NODE to observe a `HangsInCreation` collision.
/// A PER-NODE MAP, not a single slot: the static is process-global while the
/// test binaries that use it run their tests in PARALLEL, and two tests each
/// arming for their OWN manager must neither clobber nor consume each
/// other's arming (a single `Option` slot did exactly that — the second
/// test's arm REPLACED the first's mid-ladder, silently healing its forced
/// exhaustion). Every test manager carries a unique `node_name`, so the key
/// confines each arming to the test that set it.
#[cfg(any(test, feature = "test-helpers"))]
static FORCE_CONTROL_OPEN_COLLISIONS: Mutex<BTreeMap<String, usize>> = Mutex::new(BTreeMap::new());

/// Arm the seam: the next `count` open attempts by the node named
/// `node_name` fail as `HangsInCreation`. See
/// `FORCE_CONTROL_OPEN_COLLISIONS`.
#[cfg(any(test, feature = "test-helpers"))]
pub fn force_control_open_collisions_for_test(node_name: &str, count: usize) {
    FORCE_CONTROL_OPEN_COLLISIONS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .insert(node_name.to_string(), count);
}

/// The number of forced collisions still armed for `node_name` (`0` once the
/// retry ladder has consumed them all) — the oracle that the retry path
/// really ran the forced attempts.
#[cfg(any(test, feature = "test-helpers"))]
pub fn forced_control_open_collisions_remaining_for_test(node_name: &str) -> usize {
    FORCE_CONTROL_OPEN_COLLISIONS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .get(node_name)
        .copied()
        .unwrap_or(0)
}

/// Consume one forced collision if the seam is armed for THIS node. See
/// [`FORCE_CONTROL_OPEN_COLLISIONS`].
#[cfg(any(test, feature = "test-helpers"))]
fn maybe_forced_creation_collision(
    node: &Node<CerService>,
) -> Option<PublishSubscribeOpenOrCreateError> {
    let mut armed = FORCE_CONTROL_OPEN_COLLISIONS
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    match armed.get_mut(node.name().as_str()) {
        Some(remaining) if *remaining > 0 => {
            *remaining -= 1;
            Some(
                PublishSubscribeOpenOrCreateError::PublishSubscribeOpenError(
                    PublishSubscribeOpenError::HangsInCreation,
                ),
            )
        }
        _ => None,
    }
}

/// Bare-build twin of the seam consult: never fires.
#[cfg(not(any(test, feature = "test-helpers")))]
#[inline]
fn maybe_forced_creation_collision(
    _node: &Node<CerService>,
) -> Option<PublishSubscribeOpenOrCreateError> {
    None
}

/// The shared, thread-safe core of a [`RegistrationChannel`] — the control
/// publisher (Send+Sync via `ipc_threadsafe`, so it is shared by `Arc` between
/// the register callsite AND the background republish thread with no extra
/// mutex), the authoritative unbounded registration map, and the send-failure
/// flood latch. `BTreeMap` gives deterministic (sorted) republish + snapshot
/// order for reproducible tests.
struct RegChannelInner {
    publisher: Publisher<CerService, [u8], ()>,
    /// Canonical topic → advertised `schema_hash`. UNBOUNDED (no eviction — the
    /// capacity story lives here): every registration is retained and re-sent.
    records: Mutex<BTreeMap<String, u64>>,
    /// Monotonic republish counter — rotates the per-republish send order so a
    /// burst LARGER than the reader's queue ([`REG_CHANNEL_QUEUE_DEPTH`]) cannot
    /// perpetually starve the same (lowest-sorted) topics under `drop_oldest`
    /// overflow: each republish shifts the send order by one queue's worth, so
    /// every topic lands within ⌈N / queue_depth⌉ republishes. Irrelevant for a
    /// burst ≤ the queue (the whole set fits in one pass regardless of order).
    republish_count: AtomicU64,
    /// First send failure of a run logs `warn!`, repeats `debug!` (the
    /// `record_forward_failure` flood-suppression discipline); a success re-arms.
    send_warned: AtomicBool,
    /// Principle #3: UNCONDITIONAL count of pump republish ticks
    /// whose body PANICKED and was contained by the pump's per-tick
    /// `catch_unwind` — bumped on every contained tick panic, never reset. A
    /// nonzero value with [`RegistrationChannel::registration_pump_active`]
    /// still `true` is the "belt survived a panic" signature the pins assert.
    pump_tick_panics: AtomicU64,
}

impl RegChannelInner {
    /// Insert (or update) a registration. Returns `true` iff the canonical topic
    /// was not already present (a genuinely NEW registration). LAST-WRITE-WINS on
    /// the hash: re-registering an existing topic with a DIFFERENT
    /// `schema_hash` overwrites the stored value and returns `false` (not new) — so
    /// the authoritative set always carries the most recently advertised hash, and
    /// the gateway's topic→hash table converges to it via the next drain (whose Ok
    /// arm also last-write-wins).
    fn insert(&self, canonical: &str, schema_hash: u64) -> bool {
        let mut records = self.records.lock().unwrap_or_else(|e| e.into_inner());
        records.insert(canonical.to_string(), schema_hash).is_none()
    }

    /// Encode + loan + send ONE record on the control publisher. Returns whether
    /// the send succeeded. A send failure is flood-latched (warn once, then
    /// debug) and non-fatal — the periodic republish re-attempts it. `&self`:
    /// the publisher's loan/send take `&self` and the latch is atomic, so both
    /// the callsite and the republish thread share it lock-free.
    fn send_record(&self, topic: &str, schema_hash: u64, site: SendSite) -> bool {
        let bytes = match encode_record(topic, schema_hash) {
            Ok(b) => b,
            Err(e) => {
                // GENUINELY unreachable from the public API: the only
                // production writer, `register_dynamic_egress_topic`, now rejects
                // BOTH encode-failure cases up front — an empty name and a
                // canonical name longer than `MAX_REG_TOPIC_LEN` — before the
                // record ever enters the authoritative map, and every topic in the
                // map is a canonical (non-empty, leading-slash) name at or below
                // that bound. So `send_record` only ever sees encodable topics.
                // We still handle the `Err` (the encode is fallible — never send
                // garbage) and flag any regression that re-opens this path.
                debug_assert!(
                    false,
                    "send_record hit an unencodable topic ({e}) — the public \
                     API length/empty guards were bypassed (a map invariant broke)"
                );
                tracing::warn!(
                    topic = %topic,
                    error = %e,
                    "registration channel: refusing to send an unencodable record"
                );
                return false;
            }
        };
        match self.publisher.loan_slice_uninit(bytes.len()) {
            Ok(sample) => {
                let sample = sample.write_from_slice(&bytes);
                match sample.send() {
                    Ok(_) => {
                        // Fault-injection seam (test builds only): fires
                        // HERE — after the REAL loan + iceoryx2 send have run
                        // and returned — so a pin driving it proves the
                        // actual send path is lock-free and poison-tolerant,
                        // not merely that some earlier point is. Register
                        // path only (`SendSite::Register`), topic-keyed.
                        #[cfg(any(test, feature = "test-helpers"))]
                        maybe_panic_after_register_send(topic, site);
                        // Twin seam: fires on the PUMP's republish
                        // of exactly the armed topic (site-keyed the other
                        // way round), driving the per-tick containment pin.
                        #[cfg(any(test, feature = "test-helpers"))]
                        maybe_panic_after_pump_send(topic, site);
                        #[cfg(not(any(test, feature = "test-helpers")))]
                        let _ = site;
                        self.record_send_success();
                        true
                    }
                    Err(e) => {
                        self.record_send_failure(topic, &format!("{e}"));
                        false
                    }
                }
            }
            Err(e) => {
                self.record_send_failure(topic, &format!("{e}"));
                false
            }
        }
    }

    /// Re-send EVERY retained registration (the background thread body + the
    /// deterministic test seam). Returns the number of records successfully sent.
    /// Snapshots the map under the lock, then sends OUTSIDE the lock (a slow
    /// send must not block a concurrent `register`). The send order is ROTATED by
    /// one queue's worth per republish (see `republish_count`) so a burst larger
    /// than the reader's queue is not perpetually starved by `drop_oldest`.
    fn republish(&self) -> usize {
        let mut snapshot: Vec<(String, u64)> = {
            let records = self.records.lock().unwrap_or_else(|e| e.into_inner());
            records.iter().map(|(t, h)| (t.clone(), *h)).collect()
        };
        // Rotate the send order so successive republishes evict a DIFFERENT
        // window under `drop_oldest` overflow — every topic lands within
        // ⌈len / queue_depth⌉ republishes even when len > the reader queue.
        if snapshot.len() > REG_CHANNEL_QUEUE_DEPTH {
            let round = self.republish_count.fetch_add(1, Ordering::Relaxed) as usize;
            let shift = round
                .wrapping_mul(REG_CHANNEL_QUEUE_DEPTH)
                .checked_rem(snapshot.len())
                .unwrap_or(0);
            snapshot.rotate_left(shift);
        }
        let mut sent = 0usize;
        for (topic, hash) in snapshot {
            if self.send_record(&topic, hash, SendSite::Republish) {
                sent += 1;
            }
        }
        sent
    }

    fn record_send_failure(&self, topic: &str, reason: &str) {
        if self.send_warned.swap(true, Ordering::Relaxed) {
            tracing::debug!(
                topic = %topic,
                reason = %reason,
                "registration channel: control send failed (repeat — see the first warning)"
            );
        } else {
            tracing::warn!(
                topic = %topic,
                reason = %reason,
                "registration channel: control send failed — this registration will retry \
                 on the next republish (repeats log at debug)"
            );
        }
    }

    fn record_send_success(&self) {
        if self.send_warned.swap(false, Ordering::Relaxed) {
            tracing::info!("registration channel: control send recovered — republishing again");
        }
    }

    #[cfg(any(test, feature = "test-helpers"))]
    fn record_count(&self) -> usize {
        self.records.lock().unwrap_or_else(|e| e.into_inner()).len()
    }

    /// The `schema_hash` the authoritative map currently holds for `canonical`, or
    /// `None` if it is not registered. Test seam for the last-write-wins pin.
    #[cfg(any(test, feature = "test-helpers"))]
    fn record_hash(&self, canonical: &str) -> Option<u64> {
        self.records
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(canonical)
            .copied()
    }
}

/// Which caller is sending a record — the register path (the one live send
/// per registration) or the pump thread's periodic republish. Threaded into
/// `send_record` so the test seam below can be confined to the register
/// path: a republish of the SAME topic must never take a panic armed for the
/// registration that is racing it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum SendSite {
    Register,
    Republish,
}

/// Fault-injection seam (test builds only): when armed for a TOPIC, the
/// [`RegistrationChannel::register`] call for exactly that canonical topic
/// panics INSIDE `send_record` — after the REAL loan and iceoryx2 send have
/// run and returned, i.e. as deep in the registration's one panic-capable
/// step as a panic can occur — modelling a failure surfacing from the send
/// path. Firing there (rather than before the send is called) is what makes
/// the pins prove the ACTUAL path holds no lock and poisons nothing, not
/// merely that some earlier point does. Fires once, then disarms itself.
/// TOPIC-KEYED because the static is process-global while the test binaries
/// that use it run their tests in PARALLEL: a plain fire-once flag could be
/// stolen by a CONCURRENT test's register call (panicking an innocent test
/// while the arming one misses its injection — an observed flake); per-test
/// topics are unique, so the key confines the seam to the register that
/// armed it. Confined to [`SendSite::Register`] — the pump thread's
/// republish of the same topic never consults it.
#[cfg(any(test, feature = "test-helpers"))]
static PANIC_ON_REGISTER_SEND_OF: Mutex<Option<String>> = Mutex::new(None);

/// The seam's trigger — see [`PANIC_ON_REGISTER_SEND_OF`]. A no-op unless
/// armed for exactly `topic` AND called from the register path.
#[cfg(any(test, feature = "test-helpers"))]
fn maybe_panic_after_register_send(topic: &str, site: SendSite) {
    if site != SendSite::Register {
        return;
    }
    let mut armed = PANIC_ON_REGISTER_SEND_OF
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    if armed.as_deref() == Some(topic) {
        *armed = None;
        drop(armed);
        panic!("{REGISTER_SEND_PANIC_MSG}");
    }
}

/// The seam's panic payload — a test asserts on it so a contained failure is
/// attributable to the injection and not to some unrelated panic.
#[cfg(any(test, feature = "test-helpers"))]
pub const REGISTER_SEND_PANIC_MSG: &str = "test-seam: injected panic at the registration send";

/// Arm the seam for exactly one `register` call on canonical topic `topic`.
#[cfg(any(test, feature = "test-helpers"))]
pub fn panic_on_next_register_send_for_test(topic: &str) {
    *PANIC_ON_REGISTER_SEND_OF
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = Some(topic.to_string());
}

/// The [`PANIC_ON_REGISTER_SEND_OF`] twin for the PUMP side — armed
/// for a TOPIC, the next `SendSite::Republish` send of exactly that canonical
/// topic panics after the REAL loan + send have run, modelling a failure
/// surfacing from the pump's send path (as deep in the tick as a panic can
/// occur, so the containment pin proves the ACTUAL tick body is caught, not
/// merely some earlier point). TOPIC-KEYED for the same parallel-binary
/// confinement reason; fires once, then disarms. The inline test-seam
/// `republish()` also reaches `SendSite::Republish`, so a test that arms this
/// must not drive the inline republish of the armed topic in the window.
/// PER-TOPIC map, not a single slot: the test binaries run in PARALLEL and
/// the hook-suppression tests gave the seam a second in-binary user — with
/// one slot, whichever test armed last CLOBBERED the other's arming (observed:
/// the containment test's `(pk1, 1)` replacing a hook-suppression storm's
/// `(pk3, 3)` mid-regime, timing out its bounded wait). Same shape and reason as
/// [`FORCE_CONTROL_OPEN_COLLISIONS`].
#[cfg(any(test, feature = "test-helpers"))]
static PANIC_ON_PUMP_SEND_OF: Mutex<BTreeMap<String, usize>> = Mutex::new(BTreeMap::new());

/// The pump-send seam's panic payload — asserted on for attributability.
#[cfg(any(test, feature = "test-helpers"))]
pub const PUMP_SEND_PANIC_MSG: &str = "test-seam: injected panic at a pump republish send";

/// Arm the pump-send seam for canonical topic `topic` (one send).
#[cfg(any(test, feature = "test-helpers"))]
pub fn panic_on_next_pump_send_for_test(topic: &str) {
    panic_on_next_pump_sends_for_test(topic, 1);
}

/// Arm the pump-send seam for the next `count` republish sends of `topic`.
/// One armed send fires per pump TICK (the panic aborts the tick, and the
/// belt re-attempts next interval), so `count` spans `count` ticks — a
/// PERSISTENT panic regime, which is what the hook-suppression storm
/// arms need (one contained panic cannot distinguish "the hook printed once"
/// from "the hook prints per tick, unbounded").
///
/// A `count` of ZERO is a DISARM — it removes any live arming for `topic`
/// and no send fires (inserting the zero as an arming would make the trigger's
/// `Some(..)` arm fire once before removing the entry, so 0 would behave as 1,
/// against this doc's own "the next `count` sends" contract).
#[cfg(any(test, feature = "test-helpers"))]
pub fn panic_on_next_pump_sends_for_test(topic: &str, count: usize) {
    let mut armed = PANIC_ON_PUMP_SEND_OF
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    if count == 0 {
        armed.remove(topic);
    } else {
        armed.insert(topic.to_string(), count);
    }
}

/// The pump-send seam's trigger — see [`PANIC_ON_PUMP_SEND_OF`]. A no-op
/// unless armed for exactly `topic` AND called from the republish path.
/// Decrements the topic's armed count; disarms that topic at zero.
#[cfg(any(test, feature = "test-helpers"))]
fn maybe_panic_after_pump_send(topic: &str, site: SendSite) {
    if site != SendSite::Republish {
        return;
    }
    let mut armed = PANIC_ON_PUMP_SEND_OF
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let fire = match armed.get_mut(topic) {
        Some(remaining) => {
            *remaining = remaining.saturating_sub(1);
            if *remaining == 0 {
                armed.remove(topic);
            }
            true
        }
        None => false,
    };
    drop(armed);
    if fire {
        panic!("{PUMP_SEND_PANIC_MSG}");
    }
}

/// Test-observable: the LIVE publisher count on the control service — the
/// oracle for "exactly one control publisher is ever opened per process"
/// (creation is serialized by `TransportManager::dynamic_egress_channel`).
/// Opening the port FACTORY adds no publisher, so the probe is inert.
#[cfg(any(test, feature = "test-helpers"))]
pub(crate) fn control_publisher_count_for_test(node: &Node<CerService>) -> TransportResult<usize> {
    Ok(open_control_service(node)?
        .dynamic_config()
        .number_of_publishers())
}

/// Test-only keepalive: holds the control SERVICE's port factory open for the
/// guard's lifetime. iceoryx2 removes a service once its last port AND factory
/// drop, so a count probe alone cannot keep the service in existence — and a
/// test that orchestrates concurrent openers against the very first creation
/// would otherwise race iceoryx2's own `HangsInCreation` window (a harness
/// artifact, not the property under test). Holding this guard pins the
/// service — with ZERO publishers — for the test's whole body.
#[cfg(any(test, feature = "test-helpers"))]
pub struct ControlServiceKeepalive {
    _factory: PortFactory<CerService, [u8], ()>,
}

/// Open (or create) the control service and hand back a [`ControlServiceKeepalive`].
#[cfg(any(test, feature = "test-helpers"))]
pub(crate) fn control_service_keepalive(
    node: &Node<CerService>,
) -> TransportResult<ControlServiceKeepalive> {
    Ok(ControlServiceKeepalive {
        _factory: open_control_service(node)?,
    })
}

/// A joined-on-drop handle for the background republish thread.
struct PumpHandle {
    exit: Arc<AtomicBool>,
    /// Cleared by the thread body's [`PumpLiveGuard`] on ANY thread
    /// exit (normal return or an escaped panic), so
    /// [`RegistrationChannel::registration_pump_active`] reads THREAD
    /// liveness, never the handle's mere existence.
    live: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

/// RAII on the pump THREAD BODY — clears the shared live flag on
/// ANY exit path (a normal teardown return, or a panic that escaped the
/// per-tick containment) and reports the escaped-panic case loudly. Held as
/// the first value in the thread closure, so nothing can exit around it (the
/// repo's RAII-guards-for-process-state discipline).
struct PumpLiveGuard {
    live: Arc<AtomicBool>,
}

impl Drop for PumpLiveGuard {
    fn drop(&mut self) {
        self.live.store(false, Ordering::Relaxed);
        if std::thread::panicking() {
            // A panic ESCAPED the per-tick containment — it came from the
            // pump scaffolding itself, not from a republish tick. The belt is
            // DOWN and STAYS down (see `ensure_pump_running` for why there is
            // no respawn).
            tracing::error!(
                "registration channel: the republish pump thread DIED (a panic escaped \
                 the per-tick containment — see the panic hook's own message for the \
                 payload). The periodic bring-up-race belt is DOWN and stays down; each \
                 register still sends its record immediately, but a gateway that boots \
                 later may miss pre-boot registrations. registration_pump_active() now \
                 reads false"
            );
        }
    }
}

impl Drop for PumpHandle {
    fn drop(&mut self) {
        self.exit.store(true, Ordering::Relaxed);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

/// The WRITER side of the registration channel — one per network-free
/// worker process, held lazily by its [`TransportManager`]. Owns the control
/// publisher + the authoritative registration set + the background republish
/// thread. See the module docs for the periodic-republication design.
///
/// `pump` is declared BEFORE `inner` so it drops FIRST: joining the republish
/// thread before the (channel's) `inner` Arc is released. (The thread holds its
/// own `inner` clone too, so the ordering is belt-and-suspenders — the thread
/// can never outlive the publisher it borrows.)
pub(crate) struct RegistrationChannel {
    pump: Mutex<Option<PumpHandle>>,
    inner: Arc<RegChannelInner>,
}

impl RegistrationChannel {
    /// Open the control service on `node` and build a fresh channel (no records,
    /// no pump thread yet — the pump starts lazily on the first `register`).
    pub(crate) fn open(node: &Node<CerService>) -> TransportResult<Self> {
        // BACKSTOP (see the hook-suppression section): in production
        // the manager's own constructor already spent the `Once`, so this is a
        // no-op there; a bare-channel test pays the one install here, before
        // any pump can panic.
        install_contained_panic_hook_suppressor();
        let factory = open_control_service(node)?;
        let publisher = factory
            .publisher_builder()
            .initial_max_slice_len(MAX_REG_RECORD_LEN)
            .create()
            .map_err(|e| TransportError::Internal {
                reason: format!(
                    "registration channel: could not create the control publisher \
                     on '{REG_CHANNEL_SERVICE_NAME}': {e:?}"
                ),
            })?;
        Ok(Self {
            pump: Mutex::new(None),
            inner: Arc::new(RegChannelInner {
                publisher,
                records: Mutex::new(BTreeMap::new()),
                republish_count: AtomicU64::new(0),
                send_warned: AtomicBool::new(false),
                pump_tick_panics: AtomicU64::new(0),
            }),
        })
    }

    /// Register `canonical` (a canonical, leading-slash topic) for network
    /// egress: ensure the background republish thread is running (the belt
    /// that closes the bring-up race), record the topic in the authoritative
    /// set, then send it once immediately (the gateway-already-up fast path).
    /// Idempotent — a repeat call returns `false` and re-sends the (unchanged)
    /// record. Returns whether the topic was genuinely NEW.
    ///
    /// ORDER IS LOAD-BEARING. The send is the only
    /// panic-capable step (an iceoryx2 loan + send), so it runs LAST: by then
    /// the record is already retained AND the pump is already live to
    /// republish it, so a panic there strands nothing — the other order
    /// (retain, send, THEN arm the pump) could leave a retained record with
    /// no pump, never republished to a gateway that boots later. No lock is
    /// held across the send either (`insert` releases the records lock before
    /// returning; `ensure_pump_running` releases the pump lock), and the
    /// caller holds none, so a panic poisons nothing.
    pub(crate) fn register(&self, canonical: &str, schema_hash: u64) -> bool {
        self.ensure_pump_running();
        let newly = self.inner.insert(canonical, schema_hash);
        // Immediate live send (delivered iff the gateway is already connected).
        self.inner
            .send_record(canonical, schema_hash, SendSite::Register);
        newly
    }

    /// Re-send the entire registration set inline (no thread). The deterministic
    /// test seam — production drives the SAME [`RegChannelInner::republish`] body
    /// from the background thread ([`pump_loop`]).
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn republish(&self) -> usize {
        self.inner.republish()
    }

    /// Send arbitrary raw bytes on the control publisher WITHOUT recording them —
    /// the seam tests use to craft hostile / malformed frames on the wire (a
    /// well-behaved worker never does this). Returns whether the send succeeded.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn send_raw_for_test(&self, bytes: &[u8]) -> bool {
        match self.inner.publisher.loan_slice_uninit(bytes.len()) {
            Ok(sample) => sample.write_from_slice(bytes).send().is_ok(),
            Err(_) => false,
        }
    }

    /// The number of distinct topics in the authoritative registration set
    /// (Principle #3: observable state). Test seam.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn record_count(&self) -> usize {
        self.inner.record_count()
    }

    /// The `schema_hash` the authoritative set holds for `canonical` (the
    /// last-write-wins pin — a re-register with a new hash OVERWRITES). Test seam.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn record_hash(&self, canonical: &str) -> Option<u64> {
        self.inner.record_hash(canonical)
    }

    /// Test seam: cumulative pump republish ticks whose body panicked
    /// and was CONTAINED (the belt survived). Backed by the unconditional
    /// `RegChannelInner::pump_tick_panics` counter.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn pump_tick_panic_count(&self) -> u64 {
        self.inner.pump_tick_panics.load(Ordering::Relaxed)
    }

    /// Principle #3: whether the background republish thread is LIVE, i.e.
    /// spawned successfully on the first [`Self::register`]. `false` before the
    /// first registration (the pump is lazily started), `false` if the spawn
    /// FAILED (the degraded state the spawn-failure warn describes), AND `false`
    /// once the pump THREAD has EXITED: liveness is a flag the
    /// thread body clears by RAII on ANY exit — a panic that escaped the
    /// per-tick containment included — never the handle's mere existence,
    /// which survives the thread it once named. The writer-side symmetry to
    /// the reader-side
    /// [`GatewayRuntime::runtime_registration_active`](super::gateway::GatewayRuntime::runtime_registration_active),
    /// surfaced publicly via
    /// [`TransportManager::registration_pump_active`](super::TransportManager::registration_pump_active).
    pub(crate) fn registration_pump_active(&self) -> bool {
        self.pump
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .is_some_and(|p| p.live.load(Ordering::Relaxed))
    }

    /// Start the background republish thread exactly once (latched-idempotent).
    /// A spawn failure is non-fatal + loud: every [`Self::register`] STILL
    /// sends its record immediately, so a gateway that is ALREADY up receives each
    /// registration as it happens. What is lost is only the PERIODIC belt — with no
    /// pump thread there is no automagic re-send, so a gateway that boots AFTER a
    /// registration will not receive that (pre-boot) record until the next
    /// `register` call happens to re-send it. Observable via
    /// [`Self::registration_pump_active`].
    ///
    /// STAY-DOWN: a pump whose THREAD died (a panic that escaped the
    /// per-tick containment — see [`pump_loop`]) is NOT respawned here — the
    /// slot still holds its handle, so this early-returns. Deliberate: the
    /// containment already absorbs every failure the belt can heal from
    /// (republish-tick panics retry next interval), so an ESCAPED panic is
    /// structural — the pump scaffolding itself — and a respawn would re-die
    /// identically, converting the true "belt is down" signal into a silent
    /// spawn-die loop paced by the caller's register rate. The death is loud
    /// once ([`PumpLiveGuard`]) and observable forever
    /// ([`Self::registration_pump_active`] reads `false`); each `register`
    /// still live-sends its record.
    fn ensure_pump_running(&self) {
        let mut pump = self.pump.lock().unwrap_or_else(|e| e.into_inner());
        if pump.is_some() {
            return;
        }
        let exit = Arc::new(AtomicBool::new(false));
        let exit_thread = Arc::clone(&exit);
        let live = Arc::new(AtomicBool::new(true));
        let live_thread = Arc::clone(&live);
        let inner = Arc::clone(&self.inner);
        let spawned = std::thread::Builder::new()
            .name("cer-reg-republish".to_string())
            .spawn(move || {
                // The guard is the FIRST value in the body, so the
                // live flag is cleared on ANY exit — return or escaped panic.
                let _live = PumpLiveGuard { live: live_thread };
                pump_loop(&exit_thread, &inner);
            });
        match spawned {
            Ok(join) => {
                *pump = Some(PumpHandle {
                    exit,
                    live,
                    join: Some(join),
                });
            }
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "registration channel: could not spawn the republish thread — each \
                     register still sends its record immediately (a gateway already up receives \
                     it), but the periodic bring-up-race belt is DISABLED: a gateway that boots \
                     later may miss pre-boot registrations until the next register re-sends them. \
                     Query registration_pump_active() to detect this degraded state"
                );
            }
        }
    }
}

/// The background republish thread body: until asked to exit, sleep one interval
/// (in short chunks so teardown is prompt) then re-send the whole registration
/// set. Record-only — it publishes into SHM the gateway drains; it never touches
/// the network or the graph's execution (Principle #7 is unaffected).
///
/// Each tick runs under [`pump_tick`]'s `catch_unwind`, so a panic
/// surfacing from the send path kills the TICK, not the belt; the thread's
/// own liveness stays accurate through the caller-held [`PumpLiveGuard`].
fn pump_loop(exit: &AtomicBool, inner: &Arc<RegChannelInner>) {
    // Chunk the interval sleep so a Drop-join returns within ~one chunk, not one
    // whole interval.
    const CHUNK: Duration = Duration::from_millis(25);
    // One latch per pump thread — this pump's tick panics only. Never shared:
    // the decisions map onto tracing right here.
    let mut panic_latch = FailureRegimeLatch::new();
    while !exit.load(Ordering::Relaxed) {
        let mut slept = Duration::ZERO;
        while slept < REG_REPUBLISH_INTERVAL {
            if exit.load(Ordering::Relaxed) {
                return;
            }
            std::thread::sleep(CHUNK);
            slept += CHUNK;
        }
        if exit.load(Ordering::Relaxed) {
            return;
        }
        // Death seam (test builds only): fires OUTSIDE the per-tick
        // containment, modelling a failure in the pump scaffolding itself —
        // the class `catch_unwind` below cannot absorb. The [`PumpLiveGuard`]
        // is what keeps the liveness flag accurate through it.
        #[cfg(any(test, feature = "test-helpers"))]
        maybe_kill_pump_thread(inner);
        pump_tick(inner, &mut panic_latch);
    }
}

/// One CONTAINED republish tick: a panic in the tick body is
/// caught, counted unconditionally, and reported through the shared
/// [`FailureRegimeLatch`] discipline — loud first `error!`, `debug!` repeats,
/// decade re-announcements, one recovery `info!` on the next clean tick — so
/// the belt survives a transient panic and a panicking-every-tick pump cannot
/// flood the log.
fn pump_tick(inner: &Arc<RegChannelInner>, panic_latch: &mut FailureRegimeLatch) {
    // AssertUnwindSafe: everything the tick touches is either behind a
    // poison-tolerant Mutex (`records` — a BTreeMap whose insert/iterate leave
    // it structurally valid), an atomic, or the iceoryx2 publisher, whose
    // failure modes are Err returns the send path already maps. A torn tick
    // leaves at worst an unsent record — which the NEXT tick re-sends, because
    // the belt is idempotent whole-set re-publication by design.
    // The process panic HOOK fires BEFORE `catch_unwind` sees the
    // panic, so containment alone still printed one default-hook stderr line
    // per contained tick — an unbounded per-interval flood beside the latch's
    // bounded lines. The guard suppresses the hook for exactly this region and
    // this thread (see the hook-suppression section above); the hook's captured
    // `file:line` is re-served on the latch's own lines below, so suppression
    // LOSES no diagnostic — it re-homes it onto the bounded channel.
    let suppress = HookSuppressGuard::new();
    let tick = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        inner.republish();
    }));
    let panic_location = suppress.take_suppressed_location();
    drop(suppress);
    match tick {
        Ok(()) => {
            if let Some(suppressed) = panic_latch.on_success() {
                tracing::info!(
                    suppressed_count = suppressed,
                    "registration channel: the republish pump recovered — the belt is \
                     republishing again"
                );
            }
        }
        Err(payload) => {
            inner.pump_tick_panics.fetch_add(1, Ordering::Relaxed);
            let reason = panic_payload_message(payload.as_ref());
            // The suppressed default-hook line carried the panic's file:line;
            // the latch lines carry it instead ("<unknown>" when the hook was
            // never installed — bare-channel tests only).
            let location = panic_location.as_deref().unwrap_or("<unknown>");
            match panic_latch.on_failure() {
                RegimeDecision::Loud => tracing::error!(
                    error = %reason,
                    panic_location = location,
                    "registration channel: a republish pump tick PANICKED — contained; \
                     the belt keeps running and re-attempts the whole set next interval \
                     (repeats log at debug)"
                ),
                RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                    error = %reason,
                    panic_location = location,
                    suppressed_count = suppressed,
                    "registration channel: republish pump tick panicked (repeat — see \
                     the first error)"
                ),
                RegimeDecision::StillFailing { total, suppressed } => tracing::error!(
                    error = %reason,
                    panic_location = location,
                    total_failures = total,
                    suppressed_count = suppressed,
                    "registration channel: republish pump ticks are STILL panicking — \
                     the regime is open; the belt keeps re-attempting each interval"
                ),
            }
        }
    }
}

/// Best-effort human-readable panic payload: a `&str` / `String` payload is
/// echoed; anything else gets a stable placeholder (never a Debug dump of a
/// boxed Any, which prints nothing useful).
fn panic_payload_message(payload: &(dyn std::any::Any + Send)) -> String {
    if let Some(s) = payload.downcast_ref::<&str>() {
        (*s).to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "non-string panic payload".to_string()
    }
}

// ---------------------------------------------------------------------------
// Contained-panic HOOK suppression — the flood's other half
// ---------------------------------------------------------------------------
//
// `catch_unwind` does not stop the process panic HOOK: the hook runs at
// `panic!` time, BEFORE unwinding reaches the catch. So without this section a
// persistently-panicking pump tick emitted one default-hook
// `thread 'cer-reg-republish' panicked at ...` stderr line per 250 ms tick —
// unbounded, beside the latch's deliberately BOUNDED lines (measured: exactly
// one line per contained panic under the default hook). That is the
// disk-fill class the latch exists to prevent, re-opened on a channel the
// latch does not control.
//
// The mechanism is the standard wrapping pattern: ONE `Once`-installed hook
// that consults a thread-local suppress flag and otherwise delegates to the
// hook it replaced, so every other thread's panic reporting — libtest's
// capture hook, the state carrier's fork hook, a user's own — is untouched.
// The flag is set ONLY around the pump's per-tick `catch_unwind` region, by an
// RAII guard (drop runs during unwinding, so a panic cannot leak it set).
//
// # Interplay with `state_carrier/hook.rs` (the fork-safety invariant)
//
// That module establishes: **nothing may take std's hook lock for
// WRITE once a capture fork is reachable** (`set_hook`/`take_hook` take the
// WRITE side of std's nonpoison `RwLock`; a fork landing mid-write strands the
// child). This installer honors the invariant by SEQUENCING, not by luck:
//
// - It is called from every `TransportManager` construction path
//   (`init_singleton`, `init_for_test`, `detached_with_config`), and a capture
//   fork requires an ARMED recorder, which requires a live manager — so the
//   one `Once`-guarded WRITE strictly precedes fork reachability.
// - [`RegistrationChannel::open`] calls it again as a BACKSTOP; in production
//   `open` only runs via `TransportManager::dynamic_egress_channel`, where the
//   manager's own install already spent the `Once`, so the backstop never
//   takes the lock there. (A test constructing a bare channel with no manager
//   pays the WRITE at `open` — safe: no capture fork exists in such a
//   process.)
// - Installation ORDER with the fork hook is immaterial: whichever wraps the
//   other, the fork hook's parent arm delegates and its child arm `_exit`s
//   before any suppression question arises, and this wrapper delegates for
//   every un-suppressed thread.
//
// The carrier's discipline walk (`state_child_discipline_test.rs`) scans
// `src/state_carrier/` only; this second, sequenced installer lives outside
// that scope by design and is documented in `hook.rs`'s module docs.

thread_local! {
    /// When `true`, the wrapped process panic hook swallows THIS thread's
    /// panic report instead of delegating. Set only by [`HookSuppressGuard`].
    static SUPPRESS_PANIC_HOOK: Cell<bool> = const { Cell::new(false) };
    /// Mailbox: the suppressed report's `file:line:column`, captured by the
    /// wrapping hook so the latch's report can still carry the location the
    /// suppressed default-hook line would have shown (`catch_unwind`'s
    /// payload carries the message but never the location).
    static SUPPRESSED_PANIC_LOCATION: RefCell<Option<String>> = const { RefCell::new(None) };
}

/// UNCONDITIONAL count of panic-hook invocations swallowed by the suppression
/// (Principle #3) — also the test-observable seam proving containment engaged
/// the hook path rather than printing. Never reset.
static SUPPRESSED_HOOK_REPORTS: AtomicU64 = AtomicU64::new(0);

/// Read [`SUPPRESSED_HOOK_REPORTS`]. Process-global (the hook is), so it needs
/// no manager handle.
#[cfg(any(test, feature = "test-helpers"))]
pub fn suppressed_panic_hook_reports_for_test() -> u64 {
    SUPPRESSED_HOOK_REPORTS.load(Ordering::Relaxed)
}

/// Install the wrapping panic hook exactly once per process. Idempotent and
/// cheap after the first call. See the section comment above for the
/// fork-safety sequencing argument governing WHERE this may be called from.
pub(crate) fn install_contained_panic_hook_suppressor() {
    static INSTALL: Once = Once::new();
    INSTALL.call_once(|| {
        let previous = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            // `try_with`: the hook can run during thread teardown, where TLS
            // access panics — a panic inside the panic hook aborts the
            // process. Fail OPEN to "not suppressed" (an extra line beats an
            // abort).
            let suppress = SUPPRESS_PANIC_HOOK.try_with(Cell::get).unwrap_or(false);
            if suppress {
                SUPPRESSED_HOOK_REPORTS.fetch_add(1, Ordering::Relaxed);
                let location = info
                    .location()
                    // hot-path-alloc-ok: panic reporting on the pump thread — never a publish/receive path.
                    .map(|l| format!("{}:{}:{}", l.file(), l.line(), l.column()));
                let _ = SUPPRESSED_PANIC_LOCATION.try_with(|slot| *slot.borrow_mut() = location);
            } else {
                previous(info);
            }
        }));
    });
}

/// RAII: suppress THIS thread's panic-hook report for the guard's scope,
/// restoring the prior value on drop. Drop runs during unwinding, so a panic
/// inside the guarded region cannot leak the flag set past the region.
struct HookSuppressGuard {
    prior: bool,
}

impl HookSuppressGuard {
    fn new() -> Self {
        Self {
            prior: SUPPRESS_PANIC_HOOK.with(|flag| flag.replace(true)),
        }
    }

    /// Take the location the wrapping hook captured for the panic this guard
    /// suppressed (empties the mailbox). `None` when the hook was never
    /// installed (bare-channel tests) or the panic machinery skipped it.
    fn take_suppressed_location(&self) -> Option<String> {
        SUPPRESSED_PANIC_LOCATION
            .try_with(|slot| slot.borrow_mut().take())
            .ok()
            .flatten()
    }
}

impl Drop for HookSuppressGuard {
    fn drop(&mut self) {
        let prior = self.prior;
        let _ = SUPPRESS_PANIC_HOOK.try_with(|flag| flag.set(prior));
    }
}

/// Death seam (test builds only): armed with a TOPIC, the next pump
/// iteration of the channel whose authoritative records CONTAIN that topic
/// panics OUTSIDE the per-tick containment — killing the pump thread, the
/// shape [`PumpLiveGuard`] exists for. TOPIC-KEYED for the same reason as
/// [`PANIC_ON_REGISTER_SEND_OF`]: the static is process-global while the test
/// binaries run tests in PARALLEL, and per-test topics are unique, so the
/// kill is confined to the arming test's own pump. Fires once, then disarms.
#[cfg(any(test, feature = "test-helpers"))]
static KILL_PUMP_THREAD_FOR: Mutex<Option<String>> = Mutex::new(None);

/// The death seam's panic payload — asserted on so a dead pump is
/// attributable to the injection.
#[cfg(any(test, feature = "test-helpers"))]
pub const PUMP_KILL_PANIC_MSG: &str =
    "test-seam: injected panic in the pump scaffolding (outside per-tick containment)";

/// Arm the death seam for the pump whose records contain `topic`.
#[cfg(any(test, feature = "test-helpers"))]
pub fn kill_pump_thread_for_test(topic: &str) {
    *KILL_PUMP_THREAD_FOR
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = Some(topic.to_string());
}

/// The death seam's trigger — see [`KILL_PUMP_THREAD_FOR`].
#[cfg(any(test, feature = "test-helpers"))]
fn maybe_kill_pump_thread(inner: &Arc<RegChannelInner>) {
    let mut armed = KILL_PUMP_THREAD_FOR
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let fire = armed.as_deref().is_some_and(|topic| {
        inner
            .records
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .contains_key(topic)
    });
    if fire {
        *armed = None;
        drop(armed);
        panic!("{PUMP_KILL_PANIC_MSG}");
    }
}

/// The READER side of the registration channel — one per gateway
/// process. A listener-less subscriber on the control service; the gateway
/// drains it each drive pass and feeds every VALID record through
/// `register_runtime_topic`. Opened `open_or_create`, so a gateway that boots
/// before any worker CREATES the service (the worker OPENS it later).
pub(crate) struct RegistrationReader {
    subscriber: Subscriber<CerService, [u8], ()>,
    /// Cumulative malformed records dropped at the drain (Principle #3 — a
    /// hostile / corrupt frame must be visible, never silent).
    malformed: u64,
    /// First malformed record of the reader's life logs `warn!`; repeats
    /// `debug!` (a flooder must not spam the log).
    malformed_warned: bool,
    /// First drain receive failure logs `warn!`; repeats `debug!`.
    receive_warned: bool,
}

impl RegistrationReader {
    /// Open the control service on `node` and build the reader.
    pub(crate) fn open(node: &Node<CerService>) -> TransportResult<Self> {
        let factory = open_control_service(node)?;
        let subscriber =
            factory
                .subscriber_builder()
                .create()
                .map_err(|e| TransportError::Internal {
                    reason: format!(
                        "registration channel: could not create the control subscriber \
                         on '{REG_CHANNEL_SERVICE_NAME}': {e:?}"
                    ),
                })?;
        Ok(Self {
            subscriber,
            malformed: 0,
            malformed_warned: false,
            receive_warned: false,
        })
    }

    /// Drain up to [`REG_CHANNEL_QUEUE_DEPTH`] records off the control queue,
    /// decoding each and pushing the VALID ones onto `out`. A malformed record
    /// is counted + warn-once + SKIPPED (never wedges the drain); a mid-drain
    /// receive failure is warn-once + returns the partial fill (non-fatal). Never
    /// holds more than one sample borrowed (each is decoded into an owned
    /// [`RegRecord`] and dropped immediately), so no borrow-budget concern.
    /// Returns the number of valid records pushed.
    pub(crate) fn drain(&mut self, out: &mut Vec<RegRecord>) -> usize {
        let mut pushed = 0usize;
        // Bound the work per pass; anything beyond drains next pass (and valid
        // records re-arrive on the next republish — nothing is permanently lost).
        for _ in 0..REG_CHANNEL_QUEUE_DEPTH {
            match self.subscriber.receive() {
                Ok(Some(sample)) => match decode_record(sample.payload()) {
                    Ok(record) => {
                        out.push(record);
                        pushed += 1;
                    }
                    Err(e) => self.record_malformed(&e),
                },
                Ok(None) => break,
                Err(e) => {
                    self.record_receive_failure(&format!("{e}"));
                    break;
                }
            }
        }
        pushed
    }

    /// Cumulative malformed records dropped at the drain (Principle #3).
    pub(crate) fn malformed_count(&self) -> u64 {
        self.malformed
    }

    fn record_malformed(&mut self, error: &RegRecordError) {
        self.malformed += 1;
        if self.malformed_warned {
            tracing::debug!(
                error = %error,
                "registration channel: dropped a malformed control record \
                 (repeat — see the first warning)"
            );
        } else {
            self.malformed_warned = true;
            tracing::warn!(
                error = %error,
                "registration channel: dropped a malformed control record on \
                 '{REG_CHANNEL_SERVICE_NAME}' (counted; the drain continues; repeats log at debug)"
            );
        }
    }

    fn record_receive_failure(&mut self, reason: &str) {
        if self.receive_warned {
            tracing::debug!(
                reason = %reason,
                "registration channel: control drain receive failed (repeat)"
            );
        } else {
            self.receive_warned = true;
            tracing::warn!(
                reason = %reason,
                "registration channel: control drain receive failed — kept the partial \
                 fill; the next drive pass retries (repeats log at debug)"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The service name has no `/data` suffix, so `cerulion topic list` (which
    /// strips `/data` to derive topic names) never surfaces the control service.
    /// Also under the reserved namespace, so `register_runtime_topic` / the
    /// public API refuse to register it as an egress topic. Hand oracle.
    #[test]
    fn reg_channel_service_name_has_no_data_suffix() {
        assert!(
            !REG_CHANNEL_SERVICE_NAME.ends_with("/data"),
            "the control service must NOT end in /data or `topic list` would surface it"
        );
        assert!(
            REG_CHANNEL_SERVICE_NAME.starts_with(super::super::gateway::RESERVED_TOPIC_PREFIX),
            "the control service must live under the reserved control-plane namespace"
        );
        assert!(super::super::gateway::is_reserved_topic(
            REG_CHANNEL_SERVICE_NAME
        ));
    }

    /// Round-trip: encode then decode reproduces the input EXACTLY, across a
    /// plain topic, a max-length topic, a zero hash, and a max hash. Hand oracle
    /// (not a self-compare — the decoded value is checked field-by-field).
    #[test]
    fn encode_decode_round_trips() {
        let cases: &[(&str, u64)] = &[
            ("/utlidar/cloud", 0x1234_5678_9abc_def0),
            ("/a", 0),
            ("/z", u64::MAX),
            ("/lf/sportmodestate", 42),
        ];
        for &(topic, hash) in cases {
            let bytes = encode_record(topic, hash).expect("encode");
            assert_eq!(bytes.len(), REG_HEADER_LEN + topic.len());
            let decoded = decode_record(&bytes).expect("decode");
            assert_eq!(
                decoded,
                RegRecord {
                    topic: topic.to_string(),
                    schema_hash: hash,
                }
            );
        }
        // A genuinely MAX-length topic round-trips; one byte longer is rejected.
        let max_topic = format!("/{}", "x".repeat(MAX_REG_TOPIC_LEN - 1));
        assert_eq!(max_topic.len(), MAX_REG_TOPIC_LEN);
        let bytes = encode_record(&max_topic, 7).expect("encode max");
        assert_eq!(decode_record(&bytes).unwrap().topic, max_topic);
    }

    /// Encode rejects an empty topic and an over-long topic. Hand oracle.
    #[test]
    fn encode_rejects_empty_and_over_long() {
        assert_eq!(encode_record("", 1), Err(RegRecordError::EmptyTopic));
        let too_long = "x".repeat(MAX_REG_TOPIC_LEN + 1);
        assert_eq!(
            encode_record(&too_long, 1),
            Err(RegRecordError::TopicTooLong {
                len: MAX_REG_TOPIC_LEN + 1
            })
        );
    }

    /// Decode rejects every malformed shape with the PRECISE reason: truncated,
    /// bad magic, bad version, a length-mismatch (declared ≠ actual), an
    /// over-long declared length, and invalid UTF-8. Hand oracles per variant.
    #[test]
    fn decode_rejects_each_malformed_shape() {
        // Truncated (shorter than the header).
        assert_eq!(
            decode_record(&[0xCE, 0x21, 0x01]),
            Err(RegRecordError::Truncated { len: 3 })
        );
        // A valid frame we then corrupt in each field.
        let good = encode_record("/t", 9).expect("encode");
        // Bad magic.
        let mut bad_magic = good.clone();
        bad_magic[0] = 0x00;
        assert_eq!(decode_record(&bad_magic), Err(RegRecordError::BadMagic));
        // Bad version.
        let mut bad_ver = good.clone();
        bad_ver[2] = 0xFF;
        assert_eq!(
            decode_record(&bad_ver),
            Err(RegRecordError::BadVersion { found: 0xFF })
        );
        // Length mismatch: declare a 100-byte topic on a 2-byte-topic frame.
        let mut bad_len = good.clone();
        bad_len[11..13].copy_from_slice(&100u16.to_le_bytes());
        assert_eq!(
            decode_record(&bad_len),
            Err(RegRecordError::LengthMismatch {
                declared: 100,
                actual_payload: 2,
            })
        );
        // Over-long declared length (> MAX_REG_TOPIC_LEN) rejected before the
        // length-match check.
        let mut over = good.clone();
        over[11..13].copy_from_slice(&((MAX_REG_TOPIC_LEN + 1) as u16).to_le_bytes());
        assert_eq!(
            decode_record(&over),
            Err(RegRecordError::TopicTooLong {
                len: MAX_REG_TOPIC_LEN + 1
            })
        );
        // Invalid UTF-8 in the topic bytes: build a valid header for a 2-byte
        // topic, then plant non-UTF-8 bytes.
        let mut bad_utf8 = encode_record("/t", 9).expect("encode");
        let n = bad_utf8.len();
        bad_utf8[n - 2] = 0xFF;
        bad_utf8[n - 1] = 0xFE;
        assert_eq!(decode_record(&bad_utf8), Err(RegRecordError::InvalidUtf8));
    }

    /// [`is_creation_collision`] classifies EXACTLY the two
    /// `HangsInCreation` arms as the retryable cross-process first-creation
    /// collision — nothing else may buy a re-attempt (a type skew, a
    /// corruption, a port-cap mismatch must surface immediately, and
    /// `SystemInFlux` is iceoryx2's OWN retry-exhausted verdict, which
    /// re-retrying here would silently double the bound). `want_open` /
    /// `want_create` are EXHAUSTIVE matches with no `_` arm, so an iceoryx2
    /// upgrade that adds a variant fails to COMPILE here instead of silently
    /// defaulting into (or out of) the retry set — the
    /// `only_the_buffer_ceiling_variant_proves_a_shallow_incumbent` pattern.
    #[test]
    fn only_hangs_in_creation_is_the_retryable_collision() {
        use iceoryx2::service::builder::publish_subscribe::{
            PublishSubscribeCreateError as C, PublishSubscribeOpenError as O,
            PublishSubscribeOpenOrCreateError as E,
        };

        fn want_open(v: O) -> bool {
            match v {
                O::HangsInCreation => true,
                O::DoesNotExist
                | O::InternalFailure
                | O::IncompatibleTypes
                | O::IncompatibleMessagingPattern
                | O::IncompatibleAttributes
                | O::DoesNotSupportRequestedMinBufferSize
                | O::DoesNotSupportRequestedMinHistorySize
                | O::DoesNotSupportRequestedMinSubscriberBorrowedSamples
                | O::DoesNotSupportRequestedAmountOfPublishers
                | O::DoesNotSupportRequestedAmountOfSubscribers
                | O::DoesNotSupportRequestedAmountOfNodes
                | O::IncompatibleOverflowBehavior
                | O::InsufficientPermissions
                | O::ServiceInCorruptedState
                | O::ExceedsMaxNumberOfNodes
                | O::IsMarkedForDestruction
                // iceoryx2 0.10 additions. None is a creation collision:
                // `VersionMismatch` names a library skew, and a re-attempt
                // would hide it behind the retry bound.
                | O::Interrupt
                | O::UnableToCreateServiceTag
                | O::VersionMismatch
                | O::UnableToAcquireTypeDefinition
                | O::InvalidTypeDefinition => false,
            }
        }
        fn want_create(v: C) -> bool {
            match v {
                C::HangsInCreation => true,
                C::ServiceInCorruptedState
                | C::SubscriberBufferMustBeLargerThanHistorySize
                | C::AlreadyExists
                | C::InsufficientPermissions
                | C::InternalFailure
                | C::IsBeingCreatedByAnotherInstance
                // iceoryx2 0.10 additions.
                | C::Interrupt
                | C::UnableToCreateServiceTag
                | C::ServiceConfigCouldNotBeCreated
                | C::UnableToAcquireTypeDefinition
                | C::InvalidTypeDefinition
                | C::UnableToGenerateUniqueServiceId => false,
            }
        }

        let all_open = [
            O::DoesNotExist,
            O::InternalFailure,
            O::IncompatibleTypes,
            O::IncompatibleMessagingPattern,
            O::IncompatibleAttributes,
            O::DoesNotSupportRequestedMinBufferSize,
            O::DoesNotSupportRequestedMinHistorySize,
            O::DoesNotSupportRequestedMinSubscriberBorrowedSamples,
            O::DoesNotSupportRequestedAmountOfPublishers,
            O::DoesNotSupportRequestedAmountOfSubscribers,
            O::DoesNotSupportRequestedAmountOfNodes,
            O::IncompatibleOverflowBehavior,
            O::InsufficientPermissions,
            O::ServiceInCorruptedState,
            O::HangsInCreation,
            O::ExceedsMaxNumberOfNodes,
            O::IsMarkedForDestruction,
            O::Interrupt,
            O::UnableToCreateServiceTag,
            O::VersionMismatch,
            O::UnableToAcquireTypeDefinition,
            O::InvalidTypeDefinition,
        ];
        for v in all_open {
            assert_eq!(
                is_creation_collision(&E::PublishSubscribeOpenError(v)),
                want_open(v),
                "{v:?} classified wrongly — only HangsInCreation buys a re-attempt"
            );
        }
        let all_create = [
            C::ServiceInCorruptedState,
            C::SubscriberBufferMustBeLargerThanHistorySize,
            C::AlreadyExists,
            C::InsufficientPermissions,
            C::InternalFailure,
            C::IsBeingCreatedByAnotherInstance,
            C::HangsInCreation,
            C::Interrupt,
            C::UnableToCreateServiceTag,
            C::ServiceConfigCouldNotBeCreated,
            C::UnableToAcquireTypeDefinition,
            C::InvalidTypeDefinition,
            C::UnableToGenerateUniqueServiceId,
        ];
        for v in all_create {
            assert_eq!(
                is_creation_collision(&E::PublishSubscribeCreateError(v)),
                want_create(v),
                "{v:?} classified wrongly — only HangsInCreation buys a re-attempt"
            );
        }
        assert!(
            !is_creation_collision(&E::SystemInFlux),
            "SystemInFlux is iceoryx2's own retry-exhausted verdict — it must never re-retry here"
        );
        // Anti-tautology: both sweeps really contain a TRUE row, so a
        // classifier stuck at `false` cannot pass the loops above.
        assert!(all_open.iter().copied().any(want_open));
        assert!(all_create.iter().copied().any(want_create));
    }

    /// A max-length record encodes to exactly [`MAX_REG_RECORD_LEN`] bytes —
    /// pins the publisher's `initial_max_slice_len` sizing. Hand oracle.
    #[test]
    fn max_record_len_matches_max_slice_len() {
        let max_topic = format!("/{}", "x".repeat(MAX_REG_TOPIC_LEN - 1));
        let bytes = encode_record(&max_topic, 0).expect("encode");
        assert_eq!(bytes.len(), MAX_REG_RECORD_LEN);
    }

    /// The republish ORDER rotation guarantees full convergence for a burst
    /// LARGER than the reader's `drop_oldest` queue. Models the eviction exactly
    /// (a queue of `q` keeping the LAST `q` of the rotated send order — the same
    /// `rotate_left(round * q % n)` shift the production `RegChannelInner::republish`
    /// applies) and asserts the UNION of delivered indices across ⌈n/q⌉ rounds
    /// covers every one of the `n` topics — i.e. no topic is perpetually starved
    /// (the bug a fixed sorted send order would have). Hand oracle over the index
    /// set; also the anti-regression pin that WITHOUT rotation (round always 0)
    /// the lowest `n-q` indices are NEVER delivered.
    #[test]
    fn republish_rotation_converges_for_bursts_larger_than_the_queue() {
        // The set delivered in one round given the rotate_left(shift) send order
        // into a queue of `q` (drop_oldest keeps the LAST `q` of the rotated
        // sequence; rotated[i] = original[(shift + i) % n]).
        fn delivered_in_round(n: usize, q: usize, round: usize) -> Vec<usize> {
            if n <= q {
                return (0..n).collect();
            }
            let shift = round.wrapping_mul(q) % n;
            // Last `q` positions map to original indices [(shift + n - q) % n ..).
            let start = (shift + n - q) % n;
            (0..q).map(|k| (start + k) % n).collect()
        }
        for &(n, q) in &[(300usize, 256usize), (600, 256), (257, 256), (512, 256)] {
            let rounds = n.div_ceil(q);
            let mut covered = std::collections::BTreeSet::new();
            for r in 0..rounds {
                covered.extend(delivered_in_round(n, q, r));
            }
            let want: std::collections::BTreeSet<usize> = (0..n).collect();
            assert_eq!(
                covered, want,
                "rotation must cover all {n} topics within {rounds} rounds (q={q})"
            );
            // Anti-regression: WITHOUT rotation (round pinned at 0) the lowest
            // `n - q` topics are NEVER delivered — the starvation bug rotation fixes.
            let no_rotation: std::collections::BTreeSet<usize> =
                delivered_in_round(n, q, 0).into_iter().collect();
            assert!(
                !no_rotation.contains(&0),
                "without rotation, topic 0 is starved for a burst of {n} > queue {q}"
            );
        }
    }

    /// The PRODUCTION-PATH convergence pin for a burst LARGER than the
    /// reader's `drop_oldest` queue. The sibling
    /// `republish_rotation_converges_for_bursts_larger_than_the_queue` re-implements
    /// the eviction formula in the test and never touches transport, and the
    /// largest e2e burst elsewhere (90) is < [`REG_CHANNEL_QUEUE_DEPTH`], so the
    /// production `RegChannelInner::republish` rotation block was UNEXERCISED —
    /// deleting it passed every test green. This drives 300 > 256 registrations
    /// through the REAL iceoryx2 control service (a live publisher + a live
    /// [`RegistrationReader::drain`]) and asserts both halves of the convergence
    /// contract.
    ///
    /// Determinism: we drive [`RegChannelInner::insert`]/[`RegChannelInner::republish`]
    /// DIRECTLY (never [`RegistrationChannel::register`], so no background pump
    /// thread ever starts), single-threaded, and RESET `republish_count` after a
    /// connection warm-up — so the two measured rounds are the exact hand oracle.
    ///
    /// What breaks this test: delete the `if snapshot.len() > REG_CHANNEL_QUEUE_DEPTH`
    /// rotation block in `republish` and every round sends the SAME fixed sorted
    /// order, so `drop_oldest` keeps only the top-256 sorted topics EVERY round —
    /// the 44 lowest-sorted (`t0000..t0043`) NEVER land AND `republish_count` never
    /// increments. This test then fails on BOTH the all-topics-landed assertion and
    /// the `republish_count == 2` counter-delta assertion.
    #[test]
    fn production_republish_rotation_converges_over_real_queue() {
        use crate::transport::{TransportConfig, TransportManager};
        use std::collections::BTreeMap;
        use std::sync::atomic::Ordering;

        let root = crate::testing::iceoryx_test_config();
        // Reader first (creates the service); writer opens it (open_or_create).
        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "rc_fix2_reader".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("reader manager");
        let writer_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "rc_fix2_writer".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("writer manager");

        let mut reader = RegistrationReader::open(&reader_mgr.node).expect("reader");
        let channel = RegistrationChannel::open(&writer_mgr.node).expect("channel");

        // Insert N > queue registrations DIRECTLY on the inner core (never
        // `register`, so no pump thread starts). 4-digit zero-pad ⇒ the BTreeMap's
        // lexicographic order == numeric order, so "the lowest-sorted 44" are
        // exactly t0000..t0043.
        const N: usize = 300;
        const {
            assert!(
                N > REG_CHANNEL_QUEUE_DEPTH,
                "the burst must exceed the queue depth or the rotation block never runs"
            )
        };
        for i in 0..N {
            assert!(
                channel.inner.insert(&format!("/rt/fix2/t{i:04}"), i as u64),
                "each is a fresh registration"
            );
        }

        // Warm up the pub↔sub connection (in-process iceoryx2 takes a pass or two
        // to establish it). These warm-up republishes advance republish_count; we
        // reset it below so the measured rounds start from a known state.
        let mut scratch: Vec<RegRecord> = Vec::new();
        let mut connected = false;
        for _ in 0..20 {
            channel.inner.republish();
            scratch.clear();
            if reader.drain(&mut scratch) > 0 {
                connected = true;
                break;
            }
        }
        assert!(
            connected,
            "the control pub↔sub connection never established"
        );
        // Fully drain any residue, then reset the rotation counter so the clean
        // rounds below are the exact hand oracle.
        loop {
            scratch.clear();
            if reader.drain(&mut scratch) == 0 {
                break;
            }
        }
        channel.inner.republish_count.store(0, Ordering::Relaxed);

        // Exactly ⌈N/queue⌉ = 2 clean rounds. Single-threaded: each republish sends
        // the full set, drop_oldest keeps the last 256, drain pulls them all and
        // empties the queue before the next round — fully deterministic.
        let rounds = N.div_ceil(REG_CHANNEL_QUEUE_DEPTH);
        assert_eq!(rounds, 2, "the oracle assumes exactly 2 rounds for 300/256");
        let mut got: BTreeMap<String, u64> = BTreeMap::new();
        for _ in 0..rounds {
            channel.inner.republish();
            scratch.clear();
            reader.drain(&mut scratch);
            for r in &scratch {
                got.insert(r.topic.clone(), r.schema_hash);
            }
        }

        // (1) FULL convergence: every one of the N topics landed with its hash
        // (hand oracle over the whole set — never a self-compare).
        assert_eq!(
            got.len(),
            N,
            "all {N} topics converged within {rounds} republish rounds"
        );
        for i in 0..N {
            let topic = format!("/rt/fix2/t{i:04}");
            assert_eq!(
                got.get(&topic).copied(),
                Some(i as u64),
                "topic {topic} landed with its advertised hash"
            );
        }
        // The explicit mutation witness: the LOWEST-sorted topic landed — the exact
        // one a no-rotation regression starves (drop_oldest keeps only the top 256).
        assert!(
            got.contains_key("/rt/fix2/t0000"),
            "the lowest-sorted topic landed — impossible without the production rotation"
        );

        // (2) The production rotation block ran on BOTH rounds — `republish_count`
        // is bumped ONLY inside `if snapshot.len() > REG_CHANNEL_QUEUE_DEPTH`, so a
        // delta of exactly `rounds` proves the rotation engaged each round.
        assert_eq!(
            channel.inner.republish_count.load(Ordering::Relaxed),
            rounds as u64,
            "the rotation block engaged on each of the {rounds} rounds"
        );
    }

    /// Arming the pump-send seam with `count = 0` must
    /// be a DISARM/no-op, per its own doc contract ("the next `count` sends").
    /// Inserting a zero entry the trigger's `Some(..)` arm still
    /// FIRES on once before removing it would make 0 behave as 1. Unique topic: the
    /// seam static is process-global and lib tests run in parallel.
    #[test]
    fn a_zero_count_pump_seam_arming_is_a_disarm_not_a_single_fire() {
        let topic = "/rt/r3/zero_count_disarm_probe";

        // Zero on a never-armed topic: no entry, and a republish must NOT fire
        // (a zero entry would fire once and panic here).
        panic_on_next_pump_sends_for_test(topic, 0);
        maybe_panic_after_pump_send(topic, SendSite::Republish);

        // Zero DISARMS a live arming: no fire afterwards.
        panic_on_next_pump_sends_for_test(topic, 2);
        panic_on_next_pump_sends_for_test(topic, 0);
        maybe_panic_after_pump_send(topic, SendSite::Republish);

        // Anti-vacuity control: `count = 1` still fires exactly once (the
        // no-op must not over-reach), then is disarmed.
        panic_on_next_pump_sends_for_test(topic, 1);
        let fired = std::panic::catch_unwind(|| {
            maybe_panic_after_pump_send(topic, SendSite::Republish);
        });
        assert!(
            fired.is_err(),
            "count = 1 must still fire — the zero no-op must not swallow real armings"
        );
        maybe_panic_after_pump_send(topic, SendSite::Republish); // disarmed after its one fire
    }
}
