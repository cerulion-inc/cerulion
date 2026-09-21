// SPDX-License-Identifier: AGPL-3.0-only
//! The INGRESS-BUILD PROGRESS channel `/__cerulion/ingress_build`.
//!
//! # Why this exists
//!
//! A freshly-booted robot's FIRST `graph run attach --single-process --record`
//! captured **4 topics** — the declared set — while all ~98 of the bridge's raw
//! routes were reported `appeared_after_bag_creation`. The identical command two
//! minutes later, against a warm DDS bus, captured 102 topics COMPLETE. Two of
//! three fresh-participant runs lost that race.
//!
//! Neither half was wrong on its own:
//!
//! * a `ros2 attach` bridge opens its raw routes SEQUENTIALLY, and on a cold DDS
//!   bus each open waits on SPDP/SEDP matching plus the schema ladder —
//!   MEASURED at ~55-60 s from process start to the last route on the Go2, in
//!   BURSTS separated by multi-second gaps;
//! * the recorder closes its channel set at `max(settle release, schema
//!   learning)`, and the settle releases after
//!   `DISCOVERY_SETTLE_QUIET_SCANS` (2) quiet 250 ms rescans past a 500 ms floor
//!   — so ANY gap longer than half a second closes it.
//!
//! A channel that closes cannot re-open (registered channels are immutable, and
//! rotation reuses the created set verbatim), so every route opened afterwards
//! can only be REPORTED, never recorded. **Raising the settle cap cannot fix
//! this**: the cap is a ceiling, and the quiet rule releases far below it.
//!
//! What the recorder is missing is not a longer timer — it is the ONE fact only
//! the producing process holds: *"my ingress plane is still being built"*. This
//! module carries that fact across the process boundary, and
//! `cerulion_bagd` holds its channel set open while it is being told so.
//!
//! # What the signal actually claims, and what it does not
//!
//! The record says exactly one thing: **"this process has created N runtime
//! ingress routes so far"**. It is emitted from
//! [`TransportManager::create_ingress_publisher`](super::TransportManager::create_ingress_publisher)
//! — the ONE seam through which a runtime ingress route is minted — so it is a
//! property of ANY progressively-built ingress plane (a `ros2 attach` bridge, a
//! `cerulion-netd` desk mirror plane, anything future), not of one robot.
//!
//! It deliberately carries **no denominator**. The seam that mints a route
//! cannot know how many more are coming: the `dds_bridge`'s expected count lives
//! in its own YAML mapping list, on its own drain thread, in a crate that is
//! HAND-VENDORED into user workspaces (`cerulion ros2 attach` only probes for
//! `nodes/dds_bridge/`, it never generates or copies it). A signal that needed a
//! bridge-side change would leave every already-vendored workspace broken; a
//! signal minted in `cerulion_core` reaches every bridge — old or new — the
//! moment it is rebuilt. That is the whole reason the producing side landed here.
//!
//! # The record answers RECENCY, and that is what makes a cold boot work
//!
//! It also carries [`IngressBuildRecord::since_last_creation`] — how long ago
//! **by the writer's own elapsed time** it last created a route. That field is
//! the whole discriminator, and it exists because the receiver cannot compute it.
//!
//! A record is a running TOTAL, so the first one heard from a writer is a
//! snapshot of a history nobody watched: a robot whose plane finished an hour
//! ago republishes `98` forever, and by VALUE or by TIMING that is
//! indistinguishable from a robot that has just created its 98th route. A
//! receiver forced to infer motion by watching a count CHANGE therefore has to
//! bank the first record — and that is exactly wrong in the shape this whole
//! feature exists for. On a `graph run --record` COLD BOOT the recorder is armed
//! *before* the graph reaches step 0, so the first record it ever hears is
//! **route 1's**, there is nothing to compare it against, and the quiet
//! rule closes the channel set inside the multi-second gap before route 2.
//!
//! The WRITER knows which it is, because it knows when it last created
//! something. So it says so, and a receiver's question collapses from "did this
//! number change?" to "did anybody create a route recently?" — answerable from a
//! SINGLE record, including the very first one.
//!
//! Why a self-measured ELAPSED and not a `Created`/`Rebroadcast` kind bit (the
//! obvious alternative): a kind bit re-opens the same hole one dropped frame
//! later. If route 1's `Created` record is lost, the belt's next frame is a
//! `Rebroadcast` carrying `1`, the receiver banks it, and the cold boot fails
//! again. An elapsed is **drop-tolerant** — the belt's frame 250 ms later still
//! says "created 250 ms ago", which is still recent, so the hold arms anyway.
//!
//! It is also clock-domain-safe (the repo's wire-stamp rule): the field is a
//! DELTA measured entirely inside the writer, never a stamp to be compared
//! against a reader's clock. A reader ages it by ADDING its own elapsed since
//! receipt — the sum of two elapsed intervals, each in its own domain.
//!
//! [`IngressBuildProgress`]'s baseline/advance fold survives, but it now serves
//! only the REPORTED `advances` figure; the hold itself keys on recency. See
//! `cerulion_bagd::ingress_hold` for the policy.
//!
//! # Modelled on the registration channel, deliberately
//!
//! This is [`reg_channel`](super::reg_channel) with a different payload: same
//! fixed reserved service name with NO `/data` suffix, same magic + version +
//! bounded record discipline, same writer/reader-from-ONE-helper rule so their
//! static configs cannot drift, same immediate-send-plus-periodic-republish
//! belt so a reader that connects LATE still converges, same poll-drained reader
//! (no event service — the consumer already has a drive loop). **No new
//! transport concept is introduced.**
//!
//! It is NOT folded into `reg_channel` itself, even though the two ride the same
//! shape and often the same process. `reg_channel` means "ANNOUNCE this topic to
//! the network": the gateway drains it and makes each record egress-grantable.
//! Pushing ~98 raw routes through it to signal progress would change what the
//! robot advertises — a network-plane side effect in service of a recorder
//! question. Two facts, two channels.
//!
//! ## Why one record per WRITER, and why it is a running total
//!
//! Each process publishes a single record carrying its own `writer_id` and its
//! CURRENT cumulative route count — not a per-route event. So:
//!
//! * a reader that misses a record loses nothing (the next one carries the same
//!   running total, and the republish belt re-sends it);
//! * the reader's fold is idempotent (`max` per writer), so `drop_oldest`
//!   overflow and re-delivery are both harmless;
//! * a multi-process graph is handled by construction — every worker that opens
//!   ingress routes publishes its own record, and the consumer sums them.
//!
//! # Record/replay inertness
//!
//! [`INGRESS_BUILD_SERVICE_NAME`] carries NO `/data` suffix, and
//! `TransportManager::list_topics` derives topics only from `*/data` services —
//! so `cerulion topic list` never surfaces it and `cerulion_bagd`'s live-service
//! enumeration never taps it (pinned by
//! `tests::ingress_build_service_names_have_no_data_suffix` here, and at the
//! recorder by the discovery e2e arm). Nothing about this channel
//! reaches a node's execution, so Principle #7 is untouched.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use iceoryx2::node::Node;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::ServiceName;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;

use super::CerService;
use crate::error::{TransportError, TransportResult};

/// The fixed iceoryx2 service name of the ingress-build progress
/// channel — canonical `/__cerulion/ingress_build`, under
/// [`RESERVED_TOPIC_PREFIX`](super::gateway::RESERVED_TOPIC_PREFIX).
/// Deliberately carries NO `/data` suffix (unlike graph topic services
/// `{topic}/data`), so `cerulion topic list` — which enumerates `*/data`
/// services — never surfaces it, and `cerulion_bagd`'s live-topic enumeration
/// never taps it. Both the writer (`IngressBuildChannel`) and the reader
/// ([`IngressBuildReader`]) build their service from this ONE constant, so their
/// static configs cannot drift.
pub const INGRESS_BUILD_SERVICE_NAME: &str = "/__cerulion/ingress_build";

/// `max_publishers` the progress service is provisioned with — one per
/// local process that opens runtime ingress routes (a multi-process graph's
/// workers, plus a `cerulion-netd` sharing the machine). Sized like
/// [`REG_CHANNEL_MAX_WRITERS`](super::reg_channel::REG_CHANNEL_MAX_WRITERS),
/// generously above any realistic count and nowhere near iceoryx2's port
/// ceilings.
pub const INGRESS_BUILD_MAX_WRITERS: usize = 64;

/// `max_subscribers` — a recorder is the sole reader, plus headroom
/// for tests and a second introspecting reader.
///
/// This number is part of the service's STATIC config and BOTH sides
/// `open_or_create` with it, so raising it later would make a new binary unable
/// to open a service an OLD process on the same machine had already created.
/// Four is chosen once, here, for that reason.
pub const INGRESS_BUILD_MAX_READERS: usize = 4;

/// `subscriber_max_buffer_size` — the per-pass reader queue. Each
/// writer contributes at most one record per route creation plus one per
/// republish, and the fold is idempotent, so overflow costs nothing; the depth
/// simply keeps a whole burst in one drain pass.
pub const INGRESS_BUILD_QUEUE_DEPTH: usize = 256;

/// How often the background thread re-sends this process's current
/// record — the belt that closes the bring-up race regardless of which process
/// started first, and the reason a reader that connects LATE still converges to
/// the current total. Matches
/// [`REG_REPUBLISH_INTERVAL`](super::reg_channel::REG_REPUBLISH_INTERVAL).
pub const INGRESS_BUILD_REPUBLISH_INTERVAL: Duration = Duration::from_millis(250);

/// The record magic (`0xCE`, `0xDB` — Cerulion, and `0xDB` is the low
/// byte of 1243). A frame that does not open with this is not one of ours and is
/// rejected as malformed (defense in depth on a dedicated service that should
/// never carry foreign frames).
const INGRESS_BUILD_MAGIC: [u8; 2] = [0xCE, 0xDB];

/// The record wire-format version. Bumped only on an incompatible
/// record layout change; a mismatched version is rejected as malformed.
///
/// **2** since the record gained `since_last_creation_ms`, the recency field
/// that lets a cold boot's very FIRST record arm the recorder's hold (see the
/// module docs). Version 1 records are rejected as malformed rather than decoded
/// on a default, because a defaulted recency would be a POSITIVE claim about
/// when a writer last built something — precisely the kind of fabricated
/// evidence this channel exists to replace.
const INGRESS_BUILD_VERSION: u8 = 2;

/// The FIXED record length — `magic(2) + version(1) + writer_id(8) +
/// routes_created(8) + since_last_creation_ms(8)`. There is no variable payload:
/// the record carries no names, only counts and one duration, which is why the
/// whole channel costs one 27-byte send per route creation.
pub const INGRESS_BUILD_RECORD_LEN: usize = 2 + 1 + 8 + 8 + 8;

/// The wire value meaning "this writer has created NOTHING yet", so a
/// reader can tell "no creation to report" from "created 0 ms ago". Decoded to
/// [`IngressBuildRecord::since_last_creation`] `= None`.
pub const SINCE_LAST_CREATION_NONE: u64 = u64::MAX;

/// A decoded ingress-build progress record — one writer process's
/// identity plus the number of runtime ingress routes it has created so far.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IngressBuildRecord {
    /// Process-unique writer identity (see `mint_writer_id`). Distinguishes two
    /// concurrently-building processes, and makes a RESTARTED process a new
    /// entry rather than a count that appears to go backwards.
    pub writer_id: u64,
    /// Runtime ingress routes this writer has created since it opened the
    /// channel. Monotonically non-decreasing for a given `writer_id`.
    pub routes_created: u64,
    /// How long ago, **by the writer's own elapsed time**, this writer last
    /// created a route — `None` if it has created none yet.
    ///
    /// THE field that makes a cold boot work: it is a claim a receiver cannot
    /// compute, and it is answerable from a single record, so the very FIRST
    /// record heard from a writer already says whether that writer is actively
    /// building. See the module docs for why this beats a `Created`/`Rebroadcast`
    /// kind bit (that one re-opens the hole one dropped frame later).
    ///
    /// A DELTA in the writer's own domain, never a stamp: a reader ages it by
    /// adding its OWN elapsed since receipt, which sums two elapsed intervals
    /// rather than comparing two clocks.
    pub since_last_creation: Option<Duration>,
}

/// Why a wire record was rejected. Every arm is a REPORTED drop at the
/// reader, never a silent one.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IngressBuildRecordError {
    /// The frame is not [`INGRESS_BUILD_RECORD_LEN`] bytes.
    WrongLength {
        /// What arrived.
        actual: usize,
    },
    /// The frame does not open with the record magic.
    BadMagic,
    /// The frame's version byte is not the supported record version.
    BadVersion {
        /// The version byte that arrived.
        got: u8,
    },
}

impl std::fmt::Display for IngressBuildRecordError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::WrongLength { actual } => write!(
                f,
                "ingress-build record is {actual} bytes, expected exactly {INGRESS_BUILD_RECORD_LEN}"
            ),
            Self::BadMagic => write!(f, "ingress-build record does not carry the record magic"),
            Self::BadVersion { got } => write!(
                f,
                "ingress-build record version {got} is not the supported version \
                 {INGRESS_BUILD_VERSION}"
            ),
        }
    }
}

/// Encode one progress record for the wire. Pure (no transport); the
/// exact inverse of [`decode_record`]. Infallible — the record is fixed-size and
/// carries no caller-supplied bytes.
#[must_use]
pub fn encode_record(
    writer_id: u64,
    routes_created: u64,
    since_last_creation_ms: u64,
) -> [u8; INGRESS_BUILD_RECORD_LEN] {
    let mut buf = [0u8; INGRESS_BUILD_RECORD_LEN];
    buf[0..2].copy_from_slice(&INGRESS_BUILD_MAGIC);
    buf[2] = INGRESS_BUILD_VERSION;
    buf[3..11].copy_from_slice(&writer_id.to_le_bytes());
    buf[11..19].copy_from_slice(&routes_created.to_le_bytes());
    buf[19..27].copy_from_slice(&since_last_creation_ms.to_le_bytes());
    buf
}

/// Decode one progress record off the wire. Pure (no transport); the
/// exact inverse of [`encode_record`].
///
/// # Errors
///
/// Returns [`IngressBuildRecordError`] for a frame of the wrong length, a frame
/// that does not carry the record magic, or a frame carrying an unsupported
/// version.
pub fn decode_record(bytes: &[u8]) -> Result<IngressBuildRecord, IngressBuildRecordError> {
    if bytes.len() != INGRESS_BUILD_RECORD_LEN {
        return Err(IngressBuildRecordError::WrongLength {
            actual: bytes.len(),
        });
    }
    if bytes[0..2] != INGRESS_BUILD_MAGIC {
        return Err(IngressBuildRecordError::BadMagic);
    }
    if bytes[2] != INGRESS_BUILD_VERSION {
        return Err(IngressBuildRecordError::BadVersion { got: bytes[2] });
    }
    let writer_id = u64::from_le_bytes(bytes[3..11].try_into().expect("8 bytes"));
    let routes_created = u64::from_le_bytes(bytes[11..19].try_into().expect("8 bytes"));
    let since_ms = u64::from_le_bytes(bytes[19..27].try_into().expect("8 bytes"));
    Ok(IngressBuildRecord {
        writer_id,
        routes_created,
        since_last_creation: (since_ms != SINCE_LAST_CREATION_NONE)
            .then(|| Duration::from_millis(since_ms)),
    })
}

/// Mint a process-unique writer identity — pid in the high half, a
/// process-global sequence plus a nanosecond timestamp in the low half. Distinct
/// per channel by construction, which is what makes a RESTARTED producer a new
/// map entry at the reader instead of a count that appears to move backwards.
fn mint_writer_id() -> u64 {
    use std::sync::atomic::AtomicU64 as Seq;
    static SEQ: Seq = Seq::new(0);
    let pid = u64::from(std::process::id());
    let seq = SEQ.fetch_add(1, Ordering::Relaxed);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.subsec_nanos() as u64);
    (pid << 32) ^ (seq << 16) ^ nanos
}

/// Open the ONE progress pub/sub service (`open_or_create`, so
/// whichever side — a producing worker or the recorder — arrives first CREATES
/// it and the other OPENS it). Both sides call THIS helper, so the static config
/// (queue depth / writer slots / reader slots; NO event service, NO history) is
/// identical and `open_or_create` is always compatible.
fn open_progress_service(
    node: &Node<CerService>,
) -> TransportResult<PortFactory<CerService, [u8], ()>> {
    let name: ServiceName =
        INGRESS_BUILD_SERVICE_NAME
            .try_into()
            .map_err(|e| TransportError::Internal {
                reason: format!(
                    "ingress-build channel: invalid service name \
                     '{INGRESS_BUILD_SERVICE_NAME}': {e:?}"
                ),
            })?;
    node.service_builder(&name)
        .publish_subscribe::<[u8]>()
        .subscriber_max_buffer_size(INGRESS_BUILD_QUEUE_DEPTH)
        .max_subscribers(INGRESS_BUILD_MAX_READERS)
        .max_publishers(INGRESS_BUILD_MAX_WRITERS)
        .open_or_create()
        .map_err(|e| TransportError::Internal {
            reason: format!(
                "ingress-build channel: could not open the service \
                 '{INGRESS_BUILD_SERVICE_NAME}': {e:?}"
            ),
        })
}

/// The shared, thread-safe core of an [`IngressBuildChannel`] — the publisher
/// (Send+Sync via `ipc_threadsafe`, so it is shared by `Arc` between the
/// route-creation callsite AND the background republish thread with no extra
/// mutex), this process's identity and running count, and the send-failure flood
/// latch.
struct IngressBuildInner {
    publisher: Publisher<CerService, [u8], ()>,
    writer_id: u64,
    /// Runtime ingress routes created by this process so far. Monotonic.
    routes_created: AtomicU64,
    /// The channel's own monotonic origin. Every recency figure on the wire is
    /// an elapsed measured against THIS, so the field a reader receives is a
    /// delta inside one clock domain and never a stamp to be compared across
    /// two (the repo's wire-stamp rule).
    opened_at: Instant,
    /// Milliseconds since `opened_at` at the most recent route creation, or
    /// [`SINCE_LAST_CREATION_NONE`] before the first. Read by BOTH the creation
    /// call site and the republish belt, which is why it is atomic rather than
    /// held under the pump mutex.
    last_creation_ms: AtomicU64,
    /// First send failure of a run logs `warn!`, repeats `debug!`; a success
    /// re-arms. Same discipline as the registration channel's latch.
    send_warned: AtomicBool,
}

impl IngressBuildInner {
    /// Encode + loan + send THIS process's current record. Returns whether the
    /// send succeeded. A failure is flood-latched and non-fatal — the periodic
    /// republish re-attempts it, and the next route creation sends again.
    fn send(&self) -> bool {
        let bytes = encode_record(
            self.writer_id,
            self.routes_created.load(Ordering::Relaxed),
            self.since_last_creation_ms(),
        );
        match self.publisher.loan_slice_uninit(bytes.len()) {
            Ok(sample) => match sample.write_from_slice(&bytes).send() {
                Ok(_) => {
                    self.record_send_success();
                    true
                }
                Err(e) => {
                    self.record_send_failure(&e);
                    false
                }
            },
            Err(e) => {
                self.record_send_failure(&e);
                false
            }
        }
    }

    /// Report a send failure through the flood latch.
    ///
    /// `reason` is taken as a `&dyn Display` rather than a `&str` so the error
    /// is formatted LAZILY by the subscriber (the `%` sigil) instead of being
    /// rendered into a `String` at the call site. That matters here and not
    /// merely stylistically: `send` is reached from the 250 ms republish belt,
    /// so under a PERSISTENT send-failure regime a `format!` at the call site
    /// would be a heap allocation on every belt tick for as long as the regime
    /// lasts. `record_malformed` below already had this shape; these two
    /// reporters now match it, and the hot-path lint has nothing left to
    /// forgive on either. (Not per-frame either way — this channel carries a
    /// 19-byte control record and never touches a message.)
    fn record_send_failure(&self, reason: &dyn std::fmt::Display) {
        if self.send_warned.swap(true, Ordering::Relaxed) {
            tracing::debug!(
                reason = %reason,
                "ingress-build channel: progress send failed (repeat — see the first \
                 warning)"
            );
        } else {
            tracing::warn!(
                reason = %reason,
                "ingress-build channel: progress send failed — a recorder starting now \
                 may close its channel set before this process finishes opening its ingress \
                 routes, so late routes would be reported rather than recorded. The next route \
                 creation and the periodic republish both retry (repeats log at debug)"
            );
        }
    }

    /// Milliseconds since this writer last created a route, or
    /// [`SINCE_LAST_CREATION_NONE`] if it has created none.
    ///
    /// Computed AT SEND TIME, on whichever thread is sending — so a belt
    /// rebroadcast truthfully reports a growing age while a creation-triggered
    /// send reports ~0. That difference is the entire discriminator the receiver
    /// relies on, and it is why the belt cannot be a simple re-send of the
    /// previous frame's bytes.
    fn since_last_creation_ms(&self) -> u64 {
        match self.last_creation_ms.load(Ordering::Relaxed) {
            SINCE_LAST_CREATION_NONE => SINCE_LAST_CREATION_NONE,
            at => (self.opened_at.elapsed().as_millis() as u64).saturating_sub(at),
        }
    }

    /// Stamp NOW as this writer's most recent creation.
    fn note_creation(&self) {
        self.last_creation_ms.store(
            self.opened_at.elapsed().as_millis() as u64,
            Ordering::Relaxed,
        );
    }

    fn record_send_success(&self) {
        if self.send_warned.swap(false, Ordering::Relaxed) {
            tracing::info!("ingress-build channel: progress send recovered — publishing again");
        }
    }
}

/// A joined-on-drop handle for the background republish thread.
struct PumpHandle {
    exit: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

impl Drop for PumpHandle {
    fn drop(&mut self) {
        self.exit.store(true, Ordering::Relaxed);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

/// The WRITER side — one per process that creates runtime ingress
/// routes, held lazily by its [`TransportManager`](super::TransportManager).
///
/// `pump` is declared BEFORE `inner` so it drops FIRST, joining the republish
/// thread before the channel's `inner` Arc is released. (The thread holds its
/// own `inner` clone too, so the ordering is belt-and-suspenders.)
pub(crate) struct IngressBuildChannel {
    pump: Mutex<Option<PumpHandle>>,
    inner: Arc<IngressBuildInner>,
}

impl IngressBuildChannel {
    /// Open the progress service on `node` and build a fresh channel (count 0,
    /// no pump thread yet — the pump starts lazily on the first route).
    pub(crate) fn open(node: &Node<CerService>) -> TransportResult<Self> {
        let factory = open_progress_service(node)?;
        let publisher = factory
            .publisher_builder()
            .initial_max_slice_len(INGRESS_BUILD_RECORD_LEN)
            .create()
            .map_err(|e| TransportError::Internal {
                reason: format!(
                    "ingress-build channel: could not create the publisher on \
                     '{INGRESS_BUILD_SERVICE_NAME}': {e:?}"
                ),
            })?;
        Ok(Self {
            pump: Mutex::new(None),
            inner: Arc::new(IngressBuildInner {
                publisher,
                writer_id: mint_writer_id(),
                routes_created: AtomicU64::new(0),
                opened_at: Instant::now(),
                last_creation_ms: AtomicU64::new(SINCE_LAST_CREATION_NONE),
                send_warned: AtomicBool::new(false),
            }),
        })
    }

    /// Record that ONE more runtime ingress route was created: bump the running
    /// total, send it immediately (the recorder-already-listening fast path),
    /// and ensure the republish thread is running (the belt that closes the
    /// bring-up race). Returns the new total.
    pub(crate) fn note_route_created(&self) -> u64 {
        // Stamp the creation BEFORE the send, so THIS frame reports ~0 ms and is
        // self-evidently creation-triggered. A send that ran first would report
        // the PREVIOUS creation's age and the cold-boot arm would be back to
        // guessing.
        self.inner.note_creation();
        let total = self.inner.routes_created.fetch_add(1, Ordering::Relaxed) + 1;
        self.inner.send();
        self.ensure_pump_running();
        total
    }

    /// Runtime ingress routes this process has created (Principle #3: observable
    /// state).
    pub(crate) fn routes_created(&self) -> u64 {
        self.inner.routes_created.load(Ordering::Relaxed)
    }

    /// This channel's writer identity — the key a reader folds by.
    pub(crate) fn writer_id(&self) -> u64 {
        self.inner.writer_id
    }

    /// Whether the background republish thread is LIVE — spawned successfully on
    /// the first route. `false` before the first route (lazily started) and
    /// `false` if the spawn FAILED (the degraded state the spawn-failure warn
    /// describes).
    pub(crate) fn pump_active(&self) -> bool {
        self.pump
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .is_some()
    }

    /// Send arbitrary raw bytes WITHOUT recording them — the seam tests use to
    /// craft hostile / malformed frames on the wire (a well-behaved writer never
    /// does this). Returns whether the send succeeded.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn send_raw_for_test(&self, bytes: &[u8]) -> bool {
        match self.inner.publisher.loan_slice_uninit(bytes.len()) {
            Ok(sample) => sample.write_from_slice(bytes).send().is_ok(),
            Err(_) => false,
        }
    }

    /// Start the background republish thread exactly once (latched-idempotent).
    /// A spawn failure is non-fatal + loud: every route creation STILL sends its
    /// record immediately, so a recorder that is ALREADY listening sees each
    /// advance as it happens. What is lost is only the PERIODIC belt — a
    /// recorder that starts AFTER a route was created would not learn this
    /// process's total until the next route creation re-sends it. Observable via
    /// [`Self::pump_active`].
    fn ensure_pump_running(&self) {
        let mut pump = self.pump.lock().unwrap_or_else(|e| e.into_inner());
        if pump.is_some() {
            return;
        }
        let exit = Arc::new(AtomicBool::new(false));
        let exit_thread = Arc::clone(&exit);
        let inner = Arc::clone(&self.inner);
        match std::thread::Builder::new()
            // hot-path-alloc-ok: ONCE PER PROCESS. This is the latched-idempotent
            // pump spawn — the `pump.is_some()` guard four lines above returns
            // early on every later call, so the thread name is built exactly once
            // in a process's life, at its FIRST runtime ingress route. It is not
            // on any per-message path: nothing in this module ever touches a
            // sample or a payload.
            .name("cer-ingress-build".to_string())
            .spawn(move || pump_loop(&exit_thread, &inner))
        {
            Ok(join) => {
                *pump = Some(PumpHandle {
                    exit,
                    join: Some(join),
                });
            }
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "ingress-build channel: could not spawn the republish thread — each \
                     route creation still sends its record immediately (a recorder already \
                     listening sees it), but the periodic bring-up-race belt is DISABLED: a \
                     recorder that starts later may miss this process's progress until the next \
                     route is created"
                );
            }
        }
    }
}

/// The background republish thread body: until asked to exit, sleep one interval
/// (in short chunks so teardown is prompt) then re-send the current record.
/// Record-only — it publishes into SHM a recorder drains; it never touches the
/// network or the graph's execution (Principle #7 is unaffected).
fn pump_loop(exit: &AtomicBool, inner: &Arc<IngressBuildInner>) {
    // Chunk the interval sleep so a Drop-join returns within ~one chunk, not one
    // whole interval.
    const CHUNK: Duration = Duration::from_millis(25);
    while !exit.load(Ordering::Relaxed) {
        let mut slept = Duration::ZERO;
        while slept < INGRESS_BUILD_REPUBLISH_INTERVAL {
            if exit.load(Ordering::Relaxed) {
                return;
            }
            std::thread::sleep(CHUNK);
            slept += CHUNK;
        }
        if exit.load(Ordering::Relaxed) {
            return;
        }
        inner.send();
    }
}

/// The READER side — one per consumer process (the recorder). A
/// listener-less subscriber on the progress service, drained each drive pass.
/// Opened `open_or_create`, so a recorder that starts before any producer
/// CREATES the service (the producer OPENS it later).
pub struct IngressBuildReader {
    subscriber: Subscriber<CerService, [u8], ()>,
    /// Cumulative malformed records dropped at the drain (Principle #3 — a
    /// hostile / corrupt frame must be visible, never silent).
    malformed: u64,
    /// First malformed record of the reader's life logs `warn!`; repeats
    /// `debug!`.
    malformed_warned: bool,
    /// First drain receive failure logs `warn!`; repeats `debug!`.
    receive_warned: bool,
}

impl IngressBuildReader {
    /// Open the progress service on `node` and build the reader.
    ///
    /// # Errors
    ///
    /// Returns [`TransportError::Internal`] if the service or the subscriber
    /// cannot be opened.
    pub fn open(node: &Node<CerService>) -> TransportResult<Self> {
        let factory = open_progress_service(node)?;
        let subscriber =
            factory
                .subscriber_builder()
                .create()
                .map_err(|e| TransportError::Internal {
                    reason: format!(
                        "ingress-build channel: could not create the subscriber on \
                         '{INGRESS_BUILD_SERVICE_NAME}': {e:?}"
                    ),
                })?;
        Ok(Self {
            subscriber,
            malformed: 0,
            malformed_warned: false,
            receive_warned: false,
        })
    }

    /// Drain up to [`INGRESS_BUILD_QUEUE_DEPTH`] records off the queue, folding
    /// each VALID one into `progress`. A malformed record is counted +
    /// warn-once + SKIPPED (never wedges the drain); a mid-drain receive failure
    /// is warn-once and returns early (non-fatal). Never holds more than one
    /// sample borrowed. Returns the number of valid records folded.
    pub fn drain_into(&mut self, progress: &mut IngressBuildProgress) -> IngressBuildDrain {
        let mut out = IngressBuildDrain::default();
        for _ in 0..INGRESS_BUILD_QUEUE_DEPTH {
            match self.subscriber.receive() {
                Ok(Some(sample)) => match decode_record(sample.payload()) {
                    Ok(record) => {
                        progress.observe(record);
                        out.folded += 1;
                        // The FRESHEST claim wins: with several writers building
                        // at once, the machine is still building for as long as
                        // ANY of them is.
                        if let Some(age) = record.since_last_creation {
                            out.freshest_creation =
                                Some(out.freshest_creation.map_or(age, |best| best.min(age)));
                        }
                    }
                    Err(e) => self.record_malformed(&e),
                },
                Ok(None) => break,
                Err(e) => {
                    self.record_receive_failure(&e);
                    break;
                }
            }
        }
        out
    }

    /// Cumulative malformed records dropped at the drain (Principle #3).
    #[must_use]
    pub fn malformed(&self) -> u64 {
        self.malformed
    }

    fn record_malformed(&mut self, error: &IngressBuildRecordError) {
        self.malformed += 1;
        if self.malformed_warned {
            tracing::debug!(
                error = %error,
                total = self.malformed,
                "ingress-build channel: malformed record dropped (repeat)"
            );
        } else {
            self.malformed_warned = true;
            tracing::warn!(
                error = %error,
                "ingress-build channel: malformed record dropped — a frame on \
                 '{INGRESS_BUILD_SERVICE_NAME}' did not decode. Progress from that writer is not \
                 counted (repeats log at debug)"
            );
        }
    }

    /// Report a drain receive failure through the flood latch.
    ///
    /// `&dyn Display` for the same reason as `record_send_failure`, and the
    /// cadence argument is sharper on this side: `drain_into` runs once per
    /// CONSUMER drive pass (the recorder's loop), so a persistently failing
    /// subscriber would have allocated once per pass — sub-millisecond at the
    /// recorder's fastest pacing. Lazy formatting removes the allocation
    /// outright rather than justifying it.
    fn record_receive_failure(&mut self, reason: &dyn std::fmt::Display) {
        if self.receive_warned {
            tracing::debug!(
                reason = %reason,
                "ingress-build channel: drain receive failed (repeat)"
            );
        } else {
            self.receive_warned = true;
            tracing::warn!(
                reason = %reason,
                "ingress-build channel: drain receive failed — this pass observed no \
                 further progress (repeats log at debug)"
            );
        }
    }
}

/// What one drain pass observed.
///
/// `freshest_creation` is the load-bearing half and the reason this is a struct
/// rather than a count: the recorder's hold keys on RECENCY, not on the fold's
/// advance count, so the pass has to hand back the strongest creation claim it
/// heard — including from a record that changed no count at all (a belt
/// rebroadcast moments after a creation), and including the very FIRST record
/// heard from a writer, which is the whole cold-boot fix.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct IngressBuildDrain {
    /// Valid records folded this pass.
    pub folded: usize,
    /// The SMALLEST `since_last_creation` seen this pass — "somebody on this
    /// machine created a route this recently" — or `None` if no record carried
    /// one. Aged by the consumer against its own clock (see the module docs).
    pub freshest_creation: Option<Duration>,
}

/// The PURE fold of progress records into "has the machine's ingress
/// plane moved?".
///
/// # What this fold IS and IS NOT used for
///
/// It answers "how many ingress routes exist on this machine, and how much of
/// their appearance did I WITNESS?" — the `routes_observed` / `advances` pair a
/// bag's coverage manifest reports.
///
/// It is **not** what decides the recorder's hold. That keys on
/// [`IngressBuildRecord::since_last_creation`], because this fold structurally
/// cannot answer the cold-boot question: the first record from a writer is a
/// snapshot of a history nobody watched, so a count-watching receiver has
/// nothing to compare route 1 against. See the module docs.
///
/// # The BASELINE rule, and why a first sighting is not WITNESSED motion
///
/// A record is a running TOTAL, not an event. A robot whose bridge finished
/// building an hour ago republishes `routes_created = 98` forever, and counting
/// that first sighting as an advance would report motion the reader never saw.
///
/// So the first record from each `writer_id` is BANKED as that writer's baseline
/// and reports NO advance; only a strictly higher count from the same writer
/// does. This is the liveness baseline rule, for the same reason: a
/// first observation cannot distinguish "it just moved" from "it moved before I
/// arrived". Keeping the rule keeps `advances` exact — and now that the HOLD no
/// longer depends on it, being conservative here costs nothing.
///
/// A count that goes BACKWARDS for a live `writer_id` cannot happen (the writer
/// only ever increments) and is treated as no advance — the fold keeps the
/// highest seen, so a stale re-delivered record is idempotent. A RESTARTED
/// producer arrives under a NEW `writer_id` (see `mint_writer_id`), so it gets
/// its own baseline rather than appearing to rewind.
#[derive(Debug, Default, Clone)]
pub struct IngressBuildProgress {
    /// `writer_id` → highest `routes_created` seen from that writer.
    writers: std::collections::BTreeMap<u64, u64>,
    /// Sum over `writers` of the highest count seen — the machine-wide total.
    total: u64,
    /// How many times a record strictly increased a known writer's count.
    advances: u64,
}

impl IngressBuildProgress {
    /// A fresh fold that has heard nothing.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Fold ONE record in. Returns `true` iff this record is an ADVANCE — a
    /// strictly higher count from a writer already baselined. A first sighting
    /// banks the baseline and returns `false`; a repeat or a stale lower count
    /// returns `false`.
    pub fn observe(&mut self, record: IngressBuildRecord) -> bool {
        match self.writers.entry(record.writer_id) {
            std::collections::btree_map::Entry::Vacant(slot) => {
                slot.insert(record.routes_created);
                self.total = self.total.saturating_add(record.routes_created);
                false
            }
            std::collections::btree_map::Entry::Occupied(mut slot) => {
                let seen = *slot.get();
                if record.routes_created > seen {
                    self.total = self
                        .total
                        .saturating_add(record.routes_created.saturating_sub(seen));
                    slot.insert(record.routes_created);
                    self.advances = self.advances.saturating_add(1);
                    true
                } else {
                    false
                }
            }
        }
    }

    /// Machine-wide runtime ingress routes observed (the sum of every writer's
    /// highest count). Includes the baselines, so this is "how many exist", not
    /// "how many appeared while watching".
    #[must_use]
    pub fn total_routes(&self) -> u64 {
        self.total
    }

    /// How many distinct producing processes have been heard from.
    #[must_use]
    pub fn writers_seen(&self) -> usize {
        self.writers.len()
    }

    /// How many ADVANCES have been observed (baselines excluded) — the
    /// Principle #3 observable behind the recorder's hold.
    #[must_use]
    pub fn advances(&self) -> u64 {
        self.advances
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ingress_build_service_names_have_no_data_suffix() {
        // The invisibility argument in the module docs is enforced here: topic
        // surfaces derive topics from `*/data` services, so a `/data` suffix
        // would make this control channel show up as a robot topic AND make
        // `cerulion_bagd`'s live enumeration tap it — which would be a recorder
        // reading its own progress signal as data.
        assert!(!INGRESS_BUILD_SERVICE_NAME.ends_with("/data"));
        assert!(
            INGRESS_BUILD_SERVICE_NAME.starts_with(super::super::gateway::RESERVED_TOPIC_PREFIX)
        );
    }

    #[test]
    fn encode_decode_round_trips_against_a_hand_built_frame() {
        // Hand oracle: the exact 27 bytes, little-endian, not a self-compare.
        let bytes = encode_record(0x0102_0304_0506_0708, 0x1112_1314_1516_1718, 250);
        assert_eq!(bytes.len(), INGRESS_BUILD_RECORD_LEN);
        assert_eq!(&bytes[0..3], &[0xCE, 0xDB, INGRESS_BUILD_VERSION]);
        assert_eq!(&bytes[3..11], &[8, 7, 6, 5, 4, 3, 2, 1]);
        assert_eq!(
            &bytes[11..19],
            &[0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11]
        );
        assert_eq!(&bytes[19..27], &[250, 0, 0, 0, 0, 0, 0, 0]);
        let decoded = decode_record(&bytes).expect("round trip");
        assert_eq!(decoded.writer_id, 0x0102_0304_0506_0708);
        assert_eq!(decoded.routes_created, 0x1112_1314_1516_1718);
        assert_eq!(
            decoded.since_last_creation,
            Some(Duration::from_millis(250))
        );
    }

    #[test]
    fn the_never_created_sentinel_decodes_to_no_claim_not_to_zero() {
        // `0 ms ago` and `never` must not collapse: the first arms a recorder's
        // hold, the second is the absence of any creation to speak of. A
        // sentinel decoded as `Some(0)` would be a fabricated "just built" claim
        // from a writer that has built nothing.
        let bytes = encode_record(1, 0, SINCE_LAST_CREATION_NONE);
        assert_eq!(
            decode_record(&bytes).expect("decode").since_last_creation,
            None
        );
        let just_now = encode_record(1, 1, 0);
        assert_eq!(
            decode_record(&just_now)
                .expect("decode")
                .since_last_creation,
            Some(Duration::ZERO)
        );
    }

    #[test]
    fn a_version_one_record_is_rejected_rather_than_decoded_on_a_default() {
        // A v1 record carried no recency. Accepting one by defaulting the field
        // would mean inventing a claim about when that writer last built
        // something — the exact fabrication this channel exists to replace — so
        // the version gate refuses it.
        let mut v1 = encode_record(1, 5, 0);
        v1[2] = 1;
        assert_eq!(
            decode_record(&v1),
            Err(IngressBuildRecordError::BadVersion { got: 1 })
        );
    }

    #[test]
    fn decode_rejects_every_malformed_shape() {
        assert_eq!(
            decode_record(&[]),
            Err(IngressBuildRecordError::WrongLength { actual: 0 })
        );
        let good = encode_record(1, 1, 0);
        assert_eq!(
            decode_record(&good[..INGRESS_BUILD_RECORD_LEN - 1]),
            Err(IngressBuildRecordError::WrongLength {
                actual: INGRESS_BUILD_RECORD_LEN - 1
            })
        );
        // hot-path-alloc-ok: TEST-ONLY. Inside `#[cfg(test)] mod tests`; this
        // builds an over-long frame to pin the decoder's length refusal and is
        // compiled out of every shipping binary.
        let mut long = good.to_vec();
        long.push(0);
        assert_eq!(
            decode_record(&long),
            Err(IngressBuildRecordError::WrongLength {
                actual: INGRESS_BUILD_RECORD_LEN + 1
            })
        );
        let mut bad_magic = good;
        bad_magic[1] ^= 0xFF;
        assert_eq!(
            decode_record(&bad_magic),
            Err(IngressBuildRecordError::BadMagic)
        );
        let mut bad_version = good;
        bad_version[2] = INGRESS_BUILD_VERSION + 1;
        assert_eq!(
            decode_record(&bad_version),
            Err(IngressBuildRecordError::BadVersion {
                got: INGRESS_BUILD_VERSION + 1
            })
        );
    }

    #[test]
    fn a_first_sighting_is_a_baseline_and_reports_no_advance() {
        // THE rule. A robot whose plane finished building long ago republishes a
        // large constant forever; if that first sighting counted as motion,
        // every recording started against a settled machine would hold its
        // channel set open for nothing.
        let mut p = IngressBuildProgress::new();
        assert!(!p.observe(IngressBuildRecord {
            writer_id: 7,
            routes_created: 98,
            since_last_creation: None,
        }));
        assert_eq!(
            p.total_routes(),
            98,
            "the baseline still counts as EXISTING"
        );
        assert_eq!(p.advances(), 0, "but it is not MOTION");
        // Republishes of the same total are idempotent.
        for _ in 0..5 {
            assert!(!p.observe(IngressBuildRecord {
                writer_id: 7,
                routes_created: 98,
                since_last_creation: None,
            }));
        }
        assert_eq!(p.total_routes(), 98);
        assert_eq!(p.advances(), 0);
    }

    #[test]
    fn a_strictly_higher_count_from_a_baselined_writer_is_an_advance() {
        let mut p = IngressBuildProgress::new();
        assert!(!p.observe(IngressBuildRecord {
            writer_id: 7,
            routes_created: 1,
            since_last_creation: None,
        }));
        assert!(p.observe(IngressBuildRecord {
            writer_id: 7,
            routes_created: 2,
            since_last_creation: None,
        }));
        assert!(p.observe(IngressBuildRecord {
            writer_id: 7,
            routes_created: 40,
            since_last_creation: None,
        }));
        assert_eq!(p.total_routes(), 40);
        assert_eq!(p.advances(), 2);
        assert_eq!(p.writers_seen(), 1);
    }

    #[test]
    fn a_stale_or_repeated_record_never_lowers_the_total_or_counts_as_motion() {
        // `drop_oldest` overflow and periodic republish both re-deliver records
        // out of order; the fold must be idempotent under both.
        let mut p = IngressBuildProgress::new();
        p.observe(IngressBuildRecord {
            writer_id: 7,
            routes_created: 1,
            since_last_creation: None,
        });
        p.observe(IngressBuildRecord {
            writer_id: 7,
            routes_created: 50,
            since_last_creation: None,
        });
        assert!(!p.observe(IngressBuildRecord {
            writer_id: 7,
            routes_created: 20,
            since_last_creation: None,
        }));
        assert_eq!(p.total_routes(), 50);
        assert_eq!(p.advances(), 1);
    }

    #[test]
    fn each_writer_gets_its_own_baseline_and_the_totals_sum() {
        // A multi-process graph: every worker that opens ingress routes writes
        // its own record, and a restarted producer arrives under a NEW id so it
        // never looks like a rewind.
        let mut p = IngressBuildProgress::new();
        assert!(!p.observe(IngressBuildRecord {
            writer_id: 1,
            routes_created: 10,
            since_last_creation: None,
        }));
        assert!(!p.observe(IngressBuildRecord {
            writer_id: 2,
            routes_created: 5,
            since_last_creation: None,
        }));
        assert_eq!(p.total_routes(), 15);
        assert_eq!(p.writers_seen(), 2);
        assert_eq!(p.advances(), 0, "two baselines are still not motion");
        assert!(p.observe(IngressBuildRecord {
            writer_id: 2,
            routes_created: 6,
            since_last_creation: None,
        }));
        assert_eq!(p.total_routes(), 16);
        assert_eq!(p.advances(), 1);
    }

    #[test]
    fn minted_writer_ids_are_distinct_within_a_process() {
        let a = mint_writer_id();
        let b = mint_writer_id();
        assert_ne!(a, b, "two channels in one process must not fold into one");
    }
}
