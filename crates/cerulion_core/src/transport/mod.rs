// SPDX-License-Identifier: AGPL-3.0-only
//! Singleton TransportManager owning the iceoryx2 Node.
//!
//! Runtime internals, not node-author API. The graph runtime creates one
//! publisher or subscriber per port from the graph file and hands them to
//! the node; inside `tick` a node reads and writes `self.<port>` and never
//! sees a `TransportManager`, a publisher or a subscriber.
//!
//! # Design
//!
//! One iceoryx2 `Node` per process (Principle #8). The `TransportManager` owns this
//! node and provides factory methods for creating publishers and subscribers.
//!
//! # Two Services Per Topic
//!
//! Each topic creates two iceoryx2 services:
//! - `{topic}/data` — `publish_subscribe::<[u8]>()` for message data
//! - `{topic}/event` — `event()` for bidirectional notification signaling
//!
//! Both publisher and subscriber create Notifier + Listener on the same event
//! service. The `PubSubEvent` discriminant carries direction.
//!
//! # Service Name Limits
//!
//! iceoryx2 limits service names to 255 characters. Since we append `/data` (5 chars)
//! and `/event` (6 chars), topic names are limited to 249 characters —
//! INCLUDING the canonical leading slash, so the
//! `{prefix}/{node}/{output}` body has 248 usable characters.

pub mod adaptive_sizer;
pub mod bridge;
pub mod cerulion_q;
pub mod dead_node_sweep;
pub mod demand_authorizer;
pub mod discovery;
pub mod events;
pub mod failure_regime_latch;
pub mod fill_from;
pub mod frame_drop_latch;
pub mod gateway;
pub mod ingress_build;
pub mod input_view;
pub mod liveness;
pub mod mirror_registry;
pub mod network;
pub mod notify_delivery_latch;
pub mod output_discard_latch;
pub mod output_proxy;
pub mod publisher;
pub mod reg_channel;
pub mod run_artifacts;
pub mod run_registry;
pub mod service;
pub(crate) mod shm_guard;
mod shm_sample;
pub mod subscriber;
pub mod uds_write;

use std::sync::{Arc, OnceLock};
// Only the test-only `LivelinessCleaner` invocation
// counter uses these — cfg-gate the import so production builds stay free of
// the atomic AND of an unused-import error.
#[cfg(any(test, feature = "test-helpers"))]
use std::sync::atomic::{AtomicU64, Ordering};

use iceoryx2::node::Node;
use iceoryx2::prelude::*;

// THE transport Service type. ipc_threadsafe::Service = Send+Sync
// ports (MutexProtected), so NodeContext/DylibNodeEntry/rmw port-structs are
// Send-by-construction (none of them needs a manual
// `unsafe impl Send/Sync`).
pub(crate) use iceoryx2::service::ipc_threadsafe::Service as CerService;

use crate::clock::{Clock, RealClock};
use crate::error::{NodeNameRefusal, TransportError, TransportResult};
use crate::message::ShmMessage;
use crate::wire::{MaxSliceLen, WireHeader};
use bridge::TopicBridgeManager;
use network::{IngressInjector, NetworkConfig, NetworkManager};
use publisher::{CerulionPublisher, CerulionPublisherConfig};
use subscriber::{CerulionSubscriber, DataOnlySubscriber};

/// Maximum topic name length (255 - "/event" suffix = 249).
const MAX_TOPIC_LEN: usize = 249;

/// Maximum concurrent clients (publishers) on a service REQUEST topic.
///
/// Every client of a service publishes to the single shared
/// `svc/{service}/request` topic, so iceoryx2's default `max_publishers`
/// (2) fails the THIRD client's `PublisherCreation` (MoveIt hits
/// `/get_planning_scene` from rviz + move_group + planners). 16 covers
/// MoveIt-scale fan-in with headroom. Applied ONLY to request topics —
/// graph topics keep iceoryx2's single-writer-friendly defaults and reply
/// topics are single-server-writer (see
/// [`TopicServiceConfig::for_service_request`] /
/// [`TopicServiceConfig::for_service_reply`]).
const MAX_SERVICE_REQUEST_CLIENTS: usize = 16;

/// Default subscriber buffer size (number of samples).
const DEFAULT_SUBSCRIBER_BUFFER_SIZE: usize = 16;

/// The iceoryx2 0.9.1 service default for
/// `subscriber_max_borrowed_samples` (the per-connection sample-borrow
/// ceiling). Pinned because [`SUBSCRIBER_MAX_BORROWED_HELD`] budgets exactly
/// one above it. The drift-guard test
/// `non_trigger_hold_iox2_test::iceoryx2_default_borrowed_samples_is_two`
/// reads the value back off a freshly-created default service, so an upstream
/// iceoryx2 default bump fails loudly instead of silently under-provisioning
/// the held-sample headroom.
pub(crate) const ICEORYX2_DEFAULT_BORROWED_SAMPLES: usize = 2;

/// `subscriber_max_borrowed_samples` provisioned on a topic that
/// feeds a non-trigger latest-value input of a macro snapshot consumer.
/// Such a consumer HOLDS the last-delivered sample across steps, so its
/// per-connection borrow peak during a multi-sample burst is held(1) +
/// latest(1) + receive-transient(1) = 3 — one above the iceoryx2 default
/// of 2. Defined as `default + 1` so the "held + latest + transient =
/// default + 1" rationale is self-documenting (and tracks the default if it
/// ever changes — guarded by the drift-guard test). Set ONLY on
/// snapshot-source topics (see
/// [`TopicServiceConfig::subscriber_max_borrowed_samples`]).
pub(crate) const SUBSCRIBER_MAX_BORROWED_HELD: usize = ICEORYX2_DEFAULT_BORROWED_SAMPLES + 1;

/// Drift-guard hook (test-only): iceoryx2's LIVE create-default
/// `max_subscribers` for a publish-subscribe service — the subscriber cap a
/// service gets when it is created with NO explicit subscriber requirement,
/// which is exactly what a consumer-first worker's create-or-open leg mints
/// (the under-provisioned service a later producer worker dies opening).
///
/// Read from the SAME resolved config every default-namespace
/// [`TransportManager`] node is built from (`Config::global_config()`, which
/// falls back to iceoryx2's compiled-in default only when no config file is
/// found), so BOTH drift classes surface: an iceoryx2 upgrade changing the
/// compiled-in default, AND a local/CI config file overriding it.
///
/// Tests that must state the number as a literal — `cerulion_cli`'s
/// `mp_consumer_first_spawn_e2e_test`, whose entire discriminator is a CROSSING
/// of this cap — assert their mirror against this accessor, so a stale mirror
/// fails LOUDLY instead of quietly measuring something else. There is no public
/// iceoryx2 constant for it, hence the accessor rather than a `const`.
#[cfg(any(test, feature = "test-helpers"))]
#[must_use]
pub fn iceoryx2_default_max_subscribers() -> usize {
    iceoryx2::config::Config::global_config()
        .defaults
        .publish_subscribe
        .max_subscribers
}

/// The per-tap held-BATCH budget — how many owned samples one
/// recorded tap accumulates before an at-budget flush hands the batch off to be
/// written. This is the pre-writer-thread held budget (`8 - 1 = 7`), so the flush
/// cadence — hence the bag's chunk boundaries — is UNCHANGED by the writer-thread
/// pivot (a quiet, no-stall recording produces byte-identical chunks). bagd caps
/// its per-tap `held` Vec at this value regardless of the (larger) provisioned
/// borrow ceiling.
pub const RECORDING_TAP_HELD_BUDGET: usize = 7;

/// Peak concurrent in-flight held-batches per recorded tap once the
/// drain is decoupled from the writev. During a writer stall a tap's borrowed
/// SHM samples span up to THREE batches AT ONCE: the drain thread's batch being
/// filled (1) + the bounded hand-off channel (bound
/// [`RECORDING_WRITER_CHANNEL_BOUND`] = 1) + the writer thread's batch being
/// `writev`'d (1). Every [`OwnedInboundSample`](crate::transport::subscriber::OwnedInboundSample)
/// in all three pins one unit of the tap subscriber's borrow budget
/// simultaneously — hence the ceiling below is sized for their sum.
pub const RECORDING_WRITER_INFLIGHT_BATCHES: usize = 3;

/// The bounded drain→writer hand-off channel capacity (in whole
/// flush-batches). One queued batch plus the writer's in-flight batch lets the
/// writer fall a full flush behind a transient disk stall without the drain
/// blocking; a deeper channel would only raise the borrow ceiling (each queued
/// batch pins live SHM borrows) with no headroom the deep receive queue
/// does not already provide.
///
/// Defer-not-drop: kept at 1 deliberately. When this channel
/// fills, the drain does not DROP the batch — it RETAINS the frames in their
/// taps and retries next cycle (see `Recorder::flush_threaded` in cerulion_bagd),
/// so the deep receive queue (not this channel) is the real burst
/// absorber. Raising this bound would only add borrow-budget / apparent-SHM cost
/// (`RECORDING_SUBSCRIBER_MAX_BORROWED` scales with it) for no absorption the
/// receive queue does not already provide — measured on an x86 machine (humanoid @1MB,
/// 65 MB/s, 199 GB bag), defer-not-drop takes `dropped_unwritten` from ~2% of
/// frames to 0 with this bound unchanged.
pub const RECORDING_WRITER_CHANNEL_BOUND: usize = RECORDING_WRITER_INFLIGHT_BATCHES - 2;

/// The `subscriber_max_borrowed_samples` a topic is raised to
/// when it is RECORDED (`graph run --record`). The `bagd` recorder taps each
/// recorded topic as an EXTRA subscriber.
///
/// # The derivation, and why it does not describe the recorder
///
/// The arithmetic is [`RECORDING_TAP_HELD_BUDGET`] ×
/// [`RECORDING_WRITER_INFLIGHT_BATCHES`] + 1 = `7×3 + 1 = 22`.
/// It assumes a staged frame
/// stays a BORROWED shared-memory sample until its disk write finishes, so a
/// tap's live borrows span every concurrently in-flight batch at once, plus
/// one reserved for the `receive` in progress.
///
/// **That premise does not hold.** The recorder copies each payload into the
/// MCAP chunk arena at drain time and drops the sample inside the same loop, so
/// at most ONE borrow is live per tap at any instant, whatever the writer is
/// doing. The recorder is therefore affordable at any budget `>= 1` — which is
/// the point: the 71 `ros2 attach` bridge routes on a Go2 are provisioned by
/// their OWN producers at the stock 2, and no recording-side raise can
/// reach them.
///
/// The VALUE is deliberately 22 rather than what the recorder
/// needs, because the constant buys the recorder nothing and
/// shrinking it is not free of consequence: it is applied at SERVICE CREATION by
/// `graph run --record`, so it also sets the ceiling every OTHER subscriber of
/// that topic may request, and the pool slots it reserves are lazily
/// demand-paged so an unused raise costs no resident memory.
///
/// # Right-sizing was MEASURED, and the measurement said don't
///
/// Right-sizing is behaviour-visible, so it was measured
/// rather than argued, via `firehose_bench`'s `FH_BORROW` knob — which
/// sweeps the PRODUCER's `subscriber_max_borrowed_samples`, i.e. exactly what
/// this constant sets, without mutating the constant. At the bench's
/// discriminating shape (64 topics x 3000 Hz x 64 B = 17.5 MB/s), macOS desk,
/// three reps per budget, loss read as the conservation identity
/// (published - recorded):
///
/// | budget | rep 1 | rep 2 | rep 3 |
/// |--------|-------|-------|-------|
/// | 22     | 1.37% | 3.70% | 3.12% |
/// | 8      | 6.21% | 3.97% | 1.86% |
/// | 4      | 0.20% | 19.50%| 1.05% |
/// | 2      | 0.16% | 0.19% | 0.00% |
///
/// The ranges OVERLAP completely and the ordering is not even monotone — the
/// stock budget of 2 posted the best three runs of the sweep. The correct
/// reading is NOT "smaller is better"; it is that **the borrow budget is not
/// the binding term at this shape**, and that this shape sits at the desk's
/// saturation KNEE (past saturation, batching does not
/// help), which is precisely where a measurement is least able to resolve a
/// second-order term. A single run can show 32% at budget 2 and 0% at 22;
/// the repetitions show that is ambient load, not the budget. Nothing here
/// supports a reduction, so the value stays.
///
/// The bench also cannot see the reason that actually pins the FLOOR, because
/// it runs no consumers: this value is the ceiling every OTHER subscriber may
/// request, and a non-trigger latest-value (snapshot) input needs
/// `subscriber_max_borrowed_samples >= 3`. So 2 is unsafe outright on a
/// recorded topic a snapshot consumer reads, and 4 leaves exactly one slot
/// above that floor for the recorder's tap plus every introspection consumer.
///
/// To re-measure: run it on an idle machine, at a rate well INSIDE the
/// recorder's capability rather than at the knee, and with the floor as
/// the hard lower bound rather than 1.
///
/// Applied ONLY under `--record` (the zero-behavior-change firewall) — a
/// non-recording run never raises it, and because the recorder copies at drain
/// time, a tap whose service was NOT recording-provisioned records exactly like
/// one that was.
pub const RECORDING_SUBSCRIBER_MAX_BORROWED: usize =
    RECORDING_TAP_HELD_BUDGET * RECORDING_WRITER_INFLIGHT_BATCHES + 1;

/// The target receive-queue depth (`subscriber_max_buffer_size`) a
/// RECORDED topic's service ceiling is raised to under `graph run --record`, for
/// a small (typical control-loop) payload.
///
/// # The bug this fixes
///
/// bagd taps each recorded topic OPEN-ONLY, so its subscriber port inherits the
/// SERVICE ceiling as its queue depth (a `buffer_size: None` port resolves to
/// the service's `subscriber_max_buffer_size` — see
/// [`TransportManager::create_subscriber_open_only`]). At
/// the stock ceiling `DEFAULT_SUBSCRIBER_BUFFER_SIZE` (16), a transient bagd drain stall lets the
/// producer outrun the tap's 16-deep queue; iceoryx2 reclaims (drop-oldest) the
/// unread head BEFORE bagd's drain sees it. The frames vanish and bagd's
/// `dropped_unwritten` counter (which only counts post-pickup losses) stays 0,
/// so the bag finalizes "healthy" while silently missing frames. Measured:
/// an 8-core Jetson loses 1-6 frames on a single 200 Hz topic and
/// enough on a 7×~1 kHz-topic graph that replay cannot produce a verdict.
///
/// A 4096-deep tap queue turns a 16-frame stall budget into ~4096 frames — at
/// 1 kHz that is ~4 s of slack, at 200 Hz ~20 s — swamping any realistic
/// per-flush drain hiccup. (4096 rather than 1024:
/// only ~1 s of absorption at 1 kHz is too tight for
/// disk-stall-prone recorders like the Jetson.)
///
/// # SHM cost (justified)
///
/// iceoryx2 sizes a publisher's data-segment pool from the SERVICE ceiling ×
/// `max_subscribers` (each slot is a full `max_slice_len` bucket), so raising the
/// ceiling multiplies the APPARENT reservation. But iceoryx2 `Static` pools are
/// lazily demand-paged on Linux (reference: resident ≈ working set; the apparent
/// reservation is virtual address space, not RAM), so a deep queue costs almost
/// nothing resident on a healthy recording — only the frames actually in flight
/// fault in. The genuine cost is virtual-address-space / `RLIMIT_AS` pressure on
/// huge-`max_slice_len` topics, which [`recording_tap_buffer_depth`] bounds with
/// a per-topic scaling rule (see there).
///
/// # This depth is the `--record` HALF of a split contract
///
/// Everything above is about an EXPLICIT `graph run --record`, and there it
/// stands unchanged: a replay-grade recording is lossless-by-provisioning, and
/// this ceiling is what makes that true.
///
/// The always-on **window-only** Flashback recorder does NOT take this depth. It
/// runs on every serving graph whether or not anybody asked to record, so the
/// cost it has to bound is a STANDING one nobody opted into, and it opens its
/// taps through [`flashback_tap_buffer_depth`] instead — a byte budget over the
/// REAL slot size, floored at 2 and capped by the topic's own ceiling. What it
/// gives up is absorbance, and it gives it up out loud: the budget is a declared
/// knob and the resident cost is reported as `shm_pinned_bytes`.
///
/// The two are different recorders with different promises, and the choice is a
/// structural mode gate rather than a caller's discipline (see
/// [`TransportManager::create_data_only_subscriber_budgeted`]).
pub const RECORDING_TAP_BUFFER_DEPTH: usize = 4096;

/// The FLOOR of the per-topic scaled recording tap-queue depth
/// ([`recording_tap_buffer_depth`]). Even the largest-payload recorded topic
/// keeps at least this much tap queue so a realistic drain stall never silently
/// drops frames. 64 frames ≈ 64 ms at 1 kHz / 320 ms at 200 Hz — comfortably
/// above a per-flush hiccup (`DEFAULT_FLUSH_INTERVAL_MS` = 100 ms is the bagd
/// flush cadence, but a stall is a small multiple of that at worst).
pub const RECORDING_TAP_BUFFER_DEPTH_FLOOR: usize = 64;

/// The per-topic APPARENT-SHM budget the recording tap-depth scaling
/// rule holds the ceiling raise to — depth is scaled so `depth × max_slice_len ≤`
/// this (per subscriber slot; multiply by `max_subscribers` for the whole pool).
/// 1 GiB keeps a normal (≤ 256 KiB) topic at the full [`RECORDING_TAP_BUFFER_DEPTH`]
/// while scaling a multi-MiB camera topic down.
///
/// Raised from 256 MiB to 1 GiB (paired with the 1024 → 4096
/// [`RECORDING_TAP_BUFFER_DEPTH`] cap raise) to buy ~4× more drain-stall
/// absorption. The apparent-size cost is acceptable because iceoryx2 `Static`
/// SHM pools are LAZY / demand-paged on Linux AND macOS (measured:
/// resident ≈ working set — a 54.8 GiB apparent multi-pool reservation faulted
/// in at 0.84 MiB resident on a 15 GiB Jetson). So even a 1 GiB apparent tap
/// slot costs ~0 real RAM on a healthy recording; only frames actually queued
/// fault in. Because pools are lazily demand-paged this is a VIRTUAL-footprint /
/// `RLIMIT_AS` guard, not a RAM guard.
const RECORDING_TAP_APPARENT_BUDGET_BYTES: usize = 1024 * 1024 * 1024;

/// The recording tap queue depth (== the raised service ceiling) for a
/// recorded topic whose publisher loans `max_slice_len`-byte slots.
///
/// # Scaling rule (per-topic, huge-slice-aware)
///
/// A flat 4096-deep ceiling on a 2 MiB camera topic reserves 4096 × 2 MiB =
/// 8 GiB apparent PER SUBSCRIBER SLOT (× `max_subscribers` for the pool). Lazily
/// paged, that is near-free in RAM — but it is real virtual address space, so
/// this scales the depth DOWN for large slices while never dropping below
/// [`RECORDING_TAP_BUFFER_DEPTH_FLOOR`]:
///
/// ```text
/// depth = clamp( RECORDING_TAP_APPARENT_BUDGET_BYTES / max_slice_len,
///                FLOOR = 64, TARGET = 4096 )
/// ```
///
/// Worked examples (apparent bytes = depth × max_slice_len, PER subscriber slot;
/// the pool is this × `max_subscribers` = in-graph subscribers + introspection
/// headroom):
///
/// | max_slice_len | depth | apparent/slot | note                              |
/// |---------------|-------|---------------|-----------------------------------|
/// | 256 B (IMU)   | 4096  | 1 MiB         | full target — trivially free      |
/// | 64 KiB        | 4096  | 256 MiB       | full target                       |
/// | 256 KiB       | 4096  | 1 GiB         | the budget knee                   |
/// | 1 MiB         | 1024  | 1 GiB         | scaled to the budget              |
/// | 2 MiB (1080p) | 512   | 1 GiB         | scaled to the budget              |
/// | 4 MiB         | 256   | 1 GiB         | scaled to the budget              |
/// | 16 MiB (4K)   | 64    | 1 GiB         | at the floor (budget knee)        |
/// | 128 MiB (tier-3 default) | 64 | 8 GiB | floor governs — prefer explicit `max_slice_len:` |
///
/// The floor deliberately WINS over the budget for very large slices (> 16 MiB):
/// such topics are inherently low-rate (you do not publish 16 MiB frames at
/// 1 kHz), so 64 frames of queue is generous in wall-time, and the apparent cost
/// stays lazily-paged virtual memory. On a 64-bit host (128 TiB of address
/// space) even the 8 GiB pathological case is fine; a misconfigured tier-3
/// (128 MiB) default topic should set an explicit `max_slice_len:` regardless
/// (the recorded-tap raise loudly warns on the 128 MiB default — see the
/// floored-depth warn in `GraphRuntime`'s recorded-topics raise).
pub fn recording_tap_buffer_depth(max_slice_len: usize) -> usize {
    if max_slice_len == 0 {
        // Defensive: MaxSliceLen is always >= WireHeader::SIZE, so this is
        // unreachable from real topics — full target rather than a div-by-zero.
        return RECORDING_TAP_BUFFER_DEPTH;
    }
    let by_budget = RECORDING_TAP_APPARENT_BUDGET_BYTES / max_slice_len;
    by_budget.clamp(RECORDING_TAP_BUFFER_DEPTH_FLOOR, RECORDING_TAP_BUFFER_DEPTH)
}

/// The CAP of the per-route ingress receive-queue depth
/// ([`ingress_route_buffer_depth`]).
///
/// 1024 frames is ~0.6 s of absorption for a 1.66 kHz stream and ~2 s for a
/// 500 Hz one — far past any drain stall a recorder or a desk consumer can
/// plausibly have, so the per-route BUDGET below is what governs for every
/// realistic slice size and this is a backstop, not the operating point.
pub const INGRESS_ROUTE_BUFFER_DEPTH: usize = 1024;

/// The per-route APPARENT-SHM budget the ingress depth scaling holds
/// `depth × max_slice_len` to, per subscriber slot.
///
/// # Where 64 MiB comes from — MEASURED, not chosen for roundness
///
/// A `ros2 attach` route's slice is `dds_bridge`'s `DEFAULT_RAW_MAX_SLICE_LEN` =
/// 1 MiB (the generated bridge config never emits `max_slice_len:`), so this
/// budget puts a bridge route at **depth 64** = 38.6 ms of absorption at
/// 1.66 kHz, and that is sized directly off the measured Go2 rate.
///
/// The robot's EFFECTIVE drain gap follows from its own numbers with no model
/// fitting at all. For a bimodal gap distribution the loss fraction is
/// `1 - D/(R*G)` — the long-gap FREQUENCY cancels — so its 70.9 % at `R` =
/// 1.66 kHz and `D` = 16 inverts to `G = 16 / (1660 * 0.291)` = **33.1 ms**.
/// 64 frames covers that whole 33.1 ms; the stock 16 covers 9.6 ms and loses
/// 71 %. (Cross-check on the same arithmetic: this desk's bench reads 20.2 %,
/// giving `G` = 12.1 ms = one 10 ms idle tick plus ~2 ms of pass work, which is
/// what its histogram independently shows.)
///
/// Note what is deliberately NOT claimed. At least 10 ms of the robot's 33.1 ms
/// is `bagd`'s idle tick, which the recorder-side pacing fix removes, but the
/// COMPOSITION of the remaining ~23 ms is not decomposable from off-robot data,
/// so this budget is sized against the FULL measured gap rather than against a
/// residual that would have to be inferred. Depth 64 therefore closes the
/// robot's gap on its own (1.16x), and the pacing fix is additional margin
/// (>= 1.67x) rather than a load-bearing assumption.
///
/// # The apparent cost, and why it is affordable
///
/// iceoryx2 sizes a publisher pool as
/// `max_subscribers × (buffer + borrowed) + history + max_loaned` slots of
/// `max_slice_len` each. Because this rule holds `depth × slice` roughly
/// CONSTANT, the per-route pool is ~`8 × 64 MiB` = ~530 MiB whatever the slice
/// size, so the TOTAL scales with ROUTE COUNT alone: ~52 GiB apparent at the
/// Go2's 102 routes. That sits just inside a measured envelope:
/// a 54.8 GiB apparent multi-pool reservation
/// faulted in at 0.84 MiB resident on a 15 GiB Jetson, because `Static` pools
/// are lazily demand-paged. It is an `RLIMIT_AS` / address-space cost, not RAM.
///
/// # The cost that is NOT free, stated plainly
///
/// RESIDENT memory grows with OCCUPANCY, and a saturated depth-64 route at a
/// 1 MiB slice faults in 64 MiB — against 16 MiB at the stock depth. A queue only saturates
/// while the recorder is stalled, and it drains immediately after, so this is a
/// transient; but on a robot with ~100 routes all stalling at once it is real.
///
/// The durable remedy for that is right-sizing `max_slice_len` per route: a 700-byte
/// Odometry frame occupies a 1 MiB slot by default, wasting 1500x. This rule is
/// deliberately SLICE-AWARE so that right-sizing is REWARDED rather than merely
/// permitted — a route that declares a 4 KiB slice gets the full 1024-frame cap
/// (0.6 s of absorption at 1.66 kHz) for 33 MiB of apparent pool, i.e. a deeper
/// queue AND 16x less memory than the 1 MiB default slice costs.
const INGRESS_ROUTE_APPARENT_BUDGET_BYTES: usize = 64 * 1024 * 1024;

/// The receive-queue depth an INGRESS route's iceoryx2 service is
/// created with — the ceiling every consumer of that route inherits, including
/// the recorder's `create_data_only_subscriber` tap, which passes
/// `buffer_size: None`.
///
/// # Why ingress routes need their own rule
///
/// A bridge route is a producer of a FOREIGN stream at a rate nobody declared,
/// and `DEFAULT_SUBSCRIBER_BUFFER_SIZE` (16) is a general-purpose
/// default, never a considered choice for this shape. A DECLARED graph topic
/// under `--record` gets up to 4096 via [`recording_tap_buffer_depth`];
/// an ingress route at 16, measured on a Go2, cost two 1.66 kHz Point-LIO
/// outputs **70.9 %** and **70.8 %** of their frames while a 700 Hz topic on the
/// same recorder lost 0.13 % — the entire difference being that 16 frames
/// absorbs 9.6 ms at one rate and 22.9 ms at the other.
///
/// The depth CANNOT be fixed later: iceoryx2 pins a service's
/// `subscriber_max_buffer_size` at CREATE, and an opener asking for more is
/// rejected (`DoesNotSupportRequestedMinBufferSize`) while one asking for less
/// silently inherits the existing ceiling. So the only place this can be got
/// right is here, before the first frame.
///
/// # The rule
///
/// ```text
/// depth = clamp( INGRESS_ROUTE_APPARENT_BUDGET_BYTES / max_slice_len,
///                FLOOR = DEFAULT_SUBSCRIBER_BUFFER_SIZE, CAP = 1024 )
/// ```
///
/// It is RATE-INDEPENDENT on purpose. The obvious alternative — provision from
/// the route's observed arrival rate — cannot be built: the bridge has no rate
/// at create time (Cerulion's SEDP summary carries reliability and durability
/// only, never DEADLINE or any period), it measures none at runtime, and by the
/// time one could be measured the service is already created and immutable.
/// Sizing by SLICE instead bounds the SHM cost directly, which is the constraint
/// that actually binds at 100+ routes.
///
/// | max_slice_len | depth | apparent/slot | pool/route | absorbs @1.66 kHz |
/// |---|---|---|---|---|
/// | 256 B | 1024 (cap) | 256 KiB | 2 MiB | 617 ms |
/// | 4 KiB | 1024 (cap) | 4 MiB | 33 MiB | 617 ms |
/// | 64 KiB | 1024 (cap) | 64 MiB | 528 MiB | 617 ms |
/// | 1 MiB (bridge default) | 64 | 64 MiB | 530 MiB | 38.6 ms |
/// | 4 MiB | 16 (floor) | 64 MiB | 576 MiB | 9.6 ms |
/// | 16 MiB | 16 (floor) | 256 MiB | 2.3 GiB | 9.6 ms |
///
/// The FLOOR is the stock ceiling, so this rule can only ever RAISE a route's
/// depth — a huge-slice route stays at the stock depth rather than
/// being made worse. Such topics are also inherently low-rate (nobody publishes
/// 16 MiB frames at 1.66 kHz), so 16 frames is generous in wall time for them.
///
/// # What this deliberately does NOT touch, and the residual that leaves
///
/// `create_ingress_publisher` has two production callers, and the floor scopes
/// the raise to one of them:
///
/// * the `ros2 attach` **bridge route**, at a 1 MiB slice → raised 16 → **64**.
///   This is the class this rule exists for.
/// * `cerulion-netd`'s **desk mirror**, whose `MIRROR_MAX_SLICE_LEN` is derived
///   from `graph::config::DEFAULT_MAX_SLICE_LEN` = **128 MiB** → the budget
///   cannot buy a single frame past the floor, so a mirror is provisioned
///   at the stock depth.
///
/// That second row is a real residual, not a free pass: a desk mirroring a kHz
/// robot topic keeps a 16-frame queue and can overflow it the same way. This
/// rule does not address it (it targets the recorder on the robot), and it is
/// NOT fixable by widening the budget — 128 MiB × 64 frames is 8 GiB per mirror
/// per subscriber slot. The remedy there is a smaller default mirror slot
/// (netd's constant), not a deeper queue.
pub fn ingress_route_buffer_depth(max_slice_len: usize) -> usize {
    if max_slice_len == 0 {
        // Defensive: `MaxSliceLen` is always >= WireHeader::SIZE, so this is
        // unreachable from a real route — the cap rather than a div-by-zero.
        return INGRESS_ROUTE_BUFFER_DEPTH;
    }
    (INGRESS_ROUTE_APPARENT_BUDGET_BYTES / max_slice_len)
        .clamp(DEFAULT_SUBSCRIBER_BUFFER_SIZE, INGRESS_ROUTE_BUFFER_DEPTH)
}

// `usize::clamp` panics if min > max, and a rule whose floor exceeded
// the stock ceiling would LOWER a route's depth rather than raise it. Pin both
// at compile time so a constant edit fails here rather than at a runtime panic
// or, worse, as a silent regression of the very thing this fixes.
const _: () = assert!(
    DEFAULT_SUBSCRIBER_BUFFER_SIZE <= INGRESS_ROUTE_BUFFER_DEPTH,
    "the ingress depth FLOOR must not exceed its CAP (clamp invariant)"
);
const _: () = assert!(
    INGRESS_ROUTE_APPARENT_BUDGET_BYTES / (1024 * 1024) >= 32,
    "the ingress budget must leave a 1 MiB-slice bridge route at least 32 frames \
     — below that it cannot absorb the measured pass-work stall"
);

// The clamp in `recording_tap_buffer_depth` requires FLOOR <= TARGET
// (`usize::clamp` panics if min > max) — and the raise must never LOWER the stock
// ceiling. Pin both at compile time so a future constant edit fails loudly here
// rather than at a runtime `clamp` panic / silent ceiling regression.
const _: () = assert!(
    RECORDING_TAP_BUFFER_DEPTH_FLOOR <= RECORDING_TAP_BUFFER_DEPTH,
    "recording tap-depth FLOOR must not exceed the TARGET (clamp invariant)"
);
const _: () = assert!(
    RECORDING_TAP_BUFFER_DEPTH_FLOOR >= DEFAULT_SUBSCRIBER_BUFFER_SIZE,
    "the recording tap-depth floor must be >= the stock ceiling so the \
     --record raise never LOWERS a topic's buffer"
);

/// The REAL per-slot byte cost of one iceoryx2 data-segment
/// chunk for a `publish_subscribe::<[u8]>()` service whose publisher loans
/// `max_slice_len` bytes.
///
/// # Why this is not `max_slice_len`
///
/// A pool bucket is not the payload — it is `[Header][UserHeader][Payload]`,
/// sized by iceoryx2's own `MessageTypeDetails::sample_layout`
/// (`iceoryx2-0.9.1/src/service/static_config/message_type_details.rs:147`):
///
/// ```text
/// align( header.size + user_header.size + user_header.alignment - 1
///        + payload.size * number_of_elements + payload.alignment - 1,
///        header.alignment )
/// ```
///
/// and that `Layout` becomes the pool's `bucket_layout` verbatim
/// (`iceoryx2-0.9.1/src/port/details/data_segment.rs:87`), with the allocator's
/// hand-out stride equal to `bucket_layout.size()`
/// (`iceoryx2-bb-memory-0.9.1/src/pool_allocator.rs:207`). There is no
/// power-of-two rounding on this path: `AllocationStrategy::Static` is the
/// default (`iceoryx2-0.9.1/src/config.rs:346`) and `resize_hint`'s
/// power-of-two arm belongs to the RESIZABLE segment.
///
/// Every Cerulion data service is `publish_subscribe::<[u8]>()` with the DEFAULT
/// user header `()` (`iceoryx2-0.9.1/src/service/builder/mod.rs:150`), and
/// nothing in this repo calls `.payload_alignment()`, so two of the four terms
/// vanish — `user_header.size + user_header.alignment - 1 == 0` and
/// `payload.alignment - 1 == 0` for `u8` — leaving the closed form below.
///
/// # The header size is READ, never written down
///
/// `IOX2_SAMPLE_HEADER_BYTES` is `size_of::<iceoryx2 publish-subscribe Header>()`
/// rather than a transcribed `40`, so an iceoryx2 bump that grows the header
/// moves this rule with it instead of leaving a stale literal behind. (MEASURED
/// on the pinned 0.9.1: 40 bytes, align 8.)
///
/// # Scope, stated because the number is small
///
/// The gap between this and the nominal slice is **40 bytes per slot** (plus at
/// most 7 of tail padding), not a percentage — a 16 MiB slice really does cost
/// 16 MiB + 40 B, and any figure claiming ~5 % for that class is a MiB/MB unit
/// confusion (16 MiB *is* 16.78 **MB**). It is nonetheless the difference
/// between a budget that fits N slots and one that fits N−1: at a 64 MiB budget
/// a 1 MiB slice admits **63** slots, not 64, and a 16 MiB slice admits **3**,
/// not 4. A rule that divides by the nominal slice therefore over-commits by one
/// slot at every exact power-of-two slice size — which is precisely where real
/// robots sit.
///
/// This models ONE SLOT. The whole data segment additionally carries
/// `header.alignment - 1` of start slack plus `4 * (slots + 1) + 3` bytes of
/// pool-index management — per SEGMENT, amortised, and deliberately not folded
/// in here: a caller dividing a budget by a per-slot cost wants the per-slot
/// cost.
pub fn iceoryx2_slot_bytes(max_slice_len: usize) -> usize {
    let unaligned = IOX2_SAMPLE_HEADER_BYTES.saturating_add(max_slice_len);
    // `align()` from `iceoryx2-bb-elementary`, transliterated: round UP to the
    // header's alignment. Saturating so a caller handing in a slice near
    // `usize::MAX` cannot wrap into a tiny slot (which would then admit a huge
    // depth — the inversion this whole rule exists to prevent).
    let rem = unaligned % IOX2_SAMPLE_HEADER_ALIGN;
    if rem == 0 {
        unaligned
    } else {
        unaligned.saturating_add(IOX2_SAMPLE_HEADER_ALIGN - rem)
    }
}

/// `size_of` of iceoryx2's publish-subscribe sample header — the per-slot
/// overhead [`iceoryx2_slot_bytes`] adds to the payload. Read from the type, so
/// an upstream bump cannot leave a stale literal here.
const IOX2_SAMPLE_HEADER_BYTES: usize =
    std::mem::size_of::<iceoryx2::service::header::publish_subscribe::Header>();

/// `align_of` of the same header — the alignment `sample_layout` rounds the
/// whole bucket up to.
const IOX2_SAMPLE_HEADER_ALIGN: usize =
    std::mem::align_of::<iceoryx2::service::header::publish_subscribe::Header>();

// A zero alignment would divide by zero in `iceoryx2_slot_bytes`; an alignment
// that is not a power of two would mean the transliteration of `align()` above
// no longer matches upstream's. Neither is reachable on a `repr(C)` type, but
// the rule reads a FOREIGN type's layout and a compile-time refusal is cheaper
// than discovering it at runtime.
const _: () = assert!(
    IOX2_SAMPLE_HEADER_ALIGN > 0 && IOX2_SAMPLE_HEADER_ALIGN.is_power_of_two(),
    "the iceoryx2 sample header's alignment must be a non-zero power of two"
);

/// The FLOOR of a Flashback (window-only) recorder's tap
/// depth.
///
/// TWO, not one, and not the stock 16. One is a tap that can hold nothing while
/// the recorder is between drains — every frame published inside a single drive
/// pass but the last would be reclaimed before the tap saw it. Two is the
/// minimum at which a drain and a concurrent publish do not collide, and it is
/// the same floor the liveness observer uses for the same reason
/// ([`LIVENESS_TAP_BUFFER_SIZE`](liveness::LIVENESS_TAP_BUFFER_SIZE)).
///
/// It is deliberately BELOW `DEFAULT_SUBSCRIBER_BUFFER_SIZE`: this rule exists
/// to bound a STANDING footprint on every serving graph, so a huge-slice topic
/// must be allowed to go shallower than the stock default rather than being
/// floored at a depth whose byte cost is what the budget is refusing.
pub const FLASHBACK_TAP_BUFFER_DEPTH_FLOOR: usize = 2;

// The two constant relationships the rule's doc leans on, at COMPILE time — the
// repo's `const _: () = assert!(..)` drift-guard idiom rather than a `#[test]`,
// which is both stronger (a bad edit fails the BUILD) and the only form clippy's
// `assertions_on_constants` accepts.
const _: () = assert!(
    FLASHBACK_TAP_BUFFER_DEPTH_FLOOR < DEFAULT_SUBSCRIBER_BUFFER_SIZE,
    "a window-only tap must be allowed shallower than the stock default — \
     flooring it at the default would refuse exactly the byte cost the budget \
     exists to refuse"
);
const _: () = assert!(
    FLASHBACK_TAP_BUFFER_DEPTH_FLOOR >= 2,
    "one is a queue that holds nothing between drains"
);

/// The receive-queue depth a
/// **window-only** Flashback tap opens with.
///
/// ```text
/// depth = clamp( budget_bytes / iceoryx2_slot_bytes(max_slice_len),
///                FLOOR = 2, ceiling )
/// ```
///
/// # The contract SPLIT this half of
///
/// An explicit `--record` recorder keeps its ceiling-deep tap: it is a
/// replay-grade recording, lossless-by-provisioning, and that ceiling was sized
/// from a measured drain stall. The always-on window-only plane trades
/// bounded ABSORBANCE for a bounded STANDING footprint — it runs on every
/// serving graph whether or not anybody asked to record, so the cost it must
/// bound is the one nobody opted into. The two are different recorders with
/// different promises, and the mode gate is what keeps them apart (see
/// [`TransportManager::create_data_only_subscriber_budgeted`]).
///
/// # `ceiling` is the SERVICE's, not a constant
///
/// A tap can never open deeper than `subscriber_max_buffer_size` (iceoryx2 pins
/// it at create and rejects a deeper opener), so the budget can only ever
/// SHALLOW a tap below what the topic already provisioned. On a topic whose
/// ceiling is already under the budgeted depth this rule is a no-op.
///
/// # The divisor is the REAL slot, and the difference is a whole slot
///
/// See [`iceoryx2_slot_bytes`]: at the shipped 64 MiB budget a 1 MiB-slice topic
/// admits 63 slots and a 16 MiB one admits 3, where a nominal divisor would
/// promise 64 and 4 and over-commit the budget by one slot each.
///
/// # WHAT THIS BOUNDS — resident pinning, NOT the apparent reservation
///
/// Said once, here, because the number an operator sees first is the wrong one.
/// A publisher's pool is sized from the SERVICE's static config —
/// `max_subscribers * (subscriber_max_buffer_size + subscriber_max_borrowed_samples)
/// + history_size + max_loaned` slots
/// (`iceoryx2-0.9.1/src/service/static_config/publish_subscribe.rs:79`) — and a
/// SUBSCRIBER's own depth is not a term in it. So shallowing a tap does not move
/// the pool by one byte: **VIRT stays at the topic's provisioned slot count
/// (~112 slots for a `--record`-raised topic) whatever this rule returns.**
///
/// What it bounds is what the tap can PIN: a queued chunk is a slot the tap is
/// holding, and a held slot is faulted-in RAM (`Static` pools are
/// lazily demand-paged, so resident tracks occupancy, not reservation). That is
/// the standing cost of an always-on window, and it is the one this budget
/// answers for — reported as `shm_pinned_bytes`. Reading the apparent
/// reservation as the bill will say this rule changed nothing; it is measuring
/// the wrong quantity.
pub fn flashback_tap_buffer_depth(
    budget_bytes: u64,
    max_slice_len: usize,
    ceiling: usize,
) -> usize {
    let slot = iceoryx2_slot_bytes(max_slice_len);
    // `iceoryx2_slot_bytes` is >= IOX2_SAMPLE_HEADER_BYTES > 0 for every input,
    // so this cannot divide by zero — but the header size is read from a foreign
    // type, so guard rather than rely on it.
    let by_budget = if slot == 0 {
        ceiling
    } else {
        (budget_bytes / slot as u64).min(usize::MAX as u64) as usize
    };
    // The ceiling wins outright when it is BELOW the floor: a service
    // provisioned at depth 1 cannot be opened at 2, and `clamp` would panic on
    // an inverted range.
    by_budget.clamp(FLASHBACK_TAP_BUFFER_DEPTH_FLOOR.min(ceiling), ceiling)
}

/// How a data-only tap picks its receive-queue depth.
///
/// A THREE-way choice rather than an `Option<usize>`, because the third arm is a
/// different KIND of request: `Fixed` names a depth the caller already decided,
/// while `Budget` names a byte ceiling the transport prices against the topic's
/// real slot layout — a decision only the transport can take, since the slice
/// lives on the publisher's dynamic config.
///
/// It is private: the choice is expressed by WHICH constructor a caller reaches
/// for, which is what makes the `--record` / window-only mode gate structural.
enum TapDepth {
    /// The service ceiling — the deepest view the topic provisioned. The
    /// explicit `--record` recorder's tap, lossless-by-provisioning.
    Ceiling,
    /// An explicit depth, clamped into `1 ..= ceiling` (the liveness observer).
    Fixed(usize),
    /// A BYTE budget priced over [`iceoryx2_slot_bytes`]:
    /// the always-on window-only Flashback recorder's tap.
    Budget(u64),
}

static INSTANCE: OnceLock<Arc<TransportManager>> = OnceLock::new();

/// The network EGRESS posture a `graph run` resolves — the
/// default-visibility policy for a graph with no `network:` block.
///
/// HOW IT REACHES THE PLAN: as an EXPLICIT ARGUMENT, not through a manager.
/// [`compute_gateway_plan`](crate::graph::runtime::compute_gateway_plan) takes the
/// posture as its fourth parameter, and a `graph run` carries it to the
/// `run-gateway` child inside the gateway handoff — which is also where the plan
/// it decides is computed, supervisor-side.
///
/// The [`TransportManager`] carries NO copy of it. Do not
/// add a manager-side copy: nothing in the graph build would read it, the
/// manager's field set is part of the cdylib `init()` closure (so a field
/// added or removed there costs an ABI bump), and a posture the plan does not
/// consult is a
/// setting that silently does nothing.
///
/// It is not a [`TransportConfig`] field either: `TransportConfig` is constructed
/// with exhaustive struct literals across many crates (and cannot derive serde —
/// it holds `Arc<dyn Clock>`), so a new required field would break every
/// literal.
///
/// `Strict` is the default (today's behavior: an unpartitioned no-`network:`
/// graph runs LOCAL-only, nothing egresses). `PermissiveDefault` is the
/// opt-in the CLI selects for real-clock single-process `graph run`: with a
/// network session configured AND no enabled `network:` block, the build opens
/// egress to every produced topic (AllowAll + announce) until pairing
/// lands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NetworkPosture {
    /// Local-only default — nothing egresses without an explicit `network:`
    /// block. Today's behavior.
    #[default]
    Strict,
    /// Opt-in: a network-configured, no-`network:`-block graph opens
    /// egress to every produced topic (default-permissive) until pairing lands.
    PermissiveDefault,
}

/// Configuration for the transport manager.
pub struct TransportConfig {
    /// Node name for the iceoryx2 node.
    pub node_name: String,
    /// Clock implementation for timestamps.
    pub clock: Arc<dyn Clock>,
    /// Subscriber buffer size (number of samples buffered per subscriber).
    pub subscriber_buffer_size: usize,
    /// Network configuration. `Some` enables zenoh network transport (lazy).
    /// `None` means local-only mode (no zenoh session created).
    pub network: Option<NetworkConfig>,
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            node_name: "cerulion".to_string(),
            clock: Arc::new(RealClock),
            subscriber_buffer_size: DEFAULT_SUBSCRIBER_BUFFER_SIZE,
            // Local-only by default — a graph run populates this
            // from its `network:` block via
            // `GraphConfig::network_transport_config()` (the CLI does).
            network: None,
        }
    }
}

/// Per-topic iceoryx2 SERVICE configuration,
/// computed by the graph runtime from the topology.
///
/// iceoryx2 service settings are agreed between ALL openers of a service:
/// `open_or_create` treats them as requirements, so an opener demanding
/// more than the creator provisioned fails (for the buffer ceiling:
/// `DoesNotSupportRequestedMinBufferSize`). The
/// graph runtime therefore computes ONE `TopicServiceConfig` per topic and
/// threads the identical value through every open site — the publisher,
/// each consumer's body subscriber, and the data-trigger drain subscriber.
/// Non-graph openers (CLI introspection, raw transport users, tests) go
/// through [`TransportManager::default_topic_config`], whose values are
/// guaranteed ≤ any topology-derived config (see field docs), so they can
/// always open a graph-created service.
///
/// `max_subscribers` and `max_publishers` extend this
/// struct — it exists so every per-topic knob lands in ONE place that all
/// open sites share by construction. `#[non_exhaustive]` so adding those
/// fields breaks no external constructor; build values via
/// [`Self::for_topology`] or start from
/// [`TransportManager::default_topic_config`] and mutate fields.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TopicServiceConfig {
    /// Service-level `subscriber_max_buffer_size`: the largest buffer any
    /// subscriber of this topic may request. The runtime computes
    /// `max(global default, largest declared consumer depth)` — never
    /// BELOW the global default, so default openers (which require the
    /// global value) always succeed on graph-created services.
    pub subscriber_max_buffer_size: usize,
    /// The iceoryx2 `subscriber_max_borrowed_samples` for this topic.
    /// `None` = use the iceoryx2 default (2). `Some(3)` is set ONLY on
    /// topics that feed a non-trigger latest-value input of a macro node
    /// (a snapshot consumer): such a consumer HOLDS the last-delivered
    /// sample across steps, so the drain peak is held(1) +
    /// latest(1) + receive-transient(1) = 3 borrowed per connection. A
    /// burst (≥2 queued) would exceed the default 2 → `ExceedsMaxBorrows`.
    /// Trigger-only / non-snapshot topics keep the default 2 (no waste).
    pub subscriber_max_borrowed_samples: Option<usize>,
    /// A CREATE-leg-only borrow floor — the `subscriber_max_borrowed_samples`
    /// a service THIS opener creates is raised to, WITHOUT arming any open-time
    /// requirement (the field above stays the open requirement; a pre-existing
    /// smaller service is still attached, degraded-tolerantly, per the
    /// create-vs-open asymmetry — see `Self::create_borrowed_samples`,
    /// pub(crate) and deliberately not linked to keep the rustdoc gate green).
    ///
    /// Set by `rmw_cerulion`'s publisher/subscription create paths for
    /// loanable (recursively-fixed) types: an rmw loaned take
    /// (`rmw_take_loaned_message`) HOLDS an [`OwnedInboundSample`](subscriber::OwnedInboundSample)
    /// across the C ABI until rclcpp returns it, so the per-connection borrow
    /// peak is the held-loan count — above the iceoryx2 default of
    /// `ICEORYX2_DEFAULT_BORROWED_SAMPLES` (2). It differs from the
    /// owned-topic floor (which is keyed on [`PublisherProvisioning`] and pinned
    /// at `SUBSCRIBER_MAX_BORROWED_HELD` = 3 — both pub(crate) consts, unlinked
    /// for the same rustdoc reason) in both trigger and value, so it is
    /// its own knob rather than a special case folded into that rule.
    ///
    /// A floor at or below the iceoryx2 default is a no-op (`None`-equivalent);
    /// `None` (the default everywhere) changes nothing.
    pub create_borrow_floor: Option<usize>,
    /// Service-level `max_subscribers`. `Some(n)`
    /// provisions the topic for at most `n` subscribers AND requires ≥ `n`
    /// when opening an existing service; `None` leaves the setter uncalled
    /// — iceoryx2's config default (8) applies on create and NO
    /// requirement is verified on open, so default openers always pass
    /// the OPEN-time verification regardless of how the graph provisioned
    /// the topic (no dominance rule needed, unlike the buffer ceiling);
    /// subscriber-PORT creation can still hit slot exhaustion once the
    /// provisioned slots are all attached — see
    /// [`INTROSPECTION_SUBSCRIBER_HEADROOM`]. The graph runtime computes
    /// `in-graph subscribers (body + trigger drains) +
    /// `[`INTROSPECTION_SUBSCRIBER_HEADROOM`] introspection
    /// headroom — the term that bounds the publisher pool's
    /// ×max_subscribers multiplier (see the pool breadcrumb).
    /// When `Some`, the value must be in `1..=`[`MAX_REASONABLE_PORTS`] —
    /// enforced at the `open_topic_services` choke point.
    pub max_subscribers: Option<usize>,
    /// Service-level `max_publishers`. `Some(1)` for
    /// every graph topic with an in-graph producer — SERVICE-level
    /// single-writer enforcement: the topology already rejects
    /// double-producers at build, and this makes iceoryx2 itself reject a
    /// rogue second publisher (another graph, a raw transport user) at
    /// PORT creation. `None` = setter uncalled — the node config's
    /// create-default (stock iceoryx2: 2) applies and no requirement is
    /// verified on open; used by default
    /// openers AND by producer-less external topics (an external publisher
    /// must be able to attach; absolute topic names make such topics
    /// expressible). The
    /// `multi_publisher_topics:` opt-in relaxes owned topics to
    /// `Some(`[`MULTI_PUBLISHER_LOOSE_MAX`]`)` (a shared loose constant,
    /// NOT an exact declared count — see the `Multi` variant doc). When
    /// `Some`, the value must be in `1..=`[`MAX_REASONABLE_PORTS`] —
    /// enforced at the `open_topic_services` choke point.
    pub max_publishers: Option<usize>,
    /// Cross-graph collision guard: the
    /// topology's declared publisher-side intent, carried alongside the
    /// numbers it derives so enforcement sites read intent, never
    /// reconstruct it from a magnitude. The single-writer active-publisher
    /// pre-check in `create_publisher_with_topic_config` keys on THIS
    /// field (`== SingleWriter`), not on `max_publishers == Some(1)` —
    /// hand-mutating `max_publishers` does not import cross-graph
    /// rejection semantics, and a future provisioning change (e.g. the
    /// `multi_publisher_topics` opt-in's looser counts) extends the enum
    /// match instead of silently dodging a magic-value comparison.
    pub publisher_provisioning: PublisherProvisioning,
    /// Service-level iceoryx2 NATIVE history size — the
    /// number of most-recent sent frames the publisher port retains (by SHM
    /// offset, zero-copy) for automatic delivery to late joiners. Set ONLY at
    /// the CREATOR/precreate service-builder call (`.history_size(N)` when
    /// `> 0`); openers (body sub, trigger drain, introspection) leave the
    /// setter UNCALLED, so they impose no open-time requirement (iceoryx2
    /// open-verification is at-least and fires only for openers that
    /// themselves call `.history_size()`). The runtime sets this to the max
    /// `history_size` declared across the topic's producing outputs (0 =
    /// disabled). Default ([`TransportManager::default_topic_config`]) is 0.
    pub history_size: usize,
    /// EXTRA event-service listener slots to provision on
    /// top of the live data-service-derived budget (`live_subs + live_pubs`),
    /// for standalone listener-only `Listener`s that have NO backing
    /// subscriber port. Unified data-trigger provisioning is right-sized:
    /// a UNIFIED data-trigger input contributes +1 SUBSCRIBER (body only,
    /// not the +2 of a dual-subscriber path), so its WaitSet wake
    /// source — the standalone listener created by
    /// `TransportManager::create_trigger_listener` — does not ride a
    /// provisioned-but-unused second subscriber's listener half. This term
    /// re-adds exactly that listener slot per unified input WITHOUT the
    /// payload-sized subscriber SHM-pool slot a dual-subscriber over-count reserves.
    /// Summed by the runtime per topic (`trigger_listeners_by_topic`) and
    /// added to the armed event-port budget in `TransportManager::open_topic_services`
    /// BEFORE the [`MAX_REASONABLE_PORTS`] clamp. EXTERNAL topics leave this
    /// 0 — the foreign creator owns the event caps and the standalone
    /// listener opens the existing event service open-only on a default slot.
    /// Default ([`TransportManager::default_topic_config`]) is 0.
    pub extra_event_listeners: usize,
    /// The iceoryx2 `publisher_max_loaned_samples` for
    /// the publisher PORT this config creates. `None` = leave the setter
    /// uncalled — iceoryx2's config default (2) applies. `Some(n)` calls
    /// `.max_loaned_samples(n)` on the publisher builder in
    /// `finish_publisher`, raising how many samples the publisher can hold
    /// LOANED simultaneously before `loan_slice_uninit` fails
    /// `LoanError::ExceedsMaxLoans` (the rmw borrow-window publish path
    /// needs ~4 outstanding loans per adoption-enabled publisher).
    ///
    /// Unlike every other knob on this struct this is a PORT-level value, not
    /// a service-level one: it is baked into the ONE publisher port built from
    /// this config, imposes NO open-time requirement on any other opener of
    /// the service, and needs no cross-opener agreement (so there is no
    /// create-vs-open asymmetry and no `create_borrowed_samples`-style
    /// two-phase). It also deliberately does NOT enter the
    /// `TopicRequirements` harvest — the producer-owning worker's own local
    /// config is the only place the value matters.
    ///
    /// # Cost
    ///
    /// Each unit is one more term in the publisher's SHM pool size
    /// (`max_subscribers * (buffer + borrowed) + history_size + max_loaned` —
    /// `iceoryx2-0.9.1/src/service/static_config/publish_subscribe.rs:79`),
    /// sized at the publisher's SLICE CEILING — so each unit adds one APPARENT
    /// slot of `max_slice_len` bytes. That is virtual address space (an
    /// `RLIMIT_AS` term), not resident RAM: iceoryx2 `Static` pools are
    /// demand-paged, so an unheld slot costs ~0 real memory.
    ///
    /// When `Some`, the value must be in `1..=`[`MAX_REASONABLE_PORTS`] —
    /// enforced at the `finish_publisher` choke point (a zero would make
    /// EVERY loan fail at runtime; a garbage value becomes a giant pool
    /// reservation failing as a bare create error).
    pub publisher_max_loaned_samples: Option<usize>,
}

/// An RAII handle that keeps a pre-created topic's
/// iceoryx2 data + event services alive on THIS process's `Node`, from
/// pre-create until the graph's node loop attaches the real ports.
///
/// iceoryx2 counts a service's liveness by NODE registration, not by port
/// count or `PortFactory`-handle count (verified against vendored 0.9.1:
/// `service/mod.rs` `ServiceState::drop` routes through the `Node`'s
/// `RegisteredServices`, which removes the SHM service only when this Node's
/// last handle to it drops). So dropping the factory with zero ports
/// attached — the only registrant being this fresh build — would tear the
/// just-created SHM service down, undoing the pre-create. The graph runtime
/// therefore holds these handles for its whole life (see
/// [`TransportManager::precreate_topic_services`]).
#[must_use = "dropping a PrecreatedService releases its services before the ports attach"]
pub struct PrecreatedService {
    _data: iceoryx2::service::port_factory::publish_subscribe::PortFactory<CerService, [u8], ()>,
    _event: iceoryx2::service::port_factory::event::PortFactory<CerService>,
}

/// Appended to a `DoesNotExist` error when the graph node
/// loop opens an OWNED topic's service that should have been created by
/// [`TransportManager::precreate_topic_services`] before any port wiring.
const OWNED_TOPIC_NOT_PRECREATED_HINT: &str =
    " — internal: this graph-owned topic's iceoryx2 service was expected to \
     exist (the build pre-creates every owned topic's services at \
     build start, before wiring ports) but does not. Either an owned topic \
     escaped the pre-create pass (a build invariant — please report) or a \
     peer removed the service mid-startup; retry, and if it persists, report \
     it with the graph file";

/// Spare subscriber slots provisioned on every graph
/// topic beyond the graph's own attaches — room for `cerulion topic
/// echo` / `hz` and late tooling without re-provisioning the service.
///
/// # Why 5, not 4
///
/// Four of these are the genuinely-SPARE tooling slots the constant was
/// introduced for: `bagd` (`graph run --record`), `cerulion topic
/// echo`/`hz`/`info`, and a vizd attach. The fifth pays for a STANDING
/// consumer: the
/// [`TopicLivenessObserver`](liveness::TopicLivenessObserver) holds one
/// listener-less tap per topic on the robot's gateway for as long as the
/// gateway runs, so that the desk can tell a registered-but-dead route
/// from a streaming one. Left at 4 that observer would silently consume
/// one of the four, and an operator who ran `--record` while `topic
/// echo`, `hz` and vizd were attached would hit slot exhaustion on a
/// combination that otherwise fits. The observer yields its tap to a
/// topic's egress tap (never two gateway ports for one topic) and
/// `CERULION_TOPIC_LIVENESS=off` removes it entirely — but the
/// provisioning must not ASSUME either, so the slot is budgeted.
///
/// # What a slot costs (the second axis, deliberately paid)
///
/// This constant is not only a port cap: it is a term in the publisher's SHM
/// pool size, which iceoryx2 computes as
/// `max_subscribers*(subscriber_max_buffer_size + subscriber_max_borrowed_samples)
/// + history_size + publisher_max_loaned` (see the `history_size` note on
/// `open_topic_services`). So 4 → 5 grew every graph-owned topic's pool by ONE
/// ceiling-deep slot-group — sized from the SERVICE ceiling, NOT from the
/// shallow [`LIVENESS_TAP_BUFFER_SIZE`](liveness::LIVENESS_TAP_BUFFER_SIZE) the
/// observer actually requests, so the shallow tap does not offset it. On a
/// `--record` topic, where the ceiling is raised to
/// [`RECORDING_TAP_BUFFER_DEPTH`], that is up to
/// [`RECORDING_TAP_APPARENT_BUDGET_BYTES`] (1 GiB) more APPARENT per topic.
///
/// That is accepted for the same measured reason the recording ceiling is:
/// iceoryx2 `Static` pools are LAZY / demand-paged on Linux AND macOS (a
/// 54.8 GiB apparent multi-pool reservation faulted in at 0.84 MiB resident on
/// a 15 GiB Jetson), so an unused slot-group costs ~0 real RAM. It IS real
/// virtual address space, i.e. an `RLIMIT_AS` term — so the two escape hatches
/// are the ones already documented above: `CERULION_TOPIC_LIVENESS=off` (which
/// removes the standing consumer this slot exists for) and not raising the
/// ceiling (`--record` is what raises it).
pub const INTROSPECTION_SUBSCRIBER_HEADROOM: usize = 5;

/// Upper sanity bound on a per-topic port count (`max_subscribers` OR
/// `max_publishers`; the constant was renamed when the `max_publishers` guard
/// adopted it). iceoryx2 sizes
/// port registries at service create and per-publisher data segments at
/// publisher-port create from these values, so a garbage value would
/// become a giant allocation failing as a bare error. Far above any real
/// graph (fan-out is node-count bounded).
pub const MAX_REASONABLE_PORTS: usize = 4096;

/// How a topic's publisher side is provisioned, derived
/// from the graph topology. The `Multi` variant is the opt-in —
/// call sites extend a `match`, not a second signature churn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PublisherProvisioning {
    /// No in-graph producer: leave the `max_publishers` setter uncalled so
    /// iceoryx2's create-default applies and an EXTERNAL publisher can
    /// attach (expressible through absolute topic
    /// names).
    External,
    /// The graph owns the topic's single producer: provision
    /// `max_publishers = 1` — service-level single-writer enforcement
    /// against out-of-graph writers.
    SingleWriter,
    /// The topic is listed in the graph's
    /// `multi_publisher_topics:` opt-in AND has at least one in-graph
    /// producer — provision the shared loose cap
    /// [`MULTI_PUBLISHER_LOOSE_MAX`] instead of an exact count (the user
    /// can't always know how many publishers a topic like
    /// `/tf` will carry — a node you don't own may add more). The loose
    /// cap is cheap: iceoryx2 allocates data segments per ACTUAL
    /// publisher at port creation (vendored 0.9.1 publisher.rs:446-467),
    /// not reserved from the max at service creation, so unused slots
    /// cost only registry entries. A shared CONSTANT (not a user knob)
    /// keeps cross-graph opens consistent: `Some(N)` provisions AND
    /// requires on open, and every Cerulion graph requiring the same N
    /// passes iceoryx2's at-least verification by equality.
    Multi,
}

/// The `max_publishers` cap provisioned for every
/// [`PublisherProvisioning::Multi`] topic. One shared constant across all
/// Cerulion graphs — see the variant doc for why it is loose and why it
/// must not be per-graph configurable.
pub const MULTI_PUBLISHER_LOOSE_MAX: usize = 16;

/// The `max_subscribers` cap provisioned for every
/// [`PublisherProvisioning::Multi`] topic. The per-graph formula
/// (in-graph subscribers + introspection headroom) is GRAPH-DEPENDENT,
/// and iceoryx2 `Some(n)` requirements are at-least verified on open — a
/// second graph with more consumers than the creator could never attach
/// to a shared topic. Cross-graph topics therefore need a shared
/// graph-INDEPENDENT constant on BOTH port axes (the same equality
/// argument as [`MULTI_PUBLISHER_LOOSE_MAX`]). The slots are shared by
/// every graph's subscribers AND introspection tools on the topic;
/// exhaustion fails loudly at port creation with the slot hint, and the
/// graph runtime fail-fasts at build when a SINGLE graph alone would
/// exceed the cap.
pub const MULTI_SUBSCRIBER_LOOSE_MAX: usize = 16;

/// The `subscriber_max_buffer_size` ceiling provisioned
/// for every [`PublisherProvisioning::Multi`] topic — the THIRD
/// graph-dependent service knob that must become a shared constant on
/// cross-graph topics (the `max(default, largest consumer
/// depth)` formula varies per graph, so a consumer-heavy second graph
/// could never open the creator's service). Sixteen bounds the
/// per-publisher SHM pool (each publisher's data segment scales with
/// ceiling × subscriber slots × `max_slice_len`) while covering the
/// default consumer depth (10) and the stock transport default (16);
/// a consumer declaring a deeper queue on a listed topic is rejected
/// loudly at graph build — raise the constant deliberately rather than
/// letting one graph silently commit every publisher to a deeper pool.
pub const MULTI_TOPIC_BUFFER_CEILING: usize = 16;

// The ceiling's documented rationale ("covers the
// default consumer depth") is a real numeric dependency on a constant in
// another module — pin it at compile time so a `DEFAULT_CONSUMER_DEPTH`
// bump can't silently rot the Multi provisioning story.
const _: () = assert!(
    MULTI_TOPIC_BUFFER_CEILING >= crate::graph::topology::DEFAULT_CONSUMER_DEPTH,
    "MULTI_TOPIC_BUFFER_CEILING must cover DEFAULT_CONSUMER_DEPTH (see its doc)"
);

impl TopicServiceConfig {
    /// The topology-derived config
    /// for one topic. The per-field contract (each lives HERE, in the type,
    /// so a field addition extends this signature and body together — the
    /// incompatible-service-config-at-a-distance class this type exists to
    /// kill):
    ///
    /// - `subscriber_max_buffer_size`: `max(default, largest consumer
    ///   depth)` — true dominance over `default`, never below it, so
    ///   default openers always attach.
    /// - `max_subscribers` / `max_publishers`: computed from the topology
    ///   args alone; `default`'s values for these fields are NOT consulted
    ///   (the runtime's `default_topic_config` always carries `None` for
    ///   both — a hand-mutated `Some` in `default` is deliberately
    ///   discarded here).
    /// - `history_size`: the `history_size` ARGUMENT
    ///   verbatim (the runtime's max declared history across the topic's
    ///   producing outputs); `default`'s value is NOT consulted.
    /// - `subscriber_max_borrowed_samples`: `None` — the
    ///   OPEN requirement stays consumer-driven (the runtime's snapshot
    ///   post-pass raises it). The owned-topic borrow FLOOR is
    ///   create-side generosity only, applied on the CREATE leg at the open
    ///   site via `Self::create_borrowed_samples` (pub(crate) — not linked to
    ///   keep the rustdoc gate green) — never armed as an open requirement
    ///   (see the field comment below).
    ///
    /// `#[non_exhaustive]` blocks external literals, so external values
    /// must START from this constructor or
    /// [`TransportManager::default_topic_config`] — though pub-field
    /// mutation can still depart from these rules afterwards (the
    /// open-site guard catches the ZERO case, the only silent one; a
    /// nonzero below-default departure surfaces loudly at open time via
    /// iceoryx2's min-requirement check when it conflicts); in-crate
    /// construction is convention.
    pub fn for_topology(
        default: Self,
        max_consumer_depth: usize,
        in_graph_subscribers: usize,
        publisher_provisioning: PublisherProvisioning,
        history_size: usize,
        extra_event_listeners: usize,
    ) -> Self {
        // Buffer-ceiling only for externals:
        // the graph does not OWN an External topic, so it imposes only
        // its genuine consumer requirement — the buffer ceiling. Port
        // counts stay None on BOTH sides: a Some(n) subscriber
        // requirement could fail the graph at open against a foreign
        // creator's smaller provisioning, and creating first would size a
        // topic the graph doesn't own. Owned (SingleWriter) topics keep
        // the per-graph provisioning: subscribers + introspection
        // headroom, single-writer publishers. (No max() against `default`
        // for the Some arm: `None` imposes no open requirement, so a
        // graph value below iceoryx2's create-default can never fail a
        // default opener's OPEN-time check.)
        let (max_subscribers, max_publishers) = match publisher_provisioning {
            PublisherProvisioning::External => (None, None),
            PublisherProvisioning::SingleWriter => (
                Some(in_graph_subscribers.saturating_add(INTROSPECTION_SUBSCRIBER_HEADROOM)),
                Some(1),
            ),
            // A multi-publisher topic is CROSS-GRAPH by
            // definition, so BOTH port caps must be graph-independent
            // shared constants — `Some(n)` provisions AND requires
            // (at-least on open), and a graph-dependent subscriber term
            // (the in-graph + headroom formula) would lock out any second
            // graph needing more than the creator provisioned. Every
            // Cerulion graph requires the same two constants, so opens
            // pass by equality; the runtime fail-fasts at build when one
            // graph ALONE exceeds the subscriber cap.
            PublisherProvisioning::Multi => (
                Some(MULTI_SUBSCRIBER_LOOSE_MAX),
                Some(MULTI_PUBLISHER_LOOSE_MAX),
            ),
        };
        // The buffer ceiling is the third graph-dependent knob
        // that must be a shared constant on cross-graph (Multi) topics —
        // see MULTI_TOPIC_BUFFER_CEILING. The runtime fail-fasts at build
        // when a consumer's declared depth exceeds it.
        let subscriber_max_buffer_size = match publisher_provisioning {
            PublisherProvisioning::Multi => MULTI_TOPIC_BUFFER_CEILING,
            _ => max_consumer_depth.max(default.subscriber_max_buffer_size),
        };
        Self {
            subscriber_max_buffer_size,
            // The borrowed-sample OPEN REQUIREMENT is a per-CONSUMER
            // concern (only snapshot consumers hold a sample across steps), not
            // derivable from the producer-side topology args this constructor
            // receives. The runtime raises it to `Some(3)` in a post-pass over
            // the snapshot-source topic set; default (no requirement) here.
            //
            // Create-vs-open asymmetry: this field is deliberately
            // NOT floored here. iceoryx2's builder value acts as BOTH the
            // create value AND an at-least open requirement, so flooring it
            // would make every owned open REFUSE a pre-existing sub-floor
            // service (e.g. one created by a default opener like `cerulion
            // topic echo`) — breaking the repo's degraded-attach contract
            // (owners tolerate + warn, they don't refuse). The floor is
            // instead applied ONLY on the CREATE leg at the open site — see
            // [`Self::create_borrowed_samples`] and the two-phase acquisition in
            // `open_topic_services`.
            subscriber_max_borrowed_samples: None,
            // No create-leg borrow floor from topology — the rmw
            // create paths set it per-topic for loanable types.
            create_borrow_floor: None,
            max_subscribers,
            max_publishers,
            publisher_provisioning,
            // The topology's max declared `history_size`
            // across this topic's producing outputs (0 = disabled). Consumed
            // ONLY at the creator/precreate `open_topic_services` call; the
            // `default`'s value is irrelevant here (a topology config always
            // carries its OWN computed history, never the seed's — same
            // discard contract as the port-count fields above).
            history_size,
            // Standalone listener-only slots (unified
            // data-trigger inputs) on this topic. The ARGUMENT verbatim — the
            // runtime's per-topic `trigger_listeners_by_topic` count; the
            // `default`'s value is discarded like the port-count fields. The
            // External arm above zeroes `max_subscribers`/`max_publishers`,
            // so `open_topic_services` never arms the event caps for an
            // External topic regardless of this term (the runtime also passes
            // 0 for External topics — the foreign creator owns the caps).
            extra_event_listeners,
            // Not derivable from topology — an rmw borrow-window caller that
            // needs extra outstanding loans sets it explicitly on the config
            // it hands the publisher-create site. `None` keeps the iceoryx2
            // port default (2).
            publisher_max_loaned_samples: None,
        }
    }

    /// The `subscriber_max_borrowed_samples` value for the CREATE leg
    /// of a service acquisition — create-side generosity, distinct from the
    /// OPEN requirement (`self.subscriber_max_borrowed_samples`).
    ///
    /// # The create-vs-open asymmetry
    ///
    /// iceoryx2 bakes `subscriber_max_borrowed_samples` in at service CREATION
    /// and cannot raise it later, so a producer-graph-first creation at the
    /// iceoryx2 default (2) would hard-abort a later HOLD-consumer
    /// graph (borrow-3 open refused with
    /// `DoesNotSupportRequestedMinSubscriberBorrowedSamples`). So every
    /// OWNED (`SingleWriter`/`Multi`) topic is CREATED at (at least) the HOLD
    /// floor [`SUBSCRIBER_MAX_BORROWED_HELD`] — reusing the SAME constant the
    /// consumer requirement uses (the runtime's snapshot post-pass), so the
    /// create floor and the open requirement cannot drift — while the floor is
    /// NEVER armed as an open requirement: an owner TOLERATES a pre-existing
    /// sub-floor service (attach + the degraded-provisioning warn machinery,
    /// the degraded-attach contract) instead of refusing it. Only
    /// consumers that genuinely hold (the snapshot post-pass `Some(3)`, the
    /// recording raise `Some(8)`, a stamped cross-group union) keep the
    /// borrow as an open requirement, via the field itself.
    ///
    /// `max` with any genuine requirement keeps the two composable (a
    /// recording-raised `Some(8)` creates at 8, not the floor); `External`
    /// passes the field through untouched — the graph only OPENS foreign
    /// services, so it must not inflate what it would create in the
    /// no-foreign-creator race (the create value IS the open requirement's
    /// value there, preserving the loud refusal arm for external snapshot
    /// sources).
    pub(crate) fn create_borrowed_samples(&self) -> Option<usize> {
        let base = match self.publisher_provisioning {
            PublisherProvisioning::External => self.subscriber_max_borrowed_samples,
            PublisherProvisioning::SingleWriter | PublisherProvisioning::Multi => Some(
                self.subscriber_max_borrowed_samples
                    .unwrap_or(ICEORYX2_DEFAULT_BORROWED_SAMPLES)
                    .max(SUBSCRIBER_MAX_BORROWED_HELD),
            ),
        };
        // Fold in the create-leg-only borrow floor (rmw loaned-take
        // held samples — see the field doc). `max`-composed so it can only
        // RAISE what would have been created, never lower a genuine
        // requirement. The floor is NORMALIZED first (`effective_create_borrow_floor`
        // — the ONE definition of "a floor at/below the iceoryx2 default is no
        // request"), so a no-op floor keeps `base` verbatim and the
        // two-phase split (triggered by `open != create`) is not entered
        // pointlessly.
        match self.effective_create_borrow_floor() {
            None => base,
            Some(floor) => Some(base.unwrap_or(ICEORYX2_DEFAULT_BORROWED_SAMPLES).max(floor)),
        }
    }

    /// The EFFECTIVE create-leg borrow floor — [`Self::create_borrow_floor`]
    /// normalized so that a requested floor at or below
    /// `ICEORYX2_DEFAULT_BORROWED_SAMPLES` (2) is NO request at all. A default
    /// create already lands at the iceoryx2 default, so such a floor can
    /// change nothing; treating it as `None` here is what makes the create
    /// leg (`create_borrowed_samples`) and the attach-time degrade warn agree:
    /// a `Some(2)` floor neither raises a create nor warns when a pre-existing
    /// service sits at 1 (a valid configuration). THE ONE definition of the
    /// `Some(2) ≡ None` rule — both consumers read this, never the raw field.
    pub(crate) fn effective_create_borrow_floor(&self) -> Option<usize> {
        self.create_borrow_floor
            .filter(|&floor| floor > ICEORYX2_DEFAULT_BORROWED_SAMPLES)
    }

    /// Service REQUEST topic provisioning (explicit and
    /// role-derived, never sniffed from the topic
    /// name). A request topic fans N client
    /// publishers in to one server subscriber, so it provisions
    /// `MAX_SERVICE_REQUEST_CLIENTS` publisher AND subscriber slots and
    /// marks the publisher intent [`PublisherProvisioning::Multi`] (so the
    /// single-writer active-publisher pre-check never fires). The event-port
    /// budget the publisher pool derives from these data caps covers every
    /// client's notifier + listener — subsuming `for_topic`'s explicit
    /// `2 × N` notifier/listener sizing. Native history is off (a service is
    /// request/reply, not latched).
    pub fn for_service_request(subscriber_buffer_size: usize) -> Self {
        Self {
            subscriber_max_buffer_size: subscriber_buffer_size,
            // A request topic has no macro snapshot consumer — keep
            // the iceoryx2 default borrow count.
            subscriber_max_borrowed_samples: None,
            // No held-loan consumers on a service request topic.
            create_borrow_floor: None,
            max_subscribers: Some(MAX_SERVICE_REQUEST_CLIENTS),
            max_publishers: Some(MAX_SERVICE_REQUEST_CLIENTS),
            publisher_provisioning: PublisherProvisioning::Multi,
            history_size: 0,
            // Services have no standalone listener-only
            // sources — the event-port budget the data caps derive already
            // covers every client's notifier + listener.
            extra_event_listeners: 0,
            // Request topics keep the iceoryx2 loan default.
            publisher_max_loaned_samples: None,
        }
    }

    /// Service REPLY topic provisioning (`svc/{service}/reply/{guid}`
    /// — one per client). Exactly one server-writer per client, so it pins
    /// [`PublisherProvisioning::SingleWriter`] (`max_publishers = 1`) — the
    /// single-writer active-publisher pre-check + iceoryx2's own port cap
    /// then reject any rogue second writer. The lone client subscriber + its
    /// event ports fit iceoryx2's defaults (`max_subscribers` left `None`).
    pub fn for_service_reply(subscriber_buffer_size: usize) -> Self {
        Self {
            subscriber_max_buffer_size: subscriber_buffer_size,
            // A reply topic has no macro snapshot consumer — keep
            // the iceoryx2 default borrow count.
            subscriber_max_borrowed_samples: None,
            // No held-loan consumers on a service reply topic.
            create_borrow_floor: None,
            max_subscribers: None,
            max_publishers: Some(1),
            publisher_provisioning: PublisherProvisioning::SingleWriter,
            history_size: 0,
            // No standalone listener-only sources on a reply
            // topic.
            extra_event_listeners: 0,
            // Reply topics keep the iceoryx2 loan default.
            publisher_max_loaned_samples: None,
        }
    }
}

/// The per-topic iceoryx2 service provisioning REQUIREMENTS harvested
/// from a FULL-graph (monolith) build, so a multi-process worker that
/// OWNS a topic's producer can provision the UNION of ALL process groups'
/// consumer requirements — not just the consumers visible in its own subgraph.
///
/// # Why this exists — the multi-process split loses the monolith's union
///
/// A monolith [`crate::graph::GraphRuntime::build`] computes each topic's
/// [`TopicServiceConfig`] by reducing over EVERY consumer of that topic in the
/// whole graph ([`TopicServiceConfig::for_topology`] + the borrow-3
/// post-pass). A `process_groups:` split runs each group in its own
/// process against ONLY its own subgraph, so the producer-owning worker sees
/// ONLY its group's consumers when it creates the topic's iceoryx2 service. A
/// consumer in a DIFFERENT group — e.g. a non-trigger latest-value
/// (snapshot) input that needs `subscriber_max_borrowed_samples >= 3` — is then
/// invisible at create time, so the service is created under-provisioned and the
/// cross-group consumer is refused at open with
/// `DoesNotSupportRequestedMinSubscriberBorrowedSamples` (also
/// `...MinBufferSize` / `...AmountOfSubscribers` for the depth / count axes).
///
/// The supervisor's planning build already builds the FULL graph once (fail-fast
/// plus `Levels` derivation). It HARVESTS these requirements from that build (via
/// [`crate::graph::GraphRuntime::topic_requirements`], which distills the same
/// `topic_configs` the monolith would create — SINGLE SOURCE, no parallel
/// formula that could drift), stamps them into every `WorkerPlan`, and the
/// producer-owning worker provisions `max(local view, stamped union)` per owned
/// topic (`Self::union_into`).
///
/// Fields are normalized concrete `usize`s (not `Option`s) so the serde plan
/// shape carried across the process boundary is unambiguous; the iceoryx2
/// borrowed-samples default (`ICEORYX2_DEFAULT_BORROWED_SAMPLES`) is folded in at
/// harvest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct TopicRequirements {
    /// The topic's `subscriber_max_borrowed_samples` requirement — normalized
    /// from `Option` to the concrete count (the iceoryx2 default
    /// `ICEORYX2_DEFAULT_BORROWED_SAMPLES` when unset). `>=` the held-snapshot
    /// budget `SUBSCRIBER_MAX_BORROWED_HELD` iff SOME consumer of this topic holds
    /// a snapshot across steps. This is the headline term: a
    /// cross-group snapshot consumer is the reported bug. (This carries
    /// GENUINE requirements only — the owned-topic create-side borrow FLOOR
    /// deliberately does NOT enter the harvest, because a stamped union arms this
    /// term as an OPEN requirement on the worker, and the floor must never become
    /// one; every worker applies the same create-side floor locally anyway.)
    pub min_borrowed_samples: usize,
    /// The topic's `subscriber_max_buffer_size` requirement — the monolith's
    /// `max(global default, largest declared consumer depth)`.
    pub min_buffer: usize,
    /// The topic's `max_subscribers` requirement — the monolith's `in-graph
    /// subscribers (body + trigger/sync drains, across ALL groups) +
    /// introspection headroom`. `0` when the monolith left it unset (never for a
    /// harvested owned topic — every owned topic is `SingleWriter`/`Multi`).
    pub min_subscribers: usize,
    /// The topic's `extra_event_listeners` requirement — standalone
    /// listener-only wake slots for UNIFIED data-trigger consumers,
    /// counted across ALL groups so a cross-group unified consumer's listener
    /// has a slot on the producer-owned event service.
    pub min_event_listeners: usize,
}

impl TopicRequirements {
    /// Distill the requirements from a finalized [`TopicServiceConfig`]
    /// (the map the monolith build produced). The `Option` fields are folded to
    /// concrete counts — `subscriber_max_borrowed_samples` to
    /// [`ICEORYX2_DEFAULT_BORROWED_SAMPLES`] when unset, `max_subscribers` to `0`
    /// — so the serde-carried plan value is unambiguous.
    pub(crate) fn from_service_config(cfg: &TopicServiceConfig) -> Self {
        Self {
            min_borrowed_samples: cfg
                .subscriber_max_borrowed_samples
                .unwrap_or(ICEORYX2_DEFAULT_BORROWED_SAMPLES),
            min_buffer: cfg.subscriber_max_buffer_size,
            min_subscribers: cfg.max_subscribers.unwrap_or(0),
            min_event_listeners: cfg.extra_event_listeners,
        }
    }

    /// Merge this harvested (full-graph) requirement INTO a worker's
    /// LOCAL [`TopicServiceConfig`] as a field-wise `max` (the UNION). Only ever
    /// applied to OWNED (non-`External`) topics the worker CREATES — a worker
    /// opens External topics it does not own, and inflating an open requirement
    /// against a foreign creator's service would fail.
    ///
    /// The `max` is monotone, so it can only RAISE provisioning, never lower it:
    /// the worker's own local requirement is always honored, and the stamped
    /// cross-group requirement (which is `>=` local by construction, being a
    /// superset reduction) lifts it to the monolith's value. A `Multi`
    /// (cross-graph opt-in) topic is unaffected — its config is already the
    /// shared graph-independent constant the monolith harvested, so
    /// `max(const, const)` is a no-op.
    pub(crate) fn union_into(&self, cfg: &mut TopicServiceConfig) {
        cfg.subscriber_max_buffer_size = cfg.subscriber_max_buffer_size.max(self.min_buffer);
        // Fold the local Option to a concrete count, max with the stamp, then
        // re-encode: keep `None` (iceoryx2 default, no open-requirement) only
        // when the merged value is still the default — arming `Some(2)` would
        // impose a needless open-time requirement equal to the default.
        let merged_borrow = cfg
            .subscriber_max_borrowed_samples
            .unwrap_or(ICEORYX2_DEFAULT_BORROWED_SAMPLES)
            .max(self.min_borrowed_samples);
        cfg.subscriber_max_borrowed_samples = if merged_borrow > ICEORYX2_DEFAULT_BORROWED_SAMPLES {
            Some(merged_borrow)
        } else {
            cfg.subscriber_max_borrowed_samples
        };
        // An owned topic always carries `Some(local + headroom)`; the stamp's
        // `min_subscribers` already includes the same introspection headroom
        // (both are `for_topology` outputs), so `max` composes them without
        // double-counting. Guard the `0` sentinel (monolith left it unset) so a
        // harvest miss can never DROP an owned topic's port cap to `Some(0)`.
        if self.min_subscribers > 0 {
            let local = cfg.max_subscribers.unwrap_or(0);
            cfg.max_subscribers = Some(local.max(self.min_subscribers));
        }
        cfg.extra_event_listeners = cfg.extra_event_listeners.max(self.min_event_listeners);
    }
}

/// Shared event-port exhaustion context.
/// The listener/notifier caps are provisioned in `open_topic_services`
/// from the topic's subscriber + publisher counts; without this, the bare
/// iceoryx2 variant (its descriptive text logged at a level Cerulion
/// suppresses) plus `TransportError`'s generic check-your-YAML trailer
/// would point operators away from the slot-cap cause.
fn event_port_exhaustion_hint(kind: &str, cap: usize) -> String {
    // The provenance is hedged (a default-created topic's caps are
    // iceoryx2's stock 16/16, not Cerulion-provisioned), and no fake
    // lever — there is no knob that raises the caps today, so the only
    // real remedy is freeing a slot.
    format!(
        " — all {cap} of the topic's event {kind} slots are attached (each \
         data subscriber and each publisher attaches one of each; armed \
         openers provision the caps from the live data service's \
         subscriber + publisher counts, default-created topics carry \
         iceoryx2's event defaults); stop another attacher — e.g. a raw \
         event-service user"
    )
}

/// Environment-failure remedies
/// for OPEN-side publish-subscribe errors — corruption, crashed-peer
/// hangs, and payload-type skew. Shared by `open_topic_services`'
/// `open_or_create` mapping and `open_existing_topic_services`' bare
/// `.open()` (which serves the open-only introspection AND owned-topic
/// node-loop paths), so the remedy text has one source of truth and
/// cannot drift between the acquisition paths.
fn publish_subscribe_open_env_hint(
    e: &iceoryx2::service::builder::publish_subscribe::PublishSubscribeOpenError,
) -> &'static str {
    use iceoryx2::service::builder::publish_subscribe::PublishSubscribeOpenError;
    match e {
        PublishSubscribeOpenError::IncompatibleTypes => {
            "; the existing service carries a different payload type \
             — a version-skewed peer (rebuild it against this \
             Cerulion version) or a foreign iceoryx2 service \
             squatting on this topic name"
        }
        PublishSubscribeOpenError::HangsInCreation => {
            "; a peer crashed while creating this service — remove \
             the stale iceoryx2 shared-memory artifacts (or reboot) \
             before reopening"
        }
        PublishSubscribeOpenError::ServiceInCorruptedState => {
            "; the service's shared state is corrupted (a peer \
             likely crashed mid-operation) — remove the stale \
             iceoryx2 shared-memory artifacts (or reboot) before \
             reopening. NOTE: a LARGE graph more often hits this when \
             the process exhausts its open-file (fd) limit — every \
             node's services/ports consume fds, so a many-node graph \
             can blow past the default 1024; raise it with \
             `ulimit -n 65536` and retry before assuming corruption"
        }
        _ => "",
    }
}

/// The event-service twin:
/// environment remedies for `EventOpenOrCreateError`, shared by
/// `open_topic_services` and `create_subscriber_open_only`. Verified
/// against iceoryx2 (event.rs:359-372): the event
/// `open_or_create` retry set is only {AlreadyExists,
/// IsBeingCreatedByAnotherInstance}, so BOTH the open- and create-side
/// corruption/hangs variants need arms (create-side corruption is the
/// realistic crashed-peer case: orphaned dynamic config, event.rs:529).
fn event_env_hint(e: &iceoryx2::service::builder::event::EventOpenOrCreateError) -> &'static str {
    use iceoryx2::service::builder::event::{
        EventCreateError, EventOpenError, EventOpenOrCreateError,
    };
    match e {
        EventOpenOrCreateError::SystemInFlux => {
            "; another process is creating/removing this event \
             service at high frequency, or a crashed peer left stale \
             service state (iceoryx2 retried internally and gave up) \
             — retry after a moment; if it persists, remove the \
             stale iceoryx2 shared-memory artifacts (or reboot)"
        }
        EventOpenOrCreateError::EventOpenError(EventOpenError::ServiceInCorruptedState)
        | EventOpenOrCreateError::EventCreateError(EventCreateError::ServiceInCorruptedState) => {
            "; the event service's shared state is corrupted (a peer \
             likely crashed mid-operation) — remove the stale \
             iceoryx2 shared-memory artifacts (or reboot) before \
             reopening. NOTE: a LARGE graph more often hits this when \
             the process exhausts its open-file (fd) limit — every \
             node's services/ports consume fds, so a many-node graph \
             can blow past the default 1024; raise it with \
             `ulimit -n 65536` and retry before assuming corruption"
        }
        // iceoryx2 0.10 removed `EventCreateError::HangsInCreation` (it survives
        // on the Open half and on the publish-subscribe errors), so only the
        // open arm remains.
        EventOpenOrCreateError::EventOpenError(EventOpenError::HangsInCreation) => {
            "; a peer crashed while creating this event service — \
             remove the stale iceoryx2 shared-memory artifacts (or \
             reboot) before reopening"
        }
        _ => "",
    }
}

/// A failure of [`TransportManager::open_topic_services`], carrying
/// the mapped [`TransportError`] PLUS the one classification a caller cannot
/// recover from it.
///
/// `map_err` flattens the iceoryx2 error into a reason STRING, so by the time a
/// caller sees the `TransportError` the variant is gone. Every caller but one is
/// happy with that (they propagate). The exception is
/// [`TransportManager::create_ingress_publisher`]'s fallback, which
/// retries at the stock buffer ceiling and then WARNS — and the warn names a
/// cause, so it must know whether the cause was actually observed. Recovering it
/// by matching the reason text would be the message-text-classification
/// anti-pattern this repo bans; recovering it structurally is this type.
struct OpenServicesFailure {
    error: TransportError,
    /// The open was refused because the service ALREADY EXISTS with a smaller
    /// `subscriber_max_buffer_size` than this opener required — iceoryx2's
    /// `DoesNotSupportRequestedMinBufferSize`, the ONLY variant that proves a
    /// shallower incumbent service exists. Every other failure (flux,
    /// corruption, type skew, a port-cap mismatch on a different axis) says
    /// nothing about the buffer ceiling.
    buffer_ceiling_refused: bool,
}

impl OpenServicesFailure {
    /// The degrade an ingress route reports when the raised buffer
    /// ceiling was refused and the STOCK retry succeeded.
    ///
    /// Two arms, because the fallback catches every error class and only ONE of
    /// them licenses naming a culprit. The blame arm tells an operator to go
    /// stop an earlier opener; on a transient failure (`SystemInFlux`, an
    /// environment error) followed by a successful stock retry there may be no
    /// earlier opener at all, and sending someone to hunt for one is worse than
    /// saying less.
    ///
    /// Both arms report the raised failure verbatim in the `reason` field, so
    /// nothing is hidden either way — the difference is only what the prose
    /// CLAIMS.
    ///
    /// A METHOD on the failure, taking no argument, rather than a free function
    /// over a `bool`: the call site then has no discriminant to pass, so the
    /// unconditional report this replaced cannot be written back by hand at the
    /// call site — only by editing the arms below, which the oracle test
    /// `the_ingress_fallback_blames_an_earlier_opener_only_when_one_is_proven`
    /// catches. (The transient arm is not reachable from any black-box test:
    /// the raised and stock legs differ only in `subscriber_max_buffer_size`,
    /// so every non-ceiling failure the raised open can hit is hit identically
    /// by the stock retry.)
    fn ingress_fallback_degrade_message(&self) -> &'static str {
        if self.buffer_ceiling_refused {
            "ingress route attached to a SHALLOWER pre-existing service than \
             this route would provision — the service was created by an earlier \
             opener (e.g. `cerulion topic echo`, or a previous run), and its \
             buffer ceiling is fixed at create. The route works, but at a kHz \
             commit rate its tap queue may overflow under a recorder stall. To \
             get the deeper queue, stop the earlier opener and let the route \
             create the service first."
        } else {
            "ingress route attached at the STOCK buffer ceiling — the deeper \
             ingress ceiling was refused and the stock retry then succeeded. \
             The refusal is reported verbatim in `reason`; it does NOT establish \
             that anything else created the service, so there is nothing here to \
             go and stop. The route works, but at a kHz commit rate its tap \
             queue may overflow under a recorder stall."
        }
    }
}

impl From<TransportError> for OpenServicesFailure {
    /// Failures raised OUTSIDE the data-service open — the choke-point config
    /// guards, the service-name conversions, the event-service open — carry no
    /// buffer-ceiling verdict, so they classify `false`. That is the safe
    /// direction: the blame wording is gated ON the flag, so an unclassified
    /// failure gets the cause-neutral message.
    fn from(error: TransportError) -> Self {
        OpenServicesFailure {
            error,
            buffer_ceiling_refused: false,
        }
    }
}

impl From<OpenServicesFailure> for TransportError {
    fn from(f: OpenServicesFailure) -> Self {
        f.error
    }
}

/// Is this open failure the BUFFER-CEILING refusal — i.e. does it
/// prove a shallower service already exists?
///
/// Split out as a pure function so the classification is oracle-testable
/// against the whole error surface rather than only against whatever an
/// integration test can provoke over real transport (the transient variants
/// cannot be provoked on demand at all).
fn is_buffer_ceiling_refusal(
    e: &iceoryx2::service::builder::publish_subscribe::PublishSubscribeOpenOrCreateError,
) -> bool {
    use iceoryx2::service::builder::publish_subscribe::{
        PublishSubscribeOpenError, PublishSubscribeOpenOrCreateError,
    };
    matches!(
        e,
        PublishSubscribeOpenOrCreateError::PublishSubscribeOpenError(
            PublishSubscribeOpenError::DoesNotSupportRequestedMinBufferSize
        )
    )
}

/// Test-only ledger of control-channel CREATIONS per manager (see
/// [`TransportManager::dynamic_egress_channel_creations_for_test`]). Keyed by
/// the manager's iceoryx2 node id — unique per node for the life of the
/// process — rather than a process-global counter, because the pinning test
/// binary runs its tests in PARALLEL and every sibling test's first
/// registration creates a channel of its own. A `Vec` behind a `const`-
/// constructible `Mutex` (no `HashMap::new` in a static); the ledger is tiny.
#[cfg(any(test, feature = "test-helpers"))]
mod control_channel_creations {
    use std::sync::Mutex;

    static LEDGER: Mutex<Vec<(String, u64)>> = Mutex::new(Vec::new());

    fn key(node: &iceoryx2::node::Node<super::CerService>) -> String {
        format!("{:?}", node.id())
    }

    pub(super) fn note(node: &iceoryx2::node::Node<super::CerService>) {
        let key = key(node);
        let mut ledger = LEDGER.lock().unwrap_or_else(|e| e.into_inner());
        match ledger.iter_mut().find(|(k, _)| *k == key) {
            Some((_, n)) => *n += 1,
            None => ledger.push((key, 1)),
        }
    }

    pub(super) fn count(node: &iceoryx2::node::Node<super::CerService>) -> u64 {
        let key = key(node);
        LEDGER
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .iter()
            .find(|(k, _)| *k == key)
            .map_or(0, |(_, n)| *n)
    }
}

/// See [`TransportManager::external_publisher_probe`].
pub struct ExternalPublisherProbe {
    data_service:
        iceoryx2::service::port_factory::publish_subscribe::PortFactory<CerService, [u8], ()>,
}

impl ExternalPublisherProbe {
    /// Publishers currently attached to the topic (iceoryx2 dynamic
    /// config — live ports; a crashed producer's stale port is reclaimed by the
    /// runtime's explicit liveliness sweep, NOT on open — iceoryx2's on-open
    /// auto-cleanup is disabled).
    pub fn live_publishers(&self) -> usize {
        self.data_service.dynamic_config().number_of_publishers()
    }
}

/// An opaque dead-node reclaimer the graph
/// runtime's liveliness sweep calls once per sweep so a CRASHED publisher's
/// stale iceoryx2 port is reclaimed (and `number_of_publishers()` drops),
/// making the crash observable by the very next probe read.
///
/// A graceful disconnect (`Publisher::drop`) decrements the dynamic count
/// immediately and needs no cleanup. A crash leaves no `Drop`, so the dead
/// process's registered port lingers in `number_of_publishers()` until
/// iceoryx2's dead-node cleanup reclaims it. The transport DISABLES iceoryx2's
/// automatic on-open/create/destroy cleanup (a co-resident cdylib's separate
/// iceoryx2 copy would otherwise reap the live host node — the same-PID liveness
/// defeat), so this handle is how the runtime reclaims a crashed port at all: the
/// sweep drives `try_cleanup_dead_nodes` explicitly, once per sweep.
///
/// Holds only a CLONE of the iceoryx2 `Config` (not the `Node`): the cleanup
/// API is an associated fn keyed by config, so the runtime can reclaim dead
/// nodes WITHOUT holding the transport, and the iceoryx2 `Config`/`Node` types
/// stay confined to this module (runtime.rs sees only this opaque struct).
pub struct LivelinessCleaner {
    /// A clone of the iceoryx2 node's config (iceoryx2 `Config` is `Clone`).
    config: iceoryx2::config::Config,
    /// Test-only invocation counter so a test can prove the runtime calls
    /// `cleanup_dead_nodes` once per sweep. Cfg-gated so production builds
    /// carry zero overhead (the field doesn't exist).
    #[cfg(any(test, feature = "test-helpers"))]
    call_count: Arc<AtomicU64>,
}

impl LivelinessCleaner {
    /// Reclaim the stale system resources of all dead iceoryx2 nodes (a
    /// non-blocking `Duration::ZERO` registry scan — `try_cleanup_dead_nodes`).
    ///
    /// Cleanup failure is NON-FATAL: a crashed port simply isn't reclaimed
    /// THIS sweep (the next sweep retries). Permission/contention skips are
    /// expected, so the outcome is traced at `trace!`, never surfaced as an
    /// error.
    pub(crate) fn cleanup_dead_nodes(&self) {
        #[cfg(any(test, feature = "test-helpers"))]
        self.call_count.fetch_add(1, Ordering::Relaxed);

        // iceoryx2 0.10: `try_cleanup_dead_nodes` moved from an associated
        // function taking a `&Config` to a METHOD on `&Node`, so the sweep needs
        // a node in the namespace it is cleaning. SPIKE STAND-IN: mint a
        // transient node from the captured config for the call. That is a real
        // per-sweep cost and a design question for the migration proper (the
        // cleaner deliberately does NOT hold the transport); a node that cannot
        // be created is the same non-fatal skip a failed cleanup already was.
        let Ok(node) = NodeBuilder::new()
            .config(&self.config)
            .create::<CerService>()
        else {
            tracing::trace!("liveliness sweep: dead-node cleanup skipped (no node)");
            return;
        };
        let state = node.try_cleanup_dead_nodes();
        tracing::trace!(
            cleanups = state.cleanups,
            failed_cleanups = state.failed_cleanups,
            "liveliness sweep: dead-node cleanup pass"
        );
    }

    /// Test-only: how many times `Self::cleanup_dead_nodes` has been called
    /// (so a test can assert the runtime invokes it once per sweep).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn call_count(&self) -> u64 {
        self.call_count.load(Ordering::Relaxed)
    }

    /// Test-only: the shared invocation counter itself, so the owner can keep
    /// reading it after this cleaner has moved onto the reclaim thread.
    #[cfg(any(test, feature = "test-helpers"))]
    pub(crate) fn call_count_handle(&self) -> Arc<AtomicU64> {
        Arc::clone(&self.call_count)
    }
}

/// Singleton transport manager owning the iceoryx2 node.
///
/// Provides factory methods for creating publishers and subscribers.
/// Uses `OnceLock` for zero-overhead singleton access after initialization.
pub struct TransportManager {
    /// Field ORDER is load-bearing: `network` is declared BEFORE
    /// `node` so it drops FIRST. The `NetworkManager`'s ingress registry holds
    /// LOCAL iceoryx2 publishers (the re-injection ports); dropping the network
    /// manager first tears those ports down while the iceoryx2 `node` handle
    /// below is still alive — the normal ports-before-node teardown order.
    /// (Construction sites use named struct literals, so they are unaffected.)
    network: Option<Arc<NetworkManager>>,
    node: Node<CerService>,
    clock: Arc<dyn Clock>,
    subscriber_buffer_size: usize,
    bridge_mgr: Arc<TopicBridgeManager>,
    /// Latched-idempotent guard for [`Self::start_network_bridge_watch`]
    /// — the liveliness watch task registers exactly ONE network-subscriber ref
    /// no matter how many times graph build calls the start hook (once per
    /// egress topic is expected), so the task runs for the manager's lifetime
    /// and `NetworkManager::Drop` stops it. Fully-qualified atomics in the
    /// accessors (the module's `Ordering` import is test-gated).
    network_watch_started: std::sync::atomic::AtomicBool,
    /// Degraded-provisioning warns are
    /// once per (topic, port kind) per MANAGER (= per process under the
    /// production `init()` singleton; per-test managers warn
    /// independently) — `open_topic_services` runs for the producer's
    /// publisher and every body/trigger-drain subscriber of a graph
    /// topic, and the degradation is a property of the TOPIC's
    /// pre-existing service, not of each open. Cold path (service open
    /// only).
    degraded_warned: std::sync::Mutex<std::collections::HashSet<(String, &'static str)>>,
    /// The lazily-created runtime-egress REGISTRATION CHANNEL (writer
    /// side) — `None` until the first [`Self::register_dynamic_egress_topic`]
    /// call opens the `/__cerulion/gateway_topics` control service + starts the
    /// republish thread. A network-free worker (a `ros2 attach` robot's dds_bridge)
    /// pushes its runtime raw-route registrations over this to the separate
    /// gateway process. Held behind a `Mutex<Option<..>>` so creation is lazy +
    /// fallible (service open); the channel + its republish thread live for the
    /// manager's lifetime and are joined on manager Drop. See
    /// [`reg_channel`].
    /// `Arc` so a registration can run OUTSIDE this lock: the guard is held
    /// only to install-if-empty / clone the handle, never across the
    /// panic-capable iceoryx2 work (see `dynamic_egress_channel`).
    dynamic_egress: std::sync::Mutex<Option<Arc<reg_channel::RegistrationChannel>>>,
    /// Serializes channel CREATION only (never registration sends — those run
    /// with no lock held): held across the one open+install in
    /// `dynamic_egress_channel`, so exactly ONE control publisher is ever
    /// opened per process. N first registrations racing on an empty slot used
    /// to EACH open a publisher before one won the install, and the control
    /// service caps writers at `REG_CHANNEL_MAX_WRITERS` — a large-enough
    /// concurrent burst exhausted the cap and the losers stayed unregistered.
    dynamic_egress_create: std::sync::Mutex<()>,
    /// The INGRESS-BUILD progress channel writer — `None` until the
    /// first [`Self::note_ingress_route_created`] call (i.e. the first runtime
    /// ingress route this process creates), so a manager that never bridges
    /// anything opens no service and costs nothing. Held behind a
    /// `Mutex<Option<..>>` so creation is lazy + fallible (service open); the
    /// channel + its republish thread live for the manager's lifetime and are
    /// joined on manager Drop. See [`ingress_build`].
    ingress_build: std::sync::Mutex<Option<ingress_build::IngressBuildChannel>>,
    /// Topic -> the wire `sequence` a RESTORED publisher's
    /// first frame carries. EMPTY on every live path and on every from-start
    /// replay, so those are byte-unchanged; a mid-run restore fills it from
    /// the seed ladder before the graph build. Read once per publisher
    /// creation (cold path), never on publish. See
    /// [`Self::set_replay_sequence_seeds`].
    replay_sequence_seeds: std::sync::Mutex<std::collections::BTreeMap<String, u32>>,
    /// The lazily-created MIRROR-PROVENANCE registry (writer
    /// side) — `None` until the first [`Self::register_mirror_provenance`] call
    /// opens the `/__cerulion/mirrors` control service + starts the republish
    /// thread. A desk process that RE-INJECTS a remote topic into local SHM
    /// (vizd's `attach_remote`, `topic echo`'s auto-ingress, connectd's iroh
    /// re-inject) registers `(canonical topic, origin robot)` here so
    /// `cerulion topic list` folds the mirror into the REMOTE section instead of
    /// LOCAL (the "one data source = one topic" decision). Held behind a
    /// `Mutex<Option<..>>` so creation is lazy + fallible (service open); the
    /// registry + its republish thread live for the manager's lifetime and are
    /// joined on manager Drop (dropping = the mirror's provenance EXPIRES). See
    /// [`mirror_registry`].
    mirror_registry: std::sync::Mutex<Option<mirror_registry::MirrorRegistry>>,
    /// Construction-time self-reference (every constructor mints
    /// the manager via `Arc::new_cyclic`), so [`Self::arc_self`] can produce
    /// the OWNED `Arc` from any `&TransportManager` — the graph build chain's
    /// parameter shape. Exists so the runtime can hand the manager to
    /// `NodeContext` (context-carried transport): a CDYLIB node must NEVER
    /// resolve `TransportManager::get()` — the cdylib links its OWN copy of
    /// cerulion_core, whose `INSTANCE` static the host never initialized
    /// (the cross-linkage-unit global trap; ports already cross via
    /// NodeContext for exactly this reason).
    self_weak: std::sync::Weak<TransportManager>,
}

/// Return a clone of `config` with iceoryx2's automatic dead-node cleanup
/// DISABLED on every axis — the config EVERY [`TransportManager`] node is built
/// from must pass through here first.
///
/// iceoryx2 0.9.1 defaults all three auto-cleanup flags ON
/// (`iceoryx2-0.9.1/src/config.rs`: `global.service.cleanup_dead_nodes_on_open`,
/// `global.node.cleanup_dead_nodes_on_creation` / `_on_destruction`). On every
/// service open/create/destroy, auto-cleanup probes each registered node's
/// liveness and, judging it dead, reaps that node's services. The probe
/// short-circuits to Alive ONLY when the probing linkage unit's cached
/// `UniqueProcessId` equals the id the node's owner wrote into its context file
/// (`iceoryx2-bb-posix-0.9.1/src/process_state.rs` `ProcessMonitor::state`) — and
/// that id embeds a per-LINKAGE-UNIT counter (`unique_system_id.rs` `static
/// COUNTER`, cached in a per-linkage-unit `LazyLock`). A dlopen'd cdylib links
/// its OWN iceoryx2 copy, so its cached id never matches the host node's and the
/// probe falls through to an fcntl `F_GETLK` record-lock check. POSIX record
/// locks never conflict within one PID, so the cdylib reads the LIVE host node's
/// own `F_WRLCK` as unlocked → judges it DEAD → reaps every service the host owns
/// (observed: a cdylib raw-route service-open mid-run unlinked all 8 host topic
/// services). Runtime auto-reap is therefore forbidden; crash-recovery runs ONCE
/// at host init instead (`startup_dead_node_sweep`), where the host linkage unit
/// sees its own just-created node Alive via the same-id fast path and no peer
/// iceoryx2 copy is loaded yet.
// `pub(crate)` since `flashback::channel` mints its own transient
// node exactly as `run_registry` does, and it lives OUTSIDE this module (see
// `flashback.rs`). Every caller is still in
// this crate; widening it further would invite an out-of-crate node mint that
// skips the rest of the manager's contract.
pub(crate) fn disable_auto_dead_node_cleanup(
    mut config: iceoryx2::config::Config,
) -> iceoryx2::config::Config {
    config.global.service.cleanup_dead_nodes_on_open = false;
    config.global.node.cleanup_dead_nodes_on_creation = false;
    config.global.node.cleanup_dead_nodes_on_destruction = false;
    config
}

/// The iceoryx2 `Config` the singleton [`TransportManager::init`] path builds its
/// node from — the shared-config arm clones the supervisor-minted `Some` value,
/// the default arm starts from `iceoryx2::config::Config::global_config()` (what
/// `NodeBuilder`'s own no-config path resolves to), and BOTH pass through
/// [`disable_auto_dead_node_cleanup`] (never trust callers to have disabled the
/// flags themselves). Shared with a test seam so the arm-selection cannot drift
/// from the pins.
fn resolve_singleton_node_config(
    ix_config: Option<iceoryx2::config::Config>,
) -> iceoryx2::config::Config {
    disable_auto_dead_node_cleanup(
        ix_config.unwrap_or_else(|| iceoryx2::config::Config::global_config().clone()),
    )
}

/// Run iceoryx2's explicit dead-node reaper ONCE at host-manager init, on the
/// host linkage unit, before any cdylib is loaded.
///
/// [`disable_auto_dead_node_cleanup`] turns OFF iceoryx2's automatic
/// crash-recovery reaping to defend the live host node from a co-resident
/// cdylib's same-PID liveness defeat. This restores the crash-recovery half at
/// the ONE moment it is provably safe: at host init the only iceoryx2 nodes in
/// the process are the host's own (which the host linkage unit correctly sees
/// Alive via the same-`UniqueProcessId` fast path), so the reaper only removes
/// the stale resources of nodes left by PRIOR crashed processes. Failure is
/// non-fatal (permission/contention skips are expected); the outcome is logged
/// once as a breadcrumb.
fn startup_dead_node_sweep(config: &iceoryx2::config::Config) {
    // BOUNDED (see `dead_node_sweep`'s header for the measurement): the per-node
    // cost includes listing the whole shared-memory namespace, which on macOS is
    // a full `readdir` of `/tmp`. Unbounded, this ran before the first log line
    // of every process that builds a manager — graph, worker, bagd, netd — and
    // could silently delay bring-up by MINUTES on a host with accumulated
    // residue. `cerulion clean` keeps the unbounded sweep.
    let state = dead_node_sweep::cleanup_dead_nodes_bounded(
        config,
        dead_node_sweep::STARTUP_DEAD_NODE_SWEEP_BUDGET,
    );
    report_startup_dead_node_sweep(&state);
}

/// Report what the startup sweep did, at the level each outcome earns.
///
/// Split from [`startup_dead_node_sweep`] so the three arms can be driven by
/// hand-built [`dead_node_sweep::BoundedCleanup`] values (a real failed
/// removal needs a dead node owned by another user, which no test can mint):
/// pinned by `startup_sweep_report_*` in this module's tests.
fn report_startup_dead_node_sweep(state: &dead_node_sweep::BoundedCleanup) {
    // A removal that FAILED is a fault with an operator remedy (permissions,
    // or version skew after an upgrade) and the sweep counts it silently, so
    // this line is its only surface: it stays a warn. A sweep that only
    // succeeded (or found nothing) is startup bookkeeping and rides `debug!`.
    if state.failed_cleanups > 0 {
        tracing::warn!(
            cleaned = state.cleanups,
            failed = state.failed_cleanups,
            "startup dead-node sweep could not reclaim some dead iceoryx2 node state \
             (typically permission errors when the dead node belonged to a different user, \
             or post-upgrade version skew); run `cerulion clean` to retry"
        );
    } else {
        tracing::debug!(
            cleaned = state.cleanups,
            failed = state.failed_cleanups,
            "startup dead-node sweep (runtime auto-cleanup disabled)"
        );
    }
    if state.budget_exhausted() {
        // A SEPARATE arm from the counts above: those are nodes the sweep tried, this
        // is nodes it never reached. The remedy differs (there, permissions;
        // here, the sweep that has no deadline), so collapsing them would send
        // an operator to the wrong fix.
        tracing::warn!(
            cleaned = state.cleanups,
            deferred = state.deferred,
            budget_ms = dead_node_sweep::STARTUP_DEAD_NODE_SWEEP_BUDGET.as_millis() as u64,
            "iceoryx2 dead-node sweep ran out of its startup budget — this process left stale \
             state behind rather than delaying bring-up. It is reclaimed progressively by later \
             runs; run `cerulion clean` to sweep it all at once"
        );
    }
}

/// The SHM-service-discovery IDENTITY of an iceoryx2 `Config` — every field that
/// determines WHICH shared-memory files a pub/sub topic (and node) resolves to.
/// Two managers sharing this identity discover the SAME services; a manager on a
/// different identity cannot see the other's SHM topics at all.
///
/// `cerulion-netd`'s egress plane compares a forwarded producing
/// run's identity ([`ix_config_shm_identity_from_json`]) against netd's
/// shared-session identity ([`TransportManager::iox_shm_identity`]) before its
/// embedded gateway taps the run's topics. The identity must be COMPLETE:
/// iceoryx2 0.9.1 discovers a service's static config under
/// `root_path + global.service.directory` (default `services`), its nodes under
/// `root_path + global.node.directory` (default `nodes`), and resolves each of a
/// service's SHM files by a per-file SUFFIX
/// (`iceoryx2-0.9.1/src/service/config_scheme.rs`). Two configs sharing only
/// `(root_path, prefix)` but differing in ANY of those discover DIFFERENT
/// services — so an identity of `(root_path, prefix)` alone would report a MATCH
/// while netd taps its OWN (empty) service directory, sees none of the run's
/// topics, and the run silently egresses NOTHING while believing it registered.
/// This type therefore carries every discovery-keying field so the compare
/// cannot make that mistake.
///
/// It deliberately EXCLUDES the three dead-node-cleanup flags
/// (`service.cleanup_dead_nodes_on_open`, `node.cleanup_dead_nodes_on_creation`
/// / `_on_destruction`): `resolve_singleton_node_config` disables them on
/// netd's config while a forwarded run's `global_config().clone()` snapshot
/// (`cerulion_cli_engine`'s `mint_deployment_ix_config`) leaves them ON, so
/// folding them in would FALSE-mismatch every common-case run onto a per-run
/// child. Those flags change cleanup BEHAVIOR, never which SHM files a service
/// resolves to, so they play no part in discovery identity. The node-management
/// suffixes (monitor / tag / global-mgmt file names living UNDER `node_dir`) are
/// likewise excluded — they name node-liveness/bookkeeping files, not a service's
/// data-plane SHM; `node_dir` captures the node partitioning at the directory
/// granularity, which is the discovery boundary that matters for egress.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IceoryxShmIdentity {
    /// `global.root_path` — the root every service/node directory hangs off.
    pub root_path: String,
    /// `global.prefix` — the segment-name prefix on every SHM file.
    pub prefix: String,
    /// `global.service.directory` — the directory scanned to discover (and
    /// resolve) a service's static config file.
    pub service_dir: String,
    /// `global.node.directory` — the directory under which nodes are discovered.
    pub node_dir: String,
    /// `global.service.static_config_storage_suffix` — the service static-config
    /// file suffix (THE file `open`/`list` resolves a service by name to).
    pub service_static_config_suffix: String,
    /// `global.service.dynamic_config_storage_suffix` — the service
    /// dynamic-config file suffix.
    pub service_dynamic_config_suffix: String,
    /// `global.service.data_segment_suffix` — the payload SHM segment suffix the
    /// egress tap actually READS.
    pub service_data_segment_suffix: String,
    /// `global.service.connection_suffix` — the pub↔sub connection SHM suffix.
    pub service_connection_suffix: String,
    /// `global.service.event_connection_suffix` — the event/notify connection SHM
    /// suffix.
    pub service_event_connection_suffix: String,
}

impl IceoryxShmIdentity {
    /// Read the SHM-discovery identity out of a resolved iceoryx2 `Config`. Cold
    /// path — one call per egress register / manager query.
    fn from_config(cfg: &iceoryx2::config::Config) -> Self {
        let g = &cfg.global;
        let s = &g.service;
        Self {
            root_path: g.root_path().to_string(),
            prefix: g.prefix.to_string(),
            service_dir: s.directory.to_string(),
            node_dir: g.node.directory.to_string(),
            service_static_config_suffix: s.static_config_storage_suffix.to_string(),
            service_dynamic_config_suffix: s.dynamic_config_storage_suffix.to_string(),
            service_data_segment_suffix: s.data_segment_suffix.to_string(),
            service_connection_suffix: s.connection_suffix.to_string(),
            service_event_connection_suffix: s.event_connection_suffix.to_string(),
        }
    }
}

impl std::fmt::Display for IceoryxShmIdentity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "root_path='{}', prefix='{}', service_dir='{}', node_dir='{}', \
             suffixes[static='{}', dynamic='{}', data='{}', connection='{}', event='{}']",
            self.root_path,
            self.prefix,
            self.service_dir,
            self.node_dir,
            self.service_static_config_suffix,
            self.service_dynamic_config_suffix,
            self.service_data_segment_suffix,
            self.service_connection_suffix,
            self.service_event_connection_suffix,
        )
    }
}

/// Parse a serialized iceoryx2 `Config` JSON into its SHM-service-discovery
/// [`IceoryxShmIdentity`] — the same identity [`TransportManager::iox_shm_identity`]
/// reports for the SAME config. cerulion-netd hands a producing graph's
/// `ix_config_json` (the run's resolved iceoryx2 namespace) across the
/// `register_egress` verb; the egress plane compares THIS identity against netd's
/// shared-session manager before tapping, so a run on a DIFFERENT namespace is
/// refused (→ per-run gateway child) rather than silently tapped on the wrong one.
///
/// `Err` on an unparseable / non-`Config` JSON (a corrupt or wrong-shape
/// forward), surfaced LOUDLY by the caller — never a silent accept.
pub fn ix_config_shm_identity_from_json(json: &str) -> TransportResult<IceoryxShmIdentity> {
    let cfg: iceoryx2::config::Config =
        serde_json::from_str(json).map_err(|e| TransportError::InvalidTransportConfig {
            reason: format!("forwarded iceoryx2 Config JSON is not a valid Config: {e}"),
        })?;
    Ok(IceoryxShmIdentity::from_config(&cfg))
}

/// The outcome of a producer-count probe BEFORE it is
/// classified into the desk's liveness signal — separating a genuine count (incl. an
/// absent data service = a dead route) from a transient probe FAILURE. Lets
/// [`classify_producer_probe`] be a PURE, oracle-testable decision.
enum ProducerProbe {
    /// The data service opened; this many live publishers (`0` = a live-service dead route).
    Count(u32),
    /// The `{topic}/data` service does not exist — no producer ever created it: a dead
    /// route (the reported `/uslam/cloud_map` case), a known count of `0`.
    NoService,
    /// The probe itself FAILED (a transient iceoryx2 error — fd pressure, capacity, or
    /// an unprobeable name): the count is UNKNOWN, never a dead route.
    Failed,
}

/// PURE: classify a [`ProducerProbe`] into the desk liveness signal. A
/// KNOWN count — including a missing service (= a dead route) — is `Some`; a probe
/// FAILURE is `None` (UNKNOWN — the desk renders it as a plain row, NEVER dimmed as
/// dead, so a transient fd-pressure error can never dim a HEALTHY topic). Oracle-tested.
fn classify_producer_probe(probe: ProducerProbe) -> Option<u32> {
    match probe {
        ProducerProbe::Count(n) => Some(n),
        ProducerProbe::NoService => Some(0),
        ProducerProbe::Failed => None,
    }
}

/// How often
/// [`TransportManager::topic_publisher_count_checked`] may emit its probe-failure
/// `warn!`. Sustained iceoryx2 resource pressure would otherwise warn once per
/// catalogued topic per ~2 s sidebar refresh; a coarse debounce keeps the signal
/// LOUD-ONCE without flooding.
const PRODUCER_PROBE_WARN_DEBOUNCE: std::time::Duration = std::time::Duration::from_secs(30);

/// Emit the producer-probe-failure `warn!` at most once per
/// [`PRODUCER_PROBE_WARN_DEBOUNCE`] (process-global — fd pressure is a machine-wide
/// condition), carrying the offending topic + the iceoryx2 error.
fn debounced_producer_probe_warn(topic: &str, err: &dyn std::fmt::Display) {
    static LAST_WARN: std::sync::Mutex<Option<std::time::Instant>> = std::sync::Mutex::new(None);
    let now = std::time::Instant::now();
    let mut last = LAST_WARN.lock().unwrap_or_else(|e| e.into_inner());
    let due = last.is_none_or(|t| now.duration_since(t) >= PRODUCER_PROBE_WARN_DEBOUNCE);
    if due {
        *last = Some(now);
        drop(last);
        tracing::warn!(
            topic = %topic,
            error = %err,
            "producer-count probe failed for a catalogued topic — reporting UNKNOWN \
             liveness (the row renders plainly, never dimmed as dead); usually transient iceoryx2 \
             resource pressure (e.g. fd limits)"
        );
    }
}

impl TransportManager {
    /// Initialize the transport manager with the given configuration.
    ///
    /// Must be called once before `get()`. Subsequent calls return `Ok(Arc)` to
    /// the existing instance (config is ignored — but still validated: a zero
    /// `subscriber_buffer_size`, or a `node_name` iceoryx2 cannot represent,
    /// errs even when the singleton already exists).
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] if
    ///   [`TransportConfig::subscriber_buffer_size`] is out of range.
    /// - [`TransportError::UnrepresentableNodeName`] if
    ///   [`TransportConfig::node_name`] — derived from the graph's identity — is
    ///   too long for, or outside the charset of, an iceoryx2 node name. An
    ///   `.expect()` here would abort the process with no exit code and no
    ///   diagnostic on a path every `graph run` and every resim shares.
    #[must_use = "transport initialization result must be checked"]
    pub fn init(config: TransportConfig) -> TransportResult<Arc<Self>> {
        Self::init_singleton(config, None)
    }

    /// Initialize the singleton on an EXPLICIT, shared iceoryx2 `Config` — the
    /// multi-process worker path.
    ///
    /// Every worker of a multi-process deployment is handed the SAME minted
    /// `Config` (a graph+nonce prefix) by the supervisor and calls this, so all
    /// workers land on ONE shared iceoryx2 namespace and their pub/sub connect.
    /// Unlike `init_for_test` (the test-only constructor, which returns a FRESH,
    /// non-singleton manager per call so parallel tests don't collide), this IS
    /// the process singleton — one worker = one process = one manager — so
    /// [`get`](Self::get) works throughout the worker. Like [`init`](Self::init),
    /// the config is honored only on the first (initializing) call.
    #[must_use = "transport initialization result must be checked"]
    pub fn init_with_config(
        config: TransportConfig,
        ix_config: iceoryx2::config::Config,
    ) -> TransportResult<Arc<Self>> {
        Self::init_singleton(config, Some(ix_config))
    }

    /// Shared singleton constructor behind [`init`](Self::init) (`ix_config =
    /// None` → iceoryx2's global default; single-process monolith) and
    /// [`init_with_config`](Self::init_with_config) (`Some` → a supervisor-minted
    /// shared config; multi-process workers). The node builder pins the explicit
    /// config only when one is handed; otherwise it is byte-for-byte the original
    /// default-config `init` path.
    fn init_singleton(
        config: TransportConfig,
        ix_config: Option<iceoryx2::config::Config>,
    ) -> TransportResult<Arc<Self>> {
        // Install the contained-panic hook suppressor at
        // manager construction — the one point provably BEFORE a state-carrier
        // capture fork is reachable (arming a recorder needs a live manager),
        // so the `Once`-guarded `set_hook` WRITE never races a fork. See the
        // hook-suppression section in `reg_channel.rs` and the invariant in
        // `state_carrier/hook.rs`.
        reg_channel::install_contained_panic_hook_suppressor();
        validate_subscriber_buffer_size(config.subscriber_buffer_size)?;
        // The node name is converted HERE, before `get_or_init`, and
        // not inside the closure. Two reasons:
        //
        // * `get_or_init`'s closure cannot return an `Err` — a fallible
        //   conversion inside it can only panic.
        // * The closure runs only on the FIRST init, so an unrepresentable name
        //   handed to a LATER call would be silently ignored rather than refused.
        //   Validating up front makes the verdict order-independent: the same
        //   config gets the same answer whether or not this process has already
        //   initialised. A name that cannot identify a node is a broken config
        //   even when this call would have reused an existing manager.
        //
        // Nothing has been created at this point, so a refusal leaves no
        // transport state behind: `INSTANCE` stays unset.
        let node_name = node_name_or_refusal(config.node_name.as_str())?;
        // Defensive: a supervisor-minted shared config is honored ONLY on
        // the singleton's FIRST init. If the process already initialized the
        // transport (e.g. a stray default-config `init()`), `get_or_init` below
        // silently returns THAT manager and IGNORES `ix_config` — the worker would
        // then be on the WRONG iceoryx2 namespace and its cross-group pub/sub would
        // silently never connect. The worker entry is the first init today (so this
        // is latent), but warn loudly at the boundary so a silent misroute can never
        // be introduced quietly later.
        if ix_config.is_some() && INSTANCE.get().is_some() {
            tracing::warn!(
                "init_with_config: transport singleton already initialized — the supplied shared \
                 iceoryx2 config is IGNORED; this process keeps its existing namespace and may fail \
                 to connect to co-resident workers"
            );
        }
        let manager = INSTANCE.get_or_init(|| {
            // Suppress iceoryx2 "No config file was loaded" warning. Cerulion uses
            // iceoryx2's defaults intentionally — no custom config file is needed.
            crate::iceoryx_logger::init_iceoryx_log_level_from_env();

            // Pre-warm the reader-side topic sentinel
            // so the first subscriber `try_view` does not allocate inside
            // the zero-alloc critical section. The OnceLock inside
            // `read_only_topic` allocates once on first call; doing it
            // here (during one-time transport init) keeps `Arc::clone` as
            // the only cost on the subscriber hot path.
            let _ = crate::message::read_only_topic();

            // Every node this manager builds runs on a config with
            // iceoryx2's automatic dead-node cleanup DISABLED (the same-PID cdylib
            // reap defeat — see disable_auto_dead_node_cleanup). The shared-config
            // arm pins the supervisor-minted config (multi-process workers); the
            // default arm is byte-identical to NodeBuilder's own no-config path
            // (Config::global_config()) — resolve_singleton_node_config disables
            // the flags on either.
            let shared_config = ix_config.is_some();
            let ix_config = resolve_singleton_node_config(ix_config);
            let node = NodeBuilder::new()
                .name(&node_name)
                .config(&ix_config)
                .create::<CerService>()
                .expect("failed to create iceoryx2 node");

            // Restore iceoryx2's crash-recovery reaping at the ONE moment
            // it is safe — host init, before any cdylib is loaded, where the host
            // linkage unit sees its own just-created node Alive via the
            // same-UniqueProcessId fast path (runtime auto-reap stays off).
            startup_dead_node_sweep(&ix_config);

            // Detect operator-set Linux configs (shmem THP, bounded
            // RLIMIT_AS, near-exhausted vm.max_map_count) that break our
            // oversize-freely SHM assumption and warn loudly. Linux-only,
            // detect-and-warn only (never mutates a machine setting, never aborts);
            // no-op off Linux. Cold path — runs once per process at init.
            shm_guard::check_and_warn_shm_safety();

            // Lifecycle bookkeeping (every process that builds a manager would
            // otherwise print it): `debug!`.
            tracing::debug!(
                node_name = %config.node_name,
                shared_config,
                "transport manager initialized"
            );

            let network = config.network.map(|c| Arc::new(NetworkManager::new(c)));

            Arc::new_cyclic(|self_weak| Self {
                node,
                clock: config.clock,
                subscriber_buffer_size: config.subscriber_buffer_size,
                network,
                bridge_mgr: Arc::new(TopicBridgeManager::new()),
                degraded_warned: std::sync::Mutex::new(std::collections::HashSet::new()),
                network_watch_started: std::sync::atomic::AtomicBool::new(false),
                dynamic_egress: std::sync::Mutex::new(None),
                dynamic_egress_create: std::sync::Mutex::new(()),
                replay_sequence_seeds: std::sync::Mutex::new(std::collections::BTreeMap::new()),
                mirror_registry: std::sync::Mutex::new(None),
                ingress_build: std::sync::Mutex::new(None),
                self_weak: self_weak.clone(),
            })
        });
        Ok(Arc::clone(manager))
    }

    /// Initialize with default configuration.
    #[must_use = "transport initialization result must be checked"]
    pub fn init_default() -> TransportResult<Arc<Self>> {
        Self::init(TransportConfig::default())
    }

    /// Initialize the transport singleton pinned to an iceoryx2 `Config`
    /// deserialized from its JSON serialization.
    ///
    /// A thin wrapper over [`init_with_config`](Self::init_with_config) that
    /// deserializes the config here, so a downstream crate can join an explicit
    /// iceoryx2 SHM namespace WITHOUT taking a direct `iceoryx2` dependency (it
    /// only handles the opaque JSON string). This is the `bagd` recorder's
    /// hidden `--test-iox2-config` seam: a recorder launched by an integration
    /// test joins that test's ISOLATED per-test SHM namespace (the same
    /// serialized `Config` the multi-process supervisor hands its workers).
    ///
    /// # Errors
    ///
    /// [`TransportError::InvalidTransportConfig`] if `json` is not a valid
    /// serialized iceoryx2 `Config`; otherwise forwards
    /// [`init_with_config`](Self::init_with_config)'s result.
    #[must_use = "transport initialization result must be checked"]
    pub fn init_with_ix_config_json(
        config: TransportConfig,
        json: &str,
    ) -> TransportResult<Arc<Self>> {
        let ix_config: iceoryx2::config::Config =
            serde_json::from_str(json).map_err(|e| TransportError::InvalidTransportConfig {
                reason: format!("could not parse iceoryx2 Config from JSON: {e}"),
            })?;
        Self::init_with_config(config, ix_config)
    }

    /// Test-only: create a fresh `TransportManager` with a custom
    /// iceoryx2 `Config` (typically rooted at a `TempDir`-backed
    /// path via `iceoryx2::config::Config::set_root_path`). Each
    /// call returns an INDEPENDENT manager — does NOT consult the
    /// `OnceLock` singleton, so multiple per-test managers can
    /// coexist in the same process without colliding on iceoryx2's
    /// process-shared SHM region.
    ///
    /// Use the `cerulion_core::testing::iceoryx_test_config` helper
    /// to construct a per-test `Config` (iceoryx2's
    /// `generate_isolated_config` — a unique prefix under the shared
    /// test directory; no TempDir, no Drop cleanup).
    ///
    /// Lets `cargo test --workspace` run iceoryx-touching tests in
    /// parallel by giving each test its own SHM root. Without this,
    /// every iceoryx test must run with `--test-threads=1` to avoid
    /// clashing on `/tmp/iceoryx2/`.
    #[cfg(any(test, feature = "test-helpers"))]
    #[must_use = "transport initialization result must be checked"]
    pub fn init_for_test(
        config: TransportConfig,
        ix_config: iceoryx2::config::Config,
    ) -> TransportResult<Arc<Self>> {
        // Mirror of `init_singleton` — see the comment there.
        reg_channel::install_contained_panic_hook_suppressor();
        validate_subscriber_buffer_size(config.subscriber_buffer_size)?;
        crate::iceoryx_logger::init_iceoryx_log_level_from_env();

        // Mirror of `init()`: pre-warm the
        // reader-side topic sentinel so zero-alloc tests using a
        // per-test SHM root start from a primed OnceLock state.
        let _ = crate::message::read_only_topic();

        // Fallible here too. `init_for_test` already returns a
        // `TransportResult`, so the `.expect()` bought nothing — and a test
        // fixture minting a hostile name is exactly where the refusal has to be
        // observable rather than an abort.
        let node_name = node_name_or_refusal(config.node_name.as_str())?;
        // Disable iceoryx2 auto dead-node cleanup on this per-test node
        // too (the never-trust-callers rule applies to test configs). No startup
        // sweep — a per-test manager is isolated per SHM root and short-lived.
        let ix_config = disable_auto_dead_node_cleanup(ix_config);
        let node = NodeBuilder::new()
            .name(&node_name)
            .config(&ix_config)
            .create::<CerService>()
            .expect("failed to create iceoryx2 node for test");

        // Mirror `init()`: surface SHM-safety warnings once per
        // per-test manager (Linux-only; no-op off Linux).
        shm_guard::check_and_warn_shm_safety();

        let network = config.network.map(|c| Arc::new(NetworkManager::new(c)));

        // NOT stored in `INSTANCE` — fresh per-call so multiple
        // tests can each have their own SHM region.
        Ok(Arc::new_cyclic(|self_weak| Self {
            node,
            clock: config.clock,
            subscriber_buffer_size: config.subscriber_buffer_size,
            network,
            bridge_mgr: Arc::new(TopicBridgeManager::new()),
            degraded_warned: std::sync::Mutex::new(std::collections::HashSet::new()),
            network_watch_started: std::sync::atomic::AtomicBool::new(false),
            dynamic_egress: std::sync::Mutex::new(None),
            dynamic_egress_create: std::sync::Mutex::new(()),
            replay_sequence_seeds: std::sync::Mutex::new(std::collections::BTreeMap::new()),
            mirror_registry: std::sync::Mutex::new(None),
            ingress_build: std::sync::Mutex::new(None),
            self_weak: self_weak.clone(),
        }))
    }

    /// Create a FRESH, DETACHED `TransportManager` on an explicit
    /// iceoryx2 `Config` — a PRODUCTION second in-process iceoryx2 node that is
    /// deliberately NOT the process singleton.
    ///
    /// This is the always-compiled production twin of the test-only
    /// `init_for_test`: same fresh-per-call, non-singleton
    /// shape (it does NOT consult or populate the `INSTANCE` `OnceLock`), but
    /// available outside `cfg(test)`. It exists for the multi-process
    /// SUPERVISOR's DATA-PLANE pre-creation: the supervisor's own
    /// [`get`](Self::get) singleton is its PLANNING manager (the `cer_p_`
    /// namespace it builds the full graph against), but pre-creating the workers'
    /// graph-owned services requires a node on the WORKERS' shared runtime
    /// namespace (the minted `cer_r_`-style `Config` handed to `run-worker`
    /// children). The supervisor mints THAT node here, pre-creates every owned
    /// topic's services on it (holding the [`PrecreatedService`] handles for the
    /// workers' lifetime), and keeps its planning singleton untouched.
    ///
    /// Two iceoryx2 nodes in one process is INTENTIONAL here and must never be
    /// reached through [`get`](Self::get): callers hold the returned `Arc`
    /// directly. The caller passes the node's identity via
    /// [`TransportConfig::node_name`]; give it a role-distinct name (e.g. a
    /// `_data` suffix) for OBSERVABILITY — node listings and monitoring then
    /// attribute each in-process node to its job. That is hygiene, NOT
    /// collision avoidance: iceoryx2 keys node identity by a counter-derived
    /// `UniqueNodeId`, never by name (verified against the vendored 0.9.1
    /// source — `NodeCreationFailure` has no name-collision variant), and
    /// same-named nodes coexist routinely ([`TransportConfig::default`] names
    /// EVERY default-ns Cerulion process literally "cerulion"). Pass the
    /// shared workers' `Config` as `ix_config` so the pre-created services
    /// land where the workers will open them.
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] if
    ///   [`TransportConfig::subscriber_buffer_size`] is out of range.
    /// - [`TransportError::UnrepresentableNodeName`] if
    ///   [`TransportConfig::node_name`] is not a valid iceoryx2 node name
    ///   (the same refusal every constructor serves).
    /// - [`TransportError::NodeCreation`] if the iceoryx2 node cannot be created
    ///   on the supplied config. Per vendored 0.9.1, `NodeCreationFailure` is
    ///   exactly InsufficientPermissions / InternalError / SystemCorrupted —
    ///   a duplicate node NAME is NOT among them (identity is the
    ///   counter-derived `UniqueNodeId`). Every constructor returns a loud `Err`
    ///   here — a supervisor must be able to fail the deployment cleanly instead
    ///   of aborting the process.
    #[must_use = "transport initialization result must be checked"]
    pub fn detached_with_config(
        config: TransportConfig,
        ix_config: iceoryx2::config::Config,
    ) -> TransportResult<Arc<Self>> {
        // Mirror of `init_singleton` — see the comment there.
        reg_channel::install_contained_panic_hook_suppressor();
        validate_subscriber_buffer_size(config.subscriber_buffer_size)?;
        crate::iceoryx_logger::init_iceoryx_log_level_from_env();

        // Mirror of `init()` / `init_for_test()` — pre-warm the reader-side topic
        // sentinel so a subscriber `try_view` on this detached manager does not
        // allocate inside the zero-alloc critical section.
        let _ = crate::message::read_only_topic();

        // This constructor was ALREADY fallible here — it is the
        // precedent the two `.expect()` sites should have followed. It now goes
        // through the same helper, so all three constructors diagnose an
        // unrepresentable identity identically (length vs charset, with the two
        // numbers on the length arm) instead of one of them serving a bare
        // `{e:?}` behind a generic config error.
        let node_name = node_name_or_refusal(config.node_name.as_str())?;
        // Disable iceoryx2 auto dead-node cleanup on this detached
        // production node too — never trust the caller's config. No startup sweep:
        // the process singleton (the supervisor's planning manager) owns
        // crash-recovery for the deployment.
        let ix_config = disable_auto_dead_node_cleanup(ix_config);
        let node = NodeBuilder::new()
            .name(&node_name)
            .config(&ix_config)
            .create::<CerService>()
            .map_err(|e| TransportError::NodeCreation {
                node_name: config.node_name.clone(),
                reason: format!("{e:?}"),
            })?;

        // Mirror `init()` — surface SHM-safety warnings once per detached manager
        // (Linux-only; no-op off Linux).
        shm_guard::check_and_warn_shm_safety();

        let network = config.network.map(|c| Arc::new(NetworkManager::new(c)));

        // NOT stored in `INSTANCE` — the caller owns this handle; the process
        // singleton (the supervisor's planning manager) is untouched.
        Ok(Arc::new_cyclic(|self_weak| Self {
            node,
            clock: config.clock,
            subscriber_buffer_size: config.subscriber_buffer_size,
            network,
            bridge_mgr: Arc::new(TopicBridgeManager::new()),
            degraded_warned: std::sync::Mutex::new(std::collections::HashSet::new()),
            network_watch_started: std::sync::atomic::AtomicBool::new(false),
            dynamic_egress: std::sync::Mutex::new(None),
            dynamic_egress_create: std::sync::Mutex::new(()),
            replay_sequence_seeds: std::sync::Mutex::new(std::collections::BTreeMap::new()),
            mirror_registry: std::sync::Mutex::new(None),
            ingress_build: std::sync::Mutex::new(None),
            self_weak: self_weak.clone(),
        }))
    }

    /// Get the existing transport manager.
    ///
    /// Returns `Err(NotInitialized)` if `init()` has not been called.
    #[must_use = "transport manager result must be checked"]
    pub fn get() -> TransportResult<Arc<Self>> {
        INSTANCE
            .get()
            .cloned()
            .ok_or(TransportError::NotInitialized)
    }

    /// Get existing or initialize with defaults.
    #[must_use = "transport manager result must be checked"]
    pub fn get_or_init() -> TransportResult<Arc<Self>> {
        if let Some(existing) = INSTANCE.get() {
            return Ok(Arc::clone(existing));
        }
        Self::init_default()
    }

    /// Drift-guard (test-only): create a DEFAULT-config
    /// publish-subscribe service for `topic` (no
    /// `subscriber_max_borrowed_samples` override) and read its actual
    /// `subscriber_max_borrowed_samples` from the iceoryx2 static config. Lets
    /// a test pin [`ICEORYX2_DEFAULT_BORROWED_SAMPLES`] against the LIVE
    /// upstream default — an iceoryx2 default change would otherwise silently
    /// break the held(+1)=[`SUBSCRIBER_MAX_BORROWED_HELD`] borrow budget. Pass
    /// a unique topic so re-runs never open a service some other test created
    /// with a non-default override.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn default_subscriber_max_borrowed_samples_for_test(&self, topic: &str) -> usize {
        let name: ServiceName = format!("{topic}/data")
            .as_str()
            .try_into()
            .expect("valid service name");
        let svc = self
            .node
            .service_builder(&name)
            .publish_subscribe::<[u8]>()
            .open_or_create()
            .expect("create default service");
        svc.static_config().subscriber_max_borrowed_samples()
    }

    /// Create a publisher for the given topic with native history support.
    ///
    /// # Arguments
    ///
    /// - `topic` — Topic name (max 249 characters)
    /// - `max_slice_len` — Maximum message size (`MaxSliceLen` newtype
    ///   enforces both `>= WireHeader::SIZE` and `<= u32::MAX` at
    ///   construction — pathological values are unrepresentable).
    /// - `history_size` — Number of most-recent frames iceoryx2 NATIVE
    ///   publisher history retains for late-joiners (0 = disabled).
    ///   For this raw/non-graph entry point the value is threaded
    ///   into the (otherwise default) [`TopicServiceConfig`] so THIS publisher
    ///   — the service CREATOR via `open_or_create` — arms native history.
    ///   (The graph path instead passes a topology-derived config carrying
    ///   the topic's history; see [`Self::create_publisher_with_topic_config`].)
    ///
    /// # Errors
    ///
    /// - `PublisherCreation` if topic name exceeds 249 characters
    /// - `PublisherCreation` if iceoryx2 service or port creation fails
    #[must_use = "publisher creation result must be checked"]
    pub fn create_publisher(
        &self,
        topic: &str,
        max_slice_len: MaxSliceLen,
        history_size: usize,
    ) -> TransportResult<CerulionPublisher> {
        // Arm native history on the service THIS publisher
        // creates by folding the requested history into the default config —
        // a raw/test `create_publisher(topic, msl, N)` is the creator
        // (`open_or_create`), so the service it makes carries the history.
        let mut topic_config = self.default_topic_config();
        topic_config.history_size = history_size;
        self.create_publisher_with_topic_config(topic, max_slice_len, history_size, topic_config)
    }

    /// [`Self::create_publisher`] with an explicit
    /// per-topic service config (see [`TopicServiceConfig`] — the graph
    /// runtime passes the topology-derived value so every opener of the
    /// topic agrees on the service settings).
    #[must_use = "publisher creation result must be checked"]
    pub fn create_publisher_with_topic_config(
        &self,
        topic: &str,
        max_slice_len: MaxSliceLen,
        history_size: usize,
        topic_config: TopicServiceConfig,
    ) -> TransportResult<CerulionPublisher> {
        let topic_owned = topic.to_string();
        let map_err = move |reason: String| TransportError::PublisherCreation {
            topic: topic_owned.clone(),
            reason,
        };
        let (data_service, event_service) =
            self.open_topic_services(topic, &map_err, topic_config)?;
        self.finish_publisher(
            topic,
            &data_service,
            event_service,
            max_slice_len,
            history_size,
            topic_config,
            &map_err,
        )
    }

    /// The publisher-port tail shared by
    /// [`Self::create_publisher_with_topic_config`] (the CREATOR open via
    /// [`Self::open_topic_services`]) and
    /// [`Self::create_publisher_on_existing_service`] (OPEN-ONLY on an owned
    /// service already made by [`Self::precreate_topic_services`]) — the
    /// single-writer active-publisher pre-check, the publisher port, the
    /// event notifier + listener, and the history-vs-buffer warn. Mirrors
    /// [`Self::finish_subscriber`] on the subscriber side. Publishers
    /// carry NO network state — a graph process is network-free; the separate
    /// [`crate::transport::gateway::GatewayRuntime`] taps produced topics for
    /// egress.
    #[allow(clippy::too_many_arguments)]
    fn finish_publisher(
        &self,
        topic: &str,
        data_service: &iceoryx2::service::port_factory::publish_subscribe::PortFactory<
            CerService,
            [u8],
            (),
        >,
        // Taken BY VALUE — the publisher retains it for the
        // notify-elision gate's live `number_of_listeners()` read. The notifier
        // + listener below are still minted from it (by `&`) before it moves
        // into the config. All three callers pass their owned local factory,
        // for which `finish_publisher` was already the last use.
        event_service: iceoryx2::service::port_factory::event::PortFactory<CerService>,
        max_slice_len: MaxSliceLen,
        history_size: usize,
        topic_config: TopicServiceConfig,
        map_err: &impl Fn(String) -> TransportError,
    ) -> TransportResult<CerulionPublisher> {
        // Cross-graph same-topic guard: iceoryx2's port cap
        // enforces single-writer only when WE created the service — open
        // verification is at-least, so on a PRE-EXISTING service with
        // spare publisher slots (e.g. a default opener created it at the
        // iceoryx2 default of 2; the degraded-provisioning warn in
        // `open_topic_services` flags exactly this) a second graph's
        // publisher would attach SILENTLY. Refuse to take a slot when the
        // live service already has an attached publisher. Best-effort by
        // construction (a concurrent attach can race the check — the port
        // cap remains the hard backstop on services we created), but the
        // realistic collision — two same-prefix graphs starting up with
        // the same derived or overridden topic — is static and caught.
        // Keyed on the declared intent (the enum, not
        // the `Some(1)` magnitude it happens to derive): External topics
        // admit foreign publishers by design, and the multi-publisher
        // opt-in's looser provisioning extends the enum without silently
        // dodging a magic-value comparison.
        if topic_config.publisher_provisioning == PublisherProvisioning::SingleWriter {
            let live_publishers = data_service.dynamic_config().number_of_publishers();
            if live_publishers > 0 {
                return Err(map_err(format!(
                    "the topic already has {live_publishers} attached publisher(s) — \
                     graph topics are single-writer (one in-graph producer). Likely \
                     cause: another graph (or another instance of this one) sharing \
                     the prefix publishes the same topic — the same node-id/output-name \
                     derives the same topic name, and `topic:` overrides collide \
                     verbatim. If multiple publishers are intentional, list the \
                     topic in the graph's `multi_publisher_topics:` (the explicit \
                     multi-publisher opt-in; absolute names only). If you just \
                     restarted a CRASHED graph, this may be its stale publisher \
                     port not yet reclaimed (iceoryx2's on-open auto-cleanup \
                     is disabled, so the port is reclaimed by the runtime's \
                     liveliness sweep or the next host-init dead-node sweep, not \
                     on this open) — retry after a moment; if it persists, \
                     remove the stale iceoryx2 shared-memory artifacts"
                )));
            }
        }

        // The loan-knob guard twins of the
        // `open_topic_services` port-cap guards, at THIS choke point because
        // `max_loaned_samples` is a PORT-level value both publisher entries
        // (creator + open-on-existing) route through here. Zero would make
        // EVERY loan fail at runtime (silent-broken publisher); a garbage
        // value becomes a giant pool reservation failing as a bare create
        // error instead of a clear rejection.
        if topic_config.publisher_max_loaned_samples == Some(0) {
            return Err(map_err(
                "topic_config.publisher_max_loaned_samples must be >= 1 when \
                 set (a zero loan budget would fail every loan)"
                    .to_string(),
            ));
        }
        if topic_config
            .publisher_max_loaned_samples
            .is_some_and(|n| n > MAX_REASONABLE_PORTS)
        {
            return Err(map_err(format!(
                "topic_config.publisher_max_loaned_samples must be <= \
                 {MAX_REASONABLE_PORTS} (each unit adds one slice-ceiling-sized \
                 slot to the publisher's data segment)"
            )));
        }

        // The iceoryx2 `initial_max_slice_len` setter takes `usize`; cast at the
        // boundary. Lossless on all platforms (u32 ⊆ usize ≥ u32).
        // A second publisher on a single-writer graph topic fails
        // HERE (port creation) — name the cause and the contract instead
        // of the bare variant + the generic Display trailer (the same
        // treatment the subscriber-slot twin gets).
        let mut publisher_builder = data_service
            .publisher_builder()
            .initial_max_slice_len(max_slice_len.get() as usize);
        if let Some(max_loaned) = topic_config.publisher_max_loaned_samples {
            // Raise the simultaneous-loan budget above the iceoryx2
            // default (2) — the rmw borrow window holds ~4 outstanding loans per
            // adoption-enabled publisher. Port-level: no open-time
            // requirement on any other opener; the cost is one APPARENT
            // slice-ceiling slot per unit in this publisher's demand-paged
            // pool (see the field doc).
            publisher_builder = publisher_builder.max_loaned_samples(max_loaned);
        }
        let publisher = publisher_builder.create().map_err(|e| {
            use iceoryx2::port::publisher::PublisherCreateError;
            let hint = if matches!(e, PublisherCreateError::ExceedsMaxSupportedPublishers) {
                // Branch on the live provisioned
                // count — the single-writer claim is only true for
                // single-writer services, and "a publisher already
                // holds" deliberately avoids asserting WHO holds the
                // slot (a rogue process may have grabbed it before the
                // graph's own producer — naming the graph's producer
                // would point diagnosis away from the rogue).
                let slots = data_service.static_config().max_publishers();
                if slots == 1 {
                    " — a publisher already holds the topic's only slot \
                         (graph topics are provisioned single-writer: one \
                         in-graph producer; multi-publisher topics need the \
                         explicit `multi_publisher_topics:` opt-in)"
                        .to_string()
                } else {
                    format!(" — all {slots} of the topic's publisher slots are attached")
                }
            } else {
                String::new()
            };
            map_err(format!("{e}{hint}"))
        })?;

        let notifier = event_service.notifier_builder().create().map_err(|e| {
            use iceoryx2::port::notifier::NotifierCreateError;
            let hint = if matches!(e, NotifierCreateError::ExceedsMaxSupportedNotifiers) {
                event_port_exhaustion_hint(
                    "notifier",
                    event_service.static_config().max_notifiers(),
                )
            } else {
                String::new()
            };
            map_err(format!("{e}{hint}"))
        })?;

        // Publisher also listens for subscriber events (SubscriberConnected, etc.)
        let listener = event_service.listener_builder().create().map_err(|e| {
            use iceoryx2::port::listener::ListenerCreateError;
            let hint = if matches!(e, ListenerCreateError::ExceedsMaxSupportedListeners) {
                event_port_exhaustion_hint(
                    "listener",
                    event_service.static_config().max_listeners(),
                )
            } else {
                String::new()
            };
            map_err(format!("{e}{hint}"))
        })?;

        if history_size > 0 && history_size > topic_config.subscriber_max_buffer_size * 3 / 4 {
            tracing::warn!(
                topic = %topic,
                history_size,
                subscriber_max_buffer_size = topic_config.subscriber_max_buffer_size,
                "history_size exceeds 75% of the topic's subscriber buffer ceiling — \
                 late-joiners may miss entries"
            );
        }

        tracing::debug!(
            topic = %topic,
            max_slice_len = max_slice_len.get(),
            history_size,
            "publisher created"
        );

        // The pool SHM is now created & mapped,
        // so its VMAs are visible in /proc/self/maps. Advise MADV_NOHUGEPAGE on
        // our iceoryx2 pool mappings to stop a small write from faulting in a
        // whole 2MiB huge page (up to ~512x resident inflation on oversized
        // pools). Best-effort, surgical (our pools only), no-op off Linux.
        // Publisher site: the pool was JUST mapped, so a zero-VMA scan here is a
        // genuine anomaly (broken scan) and DOES warn-once. The scan matches on
        // the ACTIVE config's segment prefix (custom multi-process
        // prefixes would lose the mitigation under a hardcoded default).
        shm_guard::advise_shm_pools_no_hugepage(
            shm_guard::PoolAdviseSite::Publisher,
            &self.shm_segment_prefix(),
        );

        Ok(CerulionPublisher::new(CerulionPublisherConfig {
            // Topic as `Arc<str>`. Single alloc per
            // publisher; every `loan_proxy` does just an atomic refcount
            // bump instead of the raw-pointer reborrow `unsafe` block.
            topic: Arc::from(topic),
            publisher,
            notifier,
            listener,
            clock: Arc::clone(&self.clock),
            max_slice_len,
            history_size,
            // The history the service
            // really provides, read back rather than assumed equal to the
            // request. iceoryx2's `.history_size(N)` open verification is
            // AT-LEAST, so this open may have attached to a service someone
            // created DEEPER — and the port it just handed us is sized from
            // this static config, so that publisher really retains the
            // deeper value.
            provisioned_history_size: data_service.static_config().history_size(),
            // Hand the topic's event-service factory to the
            // publisher (last use of the owned local) so the notify-elision gate
            // can read its live `number_of_listeners()`.
            event_service,
            // 0 on every live path; a RESTORED replay seeds
            // the recorded stream's next sequence so its first frame continues
            // the numbering the bag carries instead of restarting at 0 (which
            // the byte diff would report as a mismatch on every frame).
            initial_sequence: self.replay_sequence_seed(
                topic,
                topic_config.publisher_provisioning,
                &map_err,
            )?,
        }))
    }

    /// Declare the wire sequences restored publishers start at.
    ///
    /// The map is topic -> the sequence that topic's FIRST replayed frame must
    /// carry, derived by
    /// [`cerulion_core::state_restore::seed_publisher_sequence`](crate::state_restore::seed_publisher_sequence).
    /// Set on the manager the replay mints, BEFORE the graph build that creates
    /// the publishers; a topic with no entry starts at 0 exactly as it always
    /// has, so a from-start replay and every live run are byte-unchanged.
    ///
    /// Set-once-before-build by contract. It is a manager-level table rather
    /// than a build argument because the publisher is created deep inside the
    /// build and then MOVED into the node's `NodeContext` — after which a
    /// cdylib node owns it and the host can never reach it again (the cdylib
    /// wall). Threading it through the transport is what makes the seed reach
    /// every entry kind without an FFI export.
    ///
    /// The table is keyed by TOPIC while the counter it seeds is PER PUBLISHER,
    /// so an entry for a `multi_publisher_topics:`-listed topic is refused —
    /// loudly, at publisher creation — rather than applied to every writer (the
    /// private `replay_sequence_seed` reader carries the rule and its
    /// reasoning).
    pub fn set_replay_sequence_seeds(&self, seeds: std::collections::BTreeMap<String, u32>) {
        match self.replay_sequence_seeds.lock() {
            Ok(mut guard) => *guard = seeds,
            Err(poisoned) => *poisoned.into_inner() = seeds,
        }
    }

    /// The seed declared for `topic`, or 0 (the live default).
    ///
    /// # A MULTI-publisher topic refuses its seed here, loudly
    ///
    /// The table is keyed by TOPIC while the wire `sequence` is a PER-PUBLISHER
    /// commit counter, so on a `multi_publisher_topics:`-listed topic (the
    /// [`PublisherProvisioning::Multi`] arm — that opt-in is exactly what
    /// selects it) one seed would be handed to every publisher and be wrong for
    /// at least one of them. The pure layer already refuses to DERIVE such a
    /// seed ([`crate::state_restore::seed_publisher_sequence`]); this is the
    /// structural backstop at the one place a seed can actually reach a
    /// counter, so a future adapter that forgets the rule fails at build rather
    /// than shipping a replay whose every frame on that topic is a byte
    /// mismatch.
    ///
    /// Unreachable on every live path — the table is EMPTY unless a replay
    /// called [`Self::set_replay_sequence_seeds`], so this cannot refuse a run
    /// that never asked for a seed. `External` (the raw/CLI/`default_topic_config`
    /// arm) and `SingleWriter` are both seedable: a replay only ever seeds the
    /// topics it RE-EXECUTES, which are the graph's own produced topics, and an
    /// External topic has no in-graph producer at all — its frames are injected
    /// verbatim from the bag, carrying their own sequences.
    fn replay_sequence_seed(
        &self,
        topic: &str,
        publisher_provisioning: PublisherProvisioning,
        map_err: &impl Fn(String) -> TransportError,
    ) -> TransportResult<u32> {
        let declared = match self.replay_sequence_seeds.lock() {
            Ok(guard) => guard.get(topic).copied(),
            Err(poisoned) => poisoned.into_inner().get(topic).copied(),
        };
        let Some(seed) = declared else {
            return Ok(0);
        };
        if publisher_provisioning == PublisherProvisioning::Multi {
            return Err(map_err(format!(
                "a replay declared the restored wire sequence {seed} for this topic, but the \
                 topic is provisioned MULTI-PUBLISHER (the graph's `multi_publisher_topics:` \
                 opt-in). The wire `sequence` is a PER-PUBLISHER commit counter, so one \
                 topic-level seed would be applied to every publisher and be wrong for at least \
                 one of them — every frame it emitted would read as a byte mismatch that looks \
                 like a node change. Fix: narrow the replay so this \
                 topic is injected rather than re-executed, or replay from step 0, where every \
                 publisher starts at 0 and no seed is needed"
            )));
        }
        Ok(seed)
    }

    /// Attach a publisher to an OWNED topic whose service
    /// was already created by [`Self::precreate_topic_services`] (the graph
    /// node loop's publisher path — every produced topic is graph-owned, so
    /// pre-create always covers it). Opens the existing service (`.open()`,
    /// never creates), so a missing pre-create surfaces LOUDLY instead of
    /// being masked by a lazy `open_or_create`, then attaches the
    /// single-writer publisher port via [`Self::finish_publisher`].
    pub(crate) fn create_publisher_on_existing_service(
        &self,
        topic: &str,
        max_slice_len: MaxSliceLen,
        history_size: usize,
        topic_config: TopicServiceConfig,
    ) -> TransportResult<CerulionPublisher> {
        let topic_owned = topic.to_string();
        let map_err = move |reason: String| TransportError::PublisherCreation {
            topic: topic_owned.clone(),
            reason,
        };
        let (data_service, event_service) =
            self.open_existing_topic_services(topic, OWNED_TOPIC_NOT_PRECREATED_HINT, &map_err)?;
        self.finish_publisher(
            topic,
            &data_service,
            event_service,
            max_slice_len,
            history_size,
            topic_config,
            &map_err,
        )
    }

    /// Create a publisher without history (backward-compatible convenience method).
    #[must_use = "publisher creation result must be checked"]
    pub fn create_publisher_simple(
        &self,
        topic: &str,
        max_slice_len: MaxSliceLen,
    ) -> TransportResult<CerulionPublisher> {
        self.create_publisher(topic, max_slice_len, 0)
    }

    /// Create the LOCAL iceoryx2 publisher a network→local INGRESS
    /// bridge re-injects received wire frames into (hand it to
    /// [`crate::transport::network::NetworkManager::register_ingress`]).
    ///
    /// UNLIKE every other publisher this one:
    ///
    /// - Re-injects REMOTE data into LOCAL SHM. Publishers carry no
    ///   network state at all (a graph process is network-free — the separate
    ///   [`crate::transport::gateway::GatewayRuntime`] taps produced topics for
    ///   egress), so an ingress publisher can only loop if THIS gateway also
    ///   ANNOUNCED the same topic (registered a bridge flag for it) — the
    ///   [`TopicBridgeManager::is_registered`] check below refuses exactly that.
    /// - Provisions the topic like a foreign External writer
    ///   ([`Self::default_topic_config`], `open_or_create`) with no native
    ///   history — the ingress publisher takes 1 of the 2 iceoryx2
    ///   create-default publisher slots. A topic ALREADY owned by an in-graph
    ///   `SingleWriter` producer (service created with `max_publishers = 1`, its
    ///   one slot taken) rejects the ingress publisher at iceoryx2 port
    ///   creation — you cannot inject remote data into a topic this graph
    ///   already produces locally.
    ///
    /// Refuses up front (loudly, naming the topic) if this gateway already
    /// ANNOUNCED the topic for egress
    /// ([`TopicBridgeManager::is_registered`]) — ingress + egress on the same
    /// topic is the same infinite loop (also refused earlier by
    /// [`crate::transport::gateway::GatewayPlan::validate`]).
    ///
    /// # Errors
    ///
    /// - [`TransportError::PublisherCreation`] if the topic is already announced
    ///   for egress (loop refusal), or if iceoryx2 service / port creation fails
    ///   (including the single-writer slot already taken by a live in-graph
    ///   producer).
    #[must_use = "publisher creation result must be checked"]
    pub fn create_ingress_publisher(
        &self,
        topic: &str,
        max_slice_len: MaxSliceLen,
    ) -> TransportResult<CerulionPublisher> {
        // Loop refusal: this gateway must not both ANNOUNCE (egress-tap) and
        // INGRESS the same topic (re-inject → tap → egress → a remote
        // re-injects → …).
        if self.bridge_mgr.is_registered(topic)? {
            return Err(TransportError::PublisherCreation {
                topic: topic.to_string(),
                reason: "this gateway already announced the topic for egress (registered a \
                         bridge flag), so a network→local ingress injector would loop \
                         (re-inject → egress-tap → re-inject). Use distinct topics for egress \
                         and network ingress."
                    .to_string(),
            });
        }

        let topic_owned = topic.to_string();
        let map_err = move |reason: String| TransportError::PublisherCreation {
            topic: topic_owned.clone(),
            reason,
        };
        // Ingress provisions like a foreign External writer (2 create-default
        // publisher slots) and requests NO native history.
        let stock_config = self.default_topic_config();
        // PREFER a raised, slice-aware receive-queue ceiling. A bridge
        // route carries a foreign stream at a rate nobody declared, and the
        // stock 16-frame default absorbs only 9.6 ms at 1.66 kHz — which on the
        // Go2 cost two Point-LIO outputs 71 % of their frames. `.max()`, so a
        // caller (or a `TransportConfig`) that already asked for more keeps it.
        // See `ingress_route_buffer_depth` for the budget arithmetic and for why
        // this cannot be rate-aware.
        let mut raised_config = stock_config;
        raised_config.subscriber_max_buffer_size = stock_config
            .subscriber_max_buffer_size
            .max(ingress_route_buffer_depth(max_slice_len.get() as usize));

        // TWO-PHASE, and the second phase is load-bearing.
        //
        // Setting a buffer ceiling ALSO arms iceoryx2's open-time at-least
        // verification, so asking for a deeper one turns a service that already
        // exists at the stock ceiling — a `cerulion topic echo` left running, a
        // consumer that opened first, a leftover from a previous run — from a
        // working degraded attach into a HARD REFUSAL
        // (`DoesNotSupportRequestedMinBufferSize`). A route that will not START
        // is strictly worse than one that loses frames, and it breaks this
        // repo's degraded-attach contract, stated on `for_topology`'s borrow
        // field: owners TOLERATE + WARN, they do not refuse. So the raise is
        // create-side generosity only: exactly the borrow-axis two-phase shape
        // (`create_borrowed_samples`), applied here on the buffer axis.
        //
        // The fallback FIRES on any error rather than sniffing the reason text:
        // if the stock attempt also fails, its error is returned and the
        // behaviour is byte-for-byte what a caller saw before the raise existed,
        // so no failure class is swallowed or reworded.
        //
        // What the fallback REPORTS is split, though, because the arm catches
        // every error class and only ONE of them licenses naming a culprit. Only
        // `DoesNotSupportRequestedMinBufferSize` proves a shallower service
        // already exists; on a transient failure (`SystemInFlux`, an environment
        // error) followed by a successful stock retry there may be no earlier
        // opener at all, and "stop the earlier opener" would send an operator
        // hunting one that does not exist. The classification is STRUCTURAL —
        // the iceoryx2 variant, carried out through `OpenServicesFailure` — not
        // a match on the reason text, which `map_err` composes.
        let raise_applied =
            raised_config.subscriber_max_buffer_size > stock_config.subscriber_max_buffer_size;
        let (topic_config, data_service, event_service) =
            match self.open_topic_services_classified(topic, &map_err, raised_config) {
                Ok((data, event)) => (raised_config, data, event),
                Err(raised) if raise_applied => {
                    let (data, event) = self.open_topic_services(topic, &map_err, stock_config)?;
                    tracing::warn!(
                        topic = %topic,
                        preferred_buffer = raised_config.subscriber_max_buffer_size,
                        attached_buffer = stock_config.subscriber_max_buffer_size,
                        reason = %raised.error,
                        "{}",
                        raised.ingress_fallback_degrade_message()
                    );
                    (stock_config, data, event)
                }
                Err(e) => return Err(e.into()),
            };
        let mut publisher = self.finish_publisher(
            topic,
            &data_service,
            event_service,
            max_slice_len,
            0, // history_size — ingress retains no late-joiner history
            topic_config,
            &map_err,
        )?;
        // A raw-INGRESS publisher wakes its FOREIGN consumers from
        // `publish_raw` itself — the DDS-bridge `RawIngressRoute` and the network
        // re-inject `IngressInjector` publish raw wire frames whose consumers are
        // cross-process (`topic hz`/echo, the gateway, netd mirrors). Arming the
        // wake HERE makes it structural for every ingress caller: a
        // `RawIngressRoute` that published with no wake would leave an
        // event-driven `topic hz` on a raw-routed topic (e.g. `/lowstate`)
        // blocked forever.
        publisher.arm_publish_raw_notify();
        // The route now EXISTS, so tell anyone watching that this
        // process's ingress plane grew. Reported AFTER success — a refused
        // route is not progress — and best-effort: a recording aid must never
        // be able to fail an ingress route (see `note_ingress_route_created`).
        self.note_ingress_route_created();
        Ok(publisher)
    }

    /// Report that ONE more runtime ingress route was created by this
    /// process, on the `/__cerulion/ingress_build` progress channel (see
    /// [`ingress_build`]).
    ///
    /// This is the PRODUCING half of the first-contact recording defect: a
    /// `ros2 attach` bridge opens its raw routes sequentially, and on a COLD DDS
    /// bus each open waits on SPDP/SEDP matching plus schema resolution — ~55-60 s
    /// on the measured Go2, in bursts separated by multi-second gaps. A recorder
    /// armed at the same instant releases its channel set after two quiet 250 ms
    /// rescans, so it would freeze a 4-topic bag while 98 routes are still on their way.
    /// Broadcasting the running count lets `cerulion_bagd` hold that set open
    /// while the plane is demonstrably still growing.
    ///
    /// It lives HERE, at the one seam that mints a runtime ingress route, rather
    /// than in the bridge: `cerulion ros2 attach` does not generate or copy
    /// `nodes/dds_bridge/` (it only probes that the directory exists), so a
    /// bridge-side signal would leave every already-vendored workspace broken,
    /// while a `cerulion_core` signal reaches every bridge — and every other
    /// progressively-built ingress plane, such as a `cerulion-netd` mirror
    /// plane — the moment it is rebuilt.
    ///
    /// **Best-effort by contract.** A poisoned lazy-init mutex or a channel that
    /// will not open is logged once and swallowed: the worst case is that a
    /// recorder falls back to exactly its pre-channel timing, which is strictly
    /// better than refusing to bridge a topic because a recording aid failed.
    fn note_ingress_route_created(&self) {
        let mut guard = match self.ingress_build.lock() {
            Ok(g) => g,
            Err(_) => {
                tracing::debug!(
                    "ingress-build channel: lazy-init mutex poisoned — this route's \
                     progress is not reported (a recorder falls back to its pre-channel timing)"
                );
                return;
            }
        };
        if guard.is_none() {
            match ingress_build::IngressBuildChannel::open(&self.node) {
                Ok(channel) => *guard = Some(channel),
                Err(e) => {
                    tracing::warn!(
                        error = %e,
                        "ingress-build channel: could not open the progress channel — a \
                         recorder started against this process cannot tell that its ingress plane \
                         is still being built, so routes opened after the recorder's settle window \
                         will be REPORTED rather than recorded"
                    );
                    return;
                }
            }
        }
        let total = guard
            .as_ref()
            .expect("ingress-build channel just ensured")
            .note_route_created();
        tracing::debug!(
            routes_created = total,
            "ingress-build channel: runtime ingress route created"
        );
    }

    /// Observable state (Principle #3): how many runtime ingress routes THIS process has
    /// created and broadcast on the progress channel. `0` before the first one
    /// (the channel is lazily opened) and `0` if the channel could not be opened
    /// — the degraded state the channel-open failure warns about.
    #[must_use]
    pub fn ingress_routes_created(&self) -> u64 {
        self.ingress_build
            .lock()
            .ok()
            .and_then(|g| g.as_ref().map(|c| c.routes_created()))
            .unwrap_or(0)
    }

    /// Observable state (Principle #3): this process's ingress-build `writer_id` (the
    /// key a reader folds progress records by), or `None` before the channel is
    /// opened. Lets a test tie an observed record to the process that wrote it.
    #[must_use]
    pub fn ingress_build_writer_id(&self) -> Option<u64> {
        self.ingress_build
            .lock()
            .ok()
            .and_then(|g| g.as_ref().map(|c| c.writer_id()))
    }

    /// Observable state (Principle #3): whether this process's ingress-build republish
    /// pump is LIVE. `false` before the first route (lazily started) and `false`
    /// if the spawn FAILED — the degraded state where a recorder that starts
    /// LATE cannot learn this process's progress until the next route.
    #[must_use]
    pub fn ingress_build_pump_active(&self) -> bool {
        self.ingress_build
            .lock()
            .ok()
            .and_then(|g| g.as_ref().map(|c| c.pump_active()))
            .unwrap_or(false)
    }

    /// Open a READER on the ingress-build progress channel (see
    /// [`ingress_build`]) — the RECORDER calls this, once, and drains it each
    /// drive pass.
    ///
    /// # Errors
    ///
    /// Returns [`TransportError::Internal`] if the service or the subscriber
    /// cannot be opened (the recorder degrades LOUDLY rather than failing).
    pub fn create_ingress_build_reader(
        &self,
    ) -> TransportResult<ingress_build::IngressBuildReader> {
        ingress_build::IngressBuildReader::open(&self.node)
    }

    /// Send arbitrary bytes on the progress channel WITHOUT recording
    /// them — the seam tests use to craft malformed frames. Opens the channel if
    /// it is not open yet, so a test need not create a route first.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn send_raw_ingress_build_for_test(&self, bytes: &[u8]) -> TransportResult<bool> {
        let mut guard = self
            .ingress_build
            .lock()
            .map_err(|_| TransportError::Internal {
                reason: "ingress-build channel lazy-init mutex poisoned".to_string(),
            })?;
        if guard.is_none() {
            *guard = Some(ingress_build::IngressBuildChannel::open(&self.node)?);
        }
        Ok(guard
            .as_ref()
            .expect("ingress-build channel just ensured")
            .send_raw_for_test(bytes))
    }

    /// The **zenoh-free** local-SHM ingress
    /// INJECTION seam — [`register_ingress_topic`](Self::register_ingress_topic)'s
    /// Step 2 (the injection publisher) + the validate/count logic,
    /// bundled into a reusable [`IngressInjector`] handle, WITHOUT Steps 1/3 (no
    /// zenoh watch, no zenoh subscriber, no session). Works on a `network: None`
    /// [`TransportManager`] — it opens NO zenoh session.
    ///
    /// This is the primitive both `cerulion_remoted` (robot-side ingress) and
    /// the future `cerulion connect` desk re-inject client use: dial an
    /// iroh-sourced frame stream, then call
    /// [`IngressInjector::reinject_raw`] per frame to validate `total_size` +
    /// `schema_hash` and publish into local SHM — after which the
    /// topic IS a local SHM topic (`topic echo` / `vizd` / a subscriber tap it
    /// with the identical code path as a truly-local topic — "remote = local").
    ///
    /// # Safety checks kept at this seam (vs the zenoh-only ones)
    ///
    /// It delegates publisher construction to
    /// [`create_ingress_publisher`](Self::create_ingress_publisher), so its two
    /// LOCAL invariants apply here (they are zenoh-independent):
    ///
    /// - **single-writer conflict** — a topic already owned by a live in-graph
    ///   `SingleWriter` producer rejects the injection publisher at iceoryx2
    ///   port creation (the one publisher slot is taken). ALWAYS meaningful.
    /// - **bridge-map loop exclusion** — refused if THIS manager has already
    ///   ANNOUNCED the topic for egress (registered a bridge flag). On a
    ///   `network: None` manager NOTHING registers an egress flag (only a
    ///   gateway does), so the check runs but is inert — the ingress+egress
    ///   loop it guards needs a gateway in the SAME process, which the desk /
    ///   an ingress-only `remoted` does not have.
    ///
    /// The zenoh-specific structural loop exclusion (the liveliness-token /
    /// re-inject → egress-tap → re-inject cycle) is Steps 1/3 territory and does
    /// NOT apply here — there is no zenoh subscriber or demand token to create.
    ///
    /// # Errors
    ///
    /// - [`TransportError::PublisherCreation`] from
    ///   [`create_ingress_publisher`](Self::create_ingress_publisher) (topic
    ///   already announced for egress, or iceoryx2 service / single-writer-slot
    ///   creation failure).
    #[must_use = "ingress injector creation result must be checked"]
    pub fn create_ingress_injector(
        &self,
        topic: &str,
        expected_schema_hash: u64,
        max_slice_len: MaxSliceLen,
    ) -> TransportResult<IngressInjector> {
        let publisher = self.create_ingress_publisher(topic, max_slice_len)?;
        Ok(IngressInjector::new(
            topic.to_string(),
            expected_schema_hash,
            publisher,
        ))
    }

    /// Register a complete network→local INGRESS bridge for `topic`
    /// — the one-call production wiring (the graph build's `network:`
    /// block calls this once per ingress topic).
    ///
    /// Composes the three ingress pieces in failure-atomic order:
    ///
    /// 1. **Ensures the liveliness watch task is running** (latched — see
    ///    [`Self::start_network_bridge_watch`]). Ordered FIRST because the
    ///    watch start is side-effect-benign (pure-egress machines run it with
    ///    zero ingress) while a watch failure after a successful registration
    ///    would leave a half-wired bridge behind an `Err`.
    /// 2. **Creates the ingress publisher** ([`Self::create_ingress_publisher`]
    ///    — egress-suppressed, External-provisioned; refuses a topic this
    ///    process already bridges OUT and a topic owned by a live in-graph
    ///    single-writer producer).
    /// 3. **Registers the zenoh subscriber + liveliness token**
    ///    ([`crate::transport::network::NetworkManager::register_ingress`] —
    ///    validates every received raw wire frame against
    ///    `expected_schema_hash` and re-injects verbatim; the token flips the
    ///    REMOTE machine's egress bridge flag on).
    ///
    /// The zenoh session opens lazily at step 1's first call (Principle #8:
    /// one session per process — the manager's single `NetworkManager` owns
    /// it). Observability: [`crate::transport::network::NetworkManager::ingress_stats`]
    /// via [`Self::network`].
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] when the manager has NO
    ///   network transport configured — network ingress needs one. Fix: add
    ///   the graph's `network:` block or set
    ///   `TransportConfig.network = Some(NetworkConfig { .. })` before init.
    /// - [`TransportError::SessionCreation`] if the zenoh session cannot open.
    /// - [`TransportError::PublisherCreation`] from step 2's refusals.
    /// - [`TransportError::Internal`] / [`TransportError::Receive`] from step
    ///   3 (double-registration; zenoh subscriber declaration failure).
    #[must_use = "ingress registration result must be checked"]
    pub fn register_ingress_topic(
        &self,
        topic: &str,
        expected_schema_hash: u64,
        max_slice_len: MaxSliceLen,
    ) -> TransportResult<()> {
        let Some(network) = self.network.as_ref() else {
            return Err(TransportError::InvalidTransportConfig {
                reason: format!(
                    "network ingress for topic '{topic}' requires a configured network \
                     transport, but this TransportManager has none — add the graph's \
                     `network:` block or set `TransportConfig.network = \
                     Some(NetworkConfig {{ .. }})` before initializing the transport"
                ),
            });
        };
        // Step 1: latched watch start (see ordering rationale in the doc).
        self.start_network_bridge_watch()?;
        // Step 2: the egress-suppressed local injection publisher.
        let publisher = self.create_ingress_publisher(topic, max_slice_len)?;
        // Step 3: zenoh subscriber + liveliness token + validate/re-inject.
        network.register_ingress(topic, expected_schema_hash, publisher)
    }

    /// Tear down a network→local INGRESS bridge for `topic`
    /// — the inverse of [`Self::register_ingress_topic`], the core-level release the
    /// `cerulion-netd` refcount-0 path drives when the last consumer of a mirror
    /// leaves.
    ///
    /// Delegates to
    /// [`crate::transport::network::NetworkManager::unregister_ingress`], which
    /// undeclares the zenoh data subscriber + the liveliness DEMAND token (so a
    /// remote egress publisher's bridge flag flips OFF and the frame stops crossing
    /// the network), releases the egress-suppressed local injection publisher
    /// (freeing the mirror topic's iceoryx2 publisher slot so a later re-register
    /// cleanly re-creates the mirror), and clears the topic from the
    /// self-ingress exclusion set (it is no longer ingressed → once again an egress
    /// candidate).
    ///
    /// # NOT a perfect inverse — the watch is intentionally kept
    ///
    /// [`Self::register_ingress_topic`] starts the shared liveliness WATCH task as
    /// its latched step 1; this does NOT stop it. The watch is ONE per manager,
    /// shared by every ingress topic AND the egress side (it flips remote bridge
    /// flags for produced topics too), so a per-topic unregister must not tear it
    /// down. It is stopped exactly once by `NetworkManager::Drop`.
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] when the manager has NO network
    ///   transport (nothing could have been ingress-registered) — mirrors
    ///   [`Self::register_ingress_topic`]'s no-network error.
    /// - [`TransportError::Internal`] — LOUD, not silent — if `topic` has NO
    ///   registered ingress bridge (never registered, or already unregistered), or
    ///   the ingress registry mutex is poisoned.
    #[must_use = "ingress unregistration result must be checked"]
    pub fn unregister_ingress_topic(&self, topic: &str) -> TransportResult<()> {
        let Some(network) = self.network.as_ref() else {
            return Err(TransportError::InvalidTransportConfig {
                reason: format!(
                    "network ingress teardown for topic '{topic}' requires a configured network \
                     transport, but this TransportManager has none — nothing could have been \
                     ingress-registered"
                ),
            });
        };
        network.unregister_ingress(topic)
    }

    /// Ensure the SELF-HEAL demand-keepalive loop re-affirms `topic` — what
    /// keeps a `cerulion-netd` mirror from going a SILENT BLACK HOLE after the robot's
    /// gateway restarts.
    ///
    /// A pure-ingress demander (netd) registers an ingress bridge via
    /// [`Self::register_ingress_topic`], which declares a ONE-SHOT liveliness demand
    /// token. If the robot's gateway restarts, the desk's peer-mode zenoh session
    /// reconnects the endpoint (default `connect/timeout_ms=-1` retries forever) but
    /// the dialer-side demand-token declaration does NOT reliably re-cross the fresh
    /// link (zenoh 1.8 interest asymmetry), so egress never re-enables — the
    /// silent black hole. This starts the demand-GET keepalive (a
    /// dialer→accepter QUERY — the wire direction that DOES cross a reconnected /
    /// strict link) over the netd's dynamic ingress set, which re-affirms egress
    /// within one pass once the transport is back. Call it once per demanded topic,
    /// right after `register_ingress_topic`. Idempotent + start-once.
    ///
    /// A gateway (which boots its OWN demand-GET loop from its fixed ingress list)
    /// never calls this — the manager's single loop slot is used by exactly one role.
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] when no network transport is
    ///   configured (mirrors [`Self::register_ingress_topic`]).
    /// - [`TransportError::SessionCreation`] if the keepalive thread cannot spawn.
    #[must_use = "ingress demand keepalive result must be checked"]
    pub fn ensure_ingress_demand_keepalive(&self, topic: &str) -> TransportResult<()> {
        let Some(network) = self.network.as_ref() else {
            return Err(TransportError::InvalidTransportConfig {
                reason: format!(
                    "the ingress demand keepalive for topic '{topic}' requires a configured \
                     network transport, but this TransportManager has none"
                ),
            });
        };
        network.ensure_ingress_demand_keepalive(topic)
    }

    /// Drop `topic` from the self-heal demand-keepalive set (its mirror was
    /// torn down at refcount 0). A no-network manager or a never-registered topic is
    /// a silent no-op — teardown must never fail on cleanup.
    pub fn remove_ingress_demand_keepalive(&self, topic: &str) {
        if let Some(network) = self.network.as_ref() {
            network.remove_ingress_demand_keepalive(topic);
        }
    }

    /// Run ONE inline demand-GET pass over the keepalive set — the PROMPT
    /// belt the netd health watch drives when a mirror goes stale, so a returning
    /// robot gateway is re-demanded immediately rather than after the next
    /// background pass. `should_abort` is polled before the pass and between GETs
    /// (the caller threads its shutdown flag so a teardown mid-pass returns after
    /// the current GET).
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] when no network transport is
    ///   configured.
    /// - [`TransportError::SessionCreation`] if the session cannot open.
    #[must_use = "ingress demand-get pass result must be checked"]
    pub fn run_ingress_demand_get_pass(
        &self,
        should_abort: &dyn Fn() -> bool,
    ) -> TransportResult<()> {
        let Some(network) = self.network.as_ref() else {
            return Err(TransportError::InvalidTransportConfig {
                reason: "the ingress demand-get pass requires a configured network transport, \
                         but this TransportManager has none"
                    .to_string(),
            });
        };
        network.run_ingress_demand_get_pass(should_abort)
    }

    /// Observable state (Principle #3): the number of topics in the self-heal
    /// demand-keepalive set — `0` on a no-network manager. Observable for tests +
    /// operators.
    pub fn ingress_demand_keepalive_topic_count(&self) -> usize {
        self.network
            .as_ref()
            .map(|n| n.ingress_demand_keepalive_topic_count())
            .unwrap_or(0)
    }

    /// Ensure the network liveliness WATCH task is running — the
    /// pure-EGRESS side's hook. A machine that only PUBLISHES to the network
    /// has no ingress registration to start the task, but its liveliness
    /// watcher must run for a REMOTE machine's ingress `TopicToken` to flip
    /// this machine's egress bridge flags on (`Put` → `enable_bridge`,
    /// `Delete` → `disable_bridge` — see `NetworkManager::start_task`). The
    /// graph build calls this once per egress topic.
    ///
    /// **Latched-idempotent**: the first successful call registers exactly ONE
    /// network-subscriber ref (starting the task and lazily opening the zenoh
    /// session); every later call — including via
    /// [`Self::register_ingress_topic`] — is a no-op `Ok(())`. Repeated calls
    /// do NOT stack the `NetworkManager` subscriber count, so the task runs
    /// for the manager's whole lifetime and is stopped exactly once by
    /// `NetworkManager::Drop`. A FAILED start un-latches, so a later retry can
    /// attempt the start again instead of latching a dead watch.
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] when no network transport
    ///   is configured (fix: the graph's `network:` block).
    /// - [`TransportError::SessionCreation`] if the zenoh session cannot open
    ///   or the watch thread cannot spawn.
    #[must_use = "bridge watch start result must be checked"]
    pub fn start_network_bridge_watch(&self) -> TransportResult<()> {
        let Some(network) = self.network.as_ref() else {
            return Err(TransportError::InvalidTransportConfig {
                reason: "the network bridge watch requires a configured network transport, \
                         but this TransportManager has none — add the graph's `network:` \
                         block or set `TransportConfig.network = \
                         Some(NetworkConfig { .. })` before initializing the transport"
                    .to_string(),
            });
        };
        // Latched-idempotent: `swap` wins the race — exactly one caller
        // proceeds to register; everyone else short-circuits Ok. (Fully
        // qualified `Ordering` — the module-level import is test-gated.)
        if self
            .network_watch_started
            .swap(true, std::sync::atomic::Ordering::SeqCst)
        {
            return Ok(());
        }
        if let Err(e) = network.add_network_subscriber(Arc::clone(&self.bridge_mgr)) {
            // Roll the failed registration back before un-latching. Safe for
            // BOTH failure arms: a session-creation failure never incremented
            // the count (saturating no-op here), and a post-increment
            // thread-spawn failure is decremented so a retry re-enters the
            // count==1 start path instead of silently never starting the task.
            network.remove_network_subscriber();
            self.network_watch_started
                .store(false, std::sync::atomic::Ordering::SeqCst);
            return Err(e);
        }
        Ok(())
    }

    /// The process's ONE runtime-egress registration channel, opened lazily on
    /// first use and shared from then on. Returns a CLONE of the shared
    /// handle; the caller holds NO lock afterwards, so the panic-capable
    /// registration work (the iceoryx2 loan + send) never runs under a guard —
    /// a panic there would otherwise POISON the slot's guard and turn every later
    /// registration in the process into the poisoned-lock error (every later
    /// publisher local-only). The slot lock is poison-TOLERANT besides
    /// (`into_inner`, the `lock_regime_latch` precedent: a torn `Option<Arc>`
    /// is still a valid one).
    ///
    /// CREATION is serialized by its own poison-tolerant mutex and
    /// double-checked against the slot, so exactly ONE control publisher is
    /// ever opened per process: N first registrations racing on an empty slot
    /// would otherwise EACH open a publisher before one won the install, and the
    /// control service caps writers at
    /// [`REG_CHANNEL_MAX_WRITERS`](reg_channel::REG_CHANNEL_MAX_WRITERS) — a
    /// concurrent burst larger than the cap would exhaust it and the losers'
    /// topics would stay local-only. Under the creation lock the ONLY
    /// panic-capable work is the channel OPEN (an iceoryx2 service open +
    /// publisher create): a panic there unwinds with the creation guard held —
    /// tolerated, a `Mutex<()>` has no state to tear, so the next creation
    /// simply retries — and the SLOT guard is never held across it. On the rmw
    /// path the caller's `catch_unwind` contains an open panic exactly like a
    /// send panic; elsewhere it propagates as any transport-construction
    /// panic does.
    fn dynamic_egress_channel(&self) -> TransportResult<Arc<reg_channel::RegistrationChannel>> {
        let existing = {
            let guard = self
                .dynamic_egress
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            guard.as_ref().map(Arc::clone)
        };
        if let Some(channel) = existing {
            return Ok(channel);
        }
        let _creating = self
            .dynamic_egress_create
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        // Double-check under the creation lock: a racer may have installed
        // the channel while this thread waited.
        let already = {
            let guard = self
                .dynamic_egress
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            guard.as_ref().map(Arc::clone)
        };
        if let Some(channel) = already {
            return Ok(channel);
        }
        let fresh = Arc::new(reg_channel::RegistrationChannel::open(&self.node)?);
        #[cfg(any(test, feature = "test-helpers"))]
        control_channel_creations::note(&self.node);
        let mut guard = self
            .dynamic_egress
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        Ok(Arc::clone(guard.get_or_insert(fresh)))
    }

    /// Test seam: how many control CHANNELS (each owning one control
    /// publisher) this manager has ever CREATED — the deterministic oracle for
    /// "exactly one open ever happens": an unserialized implementation that
    /// opened several and dropped the losers before any live sample could see
    /// them still COUNTED every open. Keyed to this manager's iceoryx2 node
    /// id, so sibling tests in a parallel binary cannot bleed into it.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn dynamic_egress_channel_creations_for_test(&self) -> u64 {
        control_channel_creations::count(&self.node)
    }

    /// Register `topic` for network egress AT RUNTIME from THIS
    /// (network-free) process — the cross-process entry point a `cerulion ros2
    /// attach` robot's dds_bridge calls for each raw route (`/utlidar/cloud`, …)
    /// as it opens the route's SHM publisher. Pushes a `(canonical topic,
    /// schema_hash)` record over the reserved `/__cerulion/gateway_topics` control
    /// service (see [`reg_channel`]); the separate
    /// GATEWAY process drains that service and feeds every record through its
    /// [`GatewayRuntime::register_runtime_topic`](crate::transport::gateway::GatewayRuntime::register_runtime_topic),
    /// so the raw route becomes BOTH discoverable (announced) AND demand-grantable.
    ///
    /// Cheap + network-free by construction: it requires NO zenoh session and NO
    /// `network:` config (a worker process is deliberately network-free — the
    /// gateway owns the robot's one session). The first call lazily opens the
    /// control publisher and starts a background thread that periodically
    /// RE-PUBLISHES the whole registration set, which closes the bring-up race
    /// (the gateway may open the control service AFTER many registrations were
    /// pushed) regardless of which process started first — see the
    /// [`reg_channel`] module docs.
    ///
    /// # Contract (what the return value means)
    ///
    /// `Ok(true)` / `Ok(false)` mean the topic is now QUEUED in this process's
    /// authoritative registration set (`true` = genuinely new, `false` = an
    /// idempotent repeat). It does NOT mean "announced right now": the record is
    /// carried to the gateway by the immediate live send AND the periodic
    /// republish belt, so a gateway that boots later still converges to the full
    /// set (see the [`reg_channel`] periodic-republication design). IDEMPOTENT at
    /// the callsite: repeat calls are fine.
    ///
    /// On `Err` NOTHING was queued — the record never entered the authoritative
    /// set and never reached the wire (no zombie). The caller should fix
    /// the topic name (for a validation error) or surface the error (for an
    /// environment / SHM failure); a bad topic silently ignored would leave the
    /// route un-announced and un-demandable forever.
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] if `topic` is empty, if its
    ///   canonical (leading-slash) form exceeds
    ///   [`MAX_REG_TOPIC_LEN`](reg_channel::MAX_REG_TOPIC_LEN) bytes
    ///   (validated at THIS entry so an over-long name never enters the
    ///   authoritative map as a permanent un-encodable zombie), or if it is in the
    ///   reserved control-plane namespace
    ///   ([`RESERVED_TOPIC_PREFIX`](crate::transport::gateway::RESERVED_TOPIC_PREFIX))
    ///   — each refused LOUDLY, naming the topic + reason. This is defense in
    ///   depth: the gateway-side [`register_runtime_topic`](crate::transport::gateway::GatewayRuntime::register_runtime_topic)
    ///   is the backstop, but a bad topic must never reach the wire.
    /// - [`TransportError::Internal`] if the control service / publisher cannot be
    ///   opened (a broken SHM environment). The lazy-init slot's lock is
    ///   poison-TOLERANT and is never held across the iceoryx2 work, so a panic
    ///   elsewhere in the process cannot turn every later registration
    ///   into this error.
    #[must_use = "dynamic egress registration result must be checked"]
    pub fn register_dynamic_egress_topic(
        &self,
        topic: &str,
        schema_hash: u64,
    ) -> TransportResult<bool> {
        if topic.is_empty() {
            return Err(TransportError::InvalidTransportConfig {
                reason: "register_dynamic_egress_topic requires a non-empty topic name".to_string(),
            });
        }
        let canonical = crate::transport::network::canonical_topic(topic).into_owned();
        // Principle #6: enforce the topic-length bound HERE, at the writer
        // entry, on the CANONICAL name (canonicalization prepends '/', so a 512-byte
        // raw name becomes a 513-byte canonical one). Without this an over-long name
        // would pass the empty/reserved guards, land in the unbounded authoritative
        // map (`insert` returns newly=true → this returns Ok(true)), then FAIL to
        // encode on every send (`encode_record`'s `TopicTooLong`) — a permanent
        // zombie the caller was told succeeded, re-failing on every republish. Reject
        // it loudly instead, so nothing un-encodable ever enters the set.
        if canonical.len() > reg_channel::MAX_REG_TOPIC_LEN {
            // Truncate the echoed name so a pathologically huge topic cannot bloat
            // the error string.
            let shown: String = canonical.chars().take(64).collect();
            return Err(TransportError::InvalidTransportConfig {
                reason: format!(
                    "register_dynamic_egress_topic REFUSED: canonical topic '{shown}…' is \
                     {} bytes, exceeding the {}-byte limit (MAX_REG_TOPIC_LEN). Use a shorter \
                     topic name",
                    canonical.len(),
                    reg_channel::MAX_REG_TOPIC_LEN,
                ),
            });
        }
        // Defense in depth (item 2): refuse a control-plane name at THIS entry
        // too, with the SAME predicate the gateway backstop uses.
        if crate::transport::gateway::is_reserved_topic(&canonical) {
            return Err(TransportError::InvalidTransportConfig {
                reason: format!(
                    "register_dynamic_egress_topic REFUSED: '{canonical}' is in the reserved \
                     control-plane namespace '{}' — that namespace carries Cerulion's own control \
                     services (e.g. THIS registration channel), so registering it as robot egress \
                     would let data plumbing collide with the plumbing that carries it. Use a \
                     topic outside '{}'",
                    crate::transport::gateway::RESERVED_TOPIC_PREFIX,
                    crate::transport::gateway::RESERVED_TOPIC_PREFIX,
                ),
            });
        }
        // NO lock across the panic-capable work, and channel CREATION
        // serialized so exactly one control publisher is ever opened — both
        // rules are part of `dynamic_egress_channel`'s contract.
        let channel = self.dynamic_egress_channel()?;
        let newly = channel.register(&canonical, schema_hash);
        // Wake a zero-demand gateway sharing THIS manager so it
        // drains the control channel and announces the new topic now, rather than
        // at its next `GATEWAY_ZERO_DEMAND_IDLE` fallback tick. This is the
        // in-process half only (netd's egress plane registers on the same manager
        // its embedded gateway drives); a registration published from ANOTHER
        // process cannot bump an in-process condvar and is served by that fallback
        // — which is exactly why the fallback exists. Gated on `newly` for the same
        // reason `register_topic` is: the republish pump re-sends the whole set
        // periodically, and a bump per republish would wake an idle gateway for
        // nothing.
        if newly {
            self.bridge_manager().demand_signal().signal();
        }
        Ok(newly)
    }

    /// Observable state (Principle #3): whether THIS process's runtime-egress registration
    /// channel has a LIVE background republish thread — the belt that closes the
    /// bring-up race by periodically re-sending the whole registration set. The
    /// writer-side symmetry to the reader-side
    /// [`GatewayRuntime::runtime_registration_active`](crate::transport::gateway::GatewayRuntime::runtime_registration_active).
    ///
    /// `false` before the first [`Self::register_dynamic_egress_topic`] (the
    /// channel + pump are lazily created on first register), AND `false` if the
    /// channel exists but its pump thread failed to spawn OR has since DIED
    /// (liveness is a flag the thread body clears by RAII on ANY
    /// exit — a panic escaping the pump's per-tick containment included —
    /// never the mere existence of the thread's handle) — in either degraded
    /// state registrations still send immediately on each register call, but a
    /// late-booting gateway will not receive pre-boot registrations until the next
    /// register call re-sends them. A `false` after registrations have begun is
    /// the operator's signal that the automagic belt is down (see the spawn-failure
    /// warn and the pump-death error in [`reg_channel`]).
    pub fn registration_pump_active(&self) -> bool {
        let guard = self
            .dynamic_egress
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        guard.as_ref().is_some_and(|c| c.registration_pump_active())
    }

    /// Open a READER on the runtime-egress control service (see
    /// [`reg_channel`]) — the GATEWAY calls this
    /// once at boot and drains it each drive pass. Opened `open_or_create`, so a
    /// gateway that boots before any worker CREATES the service (the worker opens
    /// it later); both sides share ONE service config, so `open_or_create` is
    /// always compatible.
    ///
    /// # Errors
    ///
    /// [`TransportError::Internal`] if the control service / subscriber cannot be
    /// opened.
    pub(crate) fn create_registration_reader(
        &self,
    ) -> TransportResult<reg_channel::RegistrationReader> {
        reg_channel::RegistrationReader::open(&self.node)
    }

    /// Test seam: re-publish the whole runtime-egress registration set
    /// inline (no thread wait) — the deterministic driver for the ordering-race
    /// pin. No-op `0` if no registration has been made yet.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn republish_dynamic_egress_for_test(&self) -> usize {
        let guard = self
            .dynamic_egress
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        guard.as_ref().map(|c| c.republish()).unwrap_or(0)
    }

    /// Test seam: send arbitrary RAW bytes on the control publisher
    /// WITHOUT recording them — lets a test craft a hostile / malformed / reserved
    /// frame on the wire (a well-behaved worker never does this). Lazily opens the
    /// channel. Returns whether the send succeeded.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn dynamic_egress_send_raw_for_test(&self, bytes: &[u8]) -> TransportResult<bool> {
        Ok(self.dynamic_egress_channel()?.send_raw_for_test(bytes))
    }

    /// Test seam: the number of distinct topics in THIS process's
    /// runtime-egress registration set (`0` before the first registration).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn dynamic_egress_record_count_for_test(&self) -> usize {
        let guard = self
            .dynamic_egress
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        guard.as_ref().map(|c| c.record_count()).unwrap_or(0)
    }

    /// Test seam: the `schema_hash` THIS process's runtime-egress
    /// registration set holds for `topic` (canonical-keyed — the input is
    /// canonicalized to match how `register_dynamic_egress_topic` stores it), or
    /// `None` if the topic is not registered. Pins worker-side last-write-wins.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn dynamic_egress_record_hash_for_test(&self, topic: &str) -> Option<u64> {
        let canonical = crate::transport::network::canonical_topic(topic).into_owned();
        let guard = self
            .dynamic_egress
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        guard.as_ref().and_then(|c| c.record_hash(&canonical))
    }

    /// Test seam: cumulative pump republish ticks whose body panicked
    /// and was CONTAINED on THIS process's registration channel (`0` before
    /// the first registration). The "belt survived a panic" oracle: nonzero
    /// here with [`Self::registration_pump_active`] still `true`.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn dynamic_egress_pump_tick_panics_for_test(&self) -> u64 {
        let guard = self
            .dynamic_egress
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        guard
            .as_ref()
            .map(|c| c.pump_tick_panic_count())
            .unwrap_or(0)
    }

    /// Test seam: hold the runtime-egress control SERVICE open (zero
    /// ports) for the guard's lifetime — see
    /// [`reg_channel::ControlServiceKeepalive`] for why a test that
    /// orchestrates a creation race needs the service pinned in existence.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn dynamic_egress_control_service_keepalive_for_test(
        &self,
    ) -> TransportResult<reg_channel::ControlServiceKeepalive> {
        reg_channel::control_service_keepalive(&self.node)
    }

    /// Test seam: the LIVE publisher count on the runtime-egress control
    /// service — `1` once any registration has happened in this process, and
    /// NEVER more (channel creation is serialized; see
    /// `dynamic_egress_channel`). The race pin's oracle.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn dynamic_egress_control_publisher_count_for_test(&self) -> TransportResult<usize> {
        reg_channel::control_publisher_count_for_test(&self.node)
    }

    /// Register `topic` as a LOCAL MIRROR of the remote
    /// `origin_robot` — the provenance side-call a desk process makes AFTER it
    /// re-injects a remote topic into local SHM (via
    /// [`Self::register_ingress_topic`] or [`Self::create_ingress_injector`]).
    /// It publishes `(topic, origin_robot)` over the reserved
    /// `/__cerulion/mirrors` control service (see [`mirror_registry`]) so
    /// `cerulion topic list` folds the mirror into the REMOTE section attributed
    /// to `origin_robot` — instead of surfacing it as a phantom SECOND LOCAL
    /// topic (the "one data source = one topic" decision).
    ///
    /// This is a SEPARATE side-call, NOT a parameter on the ingress entry points
    /// — the smallest sufficient API change: both the zenoh (`register_ingress_topic`)
    /// and zenoh-free (`create_ingress_injector`) re-inject paths call this
    /// uniformly, and a loopback / same-machine ingress with no meaningful remote
    /// robot simply does not call it (mirrors the `register_dynamic_egress_topic`
    /// lazy side-call precedent). Best-effort at the callsite: a re-injector that
    /// ignores the result still delivers frames — it just will not fold in
    /// `topic list`.
    ///
    /// Cheap + network-free: it requires NO zenoh session. The first call lazily
    /// opens the control publisher and starts a background thread that
    /// periodically re-publishes the whole provenance set so a `topic list` reader
    /// that opens later still converges; DROPPING this manager stops the republish
    /// and the mirror's provenance EXPIRES (see [`mirror_registry`]).
    ///
    /// Returns `Ok(true)` if the topic was genuinely new to this process's
    /// provenance set, `Ok(false)` on an idempotent repeat (a re-attribution to a
    /// new robot overwrites, last-write-wins).
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] if `topic` or `origin_robot` is
    ///   empty, if `topic` exceeds
    ///   [`MAX_MIRROR_TOPIC_LEN`](mirror_registry::MAX_MIRROR_TOPIC_LEN), or if
    ///   `origin_robot` exceeds
    ///   [`MAX_MIRROR_ROBOT_LEN`](mirror_registry::MAX_MIRROR_ROBOT_LEN) — each
    ///   refused LOUDLY at THIS entry so an oversized name never enters the
    ///   authoritative map as an un-encodable zombie.
    /// - [`TransportError::Internal`] if the control service / publisher cannot be
    ///   opened.
    #[must_use = "mirror provenance registration result must be checked"]
    pub fn register_mirror_provenance(
        &self,
        topic: &str,
        origin_robot: &str,
    ) -> TransportResult<bool> {
        if topic.is_empty() {
            return Err(TransportError::InvalidTransportConfig {
                reason: "register_mirror_provenance requires a non-empty topic name".to_string(),
            });
        }
        if origin_robot.is_empty() {
            return Err(TransportError::InvalidTransportConfig {
                reason: format!(
                    "register_mirror_provenance REFUSED for topic '{topic}': the origin robot \
                     identity is empty — a mirror must be attributed to the robot it came from"
                ),
            });
        }
        if origin_robot.len() > mirror_registry::MAX_MIRROR_ROBOT_LEN {
            return Err(TransportError::InvalidTransportConfig {
                reason: format!(
                    "register_mirror_provenance REFUSED: origin robot identity is {} bytes, \
                     exceeding the {}-byte limit",
                    origin_robot.len(),
                    mirror_registry::MAX_MIRROR_ROBOT_LEN
                ),
            });
        }
        // Store the topic EXACTLY as the ingress publisher used it for its
        // `{topic}/data` service (raw — NOT canonicalized): the folding contract
        // is `registry key == topic_list() name == the mirror's /data-stripped
        // service name`, and the transport builds the service from the raw topic.
        // Canonicalizing here would desync a non-canonical topic (`foo` → a
        // `foo/data` service that `topic list` shows as `foo`, but a `/foo`
        // registry key would never match). Real re-injectors (vizd, connectd)
        // already pass canonical `/…` topics, so raw == canonical for them.
        if topic.len() > mirror_registry::MAX_MIRROR_TOPIC_LEN {
            let shown: String = topic.chars().take(64).collect();
            return Err(TransportError::InvalidTransportConfig {
                reason: format!(
                    "register_mirror_provenance REFUSED: topic '{shown}…' is {} bytes, \
                     exceeding the {}-byte limit",
                    topic.len(),
                    mirror_registry::MAX_MIRROR_TOPIC_LEN
                ),
            });
        }
        let mut guard = self
            .mirror_registry
            .lock()
            .map_err(|_| TransportError::Internal {
                reason: "mirror registry lazy-init mutex poisoned".to_string(),
            })?;
        if guard.is_none() {
            *guard = Some(mirror_registry::MirrorRegistry::open(&self.node)?);
        }
        let registry = guard.as_ref().expect("mirror registry just ensured");
        Ok(registry.register(topic, origin_robot))
    }

    /// Remove a mirror's PROVENANCE registration — the inverse of
    /// [`Self::register_mirror_provenance`], the best-effort side-call a re-injector
    /// (netd's refcount-0 teardown) makes when it tears a mirror down. The topic
    /// drops out of this process's provenance set so the background republish stops
    /// sending it and `cerulion topic list` stops folding it into REMOTE (no wire
    /// tombstone — expiry is by ceasing republication, see [`mirror_registry`]).
    ///
    /// Best-effort by design: if this process never registered any provenance (the
    /// lazy registry was never opened) this is a no-op returning `Ok(false)`, NOT an
    /// error — a re-injector that tears down a topic it never attributed simply has
    /// nothing to remove. Returns `Ok(true)` iff the topic was present.
    ///
    /// # Errors
    ///
    /// - [`TransportError::Internal`] if the mirror-registry lazy-init mutex is
    ///   poisoned.
    #[must_use = "mirror provenance unregistration result must be checked"]
    pub fn unregister_mirror_provenance(&self, topic: &str) -> TransportResult<bool> {
        let guard = self
            .mirror_registry
            .lock()
            .map_err(|_| TransportError::Internal {
                reason: "mirror registry lazy-init mutex poisoned".to_string(),
            })?;
        match guard.as_ref() {
            Some(registry) => Ok(registry.unregister(topic)),
            // Never opened → nothing to remove (best-effort, not an error).
            None => Ok(false),
        }
    }

    /// Gather the CURRENT live mirror-provenance snapshot on
    /// THIS manager's iceoryx2 namespace (the per-test / embedded entry — the
    /// process-global CLI entry is
    /// [`mirror_registry::gather_current_provenance`]). Deduped by topic; a desk
    /// with no live mirror writer returns empty INSTANTLY (the
    /// no-publisher fast path). Best-effort — the CLI treats any `Err` as "no
    /// provenance" so `topic list` never fails on the registry.
    ///
    /// # Errors
    ///
    /// [`TransportError`] from opening the control service or building the reader
    /// subscriber (an SHM / config failure).
    #[must_use = "mirror provenance gather result must be checked"]
    pub fn gather_mirror_provenance(
        &self,
        window: std::time::Duration,
    ) -> TransportResult<Vec<mirror_registry::MirrorRecord>> {
        mirror_registry::gather_from_node(&self.node, window)
    }

    /// [`gather_mirror_provenance`](Self::gather_mirror_provenance), but
    /// carrying whether the answer SETTLES the question
    /// ([`mirror_registry::GatherCompleteness`]).
    ///
    /// The plain gather returns an empty `Vec` both when the desk provably has no
    /// mirrors and when the window expired having heard nothing from a live
    /// writer. Those are an absence and an UNKNOWN, and a consumer that writes a
    /// durable verdict — `cerulion_bagd`'s coverage manifest — must be able to
    /// tell them apart. Rendering consumers (`topic list`, vizd's sidebar) do not
    /// need to: a lost gather costs them one stale row on one refresh.
    ///
    /// # Errors
    ///
    /// [`TransportError`] from opening the control service or building the reader
    /// subscriber (an SHM / config failure).
    #[must_use = "mirror provenance gather result must be checked"]
    pub fn gather_mirror_provenance_checked(
        &self,
        window: std::time::Duration,
    ) -> TransportResult<mirror_registry::MirrorGather> {
        mirror_registry::gather_from_node_checked(&self.node, window)
    }

    /// Gather the live `graph run`s announcing themselves on THIS
    /// manager's namespace, carrying what the answer is evidence of.
    ///
    /// It reuses the manager's own node rather than minting one, for the same
    /// reason [`gather_mirror_provenance_checked`](Self::gather_mirror_provenance_checked)
    /// does: a consumer that already holds a manager is asking about the
    /// namespace that manager is on, and opening a second node would ask about a
    /// different one on any isolated root — which is exactly the shape every
    /// test and every multi-process deployment runs in.
    ///
    /// # Errors
    ///
    /// [`TransportError`] if the registry service or its reader cannot be
    /// opened. An EMPTY answer is not an error — read
    /// [`RunGather::completeness`](run_registry::RunGather::completeness) to
    /// learn whether it settles the question.
    pub fn gather_live_runs(
        &self,
        window: std::time::Duration,
    ) -> TransportResult<run_registry::RunGather> {
        run_registry::gather_from_node_checked(&self.node, window)
    }

    /// Create a publisher whose iceoryx2 slot size is derived from the
    /// schema `T: ShmMessage`.
    ///
    /// For **fixed schemas** (`T::VARIABLE_FIELD_COUNT == 0`) the slot size
    /// is statically known: `WireHeader::SIZE + T::WIRE_FIXED_SIZE`. The
    /// `max_slice_len` argument is ignored — fixed-schema topics never
    /// waste pool memory on a user-supplied upper bound.
    ///
    /// For **variable schemas** (`T::VARIABLE_FIELD_COUNT > 0`) the slot
    /// size is the user-supplied `max_slice_len` (required). `Some(n)` is
    /// passed through; `None` returns
    /// [`TransportError::MaxSliceLenRequired`] before any iceoryx2
    /// resources are allocated.
    ///
    /// # Errors
    ///
    /// - [`TransportError::MaxSliceLenRequired`] — variable schema with
    ///   `max_slice_len: None`.
    /// - [`TransportError::PublisherCreation`] — iceoryx2 service / port
    ///   creation failure (forwarded from [`Self::create_publisher`]).
    #[must_use = "publisher creation result must be checked"]
    pub fn create_publisher_typed<T: ShmMessage>(
        &self,
        topic: &str,
        max_slice_len: Option<MaxSliceLen>,
    ) -> TransportResult<CerulionPublisher> {
        self.create_publisher_typed_with_history::<T>(topic, max_slice_len, 0)
    }

    /// Create a typed publisher with history support.
    ///
    /// Same per-schema slot-sizing rules as [`Self::create_publisher_typed`];
    /// `history_size` is forwarded to [`Self::create_publisher`].
    #[must_use = "publisher creation result must be checked"]
    pub fn create_publisher_typed_with_history<T: ShmMessage>(
        &self,
        topic: &str,
        max_slice_len: Option<MaxSliceLen>,
        history_size: usize,
    ) -> TransportResult<CerulionPublisher> {
        // Force monomorphization-time evaluation of the
        // `_SHM_INVARIANTS` const_assert — pathological hand-written
        // `impl ShmMessage` with WIRE_FIXED_SIZE/VARIABLE_FIELD_COUNT
        // outside the wire-format ceiling fails to compile here.
        let _: () = <T as ShmMessage>::_SHM_INVARIANTS;

        let resolved_slot: MaxSliceLen = if T::VARIABLE_FIELD_COUNT == 0 {
            // Fixed schema: slot size is statically known
            // (WireHeader::SIZE + WIRE_FIXED_SIZE). `_SHM_INVARIANTS`
            // above verifies the sum fits in u32 (upper bound); the
            // `>= WireHeader::SIZE` lower bound is also satisfied
            // trivially because the sum starts at WireHeader::SIZE
            // (32) and is monotonically non-decreasing in
            // WIRE_FIXED_SIZE. Both `_SHM_INVARIANTS` + the trivial
            // lower bound make the `const_new` panic unreachable.
            MaxSliceLen::const_new((WireHeader::SIZE + T::WIRE_FIXED_SIZE) as u32)
        } else {
            // Variable schema: max_slice_len is required.
            match max_slice_len {
                Some(n) => n,
                None => {
                    return Err(TransportError::MaxSliceLenRequired {
                        topic: topic.to_string(),
                    });
                }
            }
        };
        self.create_publisher(topic, resolved_slot, history_size)
    }

    /// Create a subscriber for the given topic.
    ///
    /// Sends `SubscriberConnected` event on creation so publishers can
    /// deliver history to late-joiners.
    ///
    /// # Arguments
    ///
    /// - `topic` — Topic name (must match publisher's topic exactly)
    ///
    /// # Errors
    ///
    /// - `SubscriberCreation` if topic name exceeds 249 characters
    /// - `SubscriberCreation` if iceoryx2 service or port creation fails
    #[must_use = "subscriber creation result must be checked"]
    pub fn create_subscriber(&self, topic: &str) -> TransportResult<CerulionSubscriber> {
        self.create_subscriber_with_buffers(
            topic,
            self.default_topic_config(),
            self.subscriber_buffer_size,
        )
    }

    /// The two defensive subscriber-buffer guards shared
    /// by [`Self::create_subscriber_with_buffers`] (creator open) and
    /// [`Self::create_subscriber_on_existing_service`] (open-only on a
    /// pre-created service): reject a zero buffer (this is a `pub` entry
    /// point that bypasses init validation, and iceoryx2's port builder
    /// silently clamps 0 → 1 with no log at any level) and a
    /// buffer above the topic's ceiling (the runtime guarantees depth ≤
    /// ceiling by construction — ceiling = max(default, max depth) for
    /// owned/external topics; for Multi topics the flat shared ceiling is
    /// guaranteed by the build fail-fast + the trigger-drain clamp —
    /// surface a clear error instead of iceoryx2's builder failure).
    fn validate_subscriber_buffer(
        &self,
        buffer_size: usize,
        topic_config: TopicServiceConfig,
        map_err: &impl Fn(String) -> TransportError,
    ) -> TransportResult<()> {
        if buffer_size == 0 {
            return Err(map_err(
                "buffer_size must be >= 1 (a zero queue cannot hold any sample; \
                 iceoryx2 would silently clamp it to 1)"
                    .to_string(),
            ));
        }
        if buffer_size > topic_config.subscriber_max_buffer_size {
            return Err(map_err(format!(
                "requested buffer_size {} exceeds the topic's \
                 subscriber_max_buffer_size ceiling {}",
                buffer_size, topic_config.subscriber_max_buffer_size
            )));
        }
        Ok(())
    }

    /// [`Self::create_subscriber`] with an explicit
    /// per-topic service config plus THIS subscriber's own queue depth.
    /// The graph runtime passes the topology-derived [`TopicServiceConfig`]
    /// (identical across the topic's open sites) and `buffer_size` = the
    /// input's declared `#[input(depth = N)]` — the declared depth IS the
    /// real iceoryx2 queue.
    #[must_use = "subscriber creation result must be checked"]
    pub fn create_subscriber_with_buffers(
        &self,
        topic: &str,
        topic_config: TopicServiceConfig,
        buffer_size: usize,
    ) -> TransportResult<CerulionSubscriber> {
        let topic_owned = topic.to_string();
        let map_err = move |reason: String| TransportError::SubscriberCreation {
            topic: topic_owned.clone(),
            reason,
        };
        self.validate_subscriber_buffer(buffer_size, topic_config, &map_err)?;
        let (data_service, event_service) =
            self.open_topic_services(topic, &map_err, topic_config)?;

        self.finish_subscriber(
            topic,
            &data_service,
            &event_service,
            Some(buffer_size),
            &map_err,
        )
    }

    /// Subscriber over an EXISTING topic only (closes the TOCTOU race left by the
    /// CLI's existence pre-check): the data
    /// service is opened with the builder's `.open()` (never
    /// `open_or_create`), so this method STRUCTURALLY cannot create a
    /// topic. Introspection commands (`cerulion topic echo`/`hz`/`info`)
    /// route through here: with [`Self::create_subscriber`], a topic-name
    /// typo silently created an empty service that streamed nothing
    /// forever, and the CLI's list-then-subscribe existence pre-check only
    /// narrowed the race (TOCTOU) — it survives purely for its
    /// did-you-mean error text.
    ///
    /// Differences from [`Self::create_subscriber`] (deliberate,
    /// introspection-shaped):
    /// - NO service requirements are armed — not even the buffer-ceiling
    ///   minimum a default opener carries — so the tool observes whatever
    ///   exists, including a foreign service whose ceiling is below the
    ///   transport default.
    /// - The subscriber PORT's buffer size is left unset, which iceoryx2
    ///   resolves to the service's own ceiling (the deepest view the
    ///   service provides; vendored 0.9.1 port/subscriber.rs:235).
    /// - The EVENT service half stays `open_or_create`: the data service
    ///   is the topic's existence marker (what `topic list` enumerates)
    ///   and is opened FIRST, so a typo'd name dies before this runs; a
    ///   foreign data-only topic gets its event half created exactly as
    ///   any Cerulion opener would — which cannot mint a phantom topic.
    #[must_use = "subscriber creation result must be checked"]
    pub fn create_subscriber_open_only(&self, topic: &str) -> TransportResult<CerulionSubscriber> {
        let topic_owned = topic.to_string();
        let map_err = move |reason: String| TransportError::SubscriberCreation {
            topic: topic_owned.clone(),
            reason,
        };
        let (data_service, event_service) = self.open_existing_topic_services(
            topic,
            " — the topic does not exist (introspection is open-only \
             and never creates services); run `cerulion topic list` \
             to see active topics",
            &map_err,
        )?;

        // Port buffer None: iceoryx2 resolves it to the service's own
        // ceiling (vendored 0.9.1 port/subscriber.rs:235) — the deepest
        // view the service provides.
        self.finish_subscriber(topic, &data_service, &event_service, None, &map_err)
    }

    /// Whether `topic`'s DATA service is ABSENT: an open-only `.open()`
    /// on the pub/sub data service fails specifically with `DoesNotExist`.
    ///
    /// This is the discriminator `topic echo`/`hz`/`info` use after a FAILED
    /// [`Self::create_subscriber_open_only`]: a topic that is LISTED (in
    /// `topic list`) but whose data service is GONE is a STALE network
    /// re-injection MIRROR whose publisher outlived it (a prior remote
    /// demand's `register_ingress_topic` service; teardown now goes through
    /// [`Self::unregister_ingress_topic`], but a CRASHED re-injector or a non-netd
    /// path can still leave a stale mirror), so the observer may fall through to the
    /// remote demand rung. ANY OTHER
    /// outcome returns `false` so the caller SURFACES its original, actionable
    /// error instead of masking it with the misleading remote not-found:
    ///
    /// - the service OPENS ⇒ it exists; the failed subscriber creation was
    ///   slot/listener exhaustion or a foreign buffer-ceiling requirement (each
    ///   carries a crafted remedy hint), or a transient race — surface it;
    /// - it fails with a NON-`DoesNotExist` error (type-skew, corruption) —
    ///   surface it;
    /// - a malformed name — surface it (not the stale-mirror class).
    ///
    /// Opens only the service FACTORY (`.open()`, never creates), never a
    /// subscriber PORT, so it consumes NO subscriber slot and cannot itself trip
    /// the exhaustion it is meant to distinguish. Cold introspection path only.
    pub fn data_service_missing(&self, topic: &str) -> bool {
        use iceoryx2::service::builder::publish_subscribe::PublishSubscribeOpenError;
        if validate_topic_name(topic).is_err() {
            return false;
        }
        let Ok(data_service_name): Result<ServiceName, _> =
            format!("{}/data", topic).as_str().try_into()
        else {
            return false;
        };
        matches!(
            self.node
                .service_builder(&data_service_name)
                .publish_subscribe::<[u8]>()
                .open(),
            Err(PublishSubscribeOpenError::DoesNotExist)
        )
    }

    /// Create a **data-only** capture tap ([`DataOnlySubscriber`]) — a
    /// zero-copy SHM reader that opens ONLY the topic's data pub/sub service and
    /// registers NO event `Listener` or `Notifier`.
    ///
    /// This is the record/replay tap's constructor and the STRUCTURAL
    /// fix: a subscriber that never opens the event service registers no listener
    /// connection there, so the publisher's notifier send loop finds zero tap
    /// connections → zero `SentSample` sends → zero failed sends, at default
    /// sysctls, under any drain stall or publish rate (see [`DataOnlySubscriber`]
    /// for the full mechanism + safety argument). It replaces the listener-full
    /// [`Self::create_subscriber_open_only`] at the `bagd` recorder and replay
    /// capture tap sites; introspection (`topic echo`/`hz`/`info`), which genuinely
    /// consumes its listener, keeps the open-only path.
    ///
    /// Like [`Self::create_subscriber_open_only`] the data service is opened
    /// open-only (`.open()`, never `open_or_create`) — a missing topic dies
    /// LOUDLY with the `topic list` hint and no phantom service is minted. Unlike
    /// it, the event service is NOT touched at all (the strictly-stronger form of
    /// "skip the listener": the tap is invisible even to the event service's
    /// static config, removing any doubt about a phantom connection).
    #[must_use = "subscriber creation result must be checked"]
    pub fn create_data_only_subscriber(&self, topic: &str) -> TransportResult<DataOnlySubscriber> {
        self.create_data_only_subscriber_inner(topic, TapDepth::Ceiling)
    }

    /// [`Self::create_data_only_subscriber`] with an explicit
    /// receive-queue depth instead of the topic's full
    /// `subscriber_max_buffer_size` ceiling.
    ///
    /// A subscriber's queue depth is how many SHM chunks it can PIN from the
    /// producer's pool between drains. The default (ceiling-deep) tap is right
    /// for an explicit `bagd --record`, which must lose nothing; it is wrong for
    /// a sampling observer like
    /// [`TopicLivenessObserver`](liveness::TopicLivenessObserver), whose only
    /// question is "did anything arrive" and which drains on a 200 ms cadence —
    /// a ceiling-deep tap on a `--record` topic (ceiling in the thousands) could
    /// hold a large share of the pool hostage between sweeps.
    ///
    /// # "must lose nothing" is a per-MODE claim
    ///
    /// `bagd` serves two recorders
    /// out of one binary and the contract SPLITS between them:
    ///
    /// * an explicit **`--record`** recorder keeps the ceiling-deep tap and
    ///   keeps the claim — lossless-by-provisioning, the replay-grade path
    ///   sized from a measured drain stall;
    /// * the always-on **window-only** Flashback recorder trades bounded
    ///   absorbance for a bounded STANDING footprint, because it runs on every
    ///   serving graph whether or not anybody asked to record. It opens through
    ///   [`Self::create_data_only_subscriber_budgeted`], and what it gives up is
    ///   DECLARED (the budget) and REPORTED (`shm_pinned_bytes`), never silent.
    ///
    /// An ungated switch of THIS constructor would be a flag-day on the
    /// replay-grade loss boundary, which is why the mode gate is structural and
    /// carries its own test rather than riding a caller's care.
    ///
    /// `buffer_size` is CLAMPED into `1 ..= subscriber_max_buffer_size`: a
    /// request above the service ceiling is iceoryx2's error, not the caller's
    /// intent (the caller is asking for LESS and does not know the ceiling), and
    /// a request of 0 would be a tap that can never receive.
    #[must_use = "subscriber creation result must be checked"]
    pub fn create_data_only_subscriber_with_buffer(
        &self,
        topic: &str,
        buffer_size: usize,
    ) -> TransportResult<DataOnlySubscriber> {
        self.create_data_only_subscriber_inner(topic, TapDepth::Fixed(buffer_size))
    }

    /// A data-only tap whose depth is
    /// chosen by a BYTE BUDGET over the topic's real slot size, rather than by
    /// the service ceiling.
    ///
    /// This is the **window-only** Flashback recorder's constructor, and it is a
    /// SEPARATE entry point on purpose. `create_data_only_subscriber` serves an
    /// explicit `--record` recorder, whose ceiling-deep tap is the replay-grade
    /// loss boundary sized from a measured drain stall; switching THAT
    /// constructor would be an ungated flag-day on it. The mode gate is
    /// therefore a different function, not a different argument to one.
    ///
    /// The rule is [`flashback_tap_buffer_depth`]:
    /// `clamp(budget_bytes / iceoryx2_slot_bytes(slice), 2, ceiling)`.
    ///
    /// # Where the slice comes from, and what happens when it cannot be known
    ///
    /// A service's static config does NOT carry `max_slice_len` — the slice is a
    /// per-PUBLISHER property, and iceoryx2 exposes it on `PublisherDetails`
    /// (`iceoryx2-0.9.1/src/service/dynamic_config/publish_subscribe.rs:62`). So
    /// the slice is read LIVE off this tap's own already-open service handle,
    /// the same class of read its `publisher_count` already does on every
    /// drain — no second `.open()`, no new port.
    ///
    /// A multi-publisher topic takes the MAXIMUM: the tap can be handed a chunk
    /// from any of them, so the budget must be priced against the biggest slot.
    ///
    /// If NO publisher is attached the slice is unknowable, and the tap falls
    /// back to CEILING-DEEP with a `warn!` naming the topic. That direction is
    /// deliberate: the alternative — guessing shallow — silently shortens the
    /// absorbance of a topic that might be a kHz stream, and a budget that
    /// cannot be priced must not be enforced by invention. It is reported rather
    /// than silent because an unenforced budget an operator believes in is worse
    /// than a loud one that did not apply. On both shipping paths a publisher
    /// exists by then: `graph run` builds every publisher port before the
    /// recorder arms, and a DISCOVERED topic was found by enumerating live
    /// services.
    #[must_use = "subscriber creation result must be checked"]
    pub fn create_data_only_subscriber_budgeted(
        &self,
        topic: &str,
        budget_bytes: u64,
    ) -> TransportResult<DataOnlySubscriber> {
        self.create_data_only_subscriber_inner(topic, TapDepth::Budget(budget_bytes))
    }

    fn create_data_only_subscriber_inner(
        &self,
        topic: &str,
        depth: TapDepth,
    ) -> TransportResult<DataOnlySubscriber> {
        let topic_owned = topic.to_string();
        let map_err = move |reason: String| TransportError::SubscriberCreation {
            topic: topic_owned.clone(),
            reason,
        };
        validate_topic_name(topic).map_err(&map_err)?;
        let data_service_name: ServiceName = format!("{}/data", topic)
            .as_str()
            .try_into()
            .map_err(|e| map_err(format!("{}", e)))?;

        // Open ONLY the data service (`.open()` — never creates). The event
        // service is deliberately never opened: a data-only tap must not register
        // with, or even reference, the topic's event service (the
        // invisibility property).
        let data_service = self
            .node
            .service_builder(&data_service_name)
            .publish_subscribe::<[u8]>()
            .open()
            .map_err(|e| {
                use iceoryx2::service::builder::publish_subscribe::PublishSubscribeOpenError;
                let hint = if matches!(e, PublishSubscribeOpenError::DoesNotExist) {
                    " — the topic does not exist (the recording/replay tap is \
                     open-only and never creates services); run `cerulion topic \
                     list` to see active topics"
                } else {
                    publish_subscribe_open_env_hint(&e)
                };
                map_err(format!("{e}{hint}"))
            })?;

        // Subscriber port only (`TapDepth::Ceiling` → the service ceiling, the
        // deepest view the recording tap provisioned; the liveness sampling
        // observer asks for a shallow one and the window-only Flashback recorder
        // asks for a budgeted one). NO listener, NO notifier.
        let ceiling = data_service.static_config().subscriber_max_buffer_size();
        let mut subscriber_builder = data_service.subscriber_builder();
        match depth {
            TapDepth::Ceiling => {}
            TapDepth::Fixed(requested) => {
                // Clamp into the service's legal range — see the
                // `create_data_only_subscriber_with_buffer` docs.
                subscriber_builder =
                    subscriber_builder.buffer_size(requested.clamp(1, ceiling.max(1)));
            }
            TapDepth::Budget(budget_bytes) => {
                // The slice is a PUBLISHER property; read it live off the handle
                // this tap already holds. MAX across publishers — the tap can be
                // handed a chunk from any of them. The SAME read the tap's own
                // `live_slot_bytes()` makes on every drain, so the depth this
                // chooses and the price the accounting quotes cannot come from
                // two different derivations.
                let mut widest: Option<usize> = None;
                data_service.dynamic_config().list_publishers(|details| {
                    widest = Some(widest.map_or(details.max_slice_len, |w: usize| {
                        w.max(details.max_slice_len)
                    }));
                    iceoryx2::prelude::CallbackProgression::Continue
                });
                match widest {
                    Some(slice) => {
                        let slot = iceoryx2_slot_bytes(slice);
                        let requested =
                            flashback_tap_buffer_depth(budget_bytes, slice, ceiling.max(1));
                        tracing::debug!(
                            topic = %topic,
                            budget_bytes,
                            max_slice_len = slice,
                            slot_bytes = slot,
                            ceiling,
                            depth = requested,
                            "window-only tap sized by byte budget"
                        );
                        subscriber_builder = subscriber_builder.buffer_size(requested);
                    }
                    None => {
                        // Loud, because a budget nobody could price must not look
                        // like a budget that applied.
                        tracing::warn!(
                            topic = %topic,
                            budget_bytes,
                            ceiling,
                            "window-only tap: no publisher is attached, so this topic's \
                             slice size is unknowable and the byte budget cannot be priced — \
                             opening CEILING-DEEP (today's behaviour) rather than guessing a \
                             shallower queue for what may be a kHz stream"
                        );
                    }
                }
            }
        }
        let subscriber = subscriber_builder.create().map_err(|e| {
            use iceoryx2::port::subscriber::SubscriberCreateError;
            let hint = if matches!(e, SubscriberCreateError::ExceedsMaxSupportedSubscribers) {
                format!(
                    " — all {} of the topic's subscriber slots are attached \
                     (for graph-provisioned topics that is the graph's \
                     body/drain subscribers plus {} introspection headroom \
                     slots, one of which the topic-liveness observer \
                     holds permanently on a gateway); stop a `cerulion topic \
                     echo`/`hz`, `bagd`, or vizd to free a slot — or, if the \
                     holder is the liveness observer, set \
                     CERULION_TOPIC_LIVENESS=off",
                    data_service.static_config().max_subscribers(),
                    INTROSPECTION_SUBSCRIBER_HEADROOM
                )
            } else {
                String::new()
            };
            map_err(format!("{e}{hint}"))
        })?;

        // The topic's effective `subscriber_max_borrowed_samples`
        // (per-service truth) so the recorder can bound its held owned samples.
        let max_borrowed_samples = data_service
            .static_config()
            .subscriber_max_borrowed_samples();

        tracing::debug!(topic = %topic, "data-only capture tap created (no event listener/notifier)");

        // A subscriber-only process maps the publisher's oversized
        // `iox2_` SHM pool zero-copy; `VM_NOHUGEPAGE` is per-VMA, so advise this
        // reader's own pool mapping (same rationale as `finish_subscriber`).
        shm_guard::advise_shm_pools_no_hugepage(
            shm_guard::PoolAdviseSite::Subscriber,
            &self.shm_segment_prefix(),
        );

        Ok(DataOnlySubscriber::new(
            topic.to_string(),
            subscriber,
            max_borrowed_samples,
            // The tap keeps its service handle so the single-writer
            // probe is a dynamic-config read, never a fresh `.open()`. The
            // window-only sizing reads the live slice off the same handle.
            data_service,
        ))
    }

    /// Enumerate the live topics visible on THIS manager's iceoryx2
    /// node (the "what can I attach" answer for the dynamic viz daemon +
    /// Principle #3 observability).
    ///
    /// Enumerates iceoryx2 services on this node's OWN [`Config`] — the singleton
    /// manager sees the global namespace, and a per-test / per-deployment manager
    /// sees ONLY its own SHM root (unlike `cerulion_cli_engine::topic_cmd::topic_list`,
    /// which hardcodes `Config::global_config()`). Each `{topic}/data` service maps
    /// back to its topic (the `/data` suffix stripped exactly ONCE — a topic whose
    /// own name ends in `/data` is not mis-trimmed); the `{topic}/event` half and a
    /// bare `/data` service are ignored. The result is sorted + de-duplicated, so
    /// discovery is deterministic.
    ///
    /// [`Config`]: iceoryx2::config::Config
    pub fn list_topics(&self) -> TransportResult<Vec<String>> {
        use std::collections::BTreeSet;
        let mut topics: BTreeSet<String> = BTreeSet::new();
        <CerService as iceoryx2::service::Service>::list(
            self.node.config(),
            |service: iceoryx2::service::ServiceDetails<CerService>| {
                let name = service.static_details.name().as_str().to_string();
                // Strip the `/data` suffix exactly once (a topic legitimately
                // ending in `/data` must not be double-trimmed); skip a bare
                // `/data` service (empty topic) rather than emit a blank row.
                if let Some(topic) = name.strip_suffix("/data").filter(|t| !t.is_empty()) {
                    topics.insert(topic.to_string());
                }
                iceoryx2::prelude::CallbackProgression::Continue
            },
        )
        .map_err(|e| TransportError::SubscriberCreation {
            topic: "<discovery>".to_string(),
            reason: format!(
                "failed to enumerate iceoryx2 services: {e} — check the iceoryx2 \
                 config and shared-memory permissions"
            ),
        })?;
        Ok(topics.into_iter().collect())
    }

    /// Open BOTH halves of an ALREADY-EXISTING topic's
    /// services — the data service via `.open()` (never `open_or_create`),
    /// so callers STRUCTURALLY cannot create the data service. Shared by the
    /// open-only introspection subscriber
    /// ([`Self::create_subscriber_open_only`]) and the graph node loop's
    /// owned-topic opens ([`Self::create_publisher_on_existing_service`] /
    /// [`Self::create_subscriber_on_existing_service`]), which run AFTER
    /// [`Self::precreate_topic_services`] has created the service.
    ///
    /// `data_missing_hint` is appended to the `DoesNotExist` error so each
    /// caller explains why the topic was expected to exist (introspection:
    /// "run `topic list`"; the node loop: "owned services are pre-created").
    /// The event half stays `open_or_create`: the data `.open()` gates
    /// first, so a typo'd / missing topic dies before the event half runs,
    /// and a foreign data-only topic gets its event half minted (never a
    /// phantom data topic).
    #[allow(clippy::type_complexity)]
    fn open_existing_topic_services(
        &self,
        topic: &str,
        data_missing_hint: &str,
        map_err: &impl Fn(String) -> TransportError,
    ) -> TransportResult<(
        iceoryx2::service::port_factory::publish_subscribe::PortFactory<CerService, [u8], ()>,
        iceoryx2::service::port_factory::event::PortFactory<CerService>,
    )> {
        validate_topic_name(topic).map_err(map_err)?;
        let data_service_name: ServiceName = format!("{}/data", topic)
            .as_str()
            .try_into()
            .map_err(|e| map_err(format!("{}", e)))?;
        let event_service_name: ServiceName = format!("{}/event", topic)
            .as_str()
            .try_into()
            .map_err(|e| map_err(format!("{}", e)))?;

        let data_service = self
            .node
            .service_builder(&data_service_name)
            .publish_subscribe::<[u8]>()
            .open()
            .map_err(|e| {
                use iceoryx2::service::builder::publish_subscribe::PublishSubscribeOpenError;
                // Open-only parity: the missing-topic arm is
                // the realistic case; corruption/type-skew/hangs reuse the
                // shared env-hint helper so an open-only caller tripping
                // over a foreign-typed squatter or crashed-peer state gets
                // the same remedy text the graph openers carry.
                let hint = if matches!(e, PublishSubscribeOpenError::DoesNotExist) {
                    data_missing_hint
                } else {
                    publish_subscribe_open_env_hint(&e)
                };
                map_err(format!("{e}{hint}"))
            })?;

        let event_service = self
            .node
            .service_builder(&event_service_name)
            .event()
            .open_or_create()
            .map_err(|e| map_err(format!("{e}{}", event_env_hint(&e))))?;

        Ok((data_service, event_service))
    }

    /// Attach a subscriber to an OWNED topic whose service
    /// was already created by [`Self::precreate_topic_services`] (the graph
    /// node loop's owned-topic subscriber path — body inputs and
    /// data-trigger drains on a topic with an in-graph producer). Opens the
    /// existing service (`.open()`, never creates) — a missing pre-create
    /// surfaces LOUDLY — then attaches the subscriber port at `buffer_size`
    /// via [`Self::finish_subscriber`]. `topic_config` is used only for the
    /// defensive buffer-ceiling guard; the DATA service's caps were
    /// set+verified by pre-create, so the data `.open()` arms nothing. (The
    /// event half is `open_or_create` like the introspection path — for an
    /// owned topic it always opens the pre-created event service; only an
    /// exotic foreign removal of *just* the event half mid-startup would
    /// re-mint it at iceoryx2 event defaults rather than the provisioned
    /// caps — acceptably rare, and the data `.open()` still gates first.)
    pub(crate) fn create_subscriber_on_existing_service(
        &self,
        topic: &str,
        topic_config: TopicServiceConfig,
        buffer_size: usize,
    ) -> TransportResult<CerulionSubscriber> {
        let topic_owned = topic.to_string();
        let map_err = move |reason: String| TransportError::SubscriberCreation {
            topic: topic_owned.clone(),
            reason,
        };
        self.validate_subscriber_buffer(buffer_size, topic_config, &map_err)?;
        let (data_service, event_service) =
            self.open_existing_topic_services(topic, OWNED_TOPIC_NOT_PRECREATED_HINT, &map_err)?;
        self.finish_subscriber(
            topic,
            &data_service,
            &event_service,
            Some(buffer_size),
            &map_err,
        )
    }

    /// Create a STANDALONE listener-only iceoryx2 `Listener`
    /// on `topic`'s EVENT service — the WaitSet wake source for a UNIFIED
    /// data-trigger input. Eligible data-trigger inputs are unified onto
    /// the node's BODY subscriber (eliminating the dual-subscriber's second
    /// SHM read — the latency win). But a subscriber PORT bundles a data
    /// receiver AND a WaitSet event listener, so eliminating the second
    /// subscriber removed the listener the live-loop reactor attaches to wake
    /// on this input. This restores that listener WITHOUT a second data
    /// receiver: a bare event `Listener`, no subscriber port, no data queue —
    /// the win is preserved.
    ///
    /// Service acquisition MIRRORS the body subscriber's owned/External split
    /// (the runtime node loop, `create_subscriber_on_existing_service` vs
    /// `create_subscriber_with_buffers`): OWNED topics open the pre-created
    /// services open-only ([`Self::open_existing_topic_services`]); EXTERNAL
    /// topics create-or-open ([`Self::open_topic_services`]) — though for an
    /// External trigger topic the body subscriber has already create-or-opened
    /// the service, so the event half always opens here. Either way only the
    /// `event_service` is used; the `data_service` factory handle is dropped
    /// (it consumes no port). The listener slot it consumes is the one the
    /// unconditionally-counted (but unused, for a unified input) second
    /// subscriber already provisioned — see the count-coupling comment in
    /// `GraphRuntime::build`, so there is NO provisioning change.
    pub(crate) fn create_trigger_listener(
        &self,
        topic: &str,
        topic_config: TopicServiceConfig,
    ) -> TransportResult<iceoryx2::port::listener::Listener<CerService>> {
        let topic_owned = topic.to_string();
        let map_err = move |reason: String| TransportError::SubscriberCreation {
            topic: topic_owned.clone(),
            reason,
        };
        // Same owned/External split as the body subscriber: owned topics were
        // pre-created → open-only; external ones create-or-open.
        let (_data_service, event_service) =
            if topic_config.publisher_provisioning == PublisherProvisioning::External {
                self.open_topic_services(topic, &map_err, topic_config)?
            } else {
                self.open_existing_topic_services(topic, OWNED_TOPIC_NOT_PRECREATED_HINT, &map_err)?
            };
        // The same listener-exhaustion hint `finish_subscriber` carries — if
        // this fails, the listener-slot capacity analysis is wrong (see the
        // count-coupling comment in `GraphRuntime::build`).
        let listener = event_service.listener_builder().create().map_err(|e| {
            use iceoryx2::port::listener::ListenerCreateError;
            let hint = if matches!(e, ListenerCreateError::ExceedsMaxSupportedListeners) {
                event_port_exhaustion_hint(
                    "listener",
                    event_service.static_config().max_listeners(),
                )
            } else {
                String::new()
            };
            map_err(format!("{e}{hint}"))
        })?;
        tracing::debug!(topic = %topic, "standalone trigger listener created (no data receiver)");
        Ok(listener)
    }

    /// Test-only seam over [`Self::create_trigger_listener`] (which is
    /// `pub(crate)` — only the runtime node loop calls it in production). It
    /// exists solely so `topic_buffer_sizing_test` can pin
    /// `create_trigger_listener`'s OWN `ExceedsMaxSupportedListeners`
    /// exhaustion-hint arm — coverage independent of the three subscriber/
    /// publisher attach sites and the shared hint body. Gated so production
    /// builds carry zero extra public surface.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn create_trigger_listener_for_test(
        &self,
        topic: &str,
        topic_config: TopicServiceConfig,
    ) -> TransportResult<iceoryx2::port::listener::Listener<CerService>> {
        self.create_trigger_listener(topic, topic_config)
    }

    /// Mint a PUBLIC listener-only wake source on `topic` — the
    /// "a frame may have arrived" signal an observer holding a listener-less
    /// [`DataOnlySubscriber`] otherwise cannot get.
    ///
    /// The returned [`WakeSource`](crate::wake::WakeSource) is a SIBLING of the tap, never a replacement:
    /// it opens no subscriber port and no data queue, so the caller keeps reading
    /// through the data-only tap (same bytes, same order, same drain path) and
    /// the listener only says WHEN to look. That is what keeps the
    /// compile-time unattachability of `DataOnlySubscriber` both true and
    /// meaningful.
    ///
    /// Open discipline mirrors [`Self::create_data_only_subscriber`] exactly: the
    /// DATA service gates first with `.open()` (never creates), so a typo'd or
    /// absent topic dies LOUDLY with the `topic list` remedy and no phantom
    /// service is minted — including no phantom EVENT service, which an
    /// event-first open would leave behind. Only then is the event half
    /// `open_or_create`d, matching every other listener attach in this file.
    ///
    /// **Cost, stated because it is not free:** the producer's notifier gains one
    /// connection and pays one `sendto` per frame for it, and a GRAPH publisher's
    /// notify elision un-arms for this topic while the source is held.
    /// Whether that producer is one you may bill is the caller's decision — see the
    /// [`wake`](crate::wake) module docs. Dropping the [`WakeSource`](crate::wake::WakeSource) releases the
    /// listener and the publisher's own gate re-arms within one publish.
    pub fn create_wake_listener(&self, topic: &str) -> TransportResult<crate::wake::WakeSource> {
        let topic_owned = topic.to_string();
        let map_err = move |reason: String| TransportError::SubscriberCreation {
            topic: topic_owned.clone(),
            reason,
        };
        let (_data_service, event_service) = self.open_existing_topic_services(
            topic,
            " — the topic does not exist (a wake listener is open-only and never \
             creates services); run `cerulion topic list` to see active topics",
            &map_err,
        )?;
        let listener = event_service.listener_builder().create().map_err(|e| {
            use iceoryx2::port::listener::ListenerCreateError;
            let hint = if matches!(e, ListenerCreateError::ExceedsMaxSupportedListeners) {
                event_port_exhaustion_hint(
                    "listener",
                    event_service.static_config().max_listeners(),
                )
            } else {
                String::new()
            };
            map_err(format!("{e}{hint}"))
        })?;
        tracing::debug!(topic = %topic, "wake listener created (no data receiver, no notifier)");
        Ok(crate::wake::WakeSource::new(listener, topic.to_string()))
    }

    /// Mint an INTERNAL in-process event doorbell for a tier-2
    /// `#[cerulion_node(external)]` `Blocking` source — a `(Notifier, Listener)`
    /// pair on a PRIVATE event-only service (no data service, no graph topic).
    ///
    /// The external node's helper thread rings the `Notifier` on each `true`
    /// closure return; the live loop attaches the `Listener` as a WaitSet wake
    /// source (so a blocking-SDK event wakes the loop promptly) and
    /// `GraphRuntime::sweep_external_sources` observes readiness and marks the
    /// node via `Scheduler::trigger_external` (the fire itself stays in the
    /// deterministic `step()` External arm — the wake is record-only).
    ///
    /// The service name (`cer_ext_doorbell/<node_id>/event`) is namespaced by
    /// this manager's iceoryx2 config, so per-test managers (each a distinct SHM
    /// root via `init_for_test`) never collide. `open_or_create` because notifier
    /// AND listener are both minted here, in one process. Both ports keep the
    /// service alive after the factory handle drops (same pattern as
    /// [`Self::create_trigger_listener`]).
    ///
    /// Cold path — invoked once per `Blocking` external node at `run_live` entry.
    /// (mod.rs is on `check_hot_path_allocs.sh`'s cold-path exclude list, so the
    /// name/String construction here needs no `hot-path-alloc-ok` annotation.)
    #[allow(clippy::type_complexity)]
    pub(crate) fn create_external_doorbell(
        &self,
        node_id: &str,
    ) -> TransportResult<(
        iceoryx2::port::notifier::Notifier<CerService>,
        iceoryx2::port::listener::Listener<CerService>,
    )> {
        let node_id_owned = node_id.to_string();
        let map_err = move |reason: String| TransportError::SubscriberCreation {
            topic: format!("cer_ext_doorbell/{node_id_owned}"),
            reason,
        };
        let event_service_name: ServiceName = format!("cer_ext_doorbell/{node_id}/event")
            .as_str()
            .try_into()
            .map_err(|e| map_err(format!("{e}")))?;
        let event_service = self
            .node
            .service_builder(&event_service_name)
            .event()
            .open_or_create()
            .map_err(|e| map_err(format!("{e}{}", event_env_hint(&e))))?;
        let notifier = event_service.notifier_builder().create().map_err(|e| {
            use iceoryx2::port::notifier::NotifierCreateError;
            let hint = if matches!(e, NotifierCreateError::ExceedsMaxSupportedNotifiers) {
                event_port_exhaustion_hint(
                    "notifier",
                    event_service.static_config().max_notifiers(),
                )
            } else {
                String::new()
            };
            map_err(format!("{e}{hint}"))
        })?;
        let listener = event_service.listener_builder().create().map_err(|e| {
            use iceoryx2::port::listener::ListenerCreateError;
            let hint = if matches!(e, ListenerCreateError::ExceedsMaxSupportedListeners) {
                event_port_exhaustion_hint(
                    "listener",
                    event_service.static_config().max_listeners(),
                )
            } else {
                String::new()
            };
            map_err(format!("{e}{hint}"))
        })?;
        tracing::debug!(
            node_id = %node_id,
            "external Blocking doorbell created (private event service: notifier + listener)"
        );
        Ok((notifier, listener))
    }

    /// The port-creation tail shared
    /// by [`Self::create_subscriber_with_buffers`],
    /// [`Self::create_subscriber_open_only`], and
    /// [`Self::create_subscriber_on_existing_service`] — subscriber port (with an
    /// optional explicit buffer; `None` resolves to the service ceiling),
    /// listener + notifier with the exhaustion hints, the success debug
    /// log (emitted here, after every port exists, so a failed port creation
    /// can never log success first), and the
    /// [`CerulionSubscriber`] assembly. The callers differ ONLY in how
    /// they acquire the services (`open_or_create` with armed requirements
    /// vs bare `.open()`) — that difference is load-bearing and pinned by
    /// `open_only_subscriber_iox2_test.rs`; everything inside this helper
    /// is identical for all three.
    fn finish_subscriber(
        &self,
        topic: &str,
        data_service: &iceoryx2::service::port_factory::publish_subscribe::PortFactory<
            CerService,
            [u8],
            (),
        >,
        event_service: &iceoryx2::service::port_factory::event::PortFactory<CerService>,
        buffer_size: Option<usize>,
        map_err: &impl Fn(String) -> TransportError,
    ) -> TransportResult<CerulionSubscriber> {
        // Slot exhaustion at port creation is the
        // failure the introspection-headroom design routes tooling users
        // into — the
        // bare iceoryx2 variant (its descriptive text is logged at a level
        // Cerulion suppresses) plus TransportError's generic "check your
        // YAML" Display trailer point operators in exactly the wrong
        // direction. Name the cause, the provisioned count, and the remedy
        // IN the reason so the trailer is pre-empted.
        let mut subscriber_builder = data_service.subscriber_builder();
        if let Some(buffer_size) = buffer_size {
            subscriber_builder = subscriber_builder.buffer_size(buffer_size);
        }
        let subscriber = subscriber_builder.create().map_err(|e| {
            use iceoryx2::port::subscriber::SubscriberCreateError;
            let hint = if matches!(e, SubscriberCreateError::ExceedsMaxSupportedSubscribers) {
                format!(
                    " — all {} of the topic's subscriber slots are attached \
                     (for graph-provisioned topics that is the graph's \
                     body/drain subscribers plus {} introspection headroom \
                     slots, one of which the topic-liveness observer \
                     holds permanently on a gateway); stop a `cerulion topic \
                     echo`/`hz`, `bagd`, or vizd to free a slot — or, if the \
                     holder is the liveness observer, set \
                     CERULION_TOPIC_LIVENESS=off",
                    data_service.static_config().max_subscribers(),
                    INTROSPECTION_SUBSCRIBER_HEADROOM
                )
            } else {
                String::new()
            };
            map_err(format!("{e}{hint}"))
        })?;

        let listener = event_service.listener_builder().create().map_err(|e| {
            use iceoryx2::port::listener::ListenerCreateError;
            let hint = if matches!(e, ListenerCreateError::ExceedsMaxSupportedListeners) {
                event_port_exhaustion_hint(
                    "listener",
                    event_service.static_config().max_listeners(),
                )
            } else {
                String::new()
            };
            map_err(format!("{e}{hint}"))
        })?;

        // Subscriber also notifies publisher events (SubscriberConnected, etc.)
        let notifier = event_service.notifier_builder().create().map_err(|e| {
            use iceoryx2::port::notifier::NotifierCreateError;
            let hint = if matches!(e, NotifierCreateError::ExceedsMaxSupportedNotifiers) {
                event_port_exhaustion_hint(
                    "notifier",
                    event_service.static_config().max_notifiers(),
                )
            } else {
                String::new()
            };
            map_err(format!("{e}{hint}"))
        })?;

        // The topic's effective iceoryx2 `max_publishers`
        // (per-service truth — stays correct when later chunks set explicit
        // per-topic values) bounds the subscriber's per-publisher eviction
        // baselines.
        let max_publishers = data_service.static_config().max_publishers();
        // The effective borrow budget, cached on the subscriber
        // for the rmw's adopt-take refusal diagnostics (per-service truth —
        // an open against a pre-existing service reads ITS value, not any
        // requested create floor).
        let max_borrowed_samples = data_service
            .static_config()
            .subscriber_max_borrowed_samples();

        // Success log after every port exists (logging before this fallible
        // tail would make a failed port creation emit
        // "subscriber created" then the error). `buffer_size = None` is
        // the open-only path (port resolved to the service ceiling).
        tracing::debug!(topic = %topic, buffer_size = ?buffer_size, "subscriber created");

        // A subscriber maps the publisher's oversized `iox2_`
        // SHM pool to read it zero-copy, and `VM_NOHUGEPAGE` is PER-VMA — so a
        // subscriber-only process (e.g. `cerulion topic echo`/`hz`, a logger
        // node) that never creates a publisher must advise its OWN pool mapping,
        // else khugepaged could collapse that VMA into 2 MiB pages (the exact
        // RSS inflation the mitigation targets). iceoryx2 maps an ALREADY-LIVE
        // publisher's data segment DURING subscriber creation
        // (`force_update_connections`), so a reader attached to a running topic
        // (the common `topic echo` case) IS covered here. RESIDUAL (fast-follow):
        // a subscriber created BEFORE its publisher maps
        // the pool lazily on the first `receive` (`update_connections`) — after
        // this once-per-creation hook — so that late-mapped VMA is not advised.
        // The create-publisher site covers the writer (fault) half, which is the
        // primary defense (the writer faults the pool pages; advising its VMA
        // before the first write keeps the shmem inode base-page-backed).
        // Subscriber site: producer pools map LAZILY, so an empty scan here is a
        // legitimate pre-publisher state — it must NOT fire the anomaly warn (nor
        // burn the one-shot token), hence `PoolAdviseSite::Subscriber`. Scan
        // matches the ACTIVE config's segment prefix.
        shm_guard::advise_shm_pools_no_hugepage(
            shm_guard::PoolAdviseSite::Subscriber,
            &self.shm_segment_prefix(),
        );

        Ok(CerulionSubscriber::new(
            topic.to_string(),
            subscriber,
            listener,
            notifier,
            max_publishers,
            max_borrowed_samples,
        ))
    }

    /// A lightweight handle for watching an EXTERNAL topic's live
    /// publisher count — the graph
    /// runtime's "external topic silence" watch reads it once at the grace
    /// deadline to detect topics no publisher ever attached to (the
    /// silent-typo follow-through to the in-prefix load warn). Open-only
    /// (cannot create; the graph's own consumers already created the
    /// service at build) and port-free (a factory handle consumes no
    /// subscriber/publisher slots).
    pub fn external_publisher_probe(&self, topic: &str) -> TransportResult<ExternalPublisherProbe> {
        let topic_owned = topic.to_string();
        let map_err = move |reason: String| TransportError::SubscriberCreation {
            topic: topic_owned.clone(),
            reason,
        };
        validate_topic_name(topic).map_err(&map_err)?;
        let data_service_name: ServiceName = format!("{}/data", topic)
            .as_str()
            .try_into()
            .map_err(|e| map_err(format!("{}", e)))?;
        let data_service = self
            .node
            .service_builder(&data_service_name)
            .publish_subscribe::<[u8]>()
            .open()
            .map_err(|e| map_err(format!("{e}{}", publish_subscribe_open_env_hint(&e))))?;
        Ok(ExternalPublisherProbe { data_service })
    }

    /// Mint a [`LivelinessCleaner`] for the graph
    /// runtime's liveliness sweep. It captures a CLONE of this node's iceoryx2
    /// `Config` (iceoryx2 `Config` is `Clone`) so the runtime can reclaim dead
    /// nodes once per sweep WITHOUT holding the transport — and without the
    /// iceoryx2 `Config`/`Node` types leaking into runtime.rs (the opaque
    /// handle keeps them confined here). See [`LivelinessCleaner`].
    pub(crate) fn liveliness_cleaner(&self) -> LivelinessCleaner {
        LivelinessCleaner {
            config: self.node.config().clone(),
            #[cfg(any(test, feature = "test-helpers"))]
            call_count: Arc::new(AtomicU64::new(0)),
        }
    }

    /// A CLONE of this manager's iceoryx2 `Config` — its root_path and prefix,
    /// i.e. WHICH SHM namespace it is on.
    ///
    /// **This is not test-gated**, though the test use below might suggest
    /// a `#[cfg(any(test, feature = "test-helpers"))]` gate: the run registry's
    /// [`RunWatcher`](run_registry::RunWatcher) needs it in production, because
    /// a consumer must attach its watcher to the SAME namespace it taps on — a
    /// watcher on a different SHM root hears nothing and would call every run
    /// vanished. Widening it exposes no new type: `iceoryx2::config::Config`
    /// already crosses this crate's public API on
    /// [`RunHandle::publish_on_config`](run_registry::RunHandle::publish_on_config)
    /// and [`gather_runs_on_config`](run_registry::gather_runs_on_config).
    ///
    /// The original (still live) use — the true-CRASH
    /// liveliness e2e (`liveliness_crash_iox2_test.rs`) serializes this into a
    /// file and re-deserializes it in a FORKED CHILD so the child's
    /// `TransportManager` joins the parent's EXACT SHM region, the only way the
    /// parent's `try_cleanup_dead_nodes` can see the dead child's registered
    /// node and reclaim its publisher port. A child that called
    /// `iceoryx_test_config()` itself would get a DIFFERENT prefix (baked into
    /// both the service paths and the node-monitoring registry path) and be
    /// invisible to the parent. iceoryx2 `Config` is `Serialize` /
    /// `Deserialize`, so the test round-trips it via `serde_json`.
    pub fn iox_config(&self) -> iceoryx2::config::Config {
        self.node.config().clone()
    }

    /// The ACTIVE iceoryx2 segment-name prefix (`global.prefix`) this manager's
    /// node was initialized with — threaded into the THP mitigation's
    /// `/proc/self/maps` scan so custom-prefix namespaces (multi-process
    /// deployments) stay protected; a hardcoded `iox2_` default would make
    /// the scan miss every custom-prefix pool. Cold path (create hooks only).
    fn shm_segment_prefix(&self) -> String {
        self.node.config().global.prefix.to_string()
    }

    /// The SHM-service-discovery [`IceoryxShmIdentity`] of this manager's iceoryx2
    /// node — every discovery-keying field (root/prefix, the service + node
    /// directories, and the per-file service suffixes). Two managers sharing this
    /// identity discover the SAME services; a manager on a different identity
    /// cannot see the other's SHM topics at all.
    ///
    /// `cerulion-netd`'s egress plane compares a FORWARDED run
    /// config's identity ([`ix_config_shm_identity_from_json`]) against this
    /// (netd's shared-session manager's) identity before tapping a producing
    /// graph's topics. If they differ — the rare case where the run resolved a
    /// DIFFERENT `global_config()` than netd (e.g. a divergent `IOX2_CONFIG_FILE`
    /// between netd's spawn and the run's, or a custom service/node directory) —
    /// netd REFUSES the egress registration so the run falls back to a per-run
    /// gateway child on its OWN namespace, rather than netd silently tapping the
    /// WRONG namespace and egressing nothing. Production accessor (cold path — one
    /// call per egress register).
    pub fn iox_shm_identity(&self) -> IceoryxShmIdentity {
        IceoryxShmIdentity::from_config(self.node.config())
    }

    /// Number of subscribers currently attached to `topic` — across ALL
    /// processes on this host (iceoryx2's service registry is global).
    /// Returns 0 when the topic's service does not exist yet.
    ///
    /// Backs `rmw_service_server_is_available` (a service
    /// server is a subscriber on the request topic).
    pub fn topic_subscriber_count(&self, topic: &str) -> usize {
        self.topic_port_count(topic, false)
    }

    /// Live PUBLISHER count on a topic's data service, queried from
    /// iceoryx2's dynamic config — covers ALL processes, unlike any
    /// process-local registry (`rcl_action_server_is_
    /// available` gates on matched publisher counts for the action's
    /// feedback/status topics).
    pub fn topic_publisher_count(&self, topic: &str) -> usize {
        self.topic_port_count(topic, true)
    }

    /// Shared probe behind the two public counts:
    /// one place owns the open-or-zero semantics, so a
    /// future probe change (e.g. distinguishing capacity-exhaustion
    /// from absent-service) cannot drift between the two readiness
    /// surfaces (rclcpp wait-for-matched vs rcl_action availability).
    fn topic_port_count(&self, topic: &str, publishers: bool) -> usize {
        let Ok(name): Result<ServiceName, _> = format!("{}/data", topic).as_str().try_into() else {
            return 0;
        };
        match self
            .node
            .service_builder(&name)
            .publish_subscribe::<[u8]>()
            .open()
        {
            Ok(service) => {
                let cfg = service.dynamic_config();
                if publishers {
                    cfg.number_of_publishers()
                } else {
                    cfg.number_of_subscribers()
                }
            }
            Err(e) => {
                // A capacity-exhaustion error here reads as "absent"
                // to readiness checks — make it diagnosable.
                tracing::debug!(topic = %topic, error = %e, "port count probe failed");
                0
            }
        }
    }

    /// The topic's LIVE producer count for the DESK LIVENESS
    /// affordance, DISTINGUISHING a genuinely dead route from a probe FAILURE — unlike
    /// [`Self::topic_publisher_count`], which collapses BOTH to `0`.
    ///
    /// - `Some(n)` — the probe succeeded: `n` live publishers. `Some(0)` is a
    ///   genuinely DEAD route (a live data service with no publisher, OR a
    ///   never-created `{topic}/data` service — the reported `/uslam/cloud_map` case),
    ///   which the desk dims + labels "no data yet".
    /// - `None` — the probe itself FAILED (a transient iceoryx2 error: fd pressure,
    ///   capacity exhaustion). The desk renders this as UNKNOWN (a plain row), NEVER as
    ///   dead: dimming a healthy topic on a transient error would be the operator-visible
    ///   inverse bug. A failure emits a debounced `warn!` (at most once per
    ///   `PRODUCER_PROBE_WARN_DEBOUNCE`) carrying the topic + error, so sustained
    ///   pressure never floods the log.
    pub fn topic_publisher_count_checked(&self, topic: &str) -> Option<u32> {
        use iceoryx2::service::builder::publish_subscribe::PublishSubscribeOpenError;
        let Ok(name): Result<ServiceName, _> = format!("{}/data", topic).as_str().try_into() else {
            // An unprobeable (invalid) name is UNKNOWN, never a dead route.
            return classify_producer_probe(ProducerProbe::Failed);
        };
        let probe = match self
            .node
            .service_builder(&name)
            .publish_subscribe::<[u8]>()
            .open()
        {
            Ok(service) => {
                ProducerProbe::Count(service.dynamic_config().number_of_publishers() as u32)
            }
            // A never-created data service = no producer ever opened it = a dead route.
            Err(PublishSubscribeOpenError::DoesNotExist) => ProducerProbe::NoService,
            // Any OTHER open error is a transient probe failure — UNKNOWN, not dead.
            Err(e) => {
                debounced_producer_probe_warn(topic, &e);
                ProducerProbe::Failed
            }
        };
        classify_producer_probe(probe)
    }

    /// Owned handle to the transport clock (the service layer
    /// stamps frames with it so replay runs under `VirtualClock` produce
    /// bit-identical timestamps).
    pub fn clock_arc(&self) -> Arc<dyn Clock> {
        Arc::clone(&self.clock)
    }

    /// THIS manager as an owned `Arc`, minted from the
    /// construction-time self-reference — so any `&TransportManager` (the
    /// graph build chain's parameter shape) can produce the owned handle
    /// `NodeContext` carries across the cdylib FFI (context-carried
    /// transport). Never consults a global: a cdylib links its OWN copy of
    /// cerulion_core, so cross-linkage statics (`get()` / `get_or_init()`)
    /// either fail or mint a SECOND manager on the wrong SHM namespace.
    ///
    /// The `expect` is a construction invariant, not a runtime condition:
    /// every constructor mints the manager via `Arc::new_cyclic` (the struct
    /// cannot be built bare — fields are private), and `&self` proves the
    /// strong count is ≥ 1, so the upgrade always succeeds.
    pub fn arc_self(&self) -> Arc<TransportManager> {
        self.self_weak
            .upgrade()
            .expect("TransportManager is alive (&self) and always Arc::new_cyclic-constructed")
    }

    /// Access the clock.
    pub fn clock(&self) -> &dyn Clock {
        self.clock.as_ref()
    }

    /// The per-subscriber receive buffer depth (number of samples).
    pub fn subscriber_buffer_size(&self) -> usize {
        self.subscriber_buffer_size
    }

    /// The per-topic service config NON-graph openers use
    /// (raw transport users, tests). Topology-derived configs are always ≥
    /// these values field-by-field, so a default opener can always open a
    /// graph-created service. The REVERSE is one-directional and
    /// creation-order-dependent: iceoryx2 open requirements are minimums
    /// (`existing < required` fails), so a graph opener whose ceiling
    /// exceeds the default CANNOT open a default-CREATED service.
    ///
    /// Two changes close the realistic instances of this ordering
    /// edge: CLI introspection (`cerulion topic echo`/`hz`/`info`)
    /// is open-only ([`Self::create_subscriber_open_only`]) and can no
    /// longer create a service at the default ceiling; and the
    /// graph pre-creates every OWNED topic's service with its final config
    /// at build start ([`Self::precreate_topic_services`]), before any port
    /// wires, so it claims the service ahead of a foreign/raw default opener
    /// that would otherwise win the create race. A foreign default opener
    /// that still manages to create first (a non-Cerulion process, or a raw
    /// `create_publisher`/`create_subscriber` user) makes the graph build
    /// fail LOUDLY at the pre-create pass with the ordering-edge remedy.
    pub fn default_topic_config(&self) -> TopicServiceConfig {
        TopicServiceConfig {
            subscriber_max_buffer_size: self.subscriber_buffer_size,
            // A default/non-graph opener has no macro snapshot
            // consumer — keep the iceoryx2 default borrow count (2).
            subscriber_max_borrowed_samples: None,
            // A default opener holds no take loans — the rmw create
            // paths raise this on their own mutated copy for loanable types.
            create_borrow_floor: None,
            max_subscribers: None,
            max_publishers: None,
            // A default opener owns nothing — External is the declared
            // intent, so the single-writer pre-check never fires for
            // raw/CLI publishers even if a caller hand-mutates
            // `max_publishers` afterwards.
            publisher_provisioning: PublisherProvisioning::External,
            // A default/non-graph opener requests NO
            // native history — the setter stays uncalled (no open-time
            // requirement), and a raw `create_publisher` user who wants
            // history passes it via the `history_size` arg, which threads
            // into this config at the create site.
            history_size: 0,
            // A default/non-graph opener has no standalone
            // listener-only sources to provision.
            extra_event_listeners: 0,
            // A default/non-graph opener keeps the iceoryx2 loan
            // default (2) — rmw borrow-window callers raise it explicitly.
            publisher_max_loaned_samples: None,
        }
    }

    /// Eagerly create an OWNED topic's data + event
    /// iceoryx2 services with their final topology-derived config, BEFORE
    /// any node port attaches. The graph build calls this for every owned
    /// topic up front so a later default/foreign opener opens the
    /// already-correct service instead of racing to create it at the default
    /// ceiling (which would fail this graph's higher-requirement opens at a
    /// different, order-dependent open site — the at-a-distance diagnosis
    /// this provisioning work exists to kill).
    ///
    /// Reuses the CREATOR path `open_topic_services`: full cap
    /// arming + the ordering-edge remedy messages, so a pre-existing foreign
    /// service that is INCOMPATIBLE (smaller ceiling / fewer slots) fails
    /// here, loudly and at one well-defined point, before any port wires.
    ///
    /// The returned [`PrecreatedService`] MUST be held until the node loop's
    /// ports attach — iceoryx2 tears the service down when this process's
    /// `Node` drops its last handle to it (see [`PrecreatedService`]).
    pub fn precreate_topic_services(
        &self,
        topic: &str,
        topic_config: TopicServiceConfig,
    ) -> TransportResult<PrecreatedService> {
        let topic_owned = topic.to_string();
        let map_err = move |reason: String| TransportError::PublisherCreation {
            topic: topic_owned.clone(),
            reason,
        };
        let (data, event) = self.open_topic_services(topic, &map_err, topic_config)?;
        Ok(PrecreatedService {
            _data: data,
            _event: event,
        })
    }

    /// Returns `true` the first time
    /// `(topic, kind)` is reported degraded by this manager —
    /// `open_topic_services` runs for the publisher AND every subscriber
    /// of a graph topic (1 + K + T opens), and the degradation is a
    /// property of the topic's pre-existing SERVICE, not of each open.
    /// Poison-recovering: a panic while holding the lock must not turn
    /// every later degradation warn into a panic.
    fn mark_degraded_once(&self, topic: &str, kind: &'static str) -> bool {
        self.degraded_warned
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .insert((topic.to_string(), kind))
    }

    /// Access the network manager (if network transport is enabled).
    pub fn network(&self) -> Option<&NetworkManager> {
        self.network.as_deref()
    }

    /// Access the topic bridge manager.
    pub fn bridge_manager(&self) -> &TopicBridgeManager {
        &self.bridge_mgr
    }

    /// An OWNED clone of the bridge-manager `Arc`. [`Self::bridge_manager`]
    /// hands out a borrow for in-place calls; the demand reconciler thread
    /// ([`crate::transport::network::NetworkManager::start_demand_reconciler`])
    /// needs the shared owner to flip egress flags across the thread boundary.
    pub fn bridge_manager_arc(&self) -> Arc<TopicBridgeManager> {
        Arc::clone(&self.bridge_mgr)
    }

    /// ANNOUNCE `topic` on the network — declare + retain a
    /// discovery liveliness token (`cerulion_ann/{robot}{canonical topic}`; the
    /// robot chunk is `NetworkConfig.robot_identity`) so a remote `topic list`
    /// can list this machine's produced topic AND attribute it to this robot.
    /// IDEMPOTENT per topic.
    ///
    /// The GATEWAY boot ([`crate::transport::gateway::GatewayRuntime::new`])
    /// calls this once per EGRESS-visible topic in its plan (the declared
    /// `network: egress:` list under Strict, every produced topic under the
    /// permissive-default posture). An announce is pure presence — it opens the
    /// zenoh session lazily but never flips any egress flag (distinct key-space,
    /// see [`crate::transport::discovery`]).
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] when the manager has NO
    ///   network transport configured (fix: the graph's `network:` block
    ///   or `TransportConfig.network = Some(NetworkConfig { .. })`).
    /// - Otherwise forwards
    ///   [`crate::transport::network::NetworkManager::announce_egress_topic`].
    #[must_use = "egress announce result must be checked"]
    pub fn announce_egress_topic(&self, topic: &str) -> TransportResult<()> {
        let Some(network) = self.network.as_ref() else {
            return Err(TransportError::InvalidTransportConfig {
                reason: format!(
                    "network egress announce for topic '{topic}' requires a configured network \
                     transport, but this TransportManager has none — add the graph's `network:` \
                     block or set `TransportConfig.network = Some(NetworkConfig {{ .. }})` \
                     before initializing the transport"
                ),
            });
        };
        network.announce_egress_topic(topic)
    }

    /// Declare + retain the BARE identity announce token
    /// (`cerulion_ann/{robot}` — robot presence, no topic suffix) so this
    /// gateway surfaces a `topic list` ROBOTS row even with ZERO egress topics.
    /// Called once at gateway network boot
    /// ([`crate::transport::gateway::GatewayRuntime`]). IDEMPOTENT.
    ///
    /// # Errors
    ///
    /// - [`TransportError::InvalidTransportConfig`] when the manager has NO
    ///   network transport configured (fix: the graph's `network:` block
    ///   or `TransportConfig.network = Some(NetworkConfig { .. })`),
    ///   or when `NetworkConfig.robot_identity` is unset.
    /// - Otherwise forwards
    ///   [`crate::transport::network::NetworkManager::announce_gateway_identity`].
    #[must_use = "identity announce result must be checked"]
    pub fn announce_gateway_identity(&self) -> TransportResult<()> {
        let Some(network) = self.network.as_ref() else {
            return Err(TransportError::InvalidTransportConfig {
                reason: "gateway identity announce requires a configured network transport, \
                         but this TransportManager has none — add the graph's `network:` \
                         block or set `TransportConfig.network = \
                         Some(NetworkConfig { .. })` before initializing the transport"
                    .to_string(),
            });
        };
        network.announce_gateway_identity()
    }

    /// Open (or create) the data and event iceoryx2 services for a topic.
    ///
    /// Both `create_publisher*` and `create_subscriber*` share this setup.
    /// The `map_err` function maps error strings to the appropriate
    /// `TransportError` variant. `topic_config` carries the per-topic
    /// service settings — iceoryx2 treats them as
    /// MINIMUM requirements on open (`existing < required` fails; an
    /// opener requiring less attaches), so the graph runtime passes the
    /// identical topology-derived value at every open site of one topic,
    /// and default openers pass [`Self::default_topic_config`], which
    /// topology-derived configs dominate field-by-field.
    #[allow(clippy::type_complexity)]
    fn open_topic_services(
        &self,
        topic: &str,
        map_err: impl Fn(String) -> TransportError,
        topic_config: TopicServiceConfig,
    ) -> TransportResult<(
        iceoryx2::service::port_factory::publish_subscribe::PortFactory<CerService, [u8], ()>,
        iceoryx2::service::port_factory::event::PortFactory<CerService>,
    )> {
        self.open_topic_services_classified(topic, map_err, topic_config)
            .map_err(TransportError::from)
    }

    /// [`Self::open_topic_services`], keeping the ONE classification
    /// its `map_err` flattening destroys — see [`OpenServicesFailure`].
    ///
    /// Only [`Self::create_ingress_publisher`] needs it (its fallback WARN names
    /// a cause and must not name one it did not observe); everything else goes
    /// through the flattening wrapper above and is byte-unchanged.
    #[allow(clippy::type_complexity)]
    fn open_topic_services_classified(
        &self,
        topic: &str,
        map_err: impl Fn(String) -> TransportError,
        topic_config: TopicServiceConfig,
    ) -> Result<
        (
            iceoryx2::service::port_factory::publish_subscribe::PortFactory<CerService, [u8], ()>,
            iceoryx2::service::port_factory::event::PortFactory<CerService>,
        ),
        OpenServicesFailure,
    > {
        validate_topic_name(topic).map_err(&map_err)?;
        // The one remaining zero ingress was a hand-mutated
        // 0-ceiling config reaching service creation via the PUBLISHER
        // path (iceoryx2 clamps it 0 → 1 with a warn Cerulion suppresses).
        // Guard at the choke point both create paths share.
        if topic_config.subscriber_max_buffer_size == 0 {
            return Err(OpenServicesFailure::from(map_err(
                "topic_config.subscriber_max_buffer_size must be >= 1 (iceoryx2 \
                 would silently clamp a zero ceiling to 1)"
                    .to_string(),
            )));
        }
        // Same suppressed-clamp class — iceoryx2 clamps a zero
        // max_subscribers to 1 at create under a warn Cerulion suppresses.
        if topic_config.max_subscribers == Some(0) {
            return Err(OpenServicesFailure::from(map_err(
                "topic_config.max_subscribers must be >= 1 when set (iceoryx2 \
                 would silently clamp zero to 1)"
                    .to_string(),
            )));
        }
        // Upper sanity bound: iceoryx2 sizes registries and data
        // segments from this value at create — a garbage value
        // (hand-mutated config) becomes a giant allocation failing as a
        // bare create error instead of a clear rejection.
        if topic_config
            .max_subscribers
            .is_some_and(|n| n > MAX_REASONABLE_PORTS)
        {
            return Err(OpenServicesFailure::from(map_err(format!(
                "topic_config.max_subscribers must be <= {MAX_REASONABLE_PORTS} \
                 (iceoryx2 provisions registries and data segments from this value)"
            ))));
        }
        // The max_publishers twins of the same guard class.
        if topic_config.max_publishers == Some(0) {
            return Err(OpenServicesFailure::from(map_err(
                "topic_config.max_publishers must be >= 1 when set (iceoryx2 \
                 would silently clamp zero to 1)"
                    .to_string(),
            )));
        }
        if topic_config
            .max_publishers
            .is_some_and(|n| n > MAX_REASONABLE_PORTS)
        {
            return Err(OpenServicesFailure::from(map_err(format!(
                "topic_config.max_publishers must be <= {MAX_REASONABLE_PORTS} \
                 (iceoryx2 provisions publisher registries from this value)"
            ))));
        }

        let data_service_name: ServiceName = format!("{}/data", topic)
            .as_str()
            .try_into()
            .map_err(|e| map_err(format!("{}", e)))?;

        let event_service_name: ServiceName = format!("{}/event", topic)
            .as_str()
            .try_into()
            .map_err(|e| map_err(format!("{}", e)))?;

        // The open error's Display is a bare variant name (iceoryx2 routes
        // the descriptive text through its own debug-level log, which
        // Cerulion pins to Error by default) — append the actionable
        // context here (the
        // DoesNotSupportRequestedMinBufferSize path is reachable when a
        // default opener created the service before the graph).
        // Builder factory, parameterized on the borrowed-samples value: the
        // two-phase acquisition below arms DIFFERENT borrow values on its open and
        // create legs, and iceoryx2 builders are consumed by `.open()` /
        // `.create()`, so each attempt constructs a fresh one. Every OTHER
        // knob is identical across legs.
        let make_data_builder = |borrowed_samples: Option<usize>| {
            let mut data_builder = self
                .node
                .service_builder(&data_service_name)
                .publish_subscribe::<[u8]>()
                .subscriber_max_buffer_size(topic_config.subscriber_max_buffer_size);
            if let Some(borrowed) = borrowed_samples {
                // Raise the per-connection borrowed-sample ceiling above
                // the iceoryx2 default (2) on snapshot-source topics — a macro
                // snapshot consumer HOLDS one sample across steps, pushing the
                // drain peak to 3. Setting the value also arms iceoryx2's open-time
                // at-least verification (existing < required fails) — skipped for
                // `None` so non-snapshot openers carry no borrow requirement.
                data_builder = data_builder.subscriber_max_borrowed_samples(borrowed);
            }
            if let Some(max_subscribers) = topic_config.max_subscribers {
                // Setting the value also arms iceoryx2's open-time
                // verification (existing < required fails) — deliberately
                // skipped for `None` so default openers never carry a
                // subscriber-count requirement.
                data_builder = data_builder.max_subscribers(max_subscribers);
            }
            if let Some(max_publishers) = topic_config.max_publishers {
                // Same per-setter verification semantics.
                data_builder = data_builder.max_publishers(max_publishers);
            }
            if topic_config.history_size > 0 {
                // Arm iceoryx2 NATIVE publisher history at the
                // CREATOR/precreate site. The publisher port iceoryx2 hands back
                // on this service automatically gets a history queue of this size
                // — it retains the last N sent frames by SHM offset (zero-copy)
                // and auto-delivers them to a late subscriber on
                // `update_connections()`. Setting the value ALSO arms iceoryx2's
                // open-time at-least verification (existing < required fails),
                // but ONLY for openers that themselves call `.history_size()` —
                // so graph body/trigger subscribers and CLI introspection (which
                // go through `open_existing_topic_services` / the open-only path,
                // never setting it) impose no requirement and open fine. The
                // pool grows by `history_size` SHM slots
                // (max_subscribers*(buffer+borrow) + history_size +
                // publisher_max_loaned). `enable_safe_overflow` stays at its
                // default (true), so `history_size > subscriber buffer` is a soft
                // per-consumer truncation, not a hard open failure.
                data_builder = data_builder.history_size(topic_config.history_size);
            }
            data_builder
        };
        // Two-phase acquisition (the create-vs-open asymmetry — see
        // `TopicServiceConfig::create_borrowed_samples`): iceoryx2's builder
        // value is BOTH the create value AND an at-least open requirement, so
        // the owned-topic borrow FLOOR cannot ride a single `open_or_create`
        // without also REFUSING pre-existing sub-floor services (breaking the
        // degraded-attach contract — the regression that motivated this
        // split: a default opener's or a peer context's smaller service must
        // be attachable, warned, not fatal). When the create value differs
        // from the open requirement, split the acquisition explicitly:
        //
        //   - OPEN leg: arm only the GENUINE requirement (`open_borrow` —
        //     `None` unless the snapshot post-pass / recording raise / cross-group
        //     stamp set one), so a pre-existing smaller service attaches
        //     (the borrow-axis degraded warn below makes it loud).
        //   - CREATE leg: stamp the floor (`create_borrow`), so a service WE
        //     create is born HOLD-ready and start order between producer and
        //     consumer graphs cannot matter.
        //
        // The loop mirrors iceoryx2 0.9.1's own `open_or_create_impl` retry
        // discipline exactly (publish_subscribe.rs:714-760: retry the
        // open-absent -> create-raced TOCTOU plus mid-cleanup states, give up
        // as `SystemInFlux` after RETRY_LIMIT), so both branches below share
        // the same error surface and the one mapping underneath.
        let open_borrow = topic_config.subscriber_max_borrowed_samples;
        let create_borrow = topic_config.create_borrowed_samples();
        let data_result = if open_borrow == create_borrow {
            // No asymmetry (External passthrough, or a genuine requirement at
            // or above the floor): iceoryx2's native retry loop is exactly
            // equivalent — keep the single call.
            make_data_builder(open_borrow).open_or_create()
        } else {
            use iceoryx2::service::builder::publish_subscribe::{
                PublishSubscribeCreateError, PublishSubscribeOpenError,
                PublishSubscribeOpenOrCreateError,
            };
            // Mirrors iceoryx2's own RETRY_LIMIT (publish_subscribe.rs).
            const RETRY_LIMIT: usize = 5;
            let mut retry_count = 0usize;
            loop {
                if retry_count > RETRY_LIMIT {
                    break Err(PublishSubscribeOpenOrCreateError::SystemInFlux);
                }
                retry_count += 1;
                match make_data_builder(open_borrow).open() {
                    Ok(svc) => break Ok(svc),
                    // The retry set iceoryx2's own loop treats as transient
                    // (absent, or a peer mid-cleanup/mid-teardown).
                    Err(
                        PublishSubscribeOpenError::DoesNotExist
                        | PublishSubscribeOpenError::ServiceInCorruptedState
                        | PublishSubscribeOpenError::IsMarkedForDestruction,
                    ) => match make_data_builder(create_borrow).create() {
                        Ok(svc) => break Ok(svc),
                        // Lost the create race (or a peer is mid-create /
                        // mid-cleanup): re-run the open leg.
                        Err(
                            PublishSubscribeCreateError::AlreadyExists
                            | PublishSubscribeCreateError::ServiceInCorruptedState
                            | PublishSubscribeCreateError::IsBeingCreatedByAnotherInstance,
                        ) => continue,
                        Err(e) => break Err(e.into()),
                    },
                    Err(e) => break Err(e.into()),
                }
            }
        };
        let data_service = data_result.map_err(|e| {
            use iceoryx2::service::builder::publish_subscribe::{
                PublishSubscribeCreateError, PublishSubscribeOpenError,
                PublishSubscribeOpenOrCreateError,
            };
            // Classify BEFORE the variant is flattened into a reason
            // string — this is the one bit `map_err` destroys and the ingress
            // fallback's WARN depends on (see `OpenServicesFailure`).
            let buffer_ceiling_refused = is_buffer_ceiling_refusal(&e);
            // Two hint classes, formatted
            // differently. REQUIREMENT mismatches keep the requirements
            // parenthetical (the numbers ARE the diagnosis); ENVIRONMENT
            // failures (flux, corruption, type skew) get their remedy
            // WITHOUT it — splicing the requirements in front read as if
            // the requirement caused the corruption, and the numbers are
            // noise for those causes.
            let requirement_hint = match e {
                PublishSubscribeOpenOrCreateError::PublishSubscribeOpenError(
                    PublishSubscribeOpenError::DoesNotSupportRequestedMinBufferSize,
                ) => {
                    "; the service already exists with a smaller buffer ceiling — \
                     e.g. created by an earlier default opener like `cerulion \
                     topic echo` — stop it or start the graph first"
                }
                // The subscriber-slot twin of the same ordering edge.
                PublishSubscribeOpenOrCreateError::PublishSubscribeOpenError(
                    PublishSubscribeOpenError::DoesNotSupportRequestedAmountOfSubscribers,
                ) => {
                    "; the service already exists with fewer subscriber slots — \
                     e.g. created by an earlier default opener or another graph \
                     — stop it or start this graph first"
                }
                // The publisher-slot twin. Direction INVERTED vs
                // the subscriber arm (verified
                // against iceoryx2 0.9.1 publish_subscribe.rs:468: open
                // verification is at-least — fails only when existing <
                // required): graph provisioning LOWERS the publisher count
                // below iceoryx2's create-default (single-writer 1 < 2),
                // so the failing opener here is one requiring MORE than a
                // single-writer service provides (a raw multi-publisher
                // user, or the `multi_publisher_topics` opt-in opener) — never the graph's
                // own require-1 opener on a default-created service
                // (existing 2 ≥ required 1 passes).
                PublishSubscribeOpenOrCreateError::PublishSubscribeOpenError(
                    PublishSubscribeOpenError::DoesNotSupportRequestedAmountOfPublishers,
                ) => {
                    "; the service already exists with fewer publisher slots \
                     than this opener requires — e.g. a single-writer graph \
                     topic (max_publishers 1) opened by something requiring \
                     more — stop the owning graph or drop this opener's \
                     max_publishers requirement"
                }
                // The borrowed-sample twin of the same ordering edge.
                // A snapshot-consumer graph provisions subscriber_max_borrowed_samples
                // = 3 (the latest-value hold pushes the per-connection peak
                // one above iceoryx2's default 2); a service already created
                // with fewer borrowed slots — e.g. by an earlier default
                // opener — rejects it on open.
                PublishSubscribeOpenOrCreateError::PublishSubscribeOpenError(
                    PublishSubscribeOpenError::DoesNotSupportRequestedMinSubscriberBorrowedSamples,
                ) => {
                    "; the service already exists with fewer borrowed samples \
                     than this opener requires — a snapshot-consumer graph \
                     needs subscriber_max_borrowed_samples >= 3 (the \
                     latest-value hold) — stop the owning graph or earlier \
                     default opener, or start this graph first"
                }
                _ => "",
            };
            // The realistic non-requirement variants
            // (reachability verified against the
            // open_or_create retry loop at publish_subscribe.rs:735-760):
            // create-side ServiceInCorruptedState is always retried and
            // resurfaces as SystemInFlux after RETRY_LIMIT (5), so only
            // the OPEN-side variants (propagated by the availability
            // check) carry corruption/type-skew/hangs arms — delegated to
            // `publish_subscribe_open_env_hint`, shared with the open-only
            // introspection path. SystemInFlux itself must name the
            // stale-state remedy, because persistent corruption lands
            // THERE, where "retry" alone would never heal it. The
            // create-side HangsInCreation arm reuses the helper's text via
            // the open-side variant (same remedy; one source of truth).
            let environment_hint = match &e {
                PublishSubscribeOpenOrCreateError::SystemInFlux => {
                    "; another process is creating/removing this service at \
                     high frequency, or a crashed peer left stale service \
                     state that cleanup keeps tripping on (iceoryx2 retried \
                     internally and gave up) — retry after a moment; if it \
                     persists, remove the stale iceoryx2 shared-memory \
                     artifacts (or reboot)"
                }
                PublishSubscribeOpenOrCreateError::PublishSubscribeOpenError(inner) => {
                    publish_subscribe_open_env_hint(inner)
                }
                PublishSubscribeOpenOrCreateError::PublishSubscribeCreateError(
                    PublishSubscribeCreateError::HangsInCreation,
                ) => publish_subscribe_open_env_hint(&PublishSubscribeOpenError::HangsInCreation),
                _ => "",
            };
            let error = if !environment_hint.is_empty() {
                map_err(format!("{e}{environment_hint}"))
            } else {
                let max_subs = topic_config
                    .max_subscribers
                    .map(|n| format!(", max_subscribers {n}"))
                    .unwrap_or_default();
                let max_pubs = topic_config
                    .max_publishers
                    .map(|n| format!(", max_publishers {n}"))
                    .unwrap_or_default();
                // On a Multi topic the
                // requirement mismatch has a DIFFERENT likely cause than
                // the single-writer/default narratives above — a foreign
                // (or older-build) creator made the service without the
                // shared loose caps, or two Cerulion builds disagree on
                // the constants (version skew).
                let multi_hint = if topic_config.publisher_provisioning
                    == PublisherProvisioning::Multi
                    && matches!(
                        e,
                        PublishSubscribeOpenOrCreateError::PublishSubscribeOpenError(
                            PublishSubscribeOpenError::DoesNotSupportRequestedAmountOfPublishers
                                | PublishSubscribeOpenError::DoesNotSupportRequestedAmountOfSubscribers
                                | PublishSubscribeOpenError::DoesNotSupportRequestedMinBufferSize
                        )
                    ) {
                    "; this is a multi_publisher_topics topic — every \
                     participant must provision the shared loose caps, so a \
                     creator using iceoryx2 defaults (e.g. a foreign process \
                     or a raw transport user) locks listed graphs out: let a \
                     graph that lists the topic create the service first. If \
                     both sides ARE Cerulion graphs listing the topic, the \
                     two binaries were built with different loose-cap \
                     constants — rebuild against one Cerulion version"
                } else {
                    ""
                };
                map_err(format!(
                    "{e} (this opener requires subscriber_max_buffer_size \
                     {}{max_subs}{max_pubs}{requirement_hint}){multi_hint}",
                    topic_config.subscriber_max_buffer_size
                ))
            };
            OpenServicesFailure {
                error,
                buffer_ceiling_refused,
            }
        })?;

        // Loud-over-silent (with a
        // subscriber twin + a once-per-topic dedup): iceoryx2 enforces
        // port caps only if WE created the service — open verification is
        // at-least, so a pre-existing service with MORE slots than this
        // opener provisions satisfies the requirement SILENTLY. For
        // publishers that voids the single-writer contract (a rogue can
        // take the spare slot — the structured `single_writer_voided`
        // field flags exactly that case, since the multi-publisher opt-in
        // reuses this path with required > 1); for subscribers it voids
        // the exactly-(in-graph + headroom) bound. `live >
        // required` also proves the service PRE-EXISTED: creation sets
        // live == required exactly (the ≥1 choke guard makes iceoryx2's
        // 0→1 clamp unreachable).
        if let Some(required) = topic_config.max_publishers {
            let live = data_service.static_config().max_publishers();
            if live > required && self.mark_degraded_once(topic, "publishers") {
                tracing::warn!(
                    topic = %topic,
                    required,
                    live,
                    single_writer_voided = required == 1,
                    "publisher provisioning degraded — the service pre-existed \
                     with more publisher slots than this opener provisions, so \
                     out-of-graph publishers can attach beyond the declared \
                     bound; stop the earlier creator and restart this graph to \
                     reclaim the provisioning"
                );
            }
        }
        if let Some(required) = topic_config.max_subscribers {
            let live = data_service.static_config().max_subscribers();
            if live > required && self.mark_degraded_once(topic, "subscribers") {
                tracing::warn!(
                    topic = %topic,
                    required,
                    live,
                    "subscriber provisioning degraded — the service pre-existed \
                     with more subscriber slots than this opener provisions \
                     (the declared subscriber bound is not enforced); \
                     stop the earlier creator and restart this graph to reclaim \
                     the provisioning"
                );
            }
        }
        // The borrow-axis degrade twin — direction INVERTED vs the
        // port-cap warns above (there the service pre-existed with MORE than
        // required; here with LESS than the owned-topic create floor).
        // Reachable ONLY via the two-phase OPEN leg: creation stamps
        // `create_borrowed_samples()` >= the floor, and a genuine armed
        // requirement (>= 3) cannot open a sub-3 service at all — so
        // `live < floor` proves the service PRE-EXISTED below the floor
        // (e.g. created by a default opener, or by a peer context that only
        // consumes this topic). The owner attaches (degraded-attach contract)
        // but the consequence must be loud: a HOLD consumer of this
        // topic would be refused at open until the earlier creator goes away.
        // External topics are exempt — the graph neither creates nor floors a
        // foreign service, so there is nothing to degrade FROM.
        if topic_config.publisher_provisioning != PublisherProvisioning::External {
            let live = data_service
                .static_config()
                .subscriber_max_borrowed_samples();
            if live < SUBSCRIBER_MAX_BORROWED_HELD
                && self.mark_degraded_once(topic, "borrowed_samples")
            {
                tracing::warn!(
                    topic = %topic,
                    floor = SUBSCRIBER_MAX_BORROWED_HELD,
                    live,
                    "borrowed-samples provisioning degraded — the service \
                     pre-existed below the owned-topic borrow floor, \
                     so a snapshot (latest-value hold) consumer of \
                     this topic would be refused at open; stop the earlier \
                     creator (e.g. a default opener like `cerulion topic \
                     echo`) and restart this graph to reclaim the floor"
                );
            }
        }
        // The create-leg borrow FLOOR twin (`TopicServiceConfig::create_borrow_floor`
        // — the rmw loanable-type opener, which is `External`-provisioned and so
        // exempt from the owned-topic warn above). Same inverted direction, same
        // proof: a service WE created carries `create_borrowed_samples()`, which
        // is max-composed with the floor and therefore never below it — so
        // `live < requested` means the service PRE-EXISTED below the floor and
        // the OPEN leg (tolerant by design) attached anyway. The consequence is
        // an advertised loan budget that silently shrank to the earlier
        // creator's value: a subscriber holding that many loaned takes is
        // refused its next receive. Loud once per (topic, kind), on EVERY
        // provisioning arm — an opener that requested a floor and did not get
        // it is degraded whether or not it owns the topic. No floor requested
        // ⇒ nothing to degrade from ⇒ no warn — and "requested" means the
        // EFFECTIVE floor (`effective_create_borrow_floor`): a floor at/below
        // the iceoryx2 default imposes nothing on the create leg, so a
        // pre-existing borrow-1 service under a `Some(2)` floor is a valid
        // configuration, not a degradation.
        if let Some(requested) = topic_config.effective_create_borrow_floor() {
            let live = data_service
                .static_config()
                .subscriber_max_borrowed_samples();
            if live < requested && self.mark_degraded_once(topic, "borrow_floor") {
                tracing::warn!(
                    topic = %topic,
                    requested,
                    live,
                    "loan borrow floor degraded — the service pre-existed with \
                     fewer subscriber_max_borrowed_samples than this opener's \
                     create floor, so the loan budget on this topic is the \
                     smaller pre-existing value (a subscriber holding that many \
                     loaned takes is refused its next receive); stop the earlier \
                     creator and restart this opener to reclaim the floor"
                );
            }
        }

        // The missed attach class: every data subscriber
        // ALSO attaches one listener + one notifier to the event service,
        // and every publisher attaches one of each — iceoryx2's defaults
        // (16/16) would silently narrow the introspection-headroom promise
        // on high-fan-out topics (in-graph subscribers ≥
        // 16 - INTROSPECTION_SUBSCRIBER_HEADROOM) and reject the graph's OWN
        // attaches at ≥16.
        //
        // Both terms come from the live
        // data service (static_config = the creator's truth, available
        // because the data service opens first), never from this opener's
        // requested intent. Intent and truth diverge exactly when it
        // matters: a pre-existing service with more ports than required
        // (the degraded-open warns above) — requested-intent terms would
        // under-provision event ports for attachers the data service
        // admits (they'd pass the data-port cap and die at event-port
        // create, blamed on a rogue that doesn't exist) — and the
        // requested-subs opener whose publisher default would overshoot a
        // single-writer topic's caps (subs_req + OUR default 2 > subs + 1
        // — a lockout for tooling asserting subscriber capacity).
        // live_subs + live_pubs covers every attacher the data service
        // can admit, and an armed open requirement computed from the same
        // live values can never exceed caps a same-formula creator
        // provisioned (both read the same static_config). Armed when the
        // opener carries ANY port requirement — `None`+`None` default
        // openers set nothing and carry no requirement (the default-opener
        // asymmetry, preserved). The live sum is clamped to
        // MAX_REASONABLE_PORTS: a foreign creator can provision arbitrary
        // counts (iceoryx2 applies no upper clamp), and the choke-point
        // guards only cover THIS opener's requested config, never
        // live-derived terms.
        let live_data = data_service.static_config();
        let armed_event_ports = (topic_config.max_subscribers.is_some()
            || topic_config.max_publishers.is_some())
        .then(|| {
            // `extra_event_listeners` covers standalone
            // listener-only `Listener`s (unified data-trigger wake sources)
            // that have NO backing subscriber port, so they are NOT in the
            // live data service's `max_subscribers()` term. The runtime threads
            // the SAME per-topic value into every open site's `topic_config`
            // (create + open), so the create arm provisions these slots and the
            // open arms verify the identical at-least requirement (passes by
            // equality). External topics carry 0 here AND leave both port caps
            // `None`, so the `armed_event_ports` gate above is `false` for them
            // — this term can never force-arm an External topic's event caps.
            let ports = live_data
                .max_subscribers()
                .saturating_add(live_data.max_publishers())
                .saturating_add(topic_config.extra_event_listeners);
            if ports > MAX_REASONABLE_PORTS {
                tracing::warn!(
                    topic = %topic,
                    ports,
                    bound = MAX_REASONABLE_PORTS,
                    "live-derived event port budget exceeds the sanity bound — \
                     clamping; the data service was provisioned with an \
                     unreasonable port count by its creator"
                );
                MAX_REASONABLE_PORTS
            } else {
                ports
            }
        });
        let mut event_builder = self.node.service_builder(&event_service_name).event();
        if let Some(event_ports) = armed_event_ports {
            event_builder = event_builder
                .max_listeners(event_ports)
                .max_notifiers(event_ports);
        }
        let event_service = event_builder.open_or_create().map_err(|e| {
            use iceoryx2::service::builder::event::{EventOpenError, EventOpenOrCreateError};
            // The event-service twin of the data
            // open's hint arms (appending the caps parenthetical to
            // EVERY failure, corruption included, would mislead).
            // Open-only parity: the env arms moved to
            // `event_env_hint`, shared with the open-only introspection
            // path — see its doc for the retry-set verification.
            let environment_hint = event_env_hint(&e);
            if !environment_hint.is_empty() {
                map_err(format!("{e}{environment_hint}"))
            } else {
                let ordering_hint = if matches!(
                    e,
                    EventOpenOrCreateError::EventOpenError(
                        EventOpenError::DoesNotSupportRequestedAmountOfListeners
                            | EventOpenError::DoesNotSupportRequestedAmountOfNotifiers
                    )
                ) {
                    "; the event service already exists with fewer event \
                     ports than this opener requires — e.g. created by a raw \
                     event-service user — stop it or start this graph first"
                } else {
                    ""
                };
                let caps = armed_event_ports
                    .map(|n| format!(" (this opener requires max_listeners/max_notifiers {n})"))
                    .unwrap_or_default();
                map_err(format!("{e}{caps}{ordering_hint}"))
            }
        })?;

        Ok((data_service, event_service))
    }
}

/// Reject a zero subscriber buffer at transport
/// init. The other zero ingress points carry their own checks:
/// `create_subscriber_with_buffers` rejects a zero BUFFER, and
/// `open_topic_services` rejects a zero CEILING (covering the publisher
/// path for hand-mutated configs). iceoryx2 would otherwise silently
/// clamp 0 → 1 at service creation under a log level Cerulion suppresses —
/// the user configures 0, gets 1, and sees nothing (the silent-inference
/// class the loud-over-silent rule forbids). Graph topics are safe either
/// way (topology validates depth ≥ 1), so this guards the non-graph
/// default paths.
fn validate_subscriber_buffer_size(size: usize) -> TransportResult<()> {
    if size == 0 {
        return Err(TransportError::InvalidTransportConfig {
            reason: "TransportConfig.subscriber_buffer_size must be >= 1 (a zero \
                     buffer cannot hold any sample; iceoryx2 would silently clamp \
                     it to 1)"
                .to_string(),
        });
    }
    Ok(())
}

/// Turn a configured node name into an iceoryx2 [`NodeName`], or
/// REFUSE it loudly.
///
/// The ONE conversion every `TransportManager` constructor goes through. It used
/// to be `.expect("invalid node name")` at each of them, which made a graph named
/// `café` — or a bag DECLARING such a `name:` — abort the process with no exit
/// code and no diagnostic, on the path every `graph run` and every resim shares.
///
/// iceoryx2 is the AUTHORITY on the verdict: `NodeName::new` decides, and a
/// refusal is only then CLASSIFIED for its message. Re-encoding the rule here
/// would put a second copy of a dependency's constraint in this crate, free to
/// drift from it silently (a first-pass reading of iceoryx2 once stopped at the
/// capacity-only `StaticString` conversion and missed the charset check one level
/// below it).
fn node_name_or_refusal(raw: &str) -> TransportResult<NodeName> {
    NodeName::new(raw).map_err(|_| TransportError::UnrepresentableNodeName {
        // hot-path-alloc-ok: transport construction, once per process, and only
        // on the REFUSAL arm — this path ends in an Err, never in a node.
        node_name: raw.to_string(),
        cause: classify_node_name_refusal(raw),
    })
}

/// Which constraint a name iceoryx2 ALREADY REFUSED broke.
///
/// PURE, and deliberately only reachable after a refusal — it never decides
/// whether a name is valid, only which of the two messages that refusal deserves.
/// Length is asked first purely because it produces the better message (the two
/// numbers an operator can act on); everything else is the charset, which is what
/// `iceoryx2-bb-container`'s `insert_bytes` rejects (any byte `>= 0x80` or `== 0`)
/// one level below the capacity check.
pub(crate) fn classify_node_name_refusal(raw: &str) -> NodeNameRefusal {
    let max = NodeName::max_len();
    if raw.len() > max {
        NodeNameRefusal::TooLong {
            len: raw.len(),
            max,
        }
    } else {
        NodeNameRefusal::Charset
    }
}

/// Validate that the topic name is within iceoryx2's service name limit.
fn validate_topic_name(topic: &str) -> Result<(), String> {
    if topic.is_empty() {
        return Err("topic name cannot be empty".to_string());
    }
    if topic.len() > MAX_TOPIC_LEN {
        return Err(format!(
            "topic name '{}' is {} characters, max is {} (iceoryx2 service names limited to 255 chars, minus '/event' suffix)",
            topic, topic.len(), MAX_TOPIC_LEN
        ));
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// This module's half of the ABI LAYOUT PIN (see `crate::abi_layout`).
//
// `abi_pin_struct!` expands to an exhaustive destructuring pattern with no `..`
// rest pattern, so adding or removing a field of one of these structs is a
// COMPILE ERROR naming the struct; it also measures size/align/`offset_of!`,
// which `crate::abi_layout` compares against the snapshot table keyed to
// `CERULION_ABI_VERSION`. `abi_pin_enum!` does the same for a variant set.
// ---------------------------------------------------------------------------
#[cfg(test)]
pub(crate) fn abi_layout_pins() -> Vec<crate::abi_layout::MeasuredStruct> {
    use crate::abi_layout::abi_pin_struct;
    vec![abi_pin_struct!(TransportManager {
        network,
        node,
        clock,
        subscriber_buffer_size,
        bridge_mgr,
        network_watch_started,
        degraded_warned,
        dynamic_egress,
        dynamic_egress_create,
        ingress_build,
        replay_sequence_seeds,
        mirror_registry,
        self_weak
    })]
}

#[cfg(test)]
mod tap_depth_tests {
    use super::*;

    /// The slot really is `align(header + payload, header_align)` — checked
    /// against iceoryx2's OWN `sample_layout` arithmetic transcribed by hand,
    /// with the header size read from the type rather than repeated.
    ///
    /// The BOUNDARY cases are the point: `iceoryx2_slot_bytes` is only ever used
    /// as a DIVISOR, so what matters is that it is strictly greater than the
    /// nominal slice at every size, and that it rounds UP rather than down (a
    /// slot rounded down would over-commit the budget by a fraction of a slot at
    /// every unaligned slice).
    #[test]
    fn a_slot_is_the_header_plus_the_payload_rounded_up_to_the_headers_alignment() {
        // The two constants this rule reads out of iceoryx2. Asserted here so a
        // change upstream is visible as a NUMBER in a failing test rather than
        // only as a shifted depth three assertions later.
        assert_eq!(
            IOX2_SAMPLE_HEADER_BYTES, 40,
            "iceoryx2 0.9.1's publish-subscribe Header is 40 bytes \
             (UniqueNodeId 16 + UniquePublisherId 16 + u64 8) — if this moved, \
             every depth below moves with it and that is the point of reading it"
        );
        assert_eq!(IOX2_SAMPLE_HEADER_ALIGN, 8);

        // Aligned slices: the overhead is FLATLY the header.
        assert_eq!(iceoryx2_slot_bytes(0), 40);
        assert_eq!(iceoryx2_slot_bytes(256), 296);
        assert_eq!(iceoryx2_slot_bytes(4096), 4136);
        assert_eq!(iceoryx2_slot_bytes(1024 * 1024), 1_048_616);
        assert_eq!(iceoryx2_slot_bytes(16 * 1024 * 1024), 16_777_256);

        // UNALIGNED slices round UP. 157 is a real `/tf` frame size,
        // so this is not a synthetic number: 40 + 157 = 197 → 200.
        assert_eq!(iceoryx2_slot_bytes(157), 200);
        // One byte either side of a multiple of 8, to pin the rounding on both
        // edges rather than at one convenient point.
        assert_eq!(iceoryx2_slot_bytes(1), 48);
        assert_eq!(iceoryx2_slot_bytes(7), 48);
        assert_eq!(iceoryx2_slot_bytes(8), 48);
        assert_eq!(iceoryx2_slot_bytes(9), 56);

        // The property every caller relies on: a slot is never SMALLER than its
        // payload, at any size, so a budget divided by it can never admit more
        // bytes than it names.
        for n in [0usize, 1, 7, 63, 157, 4095, 1 << 20, 1 << 24] {
            assert!(
                iceoryx2_slot_bytes(n) > n,
                "slot for {n} must exceed the payload"
            );
        }

        // Saturating rather than wrapping at the top: a wrap would produce a
        // TINY slot and therefore an enormous depth — the exact inversion.
        assert_eq!(iceoryx2_slot_bytes(usize::MAX), usize::MAX);
    }

    /// THE budget arithmetic, and the arm that fails if anybody swaps the real
    /// slot for the nominal slice.
    ///
    /// The two discriminating rows are the exact powers of two, which is where
    /// real robots sit: at the shipped 64 MiB budget a nominal divisor answers
    /// 64 and 4, and the real one answers 63 and 3. Every other row here would
    /// pass under either divisor, which is why these two carry the claim.
    #[test]
    fn the_budget_divides_by_the_real_slot_not_the_nominal_slice() {
        const BUDGET: u64 = 64 * 1024 * 1024;
        const CEILING: usize = 4096;

        // 1 MiB slice: 64 MiB / 1_048_616 = 63. A nominal divisor says 64.
        assert_eq!(
            flashback_tap_buffer_depth(BUDGET, 1024 * 1024, CEILING),
            63,
            "a nominal divisor would answer 64 and over-commit the budget by a slot"
        );
        // 16 MiB slice: 64 MiB / 16_777_256 = 3. A nominal divisor says 4.
        assert_eq!(
            flashback_tap_buffer_depth(BUDGET, 16 * 1024 * 1024, CEILING),
            3,
            "a nominal divisor would answer 4"
        );
        // The claim above, stated as the comparison rather than as two literals,
        // so it cannot be satisfied by an oracle somebody re-blessed.
        for slice in [1024 * 1024usize, 16 * 1024 * 1024, 4 * 1024 * 1024] {
            let nominal = (BUDGET / slice as u64) as usize;
            assert!(
                flashback_tap_buffer_depth(BUDGET, slice, CEILING) < nominal,
                "slice {slice}: the real-slot depth must be strictly under the nominal one"
            );
        }

        // A small slice is bounded by the CEILING, not the budget: 64 MiB of
        // 296-byte slots is 226 719 slots, and no service is that deep.
        assert_eq!(flashback_tap_buffer_depth(BUDGET, 256, CEILING), CEILING);
        assert_eq!(flashback_tap_buffer_depth(BUDGET, 256, 16), 16);
    }

    /// The FLOOR binds for a slot the budget cannot buy even one of — and the
    /// ceiling still wins when it is lower, because a service provisioned at
    /// depth 1 cannot be opened at 2.
    #[test]
    fn the_floor_binds_for_a_huge_slot_and_the_ceiling_still_outranks_it() {
        const BUDGET: u64 = 64 * 1024 * 1024;

        // A 128 MiB slice (the tier-3 default) buys ZERO slots at this budget.
        assert_eq!(BUDGET / iceoryx2_slot_bytes(128 * 1024 * 1024) as u64, 0);
        assert_eq!(
            flashback_tap_buffer_depth(BUDGET, 128 * 1024 * 1024, 4096),
            FLASHBACK_TAP_BUFFER_DEPTH_FLOOR
        );
        // Exactly AT the budget is one slot, which the floor lifts to two — the
        // boundary on the side the floor is for.
        assert_eq!(
            flashback_tap_buffer_depth(BUDGET, 64 * 1024 * 1024, 4096),
            2
        );
        // And just under it, two slots, which the floor does not touch: the
        // other side of the same boundary, so the floor cannot be mistaken for a
        // constant answer.
        assert_eq!(
            flashback_tap_buffer_depth(BUDGET, 32 * 1024 * 1024, 4096),
            2
        );
        assert_eq!(
            flashback_tap_buffer_depth(BUDGET, 16 * 1024 * 1024, 4096),
            3
        );

        // A shallow service outranks the floor rather than panicking the clamp.
        assert_eq!(flashback_tap_buffer_depth(BUDGET, 128 * 1024 * 1024, 1), 1);
        // Including the degenerate ceiling a caller must never pass but might.
        assert_eq!(flashback_tap_buffer_depth(BUDGET, 256, 0), 0);

        // A budget of ZERO is not reachable through the resolver (a `0` env is
        // refused and falls back to the default), but the rule must still answer
        // the floor rather than a depth that can never receive.
        assert_eq!(flashback_tap_buffer_depth(0, 256, 4096), 2);
    }
}

/// The node-name refusal — the classifier, the message it renders, and
/// the SINGLETON door itself.
///
/// The singleton arms live in `src/` rather than under `tests/` on purpose. Every
/// `TransportManager::init` call here is a REFUSAL, which returns before
/// `get_or_init` and installs nothing, so the file touches no iceoryx2 namespace
/// and needs neither `#[serial]` nor a nextest fence entry — while the arms under
/// `tests/` (which DO build managers) reach only the per-test-root constructors.
/// The `INSTANCE`-stays-unset assertion below is what makes that claim testable
/// rather than asserted.
#[cfg(test)]
mod node_name_refusal_tests {
    use super::*;

    /// The classifier picks the arm with the better message, and does it by
    /// LENGTH — hand oracle, not a self-compare.
    ///
    /// It never decides validity (iceoryx2 already refused by the time it runs),
    /// so it is asked only about names that are genuinely refused.
    #[test]
    fn classify_names_the_constraint_that_broke() {
        let max = NodeName::max_len();

        // Over-long ASCII: the two numbers an operator acts on.
        let over = "a".repeat(max + 1);
        assert_eq!(
            classify_node_name_refusal(&over),
            NodeNameRefusal::TooLong { len: max + 1, max }
        );
        // Far over-long, to pin that `len` is the REAL length and not the cap.
        let way_over = "a".repeat(max * 3);
        assert_eq!(
            classify_node_name_refusal(&way_over),
            NodeNameRefusal::TooLong { len: max * 3, max }
        );

        // Fits, so the charset is what broke. `café` is 5 BYTES for 4 chars —
        // the classifier must weigh bytes, since the cap is a byte capacity.
        assert_eq!(classify_node_name_refusal("café"), NodeNameRefusal::Charset);
        assert_eq!(
            classify_node_name_refusal("cerulion_日本語"),
            NodeNameRefusal::Charset
        );
        // A NUL is the OTHER thing `insert_bytes` rejects, and it is not a
        // charset-vs-length question the length arm can answer.
        assert_eq!(
            classify_node_name_refusal("cerulion_a\0b"),
            NodeNameRefusal::Charset
        );
    }

    /// A name that is BOTH over-long and non-ASCII reports its LENGTH.
    ///
    /// The order is the contract, not an accident: both constraints are broken,
    /// and the length arm is asked first because it is the one that can quote
    /// numbers. Pinning it stops a later reorder from silently downgrading every
    /// over-long refusal to the generic charset sentence.
    #[test]
    fn a_name_breaking_both_constraints_reports_its_length() {
        let max = NodeName::max_len();
        let both = "é".repeat(max); // 2 bytes each ⇒ 2*max bytes, and non-ASCII.
        assert_eq!(
            classify_node_name_refusal(&both),
            NodeNameRefusal::TooLong { len: 2 * max, max }
        );
    }

    /// The rendered message carries the cause and the remedy (by design), and
    /// the length arm carries BOTH numbers.
    #[test]
    fn the_refusal_message_names_the_cause_the_name_and_the_remedy() {
        let long = TransportError::UnrepresentableNodeName {
            node_name: "cerulion_verylong".to_string(),
            cause: NodeNameRefusal::TooLong { len: 300, max: 128 },
        }
        .to_string();
        assert!(long.contains("cerulion_verylong"), "{long}");
        assert!(long.contains("300 bytes"), "{long}");
        assert!(long.contains("128"), "{long}");
        assert!(long.contains("rename the graph"), "{long}");

        let charset = TransportError::UnrepresentableNodeName {
            node_name: "cerulion_café".to_string(),
            cause: NodeNameRefusal::Charset,
        }
        .to_string();
        assert!(charset.contains("cerulion_café"), "{charset}");
        assert!(charset.contains("U+0080"), "{charset}");
        assert!(charset.contains("rename the graph"), "{charset}");
        // The charset arm must NOT claim a length problem — that is the whole
        // reason the two arms exist.
        assert!(!charset.contains("bytes, past the"), "{charset}");
    }

    /// THE headline: the singleton door REFUSES an unrepresentable identity
    /// instead of aborting, and leaves no transport state behind.
    ///
    /// `TransportManager::init` is what `cerulion graph run` calls with
    /// `cerulion_{graph}`, so these are the exact shapes a YAML `name:` or a
    /// bag's declared name produces. Every call fails, so nothing is installed —
    /// asserted directly via `get()`, which is also what keeps this module out of
    /// the default-namespace fence.
    #[test]
    fn init_refuses_an_unrepresentable_identity_and_installs_nothing() {
        let max = NodeName::max_len();
        let cases: Vec<(String, NodeNameRefusal)> = vec![
            (
                format!("cerulion_{}", "g".repeat(max)),
                NodeNameRefusal::TooLong {
                    len: "cerulion_".len() + max,
                    max,
                },
            ),
            ("cerulion_café".to_string(), NodeNameRefusal::Charset),
            ("cerulion_日本語".to_string(), NodeNameRefusal::Charset),
        ];
        for (name, expected) in cases {
            let err = TransportManager::init(TransportConfig {
                node_name: name.clone(),
                clock: Arc::new(RealClock),
                subscriber_buffer_size: 8,
                network: None,
            })
            .err()
            // Not `expect_err`: that would require `Arc<TransportManager>: Debug`,
            // which it deliberately is not (`GraphRuntime` is `!Debug` for the
            // same reason — these types own live iceoryx2 handles).
            .unwrap_or_else(|| panic!("`{name}` must be REFUSED, never expected away"));
            match &err {
                TransportError::UnrepresentableNodeName { node_name, cause } => {
                    assert_eq!(node_name, &name);
                    assert_eq!(cause, &expected, "wrong constraint reported for `{name}`");
                }
                other => panic!("`{name}` produced the wrong error: {other:?}"),
            }
            // The message must name the offending identity — an operator's only
            // handle on which graph to rename.
            assert!(err.to_string().contains(&name), "{err}");
        }

        // A refused init creates nothing: the process-global instance is still
        // unset. (True for the whole binary — no test in this module ever hands
        // `init` a name it accepts.)
        assert!(
            TransportManager::get().is_err(),
            "a refused init must leave the process-global instance unset"
        );
    }

    /// The length boundary, driven through iceoryx2 on BOTH sides.
    ///
    /// The refusal must begin exactly one byte past what `NodeName` admits — if
    /// it began earlier we would refuse names iceoryx2 accepts, and the ONLY way
    /// to know is to ask iceoryx2 rather than the constant this module also reads.
    #[test]
    fn the_length_boundary_is_pinned_on_both_sides_against_iceoryx2_itself() {
        let max = NodeName::max_len();
        let longest = "a".repeat(max);
        assert!(
            NodeName::new(&longest).is_ok(),
            "the cap itself must be admissible, or the boundary moved"
        );
        assert!(
            node_name_or_refusal(&longest).is_ok(),
            "a name AT the cap must convert — refusing it would be stricter than iceoryx2"
        );

        let one_more = "a".repeat(max + 1);
        assert!(
            NodeName::new(&one_more).is_err(),
            "one byte past the cap must be refused by iceoryx2, or the boundary moved"
        );
        assert!(matches!(
            node_name_or_refusal(&one_more),
            Err(TransportError::UnrepresentableNodeName {
                cause: NodeNameRefusal::TooLong { .. },
                ..
            })
        ));
    }

    /// The charset boundary, likewise on both sides: U+007F is the last code
    /// point a node name admits, U+0080 the first it refuses.
    ///
    /// Pins the exact edge rather than "some accented word is rejected", and pins
    /// it against the authority — a widened charset in a future iceoryx2 shows up
    /// here as a failure instead of as silently narrower behaviour.
    #[test]
    fn the_charset_boundary_is_pinned_on_both_sides_against_iceoryx2_itself() {
        let admitted = format!("cerulion_ok{}", '\u{7f}');
        let refused = format!("cerulion_ok{}", '\u{80}');
        assert!(
            NodeName::new(&admitted).is_ok(),
            "U+007F must still be a valid node name, or the boundary moved"
        );
        assert!(node_name_or_refusal(&admitted).is_ok());
        assert!(matches!(
            node_name_or_refusal(&refused),
            Err(TransportError::UnrepresentableNodeName {
                cause: NodeNameRefusal::Charset,
                ..
            })
        ));
    }

    /// ANTI-TAUTOLOGY: the conversion still ACCEPTS the names production
    /// actually mints, so "refuses" above is not "refuses everything".
    ///
    /// Both real prefixes are covered — `graph run`'s `cerulion_{graph}` and the
    /// resim's `cerulion_replay_{identity}` — plus the empty name, which iceoryx2
    /// admits and which nothing here may start refusing.
    #[test]
    fn ordinary_identities_still_convert() {
        for name in [
            "cerulion",
            "cerulion_perception",
            "cerulion_replay_perception_20260101T000000Z",
            "cerulion_my_graph_2",
            "",
        ] {
            let converted = node_name_or_refusal(name)
                .unwrap_or_else(|e| panic!("`{name}` must still be a valid node name: {e}"));
            assert_eq!(converted.as_str(), name);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The startup sweep's FAILED-removal arm is the only surface of a fault
    /// with an operator remedy, so it is a `warn!` naming `cerulion clean`;
    /// a sweep that only succeeded or found nothing is bookkeeping at
    /// `debug!`, and an exhausted budget is a SEPARATE warn with a different
    /// remedy. Driven by hand-built sweep reports (a real failed removal
    /// needs a dead node owned by another user); every level read through
    /// `crate::testing` (the line HEADER's level token, never a substring),
    /// and the clean path asserted to draw NO warn at all.
    ///
    /// Release-safe: this crate sets `release_max_level_info`, so `debug!` is
    /// compiled out of `cargo test --lib --release` and the clean arm's DEBUG
    /// count reads 0 there. The count is therefore routed through
    /// `debug_lines_expected`, and its `never_loud` twin is the half that still
    /// binds in release: a clean report promoted to `info!`/`warn!`/`error!`
    /// fails in both profiles.
    #[tracing_test::traced_test]
    #[test]
    fn startup_sweep_report_warns_only_on_a_failed_removal_or_a_spent_budget() {
        const FAILED: &str = "could not reclaim some dead iceoryx2 node state";
        const BUDGET: &str = "ran out of its startup budget";
        const CLEAN: &str = "startup dead-node sweep (runtime auto-cleanup disabled)";

        // The clean path: nothing found, or everything reclaimed. No warn.
        report_startup_dead_node_sweep(&dead_node_sweep::BoundedCleanup::default());
        report_startup_dead_node_sweep(&dead_node_sweep::BoundedCleanup {
            cleanups: 3,
            failed_cleanups: 0,
            deferred: 0,
        });
        logs_assert(|lines: &[&str]| {
            let warns = lines
                .iter()
                .filter(|l| crate::testing::line_level(l) == Some("WARN"))
                .count();
            if warns != 0 {
                return Err(format!("a clean sweep must draw no WARN, got {warns}"));
            }
            crate::testing::never_loud(lines, CLEAN)?;
            let clean = crate::testing::count_at_exclusively(lines, "DEBUG", &[CLEAN])?;
            let want_clean = crate::testing::debug_lines_expected(2);
            if clean != want_clean {
                return Err(format!(
                    "expected {want_clean} clean reports at DEBUG (2, or 0 where `debug!` is \
                     compiled out), got {clean}"
                ));
            }
            Ok(())
        });

        // A FAILED removal: exactly one WARN, naming the remedy, and NOT the
        // budget arm (the two remedies differ).
        report_startup_dead_node_sweep(&dead_node_sweep::BoundedCleanup {
            cleanups: 1,
            failed_cleanups: 2,
            deferred: 0,
        });
        logs_assert(|lines: &[&str]| {
            let failed = crate::testing::count_at_exclusively(lines, "WARN", &[FAILED])?;
            if failed != 1 {
                return Err(format!(
                    "expected exactly 1 WARN failed-removal line, got {failed}"
                ));
            }
            if !lines
                .iter()
                .any(|l| l.contains(FAILED) && l.contains("`cerulion clean`"))
            {
                return Err("the failed-removal warn must name `cerulion clean`".to_string());
            }
            let budget = crate::testing::count_at(lines, "WARN", BUDGET);
            if budget != 0 {
                return Err(format!(
                    "a failed removal is not a spent budget, got {budget} budget warns"
                ));
            }
            Ok(())
        });

        // A SPENT budget with no failure: the budget warn alone.
        report_startup_dead_node_sweep(&dead_node_sweep::BoundedCleanup {
            cleanups: 2,
            failed_cleanups: 0,
            deferred: 4,
        });
        logs_assert(|lines: &[&str]| {
            let budget = crate::testing::count_at_exclusively(lines, "WARN", &[BUDGET])?;
            if budget != 1 {
                return Err(format!("expected exactly 1 WARN budget line, got {budget}"));
            }
            let failed = crate::testing::count_at(lines, "WARN", FAILED);
            if failed != 1 {
                return Err(format!(
                    "the failed-removal count must not have moved (still 1), got {failed}"
                ));
            }
            // The spent-budget arm had no failure, so it printed the clean
            // line too; that line is still never loud, in either profile.
            crate::testing::never_loud(lines, CLEAN)
        });
    }

    /// The PURE producer-probe classifier: a KNOWN count
    /// (incl. a missing service = a dead route) is `Some`; a probe FAILURE is `None`
    /// (UNKNOWN), so a transient iceoryx2 error can NEVER dim a healthy topic as dead.
    /// Hand oracle over every arm (the error arm — untriggerable deterministically over
    /// real transport — is pinned HERE).
    #[test]
    fn classify_producer_probe_maps_count_deadroute_and_failure() {
        // A live producer count passes through verbatim.
        assert_eq!(classify_producer_probe(ProducerProbe::Count(3)), Some(3));
        // A live service with zero publishers is a dead route → Some(0).
        assert_eq!(classify_producer_probe(ProducerProbe::Count(0)), Some(0));
        // A never-created data service is ALSO a dead route → Some(0).
        assert_eq!(classify_producer_probe(ProducerProbe::NoService), Some(0));
        // THE fix: a probe FAILURE is UNKNOWN (None), NEVER a fabricated Some(0) — so a
        // transient fd-pressure error renders the row plainly, never dimmed as dead.
        assert_eq!(classify_producer_probe(ProducerProbe::Failed), None);
    }

    // The iceoryx2 automatic dead-node cleanup MUST be off on every
    // config a TransportManager node is built from — a dlopen'd cdylib's separate
    // iceoryx2 copy would otherwise reap the live host node's services (same-PID
    // fcntl F_GETLK defeat). Hand oracle: iceoryx2's compiled-in default has all
    // three flags ON, so the transform must FLIP each to false (not a
    // self-compare — the oracle is the upstream default).
    #[test]
    fn disable_auto_dead_node_cleanup_flips_all_three_flags() {
        let upstream = iceoryx2::config::Config::default();
        assert!(upstream.global.service.cleanup_dead_nodes_on_open);
        assert!(upstream.global.node.cleanup_dead_nodes_on_creation);
        assert!(upstream.global.node.cleanup_dead_nodes_on_destruction);

        let disabled = disable_auto_dead_node_cleanup(upstream);
        assert!(!disabled.global.service.cleanup_dead_nodes_on_open);
        assert!(!disabled.global.node.cleanup_dead_nodes_on_creation);
        assert!(!disabled.global.node.cleanup_dead_nodes_on_destruction);
    }

    // The singleton node-config resolver — the EXACT expression
    // `init_singleton` builds its node from — has the flags OFF on BOTH arms.
    #[test]
    fn resolve_singleton_node_config_disables_cleanup_on_both_arms() {
        // Default arm (ix_config = None): starts from the global default config.
        let default_arm = resolve_singleton_node_config(None);
        assert!(!default_arm.global.service.cleanup_dead_nodes_on_open);
        assert!(!default_arm.global.node.cleanup_dead_nodes_on_creation);
        assert!(!default_arm.global.node.cleanup_dead_nodes_on_destruction);

        // Passed-in arm — never trust the caller: hand it a config with all three
        // flags forced ON and require them OFF in the resolved config.
        let mut hostile = iceoryx2::config::Config::default();
        hostile.global.service.cleanup_dead_nodes_on_open = true;
        hostile.global.node.cleanup_dead_nodes_on_creation = true;
        hostile.global.node.cleanup_dead_nodes_on_destruction = true;
        let passed_in = resolve_singleton_node_config(Some(hostile));
        assert!(!passed_in.global.service.cleanup_dead_nodes_on_open);
        assert!(!passed_in.global.node.cleanup_dead_nodes_on_creation);
        assert!(!passed_in.global.node.cleanup_dead_nodes_on_destruction);
    }

    // `ix_config_shm_identity_from_json` extracts the FULL
    // SHM-discovery identity from a serialized iceoryx2 Config (root/prefix + the
    // service & node directories + the per-file service suffixes), and matches the
    // struct-path identity for the SAME config — the identity netd's egress plane
    // compares before tapping a forwarded run's SHM.
    #[test]
    fn ix_config_shm_identity_from_json_extracts_full_discovery_identity() {
        // Oracle: the identity of the DEFAULT config, read directly from the
        // struct (never a self-compare — the JSON path must agree with the struct
        // path). The mp deployment config IS this default namespace.
        let default_cfg = iceoryx2::config::Config::default();
        let want = IceoryxShmIdentity::from_config(&default_cfg);
        let json = serde_json::to_string(&default_cfg).expect("serialize default config");
        let got = ix_config_shm_identity_from_json(&json).expect("parse default config identity");
        assert_eq!(
            got, want,
            "the JSON path reads the same full identity as the struct path"
        );
        // Hand oracle for the well-known iceoryx2 0.9.1 defaults — every
        // discovery-keying field, so a codegen/default drift fails here loudly.
        assert_eq!(got.prefix, "iox2_", "default segment prefix");
        assert_eq!(got.service_dir, "services", "default service directory");
        assert_eq!(got.node_dir, "nodes", "default node directory");
        assert_eq!(got.service_static_config_suffix, ".service");
        assert_eq!(got.service_dynamic_config_suffix, ".dynamic");
        assert_eq!(got.service_data_segment_suffix, ".data");
        assert_eq!(got.service_connection_suffix, ".connection");
        assert_eq!(got.service_event_connection_suffix, ".event");

        // A config with a DISTINCT prefix (a run-scoped namespace, e.g. a custom
        // `IOX2_CONFIG_FILE`) yields a DIFFERENT identity — the mismatch netd
        // refuses.
        let mut custom = iceoryx2::config::Config::default();
        custom.global.prefix =
            iceoryx2::prelude::FileName::new(b"cer_other_").expect("valid prefix");
        let custom_json = serde_json::to_string(&custom).expect("serialize custom config");
        let custom_id =
            ix_config_shm_identity_from_json(&custom_json).expect("parse custom identity");
        assert_ne!(custom_id, got, "a distinct prefix is a distinct identity");
        assert_eq!(custom_id.prefix, "cer_other_");
        assert_eq!(
            custom_id.root_path, got.root_path,
            "same root_path, only the prefix differs"
        );
    }

    // The CORE gap: a config sharing
    // (root_path, prefix) with netd but resolving a DIFFERENT `service.directory`
    // discovers DIFFERENT services, so it MUST be a distinct identity (else netd
    // taps its own empty service dir and the run silently egresses nothing). A
    // `(root_path, prefix)`-only identity would report a MATCH here.
    #[test]
    fn service_directory_divergence_is_a_distinct_identity() {
        let default_cfg = iceoryx2::config::Config::default();
        let base = ix_config_shm_identity_from_json(
            &serde_json::to_string(&default_cfg).expect("serialize default"),
        )
        .expect("base identity");

        // Flip ONLY `global.service.directory` in the serialized JSON (Path
        // serializes as a plain string, so the swap re-parses to a valid Config).
        // The sanity assert on the default value ALSO validates the JSON key path:
        // a wrong key would resolve to Null != "services" and fail here loudly.
        let mut v: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&default_cfg).expect("serialize default"))
                .expect("parse default json");
        assert_eq!(
            v["global"]["service"]["directory"], "services",
            "sanity: default service directory (also validates the JSON key path)"
        );
        v["global"]["service"]["directory"] = serde_json::json!("cer_c6b_other_services");
        let flipped = ix_config_shm_identity_from_json(&serde_json::to_string(&v).unwrap())
            .expect("flipped identity");

        assert_ne!(
            flipped, base,
            "a different service.directory is a DIFFERENT discovery identity"
        );
        assert_eq!(
            flipped.service_dir, "cer_c6b_other_services",
            "the flip is reflected in the identity"
        );
        // Root + prefix are IDENTICAL — proving a (root,prefix)-only identity
        // would wrongly match.
        assert_eq!(flipped.root_path, base.root_path);
        assert_eq!(flipped.prefix, base.prefix);
    }

    // The node-discovery directory is likewise
    // discovery-keying — a divergent `node.directory` is a distinct identity.
    #[test]
    fn node_directory_divergence_is_a_distinct_identity() {
        let default_cfg = iceoryx2::config::Config::default();
        let base = ix_config_shm_identity_from_json(
            &serde_json::to_string(&default_cfg).expect("serialize default"),
        )
        .expect("base identity");

        let mut v: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&default_cfg).expect("serialize default"))
                .expect("parse default json");
        assert_eq!(
            v["global"]["node"]["directory"], "nodes",
            "sanity: default node directory (also validates the JSON key path)"
        );
        v["global"]["node"]["directory"] = serde_json::json!("cer_c6b_other_nodes");
        let flipped = ix_config_shm_identity_from_json(&serde_json::to_string(&v).unwrap())
            .expect("flipped identity");

        assert_ne!(
            flipped, base,
            "a different node.directory is a distinct identity"
        );
        assert_eq!(flipped.node_dir, "cer_c6b_other_nodes");
        assert_eq!(flipped.root_path, base.root_path);
        assert_eq!(flipped.prefix, base.prefix);
    }

    // A CRITICAL CONSTRAINT, made executable: the
    // dead-node-cleanup flags must NOT enter the SHM-discovery identity. This is
    // the EXACT netd-vs-run config pair: netd runs `disable_auto_dead_node_cleanup`
    // (all three flags OFF); a forwarded run's `global_config().clone()` snapshot
    // (`mint_deployment_ix_config`) leaves them ON. The two differ ONLY in those
    // flags — which change reap BEHAVIOR, never which SHM files a service resolves
    // to — so the identity MUST be equal. Folding them in would FALSE-mismatch
    // every common-case mp run onto a per-run child (the anti-false-mismatch pin).
    #[test]
    fn cleanup_flag_divergence_does_not_change_shm_identity() {
        // The forwarded run snapshot: default config, cleanup flags ON.
        let run_cfg = iceoryx2::config::Config::default();
        // netd's config: the SAME config with cleanup disabled (the production
        // transform every TransportManager node is built through).
        let netd_cfg = disable_auto_dead_node_cleanup(run_cfg.clone());

        // Anti-tautology: the two configs REALLY differ on the cleanup flags.
        assert!(run_cfg.global.service.cleanup_dead_nodes_on_open);
        assert!(!netd_cfg.global.service.cleanup_dead_nodes_on_open);
        assert!(run_cfg.global.node.cleanup_dead_nodes_on_creation);
        assert!(!netd_cfg.global.node.cleanup_dead_nodes_on_creation);
        assert!(run_cfg.global.node.cleanup_dead_nodes_on_destruction);
        assert!(!netd_cfg.global.node.cleanup_dead_nodes_on_destruction);

        // ...yet the discovery identity is IDENTICAL (struct path).
        assert_eq!(
            IceoryxShmIdentity::from_config(&run_cfg),
            IceoryxShmIdentity::from_config(&netd_cfg),
            "cleanup-flag divergence must NOT change SHM discovery identity"
        );
        // ...and IDENTICAL through the production JSON path (what netd compares):
        // the run forwards its JSON, netd builds its own identity from its node.
        let run_json = serde_json::to_string(&run_cfg).expect("serialize run");
        assert_eq!(
            ix_config_shm_identity_from_json(&run_json).expect("run identity"),
            IceoryxShmIdentity::from_config(&netd_cfg),
            "the netd-vs-run compare MATCHES despite the cleanup-flag divergence"
        );
    }

    // An unparseable / wrong-shape forward is a LOUD `Err`
    // (never a silent accept that would let netd tap on an unverified namespace).
    #[test]
    fn ix_config_shm_identity_from_json_rejects_garbage() {
        assert!(ix_config_shm_identity_from_json("not json").is_err());
        // A wrong-SHAPE object (a string where `global` must be an object) is a
        // type error — an empty `{}` is NOT garbage (it deserializes to a valid
        // all-default Config), so only genuine shape/parse failures are rejected.
        assert!(ix_config_shm_identity_from_json(r#"{"global":"wrong-shape"}"#).is_err());
        let err = ix_config_shm_identity_from_json("nope").expect_err("garbage rejected");
        assert!(
            err.to_string().contains("not a valid Config"),
            "the error names the parse failure: {err}"
        );
    }

    // The per-topic recording tap-queue depth scaling rule. Oracle
    // vectors over synthetic max_slice_len values (never a self-compare) — the
    // exact table documented on `recording_tap_buffer_depth`.
    #[test]
    fn recording_tap_depth_scales_by_slice_with_floor_and_target() {
        // Small / typical control-loop payloads → the full target 4096.
        assert_eq!(recording_tap_buffer_depth(256), RECORDING_TAP_BUFFER_DEPTH);
        assert_eq!(
            recording_tap_buffer_depth(64 * 1024),
            RECORDING_TAP_BUFFER_DEPTH
        );
        // 256 KiB is exactly the budget knee: 1 GiB / 256 KiB == 4096.
        assert_eq!(
            recording_tap_buffer_depth(256 * 1024),
            RECORDING_TAP_BUFFER_DEPTH
        );
        // Just past the knee scales below the target: 1 GiB / 512 KiB == 2048.
        assert_eq!(recording_tap_buffer_depth(512 * 1024), 2048);
        // 1080p-ish camera (2 MiB): 1 GiB / 2 MiB == 512.
        assert_eq!(recording_tap_buffer_depth(2 * 1024 * 1024), 512);
        // 4 MiB: 1 GiB / 4 MiB == 256 (still above the floor).
        assert_eq!(recording_tap_buffer_depth(4 * 1024 * 1024), 256);
        // 16 MiB lands exactly on the floor: 1 GiB / 16 MiB == 64.
        assert_eq!(
            recording_tap_buffer_depth(16 * 1024 * 1024),
            RECORDING_TAP_BUFFER_DEPTH_FLOOR
        );
        // Beyond the floor knee the floor governs (budget deliberately exceeded).
        assert_eq!(
            recording_tap_buffer_depth(64 * 1024 * 1024),
            RECORDING_TAP_BUFFER_DEPTH_FLOOR
        );
        // The tier-3 default (128 MiB) still gets the floor, never below it.
        assert_eq!(
            recording_tap_buffer_depth(crate::graph::config::DEFAULT_MAX_SLICE_LEN),
            RECORDING_TAP_BUFFER_DEPTH_FLOOR
        );
    }

    // The clamp boundary vectors for the 1 GiB
    // budget + the 4096 cap. Oracle values hand-computed from
    // `clamp(1 GiB / msl, 64, 4096)` — never a self-compare.
    #[test]
    fn recording_tap_depth_new_clamp_boundaries() {
        // The 128 MiB tier-3 default STILL floors (1 GiB / 128 MiB == 8 → 64):
        // the whole reason the floored-depth warn exists.
        assert_eq!(
            recording_tap_buffer_depth(crate::graph::config::DEFAULT_MAX_SLICE_LEN),
            RECORDING_TAP_BUFFER_DEPTH_FLOOR
        );
        // A payload-topic default (8 KiB) is deep into the cap: 1 GiB / 8 KiB ==
        // 131072 → clamped to the 4096 target.
        assert_eq!(recording_tap_buffer_depth(8192), RECORDING_TAP_BUFFER_DEPTH);
        // 1 MiB scales to exactly 1024 (the OLD cap, now a mid-range value).
        assert_eq!(recording_tap_buffer_depth(1024 * 1024), 1024);
        // 256 KiB sits EXACTLY on the new cap knee: 1 GiB / 256 KiB == 4096 —
        // the largest slice that still gets the full target (the cap edge).
        assert_eq!(
            recording_tap_buffer_depth(256 * 1024),
            RECORDING_TAP_BUFFER_DEPTH
        );
    }

    #[test]
    fn recording_tap_depth_is_monotonic_non_increasing_in_slice() {
        // A larger slice never yields a DEEPER queue (the SHM-cost guarantee):
        // sweep across the knee region and assert the sequence is non-increasing
        // and always within [FLOOR, TARGET].
        let mut prev = usize::MAX;
        for kib in [1usize, 64, 256, 384, 512, 1024, 2048, 4096, 8192, 65536] {
            let d = recording_tap_buffer_depth(kib * 1024);
            assert!(
                d <= prev,
                "depth must be non-increasing in max_slice_len (slice {kib} KiB)"
            );
            assert!(
                (RECORDING_TAP_BUFFER_DEPTH_FLOOR..=RECORDING_TAP_BUFFER_DEPTH).contains(&d),
                "depth {d} out of [FLOOR, TARGET] for slice {kib} KiB"
            );
            prev = d;
        }
    }

    /// The INGRESS route depth rule — the exact table documented on
    /// [`ingress_route_buffer_depth`], as hand-computed oracle vectors.
    ///
    /// The two rows that carry the issue are the bridge default and the stock
    /// comparison: `dds_bridge`'s `DEFAULT_RAW_MAX_SLICE_LEN` is 1 MiB, so every
    /// `ros2 attach` route lands on the 64 row, and 64 frames is 38.6 ms of
    /// absorption at 1.66 kHz against the stock 16's 9.6 ms.
    #[test]
    fn ingress_route_depth_scales_by_slice_with_floor_and_cap() {
        // (max_slice_len, expected depth) — hand-computed from a 64 MiB budget.
        let oracle: &[(usize, usize)] = &[
            (256, INGRESS_ROUTE_BUFFER_DEPTH),                  // 262144 -> cap
            (4 * 1024, INGRESS_ROUTE_BUFFER_DEPTH),             // 16384  -> cap
            (64 * 1024, INGRESS_ROUTE_BUFFER_DEPTH),            // 1024   -> exactly the cap
            (128 * 1024, 512),                                  // the budget starts to bind
            (256 * 1024, 256),                                  //
            (1024 * 1024, 64),                                  // THE BRIDGE DEFAULT
            (2 * 1024 * 1024, 32),                              //
            (4 * 1024 * 1024, 16),                              // at the floor
            (16 * 1024 * 1024, DEFAULT_SUBSCRIBER_BUFFER_SIZE), // floor governs
            (128 * 1024 * 1024, DEFAULT_SUBSCRIBER_BUFFER_SIZE), // floor governs
        ];
        for (msl, want) in oracle {
            assert_eq!(
                ingress_route_buffer_depth(*msl),
                *want,
                "a {msl}-byte slice must provision {want} frames"
            );
        }

        // The headline consequence, asserted as arithmetic rather than left in
        // a doc comment: at the bridge's own slice size the route must absorb
        // the robot's FULL measured drain gap, on its own.
        //
        // That gap is inverted, not fitted: for a bimodal distribution the loss
        // fraction is `1 - D/(R*G)` — the long-gap FREQUENCY cancels — so the measured
        // 70.9 % at 1.66 kHz on a 16-frame queue gives
        // `G = 16/(1660*0.291)` = 33.1 ms. Sizing against the WHOLE gap rather
        // than against what is left after the idle-tick fix is deliberate: at
        // least 10 ms of it is the tick, but the composition of the rest is not
        // decomposable from off-robot data, so the depth must stand alone.
        const BRIDGE_SLICE: usize = 1024 * 1024;
        const KHZ_RATE: f64 = 1_660.0;
        const GO2_MEASURED_GAP_MS: f64 = 33.1;
        let absorbs_ms = 1_000.0 * ingress_route_buffer_depth(BRIDGE_SLICE) as f64 / KHZ_RATE;
        assert!(
            absorbs_ms > GO2_MEASURED_GAP_MS,
            "a bridge route must absorb the Go2's full measured \
             {GO2_MEASURED_GAP_MS} ms drain gap on its own; got {absorbs_ms:.1} ms"
        );
        assert!(
            1_000.0 * DEFAULT_SUBSCRIBER_BUFFER_SIZE as f64 / KHZ_RATE < 10.0,
            "anti-tautology: the STOCK depth must be under 10 ms at 1.66 kHz, or \
             this test is not describing the bug it was written for"
        );

        // Monotone non-increasing in slice size, and bounded on both sides.
        let mut prev = usize::MAX;
        for shift in 5..28u32 {
            let d = ingress_route_buffer_depth(1usize << shift);
            assert!(d <= prev, "a bigger slice must never buy a deeper queue");
            assert!(d <= INGRESS_ROUTE_BUFFER_DEPTH, "capped");
            assert!(
                d >= DEFAULT_SUBSCRIBER_BUFFER_SIZE,
                "the rule may only ever RAISE a route's depth — never lower it \
                 below the stock ceiling"
            );
            prev = d;
        }

        // Defensive 0-guard: unreachable from a real route (`MaxSliceLen` has a
        // `WireHeader::SIZE` floor), and must not divide by zero.
        assert_eq!(ingress_route_buffer_depth(0), INGRESS_ROUTE_BUFFER_DEPTH);
    }

    /// The SHM budget the ingress rule promises actually holds at the
    /// route count that motivated it.
    ///
    /// Because the rule keeps `depth × slice` at (or under) one budget, the
    /// per-route pool is roughly constant whatever the slice, so the total
    /// scales with ROUTE COUNT alone — which is the quantity a robot varies and
    /// the one the issue asked to be sized against. Pinned here because it is
    /// the claim that makes the depth raise affordable, and a future budget or
    /// cap edit that quietly broke it would otherwise surface only as a robot
    /// that cannot start.
    #[test]
    fn the_ingress_depth_budget_bounds_a_hundred_route_bridge() {
        // iceoryx2 0.9.1's own pool formula, with the defaults
        // `create_ingress_publisher` leaves in place (8 subscribers, 2 borrowed,
        // 0 history, 2 loaned).
        fn pool_bytes(depth: usize, msl: usize) -> u64 {
            (8 * (depth as u64 + 2) + 2) * msl as u64
        }
        const GIB: f64 = (1024 * 1024 * 1024) as f64;
        // Measured on a 15 GiB Jetson: a 54.8 GiB apparent
        // multi-pool reservation faulted in at 0.84 MiB resident. That is the
        // envelope this repo has evidence for, so it is the bar.
        const PROVEN_APPARENT_ENVELOPE_GIB: f64 = 54.8;
        const GO2_ROUTES: u64 = 102;

        let mut raised_rows = 0;
        for msl in [
            256usize,
            4 * 1024,
            64 * 1024,
            256 * 1024,
            1024 * 1024,
            4 * 1024 * 1024,
            16 * 1024 * 1024,
        ] {
            let depth = ingress_route_buffer_depth(msl);
            let total = GO2_ROUTES * pool_bytes(depth, msl);
            let stock = GO2_ROUTES * pool_bytes(DEFAULT_SUBSCRIBER_BUFFER_SIZE, msl);
            if depth == DEFAULT_SUBSCRIBER_BUFFER_SIZE {
                // The rule is INERT at this slice — the budget cannot buy even
                // one extra frame, so the floor governs and the cost is exactly
                // today's. Whether THAT is affordable is a pre-existing question
                // about huge-slice topics (they are inherently low-rate), and
                // asserting an envelope here would be asserting something this
                // change neither causes nor can fix.
                assert_eq!(
                    total, stock,
                    "at a {msl}-byte slice the rule must add nothing at all"
                );
                continue;
            }
            raised_rows += 1;
            assert!(
                total as f64 / GIB <= PROVEN_APPARENT_ENVELOPE_GIB,
                "a {GO2_ROUTES}-route bridge at a {msl}-byte slice would reserve \
                 {:.1} GiB apparent, past the {PROVEN_APPARENT_ENVELOPE_GIB} GiB \
                 measured envelope",
                total as f64 / GIB
            );
        }
        assert!(
            raised_rows >= 5,
            "anti-vacuity: the envelope bound must actually be exercised — only \
             {raised_rows} of the swept slices had their depth raised at all"
        );

        // Anti-tautology: the bound must be tight enough to MEAN something. At
        // the bridge's own slice the reservation must be a substantial fraction
        // of the envelope — a rule that provisioned nothing would pass the
        // assertion above trivially.
        let bridge_total =
            GO2_ROUTES * pool_bytes(ingress_route_buffer_depth(1024 * 1024), 1024 * 1024);
        assert!(
            bridge_total as f64 / GIB > 0.5 * PROVEN_APPARENT_ENVELOPE_GIB,
            "the bridge row should be using most of the budget it is allowed — \
             {:.1} GiB looks like the rule is not raising anything",
            bridge_total as f64 / GIB
        );
    }

    #[test]
    fn recording_tap_depth_zero_slice_is_defensive_target() {
        // max_slice_len is always >= WireHeader::SIZE for a real topic; the
        // 0-guard must return the target (never divide by zero).
        assert_eq!(recording_tap_buffer_depth(0), RECORDING_TAP_BUFFER_DEPTH);
    }

    #[test]
    fn recording_tap_depth_never_lowers_the_stock_ceiling() {
        // The whole point is a DEEPER queue: for every realistic slice the
        // scaled depth must be >= the stock DEFAULT_SUBSCRIBER_BUFFER_SIZE, so
        // the runtime's `.max` raise can never LOWER a topic's buffer.
        for msl in [
            32usize,
            1024,
            256 * 1024,
            2 * 1024 * 1024,
            512 * 1024 * 1024,
        ] {
            assert!(
                recording_tap_buffer_depth(msl) >= DEFAULT_SUBSCRIBER_BUFFER_SIZE,
                "scaled depth for msl {msl} must not be below the stock ceiling"
            );
        }
    }

    #[test]
    fn service_topic_role_provisioning() {
        // The explicit role provisioning (no
        // name-sniffing from the topic string). The service
        // layer (transport::service) calls these directly when creating its
        // request/reply topics — the role is known at the call site, never
        // re-derived from the topic string.

        // Request topic: multi-writer fan-in for N concurrent clients.
        let req = TopicServiceConfig::for_service_request(16);
        assert_eq!(req.max_publishers, Some(MAX_SERVICE_REQUEST_CLIENTS));
        assert_eq!(req.max_subscribers, Some(MAX_SERVICE_REQUEST_CLIENTS));
        assert_eq!(req.publisher_provisioning, PublisherProvisioning::Multi);
        assert_eq!(req.subscriber_max_buffer_size, 16);
        assert_eq!(req.history_size, 0);

        // Reply topic: exactly one server-writer per client.
        let reply = TopicServiceConfig::for_service_reply(16);
        assert_eq!(reply.max_publishers, Some(1));
        assert_eq!(reply.max_subscribers, None);
        assert_eq!(
            reply.publisher_provisioning,
            PublisherProvisioning::SingleWriter
        );
        assert_eq!(reply.history_size, 0);
    }

    #[test]
    fn test_validate_topic_name_ok() {
        assert!(validate_topic_name("camera/image").is_ok());
        assert!(validate_topic_name("a").is_ok());
    }

    #[test]
    fn test_validate_topic_name_empty() {
        assert!(validate_topic_name("").is_err());
    }

    #[test]
    fn test_validate_topic_name_too_long() {
        let long = "x".repeat(250);
        let err = validate_topic_name(&long).unwrap_err();
        assert!(err.contains("250"));
        assert!(err.contains("249"));
    }

    #[test]
    fn env_hint_is_empty_for_requirement_variants() {
        // The Multi-aware requirement hint only fires because
        // `publish_subscribe_open_env_hint`
        // returns "" for the three requirement-mismatch variants (the
        // env branch short-circuits the else). That coupling is
        // implicit — pin it so a future env arm for one of these
        // variants can't silently kill the Multi hint.
        use iceoryx2::service::builder::publish_subscribe::PublishSubscribeOpenError as E;
        for v in [
            E::DoesNotSupportRequestedAmountOfPublishers,
            E::DoesNotSupportRequestedAmountOfSubscribers,
            E::DoesNotSupportRequestedMinBufferSize,
        ] {
            assert_eq!(
                publish_subscribe_open_env_hint(&v),
                "",
                "requirement variant {v:?} must take the requirement-hint \
                 path (else the Multi-aware hint goes dead)"
            );
        }
    }

    /// The buffer-ceiling classification is EXACT.
    ///
    /// [`TransportManager::create_ingress_publisher`]'s fallback catches EVERY
    /// open failure, and its warn names a cause — "an earlier opener created a
    /// shallower service". Only `DoesNotSupportRequestedMinBufferSize` proves
    /// such a service exists. Everything else the fallback also catches
    /// (transient flux, corruption, permissions, a mismatch on a DIFFERENT
    /// requirement axis) says nothing about the buffer ceiling, and blaming an
    /// opener on those sends an operator hunting a process that may not exist.
    ///
    /// `want_open` is an EXHAUSTIVE match with no `_` arm, so an iceoryx2
    /// upgrade that adds a variant fails to COMPILE here instead of silently
    /// defaulting into (or out of) the blaming arm.
    #[test]
    fn only_the_buffer_ceiling_variant_proves_a_shallow_incumbent() {
        use iceoryx2::service::builder::publish_subscribe::{
            PublishSubscribeCreateError as C, PublishSubscribeOpenError as O,
            PublishSubscribeOpenOrCreateError as E,
        };

        fn want_open(v: O) -> bool {
            match v {
                O::DoesNotSupportRequestedMinBufferSize => true,
                // Every other open failure: NOT evidence of a shallow
                // incumbent. Four of these are requirement mismatches on other
                // axes — a service can be too small on subscribers, publishers
                // or borrowed samples while its buffer ceiling is fine.
                O::DoesNotExist
                | O::InternalFailure
                | O::IncompatibleTypes
                | O::IncompatibleMessagingPattern
                | O::IncompatibleAttributes
                | O::DoesNotSupportRequestedMinHistorySize
                | O::DoesNotSupportRequestedMinSubscriberBorrowedSamples
                | O::DoesNotSupportRequestedAmountOfPublishers
                | O::DoesNotSupportRequestedAmountOfSubscribers
                | O::DoesNotSupportRequestedAmountOfNodes
                | O::IncompatibleOverflowBehavior
                | O::InsufficientPermissions
                | O::ServiceInCorruptedState
                | O::HangsInCreation
                | O::ExceedsMaxNumberOfNodes
                | O::IsMarkedForDestruction => false,
            }
        }

        // Every variant of the open enum (kept beside `want_open`: the
        // exhaustive match is what BREAKS on an upstream addition, and the fix
        // is to extend both).
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
        ];
        assert_eq!(
            all_open.len(),
            17,
            "the sweep must cover every open variant — extend BOTH this list \
             and `want_open` when iceoryx2 adds one"
        );
        for v in all_open {
            assert_eq!(
                is_buffer_ceiling_refusal(&E::PublishSubscribeOpenError(v)),
                want_open(v),
                "{v:?} classified wrongly — only the buffer-ceiling refusal may \
                 license blaming an earlier opener"
            );
        }
        // Anti-tautology: the sweep really does contain a TRUE row, so a
        // classifier stuck at `false` cannot pass the loop above.
        assert!(
            all_open.iter().copied().any(want_open),
            "the oracle must classify at least one variant as the refusal"
        );

        // The CREATE side never proves a shallow incumbent (there was nothing
        // to be shallow — it was being created), and neither does the
        // retry-exhausted state, which is the transient case by definition.
        for v in [
            C::ServiceInCorruptedState,
            C::SubscriberBufferMustBeLargerThanHistorySize,
            C::AlreadyExists,
            C::InsufficientPermissions,
            C::InternalFailure,
            C::IsBeingCreatedByAnotherInstance,
            C::HangsInCreation,
        ] {
            assert!(
                !is_buffer_ceiling_refusal(&E::PublishSubscribeCreateError(v)),
                "create-side {v:?} says nothing about an existing service's \
                 buffer ceiling"
            );
        }
        assert!(
            !is_buffer_ceiling_refusal(&E::SystemInFlux),
            "SystemInFlux is the transient case the split exists for — it must \
             never be reported as a shallow incumbent"
        );
    }

    /// The ingress fallback BLAMES an earlier opener only when one is
    /// proven, and the transient arm is cause-neutral.
    ///
    /// This is the transient-arm pin. It is a PURE one by necessity: the raised
    /// and stock legs differ only in `subscriber_max_buffer_size`, so every
    /// non-ceiling failure that the raised open can hit is hit identically by
    /// the stock retry, and no black-box test can drive the fallback into its
    /// transient arm on demand. The e2e half — that a CONFIRMED shallow
    /// incumbent takes the BLAMING arm over real iceoryx2 — is
    /// `crates/cerulion_core/tests/ingress_route_depth_iox2_test.rs` pin 4, so the two
    /// directions are covered between them.
    #[test]
    fn the_ingress_fallback_blames_an_earlier_opener_only_when_one_is_proven() {
        // The two claims the blaming arm makes, and the ONLY two the transient
        // arm must not: an attribution and a directive.
        const BLAME_CLAIM: &str = "the service was created by an earlier opener";
        const BLAME_DIRECTIVE: &str =
            "stop the earlier opener and let the route create the service first.";

        let failure = |buffer_ceiling_refused: bool| OpenServicesFailure {
            error: TransportError::PublisherCreation {
                topic: "/pin".to_string(),
                reason: "whatever iceoryx2 said".to_string(),
            },
            buffer_ceiling_refused,
        };
        let proven = failure(true).ingress_fallback_degrade_message();
        let transient = failure(false).ingress_fallback_degrade_message();

        // The proven arm's wording is pinned — the substrings below
        // span the line-continuation escapes, so a mangled join fails here.
        assert!(
            proven.contains("ingress route attached to a SHALLOWER pre-existing service"),
            "the confirmed arm keeps the wording operators already grep: {proven}"
        );
        assert!(
            proven.contains(BLAME_CLAIM),
            "the confirmed arm names the cause it has evidence for: {proven}"
        );
        assert!(
            proven.contains(BLAME_DIRECTIVE),
            "the confirmed arm keeps the remedy: {proven}"
        );

        // THE transient pin: no attribution, no directive.
        assert!(
            !transient.contains(BLAME_CLAIM),
            "a non-ceiling refusal must not claim an earlier opener created the \
             service — on a transient failure there may be no such opener: {transient}"
        );
        assert!(
            !transient.contains(BLAME_DIRECTIVE),
            "a non-ceiling refusal must not send an operator to stop a process \
             it cannot show exists: {transient}"
        );
        assert!(
            !transient.contains("earlier opener") && !transient.contains("pre-existing"),
            "the transient arm must not gesture at an incumbent at all — not \
             even to deny one, which reads as a hedge on the same claim: {transient}"
        );

        // ...but it must still be USEFUL: it points at the raw error (the
        // `reason` field carries it verbatim on BOTH arms) and it repeats the
        // consequence, which is true whatever refused the raise.
        assert!(
            transient.contains("`reason`"),
            "the transient arm must send the reader to the raw error: {transient}"
        );
        for m in [proven, transient] {
            assert!(
                m.contains("tap queue may overflow under a recorder stall"),
                "both arms report the CONSEQUENCE — that part does not depend \
                 on the cause: {m}"
            );
        }
        assert_ne!(
            proven, transient,
            "anti-tautology: the arms must actually differ, or the split is a \
             no-op and the classification above is decorative"
        );
    }

    #[test]
    fn corrupted_state_hint_names_the_fd_limit() {
        // Verified fd-limit diagnosis: iceoryx2
        // surfaces open-file (fd) EXHAUSTION on a large graph AS
        // `ServiceInCorruptedState`, whose default text said only "corrupted /
        // reboot" — which misdirects the operator (the real fix is `ulimit`).
        // A 1-source/35-data-trigger-consumer graph dies with this exact error
        // at `ulimit -n 1024` and runs clean at `ulimit -n 65536`. Pin that
        // BOTH the pub/sub and event `ServiceInCorruptedState` hints now name
        // the fd remedy, so the misleading-only message can't silently return.
        use iceoryx2::service::builder::event::{
            EventCreateError, EventOpenError, EventOpenOrCreateError,
        };
        use iceoryx2::service::builder::publish_subscribe::PublishSubscribeOpenError as PsE;

        let ps = publish_subscribe_open_env_hint(&PsE::ServiceInCorruptedState);
        assert!(
            ps.contains("ulimit -n 65536") && ps.contains("fd"),
            "pub/sub ServiceInCorruptedState hint must name the fd limit + ulimit remedy: {ps}"
        );

        for ev in [
            EventOpenOrCreateError::EventOpenError(EventOpenError::ServiceInCorruptedState),
            EventOpenOrCreateError::EventCreateError(EventCreateError::ServiceInCorruptedState),
        ] {
            let h = event_env_hint(&ev);
            assert!(
                h.contains("ulimit -n 65536") && h.contains("fd"),
                "event {ev:?} ServiceInCorruptedState hint must name the fd remedy: {h}"
            );
        }
    }

    #[test]
    fn for_topology_maps_publisher_provisioning() {
        // Pins the provisioning mapping directly —
        // `SingleWriter` → `Some(1)` (service-level single-writer) and
        // `External` → `None` (setter uncalled: iceoryx2's create-default
        // applies so an external publisher can attach). The
        // External arm is LIVE (absolute sources) and buffer-ceiling-only, so
        // it carries NO port requirements at all. Pure
        // mapping, no transport. (In-crate literal is fine despite
        // `#[non_exhaustive]`.) Hardening: `default` carries Some
        // values so the assertions PROVE the documented discard contract
        // (the two Option fields are computed from the topology args;
        // `default`'s values are never consulted).
        let default = TopicServiceConfig {
            subscriber_max_buffer_size: 8,
            // Seed value is irrelevant here (for_topology defaults it
            // to None; the runtime raises it in a post-pass) — kept None.
            subscriber_max_borrowed_samples: None,
            // A non-None seed floor must be DISCARDED by for_topology
            // (same discard contract as the port fields).
            create_borrow_floor: Some(66),
            max_subscribers: Some(99),
            max_publishers: Some(99),
            // Deliberately WRONG intent in the seed: for_topology must
            // carry the ARGUMENT's intent, never the seed's (the same
            // discard contract the Some(99) values pin).
            publisher_provisioning: PublisherProvisioning::SingleWriter,
            // A non-zero seed history must be DISCARDED —
            // for_topology carries the ARGUMENT's history verbatim, not the
            // seed's (same discard contract as the port fields above).
            history_size: 77,
            // A non-zero seed extra-listener count must be
            // DISCARDED too — for_topology carries the ARGUMENT verbatim.
            extra_event_listeners: 88,
            // A Some seed must be DISCARDED — for_topology always
            // emits None (the knob is caller-set, never topology-derived).
            publisher_max_loaned_samples: Some(66),
        };
        let sw = TopicServiceConfig::for_topology(
            default,
            16,
            3,
            PublisherProvisioning::SingleWriter,
            5,
            2,
        );
        assert_eq!(sw.max_publishers, Some(1));
        assert_eq!(
            sw.publisher_provisioning,
            PublisherProvisioning::SingleWriter
        );
        // Create-vs-open asymmetry: the FIELD (the open
        // requirement) stays None (no consumer requirement from topology
        // args), while the CREATE-leg value carries the owned HOLD floor, so
        // a producer-first CREATION cannot bake in a too-small borrow ceiling
        // for a cross-graph HOLD consumer, yet an owned OPEN never refuses a
        // pre-existing sub-floor service (degraded-attach contract).
        assert_eq!(sw.subscriber_max_borrowed_samples, None);
        assert_eq!(
            sw.create_borrowed_samples(),
            Some(SUBSCRIBER_MAX_BORROWED_HELD)
        );
        // The seed's create-leg floor (Some(66)) is DISCARDED — the
        // rmw create paths set it per-topic on their own mutated copy; a
        // topology-derived config never inherits one from the seed.
        assert_eq!(sw.create_borrow_floor, None);
        // History is the ARGUMENT (5), never the seed (77).
        assert_eq!(sw.history_size, 5);
        // Extra listeners is the ARGUMENT (2), never the
        // seed (88).
        assert_eq!(sw.extra_event_listeners, 2);
        // The loan knob is caller-set, never topology-derived —
        // for_topology emits None even from a Some(66) seed.
        assert_eq!(sw.publisher_max_loaned_samples, None);
        let ext =
            TopicServiceConfig::for_topology(default, 16, 3, PublisherProvisioning::External, 0, 0);
        assert_eq!(ext.max_publishers, None);
        assert_eq!(ext.max_subscribers, None);
        assert_eq!(ext.history_size, 0);
        assert_eq!(ext.extra_event_listeners, 0);
        // Same discard contract on the External arm.
        assert_eq!(ext.publisher_max_loaned_samples, None);
        // The intent rides along verbatim (the
        // single-writer pre-check reads THIS field, not Some(1)).
        assert_eq!(ext.publisher_provisioning, PublisherProvisioning::External);
        // `External` imposes NO borrow requirement AND no create
        // floor — the graph only OPENS a foreign service; in the
        // no-foreign-creator race its create value passes the field through
        // untouched (a snapshot read of an External source is raised
        // separately by the runtime's post-pass, which sets the FIELD).
        assert_eq!(ext.subscriber_max_borrowed_samples, None);
        assert_eq!(ext.create_borrowed_samples(), None);
        // The Multi arm provisions BOTH port caps as
        // graph-independent shared constants (cross-graph opens pass by
        // equality — a graph-dependent subscriber term would lock out a
        // second graph with more consumers than the creator); the intent
        // rides along (the single-writer pre-check must NOT fire for
        // Multi).
        let multi =
            TopicServiceConfig::for_topology(default, 16, 3, PublisherProvisioning::Multi, 4, 0);
        assert_eq!(multi.max_publishers, Some(MULTI_PUBLISHER_LOOSE_MAX));
        assert_eq!(multi.history_size, 4);
        assert_eq!(multi.max_subscribers, Some(MULTI_SUBSCRIBER_LOOSE_MAX));
        assert_eq!(multi.publisher_provisioning, PublisherProvisioning::Multi);
        assert_eq!(multi.subscriber_max_buffer_size, 16);
        // An OWNED Multi (cross-graph) topic gets the create-side
        // HOLD floor too — every Cerulion graph's Multi arm computes the same
        // create value, and the OPEN requirement stays None (a graph opening
        // a listed topic created by a peer graph imposes no borrow term).
        assert_eq!(multi.subscriber_max_borrowed_samples, None);
        assert_eq!(
            multi.create_borrowed_samples(),
            Some(SUBSCRIBER_MAX_BORROWED_HELD)
        );
        // The owned arm follows the documented contracts (buffer = max(
        // default, depth); subscribers = in-graph + headroom); the buffer
        // ceiling — the consumer requirement — is provisioning-agnostic.
        assert_eq!(sw.subscriber_max_buffer_size, 16);
        assert_eq!(
            sw.max_subscribers,
            Some(3 + INTROSPECTION_SUBSCRIBER_HEADROOM)
        );
        assert_eq!(
            sw.subscriber_max_buffer_size,
            ext.subscriber_max_buffer_size
        );
    }

    /// `create_borrowed_samples` — the create-leg borrow value —
    /// composes with a GENUINE field requirement via `max` and never touches
    /// External configs. Hand oracle per arm.
    #[test]
    fn create_borrowed_samples_floors_owned_and_passes_external_through() {
        let default = default_seed_cfg();
        let mut owned = TopicServiceConfig::for_topology(
            default,
            4,
            1,
            PublisherProvisioning::SingleWriter,
            0,
            0,
        );
        // No genuine requirement: create at exactly the floor.
        assert_eq!(owned.subscriber_max_borrowed_samples, None);
        assert_eq!(
            owned.create_borrowed_samples(),
            Some(SUBSCRIBER_MAX_BORROWED_HELD)
        );
        // Genuine requirement AT the floor (snapshot post-pass): the
        // create value equals the open requirement — no two-phase asymmetry.
        owned.subscriber_max_borrowed_samples = Some(SUBSCRIBER_MAX_BORROWED_HELD);
        assert_eq!(
            owned.create_borrowed_samples(),
            Some(SUBSCRIBER_MAX_BORROWED_HELD)
        );
        // Genuine requirement ABOVE the floor (recording raise): the
        // requirement wins the max — the floor never LOWERS a create value.
        owned.subscriber_max_borrowed_samples = Some(RECORDING_SUBSCRIBER_MAX_BORROWED);
        assert_eq!(
            owned.create_borrowed_samples(),
            Some(RECORDING_SUBSCRIBER_MAX_BORROWED)
        );
        // External: pure passthrough in BOTH states — the graph must create
        // (in the no-foreign-creator race) exactly what it requires, so the
        // loud refusal arm for external snapshot sources is preserved and an
        // unrequired external open imposes/creates nothing extra.
        let mut ext =
            TopicServiceConfig::for_topology(default, 4, 1, PublisherProvisioning::External, 0, 0);
        assert_eq!(ext.create_borrowed_samples(), None);
        ext.subscriber_max_borrowed_samples = Some(SUBSCRIBER_MAX_BORROWED_HELD);
        assert_eq!(
            ext.create_borrowed_samples(),
            Some(SUBSCRIBER_MAX_BORROWED_HELD)
        );
    }

    /// `create_borrow_floor` raises ONLY the create leg — the open
    /// requirement (the pub field) is untouched in every arm, so a floored
    /// rmw create still ATTACHES to a pre-existing smaller service (the
    /// degraded-attach contract). Hand oracle per arm.
    #[test]
    fn create_borrow_floor_raises_the_create_leg_only() {
        // The rmw shape: a default (External) config + a floor of 4.
        let mut cfg = default_seed_cfg();
        cfg.publisher_provisioning = PublisherProvisioning::External;
        cfg.create_borrow_floor = Some(4);
        assert_eq!(
            cfg.subscriber_max_borrowed_samples, None,
            "the floor must never arm an open requirement"
        );
        assert_eq!(
            cfg.create_borrowed_samples(),
            Some(4),
            "an rmw-created service is born loan-ready at the floor"
        );

        // A floor at/below the iceoryx2 default is a NO-OP (None, not
        // Some(default)) so the two-phase split is not entered for a
        // create that would be byte-identical to the single-call path. The
        // ONE definition of that rule is `effective_create_borrow_floor`,
        // pinned directly on both sides of its threshold.
        cfg.create_borrow_floor = Some(ICEORYX2_DEFAULT_BORROWED_SAMPLES);
        assert_eq!(cfg.effective_create_borrow_floor(), None);
        assert_eq!(cfg.create_borrowed_samples(), None);
        cfg.create_borrow_floor = Some(0);
        assert_eq!(cfg.effective_create_borrow_floor(), None);
        assert_eq!(cfg.create_borrowed_samples(), None);
        cfg.create_borrow_floor = Some(ICEORYX2_DEFAULT_BORROWED_SAMPLES + 1);
        assert_eq!(
            cfg.effective_create_borrow_floor(),
            Some(ICEORYX2_DEFAULT_BORROWED_SAMPLES + 1)
        );

        // The floor composes with a GENUINE requirement via max — a
        // snapshot requirement (3) under a floor of 4 creates at 4; a
        // recording raise (22) above the floor wins.
        cfg.create_borrow_floor = Some(4);
        cfg.subscriber_max_borrowed_samples = Some(SUBSCRIBER_MAX_BORROWED_HELD);
        assert_eq!(cfg.create_borrowed_samples(), Some(4));
        cfg.subscriber_max_borrowed_samples = Some(RECORDING_SUBSCRIBER_MAX_BORROWED);
        assert_eq!(
            cfg.create_borrowed_samples(),
            Some(RECORDING_SUBSCRIBER_MAX_BORROWED)
        );

        // Owned (SingleWriter) arm: the floor composes with the HOLD
        // floor the same way (max) — 4 > 3 wins.
        let mut owned = default_seed_cfg();
        owned.subscriber_max_borrowed_samples = None;
        owned.create_borrow_floor = Some(4);
        assert_eq!(
            owned.publisher_provisioning,
            PublisherProvisioning::SingleWriter
        );
        assert_eq!(owned.create_borrowed_samples(), Some(4));
    }

    // ===================================================================
    // TopicRequirements harvest (`from_service_config`) + union
    // merge (`union_into`) — hermetic oracle-vector tests (no transport).
    // ===================================================================

    /// The production default `TopicServiceConfig` seed (mirrors
    /// [`TransportManager::default_topic_config`] — buffer
    /// [`DEFAULT_SUBSCRIBER_BUFFER_SIZE`], every port knob unset). In-crate
    /// literal is fine despite `#[non_exhaustive]`.
    fn default_seed_cfg() -> TopicServiceConfig {
        TopicServiceConfig {
            subscriber_max_buffer_size: DEFAULT_SUBSCRIBER_BUFFER_SIZE,
            subscriber_max_borrowed_samples: None,
            create_borrow_floor: None,
            max_subscribers: None,
            max_publishers: None,
            publisher_provisioning: PublisherProvisioning::SingleWriter,
            history_size: 0,
            extra_event_listeners: 0,
            publisher_max_loaned_samples: None,
        }
    }

    /// A finalized `TopicServiceConfig` in the shape the monolith would produce
    /// for an OWNED SingleWriter snapshot-source topic: `for_topology` output
    /// with the borrow-3 post-pass applied.
    fn owned_snapshot_cfg() -> TopicServiceConfig {
        let default = default_seed_cfg();
        let mut cfg = TopicServiceConfig::for_topology(
            default,
            /* max_consumer_depth */ 10,
            /* in_graph_subscribers */ 2,
            PublisherProvisioning::SingleWriter,
            /* history */ 0,
            /* extra_event_listeners */ 1,
        );
        // Post-pass: a snapshot consumer raises the borrow ceiling.
        cfg.subscriber_max_borrowed_samples = Some(SUBSCRIBER_MAX_BORROWED_HELD);
        cfg
    }

    /// `from_service_config` folds the two `Option` fields to concrete counts
    /// (the iceoryx2 default for borrow, `0` for `max_subscribers`) and copies
    /// buffer + event-listener verbatim — hand oracle.
    #[test]
    fn topic_requirements_from_service_config_distills_all_axes() {
        let cfg = owned_snapshot_cfg();
        let req = TopicRequirements::from_service_config(&cfg);
        // borrow-3 (snapshot) survives the fold.
        assert_eq!(req.min_borrowed_samples, SUBSCRIBER_MAX_BORROWED_HELD);
        // buffer = max(depth 10, default 16) = 16.
        assert_eq!(req.min_buffer, cfg.subscriber_max_buffer_size);
        assert_eq!(req.min_buffer, 16);
        // subscribers = in_graph(2) + INTROSPECTION_SUBSCRIBER_HEADROOM.
        assert_eq!(req.min_subscribers, 2 + INTROSPECTION_SUBSCRIBER_HEADROOM);
        assert_eq!(req.min_event_listeners, 1);

        // A topic with the iceoryx2-default borrow (None) folds to 2, and an
        // unset max_subscribers (None) folds to 0.
        let mut plain = cfg;
        plain.subscriber_max_borrowed_samples = None;
        plain.max_subscribers = None;
        let plain_req = TopicRequirements::from_service_config(&plain);
        assert_eq!(
            plain_req.min_borrowed_samples,
            ICEORYX2_DEFAULT_BORROWED_SAMPLES
        );
        assert_eq!(plain_req.min_subscribers, 0);
    }

    /// `union_into` raises a LOCAL (subgraph-view) config to `max(local,
    /// stamped)` on every axis (the cross-group provisioning fix). Oracle: a producer-owning
    /// worker whose LOCAL view has NO snapshot consumer (borrow None→2, fewer
    /// subscribers, shallower buffer) is lifted to the cross-group union.
    /// (Note: the stamped `Some(3)` becomes a GENUINE armed open
    /// requirement on the worker — correct, it stands in for a real
    /// cross-group HOLD consumer; the create-side floor is separate and never
    /// enters this union.)
    #[test]
    fn topic_requirements_union_into_raises_local_to_max() {
        // LOCAL view: this worker sees NO snapshot consumer + only 1 in-graph sub.
        let default = default_seed_cfg();
        let mut local = TopicServiceConfig::for_topology(
            default,
            /* depth */ 4,
            /* in_graph_subscribers */ 1,
            PublisherProvisioning::SingleWriter,
            0,
            0,
        );
        // Local: borrow default (None), max_subscribers Some(1+4=5), buffer
        // max(4,16)=16, event listeners 0.
        assert_eq!(local.subscriber_max_borrowed_samples, None);
        assert_eq!(
            local.max_subscribers,
            Some(1 + INTROSPECTION_SUBSCRIBER_HEADROOM)
        );

        // STAMPED (full-graph) union: a cross-group snapshot consumer + more subs
        // + a deeper buffer + a cross-group unified listener.
        let stamp = TopicRequirements {
            min_borrowed_samples: SUBSCRIBER_MAX_BORROWED_HELD, // 3
            min_buffer: 32,
            min_subscribers: 9,
            min_event_listeners: 2,
        };
        stamp.union_into(&mut local);

        // Every axis lifted to the union (hand oracle).
        assert_eq!(
            local.subscriber_max_borrowed_samples,
            Some(SUBSCRIBER_MAX_BORROWED_HELD),
            "cross-group snapshot consumer must raise borrow to 3 (the reported bug)"
        );
        assert_eq!(local.subscriber_max_buffer_size, 32);
        assert_eq!(local.max_subscribers, Some(9));
        assert_eq!(local.extra_event_listeners, 2);
    }

    /// `union_into` is monotone: a stamp WEAKER than the local view never lowers
    /// provisioning (the worker's own requirement is always honored).
    #[test]
    fn topic_requirements_union_into_never_lowers() {
        let default = default_seed_cfg();
        // LOCAL view already strong: a snapshot consumer (borrow 3) + 5 subs.
        let mut local = TopicServiceConfig::for_topology(
            default,
            /* depth */ 40,
            /* in_graph_subscribers */ 5,
            PublisherProvisioning::SingleWriter,
            0,
            3,
        );
        local.subscriber_max_borrowed_samples = Some(SUBSCRIBER_MAX_BORROWED_HELD);
        let before = local;

        // A weaker stamp (everything below local) must be a no-op.
        let weak = TopicRequirements {
            min_borrowed_samples: ICEORYX2_DEFAULT_BORROWED_SAMPLES, // 2 < 3
            min_buffer: 8,
            min_subscribers: 2,
            min_event_listeners: 0,
        };
        weak.union_into(&mut local);
        assert_eq!(
            local, before,
            "a weaker stamp must never lower provisioning"
        );
    }

    /// `union_into` with a default-borrow stamp keeps `None` (never arms a
    /// redundant `Some(2)` open-requirement equal to the iceoryx2 default), and
    /// the `min_subscribers == 0` sentinel never DROPS an owned topic's port cap.
    #[test]
    fn topic_requirements_union_into_preserves_none_and_guards_zero() {
        let default = default_seed_cfg();
        let mut local = TopicServiceConfig::for_topology(
            default,
            4,
            1,
            PublisherProvisioning::SingleWriter,
            0,
            0,
        );
        let local_subs = local.max_subscribers;
        assert!(local_subs.is_some());
        // Stamp with a default-2 borrow + a 0-subscriber sentinel (a harvest that
        // saw the iceoryx2 default) must leave borrow None and NOT overwrite the
        // owned topic's Some(..) subscriber cap with Some(0).
        let stamp = TopicRequirements {
            min_borrowed_samples: ICEORYX2_DEFAULT_BORROWED_SAMPLES,
            min_buffer: 0,
            min_subscribers: 0,
            min_event_listeners: 0,
        };
        stamp.union_into(&mut local);
        assert_eq!(
            local.subscriber_max_borrowed_samples, None,
            "a default-2 stamp must not arm a redundant Some(2) open-requirement"
        );
        assert_eq!(
            local.max_subscribers, local_subs,
            "a 0-subscriber stamp must never drop an owned topic's Some(..) port cap"
        );
    }
}
