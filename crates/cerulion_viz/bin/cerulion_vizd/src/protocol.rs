// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion-vizd` control protocol — THE sacred open contract.
//!
//! `cerulion-vizd` speaks **newline-delimited JSON (NDJSON)** over a Unix domain
//! socket: one JSON object per line, request→response, correlated by an `id`.
//! MULTIPLE controllers drive ONE daemon concurrently (Cerulion Studio + the
//! Cerulion agent + the `cerulion viz` verb — the explicit rationale
//! being that the agent runs CLI commands to display things in the viewer rather
//! than automating a GUI), so this contract is the boundary Studio (closed-source)
//! codes against and is versioned + strictly forward-compatible.
//!
//! # Framing
//!
//! On CONNECT the daemon sends a [`Hello`] banner line FIRST (announcing the
//! [`PROTOCOL_VERSION`] so a controller can detect skew), then it is
//! request→response: the controller sends one [`Request`] object per line and
//! reads one [`Response`] object per line back, correlated by the request's `id`.
//!
//! # Forward compatibility
//!
//! Unknown request FIELDS are IGNORED (Studio may add fields the running daemon
//! predates). An unknown `method` OR a malformed line is answered with a
//! structured [`Response::error`] — NEVER a panic or a dropped connection (see
//! [`parse_request`]).
//!
//! ```jsonc
//! // banner (daemon → controller, once per connection)
//! {"vizd":"cerulion-vizd","protocol":1}
//! // requests (controller → daemon, one object per line)
//! {"id":1,"method":"discover"}
//! {"id":2,"method":"attach","topic":"/utlidar/cloud"}   // optional: "entity", "robot"
//! {"id":3,"method":"detach","topic":"/utlidar/cloud"}
//! {"id":4,"method":"list"}
//! {"id":5,"method":"status"}
//! {"id":6,"method":"representation","topic":"/imu","representation":"both"}  // auto|visual|text|both
//! // responses (daemon → controller)
//! {"id":2,"ok":true,"topic":"/utlidar/cloud","schema":"sensor_msgs/PointCloud2","archetype":"Points3D","entity":"world/utlidar/cloud","route":"utlidar/cloud","already_attached":false}
//! {"id":2,"ok":false,"error":"...","topic":"/x"}
//! ```

use cerulion_core::{LivenessState, TopicLiveness};
use serde::{Deserialize, Serialize};

// Monitors: the monitor row + alert are the ENGINE's own types, re-exported
// rather than re-declared. They are the vocabulary a controller matches on, and
// two copies of one wire vocabulary is precisely how one surface starts
// disagreeing with another. `cerulion_core::LivenessState` above
// crosses this same boundary the same way.
pub use cerulion_viz::monitor::{
    Alert, Ineligible, IneligibleReason, MonitorCondition, MonitorRow, MonitorState,
};

// The `runs` verb's run-shaped vocabulary is `cerulion_core`'s own,
// re-exported for the SAME reason the monitor types above are — a second copy of a
// wire vocabulary is how one surface starts disagreeing with another.
//
// Three of these arrive on this wire having crossed the LAN already (the gateway's
// `cerulion_q` payload), so re-declaring them would mean maintaining a desk-side
// mirror of a robot-side truth and keeping the two in step by hand:
//
// * [`RunsCompleteness`] is THE reason an empty `runs` list is readable at all, and
//   its `Settled`/`Incomplete{live_writers,writers_heard}` shape is a judgement the
//   robot made about its own gather. A mirror could drift into defaulting `Settled`,
//   which is the false-absence class exactly.
// * [`UndescribableRun`] and [`UnusableRunsAnswer`] each carry a REMEDY a person
//   acts on (go and look at that run's directory / redeploy that robot), and the
//   two are deliberately DIFFERENT types because those remedies differ.
// * [`RunEntryState`] is the run's own `live`/`ending` word.
pub use cerulion_core::{RunEntryState, RunsCompleteness, UndescribableRun, UnusableRunsAnswer};

/// The control-protocol version announced in the [`Hello`] banner. Bumped on a
/// BREAKING change to the request/response shapes; a controller compares it
/// against the version it was built for and can refuse / adapt on skew. Additive
/// fields do NOT bump it (unknown fields are ignored — forward-compatible).
///
/// **Why a new VERB did not bump this.** The `subscribe_events` verb and
/// the server-initiated [`CatalogChangedEvent`] are additive in the strict sense: a
/// controller that does not send the verb never receives an unsolicited line, so its
/// byte stream is IDENTICAL to an older daemon's. Bumping would therefore buy nothing
/// and cost a great deal: Studio's shell compares this number for EXACT EQUALITY and
/// refuses to drive a daemon that differs (its `check_banner` → `BannerVerdict::
/// Mismatch` → "refusing to drive it"), so a bump would take every shipping Studio
/// offline against a new vizd until the shell was updated in lockstep.
///
/// Capability negotiation for the new verb is therefore BY VERB, which is what the
/// contract already provides: an old daemon answers `subscribe_events` with the
/// documented structured unknown-method [`Response::error`], and the controller
/// degrades to the refresh path it already has. (netd's seam bumps ITS version for the
/// mirror-image reason — its client gates per verb and nothing compares its banner for
/// equality.)
pub const PROTOCOL_VERSION: u32 = 1;

/// The connect banner the daemon sends as the FIRST line of every accepted
/// connection, before any request is read. Carries the daemon name + the
/// [`PROTOCOL_VERSION`] so a controller detects a version skew immediately, plus
/// the `rerun_url` the daemon logs its viz to.
///
/// `rerun_url` is the ONE endpoint every viewer (a native `rerun` app, OR
/// Cerulion Studio's embedded WASM viewer) connects to: by default the daemon
/// HOSTS a gRPC message-proxy on loopback and advertises the bound
/// `rerun+http://127.0.0.1:{port}/proxy` here — so a controller can point a
/// viewer at it with zero extra config (the checkbox→scene bar).
/// `null` means viz is disabled (no endpoint hosted / configured), a stable
/// shape (present-but-null, never omitted) so Studio always parses the field.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Hello {
    /// The daemon identity (`"cerulion-vizd"`).
    pub vizd: String,
    /// The [`PROTOCOL_VERSION`] this daemon speaks.
    pub protocol: u32,
    /// The Rerun gRPC endpoint the daemon logs to + every viewer connects to
    /// (`rerun+http://127.0.0.1:{port}/proxy` in the hosted default), or `null`
    /// when viz is disabled. Always present (never omitted) — the stable Studio
    /// contract.
    pub rerun_url: Option<String>,
}

impl Hello {
    /// The banner for THIS build, advertising the daemon's live `rerun_url`
    /// (`None` when viz is disabled).
    pub fn new(rerun_url: Option<String>) -> Self {
        Self {
            vizd: "cerulion-vizd".to_string(),
            protocol: PROTOCOL_VERSION,
            rerun_url,
        }
    }

    /// Serialize to the single NDJSON line (no trailing newline; the writer adds
    /// it).
    pub fn to_json_line(&self) -> String {
        serde_json::to_string(self).expect("Hello serializes")
    }
}

/// A rerun-AGNOSTIC viewport layout spec — the payload of the `set_blueprint`
/// verb (the layout verb). Studio / the Cerulion agent builds this to arrange
/// the topics it has attached into a dashboard; the daemon validates it and
/// translates it to a `rerun` blueprint. Deliberately rerun-FREE (this whole
/// module is liftable without the rerun SDK): the agent never needs to know
/// rerun's API — it names view/container KINDS by string and entity-path origins.
///
/// The tree is a [`LayoutNode`] root (a container of children, or a single view)
/// plus [`auto_views`](Self::auto_views). Unknown sibling fields are ignored
/// (forward-compatible); an unknown view/container `kind` string is a loud,
/// actionable error from the daemon's validation (never a silent drop).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LayoutSpec {
    /// The root node — a container tree, or a single view (rerun wraps a lone
    /// view in a tab).
    pub root: LayoutNode,
    /// Whether the viewer may additionally auto-create views for archetypes the
    /// explicit views do not cover. Defaults to `true` (matching the Go2 default
    /// dashboard) when omitted.
    #[serde(default = "default_auto_views")]
    pub auto_views: bool,
}

/// The `auto_views` default (`true`) — omitting the field keeps the Go2 default's
/// "nothing logged is invisible" behavior.
fn default_auto_views() -> bool {
    true
}

/// A node in a [`LayoutSpec`] tree: a `container` that arranges children, or a
/// leaf `view`. INTERNALLY TAGGED on `type` (`{"type":"container",…}` /
/// `{"type":"view",…}`) so container-vs-view is unambiguous and a view can never
/// carry `children` (invalid states are unrepresentable); the concrete
/// container/view kind is the inner `kind` string (validated by the daemon so an
/// unknown kind names the supported set — never a silent drop).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum LayoutNode {
    /// A container arranging child nodes (a split, grid, or tab strip).
    Container(ContainerSpec),
    /// A leaf view rendering an entity subtree.
    View(ViewSpec),
}

/// A CONTAINER node: its kind + its children + optional sizing knobs. The daemon
/// validates `kind` against the supported set and the sizing knobs against the
/// kind (e.g. `columns` is grid-only, `shares` must match the child count).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContainerSpec {
    /// The container kind: `"horizontal"` | `"vertical"` | `"grid"` | `"tabs"`.
    pub kind: String,
    /// The child nodes (views and/or nested containers), in layout order.
    pub children: Vec<LayoutNode>,
    /// Optional display name for the container tab/header.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Optional relative sizes along the container's sized axis, each a finite
    /// positive value. Counted PER KIND: `horizontal` (column) and `vertical`
    /// (row) take one share per CHILD; a `grid`'s shares are one per COLUMN (so a
    /// grid with `shares` must also set `columns`); `tabs` take none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shares: Option<Vec<f32>>,
    /// Optional grid column count (`grid` only; must be `1..=children.len()`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub columns: Option<u32>,
}

/// A leaf VIEW node: its kind + optional name + optional entity-path origin. The
/// daemon validates `kind` against the supported set.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ViewSpec {
    /// The view kind: `"spatial3d"` | `"spatial2d"` | `"time_series"` |
    /// `"text_document"`.
    pub kind: String,
    /// Optional display name (defaults to the kind's title).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Optional entity-path origin the view is rooted at (e.g. `"world"`,
    /// `"world/utlidar/cloud"`). Defaults to the world root when omitted. This
    /// is where the layout composes with `attach`: use the entity path an
    /// `attach` response reported so the view shows that topic. A
    /// topic's entity is `world/<its topic path>`, so it is per-topic unique —
    /// but STILL report-then-use, never a path you construct yourself.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<String>,
}

/// A SEMANTIC layout INTENT — the payload of the `compose_layout`
/// verb. The agent expresses intent in TOPIC terms and vizd COMPILES a correct
/// blueprint (it never names an entity path or a rerun view kind). EITHER
/// role-tagged [`groups`](Self::groups) OR bare [`topics`](Self::topics) for full
/// automagic (the daemon rejects both-set / neither-set); `strategy` picks the
/// arrangement; `attach_missing` attaches any listed topic not yet tapped;
/// `include_robot` rides the robot model / tf skeleton in the hero 3D view.
/// rerun-FREE (this module stays liftable without the rerun SDK).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LayoutIntent {
    /// Role-tagged groups of topics (mutually exclusive with [`topics`](Self::topics)).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub groups: Option<Vec<LayoutGroup>>,
    /// A bare topic list for full automagic (each topic lands in every view its
    /// archetype renders in). Mutually exclusive with [`groups`](Self::groups).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub topics: Option<Vec<String>>,
    /// The arrangement strategy: `"auto"` (default) | `"focus_3d"` | `"grid"`.
    #[serde(default = "default_strategy")]
    pub strategy: String,
    /// Attach any listed topic not yet tapped (default `true`), via the existing
    /// attach path incl. its guardrails.
    #[serde(default = "default_true")]
    pub attach_missing: bool,
    /// Ride the robot model / tf skeleton in the hero 3D view (default `true`).
    #[serde(default = "default_true")]
    pub include_robot: bool,
}

/// One GROUP in a [`LayoutIntent`]: topics + an OPTIONAL coarse `role` hint
/// (`"hero"` | `"plots"` | `"images"` | `"status"`) + an optional display `title`.
/// A topic whose resolved view kinds are INCOMPATIBLE with the group's role is a
/// loud daemon refusal (never a silent drop).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LayoutGroup {
    /// The absolute topics in this group.
    pub topics: Vec<String>,
    /// The coarse role hint (omitted = per-topic automagic within the group).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub role: Option<String>,
    /// The display name for the group's view (honored only on a role group).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub title: Option<String>,
}

/// The `compose_layout` strategy default (`"auto"`).
fn default_strategy() -> String {
    "auto".to_string()
}

/// The `true` default for `attach_missing` / `include_robot`.
fn default_true() -> bool {
    true
}

/// A control request. INTERNALLY TAGGED on `method` (`{"method":"attach",…}`),
/// so an unknown `method` fails to parse and is answered with a structured error
/// (never a panic). Unknown sibling fields are IGNORED (forward-compatible — no
/// `deny_unknown_fields`).
///
/// Not `Eq`: [`Request::SetBlueprint`] carries an [`f32`]-bearing [`LayoutSpec`]
/// (`shares`), which is `PartialEq` but not `Eq`. Nothing keys a `Request`, so
/// `PartialEq` (round-trip assertions) is all that is needed.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "method", rename_all = "snake_case")]
pub enum Request {
    /// List every attachable topic (local iceoryx2 topics), each with its
    /// best-effort resolved schema + archetype.
    Discover {
        /// Correlation id echoed in the response.
        id: u64,
    },
    /// Open a runtime tap on `topic` (idempotent — a re-attach opens no second
    /// tap). `entity` overrides the derived render entity. `robot` selects the
    /// REMOTE arm: the daemon declares gateway ingress for the
    /// topic (re-injecting the remote robot's frames into desk-local SHM) and
    /// then taps the now-local mirror by the identical code path — so `schema`
    /// (the topic's ROS type, `pkg/Type`) is REQUIRED with `robot`, since a
    /// data-only tap on a not-yet-ingressed remote topic has no frame to resolve
    /// a schema hash from. For a LOCAL attach (no `robot`) `schema` is optional
    /// and ignored (the schema back-fills from the first frame).
    Attach {
        /// Correlation id echoed in the response.
        id: u64,
        /// The absolute topic to tap (e.g. `/utlidar/cloud`).
        topic: String,
        /// Optional render-entity override (Studio/agent presentation control) —
        /// an ENTITY PATH, e.g. `"world/cam"`. Rarely needed: the
        /// entity is `world/<topic path>` and therefore already unique per topic,
        /// and WHERE a topic appears on screen is a layout concern
        /// (`set_blueprint` / `compose_layout`), not an entity-path one. The
        /// override exists to place a topic somewhere specific in the entity tree.
        /// A leading `world/` is optional (`"world/cam"` and `"cam"` both mean the
        /// entity `world/cam`), so an entity this daemon REPORTED can be fed back
        /// verbatim and resolves to itself — for EVERY topic, including `/tf` and
        /// `/tf_static`, whose reported entity is the bare viz root `world`: an
        /// override naming the root is treated as ABSENT (nothing may claim the
        /// scene root, whose transform composes onto every entity), so the topic
        /// keeps its own answer instead of landing somewhere else.
        ///
        /// The override names an ENTITY and nothing else. It cannot change how a
        /// topic is INTERPRETED: `entity: "world/tf_static"` on an unrelated topic
        /// does not put it on the static-TF arm, and `entity: "world/odom"` does
        /// not make it pose the robot root — those knobs read the TOPIC.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        entity: Option<String>,
        /// The remote robot whose topic to ingress + tap. When
        /// set, the daemon declares gateway ingress for `topic` then taps the
        /// re-injected local mirror. `None` = a purely-local attach (the network
        /// is never touched).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        robot: Option<String>,
        /// The topic's ROS type qualified name (`pkg/Type`) — REQUIRED for a
        /// remote attach (`robot` set): the daemon resolves it to the wire
        /// `schema_hash` that gateway ingress validates against. Ignored for a
        /// local attach (the schema back-fills from the first drained frame).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        schema: Option<String>,
    },
    /// Drop the tap on `topic` (a no-op if it was not tapped).
    Detach {
        /// Correlation id echoed in the response.
        id: u64,
        /// The absolute topic to stop tapping.
        topic: String,
    },
    /// Choose how one topic RENDERS — `auto` (the archetype ladder's
    /// own choice, the default) / `visual` / `text` / `both`.
    ///
    /// A NEW VERB rather than a field on `attach`, for two reasons. The gesture
    /// happens on a topic the operator is already watching — the requirement is
    /// literally "I am looking at this plot and I want the message too" — so
    /// requiring a detach + re-attach to change it would make the affordance cost
    /// a gap in the data it is about. And the choice is remembered per TOPIC for
    /// the daemon's life, so it can be set on a topic that is not attached at
    /// all and applies when it is.
    ///
    /// Old daemons answer the unknown method with the structured error, which is
    /// exactly the capability negotiation `PROTOCOL_VERSION`'s doc prescribes —
    /// hence no version bump.
    Representation {
        /// Correlation id echoed in the response.
        id: u64,
        /// The absolute topic whose representation is being set.
        topic: String,
        /// One of `auto` / `visual` / `text` / `both`. Anything else is REFUSED
        /// with a structured error naming the accepted set — never silently read
        /// as `auto`, which would report success for a click that did nothing.
        representation: String,
    },
    /// List the currently-attached taps.
    List {
        /// Correlation id echoed in the response.
        id: u64,
    },
    /// Per-topic rate (Hz) + frame counts + the worker's drop/coalesce/reconnect
    /// counters.
    Status {
        /// Correlation id echoed in the response.
        id: u64,
    },
    /// Replace the viewer's blueprint (viewport layout) at RUNTIME from a
    /// rerun-agnostic [`LayoutSpec`] — the automagic-bar "optimized layout" verb.
    /// The daemon validates the spec, translates it to a `rerun` blueprint, hands
    /// it to the never-block worker, and answers with a [`SetBlueprintResponse`]
    /// (or a structured error naming the problem). It does NOT block on rendering.
    ///
    /// `layout: null` / absent RESETS the viewport to the built-in Go2 default
    /// dashboard (a `Some(spec)` supersedes it). The typical agent flow is
    /// `discover` → `attach` topics (each `attach` reports the entity path it
    /// renders under) → `set_blueprint` arranging those entity-path origins into
    /// views.
    SetBlueprint {
        /// Correlation id echoed in the response.
        id: u64,
        /// The layout to apply, or `null`/absent to reset to the Go2 default.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        layout: Option<LayoutSpec>,
    },
    /// COMPILE a viewport layout from a SEMANTIC [`LayoutIntent`]
    /// (topic terms) — the automagic "just lay these out well" verb. The daemon
    /// attaches any missing topic, resolves each, compiles a grounded blueprint
    /// (one plot per topic, no origin guessing, `auto_views` off), validates it,
    /// hands it to the never-block worker, and answers with a
    /// [`ComposeLayoutResponse`] echoing the placements + the applied arrangement
    /// (or a structured error naming the problem). It does NOT block on rendering.
    ComposeLayout {
        /// Correlation id echoed in the response.
        id: u64,
        /// The semantic layout intent to compile.
        intent: LayoutIntent,
    },
    /// SIDE-LOAD schema definitions into the daemon's schema universe.
    ///
    /// The daemon's `FrameWalker` is built at boot from the built-in ROS 2 corpus
    /// plus the `msg_dirs` of the config named by `DDS_BRIDGE_CONFIG`, so a frame
    /// carrying a type NEITHER of those covers resolves to nothing and renders
    /// nothing. The ONLY other way to add one at runtime is a REMOTE attach,
    /// whose schema fetch seeds the walker as a side effect — unreachable for a
    /// purely local topic.
    ///
    /// A player of a recorded bag is exactly that case: `cerulion bag play`
    /// republishes onto LOCAL SHM, and the bag carries the verbatim
    /// definitions of the custom types its frames use. This verb is how it hands
    /// them over, so a vendor-typed recording renders on a desk that never
    /// compiled the type.
    ///
    /// IDEMPOTENT: seeding docs the daemon already holds, byte-identical, is a
    /// no-op that rebuilds nothing — so a client may re-send the same set freely
    /// (a player does, so that a viewer started mid-playback still learns them).
    Schemas {
        /// Correlation id echoed in the response.
        id: u64,
        /// The definitions to fold in. Each is a verbatim `.msg`/YAML text with
        /// the qualified names of the other docs it references, exactly the
        /// closure vocabulary the network catalog serves.
        docs: Vec<cerulion_core::SchemaDoc>,
    },
    /// Subscribe THIS connection to catalog-change EVENTS — the daemon writes
    /// a [`CatalogChangedEvent`] line down this connection, unprompted, whenever the
    /// LAN's topic catalog changes (a robot appearing, a robot dying, an `ros2 attach`
    /// graph's topics coming up on the robot).
    ///
    /// The intended controller response is to run the refresh it already has
    /// (`discover`). The event says WHAT changed at robot granularity and carries a
    /// monotonic `version`; it is deliberately not a row-level delta, so a controller
    /// need not model the catalog to stay current, and a MISSED event cannot leave it
    /// stale (the next one carries a newer version and it refreshes anyway).
    ///
    /// Answered with a [`SubscribeEventsResponse`] carrying the current snapshot
    /// (version + robots) plus `connected`, which says plainly whether the daemon can
    /// currently receive changes at all — a controller is never left waiting on a push
    /// that cannot come.
    ///
    /// The subscription is SCOPED to this connection: closing it unsubscribes (there
    /// is deliberately no `unsubscribe` verb — closing the connection IS one). A
    /// re-subscribe is idempotent and re-reports the snapshot.
    ///
    /// A connection that sends this MUST be prepared to read an unsolicited line at
    /// any point; a connection that does NOT send it never receives one, which is what
    /// makes this additive to the sacred contract (see [`PROTOCOL_VERSION`]).
    SubscribeEvents {
        /// Correlation id echoed in the response.
        id: u64,
    },
    /// Monitors: every watched topic's current watchdog state plus the daemon's
    /// recent alert ring — answered with a [`MonitorsResponse`].
    ///
    /// A NEW VERB rather than a field on [`Self::Status`], because `status` covers
    /// ATTACHED topics only and half of what a monitor watches is not attached. It
    /// is also how the contract negotiates capability: a daemon that predates this
    /// answers the unknown `method` with the structured error, which a client reads
    /// as "not supported" — exactly what [`PROTOCOL_VERSION`]'s doc prescribes, so
    /// there is **no version bump**.
    ///
    /// POLL-ONLY in v1. The agent asks, remembers the highest `seq` it has seen,
    /// and dedupes client-side; pushing alerts over the event hub is a
    /// v1.1 extension that needs no new plumbing when it comes.
    ///
    /// The verb and its handler land TOGETHER, deliberately. The dispatch match is
    /// exhaustive, so a request variant cannot exist without a handler — and a verb
    /// that PARSES but cannot be served would answer a DIFFERENT error from an old
    /// daemon's unknown-method, leaving a client unable to tell "not supported"
    /// from "supported but broken", which is the one thing the capability argument
    /// above rests on.
    Monitors {
        /// Correlation id echoed in the response.
        id: u64,
    },
    /// The live `graph run`s this desk can see: the LOCAL machine's
    /// own, plus every remote robot's, folded into ONE list. Answered with a
    /// [`RunsResponse`].
    ///
    /// A NEW VERB, and **no [`PROTOCOL_VERSION`] bump** — capability is negotiated
    /// BY VERB (a daemon that predates this answers the unknown `method` with the
    /// structured error, which a client reads as "not supported"), exactly as the
    /// `monitors` verb above and `PROTOCOL_VERSION`'s own doc prescribe.
    ///
    /// # `robot` scopes the question, and `None` is NOT "the local machine"
    ///
    /// * `None` — ask EVERYTHING: this machine's run registry AND every announcing
    ///   robot.
    /// * `Some(r)` — ask robot `r` and nothing else. The local arm is **not run**,
    ///   and that is the rule showing through rather than an omission: a
    ///   local run is attributed `robot: None`, never stamped with this desk's own
    ///   hostname, so no robot NAME can ever select one. Passing this machine's
    ///   hostname therefore correctly yields no local rows.
    ///
    /// # Cost — fetch once per `run_id`, never in a poll
    ///
    /// A remote serve builds a fresh iceoryx2 reader node per call (the ~620 ms
    /// shape measured on an idle desk) plus one gather window, and the LOCAL arm pays its
    /// own gather. A run's DAG is immutable for the life of its `run_id`, so a
    /// controller caches on that and refreshes liveness off the `discover` poll it
    /// already makes. **Never fold this verb into a poll loop.**
    Runs {
        /// Correlation id echoed in the response.
        id: u64,
        /// Scope the query to ONE robot (see the type docs). Omitted = ask
        /// everything, this machine included.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        robot: Option<String>,
    },
}

impl Request {
    /// The correlation `id` this request carries (echoed in its response).
    pub fn id(&self) -> u64 {
        match self {
            Request::Discover { id }
            | Request::Attach { id, .. }
            | Request::Detach { id, .. }
            | Request::List { id }
            | Request::Status { id }
            | Request::SetBlueprint { id, .. }
            | Request::ComposeLayout { id, .. }
            | Request::Representation { id, .. }
            | Request::Schemas { id, .. }
            | Request::SubscribeEvents { id }
            | Request::Monitors { id }
            | Request::Runs { id, .. } => *id,
        }
    }

    /// Serialize to the single NDJSON request line (client/test helper).
    pub fn to_json_line(&self) -> String {
        serde_json::to_string(self).expect("Request serializes")
    }
}

/// A failed [`parse_request`]: the request line was not a valid request. Carries
/// the BEST-EFFORT correlation `id` (extracted from the raw JSON's `id` field
/// even when the `method` was unknown / missing) so the daemon's error response
/// still correlates where possible, plus the parse `message`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RequestError {
    /// Best-effort correlation id (`None` when the line carried no numeric `id`).
    pub id: Option<u64>,
    /// The parse-failure detail (a serde message).
    pub message: String,
}

/// Parse ONE NDJSON request line. On success returns the typed [`Request`]; on
/// ANY failure (not JSON, missing/unknown `method`, missing `id`, wrong types)
/// returns a [`RequestError`] carrying a best-effort `id` for correlation — the
/// caller answers with a structured [`Response::error`], NEVER a panic or a
/// dropped connection.
pub fn parse_request(line: &str) -> Result<Request, RequestError> {
    match serde_json::from_str::<Request>(line) {
        Ok(req) => Ok(req),
        Err(e) => {
            // Best-effort id extraction: re-parse loosely so an unknown-method /
            // malformed-body line still correlates its error to the request id.
            let id = serde_json::from_str::<serde_json::Value>(line)
                .ok()
                .and_then(|v| v.get("id").and_then(serde_json::Value::as_u64));
            Err(RequestError {
                id,
                message: e.to_string(),
            })
        }
    }
}

/// A control response. UNTAGGED (each variant is a self-contained struct with
/// `id` + `ok` + its own fields) — untagged serialization emits the inner struct
/// DIRECTLY (no `flatten` buffering), so the field order is the struct's
/// declaration order regardless of serde_json's `preserve_order` feature (the
/// exact-bytes contract the codec tests pin). Serialize-only: controllers parse
/// responses field-by-field off the `ok` discriminant (Studio uses its own
/// types); the daemon never deserializes a response.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(untagged)]
pub enum Response {
    /// `attach` succeeded (or was already attached).
    Attach(AttachResponse),
    /// `detach` result.
    Detach(DetachResponse),
    /// `representation` result.
    Representation(RepresentationResponse),
    /// `list` result.
    List(ListResponse),
    /// `discover` result.
    Discover(DiscoverResponse),
    /// `status` result.
    Status(StatusResponse),
    /// `set_blueprint` result (the applied layout's view count + soft warnings).
    SetBlueprint(SetBlueprintResponse),
    /// `compose_layout` result (the placement echo + the applied arrangement).
    ComposeLayout(ComposeLayoutResponse),
    /// `schemas` result (what the side-load changed).
    Schemas(SchemasResponse),
    /// `subscribe_events` accepted — the current catalog snapshot.
    SubscribeEvents(SubscribeEventsResponse),
    /// The `monitors` result — every watched row's current state plus the
    /// recent alert ring.
    Monitors(MonitorsResponse),
    /// The `runs` result: the LOCAL machine's live runs folded with every
    /// remote robot's.
    Runs(RunsResponse),
    /// Any error (bad request, attach failure, reserved-field rejection).
    Error(ErrorResponse),
}

/// The response to [`Request::SubscribeEvents`] — the snapshot a controller
/// starts from, so it never has to guess whether what it already renders is current.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct SubscribeEventsResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// Always `true` on this variant (the `ok` discriminator every response carries).
    pub ok: bool,
    /// The catalog generation THIS daemon has relayed up to.
    ///
    /// MONOTONIC, and monotonic for real: it is vizd's own counter, not the upstream's.
    /// `cerulion-netd`'s generation restarts when netd does, so relaying its number
    /// verbatim would walk this one backwards across a netd restart; vizd stamps its
    /// own at publish time and clamps forward when it adopts a snapshot, so the number
    /// a controller sees only ever grows for as long as this daemon runs.
    ///
    /// A controller compares it against the `version` on later events; a jump means it
    /// missed one, which is harmless because the correct response to any event is a
    /// full refresh. A DECREASE cannot happen — a consumer may safely skip an event
    /// whose version is not greater than the last it handled.
    pub version: u64,
    /// The robots currently known to be announcing, sorted.
    pub robots: Vec<String>,
    /// Whether this daemon can currently DELIVER a catalog change.
    ///
    /// `false` means NO event can arrive right now, and it covers every reason:
    /// `cerulion-netd` is not reachable, is too old to serve the push, was started
    /// without a network plane (so it accepted the subscription but is watching no
    /// announce space — its own `watching: false`, reported through rather than hidden
    /// behind an optimistic `true`), or this daemon was started with no event source at
    /// all. The controller is told PLAINLY so it keeps its own refresh path rather than
    /// waiting on something that cannot come.
    pub connected: bool,
}

/// The `event` discriminator [`CatalogChangedEvent`] always carries.
pub const CATALOG_CHANGED_EVENT: &str = "catalog_changed";

/// The SERVER-INITIATED catalog-change notification — the one message on this
/// contract the daemon writes UNPROMPTED, and only down connections that sent
/// [`Request::SubscribeEvents`].
///
/// Distinguished from a [`Response`] on the wire by its `event` field, which no
/// response carries; every response carries `id`, which this does not. A controller
/// classifies each line by which of the two it sees — position is never assumed, since
/// an event can land between a request and its response.
///
/// It is a NOTIFICATION, not a delta stream: the payload says THAT the catalog moved,
/// WHICH robots it involved, and HOW CURRENT the sender is. Row-level reconstruction
/// is deliberately not attempted — the controller refreshes, which is where rows come
/// from, and that keeps a missed event harmless instead of corrupting.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CatalogChangedEvent {
    /// Always [`CATALOG_CHANGED_EVENT`] — the discriminator.
    pub event: String,
    /// The catalog generation after this change — THIS daemon's own counter, stamped
    /// at relay time. Strictly increasing for as long as the daemon runs, INCLUDING
    /// across a `cerulion-netd` restart (netd's generation restarts; this one does
    /// not). See [`SubscribeEventsResponse::version`].
    pub version: u64,
    /// The robots announcing after this change, sorted — the authoritative
    /// post-change robot set.
    pub robots: Vec<String>,
    /// Robots that became present (net over the coalescing window).
    pub robots_added: Vec<String>,
    /// Robots that went away ENTIRELY — the whole-robot transition (a dying gateway
    /// drops every token at once, reported here as one robot removal rather than as N
    /// topic removals).
    pub robots_removed: Vec<String>,
    /// How many topic announces appeared (churn over the window, not net).
    pub topics_added: u64,
    /// How many topic announces went away (churn over the window, not net).
    pub topics_removed: u64,
}

impl CatalogChangedEvent {
    /// Relay a `cerulion-netd` catalog-change push into this contract's shape. The two
    /// are deliberately separate types — netd's seam and Studio's are different
    /// contracts with different version stories — and the VERSION is exactly where
    /// they part.
    ///
    /// netd's `version` is NOT copied. It counts netd's own generations and restarts
    /// at 1 when netd does, which would walk this contract's monotone `version`
    /// backwards on a netd restart. The relaying daemon stamps its own number at
    /// publish time (`crate::events::EventHub::publish`); the `0` here is a placeholder
    /// that is always overwritten before the event reaches a controller.
    pub fn from_netd(push: &cerulion_netd::CatalogChanged) -> Self {
        Self {
            event: CATALOG_CHANGED_EVENT.to_string(),
            version: 0,
            robots: push.robots.clone(),
            robots_added: push.robots_added.clone(),
            robots_removed: push.robots_removed.clone(),
            topics_added: push.topics_added,
            topics_removed: push.topics_removed,
        }
    }

    /// Fold a NEWER change into this still-undelivered notification (the slow-consumer
    /// path). LOSSLESS: robot membership resolves to the NET effect — a robot that
    /// arrived and then left inside the window ends up in `robots_removed` only, never
    /// in both — churn counts sum, and the version + robot set take the newest values.
    /// So the one line a lagging controller eventually reads describes everything it
    /// missed and is safe to refresh on.
    pub fn merge_newer(&mut self, newer: &CatalogChangedEvent) {
        use crate::events::{union_names, without_names};
        let added = without_names(
            &union_names(&self.robots_added, &newer.robots_added),
            &newer.robots_removed,
        );
        let removed = without_names(
            &union_names(&self.robots_removed, &newer.robots_removed),
            &newer.robots_added,
        );
        self.robots_added = added;
        self.robots_removed = removed;
        self.topics_added = self.topics_added.saturating_add(newer.topics_added);
        self.topics_removed = self.topics_removed.saturating_add(newer.topics_removed);
        self.version = newer.version;
        self.robots = newer.robots.clone();
    }

    /// Serialize to the single NDJSON push line (no trailing newline — the writer adds
    /// it).
    pub fn to_json_line(&self) -> String {
        serde_json::to_string(self).expect("CatalogChangedEvent serializes")
    }
}

impl Response {
    /// Build the structured error response (`ok:false`). `id` is the request's
    /// correlation id (`None` when the request was unparseable).
    pub fn error(id: Option<u64>, error: impl Into<String>, topic: Option<String>) -> Self {
        Response::Error(ErrorResponse {
            id,
            ok: false,
            error: error.into(),
            topic,
            // TERMINAL by default. Only the two discovery seams that can
            // truthfully say "not found YET" call `error_retryable`; every other error
            // path keeps the wire without this field byte-for-byte.
            retry: None,
        })
    }

    /// An error that a client may RE-ASK — see [`RetryHint`].
    ///
    /// `retry: None` collapses to [`Self::error`], so a seam whose hint is
    /// conditional does not need two call sites.
    pub fn error_retryable(
        id: Option<u64>,
        error: impl Into<String>,
        topic: Option<String>,
        retry: Option<RetryHint>,
    ) -> Self {
        Response::Error(ErrorResponse {
            id,
            ok: false,
            error: error.into(),
            topic,
            retry,
        })
    }

    /// Serialize to the single NDJSON response line (no trailing newline; the
    /// writer adds it). Infallible for these plain types.
    pub fn to_json_line(&self) -> String {
        serde_json::to_string(self).expect("Response serializes")
    }
}

/// Monitors: the `monitors` result — every watched topic's CURRENT state plus
/// the daemon's recent alert ring.
///
/// # Why a new VERB rather than a field on `status`
///
/// `status` covers ATTACHED topics only, and half of what a monitor watches is
/// not attached. A new verb is also how this contract negotiates capability: an
/// old daemon answers an unknown `method` with the structured error, which is
/// exactly what [`PROTOCOL_VERSION`]'s doc prescribes — hence **no version bump**.
///
/// # Retention, and why a CLEAR is a new entry
///
/// [`Self::monitors`] is STICKY per topic and always readable (Principle #3).
/// [`Self::alerts`] is a bounded ring
/// ([`MONITOR_ALERT_RING`](cerulion_viz::monitor::MONITOR_ALERT_RING)) with a
/// monotonic `seq`: the agent polls, remembers the highest `seq` it has seen, and
/// dedupes client-side. There is deliberately **no per-client read cursor on the
/// daemon** — a daemon whose answer depends on who asked is not independently
/// observable. That is also what forces a CLEAR to be a NEW entry with a NEW
/// `seq` rather than a mutation of the raise already in the ring: an agent cannot
/// learn about an in-place edit to an entry it has already read past.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct MonitorsResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// Always `true` — a failure is an [`ErrorResponse`].
    pub ok: bool,
    /// Every watched row, sorted by topic.
    pub monitors: Vec<MonitorRow>,
    /// The recent alert ring, oldest first.
    pub alerts: Vec<Alert>,
    /// The next `seq` this daemon will assign — the ring's high-water mark. An
    /// agent that has seen every entry up to `seq` knows it has missed nothing
    /// EXCEPT what the bounded ring has already evicted.
    pub seq: u64,
}

/// ONE live run, attributed to the machine executing it.
///
/// A row is a run PLUS its two self-describing artifacts, verbatim: the effective
/// `graph.yaml` the run is executing and its `run.json` manifest. The graph
/// travels as TEXT and is re-parsed desk-side by the SAME `GraphConfig` parser the
/// robot used, so the desk cannot drift from the robot by construction —
/// which is also why there is no serialized topology here
/// and probably never will be.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct RunRow {
    /// **`None` means a GENUINE LOCAL run: this machine's own.**
    ///
    /// It is never this desk's hostname, and that is the whole attribution rule
    /// rather than a formatting preference. Attribution is a property of the KEY
    /// the answer came back on — the explicit `cerulion_q/{robot}/runs` selector —
    /// so a local run, which crossed no such key, has no robot to name. Stamping
    /// one would mint a phantom "this machine" row in the
    /// sidebar, and worse: a desk and a robot would then be indistinguishable in a
    /// list whose entire purpose is telling you WHERE something is running.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot: Option<String>,
    /// The run's 128-bit identity, canonical text (`0x` + 32 lowercase hex).
    ///
    /// A restart mints a new one, which is what makes a controller's
    /// fetch-once-per-`run_id` cache correct.
    pub run_id: String,
    /// The run's graph name (its LOGICAL identity — the internal `config.name`).
    pub graph_name: String,
    /// Wall-clock ns since the Unix epoch at which the run started — for rendering
    /// and for ordering concurrent runs, never for identity.
    pub run_started_at_ns: u64,
    /// What the run said about itself at gather time (`live` / `ending`).
    pub state: RunEntryState,
    /// The run's effective `graph.yaml`, VERBATIM.
    pub graph_yaml: String,
    /// The run's `run.json` manifest, VERBATIM.
    pub run_json: String,
}

/// What ONE source of the fold answered, and what its answer is
/// EVIDENCE of.
///
/// # Why per-source rather than one verdict
///
/// [`RunsResponse::completeness`] answers the single question a renderer asks
/// ("may I read an empty list as *nothing is running*?"), and it is deliberately
/// conservative — one unsettled source makes the whole answer unsettled. That is
/// the right verdict and the wrong DIAGNOSTIC: it cannot say WHICH machine could
/// not be established, and the remedies differ completely (a robot that REFUSED
/// this desk is a pairing grant; a gather whose window expired is a retry).
///
/// So the aggregate is what you render and this is what you act on.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct RunsSourceStatus {
    /// Which machine — `None` = this one (see [`RunRow::robot`]).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot: Option<String>,
    /// What an empty contribution from THIS source means.
    pub completeness: RunsCompleteness,
    /// The source REFUSED this desk (its demand-authorization gate denied the
    /// runs GET — an account/pairing grant). Present only on a refusal,
    /// and never confusable with an absence: a refusal always rides a
    /// non-settled `completeness`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Runs this source KNOWS are live but could not describe (an unreadable
    /// `graph.yaml`, an oversized `run.json`). Their absence from
    /// [`RunsResponse::runs`] would otherwise be invisible — and a source whose
    /// ONLY run is undescribable serves an empty list, which is why any entry
    /// here also forbids a `Settled` verdict.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub undescribable: Vec<UndescribableRun>,
}

/// The `runs` result: this machine's live runs folded with every
/// remote robot's.
///
/// # The three ways this answer can be short, and why each is its own field
///
/// A renderer that collapses them tells an operator to do the wrong thing:
///
/// 1. [`Self::degraded`] — `cerulion-netd` could not RUN the remote query at all
///    (unreachable, or not network-configured). The LOCAL half still answers; the
///    remote half is UNKNOWN. Remedy: look at netd.
/// 2. [`Self::unusable`] — a robot ANSWERED with bytes this binary could not
///    decode. It is neither a reply nor a silence, and it will re-serve the same
///    bytes forever, so it renders as **redeploy that robot** and NEVER as "still
///    discovering". This is the distinction `cerulion_netd::RunsAnswer` carries on
///    its primary verb precisely so a consumer cannot lose it.
/// 3. [`Self::silent`] — a robot netd ASKED that answered nothing at all. The
///    query ran and this robot was in it, so it is neither `degraded` nor
///    `discovering`; it may answer next poll, or never (an old binary, a Strict
///    ingress-only gateway with no query surface). Remedy: wait, or upgrade it.
/// 4. A source that answered but could not establish its own picture — carried
///    per source on [`Self::sources`], with its refusal reason if it refused.
///
/// And a fifth, which is NOT a shortfall: [`Self::discovery`] `discovering`
/// means netd has not completed a discovery pass, so a robot that is right there
/// on the LAN may simply not have been asked yet. Keep refreshing.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct RunsResponse {
    /// Echoed correlation id.
    pub id: u64,
    /// Always `true` — a failure is an [`ErrorResponse`].
    pub ok: bool,
    /// Every live run, local first then by robot, each sorted by start time. One
    /// row per `run_id`: a run this desk sees both first-hand and echoed back over
    /// the LAN (its own announcing netd) is listed once, as local.
    pub runs: Vec<RunRow>,
    /// **The one question a renderer asks**: may an empty [`Self::runs`] be read
    /// as "nothing is running anywhere I can see"?
    ///
    /// `Settled` only when EVERY contributing source settled, nothing was
    /// unusable, EVERY asked robot answered ([`Self::silent`] empty), the remote
    /// arm ran, netd's discovery had converged, and at least one source
    /// contributed at all. Anything else is `Incomplete` — the answer is whatever
    /// arrived, and it is not an absence claim.
    pub completeness: RunsCompleteness,
    /// Per-source verdicts — the DIAGNOSTIC half of `completeness` (see
    /// [`RunsSourceStatus`]). A source appears here iff it contributed an answer,
    /// so an EMPTY list means nothing answered at all.
    pub sources: Vec<RunsSourceStatus>,
    /// Robots that answered with something this binary could not use — the
    /// **redeploy** signal. Omitted when empty (the healthy wire).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub unusable: Vec<UnusableRunsAnswer>,
    /// Robots that were ASKED and did not answer at all — the **coverage**
    /// shortfall, and a FOURTH remedy again.
    ///
    /// It is not `unusable` (nothing came back to fail to decode), not `degraded`
    /// (the query ran), and not `discovering` (netd finished looking and this robot
    /// was among the ones it looked at). Such a robot may answer on the next poll —
    /// a slow serve — or may never: a binary predating the `runs` verb, or a Strict
    /// ingress-only gateway that declares no query surface at all. So the correct
    /// rendering names the robot and withholds any claim about what it is running,
    /// and the operator's move is to wait or to upgrade it.
    ///
    /// **Its presence is what stops [`Self::completeness`] settling**, which is the
    /// whole reason it is on the wire rather than only in a log: without it a short
    /// answer and a complete one arrive identically, and a renderer folding either
    /// into "these are the runs" makes a settled-absence claim about a machine
    /// nobody heard from.
    ///
    /// Omitted when empty (a fully-covered answer is the healthy wire).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub silent: Vec<String>,
    /// Whether `cerulion-netd`'s LAN discovery had converged when the remote arm
    /// was answered. `None` = UNKNOWN (the remote arm did not run, or netd could
    /// not say) — never a positive claim, the same discipline
    /// [`DiscoverResponse::discovery`] follows.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub discovery: Option<DiscoveryLabel>,
    /// The remote arm could not be run at all, and why. The LOCAL half of the
    /// answer is still present and still true; the remote half is UNKNOWN.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub degraded: Option<String>,
}

/// `attach` success payload. `schema`/`archetype` are `null` when the topic was
/// silent at attach (a data-only tap has no history, so resolution is
/// best-effort); they fill in once frames flow (visible via `status`/`list`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct AttachResponse {
    /// Correlation id.
    pub id: Option<u64>,
    /// Always `true`.
    pub ok: bool,
    /// The absolute topic now tapped.
    pub topic: String,
    /// The resolved schema qualified name (`pkg/Type`), or `null` if unresolved.
    pub schema: Option<String>,
    /// The Rerun archetype family the sink will render it as, or `null`.
    pub archetype: Option<String>,
    /// The full Rerun entity path the frames render under.
    pub entity: String,
    /// The topic's render-route key — the topic with its leading/trailing `/`
    /// trimmed (`/utlidar/cloud` → `utlidar/cloud`).
    ///
    /// Diagnostic only: `entity` above is the authoritative render location, and
    /// it is a mechanical function of the topic. An `entity`
    /// OVERRIDE does not appear here — it changes the entity, never the topic's
    /// identity, and the daemon's internal key encodes both halves in a form that
    /// is not wire-safe.
    pub route: String,
    /// `true` when the topic was ALREADY tapped (idempotent — no second tap).
    pub already_attached: bool,
    /// The rerun component/archetype families the topic renders as
    /// (`["Points3D"]`, `["Transform3D","Scalars"]`, …), or `null` when the
    /// archetype is unresolved (a silent topic). Mirrors the `schema`/`archetype`
    /// null-not-absent convention: `null` = NOT yet resolved (no view answer
    /// exists), distinct from a `[]` "resolved: renders nothing".
    pub components: Option<Vec<String>>,
    /// The resolved rerun view kind(s) the topic renders into
    /// (`["spatial3d"]`, `["spatial3d","time_series"]`, …) — the placement so the
    /// agent never guesses — or `null` when the archetype is unresolved (same
    /// null-not-absent convention as `components`).
    ///
    /// **A POINT-IN-TIME reading, not a constant of the archetype.** It
    /// is not derivable from `archetype` alone; it is a fact about THIS
    /// topic AT THIS MOMENT, because two live signals feed it. The
    /// `representation` is the operator's own choice, so a client that made it
    /// knows to re-read; the render evidence arrives on FRAMES, with no
    /// client action at all — a `/map` attached while silent reports its dump
    /// companion, and loses it milliseconds later when the first frame renders
    /// cleanly (and regains it if one ever degrades). So this value must be
    /// re-read from `status`/`list` rather than cached from the attach reply,
    /// exactly as this struct's own doc already says of `schema`/`archetype`.
    ///
    /// The reply carries no CAUSE for the change, unlike the row surfaces, which
    /// carry `representation` (see [`DiscoveredEntry::representation`] for that
    /// rule). Deliberate, and pre-existing: `AttachResponse` has never carried
    /// `representation` either — the rule is applied to ROW surfaces, which are
    /// polled and rendered as a table, and the render proof additionally has no
    /// control behind it for a client to act on.
    pub view_kinds: Option<Vec<String>>,
    /// The ORIGIN ROBOT this newly-attached tap's data comes from, or
    /// ABSENT for a genuine local producer.
    ///
    /// The attach reply is the FOURTH surface naming a topic, and it is the one the
    /// client acts on immediately — so it carries the same attribution as
    /// `discover`/`list`/`status` rather than leaving the client to infer where the
    /// row belongs from the request it happened to send.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub robot: Option<String>,
}

/// `detach` result payload.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct DetachResponse {
    /// Correlation id.
    pub id: Option<u64>,
    /// Always `true`.
    pub ok: bool,
    /// The topic that was (or was not) tapped.
    pub topic: String,
    /// `true` when a tap was actually removed, `false` if it was not attached.
    pub detached: bool,
}

/// `representation` result payload.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct RepresentationResponse {
    /// Correlation id.
    pub id: Option<u64>,
    /// Always `true`.
    pub ok: bool,
    /// The topic whose representation was set.
    pub topic: String,
    /// The representation now in force — the ECHO an operator's row renders
    /// from, so the sidebar never has to assume its request took.
    pub representation: String,
    /// `true` when the choice reached the render immediately (the topic is
    /// attached); `false` when it was REMEMBERED for the next attach.
    ///
    /// Both are successes and neither is a no-op, but they differ in what the
    /// operator will SEE, so the row is told which one it got rather than left
    /// to infer it from an attach state it may be about to change.
    pub applied: bool,
}

/// One currently-attached tap in a [`ListResponse`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct AttachedEntry {
    /// The absolute topic.
    pub topic: String,
    /// The topic's render-route key (diagnostic; see
    /// [`AttachResponse::route`]). Never carries an `entity` override.
    pub route: String,
    /// The full Rerun entity path.
    pub entity: String,
    /// The resolved schema qualified name, or `null` if not yet resolved.
    pub schema: Option<String>,
    /// The resolved archetype family, or `null`.
    pub archetype: Option<String>,
    /// The rerun component/archetype families the topic renders as, or
    /// `null` when the archetype is unresolved (same null-not-absent convention
    /// as `schema`/`archetype`). Wired through the same layout seam the
    /// `attach`/`status`/`discover` responses use so `list` carries the identical
    /// placement — the agent never re-guesses per verb.
    pub components: Option<Vec<String>>,
    /// The resolved rerun view kind(s) the topic renders into, or `null`
    /// when unresolved (same convention as `components`).
    ///
    /// **A POINT-IN-TIME reading** — see
    /// [`AttachResponse::view_kinds`]. The cross-verb guarantee above is about
    /// AGREEMENT at one instant, and the live-signal read makes it stronger rather than weaker:
    /// all four attached-topic reporters read the same live signals through one
    /// seam, so `list` and `status` cannot answer differently about the same topic.
    /// The value is NOT a constant of the archetype: it
    /// moves as the topic proves it is rendering, or degrades.
    pub view_kinds: Option<Vec<String>>,
    /// The ORIGIN ROBOT this tap's data comes from, or ABSENT when the
    /// topic is a genuine local producer (a desk graph).
    ///
    /// Same field, same meaning, same source of truth as
    /// [`DiscoveredEntry::robot`] — an attached netd MIRROR of a remote robot's
    /// topic is that robot's topic, not a second data source this desk owns
    /// ("one data source = one topic"). Without this the attached row
    /// carried no attribution at all, so a remote-attached topic materialized a
    /// phantom local section the moment the demand landed, duplicating the row
    /// the user had actually checked in the robot's section.
    ///
    /// `skip_serializing_if` keeps a purely-local desk's wire BYTE-IDENTICAL to
    /// the wire without this field.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub robot: Option<String>,
    /// Is a producer still REGISTERED for this topic on this desk? Same
    /// field name and meaning as [`DiscoveredEntry::producer_count`] — one
    /// vocabulary across every surface.
    ///
    /// The registration half of the sidebar rule: a topic that is not
    /// registered disappears from the sidebar instead of persisting.
    /// A checked row is exempt from the client's discover-prune (the attach state
    /// is `list`'s to own), so without this field an attached topic whose producer has
    /// been killed persists forever with a stale rate. `Some(0)` is that row: the
    /// topic's SHM service still exists — this daemon's own tap holds it open, so
    /// service ENUMERATION can never answer this question — but nothing is
    /// registered to publish into it.
    ///
    /// `None` is UNKNOWN (a probe failure) and MUST NOT hide a row: only a
    /// confirmed `Some(0)` is evidence of absence.
    ///
    /// SCOPE for a MIRROR: the desk-side count reflects `cerulion-netd`'s
    /// re-injector, not the origin robot's publisher, so a demanded mirror of a
    /// topic whose robot has stopped reads `Some(1)` and renders as
    /// registered-but-silent (the sidebar rule's other state — red "no data") rather than
    /// vanishing. Releasing the demand (unchecking) retires the mirror and the row
    /// then goes on its own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub producer_count: Option<u32>,
    /// What THIS DAEMON'S OWN TAP observed about the topic's data flow —
    /// frames that actually arrived — or `None` when it is not observing.
    ///
    /// Same type, same vocabulary, same thresholds as
    /// [`DiscoveredEntry::liveness`], with one difference worth stating: that field
    /// carries the SERVING ROBOT's observation of a topic nobody here has tapped,
    /// while this one is the desk's FIRST-HAND observation of a tap it holds. A tap
    /// is a data-flow observer by construction, so this is available for every
    /// attached topic on any robot — including one that predates the robot-side observer entirely.
    ///
    /// Ages are dated by this daemon's own monotonic clock (never the publisher's
    /// wire stamp — see `TopicStat::last_frame_at`), and the first drain after an
    /// attach is a BASELINE that is banked but never dated.
    ///
    /// That defeats the ordinary case of a dead route whose retained history flushes
    /// into a fresh tap, but it is NOT absolute:
    /// a flush SPLIT across two poll ticks dates on its second chunk, so such a
    /// route can read `"streaming"` once per attach for up to the streaming-recency
    /// window before settling to `"idle"`. Bounded and self-correcting, and the same
    /// class of residual `cerulion_core::transport::liveness` records for the
    /// robot-side observer — which is the authority for the vocabulary; closing it
    /// here would mean a second copy of the two-guard stamp-advancement
    /// machine on this side of the wire. `TopicStat::observe_frames` states the
    /// residual in full; a reader must not treat a single `"streaming"` reading
    /// immediately after an attach as proof of a live producer.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub liveness: Option<TopicLiveness>,
    /// [`Self::liveness`] already classified, exactly as
    /// [`DiscoveredEntry::liveness_state`] — the serialized set is CLOSED to
    /// `"streaming"` / `"idle"` / `"no_data"`, and ABSENT means UNKNOWN.
    ///
    /// Classified HERE so the thresholds live in one oracle-tested place and the
    /// attach-state rows cannot drift from the discover rows.
    #[serde(default, skip_serializing_if = "liveness_state_is_absent_on_the_wire")]
    pub liveness_state: Option<LivenessState>,
    /// How this topic RENDERS — `auto` / `visual` / `text` / `both`.
    ///
    /// The row renders its representation control from THIS, not from what it last
    /// sent: the daemon is the source of truth (a second controller — the agent,
    /// another Studio window — can set one too), which is the same sidebar-truth
    /// rule already applied to the rate.
    ///
    /// ABSENT when the topic is on `auto`, so a desk where nobody has chosen
    /// anything serializes BYTE-IDENTICALLY to the wire without this field.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub representation: Option<String>,
    /// This daemon received frames on this topic and could NOT decode
    /// them — see [`UndecodableReport`]. ABSENT whenever the topic decodes (or
    /// has produced nothing yet), so a healthy desk's wire is byte-identical to
    /// the wire without this field.
    ///
    /// Read it TOGETHER with [`Self::schema`]: a `null` schema plus an absent
    /// report is a topic nothing has been decoded from YET; a `null` schema plus
    /// a report is a topic whose frames this build cannot read, which is not a
    /// waiting state and will not resolve on its own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub undecodable: Option<UndecodableReport>,
}

/// `list` result payload.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ListResponse {
    /// Correlation id.
    pub id: Option<u64>,
    /// Always `true`.
    pub ok: bool,
    /// The attached taps, sorted by topic.
    pub attached: Vec<AttachedEntry>,
}

/// This daemon received frames on the topic and could NOT decode them.
///
/// Without this, a topic whose `schema_hash` matches nothing this build holds is
/// reported EXACTLY like a silent one — `schema: null`, `archetype: null` — while
/// `frames` climbs and `liveness_state` reads `streaming`. The two have opposite
/// remedies (wait / look at the publisher, vs rebuild or acquire a definition),
/// and the sidebar rendered them identically. This is the accurate state: frames
/// ARE arriving, and here is why nothing is drawn.
///
/// Additive on the wire (`skip_serializing_if`): a desk that decodes everything
/// serializes BYTE-IDENTICALLY to the earlier wire, and an older client that ignores
/// the field sees exactly what it saw before.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct UndecodableReport {
    /// The machine token a client switches on — a CLOSED set:
    /// `schema_version_skew` (this build holds the type at a DIFFERENT hash: the
    /// two builds disagree about the message definition), `schema_not_compiled`
    /// (the type is named but no schema for it exists here), or
    /// `schema_unidentified` (nothing but the hash). Mirrors
    /// `cerulion_core::codegen::unknown_hash::UnknownHashDiagnosis::reason_code`.
    pub reason: String,
    /// The `schema_hash` the wire actually carried, `0x`-prefixed uppercase hex.
    pub schema_hash: String,
    /// The qualified type name (`pkg/Type`) the topic is BELIEVED to carry, from
    /// the catalog or the attach pin — absent when nothing named it. Never
    /// guessed from the hash: the wire does not carry a name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,
    /// The hash THIS build computes for [`Self::schema`] — present only on a
    /// version skew, where the PAIR is the diagnosis (it names which end is
    /// stale). Same `0x`-prefixed uppercase hex.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub local_schema_hash: Option<String>,
    /// One self-contained human sentence: what went wrong AND what to do about
    /// it. Rendered verbatim — the non-technical user reading a Studio sidebar
    /// gets the same instruction the operator reading the log gets.
    pub detail: String,
}

/// One attachable topic in a [`DiscoverResponse`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct DiscoveredEntry {
    /// The absolute topic.
    pub topic: String,
    /// The best-effort resolved schema qualified name, or `null` (silent /
    /// undecodable topic).
    pub schema: Option<String>,
    /// The best-effort resolved archetype family, or `null`.
    pub archetype: Option<String>,
    /// The Rerun entity path this topic WOULD render under if attached
    /// (derived deterministically from the topic name), so the agent can lay out
    /// before attaching.
    pub entity: String,
    /// The rerun component/archetype families the topic renders as, or
    /// `null` when the archetype is unresolved (a silent/undecodable topic).
    /// Mirrors the `schema`/`archetype` null-not-absent convention: `null` = NOT
    /// yet resolved, distinct from a `[]` "resolved: renders nothing".
    pub components: Option<Vec<String>>,
    /// The resolved rerun view kind(s) the topic renders into, or `null`
    /// when the archetype is unresolved (same convention as `components`).
    pub view_kinds: Option<Vec<String>>,
    /// The origin robot when this entry is a REMOTE topic (from a robot's
    /// served catalog, grouped under [`DiscoverResponse::robots`]), or `None` for a
    /// genuinely LOCAL topic in [`DiscoverResponse::topics`]. Attach carries this
    /// through so the daemon `demand`s the remote mirror from `cerulion-netd`. It is
    /// `#[serde(default, skip_serializing_if = "Option::is_none")]`, so a LOCAL entry
    /// serializes BYTE-IDENTICALLY to the wire without this field (the anti-regression contract);
    /// a lenient decoder (Studio TS / the shell forwarder) ignores it when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub robot: Option<String>,
    /// The number of LIVE producers for this topic (the desk's own SHM
    /// producer count for a LOCAL topic, or the robot's catalog
    /// [`CatalogEntry::producer_count`](cerulion_core::CatalogEntry) for a REMOTE
    /// one), or `None` when it could not be probed. The Studio sidebar uses it as a
    /// per-row LIVENESS affordance: `Some(0)` is a registered-but-DEAD route (the
    /// topic exists but nothing is publishing — a `/uslam/cloud_map` whose
    /// mapper is not running), which the sidebar DIMS + labels "no data yet"; `Some(n)` (n>0) has a
    /// live producer; `None` degrades to the earlier rendering (no affordance).
    /// This is producer PRESENCE, not a publish rate (a rate for an un-tapped topic
    /// would need a per-topic frame wait — deliberately not paid). Additive +
    /// `#[serde(default, skip_serializing_if)]`, so a topic with an un-probed count
    /// serializes BYTE-IDENTICALLY to the wire without this field and a lenient decoder (Studio
    /// TS / the shell forwarder) ignores it when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub producer_count: Option<u32>,
    /// What the serving robot OBSERVED about this topic's DATA FLOW —
    /// frames that actually crossed it — or `None` when nothing observed it.
    ///
    /// This is the field a liveness affordance must read. [`Self::producer_count`]
    /// counts registered publisher PORTS, and a `cerulion ros2 attach` robot
    /// registers one per discovered DDS topic at graph-build time, so it reports
    /// `Some(1)` for a dead route and a streaming one alike (measured on the Go2:
    /// all 75 topics read `1`). `producer_count` is retained because the two
    /// answer different questions — a row with `producer_count: Some(1)` and
    /// `liveness_state: "no_data"` is precisely "registered, but nothing is
    /// publishing".
    ///
    /// `None` means UNKNOWN and MUST NOT be rendered as dead: a robot that
    /// predates the liveness observer, one with `CERULION_TOPIC_LIVENESS=off`, a WAN/iroh-served
    /// catalog, a topic whose observation tap could not attach, and every
    /// desk-LOCAL topic (the desk runs no observer — see the `topics` field) all
    /// report `None`. Additive + `#[serde(default, skip_serializing_if)]`, so an
    /// entry without it serializes BYTE-IDENTICALLY to the wire without this field.
    ///
    /// Inside the object, `last_frame_age_ms: null` with `frames_observed > 0` is
    /// a REAL state, not a gap: the topic delivered data the robot could not date
    /// (see `liveness_state`'s `"idle"`). Do not render an age for it and do not
    /// treat the missing age as "dead".
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub liveness: Option<TopicLiveness>,
    /// [`Self::liveness`] already classified. The serialized set is
    /// CLOSED to three values — `"streaming"`, `"idle"`, `"no_data"` — and the
    /// field is ABSENT for UNKNOWN.
    ///
    /// What each value means for a row, and the one that surprises people:
    ///
    /// * `"streaming"` — a frame the robot could DATE arrived recently; render
    ///   the live dot and, if you like, the age.
    /// * `"idle"` — covers TWO shapes. Either `last_frame_age_ms` is present and
    ///   says how stale, OR IT IS `null` while `frames_observed > 0`: the topic
    ///   produced data of UNKNOWN freshness (a latched `/tf_static`, a route whose
    ///   retained history flushed into the robot's tap, or a topic whose only
    ///   observed batch established the robot's dating baseline). Render it
    ///   plainly — undimmed, no live dot, no invented age. Be aware of what the
    ///   null-age shape does NOT tell you: it is ONE-WAY (a topic that has ever
    ///   banked a frame never returns to `"no_data"`) and it carries NO ageing
    ///   signal at all — a backlog the robot drained three hours ago and one it
    ///   drained 200 ms ago serve the identical payload, because the robot cannot
    ///   date either and will not invent an age from its own clock. Treat it as
    ///   "has data, freshness unknown", never as a staleness measurement.
    /// * `"no_data"` — and ONLY this — is the dimmed row: watched long enough
    ///   having seen NO frame at all. A row that carries frames is never
    ///   `"no_data"`.
    ///
    /// That is one encoding of UNKNOWN, not two. A `"unknown"` string alongside
    /// an absent field would be two spellings of the same state and every reader
    /// would have to handle both to be correct; instead producers classify
    /// through [`TopicLiveness::wire_state`] and the skip predicate below drops
    /// `Some(Unknown)` as well as `None`, so `"unknown"` cannot reach the wire
    /// from here. ([`LivenessState`]'s own DEcode stays tolerant, so an
    /// `"unknown"` produced elsewhere still parses — it just cannot originate
    /// here.)
    ///
    /// Serialized at all so the sidebar renders one agreed classification rather
    /// than re-deriving thresholds in TypeScript, where they would silently drift
    /// from the Rust ones. The thresholds live in
    /// [`LivenessState`]'s oracle-tested producer [`TopicLiveness::state`]; the
    /// raw numbers stay on `liveness` for a row that wants to show the age.
    #[serde(default, skip_serializing_if = "liveness_state_is_absent_on_the_wire")]
    pub liveness_state: Option<LivenessState>,
    /// How this topic RENDERS — `auto` / `visual` / `text` / `both`.
    ///
    /// Same field, same meaning, same source of truth as
    /// [`AttachedEntry::representation`]. Carried here because this row's
    /// `view_kinds` are ALREADY computed through the choice, so a row that
    /// reported the consequence without the cause would show view kinds its
    /// archetype alone does not imply and nothing to explain why.
    ///
    /// ABSENT on `auto`, so a desk where nobody has chosen anything stays
    /// BYTE-IDENTICAL on the wire to one without this field.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub representation: Option<String>,
    /// This daemon received frames on this topic and could NOT decode
    /// them — see [`UndecodableReport`]. ABSENT whenever the topic decodes (or
    /// has produced nothing yet), so a healthy desk's wire is byte-identical to
    /// the wire without this field.
    ///
    /// Read it TOGETHER with [`Self::schema`]: a `null` schema plus an absent
    /// report is a topic nothing has been decoded from YET; a `null` schema plus
    /// a report is a topic whose frames this build cannot read, which is not a
    /// waiting state and will not resolve on its own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub undecodable: Option<UndecodableReport>,
}

/// The skip predicate that keeps ABSENCE the ONE wire encoding of
/// UNKNOWN — see [`DiscoveredEntry::liveness_state`]. Both `None` and an
/// explicitly-`Unknown` classification serialize as "no key".
fn liveness_state_is_absent_on_the_wire(state: &Option<LivenessState>) -> bool {
    matches!(state, None | Some(LivenessState::Unknown))
}

/// One discovered ROBOT in a [`DiscoverResponse`] — a remote robot the
/// desk's `cerulion-netd` catalog gather found announcing on the LAN, with the
/// subset of its served topics NOT already visible locally. The sidebar renders a
/// ROBOTS section: one header row per robot, its topics grouped beneath (each
/// attachable via the demand plane, which `attach_remote` already handles).
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct DiscoveredRobot {
    /// The robot's identity (the catalog's self-attributed, desk-normalized name).
    pub robot: String,
    /// The robot's remote topics (each carries `robot: Some(..)` for attach), sorted
    /// by canonical name, with topics already present locally filtered out (they are
    /// already listed + attachable under [`DiscoverResponse::topics`] — the
    /// "one data source = one topic" surface). A REFUSED catalog carries no topics.
    pub topics: Vec<DiscoveredEntry>,
    /// `Some(reason)` iff the robot REFUSED to serve its catalog to this
    /// desk (a [`DemandAuthorizer`](cerulion_core::transport::demand_authorizer::DemandAuthorizer)
    /// denial). Surfaced EXPLICITLY (the robot still rows, with its refusal reason and
    /// no topics) — never a silent empty. `#[serde(skip_serializing_if)]`: an
    /// authorized robot omits it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// `discover` result payload.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct DiscoverResponse {
    /// Correlation id.
    pub id: Option<u64>,
    /// Always `true`.
    pub ok: bool,
    /// Every attachable LOCAL topic, sorted.
    pub topics: Vec<DiscoveredEntry>,
    /// Remote robots (from the `cerulion-netd` catalog gather) + their
    /// attachable topics — the Studio sidebar's ROBOTS section. `#[serde(default,
    /// skip_serializing_if = "Vec::is_empty")]`: an EMPTY list (local-only discover,
    /// or a netd-unreachable degrade) serializes BYTE-IDENTICALLY to the wire without this field.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub robots: Vec<DiscoveredRobot>,
    /// Whether `cerulion-netd`'s discovery had CONVERGED when this gather was
    /// answered — the discriminator between "the LAN was searched and these are the
    /// robots" and "netd has not completed a discovery pass, so an empty or short
    /// `robots` list proves NOTHING".
    ///
    /// Without it a cold-start empty sidebar is indistinguishable on the wire from a
    /// settled one, and a controller has no choice but to render a terminal "no
    /// robots found" for a robot that is right there on the LAN and seconds from being
    /// discovered. That is the discovery marker's whole thesis, applied to the surface the user
    /// actually looks at.
    ///
    /// **No vizd surface waits for convergence** — every control handler answers in one
    /// `cerulion-netd` round trip, because this protocol's own client arms a 5 s reply
    /// deadline (see [`RetryHint`]). So a marker is the ONLY way a controller can tell a
    /// cold sidebar from a settled one: a controller reading
    /// [`DiscoveryLabel::Discovering`] should render "discovering…" and keep refreshing,
    /// never a terminal absence. `discover` is the surface where that costs nothing
    /// anyway — Studio re-asks it, and `subscribe_events` pushes netd's catalog changes to
    /// subscribed controllers, so it heals itself without anybody blocking.
    ///
    /// **ABSENT means UNKNOWN, never a positive claim** — the lesson about
    /// serde defaults. It is absent from an earlier vizd (which never set it) AND
    /// when netd could not be reached at all (a degraded local-only `discover`); both
    /// are cases where nothing can be told, and both must be read the same fail-closed way as
    /// `Discovering`. `#[serde(skip_serializing_if)]` keeps an absent marker
    /// byte-identical to the wire without this field.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub discovery: Option<DiscoveryLabel>,
}

/// `cerulion-netd`'s discovery state, in the vocabulary of the
/// control protocol.
///
/// Deliberately NOT `cerulion_netd::DiscoveryState` re-exported. Two reasons, and the
/// second is the load-bearing one: this module is THE sacred NDJSON contract and does
/// not otherwise speak netd's types; and `DiscoveryState`'s serde `Default` is
/// `Settled` — a POSITIVE assertion — so re-using it would make a decoder's default
/// read "the LAN was searched", which is exactly the false absence the marker exists to
/// refuse. Here the marker is an [`Option`] with no `Default`, so absence can only
/// decode as absence.
///
/// `Discovering` rather than `not_converged`: the wire word is what a controller renders
/// to a person.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DiscoveryLabel {
    /// netd has completed a discovery pass — an empty `robots` list is a real absence.
    Settled,
    /// netd has NOT completed a discovery pass. An empty or short `robots` list proves
    /// nothing; keep refreshing.
    Discovering,
}

impl DiscoveryLabel {
    /// Map netd's own state onto the wire vocabulary. Total — every state has a word.
    pub fn from_state(state: cerulion_netd::protocol::DiscoveryState) -> Self {
        match state {
            cerulion_netd::protocol::DiscoveryState::Settled => Self::Settled,
            cerulion_netd::protocol::DiscoveryState::NotConverged => Self::Discovering,
        }
    }
}

/// One tapped topic's live stats in a [`StatusResponse`].
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct TopicStatus {
    /// The absolute topic.
    pub topic: String,
    /// Publish rate in Hz (wire-sequence delta over a wall window), or `null`
    /// until enough samples exist, **and `null` once the stream has STOPPED.**
    ///
    /// A rate is a claim about NOW. The window ends at the query instant, so a
    /// dead publisher's fixed sequence delta divided by an ever-growing span decays
    /// toward zero without ever reaching it: a
    /// killed publisher would sit at "0.3 Hz" indefinitely. Past
    /// `HZ_STREAMING_RECENCY` (= `LIVENESS_STREAMING_RECENCY_MS`, so "shows a rate"
    /// and "reads streaming" cannot disagree) this is `null` and the row must fall
    /// through to [`Self::liveness_state`].
    ///
    /// A reader MUST therefore treat `null` as "clear the rate", not "keep the last
    /// one" — a frozen last-known rate reads as live, which is the same bug from
    /// the other end.
    pub hz: Option<f64>,
    /// Total frames the daemon has drained from this tap.
    pub frames: u64,
    /// Frames this tap NEVER delivered — pre-decode loss, i.e. wire
    /// sequences the producer committed that the subscriber queue evicted
    /// (`drop_oldest`) before a poll drained them.
    ///
    /// `Some(0)` is the healthy claim "this daemon checked and lost nothing", and
    /// is DIFFERENT from absent, which means "this daemon does not check" (an
    /// earlier vizd). Additive + `skip_serializing_if`, so a daemon that makes
    /// no claim is byte-identical to the earlier wire and a reader must not read the
    /// absence as a zero — the `enumerated` / `mirrors_established` precedent.
    ///
    /// It matters out of proportion to its size on encoded video: one lost access
    /// unit costs every frame that references it, so a ~1.6 % loss here renders as
    /// ~22 % of the stream missing.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub frames_missed: Option<u64>,
    /// The resolved schema qualified name, or `null`.
    pub schema: Option<String>,
    /// The resolved archetype family, or `null`.
    pub archetype: Option<String>,
    /// The Rerun entity path the topic's frames render under.
    pub entity: String,
    /// The rerun component/archetype families the topic renders as, or
    /// `null` when the archetype is unresolved. Mirrors the `schema`/`archetype`
    /// null-not-absent convention: `null` = NOT yet resolved, distinct from a `[]`
    /// "resolved: renders nothing".
    pub components: Option<Vec<String>>,
    /// The resolved rerun view kind(s) the topic renders into, or `null`
    /// when the archetype is unresolved (same convention as `components`).
    ///
    /// **A POINT-IN-TIME reading** — see
    /// [`AttachResponse::view_kinds`]. This is the surface a client should POLL for
    /// it: the value moves as the topic proves it is rendering (the dump
    /// companion going) or degrades (it coming back), with no client action, and
    /// this row carries `representation` beside it for the other cause.
    pub view_kinds: Option<Vec<String>>,
    /// The ORIGIN ROBOT whose data this rate describes, or ABSENT for a
    /// genuine local producer — see [`AttachedEntry::robot`]. The rate is the
    /// reason this matters: an unattributed `hz` on a mirrored topic is what
    /// rendered a live "9.9 Hz" row under the desk's own section beside the
    /// identical row in the robot's.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub robot: Option<String>,
    /// Is a producer still REGISTERED for this topic — see
    /// [`AttachedEntry::producer_count`], the same field on the sibling
    /// attach-state surface. Carried on BOTH so `list` and `status` cannot
    /// disagree about whether a row still exists.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub producer_count: Option<u32>,
    /// This daemon's OWN tap observation — see
    /// [`AttachedEntry::liveness`]. This is the field that replaces the decayed
    /// rate: when [`Self::hz`] is `null` because the stream stopped, this says
    /// whether the topic has ever produced and how stale it is.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub liveness: Option<TopicLiveness>,
    /// [`Self::liveness`] already classified — see
    /// [`AttachedEntry::liveness_state`]. Closed set, absence = UNKNOWN.
    #[serde(default, skip_serializing_if = "liveness_state_is_absent_on_the_wire")]
    pub liveness_state: Option<LivenessState>,
    /// How this topic RENDERS — `auto` / `visual` / `text` / `both`.
    ///
    /// Same field, same meaning, same source of truth as
    /// [`AttachedEntry::representation`]. Carried here because this row's
    /// `view_kinds` are ALREADY computed through the choice, so a row that
    /// reported the consequence without the cause would show view kinds its
    /// archetype alone does not imply and nothing to explain why.
    ///
    /// ABSENT on `auto`, so a desk where nobody has chosen anything stays
    /// BYTE-IDENTICAL on the wire to one without this field.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub representation: Option<String>,
    /// This daemon received frames on this topic and could NOT decode
    /// them — see [`UndecodableReport`]. ABSENT whenever the topic decodes (or
    /// has produced nothing yet), so a healthy desk's wire is byte-identical to
    /// the wire without this field.
    ///
    /// Read it TOGETHER with [`Self::schema`]: a `null` schema plus an absent
    /// report is a topic nothing has been decoded from YET; a `null` schema plus
    /// a report is a topic whose frames this build cannot read, which is not a
    /// waiting state and will not resolve on its own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub undecodable: Option<UndecodableReport>,
    /// Is this topic's tap WAKE-driven (`true`) or paced by the daemon's
    /// poll interval (`false`)?
    ///
    /// This is the row's latency class, and it is the only place the cost/benefit
    /// of the wake is visible from outside: a `true` row is drained within
    /// a wake (MEASURED at ~0.04-0.09 ms median against a 16 ms interval) and
    /// bills its producer one `sendto` per frame PUBLISHED; a `false` row is
    /// drained on the interval and bills nothing.
    ///
    /// "Per frame published" is exact, and the resweep's debt rule is what makes it true.
    /// A listener on a GRAPH publisher's event service also arms the publisher's
    /// live-loop boundary resweep; a resweep that fires a real `SentSample` on every
    /// `live_step` pass in which the producer has NOT published sets the bill
    /// by the graph's LOOP rate, not the topic's frame rate (MEASURED without the
    /// debt rule: 198 notifies for 2 frames; 200 for 6 in this repo's own
    /// e2e), and the notify reaches the topic's own in-graph consumer listener on
    /// the live WaitSet, free-running the graph's loop (measured 53/s → 3425/s).
    /// The resweep announces an UN-ANNOUNCED FRAME rather than observing a
    /// silent pass, so a quiescent producer costs zero.
    ///
    /// It is not merely the attach class restated. A topic ASKS for a wake and can
    /// still read `false` — the listener could not be opened (an exhausted event
    /// service), or the wake was DEMOTED because it kept firing without
    /// delivering. Both are states an operator asking "why is this topic slower
    /// than that one?" needs to be able to see.
    ///
    /// **Local topics ask too**, so every row a
    /// current daemon serves carries this field. Absent therefore means the
    /// daemon predates this field — not that the topic is timer-paced.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub wake: Option<bool>,
}

/// The never-block worker's global counters in a [`StatusResponse`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct WorkerStatus {
    /// Viz frames dropped because the viewer wedged (never blocks the daemon).
    pub dropped_frames: u64,
    /// Per-tick batches dropped (companion to `dropped_frames`).
    pub dropped_batches: u64,
    /// Frames coalesced away by the newest-per-tick rendering.
    pub coalesced_frames: u64,
    /// Live gRPC reconnects performed after a detected viewer disconnect.
    pub reconnects: u64,
}

/// `status` result payload.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct StatusResponse {
    /// Correlation id.
    pub id: Option<u64>,
    /// Always `true`.
    pub ok: bool,
    /// Per-topic rate + frame counts, sorted by topic.
    pub topics: Vec<TopicStatus>,
    /// The worker's global drop/coalesce/reconnect counters.
    pub worker: WorkerStatus,
}

/// `schemas` success payload — what the side-load actually changed, so a
/// caller can tell "the daemon learned something" from "it already knew all of
/// this" without guessing.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct SchemasResponse {
    /// Correlation id.
    pub id: Option<u64>,
    /// Always `true`.
    pub ok: bool,
    /// How many docs the request carried.
    pub offered: u32,
    /// How many were NEW to this daemon (byte-different from, or absent from,
    /// what it already held). `0` means the walker was left untouched — the
    /// idempotent re-send case.
    pub accepted: u32,
    /// The total number of side-loaded docs the daemon now holds.
    pub known: u32,
    /// Types whose definition this offer REPLACED — the daemon held a different
    /// one for that name. Destructive rather than additive: the walker keys
    /// layouts by qualified name, so the previous definition's wire hash stops
    /// resolving and anything still producing it goes dark. Reported so a caller
    /// can say so instead of announcing a plain success.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub replaced: Vec<String>,
    /// Types the daemon did NOT take: a definition for a BUILT-IN (refused — a
    /// side-load applies daemon-wide), a workspace-YAML doc (no parser here), or
    /// a `.msg` that would not parse. Those types stay UNDECODABLE, and saying so
    /// is what keeps `accepted` from being a success-shaped lie.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub rejected: Vec<String>,
}

/// `set_blueprint` success payload. The blueprint was validated + handed to the
/// worker (NOT a render acknowledgement — the daemon never blocks on rendering).
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct SetBlueprintResponse {
    /// Correlation id.
    pub id: Option<u64>,
    /// Always `true`.
    pub ok: bool,
    /// The number of leaf VIEWS in the applied layout (the Go2 default's 3 on a
    /// reset).
    pub views: u32,
    /// `true` when this reset to the built-in Go2 default (a `null`/absent
    /// `layout`), `false` when a client layout was applied.
    pub reset: bool,
    /// Soft, non-fatal hints — view origins that match NO currently-attached
    /// topic (an agent may lay out BEFORE attaching, so this is never an error).
    /// Always present (an empty array when there are none) — a stable shape for
    /// Studio.
    pub warnings: Vec<String>,
}

/// One PLACEMENT in a [`ComposeLayoutResponse`] — where a topic was
/// placed. A dual-view topic (Odometry / Imu) yields ONE placement PER view it
/// lands in, so a placement is always `(topic, view)`-unique.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Placement {
    /// The absolute topic.
    pub topic: String,
    /// The render entity path the topic's frames log under.
    pub entity: String,
    /// The resolved archetype family, or `null` for a still-silent topic placed by
    /// its declared role.
    pub archetype: Option<String>,
    /// The view kind the topic was placed in (`"spatial3d"` | `"spatial2d"` |
    /// `"time_series"` | `"text_document"`).
    pub view: String,
    /// The rerun component/archetype families this placement renders, or `null`
    /// when the archetype is unresolved (same null-not-absent convention as
    /// `archetype`).
    pub components: Option<Vec<String>>,
    /// `true` iff this is an EXPLICIT (role-tagged) placement of a
    /// registered-but-DEAD route (its `producer_count` is `Some(0)` — nothing
    /// publishing). The user (agent) named it directly, so user intent WINS and it IS
    /// placed, but the flag lets Studio warn that the pane stays empty until the topic
    /// publishes. AUTOMAGIC-path dead topics are skipped (see
    /// [`ComposeLayoutResponse::skipped_dead`]), never placed, so this is always
    /// `false` for them. `#[serde(default, skip_serializing_if)]` on the `false`
    /// default, so a live/unknown placement serializes BYTE-IDENTICALLY to the
    /// wire without this field (additive; a lenient decoder ignores it when absent).
    #[serde(default, skip_serializing_if = "is_false")]
    pub dead: bool,
}

/// One topic the AUTOMAGIC (role-less) compose path SKIPPED because it is a
/// registered-but-DEAD route — surfaced in [`ComposeLayoutResponse::skipped_dead`] so
/// the agent can tell the user which topics were left out and why (never silent).
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct SkippedDead {
    /// The absolute topic that was skipped.
    pub topic: String,
    /// A human-readable reason (e.g. "never published — 0 producers").
    pub reason: String,
}

/// `#[serde(skip_serializing_if)]` predicate for a `false`-default `bool` (keeps the
/// wire byte-identical to one without the field when the flag is not set).
fn is_false(b: &bool) -> bool {
    !*b
}

/// `compose_layout` success payload. The blueprint was compiled + validated +
/// handed to the worker (NOT a render acknowledgement — the daemon never blocks on
/// rendering). Echoes the placements + the applied arrangement so the agent SEES
/// exactly what got laid out, without inspecting rerun internals.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ComposeLayoutResponse {
    /// Correlation id.
    pub id: Option<u64>,
    /// Always `true`.
    pub ok: bool,
    /// One entry per `(topic, view)` placement, in placement order.
    pub placements: Vec<Placement>,
    /// The applied arrangement shape (`"hero_sidebar"` | `"grid"` | `"single"`).
    pub arrangement: String,
    /// The number of leaf VIEWS in the compiled layout.
    pub views: u32,
    /// The topics this call NEWLY attached (`attach_missing`), sorted. Empty when
    /// nothing was attached. Always present (a stable shape for Studio).
    pub attached: Vec<String>,
    /// Soft, non-fatal hints (a silent topic placed by role, a silent role-less
    /// topic that could not be placed, an invisible-sibling spatial2d hint).
    /// Always present (an empty array when there are none).
    pub warnings: Vec<String>,
    /// Topics the AUTOMAGIC (role-less) path SKIPPED because they are
    /// registered-but-DEAD routes (`producer_count == Some(0)`), each with a human
    /// reason — so a dead route never silently consumes a hero/sidebar slot and the
    /// agent can mention the skipped topics to the user. Empty when nothing was
    /// skipped; `#[serde(default, skip_serializing_if = "Vec::is_empty")]`, so a
    /// no-dead-topics compose serializes BYTE-IDENTICALLY to the wire without this field
    /// (additive; a lenient decoder ignores it when absent). An EXPLICIT (role-tagged)
    /// dead topic is NOT here — it is placed with [`Placement::dead`] set instead.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub skipped_dead: Vec<SkippedDead>,
}

/// Error payload (`ok:false`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ErrorResponse {
    /// Correlation id (`null` when the request was unparseable).
    pub id: Option<u64>,
    /// Always `false`.
    pub ok: bool,
    /// The human-readable, actionable error (surfaced VERBATIM from the tap
    /// layer for attach failures — e.g. "topic does not exist", "no free
    /// introspection slot").
    pub error: String,
    /// The offending topic when the error concerns one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub topic: Option<String>,
    /// PRESENT if and only if **re-asking can change this answer** — see
    /// [`RetryHint`]. Absent means TERMINAL: a bad topic name, a tap failure, a
    /// schema conflict, the `CERULION_NETWORK=off` kill switch, or any older
    /// daemon. Absent is the fail-closed reading, so an old client (and every
    /// existing error path) behaves exactly as it did.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retry: Option<RetryHint>,
}

/// The structured "this is a NOT-FOUND-**YET**" discriminator on an attach
/// failure, and everything a client needs to run the shipped first-contact policy.
///
/// # Why this exists, and why it is on the ERROR
///
/// vizd answers the **closed-world** question — "does this LAN serve the topic
/// *now*?" — and answers it PROMPTLY, in one netd round trip. It deliberately does
/// not block on the **open-world** question — "*will* it?" — because that one has no
/// bounded answer, and the party that knows how long a human is willing to wait is
/// the client, not the daemon. (The daemon also cannot afford to: the only in-repo
/// vizd control client arms a 5 s `SO_RCVTIMEO` on every reply read, so a daemon-side
/// wait past ~5 s does not slow the verb down — it kills it.)
///
/// So the daemon hands the client the facts and lets it decide. Presence of this hint
/// is the discriminator a controller could otherwise only get by string-matching
/// prose the daemon is free to reword.
///
/// # It is minted from the SEAM's question, not the gather's emptiness
///
/// A hint is emitted when *this topic / this robot* was not found in a discovery
/// answer — never from "the gather came back empty". Those differ exactly where it
/// matters: on a two-robot LAN where robot A has answered and robot B is still
/// booting, the gather is NON-empty and netd reports `Settled` (its `ever_settled`
/// bit latches permanently on the first non-empty gather), yet an attach for one of
/// B's topics found nothing. That answer is a snapshot, not a verdict about the
/// future, and it carries a hint saying so.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct RetryHint {
    /// How much `cerulion-netd` had actually read when it answered.
    ///
    /// [`DiscoveryLabel::Discovering`] — netd has never gathered anything from the
    /// network, so this answer proves NOTHING (an explicit UNKNOWN) and a client
    /// should keep asking through convergence. [`DiscoveryLabel::Settled`] — netd's
    /// discovery demonstrably works here and the LAN was searched, so the answer is
    /// authoritative *about right now*; a client with its own cadence (a sidebar that
    /// polls anyway) may still re-ask, but a one-shot command should not burn a
    /// budget on it.
    pub discovery: DiscoveryLabel,
    /// How long netd's query plane has been running without ever settling, in
    /// milliseconds — the first-contact cap, forwarded verbatim.
    ///
    /// `null` when the plane has settled, or when the answering daemon is too old to
    /// report it (an earlier release). `null` is UNKNOWN and caps nothing, exactly as
    /// `ConvergenceWait::decide` reads it — the same rule on both sides of the wire.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub plane_unsettled_ms: Option<u64>,
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Request parsing (hand-written LINES → the EXACT enum value) ──────────

    #[test]
    fn parse_discover_list_status_lines() {
        assert_eq!(
            parse_request(r#"{"id":1,"method":"discover"}"#).unwrap(),
            Request::Discover { id: 1 }
        );
        assert_eq!(
            parse_request(r#"{"id":4,"method":"list"}"#).unwrap(),
            Request::List { id: 4 }
        );
        assert_eq!(
            parse_request(r#"{"id":5,"method":"status"}"#).unwrap(),
            Request::Status { id: 5 }
        );
    }

    #[test]
    fn parse_attach_minimal_and_full() {
        // Minimal: no entity/robot/schema → all None.
        assert_eq!(
            parse_request(r#"{"id":2,"method":"attach","topic":"/utlidar/cloud"}"#).unwrap(),
            Request::Attach {
                id: 2,
                topic: "/utlidar/cloud".to_string(),
                entity: None,
                robot: None,
                schema: None,
            }
        );
        // Full: entity override + the remote `robot` + its required `schema` all
        // parse.
        assert_eq!(
            parse_request(
                r#"{"id":9,"method":"attach","topic":"/x","entity":"world/cam","robot":"go2","schema":"geometry_msgs/Vector3"}"#
            )
            .unwrap(),
            Request::Attach {
                id: 9,
                topic: "/x".to_string(),
                entity: Some("world/cam".to_string()),
                robot: Some("go2".to_string()),
                schema: Some("geometry_msgs/Vector3".to_string()),
            }
        );
    }

    #[test]
    fn parse_detach_line() {
        assert_eq!(
            parse_request(r#"{"id":3,"method":"detach","topic":"/utlidar/cloud"}"#).unwrap(),
            Request::Detach {
                id: 3,
                topic: "/utlidar/cloud".to_string(),
            }
        );
    }

    /// The EXACT line Studio's affordance sends, parsed by the shipped
    /// parser — the both-sides pin that keeps the verb from shipping inert.
    ///
    /// The studio half asserts the same three keys and the same four values are
    /// what its row BUILDS (`test/sidebar-representation.test.ts`). Two literal
    /// JSON lines, one on each side of the socket: if either drifts, one of them
    /// fails.
    #[test]
    fn parse_representation_line() {
        assert_eq!(
            parse_request(
                r#"{"id":6,"method":"representation","topic":"/imu","representation":"both"}"#
            )
            .unwrap(),
            Request::Representation {
                id: 6,
                topic: "/imu".to_string(),
                representation: "both".to_string(),
            }
        );
        // Every value the vocabulary admits parses as itself — the daemon, not the
        // parser, is what refuses an unknown one (with a message naming the set),
        // so a typo cannot arrive here as a silent `auto`.
        for value in ["auto", "visual", "text", "both"] {
            let line = format!(
                r#"{{"id":6,"method":"representation","topic":"/t","representation":"{value}"}}"#
            );
            assert_eq!(
                parse_request(&line).unwrap(),
                Request::Representation {
                    id: 6,
                    topic: "/t".to_string(),
                    representation: value.to_string(),
                }
            );
        }
        // A missing `representation` is a PARSE error, never a defaulted one.
        assert!(parse_request(r#"{"id":6,"method":"representation","topic":"/t"}"#).is_err());
    }

    /// The reply's exact wire line — the row renders its state from this echo, so
    /// the key set and their order are the contract.
    #[test]
    fn representation_response_is_the_hand_oracle_line() {
        assert_eq!(
            serde_json::to_string(&RepresentationResponse {
                id: Some(6),
                ok: true,
                topic: "/imu".to_string(),
                representation: "both".to_string(),
                applied: true,
            })
            .unwrap(),
            r#"{"id":6,"ok":true,"topic":"/imu","representation":"both","applied":true}"#
        );
    }

    #[test]
    fn unknown_fields_are_ignored_forward_compatible() {
        // A future Studio adds a field the running daemon predates → IGNORED,
        // never a parse failure (no deny_unknown_fields).
        assert_eq!(
            parse_request(r#"{"id":1,"method":"discover","future_knob":true,"note":"x"}"#).unwrap(),
            Request::Discover { id: 1 }
        );
    }

    // ── Malformed / unknown method → RequestError (NEVER a panic) ────────────

    #[test]
    fn not_json_is_a_request_error_not_a_panic() {
        let err = parse_request("this is not json").unwrap_err();
        // No numeric id recoverable from garbage.
        assert_eq!(err.id, None);
        assert!(!err.message.is_empty());
    }

    #[test]
    fn unknown_method_is_a_request_error_with_recovered_id() {
        // Unknown method → parse fails, but the id is still recovered for
        // correlation (the daemon answers a structured error, not a disconnect).
        let err = parse_request(r#"{"id":7,"method":"teleport"}"#).unwrap_err();
        assert_eq!(err.id, Some(7));
        // serde's internally-tagged enum NAMES the offending method in its error
        // (`unknown variant `teleport`, expected one of ...`), so `contains` is
        // load-bearing here — no `|| !is_empty()` escape (which made it vacuous):
        // the message must actually surface WHICH method was unknown (the UX bar
        // for a loud, actionable error).
        assert!(
            err.message.contains("teleport"),
            "the unknown-method error must name the method: {}",
            err.message
        );
    }

    #[test]
    fn missing_method_is_a_request_error_with_recovered_id() {
        let err = parse_request(r#"{"id":8}"#).unwrap_err();
        assert_eq!(err.id, Some(8));
    }

    #[test]
    fn missing_id_is_a_request_error() {
        // A request without an id cannot be correlated → id None, structured err.
        let err = parse_request(r#"{"method":"discover"}"#).unwrap_err();
        assert_eq!(err.id, None);
    }

    // ── Response serialization → EXACT hand-oracle JSON lines ────────────────

    #[test]
    fn hello_banner_carries_the_version_and_hosted_rerun_url() {
        // The hosted default: the banner advertises the daemon's bound proxy URL
        // (field ORDER is declaration order — vizd, protocol, rerun_url).
        assert_eq!(
            Hello::new(Some("rerun+http://127.0.0.1:9876/proxy".to_string())).to_json_line(),
            r#"{"vizd":"cerulion-vizd","protocol":1,"rerun_url":"rerun+http://127.0.0.1:9876/proxy"}"#
        );
        assert_eq!(Hello::new(Some("x".to_string())).protocol, PROTOCOL_VERSION);
    }

    #[test]
    fn hello_banner_with_viz_disabled_serializes_null_not_absent() {
        // Viz disabled (no endpoint hosted/configured) → rerun_url is present but
        // null (a stable shape for Studio), NEVER the field omitted.
        assert_eq!(
            Hello::new(None).to_json_line(),
            r#"{"vizd":"cerulion-vizd","protocol":1,"rerun_url":null}"#
        );
    }

    #[test]
    fn attach_success_serializes_to_the_exact_line() {
        let resp = Response::Attach(AttachResponse {
            id: Some(2),
            ok: true,
            topic: "/utlidar/cloud".to_string(),
            schema: Some("sensor_msgs/PointCloud2".to_string()),
            archetype: Some("Points3D".to_string()),
            entity: "world/utlidar/cloud".to_string(),
            route: "utlidar/cloud".to_string(),
            already_attached: false,
            components: Some(vec!["Points3D".to_string()]),
            view_kinds: Some(vec!["spatial3d".to_string()]),
            robot: None,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":2,"ok":true,"topic":"/utlidar/cloud","schema":"sensor_msgs/PointCloud2","archetype":"Points3D","entity":"world/utlidar/cloud","route":"utlidar/cloud","already_attached":false,"components":["Points3D"],"view_kinds":["spatial3d"]}"#
        );
    }

    /// A REMOTE attach's reply names the robot on the same wire field the
    /// other three surfaces use, so the client places the row without inferring it
    /// from the request it happened to send. Hand oracle.
    #[test]
    fn attach_success_carries_the_origin_robot_for_a_remote_attach() {
        let resp = Response::Attach(AttachResponse {
            id: Some(2),
            ok: true,
            topic: "/utlidar/cloud".to_string(),
            schema: Some("sensor_msgs/PointCloud2".to_string()),
            archetype: Some("Points3D".to_string()),
            entity: "world/utlidar/cloud".to_string(),
            route: "utlidar/cloud".to_string(),
            already_attached: false,
            components: Some(vec!["Points3D".to_string()]),
            view_kinds: Some(vec!["spatial3d".to_string()]),
            robot: Some("ubuntu".to_string()),
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":2,"ok":true,"topic":"/utlidar/cloud","schema":"sensor_msgs/PointCloud2","archetype":"Points3D","entity":"world/utlidar/cloud","route":"utlidar/cloud","already_attached":false,"components":["Points3D"],"view_kinds":["spatial3d"],"robot":"ubuntu"}"#
        );
    }

    #[test]
    fn attach_unresolved_schema_serializes_null_not_absent() {
        // A silent topic attaches with schema/archetype = null (stable shape for
        // Studio), NOT the fields omitted.
        let resp = Response::Attach(AttachResponse {
            id: Some(2),
            ok: true,
            topic: "/x".to_string(),
            schema: None,
            archetype: None,
            entity: "world/x".to_string(),
            route: "x".to_string(),
            already_attached: false,
            // An unresolved archetype yields `null` component/view
            // fields (NOT `[]`) — matching the schema/archetype null-not-absent
            // convention. `null` = not yet resolved; a `[]` would mean "resolved:
            // renders nothing", a distinction an agent reads as authoritative.
            components: None,
            view_kinds: None,
            robot: None,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":2,"ok":true,"topic":"/x","schema":null,"archetype":null,"entity":"world/x","route":"x","already_attached":false,"components":null,"view_kinds":null}"#
        );
    }

    #[test]
    fn error_serializes_to_the_exact_line() {
        let resp = Response::error(
            Some(2),
            "no free introspection slot (4/4 taps) — stop a tool",
            Some("/x".to_string()),
        );
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":2,"ok":false,"error":"no free introspection slot (4/4 taps) — stop a tool","topic":"/x"}"#
        );
    }

    #[test]
    fn error_without_topic_omits_the_topic_field() {
        // A malformed-line error carries no topic → the field is omitted (id is
        // null when uncorrelatable).
        let resp = Response::error(None, "malformed request", None);
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":null,"ok":false,"error":"malformed request"}"#
        );
    }

    #[test]
    fn detach_serializes_to_the_exact_line() {
        let resp = Response::Detach(DetachResponse {
            id: Some(3),
            ok: true,
            topic: "/tf".to_string(),
            detached: true,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":3,"ok":true,"topic":"/tf","detached":true}"#
        );
    }

    /// A genuine-LOCAL attached tap (`robot: None`) serializes BYTE-IDENTICALLY to
    /// the earlier wire — the back-compat half of adding the attribution.
    #[test]
    fn list_serializes_to_the_exact_line() {
        let resp = Response::List(ListResponse {
            id: Some(4),
            ok: true,
            attached: vec![AttachedEntry {
                topic: "/tf".to_string(),
                route: "tf".to_string(),
                entity: "world".to_string(),
                schema: Some("tf2_msgs/TFMessage".to_string()),
                archetype: Some("Transforms".to_string()),
                // `list` carries the SAME deterministic placement.
                components: Some(vec!["Transform3D".to_string()]),
                view_kinds: Some(vec!["spatial3d".to_string()]),
                robot: None,
                // An un-probed / un-observed row keeps the earlier wire.
                producer_count: None,
                liveness: None,
                liveness_state: None,
                representation: None,
                undecodable: None,
            }],
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":4,"ok":true,"attached":[{"topic":"/tf","route":"tf","entity":"world","schema":"tf2_msgs/TFMessage","archetype":"Transforms","components":["Transform3D"],"view_kinds":["spatial3d"]}]}"#
        );
    }

    /// An attached MIRROR carries its origin robot on the SAME wire field
    /// name `discover` uses (`robot`), so the sidebar can put the attach state on
    /// the robot's row instead of minting a phantom local one. Hand oracle.
    #[test]
    fn list_carries_the_origin_robot_for_an_attached_mirror() {
        let resp = Response::List(ListResponse {
            id: Some(4),
            ok: true,
            attached: vec![AttachedEntry {
                topic: "/tf".to_string(),
                route: "tf".to_string(),
                entity: "world".to_string(),
                schema: Some("tf2_msgs/TFMessage".to_string()),
                archetype: Some("Transforms".to_string()),
                components: Some(vec!["Transform3D".to_string()]),
                view_kinds: Some(vec!["spatial3d".to_string()]),
                robot: Some("ubuntu".to_string()),
                producer_count: None,
                liveness: None,
                liveness_state: None,
                representation: None,
                undecodable: None,
            }],
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":4,"ok":true,"attached":[{"topic":"/tf","route":"tf","entity":"world","schema":"tf2_msgs/TFMessage","archetype":"Transforms","components":["Transform3D"],"view_kinds":["spatial3d"],"robot":"ubuntu"}]}"#
        );
    }

    /// A genuine-LOCAL tap's status is BYTE-IDENTICAL to the earlier wire.
    #[test]
    fn status_serializes_to_the_exact_line() {
        let resp = Response::Status(StatusResponse {
            id: Some(5),
            ok: true,
            topics: vec![TopicStatus {
                topic: "/vel".to_string(),
                hz: Some(30.0),
                frames: 90,
                // Absent from the wire, so these byte-exact oracles are
                // unchanged — the back-compat property, asserted rather than claimed.
                frames_missed: None,
                schema: Some("geometry_msgs/Vector3".to_string()),
                archetype: Some("Scalars".to_string()),
                entity: "world/vel".to_string(),
                components: Some(vec!["Scalars".to_string()]),
                view_kinds: Some(vec!["time_series".to_string()]),
                robot: None,
                producer_count: None,
                liveness: None,
                liveness_state: None,
                representation: None,
                undecodable: None,
                wake: None,
            }],
            worker: WorkerStatus {
                dropped_frames: 0,
                dropped_batches: 0,
                coalesced_frames: 2,
                reconnects: 0,
            },
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":5,"ok":true,"topics":[{"topic":"/vel","hz":30.0,"frames":90,"schema":"geometry_msgs/Vector3","archetype":"Scalars","entity":"world/vel","components":["Scalars"],"view_kinds":["time_series"]}],"worker":{"dropped_frames":0,"dropped_batches":0,"coalesced_frames":2,"reconnects":0}}"#
        );
    }

    /// A STOPPED stream serializes `"hz":null` and carries the verdict
    /// that replaces the rate — the stalled-stream row, on the wire.
    ///
    /// **What the sidebar renders for this exact row, and the fork to know
    /// about.** The requirement is a red dot and red "no data" on a
    /// topic that is registered but not publishing. This row cannot be given that,
    /// and the reason is structural rather than a policy choice: `producer_count`
    /// here is `Some(1)` — as it is on a real robot — because `cerulion-netd`'s
    /// re-injector holds the desk-local publisher slot for as long as anything
    /// demands the mirror, and on a `ros2 attach` robot the catalog count reads 1 for
    /// a dead route and a streaming one alike (the reason the liveness observer exists).
    /// `liveness_state` is `idle` and not `no_data` for an equally structural reason:
    /// the topic demonstrably produced frames, and the liveness rule forbids libeling a
    /// produced topic as never-produced.
    ///
    /// So there is no signal on this row that says "nothing is publishing" — only
    /// "nothing has arrived for 120 s", which is exactly what the sidebar renders: a
    /// MUTED "last seen 2m ago" that visibly ages, with no live dot and no rate. It
    /// LOOKS dead without asserting something that cannot be known; red is reserved for
    /// `no_data`, where nothing has ever arrived.
    ///
    /// That case still resolves, by the other route: a serve-swap makes the
    /// robot's catalog stop listing the topic, and the row then DISAPPEARS (the
    /// unregistered path — see `fold_streaming_mirrors_into_robots`, pinned by
    /// `a_swap_killed_mirror_stays_while_the_catalog_lists_it_then_leaves_when_it_does_not`).
    /// Stale is the correct TRANSITIONAL rendering while the robot still advertises
    /// the topic.
    ///
    /// Hand oracle over the exact line, so all three claims are pinned at once:
    /// `hz` is `null` (NOT a small decayed number — the un-pruned shape), the
    /// registration probe rides along as `producer_count`, and the classification
    /// is the CLOSED liveness string (`"idle"` here: the topic HAS produced, so it
    /// is never the dimmed `"no_data"`, and the dated age says how stale).
    #[test]
    fn status_serializes_the_liveness_verdict_when_the_stream_has_stopped() {
        let resp = Response::Status(StatusResponse {
            id: Some(6),
            ok: true,
            topics: vec![TopicStatus {
                topic: "/go2/camera/jpeg".to_string(),
                hz: None,
                frames: 4200,
                // Absent from the wire, so these byte-exact oracles are
                // unchanged — the back-compat property, asserted rather than claimed.
                frames_missed: None,
                schema: Some("sensor_msgs/CompressedImage".to_string()),
                archetype: Some("EncodedImage".to_string()),
                entity: "world/jpeg".to_string(),
                components: Some(vec!["EncodedImage".to_string()]),
                view_kinds: Some(vec!["spatial2d".to_string()]),
                robot: None,
                producer_count: Some(1),
                liveness: Some(TopicLiveness {
                    last_frame_age_ms: Some(120_000),
                    observed_for_ms: 600_000,
                    frames_observed: 4200,
                    // The rate estimate is additive + absent-when-`None`, so this earlier wire
                    // oracle stays byte-identical.
                    rate_estimate: None,
                }),
                liveness_state: Some(LivenessState::Idle),
                representation: None,
                undecodable: None,
                wake: None,
            }],
            worker: WorkerStatus {
                dropped_frames: 0,
                dropped_batches: 0,
                coalesced_frames: 0,
                reconnects: 0,
            },
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":6,"ok":true,"topics":[{"topic":"/go2/camera/jpeg","hz":null,"frames":4200,"schema":"sensor_msgs/CompressedImage","archetype":"EncodedImage","entity":"world/jpeg","components":["EncodedImage"],"view_kinds":["spatial2d"],"producer_count":1,"liveness":{"last_frame_age_ms":120000,"observed_for_ms":600000,"frames_observed":4200},"liveness_state":"idle"}],"worker":{"dropped_frames":0,"dropped_batches":0,"coalesced_frames":0,"reconnects":0}}"#
        );
    }

    /// `list` carries the SAME registration + liveness truth `status`
    /// does, so the two attach-state surfaces cannot disagree about one row (two
    /// surfaces deriving one answer separately is how they
    /// come to differ).
    ///
    /// This row is the UNREGISTERED one: `producer_count: 0` with the service
    /// still enumerable (this daemon's own tap holds it open), which is exactly
    /// why service enumeration cannot answer the question and a producer probe
    /// must. `"no_data"` is the ONLY state that dims: nothing has ever arrived.
    #[test]
    fn list_carries_the_registration_and_liveness_truth_for_a_dead_row() {
        let resp = Response::List(ListResponse {
            id: Some(7),
            ok: true,
            attached: vec![AttachedEntry {
                topic: "/uslam/cloud_map".to_string(),
                route: "cloud_map".to_string(),
                entity: "world/cloud_map".to_string(),
                schema: None,
                archetype: None,
                components: None,
                view_kinds: None,
                robot: None,
                producer_count: Some(0),
                liveness: Some(TopicLiveness {
                    last_frame_age_ms: None,
                    observed_for_ms: 45_000,
                    frames_observed: 0,
                    rate_estimate: None,
                }),
                liveness_state: Some(LivenessState::NoData),
                representation: None,
                undecodable: None,
            }],
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":7,"ok":true,"attached":[{"topic":"/uslam/cloud_map","route":"cloud_map","entity":"world/cloud_map","schema":null,"archetype":null,"components":null,"view_kinds":null,"producer_count":0,"liveness":{"last_frame_age_ms":null,"observed_for_ms":45000,"frames_observed":0},"liveness_state":"no_data"}]}"#
        );
    }

    /// UNKNOWN has exactly ONE encoding on the attach-state surfaces too
    /// — ABSENCE. An explicitly-`Unknown` classification must not reach the wire as
    /// a `"unknown"` string that every reader would then have to handle alongside
    /// the missing key.
    ///
    /// Drives BOTH new surfaces through the shared skip predicate (the sibling
    /// `DiscoveredEntry` arm has its own pin), and pairs each with a `liveness`
    /// payload that IS present — so this cannot pass by dropping the whole object.
    #[test]
    fn an_unknown_classification_is_absent_on_both_attach_state_surfaces() {
        let unknown = TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms: 500,
            frames_observed: 0,
            rate_estimate: None,
        };
        assert_eq!(unknown.state(), LivenessState::Unknown);

        let listed = Response::List(ListResponse {
            id: Some(8),
            ok: true,
            attached: vec![AttachedEntry {
                topic: "/x".to_string(),
                route: "x".to_string(),
                entity: "world/x".to_string(),
                schema: None,
                archetype: None,
                components: None,
                view_kinds: None,
                robot: None,
                producer_count: Some(1),
                liveness: Some(unknown),
                liveness_state: Some(LivenessState::Unknown),
                representation: None,
                undecodable: None,
            }],
        })
        .to_json_line();
        assert!(
            !listed.contains("liveness_state"),
            "an Unknown classification must be ABSENT, not spelled: {listed}"
        );
        assert!(
            listed.contains(r#""liveness":{"last_frame_age_ms":null"#),
            "the raw observation still rides along: {listed}"
        );

        let statused = Response::Status(StatusResponse {
            id: Some(8),
            ok: true,
            topics: vec![TopicStatus {
                topic: "/x".to_string(),
                hz: None,
                frames: 0,
                // Absent from the wire, so these byte-exact oracles are
                // unchanged — the back-compat property, asserted rather than claimed.
                frames_missed: None,
                schema: None,
                archetype: None,
                entity: "world/x".to_string(),
                components: None,
                view_kinds: None,
                robot: None,
                producer_count: Some(1),
                liveness: Some(unknown),
                liveness_state: Some(LivenessState::Unknown),
                representation: None,
                undecodable: None,
                wake: None,
            }],
            worker: WorkerStatus {
                dropped_frames: 0,
                dropped_batches: 0,
                coalesced_frames: 0,
                reconnects: 0,
            },
        })
        .to_json_line();
        assert!(
            !statused.contains("liveness_state"),
            "an Unknown classification must be ABSENT, not spelled: {statused}"
        );
    }

    /// A mirrored topic's RATE is attributed to the robot it came from —
    /// the live symptom was an unattributed `9.9 Hz` row under the desk's own
    /// section, identical to the one in the robot's. Hand oracle.
    #[test]
    fn status_carries_the_origin_robot_beside_the_rate_of_a_mirror() {
        let resp = Response::Status(StatusResponse {
            id: Some(5),
            ok: true,
            topics: vec![TopicStatus {
                topic: "/tf".to_string(),
                hz: Some(9.9),
                frames: 99,
                // Absent from the wire, so these byte-exact oracles are
                // unchanged — the back-compat property, asserted rather than claimed.
                frames_missed: None,
                schema: Some("tf2_msgs/TFMessage".to_string()),
                archetype: Some("Transforms".to_string()),
                entity: "world".to_string(),
                components: Some(vec!["Transform3D".to_string()]),
                view_kinds: Some(vec!["spatial3d".to_string()]),
                robot: Some("ubuntu".to_string()),
                producer_count: None,
                liveness: None,
                liveness_state: None,
                representation: None,
                undecodable: None,
                wake: None,
            }],
            worker: WorkerStatus {
                dropped_frames: 0,
                dropped_batches: 0,
                coalesced_frames: 0,
                reconnects: 0,
            },
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":5,"ok":true,"topics":[{"topic":"/tf","hz":9.9,"frames":99,"schema":"tf2_msgs/TFMessage","archetype":"Transforms","entity":"world","components":["Transform3D"],"view_kinds":["spatial3d"],"robot":"ubuntu"}],"worker":{"dropped_frames":0,"dropped_batches":0,"coalesced_frames":0,"reconnects":0}}"#
        );
    }

    #[test]
    fn discover_serializes_to_the_exact_line() {
        let resp = Response::Discover(DiscoverResponse {
            id: Some(1),
            ok: true,
            topics: vec![DiscoveredEntry {
                topic: "/vel".to_string(),
                schema: Some("geometry_msgs/Vector3".to_string()),
                archetype: Some("Scalars".to_string()),
                entity: "world/vel".to_string(),
                components: Some(vec!["Scalars".to_string()]),
                view_kinds: Some(vec!["time_series".to_string()]),
                // A LOCAL topic carries no robot → skip-serialized.
                robot: None,
                producer_count: None,
                liveness: None,
                liveness_state: None,
                representation: None,
                undecodable: None,
            }],
            // No robots → skip-serialized. The line is BYTE-IDENTICAL to
            // the wire without either field (the anti-regression contract for both new fields).
            robots: vec![],
            discovery: None,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":1,"ok":true,"topics":[{"topic":"/vel","schema":"geometry_msgs/Vector3","archetype":"Scalars","entity":"world/vel","components":["Scalars"],"view_kinds":["time_series"]}]}"#
        );
    }

    #[test]
    fn discover_unresolved_serializes_components_and_view_kinds_as_null_not_empty() {
        // A silent/undecodable discovered topic has schema/archetype
        // = null AND components/view_kinds = null (NOT `[]`). `null` = not yet
        // resolved; a `[]` would read as "resolved: renders nothing" — a distinction
        // the layout agent treats as authoritative, so an unresolved topic must NOT
        // masquerade as one with a known-empty view set.
        let resp = Response::Discover(DiscoverResponse {
            id: Some(1),
            ok: true,
            topics: vec![DiscoveredEntry {
                topic: "/silent".to_string(),
                schema: None,
                archetype: None,
                entity: "world/silent".to_string(),
                components: None,
                view_kinds: None,
                robot: None,
                producer_count: None,
                liveness: None,
                liveness_state: None,
                representation: None,
                undecodable: None,
            }],
            robots: vec![],
            discovery: None,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":1,"ok":true,"topics":[{"topic":"/silent","schema":null,"archetype":null,"entity":"world/silent","components":null,"view_kinds":null}]}"#
        );
    }

    #[test]
    fn discover_with_robots_serializes_to_the_exact_line() {
        // A discover carrying a REMOTE robot section. The local topic is
        // unchanged; the robot's entry carries `robot: Some(..)` (for attach) and its
        // schema/archetype hints from the served catalog. Hand oracle on the exact bytes.
        let resp = Response::Discover(DiscoverResponse {
            id: Some(7),
            ok: true,
            topics: vec![DiscoveredEntry {
                topic: "/cloud".to_string(),
                schema: None,
                archetype: None,
                entity: "world/cloud".to_string(),
                components: None,
                view_kinds: None,
                robot: None,
                producer_count: None,
                liveness: None,
                liveness_state: None,
                representation: None,
                undecodable: None,
            }],
            robots: vec![DiscoveredRobot {
                robot: "ubuntu".to_string(),
                topics: vec![DiscoveredEntry {
                    topic: "/scan".to_string(),
                    schema: Some("sensor_msgs/LaserScan".to_string()),
                    archetype: Some("LaserScan".to_string()),
                    entity: "world/scan".to_string(),
                    components: Some(vec!["LineStrips3D".to_string()]),
                    view_kinds: Some(vec!["spatial3d".to_string()]),
                    robot: Some("ubuntu".to_string()),
                    producer_count: None,
                    liveness: None,
                    liveness_state: None,
                    representation: None,
                    undecodable: None,
                }],
                error: None,
            }],
            discovery: None,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":7,"ok":true,"topics":[{"topic":"/cloud","schema":null,"archetype":null,"entity":"world/cloud","components":null,"view_kinds":null}],"robots":[{"robot":"ubuntu","topics":[{"topic":"/scan","schema":"sensor_msgs/LaserScan","archetype":"LaserScan","entity":"world/scan","components":["LineStrips3D"],"view_kinds":["spatial3d"],"robot":"ubuntu"}]}]}"#
        );
    }

    #[test]
    fn discover_producer_count_liveness_serializes_when_present() {
        // The per-row liveness affordance. A LOCAL live topic carries its
        // producer count (`Some(20)` here would be a 20-producer topic; realistically
        // 1), and a remote registered-but-dead route carries `Some(0)` (the reported
        // `/uslam/cloud_map`). Both serialize the `producer_count` key on the wire
        // (skip-serialized ONLY when `None`), so the Studio sidebar sees it. Hand oracle.
        let resp = Response::Discover(DiscoverResponse {
            id: Some(1),
            ok: true,
            topics: vec![DiscoveredEntry {
                topic: "/vel".to_string(),
                schema: Some("geometry_msgs/Vector3".to_string()),
                archetype: Some("Scalars".to_string()),
                entity: "world/vel".to_string(),
                components: Some(vec!["Scalars".to_string()]),
                view_kinds: Some(vec!["time_series".to_string()]),
                robot: None,
                // A live local producer — the count rides the trailing field.
                producer_count: Some(1),
                liveness: None,
                liveness_state: None,
                representation: None,
                undecodable: None,
            }],
            robots: vec![DiscoveredRobot {
                robot: "go2".to_string(),
                topics: vec![DiscoveredEntry {
                    topic: "/uslam/cloud_map".to_string(),
                    schema: Some("sensor_msgs/PointCloud2".to_string()),
                    archetype: Some("Points3D".to_string()),
                    entity: "world/uslam/cloud_map".to_string(),
                    components: Some(vec!["Points3D".to_string()]),
                    view_kinds: Some(vec!["spatial3d".to_string()]),
                    robot: Some("go2".to_string()),
                    // A registered-but-dead DDS route — no live producer.
                    producer_count: Some(0),
                    liveness: None,
                    liveness_state: None,
                    representation: None,
                    undecodable: None,
                }],
                error: None,
            }],
            discovery: None,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":1,"ok":true,"topics":[{"topic":"/vel","schema":"geometry_msgs/Vector3","archetype":"Scalars","entity":"world/vel","components":["Scalars"],"view_kinds":["time_series"],"producer_count":1}],"robots":[{"robot":"go2","topics":[{"topic":"/uslam/cloud_map","schema":"sensor_msgs/PointCloud2","archetype":"Points3D","entity":"world/uslam/cloud_map","components":["Points3D"],"view_kinds":["spatial3d"],"robot":"go2","producer_count":0}]}]}"#
        );
    }

    /// The DATA-FLOW liveness fields on the wire, against a literal hand
    /// oracle. THE motivating pair: two rows of one robot's catalog, both with
    /// `producer_count: 1` (the bridge registered a publisher for each), where only
    /// the observation distinguishes them — `/utlidar/cloud_deskewed` streaming and
    /// `/uslam/cloud_map` watched for 30s with no frame ever. Without liveness these two rows
    /// serialize IDENTICALLY apart from the topic name, which is exactly why the
    /// sidebar affordance is inert on a `ros2 attach` robot.
    ///
    /// Also pins that `liveness_state` is emitted alongside the raw numbers, so the
    /// Studio sidebar never re-derives thresholds. (The absent-⇒-no-key half of the
    /// additive contract is pinned by
    /// `discover_producer_count_liveness_serializes_when_present`, whose byte oracle
    /// carries no `liveness` key at all.)
    #[test]
    fn discover_data_flow_liveness_distinguishes_dead_from_streaming() {
        let row = |topic: &str, liveness: Option<TopicLiveness>| DiscoveredEntry {
            topic: topic.to_string(),
            schema: Some("sensor_msgs/PointCloud2".to_string()),
            archetype: Some("Points3D".to_string()),
            entity: format!("world{topic}"),
            components: Some(vec!["Points3D".to_string()]),
            view_kinds: Some(vec!["spatial3d".to_string()]),
            robot: Some("go2".to_string()),
            // BOTH rows: the bridge graph registered a publisher for each.
            producer_count: Some(1),
            liveness,
            liveness_state: liveness.and_then(|l| l.wire_state()),
            representation: None,
            undecodable: None,
        };
        let streaming = TopicLiveness {
            last_frame_age_ms: Some(48),
            observed_for_ms: 30_000,
            frames_observed: 600,
            rate_estimate: None,
        };
        let dead = TopicLiveness {
            last_frame_age_ms: None,
            observed_for_ms: 30_000,
            frames_observed: 0,
            rate_estimate: None,
        };
        // Hand-classified, NOT read back from the value under test.
        assert_eq!(streaming.state(), LivenessState::Streaming);
        assert_eq!(dead.state(), LivenessState::NoData);

        let resp = Response::Discover(DiscoverResponse {
            id: Some(9),
            ok: true,
            topics: vec![],
            robots: vec![DiscoveredRobot {
                robot: "go2".to_string(),
                topics: vec![
                    row("/uslam/cloud_map", Some(dead)),
                    row("/utlidar/cloud_deskewed", Some(streaming)),
                ],
                error: None,
            }],
            discovery: None,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":9,"ok":true,"topics":[],"robots":[{"robot":"go2","topics":[{"topic":"/uslam/cloud_map","schema":"sensor_msgs/PointCloud2","archetype":"Points3D","entity":"world/uslam/cloud_map","components":["Points3D"],"view_kinds":["spatial3d"],"robot":"go2","producer_count":1,"liveness":{"last_frame_age_ms":null,"observed_for_ms":30000,"frames_observed":0},"liveness_state":"no_data"},{"topic":"/utlidar/cloud_deskewed","schema":"sensor_msgs/PointCloud2","archetype":"Points3D","entity":"world/utlidar/cloud_deskewed","components":["Points3D"],"view_kinds":["spatial3d"],"robot":"go2","producer_count":1,"liveness":{"last_frame_age_ms":48,"observed_for_ms":30000,"frames_observed":600},"liveness_state":"streaming"}]}]}"#
        );
    }

    /// The robot's per-topic RATE ESTIMATE reaches the Studio sidebar on
    /// the `discover` wire, nested inside the row's existing `liveness` object.
    ///
    /// **This is the integration point the sidebar renders**: an UNCHECKED row's
    /// frequency is `topics[].liveness.rate_estimate` — `millihertz` (÷1000 for
    /// Hz) and `is_floor` (render `≥ N Hz` when true). A CHECKED row keeps using
    /// vizd's own `TopicStatus::hz`, which measures the same quantity the same way
    /// over the mirror this desk is already draining.
    ///
    /// Two rows, one hand oracle, and the pair is the point: a topic streaming
    /// far above the observer tap's own throughput reports its REAL rate from the
    /// publisher's commit sequence, while a topic whose sequence cannot be
    /// counted reports a labelled FLOOR. Nothing in vizd computes either — the
    /// robot measured them.
    ///
    /// **Scope:** this pins the wire shape only — the rows are
    /// hand-built `DiscoveredEntry` values, so it says nothing about whether the
    /// catalog→discover MAPPING carries the field. That hop is pinned separately by
    /// `daemon::tests::remote_entries_carry_data_flow_liveness_and_its_classification`,
    /// which drives `build_discovered_robots` over `CatalogEntry`s carrying both
    /// `is_floor` values plus an absent-rate negative.
    #[test]
    fn discover_carries_the_robots_rate_estimate_exactly_and_marks_a_floor() {
        use cerulion_core::TopicRateEstimate;

        let row = |topic: &str, liveness: TopicLiveness| DiscoveredEntry {
            topic: topic.to_string(),
            schema: Some("unitree_go/LowState".to_string()),
            archetype: None,
            entity: format!("world{topic}"),
            components: None,
            view_kinds: None,
            robot: Some("go2".to_string()),
            producer_count: Some(1),
            liveness: Some(liveness),
            liveness_state: liveness.wire_state(),
            representation: None,
            undecodable: None,
        };
        // The reported `/lowstate`: 500 Hz, measured exactly despite the
        // observer tap holding two frames per sweep.
        let fast = TopicLiveness {
            last_frame_age_ms: Some(2),
            observed_for_ms: 30_000,
            frames_observed: 300,
            rate_estimate: Some(TopicRateEstimate {
                millihertz: 500_000,
                is_floor: false,
            }),
        };
        // A publisher whose wire sequence cannot be counted: the correct answer is
        // a FLOOR at what the observer itself drained.
        let floored = TopicLiveness {
            last_frame_age_ms: Some(30),
            observed_for_ms: 30_000,
            frames_observed: 300,
            rate_estimate: Some(TopicRateEstimate {
                millihertz: 10_000,
                is_floor: true,
            }),
        };
        // Hand-classified, NOT read back from the value under test.
        assert_eq!(fast.state(), LivenessState::Streaming);
        assert_eq!(floored.state(), LivenessState::Streaming);

        let resp = Response::Discover(DiscoverResponse {
            id: Some(4),
            ok: true,
            topics: vec![],
            robots: vec![DiscoveredRobot {
                robot: "go2".to_string(),
                topics: vec![row("/lowstate", fast), row("/odd", floored)],
                error: None,
            }],
            discovery: None,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":4,"ok":true,"topics":[],"robots":[{"robot":"go2","topics":[{"topic":"/lowstate","schema":"unitree_go/LowState","archetype":null,"entity":"world/lowstate","components":null,"view_kinds":null,"robot":"go2","producer_count":1,"liveness":{"last_frame_age_ms":2,"observed_for_ms":30000,"frames_observed":300,"rate_estimate":{"millihertz":500000,"is_floor":false}},"liveness_state":"streaming"},{"topic":"/odd","schema":"unitree_go/LowState","archetype":null,"entity":"world/odd","components":null,"view_kinds":null,"robot":"go2","producer_count":1,"liveness":{"last_frame_age_ms":30,"observed_for_ms":30000,"frames_observed":300,"rate_estimate":{"millihertz":10000,"is_floor":true}},"liveness_state":"streaming"}]}]}"#
        );
    }

    /// UNKNOWN has exactly ONE wire encoding — ABSENCE — and the
    /// serialized `liveness_state` set is CLOSED to `streaming` / `idle` /
    /// `no_data`.
    ///
    /// Hand oracles, three arms:
    ///
    /// 1. Every classification a live observation can produce serializes to its
    ///    expected literal, and the set of literals is exactly those three (the
    ///    CLOSED-SET assertion — a fourth spelling reaching the wire fails here).
    /// 2. An observation that classifies UNKNOWN (observed, but not yet long
    ///    enough to say anything) emits NO `liveness_state` key — identical to a
    ///    row that carries no observation at all.
    /// 3. Even a caller that hand-sets `Some(LivenessState::Unknown)` — bypassing
    ///    `wire_state` — cannot put `"unknown"` on the wire: the skip predicate
    ///    drops it. DEcoding stays tolerant, so an `"unknown"` from some other
    ///    producer still parses.
    #[test]
    fn liveness_state_has_exactly_one_wire_encoding_of_unknown() {
        let observation = |age: Option<u64>, observed_for_ms: u64| TopicLiveness {
            last_frame_age_ms: age,
            observed_for_ms,
            frames_observed: u64::from(age.is_some()),
            rate_estimate: None,
        };
        // (1) the CLOSED serialized set, hand-paired with the observation that
        //     produces it.
        let cases = [
            (observation(Some(10), 60_000), r#""streaming""#),
            (observation(Some(600_000), 900_000), r#""idle""#),
            (observation(None, 600_000), r#""no_data""#),
        ];
        let mut emitted = Vec::new();
        for (live, expect) in cases {
            let state = live.wire_state().expect("a classified observation");
            let json = serde_json::to_string(&state).expect("serialize");
            assert_eq!(json, expect, "classification of {live:?}");
            emitted.push(json);
        }
        assert_eq!(
            emitted,
            vec![r#""streaming""#, r#""idle""#, r#""no_data""#],
            "the serialized classification set is CLOSED to these three — anything \
             else on the wire is a fourth spelling the sidebar would have to learn"
        );

        // (2) an UNKNOWN observation emits no key, exactly like no observation.
        let row = |liveness: Option<TopicLiveness>| DiscoveredEntry {
            topic: "/t".to_string(),
            schema: None,
            archetype: None,
            entity: "world/t".to_string(),
            components: None,
            view_kinds: None,
            robot: Some("go2".to_string()),
            producer_count: None,
            liveness,
            liveness_state: liveness.and_then(|l| l.wire_state()),
            representation: None,
            undecodable: None,
        };
        // Watched, but not long enough to claim anything.
        let settling = observation(None, 1);
        assert_eq!(settling.state(), LivenessState::Unknown);
        let with_unknown = serde_json::to_string(&row(Some(settling))).expect("serialize");
        assert!(
            !with_unknown.contains("liveness_state"),
            "an UNKNOWN classification must be ABSENT, not spelled out: {with_unknown}"
        );
        let no_observation = serde_json::to_string(&row(None)).expect("serialize");
        assert!(
            !no_observation.contains("liveness_state"),
            "no observation must also be absent: {no_observation}"
        );

        // (3) a hand-set Some(Unknown) still cannot reach the wire …
        let mut forced = row(None);
        forced.liveness_state = Some(LivenessState::Unknown);
        let json = serde_json::to_string(&forced).expect("serialize");
        assert!(
            !json.contains("liveness_state"),
            "the skip predicate — not just `wire_state` — must keep `unknown` off \
             the wire: {json}"
        );
        // … while the TYPE's decode stays tolerant, so an `unknown` produced by
        // something outside this crate (an older or foreign encoder) still parses
        // rather than failing a whole reply. (`DiscoveredEntry` itself is
        // serialize-only — vizd emits it and never reads it back — so the
        // tolerance that matters is the enum's.)
        assert_eq!(
            serde_json::from_str::<LivenessState>(r#""unknown""#).expect("tolerant decode"),
            LivenessState::Unknown
        );
    }

    /// The PRODUCED-BUT-UNDATABLE row on the wire — a null age together
    /// with a nonzero frame count, classified `"idle"`.
    ///
    /// This is the shape the sidebar must not get wrong: a latched `/tf_static`, a
    /// dead route whose retained history flushed into the robot's tap, and a topic
    /// whose only observed batch established the robot's dating baseline all serve
    /// it. Byte oracle, because the field docs promise the desk exactly these
    /// bytes — and the DISCRIMINATOR is asserted in the same body: the identical
    /// row with `frames_observed: 0` is `"no_data"`, the one row the sidebar dims.
    #[test]
    fn a_produced_but_undated_row_serializes_as_idle_not_no_data() {
        let row = |frames_observed: u64| {
            let liveness = TopicLiveness {
                last_frame_age_ms: None,
                observed_for_ms: 600_000,
                frames_observed,
                // An undated row is not Streaming, so it never carries a
                // rate — the byte oracle below is unchanged by the new field.
                rate_estimate: None,
            };
            DiscoveredEntry {
                topic: "/tf_static".to_string(),
                schema: Some("tf2_msgs/TFMessage".to_string()),
                archetype: None,
                entity: "world/tf_static".to_string(),
                components: None,
                view_kinds: None,
                robot: Some("go2".to_string()),
                producer_count: Some(1),
                liveness: Some(liveness),
                liveness_state: liveness.wire_state(),
                representation: None,
                undecodable: None,
            }
        };
        assert_eq!(
            serde_json::to_string(&row(1)).expect("serialize"),
            r#"{"topic":"/tf_static","schema":"tf2_msgs/TFMessage","archetype":null,"entity":"world/tf_static","components":null,"view_kinds":null,"robot":"go2","producer_count":1,"liveness":{"last_frame_age_ms":null,"observed_for_ms":600000,"frames_observed":1},"liveness_state":"idle"}"#,
            "a topic that PRODUCED but could not be dated is `idle` with a null \
             age — never `no_data` (which would dim a row carrying data) and never \
             `streaming` (which would claim a freshness nothing proved)"
        );
        let dead = serde_json::to_string(&row(0)).expect("serialize");
        assert!(
            dead.contains(r#""liveness_state":"no_data""#),
            "the DISCRIMINATOR: the identical row with ZERO frames is the dimmed \
             dead route: {dead}"
        );
    }

    #[test]
    fn discover_refused_robot_surfaces_the_error_and_no_topics() {
        // A refused robot rows EXPLICITLY (its reason +
        // no topics) — never a silent empty. Hand oracle on the exact bytes.
        let resp = Response::Discover(DiscoverResponse {
            id: Some(8),
            ok: true,
            topics: vec![],
            robots: vec![DiscoveredRobot {
                robot: "locked".to_string(),
                topics: vec![],
                error: Some("catalog refused: pairing required".to_string()),
            }],
            discovery: None,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":8,"ok":true,"topics":[],"robots":[{"robot":"locked","topics":[],"error":"catalog refused: pairing required"}]}"#
        );
    }

    // ── set_blueprint (the layout verb) ─────────────────────────────────

    #[test]
    fn parse_set_blueprint_reset_absent_or_null_is_none() {
        // A reset: `layout` absent OR explicitly null both parse to None.
        assert_eq!(
            parse_request(r#"{"id":6,"method":"set_blueprint"}"#).unwrap(),
            Request::SetBlueprint {
                id: 6,
                layout: None
            }
        );
        assert_eq!(
            parse_request(r#"{"id":6,"method":"set_blueprint","layout":null}"#).unwrap(),
            Request::SetBlueprint {
                id: 6,
                layout: None
            }
        );
    }

    #[test]
    fn parse_set_blueprint_with_a_full_nested_layout() {
        // Horizontal[3:1]( spatial3d "Scene" | vertical( time_series | text_document ) ).
        // Omitted `auto_views` defaults to true; omitted per-node options are None.
        let line = r#"{"id":7,"method":"set_blueprint","layout":{"root":{"type":"container","kind":"horizontal","shares":[3.0,1.0],"children":[{"type":"view","kind":"spatial3d","name":"Scene","origin":"world"},{"type":"container","kind":"vertical","children":[{"type":"view","kind":"time_series"},{"type":"view","kind":"text_document"}]}]}}}"#;
        let expected = Request::SetBlueprint {
            id: 7,
            layout: Some(LayoutSpec {
                auto_views: true,
                root: LayoutNode::Container(ContainerSpec {
                    kind: "horizontal".to_string(),
                    name: None,
                    shares: Some(vec![3.0, 1.0]),
                    columns: None,
                    children: vec![
                        LayoutNode::View(ViewSpec {
                            kind: "spatial3d".to_string(),
                            name: Some("Scene".to_string()),
                            origin: Some("world".to_string()),
                        }),
                        LayoutNode::Container(ContainerSpec {
                            kind: "vertical".to_string(),
                            name: None,
                            shares: None,
                            columns: None,
                            children: vec![
                                LayoutNode::View(ViewSpec {
                                    kind: "time_series".to_string(),
                                    name: None,
                                    origin: None,
                                }),
                                LayoutNode::View(ViewSpec {
                                    kind: "text_document".to_string(),
                                    name: None,
                                    origin: None,
                                }),
                            ],
                        }),
                    ],
                }),
            }),
        };
        assert_eq!(parse_request(line).unwrap(), expected);
    }

    #[test]
    fn unknown_layout_node_type_is_a_structured_parse_error() {
        // The node `type` tag (container vs view) is serde-validated: an unknown
        // node type is a structured parse error carrying the recovered id (an
        // unknown view/container KIND string, by contrast, parses here and is
        // rejected by the daemon's validation with a set-naming error).
        let err = parse_request(
            r#"{"id":8,"method":"set_blueprint","layout":{"root":{"type":"gadget"}}}"#,
        )
        .unwrap_err();
        assert_eq!(err.id, Some(8));
        assert!(!err.message.is_empty());
    }

    #[test]
    fn set_blueprint_response_serializes_to_the_exact_line() {
        let resp = Response::SetBlueprint(SetBlueprintResponse {
            id: Some(7),
            ok: true,
            views: 4,
            reset: false,
            warnings: vec!["world/ghost".to_string()],
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":7,"ok":true,"views":4,"reset":false,"warnings":["world/ghost"]}"#
        );
        // A reset (empty warnings) still serializes `warnings:[]` (stable shape).
        let reset = Response::SetBlueprint(SetBlueprintResponse {
            id: Some(9),
            ok: true,
            views: 3,
            reset: true,
            warnings: vec![],
        });
        assert_eq!(
            reset.to_json_line(),
            r#"{"id":9,"ok":true,"views":3,"reset":true,"warnings":[]}"#
        );
    }

    // ── compose_layout (layout compiler verb) ──────────────────

    #[test]
    fn parse_compose_layout_bare_topics_applies_defaults() {
        // The minimal automagic call: a bare topic list. strategy defaults to
        // "auto", attach_missing + include_robot default to true, groups is None.
        let line = r#"{"id":8,"method":"compose_layout","intent":{"topics":["/utlidar/cloud","/cmd_vel"]}}"#;
        assert_eq!(
            parse_request(line).unwrap(),
            Request::ComposeLayout {
                id: 8,
                intent: LayoutIntent {
                    groups: None,
                    topics: Some(vec!["/utlidar/cloud".to_string(), "/cmd_vel".to_string()]),
                    strategy: "auto".to_string(),
                    attach_missing: true,
                    include_robot: true,
                },
            }
        );
    }

    #[test]
    fn parse_compose_layout_groups_full_with_overridden_defaults() {
        let line = r#"{"id":9,"method":"compose_layout","intent":{"strategy":"focus_3d","attach_missing":false,"include_robot":false,"groups":[{"topics":["/utlidar/cloud"],"role":"hero","title":"Scene"},{"topics":["/cmd_vel"],"role":"plots"}]}}"#;
        assert_eq!(
            parse_request(line).unwrap(),
            Request::ComposeLayout {
                id: 9,
                intent: LayoutIntent {
                    groups: Some(vec![
                        LayoutGroup {
                            topics: vec!["/utlidar/cloud".to_string()],
                            role: Some("hero".to_string()),
                            title: Some("Scene".to_string()),
                        },
                        LayoutGroup {
                            topics: vec!["/cmd_vel".to_string()],
                            role: Some("plots".to_string()),
                            title: None,
                        },
                    ]),
                    topics: None,
                    strategy: "focus_3d".to_string(),
                    attach_missing: false,
                    include_robot: false,
                },
            }
        );
    }

    #[test]
    fn compose_layout_response_serializes_to_the_exact_line() {
        let resp = Response::ComposeLayout(ComposeLayoutResponse {
            id: Some(8),
            ok: true,
            placements: vec![
                Placement {
                    topic: "/utlidar/cloud".to_string(),
                    entity: "world/utlidar/cloud".to_string(),
                    archetype: Some("Points3D".to_string()),
                    view: "spatial3d".to_string(),
                    components: Some(vec!["Points3D".to_string()]),
                    dead: false,
                },
                Placement {
                    topic: "/cmd_vel".to_string(),
                    entity: "world/cmd_vel".to_string(),
                    archetype: Some("Scalars".to_string()),
                    view: "time_series".to_string(),
                    components: Some(vec!["Scalars".to_string()]),
                    dead: false,
                },
            ],
            arrangement: "hero_sidebar".to_string(),
            views: 2,
            attached: vec!["/cmd_vel".to_string()],
            warnings: vec![],
            skipped_dead: vec![],
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":8,"ok":true,"placements":[{"topic":"/utlidar/cloud","entity":"world/utlidar/cloud","archetype":"Points3D","view":"spatial3d","components":["Points3D"]},{"topic":"/cmd_vel","entity":"world/cmd_vel","archetype":"Scalars","view":"time_series","components":["Scalars"]}],"arrangement":"hero_sidebar","views":2,"attached":["/cmd_vel"],"warnings":[]}"#
        );
    }

    #[test]
    fn compose_placement_of_a_silent_topic_serializes_null_not_absent() {
        // A silent topic placed by role → archetype/components = null (NOT `[]`),
        // matching the schema/archetype null-not-absent convention.
        let resp = Response::ComposeLayout(ComposeLayoutResponse {
            id: Some(1),
            ok: true,
            placements: vec![Placement {
                topic: "/boot".to_string(),
                entity: "world/boot".to_string(),
                archetype: None,
                view: "spatial3d".to_string(),
                components: None,
                dead: false,
            }],
            arrangement: "single".to_string(),
            views: 1,
            attached: vec![],
            warnings: vec!["topic '/boot' is attached but still silent".to_string()],
            skipped_dead: vec![],
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":1,"ok":true,"placements":[{"topic":"/boot","entity":"world/boot","archetype":null,"view":"spatial3d","components":null}],"arrangement":"single","views":1,"attached":[],"warnings":["topic '/boot' is attached but still silent"]}"#
        );
    }

    #[test]
    fn compose_response_carries_the_878_dead_flag_and_skipped_dead_list() {
        // An EXPLICIT (role-tagged) dead placement carries `dead: true`; the
        // AUTOMAGIC-skipped dead topics ride `skipped_dead` (each with a reason).
        let resp = Response::ComposeLayout(ComposeLayoutResponse {
            id: Some(2),
            ok: true,
            placements: vec![Placement {
                topic: "/uslam/cloud_map".to_string(),
                entity: "world/uslam/cloud_map".to_string(),
                archetype: Some("Points3D".to_string()),
                view: "spatial3d".to_string(),
                components: Some(vec!["Points3D".to_string()]),
                dead: true,
            }],
            arrangement: "single".to_string(),
            views: 1,
            attached: vec![],
            warnings: vec![],
            skipped_dead: vec![SkippedDead {
                topic: "/dead/scan".to_string(),
                reason: "never published — 0 producers".to_string(),
            }],
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":2,"ok":true,"placements":[{"topic":"/uslam/cloud_map","entity":"world/uslam/cloud_map","archetype":"Points3D","view":"spatial3d","components":["Points3D"],"dead":true}],"arrangement":"single","views":1,"attached":[],"warnings":[],"skipped_dead":[{"topic":"/dead/scan","reason":"never published — 0 producers"}]}"#
        );
    }

    #[test]
    fn compose_response_without_878_fields_is_byte_identical_to_pre_878_wire() {
        // The additive contract: a live/unknown placement (`dead: false`) and an empty
        // `skipped_dead` serialize BYTE-IDENTICALLY to the wire without them (both fields
        // are `skip_serializing_if`), so an older Studio decoder is unaffected.
        let resp = Response::ComposeLayout(ComposeLayoutResponse {
            id: Some(3),
            ok: true,
            placements: vec![Placement {
                topic: "/utlidar/cloud".to_string(),
                entity: "world/utlidar/cloud".to_string(),
                archetype: Some("Points3D".to_string()),
                view: "spatial3d".to_string(),
                components: Some(vec!["Points3D".to_string()]),
                dead: false,
            }],
            arrangement: "single".to_string(),
            views: 1,
            attached: vec![],
            warnings: vec![],
            skipped_dead: vec![],
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":3,"ok":true,"placements":[{"topic":"/utlidar/cloud","entity":"world/utlidar/cloud","archetype":"Points3D","view":"spatial3d","components":["Points3D"]}],"arrangement":"single","views":1,"attached":[],"warnings":[]}"#
        );
    }

    // ── Round-trips (client build → parse) ──────────────────────────────────

    #[test]
    fn request_round_trips_through_its_own_line() {
        for req in [
            Request::Discover { id: 1 },
            Request::Attach {
                id: 2,
                topic: "/a".to_string(),
                entity: Some("world/x".to_string()),
                robot: None,
                schema: None,
            },
            Request::Detach {
                id: 3,
                topic: "/b".to_string(),
            },
            Request::List { id: 4 },
            Request::Status { id: 5 },
            Request::SetBlueprint {
                id: 6,
                layout: None,
            },
            Request::SetBlueprint {
                id: 7,
                layout: Some(LayoutSpec {
                    auto_views: false,
                    root: LayoutNode::View(ViewSpec {
                        kind: "spatial3d".to_string(),
                        name: Some("Only".to_string()),
                        origin: Some("world".to_string()),
                    }),
                }),
            },
        ] {
            let line = req.to_json_line();
            assert_eq!(parse_request(&line).unwrap(), req, "round-trip: {line}");
        }
    }

    #[test]
    fn minimal_attach_client_line_omits_none_options() {
        // The client helper emits a MINIMAL attach line (no null entity/robot).
        let line = Request::Attach {
            id: 2,
            topic: "/utlidar/cloud".to_string(),
            entity: None,
            robot: None,
            schema: None,
        }
        .to_json_line();
        assert!(!line.contains("entity"), "None entity omitted: {line}");
        assert!(!line.contains("robot"), "None robot omitted: {line}");
        assert!(!line.contains("schema"), "None schema omitted: {line}");
        assert!(line.contains(r#""topic":"/utlidar/cloud""#));
    }

    // -----------------------------------------------------------------------
    // Monitors — the `monitors` wire types.
    // -----------------------------------------------------------------------

    fn healthy_row() -> MonitorRow {
        MonitorRow {
            topic: "/utlidar/cloud".to_string(),
            robot: None,
            state: MonitorState::Healthy,
            conditions: Vec::new(),
            baseline_mhz: Some(20_000),
            observed_mhz: Some(19_800),
            observed_is_floor: Some(false),
            liveness_state: Some(LivenessState::Streaming),
            samples: 42,
            last_sample_age_ms: 120,
            ineligible: Vec::new(),
        }
    }

    /// A HEALTHY local row's exact bytes. `robot` and `ineligible` are ABSENT
    /// (nothing to say), while `conditions` is PRESENT and empty — the two
    /// conventions differ deliberately, and the difference is what an agent reads:
    /// `[]` is "we looked and nothing is raised", an absent key is "no claim".
    #[test]
    fn a_healthy_monitor_row_serializes_to_the_exact_line() {
        assert_eq!(
            serde_json::to_string(&healthy_row()).expect("serializes"),
            r#"{"topic":"/utlidar/cloud","state":"healthy","conditions":[],"baseline_mhz":20000,"observed_mhz":19800,"observed_is_floor":false,"liveness_state":"streaming","samples":42,"last_sample_age_ms":120}"#
        );
    }

    /// An ALERTING remote row carrying every field, so the full shape and the
    /// closed vocabulary spellings are pinned in one place.
    #[test]
    fn an_alerting_monitor_row_serializes_to_the_exact_line() {
        let row = MonitorRow {
            topic: "/tf".to_string(),
            robot: Some("go2".to_string()),
            state: MonitorState::Alerting,
            conditions: vec![MonitorCondition::Stalled, MonitorCondition::RateDeviation],
            baseline_mhz: Some(100_000),
            observed_mhz: Some(10_000),
            observed_is_floor: Some(true),
            liveness_state: Some(LivenessState::Idle),
            samples: 900,
            last_sample_age_ms: 2_400,
            ineligible: vec![Ineligible {
                condition: MonitorCondition::Silent,
                reason: IneligibleReason::SlowTopic,
            }],
        };
        assert_eq!(
            serde_json::to_string(&row).expect("serializes"),
            r#"{"topic":"/tf","robot":"go2","state":"alerting","conditions":["stalled","rate_deviation"],"baseline_mhz":100000,"observed_mhz":10000,"observed_is_floor":true,"liveness_state":"idle","samples":900,"last_sample_age_ms":2400,"ineligible":[{"condition":"silent","reason":"slow_topic"}]}"#
        );
    }

    /// ABSENT IS NOT ZERO, on every optional field of the row — the house rule
    /// this repo's catalog tests already pin ("a dead route has no rate, and no
    /// rate means NO KEY").
    ///
    /// A row that has learned nothing must not serialize `baseline_mhz: 0`,
    /// `observed_mhz: 0` or `observed_is_floor: false`: each of those is a
    /// POSITIVE claim, and an agent reading a fabricated `0 Hz` would page on a
    /// topic nobody has measured. The two fields that ARE present (`samples`,
    /// `last_sample_age_ms`) are the evidence-accounting pair and are meaningful at zero.
    #[test]
    fn a_row_with_nothing_to_say_omits_every_optional_field_rather_than_zeroing_it() {
        let row = MonitorRow {
            topic: "/x".to_string(),
            robot: None,
            state: MonitorState::Unknown,
            conditions: Vec::new(),
            baseline_mhz: None,
            observed_mhz: None,
            observed_is_floor: None,
            liveness_state: None,
            samples: 0,
            last_sample_age_ms: 0,
            ineligible: vec![Ineligible {
                condition: MonitorCondition::Stalled,
                reason: IneligibleReason::NoLivenessEvidence,
            }],
        };
        let line = serde_json::to_string(&row).expect("serializes");
        assert_eq!(
            line,
            r#"{"topic":"/x","state":"unknown","conditions":[],"samples":0,"last_sample_age_ms":0,"ineligible":[{"condition":"stalled","reason":"no_liveness_evidence"}]}"#
        );
        // Spelled out, because the exact-bytes assertion above would also be
        // satisfied by a future field order that happened to match.
        for key in [
            "robot",
            "baseline_mhz",
            "observed_mhz",
            "observed_is_floor",
            "liveness_state",
        ] {
            assert!(
                !line.contains(key),
                "{key} must be ABSENT, not zeroed: {line}"
            );
        }
    }

    /// An explicitly-`Unknown` liveness classification serializes as ABSENCE, the
    /// ONE wire encoding of unknown everywhere in this repo — never the string
    /// `"unknown"`, which would be a second spelling every reader must handle.
    #[test]
    fn an_unknown_liveness_classification_is_absent_on_a_monitor_row() {
        let row = MonitorRow {
            liveness_state: Some(LivenessState::Unknown),
            ..healthy_row()
        };
        let line = serde_json::to_string(&row).expect("serializes");
        assert!(!line.contains("liveness_state"), "{line}");
        assert!(!line.contains("unknown"), "{line}");
        // Anti-tautology: a REAL classification is still carried.
        assert!(serde_json::to_string(&healthy_row())
            .expect("serializes")
            .contains(r#""liveness_state":"streaming""#));
    }

    /// A RAISE and its CLEAR are two entries with two `seq`s, and
    /// `cleared_at_ms` is the discriminator: absent on the raise, present on the
    /// clear. Both exact-bytes, in one body, so the pair cannot drift apart.
    #[test]
    fn an_alert_raise_and_its_clear_serialize_as_two_entries() {
        let raise = Alert {
            seq: 7,
            topic: "/scan".to_string(),
            robot: Some("go2".to_string()),
            condition: MonitorCondition::Stalled,
            raised_at_ms: 9_600,
            cleared_at_ms: None,
            age_ms: None,
            observed_mhz: None,
            baseline_mhz: Some(20_000),
            liveness_state: Some(LivenessState::Idle),
        };
        assert_eq!(
            serde_json::to_string(&raise).expect("serializes"),
            r#"{"seq":7,"topic":"/scan","robot":"go2","condition":"stalled","raised_at_ms":9600,"baseline_mhz":20000,"liveness_state":"idle"}"#
        );

        let clear = Alert {
            seq: 8,
            cleared_at_ms: Some(12_000),
            observed_mhz: Some(20_100),
            liveness_state: Some(LivenessState::Streaming),
            ..raise.clone()
        };
        assert_eq!(
            serde_json::to_string(&clear).expect("serializes"),
            r#"{"seq":8,"topic":"/scan","robot":"go2","condition":"stalled","raised_at_ms":9600,"cleared_at_ms":12000,"observed_mhz":20100,"baseline_mhz":20000,"liveness_state":"streaming"}"#
        );
        assert_eq!(
            clear.raised_at_ms, raise.raised_at_ms,
            "the clear names the raise it ends"
        );
    }

    /// The serve-time AGE rides the wire beside the stamps it was derived from —
    /// so a controller renders "stalled 40 s ago" without doing arithmetic on a
    /// clock it does not share — and an alert nobody stamped OMITS the key
    /// entirely. Absent is UNKNOWN; a `0` would be a positive claim that the
    /// transition happened just now (the `baseline_mhz` discipline, applied to
    /// the one field whose zero is a plausible-looking lie).
    ///
    /// Driven through the PRODUCTION `Alert::with_age`, so the wire arm and the
    /// engine's rule cannot disagree, and asserted on BOTH kinds: a raise ages
    /// from its raise stamp, a clear from its clear stamp, with the clear's
    /// oracle chosen so the two rules give different numbers.
    #[test]
    fn a_stamped_alerts_age_rides_the_wire_and_an_unstamped_one_omits_the_key() {
        let raise = Alert {
            seq: 7,
            topic: "/scan".to_string(),
            robot: Some("go2".to_string()),
            condition: MonitorCondition::Stalled,
            raised_at_ms: 9_600,
            cleared_at_ms: None,
            age_ms: None,
            observed_mhz: None,
            baseline_mhz: Some(20_000),
            liveness_state: Some(LivenessState::Idle),
        };

        // UNSTAMPED: byte-identical to the pre-age shape — no key at all.
        let unstamped = serde_json::to_string(&raise).expect("serializes");
        assert_eq!(
            unstamped,
            r#"{"seq":7,"topic":"/scan","robot":"go2","condition":"stalled","raised_at_ms":9600,"baseline_mhz":20000,"liveness_state":"idle"}"#
        );
        assert!(
            !unstamped.contains("age_ms"),
            "an unknown age is ABSENT, never a zero that claims freshness"
        );

        // STAMPED raise: 40 s after the raise stamp.
        assert_eq!(
            serde_json::to_string(&raise.clone().with_age(49_600)).expect("serializes"),
            r#"{"seq":7,"topic":"/scan","robot":"go2","condition":"stalled","raised_at_ms":9600,"age_ms":40000,"baseline_mhz":20000,"liveness_state":"idle"}"#
        );

        // STAMPED clear: aged from the CLEAR stamp, not the raise's. Served at
        // the same instant as the raise above, so the two ages differ by exactly
        // the time the condition was open.
        let clear = Alert {
            seq: 8,
            cleared_at_ms: Some(12_000),
            liveness_state: Some(LivenessState::Streaming),
            ..raise
        };
        assert_eq!(
            serde_json::to_string(&clear.with_age(49_600)).expect("serializes"),
            r#"{"seq":8,"topic":"/scan","robot":"go2","condition":"stalled","raised_at_ms":9600,"cleared_at_ms":12000,"age_ms":37600,"baseline_mhz":20000,"liveness_state":"streaming"}"#
        );
    }

    /// The whole `monitors` response line, and the EMPTY one a fresh daemon
    /// serves. `monitors` / `alerts` are always present (an agent distinguishing
    /// "nothing watched" from "no key" would otherwise have to guess), and `seq`
    /// is the ring high-water mark.
    #[test]
    fn the_monitors_response_serializes_to_the_exact_line() {
        let resp = Response::Monitors(MonitorsResponse {
            id: 11,
            ok: true,
            monitors: vec![healthy_row()],
            alerts: vec![Alert {
                seq: 0,
                topic: "/utlidar/cloud".to_string(),
                robot: None,
                condition: MonitorCondition::Silent,
                raised_at_ms: 5_200,
                cleared_at_ms: None,
                age_ms: None,
                observed_mhz: None,
                baseline_mhz: None,
                liveness_state: Some(LivenessState::NoData),
            }],
            seq: 1,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":11,"ok":true,"monitors":[{"topic":"/utlidar/cloud","state":"healthy","conditions":[],"baseline_mhz":20000,"observed_mhz":19800,"observed_is_floor":false,"liveness_state":"streaming","samples":42,"last_sample_age_ms":120}],"alerts":[{"seq":0,"topic":"/utlidar/cloud","condition":"silent","raised_at_ms":5200,"liveness_state":"no_data"}],"seq":1}"#
        );

        let empty = Response::Monitors(MonitorsResponse {
            id: 12,
            ok: true,
            monitors: Vec::new(),
            alerts: Vec::new(),
            seq: 0,
        });
        assert_eq!(
            empty.to_json_line(),
            r#"{"id":12,"ok":true,"monitors":[],"alerts":[],"seq":0}"#
        );
    }

    /// ONE topic NAME can carry SEVERAL rows — one per robot serving it, plus one
    /// for a genuine local producer — and the served line must let a controller tell
    /// them apart, because they are independent streams with independent verdicts.
    ///
    /// The engine keys rows by (topic, robot) for exactly this reason; the wire half
    /// of that is `robot` being ABSENT on the local row and PRESENT on each remote
    /// one, never a disambiguator spliced into `topic`. Exact bytes, plus the alert
    /// attribution beside them: a controller reading only the alerts must still know
    /// which robot's `/lowstate` is broken.
    #[test]
    fn two_rows_for_one_topic_name_are_distinguishable_by_robot_on_the_wire() {
        let row = |robot: Option<&str>, state, baseline| MonitorRow {
            topic: "/lowstate".to_string(),
            robot: robot.map(str::to_string),
            state,
            conditions: if state == MonitorState::Alerting {
                vec![MonitorCondition::Stalled]
            } else {
                Vec::new()
            },
            baseline_mhz: Some(baseline),
            observed_mhz: None,
            observed_is_floor: None,
            liveness_state: None,
            samples: 9,
            last_sample_age_ms: 0,
            ineligible: Vec::new(),
        };
        let resp = Response::Monitors(MonitorsResponse {
            id: 3,
            ok: true,
            monitors: vec![
                // Sorted as the engine serves them: local first, then by robot.
                row(None, MonitorState::Healthy, 10_000),
                row(Some("go2-a"), MonitorState::Alerting, 20_000),
                row(Some("go2-b"), MonitorState::Healthy, 30_000),
            ],
            alerts: vec![Alert {
                seq: 4,
                topic: "/lowstate".to_string(),
                robot: Some("go2-a".to_string()),
                condition: MonitorCondition::Stalled,
                raised_at_ms: 800,
                cleared_at_ms: None,
                age_ms: None,
                observed_mhz: None,
                baseline_mhz: Some(20_000),
                liveness_state: Some(LivenessState::Idle),
            }],
            seq: 5,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":3,"ok":true,"monitors":[{"topic":"/lowstate","state":"healthy","conditions":[],"baseline_mhz":10000,"samples":9,"last_sample_age_ms":0},{"topic":"/lowstate","robot":"go2-a","state":"alerting","conditions":["stalled"],"baseline_mhz":20000,"samples":9,"last_sample_age_ms":0},{"topic":"/lowstate","robot":"go2-b","state":"healthy","conditions":[],"baseline_mhz":30000,"samples":9,"last_sample_age_ms":0}],"alerts":[{"seq":4,"topic":"/lowstate","robot":"go2-a","condition":"stalled","raised_at_ms":800,"baseline_mhz":20000,"liveness_state":"idle"}],"seq":5}"#
        );
    }

    /// The row and alert types ROUND-TRIP, so a controller (or a test) that parses
    /// them back gets what the daemon meant — including the absent-means-`None`
    /// fields, which is the half a serialize-only oracle cannot see.
    #[test]
    fn the_monitor_row_and_alert_round_trip_through_json() {
        let row = healthy_row();
        let back: MonitorRow =
            serde_json::from_str(&serde_json::to_string(&row).expect("ser")).expect("de");
        assert_eq!(back, row);

        // The minimal shape: every omitted key must decode to `None`, never a
        // default that asserts something.
        let minimal: MonitorRow = serde_json::from_str(
            r#"{"topic":"/x","state":"unknown","conditions":[],"samples":0,"last_sample_age_ms":0}"#,
        )
        .expect("de");
        assert_eq!(minimal.robot, None);
        assert_eq!(minimal.baseline_mhz, None);
        assert_eq!(minimal.observed_mhz, None);
        assert_eq!(minimal.observed_is_floor, None);
        assert_eq!(minimal.liveness_state, None);
        assert!(minimal.ineligible.is_empty());

        let alert = Alert {
            seq: 3,
            topic: "/x".to_string(),
            robot: None,
            condition: MonitorCondition::RateDeviation,
            raised_at_ms: 1,
            cleared_at_ms: None,
            age_ms: None,
            observed_mhz: Some(5),
            baseline_mhz: Some(50),
            liveness_state: Some(LivenessState::Streaming),
        };
        let back: Alert =
            serde_json::from_str(&serde_json::to_string(&alert).expect("ser")).expect("de");
        assert_eq!(back, alert);

        // A STAMPED alert round-trips its age too — the half a serialize-only
        // oracle cannot see, and the one a controller reads.
        let stamped = alert.clone().with_age(41);
        assert_eq!(stamped.age_ms, Some(40));
        let back: Alert =
            serde_json::from_str(&serde_json::to_string(&stamped).expect("ser")).expect("de");
        assert_eq!(back, stamped);

        // …and an alert with no age key decodes to `None`, never a default that
        // asserts the transition just happened.
        let minimal_alert: Alert =
            serde_json::from_str(r#"{"seq":1,"topic":"/x","condition":"silent","raised_at_ms":7}"#)
                .expect("de");
        assert_eq!(minimal_alert.age_ms, None);
        assert_eq!(minimal_alert.cleared_at_ms, None);
    }

    /// Monitors: the REQUEST half of the verb — the line a controller
    /// sends, byte for byte.
    ///
    /// The whole capability-negotiation argument for a new verb rests on the
    /// METHOD STRING: a daemon that predates this answers `{"method":"monitors"}`
    /// with the structured unknown-method error, and a client reads that as "not
    /// supported". So the spelling is the contract, and it is pinned in both
    /// directions here — a rename would silently make every deployed client
    /// unable to reach a daemon that does support it.
    #[test]
    fn the_monitors_request_parses_and_serializes_on_its_method_string() {
        let line = r#"{"method":"monitors","id":11}"#;
        assert_eq!(
            parse_request(line).expect("parses"),
            Request::Monitors { id: 11 }
        );
        assert_eq!(Request::Monitors { id: 11 }.to_json_line(), line);
        assert_eq!(Request::Monitors { id: 11 }.id(), 11);
    }

    /// An UNKNOWN method is still refused, and `monitors` is spelled exactly.
    ///
    /// The anti-tautology half of the arm above: without it, a `Request` that had
    /// grown an untagged catch-all would satisfy the round trip while accepting
    /// anything at all — and the negotiation contract needs the REFUSAL to work
    /// just as much as the acceptance, since that refusal is how an old daemon
    /// says "not supported".
    #[test]
    fn a_near_miss_method_is_refused_with_a_correlated_error() {
        let err = parse_request(r#"{"method":"monitor","id":12}"#).expect_err("refused");
        assert_eq!(err.id, Some(12), "the error still correlates");
        let err = parse_request(r#"{"method":"Monitors","id":13}"#).expect_err("refused");
        assert_eq!(err.id, Some(13));
    }

    // ===================================================================
    // The `runs` verb
    // ===================================================================

    /// A run row for `graph` on `robot`, with everything else fixed.
    fn run_row(robot: Option<&str>, run_id: &str, graph: &str) -> RunRow {
        RunRow {
            robot: robot.map(str::to_string),
            run_id: run_id.to_string(),
            graph_name: graph.to_string(),
            run_started_at_ns: 1_700_000_000_000_000_000,
            state: RunEntryState::Live,
            graph_yaml: "name: perception\n".to_string(),
            run_json: r#"{"version":1}"#.to_string(),
        }
    }

    /// A settled source status for `robot`.
    fn settled_source(robot: Option<&str>) -> RunsSourceStatus {
        RunsSourceStatus {
            robot: robot.map(str::to_string),
            completeness: RunsCompleteness::Settled,
            error: None,
            undescribable: Vec::new(),
        }
    }

    /// The REQUEST half — the line a controller sends, byte for byte, in both
    /// shapes.
    ///
    /// The capability-negotiation argument for a new verb rests entirely on the
    /// METHOD STRING: a daemon that predates this answers `{"method":"runs"}` with
    /// the structured unknown-method error, which a client reads as "not
    /// supported". So the spelling IS the contract and is pinned in both
    /// directions — a rename would silently make every deployed client unable to
    /// reach a daemon that does support it.
    ///
    /// `robot` is `skip_serializing_if`, so the unscoped form is the SHORTER line:
    /// a controller asking the ordinary question sends no key at all.
    #[test]
    fn the_runs_request_parses_and_serializes_on_its_method_string() {
        let line = r#"{"method":"runs","id":21}"#;
        assert_eq!(
            parse_request(line).expect("parses"),
            Request::Runs {
                id: 21,
                robot: None
            }
        );
        assert_eq!(
            Request::Runs {
                id: 21,
                robot: None
            }
            .to_json_line(),
            line
        );
        assert_eq!(
            Request::Runs {
                id: 21,
                robot: None
            }
            .id(),
            21
        );

        let scoped = r#"{"method":"runs","id":22,"robot":"go2"}"#;
        assert_eq!(
            parse_request(scoped).expect("parses"),
            Request::Runs {
                id: 22,
                robot: Some("go2".to_string())
            }
        );
        assert_eq!(
            Request::Runs {
                id: 22,
                robot: Some("go2".to_string())
            }
            .to_json_line(),
            scoped
        );
    }

    /// An UNKNOWN method is still refused, and `runs` is spelled exactly.
    ///
    /// The anti-tautology half of the arm above: without it, a `Request` that had
    /// grown an untagged catch-all would satisfy the round trip while accepting
    /// anything at all — and the negotiation contract needs the REFUSAL to work
    /// just as much as the acceptance, since that refusal is how an old daemon says
    /// "not supported".
    #[test]
    fn a_near_miss_runs_method_is_refused_with_a_correlated_error() {
        let err = parse_request(r#"{"method":"run","id":23}"#).expect_err("refused");
        assert_eq!(err.id, Some(23), "the error still correlates");
        let err = parse_request(r#"{"method":"Runs","id":24}"#).expect_err("refused");
        assert_eq!(err.id, Some(24));
    }

    /// The healthy RESPONSE line, byte for byte — and it is the SHORT one.
    ///
    /// Every field whose absence means "nothing to report" is omitted: no
    /// `unusable`, no `degraded`, and a local row carries no `robot` key at all.
    /// That last omission is the rule reaching the wire — a controller
    /// reading a row with no `robot` knows it is looking at THIS machine, and there
    /// is no hostname for it to render as if it were a robot.
    ///
    /// `runs`, `completeness` and `sources` are ALWAYS present, deliberately: `[]`
    /// is a positive "we looked", which an absent key cannot say.
    #[test]
    fn the_runs_response_serializes_to_the_exact_line() {
        let resp = Response::Runs(RunsResponse {
            id: 21,
            ok: true,
            runs: vec![
                run_row(None, "0x0000000000000000000000000000002a", "perception"),
                run_row(Some("go2"), "0x0000000000000000000000000000002b", "nav"),
            ],
            completeness: RunsCompleteness::Settled,
            sources: vec![settled_source(None), settled_source(Some("go2"))],
            unusable: Vec::new(),
            silent: Vec::new(),
            discovery: Some(DiscoveryLabel::Settled),
            degraded: None,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":21,"ok":true,"runs":[{"run_id":"0x0000000000000000000000000000002a","graph_name":"perception","run_started_at_ns":1700000000000000000,"state":"live","graph_yaml":"name: perception\n","run_json":"{\"version\":1}"},{"robot":"go2","run_id":"0x0000000000000000000000000000002b","graph_name":"nav","run_started_at_ns":1700000000000000000,"state":"live","graph_yaml":"name: perception\n","run_json":"{\"version\":1}"}],"completeness":{"kind":"settled"},"sources":[{"completeness":{"kind":"settled"}},{"robot":"go2","completeness":{"kind":"settled"}}],"discovery":"settled"}"#
        );
    }

    /// The EMPTY answer a desk with nothing running and no robots serves.
    ///
    /// `completeness` is what makes it readable at all, so it is present on the
    /// leanest line this verb can produce.
    #[test]
    fn an_empty_runs_response_still_carries_the_verdict_that_makes_it_readable() {
        let resp = Response::Runs(RunsResponse {
            id: 22,
            ok: true,
            runs: Vec::new(),
            completeness: RunsCompleteness::Settled,
            sources: vec![settled_source(None)],
            unusable: Vec::new(),
            silent: Vec::new(),
            discovery: Some(DiscoveryLabel::Settled),
            degraded: None,
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":22,"ok":true,"runs":[],"completeness":{"kind":"settled"},"sources":[{"completeness":{"kind":"settled"}}],"discovery":"settled"}"#
        );
    }

    /// Every way this answer can be SHORT, on the wire at the same time, and each
    /// distinguishable from the others.
    ///
    /// This is the arm the separation exists for, and the remedies are what
    /// make the separation load-bearing rather than tidy. A controller must be able
    /// to tell "netd could not ask" (`degraded` — look at netd) from "a robot
    /// answered with unusable bytes" (`unusable` — REDEPLOY it) from "an asked
    /// robot said nothing at all" (`silent` — wait, or upgrade it) from "discovery
    /// has not finished looking" (`discovery` — keep refreshing). Collapsed into one
    /// sentence, an operator is sent to the wrong machine.
    #[test]
    fn the_short_answers_are_four_distinguishable_fields_not_one_sentence() {
        let resp = Response::Runs(RunsResponse {
            id: 23,
            ok: true,
            runs: Vec::new(),
            completeness: RunsCompleteness::Incomplete {
                live_writers: 0,
                writers_heard: 0,
            },
            sources: vec![RunsSourceStatus {
                robot: Some("go2".to_string()),
                completeness: RunsCompleteness::Incomplete {
                    live_writers: 2,
                    writers_heard: 1,
                },
                error: Some("not paired with this desk".to_string()),
                undescribable: vec![UndescribableRun {
                    run_id: "0x0000000000000000000000000000002c".to_string(),
                    reason: "graph.yaml is not valid UTF-8".to_string(),
                }],
            }],
            unusable: vec![UnusableRunsAnswer {
                robot: "spot".to_string(),
                reason: "runs reply version 2 is newer than this binary".to_string(),
            }],
            silent: vec!["orin".to_string()],
            discovery: Some(DiscoveryLabel::Discovering),
            degraded: Some("cerulion-netd unreachable".to_string()),
        });
        assert_eq!(
            resp.to_json_line(),
            r#"{"id":23,"ok":true,"runs":[],"completeness":{"kind":"incomplete","live_writers":0,"writers_heard":0},"sources":[{"robot":"go2","completeness":{"kind":"incomplete","live_writers":2,"writers_heard":1},"error":"not paired with this desk","undescribable":[{"run_id":"0x0000000000000000000000000000002c","reason":"graph.yaml is not valid UTF-8"}]}],"unusable":[{"robot":"spot","reason":"runs reply version 2 is newer than this binary"}],"silent":["orin"],"discovery":"discovering","degraded":"cerulion-netd unreachable"}"#
        );
    }

    /// Two runs of ONE graph name on two machines stay distinguishable on the wire.
    ///
    /// A desk developing against a robot runs the same graph in both places — the
    /// normal case, not a corner — and `graph_name` is therefore not an identity.
    /// The discriminators are `run_id` (a restart mints a new one) and `robot`.
    #[test]
    fn two_runs_of_one_graph_name_are_distinguishable_by_robot_and_identity() {
        let resp = Response::Runs(RunsResponse {
            id: 24,
            ok: true,
            runs: vec![
                run_row(None, "0x00000000000000000000000000000001", "perception"),
                run_row(
                    Some("go2"),
                    "0x00000000000000000000000000000002",
                    "perception",
                ),
            ],
            completeness: RunsCompleteness::Settled,
            sources: vec![settled_source(None), settled_source(Some("go2"))],
            unusable: Vec::new(),
            silent: Vec::new(),
            discovery: Some(DiscoveryLabel::Settled),
            degraded: None,
        });
        let line = resp.to_json_line();
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("valid JSON");
        let rows = parsed["runs"].as_array().expect("runs is an array");
        assert_eq!(rows.len(), 2);
        assert!(
            rows[0].get("robot").is_none(),
            "the LOCAL row omits `robot` entirely — it is not `null`, and it is \
             certainly not this desk's hostname"
        );
        assert_eq!(rows[1]["robot"], "go2");
        assert_ne!(rows[0]["run_id"], rows[1]["run_id"]);
        assert_eq!(rows[0]["graph_name"], rows[1]["graph_name"]);
    }
}
