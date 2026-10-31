// SPDX-License-Identifier: AGPL-3.0-only
//! The PURE refcount + idle-lifecycle state machine at the
//! heart of `cerulion-netd`.
//!
//! netd owns ONE mirror per remote `(robot, topic)` regardless of how many
//! consumers want it (the decision: "obtain the same copy over the network
//! once"). This module tracks, per `(robot, topic)`, HOW MANY consumers currently
//! demand it AND whether its physical mirror bridge exists — so the daemon
//! registers the shared mirror on the FIRST demand and (C2) tears it down on the
//! LAST release — and decides WHEN netd is idle enough to self-exit.
//!
//! Two invariants make the refcount crash-safe:
//!
//! 1. **The connection is the liveliness token.** Every demand is held BY a
//!    connection; [`DemandRegistry::disconnect`] releases every demand that
//!    connection held. So a consumer that crashes (drops its socket) can never
//!    leak a demand — the daemon calls `disconnect` on EOF.
//! 2. **Per-connection idempotency.** A connection demanding the same
//!    `(robot, topic)` twice counts ONCE (a buggy consumer cannot inflate the
//!    refcount and pin a mirror alive).
//!
//! # The `Lingering` state — why an entry survives its last release (C1)
//!
//! In C1 the mirror plane's teardown is a NO-OP: cerulion_core has no
//! `unregister_ingress_topic` yet, so once
//! [`DemandRegistry::mark_mirror_present`] records a mirror, the physical bridge
//! outlives the refcount reaching zero. If the registry REMOVED the entry on the
//! last release, a LATER demand for the same topic would classify as a brand-new
//! `FirstDemand` → the daemon would call `register_ingress_topic` again → the
//! transport REFUSES the already-registered bridge → the demand FAILS for a topic
//! that worked moments ago (the sequential demand→release→re-demand break:
//! `topic echo`, Ctrl-C, echo again). So on the last release the registry keeps
//! the entry in a LINGERING state (`refcount == 0`, `mirror_present == true`); a
//! re-demand of a lingering key REUSES the existing bridge (an `AlreadyMirrored`
//! outcome — no re-ensure). The daemon still calls `release_mirror` on the last
//! release; when C2's real teardown actually tears the bridge down it returns
//! `Retired` and the daemon [`DemandRegistry::retire`]s the entry. In C1 the
//! bridge is reclaimed only when the whole process idle-self-exits.
//!
//! Idle ⟺ no live connections (which — since every demand is held by a
//! connection — implies no active demands). LINGERING entries (which have no
//! connection) do NOT block idle self-exit — the process exit is exactly what
//! reclaims a lingering bridge. When netd goes idle it arms an idle-since anchor;
//! [`DemandRegistry::should_self_exit`] reports when the grace has elapsed.
//!
//! Entirely pure (no zenoh, no iceoryx2, no locks, no wall clock — the caller
//! passes an [`Instant`]), so the whole lifecycle is oracle-tested with synthetic
//! time.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::time::{Duration, Instant};

/// A monotonic per-connection id the registry assigns at [`DemandRegistry::connect`].
pub type ConnId = u64;

/// A demanded remote stream: the origin robot identity + the canonical topic. The
/// key of netd's one-mirror-per-stream refcount table. Ordered so [`DemandRegistry::snapshot`]
/// is deterministic.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TopicKey {
    /// The remote robot identity (announce/mirror provenance).
    pub robot: String,
    /// The absolute canonical topic (leading `/`).
    pub topic: String,
}

impl TopicKey {
    /// Build a key, trimming `robot` and canonicalizing `topic` (see
    /// [`canonical_topic`]) so `/foo` and `foo` map to ONE mirror.
    pub fn new(robot: impl Into<String>, topic: impl AsRef<str>) -> Self {
        Self {
            robot: robot.into().trim().to_string(),
            topic: canonical_topic(topic.as_ref()),
        }
    }
}

/// Canonicalize a topic name: trim surrounding whitespace and ensure exactly one
/// leading `/`. An empty (or all-whitespace) input canonicalizes to `""` (the
/// daemon rejects an empty topic BEFORE keying — this never fabricates a `/`).
/// Pure — oracle-tested.
pub fn canonical_topic(topic: &str) -> String {
    let t = topic.trim();
    if t.is_empty() {
        return String::new();
    }
    let stripped = t.trim_start_matches('/');
    if stripped.is_empty() {
        // The input was only slashes ("/", "///") — not a real topic; preserve a
        // single slash so it is a distinct (still-degenerate) key, never "".
        return "/".to_string();
    }
    format!("/{stripped}")
}

/// Per-`(robot, topic)` mirror state.
#[derive(Debug)]
struct TopicState {
    /// How many DISTINCT connections currently demand this key. `0` means the
    /// entry is LINGERING (the mirror physically exists but no consumer wants it).
    refcount: usize,
    /// The wire `schema_hash` the FIRST demander established. A later demand for
    /// the same key with a DIFFERENT hash is refused (one data source = one
    /// topic = one schema).
    schema_hash: u64,
    /// Whether the physical mirror bridge EXISTS (a successful `ensure_mirror` has
    /// been recorded via [`DemandRegistry::mark_mirror_present`] and no `retire`
    /// has removed it). `false` only in the transient window between a
    /// [`DemandOutcome::FirstDemand`] and the daemon's `mark_mirror_present`/rollback
    /// (both under the registry lock). An entry is kept iff `refcount > 0 ||
    /// mirror_present`.
    mirror_present: bool,
    /// Whether this lingering entry's physical bridge is being TORN DOWN
    /// right now (the daemon [`DemandRegistry::claim_tearing`]'d it and is running
    /// the — potentially blocking, off-lock — `release_mirror`). While `true` a
    /// demand for the key must NOT reuse the dying bridge (it would ack a
    /// phantom-held mirror on a slot about to be released); it returns
    /// [`DemandOutcome::Retiring`] so the daemon WAITS for the teardown to finish,
    /// then re-demands into a fresh mirror. Only ever `true` on a lingering entry
    /// (`refcount == 0`, `mirror_present == true`).
    tearing: bool,
}

/// The outcome of a [`DemandRegistry::demand`] — tells the daemon whether it must
/// register the shared mirror (only on [`Self::FirstDemand`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DemandOutcome {
    /// This demand is the FIRST for `(robot, topic)` with NO existing mirror
    /// (refcount 0→1, no lingering bridge). The daemon must ensure the shared
    /// mirror exists, then call [`DemandRegistry::mark_mirror_present`] on success
    /// (or [`DemandRegistry::release`] to roll back on failure).
    FirstDemand {
        /// The refcount after this demand (always 1).
        refcount: usize,
    },
    /// The mirror already exists; this demand joined it (refcount ≥1→≥2), REUSED a
    /// LINGERING bridge (refcount 0→1, `mirror_present`), OR the connection
    /// re-demanded a key it already holds (idempotent — refcount unchanged). In
    /// every case the daemon does NOT register a second mirror.
    AlreadyMirrored {
        /// The refcount after this demand.
        refcount: usize,
    },
    /// A demand for a key already mirrored with a DIFFERENT `schema_hash` — refused
    /// (one data source = one topic = one schema). No state change; the daemon
    /// replies with an error.
    SchemaConflict {
        /// The hash the first demander established.
        existing: u64,
        /// The hash this demand requested.
        requested: u64,
    },
    /// The CROSS-PLAN LOOP GUARD (demand direction). The topic is
    /// produced + announced for egress by a local graph on this same shared session
    /// (ACTIVE or lingering-announced — its bridge flag persists until process-exit),
    /// so mirroring it IN would be a re-inject → tap → egress → re-inject echo loop.
    /// Refused loudly WITHOUT state change; the daemon replies with an
    /// error naming the topic + the fix. (The core's `self_ingress` / permissive-probe
    /// exclusions are the structural backstop; this is the loud, early refusal.)
    EgressConflict {
        /// The topic that is (or was, this session) egress-announced locally.
        topic: String,
    },
    /// The key's bridge is currently being TORN DOWN (`tearing`). The
    /// demand did NOT reuse the dying bridge (no refcount change) — the daemon must
    /// WAIT (bounded) for the teardown to finish, then RE-DEMAND (the retired entry
    /// re-classifies as a fresh [`Self::FirstDemand`] → a real new mirror). This is
    /// what stops a same-key re-demand landing during the off-lock teardown from
    /// acking a phantom-held mirror on a slot about to be released.
    Retiring,
    /// The connection id is unknown (never connected / already disconnected) — a
    /// protocol-misuse guard. No state change.
    UnknownConnection,
}

/// The outcome of a [`DemandRegistry::release`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReleaseOutcome {
    /// This release dropped the LAST demander (refcount 1→0). If the mirror is
    /// present the entry now LINGERS (kept until `retire`/process-exit); the daemon
    /// calls `MirrorPlane::release_mirror` (C1: a no-op → the entry lingers; C2:
    /// retires). If the mirror was NOT present (a rollback of a failed ensure) the
    /// entry is removed here.
    LastRelease,
    /// Other connections still demand the key (refcount ≥2→≥1).
    StillDemanded {
        /// The refcount after this release.
        refcount: usize,
    },
    /// This connection did not hold the key (a release for something never
    /// demanded). No state change.
    NotHeld,
    /// The connection id is unknown. No state change.
    UnknownConnection,
}

/// The outcome of a [`DemandRegistry::register_egress`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegisterEgressOutcome {
    /// The plan's egress topics were registered for this connection. `added` is the
    /// canonical topics NEWLY added by this call (empty if every topic was already
    /// held — an idempotent re-register), used by the daemon to roll back precisely
    /// if the egress plane then fails. `total` is the connection's egress topic count
    /// after this call.
    Registered {
        /// Canonical topics newly added by this call (for a precise rollback).
        added: Vec<String>,
        /// The connection's total egress topic count after this call.
        total: usize,
    },
    /// The CROSS-PLAN LOOP GUARD (egress direction). One of the plan's
    /// topics is currently MIRRORED IN by an ingress demand on this same shared
    /// session, so announcing it OUT would echo-loop. Refused WITHOUT any
    /// state change (all topics validated before any commit); the daemon replies with
    /// an error naming the topic + the robot mirroring it + the fix.
    LoopConflict {
        /// The offending topic (already mirrored in).
        topic: String,
        /// The remote robot whose topic is mirrored in (the holder, for the error).
        robot: String,
    },
    /// The connection id is unknown (never connected / already disconnected). No
    /// state change.
    UnknownConnection,
}

/// What a [`DemandRegistry::disconnect`] released — the demand mirrors
/// that hit their LAST release AND the connection's egress registration. The daemon
/// drives the mirror-plane teardown for each `mirror_last_released` key and the
/// egress-plane release for `egress_released`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct DisconnectOutcome {
    /// Demand keys that hit their LAST release (refcount → 0), sorted. Each now
    /// LINGERS (mirror_present) until `retire` / process-exit.
    pub mirror_last_released: Vec<TopicKey>,
    /// The connection's released canonical egress topics (sorted). Empty if it held
    /// no egress registration.
    pub egress_released: Vec<String>,
}

/// The pure refcount + idle-lifecycle state machine (see the module docs). Not
/// thread-safe by itself — the daemon wraps it in a `Mutex` and shares it across
/// the connection + idle-watch threads.
#[derive(Debug)]
pub struct DemandRegistry {
    /// `(robot, topic)` → its mirror state. An entry exists iff `refcount > 0 ||
    /// mirror_present` (a lingering entry has `refcount == 0 && mirror_present`).
    topics: BTreeMap<TopicKey, TopicState>,
    /// Connection → the set of keys it currently holds (dedup: a connection holds
    /// a key at most once — the per-connection idempotency invariant).
    per_conn: HashMap<ConnId, BTreeSet<TopicKey>>,
    /// Connection → its canonical EGRESS topic set (the produced topics
    /// it registered for the shared gateway to announce + egress-on-demand). A
    /// connection has an entry here iff it holds ≥1 egress topic. Connection-close
    /// releases it (crash-safe, mirroring demand release). Kept SEPARATE from
    /// `per_conn` because egress is topic-only (the desk IS the robot for what it
    /// produces) while a demand is keyed by the remote `(robot, topic)`.
    egress: HashMap<ConnId, BTreeSet<String>>,
    /// Canonical egress topic → how many connections ACTIVELY registered
    /// it (bumped on `register_egress`, decremented on release/disconnect). Together
    /// with [`Self::egress_announced`] it is the membership index for the cross-plan
    /// loop guard.
    egress_topics: BTreeMap<String, usize>,
    /// The lingering-announced fix: canonical topics whose PHYSICAL
    /// egress bridge flag has been pushed onto the shared gateway this session. This
    /// mirrors the shared `TransportManager`'s ADD-ONLY bridge flag: a topic enters
    /// here when the egress plane's push SUCCEEDS ([`Self::mark_egress_announced`],
    /// called by the daemon after the plane call — the `mark_mirror_present` pattern)
    /// and NEVER leaves until process-exit (the reg-channel + announce set have no
    /// removal primitive yet — the egress twin of the mirror teardown). So on release the ACTIVE
    /// accounting ([`Self::egress_topics`]) drops but the topic stays ANNOUNCED here,
    /// keeping the demand-side loop guard CORRECT: a `demand` for a topic that was
    /// egressed this session is refused (its bridge flag is still set, so a re-inject
    /// would echo-loop past the loop guard and die in `create_ingress_publisher` on
    /// the still-registered flag). A re-`register_egress` of a lingering-announced
    /// topic REVIVES it (the flag push is idempotent) — the egress twin of the mirror
    /// plane's lingering-reuse revival. LINGERING-announced topics do NOT pin liveness
    /// (only [`Self::egress`] does — process-exit reclaims them, exactly like a
    /// lingering mirror).
    egress_announced: BTreeSet<String>,
    /// The next connection id to hand out.
    next_conn_id: ConnId,
    /// Live connection count (== `per_conn.len()`, tracked for clarity + O(1) reads).
    active_conns: usize,
    /// `Some(t)` while idle (no connections AND no egress registrations) since `t`;
    /// `None` while busy. The idle-grace timer counts from `t`. An
    /// active egress registration PINS liveness — post-C5 a running graph that
    /// produces keeps netd alive so its egress never silently drops (by design).
    idle_since: Option<Instant>,
}

impl DemandRegistry {
    /// A fresh registry at `now`. Boots IDLE (no connections), so a netd that is
    /// spawned but never demanded self-exits after the grace — nothing left behind.
    pub fn new(now: Instant) -> Self {
        Self {
            topics: BTreeMap::new(),
            per_conn: HashMap::new(),
            egress: HashMap::new(),
            egress_topics: BTreeMap::new(),
            egress_announced: BTreeSet::new(),
            next_conn_id: 0,
            active_conns: 0,
            idle_since: Some(now),
        }
    }

    /// Register a new consumer connection at `now`, returning its [`ConnId`]. The
    /// first live connection clears the idle anchor (netd is busy).
    pub fn connect(&mut self, now: Instant) -> ConnId {
        let id = self.next_conn_id;
        self.next_conn_id += 1;
        self.per_conn.insert(id, BTreeSet::new());
        self.active_conns += 1;
        self.refresh_idle(now);
        id
    }

    /// Demand `(robot, topic)` with `schema_hash` on connection `conn` at `now`.
    /// See [`DemandOutcome`]. Idempotent per connection; a demand for a LINGERING
    /// key reuses the existing mirror ([`DemandOutcome::AlreadyMirrored`], no
    /// re-ensure).
    pub fn demand(
        &mut self,
        conn: ConnId,
        key: TopicKey,
        schema_hash: u64,
        _now: Instant,
    ) -> DemandOutcome {
        let Some(held) = self.per_conn.get_mut(&conn) else {
            return DemandOutcome::UnknownConnection;
        };

        // The CROSS-PLAN LOOP GUARD (demand direction). Refuse a demand
        // for a topic a local graph produces + announces on this one shared session —
        // mirroring it back IN would echo-loop. Checks BOTH the ACTIVE
        // registrations AND the lingering-ANNOUNCED set: a topic egressed this session
        // stays announced (its bridge flag lingers until process-exit), so a demand
        // for it must be refused EXPLICITLY here rather than passing the guard and dying
        // late in create_ingress_publisher on the still-set flag. Checked FIRST (a
        // topic can never be both egress-registered and demanded). No state change.
        // (Inlined direct-field access — disjoint from the `held` per_conn borrow; a
        // `&self` method call would conflict.)
        if self.egress_topics.contains_key(&key.topic) || self.egress_announced.contains(&key.topic)
        {
            return DemandOutcome::EgressConflict {
                topic: key.topic.clone(),
            };
        }

        // A key mid-teardown accepts NO demand — reusing the dying bridge
        // would ack a phantom-held mirror on a slot about to be released. Return
        // Retiring (before the schema check: the tearing entry's schema is about to
        // vanish) so the daemon WAITS for the teardown, then re-demands into a fresh
        // mirror. Checked before the idempotency + increment paths below.
        if let Some(state) = self.topics.get(&key) {
            if state.tearing {
                return DemandOutcome::Retiring;
            }
        }

        // Schema-conflict check FIRST (against an existing OR lingering mirror),
        // regardless of whether THIS connection already holds the key — one topic,
        // one schema.
        if let Some(state) = self.topics.get(&key) {
            if state.schema_hash != schema_hash {
                return DemandOutcome::SchemaConflict {
                    existing: state.schema_hash,
                    requested: schema_hash,
                };
            }
        }

        // Per-connection idempotency: a re-demand of a held key does not
        // double-count. (A held key always has an active mirror, so this reports
        // the current refcount unchanged.)
        if held.contains(&key) {
            let refcount = self.topics.get(&key).map(|s| s.refcount).unwrap_or(0);
            return DemandOutcome::AlreadyMirrored { refcount };
        }

        // A genuinely new hold for this connection.
        held.insert(key.clone());
        match self.topics.get_mut(&key) {
            // An existing entry — either actively shared (refcount ≥ 1) OR
            // LINGERING (refcount 0, mirror_present). Either way the physical
            // mirror exists, so REUSE it: increment, NO re-ensure.
            Some(state) => {
                state.refcount += 1;
                DemandOutcome::AlreadyMirrored {
                    refcount: state.refcount,
                }
            }
            // No entry at all → a genuinely new mirror. Created with
            // `mirror_present: false`; the daemon ensures the bridge and calls
            // `mark_mirror_present` on success (or `release` to roll back).
            None => {
                self.topics.insert(
                    key,
                    TopicState {
                        refcount: 1,
                        schema_hash,
                        mirror_present: false,
                        tearing: false,
                    },
                );
                DemandOutcome::FirstDemand { refcount: 1 }
            }
        }
    }

    /// Record that the physical mirror for `key` now EXISTS — the daemon calls this
    /// after a successful `ensure_mirror` (under the registry lock, right after a
    /// [`DemandOutcome::FirstDemand`]). Idempotent; a no-op if the entry is absent
    /// (it was rolled back).
    pub fn mark_mirror_present(&mut self, key: &TopicKey) {
        if let Some(state) = self.topics.get_mut(key) {
            state.mirror_present = true;
        }
    }

    /// Release `(robot, topic)` on connection `conn` at `now`. See [`ReleaseOutcome`].
    pub fn release(&mut self, conn: ConnId, key: &TopicKey, _now: Instant) -> ReleaseOutcome {
        let Some(held) = self.per_conn.get_mut(&conn) else {
            return ReleaseOutcome::UnknownConnection;
        };
        if !held.remove(key) {
            return ReleaseOutcome::NotHeld;
        }
        Self::decrement_topic(&mut self.topics, key)
    }

    /// Disconnect `conn` at `now`, releasing EVERY demand AND egress registration it
    /// held — the crash-safe guarantee (a dropped socket can never leak either).
    /// Returns a [`DisconnectOutcome`]: the demand keys that hit their LAST release
    /// (refcount → 0, each now LINGERING and KEPT until `retire`/process-exit) so the
    /// daemon can tear those mirrors down, AND the connection's released egress topics
    /// so the daemon can drive the egress plane's release. Both sorted for
    /// deterministic downstream ordering. Dropping the last connection AND clearing
    /// its egress arms the idle anchor (egress pins liveness, so egress
    /// is released BEFORE `refresh_idle` here). Unknown / double disconnect is a no-op
    /// returning an empty outcome.
    pub fn disconnect(&mut self, conn: ConnId, now: Instant) -> DisconnectOutcome {
        let Some(held) = self.per_conn.remove(&conn) else {
            return DisconnectOutcome::default();
        };
        self.active_conns -= 1;
        // Release the connection's egress registration FIRST (before refresh_idle, so
        // the idle decision sees it gone), then its held demands.
        let egress_released = self.take_egress_locked(conn);
        let mut mirror_last_released = Vec::new();
        for key in held {
            if matches!(
                Self::decrement_topic(&mut self.topics, &key),
                ReleaseOutcome::LastRelease
            ) {
                mirror_last_released.push(key);
            }
        }
        mirror_last_released.sort();
        self.refresh_idle(now);
        DisconnectOutcome {
            mirror_last_released,
            egress_released,
        }
    }

    /// Retire a LINGERING mirror (refcount 0) for `key` — remove its entry. The
    /// daemon calls this when the mirror plane's `release_mirror` reports the
    /// physical bridge was actually torn down ([`crate::mirror::MirrorRelease::Retired`]
    /// — C2). Guarded by `refcount == 0`: a key re-demanded between the release and
    /// the retire (refcount ≥ 1) is NOT retired (a live mirror is never removed).
    /// Returns whether an entry was removed.
    pub fn retire(&mut self, key: &TopicKey) -> bool {
        match self.topics.get(key) {
            Some(state) if state.refcount == 0 => {
                self.topics.remove(key);
                true
            }
            _ => false,
        }
    }

    /// Forget mirrors whose transport has already been invalidated.
    ///
    /// Account/key replacement and removed WAN membership revoke live demands,
    /// unlike ordinary refcount-zero retirement. The caller must close readers
    /// and retire physical mirrors first, while serializing new demand admission.
    /// Remove each connection's hold as well: a later close must not decrement a
    /// new generation's demand. Connections and local egress registrations survive.
    /// Returns removed keys in deterministic order, including lingering mirrors.
    /// If any selected key has an in-flight teardown, returns those busy keys
    /// without changing anything. A delayed teardown must never retire a newly
    /// admitted generation of the same key; retry only after it has finished.
    pub fn invalidate_mirrors(
        &mut self,
        keys: &BTreeSet<TopicKey>,
    ) -> Result<Vec<TopicKey>, Vec<TopicKey>> {
        let busy: Vec<_> = keys
            .iter()
            .filter(|key| self.is_tearing(key))
            .cloned()
            .collect();
        if !busy.is_empty() {
            return Err(busy);
        }
        let removed: Vec<_> = keys
            .iter()
            .filter(|key| self.topics.remove(*key).is_some())
            .cloned()
            .collect();
        for held in self.per_conn.values_mut() {
            held.retain(|key| !keys.contains(key));
        }
        Ok(removed)
    }

    /// CLAIM a lingering key for teardown — atomically mark it `tearing`
    /// so no demand can reuse the bridge while the daemon runs the (off-lock)
    /// `release_mirror`. Returns `true` iff the claim succeeded: the entry exists,
    /// is LINGERING (`refcount == 0`, `mirror_present`), and is NOT already tearing.
    /// A `false` return means SKIP the teardown — the key was re-claimed by a demand
    /// (refcount > 0, its bridge is live) OR another teardown already claimed it
    /// (`tearing == true`, a concurrent `release_mirror` is in flight → this caller
    /// must be a clean no-op, never a second `release_mirror` that would double-
    /// unregister and fire a spurious loud error). Only the claim winner runs the
    /// teardown, and while `tearing` is set every demand for the key returns
    /// [`DemandOutcome::Retiring`], so refcount cannot leave 0 until the teardown
    /// clears it via [`Self::retire`] / [`Self::clear_tearing`].
    pub fn claim_tearing(&mut self, key: &TopicKey) -> bool {
        match self.topics.get_mut(key) {
            Some(state) if state.refcount == 0 && state.mirror_present && !state.tearing => {
                state.tearing = true;
                true
            }
            _ => false,
        }
    }

    /// Clear a key's `tearing` flag WITHOUT removing the entry — the daemon
    /// calls this when the teardown returned [`crate::mirror::MirrorRelease::Lingering`]
    /// (teardown FAILED), so the still-lingering bridge becomes reusable again (a
    /// later demand reuses it via `AlreadyMirrored`). A no-op if the entry is absent.
    /// (The `Retired` path removes the entry via [`Self::retire`] instead.)
    pub fn clear_tearing(&mut self, key: &TopicKey) {
        if let Some(state) = self.topics.get_mut(key) {
            state.tearing = false;
        }
    }

    /// Whether `key` is currently being torn down (`tearing`). Diagnostics / tests.
    pub fn is_tearing(&self, key: &TopicKey) -> bool {
        self.topics.get(key).map(|s| s.tearing).unwrap_or(false)
    }

    // ─── Egress registration (the desk egress-convergence seam) ───

    /// Register `topics` as EGRESS for connection `conn` (canonicalized + deduped;
    /// empty topics dropped). Additive — a topic the connection already holds is a
    /// no-op for it. See [`RegisterEgressOutcome`].
    ///
    /// The CROSS-PLAN LOOP GUARD validates EVERY topic against the demand table
    /// BEFORE any commit: if any is currently mirrored IN (a `(robot, topic)` in the
    /// demand table), the WHOLE call is refused with no partial state change
    /// ([`RegisterEgressOutcome::LoopConflict`]), so netd never announces OUT a topic
    /// it re-injects IN. On success the daemon boots/feeds the shared egress
    /// gateway then [`Self::mark_egress_announced`]s the `added` topics (the
    /// `mark_mirror_present` pattern — records the ADD-ONLY physical announce so a
    /// later release keeps the loop guard correct); on a gateway failure it rolls the
    /// just-added topics back via [`Self::rollback_egress`] (which never touches the
    /// announce set — the failed push left no flag).
    ///
    /// A re-register of a lingering-ANNOUNCED topic (one released by every connection
    /// but whose bridge flag persists) SUCCEEDS and revives it — the topic is not in
    /// the demand table (mutual exclusion holds), so the loop guard passes and the
    /// gateway re-push is idempotent (the mirror plane's lingering-reuse revival).
    pub fn register_egress(&mut self, conn: ConnId, topics: Vec<String>) -> RegisterEgressOutcome {
        if !self.per_conn.contains_key(&conn) {
            return RegisterEgressOutcome::UnknownConnection;
        }
        // Canonicalize + dedup, dropping empties (an empty topic names nothing).
        let mut requested: BTreeSet<String> = BTreeSet::new();
        for t in topics {
            let c = canonical_topic(&t);
            if !c.is_empty() {
                requested.insert(c);
            }
        }
        // Loop guard: validate ALL topics against the demand table before committing.
        for topic in &requested {
            if let Some(robot) = self.demanded_topic_robot(topic) {
                return RegisterEgressOutcome::LoopConflict {
                    topic: topic.clone(),
                    robot,
                };
            }
        }
        // Commit: add newly-held topics to the connection's set + bump the index.
        let mut added = Vec::new();
        for topic in requested {
            let newly = self.egress.entry(conn).or_default().insert(topic.clone());
            if newly {
                *self.egress_topics.entry(topic.clone()).or_insert(0) += 1;
                added.push(topic);
            }
        }
        // A connection whose set ended up empty (only empty topics requested) must
        // NOT retain an entry — an empty egress entry would falsely pin liveness.
        if self.egress.get(&conn).is_some_and(|s| s.is_empty()) {
            self.egress.remove(&conn);
        }
        let total = self.egress.get(&conn).map(|s| s.len()).unwrap_or(0);
        RegisterEgressOutcome::Registered { added, total }
    }

    /// Record that the shared gateway's physical egress bridge flag for each `added`
    /// topic is now SET — the daemon calls this after a SUCCESSFUL egress-plane push
    /// (the `mark_mirror_present` analogue). Add-only: the topic stays announced (and
    /// therefore demand-blocked) until process-exit, matching the add-only bridge
    /// flag. Idempotent.
    pub fn mark_egress_announced(&mut self, added: &[String]) {
        for topic in added {
            self.egress_announced.insert(canonical_topic(topic));
        }
    }

    /// Roll back a just-registered egress set for `conn` (the daemon calls this when
    /// the egress plane fails AFTER [`Self::register_egress`] recorded the topics but
    /// BEFORE [`Self::mark_egress_announced`]) — removes EXACTLY the `added` topics +
    /// decrements the active index. Deliberately does NOT touch the ANNOUNCED set: a
    /// failed push set no flag, and another connection may hold the topic announced.
    /// A no-op for a topic the connection no longer holds.
    pub fn rollback_egress(&mut self, conn: ConnId, added: &[String]) {
        for topic in added {
            let removed = self.egress.get_mut(&conn).is_some_and(|s| s.remove(topic));
            if removed {
                Self::decrement_egress_index(&mut self.egress_topics, topic);
            }
        }
        if self.egress.get(&conn).is_some_and(|s| s.is_empty()) {
            self.egress.remove(&conn);
        }
    }

    /// Release connection `conn`'s WHOLE egress registration (the explicit
    /// `release_egress` verb, while the connection stays open) — removes + returns
    /// its canonical egress topics (sorted; empty if it held none) so the daemon can
    /// drive the egress plane's release. Does NOT touch the idle anchor: a live
    /// connection is still busy (idle is armed only at disconnect).
    pub fn release_egress(&mut self, conn: ConnId) -> Vec<String> {
        self.take_egress_locked(conn)
    }

    /// Remove + return `conn`'s ACTIVE egress topic set (decrementing the active
    /// index). The shared helper for [`Self::release_egress`] and [`Self::disconnect`].
    /// Deliberately does NOT touch the lingering-ANNOUNCED set (the physical bridge
    /// flag persists until process-exit, keeping the loop guard correct) and does not
    /// refresh idle (the caller sequences that).
    fn take_egress_locked(&mut self, conn: ConnId) -> Vec<String> {
        let Some(set) = self.egress.remove(&conn) else {
            return Vec::new();
        };
        let mut released: Vec<String> = set.into_iter().collect();
        for topic in &released {
            Self::decrement_egress_index(&mut self.egress_topics, topic);
        }
        released.sort();
        released
    }

    /// Decrement the egress-topic index for `topic`, removing the entry at 0.
    fn decrement_egress_index(index: &mut BTreeMap<String, usize>, topic: &str) {
        if let Some(c) = index.get_mut(topic) {
            *c -= 1;
            if *c == 0 {
                index.remove(topic);
            }
        }
    }

    /// The robot of a demand-table entry whose canonical topic equals `topic`
    /// (present OR lingering — the physical mirror exists or is being created), or
    /// `None`. Drives the egress-side loop-guard's holder attribution.
    fn demanded_topic_robot(&self, topic: &str) -> Option<String> {
        self.topics
            .keys()
            .find(|k| k.topic == topic)
            .map(|k| k.robot.clone())
    }

    /// Whether an egress registration blocks a `demand` for `topic` (already
    /// canonical): ACTIVE by any connection OR lingering-ANNOUNCED this session. The
    /// demand-side loop guard's membership check.
    fn egress_blocks_demand(&self, topic: &str) -> bool {
        self.egress_topics.contains_key(topic) || self.egress_announced.contains(topic)
    }

    /// Whether `topic` (canonicalized) is currently egress-registered
    /// (ACTIVE) or lingering-ANNOUNCED this session — the demand-side loop guard's
    /// membership, also a Principle-#3 observable. A `true` here refuses a `demand`
    /// for `topic` (netd never mirrors IN a topic it produces OUT).
    pub fn is_egress_registered(&self, topic: &str) -> bool {
        self.egress_blocks_demand(&canonical_topic(topic))
    }

    /// The number of CONNECTIONS holding an active egress registration
    /// (each has ≥1 topic). `> 0` pins netd's liveness. Principle #3 / tests.
    pub fn active_egress_registration_count(&self) -> usize {
        self.egress.len()
    }

    /// The number of DISTINCT canonical topics currently ACTIVELY
    /// egress-registered across all connections. Principle #3 / tests.
    pub fn egress_topic_count(&self) -> usize {
        self.egress_topics.len()
    }

    /// The number of DISTINCT canonical topics whose physical egress
    /// announce has been marked this session (active + lingering — the add-only
    /// bridge-flag mirror). Principle #3 / tests.
    pub fn announced_egress_topic_count(&self) -> usize {
        self.egress_announced.len()
    }

    /// Decrement a key's refcount. On refcount 1→0: KEEP the entry as LINGERING if
    /// the mirror is present (the daemon's `release_mirror` decides teardown), or
    /// REMOVE it if the mirror was never present (a rollback of a failed ensure).
    /// The key is KNOWN to be held (the caller removed it from the connection's set
    /// first), so a missing entry is an internal invariant break — treated as
    /// `LastRelease` (nothing left to hold) rather than a panic.
    fn decrement_topic(
        topics: &mut BTreeMap<TopicKey, TopicState>,
        key: &TopicKey,
    ) -> ReleaseOutcome {
        match topics.get_mut(key) {
            Some(state) if state.refcount > 1 => {
                state.refcount -= 1;
                ReleaseOutcome::StillDemanded {
                    refcount: state.refcount,
                }
            }
            Some(state) if state.mirror_present => {
                // Last demander left, but the physical bridge exists → LINGER
                // (keep the entry at refcount 0). The daemon's release_mirror
                // decides whether to retire it (C2) or leave it (C1 no-op).
                state.refcount = 0;
                ReleaseOutcome::LastRelease
            }
            _ => {
                // refcount 1→0 with NO mirror (rollback of a failed ensure), or a
                // missing entry → remove.
                topics.remove(key);
                ReleaseOutcome::LastRelease
            }
        }
    }

    /// Set / clear the idle anchor. Called after every connect/disconnect. Preserves
    /// an already-running idle anchor (only arms it on the busy→idle transition).
    /// LINGERING mirror entries do NOT count (process exit reclaims them). An
    /// active EGRESS registration DOES count — a producing graph keeps netd
    /// alive so its egress never silently drops. In practice egress is always held by
    /// a live connection, so `active_conns` already covers it; the explicit egress
    /// term is the belt-and-suspenders contract against the idle-exit wedge
    /// class (and is directly oracle-tested).
    fn refresh_idle(&mut self, now: Instant) {
        if self.is_busy() {
            self.idle_since = None;
        } else if self.idle_since.is_none() {
            self.idle_since = Some(now);
        }
    }

    /// Whether netd has any live work pinning it alive: a live connection OR an
    /// active egress registration.
    fn is_busy(&self) -> bool {
        self.active_conns > 0 || !self.egress.is_empty()
    }

    /// Whether netd has been idle (no connections) for at least `grace` as of
    /// `now` — the self-exit decision. `false` while busy. LINGERING entries do NOT
    /// keep netd alive (see the module docs).
    pub fn should_self_exit(&self, now: Instant, grace: Duration) -> bool {
        match self.idle_since {
            Some(t) => now.saturating_duration_since(t) >= grace,
            None => false,
        }
    }

    /// Whether netd is currently idle (no live connections). The idle-grace timer
    /// toward self-exit is running while this is `true`.
    pub fn is_idle(&self) -> bool {
        self.idle_since.is_some()
    }

    /// The idle anchor (`Some(t)` while idle since `t`). Diagnostics / tests.
    pub fn idle_since(&self) -> Option<Instant> {
        self.idle_since
    }

    /// The number of live consumer connections.
    pub fn active_connections(&self) -> usize {
        self.active_conns
    }

    /// The number of physical mirror bridges (active + lingering). This is
    /// `topics.len()` — every entry is a real (or lingering) bridge.
    pub fn mirror_count(&self) -> usize {
        self.topics.len()
    }

    /// The number of ACTIVELY-demanded mirrors (refcount > 0).
    pub fn active_demand_count(&self) -> usize {
        self.topics.values().filter(|s| s.refcount > 0).count()
    }

    /// The number of LINGERING mirrors (refcount 0, bridge still present — C1's
    /// leaked-until-idle set; C2 retires them at the last release).
    pub fn lingering_count(&self) -> usize {
        self.topics.values().filter(|s| s.refcount == 0).count()
    }

    /// The refcount for `key` (0 if not demanded — including a lingering entry).
    /// Diagnostics / tests.
    pub fn refcount(&self, key: &TopicKey) -> usize {
        self.topics.get(key).map(|s| s.refcount).unwrap_or(0)
    }

    /// Whether `key` has a physical mirror (active OR lingering). Diagnostics / tests.
    pub fn has_mirror(&self, key: &TopicKey) -> bool {
        self.topics
            .get(key)
            .map(|s| s.mirror_present)
            .unwrap_or(false)
    }

    /// Whether `key` is currently LINGERING — refcount 0 AND the physical mirror is
    /// present. The daemon's disconnect teardown re-checks THIS under the lock
    /// before any destructive `release_mirror`: a demand may have re-claimed the key
    /// (refcount 0→1) in the gap since `disconnect`, and a re-claimed key must NOT
    /// have its (now-live) bridge torn down (the C2 race guard).
    pub fn is_lingering(&self, key: &TopicKey) -> bool {
        self.topics
            .get(key)
            .map(|s| s.refcount == 0 && s.mirror_present)
            .unwrap_or(false)
    }

    /// A deterministic, sorted snapshot of the ACTIVE demand table — every
    /// actively-demanded `(robot, topic)` (refcount > 0) with its refcount
    /// (Principle #3, the `status` verb's source). Lingering entries (refcount 0)
    /// are EXCLUDED — `status` reports what consumers currently want.
    pub fn snapshot(&self) -> Vec<(TopicKey, usize)> {
        self.topics
            .iter()
            .filter(|(_, s)| s.refcount > 0)
            .map(|(k, s)| (k.clone(), s.refcount))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HASH_A: u64 = 0xA;
    const HASH_B: u64 = 0xB;

    fn key(robot: &str, topic: &str) -> TopicKey {
        TopicKey::new(robot, topic)
    }

    /// Model the daemon's demand path: on FirstDemand the daemon ensures the mirror
    /// and (on success) marks it present. This helper folds that so the tests
    /// exercise the SAME lingering behavior the daemon produces.
    fn demand_and_ensure(
        reg: &mut DemandRegistry,
        conn: ConnId,
        k: &TopicKey,
        hash: u64,
    ) -> DemandOutcome {
        let now = Instant::now();
        let outcome = reg.demand(conn, k.clone(), hash, now);
        if let DemandOutcome::FirstDemand { .. } = outcome {
            reg.mark_mirror_present(k); // ensure succeeded
        }
        outcome
    }

    #[test]
    fn invalidation_removes_old_holds_without_touching_lan_or_egress() {
        let now = Instant::now();
        let mut reg = DemandRegistry::new(now);
        let old = reg.connect(now);
        let other = reg.connect(now);
        let wan = key("account-robot", "/camera");
        let lan = key("lan-robot", "/imu");
        demand_and_ensure(&mut reg, old, &wan, HASH_A);
        demand_and_ensure(&mut reg, other, &wan, HASH_A);
        demand_and_ensure(&mut reg, old, &lan, HASH_A);
        assert!(matches!(
            reg.register_egress(old, vec!["/local/output".into()]),
            RegisterEgressOutcome::Registered { .. }
        ));
        assert_eq!(
            reg.invalidate_mirrors(&BTreeSet::from([wan.clone()])),
            Ok(vec![wan.clone()])
        );
        assert_eq!(reg.snapshot(), vec![(lan.clone(), 1)]);
        assert_eq!(reg.active_connections(), 2);
        assert_eq!(reg.active_egress_registration_count(), 1);
        assert!(!reg.is_idle());
        assert_eq!(
            reg.demand(other, wan.clone(), HASH_B, now),
            DemandOutcome::FirstDemand { refcount: 1 }
        );
        reg.mark_mirror_present(&wan);
        let closed = reg.disconnect(old, now);
        assert_eq!(closed.mirror_last_released, vec![lan]);
        assert_eq!(closed.egress_released, vec!["/local/output"]);
        assert_eq!(
            reg.refcount(&wan),
            1,
            "an old generation's close cannot decrement the replacement"
        );
        assert_eq!(reg.disconnect(other, now).mirror_last_released, vec![wan]);
    }

    #[test]
    fn invalidation_waits_for_teardown_then_clears_holds_in_sorted_order() {
        let now = Instant::now();
        let mut reg = DemandRegistry::new(now);
        let conn = reg.connect(now);
        let first = key("robot", "/a");
        let second = key("robot", "/z");
        demand_and_ensure(&mut reg, conn, &second, HASH_A);
        demand_and_ensure(&mut reg, conn, &first, HASH_A);
        reg.release(conn, &second, now);
        assert!(reg.claim_tearing(&second));
        let selected = BTreeSet::from([second.clone(), first.clone(), key("missing", "/topic")]);
        assert_eq!(reg.invalidate_mirrors(&selected), Err(vec![second.clone()]));
        assert_eq!(
            reg.refcount(&first),
            1,
            "a busy result makes no partial change"
        );
        assert!(reg.is_tearing(&second));
        // The original teardown completes before an invalidation can admit a
        // replacement. This case models a lingering result; a retired one is absent.
        reg.clear_tearing(&second);
        assert_eq!(
            reg.invalidate_mirrors(&selected),
            Ok(vec![first.clone(), second.clone()])
        );
        assert_eq!(reg.mirror_count(), 0);
        assert_eq!(reg.invalidate_mirrors(&selected), Ok(vec![]));
        assert_eq!(reg.release(conn, &first, now), ReleaseOutcome::NotHeld);
        assert_eq!(reg.release(conn, &second, now), ReleaseOutcome::NotHeld);
        assert!(reg.disconnect(conn, now).mirror_last_released.is_empty());
        assert!(reg.is_idle());
    }

    #[test]
    fn canonical_topic_normalizes_leading_slash_and_whitespace() {
        assert_eq!(canonical_topic("/tf"), "/tf");
        assert_eq!(canonical_topic("tf"), "/tf");
        assert_eq!(
            canonical_topic("  /utlidar/robot_odom  "),
            "/utlidar/robot_odom"
        );
        assert_eq!(canonical_topic("//double"), "/double");
        // Degenerate all-slash / empty inputs.
        assert_eq!(canonical_topic(""), "");
        assert_eq!(canonical_topic("   "), "");
        assert_eq!(canonical_topic("/"), "/");
        assert_eq!(canonical_topic("///"), "/");
    }

    #[test]
    fn topic_key_canonicalizes_so_slash_variants_are_one_mirror() {
        assert_eq!(key("ubuntu", "/tf"), key("ubuntu", "tf"));
        assert_eq!(key("  ubuntu  ", "/tf"), key("ubuntu", "tf"));
        // Distinct robots or topics are distinct keys.
        assert_ne!(key("ubuntu", "/tf"), key("go2", "/tf"));
        assert_ne!(key("ubuntu", "/tf"), key("ubuntu", "/tf_static"));
    }

    #[test]
    fn new_boots_idle_and_connect_disarms() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        // Boots idle: a never-demanded netd self-exits after the grace.
        assert!(reg.is_idle());
        assert_eq!(reg.idle_since(), Some(t0));
        assert!(reg.should_self_exit(t0 + Duration::from_secs(30), Duration::from_secs(30)));

        let c = reg.connect(t0 + Duration::from_secs(1));
        assert_eq!(c, 0);
        assert!(!reg.is_idle(), "a live connection is not idle");
        assert_eq!(reg.idle_since(), None);
        assert_eq!(reg.active_connections(), 1);
        // Even long after boot, a live connection never self-exits.
        assert!(!reg.should_self_exit(t0 + Duration::from_secs(1000), Duration::from_secs(30)));
    }

    #[test]
    fn connect_assigns_monotonic_ids() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        assert_eq!(reg.connect(t0), 0);
        assert_eq!(reg.connect(t0), 1);
        assert_eq!(reg.connect(t0), 2);
        assert_eq!(reg.active_connections(), 3);
    }

    #[test]
    fn first_demand_then_second_connection_shares_one_mirror() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let b = reg.connect(t0);
        let k = key("ubuntu", "/utlidar/robot_odom");

        // First demand → the daemon must register the mirror.
        assert_eq!(
            demand_and_ensure(&mut reg, a, &k, HASH_A),
            DemandOutcome::FirstDemand { refcount: 1 }
        );
        assert_eq!(reg.refcount(&k), 1);
        assert_eq!(reg.mirror_count(), 1);
        assert!(reg.has_mirror(&k));

        // Second connection demands the SAME key → joins the existing mirror (no
        // second registration).
        assert_eq!(
            demand_and_ensure(&mut reg, b, &k, HASH_A),
            DemandOutcome::AlreadyMirrored { refcount: 2 }
        );
        assert_eq!(reg.refcount(&k), 2);
        assert_eq!(reg.mirror_count(), 1, "still ONE mirror for two demanders");
    }

    #[test]
    fn same_connection_redemand_is_idempotent() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        assert_eq!(
            demand_and_ensure(&mut reg, a, &k, HASH_A),
            DemandOutcome::FirstDemand { refcount: 1 }
        );
        // The SAME connection re-demanding does NOT inflate the refcount.
        assert_eq!(
            demand_and_ensure(&mut reg, a, &k, HASH_A),
            DemandOutcome::AlreadyMirrored { refcount: 1 }
        );
        assert_eq!(
            reg.refcount(&k),
            1,
            "a buggy re-demand cannot pin the mirror"
        );
    }

    #[test]
    fn schema_conflict_is_refused_without_state_change() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let b = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        assert_eq!(
            demand_and_ensure(&mut reg, a, &k, HASH_A),
            DemandOutcome::FirstDemand { refcount: 1 }
        );
        // A second demander with a DIFFERENT schema for the same topic is refused.
        assert_eq!(
            demand_and_ensure(&mut reg, b, &k, HASH_B),
            DemandOutcome::SchemaConflict {
                existing: HASH_A,
                requested: HASH_B
            }
        );
        // No state change: still one demander, the original hash stands.
        assert_eq!(reg.refcount(&k), 1);
        // The conflicting connection holds nothing (so a later disconnect releases nothing).
        assert_eq!(reg.disconnect(b, t0), DisconnectOutcome::default());
        assert_eq!(reg.refcount(&k), 1);
    }

    #[test]
    fn release_decrements_then_last_release_lingers_the_mirror() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let b = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        demand_and_ensure(&mut reg, a, &k, HASH_A);
        demand_and_ensure(&mut reg, b, &k, HASH_A);
        assert_eq!(reg.refcount(&k), 2);

        // b releases → still demanded by a.
        assert_eq!(
            reg.release(b, &k, t0),
            ReleaseOutcome::StillDemanded { refcount: 1 }
        );
        assert_eq!(reg.refcount(&k), 1);
        assert_eq!(reg.mirror_count(), 1);

        // a releases → LAST release. The physical mirror LINGERS (C1 no-op
        // teardown): the entry is kept at refcount 0, mirror still present.
        assert_eq!(reg.release(a, &k, t0), ReleaseOutcome::LastRelease);
        assert_eq!(reg.refcount(&k), 0);
        assert_eq!(
            reg.mirror_count(),
            1,
            "the bridge lingers (kept until retire/exit)"
        );
        assert_eq!(reg.lingering_count(), 1);
        assert_eq!(reg.active_demand_count(), 0);
        assert!(reg.has_mirror(&k));
    }

    #[test]
    fn redemand_of_a_lingering_mirror_reuses_it_without_re_ensuring() {
        // THE fix: demand → release → re-demand must REUSE the lingering bridge
        // (AlreadyMirrored, no FirstDemand → no re-ensure → no register error).
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        assert_eq!(
            demand_and_ensure(&mut reg, a, &k, HASH_A),
            DemandOutcome::FirstDemand { refcount: 1 }
        );
        assert_eq!(reg.release(a, &k, t0), ReleaseOutcome::LastRelease);
        assert_eq!(reg.lingering_count(), 1);

        // Re-demand the SAME key → AlreadyMirrored (reuse), NOT FirstDemand.
        assert_eq!(
            reg.demand(a, k.clone(), HASH_A, t0),
            DemandOutcome::AlreadyMirrored { refcount: 1 }
        );
        assert_eq!(reg.refcount(&k), 1);
        assert_eq!(
            reg.lingering_count(),
            0,
            "the lingering entry is now active"
        );
        assert_eq!(reg.active_demand_count(), 1);
    }

    #[test]
    fn redemand_of_a_lingering_mirror_with_a_different_schema_conflicts() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        demand_and_ensure(&mut reg, a, &k, HASH_A);
        reg.release(a, &k, t0); // lingering with HASH_A
                                // A re-demand with a DIFFERENT hash still conflicts (the lingering bridge
                                // validates against HASH_A — one topic, one schema).
        assert_eq!(
            reg.demand(a, k.clone(), HASH_B, t0),
            DemandOutcome::SchemaConflict {
                existing: HASH_A,
                requested: HASH_B
            }
        );
    }

    #[test]
    fn retire_removes_a_lingering_entry_but_not_a_redemanded_one() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        demand_and_ensure(&mut reg, a, &k, HASH_A);
        reg.release(a, &k, t0); // lingering
                                // C2's teardown path: retire removes the lingering entry.
        assert!(reg.retire(&k));
        assert_eq!(reg.mirror_count(), 0);
        // Retire of an absent key is a no-op.
        assert!(!reg.retire(&k));

        // Guard: a key re-demanded before retire (refcount 1) is NOT retired.
        demand_and_ensure(&mut reg, a, &k, HASH_A);
        assert!(
            !reg.retire(&k),
            "a live (refcount>0) mirror is never retired"
        );
        assert_eq!(reg.refcount(&k), 1);
    }

    #[test]
    fn claim_tearing_marks_a_lingering_key_and_blocks_a_second_claim() {
        // The FIRST teardown claims a lingering key (marks `tearing`); a
        // SECOND concurrent teardown claim is refused (no-op) so no double
        // release_mirror + no spurious loud error. A non-lingering key can't be
        // claimed.
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let k = key("ubuntu", "/tf");

        // Not lingering (never demanded) → cannot claim.
        assert!(!reg.claim_tearing(&k), "an absent key cannot be claimed");

        demand_and_ensure(&mut reg, a, &k, HASH_A);
        // Actively demanded (refcount 1) → cannot claim (only lingering keys tear down).
        assert!(
            !reg.claim_tearing(&k),
            "a live key is never claimed for teardown"
        );

        reg.release(a, &k, t0); // now lingering (refcount 0, mirror_present)
        assert!(!reg.is_tearing(&k));
        // First claim wins.
        assert!(
            reg.claim_tearing(&k),
            "the first teardown claims a lingering key"
        );
        assert!(reg.is_tearing(&k));
        // Second claim is a no-op (a concurrent teardown must not fire a second
        // release_mirror).
        assert!(
            !reg.claim_tearing(&k),
            "a second concurrent teardown claim is refused (no-op)"
        );
    }

    #[test]
    fn a_demand_for_a_tearing_key_returns_retiring_without_state_change() {
        // While a key is `tearing`, a demand for it returns Retiring WITHOUT
        // reusing the dying bridge (no refcount change) — the daemon waits then
        // re-demands into a fresh mirror. A DIFFERENT-hash demand also gets Retiring
        // (the tearing schema is about to vanish), not a SchemaConflict.
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let b = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        demand_and_ensure(&mut reg, a, &k, HASH_A);
        reg.release(a, &k, t0); // lingering
        assert!(reg.claim_tearing(&k)); // tearing

        // Same-hash demand by another connection → Retiring, refcount unchanged (0).
        assert_eq!(
            reg.demand(b, k.clone(), HASH_A, t0),
            DemandOutcome::Retiring
        );
        assert_eq!(
            reg.refcount(&k),
            0,
            "Retiring never increments the refcount"
        );
        // Different-hash demand also Retiring (not SchemaConflict — the schema is
        // about to be torn down).
        assert_eq!(
            reg.demand(b, k.clone(), HASH_B, t0),
            DemandOutcome::Retiring
        );
        assert_eq!(reg.refcount(&k), 0);
    }

    #[test]
    fn clear_tearing_reopens_a_lingering_bridge_for_reuse() {
        // A Lingering (teardown-failure) result clears `tearing` so the kept
        // bridge is reusable again — the next demand takes AlreadyMirrored, not
        // Retiring.
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        demand_and_ensure(&mut reg, a, &k, HASH_A);
        reg.release(a, &k, t0);
        assert!(reg.claim_tearing(&k));
        assert!(reg.is_tearing(&k));

        reg.clear_tearing(&k); // teardown failed → keep the bridge, reopen it
        assert!(!reg.is_tearing(&k));
        assert_eq!(
            reg.lingering_count(),
            1,
            "the bridge is kept (still lingering)"
        );
        // A demand now REUSES it (no Retiring).
        assert_eq!(
            reg.demand(a, k.clone(), HASH_A, t0),
            DemandOutcome::AlreadyMirrored { refcount: 1 }
        );
    }

    #[test]
    fn retiring_then_retire_lets_a_redemand_recreate_a_fresh_mirror() {
        // The full teardown cycle at the registry level: claim → (teardown) → retire →
        // a re-demand of the SAME key is a FRESH FirstDemand (a real new mirror), NOT
        // a phantom AlreadyMirrored on a dead bridge.
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        demand_and_ensure(&mut reg, a, &k, HASH_A);
        reg.release(a, &k, t0);
        assert!(reg.claim_tearing(&k));
        // A demand during tearing waits (Retiring) — no reuse.
        assert_eq!(
            reg.demand(a, k.clone(), HASH_A, t0),
            DemandOutcome::Retiring
        );
        // Teardown finished (Retired) → retire removes the entry.
        assert!(reg.retire(&k));
        assert_eq!(reg.mirror_count(), 0);
        // The re-demand is now a fresh FirstDemand (the daemon re-ensures a REAL mirror).
        assert_eq!(
            reg.demand(a, k.clone(), HASH_A, t0),
            DemandOutcome::FirstDemand { refcount: 1 }
        );
    }

    #[test]
    fn failed_ensure_rollback_removes_the_entry_no_lingering() {
        // A FirstDemand whose ensure FAILED: the daemon does NOT mark_mirror_present
        // and calls release() to roll back. The entry must be REMOVED (not linger),
        // since no physical bridge exists.
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        assert_eq!(
            reg.demand(a, k.clone(), HASH_A, t0),
            DemandOutcome::FirstDemand { refcount: 1 }
        );
        // (ensure FAILED — no mark_mirror_present)
        assert!(!reg.has_mirror(&k), "no bridge yet");
        assert_eq!(reg.release(a, &k, t0), ReleaseOutcome::LastRelease);
        assert_eq!(
            reg.mirror_count(),
            0,
            "a failed ensure leaves NO lingering entry"
        );
        assert_eq!(reg.lingering_count(), 0);
        // A later demand for the SAME key is a fresh FirstDemand (re-ensure needed).
        assert_eq!(
            reg.demand(a, k.clone(), HASH_B, t0),
            DemandOutcome::FirstDemand { refcount: 1 }
        );
    }

    #[test]
    fn release_not_held_and_unknown_connection_are_noops() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        // Releasing something never demanded.
        assert_eq!(reg.release(a, &k, t0), ReleaseOutcome::NotHeld);
        // Unknown connection ids.
        assert_eq!(reg.release(999, &k, t0), ReleaseOutcome::UnknownConnection);
        assert_eq!(
            reg.demand(999, k.clone(), HASH_A, t0),
            DemandOutcome::UnknownConnection
        );
        assert_eq!(reg.mirror_count(), 0, "no state was created");
    }

    #[test]
    fn disconnect_releases_all_held_demands_and_reports_last_releases() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let k1 = key("ubuntu", "/tf");
        let k2 = key("ubuntu", "/utlidar/robot_odom");
        demand_and_ensure(&mut reg, a, &k1, HASH_A);
        demand_and_ensure(&mut reg, a, &k2, HASH_B);
        assert_eq!(reg.mirror_count(), 2);

        // a disconnects (crash / clean) → BOTH are last-released (sorted). The
        // bridges LINGER (C1) — the daemon's release_mirror decides retirement.
        let released = reg.disconnect(a, t0);
        assert_eq!(released.mirror_last_released, {
            let mut v = vec![k1, k2];
            v.sort();
            v
        });
        assert!(released.egress_released.is_empty(), "no egress held");
        assert_eq!(reg.active_demand_count(), 0);
        assert_eq!(
            reg.lingering_count(),
            2,
            "both bridges linger until retire/exit"
        );
        assert_eq!(reg.active_connections(), 0);
    }

    #[test]
    fn disconnect_of_one_sharer_does_not_last_release_a_still_held_key() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let b = reg.connect(t0);
        let k = key("ubuntu", "/tf");
        demand_and_ensure(&mut reg, a, &k, HASH_A);
        demand_and_ensure(&mut reg, b, &k, HASH_A);

        // a disconnects → the key is STILL held by b, so it is NOT last-released.
        let released = reg.disconnect(a, t0);
        assert!(released.mirror_last_released.is_empty(), "b still holds it");
        assert_eq!(reg.refcount(&k), 1);
        assert_eq!(reg.mirror_count(), 1);

        // b disconnects → NOW it is last-released (and lingers).
        let released = reg.disconnect(b, t0);
        assert_eq!(released.mirror_last_released, vec![k.clone()]);
        assert_eq!(reg.lingering_count(), 1);
        assert_eq!(reg.refcount(&k), 0);
    }

    #[test]
    fn double_disconnect_is_a_noop() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        assert_eq!(reg.disconnect(a, t0), DisconnectOutcome::default());
        // A second disconnect of the same id is a safe no-op (does not underflow
        // active_conns).
        assert_eq!(reg.disconnect(a, t0), DisconnectOutcome::default());
        assert_eq!(reg.active_connections(), 0);
    }

    #[test]
    fn idle_arms_on_last_disconnect_and_self_exits_after_grace() {
        let t0 = Instant::now();
        let grace = Duration::from_secs(30);
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        demand_and_ensure(&mut reg, a, &key("ubuntu", "/tf"), HASH_A);
        assert!(!reg.is_idle());

        // Last connection closes at t = 100s → idle anchor at 100s.
        let t_close = t0 + Duration::from_secs(100);
        reg.disconnect(a, t_close);
        assert!(reg.is_idle());
        assert_eq!(reg.idle_since(), Some(t_close));

        // Not yet: 29s idle.
        assert!(!reg.should_self_exit(t_close + Duration::from_secs(29), grace));
        // Exactly at the grace: self-exit.
        assert!(reg.should_self_exit(t_close + grace, grace));
        // And beyond.
        assert!(reg.should_self_exit(t_close + Duration::from_secs(31), grace));
    }

    #[test]
    fn a_lingering_mirror_does_not_block_idle_self_exit() {
        // The key idle-with-lingering pin: after the last consumer disconnects, the
        // bridge LINGERS (C1), but idle is CONNECTION-based, so netd still
        // self-exits after the grace (process exit reclaims the lingering bridge).
        let t0 = Instant::now();
        let grace = Duration::from_secs(30);
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        demand_and_ensure(&mut reg, a, &key("ubuntu", "/tf"), HASH_A);
        let t_close = t0 + Duration::from_secs(100);
        let released = reg.disconnect(a, t_close);
        assert_eq!(released.mirror_last_released.len(), 1);
        assert_eq!(reg.lingering_count(), 1, "the bridge lingers");
        assert!(
            reg.is_idle(),
            "no connections ⇒ idle despite the lingering bridge"
        );
        assert!(
            reg.should_self_exit(t_close + grace, grace),
            "a lingering bridge does NOT keep netd alive"
        );
    }

    #[test]
    fn a_new_connection_before_grace_cancels_the_self_exit() {
        let t0 = Instant::now();
        let grace = Duration::from_secs(30);
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let t_close = t0 + Duration::from_secs(100);
        reg.disconnect(a, t_close);
        assert!(reg.is_idle());

        // A new consumer connects 10s into the grace → busy again, timer cleared.
        let t_reconnect = t_close + Duration::from_secs(10);
        let _b = reg.connect(t_reconnect);
        assert!(!reg.is_idle());
        assert!(!reg.should_self_exit(t_reconnect + Duration::from_secs(100), grace));
    }

    #[test]
    fn demand_while_connected_does_not_touch_the_idle_timer() {
        // Demands/releases happen WHILE connected, so they never arm/disarm idle —
        // only connect/disconnect do. (A connected consumer is never idle.)
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        assert_eq!(reg.idle_since(), None);
        demand_and_ensure(&mut reg, a, &key("ubuntu", "/tf"), HASH_A);
        assert_eq!(
            reg.idle_since(),
            None,
            "demand does not idle a live connection"
        );
        reg.release(a, &key("ubuntu", "/tf"), t0 + Duration::from_secs(6));
        assert_eq!(
            reg.idle_since(),
            None,
            "release of a live connection stays busy"
        );
    }

    #[test]
    fn snapshot_reports_only_active_demands_sorted() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let b = reg.connect(t0);
        // Insert out of order.
        demand_and_ensure(&mut reg, a, &key("ubuntu", "/tf_static"), HASH_B);
        demand_and_ensure(&mut reg, a, &key("ubuntu", "/tf"), HASH_A);
        demand_and_ensure(&mut reg, b, &key("go2", "/odom"), HASH_A);
        demand_and_ensure(&mut reg, b, &key("ubuntu", "/tf"), HASH_A); // /tf → refcount 2

        // Now release /tf_static → it LINGERS but must NOT appear in `status`.
        reg.release(a, &key("ubuntu", "/tf_static"), t0);
        assert_eq!(reg.lingering_count(), 1);

        let snap = reg.snapshot();
        // Sorted by (robot, topic), ACTIVE only: go2/odom, ubuntu/tf (refcount 2).
        let expected = vec![(key("go2", "/odom"), 1), (key("ubuntu", "/tf"), 2)];
        assert_eq!(
            snap, expected,
            "status shows what consumers WANT (active), not lingering"
        );
    }

    // ─── Egress registration + cross-plan loop guard + liveness ───

    /// Model the daemon's egress path: register + (on plane success) mark announced,
    /// so the tests exercise the SAME lingering-announced behavior the daemon produces.
    fn register_and_announce(reg: &mut DemandRegistry, conn: ConnId, topics: &[&str]) {
        let owned: Vec<String> = topics.iter().map(|s| s.to_string()).collect();
        match reg.register_egress(conn, owned) {
            RegisterEgressOutcome::Registered { added, .. } => reg.mark_egress_announced(&added),
            other => panic!("expected Registered, got {other:?}"),
        }
    }

    #[test]
    fn register_egress_records_topics_scoped_per_connection() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let b = reg.connect(t0);

        // a registers two produced topics (canonicalized).
        match reg.register_egress(a, vec!["/robot/odom".into(), "robot/imu".into()]) {
            RegisterEgressOutcome::Registered { added, total } => {
                assert_eq!(
                    added,
                    vec!["/robot/imu".to_string(), "/robot/odom".to_string()]
                );
                assert_eq!(total, 2);
            }
            other => panic!("expected Registered, got {other:?}"),
        }
        assert!(reg.is_egress_registered("/robot/odom"));
        assert!(
            reg.is_egress_registered("robot/imu"),
            "canonicalized membership"
        );
        assert_eq!(reg.active_egress_registration_count(), 1);
        assert_eq!(reg.egress_topic_count(), 2);

        // b registers a DISTINCT topic — per-connection scoping (b's registration
        // never touches a's).
        assert!(matches!(
            reg.register_egress(b, vec!["/robot/scan".into()]),
            RegisterEgressOutcome::Registered { total: 1, .. }
        ));
        assert_eq!(reg.active_egress_registration_count(), 2);
        assert_eq!(reg.egress_topic_count(), 3);

        // Unknown connection is refused, no state change.
        assert_eq!(
            reg.register_egress(999, vec!["/x".into()]),
            RegisterEgressOutcome::UnknownConnection
        );
        assert_eq!(reg.egress_topic_count(), 3);
    }

    #[test]
    fn register_egress_is_additive_and_dedups_per_connection() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        reg.register_egress(a, vec!["/t1".into()]);
        // Re-register: /t1 already held (no new add), /t2 is new.
        match reg.register_egress(a, vec!["/t1".into(), "/t2".into()]) {
            RegisterEgressOutcome::Registered { added, total } => {
                assert_eq!(
                    added,
                    vec!["/t2".to_string()],
                    "only the new topic is added"
                );
                assert_eq!(total, 2);
            }
            other => panic!("expected Registered, got {other:?}"),
        }
        assert_eq!(reg.egress_topic_count(), 2);
        // Empty / all-slash topics are dropped, not registered.
        assert!(matches!(
            reg.register_egress(a, vec!["".into(), "   ".into()]),
            RegisterEgressOutcome::Registered { total: 2, .. }
        ));
        assert_eq!(reg.egress_topic_count(), 2);
    }

    #[test]
    fn cross_plan_loop_guard_refuses_both_directions() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let b = reg.connect(t0);

        // (1) EGRESS direction: a demands (mirrors IN) /tf from robot ubuntu, then b
        //     tries to egress-register /tf → refused LOUDLY naming the topic + holder.
        demand_and_ensure(&mut reg, a, &key("ubuntu", "/tf"), HASH_A);
        match reg.register_egress(b, vec!["/sensor/x".into(), "tf".into()]) {
            RegisterEgressOutcome::LoopConflict { topic, robot } => {
                assert_eq!(topic, "/tf");
                assert_eq!(robot, "ubuntu");
            }
            other => panic!("expected LoopConflict, got {other:?}"),
        }
        // NO partial state change — the sibling /sensor/x was NOT registered.
        assert!(!reg.is_egress_registered("/sensor/x"));
        assert_eq!(reg.egress_topic_count(), 0);

        // (2) DEMAND direction: b egress-registers /cmd_vel, then a tries to demand
        //     (mirror IN) /cmd_vel → refused with EgressConflict, no refcount.
        reg.register_egress(b, vec!["/cmd_vel".into()]);
        assert_eq!(
            reg.demand(a, key("go2", "/cmd_vel"), HASH_B, t0),
            DemandOutcome::EgressConflict {
                topic: "/cmd_vel".to_string()
            }
        );
        assert_eq!(reg.refcount(&key("go2", "/cmd_vel")), 0);
        assert!(
            !reg.has_mirror(&key("go2", "/cmd_vel")),
            "the conflicting demand created no mirror"
        );
    }

    #[test]
    fn loop_guard_blocks_egress_of_a_lingering_mirror_topic() {
        // A lingering mirror (refcount 0, mirror_present) still physically exists, so
        // egressing its topic would still echo-loop → refused.
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let b = reg.connect(t0);
        demand_and_ensure(&mut reg, a, &key("ubuntu", "/tf"), HASH_A);
        reg.release(a, &key("ubuntu", "/tf"), t0); // now lingering
        assert_eq!(reg.lingering_count(), 1);
        assert!(matches!(
            reg.register_egress(b, vec!["/tf".into()]),
            RegisterEgressOutcome::LoopConflict { .. }
        ));
    }

    #[test]
    fn rollback_egress_precisely_undoes_the_added_topics() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        reg.register_egress(a, vec!["/keep".into()]);
        // A second register adds /roll1 + /roll2; the daemon then rolls JUST those
        // back (a plane failure) — /keep survives.
        let added = match reg.register_egress(a, vec!["/roll1".into(), "/roll2".into()]) {
            RegisterEgressOutcome::Registered { added, .. } => added,
            other => panic!("expected Registered, got {other:?}"),
        };
        reg.rollback_egress(a, &added);
        assert!(
            reg.is_egress_registered("/keep"),
            "the prior topic survives"
        );
        assert!(!reg.is_egress_registered("/roll1"));
        assert!(!reg.is_egress_registered("/roll2"));
        assert_eq!(reg.egress_topic_count(), 1);
        assert_eq!(reg.active_egress_registration_count(), 1);

        // Rolling back the LAST topic removes the connection's whole entry (no empty
        // entry lingering to falsely pin liveness).
        reg.rollback_egress(a, &["/keep".to_string()]);
        assert_eq!(reg.active_egress_registration_count(), 0);
        assert_eq!(reg.egress_topic_count(), 0);
    }

    #[test]
    fn release_egress_verb_removes_the_connections_registration() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        reg.register_egress(a, vec!["/b".into(), "/a".into()]);
        // The explicit release returns the released topics (sorted).
        assert_eq!(
            reg.release_egress(a),
            vec!["/a".to_string(), "/b".to_string()]
        );
        assert_eq!(reg.active_egress_registration_count(), 0);
        // A second release (nothing held) is a no-op returning empty.
        assert!(reg.release_egress(a).is_empty());
        // A demand for a now-un-egressed topic is no longer loop-blocked.
        assert!(matches!(
            reg.demand(a, key("r", "/a"), HASH_A, t0),
            DemandOutcome::FirstDemand { .. }
        ));
    }

    #[test]
    fn disconnect_releases_egress_and_reports_it() {
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        demand_and_ensure(&mut reg, a, &key("ubuntu", "/odom"), HASH_A); // a demand too
        reg.register_egress(a, vec!["/pub2".into(), "/pub1".into()]);

        // Disconnect releases BOTH the demand (last-release → lingering) AND the
        // egress registration (crash-safe), reporting each.
        let out = reg.disconnect(a, t0);
        assert_eq!(out.mirror_last_released, vec![key("ubuntu", "/odom")]);
        assert_eq!(
            out.egress_released,
            vec!["/pub1".to_string(), "/pub2".to_string()]
        );
        assert_eq!(reg.active_egress_registration_count(), 0);
        assert_eq!(reg.egress_topic_count(), 0, "egress fully released");
    }

    #[test]
    fn released_egress_lingers_announced_and_still_blocks_a_demand() {
        // After register+announce then FULL release, the ACTIVE
        // accounting drops but the physical announce LINGERS (mirroring the add-only
        // bridge flag), so a demand for the topic is STILL refused EXPLICITLY — never
        // passing the loop guard to die late in create_ingress_publisher.
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        register_and_announce(&mut reg, a, &["/produced"]);
        assert!(reg.is_egress_registered("/produced"));
        assert_eq!(reg.announced_egress_topic_count(), 1);

        // Full release (the graph stopped producing): active accounting drops...
        assert_eq!(reg.release_egress(a), vec!["/produced".to_string()]);
        assert_eq!(reg.active_egress_registration_count(), 0);
        assert_eq!(reg.egress_topic_count(), 0, "no ACTIVE registration");
        // ...but the announce LINGERS (the flag is still set on the shared gateway).
        assert_eq!(reg.announced_egress_topic_count(), 1, "announce lingers");
        assert!(
            reg.is_egress_registered("/produced"),
            "the lingering announce still blocks a demand"
        );
        // A demand for the lingering-announced topic is refused EXPLICITLY (not a late
        // create_ingress_publisher death).
        assert_eq!(
            reg.demand(a, key("remote", "/produced"), 7, t0),
            DemandOutcome::EgressConflict {
                topic: "/produced".to_string()
            }
        );
        assert_eq!(
            reg.refcount(&key("remote", "/produced")),
            0,
            "no mirror created"
        );
    }

    #[test]
    fn re_register_of_a_lingering_announced_topic_by_a_new_connection_revives_it() {
        // The egress twin of the mirror plane's lingering-reuse revival: a NEW
        // connection re-registering a lingering-announced topic SUCCEEDS (its bridge
        // flag push is idempotent) — it is not in the demand table, so the loop guard
        // passes.
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let b = reg.connect(t0);
        register_and_announce(&mut reg, a, &["/x"]);
        reg.release_egress(a); // lingering-announced, no active conn
        assert_eq!(reg.active_egress_registration_count(), 0);

        // A DIFFERENT connection re-registers /x → Registered (revived), added carries
        // /x (its accounting is fresh for b).
        match reg.register_egress(b, vec!["/x".into()]) {
            RegisterEgressOutcome::Registered { added, total } => {
                assert_eq!(added, vec!["/x".to_string()], "revival re-adds the topic");
                assert_eq!(total, 1);
            }
            other => panic!("expected Registered (revival), got {other:?}"),
        }
        reg.mark_egress_announced(&["/x".to_string()]); // idempotent
        assert_eq!(reg.active_egress_registration_count(), 1, "b now holds it");
        assert!(reg.is_egress_registered("/x"));
        // And it STILL blocks a demand (active + announced).
        assert!(matches!(
            reg.demand(a, key("r", "/x"), 1, t0),
            DemandOutcome::EgressConflict { .. }
        ));
    }

    #[test]
    fn rollback_never_marks_announced_so_a_failed_push_leaves_no_lingering_block() {
        // A plane FAILURE path: register (records active) then rollback (no mark). The
        // topic must NOT linger-announced (no flag was pushed), so a later demand for
        // it is NOT falsely blocked.
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        let added = match reg.register_egress(a, vec!["/fail".into()]) {
            RegisterEgressOutcome::Registered { added, .. } => added,
            other => panic!("expected Registered, got {other:?}"),
        };
        // (the egress plane FAILED — the daemon rolls back WITHOUT marking announced)
        reg.rollback_egress(a, &added);
        assert_eq!(reg.announced_egress_topic_count(), 0, "no flag was pushed");
        assert!(
            !reg.is_egress_registered("/fail"),
            "no false lingering block"
        );
        // A demand for /fail now proceeds (a fresh mirror), not EgressConflict.
        assert!(matches!(
            reg.demand(a, key("r", "/fail"), 1, t0),
            DemandOutcome::FirstDemand { .. }
        ));
    }

    #[test]
    fn an_egress_registration_pins_liveness_until_released() {
        // THE wedge-class guard: a producing graph keeps netd alive so its
        // egress never silently drops, and releasing it lets netd idle → self-exit.
        let t0 = Instant::now();
        let grace = Duration::from_secs(30);
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        register_and_announce(&mut reg, a, &["/produced"]);
        assert!(!reg.is_idle(), "an egress registration is busy");
        assert!(!reg.should_self_exit(t0 + Duration::from_secs(1000), grace));

        // The connection drops (releasing the egress) → NOW idle → self-exit after
        // grace, EVEN THOUGH the topic lingers-announced (a lingering announce does
        // NOT pin liveness — process-exit reclaims it, exactly like a lingering
        // mirror; only an ACTIVE registration keeps netd alive).
        let t_close = t0 + Duration::from_secs(100);
        let out = reg.disconnect(a, t_close);
        assert_eq!(out.egress_released, vec!["/produced".to_string()]);
        assert_eq!(
            reg.announced_egress_topic_count(),
            1,
            "the announce lingers"
        );
        assert!(
            reg.is_idle(),
            "no connection + no ACTIVE egress ⇒ idle despite the lingering announce"
        );
        assert!(reg.should_self_exit(t_close + grace, grace));
    }

    #[test]
    fn is_busy_egress_term_is_load_bearing() {
        // Belt-and-suspenders: even with active_conns forced to 0 (a state the daemon
        // never reaches — egress is always conn-held), a NON-EMPTY egress map keeps
        // `is_busy` true, so `refresh_idle` never arms the self-exit anchor. This
        // pins the explicit egress term against the wedge class regardless of the
        // conn-held invariant.
        let t0 = Instant::now();
        let mut reg = DemandRegistry::new(t0);
        let a = reg.connect(t0);
        reg.register_egress(a, vec!["/p".into()]);
        assert!(reg.is_busy(), "an egress registration is busy");
        reg.active_conns = 0; // hostile: pretend the connection vanished
        assert!(
            reg.is_busy(),
            "the egress term alone keeps netd busy (conn count aside)"
        );
    }
}
