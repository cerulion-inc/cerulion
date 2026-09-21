// SPDX-License-Identifier: AGPL-3.0-only
//! The LIVE wire-native `~/get_type_description` rung.
//!
//! [`WireServiceAcquirer`] implements [`SchemaAcquirer`] over the existing
//! `ros2-client`/`rustdds` stack (zero ROS install). For each unresolvable
//! `pkg/Type` it:
//!
//! 1. CONSUMES the engine's already-harvested [`DiscoveryResult`] (endpoints +
//!    RIHS01 `type_hash` + `writer_guid`, the `ros_discovery_info` node table,
//!    and remote-participant vendor ids) — threaded in through
//!    [`SchemaAcquirer::acquire`]. It runs NO second discovery window;
//! 2. plans the service calls purely ([`wire::plan_service_calls`]);
//! 3. calls `type_description_interfaces/srv/GetTypeDescription` on the owning
//!    node with the vendor-appropriate correlation mapping (retrying other
//!    nodes / mappings on failure), bounded by per-call budgets AND one overall
//!    call-phase wall cap (`WIRE_CALL_PHASE_BUDGET`), with a per-node
//!    `wait_for_service` failure cache;
//! 4. maps the response's `type_sources[]` to verbatim `.msg` closures
//!    ([`wire::sources_to_closure`]).
//!
//! # One participant per process (Principle #8)
//!
//! This rung builds its OWN [`DiscoveryParticipant`] ONLY to make the SERVICE
//! CALLS (one node, one spinner), NOT to re-discover — and ONLY when the pure
//! plan over the consumed discovery yields at least one callable plan. A
//! guaranteed-skip network (no hashes / no nodes — e.g. the Go2) builds NO
//! participant at all. It runs AFTER the engine's discovery participant has been
//! dropped (the attach flow fully completes `DdsDiscovery::discover` before
//! `run_acquisition_ladder`), so the one-per-process slot is free and two
//! participants never coexist. The engine cannot hand a LIVE participant across
//! the DDS-free `cerulion_cli_engine` boundary, but it CAN hand the DDS-free
//! [`DiscoveryResult`] its one window already produced — so this rung consumes
//! that harvest rather than opening a redundant second window
//! (which every guaranteed-skip attach would pay for nothing, and which could
//! mis-diagnose a type present in window 1 but missed in window 2).
//!
//! # Purity
//!
//! Every DECISION (which node, which mapping, response → `.msg`, the per-attempt
//! retry / wait-failure cache / wall cap) lives in [`crate::wire`] and is
//! oracle-tested there. This file is the DDS I/O shell; its live service
//! round-trip is designed against the REP-2011 spec + the ros2-client 0.10
//! source and validated against a live ROS 2 Jazzy peer).
//!
//! [`DiscoveryResult`]: crate::DiscoveryResult

use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use ros2_client::ros2::{policy, Duration as DdsDuration, QosPolicies, QosPolicyBuilder};
use ros2_client::service::{AService, ServiceMapping};
use ros2_client::{Message, Name, Node, ServiceTypeName};

use crate::participant::DiscoveryParticipant;
use crate::{
    wire, AcquiredSchema, AcquisitionOutcome, AcquisitionRung, DiscoveryParams, DiscoveryResult,
    SchemaAcquirer, TypeAcquisition,
};

/// The overall wall cap on the wire rung's CALL phase — ONE deadline covering
/// every `get_type_description` service call for the whole attach. Checked
/// between plans, between attempts, AND before every
/// correlation-mapping try inside an attempt, with each wait CLAMPED to the
/// remaining wall time — so the cap holds up to a single in-flight service
/// call (≤ `call_budget`). A live read answers in milliseconds; this bound
/// exists so a LAN with many dead or silent nodes (each burning its per-call
/// budget) cannot make the rung run for minutes.
///
/// Why a whole-attach cap and not just per-call budgets: because
/// `ros2-client` keeps `NodeEntitiesInfo::writer_gid_seq` private, an endpoint
/// maps to its owning node at PARTICIPANT granularity — so ONE type's retry set
/// is *every* node hosted by the publisher's participant (`nodes_owning_writer`
/// returns them all). On a participant hosting many nodes, that per-type fan-out
/// times the per-wait budgets could stack up; the cold/warm wait split
/// (`WAIT_BUDGET_COLD`/`WAIT_BUDGET_WARM`) keeps the fan-out cheap after the
/// first wait, the per-node `wait_for_service` failure cache collapses the
/// common "dead participant" case to one wait, and this whole-attach cap bounds
/// the residual. 15s comfortably covers a healthy multi-type attach (a handful
/// of unresolvable types × a couple of seconds each); a type not reached before
/// the cap stays UNRESOLVABLE with a loud, distinct skip reason.
const WIRE_CALL_PHASE_BUDGET: Duration = Duration::from_secs(15);

/// Wait budget for the FIRST `wait_for_service` of the attach:
/// the wire rung builds a FRESH participant, and its SEDP bring-up to
/// a recorded service match measures ~1.2 s against a live ROS 2 peer. That cold cost is
/// OUR participant's, paid ONCE — not per node — so only the attach's first
/// wait carries the 4 s (measured + margin) budget.
const WAIT_BUDGET_COLD: Duration = Duration::from_secs(4);

/// Wait budget for every SUBSEQUENT wait: SEDP is warm after
/// the first wait, so a server that exists announces fast; 2 s bounds a
/// dead/absent node without letting the participant-granularity node fan-out
/// starve the wall cap (pre-split, a single dead participant hosting 2 silent
/// unknown-vendor nodes burned 4 s × 2 mappings × 2 nodes = 16 s — past the
/// 15 s `WIRE_CALL_PHASE_BUDGET` — so a reachable third node was never tried;
/// the split arithmetic is pinned by `wait_budget_split_leaves_wall_headroom`).
const WAIT_BUDGET_WARM: Duration = Duration::from_secs(2);

/// One slice of the bounded `wait_for_service` POLL LOOP (the guard
/// against a single-shot wait hanging on a lost
/// event). ros2-client 0.10.1's `Node::wait_for_reader`/`wait_for_writer` DO
/// subscribe their event stream BEFORE the already-present map check
/// (`status_receiver()` at node.rs:1129/:1151 precedes it), so there is no
/// check-then-subscribe gap; the REAL loss vector is `send_status_event`'s
/// `async_channel::bounded(8)` `try_send`, which DROPS on Full during
/// discovery bursts (node.rs ~:473) — with the node.rs:1161 TODO's
/// stream-miss speculation as a secondary. The `writers_to_remote_readers`
/// MAP, however, is updated reliably by the spin task (participant channel,
/// depth 2048) — so a single hung `wait_for_service` whose event was dropped
/// never recovers within one call, while a RE-INVOKED wait re-runs the
/// already-present map check and returns instantly once spin has recorded the
/// match (measured against a live peer: the match lands ~1.2 s after client
/// creation on a fresh participant; a single-shot wait only succeeds on a
/// retried second client for exactly this reason). Polling FRESH wait futures
/// every slice converts the dropped-event hang into at most one slice of extra
/// latency, without touching the WaitTimedOut/CallFailed classification.
const WAIT_POLL_SLICE: Duration = Duration::from_millis(250);

// ─────────────────── REP-2011 GetTypeDescription srv types ──────────────────
//
// The serde structs below model `type_description_interfaces/srv/
// GetTypeDescription` and its `msg/*` dependencies. CDR is POSITIONAL, so field
// ORDER + TYPE (not the Rust field names) must match the `.srv`/`.msg`
// definitions exactly — a mismatch misaligns the whole response deserialization.
// The layout follows the jazzy `rcl_interfaces`/`type_description_interfaces`
// definitions. We CONSUME only
// `type_sources`, but the full `TypeDescription` is on the wire regardless —
// and a real server ALWAYS populates it (any real type has fields), with those
// bytes sitting BEFORE `type_sources` in the CDR stream — so it is modeled
// faithfully to keep the deserializer aligned. The field ORDER of EVERY level
// (request, response, TypeDescription, IndividualTypeDescription, Field,
// FieldType incl. its 8-byte-aligned u64s, TypeSource, KeyValue) is pinned
// STRUCTURALLY by `tests::request_cdr_matches_hand_oracle` /
// `tests::response_cdr_decodes_hand_oracle_*`, which drive the exact
// ros2-client/rustdds CDR serializer/deserializer these structs travel through
// against hand-built byte vectors carrying a POPULATED Field/FieldType/
// referenced-type/KeyValue payload and assert full consumption — a swapped or
// retyped field at ANY level fails there, not only in a live decode.

/// `type_description_interfaces/srv/GetTypeDescription` Request.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct GetTypeDescriptionRequest {
    /// `PACKAGE/NAMESPACE/TYPENAME` — cosmetic (server looks up BY HASH).
    type_name: String,
    /// The REP-2011 RIHS hash string (the actual lookup key).
    type_hash: String,
    /// Whether to include the verbatim `type_sources[]` (we always request it).
    include_type_sources: bool,
}
impl Message for GetTypeDescriptionRequest {}

/// `type_description_interfaces/srv/GetTypeDescription` Response.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct GetTypeDescriptionResponse {
    successful: bool,
    failure_reason: String,
    // `type_description` (structured) + `extra_information` are on the wire but
    // this rung consumes only `type_sources` (the verbatim `.msg` text). They
    // are decoded to keep CDR alignment, never read — hence the allow.
    #[allow(dead_code)]
    type_description: TypeDescription,
    type_sources: Vec<TypeSource>,
    #[allow(dead_code)]
    extra_information: Vec<KeyValue>,
}
impl Message for GetTypeDescriptionResponse {}

// The structured `TypeDescription` tree below exists ONLY so the CDR
// deserializer stays byte-aligned while walking a response — this rung reads
// `type_sources`, not the structured form (idl-only types are not supported). The
// fields are decoded but never read by PRODUCTION code, so the whole tree is
// `#[allow(dead_code)]` (the CDR oracles in `tests` DO read every level to pin
// the byte layout — cfg(test) reads don't satisfy the non-test dead_code lint).

/// `type_description_interfaces/msg/TypeDescription`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[allow(dead_code)]
struct TypeDescription {
    type_description: IndividualTypeDescription,
    referenced_type_descriptions: Vec<IndividualTypeDescription>,
}

/// `type_description_interfaces/msg/IndividualTypeDescription`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[allow(dead_code)]
struct IndividualTypeDescription {
    type_name: String,
    fields: Vec<Field>,
}

/// `type_description_interfaces/msg/Field`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[allow(dead_code)]
struct Field {
    name: String,
    /// The `.msg` field is `type`; CDR is positional so the Rust name is free.
    field_type: FieldType,
    default_value: String,
}

/// `type_description_interfaces/msg/FieldType`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[allow(dead_code)]
struct FieldType {
    type_id: u8,
    capacity: u64,
    string_capacity: u64,
    nested_type_name: String,
}

/// `type_description_interfaces/msg/TypeSource` — the byte-verbatim source half.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct TypeSource {
    type_name: String,
    encoding: String,
    raw_file_contents: String,
}

/// `type_description_interfaces/msg/KeyValue`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[allow(dead_code)]
struct KeyValue {
    key: String,
    value: String,
}

/// The `Service` type for `~/get_type_description`. `AService<Req, Resp>` is the
/// ros2-client idiom (`create_client` derives the DDS type names from a
/// [`ServiceTypeName`], not from a `Service` value, so no value is constructed).
type GetTypeDescriptionService = AService<GetTypeDescriptionRequest, GetTypeDescriptionResponse>;

// ───────────────────────────── The acquirer ────────────────────────────────

/// The live wire-native schema-acquisition rung.
pub struct WireServiceAcquirer {
    /// Discovery params — the rung reuses the same interface + domain as the
    /// attach discovery to build its call participant. (Its `window` is unused
    /// here: the rung CONSUMES the engine's harvest and opens no window.)
    params: DiscoveryParams,
    /// Per-attempt bound on one `async_call_service` round-trip.
    call_budget: Duration,
}

impl WireServiceAcquirer {
    /// Construct over the attach discovery params. WAIT budgets are SPLIT
    /// cold/warm: the attach's FIRST wait gets `WAIT_BUDGET_COLD`
    /// (4 s — it pays our fresh participant's ~1.2 s SEDP bring-up, measured
    /// against a live peer, a once-per-attach cost); every subsequent wait gets
    /// `WAIT_BUDGET_WARM` (2 s — SEDP is warm, so an existing server announces
    /// fast). Each wait is a `WAIT_POLL_SLICE`-sliced POLL LOOP whose budget is
    /// additionally CLAMPED to the remaining `WIRE_CALL_PHASE_BUDGET` wall
    /// time, and the wall cap is checked before every plan, attempt, AND
    /// correlation-mapping try — so the 15 s call-phase cap holds up to a
    /// single in-flight service call (bounded by the CALL budget, the
    /// per-round-trip `async_call_service` bound; a live read on the same LAN
    /// answers in ms).
    pub fn new(params: DiscoveryParams) -> Self {
        Self {
            params,
            call_budget: Duration::from_secs(3),
        }
    }

    /// The `rmw_qos_profile_services_default`: RELIABLE, KEEP_LAST depth 10,
    /// VOLATILE (matches the QoS ROS 2 nodes serve `get_type_description` with).
    fn service_qos(&self) -> QosPolicies {
        QosPolicyBuilder::new()
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: DdsDuration::from_millis(100),
            })
            .durability(policy::Durability::Volatile)
            .history(policy::History::KeepLast { depth: 10 })
            .build()
    }

    /// PLAN purely over the CONSUMED `discovery` (no DDS), then — only when there
    /// is at least one callable plan — build the participant + node + spinner
    /// (for the CALLS only; no second discovery window) and execute the calls. A
    /// guaranteed-skip network (no hashes / no nodes) builds NO participant at
    /// all — the "pay for nothing" fix. Any DDS setup failure is an
    /// `Err(reason)` — the caller turns it into a per-type skip (never a silent
    /// empty result).
    fn run_calls(
        &self,
        types: &[String],
        discovery: &DiscoveryResult,
    ) -> Result<Vec<TypeAcquisition>, String> {
        let endpoints = &discovery.endpoints;
        let nodes = &discovery.nodes;
        let participants = &discovery.participants;
        tracing::info!(
            endpoints = endpoints.len(),
            nodes = nodes.len(),
            participants = participants.len(),
            requested = types.len(),
            "wire acquirer: consuming the attach discovery result (no second discovery window) — \
             planning get_type_description calls"
        );

        // A hash-bearing
        // endpoint whose PARTICIPANT has no node-table entry cannot be routed —
        // per-participant, not global-empty (the common bounded(8) tail-drop
        // leaves a PARTIALLY populated table). The wording names BOTH cause
        // classes; the compiled GID width (COMPILED_ROS_DISTRO vs Iron — the
        // `ros_discovery_info` encoding boundary) decides which is structural.
        if wire::should_warn_node_table_miss(endpoints, nodes) {
            let build_is_pre_iron_gid =
                ros2_client::COMPILED_ROS_DISTRO < ros2_client::RosDistro::Iron;
            tracing::warn!("{}", wire::node_table_miss_warning(build_is_pre_iron_gid));
        }

        // PURE planning first — no DDS I/O.
        let (plans, skips) = wire::plan_service_calls(types, endpoints, nodes, participants);

        let mut outcomes: Vec<TypeAcquisition> = Vec::new();
        // Types with no callable attempt: loud per-type skip (never discovered /
        // no hash / no owning node — each a DISTINCT reason).
        for (requested, skip) in skips {
            tracing::info!(
                ros_type = %requested,
                reason = %skip.reason,
                "wire acquirer: type not callable via get_type_description"
            );
            outcomes.push(TypeAcquisition {
                requested,
                outcome: AcquisitionOutcome::Skipped(vec![skip]),
            });
        }

        // Nothing callable ⇒ build NO participant (the "pay for nothing" fix).
        if plans.is_empty() {
            tracing::info!(
                "wire acquirer: no callable get_type_description plans — skipping participant \
                 creation entirely (nothing to ask)"
            );
            return Ok(outcomes);
        }

        // At least one plan ⇒ build the call participant + spinner and execute.
        let participant =
            DiscoveryParticipant::new(self.params.domain_id, &self.params.only_networks)
                .map_err(|e| format!("could not create DDS participant: {e}"))?;
        let node = participant
            .create_node("wire_acquirer")
            .map_err(|e| format!("could not create DDS node: {e}"))?;

        let call_outcomes = smol::block_on(async move {
            // Move the node in; take the spinner BEFORE the executor. No
            // `status_receiver()` — the wire rung consumes the engine's harvest
            // and runs no discovery collect loop; the spinner runs solely to
            // drive the service client's DDS I/O.
            let mut node = node;
            let spinner = node
                .spinner()
                .map_err(|e| format!("could not start the DDS node spinner: {e:?}"))?;

            let ex = smol::LocalExecutor::new();
            let spin_task = ex.spawn(async move {
                if let Err(e) = spinner.spin().await {
                    tracing::warn!(error = ?e, "wire acquirer: node spinner exited with error");
                }
            });

            let result = ex.run(self.execute_plans(&mut node, plans)).await;
            drop(spin_task);
            Ok::<_, String>(result)
        })?;

        // `node` + `participant` drop here → the one-per-process slot frees.
        outcomes.extend(call_outcomes);
        Ok(outcomes)
    }

    /// Execute the callable `plans` under the overall call-phase wall cap + the
    /// per-node wait-failure cache, assembling per-type outcomes. Runs on the one
    /// participant's node + spinner.
    async fn execute_plans(
        &self,
        node: &mut Node,
        plans: Vec<wire::TypeCallPlan>,
    ) -> Vec<TypeAcquisition> {
        let mut outcomes: Vec<TypeAcquisition> = Vec::new();

        // One service call per (node, hash) — cache responses across a run.
        let mut response_cache: BTreeMap<(String, String), Option<Vec<wire::WireTypeSource>>> =
            BTreeMap::new();
        // Per-attach wait_for_service failure cache: a node that failed to
        // present a server once is not re-waited for a later type.
        let mut wait_failed: BTreeSet<String> = BTreeSet::new();
        // Has the attach's FIRST wait run yet? The cold
        // (fresh-participant SEDP bring-up) budget applies to that one wait
        // only; every later wait is warm (`WAIT_BUDGET_WARM`).
        let mut first_wait_done = false;
        // The overall call-phase wall cap — one deadline over EVERY call.
        let wall_deadline = Instant::now() + WIRE_CALL_PHASE_BUDGET;

        let mut plan_iter = plans.into_iter();
        let mut wall_cap_hit = false;
        for plan in plan_iter.by_ref() {
            if wire::call_phase_budget_exceeded(Instant::now(), wall_deadline) {
                outcomes.push(wall_cap_skip_outcome(plan.requested));
                wall_cap_hit = true;
                break;
            }
            let outcome = self
                .resolve_plan(
                    node,
                    &plan,
                    wall_deadline,
                    &mut response_cache,
                    &mut wait_failed,
                    &mut first_wait_done,
                )
                .await;
            outcomes.push(outcome);
        }
        // Everything the wall cap cut off gets the same distinct skip.
        if wall_cap_hit {
            for plan in plan_iter {
                outcomes.push(wall_cap_skip_outcome(plan.requested));
            }
        }
        outcomes
    }

    /// Resolve ONE type's plan: try each attempt (skipping wait-cached nodes and
    /// honoring the wall cap), folding each result via the pure
    /// [`wire::apply_attempt_outcome`] (retry / last-error retention / wait-cache
    /// populate), and mapping the first successful response to an [`AcquiredSchema`].
    async fn resolve_plan(
        &self,
        node: &mut Node,
        plan: &wire::TypeCallPlan,
        wall_deadline: Instant,
        response_cache: &mut BTreeMap<(String, String), Option<Vec<wire::WireTypeSource>>>,
        wait_failed: &mut BTreeSet<String>,
        first_wait_done: &mut bool,
    ) -> TypeAcquisition {
        let mut acquired: Option<AcquiredSchema> = None;
        let mut last_reason: Option<String> = None;

        for attempt in &plan.attempts {
            // Wall cap: stop trying further nodes for this type. BOOKKEEPING
            // reason — fills silence, never clobbers a real server diagnosis.
            if wire::call_phase_budget_exceeded(Instant::now(), wall_deadline) {
                wire::retain_bookkeeping_reason(
                    &mut last_reason,
                    format!(
                        "the wire rung's call-phase budget ({WIRE_CALL_PHASE_BUDGET:?}) elapsed \
                         mid-plan before this attempt"
                    ),
                );
                break;
            }
            // Skip a node that already failed wait_for_service this attach —
            // bookkeeping, same retention rule.
            if let Some(reason) = wire::wait_cache_skip_reason(wait_failed, &attempt.node_fqn) {
                wire::retain_bookkeeping_reason(&mut last_reason, reason);
                continue;
            }

            let key = (attempt.node_fqn.clone(), attempt.type_hash.clone());
            let sources = if let Some(cached) = response_cache.get(&key) {
                match cached {
                    Some(s) => Some(s.clone()),
                    None => {
                        // Cache bookkeeping about an EARLIER type's call — a
                        // richer reason from THIS plan's own attempts wins
                        // (the bookkeeping retention rule, applied one arm wider).
                        wire::retain_bookkeeping_reason(
                            &mut last_reason,
                            format!(
                                "{} previously returned no usable response this attach",
                                attempt.node_fqn
                            ),
                        );
                        None
                    }
                }
            } else {
                let outcome = self
                    .call_one(
                        node,
                        attempt,
                        &plan.requested,
                        wall_deadline,
                        first_wait_done,
                    )
                    .await;
                let sources = wire::apply_attempt_outcome(
                    outcome,
                    &attempt.node_fqn,
                    wait_failed,
                    &mut last_reason,
                );
                response_cache.insert(key, sources.clone());
                sources
            };

            let Some(sources) = sources else { continue };
            match wire::sources_to_closure(&plan.requested, &sources) {
                Ok(closure) => {
                    acquired = Some(AcquiredSchema {
                        rung: AcquisitionRung::WireService,
                        closure,
                    });
                    break; // retry no further nodes — the type is resolved
                }
                Err(e) => last_reason = Some(e),
            }
        }

        match acquired {
            Some(schema) => {
                tracing::info!(
                    ros_type = %plan.requested,
                    members = schema.closure.len(),
                    "wire acquirer: acquired schema via get_type_description"
                );
                TypeAcquisition {
                    requested: plan.requested.clone(),
                    outcome: AcquisitionOutcome::Acquired(schema),
                }
            }
            None => {
                let reason = last_reason.unwrap_or_else(|| {
                    "no get_type_description call succeeded for this type".to_string()
                });
                tracing::warn!(
                    ros_type = %plan.requested,
                    reason = %reason,
                    "wire acquirer: every get_type_description attempt failed — kept UNRESOLVABLE"
                );
                TypeAcquisition {
                    requested: plan.requested.clone(),
                    outcome: AcquisitionOutcome::Skipped(vec![wire::wire_skip(reason)]),
                }
            }
        }
    }

    /// Call `<node_fqn>/get_type_description` for one attempt, trying its
    /// correlation mappings in order. Returns [`wire::AttemptResult::Resolved`]
    /// on the first `successful` reply; [`wire::AttemptResult::WaitTimedOut`]
    /// when NO mapping's `wait_for_service` connected (so the node is cached);
    /// or [`wire::AttemptResult::CallFailed`] when a server WAS reached but the
    /// call failed (client-create error, transport error, `successful == false`,
    /// or timeout) — a transient failure that must NOT poison the node cache.
    /// The call-phase `wall_deadline` is checked before EVERY
    /// mapping try (an expired wall skips the remaining mappings with the
    /// wall-cap reason) and each wait's cold/warm budget is CLAMPED to the
    /// remaining wall time — a cap checked only between attempts would let
    /// one dead 2-mapping participant overrun it by a whole attempt.
    async fn call_one(
        &self,
        node: &mut Node,
        attempt: &wire::CallAttempt,
        requested: &str,
        wall_deadline: Instant,
        first_wait_done: &mut bool,
    ) -> wire::AttemptResult {
        let service_name = match Name::new(&attempt.node_fqn, "get_type_description") {
            Ok(n) => n,
            Err(e) => {
                return wire::AttemptResult::CallFailed(format!(
                    "invalid service name for node {}: {e:?}",
                    attempt.node_fqn
                ));
            }
        };
        let service_type =
            ServiceTypeName::new("type_description_interfaces", "GetTypeDescription");

        let mut last_err: Option<String> = None;
        // Distinguish "a server answered but the call failed" from "no server
        // ever appeared" — only the latter caches the node as wait-failed.
        let mut any_wait_ok = false;
        let mut any_wait_timeout = false;
        for mapping in &attempt.mapping_order {
            // Per-MAPPING wall check — a second mapping never starts
            // on an exhausted wall.
            if wire::call_phase_budget_exceeded(Instant::now(), wall_deadline) {
                last_err = Some(mapping_wall_cap_reason(&attempt.node_fqn));
                break;
            }
            let client = match node.create_client::<GetTypeDescriptionService>(
                to_service_mapping(*mapping),
                &service_name,
                &service_type,
                self.service_qos(),
                self.service_qos(),
            ) {
                Ok(c) => c,
                Err(e) => {
                    last_err = Some(format!("could not create service client: {e:?}"));
                    continue;
                }
            };

            // Bounded wait for a server to appear on both service topics —
            // a POLL LOOP over FRESH `wait_for_service` futures:
            // each fresh wait re-runs ros2-client's reliable
            // already-present MAP check, so a discovery event dropped by the
            // bounded(8) status channel (see `WAIT_POLL_SLICE`) costs at most
            // one slice instead of hanging the whole budget. The
            // budget is cold for the attach's first wait, warm after, and
            // always clamped to the remaining wall time.
            let wait_budget = clamped_wait_budget(*first_wait_done, Instant::now(), wall_deadline);
            let wait_ok = poll_wait(wait_budget, WAIT_POLL_SLICE, || {
                client.wait_for_service(&*node)
            })
            .await;
            *first_wait_done = true;
            if !wait_ok {
                any_wait_timeout = true;
                last_err = Some(format!(
                    "no get_type_description server appeared at {} within {:?} (mapping {:?})",
                    attempt.node_fqn, wait_budget, mapping
                ));
                continue;
            }
            any_wait_ok = true;

            let request = GetTypeDescriptionRequest {
                type_name: cosmetic_type_name(requested),
                type_hash: attempt.type_hash.clone(),
                include_type_sources: true,
            };
            match with_timeout(client.async_call_service(request), self.call_budget).await {
                Some(Ok(resp)) => {
                    // The inverted-successful guard (pure).
                    match wire::classify_call_response(
                        resp.successful,
                        &resp.failure_reason,
                        &attempt.node_fqn,
                    ) {
                        Ok(()) => {
                            return wire::AttemptResult::Resolved(type_sources_to_wire(
                                resp.type_sources,
                            ))
                        }
                        Err(reason) => {
                            last_err = Some(reason);
                            continue;
                        }
                    }
                }
                Some(Err(e)) => {
                    last_err = Some(format!(
                        "get_type_description call to {} failed: {e:?}",
                        attempt.node_fqn
                    ));
                    continue;
                }
                None => {
                    last_err = Some(format!(
                        "get_type_description call to {} timed out after {:?}",
                        attempt.node_fqn, self.call_budget
                    ));
                    continue;
                }
            }
        }
        let reason = last_err.unwrap_or_else(|| "no correlation mapping was attempted".to_string());
        // Cache the node ONLY when a wait timed out and NONE succeeded (a pure
        // client-create failure is transient, not a missing server). The
        // classification is pure + truth-table-oracle-tested in `wire`;
        // this shell just gathers the flags.
        wire::classify_attempt_failure(any_wait_ok, any_wait_timeout, reason)
    }
}

/// The wait budget for ONE mapping attempt — the cold/warm
/// base ([`WAIT_BUDGET_COLD`] until the attach's first wait has run,
/// [`WAIT_BUDGET_WARM`] after) CLAMPED to the remaining call-phase wall time,
/// so a wait started near the wall never overruns it (`saturating` — an
/// already-expired wall yields ZERO, though the per-mapping wall check skips
/// that mapping before any wait starts). Pure — oracle-tested.
fn clamped_wait_budget(first_wait_done: bool, now: Instant, wall_deadline: Instant) -> Duration {
    let base = if first_wait_done {
        WAIT_BUDGET_WARM
    } else {
        WAIT_BUDGET_COLD
    };
    base.min(wall_deadline.saturating_duration_since(now))
}

/// The reason retained when the call-phase wall cap expires
/// BETWEEN correlation-mapping tries of one attempt (a cap checked only
/// between attempts lets a dead 2-mapping node burn both
/// waits). Classified `CallFailed` via the (false,false) truth-table cell —
/// an un-tried node is never cached as serverless. Pure — oracle-tested.
fn mapping_wall_cap_reason(node_fqn: &str) -> String {
    format!(
        "the wire rung's call-phase budget ({WIRE_CALL_PHASE_BUDGET:?}) elapsed before a \
         correlation-mapping attempt at {node_fqn} — remaining mappings skipped"
    )
}

/// The distinct per-type skip emitted for a plan the overall call-phase wall cap
/// cut off before it could be attempted.
fn wall_cap_skip_outcome(requested: String) -> TypeAcquisition {
    let reason = format!(
        "the wire rung's overall call-phase budget ({WIRE_CALL_PHASE_BUDGET:?}) elapsed before \
         this type was reached — too many nodes to query within the cap; re-run attach or narrow \
         the topic set"
    );
    tracing::warn!(
        ros_type = %requested,
        "wire acquirer: call-phase wall cap elapsed before this type — kept UNRESOLVABLE"
    );
    TypeAcquisition {
        requested,
        outcome: AcquisitionOutcome::Skipped(vec![wire::wire_skip(reason)]),
    }
}

impl SchemaAcquirer for WireServiceAcquirer {
    fn acquire(&self, types: &[String], discovery: &DiscoveryResult) -> Vec<TypeAcquisition> {
        if types.is_empty() {
            return Vec::new();
        }
        match self.run_calls(types, discovery) {
            Ok(v) => v,
            Err(e) => {
                // A whole-rung DDS setup failure — skip EVERY requested type
                // loudly (never a silent omission), so the ladder falls through
                // to the local rung with the reason recorded.
                tracing::warn!(
                    error = %e,
                    "wire acquirer: DDS setup failed — every requested type falls through this rung"
                );
                types
                    .iter()
                    .map(|t| TypeAcquisition {
                        requested: t.clone(),
                        outcome: AcquisitionOutcome::Skipped(vec![wire::wire_skip(format!(
                            "wire rung unavailable ({e})"
                        ))]),
                    })
                    .collect()
            }
        }
    }
}

/// Map the DDS-free [`wire::WireMapping`] to ros2-client's `ServiceMapping`.
fn to_service_mapping(m: wire::WireMapping) -> ServiceMapping {
    match m {
        wire::WireMapping::Enhanced => ServiceMapping::Enhanced,
        wire::WireMapping::Cyclone => ServiceMapping::Cyclone,
    }
}

/// Reformat a canonical `pkg/Type` to the `.srv` request's
/// `PACKAGE/NAMESPACE/TYPENAME` shape (`pkg/msg/Type`) — cosmetic (the server
/// ignores it), sent for forward-compat.
fn cosmetic_type_name(requested: &str) -> String {
    match requested.split_once('/') {
        Some((pkg, ty)) => format!("{pkg}/msg/{ty}"),
        None => requested.to_string(),
    }
}

/// Convert the response's CDR `TypeSource[]` into the DDS-free
/// [`wire::WireTypeSource`] the pure closure mapper consumes.
fn type_sources_to_wire(sources: Vec<TypeSource>) -> Vec<wire::WireTypeSource> {
    sources
        .into_iter()
        .map(|ts| wire::WireTypeSource {
            type_name: ts.type_name,
            encoding: ts.encoding,
            raw_file_contents: ts.raw_file_contents,
        })
        .collect()
}

/// Race `fut` against a timer; `Some(output)` if it completed, `None` on
/// timeout. Bounds every blocking service step so a dead/absent server can never
/// wedge the rung.
async fn with_timeout<F>(fut: F, dur: Duration) -> Option<F::Output>
where
    F: std::future::Future,
{
    // Pin both arms so `select` (which requires `Unpin`) accepts them
    // regardless of the inner futures' `Unpin`-ness (`smol::Timer` is pinned
    // the same way in `discovery::collect_endpoints`).
    let fut = std::pin::pin!(fut);
    let timer = std::pin::pin!(smol::Timer::after(dur));
    match futures::future::select(fut, timer).await {
        futures::future::Either::Left((out, _)) => Some(out),
        futures::future::Either::Right(_) => None,
    }
}

/// Bounded POLL LOOP over FRESH waiter futures (the
/// `wait_for_service` lost-event-hang fix; rationale on [`WAIT_POLL_SLICE`]):
/// each iteration awaits a NEWLY-MADE `mk_wait()` future under
/// [`with_timeout`] for up to `slice` — clamped to the remaining budget so the
/// final slice never overruns — and returns `true` the moment any slice's
/// future completes, `false` once the cumulative elapsed reaches `budget`.
/// Re-making the future each slice is the point: a fresh
/// `Client::wait_for_service` re-runs ros2-client's reliable already-present
/// map check, recovering from a discovery event the single-shot wait lost.
/// `slice` must be positive (the [`WAIT_POLL_SLICE`] const is; a zero slice
/// would busy-poll). Hermetically oracle-tested below — no DDS needed.
async fn poll_wait<F, Fut>(budget: Duration, slice: Duration, mut mk_wait: F) -> bool
where
    F: FnMut() -> Fut,
    Fut: std::future::Future,
{
    let started = Instant::now();
    loop {
        let elapsed = started.elapsed();
        let Some(remaining) = budget.checked_sub(elapsed) else {
            return false;
        };
        if remaining.is_zero() {
            return false;
        }
        if with_timeout(mk_wait(), slice.min(remaining))
            .await
            .is_some()
        {
            return true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The DDS-free glue this file owns (the pure decision engine is tested in
    /// `wire`): mapping conversion, the cosmetic type-name reformat, and the
    /// `TypeSource` → `WireTypeSource` conversion feeding the closure mapper.
    #[test]
    fn test_to_service_mapping() {
        assert_eq!(
            to_service_mapping(wire::WireMapping::Enhanced),
            ServiceMapping::Enhanced
        );
        assert_eq!(
            to_service_mapping(wire::WireMapping::Cyclone),
            ServiceMapping::Cyclone
        );
    }

    #[test]
    fn test_cosmetic_type_name() {
        assert_eq!(
            cosmetic_type_name("sensor_msgs/PointCloud2"),
            "sensor_msgs/msg/PointCloud2"
        );
        // A bare name is passed through unchanged (still cosmetic).
        assert_eq!(cosmetic_type_name("Weird"), "Weird");
    }

    #[test]
    fn test_type_sources_to_wire_then_closure() {
        // The full response-mapping glue: CDR TypeSource → WireTypeSource →
        // sources_to_closure. Byte-verbatim text (comment kept) + encoding
        // filter, against a hand oracle (never a self-compare).
        const WIDGET: &str = "# a widget\nint32 id\n";
        let sources = vec![
            TypeSource {
                type_name: "acme_msgs/msg/Widget".to_string(),
                encoding: "msg".to_string(),
                raw_file_contents: WIDGET.to_string(),
            },
            TypeSource {
                type_name: "acme_msgs/msg/Widget".to_string(),
                encoding: "idl".to_string(), // skipped by the closure mapper
                raw_file_contents: "module ...".to_string(),
            },
        ];
        let wire_sources = type_sources_to_wire(sources);
        let closure = wire::sources_to_closure("acme_msgs/Widget", &wire_sources).expect("closure");
        assert_eq!(closure.len(), 1);
        assert_eq!(closure[0].qualified_name(), "acme_msgs/Widget");
        assert_eq!(closure[0].msg_text, WIDGET);
    }

    /// The request always asks for sources and carries the hash; the type name
    /// is cosmetic (server ignores it).
    #[test]
    fn test_request_fields() {
        let req = GetTypeDescriptionRequest {
            type_name: cosmetic_type_name("acme_msgs/Widget"),
            type_hash: "RIHS01_ab".to_string(),
            include_type_sources: true,
        };
        assert!(req.include_type_sources);
        assert_eq!(req.type_hash, "RIHS01_ab");
        assert_eq!(req.type_name, "acme_msgs/msg/Widget");
    }

    // ───────────── CDR field-order structural pins ──────────────────────────
    //
    // These drive the EXACT ros2-client/rustdds CDR serializer/deserializer the
    // live call uses (`ros2_client::rustdds::serialization::{to_writer_with_rep_id,
    // deserialize_from_cdr_with_rep_id}` @ CDR_LE — the same `cdr_encoding`
    // `to_writer`/`from_bytes` the DataWriter's `CDRSerializerAdapter<_,
    // LittleEndian>` calls) against HAND-BUILT byte vectors — so a swapped struct
    // field is caught structurally, not only in a live decode.

    use ros2_client::rustdds::serialization::{
        deserialize_from_cdr_with_rep_id, to_writer_with_rep_id, RepresentationIdentifier,
    };

    /// The REQUEST CDR body serialized through the production serializer equals
    /// a hand-built little-endian CDR byte vector. CDR-LE layout math (body
    /// starts at offset 0; strings = `u32 len incl NUL` (aligned 4) + bytes +
    /// NUL; bool = 1 byte, no alignment):
    ///   type_name = "abc":  [04 00 00 00] len=4  | 61 62 63  | 00 NUL       -> off 8
    ///   type_hash = "hh":   [03 00 00 00] len=3  | 68 68     | 00 NUL       -> off 15
    ///   include_type_sources = true: [01]                                   -> off 16
    #[test]
    fn request_cdr_matches_hand_oracle() {
        let req = GetTypeDescriptionRequest {
            type_name: "abc".to_string(),
            type_hash: "hh".to_string(),
            include_type_sources: true,
        };
        let mut buf: Vec<u8> = Vec::new();
        to_writer_with_rep_id(&mut buf, &req, RepresentationIdentifier::CDR_LE)
            .expect("serialize request");

        // type_name "abc": len=4 (LE) | 'a''b''c' | NUL  (offset 0..8, 4-aligned)
        // type_hash "hh":  len=3 (LE) | 'h''h'     | NUL  (offset 8..15)
        // include_type_sources = true: 0x01                (offset 15..16)
        let expected: Vec<u8> = vec![
            0x04, 0x00, 0x00, 0x00, // type_name length = 3 + NUL = 4
            0x61, 0x62, 0x63, // "abc"
            0x00, // NUL terminator (offset now 8)
            0x03, 0x00, 0x00, 0x00, // type_hash length = 2 + NUL = 3
            0x68, 0x68, // "hh"
            0x00, // NUL terminator (offset now 15)
            0x01, // include_type_sources = true
        ];
        assert_eq!(buf, expected, "request CDR field order / layout drifted");
    }

    /// A minimal, DOCUMENTED CDR-LE writer used ONLY to hand-build the RESPONSE
    /// oracle bytes — an INDEPENDENT reimplementation of the CDR primitive
    /// layout, never the production serializer (so decoding it is a real
    /// cross-check, not a serialize-then-deserialize self-compare).
    struct CdrBuf(Vec<u8>);
    impl CdrBuf {
        fn new() -> Self {
            Self(Vec::new())
        }
        /// Pad to the next `a`-byte boundary measured from the body start.
        fn align(&mut self, a: usize) {
            while !self.0.len().is_multiple_of(a) {
                self.0.push(0);
            }
        }
        fn bool(&mut self, v: bool) {
            self.0.push(u8::from(v));
        }
        /// Single byte — no alignment (`FieldType.type_id`).
        fn u8(&mut self, v: u8) {
            self.0.push(v);
        }
        fn u32(&mut self, v: u32) {
            self.align(4);
            self.0.extend_from_slice(&v.to_le_bytes());
        }
        /// 8-byte primitive — CDR aligns it to 8 from the body start
        /// (`FieldType.capacity`/`string_capacity`, the response's ONLY
        /// 8-aligned members — the padding this writer inserts is exactly what
        /// the Field/FieldType depth pin exercises).
        fn u64(&mut self, v: u64) {
            self.align(8);
            self.0.extend_from_slice(&v.to_le_bytes());
        }
        /// CDR string: `u32 len` (INCLUDING the NUL) + bytes + NUL.
        fn string(&mut self, s: &str) {
            self.u32(s.len() as u32 + 1);
            self.0.extend_from_slice(s.as_bytes());
            self.0.push(0);
        }
        /// CDR sequence header: a `u32` element count (aligned 4).
        fn seq_len(&mut self, n: u32) {
            self.u32(n);
        }
        fn into_bytes(self) -> Vec<u8> {
            self.0
        }
    }

    /// A hand-built RESPONSE CDR body decodes through the production 5-level
    /// positional deserializer to the expected fields.
    /// The oracle carries a POPULATED payload at EVERY level —
    /// a real Jazzy server always returns a non-empty
    /// `type_description.type_description.fields[]` (any real type has fields)
    /// whose bytes sit BEFORE `type_sources` in the stream, so the innermost
    /// levels an all-empty oracle never drives (`Field`,
    /// `FieldType` with its 8-byte-aligned `u64`s, a referenced
    /// `IndividualTypeDescription`, `KeyValue`) are byte-pinned: a swapped
    /// or retyped field there misaligns THIS decode. Field order: successful,
    /// failure_reason, type_description{ type_description{ type_name,
    /// fields[]{ name, field_type{ type_id, capacity, string_capacity,
    /// nested_type_name }, default_value } }, referenced_type_descriptions[] },
    /// type_sources[], extra_information[]. Full consumption asserted — a
    /// trailing field deleted from the structs would leave unconsumed bytes.
    #[test]
    fn response_cdr_decodes_hand_oracle_successful() {
        let mut b = CdrBuf::new();
        b.bool(true); // successful
        b.string(""); // failure_reason
                      // type_description.type_description (IndividualTypeDescription):
        b.string("acme_msgs/msg/Widget"); //   type_name
        b.seq_len(1); //   fields[] len 1
        b.string("id"); //     [0].name
        b.u8(5); //     [0].field_type.type_id (1 byte, unaligned)
        b.u64(7); //     [0].field_type.capacity (pads to the next 8-boundary)
        b.u64(9); //     [0].field_type.string_capacity
        b.string("acme_msgs/msg/Nested"); // [0].field_type.nested_type_name
        b.string("42"); //     [0].default_value
        b.seq_len(1); // type_description.referenced_type_descriptions[] len 1
        b.string("acme/msg/Vec2"); //   [0].type_name
        b.seq_len(0); //   [0].fields[] empty
        b.seq_len(1); // type_sources[] len 1
        b.string("acme_msgs/msg/Widget"); //   [0].type_name
        b.string("msg"); //   [0].encoding
        b.string("int32 id\n"); //   [0].raw_file_contents
        b.seq_len(1); // extra_information[] len 1
        b.string("k"); //   [0].key
        b.string("v"); //   [0].value
        let bytes = b.into_bytes();

        let (resp, consumed) = deserialize_from_cdr_with_rep_id::<GetTypeDescriptionResponse>(
            &bytes,
            RepresentationIdentifier::CDR_LE,
        )
        .expect("decode response");

        assert!(resp.successful);
        assert_eq!(resp.failure_reason, "");
        // The innermost structured levels, against the hand-written values.
        let td = &resp.type_description.type_description;
        assert_eq!(td.type_name, "acme_msgs/msg/Widget");
        assert_eq!(td.fields.len(), 1);
        assert_eq!(td.fields[0].name, "id");
        assert_eq!(td.fields[0].field_type.type_id, 5);
        assert_eq!(td.fields[0].field_type.capacity, 7);
        assert_eq!(td.fields[0].field_type.string_capacity, 9);
        assert_eq!(
            td.fields[0].field_type.nested_type_name,
            "acme_msgs/msg/Nested"
        );
        assert_eq!(td.fields[0].default_value, "42");
        let refs = &resp.type_description.referenced_type_descriptions;
        assert_eq!(refs.len(), 1);
        assert_eq!(refs[0].type_name, "acme/msg/Vec2");
        assert!(refs[0].fields.is_empty());
        // The consumed half (what production reads).
        assert_eq!(resp.type_sources.len(), 1);
        assert_eq!(resp.type_sources[0].type_name, "acme_msgs/msg/Widget");
        assert_eq!(resp.type_sources[0].encoding, "msg");
        assert_eq!(resp.type_sources[0].raw_file_contents, "int32 id\n");
        assert_eq!(resp.extra_information.len(), 1);
        assert_eq!(resp.extra_information[0].key, "k");
        assert_eq!(resp.extra_information[0].value, "v");
        // Every oracle byte consumed — a struct that silently
        // stopped short (e.g. a deleted trailing field) fails here.
        assert_eq!(consumed, bytes.len(), "decode must consume the whole body");
    }

    /// The failure shape: successful=false carries the server's failure_reason
    /// and no sources (the exact shape the inverted-successful guard keys off).
    #[test]
    fn response_cdr_decodes_hand_oracle_failure() {
        let mut b = CdrBuf::new();
        b.bool(false); // successful
        b.string("Type not currently in use by this node"); // failure_reason
        b.string(""); // type_description.type_description.type_name
        b.seq_len(0); //   fields[]
        b.seq_len(0); // referenced_type_descriptions[]
        b.seq_len(0); // type_sources[] empty
        b.seq_len(0); // extra_information[]
        let bytes = b.into_bytes();

        let (resp, consumed) = deserialize_from_cdr_with_rep_id::<GetTypeDescriptionResponse>(
            &bytes,
            RepresentationIdentifier::CDR_LE,
        )
        .expect("decode response");

        assert!(!resp.successful);
        assert_eq!(
            resp.failure_reason,
            "Type not currently in use by this node"
        );
        assert!(resp.type_sources.is_empty());
        assert_eq!(consumed, bytes.len(), "decode must consume the whole body");
    }

    // ───────────────── per-call budget pass-through ─────────────────────────

    /// `with_timeout` returns `None` when the inner future never completes
    /// within the budget (a dead/absent server) — a tiny budget vs a
    /// never-ready `pending()` future, so the timer arm fires.
    #[test]
    fn with_timeout_times_out_on_a_never_ready_future() {
        let got = smol::block_on(with_timeout(
            std::future::pending::<()>(),
            Duration::from_millis(20),
        ));
        assert!(got.is_none(), "a never-ready future must time out to None");
    }

    /// `with_timeout` passes the value through when the inner future is ready
    /// well within the budget.
    #[test]
    fn with_timeout_passes_through_a_ready_future() {
        let got = smol::block_on(with_timeout(
            std::future::ready(42u32),
            Duration::from_secs(5),
        ));
        assert_eq!(got, Some(42), "a ready future must pass through");
    }

    // ───────── wait_for_service poll loop ─────────────────────────

    /// A never-ready waiter still times out at the CUMULATIVE `budget` (the
    /// dead/absent-server arm keeps its old bound — the poll loop changes HOW
    /// the budget is spent, never how much). Lower bound asserted (the loop
    /// must consume the whole budget before giving up); upper bound left to a
    /// generous VM-stall margin.
    #[test]
    fn poll_wait_never_ready_times_out_at_budget_total() {
        let started = std::time::Instant::now();
        let ok = smol::block_on(poll_wait(
            Duration::from_millis(80),
            Duration::from_millis(20),
            std::future::pending::<()>,
        ));
        let elapsed = started.elapsed();
        assert!(!ok, "a never-ready waiter must time out");
        assert!(
            elapsed >= Duration::from_millis(80),
            "the loop must poll until the FULL budget elapses (got {elapsed:?})"
        );
        assert!(
            elapsed < Duration::from_secs(5),
            "generous VM-stall ceiling (got {elapsed:?})"
        );
    }

    /// THE poll-loop fix pin: a waiter that becomes ready only on the Nth FRESH
    /// future (readiness keyed on the CALL COUNT, not time — hermetic, no DDS)
    /// succeeds, proving the loop re-MAKES the future each slice (the fresh
    /// `wait_for_service` re-running the reliable map check) instead of
    /// hanging on the first lost-event wait. Exactly 3 makes: calls 1 and 2
    /// pend out their slices, call 3 completes instantly. A single-shot wait
    /// would hang to the budget and return false.
    #[test]
    fn poll_wait_succeeds_on_the_nth_fresh_future() {
        let calls = std::cell::Cell::new(0u32);
        let ok = smol::block_on(poll_wait(
            Duration::from_secs(10), // stall-proof headroom; success ends the loop
            Duration::from_millis(5),
            || {
                let n = calls.get() + 1;
                calls.set(n);
                async move {
                    if n < 3 {
                        std::future::pending::<()>().await;
                    }
                }
            },
        ));
        assert!(ok, "the 3rd fresh future completes → wait_ok");
        assert_eq!(
            calls.get(),
            3,
            "exactly three fresh futures made (two slices pended, the third hit)"
        );
    }

    /// A first-poll-ready waiter passes through immediately with ONE make —
    /// the healthy-server fast path is unchanged by the loop.
    #[test]
    fn poll_wait_ready_future_passes_on_the_first_slice() {
        let calls = std::cell::Cell::new(0u32);
        let ok = smol::block_on(poll_wait(
            Duration::from_secs(10),
            Duration::from_millis(250),
            || {
                calls.set(calls.get() + 1);
                std::future::ready(())
            },
        ));
        assert!(ok);
        assert_eq!(calls.get(), 1, "a ready waiter needs exactly one make");
    }

    /// The FINAL slice is clamped to the remaining budget: with a slice far
    /// larger than the budget, a never-ready waiter still returns at ~budget,
    /// not at ~slice (pre-clamp this would burn the whole 5 s slice).
    #[test]
    fn poll_wait_budget_clamps_the_final_slice() {
        let started = std::time::Instant::now();
        let ok = smol::block_on(poll_wait(
            Duration::from_millis(100),
            Duration::from_secs(5),
            std::future::pending::<()>,
        ));
        let elapsed = started.elapsed();
        assert!(!ok);
        assert!(
            elapsed >= Duration::from_millis(100),
            "full budget consumed"
        );
        assert!(
            elapsed < Duration::from_millis(2500),
            "the slice must be clamped to the budget — ~100 ms, never the 5 s \
             slice (got {elapsed:?}; generous VM margin)"
        );
    }

    // ───────── cold/warm wait split + wall clamp ──────────────────

    /// The per-mapping wait-budget selection oracle: COLD (4 s) only until the
    /// attach's first wait has run, WARM (2 s) after; BOTH clamped to the
    /// remaining wall time; an expired wall yields ZERO (the per-mapping wall
    /// check skips the mapping first, so the zero is belt-and-suspenders).
    #[test]
    fn clamped_wait_budget_cold_warm_and_clamp_oracle() {
        let now = Instant::now();
        let far = now + Duration::from_secs(60);
        // Cold vs warm selection under a distant wall.
        assert_eq!(clamped_wait_budget(false, now, far), WAIT_BUDGET_COLD);
        assert_eq!(clamped_wait_budget(true, now, far), WAIT_BUDGET_WARM);
        // Near-exhausted wall clamps BOTH bases to the remaining time.
        let near = now + Duration::from_millis(500);
        assert_eq!(
            clamped_wait_budget(true, now, near),
            Duration::from_millis(500)
        );
        assert_eq!(
            clamped_wait_budget(false, now, near),
            Duration::from_millis(500)
        );
        // Expired wall → zero (saturating, never a panic or huge wrap).
        assert_eq!(clamped_wait_budget(true, near, now), Duration::ZERO);
        assert_eq!(clamped_wait_budget(false, near, now), Duration::ZERO);
    }

    /// The budget arithmetic, pinned as a CONST relationship: the worst
    /// single dead participant (one COLD + three WARM waits — 2 silent
    /// unknown-vendor nodes × 2 mappings) must leave headroom under the wall
    /// cap for a further node to be tried (one unsplit 4 s budget: 4 s × 4 = 16 s > 15 s,
    /// so a reachable third node is NEVER tried).
    #[test]
    fn wait_budget_split_leaves_wall_headroom() {
        let worst_dead_participant = WAIT_BUDGET_COLD + WAIT_BUDGET_WARM * 3;
        assert!(
            worst_dead_participant < WIRE_CALL_PHASE_BUDGET,
            "one dead 2-node/2-mapping participant ({worst_dead_participant:?}) must not \
             exhaust the {WIRE_CALL_PHASE_BUDGET:?} wall cap"
        );
    }

    /// The expired-wall per-MAPPING skip carries the wall-cap reason (naming
    /// the budget and the node); classified CallFailed via the (false,false)
    /// truth-table cell already pinned in `wire` — never cached as serverless.
    #[test]
    fn mapping_wall_cap_reason_names_budget_and_node() {
        let reason = mapping_wall_cap_reason("/talker");
        assert!(reason.contains("call-phase budget"), "{reason}");
        assert!(reason.contains("/talker"), "{reason}");
        assert!(reason.contains("remaining mappings skipped"), "{reason}");
    }

    /// Composition pin: a wait started near wall exhaustion is bounded by the
    /// REMAINING wall time, not the 2 s WARM base — the clamped budget feeds
    /// `poll_wait`, which times out at the (clamped) cumulative budget.
    #[test]
    fn deadline_clamped_wait_is_bounded_by_remaining_wall() {
        let started = Instant::now();
        let wall_deadline = started + Duration::from_millis(60);
        let budget = clamped_wait_budget(true, Instant::now(), wall_deadline);
        assert!(
            budget <= Duration::from_millis(60),
            "the wait budget must be the remaining wall, not the WARM base (got {budget:?})"
        );
        let ok = smol::block_on(poll_wait(
            budget,
            WAIT_POLL_SLICE,
            std::future::pending::<()>,
        ));
        let elapsed = started.elapsed();
        assert!(!ok, "a never-ready waiter on a clamped budget times out");
        assert!(
            elapsed < Duration::from_millis(1500),
            "bounded by the ~60 ms remaining wall, never the 2 s WARM base \
             (got {elapsed:?}; generous VM margin)"
        );
    }
}
