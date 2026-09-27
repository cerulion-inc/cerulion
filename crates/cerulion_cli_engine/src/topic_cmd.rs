// SPDX-License-Identifier: AGPL-3.0-only
//! Topic introspection commands: list, info, echo, hz.
//!
//! Uses iceoryx2 service discovery to enumerate active shared memory topics.

use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use crate::discovery_ladder::{DiscoveredPeer, DiscoveryRung};
use crate::error::{CliError, CliResult};

// Enumerate services through the same Service type
// the core transport uses (`ipc_threadsafe::Service`). The Service type
// parameter only selects the in-process port-wrapping policy
// (`ArcThreadSafetyPolicy`); the on-SHM service identity and discovery are
// identical across `ipc::Service` and `ipc_threadsafe::Service`, so this is a
// consistency choice, not a cross-process compatibility requirement.
use iceoryx2::service::ipc_threadsafe::Service as CerService;

/// Discovered topic info.
pub struct TopicInfo {
    pub name: String,
}

/// A LOCAL topic that is really a live MIRROR of a remote
/// robot's topic (its canonical name appears in the `/__cerulion/mirrors`
/// provenance registry). Rendered in the REMOTE TOPICS section attributed to its
/// origin robot with a streaming marker, not under LOCAL — the "one data
/// source = one topic" rule. The `robot` string is externally-sourced (a remote
/// robot's identity, carried by a re-injector) so it is terminal-escape-sanitized
/// at the render seam.
///
/// The type + the fold predicate live in `cerulion_core` so
/// `cerulion topic list` and `cerulion-vizd` fold through ONE implementation; this
/// is a re-export, kept so the CLI's render seam reads unchanged.
pub use cerulion_core::transport::mirror_registry::MirrorStreamRow;

/// PURE partition of the LOCAL topic enumeration into
/// genuinely-local topics (kept under LOCAL) and mirror-streaming rows (folded to
/// REMOTE, attributed to their origin robot). A topic whose canonical name appears
/// in `mirrors` is a mirror of a remote robot and MUST NOT present as a second
/// local topic. With an EMPTY `mirrors` map this is the identity partition (every
/// topic stays local, zero streaming rows) — the registry-less / no-mirror desk
/// renders byte-identically to a desk with no mirror registry at all.
///
/// The fold itself lives in
/// [`cerulion_core::transport::mirror_registry::partition_local_topics`] (shared
/// with `cerulion-vizd`'s sidebar enumeration); this adapter only carries the
/// CLI's [`TopicInfo`] newtype across it, so `topic list`'s behaviour is
/// unchanged. Oracle-tested in BOTH places: the predicate in core, this adapter's
/// `TopicInfo` round-trip below.
pub fn partition_local_topics(
    local: Vec<TopicInfo>,
    mirrors: &BTreeMap<String, String>,
) -> (Vec<TopicInfo>, Vec<MirrorStreamRow>) {
    let (genuine_local, streaming) =
        cerulion_core::transport::mirror_registry::partition_local_topics(
            local.into_iter().map(|t| t.name),
            mirrors,
        );
    (
        genuine_local
            .into_iter()
            .map(|name| TopicInfo { name })
            .collect(),
        streaming,
    )
}

/// Gather the CURRENT live mirror-provenance snapshot from the
/// desk's `/__cerulion/mirrors` registry, as a `canonical topic → origin robot`
/// map. BEST-EFFORT: a desk with no live mirror returns empty instantly (the
/// no-publisher fast path in cerulion_core), and ANY error is treated as "no
/// provenance" (a `debug!` breadcrumb) so `topic list` never fails on the registry
/// — the local list already printed, and the fallback is that mirrored
/// topics would show as LOCAL (the no-provenance behavior) rather than crashing the
/// command.
pub fn gather_mirror_provenance() -> BTreeMap<String, String> {
    match cerulion_core::transport::mirror_registry::gather_current_provenance() {
        Ok(records) => records
            .into_iter()
            .map(|r| (r.topic, r.origin_robot))
            .collect(),
        Err(e) => {
            tracing::debug!(
                error = %e,
                "topic list: mirror-provenance gather failed — treating as no mirrors \
                 (any mirrored topics show as LOCAL this run)"
            );
            BTreeMap::new()
        }
    }
}

/// List active topics by enumerating iceoryx2 services on the process-global
/// config.
///
/// Filters for services ending with `/data` (Cerulion creates `{topic}/data` +
/// `{topic}/event` per topic) and strips the suffix. The `/__cerulion/mirrors`
/// provenance registry carries NO `/data` suffix, so it is structurally invisible
/// here (pinned behaviorally in the tests).
pub fn topic_list() -> CliResult<Vec<TopicInfo>> {
    list_topic_infos(iceoryx2::config::Config::global_config())
}

/// The config-parameterized enumeration behind [`topic_list`] — the
/// production path passes the global config; a per-test root lets a parallel-safe
/// test enumerate its own namespace (e.g. to behaviorally pin that the mirror
/// registry service never surfaces).
pub fn list_topic_infos(config: &iceoryx2::config::Config) -> CliResult<Vec<TopicInfo>> {
    let mut topics = vec![];

    cerulion_core::iceoryx_logger::init_iceoryx_log_level_from_env();

    // An enumeration failure must not
    // return Ok(vec![]) — an empty list is indistinguishable from "no
    // topics running" and hides SHM-permission / config problems.
    <CerService as iceoryx2::service::Service>::list(
        config,
        |service: iceoryx2::service::ServiceDetails<CerService>| {
            let name = service.static_details.name().as_str().to_string();
            if let Some(topic_name) = data_topic_of_service(&name) {
                topics.push(TopicInfo {
                    name: topic_name.to_string(),
                });
            }
            iceoryx2::prelude::CallbackProgression::Continue
        },
    )
    .map_err(|e| {
        CliError::Validation(format!(
            "failed to enumerate iceoryx2 services: {e} — check the iceoryx2 \
             config and shared-memory permissions"
        ))
    })?;

    topics.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(topics)
}

/// Map an iceoryx2 data-service name back to its topic —
/// strip the `/data` suffix exactly ONCE (`trim_end_matches` strips
/// REPEATED suffixes, so a topic ending in `/data` — every graph output
/// named `data` — listed wrongly). A foreign service named exactly
/// `/data` would yield an empty topic name; skip it rather than emit a
/// phantom blank row.
fn data_topic_of_service(service_name: &str) -> Option<&str> {
    service_name.strip_suffix("/data").filter(|t| !t.is_empty())
}

/// Topic-name prefixes of the framework's OWN channels: the ones `topic list`
/// HIDES by default and `bag record --all` / `--regex` never auto-select.
///
/// - `/__cerulion/` (and the slashless `__cerulion/` spelling the bag format
///   reserves) is the framework's control namespace: the mirror-provenance
///   registry, the run registry, the ingress-build channel, the flashback
///   request/outcome channels. Most of those are event-only services with no
///   `/data` counterpart, so the enumerator never yields them; the prefix is
///   still the rule, so a future data-carrying control channel is hidden by
///   construction rather than by luck.
/// - `/bagd/` is the recorder's own `/bagd/status` channel, live for the length
///   of every `graph run --record`. It is the topic that sat beside the two user
///   topics in the quickstart's `topic list` output.
///
/// ONE prefix list: `bag_cmd::AUTO_SELECT_EXCLUDED_PREFIXES` is an alias of this
/// slice, and `cerulion_bagd`'s hand-copied `EXCLUDED_TOPIC_PREFIXES` is
/// equality-pinned against it in `bag_cmd`'s tests. Matched with `starts_with`,
/// in order.
///
/// The list is NOT the whole rule. The predicate is [`is_internal_topic`]: this
/// list PLUS the bare `/__cerulion` namespace token, which no prefix can spell.
/// The listing's default view and `bag record --all` / `--regex` both CALL that
/// one function, so they cannot disagree about what "the framework's own"
/// means. bagd's live-discovery planner walks its prefix copy alone; the bare
/// token is the one name where it would differ (see [`is_internal_topic`]).
///
/// Hidden is not gone: the default listing prints a count line naming `--all`
/// whenever it hid something, `--all` lists every internal topic with an
/// `internal` marker in the row, and `topic echo` / `info` / `hz` read an
/// internal topic by name exactly like any other.
pub const INTERNAL_TOPIC_PREFIXES: &[&str] = &["__cerulion/", "/__cerulion/", "/bagd/"];

/// Whether `topic` is one of the framework's own channels (see
/// [`INTERNAL_TOPIC_PREFIXES`]). Pure; a prefix match, so `/bagd/status` and
/// `/__cerulion/anything` are internal while `/bagdx` and `/my/bagd/status`
/// are user topics. The bare namespace token `/__cerulion` is internal too,
/// matched EXACTLY against the gateway's `RESERVED_TOPIC_PREFIX` (its
/// `is_reserved_topic` rule: the token itself or any child; `/__cerulionx` is
/// a different topic), because no prefix in the list can spell "the token and
/// nothing else".
///
/// `bag record --all` / `--regex` filter with THIS function
/// (`bag_cmd::auto_selectable`), so a row the default listing hides is a topic
/// that verb never auto-selects. The one consumer that still walks the prefix
/// list alone is `cerulion_bagd`'s live-discovery planner (that crate cannot
/// depend on this one). No producer creates a `/__cerulion` data service today,
/// so that planner cannot yet observe the difference.
pub fn is_internal_topic(topic: &str) -> bool {
    topic == cerulion_core::transport::gateway::RESERVED_TOPIC_PREFIX
        || INTERNAL_TOPIC_PREFIXES
            .iter()
            .any(|prefix| topic.starts_with(prefix))
}

/// The row marker `topic list --all` prints after an internal topic's path.
/// The path stays the row's FIRST whitespace token (the Studio parser keys off
/// section headers and `/`-prefixed rows; markers ride the trailing columns,
/// like the `● streaming` marker of a mirrored row).
pub const INTERNAL_TOPIC_MARKER: &str = "internal";

/// Render the LOCAL section of `topic list`: the `TOPIC` header and one row per
/// genuine-local topic, in the order given (the enumeration sorts by name).
///
/// `show_all == false` (the default listing) SKIPS every internal topic
/// ([`is_internal_topic`]) and, when it skipped any, ends the section with ONE
/// count line, `N internal topic(s) hidden (--all shows it/them)`, so a hidden
/// topic is never silently gone: the operator is told what was hidden and how
/// to see it. With `show_all == true` every topic prints, internal ones with
/// the [`INTERNAL_TOPIC_MARKER`] in the row, and there is no count line
/// (nothing was hidden).
///
/// An empty visible set prints `No active local topics.` in place of the header
/// (the REMOTE section below may still list rows, so "no active topics" would
/// be misleading); the count line still follows it when the only local topics
/// were internal, which is the `graph run --record` desk with no user graph
/// left: `/bagd/status` alone.
///
/// Pure string rendering (the binary prints the returned string verbatim). Every
/// non-topic line stays free of a leading `/` so a row parser cannot mistake it
/// for a topic. Oracle-tested below.
pub fn render_local_topics_section(local: &[TopicInfo], show_all: bool) -> String {
    let mut rows = String::new();
    let mut hidden = 0usize;
    for topic in local {
        let internal = is_internal_topic(&topic.name);
        if internal && !show_all {
            hidden += 1;
            continue;
        }
        rows.push_str(&topic.name);
        if internal {
            rows.push_str("  ");
            rows.push_str(INTERNAL_TOPIC_MARKER);
        }
        rows.push('\n');
    }
    let mut out = if rows.is_empty() {
        String::from("No active local topics.\n")
    } else {
        format!("TOPIC\n{rows}")
    };
    if hidden > 0 {
        out.push_str(&render_hidden_internal_count(hidden));
    }
    out
}

/// The one-line footer of a default `topic list` that hid internal topics:
/// how many, and that `--all` shows them. Singular and plural agree in both
/// halves (`1 internal topic hidden (--all shows it)`,
/// `2 internal topics hidden (--all shows them)`).
fn render_hidden_internal_count(hidden: usize) -> String {
    let (noun, pronoun) = if hidden == 1 {
        ("topic", "it")
    } else {
        ("topics", "them")
    };
    format!("{hidden} internal {noun} hidden (--all shows {pronoun})\n")
}

/// Where the frames a `topic echo`/`info`/`hz`
/// observes come from — a LOCAL SHM topic, or a REMOTE robot's topic this process
/// just DEMANDED from the shared cerulion-netd daemon (carrying the seeded decode
/// walker so the observer never has to re-discover).
enum TopicSource {
    /// The topic is a live LOCAL iceoryx2 topic — read it directly.
    Local,
    /// The topic is remote; this process demanded its shared mirror from
    /// cerulion-netd (released on exit via the [`DemandGuard`]). `robot`/`walker`
    /// come from the resolve that found it — reused so the observer never re-runs
    /// discovery.
    Remote {
        robot: String,
        walker: cerulion_core::codegen::FrameWalker,
    },
}

/// Make `topic` observable and
/// return the transport, an OPEN subscriber on it, its [`TopicSource`], and — for a
/// remotely-observed topic — a [`DemandGuard`] holding the netd demand.
///
/// A GENUINELY-LOCAL topic (listed in local SHM, NOT a netd mirror, and whose
/// open-only subscriber actually opens) is read directly — this process opens NO
/// network session, so a local `topic echo`/`hz`/`info` pays zero network cost. A
/// topic ABSENT locally — OR a netd MIRROR another consumer is already streaming
/// (which is a REMOTE stream, even though its `{topic}/data` service is locally
/// openable — see [`classify_observed_topic`]) — is DEMANDED from `cerulion-netd`,
/// the ONE per-computer network gateway: this process opens a UDS
/// connection, `demand`s `(robot, topic)`, and reads the shared mirror netd
/// re-injects into the desk's SHM once (the rule: one data source = one
/// topic, obtained over the network once). netd is SPAWNED detached if it is not
/// running (the first-consumer-spawns lifecycle, by design). One loud stderr notice
/// states what happened. The `schema_hash` netd validates frames against is
/// resolved by this process over a TRANSIENT discovery session (name→hash stays a
/// consumer concern) — no
/// persistent per-consumer zenoh session.
///
/// The stale-mirror trap: a `{topic}/data` re-injection service
/// can outlive its writer (e.g. a netd mirror mid-teardown). A topic LISTED locally
/// whose `create_subscriber_open_only` fails `DoesNotExist` (the mirror's publisher
/// is gone) is a STALE mirror — do NOT surface its misleading "verify your graph
/// YAML" LOCAL error; FALL THROUGH to the netd demand rung. Only the
/// `CERULION_NETWORK=off` kill-switch skips the remote rung (local-only). The demand
/// is held by the returned [`DemandGuard`]: process exit — including a clean Ctrl-C
/// — closes its UDS connection → netd releases the demand (the crash-safe
/// refcount). A topic found NOWHERE errors precisely, naming that both local AND
/// remote were searched.
fn ensure_topic_available(
    topic: &str,
    schemas_dir: Option<&std::path::Path>,
    // The caller's cancellation flag, so the first-contact wait below stays
    // interruptible. `topic echo`/`topic hz` own one (their SIGINT handler clears it,
    // and installing that handler is what removes the default terminate disposition);
    // `topic info` installs none and passes `None`.
    running: Option<&std::sync::atomic::AtomicBool>,
) -> CliResult<(
    Arc<cerulion_core::TransportManager>,
    cerulion_core::CerulionSubscriber,
    TopicSource,
    Option<DemandGuard>,
)> {
    // The observer's transport is now strictly LOCAL-ONLY — the ONE
    // per-computer network session lives in `cerulion-netd`, not in this process.
    // A genuine LOCAL topic (a local graph produces it) is read directly with zero
    // network cost; a REMOTE topic — or a netd MIRROR another consumer is already
    // streaming — is DEMANDED from netd, which re-injects the shared frame into the
    // desk's SHM ONCE where this observer reads it via a normal subscriber (the
    // rule: one data source = one topic, obtained over the network once).
    let killed = crate::graph_cmd::remote_network_suppressed();
    let transport = cerulion_core::TransportManager::get_or_init()?;

    // Classify: a topic LISTED locally that is NOT a netd mirror is a
    // genuine local producer — read it directly. A netd MIRROR (a re-injected
    // remote stream, whose `{topic}/data` service IS locally openable) must instead
    // go through the demand plane so THIS observer holds a refcount on the shared
    // mirror (else netd would tear it down the instant the other consumer left,
    // mid-read). An absent topic likewise goes to the demand rung.
    let listed_local = topic_list()?.iter().any(|t| t.name == topic);
    // The origin robot IF `topic` is a netd mirror (in the `/__cerulion/mirrors`
    // provenance registry) — `Some` drives both the demand routing AND the accurate
    // kill-switch message below.
    let mirror_robot = gather_mirror_provenance().get(topic).cloned();
    if matches!(
        classify_observed_topic(listed_local, mirror_robot.is_some()),
        ObserveVia::LocalDirect
    ) {
        match transport.create_subscriber_open_only(topic) {
            Ok(subscriber) => return Ok((transport, subscriber, TopicSource::Local, None)),
            // DISCRIMINATE the failure. A LISTED topic whose data service
            // is GONE (`data_service_missing` ⇒ the open failed `DoesNotExist`) is
            // a STALE re-injection mirror — do NOT surface its "verify your graph
            // YAML" local error for what is really a remote topic; fall through to
            // the netd demand rung. ANY OTHER failure (slot/listener exhaustion,
            // type-skew) is on a topic that DOES exist and carries an actionable
            // remedy — surface it DIRECTLY, never masked by the remote not-found.
            Err(e) => {
                if !transport.data_service_missing(topic) {
                    return Err(e.into());
                }
                tracing::debug!(
                    topic = %topic,
                    error = %e,
                    "listed local topic's data service is gone (stale re-injection mirror) — falling through to the netd demand rung"
                );
                // fall through to the remote rung below.
            }
        }
    }

    // Under the kill-switch, the netd demand rung is unreachable. If `topic` is a
    // netd MIRROR listed locally, say so PLAINLY (it exists as a mirror of a remote
    // topic, but the kill-switch forbids demanding it — name both remedies) instead
    // of the misleading "not found locally"; otherwise it was NOT searched on any
    // robot — say that.
    if killed {
        return Err(CliError::Validation(kill_switch_unavailable_message(
            topic,
            if listed_local {
                mirror_robot.as_deref()
            } else {
                None
            },
            // The canonical-slash twin is a LOCAL fact, so it survives the
            // kill-switch: `topic info foo/bar` under CERULION_NETWORK=off must
            // still say "did you mean '/foo/bar'?" when that topic is right here.
            has_canonical_slash_twin(topic),
        )));
    }

    // Not local (and networking is allowed): can a discovered robot catalog +
    // serve it? Best-effort and bounded — but NOT sub-second:
    // this is the ABSENCE-CLAIM seam, so it waits through discovery convergence up
    // to `FIRST_CONTACT_CONVERGENCE_CEILING` (~10 s, ~15 s worst-case wall) before
    // reporting an empty answer, printing a progress line per poll. On the
    // production path the query runs over the shared `cerulion-netd` plane, not a
    // per-command transient session.
    // Resolving the wire `schema_hash` stays a CONSUMER concern;
    // netd validates every inbound frame against
    // the hash we demand with.
    let target = match resolve_remote_ingress_target(
        topic,
        schemas_dir,
        // This IS the absence-claim seam — an empty answer here becomes the
        // UNKNOWN the user reads — so it waits through convergence.
        ResolveWait::first_contact(running),
    ) {
        RemoteResolve::Found(target) => target,
        // Every non-Found outcome routes through ONE place — see
        // `unresolved_remote_error`, which is where the three different failures are
        // kept different.
        other => return Err(unresolved_remote_error(topic, other)),
    };

    // Demand the shared mirror from netd (spawning the one-per-computer daemon
    // detached if it is not running — the first-consumer-spawns lifecycle, by design),
    // then open the mirror netd created on THIS local transport. The returned
    // `DemandGuard` holds the demand for the observer's lifetime — process exit /
    // Ctrl-C closes its UDS connection → netd releases the demand (the crash-safe
    // refcount).
    let (subscriber, guard) = demand_remote_from_netd(&transport, topic, &target)?;
    Ok((
        transport,
        subscriber,
        TopicSource::Remote {
            robot: target.robot,
            walker: target.walker,
        },
        Some(guard),
    ))
}

/// How a requested `topic` should be OBSERVED given whether it is
/// listed in local SHM and whether it is a netd MIRROR (present in the
/// `/__cerulion/mirrors` provenance registry). PURE — oracle-tested.
#[derive(Debug, PartialEq, Eq)]
enum ObserveVia {
    /// Read the genuine local topic directly (no network, no netd).
    LocalDirect,
    /// Demand the topic from netd (a remote topic, OR a shared mirror this observer
    /// must refcount).
    NetdDemand,
}

/// The PURE observe-routing decision. A topic that is listed locally
/// AND is NOT a netd mirror is a genuine local producer ([`ObserveVia::LocalDirect`]);
/// everything else — a mirror (even though locally openable) or an absent topic —
/// routes through the netd demand plane ([`ObserveVia::NetdDemand`]).
fn classify_observed_topic(listed_local: bool, is_netd_mirror: bool) -> ObserveVia {
    if listed_local && !is_netd_mirror {
        ObserveVia::LocalDirect
    } else {
        ObserveVia::NetdDemand
    }
}

/// An RAII guard holding a remotely-observed topic's netd DEMAND for
/// the observer's lifetime. Dropping it closes the UDS connection to
/// `cerulion-netd`, which RELEASES the demand (the crash-safe refcount) — so the
/// shared mirror is torn down when the LAST consumer leaves, on EVERY exit path
/// (clean return / panic / Ctrl-C, all of which close the fd). The demand plane is
/// a Unix daemon, so the client is held only on Unix; the non-Unix build never
/// constructs a `DemandGuard` (the remote-observe path errors first).
pub struct DemandGuard {
    #[cfg(unix)]
    #[allow(dead_code)] // held ONLY for its Drop (connection-close = release).
    client: cerulion_netd::NetdClient,
}

/// Unix arm: demand `target`'s shared mirror from `cerulion-netd` and open
/// it on `transport`. Loud on every failure — the demand plane NEVER silently falls
/// back to a private per-consumer mirror.
#[cfg(unix)]
// Logging-rule exception: `topic echo`'s STDOUT is the frame stream a user pipes. This
// notice is deliberately on STDERR and deliberately unconditional — routing it
// through `tracing` would hide it behind RUST_LOG, and a user who does not know
// their topic is being mirrored from a robot is the condition it exists to
// prevent.
#[allow(clippy::print_stderr)]
fn demand_remote_from_netd(
    transport: &Arc<cerulion_core::TransportManager>,
    topic: &str,
    target: &RemoteIngressTarget,
) -> CliResult<(cerulion_core::CerulionSubscriber, DemandGuard)> {
    let mut client = cerulion_netd::NetdClient::connect_or_spawn().map_err(|e| {
        CliError::Validation(format!(
            "could not reach the shared network daemon (cerulion-netd) to demand remote topic \
             '{topic}' from robot '{}': {e}",
            sanitize_display(&target.robot),
        ))
    })?;
    client
        .demand(&target.robot, topic, target.schema_hash)
        .map_err(|e| {
            CliError::Validation(format!(
                "could not demand remote topic '{topic}' from robot '{}' via cerulion-netd: {e}",
                sanitize_display(&target.robot),
            ))
        })?;

    // ONE loud notice on stderr (never pollutes piped `echo` stdout).
    eprintln!(
        "[network] '{topic}' is a remote topic announced by robot '{}' — demanding it via \
         cerulion-netd (the shared per-computer gateway; released on exit)",
        sanitize_display(&target.robot),
    );

    let subscriber = open_mirror_after_demand(transport, topic)?;
    Ok((subscriber, DemandGuard { client }))
}

/// Non-Unix stub: the netd demand plane is a UDS daemon, unavailable
/// off Unix. A remote observation there is a loud error, never silent.
#[cfg(not(unix))]
fn demand_remote_from_netd(
    _transport: &Arc<cerulion_core::TransportManager>,
    topic: &str,
    target: &RemoteIngressTarget,
) -> CliResult<(cerulion_core::CerulionSubscriber, DemandGuard)> {
    Err(CliError::Validation(format!(
        "topic '{topic}' is a remote topic announced by robot '{}', but demanding it needs the \
         cerulion-netd daemon, which is Unix-only — observe it from a Linux/macOS desk",
        sanitize_display(&target.robot),
    )))
}

/// How many times [`open_mirror_after_demand`] retries opening the
/// mirror service netd just created, and the delay between attempts (~0.5 s total).
/// netd's `demand` returns only AFTER `ensure_mirror` registered the `{topic}/data`
/// service (synchronous under its registry lock), so the service exists; the retry
/// only covers the brief cross-process service-directory visibility window.
const MIRROR_OPEN_ATTEMPTS: u32 = 20;
const MIRROR_OPEN_DELAY: Duration = Duration::from_millis(25);

/// Open the mirror `cerulion-netd` just created for `topic` on the
/// LOCAL `transport`, with a bounded retry for cross-process service visibility. A
/// still-missing service after the budget is a loud error (never a silent empty
/// read).
fn open_mirror_after_demand(
    transport: &Arc<cerulion_core::TransportManager>,
    topic: &str,
) -> CliResult<cerulion_core::CerulionSubscriber> {
    let mut last: Option<CliError> = None;
    for attempt in 0..MIRROR_OPEN_ATTEMPTS {
        match transport.create_subscriber_open_only(topic) {
            Ok(subscriber) => return Ok(subscriber),
            Err(e) => {
                last = Some(e.into());
                if attempt + 1 < MIRROR_OPEN_ATTEMPTS {
                    std::thread::sleep(MIRROR_OPEN_DELAY); // ALLOW: one-time cross-process mirror-open retry at SETUP (netd is a separate process now), NOT the observer poll loop
                }
            }
        }
    }
    Err(last.unwrap_or_else(|| {
        CliError::Validation(format!(
            "cerulion-netd created the mirror for '{topic}' but it could not be opened locally"
        ))
    }))
}

/// Turn a NON-`Found` remote resolve into the error the user sees. The ONE
/// routing point for the three DIFFERENT ways a remote observe can come up empty, so
/// they cannot drift into each other:
///
/// - [`RemoteResolve::NoProducer`] — discovery RAN and no robot catalogs the topic.
///   A genuine absence claim ([`topic_not_found_anywhere`], byte-unchanged).
/// - [`RemoteResolve::DiscoveryNotConverged`] — `cerulion-netd` completed no discovery
///   pass, so nothing was searched. UNKNOWN, never absence
///   ([`discovery_not_converged_message`]).
/// - [`RemoteResolve::SchemaUnavailable`] — a robot DID catalog it but its schema is
///   unresolvable. Names the robot + cause.
///
/// `Found` is not reachable here — the caller destructures it first — but this
/// returns a LOUD internal error rather than panicking: a `RemoteResolve::Found`
/// arriving would be OUR bug, and a CLI verb must not abort the process over it
/// (the repo rule: propagate errors, map to exit codes at the CLI level). It is
/// also not swallowed — the message names the invariant so it is greppable.
/// Oracle-tested.
fn unresolved_remote_error(topic: &str, resolve: RemoteResolve) -> CliError {
    match resolve {
        RemoteResolve::Found(_) => CliError::Validation(format!(
            "internal error: topic '{topic}' resolved successfully but was routed to the \
             unresolved-remote error path — this is a Cerulion bug, please report it"
        )),
        RemoteResolve::NoProducer => topic_not_found_anywhere(topic),
        RemoteResolve::DiscoveryNotConverged => CliError::Validation(
            discovery_not_converged_message(topic, has_canonical_slash_twin(topic)),
        ),
        // The user stopped us. Say that and nothing else — the
        // observation was cut short, so it supports no claim about `topic` at all.
        RemoteResolve::Cancelled => CliError::Validation(cancelled_message(topic)),
        RemoteResolve::SchemaUnavailable { robot, cause } => CliError::Validation(format!(
            "topic '{topic}' is announced by robot '{}' but could not be demanded: {} \
             — add the type's `.msg` under `schemas/` (or update the robot to serve its \
             schema) and retry",
            sanitize_display(&robot),
            sanitize_display(&cause),
        )),
    }
}

/// The precise not-found error for a topic absent LOCALLY and on
/// every discovered robot. Keeps the canonical-slash did-you-mean the
/// transport-level open error cannot know (and no
/// misleading `topic list` steer).
fn topic_not_found_anywhere(topic: &str) -> CliError {
    CliError::Validation(not_found_message(topic, has_canonical_slash_twin(topic)))
}

/// Does `topic` differ from a LOCALLY-LISTED topic only by its leading
/// slash? A purely LOCAL question — `topic_list()` reads local SHM service discovery
/// and touches no network — so the answer is available in EVERY unresolved-topic
/// state, whatever the network did.
///
/// That matters because there are three ways for a topic to come up
/// unresolved (not found, non-convergence, and the kill-switch), and the canonical-slash
/// did-you-mean must ride ALL of them, not only [`not_found_message`]. Otherwise, on a robot-less desk
/// `topic info foo/bar` would not say "did you mean '/foo/bar'?" — about
/// a topic streaming in local SHM at that very moment — and would say the network was
/// unreachable and the answer UNKNOWN instead. The desk would know the answer and refuse
/// to give it. See [`canonical_slash_hint`].
fn has_canonical_slash_twin(topic: &str) -> bool {
    if topic.starts_with('/') {
        return false;
    }
    let slashed = format!("/{topic}");
    topic_list()
        .unwrap_or_default()
        .iter()
        .any(|t| t.name == slashed)
}

/// The canonical-slash did-you-mean FRAGMENT, or empty when there is no
/// local twin. ONE spelling shared by every unresolved-topic message (found-nowhere,
/// non-converged, kill-switched), so the affordance cannot survive on one path and
/// silently vanish on another, which is exactly what happened when the
/// non-converged path was added. PURE, oracle-tested.
fn canonical_slash_hint(topic: &str, has_slashed_twin: bool) -> String {
    if has_slashed_twin {
        format!(" — did you mean '/{topic}'? (canonical Cerulion topics start with '/')")
    } else {
        String::new()
    }
}

/// The PURE not-found message body (unit-pinned).
/// VERB-NEUTRAL — this error is emitted by `echo`, `info`, AND `hz`, so it must
/// not hardcode "echo" (the earlier text "…when you echo it" was wrong for `info`/`hz`).
/// "observe" covers all three. `has_slashed_twin` drives the canonical-slash
/// did-you-mean.
fn not_found_message(topic: &str, has_slashed_twin: bool) -> String {
    let hint = canonical_slash_hint(topic, has_slashed_twin);
    format!(
        "topic '{topic}' not found locally or on any discovered robot{hint}; run \
         `cerulion topic list` to see local + remote topics (a matching remote topic is \
         demanded automatically when you observe it)"
    )
}

/// The PURE message for a topic that is not produced locally when
/// `cerulion-netd` could NOT converge its discovery — it read nothing from the
/// network within its cold-start grace, so nothing was actually searched.
///
/// The distinction from [`not_found_message`] is the whole point of the classification: that
/// message asserts the topic is absent from every discovered robot, which is a claim
/// about the LAN. Here nothing was read, so the only true statements are (a) it is
/// not produced locally and (b) nothing on the network answered. Stating absence
/// instead is affirmatively wrong when a robot is
/// streaming the topic, and it heals on the next invocation, which teaches the user
/// the opposite of the truth.
///
/// # It must cover BOTH non-convergence causes
///
/// `DiscoveryState::NotConverged` is produced when nothing answered the announce
/// space AND when a robot WAS announced but never served its catalog (reachable —
/// netd drops robots that miss the catalog window). Flatly saying "no robot
/// was discovered … check the robot is powered on" would be FALSE in the
/// second case and would steer a user at the power switch of a robot
/// that is powered on and announcing. The wording is therefore scoped to what is
/// true in both: nothing on the network ANSWERED.
///
/// It deliberately makes no claim about WHY (powered off, wrong network, multicast
/// blocked, still booting, slow to serve): the desk cannot tell those apart, so it
/// names the checks and the retry rather than guessing. The discriminator netd DOES
/// have — whether any robot was announced — is in netd's own cold-start `warn!`.
/// Oracle-tested.
///
/// # The retry clause never promises a quick retry
///
/// "Retry — discovery often converges within a second or two" would be
/// sound advice to a user who had waited nothing. This command spends
/// up to [`FIRST_CONTACT_CONVERGENCE_CEILING`] re-asking a not-yet-converged
/// daemon BEFORE this message is ever rendered, so telling that user to retry
/// because it is usually quick contradicts the ten seconds they just watched. The
/// clause names what is TRUE on both the netd path (a ten-second wait) and the
/// transient path (a bounded sub-second gather): a robot that is booting or joining
/// can take longer to answer than this command is willing to wait.
///
/// [`FIRST_CONTACT_CONVERGENCE_CEILING`]: cerulion_netd::FIRST_CONTACT_CONVERGENCE_CEILING
fn discovery_not_converged_message(topic: &str, has_slashed_twin: bool) -> String {
    let hint = canonical_slash_hint(topic, has_slashed_twin);
    format!(
        "topic '{topic}' is not produced locally{hint}, and nothing on the network answered \
         cerulion-netd (no robot was discovered, or none served its catalog in time) — so \
         whether '{topic}' exists is UNKNOWN (this is not a claim that it is missing). \
         {RETRY_CLAUSE}"
    )
}

/// The shared closing clause of BOTH UNKNOWN messages — the retry
/// advice plus the two checks. ONE spelling so the topic and schema paths cannot
/// drift apart (they already share `canonical_slash_hint` for the same reason), and
/// so the rewording could not be applied to one and forgotten on the other.
///
/// Every clause must be true in EVERY rendering state: this
/// message renders after a ten-second netd wait AND after a sub-second transient
/// gather, so it names the SHAPE of the failure ("longer than this command waits")
/// rather than a duration only one path has.
const RETRY_CLAUSE: &str = "Retry in a few seconds — a robot that is still booting, or \
     joining the network, can take longer to answer than this command waits; if it keeps \
     failing, check the robot is powered on and on this network, and run `cerulion topic \
     list` to see which robots are discoverable from here.";

/// Did the user interrupt us before we ever observed the topic?
///
/// `topic echo` / `topic hz` install a SIGINT/SIGTERM/SIGHUP handler that only flips
/// this flag, so a signal during the first-contact wait leaves them holding an error
/// they must not render: the repo's clean-cancel precedent is exit 0 (a signalled
/// `graph run` exits 0, pinned by `signal_matrix_e2e_test`), and a command the user
/// stopped has nothing to report.
///
/// `None` (no cancellation source) is never interrupted. PURE — oracle-tested.
fn interrupted_before_observing(running: Option<&std::sync::atomic::AtomicBool>) -> bool {
    running.is_some_and(|r| !r.load(std::sync::atomic::Ordering::Relaxed))
}

/// The message for a first-contact wait the user interrupted.
///
/// It makes NO claim about `subject`: not absence, and not the claim "we searched
/// and read nothing". A cancelled observation is not evidence of anything, and the
/// earlier code rendered exactly the opposite: `WaitOutcome::Cancelled` suppressed
/// the one-line epitaph but the (empty, NotConverged) answer still flowed into the
/// absence classifier, so a Ctrl-C exited nonzero with a paragraph asserting the
/// topic's existence was UNKNOWN and telling the user to check the robot's power.
///
/// PURE — oracle-tested, including the NEGATIVE half (it must contain none of the
/// absence/UNKNOWN vocabulary the other two messages carry).
fn cancelled_message(subject: &str) -> String {
    format!("interrupted while discovering '{subject}' — nothing was concluded")
}

/// The ONE routing point from a [`RemoteSchemaFetch`] to what
/// `cerulion schema info` does — the schema-path analogue of the topic path's
/// `unresolved_remote_error`, and for the same reason.
///
/// A three-arm `match` in `main.rs` would leave the mapping with NO
/// test: replacing the `DiscoveryNotConverged` arm with
/// `return Err(local_err)` would pass every CLI test,
/// because every `schema_cli_test` arm sets `CERULION_NETWORK=off` and so routes
/// kill-switch → `NotServed` → `local_err`, structurally never reaching that arm.
///
/// Keeping the variant match here keeps it out of `main.rs` entirely — the caller
/// has a single `?`, with no arm left to mis-route — and puts the decision under
/// oracle tests. `local_err` is the caller's LOCAL `SchemaNotFound`, re-raised
/// VERBATIM for a settled absence so that path's exit code and text stay unchanged.
pub fn schema_info_remote_outcome(
    requested: &str,
    fetch: RemoteSchemaFetch,
    local_err: CliError,
) -> CliResult<cerulion_core::SchemaReply> {
    match fetch {
        // A robot served it — the caller renders it with its provenance header.
        RemoteSchemaFetch::Found(reply) => Ok(reply),
        // Discovery RAN and nobody serves it (or the user asked for local-only): the
        // local "not found" is the correct answer, verbatim.
        RemoteSchemaFetch::NotServed => Err(local_err),
        // Nothing was READ, so "it does not exist" is a claim with no evidence.
        RemoteSchemaFetch::DiscoveryNotConverged => Err(CliError::Validation(
            schema_discovery_not_converged_message(requested),
        )),
        // Interrupted — see `cancelled_message`.
        RemoteSchemaFetch::Cancelled => Err(CliError::Validation(cancelled_message(requested))),
    }
}

/// The SCHEMA-path twin of `discovery_not_converged_message` (private,
/// so deliberately not linked) — the message `cerulion schema info <type>` raises
/// when the type is unresolvable locally AND nothing on the network answered.
///
/// Re-raising the terminal LOCAL `SchemaNotFound` for that case would make a cold
/// daemon that had searched nothing render as proof the type does not exist —
/// the same false-absence the topic path refuses to make.
///
/// It states BOTH facts, because both are true and the user needs each: the type is
/// genuinely not in this workspace or the built-in registry (so a typo still reads
/// as a typo, and still gets its LOCAL remedy), AND nothing on the network answered,
/// so whether a ROBOT has it is UNKNOWN. Like its twin it makes no claim about WHY
/// (powered off, wrong network, multicast blocked, still booting) — the desk cannot
/// tell those apart, so it names the checks and the retry rather than guessing.
///
/// # Every clause must be true in EVERY rendering state
///
/// Two rules follow. It must NOT name `cerulion-netd` ("nothing on the network answered
/// cerulion-netd"), because this message also renders on the TRANSIENT path — explicit
/// `--connect`/`--listen` locators, or a netd that was unreachable — where no daemon
/// was involved at all, and a test pins exactly that
/// netd-free shape. It says "nothing on the network answered", which is true on
/// both paths. And the local fact must carry its own remedy, or
/// the overwhelmingly common case (a typo, on a desk with no robots) gets a paragraph
/// about robots and nothing about the thing it could actually fix — so the local
/// remedy leads. Oracle-tested.
///
/// Its closing clause follows the same rule as its topic twin,
/// through the SAME shared `RETRY_CLAUSE` const (private, so deliberately not
/// linked) — `cerulion schema info` also waits
/// through convergence before rendering this, so "converges within a second or two"
/// would be a claim contradicted by the wait the user had just watched.
pub fn schema_discovery_not_converged_message(requested: &str) -> String {
    format!(
        "schema '{requested}' is not in this workspace or the built-in ROS 2 registry — \
         add its `.msg`/YAML under `schemas/`, or run `cerulion schema list` to see what \
         is available. Nothing on the network answered either (no robot was discovered, \
         or none served its schemas in time), so whether any robot has '{requested}' is \
         UNKNOWN — this is not a claim that it does not exist. {RETRY_CLAUSE}"
    )
}

/// The PURE `CERULION_NETWORK=off` unavailable
/// message. `mirror_robot = Some(robot)` means `topic` IS listed locally as a netd
/// MIRROR of `robot`'s topic — the kill-switch forbids DEMANDING it (participating
/// in netd's shared refcount), so say so PLAINLY and name BOTH remedies (unset the
/// kill-switch here, or read it on the source robot) — NEVER the misleading "not
/// found locally". `None` = the topic is genuinely absent locally (no mirror), so it
/// was never searched on any robot under the kill-switch. Robot terminal-escape-
/// sanitized. Oracle-tested.
fn kill_switch_unavailable_message(
    topic: &str,
    mirror_robot: Option<&str>,
    has_slashed_twin: bool,
) -> String {
    match mirror_robot {
        Some(robot) => {
            let robot = sanitize_display(robot);
            format!(
                "topic '{topic}' exists locally as a network mirror of robot '{robot}'s topic, but \
                 CERULION_NETWORK=off forbids demanding it from cerulion-netd — unset \
                 CERULION_NETWORK to observe it here, or read it directly on robot '{robot}'"
            )
        }
        None => {
            let hint = canonical_slash_hint(topic, has_slashed_twin);
            format!(
                "topic '{topic}' not found locally{hint}, and remote discovery is disabled by \
                 CERULION_NETWORK=off — unset it to search robots on the LAN (a matching \
                 remote topic is then demanded automatically via cerulion-netd)"
            )
        }
    }
}

// ─── `topic list --network` remote discovery ────────────

/// The bounded window `query_remote_topics` waits for liveliness /
/// announce replies PER key-space. Zenoh's default query timeout is
/// seconds-long (fine for a service, too slow for an interactive `topic
/// list`), so the reply-gather is capped here. 500 ms comfortably covers a
/// same-LAN round-trip while keeping the CLI snappy. The demand + announce
/// gathers run concurrently, so this window is the
/// worst-case remote wait — not half of it.
pub const REMOTE_QUERY_GATHER_WINDOW: Duration = Duration::from_millis(500);

/// The bounded window ONE per-robot `catalog` GET waits for its reply.
/// Materially SHORTER than [`REMOTE_QUERY_GATHER_WINDOW`] (250 ms vs 500 ms): a
/// healthy catalog GET is answered by the robot's already-connected gateway in
/// single-digit ms, so the full window is only ever paid for a robot that never
/// answers (an older binary with no query surface — the announce fallback carries
/// it). Keeping this half the announce window bounds the SECOND remote round's
/// contribution to the `topic list` wall (see `query_catalogs_for_robots`).
pub const CATALOG_GATHER_WINDOW: Duration = Duration::from_millis(250);

/// Session options for `topic list` remote discovery.
///
/// The CLI runs OUTSIDE any graph, so a graph's `network:` block does NOT
/// apply here — the user may pass extra zenoh locators explicitly. Both lists
/// map 1:1 onto zenoh locator strings and onto
/// `NetworkConfig::{connect,listen}_endpoints`.
///
/// `scouting` maps onto BOTH `multicast_scouting` and `gossip_scouting`. The
/// struct DEFAULT (via `Default`/field init) keeps scouting OFF so engine
/// unit + live tests stay hermetic (an isolated session discovers only what
/// its explicit locators reach); the CLI dispatch (`topic list`) flips it ON
/// — that's where the automagic default lives, NOT in this struct.
pub struct RemoteTopicsOptions {
    /// Remote locators to connect to (e.g. `tcp/192.168.123.99:7683` — a
    /// permissive robot's gateway listens on the well-known 7683).
    pub connect: Vec<String>,
    /// Local locators to listen on (e.g. `tcp/0.0.0.0:7447`).
    pub listen: Vec<String>,
    /// Enable multicast + gossip scouting (LAN auto-discovery). The CLI-path
    /// default is `true` (automagic); tests keep it `false` for hermeticity.
    pub scouting: bool,
}

impl RemoteTopicsOptions {
    /// True when at least one locator was given — drives the accurate
    /// empty-result hint (a scouting-off, endpoint-less session CANNOT reach a
    /// remote peer, and the hint must say so instead of implying "nothing is
    /// publishing").
    pub fn has_endpoints(&self) -> bool {
        !self.connect.is_empty() || !self.listen.is_empty()
    }
}

/// Build the automagic `topic list` discovery
/// options — THE one place the CLI-path scouting-ON default lives. The binary
/// dispatch (`cerulion topic list`) is a thin passthrough to this fn, so the
/// default is unit-pinnable here (`cerulion_cli` has no lib target): a
/// regression flipping scouting off would silently disable zero-flag discovery.
///
/// - `no_network == true` (`--no-network`) ⇒ `None`: skip the remote query
///   entirely (scripts / CI / offline).
/// - otherwise ⇒ `Some` with multicast + gossip SCOUTING ON (unpaired robots
///   on the LAN are discoverable with zero flags) and any `--connect` /
///   `--listen` locators carried ADDITIVELY (they extend the scouting
///   session; they never replace it).
pub fn remote_discovery_options(
    no_network: bool,
    connect: Vec<String>,
    listen: Vec<String>,
) -> Option<RemoteTopicsOptions> {
    if no_network {
        return None;
    }
    Some(RemoteTopicsOptions {
        connect,
        listen,
        scouting: true,
    })
}

/// The ROBOTS surface of one `topic list` remote query.
///
/// `topics` is the merged canonical-topic list (the topics-only output). `robots`
/// are the presence-derived ROBOTS rows (a row exists ONLY from
/// evidence of a LIVE gateway — an announce token that actually arrived, or an
/// mDNS browse answer — never from a cache/hostname/scan candidate alone; see
/// [`build_robot_rows`]). `peers` are the ladder's raw `(robot, locator, rung)`
/// finds (empty whenever the ladder did not run — e.g. scouting off in hermetic
/// tests); their locators feed the query session's connect set. Only the mDNS
/// peers count as reachability evidence for the empty-topics hint, and only
/// they enrich rows — the non-mDNS finds render as clearly-labeled UNVERIFIED
/// candidates under the ROBOTS rows.
#[derive(Debug, Clone)]
pub struct RemoteDiscovery {
    /// Canonical topic names discovered on the DEMAND + ANNOUNCE key-spaces.
    pub topics: Vec<String>,
    /// Presence-derived ROBOTS rows (announce presence + mDNS enrichment).
    pub robots: Vec<RobotRow>,
    /// Gateways the discovery ladder surfaced (mDNS / cache / hostname /
    /// scan). Their locators fed the query session's connect set.
    pub peers: Vec<DiscoveredPeer>,
    /// The RAW ANNOUNCE entries `(robot, Option<canonical
    /// topic>)` this gather observed — the SOURCE data the `robots` rows + the
    /// merged `topics` list are derived from. `topics` STRIPS the robot chunk
    /// (a `/tf` two robots both announce dedups to one row), so this is the ONLY
    /// per-robot topic ATTRIBUTION in the result. A `(robot, None)` entry is the
    /// bare identity token (presence, no topic). Empty whenever the
    /// gather ran but nothing announced (or in hermetic topics-only test rows).
    pub announce_entries: Vec<(String, Option<String>)>,
}

impl RemoteDiscovery {
    /// An empty discovery — no network topics, robots, peers, or
    /// announce entries. Used to render the REMOTE section when the network half is
    /// skipped (`--no-network`) or failed but LOCAL mirror-streaming rows still
    /// need to surface (a mirror is REMOTE regardless of the network query, since
    /// its provenance is read from LOCAL SHM).
    pub fn empty() -> Self {
        Self {
            topics: vec![],
            robots: vec![],
            peers: vec![],
            announce_entries: vec![],
        }
    }
}

/// How a [`RobotRow`] was surfaced — its render tag + the evidence
/// class. Only mDNS enriches/creates a locator-bearing row; a
/// cache/hostname/scan candidate never creates a row (it renders on the
/// labeled unverified-candidates line instead).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RobotProvenance {
    /// The row's evidence is a live ANNOUNCE token (its produced topics arrived
    /// in the gather). Renders `(announce)`.
    Announce,
    /// The row was surfaced or enriched by a discovery-ladder rung (mDNS today —
    /// a browse answer is intrinsically a live gateway). Renders the rung name.
    Rung(DiscoveryRung),
}

impl std::fmt::Display for RobotProvenance {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RobotProvenance::Announce => f.write_str("announce"),
            RobotProvenance::Rung(rung) => write!(f, "{rung}"),
        }
    }
}

/// One ROBOTS row — evidence of a LIVE gateway (never a mere
/// candidate). Built by [`build_robot_rows`] from the announce gather + the
/// ladder's mDNS peers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RobotRow {
    /// Robot identity — the announce keys' robot chunk and/or the mDNS TXT
    /// name (the SAME string by construction: both are stamped from the
    /// resolved robot identity — the hostname or `CERULION_ROBOT_IDENTITY`
    /// override, NOT the graph prefix).
    pub robot: String,
    /// The gateway locator (the mDNS SRV record), when known. `None` for an
    /// announce-only row (mDNS did not find this robot's address).
    pub locator: Option<String>,
    /// The row's provenance / render tag.
    pub provenance: RobotProvenance,
    /// The count of DISTINCT visible announced topics for this robot (0 for an
    /// mDNS-only row or a bare-identity-token-only gateway that announced no
    /// topic in the gather).
    pub topic_count: usize,
}

/// Group the ANNOUNCE gather's `(robot, Option<canonical topic>)`
/// entries into `(robot, distinct_topic_count)` — each distinct robot chunk is
/// one candidate ROBOTS row. EXACT grouping (the robot chunk is carried in the
/// announce key — `cerulion_ann/{robot}{topic}` — so absolute mirror topics
/// like `/lf/lowstate` attribute to their PRODUCER, never to a phantom "lf"
/// robot). A `(robot, None)` entry is the bare identity token: it creates the
/// robot with ZERO topics. Topic counts are DISTINCT canonical topics
/// (`BTreeSet` — two publishers of one `/tf` count once, matching the deduped
/// REMOTE TOPICS list). Sorted by robot name for deterministic presentation.
/// Pure — oracle-tested below.
pub fn group_announce_by_robot(
    announce_entries: &[(String, Option<String>)],
) -> Vec<(String, usize)> {
    let mut topics_by_robot: BTreeMap<&str, std::collections::BTreeSet<&str>> = BTreeMap::new();
    for (robot, topic) in announce_entries {
        let set = topics_by_robot.entry(robot.as_str()).or_default();
        if let Some(topic) = topic {
            set.insert(topic.as_str());
        }
    }
    topics_by_robot
        .into_iter()
        .map(|(robot, topics)| (robot.to_string(), topics.len()))
        .collect()
}

/// Build the presence-derived ROBOTS rows from the ANNOUNCE gather +
/// the ladder's peers.
///
/// - **Announce presence** — each distinct robot chunk in the announce entries
///   ([`group_announce_by_robot`]) seeds a row (provenance
///   [`RobotProvenance::Announce`], no locator, its DISTINCT-topic count; a
///   bare identity token seeds a zero-topic row).
/// - **mDNS enrichment** — an mDNS peer (rung [`DiscoveryRung::Mdns`]; a browse
///   answer is a LIVE gateway) whose robot matches an announce row ENRICHES it
///   (adds the locator, upgrades the provenance to the rung); an mDNS peer with
///   NO matching announce still gets its OWN row (it is a live gateway that
///   simply produced no topic). The merge is name-keyed and exact by
///   construction: the mDNS TXT `robot=` value and the announce robot chunk
///   are BOTH the resolved robot identity (hostname / `CERULION_ROBOT_IDENTITY`
///   override, never the graph prefix). The FIRST mDNS locator per robot wins
///   (the ladder already deduped by resolved `(ip, port)` and sorted, so this
///   is deterministic).
/// - Cache / hostname / scan peers NEVER create a row — their locators only fed
///   the query session's connect set; if their robot is alive its announce
///   tokens surface it via the first rule.
///
/// Result rows are sorted by robot name (`BTreeMap` key order). Pure —
/// oracle-tested below.
pub fn build_robot_rows(
    announce_entries: &[(String, Option<String>)],
    peers: &[DiscoveredPeer],
) -> Vec<RobotRow> {
    let mut rows: BTreeMap<String, RobotRow> = BTreeMap::new();
    for (robot, topic_count) in group_announce_by_robot(announce_entries) {
        rows.insert(
            robot.clone(),
            RobotRow {
                robot,
                locator: None,
                provenance: RobotProvenance::Announce,
                topic_count,
            },
        );
    }
    // Only mDNS peers enrich/create rows (a browse answer is a live gateway).
    for peer in peers.iter().filter(|p| p.rung == DiscoveryRung::Mdns) {
        match rows.get_mut(&peer.robot) {
            Some(row) => {
                // First mDNS locator per robot wins (peers are pre-sorted).
                if row.locator.is_none() {
                    row.locator = Some(peer.locator.clone());
                }
                row.provenance = RobotProvenance::Rung(peer.rung);
            }
            None => {
                rows.insert(
                    peer.robot.clone(),
                    RobotRow {
                        robot: peer.robot.clone(),
                        locator: Some(peer.locator.clone()),
                        provenance: RobotProvenance::Rung(peer.rung),
                        topic_count: 0,
                    },
                );
            }
        }
    }
    rows.into_values().collect()
}

/// One-shot remote discovery for `topic list`.
///
/// When `opts.scouting` is set (the automagic CLI path), the discovery
/// LADDER runs FIRST (`discovery_ladder::discover_peers` — mDNS / cache /
/// hostname / scan) and each found gateway locator is folded into the session's
/// connect endpoints (deduped by exact string) so the gathers reach robots
/// scouting alone might miss. The ladder opens NO zenoh session — it produces
/// candidate locators; this ONE query session connects to everything. Hermetic
/// tests keep scouting OFF, so the ladder (and its mDNS/multicast I/O) never runs
/// and `peers` stays empty.
///
/// Opens a short-lived zenoh session (peer mode; the CLI process's ONE session,
/// Principle #8) with BOUNDED connect (`bounded_connect: true` — the connect
/// phase is HARD-BOUNDED at 1 s, so a black-hole connect locator can never
/// stall the command past that bound; reachable robots connect inline BEFORE
/// the open returns) and performs TWO BOUNDED gathers CONCURRENTLY over
/// [`REMOTE_QUERY_GATHER_WINDOW`]: the DEMAND topic key-space and the ANNOUNCE
/// key-space — so the worst-case remote wait stays ~ONE window after the
/// connect bound. The session closes when the manager drops.
///
/// ROBOTS are PRESENCE-derived: a row exists only from a LIVE gateway —
/// an announce entry that actually arrived (its robot chunk is the EXACT
/// producer identity) enriched by the ladder's mDNS peers
/// ([`build_robot_rows`]), or an mDNS browse answer alone; a dead
/// cache/hostname/scan candidate surfaces no row. After the gather, robots
/// CONFIRMED live are written back to the peer cache (only on the scouting
/// path — hermetic tests never touch the real `~/.cerulion/peers.json`; the
/// decision is the oracle-tested [`resolve_write_back`]).
///
/// Errors are LOUD, never an empty result: a session that cannot open (bad
/// locator, bind failure, unreachable client-mode router) and a failed
/// gather both map to `CliError::Validation` with the cause and the expected
/// locator form — an empty `RemoteDiscovery` always means "the session opened
/// and nothing remote is advertising".
pub fn query_remote_topics(opts: &RemoteTopicsOptions) -> CliResult<RemoteDiscovery> {
    // Back-compat entry: the DEFAULT discovery path never runs the opt-in
    // subnet sweep. This scan-OFF wrapper is the sole default entry point (the
    // live tests + the malformed-locator gate call it) — the sweep is reachable
    // only through [`query_remote_topics_with_scan`] with `scan = true`, whose
    // sole producer is the `--scan` CLI flag (its structural gate).
    query_remote_topics_with_scan(opts, false)
}

/// The scan-aware remote-discovery entry. Identical to
/// [`query_remote_topics`] except `scan` (the `--scan` flag) appends the opt-in
/// subnet-sweep rung to the discovery ladder (see
/// [`crate::discovery_ladder::discover_peers`]). The sweep still only runs when
/// `opts.scouting` is set (the automagic CLI path), so hermetic scouting-off
/// tests never trigger it regardless of `scan`. Runs the ladder, then delegates
/// the pre-filter + fold + open to [`query_remote_topics_with_candidates`].
pub fn query_remote_topics_with_scan(
    opts: &RemoteTopicsOptions,
    scan: bool,
) -> CliResult<RemoteDiscovery> {
    // The ladder produces candidate LOCATORS; scouting off (hermetic tests)
    // skips it entirely. The pre-filter + fold + open live in the delegate.
    let ladder_peers = if opts.scouting {
        crate::discovery_ladder::discover_peers(scan)
    } else {
        Vec::new()
    };
    query_remote_topics_with_candidates(opts, ladder_peers)
}

/// Remote discovery over an EXPLICITLY-supplied set
/// of `ladder_peers` (the automagic path passes the live discovery ladder's
/// output; tests / embedders pass a crafted set). This is the STALL-DEFENSE
/// seam: the bounded connect is a HARD 1 s bound, but that bound is FATAL to
/// `zenoh::open` (endpoints are tried SEQUENTIALLY and the global-timeout Err
/// propagates unconditionally, so ONE hanging endpoint sinks every OTHER
/// reachable robot in the gather), so two guards apply here.
///
/// 1. **Parallel TCP pre-filter** ([`crate::discovery_ladder::plan_connect_set`]
///    over two CONCURRENT [`crate::discovery_ladder::probe_reachable_locators`]
///    batches): a locator is folded into the connect set ONLY if it accepts a
///    TCP connection within its budget — LADDER candidates at
///    [`crate::discovery_ladder::LADDER_PROBE_TIMEOUT`] (300 ms), EXPLICIT
///    `--connect` locators at [`crate::discovery_ladder::EXPLICIT_PROBE_TIMEOUT`]
///    (1 s = the connect bound). Anything unreachable within its budget could
///    never have contributed to the gather within the connect bound, so dropping
///    it loses nothing: a dead ladder candidate is dropped (DEBUG); an
///    unreachable explicit locator is dropped with a LOUD WARN (the policy
///    change from the old "fold anyway" — one hanging explicit locator must not
///    sink the gather). Running the two batches concurrently keeps the worst-case
///    pre-filter wall at ~1.2 s only when explicit locators exist.
/// 2. **Open-failure fallback**: a locator that PASSES the TCP probe can still
///    poison the open (a tarpit that accepts TCP but never speaks zenoh). If the
///    open fails AND ladder candidates were folded, retry the open ONCE with the
///    REACHABLE-explicit set only (the probed survivors — never the raw
///    `opts.connect`), so a discovered false positive never loses the gather.
///    RESIDUAL: a tarpit given as an EXPLICIT locator passes its probe, poisons
///    the open, and — with no folded ladder candidates — the open error
///    propagates to the loud-note-exit-0 path; that is the accepted worst case
///    for a user who explicitly names a tarpit (its own reachable siblings are
///    lost this run, but the local list already printed and the note is loud).
pub fn query_remote_topics_with_candidates(
    opts: &RemoteTopicsOptions,
    ladder_peers: Vec<DiscoveredPeer>,
) -> CliResult<RemoteDiscovery> {
    // ── Syntax gate (BEFORE any probing): explicit `--connect` locators are
    //    USER INPUT, so a SYNTACTICALLY malformed one (a typo with no
    //    proto/host:port shape) is a HARD, actionable error — never a silent
    //    warn+skip that would hide a typo behind an empty result. (A
    //    syntactically-VALID-but-unreachable explicit locator — a host that is
    //    down or unresolvable this run — is still warn+skipped by the pre-filter
    //    below; only a malformed one is a hard error, matching the pre-pre-filter
    //    behavior where the raw string failed the zenoh session open loudly.)
    let mut malformed: Vec<&str> = Vec::new();
    for loc in &opts.connect {
        if !crate::discovery_ladder::locator_is_syntactically_valid(loc) {
            malformed.push(loc.as_str());
        }
    }
    if !malformed.is_empty() {
        return Err(CliError::Validation(format!(
            "malformed --connect locator(s) {malformed:?} for the zenoh discovery session \
             — use the zenoh form tcp/<host>:<port> (e.g. tcp/192.168.123.99:7683)"
        )));
    }

    // ── Pre-filter: probe the two endpoint classes CONCURRENTLY at their own
    //    budgets (explicit @1 s = the connect bound, ladder @300 ms), then PLAN
    //    the connect set (fold only what probed reachable within its budget).
    let explicit = opts.connect.clone();
    let ladder_locators: Vec<String> = ladder_peers.iter().map(|p| p.locator.clone()).collect();
    let reachable = std::thread::scope(|s| {
        let explicit_handle = s.spawn(|| {
            crate::discovery_ladder::probe_reachable_locators(
                &explicit,
                crate::discovery_ladder::EXPLICIT_PROBE_TIMEOUT,
            )
        });
        let mut reachable = crate::discovery_ladder::probe_reachable_locators(
            &ladder_locators,
            crate::discovery_ladder::LADDER_PROBE_TIMEOUT,
        );
        // The union: an explicit locator's reachability rides the 1 s budget.
        reachable.extend(explicit_handle.join().unwrap_or_default());
        reachable
    });
    let plan = crate::discovery_ladder::plan_connect_set(&opts.connect, &ladder_peers, &reachable);
    for loc in &plan.explicit_unreachable {
        tracing::warn!(
            locator = %loc,
            timeout_ms = crate::discovery_ladder::EXPLICIT_PROBE_TIMEOUT.as_millis() as u64,
            "--connect locator did not answer a TCP connect within 1s — it cannot contribute \
             within the discovery session's connect bound; skipping it this run"
        );
    }
    for (loc, rung) in &plan.dropped_ladder {
        tracing::debug!(
            locator = %loc,
            rung = %rung,
            "dropping unreachable ladder candidate from the discovery connect set"
        );
    }
    // The reachable-explicit set drives the open-failure fallback (retry without
    // the folded ladder candidates, keeping only the probed-reachable explicit
    // locators — never the raw opts.connect, which may hold an unreachable one).
    let reachable_explicit: Vec<String> = opts
        .connect
        .iter()
        .filter(|loc| reachable.contains(*loc))
        .cloned()
        .collect();

    // ── Open the bounded-connect discovery session. If the open FAILS and we
    //    folded ladder candidates, a tarpit candidate may have passed the TCP
    //    probe but poisoned the zenoh handshake — retry ONCE with the
    //    reachable-explicit set only (never lose the gather to a discovered
    //    false positive).
    let mgr = match open_discovery_session(&plan.connect, &opts.listen, opts.scouting) {
        Ok(m) => m,
        Err(first_err) if !plan.folded_ladder.is_empty() => {
            tracing::warn!(
                error = %first_err,
                dropped = ?plan.folded_ladder,
                "a discovered candidate poisoned the discovery session open; retrying with the \
                 reachable-explicit locators only"
            );
            open_discovery_session(&reachable_explicit, &opts.listen, opts.scouting)?
        }
        Err(first_err) => return Err(first_err),
    };
    let session = mgr
        .session()
        .expect("session already opened by open_discovery_session");
    // Query the DEMAND (ingress-interest) topic key-space +
    // the ANNOUNCE key-space (whose keys carry the producing robot chunk —
    // presence = verification, see `build_robot_rows`). A canonical name
    // advertised in both spaces (a producer that also ingresses) dedups to one
    // row after the announce entries' robot chunks are stripped.
    //
    // The demand + announce gathers run concurrently (a
    // scoped thread), so THIS round's worst-case remote wait is ~ONE
    // `REMOTE_QUERY_GATHER_WINDOW` — sequential gathers would double it.
    // NOTE (worst case): when robots ARE present a SECOND round follows —
    // the per-robot `catalog` GETs (`query_catalogs_for_robots`), which fan out
    // concurrently but are bounded by a SEPARATE, shorter `CATALOG_GATHER_WINDOW`
    // (250 ms). So with robots present the remote wall is
    // `REMOTE_QUERY_GATHER_WINDOW` + `CATALOG_GATHER_WINDOW` (~750 ms worst case,
    // and only when a robot never answers the catalog GET); with NO robots the
    // second round is skipped entirely (still ~one 500 ms window). Accepted:
    // the catalog is the richer data path and its window is deliberately
    // half the announce window so the added tail stays sub-second. The
    // two rounds are not overlapped (the catalog GET needs the identities
    // the announce round harvests).
    // Safe: `zenoh::Session` is `Send + Sync` (an `Arc` handle over atomic/lock
    // state — the scoped borrow proves it at compile time) and each liveliness
    // `get` is an independent query.
    let (demand, announce) = std::thread::scope(|s| {
        let announce_handle = s.spawn(|| {
            cerulion_core::transport::discovery::query_announce_entries(
                session,
                REMOTE_QUERY_GATHER_WINDOW,
            )
        });
        let demand = cerulion_core::transport::discovery::query_live_topics_by_prefix(
            session,
            cerulion_core::transport::discovery::liveliness_prefix(),
            REMOTE_QUERY_GATHER_WINDOW,
        );
        let announce = match announce_handle.join() {
            Ok(res) => res,
            Err(_) => Err(cerulion_core::TransportError::Internal {
                reason: "the announce-space discovery gather panicked".to_string(),
            }),
        };
        (demand, announce)
    });
    // GET each discovered robot's CATALOG over the `cerulion_q` query
    // surface as the RICHER data path — the gateway's authoritative
    // registered/announced topic set (+ schema hashes). Uses EXPLICIT-robot
    // selectors (identities harvested from the announce gather; a mid-key wildcard
    // computes an empty route on a real link). Best-effort ENRICHMENT: a robot
    // that does not answer (an older binary with no query surface) or answers with
    // an unknown wire version falls back to its announce-derived listing (already
    // folded into `all` below). Runs while the session is OPEN, so it MUST precede
    // `mgr.close()` and the result unwraps below.
    let catalog_topics: Vec<String> = match &announce {
        Ok(entries) => {
            // Distinct robot identities from the announce gather (the proven
            // direction) → EXPLICIT-robot catalog GETs (never a mid-key wildcard).
            let robots: Vec<String> = entries
                .iter()
                .map(|(robot, _)| robot.as_str())
                .collect::<BTreeSet<&str>>()
                .into_iter()
                .map(|robot| robot.to_string())
                .collect();
            // `.replies` alone. This is the ENRICHMENT path — the result
            // is folded into a topic listing and no absence is claimed from it, so a
            // robot lost to a panicked worker costs its rows and nothing else. The
            // gather has already named it at ERROR. The two sites below DO claim an
            // absence and read `is_complete()` accordingly.
            cerulion_core::transport::discovery::query_robot_catalogs(
                session,
                &robots,
                CATALOG_GATHER_WINDOW,
            )
            .replies
            .into_iter()
            .flat_map(|catalog| catalog.entries.into_iter().map(|entry| entry.topic))
            .collect()
        }
        // A failed announce gather yields no robot identities — the demand-space
        // topics still carry the run; the announce error is surfaced on unwrap.
        Err(_) => Vec::new(),
    };
    // The gathers are done and joined, so this ephemeral
    // discovery session is finished — close it in order (deterministic resource
    // release) BEFORE `mgr` drops, rather than leaving it to the implicit `Drop`
    // close. This close does NOT silence the cosmetic zenoh-internal `session
    // closed` ERROR that fires during teardown — that is suppressed at the log
    // layer via `zenoh::api::admin=off` in `cerulion_core::init_logging`.
    // Placed after the scope so it also runs on the post-gather error arms below.
    mgr.close();
    let demand =
        demand.map_err(|e| CliError::Validation(format!("remote topic discovery failed: {e}")))?;
    let announce_entries = announce
        .map_err(|e| CliError::Validation(format!("remote topic discovery failed: {e}")))?;

    // ROBOTS = live presence: group the ANNOUNCE entries by their EXACT robot
    // chunk and enrich with the ladder's mDNS peers. A dead
    // cache/hostname/scan candidate creates no row.
    let robots = build_robot_rows(&announce_entries, &ladder_peers);

    // Post-gather cache write-back — only on the scouting path (the oracle-
    // tested `resolve_write_back` gate: hermetic scouting-off tests must NEVER
    // touch the user's `~/.cerulion/peers.json`). Record only robots CONFIRMED
    // live by the gather: an mDNS-enriched row upserts its `(robot, locator)`;
    // an announce-only row has no verified locator and is logged, never cached.
    if let Some(confirmed) = resolve_write_back(opts.scouting, &robots) {
        match crate::peer_cache::default_cache_path() {
            Some(path) => crate::peer_cache::record_confirmed(&path, &confirmed),
            None => tracing::warn!(
                "no home directory; cannot locate the peer cache — not recording peers"
            ),
        }
    }

    // The REMOTE TOPICS list is the merged demand + announce spaces — the
    // announce entries' robot chunks are STRIPPED (each entry's canonical-topic
    // half), so a topic two robots both announce (e.g. /tf) dedups to one row.
    // Keep the RAW `announce_entries` for `RemoteDiscovery`
    // (per-robot topic attribution `topics` cannot recover), so clone the topic
    // halves into `all` rather than consuming the vec.
    let mut all = demand;
    all.extend(
        announce_entries
            .iter()
            .filter_map(|(_, topic)| topic.clone()),
    );
    // Fold in the per-robot catalogs (the richer data path). Additive —
    // `normalize_remote_topics` dedups a catalog topic against its announce twin,
    // so a robot that answered contributes any catalog-only topics while a robot
    // that did NOT answer is unaffected (its announce topics above stand).
    all.extend(catalog_topics);

    Ok(RemoteDiscovery {
        topics: normalize_remote_topics(all),
        robots,
        peers: ladder_peers,
        announce_entries,
    })
}

/// Seed a [`FrameWalker`](cerulion_core::codegen::FrameWalker)
/// IN MEMORY from a robot-served schema closure (`.msg`/YAML texts) + the desk's
/// built-in ROS 2 corpus — so a frame carrying a CUSTOM type the desk never
/// compiled can be decoded structurally, with NO disk writes. Each `msg` doc is
/// parsed via `parse_rosmsg`, each `yaml` doc via the workspace YAML parser, and
/// the built-in corpus is folded in so nested built-in references
/// (`std_msgs/Header`, …) resolve during layout computation. A doc that fails to
/// parse is skipped with a `warn!` (the rest still seed). PURE (no I/O) — the
/// fetch is the caller's; this is the decode-seed half, oracle-tested.
pub fn seed_framewalker(docs: &[cerulion_core::SchemaDoc]) -> cerulion_core::codegen::FrameWalker {
    use cerulion_core::codegen::{parse_rosmsg, MessageSchema};
    let mut schemas: Vec<MessageSchema> = crate::schema_cmd::parse_builtin_schemas();
    for doc in docs {
        match doc.encoding {
            cerulion_core::SchemaEncoding::Msg => {
                let (pkg, ty) = match doc.qualified.split_once('/') {
                    Some((p, t)) => (Some(p), t),
                    None => (None, doc.qualified.as_str()),
                };
                match parse_rosmsg(&doc.text, ty, pkg) {
                    Ok(schema) => schemas.push(schema),
                    Err(e) => tracing::warn!(
                        qualified = %doc.qualified, error = ?e,
                        "seed: could not parse a served .msg — skipped"
                    ),
                }
            }
            cerulion_core::SchemaEncoding::Yaml => {
                match crate::schema_cmd::parse_message_schemas(&doc.text) {
                    Ok(parsed) => schemas.extend(parsed),
                    Err(e) => tracing::warn!(
                        qualified = %doc.qualified, error = %e,
                        "seed: could not parse a served YAML schema — skipped"
                    ),
                }
            }
        }
    }
    // A served closure is UNTRUSTED — `FrameWalker::new` runs
    // `resolve_fixed_nested`, which PANICS materializing a composed-overflow
    // doc referenced as a fixed target, so one hostile
    // closure from a peer crashed `topic echo`. The ONE shared preflight
    // thins the set first; a dropped doc's referrers degrade to variable.
    crate::schema_cmd::preflight_resolution_set(&mut schemas, "served-doc frame walker");
    let (walker, warnings) = cerulion_core::codegen::FrameWalker::new(schemas);
    for w in warnings {
        tracing::debug!(warning = %w, "seed: FrameWalker resolution warning");
    }
    walker
}

/// Seed a [`FrameWalker`](cerulion_core::codegen::FrameWalker)
/// from the LOCAL schema corpus — the desk's built-in ROS 2 types PLUS (when a
/// workspace is present) its `.msg` store (`schemas/<pkg>/msg/<Type>.msg`) and
/// its `schemas/*.yaml` schemas — so `topic echo` decodes a frame carrying a
/// WORKSPACE type LOCALLY, without ever touching the network.
///
/// Live-proven gap (Go2): on the robot itself, in the go2demo
/// workspace whose `schemas/` holds `unitree_go/*.msg`, `topic echo /lf/lowstate`
/// fell through to the REMOTE tier because the echo path built NO local walker —
/// it only knew two pinned built-in hashes. This builds the missing local walker
/// (the workspace-wins ladder: built-ins, then the store, then workspace
/// YAML, so a workspace type shadows a same-named built-in) with the store
/// type's recipe-3 `schema_hash` present, so the wire hash resolves locally and
/// the remote fetch stays the FALLBACK for types the desk genuinely lacks.
///
/// `None` (no workspace) yields a built-ins-only walker — still an improvement
/// over the two hard-coded hashes. Best-effort: an unreadable/malformed local
/// schema is skipped with a `warn!` (the rest still seed).
///
/// The corpus is `schema_cmd::resolution_schema_set` — the ONE
/// shared assembly + preflight every workspace-resolving surface uses. A
/// walker that assembled its own copy of the ladder with NO preflight would let
/// a hostile composed-overflow `.msg` PANIC
/// `FrameWalker::new` inside `topic echo` where `schema info` over the same
/// files degrades.
pub fn local_walker_from_workspace(
    schemas_dir: Option<&std::path::Path>,
) -> cerulion_core::codegen::FrameWalker {
    // Built-ins first, then the store, then workspace YAML — last wins on a
    // name collision (workspace-wins, mirroring the `schema info` ladder)
    // — the shared, preflighted set.
    let schemas = crate::schema_cmd::resolution_schema_set(schemas_dir);
    let (walker, warnings) = cerulion_core::codegen::FrameWalker::new(schemas);
    for w in warnings {
        tracing::debug!(warning = %w, "echo walker: FrameWalker resolution warning");
    }
    walker
}

/// The outcome of a remote schema fetch — the `schema info` analogue of
/// [`RemoteResolve`], and for the SAME reason: an empty answer means two completely
/// different things and only one of them licenses telling the user the type does not
/// exist.
///
/// A bare `Option<SchemaReply>` would make `netd_fetch_remote_schema`
/// DROP netd's discovery verdict, and `cerulion schema info` would re-raise its terminal
/// local "schema not found" for a cold daemon that had searched nothing — the exact
/// false-absence class the TOPIC path refuses, on the SCHEMA
/// path.
pub enum RemoteSchemaFetch {
    /// A robot served the type — its `robot` field is the provenance.
    Found(cerulion_core::SchemaReply),
    /// Discovery RAN (or was deliberately not attempted — the `CERULION_NETWORK=off`
    /// kill-switch) and no robot serves this type. A genuine absence: the caller may
    /// re-raise its local "not found".
    NotServed,
    /// NOTHING was read from the network — netd spent its cold-start grace
    /// without a robot answering, or the transient path could not search at all. The
    /// answer is UNKNOWN and must never be rendered as "the type does not exist".
    DiscoveryNotConverged,
    /// The user interrupted the first-contact wait — see
    /// [`RemoteResolve::Cancelled`].
    Cancelled,
}

/// Try to fetch `requested`'s schema closure through the shared
/// `cerulion-netd` query plane (production path only). Returns:
/// - `Some(answer)` — netd ANSWERED (with a served type, a settled "nobody has it",
///   or its "discovery never converged"); the caller uses it, NO
///   transient fallback.
/// - `None` — netd not applicable (explicit locators / non-Unix) OR UNREACHABLE (a
///   LOUD warn is emitted here); the caller falls back to a transient session.
fn try_fetch_remote_schema_via_netd(
    requested: &str,
    connect: &[String],
    listen: &[String],
    scouting: bool,
    wait: ResolveWait<'_>,
) -> Option<RemoteSchemaFetch> {
    if !use_netd_query_plane(connect, listen, scouting) {
        return None;
    }
    netd_fetch_remote_schema(requested, wait)
}

/// Unix arm: the netd-backed schema fetch. GET `requested` from EVERY
/// announcing robot over netd's shared session, first-with-docs wins. Loud-warns +
/// returns `None` on a netd-unreachable client error (fall back to a transient
/// session); an authoritative answer is `Some(RemoteSchemaFetch)`.
///
/// Routes through the discovery-reporting schema query, so a docs-less
/// gather from a daemon that never completed a discovery pass is reported as
/// [`RemoteSchemaFetch::DiscoveryNotConverged`], not as absence.
///
/// It also WAITS through convergence first
/// ([`NetdClient::query_schema_converged`](cerulion_netd::NetdClient::query_schema_converged)) —
/// a cold desk's daemon is typically seconds away from seeing the robot that is
/// right there, and `cerulion schema info <type>` should not be the command that
/// tells the user to go and retry by hand.
#[cfg(unix)]
fn netd_fetch_remote_schema(requested: &str, wait: ResolveWait<'_>) -> Option<RemoteSchemaFetch> {
    let mut client = match cerulion_netd::NetdClient::connect_or_spawn() {
        Ok(c) => c,
        Err(e) => {
            tracing::warn!(requested = %requested, error = %e,
                "cerulion-netd unreachable for the remote schema fetch — falling back to a \
                 transient discovery session");
            return None;
        }
    };
    let mut sink = emit_convergence_progress;
    let mut ctx = cerulion_netd::FirstContactWait::new(wait.policy, &mut sink, wait.running);
    match client.query_schema_converged(None, requested, &mut ctx) {
        Ok(converged) => {
            note_convergence_wait_outcome(converged.outcome, converged.progress_lines);
            // A wait the user interrupted supports no conclusion. Its
            // (empty, NotConverged) answer is indistinguishable from a completed one,
            // so classifying it would render Ctrl-C as an absence-shaped verdict.
            if converged.outcome == cerulion_netd::WaitOutcome::Cancelled {
                return Some(RemoteSchemaFetch::Cancelled);
            }
            let gather = converged.answer;
            log_schema_refusals(&gather.replies);
            match gather.replies.into_iter().find(|r| !r.docs.is_empty()) {
                Some(reply) => Some(RemoteSchemaFetch::Found(reply)),
                None => Some(classify_unserved_schema(requested, gather.discovery)),
            }
        }
        Err(abort) => {
            note_convergence_wait_abandoned(&abort);
            tracing::warn!(requested = %requested, error = %abort.error,
                "cerulion-netd schema query failed — falling back to a transient discovery session");
            None
        }
    }
}

/// The first-contact wait policy the desk observers use — the SHIPPED
/// constants ([`cerulion_netd::FIRST_CONTACT_CONVERGENCE_CEILING`] /
/// [`cerulion_netd::CONVERGENCE_POLL_INTERVAL`]).
///
/// One function so `topic echo`/`info`/`hz` and `schema info` cannot end up waiting
/// for different lengths of time on the same cold daemon.
fn first_contact_wait() -> cerulion_netd::ConvergenceWait {
    cerulion_netd::ConvergenceWait::default()
}

/// The wait POSTURE a remote resolve runs under, plus the caller's
/// cancellation source.
///
/// Threaded through the resolve chain because the SAME machinery serves two call
/// sites with opposite needs, and defaulting both to "wait" is
/// wrong for one of them:
///
/// * [`Self::first_contact`] — the caller will render an ABSENCE CLAIM if this comes
///   back empty (`ensure_topic_available` → the UNKNOWN message; `schema info` → the
///   same). Waiting through convergence is the whole point.
/// * [`Self::no_wait`] — the caller makes NO claim: `topic echo`'s local-topic
///   fallback resolves a remote walker purely so an unknown-type frame COULD be
///   decoded, and collapses every failure to `None` + hex. Making a LOCAL topic's
///   first frame ten seconds late — and printing "nothing on the network answered"
///   before it — to fetch a decoder it will probably never use is a pure loss.
///
/// `running` is the CLI's `Arc<AtomicBool>`, cleared by the SIGINT/SIGTERM/SIGHUP
/// handler `topic echo`/`topic hz` install, so a wait stays interruptible. `None` is
/// a caller with no handler installed (`topic info`, `schema info`), which still
/// dies on the default signal disposition.
#[derive(Clone, Copy)]
pub struct ResolveWait<'a> {
    policy: cerulion_netd::ConvergenceWait,
    running: Option<&'a std::sync::atomic::AtomicBool>,
}

impl<'a> ResolveWait<'a> {
    /// The WAITING posture, for a caller that will render an absence claim.
    pub fn first_contact(running: Option<&'a std::sync::atomic::AtomicBool>) -> Self {
        Self {
            policy: first_contact_wait(),
            running,
        }
    }

    /// The NON-waiting posture, for a caller that renders no claim and degrades
    /// silently. See the type docs.
    pub fn no_wait() -> Self {
        Self {
            policy: cerulion_netd::ConvergenceWait::off(),
            running: None,
        }
    }
}

/// The live progress line, on **STDERR**.
///
/// stderr is not a style choice: `topic echo`'s frames and `topic hz`'s rate lines
/// are STDOUT, and a Studio-class parser reading that stream must not have a
/// progress spinner spliced into it. stderr is also where a redirected
/// `cerulion topic hz ... > out.txt` user still sees it.
///
/// # `#[cfg(unix)]`, matching its only callers
///
/// The whole convergence-reporting group is Unix-gated because its only production
/// callers (`netd_fetch_remote_schema`, `netd_resolve_topic_schema`) are — otherwise
/// a non-Unix build trips `dead_code = "deny"`. Scope: this gate does not by itself
/// make a Windows build work. `cerulion_netd` uses
/// `std::os::unix` unconditionally with no cfg in its `lib.rs`, and this crate
/// depends on it unconditionally, so the crate does not build on non-Unix for a much
/// earlier reason. This is consistency with `note_convergence_wait_abandoned` (which
/// was already gated), not a live fix.
///
/// It fires once before the first round trip (only when a wait is actually possible
/// — a [`ResolveWait::no_wait`] caller prints NOTHING) and once per poll thereafter,
/// so a warm desk sees at most one line and a cold one sees a live counter.
#[cfg(unix)]
fn emit_convergence_progress(elapsed: std::time::Duration) {
    write_convergence_line(&mut std::io::stderr(), &convergence_progress_line(elapsed));
}

/// The ONE place a convergence line reaches a stream.
///
/// Every emitter routes through it with `std::io::stderr()` so the stdout/stderr
/// split is decided in a single spot a test can point at, rather than re-decided at
/// each `eprintln!`. A write failure is deliberately ignored: this is a progress
/// breadcrumb, and failing a `topic echo` because its spinner could not be written
/// would be strictly worse than losing the spinner.
#[cfg(unix)]
fn write_convergence_line(out: &mut dyn std::io::Write, line: &str) {
    let _ = writeln!(out, "{line}");
}

/// The PURE text of the progress line — oracle-tested.
///
/// ONE DECIMAL, matching the give-up line. Whole seconds made the first two polls
/// both render `(0s)` on a warm daemon, so the counter looked STALLED at exactly the
/// moment the user is deciding whether the command is hung — the opposite of what a
/// progress line is for.
#[cfg(unix)]
fn convergence_progress_line(elapsed: std::time::Duration) -> String {
    format!(
        "discovering robots on the network… ({:.1}s)",
        elapsed.as_secs_f64()
    )
}

/// Report the END of a first-contact wait that never converged — on stderr,
/// immediately before the caller raises the UNKNOWN message.
///
/// Without it the user watches a counter tick for ten seconds and then reads an error
/// that makes no reference to the wait, which reads as if the command gave up
/// instantly.
///
/// It is gated on the loop's OWN terminal decision and on whether any line was
/// actually printed — never on a duration or a discovery marker. Re-deriving it from
/// those two proxies is wrong: under the trust gate an
/// older daemon reports `NotConverged` for EVERY answer, so a wait that polled and
/// then FOUND the topic would satisfy both proxies and print "nothing on the network
/// answered" immediately before the command streamed its frames.
#[cfg(unix)]
fn note_convergence_wait_outcome(outcome: cerulion_netd::WaitOutcome, progress_lines: u32) {
    if should_note_give_up(outcome, progress_lines) {
        write_convergence_line(&mut std::io::stderr(), &convergence_gave_up_line());
    }
}

/// The PURE predicate [`note_convergence_wait_outcome`] guards on —
/// oracle-tested. Split out so the test drives the SHIPPED decision rather than a
/// transliteration of it (a test carrying its own copy of a one-line condition
/// proves nothing about the caller).
///
/// `progress_lines == 0` means the user saw NOTHING, so there is nothing to close:
/// that is the [`ResolveWait::no_wait`] posture, where an epitaph would be the only
/// output a silent fallback ever produced.
#[cfg(unix)]
fn should_note_give_up(outcome: cerulion_netd::WaitOutcome, progress_lines: u32) -> bool {
    matches!(outcome, cerulion_netd::WaitOutcome::GaveUp) && progress_lines > 0
}

/// The PURE text of the give-up line — oracle-tested.
///
/// It quotes NO duration. The last progress line already stated the elapsed, and the
/// loop's own bound is "stop starting polls at the ceiling" — so a figure printed
/// here would be one in-flight round trip larger than the ceiling the docs name, i.e.
/// a number that invites exactly the wrong arithmetic.
#[cfg(unix)]
fn convergence_gave_up_line() -> String {
    "…giving up on discovery — nothing on the network answered".to_string()
}

/// Report a first-contact wait ABANDONED by a transport error.
///
/// The consumer then degrades to a transient session (a loud `warn!`), but that warn
/// is differently shaped and says nothing about the counter the user has been
/// watching — so without this line the progress lines are simply never closed. Fires
/// only if any were printed.
#[cfg(unix)]
fn note_convergence_wait_abandoned(abort: &cerulion_netd::ConvergenceAbort) {
    if abort.progress_lines > 0 {
        write_convergence_line(
            &mut std::io::stderr(),
            "…stopping the discovery wait — the cerulion-netd query failed; \
             retrying without it",
        );
    }
}

/// No robot served `requested` — decide whether that is a genuine
/// ABSENCE or a cold start that proves nothing. The schema-fetch twin of
/// [`classify_unmatched_catalog`], with the same rule and the same reasoning: only
/// a daemon that has COMPLETED a discovery pass can license an absence claim.
///
/// PURE — oracle-tested. `requested` is used only for the diagnostic breadcrumb.
#[cfg(unix)]
fn classify_unserved_schema(
    requested: &str,
    discovery: cerulion_netd::DiscoveryState,
) -> RemoteSchemaFetch {
    match discovery {
        cerulion_netd::DiscoveryState::Settled => RemoteSchemaFetch::NotServed,
        cerulion_netd::DiscoveryState::NotConverged => {
            tracing::debug!(
                requested = %requested,
                "cerulion-netd has not completed a discovery pass — no robot has \
                 answered it yet, so 'nobody serves this type' is NOT a supported claim"
            );
            RemoteSchemaFetch::DiscoveryNotConverged
        }
    }
}

/// Non-Unix stub: the netd query plane is Unix-only, so always fall back.
#[cfg(not(unix))]
fn netd_fetch_remote_schema(_requested: &str, _wait: ResolveWait<'_>) -> Option<RemoteSchemaFetch> {
    None
}

/// Log — at DEBUG — any serve-side error in the gathered schema replies
/// (a docs-empty `SchemaReply.error`: either a genuine NOT-FOUND, or a
/// `DemandAuthorizer` refusal). This is deliberately QUIET (not a `warn!`, unlike the
/// CATALOG sibling [`warn_on_catalog_refusals`], which STAYS loud): a schema refusal
/// is WIRE-IDENTICAL to a genuine not-found (both are docs-empty + `error: Some`), so
/// the desk cannot tell "you're not authorized" from "the robot simply lacks that
/// type" — warning on every such reply would be a false "refused!" cry on the common
/// benign case. The accepted judgment call is to keep this at `debug!` (visible under
/// `RUST_LOG=debug` for diagnosis) and let the caller's own precise error carry the
/// user-facing reason. Named `log_` (not `warn_`) to match the level it emits.
fn log_schema_refusals(replies: &[cerulion_core::SchemaReply]) {
    for reply in replies {
        if reply.docs.is_empty() {
            if let Some(reason) = &reply.error {
                tracing::debug!(
                    robot = %sanitize_display(&reply.robot),
                    requested = %reply.requested,
                    reason = %reason,
                    "cerulion-netd: robot did not serve the requested schema (not found or not \
                     authorized)"
                );
            }
        }
    }
}

/// Fetch the `.msg`/YAML closure of `requested` (a qualified
/// `pkg/Type` OR a package-less bare `Name`) from ANY
/// discoverable robot — the desk-side of the `schema` verb. Opens a bounded
/// discovery session over `opts`, harvests robot identities from the ANNOUNCE
/// space (the proven direction — explicit-robot GETs), and GETs the type from
/// each concurrently (`discovery::query_robot_schemas`). Returns the FIRST reply carrying
/// a non-empty `docs` (a robot that HAS the type — its `robot` field is the
/// provenance). Best-effort, like `topic list`'s remote half: nothing here is a hang
/// or a hard error — every failure degrades to a classified outcome, exit 0.
///
/// The PRODUCTION automagic path asks the shared `cerulion-netd` query
/// plane first (`try_fetch_remote_schema_via_netd`); it opens its OWN transient
/// discovery session only when netd is not applicable (explicit locators / a hermetic
/// test) or UNREACHABLE (a LOUD degrade).
///
/// # An empty answer is CLASSIFIED, never collapsed
///
/// This returned a bare `Option<SchemaReply>` before, which forced `cerulion schema
/// info` to render EVERY empty answer as its terminal local "schema not found" — a
/// cold daemon that had searched nothing included. Both halves now obey the same rule
/// the topic path already obeys: **only claim absence about a network you actually
/// read**. The netd half carries the daemon's own verdict; the transient half derives
/// it locally (a session that would not open, a session that could not be used, a
/// failed announce harvest, or a harvest that found NO robot to ask are all
/// [`RemoteSchemaFetch::DiscoveryNotConverged`] — nothing was searched), and each
/// failure warns LOUDLY, never dropping the error on the floor.
///
/// The `CERULION_NETWORK=off` kill-switch stays [`RemoteSchemaFetch::NotServed`]: the
/// user asked for local-only, so re-raising the local "not found" is the correct
/// answer, not an "unknown".
/// What a transient schema fan-out's result means — with the
/// positive-answer search FIRST and the completeness question second.
///
/// PURE, and split out precisely so the ORDER is drivable: reaching this decision
/// through [`fetch_remote_schema`] needs a live zenoh session AND a GET worker
/// that unwinds, so with it inlined the ordering is reachable by no test at all.
///
/// # The order is the whole point
///
/// Asking "is this pass conclusive?" BEFORE looking at what came
/// back means a pass in which robot A's worker panicked and robot B RETURNED the
/// requested docs discards B's answer and reports
/// [`RemoteSchemaFetch::DiscoveryNotConverged`]. That inverts the demotion's own
/// purpose. The demotion exists to stop a pass making a confident claim of ABSENCE it
/// did not earn; it must never suppress a PRESENCE the desk is already holding.
///
/// A found document is EVIDENCE ON ITS OWN. A sibling worker's panic says nothing
/// about it: the doc is in hand, its provenance is the robot that served it, and
/// no robot this pass failed to ask could make it less true. Incompleteness bears
/// only on the NEGATIVE reading — "nobody has it" — which is a claim about the
/// whole set, and therefore a claim a pass that skipped part of the set may not
/// make.
///
/// That is the same shape the sibling consumers already have, and this site was
/// the one that broke it: `netd_fetch_remote_schema` searches `gather.replies`
/// first and only then consults `gather.discovery`, and
/// `resolve_remote_topic_schema_with_opts` runs its catalog `find_map` first and
/// only then feeds `classify_transient_miss`. netd's own plane keeps the data too
/// — its demotion returns `(gathered, NotConverged)`, downgrading the CLAIM and
/// never withholding the payload.
fn classify_schema_gather(
    requested: &str,
    announced: usize,
    gathered: cerulion_core::transport::discovery::GatherReplies<cerulion_core::SchemaReply>,
) -> RemoteSchemaFetch {
    // Computed BEFORE the search consumes `replies`, and consulted only after it.
    let read_nothing_conclusive = gathered.replies.is_empty() || !gathered.is_complete();
    let panicked = gathered.panicked.len();
    match gathered.replies.into_iter().find(|r| !r.docs.is_empty()) {
        // A robot HAS it. Serve it, whatever else this pass lost.
        Some(reply) => RemoteSchemaFetch::Found(reply),
        // Nothing usable came back, and this pass is in no position to say the
        // type is unavailable — either nobody answered, or a worker unwound and
        // some robot was never really asked.
        None if read_nothing_conclusive => {
            tracing::debug!(requested = %requested, announced, panicked,
                "robots ANNOUNCED but the fan-out read nothing usable \
                 from all of them — nothing conclusive was read, so this is NOT evidence \
                 the type is unavailable");
            RemoteSchemaFetch::DiscoveryNotConverged
        }
        // Every robot was asked and ANSWERED (a docs-less reply is a robot saying
        // "I do not have it"), and none has it — a genuine absence.
        None => RemoteSchemaFetch::NotServed,
    }
}

pub fn fetch_remote_schema(
    requested: &str,
    opts: &RemoteTopicsOptions,
) -> CliResult<RemoteSchemaFetch> {
    use cerulion_core::transport::discovery::{query_announce_entries, query_robot_schemas};
    // The `CERULION_NETWORK=off` kill-switch is LOCAL-ONLY
    // EVERYWHERE, by contract. Consult the SHARED resolution
    // (`graph_cmd::remote_network_suppressed` → `network_env_kill`, the one
    // env-parse `graph run` uses) BEFORE opening any discovery session, so a
    // kill-switched desk never touches the network — the local-only
    // degrade (the caller re-raises its local `SchemaNotFound`).
    if crate::graph_cmd::remote_network_suppressed() {
        tracing::info!(
            requested = %requested,
            "remote schema fetch suppressed by the CERULION_NETWORK kill-switch \
             — LOCAL-ONLY (no remote attempt)"
        );
        // NOT a cold start: the user asked for local-only, so the local "not found"
        // IS the correct answer.
        return Ok(RemoteSchemaFetch::NotServed);
    }
    // Production path — fetch over the shared cerulion-netd query plane
    // (ONE zenoh session per computer). `Some(reply)` = netd answered authoritatively
    // (`Some(SchemaReply)` = a robot has the type; `None` = netd reached the LAN and
    // nobody served it). `None` = netd not applicable / unreachable ⇒ fall through to
    // a transient session (a LOUD degrade is emitted inside).
    if let Some(answer) = try_fetch_remote_schema_via_netd(
        requested,
        &opts.connect,
        &opts.listen,
        opts.scouting,
        // `schema info` RENDERS an absence claim on an empty answer, so it waits.
        // It installs no signal handler, hence no cancellation source.
        ResolveWait::first_contact(None),
    ) {
        return Ok(answer);
    }
    // A session we could not open searched NOTHING — reporting that as
    // "no robot serves this type" is a claim with no evidence behind it (and it is
    // reached exactly when netd is broken, which is when the desk is most likely to
    // get it wrong).
    let mgr = match open_discovery_session(&opts.connect, &opts.listen, opts.scouting) {
        Ok(mgr) => mgr,
        Err(e) => {
            tracing::warn!(requested = %requested, error = %e,
                "remote schema fetch: could not open a discovery session — NOTHING was \
                 searched on the network (this is not evidence the type is unavailable)");
            return Ok(RemoteSchemaFetch::DiscoveryNotConverged);
        }
    };
    let session = mgr
        .session()
        .expect("session already opened by open_discovery_session");
    // Harvest robot identities from the announce space, then GET the type from
    // each. First found (non-empty docs) wins; a NOT-FOUND reply (the robot
    // answered but lacks the type) does not.
    let outcome = match query_announce_entries(session, REMOTE_QUERY_GATHER_WINDOW) {
        Ok(entries) => {
            let robots: Vec<String> = entries
                .iter()
                .map(|(robot, _)| robot.as_str())
                .collect::<BTreeSet<&str>>()
                .into_iter()
                .map(str::to_string)
                .collect();
            // The evidence is what we READ, not who ANNOUNCED — the same
            // rule `classify_transient_miss` applies (it gates on `catalogs.is_empty()`,
            // the catalogs actually read). `query_robot_schemas` DROPS every robot that
            // misses its window, so an announced-but-silent LAN yields zero replies:
            // gating on `robots` alone called that a settled "nobody serves this type"
            // and rendered a terminal "Schema not found" from a gather that read
            // nothing. `robots.is_empty()` survives only as a cheap short-circuit (no
            // point issuing a fan-out GET to nobody).
            if robots.is_empty() {
                tracing::debug!(requested = %requested,
                    "the transient schema harvest found NO robot to ask — an empty \
                     result is NOT evidence this type is unavailable");
                RemoteSchemaFetch::DiscoveryNotConverged
            } else {
                classify_schema_gather(
                    requested,
                    robots.len(),
                    query_robot_schemas(session, &robots, requested, CATALOG_GATHER_WINDOW),
                )
            }
        }
        Err(e) => {
            tracing::warn!(requested = %requested, error = %e,
                "remote schema fetch: announce-space harvest failed — NOTHING was \
                 searched on the network (this is not evidence the type is unavailable)");
            RemoteSchemaFetch::DiscoveryNotConverged
        }
    };
    mgr.close();
    Ok(outcome)
}

/// ONE-TIME best-effort remote resolve of a LOCAL topic's schema
/// for `topic echo` — learn the topic's qualified name from a discovered robot's
/// CATALOG, then fetch that type's `.msg`/YAML closure and seed a
/// [`FrameWalker`](cerulion_core::codegen::FrameWalker). Returns
/// `Some((robot, walker))` on success — the desk can now decode a frame whose
/// schema hash it never compiled — or `None` (no robots / topic not catalogued /
/// nobody serves the type / any error). Best-effort, never a hang: the caller
/// degrades to hash-only hex on `None`. Runs ONCE before the echo loop (not per
/// frame).
///
/// # What it costs
///
/// `topic_echo` calls this UNCONDITIONALLY for every LOCAL topic, so it runs under
/// [`ResolveWait::no_wait`] — the first-contact wait belongs to callers that render
/// an absence claim, and this one renders none.
///
/// `no_wait` removes the RE-ASKING, not the round trip, so this is NOT sub-second:
/// an earlier revision claimed "sub-second is a contract here" and that was false. One round trip
/// against a cold daemon runs the daemon's whole cold-start grace (~3.75 s, capped by
/// the client's 5 s round-trip timeout), and a desk with no daemon running pays a
/// netd spawn-readiness wait first. So the BLOCKER's user symptom — a local topic's
/// first frame delayed — is reduced from ~10 s to ~4 s, not removed. What IS
/// contractual is the silence: `no_wait` prints no progress counter and no give-up
/// line, so a local `topic echo` produces no network chatter at all.
pub fn resolve_remote_walker_for_topic(
    topic: &str,
) -> Option<(String, cerulion_core::codegen::FrameWalker)> {
    // The production `topic echo` path: automagic scouting, no explicit locators.
    resolve_remote_walker_for_topic_with_opts(topic, &[], &[], true)
}

/// The locator-injectable core of [`resolve_remote_walker_for_topic`]
/// (the public one calls this with the automagic scouting opts). Split out so a
/// HERMETIC test can drive the fetch→seed→decode-seed
/// flow over a loopback locator with scouting OFF — the production path is the
/// scouting-on delegation above.
pub fn resolve_remote_walker_for_topic_with_opts(
    topic: &str,
    connect: &[String],
    listen: &[String],
    scouting: bool,
) -> Option<(String, cerulion_core::codegen::FrameWalker)> {
    // `None` schemas_dir — this is the LOCAL-topic unknown-frame
    // fallback (a frame whose type the echo body's local walker — built-ins ∪
    // workspace — already FAILED to decode), so a built-in never reaches here;
    // the local-first resolve returns None for the unknown custom type and the
    // wire fetch runs, as before. Threading the workspace dir here would be a
    // no-op for this path.
    match resolve_remote_topic_schema_with_opts(
        topic,
        connect,
        listen,
        scouting,
        None,
        // This seam makes NO absence claim (every failure
        // collapses to `None` below and the caller degrades to hex), and
        // `topic_echo` calls it UNCONDITIONALLY for every LOCAL topic — so
        // inheriting the first-contact wait stalled `cerulion topic echo
        // /my/local/topic` for the full ceiling, printed a counter, and closed
        // with "nothing on the network answered", all before the first frame of a
        // topic that was streaming in local SHM the whole time.
        ResolveWait::no_wait(),
    ) {
        SchemaResolve::Found { robot, walker, .. } => Some((robot, walker)),
        // This is the LOCAL-topic unknown-frame fallback: there is no absence claim to
        // make (the caller just degrades to hex), so the cold-start arm collapses
        // with the others into "no remote walker".
        SchemaResolve::NoProducer
        | SchemaResolve::DiscoveryNotConverged
        // A cancelled resolve degrades to hex exactly like the others —
        // this seam renders no claim, so it has nothing to say about the wait.
        | SchemaResolve::Cancelled
        | SchemaResolve::SchemaUnavailable { .. } => None,
    }
}

/// What a bounded remote resolve of a topic yields:
/// the robot that announced it, the qualified schema NAME from its catalog, the
/// wire `schema_hash` (passed to `cerulion-netd`'s `demand`, which validates every
/// re-injected frame against it), and a
/// [`FrameWalker`](cerulion_core::codegen::FrameWalker) seeded with the type's
/// fetched closure (for decoding once frames flow).
pub struct RemoteIngressTarget {
    /// The robot (announce identity) that catalogs + serves this topic's type.
    pub robot: String,
    /// The qualified schema name (`pkg/Type` or a bare `Name`).
    pub schema_name: String,
    /// The wire `schema_hash` the seeded walker computes for `schema_name`.
    pub schema_hash: u64,
    /// A walker seeded with the type's fetched `.msg`/YAML closure.
    pub walker: cerulion_core::codegen::FrameWalker,
}

/// The DISTINGUISHABLE outcome of a remote ingress
/// resolve. A topic NO robot catalogs is genuinely "not found anywhere"; a topic
/// a robot DID catalog but whose schema it could not serve/resolve is a
/// different, actionable failure (name the robot + the cause) — collapsing both
/// to `None` produced the misleading "not found on any discovered robot".
pub enum RemoteResolve {
    /// A robot catalogs the topic AND serves a resolvable schema — ready to
    /// ingress.
    Found(RemoteIngressTarget),
    /// No discovered robot catalogs this topic (also: the `CERULION_NETWORK=off`
    /// kill-switch, where the user asked for local-only). DISCOVERY RAN — a genuine
    /// absence.
    ///
    /// Session/harvest FAILURES never land here — they searched nothing,
    /// so they route to [`RemoteResolve::DiscoveryNotConverged`].
    NoProducer,
    /// Discovery never completed a pass — `cerulion-netd` spent its
    /// cold-start grace without a single robot answering, so NOTHING was searched.
    /// The caller must say THAT, not "the topic does not exist": the two are
    /// indistinguishable to a user and only one of them is a fact.
    DiscoveryNotConverged,
    /// The user INTERRUPTED the first-contact wait (Ctrl-C / SIGTERM).
    ///
    /// Distinct from every arm above because a cancelled observation supports NO
    /// conclusion at all: not absence, and not even the claim "we searched and read
    /// nothing". Collapsing it into `DiscoveryNotConverged` is what made a Ctrl-C
    /// render as a paragraph asserting the topic's existence was UNKNOWN and telling
    /// the user to check the robot's power switch.
    Cancelled,
    /// A robot catalogs the topic, but its schema could not be resolved (not
    /// served / an empty reply / no wire layout). `cause` states why.
    SchemaUnavailable { robot: String, cause: String },
}

/// Production remote resolve of `topic`'s ingress target —
/// automagic scouting, no explicit locators. See [`RemoteResolve`].
///
/// `schemas_dir` (when in a workspace) feeds the LOCAL-FIRST schema
/// resolution — a catalog-named BUILT-IN (or workspace) type resolves from the
/// desk's own corpus with NO wire fetch (the wire fetch stays the fallback for a
/// custom type the desk lacks). `None` outside a workspace still resolves
/// built-ins.
pub fn resolve_remote_ingress_target(
    topic: &str,
    schemas_dir: Option<&std::path::Path>,
    wait: ResolveWait<'_>,
) -> RemoteResolve {
    resolve_remote_ingress_target_with_opts_and_wait(topic, &[], &[], true, schemas_dir, wait)
}

/// The locator-injectable core of [`resolve_remote_ingress_target`]
/// (HERMETIC tests drive it over a loopback locator, scouting off). Reuses the
/// SAME discovery as the echo walker path, then computes the wire `schema_hash`
/// the seeded walker would stamp for the resolved name — the value `cerulion-netd`
/// validates each re-injected frame against.
pub fn resolve_remote_ingress_target_with_opts(
    topic: &str,
    connect: &[String],
    listen: &[String],
    scouting: bool,
    schemas_dir: Option<&std::path::Path>,
) -> RemoteResolve {
    // Correction: an earlier comment here claimed the posture was
    // "inert on this signature's own path because explicit locators bypass netd".
    // That is false of the SIGNATURE — `use_netd_query_plane(&[], &[], true)` is
    // TRUE (its own unit test asserts so), and this fn is `pub`, so a
    // scouting-on caller DOES take the netd path and pays the wait. It is only
    // accidentally true of today's ARGUMENTS: every current caller is a test passing
    // `scouting = false`. The WAITING posture is therefore the deliberate default
    // for this entry — it renders an absence claim — with no cancellation source,
    // which is what a caller wanting one must use `..._and_wait` for.
    resolve_remote_ingress_target_with_opts_and_wait(
        topic,
        connect,
        listen,
        scouting,
        schemas_dir,
        ResolveWait::first_contact(None),
    )
}

/// [`resolve_remote_ingress_target_with_opts`] with an explicit wait
/// posture — the production entry threads the caller's cancellation flag through it.
pub fn resolve_remote_ingress_target_with_opts_and_wait(
    topic: &str,
    connect: &[String],
    listen: &[String],
    scouting: bool,
    schemas_dir: Option<&std::path::Path>,
    wait: ResolveWait<'_>,
) -> RemoteResolve {
    match resolve_remote_topic_schema_with_opts(topic, connect, listen, scouting, schemas_dir, wait)
    {
        SchemaResolve::NoProducer => RemoteResolve::NoProducer,
        // Carried through DISTINCT — collapsing it into `NoProducer` here
        // would re-create the exact false-absence claim this issue removed.
        SchemaResolve::DiscoveryNotConverged => RemoteResolve::DiscoveryNotConverged,
        SchemaResolve::Cancelled => RemoteResolve::Cancelled,
        SchemaResolve::SchemaUnavailable { robot, cause } => {
            RemoteResolve::SchemaUnavailable { robot, cause }
        }
        SchemaResolve::Found {
            robot,
            qualified,
            walker,
        } => {
            // The seeded walker was built from THIS type's docs, so it normally
            // resolves the qualified name to a layout+hash (the same hash the
            // robot stamps on the wire — the vizd `attach_remote` precedent). A
            // `None` means the served closure did not actually contain a usable
            // layout for the named type: a DISTINCT, actionable failure, not
            // "not found anywhere".
            match walker.schema_hash_for(&qualified) {
                Some(schema_hash) => RemoteResolve::Found(RemoteIngressTarget {
                    robot,
                    schema_name: qualified,
                    schema_hash,
                    walker,
                }),
                None => RemoteResolve::SchemaUnavailable {
                    robot,
                    cause: format!(
                        "the served schema '{qualified}' could not be resolved to a wire \
                         layout (unparseable or incomplete closure)"
                    ),
                },
            }
        }
    }
}

/// The DISTINGUISHABLE inner outcome of the shared
/// bounded remote resolve — learn `topic`'s qualified schema name from a
/// discovered robot's CATALOG, fetch that type's `.msg`/YAML closure, and seed a
/// [`FrameWalker`](cerulion_core::codegen::FrameWalker).
enum SchemaResolve {
    /// A robot catalogs the topic + served a non-empty schema closure.
    Found {
        robot: String,
        qualified: String,
        walker: cerulion_core::codegen::FrameWalker,
    },
    /// No robot catalogs the topic (also: the `CERULION_NETWORK=off` kill-switch).
    /// DISCOVERY RAN — this is a genuine absence claim.
    ///
    /// Session/harvest FAILURES never land here — they searched nothing,
    /// so they route to [`SchemaResolve::DiscoveryNotConverged`].
    NoProducer,
    /// Discovery has NOT completed a pass — `cerulion-netd` has never seen
    /// a robot answer, so nothing at all was searched. Distinct from
    /// [`SchemaResolve::NoProducer`]: an empty answer here proves NOTHING about the
    /// topic, and must never be rendered as "it does not exist".
    DiscoveryNotConverged,
    /// The user interrupted the first-contact wait — see
    /// [`RemoteResolve::Cancelled`].
    Cancelled,
    /// A robot catalogs the topic but did not serve its schema.
    SchemaUnavailable { robot: String, cause: String },
}

/// Whether to route this catalog/schema query through the shared
/// `cerulion-netd` query plane (the ONE zenoh session per computer). The PRODUCTION
/// automagic path — no explicit `--connect`/`--listen` locators and scouting ON —
/// uses netd. Explicit locators OR scouting-off (a hermetic test, or a per-invocation
/// `--connect` locator netd's FIXED session cannot honor) bypass netd and open a
/// transient session directly. PURE — oracle-tested.
fn use_netd_query_plane(connect: &[String], listen: &[String], scouting: bool) -> bool {
    connect.is_empty() && listen.is_empty() && scouting
}

/// LOUDLY surface any serve-side REFUSAL in the gathered catalogs (a
/// queried robot's `DemandAuthorizer` denied its catalog — `CatalogReply.error`).
/// The rule "a denied query renders the refusal, never an empty result" applies:
/// the refusal is warned (not silently dropped as an empty catalog).
fn warn_on_catalog_refusals(catalogs: &[cerulion_core::CatalogReply]) {
    for cat in catalogs {
        if let Some(reason) = &cat.error {
            tracing::warn!(
                robot = %sanitize_display(&cat.robot),
                reason = %reason,
                "cerulion-netd: robot refused to serve its topic catalog (not authorized) — its \
                 topics will not resolve"
            );
        }
    }
}

/// Try to resolve `topic`'s ingress target through the shared
/// `cerulion-netd` query plane (production path only). Returns:
/// - `Some(resolve)` — netd ANSWERED authoritatively (a found target, a
///   catalog-but-no-schema `SchemaUnavailable`, or `NoProducer` = netd reached the
///   LAN and no robot catalogs `topic`); the caller uses it, NO transient fallback.
/// - `None` — netd is not applicable (explicit locators / non-Unix) OR UNREACHABLE
///   (spawn/connect/transport failure — a LOUD warn is emitted here); the caller
///   falls back to a transient discovery session.
fn try_resolve_remote_topic_schema_via_netd(
    topic: &str,
    connect: &[String],
    listen: &[String],
    scouting: bool,
    schemas_dir: Option<&std::path::Path>,
    wait: ResolveWait<'_>,
) -> Option<SchemaResolve> {
    if !use_netd_query_plane(connect, listen, scouting) {
        return None;
    }
    netd_resolve_topic_schema(topic, schemas_dir, wait)
}

/// Unix arm: the netd-backed topic-schema resolve. ONE `NetdClient`
/// connection runs both the catalog gather and the schema fetch over netd's shared
/// session. A netd-unreachable / mid-query client error LOUD-warns + returns `None`
/// (fall back to a transient session); an authoritative answer returns
/// `Some(SchemaResolve)`.
#[cfg(unix)]
fn netd_resolve_topic_schema(
    topic: &str,
    schemas_dir: Option<&std::path::Path>,
    wait: ResolveWait<'_>,
) -> Option<SchemaResolve> {
    let mut client = match cerulion_netd::NetdClient::connect_or_spawn() {
        Ok(c) => c,
        Err(e) => {
            tracing::warn!(topic = %topic, error = %e,
                "cerulion-netd unreachable for the remote catalog query — falling back to a \
                 transient discovery session");
            return None;
        }
    };
    // Ask for the DISCOVERY state alongside the catalogs — the ability to
    // tell "the LAN was searched and nobody has it" from "netd never saw a robot at
    // all", which must never collapse into one terminal "not found".
    //
    // The wait below acts on it. "Nothing to retry,
    // because netd has already spent its own cold-start grace" is true of the DAEMON's
    // budget, and the wrong conclusion: that budget is bounded by the control-seam
    // round-trip timeout and lands BELOW real-LAN convergence (measured ~3-13 s on a
    // real robot LAN), so a single invocation loses the race where a hand-typed retry
    // ~11 s later resolves. Re-asking is exactly what is needed; it just has
    // to happen HERE, where each round trip carries its own timeout, rather than
    // inside one.
    let gather = {
        let mut sink = emit_convergence_progress;
        let mut ctx = cerulion_netd::FirstContactWait::new(wait.policy, &mut sink, wait.running);
        match client.query_catalog_converged(None, &mut ctx) {
            Ok(converged) => {
                note_convergence_wait_outcome(converged.outcome, converged.progress_lines);
                // See the schema twin — a cancelled observation must
                // never reach the absence classifier.
                if converged.outcome == cerulion_netd::WaitOutcome::Cancelled {
                    return Some(SchemaResolve::Cancelled);
                }
                converged.answer
            }
            Err(abort) => {
                note_convergence_wait_abandoned(&abort);
                tracing::warn!(topic = %topic, error = %abort.error,
                    "cerulion-netd catalog query failed — falling back to a transient discovery \
                     session");
                return None;
            }
        }
    };
    let catalogs = gather.catalogs;
    warn_on_catalog_refusals(&catalogs);
    // Find the catalog entry for THIS topic that carries a qualified schema name.
    let target = catalogs.iter().find_map(|cat| {
        cat.entries.iter().find_map(|e| {
            if e.topic == topic {
                e.schema_name
                    .as_ref()
                    .map(|n| (cat.robot.clone(), n.clone()))
            } else {
                None
            }
        })
    });
    let (robot, qualified) = match target {
        Some(t) => t,
        // Nothing matched. WHICH kind of nothing depends on whether netd had
        // completed a discovery pass — see `classify_unmatched_catalog`.
        None => return Some(classify_unmatched_catalog(topic, gather.discovery)),
    };
    // LOCAL-FIRST: a catalog-named built-in / workspace type resolves from the
    // desk's own corpus with NO schema fetch (the wire fetch stays the fallback for a
    // custom type the desk lacks).
    let local = local_walker_from_workspace(schemas_dir);
    if local.schema_hash_for(&qualified).is_some() {
        tracing::debug!(topic = %topic, schema = %qualified,
            "resolved remote topic's type from the LOCAL corpus (via netd catalog) — no wire schema fetch");
        return Some(SchemaResolve::Found {
            robot,
            qualified,
            walker: local,
        });
    }
    // Fetch that type's closure from the SAME robot over netd's shared session.
    //
    // Deliberately does NOT wait for convergence. Reaching this line means
    // the catalog gather already ANSWERED and named `robot` as this topic's producer,
    // so discovery has demonstrably converged on this daemon; a docs-less reply here
    // is a robot that will not serve its schema, which is a different failure with a
    // different remedy (`SchemaUnavailable`, naming the robot) and no amount of
    // waiting fixes it.
    let replies = match client.query_schema(Some(&robot), &qualified) {
        Ok(replies) => replies,
        Err(e) => {
            tracing::warn!(topic = %topic, robot = %sanitize_display(&robot), error = %e,
                "cerulion-netd schema query failed — falling back to a transient discovery session");
            return None;
        }
    };
    log_schema_refusals(&replies);
    match replies.into_iter().find(|r| !r.docs.is_empty()) {
        Some(reply) => Some(SchemaResolve::Found {
            robot,
            qualified,
            walker: seed_framewalker(&reply.docs),
        }),
        None => Some(SchemaResolve::SchemaUnavailable {
            robot,
            cause: format!(
                "the robot catalogs '{topic}' as '{qualified}' but did not serve its schema \
                 (an older binary, or schema serving unavailable)"
            ),
        }),
    }
}

/// The TRANSIENT path's twin of [`classify_unmatched_catalog`] — nothing
/// matched, so decide whether that is a genuine ABSENCE or a "we read nothing".
///
/// The transient path has no daemon to carry a discovery state, so the evidence is
/// the gather itself: `catalogs_empty` means we successfully read NOBODY's catalog,
/// which licenses no absence claim; a non-empty gather means we read at least one and
/// it does not carry the topic. Same rule as the netd plane, derived locally.
///
/// PURE — oracle-tested. `topic` is used only for the diagnostic breadcrumb.
fn classify_transient_miss(topic: &str, catalogs_empty: bool) -> SchemaResolve {
    if catalogs_empty {
        tracing::debug!(
            topic = %topic,
            "the transient discovery gather read no robot's catalog — an empty \
             result is NOT evidence this topic is absent"
        );
        SchemaResolve::DiscoveryNotConverged
    } else {
        SchemaResolve::NoProducer
    }
}

/// A netd catalog gather that matched NO entry for `topic` — decide whether
/// that is a genuine ABSENCE or a cold start that proves nothing.
///
/// `Settled` means netd has completed at least one discovery pass (some query on that
/// daemon really did gather an answer from the network), so nothing matching is a
/// fact about the LAN. `NotConverged` means netd spent its cold-start grace without
/// ever reading anything — either nothing answered the announce space, or a robot was
/// announced but never served its catalog. The topic was therefore never actually
/// searched for, and the earlier code's terminal "not found locally or on any
/// discovered robot" was a claim it had no evidence for. That claim also HEALED on the
/// next invocation (the daemon was warm by then), which is the worst shape: a
/// self-contradicting error that mis-educates the user about a topic that exists.
///
/// PURE — oracle-tested. `topic` is used only for the diagnostic breadcrumb.
#[cfg(unix)]
fn classify_unmatched_catalog(
    topic: &str,
    discovery: cerulion_netd::DiscoveryState,
) -> SchemaResolve {
    match discovery {
        cerulion_netd::DiscoveryState::Settled => SchemaResolve::NoProducer,
        cerulion_netd::DiscoveryState::NotConverged => {
            tracing::debug!(
                topic = %topic,
                "cerulion-netd has not completed a discovery pass — no robot has \
                 answered it yet, so an empty catalog is NOT evidence this topic is absent"
            );
            SchemaResolve::DiscoveryNotConverged
        }
    }
}

/// Non-Unix stub: the netd query plane is a UDS daemon, unavailable off
/// Unix — always fall back to the transient discovery session.
#[cfg(not(unix))]
fn netd_resolve_topic_schema(
    _topic: &str,
    _schemas_dir: Option<&std::path::Path>,
    _wait: ResolveWait<'_>,
) -> Option<SchemaResolve> {
    None
}

/// The shared bounded remote resolve. Best-
/// effort, never a hang, and bounded by its caller's [`ResolveWait`] — ONE netd
/// round trip under [`ResolveWait::no_wait`] (bounded by the client's round-trip
/// timeout, ~3.75 s against a cold daemon — NOT sub-second), up to the
/// first-contact ceiling under [`ResolveWait::first_contact`]. Distinguishes
/// "no robot catalogs it"
/// ([`SchemaResolve::NoProducer`]) from "a robot catalogs it but did not serve
/// its schema" ([`SchemaResolve::SchemaUnavailable`]).
///
/// The PRODUCTION automagic path first asks the shared `cerulion-netd`
/// query plane (`try_resolve_remote_topic_schema_via_netd`); it only opens its OWN
/// transient discovery session when netd is not applicable (explicit locators / a
/// hermetic test) or UNREACHABLE (a LOUD degrade, never a silent behavior change).
///
/// # The TRANSIENT half CLASSIFIES by the same rule, but carries no cold-start grace
///
/// Both halves now obey the same rule — **only claim absence about a catalog you
/// actually read**. The transient path derives that locally (`classify_transient_miss`
/// on whether the gather read anything) and reports
/// [`SchemaResolve::DiscoveryNotConverged`] for every "we read nothing" outcome,
/// including the three failure arms (session would not open, session unusable,
/// announce harvest errored). Returning `NoProducer` there with the error silently
/// DROPPED would render to the user as proof the topic does not exist. Those arms
/// warn LOUDLY and classify as non-convergence. That matters most exactly when netd
/// is broken, since a netd failure is what routes here.
///
/// What it does NOT get is the RETRY grace, deliberately, on two grounds:
///
/// 1. It is not the automagic path. It is reached with EXPLICIT `--connect`/`--listen`
///    locators — where the bounded-connect session connects to the named address
///    INLINE before the gather runs, so the session-establishment race the grace exists
///    for does not arise — or after a LOUD `cerulion-netd unreachable` warn, which
///    already tells the user the answer came from a degraded path.
/// 2. A transient session is opened and closed per call, so it has no cross-call
///    memory of having ever seen a robot. A grace here could never settle, and would
///    therefore tax EVERY call (including every hermetic explicit-locator test) with
///    the full budget forever — the exact "pay the ceiling on every query" shape the
///    netd plane's settled-bit exists to avoid.
///
/// Closing it properly means giving this path the discovery LADDER (mDNS + peer cache)
/// that `topic list` uses, which is a larger change than the classification fix was.
fn resolve_remote_topic_schema_with_opts(
    topic: &str,
    connect: &[String],
    listen: &[String],
    scouting: bool,
    schemas_dir: Option<&std::path::Path>,
    wait: ResolveWait<'_>,
) -> SchemaResolve {
    use cerulion_core::transport::discovery::{
        query_announce_entries, query_robot_catalogs, query_robot_schema,
    };
    // Honor the `CERULION_NETWORK=off` kill-switch (the
    // LOCAL-ONLY contract) BEFORE opening any discovery session — the
    // SAME shared resolution `graph run` uses. Suppressed ⇒ no remote walker
    // (echo degrades to hash-only hex + its local walker), never a session. (The
    // `ensure_topic_available` caller catches the kill-switch FIRST with a
    // dedicated message, so this arm is the defensive fallback for
    // the echo LOCAL-fallback walker path.)
    if crate::graph_cmd::remote_network_suppressed() {
        tracing::info!(
            topic = %topic,
            "remote schema resolve for `topic echo` suppressed by the CERULION_NETWORK \
             kill-switch — LOCAL-ONLY (no remote attempt)"
        );
        return SchemaResolve::NoProducer;
    }
    // Production path — resolve over the shared cerulion-netd query plane
    // (ONE zenoh session per computer). Only fall through to a transient session when
    // netd is not applicable (explicit locators / non-Unix) or UNREACHABLE (a loud
    // degrade is emitted inside).
    if let Some(resolved) = try_resolve_remote_topic_schema_via_netd(
        topic,
        connect,
        listen,
        scouting,
        schemas_dir,
        wait,
    ) {
        return resolved;
    }
    // These three failure arms must not `return SchemaResolve::NoProducer`
    // with the error DROPPED — that routes to "not found locally or on any
    // discovered robot", i.e. a session we could not even open would be reported to the
    // user as PROOF the topic does not exist. Nothing was searched in any of them, so
    // they report non-convergence (and say why, loudly). This needs no grace and adds
    // no wait — it is pure classification.
    let mgr = match open_discovery_session(connect, listen, scouting) {
        Ok(mgr) => mgr,
        Err(e) => {
            tracing::warn!(topic = %topic, error = %e,
                "could not open a transient discovery session — NOTHING was searched on the \
                 network (this is not evidence the topic is absent)");
            return SchemaResolve::DiscoveryNotConverged;
        }
    };
    let session = match mgr.session() {
        Ok(session) => session,
        Err(e) => {
            tracing::warn!(topic = %topic, error = %e,
                "the transient discovery session could not be used — NOTHING was searched on the \
                 network");
            return SchemaResolve::DiscoveryNotConverged;
        }
    };
    // 1. Harvest robots + their catalogs; find the entry for THIS topic that
    //    carries a qualified schema name.
    let robots: Vec<String> = match query_announce_entries(session, REMOTE_QUERY_GATHER_WINDOW) {
        Ok(entries) => entries
            .iter()
            .map(|(robot, _)| robot.as_str())
            .collect::<BTreeSet<&str>>()
            .into_iter()
            .map(str::to_string)
            .collect(),
        Err(e) => {
            mgr.close();
            tracing::warn!(topic = %topic, error = %e,
                "the announce harvest failed — NOTHING was searched on the network");
            return SchemaResolve::DiscoveryNotConverged;
        }
    };
    let gathered = query_robot_catalogs(session, &robots, CATALOG_GATHER_WINDOW);
    // A pass that lost a worker read fewer catalogs than it meant to, so
    // it may not license the absence `classify_transient_miss` would otherwise
    // draw — the same demotion the netd plane applies at its own seam.
    let read_nothing_conclusive = gathered.replies.is_empty() || !gathered.is_complete();
    let catalogs = gathered.replies;
    let target = catalogs.iter().find_map(|cat| {
        cat.entries.iter().find_map(|e| {
            if e.topic == topic {
                e.schema_name
                    .as_ref()
                    .map(|n| (cat.robot.clone(), n.clone()))
            } else {
                None
            }
        })
    });
    let (robot, qualified) = match target {
        Some(t) => t,
        None => {
            mgr.close();
            // Nothing matched — but WHICH nothing? `catalogs` is what we
            // actually READ, so an empty one means we read nobody's catalog and have
            // no basis for an absence claim (the same rule netd's plane applies). A
            // non-empty one means we read at least one robot's catalog and it does
            // not carry this topic — a genuine absence.
            return classify_transient_miss(topic, read_nothing_conclusive);
        }
    };
    // LOCAL-FIRST resolution (vizd `resolve_remote_type` parity). The
    // catalog named the type; if the desk ALREADY has it — a BUILT-IN
    // (`sensor_msgs/PointCloud2`, `nav_msgs/Odometry`, …) or a WORKSPACE schema —
    // resolve it from our OWN corpus with NO wire fetch. The wire `schema_hash` is
    // layout-derived + recipe-stable, so the desk's own type computes the same
    // hash the robot stamps; a robot that catalogs a built-in but does not SERVE
    // its `.msg` (its serving store holds only bridge-config customs, or it is an
    // older binary) now resolves here instead of a misleading `SchemaUnavailable`.
    // The wire fetch below stays the FALLBACK for a CUSTOM type the desk lacks.
    // `local_walker_from_workspace` folds built-ins ∪ store ∪ workspace YAML with
    // workspace-wins, so a workspace type shadowing a built-in
    // resolves to the workspace layout.
    let local = local_walker_from_workspace(schemas_dir);
    if local.schema_hash_for(&qualified).is_some() {
        mgr.close();
        tracing::debug!(
            topic = %topic,
            schema = %qualified,
            "resolved remote topic's type from the LOCAL corpus (built-in/workspace) — no wire schema fetch"
        );
        return SchemaResolve::Found {
            robot,
            qualified,
            walker: local,
        };
    }
    // 2. Fetch that type's closure from the SAME robot and seed the walker. The
    //    catalog name is the requested type verbatim — qualified `pkg/Type` OR a
    //    package-less bare `Name`, fed straight to the
    //    `schema` verb (no `pkg`/`ty` split — a bare name has no package half).
    let reply = query_robot_schema(session, &robot, &qualified, CATALOG_GATHER_WINDOW);
    mgr.close();
    // The robot DID catalog the topic — a missing / empty schema reply
    // is now a NAMED "schema unavailable", never "not found anywhere".
    match reply {
        Some(reply) if !reply.docs.is_empty() => SchemaResolve::Found {
            robot,
            qualified,
            walker: seed_framewalker(&reply.docs),
        },
        _ => SchemaResolve::SchemaUnavailable {
            robot,
            cause: format!(
                "the robot catalogs '{topic}' as '{qualified}' but did not serve its schema \
                 (an older binary, or schema serving unavailable)"
            ),
        },
    }
}

/// Open ONE bounded-connect discovery session over `connect`/`listen`
/// (the EPHEMERAL `topic list` session — the one place `bounded_connect` opts
/// in, so a black-hole locator can't stall the open past the 1 s bound). A
/// FRESH manager per call, so the open-failure fallback can retry with a
/// different connect set. Returns the manager with its session already opened
/// (cached in the `OnceCell`), or a loud `CliError::Validation` naming the
/// cause + the expected locator form. Never a silent empty.
fn open_discovery_session(
    connect: &[String],
    listen: &[String],
    scouting: bool,
) -> CliResult<cerulion_core::transport::network::NetworkManager> {
    use cerulion_core::transport::network::{NetworkConfig, NetworkManager};
    let config = NetworkConfig {
        connect_endpoints: connect.to_vec(),
        listen_endpoints: listen.to_vec(),
        // Scouting is caller-controlled — ON for the automagic CLI
        // path, OFF for hermetic tests (reaches exactly the named locators).
        multicast_scouting: scouting,
        gossip_scouting: scouting,
        bounded_connect: true,
        ..NetworkConfig::default()
    };
    let mgr = NetworkManager::new(config);
    mgr.session().map_err(|e| {
        CliError::Validation(format!(
            "failed to open the zenoh discovery session: {e} — check the \
             --connect/--listen locators (zenoh form: tcp/<host>:<port>, \
             e.g. tcp/192.168.123.99:7683)"
        ))
    })?;
    Ok(mgr)
}

/// The pure post-gather write-back decision —
/// `None` = do not touch the cache (scouting OFF, i.e. a hermetic/explicit
/// session, or nothing confirmed live), `Some(tuples)` = record exactly these
/// `(robot, Option<locator>)` pairs derived from the presence rows. Split out
/// so the guard protecting the developer's real `~/.cerulion/peers.json` from
/// hermetic tests is oracle-testable (the IO half is
/// [`crate::peer_cache::record_confirmed`], path-parameterized for the same
/// reason). Pure — oracle-tested below.
pub fn resolve_write_back(
    scouting: bool,
    robots: &[RobotRow],
) -> Option<Vec<(String, Option<String>)>> {
    if !scouting || robots.is_empty() {
        return None;
    }
    Some(
        robots
            .iter()
            .map(|r| (r.robot.clone(), r.locator.clone()))
            .collect(),
    )
}

/// Deterministic presentation order for discovered remote topics —
/// sorted + deduplicated. The liveliness query's reply order is
/// network-dependent, and the same canonical topic can be advertised by
/// more than one remote publisher (one token per publisher instance), so
/// the raw list is neither stable nor unique. Pure — oracle-tested below.
pub fn normalize_remote_topics(mut topics: Vec<String>) -> Vec<String> {
    topics.sort();
    topics.dedup();
    topics
}

/// Network-surface hardening: neutralize terminal
/// control characters in LAN-supplied strings before they reach the render
/// seam. mDNS TXT robot names, announce-key robot chunks, and
/// liveliness-key topic names are ALL attacker-controllable by any LAN peer;
/// interpolated verbatim they could inject ANSI/CSI escapes (`ESC[2J`
/// screen-clear, CR overwrite, BEL) into the operator's terminal. Every C0 control
/// (U+0000..=U+001F), DEL (U+007F), and C1 control (U+0080..=U+009F) is
/// replaced with the Unicode replacement char (U+FFFD); every other code
/// point — including non-ASCII robot names like "go2-α" — passes through
/// unchanged. Pure — oracle-tested below.
///
/// It is `pub(crate)` because `bag_cmd`'s coverage renderer now
/// prints a `remote_mirror`'s origin ROBOT and an `attach_failed`'s transport
/// message, both of which can carry LAN-supplied bytes by the same route. One
/// sanitizer, one set of oracle vectors — a second copy is how the two would
/// drift.
pub(crate) fn sanitize_display(s: &str) -> String {
    s.chars()
        .map(|c| match c {
            '\u{0000}'..='\u{001F}' | '\u{007F}' | '\u{0080}'..='\u{009F}' => '\u{FFFD}',
            other => other,
        })
        .collect()
}

/// Render the `ROBOTS` section of `topic list` — the presence-derived
/// [`RobotRow`]s ([`build_robot_rows`]) plus an UNVERIFIED-candidates line for
/// the non-mDNS ladder finds — or the empty string when there is neither.
/// Rendered BEFORE the `REMOTE TOPICS` section.
///
/// Each row renders `  <robot>[  <n> topic(s)][  @ <locator>]  (<provenance>)`:
/// the DISTINCT-topic count when the robot announced any, the mDNS locator when
/// known, and the provenance tag (`announce` for a live-token-only row, `mdns`
/// for a row the ladder's mDNS rung surfaced or enriched). An announce+mDNS merge
/// is ONE row (topic count AND locator, `(mdns)` tag). Rows are already sorted by
/// robot name upstream.
///
/// Non-mDNS ladder candidates (cache / hostname /
/// scan) are NOT robot rows — they carry no liveness evidence — but a `--scan`
/// user must still SEE the address an open port answered at (it was visible on
/// earlier builds). They render on ONE clearly-labeled line under the rows:
/// `  candidates (unverified): <robot> @ <locator> (<rung>) · ...` — mandatory
/// label, so they can never be mistaken for verified robots. Pure —
/// oracle-tested.
///
/// All externally-sourced strings (robot names, locators) are passed through
/// [`sanitize_display`] first — a hostile LAN peer cannot inject terminal
/// escapes into this output. Provenance/rung tags are internal (safe) and raw.
fn render_robots_section(robots: &[RobotRow], peers: &[DiscoveredPeer]) -> String {
    let candidates: Vec<&DiscoveredPeer> = peers
        .iter()
        .filter(|p| p.rung != DiscoveryRung::Mdns)
        .collect();
    if robots.is_empty() && candidates.is_empty() {
        return String::new();
    }
    let mut out = String::from("\nROBOTS\n");
    for row in robots {
        out.push_str("  ");
        out.push_str(&sanitize_display(&row.robot));
        if row.topic_count > 0 {
            out.push_str(&format!(
                "  {} topic{}",
                row.topic_count,
                if row.topic_count == 1 { "" } else { "s" }
            ));
        }
        if let Some(locator) = &row.locator {
            out.push_str(&format!("  @ {}", sanitize_display(locator)));
        }
        out.push_str(&format!("  ({})\n", row.provenance));
    }
    if !candidates.is_empty() {
        let items: Vec<String> = candidates
            .iter()
            .map(|p| {
                format!(
                    "{} @ {} ({})",
                    sanitize_display(&p.robot),
                    sanitize_display(&p.locator),
                    p.rung
                )
            })
            .collect();
        out.push_str(&format!(
            "  candidates (unverified): {}\n",
            items.join(" · ")
        ));
    }
    out
}

/// Render the `ROBOTS` + `REMOTE TOPICS` sections
/// of `topic list`.
///
/// Pure string rendering (the binary prints the returned string verbatim;
/// engine owns the logic, the binary stays a thin dispatch). The `ROBOTS`
/// section (presence-derived robot rows + the labeled unverified-candidates
/// line) is rendered first when non-empty; with no robots AND no candidates
/// discovered nothing precedes the topics.
///
/// With NO remote topic the `REMOTE TOPICS` header is not printed at all: the
/// section collapses to the ONE line of `render_remote_none_discovered`, so
/// a local-only desk (the quickstart) ends its listing with a single `remote:`
/// line instead of a header plus a paragraph; when a `ROBOTS` section did
/// render, a blank line separates it from that line. The line is EXPLICIT about the
/// BOUNDED gather: an empty result means nothing answered within the
/// [`REMOTE_QUERY_GATHER_WINDOW`], NOT a definitive "no robot exists". A query
/// FAILURE never reaches this renderer; it stays a distinct loud error at the
/// dispatch site.
pub fn render_remote_topics_section(disc: &RemoteDiscovery, had_endpoints: bool) -> String {
    let mut out = render_robots_section(&disc.robots, &disc.peers);
    let preceded = !out.is_empty();
    out.push_str(&render_remote_topics_body(disc, had_endpoints, preceded));
    out
}

/// The topics half of the listing, split out so a caller that has already
/// rendered rows of its own (the account directory) appends the same bytes.
/// `preceded` says whether anything was rendered above: it only controls the
/// blank line that keeps the none-discovered notice its own paragraph.
fn render_remote_topics_body(
    disc: &RemoteDiscovery,
    had_endpoints: bool,
    preceded: bool,
) -> String {
    let mut out = String::new();
    let topics = &disc.topics;
    if topics.is_empty() {
        // The "pass --connect tcp/<host>:7683" escape is
        // misleading once a peer is ALREADY reachable: either the user passed
        // a locator (`had_endpoints`), a presence row proved a live gateway
        // (announce/mDNS), OR the ladder's mDNS rung got a browse answer (an
        // intrinsically-live gateway). In those cases the accurate message is
        // "a reachable peer just didn't advertise a topic in time", not "go
        // find a peer". Cache/hostname/scan
        // candidates are not reachability evidence: a dead cached robot must
        // not suppress the --connect escape for 7 days of TTL.
        let reachable = had_endpoints
            || !disc.robots.is_empty()
            || disc.peers.iter().any(|p| p.rung == DiscoveryRung::Mdns);
        if preceded {
            // A ROBOTS section preceded: keep a blank line
            // so the notice reads as its own paragraph.
            out.push('\n');
        }
        out.push_str(&render_remote_none_discovered(reachable));
        return out;
    }
    out.push_str("\nREMOTE TOPICS\n");
    for t in topics {
        out.push_str(&sanitize_display(t));
        out.push('\n');
    }
    out
}

/// The ONE line `topic list` prints in place of the `REMOTE TOPICS` section
/// when the bounded gather found no remote topic. It names the window (so an
/// empty answer reads as "nobody answered in N ms", never "no robot exists")
/// and the escape that fits the evidence:
///
/// - `reachable == false` (no locator given, no robot row, no mDNS answer):
///   the `--connect tcp/<host>:7683` hint, for a robot scouting cannot reach
///   (7683 is the well-known permissive-gateway port).
/// - `reachable == true` (a given locator, a discovered robot, or an mDNS
///   browse answer): a peer IS reachable and simply advertised no topic within
///   the window (a busy peer, or no networked publisher alive yet; a
///   liveliness token exists only while one is), so the hint is `retry`, and
///   the `--connect` escape is deliberately absent: telling an operator whose
///   robot is already on screen to go find it is noise.
///
/// Pure; oracle-tested below.
fn render_remote_none_discovered(reachable: bool) -> String {
    let window_ms = REMOTE_QUERY_GATHER_WINDOW.as_millis();
    if reachable {
        format!(
            "remote: none discovered in {window_ms} ms (a reachable peer advertised no topic \
             in time; retry)\n"
        )
    } else {
        format!(
            "remote: none discovered in {window_ms} ms (a robot off the LAN needs \
             --connect tcp/<host>:7683)\n"
        )
    }
}

/// The ONE stderr line `topic list` prints when the remote query FAILED (a
/// zenoh session that would not open, a ladder error), as opposed to the
/// query succeeding and finding nothing (`render_remote_none_discovered`):
/// `remote: discovery unavailable (<error>; pass --no-network to skip it)`.
///
/// The remote half is best effort: the local topics already printed and the
/// verb still exits 0, so this line is the ONLY trace of the failure. It
/// carries the error's own text in full and names the flag that skips the
/// query.
///
/// It is rendered HERE, not formatted inline in the binary, for the reason
/// every other `topic list` line is: the engine renders, the binary prints the
/// returned string verbatim (`eprint!`, the newline is included), and an exact
/// oracle below pins the sentence `docs/networking.md` and
/// `docs/internals/cli.md` quote.
///
/// ONE line always: an error whose text spans several lines is folded onto one
/// (each line trimmed, blank lines dropped, joined with a space; no word is
/// lost), and any remaining terminal control character is neutralized by
/// `sanitize_display`, since the text can carry a LAN-supplied locator. Like
/// every non-topic line it never starts with `/`.
///
/// Pure; oracle-tested below.
pub fn render_remote_discovery_unavailable(error: &dyn std::fmt::Display) -> String {
    let text = error.to_string();
    let folded = text
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty())
        .collect::<Vec<_>>()
        .join(" ");
    format!(
        "remote: discovery unavailable ({}; pass --no-network to skip it)\n",
        sanitize_display(&folded)
    )
}

/// The mirror-aware `REMOTE TOPICS` renderer — folds LOCAL
/// mirror-streaming rows ([`MirrorStreamRow`], from the `/__cerulion/mirrors`
/// provenance registry) into the REMOTE section attributed to their origin robot
/// with a `● streaming` marker, alongside the network-discovered (idle-announce)
/// topics.
///
/// Three render states per topic (by design): LOCAL (a genuine local
/// topic — partitioned out BEFORE this renderer, in the binary), REMOTE·streaming
/// (in the registry + a live local mirror — a marker row here), REMOTE·idle (a
/// network `disc.topics` entry with no local mirror — a bare row, unchanged).
/// DEDUP: a topic that is BOTH network-announced AND locally mirrored renders
/// ONCE, as streaming (streaming wins).
///
/// With an EMPTY `streaming` slice this delegates to
/// [`render_remote_topics_section`] and is identical to it (including its
/// one-line `remote: none discovered` arms). Only when there ARE streaming rows
/// does it take the merged path: robots section (unchanged), then the union of
/// streaming (marker) + non-duplicate announced (bare) rows, sorted by canonical
/// topic (`BTreeMap` key order) for deterministic presentation. The topic PATH is
/// always the row's first whitespace-delimited token (Studio parser compat — the
/// marker + robot ride the trailing columns); every externally-sourced string
/// (topic, robot) is `sanitize_display`-neutered so a hostile LAN identity
/// cannot inject terminal escapes. Oracle-tested below.
pub fn render_remote_topics_section_with_mirrors(
    disc: &RemoteDiscovery,
    had_endpoints: bool,
    streaming: &[MirrorStreamRow],
) -> String {
    render_remote_topics_section_with_mirrors_and_robot_rows(disc, had_endpoints, streaming, "")
}

/// Append already-sanitized account-directory rows to ROBOTS while preserving
/// independent LAN evidence and the existing mirror/topic formatting.
/// The caller renders these rows with `account_robot_access::render_rows` on Unix.
pub fn render_remote_topics_section_with_mirrors_and_robot_rows(
    disc: &RemoteDiscovery,
    had_endpoints: bool,
    streaming: &[MirrorStreamRow],
    account_rows: &str,
) -> String {
    let mut out = render_robots_section(&disc.robots, &disc.peers);
    if !account_rows.is_empty() {
        if out.is_empty() {
            out.push_str("\nROBOTS\n");
        }
        out.push_str(account_rows);
    }
    if streaming.is_empty() {
        // No mirrors ⇒ identical to the mirror-less renderer (incl. its
        // one-line none-discovered arms).
        let preceded = !out.is_empty();
        out.push_str(&render_remote_topics_body(disc, had_endpoints, preceded));
        return out;
    }
    out.push_str("\nREMOTE TOPICS\n");
    // Union rows keyed by canonical topic: streaming (Some robot) wins over an
    // idle-announce (None) entry for the same topic (dedup — one row, streaming).
    let streaming_topics: BTreeSet<&str> = streaming.iter().map(|r| r.topic.as_str()).collect();
    let mut rows: BTreeMap<&str, Option<&str>> = BTreeMap::new();
    for t in &disc.topics {
        if !streaming_topics.contains(t.as_str()) {
            rows.entry(t.as_str()).or_insert(None);
        }
    }
    for r in streaming {
        rows.insert(r.topic.as_str(), Some(r.robot.as_str()));
    }
    for (topic, robot) in rows {
        out.push_str(&sanitize_display(topic));
        if let Some(robot) = robot {
            out.push_str("  ● streaming  ");
            out.push_str(&sanitize_display(robot));
        }
        out.push('\n');
    }
    out
}

/// Render independently observed account and mirror rows when LAN discovery
/// failed or was skipped. A missing gather supplies no empty-topic evidence.
pub fn render_remote_evidence_without_discovery(
    streaming: &[MirrorStreamRow],
    account_rows: &str,
) -> String {
    if !streaming.is_empty() {
        return render_remote_topics_section_with_mirrors_and_robot_rows(
            &RemoteDiscovery::empty(),
            false,
            streaming,
            account_rows,
        );
    }
    if account_rows.is_empty() {
        String::new()
    } else {
        format!("\nROBOTS\n{account_rows}")
    }
}

/// Idle "heartbeat" timeout for the event-driven observer loops
/// (`topic_echo` / `topic_hz`). Both block on the subscriber's iceoryx2
/// event listener via `AnySubscriber::wait_for_message` and wake the
/// instant a data event arrives (µs display latency, zero idle wakeups
/// while data flows).
///
/// The timeout bounds two things when the topic is idle:
/// 1. How often a silent loop re-checks `running` (so Ctrl+C is honored
///    within one heartbeat).
/// 2. The drain cadence for a NON-NOTIFYING publisher — a foreign
///    raw-iceoryx2 writer, or a future notify-elided topic, that puts
///    data on `{topic}/data` without ringing `{topic}/event`.
///    `wait_for_message` drains the queue on timeout as well as on a data
///    event (Principle #6: no data loss), so such a topic still displays
///    at heartbeat cadence rather than going silent.
///
/// ~1 idle wake/s, versus the previous busy poll loops (echo: 100/s via a
/// 10 ms sleep; hz: 1000/s via a 1 ms sleep).
const OBSERVER_HEARTBEAT: Duration = Duration::from_millis(1000);

/// The `topic hz` reporting cadence — emit a rate line every
/// second. `topic_hz` blocks for `min(OBSERVER_HEARTBEAT, time-until-next-
/// report)` so the report still ticks on a silent topic (the wait never
/// overshoots the deadline).
const HZ_REPORT_INTERVAL: Duration = Duration::from_secs(1);

/// A SIGINT can interrupt `topic_echo`/`topic_hz`'s blocking
/// `wait_for_message` call before the process's own Ctrl-C handler has
/// actually run and flipped `running` to false — observed as a real
/// scheduling race (not a single-check ordering guarantee: the interrupted
/// wait can return before whichever thread the signal landed on has gotten
/// scheduled to run the handler). The anti-poll rule forbids a poll-sleep retry in this
/// file's production region (a structural test below bans it), so this
/// bounds the retry by ATTEMPT COUNT instead of wall-clock: each retry
/// re-enters `wait_for_message`, which blocks properly on the listener
/// again (no busy-spin) — a shutdown
/// racing the handler resolves on the very next loop check with no new
/// signal to interrupt it further; a genuinely broken receive path keeps
/// failing and is reported after `MAX_CONSECUTIVE_WAIT_ERRORS` attempts.
/// The full decision table lives in [`decide_wait_outcome`].
const MAX_CONSECUTIVE_WAIT_ERRORS: u32 = 5;

/// The three possible dispositions of one `wait_for_message` outcome in an
/// observer loop (`topic_echo` / `topic_hz`). See [`decide_wait_outcome`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WaitOutcomeAction {
    /// Keep looping: either the wait succeeded (the retry budget resets),
    /// or it failed within the budget (re-enter the same blocking wait).
    Continue,
    /// `running` flipped false — exit the loop cleanly (the Ctrl-C
    /// contract: an interrupted wait during shutdown is not a failure).
    ExitCleanly,
    /// The wait kept failing through the whole retry budget with `running`
    /// still true — report a genuine receive failure.
    Fail,
}

/// The PURE decision at the heart of the SIGINT-race fix — one
/// `wait_for_message` outcome in, one loop action out, the
/// consecutive-error budget threaded through `&mut`. Extracted (crib:
/// `cerulion_core`'s `DrainWarnLatch`) so the retry/exit/fail contract is
/// oracle-vector unit-testable without a transport or a live signal race:
/// the subprocess e2e tests in `crates/cerulion_cli/tests/
/// topic_introspect_cli_e2e_test.rs` exercise the real signal path but can
/// only pin exit codes on whichever side of the race a given run lands;
/// the unit tests below pin the decision table deterministically. Both
/// observer loops route every wait outcome through here — under the
/// workspace's `dead_code = "deny"` lint, a regression that stops
/// consulting this function fails the BUILD, not just a test.
fn decide_wait_outcome(
    wait_ok: bool,
    running: bool,
    consecutive_wait_errors: &mut u32,
) -> WaitOutcomeAction {
    if wait_ok {
        *consecutive_wait_errors = 0;
        return WaitOutcomeAction::Continue;
    }
    if !running {
        return WaitOutcomeAction::ExitCleanly;
    }
    *consecutive_wait_errors += 1;
    if *consecutive_wait_errors < MAX_CONSECUTIVE_WAIT_ERRORS {
        WaitOutcomeAction::Continue
    } else {
        WaitOutcomeAction::Fail
    }
}

/// How long an observer must have gone WITHOUT displaying a frame
/// before it says so, and how often it repeats itself afterwards.
///
/// Without a notice, `topic hz` against a `cerulion-netd` mirror that
/// REGISTERED but over which no frame ever crossed can sit silent
/// indefinitely. The operator cannot tell "about to stream" from "the data
/// plane is dead": success-shaped silence over a dead cross-machine data plane.
/// The state "attached but nothing arriving" must be VISIBLE (the CLI twin of
/// the sidebar-truth decision), so both observer verbs report it, naming what is
/// actually known: the topic, where its frames are supposed to come from, how
/// many have arrived (zero), and over how long.
///
/// The first notice is deliberately SHORT (a few seconds — the issue's own
/// suggestion) so a dead route is named before an operator gives up; the repeat
/// cadence is long enough that a genuinely idle topic does not become its own
/// flood.
const OBSERVER_FIRST_SILENCE_NOTICE: Duration = Duration::from_secs(3);
const OBSERVER_SILENCE_NOTICE_INTERVAL: Duration = Duration::from_secs(5);

/// Is a silence notice DUE, given how long the observer has gone
/// without displaying a frame and how many notices it has already emitted?
///
/// PURE — oracle-tested (the boundary is pinned on BOTH sides). The schedule is
/// `FIRST`, then `FIRST + k*INTERVAL`: the first notice lands quickly so a dead
/// route is named early, and every later one at the slower repeat cadence.
/// `frames_displayed` gates the whole thing: a topic that has EVER delivered a
/// frame in the current silence window is not silent, and an observer that is
/// showing data must never also claim to be waiting for it.
fn silence_notice_due(silent_for: Duration, notices_emitted: u32) -> bool {
    let due = OBSERVER_FIRST_SILENCE_NOTICE
        .saturating_add(OBSERVER_SILENCE_NOTICE_INTERVAL.saturating_mul(notices_emitted));
    silent_for >= due
}

/// The CPU floor an observer loop pauses for when it detects
/// progress-free churn, and how many consecutive churning iterations it
/// tolerates first.
///
/// Both observer loops are event-driven, which removed the
/// UNCONDITIONAL poll sleep — on a healthy topic the loop blocks on the
/// listener and wakes ~once per second. What it could not remove is an EXTERNAL
/// event source: `wait_for_message` returns the moment an event arrives, and an
/// event that yields no DELIVERABLE frame (a non-data event, or frames the wire
/// filter drops) sends the loop straight back around with its budget unspent.
/// Under a fast enough event source that is an unbounded tight loop — the
/// measured symptom, ~100% of a core with nothing to show for it.
///
/// So the pace is a STARVATION FLOOR, not a poll: it engages only when an
/// iteration both delivered NOTHING and returned early, and it never touches a
/// topic that is actually flowing (see [`decide_observer_pacing`]).
const OBSERVER_PACING_FLOOR: Duration = Duration::from_millis(5);
const OBSERVER_CHURN_TOLERANCE: u32 = 8;

// Drift guards, enforced at COMPILE time: the floor must be big enough
// to actually bound a spin, and far enough below the report cadence that pacing
// can never visibly delay `topic hz`'s 1s report; and a zero tolerance would
// pace the very first non-blocking wait (every attach has a legitimate flurry).
const _: () = assert!(
    OBSERVER_PACING_FLOOR.as_millis() >= 1,
    "a sub-millisecond pacing floor does not bound a spin"
);
const _: () = assert!(
    OBSERVER_PACING_FLOOR.as_millis() * 10 <= HZ_REPORT_INTERVAL.as_millis(),
    "the pacing floor must stay an order of magnitude below the hz report cadence"
);
const _: () = assert!(
    OBSERVER_CHURN_TOLERANCE >= 1,
    "a zero churn tolerance would pace the very first non-blocking wait"
);

/// The PURE per-iteration pacing decision for the `topic echo` /
/// `topic hz` observer loops — `Some(pause)` means "pause this long before
/// re-entering the wait", `None` means "go straight back around".
///
/// Both observer loops route every iteration through here, so under the
/// workspace's `dead_code = "deny"` lint a regression that stops consulting it
/// fails the BUILD, not just a test (the same structural guarantee
/// [`decide_wait_outcome`] carries).
///
/// The rules, in the order they are checked:
///
/// 1. `delivered > 0` — the topic is FLOWING. Never paced, budget reset. This
///    is the latency contract: a real stream is displayed at event speed, and
///    the pacing floor can never sit between a frame and its display.
/// 2. `waited >= requested` — the wait genuinely spent its budget (the ordinary
///    silent topic: one blocking wait per heartbeat). Not churn, budget reset,
///    never paced — a well-behaved silent topic keeps costing ~one wake per
///    second exactly as the event-driven rewrite left it.
/// 3. Otherwise the iteration returned EARLY with nothing to show — progress-
///    free churn. Tolerate a burst of it (a legitimate flurry: our own
///    `SubscriberConnected`, a history pump, a publisher reconnecting), then
///    pace.
///
/// Pausing cannot lose data (Principle #6): frames stay queued in shared memory
/// and the very next iteration drains them. The floor only ever delays the
/// display of frames on a topic that, by rule 1, delivered none.
fn decide_observer_pacing(
    delivered: usize,
    waited: Duration,
    requested: Duration,
    consecutive_idle_churn: &mut u32,
) -> Option<Duration> {
    if delivered > 0 || waited >= requested {
        *consecutive_idle_churn = 0;
        return None;
    }
    *consecutive_idle_churn = consecutive_idle_churn.saturating_add(1);
    if *consecutive_idle_churn >= OBSERVER_CHURN_TOLERANCE {
        Some(OBSERVER_PACING_FLOOR)
    } else {
        None
    }
}

/// How many times an observer loop has actually ENGAGED its pacing
/// floor in this process.
///
/// Observable state — and the pin that stops the pacing going
/// inert. Moving `let mut idle_churn = 0;` from OUTSIDE the
/// `while running` loop to INSIDE it neuters the pacing without touching a
/// single character the structural guards read: the churn budget then
/// resets every iteration, `decide_observer_pacing` can never reach
/// `OBSERVER_CHURN_TOLERANCE`, and the spin is fully restored — while the
/// code compiles with zero diagnostics under lints stricter than this
/// crate's, every structural assertion passes character for character, and
/// the pure oracle vectors stay green because they thread their OWN
/// `&mut churn` and are blind to the call site's state by construction.
///
/// A counter closes that: the e2e flood arm asserts this ADVANCED, which is
/// reachable only if a real observer loop really did engage the floor. It is a
/// LOWER bound, and therefore load-IMMUNE in exactly the way a CPU-percentage
/// gate is not — a slow machine delays the eighth consecutive
/// churning iteration, it cannot prevent it, whereas a starved spinner
/// accumulates LESS CPU and would pass an upper bound.
static OBSERVER_PACING_ENGAGED: AtomicU64 = AtomicU64::new(0);

/// How many times an observer loop has engaged its pacing floor in
/// this process (observable state).
///
/// Monotonic, so callers compare a DELTA across the window they care about
/// rather than an absolute value. This is the observable that stops the whole
/// starvation-floor fix going INERT: a call site whose churn budget is neutered still
/// passes every structural and pure-oracle assertion, but can never advance
/// this count. Asserted `> 0` under a progress-free event flood by
/// `topic_observer_iox2_test` — a LOWER bound, which a slow runner can delay
/// but not prevent.
pub fn observer_pacing_engaged_count() -> u64 {
    OBSERVER_PACING_ENGAGED.load(Ordering::Relaxed)
}

/// Record that an observer loop engaged its pacing floor. Called from
/// inside the guarded arm ONLY, so the count cannot advance unless
/// [`decide_observer_pacing`] actually returned a pause.
fn note_pacing_engaged() {
    OBSERVER_PACING_ENGAGED.fetch_add(1, Ordering::Relaxed);
}

/// A one-line, human-readable statement of WHERE an observer's frames
/// are supposed to come from, used in the silence notice so "nothing is
/// arriving" is attributable rather than ambiguous.
///
/// `robot` is LAN-sourced (it arrives in a remote robot's announce), so the
/// remote arm sanitizes it — every caller is safe by construction.
fn observer_source_description(source: &TopicSource) -> String {
    match source {
        TopicSource::Local => "a local producer on this machine".to_string(),
        TopicSource::Remote { robot, .. } => format!(
            "the shared cerulion-netd mirror of robot '{}'",
            sanitize_display(robot)
        ),
    }
}

/// The silence notice itself — what an observer prints when it has
/// been attached for a while and NOTHING has arrived.
///
/// Names every fact that is actually known and NO fact that is not: the topic,
/// where its frames should come from, that zero have arrived, and over how
/// long. It deliberately does NOT diagnose (the observer cannot know whether
/// the producer is dead, the route is broken, or the topic is simply idle) —
/// stating the observation is what makes the state visible; guessing a cause
/// would be a fabricated claim.
/// The count is DECODABLE frames, and the wording says so. Both observers count
/// only frames that survive the wire filter (`deliver_raw_frame` drops a frame
/// whose envelope is short or whose schema hash does not match), so a storm of
/// malformed frames would leave this at zero while frames are demonstrably
/// crossing. "0 frames" would be an affirmatively wrong claim about the network
/// in exactly the case an operator most needs to tell apart from a dead route.
fn silence_notice(topic: &str, source_desc: &str, silent_for: Duration) -> String {
    format!(
        "[waiting] '{}' — attached to {}; 0 decodable frames in {:.0}s",
        sanitize_display(topic),
        source_desc,
        silent_for.as_secs_f64(),
    )
}

/// Pinned recipe-3 (layout-sensitive + package-qualified) schema hash for
/// `std_msgs/String`, used by `topic_echo` (and the TUI) to pretty-print
/// known payloads.
///
/// Pinned literal — kept in sync with the generated
/// `<String as ShmMessage>::SCHEMA_HASH` by the
/// `pinned_hashes_match_generated_constants` test in
/// `crates/cerulion_cli_engine/tests/integration_test.rs`. After an intentional
/// hash-recipe change, run that test, copy the actual values from the
/// failure message, and update these literals (never hand-compute).
pub const STD_MSGS_STRING_SCHEMA_HASH: u64 = 0xC3D3C3AB405F5F49;

/// Pinned recipe-3 (layout-sensitive + package-qualified) schema hash for
/// `sensor_msgs/Image`. See [`STD_MSGS_STRING_SCHEMA_HASH`] for the sync
/// contract.
pub const SENSOR_MSGS_IMAGE_SCHEMA_HASH: u64 = 0xBD08DAFCECCFA89E;

/// The one-time remote-fallback breadcrumb `topic echo` prints when a
/// frame's type is not compiled locally and is instead decoded via a robot's
/// served schema. FACTUAL, not predictive — it is emitted only when a frame
/// ACTUALLY decodes remotely (see [`write_custom_decode`]), so the wording
/// states what IS happening ("are being decoded"), never what merely MIGHT
/// ("will be decoded" — a promise a local-heavy topic never keeps).
/// `robot` is LAN-sourced, so it is sanitized here — every caller is safe by
/// construction.
fn remote_fallback_breadcrumb(topic: &str, robot: &str) -> String {
    format!(
        "[schema: frames on '{topic}' carrying a type not compiled locally are being \
         decoded via robot '{}' (fetched over the network; nothing written to disk)]",
        sanitize_display(robot),
    )
}

/// Default max array ELEMENTS `topic echo` renders inline before
/// truncating with a trailing `...` inside the brackets plus a `(N elements)`
/// total annotation — matching `ros2 topic echo`'s default `--truncate-length`.
/// Overridable per-run via the `--truncate-length` flag; named in ONE place (the
/// CLI's flag default references this constant). Keeps a 57600-point
/// PointCloud2 `data` field or a long joint-state array readable instead of
/// flooding the terminal.
pub const DEFAULT_ECHO_TRUNCATE_LENGTH: usize = 128;

/// Total lines one frame's field render may emit.
///
/// `truncate_length` bounds each array INDEPENDENTLY, and it is re-applied at
/// every nesting level, so a frame whose element arrays nest costs
/// `truncate_length ^ depth` lines: a `visualization_msgs/MarkerArray`
/// (`Marker[] markers`, each with `Point[] points` + `ColorRGBA[] colors`)
/// renders ~1.5×10^5 lines PER FRAME at the default 128 — an echo no operator
/// can read, on a topic real robots publish at 10 Hz. Before the walker was
/// taught the canonical element framing those fields were one opaque line
/// each, so this ceiling is what keeps the new element rendering from
/// regressing `topic echo`. Mirrors `cerulion_viz::archetype`'s
/// `DUMP_MAX_LINES`, which carries a global budget for the same reason.
///
/// A frame that hits it renders its first `ECHO_MAX_LINES` lines plus a loud
/// truncation notice — never a silent cut, and never an unbounded dump. This
/// is deliberately a CONSTANT, not a multiple of `truncate_length`: raising
/// `--truncate-length` widens each array, and the whole point of the ceiling
/// is that the total does not follow that exponentially.
const ECHO_MAX_LINES: usize = 2000;

/// Append `", ...] (N elements)"` when an array of `count` elements was
/// truncated at `shown`, or close the bracket plain otherwise. ONE seam so every
/// inline leaf array (`uint8[]`, `float64[]`, …) shares the exact truncation
/// shape: a trailing `...` INSIDE the brackets + a `(N elements)` total
/// annotation for truncated renders only (an untruncated array gets NO
/// annotation — ros2-parity).
fn close_array(parts: &[String], count: usize, shown: usize) -> String {
    if count > shown {
        format!("[{}, ...] ({} elements)", parts.join(", "), count)
    } else {
        format!("[{}]", parts.join(", "))
    }
}

/// Format ONE decoded LEAF value (scalar / string / byte-blob /
/// numeric-array) as a compact inline string. `Nested` + `Array` are multi-line
/// composites rendered by [`write_named_value`] and never reach here.
///
/// Every STRING value is terminal-escape-sanitized ([`sanitize_display`]) THEN
/// quoted: a wire string is LAN/robot-supplied data,
/// so an embedded ANSI/CSI escape must never reach the operator's terminal.
/// A `uint8[]`/`int8[]` byte array now renders its ELEMENTS (decimal
/// u8) like every other array — truncated at `truncate_length` — instead of the
/// old opaque `<N bytes>` placeholder (a PointCloud2 `data` field or a LowState
/// `head: [255, 0]` are now readable).
fn format_leaf_value(
    kind: &cerulion_core::codegen::FrameValueKind,
    truncate_length: usize,
) -> String {
    use cerulion_core::codegen::FrameValueKind as K;
    match kind {
        K::Bool(v) => v.to_string(),
        K::I8(v) => v.to_string(),
        K::U8(v) => v.to_string(),
        K::I16(v) => v.to_string(),
        K::U16(v) => v.to_string(),
        K::I32(v) => v.to_string(),
        K::U32(v) => v.to_string(),
        K::I64(v) => v.to_string(),
        K::U64(v) => v.to_string(),
        // `{:?}` keeps the float-ness visible (`1.0`, not `1`) — the familiar
        // `ros2 topic echo` presentation.
        K::F32(v) => format!("{v:?}"),
        K::F64(v) => format!("{v:?}"),
        // Sanitize THEN quote: `{:?}` escapes embedded quotes/backslashes and
        // sanitize has already neutralized every control char.
        K::Str(s) => format!("{:?}", sanitize_display(s)),
        K::Bytes(b) => format_byte_array(b, truncate_length),
        K::PrimArray(a) => format_prim_array(a, truncate_length),
        // The walker maps a DynamicArray of Nested / String / bool / array
        // element to this opaque-bytes arm (frame_walker.rs) when the bytes are
        // NOT canonically framed, so the label must NOT claim
        // "nested-message" — a `bool[]` lands here too. Neutral + accurate:
        // opaque producer bytes with no framing we can enumerate.
        K::NestedArrayOpaque(b) => format!("<{} bytes, opaque array>", b.len()),
        // Composites are rendered multi-line by write_named_value; the arm is
        // unreachable in practice but keeps the match total.
        K::Nested(_) | K::Array(_) | K::NestedArray { .. } => String::from("<nested>"),
    }
}

/// Render a `uint8[]` / `int8[]` byte array as its ELEMENTS (decimal
/// u8), truncated to `truncate_length` with a trailing `...` + `(N elements)`
/// tail. Bytes are raw wire numbers, NOT a string, so no sanitize/quote applies
/// (a `u8` renders as a decimal digit sequence — never a control char).
fn format_byte_array(bytes: &[u8], truncate_length: usize) -> String {
    let shown = bytes.len().min(truncate_length);
    let parts: Vec<String> = bytes[..shown].iter().map(|b| b.to_string()).collect();
    close_array(&parts, bytes.len(), shown)
}

/// Render a numeric primitive array (`float64[]`, `uint32[9]`,
/// …) as `[e0, e1, …, ...] (N elements)`, truncated to `truncate_length`. Each
/// element is formatted in its NATIVE type (integers stay integer, floats keep
/// the decimal point) — no lossy widen. Bounds-guarded: a malformed
/// short-`bytes` array stops rather than panicking on untrusted wire input.
fn format_prim_array(a: &cerulion_core::codegen::PrimArray, truncate_length: usize) -> String {
    use cerulion_core::codegen::PrimType as P;
    let shown = a.count.min(truncate_length);
    let sz = a.elem.size();
    let mut parts: Vec<String> = Vec::with_capacity(shown);
    for i in 0..shown {
        let start = i * sz;
        let Some(b) = a.bytes.get(start..start + sz) else {
            break;
        };
        let elem = match a.elem {
            P::I16 => i16::from_le_bytes([b[0], b[1]]).to_string(),
            P::U16 => u16::from_le_bytes([b[0], b[1]]).to_string(),
            P::I32 => i32::from_le_bytes([b[0], b[1], b[2], b[3]]).to_string(),
            P::U32 => u32::from_le_bytes([b[0], b[1], b[2], b[3]]).to_string(),
            P::I64 => {
                i64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]).to_string()
            }
            P::U64 => {
                u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]).to_string()
            }
            P::F32 => format!("{:?}", f32::from_le_bytes([b[0], b[1], b[2], b[3]])),
            P::F64 => format!(
                "{:?}",
                f64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]])
            ),
        };
        parts.push(elem);
    }
    close_array(&parts, a.count, shown)
}

/// Write ONE `name: value` line — or a multi-line block for a nested
/// message / message array — indented `indent` levels under the schema header.
/// `truncate_length` bounds EVERY array — inline leaf arrays (threaded
/// through [`format_leaf_value`]) AND this multi-line nested-message array arm.
fn write_named_value(
    writer: &mut dyn Write,
    name: &str,
    kind: &cerulion_core::codegen::FrameValueKind,
    indent: usize,
    truncate_length: usize,
    lines: &mut usize,
) {
    if *lines >= ECHO_MAX_LINES {
        return;
    }
    *lines += 1;
    use cerulion_core::codegen::FrameValueKind as K;
    let pad = "  ".repeat(indent + 1);
    // Security: a field NAME is wire-derived (for a remote type it comes from
    // a robot's LAN-fetched `.msg`, which the parser does not
    // identifier-validate), so it is sanitized before display in
    // EVERY arm (a hostile `x\x1b[2J…` field name would otherwise clear the
    // operator's terminal each frame). Nested field names are sanitized on their
    // own recursion through this fn; the `[i]` index labels are numeric.
    let name = sanitize_display(name);
    match kind {
        K::Nested(inner) => {
            writeln!(writer, "{pad}{name}:").ok();
            render_frame_fields_budgeted(writer, inner, indent + 1, truncate_length, lines);
        }
        // A canonically-framed `Nested[]` / `string[]` renders exactly
        // like a fixed-length nested array — element-by-element, bounded by
        // `truncate_length` per level AND by `ECHO_MAX_LINES` overall.
        K::Array(elems)
        | K::NestedArray {
            elements: elems, ..
        } => {
            writeln!(writer, "{pad}{name}: [{} element(s)]", elems.len()).ok();
            let mut shown = 0usize;
            for (i, elem) in elems.iter().take(truncate_length).enumerate() {
                if *lines >= ECHO_MAX_LINES {
                    break;
                }
                *lines += 1;
                shown += 1;
                match elem {
                    K::Nested(inner) => {
                        writeln!(writer, "{pad}  [{i}]:").ok();
                        render_frame_fields_budgeted(
                            writer,
                            inner,
                            indent + 2,
                            truncate_length,
                            lines,
                        );
                    }
                    other => {
                        writeln!(
                            writer,
                            "{pad}  [{i}]: {}",
                            format_leaf_value(other, truncate_length)
                        )
                        .ok();
                    }
                }
            }
            if elems.len() > shown {
                writeln!(writer, "{pad}  … ({} total)", elems.len()).ok();
            }
        }
        leaf => {
            writeln!(
                writer,
                "{pad}{name}: {}",
                format_leaf_value(leaf, truncate_length)
            )
            .ok();
        }
    }
}

/// Render every field of a decoded
/// [`FrameValue`](cerulion_core::codegen::FrameValue) as readable
/// `name: value` lines — the output shows the data a frame carries, not a field
/// count. Reused by the LOCAL-walker and REMOTE-walker echo paths so there is
/// ONE rendering seam. `truncate_length` is threaded down to bound
/// every rendered array.
fn render_frame_fields(
    writer: &mut dyn Write,
    value: &cerulion_core::codegen::FrameValue,
    indent: usize,
    truncate_length: usize,
) {
    let mut lines = 0usize;
    render_frame_fields_budgeted(writer, value, indent, truncate_length, &mut lines);
    if lines >= ECHO_MAX_LINES {
        writeln!(
            writer,
            "  … (output truncated at {ECHO_MAX_LINES} lines — this frame nests more \
             element arrays than one echo can render)"
        )
        .ok();
    }
}

/// [`render_frame_fields`]'s recursive half, carrying the shared
/// [`ECHO_MAX_LINES`] budget.
fn render_frame_fields_budgeted(
    writer: &mut dyn Write,
    value: &cerulion_core::codegen::FrameValue,
    indent: usize,
    truncate_length: usize,
    lines: &mut usize,
) {
    for field in &value.fields {
        if *lines >= ECHO_MAX_LINES {
            return;
        }
        write_named_value(
            writer,
            &field.name,
            &field.value,
            indent,
            truncate_length,
            lines,
        );
    }
}

/// Resolve a frame's `schema_hash` to a `Schema: <name>
/// (0x…)` line via the SAME ladder `topic echo` uses — the LOCAL walker
/// (built-ins ∪ workspace `.msg`/YAML) first, then the remote-seeded walker.
/// An unresolved hash degrades to `Schema hash: 0x…` plus a short hint. The
/// remote-served name is LAN-supplied, so it is [`sanitize_display`]'d.
fn resolve_schema_line(
    schema_hash: u64,
    local_walker: &cerulion_core::codegen::FrameWalker,
    remote_walker: Option<&(String, cerulion_core::codegen::FrameWalker)>,
) -> String {
    if let Some(name) = local_walker.schema_name_for_hash(schema_hash) {
        return format!("Schema: {name} (0x{schema_hash:016x})");
    }
    if let Some((_robot, walker)) = remote_walker {
        if let Some(name) = walker.schema_name_for_hash(schema_hash) {
            return format!("Schema: {} (0x{schema_hash:016x})", sanitize_display(name));
        }
    }
    format!(
        "Schema hash: 0x{schema_hash:016x} (schema name unresolved — not a built-in or \
         workspace type; add the robot's `.msg` under `schemas/` to name it)"
    )
}

/// Render the decode line(s) for one `topic echo` frame whose
/// `schema_hash` is not a pinned built-in (`std_msgs/String` /
/// `sensor_msgs/Image` are handled inline by the caller).
///
/// Tries the LOCAL walker first (built-ins + the workspace `.msg` store +
/// `schemas/*.yaml`): a locally-known type decodes with NO
/// network and NO breadcrumb. Otherwise it falls back to the remote-seeded
/// `remote_walker` — and ONLY THEN, on the FIRST frame that actually decodes
/// remotely, emits the one-time [`remote_fallback_breadcrumb`]. A frame known to
/// NEITHER walker is a hash-only hex dump. `breadcrumb_shown` latches the
/// once-only discipline; keeping the breadcrumb here (not pre-loop) is what
/// makes it factual rather than a prediction a local-heavy topic breaks.
///
/// `truncate_length` bounds EVERY array rendered on this walker path —
/// byte, numeric, and nested-message arrays. It does NOT affect the caller's
/// pinned `sensor_msgs/Image` fast-path (a deliberate curated summary) nor the
/// undecodable-frame hex preview below (a fixed 64-byte raw-wire diagnostic).
#[allow(clippy::too_many_arguments)]
fn write_custom_decode(
    writer: &mut dyn Write,
    topic: &str,
    header: &cerulion_core::wire::WireHeader,
    payload: &[u8],
    local_walker: &cerulion_core::codegen::FrameWalker,
    remote_walker: Option<&(String, cerulion_core::codegen::FrameWalker)>,
    breadcrumb_shown: &mut bool,
    explained_hashes: &mut std::collections::BTreeSet<u64>,
    truncate_length: usize,
) {
    // Reconstruct the full wire frame (32-byte header + payload) once; `walk`
    // slices the header off.
    let mut frame = vec![0u8; cerulion_core::wire::WireHeader::SIZE];
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(payload);

    // A schema whose hash the walker RESOLVES but whose
    // frame fails to `walk()` (a truncated / corrupt frame) must be distinguished
    // from a genuinely-unknown type — otherwise both render as bare hex. Remember
    // the first such `(name, WalkError)` and surface it before the hex dump.
    let mut decode_failure: Option<(String, cerulion_core::codegen::WalkError)> = None;

    // LOCAL walker first: a frame whose type the workspace already knows decodes
    // LOCALLY — no network, no remote fallback, no breadcrumb.
    // render the full decoded field TREE, not a bare field count.
    if let Some(name) = local_walker.schema_name_for_hash(header.schema_hash) {
        match local_walker.walk(name, &frame) {
            Ok(value) => {
                // The schema NAME is a local, trusted identifier — no sanitize
                // needed (it never crossed the network). Field VALUES are
                // sanitized inside the renderer.
                writeln!(writer, "  {name}:").ok();
                render_frame_fields(writer, &value, 1, truncate_length);
                return;
            }
            Err(e) => decode_failure = Some((name.to_string(), e)),
        }
    }

    // Fall back to the remote-seeded walker — a frame whose custom-type hash the
    // walker knows is decoded + RENDERED (structurally validated) instead of
    // dumped as opaque hex.
    let mut remote_decoded = None;
    if let Some((robot, walker)) = remote_walker {
        if let Some(name) = walker.schema_name_for_hash(header.schema_hash) {
            match walker.walk(name, &frame) {
                Ok(value) => remote_decoded = Some((robot.as_str(), name, value)),
                // Known-to-the-remote-walker but undecodable: remember it (only
                // if the local walker did not already record a failure).
                Err(e) => {
                    decode_failure.get_or_insert((name.to_string(), e));
                }
            }
        }
    }
    match remote_decoded {
        Some((robot, name, value)) => {
            // The FACTUAL breadcrumb, emitted ONCE on the FIRST frame that
            // actually decodes remotely (never pre-loop, never for a
            // locally-decoded frame). `robot` is LAN-sourced — sanitized inside
            // `remote_fallback_breadcrumb`.
            if !*breadcrumb_shown {
                writeln!(writer, "{}", remote_fallback_breadcrumb(topic, robot)).ok();
                *breadcrumb_shown = true;
            }
            // BOTH `name` (the remote-served schema
            // name) and `robot` (announce-space) are LAN-supplied — sanitize
            // terminal-escape chars before display. Field VALUES are sanitized
            // inside the renderer.
            writeln!(
                writer,
                "  {} (via robot '{}'):",
                sanitize_display(name),
                sanitize_display(robot),
            )
            .ok();
            render_frame_fields(writer, &value, 1, truncate_length);
        }
        None => {
            // Name the known-but-undecodable schema before the
            // hex. The name AND the WalkError Display can carry wire-derived
            // field/schema names (remote), so BOTH are sanitized.
            if let Some((name, err)) = &decode_failure {
                writeln!(
                    writer,
                    "  ({}: decode failed: {})",
                    sanitize_display(name),
                    sanitize_display(&err.to_string()),
                )
                .ok();
            } else {
                // NEITHER walker names this hash. Printing
                // bare hex and nothing else would make the one surface an operator
                // reaches for first say only "here are some bytes",
                // on the command whose whole job is to
                // say what is on a topic. (`topic info` names this
                // condition too; the two surfaces agree.)
                //
                // No candidate name is threaded here: echo's remote walker is
                // keyed by hash, and had a name been known the branch above would
                // have run. `schema_unidentified` is the correct verdict for this
                // vantage.
                //
                // Once per distinct hash, never per frame.
                // Echo always passes `candidate: None`, so only the
                // `Unidentified` arm is reachable and its text is byte-invariant
                // for a given hash — printing it on every frame of a 500 Hz
                // undecodable topic is ~135 KB/s of identical prose carrying zero
                // per-frame information, which is the very flood class the rest of
                // this path latches. The per-frame hex preview is untouched: the
                // paragraph explains the dump, so it needs saying once.
                //
                // A SEPARATE set from `breadcrumb_shown` deliberately — that
                // latches the remote-fallback breadcrumb, a different condition,
                // and sharing it would let whichever fired first suppress the
                // other. Bounded by the topic's schema cardinality (in practice 1).
                if explained_hashes.insert(header.schema_hash) {
                    let diagnosis =
                        cerulion_core::codegen::diagnose_unknown_hash(header.schema_hash, None);
                    writeln!(writer, "  (undecodable: {})", diagnosis.detail()).ok();
                }
            }
            // This hex preview shows the raw wire bytes of a FAILED
            // decode (not array ELEMENTS of a decoded field), so its 64-byte
            // bound is DELIBERATELY independent of `--truncate-length` — it is a
            // fixed diagnostic dump, not a rendered array.
            let hex: String = payload
                .iter()
                .take(64)
                .map(|b| format!("{:02x}", b))
                .collect::<Vec<_>>()
                .join(" ");
            let suffix = if payload.len() > 64 { "..." } else { "" };
            writeln!(writer, "  payload: {}{}", hex, suffix).ok();
        }
    }
}

/// Echo messages from a topic.
///
/// Pretty-prints payloads for well-known schemas (`std_msgs/String`,
/// `sensor_msgs/Image`) and falls back to a hex preview otherwise. Schema
/// recognition is by `WireHeader::schema_hash` — the
/// layout-sensitive hash baked in by the message codegen, so it survives
/// recompiles and round-trips through the wire format unchanged.
// Logging-rule exception, per the comment at the site: the silence notice is CLI output
// on STDERR (stdout carries the frame stream), and it must appear whatever
// RUST_LOG says — a silent `topic echo` with no explanation is the exact
// failure it reports.
#[allow(clippy::print_stderr)]
pub fn topic_echo(
    topic: &str,
    schemas_dir: Option<&std::path::Path>,
    running: Arc<AtomicBool>,
    writer: &mut dyn Write,
    // Max array ELEMENTS to render inline before truncating with a
    // trailing `...` + `(N elements)` tail (the `--truncate-length` flag; the CLI
    // passes [`DEFAULT_ECHO_TRUNCATE_LENGTH`] when the flag is omitted). Bounds
    // every array in the CUSTOM-DECODE walker path (`write_custom_decode`) —
    // u8/byte arrays (never an opaque `<N bytes>` placeholder) AND
    // numeric/nested-message arrays. The pinned `sensor_msgs/Image` fast-path
    // deliberately keeps its one-line `data=N bytes` summary (it never dumps
    // pixel bytes), so the bound does NOT reach it.
    truncate_length: usize,
) -> CliResult<()> {
    // The observer transport is LOCAL-ONLY — a genuine local topic
    // reads directly; a REMOTE topic (or a netd mirror) is demanded from the shared
    // cerulion-netd daemon, which re-injects the frames into local SHM where the
    // subscriber below reads them normally.
    // `ensure_topic_available` returns the OPEN subscriber it validated
    // (a live local topic, or a freshly-demanded remote mirror) — opened once, no
    // stale-mirror re-open trap.
    // `_netd` is the DEMAND GUARD for a remotely-observed topic — bound (not
    // dropped) so its UDS connection to cerulion-netd stays open for the whole
    // observer loop; releasing the demand only when this fn returns / is signalled
    // (connection close = release). `_transport` stays bound for the subscriber.
    // `running` rides in so a Ctrl-C during the first-contact wait is
    // honoured — this verb's own handler is what removed the default SIGINT
    // disposition, so without it the wait is uninterruptible. And an interrupted
    // resolve exits QUIETLY (exit 0, the repo's clean-cancel precedent) rather than
    // rendering a verdict about a topic we stopped looking for.
    let (_transport, subscriber, source, _netd) =
        match ensure_topic_available(topic, schemas_dir, Some(running.as_ref())) {
            Ok(v) => v,
            Err(_) if interrupted_before_observing(Some(running.as_ref())) => return Ok(()),
            Err(e) => return Err(e),
        };

    // Build a LOCAL walker from the desk's built-in types +
    // (when present) the workspace `.msg` store + `schemas/*.yaml`, so a frame
    // carrying a WORKSPACE type decodes LOCALLY (no network). The remote walker
    // below stays the FALLBACK for types the desk genuinely lacks.
    let local_walker = local_walker_from_workspace(schemas_dir);

    // Recipe-3 layout-sensitive schema hashes — a `const fn` over the name
    // alone CANNOT reproduce them (the recipe folds in wire_fixed_size +
    // per-field canonical_str + nested layout). The pinned literals live in
    // the named consts above and are kept in sync with the generated
    // `<T>::SCHEMA_HASH` by `pinned_hashes_match_generated_constants`.
    const STD_MSGS_STRING: u64 = STD_MSGS_STRING_SCHEMA_HASH;
    const SENSOR_MSGS_IMAGE: u64 = SENSOR_MSGS_IMAGE_SCHEMA_HASH;

    // Delivery accounting: the wire sequence of the last frame this
    // run displayed. The non-notifying degrade window (~1 heartbeat between
    // timeout drains) can silently overflow the bounded `drop_oldest`
    // subscriber queue under a fast publisher — evicted frames would vanish
    // with no trace. A forward jump in the wire sequence between two
    // consecutively displayed frames makes that loss VISIBLE (the gap marker
    // below).
    //
    // Scope: `WireHeader::sequence` is PER-PUBLISHER. Graph topics are
    // single-writer by contract (the norm), where a gap is a true loss
    // signal. On an explicitly opted-in `multi_publisher_topics:` topic the
    // interleaved per-publisher sequences make the marker meaningless noise —
    // accepted, since echo cannot attribute frames to publishers.
    //
    // The FIRST displayed frame only sets the baseline (a late-joining
    // observer legitimately starts mid-stream — seq > 0 at attach is not a
    // loss). A sequence REGRESSION (publisher restart) re-baselines silently.
    let mut last_seq: Option<u32> = None;
    let mut consecutive_wait_errors: u32 = 0;

    // A REMOTE topic already carries the walker seeded during the
    // ingress resolve — reuse it (no second discovery). A LOCAL topic keeps the
    // earlier behavior: a ONE-TIME best-effort remote resolve as the FALLBACK for
    // a frame carrying a type the desk never compiled (learn the type from a
    // robot's catalog + fetch its closure + seed a FrameWalker IN MEMORY, no disk
    // writes). A failed resolve is a `None` walker — the local path is unchanged.
    //
    // WHERE this observer's frames are supposed to come from, captured
    // before `source` is consumed below — it is the whole point of the silence
    // notice (an operator staring at a blank `topic echo` must be able to tell
    // "the local graph is idle" from "the netd mirror of robot X is carrying
    // nothing").
    let source_desc = observer_source_description(&source);

    let remote_walker: Option<(String, cerulion_core::codegen::FrameWalker)> = match source {
        TopicSource::Remote { robot, walker } => Some((robot, walker)),
        TopicSource::Local => resolve_remote_walker_for_topic(topic),
    };
    // The remote-fallback breadcrumb is emitted
    // LAZILY — only on the FIRST frame that ACTUALLY decodes via the remote
    // walker (see `write_custom_decode`), never here at resolve time. A resolved
    // remote walker is a fallback a LOCAL-heavy topic never takes (its type is
    // already in the workspace store, so every frame decodes
    // locally); announcing "unknown-type frames will be decoded via robot X"
    // pre-loop PREDICTED a remote decode that never happened. `breadcrumb_shown`
    // latches the once-only discipline across the whole run.
    let mut breadcrumb_shown = false;
    // Distinct `schema_hash`es whose
    // undecodable EXPLANATION has already been printed this run. A separate
    // latch from `breadcrumb_shown` on purpose — different condition, and one
    // shared bit would let whichever fired first suppress the other.
    let mut explained_hashes: std::collections::BTreeSet<u64> = std::collections::BTreeSet::new();

    // Silence + pacing state. `silence_since` is when the current
    // run of "nothing displayed" began (attach time, then reset by every
    // displayed frame); `notices_emitted` drives the notice schedule; and
    // `idle_churn` is the progress-free-iteration budget the pacing floor
    // engages after.
    let mut silence_since = std::time::Instant::now();
    let mut notices_emitted: u32 = 0;
    let mut idle_churn: u32 = 0;

    while running.load(Ordering::Relaxed) {
        // Block on the event listener (wake on arrival) instead of
        // polling with a sleep. `wait_for_message` returns as soon as a
        // `SentSample`/`SentHistory` event fires, and on timeout it STILL
        // drains the queue — so a non-notifying publisher degrades to
        // heartbeat-cadence batched display rather than silence.
        //
        // The wait's ACTUAL duration is measured so the loop can tell
        // a wait that spent its budget (an ordinary silent topic) from one that
        // returned early with nothing (progress-free churn — see
        // `decide_observer_pacing`).
        let wait_started = std::time::Instant::now();
        let received = subscriber.wait_for_message(OBSERVER_HEARTBEAT, |msg| {
            let header = msg.header();
            let payload = msg.payload();
            if let Some(last) = last_seq {
                // `checked_sub` → `None` on a sequence REGRESSION (publisher
                // restart): re-baseline silently. delta 0 (duplicate) and 1
                // (consecutive) are gap-free; delta ≥ 2 means delta−1 frames
                // were never delivered to this observer.
                if let Some(delta) = header.sequence.checked_sub(last) {
                    let missed = delta.saturating_sub(1);
                    if missed > 0 {
                        writeln!(
                            writer,
                            "[gap: {missed} frame(s) missed — subscriber queue overflow \
                             (drop_oldest) between drains; wire seq jumped {last} -> {}]",
                            header.sequence,
                        )
                        .ok();
                    }
                }
            }
            last_seq = Some(header.sequence);
            writeln!(
                writer,
                "seq={} ts={}ns schema=0x{:016x} size={}",
                header.sequence, header.timestamp_ns, header.schema_hash, header.total_size,
            )
            .ok();

            match header.schema_hash {
                STD_MSGS_STRING => match decode_string_payload(payload) {
                    Some(s) => {
                        writeln!(writer, "  std_msgs/String: {:?}", s).ok();
                    }
                    None => {
                        writeln!(writer, "  std_msgs/String: <undecodable>").ok();
                    }
                },
                SENSOR_MSGS_IMAGE => match decode_image_meta(payload) {
                    Some(meta) => {
                        // EXCEPTION: this pinned fast-path is a
                        // curated one-line summary for the one type where dumping
                        // the `data` pixel bytes is exactly the flood we avoid —
                        // so `--truncate-length` does NOT apply here (it bounds the
                        // custom-decode walker path). `data=N bytes` stays a
                        // summary, never element-rendered.
                        writeln!(
                            writer,
                            "  sensor_msgs/Image: {}x{} encoding={:?} step={} data={} bytes",
                            meta.width, meta.height, meta.encoding, meta.step, meta.data_len,
                        )
                        .ok();
                    }
                    None => {
                        writeln!(writer, "  sensor_msgs/Image: <undecodable>").ok();
                    }
                },
                _ if !payload.is_empty() => {
                    // LOCAL walker first, remote-seeded walker as the
                    // fallback, hash-only hex otherwise. The one-time remote
                    // breadcrumb is emitted LAZILY inside — only when a frame
                    // ACTUALLY decodes remotely (factual, not predictive).
                    write_custom_decode(
                        writer,
                        topic,
                        header,
                        payload,
                        &local_walker,
                        remote_walker.as_ref(),
                        &mut breadcrumb_shown,
                        &mut explained_hashes,
                        truncate_length,
                    );
                }
                _ => {}
            }
        });

        match received {
            // `Ok(0)` = the wait returned with nothing displayed — either the
            // heartbeat elapsed on a genuinely silent topic (loop and block on
            // the listener again, no sleep) or an event woke it that carried no
            // deliverable frame (progress-free churn, which the starvation floor below
            // bounds). `Ok(n)` = displayed `n` messages (woken by a data event,
            // or drained on timeout for a non-notifying publisher). Routed
            // through `decide_wait_outcome` (always `Continue`) so the
            // budget-reset transition shares the one tested decision table.
            Ok(displayed) => {
                let _ = decide_wait_outcome(
                    true,
                    running.load(Ordering::Relaxed),
                    &mut consecutive_wait_errors,
                );
                // A displayed frame ENDS the silence run (and clears
                // the notice schedule, so a topic that stalls again is reported
                // again). Otherwise report the silence on schedule — to STDERR,
                // never stdout: `topic echo`'s stdout is the frame stream and a
                // status line there would corrupt a pipeline.
                if displayed > 0 {
                    silence_since = std::time::Instant::now();
                    notices_emitted = 0;
                } else {
                    let silent_for = silence_since.elapsed();
                    if silence_notice_due(silent_for, notices_emitted) {
                        eprintln!("{}", silence_notice(topic, &source_desc, silent_for));
                        notices_emitted = notices_emitted.saturating_add(1);
                    }
                }
                // A CLI observer must never pin a core. An external
                // event source that yields no deliverable frame returns this
                // wait instantly; without a floor that is an unbounded tight
                // loop (measured at ~100% of a core). A flowing topic is
                // never paced.
                if let Some(pause) = decide_observer_pacing(
                    displayed,
                    wait_started.elapsed(),
                    OBSERVER_HEARTBEAT,
                    &mut idle_churn,
                ) {
                    note_pacing_engaged();
                    std::thread::sleep(pause); // ALLOW: starvation floor — engaged ONLY on a progress-free early-returning wait (never on a flowing or normally-blocking topic), so it is a CPU bound on external event churn, NOT an observer poll.
                }
            }
            // A persistent receive error must never
            // busy-spin silently at 100% CPU with no output.
            //
            // A SIGINT arriving WHILE blocked in `wait_for_message`
            // interrupts the underlying iceoryx2 wait, surfacing here as an
            // `Err` indistinguishable in type from a genuine receive
            // failure — even though `running` may not have flipped false
            // YET (a real scheduling race, see `MAX_CONSECUTIVE_WAIT_ERRORS`'s
            // doc). If `running` is already false, exit cleanly like every
            // other `Ctrl-C`'d command in this repo. Otherwise retry (bounded
            // by count, not a sleep) before treating this as
            // fatal: the very next loop check picks up the shutdown once the
            // handler catches up, with no new signal to interrupt the retry.
            Err(e) => match decide_wait_outcome(
                false,
                running.load(Ordering::Relaxed),
                &mut consecutive_wait_errors,
            ) {
                WaitOutcomeAction::ExitCleanly => break,
                WaitOutcomeAction::Continue => continue,
                WaitOutcomeAction::Fail => {
                    return Err(CliError::Validation(format!(
                        "receive failed on topic '{topic}': {e}"
                    )));
                }
            },
        }
    }

    Ok(())
}

/// Pull the UTF-8 bytes from a `std_msgs/String` SHM payload.
///
/// The wire layout for a single-variable-field message: 8-byte offset table
/// at byte 0 (one 4-byte offset + 4-byte length), then the payload bytes
/// at that offset. Returns `None` if the offset table is missing or the
/// referenced range is out of bounds.
fn decode_string_payload(payload: &[u8]) -> Option<String> {
    if payload.len() < 8 {
        return None;
    }
    let offset = u32::from_le_bytes(payload[0..4].try_into().ok()?) as usize;
    let len = u32::from_le_bytes(payload[4..8].try_into().ok()?) as usize;
    let end = offset.checked_add(len)?;
    if end > payload.len() {
        return None;
    }
    std::str::from_utf8(&payload[offset..end])
        .ok()
        .map(|s| s.to_string())
}

struct ImageMeta {
    width: u32,
    height: u32,
    encoding: String,
    step: u32,
    data_len: usize,
}

/// Pull width/height/encoding/data-size out of a `sensor_msgs/Image` SHM
/// payload. The fixed section layout (matching `ImageFixedSection` in the
/// generated bindings) is `height: u32, width: u32, is_bigendian: u8,
/// step: u32` (with C alignment padding to 16 bytes), followed by a
/// 24-byte offset table covering the three variable fields
/// (header_bytes, encoding, data).
fn decode_image_meta(payload: &[u8]) -> Option<ImageMeta> {
    // FIXED_SIZE = size_of::<ImageFixedSection>() with C alignment = 16 bytes.
    const FIXED_SIZE: usize = 16;
    const TABLE_BYTES: usize = 8 * 3;
    if payload.len() < FIXED_SIZE + TABLE_BYTES {
        return None;
    }
    let height = u32::from_le_bytes(payload[0..4].try_into().ok()?);
    let width = u32::from_le_bytes(payload[4..8].try_into().ok()?);
    // is_bigendian at offset 8 (1 byte) — skipped here.
    let step = u32::from_le_bytes(payload[12..16].try_into().ok()?);

    // Offset-table entry layout: `[u32 offset, u32 len]` per variable field.
    // Field index 1 = encoding, field index 2 = data.
    let read_entry = |idx: usize| -> Option<(usize, usize)> {
        let base = FIXED_SIZE + idx * 8;
        let off = u32::from_le_bytes(payload[base..base + 4].try_into().ok()?) as usize;
        let len = u32::from_le_bytes(payload[base + 4..base + 8].try_into().ok()?) as usize;
        Some((off, len))
    };
    let (enc_off, enc_len) = read_entry(1)?;
    let (_, data_len) = read_entry(2)?;
    let enc_end = enc_off.checked_add(enc_len)?;
    if enc_end > payload.len() {
        return None;
    }
    let encoding = std::str::from_utf8(&payload[enc_off..enc_end])
        .ok()
        .map(|s| s.to_string())
        .unwrap_or_default();

    Some(ImageMeta {
        width,
        height,
        encoding,
        step,
        data_len,
    })
}

/// Measure publish rate on a topic.
pub fn topic_hz(
    topic: &str,
    schemas_dir: Option<&std::path::Path>,
    running: Arc<AtomicBool>,
    writer: &mut dyn Write,
) -> CliResult<()> {
    // Remote-aware — a topic absent locally is DEMANDED from the shared
    // cerulion-netd daemon (netd owns the mirror). The seeded walker in `source` is
    // unused here — `hz` only times arrivals — but the silence report reads its ORIGIN so a
    // silent report can name where the frames were supposed to come from. The netd
    // demand is held by the guard `_netd` (NOT `_transport`, which is the local read
    // transport): the guard's UDS connection stays open for the whole hz loop,
    // releasing the demand on exit (connection close).
    // `ensure_topic_available` returns the OPEN subscriber (opened once, no
    // stale-mirror re-open trap).
    // See `topic_echo` — the wait must be interruptible here too, and an
    // interrupted resolve exits quietly rather than rendering a verdict.
    let (_transport, subscriber, source, _netd) =
        match ensure_topic_available(topic, schemas_dir, Some(running.as_ref())) {
            Ok(v) => v,
            Err(_) if interrupted_before_observing(Some(running.as_ref())) => return Ok(()),
            Err(e) => return Err(e),
        };
    let source_desc = observer_source_description(&source);

    let mut timestamps: Vec<u64> = vec![];
    let mut last_report = std::time::Instant::now();
    let mut consecutive_wait_errors: u32 = 0;
    // How long this run has gone without a single arrival, and the
    // progress-free-iteration budget the pacing floor engages after.
    let mut silence_since = std::time::Instant::now();
    let mut idle_churn: u32 = 0;

    while running.load(Ordering::Relaxed) {
        // Block on the event listener, waking on arrival, instead of
        // a 1 ms poll sleep (~1000 wakes/s). Wait for `min(heartbeat,
        // time-until-next-report)` so the 1s report still ticks on a silent
        // topic — the wait never overshoots the report deadline — while data
        // still wakes us in µs. `wait_for_message` drains on timeout too, so a
        // non-notifying publisher's timestamps are still counted.
        let until_report = HZ_REPORT_INTERVAL.saturating_sub(last_report.elapsed());
        let wait = until_report.min(OBSERVER_HEARTBEAT);
        // Measure what the wait ACTUALLY cost, so the loop can tell a
        // wait that spent its budget from one that returned early with nothing.
        let wait_started = std::time::Instant::now();
        let before = timestamps.len();
        if let Err(e) = subscriber.wait_for_message(wait, |msg| {
            timestamps.push(msg.header().timestamp_ns);
        }) {
            // Same SIGINT-during-blocking-wait class as `topic_echo`
            // — a Ctrl-C arriving while blocked here interrupts the wait,
            // surfacing as an `Err` indistinguishable in type from a genuine
            // receive failure, even though `running` may not have flipped
            // false YET (see `MAX_CONSECUTIVE_WAIT_ERRORS`'s doc: a real
            // scheduling race, resolved by a count-bounded retry rather than
            // a forbidden poll-sleep).
            match decide_wait_outcome(
                false,
                running.load(Ordering::Relaxed),
                &mut consecutive_wait_errors,
            ) {
                WaitOutcomeAction::ExitCleanly => break,
                WaitOutcomeAction::Continue => continue,
                WaitOutcomeAction::Fail => {
                    return Err(CliError::Validation(format!(
                        "receive failed on topic '{topic}': {e}"
                    )));
                }
            }
        }
        // Wait succeeded — reset the retry budget via the shared decision
        // table (always `Continue` for an Ok outcome).
        let _ = decide_wait_outcome(
            true,
            running.load(Ordering::Relaxed),
            &mut consecutive_wait_errors,
        );

        // How many arrivals THIS iteration produced — the input to
        // both the silence clock and the pacing floor.
        let delivered = timestamps.len().saturating_sub(before);
        if delivered > 0 {
            silence_since = std::time::Instant::now();
        }
        // A CLI observer must never pin a core. An external event
        // source that yields no timestamp returns this wait instantly; without
        // a floor that is an unbounded tight loop (measured at ~100% of a
        // core). A flowing topic is never paced, and
        // the floor is far below the 1s report cadence, so the report below
        // still ticks on time.
        if let Some(pause) =
            decide_observer_pacing(delivered, wait_started.elapsed(), wait, &mut idle_churn)
        {
            note_pacing_engaged();
            std::thread::sleep(pause); // ALLOW: starvation floor — engaged ONLY on a progress-free early-returning wait (never on a flowing or normally-blocking topic), so it is a CPU bound on external event churn, NOT an observer poll.
        }

        // Report every second
        if last_report.elapsed() >= HZ_REPORT_INTERVAL {
            if timestamps.len() >= 2 {
                let first = timestamps[0];
                let last = *timestamps.last().unwrap();
                let count = timestamps.len() as f64;
                let span_ns = (last - first) as f64;
                let avg_hz = (count - 1.0) / span_ns * 1e9;

                let intervals: Vec<f64> = timestamps
                    .windows(2)
                    .map(|w| (w[1] - w[0]) as f64 / 1e9)
                    .collect();
                let min = intervals.iter().cloned().fold(f64::INFINITY, f64::min);
                let max = intervals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                let mean = intervals.iter().sum::<f64>() / intervals.len() as f64;
                let variance = intervals.iter().map(|&x| (x - mean).powi(2)).sum::<f64>()
                    / intervals.len() as f64;
                let std_dev = variance.sqrt();

                writeln!(
                    writer,
                    "average rate: {:.2} Hz, min: {:.4}s max: {:.4}s std: {:.4}s window: {}",
                    avg_hz,
                    min,
                    max,
                    std_dev,
                    timestamps.len()
                )
                .ok();
            } else if !timestamps.is_empty() {
                writeln!(
                    writer,
                    "waiting for more messages... (received {})",
                    timestamps.len()
                )
                .ok();
            } else {
                // An EMPTY report window carries no information on its own:
                // nothing distinguishes "about to stream" from "the data
                // plane is dead". The line therefore names what is actually
                // known: where the frames should come from, and how long
                // none have arrived. It states the observation only —
                // the observer cannot know WHY, and guessing would be fabricated.
                writeln!(
                    writer,
                    "no messages received — attached to {}; 0 decodable frames in {:.0}s",
                    source_desc,
                    silence_since.elapsed().as_secs_f64(),
                )
                .ok();
            }

            timestamps.clear();
            last_report = std::time::Instant::now();
        }
        // No UNCONDITIONAL poll sleep — the `wait_for_message` above
        // blocked on the event listener (waking on arrival, bounded by the
        // report deadline), so the loop is event-driven.
        //
        // Event-driven does NOT mean "not spinning":
        // being event-driven is not on its own enough, because an
        // external event source that yields no frame returns that wait
        // instantly. The starvation floor above — NOT this wait — is what bounds
        // that case, and it engages only when an iteration both delivered
        // nothing and returned early.
    }

    Ok(())
}

/// Get info about a specific topic.
///
/// Waits briefly for a message so the publisher has time to deliver
/// history or a new sample after the subscriber connects.
///
/// The reported line resolves the schema NAME (via the SAME
/// ladder `topic echo` uses — the local walker of built-ins ∪ workspace
/// `.msg`/YAML, then a remote-seeded walker) instead of showing a bare hash.
/// The remote fetch is LAZY — attempted only when the desk cannot name the
/// type locally — so a known-type `topic info` stays network-free and instant.
pub fn topic_info(topic: &str, schemas_dir: Option<&std::path::Path>) -> CliResult<String> {
    // Remote-aware — a topic absent locally is DEMANDED from the shared
    // cerulion-netd daemon (netd owns the mirror), so `info` can report a live remote
    // topic AND name its type (item 2) via the walker seeded during the resolve.
    // `ensure_topic_available` returns the OPEN subscriber (opened once, no
    // stale-mirror re-open trap). `_netd` is the DEMAND GUARD for a remotely-observed
    // topic — bound (not dropped) so its UDS connection to cerulion-netd stays open
    // for the whole info read, releasing the demand only when this fn returns / is
    // signalled (connection close). `_transport` is the LOCAL read transport.
    // `topic info` installs no signal handler, so it still dies on the
    // default SIGINT disposition and has no flag to thread.
    let (_transport, subscriber, source, _netd) = ensure_topic_available(topic, schemas_dir, None)?;

    let local_walker = local_walker_from_workspace(schemas_dir);

    // A REMOTE topic's first frame must traverse the gateway handshake (demand
    // token → gateway taps SHM → forwards → local re-inject), so wait longer for
    // it than for a LOCAL topic that is already publishing into SHM.
    let wait = match &source {
        TopicSource::Remote { .. } => Duration::from_secs(3),
        TopicSource::Local => Duration::from_secs(1),
    };
    let mut captured: Option<(u64, u32, u64)> = None;
    let _ = subscriber.wait_for_message(wait, |msg| {
        let header = msg.header();
        captured = Some((header.schema_hash, header.sequence, header.timestamp_ns));
    });

    let Some((schema_hash, sequence, timestamp_ns)) = captured else {
        return Ok(format!("Topic: {topic}\nStatus: active (no messages yet)"));
    };

    // Name the schema via the SAME ladder `topic echo` uses. A
    // REMOTE topic already carries the seeded walker (no second discovery); a
    // LOCAL topic resolves via its local walker, reaching the network LAZILY
    // only when the desk cannot name the type.
    let schema_line = match &source {
        TopicSource::Remote { robot, walker } => {
            let remote = (robot.clone(), walker.clone());
            resolve_schema_line(schema_hash, &local_walker, Some(&remote))
        }
        TopicSource::Local => {
            if local_walker.schema_name_for_hash(schema_hash).is_some() {
                resolve_schema_line(schema_hash, &local_walker, None)
            } else {
                let remote_walker = resolve_remote_walker_for_topic(topic);
                resolve_schema_line(schema_hash, &local_walker, remote_walker.as_ref())
            }
        }
    };

    Ok(format!(
        "Topic: {topic}\n{schema_line}\nLast sequence: {sequence}\nLast timestamp: {timestamp_ns}ns"
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    // `DiscoveredPeer` rides in via `super::*` (the parent's private
    // import), but the `DiscoveryRung` variants used by the robots-section
    // oracles are named explicitly.
    use crate::discovery_ladder::DiscoveryRung;

    /// This source file (`src/topic_cmd.rs`) as a compile-time string, for
    /// the structural pins below. `include_str!` resolves relative
    /// to THIS file, so it is self-referential by design (pure text — no
    /// recursion).
    const TOPIC_CMD_SRC: &str = include_str!("topic_cmd.rs");

    /// The observe-routing decision — a genuine local topic reads
    /// directly; a netd mirror (even when locally openable) or an absent topic goes
    /// through the demand plane. Hand oracle over the four (listed, mirror) cases.
    #[test]
    fn classify_observed_topic_routes_only_genuine_local_direct() {
        // Listed AND not a mirror → the ONLY LocalDirect case (a real local graph).
        assert_eq!(
            classify_observed_topic(true, false),
            ObserveVia::LocalDirect
        );
        // Listed BUT a netd mirror → demand (refcount the shared stream, by
        // design) — never read the mirror without holding a demand.
        assert_eq!(classify_observed_topic(true, true), ObserveVia::NetdDemand);
        // Absent locally → demand (resolve + demand from a robot).
        assert_eq!(
            classify_observed_topic(false, false),
            ObserveVia::NetdDemand
        );
        // Absent but flagged a mirror (a lingering provenance entry) → demand.
        assert_eq!(classify_observed_topic(false, true), ObserveVia::NetdDemand);
    }

    /// The PURE routing decision — the PRODUCTION automagic path (no
    /// explicit locators, scouting ON) uses the shared cerulion-netd query plane;
    /// explicit locators OR scouting-off (hermetic tests / a per-invocation locator
    /// netd's fixed session can't honor) bypass netd for a transient session. Hand
    /// oracle over the cases.
    #[test]
    fn use_netd_query_plane_only_on_the_automagic_path() {
        // Production automagic: no locators + scouting on → netd.
        assert!(use_netd_query_plane(&[], &[], true));
        // An explicit --connect locator → transient (netd's fixed session can't take it).
        assert!(!use_netd_query_plane(
            &["tcp/1.2.3.4:7683".to_string()],
            &[],
            true
        ));
        // An explicit --listen locator → transient.
        assert!(!use_netd_query_plane(
            &[],
            &["tcp/0.0.0.0:0".to_string()],
            true
        ));
        // Scouting OFF (a hermetic test) → transient even with no locators.
        assert!(!use_netd_query_plane(&[], &[], false));
    }

    /// Extract the `{ ... }` body of the first `pub fn <name>(` in `src` by
    /// brace-matching from the signature's opening brace. Returns the body
    /// text (braces included). Panics if the function or its body is not
    /// found — a rename that breaks this is a real signal, not a silent pass.
    fn fn_body<'a>(src: &'a str, sig: &str) -> &'a str {
        let sig_at = src
            .find(sig)
            .unwrap_or_else(|| panic!("signature not found: {sig}"));
        let open = src[sig_at..]
            .find('{')
            .map(|i| sig_at + i)
            .unwrap_or_else(|| panic!("no opening brace after: {sig}"));
        let bytes = src.as_bytes();
        let mut depth = 0usize;
        let mut i = open;
        while i < bytes.len() {
            match bytes[i] {
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        return &src[open..=i];
                    }
                }
                _ => {}
            }
            i += 1;
        }
        panic!("unbalanced braces after: {sig}");
    }

    /// A COMMENT-STRIPPED view of Rust source — `//` to end of line
    /// and `/* … */` blocks, depth-tracked because Rust block comments NEST.
    ///
    /// Every structural walk below runs over this rather than raw source: these
    /// modules NAME the tokens they forbid in order to explain why they are
    /// absent, so a raw-source anti-vacuity control ("the region still contains
    /// `wait_for_message`") can be satisfied by a COMMENT, making every
    /// companion absence assertion vacuous. Crib: `code_only` in
    /// `crates/cerulion_core/tests/cdylib_iox2_log_level_test.rs`.
    ///
    /// String literals are deliberately NOT modelled here. The reason is
    /// NOT that "the one place that would matter is
    /// excluded from the walks by construction" — the
    /// unterminated `/*` inside THIS module's own stripper oracle below opens a
    /// block comment that never closes, so this `code_only` really does truncate
    /// its view at that literal.
    ///
    /// It is safe for THESE consumers by POSITION, not by construction: every
    /// walk target (`topic_echo` / `topic_hz`, both far above) lies in the
    /// surviving prefix. A walk target added BELOW the literal would be silently
    /// invisible. The literal-aware sibling — which also REPORTS unclosed depth so
    /// a truncated view fails loudly — lives in
    /// `tests/convergence_adoption_test.rs`; the two copies are deliberately not
    /// unified because this one is a private `#[cfg(test)]` helper in a `src`
    /// module and that one is an integration test in another target, so sharing
    /// would mean a new test-support crate for ~60 lines. If a third consumer
    /// appears, hoist rather than copy again.
    fn code_only(src: &str) -> String {
        let b = src.as_bytes();
        let mut out = String::with_capacity(src.len());
        let mut i = 0usize;
        let mut depth = 0usize;
        while i < b.len() {
            if depth == 0 && b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'/' {
                while i < b.len() && b[i] != b'\n' {
                    i += 1;
                }
                continue;
            }
            if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
                depth += 1;
                i += 2;
                continue;
            }
            if depth > 0 && b[i] == b'*' && i + 1 < b.len() && b[i + 1] == b'/' {
                depth -= 1;
                i += 2;
                continue;
            }
            if depth == 0 {
                out.push(b[i] as char);
            } else if b[i] == b'\n' {
                // Keep line structure so line-oriented callers stay aligned.
                out.push('\n');
            }
            i += 1;
        }
        out
    }

    /// Extract the balanced `{ … }` block that follows the FIRST
    /// occurrence of `needle` in `hay`. Used to scope an assertion to the inside
    /// of a specific `if let` arm rather than to a whole function body.
    fn block_after(hay: &str, needle: &str) -> String {
        let at = hay
            .find(needle)
            .unwrap_or_else(|| panic!("needle not found: {needle}"));
        let b = hay.as_bytes();
        let open = hay[at..]
            .find('{')
            .map(|i| at + i)
            .unwrap_or_else(|| panic!("no opening brace after: {needle}"));
        let mut depth = 0usize;
        let mut i = open;
        while i < b.len() {
            match b[i] {
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        return hay[open..=i].to_string();
                    }
                }
                _ => {}
            }
            i += 1;
        }
        panic!("unbalanced braces after: {needle}");
    }

    #[test]
    fn code_only_strips_both_comment_syntaxes_and_nothing_else() {
        assert_eq!(
            code_only("let a = 1; // note\nlet b = 2;\n").trim(),
            "let a = 1; \nlet b = 2;".trim()
        );
        assert_eq!(code_only("a /* x */ b"), "a  b");
        // Nesting: the inner pair must not close the outer one.
        assert_eq!(code_only("a /* x /* y */ z */ b"), "a  b");
        // A block opener inside a LINE comment is not a block opener.
        assert_eq!(code_only("a // /* not a block\nb").trim(), "a \nb".trim());
        // Unterminated block fails CLOSED (everything after is stripped).
        assert_eq!(code_only("a /* unterminated b"), "a ");
    }

    /// Structural pin: the `topic_echo` / `topic_hz` observer loops
    /// MUST be event-driven (`wait_for_message`) — NO `try_receive` + sleep
    /// poll loop may survive in either. `topic_info` (which has always been
    /// event-driven) is the anti-tautology control proving the extractor
    /// finds real bodies and the `wait_for_message` assertion can fire.
    #[test]
    fn echo_and_hz_are_event_driven_not_polling() {
        // Comment-stripped, so the positive controls below cannot be
        // satisfied by prose. The ALLOW marker scan further down
        // deliberately uses RAW lines — the marker IS a comment by construction.
        let src = code_only(TOPIC_CMD_SRC);
        let echo = fn_body(&src, "pub fn topic_echo(").to_string();
        let hz = fn_body(&src, "pub fn topic_hz(").to_string();
        let info = fn_body(&src, "pub fn topic_info(").to_string();
        let raw_echo = fn_body(TOPIC_CMD_SRC, "pub fn topic_echo(");
        let raw_hz = fn_body(TOPIC_CMD_SRC, "pub fn topic_hz(");

        // Extractor sanity: real, non-trivial bodies (a broken extractor
        // returning "{}" must not vacuously pass the negative assertions).
        for (name, body) in [("topic_echo", &echo), ("topic_hz", &hz)] {
            assert!(
                body.len() > 200,
                "{name} body implausibly short ({} bytes) — extractor broken",
                body.len()
            );
        }

        // Positive: both observer loops block on the event listener.
        assert!(
            echo.contains("wait_for_message"),
            "topic_echo must block on wait_for_message (event-driven)"
        );
        assert!(
            hz.contains("wait_for_message"),
            "topic_hz must block on wait_for_message (event-driven)"
        );
        // Control: the extractor genuinely detects wait_for_message usage.
        assert!(
            info.contains("wait_for_message"),
            "extractor control: topic_info has always used wait_for_message"
        );

        // Negative (targeted arm): no polling READ remains in either loop.
        for (name, body) in [("topic_echo", &echo), ("topic_hz", &hz)] {
            assert!(
                !body.contains("try_receive"),
                "{name} still calls try_receive — the poll loop was not removed"
            );
        }

        // The sleep half is narrowed from a BLANKET ban to "no UNMARKED
        // sleep". The blanket ban was the right rule while the only reason to
        // sleep was a poll; there is now a second, genuinely different reason:
        // bounding an EXTERNAL event source that churns the wait with nothing to
        // deliver, which no amount of event-driven waiting fixes from inside the
        // loop. An unmarked sleep is still the poll this test was written to
        // kill and still fails here; the marked one must additionally be GATED
        // and POSITIONED, which
        // `the_observer_starvation_floors_are_gated_by_the_pacing_decision`
        // enforces.
        //
        // This arm reads RAW source deliberately: the exempting marker IS a
        // comment, so a comment-stripped view would delete the very token that
        // distinguishes a justified floor from a poll.
        for (name, body) in [("topic_echo", raw_echo), ("topic_hz", raw_hz)] {
            for (i, line) in body.lines().enumerate() {
                assert!(
                    !line.contains("thread::sleep") || line.contains("ALLOW:"),
                    "{name} line {} sleeps WITHOUT a justified ALLOW marker — a \
                     poll loop survives (must block on the listener): {line}",
                    i + 1
                );
            }
        }
    }

    /// Whole-file anti-poll pin: the targeted per-fn
    /// arm above is evadable by a helper-fn refactor (it brace-extracts only
    /// the two named bodies — a `fn poll_helper()` called from echo would
    /// pass it). After the event-driven rewrite there are ZERO legitimate `try_receive` or
    /// `thread::sleep` sites anywhere in this file's PRODUCTION region, so
    /// scan ALL of it — every line up to the `#[cfg(test)]` module marker
    /// (the test module itself must be excluded: this very test names both
    /// tokens in its assertions and would self-match). No allowlist: nothing
    /// in production topic_cmd.rs legitimately polls IN AN OBSERVER LOOP; a
    /// line-anchored ALLOWLIST (an `ALLOW:` marker comment ON THE SAME
    /// LINE, with a per-entry justification) exempts a genuinely-needed sleep
    /// that is NOT an observer poll. One exists today: a one-time bounded
    /// retry opening the mirror `cerulion-netd` (now a SEPARATE process) just
    /// created, covering the brief cross-process service-visibility window.
    #[test]
    fn whole_file_has_no_polling_primitives_outside_tests() {
        let marker = "#[cfg(test)]";
        let tests_at = TOPIC_CMD_SRC
            .find(marker)
            .expect("test-module marker must exist (this test lives under it)");
        let production = &TOPIC_CMD_SRC[..tests_at];

        // Extractor sanity: the production region is the bulk of the file
        // (a mis-anchored marker yielding a near-empty region must not pass
        // the negative assertions vacuously).
        assert!(
            production.len() > 5_000,
            "production region implausibly small ({} bytes) — marker mis-anchored",
            production.len()
        );
        // Control: the region genuinely contains the event-driven calls, so
        // token scanning over it works. Asserted over a COMMENT-
        // STRIPPED view — this file's prose names `wait_for_message` many
        // times, so a raw-source control could be satisfied entirely by
        // comments, making it no control at all.
        assert!(
            code_only(production).contains("wait_for_message"),
            "control: the production region must contain the wait_for_message CALLS \
             (not merely mention them in comments)"
        );

        let mut allow_markers = 0usize;
        for (line_no, line) in production.lines().enumerate() {
            let allowed = line.contains("ALLOW:");
            if allowed {
                allow_markers += 1;
            }
            // `try_receive` is NEVER a legitimate setup primitive — a poll read is
            // enforced UNCONDITIONALLY, even on an ALLOW-marked line.
            assert!(
                !line.contains("try_receive"),
                "topic_cmd.rs:{}: `try_receive` in the production region — the anti-poll rule \
                 forbids polling reads here (event-driven wait_for_message only; \
                 an ALLOW marker never exempts a poll read): {line}",
                line_no + 1
            );
            // An ALLOW marker exempts ONLY the `thread::sleep` assert — a
            // genuinely-needed non-observer-loop setup sleep (the
            // cross-process mirror-open retry).
            assert!(
                allowed || !line.contains("thread::sleep"),
                "topic_cmd.rs:{}: `thread::sleep` in the production region — the anti-poll rule \
                 forbids poll sleeps here (block on the listener instead; or add a \
                 justified ALLOW marker for a non-observer-loop setup sleep): {line}",
                line_no + 1
            );
        }
        // EXACTLY THREE production ALLOW markers. Any new marker forces a
        // DELIBERATE test edit — the allowlist never grows silently.
        //
        //  1. The one-time bounded mirror-open retry at SETUP.
        //  2. `topic_echo`'s starvation floor.
        //  3. `topic_hz`'s starvation floor.
        //
        // The two starvation-floor markers are a deliberate carve-out,
        // and they are NOT observer polls: each is reached only through
        // `decide_observer_pacing`, which returns `None` for every iteration that
        // delivered a frame OR whose wait spent its budget — i.e. for the entire
        // healthy operating range the anti-poll rule cares about. What they bound is an
        // EXTERNAL event source churning the wait with nothing to deliver, which
        // no amount of event-driven waiting can fix from inside the loop.
        assert_eq!(
            allow_markers, 3,
            "expected exactly THREE ALLOW production markers (the \
             mirror-open retry + the two observer starvation floors); a new one \
             must be justified + this count bumped deliberately"
        );
    }

    /// Carve-out guard: the two starvation-floor sleeps are permitted by
    /// the marker above ONLY because they are GATED by
    /// [`decide_observer_pacing`]. The marker count alone is not a guard — an
    /// abusive sleep carries the same marker — so this is the arm that must die
    /// when the carve-out is abused.
    ///
    /// It pins the sleep's POSITION, which is the only thing a source walk can
    /// check that a shadowing binding cannot fake. Two abuse variants
    /// shape what it asserts:
    ///
    /// 1. Keep the call, discard the verdict, `sleep(OBSERVER_PACING_FLOOR)`.
    ///    Passes a name-presence + ordering assertion. Killed by requiring the
    ///    sleep to consume a binding named `pause`.
    /// 2. `if let Some(pause) = decide_observer_pacing(..) { let _ = pause; }`
    ///    followed by a SEPARATE block `{ let pause = OBSERVER_PACING_FLOOR;
    ///    thread::sleep(pause); }`. Passes (1)'s requirement — the tokens are all
    ///    present — while sleeping unconditionally on EVERY iteration including
    ///    a flowing topic, violating the latency contract outright.
    ///
    /// So the sleep must live INSIDE the brace-extracted `if let` arm, and it
    /// must be the ONLY sleep in the body — a second one outside the arm is
    /// exactly variant 2.
    #[test]
    fn the_observer_starvation_floors_are_gated_by_the_pacing_decision() {
        // Comment-stripped: these bodies DISCUSS `thread::sleep` in the comment
        // justifying the marker, so a raw-source walk could satisfy the
        // anti-vacuity assert from prose alone.
        let src = code_only(TOPIC_CMD_SRC);
        let echo = fn_body(&src, "pub fn topic_echo(").to_string();
        let hz = fn_body(&src, "pub fn topic_hz(").to_string();

        for (name, body) in [("topic_echo", echo), ("topic_hz", hz)] {
            // Anti-vacuity: the floor exists at all.
            assert!(
                body.contains("thread::sleep"),
                "{name} must carry the starvation floor"
            );
            // EXACTLY ONE sleep in the body. A second one is variant 2's shape:
            // a guarded arm that does nothing beside an unguarded sleep.
            assert_eq!(
                body.matches("thread::sleep").count(),
                1,
                "{name} must contain EXACTLY ONE `thread::sleep` — a second one \
                 outside the guarded arm is an unconditional observer poll"
            );
            // The guard is written in the one shape that makes the marker
            // true: the decision BINDS the pause. Matched over a
            // WHITESPACE-NORMALIZED view because rustfmt wraps the two call
            // sites differently (hz's arguments fit on one line, echo's do not),
            // so a literal multi-line match would pin the FORMATTER.
            let flat = body.split_whitespace().collect::<Vec<_>>().join(" ");
            assert!(
                flat.contains("if let Some(pause) = decide_observer_pacing("),
                "{name} must GUARD its sleep with `if let Some(pause) = \
                 decide_observer_pacing(..)` — calling the decision and ignoring \
                 its verdict is an UNCONDITIONAL observer poll wearing the \
                 ALLOW marker"
            );
            // THE POSITION PIN: the sleep is INSIDE that arm's braces, and it
            // sleeps the arm's own binding. Scoping to the extracted block is
            // what rules out variant 2 — the tokens exist in its body too, just not
            // in the same block.
            let guarded = block_after(&flat, "if let Some(pause) = decide_observer_pacing(");
            assert!(
                guarded.contains("thread::sleep(pause)"),
                "{name}'s `thread::sleep(pause)` must live INSIDE the \
                 `if let Some(pause) = decide_observer_pacing(..)` arm; a sleep \
                 outside it runs on EVERY iteration, including on a flowing \
                 topic. Guarded block was: {guarded}"
            );
            // And the engagement counter is bumped in the same arm, so the e2e
            // observable cannot be advanced by anything but a real pacing.
            assert!(
                guarded.contains("note_pacing_engaged()"),
                "{name} must record the engagement inside the guarded arm — a \
                 `pacing_engaged_count` assertion is the only thing that catches \
                 a call site whose churn budget is neutered, and BOTH observer \
                 flood arms in `topic_observer_iox2_test` make one. Wiring the \
                 counter is not enough on its own: an arm that does not OBSERVE \
                 it leaves its own loop inert-able (measured — echo's arm was \
                 added only after the hz-only version let the echo-side mutant \
                 through with all 12 tests green)."
            );
        }
    }

    // ---- `decide_observer_pacing` + `silence_notice_due` oracle
    // vectors. Both are PURE, so the contract is pinned deterministically
    // against hand-written expectations rather than against a CPU-percentage
    // measurement, which on a loaded machine fails OPEN (a starved spinner uses
    // less CPU, so a "< N% of a core" assertion PASSES for the very regression
    // it exists to catch). ----

    /// A FLOWING topic is never paced, whatever the wait cost and whatever the
    /// churn budget had reached. This is the latency contract: the starvation
    /// floor must never sit between a frame and its display.
    #[test]
    fn pacing_never_delays_a_topic_that_delivered_frames() {
        for start in [
            0u32,
            1,
            OBSERVER_CHURN_TOLERANCE - 1,
            OBSERVER_CHURN_TOLERANCE + 99,
        ] {
            for delivered in [1usize, 2, 4096] {
                for waited_ms in [0u64, 1, 999, 1000, 5000] {
                    let mut churn = start;
                    assert_eq!(
                        decide_observer_pacing(
                            delivered,
                            Duration::from_millis(waited_ms),
                            Duration::from_millis(1000),
                            &mut churn,
                        ),
                        None,
                        "a delivered frame must never be paced (delivered={delivered}, \
                         waited={waited_ms}ms, churn started at {start})"
                    );
                    assert_eq!(
                        churn, 0,
                        "a delivered frame must RESET the churn budget (started at {start})"
                    );
                }
            }
        }
    }

    /// The ordinary SILENT topic — the wait blocks for its whole budget and
    /// returns nothing. That is the loop's healthy shape (~one wake per second),
    /// NOT churn: it must never be paced and must never accumulate budget.
    /// The `waited == requested` boundary is pinned as SPENT (inclusive).
    #[test]
    fn pacing_never_engages_when_the_wait_spent_its_budget() {
        for waited_ms in [1000u64, 1001, 2000] {
            let mut churn = OBSERVER_CHURN_TOLERANCE + 5; // pre-loaded: must still reset
            assert_eq!(
                decide_observer_pacing(
                    0,
                    Duration::from_millis(waited_ms),
                    Duration::from_millis(1000),
                    &mut churn,
                ),
                None,
                "a wait that spent its {waited_ms}ms budget is a silent topic, not churn"
            );
            assert_eq!(
                churn, 0,
                "a budget-spending wait must RESET the churn count"
            );
        }
    }

    /// The regression itself: progress-free early returns accumulate,
    /// are tolerated for exactly `OBSERVER_CHURN_TOLERANCE - 1` iterations, and
    /// are paced from the `OBSERVER_CHURN_TOLERANCE`-th onwards. Driven as ONE
    /// hand-written decision VECTOR so the threshold is pinned on both sides
    /// (an off-by-one in either direction fails).
    #[test]
    fn pacing_engages_exactly_at_the_churn_tolerance_and_stays_engaged() {
        let mut churn = 0u32;
        let requested = Duration::from_millis(1000);
        let early = Duration::from_micros(3); // returned essentially instantly

        // Hand oracle: the first TOLERANCE-1 churning iterations are tolerated…
        for i in 1..OBSERVER_CHURN_TOLERANCE {
            assert_eq!(
                decide_observer_pacing(0, early, requested, &mut churn),
                None,
                "churn iteration {i} of {OBSERVER_CHURN_TOLERANCE} must still be tolerated"
            );
            assert_eq!(churn, i, "churn budget must count every churning iteration");
        }
        // …the TOLERANCE-th engages the floor…
        assert_eq!(
            decide_observer_pacing(0, early, requested, &mut churn),
            Some(OBSERVER_PACING_FLOOR),
            "the {OBSERVER_CHURN_TOLERANCE}-th consecutive churning iteration must pace"
        );
        // …and it STAYS engaged while the churn continues (a flood does not get
        // a fresh tolerance window every time the floor fires).
        for _ in 0..50 {
            assert_eq!(
                decide_observer_pacing(0, early, requested, &mut churn),
                Some(OBSERVER_PACING_FLOOR),
                "pacing must persist while the churn does"
            );
        }

        // A single delivered frame breaks the regime, and the full tolerance is
        // then available again (so a burst of real data is never throttled).
        assert_eq!(
            decide_observer_pacing(1, early, requested, &mut churn),
            None
        );
        assert_eq!(churn, 0);
        assert_eq!(
            decide_observer_pacing(0, early, requested, &mut churn),
            None,
            "after a delivered frame the tolerance window must start over"
        );
    }

    /// The churn counter must not wrap under a long flood (a wrapped counter
    /// would silently DISENGAGE the floor and re-open the spin).
    #[test]
    fn pacing_churn_budget_saturates_instead_of_wrapping() {
        let mut churn = u32::MAX;
        assert_eq!(
            decide_observer_pacing(
                0,
                Duration::from_micros(1),
                Duration::from_millis(1000),
                &mut churn,
            ),
            Some(OBSERVER_PACING_FLOOR)
        );
        assert_eq!(
            churn,
            u32::MAX,
            "the churn budget must saturate, never wrap"
        );
    }

    /// The silence-notice schedule: FIRST, then FIRST + k*INTERVAL. Pinned on
    /// BOTH sides of every boundary so neither a too-eager nor a too-lazy
    /// schedule passes.
    #[test]
    fn silence_notice_schedule_is_pinned_on_both_sides_of_each_boundary() {
        // Nothing is due before the first deadline…
        assert!(!silence_notice_due(Duration::ZERO, 0));
        assert!(!silence_notice_due(
            OBSERVER_FIRST_SILENCE_NOTICE - Duration::from_millis(1),
            0
        ));
        // …and the first deadline is INCLUSIVE.
        assert!(silence_notice_due(OBSERVER_FIRST_SILENCE_NOTICE, 0));

        // With one notice already emitted the next is due a full INTERVAL later,
        // not immediately (the flood shape).
        assert!(!silence_notice_due(OBSERVER_FIRST_SILENCE_NOTICE, 1));
        assert!(!silence_notice_due(
            OBSERVER_FIRST_SILENCE_NOTICE + OBSERVER_SILENCE_NOTICE_INTERVAL
                - Duration::from_millis(1),
            1
        ));
        assert!(silence_notice_due(
            OBSERVER_FIRST_SILENCE_NOTICE + OBSERVER_SILENCE_NOTICE_INTERVAL,
            1
        ));

        // …and the schedule keeps stepping by exactly INTERVAL.
        for k in 2..6u32 {
            let due = OBSERVER_FIRST_SILENCE_NOTICE + OBSERVER_SILENCE_NOTICE_INTERVAL * k;
            assert!(
                !silence_notice_due(due - Duration::from_millis(1), k),
                "notice {k} must not be due early"
            );
            assert!(
                silence_notice_due(due, k),
                "notice {k} must be due at {due:?}"
            );
        }
    }

    /// A silence notice must NAME what is known and nothing else: the topic,
    /// where its frames should come from, that zero arrived, and over how long.
    /// The remote arm's robot name is LAN-sourced, so it is sanitized.
    #[test]
    fn silence_notice_names_the_topic_the_source_and_the_elapsed_silence() {
        let local = observer_source_description(&TopicSource::Local);
        let line = silence_notice("/lowstate", &local, Duration::from_secs(7));
        assert!(line.contains("/lowstate"), "must name the topic: {line}");
        // DECODABLE, not just "frames": both observers count only frames that
        // survive the wire filter, so a malformed-frame storm leaves this at
        // zero while frames really are crossing. "0 frames" would be an
        // affirmatively wrong claim about the network.
        assert!(
            line.contains("0 decodable frames in 7s"),
            "must state the observation, scoped to DECODABLE frames: {line}"
        );
        assert!(
            line.contains("local producer"),
            "must name the source: {line}"
        );

        // The remote arm names the robot AND the daemon the mirror comes from —
        // information a bare source label would leave out.
        let remote = observer_source_description(&TopicSource::Remote {
            robot: "ubu\u{1b}[31mntu".to_string(),
            walker: cerulion_core::codegen::FrameWalker::new(Vec::new()).0,
        });
        assert!(
            remote.contains("cerulion-netd"),
            "the remote source must name the mirror's daemon: {remote}"
        );
        assert!(
            !remote.contains('\u{1b}'),
            "a LAN-sourced robot name must be terminal-escape sanitized: {remote}"
        );
    }

    // ---- `decide_wait_outcome` oracle vectors (the deterministic
    // pin for the SIGINT-race fix — the subprocess e2e tests exercise the
    // real signal path but land on whichever side of the race a given run
    // happens to hit; these vectors pin the decision table exactly). ----

    #[test]
    fn wait_outcome_ok_resets_budget_and_continues() {
        for start in [
            0u32,
            1,
            MAX_CONSECUTIVE_WAIT_ERRORS - 1,
            MAX_CONSECUTIVE_WAIT_ERRORS + 7,
        ] {
            for running in [true, false] {
                let mut budget = start;
                assert_eq!(
                    decide_wait_outcome(true, running, &mut budget),
                    WaitOutcomeAction::Continue,
                    "an Ok wait always continues (running={running}, start={start}) — \
                     shutdown is the loop condition's job, not the decision table's"
                );
                assert_eq!(budget, 0, "an Ok wait must reset the budget from {start}");
            }
        }
    }

    #[test]
    fn wait_outcome_err_during_shutdown_exits_cleanly_at_any_budget() {
        // The headline: a wait interrupted by SIGINT during shutdown
        // is NOT a failure — exit cleanly (exit code 0), no matter how many
        // errors preceded it, without touching the budget.
        for start in [
            0u32,
            1,
            MAX_CONSECUTIVE_WAIT_ERRORS - 1,
            MAX_CONSECUTIVE_WAIT_ERRORS,
            MAX_CONSECUTIVE_WAIT_ERRORS + 3,
        ] {
            let mut budget = start;
            assert_eq!(
                decide_wait_outcome(false, false, &mut budget),
                WaitOutcomeAction::ExitCleanly,
                "Err + shutdown must exit cleanly (start={start})"
            );
            assert_eq!(budget, start, "the shutdown exit must not touch the budget");
        }
    }

    #[test]
    fn wait_outcome_err_while_running_retries_then_fails_at_exact_budget() {
        // Hand oracle for MAX_CONSECUTIVE_WAIT_ERRORS = 5: consecutive
        // errors 1..=4 retry (each re-entering the same blocking wait — the
        // count-bounded, sleep-free anti-poll-compatible shape), the 5th
        // consecutive error fails the command.
        let mut budget = 0u32;
        let actions: Vec<WaitOutcomeAction> = (0..MAX_CONSECUTIVE_WAIT_ERRORS)
            .map(|_| decide_wait_outcome(false, true, &mut budget))
            .collect();
        let mut expected =
            vec![WaitOutcomeAction::Continue; (MAX_CONSECUTIVE_WAIT_ERRORS - 1) as usize];
        expected.push(WaitOutcomeAction::Fail);
        assert_eq!(actions, expected, "retry exactly budget-1 times, then fail");
        assert_eq!(budget, MAX_CONSECUTIVE_WAIT_ERRORS);
    }

    #[test]
    fn wait_outcome_success_between_errors_grants_a_fresh_budget() {
        // The deliberate reset-on-Ok contract (intentional, not
        // a leak): the budget guards CONSECUTIVE failures only — a receive
        // path that keeps recovering never accumulates toward Fail. A
        // genuinely dead path cannot produce the interleaved Ok, so it still
        // exhausts the budget and fails via the vector above.
        let mut budget = 0u32;
        for round in 0..3 {
            for _ in 0..MAX_CONSECUTIVE_WAIT_ERRORS - 1 {
                assert_eq!(
                    decide_wait_outcome(false, true, &mut budget),
                    WaitOutcomeAction::Continue,
                    "within-budget errors must retry (round {round})"
                );
            }
            assert_eq!(
                decide_wait_outcome(true, true, &mut budget),
                WaitOutcomeAction::Continue
            );
            assert_eq!(budget, 0, "the interleaved Ok must grant a fresh budget");
        }
    }

    #[test]
    fn strip_is_exactly_once_and_skips_empty() {
        // The repeated-strip bug: a topic ending in /data must keep it.
        assert_eq!(
            data_topic_of_service("/p/ticker/data/data"),
            Some("/p/ticker/data")
        );
        assert_eq!(
            data_topic_of_service("/p/cam/image/data"),
            Some("/p/cam/image")
        );
        // Event services and non-data names are filtered.
        assert_eq!(data_topic_of_service("/p/cam/image/event"), None);
        // A foreign service named exactly "/data" must not produce a
        // phantom empty-name topic row.
        assert_eq!(data_topic_of_service("/data"), None);
    }

    /// Control-plane inertness: `cerulion topic list` enumerates
    /// `{topic}/data` services and strips the `/data` suffix. The runtime-
    /// registration control channel (`/__cerulion/gateway_topics`) carries NO
    /// `/data` suffix, so the LOCAL enumeration function itself maps it to `None` —
    /// it can never surface as a listable topic. The integration-level twin of the
    /// `reg_channel_service_name_has_no_data_suffix` unit pin: this exercises the
    /// REAL `data_topic_of_service` filter against the REAL service-name constant,
    /// so a future rename that added a `/data` suffix would fail here.
    #[test]
    fn control_channel_service_is_invisible_to_topic_list() {
        assert_eq!(
            data_topic_of_service(cerulion_core::transport::reg_channel::REG_CHANNEL_SERVICE_NAME),
            None,
            "the runtime-registration control channel must never surface in `topic list`"
        );
    }

    // ---- `topic list --network` pure halves (rendering +
    // normalization + the loud error arm). The LIVE discovery half (real
    // zenoh sessions over loopback TCP) lives in
    // `tests/topic_network_live_test.rs` — kept OUT of this transport-free
    // unit module. ----

    /// Build a topics-only `RemoteDiscovery` (no robots/peers) for the
    /// backward-compatible rendering oracles.
    fn topics_only(topics: &[&str]) -> RemoteDiscovery {
        RemoteDiscovery {
            topics: topics.iter().map(|s| s.to_string()).collect(),
            robots: vec![],
            peers: vec![],
            announce_entries: vec![],
        }
    }

    /// Populated arm: exact output oracle (header + one topic per line). With no
    /// robots/peers the output is byte-identical to the earlier topics-only render.
    #[test]
    fn remote_section_renders_topics_one_per_line() {
        let disc = topics_only(&["/go2/utlidar/cloud", "/mac/planner/cmd_vel"]);
        assert_eq!(
            render_remote_topics_section(&disc, true),
            "\nREMOTE TOPICS\n/go2/utlidar/cloud\n/mac/planner/cmd_vel\n"
        );
    }

    // ---- Mirror-provenance folding (pure partition + render) ----

    fn info(names: &[&str]) -> Vec<TopicInfo> {
        names
            .iter()
            .map(|n| TopicInfo {
                name: n.to_string(),
            })
            .collect()
    }

    fn mirror_map(pairs: &[(&str, &str)]) -> BTreeMap<String, String> {
        pairs
            .iter()
            .map(|(t, r)| (t.to_string(), r.to_string()))
            .collect()
    }

    /// Genuine-local topics (absent from the registry) stay LOCAL; a mirrored one
    /// folds to a streaming row attributed to its robot. Hand oracle.
    #[test]
    fn partition_splits_genuine_local_from_mirrored() {
        let local = info(&["/planner/cmd_vel", "/utlidar/robot_odom", "/imu/data"]);
        let mirrors = mirror_map(&[("/utlidar/robot_odom", "ubuntu")]);
        let (genuine, streaming) = partition_local_topics(local, &mirrors);
        assert_eq!(
            genuine.iter().map(|t| t.name.as_str()).collect::<Vec<_>>(),
            vec!["/planner/cmd_vel", "/imu/data"],
            "genuine-local topics keep their order and stay local"
        );
        assert_eq!(
            streaming,
            vec![MirrorStreamRow {
                topic: "/utlidar/robot_odom".to_string(),
                robot: "ubuntu".to_string(),
            }]
        );
    }

    /// An EMPTY registry is the identity partition — every topic stays local, zero
    /// streaming rows (the registry-less / no-mirror desk). Hand oracle.
    #[test]
    fn partition_with_no_mirrors_is_identity() {
        let local = info(&["/a", "/b", "/c"]);
        let (genuine, streaming) = partition_local_topics(local, &BTreeMap::new());
        assert_eq!(
            genuine.iter().map(|t| t.name.as_str()).collect::<Vec<_>>(),
            vec!["/a", "/b", "/c"]
        );
        assert!(streaming.is_empty());
    }

    /// With NO streaming rows the mirror-aware renderer is BYTE-IDENTICAL to the
    /// mirror-less render — both the populated arm and the empty-window arm. Hand oracle.
    #[test]
    fn render_with_mirrors_empty_is_byte_identical_to_pre_c0() {
        let disc = topics_only(&["/a", "/b"]);
        assert_eq!(
            render_remote_topics_section_with_mirrors(&disc, true, &[]),
            render_remote_topics_section(&disc, true),
        );
        let empty = RemoteDiscovery::empty();
        assert_eq!(
            render_remote_topics_section_with_mirrors(&empty, false, &[]),
            render_remote_topics_section(&empty, false),
        );
    }

    /// A mirrored topic renders under REMOTE TOPICS with the `● streaming` marker,
    /// attributed to its robot; the topic PATH is the row's FIRST token (Studio
    /// parser compat). Rendered over an empty disc (the --no-network path). Hand
    /// oracle.
    #[test]
    fn render_folds_streaming_row_attributed_to_robot() {
        let streaming = vec![MirrorStreamRow {
            topic: "/utlidar/robot_odom".to_string(),
            robot: "ubuntu".to_string(),
        }];
        assert_eq!(
            render_remote_topics_section_with_mirrors(&RemoteDiscovery::empty(), false, &streaming),
            "\nREMOTE TOPICS\n/utlidar/robot_odom  ● streaming  ubuntu\n"
        );
    }

    /// A topic that is BOTH network-announced (idle) AND locally mirrored renders
    /// ONCE, as streaming (streaming wins the dedup); a sibling announce-only topic
    /// stays a bare idle row. Rows sorted by canonical topic. Hand oracle.
    #[test]
    fn render_dedups_announced_and_mirrored_to_one_streaming_row() {
        let disc = topics_only(&["/tf", "/utlidar/robot_odom"]);
        let streaming = vec![MirrorStreamRow {
            topic: "/utlidar/robot_odom".to_string(),
            robot: "ubuntu".to_string(),
        }];
        assert_eq!(
            render_remote_topics_section_with_mirrors(&disc, true, &streaming),
            "\nREMOTE TOPICS\n/tf\n/utlidar/robot_odom  ● streaming  ubuntu\n",
            "the mirrored topic appears once (streaming); /tf stays a bare idle row"
        );
    }

    /// A HOSTILE robot name (ANSI erase-line escape) is terminal-escape-sanitized
    /// at the render seam: the control bytes become
    /// U+FFFD, the topic path is untouched. Hand oracle.
    #[test]
    fn render_sanitizes_hostile_robot_name() {
        let streaming = vec![MirrorStreamRow {
            topic: "/pwned".to_string(),
            robot: "evil\u{1b}[2Krobot".to_string(),
        }];
        assert_eq!(
            render_remote_topics_section_with_mirrors(&RemoteDiscovery::empty(), false, &streaming),
            "\nREMOTE TOPICS\n/pwned  ● streaming  evil\u{FFFD}[2Krobot\n",
            "the ESC control byte is neutered to U+FFFD; the printable '[2K' + the topic path are intact"
        );
    }

    /// No robots + no peers ⇒ NO `ROBOTS` header — exactly today's
    /// topics-only output (byte-identical back-compat).
    #[test]
    fn empty_robots_and_peers_renders_exactly_todays_output() {
        assert_eq!(
            render_remote_topics_section(&topics_only(&["/a", "/b"]), true),
            "\nREMOTE TOPICS\n/a\n/b\n"
        );
    }

    /// Build an mDNS-rung `DiscoveredPeer` for the row-model oracles.
    fn mdns_peer(robot: &str, locator: &str) -> DiscoveredPeer {
        DiscoveredPeer {
            robot: robot.to_string(),
            locator: locator.to_string(),
            rung: DiscoveryRung::Mdns,
        }
    }

    /// An announce-only ROBOTS row (a live gateway whose produced topics
    /// arrived — no locator known) renders `<robot>  <n> topics  (announce)`
    /// BEFORE the topics — exact hand oracle.
    #[test]
    fn robots_section_renders_announce_presence_before_topics() {
        let disc = RemoteDiscovery {
            topics: vec!["/go2/imu".to_string()],
            robots: vec![RobotRow {
                robot: "robot".to_string(),
                locator: None,
                provenance: RobotProvenance::Announce,
                topic_count: 2,
            }],
            peers: vec![],
            announce_entries: vec![],
        };
        assert_eq!(
            render_remote_topics_section(&disc, true),
            "\nROBOTS\n  robot  2 topics  (announce)\n\nREMOTE TOPICS\n/go2/imu\n"
        );
    }

    /// An mDNS-only ROBOTS row (a live gateway that announced no topic)
    /// renders `<robot>  @ <locator>  (mdns)` — exact hand oracle.
    #[test]
    fn robots_section_renders_mdns_only_row() {
        let disc = RemoteDiscovery {
            topics: vec![],
            robots: vec![RobotRow {
                robot: "robot-x".to_string(),
                locator: Some("tcp/192.168.1.9:7683".to_string()),
                provenance: RobotProvenance::Rung(DiscoveryRung::Mdns),
                topic_count: 0,
            }],
            peers: vec![mdns_peer("robot-x", "tcp/192.168.1.9:7683")],
            announce_entries: vec![],
        };
        let out = render_remote_topics_section(&disc, false);
        assert_eq!(
            out,
            format!(
                "\nROBOTS\n  robot-x  @ tcp/192.168.1.9:7683  (mdns)\n\n{}",
                reachable_none_line()
            ),
            "an mDNS-found robot with no topic renders its row, a blank line, then the one \
             reachable line"
        );
    }

    /// A robot surfaced by BOTH an announce token AND the mDNS rung is
    /// ONE row — topic count AND locator, `(mdns)` tag — exact hand oracle.
    #[test]
    fn robots_section_merged_announce_and_mdns_is_one_row() {
        let disc = RemoteDiscovery {
            topics: vec![],
            robots: vec![RobotRow {
                robot: "dual".to_string(),
                locator: Some("tcp/10.0.0.5:7683".to_string()),
                provenance: RobotProvenance::Rung(DiscoveryRung::Mdns),
                topic_count: 2,
            }],
            peers: vec![],
            announce_entries: vec![],
        };
        let out = render_remote_topics_section(&disc, false);
        assert!(
            out.contains("\nROBOTS\n  dual  2 topics  @ tcp/10.0.0.5:7683  (mdns)\n"),
            "got: {out}"
        );
        assert_eq!(
            out.matches("dual").count(),
            1,
            "a merged robot must render exactly once; got: {out}"
        );
    }

    /// `sanitize_display` neutralizes terminal
    /// control characters — ESC/CSI, CR, LF, BEL, DEL, and C1 controls all
    /// become U+FFFD; ordinary UTF-8 (incl. non-ASCII robot names) is
    /// untouched. Hand oracles.
    #[test]
    fn sanitize_display_neuters_control_chars_and_preserves_utf8() {
        // ESC-based CSI screen-clear: only the ESC (a C0 control) is replaced.
        assert_eq!(sanitize_display("\u{1b}[2J"), "\u{fffd}[2J");
        // CR overwrite, LF, BEL — each C0 control → U+FFFD.
        assert_eq!(sanitize_display("a\rb"), "a\u{fffd}b");
        assert_eq!(sanitize_display("a\nb"), "a\u{fffd}b");
        assert_eq!(sanitize_display("a\u{07}b"), "a\u{fffd}b");
        // DEL (U+007F) and a C1 control (U+0085 NEL).
        assert_eq!(sanitize_display("a\u{7f}b"), "a\u{fffd}b");
        assert_eq!(sanitize_display("a\u{85}b"), "a\u{fffd}b");
        // Ordinary UTF-8 passes through unchanged, non-ASCII included.
        assert_eq!(sanitize_display("go2-α"), "go2-α");
        assert_eq!(sanitize_display("robot"), "robot");
        assert_eq!(sanitize_display(""), "");
    }

    /// A hostile LAN-supplied robot name — from
    /// EITHER the announce-key robot chunk OR the mDNS TXT record — cannot
    /// inject terminal escapes into the rendered ROBOTS section, and the mDNS
    /// locator is sanitized too. ESC/CR/BEL become U+FFFD; no raw control byte
    /// survives (exact hand oracle). The `1 topic` singular is exercised too.
    #[test]
    fn render_robots_section_sanitizes_hostile_names_from_both_sources() {
        let rows = vec![
            // announce-key robot-chunk source: hostile name, singular count.
            RobotRow {
                robot: "\u{1b}[2Jevil\rrobot".to_string(),
                locator: None,
                provenance: RobotProvenance::Announce,
                topic_count: 1,
            },
            // mDNS TXT source: hostile name AND hostile locator.
            RobotRow {
                robot: "mdns\u{07}evil".to_string(),
                locator: Some("tcp/1.2.3.4\r:7683".to_string()),
                provenance: RobotProvenance::Rung(DiscoveryRung::Mdns),
                topic_count: 0,
            },
        ];
        let out = render_robots_section(&rows, &[]);
        assert_eq!(
            out,
            "\nROBOTS\n  \u{fffd}[2Jevil\u{fffd}robot  1 topic  (announce)\n  \
             mdns\u{fffd}evil  @ tcp/1.2.3.4\u{fffd}:7683  (mdns)\n"
        );
        assert!(
            !out.contains('\u{1b}') && !out.contains('\r') && !out.contains('\u{07}'),
            "no raw control byte may survive into terminal output; got: {out:?}"
        );
    }

    /// Non-mDNS ladder candidates render on
    /// ONE clearly-labeled `candidates (unverified):` line under the ROBOTS
    /// rows — a `--scan` user must SEE the address an open port answered at,
    /// but it must never read as a verified robot row. Hostile candidate
    /// strings are sanitized too. Exact hand oracles.
    #[test]
    fn robots_section_renders_unverified_candidates_line() {
        // A verified row + two non-mDNS candidates (scan + hostname).
        let rows = vec![RobotRow {
            robot: "go2".to_string(),
            locator: None,
            provenance: RobotProvenance::Announce,
            topic_count: 2,
        }];
        let peers = vec![
            DiscoveredPeer {
                robot: "10.0.0.42".to_string(),
                locator: "tcp/10.0.0.42:7683".to_string(),
                rung: DiscoveryRung::Scan,
            },
            DiscoveredPeer {
                robot: "ubuntu".to_string(),
                locator: "tcp/192.168.1.7:7683".to_string(),
                rung: DiscoveryRung::Hostname,
            },
            // An mDNS peer is NOT a candidate (it is row evidence) — excluded.
            mdns_peer("go2", "tcp/10.0.0.5:7683"),
        ];
        assert_eq!(
            render_robots_section(&rows, &peers),
            "\nROBOTS\n  go2  2 topics  (announce)\n  candidates (unverified): \
             10.0.0.42 @ tcp/10.0.0.42:7683 (scan) · ubuntu @ tcp/192.168.1.7:7683 (hostname)\n"
        );

        // Candidates with NO verified rows still render (under the header) —
        // the --scan visibility regression pin.
        let scan_only = vec![DiscoveredPeer {
            robot: "10.0.0.42".to_string(),
            locator: "tcp/10.0.0.42:7683".to_string(),
            rung: DiscoveryRung::Scan,
        }];
        assert_eq!(
            render_robots_section(&[], &scan_only),
            "\nROBOTS\n  candidates (unverified): 10.0.0.42 @ tcp/10.0.0.42:7683 (scan)\n"
        );

        // Hostile candidate strings are sanitized (no raw control bytes).
        let hostile = vec![DiscoveredPeer {
            robot: "ev\u{1b}il".to_string(),
            locator: "tcp/1.2.3.4\r:7683".to_string(),
            rung: DiscoveryRung::Cache,
        }];
        let out = render_robots_section(&[], &hostile);
        assert_eq!(
            out,
            "\nROBOTS\n  candidates (unverified): ev\u{fffd}il @ tcp/1.2.3.4\u{fffd}:7683 (cache)\n"
        );
        assert!(!out.contains('\u{1b}') && !out.contains('\r'));
    }

    /// The ladder found +
    /// connected a peer (shown in ROBOTS) but NO topic answered within the
    /// window — even though the user passed no `--connect`, the correct hint is
    /// the REACHABLE arm ("a reachable peer didn't advertise in time"), NOT the
    /// misleading "go pass --connect" escape. Exact hand oracle.
    #[test]
    fn remote_section_ladder_peers_but_no_topics_uses_reachable_hint() {
        let disc = RemoteDiscovery {
            topics: vec![],
            robots: vec![RobotRow {
                robot: "found-robot".to_string(),
                locator: Some("tcp/10.0.0.9:7683".to_string()),
                provenance: RobotProvenance::Rung(DiscoveryRung::Mdns),
                topic_count: 0,
            }],
            peers: vec![mdns_peer("found-robot", "tcp/10.0.0.9:7683")],
            announce_entries: vec![],
        };
        // had_endpoints == false, but a discovered robot makes us reachable.
        let out = render_remote_topics_section(&disc, false);
        assert_eq!(
            out,
            format!(
                "\nROBOTS\n  found-robot  @ tcp/10.0.0.9:7683  (mdns)\n\n{}",
                reachable_none_line()
            )
        );
        // The KEY pin: a robot is already connected + shown above, so the
        // "go pass --connect" escape must NOT appear.
        assert!(
            !out.contains("--connect tcp/<host>:7683"),
            "a connected robot must not be told to pass --connect; got: {out}"
        );
    }

    /// Build an announce-gather entry tersely.
    fn ann(robot: &str, topic: &str) -> (String, Option<String>) {
        (robot.to_string(), Some(topic.to_string()))
    }

    /// Grouping oracle: announce entries group by their EXACT robot
    /// chunk into `(robot, distinct_topic_count)`. The flagship `ros2 attach`
    /// shape — ABSOLUTE mirror topics (`/lf/lowstate`, `/utlidar/cloud`) under
    /// ONE robot — yields ONE row and NO phantom "lf"/"utlidar" robots (a
    /// first-segment heuristic would mint those). A bare identity token
    /// creates a zero-topic robot; duplicate topics (two /tf publishers) count
    /// ONCE (distinct topics, matching the deduped REMOTE TOPICS). Sorted by
    /// robot name. Hand oracle.
    #[test]
    fn group_announce_by_robot_is_exact_and_counts_distinct_topics() {
        let entries = vec![
            ann("go2", "/lf/lowstate"),   // absolute mirror topic
            ann("go2", "/utlidar/cloud"), // absolute mirror topic
            ann("go2", "/tf"),            // two /tf tokens → ONE distinct
            ann("go2", "/tf"),
            ann("mac", "/mac/planner/cmd_vel"),
            ("bare".to_string(), None), // identity token → 0 topics
        ];
        assert_eq!(
            group_announce_by_robot(&entries),
            vec![
                ("bare".to_string(), 0),
                ("go2".to_string(), 3), // lowstate + cloud + tf (distinct)
                ("mac".to_string(), 1),
            ]
        );
        // No phantom robots from topic segments: neither "lf" nor "utlidar"
        // nor "tf" appears as a robot.
        let robots: Vec<String> = group_announce_by_robot(&entries)
            .into_iter()
            .map(|(r, _)| r)
            .collect();
        for phantom in ["lf", "utlidar", "tf"] {
            assert!(
                !robots.iter().any(|r| r == phantom),
                "phantom robot '{phantom}' minted from a topic segment"
            );
        }
        // Empty input → empty.
        assert_eq!(group_announce_by_robot(&[]), Vec::<(String, usize)>::new());
    }

    /// `build_robot_rows` merge oracle: announce presence seeds rows
    /// (exact robot chunk); an mDNS peer with a matching name ENRICHES its row
    /// (locator + `(mdns)` provenance, topic count kept — ONE row); an mDNS
    /// peer with NO announce gets its OWN row; a bare identity token seeds a
    /// zero-topic row; and a cache/hostname/scan peer creates NO row. Rows are
    /// sorted by robot name. Hand oracle, never a self-compare.
    #[test]
    fn build_robot_rows_merges_announce_and_mdns_and_ignores_other_rungs() {
        let entries = vec![
            // go2 announces two ABSOLUTE mirror topics (the attach shape).
            ann("go2", "/lf/lowstate"),
            ann("go2", "/utlidar/cloud"),
            ann("silent", "/x"),        // 1 topic, no mDNS
            ("idle".to_string(), None), // bare identity token, zero egress
        ];
        let peers = vec![
            mdns_peer("go2", "tcp/10.0.0.5:7683"),    // enriches go2
            mdns_peer("lonely", "tcp/10.0.0.9:7683"), // mDNS-only → own row
            // A cache peer for a robot that did NOT announce → NO row.
            DiscoveredPeer {
                robot: "cached-ghost".to_string(),
                locator: "tcp/10.0.0.7:7683".to_string(),
                rung: DiscoveryRung::Cache,
            },
        ];
        assert_eq!(
            build_robot_rows(&entries, &peers),
            vec![
                RobotRow {
                    robot: "go2".to_string(),
                    locator: Some("tcp/10.0.0.5:7683".to_string()),
                    provenance: RobotProvenance::Rung(DiscoveryRung::Mdns),
                    topic_count: 2,
                },
                RobotRow {
                    robot: "idle".to_string(),
                    locator: None,
                    provenance: RobotProvenance::Announce,
                    topic_count: 0,
                },
                RobotRow {
                    robot: "lonely".to_string(),
                    locator: Some("tcp/10.0.0.9:7683".to_string()),
                    provenance: RobotProvenance::Rung(DiscoveryRung::Mdns),
                    topic_count: 0,
                },
                RobotRow {
                    robot: "silent".to_string(),
                    locator: None,
                    provenance: RobotProvenance::Announce,
                    topic_count: 1,
                },
            ]
        );
    }

    /// Two mDNS peers for one announce robot — the FIRST (the ladder
    /// pre-sorts peers) wins the locator; the row stays single. Hand oracle.
    #[test]
    fn build_robot_rows_first_mdns_locator_wins() {
        let entries = vec![ann("dual", "/x")];
        let peers = vec![
            mdns_peer("dual", "tcp/10.0.0.5:7683"),
            mdns_peer("dual", "tcp/10.0.0.6:7683"),
        ];
        assert_eq!(
            build_robot_rows(&entries, &peers),
            vec![RobotRow {
                robot: "dual".to_string(),
                locator: Some("tcp/10.0.0.5:7683".to_string()),
                provenance: RobotProvenance::Rung(DiscoveryRung::Mdns),
                topic_count: 1,
            }],
        );
    }

    /// A cache/hostname/scan candidate is
    /// NOT reachability evidence — with only a (possibly dead) cached candidate
    /// and no verified rows, the empty-topics hint must keep the `--connect`
    /// escape (a dead cached robot must not suppress it for 7 days of TTL).
    /// The candidate still shows on the labeled unverified line.
    #[test]
    fn remote_section_cache_candidate_alone_is_not_reachable() {
        let disc = RemoteDiscovery {
            topics: vec![],
            robots: vec![],
            peers: vec![DiscoveredPeer {
                robot: "yesterday".to_string(),
                locator: "tcp/10.0.0.8:7683".to_string(),
                rung: DiscoveryRung::Cache,
            }],
            announce_entries: vec![],
        };
        let out = render_remote_topics_section(&disc, false);
        // The unverified-candidates line renders (visibility)…
        assert!(
            out.contains("  candidates (unverified): yesterday @ tcp/10.0.0.8:7683 (cache)\n"),
            "got: {out}"
        );
        // …but the hint is the UNREACHABLE arm: the --connect escape survives.
        assert!(
            out.contains("--connect tcp/<host>:7683"),
            "a mere cache candidate must not suppress the --connect escape; got: {out}"
        );
        // The whole shape, exactly: the candidates line, a blank line, then
        // the one unreachable line (a candidates-only ROBOTS section is
        // separated from the notice like a robot row is).
        assert_eq!(
            out,
            format!(
                "\nROBOTS\n  candidates (unverified): yesterday @ tcp/10.0.0.8:7683 (cache)\n\n{}",
                unreachable_none_line()
            )
        );
    }

    /// `resolve_write_back` oracles — the
    /// pure gate keeping hermetic (scouting-off) runs from ever touching the
    /// developer's real `~/.cerulion/peers.json`. scouting=false ⇒ `None`
    /// (no write, no matter what was found); empty rows ⇒ `None`;
    /// scouting=true + rows ⇒ exactly the RobotRow-derived tuples.
    #[test]
    fn resolve_write_back_gates_on_scouting_and_maps_rows_exactly() {
        let rows = vec![
            RobotRow {
                robot: "go2".to_string(),
                locator: Some("tcp/10.0.0.5:7683".to_string()),
                provenance: RobotProvenance::Rung(DiscoveryRung::Mdns),
                topic_count: 2,
            },
            RobotRow {
                robot: "silent".to_string(),
                locator: None,
                provenance: RobotProvenance::Announce,
                topic_count: 1,
            },
        ];
        // Hermetic (scouting off): NEVER a write — even with confirmed rows.
        assert_eq!(resolve_write_back(false, &rows), None);
        // Nothing confirmed: no write.
        assert_eq!(resolve_write_back(true, &[]), None);
        // Scouting + rows: exactly the (robot, locator) tuples, in row order.
        assert_eq!(
            resolve_write_back(true, &rows),
            Some(vec![
                ("go2".to_string(), Some("tcp/10.0.0.5:7683".to_string())),
                ("silent".to_string(), None),
            ])
        );
    }

    /// The exact one-line REACHABLE arm (a given locator, a discovered robot, or
    /// an mDNS answer, and no topic within the window). Hand oracle, spelled
    /// out so a renderer change is a visible diff here, not a self-compare.
    fn reachable_none_line() -> String {
        format!(
            "remote: none discovered in {} ms (a reachable peer advertised no topic in \
             time; retry)\n",
            REMOTE_QUERY_GATHER_WINDOW.as_millis()
        )
    }

    /// The exact one-line UNREACHABLE arm (nothing given, nothing found): the
    /// `--connect` escape for a robot scouting cannot reach. Hand oracle.
    fn unreachable_none_line() -> String {
        format!(
            "remote: none discovered in {} ms (a robot off the LAN needs \
             --connect tcp/<host>:7683)\n",
            REMOTE_QUERY_GATHER_WINDOW.as_millis()
        )
    }

    /// Empty + endpoints given: ONE line (no `REMOTE TOPICS` header), explicit
    /// about the BOUNDED gather window (an empty result is "nobody answered
    /// within N ms", never a definitive "no robot exists"), naming the retry
    /// escape and NOT `--connect` (the user already gave a locator).
    #[test]
    fn remote_section_empty_with_endpoints_is_one_honest_line() {
        let out = render_remote_topics_section(&topics_only(&[]), true);
        assert_eq!(out, reachable_none_line());
        assert_eq!(
            out.matches('\n').count(),
            1,
            "the empty remote notice is exactly ONE line; got: {out:?}"
        );
        assert!(
            !out.contains("REMOTE TOPICS"),
            "no section header when there is no row to head; got: {out}"
        );
        let window = format!("{} ms", REMOTE_QUERY_GATHER_WINDOW.as_millis());
        assert!(
            out.contains(&window) && out.contains("retry"),
            "the line must name the bounded window and the retry escape; got: {out}"
        );
        assert!(
            !out.contains("--connect tcp/<host>:7683"),
            "endpoint-given arm must not tell the user to pass --connect again; got: {out}"
        );
    }

    /// Empty + no explicit endpoints: ONE line carrying the `--connect` escape
    /// (7683 = the well-known permissive-gateway port), the bounded window, and
    /// no `REMOTE TOPICS` header. The removed `--network` flag must NOT be named.
    #[test]
    fn remote_section_empty_without_endpoints_is_one_line_hinting_connect() {
        let out = render_remote_topics_section(&topics_only(&[]), false);
        assert_eq!(out, unreachable_none_line());
        assert_eq!(
            out.matches('\n').count(),
            1,
            "the empty remote notice is exactly ONE line; got: {out:?}"
        );
        assert!(
            !out.contains("REMOTE TOPICS"),
            "no section header when there is no row to head; got: {out}"
        );
        let window = format!("{} ms", REMOTE_QUERY_GATHER_WINDOW.as_millis());
        assert!(
            out.contains(&window) && out.contains("--connect tcp/<host>:7683"),
            "endpoint-less empty arm must name the bounded window + the --connect escape; \
             got: {out}"
        );
        assert!(
            !out.contains("--network"),
            "the removed --network flag must not appear in the hint; got: {out}"
        );
    }

    /// The `remote:` line is the ONLY thing the empty arms print: with nothing
    /// discovered and no candidate the whole remote half is one line, and
    /// every non-topic line stays free of a leading `/` (a row parser keys off
    /// `/`-prefixed rows). The populated arm keeps the `REMOTE TOPICS` header
    /// and its rows byte-unchanged (pinned by `remote_section_renders_topics_one_per_line`).
    #[test]
    fn remote_none_discovered_lines_never_look_like_topic_rows() {
        for out in [
            render_remote_topics_section(&RemoteDiscovery::empty(), false),
            render_remote_topics_section(&RemoteDiscovery::empty(), true),
        ] {
            assert!(out.starts_with("remote: none discovered in "), "got: {out}");
            assert!(
                out.lines().all(|l| !l.starts_with('/')),
                "a notice line must never read as a topic row; got: {out}"
            );
        }
    }

    /// The remote-query FAILURE note is ONE exact stderr line. This is the
    /// sentence `docs/networking.md` and `docs/internals/cli.md` quote with
    /// `<error>` standing for the error text, so a reworded literal fails here
    /// and the two pages are then corrected by hand. Hand oracle.
    #[test]
    fn remote_discovery_unavailable_is_one_exact_line() {
        assert_eq!(
            render_remote_discovery_unavailable(&"zenoh session failed to open"),
            "remote: discovery unavailable (zenoh session failed to open; pass --no-network to \
             skip it)\n"
        );
        // A real engine error renders through the same seam the binary uses
        // (`&e` of a `CliError`): its Display text, in full, inside the parens.
        let err = CliError::Validation("bad locator 'tcp/nope'".to_string());
        assert_eq!(
            render_remote_discovery_unavailable(&err),
            "remote: discovery unavailable (bad locator 'tcp/nope'; pass --no-network to skip \
             it)\n"
        );
    }

    /// ONE line whatever the error carries: a multi-line error folds onto one
    /// line with no word lost, a terminal control character is neutralized,
    /// and the line never reads as a topic row. Hand oracles.
    #[test]
    fn remote_discovery_unavailable_stays_one_line_and_never_looks_like_a_topic_row() {
        assert_eq!(
            render_remote_discovery_unavailable(&"first line\n   second line\r\n\nthird"),
            "remote: discovery unavailable (first line second line third; pass --no-network to \
             skip it)\n"
        );
        assert_eq!(
            render_remote_discovery_unavailable(&"open tcp/10.0.0.9:7683\u{1b}[2J failed"),
            "remote: discovery unavailable (open tcp/10.0.0.9:7683\u{fffd}[2J failed; pass \
             --no-network to skip it)\n"
        );
        for error in ["/leading/slash error", "", "plain", "a\nb"] {
            let out = render_remote_discovery_unavailable(&error);
            assert!(out.starts_with("remote: discovery unavailable ("), "{out}");
            assert!(out.contains("--no-network"), "the escape is named: {out}");
            assert_eq!(out.matches('\n').count(), 1, "exactly one line: {out:?}");
            assert!(out.ends_with('\n'), "{out:?}");
            assert!(
                out.lines().all(|l| !l.starts_with('/')),
                "a notice line must never read as a topic row; got: {out}"
            );
        }
    }

    // ---- Internal topics: hidden by default, `--all` shows them (pure) ----

    /// The predicate is a PREFIX match over the ONE shared list plus the bare
    /// reserved token: the recorder's status channel and the framework's
    /// reserved control namespace (both spellings, and the token itself) are
    /// internal; a user topic that merely CONTAINS `bagd`, nests the word
    /// deeper, or shares the reserved spelling without being a child of it, is
    /// not. Hand oracle.
    #[test]
    fn internal_topic_predicate_is_a_prefix_match_over_the_shared_list() {
        for internal in [
            "/bagd/status",
            "/bagd/anything/else",
            "/__cerulion",
            "/__cerulion/mirrors",
            "/__cerulion/runs",
            "__cerulion/reserved",
        ] {
            assert!(is_internal_topic(internal), "{internal} must be internal");
        }
        for user in [
            "/bagdx",
            "/bagd",
            "/__cerulionx",
            "/my/bagd/status",
            "/obstacle_avoidance/laser_scanner/scan",
            "/cerulion/not_reserved",
            "/",
            "",
        ] {
            assert!(!is_internal_topic(user), "{user} must be a user topic");
        }
        // The gateway's reserved control namespace is pinned against its
        // owner, not hand-copied here: the bare token AND any child classify
        // internal through the SAME predicate the listing uses (the gateway's
        // `is_reserved_topic` rule), while a distinct name that merely shares
        // the spelling is a user topic.
        let reserved = cerulion_core::transport::gateway::RESERVED_TOPIC_PREFIX;
        assert!(is_internal_topic(reserved), "the bare namespace token");
        assert!(is_internal_topic(&format!("{reserved}/anything")));
        assert!(!is_internal_topic(&format!("{reserved}x")));
    }

    /// The recorder's half of the same pin, gated like every test in this
    /// module that names a Unix-only item (`cerulion_bagd` is a `cfg(unix)`
    /// dependency and `bag_cmd` a `cfg(unix)` module): the recorder's status
    /// topic classifies internal, and the recorder's auto-select exclusions
    /// are an ALIAS of the listing's list, never a second copy.
    #[cfg(unix)]
    #[test]
    fn internal_topic_list_is_the_recorders_list_and_hides_its_status_topic() {
        assert!(is_internal_topic(cerulion_bagd::STATUS_TOPIC));
        assert_eq!(
            INTERNAL_TOPIC_PREFIXES,
            crate::bag_cmd::AUTO_SELECT_EXCLUDED_PREFIXES,
            "the recorder's auto-select exclusions are an alias of this list"
        );
    }

    /// The quickstart shape (the recorder's status topic beside two user
    /// topics): the default listing hides `/bagd/status` and ends with the ONE
    /// count line naming `--all`. Exact hand oracle.
    #[test]
    fn local_section_hides_internal_topics_and_counts_them() {
        let local = info(&[
            "/bagd/status",
            "/obstacle_avoidance/laser_scanner/scan",
            "/obstacle_avoidance/safety_controller/linear_velocity",
        ]);
        assert_eq!(
            render_local_topics_section(&local, false),
            "TOPIC\n\
             /obstacle_avoidance/laser_scanner/scan\n\
             /obstacle_avoidance/safety_controller/linear_velocity\n\
             1 internal topic hidden (--all shows it)\n"
        );
    }

    /// `--all` on the same shape: every topic prints in enumeration order, the
    /// internal one with the `internal` marker in its trailing column, and
    /// there is NO count line (nothing was hidden). Exact hand oracle.
    #[test]
    fn local_section_all_marks_internal_rows_and_hides_nothing() {
        let local = info(&[
            "/bagd/status",
            "/obstacle_avoidance/laser_scanner/scan",
            "/obstacle_avoidance/safety_controller/linear_velocity",
        ]);
        let out = render_local_topics_section(&local, true);
        assert_eq!(
            out,
            "TOPIC\n\
             /bagd/status  internal\n\
             /obstacle_avoidance/laser_scanner/scan\n\
             /obstacle_avoidance/safety_controller/linear_velocity\n"
        );
        assert!(
            !out.contains("hidden"),
            "--all hides nothing, so no count line; got: {out}"
        );
        // Studio parser compat: the topic path is every row's FIRST whitespace
        // token; the marker rides the trailing column.
        assert_eq!(
            out.lines()
                .skip(1)
                .map(|l| l.split_whitespace().next().unwrap_or(""))
                .collect::<Vec<_>>(),
            vec![
                "/bagd/status",
                "/obstacle_avoidance/laser_scanner/scan",
                "/obstacle_avoidance/safety_controller/linear_velocity"
            ]
        );
    }

    /// Singular and plural agree in BOTH halves of the count line. Hand oracle.
    #[test]
    fn local_section_count_line_pluralizes_both_halves() {
        let two = info(&["/__cerulion/x", "/bagd/status", "/user/topic"]);
        assert_eq!(
            render_local_topics_section(&two, false),
            "TOPIC\n/user/topic\n2 internal topics hidden (--all shows them)\n"
        );
        let one = info(&["/bagd/status", "/user/topic"]);
        assert_eq!(
            render_local_topics_section(&one, false),
            "TOPIC\n/user/topic\n1 internal topic hidden (--all shows it)\n"
        );
    }

    /// A desk whose ONLY local topic is internal (a recorder left running with
    /// no user graph): the default reads `No active local topics.` AND still
    /// names what it hid, so the hidden topic is never silently gone; `--all`
    /// lists it. Exact hand oracles.
    #[test]
    fn local_section_only_internal_topics_reads_no_active_plus_the_count() {
        let local = info(&["/bagd/status"]);
        assert_eq!(
            render_local_topics_section(&local, false),
            "No active local topics.\n1 internal topic hidden (--all shows it)\n"
        );
        assert_eq!(
            render_local_topics_section(&local, true),
            "TOPIC\n/bagd/status  internal\n"
        );
    }

    /// With no internal topic the two modes are byte-identical and carry no
    /// count line: a user with only user topics sees exactly the old output.
    /// The empty enumeration reads `No active local topics.` in both modes.
    #[test]
    fn local_section_without_internal_topics_is_unchanged_in_both_modes() {
        let local = info(&["/a", "/b"]);
        assert_eq!(
            render_local_topics_section(&local, false),
            "TOPIC\n/a\n/b\n"
        );
        assert_eq!(render_local_topics_section(&local, true), "TOPIC\n/a\n/b\n");
        assert_eq!(
            render_local_topics_section(&[], false),
            "No active local topics.\n"
        );
        assert_eq!(
            render_local_topics_section(&[], true),
            "No active local topics.\n"
        );
    }

    /// Every NON-topic line of the local section (the header, the empty
    /// notice, the count line) stays free of a leading `/`, so a row parser
    /// keyed on `/`-prefixed rows can never mistake one for a topic; every
    /// topic row starts with its path. Checked over every shape above.
    #[test]
    fn local_section_non_topic_lines_never_start_with_a_slash() {
        let shapes: Vec<Vec<TopicInfo>> = vec![
            info(&[]),
            info(&["/bagd/status"]),
            info(&["/bagd/status", "/user/topic"]),
            info(&["/__cerulion/x", "/bagd/status", "/user/topic"]),
        ];
        for local in &shapes {
            for show_all in [false, true] {
                let out = render_local_topics_section(local, show_all);
                let names: BTreeSet<&str> = local.iter().map(|t| t.name.as_str()).collect();
                for line in out.lines() {
                    let first = line.split_whitespace().next().unwrap_or("");
                    if line.starts_with('/') {
                        assert!(
                            names.contains(first),
                            "a `/`-prefixed line must be a topic row whose first token \
                             is an enumerated topic; got {line:?} in {out:?}"
                        );
                    } else {
                        assert!(
                            !names.contains(first),
                            "a topic row must start with its path; got {line:?} in {out:?}"
                        );
                    }
                }
            }
        }
    }

    /// The bounded reply-gather window is a snappy, sub-second cap so
    /// `topic list` never hangs on zenoh's seconds-long default query timeout.
    /// The window IS the worst-case remote wait: `query_remote_topics` runs
    /// its two key-space gathers (demand + announce) CONCURRENTLY
    /// (sequential gathers would double the bound
    /// to 2× and break this claim).
    #[test]
    fn remote_query_gather_window_is_bounded_subsecond() {
        assert_eq!(REMOTE_QUERY_GATHER_WINDOW, Duration::from_millis(500));
        assert!(
            REMOTE_QUERY_GATHER_WINDOW < Duration::from_secs(1),
            "the gather window must stay sub-second for an interactive CLI \
             (it is the worst-case wait — the two key-space gathers run concurrently)"
        );
    }

    /// The positive scouting pin — the default
    /// `topic list` dispatch (no flags) builds a scouting-ON session. This is
    /// the automagic default itself; a regression to `scouting: false` ships
    /// LAN discovery silently dead.
    #[test]
    fn remote_discovery_options_default_is_scouting_on() {
        let opts = remote_discovery_options(false, vec![], vec![])
            .expect("the default dispatch must run the remote query");
        assert!(
            opts.scouting,
            "the automagic default MUST enable scouting (LAN discovery with zero flags)"
        );
        assert!(opts.connect.is_empty() && opts.listen.is_empty());
        assert!(!opts.has_endpoints());
    }

    /// `--no-network` skips the remote query
    /// entirely — no options, no session.
    #[test]
    fn remote_discovery_options_no_network_skips_query() {
        assert!(
            remote_discovery_options(true, vec!["tcp/10.0.0.1:7447".to_string()], vec![]).is_none(),
            "--no-network must skip the remote query even with locators given"
        );
    }

    /// Explicit locators are additive — they
    /// ride alongside the scouting-ON default, never replace it.
    #[test]
    fn remote_discovery_options_connect_is_additive_to_scouting() {
        let opts = remote_discovery_options(
            false,
            vec!["tcp/192.168.123.99:7447".to_string()],
            vec!["tcp/0.0.0.0:7447".to_string()],
        )
        .expect("locators must not disable the remote query");
        assert!(
            opts.scouting,
            "explicit --connect/--listen must ADD to the scouting session, not replace it"
        );
        assert_eq!(opts.connect, vec!["tcp/192.168.123.99:7447".to_string()]);
        assert_eq!(opts.listen, vec!["tcp/0.0.0.0:7447".to_string()]);
        assert!(opts.has_endpoints());
    }

    /// Normalization: unsorted, duplicated discovery replies come out
    /// sorted + deduplicated (hand oracle).
    #[test]
    fn normalize_remote_topics_sorts_and_dedups() {
        let raw = vec![
            "/z/late".to_string(),
            "/a/first".to_string(),
            "/z/late".to_string(),
            "/m/mid".to_string(),
        ];
        assert_eq!(
            normalize_remote_topics(raw),
            vec![
                "/a/first".to_string(),
                "/m/mid".to_string(),
                "/z/late".to_string()
            ]
        );
    }

    /// `has_endpoints`: false only when BOTH lists are empty.
    #[test]
    fn remote_options_has_endpoints_matrix() {
        let mk = |c: &[&str], l: &[&str]| RemoteTopicsOptions {
            connect: c.iter().map(|s| s.to_string()).collect(),
            listen: l.iter().map(|s| s.to_string()).collect(),
            scouting: false,
        };
        assert!(!mk(&[], &[]).has_endpoints());
        assert!(mk(&["tcp/10.0.0.1:7447"], &[]).has_endpoints());
        assert!(mk(&[], &["tcp/0.0.0.0:7447"]).has_endpoints());
        assert!(mk(&["tcp/10.0.0.1:7447"], &["tcp/0.0.0.0:7447"]).has_endpoints());
    }

    /// Error arm: a malformed locator fails the session open LOUDLY with
    /// the cause + the expected zenoh locator form — never a silent empty
    /// list. (`not-a-locator` has no `proto/address` shape, so it dies at
    /// zenoh config construction / session open — no network I/O.)
    #[test]
    fn query_remote_topics_malformed_locator_errors_loudly() {
        let opts = RemoteTopicsOptions {
            connect: vec!["not-a-locator".to_string()],
            listen: vec![],
            scouting: false,
        };
        let err = query_remote_topics(&opts)
            .expect_err("a malformed locator must fail the session open")
            .to_string();
        assert!(
            err.contains("zenoh discovery session") && err.contains("tcp/<host>:<port>"),
            "session-open failure must name the cause and the expected \
             locator form; got: {err}"
        );
    }

    /// Identity is the hostname, uniformly:
    /// the ROBOTS row name is exactly what the robot ANNOUNCES, and
    /// the mDNS locator is the locator column only. The mismatch class the live
    /// Go2 hit is gone by construction (the gateway now announces its hostname,
    /// and the desk displays that), so there is NO host-derived marker — the
    /// row name is the announced identity verbatim. Hand oracle over
    /// `build_robot_rows` + `render_robots_section` (announce-known vs mDNS-only).
    #[test]
    fn robots_row_name_is_the_announced_identity_locator_is_the_locator() {
        use crate::discovery_ladder::{DiscoveredPeer, DiscoveryRung};

        // (1) Announce-KNOWN: the robot announces its hostname ('ubuntu') and its
        // mDNS beacon (same identity) enriches the row with the locator → ONE row
        // 'ubuntu' with the locator, name = the announced identity verbatim.
        let announce = vec![("ubuntu".to_string(), Some("/lf/lowstate".to_string()))];
        let mdns = vec![DiscoveredPeer {
            robot: "ubuntu".to_string(),
            locator: "tcp/192.168.123.99:7683".to_string(),
            rung: DiscoveryRung::Mdns,
        }];
        let rows = build_robot_rows(&announce, &mdns);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].robot, "ubuntu");
        assert_eq!(rows[0].locator.as_deref(), Some("tcp/192.168.123.99:7683"));
        let rendered = render_robots_section(&rows, &mdns);
        assert!(
            rendered.contains("ubuntu  1 topic  @ tcp/192.168.123.99:7683  (mdns)"),
            "the row name is the announced identity; the mDNS locator is the locator column; \
             got: {rendered}"
        );
        assert!(
            !rendered.contains("(host)"),
            "no host marker; got: {rendered}"
        );

        // (2) mDNS-ONLY (no announce heard): the mDNS record's name IS the
        // identity (hostname) — rendered plain with its locator, still no marker.
        let mdns_only = vec![DiscoveredPeer {
            robot: "orin-a".to_string(),
            locator: "tcp/10.0.0.5:7683".to_string(),
            rung: DiscoveryRung::Mdns,
        }];
        let rows2 = build_robot_rows(&[], &mdns_only);
        assert_eq!(rows2.len(), 1);
        assert_eq!(rows2[0].robot, "orin-a");
        let rendered2 = render_robots_section(&rows2, &mdns_only);
        assert!(
            rendered2.contains("orin-a  @ tcp/10.0.0.5:7683  (mdns)"),
            "an mDNS-only identity renders plain (no `(host)` marker); got: {rendered2}"
        );
        assert!(!rendered2.contains("(host)"));
    }

    /// The served-doc twin: the served-closure
    /// walker (`seed_framewalker`) fed peer docs straight into
    /// `FrameWalker::new` with no preflight, so the hostile shape
    /// served by a peer PANICKED `topic echo`. With the preflight: no panic, and
    /// the sane doc's wire hash still resolves through the seeded walker.
    #[test]
    fn r16_the_served_doc_walker_degrades_a_hostile_closure_instead_of_panicking() {
        use cerulion_core::codegen::layout::LayoutResolver;
        use cerulion_core::codegen::parse_rosmsg;

        let sane_text = "float64 x\nfloat64 y\n";
        let mut oracle_schemas = crate::schema_cmd::parse_builtin_schemas();
        oracle_schemas.push(parse_rosmsg(sane_text, "Sane", Some("hostile")).unwrap());
        let (mut resolver, _) = LayoutResolver::new(oracle_schemas);
        let sane_hash = resolver.layout_of("hostile/Sane").unwrap().schema_hash;

        // Without the preflight this PANICS inside FrameWalker::new.
        let walker = seed_framewalker(&[
            cerulion_core::SchemaDoc {
                qualified: "hostile/Inner".to_string(),
                encoding: cerulion_core::SchemaEncoding::Msg,
                text: "float64[536870907] v\n".to_string(),
                deps: vec![],
            },
            cerulion_core::SchemaDoc {
                qualified: "hostile/Outer".to_string(),
                encoding: cerulion_core::SchemaEncoding::Msg,
                text: "Inner[8589934592] arr\n".to_string(),
                deps: vec!["hostile/Inner".to_string()],
            },
            cerulion_core::SchemaDoc {
                qualified: "hostile/Wrapper".to_string(),
                encoding: cerulion_core::SchemaEncoding::Msg,
                text: "Outer o\nfloat64 w\n".to_string(),
                deps: vec!["hostile/Outer".to_string()],
            },
            cerulion_core::SchemaDoc {
                qualified: "hostile/Sane".to_string(),
                encoding: cerulion_core::SchemaEncoding::Msg,
                text: "float64 x\nfloat64 y\n".to_string(),
                deps: vec![],
            },
        ]);
        assert_eq!(
            walker.schema_name_for_hash(sane_hash),
            Some("hostile/Sane"),
            "the sane served doc still resolves after the hostile shape degrades"
        );
    }

    /// The echo walker rides the shared PREFLIGHTED resolution set.
    /// The hostile store shape (`Huge` past the u32 ceiling; `Outer =
    /// Inner[8589934592]` composing an overflow that `Wrapper` nests as a FIXED
    /// target) PANICKED `FrameWalker::new` here — `resolve_fixed_nested`
    /// materializes `Outer` as `Wrapper`'s target — while `schema info` over
    /// the same files degraded. Now: no panic, and the sane sibling's wire hash
    /// still resolves locally (the anti-vacuity half).
    #[test]
    fn r14_the_echo_walker_degrades_a_hostile_workspace_instead_of_panicking() {
        use cerulion_core::codegen::layout::LayoutResolver;
        use cerulion_core::codegen::parse_rosmsg;

        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("nav14").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Huge.msg"), "float64[2305843009213693951] v\n").unwrap();
        std::fs::write(msg_dir.join("Inner.msg"), "float64[536870907] v\n").unwrap();
        std::fs::write(msg_dir.join("Outer.msg"), "Inner[8589934592] arr\n").unwrap();
        std::fs::write(msg_dir.join("Wrapper.msg"), "Outer o\nfloat64 w\n").unwrap();
        let goal_text = "float64 x\nfloat64 y\n";
        std::fs::write(msg_dir.join("Goal.msg"), goal_text).unwrap();

        // Hand oracle: an INDEPENDENT resolver over built-ins + the sane type.
        let mut oracle_schemas = crate::schema_cmd::parse_builtin_schemas();
        oracle_schemas.push(parse_rosmsg(goal_text, "Goal", Some("nav14")).unwrap());
        let (mut resolver, _) = LayoutResolver::new(oracle_schemas);
        let goal_hash = resolver.layout_of("nav14/Goal").unwrap().schema_hash;

        // Without the preflight this PANICS inside FrameWalker::new.
        let walker = local_walker_from_workspace(Some(tmp.path()));
        assert_eq!(
            walker.schema_name_for_hash(goal_hash),
            Some("nav14/Goal"),
            "the sane sibling still resolves after the hostile shapes degrade"
        );
    }

    /// The echo walker built from a WORKSPACE `.msg` store
    /// resolves that store type's wire `schema_hash` LOCALLY, closing the
    /// gap left when the store type is absent from the echo walker (so
    /// `/lf/lowstate` falls through to the remote tier). The oracle is a THIRD,
    /// independent recipe-3 hash (`LayoutResolver` over the same corpus), and the
    /// remote seed walker (built from the SAME `.msg` text) resolves the SAME
    /// hash — proving the store-parsed and served-doc hashes are ONE computation.
    #[test]
    fn echo_local_walker_resolves_a_workspace_store_type_hash() {
        use cerulion_core::codegen::layout::LayoutResolver;
        use cerulion_core::codegen::parse_rosmsg;
        use cerulion_core::{SchemaDoc, SchemaEncoding};

        // A workspace `.msg` store: schemas/demo_msgs/msg/Widget.msg (a custom
        // type the desk never compiled).
        let text = "float64 x\nfloat64 y\n";
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("demo_msgs").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Widget.msg"), text).unwrap();

        // Hand oracle: the recipe-3 wire hash via an INDEPENDENT resolver over
        // built-ins + the custom type (what a compile-time twin's SCHEMA_HASH is).
        let mut oracle_schemas = crate::schema_cmd::parse_builtin_schemas();
        oracle_schemas.push(parse_rosmsg(text, "Widget", Some("demo_msgs")).unwrap());
        let (mut resolver, _) = LayoutResolver::new(oracle_schemas);
        let wire_hash = resolver.layout_of("demo_msgs/Widget").unwrap().schema_hash;

        // The ECHO walker (from the workspace store) resolves the store type
        // LOCALLY — no network.
        let echo_walker = local_walker_from_workspace(Some(tmp.path()));
        assert_eq!(
            echo_walker.schema_name_for_hash(wire_hash),
            Some("demo_msgs/Widget"),
            "the echo walker must resolve the workspace store type's wire hash locally"
        );

        // The REMOTE seed walker (from the SAME `.msg` text) resolves the SAME
        // hash — the store-parsed and served-doc hashes are THE SAME computation.
        let served_walker = seed_framewalker(&[SchemaDoc {
            qualified: "demo_msgs/Widget".to_string(),
            encoding: SchemaEncoding::Msg,
            text: text.to_string(),
            deps: vec![],
        }]);
        assert_eq!(
            served_walker.schema_name_for_hash(wire_hash),
            Some("demo_msgs/Widget"),
            "the served-doc walker resolves the SAME hash as the store walker"
        );

        // No workspace ⇒ a built-ins-only walker does NOT know the custom type
        // (the remote tier is the fallback there).
        let none_walker = local_walker_from_workspace(None);
        assert_eq!(none_walker.schema_name_for_hash(wire_hash), None);
    }

    /// The remote-fallback breadcrumb TEXT is
    /// FACTUAL, not predictive. A predictive wording ("unknown-type frames ...
    /// will be decoded via robot X") asserts a remote decode a local-heavy
    /// topic never takes; the line states what IS happening. Pinning
    /// the exact factual tokens makes a predictive wording fail this test.
    #[test]
    fn remote_fallback_breadcrumb_is_factual_not_predictive() {
        let line = remote_fallback_breadcrumb("/lf/lowstate", "ubuntu");
        assert!(line.contains("/lf/lowstate"), "names the topic: {line}");
        assert!(line.contains("robot 'ubuntu'"), "names the robot: {line}");
        assert!(
            line.contains("nothing written to disk"),
            "keeps the no-disk reassurance: {line}"
        );
        // FACTUAL present-tense — this fires only on a frame ACTUALLY decoding
        // remotely, so it states what IS happening.
        assert!(
            line.contains("are being decoded"),
            "must state what IS happening: {line}"
        );
        // The predictive promise must be ABSENT.
        assert!(
            !line.contains("will be decoded"),
            "must not PREDICT a remote decode that may never happen: {line}"
        );
    }

    /// `write_custom_decode` emits the
    /// remote-fallback breadcrumb LAZILY — only on the FIRST frame that ACTUALLY
    /// decodes via the remote walker, NEVER for a locally-decoded frame (the
    /// defect this pins: a breadcrumb fired pre-loop whenever a remote walker
    /// merely RESOLVED, even on a topic whose every frame decodes locally) and
    /// only ONCE. Hand oracles over a local Widget + a remote-served Gadget —
    /// NOT a self-compare.
    #[test]
    fn write_custom_decode_breadcrumb_is_lazy_factual_and_once() {
        use cerulion_core::codegen::parse_rosmsg;
        use cerulion_core::wire::WireHeader;
        use cerulion_core::{SchemaDoc, SchemaEncoding};

        let widget_text = "float64 x\nfloat64 y\n";
        let gadget_text = "float64 z\n";
        let widget_hash = parse_rosmsg(widget_text, "Widget", Some("demo_msgs"))
            .unwrap()
            .schema_hash();
        let gadget_hash = parse_rosmsg(gadget_text, "Gadget", Some("demo_msgs"))
            .unwrap()
            .schema_hash();

        // Workspace store with ONLY Widget ⇒ the LOCAL walker knows Widget but
        // NOT Gadget.
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("demo_msgs").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Widget.msg"), widget_text).unwrap();
        let local_walker = local_walker_from_workspace(Some(tmp.path()));

        // A remote-served walker that knows Gadget (robot "ubuntu").
        let remote = (
            "ubuntu".to_string(),
            seed_framewalker(&[SchemaDoc {
                qualified: "demo_msgs/Gadget".to_string(),
                encoding: SchemaEncoding::Msg,
                text: gadget_text.to_string(),
                deps: vec![],
            }]),
        );

        let widget_header = WireHeader::with_schema(widget_hash);
        let mut widget_payload = Vec::new();
        widget_payload.extend_from_slice(&1.0f64.to_le_bytes());
        widget_payload.extend_from_slice(&2.0f64.to_le_bytes());
        let gadget_header = WireHeader::with_schema(gadget_hash);
        let gadget_payload = 3.0f64.to_le_bytes().to_vec();

        let mut shown = false;
        let mut explained: std::collections::BTreeSet<u64> = std::collections::BTreeSet::new();
        let mut buf: Vec<u8> = Vec::new();

        // Frame A — Widget decodes LOCALLY. A remote walker IS present (the
        // eager trigger), yet NO breadcrumb fires for a locally-decoded frame.
        write_custom_decode(
            &mut buf,
            "/lf/lowstate",
            &widget_header,
            &widget_payload,
            &local_walker,
            Some(&remote),
            &mut shown,
            &mut explained,
            DEFAULT_ECHO_TRUNCATE_LENGTH,
        );
        let a = String::from_utf8(std::mem::take(&mut buf)).unwrap();
        assert!(
            a.contains("demo_msgs/Widget:"),
            "Widget must decode locally under its schema-name header: {a}"
        );
        // The ACTUAL field VALUES are rendered (not a count).
        assert!(
            a.contains("x: 1.0") && a.contains("y: 2.0"),
            "the decoded field values must render: {a}"
        );
        assert!(
            !a.contains("field(s)"),
            "the bare field-count line must be GONE (item 3): {a}"
        );
        assert!(
            !a.contains("[schema:"),
            "NO breadcrumb for a locally-decoded frame (the eager-breadcrumb defect): {a}"
        );
        assert!(!shown, "the latch stays unset after a purely-local frame");

        // Frame B — Gadget decodes REMOTELY. NOW (and only now) the FACTUAL
        // breadcrumb fires, once, ahead of the decode line.
        write_custom_decode(
            &mut buf,
            "/lf/lowstate",
            &gadget_header,
            &gadget_payload,
            &local_walker,
            Some(&remote),
            &mut shown,
            &mut explained,
            DEFAULT_ECHO_TRUNCATE_LENGTH,
        );
        let b = String::from_utf8(std::mem::take(&mut buf)).unwrap();
        assert!(
            b.contains("demo_msgs/Gadget (via robot 'ubuntu')"),
            "Gadget must decode remotely under its schema-name header: {b}"
        );
        // The ACTUAL field value renders on the remote path too.
        assert!(
            b.contains("z: 3.0"),
            "the remotely-decoded field value must render: {b}"
        );
        assert!(
            b.contains("are being decoded via robot 'ubuntu'")
                && b.contains("nothing written to disk"),
            "the factual breadcrumb fires on the FIRST remote decode: {b}"
        );
        assert!(
            !b.contains("will be decoded"),
            "the breadcrumb is factual, not predictive: {b}"
        );
        assert!(shown, "the latch is set after the first remote decode");

        // Frame B2 — another Gadget decodes remotely, but the breadcrumb is
        // ONCE-ONLY (no second announcement).
        write_custom_decode(
            &mut buf,
            "/lf/lowstate",
            &gadget_header,
            &gadget_payload,
            &local_walker,
            Some(&remote),
            &mut shown,
            &mut explained,
            DEFAULT_ECHO_TRUNCATE_LENGTH,
        );
        let b2 = String::from_utf8(std::mem::take(&mut buf)).unwrap();
        assert!(
            b2.contains("demo_msgs/Gadget (via robot 'ubuntu')") && b2.contains("z: 3.0"),
            "the second Gadget still decodes remotely with its value: {b2}"
        );
        assert!(
            !b2.contains("[schema:"),
            "the breadcrumb is once-only — no second announcement: {b2}"
        );
    }

    /// `topic echo` NAMES an unresolvable `schema_hash` instead
    /// of dumping bare hex.
    ///
    /// `topic echo` is the first surface an operator reaches for, and a
    /// frame whose type NEITHER walker holds must not print `payload: 3f f0 …` and
    /// nothing else, on the one command
    /// whose whole job is to say what is on a topic. (Its sibling `topic info`
    /// names the condition too; the two agree.)
    ///
    /// Hand oracles over the SAME fixtures as the breadcrumb test above: a hash
    /// no walker knows, and — the ANTI-TAUTOLOGY half, in the same body — a
    /// locally-decodable frame that must NOT carry the line, without which a
    /// version stamping it on every frame would pass.
    #[test]
    fn write_custom_decode_names_an_unresolvable_hash_instead_of_bare_hex() {
        use cerulion_core::codegen::parse_rosmsg;
        use cerulion_core::wire::WireHeader;

        let widget_text = "float64 x\nfloat64 y\n";
        let widget_hash = parse_rosmsg(widget_text, "Widget", Some("demo_msgs"))
            .unwrap()
            .schema_hash();
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("demo_msgs").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Widget.msg"), widget_text).unwrap();
        let local_walker = local_walker_from_workspace(Some(tmp.path()));

        let payload: Vec<u8> = [1.0f64, 2.0].iter().flat_map(|v| v.to_le_bytes()).collect();

        // A hash NOTHING resolves — a robot type this build never compiled, or a
        // type it holds under a different definition. Both look like this here.
        let mut shown = false;
        let mut explained: std::collections::BTreeSet<u64> = std::collections::BTreeSet::new();
        let mut buf: Vec<u8> = Vec::new();
        write_custom_decode(
            &mut buf,
            "/lf/mystery",
            &WireHeader::with_schema(0xDEAD_BEEF_DEAD_BEEF),
            &payload,
            &local_walker,
            None,
            &mut shown,
            &mut explained,
            DEFAULT_ECHO_TRUNCATE_LENGTH,
        );
        let out = String::from_utf8(std::mem::take(&mut buf)).unwrap();
        assert!(
            out.contains("undecodable: schema unidentified"),
            "an unresolvable hash must be NAMED, not silently hex-dumped: {out}"
        );
        assert!(
            out.contains("0xDEADBEEFDEADBEEF"),
            "the wire hash is the one fact the frame always carries: {out}"
        );
        assert!(
            out.contains("cerulion topic info"),
            "and the line must tell the operator what to DO next: {out}"
        );
        // The hex preview is KEPT — the diagnostic explains the dump, it does
        // not replace bytes an operator may still want to eyeball.
        assert!(out.contains("payload: "), "the hex preview survives: {out}");

        // ANTI-TAUTOLOGY: a frame this build CAN decode carries no such line.
        write_custom_decode(
            &mut buf,
            "/lf/known",
            &WireHeader::with_schema(widget_hash),
            &payload,
            &local_walker,
            None,
            &mut shown,
            &mut explained,
            DEFAULT_ECHO_TRUNCATE_LENGTH,
        );
        let good = String::from_utf8(std::mem::take(&mut buf)).unwrap();
        assert!(
            good.contains("demo_msgs/Widget:"),
            "the control really does decode: {good}"
        );
        assert!(
            !good.contains("undecodable"),
            "a decodable frame must not be labelled undecodable: {good}"
        );
    }

    /// The ~254-character remediation paragraph
    /// prints ONCE PER DISTINCT HASH, not once per frame — while EVERY frame
    /// keeps its hex preview.
    ///
    /// `write_custom_decode` sits inside the per-frame `wait_for_message`
    /// closure, and echo always passes `candidate: None`, so only the
    /// `Unidentified` arm is reachable and its text is byte-invariant for a given
    /// hash. Unlatched, a 500 Hz undecodable topic emits ~135 KB/s of identical
    /// prose carrying zero per-frame information — the flood class every OTHER
    /// emission on this path latches, in a function that already carries a once-shown
    /// latch forty lines above.
    ///
    /// Hand oracles over frame COUNTS, plus the two-hash arm (a second distinct
    /// hash gets its own explanation, so the latch is per-hash and not a single
    /// bit) and the NON-SHARING arm (it must not be `breadcrumb_shown`: sharing
    /// would let whichever condition fired first suppress the other).
    #[test]
    fn write_custom_decode_explains_an_unresolvable_hash_once_not_per_frame() {
        use cerulion_core::wire::WireHeader;

        let local_walker = local_walker_from_workspace(None);
        let payload: Vec<u8> = [1.0f64, 2.0].iter().flat_map(|v| v.to_le_bytes()).collect();
        const MARKER: &str = "undecodable: schema unidentified";

        let mut shown = false;
        let mut explained: std::collections::BTreeSet<u64> = std::collections::BTreeSet::new();
        let mut buf: Vec<u8> = Vec::new();

        // Ten frames of ONE undecodable topic — the shape a skewed 500 Hz robot
        // produces, just bounded.
        for _ in 0..10 {
            write_custom_decode(
                &mut buf,
                "/lf/mystery",
                &WireHeader::with_schema(0xDEAD_BEEF_DEAD_BEEF),
                &payload,
                &local_walker,
                None,
                &mut shown,
                &mut explained,
                DEFAULT_ECHO_TRUNCATE_LENGTH,
            );
        }
        let out = String::from_utf8(std::mem::take(&mut buf)).unwrap();
        assert_eq!(
            out.matches(MARKER).count(),
            1,
            "the remediation paragraph must print once, not per frame: {out}"
        );
        // The per-frame diagnostic an operator actually re-reads is untouched —
        // this is the anti-tautology half: a latch that suppressed the HEX too
        // would satisfy the count above.
        assert_eq!(
            out.matches("payload: ").count(),
            10,
            "every frame keeps its hex preview: {out}"
        );

        // A SECOND distinct hash on the same run earns its own explanation — the
        // latch is per-hash, not one bit for the whole run.
        write_custom_decode(
            &mut buf,
            "/lf/mystery",
            &WireHeader::with_schema(0x0BAD_F00D_0BAD_F00D),
            &payload,
            &local_walker,
            None,
            &mut shown,
            &mut explained,
            DEFAULT_ECHO_TRUNCATE_LENGTH,
        );
        let second = String::from_utf8(std::mem::take(&mut buf)).unwrap();
        assert_eq!(
            second.matches(MARKER).count(),
            1,
            "a different unresolvable hash is a different fact: {second}"
        );

        // NON-SHARING: the undecodable latch must not be `breadcrumb_shown`.
        // Ten undecodable frames have gone by; the remote-fallback breadcrumb's
        // own latch must still be UNBURNED, or the first undecodable frame on a
        // topic would silence a later remote decode's breadcrumb.
        assert!(
            !shown,
            "the undecodable explanation burned the remote-fallback breadcrumb latch"
        );
    }

    /// `format_leaf_value` renders each decoded LEAF
    /// kind in a human-readable form — scalars verbatim, floats with the decimal
    /// point, strings quoted, and byte arrays as their ELEMENTS. Hand
    /// oracles (never self-compare).
    #[test]
    fn format_leaf_value_renders_scalars_strings_and_blobs() {
        use cerulion_core::codegen::{FrameValueKind as K, PrimArray, PrimType};
        let n = DEFAULT_ECHO_TRUNCATE_LENGTH;
        assert_eq!(format_leaf_value(&K::Bool(true), n), "true");
        assert_eq!(format_leaf_value(&K::U8(7), n), "7");
        assert_eq!(format_leaf_value(&K::I32(-42), n), "-42");
        assert_eq!(
            format_leaf_value(&K::U64(18446744073709551615), n),
            "18446744073709551615"
        );
        // `{:?}` keeps the float-ness visible (`1.0`, not `1`).
        assert_eq!(format_leaf_value(&K::F64(1.0), n), "1.0");
        assert_eq!(format_leaf_value(&K::F32(0.5), n), "0.5");
        assert_eq!(format_leaf_value(&K::Str("hello"), n), "\"hello\"");
        // A short byte array renders its ELEMENTS (no more `<N bytes>`).
        let short_bytes = vec![12u8, 0, 255];
        assert_eq!(
            format_leaf_value(&K::Bytes(&short_bytes), n),
            "[12, 0, 255]"
        );
        // A numeric array renders in its NATIVE type.
        let f64_bytes: Vec<u8> = [1.0f64, 2.0, 3.0]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let arr = PrimArray {
            elem: PrimType::F64,
            bytes: &f64_bytes,
            count: 3,
        };
        assert_eq!(format_leaf_value(&K::PrimArray(arr), n), "[1.0, 2.0, 3.0]");
        let opaque = vec![0u8; 12];
        assert_eq!(
            format_leaf_value(&K::NestedArrayOpaque(&opaque), n),
            // NEUTRAL label — the walker routes bool[] here
            // too, so it must not claim "nested-message". This opaque-array
            // arm stays a summary (no element framing to enumerate).
            "<12 bytes, opaque array>"
        );
    }

    /// A numeric array longer than `truncate_length` is truncated with a
    /// trailing `...` inside the brackets + a `(N elements)` total annotation; an
    /// exactly-bound-length array is shown in full with NO annotation; integer
    /// elements stay integer.
    #[test]
    fn format_prim_array_truncates_beyond_bound() {
        use cerulion_core::codegen::{PrimArray, PrimType};
        let vals: Vec<u32> = (0..10).collect();
        let bytes: Vec<u8> = vals.iter().flat_map(|v| v.to_le_bytes()).collect();
        let arr = PrimArray {
            elem: PrimType::U32,
            bytes: &bytes,
            count: 10,
        };
        // Bound 8 < 10 → truncated: 8 elements + `...` + `(10 elements)`.
        assert_eq!(
            format_prim_array(&arr, 8),
            "[0, 1, 2, 3, 4, 5, 6, 7, ...] (10 elements)"
        );
        // Bound == count (10) → full, NO annotation (ros2-parity).
        assert_eq!(
            format_prim_array(&arr, 10),
            "[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]"
        );
    }

    /// A `uint8[]` shorter than or equal to the bound renders EVERY
    /// element with NO annotation (the LowState `head` / short-array case).
    #[test]
    fn byte_array_within_bound_renders_all_elements_no_annotation() {
        use cerulion_core::codegen::FrameValueKind as K;
        // Empty byte array → empty brackets.
        assert_eq!(format_leaf_value(&K::Bytes(&[]), 128), "[]");
        // A 2-byte LowState `head`.
        assert_eq!(format_leaf_value(&K::Bytes(&[255u8, 0]), 128), "[255, 0]");
        // Exactly-at-bound (128) renders full, no `...`, no `(N elements)`.
        let at_bound: Vec<u8> = (0..128).map(|i| (i % 256) as u8).collect();
        let rendered = format_leaf_value(&K::Bytes(&at_bound), 128);
        assert!(
            !rendered.contains("..."),
            "at-bound must not truncate: no `...`"
        );
        assert!(
            !rendered.contains("elements)"),
            "at-bound must not carry the count annotation"
        );
        let expected: Vec<String> = at_bound.iter().map(|b| b.to_string()).collect();
        assert_eq!(rendered, format!("[{}]", expected.join(", ")));
    }

    /// A `uint8[]` LONGER than the bound truncates at EXACTLY the bound
    /// with a trailing `...` inside the brackets + a `(N elements)` total
    /// annotation — the PointCloud2 `data` case (the headline ros2-parity shape).
    #[test]
    fn byte_array_beyond_bound_truncates_at_128_with_count() {
        use cerulion_core::codegen::FrameValueKind as K;
        // 57600 bytes (a small PointCloud2 `data`), first three are 12/0/255.
        let mut data: Vec<u8> = vec![0u8; 57600];
        data[0] = 12;
        data[1] = 0;
        data[2] = 255;
        let rendered = format_leaf_value(&K::Bytes(&data), 128);
        // EXACTLY 128 elements shown before the marker.
        let shown: Vec<String> = data[..128].iter().map(|b| b.to_string()).collect();
        assert_eq!(
            rendered,
            format!("[{}, ...] (57600 elements)", shown.join(", "))
        );
        // Head/tail spot-checks so the assertion is not vacuous.
        assert!(rendered.starts_with("[12, 0, 255, "), "{rendered}");
        assert!(rendered.ends_with(", ...] (57600 elements)"), "{rendered}");
    }

    /// Truncation boundary: exactly-128 is full (no annotation); 129 truncates.
    #[test]
    fn byte_array_boundary_128_full_129_truncated() {
        use cerulion_core::codegen::FrameValueKind as K;
        let n128: Vec<u8> = (0..128u32).map(|i| (i % 256) as u8).collect();
        let n129: Vec<u8> = (0..129u32).map(|i| (i % 256) as u8).collect();
        // 128 → full, no `...`.
        assert!(!format_leaf_value(&K::Bytes(&n128), 128).contains("..."));
        // 129 → truncated to 128 + `...` + `(129 elements)`.
        let r129 = format_leaf_value(&K::Bytes(&n129), 128);
        let shown: Vec<String> = n129[..128].iter().map(|b| b.to_string()).collect();
        assert_eq!(r129, format!("[{}, ...] (129 elements)", shown.join(", ")));
    }

    /// A non-u8 (numeric) array LONGER than the bound truncates in the
    /// EXACT same shape as a byte array — one seam, one shape.
    #[test]
    fn non_u8_array_beyond_bound_truncates_identically() {
        use cerulion_core::codegen::{FrameValueKind as K, PrimArray, PrimType};
        let vals: Vec<u32> = (0..200).collect();
        let bytes: Vec<u8> = vals.iter().flat_map(|v| v.to_le_bytes()).collect();
        let arr = PrimArray {
            elem: PrimType::U32,
            bytes: &bytes,
            count: 200,
        };
        let rendered = format_leaf_value(&K::PrimArray(arr), 128);
        let shown: Vec<String> = (0..128u32).map(|v| v.to_string()).collect();
        assert_eq!(
            rendered,
            format!("[{}, ...] (200 elements)", shown.join(", "))
        );
    }

    /// The `truncate_length` bound is THREADED — a bound of 4 renders 4
    /// elements + `...` + the count, on BOTH byte and numeric arrays.
    #[test]
    fn truncate_length_bound_is_honored_end_to_end() {
        use cerulion_core::codegen::{FrameValueKind as K, PrimArray, PrimType};
        // Byte array, bound 4.
        assert_eq!(
            format_leaf_value(&K::Bytes(&[10u8, 20, 30, 40, 50, 60]), 4),
            "[10, 20, 30, 40, ...] (6 elements)"
        );
        // Numeric array, bound 4.
        let vals: Vec<u32> = (0..6).collect();
        let bytes: Vec<u8> = vals.iter().flat_map(|v| v.to_le_bytes()).collect();
        let arr = PrimArray {
            elem: PrimType::U32,
            bytes: &bytes,
            count: 6,
        };
        assert_eq!(
            format_leaf_value(&K::PrimArray(arr), 4),
            "[0, 1, 2, 3, ...] (6 elements)"
        );
    }

    /// Truncation boundary: a bound of 1 is the tightest legal bound — N>1 shows one
    /// element + `...` + `(N elements)`, exactly-one-element renders `[e0]` with
    /// NO annotation, and an empty array renders `[]`.
    #[test]
    fn bound_of_one_shows_single_element_and_truncates_the_rest() {
        use cerulion_core::codegen::FrameValueKind as K;
        // N>1 → one element then the truncation marker + count.
        assert_eq!(
            format_leaf_value(&K::Bytes(&[7u8, 8, 9]), 1),
            "[7, ...] (3 elements)"
        );
        // Exactly one element at bound 1 → full, NO annotation.
        assert_eq!(format_leaf_value(&K::Bytes(&[42u8]), 1), "[42]");
        // Empty at bound 1 → empty brackets, NO annotation.
        assert_eq!(format_leaf_value(&K::Bytes(&[]), 1), "[]");
    }

    /// The MULTI-LINE nested-message array arm (`FrameValueKind::Array`,
    /// which the walker emits for a fixed array of fixed-nested messages like
    /// `geometry_msgs/Point[N]`) also honors `truncate_length` — it renders up to
    /// `bound` element blocks, elides the rest, and closes with the multi-line
    /// `… (N total)` tail. Pins BOTH the truncation bound behavior AND the exact
    /// tail wording. Hand-built oracle (never a self-compare).
    #[test]
    fn nested_message_array_truncates_at_bound_with_total_tail() {
        use cerulion_core::codegen::{FrameValue, FrameValueKind as K, NamedValue};
        // A `geometry_msgs/Point[5]`-shaped fixed array of fixed-nested messages
        // — exactly the shape the walker decodes into `FrameValueKind::Array`.
        let point = |x: f64| {
            K::Nested(Box::new(FrameValue {
                schema_name: "geometry_msgs/Point".into(),
                fields: vec![
                    NamedValue {
                        name: "x".into(),
                        value: K::F64(x),
                    },
                    NamedValue {
                        name: "y".into(),
                        value: K::F64(0.0),
                    },
                ],
            }))
        };
        let arr = K::Array(vec![
            point(0.0),
            point(1.0),
            point(2.0),
            point(3.0),
            point(4.0),
        ]);

        // Bound 2 < 5 → header count line, blocks [0] and [1] rendered, [2..]
        // elided, and the multi-line `… (5 total)` tail.
        let mut buf: Vec<u8> = Vec::new();
        write_named_value(&mut buf, "points", &arr, 0, 2, &mut 0);
        let out = String::from_utf8(buf).unwrap();
        assert!(out.contains("points: [5 element(s)]"), "{out}");
        assert!(out.contains("[0]:") && out.contains("x: 0.0"), "{out}");
        assert!(out.contains("[1]:") && out.contains("x: 1.0"), "{out}");
        assert!(
            !out.contains("[2]:"),
            "beyond-bound blocks must be elided: {out}"
        );
        assert!(
            !out.contains("x: 2.0"),
            "the 3rd element's fields must be elided: {out}"
        );
        // The EXACT multi-line truncation tail wording (unicode ellipsis).
        assert!(out.contains("… (5 total)"), "{out}");

        // Control: bound 5 == len → every block rendered, NO `… (N total)` tail.
        let mut buf2: Vec<u8> = Vec::new();
        write_named_value(&mut buf2, "points", &arr, 0, 5, &mut 0);
        let out2 = String::from_utf8(buf2).unwrap();
        assert!(out2.contains("[4]:") && out2.contains("x: 4.0"), "{out2}");
        assert!(
            !out2.contains("total)"),
            "an at-or-above-bound array must not carry the total tail: {out2}"
        );
    }

    /// STRINGS are NOT affected by the array-truncation change — a long
    /// string still renders whole + quoted, never element-split or `...`-tailed.
    #[test]
    fn strings_are_not_truncated_by_the_array_bound() {
        use cerulion_core::codegen::FrameValueKind as K;
        let long = "a".repeat(300);
        let rendered = format_leaf_value(&K::Str(&long), 4);
        assert_eq!(rendered, format!("{long:?}"));
        assert!(
            !rendered.contains("elements)") && !rendered.contains(", ..."),
            "a string must never carry the array truncation tail: {rendered}"
        );
    }

    /// `render_frame_fields` renders a nested message
    /// indented under its parent, quotes string values, and renders byte arrays
    /// as their ELEMENTS. Hand oracle over a HAND-BUILT `FrameValue` (never a
    /// self-compare).
    #[test]
    fn render_frame_fields_renders_nested_and_byte_arrays() {
        use cerulion_core::codegen::{FrameValue, FrameValueKind as K, NamedValue};
        let inner = FrameValue {
            schema_name: "demo/Inner".into(),
            fields: vec![NamedValue {
                name: "x".into(),
                value: K::F64(1.5),
            }],
        };
        // A LowState-style `head` byte field renders its elements.
        let blob = vec![255u8, 0];
        let msg = FrameValue {
            schema_name: "demo/Outer".into(),
            fields: vec![
                NamedValue {
                    name: "count".into(),
                    value: K::U32(5),
                },
                NamedValue {
                    name: "label".into(),
                    value: K::Str("ok"),
                },
                NamedValue {
                    name: "inner".into(),
                    value: K::Nested(Box::new(inner)),
                },
                NamedValue {
                    name: "head".into(),
                    value: K::Bytes(&blob),
                },
            ],
        };
        let mut buf: Vec<u8> = Vec::new();
        render_frame_fields(&mut buf, &msg, 1, DEFAULT_ECHO_TRUNCATE_LENGTH);
        let out = String::from_utf8(buf).unwrap();
        // Top-level fields at 4-space indent (indent 1 → pad = 2 levels).
        assert!(
            out.contains("\n    count: 5\n") || out.starts_with("    count: 5\n"),
            "{out}"
        );
        assert!(out.contains("    label: \"ok\""), "{out}");
        assert!(out.contains("    inner:\n"), "{out}");
        // Nested field at 6-space indent (indent 2).
        assert!(out.contains("      x: 1.5"), "{out}");
        // The byte field renders its elements, not `<2 bytes>`.
        assert!(out.contains("    head: [255, 0]"), "{out}");
    }

    /// SECURITY: a wire STRING value is terminal-escape-sanitized before
    /// display: an injected ANSI/CSI escape
    /// is neutralized to U+FFFD, the printable tail survives.
    #[test]
    fn format_leaf_value_sanitizes_terminal_escapes_in_strings() {
        use cerulion_core::codegen::FrameValueKind as K;
        let hostile = "\u{1b}[2Jowned";
        let rendered = format_leaf_value(&K::Str(hostile), DEFAULT_ECHO_TRUNCATE_LENGTH);
        assert!(
            !rendered.contains('\u{1b}'),
            "raw ESC must be neutralized: {rendered:?}"
        );
        assert!(
            rendered.contains('\u{fffd}'),
            "control chars map to U+FFFD: {rendered:?}"
        );
        assert!(
            rendered.contains("owned"),
            "the printable tail survives: {rendered:?}"
        );
    }

    /// `resolve_schema_line` names a built-in and a workspace
    /// type via the LOCAL walker, falls back to the REMOTE walker for a type the
    /// desk lacks, and degrades to a hash-only line + hint for an unknown hash.
    #[test]
    fn resolve_schema_line_names_local_remote_and_unknown() {
        use cerulion_core::codegen::parse_rosmsg;
        use cerulion_core::{SchemaDoc, SchemaEncoding};

        // Built-in → named by the desk's built-ins-only walker.
        let builtin_walker = local_walker_from_workspace(None);
        let vec3_hash = builtin_walker
            .schema_hash_for("geometry_msgs/Vector3")
            .expect("geometry_msgs/Vector3 is a built-in");
        assert_eq!(
            resolve_schema_line(vec3_hash, &builtin_walker, None),
            format!("Schema: geometry_msgs/Vector3 (0x{vec3_hash:016x})")
        );

        // Workspace `.msg` store type → named by the workspace walker.
        let text = "float64 x\nfloat64 y\n";
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("demo_msgs").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Widget.msg"), text).unwrap();
        let ws_walker = local_walker_from_workspace(Some(tmp.path()));
        let widget_hash = parse_rosmsg(text, "Widget", Some("demo_msgs"))
            .unwrap()
            .schema_hash();
        assert_eq!(
            resolve_schema_line(widget_hash, &ws_walker, None),
            format!("Schema: demo_msgs/Widget (0x{widget_hash:016x})")
        );

        // Unknown hash → hash-only + a naming hint.
        let unknown = 0xDEAD_BEEF_DEAD_BEEFu64;
        let unknown_line = resolve_schema_line(unknown, &builtin_walker, None);
        assert!(
            unknown_line.contains(&format!("Schema hash: 0x{unknown:016x}")),
            "{unknown_line}"
        );
        assert!(
            unknown_line.contains("schema name unresolved"),
            "{unknown_line}"
        );

        // Remote-walker fallback names a type the desk genuinely lacks.
        let gadget_text = "float64 z\n";
        let remote = (
            "ubuntu".to_string(),
            seed_framewalker(&[SchemaDoc {
                qualified: "demo_msgs/Gadget".into(),
                encoding: SchemaEncoding::Msg,
                text: gadget_text.into(),
                deps: vec![],
            }]),
        );
        let gadget_hash = parse_rosmsg(gadget_text, "Gadget", Some("demo_msgs"))
            .unwrap()
            .schema_hash();
        assert_eq!(
            resolve_schema_line(gadget_hash, &builtin_walker, Some(&remote)),
            format!("Schema: demo_msgs/Gadget (0x{gadget_hash:016x})")
        );
    }

    /// Security: a wire-derived field NAME is
    /// terminal-escape-sanitized before display — a hostile robot's `.msg` field
    /// named `x\x1b[2J…` must not clear the operator's terminal. Both the
    /// top-level and the NESTED (recursed) name paths are covered.
    /// `topic echo`'s field render is bounded globally, not
    /// just per array. The pathological shape is real —
    /// `visualization_msgs/MarkerArray` is `Marker[] markers` where each
    /// `Marker` carries `Point[] points` + `ColorRGBA[] colors`, and with
    /// `truncate_length` re-applied at every level that renders ~1.5×10^5 lines
    /// per frame. Without the canonical element framing those
    /// fields would be ONE opaque line each, so an unbounded render is a straight
    /// regression of the verb.
    #[test]
    fn render_frame_fields_bounds_total_lines_for_nested_element_arrays() {
        use cerulion_core::codegen::{FrameValue, FrameValueKind as K, NamedValue};

        // A `Marker`-shaped element: two nested element arrays of 128 leaves.
        let leaf = || {
            K::Nested(Box::new(FrameValue {
                schema_name: "geometry_msgs/Point".into(),
                fields: vec![
                    NamedValue {
                        name: "x".into(),
                        value: K::F64(1.0),
                    },
                    NamedValue {
                        name: "y".into(),
                        value: K::F64(2.0),
                    },
                ],
            }))
        };
        let leaves = || K::NestedArray {
            elements: (0..DEFAULT_ECHO_TRUNCATE_LENGTH).map(|_| leaf()).collect(),
            raw: &[],
        };
        let marker = || {
            K::Nested(Box::new(FrameValue {
                schema_name: "visualization_msgs/Marker".into(),
                fields: vec![
                    NamedValue {
                        name: "points".into(),
                        value: leaves(),
                    },
                    NamedValue {
                        name: "colors".into(),
                        value: leaves(),
                    },
                ],
            }))
        };
        let frame = FrameValue {
            schema_name: "visualization_msgs/MarkerArray".into(),
            fields: vec![NamedValue {
                name: "markers".into(),
                value: K::NestedArray {
                    elements: (0..DEFAULT_ECHO_TRUNCATE_LENGTH)
                        .map(|_| marker())
                        .collect(),
                    raw: &[],
                },
            }],
        };

        let mut buf: Vec<u8> = Vec::new();
        render_frame_fields(&mut buf, &frame, 1, DEFAULT_ECHO_TRUNCATE_LENGTH);
        let out = String::from_utf8(buf).unwrap();
        let rendered = out.lines().count();
        // Bounded, and the bound is what did the bounding: an unbounded render
        // of this frame is 128 × (1 + 2 × (1 + 128 × 3)) ≈ 9.9×10^4 lines.
        assert!(
            rendered <= ECHO_MAX_LINES + 8,
            "render must be globally bounded, got {rendered} lines"
        );
        assert!(
            out.contains("output truncated at"),
            "the cut must be LOUD, never silent: {}",
            &out[out.len().saturating_sub(400)..]
        );

        // CONTROL (anti-tautology): an ordinary frame is untouched — every
        // field renders and there is NO truncation notice.
        let small = FrameValue {
            schema_name: "geometry_msgs/PoseArray".into(),
            fields: vec![NamedValue {
                name: "poses".into(),
                value: K::NestedArray {
                    elements: vec![leaf(), leaf()],
                    raw: &[],
                },
            }],
        };
        let mut buf: Vec<u8> = Vec::new();
        render_frame_fields(&mut buf, &small, 1, DEFAULT_ECHO_TRUNCATE_LENGTH);
        let out = String::from_utf8(buf).unwrap();
        assert!(!out.contains("output truncated at"), "{out}");
        assert!(out.contains("poses: [2 element(s)]"), "{out}");
        assert_eq!(
            out.matches("x: 1.0").count(),
            2,
            "both elements render: {out}"
        );
    }

    #[test]
    fn write_named_value_sanitizes_hostile_field_names() {
        use cerulion_core::codegen::{FrameValue, FrameValueKind as K, NamedValue};
        let hostile = "x\u{1b}[2J";
        let mut buf: Vec<u8> = Vec::new();
        write_named_value(
            &mut buf,
            hostile,
            &K::U32(5),
            0,
            DEFAULT_ECHO_TRUNCATE_LENGTH,
            &mut 0,
        );
        let out = String::from_utf8(buf).unwrap();
        assert!(
            !out.contains('\u{1b}'),
            "raw ESC in the field NAME must be neutralized: {out:?}"
        );
        assert!(
            out.contains('\u{fffd}'),
            "control chars in the name map to U+FFFD: {out:?}"
        );
        assert!(out.contains(": 5"), "the value still renders: {out:?}");

        // Nested field names are sanitized on their own recursion.
        let inner = FrameValue {
            schema_name: "demo/Inner".into(),
            fields: vec![NamedValue {
                name: hostile.into(),
                value: K::U8(1),
            }],
        };
        let mut buf2: Vec<u8> = Vec::new();
        write_named_value(
            &mut buf2,
            "outer",
            &K::Nested(Box::new(inner)),
            0,
            DEFAULT_ECHO_TRUNCATE_LENGTH,
            &mut 0,
        );
        let out2 = String::from_utf8(buf2).unwrap();
        assert!(
            !out2.contains('\u{1b}'),
            "a nested hostile field name is also sanitized: {out2:?}"
        );
    }

    /// A KNOWN schema whose frame fails to `walk()` (a
    /// truncated frame) is NAMED with the decode error before the hex — not
    /// silently dumped as if the type were unknown.
    #[test]
    fn write_custom_decode_known_but_undecodable_reports_decode_failed() {
        use cerulion_core::codegen::parse_rosmsg;
        use cerulion_core::wire::WireHeader;

        // Widget needs 16 payload bytes (two f64); give it 4 → FixedFieldOutOfBounds.
        let text = "float64 x\nfloat64 y\n";
        let tmp = tempfile::tempdir().unwrap();
        let msg_dir = tmp.path().join("demo_msgs").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Widget.msg"), text).unwrap();
        let local_walker = local_walker_from_workspace(Some(tmp.path()));
        let widget_hash = parse_rosmsg(text, "Widget", Some("demo_msgs"))
            .unwrap()
            .schema_hash();

        let header = WireHeader::with_schema(widget_hash);
        let truncated = vec![0u8; 4];
        let mut shown = false;
        let mut explained: std::collections::BTreeSet<u64> = std::collections::BTreeSet::new();
        let mut buf: Vec<u8> = Vec::new();
        write_custom_decode(
            &mut buf,
            "/lf/widget",
            &header,
            &truncated,
            &local_walker,
            None,
            &mut shown,
            &mut explained,
            DEFAULT_ECHO_TRUNCATE_LENGTH,
        );
        let out = String::from_utf8(buf).unwrap();
        assert!(
            out.contains("demo_msgs/Widget: decode failed"),
            "a known-but-undecodable frame must say so, not silently hex-dump: {out}"
        );
        assert!(
            out.contains("payload:"),
            "the hex fallback still follows the decode-failed line: {out}"
        );
    }

    /// The not-found message is pinned. It names the topic, states that
    /// both local and robots were searched, steers to `topic list`, carries the
    /// canonical-slash did-you-mean when a slashed twin exists, and is
    /// verb-neutral (no hardcoded "echo" — it is emitted by echo/info/hz).
    #[test]
    fn not_found_message_pins_text_and_did_you_mean() {
        let m = not_found_message("foo", false);
        assert!(
            m.contains("topic 'foo' not found locally or on any discovered robot"),
            "{m}"
        );
        assert!(m.contains("cerulion topic list"), "{m}");
        assert!(m.contains("observe"), "verb-neutral wording: {m}");
        assert!(
            !m.contains("echo it"),
            "must not hardcode the echo verb: {m}"
        );
        // A topic SEARCHED locally + remotely must never surface the
        // transport-level LOCAL error — "verify your graph YAML" belongs to a
        // genuine local-open failure, not to a topic absent (or stale) locally and
        // resolved through the remote rung. The stale-mirror fall-through routes to
        // THIS clean message (or a remote demand), never the misleading local one.
        assert!(
            !m.contains("graph YAML"),
            "the searched-not-found message must not blame the graph YAML: {m}"
        );
        assert!(
            !m.contains("did you mean"),
            "no slashed twin → no did-you-mean: {m}"
        );

        let m2 = not_found_message("foo", true);
        assert!(m2.contains("did you mean '/foo'?"), "{m2}");
        assert!(
            m2.contains("canonical Cerulion topics start with '/'"),
            "{m2}"
        );
    }

    /// The cold-start message is a DIFFERENT message, not a reworded
    /// not-found — the pin is that it makes no absence claim while
    /// [`not_found_message`] still does. Both directions asserted in one body so a
    /// future edit cannot quietly converge them.
    #[test]
    fn discovery_not_converged_message_states_unknown_not_absent() {
        let m = discovery_not_converged_message("/go2/camera/h264", false);
        // It names the topic and the two facts we actually have.
        assert!(m.contains("/go2/camera/h264"), "{m}");
        assert!(m.contains("not produced locally"), "{m}");
        assert!(m.contains("nothing on the network answered"), "{m}");
        // THE pin: it says UNKNOWN and explicitly disclaims the absence reading.
        assert!(m.contains("UNKNOWN"), "{m}");
        assert!(
            m.contains("not a claim that it is missing"),
            "the disclaimer is explicit, not implied by tone: {m}"
        );
        // It must NOT reuse the absence wording — that phrasing would
        // mis-educate a user when the topic is actually streaming.
        assert!(
            !m.contains("not found locally or on any discovered robot"),
            "the cold-start message must not assert absence: {m}"
        );
        assert!(
            !m.contains("does not exist"),
            "no absence claim anywhere: {m}"
        );
        // `NotConverged` also covers "a robot ANNOUNCED but never served
        // its catalog", so the message must not flatly assert nothing was discovered
        // — that would be false in that case, and would steer the user at the power
        // switch of a robot that is powered on. Both causes must be named.
        assert!(
            m.contains("no robot was discovered, or none served its catalog in time"),
            "the message must cover BOTH non-convergence causes: {m}"
        );
        assert!(
            !m.contains("no robot was discovered on the network"),
            "it must not assert the first cause as if it were the only one: {m}"
        );
        // Retry FIRST (discovery usually converges), then the checks — in that order,
        // because retrying is both likelier to work and free.
        let retry_at = m.find("Retry").expect("names the retry");
        let check_at = m.find("powered on").expect("names the check");
        assert!(retry_at < check_at, "retry is steered to first: {m}");
        assert!(m.contains("cerulion topic list"), "{m}");

        // ANTI-TAUTOLOGY / the other direction: the SETTLED path's message still
        // asserts absence (that claim is correct there and must not be softened —
        // otherwise the cold-start fix would have degraded a genuinely useful error).
        let settled = not_found_message("/go2/camera/h264", false);
        assert!(
            settled.contains("not found locally or on any discovered robot"),
            "the settled not-found message is UNCHANGED: {settled}"
        );
        assert!(!settled.contains("UNKNOWN"), "{settled}");
    }

    /// The retry clause must not claim a duration this command already
    /// out-waited.
    ///
    /// Both UNKNOWN messages are rendered AFTER a wait of up to
    /// `FIRST_CONTACT_CONVERGENCE_CEILING`, so "discovery often converges within a
    /// second or two" — sound advice to someone who waited nothing — would be a
    /// sentence contradicted by the counter the user just watched tick past ten.
    ///
    /// This pins the removal on BOTH messages (they share `RETRY_CLAUSE` for exactly
    /// that reason) and, in the same body, that the retry advice SURVIVES: the fix is
    /// to stop claiming a duration, not to stop telling the user to try again.
    #[test]
    fn the_retry_clause_claims_no_duration_this_command_already_out_waited() {
        let topic_msg = discovery_not_converged_message("/go2/camera/h264", false);
        let schema_msg = schema_discovery_not_converged_message("go2_msgs/Odom");
        for m in [&topic_msg, &schema_msg] {
            assert!(
                !m.contains("within a second or two"),
                "the command now waits ~10s before rendering this, so promising \
                 sub-second convergence contradicts what the user just watched: {m}"
            );
            // The advice itself is still there — and still FIRST, before the checks.
            let retry_at = m.find("Retry").expect("still names the retry");
            let check_at = m.find("powered on").expect("still names the check");
            assert!(retry_at < check_at, "retry is still steered to first: {m}");
            assert!(
                m.contains("longer to answer than this command waits"),
                "and it names the real shape — the robot is slower than our wait, \
                 not that our wait is short: {m}"
            );
            assert!(m.contains("cerulion topic list"), "{m}");
        }
        // ONE spelling across both paths: the whole reason `RETRY_CLAUSE` is a const.
        assert!(topic_msg.contains(RETRY_CLAUSE), "{topic_msg}");
        assert!(schema_msg.contains(RETRY_CLAUSE), "{schema_msg}");
    }

    /// The stderr lines the wait prints, against hand-written text.
    ///
    /// They are the ONLY user-visible evidence that a ten-second command is doing
    /// something rather than hanging, so their content is a contract.
    #[test]
    fn the_convergence_lines_say_what_is_happening_and_for_how_long() {
        use std::time::Duration;
        // ONE DECIMAL — whole seconds made the first two polls of a warm daemon both
        // render `(0s)`, so the counter looked stalled exactly when the user is
        // deciding whether the command is hung.
        assert_eq!(
            convergence_progress_line(Duration::from_millis(3_940)),
            "discovering robots on the network… (3.9s)"
        );
        assert_eq!(
            convergence_progress_line(Duration::from_millis(700)),
            "discovering robots on the network… (0.7s)"
        );
        assert_eq!(
            convergence_progress_line(Duration::ZERO),
            "discovering robots on the network… (0.0s)"
        );
        // The give-up line quotes NO duration: the last progress line already stated
        // the elapsed, and a figure here would be one in-flight round trip past the
        // ceiling the docs name.
        let up = convergence_gave_up_line();
        assert_eq!(
            up,
            "…giving up on discovery — nothing on the network answered"
        );
        // It must never read as an absence claim — that is the whole classification
        // contract, and this line sits immediately above the message that carries it.
        assert!(!up.contains("not found"), "{up}");
        assert!(!up.contains("does not exist"), "{up}");
    }

    /// A wait the user interrupted renders as an interruption, never
    /// as an absence claim and never as the "we searched and read nothing" claim.
    ///
    /// If `WaitOutcome::Cancelled` were consumed at exactly one place — the
    /// one-line epitaph — and then DISCARDED, the cancelled (empty, NotConverged)
    /// answer would flow on into `classify_unmatched_catalog` and the command would exit
    /// nonzero with a paragraph asserting the topic's existence was UNKNOWN and
    /// telling the user to check the robot's power switch. Only a distinct resolve
    /// arm can separate them, which is why `RemoteResolve::Cancelled` exists.
    ///
    /// The negative half is the whole point: this message must carry NONE of the
    /// vocabulary the other two messages use to make claims.
    #[test]
    fn a_cancelled_wait_renders_an_interruption_not_an_absence_claim() {
        let m = unresolved_remote_error("/go2/camera/h264", RemoteResolve::Cancelled).to_string();
        assert!(
            m.contains("interrupted"),
            "it must name the interruption: {m}"
        );
        assert!(m.contains("/go2/camera/h264"), "{m}");
        assert!(
            m.contains("nothing was concluded"),
            "and say explicitly that it supports no conclusion: {m}"
        );
        // NOT an absence claim, NOT the unknown-existence claim, and no
        // retry/power-switch steer — the user chose to stop.
        for forbidden in [
            "not found",
            "does not exist",
            "UNKNOWN",
            "powered on",
            "Retry",
            "nothing on the network answered",
        ] {
            assert!(
                !m.contains(forbidden),
                "a cancelled observation supports NO claim, found {forbidden:?}: {m}"
            );
        }

        // The SCHEMA path routes through the same message.
        let s = schema_info_remote_outcome(
            "go2_msgs/Odom",
            RemoteSchemaFetch::Cancelled,
            CliError::Validation("the LOCAL not-found, which must NOT be raised".into()),
        )
        .expect_err("a cancelled fetch is an error, not a served schema")
        .to_string();
        assert!(
            s.contains("interrupted") && s.contains("go2_msgs/Odom"),
            "{s}"
        );
        assert!(
            !s.contains("must NOT be raised"),
            "a cancelled fetch must not re-raise the local not-found — that is an \
             absence claim in the caller's words: {s}"
        );

        // ANTI-TAUTOLOGY: the two non-cancelled arms still make their claims, so a
        // "fix" that blanket-softened every message would fail here.
        let absent = unresolved_remote_error("/go2/x", RemoteResolve::NoProducer).to_string();
        assert!(absent.contains("not found locally or on any discovered robot"));
        let unknown =
            unresolved_remote_error("/go2/x", RemoteResolve::DiscoveryNotConverged).to_string();
        assert!(unknown.contains("UNKNOWN"));
    }

    /// The clean-exit predicate the two observer verbs guard on.
    ///
    /// Drives the SHIPPED helper, not a copy. `None` (a caller with no handler —
    /// `topic info`, `schema info`) is never interrupted, which is what stops a
    /// genuine error from being swallowed on those paths.
    #[test]
    fn an_interrupted_command_exits_quietly_only_when_a_flag_was_actually_cleared() {
        use std::sync::atomic::AtomicBool;
        let live = AtomicBool::new(true);
        let stopped = AtomicBool::new(false);
        assert!(
            interrupted_before_observing(Some(&stopped)),
            "a cleared flag IS the user's Ctrl-C"
        );
        assert!(
            !interrupted_before_observing(Some(&live)),
            "a live run's error must still be reported"
        );
        assert!(
            !interrupted_before_observing(None),
            "a caller with no cancellation source is never interrupted — otherwise a \
             genuine error on `topic info` / `schema info` would be swallowed"
        );
    }

    /// The give-up line fires ONLY when the LOOP ITSELF gave up, and only
    /// when the user actually saw a counter to close.
    ///
    /// The first version of this guard keyed on `(discovery == NotConverged &&
    /// waited != 0)` — two proxies which the trust gate makes BOTH true of a
    /// wait that polled and then FOUND the topic, so a successfully resolved topic
    /// printed "nothing on the network answered" before streaming its frames. Only
    /// the loop's own terminal decision distinguishes them, and this drives the
    /// SHIPPED predicate rather than a transliteration of it.
    #[test]
    fn the_give_up_line_is_emitted_only_after_a_real_unconverged_wait() {
        use cerulion_netd::WaitOutcome;
        assert!(
            should_note_give_up(WaitOutcome::GaveUp, 3),
            "the case it exists for: the loop gave up and the user saw a counter"
        );
        assert!(
            !should_note_give_up(WaitOutcome::Answered, 3),
            "a wait that SUCCEEDED gets no epitaph — even one that polled first \
             (the trust-gate shape, where its `discovery` still reads NotConverged)"
        );
        assert!(
            !should_note_give_up(WaitOutcome::Cancelled, 3),
            "a Ctrl-C concluded NOTHING about the network"
        );
        assert!(
            !should_note_give_up(WaitOutcome::GaveUp, 0),
            "the no-wait posture printed nothing, so there is nothing to close — an \
             epitaph would be the only output a silent fallback ever produced"
        );
        assert!(!should_note_give_up(WaitOutcome::Answered, 0));
    }

    /// The two emitters write through ONE seam, and a test can pin exactly
    /// what reaches it.
    ///
    /// The STREAM choice (stderr, never stdout — `topic echo`'s frames and
    /// `topic hz`'s rate lines are stdout and a Studio-class parser must not get a
    /// spinner spliced in) is pinned structurally in
    /// `tests/convergence_adoption_test.rs`, because an in-process test cannot
    /// observe which of the process's own streams a write landed on. This pins the
    /// other half: that the seam receives the exact bytes, newline-terminated.
    #[test]
    fn convergence_lines_go_through_one_write_seam() {
        let mut sink: Vec<u8> = Vec::new();
        write_convergence_line(
            &mut sink,
            &convergence_progress_line(std::time::Duration::ZERO),
        );
        write_convergence_line(&mut sink, &convergence_gave_up_line());
        let text = String::from_utf8(sink).expect("utf-8");
        assert_eq!(
            text,
            format!(
                "{}\n{}\n",
                convergence_progress_line(std::time::Duration::ZERO),
                convergence_gave_up_line()
            ),
            "each line is written once, newline-terminated, in order"
        );
    }

    /// The ROUTING from a resolve outcome to the user's error — the arm a
    /// false absence claim surfaces through. Pins that each of the three failures
    /// reaches ITS OWN message; a mis-routed arm (the easy regression, since all
    /// three are `CliError::Validation`) fails here.
    #[test]
    fn unresolved_remote_error_routes_each_failure_to_its_own_message() {
        // Discovery RAN, nobody has it → the absence claim (correct here).
        let m = unresolved_remote_error("/t", RemoteResolve::NoProducer).to_string();
        assert!(
            m.contains("not found locally or on any discovered robot"),
            "{m}"
        );
        assert!(!m.contains("UNKNOWN"), "{m}");

        // Discovery never ran → UNKNOWN, and NOT the absence wording.
        let m = unresolved_remote_error("/t", RemoteResolve::DiscoveryNotConverged).to_string();
        assert!(m.contains("nothing on the network answered"), "{m}");
        assert!(m.contains("UNKNOWN"), "{m}");
        assert!(
            !m.contains("not found locally or on any discovered robot"),
            "the cold-start arm must NOT route to the absence message: {m}"
        );

        // A robot HAS it but its schema is unresolvable → names robot + cause, and is
        // likewise never the absence message (that contract still holds).
        let m = unresolved_remote_error(
            "/t",
            RemoteResolve::SchemaUnavailable {
                robot: "go2".to_string(),
                cause: "older binary".to_string(),
            },
        )
        .to_string();
        assert!(
            m.contains("announced by robot 'go2'") && m.contains("older binary"),
            "{m}"
        );
        assert!(
            !m.contains("not found locally or on any discovered robot") && !m.contains("UNKNOWN"),
            "{m}"
        );
        // Robot + cause stay terminal-escape-sanitized through the new routing point.
        let m = unresolved_remote_error(
            "/t",
            RemoteResolve::SchemaUnavailable {
                robot: "ro\x1b[31mbot".to_string(),
                cause: "ca\x1b[0muse".to_string(),
            },
        )
        .to_string();
        assert!(!m.contains('\x1b'), "sanitized: {m:?}");

        // The structurally-unreachable `Found` arm is a LOUD internal error, not a
        // panic (a CLI verb must not abort over our own routing bug) and not a silent
        // swallow (it names the invariant so it is greppable).
        let m = unresolved_remote_error(
            "/t",
            RemoteResolve::Found(RemoteIngressTarget {
                robot: "go2".to_string(),
                schema_name: "pkg/T".to_string(),
                schema_hash: 1,
                walker: cerulion_core::codegen::FrameWalker::new(Vec::new()).0,
            }),
        )
        .to_string();
        assert!(m.contains("internal error"), "{m}");
        assert!(m.contains("Cerulion bug"), "{m}");
        assert!(
            !m.contains("not found locally or on any discovered robot") && !m.contains("UNKNOWN"),
            "an internal routing bug must not masquerade as a discovery verdict: {m}"
        );
    }

    /// The TRANSIENT path's classifier. Same rule as the netd
    /// one, derived from the gather instead of a daemon-reported state: we may only
    /// claim absence about catalogs we actually READ.
    #[test]
    fn classify_transient_miss_only_claims_absence_when_a_catalog_was_read() {
        assert!(
            matches!(
                classify_transient_miss("/t", false),
                SchemaResolve::NoProducer
            ),
            "we read at least one robot's catalog and it lacks the topic — a real absence"
        );
        assert!(
            matches!(
                classify_transient_miss("/t", true),
                SchemaResolve::DiscoveryNotConverged
            ),
            "we read NOBODY's catalog — that licenses no absence claim (it was the \
             pre-916 silent 'not found on any discovered robot')"
        );
    }

    /// The classifier that decides WHICH of those two messages a caller
    /// gets, driven off netd's reported discovery state. Hand oracle, both arms.
    #[cfg(unix)]
    #[test]
    fn classify_unmatched_catalog_only_claims_absence_from_a_settled_daemon() {
        use cerulion_netd::DiscoveryState;
        assert!(
            matches!(
                classify_unmatched_catalog("/t", DiscoveryState::Settled),
                SchemaResolve::NoProducer
            ),
            "a settled daemon searched the LAN — nothing matching IS absence"
        );
        assert!(
            matches!(
                classify_unmatched_catalog("/t", DiscoveryState::NotConverged),
                SchemaResolve::DiscoveryNotConverged
            ),
            "a daemon that never saw a robot searched NOTHING — never an absence claim"
        );
    }

    /// The SCHEMA-path classifier — the twin of
    /// `classify_unmatched_catalog`, for the arm that was left open. `schema info`'s
    /// netd fetch DROPPED netd's discovery verdict entirely and re-raised the terminal
    /// local "schema not found", so a cold daemon that had searched nothing rendered
    /// as proof the type does not exist. Hand oracle, both arms.
    #[cfg(unix)]
    #[test]
    fn classify_unserved_schema_only_claims_absence_from_a_settled_daemon() {
        use cerulion_netd::DiscoveryState;
        assert!(
            matches!(
                classify_unserved_schema("pkg/T", DiscoveryState::Settled),
                RemoteSchemaFetch::NotServed
            ),
            "a settled daemon asked the LAN — nobody serving it IS a real absence, and \
             the caller may re-raise its local not-found"
        );
        assert!(
            matches!(
                classify_unserved_schema("pkg/T", DiscoveryState::NotConverged),
                RemoteSchemaFetch::DiscoveryNotConverged
            ),
            "a daemon that never saw a robot asked NOBODY — never an absence claim"
        );
    }

    /// A panicked sibling must never suppress a schema some
    /// other robot actually served.
    ///
    /// A wrong-result class.
    /// Asking "is this pass conclusive?" before looking at what came
    /// back means a pass that lost robot A's worker discards robot B's documents
    /// and reports `DiscoveryNotConverged` — a guard against confident ABSENCE
    /// suppressing a PRESENCE the desk is already holding.
    ///
    /// BOTH SIDES, because either alone is satisfied by a degenerate rule: a
    /// classifier that always serves whatever it finds passes (a) while making the
    /// absence claim it must not make, and one that always demotes passes (b)
    /// while throwing away answers.
    #[test]
    fn a_panicked_sibling_never_suppresses_a_schema_another_robot_served() {
        use cerulion_core::transport::discovery::GatherReplies;
        use cerulion_core::{SchemaDoc, SchemaReply};

        let doc = SchemaDoc {
            qualified: "pkg/T".to_string(),
            encoding: cerulion_core::SchemaEncoding::Msg,
            text: "uint32 x\n".to_string(),
            deps: Vec::new(),
        };

        // (a) A worker PANICKED and a sibling SERVED the type. The document is in
        // hand; nothing the pass failed to ask can make it less true.
        let served = classify_schema_gather(
            "pkg/T",
            2,
            GatherReplies {
                replies: vec![SchemaReply::found("go2", "pkg/T", vec![doc.clone()])],
                panicked: vec!["orin".to_string()],
            },
        );
        match served {
            RemoteSchemaFetch::Found(reply) => {
                assert_eq!(reply.robot, "go2");
                assert_eq!(reply.docs.len(), 1, "the served closure travels intact");
            }
            // `RemoteSchemaFetch` carries a `SchemaReply` and derives no `Debug`,
            // so the arm is named by hand rather than formatted.
            RemoteSchemaFetch::NotServed => panic!(
                "a found document must be SERVED however the pass ended — got \
                 NotServed, i.e. the pass turned this binary's own bug into a \
                 confident claim that nobody serves the type"
            ),
            RemoteSchemaFetch::DiscoveryNotConverged => panic!(
                "a found document must be SERVED however the pass ended — got \
                 DiscoveryNotConverged, the defect this test pins: the rule demotes a \
                 confident ABSENCE, it never suppresses a PRESENCE"
            ),
            RemoteSchemaFetch::Cancelled => panic!("nothing here cancels a wait"),
        }

        // (b) A worker PANICKED and NOTHING usable came back. Now the question is
        // whether "nobody has it" is a claim this pass may make, and it is not.
        assert!(
            matches!(
                classify_schema_gather(
                    "pkg/T",
                    2,
                    GatherReplies {
                        replies: vec![SchemaReply::not_found("go2", "pkg/T", "unknown type")],
                        panicked: vec!["orin".to_string()],
                    },
                ),
                RemoteSchemaFetch::DiscoveryNotConverged
            ),
            "one robot said `I do not have it` and the other was never really asked, \
             so `nobody serves this type` is a claim about a set this pass did not read"
        );

        // ANTI-TAUTOLOGY for (b): the SAME shape with every worker healthy IS a
        // genuine absence. Without this, (b) is satisfied by a rule that never
        // claims absence at all.
        assert!(
            matches!(
                classify_schema_gather(
                    "pkg/T",
                    1,
                    GatherReplies {
                        replies: vec![SchemaReply::not_found("go2", "pkg/T", "unknown type")],
                        panicked: Vec::new(),
                    },
                ),
                RemoteSchemaFetch::NotServed
            ),
            "every robot asked and ANSWERED, and none has it — a real absence the \
             caller may re-raise as its local not-found"
        );

        // The empty gather keeps its earlier meaning on both completeness
        // values: nothing was read, so nothing is licensed.
        for panicked in [Vec::new(), vec!["orin".to_string()]] {
            assert!(
                matches!(
                    classify_schema_gather(
                        "pkg/T",
                        1,
                        GatherReplies {
                            replies: Vec::<SchemaReply>::new(),
                            panicked,
                        },
                    ),
                    RemoteSchemaFetch::DiscoveryNotConverged
                ),
                "an empty gather read nothing, whoever it did or did not reach"
            );
        }
    }

    /// The schema-path not-converged message. It must state BOTH facts
    /// (locally absent AND network-unknown) without collapsing into either the local
    /// "not found" it replaces or a bare "unknown" that hides the typo case.
    #[test]
    fn schema_discovery_not_converged_message_states_unknown_not_absent() {
        let m = schema_discovery_not_converged_message("go2_msgs/Odom");
        assert!(m.contains("go2_msgs/Odom"), "names the type: {m}");
        // FACT 1 — genuinely not resolvable locally (so a typo still reads as a typo).
        assert!(
            m.contains("not in this workspace or the built-in ROS 2 registry"),
            "states the LOCAL fact: {m}"
        );
        // FACT 2 — and the network answer is UNKNOWN, explicitly not an absence.
        assert!(m.contains("Nothing on the network answered"), "{m}");
        assert!(m.contains("UNKNOWN"), "{m}");
        assert!(
            m.contains("not a claim that it does not exist"),
            "the disclaimer is explicit, not implied by tone: {m}"
        );
        // Both non-convergence causes named (the rule its topic twin holds):
        // asserting only "no robot was discovered" would be false when a robot IS
        // announced but never served, and would steer the user at a power switch.
        assert!(
            m.contains("no robot was discovered, or none served its schemas in time"),
            "the message must cover BOTH non-convergence causes: {m}"
        );
        // Retry FIRST, then the checks — same ordering rule as the topic twin.
        let retry_at = m.find("Retry").expect("names the retry");
        let check_at = m.find("powered on").expect("names the check");
        assert!(retry_at < check_at, "retry is steered to first: {m}");
        assert!(m.contains("cerulion topic list"), "{m}");

        // Every clause must be TRUE in every rendering state.
        //
        // (a) This message also renders on the TRANSIENT path (explicit locators, or a
        // netd that was unreachable), where NO daemon was involved — and a
        // test pins exactly that netd-free shape. So it must not name
        // cerulion-netd as the thing that heard nothing.
        assert!(
            !m.contains("cerulion-netd"),
            "the transient path renders this with no daemon involved — naming \
             cerulion-netd would be false there: {m}"
        );
        // (b) The LOCAL fact must carry a LOCAL remedy, and lead: the common case is a
        // typo on a desk with no robots, where everything about robots is noise.
        let local_remedy_at = m
            .find("add its `.msg`/YAML under `schemas/`")
            .expect("names the local remedy");
        let network_at = m
            .find("Nothing on the network")
            .expect("names the network half");
        assert!(
            local_remedy_at < network_at,
            "the LOCAL remedy must lead — a typo on a robot-less desk is the common \
             case and the network half is noise for it: {m}"
        );
        assert!(m.contains("cerulion schema list"), "{m}");
    }

    /// The canonical-slash did-you-mean must ride EVERY unresolved-topic
    /// message, because it states a purely LOCAL fact (`topic_list()` touches no
    /// network) and is therefore knowable in every network state.
    ///
    /// If it rode only `not_found_message`, the TWO other ways
    /// for a topic to come up unresolved — non-convergence and the kill-switch —
    /// would silently lose the affordance: on a robot-less desk
    /// `topic info foo/bar` would not say "did you mean '/foo/bar'?" about a topic
    /// streaming in local SHM at that moment, and would report the network unreachable and
    /// the answer UNKNOWN instead.
    ///
    /// Hand oracle over all three messages, BOTH directions (present with a twin,
    /// absent without one — the second half is what stops a "fix" that staples the
    /// hint onto everything unconditionally).
    #[test]
    fn the_canonical_slash_hint_rides_every_unresolved_topic_message() {
        let with: [(&str, String); 3] = [
            ("found-nowhere", not_found_message("foo/bar", true)),
            (
                "not-converged",
                discovery_not_converged_message("foo/bar", true),
            ),
            (
                "kill-switched",
                kill_switch_unavailable_message("foo/bar", None, true),
            ),
        ];
        for (which, m) in &with {
            assert!(
                m.contains("did you mean '/foo/bar'?"),
                "the {which} message must keep the canonical-slash hint: {m}"
            );
            assert!(
                m.contains("canonical Cerulion topics start with '/'"),
                "…and explain WHY, identically on every path: {m}"
            );
        }

        // NEGATIVE: no local twin ⇒ no hint anywhere (a hint stapled on
        // unconditionally would be a confident lie about a topic that is not there).
        let without: [(&str, String); 3] = [
            ("found-nowhere", not_found_message("/foo/bar", false)),
            (
                "not-converged",
                discovery_not_converged_message("/foo/bar", false),
            ),
            (
                "kill-switched",
                kill_switch_unavailable_message("/foo/bar", None, false),
            ),
        ];
        for (which, m) in &without {
            assert!(
                !m.contains("did you mean"),
                "the {which} message must NOT invent a twin that does not exist: {m}"
            );
        }

        // The fragment itself: empty without a twin, and it names the SLASHED
        // spelling (the thing to type), not the one the user typed.
        assert_eq!(canonical_slash_hint("foo/bar", false), "");
        let frag = canonical_slash_hint("foo/bar", true);
        assert!(frag.contains("'/foo/bar'"), "{frag}");
    }

    /// The `cerulion schema info` ROUTING — which of the three
    /// [`RemoteSchemaFetch`] outcomes renders what. No CLI test can reach this
    /// arm: replacing the `DiscoveryNotConverged` branch with
    /// `return Err(local_err)` passes every one of them,
    /// because every `schema_cli_test` arm sets `CERULION_NETWORK=off` and so routes
    /// kill-switch → `NotServed` → `local_err`, never reaching that branch.
    ///
    /// Hand oracles over ALL THREE variants, each asserted against the OTHER two's
    /// rendering so a mis-route cannot pass (all three outcomes are otherwise just
    /// "an error" or "a reply").
    #[test]
    fn schema_info_remote_outcome_routes_each_variant_to_its_own_rendering() {
        let local = || CliError::SchemaNotFound {
            name: "go2_msgs/Odom".to_string(),
            remedy: "checked the workspace and the built-in ROS 2 registry.",
        };
        let local_text = local().to_string();

        // FOUND → the reply comes back for the caller to render (never an error).
        let reply = cerulion_core::SchemaReply {
            version: 1,
            robot: "go2".to_string(),
            requested: "go2_msgs/Odom".to_string(),
            docs: vec![],
            error: None,
        };
        let out =
            schema_info_remote_outcome("go2_msgs/Odom", RemoteSchemaFetch::Found(reply), local())
                .expect("a served schema is not an error");
        assert_eq!(out.robot, "go2", "provenance survives the routing point");

        // NOT SERVED → the LOCAL error verbatim. Discovery ran and nobody has it, so
        // the terminal "schema not found" is the correct answer and its text (and
        // therefore its exit code) must be byte-unchanged.
        let err =
            schema_info_remote_outcome("go2_msgs/Odom", RemoteSchemaFetch::NotServed, local())
                .expect_err("a settled absence is an error");
        assert_eq!(
            err.to_string(),
            local_text,
            "a settled absence re-raises the LOCAL error VERBATIM"
        );
        assert!(!err.to_string().contains("UNKNOWN"), "{err}");

        // NOT CONVERGED → the UNKNOWN message, and explicitly NOT the local
        // "not found" (the mutation this test exists to kill returns exactly that).
        let err = schema_info_remote_outcome(
            "go2_msgs/Odom",
            RemoteSchemaFetch::DiscoveryNotConverged,
            local(),
        )
        .expect_err("a cold start is still an error — just a different one");
        let text = err.to_string();
        assert!(text.contains("UNKNOWN"), "{text}");
        assert!(text.contains("Nothing on the network answered"), "{text}");
        assert!(
            text.contains("not a claim that it does not exist"),
            "{text}"
        );
        assert_ne!(
            text, local_text,
            "THE pin: a cold start must NOT re-raise the terminal local 'schema not \
             found' — that is a claim with no evidence, and reverting this arm to \
             `Err(local_err)` is the exact mutation this test exists to kill"
        );
    }

    #[test]
    fn kill_switch_unavailable_message_is_honest_about_a_local_mirror() {
        // A topic LISTED locally as a netd MIRROR under the
        // kill-switch names the origin robot + BOTH remedies — NEVER "not found".
        let m = kill_switch_unavailable_message("/utlidar/robot_odom", Some("ubuntu"), false);
        assert!(
            m.contains("exists locally as a network mirror") && m.contains("ubuntu"),
            "names the mirror + its robot: {m}"
        );
        assert!(
            m.contains("unset CERULION_NETWORK")
                && m.contains("read it directly on robot 'ubuntu'"),
            "names BOTH remedies (unset the kill-switch OR read on the robot): {m}"
        );
        assert!(
            !m.contains("not found locally"),
            "a listed mirror must NOT claim 'not found locally': {m}"
        );
        // The robot is terminal-escape-sanitized.
        let bad = kill_switch_unavailable_message("/t", Some("ro\x1b[31mbot"), false);
        assert!(!bad.contains('\x1b'), "robot is sanitized: {bad:?}");

        // None (genuinely absent) → the unchanged not-found message.
        let none = kill_switch_unavailable_message("/gone", None, false);
        assert!(
            none.contains("not found locally")
                && none.contains("CERULION_NETWORK=off")
                && none.contains("search robots on the LAN"),
            "the genuine-absent arm is unchanged: {none}"
        );
        assert!(
            !none.contains("exists locally as a network mirror"),
            "the absent arm never claims a mirror: {none}"
        );
    }
}
