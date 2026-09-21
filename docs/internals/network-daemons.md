# Network daemons & discovery: contributor dossier

Scope: `cerulion_netd` (the per-computer network daemon), `cerulion_dds` (the DDS
platform crate behind `cerulion ros2 attach`), and the substrate crates whose
boundaries are load-bearing: `cerulion_discovery`, `cerulion_wireclient`,
`cerulion_mdns`. Read this alongside `docs/networking.md` (user-facing model) and
each crate's `AGENTS.md`. Everything below is present-tense contract; the enforcing
test for each is in the test map at the end.

## 1. Topology: one daemon per computer

`cerulion-netd` is the ONE network daemon per machine. It owns the machine's single
zenoh session (one-session-per-process is a core principle) and serves every desk
consumer (`topic echo`/`info`/`hz`, `schema info`, the viz daemon, user-graph
ingress) over a Unix-domain-socket NDJSON control seam. Many consumers demanding
the same remote `(robot, topic)` share ONE mirror: the frame crosses the network
once, re-injects into desk SHM once, and every reader subscribes to that mirror.
The daemon is spawned detached by the first consumer that needs it, refcounts its
demands, and self-exits when idle.

The crate is lib + bin: the reusable daemon, protocol, refcount registry, and
hygiene live in the lib so tests drive an in-process daemon over a temp socket with
an injected (spy) mirror plane: parallel-safe, no zenoh. `src/main.rs` is the thin
production entry (transport init + the real planes + signal handling).

Feature rule: `wan` (the iroh WAN plane) is DEFAULT-ON, and netd is deliberately
excluded from workspace `default-members`: a plain `cargo build` stays iroh-free
while a workspace-root `cargo build -p cerulion_netd` produces the WAN-capable
daemon. The lean consumers (`cerulion_cli_engine`, the viz daemon) depend on netd
with `default-features = false` so cargo's feature unification never drags the iroh
tree into their builds. Build the daemon with `cargo build -p cerulion_netd`
from inside the workspace checkout (it is a workspace member and resolves
through the root `Cargo.lock`). netd itself pulls no DDS crate; the published
`cerulion-rustdds`/`cerulion-ros2-client` forks are `cerulion_dds`'s
dependency and resolve from crates.io. For unpublished fork work, add the
temporary local overrides described in `docs/packaging/dds-forks.md` and
remove them before committing.

## 2. Demand / mirror plane

- **Control seam**: newline-delimited JSON over the UDS, one object per line. An
  oversized line is dropped; a malformed line is survived (the connection lives on).
- **Refcounted demands**: the FIRST demand for a stream ensures the mirror; a
  same-connection re-demand is idempotent; two connections share one mirror. A
  schema conflict is refused. A mirror-ensure failure is refused with the refcount
  rolled back: no phantom entry.
- **Connection close IS the release**: every demand a connection holds is released
  when it closes, so a crashed consumer can never leak a demand. The explicit
  `release` verb decrements early; drop releases the rest.
- **Mirror identity**: the mirror is keyed by the CANONICAL TOPIC name, not
  `(robot, topic)`: a consumer subscribing to the topic must find the mirror at
  that exact SHM name ("one wire name per topic, no per-machine aliasing"). The
  robot half of a demand is PROVENANCE: it rides the refcount key and the
  mirror-provenance registry, never the SHM name.
- **Provenance**: `ensure_mirror` registers the mirror's provenance
  (`TransportManager::register_mirror_provenance`) at the re-injection point, so
  `topic list` and the viz sidebar fold the mirrored topic into the REMOTE section
  attributed to its origin robot instead of surfacing a phantom local topic.
  Best-effort: a provenance failure never fails the demand.
- **Teardown**: the LAST release retires the mirror entry and frees the
  single-writer SHM slot; a re-demand re-creates the mirror (full cycle). The pure
  `process_released_key` guard re-checks refcount == 0 UNDER the registry lock
  before the destructive release, so a racing re-demand cannot lose its mirror. A
  double-release after teardown is a loud NotHeld error, never silent.
- **Restart self-heal**: a held demand survives a robot-side gateway restart; the
  mirror resumes delivering the restarted robot's frames with no client-side
  re-demand (the daemon re-affirms demand across the fresh link).

## 3. Daemon lifecycle: spawn, readiness, idle exit, singleton

- **Spawn**: the first consumer spawns the daemon detached, then waits for
  READINESS, not a fixed budget (`SPAWN_READY_TIMEOUT` is the per-attempt ceiling;
  connect-or-spawn may try twice, so both expiring compounds). A spawned daemon
  that exits immediately is reported loudly within a short post-exit grace, with an
  error naming the socket, the wait, and the child's death, the "died" vs "still
  booting" discriminator, never a silent multi-second stare. Shipped bounds are
  oracle-pinned (ceiling within a sane band; grace well under the ceiling).
- **`ChildLiveness`**: an errored `try_wait` neither fail-fasts nor claims the
  child is alive; an unobservable child is treated as unknown.
- **Binary resolution**: `resolve_netd_bin_from` is a pure ladder over an injected
  existence predicate: explicit `CERULION_NETD_BIN` override (verbatim and
  exclusive, so a wrong override fails naming itself) → sibling of
  `current_exe()` → sibling of the CANONICALIZED `current_exe()`. The third rung
  exists because macOS does not canonicalize `current_exe()` (a binary invoked
  through a symlink reports the symlink's path), so symlink-based install layouts
  would otherwise fail every sibling lookup. The failure message carries every
  path tried.
- **Idle self-exit**: after the last demand releases and the idle grace elapses,
  the daemon unlinks its socket FIRST, then makes an atomic exit decision; clients
  retry on connect failure. This ordering is what prevents a half-dead daemon from
  holding the socket and black-holing every later connection. A LISTEN-configured
  daemon (a standing gateway; see §4) spawns no idle-watch at all: it exits on
  SIGINT/SIGTERM only, because its network presence is held by configuration, not
  by a refcounted demand.
- **Singleton**: an exclusive non-blocking `flock` on the pidfile. Only the lock
  holder may unlink or overwrite; the kernel drops the lock on process death, so
  the guarantee survives crashes with no stale-pidfile heuristics.

## 4. Egress plane (gateway convergence) and the mDNS beacon

- A producing desk graph pushes its egress plan into netd over the
  `register_egress` verb, so ONE zenoh session serves the machine's whole network
  plane: the ingress mirrors and every graph's egress. The embedded gateway
  (`GatewayRuntime` under `GatewayEgressPlane`) boots LAZILY on the first
  registration and grows at runtime via dynamic egress-topic registration. A
  pure-ingress desk runs no gateway thread and announces nothing. On a
  LISTEN-configured machine (`CERULION_NETD_LISTEN` set, a robot serving the
  network) the SAME gateway additionally boots AT DAEMON START with an empty
  announce set (`boot_standing_gateway`), so runtime-registered topics (rmw
  publishers, raw routes) are announced, catalogued and demandable with no graph
  run, and the beacon rises at boot; such a daemon never idle-self-exits (§3).
  `CERULION_NETD_NETWORK=off` wins over LISTEN: a local-only daemon boots nothing.
- **Schema-serving merge (the backfill seam)**: the gateway's schema serving is
  SEEDED at boot, not frozen: every `register_egress` MERGES its plan's serving
  (topic→name + hash→name bindings + served docs) into the RUNNING gateway
  through `SchemaServingHandles` (first-wins per key, idempotent), so a gateway
  the standing boot brought up with the built-in corpus only still catalogs and
  serves a later graph's custom types. Without it a later registration's types
  catalogued `schema_name: None` with zero served docs, forever.
- The embedded gateway announces under the MACHINE's identity (the hostname,
  resolved into the shared session's `NetworkConfig.robot_identity` at init).
- The standalone `graph run-gateway` child hosts the plane for an explicit
  `network:` block (the Strict posture, whose verbatim locators and egress
  allow-list the shared permissive session cannot represent), on non-Unix hosts,
  and when netd registration fails. On the permissive default path netd hosts the
  plane and no child is spawned.
- **Peer fold at boot**: before `TransportManager::init`, `fold_cached_peers` adds
  connect endpoints from the same sources the CLI's discovery ladder trusts (the
  cached peer file, env/config/hostname convention), pre-filtered by the bounded
  TCP reachability probe, so a desk reaches a robot it has seen before with no
  environment variable, even where multicast scouting is degraded. Dialling a
  cached locator is load-bearing, not just faster: a scouting-only session's query
  plane can converge while its data plane carries nothing.
- **Beacon**: the `_cerulion._tcp` beacon is raised when the embedded egress
  gateway boots: the instant the machine becomes a producer and the session's
  listener is bound, so the advertised SRV port is a port something really listens
  on. It is held for the PLANE's lifetime (a drive-thread restart must not flap
  it) and leaves the network when netd exits. The advertise gate is the LISTEN
  endpoint (`cerulion_mdns::srv_port_from_listen_endpoints` over the session's own
  config), never robot identity; identity is stamped on every machine, so it
  cannot distinguish a robot from a desk; only a bound listen port can. No listen
  locator ⇒ a REPORTED refusal to advertise, never a default port. An advertise
  failure is a loud warn at the caller and the caller keeps serving; a discovery
  beacon never crashes a run.

## 5. Query plane and discovery convergence

- Catalog and schema queries for every desk observer route through netd's shared
  query plane (the one zenoh session). A consumer opens its OWN transient session
  only when netd is not applicable (explicit `--connect`/`--listen` locators,
  non-Unix) or unreachable, and that degrade is loud.
- **Cold-start grace**: a cold plane spends `COLD_START_DISCOVERY_BUDGET`
  re-harvesting before answering, and reports `NotConverged`, never a confident
  empty. The budget is a wall-clock LOWER bound because a peerless gather returns
  in microseconds.
- **Latches**: `ever_settled` latches only on a NON-empty gather; `grace_spent`
  latches once the budget is exhausted, bounding the cost to ONCE PER DAEMON. A
  grace-spent plane still runs exactly ONE fresh harvest per query, pinned via
  the `harvests_run()` observable, because a wall clock cannot distinguish
  "answered after one harvest" from "short-circuited without harvesting", and the
  short-circuit would permanently blind the desk to a robot appearing later.
  The pure decision is `query::classify_gather`.
- **The marker crosses the seam**: `DiscoveryState` reaches the consumer on BOTH
  query verbs. A `NotConverged` empty forbids an absence claim; a `Settled` empty
  is authoritative real absence; a cold plane that DID gather something still
  reports it (the marker is about convergence, not emptiness).
- **Protocol trust gate**: the `discovery` field's serde default is `Settled` (a
  POSITIVE claim an older daemon never made), so a reported discovery state is
  believed only from a daemon at `DISCOVERY_MIN_DAEMON_VERSION` or above; a reply
  from anything below it is marked `NotConverged`. This is a trust downgrade, not a
  refusal (the minimum accepted daemon version stays lower; a once-per-process
  warn names the restart remedy). The skew is real because the daemon is
  spawn-once: an upgraded CLI can talk to a long-running older daemon held alive
  by another consumer. The floor is guarded by a `<=` compile-time assert against
  `PROTOCOL_VERSION`, never an equality pin, which would mandate distrusting
  every genuine current-version daemon at the next unrelated bump.

## 6. Desk-side convergence-wait contracts

- Constants (in `cerulion_netd::convergence` / `client`):
  `FIRST_CONTACT_CONVERGENCE_CEILING`, `CONVERGENCE_POLL_INTERVAL`,
  `CANCEL_CHECK_SLICE`. The decide step is PREDICTIVE (it asks whether the NEXT
  poll could still land inside the ceiling), so the reachable worst-case wall
  exceeds the ceiling by up to one poll round; state walls accordingly.
- Wait posture is PER SEAM: a seam that makes no absence claim (a local-walker
  fallback, a guarded twin) passes `ResolveWait::no_wait()`; otherwise every
  LOCAL topic stalls the full ceiling before answering.
- The wait loop calls the NON-reconnecting `_once` client verbs. A reconnecting
  verb re-runs connect-or-spawn (up to two spawn-readiness waits), falsifying the
  measured wall by tens of seconds per iteration.
- `plane_unsettled_ms` is an OPTIONAL field on both query responses: absence
  (`None`) means unknown and makes no claim. It caps the wait per DAEMON, not per
  command: with a long-lived consumer (e.g. the desktop viewer) already running
  against the daemon, a later `topic hz` pays a single round trip.
- A cancelled (Ctrl-C) wait is an INTERRUPTION, never an absence claim:
  `WaitOutcome::Cancelled` maps to a no-claim message and exit 0. All convergence
  progress output goes to STDERR through the single `write_convergence_line` seam.
- **Reply reads are timeout-armed** (`cerulion_cli_engine::viz_client`): every viz
  control connection arms `CONN_IO_TIMEOUT` as socket-level read + write timeouts
  (`SO_RCVTIMEO`/`SO_SNDTIMEO`, shared across the connection's cloned fds) before the
  banner read, so every reply read is bounded: a wedged daemon surfaces as a
  `WouldBlock`-kind error, never a hang (the error KIND is what the deadline test arm
  asserts; a wall band would invert under load). Failing to ARM the timeouts is a hard
  `Err`; an un-timeoutable connection is exactly the hang this guards against. A
  FAILED round trip POISONS the connection: the protocol is strictly one reply per
  request in order, so a timed-out read leaves the daemon's reply still in flight and
  the next request would pair with the previous answer; the poisoned connection
  refuses further requests, naming reconnection as the remedy.
- **The stale-daemon memo** (`VizdConn::attach_waiting_for_discovery`, per
  connection): a retry hint reading `NotConverged` with NO plane age is the signature
  of a daemon that cannot report one; against it the per-daemon cap can never fire,
  so every not-found topic would pay the full ceiling. The sighting is only NOTED per
  round trip; the memo is COMMITTED only at the `GiveUpHonestUnknown` exit, where a
  ceiling was actually spent, never on first sighting (an attach that sees one such
  hint and then succeeds must not arm it, or the designed cold-desk happy path,
  converging after a poll or two, pays zero ceilings for the rest of the
  connection). Once armed, the memo's early return for later attaches sits BELOW the
  cancellation check: a Ctrl-C'd wait stays an INTERRUPTION, and a memo return above
  the check would make Ctrl-C invisible for every remaining topic and turn the exit
  into an absence claim. Pinned by
  `crates/cerulion_cli_engine/tests/viz_attach_convergence_test.rs`.

## 7. Catalog-change push

Announce transitions coalesce for a bounded window, then one catalog-change push
lands in a per-connection push slot. The control loop waits on **socket-readable OR
push-slot-filled** (up to the read timeout) and drains the slot before every
blocking read; a loop waiting only on the socket would stall pushes forever on an
idle client (a "sidebar stopped updating" hang that is not a hang). A lost waker is
safe: the next poll fires again and the pre-read drain picks the slot up. Push
latency budget = coalesce window + one read wait.

## 8. WAN plane (iroh)

- `DualMirrorPlane::plane_for` routes a WAN-registered robot to the iroh plane and
  an unregistered robot to zenoh; a direct iroh ensure for an unregistered robot
  is refused loudly WITHOUT a dial.
- The plane's `ensure_mirror` is SYNCHRONOUS: it owns its own tokio runtime and
  blocks on it, matching the exact seam the daemon calls under the registry lock.
  The desk-SHM ingress injector is created synchronously BEFORE the reader task
  spawns (parity with the LAN plane): no phantom mirror can exist without its
  injector, and a taken injector slot fails the demand loudly with no orphaned
  reader.
- The pairing gate holds through the plane: an unpaired desk key is a loud typed
  error with NO connection or reader tracked.
- Reader-death teardown: a mid-stream robot disconnect kills the reader, and the
  supervising task tears down all mirror state: reader, connection, provenance.
- An unreachable-but-valid endpoint fails loudly within the dial bound; it never
  hangs the daemon.

## 9. Why the demand plane is queryable inversion (zenoh wire facts)

On a real asymmetric zenoh peer link (dialer connects, accepter listens, scouting
off), the DIALER's declarations (liveliness tokens, subscribers, interests)
register on its local face only and are never forwarded to the accepter, and the
ACCEPTER's GET queries never route to the dialer. The proven directions are
accepter→dialer declarations (at connect time) and dialer→accepter queries. The
demand plane is therefore shaped as QUERYABLE INVERSION: the producer declares a
wildcard queryable on the demand keyspace (a proven direction) and the demander
sends periodic bounded GETs carrying the demand (the other proven direction), with
TTL-expiry keepalive (`cerulion_core::transport::network` owns the constants and
the ack vocabulary).

Consequences for contributors:

- Loopback sessions back-sync pre-link tokens almost instantly, so in-tree e2e
  tests CANNOT reproduce the asymmetry; any demand-path redesign needs a genuine
  two-machine validation before it can be believed.
- A GET selector containing a MID-KEY single-chunk wildcard (`prefix/*/topic`) can
  compute an EMPTY route on such links even when an intersecting `**` queryable
  arrived; use explicit selectors (harvest identities from the announce space
  first).
- `release_max_level_info` statically strips debug logs from release builds, so
  zenoh wire tracing (`RUST_LOG=zenoh=debug`) requires a DEBUG build; on release
  it is silently a no-op.

## 10. Crate boundaries (load-bearing extraction contracts)

These are dependency-graph rules, not conventions; each exists to break a cyclic
package edge or enforce a capability split. Violating one usually compiles and is
wrong.

- **`cerulion_discovery`**: exists because `cerulion_cli_engine` already depends
  on `cerulion_netd` (for `NetdClient`), so netd importing the discovery ladder
  from the engine would be a cyclic package edge. It holds only what BOTH
  consumers need: the candidate types (`DiscoveredPeer`/`DiscoveryRung`), the
  peer-cache FORMAT plus its READER (`load_peers`/`cache_rung`/TTL), the bounded
  TCP reachability pre-filter (`tcp_port_open`/`probe_reachable_locators`), and
  the pure `plan_connect_set`. The peer-cache WRITER (`save_peers` and friends)
  stays in `cerulion_cli_engine`, a CAPABILITY boundary, not packaging: a cache
  write must be backed by evidence a gather CONFIRMED a robot live, netd runs no
  gather, so netd links a crate with no `save_peers` to call, enforced by the
  dependency graph rather than a comment (it also keeps every write on the one
  atomic-secret-write path). Dependency policy: std + serde + dirs + tracing
  ONLY (no transport crates), so it is safe in `default-members`.
- **`cerulion_wireclient`**: the desk-side `cerulion/wire/1` client substrate
  (control vocabulary, dial-config parsers, the per-topic re-inject reader). It is
  a crate of its own so that `cerulion_netd` reuses the vocabulary and parsers
  with no netd→connectd cyclic edge: both daemons depend on the substrate rather
  than on each other, which is also what leaves `cerulion_connectd` free to be a
  `NetdClient` consumer. netd's iroh WAN plane drives its OWN supervised
  re-inject loop (`iroh_plane::run_reinject_reader`), NOT
  `reader::run_topic_reader` (the connect-worker session loop); the element
  genuinely shared across planes is `cerulion_core`'s
  `IngressInjector::reinject_raw` primitive. Links `cerulion_link` (→ iroh)
  unconditionally, so it is EXCLUDED from `default-members`.
- **`cerulion_mdns`**: the leaf crate owning the ADVERTISE half of the
  `_cerulion._tcp` beacon: the service type, the TXT vocabulary,
  `srv_port_from_listen_endpoints`, `MdnsAdvertiseGuard`. `CERULION_SERVICE_TYPE`
  is defined once here and imported by the browser, so the one string that must
  match on both sides cannot drift. The BROWSE half stays in
  `cerulion_cli_engine::mdns_discovery` (netd never browses). It is a crate
  because BOTH netd and the fallback `graph run-gateway` child advertise, and the
  engine already depends on netd; importing the beacon from the engine would be
  cyclic, and `cerulion_discovery` is pinned mdns-sd-free.
- **`cerulion_dds`**: the only main-workspace crate pulling the DDS stack. The
  `ros2 attach` COMMAND logic in `cerulion_cli_engine::ros_cmd` is pure over the
  plain types re-exported here (the engine depends `default-features = false`),
  so the pure half is oracle-testable without a DDS peer and this crate is the
  single flip point if placement ever moves.
- **netd's feature seam**: `wan` default-on, netd out of `default-members`, lean
  consumers on `default-features = false` (see §1). Any change to one leg of that
  triangle must re-check the other two.

## 11. DDS wire rung (`cerulion_dds`)

The user-facing schema-resolution ladder is documented in
`docs/schema_resolution.md`; this section is the contributor contract for the wire
rung itself.

- **Ladder position**: the chain predicate runs first (workspace `.msg` store +
  built-in corpus); acquirer rungs run in priority order over the
  still-unresolvable set: the wire-native `~/get_type_description` service rung
  FIRST, the local ament harvest (`$AMENT_PREFIX_PATH`) second.
- **One discovery window**: `SchemaAcquirer::acquire` carries the engine's
  `DiscoveryResult`; there is deliberately no second window. Endpoints and RIHS01
  `typehash=` hashes ride a loss-proof DiscoveryDB SNAPSHOT: rustdds 0.14.2
  supplies endpoint USER_DATA parsing and the snapshot accessors, and the
  published `cerulion-rustdds` fork is retained only for the
  `participant_lease_duration` builder knob. The USER_DATA blob arrives
  CDR-encapsulated (`[u32 len]key=value;`) and is tolerantly stripped at the one
  parse seam.
- **Call targeting**: the node table comes from the `ros_discovery_info`
  participant-entities topic, keyed by (participant prefix, namespace, name), with
  our own node filtered out. The vendor→`ServiceMapping` derives from the TARGET
  node's participant. Call targets are the union of nodes over ALL of the type's
  endpoints (writers first, reader fallback), under per-call budgets, a call-phase
  wall cap, a per-node wait-failure cache, and a sliced poll wait (working around
  ros2-client's `wait_for_service` lost-event race).
- **Explicit skips**: three distinct reasons (type-not-discovered, no-hash,
  no-owning-node), never a merged generic failure.
- **Materialization**: each `encoding == "msg"` type source materializes into
  `schemas/<pkg>/msg/<Type>.msg` with the wire-stripped trailing newline restored
  (rcl strips exactly one final newline in transport; extra blank lines normalize,
  unrecoverably). The merge duplicate-check ignores trailing-newline-only deltas
  so wire-vs-ament copies of one member never false-refuse. Materialized files
  ride the SAME consent gate as the generated graph: dry-run writes nothing;
  non-TTY requires `--yes`.
- **Distro / GID matrix**: `jazzy` (default) selects the 16-byte GID world: Iron
  and newer, which is exactly the rung's functional domain (RIHS01 hashes and the
  type-description service exist only there). `humble` is the explicit pre-Iron
  24-byte opt-in. A default build cannot decode pre-Iron participant-entities
  messages; the resulting empty node table is expected, and every other attach
  rung is GID-independent.
- **Executor shape**: discovery and service calls run through the node spinner +
  async status stream on one `smol::LocalExecutor` (`smol::block_on`), with DDS
  objects created on the drain thread.

## 12. Test map

| Test file | What it pins (1 line) | Serial? | Prereq fixtures |
|---|---|---|---|
| `crates/cerulion_netd/tests/daemon_e2e_test.rs` | Refcount→mirror wiring over the real UDS with a spy plane: shared mirror, idempotent re-demand, conflict refusal, ensure-failure rollback, close-releases-all, flock singleton, discovery marker on both query verbs | no | none |
| `crates/cerulion_netd/tests/daemon_idle_exit_e2e_test.rs` | Idle self-exit over the REAL binary: socket unlinked first, no half-dead zombie, client retry binds the respawn | no | daemon binary (built by cargo test) |
| `crates/cerulion_netd/tests/cli_version_test.rs` | `--version`/`-V` over the REAL binary: the exact `cerulion-netd <version>` line on stdout, empty stderr, exit 0, with no daemon boot | no | daemon binary (built by cargo test) |
| `crates/cerulion_netd/tests/client_e2e_test.rs` | `NetdClient` demand/release pairing, spawn-readiness wait (first-use line at `INFO`, one stall `WARN` past two seconds, none on a prompt boot), died-vs-booting discriminator, version trust downgrade at the call sites | no (env arms use a file-local mutex) | daemon binary via `CARGO_BIN_EXE` |
| `crates/cerulion_netd/tests/client_egress_e2e_test.rs` | `register_egress` composed through client + daemon + production egress plane really egresses (no inert shipping) | no | none |
| `crates/cerulion_netd/tests/mirror_plane_iox2_test.rs` | Production `GatewayMirrorPlane` over real iceoryx2 + zenoh: ensure/release, provenance registered and removed, single-writer slot freed, full demand→release→re-demand cycle | no | none |
| `crates/cerulion_netd/tests/mirror_restart_e2e_test.rs` | A held demand self-heals across a robot gateway restart, with no client re-demand needed | no | none |
| `crates/cerulion_netd/tests/egress_plane_iox2_test.rs` | Embedded gateway boots on the first egress plan and grows at runtime; frames flow SHM→tap→zenoh→remote SHM byte-identical to a hand oracle | no | none |
| `crates/cerulion_netd/tests/standing_gateway_e2e_test.rs` | LISTEN-configured start boot: `boot_standing_gateway` boots the gateway with no egress registration (beacon decision included) + a no-LISTEN plane stays unbooted; `idle_self_exit: false` daemon never self-exits (default pair self-exits); real binary: local-only wins over LISTEN; main.rs wiring walk; a custom registration AFTER the standing boot backfills the serving (catalog name + doc over the remote query surface) with the boot-registration control | no | daemon binary via `CARGO_BIN_EXE` (one arm) |
| `crates/cerulion_netd/tests/query_plane_iox2_test.rs` | Cold-start grace spent before any absence claim; `grace_spent` bounds the cost to once per daemon; exactly one harvest per later query (`harvests_run`); loud no-network refusal | no | none |
| `crates/cerulion_netd/tests/convergence_wait_e2e_test.rs` | The desk wait loop against a scripted fake daemon: served-query COUNT is the oracle, never wall time | no | none |
| `crates/cerulion_netd/tests/catalog_events_e2e_test.rs` | Catalog-change coalescing, push slots, and no-wedge contracts against a scripted announce stream | no | none |
| `crates/cerulion_netd/tests/catalog_events_live_test.rs` | Real zenoh liveliness tokens reach the announce watch (Alive→Lost), so the announce key-space wiring is not inert | no | none |
| `crates/cerulion_netd/tests/discovery_fold_test.rs` | The boot-time peer-cache fold: real cache document, real TTL reader, real TCP pre-filter, the exact `NetworkConfig` mutation main performs | no | none |
| `crates/cerulion_netd/tests/mdns_suppression_confinement_test.rs` | Source walk: suppressed-advertiser test constructors are referenced by no shipped code path | no | none |
| `crates/cerulion_netd/tests/wan_plane_iroh_test.rs` | The iroh WAN plane over loopback iroh: byte-identical re-inject, release frees the slot, pairing gate, reader-death teardown, dual-plane routing (compiles by default; absent under `--no-default-features`) | no | none |
| `cerulion_dds` inline unit tests (`wire.rs`, `discovery.rs`, `wire_acquirer.rs`) | The pure wire decision engine, discovery mapping, feature/GID pins, skew-warn emit sites, all DDS-free oracles | no | none |
| `crates/cerulion_dds/tests/live_discovery_box_test.rs` | Live SPDP/SEDP + spinner against a real CycloneDDS peer; precondition-panics with the bring-up recipe | `#[ignore]`, hardware only | `DDS_IFACE` env + a DDS talker |
| `cerulion_discovery` inline unit tests | Ladder planning (`plan_connect_set`), locator validation/dedupe, cache-reader TTL, LOUD corrupt-cache warns (level-matched) | no | none |
| `cerulion_wireclient` inline unit tests | Dial-config parsers, wire protocol round-trip, epoch-push classification with pinned remediation text | no | none |

Running: `cargo test -p cerulion_netd`, `-p cerulion_dds`, `-p cerulion_discovery`,
`-p cerulion_wireclient` are all parallel-safe as-is; none of these crates uses the
shared-memory singleton pattern that forces `--test-threads=1` elsewhere in the
workspace. `cargo check -p cerulion_netd --no-default-features` and
`cargo check -p cerulion_dds --no-default-features` are the lean-build gates the
consumers rely on; keep both green.
