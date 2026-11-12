# Remote access & pairing: contributor dossier

Scope: the remote-plane family: `cerulion_remoted` (the robot daemon), `cerud`
(the robot ops-verb service), `cerulion_pairing` (identity + trust), `cerulion_link`
(iroh dial-by-key QUIC), `cerulion_wireclient` (desk client substrate),
`cerulion_connectd` (the worker behind `cerulion connect`, `cerulion rcp` and `--robot`
on `topic pub`, `topic snapshot` and `service call`; its hidden `pair` subcommand
has no `cerulion` caller),
`cerulion_accountd` (the cloud issuer/CA + accounts), `cerulion-wire` (the
permissive frame decoder), and `cerulion_netd`'s iroh WAN plane. Read this
alongside `docs/internals/network-daemons.md` (the LAN zenoh half; its §8 and §10
overlap this family and are cross-referenced, not duplicated), `docs/remote_plane.md`
(safety posture) and `docs/revocation.md` (the revocation lifecycle end-to-end).
Everything below is present-tense contract verified against source; each claim
names its enforcing test, and a claim with none says so explicitly.

## 1. Topology: one key, one endpoint, one robot process

The remote plane is the INTERNET path, distinct from the LAN zenoh gateway. A
robot exposes ONE iroh endpoint whose identity IS its ed25519 device key: the
ONE 32-byte SECRET seeds both the iroh endpoint and the pairing
`DeviceIdentity` (`client.rs::from_seed` consumes it), and its PUBLIC half
doubles as the iroh `EndpointId` and the pairing transport public key; the
seed is never the public half. iroh multiplexes every ALPN on that endpoint
(`cerulion_link::alpn`: `cerulion/wire/2` = raw topic frames, `cerulion/ops/1` =
ops verbs, `cerulion/rcp/1` = MCP sessions with the robot's `cerulion-rcpd`; the
trailing number is each plane's protocol-version boundary). Exactly
one process owns it: `cerulion-remoted`, dispatching by negotiated ALPN at accept
(`daemon::handle_accepted`; pinned over real loopback endpoints by
`crates/cerulion_remoted/tests/loopback_test.rs`), or a host process that embeds
the same entry instead of running the daemon.

- **One entry, two hosts** (`host.rs`). `RemotePlane::bind` loads the trust state
  through a `KeyProvider` and binds the endpoint; `BoundPlane::serve` runs the
  accept loop. Its inputs are injected: the state root (files under
  `<root>/remoted/`), the keys (device key and trust-store MAC key), the robot
  identity, the schema source of the `catalog`/`schema` metadata reads and
  optionally the transport manager. `RemotePlane::new` reads the local network
  daemon's control socket (each read deadline-bounded); `RemotePlane::in_process`
  calls a `SchemaSource` closure the host supplies, for a host that serves its
  own topics and runs no daemon (the Android app library).
  The daemon's `run` checks the kill switch, then serves
  `RemotedConfig::daemon_plane()` with the `KeyFiles` provider; `serve_endpoint`
  (`#[doc(hidden)]`, tests only) shares the same serving core, which never
  reads a key file or the default socket path itself: a serving change lands
  once for both hosts. A library host
  (`RemotePlane::new`) refuses the daemon-only verbs by name with
  `unsupported_host`: `restart` (it restarts a systemd unit), `engage-estop`
  (it engages the daemon's control lease and incident capture; a host that served
  it would report an e-stop that stops nothing), and the act verbs
  `topic-publish`, `service-call`, `topic-snapshot` and `topic-echo-once` (they
  run on the process's transport singleton and the daemon's schema directory, and
  the two that send are gated by the daemon's e-stop, which a library host does
  not serve). The authorizer still runs first,
  so an unpaired caller is denied as on the daemon. A library host announces
  `cerulion-remote-plane` in the ops handshake, not `cerulion-remoted`, does not
  read the `--network` / `CERULION_NETWORK` kill switch (the host decides
  whether to serve), and still runs the epoch poll on its state root. Pinned by
  `remote_plane_library_test.rs` and the verb-name pin in `ops.rs`.

- **Stopping** (`main.rs`, `shutdown.rs`). Before it binds, the binary installs
  its SIGINT (Ctrl-C) and SIGTERM handlers on a thread with a tokio runtime of
  its own, so they still answer while the serving runtime tears down. On the
  first stop signal that thread arms the deadline, then hands serving the
  signal's name; serving logs one info line naming it (`signal`), and the
  accept loop stops. Every `tracing` line is queued for the log writer
  (below), so no such line holds the stop thread or serving: the stop thread
  queues its deadline or second-signal line and waits at most
  `EXIT_LOG_GRACE` (100 ms) for the queue before `_exit(4)`, and `main` waits
  as long for it once the serving runtime is dropped. A stderr that stays full
  delays neither the graceful stop nor the hard exit past the deadline and
  that 100 ms. The accept loop also stops when
  the endpoint closes on its own. Either way `serve_planes` drops its shutdown
  future before it logs anything, which arms the deadline when no signal did,
  so a blocked log write cannot hold it back, then runs the ops plane's
  `ShutdownStage` (`OpsServing::shutdown_stage`) before it returns, so every
  serving entry runs it once. Each registered `ShutdownParticipant` is driven
  to settlement on a task of its own, one after another in registration order,
  with no timeout, while the receipt sink is alive; after a stop signal the
  endpoint and its connections are alive too and keep serving while the stage
  runs, and when the endpoint closed on its own they are already gone. The
  participants are the ones the host registered on the ops serving's stage. A
  participant that returns an error, panics or is cancelled never stops a later
  one; `serve_planes` returns `RemotedError::Shutdown` naming each. `serve_core`
  then ends the rcpd e-stop delivery in flight (at most 5 s) and stops the
  epoch poller, failed stage or not, and logs that it did; the entry closes the
  endpoint (about 3 s with default parameters, longer on a high round-trip
  path), and the runtime teardown waits for the work left on the blocking pool,
  such as a running ops verb. The exit status is 0, also when the stop started
  because the accept loop stopped on its own, or 3 with one error line per
  participant (`participant`, `kind`) that failed, panicked or was cancelled;
  `cerulion-remoted` registers none of its own.
  `CERULION_REMOTED_HARD_EXIT_MS` (default 30000 ms) bounds all of it from the
  first stop signal, or from the moment serving dropped its shutdown future
  without one: past it an error line is logged and the process ends with
  `_exit(4)`, the explicit failure exit. A second stop signal during the stop
  also ends it with status 4, after the same wait of at most 100 ms for the
  log queue; after the accept loop stopped on its own, the first stop signal
  already counts as that second one. The default was chosen as a sum of
  nominal shares: 18 s reserved for the participants the host registers, 5 s for
  the wait for the task of an rcpd delivery that the stop has already ended,
  which gives the delivery no more time, about 3 s for the endpoint close and
  4 s of scheduling. The shares bound no real work: a
  participant may keep awaiting work past its share, and that work stays
  pending until the process exits; a participant that needs more than its share
  needs the deadline raised with it. The deadline is a separate failure policy:
  it ends the process and proves nothing finished. The daemon's own work ends
  with the process, except a thread in an uninterruptible kernel wait, such as
  an fsync on a stalled disk, which holds the exit until that wait returns; a
  child process keeps running. An ops request still running past the deadline
  (its own limit is 120 s) is cut by the hard exit. SIGKILL runs none of this
  and logs nothing. Under `cerulion-netd`'s robot supervisor, a netd stop sends
  SIGTERM and waits `CERULION_REMOTED_HARD_EXIT_MS` plus 500 ms before SIGKILL,
  so the graceful stop runs within its own deadline. Pinned by `daemon_stop_signal_test.rs`,
  `shutdown_stage_test.rs` and the stage tests in `shutdown.rs`.

- **Logging** (`log_writer.rs`, `main.rs`). The binary's fmt subscriber writes
  through a `LogWriter`: fmt formats each event on the thread that logs it,
  with that event's timestamp; the writer (`EventLine`) gathers the bytes and,
  when fmt drops it, hands them to a queue as one owned line, under the
  queue's lock for one push. One thread, `remoted-log`, pops each line in
  order and does the blocking `write_all` to stderr with the lock released;
  that lock is the only one the `tracing` path shares with it. The iceoryx2
  console logger (remoted installs no tracing bridge for it) and the default
  panic hook still write to stderr directly. A line that `admits` refuses
  (`QUEUE_LINES` 4096 lines, `QUEUE_BYTES` 1 MiB) is dropped and counted, as
  is a line whose write fails; `log_writer::lines_dropped` reads the count
  since the start, and the wire plane's `status` reply sends it as
  `log_lines_dropped` (`None`, not serialized, without an installed writer:
  a library host keeps its own subscriber). After each write, and when a drop
  wakes it while it waits for a line, the writer thread feeds the drops since
  its last report to one `FailureRegimeLatch`, one failure per line
  (`drop_report`): a batch that opens the regime or crosses a decade of the
  total logs at warn (`dropped`, `total_dropped`), any other at debug; with
  the queue empty and no drop since that report, `on_success` ends an open
  regime with one info line (the regime's `dropped`, `total_dropped`), silent
  for a regime of one line. `writing` stays set until the report is written,
  and the queue's `reported` count, which only the writer thread stores,
  shows a drop it has not reported yet, so a drain waits for both.
  The writer thread formats those lines through the same subscriber, and a
  thread-local makes its `EventLine` write them straight to stderr, never
  into the queue; a report line whose write fails is not counted.
  `LogWriter::drain` is each exit's wait: until the queue is empty, no write
  or report is in flight and no drop is left unreported, for at most four
  fifths of the grace; then
  all but the newest `KEPT_AT_EXIT` (8) queued lines are discarded, one error
  line (`discarded`) is queued after them when there are any, and it waits
  for the rest of the grace or until the queue is written. A line the stop
  thread queues behind a full queue is dropped like any other. A writer
  thread that cannot start, or a runtime that cannot be built, exits the
  binary 1 after one error line. In a build with the test seam only,
  `CERULION_REMOTED_LOG_QUEUE_LINES_FOR_TEST` sets a line bound of at most
  `QUEUE_LINES` (`test_queue_lines`: a positive whole number that fits a
  `usize`, whitespace around it trimmed, a larger one than `QUEUE_LINES`
  setting `QUEUE_LINES`; any other value, zero among them, sets none), as
  the crate's other `_FOR_TEST` variables set their seams; it takes effect
  once the daemon has logged that it listens (`shrink_queue_for_test`,
  called right after that line), so the lines before it are queued under the
  full bound. Pinned by the
  unit tests in `log_writer.rs`, `estop_full_stderr_test.rs` and
  `daemon_stop_signal_test.rs`.

- **Always-on from boot.** Ops + pairing must be reachable when NO graph is
  running, so the endpoint host is a robot-level service, not the per-run
  gateway. *This is a deployment requirement, not a runtime check: no test
  enforces it.*
- **Deny-by-default.** Every accept is classified by the `PairingAuthorizer`
  before any plane serves (see §5). The `--network off` / `CERULION_NETWORK=off`
  kill-switch makes `run` exit cleanly WITHOUT binding an endpoint or even
  reading the key file (`daemon_wiring_test.rs`).
- **Desk side, three consumers of the same robot surface**: the
  `cerulion connect` verb (spawns the `cerulion-connectd` sibling binary),
  `cerulion-connectd pair` (the code ceremony; the `cerulion pair` verb that
  spawned it is a hidden stub that refuses, §7), and `cerulion_netd`'s
  embedded iroh WAN plane (the `wan` feature). All three dial the same two
  ALPNs and share one client substrate (§7 to §8).

## 2. Crate map, licenses, and the build boundary

| Crate | Side | License | In `default-members`? | Role |
|---|---|---|---|---|
| `cerulion_pairing` | both | MIT OR Apache-2.0 | yes | formats + crypto + state machines, plus the machine identity files (`identity`: config directory, machine key, high-water file); no network I/O, no iroh dep (embeds in firmware and in the closed Studio client) |
| `cerulion_link` | both | MIT OR Apache-2.0 | **no** (pulls iroh, ~390 crates) | thin iroh wrapper: endpoints, dial, framing, relay seam, ops-stream adapter; never depends on `cerulion_core` |
| `cerulion-wire` | desk | MIT OR Apache-2.0 | yes | standalone wire-format decoder so CLOSED apps decode frames without linking the AGPL `cerulion_core` |
| `cerulion_ops_proto` | both | MIT OR Apache-2.0 | yes | the `cerulion/ops/1` wire types: BE-`u32`-length JSON framing, `Hello`/`HandshakeReply` version handshake, `Request`/`Response` envelopes, the `Inventory` result, and the protocol-only `OpsClient`; depends on serde, serde_json and thiserror only |
| `cerud` | robot | AGPL-3.0-only | yes | deliberately-dumb ops verbs + authz seam + hash-chained receipts + the lease/deadman; transport-agnostic |
| `cerulion_remoted` | robot | AGPL-3.0-only | **no** (links link + core) | THE robot daemon: endpoint, accept gate, wire plane, pairing verbs, live trust; links `cerulion_wireclient` for `sanitize_peer_text` only |
| `cerulion_wireclient` | desk | AGPL-3.0-only | **no** (links link + core) | shared `cerulion/wire/2` client substrate (protocol mirror, dial-config parsers, reader loop, epoch push) |
| `cerulion_connectd` | desk | AGPL-3.0-only | **no** | the worker binary behind `cerulion connect`, `cerulion rcp` and `topic pub`/`topic snapshot`/`service call --robot` (session driver; its hidden `pair` subcommand is the CPace initiator) |
| `cerulion_accountd` | cloud | AGPL-3.0-only | **no** (web stack) | issuer/CA + accounts + OAuth/device-code login + revocation endpoints + the Team page |

Load-bearing boundary contracts:

- **The AGPL/closed boundary for the connect worker is a PROCESS boundary**: the closed Studio spawns
  the AGPL `cerulion-connectd` binary exactly as it spawns the gateway/vizd
  (`crates/cerulion_connectd/src/lib.rs`; the CLI side is pure argv resolution in
  `cerulion_cli_engine::connect_cmd`; the `cerulion` CLI stays iroh-free).
  The permissive crates never depend on an AGPL crate:
  `the_permissive_crates_reach_no_agpl_crate`
  (`crates/cerulion_hygiene/tests/dependency_rules_test.rs`) reads every
  member's `license` from `cargo metadata` and fails if a `MIT OR Apache-2.0`
  member reaches any member not licensed exactly `MIT OR Apache-2.0` over
  normal or build edges.
- **The ops protocol is a CRATE boundary**: the `cerulion/ops/1` wire types and
  the protocol-only client live in `cerulion_ops_proto` (MIT OR Apache-2.0), so a
  closed client links them without linking `cerud`. `cerud` depends on it and
  re-exports the types and constants at their old paths (`cerud::protocol::{Hello,
  HandshakeReply, Request, Response, ..}`, `cerud::verbs::inventory::Inventory`);
  `cerud::protocol`'s frame and codec functions and `cerud::client::OpsClient`
  call the `cerulion_ops_proto` ones and convert `OpsProtoError` into the
  `CerudError` variant of the same name. `Response::from_error` takes any
  `ErrorKind + Display` error, and `cerud` implements `ErrorKind` with
  `CerudError::kind`, so the `kind` strings stay in `cerud`. The bytes are
  pinned to golden frames in
  `crates/cerulion_ops_proto/tests/fixtures/ops_wire_golden.txt` by
  `crates/cerulion_ops_proto/tests/wire_golden_test.rs`.
- **`cerulion-wire` deliberately duplicates the wire structs** (40-byte header,
  8-byte offset entry) so the permissive decode seam stands alone. Byte
  compatibility is pinned by `crates/cerulion_core/tests/wire_lockstep_test.rs` (the
  real writer encodes, this crate re-parses) plus in-crate `const` size asserts.
- **`cerulion_wireclient` exists to break a package cycle**: netd needs the wire
  vocabulary + dial parsers, and a `netd → connectd` edge would be cyclic once
  connectd consumes `NetdClient`. The boundary is documented at
  `crates/cerulion_wireclient/src/lib.rs` and `network-daemons.md` §10; connectd
  re-exports `config`/`error`/`protocol` verbatim, so its public surface is
  unaffected. The sibling `cerulion_discovery` boundary (and its reader-only
  peers-cache capability split) is `network-daemons.md` §10.
- **`cerulion_connectd` MIRRORS the robot's control protocol** instead of
  importing `cerulion_remoted` (which would drag `cerud` into the desk build).
  The no-divergence guarantee is a dev-dep test:
  `crates/cerulion_connectd/tests/protocol_parity_test.rs` round-trips every
  request/response/preamble/status/decision between the two type sets.
- A plain `cargo build` (default members) is iroh-free; the excluded crates
  build via `-p` or `--workspace`. `cargo clippy --workspace --all-targets`
  compiles all of them, and every package's tests are named in a CI step,
  enforced by `crates/cerulion_cli_engine/tests/ci_test_coverage_test.rs`
  (`every_package_with_tests_is_named_in_a_ci_test_step`).

## 3. Identity & trust model (`cerulion_pairing`)

The chain: an air-gapped **M-of-N root set** (robots trust a set, not one key) →
a rotatable online **intermediate** → short-lived **device certs** ("device key K
belongs to account X until D") and issuer-signed **grants** ("account B may
access robot R"). Grants travel WITH the client; the robot never polls the
cloud. Ownership is attested at the ACCOUNT level so account-key rotation never
strands robots. Chain verification positive path + the full negative matrix
(threshold, peer-key binding, expiry, delegation depth, epoch revocation,
rollback) is `crates/cerulion_pairing/tests/chain_verify_test.rs`.

- **Canonical signing bytes are hand-rolled, not serde** (`format/canonical.rs`):
  one deterministic layout per signable type, length-prefixed domain tags
  (a device-cert signature can never replay as a grant), length-prefixed
  variable fields. Pinned by INDEPENDENT hand-serializer byte vectors in
  `format_vectors_test.rs`; a format change must update encoder AND oracle.
- **`VerifiedPairing` is an unforgeable capability token**: private fields,
  minted only inside `TrustStore::verify_new_pairing` (a `compile_fail` doctest
  in `verify/mod.rs` pins that external code cannot construct one).
- **Delegation depth is capped at the format level**: `MAX_DELEGATION_DEPTH = 1`
  (`lib.rs`), encoded in every grant and enforced at verify time
  (`chain_verify_test.rs`).
- **Revocation is not expiry.** The robot-local durable access list is truth;
  an established pairing lapses offline only at a finite expiry its row carries
  (an owner-signed grant's `not_after`; for a cloud device, no later than its
  certificate's `not_after`); short-lived certs otherwise gate NEW pairings
  only. Revocation propagates via the monotonic signed epoch (§9).
- **The trust store is tamper-evident and anti-rollback** (`verify/store.rs`):
  HMAC-SHA256 over a versioned envelope, constant-time verify on load, atomic
  temp+rename writes; the MAC key is caller-supplied (firmware secure storage;
  the crate holds no key-management policy). `high_water_ns` is the max
  validated time ever observed; a verification whose `now_ns` is behind it is
  refused, so a clock rollback cannot resurrect an expired cert, and the floor
  survives factory reset. Pinned by `store_test.rs`. Format v4 keys access rows
  by `(account, device)` and carries the allow-from-cloud switch; a v3 store is
  MIGRATED on load (MAC verified first, every row kept as an account-wide row in
  order, the switch on, one warning) and saved as v4 on the next write. Any other
  version is REFUSED (the bump policy in `store.rs`).
- **Access rows are per device.** A row belongs to `(account, device)`; a new
  device of an account adds its own row and never rewrites another device's
  row, the owner row included. `device: None` is an account-wide row (the owner
  row a claim writes, and every migrated row); a device with no row of its own
  falls back to it, and a revoked or expired device row never does
  (`TrustStore::is_allowed`). An owner revoke covers every row of the account.
- **The cloud principal.** `PrincipalKind::Cloud` (signed discriminant 3, postcard
  variant index 2) marks a device certificate the hosted service holds for an
  account. A decoder built before it refuses the certificate. The robot's
  allow-from-cloud switch (in the trust store, ON by default, set by the owner
  from a non-cloud device with the `cloud-access` ops verb) gates every cloud row
  through one predicate, `TrustStore::cloud_allowed`, surfaced as
  `KeyAccess::CloudRefused` and refused at accept on every ALPN and for every
  verb, the bootstrap and e-stop verbs included.
- **CPace fallback ceremony** (`pake.rs`): the CFRG balanced PAKE, run INSIDE an
  already-authenticated channel with both transport identities transcript-bound;
  bounded attempts (config range 1..=5, default 3), code burn, TTL (default
  120 s), single session; the robot's counter is authoritative; a wrong code is
  detected structurally at key confirmation. Pinned by `pake_test.rs` (protocol
  invariants, never a self-compare). The cross-robot witness replay is rejected
  because the store records the robot's OWN transport key and
  `establish_code_pairing` binds the witness to it (`store.rs`, `store_test.rs`).
- **Proof-of-possession** (`pop.rs`): a server-issued, single-use, account-bound
  challenge signed by the transport key closes the registration-squatting hole
  (binding a public key you don't hold the private half of).

## 4. Owner grants and offline verification

A robot's owner signs a `SignedAccessGrant` for a subject account; the desk
carries it (owner cert + intermediate + its OWN subject cert) and presents it at
dial time; the robot verifies the whole thing OFFLINE against its own claimed
owner: no cloud contact, no synced ACL (`verify/mod.rs`
`OwnerGrantPresentation`, `TrustStore::{verify_owner_grant,
establish_by_owner_grant}`). Pinned by
`crates/cerulion_pairing/tests/owner_grant_verify_test.rs` (real chains, one-flip
negative matrix) and end-to-end by
`crates/cerulion_remoted/tests/owner_grant_present_test.rs` (the `present-grant` ops
verb over the LIVE trust). The serde-transportable wire forms
(`OwnerGrantPresentationWire`, `EpochSyncWire`) are homed in `cerulion_pairing`
(the common dependency of both sides), so desk and robot can never drift in field
order, and they ride postcard blobs (ed25519 signatures are byte-oriented serde
and do not survive serde_json). `EpochSyncWire::from_postcard` is strict on both
axes postcard is lenient about: a wrong leading version is refused BY NAME, and
trailing bytes are a loud error (`format_vectors_test.rs`, including
`an_unversioned_epoch_envelope_is_refused_loudly`).

- **Team trust formats** (`cerulion_pairing::team`, pure, nothing calls them
  yet): the X.509 profile for machine leaves (`<role>.g<N>.<team>.cerulion.invalid`,
  ed25519, extKeyUsage serverAuth plus clientAuth always written, robot leaves
  with notAfter `99991231235959Z`) and team issuing CAs (self-signed, CA with
  path length 0), written by a hand-rolled DER encoder and read back by the strict
  `check_leaf` / `check_team_ca`; the intermediate-signed `TeamTrustBundle`
  (`cerulion-pairing:team-trust:v1`: the current issuing CA, the next
  generation's CA published one rotation ahead, retired CAs and robot leaves),
  `TeamStatusList` and `StatusFreshness`
  (`cerulion-pairing:team-status:v1`, told apart by a kind byte); and the pure
  anti-rollback decisions in `team::rollback`. Pinned by `team_formats_test.rs`
  against x509-parser, rustls-webpki, rcgen-built refusals and hand-built
  signing bytes.

## 5. The robot accept gate (`cerulion_remoted`)

`PairingAuthorizer` (`authorizer.rs`) is pure over the live `SharedTrust`: map
the TLS-authenticated `remote_id` to an account via the `device_key → account`
side-map, then apply the verb table. The decision matrix is oracle-tested cell
by cell in `authorizer_matrix_test.rs`; the accept-time routing over real
endpoints is `loopback_test.rs` (`wire_unpaired_key_is_refused`,
`ops_unpaired_key_reaches_bootstrap_only`,
`unclaimed_robot_wire_refused_ops_bootstrap_only`).

The decision also sets idle fencing on the owned daemon. Only an `OpsAdmit` or
`RcpAdmit` session holds a shared `serving.hold` guard, from classification
until it ends (`daemon::fences_idle_teardown`); every ops verb outside the
bootstrap surface runs under its own guard (`ops::FencedVerb`), so a key that
claims or pairs on a bootstrap-only session runs its later calls fenced. While
netd's idle stop is committed, an arriving `OpsAdmit` or `RcpAdmit` connection
is closed at admission with `remote server is stopping`, a fenced call answers
`unavailable: the remote server is stopping, so <verb> is refused`, and a
bootstrap-only session is still served. The e-stop floor is the exception:
`engage-estop` takes a guard when it can and runs either way, so a committed
idle stop never refuses an e-stop, and the engage latches and starts its
delivery to rcpd before the server stops.
`OpsBootstrapOnly`, `WireAdmit`, `Refuse` and `UnknownAlpn` connections hold
none, so a peer that never pairs cannot keep the server running
(`network-daemons.md`, authenticated serving); an armed pairing code keeps it
serving through a bounded source of its own (section 7). An `RcpAdmit` connection
re-checks the session floor every `RcpLimits::revoke_check`, with or without a
stream open, so a revoked key's connection is closed with `rcp::CLOSE_REVOKED`
and its guard released. Pinned by `serving_fence_admission_test.rs` and, for
the revoked connection with no stream, `rcp_forward_e2e_test.rs`.

A `DemandSubject::Lan` carries no paired identity, so `PairingAuthorizer`
admits it. The LAN posture lives in front of every serving plane instead:
`NetworkManager` retains `LanAccess` separately from the caller-installed inner
authorizer and applies the posture in front of it. It refuses every LAN subject
under `LanAccess::Paired` (`CERULION_LAN_ACCESS`, the Android default, and the
Android app's `lan_access` key) and defers other subjects to the inner
authorizer. `set_demand_authorizer` replaces only the inner authorizer;
`set_lan_access` changes the posture. The liveliness watch, the demand queryable
and the reconcile belt all consult the layered gate, so no demand path enables
egress the gate refused. `set_lan_access(Paired)` also closes the bridge manager
with `TopicBridgeManager::set_lan_closed`, atomically revoking enabled egress
and preventing every demand path from re-enabling it, then waits out any
in-flight gateway forwarding pass so no frame leaves after the close returns.
The posture gates the write direction too. Under `Paired` an ingress
registration declares no zenoh data subscriber and no demand token, and the
receive callback refuses and counts (`IngressStats::lan_refused_drops`) any
frame that still reaches it. `set_lan_access(Paired)` suspends every live
ingress entry (undeclares both, keeps the injector and its counters) and waits
out every receive callback that already passed its posture check before it
returns, so no frame from the zenoh LAN plane is re-injected after the close;
reopening redeclares both. A callback that checks the posture after the close,
even one still in flight when the close returns, refuses and counts its frame.
The demand GET loop does not consult the posture: it keeps sending GETs while
paired, and each GET's selector names one ingress topic it demands. The
remote plane's wire protocol does not carry writes from a desk:
`cerulion/wire/2` moves frames from robot to desk only, and the `teleop` verb
has no handler yet (see `docs/remote_plane.md`). Its re-inject seam is the
desk-side path that copies robot frames into the desk's own shared memory; it
does not consult the LAN posture and it is not a write path to the robot. A desk
writes the robot's topics and services over the ops plane instead (§5.1):
`topic-publish` publishes a message on a robot topic and `service-call` calls a
robot service, in the robot's own shared memory, for an account with
`CAP_TELEOP` and role ≥ OPERATOR. Neither consults the LAN posture; both are
refused while the robot's e-stop latch is engaged. Every act verb refuses a
network mirror and topics under `/__cerulion`; a live mirror names its source
robot, other reserved names are usage errors, and producer exclusions preserve
the topic's remote-access policy context. Under `Paired` a desk writes a message it supplies
into the robot's shared memory only through `topic-publish` and `service-call`.
Oracle tests:
`demand_authorizer.rs` unit tests,
`demand_reconcile_never_enables_a_refused_demand` and
`paired_switch_waits_out_a_receive_callback_holding_the_ingress_gate` (the
switch side of the wait: it stores the paired flag, then blocks while a
receive callback holds the ingress gate) in `network.rs`,
`test_paired_lan_ingress_delivers_nothing_and_closing_stops_delivery` and
`test_paired_receive_callback_refuses_and_counts_every_lan_frame` in
`network_ingress_test.rs`, and
`crates/cerulion_android/tests/lan_access_test.rs` (a real zenoh peer over TCP).

The LAN posture is a whole-plane switch: a LAN source address proves no
identity, so per-peer LAN authorization, with each LAN peer's identity bound to
its pairing key, is not supported yet. The paired remote plane
binds each session to its pairing key over an authenticated transport and
authorizes each peer and each demanded topic, with revocation. Its wire protocol
carries the observe direction only (robot to desk topic data). The teleop
command direction is classified (`teleop` needs `CAP_TELEOP`) but has no
handler, so no verb takes the control lease or runs under the deadman yet. The
ops protocol still runs the verbs the robot registers (for example `restart` and
`engage-estop`), the act verbs among them (§5.1): with `CAP_TELEOP` and
role ≥ OPERATOR a desk publishes a message on a robot topic (`topic-publish`) or
calls a robot service (`service-call`), both refused while the e-stop latch is
engaged and neither taking the control lease, and with `CAP_OBSERVE` and
role ≥ VIEWER it reads one message of a topic (`topic-echo-once`) or a camera
snapshot (`topic-snapshot`).

- **The verb → capability table is the sacred security surface** (module docs of
  `authorizer.rs`): bootstrap verbs (`claim`/`pair`/`present-grant`/
  `code-pair-*`) are access-list-exempt and SELF-GATE in their handlers
  (chassis secret / cert chain / owner grant / an unforgeable `CpaceConfirmed`
  witness; `pairing_verbs.rs`); `engage-estop` is the permission floor (any
  paired account, regardless of scope); observation verbs need `CAP_OBSERVE`
  AND role ≥ VIEWER (non-vacuous: `Role` is an open `u16`, so a role less
  privileged than VIEWER holding `CAP_OBSERVE` is representable, and denied);
  lifecycle mutation needs role ≥ OPERATOR; `teleop` needs `CAP_TELEOP` AND
  role ≥ OPERATOR; `clear-estop` needs the OPERATOR role or higher and is
  refused to a row whose principal is cloud whether the allow-from-cloud switch
  is on or off (class `NonCloud`: the authorizer refuses it before dispatch, so
  the refusal is receipted `Denied`); `estop-status` needs `CAP_OBSERVE` AND the
  VIEWER role or higher; no host registers either e-stop verb yet, so a caller
  the authorizer admits gets `unknown_verb`; an UNCLASSIFIED verb is DENIED,
  never fail-open. Lower `Role` value = higher privilege.
- **The pairing subject is always the authenticated device key**, never read
  from client args (`pairing_verbs.rs` module docs; `ops_loopback_test.rs`).
  Validity and anti-rollback are evaluated against the robot's trusted
  `RemotedClock`, never a client-supplied time; an armed code's window runs on
  the store's `MonotonicClock` (section 7).
- **Live trust with fail-closed persistence** (`trust.rs`): one
  `Arc<Mutex<TrustInner>>` shared by the read side (accept gate) and write side
  (bootstrap verbs), so a claim/pair takes effect at the next accept with no
  restart. Persistence is store-BEFORE-index (a partial failure never leaves a
  durable key binding without its account row), and the live binding is
  published only AFTER the durable write
  succeeds; a verb that returned `Err` never leaves the key `Allowed`. The
  side-map is MAC-authenticated with the same key as the store, so a disk
  tamperer cannot forge a `key → owner` binding (`device_index_test.rs`).
- **E-stop is not serialized behind other sessions at the transport layer**: ops sessions are served
  CONCURRENTLY over one `Arc<cerud::OpsServer>` whose receipt sink is a
  brief-append mutex; a stalled/hostile session parks on a blocking read holding
  no lock; a paired `engage-estop` reaches the lease immediately, its safety
  EFFECT running BEFORE its audit receipt (e-stop is non-mutating, so no intent
  receipt precedes it; see `crates/cerud/src/server.rs`). Each session is bounded by
  `OPS_SESSION_DEADLINE` (120 s, `ops.rs`; a resource guard, not a
  serialization point). Pinned by `estop_starvation_test.rs`, which also
  doc-pins the 120 s value. Engaging the e-stop additionally publishes ONE
  incident capture request onto the machine-local flashback channel
  (`remoted/src/flashback.rs`), best-effort by design: a failed ask never
  fails the e-stop, and the verb type cannot be built without a real ask in a
  production build (`estop_flashback_test.rs`). On the daemon it also counts
  the engage with `rcp::RcpdEstop::send`, whose delivery sends an `estop` into
  the robot's `cerulion-rcpd` until rcpd answers (see the rcp plane, section
  6); the verb's answer never waits for it.
- **Lease over reconnects**: the holder/engager token is the stable
  TLS-authenticated device key, not the QUIC connection id; e-stop is robot
  state persisting across reconnects until cleared, and a different paired key
  cannot take the floor from the first engager (`crates/cerud/tests/lease_test.rs`
  pure; `estop_starvation_test.rs` e2e). The deadman window is
  `LEASE_DEADMAN_WINDOW_MS_ROBOT_CONFIRM` (500 ms); it requires validation
  against the deployed robot's own stopping behavior before it can be relied on.
  See `docs/remote_plane.md`; the doc's figure is pinned against the constant by
  `crates/cerud/tests/constants_test.rs`.
- **Receipts**: every ops request is authorized, dispatched and receipted into
  the append-only hash chain: allowed, denied, unknown, or failed. For a
  MUTATING verb the intent receipt is written BEFORE the side effect
  (fail-closed: a failed intent write means the effect never runs)
  (`crates/cerud/src/server.rs`; `crates/cerud/tests/authz_receipt_test.rs`).
- **Never bricks offline**: an established pairing connects on the LAN with the
  issuer down and the relay unreachable: direct dial + the offline trust store;
  the issuer and relay are conveniences, not dependencies
  (`offline_connect_test.rs`).

### 5.1 The act verbs (`act_verbs.rs`)

Four ops verbs run generic operations on the robot's own shared memory for a
paired peer, so a desk can act on a ROS 2 stack (on `rmw_cerulion`) or a
Cerulion graph without a LAN egress path for the topic or service:

| verb | row in `classify_verb` | mutating | result |
|---|---|---|---|
| `topic-publish` `{topic, type, message, times?, rate_hz?, wait_subscribers_s?}` | `CAP_TELEOP`, role OPERATOR | yes | `{topic, type, frames, subscribers, skipped, line}` |
| `service-call` `{service, type, request?, timeout_s?}` | `CAP_TELEOP`, role OPERATOR | yes | `{response, response_text}` (`response_text`: the response as one JSON line, fields in declaration order) |
| `topic-snapshot` `{topic, timeout_s?, quality?}` | `CAP_OBSERVE`, role VIEWER | no | `{jpeg_base64, width, height, encoding, stamp_ns, bytes}` |
| `topic-echo-once` `{topic, timeout_s?}` | `CAP_OBSERVE`, role VIEWER | no | `{type, stamp_ns, sequence, message}` |

The operations are `cerulion_msgio`'s, the ones `cerulion topic pub`,
`service call` and `topic snapshot` run locally. Types resolve from the
built-in corpus plus the `.msg`/`.srv` store under the directory
`CERULION_REMOTED_SCHEMAS_DIR` names (read per call). A failure is
`CerudError::Verb` whose text starts with the error class (`usage: `,
`encode: `, `unavailable: `, `internal: `); the desk maps the class to the
CLI's exit code (2, 1, 3, 1). Timeouts are capped at 60 s and a result at
12 MiB, inside the 16 MiB ops frame, and a `message` or `request` at 1 MiB of
text. `topic-publish` and `service-call` are refused while
the e-stop latch (`cerud::lease::ControlLease`) is engaged; a running publish
or call checks the latch every 10 ms and stops when it engages, a call
stopped before its request goes out sends none, and a publish sends each
frame, and a call its request, only while it holds the lease's lock and reads
the latch clear. The rcp plane shares this latch: an rcp `estop` engages it,
and while it is engaged rcp tool calls other than `estop` are refused (the rcp
plane below). A
publish is
refused when its subscriber wait plus its frames at the rate exceed 90 s, and
is stopped if it is still running 100 s after it started (a frame that takes
longer to send than its period delays the rest). A topic under
`/__cerulion` is refused. `service-call` also returns `response_text`.

## 6. The wire plane (robot side)

A `cerulion/wire/2` connection (admitted only for a paired `CAP_OBSERVE`
account) carries topic data robot to desk only: no verb takes a topic frame from
the desk, the robot opens uni data streams and never accepts one, and the desk
accepts them and never opens one. The desk sends control requests only. It
carries ONE bidi control stream (length-prefixed JSON:
`catalog`/`demand`/`undemand`/`schema`/`status`/`sync_epoch`, the same
`cerulion_q` vocabulary the LAN uses, imported not mirrored) and ONE uni data
stream per demanded topic (a JSON `StreamPreamble`, then raw wire frames
VERBATIM; per-topic QUIC flow control means a slow topic never stalls a fast
one, and the demand lifecycle IS the stream lifecycle). End-to-end over real
iroh + real iceoryx2: `wire_plane_test.rs` (byte-for-byte against hand oracles).

Schema metadata reads create no policy services when no data service exists.
Their temporary listener leases would otherwise destroy an empty marker while
a later policy reader opens it. Actual header taps acquire source and egress
leases, retaining them through the data port's lifetime. Schema responses check
producer privacy and account authorization after the provider returns.

- **Framing is NOT cancel-safe** (`cerulion_link::framing`): `read_frame`/
  `write_frame` are two sequential awaits, so a dropped future desyncs the
  stream permanently. Never `select!` on them; one dedicated task per stream
  drives them to completion. Every consumer in the family obeys this
  (wireclient's reader, connectd's worker, netd's iroh plane, remoted's serve
  loop and rcp stream reader); the pattern is documented at each site. *The
  rule itself has no single structural gate, verified at
  `crates/cerulion_link/src/framing.rs` module docs + each caller.*
- **The tap set is the gate** (`tap.rs`): a demand attaches remoted's own
  listener-less `DataOnlySubscriber`; un-demanding drops the tap; an
  un-demanded topic costs nothing. Taps are observation-only: the drain thread
  copies each frame out of the SHM slot immediately (never holds the borrow),
  and only the QUIC writer task ever blocks on flow control.
- **WAN backpressure is bounded drop-to-live** (`forward.rs`): a per-topic queue
  of depth `WAN_QUEUE_CAP = 8` between drain and writer; when the WAN stalls,
  the OLDEST frame is dropped, counted, and warned once-per-regime
  (`WanDropLatch`: first drop warns, repeats are debug, room re-arms). Never an
  unbounded buffer, never a blocked drain (blocking would back-pressure into SHM
  and perturb graph timing).
- **Per-demand authorization + mid-session eviction**: every demand runs the
  live `DemandAuthorizer` (backed by `TrustStore::is_allowed`), and an
  independent sweep re-checks streaming demanders every
  `DEFAULT_REVOCATION_SWEEP_INTERVAL` (2 s, `wire.rs`); a mid-session
  revocation or grant expiry closes the connection. Pinned by
  `wire_plane_test.rs` (`demand_gate_refuses_a_denied_topic_and_admits_others`),
  `multi_device_revoke_e2e_test.rs` (desk A evicted, sibling desk B of the SAME
  account streams on), and, against the inert-wiring class,
  `daemon_demand_gate_test.rs` (the authorizer wired by the production
  `serve_endpoint` seam itself, so reverting the one wiring line fails a test).

### The rcp plane (`cerulion_remoted::rcp`)

Each bidi stream on `cerulion/rcp/1` is one MCP session; each frame is one
JSON-RPC 2.0 message. The robot is an MCP streamable HTTP client of its local
`cerulion-rcpd` (`--http`, default `127.0.0.1:7801`): every admitted message is
one `POST` with rcpd's bearer token (read per session from
`remoted/rcp.token`), the session id and protocol version rcpd issued, and
`x-cerulion-caller: iroh:<device hex>/account:<account hex>`. rcpd runs the call
under that caller (`cerulion_rcpd::caller`), so its receipts and the journaled
call frame name the remote desk.

A task of its own reads each stream's frames, and the session reader selects
on that task's channel, never on the framing. The session reader never waits
on rcpd: it admits each message (the class check, the e-stop engage or
refusal, the one-initialize check and the slot) and hands it, without waiting,
to the stream's forwarder task. The forwarder sends the messages to rcpd in
the order they were admitted; it waits for a lifecycle message's response
head, and for the `initialize` result, before the next, and hands Observe, Act
and Estop messages to tasks of their own, which send them and relay the
answers. Such a task opens its connection to rcpd, then runs `rcp::SendCheck`
just before the send: `rcp_authorize` for an Observe or Act message's class
again, and for an Act message the e-stop latch again, so a call is refused
after a revoke or an engage that lands while it waits in the queue or for its
connection. A `notifications/cancelled` naming a request whose response head
is not in yet is sent by a task of its own once that head is in, within
`RcpLimits::head_timeout`. rcpd (rmcp's stateful streamable HTTP server) sends
a request's SSE head once the request is in the MCP session, before the tool
runs, so the cancel reaches rcpd after rcpd has taken the call it names, or
once `--rcp-head-timeout-ms` passes; requests stay concurrent up to
`--rcp-in-flight`, and the forwarder never waits on a cancel.
Its bounded queue holds `in_flight + estops_in_flight + 4`: a request holds
its slot while queued and a notification one of four notification places
(`rcp::FORWARD_NOTIFICATION_ROOM`), so a request that holds a slot always
fits; a notification with no place free is dropped. The forwarder stops once
the stream's writer has stopped (a revoke closes the connection from the
writer) and is aborted with its session. So an `estop` engages the ops latch
while rcpd holds a lifecycle request.

`rcp::rcp_authorize` is the one decision function, over one locked read
(`SharedTrust::snapshot_row_for_key`: claimed flag, key access, allowing row).
At accept (`classify_accept`) it needs a claimed robot and an allowed key. Each
message is parsed strictly (`classify_message`: one object, `jsonrpc` `"2.0"`,
string `method`, object `params`, string or integer `id`, no other member) and
classified by method: lifecycle needs an allowed row, `resources/*`,
`subscriptions/listen` and `tools/list` need `CAP_OBSERVE` with role value
`<= VIEWER`, `tools/call` naming `estop` needs an allowed row (the e-stop
floor), any other `tools/call` needs `CAP_TELEOP` with role value `<= OPERATOR`
(`rcp::requirement`). The canonical re-serialization is what reaches rcpd. Each
frame back is checked against the class that produced it, and the connection
checks the session floor every `RcpLimits::revoke_check`
(`--rcp-revoke-check-ms`), with or without a stream open; a key that fails the
floor has its connection closed. Pinned end to end against a real rcpd by
`rcp_forward_e2e_test.rs`. The desk half is `cerulion rcp`
(`cerulion-connectd rcp`, `connectd::rcp::bridge`): stdin lines to frames,
frames to stdout lines. A robot frame that is not a JSON-RPC 2.0 message is
never written to stdout: the accept decision of a refused key, an
application close with `rcp::CLOSE_REVOKED` or `rcp::CLOSE_SUPERSEDED`
(mirrored as `connectd::rcp::CLOSE_*`), the robot ending the stream
before stdin's EOF, and a stream that breaks (a reset, a lost connection, a
frame cut off, or a robot frame over the 16 MiB cap;
`cerulion_link::read_frame_or_end` tells these from a finish at a frame
boundary) each end the desk process with status 1 and an error line, the
break's cause passed through `sanitize_peer_text`.

One e-stop latch covers the ops act verbs, `restart` and the rcp plane: the
ops plane's `cerud::lease::ControlLease` behind one `EstopLease` (the lease
plus the count of e-stop engages waiting for its lock). The daemon builds it once
(`OpsHost::daemon`); `serve_core` hands it to `OpsServing` (for
`engage-estop`, `restart` and the act verbs) and a clone to `RcpPlane`, so both planes
share the latch and the count. `serve_core` also gives the daemon host an
`rcp::RcpdEstop` over the same `RcpLink` (`OpsHost::with_rcpd_estop`), which
`EngageEstopVerb` holds. A library host has no lease, no `RcpdEstop` and
serves no rcp plane. `EstopLease::holds` is the blocking read of the latch that the act verbs'
refusal, the rcp plane's Act check and `restart` share: an engage is counted
(read without the lock, and again under it, so an engage that counts itself
in while the read waits for the lock is not missed), or the latch reads
engaged under the lock, held for those reads only, and a poisoned lock reads
engaged. The act verbs' 10 ms
watcher and their send path read the latch themselves (above). The daemon
registers `restart` as `RestartVerb::new().with_gate(..)` over an
`ops::EstopRestartGate`, and cerud asks the gate before each step. It asks
for `RestartStep::Stop` before it parses the args, so the refusal reads the
same on every host and nothing runs. It asks for `RestartStep::Start` after
the stop command and before the start command, so after an engage during
`systemctl stop` the start command does not run; that refusal is
`unavailable: ` followed by `ops::RESTART_STOPPED_NOT_STARTED`. The pure
decision is `estop_policy::restart_check`. Each check takes the lease's lock
at most once, for one read, and never holds it across a command, so an engage
after the second check does not stop the graph that `systemd-run` starts. A
library host still refuses `restart` by name. The session reader engages the
latch through `EstopLease::engage` for an
authorized `RcpClass::Estop` message right after `rcp_authorize`, before the
upstream check and the e-stop bound, so an unreachable rcpd or a spent bound
still leaves it engaged; a forward failure never undoes it. The engage is
counted before it waits for the lease's lock, so an act verb sends nothing
more once it is counted. The engager is the session's caller string. A
direct engage writes no ops receipt; rcpd still receipts the forwarded
`estop`. For an `RcpClass::Act` message the reader reads the count, then the
count and the latch under the lock (a poisoned lock reads engaged), and while an engage is
counted or the latch is engaged answers `-32001` `refused: the robot's
e-stop is engaged`, or for `estop_clear` (still `RcpClass::Act`) `refused:
the robot's ops e-stop is engaged; it clears when cerulion-remoted
restarts`. The lease guard is held only for that read or that state change,
never across an `.await` and never with a connection, session, trust or
receipt lock. The latch is read when a desk's message arrives and again on
the open connection to rcpd just before the plane sends it. A call already
sent to rcpd is stopped by rcpd's own latch, which an rcp `estop` closes in
its stream and an ops `engage-estop` closes through `RcpdEstop::send`:
right after `EstopLease::engage`, and before its `e-stop ENGAGED` warning,
the verb decodes the caller's device key and calls `send`, which returns
at once (in the daemon every `tracing` line, the delivery's failure and
closed-latch lines included, is queued for the log writer, so a full stderr
holds neither the verb nor an attempt); a caller id that is not
a device key sends nothing and logs that at warn. `RcpdEstop` shares an
`Owing` with its delivery: a `period` mutex over the period's first engager
and the last delivery id, an `engages` count and the `running` delivery
id (0 for none). Under `period`, `send` names the first engager of the
period, adds one to `engages`, and, when `running` is 0, sets it to a new id;
it then releases `period`, builds the delivery, its `Running` guard first,
spawns `Delivery::run` on the runtime handle `serve_core` captured, in the
engage's span, and keeps the handle of the delivery with the larger id.
Otherwise it releases `period` and logs at info that the running delivery
covers the engage. `period` is held only for those steps, never across an
`.await`; no log line is written under it, and no `Running` guard is dropped
under it, since the guard's drop takes it. Each attempt reads `engages` as it
begins, reads the token through `load_token`, then sends `initialize`,
`notifications/initialized` and `tools/call` `estop` with `reason`
`rcp::OPS_ESTOP_REASON` (`estop_session`), each POST on a fresh loopback
connection with the headers `Upstream::request` builds (`x-cerulion-caller`
is `caller_identity(key, account_for_key(key))` of the period's first
engager), its response head within `estop_policy::estop_head_bound` of
`RcpLimits::head_timeout` (at least `OPS_ESTOP_HEAD_FLOOR`, 1 s) and its
answer within `OPS_ESTOP_ANSWER` (10 s) after it, then a `DELETE` of the
session rcpd issued it, within `OPS_ESTOP_DELETE` (2 s), also after a
stop by Ctrl-C or SIGTERM. It succeeds only when the call's result says
`"estopped": true` (`estop_outcome`); the delivery records on its `Running`
guard, as soon as the call returns and before the `DELETE`, the `engages`
count that attempt began with (`answered_at_start`), logs the answer at warn
with `failed_attempts` after the `DELETE`, then calls
`Owing::finish` on its own task. `finish`, under `period`, writes no log line and asks
`estop_policy::after_answer` of the count at the attempt's start and
now: `Done` ends the period and clears `running` while it still holds
the delivery's id, so the next engage starts a delivery, and `Again`
(an engage came during the answered attempt) sends at once again.
After a failed attempt the loop logs through one `FailureRegimeLatch`
per delivery (the first failure and each decade of failures at warn with
`reason`, rcpd's text through `sanitize_peer_text`, the others at debug
with `suppressed`), then sleeps `retry_in` (`OPS_ESTOP_RETRY_FIRST`, 100 ms,
doubled after each failure up to `OPS_ESTOP_RETRY_CAP`, 2 s).
Both the attempt and the sleep race the stop flag, a `watch` that
`RcpdEstop::shut_down` sets once `serve_planes` returns; `shut_down` then
waits for the task within `CONNECT_TIMEOUT`; a delivery whose `send` has
not yet stored its handle is not awaited, and it reads the stop and ends
as on any stop. A `Running` guard in the delivery clears `running` on any
other end (a stop by Ctrl-C or SIGTERM, which `serve_core`'s teardown sends
through `RcpdEstop::shut_down`; a panic; a runtime dropped with the task; or a
`spawn` on a runtime that has shut down, which drops the delivery on `send`'s
thread) through `Owing::clear_mark`, which under `period` swaps the guard's id
for 0 and, when it held it, reads `engages`; the guard writes any line after
`period` is released. With `answered_at_start` unset, it logs at error that
rcpd had not answered. With it set (an end during the `DELETE` of the attempt
rcpd answered, on a panic or a dropped runtime), it asks `after_answer` of it
and the count it read: `Done` logs nothing, and `Again` logs at error that the
`estop` owed to a later engage was not sent and that the next engage sends it.
On `finish`'s `Again` the delivery resets `answered_at_start` before its next
attempt, and an error answer leaves it unset. One that reaches `finish`'s
`Done` has its mark cleared there, so its guard logs nothing. A
`cerulion-remoted` that is killed (SIGKILL), crashes, or ends at the hard-exit
deadline or a second stop signal (`_exit(4)`) runs no destructor, so it may
end with no line and no `DELETE`. A `send` and an ending guard serialize on
`period`: an engage counted before the guard's swap finds `running` set, is
logged as covered and is in the count the guard reads, so the guard logs its
early-end line when rcpd had not answered, its owed line when the answered
attempt began before that engage, and nothing when that attempt began after
every engage counted; one counted after the swap reads `running` as 0 and
starts a delivery. A delivery that ends before `finish` answers `Done`,
answered or not, leaves its period open, so the next delivery still names the
period's first engager
and rcpd's journal can name an earlier device for a later engage. Every
delivery line names the engager as `by` (the device key, hex); the warn
failure lines and the closed-latch line also carry rcpd's caller identity as
`caller`. Nothing the delivery does reaches the verb's answer or the lease.
rcpd refuses each mutating call it holds when the `estop` reaches it and has
not yet published once its latch closes; the one it is running was already
published. `ControlLease::clear_estop` has no caller outside tests, so the
latch clears only when `cerulion-remoted` restarts; then rcpd's own latch
clears through its `estop_clear` (`limits.estop_clear: agent`) or by deleting
`<receipts>.estop` and restarting rcpd.

When rcpd answers HTTP 404 to a request that carried its session id (rcpd
restarted and lost the session), the request is answered `-32603`, and the
session reader reads on until the desk ends its side or `--rcp-drain-ms`
passes, sending nothing more to rcpd: an authorized `estop` engages the ops
latch through `EstopLease::engage` and is answered `-32603` with a message
that says only the ops latch is engaged, every other request gets the
session-lost error, and notifications are dropped. Its `select!` is biased to
take a frame already read before the loss. The forwarder still sends what it
holds, the tasks drain within what is left of the drain bound, and the stream
finishes. A response body that ends without the response to its request (rcpd
stopped mid-call, or a `notifications/cancelled` closed it) is answered
`-32603` as well. A notification takes no in-flight slot, so a cancel reaches rcpd while
every slot is taken.

## 7. The desk half: connect, pair, and the client substrate

- **`cerulion connect <robot>`**: the CLI resolves robot address + demand set
  into a `cerulion-connectd` argv and spawns the sibling binary
  (`connect_cmd.rs`, pure + oracle-tested; `CERULION_CONNECTD_BIN` overrides
  the binary). A positional name resolves via `~/.cerulion/robots.toml`; a
  pinned robot's name just works. No `cerulion` verb writes the file: `pair_cmd`
  pins a name after a successful `cerulion-connectd pair` ceremony, and nothing
  calls `pair_cmd`, so it holds hand-written pins and any a 1.x `cerulion pair`
  wrote.
  connectd's `worker::run_connect` dials, fetches the catalog, demands, and
  spawns one reader per accepted topic; the whole loop is pinned over real
  loopback iroh + real per-test SHM roots by `reinject_e2e_test.rs`.
- **`cerulion pair`** is a hidden stub: no robot-side verb arms a pairing code
  (`SharedCodePairSessions::arm` in `cerulion_remoted::pairing_verbs` is called
  only by `PairCodeArm`, which no daemon or host runs), so `dispatch` in
  `main.rs` refuses every spelling
  with `PAIR_UNAVAILABLE_MSG` and exit 2 above the login gate, and sends only
  its `cli_command_run` (verb `pair`, exit 2). The ceremony below lives on in
  `cerulion_cli_engine::pair_cmd` and `cerulion-connectd pair`, which no
  `cerulion` verb calls.
- **The code ceremony** (`cerulion-connectd pair`): the desk is the CPace initiator over the robot's
  ops plane (`pair.rs`); a wrong code is detected structurally desk-side and
  mapped to a distinct exit code; stdout carries machine-parseable state lines
  for Studio, everything else goes to stderr. Pinned against a REAL in-process
  remoted ops plane by `pair_e2e_test.rs`, with the robot's durable trust state
  verified from disk.
- **Desk key discipline** (`wireclient::config`): a key file is exactly 32 raw
  bytes; `resolve_or_create_desk_seed` creates 0600 with `create_new` and NEVER
  overwrites an existing key (a wrong-size file is a loud error, never silently
  regenerated); no key file ⇒ an EPHEMERAL identity, which an un-paired robot
  refuses. The cloud account rides `~/.cerulion/device.cert`
  (`base64url(postcard(SignedDeviceCert))`, written by `cerulion login`);
  `parse_device_cert` refuses a cert attesting a different device key, through
  the same verifier the CLI uses. In-module oracle tests.
- **The reader loop** (`wireclient::reader::run_topic_reader`): one task per
  topic owns its uni stream, validates every frame (`total_size` +
  `schema_hash`) and re-injects via `IngressInjector` on the desk's
  `network:None` `TransportManager`; no unbounded buffering. Driven by
  connectd's session driver; exercised end-to-end by `reinject_e2e_test.rs`.
- **Peer text is hostile**: everything a robot says about itself (names, topic
  lists, error prose) is sanitized + length-bounded where it enters an error or
  a log line; `sanitize_peer_text` is the ONE policy, re-exported so the binary
  renders the catalog through it too (`connectd/src/lib.rs`, `pair.rs`).
- `SharedCodePairSessions` keeps the armed code's ledger. Every desk shares
  one budget of tries. `code-pair-start` admits a try while one is left,
  inside the window, and at least `PAIR_CODE_RATE_INTERVAL` after the robot's
  last admitted try, whatever code that was for (a new arm does not reset it);
  a refused start uses no try, and the pure `admit_try` checks in order: used
  (a desk paired), every try failed, expired, no try left (the rest pending
  or being saved), throttled. A start reads the store's clock with its state
  lock held, and a reading before the last admitted try is refused as too
  soon, so admitted tries are at least the interval apart however the starts
  of several desks race. Each admitted try gets a fresh CPace responder
  created at the arm time with the policy's tries, so its own liveness check enforces the same
  window and its own attempt bound never fires first. The responder's CPace
  step runs with the state lock released: a start takes its try under the
  lock, runs the step, then stores the responder under the lock again, and a
  separate lock makes starts, arms and disarms take turns, so an ops
  session's open and close (an `engage-estop` session's too) never wait for a
  CPace computation (`estop_starvation_test.rs`). A code that ended while the
  step ran refuses the try: expired counts it failed, paired drops it
  uncounted. A wrong tag at
  `code-pair-finish` or a failed `respond` fails the try at once. A desk that
  sees a wrong code itself never calls `code-pair-finish`, so its try stays
  pending until every ops session of its key has ended: `OpsServing` opens
  each session on the store inside the session's blocking run and closes it
  after `serve_connection` returns, so the try is counted failed only once no
  finish for it can arrive, and its responder is dropped then. A desk key's
  new try counts its pending one failed; when the window passes, every pending
  try is counted failed. A right code's row is saved with the store's state
  lock released (a separate lock serializes saves, and arms and disarms wait
  for it), so neither the wire
  `status` nor the serving-hold source waits for a disk; a save that fails or
  panics counts as a failed try (`SAVE_FAILURE_IS_A_FAILED_TRY`), and the
  desk's next try gets a fresh responder. The pure `settle` returns the first
  outcome, which `CodeState::latch` latches: paired (the row was saved),
  exhausted (every try failed), expired (the window passed), and none while a
  right code's row is being saved. A
  paired code refuses every later try (`CODE_USED`), so one code pairs one
  desk, and the other desks' pending tries are dropped uncounted. The window
  and the rate interval run on the store's `MonotonicClock` (the process's
  `Instant` in production), so a step of the wall clock neither stretches nor
  cuts them; on Linux and macOS time the system spends suspended is not
  counted, so a suspend lengthens an armed code's window and its hold by that
  time. The row's own times still come from `RemotedClock`. The refusal texts
  are the consts `NOT_ARMED`, `CODE_USED`, `TRY_THROTTLED`,
  `TRIES_IN_PROGRESS` and `NO_SESSION`, and the functions `code_exhausted` and
  `code_expired`, which give the CPace errors' own text; the desk maps
  `code_exhausted` and a wrong code to exit 4 and every other refusal to exit
  2.
  The armed code keeps the daemon serving through a `ServingHolds` source that
  reads one atomic and the store's monotonic clock: true from the arm until a
  `paired` or `expired` end is written, for an `exhausted` code until the
  window plus `HOLD_MARGIN` (so a desk that arrives later still reaches the
  robot and is told the code was used up), and never past the window plus
  `HOLD_MARGIN`; which written ends release the hold at once is the pure
  `written_end_releases_hold`. That source and the `PairCode`
  guard the arm step holds while it arms a request are all a code adds to the
  fence: a bootstrap-only ops session (`claim`, certificate `pair`,
  `present-grant` or `code-pair-*` from a key that is not paired) holds no
  guard of its own whether or not a code is armed (section 5), so no peer that
  is not paired holds the robot serving past the code's end being written or
  its window plus `HOLD_MARGIN`, however it reconnects, and a desk that pairs
  with the code runs its later calls fenced. While no code holds serving and
  netd's idle stop is committed, the arm step's `PairCode` guard is refused, so
  no code is armed then.
  The wire `status` reply carries the armed code's state as `pair_code`
  (`status`, `tries`, `failed`), and the refused requests as
  `pair_code_refused`, absent while none was, next to `log_lines_dropped`.
  Pinned over the real ops plane by `ops_loopback_test.rs` and against the
  wall clock by `code_pair_account_test.rs`.

## 8. netd's iroh WAN plane (the dual-plane fold)

`network-daemons.md` §8 covers the plane's behavior; this section maps
ownership and the known limits. netd owns THE one mirror per `(robot, topic)` for
BOTH transports: zenoh on the LAN, iroh over the WAN (`wan.rs` module docs).
`WanRegistry` (built from `CERULION_NETD_WAN_ROBOTS`, `CERULION_NETD_DESK_KEY`,
relay + cert + epoch env) maps robot names to dial parameters; `pick_plane` is
pure and static for netd's lifetime: a WAN-registered robot goes to iroh,
everything else to zenoh (`wan.rs` unit test
`pick_plane_routes_wan_robots_to_iroh_others_to_zenoh`; e2e
`crates/cerulion_netd/tests/wan_plane_iroh_test.rs`, which stands a REAL in-process
`cerulion_remoted` robot up over loopback iroh).

- **The injector is created SYNCHRONOUSLY before the reader spawns** (LAN
  parity): a slot-taken failure fails the demand with no mirror state behind:
  no phantom mirror (`iroh_plane.rs` module docs; the slot-taken and
  release-frees-the-slot arms are in `wan_plane_iroh_test.rs`).
- **Bounded everywhere**: the daemon calls the plane under the registry lock, so
  the plane owns its own tokio runtime and every await in the dial/demand path
  is timeout-wrapped (`DEFAULT_WAN_DIAL_TIMEOUT` = 15 s, deliberately shorter
  than connect's 30 s `DEFAULT_ROBOT_TIMEOUT`). A control-stream error poisons
  the (cancel-unsafe) connection, which is dropped and re-dialed.
- **Reader death tears mirror state down** (injector freed, provenance removed);
  pinned by the reader-death arm of `wan_plane_iroh_test.rs`. The pairing gate
  holds through the fold: an unpaired desk key is a loud typed refusal with no
  connection tracked (same file).
- **Behavior not provided today**, each recorded in the module docs of the
  component that would carry it: unifying netd's own re-inject loop with
  `wireclient::reader::run_topic_reader` (the shared element today is only the
  `IngressInjector::reinject_raw` primitive): `wireclient/src/reader.rs` module
  docs; moving plane I/O off the registry lock, and the plane→daemon
  death-notification seam (so a re-demand after reader death auto-re-creates):
  `iroh_plane.rs` module docs; the automatic prefer-LAN-when-also-reachable
  picker: `wan.rs` module docs ("the plane-picker's scope").

## 8b. The account-first path: register at first serve, reach by account

Serving machines consume other robots through the same endpoint owned by
`cerulion-remoted`. The private `remoted/outbound.sock` control service uses
kernel peer credentials and bounded framed requests. It validates the live
device key and shared-memory namespace before dialing, then runs the existing
wire demand and validated reinjection path. No wire publish verb is involved.
Per-call metadata carries only its selected peer. Mutation requests carry that
peer plus peers with acknowledged live readers, from one registry snapshot;
removal still retires affected readers without coupling framing to account size.
Graph ingress retains one epoch-refresh failure latch per route across reconnects.
A refresh failure reports its cause and preserves saved authenticated trust;
source selection, pin validation and wire authorization remain mandatory.
Release retires the reader before acknowledgement; closing local control
retires every mirror on that connection. Account fencing retires outbound
sessions without closing the serving endpoint. The delivery and refusal pins
are in `crates/cerulion_netd/tests/wan_plane_iroh_test.rs`.

The ceremony above (a chassis secret, a pairing code, an operator typing at the
robot) is the path for a machine nobody has an account for. A machine that IS
logged in needs none of it, because `cerulion login` already proved which
account owns it. The shared account HTTP client sends `cerulion-pairing-protocol: 1`, which opts
into the hosted device-certificate and robot-registration routes. A client
without that header receives the identity-only compatibility response. New
clients must reuse `login_cmd::build_http_client` or send the same header.

**On the robot, `cerulion login` is the only account step.** The first
live egress registration registers the machine and starts `robot_supervisor`.
An explicit LISTEN configuration also retains its standing-server behavior,
but no LISTEN setting is required for authenticated serving. A query-only desk
does not start the supervisor merely because it discovers robots. Established
pairing state can serve without introducing a new login or enrollment step. A failed registration, or a
`cerulion-remoted` that exits, is retried with a backoff that doubles from 5 s
to a 300 s cap, at most 30 consecutive attempts; a sequence that stayed up for
10 minutes starts the count again. Each failure is a warning naming the reason
and the next wait.

The supervisor resolves two SIBLINGS of the running netd binary and runs them in
turn: `cerulion bootstrap-robot --state-root <root>`, a hidden login-gated
worker whose stdout is a machine-readable result, then `cerulion-remoted
--provision-bundle <path>` to write the robot state, then `cerulion-remoted` to
serve. The hidden worker exists so netd reuses the CLI engine's login and
registration code without a `netd -> cli_engine` package edge; no operator ever
runs either by hand. All three binaries must therefore sit in one directory.
The release archives after v1.0.0, the installer and the Debian package place
`cerulion`, `cerulion-netd` and `cerulion-remoted` side by side, so a released
install can start this supervisor path on its first live export, without a
listen setting.

The robot's identity is the login device key, so the endpoint id a desk dials is
the key registration proved possession of. Roots come from the issuer's
well-known endpoint. Registration is idempotent: an already-registered machine
does nothing. If the account service is unreachable at first serve the WAN plane
cannot register new account ownership and retries on the backoff above. Existing
validated registration and pairing state can continue offline. The paired LAN
gate remains closed; an unavailable issuer never enables unauthenticated access.

**On the desk, there is no pairing step.** `topic list` and `viz --robot` read
the account's robots from the issuer's owner-only list endpoint and mark each
one with a bounded presence probe. The directory fetch inside `topic list` has
its own 2 s deadline (`LISTING_FETCH_BUDGET`); stale-session refresh has a separate
2 s caller-thread request budget. A successful token rotation has an additional
1 s settlement budget for store publication or revocation after a concurrent
sign-out or sign-in. HTTP never holds the store lock, and publication still
checks the exact refresh-token identity before writing. A legacy sign-in with a valid session and no coupled
device certificate gets one background enrollment attempt per process, with a
2 s caller budget. It signs the account service's device-registration challenge
using the existing machine key, sends the session only to that service without
redirects, and verifies account, key, issuer and validity before publication.
Publication holds the bounded auth-store lock, rechecks the unchanged session
and key, stages consumer certificates, and commits the coupled chain last.
Failure keeps the re-login remedy; local reads keep its diagnostics at debug.
Enrollment is skipped under local scope and the network kill switch. Past a directory deadline,
or on any error, the listing prints its LAN half and one
`account robots not listed: <reason>` line. Selection is by exact, case-sensitive display
name, or by the reserved `account:<robot id>` route; a display name shared with a
LAN robot is reported as a collision and refused rather than guessed. A
directory entry is never reachability evidence by itself. The first demand of a
remote account robot dials it through netd's internet plane with the login key
and presents the owner's certificate chain ONCE, and only on a classified
unpaired refusal: never after expiry, revocation or a denied permission, and
never as a silent fallback to a code. On the robot side the `pair` verb admits a
chain whose account equals the owner account with no robot-scoped grant; every
other account still needs the full grant presentation. This is **certificate
presentation**: no code and no user step, and the presenting device gets its own
row `(owner, device)` with the owner row's scope and the certificate's principal.
The hosted service's device is admitted exactly this way, as a cloud row.

**Code pairing cannot name an account.** Possession of a code and of the device
keys proves neither, so `code-pair-finish` binds the guest to the self-account
derived from the authenticated transport key and refuses any other value; an
account certificate requires the full roots-verified chain whose device key equals
the transport peer key, presented through `pair` (certificate presentation, above).
The owner row is never preserved for a guest, because that
would hand the guest the owner's authority outright
(`crates/cerulion_remoted/tests/code_pair_account_test.rs`).

**One endpoint per machine.** A machine whose netd hosts a LISTEN gateway owns
its endpoint through the sibling remoted, so netd's OUTGOING demands from that
machine are refused at the point of demand with the reason, never by opening a
second endpoint. Robot-to-robot over the internet, and a desk that also serves,
are the documented restriction.

### Which plane served a mirror

Both planes re-inject into the same local shared memory under the same topic
name, with the same wire bytes, so nothing downstream can tell a locally
mirrored robot from one dialed over the internet. A robot on the same network is
reachable over both, which is exactly how a test that checks only that the
frames arrived can pass with the internet path never exercised: a serving
machine opens the LAN plane on every interface it has, so a desk that learns one
locator gets the topic locally.

`MirrorPlane::serving_plane` is the answer, and the `status` demand table
reports it per row as `plane`. A composing plane answers from the route it
PINNED when it ensured the mirror, not from a fresh decision, so the field
describes what happened rather than what a picker would choose now. The field is
optional on the wire: absent means the daemon does not report a plane, never
that the local plane served it, because a daemon that predates the field omits
it and any default would be a routing claim it never made.
`crates/cerulion_netd/tests/wan_plane_iroh_test/serving_plane.rs` pins it
against the delivery it describes, on a desk whose local plane cannot mirror at
all.

## 9. Revocation epochs: mint, carry, apply, evict

The full lifecycle is `docs/revocation.md`; the contributor invariants:

- **Apply is monotonic and wholesale** (`TrustStore::apply_epoch`): only a
  strictly-newer epoch is applied; its account AND device revocation sets
  REPLACE the current ones (the tombstone: re-presenting an old grant cannot
  resurrect a revoked device; only a newer epoch dropping it re-admits).
  Rollback-floor-gated like every other acceptance path. Pinned by
  `store_test.rs` + `chain_verify_test.rs`; device granularity end-to-end by
  `multi_device_revoke_e2e_test.rs`.
- **The robot fetches its own epoch.** On a registered robot, netd's supervisor
  runs the hidden `cerulion robot-sync-epoch --state-root <root>` worker (an
  internal process, never an operator command; it refuses without a robot
  registration or a provisioned `remoted/` directory) at start, every
  `CERULION_NETD_EPOCH_SYNC_PERIOD_S` (default 300 s),
  early on remoted's `epoch.request`, with 5 s to 300 s backoff and a 120 s
  deadline above its own worst case, so the deadline never cuts a session
  refresh in flight. It fetches `/v1/robots/{id}/access` with the robot's own
  login and writes `remoted/epoch.pending` (`hex(postcard(EpochSyncWire))`);
  `cerulion_remoted::epoch_pull` applies it within 2 s. This is the path that cuts
  a stolen desk. Session refreshes are serialized across processes by an
  exclusive lock beside `auth.json` (`.auth-refresh.lock`): a refresh token is
  single-use, and the loser of a race uses the session the winner wrote.
- **Desks also push.** Both desk dial paths carry the cached epoch
  unconditionally; there is deliberately NO knob to withhold it (by
  design: a withholding knob would be a footgun). The shared substrate is
  `wireclient::epoch` (`prepare_epoch_push` / `classify_epoch_reply`); the robot
  side is the `sync_epoch` wire verb feeding `SharedTrust::apply_epoch`. The
  production e2e is `epoch_push_e2e_test.rs`; the inert-wiring pin (the sink
  wired by `serve_endpoint` itself) is `daemon_epoch_sink_test.rs`.
- **Freshness never blocks a connection.** Every outcome except a mid-frame
  transport failure is policy: recorded, logged at its severity, dial proceeds.
  Severity is three classes, not two (`EpochPushOutcome::severity()`): Current
  (info), NothingToCarry (debug: a never-synced/guest desk is a PERMANENT
  steady state, not a warning), NotCarried (warn: the one class an operator
  must act on).
- **One cache, one resolver.** The `<robot>.epoch` name, the directory rule, and
  the `CERULION_EPOCH_DIR` env are homed in `cerulion_pairing::verify`
  (`epoch_cache_file_name`, separator-neutered, traversal-proof;
  `resolve_epoch_dir`, env else `epochs/` next to the desk key), because the
  WRITER (`cerulion_cli_engine::account_cmd`, iroh-free) and BOTH readers
  (connect, netd's WAN registry) must agree byte-for-byte; a divergence is
  invisible at both ends and revocations silently stop travelling. Each party's
  resolution is pinned against the same literal path by its own test
  (`both_desk_paths_resolve_one_cache_path` in `wireclient::epoch`,
  `netd_from_env_resolves_the_shared_epoch_cache_path` in `netd::wan`, and the
  writer twin in `account_cmd`).
- **Refusal classes stay distinct** because they demand different operator
  actions: no-sink / predates-the-verb / envelope-version skew (shared needle
  constants in `cerulion_pairing::verify`; the emitting and classifying sides
  import ONE string) vs "the robot verified and rejected THIS epoch".
- **The desk is a courier, not a trust anchor**: no desk-side signature check;
  the robot re-verifies the intermediate against its own roots; a stale push is
  a reported no-op. A revoked desk pushing its own revocation is correct; the
  sweep evicts its streams afterwards (`epoch_push_e2e_test.rs`).
- **The offline gap is real**: a robot nobody dials keeps honoring a grant whose
  revocation it has never seen; the offline defenses are grant expiry + the
  anti-rollback floor (`docs/revocation.md`).

## 10. Secrets and on-disk state

- **Every `~/.cerulion` secret write goes through
  `cerulion_cli_engine::auth::atomic_write_secret`**: 0700 parent, per-process
  temp sibling, symlink-proof `create_new` at 0600, fsync, atomic rename. Its
  callers include the auth store, `robots.toml` (`pair_cmd`), and `peers.json`
  (`peer_cache`). The machine key has its own writer,
  `cerulion_pairing::identity` (0700 directories; the key goes to a synced 0600
  dot-named sibling hard-linked into place, so it never overwrites and is never
  seen partly written),
  which `cerulion login`, `cerulion-connectd pair` and `wireclient`'s
  `--key-file` creation all call.
- Every per-machine config file under `~/.cerulion` resolves from one rule,
  `cerulion_pairing::identity::config_dir` (`CERULION_HOME` verbatim when set
  and non-empty). The crates without `cerulion_pairing` (`cerulion_core` run
  directories, `cerulion_discovery` peers, `cerulion_telemetry` consent) mirror
  it; `crates/cerulion_cli/tests/config_home_test.rs` pins each path to it and
  refuses shipped code with a `".cerulion"` literal within six lines after a
  `home_dir` or `"HOME"` mention, outside its allowed list. Daemon
  sockets (`cerulion_hygiene`) and the downloaded video decoder keep their own
  location rules.
- `cerulion connect` and `pair_cmd` hand `cerulion-connectd` the key's
  legacy name `desk.key`: `connect` resolves `epochs/` next to its key file,
  and `cerulion-connectd pair` recognizes the path as the machine key and loads
  it through the identity layout.
- Desk inventory (`~/.cerulion/`): `identity/device.key` (the 32-byte machine
  seed; `desk.key` is a hard link to it or an equal copy, adopted byte for byte when it is the
  only one present, and two different keys are refused), `identity/<team>/high_water`
  (MAC'd clock, bundle generation and status sequence floors),
  `identity/active_team` (the team sessions open with), `identity/<team>/leaf.der`
  and `identity/<team>/bundle` (the team leaf and the postcard trust bundle,
  accepted by `IdentityLayout::load_team_credentials` only against a trusted
  root set; `team_credentials_test.rs`), `dev-issuer/state` (feature
  `dev-issuer` only: the development issuer's root, intermediate and team CA
  keys), `device.cert`
  (login-issued), `robots.toml` (name → eid pins), `epochs/<robot>.epoch`,
  `grants/`, `auth.json`, `peers.json` (LAN discovery; see
  `network-daemons.md`).
- Robot inventory (state root, `remoted/` subdir; `remoted::config`):
  `device_key`, `trust_store` + `trust_store.mac_key` (the daemon's `KeyFiles`
  provider loads both keys from these files, so robot setup must provision
  them, mode 0600; there is no secure-storage backend. On load the daemon
  clears the group and other permission bits of the device key, the MAC key,
  the trust store and the device index, without following a symbolic link at
  the path, and saves create the trust store and the device index 0600; a
  library host supplies both keys through its own `KeyProvider` and writes no
  key file),
  `device_index.json`, `receipts.log`, `logs/` (the `log-tail` confinement
  root), `rcp.token` (the local `cerulion-rcpd` bearer token, bare or one
  `CERULION_RCP_TOKEN=<token>` line; `cerulion-remoted` creates it at startup
  when absent, one `CERULION_RCP_TOKEN=<64 hex>` line, mode 0600, owned by
  the user it runs as; one that is not a regular file or gives its group or
  others any permission, execute included, is logged at error, and the rcp
  plane answers every request with an error until it is a regular file only
  its owner has any permission on), `beacon_facts.json` (PUBLIC endpoint facts: eid, bound UDP port,
  claimable, for the gateway's mDNS TXT enrichment; no MAC because no secrets;
  `beacon_facts_test.rs`).

  An empty trust-store MAC key file is refused while a trust store or device
  index exists (`RemotePlane::bind`), and the error names the file. A
  `KeyProvider` with no file gets the same refusal without a path. Recovery:
  put this device's own key back into `<state_root>/remoted/trust_store.mac_key`
  with mode 0600. If the key is lost, move `<state_root>/remoted/` away and let
  the registration bootstrap provision the robot again. That discards the trust
  store, so desks pair again. Keep `<state_root>/robot-registration.json` only
  while the device certificate inside it is still valid, which is 7 days from
  issue (the account service's default, `CaConfig::device_cert_ttl_ns` in
  `crates/cerulion_accountd/src/ca.rs`): provisioning verifies that
  certificate again, and an expired one is refused. After that, remove
  `robot-registration.json` as well, so the bootstrap runs its
  first-registration path again; that path needs a current `cerulion login`
  on the robot. Under netd's robot supervisor, a bootstrap
  that gave up needs a `cerulion-netd` restart. A library host restores or
  streams its keys again through its own `KeyProvider`.

## 11. Test map

Every crate in this family runs PARALLEL in CI, with none ignored by default
(on a macOS desk at the 256-file soft limit, run `wire_plane_test` with
`-- --test-threads=1`). The two `cerulion_remoted` files that call the
default-namespace `TransportManager::get_or_init()` directly, `act_verbs_test`
and `rcp_forward_e2e_test`, mark every test `#[serial]`. Tests in other files
that drive `topic-publish` or `engage-estop` on a daemon host (`loopback_test`,
`ops_loopback_test`, `estop_starvation_test`) reach that manager through the
verbs without `#[serial]`. Isolation otherwise comes from per-test SHM roots
(`TransportManager::init_for_test`), per-test sockets/tempdirs and loopback iroh,
direct or through an in-process loopback relay, every await bounded. Nothing
dials an external relay, a real robot, or multicast. Every crate here is named in a CI test step on BOTH
platforms: most in the crate-tests job; `cerulion_connectd` and
`cerulion_wireclient` in the main test jobs.

| Test file | What it pins (1 line) |
|---|---|
| `crates/cerulion_remoted/tests/act_verbs_daemon_test.rs` | Every act verb is registered in the production `serve_endpoint` daemon: each answers `{}` with its own usage error over a real ops session |
| `crates/cerulion_remoted/tests/act_verbs_cloud_test.rs` | The act verbs under the allow-from-cloud switch: a cloud device is refused publish and snapshot only while the switch is off |
| `crates/cerulion_remoted/tests/act_verbs_test.rs` | The act verbs over real iceoryx2 (the process transport, a distinct topic or service per test): publish reaches a subscriber, echo-once and snapshot read a live publisher, service-call answered by core's `ServiceServer`, an e-stop engaged before the send sends no request and no frame, live mirror refusal, reserved-topic usage errors, contextual producer exclusions and local oracle delivery |
| `crates/cerulion_pairing/tests/format_vectors_test.rs` | Byte-exact canonical signing vectors from independent hand-serializers; strict epoch-envelope decode |
| `crates/cerulion_pairing/tests/chain_verify_test.rs` | Chain positive path + full negative matrix (M-of-N, peer binding, expiry, delegation cap, epoch, rollback) |
| `crates/cerulion_pairing/tests/store_test.rs` | MAC tamper-evidence, atomic save/load, anti-rollback persistence, claim/reset lifecycle, epoch monotonicity |
| `crates/cerulion_pairing/tests/owner_grant_verify_test.rs` | Offline owner-grant verification, one-flip negative matrix |
| `crates/cerulion_pairing/tests/pake_test.rs` | CPace agree-iff-code-and-identities-match, attempts/TTL/burn policy |
| `crates/cerulion_pairing/tests/client_test.rs` | Device-identity seam + the two end-to-end pairing flows across all modules |
| `crates/cerulion_pairing/tests/team_formats_test.rs` | Team X.509 leaf and CA profile read back by x509-parser and accepted by rustls-webpki, rcgen-built refusals, bundle and status signing bytes, anti-rollback order |
| `crates/cerulion_link/tests/loopback_test.rs` | Dial-by-key + mutual key auth against independently-derived oracle keys; framing round-trips |
| `crates/cerulion_link/tests/uni_and_ops_stream_test.rs` | Uni-stream helpers + the synchronous `QuicOpsStream` adapter, literal byte oracles |
| `crates/cerulion_remoted/tests/authorizer_matrix_test.rs` | Every cell of the sacred verb table, real-crypto access rows |
| `crates/cerulion_remoted/tests/loopback_test.rs` | Accept-gate routing over real endpoints (wire refuses unpaired; ops bootstraps) |
| `crates/cerulion_remoted/tests/ops_loopback_test.rs` | The real ops plane over iroh: verbs, receipts, pair-then-allowed; a code-pairing try a desk leaves pending is counted failed once its connection is gone |
| `crates/cerulion_remoted/tests/serving_fence_admission_test.rs` | Through `serve_endpoint`: a bootstrap-only ops session and a refused rcp connection leave `serving.hold` free, a paired ops session holds it until it closes and so does a paired rcp connection with no stream open; while the idle stop is committed a paired session is closed at admission, a bootstrap-only one is still served, and a key claimed on a bootstrap session has its next call refused, then served once the stop is withdrawn |
| `crates/cerulion_remoted/tests/wire_plane_test.rs` | Wire plane e2e: catalog/demand/schema/status, byte-exact frames, demand gate |
| `crates/cerulion_remoted/tests/estop_starvation_test.rs` | E-stop bounded latency under hostile concurrent sessions, and on a fresh session while a `code-pair-start` is paused in its CPace step; the 120 s doc pin |
| `crates/cerulion_remoted/tests/offline_connect_test.rs` | Established pairing connects with issuer down + relay unreachable |
| `crates/cerulion_remoted/tests/epoch_push_e2e_test.rs` | Desk-push epoch through production plane/sink/gate/sweep |
| `crates/cerulion_remoted/tests/daemon_epoch_sink_test.rs` / `daemon_demand_gate_test.rs` | Inert-wiring pins: the sink/authorizer wired by `serve_endpoint` itself |
| `crates/cerulion_remoted/tests/multi_device_revoke_e2e_test.rs` | Device-granular revocation: desk A evicted mid-stream, same-account desk B unaffected |
| `crates/cerulion_remoted/tests/device_index_test.rs` | Side-map MAC integrity: no forged key→owner binding |
| `crates/cerulion_remoted/tests/beacon_facts_test.rs` | Beacon-facts atomic write/round-trip/claimable flip |
| `crates/cerulion_remoted/tests/daemon_wiring_test.rs` | Kill-switch + loud provisioning-gap errors before any endpoint binds |
| `crates/cerulion_remoted/tests/daemon_stop_signal_test.rs` | The real binary: SIGTERM and Ctrl-C stop it gracefully with exit 0 and the teardown line; a test-seam stage participant that never finishes is ended by `CERULION_REMOTED_HARD_EXIT_MS` with status 4, also when the endpoint's own close (a test seam) started the stop with no signal; a second signal during the stop exits 4 long before the deadline; after the endpoint's own close, the first stop signal exits 4 as a second one does; a participant that fails exits 3 after the teardown ran; the stage runs before the rcpd e-stop sender is stopped; with stderr full the deadline still exits 4, a graceful stop exits 0, and a graceful stop waits for its queued lines until the pipe drains |
| `crates/cerulion_remoted/tests/estop_full_stderr_test.rs` | The real binary with its stderr pipe full and a log queue of one line (test seam): an rcp `estop` and an ops `engage-estop` each reach a stand-in rcpd within 5 s, with the desk's caller; `status` counts two or more dropped lines while the pipe is full; once it drains, the warn drop line and the info caught-up line are written, and every line `status` then counts is named by them or by a later drop or caught-up line |
| `crates/cerulion_remoted/tests/shutdown_stage_test.rs` | `serve_planes` drops its shutdown future before the stage, on a stop request (endpoint open during the stage) and when the endpoint closes on its own (endpoint closed), and before the line that says the loop stopped; `run` under the kill switch drops it before its warn line; the drop before the endpoint-closed stop line and before the kill-switch warn is pinned against a log writer that blocks |
| `crates/cerulion_remoted/tests/remote_plane_library_test.rs` | The library entry: temp state root, in-memory keys, catalog through the injected control socket or an in-process schema source, a fresh state root refused before any write, daemon-only verbs refused, key provider error arms |
| `crates/cerulion_connectd/tests/act_network_switch_test.rs` | The real `cerulion-connectd act` under `CERULION_NETWORK=off` or `--network off` exits 1 with the refusal and sends no datagram to the robot's address; with no switch the same call dials it |
| `crates/cerulion_connectd/tests/rcp_network_switch_test.rs` | The real `cerulion-connectd rcp` under `CERULION_NETWORK=off` or `--network off` exits 1 with the refusal and sends no datagram to the robot's address; with no switch the same command dials it |
| `crates/cerulion_connectd/tests/protocol_parity_test.rs` | Desk protocol mirror == robot serve types, byte-for-byte |
| `crates/cerulion_connectd/tests/reinject_e2e_test.rs` | Full connect session: dial → demand → reader → desk-SHM re-inject, hand-oracle frames |
| `crates/cerulion_connectd/tests/pair_e2e_test.rs` | Desk CPace ceremony against a real remoted ops plane; trust verified from disk; each refusal text of the robot's code-pairing store maps to its desk exit class (every try failed and a wrong code are a code mismatch; a used code, a throttled try, every try taken by a desk still pairing, an expired window, no armed code and no try in progress are refusals) |
| `cerulion_wireclient` in-module tests | Dial-config parsers, epoch push decisions + cache-path pins, reply classification |
| `crates/cerud/tests/*` | Protocol/authz/receipt/lease/deploy/verbs oracles + the constants doc pins |
| `crates/cerulion_ops_proto/tests/wire_golden_test.rs` | Every `cerulion/ops/1` message type encodes to, and decodes from, a golden frame built from JSON text |
| `crates/cerulion_accountd/tests/*` | Account-plane acceptance over a real server, CA↔shipped-verifier byte-compat, device-code state, 5xx error-leak ban |
| `crates/cerulion_netd/tests/wan_plane_iroh_test.rs` | The WAN plane against a real in-process robot (see `network-daemons.md` §12) |
| `crates/cerulion_netd/tests/wan_plane_iroh_test/serving_plane.rs` | Which plane carried a mirror, asserted against the delivery it describes |
| `crates/cerulion_remoted/tests/code_pair_account_test.rs` | Code pairing binds the self-account only; a named owner account is refused; a wall-clock step neither stretches nor cuts the code window |
| `cerulion_pairing::pair_code`, `cerulion_remoted::{pair_code_request, owner_only, pairing_verbs, wire}`, `cerulion_wireclient::protocol` in-module tests | Request and result line oracles for each status, each refusal reason and its user text, a `refused` result with and without a nonce (`-` only for `unreadable` and `malformed`), and every malformed line, the 8-digit code and the label token; the `settle` and `admit_try` oracle tables; one budget shared by every desk, pending tries held until their sessions end and answered as in progress, a new try counting the pending one, the rate gate across a re-arm and a reading before the last try, a start that waited for the lock judged at the time it got it, the window and its quiet expiry, a save error counted once with the same desk pairing next, a save that panics counted as a failed save, no outcome while a row is saved and the state lock free during the save, a second right code waiting for the first save and refused as used, a re-arm dropping the replaced code's tries, waiting for its row's save and answering the replaced code's last state with its outcome latched, each refusal of a finish with no try pending, single use, the hold's hard end, the pure `written_end_releases_hold` table (paired and expired release at once, exhausted serves to the hard end), a start's CPace step run with the state lock released (a session opens and closes meanwhile), a code that ends during the step refusing the try (expired counted, paired uncounted), arms and disarms waiting for a start in progress and a disarm waiting for a row's save, two runs of one script giving identical snapshots; the owner-only verdict; a request that is not owner-only or well formed arms nothing, is removed and is answered `refused` and counted, and replaces the code an earlier request armed, its refusal logged once, and an unreadable, a malformed and an unconfirmed request answered identically on two runs; a directory that cannot be listed or a leftover that cannot be removed holds every request; a refused serving hold leaves the request; the result written on each change, including an end first found by a refused start or a session end and a wrong-tag finish, and the serving fence held from the arm until a `paired` or `expired` end is written and an `exhausted` one until its hard end, held through a failed write of each end; a failing result write counted and retried; a newer request replacing the code and its nonce, and one whose result cannot be written leaving no code and answered `unconfirmed` once a write works, a result file that cannot be removed logged at error; a refused or newer request first logging and writing each end the replaced code reached that the result does not state yet (a pairing, used-up tries, a quiet window), also while its row is saved, never an end already written or one of a code another armed, identically on two runs, and a last result that cannot be written counted, logged and its result file removed; the pure `unwritten_end` table; a stopped arm step arms and writes nothing, withdraws the armed code as `refused` `stopping`, writes a pairing or a refusal the result does not state yet, leaves a written end, removes the result when its last write fails, and gives identical results on two runs; the shared admission's refusal and recovery lines; the `status` reply's `pair_code` and `pair_code_refused` fields on the robot and on the desk |

Running: `cargo test -p <crate>` for each of `cerulion_pairing`,
`cerulion_link`, `cerulion_wireclient`, `cerulion_connectd`, `cerulion_remoted`,
`cerud`, `cerulion_ops_proto`, `cerulion_accountd`, `cerulion-wire`; no `--test-threads=1` in CI for
this family. `cerulion_link` / `cerulion_remoted` cold-compile the ~390-crate
iroh tree: long first builds are normal. The netd WAN plane tests ride
`cargo test -p cerulion_netd` (`wan` is default-on; keep
`cargo check -p cerulion_netd --no-default-features` green).
