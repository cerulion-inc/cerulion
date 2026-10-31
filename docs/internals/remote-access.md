# Remote access & pairing: contributor dossier

Scope: the remote-plane family: `cerulion_remoted` (the robot daemon), `cerud`
(the robot ops-verb service), `cerulion_pairing` (identity + trust), `cerulion_link`
(iroh dial-by-key QUIC), `cerulion_wireclient` (desk client substrate),
`cerulion_connectd` (the `cerulion connect` / `cerulion pair` worker),
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
seed is never the public half. iroh multiplexes both ALPNs on that endpoint
(`cerulion_link::alpn`: `cerulion/wire/2` = raw topic frames, `cerulion/ops/1` =
ops verbs; the trailing `/1` is each plane's protocol-version boundary). Exactly
one process owns it: `cerulion-remoted`, dispatching by negotiated ALPN at accept
(`daemon::handle_accepted`; pinned over real loopback endpoints by
`crates/cerulion_remoted/tests/loopback_test.rs`).

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
  `cerulion pair` (spawns `cerulion-connectd pair`), and `cerulion_netd`'s
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
| `cerulion_remoted` | robot | AGPL-3.0-only | **no** (links link + core) | THE robot daemon: endpoint, accept gate, wire plane, pairing verbs, live trust |
| `cerulion_wireclient` | desk | AGPL-3.0-only | **no** (links link + core) | shared `cerulion/wire/2` client substrate (protocol mirror, dial-config parsers, reader loop, epoch push) |
| `cerulion_connectd` | desk | AGPL-3.0-only | **no** | the `cerulion connect`/`pair` worker binary (session driver + CPace initiator) |
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
  established pairings never rot offline; short-lived certs gate NEW pairings
  only. Revocation propagates via the monotonic signed epoch (§9).
- **The trust store is tamper-evident and anti-rollback** (`verify/store.rs`):
  HMAC-SHA256 over a versioned envelope, constant-time verify on load, atomic
  temp+rename writes; the MAC key is caller-supplied (firmware secure storage;
  the crate holds no key-management policy). `high_water_ns` is the max
  validated time ever observed; a verification whose `now_ns` is behind it is
  refused, so a clock rollback cannot resurrect an expired cert, and the floor
  survives factory reset. Pinned by `store_test.rs`. A store whose
  `STORE_FORMAT_VERSION` does not match is REFUSED, never migrated (the
  documented bump policy in `store.rs`).
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
  (`cerulion-pairing:team-trust:v1`), `TeamStatusList` and `StatusFreshness`
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
paired, and each GET's selector names one ingress topic it demands. Writes
from a paired desk travel over the remote plane, whose re-inject seam does not
consult the LAN posture. Oracle tests:
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
its pairing key, is not supported yet. Per-peer, per-topic and per-direction
(observe or command) access with revocation is what the paired remote plane
provides, binding each session to its pairing key over an authenticated
transport.

- **The verb → capability table is the sacred security surface** (module docs of
  `authorizer.rs`): bootstrap verbs (`claim`/`pair`/`present-grant`/
  `code-pair-*`) are access-list-exempt and SELF-GATE in their handlers
  (chassis secret / cert chain / owner grant / an unforgeable `CpaceConfirmed`
  witness; `pairing_verbs.rs`); `engage-estop` is the permission floor (any
  paired account, regardless of scope); observation verbs need `CAP_OBSERVE`
  AND role ≥ VIEWER (non-vacuous: `Role` is an open `u16`, so a role less
  privileged than VIEWER holding `CAP_OBSERVE` is representable, and denied);
  lifecycle mutation needs role ≥ OPERATOR; `teleop` needs `CAP_TELEOP` AND
  role ≥ OPERATOR; an UNCLASSIFIED verb is DENIED, never fail-open. Lower
  `Role` value = higher privilege.
- **The pairing subject is always the authenticated device key**, never read
  from client args (`pairing_verbs.rs` module docs; `ops_loopback_test.rs`).
  Validity, anti-rollback and the CPace TTL are evaluated against the robot's
  trusted `RemotedClock`, never a client-supplied time.
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
  production build (`estop_flashback_test.rs`).
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

## 6. The wire plane (robot side)

A `cerulion/wire/2` connection (admitted only for a paired `CAP_OBSERVE`
account) carries ONE bidi control stream (length-prefixed JSON:
`catalog`/`demand`/`undemand`/`schema`/`status`/`sync_epoch`, the same
`cerulion_q` vocabulary the LAN uses, imported not mirrored) and ONE uni data
stream per demanded topic (a JSON `StreamPreamble`, then raw wire frames
VERBATIM; per-topic QUIC flow control means a slow topic never stalls a fast
one, and the demand lifecycle IS the stream lifecycle). End-to-end over real
iroh + real iceoryx2: `wire_plane_test.rs` (byte-for-byte against hand oracles).

- **Framing is NOT cancel-safe** (`cerulion_link::framing`): `read_frame`/
  `write_frame` are two sequential awaits, so a dropped future desyncs the
  stream permanently. Never `select!` on them; one dedicated task per stream
  drives them to completion. Every consumer in the family obeys this
  (wireclient's reader, connectd's worker, netd's iroh plane, remoted's serve
  loop); the pattern is documented at each site. *The rule itself has no single
  structural gate, verified at `crates/cerulion_link/src/framing.rs` module docs +
  each caller.*
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

## 7. The desk half: connect, pair, and the client substrate

- **`cerulion connect <robot>`**: the CLI resolves robot address + demand set
  into a `cerulion-connectd` argv and spawns the sibling binary
  (`connect_cmd.rs`, pure + oracle-tested; `CERULION_CONNECTD_BIN` overrides
  the binary). A positional name resolves via `~/.cerulion/robots.toml`, which
  `cerulion pair` WRITES on success; a paired robot's name just works.
  connectd's `worker::run_connect` dials, fetches the catalog, demands, and
  spawns one reader per accepted topic; the whole loop is pinned over real
  loopback iroh + real per-test SHM roots by `reinject_e2e_test.rs`.
- **`cerulion pair <robot>`**: the desk is the CPace initiator over the robot's
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

The ceremony above (a chassis secret, a pairing code, an operator typing at the
robot) is the path for a machine nobody has an account for. A machine that IS
logged in needs none of it, because `cerulion login` already proved which
account owns it. **This path is proven against the in-tree `cerulion_accountd`
only; the hosted issuer is identity-only today, so nothing here works against
it until that service issues device certificates and registers robots.**

**On the robot, `cerulion login` is the only account step.** The first
network-serving run registers the machine. The gate is the LISTEN locator
(`CERULION_NETD_LISTEN`): it is what makes netd a standing gateway, and only a
standing gateway starts `robot_supervisor`. netd is spawn-once, so the variable
must be in netd's OWN environment before the first command that spawns it; a
netd already running without it must be stopped. A failed registration, or a
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
install with `CERULION_NETD_LISTEN` set runs this supervisor path.

The robot's identity is the login device key, so the endpoint id a desk dials is
the key registration proved possession of. Roots come from the issuer's
well-known endpoint. Registration is idempotent: an already-registered machine
does nothing. If the account service is unreachable at first serve the WAN plane
refuses loudly with the reason, the LAN plane keeps serving, and the supervisor
retries on the backoff above.

**On the desk, there is no pairing step.** `topic list` and `viz --robot` read
the account's robots from the issuer's owner-only list endpoint and mark each
one with a bounded presence probe. The directory fetch inside `topic list` has
its own 2 s deadline (`LISTING_FETCH_BUDGET`, session refresh included); past it,
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
other account still needs the full grant presentation.

**Code pairing cannot name an account.** Possession of a code and of the device
keys proves neither, so `code-pair-finish` binds the guest to the self-account
derived from the authenticated transport key and refuses any other value; a
cloud account requires the full roots-verified chain whose device key equals the
transport peer key. The owner row is never preserved for a guest, because that
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
  which `cerulion login`, `cerulion pair` and `wireclient`'s `--key-file`
  creation all call.
- Every per-machine config file under `~/.cerulion` resolves from one rule,
  `cerulion_pairing::identity::config_dir` (`CERULION_HOME` verbatim when set
  and non-empty). The crates without `cerulion_pairing` (`cerulion_core` run
  directories, `cerulion_discovery` peers, `cerulion_telemetry` consent) mirror
  it; `crates/cerulion_cli/tests/config_home_test.rs` pins each path to it and
  refuses shipped code with a `".cerulion"` literal within six lines after a
  `home_dir` or `"HOME"` mention, outside its allowed list. Daemon
  sockets (`cerulion_hygiene`) and the downloaded video decoder keep their own
  location rules.
- `cerulion connect` and `cerulion pair` hand `cerulion-connectd` the key's
  legacy name `desk.key`: `connect` resolves `epochs/` next to its key file,
  and `pair` recognizes the path as the machine key and loads it through the
  identity layout.
- Desk inventory (`~/.cerulion/`): `identity/device.key` (the 32-byte machine
  seed; `desk.key` is a hard link to it or an equal copy, adopted byte for byte when it is the
  only one present, and two different keys are refused), `identity/<team>/high_water`
  (MAC'd clock, bundle generation and status sequence floors), `device.cert`
  (login-issued), `robots.toml` (name → eid pins), `epochs/<robot>.epoch`,
  `grants/`, `auth.json`, `peers.json` (LAN discovery; see
  `network-daemons.md`).
- Robot inventory (state root, `remoted/` subdir; `remoted::config`):
  `device_key`, `trust_store` + `trust_store.mac_key` (the daemon loads the
  trust-store MAC key from this file, so robot setup must provision it, mode
  0600; there is no secure-storage backend. On load the daemon clears the group
  and other permission bits of the device key, the MAC key, the trust store and
  the device index, without following a symbolic link at the path, and saves
  create the trust store and the device index 0600),
  `device_index.json`, `receipts.log`, `logs/` (the `log-tail` confinement
  root), `beacon_facts.json` (PUBLIC endpoint facts: eid, bound UDP port,
  claimable, for the gateway's mDNS TXT enrichment; no MAC because no secrets;
  `beacon_facts_test.rs`).

## 11. Test map

Every crate in this family is PARALLEL-safe, with no test requiring serial
execution and none ignored by default: per-test SHM roots
(`TransportManager::init_for_test`), per-test sockets/tempdirs, loopback-only
iroh (relay disabled + direct dial, every await bounded). Nothing dials a relay,
a real robot, or multicast. Every crate here is named in a CI test step on BOTH
platforms: most in the crate-tests job; `cerulion_connectd` and
`cerulion_wireclient` in the main test jobs.

| Test file | What it pins (1 line) |
|---|---|
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
| `crates/cerulion_remoted/tests/ops_loopback_test.rs` | The real ops plane over iroh: verbs, receipts, pair-then-allowed |
| `crates/cerulion_remoted/tests/wire_plane_test.rs` | Wire plane e2e: catalog/demand/schema/status, byte-exact frames, demand gate |
| `crates/cerulion_remoted/tests/estop_starvation_test.rs` | E-stop bounded latency under hostile concurrent sessions; the 120 s doc pin |
| `crates/cerulion_remoted/tests/offline_connect_test.rs` | Established pairing connects with issuer down + relay unreachable |
| `crates/cerulion_remoted/tests/epoch_push_e2e_test.rs` | Desk-push epoch through production plane/sink/gate/sweep |
| `crates/cerulion_remoted/tests/daemon_epoch_sink_test.rs` / `daemon_demand_gate_test.rs` | Inert-wiring pins: the sink/authorizer wired by `serve_endpoint` itself |
| `crates/cerulion_remoted/tests/multi_device_revoke_e2e_test.rs` | Device-granular revocation: desk A evicted mid-stream, same-account desk B unaffected |
| `crates/cerulion_remoted/tests/device_index_test.rs` | Side-map MAC integrity: no forged key→owner binding |
| `crates/cerulion_remoted/tests/beacon_facts_test.rs` | Beacon-facts atomic write/round-trip/claimable flip |
| `crates/cerulion_remoted/tests/daemon_wiring_test.rs` | Kill-switch + loud provisioning-gap errors before any endpoint binds |
| `crates/cerulion_connectd/tests/protocol_parity_test.rs` | Desk protocol mirror == robot serve types, byte-for-byte |
| `crates/cerulion_connectd/tests/reinject_e2e_test.rs` | Full connect session: dial → demand → reader → desk-SHM re-inject, hand-oracle frames |
| `crates/cerulion_connectd/tests/pair_e2e_test.rs` | Desk CPace ceremony against a real remoted ops plane; trust verified from disk |
| `cerulion_wireclient` in-module tests | Dial-config parsers, epoch push decisions + cache-path pins, reply classification |
| `crates/cerud/tests/*` | Protocol/authz/receipt/lease/deploy/verbs oracles + the constants doc pins |
| `crates/cerulion_ops_proto/tests/wire_golden_test.rs` | Every `cerulion/ops/1` message type encodes to, and decodes from, a golden frame built from JSON text |
| `crates/cerulion_accountd/tests/*` | Account-plane acceptance over a real server, CA↔shipped-verifier byte-compat, device-code state, 5xx error-leak ban |
| `crates/cerulion_netd/tests/wan_plane_iroh_test.rs` | The WAN plane against a real in-process robot (see `network-daemons.md` §12) |
| `crates/cerulion_netd/tests/wan_plane_iroh_test/serving_plane.rs` | Which plane carried a mirror, asserted against the delivery it describes |
| `crates/cerulion_remoted/tests/code_pair_account_test.rs` | Code pairing binds the self-account only; a named owner account is refused |

Running: `cargo test -p <crate>` for each of `cerulion_pairing`,
`cerulion_link`, `cerulion_wireclient`, `cerulion_connectd`, `cerulion_remoted`,
`cerud`, `cerulion_ops_proto`, `cerulion_accountd`, `cerulion-wire`; no `--test-threads=1` anywhere in
this family. `cerulion_link` / `cerulion_remoted` cold-compile the ~390-crate
iroh tree: long first builds are normal. The netd WAN plane tests ride
`cargo test -p cerulion_netd` (`wan` is default-on; keep
`cargo check -p cerulion_netd --no-default-features` green).
