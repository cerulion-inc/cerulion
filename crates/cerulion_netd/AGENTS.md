# cerulion_netd - agent notes

The one-per-computer network gateway daemon: owns the machine's single zenoh session
and serves refcounted remote-topic demands over a UDS NDJSON control seam. lib + bin
split: tests drive an in-process daemon over a temp socket with an injected spy
mirror plane - green there proves the wiring, not the live network planes.

## Invariants

- One daemon per machine: an `flock`-held pidfile (`hygiene.rs` = `cerulion_hygiene::NETD`); the kernel drops
  the lock on death, so the singleton is stale-proof. Never add a second lock scheme.
- Connection close IS the release - every demand a UDS connection holds is released
  when it closes (crash-safe refcounts). The LAST release retires the mirror and
  frees the single-writer SHM slot; a re-demand recreates it.
- One mirror per canonical topic across BOTH planes (zenoh LAN, iroh WAN): frames
  cross the network once no matter how many consumers demand the topic.
- `wan` is DEFAULT-ON and netd is deliberately OUT of workspace `default-members` -
  a plain `cargo build` must stay iroh-free. Lean consumers (CLI, vizd) depend with
  `default-features = false`. Changing either side changes the whole workspace build.
- Never answer a cold-start query with a confident empty: report
  `DiscoveryState::NotConverged` until discovery settles - an empty gather is not
  absence. The pure oracle is `query::classify_gather`.
- An additive protocol field whose serde default is a POSITIVE claim needs a
  `PROTOCOL_VERSION` bump plus a client trust floor (`DISCOVERY_MIN_DAEMON_VERSION`) -
  a `<=` compile-time assert against `PROTOCOL_VERSION`, never an equality pin.
- Convergence-wait loops call the non-reconnecting `_once` client verbs - a
  reconnecting verb re-runs spawn-wait and falsifies the measured wall.
- A cancelled (Ctrl-C) wait is an interruption, never an absence claim.
- The control loop waits on socket-readable OR push-slot-filled - a socket-only
  wait stalls catalog-change pushes forever on an idle client.
- The mDNS beacon gate is the LISTEN endpoint (`CERULION_NETD_LISTEN`), never robot
  identity; no listen locator ⇒ a reported refusal to advertise, never a default port.
  LISTEN also makes netd a STANDING daemon: the embedded egress gateway boots at daemon
  start (empty announce; runtime registrations grow it) and idle self-exit is disabled -
  it exits on SIGINT/SIGTERM only. `CERULION_NETD_NETWORK=off` wins (no session, no boot).
- `release_max_level_info`: `debug!` does not exist in a release build at any
  `RUST_LOG` - operator-facing outcomes log at `info!` or louder.
- Sibling-binary lookup goes through `resolve_netd_bin_from`'s ladder - macOS
  `current_exe()` is NOT canonicalized (a symlinked install reports the symlink).
- netd links the READER-only `cerulion_discovery`: never add a peers.json write
  path here - a cache write needs gather-confirmed liveness evidence netd lacks.

- A WAN-enabled LISTEN machine owns one remoted child through `robot_supervisor`.
  Registration/writer work is asynchronous, deadline-bound and attempted once;
  shutdown kills and reaps the owned child before releasing the netd singleton.
  Serving machines refuse outgoing WAN demands without constructing another iroh
  runtime/endpoint or falling back to LAN.
- Automatic state and beacon facts share `cerulion_discovery::robot_state`.
  Initial LAN advertisement does not await registration. Its existing guard may
  refresh matching endpoint facts for a bounded startup window; a read failure is
  returned once to stop the refresh loop, preserving the LAN advertisement.

- LISTEN startup requires a persisted prior login before peer probing or listeners.
  Expired login records permit offline LAN serving; never-logged-in state refuses
  with `serving_login::REFUSAL`. Network-off and ordinary desk daemons are exempt.
  Production raw `register_egress` dispatch rechecks that same login before any
  registry mutation or lazy gateway boot; network-off remains exempt. Injected
  library daemons have no ambient login prerequisite unless explicitly configured.
  The shared reader is bounded, local-only and never refreshes or creates a key.

- Account identity snapshots hold the shared auth-store lock across auth/key/cert
  reads. Busy writers refuse without waiting. No key creation or token refresh;
  corrupt present proof refuses, while valid legacy logins may lack a chain.
  Expiry gates new robot enrollment, never local identity or durable bindings.
  The stamp includes public certificate bytes, not bearer refreshes. Same-account
  proof refresh preserves readers; account/key changes require controller fencing.

- Account and local schema control use one absolute deadline across connect, Hello,
  every partial write/read, and parsing; progress cannot renew it. Bounded existing-only
  clients never spawn/retry. Malformed, partial or mismatched replies poison the
  connection; correlated daemon refusals remain reusable (`client::bounded`).

## Testing

- `cargo test -p cerulion_netd` - every test file is parallel-safe (unique temp
  sockets, per-test SHM roots, scouting-off zenoh); no `--test-threads=1` needed.
- `wan` default-on means `wan_plane_iroh_test.rs` compiles and runs by default;
  `cargo check -p cerulion_netd --no-default-features` (lean build) must stay green.
- Oracles are served-query / harvest COUNTS, never wall clocks - CI load inverts
  any wall assertion tight enough to discriminate.

## Gotchas

- `#[traced_test]` capture is thread-local: logs emitted on a plane's own tokio
  worker are invisible to it - code-verify loudness there instead of log-asserting.
- Test-only beacon suppression must stay out of shipped code -
  `mdns_suppression_confinement_test.rs` walks `src/**/*.rs` and fails you if not.

Deep reference: docs/internals/network-daemons.md - read before touching the mirror /
egress / query planes, the control protocol, or any crate boundary in the network tree.


## Account WAN controller

Production uses `account_controller::AccountWanController` for both configured
manual WAN routes and installed account membership. Installation compares the
public snapshot with `identity_snapshot` under the existing login store lock;
it creates no endpoint. Network-off and serving posture refuse outgoing WAN
before plane construction. Account routes use the reserved `account:` identity
encoding; validate the raw robot string before `TopicKey` trims ordinary names.

Route pins survive live, lingering and tearing states. `mirror_retired` runs
only after successful daemon registry retirement. New demands always call
`prepare_demand`, including an already mirrored topic: stale identity and dead
reader handles must not acknowledge a phantom mirror. Controller state is taken
with `try_lock` under registry bookkeeping; LAN routing never waits behind WAN
metadata. The production identity watcher polls every 250 ms and permits five
continuous seconds of an active login writer before closing account readers.
Token/certificate expiry does not change identity or revoke durable bindings.

Run `account_controller_admission_test` and the `account_controller::tests`
unit filter. Run `wan_plane_iroh_test owner_pair::controller` individually with
`-- --test-threads=1`: it uses real loopback robot serving, durable owner issuance,
actual UDS control, and handwritten delivered-frame oracles. The natural-death
case closes a real authenticated connection while preserving its endpoint, key
and sockets; no membership refresh hides the AlreadyMirrored recovery guard. It proves recovery on a new
demand, not automatic reconnection of an existing visualization tap.
