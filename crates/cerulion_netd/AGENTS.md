# cerulion_netd - agent notes

The one-per-computer network gateway daemon: owns the machine's single zenoh session
and serves refcounted remote-topic demands over a UDS NDJSON control seam. lib + bin
split: tests drive an in-process daemon over a temp socket with an injected spy
mirror plane - green there proves the wiring, not the live network planes.

## Invariants
- One daemon per machine: an `flock`-held pidfile (`cerulion_hygiene::NETD`); the
  kernel drops the lock on death. Never add a second lock scheme. Android: abstract
  socket, same-UID peers only.
- Connection close IS the release (crash-safe refcounts). The LAST release retires
  the mirror and frees the single-writer SHM slot; a re-demand recreates it.
- ONE mirror per canonical topic across both planes, so frames cross once however
  many consumers demand. Which plane served it is `MirrorPlane::serving_plane`,
  on each `status` row from the route PINNED at ensure; absent is unknown, not LAN.
- `wan` is DEFAULT-ON and netd is OUT of `default-members` - a plain `cargo build`
  stays iroh-free, lean consumers set `default-features = false`. Either side
  changes the whole workspace build.
- Never answer a cold start with a confident empty: `DiscoveryState::NotConverged`
  until discovery settles (`query::classify_gather`). An additive field whose serde
  default is a POSITIVE claim needs a `PROTOCOL_VERSION` bump plus a client floor,
  `<=`-asserted, never an equality pin.
- Convergence waits call the non-reconnecting `_once` verbs (a reconnect re-runs
  spawn-wait and falsifies the wall). A cancelled wait is never an absence claim.
- The control loop waits on socket-readable OR push-slot-filled; socket-only stalls
  catalog pushes on an idle client.
- A nonzero non-loopback TCP `CERULION_NETD_LISTEN` port is the ONLY gate for the mDNS
  beacon and standing mode (gateway boots at start, idle self-exit off). None means a
  reported refusal, never a default port; `CERULION_NETD_NETWORK=off` wins over both.
- Network egress accepts a saved login or authenticated persistent robot pairing;
  login expiry preserves offline serving. Never-logged-in/unpaired refuses with
  `serving_login::REFUSAL`. `register_egress` rechecks before mutating. Explicit
  LISTEN startup retains its prior-login prerequisite; network-off and injected
  daemons are exempt.
- The first accepted nonempty WAN egress registration owns ONE remoted child via
  `SupervisedEgressPlane` and `robot_supervisor`, independent of LISTEN. Query-only
  and local-only daemons start none. Last release/disconnect reaps only after
  native serving holds, owed e-stop delivery and its latch clear; explicit LISTEN
  holds the standing child until shutdown. The idle watcher retains the singleton
  while `holds_serving_endpoint` is true, including last-release teardown.
- Native idle admission: remoted holds `serving.hold` shared for admitted ops/RCP,
  latch/owed delivery sources and shell/exec guards. Netd holds it exclusively
  through idle SIGTERM and reap. Never unlink it. A held source retains serving;
  source state transitions require a work guard. Missing daemon sources fail closed.
- Remoted has its own process group; stop sends SIGTERM, waits its hard-exit
  deadline plus 500 ms, then escalates and reaps. Budget this before epoch-fetch
  grace and never return endpoint ownership before both have settled.
- Endpoint transfer calls `MirrorPlane::begin_serving` before spawn and
  `end_serving` after reap. The account controller closes its outgoing endpoint,
  marks assignments pending retirement and fences new outgoing construction under
  its state lock. Until shared outbound routing is available, serving machines
  refuse WAN demands without fallback. Beacon facts share `discovery::robot_state`.
- ONE `account_controller::AccountWanController` serves manual and account routes:
  install creates no endpoint, controller state is `try_lock` under registry
  bookkeeping so LAN never waits on WAN metadata, and pins survive tearing. EVERY
  demand calls `prepare_demand`, even an already-mirrored topic: a stale identity
  or dead reader must not acknowledge a phantom mirror. Validate the reserved
  `account:` route before `TopicKey` trims ordinary names.
- Identity snapshots carry public bytes only, hold the auth-store lock across every
  read, and refuse rather than wait on a busy writer. Expiry gates new enrollment,
  never durable bindings; an account or key change fences the controller.
- `release_max_level_info`: `debug!` is absent from a release build, so operator
  outcomes log at `info!` or louder. Sibling lookup goes through
  `resolve_netd_bin_from` (macOS `current_exe()` is not canonicalized), and netd
  links READER-only `cerulion_discovery`: no peers.json write path here.

## Testing
- `cargo test -p cerulion_netd` is parallel-safe; keep `--no-default-features` green.
  Oracles are COUNTS, never wall clocks.
- `wan_plane_iroh_test owner_pair::controller` wants `-- --test-threads=1`.
- `#[traced_test]` capture is thread-local: a plane's tokio worker is invisible to
  it. Test-only beacon suppression stays out of `src/**` (a walk enforces it).

Deep reference: docs/internals/network-daemons.md, docs/internals/remote-access.md.
