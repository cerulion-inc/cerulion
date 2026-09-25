# cerulion_netd - agent notes

The one-per-computer network gateway daemon: owns the machine's single zenoh session
and serves refcounted remote-topic demands over a UDS NDJSON control seam. lib + bin
split: tests drive an in-process daemon over a temp socket with an injected spy
mirror plane - green there proves the wiring, not the live network planes.

## Invariants

- One daemon per machine: an `flock`-held pidfile (`cerulion_hygiene::NETD`); the
  kernel drops the lock on death. Never add a second lock scheme.
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
- `CERULION_NETD_LISTEN` is the ONLY gate for the mDNS beacon and for standing mode
  (gateway boots at start, idle self-exit off). No locator means a reported refusal,
  never a default port; `CERULION_NETD_NETWORK=off` wins over both.
- LISTEN startup needs a persisted prior login: expired records still serve LAN
  offline, never-logged-in refuses with `serving_login::REFUSAL`. Network-off, desk
  and injected daemons are exempt. `register_egress` rechecks before mutating.
- A WAN LISTEN machine owns ONE remoted child via `robot_supervisor`: registration
  is deadline-bound and tried ONCE, shutdown reaps before releasing the singleton,
  and the machine REFUSES outgoing WAN demands rather than building a second
  endpoint or falling back to LAN. Beacon facts share `discovery::robot_state`.
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

- `cargo test -p cerulion_netd` - parallel-safe throughout; keep
  `--no-default-features` green. Oracles are COUNTS, never wall clocks.
- `wan_plane_iroh_test owner_pair::controller` wants `-- --test-threads=1`.
- `#[traced_test]` capture is thread-local: a plane's tokio worker is invisible to
  it. Test-only beacon suppression stays out of `src/**` (a walk enforces it).

Deep reference: docs/internals/network-daemons.md, docs/internals/remote-access.md.
