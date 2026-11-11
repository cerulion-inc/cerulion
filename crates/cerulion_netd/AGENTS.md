# cerulion_netd - agent notes

One-per-machine gateway: one zenoh session, refcounted UDS NDJSON demands.
Injected spy tests prove daemon wiring; real network tests prove delivery.

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
- Serving accepts a saved login or authenticated durable local robot pairing via
  `serving_login::require_serving_authority`; expiry preserves offline serving.
  LISTEN startup retains prior-login policy; off/injected daemons are exempt.
- First nonempty egress registration starts ONE remoted child, no LISTEN needed.
  Consumer-only desks stay outbound. Last release waits for active/owed work,
  fences new admission with `serving_hold` and reaps via SIGTERM plus bounded grace.
- `begin_serving` retires the outgoing endpoint before spawning remoted;
  `end_serving` runs after reap. Both mark assignments pending under controller
  state lock. Serving mirrors reuse outbound.sock; never bind the key twice.
  Watchdogs budget child grace plus epoch-worker cleanup before ownership returns.
- ONE `account_controller::AccountWanController` serves manual and account routes:
  install creates no endpoint, controller state is `try_lock` under registry
  bookkeeping so LAN never waits on WAN metadata; pins survive tearing. EVERY
  demand calls `prepare_demand`: stale identity or dead reader cannot acknowledge
  a phantom mirror. Validate `account:` before `TopicKey` trims ordinary names.
- Serving controllers use remoted's bounded same-user `outbound.sock`, endpoint
  and runtime. Key and SHM namespace must match before dialing. Close retires
  mirrors; account fencing preserves the owned endpoint. Never bind its key twice.
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
