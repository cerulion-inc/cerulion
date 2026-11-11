# cerulion_netd - agent notes

Machine gateway owns ONE zenoh session, serving refcounted demands
through UDS NDJSON. Library/bin split: injected-plane tests prove control wiring,
not live transport.

- Protocol 10 adds paired catalogs and observer demands. Wire/2 and graph ingress
  stay canonical; WAN observers use private /__cerulion/observers/ channels, shared
  per source and released at last observer. LAN observers share canonical demands.
  Run watches report status startup failures through the shared latch.

## Invariants
- One daemon per machine: an `flock`-held pidfile (`cerulion_hygiene::NETD`); the
  kernel drops the lock on death. Never add a second lock scheme. Android: abstract
  socket, same-UID peers only.
- Connection close IS the release (crash-safe refcounts). The LAST release retires
  the mirror and frees the single-writer SHM slot; a re-demand recreates it.
- ONE mirror per source/mode across planes; shared consumers cross once.
  `MirrorPlane::serving_plane` reports the route pinned at ensure in status;
  absent means unknown, never an inferred LAN route.
- `wan` is DEFAULT-ON and netd is OUT of `default-members` - a plain `cargo build`
  stays iroh-free, lean consumers set `default-features = false`. Either side
  changes the whole workspace build.
- Never answer a cold start with a confident empty: `DiscoveryState::NotConverged`
  until discovery settles (`query::classify_gather`). An additive field whose serde
  default is a POSITIVE claim needs a `PROTOCOL_VERSION` bump plus a client floor,
  `<=`-asserted, never an equality pin.
- Convergence uses non-reconnecting `_once`: reconnect repeats spawn-wait,
  falsifying the bound. Cancellation never claims absence.
- The control loop waits on socket-readable OR push-slot-filled; socket-only stalls
  catalog pushes on an idle client.
- A nonzero non-loopback TCP `CERULION_NETD_LISTEN` port is the ONLY gate for the mDNS
  beacon and standing mode (gateway boots at start, idle self-exit off). None means a
  reported refusal, never a default port; `CERULION_NETD_NETWORK=off` wins over both.
- LISTEN startup needs a persisted prior login: expired records still serve LAN
  offline, never-logged-in refuses with `serving_login::REFUSAL`. Network-off, desk
  and injected daemons are exempt. `register_egress` rechecks before mutating.
- A WAN LISTEN machine owns ONE remoted child via `robot_supervisor`: registration
  is deadline-bound, retried with capped backoff; shutdown reaps before releasing
  the singleton, and the machine REFUSES outgoing WAN demands rather than building a second
  endpoint or falling back to LAN. Beacon facts share `discovery::robot_state`.
- ONE `account_controller::AccountWanController` serves manual/account/paired
  routes. Install creates no endpoint; state uses `try_lock` under registry
  bookkeeping so WAN metadata cannot block LAN; pins survive tearing. EVERY demand
  calls `prepare_demand`, including reuse: stale identity/dead reader cannot ack
  a phantom mirror. Validate reserved `account:` before `TopicKey` trims names.
- Identity snapshots carry public bytes only, hold the auth-store lock across every
  read, and refuse rather than wait on a busy writer. Expiry gates new enrollment,
  never durable bindings; an account or key change fences the controller.
- `release_max_level_info`: `debug!` is absent from a release build, so operator
  outcomes log at `info!` or louder. Sibling lookup goes through
  `resolve_netd_bin_from` (macOS `current_exe()` is not canonicalized), and netd
  links READER-only `cerulion_discovery`: no peers.json write path here.
## Testing
- Keep `--no-default-features` green; count oracles, never wall clocks.
- Build `cerulion_cli` first; `wan_plane_iroh_test` includes the real CLI/default
  SHM namespace and owner controller: run `-- --test-threads=1` (CI does too).
- `#[traced_test]` capture is thread-local: a plane's tokio worker is invisible to
  it. Test-only beacon suppression stays out of `src/**` (a walk enforces it).
Read: docs/internals/network-daemons.md, docs/internals/remote-access.md.
