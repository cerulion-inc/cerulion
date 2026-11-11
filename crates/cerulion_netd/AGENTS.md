# cerulion_netd

One gateway/zenoh session/host; refcounted NDJSON demands. Spies prove wiring only.

- Protocol11: paired catalogs; WAN observers use private `/__cerulion/observers/` channels, LAN observers share canonical demands. Last observer releases its source/mode mirror; wire/2 stays canonical. MirrorLost queues losses without replacing replies; demand-free probes treat busy as unknown. Run watches latch startup errors.

## Rules
- LAN defaults to Paired everywhere; explicit Open warns. Delivery fixtures select Open.
- Gateway/probe/taps obey SHM producer scope even on open LAN: local-only and outputs omitted from declared egress stay private; taps retain leases.
- Singleton is kernel flock on `cerulion_hygiene::NETD` pidfile, released on death; no second scheme. Android uses an abstract same-UID socket.
- Connection close IS the release (crash-safe refcounts). The LAST release retires
  the mirror and frees the single-writer SHM slot; a re-demand recreates it.
- ONE mirror per source/mode across planes. Status uses `MirrorPlane::serving_plane`, pinned at ensure; absent is unknown, not LAN.
- WAN default-on; netd outside default-members keeps plain builds iroh-free. Lean consumers use default-features=false; either changes workspace builds.
- Never answer a cold start with a confident empty: `DiscoveryState::NotConverged`
  until discovery settles (`query::classify_gather`). An additive field whose serde
  default is a POSITIVE claim needs a `PROTOCOL_VERSION` bump plus a client floor,
  `<=`-asserted, never an equality pin.
- Convergence uses non-reconnecting `_once` (reconnect restarts spawn-wait); cancellation never proves absence.
- Control waits on readable socket OR filled push slot; socket-only stalls idle-client catalogs.
- A nonzero non-loopback TCP `CERULION_NETD_LISTEN` port is the ONLY gate for the mDNS
  beacon and standing mode (gateway boots at start, idle self-exit off). None means a
  refusal, never a default port; `CERULION_NETD_NETWORK=off` wins over both.
- Egress rechecks prior login (expiry ignored) or claimed trust/key/index;
  LISTEN needs login; off/injected exempt. First nonempty WAN egress starts one
  remoted without LISTEN; query/off starts none; standing persists until shutdown.
- Idle reap waits for work/latch/owed delivery, retains singleton and exclusive
  `serving.hold`; guards/sources share it, never unlink. Own process group, direct
  TERM, hard budget plus 500 ms and fetch grace. See network-daemons.md.
- Lifecycle I/O runs off verbs and unlocked; outbound readers fence idle reap.
  begin/ready/end restore live pins in 5 s or name losses; no empty migration.
- ONE `account_controller::AccountWanController` serves manual and account routes:
  install creates no endpoint, controller state is `try_lock` under registry
  bookkeeping so LAN never waits on WAN metadata; pins survive tearing. EVERY
  demand calls `prepare_demand`: stale identity or dead reader cannot acknowledge
  a phantom mirror. Validate `account:` before `TopicKey` trims ordinary names.
- Bounded same-user `outbound.sock` shares endpoint/runtime; key/namespace
  gate before dial. Metadata per-call; validation refusals keep reader holds; cleanup errors fence.
  Loss retires readers; pre-Ensure Sync retries own none. Postcard owner; one key bind.
- Public identity bytes are auth-locked across reads; busy writers refuse. Expiry gates enrollment, not durable bindings; account/key changes fence it.
- Releases: debug; operator outcomes: info+. Siblings use `resolve_netd_bin_from` (macOS current_exe uncanonicalized). Discovery never writes peers.json.

## Tests
- `-- --test-threads=1` (CLI shares SHM); `--no-default-features` green. Count oracles, not clocks.
- Build `cerulion_cli`/`cerulion_remoted` first; WAN fixtures use the real CLI/default
  SHM namespace and owner controller; CI runs serial.
- `#[traced_test]` capture is thread-local: plane tokio workers are invisible.
  Test-only beacon suppression stays out of `src/**` (walk enforced).

See: docs/internals/network-daemons.md, docs/internals/remote-access.md.
