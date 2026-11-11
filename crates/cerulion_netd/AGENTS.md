# cerulion_netd

One gateway/zenoh session per host; refcounted NDJSON demands. Injected spies prove wiring, not delivery.

- Protocol10: paired catalogs; WAN observers use private `/__cerulion/observers/` channels, LAN observers share canonical demands. Last observer releases its source/mode mirror; wire/2 stays canonical.

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
  reported refusal, never a default port; `CERULION_NETD_NETWORK=off` wins over both.
- Egress rechecks prior login (expiry ignored) or claimed trust/key/index;
  LISTEN needs login; off/injected exempt. First nonempty WAN egress starts one
  remoted without LISTEN; query/off starts none; standing persists until shutdown.
- Idle reap waits for work/latch/owed delivery, retains singleton and exclusive
  `serving.hold`; guards/sources share it, never unlink. Own process group, direct
  TERM, hard budget plus 500 ms and fetch grace. See network-daemons.md.
- Controller lock fences outgoing at `begin_serving`; `end_serving` follows reap;
  facts use `discovery::robot_state`.
- ONE `account_controller::AccountWanController` serves manual and account routes:
  install creates no endpoint, controller state is `try_lock` under registry
  bookkeeping so LAN never waits on WAN metadata; pins survive tearing. EVERY
  demand calls `prepare_demand`: stale identity or dead reader cannot acknowledge
  a phantom mirror. Validate `account:` before `TopicKey` trims ordinary names.
- Serving controllers use remoted's bounded same-user `outbound.sock`, endpoint
  and runtime. Key and SHM namespace must match before dialing. Close retires
  mirrors; account fencing preserves the owned endpoint. Never bind its key twice.
- Identity snapshots are public bytes, auth-store locked across reads; busy writers refuse. Expiry gates enrollment, never durable bindings; account/key change fences the controller.
- Release removes debug; operator outcomes use info+. Siblings use `resolve_netd_bin_from` (macOS current_exe uncanonicalized). Discovery is reader-only: no peers.json writes.

## Testing
- Keep `--no-default-features` green; count oracles, never wall clocks.
- Build `cerulion_cli` first; `wan_plane_iroh_test` includes the real CLI/default
  SHM namespace and owner controller: run `-- --test-threads=1` (CI does too).
- `#[traced_test]` capture is thread-local: a plane's tokio worker is invisible to
  it. Test-only beacon suppression stays out of `src/**` (a walk enforces it).

Deep reference: docs/internals/network-daemons.md, docs/internals/remote-access.md.
