# cerulion_netd

One host gateway/zenoh session; NDJSON refcounts; spy wiring pins

- Protocol12: paired catalogs. WAN observers use `/__cerulion/observers/`; LAN shares canonical demands. Last observer releases source/mode mirror; wire/2 canonical. MirrorLost queues without replacing replies; demand-free Busy probes mean unknown. Supervised forwards startup errors; Paired/Open watches latch them; serving clears them.
- Demand/Observe: same-socket retry of exact controller/login Busy or same-source ensure wrapper, seven uneven deterministic backoffs within 2 s. Exhaustion preserves text/latches warn; retry/recovery debug.

## Rules
- LAN: Paired everywhere; explicit Open warns. Delivery fixtures: Open.
- Gateway/probe/taps obey SHM producer scope even on open LAN: local-only and outputs omitted from declared egress stay private; taps retain leases.
- Singleton is kernel flock on `cerulion_hygiene::NETD` pidfile, released on death; no second scheme. Android uses an abstract same-UID socket.
- Connection close IS release (crash-safe refcounts). LAST release retires
  the mirror and frees the single-writer SHM slot; a re-demand recreates it.
- ONE mirror per source/mode across planes. Status uses `MirrorPlane::serving_plane`, pinned at ensure; absent is unknown, not LAN.
- WAN default-on; netd outside default-members keeps plain builds iroh-free. Lean consumers use default-features=false; either changes workspace builds.
- Never claim a confident empty at cold start: `DiscoveryState::NotConverged`
  until `query::classify_gather` settles. Additive serde defaults making POSITIVE
  claims require a `PROTOCOL_VERSION` bump and client floor `<=` assertion, never equality.
- Convergence uses non-reconnecting `_once` (reconnect restarts spawn-wait); cancellation never proves absence.
- Control waits on readable socket OR push slot; socket-only stalls idle catalogs.
- Only nonzero non-loopback TCP `CERULION_NETD_LISTEN` admits mDNS and standing mode
  (gateway boots at start, no idle exit). Missing means refusal, never a default port;
  `CERULION_NETD_NETWORK=off` wins.
- Egress rechecks prior login (expiry ignored) or claimed trust/key/index;
  LISTEN needs login; off/injected exempt. Nonempty egress or hold_serving starts
  one remoted without LISTEN; query/off start none; standing lasts to shutdown.
- Idle reap waits for work/latch/owed delivery, retains singleton and exclusive
  `serving.hold`; guards/sources share it, never unlink. Own process group, direct
  TERM, hard budget plus 500 ms and fetch grace. See network-daemons.md.
- Lifecycle I/O runs outside verbs, unlocked; outbound readers fence reap.
  begin/ready/end restore live pins in 5 s or name losses; no empty migration.
- ONE `AccountWanController` serves manual/account routes. Install creates no endpoint.
  State `try_lock` under registry bookkeeping keeps LAN from waiting on WAN metadata;
  pins survive teardown. EVERY demand calls `prepare_demand`: stale identity/dead reader
  cannot acknowledge a phantom. Validate `account:` before `TopicKey` trims names.
- Bounded same-user `outbound.sock`: shared endpoint/runtime, key/namespace
  gate. Metadata per-call; snapshot selected/held peers. Refusals keep holds; cleanup errors fence.
  Loss retires readers; pre-Ensure Sync retries own none. Postcard owner; one key bind.
- Identity reads hold auth lock; busy writers refuse. Expiry gates enrollment; durable bindings survive expiry, fenced by account/key changes.
- Releases: debug; operator outcomes: info+. Siblings use `resolve_netd_bin_from` (macOS current_exe uncanonicalized). Discovery never writes peers.json.

## Tests
- `-- --test-threads=1` (CLI shares SHM); `--no-default-features` green. Count oracles, not clocks.
- Build `cerulion_{cli,remoted,connectd}` first. WAN fixtures use real CLI/default
  SHM namespace and owner controller, serially.
- `#[traced_test]` capture is thread-local: plane tokio workers are invisible.
  Test-only beacon suppression stays out of `src/**` (walk enforced).

See: docs/internals/network-daemons.md, docs/internals/remote-access.md.
