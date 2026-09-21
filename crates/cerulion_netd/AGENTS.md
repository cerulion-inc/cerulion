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
