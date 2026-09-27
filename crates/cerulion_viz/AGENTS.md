# cerulion_viz - agent notes

Three crates: `lib/cerulion_viz` (rerun render/sink library), `lib/go2_tf` (pure
TFMessage codec, no transport, no rerun) and `bin/cerulion_vizd` (the desk viz
daemon). NOT default-members: a plain `cargo build` stays rerun-free (CI enforces
it); build with `-p <crate>`.

## Invariants
- Live viz is INSTANT-ONLY: a reconnecting viewer gets statics and blueprint, never
  temporal replay, via the sparse `re_grpc_server` fork pinned in the root
  `[patch.crates-io]`. A rerun bump is an upstream import and merge in the fork repo,
  never a bare version edit here.
- Never re-log statics on a cadence: rerun `log_static` is display-idempotent but
  storage-APPEND, so dedup producer-side. No vizd control handler may wait or poll:
  the closed-world question is answered in ONE round trip
  (`convergence_adoption_test` walks for it).
- Taps and netd demands are daemon-global: they survive control-connection close and
  die only on explicit detach, compose rollback or daemon shutdown; a close releases
  ONLY the event subscription. Every attach seam opens a wake listener
  (`WakeMode::Listener`); drain before waiting, a wake is a signal not a count.
- Reserved `account:<robot-id>` routes keep netd catalog and schema refusals and never
  fall back to vizd's LAN session; validate before pinned-schema fast paths
  (`account_route_test`).
- All four topic surfaces read ONE attribution map (`Ctx::attribution_snapshot`); it
  is cached, so poll, never read once. Dump-companion panes are gated on `RenderProof`
  (per input, sticky): a new layout-affecting signal must join the drain-loop
  comparison or it never reflows.
- MarkerArray is a stateful mutation stream: never auto-clear absent markers, never
  coalesce its frames (a dropped frame may carry the only DELETE). OpenH264 is fetched
  at runtime from Cisco's CDN, never vendored; digest-check BEFORE the cache write.

## Testing
```bash
cargo test -p cerulion_viz                       # PARALLEL by design - the stronger gate
cargo test -p cerulion_vizd -- --test-threads=1  # SERIAL: blueprint mutex, env, occupancy oracle
cargo test -p go2_tf                             # pure codec, no globals
```

- A new `cerulion_viz` test confines process-global state by one of the five mechanisms
  in docs/internals/viz.md §4; if none fits, it goes in the vizd lane.
- Rate/Hz asserts: absolute ceiling plus ratio-vs-achieved-rate, never a band (a loaded
  runner only pushes measured rates DOWN), and never a wall in units of a poll interval
  (macOS CI timer coalescing): bound conditions in whole seconds, reproduce with
  `taskpolicy -b`. Exact-value frame oracles use per-frame lockstep, never
  publish-N-then-drain.
- `account_viz_loopback_test` is its own serial binary: real account HTTP, owner
  enrollment, controller, schema provider, vizd attach, exact rendered scalars and
  physical mirror retirement. Its WAN edges stay dev-only.

## Gotchas
- `MemorySinkStorage::num_msgs()` counts CHUNKS and the micro-batcher compacts
  same-entity rows: exact-count oracles need distinct entities or `flush_blocking()`
  boundaries, and never call `Mesh3D::sanity_check()`.
- `CoordinateFrame:frame` moves an entity's own data; `Transform3D:parent_frame` is
  what the resolver walks, so assert RESOLVED composition, never chunk presence. Every
  crate whose deps reach rerun declares `rust-version`. The tf transforms blob and
  PointCloud2 point-fields are bespoke encodings pinned OPAQUE by design.

Deep reference: docs/internals/viz.md, before touching vizd's control and attach
seams, render-proof or layout, the rerun fork, or adding a test.
