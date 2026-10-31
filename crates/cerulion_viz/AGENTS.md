# cerulion_viz - agent notes

Crates: `lib/cerulion_viz` (rerun render/sink lib), `lib/go2_tf` (pure TFMessage codec, no transport or
rerun), `bin/cerulion_vizd` (desk viz daemon). NOT default-members: a plain `cargo build` stays rerun-free
(pinned by `default_member_build_is_rerun_free`); use `-p <crate>`.

## Invariants

- Live viz is INSTANT-ONLY: reconnects get statics + blueprint, never replay (`re_grpc_server` fork in
  `[patch.crates-io]`); bump rerun via fork merge only.
- Never re-log statics on a cadence: `log_static` is display-idempotent but storage-APPEND; re-log on byte change.
- No vizd control handler waits or polls: ONE round trip (`convergence_adoption_test`).
- Taps and netd demands are daemon-global: they survive control-connection close and die only on
  detach, compose rollback or shutdown; close releases ONLY the event subscription.
- Every attach seam (remote/local/compose) opens a wake listener (`WakeMode::Listener`); drain
  before waiting - a wake is a signal, not a count.
- `account:<robot-id>` routes keep netd's refusals and never fall back to LAN; validate before
  pinned-schema fast paths (`account_route_test`).
- The four topic surfaces (discover/list/status/attach) read ONE attribution map
  (`Ctx::attribution_snapshot`); post-attach attribution is cached - poll, never read once.
- Dump-companion panes are gated on `RenderProof` (per input, sticky); a new layout-affecting
  signal joins the drain-loop comparison or it never reflows.
- MarkerArray is a stateful mutation stream: never auto-clear absent markers, never coalesce
  frames (a dropped one may carry the only DELETE).
- OpenH264 comes from Cisco's CDN at runtime, never vendored; digest-check before cache write.
- Model control (`load_model`/`model_status`): handlers read no files; the attachment lock spans worker
  admission/cancel AND the drained-frame handoff; every removal seam (detach, compose rollback) cancels
  only the exact model-owned route; an installed-model reflow never overrides an explicit layout.

## Testing

`cargo test -p cerulion_viz` and `-p go2_tf` run PARALLEL; `-p cerulion_vizd -- --test-threads=1`
(blueprint mutex, env, occupancy oracle).

- A new `cerulion_viz` test confines process-global state (viz.md §4's five mechanisms) or goes in the vizd lane.
- Rate/Hz asserts: absolute ceiling + ratio-vs-achieved-rate, never a band. No wall in poll-interval units (macOS timer coalescing): whole seconds, `taskpolicy -b`.
- No machine-wide port-absence asserts: read `StreamResolution::hosted_port` (`host_test.rs`). Exact
  frame oracles: per-frame lockstep, never publish-N-then-drain.
- `account_viz_loopback_test` (serial): real account HTTP through vizd attach to exact scalars.

## Gotchas

- URDF vectors are strict on EVERY visual/joint: malformed or non-finite = `InvalidVector` with its XML
  line; only an ABSENT attribute defaults. Explicit imports: `validate_urdf` preflight (fail-closed: unread
  attributes, unsupported geometry/materials, unbound movable joints), then `try_load` (bounded reads, no
  `.glb` fallback); `load` stays tolerant.
- Bound models: statics ONLY on fixed joints (one on a movable entity shadows measurements); joint SDK
  submissions pace >= 16,666,667 ns keeping the LATEST valid pose; reconnect/panic drop pose and deadline.
  Loader: no lock spans file reads or SDK calls; a handle's sender mutex BEFORE the loader mutex.
- `MemorySinkStorage::num_msgs()` counts CHUNKS; the micro-batcher compacts same-entity rows, so exact
  counts need distinct entities or `flush_blocking()` bounds. No `Mesh3D::sanity_check()`.
- `CoordinateFrame:frame` moves an entity's own data; `Transform3D:parent_frame` is what the resolver
  walks - assert RESOLVED composition, never chunk presence. Rerun-dependent crates declare `rust-version`.
- Only `try_load` admits URDF `<material>` (must match the DAE diffuse, never applied). tf blobs and
  PointCloud2 point-fields are bespoke encodings, OPAQUE by design.

Deep reference: docs/internals/viz.md (seams, layout, rerun fork, models, tests).
