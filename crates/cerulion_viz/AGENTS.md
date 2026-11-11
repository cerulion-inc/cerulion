# cerulion_viz

Crates: lib/cerulion_viz (render/sink), lib/go2_tf (pure TFMessage), bin/cerulion_vizd (desk daemon). Outside default-members; plain builds stay rerun-free (`default_member_build_is_rerun_free`); use `-p`.

- Remote: netd local_channel; viewer pins channel/topic/schema/robot. Attach/detach lock outside poll state (3 s); refusals release one demand, rollback waits 45 s. Source changes need detach. Each source holds a client; demand-free probes skip busy passes. Recovery pins source/hash/channel with backoff; drop intent before release I/O.

## Rules
- Live viz is INSTANT-ONLY: reconnects get statics + blueprint, never replay (`re_grpc_server` fork in
  `[patch.crates-io]`); bump rerun via fork merge only.
- Never re-log statics on a cadence: `log_static` is display-idempotent but storage-APPEND; re-log on byte change.
- Vizd control never waits/polls: one round trip (`convergence_adoption_test`).
- Taps and netd demands are daemon-global: they survive control-close; detach, compose rollback or shutdown ends them. Close releases only events.
- All attach seams (remote/local/compose) opens a wake listener (`WakeMode::Listener`); drain
  before waits; wakes signal, never count.
- `account:<robot-id>` routes keep netd's refusals and never fall back to LAN; validate before
  pinned-schema fast paths (`account_route_test`).
- Discover/list/status/attach share `Ctx::attribution_snapshot`; poll cached post-attach attribution, never read once.
- Dump-companion panes are gated on `RenderProof` (per input, sticky); a new layout-affecting
  signal joins the drain-loop comparison or it never reflows.
- MarkerArray is a stateful mutation stream: never auto-clear absent markers, never coalesce
  frames (a dropped one may carry the only DELETE).
- OpenH264 comes from Cisco's CDN at runtime, never vendored; digest-check before cache write.
- Models (`load_model`/`model_status`): handlers read no files; the attachment lock spans worker
  admission/cancel AND the drained-frame handoff; each removal seam (detach, compose rollback) cancels
  only the exact model-owned route; installed-model reflow never overrides explicit layout.

## Tests
`cargo test -p cerulion_viz` and `-p go2_tf` run PARALLEL; `-p cerulion_vizd -- --test-threads=1`
(blueprint mutex, env, occupancy oracle).

- Tests confine process globals (viz.md §4 mechanisms) or belong in vizd.
- Hz: absolute ceiling + ratio to achieved rate, never a band; whole seconds,
  never poll-interval wall units (macOS coalescing); use `taskpolicy -b`.
- No machine-wide port-absence asserts: read `StreamResolution::hosted_port`
  (`host_test.rs`). Frame oracles use per-frame lockstep, never publish-N/drain.
- `account_viz_loopback_test` (serial): real account HTTP through vizd attach to exact scalars.

## Gotchas
- Every URDF visual/joint vector is strict: malformed/non-finite is `InvalidVector`
  with XML line; only absent attributes default. Explicit import: `validate_urdf`
  refuses unread attributes/unsupported geometry/material/unbound movable joints,
  then `try_load` bounds reads and forbids .glb fallback; `load` stays tolerant.
- Bound models: statics ONLY on fixed joints (one on a movable entity shadows measurements); joint SDK
  submissions pace >= 16,666,667 ns keeping the LATEST valid pose; reconnect/panic drop pose and deadline.
  Loader: no lock spans file reads or SDK calls; a handle's sender mutex BEFORE the loader mutex.
- `MemorySinkStorage::num_msgs()` counts CHUNKS; the micro-batcher compacts same-entity rows, so exact
  counts need distinct entities or `flush_blocking()` bounds. No `Mesh3D::sanity_check()`.
- `CoordinateFrame:frame` moves an entity's own data; `Transform3D:parent_frame` is what the resolver
  walks - assert RESOLVED composition, never chunk presence. Rerun-dependent crates declare `rust-version`.
- Only `try_load` admits URDF `<material>` (must match the DAE diffuse, never applied). tf blobs and
  PointCloud2 point-fields are bespoke encodings, OPAQUE by design.

See: docs/internals/viz.md (seams, layout, rerun fork, models, tests).
