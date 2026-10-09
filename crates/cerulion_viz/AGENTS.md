# cerulion_viz - agent notes

Crates: `lib/cerulion_viz` (rerun render/sink lib), `lib/go2_tf` (pure TFMessage codec, no transport or
rerun), `bin/cerulion_vizd` (desk viz daemon). NOT default-members: a plain `cargo build` stays rerun-free
(pinned by `default_member_build_is_rerun_free`); use `-p <crate>`.

## Invariants

- Live viz is INSTANT-ONLY: a (re)connecting viewer gets statics + blueprint, never temporal replay (sparse
  `re_grpc_server` fork, root `[patch.crates-io]`); bump rerun via upstream-import + fork merge only.
- Never re-log statics on a cadence: `log_static` is display-idempotent but storage-APPEND; re-log
  only on payload byte change.
- No vizd control handler may wait or poll - the closed-world question is answered in ONE round
  trip (`convergence_adoption_test`).
- Taps and netd demands are daemon-global: they survive control-connection close and die only on
  detach, compose rollback or shutdown; close releases ONLY the event subscription.
- Every attach seam (remote/local/compose) opens a wake listener (`WakeMode::Listener`); drain
  before waiting - a wake is a signal, not a count.
- The four topic surfaces (discover/list/status/attach) read ONE attribution map
  (`Ctx::attribution_snapshot`); post-attach attribution is cached - poll, never read once.
- Dump-companion panes are gated on `RenderProof` (per input, sticky); a new layout-affecting
  signal joins the drain-loop comparison or it never reflows.
- MarkerArray is a stateful mutation stream: never auto-clear absent markers, never coalesce
  frames (a dropped one may carry the only DELETE).
- OpenH264 is fetched at runtime from Cisco's CDN, never vendored (patent grant); digest-check before
  the cache write.
- Model control (`load_model`/`model_status`): handlers read no files; the attachment lock spans worker
  admission/cancel AND the drained-frame handoff; every removal seam (detach, compose rollback) cancels
  only the exact model-owned route; an installed-model reflow never overrides an explicit layout.

## Testing

```bash
cargo test -p cerulion_viz  # PARALLEL by design, the stronger gate
cargo test -p cerulion_vizd -- --test-threads=1  # SERIAL: blueprint mutex, env, occupancy oracle
cargo test -p go2_tf  # pure codec, no globals
```

- A new `cerulion_viz` test confines process-global state (five mechanisms, docs/internals/viz.md §4) or goes in the vizd lane.
- Rate/Hz asserts: absolute ceiling + ratio-vs-achieved-rate, never a band (a loaded runner only pushes
  rates DOWN). No wall in poll-interval units (macOS timer coalescing): whole seconds, `taskpolicy -b`.
- No machine-wide port-absence asserts: read `StreamResolution::hosted_port` (`host_test.rs`). Exact
  frame oracles: per-frame lockstep, never publish-N-then-drain.

## Gotchas

- URDF vectors are strict on EVERY visual/joint: malformed or non-finite = `InvalidVector` with its XML
  line; only an ABSENT attribute defaults. Explicit imports: `validate_urdf` preflight, then
  `try_load` (bounded reads off control threads, format from the URDF reference, no `.glb` fallback).
- Bound models: statics ONLY on fixed joints (one on a movable entity shadows measurements); joint SDK
  submissions pace >= 16,666,667 ns keeping the LATEST valid pose; reconnect/panic drop pose and deadline.
  Loader: no lock spans file reads or SDK calls; a handle's sender mutex BEFORE the loader mutex.
- `MemorySinkStorage::num_msgs()` counts CHUNKS; the micro-batcher compacts same-entity rows, so exact
  counts need distinct entities or `flush_blocking()` bounds. No `Mesh3D::sanity_check()`.
- `CoordinateFrame:frame` moves an entity's own data; `Transform3D:parent_frame` is what the resolver
  walks - assert RESOLVED composition, never chunk presence. Rerun-dependent crates declare `rust-version`.
- Only `try_load` admits URDF `<material>`: proven against the frozen DAE's used diffuse effects, never
  applied. The tf transforms blob and PointCloud2 point-fields are bespoke encodings OPAQUE by design.

Deep reference: docs/internals/viz.md (attach seams, layout, rerun fork, models, tests).
