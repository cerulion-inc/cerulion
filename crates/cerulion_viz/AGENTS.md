# cerulion_viz - agent notes

Crates: `lib/cerulion_viz` (rerun render/sink lib), `lib/go2_tf` (pure TFMessage codec, no transport,
no rerun), `bin/cerulion_vizd` (desk viz daemon). NOT default-members: a plain `cargo build` stays
rerun-free (pinned by `default_member_build_is_rerun_free` in cerulion_hygiene); use `-p <crate>`.

## Invariants

- Live viz is INSTANT-ONLY: a (re)connecting viewer gets statics + blueprint, never temporal replay (sparse
  `re_grpc_server` fork, root `[patch.crates-io]`); bump rerun via upstream-import + fork merge, never a bare edit.
- Never re-log statics on a cadence: `log_static` is display-idempotent but storage-APPEND; dedup
  producer-side: re-log only when payload bytes change.
- No vizd control handler may wait or poll - the closed-world question is answered in ONE round
  trip (walked by `convergence_adoption_test`).
- Taps and netd demands are daemon-global: they survive control-connection close and die only on
  detach, compose rollback or shutdown; close releases ONLY the event subscription.
- Every attach seam (remote/local/compose) opens a wake listener
  (`WakeMode::Listener`); drain before waiting - a wake is a signal, not a count.
- All four topic surfaces (discover/list/status/attach) read ONE attribution map
  (`Ctx::attribution_snapshot`); post-attach attribution is cached - poll, never read once.
- Dump-companion panes are gated on `RenderProof` (per input, sticky); a new
  layout-affecting signal must join the drain-loop comparison or it never reflows.
- MarkerArray is a stateful mutation stream: never auto-clear absent markers,
  never coalesce its frames (a dropped frame may carry the only DELETE).
- OpenH264 is fetched at runtime from Cisco's CDN, never vendored (the patent
  grant covers only Cisco-distributed binaries); digest-check BEFORE cache write.

## Testing

```bash
cargo test -p cerulion_viz  # PARALLEL by design, the stronger gate
cargo test -p cerulion_vizd -- --test-threads=1  # SERIAL: blueprint mutex, env, occupancy oracle
cargo test -p go2_tf  # pure codec, no globals
```

- A new `cerulion_viz` test confines process-global state by one of the five mechanisms in
  docs/internals/viz.md §4, or it goes in the vizd lane.
- Rate/Hz asserts: absolute ceiling + ratio-vs-achieved-rate, never a band (a loaded runner only pushes
  rates DOWN). Never assert a wall in poll-interval units (macOS CI timer coalescing): whole-second
  bounds, reproduced with `taskpolicy -b`.
- Never assert a machine-wide port absence: read `StreamResolution::hosted_port` (`host_test.rs`).
- Exact-value frame oracles: per-frame lockstep (one frame in flight), never publish-N-then-drain.

## Gotchas

- URDF vectors are strict on EVERY visual/joint: malformed or non-finite = `InvalidVector` with its XML
  line; only an ABSENT attribute defaults, never to zero. Explicit imports: `validate_urdf` preflight
  (fail-closed), then `try_load` (bounded reads off control threads, format from the URDF reference,
  no `.glb` fallback); `load` stays tolerant.
- Bound models: statics ONLY on fixed joints (one on a movable entity shadows measurements); joint SDK
  submissions pace >= 16,666,667 ns keeping the LATEST valid pose; reconnect/panic drop pose AND deadline.
  Loader: no lock spans file reads or SDK calls; take a handle's sender mutex BEFORE the loader mutex.
- `MemorySinkStorage::num_msgs()` counts CHUNKS; the micro-batcher compacts same-entity rows, so
  exact-count oracles need distinct entities or `flush_blocking()` bounds. No `Mesh3D::sanity_check()`.
- `CoordinateFrame:frame` moves an entity's own data; `Transform3D:parent_frame` is what the resolver
  walks - assert RESOLVED composition, never chunk presence. Every rerun-dependent crate declares `rust-version`.
- Only `try_load` admits URDF `<material>`: proven against the frozen DAE's used diffuse effects, never
  applied. The tf transforms blob and PointCloud2 point-fields are bespoke encodings OPAQUE by design.

Deep reference: docs/internals/viz.md (control/attach seams, render-proof/layout, rerun fork, models, tests).
