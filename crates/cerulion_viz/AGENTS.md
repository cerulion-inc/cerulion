# cerulion_viz - agent notes

Three crates: `lib/cerulion_viz` (rerun render/sink library), `lib/go2_tf` (pure
TFMessage codec - no transport, no rerun), `bin/cerulion_vizd` (the desk viz daemon).
NOT default-members: a plain `cargo build` must stay rerun-free (pinned by
`default_member_build_is_rerun_free` in cerulion_hygiene); build with `-p <crate>`.

## Invariants

- Live viz is INSTANT-ONLY: a (re)connecting viewer gets statics + blueprint, never temporal
  replay - via the sparse `re_grpc_server` fork pinned in the root `[patch.crates-io]`. A rerun
  bump is an upstream-import + merge in the fork repo, never a bare version edit here.
- Never re-log statics on a cadence: rerun `log_static` is display-idempotent but
  storage-APPEND - dedup producer-side (re-log only when payload bytes change).
- No vizd control handler may wait or poll - the closed-world question is
  answered in ONE round trip (walked structurally by `convergence_adoption_test`).
- Taps and netd demands are daemon-global: they survive control-connection close and die only on
  explicit detach, compose rollback or daemon shutdown. Close releases ONLY the event subscription.
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
cargo test -p cerulion_viz                       # PARALLEL by design - the stronger gate
cargo test -p cerulion_vizd -- --test-threads=1  # SERIAL: blueprint mutex, env, occupancy oracle
cargo test -p go2_tf                             # pure codec, no globals
```

- A new `cerulion_viz` test must confine process-global state by one of the five
  mechanisms in docs/internals/viz.md §4 - if it fits none, it goes in the vizd lane.
- Rate/Hz asserts: absolute ceiling + ratio-vs-achieved-rate - never a band (a loaded runner only
  pushes measured rates DOWN). Never assert a wall in units of a poll interval (macOS CI timer
  coalescing); bound conditions in whole seconds and reproduce locally with `taskpolicy -b`.
- Never assert a machine-wide port absence: read `StreamResolution::hosted_port` (`host_test.rs`).
- Exact-value frame oracles: per-frame lockstep (one frame in flight), never publish-N-then-drain.

## Gotchas

- URDF vectors are strict on EVERY visual/joint: a malformed or non-finite supplied value is an
  `InvalidVector` error with its XML line; only an ABSENT attribute takes a default, never a zero.
  Explicit imports preflight with `Skeleton::validate_urdf` (fail-closed: unread attributes,
  unsupported geometry/materials, unbound movable joints); `load`/`from_urdf_str` stay tolerant.
- `MemorySinkStorage::num_msgs()` counts CHUNKS; the micro-batcher compacts same-entity rows, so
  exact-count oracles need distinct entities or `flush_blocking()` boundaries. No `Mesh3D::sanity_check()`.
- `CoordinateFrame:frame` moves an entity's own data; `Transform3D:parent_frame` is what the resolver
  walks - assert RESOLVED composition, never chunk presence. Every rerun-dependent crate declares `rust-version`.
- The tf transforms blob and PointCloud2 point-fields are bespoke encodings
  pinned OPAQUE by design - not canonical element framing, not a bug.

Deep reference: docs/internals/viz.md - read before touching vizd's control/attach
seams, render-proof/layout, the rerun fork, or adding a test.
