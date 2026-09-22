# cerulion_viz - agent notes

Three crates: `lib/cerulion_viz` (rerun render/sink library), `lib/go2_tf` (pure
TFMessage codec - no transport, no rerun), `bin/cerulion_vizd` (the desk viz daemon).
NOT default-members: a plain `cargo build` must stay rerun-free (CI's rerun-leanness
job enforces it); build with `-p <crate>`.

## Invariants

- Live viz is INSTANT-ONLY: a (re)connecting viewer gets statics + blueprint,
  never temporal replay - via the sparse `re_grpc_server` fork pinned in the root
  `[patch.crates-io]`. A rerun bump is an upstream-import + merge in the fork
  repo, never a bare version edit here.
- Never re-log statics on a cadence: rerun `log_static` is display-idempotent but
  storage-APPEND - dedup producer-side (re-log only when payload bytes change).
- No vizd control handler may wait or poll - the closed-world question is
  answered in ONE round trip (walked structurally by `convergence_adoption_test`).
- Taps and netd demands are daemon-global: they survive control-connection close
  and die only on explicit detach, compose rollback, or daemon shutdown. Close
  releases ONLY the event subscription.
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
- Rate/Hz asserts: absolute ceiling + ratio-vs-achieved-rate - never a band (a
  loaded runner only pushes measured rates DOWN). Never assert a wall in units of
  a poll interval (macOS CI timer coalescing); bound conditions in whole seconds
  and reproduce locally with `taskpolicy -b`.
- Exact-value frame oracles: per-frame lockstep (one frame in flight), never publish-N-then-drain.

## Gotchas

- URDF numeric attributes are strict: malformed or non-finite vectors fail with
  XML location context. Only absent attributes receive defaults; never silently
  replace invalid supplied geometry with zeros.

- `MemorySinkStorage::num_msgs()` counts CHUNKS and the micro-batcher compacts
  same-entity rows - exact-count oracles need distinct entities or
  `flush_blocking()` boundaries. Never call `Mesh3D::sanity_check()`.
- `CoordinateFrame:frame` moves an entity's own data; `Transform3D:parent_frame`
  is what the transform resolver walks - assert RESOLVED composition, never
  chunk presence. Every crate whose deps reach rerun declares `rust-version`.
- The tf transforms blob and PointCloud2 point-fields are bespoke encodings
  pinned OPAQUE by design - not canonical element framing, not a bug.

Deep reference: docs/internals/viz.md - read before touching vizd's control/attach
seams, render-proof/layout, the rerun fork, or adding a test.

Explicit URDF imports use `Skeleton::validate_urdf` preflight; legacy constructors
are intentionally best-effort. Keep unsupported geometry/materials and malformed
topology loud. Require exactly one binding for each movable joint; fixed-only
models may omit bindings. Validation alone reads no assets and proves no live articulation.

`Skeleton::try_load` freezes original mesh bytes with bounded reads; run it off
control threads. Loaded bytes do not establish GPU rendering. Derive formats from
the URDF mesh reference, not its canonical symlink target; aliases sharing a file
must agree on format. Resolve relative assets from the canonical URDF target.

Only `try_load` may defer explicit material checks: prove matching used DAE diffuse
effects against frozen bytes before returning. Never strip declarations or confuse
metadata proof with rendered appearance. Require a single explicitly selected
DAE visual scene; the native decoder does not honor multi-scene selection.
