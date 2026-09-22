# Viz internals: `crates/cerulion_viz/*`

Contributor dossier for the visualization tree: the render library, the desk
visualization daemon (`cerulion-vizd`), and the TF codec. Companion to
`crates/cerulion_viz/AGENTS.md`.

## 1. Tree and build boundary

| Crate | Path | What it is |
|---|---|---|
| `cerulion_viz` | `crates/cerulion_viz/lib/cerulion_viz` | Generic rerun render/sink library: frame decode → archetype dispatch → rerun log calls, tap manager, layout compiler, H.264 video path, OpenH264 fetcher |
| `cerulion_vizd` | `crates/cerulion_viz/bin/cerulion_vizd` | The desk visualization daemon: control server, tap/drain loop, hosted rerun gRPC proxy, layout application |
| `go2_tf` | `crates/cerulion_viz/lib/go2_tf` | Pure `TFMessage` codec (std + thiserror; no transport, no rerun) |

All three are workspace members and deliberately NOT default-members: the rerun
SDK tree enters a build only via `-p cerulion_viz` / `-p cerulion_vizd` /
`--workspace`. CI's rerun-leanness job fails if a plain `cargo build` pulls
rerun; keep new dependencies on the viz side of that line, and put tests that
need the viz stack in `crates/cerulion_viz/lib/cerulion_viz/tests/`.

### The robot/desk boundary: visualization never runs on the robot

Architecture: visualization is desk-side end to end. No viz
node is ever staged into a generated robot graph: `cerulion ros2 attach`
generates a graph declaring the `dds_bridge` node and NOTHING else, and the
removed `--no-viz` flag fails loudly as an unknown argument rather than
surviving as a no-op. The robot ships raw frames; the desk demands a topic,
`cerulion-netd` re-injects it into desk-local SHM, and `cerulion-vizd` decodes
and renders on the user's machine (`cerulion viz --robot <name>`). Pinned by
`generated_graph_stages_no_visualization_node_on_the_robot`
(`crates/cerulion_cli_engine/src/ros_cmd.rs`) and
`ros_attach_rejects_the_removed_no_viz_flag` (`crates/cerulion_cli/src/cli.rs`).

There is no robot-side visualization sink node, and one must not be added.
Recording does not need one: recording is `cerulion_bagd` draining data-only
taps into `cerulion_bag`'s MCAP, and a node declaring only inputs cannot
contribute a single frame to a bag. A robot-side sink would be a second,
redundant conversion path; it costs robot CPU and drags the rerun SDK tree into
the robot's build, the exact boundary the leanness rule above exists to hold.

## 2. vizd daemon contracts

### Control protocol

- JSON requests over a Unix-domain control socket; strictly one reply per
  request, in order. A half-read stream cannot be resynchronized, so the client
  side (`cerulion_cli_engine::viz_client`) poisons a connection whose round
  trip failed.
- **No control handler may wait or poll.** vizd answers the closed-world
  question ("what exists right now?") in ONE round trip; waiting for discovery
  to settle is the CLI's job, driven by the retry hints below. Structurally
  guarded by `tests/convergence_adoption_test.rs`, which walks the whole `src/`
  DIRECTORY (never a hand-maintained file list) so a handler moved to a new
  file stays in scope.
- Wire evolution is additive (absent field = older daemon, never assumed):
  `ErrorResponse.retry: Option<RetryHint>` marks an error a client may re-ask
  once discovery settles (absent = terminal), and `DiscoverResponse.discovery`
  distinguishes a settled gather from a still-discovering one (absent =
  unknown, never treated as settled).
- Catalog-change events: a controller subscribes; each connection owns a
  capacity-one push slot with a lossless merge, drained immediately before each
  `CONN_READ_TIMEOUT`-paced blocking control read. Push latency budget =
  coalesce window + one read timeout; an event-driven rework that waits only
  on the socket would stall pushes forever on an idle client. Connection close
  IS the unsubscribe (RAII on every handler exit path).

### Attach seams and the demand plane

- Three attach seams: **remote** (`attach {topic, robot}`: demands the
  topic's shared mirror from `cerulion-netd`; N desk consumers share one
  mirror), **local** (`attach {topic}`: taps desk SHM directly), and
  **compose** (layout composition attaches its own inputs).
- Taps and netd demands are DAEMON-GLOBAL: they survive control-connection
  close and die only on explicit `detach`, compose rollback, or daemon
  shutdown. The only per-connection state released on close is the event
  subscription. Tap lifetime is daemon-scoped, not connection-scoped: clients
  rely on a tap surviving the connection that created it, so it must not be
  changed to netd-style per-connection release-on-close.
- `detach` releases the netd demand (refcounted daemon-side; the last release
  retires the mirror) and frees the tap.

### Wake discipline

- All three attach seams open a wake listener (`WakeMode::Listener`) on the
  topic's event service; local topics are NOT excluded. The
  structural arm requires `Listener` at all three seams and `Timer` at none;
  the drain loop blocks on the wake with the poll interval as its timeout.
- A wake is a SIGNAL, never a count: drain before waiting; the timeout keeps
  the loop a strict superset of polling; pace on whether the last wait actually
  blocked; never block on an empty source slice (an event with no deliverable
  frame returns instantly and an unpaced loop becomes a busy-spin).
- Policy oracles live in `daemon.rs::wake_policy_oracle`; the e2e is
  `wake_drain_e2e_test.rs` (remote AND local production publisher shapes, with
  the `WakeMode` decision taken by production code, never the test).

### Attribution

- All four topic surfaces (`discover`, `list`, `status`, and the `attach`
  reply) read ONE map: `Ctx::attribution_snapshot` = the live
  mirror-provenance snapshot ∪ each held tap's own recorded `origin_robot`,
  live winning. The fold must not be hand-inlined into one surface: two copies
  can diverge, and a diverging copy makes a remote-attached topic double-list
  under a phantom local section beside its own robot row.
- The per-tap `origin_robot` record (not the name-keyed provenance tombstone,
  which is routing memory only) is what stops a desk producer that later
  reuses a detached remote topic's NAME from being attributed to the
  historical robot.
- Attribution after `attach` is NOT synchronous: the snapshot unions a CACHED
  provenance gather (refreshed on miss, aged by `PROVENANCE_CACHE_TTL`), so a
  missed gather is served stale for the TTL. Tests poll attribution to a
  deadline; reading `list` once right after `attach` is a known flake shape.
- `runs` lists ONE row per `run_id`. A desk whose own netd announces sees each
  local graph twice: from the registry gather (`robot` absent) and echoed back
  over the LAN under its own hostname; `runs.rs::fold_runs` keeps the first row
  it sees (local arm first, then robots in reply order) and drops later rows
  naming the same id. Dedup is over ROWS only: `sources` still carries one
  status per answering source, and two robots running the same graph mint two
  ids and stay two rows; never fold on graph name.

### Render proof and layout reflow

- The dump-companion pane (inspectable field dump beside a rendered view) is
  gated on `RenderProof { rendered_without_dumping, degraded }`, recorded PER
  INPUT (never per schema) at exactly two sites (`render_classified` and
  `anyvalues_fallback`), so a new render arm inherits the signal with no extra
  wiring. Both flags are STICKY: one degradation brings the pane back for good
  (the safe direction: a stale "rendered" claim would hide data; a stale
  "degraded" only shows an extra pane).
- Converted kinds (companion dropped while provably rendering): Image,
  OccupancyGrid, Boxes3D, Path3D, PoseArray3D, MarkerArray, VideoStream.
  Always-on companions: Skeleton, AnyValues, TextLog, ScalarsWithText.
- The drain loop re-derives the layout signal each pass and RE-APPLIES the
  layout when it changes (observable via the `poll_layout_signal_reflows`
  counter). A NEW layout-affecting signal must join that comparison or it
  silently never reflows. `layout_metadata` must report the LIVE set so
  `status` and attach `view_kinds` always match the installed blueprint. A
  steady stream of identical frames must NOT reflow; the ceiling is an inline
  assertion inside `vizd_e2e_test.rs`'s
  `a_rendering_topic_loses_its_dump_pane_and_regains_it_on_degradation_e2e`
  (across a window of proven drain passes, the reflow counter must not move).

### Hosting: instant-only and never-block

- vizd hosts the rerun gRPC message proxy. Live viz is INSTANT-ONLY: a
  (re)connecting viewer receives the scene skeleton (statics + blueprint) and
  ZERO temporal replay: no catch-up burst at any producer bandwidth
  (`live_only_history_test.rs` is the hard gate).
- The drain/sink path NEVER blocks on a slow or wedged viewer: a temporal
  frame arriving at an over-budget live queue is dropped and counted, while
  the scene skeleton always takes the reliable path. Pinned by
  `never_block_grpc_tcp_test.rs` (the real gRPC backpressure shape),
  `live_backlog_test.rs` (byte-occupancy hard gate), and
  `drop_latch_log_test.rs` (the drop-latch log discipline).
- The rerun gRPC client does NOT auto-reconnect after a server bounce; the viz
  worker owns reconnect orchestration (`reconnect_test.rs`).

## 3. rerun integration facts

### The `re_grpc_server` sparse fork

Instant-only + bounded-live hosting need two `ServerOptions` capabilities stock
rerun cannot express (a zero memory limit drops statics too, blank scene; any
non-zero budget retains a tail of high-rate temporal frames): a
drop-temporal-history mode and a live-queue byte budget with drop-not-await
semantics. Both live in a SINGLE-CRATE sparse fork of `re_grpc_server` pinned
in the root `[patch.crates-io]` (branch `upstream` = the crates.io tarball
verbatim, tagged; `main` = upstream + the patches; the version string stays in
lockstep with the rest of the `re_*` graph).

- Upgrading rerun is an upstream-import + merge in the fork repo, then a `rev`
  bump in the root manifest, never a bare version edit here. The fork repo's
  README and patch doc carry the procedure.
- The exit condition (recorded beside the pin and in `deny.toml`): drop the
  fork only when upstream ships BOTH capabilities; they bound two different
  buffers on the same path, and either alone is not enough.
- Landmine: the added `ServerOptions` fields are safe only because rerun's
  `clap`/`run`/`web_viewer` features (which construct `ServerOptions` with
  explicit-field literals) are not compiled in our sdk+server build. Re-check
  on ANY change to the `rerun` feature set.

### SDK behaviors (rerun 0.34)

- `log_static` is display-idempotent but storage-APPEND: every call adds a
  chunk viewer-side. A sink that re-broadcasts statics must dedup
  producer-side (compare payload bytes; re-log only on change) or viewer
  memory grows without bound. Corollary: never design a periodic statics
  re-send; Transform3D/Pinhole are full-history archetypes in the viewer.
- `MemorySinkStorage::num_msgs()` counts LogMsg CHUNKS, and the micro-batcher
  compacts rows logged to the SAME entity within one flush window into one
  chunk; exact-count oracles need distinct entities per logical row or
  `flush_blocking()` boundaries between batches.
- API paths that differ from older examples: `connect_grpc` (renamed from
  `connect_tcp`); `rerun::sink::MemorySinkStorage` (not re-exported at root);
  `set_timestamp_nanos_since_epoch`; `RecordingStreamBuilder::memory()`
  returns `(stream, storage)`; `flush_blocking() -> Result` must be handled
  under `-D warnings`.
- rerun's MSRV exceeds the repo default: every crate whose dep graph
  (dev-deps included) reaches rerun declares `rust-version`, so an MSRV break
  surfaces as a clean toolchain message instead of a confusing compile error.
- Process-global scene setup (ViewCoordinates + Pinhole) uses an
  AtomicBool-swap exactly-once guard, not `std::sync::Once`; a `Once` cannot
  be reset by a test seam, which breaks same-process multi-graph tests.
- Viewer-absent buffering is unbounded ABOVE the bounded write channel: the
  SDK-side batcher accumulates while disconnected (RAM grows; ticks never
  block). The fork's live budget bounds the HOSTED path only; the SDK batcher
  is a separate buffer.
- Safari cannot run the rerun web viewer (gRPC-web fetch-streaming fails);
  native viewer or Chrome are the working paths.

### Frames and transforms

`Skeleton::validate_urdf(xml, config)` is the strict preflight for explicit model
import. It rejects disconnected/cyclic trees, ambiguous link/entity names,
invalid measured-motor bindings, and geometry the current renderer would silently
discard. Supported joints are fixed, revolute, and continuous; a link may have no
visual or one mesh visual. Explicit materials, primitives, multiple visuals, and
mimic joints remain unsupported. Limits are 4096 links, depth 256, 4096-byte entity
paths, and 12 motor bindings. Every movable joint must have exactly one binding;
fixed-only models may omit bindings. Entity roots use slash-separated ASCII letters,
digits, underscores, and hyphens. They must not start with Rerun's reserved `__`
prefix; nested segments such as `world/__nested` are allowed.
This check reads no assets and installs nothing;
mesh loading, production binding, and resolved-transform acceptance remain separate.
Legacy constructors retain their existing best-effort behavior.

`Skeleton::try_load(path, config)` runs structural preflight and freezes original GLB, OBJ,
STL or DAE mesh files before returning. Relative references resolve from the
canonical URDF target's directory, including when the URDF is a symlink. Package
references resolve from matching ancestor/sibling package directories; missing
packages are errors. No converted sibling is substituted. Reads are bounded to
16 MiB of XML and 256 MiB of unique mesh bytes; repeated references share bytes.
The declared mesh extension selects the format, while its canonical path identifies
the file. Conflicting format aliases for one file are rejected. File errors return
`UrdfError` without exposing a partial model. Run loading off control-handler
threads; subsequent logging reuses the frozen bytes.

The file loader can verify inline URDF RGBA declarations against used embedded
DAE diffuse effects. Each name must match an effect ID and all four finite color
components must match exactly. Each material-bearing visual must declare the
complete used effect set; different links cannot collectively satisfy it. Visuals
without declarations retain embedded appearance. The proof follows scene geometry,
triangle groups, material bindings and effect references in the same frozen bytes
that are logged. Every user of a shared asset is checked. Bytes are never rewritten.
Missing or unused names, duplicates, changed colors and textures fail.

This path requires COLLADA 1.4.1, metre units and identity material-symbol-to-ID
bindings to match the native decoder. Other formats cannot verify URDF colors.
Limits are 4096 URDF declarations and 4096 nodes per DAE scene. Require one visual
scene and one top-level `scene/instance_visual_scene` selecting it; ambiguous or
unresolved selections fail. Multiple scene definitions are rejected because the
native decoder renders all definitions, tracked in
[native scene-selection limitation](https://github.com/cerulion-inc/cerulion-studio/issues/125).
Embedded reflectivity and refraction metadata do not establish matching shading.

Loading does not install a model into vizd or verify GPU decoding. OBJ material
libraries are ignored by the renderer; DAE support covers triangles and diffuse
materials without textures. This loader does not resolve resources inside mesh
formats. Visible geometry and appearance require separate verification.

- `CoordinateFrame:frame` relocates only that entity's own visualizer DATA;
  `Transform3D:parent_frame` is the component the transform resolver actually
  walks for frame-chain re-parenting. They are distinct component identifiers,
  so tests for anything rendered or resolved downstream must assert the
  RESOLVED composition or exact component values. A chunk-presence assertion
  passes whether or not the transform resolved, so it pins nothing. See
  `coordinate_frame_test.rs`.
- A `Transform3D` logged at an entity poses that entity's OWN geometry: log
  the pose as one `from_translation_rotation` call (logging the archetype
  fully resets the transform relationship), keep geometry in marker-local
  coordinates, and write no hand-rolled quaternion math.

### Clear semantics and MarkerArray

- A rerun `Clear` is a timestamped log event under latest-at semantics:
  "before/after" is in TIME, and same-timestamp clear-vs-add resolution is
  unspecified by rerun's docs. The sink removes the case by construction:
  subtract the frame's re-adds from the clear set, and clear only at leaf
  marker entities. Pinned by `marker_array_test`'s
  `a_deleteall_clears_every_live_entity_but_not_what_the_same_frame_re_adds`.
  Clears are also ineffective under visible-time-range queries (a rerun
  property, not fixable sink-side).
- A `MarkerArray` is a stateful mutation stream: markers persist until
  DELETE/DELETEALL, and incremental one-marker updates are a documented rviz
  idiom. Therefore auto-clearing absent markers is WRONG; frame coalescing
  must stay OFF for this kind (a coalesced-away frame may carry the only
  DELETE, a permanent ghost); and per-input `marker_live` state resets on
  viewer reconnect (pinned by
  `a_reconnect_resets_the_live_marker_set_so_a_later_deleteall_clears_nothing`).
- Markers render partially by design: a MarkerArray is a bag of independent
  objects, so the sink renders what decodes and reports the rest, unlike the
  all-or-nothing rule for a single message's fields.

### URDF numeric geometry

`skeleton.rs` rejects explicit joint/visual origin, axis, and mesh-scale vectors
unless they contain exactly three finite numbers that remain finite as `f32` at
the rendering boundary. `UrdfError::InvalidVector` identifies the XML element,
attribute, source line, and rejected value. Defaults apply only to absent
attributes: identity origins, unit mesh scale, and X for a motion axis.
Fixed joints ignore their axis. This validation does not provide a production
model-import path.

### Entity paths

- House rule: sanitize-then-plain-string. Entity strings are a contract with
  downstream controllers, and rerun's `&str → EntityPath` conversion runs
  `parse_forgiving` (escapes and reinterprets), so paths are sanitized on our
  side and logged as plain strings, never trusted to rerun's parser.
- The stable sanitizer appends a 4-hex FNV suffix whenever sanitization
  changed the string, so distinct raw namespaces cannot collide. Marker ids
  are CONSTRUCTED (`id_7`, `id_n3`), never sanitized; a sanitized `-3` would
  collide with `3`. A leading `__` is neutralized (rerun's reserved
  namespace). Pinned by `entity_path_collision_test.rs`.

### Instance caps

- `MAX_ELEMENT_INSTANCES` (archetype.rs) caps decoded-array instances logged
  into ONE archetype; its measurement harness is `element_cap_bench.rs`
  (`#[ignore]`d measurement arms).
- `MAX_MARKER_INSTANCES` is a PER-ENTITY cap that deliberately does NOT reuse
  `MAX_ELEMENT_INSTANCES`: N markers are N entities × ~2 log calls each, an
  axis the element-cap measurement does not cover. The vertex budget skips
  WHOLE markers, never truncates mid-geometry.
- Upstream traps: do not call rerun `Mesh3D::sanity_check()`; its no-indices
  branch rejects valid meshes (an upstream modulus typo). rerun 0.34 has no
  3D-anchored text archetype (TEXT_VIEW_FACING degrades to a zero-radius
  labelled point); most 3D shape archetypes expose only `from_*` constructors;
  without the glam feature, build everything from `[f32; N]` arrays.

### Decoded arrays and opaque-by-design encodings

- The render ladder consumes the frame walker's decoded canonical element
  arrays: a `Path` draws as a `LineStrips3D` polyline plus `Points3D`
  waypoints, a `PoseArray`/`GridCells` as `Points3D`, a `Detection3DArray` as
  N-instance `Boxes3D`, instead of a text dump.
- Two bespoke encodings are pinned OPAQUE by design and are NOT canonical
  element framing: the tf-source transforms blob and PointCloud2's packed
  point-fields. The walker's strict validation fails both to
  `NestedArrayOpaque` (the loud text fallback, never a guessed decode), and
  their consumers read the raw `*_bytes()` accessors directly. Any future
  encoding unification must treat these as a third convention, not a bug.

### OpenH264 runtime fetch (`openh264_fetch.rs`)

- Legal shape (do not "optimize this away"): Cisco's AVC patent grant covers
  only binaries CISCO distributes. Bundling the codec blob into an installer
  would make this project the distributor and void the grant, so the decoder
  is fetched at runtime from Cisco's CDN, never vendored (the same reason
  Firefox fetches its OpenH264 plugin at first run).
- The CDN serves bzip2-compressed blobs only; an uncompressed request is
  refused.
- TWO digest checks, not one: the loader (`OpenH264API::from_blob_path`) only
  asks "is this SOME Cisco release", so a wrong-architecture blob passes it.
  The fetcher pins per-platform sha256 digests (`CISCO_BLOB_SHA256`, taken
  from `openh264-sys2`'s own hash list, the exact set the loader enforces)
  and verifies BEFORE the cache write; a mismatch writes nothing at all.
- Knobs: cache under `~/.cerulion/openh264/`; `CERULION_OPENH264_BLOB` points
  at an existing copy (skips the fetch); `CERULION_OPENH264_FETCH=off` is the
  kill switch.
- CI runs the fetcher suite hermetically: the test build's feature selection
  turns the fetch off, so nothing opens a socket; the live CDN round trip is
  `#[ignore]`d and run by hand.

## 4. CI lanes

The viz-tests job in `.github/workflows/ci.yml` runs the whole viz tree on
Linux, split across a `viz` lane and a `vizd` lane. The macOS coverage runs the
same steps as `test-macos` shard 1. macOS coverage is load-bearing, not
symmetry: vizd is the desk daemon (macOS is its primary deployment), flakes in
this area have been macOS-only, and the Unix-socket / `#![cfg(unix)]` paths
differ per platform.

Default-cover rule: every step names a PACKAGE, never a test file, so a test
file added tomorrow runs automatically, the property that makes the job a
gate rather than a hand-maintained list that goes stale.

### The two lanes: parallel vs serial

Cargo runs test BINARIES sequentially, so cross-binary hazards (the iceoryx2
SHM singleton, the process-global rerun stream, tracing capture) are handled by
cargo itself. The remaining hazard is WITHIN one binary, and the two packages
differ there. Each package's lane below follows from the global state its own
test files touch:

- `cargo test -p cerulion_viz` runs PARALLEL, deliberately: it is the
  STRONGER gate, because it exercises the isolation the files claim in their
  own headers. Every real-iceoryx2 file uses an isolated per-test SHM root
  (`TransportManager::init_for_test`), and every process-global toucher
  confines itself by one of FIVE mechanisms. A new test that fits none of them
  belongs in the serial lane instead:
  1. a single-`#[test]` binary: the only two callers of `stream::set_stream`
     (which installs the PROCESS-GLOBAL rerun stream) and the TCP-port binder;
  2. a file-local mutex: `coordinate_frame_test`'s rerun lock,
     `sink_dispatch_test`'s skeleton-statics lock;
  3. the global is unreachable from the tests that RUN:
     `element_cap_bench`'s `#[global_allocator]` counter is armed only by its
     `#[ignore]`d arms;
  4. the sibling test touches no globals;
  5. the crate-level `#[cfg(test)]` lock, the lib unit-test binary's
     mechanism: every test that can reach the blueprint statics takes
     `test_support::blueprint_statics_guard()` as its FIRST statement (first
     statement ⇒ dropped last ⇒ still held while a worker's `Drop` joins its
     spawned thread).
- `cargo test -p cerulion_vizd -- --test-threads=1` runs SERIAL for three
  reasons: (a) `vizd_e2e_test.rs` is intra-binary parallel-unsafe without a
  blueprint mutex most of its tests must remember to take, a discipline a new
  test can silently omit; (b)
  `host_test.rs` mutates process env (`CERULION_RERUN_URL`), visible to every
  thread; (c) `live_backlog_test.rs` streams megabytes through a real
  proxy and its byte-occupancy oracle cannot share the process.
- `cargo test -p go2_tf`: pure codec, no globals, parallel.
- Extra steps: the OpenH264 fetcher suite (hermetic; the wiring pin runs in
  its own binary because it asserts a process-global consultation counter
  starts at zero), and a serial re-run of `video_decode_test` (two arms drive
  the process-global decoder-cache generation, and the decode arms share a
  rerun stream).

The job keeps its own cargo cache namespace: it builds under a different
profile than the other jobs, and its rerun-linking test binaries stay out of
the archive every other job pays to restore.

The sibling rerun-leanness job enforces the build boundary from §1, with
reverse-dependency probes that fail loudly if the probe itself goes stale.

### Assertion discipline (each rule bought by a real flake in these suites)

- Time only the operation under test; never let a latency window contain
  tracing-capture setup, a schema-corpus parse, or a handshake.
- Rate/Hz: assert an absolute CEILING plus a ratio against the rate the
  publisher ACTUALLY achieved, never a band around a nominal. A band really
  asserts runner health (load only pushes measured rates DOWN); ceiling+ratio
  is stricter and load-immune, and it catches the decimation bug a band admits.
- Never assert a wall bound in units of a poll interval: macOS CI executes at
  background QoS, where timer coalescing charges slack PER WAKEUP. Bound a
  CONDITION in whole seconds instead; reproduce locally with `taskpolicy -b`.
- Exact-value frame oracles use per-frame lockstep (at most one frame in
  flight), never publish-N-then-drain; a queue's shape across an arbitrary
  drive/drain interleaving is not a property to depend on. Fix a flaky oracle
  by strengthening the stimulus, never by loosening the assertion.
- Cached surfaces (attribution, provenance) are polled to a deadline, never
  read once.

## 5. Test map

### `crates/cerulion_viz/lib/cerulion_viz` (lane: parallel)

| Test file | What it pins | Serial? | Prereqs |
|---|---|---|---|
| `archetype_inference_test.rs` | `log_pose`'s spatial ladder drawing the point rung, transport-free (the shape-inference render arms live in `sink_dispatch_test.rs`; the map is in this file's module doc) | no | none |
| `archetype_memory_test.rs` | walker → archetype builder → rerun memory-sink ingest | no | none |
| `decode_us_test.rs` | render-log decode timing reports THIS frame, never a stale or foreign one | no | none |
| `config_test.rs` | `VizConfig` parametrization oracles | no | none |
| `coordinate_frame_test.rs` | posing by coordinate FRAME: resolved composition, not chunk presence | file-local lock | none |
| `drop_latch_log_test.rs` | never-block drop-latch log discipline over a real blocking sink | no | none |
| `dump_companion_test.rs` | dump companion refused on live `RenderProof`, per archetype; returns on degradation | no | none |
| `element_cap_bench.rs` | the measurement behind `MAX_ELEMENT_INSTANCES` | `#[ignore]`d, by hand | none |
| `entity_path_collision_test.rs` | entity path is a function of topic identity; sanitizer collision rules | no | none |
| `layout_compose_liveness_test.rs` | compose compiler prefers LIVE topics (hand oracles) | no | none |
| `layout_compose_test.rs` | deterministic layout compiler | no | none |
| `layout_default_test.rs` | dynamic consolidated default layout + wall-clock default timeline | no | none |
| `layout_mapping_test.rs` | deterministic layout mapping | no | none |
| `marker_array_test.rs` | MarkerArray kinds + DELETE/DELETEALL clear semantics (incl. `*` arms) | no | none |
| `never_block_grpc_tcp_test.rs` | THE never-block pin over the real gRPC backpressure shape | single-test binary | none |
| `openh264_fetch_wiring_test.rs` | the decode path really consults the fetcher when a decoder is missing | own binary (global counter) | none |
| `openh264_live_fetch_test.rs` | the live fetch against Cisco's CDN | `#[ignore]`d, network, by hand | none |
| `reconnect_test.rs` | worker live-reconnect orchestration (the rerun client does not auto-reconnect) | no | none |
| `sink_dispatch_test.rs` | `dispatch_frame`: walk-by-hash → archetype table → memory sink | file-local lock | none |
| `tap_manager_test.rs` | `TapManager` attach/detach + drain→dispatch over real iceoryx2; wake-listener arms | no (per-test SHM roots) | none |
| `tf_drain_all_test.rs` | `/tf` accumulate-all drain; a latest-only read would drop sibling transforms | single-test binary | none |
| `tf_memory_test.rs` | TFMessage → rerun mapping, transport-free | no | none |
| `tf_source_e2e_test.rs` | producer → tf-sink → rerun memory sink over a real graph + iceoryx2 | single-test binary | none |
| `topic_view_identity_test.rs` | a per-topic view is titled by its topic and shows only that topic's data | no | none |
| `unknown_hash_diagnostic_test.rs` | the unknown-schema-hash diagnostic at the production render site | no | none |
| `video_decode_test.rs` | desk-side H.264 decode + latest-frame presentation | CI re-runs it serial | none |
| `video_h264_test.rs` | H.264 classification, SPS-keyed rendition demux, keyframe gate, VideoStream arm | no | none |
| `video_layout_test.rs` | an interleaved H.264 topic gets ONE spatial2d view (its default rendition) in both layout producers | no | none |

Roughly half the lane's tests live in the lib's own `#[cfg(test)]` modules
(the same modules that DEFINE the crate's process-globals) and confine
themselves via mechanism 5 above.

### `crates/cerulion_viz/bin/cerulion_vizd` (lane: serial, `--test-threads=1`)

| Test file | What it pins | Serial? | Prereqs |
|---|---|---|---|
| `catalog_events_e2e_test.rs` | event-driven sidebar: subscribe → upstream catalog change → unprompted push, over the real control socket | lane | none |
| `live_backlog_test.rs` | HARD GATE: the hosted proxy's live path never buffers image-class frames behind a slow viewer | lane | none |
| `cli_args_test.rs` | real-binary argv handling (`CARGO_BIN_EXE`) | lane | none |
| `convergence_adoption_test.rs` | STRUCTURAL: no control handler waits/polls (whole-`src/` walk); seam-adoption guards invisible to hermetic e2e | lane | none |
| `host_test.rs` | the daemon's rerun-endpoint hosting; mutates process env | lane | none |
| `live_only_history_test.rs` | HARD GATE: a fresh viewer gets the scene skeleton, zero temporal replay | lane | none |
| `poll_period_test.rs` | the drain loop's period: observable, then paced | lane | none |
| `vizd_e2e_test.rs` | end-to-end daemon acceptance: attach/list/status/detach, attribution, `*` reflow arms | lane | none |
| `wake_drain_e2e_test.rs` | the drain loop blocks on the tap's wake listener (remote AND local production shapes) | lane | none |

New tests in `vizd_e2e_test.rs` that can touch blueprint statics must take the
file's blueprint-statics guard as their first statement; the serial lane hides
an omission until someone runs the binary parallel locally, so copy an existing
test's opening lines.

### `crates/cerulion_viz/lib/go2_tf`

In-module unit tests only (`src/lib.rs`): pure `TFMessage` codec, no transport,
no rerun, no globals.
