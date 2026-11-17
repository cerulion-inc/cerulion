# examples: agent notes

CLI workspaces: `basic_timer`, `obstacle_avoidance`, `camera_pipeline`, `perception`,
`v4l2_camera`, `realsense`, `aspire_arm`, `go2`; each has its own manifest/lockfile.
`moveit_hero` is the `rmw_cerulion` Docker gate.

## Shipped surface (gate: `check_public_surface.sh`)

- An example is ONE workspace: one node type per `nodes/<type>/` crate (macro form),
  `graphs/<name>.yaml`, a README that builds, validates, runs, records and replay-verifies
  via the verbs. Multi-node files, in-code graphs, hand-driven runtimes: `tests/` only.
- No README figure without a shipped package behind it; no tracker ids, no typographic dashes;
  never bulk-rewrite the inside of a string literal (fix each by hand, run its tests); a
  removal is a maintainer ruling; name only what exists. Text describes the example to its
  user, never how it was built (class `work-state`): a README reads zero; a safety fact
  stays, reworded ("Experimental").

## Invariants

- PUBLIC user API only: the `#[cerulion_node]` macro, graph/schema YAML and the
  `cerulion` CLI (the `docs/user-api.md` surface; read it before touching a node). An
  import of internals (`cerulion_core::...` beyond the prelude) needs a comment at the use
  site saying why the public surface cannot express it. The one exception is the
  protocol bridge (`examples/go2/nodes/dds_bridge` speaks CDR/DDS).
- Go2 runbooks match shipped CLI help. Paired discovery needs account/pairing,
  no LISTEN/address setup; keep egress/ingress scope. Label zenoh locators as open
  LAN compatibility. Retain vendor DDS interface and hardware safety requirements.
- Same rules as `benches/`: a harness that RUNS an example spawns `cerulion graph run` as a
  SUBPROCESS (statically-linked transport singletons do not rendezvous in-process); a node
  crate's own `tests/` may build a runtime in-process over an isolated transport. Exact-pin
  iceoryx2 in the lockfile (a patch skew is a silent zero-data hang); build `--release` for
  anything latency-visible.
- No fake data (root rule): output is real robot or replay data; synthetic input is generated
  live and labeled.

## Gotchas

- Version-gated cargo features on gstreamer crates must match the OLDEST supported robot OS, not
  your machine: `gstreamer-sys 0.24` has a base floor of GStreamer 1.14 and `v1_20` raises it to
  1.20, but older embedded robot images ship 1.16.x (`examples/go2/nodes/camera_jpeg/Cargo.toml`).
- The vendored `Go2FrontVideoData.msg` is what the firmware ACTUALLY publishes (3 fields:
  timestamp, height, H.264 Annex-B access unit); the vendor's 4-field definition disagrees with
  its own firmware, so field names are placeholders. Contract: the `h264.rs` docs in
  `examples/go2/nodes/camera_jpeg/src/` + the `.msg` in `examples/go2/schemas/`. Never "fix"
  the schema to the vendor docs: decoding stops.

## Publication checks
- Go2 DDS clouds publish complete payloads or count drops; geometry stays consistent. Use
  size-aware setters for variable payloads (`bridge_publish_test`). Raw routes retain local
  scope through publisher lifetime and skip private egress registration.
- Go2 TF fixed geometry and encoder buffers are built at construction and reused each tick;
  encoding stays in `go2_tf`, never in node-local byte offsets. A codec boundary may need
  owned memory: document each application copy apart from local SHM delivery.
- Generated images fill payload loans directly: width, height, step, encoding and data
  length agree, every loaned byte is initialized, advertised outputs reach a subscriber. Label
  generated images and teaching detectors as synthetic; the perception twin keeps
  byte-identical `algorithm.rs`/`tests.rs` copies, differing from the detector in one constant.
- `v4l2_camera` and `realsense` are driver workspaces: hardware precondition first in the
  README, hardware-free tests, never capture without hardware; v4l2 requeues a dequeued
  frame even when publication fails; realsense keeps the SDK behind a non-default feature.
