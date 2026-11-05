# cerulion_rcpd - agent notes

The Robot Context Protocol daemon: an MCP server (stdio, bearer-token streamable
HTTP) serving `robot://graph`, `robot://state/{name}` and the manifest's tools.
User reference: docs/rcp.md.

## Invariants

- Deny by default: the callable tools are the manifest's `tools`, `estop`, and
  `estop_clear` under `limits.estop_clear: agent`. Any other name is receipted
  `denied` and never reaches the graph. Built-in names cannot be declared.
- Every call that passes lookup is receipted in the `cerud` hash chain; a
  `reversible` or `irreversible` call writes its `intent` receipt (synced)
  before it is published, and is refused while the e-stop latch is closed.
- The latch is an `AtomicBool` shared with the journal thread, which re-checks
  it immediately before publishing a latch-gated frame. Its file is
  `<receipts>.estop`; a daemon that finds the file starts latched.
- `estop` closes the latch before anything that can await and publishes on the
  journal's urgent queue; it never waits on `call_lock`.
- `estop_clear` reads the `engages` counter, then takes `call_lock`; inside
  `latch_io` (a std mutex never held across an await) it refuses
  (`estop_overtaken`) if an `estop` bumped the counter since, else writes
  `intent`, deletes the file and opens the latch; then it journals a
  LATCH-GATED frame, so an `estop` that lands first stays the graph's last
  frame (`estop_overtaken`). `estop` re-closes the latch, writes its file and
  `intent`, and queues its urgent frame inside `latch_io`, so the receipt log
  and the calls topic order engages and clears as they took effect (the
  journal publishes every queued urgent frame before a normal one). An
  undeletable file leaves the latch closed (`clear_failed`); an unreadable
  one at start counts as present.
- The journal's `gate` reads the latch, then drops a latch-gated request whose
  caller dropped its reply (a cancelled MCP request); estop and read frames
  always publish.
- `limits.estop_clear: operator` unlists `estop_clear` and refuses it
  (`estop_clear_disabled`, receipted `denied`). The binary applies the switch
  with `ToolHost::with_estop_clear`; a host built without it serves `agent`.
- `limits.irreversible: refuse` fails the manifest load naming the tool;
  `allow` (default) serves it with `destructiveHint: true`, a
  `(safety class: irreversible)` description suffix and `safety_class` in
  every call result. Receipt lines carry the tool name, not the class.
- stdout carries only the MCP stream in stdio mode; logs go to stderr.
- The Jev eye (`src/eye/`) runs on its own `rcp-eye` thread, never the
  journal thread, and only when the manifest has `eyes.jev`. Its receipts
  take the receipt log's lock, the lock the e-stop's intent receipt takes
  before the e-stop frame is queued, so `EyeCore::intake` (the ask bound,
  `limits::ASK_BURST` / `ASKS_PER_SECOND`) runs before anything else and
  an ask over it gets no answer and no receipt. A failed `EyeHost::spawn`
  is logged and the daemon serves without the eye; a client that cannot be
  built answers `unavailable`. Every check in
  `EyeCore::admit` runs before any network call; every answer frame gets its
  `eye.jev` receipt (caller `rcp-eye`, digest = `request_sha256` or
  `ask_sha256`) BEFORE it is published, and an unreceipted answer is not
  published. `Look` has no `Default` and no `bool` conversion; every outcome
  but `ok` is `Absent`. The key never enters a frame, a receipt, a log or
  the status file; the client sends it only in a sensitive `Authorization`
  header. The v1 limits in `eye/limits.rs` are the numbers in the Jev limits
  table; change them only with it.

## Testing

- `cargo test -p cerulion_rcpd`: every file is parallel-safe. In-process
  tests use per-test SHM roots (`init_for_test`) and unique topics;
  `binary_smoke_test.rs` spawns the binary on the default root, so each of its
  manifests names journal topics unique to the test and process.
- The `test-helpers` feature (on for tests through the self
  dev-dependency) exposes the receipt log's fault seam, a client trusting
  one extra root, and `jpeg_header_for_test`.
- `eye_stub_test.rs` drives `EyeCore` and the HTTP client against a local
  stand-in route (no real key, no network); `eye_tls_test.rs` runs the
  client over TLS (an untrusted root is refused before any request; a
  trusted one speaks HTTP/2 by ALPN) and scans every log line for key
  bytes; `eye_host_test.rs` drives the eye thread through an ask flood, a
  key placed after start (notice before the first request), a receipt that
  cannot be written, and no client; `eye_graph_test.rs` runs the eye
  thread on a private SHM root, records a bag, scans it, the receipts, the
  status file and every log line for key bytes, resims from it and runs
  `--verify-bag`. It installs the process's log subscriber, so it stays one
  test per binary.
- `tools_test.rs` drives the MCP surface against a stand-in graph thread and
  asserts the exact receipt sequence of each scenario; `binary_smoke_test.rs`
  checks the manifest switches reach the shipped binary.
