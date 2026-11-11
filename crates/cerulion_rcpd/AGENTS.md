# cerulion_rcpd - agent notes

The Robot Context Protocol daemon: an MCP server (stdio, bearer-token streamable
HTTP) serving `robot://graph`, `robot://state/{name}` and the manifest's tools.
User reference: docs/rcp.md.

## Invariants
- Remote graph and state resources, tool journals and external eye requests respect native producer egress scope. Journal admission survives caller cancellation through the pending answer; emergency-stop operations remain available independently.
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
  `latch_io` (a std mutex held across no await and no rcpd log line, only the
  lines cerud logs while rotating) it refuses (`estop_overtaken`) if an `estop`
  bumped the counter since, else writes `intent`, deletes the file and opens
  the latch; then it journals a LATCH-GATED frame, so an `estop` that lands
  first stays the graph's last frame (`estop_overtaken`). `estop` re-closes the
  latch, writes its file and `intent`, and queues its urgent frame inside
  `latch_io`, so the receipt log and the calls topic order engages and clears
  as they took effect (the journal publishes every queued urgent frame before a
  normal one). An undeletable file leaves the latch closed (`clear_failed`); an
  unreadable one at start counts as present.
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
- The Jev eye (`src/eye/`) runs on its own `rcp-eye` thread, only under
  `eyes.jev`; a failed `EyeHost::spawn` leaves the daemon serving. Its receipts
  share the receipt log's lock with the e-stop's intent receipt. An ask from a
  replay node (`eye/source.rs`) is dropped first, then the ask bound
  (`EyeCore::intake`) and `admit` run, all before any receipt or network call;
  each answer is receipted `eye.jev` before it is published. `Look` has no
  `Default` or `bool`; the key is sent only in the `Authorization` header.

## Testing

- `cargo test -p cerulion_rcpd`: every file is parallel-safe. In-process
  tests use per-test SHM roots (`init_for_test`) and unique topics;
  `binary_smoke_test.rs` spawns the binary on the default root, so each of its
  manifests names journal topics unique to the test and process.
- `eye_*_test.rs` run against a local stand-in route (no key, no network);
  `eye_graph_test.rs` installs the log subscriber, so one test per binary.
- `tools_test.rs` drives the MCP surface against a stand-in graph thread and
  asserts the exact receipt sequence of each scenario; `binary_smoke_test.rs`
  checks the manifest switches reach the shipped binary.
