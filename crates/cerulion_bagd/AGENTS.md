# cerulion_bagd - agent notes

The recorder daemon (`bagd`): attaches listener-less data-only taps + drains
trace rings into ONE finalized `.mcap` bag per run. Green tests do NOT prove
firehose-rate losslessness - the firehose benches are `#[ignore]`d measurements.

## Invariants

- bagd NEVER evicts: both `room == 0` arms in the tap drain WAIT. The loss
  boundary is the topic's own provisioned SHM queue depth, not recorder policy.
- ONE write mode: payloads are copied into the MCAP chunk arena at drain, so
  taps arm at any `subscriber_max_borrowed_samples >= 1`.
- The chunk clock lives HERE (`CHUNK_TIME_FLOOR_MS`): a chunk closes at the
  size cap or the time floor, whichever first. `cerulion_bag` stays clock-free.
- Per-tap staging is bounded (`RECORDING_TAP_STAGING_MAX_BYTES`); hitting it
  STOPS that tap's drain for the pass (counted in
  `TopicHealth::staging_full_passes`) - never blocks other taps, never evicts.
- `dropped_unwritten` has exactly ONE producer: the dead-writer hand-off.
  Keep the loss vocabulary disjoint (see the dossier).
- Coverage is factual: `enumerated` records the OUTCOME of scanning, never
  intent; `is_incomplete()` (not `gap_count()`) escalates the terminal line;
  an `Option` coverage field left `None` means NO CLAIM, never failure.
- The schema resolver uses the NON-waiting netd verbs (`query_*_no_respawn`
  over `connect_existing`) and verifies every served doc by recomputing its
  hash against the wire before naming a channel - a mismatch records
  `Unresolved`, never a wrong name.
- A message emitted from code that cannot know which verb invoked it
  (`cerulion bag record` vs `cerulion bagd`) names NO flag - the two verbs
  spell their schema-wait/settle levers differently.

## Testing

- `cargo test -p cerulion_bagd -- --test-threads=1` - serial for a process-GLOBAL panic
  hook, not SHM (each test builds an isolated transport).
- Rendezvous with a rescan-attached tap via `await_rescan_tap` (condition poll), never
  `sleep(...)` - data-only taps have no back-fill, so earlier frames are simply gone.
- No wall-clock assertion bands: verdicts and frame-set relations only; every wait is a condition under a seconds-scale ceiling.

## Gotchas

- Topic ENUMERATION stays OFF the drive loop (`discovery_scan::DiscoveryScanner` walks on
  its own thread; the loop takes an O(1) snapshot). An inline walk starves the drain and
  loses frames the writer never saw. Both failures are loud: unspawnable degrades to inline
  with a `warn!`, spawned-but-silent rides `ScanSilenceLatch`.
- Live discovery defaults ON only for `--topics-json` and `--run`; OFF for
  `--topic`/`--all`/`--regex` and `BagdConfig::new`. `graph run --record` builds bagd's argv
  itself: retune with `CERULION_RECORD_DISCOVERY=off` / `CERULION_RECORD_DISCOVERY_SETTLE_MS`.
- Even a quiet graph pays the settle floor (`DISCOVERY_SETTLE_MIN`); the
  arm-time scan does NOT count as a quiet scan.
- Bag creation is held for max(discovery settle, schema wait) - measure it from
  `BagdSummary::channel_set_closed_after`, never the bag file's appearance (reads short).
- Both record paths run on the default `iox2_` namespace: discovery also taps
  co-tenant topics; the only bounds are `DISCOVERY_MAX_TAPS` + the env switch.
- MULTI-PROCESS trace rings ride `!no_rings`, not `record` (single-process ones ride
  `--record`); either way read `run.json`/`trace_rings`, never infer one from the run shape.
- Mid-run ring attach MUST use `open_at_live`; plain `open` refuses a lapped
  ring. A departure ring (`rank == u32::MAX`) is passthrough, never head-gated.

Deep reference: docs/internals/recording.md - read before touching drain
pacing, discovery/settle, coverage fields, or run binding.
