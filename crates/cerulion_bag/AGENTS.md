# cerulion_bag - agent notes

The recording-bag format crate: a hand-rolled, chunk-buffering standard-MCAP
writer + reader (`.mcap`, never JSONL). Format contract only - recorder
scheduling (drain, staging, chunk timing) lives in `cerulion_bagd`.

## Invariants

- ZERO wall-clock reads in this crate - the byte-determinism gate. Every
  timestamp comes from the caller; any time-based policy injects its clock
  from the caller (the chunk time floor lives in bagd, not here).
- Two identical input sequences produce byte-identical files: the CONSTRUCTION
  set's channel ids are assigned by sorted topic name and schema ids by sorted
  schema identity; a topic added later (`register_topic`) takes the next id, in
  registration order. Every id and record position is a function of the call
  sequence. Gates: `bag_determinism_test`, `bag_byte_pin_test`,
  `bag_late_channel_test`. It is NOT a claim that two RECORDER runs match.
- The writer closes a chunk on its own ONLY at the size threshold
  (`DEFAULT_CHUNK_MAX_BYTES`); `flush_chunk` is the caller's boundary. Never
  add a timer here.
- `write_message` COPIES payloads into the chunk arena - the caller's buffer
  is free the instant the call returns. Deliberate, not a zero-copy
  regression: pointer-stashing forces the recorder to hold SHM borrows until
  `writev` completes, which foreign-provisioned topics cannot afford.
- The channel set is NOT frozen at bag creation: `register_topic`
  emits a topic's Schema + Channel records as TOP-LEVEL data-section records at
  the current position, without closing the open chunk, so a producer that
  appears mid-run is recorded from that moment. Top-level, never in the arena -
  a pending chunk can be discarded, and the tables must never name a channel the
  data section did not receive. Tables commit only AFTER the bytes are durable.
- A failed write is TERMINAL: the writer latches `BagError::Poisoned` and
  refuses every later call. The ONE exemption is a chunk `writev` that made zero
  progress (the file is untouched, and the recorder's salvage retry needs it);
  the cold path has no exemption because `write_all` cannot report its progress.
- Torn-tail crash model: a truncated bag recovers every complete chunk; a
  corrupted chunk body is DETECTED, never silently returned.
- `__cerulion/` is the reserved topic prefix (scheduler trace, nondeterminism
  markers, attachments); user topics under it are rejected.
- Every schema entry carries `hash_recipe` (u32) = `HASH_RECIPE` from
  `cerulion_core::trace::bag` - bump it there, never here.

## Testing

- `cargo test -p cerulion_bag` - parallel-safe (pure file I/O, per-test
  unique temp paths, no process globals).
- Round-trip and crash tests re-read every bag with the upstream `mcap`
  crate - an independent oracle, never our own reader alone.
- `bag_writer_syscall_test` drives the `writev` loop through a scripted sink:
  IOV_MAX batching, partial-write resume, EINTR retry, no-progress hard error.

## Gotchas

- "Zero-copy" in older prose describes the recorder's drain/tap side; the
  WRITE path copies by design (see above). Do not "optimize" it back.
- The reader's completeness pre-scan touches every page of the bag; it
  batches advise-behind to bound RSS - read `advise_completeness_test`
  before touching the scan.

Deep reference: docs/internals/recording.md - read before changing the
writer, the chunk model, or any coverage/health field.
