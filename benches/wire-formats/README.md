# Wire-format microbenchmark

A single-threaded microbenchmark of Cerulion's generated message types
against FlatBuffers, Protobuf (prost) and Cap'n Proto, on the same logical
content. It measures what each format costs to write one message into a
buffer and to read a few fields back out of bytes already in memory, and how
many bytes each message takes.

**There is no transport here.** No iceoryx2 loan, publish, notify or
receive, no WaitSet wake-up, no second process and no network. Those
dominate real message latency; the round-trip suite in
[`../latency/`](../latency/README.md) measures them. This bench isolates the
serialization layer only.

## What is measured

Three messages, with identical content in every format (`src/lib.rs`,
"Shared logical content"):

| | message |
|---|---|
| (a) | IMU sample: `u64 stamp_ns, u32 seq, u32 status, f64[4] orientation, f64[3] angular_velocity, f64[3] linear_acceleration` (96 B, all fixed size). |
| (b) | `sensor_msgs/Image` with a header and a 1 MiB (`mono8`, 1024x1024) or 6 MiB (`rgb8`, 2048x1024) payload. |
| (c) | `sensor_msgs/PointCloud2` with a header, four `PointField`s and a 4 MiB payload. |

Payloads are deterministic pseudo-random bytes, identical across formats.

- **Encode** (`encode/*`): one complete message written into a slot that was
  allocated once, the way a publisher writes into a shared-memory slot. Every
  format copies the payload exactly once. Per-message work a format needs to
  reuse the slot is inside the timed loop: FlatBuffers' `reset()` memset and
  Cap'n Proto's re-zeroing of the used words. Cap'n Proto builds its one
  segment in the slot right after an 8-byte segment table, which the encode
  also writes, so the slot holds the complete flat message; `check` decodes
  those bytes before the builder's drop re-zeroes them.
- **Read** (`read/*`): three scalar fields, plus the payload's length and
  first byte for (b) and (c), from bytes already in memory. Variants:
  - `cerulion_typed`: the receive-side gate (header, schema hash, size and
    alignment checks, restated from the transport) and the generated accessors;
  - `cerulion_walker`: `FrameWalker`, Cerulion's full-frame decoder for
    untrusted input;
  - `flatbuffers_verify` / `flatbuffers_noverify`: `root` with the default
    verifier, and `root_unchecked`;
  - `capnp`: `read_message_from_flat_slice_no_alloc`, which checks pointers as
    they are followed (there is no separate verify-off path);
  - `prost_vec` / `prost_bytes`: a full decode with `Vec<u8>` fields (copies the
    payload) and with `bytes::Bytes` fields (slices the input).
- **Bytes on the wire**: printed by the `check` binary. The Cerulion frame
  includes its 40-byte header; the other formats carry no transport metadata.

The `check` binary also asserts that every read path returns the values
that were written, so a fast row cannot be a wrong row.

## Running it

From this directory, with a stable Rust toolchain. Nothing else is needed:
no `flatc`, `capnp` or `protoc` (see "Generated code").

```bash
cargo run --release --locked --bin check          # correctness + bytes-on-the-wire table
cargo bench --locked --bench wire -- --noplot     # the full criterion run (about 6 min)
cargo bench --locked --bench wire -- --noplot \
  --sample-size 10 --warm-up-time 0.2 --measurement-time 0.5   # a quick pass (under a minute)
cargo run --release --locked --bin flatbuffers_dirty_slot  # FlatBuffers over a reused slot
```

`check` also probes where the nested `Header` of each Image lands. With the
1 MiB `mono8` image it lands misaligned, and the probe prints the caught panic
from `HeaderShm::from_bytes` (cerulion-inc/cerulion#530). That message is
expected; the run passed if it ends with `all reads returned the source
values: OK`.

The default configuration is 100 samples, a 2 s warm-up and 5 s of
measurement per benchmark. Criterion writes its estimates under
`target/criterion/`. The figure to compare is the median per-iteration time
with its 95% confidence interval.

This directory is its own cargo workspace with a committed `Cargo.lock`. It
is not a member of the root workspace, so a root build never pulls in
flatbuffers, prost, capnp or criterion. Cerulion's crates are path
dependencies on this checkout, so the bench measures the tree it sits in.

## Reading the numbers

- **Machine dependent.** Absolute times depend on the CPU, its cache sizes,
  memory bandwidth and the compiler. Compare formats within one run on one
  machine; do not compare a number from one machine with a number from
  another. Run it twice and look at the run-to-run spread before trusting a
  gap of a few percent.
- **Warm caches.** The same source payload and slot are reused every
  iteration. A real ring hands out a different slot each time, and at 6 MiB
  the source plus the slot approach the size of a large L2.
- **Large-payload encode is dominated by the one payload copy** every format
  makes, so differences there are small by construction.
- **The verification models differ.** FlatBuffers verify checks the whole
  buffer's structure, Cap'n Proto checks each pointer on access, and
  Cerulion's typed path checks the header, the schema hash and per-accessor
  bounds only. The `verify` and `typed` rows are not a like-for-like safety
  comparison; `cerulion_walker` is the closest Cerulion analogue to a full
  verify.
- **Schema choices.** Cap'n Proto and Protobuf have no fixed-size arrays, so
  the IMU's arrays are flattened into scalars, which is each format's fast
  layout. A repeated field or list would be slower.
- **No tail latency.** Criterion times batches of iterations, so it reports
  per-batch means; there is no per-message p99 here.
- **Same build settings for every format**: the release profile, no LTO, no
  `target-cpu=native`, no PGO.
- **The Cerulion encode and receive gate are restatements** of the
  publisher's loan path and the typed receive check, without iceoryx2. If the
  transport's gate changes, `cer_gate` in `src/lib.rs` has to follow it.

## Generated code

- **Cerulion**: the Rust type for `msg/ImuSample.msg` is generated at build time by `build.rs`,
  through the same `cerulion_core::codegen` steps
  `crates/native_ros2_messages/build.rs` uses. `Image` and `PointCloud2` come
  from `native_ros2_messages`.
- **FlatBuffers and Cap'n Proto** are vendored in `src/`, so a build needs no
  schema compiler and every run compiles the same code. They are never edited
  by hand and are excluded from rustfmt. Regenerate after a schema change, with
  the generator versions matching the pinned runtime crates:

  ```bash
  flatc --rust -o src schema/wire.fbs     # flatc 25.12.19 -> src/wire_generated.rs
  cargo install capnpc --version 0.27.0   # provides the capnpc-rust plugin
  capnp compile -orust:src --src-prefix=schema schema/wire.capnp   # capnp 1.5.0 -> src/wire_capnp.rs
  ```

- **Protobuf**: `src/pb.rs` holds the `#[derive(prost::Message)]` structs that
  prost-build emits for `schema/wire.proto`, written out by hand so the build
  needs no `protoc`. prost-build only writes these derives, so the runtime code
  is the same.

## Licences

The bench code, the schemas and the hand-written prost structs in `src/pb.rs`
are under the repository's licence, AGPL-3.0-only. `src/wire_generated.rs` and
`src/wire_capnp.rs` are generator output for this directory's schemas and carry
no licence header of their own. The tools and runtime crates they are generated
for and linked against are licensed as follows:

| component | licence |
|---|---|
| FlatBuffers (`flatc`, the `flatbuffers` crate) | Apache-2.0 |
| Cap'n Proto (`capnp` tool, the `capnp` and `capnpc` crates) | MIT |
| prost, prost-derive | Apache-2.0 |
| bytes | MIT |
| criterion (dev-dependency) | Apache-2.0 OR MIT |

## Layout

| path | contents |
|---|---|
| `src/lib.rs` | every encode and read path, and the shared content |
| `benches/wire.rs` | the criterion benchmark |
| `src/bin/check.rs` | correctness check and the bytes-on-the-wire table |
| `src/bin/flatbuffers_dirty_slot.rs` | a FlatBuffers build over a slot that still holds an earlier message fails its own verifier, which is why the encode loop pays `reset()` |
| `schema/`, `msg/` | the FlatBuffers, Cap'n Proto, proto3 and Cerulion schemas |
