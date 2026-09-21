# PodPayload — the fixed-POD wire type of the type-class axis
# (METHODOLOGY §18), REGISTERED here so the workspace resolver can see it.
#
# WHY THIS FILE EXISTS. The Rust type is generated per sweep size by
# pod_schema/pod_codegen.rs into the cer_bench_msgs crate's OUT_DIR, which
# the CLI cannot see, and `graph validate` — which `graph run` runs
# fail-closed — refuses a `schema:` naming nothing the workspace, the .msg
# store or the built-in ROS 2 corpus can resolve. Measured with no
# registration at all: both pod graphs refused to start. This file is that
# registration: it makes the NAME `cer_bench_msgs/PodPayload` resolve.
#
# WHY THE ARRAY LENGTH DOES NOT MOVE. `data` is
# CER_BENCH_POD_BYTES - 12 bytes (12 = prep + stamp_hi + stamp_lo), so the
# layout — and therefore the hash — really is per sweep size, and the
# committed length below describes the FIRST pinned size only. That used to
# matter to more than validation: the RECORDER took a bag channel's
# `wire_fixed_size` from this file, so run_workspace.sh had to regenerate
# it before each rebuild and restore it from an EXIT/INT/TERM trap. A later
# change moved that resolution onto the producing node's own metadata
# (`OutputMeta::wire_fixed_size`, beside the wire hash the recorder has
# always taken from the node), with this file kept only as the cross-check
# — so a recording now carries what the node publishes at every size, and
# no sweep writes a tracked file.
#
# WHAT YOU WILL SEE IN A SWEEP LOG, and which warn is which. Two different
# warns exist and only one of them fires here:
# * the layout cross-check ("describe DIFFERENT wire layouts")
#     lives in the recorder and fires only under `graph run --record`.
#     run_workspace.sh does not record, so a sweep never sees it.
#   * the PRE-EXISTING schema-hash divergence warn (cerulion_core's
#     `warn_on_schema_hash_divergence`, "schema-hash mismatch") fires on
#     every `graph run` whose YAML `schema:` resolves to a different hash
#     than the node's. That one is NEW to sweep logs at every size but the
#     committed one, because the per-size rewrite used to keep the two in
#     lock-step. It is expected and harmless: it reports that the YAML
#     `schema:`'s max_slice_len override has no effect, and a fixed schema
#     has no variable field to bound. No bench gate reads it.
#
# WHY THE .msg STORE AND NOT A schemas/*.yaml. Two measurements, both on a
# CLI built from this tree:
#   * HASH. The store registers under the package its DIRECTORY names, so
#     `cerulion schema info cer_bench_msgs/PodPayload` reports
#     0xf99555aa42af17bc at 64 B — byte-for-byte the generated crate's
#     PODPAYLOAD_SCHEMA_HASH at that size. A bare workspace-YAML
#     registration reported 0xb981270238cd0358: the same fields under no
#     package hash differently, so nothing could resolve the recorded hash
#     to a name.
#   * SIZE. The YAML parser refuses a FixedArray longer than 1048576
#     (cerulion_cli_engine::schema_cmd::MAX_FIXED_ARRAY_LEN), so the 16 MiB
#     sweep point — uint8[16777204] — could not be registered as YAML at
#     all. The store parses it through parse_rosmsg, which has no such cap.
#
# DO NOT hand-edit the field list: check_percentile_parity.py pins the
# committed array length against the first pinned sweep size, and pins that
# run_workspace.sh has no writer for this file.
uint32 prep
uint32 stamp_hi
uint32 stamp_lo
uint8[52] data
