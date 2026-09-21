//! `cer_bench_msgs` — the fixed-POD bench message crate
//! (type-class axis, METHODOLOGY §18).
//!
//! The `PodPayload` type is GENERATED at build time from
//! `CER_BENCH_POD_BYTES` (see `../build.rs` → `../pod_codegen.rs`): 12
//! fixed bytes (`prep` + `stamp_hi` + `stamp_lo`) followed by a baked
//! `uint8` array, so the message totals exactly that many bytes — the
//! matched-quantity rule.
//!
//! The crate NAME is load-bearing. `cerulion_cli_engine`'s node-metadata
//! import map keeps a genuine crate path as the schema package, so a node
//! writing `use cer_bench_msgs::PodPayload;` reports
//! `cer_bench_msgs/PodPayload` — the same qualified name the graph YAML
//! declares, the `.msg` store registers, and `parse_rosmsg` folds into the
//! wire `SCHEMA_HASH`. Renaming the crate silently changes all four.
#![allow(dead_code, unused_imports, clippy::all)]

include!(concat!(env!("OUT_DIR"), "/pod_payload.rs"));
