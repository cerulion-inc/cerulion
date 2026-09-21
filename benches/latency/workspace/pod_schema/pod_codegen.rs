// pod_codegen.rs — the ONE shared build-script body for the three
// pod_* node crates (pod_ping_node / pod_pong_node / pod_latency_node),
// pulled in by each crate's two-line `build.rs` via `include!` so the
// schema definition can never drift between producer and consumers
// (identical source ⇒ identical generated `SCHEMA_HASH`, the wire
// identity the graph build validates).
//
// TYPE-CLASS axis (CER_BENCH_MSG=pod — METHODOLOGY § "The type-class
// axis"): the incumbent workspace legs already ride the VARIABLE class
// (sensor_msgs/Image, unbounded data loaned per tick). This generates
// their FIXED-POD twin — `PodPayload`, a purely-fixed Cerulion schema
// mirroring the ROS 2 harness's Pod<N> family:
//
//     uint32 prep        # written FIRST each tick — triggers the
//                        # lazy SHM loan BEFORE the stamp is
//                        # read (G3), mirroring the ROS 2 loan path's
//                        # borrow-before-stamp and the Image legs'
//                        # fixed-field prep writes
//     uint32 stamp_hi    # wall real_ns() split across two u32s —
//     uint32 stamp_lo    # the same packing the Image legs put in
//                        # height/width
//     uint8[N-12] data   # FIXED array, never written (Mode-A fill
//                        # exclusion: the loaned slot holds whatever
//                        # it holds)
//
// Matched-quantity rule: at sweep point N the message's
// WIRE_FIXED_SIZE is exactly N bytes — the same total-bytes convention
// as ROS 2's Pod<N> (uint64 ts_ns + uint8[N-8]). The VARIABLE class
// matches on the unbounded data array's length instead (data = N, the
// type's fixed fields riding as constant overhead) — cross-class
// comparisons within one stack must quote that rule.
//
// The array length is a COMPILE-TIME property (a fixed array cannot be
// resized at runtime — that is the whole class distinction), so the
// size is baked at build time from CER_BENCH_POD_BYTES
// (`cargo:rerun-if-env-changed`; default 64; MUST be a multiple of 4 —
// the repr(C) struct's u32 alignment rounds any other total up, see the
// assert below) and run_workspace.sh rebuilds the three pod crates per
// sweep size (it refuses a misaligned CER_BENCH_PAYLOAD_SIZES override
// before building anything). The mislabel guard is
// NOT a grep: each pod node's `init` compares the baked
// `PODPAYLOAD_WIRE_SIZE` against the runtime CER_BENCH_PAYLOAD_SIZE
// and returns a loud Err on mismatch (the runner's data-flow gate then
// fails the size), so a stale build can never measure one size under
// another's label.

use std::env;
use std::fs;
use std::path::Path;

fn main() {
    // Re-run when the baked size changes or this body changes.
    //
    // The path is relative to the INCLUDING package's root, and since the
    // crate extraction that is `pod_schema/` itself — `pod_schema/build.rs`
    // is the only includer now, because the three `pod_*_node` crates
    // depend on `cer_bench_msgs` as an ordinary crate and no longer carry
    // build scripts of their own. The old `../../pod_schema/…` was written
    // for those build scripts and now resolves to a path that does not
    // exist, which is worse than useless in BOTH directions: emitting any
    // `rerun-if-changed` disables cargo's default whole-package tracking,
    // and a missing path always reads as stale, so the whole
    // pod_schema → cer_bench_msgs → three-cdylib chain rebuilt on every
    // `cargo build` while a real edit to this file was never the reason.
    println!("cargo:rerun-if-env-changed=CER_BENCH_POD_BYTES");
    println!("cargo:rerun-if-changed=pod_codegen.rs");

    // `env::var` collapses two different answers into one Err — UNSET and
    // SET-BUT-NOT-UTF-8 — and the fallback here is 64, itself a pinned
    // sweep size, so the second would bake a 64-byte struct from a value
    // the operator did type. At sweep point 64 the nodes' runtime guard
    // (baked size vs CER_BENCH_PAYLOAD_SIZE) is then satisfied by
    // coincidence. `var_os` keeps the two apart: absent means 64 by
    // documented default; present-but-unreadable is a build failure.
    let raw = match env::var_os("CER_BENCH_POD_BYTES") {
        None => "64".to_string(),
        Some(v) => v.into_string().unwrap_or_else(|bad| {
            panic!(
                "CER_BENCH_POD_BYTES is set but is not valid UTF-8 ({bad:?}) — \
                 refusing to fall back to the default 64, which is itself a \
                 pinned sweep size and would bake a 64-byte payload under \
                 whatever size label the runner gives the .bin"
            )
        }),
    };
    let n: usize = raw.parse().unwrap_or_else(|e| {
        panic!(
            "CER_BENCH_POD_BYTES must be a payload byte count, got '{raw}': {e} \
             (run_workspace.sh exports it per sweep size; unset = 64)"
        )
    });
    // 12 fixed bytes (prep + stamp_hi + stamp_lo) + at least 1 data byte.
    assert!(
        n >= 13,
        "CER_BENCH_POD_BYTES={n} is below the 13-byte floor (12 fixed-field \
         bytes + a non-empty data array); the pinned sweep starts at 64"
    );
    // The generated `PodPayloadShm` is `#[repr(C)]` with three u32 fields,
    // so its size is rounded UP to a multiple of 4: a request of 65 bakes
    // a 68-byte wire struct (measured: size_of = 68, align = 4). That
    // would either mislabel a 68-byte measurement as 65 (the matched-
    // quantity rule says the message totals EXACTLY N — METHODOLOGY §18)
    // or, as the nodes' init guard actually does, fail the size at RUN
    // time after a full rebuild. Refuse HERE, at build time, with the
    // reason: the pod class can only realize multiples of 4 (every pinned
    // sweep size is one). Rounding is deliberately NOT offered — a
    // rounded size is a mislabeled row by construction.
    // `is_multiple_of`, not `n % 4 == 0`: clippy 1.96's
    // manual_is_multiple_of rejects the modulo form under -D warnings,
    // and this file is include!'d by three build scripts, so the lint
    // fires three times.
    assert!(
        n.is_multiple_of(4),
        "CER_BENCH_POD_BYTES={n} is not a multiple of 4: PodPayloadShm is \
         #[repr(C)] with u32 fields, so its wire size rounds up to {} and \
         the matched-quantity rule (exactly N bytes) cannot hold. The pod \
         class realizes multiples of 4 only (the pinned sweep sizes all \
         are); pick {} or {}, never a rounded label",
        n.div_ceil(4) * 4,
        n / 4 * 4,
        n.div_ceil(4) * 4
    );
    let data_len = n - 12;

    let msg = format!(
        "uint32 prep\nuint32 stamp_hi\nuint32 stamp_lo\nuint8[{data_len}] data\n"
    );
    // The package is load-bearing in FOUR places at once, and they must
    // all say `cer_bench_msgs`:
    //   * here, because parse_rosmsg folds the package into SCHEMA_HASH —
    //     the hash the producer stamps on every frame and the recorder
    //     writes into the MCAP channel;
    //   * the crate name, because node metadata keeps a genuine crate path
    //     as the schema package (a same-file `mod` contributes none —
    //     engine `089e47d86`), which is what `graph validate` compares the
    //     graph's `schema:` against;
    //   * the graph YAML;
    //   * the registration directory, `schemas/cer_bench_msgs/msg/`, whose
    //     NAME is the package the CLI resolves the recorded hash under.
    // Measured when the registration was a bare workspace YAML instead:
    // registered 0xb981270238cd0358 vs generated 0xf99555aa42af17bc, so
    // both POD channels recorded undescribable and the bag was not
    // replay-grade — live delivery was fine, which is why only the
    // recorder noticed.
    let schema = cerulion_core::codegen::parse_rosmsg(
        &msg, "PodPayload", Some("cer_bench_msgs"))
        .unwrap_or_else(|e| panic!("PodPayload .msg text failed to parse: {e}"));
    let code = cerulion_core::codegen::generate_schema(&schema);

    let out_dir = env::var("OUT_DIR").expect("OUT_DIR set by cargo");
    let out_path = Path::new(&out_dir).join("pod_payload.rs");
    let mut content = String::new();
    content.push_str("// Generated PodPayload schema (type-class axis).\n");
    content.push_str("// DO NOT EDIT — generated by pod_schema/pod_codegen.rs.\n");
    content.push_str(&format!(
        "// Baked payload size: {n} bytes total (CER_BENCH_POD_BYTES).\n\n"
    ));
    content.push_str(&code);
    fs::write(&out_path, content)
        .unwrap_or_else(|e| panic!("write {}: {e}", out_path.display()));
}
