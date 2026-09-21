// SPDX-License-Identifier: AGPL-3.0-only
//! Container-harness helper for `tools/ros2_migrate/run_matrix.sh` (its apply step):
//! apply `cerulion-ros2-migrate-clang` analysis JSON to the fixture sources
//! IN PLACE, through the SAME `parse_tool_output` → `merge_tool_outputs` →
//! `build_plan` → `apply_plan_in_place` path `cerulion ros2 migrate` uses —
//! so the matrix can gate the APPLIED bytes (do they still build? behave?)
//! without building the whole verb. A doubled-arrow defect
//! passes the matrix's substring oracles precisely when nothing applies
//! and compiles the bytes; applying them here closes that hole.
//!
//! Routing through `build_plan` is load-bearing, not convenience: it
//! is where the canonicalization + containment gate lives, so
//! a crafted analysis JSON naming a file outside the supplied src root is
//! REFUSED here exactly as the verb refuses it — a direct
//! `fs::read`/`fs::write` loop would write any readable path the
//! JSON named. The writes then go through the verb's anchored
//! per-component `O_NOFOLLOW` walker (`apply_plan_in_place`), never a
//! path-following `fs::write`.
//!
//! NOT a user surface. Usage: `ros2_migrate_apply <ws-src-root> <analysis.json>...`
//! (mirrors the verb's write path minus git/consent/colcon).

use cerulion_cli_engine::ros2_migrate::{
    apply_plan_in_place, build_plan, merge_tool_outputs, parse_tool_output,
    read_regular_bounded_to_string,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 2 {
        return Err("usage: ros2_migrate_apply <ws-src-root> <analysis.json>...".into());
    }
    let ws_src = std::path::PathBuf::from(&args[0])
        .canonicalize()
        .map_err(|e| format!("canonicalize src root {}: {e}", args[0]))?;
    let mut outputs = Vec::new();
    for path in &args[1..] {
        // Analysis integrity: the verb's
        // ONE bounded, non-blocking, regular-file-gated read — never a bare
        // `read_to_string`, which follows a link and waits forever on a
        // FIFO planted at a result path. The harness additionally seals the
        // results dir and re-verifies its digest immediately before and
        // after this process runs (`consume_results`), so a same-UID swap
        // in the remaining window is DETECTED; this gate is what stops a
        // non-regular object from wedging the apply before that check can
        // speak.
        let json = read_regular_bounded_to_string(std::path::Path::new(path))
            .map_err(|e| format!("read {path}: {e}"))?;
        outputs.push(parse_tool_output(&json)?);
    }
    let analysis = merge_tool_outputs(&outputs);
    let plan = build_plan(&ws_src, &ws_src, &analysis)?;
    apply_plan_in_place(&ws_src, &plan)?;
    for f in &plan.files {
        println!("applied {}", f.rel);
    }
    println!(
        "applied {} rewrite(s) across {} file(s)",
        analysis.rewrites.len(),
        plan.files.len()
    );
    Ok(())
}
