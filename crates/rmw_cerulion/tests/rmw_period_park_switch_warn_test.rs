// SPDX-License-Identifier: AGPL-3.0-only
//! The `CERULION_RMW_PERIOD_PARK` notices are LOUD and logged once: the line a user reads
//! is pinned here, in an OWN binary because `#[traced_test]` takes the process-global
//! subscriber slot (the `rmw_cpp_bypass_warn_test` precedent), and because each notice is
//! logged once per process. Which value owes which notice (`period_park_notice`) is
//! oracle-pinned in the lib tests; this file pins the emission on the path THIS build takes:
//!
//! - off Linux, or on a Linux build without the period-learned park: `1` is read once, at
//!   the first wait set, and warns that the park runs only on Linux (or is not carried);
//! - a build that carries it on Linux: every wait reads the value. With a monitor-wait
//!   primitive, `1` and `0` stay silent and a near miss warns once; without one, `0` stays
//!   silent and `1` warns once that the CPU has none.
//!
//! Each value runs two waits on two wait sets, so a notice logged per call fails.
#![cfg(unix)]

use serial_test::serial;
use std::os::raw::c_char;
use tracing_test::traced_test;

use rmw_cerulion::ffi::{self, RMW_RET_OK};
use rmw_cerulion::*;

const LEVELS: [&str; 5] = ["ERROR", "WARN", "INFO", "DEBUG", "TRACE"];

/// The level token out of a captured line's HEADER (the text before the first `": "`), as a
/// WHOLE whitespace token, so a lower-level line whose text says "WARN" cannot pass.
fn line_level(line: &str) -> Option<&'static str> {
    let header = line.split(": ").next().unwrap_or(line);
    header
        .split_whitespace()
        .find_map(|token| LEVELS.into_iter().find(|level| *level == token))
}

/// The message and fields of a captured line: the text after its header and its target.
fn body_of(line: &str) -> &str {
    line.splitn(3, ": ").nth(2).unwrap_or("")
}

fn cstr(s: &str) -> *const c_char {
    Box::leak(
        std::ffi::CString::new(s)
            .expect("no NUL")
            .into_boxed_c_str(),
    )
    .as_ptr()
}

/// The notice texts, hand-written: a change to the message the user reads fails here.
const NOT_CARRIED: &str = "CERULION_RMW_PERIOD_PARK=1, but this rmw_cerulion build does not \
     carry the period-learned park (every Linux aarch64 build does; another Linux build needs \
     the period-park Cargo feature), so rmw_wait keeps its stock wait";
const NOT_LINUX: &str = "CERULION_RMW_PERIOD_PARK=1, but the period-learned park runs only on \
     Linux, so rmw_wait keeps its stock wait";
const NO_PRIMITIVE: &str = "CERULION_RMW_PERIOD_PARK=1, but this CPU has no monitor-wait \
     instruction for the period-learned park to wait in, so rmw_wait keeps its stock wait";
const NEAR_MISS: &str = "ignoring unrecognized CERULION_RMW_PERIOD_PARK value; only \"0\" (off) \
     and \"1\" (on) are honored, so the period-learned park keeps its default, which is on \
     wherever this build can run it";

/// Two wait sets, one zero-timeout wait each, with `CERULION_RMW_PERIOD_PARK` set to `value`.
unsafe fn two_waits(context: *mut ffi::rmw_context_t, value: &str) {
    std::env::set_var("CERULION_RMW_PERIOD_PARK", value);
    for _ in 0..2 {
        let ws = rmw_create_wait_set(context, 8);
        assert!(!ws.is_null());
        let zero = ffi::rmw_time_t { sec: 0, nsec: 0 };
        let ret = rmw_wait(
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            ws,
            &zero,
        );
        assert_eq!(ret, ffi::RMW_RET_TIMEOUT, "an empty wait set times out");
        assert_eq!(rmw_destroy_wait_set(ws), RMW_RET_OK);
    }
}

#[traced_test]
#[test]
#[serial]
fn the_period_park_switch_notice_is_one_warn_line_on_this_builds_path() {
    let carries = cfg!(cerulion_period_park);
    let linux = cfg!(target_os = "linux");
    let hw = cerulion_core::monitor_wait::monitor_wait_available();
    // (values set in order, the exact body of the one notice line expected)
    let near_miss = format!("{NEAR_MISS} value=on");
    let (values, expected): (&[&str], &str) = if !linux {
        (&["1"], NOT_LINUX)
    } else if !carries {
        (&["1"], NOT_CARRIED)
    } else if hw {
        (&["1", "0", "on"], &near_miss)
    } else {
        (&["0", "1"], NO_PRIMITIVE)
    };
    let prior = std::env::var_os("CERULION_RMW_PERIOD_PARK");
    unsafe {
        let mut options: ffi::rmw_init_options_t = std::mem::zeroed();
        let allocator: ffi::rcutils_allocator_t = std::mem::zeroed();
        assert_eq!(rmw_init_options_init(&mut options, allocator), RMW_RET_OK);
        let context: *mut ffi::rmw_context_t = Box::leak(Box::new(std::mem::zeroed()));
        assert_eq!(rmw_init(&options, context), RMW_RET_OK);
        let node = rmw_create_node(context, cstr("pp_switch_warn"), cstr("/"));
        assert!(!node.is_null());
        for value in values {
            two_waits(context, value);
        }
        assert_eq!(rmw_destroy_node(node), RMW_RET_OK);
    }
    match prior {
        Some(v) => std::env::set_var("CERULION_RMW_PERIOD_PARK", v),
        None => std::env::remove_var("CERULION_RMW_PERIOD_PARK"),
    }
    logs_assert(|lines: &[&str]| {
        let hits: Vec<&str> = lines
            .iter()
            .copied()
            .filter(|l| l.contains("CERULION_RMW_PERIOD_PARK"))
            .collect();
        if hits.len() != 1 {
            return Err(format!(
                "values {values:?}: expected exactly one notice line, got {}: {hits:#?}",
                hits.len()
            ));
        }
        let line = hits[0];
        if line_level(line) != Some("WARN") {
            return Err(format!("the notice must be WARN: {line}"));
        }
        let body = body_of(line);
        if body != expected {
            return Err(format!(
                "the notice body is not exact:\n  got:  {body}\n  want: {expected}"
            ));
        }
        Ok(())
    });
}
