// SPDX-License-Identifier: AGPL-3.0-only
//! A process that builds a transport runs on a raised open-file soft limit.
//!
//! Each arm re-runs this test binary as a child whose soft `RLIMIT_NOFILE` is
//! set before `exec`, the way a login shell with a low default starts a graph.
//! The child reads its soft limit, builds a transport on an isolated iceoryx2
//! root, reads the limit again and prints both. In the transport arms the
//! child calls nothing else; in the embedding-host arms it first calls
//! `fd_limit::raise_open_file_limit_for_embedding_host`, as `rmw_cerulion`
//! and `cerulion_py` do. The expected value is derived here from the hard
//! limit and the platform rule, never from the code under test: for a
//! transport, the hard limit on Linux, at most 1048576 there, and at most 1024
//! (`FD_SETSIZE`) elsewhere; for an embedding host, at most 1024 everywhere.
#![cfg(unix)]

use std::os::unix::process::CommandExt;
use std::process::Command;

use cerulion_core::transport::{TransportConfig, TransportManager};

const CHILD: &str = "child_reports_its_open_file_limit_around_a_transport_build";
const CHILD_ENV: &str = "CER_TEST_OPEN_FILE_LIMIT_CHILD";
/// The child's value of [`CHILD_ENV`] that makes it raise as an embedding host
/// before its transport build.
const HOST: &str = "host";
const LOW_SOFT: u64 = 256;
const FD_SETSIZE: u64 = 1024;

fn nofile() -> (u64, u64) {
    let mut rl = libc::rlimit {
        rlim_cur: 0,
        rlim_max: 0,
    };
    // SAFETY: `getrlimit` writes into `rl`, a local this function owns.
    let rc = unsafe { libc::getrlimit(libc::RLIMIT_NOFILE, &mut rl) };
    assert_eq!(rc, 0, "getrlimit(RLIMIT_NOFILE)");
    (to_u64(rl.rlim_cur), to_u64(rl.rlim_max))
}

// `rlim_t` is `u64` on 64-bit targets and narrower on some 32-bit ones; an
// unlimited value reads as `u64::MAX` on every target.
#[allow(clippy::useless_conversion)] // the identity on 64-bit targets
fn to_u64(value: libc::rlim_t) -> u64 {
    if value == libc::RLIM_INFINITY {
        u64::MAX
    } else {
        u64::from(value)
    }
}

#[allow(clippy::useless_conversion, clippy::unnecessary_fallible_conversions)] // the identity on 64-bit targets
fn to_rlim(value: u64) -> libc::rlim_t {
    if value == u64::MAX {
        libc::RLIM_INFINITY
    } else {
        libc::rlim_t::try_from(value).expect("limit fits rlim_t")
    }
}

/// Runs the child arm with its soft limit set to `LOW_SOFT` and its hard limit
/// to `hard`, and returns the child's `(soft at start, soft after, hard)`.
fn run_child(hard: u64) -> (u64, u64, u64) {
    run_child_as("1", LOW_SOFT, hard)
}

/// Runs the child arm with [`CHILD_ENV`] set to `mode` and its limits set to
/// `soft` and `hard`, and returns the child's `(soft at start, soft after,
/// hard)`.
fn run_child_as(mode: &str, soft: u64, hard: u64) -> (u64, u64, u64) {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = Command::new(exe);
    cmd.args(["--exact", CHILD, "--ignored", "--nocapture"])
        .env(CHILD_ENV, mode);
    let rl = libc::rlimit {
        rlim_cur: to_rlim(soft),
        rlim_max: to_rlim(hard),
    };
    // SAFETY: the closure runs in the forked child before `exec` and calls
    // only `setrlimit`, which is async-signal-safe, on a copied local.
    unsafe {
        cmd.pre_exec(move || {
            if libc::setrlimit(libc::RLIMIT_NOFILE, &rl) != 0 {
                return Err(std::io::Error::last_os_error());
            }
            Ok(())
        });
    }
    let out = cmd.output().expect("spawn the child arm");
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        out.status.success(),
        "child arm failed: {}\nstdout:\n{stdout}\nstderr:\n{}",
        out.status,
        String::from_utf8_lossy(&out.stderr)
    );
    let line = stdout
        .lines()
        .find(|l| l.starts_with("NOFILE "))
        .unwrap_or_else(|| panic!("child printed no NOFILE line; stdout:\n{stdout}"));
    let field = |key: &str| -> u64 {
        line.split_whitespace()
            .find_map(|kv| kv.strip_prefix(key))
            .unwrap_or_else(|| panic!("no {key} in {line:?}"))
            .parse()
            .unwrap_or_else(|e| panic!("bad {key} in {line:?}: {e}"))
    };
    (field("start="), field("after="), field("hard="))
}

/// The soft limit a transport build must reach from `LOW_SOFT` under `hard`.
fn expected_soft(hard: u64) -> u64 {
    if cfg!(target_os = "linux") {
        hard.min(1 << 20)
    } else {
        hard.min(1024)
    }
}

#[test]
fn a_child_started_at_a_low_soft_limit_builds_its_transport_on_the_raised_limit() {
    let (_, hard) = nofile();
    assert!(
        hard > LOW_SOFT,
        "this arm needs a hard limit above {LOW_SOFT}, the machine has {hard}"
    );
    let (start, after, child_hard) = run_child(hard);
    assert_eq!(
        start, LOW_SOFT,
        "the child must start at the low soft limit"
    );
    assert_eq!(child_hard, hard, "the hard limit is never changed");
    assert_eq!(
        after,
        expected_soft(hard),
        "soft limit after the transport build"
    );
    assert!(
        after > LOW_SOFT,
        "the soft limit must have gone up, got {after}"
    );
}

#[test]
fn a_child_whose_hard_limit_equals_its_soft_limit_keeps_both() {
    let (start, after, hard) = run_child(LOW_SOFT);
    assert_eq!((start, after, hard), (LOW_SOFT, LOW_SOFT, LOW_SOFT));
}

#[test]
fn an_embedding_host_raises_a_low_soft_limit_no_higher_than_fd_setsize() {
    let (_, hard) = nofile();
    assert!(
        hard > LOW_SOFT,
        "this arm needs a hard limit above {LOW_SOFT}, the machine has {hard}"
    );
    let (start, after, child_hard) = run_child_as(HOST, LOW_SOFT, hard);
    assert_eq!(
        start, LOW_SOFT,
        "the child must start at the low soft limit"
    );
    assert_eq!(child_hard, hard, "the hard limit is never changed");
    assert_eq!(
        after,
        hard.min(FD_SETSIZE),
        "an embedding host's soft limit after its transport build"
    );
}

#[test]
fn an_embedding_host_keeps_a_soft_limit_already_above_fd_setsize() {
    let (_, hard) = nofile();
    let soft = 4 * FD_SETSIZE;
    assert!(
        hard >= soft,
        "this arm needs a hard limit of at least {soft}, the machine has {hard}"
    );
    let (start, after, _) = run_child_as(HOST, soft, hard);
    assert_eq!(
        (start, after),
        (soft, soft),
        "a soft limit is never lowered"
    );
}

/// The child half: inert unless the parent arm started it.
#[test]
#[ignore = "child process of the arms above; inert on its own"]
fn child_reports_its_open_file_limit_around_a_transport_build() {
    let Some(mode) = std::env::var_os(CHILD_ENV) else {
        return;
    };
    let (start, _) = nofile();
    if mode == HOST {
        cerulion_core::fd_limit::raise_open_file_limit_for_embedding_host();
    }
    let config = TransportConfig {
        node_name: format!("open_file_limit_{}", std::process::id()),
        ..Default::default()
    };
    let _manager =
        TransportManager::init_for_test(config, cerulion_core::testing::iceoryx_test_config())
            .expect("build an isolated transport");
    let (after, hard) = nofile();
    println!("NOFILE start={start} after={after} hard={hard}");
}
