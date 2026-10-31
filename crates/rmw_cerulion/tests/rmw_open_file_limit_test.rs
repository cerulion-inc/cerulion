// SPDX-License-Identifier: AGPL-3.0-only
//! The rmw runtime keeps a ROS 2 host's soft open-file limit at or below
//! `FD_SETSIZE`.
//!
//! Each arm re-runs this test binary as a child whose soft `RLIMIT_NOFILE` is
//! set before `exec`. The child brings up `rmw_cerulion::runtime::runtime()`
//! (the transport every rmw entry point uses), reads the limits and prints
//! them. The expected soft limit is derived here from the hard limit: at most
//! 1024 (`FD_SETSIZE`) on every platform, and never lower than the child
//! started with.
#![cfg(unix)]

use std::os::unix::process::CommandExt;
use std::process::Command;

const CHILD: &str = "child_reports_its_open_file_limit_around_the_rmw_runtime";
const CHILD_ENV: &str = "CER_TEST_RMW_OPEN_FILE_LIMIT_CHILD";
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

/// Runs the child arm at `soft` under `hard` and returns the child's
/// `(soft at start, soft after, hard)`.
fn run_child(soft: u64, hard: u64) -> (u64, u64, u64) {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = Command::new(exe);
    cmd.args(["--exact", CHILD, "--ignored", "--nocapture"])
        .env(CHILD_ENV, "1");
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

#[test]
fn the_rmw_runtime_raises_a_low_soft_limit_no_higher_than_fd_setsize() {
    let (_, hard) = nofile();
    assert!(
        hard > 256,
        "this arm needs a hard limit above 256, the machine has {hard}"
    );
    let (start, after, child_hard) = run_child(256, hard);
    assert_eq!(start, 256, "the child must start at the low soft limit");
    assert_eq!(child_hard, hard, "the hard limit is never changed");
    assert_eq!(after, hard.min(FD_SETSIZE), "soft limit after the runtime");
}

#[test]
fn the_rmw_runtime_keeps_a_soft_limit_already_above_fd_setsize() {
    let (_, hard) = nofile();
    let soft = 4 * FD_SETSIZE;
    assert!(
        hard >= soft,
        "this arm needs a hard limit of at least {soft}, the machine has {hard}"
    );
    let (start, after, _) = run_child(soft, hard);
    assert_eq!(
        (start, after),
        (soft, soft),
        "a soft limit is never lowered"
    );
}

/// The child half: inert unless the parent arm started it.
#[test]
#[ignore = "child process of the arms above; inert on its own"]
fn child_reports_its_open_file_limit_around_the_rmw_runtime() {
    if std::env::var_os(CHILD_ENV).is_none() {
        return;
    }
    let (start, _) = nofile();
    rmw_cerulion::runtime::runtime().expect("bring up the rmw runtime");
    let (after, hard) = nofile();
    println!("NOFILE start={start} after={after} hard={hard}");
}
