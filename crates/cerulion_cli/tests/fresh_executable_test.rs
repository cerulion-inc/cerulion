// SPDX-License-Identifier: AGPL-3.0-only
//! The `fresh_executable` test helper: which results it calls a run again
//! for, the script it writes, that it waits out a real held write descriptor
//! on Linux, and that every crate's copy of it is this one.

#![cfg(unix)]

mod fresh_executable;

use std::cell::Cell;
use std::io;
use std::path::Path;
use std::process::Command;
use std::time::Duration;

use fresh_executable::{install_script, retry_while_busy, BUSY_ATTEMPTS, PROBE_ENV};

/// The other crates' copies of `tests/fresh_executable/mod.rs`, relative to
/// this crate's directory.
const COPIES: [&str; 3] = [
    "../cerulion_cli_engine/src/fresh_executable.rs",
    "../cerulion_netd/src/fresh_executable.rs",
    "../cerulion_wsd/tests/fresh_executable/mod.rs",
];

/// The error an exec of a file held open for writing returns.
fn busy() -> io::Error {
    io::Error::from_raw_os_error(libc::ETXTBSY)
}

/// Runs `retry_while_busy` with `result_of_call` answering each call by its
/// index, and returns its result with the number of calls it made.
fn retried(result_of_call: impl Fn(usize) -> io::Result<u8>) -> (io::Result<u8>, usize) {
    let calls = Cell::new(0);
    let result = retry_while_busy(Duration::ZERO, || {
        let call = calls.get();
        calls.set(call + 1);
        result_of_call(call)
    });
    (result, calls.get())
}

#[test]
fn a_busy_run_is_called_again_until_it_returns_something_else() {
    let twice_busy = |call| if call < 2 { Err(busy()) } else { Ok(7) };
    let (result, calls) = retried(twice_busy);
    assert_eq!((result.unwrap(), calls), (7, 3));
    // The same answers give the same result again.
    let (again, calls_again) = retried(twice_busy);
    assert_eq!((again.unwrap(), calls_again), (7, 3));
}

#[test]
fn a_result_that_is_not_busy_is_returned_from_the_call_that_made_it() {
    let (ok, calls) = retried(|_| Ok(0));
    assert_eq!((ok.unwrap(), calls), (0, 1));
    for errno in [libc::ENOENT, libc::EACCES, libc::ENOEXEC] {
        let (error, calls) = retried(|_| Err(io::Error::from_raw_os_error(errno)));
        assert_eq!(error.unwrap_err().raw_os_error(), Some(errno));
        assert_eq!(calls, 1);
    }
    // A busy call followed by another error stops at that error.
    let (error, calls) = retried(|call| {
        Err(io::Error::from_raw_os_error(if call == 0 {
            libc::ETXTBSY
        } else {
            libc::ENOENT
        }))
    });
    assert_eq!(error.unwrap_err().raw_os_error(), Some(libc::ENOENT));
    assert_eq!(calls, 2);
}

#[test]
fn a_nonzero_exit_is_a_result_and_is_not_run_again() {
    let calls = Cell::new(0);
    let status = retry_while_busy(Duration::ZERO, || {
        calls.set(calls.get() + 1);
        Command::new("/bin/sh").args(["-c", "exit 78"]).status()
    })
    .unwrap();
    assert_eq!(status.code(), Some(78));
    assert_eq!(calls.get(), 1);
}

#[test]
fn a_run_busy_on_every_call_returns_its_error_after_the_last_attempt() {
    let (error, calls) = retried(|_| Err(busy()));
    assert_eq!(error.unwrap_err().raw_os_error(), Some(libc::ETXTBSY));
    assert_eq!(calls, BUSY_ATTEMPTS as usize);
}

#[test]
fn install_script_puts_the_probe_line_after_the_shebang_and_runs_the_body_without_it() {
    use std::os::unix::fs::PermissionsExt;
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("script");
    let marker = dir.path().join("ran");
    install_script(&path, "#!/bin/sh\n: > \"$MARKER\"\nexit 78\n");
    assert_eq!(
        std::fs::read_to_string(&path).unwrap(),
        "#!/bin/sh\nif [ -n \"$CERULION_TEST_PROBE\" ]; then exit 0; fi\n: > \"$MARKER\"\nexit 78\n"
    );
    assert_eq!(
        std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
        0o755
    );
    let probe = Command::new(&path)
        .env(PROBE_ENV, "1")
        .env("MARKER", &marker)
        .status()
        .unwrap();
    assert_eq!(probe.code(), Some(0));
    assert!(!marker.exists(), "a probe run reached the body");
    let real = Command::new(&path)
        .env_remove(PROBE_ENV)
        .env("MARKER", &marker)
        .status()
        .unwrap();
    assert_eq!(real.code(), Some(78));
    assert!(marker.is_file(), "a real run did not reach the body");
}

#[test]
fn install_script_keeps_the_exit_status_of_a_body_with_no_exit() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("empty");
    install_script(&path, "#!/bin/sh\n");
    let status = Command::new(&path).env_remove(PROBE_ENV).status().unwrap();
    assert_eq!(status.code(), Some(0));
}

#[test]
#[should_panic(expected = "must start with \"#!/bin/sh\\n\"")]
fn install_script_refuses_a_script_without_the_sh_shebang() {
    let dir = tempfile::tempdir().unwrap();
    install_script(&dir.path().join("bash"), "#!/bin/bash\nexit 0\n");
}

#[test]
#[should_panic(expected = "write ")]
fn install_script_panics_when_the_file_cannot_be_written() {
    let dir = tempfile::tempdir().unwrap();
    install_script(&dir.path().join("absent/script"), "#!/bin/sh\nexit 0\n");
}

/// Holds a write descriptor on a script the way a concurrently forked child
/// does: running it then fails with ETXTBSY, and `install_script` returns only
/// after the descriptor is closed.
#[cfg(target_os = "linux")]
#[test]
fn install_script_waits_out_a_held_write_descriptor() {
    use fresh_executable::BUSY_PAUSE;
    use std::os::unix::fs::PermissionsExt;
    use std::time::Instant;
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("busy");
    std::fs::write(&path, "#!/bin/sh\nexit 0\n").unwrap();
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).unwrap();
    let writer = std::fs::OpenOptions::new().write(true).open(&path).unwrap();
    let error = Command::new(&path)
        .status()
        .expect_err("a script open for writing must not run");
    let kernel = std::fs::read_to_string("/proc/sys/kernel/osrelease").unwrap_or_default();
    assert_eq!(
        error.raw_os_error(),
        Some(libc::ETXTBSY),
        "{error} (kernel {})",
        kernel.trim()
    );
    let release = std::thread::spawn(move || {
        std::thread::sleep(20 * BUSY_PAUSE);
        let released = Instant::now();
        drop(writer);
        released
    });
    install_script(&path, "#!/bin/sh\nexit 0\n");
    let returned = Instant::now();
    let released = release.join().unwrap();
    assert!(
        returned > released,
        "install_script returned before the writer closed"
    );
}

#[test]
fn every_crate_carries_this_copy() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let own = std::fs::read_to_string(dir.join("tests/fresh_executable/mod.rs")).unwrap();
    for copy in COPIES {
        let theirs = std::fs::read_to_string(dir.join(copy))
            .unwrap_or_else(|error| panic!("read {copy}: {error}"));
        assert!(
            theirs == own,
            "{copy} differs from tests/fresh_executable/mod.rs: copy one over the other"
        );
    }
}
