// SPDX-License-Identifier: AGPL-3.0-only
//! Shell scripts a test writes and then runs, or hands to a process that runs
//! them, without a "Text file busy" failure.
//!
//! On Linux, executing a file fails with ETXTBSY while any process holds it
//! open for writing. `std::fs::write` opens the file close-on-exec, but a test
//! thread that forks while that descriptor is open copies it into its child,
//! which keeps it until the child execs. A script run right after it is
//! written can therefore fail to start when another test in the same binary
//! forked while the script was open for writing.
//!
//! [`install_script`] writes the script with a probe line after its shebang
//! and returns once a probe run has started. The test's own descriptor is
//! closed by then, so no new copy of it can appear, and every later run of
//! the file starts. [`retry_while_busy`] is the same wait for a file that
//! cannot carry the probe line, such as a copied binary run with a harmless
//! argument.
//!
//! A test binary cannot use another crate's test code, so each crate whose
//! tests need this keeps a copy of this file.
//! `crates/cerulion_cli/tests/fresh_executable_test.rs` tests it and checks
//! that every copy is byte-identical.

use std::io;
use std::path::Path;
use std::process::Command;
use std::time::Duration;

/// Calls of a busy run before [`retry_while_busy`] returns its error.
pub const BUSY_ATTEMPTS: u32 = 500;

/// The pause between two calls of a busy run: [`BUSY_ATTEMPTS`] calls span
/// about 5 s.
pub const BUSY_PAUSE: Duration = Duration::from_millis(10);

/// Set on a probe run: a script from [`install_script`] then exits 0 at its
/// probe line, before any of its body runs.
pub const PROBE_ENV: &str = "CERULION_TEST_PROBE";

/// The first line of every script [`install_script`] accepts.
const SHEBANG: &str = "#!/bin/sh\n";

/// Writes `script` to `path` with the probe line after its `#!/bin/sh` line,
/// makes the file executable, and returns once a probe run of it has started
/// and exited 0.
///
/// Panics if `script` does not start with `#!/bin/sh`, if the file cannot be
/// written, or if the probe run does not start and exit 0 within
/// [`BUSY_ATTEMPTS`] calls.
pub fn install_script(path: &Path, script: &str) {
    use std::os::unix::fs::PermissionsExt;
    let body = script
        .strip_prefix(SHEBANG)
        .unwrap_or_else(|| panic!("{} must start with {SHEBANG:?}", path.display()));
    std::fs::write(
        path,
        format!("{SHEBANG}if [ -n \"${PROBE_ENV}\" ]; then exit 0; fi\n{body}"),
    )
    .unwrap_or_else(|error| panic!("write {}: {error}", path.display()));
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o755))
        .unwrap_or_else(|error| panic!("chmod {}: {error}", path.display()));
    let status = retry_while_busy(BUSY_PAUSE, || {
        Command::new(path).env(PROBE_ENV, "1").status()
    })
    .unwrap_or_else(|error| panic!("probe run of {}: {error}", path.display()));
    assert!(
        status.success(),
        "probe run of {}: {status}",
        path.display()
    );
}

/// Calls `run` until it returns anything other than an ETXTBSY error, at most
/// [`BUSY_ATTEMPTS`] times with `pause` between calls, and returns that
/// result: the last ETXTBSY error when every call was busy. An exec that fails
/// with ETXTBSY starts nothing, so a call made again has no effect to undo.
pub fn retry_while_busy<T>(
    pause: Duration,
    mut run: impl FnMut() -> io::Result<T>,
) -> io::Result<T> {
    for _ in 1..BUSY_ATTEMPTS {
        match run() {
            Err(error) if error.kind() == io::ErrorKind::ExecutableFileBusy => {
                std::thread::sleep(pause);
            }
            result => return result,
        }
    }
    run()
}
