// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-netd --version` / `-V` over the REAL binary (`CARGO_BIN_EXE_cerulion-netd`):
//! the exact `cerulion-netd <version>` line on STDOUT, nothing on stderr, exit 0.
//!
//! The flag→action mapping is oracle-tested in `src/main.rs`; THIS pins the output
//! contract that mapping feeds — the stream, the line shape, the exit code — and
//! that nothing else runs (a daemon that booted after printing would log to
//! stderr). `--version` returns before tracing and transport init, so this needs
//! no socket, lock, SHM root or env: parallel-safe like every file in this crate.

use std::process::Command;

#[test]
fn version_flags_print_the_package_version_to_stdout_and_exit_zero() {
    for flag in ["--version", "-V"] {
        let out = Command::new(env!("CARGO_BIN_EXE_cerulion-netd"))
            .arg(flag)
            .output()
            .expect("spawn cerulion-netd");
        assert!(out.status.success(), "{flag}: exit status {:?}", out.status);
        assert_eq!(
            String::from_utf8_lossy(&out.stdout),
            format!("cerulion-netd {}\n", env!("CARGO_PKG_VERSION")),
            "{flag}: stdout must be exactly the `cerulion-netd <version>` line"
        );
        assert!(
            out.stderr.is_empty(),
            "{flag}: nothing may reach stderr — got {:?}",
            String::from_utf8_lossy(&out.stderr)
        );
    }
}
