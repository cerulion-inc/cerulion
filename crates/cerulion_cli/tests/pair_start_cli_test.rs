// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion pair --start` through the real binary where no `cerulion-netd`
//! can be started: the run asks netd once, names why it could not, exits 3
//! and makes no robot state. Each test owns its temporary home, socket path
//! and state root, so the tests run in parallel.
#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const ASKING: &str = "Asking cerulion-netd to serve this robot for code pairing.\n";

/// A temporary machine: a signed-in home, a socket path nothing listens on and
/// a state root that does not exist.
struct Machine {
    tmp: tempfile::TempDir,
    home: PathBuf,
    state: PathBuf,
}

fn machine() -> Machine {
    let tmp = tempfile::tempdir().unwrap();
    let home = tmp.path().join("home");
    cerulion_cli_engine::auth::seed_logged_in_at(&home, "pair-start-cli-test").unwrap();
    let state = tmp.path().join("state");
    Machine { tmp, home, state }
}

impl Machine {
    fn command(&self, bin: &Path) -> Command {
        let mut command = Command::new(bin);
        command
            .args(["pair", "--start"])
            .current_dir(self.tmp.path())
            .env("CERULION_HOME", &self.home)
            .env("CERULION_STATE_ROOT", &self.state)
            .env("CERULION_NETD_SOCK", self.tmp.path().join("none.sock"))
            .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
            .env_remove("CERULION_NETWORK")
            .env_remove("CERULION_NETD_NETWORK")
            .env_remove("CERULION_LOGIN_GATE")
            .env_remove("RUST_LOG");
        command
    }
}

fn text(output: &Output) -> (Option<i32>, String, String) {
    (
        output.status.code(),
        String::from_utf8_lossy(&output.stdout).into_owned(),
        String::from_utf8_lossy(&output.stderr).into_owned(),
    )
}

#[test]
fn a_netd_binary_that_cannot_start_is_named_and_no_code_is_made() {
    let m = machine();
    let absent = m.tmp.path().join("absent-netd");
    let output = m
        .command(Path::new(env!("CARGO_BIN_EXE_cerulion")))
        .env("CERULION_NETD_BIN", &absent)
        .output()
        .unwrap();
    let (code, stdout, stderr) = text(&output);
    assert_eq!(code, Some(3), "{stderr}");
    assert_eq!(
        stderr,
        format!(
            "{ASKING}Error: could not ask cerulion-netd to serve this robot: could not start \
             cerulion-netd from {}: No such file or directory (os error 2). No code was made. \
             Install cerulion-netd beside cerulion or set CERULION_NETD_BIN to its path.\n",
            absent.display()
        )
    );
    assert!(stdout.is_empty(), "{stdout}");
    assert!(!m.state.exists(), "no robot state is made");
}

#[test]
fn no_netd_beside_the_binary_is_named_without_netds_own_text() {
    let m = machine();
    // The binary alone in a directory with no cerulion-netd beside it: a hard
    // link where the filesystem allows one, else a copy.
    let bin_dir = m.tmp.path().join("bin");
    std::fs::create_dir(&bin_dir).unwrap();
    let bin = bin_dir.join("cerulion");
    let source = Path::new(env!("CARGO_BIN_EXE_cerulion"));
    if std::fs::hard_link(source, &bin).is_err() {
        std::fs::copy(source, &bin).unwrap();
    }
    let output = m
        .command(&bin)
        .env_remove("CERULION_NETD_BIN")
        .output()
        .unwrap();
    let (code, stdout, stderr) = text(&output);
    assert_eq!(code, Some(3), "{stderr}");
    let canonical = bin_dir.canonicalize().unwrap();
    let expected = |dir: &Path| {
        format!(
            "{ASKING}Error: could not ask cerulion-netd to serve this robot: no cerulion-netd \
             was found beside this cerulion in {}. No code was made. Install cerulion-netd \
             beside cerulion or set CERULION_NETD_BIN to its path.\n",
            dir.display()
        )
    };
    assert!(
        stderr == expected(&bin_dir) || stderr == expected(&canonical),
        "{stderr}"
    );
    assert!(!stderr.contains('\u{2014}'), "{stderr}");
    assert_eq!(
        stderr.matches("Install cerulion-netd").count(),
        1,
        "{stderr}"
    );
    assert!(stdout.is_empty(), "{stdout}");
    assert!(!m.state.exists(), "no robot state is made");
}
