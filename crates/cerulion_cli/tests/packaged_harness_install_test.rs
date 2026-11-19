// SPDX-License-Identifier: AGPL-3.0-only
//! The harness a release package installs is the one `cerulion rcp` runs.
//!
//! `tools/scripts/install.sh` installs a release archive's `harness/` beside
//! the programs, and `cerulion rcp` never takes the folder beside its
//! canonicalized binary when every user can write that folder or the
//! binary's own. This drives the real binary through the real installer:
//! an archive holding this build's `cerulion` and a stand-in harness whose
//! tree any user could write, installed under umask 000, must leave a tree
//! `cerulion rcp bench --verify` runs from the install directory as the
//! packaged folder (`--packaged`), with no `CERULION_HARNESS_DIR` and no source
//! checkout around it. The control makes the same install directory writable
//! by others and expects the CLI to refuse it, so the first case is decided by
//! the modes the installer set and not by any other place the search looks.
#![cfg(unix)]

use std::fs;
use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};
use std::process::Command;

/// The release target `install.sh` maps this host to, or `None` where it has
/// no release archive.
fn release_target() -> Option<&'static str> {
    match (std::env::consts::OS, std::env::consts::ARCH) {
        ("linux", "x86_64") => Some("x86_64-unknown-linux-gnu"),
        ("linux", "aarch64") => Some("aarch64-unknown-linux-gnu"),
        ("macos", "x86_64") => Some("x86_64-apple-darwin"),
        ("macos", "aarch64") => Some("aarch64-apple-darwin"),
        _ => None,
    }
}

fn write_executable(path: &Path, body: &str) {
    fs::write(path, body).expect("write a fixture file");
    fs::set_permissions(path, fs::Permissions::from_mode(0o777)).expect("chmod a fixture file");
}

fn sha256_of(path: &Path) -> String {
    let tools: [(&str, &[&str]); 2] = [("sha256sum", &[]), ("shasum", &["-a", "256"])];
    for (tool, args) in tools {
        if let Ok(out) = Command::new(tool).args(args).arg(path).output() {
            if out.status.success() {
                let text = String::from_utf8(out.stdout).expect("checksum output is UTF-8");
                return text
                    .split_whitespace()
                    .next()
                    .expect("a checksum")
                    .to_string();
            }
        }
    }
    panic!("neither sha256sum nor shasum is available");
}

/// Mode bits of `path` that let group or others write it.
fn group_or_other_write(path: &Path) -> u32 {
    fs::symlink_metadata(path)
        .expect("stat an installed entry")
        .permissions()
        .mode()
        & 0o022
}

/// Every entry under `dir`, `dir` included.
fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
    out.push(dir.to_path_buf());
    if dir.is_dir() {
        for entry in fs::read_dir(dir).expect("read an installed folder") {
            walk(&entry.expect("an entry").path(), out);
        }
    }
}

struct Installed {
    _scratch: tempfile::TempDir,
    install_dir: PathBuf,
    home: PathBuf,
    record: PathBuf,
}

/// Build a release archive of this `cerulion` and a stand-in harness whose
/// tree any user could write, and install it with the repository's
/// installer under umask 000.
fn install_under_umask_000(target: &str) -> Installed {
    let scratch = tempfile::tempdir().expect("a scratch folder");
    let root = scratch.path();
    let stem = format!("cerulion-0.1.0-{target}");
    let fixture = root.join("fixture");
    let stage = fixture.join(&stem);
    let harness = stage.join("harness");
    fs::create_dir_all(&harness).expect("the archive tree");
    fs::copy(env!("CARGO_BIN_EXE_cerulion"), stage.join("cerulion")).expect("copy cerulion");
    fs::set_permissions(stage.join("cerulion"), fs::Permissions::from_mode(0o755))
        .expect("chmod cerulion");
    for program in ["cerulion-netd", "cerulion-connectd", "cerulion-remoted"] {
        write_executable(&stage.join(program), "#!/bin/sh\nexit 0\n");
    }
    fs::write(stage.join("LICENSE"), "license\n").expect("write LICENSE");
    // The launcher records how it was started; the install script only has to
    // exist for the folder to hold a harness.
    write_executable(
        &harness.join("rcp.sh"),
        "#!/bin/sh\nprintf '%s\\n' \"$0\" \"$@\" > \"$CERULION_TEST_HARNESS_RECORD\"\nexit 0\n",
    );
    write_executable(&harness.join("install.sh"), "#!/bin/sh\nexit 0\n");
    fs::set_permissions(&harness, fs::Permissions::from_mode(0o777)).expect("chmod harness/");

    let release = fixture.join("v0.1.0");
    fs::create_dir_all(&release).expect("the release folder");
    let archive = release.join(format!("{stem}.tar.gz"));
    let tar = Command::new("tar")
        .arg("-czf")
        .arg(&archive)
        .arg("-C")
        .arg(&fixture)
        .arg(&stem)
        .env("COPYFILE_DISABLE", "1")
        .status()
        .expect("run tar");
    assert!(tar.success(), "tar failed");
    fs::write(
        release.join(format!("{stem}.tar.gz.sha256")),
        format!("{}\n", sha256_of(&archive)),
    )
    .expect("write the checksum");

    let install_dir = root.join("install");
    let home = root.join("home");
    fs::create_dir_all(&home).expect("a home");
    let installer = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/scripts/install.sh");
    let out = Command::new("sh")
        .arg("-c")
        .arg("umask 000 && exec sh \"$0\" --version v0.1.0 --dir \"$1\" --base-url \"$2\"")
        .arg(&installer)
        .arg(&install_dir)
        .arg(format!("file://{}", fixture.display()))
        .env("HOME", &home)
        .env("CERULION_NO_MODIFY_PATH", "1")
        .env_remove("CERULION_INSTALL_DIR")
        .env_remove("CERULION_HOME")
        .output()
        .expect("run install.sh");
    assert!(
        out.status.success(),
        "install.sh failed: {}\n{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    Installed {
        record: root.join("launcher-record"),
        _scratch: scratch,
        install_dir,
        home,
    }
}

fn run_verify(installed: &Installed, program: &Path) -> std::process::Output {
    Command::new(program)
        .args(["rcp", "bench", "--verify"])
        .current_dir(&installed.home)
        .env("HOME", &installed.home)
        .env("CERULION_HOME", installed.home.join(".cerulion"))
        .env("CERULION_LOGIN_GATE", "off")
        .env("CERULION_TEST_HARNESS_RECORD", &installed.record)
        .env_remove("CERULION_HARNESS_DIR")
        .output()
        .expect("run the installed cerulion")
}

#[test]
fn the_installed_harness_runs_as_the_packaged_folder_and_not_from_a_directory_every_user_can_write()
{
    let Some(target) = release_target() else {
        eprintln!("no release archive for this host; nothing to install");
        return;
    };
    let installed = install_under_umask_000(target);

    let mut entries = Vec::new();
    walk(&installed.install_dir.join("harness"), &mut entries);
    entries.push(installed.install_dir.clone());
    for entry in &entries {
        assert_eq!(
            group_or_other_write(entry),
            0,
            "group or others can write {}",
            entry.display()
        );
    }

    // Run it the way a user does: through a link on the PATH, which the CLI
    // resolves to the real binary before it looks beside it.
    let link_dir = installed.home.join("bin");
    fs::create_dir_all(&link_dir).expect("a link folder");
    let link = link_dir.join("cerulion");
    std::os::unix::fs::symlink(installed.install_dir.join("cerulion"), &link)
        .expect("link cerulion");
    let out = run_verify(&installed, &link);
    assert!(
        out.status.success(),
        "cerulion rcp bench --verify failed: {}\n{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    let record = fs::read_to_string(&installed.record).expect("the launcher ran");
    let words: Vec<&str> = record.lines().collect();
    let install_dir = fs::canonicalize(&installed.install_dir).expect("canonical install dir");
    let copy_home = installed
        .home
        .join(".cerulion")
        .join("harness")
        .join(env!("CARGO_PKG_VERSION"));
    let want_launcher = install_dir.join("harness").join("rcp.sh");
    assert_eq!(
        words.len(),
        5,
        "the launcher was started with other arguments: {words:?}"
    );
    assert_eq!(
        words[0],
        want_launcher.to_str().expect("a UTF-8 path"),
        "the launcher was not started from the folder beside the installed cerulion"
    );
    assert_eq!(words[1], "--home");
    // The copy home may be spelled as given or canonicalized; either names the
    // folder under this run's Cerulion config directory.
    let canonical_home = fs::canonicalize(&installed.home)
        .expect("canonical home")
        .join(".cerulion")
        .join("harness")
        .join(env!("CARGO_PKG_VERSION"));
    assert!(
        words[2] == copy_home.to_str().expect("a UTF-8 path")
            || words[2] == canonical_home.to_str().expect("a UTF-8 path"),
        "the copy home is {}, not under this run's config directory",
        words[2]
    );
    assert_eq!(
        words[3], "--packaged",
        "the folder was not run as the packaged one"
    );
    assert_eq!(words[4], "verify");

    // The control: the same install, with its directory open to every user,
    // is not taken, and the launcher never starts.
    fs::remove_file(&installed.record).expect("clear the launcher record");
    fs::set_permissions(&installed.install_dir, fs::Permissions::from_mode(0o777))
        .expect("open the install directory to others");

    let out = run_verify(&installed, &link);
    assert!(
        !out.status.success(),
        "cerulion rcp ran a harness beside a binary in a directory others can write"
    );
    assert!(
        !installed.record.exists(),
        "the launcher ran from a directory others can write"
    );
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("no Cerulion harness found"),
        "the refusal did not say no harness was found: {stderr}"
    );
    fs::set_permissions(&installed.install_dir, fs::Permissions::from_mode(0o755))
        .expect("close the install directory again");
}
