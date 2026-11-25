// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use std::io::{BufRead, Read, Write};
use std::os::unix::process::ExitStatusExt;
use std::process::{Child, Command, Stdio};

const HELPER_ENV: &str = "CERULION_TEST_PROCESS_SESSION_HELPER";
const READY: &str = "OWNED_SESSION_READY";

struct OwnedChild(Child);

impl Drop for OwnedChild {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[test]
fn owned_session_helper() {
    if std::env::var_os(HELPER_ENV).is_none() {
        return;
    }
    let sid = rustix::process::setsid().unwrap();
    assert_eq!(sid, rustix::process::getpid());
    let mut output = std::io::stdout().lock();
    // End any libtest prefix before the independently matched readiness line.
    writeln!(output, "\n{READY}").unwrap();
    output.flush().unwrap();
    // The parent retains the pipe, so only cleanup or a failed parent ends this wait.
    std::io::stdin().lock().read_exact(&mut [0]).unwrap();
    panic!("session leader received input instead of its cleanup signal");
}

fn session_leader() -> OwnedChild {
    let mut child = OwnedChild(
        Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "remote_session::processes::tests::owned_session_helper",
                "--nocapture",
                "--test-threads=1",
            ])
            .env(HELPER_ENV, "1")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .unwrap(),
    );
    let output = child.0.stdout.take().unwrap();
    let (send, receive) = std::sync::mpsc::sync_channel(1);
    let reader = std::thread::spawn(move || {
        let result = (|| {
            for line in std::io::BufReader::new(output).lines() {
                if line? == READY {
                    return Ok(());
                }
            }
            Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "session helper exited before readiness",
            ))
        })();
        let _ = send.send(result);
    });
    let ready = receive.recv_timeout(Duration::from_secs(10));
    if !matches!(&ready, Ok(Ok(()))) {
        // Closing the only child's output also releases the reader on refusal.
        let _ = child.0.kill();
        let _ = child.0.wait();
    }
    reader.join().unwrap();
    ready.expect("session readiness deadline").unwrap();
    child
}

#[test]
fn cleanup_kills_the_owned_session_and_preserves_an_unrelated_child() {
    for force_fallback in [false, true] {
        let mode = preflight(force_fallback).unwrap();
        if force_fallback {
            assert!(matches!(mode, Mode::Revalidated));
        }
        let mut unrelated = OwnedChild(
            Command::new("/bin/cat")
                .stdin(Stdio::piped())
                .stdout(Stdio::null())
                .spawn()
                .unwrap(),
        );
        let mut leader = session_leader();
        let sid = Pid::from_raw(leader.0.id() as i32).unwrap();
        assert_eq!(rustix::process::getsid(Some(sid)).unwrap(), sid);
        assert_ne!(rustix::process::getsid(None).unwrap(), sid);
        let unrelated_pid = Pid::from_raw(unrelated.0.id() as i32).unwrap();
        assert_ne!(rustix::process::getsid(Some(unrelated_pid)).unwrap(), sid);
        assert!(leader.0.try_wait().unwrap().is_none());
        assert!(unrelated.0.try_wait().unwrap().is_none());

        // Keep the leader unreaped throughout the real, bounded cleanup call.
        terminate_session(sid, mode).unwrap();
        assert_eq!(leader.0.wait().unwrap().signal(), Some(libc::SIGKILL));
        assert!(unrelated.0.try_wait().unwrap().is_none());
    }
}

#[test]
fn revalidation_requires_both_session_and_birth_identity() {
    let sid = Pid::from_raw(17).unwrap();
    let other = Pid::from_raw(18).unwrap();
    assert!(same_process(sid, sid, [42, 0], [42, 0]));
    assert!(!same_process(sid, other, [42, 0], [42, 0]));
    assert!(!same_process(sid, sid, [42, 0], [43, 0]));
    assert!(!same_process(sid, sid, [42, 0], [42, 1]));
}
