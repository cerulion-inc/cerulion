// SPDX-License-Identifier: AGPL-3.0-only
//! The real-binary tests' stderr helpers: a reader that can stop reading a
//! daemon's stderr after a given line, a filler that makes that pipe take no
//! more bytes, a bounded search of the lines read, and a child that is killed
//! on drop. Unix only: the filler sets the pipe's file status flags.

use std::io::{BufRead, BufReader, PipeWriter, Read, Write};
use std::process::Child;
use std::sync::mpsc::{self, RecvTimeoutError};
use std::time::{Duration, Instant};

/// A child process, killed and reaped on drop.
pub struct KillOnDrop(pub Child);

impl Drop for KillOnDrop {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Send each line of `stderr` to `lines`. With `hold`, stop reading after the
/// first line containing `hold_after`, the pipe still open, until the hold's
/// sender is dropped.
pub fn pump(
    stderr: impl Read,
    lines: mpsc::Sender<String>,
    hold: Option<mpsc::Receiver<()>>,
    hold_after: &str,
) {
    let mut held = false;
    for line in BufReader::new(stderr).lines().map_while(Result::ok) {
        let reached = !held && line.contains(hold_after);
        if lines.send(line).is_err() {
            return;
        }
        if let (true, Some(hold)) = (reached, &hold) {
            held = true;
            let _ = hold.recv();
        }
    }
}

/// Fill the pipe behind `writer` until it takes no more bytes. The write end
/// is non-blocking only while it fills: its file status flags are shared with
/// the daemon's stderr, which then blocks on its next write.
pub fn fill(writer: &PipeWriter) {
    use std::os::fd::AsRawFd;
    let fd = writer.as_raw_fd();
    // SAFETY: fcntl(2) reads and sets the flags of a descriptor this test owns.
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
    assert!(flags >= 0, "F_GETFL: {}", std::io::Error::last_os_error());
    // SAFETY: as above.
    assert_eq!(
        unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) },
        0
    );
    // Large chunks first, so a pipe that grows on a large write grows to its
    // largest size before it is full.
    let mut writer = writer;
    for chunk in [65_536, 1_024, 1] {
        let bytes = vec![b'x'; chunk];
        loop {
            match writer.write(&bytes) {
                Ok(_) => {}
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => panic!("filling the pipe: {e}"),
            }
        }
    }
    // SAFETY: as above.
    assert_eq!(unsafe { libc::fcntl(fd, libc::F_SETFL, flags) }, 0);
}

/// The first line of `log` at or past index `from` that `matches`, reading
/// more from `lines` into `log` while none does, within `bound`.
pub fn read_matching(
    lines: &mpsc::Receiver<String>,
    log: &mut Vec<String>,
    from: usize,
    bound: Duration,
    matches: impl Fn(&str) -> bool,
) -> String {
    let until = Instant::now() + bound;
    loop {
        if let Some(line) = log.iter().skip(from).find(|l| matches(l)) {
            return line.clone();
        }
        let left = until.saturating_duration_since(Instant::now());
        match lines.recv_timeout(left) {
            Ok(line) => log.push(line),
            Err(RecvTimeoutError::Timeout | RecvTimeoutError::Disconnected) => panic!(
                "no matching line past {from} within {bound:?}; log:\n{}",
                log.join("\n")
            ),
        }
    }
}
