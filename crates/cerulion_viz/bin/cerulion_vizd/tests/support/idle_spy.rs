// SPDX-License-Identifier: AGPL-3.0-only
//! Idle holding sockets stay open until EOF or fixture shutdown.

use std::io::{BufRead, ErrorKind};
use std::sync::atomic::{AtomicBool, Ordering};

pub fn read_request(reader: &mut impl BufRead, running: &AtomicBool) -> Option<String> {
    let mut line = String::new();
    loop {
        match reader.read_line(&mut line) {
            Ok(0) => {
                assert!(
                    line.is_empty(),
                    "spy client closed a partial request: {line:?}"
                );
                return None;
            }
            Ok(_) => return Some(line),
            Err(error) if matches!(error.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {
                if !running.load(Ordering::Acquire) {
                    return None;
                }
                // An idle source holds its mirror without sending heartbeats.
                // Retain partial framing and wait for its next request or EOF.
            }
            Err(error) => panic!("spy request read failed: {error}"),
        }
    }
}
