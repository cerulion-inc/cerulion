// SPDX-License-Identifier: AGPL-3.0-only
//! Daemon hygiene for `cerulion-netd`: the socket-path ladder, the `flock`-held
//! single-daemon lock, stale-socket recovery and the live-daemon refusal.
//!
//! The implementation is [`cerulion_hygiene`] (shared with `cerulion-vizd` and
//! `cerulion-wsd`, and with every client that resolves the path): this module
//! only binds it to netd's identity, [`cerulion_hygiene::NETD`]. The control
//! socket is a well-known path every consumer resolves the same way, so the
//! FIRST consumer spawns netd there and every later one connects to the running
//! daemon; the flock (taken before any socket surgery) is what makes two
//! consumers racing to spawn elect exactly one winner — the loser gets the
//! `AddrInUse` refusal and connects instead.

use std::io;
use std::os::unix::net::UnixListener;
use std::path::PathBuf;

pub use cerulion_hygiene::{pidfile_for, read_pidfile, SocketGuard};

/// The env var overriding the control-socket path (absolute). Set by tests to
/// point at a temp socket, and by an operator running a non-default instance.
pub const SOCKET_ENV: &str = cerulion_hygiene::NETD.socket_env();

/// Resolve the default control-socket path: [`SOCKET_ENV`] if set, else
/// `$XDG_RUNTIME_DIR/cerulion/netd.sock`, else `$HOME/.cerulion/netd.sock`, else
/// `/tmp/cerulion-<euid>/netd.sock`.
pub fn default_socket_path() -> PathBuf {
    cerulion_hygiene::NETD.default_socket_path()
}

/// Acquire the control socket at `socket` as THE netd for that path — see
/// [`cerulion_hygiene::DaemonSocket::acquire_socket`].
pub fn acquire_socket(socket: PathBuf) -> io::Result<(UnixListener, SocketGuard)> {
    cerulion_hygiene::NETD.acquire_socket(socket)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    struct EnvGuard(&'static str, Option<String>);
    impl EnvGuard {
        fn set(key: &'static str, val: &str) -> Self {
            let prev = std::env::var(key).ok();
            std::env::set_var(key, val);
            EnvGuard(key, prev)
        }
    }
    impl Drop for EnvGuard {
        fn drop(&mut self) {
            match &self.1 {
                Some(v) => std::env::set_var(self.0, v),
                None => std::env::remove_var(self.0),
            }
        }
    }

    /// This test both WRITES the socket env var and READS the
    /// process env through `default_socket_path` (which falls back to
    /// `XDG_RUNTIME_DIR`, then `HOME` — vars the sibling `wan.rs` desk-path tests
    /// set), so it takes the CRATE-WIDE env lock.
    #[test]
    fn socket_env_override_wins_and_is_netds_own() {
        let _lock = crate::test_env::env_lock();
        assert_eq!(SOCKET_ENV, "CERULION_NETD_SOCK");
        let _g = EnvGuard::set(SOCKET_ENV, "/tmp/cerulion_netd_test_override.sock");
        assert_eq!(
            default_socket_path(),
            PathBuf::from("/tmp/cerulion_netd_test_override.sock")
        );
    }

    #[test]
    fn the_default_path_ends_in_netd_sock() {
        let _lock = crate::test_env::env_lock();
        let _g = EnvGuard::set(SOCKET_ENV, "");
        assert_eq!(
            default_socket_path().file_name(),
            Some(std::ffi::OsStr::new("netd.sock"))
        );
        assert_eq!(
            pidfile_for(Path::new("/run/cerulion/netd.sock")),
            PathBuf::from("/run/cerulion/netd.pid")
        );
    }

    /// The daemon-facing wrapper really binds through the shared crate (a wrapper
    /// that resolved but never acquired would pass every other test).
    #[test]
    fn acquire_through_the_wrapper_binds_and_refuses_a_second() {
        let dir = std::env::temp_dir().join(format!("cer_netd_wrap_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let socket = dir.join("netd.sock");
        let (_listener, guard) = acquire_socket(socket.clone()).expect("acquire");
        assert_eq!(
            read_pidfile(&pidfile_for(&socket)),
            Some(std::process::id())
        );
        let err = acquire_socket(socket.clone()).expect_err("second refused");
        assert_eq!(err.kind(), io::ErrorKind::AddrInUse);
        assert!(err.to_string().contains("cerulion-netd"), "{err}");
        drop(guard);
        assert!(!socket.exists());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
