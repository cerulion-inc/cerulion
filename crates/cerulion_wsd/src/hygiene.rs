//! Unix-socket lifecycle and single-daemon hygiene for `cerulion-wsd`.
//!
//! The implementation is [`cerulion_hygiene`] (shared with `cerulion-netd` and
//! `cerulion-vizd`, and with every client that resolves the path): this module
//! binds it to wsd's identity, [`cerulion_hygiene::WSD`], and converts the
//! bound listener to tokio's. The ladder, the flock-held singleton, stale-socket
//! recovery and the socket-directory rule are documented THERE — see
//! [`cerulion_hygiene`]'s module docs and USER_API's socket-path cell; do not
//! restate the rule here. A second copy is exactly how a rule drifts: a copy here and
//! one in `AGENTS.md` can both go on describing an older, owner-first shape after the
//! shared crate has changed it, and `no_wsd_doc_restates_the_socket_directory_rule`
//! fails if either grows one.

use std::io;
use std::path::PathBuf;

pub use cerulion_hygiene::{pidfile_for, SocketGuard};

/// Env var naming the socket path (absolute); the first rung of the ladder.
pub const SOCKET_ENV: &str = cerulion_hygiene::WSD.socket_env();

/// Resolve the daemon socket: [`SOCKET_ENV`], else
/// `$XDG_RUNTIME_DIR/cerulion/wsd.sock`, else `$HOME/.cerulion/wsd.sock`,
/// else `/tmp/cerulion-<euid>/wsd.sock`.
pub fn default_socket_path() -> PathBuf {
    cerulion_hygiene::WSD.default_socket_path()
}

/// Bind a private socket while holding the pidfile lock, as a tokio listener.
/// See [`cerulion_hygiene::DaemonSocket::acquire_socket`] for the contract.
pub fn acquire_socket(socket: PathBuf) -> io::Result<(tokio::net::UnixListener, SocketGuard)> {
    let (listener, guard) = cerulion_hygiene::WSD.acquire_socket(socket)?;
    // A failure here drops `guard`, which unlinks the socket + pidfile.
    listener.set_nonblocking(true)?;
    let listener = tokio::net::UnixListener::from_std(listener)?;
    Ok((listener, guard))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_env_var_is_wsds_own() {
        assert_eq!(SOCKET_ENV, "CERULION_WSD_SOCKET");
    }

    /// The tokio conversion really binds through the shared crate and the
    /// guard still owns the files.
    #[tokio::test]
    async fn acquire_through_the_wrapper_yields_a_tokio_listener_and_a_live_guard() {
        let dir = std::env::temp_dir().join(format!("cer_wsd_wrap_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let socket = dir.join("wsd.sock");
        let (listener, guard) = acquire_socket(socket.clone()).expect("acquire");
        assert_eq!(
            listener.local_addr().unwrap().as_pathname(),
            Some(socket.as_path())
        );
        let err = acquire_socket(socket.clone()).expect_err("singleton");
        assert_eq!(err.kind(), io::ErrorKind::AddrInUse);
        assert!(err.to_string().contains("cerulion-wsd"), "{err}");
        drop(listener);
        drop(guard);
        assert!(!socket.exists() && !pidfile_for(&socket).exists());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
