// SPDX-License-Identifier: AGPL-3.0-only
//! The transport seam: "framed bidirectional bytes" plus a Unix-domain-socket
//! dev transport for tests and the dev binary.
//!
//! The [`OpsListener`] trait is transport-agnostic — the future iroh
//! `cerulion/ops/1` transport implements the SAME trait and yields the SAME
//! [`OpsConnection`]. The protocol framing + version negotiation live one
//! layer up in [`crate::protocol`], so every transport shares them.
//!
//! # Authenticated identity
//!
//! Each accepted connection carries a [`CallerIdentity`]. The TRANSPORT is
//! responsible for authenticating the peer: iroh yields a verified node id;
//! the dev Unix-socket transport is **unauthenticated** and yields
//! [`CallerIdentity::local_dev`] (`authenticated = false`). The
//! [`crate::authz`] seam then decides what a caller may do — until a pairing
//! access-list is configured, the deny-by-default authorizer refuses **every**
//! peer (authenticated or not), not merely unauthenticated ones.
//!
//! # Why the seam is synchronous (the transport-seam decision)
//!
//! [`OpsListener`] / [`OpsStream`] are deliberately **synchronous** (`Read +
//! Write`), and `cerud` pulls no async runtime for the skeleton:
//!
//! - **Verbs are short-lived mechanical operations** (an `inventory` probe, a
//!   bounded `log-tail`, a `systemd-run` spawn) — there is no long-lived
//!   streaming workload that would benefit from async back-pressure here.
//! - **The dev transport is `std`** (`std::os::unix::net`), so a sync seam is
//!   the natural fit and keeps the dependency surface minimal.
//! - The later **iroh transport bridges async QUIC to this sync seam**: the
//!   iroh chunk owns the tokio accept loop, and each accepted connection is
//!   serviced against this seam either via `spawn_blocking` or a
//!   channel-backed `OpsStream` impl (an adapter whose `Read`/`Write` move
//!   bytes across an async channel). The server logic ([`crate::server`]) stays
//!   runtime-agnostic.
//! - **QUIC stream cardinality**: one bidirectional QUIC stream carries one
//!   request/response *session* (the handshake + the request loop), one
//!   connection per client. The framing + version negotiation in
//!   [`crate::protocol`] are transport-independent, so they are unchanged by
//!   which transport (Unix socket today, iroh later) carries the bytes.

use std::io::{Read, Write};

use crate::error::CerudResult;

/// A bidirectional byte stream to one connected peer. Any `Read + Write + Send`
/// type qualifies, so a transport hands the server a `Box<dyn OpsStream>`
/// without the server knowing which transport it is.
pub trait OpsStream: Read + Write + Send {}
impl<T: Read + Write + Send + ?Sized> OpsStream for T {}

/// The authenticated identity of a connected peer, supplied by the transport.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CallerIdentity {
    /// A stable identifier for the peer (an iroh node id, or `"local-dev"`).
    pub id: String,
    /// Whether the transport cryptographically authenticated this peer. The
    /// dev Unix-socket transport sets this to `false`.
    pub authenticated: bool,
}

impl CallerIdentity {
    /// The unauthenticated local-dev identity yielded by the Unix-socket
    /// transport.
    pub fn local_dev() -> Self {
        CallerIdentity {
            id: "local-dev".to_string(),
            authenticated: false,
        }
    }

    /// A verified peer identity (the shape the iroh transport will yield).
    pub fn verified(id: impl Into<String>) -> Self {
        CallerIdentity {
            id: id.into(),
            authenticated: true,
        }
    }
}

/// One accepted connection: the byte stream plus who is on the other end.
pub struct OpsConnection {
    pub stream: Box<dyn OpsStream>,
    pub caller: CallerIdentity,
}

/// Server-side transport: accepts inbound connections. Transport-agnostic.
pub trait OpsListener {
    /// Block until a peer connects, returning the connection. Errors are
    /// transport-level (the socket is gone); a clean shutdown is handled by
    /// the caller dropping the listener.
    fn accept(&mut self) -> CerudResult<OpsConnection>;

    /// A human-readable description of where this listener is bound (for logs).
    fn local_addr(&self) -> String;
}

// ─────────────────────────── Unix-domain-socket dev transport ──────────────
// The dev transport is Unix-only (iroh is the cross-platform production path).

#[cfg(unix)]
pub use unix_socket::{connect_unix, UnixSocketListener};

#[cfg(unix)]
mod unix_socket {
    use std::os::unix::net::{UnixListener, UnixStream};
    use std::path::{Path, PathBuf};

    use crate::error::CerudResult;

    use super::{CallerIdentity, OpsConnection, OpsListener};

    /// A dev [`OpsListener`] over a Unix domain socket. Unauthenticated: every
    /// accepted connection is [`CallerIdentity::local_dev`].
    pub struct UnixSocketListener {
        listener: UnixListener,
        path: PathBuf,
    }

    impl UnixSocketListener {
        /// Bind a fresh socket at `path`, removing any stale socket file first.
        ///
        /// Hardens permissions: if `cerud` has to CREATE the socket's parent
        /// directory, it creates it `0o700` (owner-only); and the bound socket
        /// itself is set `0o600` so only the owner can connect. An existing
        /// parent directory is left untouched (chmod-ing a shared dir like
        /// `/tmp` would be wrong).
        pub fn bind(path: impl AsRef<Path>) -> CerudResult<Self> {
            let path = path.as_ref().to_path_buf();

            // Create a missing parent dir as owner-only (0o700), tightening
            // EVERY component we create (not just the leaf). An existing parent
            // is deliberately NOT re-chmod'd.
            if let Some(parent) = path.parent() {
                if !parent.as_os_str().is_empty() && !parent.exists() {
                    create_dir_all_owner_only(parent)?;
                }
            }

            // Remove a stale socket file so re-binding is idempotent. Ignore a
            // not-found error; surface anything else (e.g. permissions).
            match std::fs::remove_file(&path) {
                Ok(()) => {}
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                Err(e) => return Err(e.into()),
            }
            let listener = UnixListener::bind(&path)?;
            // Owner-only socket: only our uid may connect to the dev transport.
            set_mode(&path, 0o600)?;
            Ok(UnixSocketListener { listener, path })
        }
    }

    /// Set a path's Unix permission bits.
    fn set_mode(path: &Path, mode: u32) -> CerudResult<()> {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(mode))?;
        Ok(())
    }

    /// Create `dir` and every missing ancestor, setting EACH newly-created
    /// component to owner-only `0o700`. Pre-existing directories are left
    /// untouched (chmod-ing a shared ancestor like `/var` or `/home` would be
    /// wrong). This is the state-root creation path — the tree holds the trust
    /// store + audit receipts — so an intermediate directory a plain
    /// `create_dir_all` would create must NOT inherit a world-readable umask
    /// default: every component WE create is tightened, not just the leaf.
    ///
    /// Each component is created with `DirBuilder::mode(0o700)` (the mkdir(2)
    /// mode, itself umask-masked so it can only be tighter than 0o700) and then
    /// pinned to exactly `0o700` with an explicit `set_mode` — umask-proof, and
    /// narrowing the create→chmod window to at worst 0o700.
    fn create_dir_all_owner_only(dir: &Path) -> CerudResult<()> {
        use std::os::unix::fs::DirBuilderExt;

        // Walk up collecting the missing tail (deepest first), stopping at the
        // first existing (or empty/root) ancestor so pre-existing dirs are never
        // touched.
        let mut missing: Vec<&Path> = Vec::new();
        let mut cur = Some(dir);
        while let Some(p) = cur {
            if p.as_os_str().is_empty() || p.exists() {
                break;
            }
            missing.push(p);
            cur = p.parent();
        }

        // Create shallowest-first so each parent exists before its child.
        for p in missing.iter().rev() {
            match std::fs::DirBuilder::new().mode(0o700).create(p) {
                Ok(()) => {}
                // A concurrent creator may have won the race — tolerate it, then
                // still pin the mode below (the dir now exists either way).
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {}
                Err(e) => return Err(e.into()),
            }
            set_mode(p, 0o700)?;
        }
        Ok(())
    }

    impl OpsListener for UnixSocketListener {
        fn accept(&mut self) -> CerudResult<OpsConnection> {
            let (stream, _addr) = self.listener.accept()?;
            Ok(OpsConnection {
                stream: Box::new(stream),
                caller: CallerIdentity::local_dev(),
            })
        }

        fn local_addr(&self) -> String {
            format!("unix:{}", self.path.display())
        }
    }

    impl Drop for UnixSocketListener {
        fn drop(&mut self) {
            // Best-effort cleanup of the socket file on shutdown.
            let _ = std::fs::remove_file(&self.path);
        }
    }

    /// Connect to a Unix-socket dev transport (the client half, used by the
    /// end-to-end tests and any dev tooling).
    pub fn connect_unix(path: impl AsRef<Path>) -> CerudResult<UnixStream> {
        Ok(UnixStream::connect(path)?)
    }
}
