// SPDX-License-Identifier: AGPL-3.0-only
//! One Unix-socket lifecycle for every Cerulion desk daemon and its clients.
//!
//! `cerulion-netd`, `cerulion-vizd` and `cerulion-wsd` each listen on a Unix socket,
//! and the clients that connect to them (the `cerulion` CLI, Studio) must find the
//! same path and trust the same directory. This crate is where both sides get that
//! from. It is an internal building block, published because the daemons and the
//! CLI depend on it; a user meets it only through the socket environment variables
//! in `docs/user-api.md`.
//!
//! A daemon is described by a [`DaemonSocket`] (its name, binary and socket
//! environment variable); the three that exist are the constants [`NETD`], [`VIZD`]
//! and [`WSD`]. Everything a daemon and its clients must agree on comes from that
//! one value:
//!
//! * **The socket-path ladder** ([`DaemonSocket::default_socket_path`]): the
//!   environment variable if set, else `$XDG_RUNTIME_DIR/cerulion/<name>.sock`, else
//!   `$HOME/.cerulion/<name>.sock`, else `/tmp/cerulion-<euid>/<name>.sock`. The
//!   last rung is per user, so two users on one machine never share a directory.
//! * **The singleton** ([`DaemonSocket::acquire_socket`]): an exclusive,
//!   non-blocking `flock` on the pidfile beside the socket, taken before the socket
//!   is touched, so two starts racing on one path can never both call the socket
//!   stale and clobber each other. The kernel drops the lock when the process dies,
//!   so a crashed daemon needs no PID liveness probe: a leftover socket under a free
//!   lock is stale by construction and is recovered.
//! * **The socket-directory rule** ([`socket_dir_verdict`]), below.
//!
//! The daemons keep a thin `hygiene` module each (their historical names and
//! environment variables are the public contract); clients resolve the path through
//! the same constant, so they agree by construction.
//!
//! # The socket-directory rule
//!
//! The directory is checked, never trusted, because the last rung lives under
//! world-writable `/tmp`, and anyone who can write into the socket's directory can
//! unlink the socket and bind an impostor at the same path. This is the code-side
//! statement of the rule; the `CERULION_NETD_SOCK` row of `docs/user-api.md` is the
//! user-facing one. It has four arms, decided in this order:
//!
//! 1. **Accept a sticky, world-writable directory owned by root or by us**: `/tmp`
//!    itself, or a `1777` directory of our own. The sticky bit stops every other
//!    user unlinking what we create, and the one user it does not stop is the
//!    directory's own owner (POSIX lets the file's owner, the directory's owner and
//!    root unlink), which on this arm is root or us. This arm comes before the
//!    ownership arm so that a daemon running as root never re-modes the machine's
//!    `/tmp` from `1777` to `1755` for every other user.
//! 2. **A directory of ours: accept it as is, and if its group or others could
//!    write into it, strip those bits first.** The write bits go (`0775` becomes
//!    `0755`; the sticky and setgid bits, if any, are kept) and one `warn!` says
//!    so. The user runs nothing, on the default ladder and on an explicitly
//!    configured path alike. With nothing to strip, nothing is touched and nothing
//!    is logged, which is the commonest outcome: every rung of the default ladder
//!    ends in a directory of ours, either the `0700` we create or one another
//!    Cerulion command created first under the ambient umask (`~/.cerulion`), used
//!    as is when that umask left no group or other write bit (`0755`) and tightened
//!    once, with the warn, when it did (`0775` under a `umask 0002`).
//! 3. **Refuse one of ours whose bits could not be stripped** (a filesystem with a
//!    fixed mode, such as a CIFS home mounted `dir_mode=0777`), with the reason.
//! 4. **Refuse everything else**: a directory we neither own nor can rely on. That
//!    includes a sticky, world-writable share owned by another non-root user: the
//!    sticky bit does not bind its own owner, so that owner can unlink our socket
//!    and bind an impostor at the same path, which is exactly the attack this rule
//!    exists to stop. The refusal says so rather than vouching for the directory.
//!
//! A missing directory is created `0700`. The pidfile is opened `O_NOFOLLOW`, and a
//! pre-existing pidfile or socket must be ours, so a planted symlink or file cannot
//! become the lock. The directory itself is opened once (`O_RDONLY | O_NOFOLLOW |
//! O_DIRECTORY`) and every later step goes through that descriptor: the verdict
//! reads `fstat`, and the tightening is `fchmod` on the same descriptor
//! ([`File::set_permissions`]), so a symlink swapped in between the check and the
//! change cannot redirect it.

// Maintainer notes. These continue the module docs above as plain comments, so the
// rendered front page states the rule while the reasoning stays next to the code.
//
// Where the rule is written down. It has exactly two statements: the module docs
// above (code side, where it is decided) and the socket-path row of
// `docs/user-api.md` (user facing). `tests/user_api_doc_test.rs` in this crate
// keeps the two in agreement. Nowhere else states it: the daemons POINT here (see
// `cerulion_wsd`'s `no_wsd_doc_restates_the_socket_directory_rule`).
//
// Why the ladder's last rung is per user. A shared `/tmp/cerulion/` made two users
// on one machine collide on the pidfile, and let either plant the directory first.
//
// Why arm 1 comes before the ownership arm. Deciding on ownership first would make
// a daemon that runs as root (or `cargo test` in a root container) "fix" the
// machine's `/tmp` from `1777` to `1755` for every other user. The impostor attack
// the rule stops was reproduced with two accounts sharing a group.
//
// Why the directory is judged through one descriptor (`open_socket_dir`, private:
// the rule is decided here, not configured). Deciding from a `stat` of the PATH and
// then chmod-ing the PATH would leave the inode judged and the inode changed
// unpinned: `chmod(2)` re-resolves the name and FOLLOWS symlinks, so whoever can
// write the socket directory's PARENT could swap a symlink in between. Because a
// tightened mode keeps read and execute bits, that is an arbitrary-chmod primitive
// (a root daemon: any path; a normal one: any file of that user's, such as a
// private SSH key re-moded to `0755`). The open also subsumes the "is it a
// directory?" question: a planted symlink or file cannot be opened this way at
// all, and its errno becomes the same refusal.
//
// RESIDUAL, written down so the choice is visible rather than assumed: the
// descriptor pins the DIRECTORY, not the path that names it. Somebody who can write
// an ANCESTOR of the socket directory can still redirect the path-based `bind` and
// pidfile create into a directory of their own; the descriptor-based tightening
// removes the arbitrary-chmod primitive, not that redirect. The complete answer is
// an ancestor walk (every component owned by root or by us and not writable by
// others unless sticky) and it is NOT implemented here. Every rung of the default
// ladder sits under `$XDG_RUNTIME_DIR`, `$HOME` or a per-user
// `/tmp/cerulion-<euid>`, whose ancestors are already ours, root's or sticky, so
// the walk would only ever bind on an explicitly configured `CERULION_*_SOCKET`
// path, which is exactly the case a maintainer adding one should weigh.

#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::path::{Path, PathBuf};

#[cfg(unix)]
use std::fs::{self, DirBuilder, File, OpenOptions};
#[cfg(unix)]
use std::io::{self, Seek, SeekFrom, Write};
#[cfg(unix)]
use std::os::fd::AsRawFd;
#[cfg(unix)]
use std::os::unix::fs::{DirBuilderExt, MetadataExt, OpenOptionsExt, PermissionsExt};
#[cfg(unix)]
use std::os::unix::net::UnixListener;

/// One daemon's socket identity: the pieces its clients and the daemon itself
/// must agree on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DaemonSocket {
    /// The socket's stem: `<name>.sock` / `<name>.pid`.
    name: &'static str,
    /// The binary's name, for messages.
    binary: &'static str,
    /// The env var that overrides the socket path (absolute).
    env: &'static str,
}

/// `cerulion-netd`, the per-computer network gateway daemon.
pub const NETD: DaemonSocket = DaemonSocket {
    name: "netd",
    binary: "cerulion-netd",
    env: "CERULION_NETD_SOCK",
};
/// `cerulion-vizd`, the desk visualization daemon.
pub const VIZD: DaemonSocket = DaemonSocket {
    name: "vizd",
    binary: "cerulion-vizd",
    env: "CERULION_VIZD_SOCK",
};
/// `cerulion-wsd`, the workspace-engine daemon Studio talks to.
pub const WSD: DaemonSocket = DaemonSocket {
    name: "wsd",
    binary: "cerulion-wsd",
    env: "CERULION_WSD_SOCKET",
};

impl DaemonSocket {
    /// The socket stem (`netd`, `vizd`, `wsd`).
    pub const fn name(&self) -> &'static str {
        self.name
    }

    /// The binary's name (`cerulion-netd`, ...).
    pub const fn binary(&self) -> &'static str {
        self.binary
    }

    /// The env var that overrides the socket path.
    pub const fn socket_env(&self) -> &'static str {
        self.env
    }

    /// Resolve the well-known socket path: the env var if set (non-empty), else
    /// `$XDG_RUNTIME_DIR/cerulion/<name>.sock`, else `$HOME/.cerulion/<name>.sock`,
    /// else `/tmp/cerulion-<euid>/<name>.sock`. Every consumer resolves the same
    /// way, so the first one to spawn the daemon and every later one find it here.
    pub fn default_socket_path(&self) -> PathBuf {
        let file = format!("{}.sock", self.name);
        if let Ok(explicit) = std::env::var(self.env) {
            if !explicit.is_empty() {
                return PathBuf::from(explicit);
            }
        }
        if let Ok(runtime) = std::env::var("XDG_RUNTIME_DIR") {
            if !runtime.is_empty() {
                return PathBuf::from(runtime).join("cerulion").join(file);
            }
        }
        if let Ok(home) = std::env::var("HOME") {
            if !home.is_empty() {
                return PathBuf::from(home).join(".cerulion").join(file);
            }
        }
        PathBuf::from(format!("/tmp/cerulion-{}", euid())).join(file)
    }

    /// Bind `socket` as THE daemon for that path: check/create its directory,
    /// take the pidfile flock, recover a stale socket, bind, restrict the socket
    /// to `0600`, and write our pid. The listener is returned OWNED (blocking;
    /// a tokio daemon sets non-blocking and converts it); the [`SocketGuard`]
    /// holds the lock and removes both files on drop.
    ///
    /// A daemon already holding the lock is reported as
    /// [`io::ErrorKind::AddrInUse`] naming the socket, its pid and the env var
    /// that runs a second instance elsewhere. What happens to the socket's
    /// DIRECTORY is the one directory rule — stated in this crate's module
    /// docs and decided by [`socket_dir_verdict`]; the arm that changes a
    /// directory of the user's is logged, and every arm that refuses one is
    /// [`io::ErrorKind::PermissionDenied`] with the reason.
    ///
    /// Every failure of the DIRECTORY-preparation step is reported as
    /// [`io::ErrorKind::PermissionDenied`] whatever its own errno — that
    /// covers every non-verdict arm too: a directory that could not be created
    /// (`ENOSPC`, `ENAMETOOLONG`), one that could not be inspected, a file or
    /// symlink planted where the directory should be, and the one verdict that
    /// can still fail on its own (a directory of ours whose bits could not be
    /// stripped) — because they all go through the same refusal. Every failure
    /// AFTER it (the pidfile and socket ownership checks, the pidfile open, the
    /// flock, the stale-socket removal, the bind, the `0600` chmod, the pidfile
    /// write) keeps its `std` kind and reason but is given the operation it
    /// failed, the path it failed on, and the env var that moves the socket: a
    /// daemon's `main` prints this error verbatim, and `File name too long (os
    /// error 63)` on its own names neither. That list is exhaustive by
    /// construction — every one of those steps reports through
    /// `socket_error`, the ownership checks included. The ONE failure
    /// after the directory step that does not is the live-daemon refusal
    /// above: it is [`io::ErrorKind::AddrInUse`] rather than the step's own
    /// kind, and it names the socket, the pid and the env var itself.
    #[cfg(unix)]
    pub fn acquire_socket(&self, socket: PathBuf) -> io::Result<(UnixListener, SocketGuard)> {
        if let Some(parent) = socket.parent() {
            prepare_socket_dir(parent, self.env)?;
        }
        let pidfile = pidfile_for(&socket);
        self.require_ours(&pidfile, "pidfile")?;
        self.require_ours(&socket, "socket")?;
        // `require_ours` stats the pidfile and this opens it, but the open
        // carries `O_NOFOLLOW`, so a symlink swapped in after the stat fails
        // the open instead of being followed — and every later write goes
        // through THIS descriptor (`write_pidfile` takes `&File`, and the flock
        // is on the same open file description), never through the path again.
        let lock = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .custom_flags(libc::O_NOFOLLOW)
            .open(&pidfile)
            .map_err(|e| self.socket_error("could not open the daemon pidfile", &pidfile, e))?;
        if let Err(error) = flock_exclusive_nonblocking(&lock) {
            if error.kind() == io::ErrorKind::WouldBlock {
                let pid = read_pidfile(&pidfile)
                    .map(|pid| pid.to_string())
                    .unwrap_or_else(|| "unknown".to_string());
                return Err(io::Error::new(
                    io::ErrorKind::AddrInUse,
                    format!(
                        "another {} daemon is already running (or starting) at {} (pid {pid}) — \
                         connect to it instead of spawning a second one, or set {} to run a \
                         separate instance on a different socket",
                        self.binary,
                        socket.display(),
                        self.env
                    ),
                ));
            }
            // Anything else (EINTR, an ENOLCK on a filesystem without `flock`,
            // an NFS mount) WAS the last path-less error in this function: the
            // daemon dies printing it verbatim, and "Operation not supported"
            // alone names neither the pidfile nor the way out — so it is
            // wrapped like the rest, naming the pidfile and the env var.
            return Err(self.socket_error("could not lock the daemon pidfile", &pidfile, error));
        }

        // We hold the lock, so any leftover socket is stale (a live daemon would
        // still hold it): recover it. Only the lock holder ever does this.
        //
        // A stat followed by a mutation of the same PATH, like the directory
        // step above — but not the same hazard, on two counts. `unlink(2)` does
        // not follow symlinks (and neither did the `symlink_metadata` that
        // vouched for the name), so the object removed is the name itself, not
        // a target somebody redirected it at; and it REMOVES an entry rather
        // than granting anything, so the worst a lost race costs is unlinking
        // an entry that appeared after the check — in a directory the rule
        // above has already established only root or we can create or replace
        // entries in.
        if fs::symlink_metadata(&socket).is_ok() {
            tracing::warn!(
                daemon = self.binary,
                path = %socket.display(),
                "recovering a leftover control socket (previous daemon gone) — rebinding"
            );
            fs::remove_file(&socket).map_err(|e| {
                self.socket_error("could not remove the leftover control socket", &socket, e)
            })?;
        }
        let listener = UnixListener::bind(&socket)
            .map_err(|e| self.socket_error("could not bind the control socket", &socket, e))?;
        let cleanup = || {
            let _ = fs::remove_file(&socket);
            let _ = fs::remove_file(&pidfile);
        };
        // The one remaining path-based `chmod`, and it stays one deliberately:
        // `fchmod` on a bound `AF_UNIX` listener does not reach the filesystem
        // entry the socket is bound at (the fd refers to the socket object, not
        // that inode), so there is no fd form of this call to prefer. It is not
        // the directory step's hazard: what makes THAT one an arbitrary-chmod
        // primitive is an attacker redirecting the name at a file of ours, and
        // the directory rule above has already established that only root or we
        // can create or replace an entry in this directory — the same property
        // the `bind` one line up depends on. The window is also
        // narrowing-only: the mode goes from the umask's to `0600`.
        if let Err(error) = fs::set_permissions(&socket, fs::Permissions::from_mode(0o600)) {
            cleanup();
            return Err(self.socket_error(
                "could not restrict the control socket to 0600",
                &socket,
                error,
            ));
        }
        if let Err(error) = write_pidfile(&lock) {
            cleanup();
            return Err(self.socket_error("could not write the daemon pidfile", &pidfile, error));
        }
        Ok((
            listener,
            SocketGuard {
                socket,
                pidfile,
                _lock: lock,
            },
        ))
    }

    /// Give a path-less `std::io` failure the operation it failed, the path it
    /// failed on and the env var that moves the socket, keeping its kind and
    /// its own reason. The mirror of `refuse_dir` for the file-level steps:
    /// a daemon's `main` prints this verbatim, so a bare
    /// `File name too long (os error 63)` from `bind` — or
    /// `Permission denied (os error 13)` from the pidfile open — would name
    /// neither the rung of the ladder that produced the path nor a way out.
    #[cfg(unix)]
    fn socket_error(&self, what: &str, path: &Path, error: io::Error) -> io::Error {
        io::Error::new(
            error.kind(),
            format!(
                "{what} {}: {error} (set {} to a path under a directory only you can write to)",
                path.display(),
                self.env
            ),
        )
    }

    /// A pre-existing pidfile or socket must be OURS: in a shared directory
    /// another user could have planted one under the name we are about to use.
    ///
    /// Both refusals go through [`Self::socket_error`] like every other
    /// file-level step of [`Self::acquire_socket`], so each carries the
    /// operation, the path and the env var that moves the socket — the
    /// ownership refusal is a REFUSAL to use somebody else's file, and "move
    /// the socket somewhere only you can write" is the way out of it, so the
    /// error that says so is the one worth printing.
    #[cfg(unix)]
    fn require_ours(&self, path: &Path, what: &str) -> io::Result<()> {
        match fs::symlink_metadata(path) {
            Ok(meta) if meta.uid() != euid() => Err(self.socket_error(
                &format!("refusing the pre-existing {what}"),
                path,
                io::Error::new(
                    io::ErrorKind::PermissionDenied,
                    format!(
                        "it is owned by uid {}, not this user (uid {})",
                        meta.uid(),
                        euid()
                    ),
                ),
            )),
            Ok(_) => Ok(()),
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
            Err(error) => {
                Err(self.socket_error(&format!("could not inspect the {what}"), path, error))
            }
        }
    }
}

/// The pidfile that pairs with `socket` (`<name>.sock` → `<name>.pid` beside it).
pub fn pidfile_for(socket: &Path) -> PathBuf {
    socket.with_extension("pid")
}

/// Read a pid from `pidfile`, or `None` if it is missing or unparseable.
pub fn read_pidfile(pidfile: &Path) -> Option<u32> {
    std::fs::read_to_string(pidfile)
        .ok()?
        .trim()
        .parse::<u32>()
        .ok()
}

#[cfg(unix)]
fn euid() -> u32 {
    // SAFETY: geteuid has no preconditions and cannot fail.
    unsafe { libc::geteuid() }
}

#[cfg(not(unix))]
fn euid() -> u32 {
    0
}

/// Removes the socket and pidfile on drop and OWNS the single-daemon flock for
/// the daemon's lifetime. The files are unlinked WHILE the lock is still held
/// (`_lock` drops last), so a guard only ever removes its own socket, never a
/// live replacement's.
#[cfg(unix)]
#[derive(Debug)]
pub struct SocketGuard {
    socket: PathBuf,
    pidfile: PathBuf,
    _lock: File,
}

#[cfg(unix)]
impl SocketGuard {
    /// The bound socket path (for logging).
    pub fn socket_path(&self) -> &Path {
        &self.socket
    }
}

#[cfg(unix)]
impl Drop for SocketGuard {
    fn drop(&mut self) {
        for path in [&self.socket, &self.pidfile] {
            if let Err(error) = fs::remove_file(path) {
                if error.kind() != io::ErrorKind::NotFound {
                    // `warn!`: a `debug!` here is compiled out of every daemon
                    // that links this crate (`release_max_level_info`).
                    tracing::warn!(path = %path.display(), error = %error, "socket cleanup failed on shutdown");
                }
            }
        }
        // `_lock` drops here, after the unlinks, releasing the flock.
    }
}

/// Refuse a socket directory, naming it, the reason, and — like
/// [`DaemonSocket::socket_error`], its mirror for the file-level steps — the
/// exact env var that moves the socket. The remedy names only directories the
/// rule can actually vouch for: one nobody else can write into, or a
/// ROOT-owned, WORLD-WRITABLE sticky directory such as `/tmp`. Both halves of
/// that qualifier decide an arm, so neither may be dropped: a root-owned
/// sticky `1770` group share is refused (arm 1 requires world-writable), and
/// so is another user's `1777` share (the sticky bit does not stop that
/// directory's own owner unlinking our socket). A remedy that said "a sticky
/// share" would recommend two directories this function itself refuses.
#[cfg(unix)]
fn refuse_dir(parent: &Path, env: &'static str, why: String) -> io::Error {
    io::Error::new(
        io::ErrorKind::PermissionDenied,
        format!(
            "refusing to bind a socket under {}: {why} (fix the directory, or set {env} to a path under a directory only you can write to, or under a root-owned, world-writable sticky directory like /tmp)",
            parent.display()
        ),
    )
}

/// What to do with an existing socket directory, decided from its metadata
/// alone (pure, so every arm is pinned by hand-written cases). The rule these
/// arms implement is stated once, in this crate's module docs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DirVerdict {
    /// Bind under it as is: the module docs' arm 1, or arm 2 with nothing to
    /// strip.
    Accept,
    /// Ours, and its group or others could write into it: strip those bits
    /// (arm 2).
    Tighten { from: u32, to: u32 },
    /// Nothing we can make safe: arm 4. The string is the REASON, and it names
    /// the residual rather than a category — an operator looking at a `1777`
    /// directory is told which single user the sticky bit does not bind, not
    /// that their directory "is not a shared one". (Arm 3, ours but
    /// unstrippable, is a [`Self::Tighten`] whose chmod failed;
    /// `prepare_socket_dir` refuses it there.)
    Refuse(String),
}

/// The directory rule (see the module docs). `mode` is the full `st_mode`
/// permission word (sticky bit included), `owner` the directory's uid, `me`
/// this process's effective uid.
pub fn socket_dir_verdict(mode: u32, owner: u32, me: u32) -> DirVerdict {
    let sticky = mode & 0o1000 != 0;
    let world_writable = mode & 0o002 != 0;
    let others_may_write = mode & 0o022;
    // Arm 1. A sticky, world-writable directory is accepted BEFORE ownership
    // is consulted — but only when its owner is root or us. The sticky bit
    // gives ALMOST the property this check exists to establish: it stops
    // every OTHER user unlinking what we create. It does not bind the
    // DIRECTORY's own owner, who may unlink anything under it (Linux
    // `check_sticky` returns 0 when `dir->i_uid == fsuid`; Darwin/BSD
    // `vnode_authorize` is the same), so the arm is scoped to the two owners
    // we already trust: root (omnipotent anyway — `/tmp` IS this directory)
    // and ourselves. Deciding on ownership first would instead make a daemon
    // that runs as root — or `cargo test` in a root container — "tighten" a
    // shared `/tmp` from 1777 to 1755 for every other user on the machine.
    if sticky && world_writable && (owner == me || owner == 0) {
        return DirVerdict::Accept;
    }
    // Arm 2: ours — accept it as is, or make it safe rather than complain
    // about it.
    if owner == me {
        if others_may_write == 0 {
            return DirVerdict::Accept;
        }
        // Ours, and not a trusted sticky share: strip the write bits others
        // could use, keep everything else (sticky, setgid, the owner's own
        // bits).
        return DirVerdict::Tighten {
            from: mode & 0o7777,
            to: (mode & 0o7777) & !0o022,
        };
    }
    // Arm 4. A foreign sticky share gets its OWN reason: saying it "is not a
    // sticky shared directory" would be plainly false to the operator looking
    // at a 1777 directory, and the residual — its owner, not the world — is
    // the whole reason we refuse.
    if sticky && world_writable {
        return DirVerdict::Refuse(format!(
            "it is a sticky, world-writable directory owned by uid {owner} — not this user \
             (uid {me}) and not root: the sticky bit stops OTHER users unlinking our socket, \
             but not that directory's own owner, who could unlink it and bind an impostor at \
             the same path"
        ));
    }
    DirVerdict::Refuse(format!(
        "it is owned by uid {owner}, not this user (uid {me}), and is not a sticky, \
         world-writable directory owned by root or by you (like /tmp)"
    ))
}

/// Open the socket directory for judging AND for changing: `O_RDONLY |
/// O_NOFOLLOW | O_DIRECTORY`, so the descriptor is the directory at that exact
/// name or the open fails — never a symlink's target, never a file. Everything
/// the rule then does (`fstat` for the verdict, `fchmod` for the tightening)
/// rides this one fd, so the inode judged is the inode changed; see the module
/// docs for why a second, path-based `chmod` would be an arbitrary-chmod
/// primitive rather than a tidier spelling.
///
/// `O_RDONLY` on a directory needs its READ bit, which a `stat` of the name did
/// not: a search-only socket directory of the user's (`0300`) used to be
/// accepted and is now refused with "it could not be inspected: Permission
/// denied". That is deliberate rather than overlooked — a directory that
/// cannot be read is one whose contents cannot be vouched for — and it is unreachable from
/// the default ladder, every rung of which is `0700`, `07x5` or `/tmp`'s
/// `1777`; only an explicitly configured `CERULION_*_SOCKET` path can be
/// search-only. `O_PATH` would open it without the read bit but is Linux-only,
/// so it would buy that corner at the price of the guarantee on macOS.
#[cfg(unix)]
fn open_socket_dir(parent: &Path) -> io::Result<File> {
    OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_DIRECTORY)
        .open(parent)
}

/// Why [`open_socket_dir`] failed, in the vocabulary of the directory rule.
///
/// The two errnos that mean "what is at that name is not a directory of ours to
/// use" are platform-split, so BOTH are the planted-object refusal: opening a
/// trailing symlink with `O_NOFOLLOW | O_DIRECTORY` is `ENOTDIR` on macOS
/// (measured: the type check fires first, while `O_NOFOLLOW` alone gives
/// `ELOOP`) and `ELOOP` on Linux (`O_NOFOLLOW` is enforced during path
/// resolution, before the `O_DIRECTORY` type check). A plain file is `ENOTDIR`
/// on both. `ENOTDIR` also covers a non-final path component that is a file,
/// which is the same sentence and the same remedy for the operator, so the
/// errno is appended verbatim rather than the arms being split further.
#[cfg(unix)]
fn describe_dir_open_failure(error: &io::Error) -> String {
    match error.raw_os_error() {
        Some(libc::ELOOP) | Some(libc::ENOTDIR) => {
            format!("it is not a directory (a symlink or file is planted there): {error}")
        }
        _ => format!("it could not be inspected: {error}"),
    }
}

/// Create the socket directory if needed (`0700`), then apply
/// [`socket_dir_verdict`] — accept, tighten (logged once), or refuse with the
/// reason. `env` is the caller's socket env var, so every refusal names the
/// way out.
#[cfg(unix)]
fn prepare_socket_dir(parent: &Path, env: &'static str) -> io::Result<()> {
    let dir = match open_socket_dir(parent) {
        Ok(dir) => dir,
        Err(e) if e.kind() == io::ErrorKind::NotFound => {
            DirBuilder::new()
                .recursive(true)
                .mode(0o700)
                .create(parent)
                .map_err(|e| refuse_dir(parent, env, format!("it could not be created: {e}")))?;
            // The directory we just made is opened the same way rather than
            // trusted: `mkdir(2)` is path-based, so between it and here the
            // name could have been swapped for something else, and from this
            // point on it is the DESCRIPTOR, not the name, that is judged and
            // chmod-ed.
            open_socket_dir(parent)
                .map_err(|e| refuse_dir(parent, env, describe_dir_open_failure(&e)))?
        }
        Err(e) => return Err(refuse_dir(parent, env, describe_dir_open_failure(&e))),
    };
    // `fstat` on the open fd — not a second `stat` of the path. No `is_dir()`
    // arm follows it: `O_DIRECTORY` already made "not a directory" an open
    // failure above, so a check here could not fail and would be dead code.
    let meta = dir
        .metadata()
        .map_err(|e| refuse_dir(parent, env, format!("it could not be inspected: {e}")))?;
    match socket_dir_verdict(meta.mode(), meta.uid(), euid()) {
        DirVerdict::Accept => Ok(()),
        DirVerdict::Tighten { from, to } => {
            // `fchmod` through the fd we judged, never `chmod` of the path.
            dir.set_permissions(fs::Permissions::from_mode(to)).map_err(|e| {
                refuse_dir(
                    parent,
                    env,
                    format!(
                        "it is writable by other users (mode {from:o}) and could not be tightened to {to:o}: {e}"
                    ),
                )
            })?;
            tracing::warn!(
                path = %parent.display(),
                from = format_args!("{from:o}"),
                to = format_args!("{to:o}"),
                "tightened the socket directory: other users could write into it and replace the daemon's socket"
            );
            Ok(())
        }
        DirVerdict::Refuse(why) => Err(refuse_dir(parent, env, why)),
    }
}

/// `flock(LOCK_EX | LOCK_NB)` on `f`; [`io::ErrorKind::WouldBlock`] when held
/// by ANY open file description (another process, or another thread's open of
/// the same file), released when `f` closes or the process dies.
#[cfg(unix)]
fn flock_exclusive_nonblocking(f: &File) -> io::Result<()> {
    // SAFETY: `f` is a live, open file owned by the caller for the whole call.
    let rc = unsafe { libc::flock(f.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) };
    if rc != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

/// Write our pid through the locked handle, truncating stale content first.
#[cfg(unix)]
fn write_pidfile(lock: &File) -> io::Result<()> {
    let mut f = lock;
    f.set_len(0)?;
    f.seek(SeekFrom::Start(0))?;
    write!(f, "{}", std::process::id())?;
    f.flush()?;
    Ok(())
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use std::sync::{Mutex, MutexGuard};

    /// Tests that touch the process env serialize here.
    fn env_lock() -> MutexGuard<'static, ()> {
        static LOCK: Mutex<()> = Mutex::new(());
        LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    struct EnvGuard(&'static str, Option<String>);
    impl EnvGuard {
        fn set(key: &'static str, val: Option<&str>) -> Self {
            let prev = std::env::var(key).ok();
            match val {
                Some(v) => std::env::set_var(key, v),
                None => std::env::remove_var(key),
            }
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

    fn tempdir_unique(tag: &str) -> PathBuf {
        use std::sync::atomic::{AtomicU64, Ordering};
        static N: AtomicU64 = AtomicU64::new(0);
        let n = N.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("cer_hyg_{tag}_{}_{n}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        dir
    }

    /// The level of a captured `tracing-test` line, read as a whole whitespace
    /// TOKEN out of the line header (`<timestamp>  WARN  <span>: <target>: …`).
    /// A bare `contains(" WARN ")` would also match an uppercase field VALUE,
    /// and every captured line carries the test function's own name.
    fn line_level(line: &str) -> Option<&str> {
        line.split_whitespace()
            .take(2)
            .find(|t| matches!(*t, "TRACE" | "DEBUG" | "INFO" | "WARN" | "ERROR"))
    }

    /// The captured lines emitted at `level` that contain every one of
    /// `needles`. The level is matched as a TOKEN, so demoting a `warn!` to
    /// `debug!` — which `release_max_level_info` deletes from every shipped
    /// daemon — cannot satisfy a `WARN` predicate.
    fn lines_at<'a>(lines: &[&'a str], level: &str, needles: &[&str]) -> Vec<&'a str> {
        lines
            .iter()
            .copied()
            .filter(|line| {
                line_level(line) == Some(level) && needles.iter().all(|n| line.contains(n))
            })
            .collect()
    }

    /// `Ok` iff EXACTLY `want` captured lines are at `level` and carry every
    /// needle — a count, not a presence check, so a doubled log fails too.
    fn expect_lines(
        lines: &[&str],
        level: &str,
        needles: &[&str],
        want: usize,
    ) -> Result<(), String> {
        let hits = lines_at(lines, level, needles);
        if hits.len() == want {
            return Ok(());
        }
        Err(format!(
            "expected exactly {want} {level} line(s) carrying {needles:?}, got {}: {lines:?}",
            hits.len()
        ))
    }

    /// A `0500` directory refuses an unlink for its non-root owner, and a
    /// `0000` one refuses a stat of anything under it — both are no-ops for
    /// root, so the tests that need them cannot run there.
    ///
    /// A skip is a PASS, and libtest DISCARDS a passing test's stderr, so the
    /// line below is invisible without `--nocapture`: on an interactive desk
    /// that is an acceptable nudge, but on a lane whose whole job is to fail a
    /// PR it is a hollow green. So when `CI` is set this PANICS instead,
    /// naming the test and the reason. (No root CI lane exists today — GitHub
    /// runners are the non-root `runner` user — which is exactly why a future
    /// one must fail loudly rather than inherit a silent skip.)
    fn skip_as_root(test: &str) -> bool {
        if euid() != 0 {
            return false;
        }
        assert!(
            std::env::var_os("CI").is_none(),
            "{test} cannot run as root (root ignores the directory permissions this pin \
             needs), and CI is set: a root CI lane would report this pin green without \
             running it. Run the cerulion_hygiene tests as a non-root user."
        );
        eprintln!("SKIPPING {test}: running as root, which ignores directory permissions");
        true
    }

    #[test]
    fn the_three_daemons_carry_their_historical_env_vars_and_names() {
        assert_eq!(
            (NETD.name(), NETD.binary(), NETD.socket_env()),
            ("netd", "cerulion-netd", "CERULION_NETD_SOCK")
        );
        assert_eq!(
            (VIZD.name(), VIZD.binary(), VIZD.socket_env()),
            ("vizd", "cerulion-vizd", "CERULION_VIZD_SOCK")
        );
        assert_eq!(
            (WSD.name(), WSD.binary(), WSD.socket_env()),
            ("wsd", "cerulion-wsd", "CERULION_WSD_SOCKET")
        );
    }

    #[test]
    fn the_ladder_is_env_then_xdg_then_home_then_a_per_user_tmp_dir() {
        let _lock = env_lock();
        let _e = EnvGuard::set("CERULION_NETD_SOCK", Some("/x/override.sock"));
        let _x = EnvGuard::set("XDG_RUNTIME_DIR", Some("/run/user/7"));
        let _h = EnvGuard::set("HOME", Some("/home/h"));
        assert_eq!(
            NETD.default_socket_path(),
            PathBuf::from("/x/override.sock")
        );
        let _e = EnvGuard::set("CERULION_NETD_SOCK", Some(""));
        assert_eq!(
            NETD.default_socket_path(),
            PathBuf::from("/run/user/7/cerulion/netd.sock")
        );
        let _x = EnvGuard::set("XDG_RUNTIME_DIR", None);
        assert_eq!(
            VIZD.default_socket_path(),
            PathBuf::from("/home/h/.cerulion/vizd.sock")
        );
        let _h = EnvGuard::set("HOME", None);
        assert_eq!(
            WSD.default_socket_path(),
            PathBuf::from(format!("/tmp/cerulion-{}/wsd.sock", euid())),
            "the last rung is PER USER"
        );
    }

    #[test]
    fn pidfile_pairs_with_socket() {
        assert_eq!(
            pidfile_for(Path::new("/run/cerulion/netd.sock")),
            PathBuf::from("/run/cerulion/netd.pid")
        );
    }

    /// The NEGATIVE twin of the two tighten tests: the ordinary path — a
    /// missing directory created `0700` — must say NOTHING at `warn!`, so the
    /// "exactly one WARN" counts below cannot be met by a line the daemon
    /// emits on every start.
    #[tracing_test::traced_test]
    #[test]
    fn acquire_binds_writes_pidfile_restricts_the_socket_and_cleanup_removes_both() {
        let dir = tempdir_unique("acquire");
        let socket = dir.join("netd.sock");
        let (_listener, guard) = NETD.acquire_socket(socket.clone()).expect("acquire");
        assert!(socket.exists(), "socket bound");
        assert_eq!(fs::metadata(&socket).unwrap().mode() & 0o777, 0o600);
        assert_eq!(
            fs::metadata(&dir).unwrap().mode() & 0o777,
            0o700,
            "a missing dir is created 0700"
        );
        assert_eq!(
            read_pidfile(&pidfile_for(&socket)),
            Some(std::process::id())
        );
        assert_eq!(guard.socket_path(), socket.as_path());
        drop(guard);
        assert!(!socket.exists(), "socket removed on cleanup");
        assert!(!pidfile_for(&socket).exists(), "pidfile removed on cleanup");
        logs_assert(|lines| expect_lines(lines, "WARN", &[], 0));
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_second_acquire_while_the_lock_is_held_is_refused_naming_pid_and_env() {
        let dir = tempdir_unique("live_refuse");
        let socket = dir.join("vizd.sock");
        let (_listener, _guard) = VIZD.acquire_socket(socket.clone()).expect("first");
        let error = VIZD
            .acquire_socket(socket.clone())
            .expect_err("second must refuse");
        assert_eq!(error.kind(), io::ErrorKind::AddrInUse);
        let message = error.to_string();
        assert!(message.contains("cerulion-vizd"), "{message}");
        assert!(
            message.contains(&std::process::id().to_string()),
            "{message}"
        );
        assert!(message.contains("CERULION_VIZD_SOCK"), "{message}");
        assert!(
            socket.exists(),
            "the refusal must not touch the live socket"
        );
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_stale_socket_under_a_free_lock_is_recovered() {
        let dir = tempdir_unique("stale");
        DirBuilder::new()
            .recursive(true)
            .mode(0o700)
            .create(&dir)
            .unwrap();
        let socket = dir.join("wsd.sock");
        fs::write(&socket, b"").unwrap(); // a leftover from a crashed daemon
        let (_listener, guard) = WSD.acquire_socket(socket.clone()).expect("recovered");
        drop(guard);
        assert!(!socket.exists());
        let _ = fs::remove_dir_all(&dir);
    }

    /// Every arm of the directory rule against hand-written cases; no
    /// environment (users, groups, filesystems) is needed to pin the security
    /// arm.
    #[test]
    fn the_directory_verdict_is_pinned_on_every_arm() {
        let (me, other) = (1000, 1001);
        assert_eq!(socket_dir_verdict(0o40700, me, me), DirVerdict::Accept);
        assert_eq!(socket_dir_verdict(0o40755, me, me), DirVerdict::Accept);
        assert_eq!(
            socket_dir_verdict(0o40775, me, me),
            DirVerdict::Tighten {
                from: 0o775,
                to: 0o755
            },
            "group write on a dir of ours is stripped, whatever the group"
        );
        assert_eq!(
            socket_dir_verdict(0o40777, me, me),
            DirVerdict::Tighten {
                from: 0o777,
                to: 0o755
            }
        );
        assert_eq!(
            socket_dir_verdict(0o40770, me, me),
            DirVerdict::Tighten {
                from: 0o770,
                to: 0o750
            }
        );
        assert_eq!(
            socket_dir_verdict(0o41775, me, me),
            DirVerdict::Tighten {
                from: 0o1775,
                to: 0o1755
            },
            "a sticky dir of ours is tightened too, keeping the sticky bit"
        );
        assert_eq!(
            socket_dir_verdict(0o42775, me, me),
            DirVerdict::Tighten {
                from: 0o2775,
                to: 0o2755
            },
            "setgid is preserved"
        );
        assert_eq!(
            socket_dir_verdict(0o41777, 0, me),
            DirVerdict::Accept,
            "/tmp: root-owned, sticky, world-writable"
        );
        assert_eq!(
            socket_dir_verdict(0o41777, 0, 0),
            DirVerdict::Accept,
            "/tmp seen by a ROOT daemon: the sticky-shared arm wins over the ownership arm, \
             or we would chmod the machine's /tmp to 1755 for every other user"
        );
        assert_eq!(
            socket_dir_verdict(0o41777, me, me),
            DirVerdict::Accept,
            "and a sticky world-writable dir of OUR OWN is left alone for the same reason \
             (the sticky+world-writable arm is decided BEFORE ownership; the 0o41775 case \
             above is the discriminator — sticky alone is not a shared /tmp, so ours is \
             still tightened)"
        );
        // THE arm the sticky bit cannot vouch for. The sticky bit binds every
        // OTHER user, but not the DIRECTORY's own owner, who may unlink
        // anything under it — so a foreign, non-root 1777 share leaves its
        // owner able to unlink our socket and bind an impostor, which is
        // verbatim the attack the whole rule exists to stop. The two Accept
        // vectors above are its discriminators: identical mode, trusted owner.
        assert!(
            matches!(
                socket_dir_verdict(0o41777, other, me),
                DirVerdict::Refuse(ref why)
                    if why.contains("uid 1001") && why.contains("own owner")
            ),
            "a sticky world-writable share owned by ANOTHER non-root user must be REFUSED, \
             naming the residual its owner keeps — got {:?}",
            socket_dir_verdict(0o41777, other, me)
        );
        assert!(
            matches!(socket_dir_verdict(0o41777, 1, 0), DirVerdict::Refuse(_)),
            "and a root daemon must refuse another account's 1777 share too: `owner == 0` \
             is a check on the DIRECTORY's owner, not on who we happen to be running as"
        );
        assert!(
            matches!(
                socket_dir_verdict(0o40777, other, me),
                DirVerdict::Refuse(ref why) if why.contains("uid 1001")
            ),
            "world-writable but NOT sticky and not ours: nothing we can fix"
        );
        assert!(matches!(
            socket_dir_verdict(0o40755, other, me),
            DirVerdict::Refuse(_)
        ));
        assert!(
            matches!(
                socket_dir_verdict(0o41755, other, me),
                DirVerdict::Refuse(_)
            ),
            "sticky but not world-writable and not ours: we could not even create the socket"
        );
    }

    /// A refusal's REMEDY is an instruction the operator will follow, so it
    /// may name only directories this same rule accepts. The oracle is
    /// [`socket_dir_verdict`] itself, run on the shape the remedy describes
    /// and on the two near-misses a shorthand would have swept in — not on a
    /// second sentence, which is how a remedy reading "a root-owned sticky
    /// share" survived review while arm 1 required sticky AND world-writable
    /// AND a trusted owner.
    ///
    /// Pure uids (never `euid()`): as root, `0o41770` takes the tighten arm
    /// rather than the refusal, and the point here is the RULE, not who runs
    /// the test.
    #[test]
    fn the_refusal_remedy_recommends_only_directories_the_rule_accepts() {
        let (me, other) = (1000, 1001);
        let remedy = refuse_dir(
            Path::new("/somewhere/cerulion"),
            "CERULION_WSD_SOCKET",
            "why".to_string(),
        )
        .to_string();

        // Each of the three decides arm 1, so the remedy must carry all three;
        // the two refusals below are the directories a shorthand missing one
        // would have sent the operator off to build.
        for qualifier in ["root-owned", "world-writable", "sticky"] {
            assert!(
                remedy.contains(qualifier),
                "the remedy dropped {qualifier:?}, so it now recommends a shape wider than \
                 the arm that accepts it: {remedy}"
            );
        }

        // What it recommends IS accepted...
        assert_eq!(
            socket_dir_verdict(0o41777, 0, me),
            DirVerdict::Accept,
            "the remedy's root-owned, world-writable sticky directory must be the arm-1 \
             accept — otherwise the message sends the operator somewhere we refuse"
        );
        assert_eq!(
            socket_dir_verdict(0o40700, me, me),
            DirVerdict::Accept,
            "and so must its other half, a directory only we can write to"
        );
        // ...while the two shapes a dropped qualifier would have covered are
        // REFUSED by the very function that printed the remedy.
        assert!(
            matches!(socket_dir_verdict(0o41770, 0, me), DirVerdict::Refuse(_)),
            "a root-owned sticky `1770` group share is refused, so a remedy saying only \
             \"a root-owned sticky share\" would recommend it"
        );
        assert!(
            matches!(
                socket_dir_verdict(0o41777, other, me),
                DirVerdict::Refuse(_)
            ),
            "another user's `1777` share is refused, so a remedy saying only \"a sticky, \
             world-writable share\" would recommend it"
        );
    }

    /// The live twin of the tighten arm: a group- and world-writable directory
    /// of ours is bound under AND left at 0755 afterwards, and the one
    /// user-visible signal — EXACTLY ONE `warn!` carrying the before/after
    /// modes — fires. The level is matched as a token and the lines are
    /// COUNTED: a `debug!` does not exist in a shipped daemon
    /// (`release_max_level_info`), and a doubled warn is not the contract
    /// either.
    #[tracing_test::traced_test]
    #[test]
    fn a_writable_dir_of_ours_is_tightened_not_refused() {
        let dir = tempdir_unique("tighten");
        fs::create_dir_all(&dir).unwrap();
        fs::set_permissions(&dir, fs::Permissions::from_mode(0o777)).unwrap();
        let (_listener, guard) = NETD
            .acquire_socket(dir.join("netd.sock"))
            .expect("a dir of ours is made safe, not refused");
        assert_eq!(
            fs::metadata(&dir).unwrap().mode() & 0o777,
            0o755,
            "group and world write bits stripped"
        );
        logs_assert(|lines| {
            expect_lines(
                lines,
                "WARN",
                &["tightened the socket directory", "from=777", "to=755"],
                1,
            )
        });
        drop(guard);
        let _ = fs::remove_dir_all(&dir);
    }

    /// A shared group is exactly the case the tightening exists for: another
    /// member could replace our socket. The directory is re-grouped to one of
    /// this user's supplementary groups when it has one (the arm then runs the
    /// real chgrp); the verdict itself is pinned above regardless. Same
    /// level-token count as the world-writable twin, with this arm's own
    /// modes — the two differ only in the mode pair, so one cannot stand in
    /// for the other.
    #[tracing_test::traced_test]
    #[test]
    fn a_dir_writable_by_a_shared_group_is_tightened() {
        let mut groups = vec![0 as libc::gid_t; 256];
        // SAFETY: getgroups(2) into a correctly sized buffer.
        let n = unsafe { libc::getgroups(groups.len() as libc::c_int, groups.as_mut_ptr()) };
        assert!(n >= 0, "getgroups(2) failed");
        groups.truncate(n as usize);
        // SAFETY: getegid has no preconditions.
        let mine = unsafe { libc::getegid() };
        let dir = tempdir_unique("sharedgroup");
        fs::create_dir_all(&dir).unwrap();
        if let Some(&other) = groups.iter().find(|&&g| g != mine) {
            let c_path = std::ffi::CString::new(dir.to_str().unwrap()).unwrap();
            // SAFETY: chown(2) with uid -1 (unchanged) and one of our own groups.
            let rc = unsafe { libc::chown(c_path.as_ptr(), u32::MAX, other) };
            assert_eq!(
                rc,
                0,
                "chgrp to a supplementary group must succeed: {}",
                io::Error::last_os_error()
            );
        } else {
            eprintln!("no supplementary group for this user — the live arm runs with the primary group only (the verdict is pinned purely above)");
        }
        fs::set_permissions(&dir, fs::Permissions::from_mode(0o770)).unwrap();
        let (_listener, guard) = NETD
            .acquire_socket(dir.join("netd.sock"))
            .expect("tightened, not refused");
        assert_eq!(fs::metadata(&dir).unwrap().mode() & 0o777, 0o750);
        logs_assert(|lines| {
            expect_lines(
                lines,
                "WARN",
                &["tightened the socket directory", "from=770", "to=750"],
                1,
            )
        });
        drop(guard);
        let _ = fs::remove_dir_all(&dir);
    }

    /// This crate's own source, for the structural pin below. `include_str!`
    /// rather than a runtime read: the pin is about what the shipped code says,
    /// so it must be the bytes that were COMPILED, and it must not depend on
    /// the test binary's working directory.
    const LIB_SOURCE: &str = include_str!("lib.rs");

    /// Strip `//` line comments and `/* … */` blocks (Rust's nest, so the
    /// depth is counted). String literals are deliberately NOT modelled: a
    /// forbidden needle sitting inside one would still fail the pin, which is
    /// the safe direction, and the pin only ever runs over function bodies that
    /// hold no such literal.
    fn code_only(src: &str) -> String {
        let mut out = String::with_capacity(src.len());
        let mut chars = src.chars().peekable();
        let mut depth = 0usize;
        while let Some(c) = chars.next() {
            // Checked before the in-comment arm, so block comments nest.
            if c == '/' && chars.peek() == Some(&'*') {
                chars.next();
                depth += 1;
                continue;
            }
            if depth > 0 {
                if c == '*' && chars.peek() == Some(&'/') {
                    chars.next();
                    depth -= 1;
                } else if c == '\n' {
                    // Newlines survive so line structure is unchanged.
                    out.push('\n');
                }
                continue;
            }
            if c == '/' && chars.peek() == Some(&'/') {
                for c in chars.by_ref() {
                    if c == '\n' {
                        out.push('\n');
                        break;
                    }
                }
                continue;
            }
            out.push(c);
        }
        out
    }

    /// One top-level function's source: `fn <name>(` at column 0 through the
    /// next line that is exactly `}` at column 0, which is where rustfmt puts
    /// an item's closing brace. Cheaper and far more robust than counting
    /// braces through `format!` strings.
    fn function_source(name: &str) -> &'static str {
        let head = format!("\nfn {name}(");
        let start = LIB_SOURCE
            .find(&head)
            .unwrap_or_else(|| panic!("no top-level `fn {name}(` in lib.rs"))
            + 1;
        let rest = &LIB_SOURCE[start..];
        let end = rest
            .find("\n}\n")
            .unwrap_or_else(|| panic!("`fn {name}` has no column-0 closing brace"))
            + 3;
        &rest[..end]
    }

    /// STRUCTURAL. The socket directory must be judged and changed through ONE
    /// descriptor, never through its PATH twice. The behavioural pins around
    /// this one cannot see the difference: a planted symlink is refused and a
    /// tightened directory really changes mode under the path-based spelling
    /// too. What that spelling loses is the guarantee that the inode read by
    /// the verdict is the inode the `chmod` lands on — `chmod(2)` re-resolves
    /// the name and follows symlinks — and because a tightened mode keeps read
    /// and execute bits, whoever can write the socket directory's PARENT gets
    /// an arbitrary-chmod primitive out of winning that race. Only the source
    /// says which spelling is in use, so only the source can pin it.
    ///
    /// The forbidden needles are `fs::`-qualified on purpose: `fs::metadata`
    /// and `fs::set_permissions` are the PATH forms, `dir.metadata()` and
    /// `dir.set_permissions()` the fd forms, and the crate keeps one legitimate
    /// path-based `fs::set_permissions` elsewhere (the socket's `0600`, which
    /// has no fd form — see its comment), so the pin is scoped to this one
    /// function rather than the file.
    #[test]
    fn the_socket_directory_is_judged_and_chmodded_through_one_fd_never_the_path() {
        let readable = code_only(function_source("prepare_socket_dir"));
        // Matched with ALL whitespace removed, because rustfmt breaks method
        // chains across lines (`let meta = dir\n    .metadata()`) and a needle
        // written as one token would otherwise silently miss. For the absence
        // pins that is also the stronger direction: a forbidden call cannot
        // dodge them by being wrapped.
        let body: String = readable.split_whitespace().collect();

        for (needle, why) in [
            (
                "fs::set_permissions(",
                "the tightening must be `fchmod` on the descriptor the verdict was read from \
                 (`dir.set_permissions(..)`); a path-based `chmod` re-resolves the name and \
                 follows symlinks, so it can be redirected at any file of ours between the \
                 stat and the chmod",
            ),
            (
                "fs::symlink_metadata(",
                "the verdict must come from `fstat` on the open directory (`dir.metadata()`); \
                 a stat of the PATH leaves the inode judged and the inode changed unpinned",
            ),
            (
                "fs::metadata(",
                "same, and this spelling would additionally FOLLOW a planted symlink before \
                 the verdict ever ran",
            ),
        ] {
            assert!(
                !body.contains(needle),
                "`prepare_socket_dir` names {needle:?}, so it touches the socket directory \
                 by PATH after opening it: {why}.\nbody: {readable}"
            );
        }

        // ANTI-TAUTOLOGY. These run AFTER the absence pins above deliberately:
        // a broken extractor or stripper hands those an empty string and they
        // pass vacuously, so something in this test must require what the
        // function still DOES — but a real regression should report as one, and
        // it reports first if the forbidden-call arms come first. Either way the
        // test fails; the ordering only decides which sentence the author reads.
        for present in [
            "open_socket_dir(parent)",
            "dir.metadata()",
            "dir.set_permissions(",
            "socket_dir_verdict(",
        ] {
            assert!(
                body.contains(present),
                "the extracted body of `prepare_socket_dir` no longer contains {present:?} — \
                 either the function changed shape or the extractor is broken, and a broken \
                 extractor makes the absence assertions above vacuous.\nbody: {readable}"
            );
        }

        // And the one opener both call sites go through carries both flags:
        // without `O_NOFOLLOW` the fd could be a symlink's target, and without
        // `O_DIRECTORY` it could be a file (which is also the arm that replaced
        // the old `is_dir()` check).
        let opener = code_only(function_source("open_socket_dir"));
        for flag in ["O_NOFOLLOW", "O_DIRECTORY"] {
            assert!(
                opener.contains(flag),
                "`open_socket_dir` must open with {flag}: {opener}"
            );
        }
    }

    /// `code_only`'s own oracle — the stripper is what makes the pin above
    /// mean anything, so a comment mentioning a forbidden call must vanish and
    /// real code must not.
    #[test]
    fn code_only_strips_both_comment_syntaxes_and_leaves_code_alone() {
        let src = "let a = 1; // fs::set_permissions(p)\n/* fs::metadata(p) /* nested */ */let b = 2;\nlet c = 3;\n";
        let out = code_only(src);
        assert!(!out.contains("fs::set_permissions("), "{out:?}");
        assert!(!out.contains("fs::metadata("), "{out:?}");
        assert!(out.contains("let a = 1;"), "{out:?}");
        assert!(out.contains("let b = 2;"), "{out:?}");
        assert!(out.contains("let c = 3;"), "{out:?}");
    }

    #[test]
    fn a_symlink_planted_at_the_socket_dir_is_refused() {
        let base = tempdir_unique("planted");
        fs::create_dir_all(&base).unwrap();
        let target = base.join("elsewhere");
        fs::create_dir_all(&target).unwrap();
        let link = base.join("cerulion");
        std::os::unix::fs::symlink(&target, &link).unwrap();
        let error = WSD
            .acquire_socket(link.join("wsd.sock"))
            .expect_err("must refuse");
        assert!(error.to_string().contains("not a directory"), "{error}");
        let _ = fs::remove_dir_all(&base);
    }

    /// `std::env::temp_dir()` is either OURS (macOS: a per-user `0700` dir —
    /// arm 2, with nothing to strip) or the ROOT-owned, world-writable sticky
    /// `/tmp` (Linux — arm 1); both must be accepted, which is what every
    /// `--socket /tmp/<name>.sock` user and the daemons' own tests rely on.
    /// The owner is named because it is now load-bearing: a `1777` directory
    /// owned by another non-root user takes arm 4 and is refused.
    #[test]
    fn the_temp_dir_is_an_accepted_socket_parent() {
        let socket = std::env::temp_dir().join(format!("cer_hyg_tmp_{}.sock", std::process::id()));
        let (_listener, guard) = NETD
            .acquire_socket(socket.clone())
            .expect("temp dir accepted");
        drop(guard);
        assert!(!socket.exists());
    }

    /// SCOPE: the ACCEPTING side of the ownership check. Its refusing arm is
    /// pinned by `a_foreign_owned_pidfile_is_refused_naming_the_owner_and_the_env_var`
    /// below — the check only STATS the path, so any root-owned path reaches
    /// it; what a normal user cannot make is a foreign-owned file INSIDE this
    /// test's own directory, which is why that pin uses `/` instead.
    #[test]
    fn a_pidfile_or_socket_of_ours_or_absent_passes_the_ownership_check() {
        let dir = tempdir_unique("ours");
        fs::create_dir_all(&dir).unwrap();
        let pidfile = dir.join("wsd.pid");
        assert!(
            WSD.require_ours(&pidfile, "pidfile").is_ok(),
            "absent is fine"
        );
        fs::write(&pidfile, b"1\n").unwrap();
        assert!(
            WSD.require_ours(&pidfile, "pidfile").is_ok(),
            "ours is fine"
        );
        let _ = fs::remove_dir_all(&dir);
    }

    /// The security arm of `require_ours`: a pre-existing pidfile (or socket)
    /// owned by someone else is refused, and the refusal names the owner, the
    /// path and the env var that moves the socket. `/` is owned by root on
    /// every runner and the check only stats its argument, so the arm is
    /// reachable without creating a foreign-owned file — as root the check
    /// passes (root owns `/`), hence the skip.
    #[test]
    fn a_foreign_owned_pidfile_is_refused_naming_the_owner_and_the_env_var() {
        if skip_as_root("a_foreign_owned_pidfile_is_refused_naming_the_owner_and_the_env_var") {
            return;
        }
        let root = Path::new("/");
        let owner = fs::symlink_metadata(root).unwrap().uid();
        assert_ne!(owner, euid(), "the premise: `/` is not ours");
        let error = WSD
            .require_ours(root, "pidfile")
            .expect_err("a pidfile owned by someone else must be refused");
        let message = error.to_string();
        assert_eq!(
            error.kind(),
            io::ErrorKind::PermissionDenied,
            "a planted file is a permission problem, not an I/O one: {message}"
        );
        for needle in [
            "refusing the pre-existing pidfile",
            &format!("owned by uid {owner}, not this user (uid {})", euid()),
            "CERULION_WSD_SOCKET",
        ] {
            assert!(
                message.contains(needle),
                "a planted-file refusal must name the way out — missing {needle:?} in: {message}"
            );
        }
        assert!(
            message.contains(" / ") || message.contains(" /:"),
            "the refusal must name the path it refused: {message}"
        );
    }

    /// `acquire_socket`'s doc promises that EVERY failure after the directory
    /// step names the operation, the path and the env var. The ownership
    /// checks run there and used to report a bare `std` message with no env
    /// var, so the promise was false for them; this drives the could-not-inspect
    /// arm through the whole production entry point.
    ///
    /// A `0000` directory of ours is ACCEPTED by the directory rule (arm 2
    /// with nothing to strip — no group or other write bit), so the run gets
    /// past `prepare_socket_dir` and dies on the pidfile stat instead, which
    /// is the arm under test. Root ignores the mode, so it cannot run there.
    #[test]
    fn an_uninspectable_pidfile_is_refused_naming_the_operation_the_path_and_the_env_var() {
        if skip_as_root(
            "an_uninspectable_pidfile_is_refused_naming_the_operation_the_path_and_the_env_var",
        ) {
            return;
        }
        let dir = tempdir_unique("noinspect");
        fs::create_dir_all(&dir).unwrap();
        let socket = dir.join("wsd.sock");
        let pidfile = pidfile_for(&socket);
        let pidfile_s = pidfile.display().to_string();
        // `0400`: READABLE, so the directory step opens it, `fstat`s it and
        // accepts it (ours, nothing to strip) — but with no SEARCH bit the
        // stat of `wsd.pid` INSIDE it fails, which is the one arm this test is
        // about. (It used to be `0000`, which since the directory step began
        // opening the directory rather than stat-ing its name also fails the
        // step BEFORE the pidfile is ever reached — so `0000` conflated two
        // refusals and could have passed while this one rotted.)
        fs::set_permissions(&dir, fs::Permissions::from_mode(0o400)).unwrap();
        let error = WSD
            .acquire_socket(socket)
            .expect_err("an uninspectable pidfile must be refused");
        fs::set_permissions(&dir, fs::Permissions::from_mode(0o700)).unwrap();
        let message = error.to_string();
        assert_eq!(
            error.kind(),
            io::ErrorKind::PermissionDenied,
            "the underlying kind is kept: {message}"
        );
        assert!(
            message.contains("could not inspect the pidfile"),
            "it must name the operation that failed: {message}"
        );
        assert!(
            message.contains(&pidfile_s),
            "it must name the pidfile, not just the socket: {message}"
        );
        assert!(
            message.contains("CERULION_WSD_SOCKET"),
            "and the way out, by NAME — a daemon's `main` prints this verbatim, and \
             \"Permission denied (os error 13)\" alone names neither the file nor a \
             remedy: {message}"
        );
        let _ = fs::remove_dir_all(&dir);
    }

    /// A path too long for `sun_path` (104 bytes on macOS, 108 on Linux) is
    /// the class every bare `?` in `acquire_socket` used to report as a naked
    /// `std` error: a daemon's `main` prints it verbatim, so the operator saw
    /// `File name too long (os error 63)` with no path, no rung and no way
    /// out. The oracle is std's OWN error (produced independently, right
    /// here) PLUS the two things std cannot know: the path and the env var.
    #[test]
    fn a_bind_failure_names_the_operation_the_path_and_the_env_var() {
        let dir = tempdir_unique("longsock");
        fs::create_dir_all(&dir).unwrap();
        // A file NAME may be 255 bytes; it is the whole PATH that `sun_path`
        // bounds, so this is unbindable on both platforms while every earlier
        // step in `acquire_socket` (stat, pidfile open, flock) still succeeds.
        let socket = dir.join(format!("{}.sock", "x".repeat(160)));
        let direct = UnixListener::bind(&socket).expect_err("a path this long cannot be bound");
        let error = WSD
            .acquire_socket(socket.clone())
            .expect_err("acquire must fail on it");
        let message = error.to_string();
        assert_eq!(
            error.kind(),
            direct.kind(),
            "the wrapper keeps std's kind: {message}"
        );
        assert!(
            message.contains(&direct.to_string()),
            "and std's own reason: {message}"
        );
        assert!(
            message.contains("could not bind the control socket"),
            "the operation is named: {message}"
        );
        assert!(
            message.contains(&socket.display().to_string()),
            "the path is named: {message}"
        );
        assert!(
            message.contains("CERULION_WSD_SOCKET"),
            "and the way out: {message}"
        );
        let _ = fs::remove_dir_all(&dir);
    }

    /// The sibling arm of the bind wrapper, and the one the finding measured:
    /// a read-only (`0500`) directory of ours is ACCEPTED by the directory
    /// rule — nobody else can write into it — so the failure lands on the
    /// pidfile open, which `std` reports as a bare
    /// `Permission denied (os error 13)` naming neither file nor remedy.
    #[test]
    fn a_pidfile_open_failure_names_the_operation_the_path_and_the_env_var() {
        if skip_as_root("a_pidfile_open_failure_names_the_operation_the_path_and_the_env_var") {
            return;
        }
        let dir = tempdir_unique("ro_pidfile");
        fs::create_dir_all(&dir).unwrap();
        fs::set_permissions(&dir, fs::Permissions::from_mode(0o500)).unwrap();
        let socket = dir.join("netd.sock");
        let error = NETD
            .acquire_socket(socket.clone())
            .expect_err("the pidfile cannot be created in a read-only directory");
        fs::set_permissions(&dir, fs::Permissions::from_mode(0o700)).unwrap();
        let message = error.to_string();
        assert_eq!(
            error.kind(),
            io::ErrorKind::PermissionDenied,
            "the wrapper keeps std's kind: {message}"
        );
        assert!(
            message.contains("could not open the daemon pidfile"),
            "the operation is named: {message}"
        );
        assert!(
            message.contains(&pidfile_for(&socket).display().to_string()),
            "the PIDFILE is named, not the socket: {message}"
        );
        assert!(
            message.contains("CERULION_NETD_SOCK"),
            "and the way out: {message}"
        );
        let _ = fs::remove_dir_all(&dir);
    }

    /// The `SocketGuard::drop` warn: a shutdown that cannot unlink says so,
    /// at `warn!` — a `debug!` is compiled out of every daemon that links this
    /// crate. Both files fail, so the count is TWO, one naming each: a guard
    /// that gave up after the first would leave the pidfile silently behind.
    #[tracing_test::traced_test]
    #[test]
    fn a_cleanup_that_cannot_unlink_warns_once_per_file_it_leaves_behind() {
        if skip_as_root("a_cleanup_that_cannot_unlink_warns_once_per_file_it_leaves_behind") {
            return;
        }
        let dir = tempdir_unique("cleanup_warn");
        let socket = dir.join("netd.sock");
        let (_listener, guard) = NETD.acquire_socket(socket.clone()).expect("acquire");
        let pidfile = pidfile_for(&socket);
        // Unlinking needs WRITE on the directory, so `r-x` refuses both
        // removals with EACCES for their non-root owner.
        fs::set_permissions(&dir, fs::Permissions::from_mode(0o500)).unwrap();
        drop(guard);
        fs::set_permissions(&dir, fs::Permissions::from_mode(0o700)).unwrap();
        // The premise, asserted before the counts: the unlinks really did fail.
        // Without it a filesystem that allowed them would make the arm
        // unreachable and every count below vacuous.
        assert!(socket.exists(), "premise: the socket survived the cleanup");
        assert!(
            pidfile.exists(),
            "premise: the pidfile survived the cleanup"
        );
        let socket_s = socket.display().to_string();
        let pidfile_s = pidfile.display().to_string();
        logs_assert(|lines| {
            let msg = "socket cleanup failed on shutdown";
            expect_lines(lines, "WARN", &[msg], 2)?;
            expect_lines(lines, "WARN", &[msg, &socket_s], 1)?;
            expect_lines(lines, "WARN", &[msg, &pidfile_s], 1)
        });
        let _ = fs::remove_dir_all(&dir);
    }

    /// The two `prepare_socket_dir` failure arms that are NOT a verdict: a
    /// directory that cannot be inspected, and one that cannot be created.
    /// Both must name the directory they failed on — the refusal is printed
    /// verbatim by the daemon's `main`, and "Permission denied" alone does not
    /// say which of `a`, `a/b` or the socket the operator must fix.
    #[test]
    fn a_socket_dir_that_cannot_be_inspected_or_created_is_refused_naming_it() {
        if skip_as_root("a_socket_dir_that_cannot_be_inspected_or_created_is_refused_naming_it") {
            return;
        }
        let base = tempdir_unique("unreadable");
        let outer = base.join("a");
        fs::create_dir_all(&outer).unwrap();
        let inner = outer.join("b");
        let inner_s = inner.display().to_string();

        // `0000`: no search permission on `a`, so the STAT of `a/b` fails.
        fs::set_permissions(&outer, fs::Permissions::from_mode(0o000)).unwrap();
        let error = WSD
            .acquire_socket(inner.join("wsd.sock"))
            .expect_err("an uninspectable directory must be refused");
        fs::set_permissions(&outer, fs::Permissions::from_mode(0o700)).unwrap();
        let message = error.to_string();
        assert_eq!(error.kind(), io::ErrorKind::PermissionDenied, "{message}");
        assert!(message.contains("could not be inspected"), "{message}");
        assert!(message.contains(&inner_s), "it must name a/b: {message}");
        assert!(
            message.contains("CERULION_WSD_SOCKET"),
            "and the way out, by NAME — the security arm must not be the one refusal in \
             the crate that says only \"the daemon's socket env var\": {message}"
        );

        // `0500`: `a/b` stats as absent (search IS allowed), so the CREATE
        // fails instead — a different arm, one line apart, same requirement.
        fs::set_permissions(&outer, fs::Permissions::from_mode(0o500)).unwrap();
        let error = WSD
            .acquire_socket(inner.join("wsd.sock"))
            .expect_err("an uncreatable directory must be refused");
        fs::set_permissions(&outer, fs::Permissions::from_mode(0o700)).unwrap();
        let message = error.to_string();
        assert_eq!(error.kind(), io::ErrorKind::PermissionDenied, "{message}");
        assert!(message.contains("could not be created"), "{message}");
        assert!(message.contains(&inner_s), "it must name a/b: {message}");
        assert!(
            message.contains("CERULION_WSD_SOCKET"),
            "and the way out, by NAME: {message}"
        );

        let _ = fs::remove_dir_all(&base);
    }
}
