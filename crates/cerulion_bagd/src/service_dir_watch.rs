// SPDX-License-Identifier: AGPL-3.0-only
//! Wake when the iceoryx2 SERVICE DIRECTORY changes.
//!
//! # Why the filesystem is the event source
//!
//! The recorder has to learn that a new producer exists. iceoryx2 0.9.1 exposes
//! no discovery EVENT: the only way to ask "what is live?" is
//! `Service::list`, a directory walk whose cost scales with the machine's
//! service count (MEASURED at 117.5 ms with 86 live topics on a robot's
//! compute module). Any
//! design built on asking that question on a cadence is a poll, and a poll is
//! what this module exists to remove.
//!
//! But the question has a substrate. A service's static config is a FILE:
//! `iceoryx2-cal`'s file-backed `StaticStorage` writes it under
//! `global.root_path` + `global.service.directory` (`/tmp/iceoryx2/services` by
//! default), and `Service::list` is a walk of exactly that directory. So a
//! producer registering is a file appearing, and the kernel will tell us when a
//! file appears: `inotify` on Linux, `kqueue`/`EVFILT_VNODE` on macOS.
//!
//! That is the whole idea. The recorder blocks on the kernel. A machine whose
//! topic set never changes costs ZERO directory walks; a machine that opens a
//! route pays one walk, promptly, because something actually happened.
//!
//! # Why this is hand-written rather than a watcher crate
//!
//! The general-purpose watcher crates are recursive, cross-platform, thread-and-
//! channel-owning engines. What is needed here is one flag from one directory,
//! and the whole of it is the fifty lines below per platform. Taking a watcher
//! dependency would add five crates to a recorder that ships on a robot, and one
//! of them carries a license the workspace's `deny.toml` does not allow, for
//! machinery none of which is used. `libc` is already in the tree.
//!
//! # What an event does and does not mean
//!
//! An event means "something in that directory changed", never "a new topic is
//! now listable". Two gaps are real and are handled by the CALLER
//! ([`crate::discovery_scan`]) rather than pretended away here:
//!
//! * A service becomes visible to `Service::list` only once its static-config
//!   file is chmod'd to its final permissions (`list_cfg` filters on exactly
//!   that), which happens shortly AFTER the file is created. On Linux the chmod
//!   is itself an event (`IN_ATTRIB`); on macOS a chmod of a file inside a
//!   directory does not change the directory, so it is not. The caller absorbs
//!   both cases with a coalescing delay and one bounded confirmation walk.
//! * Events are edges, not a queue of facts. A burst of twenty routes may
//!   collapse into one wake, which is correct and desirable: the answer is a
//!   walk either way.
//!
//! # Failure is loud, never silent
//!
//! Every constructor and every wait can fail (an unsupported platform, a
//! directory that is not there, a descriptor limit). None of them returns a
//! plausible-looking "nothing changed": [`WatchWake::Broken`] and
//! [`WatchError`] are distinct from [`WatchWake::Idle`], so the caller can
//! degrade to a timed walk and SAY so. A watcher that silently stopped waking
//! would leave a recorder blind to every producer that appears from then on,
//! which is the defect live discovery exists to prevent.

use std::path::{Path, PathBuf};
use std::time::Duration;

/// Why a watch could not be established, or could not be waited on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WatchError {
    /// This platform has no watch implementation here (neither Linux nor
    /// macOS). Not an error in the operator's world - a statement that the
    /// caller must use its timed fallback.
    Unsupported,
    /// Neither the service directory nor the iceoryx2 root exists, so there is
    /// nothing to watch yet and no parent to watch it appear in.
    NoDirectory {
        /// The directory that was looked for.
        path: String,
    },
    /// A syscall failed. Carries the OS error, verbatim.
    Os {
        /// What was being attempted.
        doing: &'static str,
        /// The OS error.
        error: String,
    },
}

impl std::fmt::Display for WatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Unsupported => write!(
                f,
                "no filesystem-watch implementation for this platform (Linux inotify and macOS \
                 kqueue are the two that exist)"
            ),
            Self::NoDirectory { path } => write!(
                f,
                "neither the iceoryx2 service directory nor its root exists yet ({path})"
            ),
            Self::Os { doing, error } => write!(f, "{doing} failed: {error}"),
        }
    }
}

/// The outcome of one bounded wait.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WatchWake {
    /// The watched directory changed. The caller should enumerate.
    Changed,
    /// The wait expired with nothing seen. Deliberately NOT an error and
    /// deliberately not a reason to enumerate: on a settled machine this is the
    /// answer forever, and walking on it would restore the poll.
    Idle,
    /// The watch itself failed and will not wake again. The caller must degrade
    /// to a timed walk and report the degradation.
    Broken {
        /// Why, for the operator.
        reason: String,
    },
}

/// A watch on the directory `Service::list` walks.
///
/// Holds the kernel watch descriptor for whichever directory is currently
/// watchable: the service directory when it exists, otherwise the iceoryx2 root
/// it will be created in. The second case is real - the recorder's own
/// iceoryx2 node creates the root, but the service directory does not exist
/// until the first service on the machine does, and a recorder can arm before
/// that.
pub struct ServiceDirWatch {
    imp: imp::DirWatch,
    /// The iceoryx2 root, watched while standing in for a service directory
    /// that does not exist yet.
    root: PathBuf,
    /// `root` + the service directory - the directory that must eventually be
    /// watched, and the one `Service::list` walks.
    services: PathBuf,
    /// Whether the current watch is on the ROOT rather than on `services`.
    standing_in: bool,
}

impl ServiceDirWatch {
    /// Watch `root`'s service directory, or `root` itself until that directory
    /// exists.
    pub fn arm(root: &Path, service_dir: &str) -> Result<Self, WatchError> {
        let services = root.join(service_dir);
        let root = root.to_path_buf();
        if services.is_dir() {
            return Ok(Self {
                imp: imp::DirWatch::open(&services)?,
                root,
                services,
                standing_in: false,
            });
        }
        if root.is_dir() {
            return Ok(Self {
                imp: imp::DirWatch::open(&root)?,
                root,
                services,
                standing_in: true,
            });
        }
        Err(WatchError::NoDirectory {
            path: services.display().to_string(),
        })
    }

    /// Block for at most `timeout`, reporting whether the directory changed.
    ///
    /// Also the seam where the watch RE-TARGETS itself. A stand-in watch on the
    /// root is promoted the moment the service directory exists, and a watch
    /// whose service directory has gone away falls back to standing in on the
    /// root - so a directory that is removed and recreated does not leave the
    /// recorder holding a descriptor for something that no longer exists.
    ///
    /// Both re-targets are reported as a change, because the directory coming
    /// into (or out of) existence is exactly when there is something to
    /// enumerate.
    ///
    /// The re-check runs on a CHANGE and while standing in, never on an idle
    /// tick: an idle tick is the answer on a settled machine and must cost
    /// nothing. Residual, stated plainly: on Linux, deleting the WATCHED service
    /// directory invalidates the inotify watch, so no further event arrives and
    /// nothing prompts the re-check. iceoryx2 removes service FILES and never
    /// the directory that holds them, so this is unreachable through the
    /// recorder's own peers; an operator who deletes it by hand gets a recorder
    /// whose discovery is quiet until it is restarted.
    pub fn wait(&mut self, timeout: Duration) -> WatchWake {
        let changed = match self.imp.wait(timeout) {
            Ok(c) => c,
            Err(e) => {
                return WatchWake::Broken {
                    reason: e.to_string(),
                }
            }
        };
        if changed || self.standing_in {
            let services_exist = self.services.is_dir();
            if services_exist == self.standing_in {
                let target = if services_exist {
                    &self.services
                } else {
                    &self.root
                };
                return match imp::DirWatch::open(target) {
                    Ok(w) => {
                        self.imp = w;
                        self.standing_in = !services_exist;
                        WatchWake::Changed
                    }
                    Err(e) => WatchWake::Broken {
                        reason: e.to_string(),
                    },
                };
            }
        }
        if changed {
            WatchWake::Changed
        } else {
            WatchWake::Idle
        }
    }

    /// The directory the kernel watch is currently on - the service directory,
    /// or the root while standing in for it. Reported in the recorder's log so
    /// an operator can see WHAT is being watched, not just that something is.
    pub fn watched_path(&self) -> PathBuf {
        if self.standing_in {
            self.root.clone()
        } else {
            self.services.clone()
        }
    }

    /// Whether the watch is on the real service directory rather than standing
    /// in on its parent.
    pub fn watching_service_dir(&self) -> bool {
        !self.standing_in
    }
}

// ===========================================================================
// Linux: inotify on one directory.
// ===========================================================================
#[cfg(target_os = "linux")]
mod imp {
    use super::WatchError;
    use std::ffi::CString;
    use std::os::unix::ffi::OsStrExt;
    use std::path::Path;
    use std::time::Duration;

    /// The event mask.
    ///
    /// `IN_ATTRIB` is the load-bearing one and is easy to leave out: a service's
    /// static-config file is CREATED with working permissions and only becomes
    /// visible to `Service::list` when it is chmod'd to its final ones, so the
    /// chmod is the edge that matters. `IN_CREATE` / `IN_MOVED_TO` catch the
    /// file appearing, and the two removal events keep a torn-down topic from
    /// leaving the watch believing nothing has changed since.
    const MASK: u32 = libc::IN_CREATE
        | libc::IN_MOVED_TO
        | libc::IN_MOVED_FROM
        | libc::IN_DELETE
        | libc::IN_ATTRIB
        | libc::IN_CLOSE_WRITE;

    pub struct DirWatch {
        fd: libc::c_int,
    }

    impl DirWatch {
        pub fn open(dir: &Path) -> Result<Self, WatchError> {
            let c = CString::new(dir.as_os_str().as_bytes()).map_err(|e| WatchError::Os {
                doing: "encoding the watch path",
                error: e.to_string(),
            })?;
            // SAFETY: a plain syscall with no pointer arguments.
            let fd = unsafe { libc::inotify_init1(libc::IN_NONBLOCK | libc::IN_CLOEXEC) };
            if fd < 0 {
                return Err(WatchError::Os {
                    doing: "inotify_init1",
                    error: std::io::Error::last_os_error().to_string(),
                });
            }
            let this = Self { fd };
            // SAFETY: `fd` is a live inotify descriptor owned by `this`, and `c`
            // is a NUL-terminated path that outlives the call.
            let wd = unsafe { libc::inotify_add_watch(this.fd, c.as_ptr(), MASK) };
            if wd < 0 {
                return Err(WatchError::Os {
                    doing: "inotify_add_watch",
                    error: std::io::Error::last_os_error().to_string(),
                });
            }
            Ok(this)
        }

        pub fn wait(&self, timeout: Duration) -> Result<bool, WatchError> {
            let mut pfd = libc::pollfd {
                fd: self.fd,
                events: libc::POLLIN,
                revents: 0,
            };
            let ms = timeout.as_millis().min(i32::MAX as u128) as libc::c_int;
            // SAFETY: one initialized `pollfd` is passed with a count of 1.
            let rc = unsafe { libc::poll(&mut pfd, 1, ms) };
            if rc < 0 {
                let err = std::io::Error::last_os_error();
                // An interrupted wait is not a broken watch - report it as "no
                // change" so the caller loops back and waits again.
                if err.kind() == std::io::ErrorKind::Interrupted {
                    return Ok(false);
                }
                return Err(WatchError::Os {
                    doing: "poll",
                    error: err.to_string(),
                });
            }
            if rc == 0 {
                return Ok(false);
            }
            // Drain whatever is queued. The CONTENT is deliberately not parsed:
            // the answer to every event is the same walk, and reading names here
            // would only invite deciding from them.
            let mut buf = [0u8; 4096];
            loop {
                // SAFETY: `buf` is a live, correctly sized byte buffer.
                let n = unsafe {
                    libc::read(
                        self.fd,
                        buf.as_mut_ptr() as *mut libc::c_void,
                        buf.len() as libc::size_t,
                    )
                };
                if n <= 0 {
                    break;
                }
            }
            Ok(true)
        }
    }

    impl Drop for DirWatch {
        fn drop(&mut self) {
            // SAFETY: `fd` was opened by this type and is closed exactly once.
            unsafe { libc::close(self.fd) };
        }
    }
}

// ===========================================================================
// macOS: kqueue EVFILT_VNODE on one directory descriptor.
// ===========================================================================
#[cfg(any(target_os = "macos", target_os = "ios"))]
mod imp {
    use super::WatchError;
    use std::ffi::CString;
    use std::os::unix::ffi::OsStrExt;
    use std::path::Path;
    use std::time::Duration;

    /// The vnode events.
    ///
    /// `NOTE_WRITE` is the one that fires when a directory entry is added or
    /// removed. `NOTE_ATTRIB` here is the DIRECTORY's own attributes, not its
    /// children's: kqueue watches a vnode, and a chmod of a file inside the
    /// directory is not a change to the directory. That asymmetry against Linux
    /// is why the caller runs one bounded confirmation walk after an
    /// event-driven one.
    const FFLAGS: u32 = libc::NOTE_WRITE
        | libc::NOTE_EXTEND
        | libc::NOTE_LINK
        | libc::NOTE_DELETE
        | libc::NOTE_RENAME
        | libc::NOTE_ATTRIB;

    pub struct DirWatch {
        kq: libc::c_int,
        dir: libc::c_int,
    }

    impl DirWatch {
        pub fn open(dir: &Path) -> Result<Self, WatchError> {
            let c = CString::new(dir.as_os_str().as_bytes()).map_err(|e| WatchError::Os {
                doing: "encoding the watch path",
                error: e.to_string(),
            })?;
            // `O_EVTONLY` opens a descriptor for event delivery only: it does
            // not count as a reference that would block an unmount.
            // SAFETY: `c` is a NUL-terminated path that outlives the call.
            let dir_fd = unsafe { libc::open(c.as_ptr(), libc::O_EVTONLY | libc::O_CLOEXEC) };
            if dir_fd < 0 {
                return Err(WatchError::Os {
                    doing: "open(O_EVTONLY) on the watch directory",
                    error: std::io::Error::last_os_error().to_string(),
                });
            }
            // SAFETY: a plain syscall with no pointer arguments.
            let kq = unsafe { libc::kqueue() };
            if kq < 0 {
                let err = std::io::Error::last_os_error();
                // SAFETY: `dir_fd` was opened just above and is not yet owned by
                // a `DirWatch`, so this is its only close.
                unsafe { libc::close(dir_fd) };
                return Err(WatchError::Os {
                    doing: "kqueue",
                    error: err.to_string(),
                });
            }
            let this = Self { kq, dir: dir_fd };
            let change = libc::kevent {
                ident: dir_fd as libc::uintptr_t,
                filter: libc::EVFILT_VNODE,
                flags: libc::EV_ADD | libc::EV_ENABLE | libc::EV_CLEAR,
                fflags: FFLAGS,
                data: 0,
                udata: std::ptr::null_mut(),
            };
            // SAFETY: one initialized change is registered and no events are
            // collected (`nevents` is 0, so the out pointer is never written).
            let rc = unsafe {
                libc::kevent(
                    this.kq,
                    &change,
                    1,
                    std::ptr::null_mut(),
                    0,
                    std::ptr::null(),
                )
            };
            if rc < 0 {
                return Err(WatchError::Os {
                    doing: "kevent registration",
                    error: std::io::Error::last_os_error().to_string(),
                });
            }
            Ok(this)
        }

        pub fn wait(&self, timeout: Duration) -> Result<bool, WatchError> {
            let ts = libc::timespec {
                tv_sec: timeout.as_secs().min(i64::MAX as u64) as libc::time_t,
                tv_nsec: timeout.subsec_nanos() as libc::c_long,
            };
            // SAFETY: `kevent` is a plain C struct whose all-zero bit pattern is
            // valid (its one pointer field becomes null, which is what an unused
            // `udata` is).
            let mut out: libc::kevent = unsafe { std::mem::zeroed() };
            // SAFETY: no changes are submitted; one event slot is offered and
            // `out` is a live, correctly typed slot.
            let rc = unsafe {
                libc::kevent(
                    self.kq,
                    std::ptr::null(),
                    0,
                    &mut out,
                    1,
                    &ts as *const libc::timespec,
                )
            };
            if rc < 0 {
                let err = std::io::Error::last_os_error();
                if err.kind() == std::io::ErrorKind::Interrupted {
                    return Ok(false);
                }
                return Err(WatchError::Os {
                    doing: "kevent wait",
                    error: err.to_string(),
                });
            }
            if rc == 0 {
                return Ok(false);
            }
            // `EV_ERROR` arrives as a normal event carrying an errno. Reading it
            // as a change would turn a dead watch into a permanently quiet one.
            if out.flags & libc::EV_ERROR != 0 && out.data != 0 {
                return Err(WatchError::Os {
                    doing: "kevent wait",
                    error: std::io::Error::from_raw_os_error(out.data as i32).to_string(),
                });
            }
            Ok(true)
        }
    }

    impl Drop for DirWatch {
        fn drop(&mut self) {
            // SAFETY: both descriptors were opened by this type and are closed
            // exactly once.
            unsafe {
                libc::close(self.kq);
                libc::close(self.dir);
            }
        }
    }
}

// ===========================================================================
// Everywhere else: there is no watch, and the caller is told so plainly.
// ===========================================================================
#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "ios")))]
mod imp {
    use super::WatchError;
    use std::path::Path;
    use std::time::Duration;

    pub struct DirWatch;

    impl DirWatch {
        pub fn open(_dir: &Path) -> Result<Self, WatchError> {
            Err(WatchError::Unsupported)
        }

        pub fn wait(&self, _timeout: Duration) -> Result<bool, WatchError> {
            Err(WatchError::Unsupported)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A watch on a real directory SEES a file appear, and reports nothing when
    /// nothing happens.
    ///
    /// Both halves in one body deliberately: a watcher stuck on `Changed`
    /// satisfies the first assertion and would turn the caller back into a
    /// walk-every-slice poll, and one stuck on `Idle` satisfies the second while
    /// leaving discovery permanently blind. Neither is a watcher.
    #[test]
    fn a_watch_sees_a_file_appear_and_stays_quiet_when_nothing_does() {
        let root = tempfile::tempdir().expect("tempdir");
        let services = root.path().join("services");
        std::fs::create_dir(&services).expect("create services dir");

        let mut watch = match ServiceDirWatch::arm(root.path(), "services") {
            Ok(w) => w,
            Err(WatchError::Unsupported) => return,
            Err(e) => panic!("arming a watch on a real directory must succeed: {e}"),
        };
        assert!(
            watch.watching_service_dir(),
            "an existing service directory must be watched directly, never stood in for"
        );

        // QUIET: nothing has happened, so the wait must expire.
        assert_eq!(
            watch.wait(Duration::from_millis(120)),
            WatchWake::Idle,
            "a watch on an unchanged directory must report Idle - a Changed here is a poll \
             wearing a watcher's clothes"
        );

        // CHANGED: a file appears.
        std::fs::write(services.join("a.service"), b"x").expect("write");
        assert_eq!(
            watch.wait(Duration::from_secs(5)),
            WatchWake::Changed,
            "a file appearing in the watched directory must wake the watch"
        );

        // And quiet again once the event is consumed - an edge, not a level.
        assert_eq!(
            watch.wait(Duration::from_millis(120)),
            WatchWake::Idle,
            "a consumed event must not re-report forever"
        );
    }

    /// A service directory that does not exist yet is WAITED FOR on its parent,
    /// and the watch promotes itself when it appears.
    ///
    /// This is the recorder arming on a machine where no service exists yet:
    /// iceoryx2 creates the root when the recorder's own node starts, but the
    /// service directory only when the first service does.
    #[test]
    fn a_missing_service_directory_is_watched_for_on_the_root_and_then_promoted() {
        let root = tempfile::tempdir().expect("tempdir");
        let services = root.path().join("services");

        let mut watch = match ServiceDirWatch::arm(root.path(), "services") {
            Ok(w) => w,
            Err(WatchError::Unsupported) => return,
            Err(e) => panic!("a missing service directory must stand in on the root: {e}"),
        };
        assert!(
            !watch.watching_service_dir(),
            "with no service directory the watch must be standing in on the root"
        );
        assert_eq!(
            watch.watched_path(),
            root.path(),
            "the stand-in watch must name the root it is actually watching"
        );

        std::fs::create_dir(&services).expect("create services dir");
        assert_eq!(
            watch.wait(Duration::from_secs(5)),
            WatchWake::Changed,
            "the service directory appearing is the moment there is something to enumerate"
        );
        assert!(
            watch.watching_service_dir(),
            "once the service directory exists the watch must be ON it, not on the root - a \
             stand-in that never promotes misses every later service"
        );
        assert_eq!(watch.watched_path(), services);

        std::fs::write(services.join("b.service"), b"x").expect("write");
        assert_eq!(
            watch.wait(Duration::from_secs(5)),
            WatchWake::Changed,
            "the promoted watch must see files in the directory it was promoted to"
        );
    }

    /// A service directory that goes AWAY drops the watch back onto the root,
    /// so a directory that is removed and recreated does not leave the recorder
    /// holding a descriptor for something that no longer exists.
    ///
    /// The promotion half is pinned above; this is its inverse, and without it
    /// a re-created directory would be watched through a stale descriptor and
    /// every later service would be missed in silence.
    #[test]
    fn a_service_directory_that_disappears_drops_the_watch_back_onto_the_root() {
        let root = tempfile::tempdir().expect("tempdir");
        let services = root.path().join("services");
        std::fs::create_dir(&services).expect("create services dir");

        let mut watch = match ServiceDirWatch::arm(root.path(), "services") {
            Ok(w) => w,
            Err(WatchError::Unsupported) => return,
            Err(e) => panic!("arming on a real directory must succeed: {e}"),
        };
        assert!(watch.watching_service_dir());

        std::fs::remove_dir_all(&services).expect("remove services dir");
        assert_eq!(
            watch.wait(Duration::from_secs(5)),
            WatchWake::Changed,
            "the service directory vanishing is a change worth enumerating"
        );
        assert!(
            !watch.watching_service_dir(),
            "with no service directory the watch must fall back to standing in on the root"
        );

        // And it promotes again when the directory comes back.
        std::fs::create_dir(&services).expect("recreate services dir");
        assert_eq!(watch.wait(Duration::from_secs(5)), WatchWake::Changed);
        assert!(
            watch.watching_service_dir(),
            "a recreated service directory must be watched directly again"
        );
        std::fs::write(services.join("c.service"), b"x").expect("write");
        assert_eq!(
            watch.wait(Duration::from_secs(5)),
            WatchWake::Changed,
            "the re-promoted watch must see files in the recreated directory"
        );
    }

    /// A root that does not exist at all is refused LOUDLY, not accepted as a
    /// watch that can never fire.
    #[test]
    fn a_root_that_does_not_exist_is_refused_rather_than_silently_inert() {
        let root = tempfile::tempdir().expect("tempdir");
        let missing = root.path().join("no-such-root");
        match ServiceDirWatch::arm(&missing, "services") {
            Err(WatchError::NoDirectory { path }) => {
                assert!(
                    path.contains("services"),
                    "the refusal must name what it looked for; got {path}"
                );
            }
            Err(WatchError::Unsupported) => {}
            other => panic!("a nonexistent root must be refused, got {other:?}"),
        }
    }
}
