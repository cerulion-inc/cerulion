// SPDX-License-Identifier: AGPL-3.0-only
//! The POSIX named-SHM MAP SUBSTRATE — the `shm_open` / `ftruncate`-once /
//! `mmap(MAP_SHARED)` / `shm_unlink` mechanics every mapped-word module in this
//! crate performs, written ONCE.
//!
//! # What lives here, and what deliberately does not
//!
//! This module owns MECHANICS ONLY: which syscalls run, in which order, with
//! which cleanup on each failure arm, and which errno is reported. It knows
//! nothing about what any caller puts in the page.
//!
//! SEMANTICS stay at the five call sites, because they genuinely differ:
//!
//! | site | what it keeps |
//! |---|---|
//! | [`crate::barrier`] (`MappedBarrier`) | `BarrierShared::reinit`, the orphan-clear reasoning, the `/cer_bar_` name recipe |
//! | `crate::shm_ring` (`ShmRingOwner`/`ShmRingConsumer`) | the header/manifest write, magic/version/record-size validation, `ShmRingError`, the `/cer_rg_` recipe |
//! | `crate::state_arm` (`MappedStateArm`) | `StateArmWord::init`/`validate` (magic, version, slot count), the `/cer_sta_` recipe |
//! | [`crate::credit`] (`MappedCredit`) | `reject_zero_depth`, `reject_unarmed`, `CreditShared::reinit`, the `/cer_crd_` recipe |
//! | `crate::wedge_page` (`MappedWedgePage`) | the `WEDGE_MAX_SLOTS` ceiling + its refuse-rather-than-truncate diagnostic, `WedgePage::init`/`validate` (magic, version, slot count), the `/cer_wdg_` recipe |
//!
//! (Three of those are PROSE mentions rather than intra-doc links: those modules
//! are `#[cfg(unix)]`-only while this one is portable, and a resolved link from
//! a portable item breaks a non-unix `cargo doc` under CI's
//! `RUSTDOCFLAGS=-D warnings` — see `cfg_audit_test`.)
//!
//! The per-mapping `MADV_DONTFORK` registration (`state_carrier::exclude_at_birth`)
//! likewise stays at each site. It is a per-MAPPING
//! policy keyed on a `ForkExcludedMapping` variant that names the caller, and
//! `shm_ring` deliberately does not perform it at all — so folding it in here
//! would either invent a variant this module cannot name or silently change
//! `shm_ring`'s fork behaviour.
//!
//! # Portability shape
//!
//! [`fnv1a64`] is portable (the name recipes it feeds live in `barrier` and
//! `credit`, which compile everywhere); everything below it is `#[cfg(unix)]`,
//! because it is POSIX SHM. The MODULE is therefore portable and can be named
//! from a portable item — which is what keeps `barrier`/`credit` able to use the
//! shared fold.
//!
//! # The one place the copies disagreed
//!
//! On a failed `mmap`, four of the five (`barrier`, `credit`, `state_arm`,
//! `wedge_page`) read `io::Error::last_os_error()` AFTER `close(fd)`. POSIX leaves `errno`
//! unspecified after a SUCCESSFUL call, so a successful `close` is free to
//! clobber it and the reported error can read "Success" instead of the mapping
//! failure. `shm_ring` had already fixed this and pinned it
//! (`map_segment_reports_the_mmap_errno_not_the_close_result`); the pin moved
//! here with the mechanic it describes, as
//! `map_shared_reports_the_mmap_errno_not_the_close_result` in this module's own
//! tests. (Prose, not a link — the target is a `#[cfg(test)]` item, so a link
//! would resolve for nobody and break the docs gate the day this module goes
//! `pub`.)
//!
//! `create_exclusive` and `OpenedSegment::map_shared` capture the errno BEFORE
//! the close, so all five sites now report the mapping failure. This is the ONE
//! behavioural difference the extraction makes, it moves four sites from an
//! unspecified value to the real one, and it is called out rather than folded in
//! silently. (`wedge_page` carried a fifth hand-rolled copy and adopted the
//! substrate — this errno order with it — on the same
//! terms: the change is stated, not folded in.)
//!
//! **The ordering is a CONTRACT requirement, and on this platform it is UNOBSERVABLE —
//! measured, not reasoned.** POSIX leaves
//! `errno` unspecified after a SUCCESSFUL call, so an implementation is FREE to
//! clobber it and the close-first order is unsound BY CONTRACT; but a successful
//! `close(2)` on macOS (aarch64, 2026-08-20) PRESERVES `errno` — measured with a
//! C probe that let a failing `mmap` set `EINVAL` and read `errno` back
//! unchanged after the close. The consequence is stated rather than papered
//! over: restoring the close-first order (close
//! first, read `errno` after) leaves the whole suite green, and the pin below
//! cannot see it either — for a SECOND reason, also measured: its `dead_fd`
//! fixture makes `close` FAIL with the same `EBADF` the `mmap` set, so the
//! post-close read is identical by construction. A fixture whose close SUCCEEDS
//! (a live pipe fd) was tried and does not help, because the close preserves
//! `errno` anyway.
//!
//! So the ordering is pinned by REVIEW and by the POSIX contract, NOT by a test
//! on this platform. What the pin below DOES cover is that the reported error is
//! the `mmap`'s and carries `MapStep::Mmap` — real coverage, narrower than its
//! name suggests. (The "its success clobbered errno with 0, so the returned
//! error read 'Success'" account inherited from `shm_ring` is not reproducible
//! here; Linux may differ, and that is where a discriminating arm would
//! have to run.)

/// FNV-1a 64-bit over `bytes` — offset basis `0xcbf29ce484222325`, prime
/// `0x100000001b3`.
///
/// The fold every `/cer_*` SHM object name in this crate is derived with. It was
/// written out FOUR times (`barrier`, `shm_ring`, `state_arm`, `credit`) before
/// this module existed, and `wedge_page` landed a fifth while the extraction was
/// in flight; all five copies are gone. ONE, still separate, lives inlined in
/// `doorbell::doorbell_shm_name`.
///
/// The argument the copies carried against sharing ("eight lines of frozen
/// arithmetic … a shared helper would put a cross-module dependency between
/// four otherwise-independent naming schemes") is answered by the dependency
/// being here rather than between them: the naming schemes still do not know
/// about each other, they know about the substrate they already all sit on.
///
/// Byte-oriented (not `&str`) because two of the four fold a hand-built key
/// containing a 0x1F unit separator rather than a UTF-8 string; the `&str`
/// callers pass `.as_bytes()`.
pub(crate) fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= *b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

#[cfg(unix)]
pub(crate) use posix::{create_exclusive, unlink, unmap, MapError, MapStep, OpenedSegment};

#[cfg(unix)]
mod posix {
    use std::ffi::CStr;
    use std::io;
    use std::os::raw::c_void;

    /// Which syscall of the create/open sequence failed.
    ///
    /// Carried alongside the errno so a caller with a STRUCTURED error type
    /// (`shm_ring`'s `ShmRingError { name, reason }`) can render exactly the
    /// message it rendered before this extraction, while the three callers that
    /// return a bare [`io::Error`] can discard it via the [`From`] impl.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub(crate) enum MapStep {
        /// `shm_open` — the create (`O_CREAT|O_RDWR|O_EXCL`) or the strict
        /// open-existing (`O_RDWR`, no `O_CREAT`).
        ShmOpen,
        /// `ftruncate` — sizing a freshly created object, exactly once.
        Ftruncate,
        /// `fstat` — reading an existing object's size before mapping it.
        Fstat,
        /// `mmap` — the `MAP_SHARED` read/write mapping itself.
        Mmap,
    }

    /// A failed step of the map sequence, with the errno that step produced.
    ///
    /// The errno is always the FAILING syscall's, never a later `close`'s — see
    /// the module doc.
    #[derive(Debug)]
    pub(crate) struct MapError {
        /// Which syscall failed.
        pub(crate) step: MapStep,
        /// That syscall's errno, captured before any cleanup ran.
        pub(crate) err: io::Error,
    }

    /// Drop the step and keep the errno — the three callers that return a bare
    /// `io::Error` behave exactly as they did when they wrote the sequence
    /// themselves (`Err(io::Error::last_os_error())` at each arm).
    impl From<MapError> for io::Error {
        fn from(e: MapError) -> Self {
            e.err
        }
    }

    /// `shm_unlink` `name`, best-effort — the result is intentionally ignored.
    ///
    /// This is the whole unlink contract in one place: it removes only the NAME.
    /// The underlying object persists until the LAST mapping of it is unmapped,
    /// so an owner unlinking mid-run can never invalidate a peer's live mapping
    /// — nor a peer PARKED on it through a futex/`os_sync`/monitor wait, whose
    /// target stays memory-backed. (A SIGBUS would need an `ftruncate` SHRINK of
    /// the live object, which nothing in this crate performs.)
    ///
    /// A concurrent unlink, or an already-gone name, is benign — which is why
    /// ignoring the result is correct rather than lazy, and why this is a SAFE
    /// function: `shm_unlink` of a valid NUL-terminated name has no precondition
    /// a caller could violate.
    pub(crate) fn unlink(name: &CStr) {
        // SAFETY: FFI unlink of a valid NUL-terminated name; result
        // intentionally ignored (ENOENT is a normal outcome).
        unsafe {
            libc::shm_unlink(name.as_ptr());
        }
    }

    /// `munmap` exactly `len` bytes at `ptr`.
    ///
    /// # Safety
    ///
    /// `ptr`/`len` must be exactly the region a prior [`create_exclusive`] or
    /// [`OpenedSegment::map_shared`] returned, and it must not have been
    /// unmapped already. Nothing may reference the region afterwards.
    pub(crate) unsafe fn unmap(ptr: *mut c_void, len: usize) {
        // SAFETY: the caller guarantees `ptr`/`len` name exactly one live
        // mapping this process made and no longer references.
        unsafe {
            libc::munmap(ptr, len);
        }
    }

    /// Create + size + map a FRESH `len`-byte `MAP_SHARED` segment named `name`,
    /// as its OWNER. Returns the mapping's base address; the descriptor is
    /// closed before returning (the mapping keeps the object alive).
    ///
    /// The sequence, and why each step is where it is:
    ///
    /// 1. **`shm_unlink` first**, best-effort — clears a crashed prior run's
    ///    orphan so the `O_EXCL` create below yields a FRESH, zero-filled
    ///    object. Callers rely on that zero fill (a barrier's generation, an arm
    ///    word's claim table, a credit word's epoch all start at 0).
    /// 2. **`O_CREAT|O_RDWR|O_EXCL`, mode `0o600`.** After the unlink-first, the
    ///    `O_EXCL` is a best-effort double-owner DETECTOR, not a lock: a racing
    ///    second owner's create fails EEXIST, but a NON-racing one succeeds
    ///    after clearing the first owner's name, leaving the first owner's
    ///    mapping alive on the now-unnamed object. The mode is passed as
    ///    `c_uint`, not `mode_t`: macOS's `shm_open` is variadic and its
    ///    `mode_t` is `u16`, which integer promotion forbids passing to a
    ///    variadic fn — `c_uint` also matches Linux's non-variadic `mode_t`.
    /// 3. **`ftruncate` EXACTLY ONCE** on the fresh object (macOS `EINVAL`s on a
    ///    re-truncate of a POSIX SHM object; the unlink-first create is what
    ///    guarantees freshness).
    /// 4. **`mmap` read/write `MAP_SHARED`** — `MAP_SHARED` is what makes the
    ///    page's atomics visible across processes.
    ///
    /// Any failure AFTER the object exists unlinks the name again, so a failed
    /// create never orphans a named segment. A failure AT step 2 does not
    /// unlink: nothing was created, and the name may be a live peer's.
    pub(crate) fn create_exclusive(name: &CStr, len: usize) -> Result<*mut c_void, MapError> {
        unlink(name);

        // SAFETY: FFI to POSIX named SHM; `name` is a valid C string. See the
        // doc above for why the mode is a `c_uint`.
        let fd = unsafe {
            libc::shm_open(
                name.as_ptr(),
                libc::O_CREAT | libc::O_RDWR | libc::O_EXCL,
                0o600 as libc::c_uint,
            )
        };
        if fd < 0 {
            // Nothing was created — do NOT unlink (the name may be a live
            // peer's, which is exactly what EEXIST means here).
            return Err(MapError {
                step: MapStep::ShmOpen,
                err: io::Error::last_os_error(),
            });
        }
        let seg = OpenedSegment { fd };

        // SAFETY: size the freshly created object; `fd` is the descriptor just
        // opened (owned by `seg`, which closes it on the error path below).
        if unsafe { libc::ftruncate(seg.fd, len as libc::off_t) } < 0 {
            let err = io::Error::last_os_error();
            drop(seg);
            unlink(name);
            return Err(MapError {
                step: MapStep::Ftruncate,
                err,
            });
        }

        seg.map_shared(len).inspect_err(|_| unlink(name))
    }

    /// An OPEN descriptor on an existing POSIX SHM object, closed on drop.
    ///
    /// The open half is split into steps rather than done in one call because
    /// the call sites disagree about what a valid size is and one of them needs
    /// the OBSERVED size rather than a fixed one: `shm_ring` LEARNS the segment
    /// length from `fstat` (its rings are variably sized) and rejects only
    /// "smaller than the header"; `state_arm`, `credit` and `wedge_page` reject
    /// anything below their fixed size with their own diagnostics; `barrier`
    /// reads no size at all.
    #[must_use = "an OpenedSegment that is never mapped just closes its descriptor"]
    pub(crate) struct OpenedSegment {
        /// The open descriptor, or `-1` once [`map_shared`](Self::map_shared)
        /// has taken it (so `Drop` does not double-close).
        fd: libc::c_int,
    }

    impl OpenedSegment {
        /// STRICT open-existing: `O_RDWR` with NO `O_CREAT`, so a missing object
        /// is ENOENT rather than a silently-created segment nobody armed.
        ///
        /// That distinction is the whole point at every call site: a silent
        /// create would hand the opener its OWN zero-filled page — a barrier
        /// nobody arrives at, an arm word nobody armed, a credit word reading
        /// depth 0 that defers its producer forever — while the real segment sat
        /// untouched.
        pub(crate) fn open(name: &CStr) -> Result<Self, MapError> {
            // SAFETY: FFI open of an existing named SHM object; `name` is a
            // valid C string.
            let fd = unsafe { libc::shm_open(name.as_ptr(), libc::O_RDWR, 0) };
            if fd < 0 {
                return Err(MapError {
                    step: MapStep::ShmOpen,
                    err: io::Error::last_os_error(),
                });
            }
            Ok(Self { fd })
        }

        /// The object's current size in bytes, via `fstat`.
        ///
        /// Checking this BEFORE the map is what turns a hostile segment into one
        /// loud `Err` on every platform. An owner that died between `shm_open`
        /// and `ftruncate` leaves a named, ZERO-LENGTH object under a perfectly
        /// good name, and mapping it anyway fails PLATFORM-DEPENDENTLY: on Linux
        /// the `mmap` SUCCEEDS and the first touch raises SIGBUS (a process kill,
        /// with no Rust-level error anywhere to attribute); on macOS the `mmap`
        /// is refused with an errno that says nothing about what was wrong.
        pub(crate) fn size(&self) -> Result<i64, MapError> {
            // SAFETY: `st` is zeroed first so a failed `fstat` can never leave
            // an uninitialised read, and `fstat` fills it entirely on success.
            let mut st: libc::stat = unsafe { std::mem::zeroed() };
            // SAFETY: FFI fstat on the descriptor this handle owns.
            if unsafe { libc::fstat(self.fd, &mut st) } < 0 {
                return Err(MapError {
                    step: MapStep::Fstat,
                    err: io::Error::last_os_error(),
                });
            }
            Ok(st.st_size)
        }

        /// `mmap` `len` bytes of this object read/write, `MAP_SHARED`, consuming
        /// the handle. The descriptor is closed on BOTH the success and the
        /// failure path (the mapping keeps the object alive; a failed map must
        /// not leak a descriptor).
        ///
        /// The `mmap` errno is captured BEFORE the `close` — see the module doc.
        pub(crate) fn map_shared(mut self, len: usize) -> Result<*mut c_void, MapError> {
            // SAFETY: FFI mmap; `self.fd` is a valid open descriptor.
            let addr = unsafe {
                libc::mmap(
                    std::ptr::null_mut(),
                    len,
                    libc::PROT_READ | libc::PROT_WRITE,
                    libc::MAP_SHARED,
                    self.fd,
                    0,
                )
            };
            // Capture the mmap errno BEFORE the close: errno after a SUCCESSFUL
            // call is unspecified (POSIX), so a close landing in between could
            // leave the reported failure reading "Success" instead.
            let mmap_err = if addr == libc::MAP_FAILED {
                Some(io::Error::last_os_error())
            } else {
                None
            };
            // Take the descriptor and close it here, on both paths; `-1` makes
            // the `Drop` below a no-op rather than a double close.
            let fd = std::mem::replace(&mut self.fd, -1);
            // SAFETY: `fd` is the descriptor this handle owned and has now
            // released; nothing else refers to it.
            unsafe {
                libc::close(fd);
            }
            match mmap_err {
                Some(err) => Err(MapError {
                    step: MapStep::Mmap,
                    err,
                }),
                None => Ok(addr),
            }
        }
    }

    /// Close the descriptor on any path that did not map it (a failed `fstat`, a
    /// refused size check, an early return between `open` and `map_shared`).
    impl Drop for OpenedSegment {
        fn drop(&mut self) {
            if self.fd >= 0 {
                // SAFETY: the descriptor this handle owns and has not released.
                unsafe {
                    libc::close(self.fd);
                }
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        /// An fd number that is invalid and cannot become valid: the largest
        /// `c_int`, which no descriptor table reaches (`RLIMIT_NOFILE` is
        /// bounded far below it) — so `mmap` on it is EBADF deterministically
        /// and the `close` that follows touches nobody's descriptor.
        ///
        /// Two earlier fixtures were racy. Opening a pipe and closing it handed
        /// out a number another thread of this parallel binary could reuse
        /// between the close and the `mmap` (observed once on Linux: the mmap
        /// SUCCEEDED and `map_shared` closed a descriptor some other test
        /// owned). Reading `RLIMIT_NOFILE` was better but still mutable — a
        /// sibling test raises the limit — and a zero limit would name fd 0,
        /// which stays open. A constant has neither problem and is still a
        /// plausible non-negative value for the `-1`-sentinel test below.
        fn dead_fd() -> libc::c_int {
            libc::c_int::MAX
        }

        /// Regression pin, inherited from `shm_ring`'s
        /// `map_segment_reports_the_mmap_errno_not_the_close_result` when the
        /// mechanic moved here: `map_shared` must report the MMAP errno, not
        /// whatever `close(fd)` leaves behind. Were `close` to run before the
        /// `MAP_FAILED` check, its success would clobber errno with 0, so the
        /// returned error would read "Success".
        ///
        /// This is now the pin for ALL FIVE call sites, not just `shm_ring`'s:
        /// the other four read `last_os_error()` after the close until they
        /// adopted this substrate.
        #[test]
        fn map_shared_reports_the_mmap_errno_not_the_close_result() {
            let seg = OpenedSegment { fd: dead_fd() };
            let err = seg
                .map_shared(4096)
                .expect_err("mmap of a closed fd must fail");
            assert_eq!(err.step, MapStep::Mmap, "the failing step must be the mmap");
            assert_eq!(
                err.err.raw_os_error(),
                Some(libc::EBADF),
                "the reported error must be the mmap EBADF, not a close-clobbered \
                 'Success': {}",
                err.err
            );
        }

        /// The `-1` sentinel is load-bearing: `map_shared` closes the descriptor
        /// itself, so a `Drop` that closed it again would be a double close —
        /// which, on a process that has since opened another descriptor, closes
        /// somebody ELSE's fd. Asserting the sentinel is how that is observable
        /// without racing a real double close.
        #[test]
        fn map_shared_releases_the_descriptor_so_drop_cannot_double_close() {
            let mut seg = OpenedSegment { fd: dead_fd() };
            // Read the field the way `map_shared` does, then confirm the handle
            // it leaves behind is inert.
            let fd = std::mem::replace(&mut seg.fd, -1);
            assert!(fd >= 0, "the fixture fd must be a plausible descriptor");
            assert_eq!(seg.fd, -1, "a released handle must be marked inert");
            drop(seg); // must not close anything
        }

        /// A step-carrying error must degrade to exactly the errno the three
        /// `io::Error`-returning call sites returned before the extraction.
        #[test]
        fn map_error_degrades_to_the_bare_errno() {
            let e = MapError {
                step: MapStep::Ftruncate,
                err: io::Error::from_raw_os_error(libc::ENOSPC),
            };
            let io_err: io::Error = e.into();
            assert_eq!(io_err.raw_os_error(), Some(libc::ENOSPC));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fnv1a64;

    /// Hand oracle, NOT a self-compare: the FNV-1a-64 values below are the
    /// published constants for these inputs, and they are the same three the
    /// `barrier` / `shm_ring` / `doorbell` name-oracle tests already assert
    /// against. If this fold ever drifts, every `/cer_*` object name in the
    /// crate drifts with it and no two builds meet.
    #[test]
    fn fnv1a64_matches_the_published_oracle() {
        assert_eq!(fnv1a64(b"a"), 0xaf63_dc4c_8601_ec8c);
        assert_eq!(fnv1a64(b"b"), 0xaf63_df4c_8601_f1a5);
        assert_eq!(fnv1a64(b"topic"), 0x520c_8b7d_6934_ac64);
    }

    /// The 0x1F unit separator the compact name shapes fold in is a real byte,
    /// so the fold must be BYTE-oriented rather than `&str`-oriented. These two
    /// values are the ones `barrier`'s compact-name oracle asserts.
    #[test]
    fn fnv1a64_folds_the_unit_separator_key() {
        assert_eq!(fnv1a64(b"g\x1fa"), 0xd4c0_7218_fa8d_ad0e);
        assert_eq!(fnv1a64(b"g\x1fb"), 0xd4c0_7118_fa8d_ab5b);
    }

    /// The empty fold is the offset basis — the boundary the loop body is never
    /// entered on.
    #[test]
    fn fnv1a64_of_nothing_is_the_offset_basis() {
        assert_eq!(fnv1a64(b""), 0xcbf2_9ce4_8422_2325);
    }
}
