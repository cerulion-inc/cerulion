// SPDX-License-Identifier: AGPL-3.0-only
//! The backend conformance suite: one set of checks, run against every
//! backend that probes on this machine.
//!
//! HostShm always runs, so CI exercises the suite on every platform. A
//! vendor backend whose library or device is absent is reported as SKIP on
//! stderr and does not run. Set `CERULION_DEVICE_REQUIRE` to a comma list of
//! backend names (`cuda,metal`) to make a missing backend a failure. That is
//! how a hardware run proves a backend ran, not just that nothing failed.
//!
//! Each check registers a real POSIX shared-memory object, the kind of
//! mapping the transport hands out, and moves bytes through the DEVICE's
//! copy path (`cuMemcpyDtoH` on CUDA, a blit on Metal). That proves the
//! device address the registry returns points at the right bytes. The
//! cross-process check maps the object read-only in a child process, as a
//! subscriber does, and reads it through the device there.
//!
//! ```bash
//! cargo test -p cerulion_device --test conformance_test
//! ```

use std::ffi::CString;
use std::process::Command;
use std::sync::Arc;

use cerulion_device::{
    Access, BackendKind, Cuda, DeviceBackend, DeviceError, Fence, Hip, HostShm, Metal,
    SegmentRegistry, Stream,
};

/// The bytes every check writes: a fixed function of the index, so any
/// shifted, truncated or stale range reads differently.
fn pattern(seed: u32, n: usize) -> Vec<u8> {
    (0..n)
        .map(|i| {
            let x = (i as u32).wrapping_mul(2_654_435_761) ^ seed.wrapping_mul(0x9E37_79B9);
            (x ^ (x >> 15)) as u8
        })
        .collect()
}

#[test]
fn the_pattern_matches_its_hand_computed_prefix() {
    // seed 7, i = 0: x = 7 * 0x9E3779B9 mod 2^32 = 0x5384_540F, and
    // x >> 15 = 0xA708, so the low byte of x ^ (x >> 15) is 0x0F ^ 0x08.
    // i = 1: x = 0x9E37_79B1 ^ 0x5384_540F = 0xCDB3_2DBE, x >> 15 = 0x1_9B66,
    // so the low byte is 0xBE ^ 0x66.
    assert_eq!(pattern(7, 2), vec![0x07, 0xd8]);
}

/// FNV-1a over `bytes`, the checksum the child reports.
fn fnv1a(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, b| {
        (h ^ u64::from(*b)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}

/// A POSIX shared-memory object mapped into this process.
struct Shm {
    name: CString,
    ptr: *mut u8,
    len: usize,
    owner: bool,
}

impl Shm {
    fn create(tag: &str, len: usize) -> Shm {
        let name = CString::new(format!("/cdev-{}-{tag}", std::process::id())).unwrap();
        // SAFETY: plain libc calls on a fresh name; results checked.
        unsafe {
            libc::shm_unlink(name.as_ptr());
            let fd = libc::shm_open(
                name.as_ptr(),
                libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
                0o600,
            );
            assert!(fd >= 0, "shm_open: {}", std::io::Error::last_os_error());
            assert_eq!(libc::ftruncate(fd, len as libc::off_t), 0);
            let shm = Shm::map(name, fd, len, Access::ReadWrite, true);
            libc::close(fd);
            shm
        }
    }

    fn open(name: &str, len: usize, access: Access) -> Shm {
        let name = CString::new(name).unwrap();
        // The fd is read-write either way; `access` sets the mapping's
        // protection, which is what the registry reads. A read-write fd lets
        // one check raise a read-only mapping's protection later.
        // SAFETY: plain libc calls; results checked.
        unsafe {
            let fd = libc::shm_open(name.as_ptr(), libc::O_RDWR, 0);
            assert!(fd >= 0, "shm_open: {}", std::io::Error::last_os_error());
            let shm = Shm::map(name, fd, len, access, false);
            libc::close(fd);
            shm
        }
    }

    /// # Safety
    /// `fd` is an open shm object of at least `len` bytes.
    unsafe fn map(name: CString, fd: libc::c_int, len: usize, access: Access, owner: bool) -> Shm {
        let prot = match access {
            Access::ReadWrite => libc::PROT_READ | libc::PROT_WRITE,
            Access::ReadOnly => libc::PROT_READ,
        };
        // SAFETY: forwarded from the caller.
        let ptr = unsafe { libc::mmap(std::ptr::null_mut(), len, prot, libc::MAP_SHARED, fd, 0) };
        assert_ne!(
            ptr,
            libc::MAP_FAILED,
            "mmap: {}",
            std::io::Error::last_os_error()
        );
        Shm {
            name,
            ptr: ptr.cast(),
            len,
            owner,
        }
    }

    fn bytes(&self) -> &[u8] {
        // SAFETY: a live mapping of `len` bytes.
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }

    fn bytes_mut(&mut self) -> &mut [u8] {
        // SAFETY: a live, writable mapping of `len` bytes (callers only use
        // this on read-write mappings).
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.len) }
    }
}

impl Drop for Shm {
    fn drop(&mut self) {
        // SAFETY: unmaps the mapping made in `map`.
        unsafe {
            libc::munmap(self.ptr.cast(), self.len);
            if self.owner {
                libc::shm_unlink(self.name.as_ptr());
            }
        }
    }
}

type Probe = Result<Arc<dyn DeviceBackend>, DeviceError>;

/// The backend names `CERULION_DEVICE_REQUIRE` lists. A name that is not a
/// backend fails, so a typo cannot turn a required run into a skip.
fn required() -> Vec<String> {
    let names: Vec<String> = std::env::var("CERULION_DEVICE_REQUIRE")
        .unwrap_or_default()
        .split(',')
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .collect();
    for name in &names {
        assert!(
            BackendKind::ALL.iter().any(|k| k.name() == name),
            "CERULION_DEVICE_REQUIRE names {name:?}, which is not one of host_shm, cuda, hip, metal"
        );
    }
    names
}

/// Every backend that probes here, plus a SKIP line for each that does not.
fn backends() -> Vec<Arc<dyn DeviceBackend>> {
    let required = required();
    let probes: [(BackendKind, Probe); 4] = [
        (BackendKind::HostShm, Ok(Arc::new(HostShm::new()))),
        (BackendKind::Cuda, Cuda::probe(0).map(|b| Arc::new(b) as _)),
        (BackendKind::Hip, Hip::probe(0).map(|b| Arc::new(b) as _)),
        (BackendKind::Metal, Metal::probe().map(|b| Arc::new(b) as _)),
    ];
    let mut out = Vec::new();
    for (kind, probe) in probes {
        match probe {
            Ok(backend) if !backend.caps().host_register => {
                assert!(
                    !required.iter().any(|r| r == kind.name()),
                    "{kind} is required but its device cannot register host memory"
                );
                eprintln!("SKIP {kind}: the device cannot register host memory");
            }
            Ok(backend) => {
                eprintln!("RUN  {kind}: {:?}", backend.caps());
                out.push(backend);
            }
            Err(error) => {
                assert!(
                    !required.iter().any(|r| r == kind.name()),
                    "{kind} is required by CERULION_DEVICE_REQUIRE but did not probe: {error}"
                );
                eprintln!("SKIP {kind}: {error}");
            }
        }
    }
    out
}

const LEN: usize = 1 << 20;

/// A read-only attach on a device without read-only registration must fail
/// with the named refusal, never register read-write instead.
fn assert_refused_read_only(kind: BackendKind, attached: Result<(), DeviceError>) {
    match attached {
        Err(DeviceError::Unsupported { what, .. }) => {
            assert_eq!(what, "read-only host memory registration", "{kind}");
        }
        other => panic!("{kind}: read-only attach without support returned {other:?}"),
    }
}

#[test]
fn attach_registers_the_whole_mapping_once() {
    for backend in backends() {
        let kind = backend.caps().kind;
        let shm = Shm::create(&format!("once-{kind}"), LEN);
        let registry = SegmentRegistry::new(backend);
        // SAFETY: `shm` outlives `registry` (declared first, dropped last).
        let span = unsafe { registry.attach(shm.ptr.wrapping_add(12_345)) }.unwrap();
        assert_eq!(span.base, shm.ptr as usize, "{kind}");
        assert_eq!(span.len, LEN, "{kind}");
        assert_eq!(span.access, Access::ReadWrite, "{kind}");
        // SAFETY: as above.
        let again = unsafe { registry.attach(shm.ptr.wrapping_add(LEN - 1)) }.unwrap();
        assert_eq!(again, span, "{kind}");
        assert_eq!(registry.len(), 1, "{kind}");
    }
}

#[test]
fn the_device_reads_what_the_cpu_wrote_at_every_offset() {
    for backend in backends() {
        let kind = backend.caps().kind;
        let mut shm = Shm::create(&format!("read-{kind}"), LEN);
        let expected = pattern(11, LEN);
        shm.bytes_mut().copy_from_slice(&expected);
        let registry = SegmentRegistry::new(backend);
        // SAFETY: `shm` outlives `registry`.
        unsafe { registry.attach(shm.ptr) }.unwrap();
        for (offset, n) in [(0, 4), (256, 4096), (LEN - 4096, 4096), (0, LEN)] {
            let mut out = vec![0u8; n];
            registry
                .copy_to_host(&shm.bytes()[offset..offset + n], &mut out)
                .unwrap_or_else(|e| panic!("{kind} offset {offset}: {e}"));
            assert!(
                out == expected[offset..offset + n],
                "{kind} offset {offset} len {n}"
            );
        }
    }
}

#[test]
fn the_cpu_reads_what_the_device_wrote() {
    for backend in backends() {
        let kind = backend.caps().kind;
        let mut shm = Shm::create(&format!("write-{kind}"), LEN);
        let registry = SegmentRegistry::new(backend);
        // SAFETY: `shm` outlives `registry`.
        unsafe { registry.attach(shm.ptr) }.unwrap();
        let data = pattern(23, 64 * 1024);
        let dst = &mut shm.bytes_mut()[8192..8192 + data.len()];
        registry.copy_from_host(&data, dst).unwrap();
        assert!(shm.bytes()[8192..8192 + data.len()] == data[..], "{kind}");
        assert!(
            shm.bytes()[..8192].iter().all(|b| *b == 0),
            "{kind}: bytes before the range changed"
        );
    }
}

#[test]
fn a_read_only_mapping_is_readable_and_refuses_writes() {
    for backend in backends() {
        let kind = backend.caps().kind;
        let mut writer = Shm::create(&format!("ro-{kind}"), LEN);
        let expected = pattern(5, LEN);
        writer.bytes_mut().copy_from_slice(&expected);
        let reader = Shm::open(writer.name.to_str().unwrap(), LEN, Access::ReadOnly);
        let read_only = backend.caps().read_only_register;
        let registry = SegmentRegistry::new(backend);
        // SAFETY: `reader` outlives `registry`.
        let attached = unsafe { registry.attach(reader.ptr) };
        if !read_only {
            // A device without read-only registration (a discrete RTX 5090
            // reports 0) refuses by name, before calling the driver.
            assert_refused_read_only(kind, attached.map(|_| ()));
            continue;
        }
        let span = attached.unwrap_or_else(|e| panic!("{kind}: read-only register: {e}"));
        assert_eq!(span.access, Access::ReadOnly, "{kind}");
        let mut out = vec![0u8; 4096];
        registry
            .copy_to_host(&reader.bytes()[4096..8192], &mut out)
            .unwrap();
        assert!(out == expected[4096..8192], "{kind}");
        let err = registry.device_address_mut(reader.ptr, 16).unwrap_err();
        assert!(
            matches!(err, DeviceError::ReadOnly { len: 16, .. }),
            "{kind}: {err}"
        );
    }
}

#[test]
fn ranges_outside_a_registered_segment_are_refused() {
    for backend in backends() {
        let kind = backend.caps().kind;
        let shm = Shm::create(&format!("range-{kind}"), LEN);
        let other = Shm::create(&format!("range2-{kind}"), LEN);
        let mut registry = SegmentRegistry::new(backend);
        // SAFETY: `shm` outlives `registry`.
        unsafe { registry.attach(shm.ptr) }.unwrap();
        let past_end = registry.device_address(shm.ptr.wrapping_add(LEN - 8), 16);
        assert!(
            matches!(past_end, Err(DeviceError::NotRegistered { len: 16, .. })),
            "{kind}"
        );
        let unattached = registry.device_address(other.ptr, 1);
        assert!(
            matches!(unattached, Err(DeviceError::NotRegistered { .. })),
            "{kind}"
        );
        assert!(registry.detach(shm.ptr).unwrap(), "{kind}");
        assert!(!registry.detach(shm.ptr).unwrap(), "{kind}: detached twice");
        let after = registry.device_address(shm.ptr, 1);
        assert!(
            matches!(after, Err(DeviceError::NotRegistered { .. })),
            "{kind}"
        );
        assert!(registry.is_empty(), "{kind}");
    }
}

#[test]
fn an_unmapped_address_is_refused_at_attach() {
    for backend in backends() {
        let registry = SegmentRegistry::new(backend);
        // Page zero is unmapped (Linux) or a no-access guard region (macOS).
        // SAFETY: attach fails before registering anything.
        let err = unsafe { registry.attach(std::ptr::dangling::<u8>()) }.unwrap_err();
        assert!(matches!(err, DeviceError::NoMapping { .. }), "{err}");
    }
}

#[test]
fn fences_round_trip_on_the_default_stream() {
    for backend in backends() {
        let fence = backend.fence_record(Stream::DEFAULT).unwrap();
        assert_eq!(fence, Fence::CpuComplete);
        backend.fence_wait(&fence, Stream::DEFAULT).unwrap();
    }
}

#[test]
fn two_device_reads_of_one_range_are_bit_identical() {
    for backend in backends() {
        let kind = backend.caps().kind;
        let mut shm = Shm::create(&format!("det-{kind}"), LEN);
        shm.bytes_mut().copy_from_slice(&pattern(31, LEN));
        let registry = SegmentRegistry::new(backend);
        // SAFETY: `shm` outlives `registry`.
        unsafe { registry.attach(shm.ptr) }.unwrap();
        let (mut a, mut b) = (vec![0u8; LEN], vec![0u8; LEN]);
        registry.copy_to_host(shm.bytes(), &mut a).unwrap();
        registry.copy_to_host(shm.bytes(), &mut b).unwrap();
        assert!(a == b, "{kind}");
    }
}

/// The child half of the cross-process check: open the parent's object
/// read-only, register it with the named backend, read it through the
/// device and print its checksum.
#[test]
fn cross_process_child() {
    let (Ok(name), Ok(kind)) = (
        std::env::var("CDEV_CHILD_SHM"),
        std::env::var("CDEV_CHILD_BACKEND"),
    ) else {
        return;
    };
    let backend = backends()
        .into_iter()
        .find(|b| b.caps().kind.name() == kind)
        .expect("child: backend did not probe");
    let read_only = backend.caps().read_only_register;
    let ro = Shm::open(&name, LEN, Access::ReadOnly);
    let registry = SegmentRegistry::new(backend);
    if !read_only {
        // SAFETY: `ro` outlives `registry`.
        let refused = unsafe { registry.attach(ro.ptr) };
        assert_refused_read_only(registry.backend().caps().kind, refused.map(|_| ()));
        println!("CHILD_REFUSED_READ_ONLY");
    }
    // Without read-only registration the child maps the object read-write
    // instead, so the device read in a second process still runs.
    let rw;
    let reader = if read_only {
        &ro
    } else {
        rw = Shm::open(&name, LEN, Access::ReadWrite);
        &rw
    };
    // SAFETY: `reader` outlives `registry`.
    unsafe { registry.attach(reader.ptr) }.unwrap();
    let mut out = vec![0u8; LEN];
    registry.copy_to_host(reader.bytes(), &mut out).unwrap();
    println!("CHILD_FNV={:016x}", fnv1a(&out));
}

#[test]
fn a_second_process_reads_the_segment_through_its_device() {
    let exe = std::env::current_exe().unwrap();
    for backend in backends() {
        let kind = backend.caps().kind;
        let mut shm = Shm::create(&format!("xp-{kind}"), LEN);
        let data = pattern(47, LEN);
        shm.bytes_mut().copy_from_slice(&data);
        let out = Command::new(&exe)
            .args([
                "--exact",
                "cross_process_child",
                "--nocapture",
                "--test-threads=1",
            ])
            .env("CDEV_CHILD_SHM", shm.name.to_str().unwrap())
            .env("CDEV_CHILD_BACKEND", kind.name())
            .output()
            .unwrap();
        let stdout = String::from_utf8_lossy(&out.stdout);
        assert!(
            out.status.success(),
            "{kind} child failed: {stdout}\n{}",
            String::from_utf8_lossy(&out.stderr)
        );
        let line = format!("CHILD_FNV={:016x}", fnv1a(&data));
        assert!(
            stdout.contains(&line),
            "{kind}: expected {line}, child said: {stdout}"
        );
        if !backend.caps().read_only_register {
            assert!(
                stdout.contains("CHILD_REFUSED_READ_ONLY"),
                "{kind}: {stdout}"
            );
        }
    }
}

#[test]
fn copies_refuse_mismatched_lengths_and_writes_through_read_only_segments() {
    for backend in backends() {
        let kind = backend.caps().kind;
        let mut writer = Shm::create(&format!("len-{kind}"), LEN);
        let registry = SegmentRegistry::new(backend.clone());
        // SAFETY: `writer` outlives `registry`.
        unsafe { registry.attach(writer.ptr) }.unwrap();
        let mut small = [0u8; 8];
        let err = registry
            .copy_to_host(&writer.bytes()[..16], &mut small)
            .unwrap_err();
        assert_eq!(
            err,
            DeviceError::LengthMismatch { src: 16, dst: 8 },
            "{kind}"
        );
        let err = registry
            .copy_from_host(&[1u8; 4], &mut writer.bytes_mut()[..8])
            .unwrap_err();
        assert_eq!(
            err,
            DeviceError::LengthMismatch { src: 4, dst: 8 },
            "{kind}"
        );
        if !backend.caps().read_only_register {
            continue;
        }
        let reader = Shm::open(writer.name.to_str().unwrap(), LEN, Access::ReadOnly);
        let ro = SegmentRegistry::new(backend);
        // SAFETY: `reader` outlives `ro`.
        unsafe { ro.attach(reader.ptr) }.unwrap();
        // Make the pages writable AFTER the registry cached the segment as
        // read-only, so a mutable slice over them is sound; the registry
        // must still refuse from its cached access.
        // SAFETY: `reader` is a live mapping of LEN bytes of a shared object
        // opened read-write by `writer`.
        let rc =
            unsafe { libc::mprotect(reader.ptr.cast(), LEN, libc::PROT_READ | libc::PROT_WRITE) };
        assert_eq!(rc, 0, "mprotect: {}", std::io::Error::last_os_error());
        // SAFETY: the mapping is writable now, and nothing else borrows it.
        let target = unsafe { std::slice::from_raw_parts_mut(reader.ptr, 8) };
        let err = ro.copy_from_host(&[1u8; 8], target).unwrap_err();
        assert!(
            matches!(err, DeviceError::ReadOnly { len: 8, .. }),
            "{kind}: {err}"
        );
    }
}

#[test]
fn a_backend_refuses_the_other_address_shape() {
    use cerulion_device::{BufferHandle, DeviceAddress};
    for backend in backends() {
        let kind = backend.caps().kind;
        let wrong = match kind {
            BackendKind::Metal => DeviceAddress::Flat(0),
            _ => DeviceAddress::Buffer {
                buffer: BufferHandle::from_ptr(std::ptr::null_mut()),
                offset: 0,
            },
        };
        let mut out = [0u8; 4];
        // SAFETY: the address has the wrong shape, so the backend refuses it
        // before touching memory.
        let err = unsafe { backend.copy_to_host(wrong, &mut out) }.unwrap_err();
        assert!(
            matches!(err, DeviceError::Unsupported { .. }),
            "{kind}: {err}"
        );
        // SAFETY: as above.
        let err = unsafe { backend.copy_from_host(wrong, &out) }.unwrap_err();
        assert!(
            matches!(err, DeviceError::Unsupported { .. }),
            "{kind}: {err}"
        );
    }
}

#[test]
fn concurrent_first_attaches_register_the_mapping_once() {
    for backend in backends() {
        let kind = backend.caps().kind;
        let shm = Shm::create(&format!("race-{kind}"), LEN);
        let registry = SegmentRegistry::new(backend);
        let base = shm.ptr as usize;
        std::thread::scope(|scope| {
            for t in 0..8 {
                let registry = &registry;
                scope.spawn(move || {
                    // SAFETY: `shm` outlives the scope and the registry.
                    unsafe { registry.attach((base + t * 4096) as *const u8) }.unwrap();
                });
            }
        });
        assert_eq!(registry.len(), 1, "{kind}");
    }
}

#[test]
fn metal_refuses_an_unaligned_blit_through_the_registry() {
    let Some(backend) = backends()
        .into_iter()
        .find(|b| b.caps().kind == BackendKind::Metal)
    else {
        return;
    };
    let mut shm = Shm::create("unaligned-metal", LEN);
    shm.bytes_mut()[..8].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
    let registry = SegmentRegistry::new(backend);
    // SAFETY: `shm` outlives `registry`.
    unsafe { registry.attach(shm.ptr) }.unwrap();
    let mut out = [0u8; 3];
    let err = registry
        .copy_to_host(&shm.bytes()[1..4], &mut out)
        .unwrap_err();
    assert!(matches!(err, DeviceError::Unsupported { .. }), "{err}");
}
