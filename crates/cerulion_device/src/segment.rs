// SPDX-License-Identifier: AGPL-3.0-only
//! Host segments: finding the mapping that holds an address, and the
//! per-process registry of segments registered with a backend.
//!
//! A shared-memory transport hands out slots inside a few large mappings.
//! Registering a mapping with a GPU is expensive (S0 measured 0.3 to 0.7 ms
//! per MiB on Jetson Orin), so it happens once per mapping, and every slot
//! address is then translated with an offset from the cached segment base.

use std::sync::{Arc, RwLock, RwLockReadGuard, RwLockWriteGuard};

use crate::backend::{Access, DeviceAddress, DeviceBackend, SegmentDevice};
use crate::error::{DeviceError, DeviceResult};

/// One whole mapping of this process.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SegmentSpan {
    /// First byte of the mapping (page-aligned).
    pub base: usize,
    /// Length in bytes (a whole number of pages).
    pub len: usize,
    /// Whether the mapping is writable.
    pub access: Access,
}

impl SegmentSpan {
    /// Whether `[addr, addr + len)` lies inside this span. The first byte,
    /// `addr`, must be inside even when `len` is 0.
    pub fn contains(&self, addr: usize, len: usize) -> bool {
        let end_of_span = self.base + self.len;
        addr >= self.base
            && addr < end_of_span
            && addr.checked_add(len).is_some_and(|end| end <= end_of_span)
    }
}

/// The mapping of this process that contains `addr`.
///
/// Linux reads `/proc/self/maps`; macOS asks `mach_vm_region`. This reads
/// the whole memory map, so call it when a port connects, never per message.
pub fn span_of(addr: usize) -> DeviceResult<SegmentSpan> {
    platform::span_of(addr)
}

/// Parse one `/proc/<pid>/maps` line into its range and access, or `None`
/// for a line that does not have the `start-end perms` shape, names a
/// mapping the process cannot read (a `---p` guard region), or names a
/// private mapping (`p` in the fourth perms column).
///
/// Only a shared mapping is a segment. The kernel merges adjacent private
/// anonymous mappings with equal flags into one line, so the line holding a
/// heap or stack address can span memory the caller never handed over; a
/// shared mapping's line covers one object.
pub fn parse_maps_line(line: &str) -> Option<SegmentSpan> {
    let mut fields = line.split_ascii_whitespace();
    let (start, end) = fields.next()?.split_once('-')?;
    let perms = fields.next()?.as_bytes();
    let start = usize::from_str_radix(start, 16).ok()?;
    let end = usize::from_str_radix(end, 16).ok()?;
    if end <= start || perms.len() < 4 || perms[0] != b'r' || perms[3] != b's' {
        return None;
    }
    let access = if perms[1] == b'w' {
        Access::ReadWrite
    } else {
        Access::ReadOnly
    };
    Some(SegmentSpan {
        base: start,
        len: end - start,
        access,
    })
}

#[cfg(target_os = "linux")]
mod platform {
    use super::{parse_maps_line, SegmentSpan};
    use crate::error::{DeviceError, DeviceResult};

    pub(super) fn span_of(addr: usize) -> DeviceResult<SegmentSpan> {
        let maps =
            std::fs::read_to_string("/proc/self/maps").map_err(|e| DeviceError::MemoryMap {
                reason: e.to_string(),
            })?;
        maps.lines()
            .filter_map(parse_maps_line)
            .find(|span| span.contains(addr, 1))
            .ok_or(DeviceError::NoMapping { addr })
    }
}

#[cfg(target_os = "macos")]
mod platform {
    use super::SegmentSpan;
    use crate::backend::Access;
    use crate::error::{DeviceError, DeviceResult};

    /// `VM_INHERIT_SHARE` from `<mach/vm_inherit.h>`.
    const VM_INHERIT_SHARE: libc::c_uint = 0;

    /// `VM_REGION_BASIC_INFO_64` from `<mach/vm_region.h>`.
    const VM_REGION_BASIC_INFO_64: libc::c_int = 9;

    /// `struct vm_region_basic_info_64`, declared under `#pragma pack(4)`.
    #[repr(C, packed(4))]
    #[derive(Default)]
    struct BasicInfo64 {
        protection: libc::c_int,
        max_protection: libc::c_int,
        inheritance: libc::c_uint,
        shared: libc::c_uint,
        reserved: libc::c_uint,
        offset: u64,
        behavior: libc::c_int,
        user_wired_count: libc::c_ushort,
    }

    /// `VM_REGION_BASIC_INFO_COUNT_64`: the struct size in `int`s.
    const BASIC_INFO_COUNT: u32 =
        (std::mem::size_of::<BasicInfo64>() / std::mem::size_of::<libc::c_int>()) as u32;

    extern "C" {
        /// The task port of this process; `mach_task_self()` in C reads it.
        static mach_task_self_: libc::mach_port_t;

        fn mach_vm_region(
            target_task: libc::mach_port_t,
            address: *mut u64,
            size: *mut u64,
            flavor: libc::c_int,
            info: *mut libc::c_int,
            info_count: *mut u32,
            object_name: *mut libc::mach_port_t,
        ) -> libc::c_int;
    }

    pub(super) fn span_of(addr: usize) -> DeviceResult<SegmentSpan> {
        let mut start = addr as u64;
        let mut size = 0u64;
        let mut info = BasicInfo64::default();
        let mut count = BASIC_INFO_COUNT;
        let mut object: libc::mach_port_t = 0;
        // SAFETY: every out-pointer is a live local of the type the call
        // writes; `count` holds the capacity of `info` in ints.
        let kr = unsafe {
            mach_vm_region(
                mach_task_self_,
                &mut start,
                &mut size,
                VM_REGION_BASIC_INFO_64,
                (&mut info as *mut BasicInfo64).cast(),
                &mut count,
                &mut object,
            )
        };
        // KERN_INVALID_ADDRESS: nothing is mapped at or above the address.
        if kr == 1 {
            return Err(DeviceError::NoMapping { addr });
        }
        if kr != 0 {
            return Err(DeviceError::MemoryMap {
                reason: format!("mach_vm_region returned {kr}"),
            });
        }
        // mach_vm_region returns the first region at or ABOVE the address,
        // so a hole below it means the address itself is unmapped.
        let span = SegmentSpan {
            base: start as usize,
            len: size as usize,
            access: if info.protection & libc::VM_PROT_WRITE != 0 {
                Access::ReadWrite
            } else {
                Access::ReadOnly
            },
        };
        // A region the process cannot read (a guard page) is not a segment,
        // and neither is a private one: adjacent private regions coalesce,
        // so the region holding a heap address can span unrelated memory.
        // mmap(MAP_SHARED) sets VM_INHERIT_SHARE. `shared` is not read: it
        // is 0 for a shared region with no resident page yet.
        if span.contains(addr, 1)
            && info.protection & libc::VM_PROT_READ != 0
            && info.inheritance == VM_INHERIT_SHARE
        {
            Ok(span)
        } else {
            Err(DeviceError::NoMapping { addr })
        }
    }
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
mod platform {
    use super::SegmentSpan;
    use crate::error::{DeviceError, DeviceResult};

    pub(super) fn span_of(_addr: usize) -> DeviceResult<SegmentSpan> {
        Err(DeviceError::MemoryMap {
            reason: "segment lookup is implemented for Linux and macOS only".to_string(),
        })
    }
}

#[derive(Debug)]
struct Entry {
    span: SegmentSpan,
    device: SegmentDevice,
}

/// The segments this process has registered with one backend.
///
/// Registration is per mapping: [`Self::attach`] registers the mapping that
/// holds an address the first time and is a lookup after that. Translation
/// ([`Self::device_address`]) takes a read lock and scans the few cached
/// segments; it never allocates and never reads the memory map.
///
/// Keep one registry per backend per process: CUDA and HIP refuse to
/// register the same mapping twice. Dropping the registry unregisters every segment.
/// A first `attach` of a mapping holds the write lock while the driver
/// registers it, so translations on other threads wait for that one call.
#[derive(Debug)]
pub struct SegmentRegistry {
    backend: Arc<dyn DeviceBackend>,
    entries: RwLock<Vec<Entry>>,
}

impl SegmentRegistry {
    /// An empty registry over `backend`.
    pub fn new(backend: Arc<dyn DeviceBackend>) -> Self {
        Self {
            backend,
            // hot-path-alloc-ok: constructor, once per backend per process;
            // Vec::new reserves nothing until the first attach.
            entries: RwLock::new(Vec::new()),
        }
    }

    /// The backend segments are registered with.
    pub fn backend(&self) -> &Arc<dyn DeviceBackend> {
        &self.backend
    }

    /// How many segments are registered.
    pub fn len(&self) -> usize {
        self.read().len()
    }

    /// Whether no segment is registered.
    pub fn is_empty(&self) -> bool {
        self.read().is_empty()
    }

    /// Register the mapping that holds `addr`, unless it already is, and
    /// return its span.
    ///
    /// Reads the process memory map and calls the driver, so call it when a
    /// port connects.
    ///
    /// # Safety
    ///
    /// The mapping holding `addr` must stay mapped until [`Self::detach`]
    /// removes it or the registry is dropped.
    pub unsafe fn attach(&self, addr: *const u8) -> DeviceResult<SegmentSpan> {
        let addr = addr as usize;
        if let Some(entry) = self.read().iter().find(|e| e.span.contains(addr, 1)) {
            return Ok(entry.span);
        }
        let span = span_of(addr)?;
        let mut entries = self.write();
        // Another thread may have registered it between the two locks.
        if let Some(entry) = entries.iter().find(|e| e.span.contains(addr, 1)) {
            return Ok(entry.span);
        }
        // SAFETY: `span` is a whole mapping of this process, read from the
        // memory map just now; the caller keeps it mapped until detach.
        let device = unsafe { self.backend.register_segment(&span) }?;
        tracing::debug!(
            backend = %self.backend.caps().kind,
            base = span.base,
            size_bytes = span.len,
            access = ?span.access,
            "registered segment"
        );
        entries.push(Entry { span, device });
        Ok(span)
    }

    /// Unregister the segment that holds `addr`. Returns `false` when none
    /// does.
    ///
    /// Takes `&mut self`, so no copy or `device_address*` call through this
    /// registry can be running while a segment goes away. A
    /// [`DeviceAddress`] read out earlier is invalid after this, and the
    /// caller must finish its own device work on the segment first.
    ///
    /// On `Err` the segment stays registered, so the mapping must stay
    /// mapped (the [`Self::attach`] contract still holds); `Drop` tries
    /// again.
    pub fn detach(&mut self, addr: *const u8) -> DeviceResult<bool> {
        let addr = addr as usize;
        let entries = self.entries.get_mut().unwrap_or_else(|p| p.into_inner());
        let Some(index) = entries.iter().position(|e| e.span.contains(addr, 1)) else {
            return Ok(false);
        };
        let entry = &entries[index];
        // SAFETY: the pair came from a successful register_segment; it is
        // removed from the table only once unregistering succeeds, so it is
        // never unregistered twice and never forgotten while still registered.
        unsafe { self.backend.unregister_segment(&entry.span, entry.device) }?;
        entries.swap_remove(index);
        Ok(true)
    }

    /// The device address of `[ptr, ptr + len)`, for the device to read.
    pub fn device_address(&self, ptr: *const u8, len: usize) -> DeviceResult<DeviceAddress> {
        self.lookup(ptr as usize, len).map(|(_, address)| address)
    }

    /// The device address of `[ptr, ptr + len)`, for the device to write.
    /// Refuses a range in a segment registered read-only.
    pub fn device_address_mut(&self, ptr: *mut u8, len: usize) -> DeviceResult<DeviceAddress> {
        let addr = ptr as usize;
        let (access, address) = self.lookup(addr, len)?;
        match access {
            Access::ReadWrite => Ok(address),
            Access::ReadOnly => Err(DeviceError::ReadOnly { addr, len }),
        }
    }

    /// Copy the registered range `src` into `dst` through the device. The
    /// two lengths must be equal.
    pub fn copy_to_host(&self, src: &[u8], dst: &mut [u8]) -> DeviceResult<()> {
        same_len(src.len(), dst.len())?;
        let address = self.device_address(src.as_ptr(), src.len())?;
        // SAFETY: the registry checked that `src` is inside a registered
        // segment; the borrow keeps it live, and `detach` needs `&mut self`,
        // so the segment stays registered for the call.
        unsafe { self.backend.copy_to_host(address, dst) }
    }

    /// Copy `src` into the registered range `dst` through the device. The
    /// two lengths must be equal.
    pub fn copy_from_host(&self, src: &[u8], dst: &mut [u8]) -> DeviceResult<()> {
        same_len(src.len(), dst.len())?;
        let address = self.device_address_mut(dst.as_mut_ptr(), dst.len())?;
        // SAFETY: the registry checked that `dst` is inside a segment
        // registered read-write; the borrow keeps it live, and `detach` needs
        // `&mut self`, so the segment stays registered for the call.
        unsafe { self.backend.copy_from_host(address, src) }
    }

    fn lookup(&self, addr: usize, len: usize) -> DeviceResult<(Access, DeviceAddress)> {
        self.read()
            .iter()
            .find(|e| e.span.contains(addr, len))
            .map(|e| (e.span.access, e.device.at(addr - e.span.base)))
            .ok_or(DeviceError::NotRegistered { addr, len })
    }

    fn read(&self) -> RwLockReadGuard<'_, Vec<Entry>> {
        self.entries.read().unwrap_or_else(|p| p.into_inner())
    }

    fn write(&self) -> RwLockWriteGuard<'_, Vec<Entry>> {
        self.entries.write().unwrap_or_else(|p| p.into_inner())
    }
}

fn same_len(src: usize, dst: usize) -> DeviceResult<()> {
    if src == dst {
        Ok(())
    } else {
        Err(DeviceError::LengthMismatch { src, dst })
    }
}

impl Drop for SegmentRegistry {
    fn drop(&mut self) {
        let entries = std::mem::take(self.entries.get_mut().unwrap_or_else(|p| p.into_inner()));
        for entry in entries {
            // SAFETY: each pair came from a successful register_segment and
            // was taken out of the table, so it is unregistered once.
            if let Err(error) =
                unsafe { self.backend.unregister_segment(&entry.span, entry.device) }
            {
                tracing::warn!(%error, base = entry.span.base, "unregister on drop failed");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_writable_shared_line_parses_to_its_range() {
        let line = "ffff8a200000-ffff8a206000 rw-s 00000000 00:1a 123 /dev/shm/iox2_abc.data";
        assert_eq!(
            parse_maps_line(line),
            Some(SegmentSpan {
                base: 0xffff_8a20_0000,
                len: 0x6000,
                access: Access::ReadWrite
            })
        );
    }

    #[test]
    fn a_read_only_line_parses_as_read_only() {
        let line = "7f0000000000-7f0000001000 r--s 00000000 00:1a 9 /dev/shm/x";
        assert_eq!(
            parse_maps_line(line).map(|s| s.access),
            Some(Access::ReadOnly)
        );
    }

    #[test]
    fn a_shared_line_parses_and_a_private_one_is_not_a_segment() {
        let shared = "7f0000000000-7f0000021000 rw-s 00000000 00:1a 9 /dev/shm/x";
        assert_eq!(parse_maps_line(shared).map(|s| s.len), Some(0x21000));
        // Heap and anonymous private memory: the kernel merges neighbours.
        let private = "55d0c0000000-55d0c0021000 rw-p 00000000 00:00 0";
        assert_eq!(parse_maps_line(private), None);
        assert_eq!(parse_maps_line("7f0000000000-7f0000001000 rw-"), None);
    }

    #[test]
    fn an_unreadable_guard_mapping_is_not_a_segment() {
        assert_eq!(
            parse_maps_line("7f0000000000-7f0000001000 ---p 00000000 00:00 0"),
            None
        );
        assert_eq!(
            parse_maps_line("7f0000000000-7f0000001000 -w-s 00000000 00:1a 9 /x"),
            None
        );
    }

    #[test]
    fn malformed_lines_are_rejected() {
        for line in [
            "",
            "zz-10 rw-p",
            "1000-1000 rw-p",
            "2000-1000 rw-p",
            "1000-2000",
            "1000 rw-p",
        ] {
            assert_eq!(parse_maps_line(line), None, "{line:?}");
        }
    }

    #[test]
    fn contains_checks_both_ends_and_overflow() {
        let span = SegmentSpan {
            base: 0x1000,
            len: 0x1000,
            access: Access::ReadWrite,
        };
        assert!(span.contains(0x1000, 0x1000));
        assert!(span.contains(0x1fff, 1));
        assert!(!span.contains(0x1fff, 2));
        assert!(!span.contains(0xfff, 1));
        assert!(!span.contains(0x2000, 0));
        assert!(!span.contains(0x1800, usize::MAX));
    }
}
