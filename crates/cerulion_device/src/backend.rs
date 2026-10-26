// SPDX-License-Identifier: AGPL-3.0-only
//! The [`DeviceBackend`] trait and the value types every backend shares.

use std::fmt;

use crate::error::DeviceResult;
use crate::segment::SegmentSpan;

/// Which backend produced or can import a resident payload.
///
/// The discriminant is the value the device descriptor carries on the wire,
/// so it never changes for an existing variant.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BackendKind {
    /// Plain host shared memory. Every vendor backend is tested against it.
    HostShm = 0,
    /// NVIDIA CUDA, through the driver API in `libcuda`.
    Cuda = 1,
    /// AMD HIP, through `libamdhip64`.
    Hip = 2,
    /// Apple Metal, through `Metal.framework`.
    Metal = 3,
}

impl BackendKind {
    /// Every backend, in wire order.
    pub const ALL: [BackendKind; 4] = [Self::HostShm, Self::Cuda, Self::Hip, Self::Metal];

    /// The backend a wire value names, or `None` for a value this build does
    /// not know.
    pub fn from_wire(value: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|kind| *kind as u8 == value)
    }

    /// The lowercase name used in logs, config and error text.
    pub fn name(self) -> &'static str {
        match self {
            Self::HostShm => "host_shm",
            Self::Cuda => "cuda",
            Self::Hip => "hip",
            Self::Metal => "metal",
        }
    }

    /// Whether this backend has passed its conformance suite on real
    /// hardware. HIP is built and unit tested but has not run on an AMD
    /// device, so it reports `false` until it has.
    pub fn hardware_validated(self) -> bool {
        !matches!(self, Self::Hip)
    }
}

impl fmt::Display for BackendKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// A device identity that survives reboots and renumbering: the vendor's
/// 16-byte UUID, never an ordinal.
///
/// Metal has no device UUID. Its backend stores the 64-bit `registryID`
/// little-endian in the first 8 bytes and zeroes the rest; a registry ID is
/// stable only for one boot of one machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct DeviceUuid(pub [u8; 16]);

impl fmt::Display for DeviceUuid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, byte) in self.0.iter().enumerate() {
            if matches!(i, 4 | 6 | 8 | 10) {
                f.write_str("-")?;
            }
            write!(f, "{byte:02x}")?;
        }
        Ok(())
    }
}

/// What this process may do to a mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Access {
    /// The mapping is writable. A publisher's data segment.
    ReadWrite,
    /// The mapping is read-only. A subscriber's view of a publisher's data
    /// segment. CUDA registers it with its read-only flag. Metal has none,
    /// so for Metal the registry's refusal of device writes
    /// ([`crate::SegmentRegistry::device_address_mut`]) is the only guard.
    /// HIP refuses it: ROCm ignores its read-only flag.
    ReadOnly,
}

/// What a probed device can do, read from the driver once at probe time.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Caps {
    /// The backend these capabilities belong to.
    pub kind: BackendKind,
    /// The device name the driver reports.
    pub name: String,
    /// The device identity.
    pub uuid: DeviceUuid,
    /// The device shares DRAM with the CPU (Jetson, Apple silicon, AMD APUs).
    pub integrated: bool,
    /// Host memory can be registered so the device addresses it in place.
    /// UNIFIED residence needs this.
    pub host_register: bool,
    /// A read-only host mapping can be registered, which a subscriber
    /// needs: a reader on a device that reports `false` (an RTX 5090 does)
    /// cannot use UNIFIED residence. CUDA registers with the driver's
    /// read-only flag. Metal has no such flag: it wraps the read-only
    /// mapping as it is, and only this crate's refusal of
    /// `device_address_mut` on that segment keeps a write away; a kernel
    /// handed the buffer is not stopped by the driver.
    pub read_only_register: bool,
    /// A registered host address is also the device address. When `false`
    /// (Jetson Orin, for one) every pointer is translated per segment.
    pub host_pointer_is_device_pointer: bool,
    /// Device memory can be exported as a POSIX file descriptor. DEVICE
    /// residence needs this.
    pub posix_fd_export: bool,
    /// Events can be shared between processes.
    pub ipc_event: bool,
    /// CUDA only: contexts on this device are shared through an MPS daemon.
    /// `None` for a backend that has no such notion, or a driver too old to
    /// report it. Without MPS a CUDA
    /// handoff between two processes pays a context switch on the GPU, so
    /// every latency figure must say which of the two it was measured under.
    pub mps: Option<bool>,
}

/// Where the device sees a range of registered host memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceAddress {
    /// A pointer in the device's address space (CUDA, HIP, HostShm).
    Flat(u64),
    /// A buffer object plus a byte offset into it (Metal: an `MTLBuffer`
    /// that wraps the whole segment).
    Buffer {
        /// The backend's buffer object.
        buffer: BufferHandle,
        /// Byte offset of the range inside the buffer.
        offset: usize,
    },
}

/// An opaque, backend-owned buffer object (a retained `MTLBuffer` for
/// Metal). It stays valid while its segment is registered.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BufferHandle(usize);

impl BufferHandle {
    /// Wrap a backend object pointer.
    pub fn from_ptr(ptr: *mut std::ffi::c_void) -> Self {
        Self(ptr as usize)
    }

    /// The object pointer, for passing to the vendor API (for Metal, an
    /// `id<MTLBuffer>` to hand to an encoder's `setBuffer`).
    pub fn as_ptr(self) -> *mut std::ffi::c_void {
        self.0 as *mut std::ffi::c_void
    }
}

/// How a registered segment appears on the device, as returned by
/// [`DeviceBackend::register_segment`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentDevice {
    /// The segment's first byte is at this device address.
    Flat {
        /// Device address of the segment base.
        base: u64,
    },
    /// The segment is wrapped by one buffer object.
    Buffer(BufferHandle),
}

impl SegmentDevice {
    /// The device address of the byte `offset` bytes into the segment.
    pub fn at(self, offset: usize) -> DeviceAddress {
        match self {
            Self::Flat { base } => DeviceAddress::Flat(base + offset as u64),
            Self::Buffer(buffer) => DeviceAddress::Buffer { buffer, offset },
        }
    }
}

/// A vendor stream or queue handle, passed through untouched.
///
/// [`Stream::DEFAULT`] is the null handle, which CUDA and HIP read as the
/// legacy default stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stream(usize);

impl Stream {
    /// The default stream.
    pub const DEFAULT: Stream = Stream(0);

    /// Wrap a raw vendor handle (`CUstream`, `hipStream_t`).
    pub fn from_raw(handle: *mut std::ffi::c_void) -> Self {
        Self(handle as usize)
    }

    /// The raw vendor handle.
    pub fn as_raw(self) -> *mut std::ffi::c_void {
        self.0 as *mut std::ffi::c_void
    }
}

/// Proof that work queued before it has finished, in a form a reader can
/// wait on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fence {
    /// The producer's work on the bytes was complete when the fence was
    /// recorded, so a reader has nothing to wait for. CUDA and HIP record it
    /// by synchronizing the stream; Metal records it only for the default
    /// stream, after the caller has waited on its command buffer.
    CpuComplete,
}

/// One GPU vendor's implementation of device residence.
///
/// Every method that names an address is `unsafe`: the caller states that
/// the address is live for the call. [`crate::SegmentRegistry`] wraps the
/// trait with safe, bounds-checked methods, and is what code outside a
/// backend should use.
pub trait DeviceBackend: Send + Sync + fmt::Debug {
    /// What the probed device can do.
    fn caps(&self) -> &Caps;

    /// Make a host mapping addressable by the device.
    ///
    /// Called once per segment, never per message.
    ///
    /// # Safety
    ///
    /// `span` must be a whole mapping of this process with the access it
    /// names, and must stay mapped until [`Self::unregister_segment`] runs.
    unsafe fn register_segment(&self, span: &SegmentSpan) -> DeviceResult<SegmentDevice>;

    /// Undo [`Self::register_segment`].
    ///
    /// # Safety
    ///
    /// `span` and `device` must be one pair returned by a successful
    /// `register_segment` on this backend, successfully unregistered at
    /// most once; a call that failed may be retried.
    unsafe fn unregister_segment(
        &self,
        span: &SegmentSpan,
        device: SegmentDevice,
    ) -> DeviceResult<()>;

    /// Copy `dst.len()` bytes from device address `src` into host memory,
    /// using the device's own access path, and return once the copy is done.
    ///
    /// # Safety
    ///
    /// `src` must address at least `dst.len()` live bytes.
    unsafe fn copy_to_host(&self, src: DeviceAddress, dst: &mut [u8]) -> DeviceResult<()>;

    /// Copy `src` into device address `dst`, using the device's own access
    /// path, and return once the copy is done.
    ///
    /// # Safety
    ///
    /// `dst` must address at least `src.len()` live, writable bytes.
    unsafe fn copy_from_host(&self, dst: DeviceAddress, src: &[u8]) -> DeviceResult<()>;

    /// Record a fence after the work queued on `stream`.
    fn fence_record(&self, stream: Stream) -> DeviceResult<Fence>;

    /// Make `stream` wait for `fence` before its later work runs.
    fn fence_wait(&self, fence: &Fence, stream: Stream) -> DeviceResult<()>;
}
