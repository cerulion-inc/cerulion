// SPDX-License-Identifier: AGPL-3.0-only
//! The HostShm backend: host shared memory as its own "device".
//!
//! Registration is the identity, a device address is the host address, and
//! every copy is a `memcpy`. It needs no GPU, so it runs everywhere. The
//! conformance suite runs it beside the vendor backends: each backend's
//! bytes are compared with a known pattern, and HostShm's identity
//! translation is the expected answer for its handed-out addresses.

use crate::backend::{
    BackendKind, Caps, DeviceAddress, DeviceBackend, DeviceUuid, Fence, SegmentDevice, Stream,
};
use crate::error::{DeviceError, DeviceResult};
use crate::segment::SegmentSpan;

/// Host shared memory with no GPU.
#[derive(Debug)]
pub struct HostShm {
    caps: Caps,
}

impl HostShm {
    /// The HostShm backend. It is always available.
    pub fn new() -> Self {
        Self {
            caps: Caps {
                kind: BackendKind::HostShm,
                name: "host shared memory".to_string(),
                uuid: DeviceUuid::default(),
                integrated: true,
                host_register: true,
                read_only_register: true,
                host_pointer_is_device_pointer: true,
                posix_fd_export: true,
                ipc_event: false,
                mps: None,
            },
        }
    }
}

impl Default for HostShm {
    fn default() -> Self {
        Self::new()
    }
}

fn flat(address: DeviceAddress) -> DeviceResult<*mut u8> {
    match address {
        DeviceAddress::Flat(ptr) => Ok(ptr as usize as *mut u8),
        DeviceAddress::Buffer { .. } => Err(DeviceError::Unsupported {
            backend: BackendKind::HostShm,
            what: "a buffer-object device address",
        }),
    }
}

impl DeviceBackend for HostShm {
    fn caps(&self) -> &Caps {
        &self.caps
    }

    unsafe fn register_segment(&self, span: &SegmentSpan) -> DeviceResult<SegmentDevice> {
        Ok(SegmentDevice::Flat {
            base: span.base as u64,
        })
    }

    unsafe fn unregister_segment(
        &self,
        _span: &SegmentSpan,
        _device: SegmentDevice,
    ) -> DeviceResult<()> {
        Ok(())
    }

    unsafe fn copy_to_host(&self, src: DeviceAddress, dst: &mut [u8]) -> DeviceResult<()> {
        let src = flat(src)?;
        // SAFETY: the caller guarantees `src` addresses `dst.len()` live
        // bytes; `dst` is a distinct borrowed slice.
        unsafe { std::ptr::copy(src, dst.as_mut_ptr(), dst.len()) };
        Ok(())
    }

    unsafe fn copy_from_host(&self, dst: DeviceAddress, src: &[u8]) -> DeviceResult<()> {
        let dst = flat(dst)?;
        // SAFETY: the caller guarantees `dst` addresses `src.len()` live,
        // writable bytes.
        unsafe { std::ptr::copy(src.as_ptr(), dst, src.len()) };
        Ok(())
    }

    fn fence_record(&self, _stream: Stream) -> DeviceResult<Fence> {
        Ok(Fence::CpuComplete)
    }

    fn fence_wait(&self, fence: &Fence, _stream: Stream) -> DeviceResult<()> {
        match fence {
            Fence::CpuComplete => Ok(()),
        }
    }
}
