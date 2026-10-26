// SPDX-License-Identifier: AGPL-3.0-only
//! The CUDA backend, over the driver API in `libcuda`, opened at run time.
//!
//! UNIFIED residence registers a host mapping with `cuMemHostRegister`
//! and translates addresses through `cuMemHostGetDevicePointer`. On Jetson
//! Orin the device address of a registered page is not its host address
//! (`CAN_USE_HOST_POINTER_FOR_REGISTERED_MEM` is 0), so translation is per
//! segment, and a subscriber's read-only mapping is registered with
//! `CU_MEMHOSTREGISTER_READ_ONLY`.
//!
//! The backend uses the device's primary context, the one the CUDA runtime,
//! PyTorch and CuPy share, and pushes it around each call so a caller's
//! current context is left as it was.

use std::ffi::{c_char, c_int, c_uint, c_void, CStr};

use crate::backend::{
    Access, BackendKind, Caps, DeviceAddress, DeviceBackend, DeviceUuid, Fence, SegmentDevice,
    Stream,
};
use crate::error::{DeviceError, DeviceResult};
use crate::loader::{fn_table, Library};
use crate::segment::SegmentSpan;

type CuResult = c_int;
type CuDevice = c_int;
type CuContext = *mut c_void;
type CuDevicePtr = u64;
type CuStream = *mut c_void;

/// Library names tried, in order.
const LIBRARIES: &[&str] = &["libcuda.so.1", "libcuda.so"];

/// `cuMemHostRegister` flags, from `cuda.h`.
const MEMHOSTREGISTER_PORTABLE: c_uint = 0x01;
const MEMHOSTREGISTER_DEVICEMAP: c_uint = 0x02;
const MEMHOSTREGISTER_READ_ONLY: c_uint = 0x08;

/// `CUdevice_attribute` values used here, from `cuda.h` (CUDA 12.6).
mod attr {
    pub(super) const INTEGRATED: i32 = 18;
    pub(super) const CAN_MAP_HOST_MEMORY: i32 = 19;
    pub(super) const CAN_USE_HOST_POINTER_FOR_REGISTERED_MEM: i32 = 91;
    pub(super) const HOST_REGISTER_SUPPORTED: i32 = 99;
    pub(super) const VIRTUAL_MEMORY_MANAGEMENT_SUPPORTED: i32 = 102;
    pub(super) const HANDLE_TYPE_POSIX_FILE_DESCRIPTOR_SUPPORTED: i32 = 103;
    pub(super) const READ_ONLY_HOST_REGISTER_SUPPORTED: i32 = 113;
    pub(super) const IPC_EVENT_SUPPORTED: i32 = 125;
    pub(super) const MPS_ENABLED: i32 = 133;
}

fn_table! {
    /// The driver entry points this backend calls.
    CudaApi {
        init: fn(c_uint) -> CuResult = "cuInit";
        device_get_count: fn(*mut c_int) -> CuResult = "cuDeviceGetCount";
        device_get: fn(*mut CuDevice, c_int) -> CuResult = "cuDeviceGet";
        device_get_name: fn(*mut c_char, c_int, CuDevice) -> CuResult = "cuDeviceGetName";
        device_get_uuid: fn(*mut [u8; 16], CuDevice) -> CuResult = "cuDeviceGetUuid_v2";
        device_get_attribute: fn(*mut c_int, c_int, CuDevice) -> CuResult = "cuDeviceGetAttribute";
        primary_ctx_retain: fn(*mut CuContext, CuDevice) -> CuResult = "cuDevicePrimaryCtxRetain";
        primary_ctx_release: fn(CuDevice) -> CuResult = "cuDevicePrimaryCtxRelease_v2";
        ctx_push: fn(CuContext) -> CuResult = "cuCtxPushCurrent_v2";
        ctx_pop: fn(*mut CuContext) -> CuResult = "cuCtxPopCurrent_v2";
        mem_host_register: fn(*mut c_void, usize, c_uint) -> CuResult = "cuMemHostRegister_v2";
        mem_host_unregister: fn(*mut c_void) -> CuResult = "cuMemHostUnregister";
        mem_host_get_device_pointer: fn(*mut CuDevicePtr, *mut c_void, c_uint) -> CuResult = "cuMemHostGetDevicePointer_v2";
        memcpy_dtoh: fn(*mut c_void, CuDevicePtr, usize) -> CuResult = "cuMemcpyDtoH_v2";
        memcpy_htod: fn(CuDevicePtr, *const c_void, usize) -> CuResult = "cuMemcpyHtoD_v2";
        stream_synchronize: fn(CuStream) -> CuResult = "cuStreamSynchronize";
        get_error_string: fn(CuResult, *mut *const c_char) -> CuResult = "cuGetErrorString";
    }
}

/// One CUDA device, through its primary context.
#[derive(Debug)]
pub struct Cuda {
    api: CudaApi,
    device: CuDevice,
    ctx: usize,
    caps: Caps,
    // Kept open for the life of the backend: `api` points into it.
    _lib: Library,
}

impl Cuda {
    /// Open `libcuda`, initialize the driver and retain the primary context
    /// of device `ordinal`.
    pub fn probe(ordinal: i32) -> DeviceResult<Self> {
        let lib = Library::open(BackendKind::Cuda, LIBRARIES)?;
        // SAFETY: the rows of `CudaApi` match the declarations in cuda.h.
        let api = unsafe { CudaApi::load(&lib) }?;
        let check = |call, code| check(&api, call, code);
        // SAFETY: plain driver calls with live out-pointers.
        unsafe {
            let init = (api.init)(0);
            if init == ERROR_NO_DEVICE {
                return Err(DeviceError::NoDevice {
                    backend: BackendKind::Cuda,
                    detail: "cuInit reported CUDA_ERROR_NO_DEVICE".to_string(),
                });
            }
            check("cuInit", init)?;
            let mut count = 0;
            check("cuDeviceGetCount", (api.device_get_count)(&mut count))?;
            if ordinal < 0 || ordinal >= count {
                return Err(DeviceError::NoDevice {
                    backend: BackendKind::Cuda,
                    detail: format!("ordinal {ordinal} requested, {count} device(s) present"),
                });
            }
            let mut device = 0;
            check("cuDeviceGet", (api.device_get)(&mut device, ordinal))?;
            let mut name = [0 as c_char; 256];
            check(
                "cuDeviceGetName",
                (api.device_get_name)(name.as_mut_ptr(), name.len() as c_int, device),
            )?;
            let mut uuid = [0u8; 16];
            check(
                "cuDeviceGetUuid_v2",
                (api.device_get_uuid)(&mut uuid, device),
            )?;
            let attr = |a| read_attr(&api, device, a);
            let on = |a| attr(a).map(|v| v.is_some_and(|v| v != 0));
            let caps = Caps {
                kind: BackendKind::Cuda,
                name: CStr::from_ptr(name.as_ptr()).to_string_lossy().into_owned(),
                uuid: DeviceUuid(uuid),
                integrated: on(attr::INTEGRATED)?,
                host_register: on(attr::HOST_REGISTER_SUPPORTED)? && on(attr::CAN_MAP_HOST_MEMORY)?,
                read_only_register: on(attr::READ_ONLY_HOST_REGISTER_SUPPORTED)?,
                host_pointer_is_device_pointer: on(attr::CAN_USE_HOST_POINTER_FOR_REGISTERED_MEM)?,
                posix_fd_export: on(attr::VIRTUAL_MEMORY_MANAGEMENT_SUPPORTED)?
                    && on(attr::HANDLE_TYPE_POSIX_FILE_DESCRIPTOR_SUPPORTED)?,
                ipc_event: on(attr::IPC_EVENT_SUPPORTED)?,
                mps: attr(attr::MPS_ENABLED)?.map(|v| v != 0),
            };
            let mut ctx: CuContext = std::ptr::null_mut();
            check(
                "cuDevicePrimaryCtxRetain",
                (api.primary_ctx_retain)(&mut ctx, device),
            )?;
            tracing::info!(
                backend = "cuda",
                library = lib.name(),
                device = %caps.name,
                uuid = %caps.uuid,
                integrated = caps.integrated,
                mps = ?caps.mps,
                "probed device"
            );
            Ok(Self {
                api,
                device,
                ctx: ctx as usize,
                caps,
                _lib: lib,
            })
        }
    }

    fn check(&self, call: &'static str, code: CuResult) -> DeviceResult<()> {
        check(&self.api, call, code)
    }

    /// Run `f` with this device's primary context current, then restore the
    /// caller's context.
    fn in_context<T>(&self, f: impl FnOnce() -> DeviceResult<T>) -> DeviceResult<T> {
        // SAFETY: `ctx` is the retained primary context; push and pop are
        // paired on this thread.
        unsafe {
            self.check(
                "cuCtxPushCurrent_v2",
                (self.api.ctx_push)(self.ctx as CuContext),
            )?;
        }
        let out = f();
        let mut popped: CuContext = std::ptr::null_mut();
        // SAFETY: pops the context pushed above.
        let pop = self.check("cuCtxPopCurrent_v2", unsafe {
            (self.api.ctx_pop)(&mut popped)
        });
        match (out, pop) {
            (Ok(out), Ok(())) => Ok(out),
            // The call itself succeeded (a registration exists), so its
            // result is returned; reporting it as an error would leak it.
            (Ok(out), Err(error)) => {
                tracing::warn!(%error, "context pop after a successful call failed");
                Ok(out)
            }
            (Err(out), pop) => {
                if let Err(error) = pop {
                    tracing::warn!(%error, "context pop after a failed call also failed");
                }
                Err(out)
            }
        }
    }
}

/// `CUDA_ERROR_INVALID_VALUE`: what an older driver returns for an
/// attribute it does not know.
const ERROR_INVALID_VALUE: CuResult = 1;

/// `CUDA_ERROR_NO_DEVICE`: what `cuInit` returns on a machine with the
/// driver library and no CUDA device.
const ERROR_NO_DEVICE: CuResult = 100;

/// Read one device attribute. `Ok(None)` when the driver does not know the
/// attribute (logged); any other failure is an error.
fn read_attr(api: &CudaApi, device: CuDevice, attribute: c_int) -> DeviceResult<Option<c_int>> {
    let mut v = 0;
    // SAFETY: a live out-pointer and a device from cuDeviceGet.
    let code = unsafe { (api.device_get_attribute)(&mut v, attribute, device) };
    if code == ERROR_INVALID_VALUE {
        tracing::warn!(
            backend = "cuda",
            attribute,
            "the driver does not know this attribute; treating it as absent"
        );
        return Ok(None);
    }
    check(api, "cuDeviceGetAttribute", code)?;
    Ok(Some(v))
}

fn check(api: &CudaApi, call: &'static str, code: CuResult) -> DeviceResult<()> {
    if code == 0 {
        return Ok(());
    }
    let mut text: *const c_char = std::ptr::null();
    // SAFETY: cuGetErrorString writes a pointer to a static string or
    // leaves it null.
    let message = if unsafe { (api.get_error_string)(code, &mut text) } == 0 && !text.is_null() {
        // SAFETY: non-null, NUL-terminated, static.
        unsafe { CStr::from_ptr(text) }
            .to_string_lossy()
            .into_owned()
    } else {
        "unknown".to_string()
    };
    Err(DeviceError::Driver {
        backend: BackendKind::Cuda,
        call,
        code: i64::from(code),
        message,
    })
}

fn flat(address: DeviceAddress) -> DeviceResult<CuDevicePtr> {
    match address {
        DeviceAddress::Flat(ptr) => Ok(ptr),
        DeviceAddress::Buffer { .. } => Err(DeviceError::Unsupported {
            backend: BackendKind::Cuda,
            what: "a buffer-object device address",
        }),
    }
}

/// The `cuMemHostRegister` flags for a mapping with `access`.
fn register_flags(access: Access) -> c_uint {
    let base = MEMHOSTREGISTER_PORTABLE | MEMHOSTREGISTER_DEVICEMAP;
    match access {
        Access::ReadWrite => base,
        Access::ReadOnly => base | MEMHOSTREGISTER_READ_ONLY,
    }
}

impl DeviceBackend for Cuda {
    fn caps(&self) -> &Caps {
        &self.caps
    }

    unsafe fn register_segment(&self, span: &SegmentSpan) -> DeviceResult<SegmentDevice> {
        if !self.caps.host_register {
            return Err(DeviceError::Unsupported {
                backend: BackendKind::Cuda,
                what: "host memory registration",
            });
        }
        if span.access == Access::ReadOnly && !self.caps.read_only_register {
            return Err(DeviceError::Unsupported {
                backend: BackendKind::Cuda,
                what: "read-only host memory registration",
            });
        }
        let base = span.base as *mut c_void;
        self.in_context(|| {
            // SAFETY: the caller guarantees `span` is a whole live mapping.
            unsafe {
                self.check(
                    "cuMemHostRegister_v2",
                    (self.api.mem_host_register)(base, span.len, register_flags(span.access)),
                )?;
            }
            let mut device: CuDevicePtr = 0;
            // SAFETY: `base` was registered just above.
            let got = unsafe { (self.api.mem_host_get_device_pointer)(&mut device, base, 0) };
            if let Err(e) = self.check("cuMemHostGetDevicePointer_v2", got) {
                // SAFETY: undoes the registration made above.
                let undo = unsafe { (self.api.mem_host_unregister)(base) };
                if let Err(error) = self.check("cuMemHostUnregister", undo) {
                    tracing::warn!(%error, "rollback of a half-made registration failed");
                }
                return Err(e);
            }
            Ok(SegmentDevice::Flat { base: device })
        })
    }

    unsafe fn unregister_segment(
        &self,
        span: &SegmentSpan,
        _device: SegmentDevice,
    ) -> DeviceResult<()> {
        self.in_context(|| {
            // SAFETY: the caller passes a span this backend registered.
            let code = unsafe { (self.api.mem_host_unregister)(span.base as *mut c_void) };
            self.check("cuMemHostUnregister", code)
        })
    }

    unsafe fn copy_to_host(&self, src: DeviceAddress, dst: &mut [u8]) -> DeviceResult<()> {
        let src = flat(src)?;
        self.in_context(|| {
            // SAFETY: the caller guarantees `src` addresses dst.len() bytes.
            let code = unsafe { (self.api.memcpy_dtoh)(dst.as_mut_ptr().cast(), src, dst.len()) };
            self.check("cuMemcpyDtoH_v2", code)
        })
    }

    unsafe fn copy_from_host(&self, dst: DeviceAddress, src: &[u8]) -> DeviceResult<()> {
        let dst = flat(dst)?;
        self.in_context(|| {
            // SAFETY: the caller guarantees `dst` addresses src.len()
            // writable bytes.
            let code = unsafe { (self.api.memcpy_htod)(dst, src.as_ptr().cast(), src.len()) };
            self.check("cuMemcpyHtoD_v2", code)
        })
    }

    fn fence_record(&self, stream: Stream) -> DeviceResult<Fence> {
        self.in_context(|| {
            // SAFETY: the caller's stream handle, or null for the default.
            let code = unsafe { (self.api.stream_synchronize)(stream.as_raw()) };
            self.check("cuStreamSynchronize", code)
        })?;
        Ok(Fence::CpuComplete)
    }

    fn fence_wait(&self, fence: &Fence, _stream: Stream) -> DeviceResult<()> {
        match fence {
            Fence::CpuComplete => Ok(()),
        }
    }
}

impl Drop for Cuda {
    fn drop(&mut self) {
        // SAFETY: releases the retain taken in `probe`.
        let code = unsafe { (self.api.primary_ctx_release)(self.device) };
        if let Err(error) = self.check("cuDevicePrimaryCtxRelease_v2", code) {
            tracing::warn!(%error, "primary context release failed");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_symbol_table_names_the_versioned_driver_entry_points() {
        // cuda.h maps these names onto `_v2` entry points; the unversioned
        // symbols are the old ABI and must never be read.
        for sym in [
            "cuMemHostRegister_v2",
            "cuMemHostGetDevicePointer_v2",
            "cuMemcpyDtoH_v2",
            "cuMemcpyHtoD_v2",
            "cuCtxPushCurrent_v2",
            "cuCtxPopCurrent_v2",
            "cuDevicePrimaryCtxRelease_v2",
            "cuDeviceGetUuid_v2",
        ] {
            assert!(CudaApi::SYMBOLS.contains(&sym), "{sym}");
        }
        for old in ["cuMemHostRegister", "cuMemcpyDtoH", "cuCtxPushCurrent"] {
            assert!(!CudaApi::SYMBOLS.contains(&old), "{old}");
        }
    }

    #[test]
    fn a_reader_mapping_registers_read_only_and_a_writer_does_not() {
        assert_eq!(register_flags(Access::ReadWrite), 0x03);
        assert_eq!(register_flags(Access::ReadOnly), 0x0b);
    }
}
