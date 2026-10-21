// SPDX-License-Identifier: AGPL-3.0-only
//! The HIP backend, over `libamdhip64`, opened at run time.
//!
//! **Built, not run.** No AMD device has run this backend yet, so
//! [`BackendKind::hardware_validated`] is `false` for it and `probe` logs a
//! warning that says so. What is tested without hardware: the symbol table,
//! the attribute and flag values (read from `hip_runtime_api.h` of ROCm
//! 6.2 by compiling its `hipDeviceAttribute_t` enum) and the absent-library
//! path.
//!
//! UNIFIED residence on an APU (Strix Halo, MI300A) registers the mapping
//! with `hipHostRegister` and translates addresses with
//! `hipHostGetDevicePointer`, the same shape as CUDA. ROCm's runtime ignores
//! `hipHostRegisterReadOnly` (the value is shared with
//! `hipExtHostRegisterCoarseGrained`) and its device properties report
//! read-only registration as unsupported, so this backend reports
//! `read_only_register = false` and refuses a read-only mapping by name.

use std::ffi::{c_char, c_int, c_uint, c_void, CStr};

use crate::backend::{
    Access, BackendKind, Caps, DeviceAddress, DeviceBackend, DeviceUuid, Fence, SegmentDevice,
    Stream,
};
use crate::error::{DeviceError, DeviceResult};
use crate::loader::{fn_table, Library};
use crate::segment::SegmentSpan;

type HipError = c_int;
type HipDevice = c_int;
type HipStream = *mut c_void;

/// Library names tried, in order.
const LIBRARIES: &[&str] = &[
    "libamdhip64.so.7",
    "libamdhip64.so.6",
    "libamdhip64.so.5",
    "libamdhip64.so",
];

/// `hipHostRegister` flags, from `hip_runtime_api.h`.
const HOST_REGISTER_PORTABLE: c_uint = 0x1;
const HOST_REGISTER_MAPPED: c_uint = 0x2;

/// `hipMemcpyKind` values.
const MEMCPY_HOST_TO_DEVICE: c_int = 1;
const MEMCPY_DEVICE_TO_HOST: c_int = 2;

/// `hipDeviceAttribute_t` values used here (ROCm 6.2 `hip_runtime_api.h`).
mod attr {
    pub(super) const CAN_MAP_HOST_MEMORY: i32 = 3;
    pub(super) const CAN_USE_HOST_POINTER_FOR_REGISTERED_MEM: i32 = 4;
    pub(super) const INTEGRATED: i32 = 16;
    pub(super) const VIRTUAL_MEMORY_MANAGEMENT_SUPPORTED: i32 = 89;
    pub(super) const HOST_REGISTER_SUPPORTED: i32 = 90;
}

fn_table! {
    /// The runtime entry points this backend calls.
    HipApi {
        init: fn(c_uint) -> HipError = "hipInit";
        get_device_count: fn(*mut c_int) -> HipError = "hipGetDeviceCount";
        device_get: fn(*mut HipDevice, c_int) -> HipError = "hipDeviceGet";
        device_get_name: fn(*mut c_char, c_int, HipDevice) -> HipError = "hipDeviceGetName";
        device_get_uuid: fn(*mut [u8; 16], HipDevice) -> HipError = "hipDeviceGetUuid";
        device_get_attribute: fn(*mut c_int, c_int, c_int) -> HipError = "hipDeviceGetAttribute";
        get_device: fn(*mut c_int) -> HipError = "hipGetDevice";
        set_device: fn(c_int) -> HipError = "hipSetDevice";
        host_register: fn(*mut c_void, usize, c_uint) -> HipError = "hipHostRegister";
        host_unregister: fn(*mut c_void) -> HipError = "hipHostUnregister";
        host_get_device_pointer: fn(*mut *mut c_void, *mut c_void, c_uint) -> HipError = "hipHostGetDevicePointer";
        memcpy: fn(*mut c_void, *const c_void, usize, c_int) -> HipError = "hipMemcpy";
        stream_synchronize: fn(HipStream) -> HipError = "hipStreamSynchronize";
        get_error_string: fn(HipError) -> *const c_char = "hipGetErrorString";
    }
}

/// One HIP device.
#[derive(Debug)]
pub struct Hip {
    api: HipApi,
    ordinal: c_int,
    caps: Caps,
    // Kept open for the life of the backend: `api` points into it.
    _lib: Library,
}

impl Hip {
    /// Open `libamdhip64` and initialize device `ordinal`.
    pub fn probe(ordinal: i32) -> DeviceResult<Self> {
        let lib = Library::open(BackendKind::Hip, LIBRARIES)?;
        // SAFETY: the rows of `HipApi` match hip_runtime_api.h.
        let api = unsafe { HipApi::load(&lib) }?;
        let check = |call, code| check(&api, call, code);
        // SAFETY: plain runtime calls with live out-pointers.
        unsafe {
            check("hipInit", (api.init)(0))?;
            let mut count = 0;
            check("hipGetDeviceCount", (api.get_device_count)(&mut count))?;
            if ordinal < 0 || ordinal >= count {
                return Err(DeviceError::NoDevice {
                    backend: BackendKind::Hip,
                    detail: format!("ordinal {ordinal} requested, {count} device(s) present"),
                });
            }
            let mut device = 0;
            check("hipDeviceGet", (api.device_get)(&mut device, ordinal))?;
            let mut name = [0 as c_char; 256];
            check(
                "hipDeviceGetName",
                (api.device_get_name)(name.as_mut_ptr(), name.len() as c_int, device),
            )?;
            let mut uuid = [0u8; 16];
            check("hipDeviceGetUuid", (api.device_get_uuid)(&mut uuid, device))?;
            let on = |a| read_attr(&api, ordinal, a);
            let integrated = on(attr::INTEGRATED)?;
            let caps = Caps {
                kind: BackendKind::Hip,
                name: CStr::from_ptr(name.as_ptr()).to_string_lossy().into_owned(),
                uuid: DeviceUuid(uuid),
                integrated,
                host_register: on(attr::HOST_REGISTER_SUPPORTED)? && on(attr::CAN_MAP_HOST_MEMORY)?,
                read_only_register: false,
                host_pointer_is_device_pointer: on(attr::CAN_USE_HOST_POINTER_FOR_REGISTERED_MEM)?,
                posix_fd_export: on(attr::VIRTUAL_MEMORY_MANAGEMENT_SUPPORTED)?,
                // Not probed yet: the DEVICE-mode slice sets this from a
                // real hipIpcGetEventHandle call.
                ipc_event: false,
                mps: None,
            };
            tracing::warn!(
                backend = "hip",
                library = lib.name(),
                device = %caps.name,
                "the HIP backend is built but has not run on AMD hardware; treat results as unvalidated"
            );
            Ok(Self {
                api,
                ordinal,
                caps,
                _lib: lib,
            })
        }
    }

    fn check(&self, call: &'static str, code: HipError) -> DeviceResult<()> {
        check(&self.api, call, code)
    }

    /// Run `f` with this device current on the calling thread, then restore
    /// the thread's previous device.
    fn on_device<T>(&self, f: impl FnOnce() -> DeviceResult<T>) -> DeviceResult<T> {
        let mut previous = 0;
        // SAFETY: plain runtime calls with a live out-pointer.
        unsafe {
            self.check("hipGetDevice", (self.api.get_device)(&mut previous))?;
            self.check("hipSetDevice", (self.api.set_device)(self.ordinal))?;
        }
        let out = f();
        // SAFETY: restores the device read above.
        let restore = self.check("hipSetDevice", unsafe { (self.api.set_device)(previous) });
        match (out, restore) {
            (Ok(out), Ok(())) => Ok(out),
            (Ok(_), Err(restore)) => Err(restore),
            (Err(out), restore) => {
                if let Err(error) = restore {
                    tracing::warn!(%error, "device restore after a failed call also failed");
                }
                Err(out)
            }
        }
    }
}

/// `hipErrorInvalidValue`: what an older runtime returns for an attribute
/// it does not know.
const ERROR_INVALID_VALUE: HipError = 1;

/// Read one boolean device attribute. `false` (logged) when the runtime does
/// not know the attribute; any other failure is an error.
fn read_attr(api: &HipApi, ordinal: c_int, attribute: c_int) -> DeviceResult<bool> {
    let mut v = 0;
    // SAFETY: a live out-pointer and an ordinal below hipGetDeviceCount.
    let code = unsafe { (api.device_get_attribute)(&mut v, attribute, ordinal) };
    if code == ERROR_INVALID_VALUE {
        tracing::warn!(
            backend = "hip",
            attribute,
            "the runtime does not know this attribute; treating it as absent"
        );
        return Ok(false);
    }
    check(api, "hipDeviceGetAttribute", code)?;
    Ok(v != 0)
}

fn check(api: &HipApi, call: &'static str, code: HipError) -> DeviceResult<()> {
    if code == 0 {
        return Ok(());
    }
    // SAFETY: hipGetErrorString returns a static string for any code.
    let text = unsafe { (api.get_error_string)(code) };
    let message = if text.is_null() {
        "unknown".to_string()
    } else {
        // SAFETY: non-null, NUL-terminated, static.
        unsafe { CStr::from_ptr(text) }
            .to_string_lossy()
            .into_owned()
    };
    Err(DeviceError::Driver {
        backend: BackendKind::Hip,
        call,
        code: i64::from(code),
        message,
    })
}

fn flat(address: DeviceAddress) -> DeviceResult<*mut c_void> {
    match address {
        DeviceAddress::Flat(ptr) => Ok(ptr as usize as *mut c_void),
        DeviceAddress::Buffer { .. } => Err(DeviceError::Unsupported {
            backend: BackendKind::Hip,
            what: "a buffer-object device address",
        }),
    }
}

/// The `hipHostRegister` flags for a writable mapping.
const REGISTER_FLAGS: c_uint = HOST_REGISTER_PORTABLE | HOST_REGISTER_MAPPED;

impl DeviceBackend for Hip {
    fn caps(&self) -> &Caps {
        &self.caps
    }

    unsafe fn register_segment(&self, span: &SegmentSpan) -> DeviceResult<SegmentDevice> {
        if !self.caps.host_register {
            return Err(DeviceError::Unsupported {
                backend: BackendKind::Hip,
                what: "host memory registration",
            });
        }
        if span.access == Access::ReadOnly {
            return Err(DeviceError::Unsupported {
                backend: BackendKind::Hip,
                what: "read-only host memory registration",
            });
        }
        let base = span.base as *mut c_void;
        self.on_device(|| {
            // SAFETY: the caller guarantees `span` is a whole live mapping.
            unsafe {
                self.check(
                    "hipHostRegister",
                    (self.api.host_register)(base, span.len, REGISTER_FLAGS),
                )?;
            }
            let mut device: *mut c_void = std::ptr::null_mut();
            // SAFETY: `base` was registered just above.
            let got = unsafe { (self.api.host_get_device_pointer)(&mut device, base, 0) };
            if let Err(e) = self.check("hipHostGetDevicePointer", got) {
                // SAFETY: undoes the registration made above.
                let undo = unsafe { (self.api.host_unregister)(base) };
                if let Err(error) = self.check("hipHostUnregister", undo) {
                    tracing::warn!(%error, "rollback of a half-made registration failed");
                }
                return Err(e);
            }
            Ok(SegmentDevice::Flat {
                base: device as usize as u64,
            })
        })
    }

    unsafe fn unregister_segment(
        &self,
        span: &SegmentSpan,
        _device: SegmentDevice,
    ) -> DeviceResult<()> {
        self.on_device(|| {
            // SAFETY: the caller passes a span this backend registered.
            let code = unsafe { (self.api.host_unregister)(span.base as *mut c_void) };
            self.check("hipHostUnregister", code)
        })
    }

    unsafe fn copy_to_host(&self, src: DeviceAddress, dst: &mut [u8]) -> DeviceResult<()> {
        let src = flat(src)?;
        self.on_device(|| {
            // SAFETY: the caller guarantees `src` addresses dst.len() bytes.
            let code = unsafe {
                (self.api.memcpy)(
                    dst.as_mut_ptr().cast(),
                    src,
                    dst.len(),
                    MEMCPY_DEVICE_TO_HOST,
                )
            };
            self.check("hipMemcpy", code)
        })
    }

    unsafe fn copy_from_host(&self, dst: DeviceAddress, src: &[u8]) -> DeviceResult<()> {
        let dst = flat(dst)?;
        self.on_device(|| {
            // SAFETY: the caller guarantees `dst` addresses src.len()
            // writable bytes.
            let code = unsafe {
                (self.api.memcpy)(dst, src.as_ptr().cast(), src.len(), MEMCPY_HOST_TO_DEVICE)
            };
            self.check("hipMemcpy", code)
        })
    }

    fn fence_record(&self, stream: Stream) -> DeviceResult<Fence> {
        self.on_device(|| {
            // SAFETY: the caller's stream handle, or null for the default.
            let code = unsafe { (self.api.stream_synchronize)(stream.as_raw()) };
            self.check("hipStreamSynchronize", code)
        })?;
        Ok(Fence::CpuComplete)
    }

    fn fence_wait(&self, fence: &Fence, _stream: Stream) -> DeviceResult<()> {
        match fence {
            Fence::CpuComplete => Ok(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_symbol_table_reads_the_hip_runtime_names() {
        assert_eq!(HipApi::SYMBOLS.len(), 14);
        for sym in HipApi::SYMBOLS {
            assert!(sym.starts_with("hip"), "{sym}");
        }
        for sym in [
            "hipHostRegister",
            "hipHostGetDevicePointer",
            "hipHostUnregister",
        ] {
            assert!(HipApi::SYMBOLS.contains(&sym), "{sym}");
        }
    }

    #[test]
    fn register_flags_match_the_hip_header() {
        // hipHostRegisterPortable 0x1 | hipHostRegisterMapped 0x2.
        assert_eq!(REGISTER_FLAGS, 0x3);
    }

    #[test]
    fn attribute_values_match_the_rocm_6_2_header() {
        // hipDeviceAttribute_t in ROCm 6.2 hip_runtime_api.h, values printed
        // by compiling the enum: CanMapHostMemory 3,
        // CanUseHostPointerForRegisteredMem 4, Integrated 16,
        // VirtualMemoryManagementSupported 89, HostRegisterSupported 90.
        assert_eq!(attr::CAN_MAP_HOST_MEMORY, 3);
        assert_eq!(attr::CAN_USE_HOST_POINTER_FOR_REGISTERED_MEM, 4);
        assert_eq!(attr::INTEGRATED, 16);
        assert_eq!(attr::VIRTUAL_MEMORY_MANAGEMENT_SUPPORTED, 89);
        assert_eq!(attr::HOST_REGISTER_SUPPORTED, 90);
    }

    #[test]
    fn hip_reports_itself_as_not_hardware_validated() {
        assert!(!BackendKind::Hip.hardware_validated());
    }
}
