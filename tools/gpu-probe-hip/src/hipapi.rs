//! The probe's own dlopen tables: what `cerulion_device::Hip` does not cover
//! yet (streams, events, IPC, VMM, external memory, modules, hiprtc, the HSA
//! dma-buf export).
//!
//! Every value and layout below was printed by `layout/dump.c` and
//! `layout/dump2.c` compiled against the ROCm 6.2.0 and 7.0.0 HIP headers;
//! both releases gave identical numbers (`layout/rocm-6.2.0.txt`,
//! `layout/rocm-7.0.0.txt`). Each symbol is optional: a missing one disables
//! the modes that need it, and the error names the symbol and the library.

use std::ffi::{c_char, c_int, c_uint, c_void, CStr};

pub type HipError = c_int;
pub type HipStream = *mut c_void;
pub type HipEvent = *mut c_void;
pub type HipModule = *mut c_void;
pub type HipFunction = *mut c_void;
pub type HipMemHandle = *mut c_void; // hipMemGenericAllocationHandle_t
pub type HipExtMem = *mut c_void;
pub type RtcProgram = *mut c_void;

/// Library names tried, in order. The first three are cerulion_device's list
/// in the same order, so both tables resolve to the same loaded file.
pub const HIP_LIBS: &[&str] = &[
    "libamdhip64.so.6",
    "libamdhip64.so.5",
    "libamdhip64.so",
    "libamdhip64.so.7",
    "/opt/rocm/lib/libamdhip64.so",
];
pub const HIPRTC_LIBS: &[&str] = &[
    "libhiprtc.so.6",
    "libhiprtc.so.7",
    "libhiprtc.so",
    "libhiprtc.so.5",
    "/opt/rocm/lib/libhiprtc.so",
];
pub const HSA_LIBS: &[&str] = &["libhsa-runtime64.so.1", "libhsa-runtime64.so", "/opt/rocm/lib/libhsa-runtime64.so.1"];

// hipHostRegister flags.
pub const HOST_REGISTER_PORTABLE: c_uint = 0x1;
pub const HOST_REGISTER_MAPPED: c_uint = 0x2;
/// hipHostRegisterReadOnly. The header says "ignored on AMD devices" and
/// gives hipExtHostRegisterCoarseGrained the same value 0x8; clr 6.2 stores
/// the flags without reading them (it rejects only flags > 15).
pub const HOST_REGISTER_READ_ONLY: c_uint = 0x8;

pub const EVENT_DISABLE_TIMING: c_uint = 0x2;
pub const EVENT_INTERPROCESS: c_uint = 0x4;
/// clr hipIpcOpenMemHandle returns hipErrorInvalidValue for any other flags.
pub const IPC_MEM_LAZY_ENABLE_PEER_ACCESS: c_uint = 0x1;
pub const STREAM_NON_BLOCKING: c_uint = 0x1;
pub const MEMCPY_H2D: c_int = 1;
pub const MEMCPY_D2H: c_int = 2;

pub const MEM_ALLOCATION_TYPE_PINNED: c_int = 1;
pub const MEM_HANDLE_TYPE_POSIX_FD: c_int = 1;
pub const MEM_LOCATION_TYPE_DEVICE: c_int = 1;
pub const MEM_ACCESS_PROT_READWRITE: c_int = 3;
pub const MEM_GRANULARITY_MINIMUM: c_int = 0;
pub const EXT_MEM_HANDLE_OPAQUE_FD: c_int = 1;

/// hipDeviceProp_tR0600: size and the field offsets the probe reads.
pub mod prop {
    pub const SIZE: usize = 1472;
    pub const NAME: usize = 0;
    pub const TOTAL_GLOBAL_MEM: usize = 288;
    pub const WARP_SIZE: usize = 308;
    pub const MULTIPROCESSOR_COUNT: usize = 388;
    pub const PCI_BUS_ID: usize = 584;
    pub const CAN_USE_HOST_POINTER_FOR_REGISTERED_MEM: usize = 684;
    pub const HOST_REGISTER_SUPPORTED: usize = 728;
    pub const HOST_REGISTER_READ_ONLY_SUPPORTED: usize = 736;
    pub const IPC_EVENT_SUPPORTED: usize = 768;
    pub const GCN_ARCH_NAME: usize = 1160;
}

/// hipDeviceAttribute_t values dumped at probe time.
pub const ATTRS: &[(&str, c_int)] = &[
    ("CanMapHostMemory", 3),
    ("CanUseHostPointerForRegisteredMem", 4),
    ("ClockRate", 5),
    ("ConcurrentManagedAccess", 9),
    ("DirectManagedMemAccessFromHost", 13),
    ("HostNativeAtomicSupported", 15),
    ("Integrated", 16),
    ("IsMultiGpuBoard", 17),
    ("L2CacheSize", 19),
    ("ManagedMemory", 24),
    ("MaxThreadsPerBlock", 56),
    ("MemoryBusWidth", 59),
    ("MemoryClockRate", 60),
    ("MultiprocessorCount", 63),
    ("PageableMemoryAccess", 65),
    ("PageableMemoryAccessUsesHostPageTables", 66),
    ("PciBusId", 67),
    ("PciDeviceId", 68),
    ("PciDomainID", 69),
    ("UnifiedAddressing", 85),
    ("WarpSize", 87),
    ("MemoryPoolsSupported", 88),
    ("VirtualMemoryManagementSupported", 89),
    ("HostRegisterSupported", 90),
    ("AsicRevision", 10012),
    ("CanUseStreamWaitValue", 10013),
];
pub const ATTR_MULTIPROCESSOR_COUNT: c_int = 63;
pub const ATTR_WARP_SIZE: c_int = 87;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct IpcHandle {
    pub reserved: [c_char; 64],
}
impl IpcHandle {
    pub fn zeroed() -> Self {
        IpcHandle { reserved: [0; 64] }
    }
    pub fn to_bytes(self) -> [u8; 64] {
        let mut b = [0u8; 64];
        for (d, s) in b.iter_mut().zip(self.reserved.iter()) {
            *d = *s as u8;
        }
        b
    }
    pub fn from_bytes(b: &[u8; 64]) -> Self {
        let mut h = Self::zeroed();
        for (d, s) in h.reserved.iter_mut().zip(b.iter()) {
            *d = *s as c_char;
        }
        h
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct MemLocation {
    pub type_: c_int,
    pub id: c_int,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct AllocFlags {
    pub compression_type: u8,
    pub gpu_direct_rdma_capable: u8,
    pub usage: u16,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct MemAllocationProp {
    pub type_: c_int,
    pub requested_handle_type: c_int,
    pub location: MemLocation,
    pub win32_handle_meta_data: *mut c_void,
    pub alloc_flags: AllocFlags,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct MemAccessDesc {
    pub location: MemLocation,
    pub flags: c_int,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct ExtMemHandleDesc {
    pub type_: c_int,
    pub _pad: c_int,
    /// union { int fd; struct { void* handle; const void* name; } win32; ... }
    pub handle: [u64; 2],
    pub size: u64,
    pub flags: c_uint,
    pub reserved: [c_uint; 16],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct ExtMemBufferDesc {
    pub offset: u64,
    pub size: u64,
    pub flags: c_uint,
    pub reserved: [c_uint; 16],
}

/// Declare a table of optional function pointers and its loader.
macro_rules! table {
    ($name:ident { $( $field:ident : fn($($arg:ty),* $(,)?) -> $ret:ty = $sym:literal; )* }) => {
        #[allow(dead_code)]
        pub struct $name {
            $( pub $field: Option<unsafe extern "C" fn($($arg),*) -> $ret>, )*
            pub lib_name: &'static str,
            _lib: libloading::Library,
        }

        #[allow(dead_code)]
        impl $name {
            pub const SYMBOLS: &'static [&'static str] = &[$($sym),*];

            /// Open the first library of `names` that loads and read every symbol.
            pub fn open(names: &[&'static str]) -> Result<Self, String> {
                let mut last = String::from("no library names given");
                for &n in names {
                    // SAFETY: loading the vendor's shipped runtime library runs its
                    // initializers, the same as linking it would.
                    match unsafe { libloading::Library::new(n) } {
                        Ok(lib) => {
                            $(
                                // SAFETY: the row's signature matches the ROCm header
                                // declaration (see the module docs); the pointer is
                                // used only while `_lib` is held by this struct.
                                let $field = unsafe {
                                    lib.get::<unsafe extern "C" fn($($arg),*) -> $ret>(concat!($sym, "\0").as_bytes())
                                }.ok().map(|s| *s);
                            )*
                            return Ok(Self { $($field,)* lib_name: n, _lib: lib });
                        }
                        Err(e) => last = e.to_string(),
                    }
                }
                Err(format!("could not open any of {:?}: {last}", names))
            }

            /// Symbols the opened library does not export.
            pub fn missing(&self) -> Vec<&'static str> {
                let mut v = vec![];
                $( if self.$field.is_none() { v.push($sym); } )*
                v
            }

            pub fn symbol_of(field: &str) -> &'static str {
                match field { $( stringify!($field) => $sym, )* _ => "?" }
            }
        }
    };
}

table! {
    HipApi {
        init: fn(c_uint) -> HipError = "hipInit";
        get_device_count: fn(*mut c_int) -> HipError = "hipGetDeviceCount";
        set_device: fn(c_int) -> HipError = "hipSetDevice";
        device_get_attribute: fn(*mut c_int, c_int, c_int) -> HipError = "hipDeviceGetAttribute";
        device_get_name: fn(*mut c_char, c_int, c_int) -> HipError = "hipDeviceGetName";
        get_device_properties: fn(*mut c_void, c_int) -> HipError = "hipGetDevicePropertiesR0600";
        runtime_get_version: fn(*mut c_int) -> HipError = "hipRuntimeGetVersion";
        driver_get_version: fn(*mut c_int) -> HipError = "hipDriverGetVersion";
        get_error_name: fn(HipError) -> *const c_char = "hipGetErrorName";
        stream_create_with_flags: fn(*mut HipStream, c_uint) -> HipError = "hipStreamCreateWithFlags";
        stream_synchronize: fn(HipStream) -> HipError = "hipStreamSynchronize";
        stream_wait_event: fn(HipStream, HipEvent, c_uint) -> HipError = "hipStreamWaitEvent";
        malloc: fn(*mut *mut c_void, usize) -> HipError = "hipMalloc";
        free: fn(*mut c_void) -> HipError = "hipFree";
        host_malloc: fn(*mut *mut c_void, usize, c_uint) -> HipError = "hipHostMalloc";
        host_free: fn(*mut c_void) -> HipError = "hipHostFree";
        memcpy_async: fn(*mut c_void, *const c_void, usize, c_int, HipStream) -> HipError = "hipMemcpyAsync";
        memset_async: fn(*mut c_void, c_int, usize, HipStream) -> HipError = "hipMemsetAsync";
        host_register: fn(*mut c_void, usize, c_uint) -> HipError = "hipHostRegister";
        host_unregister: fn(*mut c_void) -> HipError = "hipHostUnregister";
        host_get_device_pointer: fn(*mut *mut c_void, *mut c_void, c_uint) -> HipError = "hipHostGetDevicePointer";
        module_load_data: fn(*mut HipModule, *const c_void) -> HipError = "hipModuleLoadData";
        module_get_function: fn(*mut HipFunction, HipModule, *const c_char) -> HipError = "hipModuleGetFunction";
        module_launch_kernel: fn(HipFunction, c_uint, c_uint, c_uint, c_uint, c_uint, c_uint, c_uint, HipStream, *mut *mut c_void, *mut *mut c_void) -> HipError = "hipModuleLaunchKernel";
        event_create_with_flags: fn(*mut HipEvent, c_uint) -> HipError = "hipEventCreateWithFlags";
        event_record: fn(HipEvent, HipStream) -> HipError = "hipEventRecord";
        event_destroy: fn(HipEvent) -> HipError = "hipEventDestroy";
        ipc_get_event_handle: fn(*mut IpcHandle, HipEvent) -> HipError = "hipIpcGetEventHandle";
        ipc_open_event_handle: fn(*mut HipEvent, IpcHandle) -> HipError = "hipIpcOpenEventHandle";
        ipc_get_mem_handle: fn(*mut IpcHandle, *mut c_void) -> HipError = "hipIpcGetMemHandle";
        ipc_open_mem_handle: fn(*mut *mut c_void, IpcHandle, c_uint) -> HipError = "hipIpcOpenMemHandle";
        ipc_close_mem_handle: fn(*mut c_void) -> HipError = "hipIpcCloseMemHandle";
        mem_get_allocation_granularity: fn(*mut usize, *const MemAllocationProp, c_int) -> HipError = "hipMemGetAllocationGranularity";
        mem_create: fn(*mut HipMemHandle, usize, *const MemAllocationProp, u64) -> HipError = "hipMemCreate";
        mem_export_to_shareable_handle: fn(*mut c_void, HipMemHandle, c_int, u64) -> HipError = "hipMemExportToShareableHandle";
        mem_import_from_shareable_handle: fn(*mut HipMemHandle, *mut c_void, c_int) -> HipError = "hipMemImportFromShareableHandle";
        mem_address_reserve: fn(*mut *mut c_void, usize, usize, *mut c_void, u64) -> HipError = "hipMemAddressReserve";
        mem_map: fn(*mut c_void, usize, usize, HipMemHandle, u64) -> HipError = "hipMemMap";
        mem_set_access: fn(*mut c_void, usize, *const MemAccessDesc, usize) -> HipError = "hipMemSetAccess";
        mem_unmap: fn(*mut c_void, usize) -> HipError = "hipMemUnmap";
        mem_address_free: fn(*mut c_void, usize) -> HipError = "hipMemAddressFree";
        mem_release: fn(HipMemHandle) -> HipError = "hipMemRelease";
        import_external_memory: fn(*mut HipExtMem, *const ExtMemHandleDesc) -> HipError = "hipImportExternalMemory";
        external_memory_get_mapped_buffer: fn(*mut *mut c_void, HipExtMem, *const ExtMemBufferDesc) -> HipError = "hipExternalMemoryGetMappedBuffer";
        destroy_external_memory: fn(HipExtMem) -> HipError = "hipDestroyExternalMemory";
    }
}

table! {
    RtcApi {
        version: fn(*mut c_int, *mut c_int) -> c_int = "hiprtcVersion";
        create_program: fn(*mut RtcProgram, *const c_char, *const c_char, c_int, *const *const c_char, *const *const c_char) -> c_int = "hiprtcCreateProgram";
        compile_program: fn(RtcProgram, c_int, *const *const c_char) -> c_int = "hiprtcCompileProgram";
        get_program_log_size: fn(RtcProgram, *mut usize) -> c_int = "hiprtcGetProgramLogSize";
        get_program_log: fn(RtcProgram, *mut c_char) -> c_int = "hiprtcGetProgramLog";
        get_code_size: fn(RtcProgram, *mut usize) -> c_int = "hiprtcGetCodeSize";
        get_code: fn(RtcProgram, *mut c_char) -> c_int = "hiprtcGetCode";
        destroy_program: fn(*mut RtcProgram) -> c_int = "hiprtcDestroyProgram";
        get_error_string: fn(c_int) -> *const c_char = "hiprtcGetErrorString";
    }
}

table! {
    HsaApi {
        init: fn() -> c_int = "hsa_init";
        portable_export_dmabuf: fn(*const c_void, usize, *mut c_int, *mut u64) -> c_int = "hsa_amd_portable_export_dmabuf";
        portable_close_dmabuf: fn(c_int) -> c_int = "hsa_amd_portable_close_dmabuf";
        status_string: fn(c_int, *mut *const c_char) -> c_int = "hsa_status_string";
    }
}

/// Call a HipApi entry; Ok on hipSuccess, else an error naming the symbol,
/// the code and the runtime's error name.
///
/// Every call site passes pointers that are live for the call, as each
/// site's SAFETY comment states.
#[macro_export]
macro_rules! hip {
    ($api:expr, $f:ident ( $($a:expr),* $(,)? )) => {{
        let api: &$crate::hipapi::HipApi = &$api;
        match api.$f {
            None => Err(format!("{} is not exported by {}", $crate::hipapi::HipApi::symbol_of(stringify!($f)), api.lib_name)),
            Some(f) => {
                #[allow(unused_unsafe)]
                let r = unsafe { f($($a),*) };
                if r == 0 {
                    Ok(())
                } else {
                    Err(format!("{} -> {} ({})", $crate::hipapi::HipApi::symbol_of(stringify!($f)), r, $crate::hipapi::err_name(api, r)))
                }
            }
        }
    }};
}

pub fn err_name(api: &HipApi, code: HipError) -> String {
    match api.get_error_name {
        None => "?".into(),
        Some(f) => {
            // SAFETY: hipGetErrorName returns a static NUL-terminated string for any code.
            let p = unsafe { f(code) };
            if p.is_null() {
                "?".into()
            } else {
                // SAFETY: non-null static C string from the runtime.
                unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned()
            }
        }
    }
}

pub fn rtc_err(api: &RtcApi, code: c_int) -> String {
    match api.get_error_string {
        None => format!("hiprtc code {code}"),
        Some(f) => {
            // SAFETY: hiprtcGetErrorString returns a static string for any code.
            let p = unsafe { f(code) };
            if p.is_null() {
                format!("hiprtc code {code}")
            } else {
                // SAFETY: non-null static C string.
                format!("{code} ({})", unsafe { CStr::from_ptr(p) }.to_string_lossy())
            }
        }
    }
}

pub fn hsa_err(api: &HsaApi, code: c_int) -> String {
    let mut p: *const c_char = std::ptr::null();
    if let Some(f) = api.status_string {
        // SAFETY: live out-pointer; the runtime writes a static string.
        unsafe { f(code, &mut p) };
    }
    if p.is_null() {
        format!("hsa status {code:#x}")
    } else {
        // SAFETY: non-null static C string from the runtime.
        format!("{code:#x} ({})", unsafe { CStr::from_ptr(p) }.to_string_lossy())
    }
}

/// Read a little-endian i32 at `off` in a property buffer.
pub fn prop_i32(buf: &[u8], off: usize) -> i32 {
    i32::from_le_bytes(buf[off..off + 4].try_into().unwrap())
}

/// Read a NUL-terminated string at `off` (at most `max` bytes).
pub fn prop_str(buf: &[u8], off: usize, max: usize) -> String {
    let s = &buf[off..(off + max).min(buf.len())];
    let end = s.iter().position(|&c| c == 0).unwrap_or(s.len());
    String::from_utf8_lossy(&s[..end]).into_owned()
}

/// Decode hipRuntimeGetVersion (major * 10^7 + minor * 10^5 + patch).
pub fn decode_hip_version(v: i32) -> (i32, i32, i32) {
    (v / 10_000_000, (v / 100_000) % 100, v % 100_000)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::{offset_of, size_of};

    #[test]
    fn struct_layouts_match_the_rocm_headers() {
        // From layout/rocm-6.2.0.txt and layout/rocm-7.0.0.txt (identical).
        assert_eq!(size_of::<MemAllocationProp>(), 32);
        assert_eq!(offset_of!(MemAllocationProp, requested_handle_type), 4);
        assert_eq!(offset_of!(MemAllocationProp, location), 8);
        assert_eq!(offset_of!(MemAllocationProp, win32_handle_meta_data), 16);
        assert_eq!(offset_of!(MemAllocationProp, alloc_flags), 24);
        assert_eq!(size_of::<MemLocation>(), 8);
        assert_eq!(size_of::<MemAccessDesc>(), 12);
        assert_eq!(offset_of!(MemAccessDesc, flags), 8);
        assert_eq!(size_of::<ExtMemHandleDesc>(), 104);
        assert_eq!(offset_of!(ExtMemHandleDesc, handle), 8);
        assert_eq!(offset_of!(ExtMemHandleDesc, size), 24);
        assert_eq!(offset_of!(ExtMemHandleDesc, flags), 32);
        assert_eq!(size_of::<ExtMemBufferDesc>(), 88);
        assert_eq!(offset_of!(ExtMemBufferDesc, size), 8);
        assert_eq!(offset_of!(ExtMemBufferDesc, flags), 16);
        assert_eq!(size_of::<IpcHandle>(), 64);
    }

    #[test]
    fn the_layout_dump_files_agree_with_the_constants() {
        for f in ["layout/rocm-6.2.0.txt", "layout/rocm-7.0.0.txt"] {
            let t = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/").to_string() + f).unwrap();
            let get = |k: &str| -> i64 {
                t.lines()
                    .find_map(|l| {
                        let mut it = l.split_whitespace();
                        let (_, key, v) = (it.next()?, it.next()?, it.next()?);
                        (key == k).then(|| v.parse().unwrap())
                    })
                    .unwrap_or_else(|| panic!("{k} missing from {f}"))
            };
            for (name, v) in ATTRS {
                assert_eq!(get(name), i64::from(*v), "{f} attr {name}");
            }
            assert_eq!(get("hipHostRegisterReadOnly"), i64::from(HOST_REGISTER_READ_ONLY));
            assert_eq!(get("hipExtHostRegisterCoarseGrained"), i64::from(HOST_REGISTER_READ_ONLY));
            assert_eq!(get("hipHostRegisterPortable"), i64::from(HOST_REGISTER_PORTABLE));
            assert_eq!(get("hipHostRegisterMapped"), i64::from(HOST_REGISTER_MAPPED));
            assert_eq!(get("hipEventDisableTiming"), i64::from(EVENT_DISABLE_TIMING));
            assert_eq!(get("hipEventInterprocess"), i64::from(EVENT_INTERPROCESS));
            assert_eq!(get("hipIpcMemLazyEnablePeerAccess"), i64::from(IPC_MEM_LAZY_ENABLE_PEER_ACCESS));
            assert_eq!(get("hipStreamNonBlocking"), i64::from(STREAM_NON_BLOCKING));
            assert_eq!(get("hipMemcpyHostToDevice"), i64::from(MEMCPY_H2D));
            assert_eq!(get("hipMemcpyDeviceToHost"), i64::from(MEMCPY_D2H));
            assert_eq!(get("hipMemAllocationTypePinned"), i64::from(MEM_ALLOCATION_TYPE_PINNED));
            assert_eq!(get("hipMemHandleTypePosixFileDescriptor"), i64::from(MEM_HANDLE_TYPE_POSIX_FD));
            assert_eq!(get("hipMemLocationTypeDevice"), i64::from(MEM_LOCATION_TYPE_DEVICE));
            assert_eq!(get("hipMemAccessFlagsProtReadWrite"), i64::from(MEM_ACCESS_PROT_READWRITE));
            assert_eq!(get("hipMemAllocationGranularityMinimum"), i64::from(MEM_GRANULARITY_MINIMUM));
            assert_eq!(get("hipExternalMemoryHandleTypeOpaqueFd"), i64::from(EXT_MEM_HANDLE_OPAQUE_FD));
            assert_eq!(get("hipDeviceProp_tR0600"), prop::SIZE as i64);
            assert_eq!(get("hipDeviceProp_tR0600.totalGlobalMem"), prop::TOTAL_GLOBAL_MEM as i64);
            assert_eq!(get("hipDeviceProp_tR0600.warpSize"), prop::WARP_SIZE as i64);
            assert_eq!(get("hipDeviceProp_tR0600.multiProcessorCount"), prop::MULTIPROCESSOR_COUNT as i64);
            assert_eq!(get("hipDeviceProp_tR0600.pciBusID"), prop::PCI_BUS_ID as i64);
            assert_eq!(get("hipDeviceProp_tR0600.canUseHostPointerForRegisteredMem"), prop::CAN_USE_HOST_POINTER_FOR_REGISTERED_MEM as i64);
            assert_eq!(get("hipDeviceProp_tR0600.hostRegisterSupported"), prop::HOST_REGISTER_SUPPORTED as i64);
            assert_eq!(get("hipDeviceProp_tR0600.hostRegisterReadOnlySupported"), prop::HOST_REGISTER_READ_ONLY_SUPPORTED as i64);
            assert_eq!(get("hipDeviceProp_tR0600.ipcEventSupported"), prop::IPC_EVENT_SUPPORTED as i64);
            assert_eq!(get("hipDeviceProp_tR0600.gcnArchName"), prop::GCN_ARCH_NAME as i64);
            assert_eq!(get("hipDeviceProp_tR0600.name"), prop::NAME as i64);
            assert_eq!(get("hipMemAllocationProp"), size_of::<MemAllocationProp>() as i64);
            assert_eq!(get("hipExternalMemoryHandleDesc"), size_of::<ExtMemHandleDesc>() as i64);
            assert_eq!(get("hipExternalMemoryBufferDesc"), size_of::<ExtMemBufferDesc>() as i64);
            assert_eq!(get("hipIpcMemHandle_t"), 64);
            assert_eq!(get("hipIpcEventHandle_t"), 64);
        }
    }

    #[test]
    fn symbol_tables_name_real_entry_points() {
        assert!(HipApi::SYMBOLS.iter().all(|s| s.starts_with("hip")));
        assert!(RtcApi::SYMBOLS.iter().all(|s| s.starts_with("hiprtc")));
        assert!(HsaApi::SYMBOLS.iter().all(|s| s.starts_with("hsa_")));
        let mut all: Vec<_> = HipApi::SYMBOLS.to_vec();
        all.sort();
        all.dedup();
        assert_eq!(all.len(), HipApi::SYMBOLS.len(), "duplicate symbol");
        for s in ["hipIpcOpenMemHandle", "hipMemImportFromShareableHandle", "hipIpcOpenEventHandle", "hipModuleLaunchKernel", "hipHostRegister"] {
            assert!(HipApi::SYMBOLS.contains(&s), "{s}");
        }
        assert_eq!(HipApi::symbol_of("mem_create"), "hipMemCreate");
        assert_eq!(HipApi::symbol_of("nope"), "?");
        assert_eq!(&HIP_LIBS[..3], &["libamdhip64.so.6", "libamdhip64.so.5", "libamdhip64.so"]);
    }

    #[test]
    fn missing_libraries_report_every_name_tried() {
        let e = HipApi::open(&["libdefinitely-not-here-hip.so", "libnor-this.so"]).err().unwrap();
        assert!(e.contains("libdefinitely-not-here-hip.so") && e.contains("libnor-this.so"), "{e}");
    }

    #[test]
    fn version_and_prop_decoding() {
        assert_eq!(decode_hip_version(60241134), (6, 2, 41134));
        assert_eq!(decode_hip_version(70051831), (7, 0, 51831));
        let mut b = vec![0u8; prop::SIZE];
        b[prop::WARP_SIZE..prop::WARP_SIZE + 4].copy_from_slice(&64i32.to_le_bytes());
        b[prop::GCN_ARCH_NAME..prop::GCN_ARCH_NAME + 6].copy_from_slice(b"gfx942");
        assert_eq!(prop_i32(&b, prop::WARP_SIZE), 64);
        assert_eq!(prop_str(&b, prop::GCN_ARCH_NAME, 256), "gfx942");
        let h = IpcHandle::from_bytes(&[7u8; 64]);
        assert_eq!(h.to_bytes(), [7u8; 64]);
    }
}
