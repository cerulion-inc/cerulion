//! S0 GPU IPC probe for AMD (HIP/ROCm). Scratch code; the HIP twin of
//! tools/gpu-probe (CUDA, Jetson Orin) on branch spike/gpu-s0.
//!
//! Two processes share one iceoryx2 0.10 `[u8]` service per (mode, size).
//! Every frame starts with a 256-byte descriptor at the next 256-byte
//! boundary inside the slot (the in-slot pad; iceoryx2 payload_alignment is
//! never used). The driver process is the publisher and spawns itself as the
//! subscriber.
//!
//! HIP is opened with dlopen at run time: `cerulion_device::Hip` (probe,
//! hipHostRegister, hipHostGetDevicePointer) through `SegmentRegistry` for
//! the UNIFIED path, plus this probe's own table (src/hipapi.rs) for
//! streams, events, IPC, VMM, external memory, modules and hiprtc. The build
//! needs no ROCm SDK; a machine without ROCm gets a clear error and exit 3.
//!
//! Modes (see README.md):
//!   cpu                      memcpy into the slot, CPU verify (no GPU)
//!   copy                     fill kernel -> D2H into the slot -> H2D -> verify kernel
//!   copy-cpuread             fill kernel -> D2H into the slot -> CPU verify
//!   unified-event            slot hipHostRegister'ed in both processes, IPC event gates the reader
//!   unified-cpusync          same, writer syncs before send
//!   unified-cpuread          fill kernel writes the registered slot, CPU verify
//!   device-ipcmem-{event,cpusync}  hipMalloc + hipIpcGetMemHandle / hipIpcOpenMemHandle
//!   device-vmm-{event,cpusync}     hipMemCreate (POSIX fd) exported over SCM_RIGHTS, imported + mapped
//!   device-dmabuf-event      hipMalloc + hsa_amd_portable_export_dmabuf, imported as external memory
//!   floor                    one process: fill + verify on device, registered shm, hipHostMalloc
//!
//! Latency = CLOCK_MONOTONIC from t0 (writer, before the producing kernel or
//! copy) to t1 (reader, after the verify result is on the host). The reader
//! acks each frame (ping-pong).

#[macro_use]
mod hipapi;
mod layout;

use std::ffi::{c_char, c_int, c_void, CString};
use std::io::{Read as _, Write as _};
use std::os::unix::io::{AsRawFd, RawFd};
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::{Arc, OnceLock};
use std::time::Duration;

use cerulion_device::{DeviceAddress, DeviceBackend, SegmentRegistry};
use hipapi::*;
use iceoryx2::prelude::*;
use layout::*;

fn now_ns() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: live out-pointer; CLOCK_MONOTONIC always exists.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

fn emit(r: Row) {
    let mut o = std::io::stdout().lock();
    writeln!(o, "{}", r.line()).unwrap();
    o.flush().unwrap();
}

/// Print a failure row and exit non-zero.
fn fatal(mode: &str, size: usize, msg: &str) -> ! {
    eprintln!("gpu-probe-hip: {mode} size {size}: {msg}");
    emit(Row::new("failed").s("mode", mode).u("size", size as u64).s("reason", msg));
    std::process::exit(5);
}

macro_rules! must {
    ($mode:expr, $size:expr, $e:expr) => {
        match $e {
            Ok(v) => v,
            Err(e) => fatal($mode, $size, &e),
        }
    };
}

// ---------------------------------------------------------------- kernels

/// Same pat() as the CUDA probe. The verify reduction goes through shared
/// memory for a fixed 256-thread block, so it does not depend on the
/// wavefront width (64 on MI300X, 32 on RDNA).
const KERNELS: &str = r#"
__device__ __forceinline__ unsigned int pat(unsigned long long i, unsigned int seed) {
    unsigned int x = ((unsigned int)i * 2654435761u) ^ (seed * 0x9E3779B9u) ^ (unsigned int)(i >> 32);
    x ^= x >> 15; x *= 2246822519u; x ^= x >> 13;
    return x;
}
extern "C" __global__ void fill(unsigned int* p, unsigned long long n, unsigned int seed) {
    unsigned long long stride = (unsigned long long)blockDim.x * gridDim.x;
    for (unsigned long long i = blockIdx.x * (unsigned long long)blockDim.x + threadIdx.x; i < n; i += stride)
        p[i] = pat(i, seed);
}
extern "C" __global__ void verify(const unsigned int* p, unsigned long long n, unsigned int seed,
                                  unsigned long long* out) {
    __shared__ unsigned long long sbad[256];
    __shared__ unsigned long long ssum[256];
    unsigned long long stride = (unsigned long long)blockDim.x * gridDim.x;
    unsigned long long bad = 0, sum = 0;
    for (unsigned long long i = blockIdx.x * (unsigned long long)blockDim.x + threadIdx.x; i < n; i += stride) {
        unsigned int v = p[i];
        bad += (v != pat(i, seed));
        sum += v;
    }
    unsigned int t = threadIdx.x;
    sbad[t] = bad; ssum[t] = sum;
    __syncthreads();
    for (unsigned int o = blockDim.x / 2; o > 0; o >>= 1) {
        if (t < o) { sbad[t] += sbad[t + o]; ssum[t] += ssum[t + o]; }
        __syncthreads();
    }
    if (t == 0) { atomicAdd(&out[0], sbad[0]); atomicAdd(&out[1], ssum[0]); }
}
"#;

fn compile_kernels(key_extra: &str) -> Result<(Vec<u8>, String), String> {
    let opts_env = std::env::var("GPU_S0_HIPRTC_OPTS").unwrap_or_default();
    let mut opts: Vec<String> = vec!["-O3".into()];
    opts.extend(opts_env.split_whitespace().map(String::from));
    let key = fnv1a(format!("{KERNELS}|{key_extra}|{}", opts.join(" ")).as_bytes());
    let cache = std::env::var_os("GPU_S0_KCACHE").map(std::path::PathBuf::from);
    let path = cache.as_ref().map(|d| d.join(format!("gpu-s0-hip-{key:016x}.co")));
    if let Some(p) = &path {
        if let Ok(code) = std::fs::read(p) {
            if !code.is_empty() {
                return Ok((code, format!("cache-hit {}", p.display())));
            }
        }
    }
    let rtc = RtcApi::open(HIPRTC_LIBS).map_err(|e| format!("hiprtc library not found: {e}"))?;
    macro_rules! need {
        ($o:expr, $s:literal) => {
            $o.ok_or_else(|| format!("{} is not exported by {}", $s, rtc.lib_name))
        };
    }
    let create = need!(rtc.create_program, "hiprtcCreateProgram")?;
    let compile = need!(rtc.compile_program, "hiprtcCompileProgram")?;
    let log_size = need!(rtc.get_program_log_size, "hiprtcGetProgramLogSize")?;
    let get_log = need!(rtc.get_program_log, "hiprtcGetProgramLog")?;
    let code_size = need!(rtc.get_code_size, "hiprtcGetCodeSize")?;
    let get_code = need!(rtc.get_code, "hiprtcGetCode")?;
    let destroy = need!(rtc.destroy_program, "hiprtcDestroyProgram")?;
    let src = CString::new(KERNELS).unwrap();
    let name = c"gpu_s0_hip.hip";
    let copts: Vec<CString> = opts.iter().map(|o| CString::new(o.as_str()).unwrap()).collect();
    let optp: Vec<*const c_char> = copts.iter().map(|c| c.as_ptr()).collect();
    let t = now_ns();
    let mut prog: RtcProgram = std::ptr::null_mut();
    // SAFETY: live out-pointer and NUL-terminated strings; no headers.
    let r = unsafe { create(&mut prog, src.as_ptr(), name.as_ptr(), 0, std::ptr::null(), std::ptr::null()) };
    if r != 0 {
        return Err(format!("hiprtcCreateProgram -> {}", rtc_err(&rtc, r)));
    }
    // SAFETY: `prog` was created above; option pointers live in `copts`.
    let r = unsafe { compile(prog, optp.len() as c_int, optp.as_ptr()) };
    let mut n = 0usize;
    // SAFETY: live out-pointer on a created program.
    unsafe { log_size(prog, &mut n) };
    let mut log = vec![0u8; n.max(1)];
    // SAFETY: `log` holds `n` bytes as hiprtc asked.
    unsafe { get_log(prog, log.as_mut_ptr() as *mut c_char) };
    let log = String::from_utf8_lossy(&log).trim_matches(char::from(0)).trim().to_string();
    if r != 0 {
        // SAFETY: destroys the program created above, once.
        unsafe { destroy(&mut prog) };
        return Err(format!("hiprtcCompileProgram -> {}: {log}", rtc_err(&rtc, r)));
    }
    let mut sz = 0usize;
    // SAFETY: live out-pointer on a compiled program.
    unsafe { code_size(prog, &mut sz) };
    let mut code = vec![0u8; sz];
    // SAFETY: `code` holds the size hiprtc reported.
    unsafe { get_code(prog, code.as_mut_ptr() as *mut c_char) };
    // SAFETY: destroys the program created above, once.
    unsafe { destroy(&mut prog) };
    let ms = (now_ns() - t) as f64 / 1e6;
    if let Some(p) = &path {
        let _ = std::fs::create_dir_all(p.parent().unwrap());
        let tmp = p.with_extension(format!("tmp{}", std::process::id()));
        if std::fs::write(&tmp, &code).is_ok() {
            let _ = std::fs::rename(&tmp, p);
        }
    }
    Ok((code, format!("compiled {ms:.0} ms by {} opts [{}]{}", rtc.lib_name, opts.join(" "), if log.is_empty() { String::new() } else { format!(" log: {log}") })))
}

// ---------------------------------------------------------------- GPU

struct Gpu {
    api: HipApi,
    backend: Option<Arc<cerulion_device::Hip>>,
    backend_err: Option<String>,
    hsa: OnceLock<Result<HsaApi, String>>,
    stream: HipStream,
    f_fill: HipFunction,
    f_verify: HipFunction,
    d_out: *mut c_void,
    h_out: *mut u64,
    grid: u32,
    props: Vec<u8>,
    kernel_info: String,
}

fn open_hip() -> Result<(HipApi, Option<Arc<cerulion_device::Hip>>, Option<String>), String> {
    let api = match HipApi::open(HIP_LIBS) {
        Ok(a) => a,
        Err(e) => {
            let dev = match cerulion_device::Hip::probe(0) {
                Ok(_) => "cerulion_device opened it".to_string(),
                Err(d) => d.to_string(),
            };
            return Err(format!("HIP library not found. gpu-probe-hip {e}. cerulion_device: {dev}"));
        }
    };
    let (backend, backend_err) = match cerulion_device::Hip::probe(0) {
        Ok(b) => (Some(Arc::new(b)), None),
        Err(e) => (None, Some(e.to_string())),
    };
    Ok((api, backend, backend_err))
}

impl Gpu {
    fn new() -> Result<Gpu, String> {
        let (api, backend, backend_err) = open_hip()?;
        hip!(api, init(0))?;
        let mut count = 0;
        // SAFETY: live out-pointer.
        hip!(api, get_device_count(&mut count))?;
        if count < 1 {
            return Err("HIP reports no device".into());
        }
        hip!(api, set_device(0))?;
        let mut props = vec![0u8; 4096];
        assert!(props.len() >= prop::SIZE);
        // SAFETY: the buffer is 4096 bytes, larger than hipDeviceProp_tR0600 (1472).
        let _ = hip!(api, get_device_properties(props.as_mut_ptr() as *mut c_void, 0));
        let mut rt = 0;
        // SAFETY: live out-pointer.
        let _ = hip!(api, runtime_get_version(&mut rt));
        let key = format!("{}|{}|{rt}", prop_str(&props, prop::NAME, 256), prop_str(&props, prop::GCN_ARCH_NAME, 256));
        let (code, kernel_info) = compile_kernels(&key)?;
        let mut stream: HipStream = std::ptr::null_mut();
        // SAFETY: live out-pointer.
        hip!(api, stream_create_with_flags(&mut stream, STREAM_NON_BLOCKING))?;
        let mut module: HipModule = std::ptr::null_mut();
        // SAFETY: `code` is a complete code object from hiprtc; it outlives the call.
        hip!(api, module_load_data(&mut module, code.as_ptr() as *const c_void))?;
        let mut f_fill: HipFunction = std::ptr::null_mut();
        let mut f_verify: HipFunction = std::ptr::null_mut();
        // SAFETY: live out-pointers, NUL-terminated names, module loaded above.
        hip!(api, module_get_function(&mut f_fill, module, c"fill".as_ptr()))?;
        // SAFETY: as above.
        hip!(api, module_get_function(&mut f_verify, module, c"verify".as_ptr()))?;
        let mut d_out: *mut c_void = std::ptr::null_mut();
        // SAFETY: live out-pointer.
        hip!(api, malloc(&mut d_out, 16))?;
        let mut h_out: *mut c_void = std::ptr::null_mut();
        // SAFETY: live out-pointer; flags 0 = hipHostMallocDefault.
        hip!(api, host_malloc(&mut h_out, 16, 0))?;
        let mut cus = 0;
        // SAFETY: live out-pointer.
        hip!(api, device_get_attribute(&mut cus, ATTR_MULTIPROCESSOR_COUNT, 0))?;
        Ok(Gpu {
            api,
            backend,
            backend_err,
            hsa: OnceLock::new(),
            stream,
            f_fill,
            f_verify,
            d_out,
            h_out: h_out as *mut u64,
            grid: (cus.max(1) as u32) * 8,
            props,
            kernel_info,
        })
    }

    fn hsa(&self) -> Result<&HsaApi, String> {
        self.hsa
            .get_or_init(|| {
                let h = HsaApi::open(HSA_LIBS)?;
                if let Some(init) = h.init {
                    // SAFETY: hsa_init is reference counted; the HIP runtime already initialized ROCr.
                    let r = unsafe { init() };
                    if r != 0 {
                        return Err(format!("hsa_init -> {}", hsa_err(&h, r)));
                    }
                }
                Ok(h)
            })
            .as_ref()
            .map_err(|e| e.clone())
    }

    fn attr(&self, a: c_int) -> i64 {
        let mut v = -1;
        // SAFETY: live out-pointer.
        match hip!(self.api, device_get_attribute(&mut v, a, 0)) {
            Ok(()) => i64::from(v),
            Err(_) => -1,
        }
    }

    fn grid_for(&self, words: u64) -> u32 {
        (words.div_ceil(u64::from(BLOCK)) as u32).clamp(1, self.grid)
    }

    fn launch(&self, f: HipFunction, words: u64, args: &mut [*mut c_void]) -> Result<(), String> {
        // SAFETY: `args` holds pointers to live locals matching the kernel signature.
        hip!(self.api, module_launch_kernel(f, self.grid_for(words), 1, 1, BLOCK, 1, 1, 0, self.stream, args.as_mut_ptr(), std::ptr::null_mut()))
    }

    fn fill(&self, ptr: u64, words: u64, seed: u32) -> Result<(), String> {
        let (mut p, mut n, mut s) = (ptr, words, seed);
        self.launch(self.f_fill, words, &mut [&mut p as *mut _ as *mut c_void, &mut n as *mut _ as *mut c_void, &mut s as *mut _ as *mut c_void])
    }

    /// Enqueue verify + result copy; the caller syncs, then reads `result()`.
    fn verify(&self, ptr: u64, words: u64, seed: u32) -> Result<(), String> {
        // SAFETY: d_out is a live 16-byte device allocation.
        hip!(self.api, memset_async(self.d_out, 0, 16, self.stream))?;
        let (mut p, mut n, mut s, mut o) = (ptr, words, seed, self.d_out as u64);
        self.launch(
            self.f_verify,
            words,
            &mut [&mut p as *mut _ as *mut c_void, &mut n as *mut _ as *mut c_void, &mut s as *mut _ as *mut c_void, &mut o as *mut _ as *mut c_void],
        )?;
        // SAFETY: h_out is a live 16-byte pinned host buffer, d_out 16 device bytes.
        hip!(self.api, memcpy_async(self.h_out as *mut c_void, self.d_out, 16, MEMCPY_D2H, self.stream))
    }

    fn sync(&self) -> Result<(), String> {
        hip!(self.api, stream_synchronize(self.stream))
    }

    fn result(&self) -> (u64, u64) {
        // SAFETY: h_out holds two u64 written by the synced copy.
        unsafe { (self.h_out.read_volatile(), self.h_out.add(1).read_volatile()) }
    }

    fn malloc(&self, size: usize) -> Result<*mut c_void, String> {
        let mut p: *mut c_void = std::ptr::null_mut();
        // SAFETY: live out-pointer.
        hip!(self.api, malloc(&mut p, size))?;
        Ok(p)
    }
}

// ---------------------------------------------------------------- GPU clock

fn clock_path() -> Option<&'static std::path::Path> {
    static P: OnceLock<Option<std::path::PathBuf>> = OnceLock::new();
    P.get_or_init(|| {
        let mut cards: Vec<_> = std::fs::read_dir("/sys/class/drm").ok()?.filter_map(|e| e.ok()).map(|e| e.path()).collect();
        cards.sort();
        for c in cards {
            let n = c.file_name()?.to_string_lossy().to_string();
            if !n.starts_with("card") || n.contains('-') {
                continue;
            }
            let vendor = std::fs::read_to_string(c.join("device/vendor")).unwrap_or_default();
            let p = c.join("device/pp_dpm_sclk");
            if vendor.trim() == "0x1002" && p.exists() {
                return Some(p);
            }
        }
        None
    })
    .as_deref()
}

/// Current shader clock from amdgpu sysfs (the file rocm-smi reads), 0 if unknown.
fn gpu_mhz() -> u64 {
    clock_path().and_then(|p| std::fs::read_to_string(p).ok()).and_then(|t| parse_dpm_current_mhz(&t)).unwrap_or(0)
}

/// rocm-smi's sclk line(s), outside the timed loop, when GPU_S0_SMI=1.
fn smi_sclk() -> Option<String> {
    if std::env::var("GPU_S0_SMI").ok().as_deref() != Some("1") {
        return None;
    }
    let out = std::process::Command::new("timeout").args(["10", "rocm-smi", "--showclocks"]).output().ok()?;
    let text = String::from_utf8_lossy(&out.stdout);
    let lines: Vec<&str> = text.lines().filter(|l| l.to_ascii_lowercase().contains("sclk")).map(str::trim).collect();
    Some(if lines.is_empty() { "n/a".into() } else { lines.join(" | ") })
}

// ---------------------------------------------------------------- UNIFIED registration

enum RegKind {
    /// cerulion_device::SegmentRegistry (the backend under test).
    Registry(SegmentRegistry),
    /// The probe's own hipHostRegister on the mapping.
    Direct,
    /// Reader only: a second, writable MAP_SHARED mapping of the same file,
    /// registered instead of the read-only one.
    Alias { alias_base: usize },
}

struct Reg {
    kind: RegKind,
    base: usize,
    len: usize,
    dbase: u64,
    flags: u32,
    reg_ns: u64,
    path: &'static str,
    map: String,
    tried: Vec<String>,
}

fn map_desc(addr: usize) -> Option<(usize, usize, String, u64, String)> {
    let maps = std::fs::read_to_string("/proc/self/maps").ok()?;
    maps.lines().filter_map(parse_maps_full).find(|(a, b, ..)| addr >= *a && addr < *b)
}

fn host_register_direct(gpu: &Gpu, base: usize, len: usize, flags: u32) -> Result<u64, String> {
    // SAFETY: [base, base+len) is a whole live mapping of this process.
    hip!(gpu.api, host_register(base as *mut c_void, len, flags))?;
    let mut d: *mut c_void = std::ptr::null_mut();
    // SAFETY: base was registered just above; live out-pointer.
    if let Err(e) = hip!(gpu.api, host_get_device_pointer(&mut d, base as *mut c_void, 0)) {
        // SAFETY: undoes the registration above.
        let _ = hip!(gpu.api, host_unregister(base as *mut c_void));
        return Err(e);
    }
    Ok(d as u64)
}

impl Reg {
    /// Register the mapping holding `addr`. Order: cerulion_device registry,
    /// then direct hipHostRegister (flags Portable|Mapped), then (reader only)
    /// a writable alias of the backing file. GPU_S0_REG=registry|direct|alias
    /// forces one path.
    fn register(gpu: &Gpu, addr: usize, writer: bool) -> Result<Reg, String> {
        let force = std::env::var("GPU_S0_REG").ok();
        let allow = |p: &str| force.as_deref().is_none_or(|f| f == p);
        let span = cerulion_device::span_of(addr).map_err(|e| e.to_string())?;
        let md = map_desc(addr);
        let map = md.as_ref().map(|(_, _, p, _, path)| format!("{p} {path}")).unwrap_or_default();
        let mut tried = vec![];
        if allow("registry") {
            match &gpu.backend {
                None => tried.push(format!("registry: cerulion_device Hip unavailable: {}", gpu.backend_err.clone().unwrap_or_default())),
                Some(b) => {
                    let backend: Arc<dyn DeviceBackend> = b.clone();
                    let reg = SegmentRegistry::new(backend);
                    let t = now_ns();
                    // SAFETY: the mapping is an iceoryx2 segment (or the floor's memfd) that
                    // stays mapped until `release` drops the registry.
                    match unsafe { reg.attach(addr as *const u8) } {
                        Ok(s) => {
                            let reg_ns = now_ns() - t;
                            // cerulion_device's register_flags(): Portable|Mapped, plus ReadOnly for a read-only mapping.
                            let rw = HOST_REGISTER_PORTABLE | HOST_REGISTER_MAPPED;
                            let flags = if s.access == cerulion_device::Access::ReadOnly { rw | HOST_REGISTER_READ_ONLY } else { rw };
                            let dbase = match reg.device_address(s.base as *const u8, 1) {
                                Ok(DeviceAddress::Flat(d)) => d,
                                Ok(other) => return Err(format!("registry returned a non-flat address {other:?}")),
                                Err(e) => return Err(e.to_string()),
                            };
                            return Ok(Reg { kind: RegKind::Registry(reg), base: s.base, len: s.len, dbase, flags, reg_ns, path: "cerulion_device", map, tried });
                        }
                        Err(e) => tried.push(format!("registry: {e}")),
                    }
                }
            }
        }
        if allow("direct") {
            let flags = HOST_REGISTER_PORTABLE | HOST_REGISTER_MAPPED;
            let t = now_ns();
            match host_register_direct(gpu, span.base, span.len, flags) {
                Ok(dbase) => {
                    return Ok(Reg { kind: RegKind::Direct, base: span.base, len: span.len, dbase, flags, reg_ns: now_ns() - t, path: "direct", map, tried });
                }
                Err(e) => tried.push(format!("direct flags={flags:#x}: {e}")),
            }
        }
        if !writer && allow("alias") {
            match Self::alias(gpu, span.len, &md) {
                Ok((alias_base, dbase, reg_ns)) => {
                    let flags = HOST_REGISTER_PORTABLE | HOST_REGISTER_MAPPED;
                    return Ok(Reg { kind: RegKind::Alias { alias_base }, base: span.base, len: span.len, dbase, flags, reg_ns, path: "rw-alias", map, tried });
                }
                Err(e) => tried.push(format!("rw-alias: {e}")),
            }
        }
        Err(format!("register {} bytes at {:#x} ({map}) failed: {}", span.len, span.base, tried.join("; ")))
    }

    fn alias(gpu: &Gpu, len: usize, md: &Option<(usize, usize, String, u64, String)>) -> Result<(usize, u64, u64), String> {
        let Some((_, _, _, off, path)) = md else { return Err("no /proc/self/maps entry (Linux only)".into()) };
        if path.is_empty() {
            return Err("mapping has no backing file".into());
        }
        let cpath = CString::new(path.as_str()).unwrap();
        // SAFETY: NUL-terminated path; the fd is closed below.
        let fd = unsafe { libc::open(cpath.as_ptr(), libc::O_RDWR) };
        if fd < 0 {
            return Err(format!("open {path} O_RDWR: {}", std::io::Error::last_os_error()));
        }
        // SAFETY: a new shared mapping of the same file range the read-only mapping shows.
        let p = unsafe { libc::mmap(std::ptr::null_mut(), len, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, fd, *off as libc::off_t) };
        // SAFETY: closing the fd opened above; the mapping keeps the file.
        unsafe { libc::close(fd) };
        if p == libc::MAP_FAILED {
            return Err(format!("mmap rw alias: {}", std::io::Error::last_os_error()));
        }
        let t = now_ns();
        match host_register_direct(gpu, p as usize, len, HOST_REGISTER_PORTABLE | HOST_REGISTER_MAPPED) {
            Ok(d) => Ok((p as usize, d, now_ns() - t)),
            Err(e) => {
                // SAFETY: unmapping the alias created above.
                unsafe { libc::munmap(p, len) };
                Err(e)
            }
        }
    }

    /// Device address of [host, host+len) for the kernel.
    fn dptr(&self, host: usize, len: usize, write: bool) -> Result<u64, String> {
        match &self.kind {
            RegKind::Registry(r) => {
                let a = if write { r.device_address_mut(host as *mut u8, len) } else { r.device_address(host as *const u8, len) };
                match a.map_err(|e| e.to_string())? {
                    DeviceAddress::Flat(d) => Ok(d),
                    other => Err(format!("non-flat address {other:?}")),
                }
            }
            _ => {
                if host < self.base || host + len > self.base + self.len {
                    return Err(format!("{host:#x}+{len} outside the registered segment"));
                }
                Ok(self.dbase + (host - self.base) as u64)
            }
        }
    }

    fn release(self, gpu: &Gpu) {
        match self.kind {
            RegKind::Registry(r) => drop(r),
            RegKind::Direct => {
                // SAFETY: base was registered by register().
                let _ = hip!(gpu.api, host_unregister(self.base as *mut c_void));
            }
            RegKind::Alias { alias_base } => {
                // SAFETY: alias_base was registered and mapped by alias().
                let _ = hip!(gpu.api, host_unregister(alias_base as *mut c_void));
                // SAFETY: unmapping the alias mapping created by alias().
                unsafe { libc::munmap(alias_base as *mut c_void, self.len) };
            }
        }
    }

    fn row(&self, side: &str, mode: &str, size: usize) -> Row {
        Row::new("onetime")
            .s("side", side)
            .s("mode", mode)
            .u("size", size as u64)
            .u("segment", self.len as u64)
            .f("register_us", self.reg_ns as f64 / 1e3)
            .f("register_us_per_mib", self.reg_ns as f64 / 1e3 / (self.len as f64 / (1 << 20) as f64))
            .u("flags", u64::from(self.flags))
            .s("reg_path", self.path)
            .s("map", &self.map)
            .b("dptr_eq_host", self.dbase as usize == self.base)
            .s("reg_tried", &self.tried.join("; "))
    }
}

/// CPU read bandwidth in GB/s (best of 3) over [ptr, ptr+len).
fn cpu_read_gbps(ptr: *const u8, len: usize) -> f64 {
    let words = len / 8;
    let p = ptr as *const u64;
    let mut best = f64::MAX;
    let mut acc = 0u64;
    for _ in 0..3 {
        let t = now_ns();
        let mut s = [0u64; 4];
        let mut i = 0;
        while i + 4 <= words {
            // SAFETY: i + 3 < words, inside [ptr, ptr+len).
            unsafe {
                s[0] = s[0].wrapping_add(p.add(i).read_volatile());
                s[1] = s[1].wrapping_add(p.add(i + 1).read_volatile());
                s[2] = s[2].wrapping_add(p.add(i + 2).read_volatile());
                s[3] = s[3].wrapping_add(p.add(i + 3).read_volatile());
            }
            i += 4;
        }
        acc = acc.wrapping_add(s[0] ^ s[1] ^ s[2] ^ s[3]);
        best = best.min((now_ns() - t) as f64);
    }
    std::hint::black_box(acc);
    len as f64 / best
}

// ---------------------------------------------------------------- events

fn make_ipc_event(gpu: &Gpu) -> Result<(HipEvent, [u8; 64]), String> {
    let mut ev: HipEvent = std::ptr::null_mut();
    // SAFETY: live out-pointer.
    hip!(gpu.api, event_create_with_flags(&mut ev, EVENT_DISABLE_TIMING | EVENT_INTERPROCESS))?;
    let mut h = IpcHandle::zeroed();
    // SAFETY: live out-pointer; ev created above.
    hip!(gpu.api, ipc_get_event_handle(&mut h, ev))?;
    Ok((ev, h.to_bytes()))
}

fn open_ipc_event(gpu: &Gpu, handle: &[u8; 64]) -> Result<HipEvent, String> {
    let mut ev: HipEvent = std::ptr::null_mut();
    // SAFETY: live out-pointer; the handle is passed by value.
    hip!(gpu.api, ipc_open_event_handle(&mut ev, IpcHandle::from_bytes(handle)))?;
    Ok(ev)
}

// ---------------------------------------------------------------- VMM / IPC mem / dma-buf

fn vmm_prop() -> MemAllocationProp {
    MemAllocationProp {
        type_: MEM_ALLOCATION_TYPE_PINNED,
        requested_handle_type: MEM_HANDLE_TYPE_POSIX_FD,
        location: MemLocation { type_: MEM_LOCATION_TYPE_DEVICE, id: 0 },
        win32_handle_meta_data: std::ptr::null_mut(),
        alloc_flags: AllocFlags { compression_type: 0, gpu_direct_rdma_capable: 0, usage: 0 },
    }
}

fn vmm_granularity(gpu: &Gpu) -> Result<usize, String> {
    let prop = vmm_prop();
    let mut g = 0usize;
    // SAFETY: live out-pointer and prop.
    hip!(gpu.api, mem_get_allocation_granularity(&mut g, &prop, MEM_GRANULARITY_MINIMUM))?;
    Ok(g)
}

fn vmm_map(gpu: &Gpu, h: HipMemHandle, size: usize) -> Result<*mut c_void, String> {
    let mut ptr: *mut c_void = std::ptr::null_mut();
    // SAFETY: live out-pointer; no fixed address requested.
    hip!(gpu.api, mem_address_reserve(&mut ptr, size, 0, std::ptr::null_mut(), 0))?;
    // SAFETY: ptr is a fresh reservation of `size` bytes; h is a live allocation handle.
    hip!(gpu.api, mem_map(ptr, size, 0, h, 0))?;
    let acc = MemAccessDesc { location: MemLocation { type_: MEM_LOCATION_TYPE_DEVICE, id: 0 }, flags: MEM_ACCESS_PROT_READWRITE };
    // SAFETY: ptr..ptr+size is mapped above; one descriptor.
    hip!(gpu.api, mem_set_access(ptr, size, &acc, 1))?;
    Ok(ptr)
}

fn vmm_unmap(gpu: &Gpu, ptr: *mut c_void, size: usize, h: HipMemHandle) {
    // SAFETY: tearing down a mapping made by vmm_map, once.
    let _ = hip!(gpu.api, mem_unmap(ptr, size));
    // SAFETY: as above.
    let _ = hip!(gpu.api, mem_address_free(ptr, size));
    // SAFETY: releasing the handle this process created or imported, once.
    let _ = hip!(gpu.api, mem_release(h));
}

fn vmm_import(gpu: &Gpu, fd: RawFd) -> Result<HipMemHandle, String> {
    let arg = DualFdArg::new(fd)?;
    if arg.by_value() != fd || arg.by_pointer() != fd {
        return Err(format!("dual fd handle self-check failed for fd {fd}"));
    }
    let mut h: HipMemHandle = std::ptr::null_mut();
    // SAFETY: `arg` reads as `fd` under both the by-pointer (ROCm <= 7.0.2) and the
    // by-value (clr develop) conventions; it lives for the call.
    hip!(gpu.api, mem_import_from_shareable_handle(&mut h, arg.ptr(), MEM_HANDLE_TYPE_POSIX_FD))?;
    Ok(h)
}

fn round_up(v: usize, g: usize) -> usize {
    v.div_ceil(g) * g
}

fn send_fd(sock: &UnixStream, fd: Option<RawFd>, data: &[u8]) -> Result<(), String> {
    // SAFETY: msghdr/iovec/cmsg buffers are locals that outlive the sendmsg call.
    unsafe {
        let mut iov = libc::iovec { iov_base: data.as_ptr() as *mut c_void, iov_len: data.len() };
        let space = libc::CMSG_SPACE(std::mem::size_of::<RawFd>() as u32) as usize;
        let mut cbuf = vec![0u8; space];
        let mut msg: libc::msghdr = std::mem::zeroed();
        msg.msg_iov = &mut iov;
        msg.msg_iovlen = 1;
        if let Some(fd) = fd {
            msg.msg_control = cbuf.as_mut_ptr() as *mut c_void;
            msg.msg_controllen = space as _;
            let c = libc::CMSG_FIRSTHDR(&msg);
            (*c).cmsg_level = libc::SOL_SOCKET;
            (*c).cmsg_type = libc::SCM_RIGHTS;
            (*c).cmsg_len = libc::CMSG_LEN(std::mem::size_of::<RawFd>() as u32) as _;
            std::ptr::write_unaligned(libc::CMSG_DATA(c) as *mut RawFd, fd);
        }
        let n = libc::sendmsg(sock.as_raw_fd(), &msg, 0);
        if n as usize != data.len() {
            return Err(format!("sendmsg: {}", std::io::Error::last_os_error()));
        }
    }
    Ok(())
}

fn recv_fd(sock: &UnixStream, data: &mut [u8]) -> Result<Option<RawFd>, String> {
    // SAFETY: msghdr/iovec/cmsg buffers are locals that outlive the recvmsg call.
    unsafe {
        let mut iov = libc::iovec { iov_base: data.as_mut_ptr() as *mut c_void, iov_len: data.len() };
        let space = libc::CMSG_SPACE(std::mem::size_of::<RawFd>() as u32) as usize;
        let mut cbuf = vec![0u8; space];
        let mut msg: libc::msghdr = std::mem::zeroed();
        msg.msg_iov = &mut iov;
        msg.msg_iovlen = 1;
        msg.msg_control = cbuf.as_mut_ptr() as *mut c_void;
        msg.msg_controllen = space as _;
        let n = libc::recvmsg(sock.as_raw_fd(), &mut msg, libc::MSG_WAITALL);
        if n as usize != data.len() {
            return Err(format!("recvmsg got {n} of {} bytes: {}", data.len(), std::io::Error::last_os_error()));
        }
        let c = libc::CMSG_FIRSTHDR(&msg);
        if c.is_null() || (*c).cmsg_type != libc::SCM_RIGHTS {
            return Ok(None);
        }
        Ok(Some(std::ptr::read_unaligned(libc::CMSG_DATA(c) as *const RawFd)))
    }
}

/// Writer-side device memory for one size.
enum WDev {
    Vmm { ptr: *mut c_void, bsize: usize, h: HipMemHandle },
    Ipc { ptr: *mut c_void },
    Dmabuf { ptr: *mut c_void },
}
impl WDev {
    fn ptr(&self) -> u64 {
        match self {
            WDev::Vmm { ptr, .. } | WDev::Ipc { ptr } | WDev::Dmabuf { ptr } => *ptr as u64,
        }
    }
    fn release(self, gpu: &Gpu) {
        match self {
            WDev::Vmm { ptr, bsize, h } => vmm_unmap(gpu, ptr, bsize, h),
            WDev::Ipc { ptr } | WDev::Dmabuf { ptr } => {
                // SAFETY: freeing this process's own hipMalloc allocation, once.
                let _ = hip!(gpu.api, free(ptr));
            }
        }
    }
}

/// Reader-side imported memory for one size.
enum RDev {
    Vmm { ptr: *mut c_void, bsize: usize, h: HipMemHandle },
    Ipc { ptr: *mut c_void },
    Ext { ptr: *mut c_void, ext: HipExtMem },
    DmabufVmm { base: *mut c_void, dsize: usize, h: HipMemHandle, off: u64 },
}
impl RDev {
    fn ptr(&self) -> u64 {
        match self {
            RDev::Vmm { ptr, .. } | RDev::Ipc { ptr } | RDev::Ext { ptr, .. } => *ptr as u64,
            RDev::DmabufVmm { base, off, .. } => *base as u64 + off,
        }
    }
    fn release(self, gpu: &Gpu) {
        match self {
            RDev::Vmm { ptr, bsize, h } => vmm_unmap(gpu, ptr, bsize, h),
            RDev::DmabufVmm { base, dsize, h, .. } => vmm_unmap(gpu, base, dsize, h),
            RDev::Ipc { ptr } => {
                // SAFETY: closing a handle opened by this process, once.
                let _ = hip!(gpu.api, ipc_close_mem_handle(ptr));
            }
            RDev::Ext { ext, .. } => {
                // SAFETY: destroying external memory imported by this process, once.
                let _ = hip!(gpu.api, destroy_external_memory(ext));
            }
        }
    }
}

/// Writer: allocate + export, send the handle, read the reader's status.
/// Ok(None) = skipped (a row explains why).
fn writer_device_setup(gpu: &Gpu, mode: Mode, size: usize, sk: &UnixStream) -> Result<Option<WDev>, String> {
    let mut b = [0u8; 8];
    (&*sk).read_exact(&mut b[..1]).map_err(|e| format!("reader ready byte: {e}"))?;
    let mut msg = DevMsg::zeroed();
    msg.size = size as u64;
    let mut row = Row::new("onetime").s("side", "writer").s("mode", mode.name()).u("size", size as u64);
    let mut fd_to_send: Option<RawFd> = None;
    let mut close_dmabuf = false;
    let setup: Result<WDev, String> = (|| -> Result<WDev, String> { match mode.dev_kind() {
        DevKind::Vmm => {
            let gran = vmm_granularity(gpu)?;
            let bsize = round_up(size, gran);
            let prop = vmm_prop();
            let t = now_ns();
            let mut h: HipMemHandle = std::ptr::null_mut();
            // SAFETY: live out-pointer and prop.
            hip!(gpu.api, mem_create(&mut h, bsize, &prop, 0))?;
            let ptr = vmm_map(gpu, h, bsize)?;
            let t_create = now_ns() - t;
            let t = now_ns();
            let mut fd: c_int = -1;
            // SAFETY: the runtime writes one int (the fd) through the out-pointer.
            hip!(gpu.api, mem_export_to_shareable_handle(&mut fd as *mut c_int as *mut c_void, h, MEM_HANDLE_TYPE_POSIX_FD, 0))?;
            let t_export = now_ns() - t;
            row = std::mem::replace(&mut row, Row::new("x")).u("block", bsize as u64).u("gran", gran as u64).f("create_map_us", t_create as f64 / 1e3).f("export_us", t_export as f64 / 1e3);
            msg.kind = 2;
            msg.bsize = bsize as u64;
            msg.has_fd = 1;
            fd_to_send = Some(fd);
            Ok(WDev::Vmm { ptr, bsize, h })
        }
        DevKind::Ipc => {
            let t = now_ns();
            let ptr = gpu.malloc(size)?;
            let t_alloc = now_ns() - t;
            let t = now_ns();
            let mut h = IpcHandle::zeroed();
            // SAFETY: live out-pointer; ptr is a live hipMalloc allocation.
            hip!(gpu.api, ipc_get_mem_handle(&mut h, ptr))?;
            let t_get = now_ns() - t;
            row = std::mem::replace(&mut row, Row::new("x")).u("block", size as u64).f("malloc_us", t_alloc as f64 / 1e3).f("ipc_get_mem_handle_us", t_get as f64 / 1e3);
            msg.kind = 1;
            msg.bsize = size as u64;
            msg.ipc = h.to_bytes();
            Ok(WDev::Ipc { ptr })
        }
        DevKind::Dmabuf => {
            let hsa = gpu.hsa()?;
            let export = hsa.portable_export_dmabuf.ok_or("hsa_amd_portable_export_dmabuf is not exported")?;
            let bsize = round_up(size, 2 << 20);
            let ptr = gpu.malloc(bsize)?;
            let t = now_ns();
            let (mut fd, mut off): (c_int, u64) = (-1, 0);
            // SAFETY: ptr..ptr+bsize is a live device allocation; live out-pointers.
            let r = unsafe { export(ptr, bsize, &mut fd, &mut off) };
            if r != 0 {
                // SAFETY: freeing the allocation made above.
                let _ = hip!(gpu.api, free(ptr));
                return Err(format!("hsa_amd_portable_export_dmabuf -> {}", hsa_err(hsa, r)));
            }
            let t_export = now_ns() - t;
            row = std::mem::replace(&mut row, Row::new("x")).u("block", bsize as u64).f("dmabuf_export_us", t_export as f64 / 1e3).u("dmabuf_offset", off);
            msg.kind = 3;
            msg.bsize = bsize as u64;
            msg.offset = off;
            msg.has_fd = 1;
            fd_to_send = Some(fd);
            close_dmabuf = true;
            Ok(WDev::Dmabuf { ptr })
        }
    }})();
    let dev = match setup {
        Ok(d) => d,
        Err(e) => {
            msg.kind = 0;
            send_fd(sk, None, as_bytes(&msg))?;
            emit(Row::new("skip").s("side", "writer").s("mode", mode.name()).u("size", size as u64).s("reason", &e));
            return Ok(None);
        }
    };
    let t = now_ns();
    msg.t_send_ns = t;
    send_fd(sk, fd_to_send, as_bytes(&msg))?;
    (&*sk).read_exact(&mut b).map_err(|e| format!("reader status: {e}"))?;
    let rtt = now_ns() - t;
    if let Some(fd) = fd_to_send {
        if close_dmabuf {
            if let Some(close) = gpu.hsa()?.portable_close_dmabuf {
                // SAFETY: closing the dma-buf fd exported above, once.
                unsafe { close(fd) };
            }
        } else {
            // SAFETY: closing the exported fd; the reader holds its own copy.
            unsafe { libc::close(fd) };
        }
    }
    let status = u64::from_ne_bytes(b);
    if status != 0 {
        emit(Row::new("skip").s("side", "writer").s("mode", mode.name()).u("size", size as u64).s("reason", "reader could not import (see the reader's skip row)"));
        dev.release(gpu);
        return Ok(None);
    }
    emit(row.f("handle_rtt_us", rtt as f64 / 1e3));
    Ok(Some(dev))
}

/// Reader: receive the handle, import, map, send status. Ok(None) = skipped.
fn reader_device_setup(gpu: &Gpu, mode: Mode, size: usize, sk: &UnixStream) -> Result<Option<RDev>, String> {
    (&*sk).write_all(b"R").map_err(|e| e.to_string())?;
    let mut msg = DevMsg::zeroed();
    // SAFETY: DevMsg is plain data; the slice covers exactly its bytes.
    let fd = recv_fd(sk, unsafe { std::slice::from_raw_parts_mut(&mut msg as *mut DevMsg as *mut u8, std::mem::size_of::<DevMsg>()) })?;
    let t_recv = now_ns();
    if msg.kind == 0 {
        return Ok(None);
    }
    let mut row = Row::new("onetime").s("side", "reader").s("mode", mode.name()).u("size", size as u64).u("block", msg.bsize).f("handle_one_way_us", (t_recv.saturating_sub(msg.t_send_ns)) as f64 / 1e3);
    let res: Result<RDev, String> = (|| -> Result<RDev, String> { match msg.kind {
        2 => {
            let fd = fd.ok_or("no fd arrived with the VMM handle")?;
            let t = now_ns();
            let h = vmm_import(gpu, fd);
            let t_import = now_ns() - t;
            // SAFETY: closing the received fd; the import holds its own reference.
            unsafe { libc::close(fd) };
            let h = h?;
            let t = now_ns();
            let ptr = vmm_map(gpu, h, msg.bsize as usize)?;
            row = std::mem::replace(&mut row, Row::new("x")).f("import_us", t_import as f64 / 1e3).f("map_access_us", (now_ns() - t) as f64 / 1e3).s("import_arg", "dual (pointer and value)");
            Ok(RDev::Vmm { ptr, bsize: msg.bsize as usize, h })
        }
        1 => {
            let mut ptr: *mut c_void = std::ptr::null_mut();
            let t = now_ns();
            // SAFETY: live out-pointer; handle passed by value; the only flag clr accepts.
            hip!(gpu.api, ipc_open_mem_handle(&mut ptr, IpcHandle::from_bytes(&msg.ipc), IPC_MEM_LAZY_ENABLE_PEER_ACCESS))?;
            row = std::mem::replace(&mut row, Row::new("x")).f("ipc_open_mem_handle_us", (now_ns() - t) as f64 / 1e3);
            Ok(RDev::Ipc { ptr })
        }
        3 => {
            let fd = fd.ok_or("no fd arrived with the dma-buf")?;
            // SAFETY: lseek on a dma-buf fd reports its size (SEEK_END) and rewinds.
            let end = unsafe { libc::lseek(fd, 0, libc::SEEK_END) };
            // SAFETY: as above.
            unsafe { libc::lseek(fd, 0, libc::SEEK_SET) };
            let dsize = if end > 0 { end as u64 } else { msg.offset + msg.bsize };
            let mut errs = vec![];
            // Path 1: external memory (opaque fd; ROCr imports it as a dma-buf).
            // SAFETY: dup of a live fd; the import may take ownership of the copy.
            let dupfd = unsafe { libc::dup(fd) };
            let desc = ExtMemHandleDesc { type_: EXT_MEM_HANDLE_OPAQUE_FD, _pad: 0, handle: [dupfd as u32 as u64, 0], size: dsize, flags: 0, reserved: [0; 16] };
            let mut ext: HipExtMem = std::ptr::null_mut();
            let t = now_ns();
            // SAFETY: live descriptor and out-pointer.
            match hip!(gpu.api, import_external_memory(&mut ext, &desc)) {
                Ok(()) => {
                    let t_import = now_ns() - t;
                    let bd = ExtMemBufferDesc { offset: msg.offset, size: msg.bsize, flags: 0, reserved: [0; 16] };
                    let mut ptr: *mut c_void = std::ptr::null_mut();
                    let t = now_ns();
                    // SAFETY: ext imported above; live descriptor and out-pointer.
                    match hip!(gpu.api, external_memory_get_mapped_buffer(&mut ptr, ext, &bd)) {
                        Ok(()) => {
                            // SAFETY: closing the original fd; the import holds the dup.
                            unsafe { libc::close(fd) };
                            row = std::mem::replace(&mut row, Row::new("x")).s("dmabuf_import", "external-memory").u("dmabuf_size", dsize).f("import_us", t_import as f64 / 1e3).f("map_access_us", (now_ns() - t) as f64 / 1e3);
                            return Ok(RDev::Ext { ptr, ext });
                        }
                        Err(e) => {
                            errs.push(format!("external memory map: {e}"));
                            // SAFETY: destroying the import made above.
                            let _ = hip!(gpu.api, destroy_external_memory(ext));
                        }
                    }
                }
                Err(e) => {
                    errs.push(format!("external memory import: {e}"));
                    // SAFETY: the failed import did not take the dup.
                    unsafe { libc::close(dupfd) };
                }
            }
            // Path 2: the VMM import, which takes a dma-buf fd in ROCr.
            let t = now_ns();
            let h = vmm_import(gpu, fd);
            let t_import = now_ns() - t;
            // SAFETY: closing the received fd after the last import attempt.
            unsafe { libc::close(fd) };
            match h.and_then(|h| vmm_map(gpu, h, dsize as usize).map(|p| (h, p))) {
                Ok((h, base)) => {
                    row = std::mem::replace(&mut row, Row::new("x")).s("dmabuf_import", "vmm-import").u("dmabuf_size", dsize).f("import_us", t_import as f64 / 1e3).s("dmabuf_tried", &errs.join("; "));
                    Ok(RDev::DmabufVmm { base, dsize: dsize as usize, h, off: msg.offset })
                }
                Err(e) => {
                    errs.push(format!("vmm import: {e}"));
                    Err(errs.join("; "))
                }
            }
        }
        k => Err(format!("unknown handshake kind {k}")),
    }})();
    match res {
        Ok(d) => {
            (&*sk).write_all(&0u64.to_ne_bytes()).map_err(|e| e.to_string())?;
            emit(row);
            Ok(Some(d))
        }
        Err(e) => {
            emit(Row::new("skip").s("side", "reader").s("mode", mode.name()).u("size", size as u64).s("reason", &e));
            (&*sk).write_all(&1u64.to_ne_bytes()).map_err(|e| e.to_string())?;
            Ok(None)
        }
    }
}

// ---------------------------------------------------------------- iceoryx2 plumbing

type Pub = iceoryx2::port::publisher::Publisher<ipc::Service, [u8], ()>;
type Sub = iceoryx2::port::subscriber::Subscriber<ipc::Service, [u8], ()>;

fn service(node: &Node<ipc::Service>, name: &str) -> iceoryx2::service::port_factory::publish_subscribe::PortFactory<ipc::Service, [u8], ()> {
    let sn = ServiceName::new(name).unwrap();
    for _ in 0..500 {
        // Natural [u8] alignment: payload_alignment(256) pads every element 256x in 0.10.
        match node
            .service_builder(&sn)
            .publish_subscribe::<[u8]>()
            .max_publishers(1)
            .max_subscribers(1)
            .history_size(0)
            .subscriber_max_buffer_size(1)
            .open_or_create()
        {
            Ok(s) => return s,
            Err(_) => std::thread::sleep(Duration::from_millis(10)),
        }
    }
    panic!("open_or_create {name}");
}

fn make_pub(node: &Node<ipc::Service>, name: &str, max_len: usize) -> Pub {
    service(node, name).publisher_builder().initial_max_slice_len(max_len).allocation_strategy(AllocationStrategy::Static).create().expect("publisher")
}

fn make_sub(node: &Node<ipc::Service>, name: &str) -> Sub {
    service(node, name).subscriber_builder().buffer_size(1).create().expect("subscriber")
}

fn send_bytes(p: &Pub, bytes: &[u8]) {
    let s = p.loan_slice_uninit(bytes.len()).expect("loan");
    let s = s.write_from_slice(bytes);
    s.send().expect("send");
}

// ---------------------------------------------------------------- modes

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum DevKind {
    Ipc,
    Vmm,
    Dmabuf,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mode {
    Cpu = 0,
    Copy = 1,
    UnifiedEvent = 2,
    UnifiedCpuSync = 3,
    UnifiedCpuRead = 6,
    CopyCpuRead = 7,
    IpcMemEvent = 8,
    IpcMemCpuSync = 9,
    VmmEvent = 10,
    VmmCpuSync = 11,
    DmabufEvent = 12,
}

const ALL_MODES: &[Mode] = &[
    Mode::Cpu,
    Mode::Copy,
    Mode::CopyCpuRead,
    Mode::UnifiedCpuRead,
    Mode::UnifiedCpuSync,
    Mode::UnifiedEvent,
    Mode::IpcMemCpuSync,
    Mode::IpcMemEvent,
    Mode::VmmCpuSync,
    Mode::VmmEvent,
    Mode::DmabufEvent,
];

impl Mode {
    fn parse(s: &str) -> Option<Mode> {
        ALL_MODES.iter().copied().find(|m| m.name() == s)
    }
    fn name(self) -> &'static str {
        match self {
            Mode::Cpu => "cpu",
            Mode::Copy => "copy",
            Mode::CopyCpuRead => "copy-cpuread",
            Mode::UnifiedEvent => "unified-event",
            Mode::UnifiedCpuSync => "unified-cpusync",
            Mode::UnifiedCpuRead => "unified-cpuread",
            Mode::IpcMemEvent => "device-ipcmem-event",
            Mode::IpcMemCpuSync => "device-ipcmem-cpusync",
            Mode::VmmEvent => "device-vmm-event",
            Mode::VmmCpuSync => "device-vmm-cpusync",
            Mode::DmabufEvent => "device-dmabuf-event",
        }
    }
    fn unified(self) -> bool {
        matches!(self, Mode::UnifiedEvent | Mode::UnifiedCpuSync | Mode::UnifiedCpuRead)
    }
    fn reader_registers(self) -> bool {
        matches!(self, Mode::UnifiedEvent | Mode::UnifiedCpuSync)
    }
    fn reader_gpu(self) -> bool {
        !matches!(self, Mode::Cpu | Mode::UnifiedCpuRead | Mode::CopyCpuRead)
    }
    fn device(self) -> bool {
        matches!(self, Mode::IpcMemEvent | Mode::IpcMemCpuSync | Mode::VmmEvent | Mode::VmmCpuSync | Mode::DmabufEvent)
    }
    fn dev_kind(self) -> DevKind {
        match self {
            Mode::IpcMemEvent | Mode::IpcMemCpuSync => DevKind::Ipc,
            Mode::DmabufEvent => DevKind::Dmabuf,
            _ => DevKind::Vmm,
        }
    }
    fn event(self) -> bool {
        matches!(self, Mode::UnifiedEvent | Mode::IpcMemEvent | Mode::VmmEvent | Mode::DmabufEvent)
    }
}

fn gpu_or_die(mode: &str) -> Gpu {
    match Gpu::new() {
        Ok(g) => g,
        Err(e) => {
            eprintln!("gpu-probe-hip: {e}");
            emit(Row::new("failed").s("mode", mode).s("reason", &e));
            std::process::exit(3);
        }
    }
}

fn sock_path(run: &str) -> String {
    format!("/tmp/gpu-s0-hip-{run}.sock")
}

/// Accept the reader's connection, giving up if the reader exits first.
fn accept_or_child_exit(l: &UnixListener, child: &mut std::process::Child, mode: Mode) -> UnixStream {
    l.set_nonblocking(true).unwrap();
    loop {
        match l.accept() {
            Ok((s, _)) => {
                s.set_nonblocking(false).unwrap();
                return s;
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                if let Ok(Some(st)) = child.try_wait() {
                    fatal(mode.name(), 0, &format!("reader exited before connecting: {st:?}"));
                }
                std::thread::sleep(Duration::from_millis(5));
            }
            Err(e) => fatal(mode.name(), 0, &format!("accept: {e}")),
        }
    }
}

fn writer(mode: Mode, sizes: &[usize], run: &str, iters_override: Option<usize>) {
    let m = mode.name();
    let gpu = if mode == Mode::Cpu { None } else { Some(gpu_or_die(m)) };
    let node = NodeBuilder::new().create::<ipc::Service>().expect("node");
    let sp = sock_path(run);
    let listener = if mode.device() {
        let _ = std::fs::remove_file(&sp);
        Some(UnixListener::bind(&sp).expect("bind"))
    } else {
        None
    };
    let ev = if mode.event() {
        match make_ipc_event(gpu.as_ref().unwrap()) {
            Ok(e) => Some(e),
            Err(e) => {
                emit(Row::new("skip").s("mode", m).s("reason", &format!("interprocess event: {e}")));
                return;
            }
        }
    } else {
        None
    };

    let exe = std::env::current_exe().unwrap();
    let sizes_arg = sizes.iter().map(|s| s.to_string()).collect::<Vec<_>>().join(",");
    let mut child_args = vec!["reader".to_string(), m.into(), sizes_arg, run.into()];
    if let Some(n) = iters_override {
        child_args.push(n.to_string());
    }
    let mut child = std::process::Command::new(exe).args(&child_args).spawn().expect("spawn reader");
    let sock = listener.as_ref().map(|l| accept_or_child_exit(l, &mut child, mode));

    for &size in sizes {
        let (warm, mut iters) = iters_for(size);
        if let Some(n) = iters_override {
            iters = n;
        }
        let words = (size / 4) as u64;
        let sl = slot_len(size, mode.device());
        let base = format!("gpu-s0-hip/{run}/{m}/{size}");
        let data = make_pub(&node, &format!("{base}/data"), sl);
        let ack = make_sub(&node, &format!("{base}/ack"));

        let mut d_src: *mut c_void = std::ptr::null_mut();
        let mut host_src: Vec<u8> = vec![];
        let mut reg: Option<Reg> = None;
        let mut wdev: Option<WDev> = None;
        if matches!(mode, Mode::Copy | Mode::CopyCpuRead) {
            d_src = must!(m, size, gpu.as_ref().unwrap().malloc(size));
        }
        if mode == Mode::Cpu {
            host_src = vec![0u8; size];
        }
        if mode.device() {
            match must!(m, size, writer_device_setup(gpu.as_ref().unwrap(), mode, size, sock.as_ref().unwrap())) {
                Some(d) => wdev = Some(d),
                None => continue,
            }
        }

        // wait for the reader's READY, then drain
        let mut spins = 0u64;
        loop {
            if let Some(s) = ack.receive().expect("ack receive") {
                let a: Ack = from_bytes(s.payload());
                if a.kind == 1 {
                    break;
                }
            }
            spins += 1;
            if spins % (1 << 20) == 0 {
                if let Ok(Some(st)) = child.try_wait() {
                    emit(Row::new("skip").s("mode", m).u("size", size as u64).s("reason", &format!("reader exited {st:?} before READY")));
                    return;
                }
            }
            std::hint::spin_loop();
        }
        std::thread::sleep(Duration::from_millis(20));
        while ack.receive().expect("drain").is_some() {}

        let mut lat = Vec::with_capacity(iters);
        let mut reader_ns = Vec::with_capacity(iters);
        let (mut gpu_mm, mut cpu_mm) = (0u64, 0u64);
        let mut gpu_mhz_mid = 0u64;
        let t_row = now_ns();
        for i in 0..warm + iters {
            let seed = seed_for(i);
            if mode == Mode::Cpu {
                let p = host_src.as_mut_ptr() as *mut u32;
                for w in 0..words as usize {
                    // SAFETY: w < words = size / 4, inside host_src.
                    unsafe { p.add(w).write(pat(w as u64, seed)) };
                }
            }
            let mut desc = Descriptor::zeroed();
            desc.magic = MAGIC;
            desc.ver = 1;
            desc.mode = mode as u8;
            desc.seq = i as u64;
            desc.seed = seed;
            desc.len = size as u64;

            // t0: the writer starts handing its data over (GPU modes: before the producing kernel)
            let t0 = now_ns();
            desc.t0_ns = t0;
            let mut s = data.loan_slice_uninit(sl).expect("loan");
            let raw = s.payload_mut().as_mut_ptr() as *mut u8;
            // SAFETY: frame_off < 256 = PAD, inside the loaned slot.
            let slot = unsafe { raw.add(frame_off(raw as usize)) };
            assert_eq!(slot as usize % 256, 0, "slot not 256-aligned");
            // SAFETY: DESC + size bytes follow `slot` inside the slot (slot_len).
            let payload = unsafe { slot.add(DESC) };
            match mode {
                // SAFETY: payload has `size` writable bytes; host_src has `size` bytes.
                Mode::Cpu => unsafe { std::ptr::copy_nonoverlapping(host_src.as_ptr(), payload, size) },
                Mode::Copy | Mode::CopyCpuRead => {
                    let g = gpu.as_ref().unwrap();
                    must!(m, size, g.fill(d_src as u64, words, seed));
                    // SAFETY: payload has `size` writable bytes; d_src is a `size`-byte allocation.
                    must!(m, size, hip!(g.api, memcpy_async(payload as *mut c_void, d_src, size, MEMCPY_D2H, g.stream)));
                    must!(m, size, g.sync());
                    desc.sync = 1;
                }
                Mode::UnifiedEvent | Mode::UnifiedCpuSync | Mode::UnifiedCpuRead => {
                    let g = gpu.as_ref().unwrap();
                    if reg.is_none() {
                        match Reg::register(g, payload as usize, true) {
                            Ok(r) => {
                                emit(r.row("writer", m, size));
                                reg = Some(r);
                            }
                            Err(e) => {
                                emit(Row::new("skip").s("side", "writer").s("mode", m).u("size", size as u64).s("reason", &e));
                                let _ = child.kill();
                                let _ = child.wait();
                                return;
                            }
                        }
                    }
                    let d = must!(m, size, reg.as_ref().unwrap().dptr(payload as usize, size, true));
                    must!(m, size, g.fill(d, words, seed));
                }
                _ => {
                    let g = gpu.as_ref().unwrap();
                    must!(m, size, g.fill(wdev.as_ref().unwrap().ptr(), words, seed));
                    desc.block_id = 1;
                    desc.gen = 1;
                }
            }
            if mode.unified() || mode.device() {
                let g = gpu.as_ref().unwrap();
                if let Some((e, h)) = ev.as_ref() {
                    must!(m, size, hip!(g.api, event_record(*e, g.stream)));
                    desc.sync = 2;
                    desc.event = *h;
                } else {
                    must!(m, size, g.sync());
                    desc.sync = 1;
                }
            }
            // SAFETY: slot has DESC writable bytes; desc is DESC bytes.
            unsafe { std::ptr::copy_nonoverlapping(&desc as *const Descriptor as *const u8, slot, DESC) };
            // SAFETY: every byte the reader reads (descriptor + payload) was written above;
            // the pad bytes are never read.
            let s = unsafe { s.assume_init() };
            s.send().expect("send");

            let mut spins = 0u64;
            let a: Ack = loop {
                spins += 1;
                if spins % (1 << 20) == 0 {
                    if let Ok(Some(st)) = child.try_wait() {
                        emit(Row::new("skip").s("mode", m).u("size", size as u64).s("reason", &format!("reader exited {st:?}")));
                        return;
                    }
                }
                if let Some(s) = ack.receive().expect("ack receive") {
                    let a: Ack = from_bytes(s.payload());
                    if a.kind == 2 && a.seq == i as u64 {
                        break a;
                    }
                }
                std::hint::spin_loop();
            };
            if i == warm + iters / 2 {
                gpu_mhz_mid = gpu_mhz();
            }
            if i >= warm {
                lat.push(a.t1_ns - t0);
                reader_ns.push(a.reader_ns);
            }
            gpu_mm += a.gpu_mismatch;
            cpu_mm += a.cpu_mismatch;
        }
        let row_ms = (now_ns() - t_row) as f64 / 1e6;
        if lat.len() == iters {
            lat.sort_unstable();
            reader_ns.sort_unstable();
            let mean = lat.iter().sum::<u64>() as f64 / lat.len() as f64 / 1e3;
            let mut r = Row::new("latency")
                .s("mode", m)
                .u("size", size as u64)
                .u("iters", iters as u64)
                .u("warmup", warm as u64)
                .f("p50_us", pct_us(&lat, 0.5))
                .f("p99_us", pct_us(&lat, 0.99))
                .f("min_us", pct_us(&lat, 0.0))
                .f("max_us", pct_us(&lat, 1.0))
                .f("mean_us", mean)
                .f("reader_p50_us", pct_us(&reader_ns, 0.5))
                .u("gpu_mismatch", gpu_mm)
                .u("cpu_mismatch", cpu_mm)
                .u("words_per_frame", words)
                .u("frames", (warm + iters) as u64)
                .u("gpu_mhz_mid", gpu_mhz_mid)
                .f("row_wall_ms", row_ms);
            if let Some(smi) = smi_sclk() {
                r = r.s("smi_sclk_post", &smi);
            }
            emit(r);
        }
        if let Some(r) = reg.take() {
            r.release(gpu.as_ref().unwrap());
        }
        if !d_src.is_null() {
            // SAFETY: freeing this size's copy-mode allocation, once.
            let _ = hip!(gpu.as_ref().unwrap().api, free(d_src));
        }
        if let Some(d) = wdev.take() {
            // Let the reader drop its import before the export goes away.
            std::thread::sleep(Duration::from_millis(60));
            d.release(gpu.as_ref().unwrap());
        }
        drop(data);
        drop(ack);
    }
    let st = child.wait().expect("reader wait");
    if !st.success() {
        emit(Row::new("failed").s("mode", m).s("reason", &format!("reader exited {st:?}")));
        std::process::exit(6);
    }
    if listener.is_some() {
        let _ = std::fs::remove_file(&sp);
    }
}

fn parent_gone(ppid0: libc::pid_t) -> bool {
    // SAFETY: getppid has no preconditions.
    unsafe { libc::getppid() != ppid0 }
}

fn reader(mode: Mode, sizes: &[usize], run: &str, iters_override: Option<usize>) {
    let m = mode.name();
    // SAFETY: getppid has no preconditions.
    let ppid0 = unsafe { libc::getppid() };
    let gpu = if mode.reader_gpu() || mode.reader_registers() { Some(gpu_or_die(m)) } else { None };
    let node = NodeBuilder::new().create::<ipc::Service>().expect("node");
    let sock = if mode.device() {
        let path = sock_path(run);
        let mut s = None;
        for _ in 0..1000 {
            if let Ok(c) = UnixStream::connect(&path) {
                s = Some(c);
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        Some(s.expect("connect broker socket"))
    } else {
        None
    };
    let mut ev_cache: Option<([u8; 64], HipEvent)> = None;

    for &size in sizes {
        let (warm, mut iters) = iters_for(size);
        if let Some(n) = iters_override {
            iters = n;
        }
        let words = (size / 4) as u64;
        let base = format!("gpu-s0-hip/{run}/{m}/{size}");
        let data = make_sub(&node, &format!("{base}/data"));
        let ack = make_pub(&node, &format!("{base}/ack"), std::mem::size_of::<Ack>());
        let mut d_dst: *mut c_void = std::ptr::null_mut();
        if mode == Mode::Copy {
            d_dst = must!(m, size, gpu.as_ref().unwrap().malloc(size));
        }
        let mut rdev: Option<RDev> = None;
        if mode.device() {
            match must!(m, size, reader_device_setup(gpu.as_ref().unwrap(), mode, size, sock.as_ref().unwrap())) {
                Some(d) => rdev = Some(d),
                None => continue,
            }
        }
        let mut reg: Option<Reg> = None;
        let mut first = true;
        let mut done = 0usize;
        let mut last_ready = 0u64;
        let mut idle_spins = 0u64;
        while done < warm + iters {
            let sample = match data.receive().expect("receive") {
                Some(s) => s,
                None => {
                    if first && now_ns() - last_ready > 5_000_000 {
                        send_bytes(&ack, as_bytes(&Ack { kind: 1, ..Default::default() }));
                        last_ready = now_ns();
                    }
                    idle_spins += 1;
                    if idle_spins % (1 << 22) == 0 && parent_gone(ppid0) {
                        eprintln!("gpu-probe-hip reader: writer is gone; exiting");
                        std::process::exit(7);
                    }
                    std::hint::spin_loop();
                    continue;
                }
            };
            let t_rx = now_ns();
            let raw = sample.payload();
            let slot = &raw[frame_off(raw.as_ptr() as usize)..];
            assert_eq!(slot.as_ptr() as usize % 256, 0, "reader slot not 256-aligned");
            let desc: Descriptor = from_bytes(&slot[..DESC]);
            assert_eq!(desc.magic, MAGIC);
            let seed = desc.seed;
            let mut a = Ack { kind: 2, seq: desc.seq, ..Default::default() };

            if first && mode.reader_registers() {
                let g = gpu.as_ref().unwrap();
                let payload = &slot[DESC..];
                let pre = if size >= 1 << 20 { cpu_read_gbps(payload.as_ptr(), size) } else { 0.0 };
                match Reg::register(g, payload.as_ptr() as usize, false) {
                    Ok(r) => {
                        let post = if size >= 1 << 20 { cpu_read_gbps(payload.as_ptr(), size) } else { 0.0 };
                        let (malloc_bw, pinned_bw) = if size >= 1 << 20 {
                            let v = vec![1u8; size];
                            let mb = cpu_read_gbps(v.as_ptr(), size);
                            let mut h: *mut c_void = std::ptr::null_mut();
                            // SAFETY: live out-pointer.
                            let pb = match hip!(g.api, host_malloc(&mut h, size, 0)) {
                                Ok(()) => {
                                    // SAFETY: h is a fresh `size`-byte host allocation.
                                    unsafe { std::ptr::write_bytes(h as *mut u8, 1, size) };
                                    let pb = cpu_read_gbps(h as *const u8, size);
                                    // SAFETY: freeing the allocation made above.
                                    let _ = hip!(g.api, host_free(h));
                                    pb
                                }
                                Err(_) => -1.0,
                            };
                            (mb, pb)
                        } else {
                            (0.0, 0.0)
                        };
                        emit(
                            r.row("reader", m, size)
                                .f("cpu_read_gbps_shm_unregistered", pre)
                                .f("cpu_read_gbps_shm_registered", post)
                                .f("cpu_read_gbps_malloc", malloc_bw)
                                .f("cpu_read_gbps_hipHostMalloc", pinned_bw),
                        );
                        reg = Some(r);
                    }
                    Err(e) => {
                        emit(Row::new("skip").s("side", "reader").s("mode", m).u("size", size as u64).s("reason", &e));
                        std::process::exit(3);
                    }
                }
            }
            if first && desc.sync == 2 {
                let fresh = !matches!(&ev_cache, Some((h, _)) if *h == desc.event);
                if fresh {
                    let t = now_ns();
                    match open_ipc_event(gpu.as_ref().unwrap(), &desc.event) {
                        Ok(ev) => {
                            emit(Row::new("onetime").s("side", "reader").s("mode", m).u("size", size as u64).f("ipc_event_open_us", (now_ns() - t) as f64 / 1e3));
                            ev_cache = Some((desc.event, ev));
                        }
                        Err(e) => {
                            emit(Row::new("skip").s("side", "reader").s("mode", m).s("reason", &format!("hipIpcOpenEventHandle: {e}")));
                            std::process::exit(4);
                        }
                    }
                }
            }
            first = false;

            match mode {
                Mode::Cpu | Mode::UnifiedCpuRead | Mode::CopyCpuRead => {
                    a.cpu_mismatch = cpu_verify(&slot[DESC..DESC + size], seed);
                    a.t1_ns = now_ns();
                }
                Mode::Copy => {
                    let g = gpu.as_ref().unwrap();
                    // SAFETY: d_dst is a `size`-byte allocation; the slot holds `size` payload bytes.
                    must!(m, size, hip!(g.api, memcpy_async(d_dst, slot[DESC..].as_ptr() as *const c_void, size, MEMCPY_H2D, g.stream)));
                    must!(m, size, g.verify(d_dst as u64, words, seed));
                    must!(m, size, g.sync());
                    a.t1_ns = now_ns();
                    a.gpu_mismatch = g.result().0;
                    if done % 64 == 0 || done + 1 == warm + iters {
                        a.cpu_mismatch = cpu_verify(&slot[DESC..DESC + size], seed);
                    }
                }
                Mode::UnifiedEvent | Mode::UnifiedCpuSync => {
                    let g = gpu.as_ref().unwrap();
                    if desc.sync == 2 {
                        must!(m, size, hip!(g.api, stream_wait_event(g.stream, ev_cache.as_ref().unwrap().1, 0)));
                    }
                    let d = must!(m, size, reg.as_ref().unwrap().dptr(slot[DESC..].as_ptr() as usize, size, false));
                    must!(m, size, g.verify(d, words, seed));
                    must!(m, size, g.sync());
                    a.t1_ns = now_ns();
                    a.gpu_mismatch = g.result().0;
                    if done % 64 == 0 || done + 1 == warm + iters {
                        a.cpu_mismatch = cpu_verify(&slot[DESC..DESC + size], seed);
                    }
                }
                _ => {
                    let g = gpu.as_ref().unwrap();
                    if desc.sync == 2 {
                        must!(m, size, hip!(g.api, stream_wait_event(g.stream, ev_cache.as_ref().unwrap().1, 0)));
                    }
                    must!(m, size, g.verify(rdev.as_ref().unwrap().ptr() + desc.offset, words, seed));
                    must!(m, size, g.sync());
                    a.t1_ns = now_ns();
                    a.gpu_mismatch = g.result().0;
                }
            }
            a.reader_ns = a.t1_ns - t_rx;
            drop(sample);
            send_bytes(&ack, as_bytes(&a));
            done += 1;
        }
        if let Some(r) = reg.take() {
            r.release(gpu.as_ref().unwrap());
        }
        if !d_dst.is_null() {
            // SAFETY: freeing this size's allocation, once.
            let _ = hip!(gpu.as_ref().unwrap().api, free(d_dst));
        }
        if let Some(d) = rdev.take() {
            d.release(gpu.as_ref().unwrap());
        }
        // let the writer read the last ack before the service goes away
        std::thread::sleep(Duration::from_millis(50));
        drop(data);
        drop(ack);
    }
}

// ---------------------------------------------------------------- floor + probe

/// A MAP_SHARED anonymous-file mapping: what an iceoryx2 /dev/shm segment is.
fn shared_mapping(size: usize) -> Result<usize, String> {
    #[cfg(target_os = "linux")]
    // SAFETY: memfd_create with a NUL-terminated name.
    let fd = unsafe { libc::memfd_create(c"gpu-s0-hip-floor".as_ptr(), 0) };
    #[cfg(not(target_os = "linux"))]
    let fd = {
        let name = CString::new(format!("/gpu-s0-hip-{}", std::process::id())).unwrap();
        // SAFETY: NUL-terminated name; unlinked right away, the fd keeps the object.
        let fd = unsafe { libc::shm_open(name.as_ptr(), libc::O_RDWR | libc::O_CREAT | libc::O_EXCL, 0o600) };
        // SAFETY: as above.
        unsafe { libc::shm_unlink(name.as_ptr()) };
        fd
    };
    if fd < 0 {
        return Err(format!("shared memory fd: {}", std::io::Error::last_os_error()));
    }
    // SAFETY: fd is ours; the mapping keeps the object alive after close.
    unsafe {
        if libc::ftruncate(fd, size as libc::off_t) != 0 {
            libc::close(fd);
            return Err(format!("ftruncate: {}", std::io::Error::last_os_error()));
        }
        let p = libc::mmap(std::ptr::null_mut(), size, libc::PROT_READ | libc::PROT_WRITE, libc::MAP_SHARED, fd, 0);
        libc::close(fd);
        if p == libc::MAP_FAILED {
            return Err(format!("mmap: {}", std::io::Error::last_os_error()));
        }
        std::ptr::write_bytes(p as *mut u8, 0, size);
        Ok(p as usize)
    }
}

fn floor(sizes: &[usize], iters_override: Option<usize>) {
    // One process, one stream: fill + verify + sync with no transport, on three memory kinds,
    // interleaved per iteration so all three see the same GPU clock.
    let g = gpu_or_die("floor");
    for &size in sizes {
        let (warm, mut iters) = iters_for(size);
        if let Some(n) = iters_override {
            iters = n;
        }
        let words = (size / 4) as u64;
        let d = must!("floor", size, g.malloc(size));
        let shm = must!("floor", size, shared_mapping(size));
        let reg = must!("floor", size, Reg::register(&g, shm, true));
        let mut ha: *mut c_void = std::ptr::null_mut();
        // SAFETY: live out-pointer.
        must!("floor", size, hip!(g.api, host_malloc(&mut ha, size, 0)));
        let mut dha: *mut c_void = std::ptr::null_mut();
        // SAFETY: ha is a live hipHostMalloc allocation; live out-pointer.
        must!("floor", size, hip!(g.api, host_get_device_pointer(&mut dha, ha, 0)));
        let shm_d = must!("floor", size, reg.dptr(shm, size, true));
        let kinds = [("floor-device", d as u64), ("floor-registered-shm", shm_d), ("floor-hipHostMalloc", dha as u64)];
        let mut lat = vec![vec![]; 3];
        let mut mm = [0u64; 3];
        let mut mhz = 0;
        let t_row = now_ns();
        for i in 0..warm + iters {
            let seed = seed_for(i);
            for (k, (_, ptr)) in kinds.iter().enumerate() {
                let t0 = now_ns();
                must!("floor", size, g.fill(*ptr, words, seed));
                must!("floor", size, g.verify(*ptr, words, seed));
                must!("floor", size, g.sync());
                let t1 = now_ns();
                mm[k] += g.result().0;
                if i >= warm {
                    lat[k].push(t1 - t0);
                }
            }
            if i == warm + iters / 2 {
                mhz = gpu_mhz();
            }
        }
        let row_ms = (now_ns() - t_row) as f64 / 1e6;
        for (k, (name, _)) in kinds.iter().enumerate() {
            let l = &mut lat[k];
            l.sort_unstable();
            let mut r = Row::new("latency")
                .s("mode", name)
                .u("size", size as u64)
                .u("iters", iters as u64)
                .u("warmup", warm as u64)
                .f("p50_us", pct_us(l, 0.5))
                .f("p99_us", pct_us(l, 0.99))
                .f("min_us", pct_us(l, 0.0))
                .f("max_us", pct_us(l, 1.0))
                .u("gpu_mismatch", mm[k])
                .u("cpu_mismatch", 0)
                .u("frames", (warm + iters) as u64)
                .u("gpu_mhz_mid", mhz)
                .f("row_wall_ms", row_ms);
            if k == 1 {
                r = r.f("register_us", reg.reg_ns as f64 / 1e3).s("reg_path", reg.path).u("flags", u64::from(reg.flags));
            }
            emit(r);
        }
        reg.release(&g);
        // SAFETY: unmapping the floor's shared mapping after its registration is released.
        unsafe { libc::munmap(shm as *mut c_void, size) };
        // SAFETY: freeing allocations made above, once each.
        let _ = hip!(g.api, host_free(ha));
        // SAFETY: as above.
        let _ = hip!(g.api, free(d));
    }
}

fn probe() {
    let g = gpu_or_die("probe");
    let mut name = [0 as c_char; 256];
    // SAFETY: the buffer holds 256 chars.
    let _ = hip!(g.api, device_get_name(name.as_mut_ptr(), 256, 0));
    // SAFETY: NUL-terminated by the runtime (buffer zeroed first).
    let name = unsafe { std::ffi::CStr::from_ptr(name.as_ptr()) }.to_string_lossy().into_owned();
    let (mut rt, mut drv) = (0, 0);
    // SAFETY: live out-pointers.
    let _ = hip!(g.api, runtime_get_version(&mut rt));
    // SAFETY: live out-pointer.
    let _ = hip!(g.api, driver_get_version(&mut drv));
    let (ma, mi, pa) = decode_hip_version(rt);
    let p = &g.props;
    let warp_prop = prop_i32(p, prop::WARP_SIZE);
    let cu_prop = prop_i32(p, prop::MULTIPROCESSOR_COUNT);
    let layout_ok = i64::from(warp_prop) == g.attr(ATTR_WARP_SIZE) && i64::from(cu_prop) == g.attr(ATTR_MULTIPROCESSOR_COUNT);
    let mut r = Row::new("probe")
        .s("device", &name)
        .s("gcn_arch", &prop_str(p, prop::GCN_ARCH_NAME, 256))
        .i("hip_runtime_version", i64::from(rt))
        .s("hip_runtime", &format!("{ma}.{mi}.{pa}"))
        .i("hip_driver_version", i64::from(drv))
        .s("hip_library", g.api.lib_name)
        .u("total_global_mem", u64::from_le_bytes(p[prop::TOTAL_GLOBAL_MEM..prop::TOTAL_GLOBAL_MEM + 8].try_into().unwrap()))
        .i("prop_hostRegisterSupported", i64::from(prop_i32(p, prop::HOST_REGISTER_SUPPORTED)))
        .i("prop_hostRegisterReadOnlySupported", i64::from(prop_i32(p, prop::HOST_REGISTER_READ_ONLY_SUPPORTED)))
        .i("prop_canUseHostPointerForRegisteredMem", i64::from(prop_i32(p, prop::CAN_USE_HOST_POINTER_FOR_REGISTERED_MEM)))
        .i("prop_ipcEventSupported", i64::from(prop_i32(p, prop::IPC_EVENT_SUPPORTED)))
        .i("prop_warpSize", i64::from(warp_prop))
        .i("prop_multiProcessorCount", i64::from(cu_prop))
        .i("prop_pciBusID", i64::from(prop_i32(p, prop::PCI_BUS_ID)))
        .b("prop_layout_matches_attributes", layout_ok)
        .s("kernels", &g.kernel_info)
        .s("clock_sysfs", &clock_path().map(|p| p.display().to_string()).unwrap_or_else(|| "none".into()))
        .u("gpu_mhz_now", gpu_mhz())
        .s("hip_missing_symbols", &g.api.missing().join(","));
    for (n, a) in ATTRS {
        r = r.i(n, g.attr(*a));
    }
    match &g.backend {
        Some(b) => {
            let c = b.caps();
            r = r
                .s("cerulion_device", "ok")
                .b("caps_integrated", c.integrated)
                .b("caps_host_register", c.host_register)
                .b("caps_read_only_register", c.read_only_register)
                .b("caps_host_pointer_is_device_pointer", c.host_pointer_is_device_pointer)
                .b("caps_posix_fd_export", c.posix_fd_export)
                .s("caps_uuid", &c.uuid.to_string());
        }
        None => r = r.s("cerulion_device", g.backend_err.as_deref().unwrap_or("?")),
    }
    r = r.s("ipc_event_handle", &make_ipc_event(&g).map(|_| "ok".to_string()).unwrap_or_else(|e| e));
    let ipcmem = g.malloc(1 << 20).and_then(|ptr| {
        let mut h = IpcHandle::zeroed();
        // SAFETY: live out-pointer; ptr is a live allocation.
        let res = hip!(g.api, ipc_get_mem_handle(&mut h, ptr));
        // SAFETY: freeing the allocation made above.
        let _ = hip!(g.api, free(ptr));
        res
    });
    r = r.s("ipc_mem_handle", &ipcmem.map(|_| "ok".to_string()).unwrap_or_else(|e| e));
    let vmm = (|| -> Result<String, String> {
        let gran = vmm_granularity(&g)?;
        let prop = vmm_prop();
        let mut h: HipMemHandle = std::ptr::null_mut();
        // SAFETY: live out-pointer and prop.
        hip!(g.api, mem_create(&mut h, gran, &prop, 0))?;
        let mut fd: c_int = -1;
        // SAFETY: the runtime writes one int through the out-pointer.
        let ex = hip!(g.api, mem_export_to_shareable_handle(&mut fd as *mut c_int as *mut c_void, h, MEM_HANDLE_TYPE_POSIX_FD, 0));
        if fd >= 0 {
            // SAFETY: closing the exported fd.
            unsafe { libc::close(fd) };
        }
        // SAFETY: releasing the handle created above.
        let _ = hip!(g.api, mem_release(h));
        ex.map(|_| format!("ok gran={gran}"))
    })();
    r = r.s("vmm_create_export", &vmm.unwrap_or_else(|e| e));
    let dmabuf = (|| -> Result<String, String> {
        let hsa = g.hsa()?;
        let export = hsa.portable_export_dmabuf.ok_or("hsa_amd_portable_export_dmabuf is not exported")?;
        let ptr = g.malloc(2 << 20)?;
        let (mut fd, mut off) = (-1, 0u64);
        // SAFETY: live 2 MiB device allocation; live out-pointers.
        let rc = unsafe { export(ptr, 2 << 20, &mut fd, &mut off) };
        let res = if rc == 0 {
            if let Some(close) = hsa.portable_close_dmabuf {
                // SAFETY: closing the dma-buf exported above.
                unsafe { close(fd) };
            }
            Ok(format!("ok offset={off} lib={}", hsa.lib_name))
        } else {
            Err(format!("hsa_amd_portable_export_dmabuf -> {}", hsa_err(hsa, rc)))
        };
        // SAFETY: freeing the allocation made above.
        let _ = hip!(g.api, free(ptr));
        res
    })();
    r = r.s("dmabuf_export", &dmabuf.unwrap_or_else(|e| e));
    emit(r);
}

/// Remove the iceoryx2 resources of dead nodes (a unit killed by its
/// timeout leaves its segments in /dev/shm). Run only on a machine this
/// probe owns: it cleans every dead node, not just the probe's.
fn cleanup_dead_nodes() {
    let (mut dead, mut cleaned, mut failed) = (0u64, 0u64, 0u64);
    let res = Node::<ipc::Service>::list(Config::global_config(), |state| {
        if let NodeState::<ipc::Service>::Dead(view) = state {
            dead += 1;
            match view.try_remove_stale_resources() {
                Ok(()) => cleaned += 1,
                Err(_) => failed += 1,
            }
        }
        CallbackProgression::Continue
    });
    emit(Row::new("cleanup").u("dead_nodes", dead).u("cleaned", cleaned).u("failed", failed).b("list_ok", res.is_ok()));
}

fn usage() -> ! {
    eprintln!(
        "usage: gpu-probe-hip probe | floor [iters|default] [sizes|all] | writer <mode> [sizes|all] [iters|default] | dry-cpu [sizes|all] [iters|default] | modes | cleanup-dead-nodes\n\
         modes: {}",
        ALL_MODES.iter().map(|m| m.name()).collect::<Vec<_>>().join(" ")
    );
    std::process::exit(2);
}

fn arg_iters(a: Option<&String>) -> Option<usize> {
    a.filter(|x| x.as_str() != "default").map(|x| x.parse().unwrap_or_else(|_| usage()))
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let sizes_of = |a: Option<&String>| match a {
        None => default_sizes(),
        Some(s) => parse_sizes(s).unwrap_or_else(|e| {
            eprintln!("{e}");
            usage()
        }),
    };
    let run_id = || format!("{}-{}", std::process::id(), now_ns() % 1_000_000);
    match args.get(1).map(String::as_str) {
        Some("probe") => probe(),
        Some("cleanup-dead-nodes") => cleanup_dead_nodes(),
        Some("modes") => println!("{}", ALL_MODES.iter().map(|m| m.name()).collect::<Vec<_>>().join(" ")),
        Some("floor") => floor(&sizes_of(args.get(3)), arg_iters(args.get(2))),
        Some("dry-cpu") => writer(Mode::Cpu, &sizes_of(args.get(2)), &run_id(), arg_iters(args.get(3))),
        Some("writer") => {
            let mode = args.get(2).and_then(|s| Mode::parse(s)).unwrap_or_else(|| usage());
            writer(mode, &sizes_of(args.get(3)), &run_id(), arg_iters(args.get(4)));
        }
        Some("reader") => {
            let mode = args.get(2).and_then(|s| Mode::parse(s)).unwrap_or_else(|| usage());
            let run = args.get(4).unwrap_or_else(|| usage());
            reader(mode, &sizes_of(args.get(3)), run, arg_iters(args.get(5)));
        }
        _ => usage(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_names_roundtrip_and_are_unique() {
        let mut seen = std::collections::HashSet::new();
        for m in ALL_MODES {
            assert_eq!(Mode::parse(m.name()), Some(*m));
            assert!(seen.insert(*m as u8), "duplicate wire value {}", *m as u8);
        }
        assert_eq!(Mode::parse("device-event"), None);
        assert_eq!(ALL_MODES.len(), 11);
    }

    #[test]
    fn mode_properties() {
        assert!(Mode::VmmEvent.device() && Mode::VmmEvent.event() && Mode::VmmEvent.dev_kind() == DevKind::Vmm);
        assert!(Mode::IpcMemCpuSync.device() && !Mode::IpcMemCpuSync.event() && Mode::IpcMemCpuSync.dev_kind() == DevKind::Ipc);
        assert!(Mode::DmabufEvent.device() && Mode::DmabufEvent.dev_kind() == DevKind::Dmabuf);
        assert!(Mode::UnifiedCpuRead.unified() && !Mode::UnifiedCpuRead.reader_gpu() && !Mode::UnifiedCpuRead.reader_registers());
        assert!(!Mode::Cpu.reader_gpu() && !Mode::Cpu.device() && !Mode::Cpu.unified());
        assert!(Mode::Copy.reader_gpu() && !Mode::CopyCpuRead.reader_gpu());
    }

    #[test]
    fn kernel_source_keeps_the_shared_pattern_and_a_fixed_block_reduction() {
        assert!(KERNELS.contains("(seed * 0x9E3779B9u)"));
        assert!(KERNELS.contains("x *= 2246822519u"));
        assert!(KERNELS.contains("__shared__ unsigned long long sbad[256]"));
        assert!(!KERNELS.contains("__shfl"), "no lane-width-dependent shuffles");
        assert_eq!(BLOCK, 256);
    }

    #[test]
    fn the_floor_mapping_is_shared_and_zeroed() {
        let p = shared_mapping(1 << 16).unwrap();
        // SAFETY: p is a fresh 64 KiB mapping.
        let s = unsafe { std::slice::from_raw_parts(p as *const u8, 1 << 16) };
        assert!(s.iter().all(|&b| b == 0));
        let span = cerulion_device::span_of(p).unwrap();
        assert!(span.contains(p, 1 << 16));
        // SAFETY: unmapping the test mapping.
        unsafe { libc::munmap(p as *mut c_void, 1 << 16) };
    }

    #[test]
    fn round_up_to_granularity() {
        assert_eq!(round_up(4096, 2 << 20), 2 << 20);
        assert_eq!(round_up(2 << 20, 2 << 20), 2 << 20);
        assert_eq!(round_up((2 << 20) + 1, 2 << 20), 4 << 20);
    }

    #[test]
    fn without_rocm_the_gpu_open_names_the_libraries() {
        if HipApi::open(HIP_LIBS).is_ok() {
            return; // a ROCm machine: nothing to check here
        }
        let e = Gpu::new().err().unwrap();
        assert!(e.starts_with("HIP library not found"), "{e}");
        for lib in HIP_LIBS {
            assert!(e.contains(lib), "{lib} missing from: {e}");
        }
        assert!(e.contains("cerulion_device"), "{e}");
    }
}
