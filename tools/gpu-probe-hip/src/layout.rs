//! Pure pieces of the probe: the wire descriptor, the in-slot pad, the JSON
//! row builder, the iteration plan and the fd argument for VMM import.
//! Everything here is unit tested without a GPU.

use std::ffi::c_void;

/// Every frame starts with this many descriptor bytes.
pub const DESC: usize = 256;

/// The frame starts at the next 256-byte boundary inside the slot
/// (GPU_S0_ALIGN=pad, the only alignment this probe uses). The slot is this
/// many bytes longer than the frame so the pad always fits.
pub const PAD: usize = 256;

pub const MAGIC: u32 = 0x5330_4750; // "PG0S", same as the CUDA probe

/// Threads per block for both kernels. The verify kernel's shared-memory
/// reduction is sized for exactly this many threads.
pub const BLOCK: u32 = 256;

/// Byte offset from `ptr` to the next 256-byte boundary (0 when aligned).
pub fn frame_off(ptr: usize) -> usize {
    (256 - ptr % 256) % 256
}

/// Slot length to loan for a frame of `size` payload bytes. Device modes send
/// only the descriptor.
pub fn slot_len(size: usize, descriptor_only: bool) -> usize {
    PAD + if descriptor_only { DESC } else { DESC + size }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Descriptor {
    pub magic: u32,
    pub ver: u16,
    pub mode: u8,
    pub sync: u8, // 0 none, 1 cpu-complete, 2 hip ipc event
    pub seq: u64,
    pub seed: u32,
    pub gen: u32,
    pub len: u64,
    pub t0_ns: u64,
    pub block_id: u32,
    pub _pad0: u32,
    pub offset: u64,
    pub event: [u8; 64],
    pub _pad: [u8; 256 - 120],
}
const _: () = assert!(std::mem::size_of::<Descriptor>() == DESC);

impl Descriptor {
    pub fn zeroed() -> Self {
        // SAFETY: every field is an integer or an integer array; all-zero is valid.
        unsafe { std::mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Ack {
    pub kind: u64, // 1 ready, 2 frame
    pub seq: u64,
    pub t1_ns: u64,
    pub gpu_mismatch: u64,
    pub cpu_mismatch: u64,
    pub reader_ns: u64,
    pub _r: [u64; 2],
}

/// One-time handshake for the device modes, writer to reader over the unix
/// socket. `kind` 0 tells the reader to skip this size.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct DevMsg {
    pub kind: u64,
    pub bsize: u64,
    pub size: u64,
    pub t_send_ns: u64,
    pub offset: u64,
    pub has_fd: u64,
    pub ipc: [u8; 64],
}
const _: () = assert!(std::mem::size_of::<DevMsg>() == 112);

impl DevMsg {
    pub fn zeroed() -> Self {
        // SAFETY: integers and an integer array; all-zero is valid.
        unsafe { std::mem::zeroed() }
    }
}

pub fn as_bytes<T: Copy>(v: &T) -> &[u8] {
    // SAFETY: T is a repr(C) plain-data type in this crate; reading its bytes is defined.
    unsafe { std::slice::from_raw_parts(v as *const T as *const u8, std::mem::size_of::<T>()) }
}

pub fn from_bytes<T: Copy>(b: &[u8]) -> T {
    assert!(b.len() >= std::mem::size_of::<T>());
    // SAFETY: length checked above; T is plain data valid for any bit pattern.
    unsafe { std::ptr::read_unaligned(b.as_ptr() as *const T) }
}

/// The seeded word pattern; identical to the CUDA S0 probe and to `pat()` in
/// the HIP kernels.
pub fn pat(i: u64, seed: u32) -> u32 {
    let mut x = (i as u32).wrapping_mul(2654435761) ^ seed.wrapping_mul(0x9E37_79B9) ^ ((i >> 32) as u32);
    x ^= x >> 15;
    x = x.wrapping_mul(2246822519);
    x ^= x >> 13;
    x
}

pub fn seed_for(i: usize) -> u32 {
    (i as u32).wrapping_add(1).wrapping_mul(7919)
}

/// (warmup, iters) per size: same as the CUDA S0 probe.
pub fn iters_for(size: usize) -> (usize, usize) {
    if size <= 1 << 20 {
        (100, 2000)
    } else if size <= 16 << 20 {
        (30, 500)
    } else {
        (20, 300)
    }
}

/// 4 KiB to 64 MiB in steps of 4x: the 8 sizes of the CUDA S0 probe.
pub fn default_sizes() -> Vec<usize> {
    (0..8).map(|k| 4096usize << (2 * k)).collect()
}

pub fn parse_sizes(s: &str) -> Result<Vec<usize>, String> {
    if s == "all" {
        return Ok(default_sizes());
    }
    s.split(',')
        .map(|x| {
            let v: usize = x.trim().parse().map_err(|e| format!("bad size {x:?}: {e}"))?;
            if v == 0 || v % 4 != 0 {
                return Err(format!("size {v} must be a positive multiple of 4"));
            }
            Ok(v)
        })
        .collect()
}

/// Percentile (nearest rank on a sorted slice), in microseconds.
pub fn pct_us(sorted_ns: &[u64], p: f64) -> f64 {
    let i = ((sorted_ns.len() as f64 - 1.0) * p).round() as usize;
    sorted_ns[i] as f64 / 1000.0
}

pub fn cpu_verify(payload: &[u8], seed: u32) -> u64 {
    let words = payload.len() / 4;
    let p = payload.as_ptr() as *const u32;
    let mut bad = 0u64;
    for i in 0..words {
        // SAFETY: i < words, so the 4 bytes at 4*i are inside `payload`.
        let v = unsafe { p.add(i).read_unaligned() };
        bad += (v != pat(i as u64, seed)) as u64;
    }
    bad
}

/// Escape a string for a JSON string literal.
pub fn json_str(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            '\r' => o.push_str("\\r"),
            '\t' => o.push_str("\\t"),
            c if (c as u32) < 0x20 => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

/// A RESULT line: `RESULT {"align":"pad","kind":...,...}`.
pub struct Row(String);

impl Row {
    pub fn new(kind: &str) -> Row {
        Row(format!("{{\"align\":\"pad\",\"kind\":{}", json_str(kind)))
    }
    pub fn s(mut self, k: &str, v: &str) -> Row {
        self.0.push_str(&format!(",{}:{}", json_str(k), json_str(v)));
        self
    }
    pub fn u(mut self, k: &str, v: u64) -> Row {
        self.0.push_str(&format!(",{}:{v}", json_str(k)));
        self
    }
    pub fn i(mut self, k: &str, v: i64) -> Row {
        self.0.push_str(&format!(",{}:{v}", json_str(k)));
        self
    }
    pub fn f(mut self, k: &str, v: f64) -> Row {
        let v = if v.is_finite() { format!("{v:.2}") } else { "null".into() };
        self.0.push_str(&format!(",{}:{v}", json_str(k)));
        self
    }
    pub fn b(mut self, k: &str, v: bool) -> Row {
        self.0.push_str(&format!(",{}:{v}", json_str(k)));
        self
    }
    pub fn line(&self) -> String {
        format!("RESULT {}}}", self.0)
    }
}

/// The `osHandle` argument for `hipMemImportFromShareableHandle` with a POSIX
/// fd, built so that BOTH conventions read the same fd:
///
/// - ROCm up to 7.0.2 (clr `rocdevice.cpp`) reads `*(int*)osHandle`;
/// - clr `develop` reads `(int)(uintptr_t)osHandle`, the CUDA convention.
///
/// The pointer is `base + fd` with `base` a multiple of 2^32, so its low 32
/// bits are the fd, and the page it points into holds the fd at that byte.
/// A wrong single guess would either fail or (by-value fd read as a pointer)
/// fault on address `fd`.
pub struct DualFdArg {
    page: *mut c_void,
    ptr: *mut c_void,
}

impl DualFdArg {
    pub fn new(fd: i32) -> Result<DualFdArg, String> {
        if !(0..4092).contains(&fd) {
            return Err(format!("fd {fd} out of range for the dual-convention handle"));
        }
        let in_page = (fd as usize) & !0xfff;
        for k in 0x40usize..0x7e00 {
            let base = k << 32;
            let want = base + in_page;
            // SAFETY: anonymous private mapping with a placement hint; the
            // kernel picks another address if the hint is taken, which is
            // checked below. No existing mapping is replaced (no MAP_FIXED).
            let p = unsafe {
                libc::mmap(
                    want as *mut c_void,
                    4096,
                    libc::PROT_READ | libc::PROT_WRITE,
                    libc::MAP_PRIVATE | libc::MAP_ANON,
                    -1,
                    0,
                )
            };
            if p == libc::MAP_FAILED {
                continue;
            }
            if p as usize != want {
                // SAFETY: unmapping the page this call just mapped.
                unsafe { libc::munmap(p, 4096) };
                continue;
            }
            let ptr = (base + fd as usize) as *mut c_void;
            // SAFETY: ptr..ptr+4 lies inside the fresh page (fd < 4092).
            unsafe { std::ptr::write_unaligned(ptr as *mut i32, fd) };
            return Ok(DualFdArg { page: p, ptr });
        }
        Err("no free address for the dual-convention fd handle".into())
    }
    pub fn ptr(&self) -> *mut c_void {
        self.ptr
    }
    /// What a by-value implementation reads.
    pub fn by_value(&self) -> i32 {
        self.ptr as usize as u32 as i32
    }
    /// What a by-pointer implementation reads.
    pub fn by_pointer(&self) -> i32 {
        // SAFETY: ptr points 4 bytes inside our live page (see new).
        unsafe { std::ptr::read_unaligned(self.ptr as *const i32) }
    }
}

impl Drop for DualFdArg {
    fn drop(&mut self) {
        // SAFETY: the page was mapped by DualFdArg::new and is unmapped once.
        unsafe { libc::munmap(self.page, 4096) };
    }
}

/// Parse an amdgpu `pp_dpm_sclk` dump: the MHz of the line marked `*`.
pub fn parse_dpm_current_mhz(text: &str) -> Option<u64> {
    for line in text.lines() {
        if !line.contains('*') {
            continue;
        }
        let lower = line.to_ascii_lowercase();
        let pos = lower.find("mhz")?;
        let digits: String = lower[..pos].chars().rev().take_while(|c| c.is_ascii_digit()).collect();
        let digits: String = digits.chars().rev().collect();
        if let Ok(v) = digits.parse() {
            return Some(v);
        }
    }
    None
}

/// One `/proc/self/maps` line: (start, end, perms, file offset, path).
pub fn parse_maps_full(line: &str) -> Option<(usize, usize, String, u64, String)> {
    let mut it = line.split_ascii_whitespace();
    let (a, b) = it.next()?.split_once('-')?;
    let perms = it.next()?.to_string();
    let off = u64::from_str_radix(it.next()?, 16).ok()?;
    let _dev = it.next()?;
    let _inode = it.next()?;
    let path = it.collect::<Vec<_>>().join(" ");
    Some((usize::from_str_radix(a, 16).ok()?, usize::from_str_radix(b, 16).ok()?, perms, off, path))
}

pub fn fnv1a(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, b| (h ^ u64::from(*b)).wrapping_mul(0x0000_0100_0000_01b3))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::offset_of;

    #[test]
    fn descriptor_is_256_bytes_with_the_cuda_probe_offsets() {
        assert_eq!(std::mem::size_of::<Descriptor>(), 256);
        assert_eq!(offset_of!(Descriptor, seq), 8);
        assert_eq!(offset_of!(Descriptor, seed), 16);
        assert_eq!(offset_of!(Descriptor, len), 24);
        assert_eq!(offset_of!(Descriptor, t0_ns), 32);
        assert_eq!(offset_of!(Descriptor, block_id), 40);
        assert_eq!(offset_of!(Descriptor, offset), 48);
        assert_eq!(offset_of!(Descriptor, event), 56);
        assert_eq!(std::mem::size_of::<Ack>(), 64);
    }

    #[test]
    fn pad_puts_every_frame_on_a_256_boundary_inside_the_slot() {
        for size in [4096usize, 1 << 20, 64 << 20] {
            for ptr in (0x7f00_0000_0000usize..0x7f00_0000_0000 + 4096).step_by(8) {
                let off = frame_off(ptr);
                assert!(off < 256);
                assert_eq!((ptr + off) % 256, 0);
                assert!(off + DESC + size <= slot_len(size, false));
                assert!(off + DESC <= slot_len(size, true));
            }
        }
        assert_eq!(frame_off(0x1000), 0);
        assert_eq!(frame_off(0x1001), 255);
        assert_eq!(frame_off(0x10ff), 1);
    }

    #[test]
    fn pat_matches_the_cuda_probe_values() {
        // Hand-checked against the CUDA probe's pat(): i=0 seed=7919*1.
        let mut x = 0u32 ^ 7919u32.wrapping_mul(0x9E37_79B9);
        x ^= x >> 15;
        x = x.wrapping_mul(2246822519);
        x ^= x >> 13;
        assert_eq!(pat(0, 7919), x);
        assert_ne!(pat(1, 7919), pat(0, 7919));
        assert_ne!(pat(0, 7919), pat(0, 7919 * 2));
        assert_eq!(seed_for(0), 7919);
    }

    #[test]
    fn cpu_verify_counts_mismatched_words() {
        let seed = 42;
        let mut v: Vec<u8> = (0..1024u64).flat_map(|i| pat(i, seed).to_ne_bytes()).collect();
        assert_eq!(cpu_verify(&v, seed), 0);
        v[4 * 10] ^= 1;
        v[4 * 500 + 3] ^= 0x80;
        assert_eq!(cpu_verify(&v, seed), 2);
    }

    #[test]
    fn sizes_and_iteration_plan_match_the_cuda_probe() {
        let s = default_sizes();
        assert_eq!(s.len(), 8);
        assert_eq!(s[0], 4096);
        assert_eq!(s[7], 64 << 20);
        assert_eq!(iters_for(1 << 20), (100, 2000));
        assert_eq!(iters_for(4 << 20), (30, 500));
        assert_eq!(iters_for(64 << 20), (20, 300));
        assert_eq!(parse_sizes("4096,65536").unwrap(), vec![4096, 65536]);
        assert!(parse_sizes("3").is_err());
        assert_eq!(parse_sizes("all").unwrap(), s);
    }

    #[test]
    fn percentiles_are_nearest_rank() {
        let v: Vec<u64> = (1..=100).map(|x| x * 1000).collect();
        assert_eq!(pct_us(&v, 0.0), 1.0);
        assert_eq!(pct_us(&v, 0.5), 51.0);
        assert_eq!(pct_us(&v, 0.99), 99.0);
        assert_eq!(pct_us(&v, 1.0), 100.0);
    }

    #[test]
    fn rows_are_valid_json_with_escaped_strings() {
        let r = Row::new("skip").s("mode", "copy").s("reason", "a \"quoted\"\nline\\").u("size", 4096).f("x", 1.234).b("ok", true).i("n", -3);
        let line = r.line();
        assert!(line.starts_with("RESULT {\"align\":\"pad\",\"kind\":\"skip\""));
        assert!(line.ends_with('}'));
        assert!(line.contains("\"reason\":\"a \\\"quoted\\\"\\nline\\\\\""));
        assert!(line.contains("\"x\":1.23"));
        assert!(line.contains("\"n\":-3"));
        assert_eq!(json_str("\u{1}"), "\"\\u0001\"");
        assert!(Row::new("k").f("nan", f64::NAN).line().contains("\"nan\":null"));
    }

    #[test]
    fn the_dual_fd_handle_reads_the_same_fd_both_ways() {
        for fd in [0, 3, 7, 255, 1023, 4091] {
            let h = DualFdArg::new(fd).unwrap();
            assert_eq!(h.by_value(), fd, "by value");
            assert_eq!(h.by_pointer(), fd, "by pointer");
            assert_eq!(h.ptr() as usize >> 32 << 32, h.ptr() as usize - fd as usize);
        }
        assert!(DualFdArg::new(5000).is_err());
        assert!(DualFdArg::new(-1).is_err());
    }

    #[test]
    fn dpm_parse_reads_the_starred_level() {
        let t = "0: 500Mhz\n1: 1300Mhz *\n2: 2100Mhz\n";
        assert_eq!(parse_dpm_current_mhz(t), Some(1300));
        assert_eq!(parse_dpm_current_mhz("S: 1999Mhz *\n"), Some(1999));
        assert_eq!(parse_dpm_current_mhz("0: 500Mhz\n"), None);
    }

    #[test]
    fn maps_full_parse_keeps_offset_and_path() {
        let l = "7f0000000000-7f0000005000 r--s 00003000 00:1a 9 /dev/shm/iox2_x.data";
        let (a, b, p, off, path) = parse_maps_full(l).unwrap();
        assert_eq!((a, b, p.as_str(), off, path.as_str()), (0x7f00_0000_0000, 0x7f00_0000_5000, "r--s", 0x3000, "/dev/shm/iox2_x.data"));
        let anon = parse_maps_full("55d0c0000000-55d0c0021000 rw-p 00000000 00:00 0").unwrap();
        assert_eq!(anon.4, "");
        assert!(parse_maps_full("garbage").is_none());
    }

    #[test]
    fn devmsg_roundtrips_through_bytes() {
        let mut m = DevMsg::zeroed();
        m.kind = 3;
        m.bsize = 2 << 20;
        m.offset = 4096;
        m.ipc[63] = 9;
        let back: DevMsg = from_bytes(as_bytes(&m));
        assert_eq!((back.kind, back.bsize, back.offset, back.ipc[63]), (3, 2 << 20, 4096, 9));
    }
}
