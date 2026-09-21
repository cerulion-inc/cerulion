//! Minimal hand-rolled V4L2 (Video4Linux2) MMAP-streaming FFI: the ioctl
//! request codes, the streaming structs (`v4l2_requestbuffers`,
//! `v4l2_buffer`) and thin `libc` wrappers. No extra dependency.
//!
//! The ioctl request numbers are computed from the struct sizes with the
//! kernel's `_IOC` formula, so they stay correct as long as the struct layouts
//! match the kernel's, which is why the layouts are reproduced field for field
//! and pinned by the size assertions below.

use std::io;
use std::os::unix::io::RawFd;

use cerulion_core::prelude::tracing;

use crate::frame::{g_fmt_request, validate_format, V4l2Format, V4l2PixFormat};
use crate::{Frame, MmapBuf};

const V4L2_BUF_TYPE_VIDEO_CAPTURE: u32 = 1;
const V4L2_MEMORY_MMAP: u32 = 1;
const BUFFER_COUNT: u32 = 4;

// `_IOC(dir, type, nr, size) = (dir<<30) | (size<<16) | (type<<8) | nr`
//   dir:  1 = write (user to kernel), 2 = read (kernel to user), 3 = both
//   type: the driver magic byte, 'V' for V4L2
const IOC_WRITE: libc::c_ulong = 1;
const IOC_READ: libc::c_ulong = 2;
const IOC_MAGIC_V: libc::c_ulong = b'V' as libc::c_ulong;

const fn ioc(dir: libc::c_ulong, nr: libc::c_ulong, size: usize) -> libc::c_ulong {
    (dir << 30) | (IOC_MAGIC_V << 8) | nr | ((size as libc::c_ulong) << 16)
}

// VIDIOC_{REQBUFS=8, QUERYBUF=9, QBUF=15, DQBUF=17} are _IOWR(...);
// STREAMON=18 / STREAMOFF=19 are _IOW('V', n, int).
fn vidioc_reqbufs() -> libc::c_ulong {
    ioc(
        IOC_READ | IOC_WRITE,
        8,
        std::mem::size_of::<V4l2RequestBuffers>(),
    )
}
fn vidioc_querybuf() -> libc::c_ulong {
    ioc(IOC_READ | IOC_WRITE, 9, std::mem::size_of::<V4l2Buffer>())
}
fn vidioc_qbuf() -> libc::c_ulong {
    ioc(IOC_READ | IOC_WRITE, 15, std::mem::size_of::<V4l2Buffer>())
}
fn vidioc_dqbuf() -> libc::c_ulong {
    ioc(IOC_READ | IOC_WRITE, 17, std::mem::size_of::<V4l2Buffer>())
}
fn vidioc_streamon() -> libc::c_ulong {
    ioc(IOC_WRITE, 18, std::mem::size_of::<i32>())
}
fn vidioc_streamoff() -> libc::c_ulong {
    ioc(IOC_WRITE, 19, std::mem::size_of::<i32>())
}

// `#[repr(C)]` mirrors of the kernel UAPI structs. Many fields are written but
// never read, or are pure layout: they exist so the Rust layout matches the
// kernel's byte for byte, which is what makes the size-derived request codes
// above correct.

/// `struct v4l2_requestbuffers`: 20 bytes.
#[repr(C)]
#[derive(Default)]
#[allow(dead_code)]
struct V4l2RequestBuffers {
    count: u32,
    type_: u32,
    memory: u32,
    capabilities: u32,
    flags: u8,
    reserved: [u8; 3],
}

/// `struct timeval` on 64-bit Linux (both fields are `long`).
#[repr(C)]
#[derive(Default, Clone, Copy)]
#[allow(dead_code)]
struct Timeval {
    tv_sec: i64,
    tv_usec: i64,
}

/// `struct v4l2_timecode`: 16 bytes.
#[repr(C)]
#[derive(Default, Clone, Copy)]
#[allow(dead_code)]
struct V4l2Timecode {
    type_: u32,
    flags: u32,
    frames: u8,
    seconds: u8,
    minutes: u8,
    hours: u8,
    userbits: [u8; 4],
}

/// `struct v4l2_buffer`: 88 bytes on 64-bit. The `m` union (offset / userptr /
/// planes* / fd) is a `u64`; for MMAP the low 32 bits are `m.offset`.
#[repr(C)]
#[derive(Default)]
#[allow(dead_code)]
struct V4l2Buffer {
    index: u32,
    type_: u32,
    bytesused: u32,
    flags: u32,
    field: u32,
    timestamp: Timeval,
    timecode: V4l2Timecode,
    sequence: u32,
    memory: u32,
    m: u64,
    length: u32,
    reserved2: u32,
    request_fd: i32,
}

// If a field or padding is wrong the sizes drift and the computed request
// codes no longer match the kernel's.
const _: () = assert!(std::mem::size_of::<V4l2RequestBuffers>() == 20);
const _: () = assert!(std::mem::size_of::<V4l2Buffer>() == 88);

/// Retry-on-`EINTR` `ioctl` wrapper. `arg` points at the request struct.
///
/// SAFETY: the caller guarantees `arg` points at a valid, correctly sized
/// object for `request`, and `fd` is a live V4L2 device fd.
unsafe fn xioctl(fd: RawFd, request: libc::c_ulong, arg: *mut libc::c_void) -> io::Result<()> {
    loop {
        let rc = libc::ioctl(fd, request, arg);
        if rc == -1 {
            let err = io::Error::last_os_error();
            if err.raw_os_error() == Some(libc::EINTR) {
                continue;
            }
            return Err(err);
        }
        return Ok(());
    }
}

fn check_capture_format(fd: RawFd) -> io::Result<()> {
    let mut format = V4l2Format {
        type_: V4L2_BUF_TYPE_VIDEO_CAPTURE,
        padding: 0,
        pix: V4l2PixFormat::default(),
        remainder: [0; 152],
    };
    // SAFETY: `fd` is the open capture descriptor; this fully initialized,
    // 208-byte, 8-aligned buffer matches the 64-bit v4l2_format layout. G_FMT
    // reads and writes only that buffer and retains no pointer.
    unsafe {
        xioctl(
            fd,
            g_fmt_request(),
            &mut format as *mut _ as *mut libc::c_void,
        )?;
    }
    validate_format(&format.pix)
}

/// Open `device`, check its format, request, map and enqueue the streaming
/// buffers, and `STREAMON`. Returns the fd and the mapped buffers.
pub fn open_and_stream(device: &str) -> io::Result<(RawFd, Vec<MmapBuf>)> {
    let c_dev = std::ffi::CString::new(device)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "device path has a NUL byte"))?;

    // O_NONBLOCK so DQBUF never blocks the single live-loop thread.
    // SAFETY: `c_dev` is a valid NUL-terminated path for the call's duration.
    let fd = unsafe { libc::open(c_dev.as_ptr(), libc::O_RDWR | libc::O_NONBLOCK) };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    let result = setup_streaming(fd);
    if result.is_err() {
        // SAFETY: `fd` is the just-opened, not-yet-handed-over descriptor.
        unsafe { libc::close(fd) };
    }
    result.map(|buffers| (fd, buffers))
}

fn setup_streaming(fd: RawFd) -> io::Result<Vec<MmapBuf>> {
    check_capture_format(fd)?;

    // 1. VIDIOC_REQBUFS: ask for BUFFER_COUNT MMAP buffers.
    let mut req = V4l2RequestBuffers {
        count: BUFFER_COUNT,
        type_: V4L2_BUF_TYPE_VIDEO_CAPTURE,
        memory: V4L2_MEMORY_MMAP,
        ..Default::default()
    };
    // SAFETY: `&mut req` is a live, correctly sized `v4l2_requestbuffers`.
    unsafe {
        xioctl(
            fd,
            vidioc_reqbufs(),
            &mut req as *mut _ as *mut libc::c_void,
        )?
    };
    if req.count < 2 {
        return Err(io::Error::other(
            "driver granted too few buffers for MMAP streaming",
        ));
    }

    // 2. VIDIOC_QUERYBUF + mmap each buffer.
    let mut buffers = Vec::with_capacity(req.count as usize);
    for index in 0..req.count {
        let mut buf = V4l2Buffer {
            type_: V4L2_BUF_TYPE_VIDEO_CAPTURE,
            memory: V4L2_MEMORY_MMAP,
            index,
            ..Default::default()
        };
        // SAFETY: `&mut buf` is a live, correctly sized `v4l2_buffer`.
        let queried = unsafe {
            xioctl(
                fd,
                vidioc_querybuf(),
                &mut buf as *mut _ as *mut libc::c_void,
            )
        };
        if let Err(error) = queried {
            unmap_all(&buffers);
            return Err(error);
        }
        let offset = (buf.m & 0xffff_ffff) as libc::off_t;
        let len = buf.length as usize;
        // SAFETY: standard V4L2 MMAP map of `len` bytes at the buffer's
        // `m.offset`; `fd` is the streaming device.
        let addr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                offset,
            )
        };
        if addr == libc::MAP_FAILED {
            let error = io::Error::last_os_error();
            unmap_all(&buffers);
            return Err(error);
        }
        buffers.push(MmapBuf {
            addr: addr as usize,
            len,
        });
    }

    // 3. VIDIOC_QBUF each buffer (hand it to the driver to fill).
    for index in 0..buffers.len() as u32 {
        if let Err(error) = requeue(fd, index) {
            unmap_all(&buffers);
            return Err(error);
        }
    }

    // 4. VIDIOC_STREAMON.
    let mut typ: i32 = V4L2_BUF_TYPE_VIDEO_CAPTURE as i32;
    // SAFETY: `&mut typ` is a live `int` (the buffer-type argument).
    if let Err(error) = unsafe {
        xioctl(
            fd,
            vidioc_streamon(),
            &mut typ as *mut _ as *mut libc::c_void,
        )
    } {
        unmap_all(&buffers);
        return Err(error);
    }
    Ok(buffers)
}

/// `VIDIOC_DQBUF` one frame. `Ok(None)` on `EAGAIN` (no frame ready).
pub fn dequeue_frame(fd: RawFd) -> io::Result<Option<Frame>> {
    let mut buf = V4l2Buffer {
        type_: V4L2_BUF_TYPE_VIDEO_CAPTURE,
        memory: V4L2_MEMORY_MMAP,
        ..Default::default()
    };
    // SAFETY: `&mut buf` is a live, correctly sized `v4l2_buffer`.
    match unsafe { xioctl(fd, vidioc_dqbuf(), &mut buf as *mut _ as *mut libc::c_void) } {
        Ok(()) => Ok(Some(Frame {
            index: buf.index,
            bytes_used: buf.bytesused,
        })),
        Err(error) if error.raw_os_error() == Some(libc::EAGAIN) => Ok(None),
        Err(error) => Err(error),
    }
}

/// `VIDIOC_QBUF`: hand buffer `index` back to the driver.
pub fn requeue(fd: RawFd, index: u32) -> io::Result<()> {
    let mut buf = V4l2Buffer {
        type_: V4L2_BUF_TYPE_VIDEO_CAPTURE,
        memory: V4L2_MEMORY_MMAP,
        index,
        ..Default::default()
    };
    // SAFETY: `&mut buf` is a live, correctly sized `v4l2_buffer`.
    unsafe { xioctl(fd, vidioc_qbuf(), &mut buf as *mut _ as *mut libc::c_void) }
}

/// The first `len` bytes of a mapped buffer, for a frame the driver has just
/// dequeued (so the kernel is not writing it).
pub fn frame_bytes(buffer: &MmapBuf, len: usize) -> io::Result<&[u8]> {
    if buffer.addr == 0 || len > buffer.len {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "frame length exceeds the mapped capture buffer",
        ));
    }
    // SAFETY: `buffer.addr` is the base of a live `mmap` of `buffer.len` bytes
    // (from VIDIOC_QUERYBUF + mmap), currently dequeued, and `len <= buffer.len`.
    Ok(unsafe { std::slice::from_raw_parts(buffer.addr as *const u8, len) })
}

/// Teardown: `STREAMOFF`, unmap, close. Best effort: logs, never panics.
pub fn stop_and_close(fd: RawFd, buffers: &[MmapBuf]) {
    let mut typ: i32 = V4L2_BUF_TYPE_VIDEO_CAPTURE as i32;
    // SAFETY: `&mut typ` is a live `int`; `fd` is the streaming device.
    if let Err(error) = unsafe {
        xioctl(
            fd,
            vidioc_streamoff(),
            &mut typ as *mut _ as *mut libc::c_void,
        )
    } {
        tracing::warn!(error = %error, "VIDIOC_STREAMOFF failed during teardown");
    }
    unmap_all(buffers);
    // SAFETY: teardown-only close of the device fd this node owns; runs after
    // the runtime has dropped its non-owning watch.
    unsafe { libc::close(fd) };
}

fn unmap_all(buffers: &[MmapBuf]) {
    for buffer in buffers {
        if buffer.addr != 0 {
            // SAFETY: `(addr, len)` is a mapping this module created with `mmap`.
            unsafe { libc::munmap(buffer.addr as *mut libc::c_void, buffer.len) };
        }
    }
}
