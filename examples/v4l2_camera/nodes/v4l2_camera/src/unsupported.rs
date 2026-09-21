//! The non-Linux face of the capture device. V4L2 is a Linux kernel API, so
//! every entry point here reports `Unsupported`. The node compiles and loads
//! on this platform, `external_source` reports `HostDriven`, and
//! `cerulion graph run` refuses the launch naming this node and the reason.

use std::io;
use std::os::unix::io::RawFd;

use crate::{Frame, MmapBuf};

fn unsupported() -> io::Error {
    io::Error::new(
        io::ErrorKind::Unsupported,
        "V4L2 capture needs Linux on x86_64 or aarch64; this platform has no capture path",
    )
}

pub fn open_and_stream(_device: &str) -> io::Result<(RawFd, Vec<MmapBuf>)> {
    Err(unsupported())
}

pub fn dequeue_frame(_fd: RawFd) -> io::Result<Option<Frame>> {
    Err(unsupported())
}

pub fn requeue(_fd: RawFd, _index: u32) -> io::Result<()> {
    Err(unsupported())
}

pub fn frame_bytes(_buffer: &MmapBuf, _len: usize) -> io::Result<&[u8]> {
    Err(unsupported())
}

pub fn stop_and_close(_fd: RawFd, _buffers: &[MmapBuf]) {}
