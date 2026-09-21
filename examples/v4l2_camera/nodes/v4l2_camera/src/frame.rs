//! Pure helpers: the one capture format this example accepts, the kernel
//! format struct it reads back, and two bookkeeping rules. Nothing here talks
//! to a device, so all of it runs in the unit tests on every platform.
//!
//! The format helpers are called only by the Linux capture face; elsewhere
//! the unit tests are their only caller, so the dead-code lint is quiet there.
#![cfg_attr(
    not(all(
        target_os = "linux",
        any(target_arch = "x86_64", target_arch = "aarch64")
    )),
    allow(dead_code)
)]

use std::io;

/// This example supports one explicit format. Startup reads the active format
/// with `VIDIOC_G_FMT` and refuses anything else before capture begins.
pub const FRAME_WIDTH: u32 = 640;
pub const FRAME_HEIGHT: u32 = 480;
/// YUYV packs two pixels into four bytes: two bytes per pixel.
pub const BYTES_PER_PIXEL: u32 = 2;
/// The ROS encoding name for packed YUYV (`sensor_msgs/image_encodings`).
pub const ENCODING: &str = "yuv422_yuy2";

/// `VIDIOC_G_FMT`: `_IOWR('V', 4, struct v4l2_format)`.
pub const fn g_fmt_request() -> u64 {
    (3 << 30) | ((b'V' as u64) << 8) | 4 | ((std::mem::size_of::<V4l2Format>() as u64) << 16)
}

/// Single-planar `v4l2_pix_format` (48 bytes in linux/videodev2.h).
#[repr(C)]
#[derive(Default)]
#[allow(dead_code)] // Every field is required by the kernel UAPI layout.
pub struct V4l2PixFormat {
    pub width: u32,
    pub height: u32,
    pub pixel_format: u32,
    pub field: u32,
    pub bytes_per_line: u32,
    pub size_image: u32,
    pub colorspace: u32,
    pub private: u32,
    pub flags: u32,
    pub ycbcr_enc: u32,
    pub quantization: u32,
    pub transfer_function: u32,
}

/// 64-bit `v4l2_format`: type, alignment padding, and its 200-byte union.
/// Only the single-planar pix member is read. Keeping the rest as initialized
/// bytes avoids reading a Rust union or padding.
#[repr(C, align(8))]
#[allow(dead_code)] // The entire UAPI union storage must exist.
pub struct V4l2Format {
    pub type_: u32,
    pub padding: u32,
    pub pix: V4l2PixFormat,
    pub remainder: [u8; 152],
}

const _: () = assert!(std::mem::size_of::<V4l2PixFormat>() == 48);
const _: () = assert!(std::mem::size_of::<V4l2Format>() == 208);
const _: () = assert!(std::mem::offset_of!(V4l2Format, pix) == 8);

/// Accept exactly 640x480 YUYV with a 1280-byte row stride.
pub fn validate_format(format: &V4l2PixFormat) -> io::Result<()> {
    if format.width != FRAME_WIDTH
        || format.height != FRAME_HEIGHT
        || format.pixel_format != u32::from_le_bytes(*b"YUYV")
        || format.bytes_per_line != FRAME_WIDTH * BYTES_PER_PIXEL
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "camera must use 640x480 YUYV with a 1280-byte row stride",
        ));
    }
    Ok(())
}

/// Validate the captured payload before exposing it as a complete ROS Image:
/// the driver must have written exactly `step * height` bytes, all inside the
/// mapped buffer.
pub fn capture_len(
    bytes_used: usize,
    mapped_len: usize,
    step: u32,
    height: u32,
) -> io::Result<usize> {
    let expected = usize::try_from(u64::from(step) * u64::from(height))
        .map_err(|_| io::Error::other("capture dimensions exceed addressable memory"))?;
    if bytes_used > mapped_len || bytes_used != expected {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "capture payload does not match the image dimensions or mapped buffer",
        ));
    }
    Ok(bytes_used)
}

/// A failed publish must still return the dequeued buffer to the driver, or
/// the camera runs out of buffers and stops delivering frames.
pub fn publish_and_requeue<E>(
    publish: impl FnOnce() -> Result<(), E>,
    requeue: impl FnOnce() -> Result<(), E>,
) -> Result<(), E> {
    let result = publish();
    requeue()?;
    result
}
