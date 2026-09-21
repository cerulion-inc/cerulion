// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::builtin_interfaces::Time;
use native_ros2_messages::sensor_msgs::Image;

use crate::frame::{
    capture_len, publish_and_requeue, BYTES_PER_PIXEL, ENCODING, FRAME_HEIGHT, FRAME_WIDTH,
};

// Pure format, layout and bookkeeping helpers. No kernel calls: these run in
// the unit tests on every platform.
mod frame;

// The capture device itself. V4L2 is a Linux kernel API, so the ioctl/mmap
// face is compiled only on Linux (x86_64 or aarch64, where the hand-rolled
// request codes and struct layouts below are verified). Every other platform
// gets a stub whose every call reports `Unsupported`; the node still compiles
// there, and `external_source` turns the refusal into a `HostDriven` report so
// `cerulion graph run` refuses the launch and names this node.
#[cfg(all(
    target_os = "linux",
    any(target_arch = "x86_64", target_arch = "aarch64")
))]
#[path = "v4l2.rs"]
mod device;
#[cfg(not(all(
    target_os = "linux",
    any(target_arch = "x86_64", target_arch = "aarch64")
)))]
#[path = "unsupported.rs"]
mod device;

/// Environment variable naming the capture device. Read once in `init` from
/// the node's frozen env snapshot, so the choice is part of the recorded run.
const DEVICE_ENV: &str = "CERULION_V4L2_DEV";
/// The device used when `CERULION_V4L2_DEV` is unset.
const DEFAULT_DEVICE: &str = "/dev/video0";
/// `header.frame_id` on every published image.
const FRAME_ID: &str = "camera";

/// One `mmap`'d V4L2 streaming buffer, stored as plain numbers (an address as
/// `usize`, not a raw pointer) so the node stays `Send`.
#[derive(Debug, Clone, Copy, Default)]
pub struct MmapBuf {
    pub addr: usize,
    pub len: usize,
}

/// One dequeued frame: which buffer holds it and how many bytes the driver
/// wrote.
#[derive(Debug, Clone, Copy)]
pub struct Frame {
    pub index: u32,
    pub bytes_used: u32,
}

// A camera is a DRIVER node: nothing upstream in the graph fires it, so it
// declares `external` and hands the runtime the OS primitive to watch, here
// the device file descriptor. The runtime wakes the node when the fd is
// readable; `tick` dequeues one frame and publishes it.
#[cerulion_node(external)]
#[derive(Default)]
struct V4l2CameraNode {
    #[output]
    image: Image,
    // The device path, read in `init` from the frozen env snapshot.
    device: String,
    // The open device, `None` until `external_source` opens it. `RawFd` is a
    // handle the framework recognises, so a capture never records it.
    fd: Option<std::os::unix::io::RawFd>,
    // The kernel's capture buffers, mapped into this process. A handle to
    // rebuild, never state to restore: `external_source` maps them again on
    // the next live run.
    #[cerulion(reconstruct)]
    buffers: Vec<MmapBuf>,
    // Frames published so far.
    frames: u64,
}

#[cerulion_node_impl]
impl V4l2CameraNode {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.device = ctx.env_str(DEVICE_ENV, DEFAULT_DEVICE);
        Ok(())
    }

    // Queried once, at live-loop entry. Opens the camera, maps and queues the
    // streaming buffers, and hands the runtime the fd to watch. The runtime
    // never reads or closes that fd: this node owns the device and releases it
    // in `Drop`.
    fn external_source(&mut self) -> ExternalSource {
        match device::open_and_stream(&self.device) {
            Ok((fd, buffers)) => {
                let mapped = buffers.len();
                self.fd = Some(fd);
                self.buffers = buffers;
                let device = self.device.as_str();
                tracing::info!(device, fd, mapped, "v4l2 camera streaming");
                ExternalSource::Fd(fd)
            }
            Err(error) => {
                // No camera means nothing could ever fire this node. Reporting
                // `HostDriven` makes `cerulion graph run` refuse the launch and
                // name this node, instead of running a graph that never
                // publishes a frame.
                let device = self.device.as_str();
                tracing::error!(
                    device,
                    error = %error,
                    "v4l2_camera cannot capture, so it reports HostDriven and the \
                     launch is refused. Needs Linux (x86_64 or aarch64) and a V4L2 \
                     camera delivering 640x480 YUYV; set CERULION_V4L2_DEV to the \
                     device path if it is not /dev/video0, then relaunch"
                );
                ExternalSource::HostDriven
            }
        }
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        let Some(fd) = self.fd else {
            // `external_source` reported HostDriven, so the live run was refused
            // before any tick; nothing to drain.
            return Ok(());
        };

        // Dequeue ONE ready frame. The device is opened O_NONBLOCK, so a wake
        // with no frame ready (`Ok(None)`) returns without touching `image`: an
        // output a tick never writes is never loaned, so nothing is published
        // and nothing is discarded. A backlog drains over consecutive steps,
        // because the fd stays readable while frames remain.
        let Some(frame) = device::dequeue_frame(fd)? else {
            return Ok(());
        };

        // Whatever happens to the publish, the buffer goes back to the driver.
        publish_and_requeue(
            || self.publish_frame(frame),
            || device::requeue(fd, frame.index).map_err(NodeError::from),
        )?;

        self.frames += 1;
        if self.frames.is_multiple_of(30) {
            let frames = self.frames;
            tracing::info!(frames, "frames published");
        }
        Ok(())
    }

    fn publish_frame(&mut self, frame: Frame) -> Result<(), NodeError> {
        // Copy the buffer's two numbers out first, so no borrow of
        // `self.buffers` is held across the port writes below.
        let buffer = *self
            .buffers
            .get(frame.index as usize)
            .ok_or_else(|| NodeError::Logic("driver returned an unknown capture buffer".into()))?;
        let len = capture_len(
            frame.bytes_used as usize,
            buffer.len,
            FRAME_WIDTH * BYTES_PER_PIXEL,
            FRAME_HEIGHT,
        )?;

        // Fixed fields write straight into the loaned shared-memory slot.
        self.image.height = FRAME_HEIGHT;
        self.image.width = FRAME_WIDTH;
        self.image.is_bigendian = 0;
        self.image.step = FRAME_WIDTH * BYTES_PER_PIXEL;
        // The three variable fields (header, encoding, data) must each be
        // written before the frame publishes, in this order.
        let stamp = Time::from_ns(self.now_ns());
        self.image.header.stamp = stamp;
        self.image.header.frame_id = FRAME_ID;
        self.image.encoding = ENCODING;
        // The ONE copy: from the kernel's mmap'd capture buffer into the loaned
        // shared-memory slot. Everything downstream reads that slot in place.
        let dst = self.image.loan_data(len)?;
        dst.copy_from_slice(device::frame_bytes(&buffer, len)?);
        Ok(())
    }
}

// Teardown: stop streaming, unmap the buffers, close the fd. This is the only
// place the handed-over fd is closed, so the runtime's non-owning watch never
// races a stray close.
impl Drop for V4l2CameraNode {
    fn drop(&mut self) {
        if let Some(fd) = self.fd.take() {
            device::stop_and_close(fd, &self.buffers);
        }
    }
}

#[cfg(test)]
mod tests;
