// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use std::sync::{Arc, Mutex};

use cerulion_core::prelude::*;
use native_ros2_messages::builtin_interfaces::Time;
use native_ros2_messages::sensor_msgs::Image;

use crate::frame::{
    check_plane, Slot, COLOR_BYTES_PER_PIXEL, COLOR_ENCODING, COLOR_FRAME_ID,
    DEPTH_BYTES_PER_PIXEL, DEPTH_ENCODING, DEPTH_FRAME_ID, FPS, HEIGHT, WIDTH,
};

// Pure helpers: stream shape, encodings, plane checks, the hand-over slot.
mod frame;

// The camera driver. With the `realsense` feature the `sdk` face links the
// official Intel SDK (librealsense2) through realsense-rust; without it the
// `no_sdk` face compiles the same interface with no camera behind it, so the
// node builds everywhere and `cerulion graph run` refuses it loudly at launch.
#[cfg(feature = "realsense")]
#[path = "sdk.rs"]
mod driver;
#[cfg(not(feature = "realsense"))]
#[path = "no_sdk.rs"]
mod driver;

// A camera is a DRIVER node: nothing upstream in the graph fires it, so it
// declares `external`. librealsense2 has no file descriptor to watch, only a
// blocking wait, so this node hands the runtime a `Blocking` source: the
// runtime drives that closure on a helper thread, the closure offers each
// frameset to the slot and rings the node's doorbell, and `tick` copies the
// frameset's two planes into the loaned shared-memory slots.
#[cerulion_node(external)]
#[derive(Default)]
struct RealsenseCameraNode {
    #[output]
    color: Image,
    #[output]
    depth: Image,
    // The hand-over slot shared with the helper thread. A handle to rebuild,
    // never state to restore: `external_source` starts a fresh pipeline on
    // the next live run.
    #[cerulion(reconstruct)]
    slot: Arc<Mutex<Slot<driver::Frameset>>>,
    // Framesets published so far.
    frames: u64,
    // Framesets the helper replaced before `tick` took them (latest wins).
    overwritten: u64,
}

impl RealsenseCameraNode {
    // Lives in this plain impl block, outside `#[cerulion_node_impl]`, so the
    // helper-thread plumbing stays out of the deterministic tick surface.
    fn blocking_source(&self, camera: driver::Camera) -> ExternalSource {
        ExternalSource::Blocking(camera.into_waiter(Arc::clone(&self.slot)))
    }
}

#[cerulion_node_impl]
impl RealsenseCameraNode {
    // Queried once, at live-loop entry. Starts the pipeline and hands the
    // runtime the blocking wait to drive.
    fn external_source(&mut self) -> ExternalSource {
        match driver::open(WIDTH, HEIGHT, FPS) {
            Ok(camera) => {
                let device = camera.describe();
                tracing::info!(
                    device,
                    width = WIDTH,
                    height = HEIGHT,
                    fps = FPS,
                    "realsense camera streaming"
                );
                self.blocking_source(camera)
            }
            Err(reason) => {
                // No camera means nothing could ever fire this node. Reporting
                // `HostDriven` makes `cerulion graph run` refuse the launch and
                // name this node, instead of running a graph that never
                // publishes a frame.
                tracing::error!(
                    reason,
                    "realsense_camera cannot capture, so it reports HostDriven and the \
                     launch is refused. Connect a RealSense camera over USB 3 and relaunch"
                );
                ExternalSource::HostDriven
            }
        }
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        // Take the latest frameset. Several doorbell rings before one step
        // coalesce into one tick, so a tick can also find the slot empty; then
        // nothing is written, so nothing is loaned or published.
        let taken = {
            let mut slot = self
                .slot
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            self.overwritten = slot.overwritten();
            slot.take()
        };
        let Some(frameset) = taken else {
            return Ok(());
        };

        let stamp = Time::from_ns(self.now_ns());

        let color = frameset
            .color()
            .ok_or_else(|| NodeError::Logic("frameset carries no color frame".into()))?;
        let plane = color.plane();
        check_plane(&plane, COLOR_BYTES_PER_PIXEL)?;
        // Fixed fields write straight into the loaned shared-memory slot; the
        // three variable fields (header, encoding, data) follow in this order.
        self.color.height = plane.height;
        self.color.width = plane.width;
        self.color.is_bigendian = 0;
        self.color.step = plane.stride;
        self.color.header.stamp = stamp;
        self.color.header.frame_id = COLOR_FRAME_ID;
        self.color.encoding = COLOR_ENCODING;
        // The ONE copy of the color pixels: from the SDK's frame buffer into
        // the loaned shared-memory slot.
        self.color
            .loan_data(plane.bytes.len())?
            .copy_from_slice(plane.bytes);

        let depth = frameset
            .depth()
            .ok_or_else(|| NodeError::Logic("frameset carries no depth frame".into()))?;
        let plane = depth.plane();
        check_plane(&plane, DEPTH_BYTES_PER_PIXEL)?;
        self.depth.height = plane.height;
        self.depth.width = plane.width;
        self.depth.is_bigendian = 0;
        self.depth.step = plane.stride;
        self.depth.header.stamp = stamp;
        self.depth.header.frame_id = DEPTH_FRAME_ID;
        self.depth.encoding = DEPTH_ENCODING;
        // The ONE copy of the depth counts (16UC1, millimetres).
        self.depth
            .loan_data(plane.bytes.len())?
            .copy_from_slice(plane.bytes);

        self.frames += 1;
        if self.frames.is_multiple_of(30) {
            let frames = self.frames;
            let overwritten = self.overwritten;
            tracing::info!(frames, overwritten, "framesets published");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
