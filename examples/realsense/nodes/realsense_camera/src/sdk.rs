//! The librealsense2 face of the driver, compiled with the `realsense`
//! feature: the official Intel SDK through the `realsense-rust` bindings.
//!
//! `open` runs once on the live-loop thread (from `external_source`): it
//! enumerates devices, starts a pipeline for 640x480 color (RGB8) and depth
//! (Z16) at 30 fps, and checks the camera's depth unit. `into_waiter` then
//! moves the started pipeline into the closure the runtime drives on its
//! helper thread: each call waits (bounded) for one frameset and offers it to
//! the hand-over slot, ringing the node's doorbell when it did.

use std::collections::HashSet;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::prelude::tracing;
use realsense_rust::config::Config;
use realsense_rust::context::Context;
use realsense_rust::frame::{ColorFrame, CompositeFrame, DepthFrame, ImageFrame};
use realsense_rust::kind::{Rs2CameraInfo, Rs2Format, Rs2Option, Rs2StreamKind};
use realsense_rust::pipeline::{ActivePipeline, FrameWaitError, InactivePipeline};

use crate::frame::{Plane, Slot, DEPTH_UNIT_M};

/// How long one helper-thread wait blocks before giving up and returning
/// `false`. Bounded so the runtime can stop the helper between calls.
const WAIT_TIMEOUT: Duration = Duration::from_millis(100);

/// A synchronized color + depth frameset, owned until `tick` drops it.
pub struct Frameset(CompositeFrame);

/// The color plane of a frameset, owning its SDK frame handle.
pub struct ColorPicture(ColorFrame);

/// The depth plane of a frameset, owning its SDK frame handle.
pub struct DepthPicture(DepthFrame);

impl Frameset {
    pub fn color(&self) -> Option<ColorPicture> {
        self.0
            .frames_of_type::<ColorFrame>()
            .pop()
            .map(ColorPicture)
    }

    pub fn depth(&self) -> Option<DepthPicture> {
        self.0
            .frames_of_type::<DepthFrame>()
            .pop()
            .map(DepthPicture)
    }
}

impl ColorPicture {
    /// The plane's dimensions and pixel bytes, borrowed from the SDK's frame
    /// buffer for as long as this picture lives.
    pub fn plane(&self) -> Plane<'_> {
        plane_of(&self.0)
    }
}

impl DepthPicture {
    /// The plane's dimensions and depth counts, borrowed from the SDK's frame
    /// buffer for as long as this picture lives.
    pub fn plane(&self) -> Plane<'_> {
        plane_of(&self.0)
    }
}

fn plane_of<K>(frame: &ImageFrame<K>) -> Plane<'_> {
    let len = frame.get_data_size();
    // SAFETY: `get_data` is the SDK's pointer to this frame's pixel buffer,
    // valid and unchanged for the frame's lifetime, and `get_data_size` is
    // that buffer's length in bytes. The borrow is tied to `frame`, so the
    // slice cannot outlive it.
    let bytes = unsafe {
        let data = frame.get_data() as *const std::os::raw::c_void;
        std::slice::from_raw_parts(data.cast::<u8>(), len)
    };
    Plane {
        width: frame.width() as u32,
        height: frame.height() as u32,
        stride: frame.stride() as u32,
        bytes,
    }
}

/// A started camera: the active pipeline plus what it identified as.
pub struct Camera {
    pipeline: ActivePipeline,
    name: String,
    serial: String,
}

/// Start streaming from the first RealSense the SDK finds. Every failure is
/// a `String` naming the SDK's own reason, because the caller turns it into
/// the launch refusal a user reads.
pub fn open(width: u32, height: u32, fps: u32) -> Result<Camera, String> {
    let context = Context::new().map_err(|error| error.to_string())?;
    let devices = context.query_devices(HashSet::new()).len();

    let mut config = Config::new();
    config
        .enable_stream(
            Rs2StreamKind::Depth,
            None,
            width as usize,
            height as usize,
            Rs2Format::Z16,
            fps as usize,
        )
        .map_err(|error| error.to_string())?;
    config
        .enable_stream(
            Rs2StreamKind::Color,
            None,
            width as usize,
            height as usize,
            Rs2Format::Rgb8,
            fps as usize,
        )
        .map_err(|error| error.to_string())?;

    let pipeline = InactivePipeline::try_from(&context).map_err(|error| error.to_string())?;
    let pipeline = pipeline.start(Some(config)).map_err(|error| {
        format!("{error} (librealsense2 enumerated {devices} RealSense device(s))")
    })?;

    // The published depth is `16UC1` millimetres, which is only true when the
    // camera's depth unit is the 0.001 m default. Refuse any other unit rather
    // than publish counts under the wrong label.
    let device = pipeline.profile().device();
    let unit = device
        .sensors()
        .iter()
        .find_map(|sensor| sensor.get_option(Rs2Option::DepthUnits))
        .ok_or_else(|| "no sensor on this device reports a depth unit".to_string())?;
    if (unit - DEPTH_UNIT_M).abs() > 1e-7 {
        return Err(format!(
            "the camera's depth unit is {unit} m per count; this example publishes 16UC1 \
             millimetres and needs the 0.001 m default (set Depth Units back to 0.001 in \
             realsense-viewer)"
        ));
    }

    let info = |kind| {
        device
            .info(kind)
            .map(|value| value.to_string_lossy().into_owned())
            .unwrap_or_default()
    };
    let name = info(Rs2CameraInfo::Name);
    let serial = info(Rs2CameraInfo::SerialNumber);
    Ok(Camera {
        pipeline,
        name,
        serial,
    })
}

impl Camera {
    /// The device as the SDK names it, for the startup log line.
    pub fn describe(&self) -> String {
        format!("{} (serial {})", self.name, self.serial)
    }

    /// Move the pipeline into the blocking-source closure. Each call waits
    /// for one frameset and returns `true` when it offered one to `slot`
    /// (the doorbell), `false` on a timeout or an SDK error.
    pub fn into_waiter(
        mut self,
        slot: Arc<Mutex<Slot<Frameset>>>,
    ) -> Box<dyn FnMut() -> bool + Send + 'static> {
        // One warning per error regime, not one per failed wait.
        let mut failing = false;
        Box::new(move || match self.pipeline.wait(Some(WAIT_TIMEOUT)) {
            Ok(frames) => {
                failing = false;
                slot.lock()
                    .unwrap_or_else(|poisoned| poisoned.into_inner())
                    .offer(Frameset(frames));
                true
            }
            Err(FrameWaitError::DidTimeoutBeforeFrameArrival) => false,
            Err(error) => {
                if !failing {
                    failing = true;
                    tracing::warn!(error = %error, "realsense wait failed; retrying");
                }
                false
            }
        })
    }
}
