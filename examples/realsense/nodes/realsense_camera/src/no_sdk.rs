//! The face of the driver compiled WITHOUT the `realsense` feature: no SDK is
//! linked, so there is no camera. `open` refuses with the install and rebuild
//! instructions; `external_source` turns that into a `HostDriven` report and
//! `cerulion graph run` refuses the launch naming this node. The frame types
//! are uninhabited: nothing can ever put one in the slot.

use std::sync::{Arc, Mutex};

use crate::frame::{Plane, Slot};

/// Why this build cannot capture, and what to do about it. The same text the
/// README carries under "Install the SDK".
pub const NOT_BUILT: &str = "realsense_camera was built without the `realsense` feature, so it \
    has no camera driver. Install librealsense2 (macOS: `brew install librealsense`; Ubuntu: \
    Intel's apt repository or the `ros-<distro>-librealsense2` package, see README.md), then \
    rebuild with `cerulion node build realsense_camera --release`, which enables the feature \
    when pkg-config finds `realsense2`";

/// A frameset from the SDK. Uninhabited in this build.
pub enum Frameset {}

/// The color plane of a frameset. Uninhabited in this build.
pub enum ColorPicture {}

/// The depth plane of a frameset. Uninhabited in this build.
pub enum DepthPicture {}

impl Frameset {
    pub fn color(&self) -> Option<ColorPicture> {
        match *self {}
    }

    pub fn depth(&self) -> Option<DepthPicture> {
        match *self {}
    }
}

impl ColorPicture {
    pub fn plane(&self) -> Plane<'_> {
        match *self {}
    }
}

impl DepthPicture {
    pub fn plane(&self) -> Plane<'_> {
        match *self {}
    }
}

/// A started camera. Uninhabited in this build: `open` never succeeds.
pub enum Camera {}

pub fn open(_width: u32, _height: u32, _fps: u32) -> Result<Camera, String> {
    Err(NOT_BUILT.to_string())
}

impl Camera {
    pub fn describe(&self) -> String {
        match *self {}
    }

    pub fn into_waiter(
        self,
        _slot: Arc<Mutex<Slot<Frameset>>>,
    ) -> Box<dyn FnMut() -> bool + Send + 'static> {
        match self {}
    }
}
