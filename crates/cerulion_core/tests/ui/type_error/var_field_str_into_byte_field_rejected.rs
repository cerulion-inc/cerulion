//! Regression guard: assigning a string-shaped RHS
//! to a byte-typed (`uint8[]`) variable field MUST be a compile error, not a
//! silent UTF-8 write.
//!
//! `sensor_msgs::Image::data` is `uint8[]`, so its generated shim
//! (`__cer_assign_data`, which delegates to `set_data`) takes `&[u8]`. A
//! `&str` does not deref-coerce to `&[u8]`, so the shimmed `=` rewrite must
//! reject `self.image.data = "..."` at compile time. (An earlier
//! `AsRef::as_ref` codegen briefly accepted it because `str: AsRef<[u8]>`;
//! this fixture pins the loud-failure contract so that regression cannot
//! return.)
#![allow(unexpected_cfgs)]

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(external)]
struct StrIntoByteField {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl StrIntoByteField {
    fn tick(&mut self) -> Result<(), cerulion_core::error::NodeError> {
        self.image.data = "should-not-compile";
        Ok(())
    }

    // External nodes require an `external_source` method. Added
    // AFTER `tick` so the pinned E0308 line number (the `self.image.data = ...`
    // site) does not shift.
    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

fn main() {}
