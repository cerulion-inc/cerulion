// SPDX-License-Identifier: AGPL-3.0-only
//! Transforms real input vectors for downstream compiler compatibility checks.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::node_abi::Bytes;
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node]
#[derive(Default)]
pub struct Transform {
    #[input(trigger)]
    input: Vector3,
    #[output]
    output: Vector3,
}

#[cerulion_node_impl]
impl Transform {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.output.x = self.input.x + 10.0;
        self.output.y = self.input.y * 2.0;
        self.output.z = self.input.z - 1.0;
        Ok(())
    }
}

/// Actual compiler provenance from this library's core build script. Borrowed
/// immutable bytes stay live until this library is unloaded; caller frees none.
#[stabby::export]
pub extern "C" fn cerulion_test_compiler_fingerprint() -> Bytes {
    let fingerprint = cerulion_core::RUSTC_FINGERPRINT;
    Bytes {
        ptr: fingerprint.as_ptr(),
        len: fingerprint.len() as u64,
    }
}
