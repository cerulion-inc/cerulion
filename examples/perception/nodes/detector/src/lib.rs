// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

// The shared generated schema includes writers and snapshots this node does not use.
#[allow(dead_code)]
mod detections {
    include!(concat!(env!("OUT_DIR"), "/detections_schema.rs"));
}
use detections::DetectionArray;

mod algorithm;

// The perturbed twin changes only this x-translation. Moving a 20×20 box
// by 8 px gives IoU 240/560, below the tutorial's 0.5 acceptance floor.
const DETECTION_SHIFT: f64 = 0.0;

#[cerulion_node]
#[derive(Default)]
struct DetectorNode {
    #[input(trigger)]
    image_raw: Image,
    #[output]
    detections: DetectionArray,
}

#[cerulion_node_impl]
impl DetectorNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let regions = algorithm::detect(
            self.image_raw.data(),
            self.image_raw.width,
            self.image_raw.height,
            self.image_raw.step,
            self.image_raw
                .encoding()
                .map_err(|error| NodeError::InvalidInput {
                    input: "image_raw".into(),
                    reason: error.to_string(),
                })?,
        )?;
        let count = regions.iter().flatten().count();
        // Loan arrays in schema order; each borrow ends before the next loan.
        // Only a few scalar bounds live on the stack. No image copy, temporary
        // vector, or intermediate output array is needed.
        let boxes = self.detections.loan_boxes(count * 4)?;
        for (dst, region) in boxes
            .as_chunks_mut::<4>()
            .0
            .iter_mut()
            .zip(regions.iter().flatten())
        {
            dst[0] = region.left as f64 + DETECTION_SHIFT;
            dst[1] = region.top as f64;
            dst[2] = (region.right - region.left + 1) as f64;
            dst[3] = (region.bottom - region.top + 1) as f64;
        }
        let scores = self.detections.loan_scores(count)?;
        for (dst, (class, _)) in scores
            .iter_mut()
            .zip(regions.iter().enumerate().filter(|(_, r)| r.is_some()))
        {
            // Fixed tutorial scores; these are not model confidence estimates.
            *dst = 0.9 - 0.1 * class as f64;
        }
        let classes = self.detections.loan_class_ids(count)?;
        for (dst, (class, _)) in classes
            .iter_mut()
            .zip(regions.iter().enumerate().filter(|(_, r)| r.is_some()))
        {
            *dst = class as f64;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
