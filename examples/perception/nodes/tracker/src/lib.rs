// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

// The DetectionArray type is codegen'd from schemas/detections.yaml by build.rs.
// The shared generated schema includes writers and snapshots this node does not use.
#[allow(dead_code)]
mod detections {
    include!(concat!(env!("OUT_DIR"), "/detections_schema.rs"));
}
use detections::DetectionArray;

// tracker: data-triggered on the detector output. Reads the DetectionArray and
// publishes a running track count as a Vector3 (x = detections this frame,
// y = cumulative). Demo realism only; it is NOT part of perception_min (the
// CI-cheap loop the demo records). Pure function of the input, so it replays
// byte-identically too.
#[cerulion_node]
#[derive(Default)]
struct TrackerNode {
    #[input(trigger)]
    detections: DetectionArray,
    #[output]
    tracks: Vector3,
    cumulative: f64,
}

#[cerulion_node_impl]
impl TrackerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Variable-array reads are typed accessors: boxes() -> &[f64], flattened
        // [x, y, w, h] per detection, so len()/4 is the detection count.
        let n = (self.detections.boxes().len() / 4) as f64;
        self.cumulative += n;
        self.tracks.x = n;
        self.tracks.y = self.cumulative;
        self.tracks.z = 0.0;
        Ok(())
    }
}
