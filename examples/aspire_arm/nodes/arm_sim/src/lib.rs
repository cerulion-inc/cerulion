// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

//! `arm_sim`: a simulated six-joint arm with a gripper and one cube.
//!
//! Fired once per frame on `calls` (the graph wires it to `/rcp/calls`, where
//! `cerulion-rcpd` publishes every tool call an agent makes). Each frame is one
//! JSON tool call; the node applies it to the plant in [`plant`] and publishes
//! the JSON result on `results` (`/rcp/results`) and the plant state on
//! `state` (`/arm/state`). The plant is a pure function of the call sequence,
//! so `cerulion bag play --resim all --verify` recomputes both outputs
//! byte for byte from the recorded calls.

pub mod plant;

use cerulion_core::prelude::*;
use native_ros2_messages::std_msgs::String as RosString;

use plant::Plant;

#[cerulion_node]
#[derive(Default)]
struct ArmSimNode {
    #[input(trigger)]
    calls: RosString,
    #[output]
    results: RosString,
    #[output]
    state: RosString,

    plant: Plant,
}

#[cerulion_node_impl]
impl ArmSimNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let call = self.calls.data().unwrap_or("").to_owned();
        let result = self.plant.apply(&call);
        let result_json = serde_json::to_string(&result)
            .map_err(|e| NodeError::Logic(format!("result does not serialize: {e}")))?;
        let state_json = serde_json::to_string(&result.state)
            .map_err(|e| NodeError::Logic(format!("state does not serialize: {e}")))?;
        if !result.ok {
            let code = result.error.as_ref().map_or("", |e| e.code);
            let call_id = result.call_id;
            tracing::warn!(call_id, code, "call refused");
        }
        self.results.data = result_json.as_str();
        self.state.data = state_json.as_str();
        Ok(())
    }
}
