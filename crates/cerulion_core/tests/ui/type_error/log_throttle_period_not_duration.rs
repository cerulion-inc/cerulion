use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// A throttle period is a `Duration`. A bare number (which unit?) or a
/// message in the period position is refused with a message that names
/// `Duration` and shows the right form.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct LogPeriodNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl LogPeriodNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        warn_throttle!(self, 500, "a bare number");
        warn_throttle!(self, "a message", n = 1);
        self.out.x = 1.0;
        Ok(())
    }
}

fn main() {}
