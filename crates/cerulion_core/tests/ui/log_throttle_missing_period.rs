use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// A throttle needs a period before its message. Leaving it out (so the
/// message lands where the period goes) or giving no message at all is a
/// compile error that shows the right form.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct LogMisuseNode {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl LogMisuseNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        warn_throttle!(self, "no period given");
        info_throttle!(self);
        error_once!(self);
        self.out.x = 1.0;
        Ok(())
    }
}

fn main() {}
