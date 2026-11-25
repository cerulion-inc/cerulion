use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

/// `#[output]` takes no field list: codegen resolves fixed vs variable
/// fields at compile time, so listing them declares nothing. The diagnostic
/// must name the offending ident and prescribe writing
/// `self.<port>.<field> = expr` in the node body, with bare `#[output]`
/// (or `promise_within_ms = N`) on the port.
#[cerulion_node(period_ms = 100)]
struct OutputFieldListNode {
    #[output(data, encoding)]
    image: Image,
}

fn main() {}
