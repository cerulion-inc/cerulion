use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

/// `#[output(...)]` accepts NO parenthesized items. Every `name(...)` form
/// — `variable(...)`, `complex(...)`, anything else — must be rejected at
/// macro-expansion time by the one generic arm, naming the offending ident
/// and the accepted forms (bare `#[output]` / `promise_within_ms = N`).
#[cerulion_node(period_ms = 100)]
struct UnknownOutputParenNode {
    #[output(variable(data))]
    image: Image,
}

fn main() {}
