use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Node {
    count: u64,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl Node {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.count = Node::configure(ctx);
        Ok(())
    }
    fn configure(ctx: &NodeContext) -> u64 {
        let _: Option<&cerulion_core::graph::node::AnySubscriber> = ctx.subscriber("inp");
        ctx.env("COUNT", 7_u64)
    }
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.count as f64;
        Ok(())
    }
}

fn main() { let _ = NodeEntry::new(); }
