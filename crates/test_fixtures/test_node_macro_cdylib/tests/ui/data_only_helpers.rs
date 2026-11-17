use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Node {
    count: u64,
    #[output]
    out: Vector3,
}

fn double(value: u64) -> u64 { value * 2 }
impl Node {
    fn data_only(&self) -> u64 { double(self.count) }
}
#[cerulion_node_impl]
impl Node {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.data_only() as f64;
        Ok(())
    }
}

fn main() { let _ = NodeEntry::new(); }
