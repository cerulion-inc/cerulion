# cerulion_core

The core runtime of [Cerulion](https://github.com/cerulion-inc/cerulion), a
zero-copy, deterministic framework for real-time robotics built on
[iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2).

Every Cerulion node depends on this crate. You do not add it by hand:
`cerulion node create` writes the dependency, and a node is then a struct, its
ports and a `tick` method:

```rust
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::LaserScan;

#[cerulion_node]
#[derive(Default)]
struct SafetyControllerNode {
    #[input(trigger, depth = 1, expect_within_ms = 100)]
    scan: LaserScan,
    #[output(promise_within_ms = 100)]
    linear_velocity: Vector3,
}

#[cerulion_node_impl]
impl SafetyControllerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let ranges = self.scan.ranges();
        let stop = ranges.is_empty() || ranges.iter().any(|&r| r.is_nan() || r < 0.5);
        self.linear_velocity.x = if stop { 0.0 } else { 0.3 };
        Ok(())
    }
}
```

The node is built with `cerulion node build`, wired in a graph YAML file and run
with `cerulion graph run`. Node code never creates the runtime, a publisher or a
subscriber itself; the [quickstart](https://docs.cerulion.com/cerulion/quickstart)
and the [node guide](https://docs.cerulion.com/cerulion/guides/define-a-node)
walk through the whole loop.

## What is inside

- **Wire format**: a 32-byte message header with schema-hash validation.
- **Codegen**: schema parsing (YAML and ROS 2 `.msg`) and Rust code generation.
- **Transport**: zero-copy publish and subscribe over iceoryx2 shared memory,
  and the zenoh bridge the network gateway runs. All local transport is shared
  memory; there is no second in-process backend.
- **Scheduler**: deterministic execution with four trigger policies (period,
  data, sync and external). Deadlines are not a trigger: they are per-port and
  per-node watchdogs (`expect_within_ms`, `promise_within_ms`,
  `tick_within_ms`).
- **Graph runtime**: graph YAML loading, validation and execution, in one
  process or split across several.

Everything outside `cerulion_core::prelude` and the macros is framework
internals and can change between releases. The
[API reference](https://github.com/cerulion-inc/cerulion/blob/main/docs/user-api.md)
is the ground truth for the surface an application author touches.

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
