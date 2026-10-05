# cerulion

The umbrella crate for [Cerulion](https://github.com/cerulion-inc/cerulion):
zero-copy shared-memory transport, a deterministic scheduler and the ROS 2
message types, re-exported together at a single version.

```bash
cargo add cerulion
```

This crate holds no logic of its own. It re-exports four things:

| Path | What it is |
| --- | --- |
| `cerulion::prelude` | Everything a node body names. Import it with a glob. |
| `cerulion::msgs` | The generated ROS 2 message types, one module per package. |
| `cerulion::core` | The runtime: transport, scheduler and graph execution. |
| `cerulion::macros` | The node macros on their own. |

## A package that writes nodes also names `cerulion_core`

The node macros expand to absolute paths into the runtime crate, of the form
`::cerulion_core::graph::node::NodeEntry`. Rust resolves the first segment of
such a path only against a crate the consuming package names in its own
manifest, and a re-export cannot put one there. So a package that uses the
macros lists both:

```toml
[dependencies]
cerulion = "1.0.0"
cerulion_core = "1.0.0"
```

With those two, a node is a struct whose fields are its ports and whose `tick`
runs when the trigger policy says so. Writing a port field writes straight into
the loaned shared-memory slot; the frame is published when `tick` returns `Ok`.

```rust
use cerulion::msgs::geometry_msgs::Vector3;
use cerulion::prelude::*;

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct SensorNode {
    #[output]
    reading: Vector3,
    tick_count: u32,
}

#[cerulion_node_impl]
impl SensorNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        self.reading.x = f64::from(self.tick_count);
        Ok(())
    }
}
```

## Depending on the crates directly

Naming `cerulion_core`, `cerulion_macros` and `native_ros2_messages`
separately keeps working, and is what the Cerulion workspace itself does.

See the
[node guide](https://docs.cerulion.com/cerulion/guides/define-a-node) and the
[API reference](https://github.com/cerulion-inc/cerulion/blob/main/docs/user-api.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
