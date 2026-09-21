# native_ros2_messages

ROS 2 message types for
[Cerulion](https://github.com/cerulion-inc/cerulion) zero-copy messaging.

The crate vendors 22 message packages (254 messages) as `.msg` files under
`msg/` and generates the Rust types at build time with `cerulion_core`'s
codegen: `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `tf2_msgs`,
`vision_msgs`, `control_msgs`, `moveit_msgs`, `visualization_msgs` and more.
`cerulion schema list` prints the full set. Most packages follow ROS 2 Jazzy.
Where the vendored definition matches the one a ROS 2 process was built
against, the layout and schema hash agree and native nodes and ROS 2 nodes
share the topic; the compatibility guide lists the vendored package versions,
the known skews and how schema resolution handles them.

A node uses a message as a port type and assigns its fields:

```rust
use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 33)]
#[derive(Default)]
struct CameraNode {
    #[output]
    image: Image,
}

#[cerulion_node_impl]
impl CameraNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Fixed fields are written straight into the loaned shared-memory slot.
        self.image.height = 480;
        self.image.width = 640;
        self.image.step = 640 * 3;
        // Every variable-length field must be written each tick, or the frame
        // is discarded: for Image that is header, encoding and data.
        self.image.header.frame_id = "camera";
        self.image.encoding = "rgb8";
        self.image.loan_data(480 * 640 * 3)?.fill(0);
        Ok(())
    }
}
```

You do not add this crate by hand: `cerulion node create <type> -o
sensor_msgs/Image image` writes the dependency and the `use` line. In graph
YAML the same type is named `sensor_msgs/Image`.

See the [node guide](https://docs.cerulion.com/cerulion/guides/define-a-node).

## License

This crate is two kinds of material under two kinds of license, and both
travel with the package.

The Rust types generated at build time are part of Cerulion and are licensed
under the GNU Affero General Public License v3.0 only (AGPL-3.0-only). The
full text is in the `LICENSE` file beside this README. A separate commercial
license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).

The vendored `.msg` definitions under `msg/` are not Cerulion's work and are
not covered by that license. They stay under their own upstream licenses,
which are not all the same license: 15 packages are Apache-2.0 and 7 are
BSD-3-Clause. Every package is listed with its upstream source, revision and
copyright holder in [`msg/README.md`](msg/README.md), and again in the
`NOTICE` file. The two upstream texts ship as `LICENSE-APACHE` and
`LICENSE-BSD-3-CLAUSE`. All four files are in this crate directory and in a
crates.io download of it.
