# cerulion_cli

The `cerulion` command-line tool for
[Cerulion](https://github.com/cerulion-inc/cerulion), a zero-copy, deterministic
framework for real-time robotics.

## Install

```bash
cargo install --locked cerulion_cli cerulion_netd
```

`cerulion_netd` is the network gateway daemon the CLI starts when a command
needs a remote robot's topic. Release binaries and the Homebrew and Debian
packages are in the
[installation guide](https://docs.cerulion.com/cerulion/installation).

## First run

```bash
cerulion workspace create my_robot
cd my_robot

# A node that fires every 100 ms and publishes one geometry_msgs/Vector3.
cerulion node create sensor --policy period_ms=100 -o geometry_msgs/Vector3 reading
# Edit nodes/sensor/src/lib.rs so tick writes the output, for example:
#     self.reading.x = self.tick_count as f64;
cerulion node build sensor

cerulion graph create perception -n my_robot
cerulion node stage sensor -g perception
cerulion graph validate perception
cerulion graph run perception

# In a second terminal:
cerulion topic echo /my_robot/sensor/reading
```

A node with no input has nothing to trigger it, so `node create` needs
`--policy period_ms=N` (or `--policy external`). An output that `tick` never
writes publishes nothing. On Unix, `graph run` runs the graph across several
processes by default and asks before it writes that layout into the graph file;
`--single-process` opts out.

`cerulion --help` prints the whole command tree, and each subcommand answers
`--help` except `ros2 run` and `ros2 launch`: those forward every argument,
`--help` included, to the native `ros2`, and refuse with exit 69 until
`librmw_cerulion.so` sits beside the `cerulion` binary. Install the ROS 2
release package, or build `rmw_cerulion` against your ROS 2 headers and set
`CERULION_LIB_DIR` to the directory holding `librmw_cerulion.so`. The
[quickstart](https://docs.cerulion.com/cerulion/quickstart) adds
recording and replay verification, and the
[API reference](https://github.com/cerulion-inc/cerulion/blob/main/docs/user-api.md)
documents every verb.

This crate is a thin clap wrapper over `cerulion_cli_engine`, where the command
logic lives.

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
