# cerulion_dds

DDS discovery for [Cerulion](https://github.com/cerulion-inc/cerulion): the
crate behind `cerulion ros2 attach`, which finds the topics of a ROS 2 stack
that is already running and bridges them into a Cerulion graph.

It provides two things:

- **A DDS-free discovery vocabulary** (`DiscoveredEndpoint`, `DiscoveredQos`,
  `DiscoveryParams`, `DiscoveryResult`, the `DdsDiscovery` trait and
  `DdsError`), which always compiles and carries no DDS types.
- **The live backend** (`LiveDiscovery`, behind the `live` feature): one DDS
  participant per process, a bounded discovery window, and the list of
  endpoints, nodes and type hashes it saw.

## Features

- `jazzy` (default): the live backend with the 16-byte GID used by ROS 2 Iron
  and newer.
- `humble`: the live backend with the 24-byte GID of older distributions. It
  cannot decode discovery data from Iron or newer.
- `live`: the backend without choosing a distribution; enabled by both of the
  above.

## Who uses it

You normally reach this crate through the CLI:

```bash
cerulion ros2 attach --iface <local-interface-IP>
```

See the [ROS 2 guide](https://docs.cerulion.com/cerulion/guides/bridge-ros2) and
[ROS 2 compatibility](https://github.com/cerulion-inc/cerulion/blob/main/docs/ros2_compatibility.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
