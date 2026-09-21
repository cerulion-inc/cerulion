# cerulion_wireclient

Desk-side client pieces for the `cerulion/wire/1` remote protocol of
[Cerulion](https://github.com/cerulion-inc/cerulion): what a computer needs to
read a remote robot's topics over an iroh connection.

- `protocol`: the control vocabulary (`WireRequest`, `WireResponse`,
  `StreamPreamble`), mirroring what the robot serves.
- `config`: the resolved dial parameters (`ConnectConfig`) and the parsers for
  the endpoint id, addresses, desk key and account.
- `reader`: `run_topic_reader`, which owns one data stream per topic and
  re-injects each validated frame into local shared memory.
- `epoch`: the shared revocation-epoch push logic.

## Who uses it

This is an internal building block: `cerulion-netd` (its internet plane) and
the `cerulion connect` worker share it, and it is published because
`cerulion_netd` depends on it. It links `cerulion_link`, and through it iroh.

See [remote access](https://github.com/cerulion-inc/cerulion/blob/main/docs/remote_plane.md).
Design notes for contributors live in
[docs/internals/remote-access.md](https://github.com/cerulion-inc/cerulion/blob/main/docs/internals/remote-access.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
