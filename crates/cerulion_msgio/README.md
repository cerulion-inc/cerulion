# cerulion_msgio

Messages as JSON or YAML text on the local shared memory of
[Cerulion](https://github.com/cerulion-inc/cerulion): the engine of the
`cerulion topic pub`, `cerulion service call` and `cerulion topic snapshot`
verbs, and of the same operations `cerulion-remoted` serves on a robot to a
paired desk.

- **`text`** encodes a JSON or YAML mapping as one wire frame of a message
  type (a field the text leaves out takes its `.msg` default, or zero) and
  renders a message as one line of JSON.
- **`publish`** publishes frames of one message on a topic at a set rate.
- **`service`** calls a service over the protocol `rmw_cerulion` serves.
- **`snapshot`** encodes the newest frame of a camera topic as a JPEG.
- **`schema_store`** reads a workspace's `.msg` and `.srv` files.

The verbs are documented in
[docs/user-api.md](https://github.com/cerulion-inc/cerulion/blob/main/docs/user-api.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
