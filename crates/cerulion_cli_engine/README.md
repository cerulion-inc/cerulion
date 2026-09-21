# cerulion_cli_engine

The command engine behind the
[Cerulion](https://github.com/cerulion-inc/cerulion) CLI.

It implements the logic of every `cerulion` command (workspace, node, graph,
topic, schema, recording and ROS 2 verbs) as a library that can be tested
without spawning a process. The `cerulion_cli` binary crate parses arguments
with clap and dispatches here; the `cerulion-wsd` workspace daemon calls the
same code, so a GUI applies the same rules as the terminal.

To use Cerulion, install the CLI instead:

```bash
cargo install --locked cerulion_cli cerulion_netd
```

See the [project README](https://github.com/cerulion-inc/cerulion#readme) for
an overview.

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
