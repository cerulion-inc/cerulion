# cerulion_wsd

The local workspace-engine daemon for
[Cerulion](https://github.com/cerulion-inc/cerulion), `cerulion-wsd`.

Cerulion Studio talks to this daemon to read and edit a Cerulion workspace. It
serves workspace, graph and node inspection plus versioned node and graph edits
over a private Unix-socket protocol (one JSON object per line). Every request
goes through `cerulion_cli_engine`, the same code the `cerulion` CLI runs, so a
GUI never re-implements the engine's rules, and edits take the same workspace
lock as the CLI.

## Who uses it

Studio bundles and starts `cerulion-wsd` itself, so a Studio user never runs it
by hand. To run it for your own client:

```bash
cargo install --locked cerulion_wsd
cerulion-wsd            # serves on the default socket
cerulion-wsd --help     # socket path and environment variables
```

The protocol is described under "Desk daemons" in the
[API reference](https://github.com/cerulion-inc/cerulion/blob/main/docs/user-api.md).
Unix only.

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
