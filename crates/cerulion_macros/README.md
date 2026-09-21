# cerulion_macros

Procedural macros for defining
[Cerulion](https://github.com/cerulion-inc/cerulion) node types.

- `#[cerulion_node(...)]` on a struct declares a node type and its trigger
  policy (`period_ms`, `sync_window_ms`, `unbounded_sync`, `external`, or
  one input marked `#[input(trigger)]`). Fields marked `#[input(...)]` and
  `#[output(...)]` are its ports.
- `#[cerulion_node_impl]` on the `impl` block supplies the `tick` method and
  the optional `init`, `shutdown` and `#[on_event(...)]` handlers.
- `#[derive(CerulionState)]` marks a type a node holds as state that a
  recording can capture and restore.

You normally do not depend on this crate directly: `cerulion_core::prelude`
re-exports the two attribute macros, `cerulion_core::state` re-exports the
derive, and `cerulion node create` writes a node that already uses them.

See the [node guide](https://docs.cerulion.com/cerulion/guides/define-a-node)
and the
[API reference](https://github.com/cerulion-inc/cerulion/blob/main/docs/user-api.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
