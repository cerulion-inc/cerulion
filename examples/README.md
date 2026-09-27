# Examples

Each directory here is one runnable Cerulion workspace. The root
[README](../README.md#examples) says what each one demonstrates; this page
carries the two things they all share.

## These are standalone workspaces

Every example has its own `[workspace]` `Cargo.toml` and is excluded from the
repository's root workspace, so run its commands from inside its own directory.
Inside this repository an example depends on `cerulion_core` and
`native_ros2_messages` through relative paths in that `Cargo.toml`. A workspace
you create yourself with `cerulion workspace create` gets the published
crates.io versions instead.

## The first build and the partition prompt

The first node build in a workspace also compiles the Cerulion runtime, so it
takes a few minutes on a laptop; later builds take seconds.

On the first run of a graph with no `process_groups:` block, Cerulion proposes
one process per node and asks:

```
Apply this partition to the graph file? [y/N]
```

Press **Enter** to use that layout for this run only, or **y** to save it in the
graph YAML. `--single-process` runs the whole graph in one process instead, and
`--yes` accepts the proposal without asking.
