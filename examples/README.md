# Examples

The root [README](../README.md#examples) describes the main examples.
This page covers what the example workspaces have in common.

## Which directories are Cargo workspaces

These eight directories are Cerulion workspaces, each with its own
`[workspace]` `Cargo.toml` and `Cargo.lock`:

- `aspire_arm`
- `basic_timer`
- `camera_pipeline`
- `go2`
- `obstacle_avoidance`
- `perception`
- `realsense`
- `v4l2_camera`

`moveit_hero` is not one of them. It is a Docker-driven gate for
`rmw_cerulion`, run with `bash examples/moveit_hero/run_demo.sh`; its
[README](moveit_hero/README.md) covers what it needs.

## Build and run from inside the example

Each workspace builds on its own and is not part of the repository's root
workspace, so run its commands from inside its directory:

```bash
cd examples/basic_timer
cerulion node build timer --release
cerulion node build printer --release
cerulion graph validate basic_timer
cerulion graph run basic_timer --release --record
```

Inside this repository an example depends on `cerulion_core` and
`native_ros2_messages` through relative paths in its `Cargo.toml`, so it builds
against this checkout. Use a `cerulion` CLI built from the same checkout: the
loader refuses a node built against a different ABI version. A workspace you
create with `cerulion workspace create` uses the published crates at the CLI's
own version instead, unless the CLI was built from a source checkout, in which
case it points at that checkout.

The first node build in a workspace also compiles the Cerulion runtime, so it
takes a few minutes on a laptop; later builds take seconds.

## The compiler has to match the CLI

A node is a shared library loaded into the CLI's own process, so the loader
refuses a node built by a different rustc and names both compilers. These
example directories carry no `rust-toolchain.toml`, so a build uses the
compiler your environment selects. When your rustup default is a different
compiler from the one that built `cerulion`, name it on the command line:

```bash
RUSTUP_TOOLCHAIN=<release> cerulion node build <node>
```

`cerulion node build` warns, and continues, when the `rustc` on your `PATH`
reports a different release. See
[Build nodes with the compiler that built the CLI](../docs/install.md#build-nodes-with-the-compiler-that-built-the-cli).

## The process layout on the first run

On Linux and macOS, a graph with no `process_groups:` block that runs under the
real clock runs multi-process by default. `cerulion graph run` derives one
process per node (or a layout from the cost snapshot `graphs/<name>.costs.yaml`
when one exists) and runs it from memory, with no prompt and no change to the
graph file. The run log says:

```
auto-partition: derived process groups in memory; graph file unchanged
```

The other choices:

- `--yes` saves the derived partition into the graph YAML, with a `.bak` backup.
- `--single-process` runs the whole graph in one process instead.
- `cerulion graph partition <name>` previews the partition and asks
  `Apply this partition to the graph file? [y/N]`; only an explicit yes writes
  it.

The 1.0.0 release binaries, built from the `v1.0.0` tag, ask that question at
the first `graph run` on a terminal instead: press **Enter** to use the layout
for that run only, or **y** to save it.
