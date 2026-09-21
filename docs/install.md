# Installation details

The [README](../README.md#install) has the commands for every route. This page holds the rest: what the installer does to your machine, how the Rust compiler is provisioned on each route, and the cases where a node build needs care.

## What the install script does

`tools/scripts/install.sh` runs these steps in order:

1. It downloads the release archive for your platform and verifies its SHA-256 checksum.
2. It provisions Rust, as described in the next section. A failure here stops the install before any binary is touched, so an existing Cerulion install stays as it was.
3. It places `cerulion`, `cerulion-netd` and `cerulion-connectd` in `CERULION_INSTALL_DIR` (default `~/.cerulion/bin`). On Linux it also places `librmw_cerulion.so` and `libcerulion_heaphook.so` beside them: the ROS 2 Jazzy rmw that `cerulion ros2 run` and `cerulion ros2 launch` use, and the heap hook they preload into ROS 2 nodes. The macOS archives carry neither, because ROS 2 Jazzy has no macOS binaries.
4. Last, it writes one line to your shell startup files so every new shell has the programs on the PATH. Set `CERULION_NO_MODIFY_PATH=1` to leave your startup files alone; the installer then prints the PATH line for you to add yourself.

## How the install script provisions Rust

Each release archive carries the compiler metadata of the build beside the binaries, and the script installs that exact toolchain through rustup:

- When rustup is not already installed, the script downloads the official rustup installer from `sh.rustup.rs` and runs it with the pinned compiler as the default toolchain, so nothing further is needed.
- When rustup is already installed, the script adds the pinned compiler beside your existing toolchains and leaves your default untouched. A build outside a workspace Cerulion created then needs the compiler named on the command line (see below).
- On either path the script then checks the installed compiler's release and commit hash against the metadata the archive carries, and stops without replacing any binary when they differ.
- The Rust step honors `CARGO_HOME` and `RUSTUP_HOME`. Without `HOME`, set those two and `CERULION_INSTALL_DIR` to absolute paths first.
- A machine that has Rust without rustup is left alone: the script stops there without replacing any binary. Add [rustup](https://rustup.rs) first, or take a route that does not provision Rust.
- The Rust step writes no startup file of its own; the one PATH line comes from the install script.

## Rust on the other routes

Homebrew and the Debian package do not provision Rust. Both carry `cerulion-install-rust`, which installs the pinned compiler the same way the script does; run it once, then put `${CARGO_HOME:-$HOME/.cargo}/bin` on your PATH.

An archive you unpack by hand carries the same helper as `install_rust.sh` beside `rustc-version.txt`:

```bash
sh install_rust.sh rustc-version.txt
```

Downloaded release binaries are built with Rust 1.93.0 from rustup, pinned in the [artifact build workflow](../.github/workflows/release-artifacts.yml). For another release, read that workflow at its tag.

## The C toolchain

Building nodes needs a C toolchain, installed before rustup. On Debian and Ubuntu: `sudo apt-get install -y curl git build-essential`. On macOS: `xcode-select --install`. An `apt-get install cerulion` pulls a C toolchain in through the package's Recommends; installing the `.deb` by hand with `dpkg -i` does not.

## Build nodes with the compiler that built the CLI

A node is a shared library the CLI loads into its own process. Two rustc releases can lay out a shared type identically and still encode it differently at run time, so the loader compares the full compiler fingerprint (release plus commit hash) and refuses a node built by a different one. The refusal names both compilers and the two ways out: rebuild the node with the host's compiler, or reinstall the CLI with yours.

- A workspace you create with `cerulion workspace create` or `cerulion workspace init` records the CLI's compiler in its own `rust-toolchain.toml`, so `cerulion node build` inside it selects the right one with no environment variable. The CLI writes that file only when a rustup toolchain whose release and full commit hash match its own is already installed; the check is offline, installs nothing, and leaves your rustup default alone. A missing, custom, nightly or beta compiler produces a warning instead, and the workspace keeps whatever compiler your environment selects. An existing `rust-toolchain.toml` in the workspace, or an explicit rustup override, is left in place and has to select a matching compiler itself.
- In a workspace without a `rust-toolchain.toml`, the cloned example in the [README quickstart](../README.md#quickstart-run-record-and-verify) included, a rustup default that is a different compiler has to be overridden on the command line:

  ```bash
  RUSTUP_TOOLCHAIN=1.93.0 cerulion node build <node>
  ```

- With `cargo install --locked cerulion_cli`, or a source build, the CLI carries whatever compiler Cargo selected at install time, so a plain `cerulion node build` already matches as long as you do not change your default toolchain in between. After changing it, rebuild the CLI and your nodes together. A distribution, custom, nightly or beta compiler may not reproduce the fingerprint from its release string alone: use one compiler installation for both sides.
