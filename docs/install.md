# Installation details

The [README](../README.md#install) has the commands for every route. This page holds the rest: what the installer does to your machine, how the Rust compiler is provisioned on each route, and the cases where a node build needs care.

## What the install script does

`tools/scripts/install.sh` runs these steps in order:

1. It downloads the release archive for your platform and verifies its SHA-256 checksum.
2. It provisions Rust, as described in the next section. A failure here stops the install before any binary is touched, so an existing Cerulion install stays as it was.
3. It places `cerulion`, `cerulion-netd` and `cerulion-connectd` in `CERULION_INSTALL_DIR` (default `~/.cerulion/bin`). On Linux it also places `librmw_cerulion.so` and `libcerulion_heaphook.so` beside them: the ROS 2 Jazzy rmw that `cerulion ros2 run` and `cerulion ros2 launch` use, and the heap hook they preload into ROS 2 nodes. The macOS archives carry neither, because ROS 2 Jazzy has no macOS binaries.
4. Last, it puts the programs on the PATH:
   - It links `cerulion`, `cerulion-netd` and `cerulion-connectd` into `~/.local/bin`, creating the directory if needed. The default `~/.profile` on Debian and Ubuntu puts that directory on the PATH of a bash or sh login once it exists. When `~/.local/bin` is not on the PATH of the shell running the installer, `/usr/local/bin` is on it and you can write there without `sudo`, the programs are linked there too, unless other accounts could not follow the link into the install directory (a root install under `/root`, or a macOS home directory) or a different `cerulion` is already on the PATH. They are also not linked there when another account could change what the links run: when `/usr/local/bin`, the install directory, a directory above either, or one of the programs is group- or world-writable (a sticky `/tmp` included), is owned by an account other than root or you, or carries a macOS ACL that allows writes. The installer then prints a warning naming that directory and keeps only the `~/.local/bin` links. A name already taken by a file or another link is left alone, with a warning.
   - It writes `~/.cerulion/env` (and `env.fish`), which puts the install directory and, when this run installed Rust, `~/.cargo/bin` at the front of the PATH. It then adds one marked line that sources it to `~/.profile`, to the bash startup files that already exist, to `.zshenv` when zsh is present, and to fish's `conf.d` when fish is present. A rerun finds its line and adds nothing.
   - Its last line says what to do in the terminal you ran it from. A piped installer runs as a child process and cannot change that shell's PATH, so when `cerulion` (or Cargo) is not already found there, the line names the one command that fixes it, `. ~/.cerulion/env` (`source ~/.cerulion/env.fish` in fish). The README's one-liner ends with that command, so nothing more is needed. Otherwise the line says `cerulion` is ready.

   Set `CERULION_NO_MODIFY_PATH=1` (or pass `--no-modify-path`) to leave your startup files and `~/.local/bin` alone; the installer then prints the PATH line for you to add yourself.

## Uninstall

```bash
curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh -s -- --uninstall
```

This removes the programs and libraries from `~/.cerulion/bin` (or from the `--dir` or `CERULION_INSTALL_DIR` you installed with, which must carry the installer's marker), the links that point at them, and, when the env file names that directory, the marked line in each startup file and then the two env files. It prints `Cerulion is uninstalled` only when all of that succeeded, says so when it found nothing to remove, and otherwise names what is left. `~/.cerulion` keeps your sign-in, keys and settings; delete it to remove those too. The Rust toolchain stays with rustup (`rustup self uninstall` removes rustup and every toolchain). A new terminal no longer finds `cerulion`; in the current one, run `hash -r` (bash) or `rehash` (zsh) if it still does.

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

The 1.0.0 release binaries were built with Rust 1.93.0; the workspace's minimum is Rust 1.95, and each release archive names its own compiler in `rustc-version.txt`, pinned in the [artifact build workflow](../.github/workflows/release-artifacts.yml) at the release's tag. For another release, read that workflow at its tag.

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
