# Installation details

The [README](../README.md#install) has the commands for every route. This page holds the rest: what the installer does to your machine, how the Rust compiler is provisioned on each route, and the cases where a node build needs care.

## What the install script does

`tools/scripts/install.sh` runs these steps in order:

1. It downloads the release archive for your platform and verifies its SHA-256 checksum.
2. It provisions Rust, as described in the next section. A failure here stops the install before any binary is touched, so an existing Cerulion install stays as it was.
3. It places `cerulion`, `cerulion-netd`, `cerulion-connectd` and `cerulion-remoted` in `CERULION_INSTALL_DIR` (default `~/.cerulion/bin`). On Linux it also places `librmw_cerulion.so` and `libcerulion_heaphook.so` beside them: the ROS 2 Jazzy rmw that `cerulion ros2 run` and `cerulion ros2 launch` use, and the heap hook they preload into ROS 2 nodes. The macOS archives carry neither, because ROS 2 Jazzy has no macOS binaries. The installer does not place a Lyrical library; for ROS 2 Lyrical, each release also publishes `cerulion-rmw-lyrical-<version>-<target>.tar.gz` for Linux x86_64 and ARM64, holding the Lyrical `librmw_cerulion.so` and the heap hook: unpack it and set `CERULION_LIB_DIR` to the unpacked directory. It is built and tested in CI but has not been run on a robot. The v1.0.0 archives predate `cerulion-remoted` and carry the other three programs. Installing one of them (`--version v1.0.0`) places those three, and removes a `cerulion-remoted` that a later release left in the install directory along with the installer's link to it.
4. Last, it puts the programs on the PATH:
   - It links `cerulion`, `cerulion-netd`, `cerulion-connectd` and `cerulion-remoted` into `~/.local/bin`, creating the directory if needed. The default `~/.profile` on Debian and Ubuntu puts that directory on the PATH of a bash or sh login once it exists. When `~/.local/bin` is not on the PATH of the shell running the installer, `/usr/local/bin` is on it and you can write there without `sudo`, the programs are linked there too, unless other accounts could not follow the link into the install directory (a root install under `/root`, or a macOS home directory) or a different `cerulion` is already on the PATH. They are also not linked there when another account could change what the links run: when `/usr/local/bin`, the install directory, a directory above either, or one of the programs is group- or world-writable (a sticky `/tmp` included), is owned by an account other than root or you, or carries a macOS ACL that allows writes. The installer then prints a warning naming that directory and keeps only the `~/.local/bin` links. A name already taken by a file or another link is left alone, with a warning.
   - It writes `~/.cerulion/env` (and `env.fish`), which puts the install directory and, when this run installed Rust, `~/.cargo/bin` at the front of the PATH. It then adds one marked line that sources it to `~/.profile`, to the bash startup files that already exist, to `.zshenv` when zsh is present, and to fish's `conf.d` when fish is present. A rerun finds its line and adds nothing.
   - Its last line says what to do in the terminal you ran it from. A piped installer runs as a child process and cannot change that shell's PATH, so when `cerulion` (or Cargo) is not already found there, the line names the one command that fixes it, `. ~/.cerulion/env` (`source ~/.cerulion/env.fish` in fish). The README's one-liner ends with that command, so nothing more is needed. Otherwise the line says `cerulion` is ready.

   Set `CERULION_NO_MODIFY_PATH=1` (or pass `--no-modify-path`) to leave your startup files and `~/.local/bin` alone; the installer then prints the PATH line for you to add yourself.

## Uninstall

```bash
curl -fsSL https://raw.githubusercontent.com/cerulion-inc/cerulion/main/tools/scripts/install.sh | sh -s -- --uninstall
```

This removes the programs and libraries from `~/.cerulion/bin` (or from the `--dir` or `CERULION_INSTALL_DIR` you installed with, which must carry the installer's marker), the links that point at them, and, when the env file names that directory, the marked line in each startup file and then the two env files. It prints `Cerulion is uninstalled` only when all of that succeeded, says so when it found nothing to remove, and otherwise names what is left. `~/.cerulion` keeps your sign-in, keys and settings; delete it to remove those too. The Rust toolchain stays with rustup (`rustup self uninstall` removes rustup and every toolchain). A new terminal no longer finds `cerulion`; in the current one, run `hash -r` (bash) or `rehash` (zsh) if it still does.

## How the install script provisions Rust

Each release archive carries `install_rust.sh`, the helper that makes Rust 1.95 or newer available for building nodes. The script runs it before it places any binary:

- When the Rust that will build nodes reports Rust 1.95 or newer and its `cargo` runs, the helper says `Rust/Cargo <version> is ready.` and changes nothing. That is the `rustc` in `${CARGO_HOME:-$HOME/.cargo}/bin` when that directory holds both programs, since the script puts it first on your PATH, and otherwise the one on your PATH. This check turns rustup's automatic toolchain install off (`RUSTUP_AUTO_INSTALL=0`), and a toolchain file in the directory you run the script from does not apply.
- When rustup is already installed, the helper adds current stable beside your existing toolchains (`rustup toolchain install stable --profile minimal`), unless the stable you have is already 1.95 or newer. Stable becomes the default only when you have no default. An older default is kept, and the helper says how to select stable: `rustup default stable`, or `RUSTUP_TOOLCHAIN=stable cerulion node build <node>` for one build.
- When there is neither Rust nor rustup, the helper downloads the official rustup installer from `sh.rustup.rs` and runs it with stable as the default toolchain and the minimal profile, so nothing further is needed.
- The Rust step honors `CARGO_HOME` and `RUSTUP_HOME`. Without `HOME`, set those two and `CERULION_INSTALL_DIR` to absolute paths first.
- When there is no rustup, a Rust that is older than 1.95 or has no working Cargo is left alone: the script stops there without replacing any binary. Upgrade it with the tool that installed it, or add [rustup](https://rustup.rs) first.
- The Rust step writes no startup file of its own; the one PATH line comes from the install script.

## Rust on the other routes

Homebrew and the Debian package do not provision Rust. Both carry `cerulion-install-rust`, which sets up Rust 1.95 or newer the same way the script does, installing current stable when none is ready; run it once, then put `${CARGO_HOME:-$HOME/.cargo}/bin` on your PATH.

An archive you unpack by hand carries the same helper as `install_rust.sh`:

```bash
sh install_rust.sh
```

The 2.0.0 release binaries are built with Rust 1.95.0 and the 1.0.0 binaries were built with Rust 1.93.0, each pinned in the [artifact build workflow](../.github/workflows/release-artifacts.yml) at the release's tag. The workspace's minimum is Rust 1.95, and a node does not need the compiler that built the CLI.

## What the Debian package requires and ships

The package depends on `libc6 (>= 2.31)`, `libstdc++6` and `libgcc-s1`, so it installs on Ubuntu 20.04 and later, Debian 11 and later, or another distribution with glibc 2.31 or newer; `apt` refuses the install on an older release. ROS 2 is a `Suggests` (`ros-jazzy-ros-base`), not a dependency: the rmw is loaded by a ROS 2 process and links no ROS library of its own. The rmw works where ROS 2 Jazzy works, Ubuntu 24.04 with `ros-jazzy-ros-base` installed. The rmw library is built in the ROS 2 Jazzy image and is not held to the 2.31 floor, so on Ubuntu 20.04 or 22.04 the CLI, the daemons and the heap hook run but the packaged rmw needs Jazzy's glibc and does not load. On a host without ROS 2 the CLI installs, and `cerulion ros2 run` and `cerulion ros2 launch` exit with an error saying `ros2` was not found on `PATH`.

The package installs `cerulion`, `cerulion-netd`, `cerulion-connectd` and `cerulion-remoted` into `/usr/bin`, with `librmw_cerulion.so`, `libcerulion_heaphook.so` and the `cerulion-install-rust` helper beside them. The v1.0.0 package predates `cerulion-remoted` and installs the other three programs. Every release archive, and the package built from it, also carries four license files, because the binaries statically link their dependency graph and most of those licenses require their text and copyright notice to travel with a binary:

| File | What it is |
|---|---|
| `LICENSE` | The AGPL-3.0-only text the combined work is under. |
| `NOTICE` | Attribution for the components a reader is most likely to care about, including the vendored ROS 2 message packages. |
| `LICENSE-BSD-3-CLAUSE` | The BSD text that seven of those message packages require. |
| `THIRD-PARTY-LICENSES.md` | The generated inventory of every crate linked into the binaries, grouped by license, with each license text reproduced once. |

In the archive the four sit at the top level beside the binaries. The package installs them into `/usr/share/doc/cerulion/`, beside a `copyright` file that names each of them by path. The [APT packaging notes](packaging/apt.md) cover how the package is built and verified.

## The C toolchain

Building nodes needs a C toolchain, installed before rustup. On Debian and Ubuntu: `sudo apt-get install -y curl git build-essential`. On macOS: `xcode-select --install`. An `apt-get install cerulion` pulls a C toolchain in through the package's Recommends; installing the `.deb` by hand with `dpkg -i` does not.

## Build nodes with a supported Rust compiler

Nodes use stable boundary major 1 and load across Rust compiler versions. Build them with Rust 1.95 or newer and the C toolchain described above. The compiler does not have to match the CLI. Rebuild existing node libraries once with `cerulion node build <node>`; older Rust-boundary libraries and different boundary majors are refused with that instruction.

A new workspace names no compiler: `cerulion workspace create` and `cerulion workspace init` write no `rust-toolchain.toml`, so Cargo and rustup select one as they do anywhere else. A toolchain file you add, and rustup overrides, keep their usual precedence, and an existing toolchain file is left as it is.

## When Rust is missing

`cerulion workspace create`, `cerulion workspace init` and `cerulion node build` check for Rust 1.95 or newer: the `cargo` on your PATH and the compiler it uses (`RUSTC`, then `CARGO_BUILD_RUSTC`, else `rustc`), with rustup's automatic toolchain install turned off. On a terminal, where stdin and stderr are both terminals, a missing or older Rust is offered for install:

```text
Rust 1.95 or newer is needed to build nodes. Install it now? [Y/n]
```

Enter or `y` runs `cerulion-install-rust` when it is on your PATH or beside `cerulion`. Without it, the CLI adds stable through the rustup you already have, or runs the official rustup installer with `-y`, stable and the minimal profile, leaving your shell startup files alone. Then it checks again and carries on with the command. When the new Cargo is only in `${CARGO_HOME:-$HOME/.cargo}/bin`, `node build` uses it from there and says how to put it on your PATH.

Off a terminal, or after a no, nothing is installed: the CLI prints the command that installs Rust, and `node build` then fails with the same error and exit code as without the check. Workspace creation needs no Rust itself, so it still succeeds. When no rustup is found on your PATH or in Cargo's home, an older Rust is not offered an install; the CLI says how to upgrade it. An older Rust that a `rust-toolchain.toml` (or `rust-toolchain`) in the workspace or above it selects is not offered one either: the CLI names the file, which you can make name `stable` or remove. When Cargo is installed in Cargo's home but your PATH does not include it, the CLI names the PATH line to add. A compiler that reports no release is left to Cargo's own error, except rustup with no default toolchain, which is offered stable.
