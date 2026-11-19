# The Cerulion harness

This folder is the Cerulion harness: the tools and body knowledge a general model needs to work a robot it was
never trained on, the simulated robot it runs on in this release, and the benchmark that measured it. You run it
through the `cerulion` command; the guide is [docs/harness.md](../docs/harness.md).

```bash
cerulion login                                   # once, on this machine
cerulion rcp --sim --http 127.0.0.1:8765         # the harness's MCP tools with a simulated robot
claude mcp add --transport http cerulion-sim http://127.0.0.1:8765/mcp
cerulion rcp bench --model YOUR_MODEL            # one benchmark task, with and without the harness
cerulion rcp bench --verify                      # the code here is the code the benchmark measured
```

Every release package installs this folder beside the `cerulion` binary, without `tests/` and the development
files, and nothing writes into it there: the first run copies it once to `harness/<version>` under the Cerulion
config directory and runs from the copy. A `cerulion` built in this checkout (`cargo run -p cerulion_cli -- rcp
--sim`, or `target/debug/cerulion`) finds this folder from any directory, and `CERULION_HARNESS_DIR` points any
`cerulion` at it; both run it in place while your user can write to it.
The current directory is never searched for a harness. `CERULION_HARNESS_DIR` is taken as given; this folder
beside the binary, or in a checkout, is never taken from a folder that every user can write to, such as `/tmp`.
A `cerulion` installed with `cargo install` has no harness beside it: clone this repository at the tag of its version (`git clone --depth 1 --branch v<version> https://github.com/cerulion-inc/cerulion`, with the version `cerulion --version` prints) and set `CERULION_HARNESS_DIR` to the clone's `harness/` folder, or, for one installed from a checkout (`cargo install --path`), to that checkout's `harness/`.

## What is in it

| Path | What it holds |
|---|---|
| `rcp.sh` | the program `cerulion rcp --sim` and `cerulion rcp bench` run: it installs on first use, then starts the sim server or the benchmark run |
| `install.sh` | the one-time install: the simulator, the benchmark scenes at their pinned revisions, the episode server's environment and this machine's robot calibration record. `install.sh --check` reports what is missing and writes nothing |
| `playground/` | the sim server: the harness's MCP tools on a simulated robot, one episode per connection, over stdio or HTTP |
| `src/cerulion_harness/` | the benchmark run (`run-cell`, the engine behind `cerulion rcp bench`), the calibration records, the price table and `verify` |
| `measured/` | the code the published benchmark ran, file for file: `measured/rcp/` for the setup with the harness, and `measured/plain-overlay/` for the files the setup without it has in their place |
| `provenance/` | the manifests of the two measured code trees and the ledger of every shipped file; `verify` checks them |
| `body_records/` | the robot calibration records measured on Linux x86-64 |
| `benchmarks/` | the pinned revisions of LIBERO and LIBERO-PRO and the SHA-256 of every data file the install fetches |
| `requirements/` | every Python package of both environments, pinned |
| `macos/` | the simulator host for macOS, which renders through Apple's OpenGL |
| `container/` | a container recipe; no image is published |
| `tests/` | standard-library tests that need no simulator and no network; the release packages leave them out |

The install writes `bx/` (the simulator environment and the benchmark trees), `venv/` (the episode server's
environment) and `out/` (what the sim server keeps of each episode) inside the folder it runs from: this folder
when it runs in place, its copy when it sits beside the binary. Git ignores all three. Nothing of the benchmark's
scenes is committed here: they are fetched and checked against their pins.

The measured code keeps the package names it ran under, `rcpx` and `bench`, so that it stays byte for byte the
code the benchmark measured. RCP, the Robot Context Protocol, is the protocol's name in [docs/rcp.md](../docs/rcp.md).

## Licence

This folder is part of Cerulion and carries the repository's licence: GNU Affero General Public License v3.0
only (AGPL-3.0-only), with a separate commercial license available from Cerulion (see the repository's
`LICENSE` and `docs/legal/NOTICE`). The benchmark trees and data the install fetches carry their own licences;
`LICENSES.md` in this folder lists them.
