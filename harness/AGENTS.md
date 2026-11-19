# harness

The Cerulion harness: Python, shell and pinned data that no Rust package builds. `cerulion rcp --sim`
and `cerulion rcp bench` run it; the user guide is `docs/harness.md`, the folder map `README.md`.

## Contract with the CLI (`crates/cerulion_cli_engine/src/harness_cmd.rs`)
- The CLI finds this folder (`CERULION_HARNESS_DIR`, then beside the canonicalized binary, then
  the root of the checkout the binary sits in, proven by `crates/cerulion_cli/Cargo.toml`; never
  the current directory) by two marker files, `rcp.sh` and `install.sh`. Renaming any of the
  three breaks the search. The last two places take nothing from a folder other users can write
  to (mode `o+w`, as `/tmp`): the walk up to a checkout root stops at the first one. No release
  package installs this folder. A test that runs the built binary without
  `CERULION_HARNESS_DIR` finds this checkout's own harness: set the variable, or run a link to
  the binary from outside the checkout (`login_gate_e2e_test` does both).
- It runs `bash rcp.sh --home DIR [--packaged] sim [--http ADDR] [--task NAME]`, `... bench --model M
  [--base-url URL] [--key-env NAME] [--cell ID] [--out DIR]` or `... verify`, with the terminal
  attached, and exits with the script's code. A flag added on one side is added on the other
  and in `harness/tests/test_rcp_launcher.py`.
- `rcp.sh` maps them to `install.sh` (`--check` first; the install's stdout goes to stderr),
  `venv/bin/python playground --stdio|--http ADDR [--state NAME]`,
  `venv/bin/python -P -m cerulion_harness run-cell ... --out ABSOLUTE` and
  `python -m cerulion_harness.verify`. bench takes exactly the flags above, each with one value.
- The check, the install and `run-cell` run in this folder as their own process group with no
  stdin, `GIT_TERMINAL_PROMPT=0` and SIGTTOU and SIGTTIN ignored (a terminal set to `stty tostop`
  never stops them); the launcher passes a SIGINT, SIGTERM or SIGHUP it gets to that whole group,
  then exits 128 + the signal's number once the group's first process has ended. Nothing in the
  directory `cerulion` started in is imported, by them or by a Python they start that keeps its
  directory: `--out` is made absolute against that directory first, and without one it names a
  new `cerulion-bench-<UTC time>` folder there, as `run-cell` would.
- STDOUT of `sim` is the MCP stream: nothing else may print there, the install included.
- The install writes inside the folder it runs from; nothing may write into the folder beside
  the binary. `--home DIR` is where the folder is copied once, byte for byte, and run from:
  always after `--packaged` (the CLI passes it for the folder beside the binary, also when
  `CERULION_HARNESS_DIR` names it), else only when this user cannot write the folder. A
  `--home` that resolves to the packaged folder itself is refused.

## Invariants
- `measured/` is the code the benchmark ran, byte for byte: never edit, format or lint it;
  `provenance/` manifests and `verify` prove it. A change there changes the snapshot ids.
- Nothing `install.sh` fetches or builds (`bx/`, `venv/`, `out/`) is ever committed; scenes and
  datasets are pinned by digest in `benchmarks/`, never stored (`tests/test_tracked_files.py`).
- No hosted piece ships here: no sign-in backend, no shared ledger, no cloud storage name.
- Scripts run under bash 3.2 (macOS): no associative arrays, no empty-array expansion under
  `set -u`. Every tracked `*.sh` passes `shellcheck -x`.
- Python tests use the standard library only and need no simulator and no network.

## Tests
- `python3 -B -m unittest discover -s harness/tests` (the CI job `Harness (Linux)` runs it,
  with shellcheck over every tracked script).
- The CLI side: `cargo test -p cerulion_cli_engine --lib harness_cmd` and
  `cargo test -p cerulion_cli --test login_gate_e2e_test -- --test-threads=1`.
