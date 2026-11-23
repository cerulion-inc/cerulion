# Install

The first `cerulion rcp --sim` or `cerulion rcp bench` installs, once, inside the folder the harness runs
from: the simulator's Python 3.10 environment, the LIBERO and LIBERO-PRO trees at pinned revisions with every
data file checked against its SHA-256, the episode server's Python 3.12 environment, and a body record of the
default start state's robot for this machine. A later run checks the install in under a second.

## Sub-features

- `install-check` reports what is missing and writes nothing.
- `install-run` installs the missing steps and skips the current ones.
- `install-resume` picks up after a stopped install.
- `install-refuse` refuses with exit 2 under 5 GB free, without `git` or `curl`, or on an unsupported host.
- `install-record` measures a body record when this machine has none for the robot.

## How to get to it (user POV)

- The first `cerulion rcp --sim` prints `First run: installing the simulator and the benchmark scenes` on
  stderr.
- A checkout runs `bash harness/install.sh` and `bash harness/install.sh --check`.

## Driving it with hv.py and the shell

Preconditions:

- Network access to GitHub, Hugging Face and PyPI, and 5 GB free beside the harness.

- **Check before.** Run `bash harness/install.sh --check`. On a fresh checkout it prints three
  `not installed:` lines and exits 1.
- **Install.** Run `bash harness/install.sh`. It prints `[1/4]` to `[4/4]`, each with `done in N s`, and
  exits 0. A first body record prints `the robot measures itself now, once`.
- **Check after.** Run `bash harness/install.sh --check`. It prints `installed: <folder>` and exits 0.
  `$HV doctor` prints `PASS  install`.
- **Nothing tracked.** Run `git status --short harness/`. It prints nothing: `bx/`, `venv/` and the new body
  record are ignored.

## Gotchas

- The install writes inside the harness folder of the checkout you are in. Each worktree has its own.
- `install.sh --bench-from DIR` links the benchmark trees of another install instead of fetching them.
- On macOS the pictures come from Apple's OpenGL and differ slightly from the Linux ones the results were
  measured with.
