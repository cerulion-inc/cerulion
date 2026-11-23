# Bench

`cerulion rcp bench --model MODEL` runs one cell (one task at one start state) twice against the user's own
OpenAI-compatible endpoint: once through the benchmark's plain three-tool setup and once through the
harness, with the same budget and scorer. It prints both verdicts and writes `summary.json` to `--out`.
Before the simulator starts it sends one small test request, so a wrong address, key or model is refused at
once.

## Sub-features

- `bench-refuse` exits 2 with a reason, before any install or simulator, when the endpoint, the key or the
  key's variable name is missing or wrong.
- `bench-preflight` exits 2 when the endpoint does not take the test request, and starts no episode.
- `bench-run` runs both setups and exits 0 when both were scored, 1 when one was not.
- `bench-images` picks the picture mode per model (`append` for a Claude model with the harness, `swap`
  otherwise) and records it in `summary.json`.

## How to get to it (user POV)

- `cerulion rcp bench --model MODEL [--base-url URL] [--key-env NAME] [--cell ID] [--out DIR]`.
- A checkout runs the same program with `bash harness/rcp.sh bench ...`, or the engine behind it with
  `cd harness && venv/bin/python -P -m cerulion_harness run-cell --model MODEL --image-mode swap --out DIR`.

## Driving it with hv.py and the shell

Preconditions:

- For `bench-preflight`: `$HV doctor` passes its install line. Without an install, `rcp.sh bench` installs
  first (minutes, network, several GB), and `run-cell` refuses before the test request when the simulator is
  missing.
- For `bench-run` and `bench-images`: the operator said yes in this session and named the endpoint and the
  key variable. Without that, stop after the refusals.

- **No endpoint.** Run `env -u OPENAI_BASE_URL bash harness/rcp.sh bench --model m`. Exit 2, stderr
  `no endpoint: pass --base-url or set OPENAI_BASE_URL`.
- **No key.** Run `env -u OPENAI_API_KEY OPENAI_BASE_URL=http://127.0.0.1:9/v1 bash harness/rcp.sh bench --model m`.
  Exit 2, stderr names `OPENAI_API_KEY`.
- **Bad variable name.** Add `--key-env 1bad`. Exit 2, stderr `'1bad' is not an environment variable name`.
- **Preflight.** Run `REPO=$PWD D=$(mktemp -d)`, then
  `(cd "$D" && OPENAI_BASE_URL=http://127.0.0.1:9/v1 OPENAI_API_KEY=not-a-key bash "$REPO/harness/rcp.sh" bench --model m); echo $?`
  and `ls -A "$D"`. Exit 2, stderr `the model endpoint did not take a test request, so no episode was
  started`, and the folder stays empty. The closed port means no model is reached.
- **Scored run (gated).** With the operator's yes, run
  `bash harness/rcp.sh bench --model MODEL --base-url URL --key-env NAME --out ~/cerulion-bench-check`.
  Exit 0, two verdict lines, and `summary.json` naming each setup's picture mode.

## Gotchas

- One episode is one draw at temperature 1.0. One pair proves the run works, not that a model does better.
- A scored run bills the operator's endpoint for up to 80 replies with images per setup and attempt;
  `run-cell` retries an unscored episode up to 3 times.
- `run-cell` must run inside `harness/` with `-P`: nothing from the starting folder may be imported.
- Ctrl+C stops the run and its model client, but the episode's simulator runs in a session of its own.
  `run-cell` works in `/tmp/cerulion-bench-XXXXXXXX` and removes it at the end, unless the stop timed out.
  After a stopped run, check `ls -d /tmp/cerulion-bench-*` and stop only the processes whose command line
  names that folder.
