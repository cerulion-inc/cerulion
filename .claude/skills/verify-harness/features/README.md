# Harness verification map

This folder is the maintained source for verifying the user-facing behavior of the Cerulion harness. Read
the index before you drive anything, then use the matching feature file as the recipe.

## Baseline preconditions

- Work from the repository root of the checkout you change. `HV=.claude/skills/verify-harness/scripts/hv.py`.
- `$HV doctor` passes. A recipe that needs the simulator also needs the install line to pass.
- A server recipe starts its own server with `$HV up` and ends with `$HV down`. Never drive a server you did
  not start.
- Each episode gets a new label (`--episode a`, then `b`). A label whose episode ended is not reused.

## Driving conventions

- Every command is literal. Keep quoted JSON, flags and labels unchanged.
- Read each image path that `$HV call` prints before you choose pixels. Pixels are `[u, v]`, u right and v
  down from the top-left of a 256 by 256 picture.
- Wait for each command to return. A motion call takes seconds; an episode with no call for about 430 s is
  stopped as idle.

## Proof and skip reporting

- A proof names the run folder that `$HV up`, `$HV test` or `$HV stdio` printed.
- Episode proof is the line from `$HV episode` with the end reason and the verdict, not the tool's own text.
- CLI proof includes the command, its exit code and the stderr line that explains it.
- Report a recipe you could not run with the command you tried and the unmet precondition (no network, no
  model key, no operator yes). Do not report a skipped entry point as verified through another one.

## Feature entry contract

Each feature file starts with an H1 title and one paragraph on the user-visible behavior. It then uses
exactly four H2 sections in this order: `Sub-features`, `How to get to it (user POV)`,
`Driving it with hv.py and the shell`, `Gotchas`.

## Features

- [Sim episode over HTTP](./sim-http.md) covers the server, one scored episode, the tools and resources, and
  the viewer pages.
- [Sim server over stdio](./sim-stdio.md) covers a client that starts the server itself, and the clean MCP
  stream on stdout.
- [Bench](./bench.md) covers `cerulion rcp bench`: the free refusals and the scored, billed run.
- [Provenance verify](./verify-provenance.md) covers `cerulion rcp bench --verify` and the measured-code
  fence.
- [Install](./install.md) covers the one-time install, its check and its refusals.
