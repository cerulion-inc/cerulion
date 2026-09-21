# docs: agent notes

User guides (`docs/*.md`), the user API reference (`docs/user-api.md`), tutorials, packaging
and legal pages, the benchmark packages under `docs/benchmarks/results/`, and the README's
media under `docs/media/`. Contributor dossiers live in `docs/internals/`. Everything here
ships to the public repository and to docs.cerulion.com.

## Shipped surface (gate: `tools/scripts/check_public_surface.sh`)

Run the gate after every edit (`--self-test` first, then the tree run). The rules it cannot judge:

- A page shows only the workspace form of an example: one node type per `nodes/<type>/`
  crate, wiring in `graphs/*.yaml`, run through the `cerulion` verbs. A single-file
  multi-node program, an in-code graph or runtime-API construction (`GraphRuntime`,
  `TransportManager`, `parse_graph`) is never shown as usage; such code lives under tests.
- No number on a user-facing page without a shipped package behind it. Print a figure exactly
  as the package under `docs/benchmarks/results/` prints it (a CSV, `HEADLINE.md` or the
  package README): 10.45, never a rounded 10.4. A new figure ships with its package.
- Plain English: no tracker ids, no typographic dashes in shipped text. A dash you remove
  lowers that file's line in `tools/scripts/public_surface_dash_ledger.txt`; never raise a line.
- A page describes the product to its user: what works, what is experimental, what is not
  supported, what to do. It never reports how the project was built (who decided, a plan step,
  a review round, one of our machines, a to-do list): gate class `work-state`, patterns in
  `tools/scripts/public_surface_workstate.txt`. A page outside `docs/internals/` reads zero and is
  never ledgered; a legitimate line (an MCAP chunk number) is named in
  `tools/scripts/public_surface_allow.txt`.
- A bulk edit never rewrites the inside of a string literal, in a doc's code block or in code:
  fix each site by hand and run the tests that read it (some pages are pinned by tests).
- Every removal of shipped content (a page, an asset, a package, a sentence with a claim) is a
  maintainer ruling: propose it in the report, never decide it in a lane.
- A page names only verbs, flags, paths and files that exist: relative links resolve, every
  `cerulion <verb>` parses against the CLI, paths are post-move (`crates/...`,
  `tools/scripts/...`, `docs/user-api.md`). Describe a removed verb in prose, never as a command.
- Every file under `docs/media/` is used by the README or a page (the media index is not a
  use); every package under `docs/benchmarks/results/` is cited by the README, the
  performance page or `docs/benchmarks/README.md`.
- A phrase the README contradicts (an old toolchain floor, an unpublished-crate note) goes into
  `tools/scripts/public_surface_phrases.txt` the day the README changes the fact.
