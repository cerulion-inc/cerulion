@AGENTS.md

## Claude Code

- All repo guidance lives in `AGENTS.md` (root + scoped); this file only imports it.
- Scoped context loads on demand (via each directory's one-line `CLAUDE.md` shim) only when
  you touch files there - before running a crate's tests, read `crates/<crate>/AGENTS.md`
  yourself: it lists which test binaries must run with `-- --test-threads=1`.
- `CLAUDE.local.md` is gitignored - put personal, uncommitted instructions there.
