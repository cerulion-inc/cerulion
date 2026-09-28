<!--
Write the body in product voice, as a stranger to the project will read it: at most about 40 lines
and about 450 words of prose. State what changed and what was measured. No questions, no account of
how the work was done, no HTML, no em or en dashes, no internal identifiers, no hostnames or paths.
Delete every comment before opening the pull request.
-->

One paragraph on what this change does and why it is needed.

## What changed

* One bullet per change a reader can verify in the diff.

## How to verify

```bash
# Real commands a reader can run. Name the packages you touched: the ROOT workspace has no
# `cargo test --workspace` step (it deadlocks on iceoryx2's shared-memory singleton), so
# ci.yml's root-workspace test steps enumerate their packages, and `cerulion_core` goes
# through the shard runner. New tests and review replies follow docs/internals/testing-rules.md.
cargo test -p <crate>
./tools/scripts/ci_test_shard.sh cerulion_core <shard> 4
cargo clippy --workspace --all-targets -- -D warnings
```

<!--
Keep the next section only when the change touches a non-test path under crates/: both halves measured
on the same machine one after the other, before being the commit this pull request was rebased onto and
after being its own head, rmw p50 and p99 per size.
-->
## Latency before and after

| path | p50 before | p50 after | p99 before | p99 after |
|---|---|---|---|---|

<!-- Closes #N on its own line, last; delete the line when no issue closes with this change. -->
Closes #N
