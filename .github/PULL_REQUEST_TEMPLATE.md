## Summary

<!-- 1-2 sentence overview. What does this pull request accomplish? -->

## What Changed

<!-- Organize changes by category. Use tables for structured data, bullets for simple changes. -->

### Category Name
<!-- e.g., "New Feature", "Bug Fix", "Test Coverage", "Documentation" -->

- Change description

## Checkpoints

<!-- List acceptance criteria. Mark each as PASS/FAIL with details. -->

| Checkpoint | Description | Status |
|------------|-------------|--------|
| 1 | Description | :white_check_mark: **PASS** |

## How to Test

```bash
# Commands to verify this pull request. Name the packages you touched: the ROOT
# workspace has no `cargo test --workspace` step (it deadlocks on iceoryx2's
# SHM singleton), so ci.yml's root-workspace test steps enumerate their
# packages, and `cerulion_core` goes through the shard runner.
cargo test -p <crate>
./tools/scripts/ci_test_shard.sh cerulion_core <shard> 4   # a cerulion_core shard
cargo clippy --workspace --all-targets -- -D warnings
```

## Test Output

```
# Paste actual test output here
```

## Checklist

- [ ] `cargo fmt --all -- --check` passes
- [ ] `cargo clippy --workspace --all-targets -- -D warnings` passes
- [ ] The `.github/workflows/ci.yml` test steps for the crates you touched pass locally
      (or `./tools/scripts/ci_test_shard.sh <package> <shard> <count>` for a `cerulion_core` shard).
      The root workspace deliberately has no `cargo test --workspace` step: it deadlocks on
      iceoryx2's shared-memory singleton. (`examples/go2` is its own workspace and does run
      `cargo test --workspace` inside it; that one is fine.)
- [ ] Iceoryx2 tests run with `--test-threads=1` (if applicable)
- [ ] No new hot-path allocations (or annotated with `// hot-path-alloc-ok: <reason>`)
- [ ] Summary above describes what changed and why
- [ ] Linked the related issue (e.g., `Closes #123`)
- [ ] Public items have doc comments
- [ ] PR is scoped to one logical change
- [ ] Self-reviewed the diff before requesting review
