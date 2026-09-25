## Summary

<!-- One paragraph: what this changes and why it matters to someone using Cerulion.
     A pull request body is read by people outside the project, so write it for a
     stranger: plain language, no internal shorthand, no first person, no questions.
     Keep the whole body to about 40 lines. -->
<!-- Name the related issue in that paragraph, for example: Closes #123 -->

## What changed

<!-- 3 to 8 bullets, each naming a file or a behaviour. No checkpoint tables, no
     to-do or follow-up lists, no HTML, and no pasted test output: CI is the record. -->

-

## How to verify

<!-- The exact commands a reader can paste, or the name of the CI check that covers
     this change. Name the packages you touched: the root workspace has no
     `cargo test --workspace` step (it deadlocks on iceoryx2's shared-memory
     singleton), so ci.yml enumerates its packages and `cerulion_core` goes through
     the shard runner. -->

```bash
cargo test -p <crate>
./tools/scripts/ci_test_shard.sh cerulion_core <shard> 4   # a cerulion_core shard
cargo clippy --workspace --all-targets -- -D warnings
```

<!-- Optional: when the change moves latency or CI time, uncomment the section below,
     fill in the table, and add one line saying where the numbers came from (machine,
     build, sample count). Leave it commented out otherwise.

## Measurements

| Metric | Before | After |
| - | - | - |
| | | |
-->
