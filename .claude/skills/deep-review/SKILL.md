---
name: Deep Review (Two-Pass)
description: >-
  This skill should be used when the user asks to "deep review", "thorough review",
  "two-pass review", "double review", "review my changes twice",
  "confidence review", "review before PR", "catch all bugs", "full review",
  "deep code review", "review with confidence score", or any request for a
  comprehensive multi-agent review that goes beyond a single-pass code review.
  Launches parallel specialized agents in two passes to maximize bug detection.
version: 0.1.0
---

# Deep Review (Two-Pass Parallel Agent Review)

## Purpose

Run a comprehensive two-pass code review using 4 specialized agents per pass,
launched in parallel. Pass 1 finds issues; you fix them. Pass 2 validates fixes
and catches anything pass 1 missed or that the fixes introduced.

Pass 2 catches what pass 1 misses: for example, the comment analyzer
in pass 2 finds an inaccurate comment that masks a behavioral regression introduced
during the pass 1 fixes.

## Scope

By default, review **unstaged changes** (`git diff`). If the user specifies a
different scope (e.g., a branch diff, specific files, or a PR number), adapt
the agent prompts accordingly.

## Protocol

### Pass 1: Find Issues

Launch **4 agents in parallel** (single message, 4 Agent tool calls):

| Agent | Focus |
|-------|-------|
| `pr-review-toolkit:code-reviewer` | Correctness, all call sites updated, lint compliance, behavioral regressions, dead code, thread safety |
| `pr-review-toolkit:silent-failure-hunter` | Silent failures, swallowed errors, inappropriate fallbacks, resource leaks, error handling adequacy |
| `pr-review-toolkit:type-design-analyzer` | New/modified type encapsulation, invariant expression, field visibility, missing trait impls |
| `pr-review-toolkit:pr-test-analyzer` | Test coverage gaps, untested error paths, test quality issues, missing edge case tests |

**Agent prompt guidelines for pass 1:**
- Tell each agent to run `git diff` to see the changes
- Include the project's code rules (from CLAUDE.md): no `.unwrap()` in lib code,
  `dead_code = "deny"`, structured tracing fields, no `#[allow(clippy::...)]` suppressions
- Tell each agent which files changed and what the changes are about
- Ask each agent to focus on its specialty: don't duplicate work across agents

**After all 4 complete:**
1. Synthesize findings into a table: issue, severity, which agent found it, file/line
2. Separate into: **actionable** (bugs, regressions, real lint violations) vs
   **informational** (pre-existing issues, style observations, out-of-scope)
3. Fix all actionable issues
4. Run the applicable verification commands from `AGENTS.md`: `cargo fmt --all -- --check`,
   scoped Clippy with `-- -D warnings`, and tests for the affected crate or named binary.
   Follow each crate's shared-memory isolation and serial-test requirements. Never use
   `cargo test --workspace` or an unqualified workspace-wide test command.

### Pass 2: Validate Fixes and Catch Regressions

Launch **4 agents in parallel** with explicit "pass 2" context:

| Agent | Focus |
|-------|-------|
| `pr-review-toolkit:code-reviewer` | What pass 1 missed; verify fixes didn't introduce regressions; re-check all items from pass 1 |
| `pr-review-toolkit:silent-failure-hunter` | Probe specific edge cases surfaced by pass 1; trace control flow through fixed code; verify resource cleanup |
| `pr-review-toolkit:comment-analyzer` | Verify all comments (especially new/modified ones) accurately describe the code; flag misleading comments that could mask bugs |
| `pr-review-toolkit:pr-test-analyzer` | Verify fixes didn't break existing tests; check if new code paths from fixes need test coverage |

**Agent prompt guidelines for pass 2:**
- Tell each agent this is a SECOND PASS: their job is to find what pass 1 missed
- Include a summary of what pass 1 found and what was fixed
- Ask agents to read the CURRENT file contents, not just the diff
- For the silent-failure-hunter: include specific edge cases to probe (e.g., "does
  `break` exit the right loop level?", "what happens when X is true but Y is false?")
- For the comment-analyzer: list every new/modified comment and ask if it's accurate

**After all 4 complete:**
1. Synthesize findings: if any critical/high issues remain, fix them
2. Repeat the affected verification after fixes, following the same `AGENTS.md`
   commands and test-isolation rules. Report the commands and actual results.

### Final Output

Present a confidence assessment:

```
## Confidence: X/5

### Pass 1 findings (N issues):
| Issue | Severity | Agent | Status |
|-------|----------|-------|--------|

### Pass 2 findings (N issues):
| Issue | Severity | Agent | Status |
|-------|----------|-------|--------|

### Verification:
- cargo fmt: PASS/FAIL
- cargo clippy: PASS/FAIL
- cargo test: PASS/FAIL (N tests)
```

**Scoring guide:**
- **5/5**: Both passes found no remaining issues after fixes; all verification passes
- **4/5**: Pass 2 found minor issues (informational, pre-existing) but no new bugs
- **3/5**: Pass 2 found issues that were fixed but indicate the changes need more care
- **2/5**: Unresolved issues remain after both passes
- **1/5**: Fundamental design problems surfaced that require rethinking

## Why Two Passes Work

Single-pass reviews miss issues that arise from **fix interactions**:
- Pass 1 identifies a redundant check and removes it
- Pass 2's comment analyzer notices the comment now describes wrong behavior
- The wrong comment masks a real logic bug introduced by the "fix"

The type-design-analyzer runs only in pass 1 (types don't change between passes).
Pass 2 replaces it with the comment-analyzer, which is most valuable AFTER code
has been modified by pass 1 fixes.

## Adapting to Non-Rust Projects

The verification commands (`cargo fmt`, `cargo clippy`, `cargo test`) are
Rust-specific. For other languages, substitute the equivalent:
- Format check (e.g., `prettier --check`, `black --check`)
- Lint check (e.g., `eslint`, `pylint`)
- Test suite (e.g., `npm test`, `pytest`)

The agent selection and two-pass protocol work for any language.
