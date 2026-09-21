# cerulion-ros2-migrate-clang

The AST prover/rewriter behind `cerulion ros2 migrate`: a standalone clang
LibTooling tool (clang-tidy-check style) that analyses a colcon workspace's
translation units through `compile_commands.json` and emits, as JSON on
stdout, byte-range edits for every publish call site it can PROVE safe to
rewrite to the upstream loaned-message API, plus a manual-candidates list
with a fixed reason string for everything it refuses.

The tool never writes a file. The Rust verb
(`cerulion_cli_engine::ros2_migrate`) owns edit application, diff rendering,
consent, git, and the automatic `colcon build --packages-select` of affected
packages; one code path serves dry-run and `--write`, so the printed diff
and the applied bytes cannot diverge.

This directory is deliberately NOT part of the Rust workspace: the Rust tree
never grows a libclang dependency. The tool is built and tested inside the
ros2-bench Jazzy container (`tools/ros2_toolchain/Dockerfile`), the repo's
ROS/clang toolchain home.

## Layout

| path | what |
|---|---|
| `cerulion_ros2_migrate_clang.cpp` | the tool (single file; the safe-pattern prover + JSON emitter; the candidate reason vocabulary is documented in its header) |
| `build.sh` | in-container build (probes `llvm-config`; `--mutant NAME` builds a prover mutant for the matrix) |
| `run_matrix.sh` | the full in-container gate: fixture build, accept/reject matrix, stage 3b (the analysis APPLIED through the verb's own Rust plan path (the `ros2_migrate_apply` example: `parse_tool_output` → `build_plan`'s canonicalization + containment gate → the anchored `O_NOFOLLOW` writes) then rebuilt + behavior-pinned, so rewrite bytes that do not compile fail the matrix itself), determinism, prover mutants (run + restore), and `--with-verb-e2e` the real-verb end-to-end (write/commit/patch/auto-build/idempotence/behavior pin). Stage 3b builds the `cerulion_cli_engine` example in-container (first run compiles the engine tree) |
| `assert_matrix.py` | hand-written matrix oracles (per-fixture rewrite kinds + refusal reasons) |
| `fixtures/` | a colcon workspace source tree: `migrate_fixture_pkg` (one TU per matrix class) + `migrate_fixture_py` (the rclpy report-only fixture) |

## Running the matrix

From the repo root on a machine with docker:

```bash
# one-time: build the ros2-bench image (heavy: ROS Jazzy + MoveIt)
docker build -t ros2-bench tools/ros2_toolchain/

# the prover matrix (+ mutants + determinism)
docker run --rm --shm-size=1g -v "$PWD":/work ros2-bench \
    bash -lc 'cd /work/tools/ros2_migrate && ./run_matrix.sh'

# the full gate including the real-verb e2e (builds cerulion_cli in-container)
docker run --rm --shm-size=1g -v "$PWD":/work ros2-bench \
    bash -lc 'cd /work/tools/ros2_migrate && ./run_matrix.sh --with-verb-e2e'
```

`run_matrix.sh` copies the fixtures to a FRESH scratch workspace created
with `mktemp -d` under a parent directory (default `/tmp/cerulion_migrate_matrix`
inside the container; `CERULION_MIGRATE_MATRIX_WS` overrides the PARENT;
it does not name the workspace itself). The script never deletes a
pre-existing path, and its success-cleanup is DESCRIPTOR-RELATIVE: the
trap binds its working directory to the directory object it created,
identity-checks it (dev:inode), deletes the contents fd-relative
(`find . -mindepth 1 -delete`), and finishes with `rmdir`, which
removes exclusively an EMPTY directory, so a replacement tree swapped in
at the pathname at ANY point survives (with the refusal reported). That
guarantee is scoped to objects OUTSIDE the script's own scratch
workspace. The two residuals, precisely: (a) contents another process
plants INSIDE the workspace between the identity check and the deletion
are removed with it: that is what deleting one's own temp directory
means, the same semantics as every `mktemp` consumer and `/tmp` cleaner
(data placed inside another process's scratch directory has no integrity
expectation); (b) an attacker's *empty* directory at the pathname can be
`rmdir`'d, not a data-destruction primitive. Both are explicitly
outside this test tooling's threat model. The workspace path is
printed at start; the workspace is retained on failure (and always
retained when the parent was set explicitly) for inspection. The repo's
SOURCE tree is never mutated (the `--with-verb-e2e` stage does build
`cerulion_cli` into `/work/target`).

## Deploying the tool for the verb

`cerulion ros2 migrate` looks for the engine binary as:

1. `$CERULION_ROS2_MIGRATE_TOOL` (explicit path; a missing file is a loud
   error, never a silent fallback),
2. `cerulion-ros2-migrate-clang` beside the `cerulion` binary,
3. `cerulion-ros2-migrate-clang` on `PATH`.

If none resolves, the verb REFUSES with the build instructions; it never
ships an inert analysis.

## Known v1 limits

Recorded because it is the shape of the class: a
publisher name RE-DECLARED between the message declaration and the publish
would otherwise be accepted. The publisher is spliced into the rewrite as TEXT, at
the declaration, above the shadow, so the borrow resolves to the outer
publisher while the publish resolves to the inner one: a silent
cross-publisher loan out of well-defined code, reached through NAME LOOKUP
rather than through the AST, which is why every AST-level publisher proof
walks past it. Such a site is refused with `publisher-shadowed`
(`unsafe_shadowed_publisher.cpp`), and the accept envelope is held by
`safe_same_name_other_scope.cpp`: the same name in positions where it
shadows neither edit still rewrites.

Two sibling refusals close the rest of that class, because a name's meaning
can move without any declaration of that name. A using-DIRECTIVE written
between the two edits widens unqualified lookup below it, so the spliced
borrow can resolve to something else, or to nothing at all, and is
refused with `publisher-lookup-changed`. A TEMPLATE ARGUMENT written inside
the publisher expression (`ns::Holder<Tag>::pub_`, or `pub_v<Tag>`) is
looked up unqualified AT THE PUBLISH, so a block-scope alias between the
edits changes which specialization the same written text names; that is
refused with `publisher-expr-not-trivial`
(`unsafe_template_argument_publisher.cpp`), a reason the prover shares with
three other non-trivial-expression shapes, while the using-directive case
carries its own `publisher-lookup-changed` because its remedy differs from a
shadow's.

Their failure modes differ, and the difference is worth stating because it
decides what a downstream build check can catch. Under a TEMPLATE ARGUMENT
the migrated source still COMPILES and silently names a different
specialization, so nothing downstream can see it. A using-DIRECTIVE can land
either way: where the widened lookup finds NOTHING the migrated source does
not compile (measured on the reported shape: the original compiles and the
migrated source does not, `use of undeclared identifier 'pub_'`), and where
it finds a DIFFERENT publisher the result compiles and is silently wrong,
exactly like the template-argument case. A dry-run gives no hint of either:
one hands the user a broken patch, the other a wrong one.

Documented in the tool header: dependent (template) function bodies are not
analysed; generic lambdas likewise; a reference alias extracted from a field
is a plain fill use (its post-publish use is not tracked; it was equally
dangling in the original `std::move` form). Those limits fail toward REFUSAL
or no-report, never toward a wrong rewrite.

ONE limit is weaker, and is stated at its real strength: the
publisher-reassignment scan keys on arguments NAMING the publisher chain, so
it proves nothing about state a callee reaches without naming it. A MEMBER
publisher can be swapped by any member call on the node (or a helper handed
`this`, `*this`, or an alias of the node object) between borrow and
publish, and a global/static publisher by any call at all; the scan does not
see these, and a site rewritten under such a swap borrows from the OLD
publisher while publishing on the replacement (a cross-publisher loan,
behavior-changing), where the original code published its heap message on
the replacement (well-defined). Follow-up hardening: conservatively refuse a
site whose function contains a non-analyzable call receiving `this`/a node
alias while the publisher chain roots in a member, deliberately NOT done
with this fix, because it refuses common shapes (any logging or member
helper call in the callback) and needs its own accept-envelope review.
