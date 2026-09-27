#!/usr/bin/env bash
# check_hot_path_allocs.sh — CI lint: no heap allocations on the hot path
#
# Greps the hot-path source files — the transport plane, the graph level
# executor and the scheduler — for heap-allocation patterns and fails (exit 1,
# printing file:line) on any hit that is not explicitly allowlisted. Motivated
# by the class of bug where a Vec<u8> clone on the fan-out path should
# be an Arc<Vec<u8>> — the lint makes every allocation in hot-path files
# a deliberate, annotated decision.
#
# A scan of `transport/` ALONE is not enough: 7.0
# allocations per step in the LEVEL EXECUTOR would be caught only by
# `step_zero_alloc_test.rs`, not here, and only when a test happens
# to build a >=2-fire-level graph. So `graph/` and `scheduler/` are scanned too.
#
# Usage:
#   ./scripts/check_hot_path_allocs.sh        # exit 0 = clean, 1 = findings
#   ./scripts/check_hot_path_allocs.sh --self-test
#                                             # drive the classifier over
#                                             # synthetic fixtures with hand
#                                             # oracles (the annotation grammar
#                                             # is never exercised by the real
#                                             # tree, which is clean)
#
# Allowlisting a genuine cold-path allocation (constructor, error path,
# late-joiner history, test-only helper):
#   - same line:        let x = foo.to_vec(); // hot-path-alloc-ok: <reason>
#   - comment block immediately above the line (the marker may sit on any
#     line of a contiguous `//` block; a blank or code line ends the
#     block, but `#[...]`/`#![...]` attribute lines between the block and
#     the allocation are allowed and do not consume the annotation):
#                       // hot-path-alloc-ok: <reason explaining why this
#                       // never runs on the per-message hot path>
#                       let x = foo.to_vec();
#   The <reason> is mandatory by convention — maintainers should reject
#   annotations without one.
#
# Recording a REAL hot-path allocation that is not being fixed here:
#   - same line, or in the comment block immediately above:
#                       // hot-path-alloc-known: <why it is hot, and where the
#                       // fix is tracked>
#   This does NOT claim the site is cold. It keeps the gate green while the
#   allocation is REPORTED, loudly, on every run under a `KNOWN HOT-PATH
#   ALLOCATIONS` banner with a nonzero count — so widening the lint over a file
#   that has real findings can never launder them into silence. There is no
#   function-scope form on purpose: a real hot allocation must be pinpointed.
#
# Allowlisting a whole COLD FUNCTION:
#   - on, or in the comment block immediately above, the `fn` signature:
#                       // hot-path-alloc-ok-fn: <why this whole function
#                       // never runs on the per-message hot path>
#                       fn build_with_scheduler(..) -> .. {
#     Suppression runs from the signature to the function's closing brace
#     (the line carrying `}` at the SAME indentation — `cargo fmt`
#     guarantees this shape). A `-fn` annotation that never closes is a
#     hard ERROR, so a mis-indented or hand-formatted body cannot silently
#     swallow the rest of the file.
#   WHY a function scope exists at all: the graph and scheduler files below mix a
#   ~11k-line graph BUILD path with the per-step executor in ONE module, and
#   `build_with_scheduler` alone holds 79 allocations that are all cold for
#   the SAME reason. Seventy-nine copies of one sentence is exactly the
#   annotation-noise this header warns about; one sentence on the function is
#   the right unit. Use the LINE form inside a hot function.
#
# Built-in exclusions (no annotation needed):
#   1. Comment-only lines (first non-whitespace is `//`).
#   2. Error-struct field initializers: lines starting with `topic:` or
#      `reason:`. These are TransportError construction sites — error
#      structs own String fields, and error construction happens only on
#      failure paths (loan failure, receive failure, header mismatch),
#      never on the success hot path. Annotating all ~50 of them would
#      drown the meaningful annotations.
#   3. `#[cfg(test)]` items declared at column 0 — an inline `mod tests { .. }`
#      (skipped to its closing brace) or a `mod x;` declaration (one line).
#      Test code is compiled out of every shipping binary, so it is not the
#      hot path BY CONSTRUCTION; without this rule, 88 of runtime.rs's 250
#      findings and 14 of scheduler/mod.rs's 61 are test-module scratch Vecs.
#
# Scanned files: for every (directory, exclusions) TARGET below, every
# `*.rs` in that directory EXCEPT its listed exclusions. Scanning by glob
# (rather than enumerating hot-path files) means a new file in a scanned
# directory is linted by default — forgetting to classify it fails loud
# (findings) instead of silently skipping it — and a deleted/renamed file
# cannot strand the script
# on a missing path. Every exclusion carries a reason.

set -euo pipefail

# The `#KNOWN# ` sentinel is needed in three
# places — awk's printf, the failure-section `grep -v`, and the
# banner's `grep`+`sed`. Spelled independently, if any copy drifted, known records would leak into the
# failure section while `failures` stayed 0, the banner would go empty, and the
# script would exit 0 — confusing output, no red, and the self-test could not
# see it because it drives awk directly and never the shell split. One
# definition, passed to awk with `-v`, makes that drift unrepresentable.
KNOWN_PREFIX='#KNOWN# '
UNMATCHED_PREFIX='#UNMATCHED# '

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# ---------------------------------------------------------------------------
# TARGETS — one entry per scanned directory: "<dir>|<space-separated exclusions>"
#
# The exclusion REASONS live in the blocks above each list; an exclusion with no
# stated reason should be rejected in review, exactly like an un-reasoned
# `hot-path-alloc-ok:`.
# ---------------------------------------------------------------------------

# cerulion_core/src/transport — the zero-copy publish/receive plane.
#
# Cold-path-only modules are excluded: they are init-time, network/control-plane,
# or by-design-non-zero-copy code where heap allocation is expected.
#   adaptive_sizer.rs   init-time pool sizing heuristics
#   bridge.rs           the rmw/DDS bridge's cold construction half
#   cerulion_q.rs       the per-robot verb-dispatched QUERY SURFACE:
#                       `cerulion_q/{robot}/{verb}` key construction/parsing (built
#                       once at gateway boot / per remote demand-or-catalog GET)
#                       plus the `catalog` verb's JSON serde. Control plane:
#                       gather/serve time, never the per-message hot path. The
#                       genuinely-notable production sites still carry inline
#                       `hot-path-alloc-ok:` reasons; whole-file exclusion avoids
#                       ~20 near-identical annotations.
#   discovery.rs        liveliness/topic discovery over the network
#   gateway.rs          the separate gateway process's control plane
#   mirror_registry.rs  the MIRROR-PROVENANCE control plane: record
#                       encode/decode, the authoritative topic→robot set, and the
#                       150ms republish pump. Runs at re-inject time (once per
#                       mirrored topic) and on the periodic belt.
#   mod.rs              service setup / TransportManager construction
#   network.rs          the zenoh network plane
#   reg_channel.rs      the runtime-egress REGISTRATION control plane:
#                       record encode/decode + the 250ms republish pump. Runs at
#                       route-OPEN time (once per dynamic topic).
#   run_artifacts.rs    the `runs` verb's RUN-DIRECTORY half —
#                       reading a live run's `graph.yaml`/`run.json` off disk and
#                       folding them into served entries, once per remote runs GET
#                       on the control plane. Filesystem I/O by definition (every
#                       read allocates the document it returns, every skip its
#                       operator-readable reason). The two genuinely notable sites
#                       still carry inline `hot-path-alloc-ok:` reasons.
#   run_registry.rs     the RUN-DESCRIPTOR control plane: one small
#                       control record per interval; nothing here ever touches a
#                       message, a sample or a payload.
#   service.rs          request-response is BY DESIGN not the zero-copy
#                       path — its module doc concedes 1 copy per service message,
#                       matching rmw_iceoryx.
#   shm_guard.rs        init-time THP/RLIMIT detection + a create-time
#                       /proc/self/maps scan that madvises our SHM pool VMAs.
TRANSPORT_EXCLUDE="adaptive_sizer.rs bridge.rs cerulion_q.rs discovery.rs \
gateway.rs mirror_registry.rs mod.rs network.rs \
reg_channel.rs run_artifacts.rs run_registry.rs service.rs shm_guard.rs"

# cerulion_core/src/graph — the level executor lives here (runtime.rs).
#
#   chain.rs       the STATIC chain-fusion census; `census_chains` walks the
#                  topology, the trigger edges and the levels once at graph
#                  build and no execution path reads its result. Same class as
#                  topology.rs below, which it is built on top of.
#   config.rs      serde YAML types; parsed once at graph LOAD.
#   node.rs        NodeEntry loading + the cdylib FFI shim. Its per-step half is
#                  a thin dispatch, but the file is dominated by dlopen/info-JSON
#                  parsing and the capability probes, all of which
#                  run at LOAD. Excluded rather than swept: a real sweep
#                  of its 125 findings has not been done, and the exclusion
#                  says so instead of annotating them away here.
#   partition.rs   the auto-partitioner; runs at `graph run` preflight.
#   topology.rs    the STATIC build-time dataflow model; `build()`/`validate()`
#                  run once at graph build.
#   validation.rs  build-time graph validation.
GRAPH_EXCLUDE="chain.rs config.rs node.rs partition.rs topology.rs validation.rs"

# cerulion_core/src/scheduler — the fire/decide engine.
#
#   catchup_clamp_wiring_tests.rs  the WHOLE FILE is `#[cfg(test)] mod
#                                  catchup_clamp_wiring_tests;` (scheduler/mod.rs),
#                                  so it never reaches a shipping binary. The
#                                  per-file `#[cfg(test)]` skip below cannot see
#                                  this — the gate is in the DECLARING module.
#   wake.rs                        same shape: `#[cfg(test)] mod wake;`
#                                  (scheduler/mod.rs): the
#                                  wake-origin taxonomy is test-gated until its
#                                  reactor consumer lands.
#
# Everything else is per-step code or a pure state machine, and is scanned.
SCHEDULER_EXCLUDE="catchup_clamp_wiring_tests.rs wake.rs"

TARGETS=(
    "crates/cerulion_core/src/transport|$TRANSPORT_EXCLUDE"
    "crates/cerulion_core/src/graph|$GRAPH_EXCLUDE"
    "crates/cerulion_core/src/scheduler|$SCHEDULER_EXCLUDE"
)

# nullglob: a missing/empty target dir must yield an empty array (and trip the
# guard below), not a literal '*.rs' entry that awk then fails on.
shopt -s nullglob
HOT_PATH_FILES=()
for target in "${TARGETS[@]}"; do
    dir="${target%%|*}"
    excl_list="${target#*|}"
    before="${#HOT_PATH_FILES[@]}"
    for f in "$REPO_ROOT/$dir"/*.rs; do
        # Belt-and-suspenders with nullglob above: if nullglob is ever removed,
        # the literal '*.rs' pattern still gets skipped here rather than
        # defeating the empty-set guard below.
        [[ -e "$f" ]] || continue
        base="$(basename "$f")"
        skip=0
        for excl in $excl_list; do
            [[ "$base" == "$excl" ]] && { skip=1; break; }
        done
        [[ "$skip" -eq 1 ]] || HOT_PATH_FILES+=("$dir/$base")
    done
    if [[ "${#HOT_PATH_FILES[@]}" -eq "$before" ]]; then
        echo "ERROR: target '$dir' contributed no files" >&2
        echo "       (directory moved/renamed, or every file excluded?" >&2
        echo "        update TARGETS in $0)" >&2
        exit 2
    fi
done
shopt -u nullglob

if [[ "${#HOT_PATH_FILES[@]}" -eq 0 ]]; then
    echo "ERROR: no hot-path files found across ${#TARGETS[@]} target(s)" >&2
    echo "       (modules moved? update TARGETS in $0)" >&2
    exit 2
fi

# Allocation patterns scanned (defined inside the awk program below so the
# regex never round-trips through awk's -v escape processing):
#   Box::new(   vec!   .to_vec()   Vec::with_capacity   Vec::new(
#   String::from(   .to_string()   format!(   .clone()   .collect(
#   .to_owned()   the map/set/deque constructors   .extend(   .reserve(
# `.clone()` is included because the hot-path types that get cloned
# (String topics, Vec payload buffers) all heap-allocate; Copy-type
# clones are already denied by clippy.
#
# `.push(` is DELIBERATELY ABSENT, and the reason is measured rather than
# assumed. Adding it was tried: it produces 25 findings across the scanned
# files, and every one is the same non-finding — a push into a buffer this
# codebase CLEARS AND REUSES (`drain_scratch`, `scratch_heads`, `fired_idx`,
# the per-level `decision_slots`), which is the exact shape that
# makes the level executor alloc-free, plus
# `TraceRingProducer::push`, whose wait-free zero-alloc path has its own
# dedicated regression test (`shm_ring_zero_alloc_test.rs`). Matching it would
# demand 25 annotations that all say "this buffer is warm", which is the
# reflexive annotating this lint's header warns against — it trains a maintainer
# to type the marker rather than to think, and every real finding then arrives
# in a haystack of them. `.extend(`/`.reserve(` do NOT have that property
# (2 sites between them, and `reserve` is written precisely when the author
# INTENDS to allocate), which is why they are matched and `.push(` is not.

AWK_PROG=$(cat <<'AWKEOF'
        # Strip a trailing `//` comment. Used everywhere a STRUCTURAL test is made
# about a line (is it a one-line body? a bodiless declaration? the closing
# brace? a `mod tests {` opener?), because rustfmt PRESERVES trailing comments
# and every one of those tests is fooled by one on the raw line.
function strip_line_comment(l,   c) {
    c = l
    sub(/\/\/.*$/, "", c)
    sub(/[[:space:]]+$/, "", c)
    return c
}

# Advance the STRING-LITERAL state across one RAW source line, and report
# whether the line BEGAN inside a multi-line literal.
#
# The fn-scope machinery reads a line`s leading token, and without this mask
# nothing tells it that a
# line can be TEXT rather than code. A `cargo fmt`-clean file holding a raw
# string whose content has a line-initial `fn embedded_text() {}` at the
# annotated function`s own indentation fires a FALSE `OVERRAN`, cancels the
# suppression, and turns the annotated body`s own allocations into findings —
# exit 1 on valid source (reproduced end to end). Embedded-source fixtures are ordinary in this tree.
#
# Why this does not re-open the brace-counting hazard: brace
# counting was tried and rejected: a per-line counter desynchronises on exactly these
# literals, and because the count drives SUPPRESSION SCOPE one desync turned 0
# findings into 235. This mask drives NEITHER the pattern scan nor the
# annotation grammar — only the two places that read a line as STRUCTURE (the
# terminator byte-compare and the sibling-fn tripwire). So its failure
# directions are bounded and both are safe: a false "inside" merely defers a
# scope decision (and an unclosed scope is still a hard error at EOF), and a
# false "outside" is exactly today`s behaviour. It can never manufacture a
# finding, which is the property brace counting lacked.
#
# Modelled, each because the alternative silently mis-reads real source:
#   * RAW strings (`r"…"`, `r#"…"#`, `br##"…"##`) — the hash count must MATCH,
#     so a `"#` inside an `r##"…"##` does not close it.
#   * ORDINARY strings, including `\`-continued ones, which is how nearly every
#     operator paragraph in this tree is written.
#   * A `//` comment ENDS the scan for the line, so a lone `"` in prose cannot
#     open a string.
#   * BLOCK comments, which Rust NESTS (`/* /* */ */`), so the depth is counted
#     rather than closed at the first `*/`. A mask without this
#     fails in exactly the class the mask exists to fix: an ODD
#     number of quotes inside a block comment (`/* the " character */`, or a
#     multi-line note mentioning one) opens `in_str`, which then never closes,
#     so every following line reads as text — the terminator is never seen and
#     a `cargo fmt`-clean file is REJECTED with "the body never closed".
#     Measured on all three shapes: single-line, multi-line spanning the closing
#     brace, and nested. A line inside a block comment is also not code, so it
#     joins the mask for the same reason a literal line does: an example in a
#     comment can carry a line-initial `fn` or `}` at the annotated indentation.
#   * CHAR literals (`'"'`), or a quote inside one would open a string and
#     swallow the file. Only the short shapes are consumed, so a LIFETIME
#     (`&'static str`) is never eaten.
function scan_literals(l,   i, n, c, j, h, began) {
    began = (in_raw || in_str || in_block)
    i = 1
    n = length(l)
    while (i <= n) {
        c = substr(l, i, 1)
        if (in_raw) {
            if (c == "\"") {
                h = 0
                while (substr(l, i + 1 + h, 1) == "#") h++
                if (h >= raw_hashes) { in_raw = 0; i += 1 + raw_hashes; continue }
            }
            i++
            continue
        }
        if (in_str) {
            if (c == "\\") { i += 2; continue }
            if (c == "\"") in_str = 0
            i++
            continue
        }
        if (in_block) {
            # Rust block comments NEST, so count depth: a `*/` inside an outer
            # comment must not end it.
            if (c == "/" && substr(l, i + 1, 1) == "*") { in_block++; i += 2; continue }
            if (c == "*" && substr(l, i + 1, 1) == "/") { in_block--; i += 2; continue }
            i++
            continue
        }
        if (c == "/") {
            if (substr(l, i + 1, 1) == "/") return began
            if (substr(l, i + 1, 1) == "*") { in_block++; i += 2; continue }
            i++
            continue
        }
        if (c == "'") {
            if (substr(l, i + 2, 1) == "'") { i += 3; continue }
            if (substr(l, i + 1, 1) == "\\" && substr(l, i + 3, 1) == "'") { i += 4; continue }
            i++
            continue
        }
        if (c == "r" || c == "b") {
            # The prefix must START a token, or `for` / `substr` would look like
            # a raw string opener.
            if (i > 1 && substr(l, i - 1, 1) ~ /[A-Za-z0-9_]/) { i++; continue }
            j = i
            if (c == "b" && substr(l, j + 1, 1) == "r") j++
            if (substr(l, j, 1) == "r") {
                h = 0
                while (substr(l, j + 1 + h, 1) == "#") h++
                if (substr(l, j + 1 + h, 1) == "\"") {
                    raw_hashes = h
                    in_raw = 1
                    i = j + h + 2
                    continue
                }
            }
            i++
            continue
        }
        if (c == "\"") { in_str = 1; i++; continue }
        i++
    }
    return began
}


BEGIN {
            bad = 0
            allow_next = 0            # a line-scope `hot-path-alloc-ok:` is armed
            known_next = 0            # a line-scope `hot-path-alloc-known:` is armed
            fn_allow_next = 0         # a fn-scope `hot-path-alloc-ok-fn:` is armed
            fn_suppress = 0           # inside an allowlisted function body
            fn_indent = ""            # the annotated fn's own indentation
            fn_open_line = 0          # where that body opened, for the overrun error
            fn_line = 0               # where the annotation was, for the error
            in_cfg_test = 0           # inside a `#[cfg(test)]` item at column 0
            cfg_test_open = 0
            in_str = 0                # inside an ordinary "…" literal
            in_raw = 0                # inside a raw r#"…"# literal
            raw_hashes = 0            # …with this many hashes, which must match
            in_block = 0              # depth of nested /* … */ block comments
            # `.collect(` is matched: without it an unannotated
            # `.collect::<Vec<_>>()` — the single most common way to build a
            # collection in this codebase — walks straight through a lint whose
            # whole purpose is to make every hot-path allocation deliberate.
            # Catching `vec!` and not `collect` would be an arbitrary
            # line for a maintainer to have to know. It is an OVER-approximation
            # on purpose (a `collect::<Result<(), _>>()` allocates nothing);
            # the answer to a cold hit is the same as everywhere else — annotate
            # it with a reason.
            # `\.collect[(:]` matches BOTH spellings. The turbofish form
            # (`.collect::<Vec<_>>()`) is the common one in this tree, and a
            # naive `\.collect\(` matches only the inferred form — a hole the
            # self-test's own `collect` arm catches.
            # The header cites the executor regression (7.0
            # allocations per step in the LEVEL EXECUTOR) as the reason the scan
            # covers `graph/` and `scheduler/`. Those allocations
            # were a per-step `HashMap` plus an `into_par_iter().collect()`, so
            # `HashMap::new`, `String::new` and `collect` must all be in the
            # pattern set, or the header promises coverage the regex does not have:
            # re-introducing exactly that regression would pass this lint silently
            # and the only net left would be `step_zero_alloc_test`. The map/set
            # constructors below close it.
            # `.extend(` and `.reserve(` are matched: they are
            # shapes the script's OWN unmatched-annotation banner names —
            # an annotation on a line this regex cannot match arms
            # nothing, and the banner cannot tell that from a
            # STALE annotation (measured: 2
            # sites, both cold, both annotated). `.push(` was
            # measured too and is deliberately NOT matched — see the header.
            ALLOC_RE = "Box::new\\(|vec!|\\.to_vec\\(\\)|Vec::with_capacity|Vec::new\\(|String::from\\(|\\.to_string\\(\\)|format!\\(|\\.clone\\(\\)|\\.collect[(:]|\\.to_owned\\(\\)|String::new\\(|String::with_capacity|(Hash|BTree)(Map|Set)::(new\\(|with_capacity|from\\()|VecDeque::(new\\(|with_capacity)|IndexMap::(new\\(|with_capacity)|\\.extend\\(|\\.reserve\\("
            FN_RE = "^([[:space:]]*)((pub([(][^)]*[)])?[[:space:]]+)?(default[[:space:]]+)?(const[[:space:]]+)?(async[[:space:]]+)?(unsafe[[:space:]]+)?(extern[[:space:]]+\"[^\"]*\"[[:space:]]+)?fn[[:space:]])"
        }
        {
            line = $0
            # FIRST, and before any `next`: the mask has to see EVERY line or
            # its state desynchronises. `in_literal` is true when THIS line
            # began inside a multi-line literal OR a block comment — i.e. when
            # its leading token is TEXT rather than code.
            in_literal = scan_literals(line)

            # Exclusion 3: `#[cfg(test)]` items declared at column 0. Test code
            # is compiled out of every shipping binary, so it is not the hot
            # path by construction. Two shapes: an INLINE `mod tests { .. }`
            # (skip to its column-0 closing brace) and a `mod x;` declaration
            # (one line). Distinguishing them matters — scheduler/mod.rs has
            # BOTH, and a scanner that stopped at the first `#[cfg(test)]`
            # would skip 99% of that file.
            if (in_cfg_test) {
                # Test both shapes on the comment-stripped
                # line. `mod tests { // helpers` is rustfmt-stable and on the raw line
                # misses the `{$` opener test, leaving the machine in limbo where
                # the next `;`-ending line (`use super::*;`) is misread as the
                # one-line `mod x;` form — which TERMINATES the skip and makes
                # the rest of the test module scan as production code.
                cfgc = strip_line_comment(line)
                if (!cfg_test_open && cfgc ~ /;[[:space:]]*$/) { in_cfg_test = 0; next }
                if (cfgc ~ /\{[[:space:]]*$/) { cfg_test_open = 1; next }
                if (cfg_test_open && cfgc ~ /^\}/) { in_cfg_test = 0; cfg_test_open = 0; next }
                next
            }
            if (line ~ /^#\[cfg\(test\)\]/) { in_cfg_test = 1; cfg_test_open = 0; next }

            # Marker recognition runs BEFORE fn-scope suppression, for two
            # reasons.
            #
            # First: an annotation with no <reason> is not an annotation.
            # The header documents `<reason>` as mandatory.
            # A matcher that only looked for the marker would let
            # a bare `// hot-path-alloc-ok:` suppress exactly as well as a
            # reasoned one. A maintainer skimming a diff sees a marker and moves
            # on. A marker with no reason text is a hard ERROR — the failure
            # mode of "I annotated it" must never be "the gate stopped looking".
            has_allow = (line ~ /hot-path-alloc-ok:[[:space:]]*[^[:space:]]/)
            has_fn_allow = (line ~ /hot-path-alloc-ok-fn:[[:space:]]*[^[:space:]]/)
            has_known = (line ~ /hot-path-alloc-known:[[:space:]]*[^[:space:]]/)

            bare = ""
            if (!has_allow && line ~ /hot-path-alloc-ok:/) { bare = "hot-path-alloc-ok:" }
            else if (!has_fn_allow && line ~ /hot-path-alloc-ok-fn:/) { bare = "hot-path-alloc-ok-fn:" }
            else if (!has_known && line ~ /hot-path-alloc-known:/) { bare = "hot-path-alloc-known:" }
            if (bare != "") {
                printf "%s:%d: `%s` carries no <reason>. Every annotation must state why (see the script header); a bare marker suppresses nothing.\n", rel, NR, bare
                bad = 1
            }

            # Inside an allowlisted function. Scope tracking is BRACE DEPTH
            # from the signature line, not a bare `line == fn_indent "}"`
            # byte-compare, which has three reproduced silent-overrun
            # shapes: a trailing comment on the closing brace
            # (`} // end of foo`, which rustfmt preserves) or a hand-indented
            # body makes the terminator miss, and suppression then closes on the
            # NEXT SIBLING function's brace — swallowing an entire unrelated
            # function with exit 0 and no error, because the "never closed" EOF
            # guard cannot fire when some later brace matches.
            #
            # Precise Rust lexing is not available in awk (string literals hold
            # braces), so the depth counter is backed by TWO further defenses:
            # the indent match is still ACCEPTED as a close (so a miscount that
            # runs long is rescued), and a `fn` signature seen at the annotated
            # function's OWN indentation while still suppressed is structurally
            # impossible — that is an overrun, and it is a hard ERROR
            # instead of silence. Between them, a miscount fails loud in the
            # direction that matters rather than silently unscanning code.
            if (fn_suppress) {
                # The terminator is compared on the COMMENT-STRIPPED line:
                # `} // end of foo` is rustfmt-stable and MISSES on the raw line, after
                # which suppression closes on the NEXT SIBLING function's brace
                # and swallows that whole function silently.
                fsc = in_literal ? "" : strip_line_comment(line)
                if (fsc == fn_indent "}" || fsc == fn_indent "}," || fsc == fn_indent "};") {
                    fn_suppress = 0
                    known_next = 0
                    next
                }
                # THE OVERRUN TRIPWIRE. A `fn` signature at the annotated
                # function's OWN indentation, while its body is still open, is
                # structurally impossible — a nested fn is deeper, a sibling
                # means the terminator was missed. This is what makes the
                # remaining heuristic fail LOUD instead of silently unscanning:
                # whatever shape defeats the byte-compare (a hand-indented
                # brace, a macro-generated body), the scan cannot cross into
                # the next function without saying so.
                if (fsc ~ FN_RE) {
                    match(fsc, /^[[:space:]]*/)
                    if (substr(fsc, 1, RLENGTH) == fn_indent) {
                        printf "%s:%d: a `hot-path-alloc-ok-fn:` scope (opened at line %d) is still open at a SIBLING `fn` at the same indentation — the annotated body never closed, so the scan OVERRAN into unrelated code. Reformat with `cargo fmt`, or use the per-line `hot-path-alloc-ok:` form.\n", rel, NR, fn_open_line
                        bad = 1
                        fn_suppress = 0
                        known_next = 0
                    }
                }
                # Second: a `-known:` marker is never swallowed by an
                # enclosing `-ok-fn:`. The fn-scope form exists for a function
                # whose allocations are cold for ONE shared reason; a `-known:`
                # inside it is the author saying that shared reason does not
                # hold HERE. Swallowing it is the precise hazard the fn form
                # creates — `build_with_scheduler` spans 2,661 lines and
                # CONSTRUCTS the per-fire tick callback inline, so one line
                # inside a function that is 88/89 build-time really does run at
                # fire rate. A recorded finding must survive its own container.
                if (line ~ /^[[:space:]]*\/\//) {
                    if (has_known) { known_next = 1 }
                    next
                }
                if (line ~ /^[[:space:]]*#!?\[/) { next }
                code = line
                sub(/\/\/.*$/, "", code)
                if ((has_known || known_next) && code ~ ALLOC_RE) {
                    printf "%s%s:%d: %s\n", KNOWN_PREFIX, rel, NR, line
                }
                known_next = 0
                next
            }

            # Exclusion 1: comment-only lines. An allow marker anywhere in a
            # contiguous comment block arms suppression for the first code line
            # after the block (comment lines preserve an already armed flag; any
            # non-comment line clears it at the bottom).
            if (line ~ /^[[:space:]]*\/\//) {
                if (has_fn_allow) { fn_allow_next = 1; fn_line = NR }
                else if (has_known) { known_next = 1 }
                else if (has_allow) { allow_next = 1 }
                next
            }

            # Attribute lines (#[...] / #![...]) pass through without consuming
            # an armed allow flag — a `// hot-path-alloc-ok:` block may
            # legitimately sit above `#[allow(...)]` or `#[cfg(...)]` attributes
            # that precede the allocating line (or the annotated `fn`).
            if (line ~ /^[[:space:]]*#!?\[/) {
                next
            }

            # A fn-scope annotation must land on a `fn` signature.
            if (fn_allow_next || has_fn_allow) {
                if (line ~ FN_RE) {
                    # Every test below runs on the COMMENT-STRIPPED line.
                    # A same-line `-ok-fn:` is by definition a
                    # trailing comment, so a raw-line one-line-body test never
                    # sees the `}` of `fn f() -> usize { 1 } // hot-path-alloc-ok-fn: ..`
                    # and arms suppression that then eats the NEXT function.
                    sigc = strip_line_comment(line)
                    match(sigc, /^[[:space:]]*/)
                    fn_indent = substr(sigc, 1, RLENGTH)
                    # FN_RE also matches a bodiless
                    # declaration (a trait method, an extern-block fn). Those
                    # have no body to scope, and arming on one would skip every
                    # line to some unrelated `}` — so it is a hard error, the
                    # same class as landing off a `fn` at all.
                    if (sigc ~ /;[[:space:]]*$/) {
                        printf "%s:%d: `hot-path-alloc-ok-fn:` is on a BODILESS `fn` declaration (a trait method or extern fn). There is no body to scope — annotate the implementation, or use the per-line `hot-path-alloc-ok:` form.\n", rel, (fn_line ? fn_line : NR)
                        bad = 1
                    } else if (sigc !~ /\}[[:space:],;]*$/) {
                        # A one-line body (`fn f() { 1 }`) has no closing-brace
                        # LINE, so there is nothing to scope. Tested on the
                        # stripped line: with a same-line annotation the `}` is
                        # followed by the marker comment, which is exactly how
                        # a raw-line test would miss it and arm.
                        fn_suppress = 1
                        fn_open_line = NR
                        if (fn_line == 0) { fn_line = NR }
                    }
                    fn_allow_next = 0
                    fn_line = 0
                    next
                }
                printf "%s:%d: `hot-path-alloc-ok-fn:` is not on a `fn` signature\n", rel, (fn_line ? fn_line : NR)
                bad = 1
                fn_allow_next = 0
                fn_line = 0
            }

            # Strip any trailing // comment so patterns inside prose comments
            # after code do not false-positive. (Crude: does not parse string
            # literals, but `//` inside hot-path string literals is not a case
            # we have.)
            code = line
            sub(/\/\/.*$/, "", code)

            if (code ~ ALLOC_RE) {
                if (has_known || known_next) {
                    # A REAL hot-path allocation, recorded rather than claimed
                    # cold. Reported under its own banner; does not fail.
                    printf "%s%s:%d: %s\n", KNOWN_PREFIX, rel, NR, line
                } else if (has_allow || allow_next) {
                    # explicitly allowlisted
                } else if (code ~ /^[[:space:]]*(topic|reason):/) {
                    # Exclusion 2: error-struct field initializer (cold).
                } else {
                    printf "%s:%d: %s\n", rel, NR, line
                    bad = 1
                }
            }
            # An armed `-ok:`/`-known:` consumed by a
            # line matching no allocation pattern must not clear in SILENCE, or a
            # marker whose allocation was refactored away sits in the tree
            # claiming a hot site that no longer exists.
            #
            # It is REPORTED, not failed, and the reason is a measurement: run
            # as an error, the real tree produced three hits and ALL THREE were
            # truthful annotations on genuine allocations whose SHAPE this regex
            # does not match — `order.extend(..cloned())`, `out.push(..)`,
            # `drain_scratch.reserve(..)`. A gate that forces deleting a
            # truthful annotation is worse than the silence it replaces, and
            # the two cases are not distinguishable from here. So the banner is
            # informational and doubles as a map of the pattern set's own blind
            # spots, made visible instead of asserted.
            if ((allow_next || known_next) && code !~ ALLOC_RE) {
                printf "%s%s:%d: %s\n", UNMATCHED_PREFIX, rel, NR, line
            }
            allow_next = 0
            known_next = 0
        }
        END {
            # A `-fn` annotation whose closing brace never arrived swallowed the
            # rest of the file. Fail LOUDLY rather than under-report.
            if (fn_suppress) {
                printf "%s: a `hot-path-alloc-ok-fn:` function body never closed at its own indentation — the rest of the file went UNSCANNED. Reformat with `cargo fmt`, or use the per-line `hot-path-alloc-ok:` form.\n", rel
                bad = 1
            }
            if (in_cfg_test) {
                printf "%s: a column-0 `#[cfg(test)]` item never closed — the rest of the file went UNSCANNED.\n", rel
                bad = 1
            }
            exit bad
        }
AWKEOF
)

# ---------------------------------------------------------------------------
# --self-test: drive the classifier over synthetic fixtures with hand oracles.
#
# CI never exercises the annotation grammar on a file that VIOLATES it — the
# tree is clean by construction, so a rule that stopped working (a `-fn:`
# annotation that suppressed nothing, a `#[cfg(test)]` skip that swallowed the
# file, a `-known:` marker that started failing the build) would look exactly
# like success. Each case below is a variant of one rule with the expected
# verdict written next to it.
# ---------------------------------------------------------------------------
self_test() {
    local tmp status out rc=0
    tmp="$(mktemp -d)"
    trap 'rm -rf "$tmp"' RETURN

    # $1 name, $2 expected exit (0/1), $3 expected substring ("" = none),
    # $4 forbidden substring ("" = none); fixture arrives on stdin.
    check() {
        local name="$1" want_rc="$2" want="$3" deny="$4"
        cat > "$tmp/case.rs"
        set +e
        out="$(awk -v rel="case.rs" -v KNOWN_PREFIX="$KNOWN_PREFIX" -v UNMATCHED_PREFIX="$UNMATCHED_PREFIX" "$AWK_PROG" "$tmp/case.rs")"
        status=$?
        set -e
        if [[ "$status" -ne "$want_rc" ]]; then
            echo "SELF-TEST FAIL [$name]: exit $status, wanted $want_rc" >&2
            echo "--- classifier output ---" >&2
            printf '%s\n' "$out" >&2
            rc=1
            return
        fi
        if [[ -n "$want" && "$out" != *"$want"* ]]; then
            echo "SELF-TEST FAIL [$name]: output does not contain '$want'" >&2
            printf '%s\n' "$out" >&2
            rc=1
            return
        fi
        if [[ -n "$deny" && "$out" == *"$deny"* ]]; then
            echo "SELF-TEST FAIL [$name]: output must NOT contain '$deny'" >&2
            printf '%s\n' "$out" >&2
            rc=1
            return
        fi
        echo "  ok  $name"
    }

    echo "check_hot_path_allocs.sh --self-test"

    check "a bare allocation FAILS" 1 "case.rs:2:" "" <<'EOF'
fn hot() {
    let v = Vec::new();
}
EOF

    check "a line annotation suppresses it" 0 "" "case.rs:" <<'EOF'
fn hot() {
    // hot-path-alloc-ok: cold, because reasons
    let v = Vec::new();
}
EOF

    check "a same-line annotation suppresses it" 0 "" "case.rs:" <<'EOF'
fn hot() {
    let v = Vec::new(); // hot-path-alloc-ok: cold, because reasons
}
EOF

    check "a line annotation covers ONE line only" 1 "case.rs:4:" "" <<'EOF'
fn hot() {
    // hot-path-alloc-ok: cold, because reasons
    let v = Vec::new();
    let w = Vec::new();
}
EOF

    check "a fn annotation covers the whole body" 0 "" "case.rs:" <<'EOF'
    // hot-path-alloc-ok-fn: cold, runs once at build
    fn cold(&self) {
        let v = Vec::new();
        let w = String::from("x");
        if true {
            let x = format!("{}", 1);
        }
    }
EOF

    check "a fn annotation STOPS at the closing brace" 1 "case.rs:7:" "" <<'EOF'
    // hot-path-alloc-ok-fn: cold, runs once at build
    fn cold(&self) {
        let v = Vec::new();
    }

    fn hot(&self) {
        let w = Vec::new();
    }
EOF

    check "a fn annotation not on a fn is an ERROR" 1 "is not on a \`fn\` signature" "" <<'EOF'
    // hot-path-alloc-ok-fn: misplaced
    let v = Vec::new();
EOF

    check "an unclosed fn annotation is an ERROR" 1 "never closed at its own indentation" "" <<'EOF'
    // hot-path-alloc-ok-fn: cold, runs once at build
    fn cold(&self) {
        let v = Vec::new();
EOF

    check "a known marker records without failing" 0 "#KNOWN# case.rs:3:" "" <<'EOF'
fn hot() {
    // hot-path-alloc-known: really hot, not fixed yet
    let v = Vec::new();
}
EOF

    # ── collection building is an allocation ─────────────────────────────────
    check "a collect() is caught" 1 "case.rs:2:" "" <<'EOF'
fn hot(xs: &[u8]) -> Vec<u8> {
    xs.iter().copied().collect::<Vec<_>>()
}
EOF

    check "an annotated collect() is suppressed" 0 "" "case.rs:" <<'EOF'
fn cold(xs: &[u8]) -> Vec<u8> {
    // hot-path-alloc-ok: cold, once at build
    xs.iter().copied().collect::<Vec<_>>()
}
EOF

    # The INFERRED spelling too, so the pattern is pinned on both forms rather
    # than on whichever one happened to be written first.
    check "an inferred-type collect() is caught" 1 "case.rs:2:" "" <<'EOF'
fn hot(xs: &[u8]) -> Vec<u8> {
    xs.iter().copied().collect()
}
EOF

    check "a to_owned() is caught" 1 "case.rs:2:" "" <<'EOF'
fn hot(s: &str) -> String {
    s.to_owned()
}
EOF

    # ── `.extend(` and `.reserve(`: shapes the unmatched-annotation
    #    banner would otherwise name as blind spots ────────────────────────────
    check "an extend() is caught" 1 "case.rs:2:" "" <<'EOF'
fn hot(out: &mut Vec<u8>, xs: &[u8]) {
    out.extend(xs.iter().copied());
}
EOF

    check "a reserve() is caught" 1 "case.rs:2:" "" <<'EOF'
fn hot(out: &mut Vec<u8>, n: usize) {
    out.reserve(n);
}
EOF

    # …and the ANTI-TAUTOLOGY half: `.push(` stays unmatched ON PURPOSE, so a
    # push into a cleared-and-reused buffer is not a finding. Without this,
    # someone "closing the last blind spot" would add `.push(` and produce 25
    # findings whose only truthful annotation is "this buffer is warm".
    check "a push() is deliberately NOT caught" 0 "" "case.rs:" <<'EOF'
fn hot(out: &mut Vec<u8>, x: u8) {
    out.push(x);
}
EOF

    # ── an annotation with no reason is not an annotation ────────────────────
    check "a bare -ok: marker is an ERROR, not a pass" 1 "carries no <reason>" "" <<'EOF'
fn hot() {
    // hot-path-alloc-ok:
    let v = Vec::new();
}
EOF

    check "a bare -ok: does NOT suppress the allocation either" 1 "case.rs:3:" "" <<'EOF'
fn hot() {
    // hot-path-alloc-ok:
    let v = Vec::new();
}
EOF

    check "a bare same-line -ok: marker is an ERROR" 1 "carries no <reason>" "" <<'EOF'
fn hot() {
    let v = Vec::new(); // hot-path-alloc-ok:
}
EOF

    check "a bare -ok-fn: marker is an ERROR" 1 "carries no <reason>" "" <<'EOF'
    // hot-path-alloc-ok-fn:
    fn cold(&self) {
        let v = Vec::new();
    }
EOF

    check "a bare -known: marker is an ERROR" 1 "carries no <reason>" "" <<'EOF'
fn hot() {
    // hot-path-alloc-known:
    let v = Vec::new();
}
EOF

    # The anti-tautology half: a REASONED marker of each kind still passes, so
    # the arms above are pinning the empty reason and not the marker itself.
    check "a reasoned marker of each kind still passes" 0 "" "carries no <reason>" <<'EOF'
    // hot-path-alloc-ok-fn: cold, runs once at build
    fn cold(&self) {
        let v = Vec::new();
    }
EOF

    # ── `-known:` survives an enclosing `-ok-fn:` ────────────────────────────
    check "a -known: line inside an -ok-fn: body is still REPORTED" 0 "#KNOWN# case.rs:5:" "" <<'EOF'
    // hot-path-alloc-ok-fn: cold, runs once at build
    fn mostly_cold(&self) {
        let a = Vec::new();
        // hot-path-alloc-known: this one runs per fire, not fixed yet
        let b = Vec::new();
        let c = Vec::new();
    }
EOF

    # The other direction: the -ok-fn STILL suppresses everything that is not
    # explicitly marked known, so the `-known:` rule does not turn the fn form into a no-op.
    check "an -ok-fn: still suppresses its unmarked lines" 0 "" "case.rs:3:" <<'EOF'
    // hot-path-alloc-ok-fn: cold, runs once at build
    fn mostly_cold(&self) {
        let a = Vec::new();
        // hot-path-alloc-known: this one runs per fire, not fixed yet
        let b = Vec::new();
    }
EOF

    check "a same-line -known: inside an -ok-fn: body is REPORTED" 0 "#KNOWN# case.rs:4:" "" <<'EOF'
    // hot-path-alloc-ok-fn: cold, runs once at build
    fn mostly_cold(&self) {
        let a = Vec::new();
        let b = Vec::new(); // hot-path-alloc-known: per fire, not fixed yet
    }
EOF

    # ── the -ok-fn scope family ──────────────────────────────────────────────
    check "a trailing comment on the closing brace does not leak into the next fn" 1 "case.rs:6:" "OVERRAN" <<'EOF'
    // hot-path-alloc-ok-fn: cold, runs once at build
    fn cold(&self) {
        let v = Vec::new();
    } // end of cold
    fn hot(&self) {
        let w = Vec::new();
    }
EOF

    check "a same-line -ok-fn: on a ONE-LINE fn does not swallow the next fn" 1 "case.rs:3:" "OVERRAN" <<'EOF'
    fn cold(&self) -> usize { 1 } // hot-path-alloc-ok-fn: cold accessor
    fn hot(&self) {
        let v = Vec::new();
    }
EOF

    check "an -ok-fn: on a BODILESS fn declaration is an ERROR" 1 "BODILESS" "" <<'EOF'
    // hot-path-alloc-ok-fn: cold trait method
    fn cold(&self) -> usize;
EOF

    check "a bodiless -ok-fn: does not suppress the impl below it" 1 "case.rs:6:" "" <<'EOF'
    // hot-path-alloc-ok-fn: cold trait method
    fn cold(&self) -> usize;
    fn other(&self) -> usize;
    }
    impl Foo for Bar {
    fn hot(&self) { let v = Vec::new(); }
    }
EOF

    check "a scope still open at a SIBLING fn is a loud OVERRUN error" 1 "OVERRAN" "" <<'EOF'
    // hot-path-alloc-ok-fn: cold, runs once at build
    fn cold(&self) {
        let v = Vec::new();
      }
    fn hot(&self) {
        let w = Vec::new();
    }
EOF

    # Anti-tautology for the three above: a NORMAL fn-scope annotation, with a
    # bare closing brace at its own indentation, still suppresses its body and
    # still stops at the end of it.
    check "a well-formed -ok-fn: still suppresses exactly its own body" 1 "case.rs:6:" "" <<'EOF'
    // hot-path-alloc-ok-fn: cold, runs once at build
    fn cold(&self) {
        let v = Vec::new();
    }
    fn hot(&self) {
        let w = Vec::new();
    }
EOF

    # ── a cfg(test) opener carrying a trailing comment ───────────────────────
    check "a cfg(test) module opener with a trailing comment is still skipped" 0 "" "case.rs:" <<'EOF'
fn prod() {
}

#[cfg(test)]
mod tests { // helpers live here
    use super::*;
    fn t() {
        let v = Vec::new();
    }
}
EOF

    # ── a marker landing on no matched allocation ────────────────────────────
    # The fixture is a `.push(` because that is the one allocating shape this
    # lint deliberately does NOT match (see the header). A shape the pattern
    # set DOES match (`.reserve(`, say) would make this arm
    # assert the absence of a report the lint
    # produces, which is exactly the drift the arm exists to catch, pointed at
    # itself.
    check "an annotation on an unmatched line is REPORTED, not failed" 0 "#UNMATCHED# case.rs:3:" "" <<'EOF'
fn hot(&mut self) {
    // hot-path-alloc-ok: cold, because reasons
    self.buf.push(8);
}
EOF

    check "an inline cfg(test) module is skipped whole" 0 "" "case.rs:" <<'EOF'
fn hot() {
}

#[cfg(test)]
mod tests {
    fn t() {
        let v = Vec::new();
    }
}
EOF

    check "a cfg(test) mod DECLARATION skips one line only" 1 "case.rs:5:" "" <<'EOF'
#[cfg(test)]
mod wiring_tests;

fn hot() {
    let v = Vec::new();
}
EOF

    check "code BEFORE a trailing cfg(test) module still scans" 1 "case.rs:2:" "" <<'EOF'
fn hot() {
    let v = Vec::new();
}

#[cfg(test)]
mod tests {
    fn t() {
        let w = Vec::new();
    }
}
EOF

    check "an error-struct field initializer is exempt" 0 "" "case.rs:" <<'EOF'
fn hot() {
    return Err(E::X {
        topic: topic.to_string(),
        reason: reason.to_string(),
    });
}
EOF

    check "attributes do not consume an armed annotation" 0 "" "case.rs:" <<'EOF'
fn hot() {
    // hot-path-alloc-ok: cold, because reasons
    #[allow(clippy::useless_vec)]
    let v = vec![1];
}
EOF

    # Reproduced end to end without the literal mask: a `cargo fmt`-clean file
    # holding a raw string whose content has a line-initial `fn` at the
    # annotated function's OWN indentation fires a FALSE `OVERRAN`, cancels
    # the suppression, and turns the annotated body's own allocation into a
    # finding. Embedded-source fixtures are ordinary in this tree.
    check "a raw string's line-initial fn is TEXT, not a sibling" 0 "" "OVERRAN" <<'EOF'
impl P {
    // hot-path-alloc-ok-fn: cold: builds a source fixture at startup
    fn build_fixture(&mut self) {
        let src = r#"
    fn embedded_text() {}
"#;
        let v = Vec::new();
        let _ = (src, v);
    }
}
EOF

    # The SAME mask, on the other structural read: a `}` alone inside a raw
    # string at the annotated indentation would have closed the scope EARLY,
    # which is the silent half of this class — the rest of the real body then
    # scans unsuppressed. The oracle is that the body's own allocation, AFTER
    # the literal, is still suppressed.
    check "a raw string's lone brace does not close the scope" 0 "" "case.rs:" <<'EOF'
impl P {
    // hot-path-alloc-ok-fn: cold: builds a source fixture at startup
    fn build_fixture(&mut self) {
        let src = r#"
    }
"#;
        let v = Vec::new();
        let _ = (src, v);
    }
}
EOF

    # BLOCK comments (reproduced on all three shapes). A mask that
    # ends a line's scan on `//` and knows nothing of BLOCK
    # comments lets an ODD number of quotes inside one open `in_str`, which
    # then never closes: every following line reads as text, the terminator is
    # never seen, and a `cargo fmt`-clean file is REJECTED with "the body never
    # closed". Note the failure direction — LOUD, never a fabricated finding —
    # which is the mask's design property holding even when it is wrong.
    check "a block comment's lone quote does not open a string" 0 "" "case.rs:" <<'EOF'
impl P {
    // hot-path-alloc-ok-fn: cold: startup only
    fn single(&mut self) {
        /* the " character is legal prose here */
        let v = Vec::new();
        let _ = v;
    }
}
EOF

    check "a multi-line block comment does not swallow the closing brace" 0 "" "case.rs:" <<'EOF'
impl P {
    // hot-path-alloc-ok-fn: cold: startup only
    fn multi(&mut self) {
        let v = Vec::new();
        let _ = v;
        /* a note that runs on
           and mentions a " quote
           and keeps going */
    }
}
EOF

    # Rust block comments NEST. Closing at the FIRST `*/` leaves the outer
    # comment's tail as code — and its lone quote then opens a string that never
    # closes, which is the shape above all over again.
    check "a nested block comment closes at the right depth" 0 "" "case.rs:" <<'EOF'
impl P {
    // hot-path-alloc-ok-fn: cold: startup only
    fn nested(&mut self) {
        /* outer /* inner */ and the " quote is still prose */
        let v = Vec::new();
        let _ = v;
    }
}
EOF

    # The STRUCTURAL half: a commented-out example carrying a line-initial `fn`
    # AND a `}` at the annotated indentation. Without the mask the first trips
    # the OVERRUN tripwire and the second closes the scope early.
    check "a commented-out fn is TEXT, not a sibling" 0 "" "case.rs:" <<'EOF'
impl P {
    // hot-path-alloc-ok-fn: cold: startup only
    fn documented(&mut self) {
        /* example:
    fn embedded() {}
    }
        */
        let v = Vec::new();
        let _ = v;
    }
}
EOF

    # ANTI-INTERACTION: `'/'` and `'*'` are CHAR literals, not comment
    # openers. The char arm runs before the comment dispatch and consumes the
    # whole literal, so the slash inside one is never seen — this pins that the
    # block-comment arm did not disturb it.
    check "a slash or star CHAR literal is not a comment opener" 0 "" "case.rs:" <<'EOF'
impl P {
    // hot-path-alloc-ok-fn: cold: startup only
    fn chars(&mut self) {
        let a = '/';
        let b = '*';
        let v = Vec::new();
        let _ = (a, b, v);
    }
}
EOF

    # ANTI-TAUTOLOGY for both arms: a REAL sibling `fn` at the annotated
    # indentation — no literal in sight — must still be the loud OVERRUN. The
    # mask narrowed the tripwire; it must not have deleted it.
    check "a REAL sibling fn is still a loud OVERRUN" 1 "OVERRAN" "" <<'EOF'
impl P {
    // hot-path-alloc-ok-fn: cold
    fn build_fixture(&mut self) {
        let v = Vec::new();
        let _ = v;
      }

    fn sibling(&mut self) {
        let leak = Vec::new();
        let _ = leak;
    }
}
EOF

    # Reproduced (`--self-test --typo` exits 0): a
    # self-test dispatch that returns BEFORE argv is validated lets a malformed
    # validation command report a PASSED self-test.
    #
    # Driven as a SUBPROCESS — the flag parser cannot be tested by the awk
    # harness, which never sees argv. `SELFTEST_NO_RECURSE` bounds it:
    # if the guard under test is REMOVED, the child falls through to
    # `self_test`, which sees the sentinel and skips these arms rather than
    # spawning itself again. The child then exits 0, the arm sees 0 != 2, and
    # removing the guard fails LOUDLY instead of forking unbounded.
    if [[ -z "${SELFTEST_NO_RECURSE:-}" ]]; then
        check_argv() {
            local name="$1" want_rc="$2"
            shift 2
            local o s
            set +e
            o="$(SELFTEST_NO_RECURSE=1 "${BASH_SOURCE[0]}" "$@" 2>&1)"
            s=$?
            set -e
            if [[ "$s" -ne "$want_rc" ]]; then
                echo "SELF-TEST FAIL [$name]: exit $s, wanted $want_rc" >&2
                printf '%s\n' "$o" >&2
                rc=1
                return
            fi
            echo "  ok  $name"
        }
        check_argv "an argument AFTER --self-test is an ERROR" 2 --self-test --bogus
        check_argv "…and so is a typo of it" 2 --self-tets
        check_argv "a lone unknown argument is an ERROR" 2 --typo
    fi

    if [[ "$rc" -eq 0 ]]; then
        echo "check_hot_path_allocs.sh --self-test: OK"
    fi
    return "$rc"
}

# A self-test dispatch that returns before argv is validated lets a malformed
# validation command report a passed self-test (reproduced: `--self-test --typo`
# exits 0) — the same defect as the `--self-tets` typo
# below, in the one mode whose entire job is to prove the
# script still works. `--self-test` takes no further arguments; anything after
# it is rejected LOUDLY rather than ignored.
if [[ "${1:-}" == "--self-test" ]]; then
    if [[ $# -gt 1 ]]; then
        echo "check_hot_path_allocs.sh: --self-test takes no further arguments (got: $2)" >&2
        echo "usage: check_hot_path_allocs.sh [--self-test]" >&2
        exit 2
    fi
    self_test
    exit $?
fi

# An unrecognized argument must not fall through to
# the main lint, which prints its OK line and exits 0 — a typo of
# `--self-test` (`--self-tets`, `--selftest`) would read as a PASSED self-test while
# the mode the operator asked for never ran. The repo's own convention is loud
# rejection of an unknown value (the drain-discipline seam warns on any
# unrecognized value; `--policy deadline_ms=N` is rejected as unknown).
if [[ $# -gt 0 ]]; then
    echo "check_hot_path_allocs.sh: unknown argument: $1" >&2
    echo "usage: check_hot_path_allocs.sh [--self-test]" >&2
    exit 2
fi

# ── the wake.rs exclusion is a CROSS-FILE invariant, so guard it ─────────────
# `wake.rs` is excluded because `scheduler/mod.rs` declares `#[cfg(test)] mod
# wake;` — and BOTH files' comments say that gate is temporary ("test-gated
# UNTIL its reactor consumer lands"). Without this check, the
# day the gate is removed, `wake.rs` becomes shipping per-step scheduler code
# that stays permanently unscanned, and no existing guard trips — the
# empty-target check is satisfied by the directory's other files, and the awk
# `#[cfg(test)]` skip cannot see a gate living in the DECLARING module (which is
# why the file was hand-excluded in the first place). That defeats this script's
# own "a new file in a scanned directory is linted BY DEFAULT" claim for exactly
# the file most likely to change category. This tripwire makes the removal loud.
WAKE_DECL_FILE="$REPO_ROOT/crates/cerulion_core/src/scheduler/mod.rs"
if [[ -f "$WAKE_DECL_FILE" ]]; then
    if ! grep -qE '^[[:space:]]*#\[cfg\(test\)\][[:space:]]*$' <(grep -B1 -E '^[[:space:]]*(pub[[:space:]]+)?mod wake;' "$WAKE_DECL_FILE" || true); then
        echo "check_hot_path_allocs.sh: SCHEDULER_EXCLUDE lists wake.rs because scheduler/mod.rs declares it \`#[cfg(test)] mod wake;\`, but that gate is no longer there." >&2
        echo "  wake.rs is now shipping scheduler code and must be SCANNED: remove it from SCHEDULER_EXCLUDE and adjudicate its allocations." >&2
        exit 2
    fi
else
    echo "check_hot_path_allocs.sh: cannot verify the wake.rs exclusion — $WAKE_DECL_FILE is missing." >&2
    exit 2
fi

failures=0
known_count=0
known_report=""
unmatched_count=0
unmatched_report=""

for rel in "${HOT_PATH_FILES[@]}"; do
    file="$REPO_ROOT/$rel"

    # awk does the line-by-line classification so we get one pass per file
    # with preceding-line allow tracking. Its stdout carries two kinds of line:
    # a FAILURE (`<rel>:<line>: <src>`) and a `#KNOWN# `-prefixed record of a
    # real hot-path allocation that is deliberately not fixed here. They are
    # split below so a known allocation is REPORTED loudly without failing.
    out=""
    if ! out="$(awk -v rel="$rel" -v KNOWN_PREFIX="$KNOWN_PREFIX" -v UNMATCHED_PREFIX="$UNMATCHED_PREFIX" "$AWK_PROG" "$file")"; then
        failures=1
    fi
    if [[ -n "$out" ]]; then
        # Failures first (stdout, as before), then bank the known records.
        printf '%s\n' "$out" | { grep -v -e "^$KNOWN_PREFIX" -e "^$UNMATCHED_PREFIX" || true; }
        this_known="$(printf '%s\n' "$out" | { grep -- "^$KNOWN_PREFIX" || true; } | sed "s/^$KNOWN_PREFIX/    /")"
        if [[ -n "$this_known" ]]; then
            known_report+="$this_known"$'\n'
            known_count=$((known_count + $(printf '%s\n' "$this_known" | wc -l | tr -d ' ')))
        fi
        this_unmatched="$(printf '%s\n' "$out" | { grep -- "^$UNMATCHED_PREFIX" || true; } | sed "s/^$UNMATCHED_PREFIX/    /")"
        if [[ -n "$this_unmatched" ]]; then
            unmatched_report+="$this_unmatched"$'\n'
            unmatched_count=$((unmatched_count + $(printf '%s\n' "$this_unmatched" | wc -l | tr -d ' ')))
        fi
    fi
done

if [[ "$known_count" -ne 0 ]]; then
    {
        echo
        echo "KNOWN HOT-PATH ALLOCATIONS ($known_count) — recorded, NOT claimed cold:"
        printf '%s' "$known_report"
        echo
        echo "Each carries a \`// hot-path-alloc-known:\` reason naming why it is hot"
        echo "and where the fix is tracked. This banner exists so widening the lint"
        echo "over a file with real findings cannot launder them into silence."
    } >&2
fi

if [[ "$unmatched_count" -ne 0 ]]; then
    {
        echo
        echo "ANNOTATIONS ON UNMATCHED LINES ($unmatched_count) — informational, not a failure:"
        printf '%s' "$unmatched_report"
        echo
        echo "Each of these carries an allocation annotation on a line this lint's"
        echo "own pattern set does NOT match, so the marker arms NOTHING. Two"
        echo "readings, and the script cannot tell them apart: the annotation is"
        echo "STALE (its allocation was refactored away, or it duplicates one"
        echo "already carried by the real allocating line), or it is TRUTHFUL and"
        echo "the pattern set is blind to that shape."
        echo
        echo "This list is EMPTY on a clean tree, which is what makes"
        echo "it a signal: \`.extend(..)\` and \`.reserve(..)\` are"
        echo "matched, and a stale or duplicate marker belongs in"
        echo "plain prose. \`.push(..)\` is the one shape known to"
        echo "be unmatched ON PURPOSE (25 sites, all pushes into cleared-and-"
        echo "reused buffers — see the header), so an annotation on a \`.push(\`"
        echo "line will land here and should be plain prose instead."
    } >&2
fi

if [[ "$failures" -ne 0 ]]; then
    cat >&2 <<'EOF'

check_hot_path_allocs.sh: unallowed allocation pattern(s) on the hot path.

If a finding is a genuine cold-path allocation (constructor, error path,
late-joiner history delivery, test-only helper), annotate it:

    // hot-path-alloc-ok: <why this never runs on the hot path>

If it is on the publish/receive hot path: do not allocate. Loan and write
directly (Principle #10), or share via Arc instead of cloning.
EOF
    exit 1
fi

if [[ "$known_count" -ne 0 ]]; then
    echo "check_hot_path_allocs.sh: OK (${#HOT_PATH_FILES[@]} files scanned; \
$known_count known hot-path allocation(s) recorded — see the banner above)"
else
    echo "check_hot_path_allocs.sh: OK (${#HOT_PATH_FILES[@]} files clean)"
fi
