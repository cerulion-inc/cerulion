#!/usr/bin/env bash
# check_pr_title.sh — the pull-request naming gate.
#
# WHAT IT ENFORCES
#   TITLE   `<type>(<scope>)!: <description>`
#           type   ∈ feat fix docs test refactor ci chore perf style build
#           scope  optional, `[a-z0-9_-]+`; several may be listed
#                  comma-separated with no spaces (`fix(core,cli): …`) —
#                  26 of the 200 merges before this gate landed used the
#                  comma form, so it is the repo's convention, not an add-on
#           `!`    optional (the conventional-commits breaking marker)
#           a non-empty description after `: `
#
#   BRANCH  `<type>/<kebab-slug>`
#           the same type vocabulary, then a lowercase kebab slug.
#           NO personal prefix (the retired two-letter forms), and NO tracker
#           id anywhere in the name.
#
# WHY THE BRANCH RULE FORBIDS A TRACKER ID. A branch name is permanent, public
# and quoted in every merge commit, while a tracker id means nothing to anyone
# outside the tracker — and the repo's own agent-docs gate
# (`scripts/check_agents_md.sh`) already treats that token shape as internal
# vocabulary that must not ship in public documentation. The id belongs in the
# PR BODY, where it links, is editable, and is not baked into history.
#
# GRANDFATHERED BRANCHES. Renaming the head branch of a PR CLOSES the PR on
# GitHub — the review threads, the bot summaries and the CI history go with it —
# so a branch that predates this rule can be accepted VERBATIM (exact match, no
# pattern) and nothing else is: a third name of the same shape still fails.
# Each name is BOUND TO ITS
# PULL REQUEST NUMBER: a head ref is fork-controlled text (any fork can push a
# branch of the same name and open a PR from it), while a PR number is minted
# by this repository once and never reused — so the exemption needs BOTH, and
# with no `--pr` it grants nothing. The list is empty, as intended; do not add
# to it.
#
# WHY THIS IS AN IN-REPO SCRIPT AND NOT A MARKETPLACE ACTION. Every third-party
# action is code this repo executes with its own token; the two the ecosystem
# offers for this are a hundred lines of JavaScript each for a regex. It is
# also runnable locally (`--self-test`, or just pass a title and a branch),
# reviewable as a diff, and covered by `shellcheck scripts/*.sh`.
#
# USAGE
#   check_pr_title.sh --title "<title>" --branch "<branch>" [--pr <number>]
#       --pr is the pull request NUMBER (CI passes it from the event payload);
#       it is consulted only by the grandfather list and is otherwise unused.
#   check_pr_title.sh --self-test        # the oracle table; runs no I/O
#
# Exit 0 = both fields conform. Exit 1 = one `VIOLATION:` line per problem,
# each naming the offending value and the fix. Portable: bash 3.2+ (macOS),
# BSD and GNU userland, no GNU-only flags.

set -uo pipefail

# The ONE type vocabulary, shared by both patterns so a title and a branch can
# never disagree about what a type is.
TYPES='feat|fix|docs|test|refactor|ci|chore|perf|style|build'

TITLE_RE="^(${TYPES})(\([a-z0-9_-]+(,[a-z0-9_-]+)*\))?!?: .+"
BRANCH_RE="^(${TYPES})/[a-z0-9][a-z0-9-]*$"
# A tracker id ANYWHERE in the branch name, case-insensitively, with or
# without a `-` or `_` between the letters and the digits.
#
# The LEFT BOUNDARY is load-bearing: without it the pattern matches the tail of
# any word ending in those three letters, so `perf/tracer-3-fix` and
# `fix/reducer-2-overflow` are rejected as carrying a tracker id, naming a
# remedy ("put it in the PR BODY") that makes no sense for either. `\b` is not
# portable across BSD and GNU grep, so the boundary is spelled as an explicit
# alternation. No RIGHT boundary is needed — the digits are already anchored by
# the required `[0-9]+`, so `feat/certificate-v1` cannot match.
TRACKER_RE='(^|[^a-zA-Z0-9])[cC][eE][rR][-_]?[0-9]+'

# The grandfather list (see the header): one `<pr-number> <branch>` pair per
# line, and EMPTY today. The lookup builds the same pair from the inputs and
# matches it with `grep -qxF`: whole-line and fixed-string, so a name can never
# be a prefix, a glob or a regex of another, the number cannot be a prefix of
# a longer one, and `-e` keeps a value starting with `-` from being read as an
# option.
GRANDFATHERED_BRANCHES=''

violations=0
fail() {
    printf 'VIOLATION: %s\n' "$1"
    violations=$((violations + 1))
}

# validate_inputs <title> <branch> <pr-given: 0|1> <pr> — the caller-error gate.
# Exit 2 (not a VIOLATION) because these are malformed INVOCATIONS, not
# non-conforming PRs: the workflow passing an expression, an empty `--pr`, or
# a value with an embedded newline. Every check here is a shell `case`, which
# matches the WHOLE string — `grep` matches a LINE, so a multi-line value
# whose first line conforms would slip through a grep-only check (and a
# newline inside a `grep -e` pattern silently becomes TWO patterns).
validate_inputs() {
    _vt=$1
    _vb=$2
    _vpg=$3
    _vp=$4
    # A title may legitimately carry a tab or other odd byte (the TITLE_RE's
    # `.+` accepts it); only a LINE BREAK is refused here, because that is the
    # one shape the line-matching checks below cannot see.
    _vnl='
'
    _vcr=$(printf '\r')
    case "$_vt" in *"$_vnl"*|*"$_vcr"*)
        printf 'check_pr_title: the title contains a line break\n' >&2
        return 2 ;;
    esac
    case "$_vb" in *[[:cntrl:][:space:]]*)
        printf 'check_pr_title: the branch name contains whitespace or a control character\n' >&2
        return 2 ;;
    esac
    if [ "$_vpg" -eq 1 ]; then
        case "$_vp" in
            ''|*[!0-9]*|0*)
                printf 'check_pr_title: --pr must be a pull request number, got: %s\n' "$_vp" >&2
                return 2 ;;
        esac
    fi
    return 0
}

check_title() {
    _t=$1
    if [ -z "$_t" ]; then
        fail "the PR title is empty"
        return
    fi
    if ! printf '%s' "$_t" | grep -qE "$TITLE_RE"; then
        fail "PR title does not match '<type>(<scope>): <description>' — got: '$_t'
    types: ${TYPES//|/, }
    scope is optional and lowercase, e.g. 'feat(transport): add the wake set'
    several scopes may be comma-separated with no spaces, e.g. 'fix(core,cli): …'
    a breaking change may carry '!', e.g. 'refactor(core)!: drop the old seam'"
    fi
}

check_branch() { # <branch> [pr-number]
    _b=$1
    _pr=${2:-}
    if [ -z "$_b" ]; then
        fail "the head branch name is empty"
        return
    fi
    # A grandfathered (PR, name) pair is accepted verbatim and the rest of the
    # branch rule is skipped for it — the name would fail BOTH the tracker
    # check and the shape check, and each remedy ("rename") is exactly the
    # action that closes the PR. The pair needs the PR number: without one the
    # name is checked like any other. The TITLE is still checked by the
    # caller: the exemption covers the name that cannot change, not the title
    # that can.
    if [ -n "$_pr" ] && printf '%s\n' "$GRANDFATHERED_BRANCHES" | grep -qxF -e "$_pr $_b"; then
        return
    fi
    # The tracker check runs FIRST and reports its own remedy: a branch whose
    # slug carries an id still matches the shape rule, so a shape-only message
    # would tell the author their branch is fine.
    if printf '%s' "$_b" | grep -qE "$TRACKER_RE"; then
        fail "head branch '$_b' carries a tracker id — put it in the PR BODY instead.
    A branch name is permanent and public; the id links from the body and means
    nothing outside the tracker. Rename to '<type>/<what-it-does>'."
    fi
    if ! printf '%s' "$_b" | grep -qE "$BRANCH_RE"; then
        fail "head branch '$_b' does not match '<type>/<kebab-slug>'
    types: ${TYPES//|/, }
    lowercase letters, digits and hyphens only, e.g. 'fix/held-input-replay'
    personal prefixes are retired — the type says what the branch is for"
    fi
}

# ---------------------------------------------------------------------------
# --self-test: the oracle table. Every row states the expected verdict, and
# the accept rows exist so a pattern tightened into uselessness (`^$`, or a
# type list emptied) fails here rather than rejecting every future PR.
# ---------------------------------------------------------------------------
self_test() {
    _fails=0
    _run=0

    _expect_title() { # <expected: ok|bad> <title>
        _run=$((_run + 1))
        violations=0
        check_title "$2" >/dev/null
        if [ "$1" = ok ] && [ "$violations" -ne 0 ]; then
            printf 'SELF-TEST FAIL: title should be ACCEPTED: %s\n' "$2" >&2
            _fails=$((_fails + 1))
        fi
        if [ "$1" = bad ] && [ "$violations" -eq 0 ]; then
            printf 'SELF-TEST FAIL: title should be REJECTED: %s\n' "$2" >&2
            _fails=$((_fails + 1))
        fi
    }
    _expect_inputs() { # <expected: ok|bad> <title> <branch> <pr-given> <pr>
        _run=$((_run + 1))
        if validate_inputs "$2" "$3" "$4" "$5" 2>/dev/null; then _got=ok; else _got=bad; fi
        if [ "$1" != "$_got" ]; then
            printf 'SELF-TEST FAIL: inputs should be %s: title=%s branch=%s pr_given=%s pr=%s\n' "$1" "$2" "$3" "$4" "$5" >&2
            _fails=$((_fails + 1))
        fi
    }
    _expect_branch() { # <expected: ok|bad> <branch> [pr-number]
        _run=$((_run + 1))
        violations=0
        check_branch "$2" "${3:-}" >/dev/null
        if [ "$1" = ok ] && [ "$violations" -ne 0 ]; then
            printf 'SELF-TEST FAIL: branch should be ACCEPTED: %s\n' "$2" >&2
            _fails=$((_fails + 1))
        fi
        if [ "$1" = bad ] && [ "$violations" -eq 0 ]; then
            printf 'SELF-TEST FAIL: branch should be REJECTED: %s\n' "$2" >&2
            _fails=$((_fails + 1))
        fi
    }

    # --- titles: accepted -------------------------------------------------
    _expect_title ok  'feat: add the wake set'
    _expect_title ok  'fix(transport): drain the publisher listener'
    _expect_title ok  'ci: shard the test jobs'
    _expect_title ok  'refactor(core)!: drop the in-process backend'
    _expect_title ok  'test(cli_engine): gate the coverage walk'
    _expect_title ok  'chore(demos-go2): refresh the lockfile'
    _expect_title ok  'perf: cut a receive from the data hop'
    _expect_title ok  'style: rustfmt the excluded trees'
    _expect_title ok  'build: pin the nightly'
    _expect_title ok  'docs: correct the liveness note'
    _expect_title ok  'fix(core,cli): span two crates'
    _expect_title ok  'fix(core,bagd,cli): span three crates'
    _expect_title ok  'refactor(core,replay)!: breaking, two scopes'
    # --- titles: rejected, each naming what it catches ---------------------
    _expect_title bad ''                                  # empty
    _expect_title bad 'add the wake set'                  # no type
    _expect_title bad 'feat add the wake set'             # no colon
    _expect_title bad 'feat:'                             # no description
    _expect_title bad 'feat: '                            # whitespace-only description
    _expect_title bad 'Feat: add the wake set'            # capitalised type
    _expect_title bad 'feature: add the wake set'         # not in the vocabulary
    _expect_title bad 'feat(Transport): add it'           # capitalised scope
    _expect_title bad 'feat(transport) add it'            # scope without a colon
    _expect_title bad ' feat: add it'                     # leading space
    _expect_title bad 'wip: add it'                       # a type nobody agreed to
    _expect_title bad 'fix(core,): trailing comma'        # empty scope after comma
    _expect_title bad 'fix(,cli): leading comma'          # empty scope before comma
    _expect_title bad 'fix(core, cli): spaced list'       # space after comma
    _expect_title bad 'fix(core,,cli): doubled comma'     # empty scope mid-list

    # THE TRACKER-ID SAMPLES ARE DERIVED FROM TRACKER_RE, NOT TYPED. A gate that
    # spells the token it bans is itself a leak (the agent-docs gate derives its
    # fixture the same way). Drop the left-boundary group, keep the first letter
    # of each two-letter class, fill the optional separator with a hyphen and
    # the digit class with one digit; every variant below is spelled from that
    # one sample.
    _tid=$(printf '%s' "$TRACKER_RE" \
        | sed -E 's/^\(\^\|\[\^a-zA-Z0-9\]\)//' \
        | sed -E 's/\[([a-z])[A-Z]\]/\1/g' \
        | sed -E 's/\[-_\]\?/-/' \
        | sed -E 's/\[0-9\]\+/7/')
    _tid_head=$(printf '%s' "$_tid" | cut -c1)
    _tid_rest=$(printf '%s' "$_tid" | cut -c2-)
    _tid_cap="$(printf '%s' "$_tid_head" | tr '[:lower:]' '[:upper:]')$_tid_rest"
    _tid_tail_upper="$_tid_head$(printf '%s' "$_tid_rest" | tr '[:lower:]' '[:upper:]')"
    _tid_joined=$(printf '%s' "$_tid" | tr -d '-')
    _tid_underscore=${_tid/-/_}
    _tid_nine=${_tid%7}9
    # Precondition: the derivation must itself be a tracker id, or every
    # tracker row below is vacuous.
    _run=$((_run + 1))
    if ! printf '%s' "$_tid" | grep -qE "$TRACKER_RE"; then
        printf 'SELF-TEST FAIL: could not derive a tracker-id sample from TRACKER_RE (got: %s)\n' "$_tid" >&2
        _fails=$((_fails + 1))
    fi

    # --- branches: accepted -----------------------------------------------
    _expect_branch ok  'ci/hardening-workflows-and-jobs'
    _expect_branch ok  'fix/held-input-replay'
    _expect_branch ok  'feat/wake-set'
    _expect_branch ok  'chore/lockfile-refresh'
    _expect_branch ok  'test/coverage-walk'
    _expect_branch ok  'perf/one-fewer-receive'
    # --- branches: rejected ------------------------------------------------
    _expect_branch bad ''                                 # empty
    _expect_branch bad 'wake-set'                         # no type
    _expect_branch bad 'feature/wake-set'                 # not in the vocabulary
    _expect_branch bad 'CI/wake-set'                      # capitalised type
    _expect_branch bad 'feat/Wake-Set'                    # capitalised slug
    _expect_branch bad 'feat/wake_set'                    # underscore, not kebab
    _expect_branch bad 'feat/wake/set'                    # a second segment
    _expect_branch bad 'feat/-leading-hyphen'             # slug starts with a hyphen
    _expect_branch bad 'zz/wake-set'                      # a two-letter personal prefix (the retired shape)
    _expect_branch bad 'qx/wake-set'                      # another: the type vocabulary is the only first segment
    _expect_branch bad 'ab/wake-set'                      # and a third
    # the grandfather list is empty (see the header), so a PR number on its
    # own rescues nothing: neither an ordinary offender nor a tracker id.
    _expect_branch bad 'zz/wake-set'                     596
    _expect_branch bad "feat/${_tid}-wake-set"           596
    # tracker ids, in every spelling seen in this repo's history: hyphen,
    # nothing, underscore, at the tail, and mixed case (the two mixed forms
    # between them put every letter of the case-insensitive class in upper
    # case once)
    _expect_branch bad "feat/${_tid}-wake-set"
    _expect_branch bad "feat/${_tid_cap}-wake-set"
    _expect_branch bad "feat/${_tid_tail_upper}-wake-set"
    _expect_branch bad "feat/${_tid_joined}-wake-set"
    _expect_branch bad "fix/wake-set-${_tid_nine}"
    _expect_branch bad "feat/${_tid_underscore}-wake-set"
    # ...and words that merely END in those letters are NOT tracker ids. The
    # boundary-less pattern rejected every one of these.
    _expect_branch ok  'perf/tracer-3-overhead'
    _expect_branch ok  'fix/reducer-2-overflow'
    _expect_branch ok  'refactor/soccer-2-demo'
    _expect_branch ok  'feat/certificate-v1'

    # --- the invocation gate (exit 2, not a VIOLATION) ----------------------
    _nl='
'
    _expect_inputs ok  'feat: x' 'feat/x' 0 ''            # --pr omitted: fine
    _expect_inputs ok  'feat: x' 'feat/x' 1 '596'
    _expect_inputs bad 'feat: x' 'feat/x' 1 ''            # explicitly empty --pr
    _expect_inputs bad 'feat: x' 'feat/x' 1 'abc'
    _expect_inputs bad 'feat: x' 'feat/x' 1 '0596'        # leading zero is not a PR number
    _expect_inputs bad 'feat: x' 'feat/x' 1 '0'
    _expect_inputs bad 'feat: x' 'feat/x' 1 '-1'
    _expect_inputs bad 'feat: x' 'feat/x' 1 "596${_nl}junk"   # a line-matcher would pass this
    _expect_inputs bad 'feat: x' 'feat/x' 1 ' 596'
    _expect_inputs bad "feat: x${_nl}garbage" 'feat/x' 0 ''   # multi-line title
    _expect_inputs bad 'feat: x' "feat/x${_nl}bad" 0 ''       # multi-line branch
    _expect_inputs bad 'feat: x' 'feat/x y' 0 ''              # whitespace in a ref
    _expect_inputs ok  'feat(core,cli)!: add it — with unicode' 'fix/held-input-replay' 1 '12'
    _expect_inputs ok  "feat: a tab	is not a line break" 'feat/x' 0 ''
    _expect_inputs bad "feat: x$(printf '\r')y" 'feat/x' 0 ''       # a CR is

    violations=0
    if [ "$_fails" -ne 0 ]; then
        printf 'check_pr_title --self-test: FAIL (%d of %d rows)\n' "$_fails" "$_run" >&2
        exit 1
    fi
    printf 'check_pr_title --self-test: OK (%d rows)\n' "$_run"
    exit 0
}

# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
TITLE=""
BRANCH=""
PR=""
PR_GIVEN=0
while [ "$#" -gt 0 ]; do
    case "$1" in
        --self-test) self_test ;;
        --title|--branch|--pr)
            # `shift 2` with ONE argument left shifts NOTHING and returns
            # nonzero, and this loop does not run under `set -e`: a trailing
            # `--title` spun forever instead of complaining. An option missing
            # its value is a caller error and says so.
            [ "$#" -ge 2 ] || {
                printf 'check_pr_title: %s needs a value\n' "$1" >&2
                exit 2
            }
            case "$1" in
                --title)  TITLE=$2 ;;
                --branch) BRANCH=$2 ;;
                --pr)     PR=$2; PR_GIVEN=1 ;;
            esac
            shift 2
            ;;
        -h|--help) sed -n '/^# USAGE/,/^$/p' "$0"; exit 0 ;;
        *) printf 'check_pr_title: unknown argument %s\n' "$1" >&2; exit 2 ;;
    esac
done

# Both fields are REQUIRED. An absent one must not read as "nothing to check":
# a gate whose inputs went missing has to say so, or a workflow that stops
# passing them turns green forever.
if [ -z "$TITLE" ] && [ -z "$BRANCH" ]; then
    printf 'check_pr_title: pass --title and --branch (or --self-test)\n' >&2
    exit 2
fi

# Malformed invocation (an explicitly empty or non-decimal `--pr`, a value
# with an embedded newline) is a caller error, exit 2 — never a pass.
validate_inputs "$TITLE" "$BRANCH" "$PR_GIVEN" "$PR" || exit 2

check_title "$TITLE"
check_branch "$BRANCH" "$PR"

if [ "$violations" -gt 0 ]; then
    printf 'check_pr_title: FAIL (%d violation(s))\n' "$violations"
    exit 1
fi
printf 'check_pr_title: OK — title and branch both conform\n'
exit 0
