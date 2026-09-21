#!/usr/bin/env bash
# check_agents_md.sh — the agent-docs gate.
#
# Context files are loaded by coding agents with SILENT truncation and
# nearest-file-wins chaining, so the repo must be the alarm. This script
# enforces, over the whole tree (skipping .git/, target/, notes/,
# node_modules/):
#
#   1. Size budgets:
#        root AGENTS.md              <= 190 lines AND <= 13000 bytes
#        every other AGENTS.md       <=  60 lines AND <=  4096 bytes
#   2. Chain budget: root AGENTS.md + the LARGEST non-root AGENTS.md
#      <= 24576 bytes (the worst-case chain a tool actually loads,
#      leaving headroom under the 32 KiB combined-context ceiling).
#   3. Shims: every directory holding an AGENTS.md also holds a CLAUDE.md —
#        crate dirs: content exactly `@AGENTS.md` (one line, nothing else);
#        repo root:  first line exactly `@AGENTS.md` (a short tool-specific
#                    addendum may follow).
#   4. Banned tokens: zero internal vocabulary in any AGENTS.md, the root
#      CLAUDE.md addendum, or docs/internals/*.md. Case-insensitive,
#      word-boundary-aware (grep -w), so a banned fragment inside a longer
#      identifier does not false-positive. Three tiers:
#        - GENERIC patterns (shipped below): tracker-token shapes, session
#          refs, role words, LAN/VPN IPs, the overlay-network domain.
#        - NAME-CLASS list (people, machines, tools, private repos): loaded
#          from an UNTRACKED file — $AGENTS_TOKENS_FILE or .agents-tokens.local
#          (gitignored; the private overlay's setup links it). The list itself
#          is internal vocabulary, so it never ships in this script. When the
#          file is absent the name-class scan is skipped with an INFO line
#          (public CI enforces the generic tier; maintainer machines and
#          private CI enforce both).
#        - PRIVATE-LIST tier (machine names, logins, retired personal branch
#          prefixes): the leak guard's private pattern list, read by the leak
#          guard's OWN loader (env LEAK_PATTERNS, then the file named by
#          LEAK_PATTERNS_FILE, then the default file under the user's config
#          directory), so the two gates cannot disagree about the sources or
#          the entry grammar. Nothing name-shaped is spelled in this script.
#          When no list is loaded the tier is skipped with a NOTE (public CI;
#          the leak guard's private tier covers that content where it runs).
#          An unreadable list or a pattern that does not compile is a
#          VIOLATION, never a clean file.
#   5. Symlinked AGENTS.md/CLAUDE.md files fail outright — a symlink dodges
#      every budget and token check while tools happily read the target.
#   6. HOST IDENTITY in the operator-facing files (`tools/scripts/*.sh`,
#      `.github/workflows/*.yml`): a NARROWER scan, for machine names and
#      overlay-network addresses only. The address shapes ship here; the
#      machine names themselves come from the private list (its host
#      entries), skipped with the same NOTE when no list is loaded.
#
#      It is a separate rule rather than a widened scope because the tiers
#      above are about tracker-id shapes, a different hazard from a hostname;
#      this rule is the one that ships for those files: hostnames and
#      overlay-network addresses never enter the repo.
#
#      Tracked rather than trusted because that surface is the one an operator
#      doc is most tempted to name a machine in ("ssh to <host>, then run..."), and
#      a hostname is the one detail that turns published CI tooling into a map
#      of somebody's network.
#
# `--self-test` drives every pattern in every tier against a fixture that
# contains one of each, and a clean fixture that must produce nothing. A
# banned-token gate whose own regex has stopped matching looks exactly like a
# clean tree — which is the failure mode the whole file exists to prevent.
#
# Files that do not exist are simply not checked (mid-migration safe), but a
# directory that HAS an AGENTS.md and no CLAUDE.md shim always fails, and an
# unreadable checked file is a violation (a gate must not fail open).
#
# Exit 0 = clean. Exit 1 = one "VIOLATION:" line per problem on stdout.
# Portable: bash 3.2+ (macOS), BSD and GNU userland. No GNU-only flags.

set -u

cd "$(dirname "$0")/../.." || exit 2

SELF_TEST=no
while [ "$#" -gt 0 ]; do
    case "$1" in
        --self-test) SELF_TEST=yes; shift ;;
        -h|--help)
            # The header ENDS where the comments end, computed rather than
            # counted: a hardcoded line range silently truncates `--help` the
            # moment the header grows, which is how it came to stop two rules
            # short of the end.
            awk 'NR == 1 { next } /^#/ { print; next } { exit }' "$0"
            exit 0
            ;;
        *)
            # Unknown arguments used to be silently IGNORED, which is how a
            # typo'd flag turns a gate into a no-op that still prints OK.
            printf 'check_agents_md: unknown argument %s\n' "$1" >&2
            exit 2
            ;;
    esac
done

ROOT_MAX_LINES=190
ROOT_MAX_BYTES=13000
CRATE_MAX_LINES=60
CRATE_MAX_BYTES=4096
CHAIN_MAX_BYTES=24576

violations=0
fail() {
    printf 'VIOLATION: %s\n' "$1"
    violations=$((violations + 1))
}

# ---------------------------------------------------------------------------
# 4: banned tokens over all public agent docs + contributor dossiers.
#
# GENERIC tier (shipped here): shape-based extended regexes, matched
# case-insensitively with whole-word semantics (grep -w: a match may not be
# adjacent to a word character, so fragments inside longer identifiers do not
# hit). Deliberately contains NO specific names — a scrub gate that lists
# what it scrubs is itself a leak. Name-class tokens (people, machines,
# internal tools and repos) load from the untracked file below.
#
# NOTE 1: grep -w rejects a match FOLLOWED by a word character even when the
# match itself ends in a non-word character, so a prefix-shaped entry (a
# machine-name prefix, a branch prefix) cannot live in this whole-word tier.
# Those come from the private list below, whose loader carries its own
# boundary rules (BSD grep has no \b, GNU grep has no [[:<:]]).
# NOTE 2: literals start as a bracket class ('[f]ounder') so this list never
# matches itself if the script is ever added to its own scan scope.
# ---------------------------------------------------------------------------
banned_tokens=(
    '[f]ounder'
    '[p]t[0-9]+'
    '[l]inear\.app'
    '[c]er-[0-9]+'
    '[t]s\.net'
    '192\.168\.[0-9]{1,3}\.[0-9]{1,3}'
    '10\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}'
    '100\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}'
)

# PRIVATE-LIST tier: the names themselves. Machine names (host entries),
# logins (login entries) and retired personal branch prefixes (hygiene
# entries) come from the leak guard's private pattern list, and the leak
# guard's own loader reads it (`load_private` in tools/scripts/leak_scan.py:
# env LEAK_PATTERNS, then the file named by LEAK_PATTERNS_FILE, then the
# default file under the user's config directory), so this gate and the leak
# guard cannot disagree about the sources or the entry grammar. Nothing
# name-shaped is spelled in this script: a scrub gate that lists what it
# scrubs is itself a leak, even behind one-letter bracket disguises.
#
# One python process per tier. Prints one TAB-separated record per finding:
#   hit<TAB><tier><TAB><pattern label><TAB><file>:<line>:<masked line>
#   scanfail<TAB><tier><TAB>-<TAB><reason>   (an unreadable list, a pattern that
#                                             does not compile, no python3, an
#                                             unreadable file)
#   nolist<TAB><tier><TAB>-<TAB><reason>     (no usable private list: a NOTE)
#   note<TAB><tier><TAB>-<TAB><text>         (a loader warning, e.g. file mode)
# The matched text is never printed: the leak guard's own masker replaces
# every private span in the reported line with the pattern's label.
private_scan() {  # <tier> <comma-separated tags> <file>...
    _ps_tier=$1
    _ps_tags=$2
    shift 2
    [ "$#" -gt 0 ] || return 0
    if ! command -v python3 >/dev/null 2>&1; then
        printf 'scanfail\t%s\t-\tpython3 is not available, so the private-list tier could not run\n' "$_ps_tier"
        return 0
    fi
    python3 -B - "$_ps_tier" "$_ps_tags" "$@" <<'PY' || printf 'scanfail\t%s\t-\tthe private-list scan exited nonzero\n' "$_ps_tier"
import os, signal, sys
signal.signal(signal.SIGPIPE, signal.SIG_DFL)  # a closed pipe ends this quietly, as it ends grep
sys.path.insert(0, os.path.join('tools', 'scripts'))
tier, tags = sys.argv[1], set(sys.argv[2].split(','))
files = sys.argv[3:]
def rec(kind, label, detail):
    print('%s\t%s\t%s\t%s' % (kind, tier, label, detail.replace('\t', ' ').replace('\n', ' ')))
try:
    import leak_scan
    pats, kind, warnings = leak_scan.load_private(os.environ, os.environ.get('HOME'))
except Exception as exc:  # the loader REFUSES: unreadable file, a pattern that does not compile
    rec('scanfail', '-', str(exc))
    sys.exit(0)
for w in warnings:
    rec('note', '-', w)
if not pats:
    rec('nolist', '-', 'no private list loaded (sources, in order: env LEAK_PATTERNS, the file named by '
        'LEAK_PATTERNS_FILE, the default file under the user config directory)')
    sys.exit(0)
pats = [p for p in pats if p.tag in tags]
if not pats:
    rec('nolist', '-', 'the private list (%s) has no entry tagged %s'
        % (kind, ' or '.join('@' + t for t in sorted(tags))))
    sys.exit(0)
ps = leak_scan.PrivateSet(pats)
for path in files:
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as fh:
            lines = fh.read().split('\n')
    except OSError as exc:
        rec('scanfail', '-', '%s is unreadable (%s)' % (path, exc.__class__.__name__))
        continue
    for n, line in enumerate(lines, 1):
        for p in pats:
            if any(p.rx.search(v) for v in leak_scan.text_views(line)):
                rec('hit', p.label(), '%s:%d:%s' % (path, n, ps.mask(line)))
PY
}

# Turn private-tier records into violations and notes. A hit names the
# pattern's label and the masked line, never the name.
report_private() {
    while IFS="$(printf '\t')" read -r _rp_kind _rp_tier _rp_label _rp_detail; do
        [ -n "${_rp_kind:-}" ] || continue
        case "$_rp_kind" in
            hit)
                case "$_rp_tier" in
                    host) fail "HOST IDENTITY $_rp_label in $_rp_detail (a machine name from the private list; machine names never enter the repo)" ;;
                    *)    fail "private-list token $_rp_label in $_rp_detail" ;;
                esac
                ;;
            scanfail) fail "private-list scan FAILED for tier $_rp_tier: $_rp_detail (so this tier was NOT scanned)" ;;
            nolist)   printf 'NOTE: %s\n' "private-list tier ($_rp_tier) SKIPPED: $_rp_detail. Machine names, logins and retired branch prefixes were NOT checked by this run; the leak guard's private tier covers them where it runs." ;;
            note)     printf 'NOTE: %s\n' "$_rp_detail" ;;
        esac
    done
}

# RULE 6's tier: host identity only, for the public operator files. Kept
# separate from the tiers above (see the header) because those ban tracker
# ids, which those files legitimately carry. These are the address SHAPES;
# the machine names themselves (full and short forms) come from the private
# list, scanned over the same files by `private_scan` above.
host_identity_patterns=(
    '100\.(6[4-9]|[7-9][0-9]|1[01][0-9]|12[0-7])\.[0-9]{1,3}\.[0-9]{1,3}'
    '[t]s\.net'
)

# The files rule 6 covers. A GLOB, not a hand list, so a new runner script is
# covered the day it is added — the property the repo's coverage gates keep
# learning the hard way.
host_identity_files() {
    for f in tools/scripts/*.sh .github/workflows/*.yml; do
        [ "$f" = tools/scripts/check_agents_md.sh ] && continue   # its self-test fixture carries a literal sample of a shape it bans, so it cannot scan itself
        [ -f "$f" ] && printf '%s\n' "$f"
    done
}

# ONE scanner, two callers (the real sweep below and `--self-test`). Written as
# a function for exactly that reason: a self-test driving its own copy of the
# grep invocations would pass while the real scan was broken, which is the
# two-copies class this repo keeps paying for.
#
# Prints one TAB-separated record per finding:
#   hit<TAB><tier><TAB><pattern><TAB><grep -in output>
#   scanfail<TAB><tier><TAB><pattern><TAB><grep status>
#
# `grep` exits 0 on a match, 1 on NO match, and >=2 on an ERROR — a malformed
# pattern, an unreadable file. Collapsing 1 and >=2 into one `|| continue` made
# a broken pattern indistinguishable from a clean file: the scan reported the
# token absent and the gate went green on content it had never actually
# searched. Fail CLOSED, naming the pattern.
scan_file() {
    _sf_file=$1
    shift
    # "$1" is the tier label; the rest are the patterns, and the tier decides
    # whether the match is whole-word.
    _sf_tier=$1
    shift
    for _sf_pat in "$@"; do
        case "$_sf_tier" in
            token) _sf_hits=$(grep -inwE -e "$_sf_pat" "$_sf_file") ;;
            *)     _sf_hits=$(grep -inE  -e "$_sf_pat" "$_sf_file") ;;
        esac || {
            _sf_status=$?
            if [ "$_sf_status" -ge 2 ]; then
                printf 'scanfail\t%s\t%s\t%s\n' "$_sf_tier" "$_sf_pat" "$_sf_status"
            fi
            continue
        }
        while IFS= read -r _sf_hit; do
            [ -n "$_sf_hit" ] || continue
            printf 'hit\t%s\t%s\t%s\n' "$_sf_tier" "$_sf_pat" "$_sf_hit"
        done <<EOF2
$_sf_hits
EOF2
    done
}

# Turn one file's records into violations. Separate from the scanner so the
# self-test can count records without the side effects.
report_scan() {
    _rs_file=$1
    while IFS="$(printf '\t')" read -r _rs_kind _rs_tier _rs_pat _rs_detail; do
        [ -n "${_rs_kind:-}" ] || continue
        case "$_rs_kind" in
            hit)
                case "$_rs_tier" in
                    host) fail "HOST IDENTITY /$_rs_pat/ in $_rs_file:$_rs_detail — machine names and overlay-network addresses never enter the repo (the issue ids in these files deliberately do)" ;;
                    *)    fail "banned token /$_rs_pat/ in $_rs_file:$_rs_detail" ;;
                esac
                ;;
            scanfail)
                case "$_rs_tier" in
                    token) fail "banned-token scan FAILED on $_rs_file: grep exited $_rs_detail for pattern /$_rs_pat/ — the pattern is malformed or the file is unreadable, so this file was NOT scanned" ;;
                    host)  fail "host-identity scan FAILED on $_rs_file: grep exited $_rs_detail for pattern /$_rs_pat/ — the pattern is malformed or the file is unreadable, so this file was NOT scanned" ;;
                    *)     fail "banned-pattern scan FAILED on $_rs_file: grep exited $_rs_detail for pattern /$_rs_pat/ — the pattern is malformed or the file is unreadable, so this file was NOT scanned" ;;
                esac
                ;;
        esac
    done
}

# ---------------------------------------------------------------------------
# --self-test: drive EVERY pattern in every tier against a fixture that
# contains one of each, and a clean fixture that must produce nothing.
#
# WHY IT EXISTS. A banned-token gate whose regex has stopped matching looks
# exactly like a clean tree — the same shape `check_hot_path_allocs.sh
# --self-test` and `check_pr_title.sh --self-test` exist for, and the reason
# both run FIRST in `lint`. The real tree is clean by construction, so CI
# otherwise never drives these patterns over content that VIOLATES them.
#
# It scans through the SAME `scan_file` the real sweep uses, so a self-test that
# passes while the sweep is broken is not possible.
# ---------------------------------------------------------------------------
self_test() {
    st_fail=0
    # The tier SIZES are a precondition. Nothing indexes these arrays any more,
    # so this is no longer about `set -u` aborting the fixture — it is the arm
    # that catches a DELETED pattern, which the per-pattern arms below cannot
    # see (they only iterate the patterns that still exist).
    # Reported, NOT returned on: the substantive arms below give a better
    # diagnosis ("this pattern matched nothing"), and returning here would hide
    # them behind a count. The indexed reads use `-` defaults so `set -u` cannot
    # abort the fixture block either way.
    if [ "${#host_identity_patterns[@]}" -lt 2 ]; then
        printf 'self-test FAIL: host_identity_patterns has %s entries, expected at least 2 (CGNAT range, VPN domain); a pattern was deleted\n' \
            "${#host_identity_patterns[@]}" >&2
        st_fail=1
    fi
    st_dir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-agents-selftest.XXXXXX") || {
        printf 'self-test FAIL: could not create a fixture directory\n' >&2
        return 1
    }

    # THE DIRTY FIXTURE, and how it is built is the point.
    #
    # A fixture for this gate has to contain the shapes the gate bans, which for
    # the machine-name and ssh-handle tiers means it would contain exactly the
    # vocabulary the repo is not allowed to carry — and the first version of this
    # file did: it named a REAL box and a REAL VPN address, inside the gate that
    # exists to keep both out. Caught while reading the diff.
    #
    # So every line whose shape is an identifier is DERIVED FROM THE PATTERN
    # ITSELF and suffixed with an obvious placeholder. Nothing identifying is
    # typed here, and the arms keep working if a prefix ever changes. The
    # hand-written lines are generic vocabulary or documentation/private
    # addresses that name no host, and they carry the shapes whose patterns use
    # counted quantifiers that no un-escaping can turn back into a sample.
    #
    # ORDER-INDEPENDENT: reading patterns BY POSITION to build a fixture would let
    # inserting a pattern into the middle of an array silently point the fixture
    # at the wrong one, failing arms for a reason that has nothing to do with
    # what they test. Nothing here indexes an array.
    #
    # Turn a pattern into a string it matches: drop the left-boundary group, drop
    # a right-boundary group, unwrap the single-character classes that stop the
    # pattern list matching itself, fill a digit class, and unescape `\.`. A
    # pattern it cannot fully de-regex comes back still carrying a metacharacter,
    # and the caller skips it. A `+` after a filled digit becomes a second digit:
    # one-or-more is satisfied by two, and it is what keeps the tracker-id token
    # derivable rather than typed.
    st_sample_for() {
        printf '%s' "$1" \
            | sed -E 's/^\(\^\|\[\^\[:alnum:\]_\]\)//' \
            | sed -E 's/\(\[\^\[:alnum:\]_\]\|\$\)$//' \
            | sed -E 's/\[([A-Za-z])\]/\1/g' \
            | sed -E 's/\[0-9\]/7/g' \
            | sed -E 's/([0-9])\+/\1\1/g' \
            | sed -E 's/\\([.])/\1/g'
    }
    st_is_plain() {   # no regex metacharacter survived the un-escaping
        case "$1" in
            ''|*'['*|*']'*|*'('*|*')'*|*'{'*|*'}'*|*'|'*|*'^'*|*'$'*|*'*'*|*'+'*|*'?'*) return 1 ;;
        esac
        # The backslash is tested on its own, in double quotes: a lone `'\'`
        # inside a case pattern is ambiguous enough that shellcheck asks about it.
        case "$1" in *"\\"*) return 1 ;; esac
    }

    {
        printf 'the founder said so\n'
        printf 'see pt56 for the measurement\n'
        printf '192.168.0.1\n'
        printf '10.0.0.1\n'
        printf '100.64.0.1\n'  # leak-scan: allow cgnat-addr a literal sample for this gate's own self-test
        # One line per pattern in every tier, derived. Deduplicated, because a
        # pattern may legitimately appear in two tiers.
        st_seen=""
        for st_p in "${banned_tokens[@]}" "${host_identity_patterns[@]}"; do
            st_sample=$(st_sample_for "$st_p")
            st_is_plain "$st_sample" || continue
            case " $st_seen " in *" $st_sample "*) continue ;; esac
            st_seen="$st_seen $st_sample"
            # The placeholder goes BEFORE the sample, with a non-word separator,
            # and NOTHING follows it. Both halves are load-bearing and both were
            # learned by running it: a trailing placeholder puts a WORD character
            # after the match, which the whole-word tier (`grep -inwE`) rejects
            # and a right-boundary pattern cannot match either — so `ts.net` and
            # the machine short form both matched nothing. A leading one gives
            # every pattern the left boundary it needs and leaves the right one
            # to end-of-line.
            printf 'sample placeholder-%s\n' "$st_sample"
        done
    } > "$st_dir/dirty.md"
    cat > "$st_dir/clean.md" <<'CLEAN'
# A clean document
It names no people, no machines, no addresses and no tracker ids.
Branches follow type/kebab-slug. Boxes are referred to by role.
CLEAN

    st_expect_hit() { # <tier> <pattern> <label>
        if scan_file "$st_dir/dirty.md" "$1" "$2" | grep -q '^hit'; then return 0; fi
        printf 'self-test FAIL: tier %s pattern /%s/ matched NOTHING in the dirty fixture (%s)\n' "$1" "$2" "$3" >&2
        st_fail=1
    }

    for st_pat in "${banned_tokens[@]}"; do
        st_expect_hit token "$st_pat" 'banned_tokens'
    done
    for st_pat in "${host_identity_patterns[@]}"; do
        st_expect_hit host "$st_pat" 'host_identity_patterns'
    done

    # ANTI-TAUTOLOGY: every tier must be SILENT on clean prose. Without this a
    # pattern widened to match anything would pass every arm above.
    for st_tier_pats in "token:${banned_tokens[*]}" "host:${host_identity_patterns[*]}"; do
        st_tier=${st_tier_pats%%:*}
        # shellcheck disable=SC2086  # deliberate word splitting back into patterns
        st_out=$(scan_file "$st_dir/clean.md" "$st_tier" ${st_tier_pats#*:} | grep '^hit' || true)
        if [ -n "$st_out" ]; then
            printf 'self-test FAIL: tier %s matched CLEAN prose:\n%s\n' "$st_tier" "$st_out" >&2
            st_fail=1
        fi
    done

    # THE PRIVATE-LIST TIER, driven with an INVENTED stand-in list. Every call
    # here supplies LEAK_PATTERNS itself (the loader's FIRST source, so it wins
    # over any file), which is why these arms never touch a maintainer's real
    # list and pass on a machine that has none. The stand-ins are placeholders
    # by construction: nothing here names a machine, a login or a person.
    st_tab=$(printf '\t')
    st_private_list='word:placeholderbox7 @host
text:login-placeholder @login
(^|[^a-z0-9_])zz/ @hygiene
word:placeholderperson @person'
    {
        # The machine name as the PREFIX of a longer word: the shape the old
        # whole-word tier missed, and the one people actually write.
        printf 'ssh to placeholderbox7-slot and run it\n'
        printf 'log in as login-placeholder there\n'
        printf 'the branch was zz/wake-set\n'
        printf 'ask placeholderperson about it\n'
    } > "$st_dir/private-dirty.md"
    mkdir "$st_dir/nohome"

    # Every stand-in the tier asks for must hit, under its own tag.
    st_priv_out=$(LEAK_PATTERNS="$st_private_list" private_scan name host,login,hygiene "$st_dir/private-dirty.md")
    for st_tag in host login hygiene; do
        if ! printf '%s\n' "$st_priv_out" | grep -qE "^hit${st_tab}name${st_tab}private#[0-9]+@${st_tag}${st_tab}"; then
            printf 'self-test FAIL: private-list tier: the stand-in @%s entry matched NOTHING in the dirty fixture\n' "$st_tag" >&2
            st_fail=1
        fi
    done
    # Tag filtering: the entry the tier did NOT ask for stays out of it.
    if printf '%s\n' "$st_priv_out" | grep -q '@person'; then
        printf 'self-test FAIL: private-list tier: an entry outside the requested tags was reported\n' >&2
        st_fail=1
    fi
    # The report never carries the matched text, only the label and a masked line.
    for st_lit in placeholderbox7 login-placeholder zz/wake; do
        case "$st_priv_out" in
            *"$st_lit"*)
                printf 'self-test FAIL: private-list tier: a hit record printed the matched text (%s) instead of masking it\n' "$st_lit" >&2
                st_fail=1 ;;
        esac
    done
    # Rule 6 asks for host entries only.
    st_priv_host=$(LEAK_PATTERNS="$st_private_list" private_scan host host "$st_dir/private-dirty.md")
    if ! printf '%s\n' "$st_priv_host" | grep -qE "^hit${st_tab}host${st_tab}private#[0-9]+@host${st_tab}"; then
        printf 'self-test FAIL: private-list tier: the host tier did not report the stand-in machine name\n' >&2
        st_fail=1
    fi
    if printf '%s\n' "$st_priv_host" | grep -qE '@(login|hygiene)'; then
        printf 'self-test FAIL: private-list tier: the host tier reported a login or a branch prefix\n' >&2
        st_fail=1
    fi
    # ANTI-TAUTOLOGY: the stand-ins are silent on clean prose.
    st_priv_clean=$(LEAK_PATTERNS="$st_private_list" private_scan name host,login,hygiene "$st_dir/clean.md" | grep '^hit' || true)
    if [ -n "$st_priv_clean" ]; then
        printf 'self-test FAIL: private-list tier matched CLEAN prose:\n%s\n' "$st_priv_clean" >&2
        st_fail=1
    fi
    # NO LIST is a loud NOTE, never a silent pass and never a hit: every source
    # is emptied (env unset, no file named, a HOME with no default file).
    st_priv_none=$(LEAK_PATTERNS='' HOME="$st_dir/nohome" private_scan name host,login,hygiene "$st_dir/private-dirty.md")
    if ! printf '%s\n' "$st_priv_none" | grep -q '^nolist'; then
        printf 'self-test FAIL: private-list tier: with no list loaded there was no nolist record (a silent pass)\n' >&2
        st_fail=1
    fi
    if printf '%s\n' "$st_priv_none" | grep -q '^hit'; then
        printf 'self-test FAIL: private-list tier: with no list loaded something still hit\n' >&2
        st_fail=1
    fi
    if ! printf '%s\n' "$st_priv_none" | report_private | grep -q '^NOTE: private-list tier (name) SKIPPED'; then
        printf 'self-test FAIL: private-list tier: the nolist record did not render as a NOTE\n' >&2
        st_fail=1
    fi
    # FAIL CLOSED: an unreadable list named by the environment, and a pattern
    # that does not compile, are scan failures (a VIOLATION), not clean files.
    st_priv_bad=$(LEAK_PATTERNS='' LEAK_PATTERNS_FILE="$st_dir/absent" private_scan name host,login,hygiene "$st_dir/private-dirty.md")
    if ! printf '%s\n' "$st_priv_bad" | grep -q '^scanfail'; then
        printf 'self-test FAIL: private-list tier: an unreadable list did not report a scan failure\n' >&2
        st_fail=1
    fi
    st_priv_bad=$(LEAK_PATTERNS='(a|' private_scan name host,login,hygiene "$st_dir/private-dirty.md")
    if ! printf '%s\n' "$st_priv_bad" | grep -q '^scanfail'; then
        printf 'self-test FAIL: private-list tier: a pattern that does not compile did not report a scan failure\n' >&2
        st_fail=1
    fi

    # A malformed pattern must FAIL CLOSED — report a scan failure rather than
    # "no match". `[` opens an unterminated bracket expression in ERE.
    # stderr is dropped: grep's own "brackets not balanced" complaint is the
    # EXPECTED outcome here, and leaking it into a CI log makes a passing
    # self-test look like a failing one.
    if ! scan_file "$st_dir/dirty.md" token '[' 2>/dev/null | grep -q '^scanfail'; then
        printf 'self-test FAIL: a malformed pattern did not report a scan failure — a broken pattern would read as a clean file\n' >&2
        st_fail=1
    fi

    # Rule 6 must actually cover something. A glob that matches nothing is a
    # rule that cannot fail.
    st_n=$(host_identity_files | grep -c .)
    if [ "$st_n" -lt 4 ]; then
        printf 'self-test FAIL: host_identity_files() matched only %s file(s) — the glob is not reaching tools/scripts or .github/workflows\n' "$st_n" >&2
        st_fail=1
    fi

    rm -rf "$st_dir"
    if [ "$st_fail" -ne 0 ]; then
        printf 'check_agents_md --self-test: FAIL\n'
        return 1
    fi
    printf 'check_agents_md --self-test: OK (%s token + %s host-identity pattern(s) all match a dirty fixture and none match a clean one; the private-list tier hits a stand-in list, masks, filters by tag, notes an absent list and fails closed; rule 6 covers %s file(s))\n' \
        "${#banned_tokens[@]}" "${#host_identity_patterns[@]}" "$st_n"
    return 0
}

if [ "$SELF_TEST" = yes ]; then
    self_test
    exit $?
fi

# ---------------------------------------------------------------------------
# Collect every AGENTS.md in the tree (newline list; repo paths contain no
# whitespace, and the find prunes anything that could).
# ---------------------------------------------------------------------------
agents_files=$(find . \
    \( -name .git -o -name target -o -name notes -o -name node_modules \) -prune \
    -o -type f -name AGENTS.md -print | sort)

# A symlinked context file bypasses -type f entirely: fail each one outright.
symlinked=$(find . \
    \( -name .git -o -name target -o -name notes -o -name node_modules \) -prune \
    -o -type l \( -name AGENTS.md -o -name CLAUDE.md \) -print | sort)
while IFS= read -r f; do
    [ -n "$f" ] || continue
    fail "$f is a symlink — context files must be regular files (symlinks dodge every check here)"
done <<EOF
$symlinked
EOF

# ---------------------------------------------------------------------------
# 1 + 2 + 3: budgets, chain, shims
# ---------------------------------------------------------------------------
root_bytes=0
largest_crate_bytes=0
largest_crate_file=""

while IFS= read -r f; do
    [ -n "$f" ] || continue

    if [ ! -r "$f" ]; then
        fail "$f is unreadable — cannot verify (a gate must not fail open)"
        continue
    fi
    lines=$(wc -l < "$f"); lines=$((lines))
    bytes=$(wc -c < "$f"); bytes=$((bytes))

    if [ "$f" = "./AGENTS.md" ]; then
        max_lines=$ROOT_MAX_LINES
        max_bytes=$ROOT_MAX_BYTES
        root_bytes=$bytes
    else
        max_lines=$CRATE_MAX_LINES
        max_bytes=$CRATE_MAX_BYTES
        if [ "$bytes" -gt "$largest_crate_bytes" ]; then
            largest_crate_bytes=$bytes
            largest_crate_file=$f
        fi
    fi

    if [ "$lines" -gt "$max_lines" ]; then
        fail "$f is $lines lines (budget: $max_lines)"
    fi
    if [ "$bytes" -gt "$max_bytes" ]; then
        fail "$f is $bytes bytes (budget: $max_bytes)"
    fi

    # Shim check: the directory must carry a CLAUDE.md pointing at AGENTS.md.
    dir=$(dirname "$f")
    shim="$dir/CLAUDE.md"
    if [ ! -f "$shim" ]; then
        fail "$dir/ has AGENTS.md but no CLAUDE.md shim (create $shim containing exactly '@AGENTS.md')"
    elif [ "$f" = "./AGENTS.md" ]; then
        first_line=$(head -n 1 "$shim")
        if [ "$first_line" != "@AGENTS.md" ]; then
            fail "$shim line 1 must be exactly '@AGENTS.md' (got: '$first_line')"
        fi
    else
        # Command substitution strips trailing newlines, so a single
        # '@AGENTS.md' line (with or without trailing newline) passes and
        # anything else fails.
        content=$(cat "$shim")
        if [ "$content" != "@AGENTS.md" ]; then
            fail "$shim content must be exactly '@AGENTS.md' (crate shims carry nothing else)"
        fi
    fi
done <<EOF
$agents_files
EOF

# Belt-and-suspenders: with today's per-file budgets (10000 + 4096) this can
# only fire if those are raised — it exists so a future budget bump cannot
# silently cross the 32 KiB combined-context ceiling. Model = root + largest
# non-root file (the tree keeps AGENTS.md at root + first level only; add
# ancestor-path summing if files ever nest deeper).
if [ "$root_bytes" -gt 0 ] && [ "$largest_crate_bytes" -gt 0 ]; then
    chain=$((root_bytes + largest_crate_bytes))
    if [ "$chain" -gt "$CHAIN_MAX_BYTES" ]; then
        fail "chain budget: root ($root_bytes) + $largest_crate_file ($largest_crate_bytes) = $chain bytes (budget: $CHAIN_MAX_BYTES)"
    fi
fi

# NAME-CLASS tier: one extended-regex per line, '#' comments allowed. The
# file is untracked by design (see header). Loaded verbatim into the same
# whole-word scan as banned_tokens.
tokens_file="${AGENTS_TOKENS_FILE:-.agents-tokens.local}"
name_tokens=()
if [ -f "$tokens_file" ]; then
    while IFS= read -r line; do
        case "$line" in ''|'#'*) continue ;; esac
        name_tokens+=("$line")
    done < "$tokens_file"
else
    printf 'INFO: %s\n' "name-class token list not found ($tokens_file) — generic patterns only (maintainers: link it via the private overlay setup)"
fi

scan_files=$agents_files
# The root CLAUDE.md addendum is public content too — scan it once it is the
# '@AGENTS.md' shim form (mid-migration, the legacy monolith is exempt: it is
# replaced wholesale, not scrubbed).
if [ -f ./CLAUDE.md ] && [ "$(head -n 1 ./CLAUDE.md)" = "@AGENTS.md" ]; then
    scan_files="$scan_files
./CLAUDE.md"
fi
for d in docs/internals/*.md; do
    [ -f "$d" ] || continue
    scan_files="$scan_files
$d"
done

while IFS= read -r f; do
    [ -n "$f" ] || continue
    if [ ! -r "$f" ]; then
        fail "$f is unreadable — cannot scan for banned tokens"
        continue
    fi
    report_scan "$f" < <(scan_file "$f" token "${banned_tokens[@]}" ${name_tokens[@]+"${name_tokens[@]}"})
done <<EOF
$scan_files
EOF

# PRIVATE-LIST tier over the same files, in one pass.
private_files=()
while IFS= read -r f; do
    [ -n "$f" ] || continue
    private_files+=("$f")
done <<EOF
$scan_files
EOF
report_private < <(private_scan name host,login,hygiene ${private_files[@]+"${private_files[@]}"})

# RULE 6: host identity in the public operator files.
while IFS= read -r f; do
    [ -n "$f" ] || continue
    if [ ! -r "$f" ]; then
        fail "$f is unreadable — cannot scan for host identity"
        continue
    fi
    report_scan "$f" < <(scan_file "$f" host "${host_identity_patterns[@]}")
done <<EOF
$(host_identity_files)
EOF
host_files=()
while IFS= read -r f; do
    [ -n "$f" ] || continue
    host_files+=("$f")
done <<EOF
$(host_identity_files)
EOF
report_private < <(private_scan host host ${host_files[@]+"${host_files[@]}"})

# ---------------------------------------------------------------------------
# Verdict
# ---------------------------------------------------------------------------
if [ "$violations" -gt 0 ]; then
    printf '%s\n' "check_agents_md: FAIL ($violations violation(s))"
    exit 1
fi
printf '%s\n' "check_agents_md: OK"
exit 0
