#!/bin/sh
# Drive tools/scripts/render_homebrew_formula.sh against a hand-written
# checksum file, so the formula that reaches users can be checked without
# cutting a release.
#
# What it asserts: the rendered file is valid Ruby, it names all four platform
# archives with the checksums the file gave it, it carries the caveat and the
# setup wrapper, and a checksum file missing one archive is refused instead of
# rendering an empty sha256. The Ruby syntax check is the headline assertion,
# so it is refused rather than skipped under CI; on a machine with no ruby at
# hand it is skipped loudly and every other assertion still runs.

set -eu

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
render="$script_dir/render_homebrew_formula.sh"

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT INT TERM

version=9.8.7
tag=v9.8.7
repository=cerulion-inc/cerulion

linux_x86_sha=1111111111111111111111111111111111111111111111111111111111111111
linux_arm_sha=2222222222222222222222222222222222222222222222222222222222222222
mac_x86_sha=3333333333333333333333333333333333333333333333333333333333333333
mac_arm_sha=4444444444444444444444444444444444444444444444444444444444444444

sums="$work/SHA256SUMS"
cat > "$sums" <<SUMS
${linux_x86_sha}  cerulion-${version}-x86_64-unknown-linux-gnu.tar.gz
${linux_arm_sha}  cerulion-${version}-aarch64-unknown-linux-gnu.tar.gz
${mac_x86_sha}  cerulion-${version}-x86_64-apple-darwin.tar.gz
${mac_arm_sha}  cerulion-${version}-aarch64-apple-darwin.tar.gz
SUMS

formula="$work/cerulion.rb"
render_noise="$work/render-stderr"
"$render" "$version" "$tag" "$repository" "$sums" > "$formula" 2>"$render_noise"

fail() {
    printf 'error: %s\n' "$1" >&2
    exit 1
}

# The formula is written by a shell heredoc that expands what it is given, so
# a backtick or a dollar sign left unescaped in the template runs as a command
# and drops its output into the formula. A quiet render is the cheap half of
# noticing that; the line assertions below are the other half.
if [ -s "$render_noise" ]; then
    cat "$render_noise" >&2
    fail 'rendering the formula wrote to standard error'
fi

# Whole lines, not substrings: a renamed `def caveats` still contains
# `def caveats` and would satisfy a substring match.
want_line() {
    grep -Fxq "$1" "$formula" || fail "rendered formula is missing the line: $1"
}

want_text() {
    grep -Fq "$1" "$formula" || fail "rendered formula is missing: $1"
}

if command -v ruby >/dev/null 2>&1; then
    ruby -c "$formula" >/dev/null || fail 'rendered formula is not valid Ruby'
elif [ -n "${CI:-}" ] || [ -n "${GITHUB_ACTIONS:-}" ]; then
    # A skipped headline assertion that still reports "passed" is the shape
    # nobody reads. No other check in this repository parses Ruby, so an
    # absent interpreter here means the formula reaches users unparsed.
    fail 'no ruby is installed, so the formula syntax check cannot run'
else
    printf '%s\n' 'note: no ruby on this machine; the syntax check was not run'
fi

base="https://github.com/${repository}/releases/download/${tag}"
for target in x86_64-unknown-linux-gnu aarch64-unknown-linux-gnu \
    x86_64-apple-darwin aarch64-apple-darwin; do
    want_line "      url \"${base}/cerulion-${version}-${target}.tar.gz\""
done

for value in "$linux_x86_sha" "$linux_arm_sha" "$mac_x86_sha" "$mac_arm_sha"; do
    want_line "      sha256 \"${value}\""
done

# One url and one sha256 per platform, and nothing else: a render that emitted
# a stray pair would still satisfy the four assertions above.
url_count=$(grep -c '^ *url "' "$formula")
sha_count=$(grep -c '^ *sha256 "' "$formula")
[ "$url_count" = 4 ] || fail "rendered formula has $url_count urls, expected 4"
[ "$sha_count" = 4 ] || fail "rendered formula has $sha_count checksums, expected 4"

want_line '  version "9.8.7"'
want_line '  def caveats'
want_line '    (share/"cerulion/install.json").write "{\"method\":\"brew\",\"version\":\"#{version}\"}\n"'
want_line '        cerulion-install-rust'
want_text 'Homebrew itself never writes into your'

# The caveat reaches `brew info` before anyone installs, so it carries both
# steps and neither of them sits behind a condition.
# shellcheck disable=SC2016  # the assertion is the literal text of the formula
want_line '        export PATH="${CARGO_HOME:-$HOME/.cargo}/bin:$PATH"'
# shellcheck disable=SC2016  # the assertion is the literal text of the formula
want_line '  # Printed unconditionally, so `brew info cerulion` carries the one extra'
if grep -Fq 'return unless' "$formula"; then
    fail 'the caveat is printed conditionally'
fi

# Reached through the wrapper, the helper's own failure text speaks for the
# archive installer and claims a rollback that did not happen here.
# shellcheck disable=SC2016  # the assertion is the literal text of the formula
want_line '        exit "$status"'
want_text 'The Cerulion programs are installed; only the compiler setup failed.'
want_line '      (bin/"cerulion-install-rust").write <<~WRAPPER'
want_line '      chmod 0755, bin/"cerulion-install-rust"'
want_line '      libexec.install helper'
want_line '      libexec.install metadata'
want_line '    generate_completions_from_executable(bin/"cerulion", "completions")'
want_line '  test do'
want_line '# The release workflow writes this file.'

# A checksum file that does not cover every archive must be refused. Without
# this the render would put an empty sha256 in the formula, and Homebrew would
# install a download it cannot verify.
partial="$work/SHA256SUMS.partial"
grep -v 'aarch64-apple-darwin' "$sums" > "$partial"
set +e
output=$("$render" "$version" "$tag" "$repository" "$partial" 2>&1)
status=$?
set -e
if [ "$status" -eq 0 ]; then
    printf '%s\n' 'error: a checksum file missing an archive was accepted' >&2
    printf '%s\n' "$output" >&2
    exit 1
fi
printf '%s\n' "$output" | grep -Fq 'has no checksum in' || {
    printf '%s\n' 'error: the missing-checksum refusal had the wrong diagnostic' >&2
    printf '%s\n' "$output" >&2
    exit 1
}

# A wrong argument count and an unreadable checksum file are refused too.
set +e
"$render" "$version" "$tag" "$repository" >/dev/null 2>&1
status=$?
set -e
[ "$status" -eq 2 ] || fail "a three-argument call exited $status, expected 2"

set +e
"$render" "$version" "$tag" "$repository" "$work/absent" >/dev/null 2>&1
status=$?
set -e
[ "$status" -eq 2 ] || fail "an absent checksum file exited $status, expected 2"

printf '%s\n' 'render_homebrew_formula: passed'
