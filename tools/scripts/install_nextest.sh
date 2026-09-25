#!/usr/bin/env bash
# install_nextest.sh — put a PINNED `cargo-nextest` on PATH.
#
# WHY A SCRIPT AND NOT AN ACTION. This repo does not take third-party
# GitHub Actions for tool installs (the SHA-pinned exceptions in ci.yml are
# deliberate and few), and `cargo install cargo-nextest --locked` builds the
# tool from source — minutes per job, on every job, for a binary upstream
# already publishes. So this fetches the OFFICIAL pre-built tarball from
# nextest's own domain, which is the install route nextest documents for CI.
#
# PINNED, never `latest`, and pinned BY CONTENT as well as by version. The
# whole point of the fence in `.config/nextest.toml` is that the runner's
# behaviour is pinned deliberately; a floating version would let an upstream
# release change how tests are scheduled without a commit in this repo. A
# version pin alone fixes the URL, not the bytes — so each platform's tarball
# carries a SHA-256 here, in the same spirit as the SHA-pinned actions in
# ci.yml. The pinned version and its three checksums are ONE fact, edited
# together — a `NEXTEST_VERSION` override that disagrees with the pin is
# refused UP FRONT rather than left to fail later as a puzzling hash mismatch.
# Moving the pin means editing NEXTEST_PINNED_VERSION and regenerating all
# three hashes, which is the reviewable act:
#
#   for p in linux linux-arm mac; do
#       curl -sSL --fail "https://get.nexte.st/<version>/$p" -o "$p.tar.gz"
#       printf '%s %s\n' "$p" "$(shasum -a 256 "$p.tar.gz" | cut -d' ' -f1)"
#   done
#
# Idempotent: an already-present binary of the right version is left alone, so
# a self-hosted runner does not re-download on every job.
#
# Exit 0 = a `cargo nextest` of NEXTEST_VERSION is on PATH. Nonzero = it is
# not, loudly — never a silent fallback to whatever happens to be installed,
# because that is how a pinned toolchain quietly stops being pinned.

set -euo pipefail

# The pinned version and ITS checksums are ONE fact, edited together.
NEXTEST_PINNED_VERSION=0.9.137
NEXTEST_SHA256_linux=38fd6275e111b200bbbed1bd2ae91cbb0d7edd28504879875cff2b3d96f3f311
NEXTEST_SHA256_linux_arm=8f878903d63a69dd0a8fba65d0dc871043423bd0591076f688023b862d58d1a6
NEXTEST_SHA256_mac=94e89c20b233c29c042683e885131d76abf95e795c355a4fe7d64d4b128b90be

NEXTEST_VERSION="${NEXTEST_VERSION:-$NEXTEST_PINNED_VERSION}"

die() {
    printf 'install_nextest: %s\n' "$1" >&2
    exit 1
}

# NEXTEST_VERSION selects the download; the checksums above do not follow it.
# Left alone, that combination reads as a usable knob and behaves as a trap: the
# override changes the URL, the download succeeds, and the run dies on a
# checksum mismatch that looks like a corrupted or tampered artifact rather than
# like "you changed the version and not the hashes".
#
# So the mismatch is refused HERE, before anything is fetched, naming both
# versions. The override stays — it is genuinely useful for trying a release
# before pinning it — but it now tells you the truth immediately instead of
# after a download and a confusing hash error.
if [ "$NEXTEST_VERSION" != "$NEXTEST_PINNED_VERSION" ]; then
    die "NEXTEST_VERSION is '$NEXTEST_VERSION' but the checksums in this script are
pinned to '$NEXTEST_PINNED_VERSION', so the download could not be verified.

A version and its checksums are ONE edit. To move the pin, change
NEXTEST_PINNED_VERSION and regenerate all three hashes (the header shows how),
then re-read the fence semantics in .config/nextest.toml — an upstream release
can change how tests are SCHEDULED, which is the whole reason this is pinned."
fi

# The version as a WHOLE FIELD, never a prefix.
#
# `--version` prints `cargo-nextest 0.9.137 (…)`, so a substring test accepts
# 0.9.137 as a match for a pin of "0.9.13" — and because this check
# short-circuits before the download, pinning 0.9.13 on a machine that already
# had 0.9.137 would report "already on PATH" and never install the pinned
# build, silently defeating the pin this script exists to enforce.
installed_version() {
    command -v cargo-nextest >/dev/null 2>&1 || return 1
    cargo-nextest --version 2>/dev/null | head -1 | awk '{print $2}'
}

have_pinned() {
    [ "$(installed_version || true)" = "$NEXTEST_VERSION" ]
}

if have_pinned; then
    printf 'install_nextest: cargo-nextest %s already on PATH (%s)\n' \
        "$NEXTEST_VERSION" "$(command -v cargo-nextest)"
    exit 0
fi

# ARCH matters on Linux: `…/linux` is the x86_64 asset, so an aarch64 runner
# (`ubuntu-*-arm` is now offered, and this repo already cross-checks that
# target) would download a binary it cannot exec. macOS ships one universal
# binary, so it needs no arch arm.
case "$(uname -s)" in
    Linux)
        case "$(uname -m)" in
            x86_64|amd64)   platform=linux;     expected=$NEXTEST_SHA256_linux ;;
            aarch64|arm64)  platform=linux-arm; expected=$NEXTEST_SHA256_linux_arm ;;
            *) die "unsupported Linux arch '$(uname -m)' — add it here or install cargo-nextest by hand" ;;
        esac
        ;;
    Darwin) platform=mac; expected=$NEXTEST_SHA256_mac ;;
    *)      die "unsupported OS '$(uname -s)' — add it here or install cargo-nextest by hand" ;;
esac

# `~/.cargo/bin` is on PATH for every job in this workflow (the toolchain
# action puts it there), so installing beside the other cargo binaries needs no
# PATH surgery and no `GITHUB_PATH` write.
dest="${CARGO_HOME:-$HOME/.cargo}/bin"
mkdir -p "$dest"

url="https://get.nexte.st/${NEXTEST_VERSION}/${platform}"
printf 'install_nextest: fetching %s\n' "$url"

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

# `--fail` so an HTTP error is an ERROR and not a tarball made of an error
# page; `--location` because the pinned URL redirects to the release asset.
#
# `--retry-all-errors` and not `--retry` alone, and the difference is the whole
# reason this line changed. curl's `--retry` covers TRANSIENT errors as curl
# defines them: a timeout, and the 408 / 429 / 5xx replies. It does not cover a
# connection that is reset mid-transfer, which is what a hosted runner produced
# here:
#
#   curl: (35) Recv failure: Connection reset by peer
#
# so the retry budget went unspent and the job died on the first reset.
# `--retry-all-errors` retries the transport failures too. The cost is that a
# genuine 404 (a version that does not exist) is now attempted 4 times before
# the `die` below names it, which is 6 seconds on a path that is already
# failing. The checksum gate is untouched: a retry re-downloads bytes that are
# still verified before anything is extracted.
curl --proto '=https' --tlsv1.2 --fail --silent --show-error --location \
    --retry 3 --retry-delay 2 --retry-all-errors --max-time 180 \
    "$url" -o "$tmp/nextest.tar.gz" \
    || die "download failed for $url (version '$NEXTEST_VERSION' may not exist)"

# VERIFY BEFORE EXTRACTING. Everything below this point runs code that came off
# the network, so the checksum is the last moment it is still just bytes.
if command -v shasum >/dev/null 2>&1; then
    actual=$(shasum -a 256 "$tmp/nextest.tar.gz" | cut -d' ' -f1)
elif command -v sha256sum >/dev/null 2>&1; then
    actual=$(sha256sum "$tmp/nextest.tar.gz" | cut -d' ' -f1)
else
    die "neither shasum nor sha256sum is available — refusing to run an unverified download"
fi
if [ "$actual" != "$expected" ]; then
    die "checksum mismatch for $url
  expected $expected
  actual   $actual
If NEXTEST_VERSION was just bumped, the pinned checksums in this script must be
bumped with it (the header shows how to regenerate them). Otherwise the
download is not the release this repo pinned — do not run it."
fi

tar -xzf "$tmp/nextest.tar.gz" -C "$tmp" || die "the download was not a gzip tarball"
[ -f "$tmp/cargo-nextest" ] || die "the tarball did not contain a cargo-nextest binary"

chmod +x "$tmp/cargo-nextest"
mv "$tmp/cargo-nextest" "$dest/cargo-nextest"

# Verify the BINARY WE JUST WROTE, by path. Re-running `have_pinned` here would
# ask PATH instead, which conflates two different failures: a download that
# produced the wrong version, and a correct install into a directory PATH does
# not carry. The second is a real possibility whenever `CARGO_HOME` is set
# somewhere unusual, and reporting it as "wrong version" sends the reader
# hunting for a bad tarball.
# `--version` prints a multi-line report; the FIRST line carries the version.
# Compared as a WHOLE FIELD for the same reason `have_pinned` is — a trailing
# `*` glob would accept 0.9.137 for a pin of 0.9.13.
reported=$("$dest/cargo-nextest" --version 2>/dev/null | head -1 || true)
got=$(printf '%s' "$reported" | awk '{print $2}')
if [ -z "$reported" ]; then
    die "the installed binary at $dest/cargo-nextest does not run"
fi
if [ "$got" != "$NEXTEST_VERSION" ]; then
    die "the installed binary reports version '$got' ('$reported'), not $NEXTEST_VERSION"
fi
printf 'install_nextest: installed %s -> %s\n' "$reported" "$dest/cargo-nextest"

# PATH is a SEPARATE question, and a separate diagnostic. Everything downstream
# invokes `cargo nextest`, which resolves `cargo-nextest` through PATH.
if ! have_pinned; then
    die "installed $dest/cargo-nextest, but '$dest' is not on PATH (or an older
cargo-nextest shadows it) — \`cargo nextest\` would not find it. Add '$dest' to
PATH before this step."
fi
