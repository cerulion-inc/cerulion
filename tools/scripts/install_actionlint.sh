#!/usr/bin/env bash
# install_actionlint.sh — put a PINNED `actionlint` on PATH.
#
# WHY A SCRIPT AND NOT AN ACTION, and why a download at all. This repo does not
# take third-party GitHub Actions for tool installs, and `actionlint` is on
# NEITHER runner this job can land on: it is not an Ubuntu package, so
# `ubuntu-latest` does not carry it, and the self-hosted image deliberately does
# not install it. Building it from source needs a Go toolchain the container
# image does not ship. So this fetches the official pre-built tarball from the
# project's own release, which is the install route actionlint documents for CI.
#
# PINNED, never `latest`, and pinned BY CONTENT as well as by version — the same
# argument as `install_nextest.sh`, which this deliberately mirrors rather than
# inventing a second shape. A linter that floats can start failing the build on
# an upstream release nobody in this repo chose; a version pin alone fixes the
# URL, not the bytes, so each platform's tarball carries a SHA-256 here. The
# pinned version and its checksums are ONE fact, edited together, and an
# ACTIONLINT_VERSION override that disagrees with the pin is refused UP FRONT
# rather than left to fail later as a puzzling hash mismatch.
#
# Moving the pin means editing ACTIONLINT_PINNED_VERSION and regenerating the
# hashes, which is the reviewable act:
#
#   v=<version>
#   curl -sSL --fail "https://github.com/rhysd/actionlint/releases/download/v$v/actionlint_${v}_checksums.txt"
#
# Idempotent: an already-present binary of the right version is left alone, so a
# self-hosted runner does not re-download on every job.
#
# Exit 0 = an `actionlint` of ACTIONLINT_VERSION is on PATH at the printed path.
# Nonzero = it is not, loudly — never a silent fallback to whatever happens to be
# installed, because that is how a pinned linter quietly stops being pinned.

set -euo pipefail

# The pinned version and ITS checksums are ONE fact, edited together.
ACTIONLINT_PINNED_VERSION=1.7.12
ACTIONLINT_SHA256_linux_amd64=8aca8db96f1b94770f1b0d72b6dddcb1ebb8123cb3712530b08cc387b349a3d8
ACTIONLINT_SHA256_linux_arm64=325e971b6ba9bfa504672e29be93c24981eeb1c07576d730e9f7c8805afff0c6
ACTIONLINT_SHA256_darwin_amd64=5b44c3bc2255115c9b69e30efc0fecdf498fdb63c5d58e17084fd5f16324c644
ACTIONLINT_SHA256_darwin_arm64=aba9ced2dee8d27fecca3dc7feb1a7f9a52caefa1eb46f3271ea66b6e0e6953f

ACTIONLINT_VERSION="${ACTIONLINT_VERSION:-$ACTIONLINT_PINNED_VERSION}"
INSTALL_DIR="${ACTIONLINT_INSTALL_DIR:-$HOME/.local/bin}"

die() {
  printf '::error::install_actionlint: %s\n' "$1" >&2
  exit 1
}

# An override that disagrees with the pin has no checksum here, so it could only
# be installed UNVERIFIED. Refuse up front and say why, rather than failing later
# with a hash mismatch that reads like corruption.
if [ "$ACTIONLINT_VERSION" != "$ACTIONLINT_PINNED_VERSION" ]; then
  die "ACTIONLINT_VERSION='$ACTIONLINT_VERSION' but this script pins $ACTIONLINT_PINNED_VERSION and carries checksums only for the pin. Moving the version means editing ACTIONLINT_PINNED_VERSION and its hashes together, in one reviewable commit."
fi

# Direct references, not an `eval` over a constructed name: this is the shape
# `install_nextest.sh` uses, and it keeps the checksums visibly USED — an
# indirect read makes every one of them look dead to shellcheck, which then has
# to be silenced, which is exactly the kind of suppression that later hides a
# genuinely orphaned pin.
case "$(uname -s)" in
  Linux)
    case "$(uname -m)" in
      x86_64|amd64)  platform=linux_amd64;  want_sha=$ACTIONLINT_SHA256_linux_amd64 ;;
      aarch64|arm64) platform=linux_arm64;  want_sha=$ACTIONLINT_SHA256_linux_arm64 ;;
      *)             die "unsupported Linux architecture '$(uname -m)'" ;;
    esac
    ;;
  Darwin)
    case "$(uname -m)" in
      x86_64|amd64)  platform=darwin_amd64; want_sha=$ACTIONLINT_SHA256_darwin_amd64 ;;
      aarch64|arm64) platform=darwin_arm64; want_sha=$ACTIONLINT_SHA256_darwin_arm64 ;;
      *)             die "unsupported macOS architecture '$(uname -m)'" ;;
    esac
    ;;
  *)
    die "unsupported OS '$(uname -s)' — actionlint publishes linux and darwin builds"
    ;;
esac

# Already installed at the pinned version? Leave it alone. `actionlint -version`
# prints the bare version on its first line.
if command -v actionlint >/dev/null 2>&1; then
  have="$(actionlint -version 2>/dev/null | head -n 1 | tr -d '[:space:]')" || have=""
  if [ "$have" = "$ACTIONLINT_VERSION" ]; then
    printf 'actionlint %s already on PATH (%s)\n' "$have" "$(command -v actionlint)"
    exit 0
  fi
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

tarball="actionlint_${ACTIONLINT_VERSION}_${platform}.tar.gz"
url="https://github.com/rhysd/actionlint/releases/download/v${ACTIONLINT_VERSION}/${tarball}"

curl -sSL --fail --retry 3 --retry-delay 2 "$url" -o "$tmp/$tarball" \
  || die "download failed: $url"

# Verify BEFORE unpacking: an unverified archive is not extracted at all.
if command -v sha256sum >/dev/null 2>&1; then
  got="$(sha256sum "$tmp/$tarball" | cut -d' ' -f1)"
else
  got="$(shasum -a 256 "$tmp/$tarball" | cut -d' ' -f1)"
fi
[ "$got" = "$want_sha" ] \
  || die "checksum mismatch for $tarball — expected $want_sha, got $got. The pin and the bytes disagree; do not proceed."

tar -xzf "$tmp/$tarball" -C "$tmp" actionlint \
  || die "tarball did not contain an 'actionlint' binary"

mkdir -p "$INSTALL_DIR"
install -m 0755 "$tmp/actionlint" "$INSTALL_DIR/actionlint" \
  || die "could not install into $INSTALL_DIR"

# Confirm the thing we just installed actually runs and reports the pinned
# version — a binary that unpacked but cannot execute is still a failure.
installed="$("$INSTALL_DIR/actionlint" -version 2>/dev/null | head -n 1 | tr -d '[:space:]')" || installed=""
[ "$installed" = "$ACTIONLINT_VERSION" ] \
  || die "installed binary reports version '${installed:-<none>}', expected $ACTIONLINT_VERSION"

printf 'actionlint %s installed at %s/actionlint\n' "$ACTIONLINT_VERSION" "$INSTALL_DIR"
