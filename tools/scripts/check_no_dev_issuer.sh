#!/usr/bin/env bash
# check_no_dev_issuer.sh: fail when a release binary builds cerulion_pairing
# with its `dev-issuer` feature, which trusts team credentials from a local
# development issuer.
#
# RELEASE_PACKAGES below is the package list the release binaries are built
# from; `--print-packages` prints it for the release build step. Every other
# argument (a `--target`, `--offline`) is passed to `cargo tree`, which walks
# normal and build dependencies with the features the resolver turns on.
set -euo pipefail
RELEASE_PACKAGES=(-p cerulion_cli -p cerulion_netd -p cerulion_connectd -p cerulion_remoted)
if [ "${1:-}" = --print-packages ]; then
    echo "${RELEASE_PACKAGES[*]}"
    exit 0
fi
packages=("${RELEASE_PACKAGES[@]}")
if [ "${1:-}" = --packages ]; then
    shift
    packages=()
    while [ $# -gt 0 ] && [ "$1" != -- ]; do
        packages+=("$1")
        shift
    done
    [ "${1:-}" = -- ] && shift
fi
out=$(cargo tree --locked -e features,normal,build -i cerulion_pairing "${packages[@]}" "$@" 2>&1) || {
    printf '%s\n' "$out" >&2
    echo "check_no_dev_issuer: cargo tree failed" >&2
    exit 2
}
if printf '%s\n' "$out" | grep -q 'feature "dev-issuer"'; then
    echo "check_no_dev_issuer: a release package enables cerulion_pairing/dev-issuer:" >&2
    printf '%s\n' "$out" | grep -A3 'feature "dev-issuer"' >&2
    exit 1
fi
echo "check_no_dev_issuer: OK (${packages[*]} $*)"
