#!/bin/sh

set -eu

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

[ "$#" -eq 2 ] || die "usage: $0 KEYRING KEYS_FILE"
keyring=$1
keys_file=$2

[ -f "$keyring" ] || die "keyring does not exist: $keyring"
[ -f "$keys_file" ] || die "signing key list does not exist: $keys_file"
[ -s "$keys_file" ] || die "signing key list is empty: $keys_file"
command -v gpg >/dev/null 2>&1 ||
    die "gpg is required to inspect the archive keyring"

gpg_output=
gpg_error=
cleanup() {
    status=$?
    [ -z "$gpg_output" ] || rm -f "$gpg_output"
    [ -z "$gpg_error" ] || rm -f "$gpg_error"
    exit "$status"
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

gpg_output=$(mktemp "${TMPDIR:-/tmp}/cerulion-apt-keyring-output.XXXXXX") ||
    die "could not create temporary archive keyring inspection output"
gpg_error=$(mktemp "${TMPDIR:-/tmp}/cerulion-apt-keyring-error.XXXXXX") ||
    die "could not create temporary archive keyring inspection error"
if ! gpg --batch --with-colons --show-keys "$keyring" >"$gpg_output" \
    2>"$gpg_error"; then
    gpg_diagnostic=$(cat "$gpg_error")
    die "could not inspect archive keyring: $keyring: $gpg_diagnostic"
fi
if ! keyring_fingerprints=$(awk -F: '$1 == "fpr" { print toupper($10) }' \
    "$gpg_output"); then
    die "could not parse archive keyring inspection: $keyring"
fi

while IFS= read -r key || [ -n "$key" ]; do
    [ -n "$key" ] || die "signing key list contains an empty fingerprint"
    normalized_key=$(printf '%s' "$key" | tr '[:lower:]' '[:upper:]')
    if ! printf '%s\n' "$keyring_fingerprints" | grep -Fqx "$normalized_key"; then
        die "keyring is missing signing key fingerprint: $key"
    fi
done < "$keys_file"
