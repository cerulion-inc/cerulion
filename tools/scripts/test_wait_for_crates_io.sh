#!/bin/sh

set -eu

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
set +e
output=$("$script_dir/wait_for_crates_io.sh" 0.1.0 \
    9223372036854775808 cerulion_core 2>&1)
status=$?
set -e
if [ "$status" -ne 2 ]; then
    printf '%s\n' 'error: arithmetic-overflow timeout was accepted' >&2
    exit 1
fi
printf '%s\n' "$output" | grep -Fxq \
    'error: invalid timeout: 9223372036854775808' || {
    printf '%s\n' 'error: arithmetic-overflow timeout had the wrong diagnostic' >&2
    printf '%s\n' "$output" >&2
    exit 1
}
printf '%s\n' 'wait_for_crates_io timeout range: passed'
