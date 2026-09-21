#!/bin/sh
#
# Wait until released Cerulion crates are visible in crates.io's sparse index.
#
set -eu

usage() {
    printf 'usage: %s VERSION TIMEOUT_SECONDS CRATE [CRATE ...]\n' "$0" >&2
}

decimal_leq() {
    left=$1
    right=$2
    [ "${#left}" -lt "${#right}" ] && return 0
    [ "${#left}" -gt "${#right}" ] && return 1
    while [ -n "$left" ]; do
        left_digit=${left%"${left#?}"}
        right_digit=${right%"${right#?}"}
        if [ "$left_digit" -lt "$right_digit" ]; then
            return 0
        fi
        if [ "$left_digit" -gt "$right_digit" ]; then
            return 1
        fi
        left=${left#?}
        right=${right#?}
    done
    return 0
}

index_path() {
    crate=$1
    case "$crate" in
        ?)
            printf '1/%s\n' "$crate"
            ;;
        ??)
            printf '2/%s\n' "$crate"
            ;;
        ???)
            first=${crate%"${crate#?}"}
            printf '3/%s/%s\n' "$first" "$crate"
            ;;
        *)
            first_two=${crate%"${crate#??}"}
            remainder=${crate#??}
            second_two=${remainder%"${remainder#??}"}
            printf '%s/%s/%s\n' "$first_two" "$second_two" "$crate"
            ;;
    esac
}

if [ "$#" -lt 3 ]; then
    usage
    exit 2
fi

version=$1
timeout_seconds=$2
shift 2

if [ -z "$version" ]; then
    usage
    exit 2
fi

printf '%s' "$version" |
    grep -Eq '^[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?$' ||
    { printf 'error: invalid crates.io version: %s\n' "$version" >&2; exit 2; }
case "$timeout_seconds" in
    ''|*[!0-9]*)
        printf 'error: invalid timeout: %s\n' "$timeout_seconds" >&2
        exit 2
        ;;
esac
# Strip leading zeroes: /bin/sh reads 08 as an invalid octal literal in $(( )).
timeout_seconds=${timeout_seconds#"${timeout_seconds%%[!0]*}"}
timeout_seconds=${timeout_seconds:-0}
decimal_leq "$timeout_seconds" 9223372036854775807 || {
    printf 'error: invalid timeout: %s\n' "$timeout_seconds" >&2
    exit 2
}

for crate in "$@"; do
    case "$crate" in
        ''|*[!A-Za-z0-9_-]*)
            printf 'error: invalid crate name: %s\n' "$crate" >&2
            exit 2
            ;;
    esac
done

command -v curl >/dev/null 2>&1 ||
    { printf 'error: curl is required to poll crates.io\n' >&2; exit 1; }
command -v date >/dev/null 2>&1 ||
    { printf 'error: date is required to poll crates.io\n' >&2; exit 1; }

now=$(date +%s)
arithmetic_limit=$((9223372036854775807 - now))
if ! decimal_leq "$timeout_seconds" "$arithmetic_limit"; then
    printf 'error: invalid timeout: %s\n' "$timeout_seconds" >&2
    exit 2
fi
deadline=$(( now + timeout_seconds ))
missing_crates="$*"
while [ "$(date +%s)" -lt "$deadline" ]; do
    missing_crates=
    checked_crates=
    deadline_reached=0
    for crate in "$@"; do
        now=$(date +%s)
        if [ "$now" -ge "$deadline" ]; then
            deadline_reached=1
            break
        fi
        remaining=$(( deadline - now ))
        [ "$remaining" -gt 0 ] || remaining=1
        crate_index_name=$(printf '%s' "$crate" | tr '[:upper:]' '[:lower:]')
        index_url="https://index.crates.io/$(index_path "$crate_index_name")"
        if ! index=$(curl -fsSL --max-time "$remaining" \
            --silent --show-error "$index_url"); then
            missing_crates="$missing_crates $crate"
        elif ! printf '%s\n' "$index" | grep -Fq "\"vers\":\"$version\""; then
            missing_crates="$missing_crates $crate"
        fi
        checked_crates="$checked_crates $crate"
    done
    if [ "$deadline_reached" -eq 1 ]; then
        for crate in "$@"; do
            case " $checked_crates " in
                *" $crate "*) ;;
                *) missing_crates="$missing_crates $crate" ;;
            esac
        done
    fi
    if [ -z "$missing_crates" ]; then
        printf 'crates.io index contains %s for version %s\n' "$*" "$version"
        exit 0
    fi
    now=$(date +%s)
    if [ "$now" -ge "$deadline" ]; then
        break
    fi
    remaining=$(( deadline - now ))
    sleep_seconds=15
    [ "$sleep_seconds" -lt "$remaining" ] || sleep_seconds=$remaining
    sleep "$sleep_seconds"
done

missing_crates=${missing_crates# }
printf 'error: crates.io did not expose %s for version %s within %s seconds; the crates.io publish is a missing precondition\n' \
    "$missing_crates" "$version" "$timeout_seconds" >&2
exit 1
