#!/usr/bin/env bash
# Refuse to publish a crate whose package would carry anything but source,
# manifest, README, license and notice files. Run before every `cargo publish`.
#
#   publish_preflight.sh <crate name>...      (workspace members, by package name)
#   publish_preflight.sh --dir <crate dir>    (a standalone crate directory)
#
# `cargo package --list` is the oracle: it prints exactly the files the
# registry would receive. Every path must match one of the allowed shapes
# below; a log, a note, an agent instruction file, an editor artifact or a
# stray directory fails the run and names the offender. A crate directory
# without an explicit `include` list is refused outright, because then the
# package is whatever the directory happens to hold.
set -euo pipefail

# Three of these are cargo's own: `.cargo_vcs_info.json` (the commit the package was
# built from, added inside any git checkout), `Cargo.toml.orig` and `Cargo.lock`.
# `upstream_msg_manifest.txt` is native_ros2_messages' vendored-message manifest.
allowed='^(\.cargo_vcs_info\.json|Cargo\.toml|Cargo\.toml\.orig|Cargo\.lock|README\.md|LICENSE([-A-Z0-9]+)?|NOTICE|upstream_msg_manifest\.txt|build\.rs|src/.*|msg/.*|benches/.*|examples/.*)$'
fail=0

check_listing() {
    local label=$1 listing=$2 manifest=$3
    if ! grep -qE '^include *= *\[' "$manifest"; then
        echo "publish preflight: $label: Cargo.toml has no include list; the package would be whatever the directory holds" >&2
        fail=1
    fi
    while IFS= read -r path; do
        [ -n "$path" ] || continue
        if ! printf '%s\n' "$path" | grep -qE "$allowed"; then
            echo "publish preflight: $label: refusing to package '$path'" >&2
            fail=1
        fi
        case "$path" in
            *AGENTS.md|*CLAUDE.md|*.log|*.DS_Store|*notes/*|*-DESIGN.md|*-RUNNING-LOG.md)
                echo "publish preflight: $label: '$path' is never published" >&2; fail=1 ;;
        esac
    done <<<"$listing"
}

if [ "${1:-}" = "--dir" ]; then
    dir=${2:?crate directory}
    listing=$(cd "$dir" && cargo package --list --allow-dirty 2>/dev/null)
    check_listing "$dir" "$listing" "$dir/Cargo.toml"
else
    for crate in "$@"; do
        listing=$(cargo package --list --allow-dirty -p "$crate" 2>/dev/null)
        manifest=$(cargo metadata --no-deps --format-version 1 | python3 -c 'import json,sys; c=sys.argv[1]; m=[p["manifest_path"] for p in json.load(sys.stdin)["packages"] if p["name"]==c]; print(m[0] if m else "")' "$crate")
        if [ -z "$manifest" ]; then
            echo "publish preflight: $crate: not a workspace member cargo metadata knows" >&2
            fail=1; continue
        fi
        check_listing "$crate" "$listing" "$manifest"
    done
fi
if [ "$fail" -ne 0 ]; then
    echo "publish preflight: FAILED; nothing may be published until the listing is clean" >&2
    exit 1
fi
echo "publish preflight: OK"
