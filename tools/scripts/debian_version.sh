#!/bin/sh
#
# Convert a semver release version to its Debian version spelling.
#
set -eu

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

[ "$#" -eq 1 ] || {
    printf 'usage: %s VERSION\n' "$0" >&2
    exit 2
}

version=$1
[ -n "$version" ] || die "version is empty"
version_without_controls=$(printf '%s' "$version" | LC_ALL=C tr -d '[:cntrl:]')
if [ "$version_without_controls" != "$version" ]; then
    die "version contains a control character"
fi
printf '%s' "$version" |
    LC_ALL=C grep -Eq '^[0-9][0-9A-Za-z.+~-]*$' ||
    die "invalid version: $version"

version_without_build=$version
build_metadata=
has_build=0
case "$version_without_build" in
    *+*)
        build_metadata=${version_without_build#*+}
        version_without_build=${version_without_build%%+*}
        has_build=1
        ;;
esac
core=$version_without_build
prerelease=
has_prerelease=0
case "$version_without_build" in
    *-*)
        base=${version_without_build%%-*}
        prerelease=${version_without_build#"$base"-}
        core=$base
        has_prerelease=1
        ;;
esac
case "$core" in
    *.*.*) ;;
    *) die "invalid version: $version" ;;
esac
core_first=${core%%.*}
core_rest=${core#*.}
core_second=${core_rest%%.*}
core_third=${core_rest#*.}
case "$core_third" in
    *.*) die "invalid version: $version" ;;
    *) ;;
esac
for component in "$core_first" "$core_second" "$core_third"; do
    case "$component" in
        ''|*[!0-9]*|0[0-9]*) die "invalid version: $version" ;;
        0|[1-9]*) ;;
        *) die "invalid version: $version" ;;
    esac
done
validate_identifiers() {
    identifiers=$1
    label=$2
    [ -n "$identifiers" ] || die "invalid empty $label identifier: $version"
    case "$identifiers" in
        .*|*.|*..*) die "invalid empty $label identifier in version: $version" ;;
        *) ;;
    esac
    old_ifs=$IFS
    IFS=.
    # shellcheck disable=SC2086
    set -- $identifiers
    IFS=$old_ifs
    for identifier do
        case "$identifier" in
            ''|*[!0-9A-Za-z-]*) die "invalid $label identifier in version: $version" ;;
            *) ;;
        esac
        if [ "$label" = prerelease ]; then
            case "$identifier" in
                *-*) die "prerelease identifiers containing '-' are not supported: $version" ;;
                0|*[A-Za-z]*) ;;
                [1-9]*) ;;
                *) die "invalid numeric prerelease identifier in version: $version" ;;
            esac
        elif [ "$label" = build ]; then
            case "$identifier" in
                *-*) die "build metadata identifiers containing '-' are not supported: $version" ;;
                *) ;;
            esac
        fi
    done
}
if [ "$has_prerelease" -eq 1 ]; then
    validate_identifiers "$prerelease" prerelease
    version_without_build=$core~$prerelease
else
    version_without_build=$core
fi
if [ "$has_build" -eq 1 ]; then
    validate_identifiers "$build_metadata" build
    printf '%s+%s\n' "$version_without_build" "$build_metadata"
else
    printf '%s\n' "$version_without_build"
fi

# A trailing comment, so this file appears in a diff without changing behaviour.
