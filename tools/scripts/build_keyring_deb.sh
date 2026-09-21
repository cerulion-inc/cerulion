#!/bin/sh
#
# Build the Debian package that installs the Cerulion APT archive keyring.
#
set -eu

usage() {
    printf 'usage: %s KEYRING VERSION REVISION OUTDIR\n' "$0" >&2
}

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

[ "$#" -eq 4 ] || {
    usage
    exit 2
}

keyring=$1
version=$2
revision=$3
outdir=$4

mkdir -p "$outdir" ||
    die "could not create output directory: $outdir"
outdir=$(CDPATH='' cd "$outdir" && pwd) ||
    die "could not resolve output directory: $outdir"
tmpdir=${TMPDIR:-/tmp}
case "$tmpdir" in
    /*) ;;
    *) tmpdir=$(CDPATH='' cd "$tmpdir" && pwd) ||
        die "could not resolve temporary directory: $tmpdir" ;;
esac

[ -f "$keyring" ] || die "archive keyring does not exist: $keyring"
command -v gpg >/dev/null 2>&1 ||
    die "gpg is required to validate the archive keyring"
keyring_fingerprints=$(TMPDIR="$tmpdir" gpg --batch --with-colons \
    --show-keys "$keyring" 2>/dev/null) ||
    die "archive keyring is not a valid GPG keyring: $keyring"
[ -n "$keyring_fingerprints" ] ||
    die "archive keyring is not a valid GPG keyring: $keyring"
secret_check_home=$(mktemp -d "$tmpdir/cerulion-keyring-secret-check.XXXXXX") ||
    die "could not create a temporary GPG home"
cleanup_secret_check_home() {
    [ -z "${secret_check_home:-}" ] || rm -rf "$secret_check_home"
    [ -z "${secret_check_output:-}" ] || rm -f "$secret_check_output"
}
trap cleanup_secret_check_home EXIT
secret_check_output=$(mktemp "$tmpdir/cerulion-keyring-secret-check-output.XXXXXX") ||
    die "could not create a temporary GPG output file"
chmod 0700 "$secret_check_home"
if ! TMPDIR="$tmpdir" gpg --batch --no-autostart \
    --homedir "$secret_check_home" --import-options show-only --with-colons \
    --import "$keyring" >"$secret_check_output" 2>/dev/null; then
    die "archive keyring is not a valid GPG keyring: $keyring"
fi
if ! awk -F: '$1 == "pub" || $1 == "sec" { found=1 }
    END { exit found ? 0 : 1 }' "$secret_check_output"; then
    die "could not inspect archive keyring for private key material: $keyring"
fi
secret_keys=$(awk -F: '$1 == "sec" { print }' "$secret_check_output")
rm -rf "$secret_check_home"
secret_check_home=
rm -f "$secret_check_output"
secret_check_output=
trap - EXIT
[ -z "$secret_keys" ] ||
    die "archive keyring contains private key material: $keyring"
[ -n "$version" ] || die "keyring package version is empty"
case "$version" in
    *"
"*) die "invalid keyring package version: $version" ;;
esac
printf '%s' "$version" |
    grep -Eq '^[0-9][0-9A-Za-z.+~-]*$' ||
    die "invalid keyring package version: $version"
version_upstream=$version
version_build=
case "$version" in
    *+*)
        version_upstream=${version%%+*}
        version_build=${version#*+}
        case "$version_build" in
            ''|*[!0-9A-Za-z.-]*) \
                die "keyring package version is not Debian-normalized: $version" ;;
            *) ;;
        esac
        ;;
    *) ;;
esac
case "$version_upstream" in
    *-*)
        version_revision=${version_upstream#*-}
        version_base=${version_upstream%%-*}
        case "$version_revision" in
            ''|[!0-9]*|*[!0-9A-Za-z.+~]*) \
                die "keyring package version is not Debian-normalized: $version" ;;
            *) ;;
        esac
        version_upstream=$version_base
        ;;
    *) ;;
esac
printf '%s' "$version_upstream" |
    grep -Eq '^[0-9]+\.[0-9]+\.[0-9]+(~[0-9A-Za-z.]+)?$' ||
    die "keyring package version is not Debian-normalized: $version"
case "$revision" in
    ''|*[!0-9]*|0[0-9]*) die "keyring revision must be a canonical non-negative integer: $revision" ;;
    *) ;;
esac
command -v dpkg-deb >/dev/null 2>&1 ||
    die "dpkg-deb is required to build a Debian package"
command -v ar >/dev/null 2>&1 ||
    die "ar is required to build a Debian package"
command -v tar >/dev/null 2>&1 ||
    die "tar is required to build a Debian package"

workdir=$(mktemp -d "$tmpdir/cerulion-keyring-deb.XXXXXX") ||
    die "could not create a temporary directory"
workdir=$(CDPATH='' cd "$workdir" && pwd) ||
    die "could not resolve temporary directory"
tmp_deb=
cleanup() {
    rm -rf "$workdir"
    [ -z "$tmp_deb" ] || rm -f "$tmp_deb"
}
trap cleanup EXIT

stage="$workdir/root"
mkdir -p "$stage/DEBIAN" "$stage/usr/share/keyrings"
chmod 0755 "$stage" "$stage/DEBIAN" "$stage/usr" "$stage/usr/share" \
    "$stage/usr/share/keyrings"
cp "$keyring" "$stage/usr/share/keyrings/cerulion-archive-keyring.gpg"
chmod 0644 "$stage/usr/share/keyrings/cerulion-archive-keyring.gpg"

cat > "$stage/DEBIAN/control" <<EOF
Package: cerulion-archive-keyring
Version: $version-$revision
Architecture: all
Section: admin
Priority: optional
Maintainer: Cerulion <packaging@cerulion.com>
Homepage: https://cerulion.com
Description: Cerulion APT archive signing keys
 The public keys used to verify Cerulion APT repository metadata.
EOF
chmod 0644 "$stage/DEBIAN/control"

deb="$outdir/cerulion-archive-keyring_${version}-${revision}_all.deb"
[ ! -d "$deb" ] || die "output path is a directory: $deb"
package_workdir="$workdir/package"
mkdir -p "$package_workdir"
printf '2.0\n' > "$package_workdir/debian-binary"
(cd "$stage/DEBIAN" && tar --sort=name --mtime="@${SOURCE_DATE_EPOCH:-0}" \
    --owner=0 --group=0 --numeric-owner -czf "$package_workdir/control.tar.gz" .)
(cd "$stage" && tar --sort=name --mtime="@${SOURCE_DATE_EPOCH:-0}" \
    --owner=0 --group=0 --numeric-owner --exclude ./DEBIAN \
    -czf "$package_workdir/data.tar.gz" .)
tmp_deb=$(mktemp "$outdir/.cerulion-archive-keyring.deb.XXXXXX") ||
    die "could not create temporary Debian package"
chmod 0600 "$tmp_deb"
printf '!<arch>\n' > "$tmp_deb"
ar rcsD "$tmp_deb" "$package_workdir/debian-binary" \
    "$package_workdir/control.tar.gz" "$package_workdir/data.tar.gz" ||
    die "could not create Debian package"
chmod 0644 "$tmp_deb" ||
    die "could not set Debian package permissions"
mv "$tmp_deb" "$deb" ||
    die "could not promote Debian package"
tmp_deb=
printf '%s\n' "$deb"
