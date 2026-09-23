#!/bin/sh
#
# Build the signed Cerulion APT repository metadata around one or more debs.
#
set -eu

usage() {
    printf 'usage: %s REPO_DIR GPG_KEY_ID[,GPG_KEY_ID...] DEB [DEB ...]\n' "$0" >&2
}

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

[ "$#" -ge 3 ] || {
    usage
    exit 2
}

repo_dir=$1
key_ids=$2
shift 2

mkdir -p "$repo_dir"
repo_dir=$(CDPATH='' cd "$repo_dir" && pwd)
script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)

command -v apt-ftparchive >/dev/null 2>&1 ||
    die "apt-ftparchive is required to build the APT repository"
command -v dpkg-deb >/dev/null 2>&1 ||
    die "dpkg-deb is required to inspect Debian packages"
command -v gpg >/dev/null 2>&1 ||
    die "gpg is required to sign the APT repository"
command -v gzip >/dev/null 2>&1 ||
    die "gzip is required to compress APT indexes"
command -v sha256sum >/dev/null 2>&1 ||
    die "sha256sum is required to create APT by-hash indexes"
[ -n "$key_ids" ] || die "a signing key fingerprint is required"

stage_root=
generation_probe=
raw_keys=
keys_file=
keyring_raw_keys=
keyring_keys_file=
config=
pass_dir=
pass_file=
signature_file=
pool_manifest=
keyring_tmp=
previous_dists=
link_tmp=
restore_tmp=
promoted_generation=
cleanup() {
    status=$?
    rm -rf "$stage_root"
    [ -z "$promoted_generation" ] || rm -rf "$promoted_generation"
    [ -z "$raw_keys" ] || rm -f "$raw_keys"
    [ -z "$keys_file" ] || rm -f "$keys_file"
    [ -z "$keyring_raw_keys" ] || rm -f "$keyring_raw_keys"
    [ -z "$keyring_keys_file" ] || rm -f "$keyring_keys_file"
    [ -z "$config" ] || rm -f "$config"
    [ -z "$pass_dir" ] || rm -rf "$pass_dir"
    [ -z "$signature_file" ] || rm -f "$signature_file"
    [ -z "$pool_manifest" ] || rm -f "$pool_manifest"
    [ -z "$keyring_tmp" ] || rm -f "$keyring_tmp"
    [ -z "$previous_dists" ] || rm -rf "$previous_dists"
    [ -z "$link_tmp" ] || rm -f "$link_tmp"
    [ -z "$restore_tmp" ] || rm -f "$restore_tmp"
    exit "$status"
}
trap cleanup EXIT

stage_root=$(mktemp -d "$repo_dir/.apt-stage.XXXXXX") ||
    die "could not create repository staging directory"
generation_probe=$(mktemp -d "$repo_dir/dists.XXXXXX") ||
    die "could not create repository generation directory"
generation=$(basename "$generation_probe")
rmdir "$generation_probe" ||
    die "could not release repository generation directory"
stage_pool="$stage_root/pool"
new_stable="$stage_root/$generation/stable"
live_dists="$repo_dir/dists"
live_stable="$live_dists/stable"
pool_dir="$stage_pool/main/c/cerulion"

mkdir -p "$stage_pool"

raw_keys=$(mktemp "${TMPDIR:-/tmp}/cerulion-apt-keys.XXXXXX") ||
    die "could not create a temporary key list"
keys_file=$(mktemp "${TMPDIR:-/tmp}/cerulion-apt-normalized-keys.XXXXXX") ||
    die "could not create a normalized key list"
printf '%s\n' "$key_ids" | tr ',' '\n' > "$raw_keys"
while IFS= read -r key; do
    key=$(printf '%s' "$key" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')
    [ -n "$key" ] || die "signing key list contains an empty fingerprint"
    gpg --batch --list-secret-keys "$key" >/dev/null 2>&1 ||
        die "signing key is not available in the current GPG home: $key"
    printf '%s\n' "$key" >> "$keys_file"
done < "$raw_keys"
[ -s "$keys_file" ] || die "a signing key fingerprint is required"
primary_key=$(sed -n '1p' "$keys_file")

keyring_raw_keys=$(mktemp "${TMPDIR:-/tmp}/cerulion-apt-keyring-keys.XXXXXX") ||
    die "could not create a keyring key list"
keyring_keys_file=$(mktemp "${TMPDIR:-/tmp}/cerulion-apt-normalized-keyring-keys.XXXXXX") ||
    die "could not create a normalized keyring key list"
keyring_key_ids=${APT_KEYRING_KEYS:-$key_ids}
printf '%s\n' "$keyring_key_ids" | tr ',' '\n' > "$keyring_raw_keys"
while IFS= read -r key; do
    key=$(printf '%s' "$key" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')
    [ -n "$key" ] || die "keyring key list contains an empty fingerprint"
    gpg --batch --list-keys "$key" >/dev/null 2>&1 ||
        die "keyring public key is not available in the current GPG home: $key"
    printf '%s\n' "$key" >> "$keyring_keys_file"
done < "$keyring_raw_keys"
[ -s "$keyring_keys_file" ] || die "a keyring fingerprint is required"

raw_package_version=
package_version=
amd64_deb=
arm64_deb=
for deb in "$@"; do
    [ -f "$deb" ] || die "Debian package does not exist: $deb"
    package=$(dpkg-deb -f "$deb" Package)
    version=$(dpkg-deb -f "$deb" Version)
    arch=$(dpkg-deb -f "$deb" Architecture)
    [ "$package" = "cerulion" ] ||
        die "expected package cerulion, found $package in $deb"
    case "$arch" in
        amd64)
            if [ -n "$amd64_deb" ]; then
                die "duplicate Debian architecture 'amd64' in inputs: $amd64_deb and $deb"
            fi
            amd64_deb=$deb
            ;;
        arm64)
            if [ -n "$arm64_deb" ]; then
                die "duplicate Debian architecture 'arm64' in inputs: $arm64_deb and $deb"
            fi
            arm64_deb=$deb
            ;;
        *) die "unsupported Debian architecture '$arch' in $deb" ;;
    esac
    [ -n "$version" ] || die "package version is empty in $deb"
    if [ -z "$raw_package_version" ]; then
        raw_package_version=$version
        package_version=$version
    elif [ "$raw_package_version" != "$version" ]; then
        die "Debian packages have different versions: $raw_package_version and $version"
    fi
done

sha256_digest() {
    sha256_output=$(sha256sum "$1") ||
        return 1
    sha256_digest_value=${sha256_output%%[[:space:]]*}
    printf '%s\n' "$sha256_digest_value" |
        grep -Eq '^[[:xdigit:]]{64}$' || return 1
    printf '%s\n' "$sha256_digest_value"
}

if [ -e "$repo_dir/pool" ] || [ -L "$repo_dir/pool" ]; then
    [ -d "$repo_dir/pool" ] || die "APT pool is not a directory: $repo_dir/pool"
    pool_symlink=$(find "$repo_dir/pool" -type l -print -quit)
    [ -z "$pool_symlink" ] ||
        die "APT pool contains a symlink: $pool_symlink"
    cp -a "$repo_dir/pool/." "$stage_pool/"
fi
mkdir -p "$pool_dir"
for deb in "$@"; do
    version=$(dpkg-deb -f "$deb" Version)
    arch=$(dpkg-deb -f "$deb" Architecture)
    cp "$deb" "$pool_dir/cerulion_${version}_${arch}.deb"
done

keyring="$stage_root/cerulion-archive-keyring.gpg"
: > "$keyring"
chmod 0644 "$keyring"
while IFS= read -r key; do
    gpg --batch --yes --export "$key" >> "$keyring"
done < "$keyring_keys_file"
"$script_dir/check_apt_keyring_coverage.sh" "$keyring" "$keys_file"

# The address clients fetch from, written into the sources.list entry the
# keyring package carries. It is a publication fact, not something this script
# can derive from the tree, so it defaults to the repository the project
# publishes and documents; a mirror or a staging bucket overrides it, and a
# local repository served over HTTP for a test sets it to that server.
: "${APT_REPO_URL:=https://d2tdat71jcoj6e.cloudfront.net}"
export APT_REPO_URL

keyring_revision=${APT_KEYRING_REVISION:-1}
keyring_pool_dir="$stage_pool/main/c/cerulion-archive-keyring"
mkdir -p "$keyring_pool_dir"
keyring_package_stage="$stage_root/keyring-package"
mkdir -p "$keyring_package_stage"
keyring_package=$("$script_dir/build_keyring_deb.sh" "$keyring" "$package_version" \
    "$keyring_revision" "$keyring_package_stage")
keyring_package_name=$(basename "$keyring_package")
keyring_package_destination="$keyring_pool_dir/$keyring_package_name"
if [ -f "$keyring_package_destination" ] &&
    cmp -s "$keyring_package" "$keyring_package_destination"; then
    :
else
    [ ! -d "$keyring_package_destination" ] ||
        die "keyring package output path is a directory: $keyring_package_destination"
    mv -f "$keyring_package" "$keyring_package_destination"
fi

rm -rf "${stage_root:?}/$generation"
mkdir -p "$new_stable"
for arch in amd64 arm64; do
    binary_dir="$new_stable/main/binary-$arch"
    mkdir -p "$binary_dir"
    (
        cd "$stage_root"
        apt-ftparchive -a "$arch" packages pool/main
    ) > "$binary_dir/Packages"
    gzip -n -9 -c "$binary_dir/Packages" > "$binary_dir/Packages.gz"
    for index in Packages Packages.gz; do
        hash=$(sha256_digest "$binary_dir/$index") ||
            die "could not calculate a valid SHA-256 hash for $binary_dir/$index"
        mkdir -p "$binary_dir/by-hash/SHA256"
        cp "$binary_dir/$index" "$binary_dir/by-hash/SHA256/$hash"
    done
done

if [ -d "$live_stable" ]; then
    for old_hash_dir in "$live_stable"/main/binary-*/by-hash/SHA256; do
        [ -d "$old_hash_dir" ] || continue
        relative_dir=${old_hash_dir#"$live_stable"/}
        mkdir -p "$new_stable/$relative_dir"
        for object in "$old_hash_dir"/*; do
            [ -f "$object" ] || continue
            cp "$object" "$new_stable/$relative_dir/"
        done
    done
fi

config=$(mktemp "${TMPDIR:-/tmp}/cerulion-apt-release.XXXXXX") ||
    die "could not create a temporary APT release configuration"
cat > "$config" <<'EOF'
APT::FTPArchive::Release {
  Origin "Cerulion";
  Label "Cerulion";
  Suite "stable";
  Codename "stable";
  Architectures "amd64 arm64";
  Components "main";
  Acquire-By-Hash "yes";
};
EOF

apt-ftparchive -c "$config" release "$new_stable" > "$new_stable/.Release"
mv "$new_stable/.Release" "$new_stable/Release"

if [ -n "${GPG_PASSPHRASE:-}" ]; then
    pass_dir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-apt-passphrase.XXXXXX") ||
        die "could not create a temporary passphrase directory"
    chmod 0700 "$pass_dir"
    pass_file="$pass_dir/passphrase"
    (umask 077 && printf '%s' "$GPG_PASSPHRASE" > "$pass_file") ||
        die "could not write the temporary passphrase file"
    chmod 0600 "$pass_file"
fi

gpg_sign() {
    if [ -n "${GPG_PASSPHRASE:-}" ]; then
        gpg --batch --yes --pinentry-mode loopback \
            --passphrase-file "$pass_file" "$@"
    else
        gpg --batch --yes "$@"
    fi
}

gpg_sign --local-user "$primary_key" --clearsign \
    --output "$new_stable/InRelease" "$new_stable/Release"
: > "$new_stable/Release.gpg"
while IFS= read -r key; do
    signature_file=$(mktemp "${TMPDIR:-/tmp}/cerulion-apt-signature.XXXXXX") ||
        die "could not create a temporary signature file"
    gpg_sign --local-user "$key" --detach-sign --armor \
        --output "$signature_file" "$new_stable/Release"
    cat "$signature_file" >> "$new_stable/Release.gpg"
    rm -f "$signature_file"
    signature_file=
done < "$keys_file"

# No live repository path is changed until every index, signature, and keyring
# coverage check above has succeeded. New pool files are copied completely
# beside their destinations and atomically renamed into place; a failed copy
# therefore leaves existing pool files untouched. The keyring is copied to a
# same-directory temporary path and promoted only after the client-visible
# dists symlink swap succeeds. Because the keyring is checked to contain every
# signing key used for the generation, the publication workflow can order
# widening and narrowing key transitions without guessing. Older generations
# remain available for rollback and for any process that still has an open path
# into them.
mkdir -p "$repo_dir/pool"
pool_manifest=$(mktemp "${TMPDIR:-/tmp}/cerulion-apt-pool.XXXXXX") ||
    die "could not create a temporary pool manifest"
find "$stage_pool" -type f -name '*.deb' -print > "$pool_manifest" ||
    die "could not enumerate staged pool packages"
while IFS= read -r staged_deb; do
    relative_deb=${staged_deb#"$stage_pool"/}
    destination="$repo_dir/pool/$relative_deb"
    destination_dir=$(dirname "$destination")
    destination_name=$(basename "$destination")
    mkdir -p "$destination_dir"
    if [ -e "$destination" ] || [ -L "$destination" ]; then
        staged_hash=$(sha256_digest "$staged_deb") ||
            die "could not hash staged pool package: $staged_deb"
        existing_hash=$(sha256_digest "$destination") ||
            die "could not hash existing pool package: $destination"
        [ "$staged_hash" = "$existing_hash" ] ||
            die "existing pool package differs from staged artifact: $destination"
        continue
    fi
    temporary_destination=$(mktemp "$destination_dir/.${destination_name}.XXXXXX") ||
        die "could not create temporary pool package: $destination"
    if ! cp "$staged_deb" "$temporary_destination"; then
        rm -f "$temporary_destination"
        die "could not copy staged pool package: $destination"
    fi
    if ! chmod 0644 "$temporary_destination"; then
        rm -f "$temporary_destination"
        die "could not set permissions on staged pool package: $destination"
    fi
    if ! ln "$temporary_destination" "$destination"; then
        rm -f "$temporary_destination"
        die "could not publish pool package: $destination"
    fi
    rm -f "$temporary_destination"
done < "$pool_manifest"
keyring_tmp=$(mktemp "$repo_dir/.cerulion-archive-keyring.XXXXXX") ||
    die "could not create temporary archive keyring path"
cp "$keyring" "$keyring_tmp" ||
    die "could not stage archive keyring"
chmod 0644 "$keyring_tmp"
generation_dir="$repo_dir/$generation"
[ ! -e "$generation_dir" ] ||
    die "repository generation path already exists: $generation_dir"
mv "$stage_root/$generation" "$generation_dir"
promoted_generation=$generation_dir
previous_dists_kind=absent
if [ -L "$live_dists" ]; then
    previous_dists_kind=symlink
    previous_dists_target=$(readlink "$live_dists") ||
        die "could not inspect the current dists symlink"
elif [ -d "$live_dists" ]; then
    previous_dists_kind=directory
    previous_dists="$repo_dir/.dists-previous.$generation"
    cp -a "$live_dists" "$previous_dists" ||
        die "could not preserve the current dists directory"
elif [ -e "$live_dists" ]; then
    die "current dists path is neither a symlink nor a directory"
fi
link_tmp="$repo_dir/.dists-link.$generation"
ln -sfn "$(basename "$generation_dir")" "$link_tmp"
if ! mv -Tf "$link_tmp" "$live_dists"; then
    if [ "$previous_dists_kind" = directory ]; then
        die "could not atomically replace the current real dists directory"
    fi
    die "could not publish the new dists path"
fi
link_tmp=
keyring_destination="$repo_dir/cerulion-archive-keyring.gpg"
if [ -d "$keyring_destination" ]; then
    case "$previous_dists_kind" in
        symlink)
            restore_tmp="$repo_dir/.dists-restore.$generation"
            ln -s "$previous_dists_target" "$restore_tmp" ||
                die "could not publish the archive keyring or prepare dists rollback"
            mv -Tf "$restore_tmp" "$live_dists" ||
                die "could not publish the archive keyring or restore the current dists symlink"
            restore_tmp=
            ;;
        absent)
            rm -f "$live_dists" ||
                die "could not publish the archive keyring or remove the new dists path"
            ;;
        directory)
            if [ ! -d "$live_dists" ] || [ -L "$live_dists" ]; then
                die "could not publish the archive keyring or preserve the current dists directory"
            fi
            ;;
    esac
    die "keyring destination is a directory: $keyring_destination"
fi
if ! mv -f "$keyring_tmp" "$keyring_destination"; then
    case "$previous_dists_kind" in
        symlink)
            restore_tmp="$repo_dir/.dists-restore.$generation"
            ln -s "$previous_dists_target" "$restore_tmp" ||
                die "could not publish the archive keyring or prepare dists rollback"
            mv -Tf "$restore_tmp" "$live_dists" ||
                die "could not publish the archive keyring or restore the current dists symlink"
            restore_tmp=
            ;;
        absent)
            rm -f "$live_dists" ||
                die "could not publish the archive keyring or remove the new dists path"
            ;;
        directory)
            if [ ! -d "$live_dists" ] || [ -L "$live_dists" ]; then
                die "could not publish the archive keyring or preserve the current dists directory"
            fi
            ;;
    esac
    die "could not publish the archive keyring"
fi
keyring_tmp=
promoted_generation=
if [ -n "$previous_dists" ]; then
    rm -rf "$previous_dists" ||
        die "could not remove the preserved dists directory"
    previous_dists=
fi

printf '%s\n' "$repo_dir"
