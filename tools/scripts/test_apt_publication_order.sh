#!/bin/sh

set -eu

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-apt-publication-order.XXXXXX")
cleanup() {
    rm -rf "$workdir"
}
trap cleanup EXIT

export GNUPGHOME="$workdir/gnupg"
mkdir -m 0700 "$GNUPGHOME"
gpg --batch --passphrase '' --quick-generate-key \
    'Cerulion publication-order A <apt-order-a@cerulion.com>' rsa1024 sign 1d
gpg --batch --passphrase '' --quick-generate-key \
    'Cerulion publication-order B <apt-order-b@cerulion.com>' rsa1024 sign 1d
fingerprint_a=$(gpg --batch --with-colons --list-secret-keys |
    awk -F: '$1 == "fpr" { print $10; exit }')
fingerprint_b=$(gpg --batch --with-colons --list-secret-keys |
    awk -F: -v first="$fingerprint_a" '$1 == "fpr" && $10 != first { print $10; exit }')
[ -n "$fingerprint_a" ] && [ -n "$fingerprint_b" ]

repo="$workdir/repo"
mkdir -p "$repo/dists/stable"
touch "$repo/dists/stable/Release" "$repo/dists/stable/Release.gpg" \
    "$repo/dists/stable/InRelease"
mkdir -p "$repo/dists/stable/main/binary-amd64/by-hash/SHA256"
printf '%s\n' immutable-index > \
    "$repo/dists/stable/main/binary-amd64/by-hash/SHA256/immutable"
published_keyring="$workdir/published.gpg"
published_overlap_keyring="$workdir/published-overlap.gpg"
incoming_keyring="$repo/cerulion-archive-keyring.gpg"
gpg --batch --export "$fingerprint_a" > "$published_keyring"
gpg --batch --export "$fingerprint_a" "$fingerprint_b" > "$published_overlap_keyring"
gpg --batch --export "$fingerprint_a" "$fingerprint_b" > "$incoming_keyring"
coverage_keys="$workdir/coverage-keys"
printf '%s\n' "$fingerprint_a" > "$coverage_keys"
printf '%s' "$fingerprint_b" >> "$coverage_keys"
if coverage_output=$("$script_dir/check_apt_keyring_coverage.sh" "$published_keyring" \
    "$coverage_keys" 2>&1); then
    printf 'error: newline-free missing signing key was accepted\n' >&2
    exit 1
fi
printf '%s\n' "$coverage_output" |
    grep -Fq "keyring is missing signing key fingerprint: $fingerprint_b" || {
        printf 'error: newline-free missing signing key had the wrong diagnostic\n' >&2
        printf '%s\n' "$coverage_output" >&2
        exit 1
    }
printf '%s\n' 'newline-free keyring coverage rejection passed'
invalid_coverage_keyring="$workdir/invalid-coverage-keyring.gpg"
printf '%s\n' 'not-a-keyring' > "$invalid_coverage_keyring"
if invalid_coverage_output=$("$script_dir/check_apt_keyring_coverage.sh" \
    "$invalid_coverage_keyring" "$coverage_keys" 2>&1); then
    printf '%s\n' 'error: malformed keyring inspection unexpectedly succeeded' >&2
    exit 1
fi
printf '%s\n' "$invalid_coverage_output" |
    grep -Fq "could not inspect archive keyring: $invalid_coverage_keyring" || {
    printf '%s\n' 'error: malformed keyring inspection had the wrong diagnostic' >&2
    printf '%s\n' "$invalid_coverage_output" >&2
    exit 1
}
printf '%s\n' "$invalid_coverage_output" | grep -Fq 'gpg:' || {
    printf '%s\n' 'error: malformed keyring inspection omitted gpg stderr' >&2
    printf '%s\n' "$invalid_coverage_output" >&2
    exit 1
}
printf '%s\n' 'gpg keyring inspection failure propagation passed'
coverage_signal_bin="$workdir/coverage-signal-bin"
coverage_signal_tmp="$workdir/coverage-signal-tmp"
mkdir "$coverage_signal_bin" "$coverage_signal_tmp"
real_gpg=$(command -v gpg)
cat > "$coverage_signal_bin/gpg" <<'EOF'
#!/bin/sh
kill -TERM "$PPID"
sleep 1
exec "$APT_REAL_GPG" "$@"
EOF
chmod 0755 "$coverage_signal_bin/gpg"
coverage_signal_status=0
if TMPDIR="$coverage_signal_tmp" PATH="$coverage_signal_bin:$PATH" \
    APT_REAL_GPG="$real_gpg" "$script_dir/check_apt_keyring_coverage.sh" \
    "$published_keyring" "$coverage_keys" >/dev/null 2>&1; then
    coverage_signal_status=0
else
    coverage_signal_status=$?
fi
if [ "$coverage_signal_status" -ne 143 ]; then
    printf 'error: interrupted keyring coverage returned %s instead of 143\n' \
        "$coverage_signal_status" >&2
    exit 1
fi
if find "$coverage_signal_tmp" -maxdepth 1 -name 'cerulion-apt-keyring-*' -print |
    grep -q .; then
    printf '%s\n' 'error: interrupted keyring coverage left temporary inspection files' >&2
    exit 1
fi
printf '%s\n' 'keyring coverage interruption cleanup passed'

mock_bin="$workdir/bin"
mkdir "$mock_bin"
mock_s3_root="$workdir/mock-s3"
mkdir -p "$mock_s3_root"
cat > "$mock_bin/aws" <<'EOF'
#!/bin/sh

printf '%s\n' "$*" >> "$AWS_LOG"
remote_path() {
    case "$1" in
    s3://bucket/repo)
        printf '%s\n' "$AWS_MOCK_S3_ROOT"
        ;;
    s3://bucket/repo/*)
        printf '%s/%s\n' "$AWS_MOCK_S3_ROOT" "${1#s3://bucket/repo/}"
        ;;
    *)
        return 1
        ;;
    esac
}
if [ "$1" = s3api ] && [ "$2" = head-object ]; then
    bucket=
    key=
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --bucket) bucket=$2; shift 2 ;;
            --key) key=$2; shift 2 ;;
            *) shift ;;
        esac
    done
    if [ "${AWS_FAIL_DETACHED_HEAD:-0}" -eq 1 ] &&
        printf '%s\n' "$key" | grep -Eiq '/(Release|Release\.gpg)$'; then
        printf '%s\n' 'An error occurred (AccessDenied) when calling the HeadObject operation' >&2
        exit 1
    fi
    object_uri="s3://$bucket/$key"
    if object_path=$(remote_path "$object_uri") &&
        [ -f "$object_path" ]; then
        printf '%s\n' '{}'
        exit 0
    fi
    printf '%s\n' 'An error occurred (404) when calling the HeadObject operation: Not Found' >&2
    exit 1
fi
if [ "${AWS_FAIL_INRELEASE:-0}" -eq 1 ] &&
    [ "$1" = s3 ] && [ "$2" = cp ] &&
    printf '%s' "$4" | grep -Fq 's3://bucket/repo/dists/stable/InRelease'; then
    if [ ! -e "$AWS_FAIL_MARKER" ]; then
        : > "$AWS_FAIL_MARKER"
        exit 1
    fi
fi
if [ "${AWS_FAIL_KEYRING_CAPTURE:-0}" -eq 1 ] &&
    [ "$1" = s3 ] && [ "$2" = cp ] &&
    printf '%s' "$3" | grep -Fq '/cerulion-archive-keyring.gpg' &&
    printf '%s' "$4" | grep -Fq '/cerulion-apt-keyring.'; then
    exit 1
fi
if [ "${AWS_FAIL_KEYRING_RESTORE_INVALIDATION:-0}" -eq 1 ] &&
    [ "$1" = cloudfront ] && [ "$2" = create-invalidation ] &&
    printf '%s' "$*" | grep -Fq '/cerulion-archive-keyring.gpg'; then
    exit 1
fi
if [ "${AWS_FAIL_SIGNED_RESTORE:-0}" -eq 1 ] &&
    [ "$1" = s3 ] && [ "$2" = cp ] &&
    printf '%s' "$3" | grep -Eq '/cerulion-apt-metadata\.[^/]*/(Release|Release.gpg|InRelease)$'; then
    exit 1
fi
if [ "${AWS_FAIL_INDEX_RESTORE:-0}" -eq 1 ] &&
    [ "$1" = s3 ] && [ "$2" = cp ] &&
    printf '%s' "$3" | grep -Fq '/indexes/' &&
    remote_path "$4" >/dev/null 2>&1; then
    exit 1
fi
if [ "${AWS_FAIL_CANONICAL_SYNC:-0}" -eq 1 ] &&
    [ "$1" = s3 ] && [ "$2" = sync ] &&
    ! printf '%s' "$*" | grep -Fq -- '--include */by-hash/*'; then
    exit 1
fi
if [ "${AWS_FAIL_CANONICAL_ONCE:-0}" -eq 1 ] &&
    [ "$1" = s3 ] && [ "$2" = sync ] &&
    ! printf '%s' "$*" | grep -Fq -- '--include */by-hash/*' &&
    [ ! -e "$AWS_FAIL_MARKER" ]; then
    : > "$AWS_FAIL_MARKER"
    exit 1
fi
if [ "$1" = s3 ] && [ "$2" = cp ]; then
    if src_remote=$(remote_path "$3"); then
        [ -f "$src_remote" ] || {
            printf '%s\n' '404 Not Found' >&2
            exit 1
        }
        mkdir -p "$(dirname "$4")"
        [ "${AWS_FAIL_COPY:-0}" -eq 1 ] && exit 1
        cp "$src_remote" "$4" || exit 1
        exit 0
    fi
    if dst_remote=$(remote_path "$4"); then
        mkdir -p "$(dirname "$dst_remote")"
        [ "${AWS_FAIL_COPY:-0}" -eq 1 ] && exit 1
        cp "$3" "$dst_remote" || exit 1
        if [ "${AWS_SIGNAL_ON_KEYRING_UPLOAD:-0}" -eq 1 ] &&
            printf '%s' "$4" | grep -Fq '/cerulion-archive-keyring.gpg' &&
            [ ! -e "$AWS_SIGNAL_MARKER" ]; then
            : > "$AWS_SIGNAL_MARKER"
            kill -"${AWS_SIGNAL_NAME:-INT}" "$AWS_SIGNAL_PID"
            sleep 1
        fi
        exit 0
    fi
fi
if [ "$1" = s3 ] && [ "$2" = sync ]; then
    source_root=$3
    destination_root=$4
    shift 4
    sync_allowed() {
        sync_relative=$1
        shift
        sync_include=1
        while [ "$#" -gt 0 ]; do
            case "$1" in
                --exclude|--include)
                    sync_rule=$1
                    sync_pattern=$2
                    shift 2
                    case "$sync_relative" in
                        $sync_pattern)
                            if [ "$sync_rule" = '--exclude' ]; then
                                sync_include=0
                            else
                                sync_include=1
                            fi
                            ;;
                    esac
                    ;;
                *)
                    shift
                    ;;
            esac
        done
        [ "$sync_include" -eq 1 ]
    }
    find -L "$source_root" -type f -print |
        while IFS= read -r source_path; do
            relative_path=${source_path#"$source_root"}
            sync_allowed "$relative_path" "$@" || continue
            destination_uri="${destination_root%/}/$relative_path"
            destination_path=$(remote_path "$destination_uri") || exit 1
            mkdir -p "$(dirname "$destination_path")"
            cp "$source_path" "$destination_path"
        done
    exit 0
fi
if [ "${AWS_FAIL_KEYRING_INVALIDATION:-0}" -eq 1 ] &&
    printf '%s' "$*" | grep -Fq '/cerulion-archive-keyring.gpg' &&
    [ ! -e "$AWS_FAIL_KEYRING_INVALIDATION_MARKER" ]; then
    : > "$AWS_FAIL_KEYRING_INVALIDATION_MARKER"
    exit 1
fi
if [ "${AWS_FAIL_METADATA_INVALIDATION:-0}" -eq 1 ] &&
    printf '%s' "$*" | grep -Fq '/dists/*'; then
    if [ -e "$AWS_FAIL_METADATA_INVALIDATION_MARKER" ]; then
        exit 1
    fi
    : > "$AWS_FAIL_METADATA_INVALIDATION_MARKER"
fi
if [ "${AWS_FAIL_METADATA_INVALIDATION_ALWAYS:-0}" -eq 1 ] &&
    printf '%s' "$*" | grep -Fq '/dists/*'; then
    : > "$AWS_FAIL_METADATA_INVALIDATION_MARKER"
    exit 1
fi
if [ "${AWS_FAIL_POOL_INVALIDATION_ALWAYS:-0}" -eq 1 ] &&
    printf '%s' "$*" | grep -Fq '/pool/*'; then
    : > "$AWS_FAIL_POOL_INVALIDATION_MARKER"
    exit 1
fi
case "$*" in
    *'cloudfront create-invalidation'*'/dists/'*) printf '%s\n' dists-id; exit 0 ;;
    *'cloudfront create-invalidation'*'/cerulion-archive-keyring.gpg'*) printf '%s\n' keyring-id; exit 0 ;;
    *'cloudfront create-invalidation'*'/pool/'*) printf '%s\n' pool-id; exit 0 ;;
esac
if [ "$1" = cloudfront ] && [ "$2" = create-invalidation ]; then
    printf 'error: unsupported CloudFront invalidation path: %s\n' "${6:-}" >&2
    exit 1
fi
if [ "$1" = cloudfront ] && [ "$2" = wait ] &&
    [ "$3" = invalidation-completed ]; then
    case "$*" in
        *'--id dists-id'|*'--id keyring-id'|*'--id pool-id'*) exit 0 ;;
        *) exit 1 ;;
    esac
fi
printf 'error: unsupported aws invocation: %s (argv1=%s argv2=%s)\n' \
    "$*" "$1" "$2" >&2
exit 1
EOF
chmod 0755 "$mock_bin/aws"
export AWS_MOCK_S3_ROOT="$mock_s3_root"
export PATH="$mock_bin:$PATH"
assert_metadata_order() {
    log_file=$1
    [ -f "$log_file" ] || {
        printf 'error: publication fixture did not record a publication: missing log %s\n' \
            "$log_file" >&2
        exit 1
    }
    [ -s "$log_file" ] || {
        printf 'error: publication fixture did not record a publication: empty log %s\n' \
            "$log_file" >&2
        exit 1
    }
    for required_operation in \
        's3 sync ' \
        's3 cp '"$repo"'/dists/stable/InRelease ' \
        'cloudfront create-invalidation '; do
        grep -Fq "$required_operation" "$log_file" || {
            printf 'error: publication fixture did not record a publication step: %s\n' \
                "$required_operation" >&2
            cat "$log_file" >&2
            exit 1
        }
    done
    by_hash_line=$(grep -nF \
        "s3 sync $repo/dists/ s3://bucket/repo/dists/ --follow-symlinks --exclude * --exclude */Release --exclude */Release.gpg --exclude */InRelease --include */by-hash/*" \
        "$log_file" | head -n 1 | cut -d: -f1)
    inrelease_line=$(grep -nF \
        "s3 cp $repo/dists/stable/InRelease s3://bucket/repo/dists/stable/InRelease" \
        "$log_file" | head -n 1 | cut -d: -f1)
    canonical_line=$(grep -nF \
        "s3 sync $repo/dists/ s3://bucket/repo/dists/ --follow-symlinks --exclude */Release --exclude */Release.gpg --exclude */InRelease --exclude */by-hash/*" \
        "$log_file" | head -n 1 | cut -d: -f1)
    detached_uploads=$(grep -Ec \
        's3 cp .*s3://bucket/repo/dists/stable/(Release|Release\.gpg)$' \
        "$log_file" || :)
    if [ -z "$by_hash_line" ] || [ -z "$inrelease_line" ] ||
        [ -z "$canonical_line" ] ||
        [ "$detached_uploads" -ne 0 ] ||
        [ "$by_hash_line" -ge "$inrelease_line" ] ||
        [ "$inrelease_line" -ge "$canonical_line" ]; then
        printf 'error: metadata publication did not order by-hash, signed, canonical\n' >&2
        cat "$log_file" >&2
        exit 1
    fi
    if grep -E '^s3 sync ' "$log_file" |
        grep -Ev -- '--exclude \*/Release( |$)' >/dev/null ||
        grep -E '^s3 sync ' "$log_file" |
        grep -Ev -- '--exclude \*/Release\.gpg( |$)' >/dev/null; then
        printf 'error: a metadata sync can upload detached metadata\n' >&2
        cat "$log_file" >&2
        exit 1
    fi
}
publication_mutations() {
    grep -E '^(s3 cp |s3 sync |cloudfront )' "$1" |
        grep -Ev \
            '^s3 cp s3://bucket/repo/cerulion-archive-keyring.gpg /[^ ]*/cerulion-apt-keyring\.[^ ]+$' ||
        :
}
assert_widening_order() {
    log_file=$1
    [ -f "$log_file" ] || {
        printf 'error: publication fixture did not record a publication: missing log %s\n' \
            "$log_file" >&2
        exit 1
    }
    [ -s "$log_file" ] || {
        printf 'error: publication fixture did not record a publication: empty log %s\n' \
            "$log_file" >&2
        exit 1
    }
    for required_operation in \
        's3 cp '"$repo"'/cerulion-archive-keyring.gpg ' \
        'cloudfront create-invalidation ' \
        'cloudfront wait invalidation-completed ' \
        's3 sync '"$repo"'/dists/ '; do
        grep -Fq "$required_operation" "$log_file" || {
            printf 'error: publication fixture did not record a publication step: %s\n' \
                "$required_operation" >&2
            cat "$log_file" >&2
            exit 1
        }
    done
    first_mutation=$(publication_mutations "$log_file" | head -n 1)
    case "$first_mutation" in
        "s3 cp $repo/cerulion-archive-keyring.gpg "*) ;;
        *)
            printf 'error: widening publication did not upload the keyring first\n' >&2
            cat "$log_file" >&2
            exit 1
            ;;
    esac
    keyring_upload_line=$(grep -nF "s3 cp $repo/cerulion-archive-keyring.gpg" "$log_file" |
        cut -d: -f1)
    keyring_invalidation_line=$(grep -nF \
        'cloudfront create-invalidation --distribution-id distribution-id --paths /cerulion-archive-keyring.gpg' \
        "$log_file" | cut -d: -f1)
    keyring_wait_line=$(grep -nF \
        'cloudfront wait invalidation-completed --distribution-id distribution-id --id keyring-id' \
        "$log_file" | cut -d: -f1)
    metadata_upload_line=$(grep -nF \
        "s3 sync $repo/dists/ s3://bucket/repo/dists/ --follow-symlinks --exclude * --exclude */Release --exclude */Release.gpg --exclude */InRelease --include */by-hash/*" \
        "$log_file" | head -n 1 | cut -d: -f1)
    canonical_line=$(grep -nF \
        "s3 sync $repo/dists/ s3://bucket/repo/dists/ --follow-symlinks --exclude */Release --exclude */Release.gpg --exclude */InRelease --exclude */by-hash/*" \
        "$log_file" | head -n 1 | cut -d: -f1)
    final_invalidation_line=$(grep -nF \
        'cloudfront create-invalidation --distribution-id distribution-id --paths /dists/* /pool/*' \
        "$log_file" | cut -d: -f1)
    final_wait_line=$(grep -nF \
        'cloudfront wait invalidation-completed --distribution-id distribution-id --id dists-id' \
        "$log_file" | tail -n 1 | cut -d: -f1)
    if [ -z "$keyring_upload_line" ] || [ -z "$keyring_invalidation_line" ] ||
        [ -z "$keyring_wait_line" ] || [ -z "$metadata_upload_line" ] ||
        [ -z "$final_invalidation_line" ] || [ -z "$final_wait_line" ] ||
        [ -z "$canonical_line" ] ||
        [ "$keyring_upload_line" -ge "$keyring_invalidation_line" ] ||
        [ "$keyring_invalidation_line" -ge "$keyring_wait_line" ] ||
        [ "$keyring_wait_line" -ge "$metadata_upload_line" ] ||
        [ "$final_invalidation_line" -le "$canonical_line" ] ||
        [ "$final_invalidation_line" -ge "$final_wait_line" ]; then
        printf 'error: widening publication did not wait for keyring invalidation before metadata\n' >&2
        cat "$log_file" >&2
        exit 1
    fi
    assert_metadata_order "$log_file"
}

rm -rf "${mock_s3_root:?}"/*
export AWS_LOG="$workdir/aws-orphan.log"
: > "$AWS_LOG"
gpg --batch --export "$fingerprint_a" > "$incoming_keyring"
export AWS_FAIL_INRELEASE=1
export AWS_FAIL_MARKER="$workdir/orphan-failure-marker"
if orphan_failure_output=$("$script_dir/publish_apt_repo.sh" "$repo" \
    s3://bucket/repo distribution-id "" "" "$fingerprint_a" 2>&1); then
    printf '%s\n' 'error: orphaned-keyring metadata failure unexpectedly succeeded' >&2
    exit 1
fi
printf '%s\n' "$orphan_failure_output" |
    grep -Fq 'orphaned APT keyring retained in the remote repository' || {
    printf '%s\n' 'error: orphaned-keyring failure omitted the retained keyring diagnostic' >&2
    printf '%s\n' "$orphan_failure_output" >&2
    exit 1
}
unset AWS_FAIL_INRELEASE AWS_FAIL_MARKER
orphaned_published_keyring="$workdir/orphaned-published.gpg"
cp "$mock_s3_root/cerulion-archive-keyring.gpg" "$orphaned_published_keyring"
gpg --batch --export "$fingerprint_b" > "$incoming_keyring"
"$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo distribution-id "" \
    "$orphaned_published_keyring" "$fingerprint_b" > "$workdir/orphan-retry-output"
cmp "$incoming_keyring" "$mock_s3_root/cerulion-archive-keyring.gpg"
printf '%s\n' 'virgin orphaned-keyring replacement passed'
rm -rf "${mock_s3_root:?}"/*

duplicate_deb_dir="$workdir/duplicate-debs"
duplicate_repo="$workdir/duplicate-repo"
for duplicate_name in first second; do
    duplicate_root="$duplicate_deb_dir/$duplicate_name"
    mkdir -p "$duplicate_root/DEBIAN"
    cat > "$duplicate_root/DEBIAN/control" <<EOF
Package: cerulion
Version: 0.1.0
Architecture: amd64
Maintainer: Cerulion <apt-order@cerulion.com>
Description: duplicate architecture fixture
EOF
    dpkg-deb --build "$duplicate_root" "$duplicate_deb_dir/$duplicate_name.deb" \
        >/dev/null
done
if duplicate_output=$("$script_dir/build_apt_repo.sh" "$duplicate_repo" "$fingerprint_a" \
    "$duplicate_deb_dir/first.deb" "$duplicate_deb_dir/second.deb" 2>&1); then
    printf 'error: duplicate Debian architecture was accepted\n' >&2
    exit 1
fi
printf '%s\n' "$duplicate_output" |
    grep -Fq "duplicate Debian architecture 'amd64' in inputs: $duplicate_deb_dir/first.deb and $duplicate_deb_dir/second.deb" || {
        printf 'error: duplicate architecture had the wrong diagnostic\n' >&2
        printf '%s\n' "$duplicate_output" >&2
        exit 1
    }
[ ! -e "$duplicate_repo/pool" ] || {
    printf 'error: duplicate architecture staged a package before validation\n' >&2
    exit 1
}

second_mktemp_repo="$workdir/second-mktemp-repo"
second_mktemp_bin="$workdir/second-mktemp-bin"
mkdir "$second_mktemp_bin"
real_mktemp=$(command -v mktemp)
cat > "$second_mktemp_bin/mktemp" <<'EOF'
#!/bin/sh
count_file=$APT_MKTEMP_COUNT_FILE
count=0
if [ -f "$count_file" ]; then
    count=$(cat "$count_file")
fi
count=$((count + 1))
printf '%s\n' "$count" > "$count_file"
if [ "$count" -eq 2 ]; then
    exit 1
fi
exec "$APT_REAL_MKTEMP" "$@"
EOF
chmod 0755 "$second_mktemp_bin/mktemp"
if APT_MKTEMP_COUNT_FILE="$workdir/second-mktemp-count" \
    APT_REAL_MKTEMP="$real_mktemp" \
    PATH="$second_mktemp_bin:$PATH" \
    "$script_dir/build_apt_repo.sh" "$second_mktemp_repo" "$fingerprint_a" \
    "$duplicate_deb_dir/first.deb" >"$workdir/second-mktemp-output" 2>&1; then
    printf '%s\n' 'error: second APT staging allocation failure was accepted' >&2
    exit 1
fi
if find "$second_mktemp_repo" -maxdepth 1 -name '.apt-stage.*' -print |
    grep -q .; then
    printf '%s\n' 'error: second APT staging allocation left a temporary directory' >&2
    exit 1
fi
printf '%s\n' 'build_apt_repo second temporary allocation cleanup passed'

symlink_pool_repo="$workdir/symlink-pool-repo"
mkdir -p "$symlink_pool_repo/pool/main/c"
ln -s /tmp "$symlink_pool_repo/pool/main/c/escape"
if symlink_pool_output=$("$script_dir/build_apt_repo.sh" "$symlink_pool_repo" \
    "$fingerprint_a" "$duplicate_deb_dir/first.deb" 2>&1); then
    printf '%s\n' 'error: symlinked APT pool entry was accepted' >&2
    exit 1
fi
printf '%s\n' "$symlink_pool_output" |
    grep -Fq "APT pool contains a symlink: $symlink_pool_repo/pool/main/c/escape" || {
        printf '%s\n' 'error: symlinked APT pool entry had the wrong diagnostic' >&2
        printf '%s\n' "$symlink_pool_output" >&2
        exit 1
    }
printf '%s\n' 'APT pool symlink rejection passed'

rollback_repo="$workdir/rollback-repo"
"$script_dir/build_apt_repo.sh" "$rollback_repo" "$fingerprint_a" \
    "$duplicate_deb_dir/first.deb" >/dev/null
rollback_target=$(readlink "$rollback_repo/dists")
rollback_keyring=$(sha256sum "$rollback_repo/cerulion-archive-keyring.gpg" |
    cut -d' ' -f1)
rollback_keyring_package=$(find "$rollback_repo/pool" -type f \
    -name 'cerulion-archive-keyring_*.deb' -print -quit)
rollback_keyring_package_inode=$(stat -c '%d:%i' "$rollback_keyring_package")
rollback_generations_before=$(find "$rollback_repo" -maxdepth 1 -type d \
    -name 'dists.*' -printf '%f\n' | sort)
cat > "$mock_bin/mv" <<'EOF'
#!/bin/sh
last=
for arg do last=$arg; done
case "$last" in
    */cerulion-archive-keyring.gpg) exit 1 ;;
esac
exec /bin/mv "$@"
EOF
chmod 0755 "$mock_bin/mv"
if PATH="$mock_bin:$PATH" "$script_dir/build_apt_repo.sh" "$rollback_repo" \
    "$fingerprint_a" "$duplicate_deb_dir/first.deb" >"$workdir/rollback-output" 2>&1; then
    printf 'error: keyring promotion failure unexpectedly succeeded\n' >&2
    exit 1
fi
[ "$(readlink "$rollback_repo/dists")" = "$rollback_target" ] || {
    printf 'error: keyring failure did not restore the previous dists symlink\n' >&2
    exit 1
}
rollback_keyring_after=$(sha256sum "$rollback_repo/cerulion-archive-keyring.gpg" |
    cut -d' ' -f1)
if [ "$rollback_keyring_after" != "$rollback_keyring" ]; then
    printf 'error: keyring failure changed the published keyring\n' >&2
    exit 1
fi
if [ "$rollback_keyring_package_inode" != \
    "$(stat -c '%d:%i' "$rollback_keyring_package")" ]; then
    printf 'error: identical keyring package was unnecessarily replaced\n' >&2
    exit 1
fi
rollback_generations_after=$(find "$rollback_repo" -maxdepth 1 -type d \
    -name 'dists.*' -printf '%f\n' | sort)
[ "$rollback_generations_after" = "$rollback_generations_before" ] || {
    printf '%s\n' 'error: keyring promotion failure left an unpromoted generation behind' >&2
    printf 'before:\n%s\nafter:\n%s\n' "$rollback_generations_before" \
        "$rollback_generations_after" >&2
    exit 1
}
if find "$rollback_repo" -maxdepth 1 \( -name '.dists-previous.*' -o \
    -name '.dists-link.*' -o -name '.dists-restore.*' \) -print | grep -q .; then
    printf 'error: keyring failure left a dists promotion temporary behind\n' >&2
    exit 1
fi
cat > "$mock_bin/mv" <<'EOF'
#!/bin/sh
last=
for arg do last=$arg; done
if [ "$last" = "$APT_FAIL_DISTS_SWAP_TARGET" ]; then
    exit 1
fi
exec /bin/mv "$@"
EOF
chmod 0755 "$mock_bin/mv"
rollback_generations_before_dists=$(find "$rollback_repo" -maxdepth 1 \
    -type d -name 'dists.*' -printf '%f\n' | sort)
if PATH="$mock_bin:$PATH" APT_FAIL_DISTS_SWAP_TARGET="$rollback_repo/dists" \
    "$script_dir/build_apt_repo.sh" "$rollback_repo" "$fingerprint_a" \
    "$duplicate_deb_dir/first.deb" >"$workdir/dists-swap-output" 2>&1; then
    printf '%s\n' 'error: dists promotion failure unexpectedly succeeded' >&2
    exit 1
fi
[ "$(readlink "$rollback_repo/dists")" = "$rollback_target" ] || {
    printf '%s\n' 'error: dists failure changed the published dists symlink' >&2
    exit 1
}
rollback_generations_after_dists=$(find "$rollback_repo" -maxdepth 1 \
    -type d -name 'dists.*' -printf '%f\n' | sort)
[ "$rollback_generations_after_dists" = "$rollback_generations_before_dists" ] || {
    printf '%s\n' 'error: dists promotion failure left an unpromoted generation behind' >&2
    printf 'before:\n%s\nafter:\n%s\n' "$rollback_generations_before_dists" \
        "$rollback_generations_after_dists" >&2
    exit 1
}
rm "$mock_bin/mv"
printf '%s\n' 'APT promotion failure generation cleanup passed'

directory_repo="$workdir/directory-repo"
mkdir -p "$directory_repo/dists"
cp -a "$rollback_repo/$rollback_target/." "$directory_repo/dists/"
printf '%s\n' preserved > "$directory_repo/dists/rollback-sentinel"
if "$script_dir/build_apt_repo.sh" "$directory_repo" "$fingerprint_a" \
    "$duplicate_deb_dir/first.deb" >"$workdir/directory-output" 2>&1; then
    printf 'error: real-directory dists was replaced non-atomically\n' >&2
    exit 1
fi
[ -f "$directory_repo/dists/rollback-sentinel" ] || {
    printf 'error: real-directory dists was not preserved\n' >&2
    exit 1
}
if find "$directory_repo" -maxdepth 1 \( -name '.dists-previous.*' -o \
    -name '.dists-link.*' -o -name '.dists-restore.*' \) -print | grep -q .; then
    printf 'error: real-directory dists left a promotion temporary behind\n' >&2
    exit 1
fi

keyring_directory_repo="$workdir/keyring-directory-repo"
"$script_dir/build_apt_repo.sh" "$keyring_directory_repo" "$fingerprint_a" \
    "$duplicate_deb_dir/first.deb" >/dev/null
rm "$keyring_directory_repo/cerulion-archive-keyring.gpg"
mkdir "$keyring_directory_repo/cerulion-archive-keyring.gpg"
if "$script_dir/build_apt_repo.sh" "$keyring_directory_repo" "$fingerprint_a" \
    "$duplicate_deb_dir/first.deb" >"$workdir/keyring-directory-output" 2>&1; then
    printf 'error: directory-valued keyring destination was accepted\n' >&2
    exit 1
fi
grep -q 'keyring destination is a directory' "$workdir/keyring-directory-output" || {
    printf 'error: directory-valued keyring destination had the wrong diagnostic\n' >&2
    cat "$workdir/keyring-directory-output" >&2
    exit 1
}
[ -d "$keyring_directory_repo/cerulion-archive-keyring.gpg" ]

export AWS_LOG="$workdir/aws-widening.log"
: > "$AWS_LOG"

"$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo distribution-id "" "" \
    "$fingerprint_b" \
    > "$workdir/first-publish-output"
grep -qx 'APT keyring publication order: widening' "$workdir/first-publish-output"
first_publish_operation=$(publication_mutations "$AWS_LOG" | head -n 1)
case "$first_publish_operation" in
    *"s3 cp $repo/cerulion-archive-keyring.gpg"*) ;;
    *)
        printf 'error: first publication did not choose widening keyring-first order\n' >&2
        cat "$AWS_LOG" >&2
        exit 1
        ;;
esac
assert_widening_order "$AWS_LOG"
mutation_widening_log="$workdir/aws-mutated-widening.log"
mutation_invalidation_line=$(grep -nF \
    'cloudfront create-invalidation --distribution-id distribution-id --paths /dists/* /pool/*' \
    "$AWS_LOG" | head -n 1 | cut -d: -f1)
mutation_canonical_line=$(grep -nF \
    "s3 sync $repo/dists/ s3://bucket/repo/dists/ --follow-symlinks --exclude */Release --exclude */Release.gpg --exclude */InRelease --exclude */by-hash/*" \
    "$AWS_LOG" | head -n 1 | cut -d: -f1)
if [ -z "$mutation_invalidation_line" ] ||
    [ -z "$mutation_canonical_line" ]; then
    printf '%s\n' 'error: publication fixture did not record the widening mutation steps' >&2
    cat "$AWS_LOG" >&2
    exit 1
fi
awk -v invalidation_line="$mutation_invalidation_line" \
    -v canonical_line="$mutation_canonical_line" '
    NR == invalidation_line { invalidation = $0; next }
    NR == canonical_line { print invalidation; print; next }
    { print }
' "$AWS_LOG" > "$mutation_widening_log"
if (assert_widening_order "$mutation_widening_log") \
    >"$workdir/mutation-widening-output" 2>&1; then
    printf 'error: publication-order mutation unexpectedly passed\n' >&2
    cat "$workdir/mutation-widening-output" >&2
    exit 1
fi
printf '%s\n' 'publication-order mutation rejection passed'
if [ -e "$mock_s3_root/dists/stable/Release" ] ||
    [ -e "$mock_s3_root/dists/stable/Release.gpg" ]; then
    printf 'error: normal publication uploaded detached metadata\n' >&2
    exit 1
fi
printf '%s\n' old-release > "$mock_s3_root/dists/stable/Release"
: > "$AWS_LOG"
if detached_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "" "$fingerprint_b" 2>&1); then
    printf 'error: stale detached metadata publication unexpectedly succeeded\n' >&2
    exit 1
fi
printf '%s\n' "$detached_output" |
    grep -Fq 'remote repository already serves detached fallback metadata at object keys:' || {
    printf 'error: stale detached metadata had the wrong diagnostic\n' >&2
    printf '%s\n' "$detached_output" >&2
    exit 1
}
printf '%s\n' "$detached_output" |
    grep -Fq 'aws s3 rm s3://bucket/repo/dists/stable/Release' || {
    printf 'error: stale detached metadata omitted the removal command\n' >&2
    printf '%s\n' "$detached_output" >&2
    exit 1
}
if grep -Eq 's3 cp|s3 sync|cloudfront' "$AWS_LOG"; then
    printf 'error: stale detached metadata guard made a mutating AWS call\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
fi
rm "$mock_s3_root/dists/stable/Release"
printf '%s\n' 'detached metadata preflight guard passed'
export AWS_FAIL_DETACHED_HEAD=1
: > "$AWS_LOG"
if detached_lookup_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "" "$fingerprint_b" 2>&1); then
    printf 'error: detached metadata lookup failure unexpectedly succeeded\n' >&2
    exit 1
fi
printf '%s\n' "$detached_lookup_output" |
    grep -Fq 'could not determine whether detached fallback metadata exists' || {
    printf 'error: detached metadata lookup failure had the wrong diagnostic\n' >&2
    printf '%s\n' "$detached_lookup_output" >&2
    exit 1
}
if grep -Eq 's3 cp|s3 sync|cloudfront' "$AWS_LOG"; then
    printf 'error: detached metadata lookup failure made a mutating AWS call\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
fi
unset AWS_FAIL_DETACHED_HEAD
printf '%s\n' 'detached metadata lookup failure guard passed'
[ -f "$mock_s3_root/dists/stable/main/binary-amd64/by-hash/SHA256/immutable" ] || {
    printf 'error: widening publication stored by-hash data at the wrong path\n' >&2
    exit 1
}
[ ! -e "$mock_s3_root/distsstable" ] || {
    printf 'error: widening publication lost the dists path separator\n' >&2
    exit 1
}

gpg --batch --export "$fingerprint_a" > "$incoming_keyring"
: > "$AWS_LOG"
"$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo distribution-id "" \
    "$published_keyring" "$fingerprint_a" > "$workdir/widening-output"
first_operation=$(publication_mutations "$AWS_LOG" | head -n 1)
case "$first_operation" in
    *"s3 cp $repo/cerulion-archive-keyring.gpg"*) ;;
    *)
        printf 'error: widening publication did not upload the keyring first\n' >&2
        cat "$AWS_LOG" >&2
        exit 1
        ;;
esac
assert_widening_order "$AWS_LOG"

export AWS_LOG="$workdir/aws-rejected-widening.log"
: > "$AWS_LOG"
gpg --batch --export "$fingerprint_b" > "$incoming_keyring"
if direct_replacement_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "$published_keyring" "$fingerprint_b" 2>&1); then
    printf 'error: direct A-to-B replacement was accepted\n' >&2
    exit 1
fi
printf '%s\n' "$direct_replacement_output" |
    grep -q "direct replacement is refused; widen from A to A+B first" || {
        printf 'error: direct replacement had the wrong diagnostic\n' >&2
        printf '%s\n' "$direct_replacement_output" >&2
        exit 1
    }
if grep -Eq 's3 cp|s3 sync|cloudfront' "$AWS_LOG"; then
    printf 'error: rejected direct replacement made a mutating AWS call\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
fi

gpg --batch --export "$fingerprint_a" "$fingerprint_b" > "$incoming_keyring"
if rejected_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "$published_keyring" "$fingerprint_b" 2>&1); then
    printf 'error: widening publication with a new primary signer was accepted\n' >&2
    exit 1
fi
printf '%s\n' "$rejected_output" |
    grep -q "primary InRelease signer $fingerprint_b is not in the currently published APT keyring" || {
        printf 'error: rejected widening publication had the wrong diagnostic\n' >&2
        printf '%s\n' "$rejected_output" >&2
        exit 1
    }
if grep -Eq 's3 cp|s3 sync|cloudfront' "$AWS_LOG"; then
    printf 'error: rejected widening publication made a mutating AWS call\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
fi

gpg --batch --export "$fingerprint_b" > "$incoming_keyring"
export AWS_LOG="$workdir/aws-narrowing.log"
: > "$AWS_LOG"
"$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo distribution-id "" \
    "$published_overlap_keyring" "$fingerprint_b" > "$workdir/narrowing-output"
assert_metadata_order "$AWS_LOG"
metadata_upload_line=$(grep -nF \
    "s3 sync $repo/dists/ s3://bucket/repo/dists/ --follow-symlinks --exclude * --exclude */Release --exclude */Release.gpg --exclude */InRelease --include */by-hash/*" \
    "$AWS_LOG" | head -n 1 | cut -d: -f1)
canonical_line=$(grep -nF \
    "s3 sync $repo/dists/ s3://bucket/repo/dists/ --follow-symlinks --exclude */Release --exclude */Release.gpg --exclude */InRelease --exclude */by-hash/*" \
    "$AWS_LOG" | head -n 1 | cut -d: -f1)
metadata_wait_line=$(grep -n '^cloudfront wait .*--id dists-id$' "$AWS_LOG" | cut -d: -f1)
metadata_invalidation_line=$(grep -nF \
    'cloudfront create-invalidation --distribution-id distribution-id --paths /dists/*' \
    "$AWS_LOG" | cut -d: -f1)
keyring_upload_line=$(grep -n "s3 cp $repo/cerulion-archive-keyring.gpg" "$AWS_LOG" |
    cut -d: -f1)
if [ -z "$metadata_upload_line" ] || [ -z "$canonical_line" ] ||
   [ -z "$metadata_invalidation_line" ] ||
   [ -z "$metadata_wait_line" ] || [ -z "$keyring_upload_line" ] ||
   [ "$canonical_line" -ge "$metadata_invalidation_line" ] ||
   [ "$metadata_wait_line" -ge "$keyring_upload_line" ]; then
        printf 'error: narrowing publication did not wait for metadata before keyring upload\n' >&2
        cat "$AWS_LOG" >&2
        exit 1
fi

export AWS_REPO="$repo"
export AWS_FAIL_INRELEASE=1
export AWS_FAIL_MARKER="$workdir/fail-inrelease"
fallback_index_dir="$mock_s3_root/dists/stable/main/binary-amd64"
mkdir -p "$fallback_index_dir" "$repo/dists/stable/main/binary-amd64"
printf '%s\n' old-packages > "$fallback_index_dir/Packages"
printf '%s\n' old-packages-gzip > "$fallback_index_dir/Packages.gz"
printf '%s\n' new-packages > "$repo/dists/stable/main/binary-amd64/Packages"
printf '%s\n' new-packages-gzip > "$repo/dists/stable/main/binary-amd64/Packages.gz"
fallback_packages_sha=$(sha256sum "$fallback_index_dir/Packages" | cut -d' ' -f1)
fallback_packages_gz_sha=$(sha256sum "$fallback_index_dir/Packages.gz" | cut -d' ' -f1)
cat > "$mock_s3_root/dists/stable/InRelease" <<EOF
SHA256:
 $fallback_packages_sha $(wc -c < "$fallback_index_dir/Packages") main/binary-amd64/Packages
 $fallback_packages_gz_sha $(wc -c < "$fallback_index_dir/Packages.gz") main/binary-amd64/Packages.gz
EOF
printf '%s\n' old-inrelease > "$mock_s3_root/dists/stable/InRelease"
fallback_inrelease=$(cat "$mock_s3_root/dists/stable/InRelease")
fallback_packages=$(cat "$fallback_index_dir/Packages")
fallback_packages_gz=$(cat "$fallback_index_dir/Packages.gz")
gpg --batch --export "$fingerprint_a" "$fingerprint_b" > "$incoming_keyring"
: > "$AWS_LOG"
if "$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo distribution-id "" \
    "$published_overlap_keyring" "$fingerprint_a" >"$workdir/fallback-output" 2>&1; then
    printf 'error: InRelease failure unexpectedly succeeded\n' >&2
    exit 1
fi
grep -q 'could not publish APT InRelease metadata' "$workdir/fallback-output" || {
    printf 'error: InRelease failure had the wrong diagnostic\n' >&2
    cat "$workdir/fallback-output" >&2
    exit 1
}
[ "$(grep -c "s3 cp .*s3://bucket/repo/dists/stable/InRelease$" "$AWS_LOG")" -eq 2 ] || {
    printf 'error: signed-metadata failure did not restore InRelease\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
}
[ "$(cat "$mock_s3_root/dists/stable/InRelease")" = "$fallback_inrelease" ] || {
    printf 'error: signed-metadata failure changed InRelease\n' >&2
    exit 1
}
[ "$(cat "$fallback_index_dir/Packages")" = "$fallback_packages" ] || {
    printf 'error: signed-metadata failure changed Packages\n' >&2
    exit 1
}
[ "$(cat "$fallback_index_dir/Packages.gz")" = "$fallback_packages_gz" ] || {
    printf 'error: signed-metadata failure changed Packages.gz\n' >&2
    exit 1
}
if grep -qF -- \
    "s3 sync $repo/dists/ s3://bucket/repo/dists/ --follow-symlinks --exclude */Release --exclude */Release.gpg --exclude */InRelease --exclude */by-hash/*" \
    "$AWS_LOG"; then
    printf 'error: signed-metadata failure reached canonical index publication\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
fi
signed_rollback_invalidation=$(grep -cF \
    'cloudfront create-invalidation --distribution-id distribution-id --paths /dists/*' \
    "$AWS_LOG")
[ "$signed_rollback_invalidation" -ge 1 ] || {
    printf 'error: signed-metadata rollback did not invalidate CloudFront\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
}

export AWS_FAIL_SIGNED_RESTORE=1
rm -f "$AWS_FAIL_MARKER"
: > "$AWS_LOG"
if signed_restore_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "$published_overlap_keyring" "$fingerprint_a" 2>&1); then
    printf 'error: signed-metadata restore failure unexpectedly succeeded\n' >&2
    exit 1
fi
printf '%s\n' "$signed_restore_output" |
    grep -q 'failed to restore the previous signed APT metadata generation' || {
        printf 'error: signed-metadata restore failure had the wrong diagnostic\n' >&2
        printf '%s\n' "$signed_restore_output" >&2
        exit 1
    }
signed_backup_dir=$(printf '%s\n' "$signed_restore_output" |
    sed -n 's/^error: retained metadata backup directory: //p' | tail -n 1)
if [ -z "$signed_backup_dir" ] || [ ! -d "$signed_backup_dir" ]; then
    printf 'error: signed-metadata restore failure did not retain its backup directory\n' >&2
    printf '%s\n' "$signed_restore_output" >&2
    exit 1
fi
printf '%s\n' 'signed-metadata restore backup retention passed'

unset AWS_FAIL_SIGNED_RESTORE
unset AWS_FAIL_INRELEASE AWS_FAIL_MARKER
printf '%s\n' "$fallback_inrelease" > "$mock_s3_root/dists/stable/InRelease"
export AWS_FAIL_CANONICAL_ONCE=1
export AWS_FAIL_MARKER="$workdir/fail-canonical-once"
: > "$AWS_LOG"
if canonical_rollback_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "$published_overlap_keyring" "$fingerprint_a" 2>&1); then
    printf 'error: one-shot canonical failure unexpectedly succeeded\n' >&2
    exit 1
fi
grep -q 'could not publish canonical APT metadata indexes' <<EOF
$canonical_rollback_output
EOF
[ "$(cat "$fallback_index_dir/Packages")" = "$fallback_packages" ] || {
    printf 'error: successful canonical rollback changed Packages\n' >&2
    exit 1
}
[ "$(cat "$fallback_index_dir/Packages.gz")" = "$fallback_packages_gz" ] || {
    printf 'error: successful canonical rollback changed Packages.gz\n' >&2
    exit 1
}
[ "$(cat "$mock_s3_root/dists/stable/InRelease")" = "$fallback_inrelease" ] || {
    printf 'error: successful canonical rollback changed InRelease\n' >&2
    exit 1
}
canonical_failure_line=$(grep -nF \
    "s3 sync $repo/dists/ s3://bucket/repo/dists/ --follow-symlinks --exclude */Release --exclude */Release.gpg --exclude */InRelease --exclude */by-hash/*" \
    "$AWS_LOG" | head -n 1 | cut -d: -f1)
rollback_invalidation_line=$(grep -nF \
    'cloudfront create-invalidation --distribution-id distribution-id --paths /dists/*' \
    "$AWS_LOG" | tail -n 1 | cut -d: -f1)
if [ -z "$rollback_invalidation_line" ] ||
    [ "$rollback_invalidation_line" -le "$canonical_failure_line" ]; then
    printf 'error: canonical rollback did not invalidate CloudFront after restoration\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
fi

printf '%s\n' new-inrelease > "$repo/dists/stable/InRelease"
unset AWS_FAIL_CANONICAL_ONCE AWS_FAIL_MARKER
export AWS_FAIL_INDEX_RESTORE=1
export AWS_FAIL_CANONICAL_SYNC=1
: > "$AWS_LOG"
if mixed_state_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "$published_overlap_keyring" "$fingerprint_a" 2>&1); then
    printf 'error: persistent index restore failure unexpectedly succeeded\n' >&2
    exit 1
fi
printf '%s\n' "$mixed_state_output" |
    grep -q 'repository is in a mixed state' || {
        printf 'error: persistent index restore failure lacked the mixed-state diagnostic\n' >&2
        printf '%s\n' "$mixed_state_output" >&2
        exit 1
    }
printf '%s\n' "$mixed_state_output" |
    grep -q 'retained metadata backup directory' || {
        printf 'error: persistent index restore failure lacked retained-backup guidance\n' >&2
        printf '%s\n' "$mixed_state_output" >&2
        exit 1
    }
printf '%s\n' "$mixed_state_output" |
    grep -q 'while IFS= read -r index_rel' || {
        printf 'error: persistent index restore failure lacked copyable index recovery commands\n' >&2
        printf '%s\n' "$mixed_state_output" >&2
        exit 1
    }
if grep -qE 's3 cp .*s3://bucket/repo/dists/stable/(Release|Release.gpg)$' \
    "$AWS_LOG"; then
    printf 'error: persistent index restore attempted detached metadata upload\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
fi
mixed_state_invalidation_line=$(grep -nF \
    'cloudfront create-invalidation --distribution-id distribution-id --paths /dists/*' \
    "$AWS_LOG" | tail -n 1 | cut -d: -f1)
[ -n "$mixed_state_invalidation_line" ] || {
    printf 'error: persistent index restore failure did not invalidate CloudFront\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
}
unset AWS_FAIL_INRELEASE AWS_FAIL_MARKER AWS_FAIL_INDEX_RESTORE \
    AWS_FAIL_CANONICAL_SYNC AWS_METADATA_PRESENT AWS_REPO

export AWS_FAIL_SIGNED_RESTORE=1
export AWS_FAIL_CANONICAL_SYNC=1
gpg --batch --export "$fingerprint_b" > "$incoming_keyring"
: > "$AWS_LOG"
if mismatched_generation_output=$("$script_dir/publish_apt_repo.sh" "$repo" \
    s3://bucket/repo distribution-id "" "$published_overlap_keyring" \
    "$fingerprint_b" 2>&1); then
    printf 'error: failed InRelease restoration unexpectedly succeeded\n' >&2
    exit 1
fi
inrelease_restore_attempts=$(grep -c \
    's3 cp .*cerulion-apt-metadata\.[^/]*/InRelease s3://bucket/repo/dists/stable/InRelease' \
    "$AWS_LOG" || true)
if [ "$inrelease_restore_attempts" -ne 3 ]; then
    printf 'error: failed InRelease restoration did not use all retries\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
fi
printf '%s\n' "$mismatched_generation_output" |
    grep -q 'signed metadata restoration failed after 3 attempts' || {
    printf 'error: failed InRelease restoration omitted the retry diagnostic\n' >&2
    printf '%s\n' "$mismatched_generation_output" >&2
    exit 1
}
printf '%s\n' "$mismatched_generation_output" |
    grep -q 'signed metadata from the new generation against restored old indexes' || {
    printf 'error: failed InRelease restoration omitted the mismatch diagnostic\n' >&2
    printf '%s\n' "$mismatched_generation_output" >&2
    exit 1
}
printf '%s\n' "$mismatched_generation_output" |
    grep -q 'Acquire::By-Hash=false clients will fail apt-get update' || {
    printf 'error: failed InRelease restoration omitted the client impact\n' >&2
    printf '%s\n' "$mismatched_generation_output" >&2
    exit 1
}
printf '%s\n' "$mismatched_generation_output" |
    grep -q 'aws s3 cp .*cerulion-apt-metadata\.[^/]*/InRelease s3://bucket/repo/dists/stable/InRelease' || {
    printf 'error: failed InRelease restoration omitted the repair command\n' >&2
    printf '%s\n' "$mismatched_generation_output" >&2
    exit 1
}
mismatched_generation_invalidation_line=$(grep -nF \
    'cloudfront create-invalidation --distribution-id distribution-id --paths /dists/*' \
    "$AWS_LOG" | tail -n 1 | cut -d: -f1)
[ -n "$mismatched_generation_invalidation_line" ] || {
    printf 'error: failed InRelease restoration did not invalidate CloudFront\n' >&2
    cat "$AWS_LOG" >&2
    exit 1
}
printf '%s\n' "$mismatched_generation_output" |
    grep -q 'retained metadata backup:' || {
    printf 'error: failed InRelease restoration did not retain its backup\n' >&2
    printf '%s\n' "$mismatched_generation_output" >&2
    exit 1
}
unset AWS_FAIL_SIGNED_RESTORE AWS_FAIL_CANONICAL_SYNC
printf '%s\n' 'failed InRelease restoration mismatch recovery passed'

cat > "$workdir/run-signal-virgin" <<EOF
#!/bin/sh
export AWS_SIGNAL_PID=\$\$
export AWS_SIGNAL_NAME=\${1:?}
exec "$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo distribution-id "" "" \
    "$fingerprint_a"
EOF
chmod 0755 "$workdir/run-signal-virgin"
rm -f "$mock_s3_root/cerulion-archive-keyring.gpg"
gpg --batch --export "$fingerprint_a" > "$incoming_keyring"
export AWS_SIGNAL_ON_KEYRING_UPLOAD=1
export AWS_SIGNAL_NAME=TERM
export AWS_SIGNAL_MARKER="$workdir/signal-virgin-term"
rm -f "$AWS_SIGNAL_MARKER"
: > "$AWS_LOG"
if "$workdir/run-signal-virgin" TERM \
    >"$workdir/signal-virgin-output" 2>&1; then
    printf '%s\n' 'error: SIGTERM-during-virgin publication unexpectedly succeeded' >&2
    exit 1
else
    signal_virgin_status=$?
fi
[ "$signal_virgin_status" -eq 143 ] || {
    printf 'error: SIGTERM-during-virgin returned %s instead of 143\n' \
        "$signal_virgin_status" >&2
    cat "$workdir/signal-virgin-output" >&2
    exit 1
}
grep -Fq \
    'orphaned APT keyring retained in the remote repository: s3://bucket/repo/cerulion-archive-keyring.gpg' \
    "$workdir/signal-virgin-output" || {
    printf '%s\n' 'error: SIGTERM-during-virgin omitted the orphaned-keyring diagnostic' >&2
    cat "$workdir/signal-virgin-output" >&2
    exit 1
}
grep -Fq \
    'aws cloudfront create-invalidation --distribution-id distribution-id --paths "/cerulion-archive-keyring.gpg"' \
    "$workdir/signal-virgin-output" || {
    printf '%s\n' 'error: SIGTERM-during-virgin omitted cache recovery guidance' >&2
    cat "$workdir/signal-virgin-output" >&2
    exit 1
}
unset AWS_SIGNAL_ON_KEYRING_UPLOAD AWS_SIGNAL_NAME AWS_SIGNAL_MARKER
printf '%s\n' 'signal-during-virgin TERM keyring diagnostics passed'

rm -f "$mock_s3_root/cerulion-archive-keyring.gpg"
gpg --batch --export "$fingerprint_a" > "$incoming_keyring"
export AWS_FAIL_COPY=1
if virgin_upload_failure_output=$("$script_dir/publish_apt_repo.sh" "$repo" \
    s3://bucket/repo distribution-id "" "" "$fingerprint_a" 2>&1); then
    printf '%s\n' 'error: virgin keyring upload failure unexpectedly succeeded' >&2
    exit 1
fi
printf '%s\n' "$virgin_upload_failure_output" |
    grep -Fq 'APT keyring upload failed; the remote object may or may not exist' || {
    printf '%s\n' 'error: virgin keyring upload failure omitted uncertain-state wording' >&2
    printf '%s\n' "$virgin_upload_failure_output" >&2
    exit 1
}
printf '%s\n' "$virgin_upload_failure_output" |
    grep -Fq 'aws s3 ls s3://bucket/repo/cerulion-archive-keyring.gpg' || {
    printf '%s\n' 'error: virgin keyring upload failure omitted verification guidance' >&2
    printf '%s\n' "$virgin_upload_failure_output" >&2
    exit 1
}
if printf '%s\n' "$virgin_upload_failure_output" |
    grep -Fq 'orphaned APT keyring retained'; then
    printf '%s\n' 'error: failed virgin upload incorrectly claimed an orphaned keyring' >&2
    printf '%s\n' "$virgin_upload_failure_output" >&2
    exit 1
fi
unset AWS_FAIL_COPY
printf '%s\n' 'virgin keyring upload uncertainty diagnostic passed'

gpg --batch --export "$fingerprint_b" > "$incoming_keyring"
cat > "$workdir/run-signal-widening" <<EOF
#!/bin/sh
export AWS_SIGNAL_PID=\$\$
export AWS_SIGNAL_NAME=\${1:?}
exec "$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo distribution-id "" \
    "$published_keyring" "$fingerprint_a"
EOF
chmod 0755 "$workdir/run-signal-widening"
gpg --batch --export "$fingerprint_a" "$fingerprint_b" > "$incoming_keyring"
cp "$published_keyring" "$mock_s3_root/cerulion-archive-keyring.gpg"
printf '%s\n' old-inrelease > "$mock_s3_root/dists/stable/InRelease"
export AWS_SIGNAL_ON_KEYRING_UPLOAD=1
export AWS_SIGNAL_NAME=TERM
export AWS_SIGNAL_MARKER="$workdir/signal-widening-term"
export AWS_FAIL_KEYRING_RESTORE_INVALIDATION=1
rm -f "$AWS_SIGNAL_MARKER"
: > "$AWS_LOG"
if "$workdir/run-signal-widening" TERM \
    >"$workdir/signal-widening-output" 2>&1; then
    printf '%s\n' 'error: SIGTERM-during-widening publication unexpectedly succeeded' >&2
    exit 1
else
    signal_widening_status=$?
fi
[ "$signal_widening_status" -eq 143 ] || {
    printf 'error: SIGTERM-during-widening returned %s instead of 143\n' \
        "$signal_widening_status" >&2
    cat "$workdir/signal-widening-output" >&2
    exit 1
}
cmp "$mock_s3_root/cerulion-archive-keyring.gpg" "$published_keyring" || {
    printf '%s\n' 'error: SIGTERM-during-widening did not restore the old keyring' >&2
    exit 1
}
retained_keyring_backup=$(sed -n 's/^error: retained keyring backup: //p' \
    "$workdir/signal-widening-output")
if [ -z "$retained_keyring_backup" ] || [ ! -f "$retained_keyring_backup" ]; then
    printf '%s\n' 'error: SIGTERM-during-widening did not retain the keyring backup' >&2
    cat "$workdir/signal-widening-output" >&2
    exit 1
fi
grep -Fq "aws s3 cp $retained_keyring_backup s3://bucket/repo/cerulion-archive-keyring.gpg" \
    "$workdir/signal-widening-output" || {
    printf '%s\n' 'error: SIGTERM-during-widening omitted the keyring repair command' >&2
    cat "$workdir/signal-widening-output" >&2
    exit 1
}
unset AWS_SIGNAL_ON_KEYRING_UPLOAD AWS_SIGNAL_NAME AWS_SIGNAL_MARKER \
    AWS_FAIL_KEYRING_RESTORE_INVALIDATION
printf '%s\n' 'signal-during-widening TERM keyring rollback passed'

cp "$published_overlap_keyring" "$mock_s3_root/cerulion-archive-keyring.gpg"
printf '%s\n' old-inrelease > "$mock_s3_root/dists/stable/InRelease"
printf '%s\n' old-packages > "$mock_s3_root/dists/stable/main/binary-amd64/Packages"
printf '%s\n' old-packages-gzip > "$mock_s3_root/dists/stable/main/binary-amd64/Packages.gz"
export AWS_FAIL_METADATA_INVALIDATION_ALWAYS=1
export AWS_FAIL_METADATA_INVALIDATION_MARKER="$workdir/metadata-invalidation-always-failure"
: > "$AWS_FAIL_METADATA_INVALIDATION_MARKER"
: > "$AWS_LOG"
if metadata_invalidation_rollback_output=$("$script_dir/publish_apt_repo.sh" "$repo" \
    s3://bucket/repo distribution-id "" "$published_overlap_keyring" \
    "$fingerprint_b" 2>&1); then
    printf '%s\n' 'error: persistent metadata invalidation failure unexpectedly succeeded' >&2
    exit 1
fi
[ "$(cat "$mock_s3_root/dists/stable/InRelease")" = old-inrelease ] || {
    printf '%s\n' 'error: metadata invalidation failure did not restore InRelease' >&2
    exit 1
}
[ "$(cat "$mock_s3_root/dists/stable/main/binary-amd64/Packages")" = old-packages ] || {
    printf '%s\n' 'error: metadata invalidation failure did not restore Packages' >&2
    exit 1
}
[ "$(cat "$mock_s3_root/dists/stable/main/binary-amd64/Packages.gz")" = old-packages-gzip ] || {
    printf '%s\n' 'error: metadata invalidation failure did not restore Packages.gz' >&2
    exit 1
}
printf '%s\n' "$metadata_invalidation_rollback_output" |
    grep -q 'metadata restoration: succeeded' || {
    printf '%s\n' 'error: metadata invalidation failure omitted restoration status' >&2
    printf '%s\n' "$metadata_invalidation_rollback_output" >&2
    exit 1
}
printf '%s\n' "$metadata_invalidation_rollback_output" |
    grep -q 'CloudFront may still be serving the new APT metadata from cache' || {
    printf '%s\n' 'error: metadata invalidation failure omitted cache diagnostic' >&2
    printf '%s\n' "$metadata_invalidation_rollback_output" >&2
    exit 1
}
printf '%s\n' "$metadata_invalidation_rollback_output" |
    grep -Fq 'aws cloudfront create-invalidation --distribution-id distribution-id --paths "/dists/*"' || {
    printf '%s\n' 'error: metadata invalidation failure omitted the recovery invalidation command' >&2
    printf '%s\n' "$metadata_invalidation_rollback_output" >&2
    exit 1
}
unset AWS_FAIL_METADATA_INVALIDATION_ALWAYS AWS_FAIL_METADATA_INVALIDATION_MARKER
printf '%s\n' 'narrowing metadata invalidation failure restored metadata and emitted cache recovery diagnostics'

cp "$published_overlap_keyring" "$mock_s3_root/cerulion-archive-keyring.gpg"
printf '%s\n' old-inrelease > "$mock_s3_root/dists/stable/InRelease"
cat > "$workdir/run-signal-narrowing" <<EOF
#!/bin/sh
export AWS_SIGNAL_PID=\$\$
export AWS_SIGNAL_NAME=\${1:?}
exec "$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo distribution-id "" \
    "$published_overlap_keyring" "$fingerprint_b"
EOF
chmod 0755 "$workdir/run-signal-narrowing"
export AWS_SIGNAL_ON_KEYRING_UPLOAD=1
expected_overlap_fingerprints=$(gpg --batch --with-colons --show-keys \
    "$published_overlap_keyring" |
    awk -F: '$1 == "fpr" { print toupper($10) }' | sort)
for signal in HUP INT TERM; do
    gpg --batch --export "$fingerprint_b" > "$incoming_keyring"
    cp "$published_overlap_keyring" "$mock_s3_root/cerulion-archive-keyring.gpg"
    printf '%s\n' old-inrelease > "$mock_s3_root/dists/stable/InRelease"
    export AWS_SIGNAL_MARKER="$workdir/signal-keyring-$signal"
    : > "$AWS_LOG"
    signal_status=0
    if "$workdir/run-signal-narrowing" "$signal" \
        >"$workdir/signal-output-$signal" 2>&1; then
        printf 'error: SIG%s-during-narrowing publication unexpectedly succeeded\n' \
            "$signal" >&2
        exit 1
    else
        signal_status=$?
    fi
    [ "$signal_status" -ne 0 ] || {
        printf 'error: SIG%s-during-narrowing returned success\n' "$signal" >&2
        cat "$workdir/signal-output-$signal" >&2
        exit 1
    }
    [ "$(cat "$mock_s3_root/dists/stable/InRelease")" = old-inrelease ] || {
        printf 'error: SIG%s-during-narrowing changed restored InRelease\n' "$signal" >&2
        exit 1
    }
    restored_keyring_fingerprints=$(gpg --batch --with-colons --show-keys \
        "$mock_s3_root/cerulion-archive-keyring.gpg" |
        awk -F: '$1 == "fpr" { print toupper($10) }')
    if [ "$(printf '%s\n' "$restored_keyring_fingerprints" | sort)" != \
        "$expected_overlap_fingerprints" ]; then
        printf 'error: SIG%s-during-narrowing did not restore the complete overlap keyring\n' \
            "$signal" >&2
        printf '%s\n' "$restored_keyring_fingerprints" >&2
        exit 1
    fi
done
unset AWS_SIGNAL_ON_KEYRING_UPLOAD AWS_SIGNAL_MARKER
printf '%s\n' 'signal-during-narrowing HUP/INT/TERM keyring rollback passed'

gpg --batch --export "$fingerprint_b" > "$incoming_keyring"
cp "$published_overlap_keyring" "$mock_s3_root/cerulion-archive-keyring.gpg"
printf '%s\n' old-inrelease > "$mock_s3_root/dists/stable/InRelease"
export AWS_SIGNAL_ON_KEYRING_UPLOAD=1
export AWS_SIGNAL_NAME=INT
export AWS_SIGNAL_MARKER="$workdir/metadata-invalidation-signal"
export AWS_FAIL_METADATA_INVALIDATION=1
export AWS_FAIL_METADATA_INVALIDATION_MARKER="$workdir/metadata-invalidation-failure"
rm -f "$AWS_SIGNAL_MARKER" "$AWS_FAIL_METADATA_INVALIDATION_MARKER"
: > "$AWS_LOG"
if "$workdir/run-signal-narrowing" INT \
    >"$workdir/metadata-invalidation-output" 2>&1; then
    printf '%s\n' 'error: metadata invalidation failure unexpectedly succeeded' >&2
    exit 1
fi
metadata_invalidation_output=$(cat "$workdir/metadata-invalidation-output")
grep -q 'metadata restoration: succeeded' <<EOF
$metadata_invalidation_output
EOF
grep -q 'metadata invalidation: failed' <<EOF
$metadata_invalidation_output
EOF
grep -q 'keyring restoration: succeeded' <<EOF
$metadata_invalidation_output
EOF
if ! grep -q 'CloudFront may still be serving the new APT metadata from cache' <<EOF
$metadata_invalidation_output
EOF
then
    printf '%s\n' 'error: rollback invalidation failure omitted cache diagnostic' >&2
    exit 1
fi
if ! grep -Fq 'aws cloudfront create-invalidation --distribution-id distribution-id --paths "/dists/*"' <<EOF
$metadata_invalidation_output
EOF
then
    printf '%s\n' 'error: rollback invalidation failure omitted recovery command' >&2
    exit 1
fi
cmp "$mock_s3_root/cerulion-archive-keyring.gpg" "$published_overlap_keyring" || {
    printf '%s\n' 'error: metadata invalidation failure did not restore overlap keyring' >&2
    exit 1
}
unset AWS_SIGNAL_ON_KEYRING_UPLOAD AWS_SIGNAL_NAME AWS_SIGNAL_MARKER \
    AWS_FAIL_METADATA_INVALIDATION AWS_FAIL_METADATA_INVALIDATION_MARKER
printf '%s\n' 'narrowing metadata invalidation failure keyring rollback passed'

gpg --batch --export "$fingerprint_b" > "$incoming_keyring"
cp "$published_overlap_keyring" "$mock_s3_root/cerulion-archive-keyring.gpg"
printf '%s\n' old-pool-inrelease > "$mock_s3_root/dists/stable/InRelease"
printf '%s\n' new-pool-inrelease > "$repo/dists/stable/InRelease"
cp "$incoming_keyring" "$workdir/new-pool-keyring.gpg"
export AWS_FAIL_POOL_INVALIDATION_ALWAYS=1
export AWS_FAIL_POOL_INVALIDATION_MARKER="$workdir/pool-invalidation-failure"
: > "$AWS_LOG"
if pool_invalidation_output=$("$script_dir/publish_apt_repo.sh" "$repo" \
    s3://bucket/repo distribution-id "" "$published_overlap_keyring" \
    "$fingerprint_b" 2>&1); then
    printf '%s\n' 'error: persistent pool invalidation failure unexpectedly succeeded' >&2
    exit 1
fi
[ "$(cat "$mock_s3_root/dists/stable/InRelease")" = new-pool-inrelease ] || {
    printf '%s\n' 'error: pool invalidation failure rolled back new InRelease' >&2
    exit 1
}
cmp "$mock_s3_root/cerulion-archive-keyring.gpg" "$workdir/new-pool-keyring.gpg" || {
    printf '%s\n' 'error: pool invalidation failure rolled back narrowed keyring' >&2
    exit 1
}
[ "$(grep -cF \
    'cloudfront create-invalidation --distribution-id distribution-id --paths /pool/*' \
    "$AWS_LOG")" -eq 3 ] || {
    printf '%s\n' 'error: pool invalidation failure did not retry exactly three times' >&2
    cat "$AWS_LOG" >&2
    exit 1
}
printf '%s\n' "$pool_invalidation_output" |
    grep -q 'metadata and keyring publication succeeded; no rollback was attempted' || {
    printf '%s\n' 'error: pool invalidation failure omitted the no-rollback diagnostic' >&2
    printf '%s\n' "$pool_invalidation_output" >&2
    exit 1
}
printf '%s\n' "$pool_invalidation_output" |
    grep -q 'CloudFront may still serve pool objects whose bytes do not match the published index hashes' || {
    printf '%s\n' 'error: pool invalidation failure omitted the stale-pool warning' >&2
    printf '%s\n' "$pool_invalidation_output" >&2
    exit 1
}
printf '%s\n' "$pool_invalidation_output" |
    grep -Fq 'aws cloudfront create-invalidation --distribution-id distribution-id --paths "/pool/*"' || {
    printf '%s\n' 'error: pool invalidation failure omitted the recovery command' >&2
    printf '%s\n' "$pool_invalidation_output" >&2
    exit 1
}
unset AWS_FAIL_POOL_INVALIDATION_ALWAYS AWS_FAIL_POOL_INVALIDATION_MARKER
printf '%s\n' 'narrowing pool invalidation failure retained new metadata and keyring with cache recovery diagnostics'

gpg --batch --export "$fingerprint_b" > "$incoming_keyring"
export AWS_FAIL_KEYRING_CAPTURE=1
: > "$AWS_LOG"
capture_before_metadata_inrelease=$(cat "$mock_s3_root/dists/stable/InRelease")
if capture_failure_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "$published_overlap_keyring" "$fingerprint_b" 2>&1); then
    printf 'error: keyring capture failure unexpectedly succeeded\n' >&2
    exit 1
fi
printf '%s\n' "$capture_failure_output" |
    grep -q 'could not capture the currently published APT keyring' || {
    printf 'error: keyring capture failure had the wrong diagnostic\n' >&2
    printf '%s\n' "$capture_failure_output" >&2
    exit 1
}
[ "$(cat "$mock_s3_root/dists/stable/InRelease")" = \
    "$capture_before_metadata_inrelease" ] || {
    printf '%s\n' 'error: keyring capture failure published metadata' >&2
    exit 1
}
unset AWS_FAIL_KEYRING_CAPTURE
printf '%s\n' 'keyring capture precedes metadata publication passed'

export AWS_FAIL_COPY=1
: > "$AWS_LOG"
if copy_failure_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "$published_overlap_keyring" "$fingerprint_b" 2>&1); then
    printf '%s\n' 'error: mocked copy failure was swallowed by publication' >&2
    exit 1
fi
printf '%s\n' "$copy_failure_output" |
    grep -q 'could not capture the currently published APT keyring' || {
        printf '%s\n' 'error: mocked copy failure had the wrong publication diagnostic' >&2
        printf '%s\n' "$copy_failure_output" >&2
        exit 1
    }
unset AWS_FAIL_COPY
printf '%s\n' 'mocked copy failure propagation passed'

export AWS_FAIL_KEYRING_INVALIDATION=1
export AWS_FAIL_KEYRING_INVALIDATION_MARKER="$workdir/fail-keyring-invalidation"
rm -f "$AWS_FAIL_KEYRING_INVALIDATION_MARKER"
cp "$published_overlap_keyring" "$mock_s3_root/cerulion-archive-keyring.gpg"
: > "$AWS_LOG"
if keyring_invalidation_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "$published_overlap_keyring" "$fingerprint_b" 2>&1); then
    printf 'error: keyring invalidation failure unexpectedly succeeded\n' >&2
    exit 1
fi
printf '%s\n' "$keyring_invalidation_output" |
    grep -q 'could not publish the APT keyring' || {
    printf 'error: keyring invalidation failure had the wrong diagnostic\n' >&2
    printf '%s\n' "$keyring_invalidation_output" >&2
    exit 1
}
cmp "$mock_s3_root/cerulion-archive-keyring.gpg" "$published_overlap_keyring" || {
    printf '%s\n' 'error: keyring invalidation failure did not restore the old keyring' >&2
    exit 1
}
unset AWS_FAIL_KEYRING_INVALIDATION AWS_FAIL_KEYRING_INVALIDATION_MARKER
printf '%s\n' 'keyring publication rollback passed'

gpg --batch --export "$fingerprint_a" > "$published_overlap_keyring"
gpg --batch --export "$fingerprint_a" "$fingerprint_b" > "$incoming_keyring"
cp "$published_overlap_keyring" "$mock_s3_root/cerulion-archive-keyring.gpg"
export AWS_FAIL_KEYRING_INVALIDATION=1
export AWS_FAIL_KEYRING_INVALIDATION_MARKER="$workdir/fail-widening-keyring-invalidation"
rm -f "$AWS_FAIL_KEYRING_INVALIDATION_MARKER"
: > "$AWS_LOG"
if widening_keyring_invalidation_output=$("$script_dir/publish_apt_repo.sh" "$repo" \
    s3://bucket/repo distribution-id "" "$published_overlap_keyring" \
    "$fingerprint_a" 2>&1); then
    printf '%s\n' 'error: widening keyring invalidation failure unexpectedly succeeded' >&2
    exit 1
fi
printf '%s\n' "$widening_keyring_invalidation_output" |
    grep -q 'previous keyring restored after publication failure' || {
        printf '%s\n' 'error: widening keyring invalidation failure lacked rollback diagnostic' >&2
        printf '%s\n' "$widening_keyring_invalidation_output" >&2
        exit 1
    }
cmp "$mock_s3_root/cerulion-archive-keyring.gpg" "$published_overlap_keyring" || {
    printf '%s\n' 'error: widening keyring invalidation failure did not restore the old keyring' >&2
    exit 1
}
unset AWS_FAIL_KEYRING_INVALIDATION AWS_FAIL_KEYRING_INVALIDATION_MARKER
printf '%s\n' 'widening keyring invalidation rollback passed'

rm -rf "$mock_s3_root/dists"
mkdir -p "$mock_s3_root/dists"
export AWS_FAIL_INRELEASE=1
export AWS_FAIL_MARKER="$workdir/virgin-inrelease"
rm -f "$AWS_FAIL_MARKER"
: > "$AWS_LOG"
if virgin_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "" "$fingerprint_b" 2>&1); then
    printf 'error: virgin partial publication unexpectedly succeeded\n' >&2
    exit 1
fi
printf '%s\n' "$virgin_output" |
    grep -q 'no previous signed APT metadata generation existed' || {
    printf 'error: virgin partial publication had the wrong diagnostic\n' >&2
    printf '%s\n' "$virgin_output" >&2
    exit 1
}
printf '%s\n' "$virgin_output" |
    grep -q 'keyring restoration: not_needed' || {
    printf 'error: virgin publication rollback claimed a keyring restoration\n' >&2
    printf '%s\n' "$virgin_output" >&2
    exit 1
}
for uploaded_object in \
    'dists/*/by-hash/*' 'dists/stable/InRelease'; do
    printf '%s\n' "$virgin_output" | grep -Fq "$uploaded_object" || {
        printf 'error: virgin diagnostic omitted uploaded object %s\n' \
            "$uploaded_object" >&2
        printf '%s\n' "$virgin_output" >&2
        exit 1
    }
done
printf '%s\n' "$virgin_output" |
    grep -Fq 'uploaded metadata was invalidated at /dists/*' || {
    printf '%s\n' 'error: virgin partial publication omitted metadata invalidation diagnostic' >&2
    printf '%s\n' "$virgin_output" >&2
    exit 1
}
virgin_invalidation_count=$(grep -cF \
    'cloudfront create-invalidation --distribution-id distribution-id --paths /dists/*' \
    "$AWS_LOG")
[ "$virgin_invalidation_count" -ge 1 ] || {
    printf '%s\n' 'error: virgin partial publication did not invalidate metadata' >&2
    cat "$AWS_LOG" >&2
    exit 1
}
unset AWS_FAIL_INRELEASE AWS_FAIL_MARKER
printf '%s\n' 'virgin publication rollback diagnostic passed'

printf '%s\n' 'not-a-keyring' > "$workdir/unreadable-keyring"
if unreadable_output=$("$script_dir/publish_apt_repo.sh" "$repo" s3://bucket/repo \
    distribution-id "" "$workdir/unreadable-keyring" "$fingerprint_b" 2>&1); then
    printf 'error: unreadable published keyring was accepted\n' >&2
    exit 1
fi
printf '%s\n' "$unreadable_output" |
    grep -q 'could not inspect the currently published APT keyring' || {
        printf 'error: unreadable published keyring did not fail closed\n' >&2
        printf '%s\n' "$unreadable_output" >&2
        exit 1
    }
printf '%s\n' 'APT publication ordering passed (widening and narrowing)'
