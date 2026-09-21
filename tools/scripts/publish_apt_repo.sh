#!/bin/sh
#
# Publish a built APT repository with cache-safe keyring ordering.
#
set -eu
METADATA_INDEX_RESTORE_ATTEMPTS=3

usage() {
    printf 'usage: scripts/publish_apt_repo.sh REPO_DIR REMOTE DISTRIBUTION_ID PATH_PREFIX PUBLISHED_KEYRING PRIMARY_FINGERPRINT\n' >&2
    printf '       PUBLISHED_KEYRING may be empty for the first publication.\n' >&2
}

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

[ "$#" -eq 6 ] || {
    usage
    exit 2
}

repo=$1
remote=$2
distribution_id=$3
path_prefix=$4
published_keyring=$5
primary_fingerprint=$(printf '%s' "$6" |
    tr -d '[:space:]' | tr '[:lower:]' '[:upper:]')
incoming_keyring="$repo/cerulion-archive-keyring.gpg"

[ -f "$incoming_keyring" ] ||
    die "incoming APT keyring does not exist: $incoming_keyring"
[ -n "$distribution_id" ] ||
    die "CloudFront distribution ID is required for publication"
printf '%s' "$primary_fingerprint" | grep -Eq '^[[:xdigit:]]{40}$' ||
    die "primary InRelease signer must be a 40-hex-character fingerprint: $primary_fingerprint"

keyring_fingerprints() {
    fingerprint_output=$(mktemp "${TMPDIR:-/tmp}/cerulion-keyring-fingerprints.XXXXXX") ||
        return 1
    if ! gpg --batch --with-colons --show-keys "$1" > "$fingerprint_output" 2>/dev/null; then
        rm -f "$fingerprint_output"
        return 1
    fi
    awk -F: '$1 == "fpr" { print toupper($10) }' "$fingerprint_output" | sort -u
    status=$?
    rm -f "$fingerprint_output"
    return "$status"
}

incoming_fingerprints=$(keyring_fingerprints "$incoming_keyring") ||
    die "could not inspect the incoming APT keyring; refusing to guess publication order"
[ -n "$incoming_fingerprints" ] || {
    die "incoming APT keyring contains no fingerprints; refusing to guess publication order"
}
if ! printf '%s\n' "$incoming_fingerprints" |
    grep -Fqx "$primary_fingerprint"; then
    die "primary InRelease signer $primary_fingerprint is not present in the generated keyring; refusing to publish metadata no client can verify"
fi
published_fingerprints=

published_file=$(mktemp "${TMPDIR:-/tmp}/cerulion-published-fingerprints.XXXXXX") ||
    die "could not create published APT fingerprint list"
incoming_file=
if ! incoming_file=$(mktemp "${TMPDIR:-/tmp}/cerulion-incoming-fingerprints.XXXXXX"); then
    rm -f "$published_file"
    die "could not create incoming APT fingerprint list"
fi
metadata_backup_dir=
metadata_backup_inrelease=
metadata_backup_index_manifest=
metadata_backup_inrelease_present=0
metadata_backup_retained=0
metadata_restore_attempted=0
metadata_no_previous_inrelease=0
canonical_publish_attempted=0
signed_publish_attempted=0
keyring_backup=
keyring_backup_present=0
keyring_backup_retained=0
keyring_publish_attempted=0
published_metadata_objects=
metadata_invalidation_failed=0
cleanup() {
    status=$?
    rm -f "$published_file" "$incoming_file"
    if [ "$keyring_backup_retained" -eq 0 ] && [ -n "$keyring_backup" ]; then
        rm -f "$keyring_backup"
    fi
    if [ "$metadata_backup_retained" -eq 0 ] && [ -n "$metadata_backup_dir" ]; then
        rm -rf "$metadata_backup_dir"
    fi
    exit "$status"
}
trap cleanup EXIT
printf '%s\n' "$published_fingerprints" > "$published_file"
printf '%s\n' "$incoming_fingerprints" > "$incoming_file"

if [ -z "$published_keyring" ]; then
    publication_order=widening
else
    [ -f "$published_keyring" ] ||
        die "published APT keyring does not exist: $published_keyring"
    published_fingerprints=$(keyring_fingerprints "$published_keyring") ||
        die "could not inspect the currently published APT keyring; refusing to guess publication order"
    [ -n "$published_fingerprints" ] || {
        die "currently published APT keyring contains no fingerprints; refusing to guess publication order"
    }
    printf '%s\n' "$published_fingerprints" > "$published_file"
    if comm -23 "$published_file" "$incoming_file" | grep -q .; then
        publication_order=narrowing
    else
        publication_order=widening
    fi
    remote_location=${remote#s3://}
    remote_bucket=${remote_location%%/*}
    remote_prefix=${remote_location#"$remote_bucket"}
    remote_prefix=${remote_prefix#/}
    if [ "$remote_location" = "$remote" ] || [ -z "$remote_bucket" ]; then
        die "remote must be an S3 URI: $remote"
    fi
    metadata_key=
    if [ -n "$remote_prefix" ]; then
        metadata_key="$remote_prefix/dists/stable/InRelease"
    else
        metadata_key="dists/stable/InRelease"
    fi
    if metadata_status_output=$(aws s3api head-object --bucket "$remote_bucket" \
        --key "$metadata_key" 2>&1 >/dev/null); then
        published_metadata_present=1
    elif printf '%s\n' "$metadata_status_output" | grep -Eiq '404|not found|nosuchkey'; then
        published_metadata_present=0
    else
        die "could not determine whether signed APT metadata exists; refusing to guess publication order"
    fi
    if [ "$published_metadata_present" -eq 1 ] &&
        ! printf '%s\n' "$published_fingerprints" |
        grep -Fqx "$primary_fingerprint"; then
        die "primary InRelease signer $primary_fingerprint is not in the currently published APT keyring; direct replacement is refused; widen from A to A+B first with a currently trusted primary signer"
    fi
fi
printf 'APT keyring publication order: %s\n' "$publication_order"

cloudfront_prefix=${path_prefix#/}
cloudfront_prefix=${cloudfront_prefix%/}
if [ -n "$cloudfront_prefix" ]; then
    cloudfront_prefix="/$cloudfront_prefix"
fi

detached_metadata_guard() {
    remote_location=${remote#s3://}
    remote_bucket=${remote_location%%/*}
    remote_prefix=${remote_location#"$remote_bucket"}
    remote_prefix=${remote_prefix#/}
    if [ "$remote_location" = "$remote" ] || [ -z "$remote_bucket" ]; then
        die "remote must be an S3 URI: $remote"
    fi
    detached_found=
    detached_removals=
    for detached_name in Release Release.gpg; do
        if [ -n "$remote_prefix" ]; then
            detached_key="$remote_prefix/dists/stable/$detached_name"
        else
            detached_key="dists/stable/$detached_name"
        fi
        detached_uri="$remote/dists/stable/$detached_name"
        if detached_status_output=$(
            aws s3api head-object --bucket "$remote_bucket" --key "$detached_key" \
                2>&1 >/dev/null
        ); then
            detached_found="$detached_found
  $detached_uri"
            detached_removals="$detached_removals
  aws s3 rm $detached_uri"
        elif printf '%s\n' "$detached_status_output" |
            grep -Eiq '404|not found|nosuchkey'; then
            :
        else
            die "could not determine whether detached fallback metadata exists at $detached_uri: $detached_status_output"
        fi
    done
    [ -z "$detached_found" ] ||
        die "remote repository already serves detached fallback metadata at object keys:$detached_found
InRelease-only publication cannot keep detached fallback metadata coherent.
An operator with delete permission must remove these keys before publishing:
$detached_removals"
}

capture_metadata_backup() {
    metadata_backup_dir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-apt-metadata.XXXXXX") ||
        die "could not create APT metadata backup directory"
    metadata_backup_inrelease="$metadata_backup_dir/InRelease"
    metadata_backup_index_manifest="$metadata_backup_dir/index-manifest"
    : > "$metadata_backup_index_manifest"
    if aws s3 cp "$remote/dists/stable/InRelease" "$metadata_backup_inrelease" \
        >/dev/null 2>"$metadata_backup_dir/inrelease-error"; then
        metadata_backup_inrelease_present=1
    elif grep -Eq '404|Not Found' "$metadata_backup_dir/inrelease-error"; then
        metadata_no_previous_inrelease=1
    else
        die "could not capture the currently published APT InRelease metadata"
    fi

    find -L "$repo/dists" -type f -print > "$metadata_backup_dir/index-paths" ||
        die "could not enumerate APT metadata indexes"
    while IFS= read -r index_path; do
        index_rel=${index_path#"$repo/dists/"}
        case "$index_rel" in
        Release|Release.gpg|InRelease|*/Release|*/Release.gpg|*/InRelease|*/by-hash/*)
            continue
            ;;
        esac
        index_backup="$metadata_backup_dir/indexes/$index_rel"
        index_backup_parent=$(dirname "$index_backup")
        mkdir -p "$index_backup_parent" ||
            die "could not create APT metadata index backup directory"
        if aws s3 cp "$remote/dists/$index_rel" "$index_backup" \
            >/dev/null 2>"$metadata_backup_dir/index-error"; then
            printf '%s\n' "$index_rel" >> "$metadata_backup_index_manifest"
        elif grep -Eq '404|Not Found' "$metadata_backup_dir/index-error"; then
            :
        else
            die "could not capture the currently published APT metadata index: $index_rel"
        fi
    done < "$metadata_backup_dir/index-paths"
}

restore_metadata_backup() {
    restore_failed=0
    index_restore_failed=0
    missing_signed_metadata=0
    restored_indexes=
    failed_indexes=
    if [ "$metadata_backup_inrelease_present" -eq 0 ] &&
        [ ! -s "$metadata_backup_index_manifest" ]; then
        metadata_backup_retained=1
        metadata_restore_attempted=1
        printf 'error: no previous signed APT metadata generation existed (no previous InRelease); there is no state to restore\n' >&2
        printf 'error: uploaded objects retained in the remote repository:%s\n' \
            "${published_metadata_objects:- none}" >&2
        printf 'error: orphaned APT keyring retained in the remote repository: cerulion-archive-keyring.gpg\n' >&2
        printf 'error: remove every listed object if safe, then recreate the repository metadata and rerun publication; otherwise restore the repository manually before retrying\n' >&2
        return 1
    fi
    while IFS= read -r index_rel; do
        index_restored=0
        index_restore_attempt=1
        while [ "$index_restore_attempt" -le "$METADATA_INDEX_RESTORE_ATTEMPTS" ]; do
            metadata_restore_attempted=1
            if aws s3 cp "$metadata_backup_dir/indexes/$index_rel" \
                "$remote/dists/$index_rel"; then
                index_restored=1
                break
            fi
            index_restore_attempt=$((index_restore_attempt + 1))
        done
        if [ "$index_restored" -eq 1 ]; then
            restored_indexes="$restored_indexes $index_rel"
        else
            failed_indexes="$failed_indexes $index_rel"
            index_restore_failed=1
        fi
    done < "$metadata_backup_index_manifest"
    if [ "$index_restore_failed" -ne 0 ]; then
        metadata_backup_retained=1
        printf 'error: repository is in a mixed state: canonical index restoration failed after %s attempts\n' \
            "$METADATA_INDEX_RESTORE_ATTEMPTS" >&2
        printf 'error: signed metadata was deliberately left unchanged; do not overwrite it until all canonical indexes are restored\n' >&2
        printf 'error: canonical indexes restored:%s\n' "${restored_indexes:- none}" >&2
        printf 'error: canonical indexes still failed:%s\n' "${failed_indexes:- none}" >&2
        printf 'error: retained metadata backup directory: %s\n' "$metadata_backup_dir" >&2
        printf 'error: retry canonical index restoration with:\n' >&2
        printf '  while IFS= read -r index_rel; do\n' >&2
        printf "    aws s3 cp %s/indexes/\$index_rel %s/dists/\$index_rel\n" \
            "$metadata_backup_dir" "$remote" >&2
        printf '  done < %s\n' "$metadata_backup_index_manifest" >&2
        if [ "$metadata_backup_inrelease_present" -eq 1 ]; then
            printf '  aws s3 cp %s %s\n' "$metadata_backup_inrelease" \
                "$remote/dists/stable/InRelease" >&2
        fi
        return 1
    fi
    if [ "$metadata_backup_inrelease_present" -eq 0 ]; then
        missing_signed_metadata=1
    fi
    if [ "$metadata_backup_inrelease_present" -eq 1 ]; then
        metadata_restore_attempted=1
        inrelease_restore_attempt=1
        while [ "$inrelease_restore_attempt" -le "$METADATA_INDEX_RESTORE_ATTEMPTS" ]; do
            if aws s3 cp "$metadata_backup_inrelease" \
                "$remote/dists/stable/InRelease"; then
                break
            fi
            inrelease_restore_attempt=$((inrelease_restore_attempt + 1))
        done
        if [ "$inrelease_restore_attempt" -gt "$METADATA_INDEX_RESTORE_ATTEMPTS" ]; then
            restore_failed=1
            metadata_backup_retained=1
            printf 'error: signed metadata restoration failed after %s attempts\n' \
                "$METADATA_INDEX_RESTORE_ATTEMPTS" >&2
            printf 'error: the repository is left with signed metadata from the new generation against restored old indexes\n' >&2
            printf 'error: Acquire::By-Hash=false clients will fail apt-get update until the mismatch is repaired\n' >&2
            printf 'error: complete the repair with:\n' >&2
            printf '  aws s3 cp %s %s\n' "$metadata_backup_inrelease" \
                "$remote/dists/stable/InRelease" >&2
        fi
    fi
    if [ "$missing_signed_metadata" -ne 0 ]; then
        metadata_backup_retained=1
        printf 'error: cannot restore the previous InRelease because it was absent\n' >&2
        printf 'error: manually recreate and publish the previous InRelease, then rerun publication:\n' >&2
        if [ "$metadata_backup_inrelease_present" -eq 1 ]; then
            printf '  aws s3 cp %s %s\n' "$metadata_backup_inrelease" \
                "$remote/dists/stable/InRelease" >&2
        fi
        printf 'error: canonical index backups are in %s\n' "$metadata_backup_dir/indexes" >&2
        printf '  while IFS= read -r index_rel; do\n' >&2
        printf "    aws s3 cp %s/indexes/\$index_rel %s/dists/\$index_rel\n" \
            "$metadata_backup_dir" "$remote" >&2
        printf '  done < %s\n' "$metadata_backup_index_manifest" >&2
        return 1
    elif [ "$restore_failed" -ne 0 ]; then
        metadata_backup_retained=1
        printf 'error: failed to restore the previous APT metadata generation\n' >&2
        printf 'error: manually restore InRelease and canonical indexes, then rerun publication:\n' >&2
        if [ "$metadata_backup_inrelease_present" -eq 1 ]; then
            printf '  aws s3 cp %s %s\n' "$metadata_backup_inrelease" \
                "$remote/dists/stable/InRelease" >&2
        fi
        printf 'error: canonical index backups are in %s\n' "$metadata_backup_dir/indexes" >&2
        printf '  while IFS= read -r index_rel; do\n' >&2
        printf "    aws s3 cp %s/indexes/\$index_rel %s/dists/\$index_rel\n" \
            "$metadata_backup_dir" "$remote" >&2
        printf '  done < %s\n' "$metadata_backup_index_manifest" >&2
        return 1
    fi
    return 0
}

restore_signed_metadata_backup() {
    if [ "$metadata_backup_inrelease_present" -eq 0 ] &&
        [ ! -s "$metadata_backup_index_manifest" ]; then
        metadata_backup_retained=1
        metadata_restore_attempted=1
        printf 'error: no previous signed APT metadata generation existed (no previous InRelease); there is no state to restore\n' >&2
        printf 'error: uploaded objects retained in the remote repository:%s\n' \
            "${published_metadata_objects:- none}" >&2
        printf 'error: orphaned APT keyring retained in the remote repository: cerulion-archive-keyring.gpg\n' >&2
        printf 'error: remove every listed object if safe, then recreate the repository metadata and rerun publication; otherwise restore the repository manually before retrying\n' >&2
        return 1
    fi
    signed_restore_failed=0
    if [ "$metadata_backup_inrelease_present" -eq 1 ]; then
        metadata_restore_attempted=1
        inrelease_restore_attempt=1
        while [ "$inrelease_restore_attempt" -le "$METADATA_INDEX_RESTORE_ATTEMPTS" ]; do
            if aws s3 cp "$metadata_backup_inrelease" \
                "$remote/dists/stable/InRelease"; then
                break
            fi
            inrelease_restore_attempt=$((inrelease_restore_attempt + 1))
        done
        if [ "$inrelease_restore_attempt" -gt "$METADATA_INDEX_RESTORE_ATTEMPTS" ]; then
            signed_restore_failed=1
            metadata_backup_retained=1
            printf 'error: signed metadata restoration failed after %s attempts\n' \
                "$METADATA_INDEX_RESTORE_ATTEMPTS" >&2
            printf 'error: the repository is left with signed metadata from the new generation against restored old indexes\n' >&2
            printf 'error: Acquire::By-Hash=false clients will fail apt-get update until the mismatch is repaired\n' >&2
            printf 'error: complete the repair with:\n' >&2
            printf '  aws s3 cp %s %s\n' "$metadata_backup_inrelease" \
                "$remote/dists/stable/InRelease" >&2
        fi
    fi
    if [ "$signed_restore_failed" -ne 0 ]; then
        metadata_backup_retained=1
        printf 'error: failed to restore the previous signed APT metadata generation\n' >&2
        printf 'error: retained metadata backup directory: %s\n' \
            "$metadata_backup_dir" >&2
        return 1
    fi
    return 0
}

capture_keyring_backup() {
    keyring_backup=$(mktemp "${TMPDIR:-/tmp}/cerulion-apt-keyring.XXXXXX") ||
        return 1
    if ! aws s3 cp "$remote/cerulion-archive-keyring.gpg" \
        "$keyring_backup" >/dev/null 2>&1; then
        rm -f "$keyring_backup"
        keyring_backup=
        return 1
    fi
    keyring_backup_present=1
    return 0
}

restore_keyring_backup() {
    [ "$keyring_backup_present" -eq 1 ] || return 0
    aws s3 cp "$keyring_backup" "$remote/cerulion-archive-keyring.gpg" ||
        return 1
    if ! keyring_restore_invalidation_id=$(aws cloudfront create-invalidation \
        --distribution-id "$distribution_id" \
        --paths "${cloudfront_prefix}/cerulion-archive-keyring.gpg" \
        --query Invalidation.Id --output text); then
        return 1
    fi
    [ -n "$keyring_restore_invalidation_id" ] || return 1
    aws cloudfront wait invalidation-completed \
        --distribution-id "$distribution_id" \
        --id "$keyring_restore_invalidation_id"
}

metadata_cache_recovery_diagnostic() {
    printf 'error: CloudFront may still be serving the new APT metadata from cache\n' >&2
    printf 'error: after restoring the previous metadata generation, invalidate it with:\n' >&2
    printf '  aws cloudfront create-invalidation --distribution-id %s --paths "%s/dists/*"\n' \
        "$distribution_id" "$cloudfront_prefix" >&2
}

pool_cache_recovery_diagnostic() {
    printf 'error: CloudFront may still serve pool objects whose bytes do not match the published index hashes\n' >&2
    printf 'error: apt-get install can fail until the pool cache is invalidated; invalidate it with:\n' >&2
    printf '  aws cloudfront create-invalidation --distribution-id %s --paths "%s/pool/*"\n' \
        "$distribution_id" "$cloudfront_prefix" >&2
}

keyring_cache_recovery_diagnostic() {
    printf 'error: CloudFront may still serve a failed keyring publication from cache; invalidate it with:\n' >&2
    printf '  aws cloudfront create-invalidation --distribution-id %s --paths "%s/cerulion-archive-keyring.gpg"\n' \
        "$distribution_id" "$cloudfront_prefix" >&2
}

keyring_orphan_diagnostic() {
    printf 'error: orphaned APT keyring retained in the remote repository: %s\n' \
        "$remote/cerulion-archive-keyring.gpg" >&2
    keyring_cache_recovery_diagnostic
}

keyring_upload_uncertain_diagnostic() {
    printf 'error: APT keyring upload failed; the remote object may or may not exist: %s\n' \
        "$remote/cerulion-archive-keyring.gpg" >&2
    printf 'error: verify the remote keyring with:\n' >&2
    printf '  aws s3 ls %s\n' "$remote/cerulion-archive-keyring.gpg" >&2
    keyring_cache_recovery_diagnostic
}

keyring_repair_diagnostic() {
    keyring_backup_retained=1
    printf 'error: human repair may be required for cerulion-archive-keyring.gpg in S3 and CloudFront\n' >&2
    printf 'error: retained keyring backup: %s\n' "$keyring_backup" >&2
    printf 'error: restore the previous keyring with:\n' >&2
    printf '  aws s3 cp %s %s\n' "$keyring_backup" \
        "$remote/cerulion-archive-keyring.gpg" >&2
    keyring_cache_recovery_diagnostic
}

metadata_failure() {
    failure_message=$1
    failure_exit_status=${2:-1}
    metadata_restore_status=not_attempted
    metadata_invalidation_status=not_attempted
    keyring_restore_status=not_needed
    metadata_restore_attempted=0
    if [ "$canonical_publish_attempted" -eq 1 ]; then
        if restore_metadata_backup; then
            metadata_restore_status=succeeded
        else
            metadata_restore_status=failed
        fi
    elif [ "$signed_publish_attempted" -eq 1 ]; then
        if restore_signed_metadata_backup; then
            metadata_restore_status=succeeded
        else
            metadata_restore_status=failed
        fi
    fi
    if [ "$metadata_restore_attempted" -eq 1 ]; then
        metadata_backup_retained=1
        if invalidate_metadata; then
            metadata_invalidation_status=succeeded
            if [ "$metadata_restore_status" = succeeded ]; then
                metadata_backup_retained=0
            fi
        else
            metadata_invalidation_status=failed
        fi
    fi
    if [ "$keyring_backup_present" -eq 1 ] &&
        [ "$keyring_publish_attempted" -eq 1 ]; then
        if restore_keyring_backup; then
            keyring_restore_status=succeeded
        else
            keyring_restore_status=failed
        fi
    fi
    if [ "$metadata_restore_status" != succeeded ] ||
        [ "$metadata_invalidation_status" = failed ] ||
        [ "$keyring_restore_status" = failed ]; then
        printf 'error: %s\n' "$failure_message" >&2
        if [ "$metadata_no_previous_inrelease" -eq 1 ] &&
            [ "$metadata_invalidation_status" = succeeded ]; then
            printf 'error: no previous signed APT metadata generation existed (no previous InRelease); uploaded metadata was invalidated at /dists/*\n' >&2
        else
            printf 'error: metadata restoration: %s (dists/stable/InRelease and canonical indexes)\n' \
                "$metadata_restore_status" >&2
        fi
        printf 'error: metadata invalidation: %s (/dists/*)\n' \
            "$metadata_invalidation_status" >&2
        printf 'error: keyring restoration: %s (cerulion-archive-keyring.gpg)\n' \
            "$keyring_restore_status" >&2
        if [ "$metadata_restore_status" = failed ] ||
            [ "$metadata_invalidation_status" = failed ]; then
            printf 'error: human repair may be required for dists/stable/InRelease and dists/*; retained metadata backup: %s\n' \
                "$metadata_backup_dir" >&2
        fi
        if [ "$keyring_restore_status" = failed ]; then
            keyring_repair_diagnostic
        fi
        if [ "$keyring_publish_attempted" -eq 1 ] &&
            [ "$keyring_backup_present" -eq 0 ]; then
            keyring_orphan_diagnostic
        fi
        if [ "$metadata_invalidation_failed" -eq 1 ] ||
            [ "$metadata_invalidation_status" = failed ]; then
            metadata_cache_recovery_diagnostic
        fi
        exit "$failure_exit_status"
    fi
    if [ "$metadata_invalidation_failed" -eq 1 ]; then
        metadata_cache_recovery_diagnostic
    fi
    printf 'error: %s\n' "$failure_message" >&2
    exit "$failure_exit_status"
}

publication_signal_handler() {
    signal=$1
    trap '' HUP INT TERM
    case "$signal" in
        HUP) signal_status=129 ;;
        INT) signal_status=130 ;;
        TERM) signal_status=143 ;;
        *) signal_status=1 ;;
    esac
    if [ "$canonical_publish_attempted" -eq 1 ] ||
        [ "$signed_publish_attempted" -eq 1 ]; then
        metadata_failure "APT metadata publication interrupted by SIG$signal" \
            "$signal_status"
    fi
    if [ "$keyring_publish_attempted" -eq 1 ] &&
        [ "$keyring_backup_present" -eq 1 ]; then
        if restore_keyring_backup; then
            printf 'error: APT keyring publication interrupted by SIG%s; previous keyring restored\n' \
                "$signal" >&2
        else
            keyring_repair_diagnostic
        fi
    elif [ "$keyring_publish_attempted" -eq 1 ]; then
        keyring_orphan_diagnostic
    fi
    exit "$signal_status"
}

publish_metadata() {
    capture_metadata_backup
    # Upload immutable by-hash objects before signing metadata. They are
    # content-addressed and additive, so this cannot invalidate the current
    # generation. aws s3 sync follows symlinks by default; keep it explicit
    # because build_apt_repo.sh promotes dists through a local generation link.
    if ! aws s3 sync "$repo/dists/" "$remote/dists/" --follow-symlinks \
        --exclude '*' --exclude '*/Release' --exclude '*/Release.gpg' \
        --exclude '*/InRelease' --include '*/by-hash/*'
    then
        metadata_failure "could not publish APT metadata indexes"
    fi
    signed_publish_attempted=1
    published_metadata_objects=" dists/*/by-hash/*"
    if ! aws s3 cp "$repo/dists/stable/InRelease" "$remote/dists/stable/InRelease"; then
        metadata_failure "could not publish APT InRelease metadata"
    fi
    published_metadata_objects="$published_metadata_objects dists/stable/InRelease"
    # Canonical indexes are written last. A failure here can leave a legacy
    # client between generations, so retain the bounded rollback path.
    canonical_publish_attempted=1
    if ! aws s3 sync "$repo/dists/" "$remote/dists/" --follow-symlinks \
        --exclude '*/Release' --exclude '*/Release.gpg' \
        --exclude '*/InRelease' --exclude '*/by-hash/*'
    then
        metadata_failure "could not publish canonical APT metadata indexes"
    fi
    published_metadata_objects="$published_metadata_objects dists/*/binary-*/Packages dists/*/binary-*/Packages.gz"
}

invalidate_paths() {
    if ! invalidation_id=$(aws cloudfront create-invalidation \
        --distribution-id "$distribution_id" \
        --paths "$@" --query Invalidation.Id --output text); then
        printf '%s\n' 'error: could not create CloudFront invalidation' >&2
        return 1
    fi
    if [ -z "$invalidation_id" ]; then
        printf '%s\n' 'error: CloudFront invalidation returned no ID' >&2
        return 1
    fi
    if ! aws cloudfront wait invalidation-completed \
        --distribution-id "$distribution_id" \
        --id "$invalidation_id"; then
        printf '%s\n' 'error: CloudFront invalidation did not complete' >&2
        return 1
    fi
}

invalidate_metadata() {
    invalidate_paths "${cloudfront_prefix}/dists/*"
}

publish_keyring() {
    # Mark the replacement before starting the upload. A signal can arrive
    # after the remote copy completes but before this shell resumes, so
    # recording the state only after aws returns leaves a rollback gap.
    keyring_publish_attempted=1
    if ! aws s3 cp "$incoming_keyring" \
        "$remote/cerulion-archive-keyring.gpg"; then
        keyring_publish_attempted=0
        return 1
    fi
    if ! keyring_invalidation_id=$(aws cloudfront create-invalidation \
        --distribution-id "$distribution_id" \
        --paths "${cloudfront_prefix}/cerulion-archive-keyring.gpg" \
        --query Invalidation.Id --output text); then
        return 1
    fi
    [ -n "$keyring_invalidation_id" ] ||
        return 1
    if ! aws cloudfront wait invalidation-completed \
        --distribution-id "$distribution_id" \
        --id "$keyring_invalidation_id"; then
        return 1
    fi
}

trap 'publication_signal_handler HUP' HUP
trap 'publication_signal_handler INT' INT
trap 'publication_signal_handler TERM' TERM
detached_metadata_guard
if [ "$publication_order" = widening ]; then
    if [ -n "$published_keyring" ] && ! capture_keyring_backup; then
        die "could not capture the currently published APT keyring"
    fi
    if ! publish_keyring; then
        if [ "$keyring_backup_present" -eq 1 ]; then
            if restore_keyring_backup; then
                die "could not publish the APT keyring; previous keyring restored after publication failure"
            fi
            printf 'error: could not publish the APT keyring; keyring restoration failed\n' >&2
            keyring_repair_diagnostic
            exit 1
        fi
        if [ "$keyring_publish_attempted" -eq 1 ]; then
            keyring_orphan_diagnostic
        else
            keyring_upload_uncertain_diagnostic
        fi
        exit 1
    fi
    publish_metadata
    if ! invalidate_paths "${cloudfront_prefix}/dists/*" "${cloudfront_prefix}/pool/*"; then
        metadata_invalidation_failed=1
        pool_cache_recovery_diagnostic
        metadata_failure "could not invalidate APT metadata after publication"
    fi
else
    # Narrowing is the mirror image of widening: let signed metadata propagate
    # through CloudFront before exposing the narrowed keyring to clients.
    if ! capture_keyring_backup; then
        die "could not capture the currently published APT keyring"
    fi
    publish_metadata
    if ! invalidate_metadata; then
        metadata_invalidation_failed=1
        metadata_failure "could not invalidate APT metadata before keyring publication"
    fi
    if ! publish_keyring; then
        if ! restore_keyring_backup; then
            die "could not publish the APT keyring; keyring restoration failed"
        fi
        die "could not publish the APT keyring"
    fi
    pool_invalidation_attempt=1
    while [ "$pool_invalidation_attempt" -le "$METADATA_INDEX_RESTORE_ATTEMPTS" ]; do
        if invalidate_paths "${cloudfront_prefix}/pool/*"; then
            break
        fi
        pool_invalidation_attempt=$((pool_invalidation_attempt + 1))
    done
    if [ "$pool_invalidation_attempt" -gt "$METADATA_INDEX_RESTORE_ATTEMPTS" ]; then
        printf 'error: metadata and keyring publication succeeded; no rollback was attempted\n' >&2
        pool_cache_recovery_diagnostic
        exit 1
    fi
fi
