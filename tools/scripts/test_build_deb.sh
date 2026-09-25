#!/bin/sh

set -eu

# Most fixture archives below carry no rmw library; build_deb.sh refuses
# those by default. The refusal itself and the with-library package are
# exercised further down with the variable unset.
CERULION_DEB_ALLOW_MISSING_RMW=1
export CERULION_DEB_ALLOW_MISSING_RMW

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-build-deb-test.XXXXXX")
cleanup() {
    rm -rf "$workdir"
}
trap cleanup EXIT

archive_root="$workdir/cerulion-0.1.0-x86_64-unknown-linux-gnu"
mkdir -p "$archive_root"
ln -s /etc/hostname "$archive_root/cerulion"
for binary in cerulion-netd cerulion-connectd; do
    printf '%s\n' '#!/bin/sh' > "$archive_root/$binary"
done
printf '%s\n' 'license' > "$archive_root/LICENSE"
printf '%s\n' 'notice' > "$archive_root/NOTICE"
printf '%s\n' 'bsd' > "$archive_root/LICENSE-BSD-3-CLAUSE"
printf '%s\n' 'third-party' > "$archive_root/THIRD-PARTY-LICENSES.md"
archive="$workdir/cerulion-0.1.0-x86_64-unknown-linux-gnu.tar.gz"
tar -czf "$archive" -C "$workdir" \
    cerulion-0.1.0-x86_64-unknown-linux-gnu

if "$script_dir/build_deb.sh" "$archive" amd64 "$workdir/debs" \
    >"$workdir/output" 2>"$workdir/error"; then
    printf '%s\n' 'error: symlink archive entry was accepted' >&2
    exit 1
fi
grep -Eq 'non-regular archive entry.*cerulion' "$workdir/error" ||
    {
        cat "$workdir/error" >&2
        printf '%s\n' 'error: symlink rejection did not name cerulion' >&2
        exit 1
    }
printf '%s\n' 'build_deb archive rejection passed'

rm "$archive_root/cerulion"
for binary in cerulion cerulion-netd cerulion-connectd; do
    printf '%s\n' '#!/bin/sh' > "$archive_root/$binary"
done
printf '%s\n' 'license' > "$archive_root/LICENSE"
printf '%s\n' 'notice' > "$archive_root/NOTICE"
printf '%s\n' 'bsd' > "$archive_root/LICENSE-BSD-3-CLAUSE"
printf '%s\n' 'third-party' > "$archive_root/THIRD-PARTY-LICENSES.md"
tar -czf "$archive" -C "$workdir" \
    cerulion-0.1.0-x86_64-unknown-linux-gnu
valid_deb=$("$script_dir/build_deb.sh" "$archive" amd64 "$workdir/debs")
valid_hash=$(sha256sum "$valid_deb" | awk '{ print $1 }')
valid_contents=$(dpkg-deb --fsys-tarfile "$valid_deb" | tar -tvf -)
for binary in cerulion cerulion-netd cerulion-connectd; do
    printf '%s\n' "$valid_contents" |
        grep -Eq "^-rwxr-xr-x .* \./usr/bin/$binary$" || {
        printf 'error: Debian binary mode is not 0755: %s\n' "$binary" >&2
        exit 1
    }
done
printf '%s\n' "$valid_contents" |
    grep -Eq '^-rw-r--r-- .* \./usr/share/cerulion/install\.json$' || {
    printf '%s\n' 'error: Debian package lacks the 0644 install marker' >&2
    exit 1
}
[ "$(dpkg-deb --fsys-tarfile "$valid_deb" | tar -xOf - ./usr/share/cerulion/install.json)" = \
    '{"method":"deb","version":"0.1.0"}' ] || {
    printf '%s\n' 'error: Debian install marker has the wrong contents' >&2
    exit 1
}
printf '%s\n' "$valid_contents" |
    grep -Eq '^drwxr-xr-x .* \./usr/bin/$' || {
    printf '%s\n' 'error: Debian binary directory mode is not 0755' >&2
    exit 1
}
if "$script_dir/build_deb.sh" "$archive" amd64 "$workdir/debs" \
    >"$workdir/duplicate-output" 2>"$workdir/duplicate-error"; then
    printf '%s\n' 'error: existing Debian artifact was overwritten' >&2
    exit 1
fi
grep -Fq "output path already exists: $valid_deb" "$workdir/duplicate-error" || {
    cat "$workdir/duplicate-error" >&2
    printf '%s\n' 'error: existing Debian artifact had the wrong diagnostic' >&2
    exit 1
}
[ "$valid_hash" = "$(sha256sum "$valid_deb" | awk '{ print $1 }')" ] || {
    printf '%s\n' 'error: existing Debian artifact changed after rejected rebuild' >&2
    exit 1
}
race_bin="$workdir/race-bin"
race_debs="$workdir/race-debs"
mkdir "$race_bin" "$race_debs"
real_ln=$(command -v ln)
cat > "$race_bin/ln" <<'EOF'
#!/bin/sh
if [ "${LN_RACE_CREATE:-0}" -eq 1 ]; then
    printf '%s\n' 'pre-existing artifact' > "$LN_RACE_TARGET"
fi
exec "$REAL_LN" "$@"
EOF
chmod 0755 "$race_bin/ln"
race_deb="$race_debs/cerulion_0.1.0_amd64.deb"
if PATH="$race_bin:$PATH" REAL_LN="$real_ln" LN_RACE_CREATE=1 \
    LN_RACE_TARGET="$race_deb" "$script_dir/build_deb.sh" "$archive" amd64 \
    "$race_debs" >"$workdir/race-output" 2>"$workdir/race-error"; then
    printf '%s\n' 'error: concurrent Debian artifact creation was overwritten' >&2
    exit 1
fi
grep -Fq "output path already exists: $race_deb" "$workdir/race-error" || {
    cat "$workdir/race-error" >&2
    printf '%s\n' 'error: concurrent artifact had the wrong diagnostic' >&2
    exit 1
}
grep -Fq 'pre-existing artifact' "$race_deb" || {
    printf '%s\n' 'error: concurrent artifact bytes were overwritten' >&2
    exit 1
}
printf '%s\n' 'build_deb existing artifact protection passed'
directory_race_bin="$workdir/directory-race-bin"
directory_race_debs="$workdir/directory-race-debs"
mkdir "$directory_race_bin" "$directory_race_debs"
real_dpkg_deb=$(command -v dpkg-deb)
cat > "$directory_race_bin/dpkg-deb" <<'EOF'
#!/bin/sh
if [ "${DPKG_DEB_RACE_CREATE:-0}" -eq 1 ]; then
    mkdir "$DPKG_DEB_RACE_TARGET"
fi
exec "$REAL_DPKG_DEB" "$@"
EOF
chmod 0755 "$directory_race_bin/dpkg-deb"
directory_race_deb="$directory_race_debs/cerulion_0.1.0_amd64.deb"
if PATH="$directory_race_bin:$PATH" REAL_DPKG_DEB="$real_dpkg_deb" \
    DPKG_DEB_RACE_CREATE=1 DPKG_DEB_RACE_TARGET="$directory_race_deb" \
    "$script_dir/build_deb.sh" "$archive" amd64 "$directory_race_debs" \
    >"$workdir/directory-race-output" 2>"$workdir/directory-race-error"; then
    printf '%s\n' 'error: directory-valued concurrent artifact was accepted' >&2
    exit 1
fi
grep -Fq "output path already exists: $directory_race_deb" \
    "$workdir/directory-race-error" || {
    cat "$workdir/directory-race-error" >&2
    printf '%s\n' 'error: directory race had the wrong diagnostic' >&2
    exit 1
}
[ -d "$directory_race_deb" ] || {
    printf '%s\n' 'error: directory race destination disappeared' >&2
    exit 1
}
if find "$directory_race_deb" -mindepth 1 -print | grep -q .; then
    printf '%s\n' 'error: directory race populated the destination directory' >&2
    exit 1
fi
if find "$directory_race_debs" -maxdepth 1 -name '.cerulion.*' -print |
    grep -q .; then
    printf '%s\n' 'error: directory race left a temporary Debian package' >&2
    exit 1
fi
printf '%s\n' 'build_deb directory race protection passed'
printf '%s\n' "$valid_contents" |
    grep -Eq '^-rw-r--r-- .* \./usr/share/doc/cerulion/LICENSE$' || {
    printf '%s\n' 'error: Debian documentation mode is not 0644' >&2
    exit 1
}
# The attribution set, each asserted by name at 0644: an install loop that
# dropped one of them is not covered by its siblings. The LICENSE arm above is
# the mode pin. Dots in a name are escaped so the pattern cannot match a
# neighbour that differs only there.
for notice in NOTICE LICENSE-BSD-3-CLAUSE THIRD-PARTY-LICENSES.md; do
    notice_pattern=$(printf '%s' "$notice" | sed 's/\./\\./g')
    printf '%s\n' "$valid_contents" |
        grep -Eq "^-rw-r--r-- .* \./usr/share/doc/cerulion/$notice_pattern$" || {
        printf '%s\n' "error: Debian package is missing $notice" >&2
        exit 1
    }
done
if ! (umask 077
    "$script_dir/build_deb.sh" "$archive" amd64 "$workdir/restrictive-debs" \
        >"$workdir/restrictive-output" 2>"$workdir/restrictive-error"
); then
    cat "$workdir/restrictive-error" >&2
    printf '%s\n' 'error: build_deb failed under restrictive umask' >&2
    exit 1
fi
restrictive_deb="$workdir/restrictive-debs/cerulion_0.1.0_amd64.deb"
restrictive_contents=$(dpkg-deb --fsys-tarfile "$restrictive_deb" | tar -tvf -)
printf '%s\n' "$restrictive_contents" |
    grep -Eq '^-rwxr-xr-x .* \./usr/bin/cerulion$' || {
    printf '%s\n' 'error: restrictive umask removed executable mode' >&2
    exit 1
}
printf '%s\n' 'build_deb explicit modes passed'
# Each notice is required on its own. Every round removes exactly one and
# leaves the other three in place, so the refusal has to name the one that is
# gone: a require loop hardcoded to LICENSE passes the first round and fails
# the other three. The whole-line fixed-string match is what makes the LICENSE
# round distinguishable from LICENSE-BSD-3-CLAUSE, which contains it.
for notice in LICENSE NOTICE LICENSE-BSD-3-CLAUSE THIRD-PARTY-LICENSES.md; do
    rm "$archive_root/$notice"
    tar -czf "$archive" -C "$workdir" \
        cerulion-0.1.0-x86_64-unknown-linux-gnu
    if "$script_dir/build_deb.sh" "$archive" amd64 "$workdir/debs" \
        >"$workdir/rebuild-output" 2>"$workdir/rebuild-error"; then
        printf '%s\n' "error: rebuild archive without $notice was accepted" >&2
        exit 1
    fi
    grep -Fqx "error: archive is missing $notice" "$workdir/rebuild-error" || {
        printf '%s\n' "error: the refusal did not name the missing $notice" >&2
        cat "$workdir/rebuild-error" >&2
        exit 1
    }
    [ "$(sha256sum "$valid_deb" | awk '{ print $1 }')" = "$valid_hash" ] || {
        printf '%s\n' 'error: failed rebuild destroyed the existing Debian package' >&2
        exit 1
    }
    if find "$workdir/debs" -maxdepth 1 -name '.cerulion.*' -print |
        grep -q .; then
        printf '%s\n' 'error: failed rebuild left a temporary Debian package' >&2
        exit 1
    fi
    printf '%s\n' 'restored' > "$archive_root/$notice"
done
printf '%s\n' 'build_deb replacement preservation passed'

# The two shared objects. A Linux archive without the rmw, or with the rmw
# but without the heap hook, is refused unless the synthetic opt-out is set
# (the variable is exported at the top of this file, so the refusals are
# exercised with it removed from the environment); an archive that carries
# both produces them under /usr/bin at 0644 and declares the rmw's own
# link-time dependencies plus ROS 2 as a Suggests.
no_rmw_root="$workdir/cerulion-0.1.1-x86_64-unknown-linux-gnu"
mkdir -p "$no_rmw_root"
for binary in cerulion cerulion-netd cerulion-connectd; do
    printf '%s\n' '#!/bin/sh' > "$no_rmw_root/$binary"
done
printf '%s\n' 'license' > "$no_rmw_root/LICENSE"
printf '%s\n' 'notice' > "$no_rmw_root/NOTICE"
printf '%s\n' 'bsd' > "$no_rmw_root/LICENSE-BSD-3-CLAUSE"
printf '%s\n' 'third-party' > "$no_rmw_root/THIRD-PARTY-LICENSES.md"
no_rmw_archive="$workdir/cerulion-0.1.1-x86_64-unknown-linux-gnu.tar.gz"
tar -czf "$no_rmw_archive" -C "$workdir" \
    cerulion-0.1.1-x86_64-unknown-linux-gnu
if env -u CERULION_DEB_ALLOW_MISSING_RMW "$script_dir/build_deb.sh" \
    "$no_rmw_archive" amd64 "$workdir/no-rmw-debs" \
    >"$workdir/no-rmw-output" 2>"$workdir/no-rmw-error"; then
    printf '%s\n' 'error: archive without the rmw library was packaged' >&2
    exit 1
fi
grep -Fq 'archive is missing librmw_cerulion.so' "$workdir/no-rmw-error" || {
    cat "$workdir/no-rmw-error" >&2
    printf '%s\n' 'error: missing rmw library had the wrong diagnostic' >&2
    exit 1
}
if find "$workdir/no-rmw-debs" -type f -name '*.deb' -print -quit 2>/dev/null |
    grep -q .; then
    printf '%s\n' 'error: refused rmw-less archive left a Debian package behind' >&2
    exit 1
fi
printf '%s\n' 'build_deb rmw library requirement passed'

printf '%s\n' 'not a real shared object' > "$no_rmw_root/librmw_cerulion.so"
rmw_archive="$workdir/cerulion-0.1.1-x86_64-unknown-linux-gnu.tar.gz"
rm -f "$rmw_archive"
tar -czf "$rmw_archive" -C "$workdir" \
    cerulion-0.1.1-x86_64-unknown-linux-gnu
if env -u CERULION_DEB_ALLOW_MISSING_RMW "$script_dir/build_deb.sh" \
    "$rmw_archive" amd64 "$workdir/no-hook-debs" \
    >"$workdir/no-hook-output" 2>"$workdir/no-hook-error"; then
    printf '%s\n' 'error: archive with the rmw but without the heap hook was packaged' >&2
    exit 1
fi
grep -Fq 'archive is missing libcerulion_heaphook.so' "$workdir/no-hook-error" || {
    cat "$workdir/no-hook-error" >&2
    printf '%s\n' 'error: missing heap hook had the wrong diagnostic' >&2
    exit 1
}
printf '%s\n' 'build_deb heap hook requirement passed'

printf '%s\n' 'not a real hook either' > "$no_rmw_root/libcerulion_heaphook.so"
rm -f "$rmw_archive"
tar -czf "$rmw_archive" -C "$workdir" \
    cerulion-0.1.1-x86_64-unknown-linux-gnu
rmw_deb=$(env -u CERULION_DEB_ALLOW_MISSING_RMW "$script_dir/build_deb.sh" \
    "$rmw_archive" amd64 "$workdir/rmw-debs")
rmw_contents=$(dpkg-deb --fsys-tarfile "$rmw_deb" | tar -tvf -)
for library in librmw_cerulion.so libcerulion_heaphook.so; do
    printf '%s\n' "$rmw_contents" |
        grep -Eq "^-rw-r--r-- .* \./usr/bin/$library\$" || {
        printf '%s\n' "$rmw_contents" >&2
        printf 'error: %s is not installed beside the binaries at 0644\n' "$library" >&2
        exit 1
    }
done
rmw_depends=$(dpkg-deb -f "$rmw_deb" Depends)
for dependency in 'libc6 (>= 2.35)' libstdc++6 libgcc-s1; do
    case ", $rmw_depends, " in
        *", $dependency, "*) ;;
        *)
            printf 'error: Debian Depends lacks %s: %s\n' "$dependency" "$rmw_depends" >&2
            exit 1
            ;;
    esac
done
test "$(dpkg-deb -f "$rmw_deb" Suggests)" = ros-jazzy-ros-base || {
    printf '%s\n' 'error: Debian Suggests does not name ros-jazzy-ros-base' >&2
    exit 1
}
for library in librmw_cerulion.so libcerulion_heaphook.so; do
    dpkg-deb -f "$rmw_deb" Description | grep -Fq "$library" || {
        printf 'error: Debian description does not mention %s\n' "$library" >&2
        exit 1
    }
done
printf '%s\n' 'build_deb rmw library packaging passed'

# apt installs Recommends by default, so these four are how an apt user ends
# up with a C linker without typing anything, and the keyring entry must
# survive beside them.
rmw_recommends=$(dpkg-deb -f "$rmw_deb" Recommends)
for recommendation in cerulion-archive-keyring build-essential git curl; do
    case ", $rmw_recommends, " in
        *", $recommendation, "*) ;;
        *)
            printf 'error: Debian Recommends lacks %s: %s\n' \
                "$recommendation" "$rmw_recommends" >&2
            exit 1
            ;;
    esac
done
printf '%s\n' 'build_deb toolchain recommendations passed'

# The compiler bootstrap. The archives above carry neither half, so they are
# the control: no bootstrap files under /usr/share/cerulion (only the install
# marker), no wrapper.
if printf '%s\n' "$rmw_contents" | grep -Fq './usr/bin/cerulion-install-rust'; then
    printf '%s\n' 'error: bootstrap-less archive produced the Rust wrapper' >&2
    exit 1
fi
if printf '%s\n' "$rmw_contents" | grep -E '\./usr/share/cerulion/.+$' |
    grep -Evq '\./usr/share/cerulion/install\.json$'; then
    printf '%s\n' 'error: bootstrap-less archive produced Rust setup files' >&2
    exit 1
fi

bootstrap_root="$workdir/cerulion-0.1.2-x86_64-unknown-linux-gnu"
mkdir -p "$bootstrap_root"
for binary in cerulion cerulion-netd cerulion-connectd; do
    printf '%s\n' '#!/bin/sh' > "$bootstrap_root/$binary"
done
printf '%s\n' 'license' > "$bootstrap_root/LICENSE"
printf '%s\n' 'notice' > "$bootstrap_root/NOTICE"
printf '%s\n' 'bsd' > "$bootstrap_root/LICENSE-BSD-3-CLAUSE"
printf '%s\n' 'third-party' > "$bootstrap_root/THIRD-PARTY-LICENSES.md"
printf '%s\n' 'not a real shared object' > "$bootstrap_root/librmw_cerulion.so"
printf '%s\n' 'not a real hook either' > "$bootstrap_root/libcerulion_heaphook.so"
printf '%s\n' 'release: 1.93.0' > "$bootstrap_root/rustc-version.txt"
bootstrap_archive="$workdir/cerulion-0.1.2-x86_64-unknown-linux-gnu.tar.gz"
tar -czf "$bootstrap_archive" -C "$workdir" \
    cerulion-0.1.2-x86_64-unknown-linux-gnu
if env -u CERULION_DEB_ALLOW_MISSING_RMW "$script_dir/build_deb.sh" \
    "$bootstrap_archive" amd64 "$workdir/half-bootstrap-debs" \
    >"$workdir/half-bootstrap-output" 2>"$workdir/half-bootstrap-error"; then
    printf '%s\n' 'error: archive with half a Rust setup was packaged' >&2
    exit 1
fi
grep -Fq 'carries only part of its Rust setup' "$workdir/half-bootstrap-error" || {
    cat "$workdir/half-bootstrap-error" >&2
    printf '%s\n' 'error: half Rust setup had the wrong diagnostic' >&2
    exit 1
}

printf '%s\n' '#!/bin/sh' > "$bootstrap_root/install_rust.sh"
rm -f "$bootstrap_archive"
tar -czf "$bootstrap_archive" -C "$workdir" \
    cerulion-0.1.2-x86_64-unknown-linux-gnu
bootstrap_deb=$(env -u CERULION_DEB_ALLOW_MISSING_RMW "$script_dir/build_deb.sh" \
    "$bootstrap_archive" amd64 "$workdir/bootstrap-debs")
bootstrap_contents=$(dpkg-deb --fsys-tarfile "$bootstrap_deb" | tar -tvf -)
for payload in install_rust.sh rustc-version.txt; do
    printf '%s\n' "$bootstrap_contents" |
        grep -Eq "^-rw-r--r-- .* \./usr/share/cerulion/$payload\$" || {
        printf '%s\n' "$bootstrap_contents" >&2
        printf 'error: %s is not shipped read-only under /usr/share/cerulion\n' \
            "$payload" >&2
        exit 1
    }
done
printf '%s\n' "$bootstrap_contents" |
    grep -Eq '^-rwxr-xr-x .* \./usr/bin/cerulion-install-rust$' || {
    printf '%s\n' "$bootstrap_contents" >&2
    printf '%s\n' 'error: the Rust wrapper is not installed executable in /usr/bin' >&2
    exit 1
}
bootstrap_wrapper=$(dpkg-deb --fsys-tarfile "$bootstrap_deb" |
    tar -xOf - ./usr/bin/cerulion-install-rust)
printf '%s\n' "$bootstrap_wrapper" |
    grep -Fq '/usr/share/cerulion/install_rust.sh /usr/share/cerulion/rustc-version.txt' || {
    printf '%s\n' "$bootstrap_wrapper" >&2
    printf '%s\n' 'error: the Rust wrapper does not run the shipped pair' >&2
    exit 1
}
# The helper's failure arms say the Cerulion binaries were left alone, which
# describes the archive installer's transaction and not this one: the package
# put them in place already. The wrapper has to say so itself, which means it
# has to survive the helper's exit rather than exec into it.
printf '%s\n' "$bootstrap_wrapper" |
    grep -Fq 'The Cerulion programs are installed; only the compiler setup failed.' || {
    printf '%s\n' "$bootstrap_wrapper" >&2
    printf '%s\n' 'error: the Rust wrapper does not say what a failed setup left behind' >&2
    exit 1
}
# shellcheck disable=SC2016  # the assertion is the literal text of the wrapper
printf '%s\n' "$bootstrap_wrapper" | grep -Fxq 'exit "$status"' || {
    printf '%s\n' "$bootstrap_wrapper" >&2
    printf '%s\n' 'error: the Rust wrapper does not pass the helper status on' >&2
    exit 1
}
if printf '%s\n' "$bootstrap_wrapper" | grep -Eq '^exec '; then
    printf '%s\n' "$bootstrap_wrapper" >&2
    printf '%s\n' 'error: the Rust wrapper execs the helper, so it cannot report' >&2
    exit 1
fi
# Nothing provisions a compiler behind the user's back: rustup writes into one
# user's home, and a maintainer script runs as root at unpack time.
bootstrap_control=$(dpkg-deb --ctrl-tarfile "$bootstrap_deb" | tar -tf -)
for maintainer_script in preinst postinst prerm postrm; do
    if printf '%s\n' "$bootstrap_control" | grep -Eq "(^|/)$maintainer_script\$"; then
        printf 'error: package carries a %s maintainer script\n' \
            "$maintainer_script" >&2
        exit 1
    fi
done
printf '%s\n' 'build_deb Rust bootstrap packaging passed'

key_gnupg="$workdir/gnupg"
mkdir -m 0700 "$key_gnupg"
GNUPGHOME="$key_gnupg" gpg --batch --passphrase '' --quick-generate-key \
    'Cerulion build test <build-test@cerulion.com>' rsa1024 sign 1d >/dev/null 2>&1
key_fingerprint=$(GNUPGHOME="$key_gnupg" gpg --batch --with-colons \
    --list-secret-keys | awk -F: '$1 == "fpr" { print $10; exit }')
keyring="$workdir/keyring.gpg"
GNUPGHOME="$key_gnupg" gpg --batch --export "$key_fingerprint" > "$keyring"

invalid_keyring="$workdir/invalid-keyring.gpg"
printf '%s\n' 'not-a-keyring' > "$invalid_keyring"
if "$script_dir/build_keyring_deb.sh" "$invalid_keyring" 0.1.0 1 \
    "$workdir/keyring-debs" >"$workdir/keyring-output" 2>"$workdir/keyring-error"; then
    printf '%s\n' 'error: invalid keyring was accepted' >&2
    exit 1
fi
grep -Fq 'archive keyring is not a valid GPG keyring' "$workdir/keyring-error" || {
    cat "$workdir/keyring-error" >&2
    printf '%s\n' 'error: invalid keyring rejection had the wrong diagnostic' >&2
    exit 1
}
printf '%s\n' 'build_keyring_deb validation passed'

public_keyring_deb=$("$script_dir/build_keyring_deb.sh" "$keyring" 0.1.0 1 \
    "$workdir/public-keyring-debs")
test -f "$public_keyring_deb"
printf '%s\n' 'build_keyring_deb public-only keyring passed'

REAL_MKTEMP=$(command -v mktemp)
mktemp_wrapper="$workdir/mktemp"
cat > "$mktemp_wrapper" <<'EOF'
#!/bin/sh
count_file=$MKtemp_COUNT_FILE
count=0
if [ -f "$count_file" ]; then
    count=$(cat "$count_file")
fi
count=$((count + 1))
printf '%s\n' "$count" > "$count_file"
if [ "$count" -eq 2 ]; then
    exit 1
fi
exec "$REAL_MKTEMP" "$@"
EOF
chmod 0755 "$mktemp_wrapper"
mktemp_count="$workdir/mktemp-count"
if TMPDIR="$workdir" PATH="$workdir:$PATH" MKtemp_COUNT_FILE="$mktemp_count" \
    REAL_MKTEMP="$REAL_MKTEMP" \
    "$script_dir/build_keyring_deb.sh" "$keyring" 0.1.0 1 \
    "$workdir/second-mktemp-debs" >"$workdir/second-mktemp-output" \
    2>"$workdir/second-mktemp-error"; then
    printf '%s\n' 'error: second keyring temporary allocation failure was accepted' >&2
    exit 1
fi
if find "$workdir" -maxdepth 1 -name 'cerulion-keyring-secret-check.*' -print |
    grep -q .; then
    printf '%s\n' 'error: failed keyring validation left a temporary GPG home' >&2
    exit 1
fi
printf '%s\n' 'build_keyring_deb second temporary allocation cleanup passed'

secret_keyring="$workdir/secret-keyring.gpg"
GNUPGHOME="$key_gnupg" gpg --batch --export-secret-keys "$key_fingerprint" > "$secret_keyring"
if "$script_dir/build_keyring_deb.sh" "$secret_keyring" 0.1.0 1 \
    "$workdir/secret-keyring-debs" >"$workdir/secret-output" 2>"$workdir/secret-error"; then
    printf '%s\n' 'error: secret-bearing keyring was accepted' >&2
    exit 1
fi
grep -Fq 'archive keyring contains private key material' "$workdir/secret-error" || {
    cat "$workdir/secret-error" >&2
    printf '%s\n' 'error: private-key rejection had the wrong diagnostic' >&2
    exit 1
}
printf '%s\n' 'build_keyring_deb private-key rejection passed'

repro_debs_a="$workdir/repro-debs-a"
repro_debs_b="$workdir/repro-debs-b"
SOURCE_DATE_EPOCH=1700000000 "$script_dir/build_keyring_deb.sh" "$keyring" 0.1.0 3 \
    "$repro_debs_a" >/dev/null
SOURCE_DATE_EPOCH=1700000000 "$script_dir/build_keyring_deb.sh" "$keyring" 0.1.0 3 \
    "$repro_debs_b" >/dev/null
cmp "$repro_debs_a/cerulion-archive-keyring_0.1.0-3_all.deb" \
    "$repro_debs_b/cerulion-archive-keyring_0.1.0-3_all.deb"
printf '%s\n' 'build_keyring_deb reproducibility passed'

for invalid_revision in 00 01; do
    if "$script_dir/build_keyring_deb.sh" "$keyring" 0.1.0 \
        "$invalid_revision" "$workdir/keyring-debs" \
        >"$workdir/revision-output" 2>"$workdir/revision-error"; then
        printf 'error: non-canonical revision accepted: %s\n' "$invalid_revision" >&2
        exit 1
    fi
    grep -Fq 'canonical non-negative integer' "$workdir/revision-error" || {
        cat "$workdir/revision-error" >&2
        printf '%s\n' 'error: invalid revision had the wrong diagnostic' >&2
        exit 1
    }
done
printf '%s\n' 'build_keyring_deb revision validation passed'

newline_version=$(printf '1.0.0\nDepends: evil')
if "$script_dir/build_keyring_deb.sh" "$keyring" "$newline_version" 1 \
    "$workdir/keyring-debs" >"$workdir/version-output" 2>"$workdir/version-error"; then
    printf '%s\n' 'error: newline-injection version was accepted' >&2
    exit 1
fi
grep -Fq 'invalid keyring package version' "$workdir/version-error" || {
    cat "$workdir/version-error" >&2
    printf '%s\n' 'error: invalid version had the wrong diagnostic' >&2
    exit 1
}
printf '%s\n' 'build_keyring_deb version validation passed'

for accepted_version in 0.1.0~rc.1 0.1.0+build-1 0.1.0+build.5 \
    0.1.0+build-foo 1.2.3-1; do
    accepted_deb=$("$script_dir/build_keyring_deb.sh" "$keyring" \
        "$accepted_version" 3 "$workdir/keyring-debs")
    test -f "$accepted_deb"
    test "$(dpkg-deb -f "$accepted_deb" Version)" = "$accepted_version-3"
done
printf '%s\n' 'build_keyring_deb Debian prerelease versions passed'

if "$script_dir/build_keyring_deb.sh" "$keyring" 0.1.0-rc.1 3 \
    "$workdir/keyring-debs" >"$workdir/unnormalized-output" \
    2>"$workdir/unnormalized-error"; then
    printf '%s\n' 'error: unnormalized keyring package version was accepted' >&2
    exit 1
fi
grep -Fq 'not Debian-normalized' "$workdir/unnormalized-error" || {
    cat "$workdir/unnormalized-error" >&2
    printf '%s\n' 'error: unnormalized version had the wrong diagnostic' >&2
    exit 1
}
printf '%s\n' 'build_keyring_deb normalized-version validation passed'

control_char_version=$(printf '0.1.0\tDepends: evil')
if "$script_dir/build_keyring_deb.sh" "$keyring" "$control_char_version" 1 \
    "$workdir/keyring-debs" >"$workdir/control-output" 2>"$workdir/control-error"; then
    printf '%s\n' 'error: control-character version was accepted' >&2
    exit 1
fi
grep -Fq 'invalid keyring package version' "$workdir/control-error" || {
    cat "$workdir/control-error" >&2
    printf '%s\n' 'error: control-character version had the wrong diagnostic' >&2
    exit 1
}
printf '%s\n' 'build_keyring_deb control-character validation passed'

for mapping in \
    '0.1.0|0.1.0' \
    '0.1.0-rc.1|0.1.0~rc.1' \
    '0.1.0+build.5|0.1.0+build.5' \
    '0.1.0-rc.1+build.5|0.1.0~rc.1+build.5'
do
    version=${mapping%%|*}
    expected=${mapping#*|}
    mapped_version=$("$script_dir/debian_version.sh" "$version")
    test "$mapped_version" = "$expected"
    mapping_root="$workdir/cerulion-$version-x86_64-unknown-linux-gnu"
    mkdir -p "$mapping_root"
    for binary in cerulion cerulion-netd cerulion-connectd; do
        printf '%s\n' '#!/bin/sh' > "$mapping_root/$binary"
    done
    printf '%s\n' 'license' > "$mapping_root/LICENSE"
    printf '%s\n' 'notice' > "$mapping_root/NOTICE"
    printf '%s\n' 'bsd' > "$mapping_root/LICENSE-BSD-3-CLAUSE"
    printf '%s\n' 'third-party' > "$mapping_root/THIRD-PARTY-LICENSES.md"
    mapping_archive="$workdir/cerulion-$version-x86_64-unknown-linux-gnu.tar.gz"
    tar -czf "$mapping_archive" -C "$workdir" \
        "cerulion-$version-x86_64-unknown-linux-gnu"
    mapping_deb=$("$script_dir/build_deb.sh" "$mapping_archive" amd64 \
        "$workdir/mapping-debs")
    test "$(basename "$mapping_deb")" = "cerulion_${mapped_version}_amd64.deb"
    test "$(dpkg-deb -f "$mapping_deb" Version)" = "$mapped_version"
done
printf '%s\n' 'debian version mapping passed'

for invalid_version in 1+ 1- 1.2.3- 1.2.3+ 1.2.3-rc. \
    1.2.3+build. 1.2.3+build- 1.2.3+-build 1.2.3+build-a-b \
    1.2.3-rc..1 1.2.3+build..5 1.2.3-rc-1 \
    1.2.3. 1.2.3..4; do
    if "$script_dir/debian_version.sh" "$invalid_version" \
        >"$workdir/invalid-version-output" 2>"$workdir/invalid-version-error"; then
        printf 'error: malformed version was accepted: %s\n' "$invalid_version" >&2
        exit 1
    fi
    if [ "$invalid_version" = 1.2.3-rc-1 ]; then
        grep -Fq "prerelease identifiers containing '-' are not supported" \
            "$workdir/invalid-version-error" || {
            printf '%s\n' 'error: hyphenated prerelease had the wrong diagnostic' >&2
            cat "$workdir/invalid-version-error" >&2
            exit 1
        }
    fi
    if [ "$invalid_version" = 1.2.3+build-a-b ]; then
        grep -Fq "build metadata identifiers containing '-' are not supported" \
            "$workdir/invalid-version-error" || {
            printf '%s\n' 'error: hyphenated build metadata had the wrong diagnostic' >&2
            cat "$workdir/invalid-version-error" >&2
            exit 1
        }
    fi
done
printf '%s\n' 'strict Debian version validation passed'

dotted_build_output=$("$script_dir/debian_version.sh" 0.1.0+build.a.b)
if "$script_dir/debian_version.sh" 0.1.0+build-a-b \
    >"$workdir/hyphenated-build-output" 2>"$workdir/hyphenated-build-error"; then
    printf '%s\n' 'error: colliding hyphenated build metadata was accepted' >&2
    exit 1
fi
test "$dotted_build_output" = '0.1.0+build.a.b'
grep -Fq "build metadata identifiers containing '-' are not supported" \
    "$workdir/hyphenated-build-error" || {
    cat "$workdir/hyphenated-build-error" >&2
    printf '%s\n' 'error: colliding build metadata had the wrong diagnostic' >&2
    exit 1
}
printf '%s\n' 'debian build metadata collision rejection passed'

for invalid_version in "$(printf '0.1.0\nrc')" "$(printf '0.1.0\trc')"; do
    if "$script_dir/debian_version.sh" "$invalid_version" \
        >"$workdir/debian-version-output" 2>"$workdir/debian-version-error"; then
        printf '%s\n' 'error: control-character version was accepted by debian version helper' >&2
        exit 1
    fi
    grep -Fq 'version contains a control character' \
        "$workdir/debian-version-error" || {
        cat "$workdir/debian-version-error" >&2
        printf '%s\n' 'error: helper control-character rejection had the wrong diagnostic' >&2
        exit 1
    }
done
printf '%s\n' 'debian version control-character rejection passed'

prerelease_root="$workdir/cerulion-0.1.0-rc.1-x86_64-unknown-linux-gnu"
mkdir -p "$prerelease_root"
for binary in cerulion cerulion-netd cerulion-connectd; do
    printf '%s\n' '#!/bin/sh' > "$prerelease_root/$binary"
done
printf '%s\n' 'license' > "$prerelease_root/LICENSE"
printf '%s\n' 'notice' > "$prerelease_root/NOTICE"
printf '%s\n' 'bsd' > "$prerelease_root/LICENSE-BSD-3-CLAUSE"
printf '%s\n' 'third-party' > "$prerelease_root/THIRD-PARTY-LICENSES.md"
prerelease_archive="$workdir/cerulion-0.1.0-rc.1-x86_64-unknown-linux-gnu.tar.gz"
tar -czf "$prerelease_archive" -C "$workdir" \
    cerulion-0.1.0-rc.1-x86_64-unknown-linux-gnu
prerelease_deb=$("$script_dir/build_deb.sh" "$prerelease_archive" amd64 \
    "$workdir/prerelease-debs")
test "$(dpkg-deb -f "$prerelease_deb" Version)" = '0.1.0~rc.1'
plus_root="$workdir/cerulion-0.1.0+build.1-x86_64-unknown-linux-gnu"
mkdir -p "$plus_root"
for binary in cerulion cerulion-netd cerulion-connectd; do
    printf '%s\n' '#!/bin/sh' > "$plus_root/$binary"
done
printf '%s\n' 'license' > "$plus_root/LICENSE"
printf '%s\n' 'notice' > "$plus_root/NOTICE"
printf '%s\n' 'bsd' > "$plus_root/LICENSE-BSD-3-CLAUSE"
printf '%s\n' 'third-party' > "$plus_root/THIRD-PARTY-LICENSES.md"
plus_archive="$workdir/cerulion-0.1.0+build.1-x86_64-unknown-linux-gnu.tar.gz"
tar -czf "$plus_archive" -C "$workdir" \
    cerulion-0.1.0+build.1-x86_64-unknown-linux-gnu
plus_deb=$("$script_dir/build_deb.sh" "$plus_archive" amd64 "$workdir/plus-debs")
test "$(basename "$plus_deb")" = 'cerulion_0.1.0+build.1_amd64.deb'
test "$(dpkg-deb -f "$plus_deb" Version)" = '0.1.0+build.1'
plus_repo="$workdir/plus-repo"
GNUPGHOME="$key_gnupg" APT_KEYRING_KEYS="$key_fingerprint" APT_KEYRING_REVISION=1 \
    "$script_dir/build_apt_repo.sh" "$plus_repo" "$key_fingerprint" \
    "$plus_deb" >/dev/null
plus_keyring_deb=$(find "$plus_repo/pool/main/c/cerulion-archive-keyring" \
    -type f -name '*.deb' -print -quit)
test -n "$plus_keyring_deb"
test "$(dpkg-deb -f "$plus_keyring_deb" Version)" = \
    "$(dpkg-deb -f "$plus_deb" Version)-1"
printf '%s\n' 'build_apt_repo app/keyring version parity passed'

raw_version_root="$workdir/raw-version-root"
mkdir -p "$raw_version_root/DEBIAN" "$raw_version_root/usr/bin"
cat > "$raw_version_root/DEBIAN/control" <<'EOF'
Package: cerulion
Version: 1.2.3-1
Architecture: amd64
Description: raw version fixture
EOF
printf '%s\n' '#!/bin/sh' > "$raw_version_root/usr/bin/cerulion"
chmod 0755 "$raw_version_root/usr/bin/cerulion"
raw_amd64_deb="$workdir/raw-version-amd64.deb"
dpkg-deb --root-owner-group --build "$raw_version_root" "$raw_amd64_deb" >/dev/null
sed 's/Architecture: amd64/Architecture: arm64/' \
    "$raw_version_root/DEBIAN/control" > "$raw_version_root/DEBIAN/control.tmp"
mv "$raw_version_root/DEBIAN/control.tmp" "$raw_version_root/DEBIAN/control"
raw_arm64_deb="$workdir/raw-version-arm64.deb"
dpkg-deb --root-owner-group --build "$raw_version_root" "$raw_arm64_deb" >/dev/null
raw_repo="$workdir/raw-version-repo"
GNUPGHOME="$key_gnupg" APT_KEYRING_KEYS="$key_fingerprint" APT_KEYRING_REVISION=1 \
    "$script_dir/build_apt_repo.sh" "$raw_repo" "$key_fingerprint" \
    "$raw_amd64_deb" "$raw_arm64_deb" >/dev/null
raw_keyring_deb=$(find "$raw_repo/pool/main/c/cerulion-archive-keyring" \
    -type f -name '*.deb' -print -quit)
test -n "$raw_keyring_deb"
test "$(dpkg-deb -f "$raw_keyring_deb" Version)" = '1.2.3-1-1'
printf '%s\n' 'build_apt_repo raw multi-architecture version equality passed'

hash_failure_bin="$workdir/hash-failure-bin"
mkdir -p "$hash_failure_bin"
cat > "$hash_failure_bin/sha256sum" <<'EOF'
#!/bin/sh
exit 1
EOF
chmod 0755 "$hash_failure_bin/sha256sum"
if PATH="$hash_failure_bin:$PATH" GNUPGHOME="$key_gnupg" \
    APT_KEYRING_KEYS="$key_fingerprint" APT_KEYRING_REVISION=1 \
    "$script_dir/build_apt_repo.sh" "$workdir/hash-failure-repo" \
    "$key_fingerprint" "$plus_deb" >"$workdir/hash-failure-output" \
    2>"$workdir/hash-failure-error"; then
    printf '%s\n' 'error: sha256sum failure was accepted' >&2
    exit 1
fi
grep -Fq 'could not calculate a valid SHA-256 hash' "$workdir/hash-failure-error" || {
    cat "$workdir/hash-failure-error" >&2
    printf '%s\n' 'error: sha256sum failure had the wrong diagnostic' >&2
    exit 1
}
cat > "$hash_failure_bin/sha256sum" <<'EOF'
#!/bin/sh
printf '%s\n' malformed-output
EOF
if PATH="$hash_failure_bin:$PATH" GNUPGHOME="$key_gnupg" \
    APT_KEYRING_KEYS="$key_fingerprint" APT_KEYRING_REVISION=1 \
    "$script_dir/build_apt_repo.sh" "$workdir/hash-malformed-repo" \
    "$key_fingerprint" "$plus_deb" >"$workdir/hash-malformed-output" \
    2>"$workdir/hash-malformed-error"; then
    printf '%s\n' 'error: malformed sha256sum output was accepted' >&2
    exit 1
fi
grep -Fq 'could not calculate a valid SHA-256 hash' \
    "$workdir/hash-malformed-error" || {
    cat "$workdir/hash-malformed-error" >&2
    printf '%s\n' 'error: malformed hash output had the wrong diagnostic' >&2
    exit 1
}
printf '%s\n' 'build_apt_repo SHA-256 failure handling passed'

prerelease_plus_root="$workdir/cerulion-0.1.0-rc.1+build.1-x86_64-unknown-linux-gnu"
mkdir -p "$prerelease_plus_root"
for binary in cerulion cerulion-netd cerulion-connectd; do
    printf '%s\n' '#!/bin/sh' > "$prerelease_plus_root/$binary"
done
printf '%s\n' 'license' > "$prerelease_plus_root/LICENSE"
printf '%s\n' 'notice' > "$prerelease_plus_root/NOTICE"
printf '%s\n' 'bsd' > "$prerelease_plus_root/LICENSE-BSD-3-CLAUSE"
printf '%s\n' 'third-party' > "$prerelease_plus_root/THIRD-PARTY-LICENSES.md"
prerelease_plus_archive="$workdir/cerulion-0.1.0-rc.1+build.1-x86_64-unknown-linux-gnu.tar.gz"
tar -czf "$prerelease_plus_archive" -C "$workdir" \
    cerulion-0.1.0-rc.1+build.1-x86_64-unknown-linux-gnu
prerelease_plus_deb=$("$script_dir/build_deb.sh" "$prerelease_plus_archive" amd64 \
    "$workdir/prerelease-plus-debs")
test "$(basename "$prerelease_plus_deb")" = 'cerulion_0.1.0~rc.1+build.1_amd64.deb'
test "$(dpkg-deb -f "$prerelease_plus_deb" Version)" = '0.1.0~rc.1+build.1'
prerelease_repo="$workdir/prerelease-repo"
GNUPGHOME="$key_gnupg" APT_KEYRING_KEYS="$key_fingerprint" APT_KEYRING_REVISION=1 \
    "$script_dir/build_apt_repo.sh" "$prerelease_repo" "$key_fingerprint" \
    "$prerelease_deb" >/dev/null
test -f "$prerelease_repo/pool/main/c/cerulion-archive-keyring/"*.deb
printf '%s\n' 'build_apt_repo prerelease caller path passed'

relative_keyring_dir="$workdir/relative-keyring"
mkdir -p "$relative_keyring_dir"
(
    cd "$workdir"
    "$script_dir/build_keyring_deb.sh" "$keyring" 0.1.0 1 relative-keyring
)
test -f "$relative_keyring_dir/cerulion-archive-keyring_0.1.0-1_all.deb"
printf '%s\n' 'build_keyring_deb relative output directory passed'

relative_tmpdir="$workdir/relative-tmpdir"
relative_tmp_output="$workdir/relative-tmp-output"
mkdir -p "$relative_tmpdir"
(
    cd "$workdir"
    TMPDIR=relative-tmpdir "$script_dir/build_keyring_deb.sh" "$keyring" \
        0.1.0+build-foo 1 relative-tmp-output
)
relative_tmp_deb="$relative_tmp_output/cerulion-archive-keyring_0.1.0+build-foo-1_all.deb"
test -f "$relative_tmp_deb"
test "$(stat -c '%a' "$relative_tmp_deb")" = 644
printf '%s\n' 'build_keyring_deb relative TMPDIR promotion passed'

umask_debs="$workdir/umask-debs"
if ! (umask 000
    "$script_dir/build_keyring_deb.sh" "$keyring" 0.1.0 2 "$umask_debs" \
        >"$workdir/umask-output" 2>"$workdir/umask-error"
); then
    cat "$workdir/umask-error" >&2
    printf '%s\n' 'error: keyring package failed under permissive umask' >&2
    exit 1
fi
test -f "$umask_debs/cerulion-archive-keyring_0.1.0-2_all.deb"
printf '%s\n' 'build_keyring_deb umask handling passed'
