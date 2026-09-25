#!/bin/sh
#
# Assemble a Cerulion Debian package from an existing release archive.
#
# The archive carries the three binaries and four license notices: LICENSE
# (the AGPL-3.0-only text the combined work is under), NOTICE (third-party
# attributions), LICENSE-BSD-3-CLAUSE (the license of the vendored ROS 2
# message packages that carry it) and THIRD-PARTY-LICENSES.md (the generated
# per-dependency inventory for the crates linked into these binaries). It
# also carries, for every released Linux target, two shared objects:
# librmw_cerulion.so (the ROS 2 Jazzy rmw)
# and libcerulion_heaphook.so (the LD_PRELOAD heap hook the CLI injects into
# ROS 2 nodes). All of them land in /usr/bin so the CLI finds its siblings by
# its own directory.
#
# A release archive also carries install_rust.sh and rustc-version.txt, the
# compiler bootstrap the tarball installer runs for itself. A package cannot
# run it: apt installs system wide while rustup writes into one user's home,
# so the pair ships under /usr/share/cerulion with a /usr/bin wrapper the user
# runs once, by hand. No maintainer script touches it.
#
# Env:
#   CERULION_DEB_ALLOW_MISSING_RMW=1   accept an archive without either shared
#                                      object (synthetic test archives only)
set -eu

usage() {
    printf 'usage: %s ARCHIVE.tar.gz ARCH OUTDIR\n' "$0" >&2
}

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

[ "$#" -eq 3 ] || {
    usage
    exit 2
}

archive=$1
arch=$2
outdir=$3
script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)

case "$arch" in
    amd64|arm64) ;;
    *) die "unsupported Debian architecture '$arch' (expected amd64 or arm64)" ;;
esac

[ -f "$archive" ] || die "release archive does not exist: $archive"
command -v dpkg-deb >/dev/null 2>&1 ||
    die "dpkg-deb is required to build a Debian package"
command -v tar >/dev/null 2>&1 ||
    die "tar is required to extract the release archive"

archive_name=$(basename "$archive")
version=$(printf '%s\n' "$archive_name" |
    sed -n 's/^cerulion-\(.*\)-\(x86_64-unknown-linux-gnu\|aarch64-unknown-linux-gnu\)\.tar\.gz$/\1/p')
archive_target=$(printf '%s\n' "$archive_name" |
    sed -n 's/^cerulion-.*-\(x86_64-unknown-linux-gnu\|aarch64-unknown-linux-gnu\)\.tar\.gz$/\1/p')
[ -n "$version" ] || die "archive name must be cerulion-VERSION-LINUX_TARGET.tar.gz"
[ -n "$archive_target" ] || die "archive name must include a supported Linux target"
expected_arch=
case "$archive_target" in
    x86_64-unknown-linux-gnu) expected_arch=amd64 ;;
    aarch64-unknown-linux-gnu) expected_arch=arm64 ;;
esac
[ "$arch" = "$expected_arch" ] ||
    die "requested Debian architecture '$arch' disagrees with archive target '$archive_target' (expected '$expected_arch')"
archive_stem=${archive_name%.tar.gz}

debian_version=$("$script_dir/debian_version.sh" "$version")

workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-deb.XXXXXX") ||
    die "could not create a temporary directory"
tmp_deb=
cleanup() {
    rm -rf "$workdir"
    [ -z "$tmp_deb" ] || rm -f "$tmp_deb"
}
trap cleanup EXIT

archive_listing=$(tar -tvzf "$archive") ||
    die "could not list release archive: $archive"
while IFS= read -r archive_entry; do
    archive_type=${archive_entry%"${archive_entry#?}"}
    case "$archive_type" in
        -|d) ;;
        *) die "non-regular archive entry: $archive_entry" ;;
    esac
done <<EOF
$archive_listing
EOF
archive_paths=$(tar -tzf "$archive") ||
    die "could not inspect release archive paths: $archive"
while IFS= read -r archive_path; do
    case "$archive_path" in
        /*|..|../*|*/../*|*/..)
            die "unsafe archive path: $archive_path"
            ;;
    esac
done <<EOF
$archive_paths
EOF
tar -xzf "$archive" -C "$workdir" ||
    die "could not extract release archive: $archive"
archive_dir="$workdir/$archive_stem"
[ -d "$archive_dir" ] || die "archive is missing top-level directory $archive_stem"

for binary in cerulion cerulion-netd cerulion-connectd; do
    if [ ! -f "$archive_dir/$binary" ] || [ -L "$archive_dir/$binary" ]; then
        die "archive is missing $binary"
    fi
done
# Redistributing the binaries carries the notices with them, so a package
# without one is a defect this check exists to refuse rather than a cosmetic
# omission. Named once here and reused by the install loop below, so the two
# can never drift apart.
notice_files="LICENSE NOTICE LICENSE-BSD-3-CLAUSE THIRD-PARTY-LICENSES.md"
for notice in $notice_files; do
    if [ ! -f "$archive_dir/$notice" ] || [ -L "$archive_dir/$notice" ]; then
        die "archive is missing $notice"
    fi
done
# The two shared objects. Every released Linux archive carries both, so a
# Linux package without one is a defect this check exists to refuse: without
# the rmw, `cerulion ros2 run` and `ros2 launch` exit 69 on every install;
# without the hook, every ROS 2 node those verbs start runs the copy path
# with no word said. The opt-out is for synthetic archives only (the CI
# packaging smoke and the script tests build debug binaries with no Jazzy
# container); the release workflow never sets it.
shipped_libraries="librmw_cerulion.so libcerulion_heaphook.so"
staged_libraries=
for library in $shipped_libraries; do
    if [ -f "$archive_dir/$library" ]; then
        staged_libraries="$staged_libraries $library"
    elif [ "${CERULION_DEB_ALLOW_MISSING_RMW:-0}" = 1 ]; then
        printf 'warning: packaging without %s (CERULION_DEB_ALLOW_MISSING_RMW=1); this package must not be released\n' \
            "$library" >&2
    else
        die "archive is missing $library (every Linux package carries the ROS 2 Jazzy rmw and the heap hook); set CERULION_DEB_ALLOW_MISSING_RMW=1 only for a synthetic archive that is never released"
    fi
done

# The compiler bootstrap travels as a pair or not at all: an archive carrying
# one half is defective, and the tarball installer refuses the same shape.
rust_bootstrap=0
if [ -f "$archive_dir/install_rust.sh" ] && [ -f "$archive_dir/rustc-version.txt" ]; then
    rust_bootstrap=1
elif [ -e "$archive_dir/install_rust.sh" ] || [ -e "$archive_dir/rustc-version.txt" ]; then
    die "archive carries only part of its Rust setup (install_rust.sh and rustc-version.txt travel together)"
fi

stage="$workdir/root"
mkdir -p "$stage/DEBIAN" "$stage/usr/bin" "$stage/usr/share/doc/cerulion"
chmod 0755 "$stage" "$stage/DEBIAN" "$stage/usr" "$stage/usr/bin" \
    "$stage/usr/share" "$stage/usr/share/doc" "$stage/usr/share/doc/cerulion"
for binary in cerulion cerulion-netd cerulion-connectd; do
    cp "$archive_dir/$binary" "$stage/usr/bin/$binary"
    chmod 0755 "$stage/usr/bin/$binary"
done
# Beside the binaries, the layout the CLI resolves without a code change:
# `cerulion ros2 run` looks for the rmw in its own directory and stages a
# minimal ament prefix that links it, and prepends the hook to the child's
# LD_PRELOAD when it sits there. A shared object takes 0644.
for library in $staged_libraries; do
    cp "$archive_dir/$library" "$stage/usr/bin/$library"
    chmod 0644 "$stage/usr/bin/$library"
done
for notice in $notice_files; do
    cp "$archive_dir/$notice" "$stage/usr/share/doc/cerulion/$notice"
    chmod 0644 "$stage/usr/share/doc/cerulion/$notice"
done

# The install-provenance marker the CLI reports as its install method in usage
# telemetry: it looks for share/cerulion/install.json one level above its bin.
mkdir -p "$stage/usr/share/cerulion"
chmod 0755 "$stage/usr/share/cerulion"
printf '{"method":"deb","version":"%s"}\n' "$version" > "$stage/usr/share/cerulion/install.json"
chmod 0644 "$stage/usr/share/cerulion/install.json"

# The bootstrap and the wrapper that runs it. The wrapper takes no arguments
# so the metadata file it passes is the one this package shipped, never a path
# a caller chose.
if [ "$rust_bootstrap" -eq 1 ]; then
    mkdir -p "$stage/usr/share/cerulion"
    chmod 0755 "$stage/usr/share/cerulion"
    cp "$archive_dir/install_rust.sh" "$stage/usr/share/cerulion/install_rust.sh"
    chmod 0644 "$stage/usr/share/cerulion/install_rust.sh"
    cp "$archive_dir/rustc-version.txt" "$stage/usr/share/cerulion/rustc-version.txt"
    chmod 0644 "$stage/usr/share/cerulion/rustc-version.txt"
    cat > "$stage/usr/bin/cerulion-install-rust" <<'EOF'
#!/bin/sh
# Install the Rust compiler that built this release, for building nodes.
# Run it once, as the user who will build nodes: it writes into that user's
# rustup and Cargo homes and leaves existing rustup defaults alone.
set -u
if [ "$#" -ne 0 ]; then
    printf 'usage: cerulion-install-rust\n' >&2
    exit 2
fi
# The helper speaks for the archive installer, whose failure arms say the
# Cerulion binaries were left alone. Here the package installed them already
# and nothing was staged to replace, so say what happened.
sh /usr/share/cerulion/install_rust.sh /usr/share/cerulion/rustc-version.txt
status=$?
if [ "$status" -ne 0 ]; then
    printf '%s\n' \
        'The Cerulion programs are installed; only the compiler setup failed.' >&2
fi
exit "$status"
EOF
    chmod 0755 "$stage/usr/bin/cerulion-install-rust"
fi

# Depends: what the shipped ELF objects link against. The rmw library NEEDs
# libstdc++ and libgcc_s (its std::string shim is compiled C++); it links no
# ROS library, because it is dlopened by a ROS 2 process that already
# carries rcutils. The heap hook links libc alone (glibc 2.34 or newer by
# design; the 2.35 floor below already covers it). ROS 2 itself is therefore a Suggests, not a Depends (that
# would refuse the CLI on every host without the ROS apt repository) and not
# a Recommends (apt installs those by default, which would pull all of ROS 2
# base into every install on a host that has the repository configured).
# build-essential, git and curl are Recommends for the opposite reason: apt
# installs them by default, so an apt user has the C linker `cargo` needs
# before the first `cerulion node build`, and a host that opts out of
# Recommends still gets the CLI.
cat > "$stage/DEBIAN/control" <<EOF
Package: cerulion
Version: $debian_version
Architecture: $arch
Section: devel
Priority: optional
Maintainer: Cerulion <packaging@cerulion.com>
Homepage: https://cerulion.com
Depends: libc6 (>= 2.35), libstdc++6, libgcc-s1
Recommends: cerulion-archive-keyring, build-essential, git, curl
Suggests: ros-jazzy-ros-base
Description: Zero-copy deterministic communication for real-time robotics
 Cerulion provides deterministic, zero-copy communication tools for robotics.
 The package includes the CLI and its network and desk-side daemon siblings,
 plus librmw_cerulion.so, the ROS 2 Jazzy rmw that cerulion ros2 run and
 cerulion ros2 launch load, and libcerulion_heaphook.so, the heap hook those
 verbs preload into ROS 2 nodes. ROS 2 Jazzy itself (ros-jazzy-ros-base on
 Ubuntu 24.04) is installed separately.
EOF
chmod 0644 "$stage/DEBIAN/control"

cat > "$stage/usr/share/doc/cerulion/copyright" <<'EOF'
Cerulion
Copyright: Cerulion contributors
License: AGPL-3.0-only
 The complete AGPL-3.0-only license text is installed in
 /usr/share/doc/cerulion/LICENSE. Third-party attributions are in
 /usr/share/doc/cerulion/NOTICE, the BSD-3-Clause text that some of
 them require in /usr/share/doc/cerulion/LICENSE-BSD-3-CLAUSE, and the
 per-dependency license inventory for the crates linked into these binaries
 in /usr/share/doc/cerulion/THIRD-PARTY-LICENSES.md.
EOF
chmod 0644 "$stage/usr/share/doc/cerulion/copyright"

mkdir -p "$outdir"
deb="$outdir/cerulion_${debian_version}_${arch}.deb"
if [ -e "$deb" ] || [ -L "$deb" ]; then
    die "output path already exists: $deb"
fi
tmp_deb=$(mktemp "$outdir/.cerulion.XXXXXX") ||
    die "could not create temporary package path"
dpkg-deb --root-owner-group --build "$stage" "$tmp_deb" >&2
chmod 0644 "$tmp_deb"
if ! ln "$tmp_deb" "$deb"; then
    die "output path already exists: $deb"
fi
if [ -d "$deb" ]; then
    # `ln` links into an existing directory, so a directory that appeared
    # after the precheck takes the link under its own name.
    rm -f "$deb/${tmp_deb##*/}"
    die "output path already exists: $deb"
fi
rm -f "$tmp_deb"
tmp_deb=
printf '%s\n' "$deb"
