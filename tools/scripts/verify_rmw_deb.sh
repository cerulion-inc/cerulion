#!/bin/sh
#
# Prove a Cerulion Debian package carries a working ROS 2 Jazzy rmw.
#
# Installs the package in a clean `ros:jazzy-ros-base` container (Ubuntu 24.04
# with ros-jazzy-ros-base and no Cerulion), runs `cerulion ros2 run
# demo_nodes_cpp talker` and a listener through the shipped CLI, and checks
# with the stock `ros2 topic echo` and rclpy that the traffic crossed
# rmw_cerulion. The container half is verify_rmw_deb_container.sh.
#
# Usage: verify_rmw_deb.sh PACKAGE.deb OUT_DIR
#
# Every log the container writes lands in OUT_DIR; OUT_DIR/result.txt holds
# VERIFY_RMW_DEB_PASS on success.
#
# Env:
#   CERULION_RMW_VERIFY_IMAGE   container image (default ros:jazzy-ros-base)
set -eu

usage() {
    printf 'usage: %s PACKAGE.deb OUT_DIR\n' "$0" >&2
}

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

[ "$#" -eq 2 ] || {
    usage
    exit 2
}

deb=$1
out_arg=$2
script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
repo_root=$(CDPATH='' cd "$script_dir/../.." && pwd)
image=${CERULION_RMW_VERIFY_IMAGE:-ros:jazzy-ros-base}
container_script="$script_dir/verify_rmw_deb_container.sh"
# Raised iceoryx2 per-service port caps: every ROS node publishes on the shared
# /rosout and /parameter_events topics, and the ros2 CLI's discovery daemon is
# a third node beside the talker and the listener. The default cap of two
# publishers per service is a documented limit of multi-node graphs; it is not
# what this check proves.
iox2_config="$repo_root/examples/moveit_hero/iceoryx2.toml"

command -v docker >/dev/null 2>&1 ||
    die "docker is required to verify the package in a Jazzy container"
[ -f "$deb" ] || die "package does not exist: $deb"
case "$deb" in
    *.deb) ;;
    *) die "package must be a .deb file: $deb" ;;
esac
[ -f "$container_script" ] || die "container script missing: $container_script"
[ -f "$iox2_config" ] || die "iceoryx2 config missing: $iox2_config"

mkdir -p "$out_arg"
out=$(CDPATH='' cd "$out_arg" && pwd)
pkg_dir="$out/pkg"
rm -rf "$pkg_dir"
mkdir -p "$pkg_dir"
cp "$deb" "$pkg_dir/"

# --shm-size: the 64M default starves iceoryx2's shared-memory segments.
# --init: an init process reaps the C++ nodes the python ros2 wrapper leaves
# behind when its group is stopped, so no stage can wait on a zombie.
docker run --rm \
    --init \
    --shm-size=1g \
    -v "$pkg_dir":/pkg:ro \
    -v "$out":/out \
    -v "$container_script":/verify_rmw.sh:ro \
    -v "$iox2_config":/root/.config/iceoryx2/iceoryx2.toml:ro \
    "$image" \
    bash /verify_rmw.sh ||
    die "the container verification failed; logs are under $out"

grep -qx 'VERIFY_RMW_DEB_PASS' "$out/result.txt" 2>/dev/null ||
    die "the container did not record a pass in $out/result.txt"
printf 'verified %s\n' "$deb"
