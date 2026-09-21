#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
#
# Refresh native_ros2_messages/upstream_msg_manifest.txt — the
# checked-in snapshot of every vendored message's UPSTREAM ROS 2 signature,
# which `upstream_drift_test.rs` gates the corpus against. Each package is
# pinned to the distro it is vendored from: of 22 packages, 19 Jazzy,
# control_msgs LYRICAL, and the two autoware_* packages have NO distro pin
# at all (see PINNED REFS below).
#
# WHY THIS IS A SEPARATE, DELIBERATE STEP
# ---------------------------------------
# CI has no network and no ROS install, so the drift gate cannot fetch
# upstream; it compares against the checked-in manifest. That means the gate
# catches "our corpus moved away from recorded upstream" but NOT "upstream
# itself changed". Closing that second gap is this script's job, and its
# output is a reviewable git diff of the manifest — so adopting an upstream
# change is an explicit, reviewed act instead of an invisible one.
#
# The refresh rewrites ONLY the signature blocks. The `!source`, `!accept`
# and `!accept-fields` lines are human judgements and are preserved verbatim.
# It REFUSES to run without an existing manifest rather than regenerating a
# bare one, which would silently delete every one of those judgements.
#
# USAGE
#   ./scripts/refresh_upstream_msg_manifest.sh
#   git diff crates/native_ros2_messages/upstream_msg_manifest.txt   # REVIEW THIS
#
# PINNED REFS — one DISTRO PIN PER PACKAGE
# ----------------------------------------
# Each source below is pinned to the ref that IS the upstream release of
# that package for the distro WE VENDOR IT FROM, per
# https://github.com/ros/rosdistro `<distro>/distribution.yaml`:
#   * repos with a `<distro>` branch -> that branch (it IS the release branch)
#   * repos without one              -> the ros2-gbp release repo at the exact
#                                       `release/<distro>/<pkg>/<version>` tag
#                                       named by distribution.yaml (the
#                                       authoritative released source)
#
# Of the 22 packages, 20 carry a distro pin (19 Jazzy + control_msgs on
# Lyrical) and 2 (the autoware_* pair) carry none at all. "Jazzy" is the
# majority, NOT the whole corpus. Only the 5 ros2-gbp `release/...` pins are
# immutable tags; the other 17 name a MOVING branch, so two refreshes months
# apart can produce different signatures under identical `!source` lines.
#
# These refs are cross-checked against the manifest's `!source` lines by
# upstream_drift_test.rs::refresh_script_clone_refs_match_the_manifest_source_pins,
# so bumping a pin HERE without bumping it there (or vice versa) fails CI
# rather than leaving the manifest claiming an upstream it did not come from.
#
# control_msgs is pinned to LYRICAL. It is the one package we
# vendor from a later distro: 13 of its 39 messages do not exist in Jazzy at
# all, so a Jazzy pin could not compare them and waived them wholesale — and
# that blanket waiver HID a real, hash-affecting fork in `VDA5050State`
# (`string last_node_id` where upstream declares `uint32`), which silently
# drops every frame from a stock robot at the schema-hash gate. Pinning to
# the distro that actually ships these messages makes all 39 comparable and
# collapses 15 waivers to zero. Deleting the 13 Jazzy-less types instead was
# considered and rejected: they are public API.
#
# autoware_msgs is NOT a rosdistro package and has no distro pin at all; it
# is fetched from `main` and is recorded UNVERIFIED in the manifest.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
SRC="$WORK/src"
FLAT="$WORK/flat"
mkdir -p "$SRC" "$FLAT"

clone() { # <owner/repo> <ref> <dest-name>
  echo "  fetching $1@$2"
  git clone --quiet --depth 1 --branch "$2" "https://github.com/$1.git" "$SRC/$3"
}

# <dest-name> <path-within-repo> <package>  -> $FLAT/<package>/*.msg
place() {
  local from="$SRC/$1/$2" pkg="$3"
  if [ ! -d "$from" ]; then
    echo "ERROR: expected msg dir missing: $from" >&2
    exit 1
  fi
  mkdir -p "$FLAT/$pkg"
  cp "$from"/*.msg "$FLAT/$pkg/" 2>/dev/null || true
}

echo "manifest refresh: fetching distro-pinned upstream sources..."
clone ros2/common_interfaces                jazzy                                          common_interfaces
clone ros2/rcl_interfaces                   jazzy                                          rcl_interfaces
clone ros2/unique_identifier_msgs           jazzy                                          unique_identifier_msgs
clone ros2/geometry2                        jazzy                                          geometry2
clone ANYbotics/grid_map                    jazzy                                          grid_map
clone ros-perception/vision_msgs            jazzy                                          vision_msgs
clone ros2-gbp/control_msgs-release             release/lyrical/control_msgs/6.10.0-1                 control_msgs
clone ros2-gbp/moveit_msgs-release              release/jazzy/moveit_msgs/2.6.0-1                     moveit_msgs
clone ros2-gbp/octomap_msgs-release             release/jazzy/octomap_msgs/2.0.1-1                    octomap_msgs
clone ros2-gbp/object_recognition_msgs-release  release/jazzy/object_recognition_msgs/2.0.0-5         object_recognition_msgs
clone ros2-gbp/radar_msgs-release               release/jazzy/radar_msgs/0.2.2-4                      radar_msgs
clone autowarefoundation/autoware_msgs      main                                           autoware_msgs

echo "manifest refresh: laying out <package>/<Name>.msg..."
place common_interfaces        diagnostic_msgs/msg          diagnostic_msgs
place common_interfaces        geometry_msgs/msg            geometry_msgs
place common_interfaces        nav_msgs/msg                 nav_msgs
place common_interfaces        sensor_msgs/msg              sensor_msgs
place common_interfaces        shape_msgs/msg               shape_msgs
place common_interfaces        std_msgs/msg                 std_msgs
place common_interfaces        trajectory_msgs/msg          trajectory_msgs
place common_interfaces        visualization_msgs/msg       visualization_msgs
place rcl_interfaces           action_msgs/msg              action_msgs
place rcl_interfaces           builtin_interfaces/msg       builtin_interfaces
place rcl_interfaces           statistics_msgs/msg          statistics_msgs
place unique_identifier_msgs   msg                          unique_identifier_msgs
place geometry2                tf2_msgs/msg                 tf2_msgs
place grid_map                 grid_map_msgs/msg            grid_map_msgs
place vision_msgs              vision_msgs/msg              vision_msgs
place control_msgs             msg                          control_msgs
place moveit_msgs              msg                          moveit_msgs
place octomap_msgs             msg                          octomap_msgs
place object_recognition_msgs  msg                          object_recognition_msgs
place radar_msgs               msg                          radar_msgs
place autoware_msgs            autoware_perception_msgs/msg autoware_perception_msgs
place autoware_msgs            autoware_planning_msgs/msg   autoware_planning_msgs

echo "manifest refresh: regenerating manifest signature blocks..."
cd "$REPO_ROOT"
UPSTREAM_REFRESH_FROM="$FLAT" cargo test -p native_ros2_messages \
  --test upstream_drift_test -- --ignored --nocapture refresh_manifest_from_upstream_tree

cat <<'EOM'

Upstream manifest refresh complete.

NEXT — this is the whole point of the step, do not skip it:
  git diff crates/native_ros2_messages/upstream_msg_manifest.txt

Any signature change is upstream having moved. Decide per message, IN THIS
ORDER — a waiver is the last resort, not the first, because a waiver buys
silence and silence is what hid the original drift's string-vs-uint32 fork:

  1. FOLLOW UPSTREAM — edit the vendored .msg to match. Almost always right.
  2. RE-PIN THE DISTRO — if a whole package looks wrong, or messages have no
     upstream text at all, its '!source' line probably names a distro that
     does not ship them. That WAS the original drift's actual root cause.
     Re-pin and re-run this script (update the manifest '!source' AND the
     clone table below; CI cross-checks that they agree).
  3. '!accept <pkg/Name> <reason>' — waives the CONSTANT lines ONLY. Field
     drift under it still FAILS, by design.
  4. '!accept-fields <pkg/Name> <reason>' — LAST RESORT. Waives the FIELD
     list too, i.e. accepts a divergence that WILL make a stock robot's
     frames hash-mismatch. Also the only directive that covers a message
     upstream does not have at all (absence compares nothing, fields
     included). Say why in the reason.

Then run:
  cargo test -p native_ros2_messages --test upstream_drift_test
EOM
