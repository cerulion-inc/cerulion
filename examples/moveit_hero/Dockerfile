# The unmodified-MoveIt hero demo.
# ROS 2 Jazzy + MoveIt 2 (binaries) + the Panda demo config + a Rust
# toolchain to build rmw_cerulion from the repo source. move_group runs
# UNMODIFIED on RMW_IMPLEMENTATION=rmw_cerulion (Cerulion's zero-copy
# iceoryx2 transport) — zero MoveIt source edits.
#
# This is a trimmed sibling of the ROS 2 toolchain image
# (tools/ros2_toolchain/Dockerfile, which also carries the ROS 2 demo nodes and
# the LLVM headers the migrate prover builds against). The hero demo needs none
# of that, only MoveIt + the Panda config + Rust.
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# Build deps for rmw_cerulion (clang/libclang for bindgen against the live
# distro headers; see crates/rmw_cerulion/build.rs).
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl build-essential clang libclang-dev pkg-config git \
    && rm -rf /var/lib/apt/lists/*

# MoveIt 2 + the canonical Panda demo config + the controllers the demo
# launch activates. Separate layer: it is large (~GB).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-moveit \
    ros-jazzy-moveit-resources-panda-moveit-config \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# iceoryx2 per-service port ceiling: raise max-publishers/subscribers so a
# multi-node ROS graph fits on the shared /rosout + /parameter_events topics
# (default cap is 2). Auto-loaded by Config::global_config() from the user
# config path. See the file header for the full rationale.
COPY iceoryx2.toml /root/.config/iceoryx2/iceoryx2.toml

# Rust (stable). rmw_cerulion is built at RUN time from the bind-mounted repo
# at /work, not baked into the image — build.rs must bindgen against the
# distro headers on AMENT_PREFIX_PATH, and the source is the mounted repo.
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"

WORKDIR /work

# Runtime requirements (the run/CI scripts set these):
#   docker run --shm-size=1g       # 64M default starves iceoryx2 segments
#   docker run --ulimit nofile=524288   # move_group maps hundreds of segments
