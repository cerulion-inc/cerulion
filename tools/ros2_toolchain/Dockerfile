# ROS 2 Jazzy + Rust toolchain for building & testing
# rmw_cerulion against the REAL distro headers, plus MoveIt 2 binaries
# for the unmodified-MoveIt e2e.
#
# This is a TOOLCHAIN image, not a benchmark. It has no COPY, so the build
# context carries nothing. Build it from the repository root:
#
#   docker build -t ros2-bench tools/ros2_toolchain/
#
# The `ros2-bench` tag is kept as it is: the `cerulion ros2 migrate` tooling
# (tools/ros2_migrate), the container-only rmw tests and the ros2 e2e tests all
# name it.
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl build-essential clang libclang-dev pkg-config git \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*

# The stock ROS 2 RMW, installed by name and not left to the base image: the
# `cerulion ros2 migrate` prover matrix (tools/ros2_migrate/run_matrix.sh) runs
# its pre-migration behavior baseline under rmw_fastrtps_cpp.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# MoveIt 2 (binaries) + the panda demo config used by the canonical
# pick-and-place tutorial. Separate layer: it is large.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-moveit \
    ros-jazzy-moveit-resources-panda-moveit-config \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Runtime requirements for the full MoveIt graph over rmw_cerulion:
#   docker run --shm-size=1g ...   # default 64M starves iceoryx2 segments
#   ulimit -n 524288               # move_group maps hundreds of segments;
#                                  # the 1024 default exhausts fds mid-init

# Rust (stable).
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"

WORKDIR /work

# For `cerulion ros2 migrate`: LLVM development headers + llvm-config
# for building the standalone clang LibTooling prover
# (tools/ros2_migrate/cerulion_ros2_migrate_clang.cpp). clang + libclang-dev
# are already installed above; llvm-dev supplies llvm-config and the LLVM
# headers LibTooling links against. Appended as its own layer so earlier
# layers stay cached.
RUN apt-get update && apt-get install -y --no-install-recommends \
    llvm-dev \
    && rm -rf /var/lib/apt/lists/*
