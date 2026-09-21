# Third-party notice for the vendored ROS 2 message definitions

The `.msg` files in this directory are ROS 2 interface definitions taken from
upstream ROS 2 packages. They are not Cerulion's own work and they are not
covered by Cerulion's license. The Rust types generated from them at build
time are part of Cerulion and are licensed under AGPL-3.0-only.

Each package below is listed with its upstream repository, the revision
recorded for it in `../upstream_msg_manifest.txt`, and the exact value its
upstream `package.xml` gives in the `<license>` element at that revision. A
copyright holder is named only where upstream states one.

## Apache-2.0

Full text: <https://www.apache.org/licenses/LICENSE-2.0>

| Package | Upstream | Revision | Declared as |
| --- | --- | --- | --- |
| `action_msgs`, `builtin_interfaces`, `statistics_msgs` | [ros2/rcl_interfaces](https://github.com/ros2/rcl_interfaces) | `jazzy` | Apache License 2.0 |
| `diagnostic_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `shape_msgs`, `std_msgs`, `trajectory_msgs`, `visualization_msgs` | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) | `jazzy` | Apache License 2.0 |
| `vision_msgs` | [ros-perception/vision_msgs](https://github.com/ros-perception/vision_msgs) | `jazzy` | Apache License 2.0 |
| `radar_msgs` | [ros2-gbp/radar_msgs-release](https://github.com/ros2-gbp/radar_msgs-release) | `release/jazzy/radar_msgs/0.2.2-4` | Apache-2.0 |
| `autoware_perception_msgs`, `autoware_planning_msgs` | [autowarefoundation/autoware_msgs](https://github.com/autowarefoundation/autoware_msgs) | `main` | Apache License 2.0 |

The Autoware packages state a copyright holder: Copyright 2022 The Autoware
Foundation. The others state none.

## BSD-3-Clause

| Package | Upstream | Revision | Declared as | Copyright |
| --- | --- | --- | --- | --- |
| `unique_identifier_msgs` | [ros2/unique_identifier_msgs](https://github.com/ros2/unique_identifier_msgs) | `jazzy` | BSD | Copyright (C) 2012, Jack O'Quin |
| `tf2_msgs` | [ros2/geometry2](https://github.com/ros2/geometry2) | `jazzy` | BSD | Copyright (c) 2008, Willow Garage, Inc. |
| `grid_map_msgs` | [ANYbotics/grid_map](https://github.com/ANYbotics/grid_map) | `jazzy` | BSD | Copyright 2019, ANYbotics AG |
| `control_msgs` | [ros2-gbp/control_msgs-release](https://github.com/ros2-gbp/control_msgs-release) | `release/lyrical/control_msgs/6.10.0-1` | BSD-3-Clause | Copyright 2025 ros2_control Development Team |
| `moveit_msgs` | [ros2-gbp/moveit_msgs-release](https://github.com/ros2-gbp/moveit_msgs-release) | `release/jazzy/moveit_msgs/2.6.0-1` | BSD | none stated upstream |
| `octomap_msgs` | [ros2-gbp/octomap_msgs-release](https://github.com/ros2-gbp/octomap_msgs-release) | `release/jazzy/octomap_msgs/2.0.1-1` | BSD | Copyright (c) 2011-2013, A. Hornung, University of Freiburg |
| `object_recognition_msgs` | [ros2-gbp/object_recognition_msgs-release](https://github.com/ros2-gbp/object_recognition_msgs-release) | `release/jazzy/object_recognition_msgs/2.0.0-5` | BSD | none stated upstream |

`object_recognition_msgs` declares a bare `BSD` with no license file and no
copyright holder anywhere upstream. It is treated as BSD-3-Clause here because
that is the stricter reading, so honoring it also satisfies the two-clause
form.

The BSD 3-Clause text those packages are distributed under:

```
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

This file is documentation only. The build reads `.msg` files and ignores
everything else in this directory.
