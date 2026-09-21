#!/usr/bin/env python3
"""Headless move_group launch for the Panda hero demo.

The UNMODIFIED move_group node plus the minimal support graph (robot_state_
publisher, ros2_control with the demo's mock hardware, the joint-state
broadcaster + arm controller) — no RViz, so it runs in CI/headless. Both the
OMPL and pilz pipelines are loaded so scripted_plan.py can exercise each.

Nothing here is Cerulion-specific: the transport is selected entirely by the
RMW_IMPLEMENTATION env var the caller sets. That is the whole point — stock
MoveIt, zero source edits, different wire underneath.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )
        for name in ("joint_state_broadcaster", "panda_arm_controller")
    ]

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            *spawners,
        ]
    )
