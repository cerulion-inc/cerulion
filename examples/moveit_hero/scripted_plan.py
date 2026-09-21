#!/usr/bin/env python3
"""Scripted MoveIt plan over rmw_cerulion.

Drives the UNMODIFIED move_group node through its MoveGroup action
(`/move_action`) over RMW_IMPLEMENTATION=rmw_cerulion. No MoveIt source is
touched; the only thing that changed vs a stock ROS 2 stack is the transport
underneath.

Two modes:

  ompl   (default)  Send one joint-space MotionPlanRequest (OMPL / RRTConnect,
                    plan_only) to the running move_group. Assert the action
                    returns SUCCESS and a non-empty trajectory; report the
                    point count + plan wall time. This is the "move_group is
                    alive on rmw_cerulion and produced a plan" gate.

  pilz              Send THREE identical PTP requests through the pilz
                    industrial planner (a DETERMINISTIC planner, unlike OMPL).
                    SHA-256 over every trajectory float; assert all three
                    digests are identical. This backs the launch-safe claim:
                    "deterministic planning". It is NOT replay-grade whole-host
                    determinism — it is plan-output bit-identity for a
                    deterministic planner over a scene that is static after
                    init. See the claim-discipline section of the README.

Exit: 0 pass, 1 assertion failed, 2 setup/timeout.
"""

import hashlib
import json
import os
import struct
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    WorkspaceParameters,
)

# Panda arm: 7 revolute joints. A modest, well-within-limits joint-space goal
# (distinct from the "ready" pose so the planner has to actually move).
PLANNING_GROUP = "panda_arm"
JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
GOAL_POSITIONS = [0.0, -0.785, 0.0, -2.0, 0.0, 1.2, 0.785]

ACTION_NAME = "/move_action"
MOVEIT_SUCCESS = 1  # moveit_msgs/MoveItErrorCodes.SUCCESS


def _build_goal(pipeline_id: str, planner_id: str) -> MoveGroup.Goal:
    req = MotionPlanRequest()
    req.group_name = PLANNING_GROUP
    req.pipeline_id = pipeline_id
    req.planner_id = planner_id
    req.num_planning_attempts = 1
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.1
    req.max_acceleration_scaling_factor = 0.1
    # A minimal workspace box so move_group does not reject the request.
    ws = WorkspaceParameters()
    ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -1.0
    ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 1.0
    req.workspace_parameters = ws

    constraints = Constraints()
    for name, pos in zip(JOINT_NAMES, GOAL_POSITIONS):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = pos
        jc.tolerance_above = 0.001
        jc.tolerance_below = 0.001
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    req.goal_constraints.append(constraints)

    goal = MoveGroup.Goal()
    goal.request = req
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = True  # plan, don't execute
    return goal


class Planner(Node):
    def __init__(self) -> None:
        super().__init__("cerulion_moveit_hero_plan")
        self._client = ActionClient(self, MoveGroup, ACTION_NAME)

    def wait_for_move_group(self, timeout_s: float = 60.0) -> bool:
        self.get_logger().info(
            f"waiting for move_group action server '{ACTION_NAME}' "
            f"(rmw={_active_rmw()}) ...")
        return self._client.wait_for_server(timeout_sec=timeout_s)

    def plan_once(self, pipeline_id: str, planner_id: str):
        """Send one goal, block for the result. Returns (error_code, traj)."""
        goal = _build_goal(pipeline_id, planner_id)
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=30.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("goal was rejected by move_group")
            return None, None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        wrapper = result_future.result()
        if wrapper is None:
            self.get_logger().error("no result from move_group (timeout)")
            return None, None
        res = wrapper.result
        return res.error_code.val, res.planned_trajectory.joint_trajectory


def _active_rmw() -> str:
    """The middleware rclpy ACTUALLY bound — not the env var that asked for it.

    `RMW_IMPLEMENTATION` records what was *requested*. This records what the
    ROS middleware really loaded. Asserting on the env var would be circular:
    this process is the one that exported it, so the check could never fail.
    The whole demo rests on rmw_cerulion being the middleware in use, so the
    CI gate asserts on this value.

    Fails CLOSED — if the identifier cannot be read, the returned string is
    deliberately not a valid implementation name, so the gate rejects the run
    rather than falling back to the env var and re-introducing the tautology.
    """
    try:
        return rclpy.get_rmw_implementation_identifier()
    except Exception as exc:  # rclpy too old, or context not initialised
        requested = os.environ.get("RMW_IMPLEMENTATION", "unset")
        return f"<unresolved requested={requested} err={type(exc).__name__}>"


def _write_result(mode: str, **fields) -> None:
    """Write a machine-readable result file for the CI gate to jq-assert.

    This JSON is the primary CI signal; the grep-able stdout sentinel (below)
    is the belt-and-suspenders one — neither depends on the ROS 2 logger.
    No-op when CER_DEMO_RESULT_DIR is unset (interactive runs).
    """
    out_dir = os.environ.get("CER_DEMO_RESULT_DIR")
    if not out_dir:
        return
    fields["mode"] = mode
    fields["rmw"] = _active_rmw()
    with open(os.path.join(out_dir, f"{mode}.json"), "w") as f:
        json.dump(fields, f)


def _traj_digest(traj) -> str:
    """SHA-256 over every float in a JointTrajectory (names + all point data).

    Order-stable byte encoding so the digest is a faithful fingerprint of the
    planned trajectory. Any difference in point count, timing, or joint values
    changes the digest.

    `effort` is included even though pilz leaves it empty for PTP: the claim
    this backs is "every trajectory float", and a digest that silently skipped
    a field would hash two genuinely different plans identically.
    """
    h = hashlib.sha256()
    for name in traj.joint_names:
        h.update(name.encode("utf-8"))
    for pt in traj.points:
        for arr in (pt.positions, pt.velocities, pt.accelerations,
                    pt.effort):
            for v in arr:
                h.update(struct.pack("<d", v))
        h.update(struct.pack("<ii", pt.time_from_start.sec,
                             pt.time_from_start.nanosec))
    return h.hexdigest()


def run_ompl(node: Planner) -> int:
    t0 = time.monotonic()
    code, traj = node.plan_once("ompl", "RRTConnectkConfigDefault")
    wall_ms = (time.monotonic() - t0) * 1000.0
    if code != MOVEIT_SUCCESS:
        node.get_logger().error(f"OMPL plan FAILED error_code={code}")
        return 1
    n = len(traj.points)
    if n == 0:
        node.get_logger().error("OMPL plan SUCCESS but empty trajectory")
        return 1
    node.get_logger().info(
        f"OMPL_PLAN_OK rmw={_active_rmw()} points={n} wall_ms={wall_ms:.1f}")
    # Sentinel on guaranteed stdout (not the ROS 2 logger) — the CI gate greps
    # this, independent of logger level/routing.
    print(f"OMPL_PLAN_OK RESULT ompl points={n} wall_ms={wall_ms:.1f}", flush=True)
    _write_result("ompl", status="pass", points=n, wall_ms=round(wall_ms, 1))
    return 0


def run_pilz(node: Planner, runs: int = 3) -> int:
    digests = []
    for i in range(runs):
        code, traj = node.plan_once("pilz_industrial_motion_planner", "PTP")
        if code != MOVEIT_SUCCESS:
            node.get_logger().error(
                f"pilz plan {i} FAILED error_code={code}")
            return 1
        if len(traj.points) == 0:
            node.get_logger().error(f"pilz plan {i} empty trajectory")
            return 1
        d = _traj_digest(traj)
        digests.append((len(traj.points), d))
        node.get_logger().info(
            f"pilz run {i}: points={len(traj.points)} sha256={d[:16]}...")

    counts = {c for c, _ in digests}
    hashes = {d for _, d in digests}
    if len(hashes) == 1 and len(counts) == 1:
        points = digests[0][0]
        full = digests[0][1]
        node.get_logger().info(
            f"PILZ_DETERMINISM_PASS rmw={_active_rmw()} runs={runs} "
            f"points={points} sha256={full}")
        # Sentinel on guaranteed stdout (not the ROS 2 logger).
        print(f"PILZ_DETERMINISM_PASS RESULT pilz runs={runs} points={points} "
              f"identical=1 sha256={full}", flush=True)
        _write_result("pilz", status="pass", runs=runs, points=points,
                      identical=True, sha256=full)
        return 0
    node.get_logger().error(
        f"PILZ_DETERMINISM_FAIL distinct_hashes={len(hashes)} "
        f"distinct_counts={len(counts)} digests={digests}")
    return 1


def main() -> int:
    mode = sys.argv[1] if len(sys.argv) > 1 else "ompl"
    if mode not in ("ompl", "pilz"):
        sys.stderr.write(f"usage: scripted_plan.py [ompl|pilz]; got {mode!r}\n")
        return 2

    rclpy.init()
    node = Planner()
    try:
        if not node.wait_for_move_group():
            node.get_logger().error("move_group action server never appeared")
            return 2
        return run_ompl(node) if mode == "ompl" else run_pilz(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
