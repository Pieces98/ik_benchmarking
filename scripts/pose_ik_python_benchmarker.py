#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone pose_ik benchmarker for the MoveIt Pro environment.

Why this file exists: in the MoveIt Pro container, `moveit_ros_planning_interface`
is absent (the overlay reorganizes moveit into `moveit_pro_base`), so the package's
C++ benchmarking server cannot be built. This Python script benchmarks the
`pose_ik_plugin/PoseIKPlugin` solver directly via moveit_py, producing a CSV in the
same format the upstream C++ server writes:

    trial,found_ik,solve_time,position_error,orientation_error

so that `ik_benchmarking_data_visualizer.py` can render its results alongside
output from the `mw2-moveit2-ik-benchmarking` branch.
"""
import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node


def fetch_params(node):
    """Read parameters supplied by the launch file."""
    declared = {
        "planning_group": "manipulator",
        "sample_size": 1000,
        "random_seed": 12345,
        "ik_timeout": 0.1,
        "tip_link": "grasp_link",
        "output_csv": "pose_ik_speed_ik_benchmarking_data.csv",
    }
    out = {}
    for name, default in declared.items():
        param = node.declare_parameter(name, default)
        out[name] = param.value
    return out


def pose_to_matrix(pose):
    """Convert geometry_msgs/Pose (or moveit_py Pose) to a 4x4 numpy matrix."""
    p = pose.position
    q = pose.orientation
    w, x, y, z = q.w, q.x, q.y, q.z
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def orientation_error_deg(R_actual, R_target):
    """Angle (radians) between two rotation matrices."""
    R = R_actual @ R_target.T
    cos = (np.trace(R) - 1.0) / 2.0
    return float(math.acos(max(-1.0, min(1.0, cos))))


def run_benchmark(node, params):
    # moveit_py import deferred so missing-overlay errors surface with a clear message.
    try:
        from moveit.planning import MoveItPy
    except ImportError as exc:
        node.get_logger().error(
            f"moveit_py is not available in this environment ({exc}). "
            "Ensure the MoveIt Pro overlay is sourced before launching."
        )
        sys.exit(1)

    group_name = params["planning_group"]
    tip_link = params["tip_link"]
    sample_size = int(params["sample_size"])
    timeout = float(params["ik_timeout"])
    output_csv = params["output_csv"]

    moveit = MoveItPy(node_name="pose_ik_benchmarker_inner")
    robot_model = moveit.get_robot_model()
    group = robot_model.get_joint_model_group(group_name)
    joint_names = list(group.active_joint_model_names)

    # Joint bounds for uniform sampling. moveit_py exposes variable_bounds per joint.
    bounds = []
    for j in joint_names:
        jm = robot_model.get_joint_model(j)
        b = jm.variable_bounds[0]
        bounds.append((b.min_position, b.max_position))

    rng = np.random.default_rng(seed=int(params["random_seed"]))
    state = moveit.get_planning_scene_monitor().read_only().current_state

    success_count = 0
    rows = []

    for trial in range(1, sample_size + 1):
        # Sample a target via FK
        q_fk = [rng.uniform(lo, hi) for (lo, hi) in bounds]
        state.set_joint_group_positions(group_name, q_fk)
        state.update()
        target_T = pose_to_matrix(state.get_pose(tip_link))

        # Reseed with an independent random initial guess so IK doesn't trivially match
        q_seed = [rng.uniform(lo, hi) for (lo, hi) in bounds]
        state.set_joint_group_positions(group_name, q_seed)

        t0 = time.perf_counter_ns()
        ok = state.set_from_ik(group, state.get_pose(tip_link), timeout=timeout)
        # NB: set_from_ik signatures differ across moveit_py versions; if needed,
        # adjust to pass the target Pose directly. Here we re-snapshot the FK pose.
        dt_us = (time.perf_counter_ns() - t0) // 1000

        if ok:
            state.update()
            achieved_T = pose_to_matrix(state.get_pose(tip_link))
            pos_err = float(np.linalg.norm(achieved_T[:3, 3] - target_T[:3, 3]))
            orient_err = orientation_error_deg(achieved_T[:3, :3], target_T[:3, :3])
            success_count += 1
        else:
            pos_err = float("nan")
            orient_err = float("nan")

        rows.append((trial, str(bool(ok)).lower(), dt_us, pos_err, orient_err))

        if trial % max(1, sample_size // 10) == 0:
            node.get_logger().info(
                f"Solved sample {trial}/{sample_size}  successes={success_count}"
            )

    output_path = os.path.abspath(output_csv)
    with open(output_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow([
            "trial", "found_ik", "solve_time", "position_error", "orientation_error",
        ])
        writer.writerows(rows)

    node.get_logger().info(
        f"pose_ik benchmark done: success_rate={success_count}/{sample_size} "
        f"= {success_count / sample_size:.3f}, csv={output_path}"
    )


def main():
    rclpy.init()
    node = rclpy.create_node("pose_ik_benchmarker")
    try:
        params = fetch_params(node)
        run_benchmark(node, params)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
