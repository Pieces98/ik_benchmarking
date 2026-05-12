# -*- coding: utf-8 -*-
"""Launch the pose_ik Python benchmarker inside the MoveIt Pro container.

Builds a MoveItConfigs bundle for mw2 (URDF / SRDF / pose_ik kinematics) and
passes it as ROS parameters to `pose_ik_python_benchmarker.py`, which uses
moveit_py to load the robot model and exercise the pose_ik plugin.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_benchmarking_config():
    """Read ik_benchmarking.yaml and return a flat dict of parameters."""
    file_path = os.path.join(
        get_package_share_directory("ik_benchmarking"),
        "config",
        "ik_benchmarking.yaml",
    )
    with open(file_path, "r") as fh:
        cfg = yaml.safe_load(fh)

    required = (
        "moveit_config_pkg",
        "robot_name",
        "planning_group",
        "sample_size",
        "random_seed",
        "ik_timeout",
    )
    for key in required:
        if cfg.get(key) is None:
            raise ValueError(f"Missing required configuration key {key}")

    solvers = cfg.get("ik_solvers", [])
    if not solvers:
        raise ValueError("ik_benchmarking.yaml has no ik_solvers entries")
    pose_ik = next((s for s in solvers if s.get("name") == "pose_ik_speed"), solvers[0])

    return {
        "moveit_config_pkg": cfg["moveit_config_pkg"],
        "robot_name": cfg["robot_name"],
        "planning_group": cfg["planning_group"],
        "sample_size": cfg["sample_size"],
        "random_seed": cfg["random_seed"],
        "ik_timeout": cfg["ik_timeout"],
        "urdf_path": cfg.get("urdf_path"),
        "srdf_path": cfg.get("srdf_path"),
        "joint_limits_path": cfg.get("joint_limits_path"),
        "kinematics_source_pkg": cfg.get("kinematics_source_pkg", cfg["moveit_config_pkg"]),
        "kinematics_subdir": cfg.get("kinematics_subdir", "config"),
        "kinematics_file": pose_ik["kinematics_file"],
    }


def build_moveit_config(cfg):
    pkg_share = get_package_share_directory(cfg["moveit_config_pkg"])
    builder = MoveItConfigsBuilder(cfg["robot_name"], package_name=cfg["moveit_config_pkg"])
    if cfg["urdf_path"]:
        builder = builder.robot_description(file_path=os.path.join(pkg_share, cfg["urdf_path"]))
    if cfg["srdf_path"]:
        builder = builder.robot_description_semantic(
            file_path=os.path.join(pkg_share, cfg["srdf_path"])
        )
    if cfg["joint_limits_path"]:
        builder = builder.joint_limits(
            file_path=os.path.join(pkg_share, cfg["joint_limits_path"])
        )
    # Restrict planning_pipelines to ompl only. Otherwise the builder auto-discovers
    # pilz_industrial_motion_planner from its default_configs directory, which then forces
    # pilz_cartesian_limits.yaml to be loaded from mw2_base_config — and that file isn't
    # present in MoveIt Pro–style layouts. We only run IK here, so pipeline choice is moot.
    builder = builder.planning_pipelines(pipelines=["ompl"])
    kin_pkg_share = get_package_share_directory(cfg["kinematics_source_pkg"])
    kinematics_file = os.path.join(kin_pkg_share, cfg["kinematics_subdir"], cfg["kinematics_file"])
    return builder.robot_description_kinematics(file_path=kinematics_file).to_moveit_configs()


def launch_setup(_context, *_args, **_kwargs):
    cfg = load_benchmarking_config()
    moveit_config = build_moveit_config(cfg)

    benchmarker = Node(
        package="ik_benchmarking",
        executable="pose_ik_python_benchmarker.py",
        name="pose_ik_benchmarker",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "planning_group": cfg["planning_group"],
                "sample_size": cfg["sample_size"],
                "random_seed": cfg["random_seed"],
                "ik_timeout": cfg["ik_timeout"],
            },
        ],
    )
    return [benchmarker]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
