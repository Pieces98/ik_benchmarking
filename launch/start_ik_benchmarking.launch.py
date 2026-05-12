# -*- coding: utf-8 -*-
# Reads URDF (xacro-expanded) and SRDF from disk and pushes them as
# robot_description / robot_description_semantic parameters on the server node.
# The server then forwards both to the IKBenchmarking child so MoveIt Pro's
# RobotModelLoader builds the model from those parameters — no /move_group
# required. This bypasses MoveItConfigsBuilder entirely (whose default file
# layout doesn't match mw2_base_config's MoveIt Pro style).

import os
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_benchmarking_config(pkg, filename):
    file_path = os.path.join(get_package_share_directory(pkg), "config", filename)
    with open(file_path, "r") as f:
        config_data = yaml.safe_load(f)

    def need(key):
        v = config_data.get(key)
        if v is None:
            raise ValueError(f"Missing required configuration key {key}")
        return v

    return {
        "urdf_pkg": config_data.get("urdf_pkg", "mw2_description"),
        "urdf_path": config_data.get("urdf_path", "urdf/mw2.urdf"),
        "srdf_pkg": config_data.get("srdf_pkg", "mw2_base_config"),
        "srdf_path": config_data.get("srdf_path", "config/moveit/mw2.srdf"),
        "planning_group": need("planning_group"),
        "sample_size": need("sample_size"),
        "random_seed": need("random_seed"),
        "ik_timeout": need("ik_timeout"),
        "ik_iteration_display_step": need("ik_iteration_display_step"),
        "check_self_collision": config_data.get("check_self_collision", False),
        "ik_solvers": [
            {"name": s.get("name")} for s in need("ik_solvers")
        ],
    }


def _read_text_from_pkg(pkg, relpath):
    abspath = os.path.join(get_package_share_directory(pkg), relpath)
    with open(abspath, "r") as f:
        return f.read()


def _process_urdf(pkg, relpath):
    """Resolve URDF text. If the path ends in .xacro it is expanded; otherwise
    the file is read as-is. Xacro expansion handles `$(find ...)` and relative
    `<xacro:include>` so the dependent files (e.g. mw2.ros2_control.xacro,
    mw2_description/urdf/mw2.urdf) are pulled in correctly."""
    abspath = os.path.join(get_package_share_directory(pkg), relpath)
    if abspath.endswith(".xacro"):
        return xacro.process_file(abspath).toxml()
    with open(abspath, "r") as f:
        return f.read()


def prepare_benchmarking(context, *args, **kwargs):
    cfg = load_benchmarking_config("ik_benchmarking", "ik_benchmarking.yaml")

    ik_solver_name = LaunchConfiguration("ik_solver_name").perform(context)
    if ik_solver_name == "":
        print("\n Error: The 'ik_solver_name' argument should be provided.\n")
        exit(1)
    if not any(s["name"] == ik_solver_name for s in cfg["ik_solvers"]):
        print(
            f"\n Error: The requested IK solver name '{ik_solver_name}' is not in ik_benchmarking.yaml.\n"
        )
        exit(1)

    urdf_text = _process_urdf(cfg["urdf_pkg"], cfg["urdf_path"])
    srdf_text = _read_text_from_pkg(cfg["srdf_pkg"], cfg["srdf_path"])

    print(
        f"\n Running IK benchmarking against MoveIt Pro for solver: {ik_solver_name} "
        f"(self-collision={'on' if cfg['check_self_collision'] else 'off'}, "
        f"URDF={cfg['urdf_pkg']}/{cfg['urdf_path']}, "
        f"SRDF={cfg['srdf_pkg']}/{cfg['srdf_path']})\n"
    )

    server = Node(
        package="ik_benchmarking",
        executable="ik_benchmarking_server",
        output="screen",
        parameters=[
            {
                "planning_group": cfg["planning_group"],
                "random_seed": cfg["random_seed"],
                "sample_size": cfg["sample_size"],
                "ik_timeout": cfg["ik_timeout"],
                "ik_iteration_display_step": cfg["ik_iteration_display_step"],
                "check_self_collision": cfg["check_self_collision"],
                "robot_description": urdf_text,
                "robot_description_semantic": srdf_text,
            },
        ],
    )

    client = Node(
        package="ik_benchmarking",
        executable="ik_benchmarking_client",
        output="screen",
        parameters=[
            {
                "planning_group": cfg["planning_group"],
                "sample_size": cfg["sample_size"],
                "ik_solver": ik_solver_name,
            },
        ],
    )

    return [server, client]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ik_solver_name",
                default_value="",
                description="IK solver name corresponding to the name value in ik_benchmarking.yaml.",
            ),
            OpaqueFunction(function=prepare_benchmarking),
        ]
    )
