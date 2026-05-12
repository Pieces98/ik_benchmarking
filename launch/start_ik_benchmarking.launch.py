# -*- coding: utf-8 -*-
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def load_benchmarking_config(ik_benchmarking_pkg, ik_benchmarking_config):
    # Construct the configuration file path
    file_path = os.path.join(
        get_package_share_directory(ik_benchmarking_pkg),
        "config",
        ik_benchmarking_config,
    )
    # Open file and parse content
    with open(file_path, "r") as config_file:
        config_data = yaml.safe_load(config_file)

    # Extract content and handle missing keys
    def get_config_data(key, parent_data=None):
        source_data = parent_data if parent_data else config_data
        value = source_data.get(key)
        if value is None:
            raise ValueError(f"Missing required configuration key {key}")
        return value

    moveit_config_pkg = get_config_data("moveit_config_pkg")
    robot_name = get_config_data("robot_name")
    planning_group = get_config_data("planning_group")
    sample_size = get_config_data("sample_size")
    random_seed = get_config_data("random_seed")
    ik_timeout = get_config_data("ik_timeout")
    ik_iteration_display_step = get_config_data("ik_iteration_display_step")

    # Optional overrides for non-standard MoveIt config layouts (e.g., MoveIt Pro style)
    urdf_path = config_data.get("urdf_path")
    srdf_path = config_data.get("srdf_path")
    joint_limits_path = config_data.get("joint_limits_path")
    kinematics_source_pkg = config_data.get("kinematics_source_pkg", moveit_config_pkg)
    kinematics_subdir = config_data.get("kinematics_subdir", "config")

    # Extract IK solvers details
    ik_solvers_list = []
    ik_solvers_data = get_config_data("ik_solvers")

    for ik_value in ik_solvers_data:
        ik_solver_name = ik_value.get("name")
        ik_solver_kinematics_file = ik_value.get("kinematics_file")

        ik_solvers_list.append(
            {"name": ik_solver_name, "kinematics_file": ik_solver_kinematics_file}
        )

    # Return a dictionary to avoid errors due to return order
    return {
        "moveit_config_pkg": moveit_config_pkg,
        "robot_name": robot_name,
        "planning_group": planning_group,
        "sample_size": sample_size,
        "random_seed": random_seed,
        "ik_timeout": ik_timeout,
        "ik_iteration_display_step": ik_iteration_display_step,
        "urdf_path": urdf_path,
        "srdf_path": srdf_path,
        "joint_limits_path": joint_limits_path,
        "kinematics_source_pkg": kinematics_source_pkg,
        "kinematics_subdir": kinematics_subdir,
        "ik_solvers": ik_solvers_list,
    }


# Utilize Opaque functions to retrieve the string values of launch arguments
def prepare_benchmarking(context, *args, **kwargs):
    # Load the ik_benchmarking configuration data
    ik_benchmarking_pkg = "ik_benchmarking"
    ik_benchmarking_config = "ik_benchmarking.yaml"
    benchmarking_config = load_benchmarking_config(
        ik_benchmarking_pkg, ik_benchmarking_config
    )

    # Get the actual values of ik_solver and kinematics file
    ik_solver_name = LaunchConfiguration("ik_solver_name").perform(context)
    kinematics_file_name = ""

    if ik_solver_name != "":
        found = False
        for _, ik_solver in enumerate(benchmarking_config["ik_solvers"]):
            if ik_solver["name"] == ik_solver_name:
                found = True
                kinematics_file_name = ik_solver["kinematics_file"]
                break

        if not found:
            print(
                f"\n Error: The requested IK solver name {ik_solver_name} is not available in the ik_benchmarking configuration file.\n"
            )
            exit(1)
    else:
        print(
            f"\n Error: The 'ik_solver_name' argument should be provided when starting the 'start_ik_benchmarking.launch.py' file.\n"
        )
        exit(1)

    # Build moveit_config using the robot name and kinematic file
    robot_name = benchmarking_config["robot_name"]
    package_name = benchmarking_config["moveit_config_pkg"]
    pkg_share = get_package_share_directory(package_name)

    builder = MoveItConfigsBuilder(robot_name, package_name=package_name)

    # Override URDF/SRDF lookups when the MoveIt config package uses a non-standard layout
    if benchmarking_config.get("urdf_path"):
        builder = builder.robot_description(
            file_path=os.path.join(pkg_share, benchmarking_config["urdf_path"])
        )
    if benchmarking_config.get("srdf_path"):
        builder = builder.robot_description_semantic(
            file_path=os.path.join(pkg_share, benchmarking_config["srdf_path"])
        )
    if benchmarking_config.get("joint_limits_path"):
        builder = builder.joint_limits(
            file_path=os.path.join(pkg_share, benchmarking_config["joint_limits_path"])
        )

    # Per-solver kinematics YAML may live in a different package than moveit_config_pkg
    kin_pkg_share = get_package_share_directory(benchmarking_config["kinematics_source_pkg"])
    kinematics_file_path = os.path.join(
        kin_pkg_share, benchmarking_config["kinematics_subdir"], kinematics_file_name
    )

    moveit_config = builder.robot_description_kinematics(
        file_path=kinematics_file_path
    ).to_moveit_configs()

    # Start benchmarking server node with required robot description and planning_group parameters
    benchmarking_server_node = Node(
        package="ik_benchmarking",
        executable="ik_benchmarking_server",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "planning_group": benchmarking_config["planning_group"],
                "random_seed": benchmarking_config["random_seed"],
                "sample_size": benchmarking_config["sample_size"],
                "ik_timeout": benchmarking_config["ik_timeout"],
                "ik_iteration_display_step": benchmarking_config[
                    "ik_iteration_display_step"
                ],
            },
        ],
    )

    print(
        f"\n Running calculations for IK Solver: {ik_solver_name} \n",
    )

    # Start benchmarking client node with the same parameters as the server, but with delay
    benchmarking_client_node = Node(
        package="ik_benchmarking",
        executable="ik_benchmarking_client",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "planning_group": benchmarking_config["planning_group"],
                "sample_size": benchmarking_config["sample_size"],
                "ik_solver": ik_solver_name,
            },
        ],
    )

    return [benchmarking_server_node, benchmarking_client_node]


def generate_launch_description():
    # Declare a launch argument to decide the IK solver and kinematic file to use
    declare_ik_solver_name_arg = DeclareLaunchArgument(
        "ik_solver_name",
        default_value="",
        description="IK solver name corresponding to the name value in ik_benchmarking.yaml config file.",
    )

    return LaunchDescription(
        [declare_ik_solver_name_arg, OpaqueFunction(function=prepare_benchmarking)]
    )
