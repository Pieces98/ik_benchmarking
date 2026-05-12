cmake_minimum_required(VERSION 3.8)
project(ik_benchmarking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# This branch targets the MoveIt Pro container, where moveit_ros_planning_interface
# does not exist. The C++ benchmarker (ik_benchmarking_server / _client) is therefore
# not built here — pose_ik is exercised via scripts/pose_ik_python_benchmarker.py
# instead, which loads the plugin through moveit_py.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Action interface kept for parity with the mw2-moveit2-ik-benchmarking branch.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/IKBenchmark.action"
  DEPENDENCIES std_msgs
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pose_ik_python_benchmarker.py
  scripts/ik_benchmarking_data_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
