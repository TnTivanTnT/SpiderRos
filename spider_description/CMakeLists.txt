cmake_minimum_required(VERSION 3.8)
project(spider_description)

find_package(ament_cmake REQUIRED)

# Install data directories (URDF, launch, config, etc.)
install(
  DIRECTORY urdf launch config worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executable scripts
install(
  PROGRAMS
    nodes/spider_teleop_key.py
    nodes/spider_ik_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
