cmake_minimum_required(VERSION 3.8)
project(swarm_data)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(std_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS
    src/robot_spawner.py
    src/obstacle_spawner.py
    src/autopilot.py
    src/dataset_create.py
    src/swarm_transformer.py
    src/gossip_manager.py
    src/formation_command.py
    src/expert_controller.py
    
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
