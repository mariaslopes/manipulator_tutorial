cmake_minimum_required(VERSION 3.8)
project(manipulator_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

set(DEPENDENCIES
  ament_cmake
  moveit_core
  moveit_ros_planning
  moveit_task_constructor_core
  rclcpp
)

add_executable(task_node src/task_node.cpp)
ament_target_dependencies(task_node ${DEPENDENCIES})

install(TARGETS task_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/scene_to_tf.py
  src/ik_request.py
  src/fk_request.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
