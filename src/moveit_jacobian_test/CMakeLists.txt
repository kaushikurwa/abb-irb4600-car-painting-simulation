cmake_minimum_required(VERSION 3.8)
project(moveit_jacobian_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(moveit_jacobian src/moveit_jacobian.cpp)

ament_target_dependencies(moveit_jacobian
  rclcpp
  moveit_core
  moveit_ros_planning
)

target_link_libraries(moveit_jacobian Eigen3::Eigen)

install(TARGETS moveit_jacobian
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
