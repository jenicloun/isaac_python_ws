cmake_minimum_required(VERSION 3.8)
project(mapf_nav_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mapf_msgs REQUIRED)

add_executable(path_executor src/path_executor.cpp)
ament_target_dependencies(path_executor
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  mapf_msgs
)
target_compile_features(path_executor PUBLIC cxx_std_17)
set_target_properties(path_executor PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(path_executor
  m
  stdc++
)

add_executable(path_follower src/path_follower.cpp)
ament_target_dependencies(path_follower
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
  mapf_msgs
)
target_compile_features(path_follower PUBLIC cxx_std_17)
set_target_properties(path_follower PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(path_follower
  stdc++
)

install(TARGETS
  path_executor
  path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
