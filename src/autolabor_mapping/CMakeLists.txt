cmake_minimum_required(VERSION 3.8)
project(autolabor_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# PCD -> 2D 占据栅格地图(离线工具)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcd_to_grid src/pcd_to_grid.cpp)
target_link_libraries(pcd_to_grid ${PCL_LIBRARIES})

# Livox PointCloud2 -> CustomMsg 转换节点(供 FAST-LIO 建图)
add_executable(pc2_to_custom src/pc2_to_custom.cpp)
ament_target_dependencies(pc2_to_custom rclcpp sensor_msgs livox_ros_driver2)

# FAST-LIO /Odometry -> odom->base_link TF 转发节点(导航用)
add_executable(lio_tf_relay src/lio_tf_relay.cpp)
ament_target_dependencies(lio_tf_relay rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS pcd_to_grid pc2_to_custom lio_tf_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
