cmake_minimum_required(VERSION 3.8)
project(autolabor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(autolabor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

add_executable(autolabor_driver src/autolabor_driver.cpp)
target_include_directories(autolabor_driver PRIVATE
  include
)
ament_target_dependencies(autolabor_driver
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(autolabor_driver
  Boost::system
  Threads::Threads
)

add_executable(sim_autolabor_driver src/sim_autolabor_driver.cpp)
target_include_directories(sim_autolabor_driver PRIVATE
  include
)
ament_target_dependencies(sim_autolabor_driver
  rclcpp
  autolabor_msgs
)
target_link_libraries(sim_autolabor_driver
  Boost::system
  Threads::Threads
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  autolabor_driver
  sim_autolabor_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_chassis_protocol test/test_chassis_protocol.cpp)
  target_include_directories(test_chassis_protocol PRIVATE
    include
  )

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
