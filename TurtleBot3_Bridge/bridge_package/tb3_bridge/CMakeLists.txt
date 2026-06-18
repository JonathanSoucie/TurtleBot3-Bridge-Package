cmake_minimum_required(VERSION 3.8)
project(tb3_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake     REQUIRED)
find_package(rclcpp          REQUIRED)
find_package(geometry_msgs   REQUIRED)
find_package(sensor_msgs     REQUIRED)
find_package(nav_msgs        REQUIRED)
find_package(std_srvs        REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(tb3_bridge_node src/tb3_bridge_node.cpp)

target_include_directories(tb3_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(tb3_bridge_node PUBLIC cxx_std_17)

ament_target_dependencies(tb3_bridge_node
  rclcpp geometry_msgs sensor_msgs nav_msgs std_srvs diagnostic_msgs
)

install(TARGETS tb3_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
