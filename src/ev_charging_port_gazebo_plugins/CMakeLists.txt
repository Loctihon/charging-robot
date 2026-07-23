cmake_minimum_required(VERSION 3.8)
project(ev_charging_port_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(cover_open_plugin SHARED
  src/cover_open_plugin.cpp
)

target_include_directories(cover_open_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cover_open_plugin
  gazebo_dev
  gazebo_ros
  rclcpp
  std_msgs
)

# Installed to bare lib/ (not lib/${PROJECT_NAME}) so the workspace's
# LD_LIBRARY_PATH picks it up and the SDF can reference it by name alone
# (filename="libcover_open_plugin.so"), matching the standard
# ros-gazebo-classic plugin-packaging convention.
install(TARGETS cover_open_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY worlds launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(cover_open_plugin)
ament_package()
