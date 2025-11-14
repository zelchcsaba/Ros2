cmake_minimum_required(VERSION 3.8)
project(my_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(my_controller SHARED src/my_controller.cpp)

add_library(my_controller::my_controller ALIAS my_controller)

target_include_directories(my_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/my_controller>
)

ament_target_dependencies(
  my_controller
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include/my_controller
)

install(
  TARGETS my_controller
  EXPORT export_my_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_include_directories("include/my_controller")

ament_export_libraries(my_controller)

ament_export_targets(export_my_controller)

pluginlib_export_plugin_description_file(controller_interface my_controller.xml)

install(
  DIRECTORY launch config
  DESTINATION share/my_controller
)

ament_package()
