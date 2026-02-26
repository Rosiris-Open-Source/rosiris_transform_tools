cmake_minimum_required(VERSION 3.8)
project(transform_rviz_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(THIS_PACKAGE_ROS_DEPENDENCIES
  geometry_msgs
  interactive_markers
  pluginlib
  rclcpp
  rviz_common
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_ROS_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

set(CMAKE_AUTOMOC ON)

set(plugin_headers
  include/transform_rviz_plugin/transform_rviz_plugin.hpp
)

add_library(${PROJECT_NAME} SHARED
  ${plugin_headers}
  src/transform_rviz_plugin.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  interactive_markers::interactive_markers
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  tf2_ros::tf2_ros
  tf2::tf2
  ${geometry_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  Qt5::Widgets
)

# Export the plugin
pluginlib_export_plugin_description_file(
  rviz_common plugin_description.xml
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()