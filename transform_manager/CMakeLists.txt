cmake_minimum_required(VERSION 3.20)
project(transform_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
set(PACKAGE_INCLUDE_DEPS
  generate_parameter_library
  geometry_msgs
  rclcpp
  rosiris_transform_interfaces
  tf2_geometry_msgs
  tf2_ros
)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

# parameters for transform_manager
generate_parameter_library(
  transform_manager_parameters 
  src/transform_manager_parameters.yaml 
)

# transform manager lib
add_library(transform_manager SHARED
  src/transform_manager.cpp
)
target_include_directories(transform_manager
  PUBLIC # when building package and anything that links against this
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> # building inside this package
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>  # define where consumers should include from 
)
target_link_libraries(transform_manager 
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${rosiris_transform_interfaces_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  transform_manager_parameters
)
install(
  TARGETS 
    transform_manager
    transform_manager_parameters # include parameters in export
  EXPORT export_transform_manager # create export group for transform manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include/
)
# export lib so other packages can link against
ament_export_targets(export_transform_manager HAS_LIBRARY_TARGET)
# propagate dependencies so downstream do not need to include deps
ament_export_dependencies(${PACKAGE_INCLUDE_DEPS})

# transform manager node
add_executable(transform_manager_node
  src/transform_manager_node.cpp
) 
target_link_libraries(transform_manager_node
  transform_manager
)
install(TARGETS
  transform_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# example launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
