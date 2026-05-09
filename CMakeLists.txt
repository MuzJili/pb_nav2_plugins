cmake_minimum_required(VERSION 3.5)
project(pb_nav2_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(yaml-cpp REQUIRED)

ament_auto_add_library(pb_back_up_frees_space_behavior SHARED
  src/behaviors/back_up_free_space.cpp
)

ament_auto_add_library(pb_nav2_costmap_layers SHARED
  src/layers/intensity_voxel_layer.cpp
  src/layers/semantic_polygon_layer.cpp
  src/terrain_semantic_zones.cpp
)

target_link_libraries(pb_nav2_costmap_layers
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
)

add_executable(terrain_zone_monitor
  src/terrain_zone_monitor.cpp
  src/terrain_semantic_zones.cpp
)

target_include_directories(terrain_zone_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(terrain_zone_monitor
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

if(TARGET yaml-cpp)
  target_link_libraries(pb_nav2_costmap_layers yaml-cpp)
  target_link_libraries(terrain_zone_monitor yaml-cpp)
else()
  target_include_directories(pb_nav2_costmap_layers PRIVATE ${YAML_CPP_INCLUDE_DIRS})
  target_include_directories(terrain_zone_monitor PRIVATE ${YAML_CPP_INCLUDE_DIRS})
  target_link_libraries(pb_nav2_costmap_layers ${YAML_CPP_LIBRARIES})
  target_link_libraries(terrain_zone_monitor ${YAML_CPP_LIBRARIES})
endif()

install(TARGETS terrain_zone_monitor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
