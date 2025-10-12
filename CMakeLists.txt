cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(ray_tracing_plugin)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz-sim REQUIRED)
 
# Add sources for each plugin to be registered.
add_library(${PROJECT_NAME} SHARED 
  src/ray_tracing_plugin.cpp)

install(DIRECTORY include/
        DESTINATION include)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
  PRIVATE 
    gz-plugin::gz-plugin
    gz-sim::gz-sim)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch worlds
    DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

# ament_export_include_directories("include/${PROJECT_NAME}")
# ament_export_libraries(${PROJECT_NAME})
# ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  sensor_msgs
)

ament_package()