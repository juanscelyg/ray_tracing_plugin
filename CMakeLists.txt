cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(ray_tracing_plugin)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
# Gazebo Harmonic, the version of ROS 2 Jazzy (gz_*_vendor packages). The versioned names are
# used so that another Gazebo installed in the system (e.g. Jetty in /usr) is not picked up: a
# plugin built against another gz-sim crashes the simulator when it is loaded.
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(${PROJECT_NAME} SHARED 
  src/ray_tracing_plugin.cpp)

install(DIRECTORY include/
        DESTINATION include)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
  PRIVATE
    gz-plugin2::register)

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

install(DIRECTORY config launch maps worlds rviz
    DESTINATION share/${PROJECT_NAME}
)

# Not installed in lib/${PROJECT_NAME}: a folder with the name of the plugin there is found
# by Gazebo before libray_tracing_plugin.so and the plugin fails to load
install(PROGRAMS scripts/save_pointcloud.py
    DESTINATION share/${PROJECT_NAME}/scripts
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_export_dependencies(
  gz_sim_vendor
  gz_plugin_vendor
  gz_transport_vendor
  gz_msgs_vendor
)

ament_package()