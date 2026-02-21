cmake_minimum_required(VERSION 3.10.2)
project(gz_attach_links)

# ROS 2
find_package(ament_cmake REQUIRED)

# Gazebo Harmonic (ROS 2 Jazzy pairing)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)

# Needed for resource paths (models, plugins)
ament_environment_hooks("hooks/resource_paths.dsv.in")

add_definitions(-DGZ_NS=gz)

add_library(attach-links SHARED
  src/attach_links/AttachLinks.cc
)

target_include_directories(attach-links PRIVATE
  ${gz-plugin2_INCLUDE_DIRS}
  ${gz-sim8_INCLUDE_DIRS}
  ${gz-common5_INCLUDE_DIRS}
)

target_link_libraries(attach-links
  gz-plugin2::core
  gz-sim8::core
  gz-common5::core
)

install(
  TARGETS attach-links
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS src/unroll_loops
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
