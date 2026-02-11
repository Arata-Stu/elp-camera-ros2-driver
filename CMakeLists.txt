cmake_minimum_required(VERSION 3.8)
project(elp-camera-ros2-driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# Create a shared library (component)
ament_auto_add_library(elp_camera_component SHARED
  src/elp_camera_node.cpp
  src/elp_camera_core.cpp
)

# Register the component
rclcpp_components_register_nodes(elp_camera_component "elp_camera::ElpCameraNode")

# Create the standalone executable
ament_auto_add_executable(elp_camera_node
  src/elp_camera_main.cpp
)

# Link the executable to the component library
target_link_libraries(elp_camera_node elp_camera_component)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)