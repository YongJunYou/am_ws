cmake_minimum_required(VERSION 3.8)
project(mani_controller)

# Generate compile_commands.json for clangd/IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#-------------------------------#
# Compiler / C++ standard
#-------------------------------#
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#-------------------------------#
# Dependencies
#-------------------------------#
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dynamixel_sdk REQUIRED)  # <= 추가
find_package(pinocchio REQUIRED)

# include directory
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

#-------------------------------#
# Library (Controller Components)
#-------------------------------#
add_library(mani_controller_lib
  src/dynamixel_sdk_interface.cpp
  src/gravitational_moment_calculator.cpp
  src/mani.cpp
)

ament_target_dependencies(mani_controller_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  dynamixel_sdk        # <= 추가
  pinocchio
  Eigen3
)

#-------------------------------#
# Executable Node
#-------------------------------#
add_executable(mani_node src/mani_node.cpp)

target_link_libraries(mani_node
  mani_controller_lib
)

ament_target_dependencies(mani_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

#-------------------------------#
# Install
#-------------------------------#
install(TARGETS
  mani_controller_lib
  mani_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

#-------------------------------#
# ament package
#-------------------------------#
ament_package()
