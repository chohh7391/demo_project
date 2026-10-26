# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(demo_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# The build type comes from the command line (-DCMAKE_BUILD_TYPE=Release), as
# for the core's controllers. Never -Ofast: it folds isfinite() to a constant,
# and the allFinite() guard before the command write relies on it.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cho_interfaces REQUIRED)
find_package(cho_controller_common REQUIRED)
find_package(cho_controller_base REQUIRED)
# FrankaBaseController: the installed package exports its include directory and
# libcho_controller_franka.so the classic way, without CMake targets, and not
# every package its headers include, which are found above.
find_package(cho_controller_franka REQUIRED)

generate_parameter_library(demo_sine_joint_controller_parameters
  src/sine_joint_controller_parameters.yaml)

add_library(${PROJECT_NAME} SHARED src/sine_joint_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  cho_controller_franka
  cho_controller_base
  cho_controller_common
  cho_interfaces
  control_msgs
  controller_interface
  hardware_interface
  pinocchio
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
)
target_link_libraries(${PROJECT_NAME} demo_sine_joint_controller_parameters)

pluginlib_export_plugin_description_file(controller_interface demo_controller.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The sweep itself, without ROS (include/demo_controller/sine_reference.hpp).
  ament_add_gtest(test_sine_reference test/test_sine_reference.cpp)
  target_include_directories(test_sine_reference PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
