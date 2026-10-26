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
project(demo_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})

# Sourcing the workspace adds config/robot_config_additions.yaml to
# CHO_ROBOT_CONFIG_ADDITIONS, which the core's registry loader reads.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robot_config_additions.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robot_config_additions.sh.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.flake8")
  ament_add_pytest_test(test_robot_config test/test_robot_config.py TIMEOUT 120)
  ament_add_pytest_test(test_launch test/test_launch.py TIMEOUT 120)
endif()

ament_package()
