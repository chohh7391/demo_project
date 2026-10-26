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

"""The registry a sourced workspace reads is the core's, plus sine_joint_controller.

Run against the install (colcon test sources it), so CHO_ROBOT_CONFIG_ADDITIONS
is what the environment hook set.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cho_robot_config import ADDITIONS_ENV, load_robot_config
from cho_task_manager.utils.controller_names import (
    exclusive_arm_controllers,
    load_robot_config as task_manager_config,
)
import yaml

ADDITIONS = Path(get_package_share_directory('demo_bringup')) / 'config' / 'robot_config_additions.yaml'


def test_sourcing_the_workspace_lists_the_projects_additions():
    listed = [Path(entry).resolve() for entry in os.environ[ADDITIONS_ENV].split(os.pathsep) if entry]
    assert ADDITIONS.resolve() in listed


def test_every_exclusive_switch_takes_the_projects_controller_down():
    added = yaml.safe_load(ADDITIONS.read_text())['franka']['additional_arm']
    assert load_robot_config('franka')['controllers']['additional_arm'][-len(added):] == added
    exclusive = exclusive_arm_controllers(task_manager_config('franka'))
    for name in added:
        assert name in exclusive, (name, exclusive)
