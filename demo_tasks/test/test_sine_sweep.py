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

"""The project's task, checked as cho_task_manager checks its own.

Run against the install (colcon test sources it): the task is found through the
entry point setup.py declares, and the registry is the core's plus
demo_bringup's additions, as in a terminal that sourced the workspace.
"""

import os

from cho_robot_config import ADDITIONS_ENV
from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.tasks import available_tasks
from cho_task_manager.testing import goal_problems, registry_tree
import pytest
import yaml

TASK = 'demo_sine_sweep'
SWEEPER = 'sine_joint_controller'
HOLD = 'joint_space_position_controller'
EXAMPLE_PARAMS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              'config', 'sine_sweep.yaml')


def _switches(tree):
    return [node for node in tree.iterate() if isinstance(node, SwitchControllerServiceBehavior)]


def test_the_task_is_registered_for_franka_only():
    assert TASK in available_tasks('franka')
    assert TASK not in available_tasks('ur5e')


@pytest.mark.parametrize('task_params', [{}, yaml.safe_load(open(EXAMPLE_PARAMS))])
def test_every_goal_names_the_registry_joints(task_params):
    tree, config = registry_tree(TASK, 'franka', task_params=task_params)
    assert goal_problems(tree, config) == []


def test_it_homes_sweeps_and_homes_on_the_hold():
    tree, _config = registry_tree(TASK, 'franka')
    mission = next(node for node in tree.iterate() if node.name == 'Demo_Sine_Sweep')
    assert [child.name for child in mission.children] == [
        '1_Initialize', f'2_Switch_To_{SWEEPER}', 'Sweep', '3_Finish']
    switches = {node.name: node for node in _switches(mission)}
    assert switches[f'Switch_To_{HOLD}'].activate == [HOLD]
    # Each switch takes the other down, which only the registry additions make possible.
    assert HOLD in switches[f'2_Switch_To_{SWEEPER}'].deactivate
    assert SWEEPER in switches[f'Switch_To_{HOLD}_Final'].deactivate


def test_every_switch_to_the_hold_takes_the_sweep_down_the_abort_included():
    tree, _config = registry_tree(TASK, 'franka')
    to_hold = [node for node in _switches(tree) if HOLD in node.activate]
    # The two homes and guarded_mission's abort.
    assert len(to_hold) >= 3
    for node in to_hold:
        assert SWEEPER in node.deactivate, node.name


@pytest.mark.parametrize('task_params,message', [
    ({'sweep': 5.0}, "does not take \\['sweep'\\]"),
    ({'sweep_sec': 0.0}, 'sweep_sec must be a number > 0'),
    ({'home_duration': True}, 'home_duration must be a number > 0'),
    ({'controller': ''}, 'controller must be a name'),
    ({'controller': 'no_such_controller'}, "not one of the registry's arm controllers"),
])
def test_a_parameter_it_cannot_run_is_refused_before_anything_moves(task_params, message):
    with pytest.raises(ValueError, match=message):
        registry_tree(TASK, 'franka', task_params=task_params)


def test_without_the_registry_additions_the_tree_refuses_to_build(monkeypatch):
    monkeypatch.setenv(ADDITIONS_ENV, '')
    with pytest.raises(ValueError, match=f'{SWEEPER} is not one of the registry'):
        registry_tree(TASK, 'franka')
