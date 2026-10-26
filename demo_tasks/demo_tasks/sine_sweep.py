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

"""The demo project's task: sweep a joint with the project's own controller.

On Franka, position mode, every step but one from the core:

1. switch to the hold controller and go to the registry's task home
   (``home_subtree``);
2. switch to ``sine_joint_controller`` (demo_controller), which sweeps one
   joint about where the hold left it -- which joint, how far and how fast is
   the controller's, in demo_bringup's parameter file;
3. let it sweep for ``sweep_sec`` (``WaitBehavior``);
4. switch the hold controller back in, which takes over the moving command and
   brings it to rest, and go home again.

The root is ``guarded_mission``: a failure ends on the hold controller, and
that switch takes the sweep down because demo_bringup's registry additions list
it. Built against a registry without them, the tree refuses to build::

    ros2 launch demo_bringup bringup_mujoco.launch.py
    ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=franka \\
        task:=demo_sine_sweep control_mode:=position use_sim_time:=true \\
        task_params:=$(ros2 pkg prefix --share demo_tasks)/config/sine_sweep.yaml
"""

import math

import py_trees

from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.behaviors.wait import WaitBehavior
from cho_task_manager.subtrees import guarded_mission, home_joint_state, home_subtree
from cho_task_manager.utils.controller_names import (
    exclusive_arm_controllers,
    hold_controllers,
    resolve_control_mode,
)

TASK_NAME = 'demo_sine_sweep'
CONTROL_MODE = 'position'

#: What task_params may set, and its value when it does not.
DEFAULTS = {
    'controller': 'sine_joint_controller',  # a controller the registry additions name
    'sweep_sec': 10.0,                      # [s] the controller runs before the hold takes over
    'home_duration': 3.0,                   # [s] per home move, a minimum (cho_interfaces/CONTRACT.md)
}


def sine_sweep_params(robot_config):
    """The task's parameters: DEFAULTS, overridden by ``robot_config['task_params']``.

    Raises ValueError, which the node reports before anything moves, for a
    parameter it does not take or a value it cannot run.
    """
    given = robot_config['task_params']
    unknown = sorted(set(given) - set(DEFAULTS))
    if unknown:
        raise ValueError(f'{TASK_NAME} does not take {unknown}; it takes {sorted(DEFAULTS)}')
    params = {**DEFAULTS, **given}
    if not isinstance(params['controller'], str) or not params['controller']:
        raise ValueError(f"{TASK_NAME}: controller must be a name, got {params['controller']!r}")
    for key in ('sweep_sec', 'home_duration'):
        value = params[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{TASK_NAME}: {key} must be a number > 0, got {value!r}')
    # The switches below deactivate what the registry lists. A controller it
    # does not list would stay active through the abort to the hold, which
    # then fails on the claimed interfaces.
    if params['controller'] not in exclusive_arm_controllers(robot_config):
        raise ValueError(
            f"{TASK_NAME}: {params['controller']} is not one of the registry's arm controllers; "
            'source the workspace whose registry additions name it (CHO_ROBOT_CONFIG_ADDITIONS)')
    return params


def create_sine_sweep_tree(robot_config):
    """Home on the hold, sweep with the project's controller, home on the hold."""
    params = sine_sweep_params(robot_config)
    control_mode = resolve_control_mode(robot_config, default=CONTROL_MODE)
    hold = hold_controllers(robot_config, control_mode)[0]
    home = home_joint_state(robot_config)
    sweeper = params['controller']

    mission = py_trees.composites.Sequence(name='Demo_Sine_Sweep', memory=True)
    mission.add_children([
        home_subtree(robot_config, target_joints=home, controller=hold,
                     duration=params['home_duration'], open_gripper=False),
        SwitchControllerServiceBehavior(
            name=f'2_Switch_To_{sweeper}', activate=[sweeper], robot_config=robot_config),
        WaitBehavior(name='Sweep', duration_sec=params['sweep_sec']),
        home_subtree(robot_config, target_joints=home, controller=hold,
                     duration=params['home_duration'], name='3_Finish', suffix='_Final',
                     open_gripper=False),
    ])
    return guarded_mission(mission, robot_config, control_mode, name='Demo_Sine_Sweep_Root')


#: The registration setup.py points at: {robot_type: tree builder}.
TASK = {'franka': create_sine_sweep_tree}
