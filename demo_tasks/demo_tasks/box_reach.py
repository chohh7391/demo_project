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

"""The demo project's second task: read a body's pose out of the scene and stop above it.

On Franka, position mode, in the demo's world (demo_bringup worlds/bench.xml):

1. switch to the hold controller and go to the registry's task home;
2. latch the box's pose -- the simulation's ground truth, which the core's
   BodyPosePublisher plugin publishes on /sim/body_pose/box in fr3_link0
   (demo_bringup config/mujoco/plugins.yaml);
3. switch to task_space_ik_controller and move the TCP ``standoff`` above the
   box, pointing down;
4. switch the hold back in and go home.

The scene, its loose objects and their poses come from the project: the core
loads the world it is given and publishes what its plugin is configured with.
A task that needs a perceived pose instead reads cho_object_pose's topic the
same way::

    ros2 launch demo_bringup bringup_mujoco.launch.py
    ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=franka \\
        task:=demo_box_reach control_mode:=position use_sim_time:=true \\
        task_params:=$(ros2 pkg prefix --share demo_tasks)/config/box_reach.yaml
"""

import math

import py_trees

from cho_task_manager.behaviors.action import TaskSpaceActionBehavior
from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.behaviors.topic import PoseTargetBehavior
from cho_task_manager.subtrees import guarded_mission, home_joint_state, home_subtree
from cho_task_manager.utils.controller_names import (
    exclusive_arm_controllers,
    hold_controllers,
    resolve_control_mode,
)
from demo_tasks.standoff import StandoffPoseBehavior

TASK_NAME = 'demo_box_reach'
CONTROL_MODE = 'position'
#: Where BodyPosePublisher publishes a body's pose: its topic_prefix + the body.
BODY_POSE_TOPIC_PREFIX = '/sim/body_pose/'
BODY_POSE_KEY = 'body_pose'
STANDOFF_KEY = 'body_standoff'

#: What task_params may set, and its value when it does not.
DEFAULTS = {
    'body': 'box',                                # a body the plugin publishes
    'standoff': 0.15,                             # [m] above the body, where the TCP stops
    'task_controller': 'task_space_ik_controller',  # the position-mode task-space controller
    'move_duration': 4.0,                         # [s] per move, a minimum (cho_interfaces/CONTRACT.md)
    'pose_timeout': 5.0,                          # [s] to wait for the body's pose
}


def box_reach_params(robot_config):
    """The task's parameters: DEFAULTS, overridden by ``robot_config['task_params']``.

    Raises ValueError, which the node reports before anything moves, for a
    parameter it does not take or a value it cannot run.
    """
    given = robot_config['task_params']
    unknown = sorted(set(given) - set(DEFAULTS))
    if unknown:
        raise ValueError(f'{TASK_NAME} does not take {unknown}; it takes {sorted(DEFAULTS)}')
    params = {**DEFAULTS, **given}
    for key in ('body', 'task_controller'):
        if not isinstance(params[key], str) or not params[key]:
            raise ValueError(f'{TASK_NAME}: {key} must be a name, got {params[key]!r}')
    for key in ('standoff', 'move_duration', 'pose_timeout'):
        value = params[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{TASK_NAME}: {key} must be a number > 0, got {value!r}')
    if params['task_controller'] not in exclusive_arm_controllers(robot_config):
        raise ValueError(
            f"{TASK_NAME}: {params['task_controller']} is not one of the registry's arm controllers")
    return params


def create_box_reach_tree(robot_config):
    """Home on the hold, latch the body's pose, stop the TCP above it, home on the hold."""
    params = box_reach_params(robot_config)
    control_mode = resolve_control_mode(robot_config, default=CONTROL_MODE)
    hold = hold_controllers(robot_config, control_mode)[0]
    home = home_joint_state(robot_config)
    # The pose is checked in, and the goal stamped with, the frame the
    # registry's absolute TaskSpace goals are in: fr3_link0 for Franka, which
    # is the MuJoCo world's origin.
    base_frame = robot_config['arm_base_link']
    controller = params['task_controller']
    body = params['body']

    reach = py_trees.composites.Sequence(name=f'2_Reach_{body}', memory=True)
    reach.add_children([
        PoseTargetBehavior(
            name=f'Latch_{body}', record_as=BODY_POSE_KEY, topic=BODY_POSE_TOPIC_PREFIX + body,
            required_frame=base_frame, timeout_sec=params['pose_timeout']),
        StandoffPoseBehavior(
            name=f'Above_{body}', source=BODY_POSE_KEY, record_as=STANDOFF_KEY, height=params['standoff']),
        SwitchControllerServiceBehavior(
            name=f'Switch_To_{controller}', activate=[controller], robot_config=robot_config),
        TaskSpaceActionBehavior(
            name=f'Move_Above_{body}', target_pose_key=STANDOFF_KEY, controller_name=controller,
            duration=params['move_duration'], frame_id=base_frame),
    ])
    mission = py_trees.composites.Sequence(name='Demo_Box_Reach', memory=True)
    mission.add_children([
        home_subtree(robot_config, target_joints=home, controller=hold,
                     duration=params['move_duration'], open_gripper=False),
        reach,
        home_subtree(robot_config, target_joints=home, controller=hold,
                     duration=params['move_duration'], name='3_Finish', suffix='_Final',
                     open_gripper=False),
    ])
    return guarded_mission(mission, robot_config, control_mode, name='Demo_Box_Reach_Root')


#: The registration setup.py points at: {robot_type: tree builder}.
TASK = {'franka': create_box_reach_tree}
