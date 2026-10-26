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

"""demo_box_reach, checked as cho_task_manager checks its own, and the pose it approaches."""

import os

from cho_task_manager.behaviors.action import TaskSpaceActionBehavior
from cho_task_manager.behaviors.topic import PoseTargetBehavior
from cho_task_manager.tasks import available_tasks
from cho_task_manager.testing import goal_problems, registry_tree
from geometry_msgs.msg import Pose
import pytest
import yaml

from demo_tasks.box_reach import BODY_POSE_TOPIC_PREFIX
from demo_tasks.standoff import standoff_pose, StandoffPoseBehavior, TOP_DOWN

TASK = 'demo_box_reach'
EXAMPLE_PARAMS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              'config', 'box_reach.yaml')


def test_the_task_is_registered_for_franka_only():
    assert TASK in available_tasks('franka')
    assert TASK not in available_tasks('ur5e')


@pytest.mark.parametrize('task_params', [{}, yaml.safe_load(open(EXAMPLE_PARAMS))])
def test_every_goal_names_the_registry_frame_and_joints(task_params):
    tree, config = registry_tree(TASK, 'franka', task_params=task_params)
    assert goal_problems(tree, config) == []


def test_it_latches_the_bodys_pose_in_the_base_frame_and_moves_above_it():
    tree, config = registry_tree(TASK, 'franka', task_params={'body': 'cup', 'standoff': 0.2})
    nodes = list(tree.iterate())
    latch = next(node for node in nodes if isinstance(node, PoseTargetBehavior))
    assert latch.topic == BODY_POSE_TOPIC_PREFIX + 'cup'
    assert latch.required_frame == config['arm_base_link'] == 'fr3_link0'
    above = next(node for node in nodes if isinstance(node, StandoffPoseBehavior))
    assert above.height == 0.2
    move = next(node for node in nodes if isinstance(node, TaskSpaceActionBehavior))
    assert move.action_name == '/task_space_ik_controller/task_space'


def test_the_standoff_keeps_the_position_and_points_the_tcp_down():
    box = Pose()
    box.position.x, box.position.y, box.position.z = 0.45, 0.10, 0.125
    box.orientation.z, box.orientation.w = 0.38, 0.92    # a box turned on the bench
    pose = standoff_pose(box, 0.15)
    assert (pose.position.x, pose.position.y) == (0.45, 0.10)
    assert pose.position.z == pytest.approx(0.275)
    assert (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) == TOP_DOWN


@pytest.mark.parametrize('task_params,message', [
    ({'bodies': 'box'}, "does not take \\['bodies'\\]"),
    ({'standoff': 0.0}, 'standoff must be a number > 0'),
    ({'body': ''}, 'body must be a name'),
    ({'task_controller': 'no_such_controller'}, "not one of the registry's arm controllers"),
])
def test_a_parameter_it_cannot_run_is_refused_before_anything_moves(task_params, message):
    with pytest.raises(ValueError, match=message):
        registry_tree(TASK, 'franka', task_params=task_params)
