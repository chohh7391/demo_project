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

"""A pose above a latched one, for a TaskSpace goal to approach it from."""

import math

from geometry_msgs.msg import Pose
import py_trees

from cho_task_manager.utils.blackboard import TASK_NAMESPACE, TargetKey, write_client

#: The TCP pointing straight down (x, y, z, w): half a turn about x.
TOP_DOWN = (1.0, 0.0, 0.0, 0.0)


def standoff_pose(target, height, orientation=TOP_DOWN):
    """*target*'s position raised by *height* [m], with *orientation*: where to stop above it."""
    pose = Pose()
    pose.position.x = target.position.x
    pose.position.y = target.position.y
    pose.position.z = target.position.z + height
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    return pose


class StandoffPoseBehavior(py_trees.behaviour.Behaviour):
    """Write the pose *height* above the blackboard pose *source* under *record_as*.

    The source is a latched body pose (PoseTargetBehavior), in the robot's base
    frame; only its position is kept, so a box that has tipped over is still
    approached from above. FAILURE when the source is unset, which
    PoseTargetBehavior ahead of it rules out.
    """

    def __init__(self, name, source, record_as, height, namespace=TASK_NAMESPACE):
        super().__init__(name)
        if not (isinstance(height, (int, float)) and math.isfinite(height) and height > 0.0):
            raise ValueError(f'[{name}] height must be a positive number of metres, got {height!r}')
        self.height = float(height)
        self.record_as = record_as
        self.source = TargetKey(name, source, Pose, namespace)
        self.board = write_client(name, [record_as], namespace)
        self.node = None

    def setup(self, **kwargs):
        # TargetKey.read() logs through the node when the source is unusable.
        self.node = kwargs['node']
        return True

    def update(self):
        target = self.source.read(self)
        if target is None:
            return py_trees.common.Status.FAILURE
        setattr(self.board, self.record_as, standoff_pose(target, self.height))
        return py_trees.common.Status.SUCCESS
