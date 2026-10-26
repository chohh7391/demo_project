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

"""The demo project in MuJoCo: the core's Franka bringup in the demo's world, plus sine_joint_controller.

    ros2 launch demo_bringup bringup_mujoco.launch.py

Includes cho_bringup_franka's bringup_mujoco_robot unchanged, in the position
control mode (the only one sine_joint_controller runs in), with
``controller_name`` its first active controller, and gives it the demo's
world (worlds/bench.xml, a bench and a box) and plugins
(config/mujoco/plugins.yaml: the core's BodyPosePublisher, the box's pose on
/sim/body_pose/box). Then it loads sine_joint_controller inactive, its type and
parameters given to the spawner, so the core's controller list and config need
no entry for it. The project's registry additions
(config/robot_config_additions.yaml) are what make every exclusive switch take
it down again.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SINE_CONTROLLER = 'sine_joint_controller'
SINE_CONTROLLER_TYPE = 'demo_controller/SineJointController'
CORE_LAUNCH = os.path.join(
    get_package_share_directory('cho_bringup_franka'), 'launch', 'bringup_mujoco_robot.launch.py')


def generate_launch_description():
    share = get_package_share_directory('demo_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_name', default_value='joint_space_position_controller',
            description="The core bringup's controller_name:=, a position-mode controller"),
        DeclareLaunchArgument(
            'load_gripper', default_value='true',
            description="The core bringup's load_gripper:="),
        DeclareLaunchArgument(
            'sine_params', default_value=os.path.join(share, 'config', 'mujoco', 'sine_joint_controller.yaml'),
            description='sine_joint_controller parameter file'),
        DeclareLaunchArgument(
            'mujoco_world', default_value=os.path.join(share, 'worlds', 'bench.xml'),
            description="The core bringup's mujoco_world:=, the world the FR3 stands in; empty is the "
                        "core's own floor"),
        DeclareLaunchArgument(
            'mujoco_plugins', default_value=os.path.join(share, 'config', 'mujoco', 'plugins.yaml'),
            description="The core bringup's mujoco_plugins:=, the plugins loaded into the simulation"),
        DeclareLaunchArgument(
            'controller_manager_timeout', default_value='60',
            description='How long the sine spawner waits for the controller_manager [s]'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(CORE_LAUNCH),
            launch_arguments={
                'control_mode': 'position',
                'controller_name': LaunchConfiguration('controller_name'),
                'load_gripper': LaunchConfiguration('load_gripper'),
                'mujoco_world': LaunchConfiguration('mujoco_world'),
                'mujoco_plugins': LaunchConfiguration('mujoco_plugins'),
            }.items(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                SINE_CONTROLLER,
                '--controller-type', SINE_CONTROLLER_TYPE,
                '--param-file', LaunchConfiguration('sine_params'),
                '--inactive',
                '--controller-manager-timeout', LaunchConfiguration('controller_manager_timeout'),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
