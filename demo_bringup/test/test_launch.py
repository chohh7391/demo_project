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

"""The launch file loads, and the controller's name and type agree across the files."""

import importlib.util
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import yaml

SHARE = get_package_share_directory('demo_bringup')
LAUNCH = os.path.join(SHARE, 'launch', 'bringup_mujoco.launch.py')


def _launch_module():
    spec = importlib.util.spec_from_file_location('bringup_mujoco', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _text(substitutions):
    return ''.join(part.text for part in substitutions)


def test_it_includes_a_core_bringup_that_exists():
    assert os.path.isfile(_launch_module().CORE_LAUNCH)


def test_the_default_parameters_name_the_controller_the_registry_adds():
    module = _launch_module()
    arguments = {entity.name: entity for entity in module.generate_launch_description().entities
                 if isinstance(entity, DeclareLaunchArgument)}
    params_file = _text(arguments['sine_params'].default_value)
    with open(params_file) as stream:
        params = yaml.safe_load(stream)
    assert list(params) == [module.SINE_CONTROLLER]
    assert params[module.SINE_CONTROLLER]['ros__parameters']['bringup_type'] == 'mujoco'
    with open(os.path.join(SHARE, 'config', 'robot_config_additions.yaml')) as stream:
        assert yaml.safe_load(stream)['franka']['additional_arm'] == [module.SINE_CONTROLLER]


def test_the_controller_type_is_the_plugin_demo_controller_exports():
    plugins = ET.parse(os.path.join(get_package_share_directory('demo_controller'), 'demo_controller.xml'))
    names = [entry.get('name') for entry in plugins.getroot().iter('class')]
    assert _launch_module().SINE_CONTROLLER_TYPE in names


def test_the_plugins_publish_a_body_the_world_has_with_a_plugin_the_core_exports():
    module = _launch_module()
    arguments = {entity.name: entity for entity in module.generate_launch_description().entities
                 if isinstance(entity, DeclareLaunchArgument)}
    world = ET.parse(_text(arguments['mujoco_world'].default_value)).getroot()
    bodies = {body.get('name') for body in world.iter('body')}
    with open(_text(arguments['mujoco_plugins'].default_value)) as stream:
        plugins = yaml.safe_load(stream)['/**']['ros__parameters']['mujoco_plugins']
    exported = ET.parse(os.path.join(get_package_share_directory('cho_simulation_mujoco'), 'plugins.xml'))
    classes = {entry.get('name') for entry in exported.getroot().iter('class')}
    for name, plugin in plugins.items():
        assert plugin['type'] in classes, (name, plugin['type'], classes)
        assert set(plugin.get('bodies', [])) <= bodies, (name, plugin.get('bodies'), bodies)
