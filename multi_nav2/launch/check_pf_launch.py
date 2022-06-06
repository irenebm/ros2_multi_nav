# Copyright 2021 Irene Bandera Moreno
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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    start_robot_1 = Node(
        package='multi_nav2',
        executable='check_pf',
        output='screen',
        remappings=[('my_markers', '/robot1/markers'),
                    ('my_poses', '/robot1/poses')])

    start_robot_2 = Node(
        package='multi_nav2',
        executable='check_pf',
        output='screen',
        remappings=[('my_markers', '/robot2/markers'),
                    ('my_poses', '/robot2/poses')])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_1)
    ld.add_action(start_robot_2)

    return ld
