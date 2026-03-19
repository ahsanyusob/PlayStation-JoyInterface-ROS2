# Copyright 2023 HarvestX Inc.
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for Ackermann Teleop."""
    
    # 1. Declare Arguments
    hw_type_arg = DeclareLaunchArgument(
        'hw_type', default_value=TextSubstitution(text='DualShock3'))
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name', default_value=TextSubstitution(text='ackermann_drive'))
    
    publish_hz_arg = DeclareLaunchArgument(
        'publish_hz', default_value=TextSubstitution(text='50.0'))

    # Initial limits (can be modified at runtime via D-Pad)
    initial_max_speed_arg = DeclareLaunchArgument(
        'initial_max_speed', default_value=TextSubstitution(text='1.0'))
    
    initial_max_steer_deg_arg = DeclareLaunchArgument(
        'initial_max_steer_deg', default_value=TextSubstitution(text='30.0'))

    # 2. Define Container with Composable Nodes
    joy_container = ComposableNodeContainer(
        name='p9n_ackermann_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy_node',
                namespace='',
                parameters=[{
                    'deadzone': 0.05,
                    'autorepeat_rate': 20.0,
                }]
            ),
            ComposableNode(
                package='p9n_node',
                plugin='p9n_node::TeleopAckermannJoyNode',
                name='teleop_ackermann_joy_node',
                namespace='',
                parameters=[{
                        'hw_type': LaunchConfiguration('hw_type'),
                        'publish_hz': LaunchConfiguration('publish_hz'),
                        'initial_max_speed': LaunchConfiguration('initial_max_speed'),
                        'initial_max_steer_deg': LaunchConfiguration('initial_max_steer_deg')
                }],
                remappings=[
                    ('ackermann_drive', LaunchConfiguration('topic_name'))
                ],
            )
        ],
    )

    # 3. Assemble Launch Description
    ld = LaunchDescription()

    ld.add_action(hw_type_arg)
    ld.add_action(topic_name_arg)
    ld.add_action(publish_hz_arg)
    ld.add_action(initial_max_speed_arg)
    ld.add_action(initial_max_steer_deg_arg)

    ld.add_action(joy_container)

    return ld