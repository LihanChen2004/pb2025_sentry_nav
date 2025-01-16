# Copyright 2025 Lihan Chen
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


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the package directory
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    joy_vel = LaunchConfiguration("joy_vel")
    joy_dev = LaunchConfiguration("joy_dev")
    joy_config_file = LaunchConfiguration("joy_config_file")
    publish_stamped_twist = LaunchConfiguration("publish_stamped_twist")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=joy_config_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_joy_vel_cmd = DeclareLaunchArgument(
        "joy_vel",
        default_value="cmd_vel",
        description="The topic to publish velocity commands to",
    )

    declare_joy_config_file_cmd = DeclareLaunchArgument(
        "joy_config_file",
        default_value=os.path.join(
            bringup_dir, "config", "simulation", "nav2_params.yaml"
        ),
        description="The joystick configuration file path",
    )

    declare_joy_dev_cmd = DeclareLaunchArgument(
        "joy_dev", default_value="0", description="The joystick device ID"
    )

    declare_publish_stamped_twist_cmd = DeclareLaunchArgument(
        "publish_stamped_twist",
        default_value="false",
        description="Whether to publish stamped twist messages",
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "device_id": joy_dev,
                        "deadzone": 0.3,
                        "autorepeat_rate": 20.0,
                    }
                ],
            ),
            Node(
                package="pb_teleop_twist_joy",
                executable="pb_teleop_twist_joy_node",
                name="pb_teleop_twist_joy_node",
                output="screen",
                parameters=[
                    configured_params,
                    {
                        "publish_stamped_twist": publish_stamped_twist,
                    },
                ],
                remappings=[
                    ("/cmd_vel", joy_vel),
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_joy_vel_cmd)
    ld.add_action(declare_joy_config_file_cmd)
    ld.add_action(declare_joy_dev_cmd)
    ld.add_action(declare_publish_stamped_twist_cmd)

    # Add the actions to launch the nodes
    ld.add_action(bringup_cmd_group)

    return ld
