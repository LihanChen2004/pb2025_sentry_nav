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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    lidar_frame = LaunchConfiguration("lidar_frame")
    base_frame = LaunchConfiguration("base_frame")
    robot_base_frame = LaunchConfiguration("robot_base_frame")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the node"
    )

    declare_lidar_frame = DeclareLaunchArgument(
        "lidar_frame",
        default_value="front_mid360",
        description="Frame ID for LiDAR sensor",
    )

    declare_base_frame = DeclareLaunchArgument(
        "base_frame",
        default_value="chassis",
        description="Frame ID for Vehicle Base",
    )

    declare_robot_base_frame = DeclareLaunchArgument(
        "robot_base_frame",
        default_value="gimbal_yaw",
        description="Frame ID for Gimbal",
    )

    start_sensor_scan_generation = Node(
        package="sensor_scan_generation",
        executable="sensor_scan_generation_node",
        namespace=namespace,
        output="screen",
        remappings=remappings,
        parameters=[
            {"lidar_frame": lidar_frame},
            {"base_frame": base_frame},
            {"robot_base_frame": robot_base_frame},
        ],
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_namespace)
    ld.add_action(declare_lidar_frame)
    ld.add_action(declare_base_frame)
    ld.add_action(declare_robot_base_frame)
    ld.add_action(start_sensor_scan_generation)

    return ld
