#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool
#
# Adapted from turtlebot3_gazebo package
# Ilija Hadzic <ih2435@columbia.edu>
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='door.world')
    x_pose = LaunchConfiguration('x_pose', default='-1.5')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    run_door_opener = LaunchConfiguration('run_door_opener', default='false')

    run_vision_processor = LaunchConfiguration('run_vision_processor', default='false')
    max_vision_features = LaunchConfiguration('max_vision_features', default='200')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Set to true for simulation',
    )

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='door.world',
        description='Name of the world file (located in prob_rob_labs/worlds/)'
    )

    declare_run_door_opener_arg = DeclareLaunchArgument(
        'run_door_opener',
        default_value='false',
        description='Whether to run the flaky door opener node'
    )

    declare_run_vision_processor_arg = DeclareLaunchArgument(
        'run_vision_processor',
        default_value='false',
        description='Whether to run the vision processing nodes'
    )

    declare_max_vision_features_arg = DeclareLaunchArgument(
        'max_vision_features',
        default_value='200',
        description='Maximum number of features for the vision processor'
    )

    turtlebot3_world_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('prob_rob_labs'),
                         'launch', 'turtlebot3_world_launch.py')
        ),
        launch_arguments={
            'world': world,
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    door_torque_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='door_torque_bridge',
        arguments=['/model/hinged_glass_door/joint/hinge/cmd_force@std_msgs/msg/Float64]gz.msgs.Double'],
        remappings=[
            ('/model/hinged_glass_door/joint/hinge/cmd_force', '/hinged_glass_door/torque')
        ],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    flaky_door_opener_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('prob_rob_labs'), 'launch', 'flaky_door_opener_launch.py')
        ),
        condition=IfCondition(run_door_opener)
    )

    video_processor_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('prob_rob_vision'), 'launch', 'video_processor_launch.py')
        ),
        launch_arguments={
            'max_features': max_vision_features
        }.items(),
        condition=IfCondition(run_vision_processor)
    )

    image_mean_feature_x_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('prob_rob_labs'), 'launch', 'image_mean_feature_x_launch.py')
        ),
        condition=IfCondition(run_vision_processor)
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_world_arg)
    ld.add_action(declare_run_door_opener_arg)
    ld.add_action(declare_run_vision_processor_arg)
    ld.add_action(declare_max_vision_features_arg)
    ld.add_action(turtlebot3_world_launch_cmd)
    ld.add_action(door_torque_bridge_cmd)
    ld.add_action(flaky_door_opener_cmd)
    ld.add_action(video_processor_cmd)
    ld.add_action(image_mean_feature_x_cmd)

    return ld
