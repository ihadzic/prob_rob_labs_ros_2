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

def generate_launch_description():
    turtlebot3_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('prob_rob_labs'),
                         'launch', 'turtlebot3_world_launch.py')
        ),
        launch_arguments={
            'world': 'landmarks.world',
            'x_pose': '-1.5',
            'y_pose': '0.0',
            'cmd_noise': '0.005'
        }.items()
    )

    landmark_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('prob_rob_labs'),
                         'launch', 'landmark_vision_launch.py')
        )
    )

    return LaunchDescription([
        turtlebot3_world_launch,
        landmark_vision_launch
    ])
