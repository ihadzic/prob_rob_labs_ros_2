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
from launch.actions import AppendEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tb3_launch_dir = os.path.join(get_package_share_directory(
        'turtlebot3_gazebo'), 'launch')
    prob_rob_labs_dir = get_package_share_directory('prob_rob_labs')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    world = LaunchConfiguration('world', default='empty.world')
    use_noisy_dynamics = LaunchConfiguration('use_noisy_dynamics', default='true')
    use_actuation_noise = LaunchConfiguration('use_actuation_noise', default='true')
    sensor_noise_scale = LaunchConfiguration('sensor_noise_scale', default='2.0')
    wheel_slip = LaunchConfiguration('wheel_slip', default='0.06')
    cmd_noise = LaunchConfiguration('cmd_noise', default='0.0')

    world_path = PathJoinSubstitution([
        FindPackageShare('prob_rob_labs'),
        'worlds',
        world
    ])

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='Name of the world file (located in prob_rob_labs/worlds/)'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_noisy_dynamics_arg = DeclareLaunchArgument(
        'use_noisy_dynamics',
        default_value='true',
        description='Enable stronger sensor noise and wheel slip'
    )

    declare_use_actuation_noise_arg = DeclareLaunchArgument(
        'use_actuation_noise',
        default_value='true',
        description='Inject cmd_vel noise and idle jitter'
    )

    declare_sensor_noise_scale_arg = DeclareLaunchArgument(
        'sensor_noise_scale',
        default_value='2.0',
        description='Multiplier for built-in sensor noise values'
    )

    declare_wheel_slip_arg = DeclareLaunchArgument(
        'wheel_slip',
        default_value='0.06',
        description='Slip compliance used by the Gazebo wheel-slip system'
    )

    declare_cmd_noise_arg = DeclareLaunchArgument(
        'cmd_noise',
        default_value='0.0',
        description='Velocity command noise'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', world_path], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                     'models'))

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(prob_rob_labs_dir, 'launch', 'spawn_turtlebot3_noisy_launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_noisy_dynamics': use_noisy_dynamics,
            'use_actuation_noise': use_actuation_noise,
            'sensor_noise_scale': sensor_noise_scale,
            'wheel_slip': wheel_slip,
            'cmd_noise': cmd_noise
        }.items()
    )

    gz_pose_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('prob_rob_labs'),
                         'launch', 'gz_pose_publisher_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    gz_twist_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('prob_rob_labs'),
                         'launch', 'gz_twist_publisher_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()

    ld.add_action(set_env_vars_resources)
    ld.add_action(declare_world_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_use_noisy_dynamics_arg)
    ld.add_action(declare_use_actuation_noise_arg)
    ld.add_action(declare_sensor_noise_scale_arg)
    ld.add_action(declare_wheel_slip_arg)
    ld.add_action(declare_cmd_noise_arg)
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(gz_pose_publisher_cmd)
    ld.add_action(gz_twist_publisher_cmd)

    return ld
