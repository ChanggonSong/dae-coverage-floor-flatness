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
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    xacro_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf.xacro'
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    print('xacro_file_name : {}'.format(xacro_file_name))

    # xacro 파일 경로
    xacro_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        xacro_file_name)

    # xacro 파싱 — namespace arg는 기본값 '' (빈 문자열)로 전달
    robot_desc_xml = xacro.process_file(
        xacro_path,
        mappings={'namespace': ''}  # namespace 프로퍼티에 빈 문자열 할당
    ).toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc_xml,
                'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
            }],
        ),
    ])
