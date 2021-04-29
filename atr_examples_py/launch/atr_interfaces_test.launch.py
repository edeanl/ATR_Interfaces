# file
#
#  \author Emmanuel Dean
#
#  \version 0.1
#  \date 15.03.2021
#
#  \copyright Copyright 2021 Chalmers
#
# Licence
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
# Acknowledgment
#   This project has received financial  support  from  Chalmers  AI  Re-search Centre
#   (CHAIR) and AB Volvo (Project ViMCoR).


# \file atr_interfaces_test_launch.py
# \brief Runs all the nodes to test the atr_interfaces.
# Provides the following functionalities:
#   Executes the nodes (emulated nodes):
#       ATR Tracker
#       Semantic Segmentation
#       NONA Generator
#       Dynamic Obstacle Prediction
#       ATR Trajectory Generator
#       ATR Fleet Control
#       ATR Control
#   Runs Rviz and RQT
#

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('atr_examples_py')
    launch_dir = os.path.join(bringup_dir, 'launch')

    #-------- NODES

    # ATR Formation List Server
    dummy_yaml = os.path.join(
        get_package_share_directory('atr_examples_py'),
        'config',
        'dummy.yaml'
    )
    dummy_app = Node(package='atr_examples_py',
                     name='dummy_node',
                     executable='dummy_node',
                     emulate_tty=True,
                     parameters=[dummy_yaml],
                     output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Load nodes (actions)
    ld.add_action(dummy_app)

    return ld
