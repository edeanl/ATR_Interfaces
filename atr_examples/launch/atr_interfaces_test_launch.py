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
    bringup_dir = get_package_share_directory('atr_examples')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Define arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument('rviz_config_file', default_value=os.path.join(
        bringup_dir, 'rviz', 'atrScenario.rviz'), description='Full path to the RVIZ config file to use')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # RQT
    rqt_graph_node = Node(package='rqt_graph',
                          executable='rqt_graph', output='screen')

    # Rviz node
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    #-------- NODES

    # ATR Formation List Server
    atr_formation_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'atr_formation_list.yaml'
    )
    atr_formation_list_server = Node(package='atr_examples',
                                     name='atr_formation_list_server',
                                     executable='atr_formation_list_server',
                                     parameters=[atr_formation_yaml],
                                     output='screen')

    # Object List Publisher (NONA objects)
    nona_generator_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'nona_generator.yaml'
    )
    nona_list_pub = Node(package='atr_examples',
                         name='nona_generator',
                         executable='nona_generator',
                         parameters=[nona_generator_yaml],
                         output='screen')

    # ObjectList publisher (This is the semantic_segmentation module with the Client/Service option)
    # obj_list_pub = Node(package='atr_examples',
    #                     name='object_list_publisher',
    #                     executable='object_list_publisher', output='screen')
    # Object List Publisher (Static and Dynamic Objects)
    semantic_segmentation_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'semantic_segmentation.yaml'
    )
    semantic_segmentation = Node(package='atr_examples',
                                 name='semantic_segmentation',
                                 executable='semantic_segmentation',
                                 parameters=[semantic_segmentation_yaml],
                                 output='screen')

    # Predicted Object Node with Server/Client interface
    # obj_list_srv = Node(package='atr_examples',
    #                     name='object_list_server',
    #                     executable='object_list_server',
    #                     parameters=[atr_objects_yaml],
    #                     output='screen')

    # Subscribes to the Object List topic and generates messages for visualizing the obj in rviz
    object_list_subscriber_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'object_list_subscriber.yaml'
    )
    obj_list_sub = Node(package='atr_examples',
                        name='object_list_subscriber',
                        executable='object_list_subscriber',
                        parameters=[object_list_subscriber_yaml],
                        output='screen')

    # ATR state publishers, simulates the ATR (ATR Control)
    # In this case we have two ATRs
    atr_state_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'atr_parameters.yaml'
    )
    atr_state_pub1 = Node(
        package='atr_examples',
        name='atr1_state_publisher',
        executable='atr_state_publisher',
        parameters=[atr_state_yaml],
        output='screen'
    )

    atr_state_pub2 = Node(
        package='atr_examples',
        name='atr2_state_publisher',
        executable='atr_state_publisher',
        parameters=[atr_state_yaml],
        output='screen'
    )

    # ATR state list publisher. This node simulates ATR Tracker.
    atr_tracker_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'atr_tracker.yaml'
    )
    atr_tracker = Node(
        package='atr_examples',
        name='atr_tracker',
        executable='atr_state_list_publisher',
        parameters=[atr_tracker_yaml],
        output='screen'
    )

    # ATR state list subscriber. It publishes the ATR as MarkerArray
    atr_state_list_subscriber_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'atr_state_list_subscriber.yaml'
    )
    atr_state_list_subscriber = Node(
        package='atr_examples',
        name='atr_state_list_subscriber',
        executable='atr_state_list_subscriber',
        parameters=[atr_state_list_subscriber_yaml],
        output='screen'
    )

    # ObjectList publisher
    atr_objects_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'atr_predicted_objects.yaml'
    )
    # Dynamic Obstacle predictor
    dyn_obs_pred = Node(package='atr_examples',
                        name='dynamic_obstacle_predictor',
                        executable='dynamic_obstacle_predictor',
                        parameters=[atr_objects_yaml],
                        output='screen')

    # Predicted Objects Subscriber (Transforms PredObj msg to MarkerArray)
    pred_object_list_subscriber_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'pred_object_list_subscriber.yaml'
    )
    pred_obj_list_sub = Node(package='atr_examples',
                             name='pred_object_list_subscriber',
                             executable='pred_object_list_subscriber',
                             parameters=[pred_object_list_subscriber_yaml],
                             output='screen')

    # ATR Trajectory Generator
    atr_trajectory_generator_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'atr_trajectory_generator.yaml'
    )
    atr_trajectory_generator = Node(package='atr_examples',
                                    name='atr_trajectory_generator',
                                    executable='atr_trajectory_generator',
                                    parameters=[atr_trajectory_generator_yaml],
                                    output='screen')

    # ATR Path List subscriber (transforms the ATRPathList topic to MarkerArray to visualize it in Rviz)
    atr_path_list_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'atr_path_list.yaml'
    )
    atr_path_list_subs_tg = Node(package='atr_examples',
                                 name='atr_path_list_subscriber_tg',
                                 executable='atr_path_list_subscriber',
                                 parameters=[atr_path_list_yaml],
                                 output='screen')
    atr_path_list_subs_fc = Node(package='atr_examples',
                                 name='atr_path_list_subscriber_fc',
                                 executable='atr_path_list_subscriber',
                                 parameters=[atr_path_list_yaml],
                                 output='screen')

    # ATR Fleet Control
    atr_fleet_ctrl_yaml = os.path.join(
        get_package_share_directory('atr_examples'),
        'config',
        'atr_fleet_ctrl.yaml'
    )
    atr_fleet_control = Node(package='atr_examples',
                             name='atr_fleet_control',
                             executable='atr_fleet_control',
                             parameters=[atr_fleet_ctrl_yaml],
                             output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Load nodes (actions)
    # ld.add_action(rqt_graph_node)
    ld.add_action(rviz_cmd)
    ld.add_action(atr_formation_list_server)
    ld.add_action(nona_list_pub)
    ld.add_action(semantic_segmentation)
    ld.add_action(obj_list_sub)
    ld.add_action(atr_state_pub1)
    ld.add_action(atr_state_pub2)
    ld.add_action(atr_tracker)
    ld.add_action(atr_state_list_subscriber)
    # # ld.add_action(obj_list_srv)
    ld.add_action(dyn_obs_pred)
    ld.add_action(pred_obj_list_sub)
    ld.add_action(atr_trajectory_generator)
    ld.add_action(atr_path_list_subs_tg)
    ld.add_action(atr_path_list_subs_fc)
    ld.add_action(atr_fleet_control)

    return ld
