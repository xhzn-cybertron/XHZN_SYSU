#
#   Copyright (c)     
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#

import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    dlio_dir = launch.substitutions.LaunchConfiguration(
        'dlio_dir',
        default=os.path.join(
            get_package_share_directory('direct_lidar_inertial_odometry'),
            'cfg',
            'dlio.yaml'))
    params_dir = launch.substitutions.LaunchConfiguration(
        'params_dir',
        default=os.path.join(
            get_package_share_directory('direct_lidar_inertial_odometry'),
            'cfg',
            'params.yaml'))


    # DLIO Odometry Node
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=[dlio_dir, params_dir],
        remappings=[
            ('pointcloud', '/livox/lidar'),
            ('imu', '/livox/imu'),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ]
    )

    # DLIO Mapping Node
    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=[dlio_dir, params_dir],
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
        ]
    )    

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', (
            get_package_share_directory('direct_lidar_inertial_odometry'),
            'launch',
            'dlio.rviz')],
        output='screen'
    )
    
    ld.add_action(dlio_odom_node)
    ld.add_action(dlio_map_node)
    # ld.add_action(imu_tf)
    # ld.add_action(laser_tf)

    ld.add_action(rviz_node)

