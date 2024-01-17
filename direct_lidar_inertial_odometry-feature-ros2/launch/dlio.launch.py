#
#   Copyright (c)     
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition   
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    current_pkg = get_package_share_directory("direct_lidar_inertial_odometry")

    # DLIO Odometry Node
    dlio_odom_node = Node(
        name='dlio',
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=
        [
          os.path.join(current_pkg,"cfg","dlio.yaml"), 
          os.path.join(current_pkg,"cfg","params.yaml")
        ],
        remappings=[
            ('pointcloud', "/livox/lidar"),
            ('imu', "/livox/imu"),
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
        name='mapping',
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=
        [
          os.path.join(current_pkg,"cfg","dlio.yaml"), 
          os.path.join(current_pkg,"cfg","params.yaml")
        ],
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', os.path.join(current_pkg,"launch","dlio.rviz")],
        output='screen'
    )

    return LaunchDescription([
        dlio_odom_node,
        dlio_map_node,
        rviz_node
    ])
