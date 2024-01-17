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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # current_pkg = FindPackageShare('direct_lidar_inertial_odometry')
    current_pkg = get_package_share_directory('direct_lidar_inertial_odometry')
    print(current_pkg)

    # Set default arguments
    rviz = LaunchConfiguration('rviz', default='false')
    # pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/points_raw')
    # imu_topic = LaunchConfiguration('imu_topic', default='/imu_data')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/livox/lidar')
    imu_topic = LaunchConfiguration('imu_topic', default='/livox/imu')    

    # Define arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=rviz,
        description='Launch RViz'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name' 
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )

    # Load parameters
    # dlio_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'dlio.yaml'])
    # dlio_params_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'params.yaml'])
    
    dlio_yaml_path = launch.substitutions.LaunchConfiguration(
        'dlio_yaml_path',
        default=os.path.join(
            current_pkg,
            'cfg',
            'dlio.yaml'))
    print(dlio_yaml_path)
    dlio_params_yaml_path = launch.substitutions.LaunchConfiguration(
        'dlio_params_yaml_path',
        default=os.path.join(
            current_pkg,
            'cfg',
            'params.yaml'))
    print(dlio_params_yaml_path)



    # DLIO Odometry Node
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('imu', imu_topic),
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
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
        ]
    )

    # RViz node
    rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'dlio.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        dlio_odom_node,
        dlio_map_node,
        rviz_node
    ])
