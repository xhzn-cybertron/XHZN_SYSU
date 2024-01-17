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

    odom_tf = launch_ros.actions.Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','odom','base_link']
        )
    
    laser_tf = launch_ros.actions.Node(
        name='laser_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','laser_link']
        )
    imu_tf = launch_ros.actions.Node(
        name='imu_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','imu']
        )

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('direct_lidar_inertial_odometry_localization'),
            'param',
            'localization.yaml'))
    
    pcl_localization = launch_ros.actions.LifecycleNode(
        name='dlio_loc',
        namespace='',
        package='direct_lidar_inertial_odometry_localization',
        executable='dlio_odom_node',
        remappings=[('/cloud','/c16/lslidar_point_cloud'),('/imu','/data')],
        parameters=[localization_param_dir],
        output='screen')

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization, 
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization, 
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )


    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', "/home/ls/ros2_car_ws/src/dlio_loc_ros2-humble/rviz"],
        output='log'
    )
    
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)
    
    ld.add_action(pcl_localization)
    ld.add_action(imu_tf)
    ld.add_action(laser_tf)

    ld.add_action(to_inactive)
    ld.add_action(rviz_node)

    return ld