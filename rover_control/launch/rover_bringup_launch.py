#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

from launch.substitutions import EnvironmentVariable
import os

def generate_launch_description():

    ####################################################################
    # micro-ROS Agent
    ####################################################################
    microros = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial',
            '--dev', '/dev/ttyUSB0',
            '--baudrate', '115200',
            '-v4'
        ],
        output='screen'
    )

    ####################################################################
    # Intel RealSense D435
    ####################################################################
    realsense = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            'realsense2_camera',
            'rs_launch.py',

            'enable_color:=true',
            'enable_depth:=true',
            'align_depth.enable:=true',

            'rgb_camera.color_profile:=640x480x30',
            'depth_module.depth_profile:=640x480x30',

            'pointcloud.enable:=true'
        ],
        output='screen'
    )

    ####################################################################
    # ORB_SLAM3
    ####################################################################
    orbslam = ExecuteProcess(
        cmd=[
            'ros2', 'run',
            'orbslam3',
            'rgbd',

            '/home/$USER/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            '/home/$USER/orb_ws/config/d435_rgbd.yaml'
        ],
        shell=True,
        output='screen'
    )

    ####################################################################
    # Map Builder
    ####################################################################
    map_builder = Node(
        package='map_builder',
        executable='map_builder',
        output='screen'
    )

    ####################################################################
    # RViz
    ####################################################################
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.expanduser(
                '~/orb_ws/src/map_builder/rviz/map_builder.rviz'
            )
        ],
        output='screen'
    )

    ####################################################################
    # Rover Controller
    ####################################################################
    rover = Node(
        package='rover_control',
        executable='rover_controller',
        output='screen'
    )

    ####################################################################
    # Rosbag Recorder (MCAP)
    ####################################################################
    rosbag = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            '''
            ros2 bag record \
            --storage mcap \
            -o /home/$USER/rosbags/$(date +%Y%m%d_%H%M%S) \
            /camera/camera/aligned_depth_to_color/image_raw \
            /camera/camera/color/image_raw \
            /camera/camera/depth/color/points \
            /global_cloud \
            /goal_pose \
            /initialpose \
            /orb_pose \
            /robot_marker \
            /rover_odo \
            /rover_sensor \
            /rover_twist \
            /tf \
            /tf_static
            '''
        ],
        output='screen'
    )

    return LaunchDescription([
        microros,

        realsense,
        
        TimerAction(
            period=5.0,
            actions=[orbslam]
        ),
        TimerAction(
            period=10.0,
            actions=[map_builder]
        ),

        TimerAction(
            period=12.0,
            actions=[rviz]
        ),

        TimerAction(
            period=14.0,
            actions=[rosbag]
        ),

        TimerAction(
            period=16.0,
            actions=[rover]
        ),
    ])
