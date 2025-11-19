#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
   
    tortoisebot_gazebo_dir = get_package_share_directory('tortoisebot_gazebo')
    tortoisebot_description_dir = get_package_share_directory('tortoisebot_description')

  
    world_file = os.path.join(tortoisebot_gazebo_dir, 'worlds', 'custom_obstacles.world')
    bridge_config = os.path.join(get_package_share_directory('tortoisebot_gazebo'),
        'config',
        'ros_gz_bridge.yaml'
    )
    
    octomap_config = os.path.join(get_package_share_directory('tortoisebot_gazebo'),
    
        'config',
        'octomap_server.yaml'
    )

    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-v', '4'],
        output='screen'
    )

   
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('tortoisebot_description'),
            'urdf',
            'tortoisebot.urdf.xacro'
        ])
    ])
    
    
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[octomap_config],
         remappings=[
            ('cloud_in', '/fused_pointcloud') 
        ]
      
    )
    

  
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

   
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'tortoisebot',
                    '-topic', '/robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1',
                ],
                output='screen'
            )
        ]
    )
    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_publisher',
        arguments=[
            '0.15', '0', '0.32',  # x, y, z position (matches your camera location)
            '0', '0', '0',         # roll, pitch, yaw (no rotation)
            'base_link',           # parent frame
            'tortoisebot/base_footprint/depth_camera'  # child frame (Gazebo's name)
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    sensor_fusion = Node(
        package='tortoisebot_gazebo',
        executable='sensor_fusion_node.py',
        name='sensor_fusion_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
     
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'tortoisebot/base_footprint/lidar_sensor'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        camera_transform,
        static_transform, 
        
        
         TimerAction(
            period=5.0,
            actions=[
                sensor_fusion,      
                octomap_server,
  
               
                
            ])
        
    ])