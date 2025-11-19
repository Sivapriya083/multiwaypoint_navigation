#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # Bridge for cmd_vel (ROS2 -> Gazebo)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            output='screen'
        ),
        
        # Bridge for odom (Gazebo -> ROS2)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='odom_bridge',
            arguments=[
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            output='screen'
        ),
        
        # Bridge for scan (Gazebo -> ROS2)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='scan_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
            output='screen'
        ),
        
        # Bridge for joint_states (Gazebo -> ROS2)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='joint_states_bridge',
            arguments=[
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            output='screen'
        ),
        
        # Bridge for TF
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='tf_bridge',
            arguments=[
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            output='screen'
        ),
        
        # Bridge for depth camera point cloud
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_points_bridge',
            arguments=[
                '/camera/depth/color/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            output='screen'
        ),
        
        # Bridge for depth image
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_image_bridge',
            arguments=[
                '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            output='screen'
        ),
    ])