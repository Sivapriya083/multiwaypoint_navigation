#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection     # tool1: laser_geometry
from sensor_msgs_py import point_cloud2        # tool2: sensor_msgs_py


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')  

        self.laser_projector = LaserProjection()  # initialize ROS2 tool
        
        # Storage
        self.latest_scan = None
        self.latest_depth = None

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PointCloud2, '/camera/depth/points', self.depth_callback, 10)
    
        # Publisher
        self.fused_pub = self.create_publisher(PointCloud2, 'fused_pointcloud', 10)

        # Timer: run merge every 0.2 sec
        self.create_timer(0.2, self.merge_callback)

        self.count = 0

    def scan_callback(self, msg):
        self.latest_scan = msg

    def depth_callback(self, msg):
        self.latest_depth = msg  

    def merge_callback(self):

        # Data availability check
        if not self.latest_scan or not self.latest_depth:
            if self.count % 25 == 0:
                has_scan = 'True' if self.latest_scan else 'False'   
                has_depth = 'True' if self.latest_depth else 'False' 
                self.get_logger().info(f'Waiting... LIDAR: {has_scan} Depth: {has_depth}') 
            self.count += 1
            return

        # Convert LiDAR to point cloud
        scan_cloud = self.laser_projector.projectLaser(self.latest_scan)  

        # Extract points from LiDAR
        lidar_points = []
        for p in point_cloud2.read_points(scan_cloud, skip_nans=True):
            lidar_points.append([p[0], p[1], p[2]])

        # Extract points from depth cloud
        depth_points = []
        for p in point_cloud2.read_points(self.latest_depth, skip_nans=True):  
            depth_points.append([p[0], p[1], p[2]])

        # Merge points
        all_points = lidar_points + depth_points

        # Create a fused point cloud
        header = self.latest_depth.header
        header.frame_id = 'base_link' 
        fused_cloud = point_cloud2.create_cloud_xyz32(header, all_points)

        # Publish fused cloud
        self.fused_pub.publish(fused_cloud)

        # Log occasionally
        self.count += 1
        if self.count % 25 == 0:
            self.get_logger().info(
                f' Merged: {len(lidar_points)} LIDAR + '
                f'{len(depth_points)} depth = {len(all_points)} points'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
