#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math

class SimpleBallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        
      
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
  
        self.angular_speed = 0.5    
        self.linear_speed = 0.2     
        self.target_distance = 1.0 
        
        self.get_logger().info('Ball Follower Started!')

    def scan_callback(self, msg):
        """Find closest object and follow it"""
        ranges = np.array(msg.ranges)
        
       
        valid_ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges) & (ranges > 0.1)]
        
        if len(valid_ranges) == 0:
            self.stop_robot()
            self.get_logger().info('No objects detected')
            return
        
        closest_distance = np.min(valid_ranges)
        closest_index = np.argmin(ranges)
        

        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        closest_angle = angle_min + closest_index * angle_increment
        closest_angle_deg = math.degrees(closest_angle)
        
     
        direction = ""
        if closest_angle > 0.1:      
            direction = "LEFT"
        elif closest_angle < -0.1:    
            direction = "RIGHT"
        else:                        
            direction = "CENTER"
            
        self.get_logger().info(f'Ball at {closest_distance:.2f}m, {closest_angle_deg:.1f}° ({direction})')
        
       
        cmd = Twist()
        
        if closest_angle > 0.1:  
            cmd.angular.z = self.angular_speed
            cmd.linear.x = 0.0
            
        elif closest_angle < -0.1:  
            cmd.angular.z = -self.angular_speed  
            cmd.linear.x = 0.0
            
        else:  
            cmd.angular.z = 0.0
            if closest_distance > self.target_distance:
                cmd.linear.x = self.linear_speed
            else:
                cmd.linear.x = 0.0 
        self.cmd_vel_publisher.publish(cmd)

    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleBallFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()