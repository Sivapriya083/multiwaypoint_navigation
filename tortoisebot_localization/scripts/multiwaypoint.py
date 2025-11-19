#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from enum import Enum
import time


class NavigationState(Enum):
    """State machine for navigation control"""
    IDLE = 0
    WAITING_FOR_NAV2 = 1
    NAVIGATING = 2
    AT_WAYPOINT = 3
    RETURNING_HOME = 4
    MISSION_COMPLETE = 5
    FAILED = 6


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Navigation action client
        self._action_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        # State management
        self.state = NavigationState.IDLE
        self.current_waypoint_idx = 0
        self.goal_handle = None
        
        #  waypoints (x, y, orientation_z, orientation_w)
        self.waypoints = [
            
            {'x': 1.5, 'y': 0.0, 'z': 0.0, 'w': 1.0, 'wait': 3.0},
            {'x': 1.5, 'y': 1.5, 'z': 0.707, 'w': 0.707, 'wait': 5.0},
            {'x': 0.0, 'y': 1.5, 'z': 1.0, 'w': 0.0, 'wait': 2.0},
            {'x': -1.5, 'y': 1.5, 'z': -0.707, 'w': 0.707, 'wait': 4.0},
            {'x': -1.5, 'y': 0.5, 'z': 0.0, 'w': 1.0, 'wait': 3.0}
        ]
        
        # Home position (starting position)
        self.home_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        
        # Timing tracking
        self.mission_start_time = None
        self.waypoint_arrival_time = None
        self.waypoint_departure_time = None
        self.waypoint_times = []
        
        # Timer for state machine updates
        self.timer = self.create_timer(0.5, self.state_machine_callback)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Waypoint Navigator with Action Client')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Total waypoints to visit: {len(self.waypoints)}')
        self.get_logger().info('\nNote: This implementation uses Nav2 action client')
        self.get_logger().info('Nav2 lifecycle nodes (bt_navigator, controller_server,')
        self.get_logger().info('planner_server, behavior_server) must be active.')
        
    def wait_for_nav2_active(self):
        """Wait for Nav2 action server to be ready"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Checking Nav2 readiness...')
        self.get_logger().info('='*60)
        
        # The action server being ready indicates that:
        # 1. Nav2 lifecycle nodes (bt_navigator, controller_server, 
        #    planner_server, behavior_server) are running
        # 2. They have been configured and activated
        # 3. The navigation stack is ready to accept goals
        
        self.get_logger().info('\nWaiting for navigate_to_pose action server...')
        self.get_logger().info('(This confirms Nav2 lifecycle nodes are active)')
        
        retry_count = 0
        max_retries = 60  # 60 seconds timeout
        
        while retry_count < max_retries:
            if self._action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('\n' + '='*60)
                self.get_logger().info('✓ Nav2 Action Server is Ready!')
                self.get_logger().info('='*60)
                self.get_logger().info('This means:')
                self.get_logger().info('  ✓ bt_navigator lifecycle node is active')
                self.get_logger().info('  ✓ controller_server lifecycle node is active')
                self.get_logger().info('  ✓ planner_server lifecycle node is active')
                self.get_logger().info('  ✓ behavior_server lifecycle node is active')
                self.get_logger().info('  ✓ Navigation stack is operational')
                self.get_logger().info('='*60)
                return True
            
            retry_count += 1
            if retry_count % 10 == 0:
                self.get_logger().info(f'Still waiting... ({retry_count}s elapsed)')
        
        self.get_logger().error('\n' + '='*60)
        self.get_logger().error('✗ Action server not available!')
        self.get_logger().error('='*60)
        self.get_logger().error('Please ensure:')
        self.get_logger().error('  1. Nav2 is launched:')
        self.get_logger().error('     ros2 launch <your_pkg> navigation.launch.py')
        self.get_logger().error('  2. Robot is localized (set 2D Pose Estimate in RViz)')
        self.get_logger().error('  3. All lifecycle nodes are active')
        self.get_logger().error('='*60)
        return False
    
    def create_pose_stamped(self, x, y, z, w):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        
        return pose
    
    def send_goal(self, x, y, z, w):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(x, y, z, w)
        
        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}')
        self.waypoint_departure_time = time.time()
        
        # Send goal asynchronously
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.state = NavigationState.FAILED
            return
        
        self.get_logger().info('Goal accepted, navigating...')
        
        # Get result
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Process navigation feedback"""
        feedback = feedback_msg.feedback
        # Log distance remaining periodically
        if hasattr(feedback, 'distance_remaining') and feedback.distance_remaining > 0.1:
            # Uncomment to see progress:
            # self.get_logger().info(f'  Distance remaining: {feedback.distance_remaining:.2f}m')
            pass
    
    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('✓ Navigation succeeded!')
            self.waypoint_arrival_time = time.time()
            
            if self.state == NavigationState.NAVIGATING:
                self.state = NavigationState.AT_WAYPOINT
            elif self.state == NavigationState.RETURNING_HOME:
                self.state = NavigationState.MISSION_COMPLETE
        else:
            self.get_logger().error(f'✗ Navigation failed with status: {status}')
            self.state = NavigationState.FAILED
    
    def state_machine_callback(self):
        """Main state machine logic"""
        
        if self.state == NavigationState.IDLE:
            self.get_logger().info('\n' + '='*60)
            self.get_logger().info('🚀 STARTING WAYPOINT MISSION')
            self.get_logger().info('='*60)
            self.state = NavigationState.WAITING_FOR_NAV2
            
        elif self.state == NavigationState.WAITING_FOR_NAV2:
            # Wait for Nav2 to be ready
            # The action server being available confirms that all Nav2
            # lifecycle nodes are in the 'active' state
            if self.wait_for_nav2_active():
                self.get_logger().info('\n✓ Starting mission...\n')
                self.mission_start_time = time.time()
                self.current_waypoint_idx = 0
                self.state = NavigationState.NAVIGATING
                
                # Send first waypoint
                wp = self.waypoints[self.current_waypoint_idx]
                self.send_goal(wp['x'], wp['y'], wp['z'], wp['w'])
            else:
                self.state = NavigationState.FAILED
            
        elif self.state == NavigationState.AT_WAYPOINT:
            if self.waypoint_arrival_time:
                wp = self.waypoints[self.current_waypoint_idx]
                wait_time = wp['wait']
                
                # Calculate navigation time
                nav_time = self.waypoint_arrival_time - self.waypoint_departure_time
                
                self.get_logger().info(
                    f'\n📍 Arrived at waypoint {self.current_waypoint_idx + 1}/'
                    f'{len(self.waypoints)}'
                )
                self.get_logger().info(f'   Navigation time: {nav_time:.2f}s')
                self.get_logger().info(f'   ⏸️  Pausing for {wait_time}s at pit stop...')
                
                # Wait at waypoint
                time.sleep(wait_time)
                
                # Log waypoint completion
                self.waypoint_times.append({
                    'waypoint': self.current_waypoint_idx + 1,
                    'nav_time': nav_time,
                    'wait_time': wait_time
                })
                
                self.get_logger().info(f'   🚀 Departing from waypoint {self.current_waypoint_idx + 1}\n')
                
                # Move to next waypoint
                self.current_waypoint_idx += 1
                
                if self.current_waypoint_idx < len(self.waypoints):
                    # Navigate to next waypoint
                    self.state = NavigationState.NAVIGATING
                    wp = self.waypoints[self.current_waypoint_idx]
                    self.send_goal(wp['x'], wp['y'], wp['z'], wp['w'])
                else:
                    # All waypoints visited, return home
                    self.get_logger().info('='*60)
                    self.get_logger().info('✓ All waypoints visited!')
                    self.get_logger().info('🏠 Returning to home position...')
                    self.get_logger().info('='*60 + '\n')
                    self.state = NavigationState.RETURNING_HOME
                    home = self.home_position
                    self.send_goal(home['x'], home['y'], home['z'], home['w'])
                    
        elif self.state == NavigationState.MISSION_COMPLETE:
            # Calculate total mission time
            total_time = time.time() - self.mission_start_time
            
            self.get_logger().info('\n' + '='*60)
            self.get_logger().info('🎉 MISSION COMPLETE!')
            self.get_logger().info('='*60)
            
            # Print waypoint summary
            self.get_logger().info('\n📊 WAYPOINT SUMMARY:')
            self.get_logger().info('-'*60)
            total_wait_time = 0
            total_nav_time = 0
            
            for wpt in self.waypoint_times:
                self.get_logger().info(
                    f"  Waypoint {wpt['waypoint']}: "
                    f"Navigation = {wpt['nav_time']:5.2f}s | "
                    f"Wait = {wpt['wait_time']:5.2f}s"
                )
                total_wait_time += wpt['wait_time']
                total_nav_time += wpt['nav_time']
            
            return_nav_time = self.waypoint_arrival_time - self.waypoint_departure_time
            total_nav_time += return_nav_time
            
            self.get_logger().info('-'*60)
            self.get_logger().info(f"  Return Home: Navigation = {return_nav_time:5.2f}s")
            
            self.get_logger().info('\n' + '='*60)
            self.get_logger().info('⏱️  TIMING BREAKDOWN:')
            self.get_logger().info('='*60)
            self.get_logger().info(f'  Total Navigation Time: {total_nav_time:6.2f}s')
            self.get_logger().info(f'  Total Wait Time:       {total_wait_time:6.2f}s')
            self.get_logger().info(f'  TOTAL MISSION TIME:    {total_time:6.2f}s')
            self.get_logger().info('='*60 + '\n')
            
            # Shutdown node
            self.timer.cancel()
            rclpy.shutdown()
            
        elif self.state == NavigationState.FAILED:
            self.get_logger().error('\n' + '='*60)
            self.get_logger().error('❌ MISSION FAILED!')
            self.get_logger().error('='*60)
            self.get_logger().error('Possible reasons:')
            self.get_logger().error('  1. Nav2 not launched')
            self.get_logger().error('  2. Robot not localized')
            self.get_logger().error('  3. Navigation goal unreachable')
            self.get_logger().error('  4. Collision detected')
            self.get_logger().error('='*60)
            self.timer.cancel()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('\n⚠️  Mission interrupted by user')
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()