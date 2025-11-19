#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import time


def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose (ONLY ONCE, where your robot starts in RViz!)
    initial_pose = create_pose_stamped(nav, 6.0, 1.0, 0.0)
    nav.setInitialPose(initial_pose)

    # --- wait for Nav2 to activate
    nav.waitUntilNav2Active()

    # --- Define waypoints
    waypoints = [
        create_pose_stamped(nav, 4.0, 0.0, 3.14),
        create_pose_stamped(nav, 3.0, -2.0, -1.57),
        create_pose_stamped(nav, 0.0, 2.0, 1.57),
        create_pose_stamped(nav, -2.0, -2.0, -1.57),
        create_pose_stamped(nav, -2.0, 2.0, 1.57),
        create_pose_stamped(nav, -4.0, -1.0, 3.14),
        create_pose_stamped(nav, 6.0, 1.0, -0.25)
    ]


    wait_time = [3,5,2,3,5,2,4]

    # --- Patrol loop
    try:
        while True:
            print("Starting patrol sequence...")
            for wp in waypoints:
                print(f"Going to waypoint: ({wp.pose.position.x}, {wp.pose.position.y})")
                nav.goToPose(wp)

                while not nav.isTaskComplete():
                    feedback = nav.getFeedback()
                    if feedback:
                        print("Distance remaining:", feedback.distance_remaining)

                print("Reached waypoint:", nav.getResult())

            print(f"waiting {wait_time} seconds to reach the next point")
            time.sleep(wait_time)  

    except KeyboardInterrupt:
        print("Patrol stopped by user.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()