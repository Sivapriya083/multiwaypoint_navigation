# multiwaypoint_navigation
Implementing Autonomous Navigation Using Nav2 NavigationToPose action . The robot navigate through 5 predefined waypoints.


# Launch the robot in Gazebo using the command
```ros2 launch tortoisebot_gazebo custom_world.py```

# Terminal 2 : Open rviz2
```rviz2 --ros-args -p use_sim_time:=true```

# Terminal 3 : Run slam 
```ros2 launch slam_toolbox online_async_launch.py```

# Terminal 4 : Run Navigation
```ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true```
