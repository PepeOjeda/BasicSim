# 1.2.0
Changed the odometry behaviour to be consistent with the ROS standard. The origin of the odom frame is now at the initial position of the robot, and all odometry information is relative to that frame.

# 1.1.1
Fixed the laser scanner readings for rays that leave the occupancy map.

## 1.1.0
Added robot radius to parameters that can be specified though the YAML file. It is used to detect collisions with the environment.

## 1.0.0
It works!