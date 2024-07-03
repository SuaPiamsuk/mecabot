# Publish urdf 

This launch file can show Robotmodel on Rviz2.

```
ros2 launch mecabot_bringup mecabot_state_publisher.launch.py
```

# Simulation with Gazebo

The robot can simulate lidar, imu, movement but it still have problem 
1. Movement is not real value.
2. Odom data after sensor fusion is invalid.

You can try by this command.
```
ros2 launch mecabot_description spawn.launch.py
```

This pkg reference by
- **create3_sim** project from this link https://github.com/iRobotEducation/create3_sim.

# Behavior tree test

The behavior tree is not already to test with **mecabot** simulation because my pkg don't have world environment but you can test with **Turtlebot3** simulation [https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation]

```
ros2 run mecabot_behavior_tree main_root.py
```
You should to edit **map_path** and **goal** in **main_root.py** before test behavior tree.

![main_behavior](https://github.com/SuaPiamsuk/mecabot/assets/56068294/5c7eea36-9f07-405d-a519-8dabaab36440)

if you want to edit action node, you can edit in **behaviours.py**


## RPLIDAR with ROS2 HUMBLE
Follow step from https://github.com/babakhani/rplidar_ros2. or use below command.

```
git clone https://github.com/babakhani/rplidar_ros2.git
```
