# Requirement
- Ubuntu 22.04
- ROS2 HUMBLE

# Publish urdf 

This launch file can show Robotmodel on Rviz2.

```
ros2 launch mecabot_bringup mecabot_state_publisher.launch.py
```

# Simulation with Gazebo

The robot can simulate lidar, imu, movement but it still have problem 
1. ความเร็วในการเคลื่อนที่ไม่ตรงกับ /cmd_vel แต่ถ้าปรับมวลลดลงจะทำให้สามารถเคลื่อนที่ใกล้กับค่าจริงมากขึ้น.
2. Odom data after fuse sensor is invalid.

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

if you want to view tree you need to install from source.

```
git clone https://github.com/splintered-reality/py_trees_ros_viewer
```
***Note**: it may error when you build. You should install dependency that show in error.

And use this cmd to view tree.

```
py-trees-tree-viewer
```

![Screenshot from 2024-07-03 09-32-07](https://github.com/SuaPiamsuk/mecabot/assets/56068294/d8fb7041-27bf-492f-a5dc-98e4115d3aab)



## RPLIDAR with ROS2 HUMBLE
Follow step from https://github.com/babakhani/rplidar_ros2. or use below command.

```
git clone https://github.com/babakhani/rplidar_ros2.git
```
