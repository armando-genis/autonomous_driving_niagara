# autonomous_driving_niagara
Simulation for an Autonomous golf car for Monterrey Institute of Technology that uses Velodyne LiDAR and a Stereo Camara for environmental perception and an Ackermann steering controller for trajectory tracking.

 ## Set up ROS2
```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
```

# Niagara Model description
```bash
colcon build --packages-select niagara_model
source install/setup.bash
ros2 launch niagara_model display.launch.py #For launching with gazebo and rviz
ros2 launch niagara_model display_gui.launch.py #For only launching rviz
```

# Velodyne Lidar Gazebo Plugin
```bash
colcon build --packages-select velodyne_gazebo_plugins
source install/setup.bash
```

# Ackermann teleop
```bash
colcon build --packages-select ackermann_teleop
source install/setup.bash
ros2 run ackermann_teleop ackermann_drive_keyop
```

# Kill gazebo
```bash
ps aux | grep gazebo
kill -9 22656
```

## Install
```bash
sudo apt install ros-<distro>-ros2-control ros-<distro>-ros2-controllers
sudo apt install ros-<distro>-controller-manager

```

## Authors

- [@armando-genis](https://github.com/armando-genis)