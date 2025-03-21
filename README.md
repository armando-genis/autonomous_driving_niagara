# autonomous_driving_niagara

** Update Version:**

A new update version has been added where you can modify the scale of the robot along with other improvements. For more details, check out the updated features on [DPU-NaviCar](https://github.com/armando-genis/DPU-NaviCar).

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

# Ackermann keyboard teleop
```bash
colcon build --packages-select ackermann_teleop
source install/setup.bash
ros2 run ackermann_teleop ackermann_drive_keyop
```

# liom sam
```bash
ros2 launch lio_sam run.launch.py
```

# Waypoints creator
```bash
colcon build --packages-select waypoints_niagara_creator
source install/setup.bash
ros2 launch waypoints_niagara_creator waypoints.launch.py
```
# Waypoints loader
```bash
colcon build --packages-select waypoints_niagara_loader
source install/setup.bash
ros2 launch waypoints_niagara_loader waypointsLoader.launch.py
```

# Waypoints calculations
```bash
colcon build --packages-select waypoints_calculations
source install/setup.bash
ros2 launch waypoints_calculations calculation.launch.py
```

# Stanley Controller 
```bash
colcon build --packages-select stanley_control_niagara
source install/setup.bash
ros2 launch stanley_control_niagara stanleyControl.launch.py
```

# City simualtion
You need to include the path to the directory containing the models directory in the GAZEBO_MODEL_PATH environment variable. To make this change permanent, you can add the above line to your ~/.bashrc file. 
```bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/ros2_ws/src/autonomous_driving_niagara/city_simulation
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/ros2_ws/src/autonomous_driving_niagara/city_simulation" >> ~/.bashrc
source ~/.bashrc
```

# Kill gazebo
```bash
ps aux | grep gazebo
kill -9 22656
```



# Lio sam Instalation
```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
sudo apt install libgtsam-dev libgtsam-unstable-dev
sudo snap install cloudcompare
```

## Install For simulation
```bash
sudo apt-get install libeigen3-dev
sudo apt install ros-<ros2-distro>-joint-state-publisher-gui
sudo apt install ros-<ros2-distro>-xacro
sudo apt install ros-<ros2-distro>-gazebo-ros-pkgs
sudo apt install ros-<ros2-distro>-ackermann-msgs
sudo apt install ros-<distro>-ros2-control ros-<distro>-ros2-controllers
sudo apt install ros-<distro>-controller-manager
```

## Authors

- [@armando-genis](https://github.com/armando-genis)
