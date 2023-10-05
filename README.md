# autonomous_driving_niagara
 autonomous golf car for Monterrey Institute of Technology Mexico city
 
 
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
colcon build --packages-select niagara_model
source install/setup.bash
ros2 launch niagara_model display.launch.py
ros2 launch niagara_model display_gui.launch.py
 
 
 
 
 sudo apt update
sudo apt install ros-humble-controller-manager
sudo apt update
sudo apt install ros-<distro>-ros2-control ros-<distro>-ros2-controllers
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active joint_trajectory_controller

source install/setup.bash
colcon build --packages-select niagara_controller_cpp
ros2 run niagara_controller_cpp publish_trajectory


ps aux | grep gazebo
kill -9 22656

