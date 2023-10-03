# autonomous_driving_niagara
 autonomous golf car for Monterrey Institute of Technology Mexico city
 
 colcon build --packages-select niagara_model
 
 sudo apt update
sudo apt install ros-humble-controller-manager
sudo apt update
sudo apt install ros-<distro>-ros2-control ros-<distro>-ros2-controllers
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active joint_trajectory_controller

