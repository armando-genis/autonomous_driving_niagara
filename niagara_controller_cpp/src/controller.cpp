#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("velocity_command_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("velocity_controller/commands", 10);

    std_msgs::msg::Float64MultiArray velocity_command;
    velocity_command.data = {1.0, 1.0, 1.0, 1.0};  // Replace with your desired velocities

    rclcpp::WallRate loop_rate(10);  // 10 Hz
    while (rclcpp::ok())
    {
        publisher->publish(velocity_command);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
