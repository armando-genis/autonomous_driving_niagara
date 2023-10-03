#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher()
        : Node("trajectory_publisher")
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "joint_trajectory_controller/joint_trajectory", 10);
        send_trajectory();
    }

    void send_trajectory()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.joint_names = {"right_steering_joint", "left_steering_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {1.0, 1.0};  // Replace with desired positions
        point.time_from_start.sec = 1;
        message.points.push_back(point);

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published joint trajectory");
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
