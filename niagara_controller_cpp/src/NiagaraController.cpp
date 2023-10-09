#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std;
class NiagaraController : public rclcpp::Node
{
private:

    // Function declarations
    void commandCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_sub_;

public:
    NiagaraController(/* args */);
    ~NiagaraController();
};

NiagaraController::NiagaraController(/* args */): Node("NiagaraController_node")
{
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("velocity_controller/commands", 10);
    position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("position_controller/commands", 10);
    command_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "ackermann_cmd", 10, std::bind(&NiagaraController::commandCallback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "NiagaraController_node initialized");
}

NiagaraController::~NiagaraController()
{
}

void NiagaraController::commandCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    auto vel_command = std::make_shared<std_msgs::msg::Float64MultiArray>();
    vel_command->data = {msg->drive.speed, msg->drive.speed}; // Same speed for left and right rear axles

    auto pos_command = std::make_shared<std_msgs::msg::Float64MultiArray>();
    pos_command->data = {msg->drive.steering_angle, msg->drive.steering_angle}; // Same angle for left and right steering joints

    velocity_pub_->publish(*vel_command);
    position_pub_->publish(*pos_command);
}



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NiagaraController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}