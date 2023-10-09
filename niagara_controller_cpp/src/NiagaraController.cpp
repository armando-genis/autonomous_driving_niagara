#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std;

#define PI 3.1415926535
#define WHEEL_DISTANCE 2.5     // Change this value based on your robot's configuration
#define WHEEL_DIAMETER 0.470   // Change this value based on your robot's wheel size

class NiagaraController : public rclcpp::Node
{
private:

    // Initialize robot pose
    double robot_pose_px_ = 0.0;
    double robot_pose_py_ = 0.0;
    double robot_pose_pa_ = 0.0;
    double linearSpeedXMps_ = 0.0;

    // Function declarations
    void commandCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void UpdateOdometry();
    void PublishOdometry();

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    tf2_ros::TransformBroadcaster *tf_broadcaster_;

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
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    RCLCPP_INFO(this->get_logger(), "NiagaraController_node initialized");
}

NiagaraController::~NiagaraController()
{
    delete tf_broadcaster_;
}

void NiagaraController::commandCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    auto vel_command = std::make_shared<std_msgs::msg::Float64MultiArray>();
    vel_command->data = {msg->drive.speed, msg->drive.speed, msg->drive.speed, msg->drive.speed}; // Same speed for left and right rear axles

    auto pos_command = std::make_shared<std_msgs::msg::Float64MultiArray>();
    pos_command->data = {msg->drive.steering_angle, msg->drive.steering_angle}; // Same angle for left and right steering joints

    velocity_pub_->publish(*vel_command);
    position_pub_->publish(*pos_command);

    linearSpeedXMps_ = msg->drive.speed;
    UpdateOdometry();
    PublishOdometry();
}

void NiagaraController::UpdateOdometry() {
    double dt = 1.0/100.0;  // Assuming 100Hz update rate, change if different

    // Basic kinematic model
    robot_pose_px_ += linearSpeedXMps_ * cos(robot_pose_pa_) * dt;
    robot_pose_py_ += linearSpeedXMps_ * sin(robot_pose_pa_) * dt;
    robot_pose_pa_ += linearSpeedXMps_ * tan(linearSpeedXMps_) / WHEEL_DISTANCE * dt;

    // Normalize the orientation between [-PI, PI]
    while (robot_pose_pa_ > PI) robot_pose_pa_ -= 2.0 * PI;
    while (robot_pose_pa_ <= -PI) robot_pose_pa_ += 2.0 * PI;
}

void NiagaraController::PublishOdometry() {
    auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();

    odom_msg->header.stamp = this->now();
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";

    // Set position
    odom_msg->pose.pose.position.x = robot_pose_px_;
    odom_msg->pose.pose.position.y = robot_pose_py_;
    odom_msg->pose.pose.orientation.z = sin(robot_pose_pa_/2.0);
    odom_msg->pose.pose.orientation.w = cos(robot_pose_pa_/2.0);

    // Set velocity (in this case, only linear velocity is considered)
    odom_msg->twist.twist.linear.x = linearSpeedXMps_;

    odom_pub_->publish(*odom_msg);

    // Also publish the transform if required (tf between "odom" and "base_link")
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header = odom_msg->header;
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = robot_pose_px_;
    odom_trans.transform.translation.y = robot_pose_py_;
    odom_trans.transform.rotation = odom_msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(odom_trans);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NiagaraController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}