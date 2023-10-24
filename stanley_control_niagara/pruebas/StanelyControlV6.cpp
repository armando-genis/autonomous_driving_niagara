#include <rclcpp/rclcpp.hpp>

// ROS
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

// msg
#include <std_msgs/msg/float32.hpp> 
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>



// C++
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>

using namespace std;


class StanleyControl : public rclcpp::Node
{
private:
    // Variables of the car
    double current_yaw_ = 0.0; // [rad]
    double current_x_ = 0.0; // [m]
    double current_y_ = 0.0; // [m]
    double current_velocity_ = 0; // [m/s]

    // Variables of the path
    double path_x_ = 0.0; // [m]
    double path_y_ = 0.0; // [m]
    double path_yaw_ = 0.0; // [rad]
    double path_velocity_ = 0.0; // [m/s]

    // Variables of the Stanley Control
    double wheelbase = 2.789;
    double max_steer = 0.0; // [rad]
    double L = 0.40; // [m] Wheel base of vehicle
    double error_front_axle = 0.0; // [m]
    double K = 0.5; // Gain of the Stanley Control
    double stering_angle = 0.0; // [rad]

    // Function of the Stanley Control
    void computeCrossTrackError();
    void computeSteeringAngle();
    double GetNormaliceAngle(double angle);

    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pub_callback();
    void targetWaypointCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
    void speedCallback(const std_msgs::msg::Float32::SharedPtr msg);


    // Subcribers, timers & publishers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr target_waypoint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;  
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

public:
    StanleyControl(/* args */);
    ~StanleyControl();
};

StanleyControl::StanleyControl(/* args */) : Node("stanley_control_niagara_node")
{

    this->declare_parameter("wheelbase", double(0.0));
    this->declare_parameter("max_steer", double(0.0));

    this->get_parameter("wheelbase", wheelbase);
    this->get_parameter("max_steer", max_steer);

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&StanleyControl::odom_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&StanleyControl::pub_callback, this));

    target_waypoint_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "target_waypoint_marker", 10,
        std::bind(&StanleyControl::targetWaypointCallback, this, std::placeholders::_1)
    );
    
    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/pursuitspeedtarget", 10,
        std::bind(&StanleyControl::speedCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);


    RCLCPP_INFO(this->get_logger(), "stanley_control_niagara_node initialized");

}

StanleyControl::~StanleyControl()
{
}

// TIMER CALLBACK
void StanleyControl::pub_callback()
{
    computeSteeringAngle();
    auto msg = geometry_msgs::msg::Twist();

    // Set the linear and angular velocities
    msg.linear.x = path_velocity_;  
    msg.angular.z = stering_angle;  
    // print path_velocity_ and stering_angle
    RCLCPP_INFO(this->get_logger(), "path_velocity_: %f", path_velocity_);
    RCLCPP_INFO(this->get_logger(), "stering_angle: %f", stering_angle);
    // Publish the message
    cmd_vel_pub_->publish(msg);
}

// ODOM CALLBACK    
void StanleyControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_velocity_ = msg->twist.twist.linear.x;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
}


void StanleyControl::targetWaypointCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
    path_x_ = msg->pose.position.x;
    path_y_ = msg->pose.position.y;
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    path_yaw_ = yaw;
}

void StanleyControl::speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    path_velocity_ = msg->data;
}

double StanleyControl::GetNormaliceAngle(double angle){
    if (angle > M_PI) angle -= 2 * M_PI;
    if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void StanleyControl::computeCrossTrackError(){
    double current_yaw_stanley = GetNormaliceAngle(current_yaw_);
    double fx = current_x_ + L * cos(current_yaw_stanley);
    double fy = current_y_ + L * sin(current_yaw_stanley);

    // Compute cross track error
    double front_axle_vec[2] = {-std::cos(current_yaw_stanley + M_PI / 2), -std::sin(current_yaw_stanley + M_PI / 2)};
    error_front_axle = (fx - path_x_) * front_axle_vec[0] + (fy - path_y_) * front_axle_vec[1];
}

void StanleyControl::computeSteeringAngle(){
    computeCrossTrackError();
    double current_yaw_car = GetNormaliceAngle(current_yaw_);
    double yaw_path = GetNormaliceAngle(path_yaw_);
    double theta_e = GetNormaliceAngle(yaw_path - current_yaw_car);
    double theta_d = atan2(K * error_front_axle, current_velocity_);
    double delta = theta_e + theta_d;
    stering_angle = GetNormaliceAngle(delta);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StanleyControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
