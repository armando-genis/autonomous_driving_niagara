#include <rclcpp/rclcpp.hpp>

// ROS
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

// C++
#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

class StanleyControl : public rclcpp::Node
{
private:
    /* data */
    double wheelbase = 0.0; // [m]
    double max_steer = 0.0; // [rad]

    vector<Eigen::VectorXd> waypoints;

    void pub_callback();
    void waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_sub_;

    // Callback function to handle incoming messages
    

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

    waypoints_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "waypoints_loaded", 10,
        std::bind(&StanleyControl::waypoints_callback, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&StanleyControl::pub_callback, this));
    RCLCPP_INFO(this->get_logger(), "stanley_control_niagara_node initialized");
}

StanleyControl::~StanleyControl()
{
}

void StanleyControl::pub_callback()
{
    RCLCPP_INFO(this->get_logger(), "stanley_control_niagara_node callback");
    // print parameters
    // RCLCPP_INFO(this->get_logger(), "wheelbase: %f", wheelbase);
    // RCLCPP_INFO(this->get_logger(), "max_steer: %f", max_steer);
    // print fist point of the waypoints
    RCLCPP_INFO(this->get_logger(), "waypoints[0]: %f, %f, %f, %f" , waypoints[0](0), waypoints[0](1), waypoints[0](2), waypoints[0](3));
}

void StanleyControl::waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    waypoints.clear();

    for (const auto& marker : msg->markers) {
        Eigen::VectorXd waypoint(4);
        waypoint(0) = marker.pose.position.x;
        waypoint(1) = marker.pose.position.y;
        waypoint(2) = marker.pose.position.z;

        tf2::Quaternion q(
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w);

        // Extract yaw from the quaternion
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        waypoint(3) = yaw;

        waypoints.push_back(waypoint);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StanleyControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}