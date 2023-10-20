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
    size_t closest_waypoint = 0;
    bool waypoints_in_loop = false;

    vector<Eigen::VectorXd> waypoints; // [x, y, z, yaw, velocity]
    vector<Eigen::VectorXd> waypointsCubeLines;
    vector<float> velocities;

    nav_msgs::msg::Odometry current_pose_;
    geometry_msgs::msg::Pose current_pose_from_odom_;

    // Computations functions
    void pub_callback();
    double GetNormaliceAngle(double angle);
    void waypointsComputation();

    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void waypointCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void velocities_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // utils functions
    double getDistance(Eigen::VectorXd point1, Eigen::VectorXd point2);

    // Subcribers, timers & publishers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocities_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;


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

    velocities_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "waypointarray_speeds", 10,
        std::bind(&StanleyControl::velocities_callback, this, std::placeholders::_1)
    );

    waypoint_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        "waypoint_loader_cubelines", 10,
        std::bind(&StanleyControl::waypointCallback, this, std::placeholders::_1)
    );

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&StanleyControl::odom_callback, this, std::placeholders::_1));

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
    // RCLCPP_INFO(this->get_logger(), "waypoints[0]: %f, %f, %f, %f, %f" , waypoints[0](0), waypoints[0](1), waypoints[0](2), waypoints[0](3), velocities[0]);
    // sent the size of waypoitns: 
    // RCLCPP_INFO(this->get_logger(), "waypoints.size(): %d", waypoints.size());
    waypointsComputation();
}

void StanleyControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = *msg;
  current_pose_from_odom_ = current_pose_.pose.pose;
}

void StanleyControl::velocities_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{

    // for (size_t i = 0; i < msg->data.size(); ++i) {

    //     if (waypoints[i].size() < 5) {

    //         waypoints[i].conservativeResize(5);
    //     }
    //     waypoints[i](4) = msg->data[i];
    // }

    velocities.clear();
    velocities.insert(velocities.end(), msg->data.begin(), msg->data.end());

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

double StanleyControl::getDistance(Eigen::VectorXd point1, Eigen::VectorXd point2){
    double distance = sqrt(pow(point1(0) - point2(0), 2) + pow(point1(1) - point2(1), 2));
    return distance;
}

void StanleyControl::waypointsComputation(){
    size_t target_waypoint = 0;                 
    size_t first_wp = 0;                        
    size_t last_wp = int(waypoints.size() - 1); 

    // loop closure is true, when the first and last waypoint are closer than 4.0 meters
    double distance = getDistance(waypoints[first_wp], waypoints[last_wp]);
    waypoints_in_loop = distance < 4.0;

    // find the closest waypoint

}

double StanleyControl::GetNormaliceAngle(double angle){
    if (angle > M_PI) angle -= 2 * M_PI;
    if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StanleyControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}